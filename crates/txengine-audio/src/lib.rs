//! Audio input for txengine.
//!
//! An [`AudioInput`] decodes a source on a background thread, converts it to
//! stereo `f32` at the rate requested by the transmission mode (using
//! rubato's band-limited sinc resampler) and hands frames to the real-time
//! thread through a lock-free ring buffer. The real-time side never blocks:
//! when no audio is available it simply receives fewer frames.

mod decode;
mod source;

use std::sync::atomic::{AtomicBool, AtomicI64, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;

use log::{debug, info, warn};

pub use source::{InputSpec, SourceInfo};

/// Errors raised while opening an input.
#[derive(Debug, thiserror::Error)]
pub enum AudioError {
    #[error("{0}: {1}")]
    Io(String, #[source] std::io::Error),
    #[error("unsupported audio: {0}")]
    Unsupported(String),
    #[error("decoding failed: {0}")]
    Decode(String),
}

/// What to do when the source ends.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EndOfInput {
    /// Seek back to the start (files only; streams behave like `Wait`).
    Repeat,
    /// Keep transmitting silence.
    Wait,
    /// Report the input as finished.
    Stop,
}

/// Options for [`AudioInput::open`].
#[derive(Debug, Clone)]
pub struct AudioOptions {
    pub end: EndOfInput,
    /// Compensate clock drift between a live source and the transmitter by
    /// steering the resampler (±`max_drift_ppm`) to keep the buffer half
    /// full. `None` enables it automatically for stdin.
    pub drift_compensation: Option<bool>,
    pub max_drift_ppm: f64,
    /// The transmitter will correct the playback rate while running
    /// ([`AudioInput::set_rate_correction_ppm`]), so always resample, even
    /// when the source already has the requested rate.
    pub adjustable_rate: bool,
    /// Buffer between the decoder and the transmitter, in seconds.
    pub buffer: Duration,
}

impl Default for AudioOptions {
    fn default() -> Self {
        AudioOptions {
            end: EndOfInput::Repeat,
            drift_compensation: None,
            max_drift_ppm: 500.0,
            adjustable_rate: false,
            buffer: Duration::from_millis(500),
        }
    }
}

/// Counters shared with the decoder thread.
#[derive(Debug, Default)]
struct Shared {
    stop: AtomicBool,
    eof: AtomicBool,
    error: Mutex<Option<String>>,
    frames_decoded: AtomicU64,
    /// Current drift correction in parts per billion.
    drift_ppb: AtomicI64,
    /// Playback rate correction requested by the transmitter (ppm, as f64 bits).
    rate_ppm: AtomicU64,
    loops: AtomicU64,
}

/// Snapshot of the input's state.
#[derive(Debug, Clone, Default)]
pub struct AudioStats {
    pub frames_decoded: u64,
    pub buffered: usize,
    pub capacity: usize,
    pub drift_ppm: f64,
    pub loops: u64,
    pub finished: bool,
    pub error: Option<String>,
}

/// A running audio input.
pub struct AudioInput {
    rx: rtrb::Consumer<[f32; 2]>,
    shared: Arc<Shared>,
    capacity: usize,
    thread: Option<JoinHandle<()>>,
    /// Wait for the decoder instead of returning short (rendering to files).
    blocking: bool,
    pub info: SourceInfo,
    pub rate: f64,
}

impl std::fmt::Debug for AudioInput {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "AudioInput({:?} -> {:.1} Hz)", self.info, self.rate)
    }
}

impl AudioInput {
    /// Open `spec` and start decoding, converting to `rate` Hz stereo.
    ///
    /// The source is probed synchronously so that format errors are reported
    /// here rather than later from the decoder thread.
    pub fn open(spec: &InputSpec, rate: f64, opts: &AudioOptions) -> Result<AudioInput, AudioError> {
        let src = source::open(spec)?;
        let info = src.info();
        let drift = opts.drift_compensation.unwrap_or(info.live);
        info!(
            "audio input: {} ({} Hz, {} channel{}){}",
            info.description,
            info.rate,
            info.channels,
            if info.channels == 1 { "" } else { "s" },
            if drift { ", clock drift compensation on" } else { "" }
        );
        let capacity = ((opts.buffer.as_secs_f64() * rate) as usize).max(4096);
        let (tx, rx) = rtrb::RingBuffer::new(capacity);
        let shared = Arc::new(Shared::default());
        let worker = decode::Worker::new(src, rate, opts, drift, tx, capacity, shared.clone())?;
        let thread = std::thread::Builder::new()
            .name("txengine-audio".into())
            .spawn(move || worker.run())
            .map_err(|e| AudioError::Io("spawn audio thread".into(), e))?;
        Ok(AudioInput { rx, shared, capacity, thread: Some(thread), blocking: false, info, rate })
    }

    /// In blocking mode [`AudioInput::read`] waits until `out` can be filled
    /// or the input has ended. For producers that run faster than real time
    /// (rendering to a file), where a short read would insert silence.
    pub fn set_blocking(&mut self, on: bool) {
        self.blocking = on;
    }

    /// Pop up to `out.len()` frames; never blocks unless blocking mode is on.
    /// Returns the number read.
    #[inline]
    pub fn read(&mut self, out: &mut [[f32; 2]]) -> usize {
        let mut n = self.pop(out);
        while self.blocking && n < out.len() && !self.shared.eof.load(Ordering::Acquire) {
            std::thread::sleep(std::time::Duration::from_micros(200));
            n += self.pop(&mut out[n..]);
        }
        if self.blocking && n < out.len() {
            // Ended: whatever arrived before the end-of-input flag.
            n += self.pop(&mut out[n..]);
        }
        n
    }

    fn pop(&mut self, out: &mut [[f32; 2]]) -> usize {
        let n = self.rx.slots().min(out.len());
        if n == 0 {
            return 0;
        }
        if let Ok(chunk) = self.rx.read_chunk(n) {
            let (a, b) = chunk.as_slices();
            out[..a.len()].copy_from_slice(a);
            out[a.len()..a.len() + b.len()].copy_from_slice(b);
            chunk.commit_all();
        }
        n
    }

    /// The transmitter's audio clock runs `ppm` fast relative to true time
    /// (its crystal error): produce that many more frames per second of
    /// programme, so that the programme plays at its true speed. Live
    /// sources with drift compensation follow their own clock instead; this
    /// then only spares the drift loop the work. Needs
    /// [`AudioOptions::adjustable_rate`] unless the input is resampled anyway.
    pub fn set_rate_correction_ppm(&self, ppm: f64) {
        self.shared.rate_ppm.store(ppm.to_bits(), Ordering::Relaxed);
    }

    /// Frames currently buffered.
    pub fn buffered(&self) -> usize {
        self.rx.slots()
    }

    /// True when the source has ended (and, unless it repeats or waits,
    /// everything buffered has been consumed).
    pub fn finished(&self) -> bool {
        self.shared.eof.load(Ordering::Acquire) && self.rx.is_empty()
    }

    pub fn stats(&self) -> AudioStats {
        AudioStats {
            frames_decoded: self.shared.frames_decoded.load(Ordering::Relaxed),
            buffered: self.rx.slots(),
            capacity: self.capacity,
            drift_ppm: self.shared.drift_ppb.load(Ordering::Relaxed) as f64 / 1000.0,
            loops: self.shared.loops.load(Ordering::Relaxed),
            finished: self.finished(),
            error: self.shared.error.lock().map(|e| e.clone()).unwrap_or(None),
        }
    }
}

impl Drop for AudioInput {
    fn drop(&mut self) {
        self.shared.stop.store(true, Ordering::Release);
        if let Some(t) = self.thread.take() {
            // The thread may be blocked reading stdin; don't wait for it then.
            if t.is_finished() {
                let _ = t.join();
            } else {
                debug!("audio thread still blocked on input; detaching");
            }
        }
        if let Ok(Some(e)) = self.shared.error.lock().map(|e| e.clone()) {
            warn!("audio input error: {e}");
        }
    }
}

/// Decode a whole file into interleaved `f32` samples (for offline tools).
/// Returns `(sample_rate, channels, samples)`.
pub fn read_file(path: &std::path::Path) -> Result<(u32, usize, Vec<f32>), AudioError> {
    let mut src = source::open(&InputSpec::File(path.to_path_buf()))?;
    let info = src.info();
    let mut out = Vec::new();
    while src.read(&mut out)? {}
    Ok((info.rate, info.channels, out))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Amplitude of the left channel's component at `f` Hz.
    fn amplitude(x: &[[f32; 2]], rate: f64, f: f64) -> f64 {
        let (mut s, mut c) = (0.0, 0.0);
        for (i, v) in x.iter().enumerate() {
            let ph = 2.0 * std::f64::consts::PI * f * i as f64 / rate;
            s += v[0] as f64 * ph.sin();
            c += v[0] as f64 * ph.cos();
        }
        2.0 * (s * s + c * c).sqrt() / x.len() as f64
    }

    #[test]
    fn rate_correction_sets_the_playback_speed() {
        let opts = AudioOptions { adjustable_rate: true, ..Default::default() };
        let tone = InputSpec::Tone { freq: 1000.0, amplitude: 0.5, stereo: false };
        let rate = 50_000.0;
        let mut a = AudioInput::open(&tone, rate, &opts).unwrap();
        a.set_blocking(true);
        // The transmitter's clock runs 0.5 % fast: it must get 0.5 % more
        // frames per second of programme.
        a.set_rate_correction_ppm(5000.0);
        let mut x = vec![[0f32; 2]; rate as usize];
        // What was queued before the correction.
        assert_eq!(a.read(&mut x), x.len());
        assert_eq!(a.read(&mut x), x.len());
        let corrected = amplitude(&x, rate, 1000.0 / 1.005);
        assert!((corrected - 0.5).abs() < 0.01, "{corrected}");
        assert!(amplitude(&x, rate, 1000.0) < 0.05);
    }
}
