//! The decoder thread: source → stereo → resampler → ring buffer.

use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::Duration;

use audioadapter_buffers::direct::InterleavedSlice;
use log::{debug, warn};
use rubato::{
    Adjustable, Async, FixedAsync, Indexing, Resampler, SincInterpolationParameters, SincInterpolationType,
    WindowFunction,
};

use crate::source::Source;
use crate::{AudioError, AudioOptions, EndOfInput, Shared};

const CHUNK: usize = 1024;

pub(crate) struct Worker {
    src: Box<dyn Source>,
    channels: usize,
    resampler: Option<Async<f32>>,
    end: EndOfInput,
    drift: bool,
    max_drift_ppm: f64,
    tx: rtrb::Producer<[f32; 2]>,
    capacity: usize,
    shared: Arc<Shared>,
    /// Rate correction (ppm) the resampler was last set to without drift steering.
    rate_ppm: f64,
    // Scratch buffers.
    decoded: Vec<f32>,
    staging: Vec<f32>,
    out: Vec<f32>,
    integral: f64,
    /// Chunks emitted since the producer last had to wait for space.
    since_blocked: u32,
}

impl Worker {
    pub(crate) fn new(
        src: Box<dyn Source>,
        rate: f64,
        opts: &AudioOptions,
        drift: bool,
        tx: rtrb::Producer<[f32; 2]>,
        capacity: usize,
        shared: Arc<Shared>,
    ) -> Result<Worker, AudioError> {
        let info = src.info();
        let ratio = rate / info.rate as f64;
        // Resample unless the rates match exactly and the rate is never steered.
        let resampler = if (ratio - 1.0).abs() > 1e-12 || drift || opts.adjustable_rate {
            // 64-tap Blackman-Harris sinc: the transmit chain low-passes at
            // 15 kHz anyway, so a moderate filter length is transparent while
            // keeping the CPU cost low on the smaller boards.
            let params = SincInterpolationParameters::new(64, WindowFunction::BlackmanHarris2)
                .oversampling_factor(256)
                .interpolation(SincInterpolationType::Linear);
            let r = Async::<f32>::new_sinc(ratio, 1.01, &params, CHUNK, 2, FixedAsync::Input)
                .map_err(|e| AudioError::Unsupported(format!("resampler: {e}")))?;
            debug!("resampling {} Hz -> {rate:.3} Hz (ratio {ratio:.6})", info.rate);
            Some(r)
        } else {
            None
        };
        let out_max = resampler.as_ref().map_or(CHUNK, |r| r.output_frames_max());
        Ok(Worker {
            src,
            channels: info.channels,
            resampler,
            end: opts.end,
            drift,
            max_drift_ppm: opts.max_drift_ppm,
            tx,
            capacity,
            shared,
            rate_ppm: 0.0,
            decoded: Vec::with_capacity(16384),
            staging: Vec::with_capacity(2 * CHUNK),
            out: vec![0.0; 2 * out_max],
            integral: 0.0,
            since_blocked: u32::MAX,
        })
    }

    pub(crate) fn run(mut self) {
        if let Err(e) = self.run_inner() {
            warn!("audio input stopped: {e}");
            if let Ok(mut err) = self.shared.error.lock() {
                *err = Some(e.to_string());
            }
        }
        self.shared.eof.store(true, Ordering::Release);
    }

    fn stopping(&self) -> bool {
        self.shared.stop.load(Ordering::Acquire)
    }

    fn run_inner(&mut self) -> Result<(), AudioError> {
        loop {
            if self.stopping() {
                return Ok(());
            }
            self.decoded.clear();
            let more = self.src.read(&mut self.decoded)?;
            self.push_decoded()?;
            if !more {
                if self.end == EndOfInput::Repeat && self.src.rewind() {
                    self.shared.loops.fetch_add(1, Ordering::Relaxed);
                    continue;
                }
                self.flush()?;
                return Ok(());
            }
        }
    }

    /// Convert decoded interleaved samples to stereo and feed the chain.
    fn push_decoded(&mut self) -> Result<(), AudioError> {
        let ch = self.channels;
        let frames = self.decoded.len() / ch;
        self.shared.frames_decoded.fetch_add(frames as u64, Ordering::Relaxed);
        for f in 0..frames {
            let (l, r) = if ch == 1 {
                let x = self.decoded[f];
                (x, x)
            } else {
                (self.decoded[f * ch], self.decoded[f * ch + 1])
            };
            self.staging.push(l);
            self.staging.push(r);
            if self.staging.len() == 2 * CHUNK {
                self.process_chunk(CHUNK)?;
            }
        }
        Ok(())
    }

    fn process_chunk(&mut self, valid: usize) -> Result<(), AudioError> {
        let Some(r) = self.resampler.as_mut() else {
            let n = valid;
            let frames: Vec<[f32; 2]> = self.staging[..2 * n].chunks_exact(2).map(|c| [c[0], c[1]]).collect();
            self.staging.clear();
            return self.emit(&frames);
        };
        // The transmitter's correction for its crystal error.
        let rate_ppm = f64::from_bits(self.shared.rate_ppm.load(Ordering::Relaxed));
        let rate = 1.0 + rate_ppm * 1e-6;
        if self.drift && self.since_blocked < 16 {
            // The source delivers faster than real time (a file piped into
            // stdin): the ring is full because of back pressure, not clock
            // drift. Play at the programme's true speed.
            self.integral = 0.0;
            let _ = r.set_resample_ratio_relative(rate, true);
            self.shared.drift_ppb.store(0, Ordering::Relaxed);
        } else if self.drift {
            // Keep the ring half full: too full → produce fewer frames.
            let used = (self.capacity - self.tx.slots()) as f64 / self.capacity as f64;
            let err = used - 0.5;
            self.integral = (self.integral + err * 0.02).clamp(-1.0, 1.0);
            let ppm = (err * 400.0 + self.integral * 200.0).clamp(-self.max_drift_ppm, self.max_drift_ppm);
            let _ = r.set_resample_ratio_relative(rate * (1.0 - ppm * 1e-6), true);
            self.shared.drift_ppb.store((-ppm * 1000.0) as i64, Ordering::Relaxed);
        } else if rate_ppm != self.rate_ppm {
            self.rate_ppm = rate_ppm;
            let _ = r.set_resample_ratio_relative(rate, true);
        }
        self.staging.resize(2 * CHUNK, 0.0);
        let input = InterleavedSlice::new(&self.staging[..], 2, CHUNK).expect("input adapter");
        let cap = self.out.len() / 2;
        let mut output = InterleavedSlice::new_mut(&mut self.out[..], 2, cap).expect("output adapter");
        let mut idx = Indexing::new();
        if valid < CHUNK {
            idx.partial_len = Some(valid);
        }
        let (_, written) = r
            .process_into_buffer(&input, &mut output, Some(&idx))
            .map_err(|e| AudioError::Decode(format!("resampler: {e}")))?;
        self.staging.clear();
        let frames: Vec<[f32; 2]> = self.out[..2 * written].chunks_exact(2).map(|c| [c[0], c[1]]).collect();
        self.emit(&frames)
    }

    /// Push frames into the ring, waiting for space (back pressure).
    fn emit(&mut self, frames: &[[f32; 2]]) -> Result<(), AudioError> {
        let mut off = 0;
        self.since_blocked = self.since_blocked.saturating_add(1);
        while off < frames.len() {
            if self.stopping() {
                return Ok(());
            }
            let n = self.tx.slots().min(frames.len() - off);
            if n == 0 {
                self.since_blocked = 0;
                std::thread::sleep(Duration::from_millis(2));
                continue;
            }
            if let Ok(mut chunk) = self.tx.write_chunk(n) {
                let (a, b) = chunk.as_mut_slices();
                a.copy_from_slice(&frames[off..off + a.len()]);
                b.copy_from_slice(&frames[off + a.len()..off + n]);
                chunk.commit_all();
            }
            off += n;
        }
        Ok(())
    }

    /// Push out the remaining partial chunk and the resampler's delay line.
    fn flush(&mut self) -> Result<(), AudioError> {
        let rem = self.staging.len() / 2;
        if rem > 0 {
            self.process_chunk(rem)?;
        }
        if self.resampler.is_some() {
            self.staging.clear();
            self.process_chunk(CHUNK)?;
        }
        Ok(())
    }
}
