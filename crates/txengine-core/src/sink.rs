use std::fmt;

/// Kind of RF control samples a sink consumes.
///
/// Only frequency modulation is implemented today. Amplitude capable sinks
/// (e.g. pad drive-strength modulation for AM/SSB) would add variants here and
/// modulators would negotiate the format with [`RfSink::info`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SampleFormat {
    /// Each sample is the instantaneous carrier frequency offset in Hz
    /// (positive = above the nominal carrier), held for one sample period.
    FrequencyOffset,
}

/// Static description of an RF sink, valid after construction.
#[derive(Debug, Clone)]
pub struct SinkInfo {
    /// Human readable description of the backend and its configuration.
    pub description: String,
    /// Exact output sample rate in Hz. Modulators must render at this rate.
    pub sample_rate: f64,
    /// Carrier frequency actually synthesised by the hardware, in Hz.
    pub carrier_hz: f64,
    /// Largest frequency offset (in Hz, symmetric) the sink can represent.
    /// Samples beyond this are clamped by the sink.
    pub max_offset_hz: f64,
    /// Smallest frequency step the sink can produce at the carrier, in Hz.
    pub resolution_hz: f64,
    /// Largest error of the sink's reference oscillator (± ppm) that
    /// [`RfSink::set_clock_error_ppm`] can correct; 0 if it cannot.
    pub max_clock_error_ppm: f64,
    /// Total buffer capacity in samples (the transmit latency when full).
    pub buffer_len: usize,
    /// Sample format the sink consumes.
    pub format: SampleFormat,
    /// The sink consumes samples in real time (hardware). Software sinks
    /// (files) accept data as fast as it is produced.
    pub realtime: bool,
    /// The output holds each sample for one period (zero-order hold), so
    /// its frequency response has a `sinc(f / fs)` roll-off that modulators
    /// may pre-compensate.
    pub zero_order_hold: bool,
    /// Measured modulation response of the hardware beyond the zero-order
    /// hold, as `(frequency Hz, gain dB)` points in ascending frequency
    /// (linearly interpolated, held beyond the ends). Empty: ideal.
    pub response_db: Vec<(f64, f64)>,
}

impl SinkInfo {
    /// Linear gain of the hardware's modulation response at `f` Hz
    /// (zero-order hold and [`SinkInfo::response_db`]).
    pub fn response_gain(&self, f: f64) -> f64 {
        let hold = if self.zero_order_hold { sinc(f / self.sample_rate) } else { 1.0 };
        hold * 10f64.powf(interpolate_db(&self.response_db, f) / 20.0)
    }
}

/// Linear interpolation in a `(frequency, dB)` table, held beyond the ends.
pub fn interpolate_db(table: &[(f64, f64)], f: f64) -> f64 {
    match table {
        [] => 0.0,
        [(f0, g0), ..] if f <= *f0 => *g0,
        [.., (f1, g1)] if f >= *f1 => *g1,
        _ => {
            let i = table.iter().position(|&(x, _)| x > f).expect("bracketed");
            let ((xa, ya), (xb, yb)) = (table[i - 1], table[i]);
            ya + (yb - ya) * (f - xa) / (xb - xa)
        }
    }
}

fn sinc(x: f64) -> f64 {
    if x.abs() < 1e-12 { 1.0 } else { (std::f64::consts::PI * x).sin() / (std::f64::consts::PI * x) }
}

/// Result of [`RfSink::poll`].
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SinkPoll {
    /// Number of samples that can be written right now.
    pub writable: usize,
    /// Total number of samples the hardware has consumed since start.
    pub consumed: u64,
    /// Total number of samples written by the producer since start.
    pub written: u64,
    /// Number of underruns detected since start (the producer was too slow
    /// and the hardware ran out of fresh samples).
    pub underruns: u64,
}

/// Errors reported by sinks.
#[derive(Debug, thiserror::Error)]
pub enum SinkError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
    #[error("hardware error: {0}")]
    Hardware(String),
    #[error("invalid configuration: {0}")]
    Config(String),
    #[error("sink is not running")]
    NotRunning,
}

impl SinkError {
    pub fn hw(msg: impl fmt::Display) -> Self {
        SinkError::Hardware(msg.to_string())
    }
    pub fn config(msg: impl fmt::Display) -> Self {
        SinkError::Config(msg.to_string())
    }
}

/// A consumer of RF control samples (hardware transmitter or software sink).
///
/// Life cycle: construct (hardware configured but silent) → [`write`] to
/// pre-fill the buffer → [`start`] → repeatedly [`poll`] + [`write`] →
/// [`stop`]. Dropping a sink must leave the hardware in its original state.
///
/// [`write`]: RfSink::write
/// [`start`]: RfSink::start
/// [`poll`]: RfSink::poll
/// [`stop`]: RfSink::stop
pub trait RfSink: Send {
    /// Static information about the sink.
    fn info(&self) -> &SinkInfo;

    /// Start emitting. The buffer should have been pre-filled with [`RfSink::write`].
    fn start(&mut self) -> Result<(), SinkError>;

    /// Query buffer state and hardware health. Must be cheap; it is called
    /// every few milliseconds.
    fn poll(&mut self) -> Result<SinkPoll, SinkError>;

    /// Append samples. `samples.len()` must not exceed the `writable` count
    /// returned by the latest [`RfSink::poll`] (before start: the buffer length).
    fn write(&mut self, samples: &[f32]) -> Result<(), SinkError>;

    /// Stop emitting and release the hardware. Idempotent.
    fn stop(&mut self) -> Result<(), SinkError>;

    /// Tell the sink that its reference oscillator (the crystal its clocks
    /// derive from) runs `ppm` parts per million fast (negative: slow). The
    /// sink retunes the carrier so that it, and the frequency offsets of the
    /// samples, are right in true frequency, starting with the next samples
    /// written. The sample clock is not changed: samples are emitted at
    /// [`SinkInfo::sample_rate`]` · (1 + ppm · 10⁻⁶)`, which modulators learn
    /// from [`RenderContext::sample_rate`](crate::RenderContext::sample_rate). May be
    /// called at any time, also before [`RfSink::start`]; fails beyond
    /// ±[`SinkInfo::max_clock_error_ppm`].
    fn set_clock_error_ppm(&mut self, ppm: f64) -> Result<(), SinkError> {
        if ppm == 0.0 {
            Ok(())
        } else {
            Err(SinkError::config("this output cannot correct a clock error"))
        }
    }
}
