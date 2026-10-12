//! Clock discipline: measures the error of the transmitter's crystal while
//! transmitting, and corrects for it.
//!
//! All clocks of an output derive from one crystal: the synthesiser that
//! makes the carrier and the timer that paces the samples. A crystal that
//! runs `e` fast (Raspberry Pi crystals: up to a few tens of ppm, drifting
//! with temperature) puts the carrier `e` too high and emits the samples `e`
//! too fast, which raises the pilot, the subcarriers and the playback speed
//! by the same `e`. One measured error therefore drives two corrections:
//!
//! * the sink retunes the carrier ([`RfSink::set_clock_error_ppm`]);
//! * the modulator renders for the actual sample rate
//!   ([`RenderContext::sample_rate`]), which keeps the subcarriers on
//!   frequency and plays the programme at its true speed.
//!
//! The hardware's sample clock itself is not touched, so the correction
//! never fills or drains the output buffer. A live input follows its own
//! clock: its drift steering keeps its buffer centred and absorbs whatever
//! difference remains between the source's clock and true time.
//!
//! Sinks whose carrier paces the samples ([`SinkInfo::paced_by_carrier`],
//! AM) are the exception: correcting their carrier corrects their sample
//! clock too, so the samples leave at the nominal rate in true time and the
//! modulator has nothing to correct. The estimator must not see those
//! corrections: the hardware position is mapped back to the position the
//! sink would have reached uncorrected (`Uncorrected`).
//!
//! Measurement: every poll of the sink reads the hardware position (the
//! index of the sample being emitted), bracketed by readings of the raw
//! monotonic clock. A [`TimeReference`] turns these readings into
//! observations "sample n was emitted at time t", and the [`RateEstimator`]
//! fits the time error of the sample clock, `n / fs - t`, with an
//! exponentially weighted straight line whose slope is the crystal error.
//! The corrections do not change what is measured, so the estimate is
//! open loop and cannot oscillate.
//!
//! The reference today is the [`SystemClock`]: the frequency an NTP daemon
//! has given the system clock. A GPS receiver works through it as well:
//! chrony (with gpsd and the receiver's PPS) holds the system clock within
//! about a microsecond of GPS time. A single frequency network, which also
//! needs the timeline aligned to the second, can implement [`TimeReference`]
//! directly: timestamp each PPS edge and interpolate the sample index
//! emitted at the edge between the readings around it.
//!
//! [`RenderContext::sample_rate`]: txengine_core::RenderContext::sample_rate
//! [`SinkInfo::paced_by_carrier`]: txengine_core::SinkInfo::paced_by_carrier

use std::collections::VecDeque;

use log::{debug, info, trace, warn};
use txengine_core::{RfSink, SinkError, SinkPoll};

use crate::cli::{self, Ppm};
use crate::rt;

/// Time constant of the estimator's exponential weighting (s): long enough
/// to smooth the steps in which an NTP daemon adjusts the system clock's
/// frequency, short enough to follow the crystal while it warms up.
const TAU: f64 = 120.0;
/// Time a fit's data must span before it is used (s).
const MIN_SPAN: f64 = 20.0;
/// Largest standard error of a fit that is used (ppm).
const MAX_STDERR_PPM: f64 = 0.2;
/// Observations are ignored for this long at the start and after an
/// underrun, which interrupts the timeline (s).
const SETTLE: f64 = 2.0;

/// "Sample `sample` was emitted at `time`" (seconds on the reference's
/// time scale), known to ±`uncertainty` seconds. The sample index may be
/// fractional (a position between samples).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Observation {
    pub sample: f64,
    pub time: f64,
    pub uncertainty: f64,
}

/// A time reference the sample clock is measured against.
pub trait TimeReference: Send {
    /// Name for messages ("the system clock").
    fn name(&self) -> &'static str;

    /// A reading of the hardware position: sample `sample` was being
    /// emitted at some moment between the raw monotonic times `t0` and `t1`
    /// ([`rt::monotonic_raw`]). Returns the observation it yields on the
    /// reference's time scale, if any.
    fn observe(&mut self, sample: f64, t0: f64, t1: f64) -> Option<Observation>;

    /// Whether the reference keeps true time at the moment (an NTP daemon
    /// has synchronised the system clock, a GPS receiver has a fix).
    /// Observations are only used while it does.
    fn synchronised(&mut self) -> bool;
}

/// Readings that took longer than this were interrupted (s).
const MAX_BRACKET: f64 = 500e-6;

/// The observation of a reading at its middle, unless it was interrupted;
/// times as given.
fn bracket(sample: f64, t0: f64, t1: f64) -> Option<Observation> {
    (t1 - t0 <= MAX_BRACKET).then_some(Observation { sample, time: 0.5 * (t0 + t1), uncertainty: 0.5 * (t1 - t0) })
}

/// The system clock as a frequency reference: the raw monotonic clock (the
/// CPU's timer) running at the frequency the NTP daemon has set for the
/// system clock (or chrony from a GPS receiver's PPS).
///
/// The system clock itself (`CLOCK_MONOTONIC`) is no good for this: the
/// kernel also slews it to remove time offsets, which systemd-timesyncd and
/// ntpd leave to the kernel's PLL. After each NTP poll that runs the clock
/// up to tens of ppm off for many minutes (a 15 ms offset taken out over
/// 512 s starts at 29 ppm). chrony slews by adjusting the frequency, which
/// this follows; its slews are gentle.
#[derive(Debug, Default)]
pub struct SystemClock {
    /// The kernel's last answer, and the raw time it was asked.
    kernel: Option<(rt::KernelClock, f64)>,
    /// A raw time and the reference time at it, from which the reference
    /// runs at the kernel's frequency.
    anchor: (f64, f64),
}

impl SystemClock {
    /// Ask the kernel again once a second (it changes the frequency at NTP
    /// polls, every 1-30 minutes).
    fn refresh(&mut self, raw: f64) -> rt::KernelClock {
        match self.kernel {
            Some((k, asked)) if raw - asked < 1.0 => k,
            _ => {
                // Carry the time scale forward at the old frequency.
                self.anchor = (raw, self.time_at(raw));
                let k = rt::kernel_clock().unwrap_or(rt::KernelClock { synchronised: false, frequency: 0.0 });
                self.kernel = Some((k, raw));
                k
            }
        }
    }

    fn time_at(&self, raw: f64) -> f64 {
        let frequency = self.kernel.map_or(0.0, |(k, _)| k.frequency);
        self.anchor.1 + (raw - self.anchor.0) * (1.0 + frequency)
    }
}

impl TimeReference for SystemClock {
    fn name(&self) -> &'static str {
        "the system clock"
    }

    fn observe(&mut self, sample: f64, t0: f64, t1: f64) -> Option<Observation> {
        self.refresh(t1);
        let o = bracket(sample, t0, t1)?;
        Some(Observation { time: self.time_at(o.time), ..o })
    }

    fn synchronised(&mut self) -> bool {
        self.refresh(rt::monotonic_raw()).synchronised
    }
}

/// A fitted frequency error of the sample clock.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Fit {
    /// The sample clock runs this many ppm fast (negative: slow).
    pub ppm: f64,
    /// Standard error of `ppm`, from the scatter of the data.
    pub stderr_ppm: f64,
    /// Time spanned by the data (s).
    pub span: f64,
}

/// The current line: weighted means, slope and RMS residual.
#[derive(Debug, Clone, Copy)]
struct Line {
    x: f64,
    z: f64,
    slope: f64,
    rms: f64,
}

/// Estimates the frequency error of a sample clock that runs nominally at
/// `fs`, as the slope of its time error `n / fs - t`: a least-squares line
/// through one point per second (that second's most precise observation),
/// weighted with `exp(-age / TAU)`.
#[derive(Debug)]
pub struct RateEstimator {
    fs: f64,
    /// Observation the time error is counted from.
    origin: Option<Observation>,
    /// (time since the origin, time error), oldest first.
    points: VecDeque<(f64, f64)>,
    weights: Vec<f64>,
    /// The second being collected and its best observation.
    bin: Option<(i64, Observation)>,
    line: Option<Line>,
    /// Consecutive points rejected as outliers.
    rejected: u32,
}

impl RateEstimator {
    pub fn new(fs: f64) -> RateEstimator {
        RateEstimator {
            fs,
            origin: None,
            points: VecDeque::new(),
            weights: Vec::new(),
            bin: None,
            line: None,
            rejected: 0,
        }
    }

    /// Forget the data (the timeline was interrupted).
    pub fn restart(&mut self) {
        self.origin = None;
        self.points.clear();
        self.bin = None;
        self.line = None;
        self.rejected = 0;
    }

    /// Add an observation. Returns a new fit when a second is complete.
    pub fn add(&mut self, o: Observation) -> Option<Fit> {
        let second = o.time.floor() as i64;
        match self.bin {
            Some((s, best)) if s == second => {
                if o.uncertainty < best.uncertainty {
                    self.bin = Some((s, o));
                }
                None
            }
            Some((_, best)) => {
                self.bin = Some((second, o));
                self.push(best)
            }
            None => {
                self.bin = Some((second, o));
                None
            }
        }
    }

    fn push(&mut self, o: Observation) -> Option<Fit> {
        let origin = *self.origin.get_or_insert(o);
        let x = o.time - origin.time;
        let z = (o.sample - origin.sample) / self.fs - x;
        if let Some(l) = self.line
            && self.points.len() >= 10
        {
            // A jump the caller did not report: drop single outliers, and
            // start again if the line has moved for good.
            let r = z - (l.z + l.slope * (x - l.x));
            if r.abs() > (10.0 * l.rms).max(1e-3) {
                self.rejected += 1;
                debug!("clock: observation {:.0} µs off the line ignored", r * 1e6);
                if self.rejected >= 10 {
                    warn!("the transmitter's timeline jumped; measuring the crystal error again");
                    self.restart();
                }
                return None;
            }
        }
        self.rejected = 0;
        self.points.push_back((x, z));
        while self.points.front().is_some_and(|&(x0, _)| x - x0 > 6.0 * TAU) {
            self.points.pop_front();
        }
        if self.points.len() < 3 {
            return None;
        }
        self.weights.clear();
        self.weights.extend(self.points.iter().map(|&(xi, _)| (-(x - xi) / TAU).exp()));
        let pw = || self.points.iter().zip(&self.weights);
        let (mut sw, mut sx, mut sz) = (0.0, 0.0, 0.0);
        for (&(xi, zi), &w) in pw() {
            sw += w;
            sx += w * xi;
            sz += w * zi;
        }
        let (xm, zm) = (sx / sw, sz / sw);
        let (mut sxx, mut sxz) = (0.0, 0.0);
        for (&(xi, zi), &w) in pw() {
            sxx += w * (xi - xm) * (xi - xm);
            sxz += w * (xi - xm) * (zi - zm);
        }
        let slope = sxz / sxx;
        let (mut srr, mut sxx2) = (0.0, 0.0);
        for (&(xi, zi), &w) in pw() {
            let r = zi - zm - slope * (xi - xm);
            srr += w * r * r;
            sxx2 += w * w * (xi - xm) * (xi - xm);
        }
        let n = self.points.len() as f64;
        let var = srr / sw * n / (n - 2.0);
        self.line = Some(Line { x: xm, z: zm, slope, rms: var.sqrt() });
        Some(Fit { ppm: slope * 1e6, stderr_ppm: (var * sxx2).sqrt() / sxx * 1e6, span: x - self.points[0].0 })
    }
}

/// The position a sink paced by its carrier would have reached without
/// the corrections, which change its sample clock: every correction `c`
/// (ppm) slows the samples by `1 + c · 10⁻⁶`, so each sample emitted under
/// it counts as that many uncorrected ones.
#[derive(Debug, Clone, Copy, Default)]
struct Uncorrected {
    /// The sample from which `ppm` is in effect, and the position there.
    since: u64,
    position: f64,
    ppm: f64,
}

impl Uncorrected {
    fn at(&self, sample: u64) -> f64 {
        self.position + sample.saturating_sub(self.since) as f64 * (1.0 + self.ppm * 1e-6)
    }

    /// The correction changes to `ppm` at `sample`.
    fn change(&mut self, sample: u64, ppm: f64) {
        *self = Uncorrected { since: sample, position: self.at(sample), ppm };
    }
}

/// Keeps a sink and the timeline corrected for the sink's crystal error:
/// fixed, or measured continuously against a [`TimeReference`] (see the
/// module documentation).
pub struct ClockDiscipline {
    mode: Ppm,
    reference: Box<dyn TimeReference>,
    /// Created when attached to a sink (needs its sample rate).
    estimator: Option<RateEstimator>,
    carrier_hz: f64,
    /// Correction in effect (ppm).
    ppm: f64,
    /// The sink's samples are paced by its carrier
    /// ([`SinkInfo::paced_by_carrier`](txengine_core::SinkInfo::paced_by_carrier)),
    /// and its position as it would be uncorrected.
    paced: bool,
    uncorrected: Uncorrected,
    /// The sink's position at the latest poll.
    position: u64,
    /// Latest usable measurement.
    measured: Option<Fit>,
    synchronised: Option<bool>,
    underruns: u64,
    /// Reference time of the first observation of the current stretch.
    stretch_start: Option<f64>,
    /// Correction last reported in the log.
    logged_ppm: Option<f64>,
    implausible_reported: bool,
}

impl ClockDiscipline {
    pub fn new(mode: Ppm, reference: Box<dyn TimeReference>) -> ClockDiscipline {
        ClockDiscipline {
            mode,
            reference,
            estimator: None,
            carrier_hz: 0.0,
            ppm: 0.0,
            paced: false,
            uncorrected: Uncorrected::default(),
            position: 0,
            measured: None,
            synchronised: None,
            underruns: 0,
            stretch_start: None,
            logged_ppm: None,
            implausible_reported: false,
        }
    }

    /// Start working with `sink` (before it is pre-filled): applies a fixed
    /// correction.
    pub fn attach(&mut self, sink: &mut dyn RfSink) -> Result<(), SinkError> {
        let info = sink.info();
        self.carrier_hz = info.carrier_hz;
        self.paced = info.paced_by_carrier;
        self.estimator = Some(RateEstimator::new(info.sample_rate));
        match self.mode {
            Ppm::Fixed(v) => {
                self.correct(sink, v)?;
                if v != 0.0 {
                    info!("correcting a crystal error of {v:+.2} ppm (carrier {:+.0} Hz)", -v * 1e-6 * self.carrier_hz);
                }
            }
            Ppm::Auto if info.max_clock_error_ppm == 0.0 => warn!("this output cannot correct its crystal error"),
            Ppm::Auto => {}
            Ppm::Off => debug!("crystal error correction off"),
        }
        Ok(())
    }

    fn mode_name(&self) -> &'static str {
        match self.mode {
            Ppm::Auto => "auto",
            Ppm::Fixed(_) => "fixed",
            Ppm::Off => "off",
        }
    }

    /// Actual sample rate relative to the sink's nominal rate: the
    /// crystal error, corrected for a sink paced by its carrier.
    pub fn rate_ratio(&self) -> f64 {
        if self.paced { 1.0 } else { 1.0 + self.ppm * 1e-6 }
    }

    /// Put the correction `ppm` into effect.
    fn correct(&mut self, sink: &mut dyn RfSink, ppm: f64) -> Result<(), SinkError> {
        sink.set_clock_error_ppm(ppm)?;
        if self.paced {
            self.uncorrected.change(self.position, ppm);
        }
        self.ppm = ppm;
        Ok(())
    }

    /// Poll the sink, timing its position against the reference, and
    /// correct it when the estimate changes.
    pub fn poll(&mut self, sink: &mut dyn RfSink) -> Result<SinkPoll, SinkError> {
        let t0 = rt::monotonic_raw();
        let p = sink.poll()?;
        let t1 = rt::monotonic_raw();
        self.observe(sink, &p, t0, t1)?;
        Ok(p)
    }

    fn observe(&mut self, sink: &mut dyn RfSink, p: &SinkPoll, t0: f64, t1: f64) -> Result<(), SinkError> {
        self.position = p.consumed;
        if p.underruns != self.underruns {
            // The hardware waited for data: the timeline has a gap.
            self.underruns = p.underruns;
            if let Some(e) = self.estimator.as_mut() {
                e.restart();
            }
            self.stretch_start = None;
            return Ok(());
        }
        if self.estimator.is_none() || self.mode == Ppm::Off {
            return Ok(());
        }
        if !self.reference_synchronised() {
            return Ok(());
        }
        let Some(o) = self.reference.observe(self.uncorrected.at(p.consumed), t0, t1) else { return Ok(()) };
        if o.time < *self.stretch_start.get_or_insert(o.time) + SETTLE {
            return Ok(());
        }
        let Some(fit) = self.estimator.as_mut().and_then(|e| e.add(o)) else { return Ok(()) };
        trace!("clock: {:+.4} ppm ± {:.4} over {:.0} s", fit.ppm, fit.stderr_ppm, fit.span);
        if fit.span < MIN_SPAN || fit.stderr_ppm > MAX_STDERR_PPM {
            return Ok(());
        }
        if fit.ppm.abs() > txengine_hal::MAX_CLOCK_ERROR_PPM {
            if !self.implausible_reported {
                self.implausible_reported = true;
                warn!(
                    "measured a crystal error of {:+.1} ppm against {}, more than a crystal can be off; ignoring it",
                    fit.ppm,
                    self.reference.name()
                );
            }
            return Ok(());
        }
        self.measured = Some(fit);
        if self.mode == Ppm::Auto {
            self.apply(sink, fit.ppm)?;
        }
        Ok(())
    }

    /// Ask the reference, report changes, and start measuring afresh when
    /// it (re)gains synchronisation: its rate is unknown for the time it was
    /// not synchronised.
    fn reference_synchronised(&mut self) -> bool {
        let now = self.reference.synchronised();
        if self.synchronised != Some(now) {
            let name = self.reference.name();
            let auto = self.mode == Ppm::Auto;
            match (self.synchronised, now) {
                (_, true) => {
                    if self.synchronised.is_some() {
                        info!("{name} is synchronised; measuring the crystal error");
                    }
                    if let Some(e) = self.estimator.as_mut() {
                        e.restart();
                    }
                    self.stretch_start = None;
                }
                (None, false) if auto => warn!(
                    "{name} is not synchronised (no NTP or GPS time), so the crystal error cannot be measured; \
                     transmitting uncorrected until it is (or give --ppm)"
                ),
                (Some(true), false) if auto => {
                    warn!("{name} lost synchronisation; keeping the crystal error correction at {:+.2} ppm", self.ppm)
                }
                _ => debug!("{name} synchronised: {now}"),
            }
            self.synchronised = Some(now);
        }
        now
    }

    fn apply(&mut self, sink: &mut dyn RfSink, ppm: f64) -> Result<(), SinkError> {
        self.correct(sink, ppm)?;
        match self.logged_ppm {
            None => info!(
                "crystal error {ppm:+.2} ppm (measured against {}): correcting the carrier by {:+.0} Hz and the sample rate",
                self.reference.name(),
                -ppm * 1e-6 * self.carrier_hz
            ),
            Some(l) if (ppm - l).abs() >= 0.5 => info!("crystal error now {ppm:+.2} ppm"),
            Some(_) => return Ok(()),
        }
        self.logged_ppm = Some(ppm);
        Ok(())
    }

    /// The `PPM` control command: no argument reports, a number fixes the
    /// correction, `auto` returns to measuring, `off` stops both.
    pub fn command(&mut self, sink: &mut dyn RfSink, arg: &str) -> Result<String, String> {
        let arg = arg.trim();
        if !arg.is_empty() {
            let mode = cli::parse_ppm(arg)?;
            match mode {
                Ppm::Fixed(v) => self.correct(sink, v).map_err(|e| e.to_string())?,
                Ppm::Auto => {
                    if let Some(f) = self.measured {
                        self.apply(sink, f.ppm).map_err(|e| e.to_string())?;
                    }
                }
                Ppm::Off => {
                    self.correct(sink, 0.0).map_err(|e| e.to_string())?;
                    self.measured = None;
                }
            }
            if self.mode == Ppm::Off && mode != Ppm::Off {
                // Nothing was observed meanwhile: start measuring afresh.
                if let Some(e) = self.estimator.as_mut() {
                    e.restart();
                }
                self.stretch_start = None;
            }
            self.mode = mode;
        }
        Ok(self.describe())
    }

    /// One line on the correction and the measurement.
    pub fn describe(&self) -> String {
        if self.mode == Ppm::Off {
            return "crystal error correction off".into();
        }
        let measured = match (self.measured, self.synchronised) {
            (Some(f), _) => format!(
                "measured {:+.3} ± {:.3} ppm over {:.0} s against {}",
                f.ppm,
                f.stderr_ppm,
                f.span,
                self.reference.name()
            ),
            (None, Some(false)) => format!("not measured: {} is not synchronised", self.reference.name()),
            (None, _) => "not measured yet".into(),
        };
        format!("crystal error correction {:+.3} ppm ({}), {measured}", self.ppm, self.mode_name())
    }

    /// Short form for the periodic status line.
    pub fn summary(&self) -> String {
        match (self.mode, self.measured) {
            (Ppm::Auto, Some(_)) => format!("crystal {:+.2} ppm", self.ppm),
            (Ppm::Auto, None) => "crystal not measured yet".into(),
            (Ppm::Fixed(v), m) => {
                format!(
                    "crystal {v:+.2} ppm (fixed{})",
                    m.map_or(String::new(), |f| format!(", measured {:+.2}", f.ppm))
                )
            }
            (Ppm::Off, _) => "crystal correction off".into(),
        }
    }

    /// `key=value` fields for the `STATUS` command.
    pub fn status_fields(&self) -> String {
        let measured = self.measured.map_or("none".into(), |f| format!("{:+.3}", f.ppm));
        format!("ppm={:+.3} ppm_mode={} ppm_measured={measured}", self.ppm, self.mode_name())
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;
    use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

    use txengine_core::{SampleFormat, SinkInfo};

    use super::*;

    const FS: f64 = 250_000.0;

    /// Small deterministic generator (xorshift) for the jitter.
    struct Noise(u64);

    impl Noise {
        fn uniform(&mut self) -> f64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            (self.0 >> 11) as f64 / (1u64 << 53) as f64
        }
    }

    /// Readings of a sample clock whose error (ppm) is `ppm(t)`, taken every
    /// 4 ms with 2-30 µs brackets (one in 50 interrupted for a millisecond).
    fn readings(seconds: f64, ppm: impl Fn(f64) -> f64, mut f: impl FnMut(f64, u64, f64, f64)) {
        let mut noise = Noise(0x2545_f491_4f6c_dd1d);
        let (mut t, mut phase) = (0.0, 0.0);
        let dt = 0.004;
        while t < seconds {
            phase += FS * (1.0 + ppm(t) * 1e-6) * dt;
            t += dt;
            let width = if noise.uniform() < 0.02 { 1e-3 } else { 2e-6 + 28e-6 * noise.uniform() };
            let t0 = t - width * noise.uniform();
            f(t, phase as u64, t0, t0 + width);
        }
    }

    #[test]
    fn estimates_a_constant_error() {
        let mut est = RateEstimator::new(FS);
        let mut last = None;
        readings(
            60.0,
            |_| 12.3,
            |_, n, t0, t1| {
                if let Some(o) = bracket(n as f64, t0, t1) {
                    last = est.add(o).or(last);
                }
            },
        );
        let fit = last.unwrap();
        assert!((fit.ppm - 12.3).abs() < 0.01, "{fit:?}");
        assert!(fit.stderr_ppm < 0.01, "{fit:?}");
        assert!(fit.span > 55.0);
    }

    #[test]
    fn follows_a_drifting_crystal() {
        let mut est = RateEstimator::new(FS);
        let mut last = None;
        // Warming up: +1 ppm per minute.
        let ppm = |t: f64| -5.0 + t / 60.0;
        readings(900.0, ppm, |_, n, t0, t1| {
            if let Some(o) = bracket(n as f64, t0, t1) {
                last = est.add(o).or(last);
            }
        });
        // For a ramp, the exponentially weighted slope is the slope 2·TAU
        // ago, or 1.78·TAU with the window cut off at 6·TAU.
        let fit = last.unwrap();
        assert!((fit.ppm - ppm(900.0 - 1.78 * TAU)).abs() < 0.05, "{fit:?}");
    }

    #[test]
    fn ignores_outliers_and_restarts_after_a_jump() {
        let mut est = RateEstimator::new(FS);
        let mut fits = Vec::new();
        let o = |time: f64, offset: f64| Observation {
            sample: ((time - offset) * FS * (1.0 + 8e-6)).floor(),
            time,
            uncertainty: 1e-6,
        };
        for k in 0..100 {
            // One wild point at 50 s.
            let offset = if k == 50 { 5e-3 } else { 0.0 };
            fits.extend(est.add(o(k as f64 + 0.5, offset)));
        }
        let fit = *fits.last().unwrap();
        assert!((fit.ppm - 8.0).abs() < 0.01, "{fit:?}");
        // The timeline moves by 20 ms for good (an unreported gap).
        for k in 100..140 {
            fits.extend(est.add(o(k as f64 + 0.5, 0.02)));
        }
        let fit = *fits.last().unwrap();
        assert!((fit.ppm - 8.0).abs() < 0.05 && fit.span < 35.0, "{fit:?}");
        assert!(fits.iter().filter(|f| f.span >= MIN_SPAN).all(|f| (f.ppm - 8.0).abs() < 0.1), "no fit is thrown off");
    }

    /// A sink whose position the test sets.
    struct FakeSink {
        info: SinkInfo,
        poll: SinkPoll,
        ppm: f64,
    }

    impl FakeSink {
        fn new() -> FakeSink {
            FakeSink {
                info: SinkInfo {
                    description: "fake".into(),
                    sample_rate: FS,
                    carrier_hz: 100e6,
                    max_offset_hz: 100e3,
                    resolution_hz: 1.0,
                    max_clock_error_ppm: 200.0,
                    paced_by_carrier: false,
                    buffer_len: 1024,
                    format: SampleFormat::FrequencyOffset,
                    realtime: true,
                    zero_order_hold: true,
                    response_db: Vec::new(),
                },
                poll: SinkPoll::default(),
                ppm: 0.0,
            }
        }
    }

    impl RfSink for FakeSink {
        fn info(&self) -> &SinkInfo {
            &self.info
        }
        fn start(&mut self) -> Result<(), SinkError> {
            Ok(())
        }
        fn poll(&mut self) -> Result<SinkPoll, SinkError> {
            Ok(self.poll)
        }
        fn write(&mut self, _: &[f32]) -> Result<(), SinkError> {
            Ok(())
        }
        fn stop(&mut self) -> Result<(), SinkError> {
            Ok(())
        }
        fn set_clock_error_ppm(&mut self, ppm: f64) -> Result<(), SinkError> {
            self.ppm = ppm;
            Ok(())
        }
    }

    /// A reference on simulated time: the test sets the time of the next
    /// reading and whether it is synchronised.
    #[derive(Clone)]
    struct FakeReference {
        time: Arc<AtomicU64>,
        synced: Arc<AtomicBool>,
    }

    impl TimeReference for FakeReference {
        fn name(&self) -> &'static str {
            "the fake clock"
        }
        fn observe(&mut self, sample: f64, _: f64, _: f64) -> Option<Observation> {
            Some(Observation { sample, time: f64::from_bits(self.time.load(Ordering::Relaxed)), uncertainty: 1e-6 })
        }
        fn synchronised(&mut self) -> bool {
            self.synced.load(Ordering::Relaxed)
        }
    }

    struct Rig {
        clock: ClockDiscipline,
        sink: FakeSink,
        reference: FakeReference,
        t: f64,
        phase: f64,
    }

    impl Rig {
        fn new(mode: Ppm) -> Rig {
            Self::with_sink(mode, FakeSink::new())
        }

        fn with_sink(mode: Ppm, mut sink: FakeSink) -> Rig {
            let reference =
                FakeReference { time: Arc::new(AtomicU64::new(0)), synced: Arc::new(AtomicBool::new(true)) };
            let mut clock = ClockDiscipline::new(mode, Box::new(reference.clone()));
            clock.attach(&mut sink).unwrap();
            Rig { clock, sink, reference, t: 0.0, phase: 0.0 }
        }

        /// Run `seconds` with the crystal `ppm` off; `stall` stops the
        /// hardware for that long at the start, with an underrun.
        fn run(&mut self, seconds: f64, ppm: f64, stall: f64) {
            if stall > 0.0 {
                self.sink.poll.underruns += 1;
                self.t += stall;
            }
            let end = self.t + seconds;
            while self.t < end {
                self.t += 0.01;
                // A sink paced by its carrier is slowed by its correction.
                let corrected = if self.sink.info.paced_by_carrier { 1.0 + self.sink.ppm * 1e-6 } else { 1.0 };
                self.phase += FS * (1.0 + ppm * 1e-6) / corrected * 0.01;
                self.sink.poll.consumed = self.phase as u64;
                self.reference.time.store(self.t.to_bits(), Ordering::Relaxed);
                self.clock.poll(&mut self.sink).unwrap();
                assert!(self.clock.ppm.abs() < 100.0, "never a wild correction");
            }
        }
    }

    #[test]
    fn measures_and_corrects_the_sink_and_the_rate() {
        let mut r = Rig::new(Ppm::Auto);
        r.run(10.0, 15.0, 0.0);
        assert_eq!(r.clock.ppm, 0.0, "nothing applied before the fit has settled");
        // (Readings every 10 ms make the 4 µs quantisation of the position
        // correlated: a few hundredths of a ppm.)
        r.run(30.0, 15.0, 0.0);
        assert!((r.clock.ppm - 15.0).abs() < 0.05, "{}", r.clock.describe());
        assert_eq!(r.sink.ppm, r.clock.ppm);
        assert_eq!(r.clock.rate_ratio(), 1.0 + r.clock.ppm * 1e-6);
        // An underrun stops the hardware for 50 ms: the estimate carries
        // on, and the new stretch takes over once it has settled.
        r.run(5.0, 15.0, 0.05);
        assert!((r.clock.ppm - 15.0).abs() < 0.05);
        r.run(60.0, 16.0, 0.0);
        assert!((r.clock.ppm - 16.0).abs() < 0.1, "{}", r.clock.describe());
    }

    #[test]
    fn corrects_a_sink_paced_by_its_carrier_without_chasing_itself() {
        let mut sink = FakeSink::new();
        sink.info.paced_by_carrier = true;
        let mut r = Rig::with_sink(Ppm::Auto, sink);
        r.run(40.0, -54.0, 0.0);
        assert!((r.clock.ppm + 54.0).abs() < 0.1, "{}", r.clock.describe());
        // The samples now leave at the nominal rate: nothing for the
        // modulator to correct.
        assert_eq!(r.clock.rate_ratio(), 1.0);
        // The correction holds although the sink now runs on time, and a
        // change of the crystal is followed.
        r.run(120.0, -54.0, 0.0);
        assert!((r.clock.ppm + 54.0).abs() < 0.05, "{}", r.clock.describe());
        r.run(5.0, -54.0, 0.05);
        r.run(300.0, -50.0, 0.0);
        assert!((r.clock.ppm + 50.0).abs() < 0.2, "{}", r.clock.describe());
        // A fixed correction set at run time does not upset the estimate.
        r.clock.command(&mut r.sink, "-20").unwrap();
        r.run(60.0, -50.0, 0.0);
        assert!((r.clock.measured.unwrap().ppm + 50.0).abs() < 0.2, "{}", r.clock.describe());
    }

    #[test]
    fn waits_for_a_synchronised_reference() {
        let mut r = Rig::new(Ppm::Auto);
        r.reference.synced.store(false, Ordering::Relaxed);
        r.run(60.0, 15.0, 0.0);
        assert_eq!(r.clock.ppm, 0.0);
        assert!(r.clock.describe().contains("not synchronised"));
        r.reference.synced.store(true, Ordering::Relaxed);
        r.run(30.0, 15.0, 0.0);
        assert!((r.clock.ppm - 15.0).abs() < 0.05);
    }

    #[test]
    fn fixed_correction_and_control() {
        let mut r = Rig::new(Ppm::Fixed(-3.5));
        assert_eq!(r.sink.ppm, -3.5);
        r.run(40.0, 7.0, 0.0);
        // Measured, but the fixed value stays in effect.
        assert_eq!(r.clock.ppm, -3.5);
        assert!((r.clock.measured.unwrap().ppm - 7.0).abs() < 0.05);
        assert!(r.clock.summary().contains("fixed"));
        r.clock.command(&mut r.sink, "auto").unwrap();
        assert!((r.sink.ppm - 7.0).abs() < 0.05);
        r.clock.command(&mut r.sink, "1.25").unwrap();
        assert_eq!(r.sink.ppm, 1.25);
        assert!(r.clock.command(&mut r.sink, "fast").is_err());
        assert!(r.clock.command(&mut r.sink, "").unwrap().contains("+1.250 ppm (fixed)"));
    }

    #[test]
    fn can_be_turned_off() {
        let mut r = Rig::new(Ppm::Off);
        r.run(40.0, 7.0, 0.0);
        assert_eq!((r.clock.ppm, r.sink.ppm), (0.0, 0.0));
        assert!(r.clock.measured.is_none(), "not even measured");
        assert_eq!(r.clock.summary(), "crystal correction off");
        // Switched on at run time, it measures from then on.
        r.clock.command(&mut r.sink, "AUTO").unwrap();
        r.run(30.0, 7.0, 0.0);
        assert!((r.sink.ppm - 7.0).abs() < 0.05);
        r.clock.command(&mut r.sink, "off").unwrap();
        assert_eq!((r.clock.ppm, r.sink.ppm, r.clock.rate_ratio()), (0.0, 0.0, 1.0));
        r.run(30.0, 7.0, 0.0);
        assert_eq!(r.sink.ppm, 0.0);
    }
}
