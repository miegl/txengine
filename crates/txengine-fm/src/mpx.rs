//! FM stereo multiplex generator (ITU-R BS.450 pilot-tone system + RDS).
//!
//! ```text
//!            fa ≈ 48 kHz                                         fs (hardware rate)
//! L,R ─► gain ─► pre-emphasis ─► 15 kHz LPF ─► M,S ─► ×L interpolator ─► limiter ─┐
//!                                                                                  ▼
//!   deviation = A·(M + S·sin 2φ) + P·sin φ + R·rds(t)·sin 3φ ─► 1/sinc ─► clip ─► out
//! ```
//!
//! The look-ahead limiter measures `|M| + |S|` at the output rate, which is
//! exactly the peak of the audio part of the composite (`M + S·sin 2φ`
//! interpolates between `L` and `R`), so audio, pilot and RDS together
//! never exceed the maximum deviation, including inter-sample peaks. Its
//! gain is computed once per audio-rate frame and interpolated across the
//! frame's output samples.
//!
//! The generator works in blocks, one processing stage at a time, which
//! keeps the inner loops short and branch free: this matters on the
//! in-order cores of the Raspberry Pi 2/3/Zero 2 W.
//!
//! `φ` is a single 19 kHz phase accumulator: the 38 kHz subcarrier, the
//! 57 kHz RDS subcarrier and the RDS bit clock (16 pilot periods per bit) are
//! all derived from it, so their mutual phase is exact. With `φ = 0` at a
//! positive-going pilot zero crossing, `sin 2φ` crosses zero with a positive
//! slope there too, as BS.450 §2.2.2.5 requires.

use txengine_dsp::{FirDesign, FrameLimiter, LinearPhaseFir, PhaseAccumulator, PolyphaseInterpolator, Preemphasis, SineTable};
use txengine_rds::{RdsEncoder, RdsModulator};

/// Output samples per processing block.
const BLOCK: usize = 512;

/// Stereo coding of the multiplex.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StereoMode {
    /// Stereo (pilot-tone system) for stereo sources, mono otherwise.
    Auto,
    Mono,
    Stereo,
}

/// Level and processing configuration of the multiplex. Deviations in Hz.
#[derive(Debug, Clone)]
pub struct MpxConfig {
    /// Peak deviation never to be exceeded (BS.450: 75 kHz, or 50 kHz).
    pub max_deviation: f64,
    /// Peak deviation of the programme audio. `None`: whatever is left of
    /// `max_deviation` after pilot and RDS.
    pub audio_deviation: Option<f64>,
    /// Pilot deviation (BS.450: 8–10 % of the maximum).
    pub pilot_deviation: f64,
    /// RDS injection: peak deviation of the unmodulated reference
    /// (EN 50067 1.3: 1.0–7.5 kHz, 2 kHz recommended).
    pub rds_deviation: f64,
    /// Pre-emphasis time constant in seconds (50 µs Europe, 75 µs Americas), or none.
    pub preemphasis: Option<f64>,
    /// Audio low-pass cut-off (pass-band edge) in Hz.
    pub cutoff: f64,
    pub stereo: StereoMode,
    /// Input gain in dB applied before processing.
    pub gain_db: f64,
    /// Look-ahead peak limiter on the pre-emphasised audio.
    pub limiter: bool,
    /// Compensate the sinc roll-off of a sample-and-hold output stage.
    pub hold_compensation: bool,
    /// Measured response of the output beyond the hold, `(Hz, dB)` points
    /// (see `SinkInfo::response_db`); compensated together with the hold.
    pub output_response_db: Vec<(f64, f64)>,
}

impl Default for MpxConfig {
    fn default() -> Self {
        MpxConfig {
            max_deviation: 75_000.0,
            audio_deviation: None,
            pilot_deviation: 6_750.0,
            rds_deviation: 2_000.0,
            preemphasis: Some(50e-6),
            cutoff: 15_000.0,
            stereo: StereoMode::Auto,
            gain_db: 0.0,
            limiter: true,
            hold_compensation: true,
            output_response_db: Vec::new(),
        }
    }
}

/// Live monitoring values.
#[derive(Debug, Clone, Copy, Default)]
pub struct MpxStats {
    /// Peak absolute deviation since the last reset (Hz).
    pub peak_deviation: f64,
    /// Lowest limiter gain since the last reset (1.0 = no limiting).
    pub limiter_gain: f32,
    /// Samples clipped by the final deviation limiter since start.
    pub clipped: u64,
    /// Audio frames that had to be substituted with silence since start.
    pub audio_underruns: u64,
}

/// Audio supplier for the generator (non-blocking).
pub trait AudioFeed {
    /// Fill `out` with as many frames as are available; return the count.
    fn read(&mut self, out: &mut [[f32; 2]]) -> usize;
    /// The input has ended for good (missing frames are then not underruns).
    fn ended(&self) -> bool {
        false
    }
}

impl AudioFeed for txengine_audio::AudioInput {
    fn read(&mut self, out: &mut [[f32; 2]]) -> usize {
        txengine_audio::AudioInput::read(self, out)
    }
    fn ended(&self) -> bool {
        self.finished()
    }
}

/// Silence.
#[derive(Debug, Clone, Copy, Default)]
pub struct NoAudio;

impl AudioFeed for NoAudio {
    fn read(&mut self, out: &mut [[f32; 2]]) -> usize {
        out.fill([0.0; 2]);
        out.len()
    }
    fn ended(&self) -> bool {
        true
    }
}

/// The multiplex generator.
pub struct MpxGenerator {
    cfg: MpxConfig,
    /// Actual output rate (see [`MpxGenerator::set_output_rate`]).
    fs: f64,
    /// Interpolation factor fs / fa.
    factor: usize,
    fa: f64,
    stereo: bool,
    gain: f32,
    pre: Option<Preemphasis>,
    lpf: LinearPhaseFir,
    limiter: FrameLimiter,
    interp: [PolyphaseInterpolator; 2],
    comp: Option<LinearPhaseFir>,
    pilot: PhaseAccumulator,
    sin: SineTable,
    rds: RdsModulator,
    rds_on: bool,
    // Levels in Hz.
    a_dev: f32,
    p_dev: f32,
    r_dev: f32,
    max_dev: f32,
    /// Audio frames read for a block, and their processed L/R (or mono).
    frames: Vec<[f32; 2]>,
    left: Vec<f32>,
    right: Vec<f32>,
    /// Limited M and S at the output rate; `have` samples are valid.
    m: Vec<f32>,
    s: Vec<f32>,
    have: usize,
    /// One interpolated frame before limiting.
    fm: Vec<f32>,
    fsd: Vec<f32>,
    /// RDS baseband of the block.
    rds_bb: Vec<f32>,
    stats: MpxStats,
}

impl std::fmt::Debug for MpxGenerator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "MpxGenerator(fs {} Hz, fa {} Hz, stereo {})", self.fs, self.fa, self.stereo)
    }
}

impl MpxGenerator {
    /// `fs`: output rate. `source_stereo`: whether the programme is stereo
    /// (for [`StereoMode::Auto`]).
    pub fn new(cfg: MpxConfig, fs: f64, source_stereo: bool) -> Result<MpxGenerator, String> {
        if fs < 2.0 * 62_000.0 {
            return Err(format!("sample rate {fs} Hz is too low for an FM multiplex (need >= 124 kHz)"));
        }
        if !(0.0..=1.0).contains(&(cfg.pilot_deviation / cfg.max_deviation)) {
            return Err("pilot deviation exceeds the maximum deviation".into());
        }
        // Audio processing rate: an integer fraction of fs near 48 kHz.
        let factor = (fs / 48_000.0).round().max(3.0) as usize;
        let fa = fs / factor as f64;
        if cfg.cutoff > 16_500.0 || cfg.cutoff > fa / 2.0 - 3_000.0 {
            return Err(format!("audio cut-off {} Hz too high (maximum 16.5 kHz)", cfg.cutoff));
        }
        let stereo = match cfg.stereo {
            StereoMode::Auto => source_stereo,
            StereoMode::Mono => false,
            StereoMode::Stereo => true,
        };
        let rds = RdsModulator::new();
        let rds_peak = cfg.rds_deviation * rds.peak_factor() as f64;
        let pilot = if stereo { cfg.pilot_deviation } else { 0.0 };
        // Leave ~1 % for interpolation overshoot of the audio.
        let a_dev = cfg.audio_deviation.unwrap_or((cfg.max_deviation - pilot - rds_peak) * 0.99);
        if a_dev <= 0.0 {
            return Err("no deviation left for audio after pilot and RDS".into());
        }

        // 15 kHz programme low-pass: ≥ 80 dB from 18.5 kHz (protects the pilot).
        let stop = (cfg.cutoff + 3_500.0).min(fa / 2.0);
        let lp = FirDesign::lowpass(fa, cfg.cutoff, stop, 80.0);
        // Interpolation filter: images of the fa-rate signal start at
        // fa - cutoff; suppress them by ≥ 90 dB (they would land on the
        // stereo subband and the RDS channel).
        let ip = FirDesign::lowpass(fs, cfg.cutoff + 500.0, fa - cfg.cutoff - 500.0, 90.0);
        // Inverse of the output response (hold, plus the measured hardware
        // response if known) up to the top of the RDS band.
        let comp = cfg.hold_compensation.then(|| {
            let table = &cfg.output_response_db;
            let taps = if table.is_empty() {
                FirDesign::inverse_sinc(9, fs, 62_000.0)
            } else {
                FirDesign::inverse_response(11, fs, 62_000.0, |f| {
                    txengine_dsp::sinc(f / fs) * 10f64.powf(txengine_core::interpolate_db(table, f) / 20.0)
                })
            };
            LinearPhaseFir::new(&taps)
        });

        Ok(MpxGenerator {
            fs,
            factor,
            fa,
            stereo,
            gain: txengine_dsp::from_db(cfg.gain_db) as f32,
            pre: cfg.preemphasis.map(|tau| Preemphasis::new(tau, fa, cfg.cutoff)),
            lpf: LinearPhaseFir::new(&lp),
            limiter: FrameLimiter::new(factor, fa, 1.0, 0.0015, 0.08),
            interp: [PolyphaseInterpolator::new(factor, &ip), PolyphaseInterpolator::new(factor, &ip)],
            comp,
            pilot: PhaseAccumulator::new(19_000.0, fs),
            sin: SineTable::new(),
            rds,
            rds_on: true,
            a_dev: a_dev as f32,
            p_dev: pilot as f32,
            r_dev: cfg.rds_deviation as f32,
            max_dev: cfg.max_deviation as f32,
            frames: vec![[0.0; 2]; BLOCK.div_ceil(factor)],
            left: vec![0.0; BLOCK.div_ceil(factor)],
            right: vec![0.0; BLOCK.div_ceil(factor)],
            m: vec![0.0; BLOCK + factor],
            s: vec![0.0; BLOCK + factor],
            have: 0,
            fm: vec![0.0; factor],
            fsd: vec![0.0; factor],
            rds_bb: vec![0.0; BLOCK],
            stats: MpxStats { limiter_gain: 1.0, ..Default::default() },
            cfg,
        })
    }

    pub fn config(&self) -> &MpxConfig {
        &self.cfg
    }

    pub fn is_stereo(&self) -> bool {
        self.stereo
    }

    /// Internal audio processing rate.
    pub fn audio_rate(&self) -> f64 {
        self.fa
    }

    /// The rate at which the output is actually emitted, in Hz of true
    /// time: the nominal rate corrected for the error of the hardware's
    /// crystal. The pilot, the subcarriers and the RDS bit rate follow it
    /// (phase continuously), so they stay at their true frequencies; the
    /// filters keep their design for the nominal rate, which a few ppm
    /// cannot change measurably.
    pub fn set_output_rate(&mut self, fs: f64) {
        self.fs = fs;
        self.pilot.set_frequency(19_000.0, fs);
    }

    pub fn audio_deviation(&self) -> f64 {
        self.a_dev as f64
    }

    pub fn set_gain_db(&mut self, db: f64) {
        self.cfg.gain_db = db;
        self.gain = txengine_dsp::from_db(db) as f32;
    }

    pub fn set_rds_enabled(&mut self, on: bool) {
        self.rds_on = on;
    }

    pub fn rds_enabled(&self) -> bool {
        self.rds_on
    }

    /// Change the RDS injection level (Hz). Audio headroom is not re-derived.
    pub fn set_rds_deviation(&mut self, hz: f64) {
        self.cfg.rds_deviation = hz;
        self.r_dev = hz as f32;
    }

    /// Statistics; `reset_peaks` clears the peak meters.
    pub fn stats(&mut self, reset_peaks: bool) -> MpxStats {
        let s = self.stats;
        if reset_peaks {
            self.stats.peak_deviation = 0.0;
            self.stats.limiter_gain = 1.0;
        }
        s
    }

    /// Make at least `n` samples of limited M and S available.
    fn produce_audio(&mut self, n: usize, audio: &mut dyn AudioFeed) {
        let f = self.factor;
        let frames = (n.saturating_sub(self.have)).div_ceil(f);
        if frames == 0 {
            return;
        }
        let got = audio.read(&mut self.frames[..frames]);
        if got < frames {
            self.frames[got..frames].fill([0.0; 2]);
            if !audio.ended() {
                self.stats.audio_underruns += (frames - got) as u64;
            }
        }
        // Gain, pre-emphasis and the programme low-pass, block-wise.
        let (left, right) = (&mut self.left[..frames], &mut self.right[..frames]);
        let g = self.gain;
        if self.stereo {
            for ((&[l, r], lo), ro) in self.frames[..frames].iter().zip(left.iter_mut()).zip(right.iter_mut()) {
                (*lo, *ro) = (l * g, r * g);
            }
            if let Some(p) = self.pre.as_mut() {
                for (l, r) in left.iter_mut().zip(right.iter_mut()) {
                    *l = p.process(0, *l);
                    *r = p.process(1, *r);
                }
            }
            self.lpf.process2(left, right);
            for (l, r) in left.iter_mut().zip(right.iter_mut()) {
                (*l, *r) = (0.5 * (*l + *r), 0.5 * (*l - *r));
            }
        } else {
            for (&[l, r], m) in self.frames[..frames].iter().zip(left.iter_mut()) {
                *m = 0.5 * (l + r) * g;
            }
            if let Some(p) = self.pre.as_mut() {
                for m in left.iter_mut() {
                    *m = p.process(0, *m);
                }
            }
            self.lpf.process(left);
        }

        // Interpolation and limiting, frame by frame.
        for k in 0..frames {
            let (m, s) = (self.left[k], self.right[k]);
            let at = self.have;
            let (om, os) = (&mut self.m[at..at + f], &mut self.s[at..at + f]);
            if self.cfg.limiter {
                self.interp[0].process(m, &mut self.fm);
                // Peak via the bit patterns (see `render_block`).
                let mut peak = 0u32;
                if self.stereo {
                    self.interp[1].process(s, &mut self.fsd);
                    for (&a, &b) in self.fm.iter().zip(&self.fsd) {
                        peak = peak.max((a.abs() + b.abs()).to_bits());
                    }
                } else {
                    for &a in &self.fm {
                        peak = peak.max(a.to_bits() & 0x7fff_ffff);
                    }
                }
                let peak = f32::from_bits(peak);
                // max(|M+S|, |M-S|) = |M| + |S|.
                self.limiter.process(peak, &self.fm, &self.fsd, om, os);
            } else {
                self.interp[0].process(m, om);
                if self.stereo {
                    self.interp[1].process(s, os);
                }
            }
            self.have += f;
        }
    }

    /// Render `out.len()` samples of instantaneous frequency deviation (Hz).
    /// `first_sample_time` is the emission time of `out[0]` (Unix seconds,
    /// NaN if unknown), used for the RDS clock-time group.
    pub fn render(
        &mut self,
        out: &mut [f32],
        audio: &mut dyn AudioFeed,
        rds: &mut RdsEncoder,
        first_sample_time: f64,
    ) {
        let dt = 1.0 / self.fs;
        for (k, block) in out.chunks_mut(BLOCK).enumerate() {
            self.render_block(block, audio, rds, first_sample_time + (k * BLOCK) as f64 * dt);
        }
        if self.cfg.limiter {
            self.stats.limiter_gain = self.stats.limiter_gain.min(self.limiter.take_min_gain());
        }
    }

    fn render_block(&mut self, out: &mut [f32], audio: &mut dyn AudioFeed, rds: &mut RdsEncoder, t0: f64) {
        let n = out.len();
        self.produce_audio(n, audio);
        let rds_on = self.rds_on && self.r_dev != 0.0;
        if rds_on {
            self.rds.render(&self.pilot, &mut self.rds_bb[..n], rds, t0, 1.0 / self.fs);
        }

        // Composite. φ is the pilot phase; sin 2φ and sin 3φ follow from
        // sin φ and cos φ.
        let (a, p, r) = (self.a_dev, self.p_dev, self.r_dev);
        let (m, s) = (&self.m[..n], &self.s[..n]);
        let acc = &mut self.pilot;
        let sin = &self.sin;
        match (self.stereo, rds_on) {
            (true, true) => {
                for (((o, &m), &s), &b) in out.iter_mut().zip(m).zip(s).zip(&self.rds_bb[..n]) {
                    let (s1, c1) = sin.sin_cos(acc.harmonic_frac(1));
                    let s2 = 2.0 * s1 * c1;
                    let s3 = s1 * (3.0 - 4.0 * s1 * s1);
                    *o = a * (m + s * s2) + p * s1 + r * b * s3;
                    acc.advance();
                }
            }
            (true, false) => {
                for ((o, &m), &s) in out.iter_mut().zip(m).zip(s) {
                    let (s1, c1) = sin.sin_cos(acc.harmonic_frac(1));
                    *o = a * (m + s * 2.0 * s1 * c1) + p * s1;
                    acc.advance();
                }
            }
            (false, true) => {
                for ((o, &m), &b) in out.iter_mut().zip(m).zip(&self.rds_bb[..n]) {
                    let s1 = sin.sin(acc.harmonic_frac(1));
                    *o = a * m + r * b * s1 * (3.0 - 4.0 * s1 * s1);
                    acc.advance();
                }
            }
            (false, false) => {
                for (o, &m) in out.iter_mut().zip(m) {
                    *o = a * m;
                }
                acc.advance_by(n as u64);
            }
        }
        // Keep the samples beyond this block.
        self.m.copy_within(n..self.have, 0);
        self.s.copy_within(n..self.have, 0);
        self.have -= n;

        if let Some(c) = self.comp.as_mut() {
            c.process(out);
        }
        // Final safety clip and peak meter. The comparisons are done on the
        // bit patterns (ordered like the magnitudes): float compares are
        // slow on 32-bit ARM.
        let max = self.max_dev.to_bits();
        let mut peak = 0u32;
        let mut clipped = 0u32;
        for x in out.iter_mut() {
            let b = x.to_bits();
            let a = b & 0x7fff_ffff;
            if a > max {
                *x = f32::from_bits(max | (b & 0x8000_0000));
                clipped += 1;
            }
            peak = peak.max(a);
        }
        let peak = f32::from_bits(peak.min(max));
        self.stats.clipped += clipped as u64;
        self.stats.peak_deviation = self.stats.peak_deviation.max(peak as f64);
    }
}
