//! FM stereo multiplex generator (ITU-R BS.450 pilot-tone system + RDS).
//!
//! ```text
//!            fa ≈ 48 kHz                                         fs (hardware rate)
//! L,R ─► gain ─► 15 kHz LPF ─┬───────────────────► M,S ─► ×L interpolator ─┐ flat
//!                            └─► pre-emphasis − 1 ─► M,S ─► ×L interpolator ─┤ boost
//!                                                                            ▼
//!                                  HF and peak limiter: g·(flat + h·boost) ─┐
//!                                                                            ▼
//!   deviation = A·(M + S·sin 2φ) + P·sin φ + R·rds(t)·sin 3φ ─► 1/sinc ─► clip ─► out
//! ```
//!
//! Levels: full scale is the audio deviation, but the pre-emphasis raises
//! the multiplex power that ITU-R BS.412 limits (to 0 dBr, the power of a
//! ±19 kHz sine). The gain therefore includes a drive correction: minus the
//! rise of the power of pink noise, a spectrum programme material rarely
//! exceeds in brightness (4.3 dB at 50 µs, 6.8 dB at 75 µs). A programme no
//! brighter than pink noise is then transmitted at about, and not above, the
//! multiplex power it would have without pre-emphasis.
//!
//! The limiter (see [`PreemphasisLimiter`]) gets the flat programme and the
//! boost the pre-emphasis adds separately, interpolated to the output rate,
//! and measures `|M| + |S|` there, which is exactly the peak of the audio
//! part of the composite (`M + S·sin 2φ` interpolates between `L` and `R`):
//! audio, pilot and RDS together never exceed the maximum deviation,
//! including inter-sample peaks. A high-frequency overload is resolved by
//! reducing the boost alone (the HF gain `h`), so the treble drops for a
//! moment instead of the whole programme; the whole programme (gain `g`)
//! is only reduced when the flat programme itself is over full scale. The
//! gains are computed once per audio-rate frame and interpolated across the
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

use txengine_dsp::{
    FirDesign, LinearPhaseFir, PhaseAccumulator, PolyphaseInterpolator, Preemphasis, PreemphasisLimiter, SineTable,
};
use txengine_rds::{RdsEncoder, RdsModulator};

/// Output samples per processing block.
const BLOCK: usize = 512;

/// ITU-R BS.412 reference: 0 dBr is the power of a sine of ±19 kHz deviation (Hz²).
const MPX_POWER_REF: f64 = 19_000.0 * 19_000.0 / 2.0;

/// Look-ahead of the limiters (s).
const LOOKAHEAD: f64 = 0.0015;

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
    /// Input gain in dB, on top of the pre-emphasis drive correction (see
    /// [`MpxGenerator::drive_db`]).
    pub gain_db: f64,
    /// HF limiter and look-ahead peak limiter (without them, overload is
    /// clipped).
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
    /// Lowest peak limiter gain since the last reset (1.0 = no limiting).
    pub limiter_gain: f32,
    /// Largest reduction by the HF limiter since the last reset, in dB at
    /// the audio cut-off frequency (0 = none).
    pub hf_limiter_db: f64,
    /// Multiplex power over the last 60 s (or since the start), in dBr
    /// (ITU-R BS.412).
    pub mpx_power_dbr: f64,
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

/// Multiplex power over the last 60 s (ITU-R BS.412), kept in buckets of
/// about one second.
#[derive(Debug, Clone)]
struct PowerMeter {
    bucket_len: u64,
    /// Energy (Hz²·samples) and length of the bucket being filled.
    energy: f64,
    n: u64,
    /// Completed buckets: (energy, samples).
    ring: Vec<(f64, u64)>,
    pos: usize,
}

impl PowerMeter {
    fn new(fs: f64) -> PowerMeter {
        PowerMeter { bucket_len: fs.round() as u64, energy: 0.0, n: 0, ring: Vec::with_capacity(60), pos: 0 }
    }

    fn add(&mut self, block: &[f32]) {
        self.energy += block.iter().map(|&v| v * v).sum::<f32>() as f64;
        self.n += block.len() as u64;
        if self.n >= self.bucket_len {
            let b = (std::mem::take(&mut self.energy), std::mem::take(&mut self.n));
            if self.ring.len() < 60 {
                self.ring.push(b);
            } else {
                self.ring[self.pos] = b;
                self.pos = (self.pos + 1) % 60;
            }
        }
    }

    /// Power over the window in dBr (the bucket being filled replaces the
    /// oldest one only once the window is full).
    fn dbr(&self) -> f64 {
        let (e, n) = self.ring.iter().fold((self.energy, self.n), |a, b| (a.0 + b.0, a.1 + b.1));
        let (e, n) = if self.ring.len() == 60 { (e - self.ring[self.pos].0, n - self.ring[self.pos].1) } else { (e, n) };
        if n == 0 {
            return f64::NEG_INFINITY;
        }
        10.0 * (e / n as f64 / MPX_POWER_REF).log10()
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
    /// Drive correction for the pre-emphasis (dB, included in `gain`).
    drive_db: f64,
    pre: Option<Preemphasis>,
    lpf: LinearPhaseFir,
    limiter: PreemphasisLimiter,
    /// Interpolators for the flat M and S and their pre-emphasis boost.
    interp: [PolyphaseInterpolator; 4],
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
    /// Pre-emphasis boost of `left` / `right` (when limiting).
    boost: [Vec<f32>; 2],
    /// Lowest fraction of the pre-emphasis boost the HF limiter passed.
    hf_min_gain: f32,
    power: PowerMeter,
    /// Limited M and S at the output rate; `have` samples are valid.
    m: Vec<f32>,
    s: Vec<f32>,
    have: usize,
    /// One interpolated frame before limiting: flat M, S, boost M, S.
    frame: [Vec<f32>; 4],
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
        let pre = cfg.preemphasis.map(|tau| Preemphasis::new(tau, fa, cfg.cutoff));
        let drive_db = pre.as_ref().map_or(0.0, |p| -p.pink_power_gain_db(fa, 20.0, cfg.cutoff));
        // Leave ~1 % for interpolation overshoot of the audio.
        let a_dev = cfg.audio_deviation.unwrap_or((cfg.max_deviation - pilot - rds_peak) * 0.99);
        if a_dev <= 0.0 {
            return Err("no deviation left for audio after pilot and RDS".into());
        }

        // 15 kHz programme low-pass: ≥ 80 dB from 18.5 kHz (protects the pilot).
        let stop = (cfg.cutoff + 3_500.0).min(fa / 2.0);
        let lp = FirDesign::lowpass(fa, cfg.cutoff, stop, 80.0);
        // Interpolation filter: images of the fa-rate signal start at
        // fa - cutoff, on the stereo subband and the RDS channel. A stereo
        // receiver demodulates the image of a tone at f, at fa - f, to
        // |fa - 38 kHz - f| (e.g. 13 kHz -> 1 kHz at fa = 50 kHz), where the
        // de-emphasis does not take back the up to 13.7 dB the
        // pre-emphasis gave the tone: suppress them by ≥ 120 dB, so that
        // these products stay ~105 dB or more below the programme.
        let ip = FirDesign::lowpass(fs, cfg.cutoff + 500.0, fa - cfg.cutoff - 500.0, 120.0);
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
            gain: txengine_dsp::from_db(cfg.gain_db + drive_db) as f32,
            drive_db,
            pre,
            lpf: LinearPhaseFir::new(&lp),
            limiter: PreemphasisLimiter::new(factor, fa, 1.0, LOOKAHEAD, 0.05, 0.08),
            interp: std::array::from_fn(|_| PolyphaseInterpolator::new(factor, &ip)),
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
            boost: [vec![0.0; BLOCK.div_ceil(factor)], vec![0.0; BLOCK.div_ceil(factor)]],
            hf_min_gain: 1.0,
            power: PowerMeter::new(fs),
            m: vec![0.0; BLOCK + factor],
            s: vec![0.0; BLOCK + factor],
            have: 0,
            frame: std::array::from_fn(|_| vec![0.0; factor]),
            rds_bb: vec![0.0; BLOCK],
            stats: MpxStats { limiter_gain: 1.0, mpx_power_dbr: f64::NEG_INFINITY, ..Default::default() },
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

    /// Drive correction for the pre-emphasis in dB (0 without it): the
    /// level is lowered by the rise of the multiplex power the pre-emphasis
    /// causes for pink noise. The gain applies on top of it.
    pub fn drive_db(&self) -> f64 {
        self.drive_db
    }

    pub fn set_gain_db(&mut self, db: f64) {
        self.cfg.gain_db = db;
        self.gain = txengine_dsp::from_db(db + self.drive_db) as f32;
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
        let mut s = self.stats;
        if let Some(p) = &self.pre {
            let (k, f) = (self.hf_min_gain as f64, self.cfg.cutoff);
            s.hf_limiter_db = p.partial_gain_db(1.0, f, self.fa) - p.partial_gain_db(k, f, self.fa);
        }
        s.mpx_power_dbr = self.power.dbr();
        if reset_peaks {
            self.stats.peak_deviation = 0.0;
            self.stats.limiter_gain = 1.0;
            self.hf_min_gain = 1.0;
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
        // Gain, the programme low-pass and pre-emphasis, block-wise. With
        // the limiter, the flat signal and the boost of the pre-emphasis
        // stay apart (the limiter reduces the boost first). Both filters
        // are linear, so filtering before the pre-emphasis changes nothing.
        let (left, right) = (&mut self.left[..frames], &mut self.right[..frames]);
        let [bl, br] = &mut self.boost;
        let (bl, br) = (&mut bl[..frames], &mut br[..frames]);
        let g = self.gain;
        let split = self.cfg.limiter;
        if self.stereo {
            for ((&[l, r], lo), ro) in self.frames[..frames].iter().zip(left.iter_mut()).zip(right.iter_mut()) {
                (*lo, *ro) = (l * g, r * g);
            }
            self.lpf.process2(left, right);
            if let Some(p) = self.pre.as_mut() {
                if split {
                    for (((&l, &r), dl), dr) in left.iter().zip(right.iter()).zip(bl.iter_mut()).zip(br.iter_mut()) {
                        let (el, er) = (p.process(0, l) - l, p.process(1, r) - r);
                        (*dl, *dr) = (0.5 * (el + er), 0.5 * (el - er));
                    }
                } else {
                    for (l, r) in left.iter_mut().zip(right.iter_mut()) {
                        *l = p.process(0, *l);
                        *r = p.process(1, *r);
                    }
                }
            }
            for (l, r) in left.iter_mut().zip(right.iter_mut()) {
                (*l, *r) = (0.5 * (*l + *r), 0.5 * (*l - *r));
            }
        } else {
            for (&[l, r], m) in self.frames[..frames].iter().zip(left.iter_mut()) {
                *m = 0.5 * (l + r) * g;
            }
            self.lpf.process(left);
            if let Some(p) = self.pre.as_mut() {
                if split {
                    for (&m, d) in left.iter().zip(bl.iter_mut()) {
                        *d = p.process(0, m) - m;
                    }
                } else {
                    for m in left.iter_mut() {
                        *m = p.process(0, *m);
                    }
                }
            }
        }

        // Interpolation and limiting, frame by frame. Channels that are
        // not used (S in mono, the boost without pre-emphasis) stay zero.
        let boost = split && self.pre.is_some();
        for k in 0..frames {
            let at = self.have;
            let (om, os) = (&mut self.m[at..at + f], &mut self.s[at..at + f]);
            if self.cfg.limiter {
                let [fm, fsd, fdm, fds] = &mut self.frame;
                self.interp[0].process(self.left[k], fm);
                if self.stereo {
                    self.interp[1].process(self.right[k], fsd);
                }
                if boost {
                    self.interp[2].process(self.boost[0][k], fdm);
                    if self.stereo {
                        self.interp[3].process(self.boost[1][k], fds);
                    }
                }
                self.limiter.process(fm, fsd, fdm, fds, om, os);
            } else {
                self.interp[0].process(self.left[k], om);
                if self.stereo {
                    self.interp[1].process(self.right[k], os);
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
            let (h, g) = self.limiter.take_min_gains();
            self.stats.limiter_gain = self.stats.limiter_gain.min(g);
            self.hf_min_gain = self.hf_min_gain.min(h);
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
        self.power.add(out);
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
