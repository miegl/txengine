//! The NFM baseband generator: the carrier's frequency deviation, voice and
//! sub-audio signalling together.
//!
//! ```text
//!               fa ≈ 16 kHz                                                                fs (output rate)
//! L,R ─► mono ─► gain ─► high-pass ─► pre-emphasis ─► low-pass ─► ×L interpolator ─► peak limiter ─► ×Dv ─┬─► out (Hz)
//!                                                                      CTCSS tone or DCS code ─► ×Ds ─┘
//! ```
//!
//! The voice band is 300 Hz up to the audio bandwidth (3 kHz by default),
//! as for analogue land mobile, PMR446, CB and amateur radio. The high-pass
//! (elliptic, 8th order) passes it from 300 Hz within 0.1 dB and takes
//! 60 dB off everything from 254 Hz down, where the sub-audio signalling
//! lives. The pre-emphasis rises with 6 dB per octave above the corner of
//! its time constant (750 µs: 212 Hz) and is normalised to 1 at 1 kHz, the
//! reference frequency of these services: a full-scale 1 kHz tone deviates
//! the full voice deviation `Dv`, and receivers de-emphasise it back. The
//! low-pass at the bandwidth (60 dB from 10 % above it) keeps the signal in
//! its channel, and the limiter keeps the true peaks of the voice within
//! `Dv` (look-ahead, so it neither clips nor splatters into neighbouring
//! channels).
//!
//! The sub-audio signal is added after the limiter at its own deviation
//! `Ds` (15 % of the total by default, as usual), and the voice gets what
//! is left: `Dv = D − Ds`, so the total never exceeds the deviation `D`.

use txengine_audio::AudioFeed;
use txengine_dsp::{Biquads, FirDesign, LinearPhaseFir, PeakLimiter, PolyphaseInterpolator, Preemphasis};

use crate::subaudio::{Subaudio, SubaudioGenerator};

/// Output samples per processing block.
const BLOCK: usize = 512;

/// Look-ahead and release of the limiter (s).
const LOOKAHEAD: f64 = 0.0015;
const RELEASE: f64 = 0.08;

/// Lowest audio processing rate (Hz): room for the widest bandwidth and its
/// low-pass's transition band, and images of it far enough out for the
/// interpolation filter.
const MIN_AUDIO_RATE: f64 = 16_000.0;

/// Lower edge of the voice band (Hz).
pub const VOICE_LOW_HZ: f64 = 300.0;
/// Where the pre-emphasis has unity gain (Hz).
pub const PREEMPHASIS_REFERENCE_HZ: f64 = 1_000.0;
/// The usual pre-emphasis time constant of narrow-band FM (s).
pub const DEFAULT_PREEMPHASIS: f64 = 750e-6;

/// Largest and smallest deviation (Hz).
pub const MAX_DEVIATION: f64 = 10_000.0;
pub const MIN_DEVIATION: f64 = 500.0;
/// Largest and smallest audio bandwidth (Hz).
pub const MAX_BANDWIDTH: f64 = 5_000.0;
pub const MIN_BANDWIDTH: f64 = 2_000.0;
/// The sub-audio signal's usual share of the deviation.
pub const SUBAUDIO_SHARE: f64 = 0.15;

/// Configuration of the NFM modulation.
#[derive(Debug, Clone)]
pub struct NfmConfig {
    /// Peak frequency deviation of voice and sub-audio together (Hz).
    pub deviation: f64,
    /// Highest audio frequency transmitted (Hz).
    pub bandwidth: f64,
    /// Pre-emphasis time constant (s); `None`: flat.
    pub preemphasis: Option<f64>,
    pub subaudio: Subaudio,
    /// Peak deviation of the sub-audio signal (Hz).
    pub subaudio_deviation: f64,
    /// Input gain in dB.
    pub gain_db: f64,
    /// Look-ahead peak limiter (without it, overload is clipped).
    pub limiter: bool,
}

impl Default for NfmConfig {
    fn default() -> Self {
        NfmConfig {
            deviation: 2_500.0,
            bandwidth: 3_000.0,
            preemphasis: Some(DEFAULT_PREEMPHASIS),
            subaudio: Subaudio::Off,
            subaudio_deviation: SUBAUDIO_SHARE * 2_500.0,
            gain_db: 0.0,
            limiter: true,
        }
    }
}

impl NfmConfig {
    pub fn check(&self) -> Result<(), String> {
        if !(MIN_DEVIATION..=MAX_DEVIATION).contains(&self.deviation) {
            return Err(format!("the deviation must be {MIN_DEVIATION:.0}-{MAX_DEVIATION:.0} Hz"));
        }
        if !(MIN_BANDWIDTH..=MAX_BANDWIDTH).contains(&self.bandwidth) {
            return Err(format!("the audio bandwidth must be {MIN_BANDWIDTH:.0}-{MAX_BANDWIDTH:.0} Hz"));
        }
        if self.preemphasis.is_some_and(|t| !(50e-6..=2e-3).contains(&t)) {
            return Err("the pre-emphasis time constant must be 50-2000 µs".into());
        }
        self.subaudio.check()?;
        if !(self.subaudio_deviation > 0.0 && self.subaudio_deviation <= 0.3 * self.deviation) {
            return Err("the sub-audio deviation must be above 0 and at most 30 % of the deviation".into());
        }
        if !(-60.0..=30.0).contains(&self.gain_db) {
            return Err("the gain must be -60 to +30 dB".into());
        }
        Ok(())
    }

    /// Peak deviation of the voice (Hz): what the sub-audio signal leaves.
    pub fn voice_deviation(&self) -> f64 {
        match self.subaudio {
            Subaudio::Off => self.deviation,
            _ => self.deviation - self.subaudio_deviation,
        }
    }
}

/// Live monitoring values.
#[derive(Debug, Clone, Copy)]
pub struct NfmStats {
    /// Largest deviation since the last reset (Hz).
    pub peak_deviation: f64,
    /// Lowest limiter gain since the last reset (1 = no limiting).
    pub limiter_gain: f32,
    /// Samples clipped since start (without the limiter).
    pub clipped: u64,
    /// Audio frames that had to be substituted with silence since start.
    pub audio_underruns: u64,
}

impl Default for NfmStats {
    fn default() -> Self {
        NfmStats { peak_deviation: 0.0, limiter_gain: 1.0, clipped: 0, audio_underruns: 0 }
    }
}

/// Turns programme audio into the frequency deviation of an NFM carrier
/// (see the module documentation).
pub struct BasebandGenerator {
    cfg: NfmConfig,
    /// The nominal output rate, and the rate the samples actually leave at.
    fs: f64,
    fs_actual: f64,
    factor: usize,
    fa: f64,
    /// Input gain, including the pre-emphasis's normalisation to 1 kHz.
    gain: f32,
    highpass: Biquads,
    preemphasis: Option<Preemphasis>,
    lp_taps: Vec<f64>,
    lpf: LinearPhaseFir,
    interp: PolyphaseInterpolator,
    limiter: PeakLimiter,
    subaudio: SubaudioGenerator,
    /// Audio frames read for a block, and their processed mono programme.
    frames: Vec<[f32; 2]>,
    mono: Vec<f32>,
    /// One interpolated frame before limiting.
    frame: Vec<f32>,
    /// The voice at the output rate, limited to ±1; `have` samples are
    /// valid.
    x: Vec<f32>,
    have: usize,
    stats: NfmStats,
}

impl std::fmt::Debug for BasebandGenerator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "BasebandGenerator(fs {} Hz, fa {} Hz)", self.fs, self.fa)
    }
}

impl BasebandGenerator {
    /// Audio processing rate for an output rate of `fs`: the highest
    /// integer fraction of it that is at least 16 kHz.
    pub fn audio_rate_for(fs: f64) -> f64 {
        fs / (fs / MIN_AUDIO_RATE).floor().max(1.0)
    }

    /// `fs`: the output rate.
    pub fn new(cfg: NfmConfig, fs: f64) -> Result<BasebandGenerator, String> {
        cfg.check()?;
        if fs < MIN_AUDIO_RATE {
            return Err(format!("sample rate {fs} Hz is too low for NFM (need >= {MIN_AUDIO_RATE} Hz)"));
        }
        let factor = (fs / MIN_AUDIO_RATE).floor() as usize;
        let fa = fs / factor as f64;
        let pass = cfg.bandwidth;
        let stop = (1.1 * pass).min(fa / 2.0);
        let lp = FirDesign::lowpass(fa, pass, stop, 60.0);
        // The images of the fa-rate signal start at fa - bandwidth; they
        // would widen the signal beyond its channel.
        let ip = if factor > 1 { FirDesign::lowpass(fs, stop, fa - stop, 80.0) } else { vec![1.0] };
        let preemphasis = cfg.preemphasis.map(|tau| Preemphasis::new(tau, fa, pass));
        let mut g = BasebandGenerator {
            gain: 0.0,
            highpass: Biquads::elliptic_highpass(fa, VOICE_LOW_HZ),
            preemphasis,
            lpf: LinearPhaseFir::new(&lp),
            lp_taps: lp,
            interp: PolyphaseInterpolator::new(factor, &ip),
            limiter: PeakLimiter::new(factor, fa, 1.0, LOOKAHEAD, RELEASE),
            subaudio: SubaudioGenerator::new(cfg.subaudio, fs),
            frames: vec![[0.0; 2]; BLOCK.div_ceil(factor)],
            mono: vec![0.0; BLOCK.div_ceil(factor)],
            frame: vec![0.0; factor],
            x: vec![0.0; BLOCK + factor],
            have: 0,
            stats: NfmStats::default(),
            cfg,
            fs,
            fs_actual: fs,
            factor,
            fa,
        };
        g.set_gain_db(g.cfg.gain_db);
        Ok(g)
    }

    pub fn config(&self) -> &NfmConfig {
        &self.cfg
    }

    /// The rate the programme must be supplied at.
    pub fn audio_rate(&self) -> f64 {
        self.fa
    }

    /// Gain of the voice path at `f` Hz relative to 1 kHz (high-pass,
    /// pre-emphasis and low-pass), for checks and messages.
    pub fn voice_response(&self, f: f64) -> f64 {
        let gain = |f: f64| {
            self.highpass.gain(f, self.fa)
                * self.preemphasis.as_ref().map_or(1.0, |p| p.gain_at(f, self.fa))
                * FirDesign::response(&self.lp_taps, f, self.fa).0
        };
        gain(f) / gain(PREEMPHASIS_REFERENCE_HZ)
    }

    pub fn set_gain_db(&mut self, db: f64) {
        self.cfg.gain_db = db;
        let reference = self.preemphasis.as_ref().map_or(1.0, |p| p.gain_at(PREEMPHASIS_REFERENCE_HZ, self.fa));
        self.gain = (txengine_dsp::from_db(db) / reference) as f32;
    }

    /// Change the sub-audio signal and its deviation (Hz); the voice gets
    /// the rest of the deviation.
    pub fn set_subaudio(&mut self, what: Subaudio, deviation: f64) -> Result<(), String> {
        let cfg = NfmConfig { subaudio: what, subaudio_deviation: deviation, ..self.cfg.clone() };
        cfg.check()?;
        self.cfg = cfg;
        self.subaudio = SubaudioGenerator::new(what, self.fs_actual);
        Ok(())
    }

    /// The samples now leave at `fs` Hz (a crystal error the output does not
    /// correct): keep the sub-audio tone and bit rate true.
    pub fn set_output_rate(&mut self, fs: f64) {
        self.fs_actual = fs;
        self.subaudio.set_rate(fs);
    }

    /// Statistics; `reset_peaks` clears the peak meters.
    pub fn stats(&mut self, reset_peaks: bool) -> NfmStats {
        let s = self.stats;
        if reset_peaks {
            self.stats.peak_deviation = 0.0;
            self.stats.limiter_gain = 1.0;
        }
        s
    }

    /// Make at least `n` samples of the limited voice available.
    fn produce(&mut self, n: usize, audio: &mut dyn AudioFeed) {
        let f = self.factor;
        let frames = n.saturating_sub(self.have).div_ceil(f);
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
        let mono = &mut self.mono[..frames];
        let g = self.gain;
        for (&[l, r], m) in self.frames[..frames].iter().zip(mono.iter_mut()) {
            *m = 0.5 * (l + r) * g;
        }
        self.highpass.process(mono);
        if let Some(p) = self.preemphasis.as_mut() {
            for m in mono.iter_mut() {
                *m = p.process(0, *m);
            }
        }
        self.lpf.process(mono);
        for k in 0..frames {
            let at = self.have;
            let out = &mut self.x[at..at + f];
            if self.cfg.limiter {
                self.interp.process(self.mono[k], &mut self.frame);
                self.limiter.process(&self.frame, out);
            } else {
                self.interp.process(self.mono[k], out);
            }
            self.have += f;
        }
    }

    /// Render `out.len()` samples of the deviation (Hz).
    pub fn render(&mut self, out: &mut [f32], audio: &mut dyn AudioFeed) {
        for block in out.chunks_mut(BLOCK) {
            self.render_block(block, audio);
        }
        if self.cfg.limiter {
            self.stats.limiter_gain = self.stats.limiter_gain.min(self.limiter.take_min_gain());
        }
    }

    fn render_block(&mut self, out: &mut [f32], audio: &mut dyn AudioFeed) {
        let n = out.len();
        self.produce(n, audio);
        let dv = self.cfg.voice_deviation() as f32;
        // The limiter, if on, has already brought every sample within ±1,
        // up to rounding.
        for (o, &x) in out.iter_mut().zip(&self.x[..n]) {
            *o = dv * txengine_dsp::fmin(txengine_dsp::fmax(x, -1.0), 1.0);
        }
        if !self.cfg.limiter {
            self.stats.clipped += self.x[..n].iter().filter(|x| x.abs() > 1.0).count() as u64;
        }
        self.subaudio.add(out, self.cfg.subaudio_deviation as f32);
        let peak = out.iter().fold(0f32, |m, &v| txengine_dsp::fmax(m, v.abs()));
        self.stats.peak_deviation = self.stats.peak_deviation.max(peak as f64);
        self.x.copy_within(n..self.have, 0);
        self.have -= n;
    }
}
