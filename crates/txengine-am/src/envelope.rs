//! The AM envelope generator.
//!
//! ```text
//!               fa ≈ 25 kHz                                         fs (output rate)
//! L,R ─► mono ─► DC block ─► gain ─► low-pass ─► ×L interpolator ─► peak limiter ─► c·(1 + m·x) ─► out
//! ```
//!
//! The output is the carrier's envelope ([`SampleFormat::Amplitude`]): 1 is
//! the strongest carrier the transmitter makes. With the modulation depth
//! `m`, the unmodulated carrier sits at `c = 1 / (1 + m)`, so that the
//! programme's positive peaks reach the transmitter's full output and its
//! negative peaks `c·(1 - m)`: no peak is lost to the transmitter, and the
//! carrier is never cut off (which would splatter over the neighbouring
//! channels).
//!
//! The low-pass at the audio bandwidth keeps the sidebands inside the
//! channel (±4.5 kHz in the 9 kHz raster of ITU regions 1 and 3). The
//! limiter works at the output rate, on true peaks, like the WFM mode's
//! (see [`PeakLimiter`]).
//!
//! [`SampleFormat::Amplitude`]: txengine_core::SampleFormat::Amplitude

use txengine_audio::AudioFeed;
use txengine_dsp::{FirDesign, LinearPhaseFir, PeakLimiter, PolyphaseInterpolator};

/// Output samples per processing block.
const BLOCK: usize = 512;

/// Look-ahead and release of the limiter (s).
const LOOKAHEAD: f64 = 0.0015;
const RELEASE: f64 = 0.08;

/// Corner of the DC blocker (Hz): a DC offset in the programme would move
/// the carrier level and take headroom from one side of the modulation.
const DC_BLOCK_HZ: f64 = 10.0;

/// Lowest audio processing rate (Hz): room for a 10 kHz bandwidth and its
/// low-pass's transition band.
const MIN_AUDIO_RATE: f64 = 24_000.0;

/// Largest and smallest audio bandwidth (Hz).
pub const MAX_BANDWIDTH: f64 = 10_000.0;
pub const MIN_BANDWIDTH: f64 = 2_500.0;

/// Configuration of the AM modulation.
#[derive(Debug, Clone)]
pub struct AmConfig {
    /// Highest audio frequency transmitted (Hz): the low-pass's pass band.
    /// The occupied bandwidth is twice this.
    pub bandwidth: f64,
    /// Modulation depth of the programme's peaks (0 < m ≤ 1).
    pub depth: f64,
    /// Input gain in dB.
    pub gain_db: f64,
    /// Look-ahead peak limiter (without it, overload is clipped).
    pub limiter: bool,
}

impl Default for AmConfig {
    fn default() -> Self {
        AmConfig { bandwidth: 4_500.0, depth: 0.9, gain_db: 0.0, limiter: true }
    }
}

impl AmConfig {
    pub fn check(&self) -> Result<(), String> {
        if !(MIN_BANDWIDTH..=MAX_BANDWIDTH).contains(&self.bandwidth) {
            return Err(format!("the audio bandwidth must be {MIN_BANDWIDTH:.0}-{MAX_BANDWIDTH:.0} Hz"));
        }
        if !(self.depth > 0.0 && self.depth <= 1.0) {
            return Err("the modulation depth must be above 0 and at most 100 %".into());
        }
        if !(-60.0..=30.0).contains(&self.gain_db) {
            return Err("the gain must be -60 to +30 dB".into());
        }
        Ok(())
    }

    /// Carrier level (re the transmitter's full output) for the depth.
    pub fn carrier(&self) -> f64 {
        1.0 / (1.0 + self.depth)
    }
}

/// Live monitoring values.
#[derive(Debug, Clone, Copy)]
pub struct AmStats {
    /// Largest positive and negative modulation since the last reset
    /// (fractions of 100 %; the negative one as a positive number).
    pub peak_positive: f64,
    pub peak_negative: f64,
    /// Lowest limiter gain since the last reset (1 = no limiting).
    pub limiter_gain: f32,
    /// Samples clipped since start (without the limiter).
    pub clipped: u64,
    /// Audio frames that had to be substituted with silence since start.
    pub audio_underruns: u64,
}

impl Default for AmStats {
    fn default() -> Self {
        AmStats { peak_positive: 0.0, peak_negative: 0.0, limiter_gain: 1.0, clipped: 0, audio_underruns: 0 }
    }
}

/// Turns programme audio into the envelope of an AM carrier (see the module
/// documentation).
pub struct EnvelopeGenerator {
    cfg: AmConfig,
    fs: f64,
    factor: usize,
    fa: f64,
    gain: f32,
    /// DC blocker state: pole, previous input and output.
    dc_pole: f32,
    dc_x: f32,
    dc_y: f32,
    lpf: LinearPhaseFir,
    interp: PolyphaseInterpolator,
    limiter: PeakLimiter,
    /// Audio frames read for a block, and their processed mono programme.
    frames: Vec<[f32; 2]>,
    mono: Vec<f32>,
    /// One interpolated frame before limiting.
    frame: Vec<f32>,
    /// The programme at the output rate, limited to ±1; `have` samples are
    /// valid.
    x: Vec<f32>,
    have: usize,
    stats: AmStats,
}

impl std::fmt::Debug for EnvelopeGenerator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "EnvelopeGenerator(fs {} Hz, fa {} Hz)", self.fs, self.fa)
    }
}

impl EnvelopeGenerator {
    /// Audio processing rate for an output rate of `fs`: the highest
    /// integer fraction of it that is at least 24 kHz.
    pub fn audio_rate_for(fs: f64) -> f64 {
        fs / (fs / MIN_AUDIO_RATE).floor().max(1.0)
    }

    /// `fs`: the output rate.
    pub fn new(cfg: AmConfig, fs: f64) -> Result<EnvelopeGenerator, String> {
        cfg.check()?;
        if fs < MIN_AUDIO_RATE {
            return Err(format!("sample rate {fs} Hz is too low for AM (need >= {MIN_AUDIO_RATE} Hz)"));
        }
        let factor = (fs / MIN_AUDIO_RATE).floor() as usize;
        let fa = fs / factor as f64;
        // The programme low-pass: the pass band up to the bandwidth, 60 dB
        // from 10 % above it (keeps the sidebands in the channel).
        let pass = cfg.bandwidth;
        let stop = (1.1 * pass).min(fa / 2.0);
        let lp = FirDesign::lowpass(fa, pass, stop, 60.0);
        // Interpolation filter: the images of the fa-rate signal start at
        // fa - bandwidth.
        let ip = if factor > 1 { FirDesign::lowpass(fs, stop, fa - stop, 80.0) } else { vec![1.0] };
        Ok(EnvelopeGenerator {
            gain: txengine_dsp::from_db(cfg.gain_db) as f32,
            dc_pole: (-2.0 * std::f64::consts::PI * DC_BLOCK_HZ / fa).exp() as f32,
            dc_x: 0.0,
            dc_y: 0.0,
            lpf: LinearPhaseFir::new(&lp),
            interp: PolyphaseInterpolator::new(factor, &ip),
            limiter: PeakLimiter::new(factor, fa, 1.0, LOOKAHEAD, RELEASE),
            frames: vec![[0.0; 2]; BLOCK.div_ceil(factor)],
            mono: vec![0.0; BLOCK.div_ceil(factor)],
            frame: vec![0.0; factor],
            x: vec![0.0; BLOCK + factor],
            have: 0,
            stats: AmStats::default(),
            cfg,
            fs,
            factor,
            fa,
        })
    }

    pub fn config(&self) -> &AmConfig {
        &self.cfg
    }

    /// The rate the programme must be supplied at.
    pub fn audio_rate(&self) -> f64 {
        self.fa
    }

    pub fn set_gain_db(&mut self, db: f64) {
        self.cfg.gain_db = db;
        self.gain = txengine_dsp::from_db(db) as f32;
    }

    pub fn set_depth(&mut self, depth: f64) {
        self.cfg.depth = depth;
    }

    /// Statistics; `reset_peaks` clears the peak meters.
    pub fn stats(&mut self, reset_peaks: bool) -> AmStats {
        let s = self.stats;
        if reset_peaks {
            self.stats.peak_positive = 0.0;
            self.stats.peak_negative = 0.0;
            self.stats.limiter_gain = 1.0;
        }
        s
    }

    /// Make at least `n` samples of the limited programme available.
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
        let (p, g) = (self.dc_pole, self.gain);
        for (&[l, r], m) in self.frames[..frames].iter().zip(mono.iter_mut()) {
            let x = 0.5 * (l + r) * g;
            self.dc_y = x - self.dc_x + p * self.dc_y;
            self.dc_x = x;
            *m = self.dc_y;
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

    /// Render `out.len()` samples of the envelope (0-1).
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
        let c = self.cfg.carrier() as f32;
        let cm = (self.cfg.carrier() * self.cfg.depth) as f32;
        // Peak meter and clip on ±1 (the limiter, if on, has already
        // brought every sample within, up to rounding).
        let (mut hi, mut lo) = (0f32, 0f32);
        for (o, &x) in out.iter_mut().zip(&self.x[..n]) {
            hi = txengine_dsp::fmax(hi, x);
            lo = txengine_dsp::fmin(lo, x);
            *o = c + cm * txengine_dsp::fmin(txengine_dsp::fmax(x, -1.0), 1.0);
        }
        if !self.cfg.limiter {
            self.stats.clipped += self.x[..n].iter().filter(|x| x.abs() > 1.0).count() as u64;
        }
        let m = self.cfg.depth;
        self.stats.peak_positive = self.stats.peak_positive.max(m * hi.min(1.0) as f64);
        self.stats.peak_negative = self.stats.peak_negative.max(-m * lo.max(-1.0) as f64);
        self.x.copy_within(n..self.have, 0);
        self.have -= n;
    }
}
