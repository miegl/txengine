//! The NFM baseband: voice level and band, the pre-emphasis, the peak
//! deviation, and the sub-audio signalling.

use txengine_audio::{AudioFeed, NoAudio};
use txengine_nfm::subaudio::{DCS_BIT_RATE, DCS_WORD_BITS};
use txengine_nfm::{BasebandGenerator, DcsCode, NfmConfig, Subaudio};

const FS: f64 = 190_000.0;
const PI2: f64 = 2.0 * std::f64::consts::PI;

/// Sines of amplitude `amp` at the given frequencies (both channels), at `rate`.
struct Tones {
    tones: Vec<(f64, f64)>,
    rate: f64,
    n: u64,
}

impl AudioFeed for Tones {
    fn read(&mut self, out: &mut [[f32; 2]]) -> usize {
        for o in out.iter_mut() {
            let t = self.n as f64 / self.rate;
            let v = self.tones.iter().map(|&(f, a)| a * (PI2 * f * t).sin()).sum::<f64>() as f32;
            *o = [v, v];
            self.n += 1;
        }
        out.len()
    }
}

/// Uniform noise of amplitude `amp`, white.
struct Noise {
    amp: f32,
    state: u32,
}

impl AudioFeed for Noise {
    fn read(&mut self, out: &mut [[f32; 2]]) -> usize {
        for o in out.iter_mut() {
            self.state ^= self.state << 13;
            self.state ^= self.state >> 17;
            self.state ^= self.state << 5;
            let v = self.amp * (self.state as f32 / u32::MAX as f32 * 2.0 - 1.0);
            *o = [v, v];
        }
        out.len()
    }
}

fn tones(t: &[(f64, f64)]) -> Tones {
    Tones { tones: t.to_vec(), rate: BasebandGenerator::audio_rate_for(FS), n: 0 }
}

/// Render `seconds` and drop the first 0.2 s (filters settling).
fn render(cfg: NfmConfig, audio: &mut dyn AudioFeed, seconds: f64) -> (Vec<f32>, BasebandGenerator) {
    let mut g = BasebandGenerator::new(cfg, FS).unwrap();
    let mut out = vec![0f32; (seconds * FS) as usize];
    for chunk in out.chunks_mut(1000) {
        g.render(chunk, audio);
    }
    (out.split_off((0.2 * FS) as usize), g)
}

/// Amplitude of the component at `f` (Hz), Hann-windowed.
fn amplitude(x: &[f32], f: f64) -> f64 {
    let n = x.len();
    let (mut re, mut im, mut wsum) = (0.0, 0.0, 0.0);
    for (k, &v) in x.iter().enumerate() {
        let w = 0.5 - 0.5 * (PI2 * k as f64 / n as f64).cos();
        let p = PI2 * f * k as f64 / FS;
        re += w * v as f64 * p.cos();
        im += w * v as f64 * p.sin();
        wsum += w;
    }
    2.0 * (re * re + im * im).sqrt() / wsum
}

fn db(x: f64) -> f64 {
    20.0 * x.log10()
}

fn peak(x: &[f32]) -> f64 {
    x.iter().fold(0f32, |m, &v| m.max(v.abs())) as f64
}

#[test]
fn a_full_scale_1khz_tone_deviates_the_voice_deviation() {
    for (deviation, amp) in [(2_500.0, 1.0), (2_500.0, 0.5), (5_000.0, 0.5)] {
        let cfg = NfmConfig { deviation, ..Default::default() };
        let (out, _) = render(cfg, &mut tones(&[(1000.0, amp)]), 1.0);
        let got = amplitude(&out, 1000.0);
        assert!((db(got / (amp * deviation))).abs() < 0.05, "{deviation} Hz, amplitude {amp}: {got} Hz");
    }
}

#[test]
fn the_preemphasis_rises_6_db_per_octave() {
    let cfg = NfmConfig::default();
    let tau = cfg.preemphasis.unwrap();
    let analogue = |f: f64| (1.0 + (PI2 * f * tau).powi(2)).sqrt();
    // Quiet enough that the limiter stays out of it.
    for f in [400.0, 700.0, 1500.0, 2000.0, 2700.0] {
        let (out, g) = render(cfg.clone(), &mut tones(&[(f, 0.05)]), 1.0);
        let got = amplitude(&out, f) / (0.05 * cfg.deviation);
        let want = analogue(f) / analogue(1000.0);
        assert!(db(got / want).abs() < 0.1, "{f} Hz: {:.2} dB re the analogue curve", db(got / want));
        assert!(db(g.voice_response(f) / want).abs() < 0.1, "{f} Hz");
    }
    // Flat without it.
    let flat = NfmConfig { preemphasis: None, ..cfg };
    for f in [400.0, 2500.0] {
        let (out, _) = render(flat.clone(), &mut tones(&[(f, 0.5)]), 1.0);
        assert!(db(amplitude(&out, f) / (0.5 * flat.deviation)).abs() < 0.1, "{f} Hz");
    }
}

#[test]
fn only_the_voice_band_is_transmitted() {
    let cfg = NfmConfig::default();
    let reference = 0.05 * cfg.deviation;
    for (f, max_db) in [(100.0, -60.0), (200.0, -60.0), (250.0, -60.0), (3400.0, -59.0), (5000.0, -59.0)] {
        let (out, _) = render(cfg.clone(), &mut tones(&[(f, 0.05)]), 1.0);
        let got = db(amplitude(&out, f) / reference);
        assert!(got < max_db, "{f} Hz: {got:.1} dB");
    }
    // The pass band's edges.
    for f in [300.0, 3000.0] {
        let (out, g) = render(cfg.clone(), &mut tones(&[(f, 0.05)]), 1.0);
        let want = g.voice_response(f);
        assert!(db(amplitude(&out, f) / (reference * want)).abs() < 0.1, "{f} Hz");
        assert!(db(want) > -12.0, "{f} Hz: {} dB", db(want));
    }
}

#[test]
fn the_deviation_is_never_exceeded() {
    for subaudio in [Subaudio::Off, Subaudio::Ctcss(250.3), Subaudio::Dcs(DcsCode::parse("023").unwrap())] {
        let cfg = NfmConfig { subaudio, gain_db: 20.0, ..Default::default() };
        let (out, mut g) = render(cfg.clone(), &mut Noise { amp: 1.0, state: 1 }, 2.0);
        let p = peak(&out);
        assert!(p <= cfg.deviation * (1.0 + 1e-6), "{subaudio}: {p} Hz");
        // and it is used: the limiter holds the voice at its peak.
        assert!(p > 0.97 * cfg.deviation, "{subaudio}: {p} Hz");
        assert!(g.stats(false).limiter_gain < 0.5);
    }
}

#[test]
fn ctcss_tone_and_voice_share_the_deviation() {
    let cfg = NfmConfig { subaudio: Subaudio::Ctcss(88.5), ..Default::default() };
    let (out, _) = render(cfg.clone(), &mut tones(&[(1000.0, 1.0)]), 2.0);
    assert!((amplitude(&out, 88.5) / cfg.subaudio_deviation - 1.0).abs() < 0.002);
    assert!((amplitude(&out, 1000.0) / (cfg.deviation - cfg.subaudio_deviation) - 1.0).abs() < 0.01);
    // A tone of the voice in the sub-audio band is gone.
    let (out, _) = render(cfg.clone(), &mut tones(&[(1000.0, 1.0), (150.0, 1.0)]), 2.0);
    assert!(db(amplitude(&out, 150.0) / cfg.subaudio_deviation) < -60.0);
}

#[test]
fn ctcss_follows_the_actual_sample_rate() {
    // The samples leave 50 ppm slow, so in the nominal timeline (sample
    // index / FS) the tone must run 50 ppm fast to be true when emitted.
    let e = -50e-6;
    let cfg = NfmConfig { subaudio: Subaudio::Ctcss(100.0), ..Default::default() };
    let mut g = BasebandGenerator::new(cfg, FS).unwrap();
    g.set_output_rate(FS * (1.0 + e));
    let mut out = vec![0f32; (4.0 * FS) as usize];
    g.render(&mut out, &mut NoAudio);
    // The phase of 100 Hz in the first and the last second.
    let phase = |x: &[f32], t0: f64| {
        let (mut re, mut im) = (0.0, 0.0);
        for (k, &v) in x.iter().enumerate() {
            let p = PI2 * 100.0 * (t0 + k as f64 / FS);
            re += v as f64 * p.cos();
            im -= v as f64 * p.sin();
        }
        im.atan2(re)
    };
    let n = FS as usize;
    let drift = phase(&out[3 * n..4 * n], 3.0) - phase(&out[..n], 0.0);
    let df = drift / (PI2 * 3.0);
    let want = 100.0 / (1.0 + e) - 100.0;
    assert!((df - want).abs() < 1e-4, "{df} Hz, want {want}");
}

/// The bits of the DCS word, read back at their centres.
#[test]
fn dcs_bits_and_spectrum() {
    for code in ["023", "754I"] {
        let code = DcsCode::parse(code).unwrap();
        let cfg = NfmConfig { subaudio: Subaudio::Dcs(code), ..Default::default() };
        let mut g = BasebandGenerator::new(cfg.clone(), FS).unwrap();
        let mut out = vec![0f32; (2.0 * FS) as usize];
        g.render(&mut out, &mut NoAudio);
        let bit_len = FS / DCS_BIT_RATE;
        for k in 0..(out.len() as f64 / bit_len) as usize - 1 {
            let v = out[((k as f64 + 0.5) * bit_len) as usize];
            let one = code.word() >> (k % DCS_WORD_BITS) & 1 != 0;
            assert!((v as f64 - if one { 1.0 } else { -1.0 } * cfg.subaudio_deviation).abs() < 1.0, "bit {k}: {v}");
        }
        // Next to nothing in the voice band.
        let total: f64 = out.iter().map(|&v| (v as f64).powi(2)).sum();
        let mut above = 0.0;
        for f in (300..3000).step_by(20) {
            above += amplitude(&out, f as f64).powi(2) / 2.0 * out.len() as f64;
        }
        assert!(10.0 * (above / total).log10() < -45.0, "{code}: {:.1} dB", 10.0 * (above / total).log10());
    }
}
