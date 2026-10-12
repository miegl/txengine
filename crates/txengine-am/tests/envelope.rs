//! The envelope generator: carrier level, modulation depth, bandwidth and
//! limiting.

use txengine_am::{AmConfig, EnvelopeGenerator};
use txengine_audio::{AudioFeed, NoAudio};

const FS: f64 = 100_000.0;

/// A sine of amplitude `amp` at `freq` Hz (both channels), at `rate`.
struct Tone {
    freq: f64,
    amp: f64,
    rate: f64,
    n: u64,
}

impl AudioFeed for Tone {
    fn read(&mut self, out: &mut [[f32; 2]]) -> usize {
        for o in out.iter_mut() {
            let v = (self.amp * (2.0 * std::f64::consts::PI * self.freq * self.n as f64 / self.rate).sin()) as f32;
            *o = [v, v];
            self.n += 1;
        }
        out.len()
    }
}

/// Render `seconds` and drop the first 0.1 s (filters settling).
fn render(cfg: AmConfig, audio: &mut dyn AudioFeed, seconds: f64) -> (Vec<f32>, EnvelopeGenerator) {
    let mut g = EnvelopeGenerator::new(cfg, FS).unwrap();
    let mut out = vec![0f32; (seconds * FS) as usize];
    for chunk in out.chunks_mut(1000) {
        g.render(chunk, audio);
    }
    (out.split_off((0.1 * FS) as usize), g)
}

fn tone(freq: f64, amp: f64) -> Tone {
    Tone { freq, amp, rate: EnvelopeGenerator::audio_rate_for(FS), n: 0 }
}

/// Amplitude of the component at `f` (Hz) of `x` around its mean.
fn amplitude(x: &[f32], f: f64) -> f64 {
    let mean = x.iter().map(|&v| v as f64).sum::<f64>() / x.len() as f64;
    let (mut re, mut im) = (0.0, 0.0);
    for (n, &v) in x.iter().enumerate() {
        let p = 2.0 * std::f64::consts::PI * f * n as f64 / FS;
        re += (v as f64 - mean) * p.cos();
        im += (v as f64 - mean) * p.sin();
    }
    2.0 * (re * re + im * im).sqrt() / x.len() as f64
}

#[test]
fn carrier_level_follows_the_depth() {
    for depth in [0.5, 0.9, 1.0] {
        let cfg = AmConfig { depth, ..Default::default() };
        let (out, _) = render(cfg, &mut NoAudio, 0.3);
        let c = 1.0 / (1.0 + depth);
        assert!(out.iter().all(|&v| (v as f64 - c).abs() < 1e-6), "depth {depth}");
    }
}

#[test]
fn a_tone_modulates_at_the_set_depth() {
    // Full scale (amplitude 1) is the configured depth.
    let cfg = AmConfig { depth: 0.8, ..Default::default() };
    let (out, mut g) = render(cfg, &mut tone(1000.0, 0.5), 1.1);
    let c = 1.0 / 1.8;
    let m = amplitude(&out, 1000.0) / c;
    assert!((m - 0.4).abs() < 0.002, "modulation {m}");
    let st = g.stats(true);
    assert!((st.peak_positive - 0.4).abs() < 0.01 && (st.peak_negative - 0.4).abs() < 0.01, "{st:?}");
    assert_eq!(st.limiter_gain, 1.0);
}

#[test]
fn audio_above_the_bandwidth_is_removed() {
    for (bandwidth, inside, outside) in [(4_500.0, 4_000.0, 5_500.0), (10_000.0, 9_000.0, 11_500.0)] {
        let cfg = AmConfig { bandwidth, ..Default::default() };
        let (pass, _) = render(cfg.clone(), &mut tone(inside, 0.5), 0.6);
        let (stop, _) = render(cfg, &mut tone(outside, 0.5), 0.6);
        let (a, b) = (amplitude(&pass, inside), amplitude(&stop, outside));
        assert!(20.0 * (a / (0.5 * 0.9 / 1.9)).log10() > -0.1, "{bandwidth}: pass band {a}");
        assert!(20.0 * (b / a).log10() < -58.0, "{bandwidth}: stop band {:.1} dB", 20.0 * (b / a).log10());
    }
}

#[test]
fn overload_is_limited_inside_the_envelope() {
    let cfg = AmConfig { depth: 0.95, ..Default::default() };
    let c = 1.0 / 1.95;
    let (out, mut g) = render(cfg.clone(), &mut tone(400.0, 4.0), 1.0);
    let (lo, hi) = out.iter().fold((1f32, 0f32), |(lo, hi), &v| (lo.min(v), hi.max(v)));
    assert!(hi <= 1.0 + 1e-5 && lo as f64 >= c * 0.05 - 1e-5, "envelope {lo}..{hi}");
    assert!(hi > 0.99, "the peaks reach full output ({hi})");
    assert!(g.stats(true).limiter_gain < 0.3);
    // Without the limiter, clipped.
    let (out, mut g) = render(AmConfig { limiter: false, ..cfg }, &mut tone(400.0, 4.0), 0.5);
    assert!(out.iter().all(|&v| (-1e-6..=1.0 + 1e-6).contains(&v)));
    assert!(g.stats(true).clipped > 0);
}
