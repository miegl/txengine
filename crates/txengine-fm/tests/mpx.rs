//! Multiplex conformance tests (ITU-R BS.450-4, EN 50067).

use txengine_dsp::{FirDesign, Preemphasis, sinc};
use txengine_fm::mpx::{AudioFeed, MpxConfig, MpxGenerator, StereoMode};
use txengine_rds::decoder::decode_mpx;
use txengine_rds::encoder::{encode8, encode_rt};
use txengine_rds::{RdsEncoder, RdsParams};

/// Stereo test signal: independent tones on L and R.
struct Tones {
    fa: f64,
    n: u64,
    left: (f64, f64),
    right: (f64, f64),
}

impl AudioFeed for Tones {
    fn read(&mut self, out: &mut [[f32; 2]]) -> usize {
        for o in out.iter_mut() {
            let t = self.n as f64 / self.fa;
            // Cosine so that a 0 Hz "tone" is DC.
            let w = |(f, a): (f64, f64)| (a * (2.0 * std::f64::consts::PI * f * t).cos()) as f32;
            *o = [w(self.left), w(self.right)];
            self.n += 1;
        }
        out.len()
    }
}

fn render(cfg: MpxConfig, fs: f64, left: (f64, f64), right: (f64, f64), seconds: f64) -> (Vec<f32>, MpxGenerator) {
    let mut mpx = MpxGenerator::new(cfg, fs, true).unwrap();
    let mut audio = Tones { fa: mpx.audio_rate(), n: 0, left, right };
    let mut rds = RdsEncoder::new(RdsParams {
        ps: encode8("TEST").unwrap(),
        rt: encode_rt("Stereo multiplex test").unwrap(),
        ct: false,
        di: txengine_rds::DecoderInfo { stereo: true, ..Default::default() },
        ..Default::default()
    })
    .unwrap();
    let mut out = vec![0f32; (fs * seconds) as usize];
    for chunk in out.chunks_mut(4096) {
        mpx.render(chunk, &mut audio, &mut rds, 0.0);
    }
    (out, mpx)
}

/// Amplitude and phase (radians, relative to sin) of the component at `f`
/// over a whole number of cycles, skipping the start-up transient.
fn component(x: &[f32], fs: f64, f: f64) -> (f64, f64) {
    let start = (fs * 0.2) as usize;
    let cycles = ((x.len() - start) as f64 * f / fs).floor();
    let n = (cycles * fs / f).round() as usize;
    let (mut c, mut s) = (0.0, 0.0);
    for (i, &v) in x[start..start + n].iter().enumerate() {
        let ph = 2.0 * std::f64::consts::PI * f * (start + i) as f64 / fs;
        s += v as f64 * ph.sin();
        c += v as f64 * ph.cos();
    }
    let (s, c) = (2.0 * s / n as f64, 2.0 * c / n as f64);
    ((s * s + c * c).sqrt(), c.atan2(s))
}

/// Model the sample-and-hold of the hardware: the generator pre-compensates
/// with 1/sinc, so dividing measured amplitudes by it gives the RF levels.
fn on_air(amp: f64, f: f64, fs: f64) -> f64 {
    amp * sinc(f / fs)
}

#[test]
fn levels_and_phase_bs450() {
    let fs = 250_000.0;
    let cfg = MpxConfig { preemphasis: None, ..Default::default() };
    let (x, mpx) = render(cfg, fs, (1000.0, 0.5), (1000.0, 0.5), 3.0);
    // Pilot: 6.75 kHz (9 %), 19 kHz.
    let (p, pph) = component(&x, fs, 19_000.0);
    assert!((on_air(p, 19_000.0, fs) - 6_750.0).abs() < 7.0, "pilot {p}");
    // 38 kHz carrier suppressed (BS.450: residual < 1 %).
    let (c38, _) = component(&x, fs, 38_000.0);
    assert!(c38 < 75.0 * 0.01, "38 kHz residual {c38} Hz");
    // L = R = 0.5 → M only at half the audio deviation; S absent.
    let (m, _) = component(&x, fs, 1000.0);
    assert!((on_air(m, 1000.0, fs) - 0.5 * mpx.audio_deviation()).abs() < 0.005 * mpx.audio_deviation(), "M {m}");
    let (sb, _) = component(&x, fs, 39_000.0);
    assert!(sb < 1.0, "S sideband for mono content: {sb}");
    // The pilot starts at phase 0 (sine) at sample 0; the only shift is the
    // 4-sample group delay of the 9-tap hold compensation filter.
    let expect = -2.0 * std::f64::consts::PI * 19_000.0 * 4.0 / fs;
    let diff = (pph - expect + std::f64::consts::PI).rem_euclid(2.0 * std::f64::consts::PI) - std::f64::consts::PI;
    assert!(diff.abs() < 0.01, "pilot phase {pph}, expected {expect}");
}

#[test]
fn subcarrier_phase_bs450_2_2_2_5() {
    // BS.450 §2.2.2.5: with A positive and B = -A, the multiplex crosses
    // zero with positive slope whenever the pilot is zero. With A = -B = +c
    // (DC), S = c and the 38 kHz term is c·sin(2φ) for pilot sin(φ): the
    // 38 kHz phase must be twice the pilot phase (both relative to sine).
    let fs = 228_000.0;
    let cfg = MpxConfig { preemphasis: None, rds_deviation: 0.0, ..Default::default() };
    let (x, mpx) = render(cfg, fs, (0.0, 0.5), (0.0, -0.5), 1.0);
    let (p, pph) = component(&x, fs, 19_000.0);
    let (c, cph) = component(&x, fs, 38_000.0);
    assert!(p > 6_000.0);
    assert!((on_air(c, 38_000.0, fs) - 0.5 * mpx.audio_deviation()).abs() < 0.01 * mpx.audio_deviation(), "38 kHz {c}");
    let diff = (cph - 2.0 * pph + std::f64::consts::PI).rem_euclid(2.0 * std::f64::consts::PI) - std::f64::consts::PI;
    // Tolerance: BS.450 allows ±3° of pilot phase error.
    assert!(diff.abs() < 3f64.to_radians(), "38 kHz phase {cph} vs pilot {pph}: error {:.2}°", diff.to_degrees());
}

#[test]
fn stereo_separation() {
    // Left-only tones; measure crosstalk into R after an ideal decoder.
    let fs = 250_000.0;
    for tone in [400.0, 1000.0, 5000.0, 10_000.0, 14_000.0] {
        let cfg = MpxConfig { preemphasis: None, rds_deviation: 0.0, hold_compensation: false, ..Default::default() };
        let (x, _) = render(cfg, fs, (tone, 0.7), (tone, 0.0), 1.5);
        let lp = FirDesign::lowpass(fs, 15_500.0, 20_000.0, 90.0);
        let (mut fm, mut fsd) = (txengine_dsp::Fir::new(&lp), txengine_dsp::Fir::new(&lp));
        let mut l = Vec::new();
        let mut r = Vec::new();
        for (i, &v) in x.iter().enumerate() {
            let ph = 2.0 * std::f64::consts::PI * 38_000.0 * i as f64 / fs;
            let m = fm.process(v);
            let s = fsd.process(v * 2.0 * ph.sin() as f32);
            l.push(m + s);
            r.push(m - s);
        }
        let (al, _) = component(&l, fs, tone);
        let (ar, _) = component(&r, fs, tone);
        let sep = 20.0 * (al / ar).log10();
        assert!(sep > 50.0, "{tone} Hz: separation {sep:.1} dB");
    }
}

#[test]
fn deviation_is_limited() {
    // Grossly overdriven, pre-emphasised high-frequency programme: the peak
    // deviation must stay within the configured maximum.
    let fs = 250_000.0;
    let cfg = MpxConfig { gain_db: 20.0, ..Default::default() };
    let (x, mut mpx) = render(cfg, fs, (9_000.0, 1.0), (12_500.0, 1.0), 2.0);
    let peak = x.iter().fold(0f32, |m, &v| m.max(v.abs()));
    assert!(peak <= 75_000.0, "peak {peak}");
    let st = mpx.stats(false);
    assert!(st.limiter_gain < 0.2, "limiter must be active");
    // The final clipper should almost never act (the limiter does the work).
    assert!((st.clipped as f64) < x.len() as f64 * 1e-4, "clipped {} samples", st.clipped);
}

#[test]
fn rds_decodes_with_programme() {
    let fs = 50e6 / 200.0;
    let cfg = MpxConfig { rds_deviation: 2_000.0, ..Default::default() };
    let (x, _) = render(cfg, fs, (1000.0, 0.9), (3150.0, 0.9), 5.0);
    let d = decode_mpx(&x, fs);
    assert!(d.blocks_total > 150);
    assert_eq!(d.blocks_ok, d.blocks_total, "RDS block errors under full programme modulation");
    assert_eq!(d.station.ps_string(), "TEST    ");
    assert_eq!(d.station.rt_string(), "Stereo multiplex test");
    assert_eq!(d.station.di[0], Some(true), "DI d0 = stereo");
    // Measured injection (compensated for the hold model).
    let level = on_air(d.level, 57_000.0, fs);
    assert!((level - 2_000.0).abs() < 40.0, "RDS level {level}");
}

#[test]
fn mono_has_no_pilot() {
    let fs = 250_000.0;
    let cfg = MpxConfig { stereo: StereoMode::Mono, ..Default::default() };
    let (x, mpx) = render(cfg, fs, (1000.0, 0.5), (1000.0, 0.5), 1.0);
    assert!(!mpx.is_stereo());
    let (p, _) = component(&x, fs, 19_000.0);
    assert!(p < 1.0, "pilot in mono: {p}");
}

/// An input that has nothing to deliver, either because it ended or because
/// it is late.
struct Empty {
    ended: bool,
}

impl AudioFeed for Empty {
    fn read(&mut self, _out: &mut [[f32; 2]]) -> usize {
        0
    }
    fn ended(&self) -> bool {
        self.ended
    }
}

#[test]
fn silence_after_the_end_is_not_an_underrun() {
    for ended in [true, false] {
        let mut mpx = MpxGenerator::new(MpxConfig::default(), 250_000.0, true).unwrap();
        let mut rds = RdsEncoder::new(RdsParams::default()).unwrap();
        let mut out = vec![0f32; 10_000];
        mpx.render(&mut out, &mut Empty { ended }, &mut rds, 0.0);
        let u = mpx.stats(false).audio_underruns;
        if ended {
            assert_eq!(u, 0, "missing audio after the end counted as underrun");
        } else {
            assert!(u >= 1900, "late audio not counted: {u}");
        }
    }
}

#[test]
fn measured_output_response_is_equalised() {
    // An output stage that lifts high modulation frequencies (like the
    // BCM283x PLL loop), on top of the sample-and-hold: after the generator's
    // pre-compensation the on-air levels must be the nominal ones.
    let table = [(0.0, 0.0), (10e3, 0.03), (19e3, 0.10), (38e3, 0.34), (57e3, 0.50), (67e3, 0.545)];
    let fs = 195_925.0;
    let response = |f: f64| sinc(f / fs) * 10f64.powf(txengine_core::interpolate_db(&table, f) / 20.0);
    let cfg = MpxConfig { preemphasis: None, rds_deviation: 0.0, output_response_db: table.to_vec(), ..Default::default() };
    let (x, mpx) = render(cfg, fs, (1000.0, 0.5), (1000.0, 0.0), 2.0);
    let (p, _) = component(&x, fs, 19_000.0);
    assert!((p * response(19_000.0) - 6_750.0).abs() < 7.0, "pilot {:.1}", p * response(19_000.0));
    // Left only: M = S = A/2 on air, S as two sidebands of S/2 each.
    let m = component(&x, fs, 1000.0).0 * response(1000.0);
    let lo = component(&x, fs, 37_000.0).0 * response(37_000.0);
    let hi = component(&x, fs, 39_000.0).0 * response(39_000.0);
    let half = 0.25 * mpx.audio_deviation();
    assert!((m - half).abs() < 0.002 * half, "M {m:.1}, expected {half:.1}");
    assert!((lo + hi - m).abs() < 0.002 * m, "S sidebands {lo:.1} + {hi:.1}, M {m:.1}");
}

#[test]
fn subcarriers_follow_the_actual_sample_rate() {
    use txengine_core::{Modulator, RenderContext, SampleFormat, SinkInfo};
    use txengine_fm::{Programme, WfmModulator};
    let fs = 250_000.0;
    let sink = SinkInfo {
        description: "test".into(),
        sample_rate: fs,
        carrier_hz: 87.6e6,
        max_offset_hz: 100e3,
        resolution_hz: 1.0,
        max_clock_error_ppm: 200.0,
        buffer_len: 4096,
        format: SampleFormat::FrequencyOffset,
        realtime: true,
        zero_order_hold: false,
        response_db: Vec::new(),
    };
    let cfg = MpxConfig { stereo: StereoMode::Stereo, ..Default::default() };
    let mut m = WfmModulator::new(cfg, Programme::None, RdsParams::default(), false).unwrap();
    m.configure(&sink).unwrap();
    // The crystal runs 150 ppm fast, so the samples leave 150 ppm faster
    // than nominal.
    let actual = fs * (1.0 + 150e-6);
    let mut x = vec![0f32; (fs * 3.0) as usize];
    let mut first_sample = 0;
    for chunk in x.chunks_mut(4096) {
        m.render(&RenderContext { first_sample, sample_rate: actual, emit_time_unix: None }, chunk);
        first_sample += chunk.len() as u64;
    }
    // The pilot is at 19 kHz in true time, 2.85 Hz lower in nominal samples.
    let (pilot, _) = component(&x, fs, 19_000.0 * fs / actual);
    assert!((pilot - 6_750.0).abs() < 10.0, "pilot {pilot}");
    let (uncorrected, _) = component(&x, fs, 19_000.0);
    assert!(uncorrected < 0.1 * pilot, "{uncorrected}");
}

/// Pink noise (Paul Kellett's refined filter on a fixed white sequence):
/// `L = p + w·q`, `R = p - w·q` with independent pink `p`, `q`, scaled by `amp`.
struct Pink {
    amp: f32,
    width: f32,
    seed: u32,
    state: [[f32; 7]; 2],
}

impl Pink {
    fn new(amp: f32, width: f32) -> Pink {
        Pink { amp, width, seed: 1, state: [[0.0; 7]; 2] }
    }

    fn next(&mut self, ch: usize) -> f32 {
        self.seed = self.seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let w = (self.seed >> 8) as f32 / (1 << 23) as f32 - 1.0;
        let b = &mut self.state[ch];
        b[0] = 0.99886 * b[0] + w * 0.0555179;
        b[1] = 0.99332 * b[1] + w * 0.0750759;
        b[2] = 0.96900 * b[2] + w * 0.153_852;
        b[3] = 0.86650 * b[3] + w * 0.3104856;
        b[4] = 0.55000 * b[4] + w * 0.5329522;
        b[5] = -0.7616 * b[5] - w * 0.016_898;
        let p = b[0] + b[1] + b[2] + b[3] + b[4] + b[5] + b[6] + w * 0.5362;
        b[6] = w * 0.115926;
        0.11 * p
    }
}

impl AudioFeed for Pink {
    fn read(&mut self, out: &mut [[f32; 2]]) -> usize {
        for o in out.iter_mut() {
            let (p, q) = (self.next(0), self.next(1));
            *o = [self.amp * (p + self.width * q), self.amp * (p - self.width * q)];
        }
        out.len()
    }
}

/// Render `seconds` of `audio` and return the generator (for its meters).
fn render_feed(cfg: MpxConfig, fs: f64, audio: &mut dyn AudioFeed, seconds: f64) -> MpxGenerator {
    let mut mpx = MpxGenerator::new(cfg, fs, true).unwrap();
    let mut rds = RdsEncoder::new(RdsParams::default()).unwrap();
    let mut out = vec![0f32; 4096];
    for _ in 0..(fs * seconds) as usize / out.len() {
        mpx.render(&mut out, audio, &mut rds, 0.0);
    }
    mpx
}

#[test]
fn mpx_power_meter_bs412() {
    // A sine of ±19 kHz deviation is 0 dBr.
    let fs = 250_000.0;
    let cfg = MpxConfig { preemphasis: None, rds_deviation: 0.0, stereo: StereoMode::Mono, ..Default::default() };
    let amp = 19_000.0 / MpxGenerator::new(cfg.clone(), fs, false).unwrap().audio_deviation();
    let (_, mut mpx) = render(cfg, fs, (1000.0, amp), (1000.0, amp), 3.0);
    let p = mpx.stats(false).mpx_power_dbr;
    assert!(p.abs() < 0.05, "MPX power {p} dBr");
}

#[test]
fn preemphasis_keeps_the_mpx_power_of_pink_noise() {
    // The drive correction makes the pre-emphasis power neutral for pink
    // noise (the test noise also has power below 20 Hz, which the
    // pre-emphasis does not raise, so it ends up a little lower).
    let fs = 250_000.0;
    let base = MpxConfig { rds_deviation: 0.0, stereo: StereoMode::Mono, limiter: false, ..Default::default() };
    let power = |preemphasis: Option<f64>| {
        let cfg = MpxConfig { preemphasis, ..base.clone() };
        render_feed(cfg, fs, &mut Pink::new(0.1, 0.0), 4.0).stats(false).mpx_power_dbr
    };
    let (flat, emph50, emph75) = (power(None), power(Some(50e-6)), power(Some(75e-6)));
    for (tau, p) in [(50, emph50), (75, emph75)] {
        assert!(p < flat + 0.1 && p > flat - 0.7, "{tau} µs: {p:.2} dBr vs {flat:.2} dBr without pre-emphasis");
    }
}

#[test]
fn pink_noise_at_0_dbr_is_not_limited() {
    // Mostly mono pink noise, a Gaussian programme with a crest factor of
    // about 14 dB, at 0 dBr including pilot and RDS: neither limiter acts
    // (the HF limiter starts at about +2 dBr).
    let fs = 250_000.0;
    let flat = MpxConfig { preemphasis: None, limiter: false, ..Default::default() };
    let p0 = render_feed(flat, fs, &mut Pink::new(0.1, 0.3), 4.0).stats(false).mpx_power_dbr;
    let pilot_rds = render_feed(MpxConfig::default(), fs, &mut Pink::new(0.0, 0.0), 4.0).stats(false).mpx_power_dbr;
    // Gain that puts the flat programme at 0 dBr in total.
    let audio = |dbr: f64| 10.0 * (10f64.powf(dbr / 10.0) - 10f64.powf(pilot_rds / 10.0)).log10();
    let gain_db = audio(0.0) - audio(p0);
    let mut mpx = render_feed(MpxConfig { gain_db, ..Default::default() }, fs, &mut Pink::new(0.1, 0.3), 4.0);
    let st = mpx.stats(false);
    assert!(st.mpx_power_dbr.abs() < 0.6, "MPX power {:.2} dBr", st.mpx_power_dbr);
    assert_eq!(st.limiter_gain, 1.0, "peak limiter acted");
    assert_eq!(st.hf_limiter_db, 0.0, "HF limiter acted");
    assert_eq!(st.clipped, 0);
}

#[test]
fn hf_overload_lowers_only_the_treble() {
    // 1 kHz at 0.5 plus 10 kHz at 0.4 of full scale: the flat signal fits,
    // the pre-emphasised 10 kHz (+10.4 dB) does not. Only the treble may be
    // reduced: the 1 kHz level changes by at most the 0.4 dB the
    // pre-emphasis adds at 1 kHz, and the peak limiter does not act.
    let fs = 250_000.0;
    struct TwoTones {
        fa: f64,
        n: u64,
    }
    impl AudioFeed for TwoTones {
        fn read(&mut self, out: &mut [[f32; 2]]) -> usize {
            for o in out.iter_mut() {
                let t = self.n as f64 / self.fa;
                let v = 0.5 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin() + 0.4 * (2.0 * std::f64::consts::PI * 10_000.0 * t).sin();
                *o = [v as f32; 2];
                self.n += 1;
            }
            out.len()
        }
    }
    let cfg = MpxConfig { rds_deviation: 0.0, hold_compensation: false, ..Default::default() };
    let mut mpx = MpxGenerator::new(cfg.clone(), fs, true).unwrap();
    let mut audio = TwoTones { fa: mpx.audio_rate(), n: 0 };
    let mut rds = RdsEncoder::new(RdsParams::default()).unwrap();
    let mut x = vec![0f32; (fs * 2.0) as usize];
    for chunk in x.chunks_mut(4096) {
        mpx.render(chunk, &mut audio, &mut rds, 0.0);
    }
    let st = mpx.stats(false);
    assert_eq!(st.limiter_gain, 1.0, "the whole programme was limited");
    assert!(st.hf_limiter_db > 1.0, "HF limiter inactive ({} dB)", st.hf_limiter_db);
    let peak = x.iter().fold(0f32, |m, &v| m.max(v.abs()));
    assert!(peak <= 75_000.0);
    let (lf, _) = component(&x, fs, 1000.0);
    let nominal = 0.5 * txengine_dsp::from_db(mpx.drive_db()) * mpx.audio_deviation();
    let change = 20.0 * (lf / nominal).log10();
    assert!((-0.05..0.45).contains(&change), "1 kHz level {change:+.2} dB re flat");
    let (hf, _) = component(&x, fs, 10_000.0);
    let full = 0.4 * txengine_dsp::from_db(mpx.drive_db()) * mpx.audio_deviation() * Preemphasis::new(50e-6, mpx.audio_rate(), 15_000.0).gain_at(10_000.0, mpx.audio_rate());
    assert!(hf < 0.9 * full, "10 kHz {hf} Hz, unlimited {full} Hz");
}
