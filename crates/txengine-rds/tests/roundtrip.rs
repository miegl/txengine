//! End-to-end: encoder → modulator → independent decoder.

use txengine_dsp::{PhaseAccumulator, SineTable};
use txengine_rds::decoder::decode_mpx;
use txengine_rds::encoder::{RdsParams, af_code, encode_rt, encode8};
use txengine_rds::{DecoderInfo, RdsEncoder, RdsModulator};

/// RDS-only multiplex: `level` Hz deviation on the 57 kHz subcarrier.
fn render(fs: f64, seconds: f64, params: RdsParams, t0: f64) -> Vec<f32> {
    let mut acc = PhaseAccumulator::new(19_000.0, fs);
    let sin = SineTable::new();
    let mut enc = RdsEncoder::new(params).unwrap();
    enc.set_local_offset_fn(Box::new(|_| 3600));
    let mut m = RdsModulator::new();
    let n = (fs * seconds) as usize;
    let mut out = vec![0f32; n];
    // Odd block sizes exercise the block boundaries.
    for (k, block) in out.chunks_mut(997).enumerate() {
        m.render(&acc, block, &mut enc, t0 + (k * 997) as f64 / fs, 1.0 / fs);
        for b in block.iter_mut() {
            *b *= 2000.0 * sin.sin(acc.harmonic_frac(3));
            acc.advance();
        }
    }
    out
}

fn params() -> RdsParams {
    RdsParams {
        pi: 0x2e5a,
        ps: encode8("ČRo D-dur").unwrap_or(*b"CRo Ddur"),
        rt: encode_rt("Příliš žluťoučký kůň — 87.6 MHz test").unwrap(),
        pty: 14,
        tp: true,
        ta: false,
        ms: true,
        di: DecoderInfo { stereo: true, ..Default::default() },
        af: [87.6, 99.5, 104.3].iter().map(|&f| af_code(f).unwrap()).collect(),
        ptyn: Some(encode8("Classic").unwrap()),
        ecc: Some(0xe2),
        lic: None,
        ct: true,
    }
}

fn check(fs: f64, seconds: f64) {
    // Start 3 s before a minute edge so one CT group is sent.
    let t0 = 1_791_236_580.0 - 3.0;
    let p = RdsParams { ps: encode8("CRo Dur").unwrap(), ..params() };
    let mpx = render(fs, seconds, p.clone(), t0);
    let d = decode_mpx(&mpx, fs);
    assert!(d.blocks_total > 100, "fs {fs}: only {} blocks", d.blocks_total);
    assert_eq!(d.blocks_ok, d.blocks_total, "fs {fs}: block errors in a clean signal");
    let st = &d.station;
    assert_eq!(st.pi, Some(0x2e5a));
    assert_eq!(st.pty, Some(14));
    assert_eq!(st.tp, Some(true));
    assert_eq!(st.ms, Some(true));
    assert_eq!(st.ps_string(), "CRo Dur ");
    assert_eq!(st.rt_string(), "Příliš žlutoučký kuň - 87.6 MHz test");
    assert_eq!(st.ptyn_string(), "Classic ");
    assert_eq!(st.ecc, Some(0xe2));
    // di[n] is decoder-identification bit dn; only d0 (stereo) is set.
    assert_eq!(st.di, [Some(true), Some(false), Some(false), Some(false)]);
    let mut af = st.af.clone();
    af.sort();
    assert_eq!(af, vec![1, 120, 168]);
    assert_eq!(st.ct, vec![(61318, 21, 43, 2)], "one CT group for 21:43 UTC, +1 h");
    assert!((d.level - 2000.0).abs() < 20.0, "level {}", d.level);
}

#[test]
fn roundtrip_228k() {
    check(228_000.0, 6.0);
}

#[test]
fn roundtrip_250k() {
    check(250_000.0, 6.0);
}

#[test]
fn roundtrip_non_integer_rate() {
    check(50e6 / 219.0, 6.0);
}

#[test]
fn decodes_with_clock_offset() {
    // Receiver clock 80 ppm off: resample by linear interpolation.
    let fs = 250_000.0;
    let mpx = render(fs, 5.0, params(), 0.0);
    let r = 1.0 + 80e-6;
    let res: Vec<f32> = (0..((mpx.len() - 2) as f64 / r) as usize)
        .map(|i| {
            let t = i as f64 * r;
            let k = t as usize;
            let f = (t - k as f64) as f32;
            mpx[k] * (1.0 - f) + mpx[k + 1] * f
        })
        .collect();
    let d = decode_mpx(&res, fs);
    assert!(d.blocks_total > 100);
    assert!(d.blocks_ok as f64 / d.blocks_total as f64 > 0.99, "{}/{}", d.blocks_ok, d.blocks_total);
    // Resampling by 1 + 80 ppm raises every frequency by 80 ppm.
    assert!((d.carrier_offset - 57_000.0 * 80e-6).abs() < 0.2, "offset {}", d.carrier_offset);
}

#[test]
fn spectrum_confined_to_rds_channel() {
    // EN 50067: the shaped data spectrum ends at 57 kHz ± 2.4 kHz.
    let fs = 250_000.0;
    let mpx = render(fs, 2.0, params(), 0.0);
    let n = 1 << 18;
    let x = &mpx[..n];
    // Goertzel power at a few frequencies (Hann window).
    let power = |f: f64| -> f64 {
        let w = 2.0 * std::f64::consts::PI * f / fs;
        let (mut re, mut im) = (0.0, 0.0);
        for (i, &v) in x.iter().enumerate() {
            let win = 0.5 - 0.5 * (2.0 * std::f64::consts::PI * i as f64 / n as f64).cos();
            re += v as f64 * win * (w * i as f64).cos();
            im -= v as f64 * win * (w * i as f64).sin();
        }
        re * re + im * im
    };
    let inband = power(57_000.0 + 1187.5).max(power(57_000.0 - 600.0));
    for f in [50_000.0, 53_500.0, 60_500.0, 64_000.0, 19_000.0, 38_000.0] {
        let rel = 10.0 * (power(f) / inband).log10();
        assert!(rel < -60.0, "{f} Hz: {rel:.1} dB");
    }
}
