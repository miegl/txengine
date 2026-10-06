//! `txengine analyze`: measure an FM multiplex recording.
//!
//! Works on files rendered by `txengine render` and on multiplex signals
//! demodulated from over-the-air captures, so the transmitter can be
//! verified end to end against BS.450 / EN 50067.

use std::f64::consts::PI;

use anyhow::bail;
use txengine_dsp::{Fir, FirDesign};
use txengine_rds::decoder::decode_mpx;

use crate::cli::AnalyzeArgs;

/// Complex correlation of `x` with a tone at `f` (time origin at x[0]).
fn corr(x: &[f32], fs: f64, f: f64, origin: usize) -> (f64, f64) {
    let (mut re, mut im) = (0.0, 0.0);
    for (i, &v) in x.iter().enumerate() {
        let ph = 2.0 * PI * f * (origin + i) as f64 / fs;
        re += v as f64 * ph.cos();
        im += v as f64 * ph.sin();
    }
    (re, im)
}

/// Measure a tone within ±`span` Hz of `f`: coarse scan on a short segment,
/// then the exact frequency from a linear fit of the phase of 50 ms
/// segments. Returns (amplitude, phase at sample 0 relative to sine, frequency).
fn find_tone(x: &[f32], fs: f64, f: f64, span: f64) -> (f64, f64, f64) {
    let seg = &x[..x.len().min((fs * 0.2) as usize)];
    let mut coarse = (0.0, f);
    let mut g = f - span;
    while g <= f + span {
        let (re, im) = corr(seg, fs, g, 0);
        if re.hypot(im) > coarse.0 {
            coarse = (re.hypot(im), g);
        }
        g += 1.0;
    }
    let fc = coarse.1;
    let n = (fs * 0.05) as usize;
    let mut pts = Vec::new();
    let mut prev: Option<f64> = None;
    let mut unwrap = 0.0;
    for (k, chunk) in x.chunks_exact(n).enumerate() {
        let (re, im) = corr(chunk, fs, fc, k * n);
        let mut ph = im.atan2(re);
        if let Some(p) = prev {
            while ph + unwrap - p > PI {
                unwrap -= 2.0 * PI;
            }
            while ph + unwrap - p < -PI {
                unwrap += 2.0 * PI;
            }
        }
        ph += unwrap;
        prev = Some(ph);
        pts.push(((k * n + n / 2) as f64 / fs, ph));
    }
    let m = pts.len() as f64;
    let (st, sp) = pts.iter().fold((0.0, 0.0), |a, p| (a.0 + p.0, a.1 + p.1));
    let (mt, mp) = (st / m, sp / m);
    let num: f64 = pts.iter().map(|p| (p.0 - mt) * (p.1 - mp)).sum();
    let den: f64 = pts.iter().map(|p| (p.0 - mt).powi(2)).sum();
    // corr() phase = atan2(Σ x sin, Σ x cos): a tone sin(wt + φ) correlated
    // at a slightly lower frequency gains phase... its phase advances at
    // +2π·Δf, so the slope gives the offset directly.
    let df = if den > 0.0 { num / den / (2.0 * PI) } else { 0.0 };
    let fr = fc - df;
    let (re, im) = corr(x, fs, fr, 0);
    let amp = 2.0 * re.hypot(im) / x.len() as f64;
    (amp, re.atan2(im), fr)
}

pub fn run(a: &AnalyzeArgs) -> anyhow::Result<()> {
    let (rate, channels, data) = txengine_audio::read_file(&a.file)?;
    let fs = rate as f64;
    if fs < 120_000.0 {
        bail!("{} Hz is too low for an FM multiplex (need >= 128 kHz)", rate);
    }
    let x: Vec<f32> = data.iter().step_by(channels).map(|&v| (v as f64 * a.full_scale) as f32).collect();
    let secs = x.len() as f64 / fs;
    println!("File:       {} ({} Hz, {:.2} s)", a.file.display(), rate, secs);
    let skip = (fs * 0.1) as usize;
    let body = &x[skip.min(x.len())..];
    let peak = body.iter().fold(0f32, |m, &v| m.max(v.abs()));
    let mean = body.iter().map(|&v| v as f64).sum::<f64>() / body.len() as f64;
    let power = body.iter().map(|&v| (v as f64 - mean).powi(2)).sum::<f64>() / body.len() as f64;
    println!("Deviation:  peak {:.2} kHz, mean (carrier offset) {:+.1} Hz", peak / 1e3, mean);
    // ITU-R BS.412: 0 dBr = power of a sine with ±19 kHz peak deviation.
    println!("MPX power:  {:+.2} dBr (BS.412 limit +0 dBr, measured over 60 s)", 10.0 * (power / (19_000f64.powi(2) / 2.0)).log10());

    let (pilot, pilot_ph, pilot_f) = find_tone(body, fs, 19_000.0, 20.0);
    let stereo = pilot > 1_000.0;
    if stereo {
        println!(
            "Pilot:      {:.0} Hz deviation ({:.1} % of 75 kHz) at {:.3} Hz",
            pilot,
            pilot / 750.0,
            pilot_f
        );
        // Stereo demodulation with a pilot-locked 38 kHz reference.
        let lp = FirDesign::lowpass(fs, 15_000.0, 19_000.0, 70.0);
        let (mut fm, mut fsub) = (Fir::new(&lp), Fir::new(&lp));
        let (mut pm, mut ps, mut c38) = (0.0f64, 0.0f64, (0.0f64, 0.0f64));
        let w = 2.0 * PI * pilot_f / fs;
        for (i, &v) in body.iter().enumerate() {
            let ph = w * (skip + i) as f64 + pilot_ph;
            let m = fm.process(v);
            let s = fsub.process(v * 2.0 * (2.0 * ph).sin() as f32);
            pm = pm.max(m.abs() as f64);
            ps = ps.max(s.abs() as f64);
            c38.0 += v as f64 * (2.0 * ph).cos();
            c38.1 += v as f64 * (2.0 * ph).sin();
        }
        let resid = 2.0 * (c38.0.hypot(c38.1)) / body.len() as f64;
        println!("Stereo:     M peak {:.1} kHz, S peak {:.1} kHz", pm / 1e3, ps / 1e3);
        println!("38 kHz:     residual carrier {:.1} Hz ({:.3} %, limit 1 %)", resid, resid / 750.0);
    } else {
        println!("Pilot:      none (mono)");
    }

    let d = decode_mpx(&x, fs);
    if d.blocks_total == 0 {
        println!("RDS:        not detected");
        return Ok(());
    }
    let st = &d.station;
    println!(
        "RDS:        level {:.0} Hz, subcarrier offset {:+.2} Hz, {} groups, block error rate {:.2} %",
        d.level,
        d.carrier_offset,
        d.groups.len(),
        100.0 * (d.blocks_total - d.blocks_ok) as f64 / d.blocks_total as f64
    );
    if let Some(pi) = st.pi {
        println!("  PI {pi:04X}  PTY {}  TP {}  TA {}  MS {}", st.pty.unwrap_or(0), st.tp.unwrap_or(false) as u8, st.ta.unwrap_or(false) as u8, st.ms.unwrap_or(false) as u8);
    }
    println!("  PS   \"{}\"", st.ps_string());
    if !st.rt.is_empty() {
        println!("  RT   \"{}\"", st.rt_string());
    }
    if st.ptyn.iter().any(|c| c.is_some()) {
        println!("  PTYN \"{}\"", st.ptyn_string());
    }
    let di: Vec<String> = st.di.iter().map(|b| b.map_or("?".into(), |b| (b as u8).to_string())).collect();
    println!("  DI   d0..d3 = {}", di.join(""));
    if !st.af.is_empty() {
        let af: Vec<String> = st.af.iter().map(|&c| format!("{:.1}", 87.5 + c as f64 / 10.0)).collect();
        println!("  AF   {}", af.join(" "));
    }
    if let Some(ecc) = st.ecc {
        println!("  ECC  {ecc:02X}");
    }
    for (mjd, h, m, off) in &st.ct {
        println!("  CT   MJD {mjd} {h:02}:{m:02} UTC, local offset {:+.1} h", *off as f64 / 2.0);
    }
    // Clock-time accuracy: the minute edge must fall within ±0.1 s of the
    // end of the CT group (EN 50067 3.1.5.6 note 4).
    let start = a.start_time.or_else(|| {
        let side = std::fs::read_to_string(format!("{}.json", a.file.display())).ok()?;
        let v = side.split("\"start_time\"").nth(1)?.trim_start_matches([':', ' ']);
        v.split([',', '}']).next()?.trim().parse().ok()
    });
    if let Some(t0) = start {
        for (g, &t) in d.groups.iter().zip(&d.group_end_times) {
            if g.group_type() == txengine_rds::GroupType::a(4) {
                let mjd = ((g.blocks[1] as u32 & 3) << 15) | (g.blocks[2] as u32 >> 1);
                let hour = (((g.blocks[2] & 1) << 4) | (g.blocks[3] >> 12)) as i64;
                let min = ((g.blocks[3] >> 6) & 0x3f) as i64;
                let edge = (mjd as i64 - 40_587) * 86_400 + hour * 3600 + min * 60;
                let err = t0 + t - edge as f64;
                println!("  CT   group ended {:+.1} ms from the minute edge it announces (limit ±100 ms)", err * 1e3);
            }
        }
    }
    let groups: Vec<String> = st.group_counts.iter().map(|(k, v)| format!("{k}:{v}")).collect();
    println!("  groups {}", groups.join(" "));
    Ok(())
}
