//! Throughput of the multiplex generator and its parts (run on the target
//! board: `cargo build --release --example bench`).
use std::hint::black_box;
use std::time::Instant;

use txengine_dsp::{PhaseAccumulator, SineTable};
use txengine_fm::mpx::{AudioFeed, MpxConfig, MpxGenerator};
use txengine_rds::{RdsEncoder, RdsModulator, RdsParams};

struct Noise(u32);
impl AudioFeed for Noise {
    fn read(&mut self, out: &mut [[f32; 2]]) -> usize {
        for o in out.iter_mut() {
            self.0 = self.0.wrapping_mul(1664525).wrapping_add(1013904223);
            let v = (self.0 >> 9) as f32 / (1 << 23) as f32 - 1.0;
            *o = [0.3 * v, -0.3 * v];
        }
        out.len()
    }
}

fn report(name: &str, dt: f64, n: usize, fs: f64) {
    println!(
        "{name:28} {:7.1} ns/sample  {:5.1} % of one core at {:.0} kS/s",
        dt / n as f64 * 1e9,
        dt / (n as f64 / fs) * 100.0,
        fs / 1e3
    );
}

fn main() {
    let fs = 250_000.0;
    let n = 1_250_000usize;
    let mut enc = RdsEncoder::new(RdsParams::default()).unwrap();

    let mut acc = PhaseAccumulator::new(19_000.0, fs);
    let sin = SineTable::new();
    let t = Instant::now();
    let mut sum = 0f32;
    for _ in 0..n {
        sum += sin.sin(acc.harmonic_frac(1)) + sin.sin(acc.harmonic_frac(2)) + sin.sin(acc.harmonic_frac(3));
        acc.advance();
    }
    black_box(sum);
    report("3 subcarriers", t.elapsed().as_secs_f64(), n, fs);

    let mut m = RdsModulator::new();
    let mut bb = vec![0f32; 500];
    let t = Instant::now();
    for _ in 0..n / bb.len() {
        m.render(&acc, &mut bb, &mut enc, 0.0, 1.0 / fs);
        acc.advance_by(bb.len() as u64);
        black_box(&bb);
    }
    report("rds baseband", t.elapsed().as_secs_f64(), n, fs);

    for (name, stereo, rds, lim, comp) in [
        ("mpx mono", false, 0.0, true, true),
        ("mpx stereo", true, 0.0, true, true),
        ("mpx stereo, no limiter", true, 0.0, false, true),
        ("mpx stereo, no lim/comp", true, 0.0, false, false),
        ("mpx stereo + rds", true, 2000.0, true, true),
    ] {
        let cfg = MpxConfig { rds_deviation: rds, limiter: lim, hold_compensation: comp, ..Default::default() };
        let mut g = MpxGenerator::new(cfg, fs, stereo).unwrap();
        g.set_rds_enabled(rds > 0.0);
        let mut out = vec![0f32; 1000];
        let mut audio = Noise(1);
        let t = Instant::now();
        let mut done = 0;
        while done < n {
            g.render(&mut out, &mut audio, &mut enc, 0.0);
            done += out.len();
        }
        black_box(&out);
        report(name, t.elapsed().as_secs_f64(), done, fs);
    }
}
