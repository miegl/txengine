//! Per-component throughput of the DSP blocks (run on the target board).
use std::hint::black_box;
use std::time::Instant;

use txengine_dsp::*;

fn run(name: &str, n: usize, mut f: impl FnMut(usize) -> f32) {
    let t = Instant::now();
    let mut s = 0.0;
    for i in 0..n {
        s += f(i);
    }
    black_box(s);
    let dt = t.elapsed().as_secs_f64();
    println!("{name:30} {:7.1} ns/call", dt / n as f64 * 1e9);
}

fn main() {
    let n = 1_000_000;
    let x = |i: usize| ((i * 7919) % 1000) as f32 * 1e-3 - 0.5;
    let mut f9 = Fir::new(&FirDesign::inverse_sinc(9, 250_000.0, 62_000.0));
    run("fir 9 taps", n, |i| f9.process(x(i)));
    let lp = FirDesign::lowpass(50_000.0, 15_000.0, 18_500.0, 80.0);
    println!("(lpf has {} taps)", lp.len());
    let mut flp = Fir::new(&lp);
    run("fir lpf", n / 4, |i| flp.process(x(i)));
    let mut lpb = LinearPhaseFir::new(&lp);
    let (mut ba, mut bb) = (vec![0f32; 100], vec![0f32; 100]);
    run("linear-phase lpf x2 (per sample)", n / 400, |i| {
        ba.iter_mut().zip(bb.iter_mut()).enumerate().for_each(|(k, (a, b))| (*a, *b) = (x(i + k), x(i + 2 * k)));
        lpb.process2(&mut ba, &mut bb);
        ba[3] + bb[7]
    });
    println!("  (divide by 100)");
    let mut cb = LinearPhaseFir::new(&FirDesign::inverse_sinc(9, 250_000.0, 62_000.0));
    let mut blk = vec![0f32; 500];
    run("linear-phase 9 taps (per 500)", n / 500, |i| {
        blk.iter_mut().enumerate().for_each(|(k, a)| *a = x(i + k));
        cb.process(&mut blk);
        blk[3]
    });
    let ip = FirDesign::lowpass(250_000.0, 15_500.0, 34_500.0, 90.0);
    println!("(interp prototype {} taps)", ip.len());
    let mut pi = PolyphaseInterpolator::new(5, &ip);
    let mut out = [0f32; 5];
    run("interp x5 (per input)", n / 4, |i| {
        pi.process(x(i), &mut out);
        out[0]
    });
    let mut lim = FrameLimiter::new(5, 50_000.0, 1.0, 0.0015, 0.08);
    let (mut a, mut b) = ([0f32; 5], [0f32; 5]);
    run("limiter (per 5-sample frame)", n / 5, |i| {
        let v = 1.5 * x(i);
        let a_in = [v; 5];
        lim.process(v.abs(), &a_in, &a_in, &mut a, &mut b);
        a[0]
    });
    let mut pre = Preemphasis::new(50e-6, 50_000.0, 15_000.0);
    run("preemphasis", n, |i| pre.process(0, x(i)));
    let mut acc = PhaseAccumulator::new(19_000.0, 250_000.0);
    let sin = SineTable::new();
    run("sine lookup", n, |_| {
        acc.advance();
        sin.sin(acc.harmonic_frac(3))
    });
    run("subdivided", n, |_| {
        acc.advance();
        acc.subdivided(4).1 as f32
    });
}
