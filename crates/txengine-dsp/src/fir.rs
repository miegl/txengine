//! FIR filter design and filtering.

use crate::window::{kaiser, kaiser_beta, kaiser_length};

/// FIR design helpers. All frequencies are in Hz.
#[derive(Debug, Clone, Copy)]
pub struct FirDesign;

impl FirDesign {
    /// Kaiser-windowed sinc low-pass with unity DC gain.
    ///
    /// The -6 dB point is in the middle of the transition band
    /// `[pass, stop]`; `atten_db` is the stop-band attenuation.
    pub fn lowpass(fs: f64, pass: f64, stop: f64, atten_db: f64) -> Vec<f64> {
        assert!(0.0 < pass && pass < stop && stop <= fs / 2.0 + 1e-9, "invalid low-pass edges");
        let n = kaiser_length(atten_db, (stop - pass) / fs);
        Self::lowpass_n(fs, (pass + stop) / 2.0, n, kaiser_beta(atten_db))
    }

    /// Windowed-sinc low-pass of exactly `n` taps (n odd) with cutoff `fc`.
    pub fn lowpass_n(fs: f64, fc: f64, n: usize, beta: f64) -> Vec<f64> {
        assert!(n % 2 == 1, "use an odd number of taps for linear phase");
        let w = kaiser(n, beta);
        let m = (n / 2) as f64;
        let wc = fc / fs;
        let mut h: Vec<f64> = (0..n).map(|i| 2.0 * wc * crate::sinc(2.0 * wc * (i as f64 - m)) * w[i]).collect();
        let sum: f64 = h.iter().sum();
        for x in &mut h {
            *x /= sum;
        }
        h
    }

    /// Complex frequency response of `taps` at frequency `f`, as (magnitude, phase).
    pub fn response(taps: &[f64], f: f64, fs: f64) -> (f64, f64) {
        let w = 2.0 * std::f64::consts::PI * f / fs;
        let (mut re, mut im) = (0.0, 0.0);
        for (k, &h) in taps.iter().enumerate() {
            re += h * (w * k as f64).cos();
            im -= h * (w * k as f64).sin();
        }
        ((re * re + im * im).sqrt(), im.atan2(re))
    }

    /// Symmetric FIR (odd length `n`) approximating `1 / sinc(f / fs)` on
    /// `[0, f_max]` in the least-squares sense: compensates the droop of a
    /// zero-order hold at sample rate `fs`.
    pub fn inverse_sinc(n: usize, fs: f64, f_max: f64) -> Vec<f64> {
        Self::inverse_response(n, fs, f_max, |f| crate::sinc(f / fs))
    }

    /// Symmetric FIR (odd length `n`) approximating `1 / response(f)` on
    /// `[0, f_max]` in the least-squares sense (an equaliser for a
    /// zero-phase magnitude response, e.g. a hold plus a measured
    /// correction).
    pub fn inverse_response(n: usize, fs: f64, f_max: f64, response: impl Fn(f64) -> f64) -> Vec<f64> {
        assert!(n % 2 == 1);
        let m = n / 2;
        let grid = 400;
        // Basis: H(f) = c0 + sum_k 2 ck cos(2 pi f k / fs)
        let mut ata = vec![vec![0.0; m + 1]; m + 1];
        let mut atb = vec![0.0; m + 1];
        for g in 0..=grid {
            let f = f_max * g as f64 / grid as f64;
            let target = 1.0 / response(f);
            let basis: Vec<f64> = (0..=m)
                .map(|k| if k == 0 { 1.0 } else { 2.0 * (2.0 * std::f64::consts::PI * f * k as f64 / fs).cos() })
                .collect();
            for i in 0..=m {
                atb[i] += basis[i] * target;
                for j in 0..=m {
                    ata[i][j] += basis[i] * basis[j];
                }
            }
        }
        let c = solve(ata, atb);
        let mut h = vec![0.0; n];
        h[m] = c[0];
        for k in 1..=m {
            h[m - k] = c[k];
            h[m + k] = c[k];
        }
        h
    }
}

/// Solve a small dense linear system with partial pivoting.
pub(crate) fn solve(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Vec<f64> {
    let n = b.len();
    for col in 0..n {
        let piv = (col..n).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs())).unwrap();
        a.swap(col, piv);
        b.swap(col, piv);
        let d = a[col][col];
        let (upper, lower) = a.split_at_mut(col + 1);
        let pivot = &upper[col];
        for (r, row) in lower.iter_mut().enumerate() {
            let f = row[col] / d;
            for (x, p) in row[col..].iter_mut().zip(&pivot[col..]) {
                *x -= f * p;
            }
            b[col + 1 + r] -= f * b[col];
        }
    }
    let mut x = vec![0.0; n];
    for row in (0..n).rev() {
        let s: f64 = (row + 1..n).map(|k| a[row][k] * x[k]).sum();
        x[row] = (b[row] - s) / a[row][row];
    }
    x
}

/// Streaming single-channel FIR filter.
///
/// Uses a doubled delay line so the convolution always reads one
/// contiguous slice (no modulo in the inner loop). The taps are padded with
/// zeros to a multiple of 8 for [`dot`].
#[derive(Debug, Clone)]
pub struct Fir {
    taps: Vec<f32>,
    buf: Vec<f32>,
    pos: usize,
    len: usize,
}

impl Fir {
    pub fn new(taps: &[f64]) -> Fir {
        let len = taps.len();
        let n = len.div_ceil(8) * 8;
        // Stored reversed so that the newest sample multiplies the last tap;
        // the padding multiplies the oldest samples.
        let mut t = vec![0f32; n - len];
        t.extend(taps.iter().rev().map(|&t| t as f32));
        Fir { taps: t, buf: vec![0.0; 2 * n], pos: 0, len }
    }

    /// Number of taps of the filter.
    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Group delay in samples.
    pub fn delay(&self) -> usize {
        self.len / 2
    }

    #[inline]
    pub fn push(&mut self, x: f32) {
        let n = self.taps.len();
        self.buf[self.pos] = x;
        self.buf[self.pos + n] = x;
        self.pos += 1;
        if self.pos == n {
            self.pos = 0;
        }
    }

    /// Output for the samples pushed so far.
    #[inline]
    pub fn output(&self) -> f32 {
        let n = self.taps.len();
        dot(&self.taps, &self.buf[self.pos..self.pos + n])
    }

    #[inline]
    pub fn process(&mut self, x: f32) -> f32 {
        self.push(x);
        self.output()
    }

    /// Filter `data` in place.
    pub fn process_block(&mut self, data: &mut [f32]) {
        for x in data {
            *x = self.process(*x);
        }
    }

    pub fn reset(&mut self) {
        self.buf.fill(0.0);
        self.pos = 0;
    }
}

/// Block FIR filter for linear-phase (symmetric) responses, for one or two
/// channels sharing the taps.
///
/// The symmetry halves the multiplications (`h[k] (x[k] + x[n-1-k])`), and
/// filtering a block at a time from a linear history buffer avoids
/// per-sample bookkeeping.
#[derive(Debug, Clone)]
pub struct LinearPhaseFir {
    /// First half of the (reversed) taps; `half[k]` pairs history samples
    /// `k` and `n - 1 - k` (oldest first).
    half: Vec<f32>,
    /// Centre tap of an odd-length filter.
    mid: Option<f32>,
    n: usize,
    /// Per channel: `n - 1` samples of history followed by the block.
    buf: [Vec<f32>; 2],
}

impl LinearPhaseFir {
    /// Panics unless `taps` is symmetric.
    pub fn new(taps: &[f64]) -> LinearPhaseFir {
        let n = taps.len();
        assert!(n >= 1);
        for k in 0..n / 2 {
            assert!((taps[k] - taps[n - 1 - k]).abs() <= 1e-9 * taps[k].abs().max(1e-12), "taps are not symmetric");
        }
        LinearPhaseFir {
            half: taps[..n / 2].iter().map(|&t| t as f32).collect(),
            mid: (n % 2 == 1).then(|| taps[n / 2] as f32),
            n,
            buf: [vec![0.0; n - 1], vec![0.0; n - 1]],
        }
    }

    pub fn len(&self) -> usize {
        self.n
    }

    pub fn is_empty(&self) -> bool {
        false
    }

    /// Group delay in samples.
    pub fn delay(&self) -> f64 {
        (self.n - 1) as f64 / 2.0
    }

    fn load(buf: &mut Vec<f32>, n: usize, data: &[f32]) {
        buf.truncate(n - 1);
        buf.extend_from_slice(data);
    }

    fn keep_history(buf: &mut Vec<f32>, n: usize) {
        let len = buf.len();
        buf.copy_within(len - (n - 1).., 0);
        buf.truncate(n - 1);
    }

    /// Filter `data` (channel 0) in place.
    pub fn process(&mut self, data: &mut [f32]) {
        let n = self.n;
        Self::load(&mut self.buf[0], n, data);
        let x = &self.buf[0];
        for (i, y) in data.iter_mut().enumerate() {
            let w = &x[i..i + n];
            let mut v = folded_dot(&self.half, w);
            if let Some(m) = self.mid {
                v += m * w[n / 2];
            }
            *y = v;
        }
        Self::keep_history(&mut self.buf[0], n);
    }

    /// Filter `a` (channel 0) and `b` (channel 1) in place; equal lengths.
    pub fn process2(&mut self, a: &mut [f32], b: &mut [f32]) {
        let n = self.n;
        let len = a.len().min(b.len());
        Self::load(&mut self.buf[0], n, &a[..len]);
        Self::load(&mut self.buf[1], n, &b[..len]);
        let (xa, xb) = (&self.buf[0], &self.buf[1]);
        for i in 0..len {
            let (wa, wb) = (&xa[i..i + n], &xb[i..i + n]);
            let (mut va, mut vb) = folded_dot2(&self.half, wa, wb);
            if let Some(m) = self.mid {
                va += m * wa[n / 2];
                vb += m * wb[n / 2];
            }
            a[i] = va;
            b[i] = vb;
        }
        Self::keep_history(&mut self.buf[0], n);
        Self::keep_history(&mut self.buf[1], n);
    }
}

/// `Σ half[k] (w[k] + w[len - 1 - k])`. Iterators instead of indexing keep
/// bounds checks out of the loop (they cost a lot on in-order cores).
#[inline(always)]
fn folded_dot(half: &[f32], w: &[f32]) -> f32 {
    let m = half.len();
    let (lo, hi) = (&w[..m], &w[w.len() - m..]);
    let (mut a0, mut a1) = (0f32, 0f32);
    let (ch, cl, chi) = (half.chunks_exact(2), lo.chunks_exact(2), hi.rchunks_exact(2));
    if m % 2 == 1 {
        a0 = half[m - 1] * (lo[m - 1] + hi[0]);
    }
    for ((h, l), r) in ch.zip(cl).zip(chi) {
        a0 += h[0] * (l[0] + r[1]);
        a1 += h[1] * (l[1] + r[0]);
    }
    a0 + a1
}

/// [`folded_dot`] for two channels.
#[inline(always)]
fn folded_dot2(half: &[f32], wa: &[f32], wb: &[f32]) -> (f32, f32) {
    let m = half.len();
    let (la, ha) = (&wa[..m], &wa[wa.len() - m..]);
    let (lb, hb) = (&wb[..m], &wb[wb.len() - m..]);
    let (mut a0, mut a1, mut b0, mut b1) = (0f32, 0f32, 0f32, 0f32);
    if m % 2 == 1 {
        a0 = half[m - 1] * (la[m - 1] + ha[0]);
        b0 = half[m - 1] * (lb[m - 1] + hb[0]);
    }
    let it = half
        .chunks_exact(2)
        .zip(la.chunks_exact(2))
        .zip(lb.chunks_exact(2))
        .zip(ha.rchunks_exact(2))
        .zip(hb.rchunks_exact(2));
    for ((((h, la), lb), ha), hb) in it {
        a0 += h[0] * (la[0] + ha[1]);
        b0 += h[0] * (lb[0] + hb[1]);
        a1 += h[1] * (la[1] + ha[0]);
        b1 += h[1] * (lb[1] + hb[0]);
    }
    (a0 + a1, b0 + b1)
}

/// Dot product of `a` with the first `a.len()` elements of `b`.
///
/// Eight independent accumulators let in-order cores (Cortex-A7/A53)
/// overlap the floating-point latencies, and let the compiler vectorise
/// where floating-point SIMD is IEEE compliant (AArch64, x86).
#[inline]
pub fn dot(a: &[f32], b: &[f32]) -> f32 {
    let b = &b[..a.len()];
    let mut acc = [0f32; 8];
    let (ca, cb) = (a.chunks_exact(8), b.chunks_exact(8));
    let mut tail = 0.0;
    for (x, y) in ca.remainder().iter().zip(cb.remainder()) {
        tail += x * y;
    }
    for (x, y) in ca.zip(cb) {
        for j in 0..8 {
            acc[j] += x[j] * y[j];
        }
    }
    (((acc[0] + acc[4]) + (acc[1] + acc[5])) + ((acc[2] + acc[6]) + (acc[3] + acc[7]))) + tail
}

/// Integer-factor polyphase interpolator for one channel.
///
/// The taps are interleaved by phase (`taps[k * factor + p]` is tap `k` of
/// branch `p`), so one pass over the history computes all `factor` outputs
/// with independent accumulators.
#[derive(Debug, Clone)]
pub struct PolyphaseInterpolator {
    factor: usize,
    taps: Vec<f32>,
    buf: Vec<f32>,
    pos: usize,
    len: usize,
}

impl PolyphaseInterpolator {
    /// `taps` is the prototype low-pass designed at the *output* rate with
    /// unity DC gain; it is scaled by `factor` internally.
    pub fn new(factor: usize, taps: &[f64]) -> PolyphaseInterpolator {
        assert!(factor >= 1);
        let len = taps.len().div_ceil(factor);
        // Branch p, tap k (k = 0 multiplies the newest input) is
        // taps[k * factor + p]. History is stored oldest first, so store the
        // branch taps in reverse order of k.
        let mut t = vec![0f32; len * factor];
        for (i, &h) in taps.iter().enumerate() {
            let (p, k) = (i % factor, i / factor);
            t[(len - 1 - k) * factor + p] = (h * factor as f64) as f32;
        }
        PolyphaseInterpolator { factor, taps: t, buf: vec![0.0; 2 * len], pos: 0, len }
    }

    pub fn factor(&self) -> usize {
        self.factor
    }

    /// Group delay in output samples.
    pub fn delay(&self) -> f64 {
        (self.len * self.factor - 1) as f64 / 2.0
    }

    /// Push one input sample and write `factor` output samples to `out`.
    #[inline]
    pub fn process(&mut self, x: f32, out: &mut [f32]) {
        self.buf[self.pos] = x;
        self.buf[self.pos + self.len] = x;
        self.pos += 1;
        if self.pos == self.len {
            self.pos = 0;
        }
        let hist = &self.buf[self.pos..self.pos + self.len];
        let out = &mut out[..self.factor];
        match self.factor {
            2 => branches::<2>(&self.taps, hist, out),
            3 => branches::<3>(&self.taps, hist, out),
            4 => branches::<4>(&self.taps, hist, out),
            5 => branches::<5>(&self.taps, hist, out),
            6 => branches::<6>(&self.taps, hist, out),
            7 => branches::<7>(&self.taps, hist, out),
            8 => branches::<8>(&self.taps, hist, out),
            f => {
                out.fill(0.0);
                for (t, &x) in self.taps.chunks_exact(f).zip(hist) {
                    for (o, &t) in out.iter_mut().zip(t) {
                        *o += t * x;
                    }
                }
            }
        }
    }
}

#[inline(always)]
fn branches<const F: usize>(taps: &[f32], hist: &[f32], out: &mut [f32]) {
    let mut acc = [0f32; F];
    for (t, &x) in taps.chunks_exact(F).zip(hist) {
        for p in 0..F {
            acc[p] += t[p] * x;
        }
    }
    out.copy_from_slice(&acc);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::db;

    #[test]
    fn lowpass_meets_spec() {
        let fs = 50_000.0;
        let h = FirDesign::lowpass(fs, 15_000.0, 18_500.0, 80.0);
        for f in [0.0, 1000.0, 10_000.0, 15_000.0] {
            let (m, _) = FirDesign::response(&h, f, fs);
            assert!(db(m).abs() < 0.01, "passband ripple at {f}: {} dB", db(m));
        }
        for f in [18_500.0, 19_000.0, 22_000.0, 25_000.0] {
            let (m, _) = FirDesign::response(&h, f, fs);
            assert!(db(m) < -78.0, "stopband at {f}: {} dB", db(m));
        }
    }

    #[test]
    fn fir_streaming_matches_convolution() {
        let taps = [0.1, 0.2, 0.4, 0.2, 0.1];
        let mut f = Fir::new(&taps);
        let x: Vec<f32> = (0..20).map(|i| ((i * 7) % 5) as f32 - 2.0).collect();
        for n in 0..x.len() {
            let y = f.process(x[n]);
            let mut r = 0.0;
            for (k, &t) in taps.iter().enumerate() {
                if n >= k {
                    r += t as f32 * x[n - k];
                }
            }
            assert!((y - r).abs() < 1e-6);
        }
    }

    #[test]
    fn linear_phase_matches_direct_form() {
        for taps in [
            FirDesign::lowpass(50_000.0, 15_000.0, 18_500.0, 80.0),
            FirDesign::inverse_sinc(9, 250_000.0, 62_000.0),
            vec![0.25, 0.5, 0.5, 0.25],
        ] {
            let mut d0 = Fir::new(&taps);
            let mut d1 = Fir::new(&taps);
            let mut lp = LinearPhaseFir::new(&taps);
            let mut lp1 = LinearPhaseFir::new(&taps);
            let x: Vec<f32> = (0..1000).map(|i| ((i * 7919) % 1000) as f32 * 1e-3 - 0.5).collect();
            let y: Vec<f32> = x.iter().map(|&v| -0.5 * v + 0.1).collect();
            for block in [1usize, 7, 64, 300] {
                let (mut a, mut b) = (x[..block].to_vec(), y[..block].to_vec());
                let mut c = a.clone();
                lp.process2(&mut a, &mut b);
                lp1.process(&mut c);
                for i in 0..block {
                    let (ea, eb) = (d0.process(x[i]), d1.process(y[i]));
                    assert!(
                        (a[i] - ea).abs() < 1e-5 && (b[i] - eb).abs() < 1e-5 && (c[i] - ea).abs() < 1e-5,
                        "block {block} i {i}"
                    );
                }
            }
        }
    }

    #[test]
    fn interpolator_reproduces_sine() {
        let (fa, l) = (50_000.0, 5usize);
        let fs = fa * l as f64;
        let h = FirDesign::lowpass(fs, 15_000.0, fa - 15_000.0, 90.0);
        let mut ip = PolyphaseInterpolator::new(l, &h);
        let f = 7_000.0;
        let mut out = Vec::new();
        let mut buf = [0f32; 5];
        for n in 0..4000 {
            ip.process((2.0 * std::f64::consts::PI * f * n as f64 / fa).sin() as f32, &mut buf);
            out.extend_from_slice(&buf);
        }
        // Compare against the ideal sine (accounting for the filter delay).
        let delay = (h.len() - 1) as f64 / 2.0;
        let mut err: f64 = 0.0;
        for (i, &y) in out.iter().enumerate().skip(2000) {
            let t = (i as f64 - delay) / fs;
            err = err.max((y as f64 - (2.0 * std::f64::consts::PI * f * t).sin()).abs());
        }
        assert!(err < 1e-3, "interpolation error {err}");
    }

    #[test]
    fn inverse_response_flattens_measured_response() {
        // A hold plus a rising correction (shape of the Raspberry Pi 2 PLL).
        let corr = [(0.0, 0.0), (19e3, 0.16), (38e3, 0.45), (57e3, 0.65), (67e3, 0.69)];
        for fs in [150e3, 195e3, 250e3] {
            let resp = |f: f64| {
                let i = corr.iter().position(|&(x, _)| x > f).unwrap_or(corr.len() - 1).max(1);
                let ((xa, ya), (xb, yb)) = (corr[i - 1], corr[i]);
                let db = ya + (yb - ya) * ((f - xa) / (xb - xa)).clamp(0.0, 1.0);
                crate::sinc(f / fs) * 10f64.powf(db / 20.0)
            };
            let h = FirDesign::inverse_response(11, fs, 62_000.0, resp);
            let worst = (0..=62)
                .map(|k| k as f64 * 1e3)
                .map(|f| db(FirDesign::response(&h, f, fs).0 * resp(f)).abs())
                .fold(0.0, f64::max);
            assert!(worst < 0.03, "fs {fs}: residual {worst} dB");
        }
    }

    #[test]
    fn inverse_sinc_flattens_zoh() {
        let fs = 250_000.0;
        let h = FirDesign::inverse_sinc(9, fs, 62_000.0);
        for f in [0.0, 19_000.0, 38_000.0, 57_000.0, 60_000.0] {
            let (m, _) = FirDesign::response(&h, f, fs);
            let total = m * crate::sinc(f / fs);
            assert!(db(total).abs() < 0.02, "residual at {f}: {} dB", db(total));
        }
    }
}
