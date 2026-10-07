//! FM pre-emphasis (ITU-R BS.450: 50 µs in Europe, 75 µs in the Americas).
//!
//! The analogue characteristic is that of a parallel RC admittance,
//! `H(f) = 1 + j 2 pi f tau`. It is realised as a first-order minimum-phase
//! IIR shelf whose pole and zero are fitted at start-up (minimax in dB over
//! 20 Hz – 15 kHz) for the actual sample rate. The fit is within ~0.05 dB of
//! the analogue curve at 44.1–52 kHz sample rates.

use crate::optimize::nelder_mead;

/// Stereo pre-emphasis filter.
#[derive(Debug, Clone)]
pub struct Preemphasis {
    g: f32,
    a: f32,
    b: f32,
    x1: [f32; 2],
    y1: [f32; 2],
    /// Maximum deviation of the fit from the analogue curve (dB).
    pub fit_error_db: f64,
}

/// Coefficients `(a, b)` of `g (1 - b z^-1) / (1 - a z^-1)` with unity DC gain.
fn response_db(a: f64, b: f64, f: f64, fs: f64) -> f64 {
    let w = 2.0 * std::f64::consts::PI * f / fs;
    let (c, s) = (w.cos(), w.sin());
    let g = (1.0 - a) / (1.0 - b);
    let num = (1.0 - b * c).powi(2) + (b * s).powi(2);
    let den = (1.0 - a * c).powi(2) + (a * s).powi(2);
    10.0 * (g * g * num / den).log10()
}

impl Preemphasis {
    /// Pre-emphasis with time constant `tau` (seconds) at sample rate `fs`,
    /// accurate up to `f_max` Hz.
    pub fn new(tau: f64, fs: f64, f_max: f64) -> Preemphasis {
        let grid: Vec<f64> = (0..=200).map(|i| 20.0 + (f_max - 20.0) * i as f64 / 200.0).collect();
        let target: Vec<f64> = grid
            .iter()
            .map(|&f| 10.0 * (1.0 + (2.0 * std::f64::consts::PI * f * tau).powi(2)).log10())
            .collect();
        let cost = |p: &[f64]| -> f64 {
            let (a, b) = (p[0], p[1]);
            if !(-0.95..0.95).contains(&a) || !(0.0..0.999).contains(&b) {
                return 1e9;
            }
            grid.iter().zip(&target).map(|(&f, &t)| (response_db(a, b, f, fs) - t).abs()).fold(0.0, f64::max)
        };
        // Start from a few points; the problem is smooth and two-dimensional.
        let mut best = (vec![0.0, 0.0], f64::INFINITY);
        for &(a0, b0) in &[(-0.13, 0.66), (-0.13, 0.76), (0.0, 0.5), (-0.3, 0.8), (0.2, 0.9)] {
            let r = nelder_mead(cost, &[a0, b0], 0.05, 2000);
            if r.1 < best.1 {
                best = r;
            }
        }
        let (a, b) = (best.0[0], best.0[1]);
        Preemphasis {
            g: ((1.0 - a) / (1.0 - b)) as f32,
            a: a as f32,
            b: b as f32,
            x1: [0.0; 2],
            y1: [0.0; 2],
            fit_error_db: best.1,
        }
    }

    /// Gain of the filter at `f` (linear), for headroom calculations.
    pub fn gain_at(&self, f: f64, fs: f64) -> f64 {
        crate::from_db(response_db(self.a as f64, self.b as f64, f, fs))
    }

    /// Gain in dB at `f` with only the fraction `k` of the boost applied,
    /// `1 + k (H - 1)` (as the [`crate::PreemphasisLimiter`] does).
    pub fn partial_gain_db(&self, k: f64, f: f64, fs: f64) -> f64 {
        let w = 2.0 * std::f64::consts::PI * f / fs;
        let (c, s) = (w.cos(), w.sin());
        let (g, a, b) = (self.g as f64, self.a as f64, self.b as f64);
        // H = g (1 - b z^-1) / (1 - a z^-1) at z^-1 = c - js.
        let (nr, ni) = (g * (1.0 - b * c), g * b * s);
        let (dr, di) = (1.0 - a * c, a * s);
        let den = dr * dr + di * di;
        let (hr, hi) = ((nr * dr + ni * di) / den, (ni * dr - nr * di) / den);
        crate::db((1.0 + k * (hr - 1.0)).hypot(k * hi))
    }

    /// How much the filter raises the power of pink noise band-limited to
    /// `f_lo` – `f_hi` Hz, in dB (the mean of the power response over
    /// log-frequency). Programme material is rarely brighter than pink
    /// noise, so this bounds the rise of its power.
    pub fn pink_power_gain_db(&self, fs: f64, f_lo: f64, f_hi: f64) -> f64 {
        const N: usize = 2000;
        let r = (f_hi / f_lo).ln();
        let p = |i: usize| crate::from_db(2.0 * response_db(self.a as f64, self.b as f64, f_lo * (r * i as f64 / N as f64).exp(), fs));
        // Trapezoidal rule on a uniform log-frequency grid.
        let sum: f64 = (1..N).map(p).sum::<f64>() + 0.5 * (p(0) + p(N));
        10.0 * (sum / N as f64).log10()
    }

    /// Process one sample of channel `ch` (0 or 1).
    #[inline]
    pub fn process(&mut self, ch: usize, x: f32) -> f32 {
        let y = self.g * (x - self.b * self.x1[ch]) + self.a * self.y1[ch];
        self.x1[ch] = x;
        self.y1[ch] = y;
        y
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_analogue_curve() {
        for tau in [50e-6, 75e-6] {
            for fs in [44_100.0, 45_600.0, 48_000.0, 50_000.0] {
                let p = Preemphasis::new(tau, fs, 15_000.0);
                assert!(p.fit_error_db < 0.06, "tau {tau} fs {fs}: {} dB", p.fit_error_db);
                // 50 us: +13.7 dB at 15 kHz; 75 us: +17.1 dB.
                let want = 10.0 * (1.0 + (2.0 * std::f64::consts::PI * 15_000.0 * tau).powi(2)).log10();
                let got = crate::db(p.gain_at(15_000.0, fs));
                assert!((got - want).abs() < 0.06);
            }
        }
    }

    #[test]
    fn pink_noise_power_gain() {
        // Analogue curve, 20 Hz - 15 kHz: 4.28 dB (50 us), 6.79 dB (75 us).
        for (tau, want) in [(50e-6, 4.28), (75e-6, 6.79)] {
            let p = Preemphasis::new(tau, 50_000.0, 15_000.0);
            let got = p.pink_power_gain_db(50_000.0, 20.0, 15_000.0);
            assert!((got - want).abs() < 0.03, "tau {tau}: {got} dB");
        }
    }

    #[test]
    fn partial_gain() {
        let (fs, f) = (50_000.0, 15_000.0);
        let p = Preemphasis::new(50e-6, fs, 15_000.0);
        assert!((p.partial_gain_db(1.0, f, fs) - crate::db(p.gain_at(f, fs))).abs() < 1e-4);
        assert!(p.partial_gain_db(0.0, f, fs).abs() < 1e-12);
        let half = p.partial_gain_db(0.5, f, fs);
        assert!(half > 0.0 && half < p.partial_gain_db(1.0, f, fs));
    }

    #[test]
    fn dc_gain_is_unity() {
        let mut p = Preemphasis::new(50e-6, 50_000.0, 15_000.0);
        let mut y = 0.0;
        for _ in 0..10_000 {
            y = p.process(0, 0.5);
        }
        assert!((y - 0.5).abs() < 1e-5);
    }
}
