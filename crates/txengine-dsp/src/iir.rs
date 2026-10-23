//! Recursive filters: cascades of second-order sections.

use std::f64::consts::PI;

/// Zeros and poles (upper half plane; the rest are their conjugates) of the
/// normalised elliptic low-pass of order 8 with 0.1 dB of pass-band ripple
/// up to 1 rad/s and 60 dB of stop-band attenuation from
/// [`ELLIPTIC_STOP_RATIO`] rad/s (`scipy.signal.ellipap(8, 0.1, 60)`; the
/// tests check that response).
const ELLIPTIC_ZEROS: [f64; 4] = [4.5746269031186655, 1.754118136160681, 1.309585699523888, 1.1914390300529494];
const ELLIPTIC_POLES: [(f64, f64); 4] = [
    (-0.42595527318370724, 0.3009196664683167),
    (-0.25985772870761314, 0.742355698261655),
    (-0.114085062727376, 0.9466995511242549),
    (-0.03032655975056577, 1.0176451518280614),
];
/// Where the stop band of the normalised elliptic low-pass starts.
pub const ELLIPTIC_STOP_RATIO: f64 = 1.1797772030511935;
/// Its pass-band ripple and stop-band attenuation (dB).
pub const ELLIPTIC_RIPPLE_DB: f64 = 0.1;
pub const ELLIPTIC_STOP_DB: f64 = 60.0;

/// Complex numbers, just enough for the filter design.
#[derive(Debug, Clone, Copy, PartialEq)]
struct C(f64, f64);

impl C {
    fn mul(self, o: C) -> C {
        C(self.0 * o.0 - self.1 * o.1, self.0 * o.1 + self.1 * o.0)
    }
    fn div(self, o: C) -> C {
        let d = o.0 * o.0 + o.1 * o.1;
        C((self.0 * o.0 + self.1 * o.1) / d, (self.1 * o.0 - self.0 * o.1) / d)
    }
    fn norm2(self) -> f64 {
        self.0 * self.0 + self.1 * self.1
    }
}

/// One section `(b0 + b1 z⁻¹ + b2 z⁻²) / (1 + a1 z⁻¹ + a2 z⁻²)`, run in
/// transposed direct form II.
#[derive(Debug, Clone)]
struct Section {
    b: [f64; 3],
    a: [f64; 2],
    s: [f64; 2],
}

impl Section {
    /// The section with the digital zeros `z, z*` and poles `p, p*`,
    /// scaled to `gain` at the Nyquist frequency (z = -1).
    fn from_roots(z: C, p: C, gain: f64) -> Section {
        let b = [1.0, -2.0 * z.0, z.norm2()];
        let a = [-2.0 * p.0, p.norm2()];
        // At z = -1 the odd terms change sign.
        let k = gain * (1.0 - a[0] + a[1]) / (b[0] - b[1] + b[2]);
        Section { b: [k * b[0], k * b[1], k * b[2]], a, s: [0.0; 2] }
    }

    #[inline]
    fn process(&mut self, x: f64) -> f64 {
        let y = self.b[0] * x + self.s[0];
        self.s[0] = self.b[1] * x - self.a[0] * y + self.s[1];
        self.s[1] = self.b[2] * x - self.a[1] * y;
        y
    }

    fn response(&self, w: f64) -> C {
        // z⁻¹ = e^{-jw}
        let z1 = C(w.cos(), -w.sin());
        let z2 = z1.mul(z1);
        let num = C(self.b[0] + self.b[1] * z1.0 + self.b[2] * z2.0, self.b[1] * z1.1 + self.b[2] * z2.1);
        let den = C(1.0 + self.a[0] * z1.0 + self.a[1] * z2.0, self.a[0] * z1.1 + self.a[1] * z2.1);
        num.div(den)
    }
}

/// A cascade of second-order sections (state in `f64`: poles close to
/// z = 1, as a high-pass at a few hundred hertz has, need it).
#[derive(Debug, Clone)]
pub struct Biquads {
    sections: Vec<Section>,
}

impl Biquads {
    /// Elliptic high-pass of order 8 at the sample rate `fs`: pass band
    /// from `f_pass` (within 0.1 dB), stop band (at least 60 dB down) from
    /// `f_pass / ELLIPTIC_STOP_RATIO` down to 0 Hz.
    ///
    /// The normalised low-pass becomes a high-pass at the pre-warped edge
    /// `wc` (s → wc / s, so every root r moves to wc / r), then a digital
    /// filter by the bilinear transform (r → (2 fs + r) / (2 fs − r)).
    pub fn elliptic_highpass(fs: f64, f_pass: f64) -> Biquads {
        let wc = 2.0 * fs * (PI * f_pass / fs).tan();
        let t = 2.0 * fs;
        let digital = |r: C| C(t + r.0, r.1).div(C(t - r.0, -r.1));
        let n = ELLIPTIC_ZEROS.len();
        // At s = ∞ (the digital Nyquist frequency) the high-pass passes
        // what the low-pass passes at DC: 0.1 dB below 1 for an even order.
        let nyquist_gain = crate::from_db(-ELLIPTIC_RIPPLE_DB);
        let sections = (0..n)
            .map(|i| {
                let z = digital(C(wc, 0.0).div(C(0.0, ELLIPTIC_ZEROS[i])));
                let (pr, pi) = ELLIPTIC_POLES[i];
                let p = digital(C(wc, 0.0).div(C(pr, pi)));
                Section::from_roots(z, p, if i == 0 { nyquist_gain } else { 1.0 })
            })
            .collect();
        Biquads { sections }
    }

    /// Gain (linear) at `f` Hz for the sample rate `fs`.
    pub fn gain(&self, f: f64, fs: f64) -> f64 {
        let w = 2.0 * PI * f / fs;
        self.sections.iter().fold(C(1.0, 0.0), |h, s| h.mul(s.response(w))).norm2().sqrt()
    }

    /// Filter `data` in place.
    pub fn process(&mut self, data: &mut [f32]) {
        for x in data.iter_mut() {
            let mut v = *x as f64;
            for s in self.sections.iter_mut() {
                v = s.process(v);
            }
            *x = v as f32;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The tabulated prototype is the elliptic filter it claims to be.
    #[test]
    fn prototype_response() {
        let h = |w: f64| -> f64 {
            let s = C(0.0, w);
            let mut v = C(1.0, 0.0);
            for i in 0..4 {
                let (pr, pi) = ELLIPTIC_POLES[i];
                for (zi, p) in [(ELLIPTIC_ZEROS[i], C(pr, pi)), (-ELLIPTIC_ZEROS[i], C(pr, -pi))] {
                    v = v.mul(C(s.0, s.1 - zi)).div(C(s.0 - p.0, s.1 - p.1));
                }
            }
            v.norm2().sqrt()
        };
        // Normalised to DC, which an even-order elliptic low-pass passes at
        // the bottom of its ripple.
        let dc = h(0.0) / crate::from_db(-ELLIPTIC_RIPPLE_DB);
        for i in 0..=1000 {
            let w = i as f64 / 1000.0;
            let db = crate::db(h(w) / dc);
            assert!((-ELLIPTIC_RIPPLE_DB - 1e-6..=1e-6).contains(&db), "{w}: {db}");
        }
        for i in 0..=2000 {
            let w = ELLIPTIC_STOP_RATIO + i as f64 * 0.01;
            let db = crate::db(h(w) / dc);
            assert!(db <= -ELLIPTIC_STOP_DB + 1e-6, "{w}: {db}");
        }
    }

    #[test]
    fn highpass_response() {
        for fs in [16_000.0, 16_667.0, 19_000.0, 24_000.0] {
            let f = Biquads::elliptic_highpass(fs, 300.0);
            for i in 0..=400 {
                let x = 300.0 + (fs / 2.0 - 301.0) * i as f64 / 400.0;
                let db = crate::db(f.gain(x, fs));
                assert!((-ELLIPTIC_RIPPLE_DB - 1e-3..=1e-3).contains(&db), "fs {fs}, {x} Hz: {db} dB");
            }
            let stop = 300.0 / ELLIPTIC_STOP_RATIO;
            for i in 0..=400 {
                let x = 1.0 + (stop - 1.0) * i as f64 / 400.0;
                let db = crate::db(f.gain(x, fs));
                assert!(db < -ELLIPTIC_STOP_DB + 1e-3, "fs {fs}, {x} Hz: {db} dB");
            }
        }
    }

    #[test]
    fn filters_as_designed() {
        let fs = 16_000.0;
        let mut f = Biquads::elliptic_highpass(fs, 300.0);
        let want = f.gain(1000.0, fs);
        // A 1 kHz tone passes, a 200 Hz tone is gone, once settled.
        for (freq, want_db) in [(1000.0, crate::db(want)), (200.0, -60.0)] {
            let mut x: Vec<f32> = (0..32_000).map(|i| (2.0 * PI * freq * i as f64 / fs).sin() as f32).collect();
            f.process(&mut x);
            let tail = &x[16_000..];
            let amp = (2.0 * tail.iter().map(|&v| (v as f64).powi(2)).sum::<f64>() / tail.len() as f64).sqrt();
            if want_db > -10.0 {
                assert!((crate::db(amp) - want_db).abs() < 0.01, "{freq}: {}", crate::db(amp));
            } else {
                assert!(crate::db(amp) < want_db, "{freq}: {}", crate::db(amp));
            }
        }
    }
}
