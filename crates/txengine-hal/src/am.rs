//! Amplitude modulation by pulse width, for the backends that make AM with
//! a PWM peripheral.
//!
//! A GPIO only switches between two levels, so the carrier's amplitude has
//! to come from the shape of the waveform. A pulse of width `w` in every
//! carrier period `P`, centred on the same instant of each period, has a
//! fundamental of amplitude `(4/π)·sin(π w / P) / 2` (re the swing) whose
//! phase does not depend on `w`. The amplitude law is therefore
//! `a = sin(π w / P)`: nothing at `w = 0`, the full square wave at
//! `w = P / 2`. (A pulse that starts at the start of the period instead
//! moves the fundamental's phase with its width, by up to 90°: incidental
//! phase modulation.) Widths `w` and `P - w` give the same amplitude.
//!
//! The hardware has a limited number of widths (on the Raspberry Pi 5, 100
//! steps up to the full square wave at 999 kHz, 16 at 6.1 MHz), far too
//! coarse by themselves. A second-order error-feedback quantiser moves the
//! error away from the carrier, where the programme is, towards ±fc/2
//! (noise transfer function `(1 - z⁻¹)²`, at the carrier rate): 4.5 kHz
//! from a 999 kHz carrier it is 63 dB below the unshaped error. There is
//! no dither: the patterns a steady carrier makes repeat at fractions of
//! the carrier frequency, so their lines near the carrier are suppressed by
//! the same noise transfer function.
//!
//! The amplitude samples arrive at a fraction of the carrier rate (each
//! lasts a whole number of carrier cycles); every cycle gets its own width,
//! the amplitude interpolated linearly between the samples. The frequency
//! response of the interpolation is `sinc²(f / fs)` ([`interpolation_response_db`]).

/// Entries of the arcsine table.
const TABLE: usize = 4096;

/// Converts amplitude samples into the pulse widths of their carrier
/// cycles (see the module documentation).
#[derive(Debug, Clone)]
pub struct PulseWidths {
    cycles: usize,
    /// `table[i]`: the width (in steps) for amplitude `i / TABLE`; one
    /// extra entry for the interpolation at `a = 1`.
    table: Vec<f32>,
    max: f32,
    /// The previous amplitude sample.
    prev: f32,
    /// The quantiser's last two errors.
    e1: f32,
    e2: f32,
}

impl PulseWidths {
    /// `cycles`: carrier cycles per amplitude sample; `period`: the carrier
    /// period in width steps (the full square wave is `period / 2`);
    /// `max`: the largest width the hardware takes (at least `period / 2`).
    pub fn new(cycles: usize, period: f64, max: u32) -> PulseWidths {
        assert!(cycles >= 1 && period >= 2.0 && max as f64 >= period / 2.0);
        let mut table: Vec<f32> =
            (0..=TABLE).map(|i| (period * (i as f64 / TABLE as f64).asin() / std::f64::consts::PI) as f32).collect();
        table.push(table[TABLE]);
        PulseWidths { cycles, table, max: max as f32, prev: 0.0, e1: 0.0, e2: 0.0 }
    }

    pub fn cycles(&self) -> usize {
        self.cycles
    }

    /// The width for amplitude `a` (0-1), unquantised.
    #[inline(always)]
    fn width(&self, a: f32) -> f32 {
        let x = a * TABLE as f32;
        let i = x as usize;
        let f = x - i as f32;
        let (w0, w1) = (self.table[i], self.table[i + 1]);
        w0 + (w1 - w0) * f
    }

    /// Write the widths of the carrier cycles of one amplitude sample `a`
    /// (0-1; clamped) to `out` (`cycles` of them).
    #[inline]
    pub fn sample(&mut self, a: f32, out: &mut [u32]) {
        let a = if a > 1.0 {
            1.0
        } else if a > 0.0 {
            a
        } else {
            0.0
        };
        let step = (a - self.prev) / self.cycles as f32;
        let mut x = self.prev;
        for o in &mut out[..self.cycles] {
            x += step;
            let u = self.width(x);
            let v = u - (2.0 * self.e1 - self.e2);
            // Round to the nearest width (v + 0.5 >= 0 truncates like floor).
            let q = ((v + 0.5) as i32 as f32).clamp(0.0, self.max);
            self.e2 = self.e1;
            // A clamped width leaves a large error; bounding it keeps the
            // loop from winding up.
            self.e1 = (q - v).clamp(-2.0, 2.0);
            *o = q as u32;
        }
        self.prev = a;
    }
}

/// The response of the linear interpolation between amplitude samples at
/// `fs`, as `(Hz, dB)` points up to `f_max` (for `SinkInfo::response_db`;
/// the interpolation is not a hold).
pub fn interpolation_response_db(fs: f64, f_max: f64) -> Vec<(f64, f64)> {
    (0..=20)
        .map(|k| {
            let f = f_max * k as f64 / 20.0;
            let x = std::f64::consts::PI * f / fs;
            let sinc = if x == 0.0 { 1.0 } else { x.sin() / x };
            (f, 40.0 * sinc.log10())
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The fundamental's amplitude of every cycle for the amplitude samples
    /// `a`, with a period of `period` steps.
    fn fundamentals(pw: &mut PulseWidths, period: f64, a: &[f32]) -> Vec<f64> {
        let mut out = vec![0u32; pw.cycles()];
        let mut fund = Vec::with_capacity(a.len() * pw.cycles());
        for &s in a {
            pw.sample(s, &mut out);
            fund.extend(out.iter().map(|&w| (std::f64::consts::PI * w as f64 / period).sin()));
        }
        fund
    }

    /// Power of `x` (at `rate`) within `f_lo..f_hi`, by projection onto a
    /// grid of frequencies (rectangular window over whole periods).
    fn band_power(x: &[f64], rate: f64, f_lo: f64, f_hi: f64) -> f64 {
        let n = x.len();
        let df = rate / n as f64;
        let (k0, k1) = ((f_lo / df).ceil() as usize, (f_hi / df) as usize);
        (k0..=k1)
            .map(|k| {
                let (mut re, mut im) = (0.0, 0.0);
                for (i, &v) in x.iter().enumerate() {
                    let p = 2.0 * std::f64::consts::PI * (k * i) as f64 / n as f64;
                    re += v * p.cos();
                    im += v * p.sin();
                }
                2.0 * (re * re + im * im) / (n * n) as f64
            })
            .sum()
    }

    #[test]
    fn widths_follow_the_amplitude_with_little_error_near_the_carrier() {
        // 999 kHz from 399.6 MHz (phase-correct: 200 steps per half period,
        // i.e. a period of 200 width units), 10 cycles per sample.
        let (fc, period, cycles) = (999e3, 200.0, 10);
        let fs = fc / cycles as f64;
        let n = 10_000; // 0.1 s of samples
        let tone = 1_000.0;
        let a: Vec<f32> = (0..n)
            .map(|k| (0.5 * (1.0 + 0.8 * (2.0 * std::f64::consts::PI * tone * k as f64 / fs).sin())) as f32)
            .collect();
        let mut pw = PulseWidths::new(cycles, period, 200);
        let fund = fundamentals(&mut pw, period, &a);
        // The wanted envelope at the carrier rate: the samples interpolated
        // linearly, each reached at its last cycle. The first sample (a
        // ramp up from nothing) is left out.
        let want: Vec<f64> = (cycles..fund.len())
            .map(|i| {
                let (k, j) = (i / cycles, i % cycles);
                let (a0, a1) = (a[k - 1] as f64, a[k] as f64);
                a0 + (a1 - a0) * (j + 1) as f64 / cycles as f64
            })
            .collect();
        let fund = &fund[cycles..];
        let err: Vec<f64> = fund.iter().zip(&want).map(|(f, w)| f - w).collect();
        // Error power within ±5 kHz of the carrier (the envelope's 0-5 kHz),
        // re the carrier (0.5), over 0.05 s.
        let inband = band_power(&err[..49_950], fc, 1.0, 5_000.0);
        let db = 10.0 * (inband / 0.25).log10();
        assert!(db < -85.0, "in-band error {db:.1} dB re the carrier");
        // Unshaped, the error would be about 40 dB higher.
        let total = err.iter().map(|e| e * e).sum::<f64>() / err.len() as f64;
        assert!(10.0 * (total / 0.25).log10() > db + 40.0);
    }

    #[test]
    fn the_extremes() {
        let mut pw = PulseWidths::new(4, 64.0, 32);
        let mut out = [0u32; 4];
        pw.sample(1.5, &mut out);
        pw.sample(1.0, &mut out);
        assert_eq!(out, [32; 4], "full output: the square wave");
        for _ in 0..10 {
            pw.sample(-0.2, &mut out);
        }
        assert_eq!(out, [0; 4], "no carrier");
    }

    #[test]
    fn response() {
        let r = interpolation_response_db(100e3, 10e3);
        assert_eq!(r[0], (0.0, 0.0));
        let (f, db) = r[9];
        assert!((f - 4500.0).abs() < 1e-9 && (db + 0.0577).abs() < 1e-3, "{db}");
    }
}
