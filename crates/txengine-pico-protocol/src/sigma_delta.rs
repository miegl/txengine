//! The sigma-delta modulator that dithers the PLL's feedback divider.
//!
//! A MASH 1-1 (two cascaded first-order accumulators): the output
//! `y = c1 + c2 - c2'` is the input plus quantisation noise shaped by
//! `(1 - z⁻¹)²`, so the frequency error of the carrier rises with the
//! square of the offset from the carrier, and its phase error with the
//! offset: within the multiplex band it is tiny, and the PLL's loop filter
//! takes away much of what lies further out.
//!
//! This is the arithmetic the firmware runs (and the host can simulate):
//! the input is an unsigned fixed-point number with [`FRAC_BITS`]
//! fractional bits in `0 ..= 2 << FRAC_BITS`, the output `y` lies in
//! `-1 ..= 3`, and the code sent to the hardware is `y + 1`.
//!
//! [`FRAC_BITS`]: crate::FRAC_BITS

use crate::FRAC_BITS;

const MASK: u32 = (1 << FRAC_BITS) - 1;

/// Second-order MASH modulator state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mash11 {
    acc1: u32,
    acc2: u32,
    c2_prev: u32,
    /// Added to every output: 1 makes it the code (0-4); the firmware
    /// adds the low bits of the feedback divider's base as well.
    bias: u32,
}

impl Default for Mash11 {
    fn default() -> Self {
        Self::new()
    }
}

impl Mash11 {
    pub const fn new() -> Mash11 {
        Mash11::with_bias(1)
    }

    /// A modulator whose outputs are `y + bias` (`bias` ≥ 1).
    pub const fn with_bias(bias: u32) -> Mash11 {
        Mash11 { acc1: 0, acc2: 0, c2_prev: 0, bias }
    }

    /// The accumulators and the previous second-stage carry.
    pub fn state(&self) -> (u32, u32, u32) {
        (self.acc1, self.acc2, self.c2_prev)
    }

    /// Restore [`Mash11::state`].
    pub fn set_state(&mut self, acc1: u32, acc2: u32, c2_prev: u32) {
        (self.acc1, self.acc2, self.c2_prev) = (acc1 & MASK, acc2 & MASK, c2_prev);
    }

    /// One update for input `x`: `y + bias` (with the default bias 1, the
    /// code 0-4).
    #[inline(always)]
    pub fn code(&mut self, x: u32) -> u32 {
        // acc1 < 2^30 and x ≤ 2^31: no overflow.
        let s1 = self.acc1 + x;
        self.acc1 = s1 & MASK;
        let s2 = self.acc2 + self.acc1;
        self.acc2 = s2 & MASK;
        let c2 = s2 >> FRAC_BITS;
        // y + bias = c1 + c2 + bias - c2_prev, never negative.
        let code = (s1 >> FRAC_BITS) + c2 + self.bias - self.c2_prev;
        self.c2_prev = c2;
        code
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ONE: f64 = (1u64 << FRAC_BITS) as f64;

    #[test]
    fn average_is_the_input() {
        for &x in &[0.0, 0.123456, 0.5, 0.999, 1.0, 1.37, 2.0] {
            let xi = (x * ONE) as u32;
            let mut m = Mash11::new();
            let n = 1 << 20;
            let mut sum = 0i64;
            for _ in 0..n {
                let c = m.code(xi);
                assert!(c <= 4);
                sum += c as i64 - 1;
            }
            assert!((sum as f64 / n as f64 - xi as f64 / ONE).abs() < 4.0 / n as f64, "{x}");
        }
    }

    /// Second-order shaping: the error, summed twice, stays bounded (it is
    /// minus the second accumulator).
    #[test]
    fn error_is_shaped_twice() {
        let mut m = Mash11::new();
        let (mut e1, mut e2) = (0f64, 0f64);
        let mut worst = 0f64;
        for n in 0..200_000u32 {
            // A slowly moving input, like a modulated carrier.
            let x = 0.7 + 0.3 * (n as f64 * 1e-3).sin();
            let xi = (x * ONE) as u32;
            let y = m.code(xi) as f64 - 1.0;
            e1 += y - xi as f64 / ONE;
            e2 += e1;
            worst = worst.max(e2.abs());
        }
        assert!(worst <= 1.0 + 1e-6, "{worst}");
    }
}
