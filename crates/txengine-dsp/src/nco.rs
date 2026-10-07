//! Numerically controlled oscillators.
//!
//! All subcarriers of the FM multiplex (19 kHz pilot, 38 kHz stereo
//! subcarrier, 57 kHz RDS subcarrier, and the RDS bit clock) are derived from
//! one phase accumulator, so they are phase locked to each other by
//! construction, exactly as BS.450 and EN 50067 require.

/// Phase accumulator with 40 fractional bits (and 24 integer bits of cycles).
#[derive(Debug, Clone)]
pub struct PhaseAccumulator {
    phase: u64,
    inc: u64,
}

const FRAC_BITS: u32 = 40;
const FRAC_MASK: u64 = (1 << FRAC_BITS) - 1;

impl PhaseAccumulator {
    /// Fractional bits of the phase (cycles in 24.40 fixed point).
    pub const FRAC_BITS: u32 = FRAC_BITS;
    /// Integer bits: the cycle count wraps every 2^24 cycles.
    pub const INT_BITS: u32 = 64 - FRAC_BITS;

    /// Oscillator at `freq` Hz for sample rate `fs`.
    pub fn new(freq: f64, fs: f64) -> PhaseAccumulator {
        PhaseAccumulator { phase: 0, inc: Self::increment(freq, fs) }
    }

    fn increment(freq: f64, fs: f64) -> u64 {
        (freq / fs * (1u64 << FRAC_BITS) as f64).round() as u64
    }

    /// Change the frequency, continuing from the current phase.
    pub fn set_frequency(&mut self, freq: f64, fs: f64) {
        self.inc = Self::increment(freq, fs);
    }

    /// Actual frequency after quantisation of the increment.
    pub fn frequency(&self, fs: f64) -> f64 {
        self.inc as f64 / (1u64 << FRAC_BITS) as f64 * fs
    }

    #[inline]
    pub fn advance(&mut self) {
        self.phase = self.phase.wrapping_add(self.inc);
    }

    /// Advance by `n` samples at once.
    #[inline]
    pub fn advance_by(&mut self, n: u64) {
        self.phase = self.phase.wrapping_add(self.inc.wrapping_mul(n));
    }

    /// Raw phase (cycles in 24.40 fixed point).
    #[inline]
    pub fn raw(&self) -> u64 {
        self.phase
    }

    /// Fractional phase of the `h`-th harmonic as a 32-bit fraction of a cycle.
    #[inline]
    pub fn harmonic_frac(&self, h: u64) -> u32 {
        ((self.phase.wrapping_mul(h) & FRAC_MASK) >> (FRAC_BITS - 32)) as u32
    }

    /// Position in units of `1/div` cycles: (integer part, fraction in [0,1)).
    /// With `div` a power of two the integer part wraps consistently.
    #[inline]
    pub fn subdivided(&self, div_log2: u32) -> (u64, f64) {
        let p = self.phase >> div_log2;
        (p >> FRAC_BITS, (p & FRAC_MASK) as f64 / (1u64 << FRAC_BITS) as f64)
    }
}

const TABLE_BITS: u32 = 12;
const TABLE_LEN: usize = 1 << TABLE_BITS;

/// Sine look-up table with linear interpolation (error < 3e-7).
#[derive(Debug, Clone)]
pub struct SineTable {
    table: Vec<f32>,
}

impl Default for SineTable {
    fn default() -> Self {
        Self::new()
    }
}

impl SineTable {
    pub fn new() -> SineTable {
        // One and a quarter cycles, so cos reads the same index range.
        let table = (0..=TABLE_LEN + TABLE_LEN / 4)
            .map(|i| (2.0 * std::f64::consts::PI * i as f64 / TABLE_LEN as f64).sin() as f32)
            .collect();
        SineTable { table }
    }

    /// `sin(2 pi frac / 2^32)`.
    #[inline]
    pub fn sin(&self, frac: u32) -> f32 {
        let idx = (frac >> (32 - TABLE_BITS)) as usize;
        let t = (frac & ((1 << (32 - TABLE_BITS)) - 1)) as f32 * (1.0 / (1u32 << (32 - TABLE_BITS)) as f32);
        let a = self.table[idx];
        let b = self.table[idx + 1];
        a + (b - a) * t
    }

    /// `cos(2 pi frac / 2^32)`.
    #[inline]
    pub fn cos(&self, frac: u32) -> f32 {
        self.sin(frac.wrapping_add(1 << 30))
    }

    /// `(sin, cos)` of `2 pi frac / 2^32`, sharing the index computation.
    #[inline]
    pub fn sin_cos(&self, frac: u32) -> (f32, f32) {
        let idx = (frac >> (32 - TABLE_BITS)) as usize;
        let t = (frac & ((1 << (32 - TABLE_BITS)) - 1)) as f32 * (1.0 / (1u32 << (32 - TABLE_BITS)) as f32);
        let s = &self.table[idx..idx + TABLE_LEN / 4 + 2];
        let (s0, s1) = (s[0], s[1]);
        let (c0, c1) = (s[TABLE_LEN / 4], s[TABLE_LEN / 4 + 1]);
        (s0 + (s1 - s0) * t, c0 + (c1 - c0) * t)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn table_accuracy() {
        let t = SineTable::new();
        let mut worst: f64 = 0.0;
        for i in 0..100_000u64 {
            let frac = (i.wrapping_mul(2_654_435_761)) as u32;
            let exact = (2.0 * std::f64::consts::PI * frac as f64 / 4_294_967_296.0).sin();
            worst = worst.max((t.sin(frac) as f64 - exact).abs());
        }
        assert!(worst < 5e-7, "{worst}");
        for i in 0..100_000u64 {
            let frac = (i.wrapping_mul(2_654_435_761)) as u32;
            let (sn, cs) = t.sin_cos(frac);
            assert_eq!(sn, t.sin(frac));
            assert!((cs - t.cos(frac)).abs() < 1e-6);
        }
    }

    #[test]
    fn harmonics_are_locked() {
        let mut p = PhaseAccumulator::new(19_000.0, 250_000.0);
        let t = SineTable::new();
        for _ in 0..12_345 {
            p.advance();
        }
        // sin(3x) = 3 sin x - 4 sin^3 x
        let s = t.sin(p.harmonic_frac(1)) as f64;
        let s3 = t.sin(p.harmonic_frac(3)) as f64;
        assert!((s3 - (3.0 * s - 4.0 * s.powi(3))).abs() < 1e-5);
        assert!((p.frequency(250_000.0) - 19_000.0).abs() < 1e-6);
    }

    #[test]
    fn frequency_change_keeps_the_phase() {
        let mut p = PhaseAccumulator::new(19_000.0, 250_000.0);
        p.advance_by(1000);
        let before = p.raw();
        // The sample clock runs 20 ppm fast: a lower increment per sample.
        p.set_frequency(19_000.0, 250_000.0 * (1.0 + 20e-6));
        assert_eq!(p.raw(), before);
        assert!((p.frequency(250_000.0) / 19_000.0 - 1.0 + 20e-6).abs() < 1e-9);
    }

    #[test]
    fn subdivision_counts_bits() {
        // 16 pilot cycles per RDS bit.
        let mut p = PhaseAccumulator::new(19_000.0, 228_000.0);
        let mut last = 0;
        let mut transitions = 0;
        for _ in 0..228_000 {
            p.advance();
            let (bit, _) = p.subdivided(4);
            if bit != last {
                transitions += 1;
                last = bit;
            }
        }
        assert_eq!(transitions, 1187); // 1187.5 bit/s
    }
}
