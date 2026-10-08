//! RDS physical layer (EN 50067 clause 1): differential coding, biphase
//! symbols and data-channel spectrum shaping.
//!
//! Each differentially encoded bit produces an odd impulse pair
//! `±(δ(t) − δ(t − td/2))` filtered by
//! `H_T(f) = cos(π f td / 4)` for `f ≤ 2/td` (zero above), `td = 1/1187.5 s`.
//! The impulse response of `H_T` has the closed form
//!
//! ```text
//!          td      cos(4π t / td)
//! h(t) = ---- · ------------------
//!         2π     td²/16 − 4 t²
//! ```
//!
//! which is evaluated directly, so the shaping is exact at any sample rate
//! (no resampling of a waveform designed for 228 kHz).
//!
//! The shaped symbol is truncated to ±[`SPAN`] bits, so the waveform during
//! one bit depends on `2·SPAN` symbols. Their contributions are tabulated
//! in groups of four (16 sign patterns per group); once per bit the
//! waveform of the bit is assembled from one table row per group, and each
//! output sample is a single interpolated look-up in it.
//!
//! Timing is driven by the caller's pilot phase accumulator: one bit lasts
//! exactly 16 pilot cycles (48 cycles of the 57 kHz subcarrier, EN 50067 1.5).

use txengine_dsp::PhaseAccumulator;

use crate::encoder::RdsEncoder;

/// Bits on each side of the current one that contribute to the waveform.
pub const SPAN: usize = 6;
/// Symbols per pattern table.
const GROUP: usize = 4;
const GROUPS: usize = 2 * SPAN / GROUP;
/// Table resolution: points per bit (power of two).
const RES_BITS: u32 = 8;
const RES: usize = 1 << RES_BITS;
/// Stride of one table row (`RES + 1` points, padded).
const ROW: usize = RES + 4;
/// Pilot cycles per bit, log2.
const CYCLES_PER_BIT_LOG2: u32 = 4;
/// The bit counter derived from the accumulator wraps at this modulus.
const BIT_MASK: u32 = (1 << (PhaseAccumulator::INT_BITS - CYCLES_PER_BIT_LOG2)) - 1;

/// Unnormalised impulse response of H_T, with t in bit periods.
fn h(x: f64) -> f64 {
    let den = 1.0 / 16.0 - 4.0 * x * x;
    if den.abs() < 1e-9 {
        // Limit at x = ±1/8: 4π (L'Hôpital).
        return 4.0 * std::f64::consts::PI;
    }
    (4.0 * std::f64::consts::PI * x).cos() / den
}

/// Shaped biphase symbol (impulse at 0, inverted impulse half a bit
/// later), truncated to ±SPAN bits with a gentle taper over the outermost
/// bit (avoids a truncation step).
fn symbol(x: f64) -> f64 {
    let edge = SPAN as f64 - x.abs();
    if edge <= 0.0 {
        return 0.0;
    }
    let taper = if edge < 1.0 { 0.5 - 0.5 * (std::f64::consts::PI * edge).cos() } else { 1.0 };
    (h(x) - h(x - 0.5)) * taper
}

/// Pre-computed shaped symbol contributions.
#[derive(Debug, Clone)]
pub struct BiphaseShaper {
    /// `tables[(g * 16 + pattern) * ROW + i]`: waveform at `i / RES` bits
    /// into the current bit, from the four symbols of group `g` with signs
    /// given by `pattern` (bit set = positive), normalised.
    tables: Vec<f32>,
    /// Peak of the waveform for the worst-case data pattern (normalised).
    peak: f32,
    /// Amplitude of the all-zero reference before normalisation.
    #[cfg_attr(not(test), allow(dead_code))]
    a0: f64,
}

impl Default for BiphaseShaper {
    fn default() -> Self {
        Self::new()
    }
}

impl BiphaseShaper {
    pub fn new() -> BiphaseShaper {
        // Symbol q (0 .. 2·SPAN) of the window starts SPAN - 1 - q bits
        // before the current bit.
        let offset = |q: usize| SPAN as f64 - 1.0 - q as f64;
        // Normalise to the amplitude of the 1187.5 Hz sine produced by an
        // all-zero message (the reference for the injection level, EN 50067
        // 1.3 and its footnote); find the worst-case peak on the way.
        let mut a0: f64 = 0.0;
        let mut worst: f64 = 0.0;
        for i in 0..4 * RES {
            let x = i as f64 / (4 * RES) as f64;
            let (mut sum, mut abs) = (0.0, 0.0);
            for q in 0..2 * SPAN {
                let v = symbol(x + offset(q));
                sum += v;
                abs += v.abs();
            }
            a0 = a0.max(sum.abs());
            worst = worst.max(abs);
        }
        let mut tables = vec![0f32; GROUPS * 16 * ROW];
        for g in 0..GROUPS {
            for pattern in 0..16 {
                let row = &mut tables[(g * 16 + pattern) * ROW..][..ROW];
                for (i, v) in row.iter_mut().enumerate().take(RES + 1) {
                    let x = i as f64 / RES as f64;
                    let mut sum = 0.0;
                    for b in 0..GROUP {
                        let sign = if pattern & (1 << b) != 0 { 1.0 } else { -1.0 };
                        sum += sign * symbol(x + offset(g * GROUP + b));
                    }
                    *v = (sum / a0) as f32;
                }
            }
        }
        BiphaseShaper { tables, peak: (worst / a0) as f32, a0 }
    }

    /// Worst-case peak of the shaped data signal relative to the all-zero
    /// reference amplitude.
    pub fn peak_factor(&self) -> f32 {
        self.peak
    }

    /// Assemble the waveform of one bit from the symbol window `window`
    /// (bit q set: symbol q positive).
    fn assemble(&self, window: u32, wave: &mut [f32]) {
        let row = |g: usize| {
            let pattern = (window >> (g * GROUP)) as usize & 15;
            &self.tables[(g * 16 + pattern) * ROW..][..RES + 1]
        };
        let (r0, r1, r2) = (row(0), row(1), row(2));
        for (((w, &a), &b), &c) in wave[..RES + 1].iter_mut().zip(r0).zip(r1).zip(r2) {
            *w = a + b + c;
        }
    }
}

const _: () = assert!(GROUPS == 3, "assemble() sums three groups");

/// Bit stream and waveform generator.
#[derive(Debug, Clone)]
pub struct RdsModulator {
    shaper: BiphaseShaper,
    /// Waveform of the current bit (`RES + 1` points).
    wave: Vec<f32>,
    /// Differential symbols of the window around the current bit: bit q is
    /// symbol `bit - SPAN + 1 + q`.
    window: u32,
    /// Bit counter value of the current bit as derived from the accumulator.
    raw_bit: u32,
    /// Absolute index of the current bit.
    bit: u64,
    /// Group being sent and the index of its next bit to enter the window.
    group_bits: [bool; 104],
    group_pos: usize,
    diff: bool,
    started: bool,
}

impl Default for RdsModulator {
    fn default() -> Self {
        Self::new()
    }
}

impl RdsModulator {
    pub fn new() -> RdsModulator {
        RdsModulator {
            shaper: BiphaseShaper::new(),
            wave: vec![0.0; ROW],
            window: 0,
            raw_bit: 0,
            bit: 0,
            group_bits: [false; 104],
            group_pos: 104,
            diff: false,
            started: false,
        }
    }

    pub fn peak_factor(&self) -> f32 {
        self.shaper.peak_factor()
    }

    /// Bits sent since the start.
    pub fn bits(&self) -> u64 {
        self.bit
    }

    /// Differentially encode the data bit sent as bit `index`; a new group
    /// is fetched when needed. `now` is the emission time of bit position
    /// `pos` (in bits), from which the end time of a new group is derived.
    fn next_symbol(&mut self, index: u64, encoder: &mut RdsEncoder, now: f64, pos: f64) -> bool {
        if self.group_pos == 104 {
            let end = now + ((index + 104) as f64 - pos) / 1187.5;
            self.group_bits = encoder.next_group(end).bits();
            self.group_pos = 0;
        }
        self.diff ^= self.group_bits[self.group_pos];
        self.group_pos += 1;
        self.diff
    }

    /// Render the baseband (before the 57 kHz carrier) for `out.len()`
    /// samples, normalised so an all-zero message is a unit sine.
    ///
    /// `pilot` is the 19 kHz phase accumulator at the first sample (a copy
    /// is advanced here); `t0` is the emission time of the first sample
    /// (Unix seconds) and `dt` the sample period, used to time the groups
    /// (clock-time group at the minute edge).
    pub fn render(&mut self, pilot: &PhaseAccumulator, out: &mut [f32], encoder: &mut RdsEncoder, t0: f64, dt: f64) {
        let mut acc = pilot.clone();
        const FRAC_SHIFT: u32 = PhaseAccumulator::FRAC_BITS + CYCLES_PER_BIT_LOG2 - 32;
        for (i, o) in out.iter_mut().enumerate() {
            // Position in bits: integer part (wrapping) and 32-bit fraction.
            let p = acc.raw();
            let raw_bit = (p >> (PhaseAccumulator::FRAC_BITS + CYCLES_PER_BIT_LOG2)) as u32 & BIT_MASK;
            let frac = (p >> FRAC_SHIFT) as u32;
            if raw_bit != self.raw_bit || !self.started {
                self.advance(raw_bit, frac, encoder, t0 + i as f64 * dt);
            }
            let idx = (frac >> (32 - RES_BITS)) as usize;
            let t = (frac & ((1 << (32 - RES_BITS)) - 1)) as f32 * (1.0 / (1u64 << (32 - RES_BITS)) as f32);
            let a = self.wave[idx];
            let b = self.wave[idx + 1];
            *o = a + (b - a) * t;
            acc.advance();
        }
    }

    #[cold]
    fn advance(&mut self, raw_bit: u32, frac: u32, encoder: &mut RdsEncoder, now: f64) {
        let frac = frac as f64 / 4_294_967_296.0;
        if !self.started {
            self.started = true;
            // Symbols before the start: all-zero data. The first group starts
            // with the current bit.
            self.window = 0;
            let pos = self.bit as f64 + frac;
            for q in SPAN - 1..2 * SPAN {
                let index = self.bit + (q + 1 - SPAN) as u64;
                let d = self.next_symbol(index, encoder, now, pos);
                self.window |= (d as u32) << q;
            }
        } else {
            let steps = raw_bit.wrapping_sub(self.raw_bit) & BIT_MASK;
            for _ in 0..steps {
                self.bit += 1;
                // The symbol entering the window is that of bit `bit + SPAN`.
                let pos = self.bit as f64 + frac;
                let d = self.next_symbol(self.bit + SPAN as u64, encoder, now, pos);
                self.window = (self.window >> 1) | ((d as u32) << (2 * SPAN - 1));
            }
        }
        self.raw_bit = raw_bit;
        self.shaper.assemble(self.window, &mut self.wave);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn closed_form_matches_numeric_integral() {
        // h(t) = ∫_{-2/td}^{2/td} cos(π f td / 4) e^{j2πft} df with td = 1.
        for &x in &[0.0, 0.05, 0.125, 0.3, 0.77, 1.4] {
            let n = 20_000;
            let mut s = 0.0;
            for i in 0..n {
                let f = -2.0 + 4.0 * (i as f64 + 0.5) / n as f64;
                s += (std::f64::consts::PI * f / 4.0).cos() * (2.0 * std::f64::consts::PI * f * x).cos() * 4.0
                    / n as f64;
            }
            let closed = h(x) / (2.0 * std::f64::consts::PI);
            assert!((s - closed).abs() < 1e-3, "x={x}: {s} vs {closed}");
        }
    }

    #[test]
    fn peak_factor_is_sane() {
        let p = BiphaseShaper::new().peak_factor();
        // Verified independently: 1.020 for the EN 50067 shaping.
        assert!((1.0..1.05).contains(&p), "{p}");
    }

    #[test]
    fn tables_match_direct_sum() {
        let s = BiphaseShaper::new();
        let mut wave = vec![0f32; ROW];
        let window = 0b1011_0010_1101u32;
        s.assemble(window, &mut wave);
        for (i, &w) in wave.iter().enumerate().take(RES + 1) {
            let x = i as f64 / RES as f64;
            let mut direct = 0.0;
            for q in 0..2 * SPAN {
                let v = symbol(x + SPAN as f64 - 1.0 - q as f64);
                direct += if window & (1 << q) != 0 { v } else { -v };
            }
            let expect = (direct / s.a0) as f32;
            assert!((w - expect).abs() < 1e-6, "i={i}: {w} vs {expect}");
        }
        // All-zero data: a unit sine.
        s.assemble(0, &mut wave);
        let peak = wave[..=RES].iter().fold(0f32, |m, &v| m.max(v.abs()));
        assert!((peak - 1.0).abs() < 1e-3, "{peak}");
    }

    #[test]
    fn survives_accumulator_wrap() {
        // The bit counter derived from the accumulator wraps after 2^20 bits
        // (~14.7 min); start just before the wrap and check the bit count.
        let fs = 228_000.0;
        let mut acc = PhaseAccumulator::new(19_000.0, fs);
        let cycles_to_wrap = (1u64 << PhaseAccumulator::INT_BITS) - 40;
        let samples = (cycles_to_wrap as f64 * fs / 19_000.0) as u64;
        acc.advance_by(samples);
        let mut m = RdsModulator::new();
        let mut enc = RdsEncoder::new(Default::default()).unwrap();
        let mut out = vec![0f32; 228_000];
        let mut rms = 0.0;
        for _ in 0..2 {
            m.render(&acc, &mut out, &mut enc, f64::NAN, 1.0 / fs);
            acc.advance_by(out.len() as u64);
            rms = (out.iter().map(|&v| (v * v) as f64).sum::<f64>() / out.len() as f64).sqrt();
        }
        assert!((m.bits() as f64 - 2375.0).abs() <= 1.0, "bits {}", m.bits());
        assert!(rms > 0.3, "signal after wrap: rms {rms}");
    }
}
