//! AM through the Pico: one pulse per carrier cycle, and how its edges are
//! chosen.
//!
//! A PIO state machine drives the pin from words of two counts, one word
//! per carrier cycle: low for `x + 4` system clock cycles, then high for
//! `y` ([`word`]). The amplitude of the carrier is the pulse's width and
//! its phase the pulse's position: a pulse over the carrier phases
//! `α_r … α_f` (radians) has the fundamental `z = (e^{-jα_r} - e^{-jα_f}) / 2j`
//! of the full square wave's, `sin(β)·e^{-jθ}` for a pulse of half-width
//! `β` centred at `θ`. The carrier period `P` is not a whole number of
//! clock cycles, so the edges must be rounded to clock cycles every
//! period, which moves each pulse's width and centre at random.
//!
//! The rounding is noise-shaped in the phasor domain: each pulse is chosen
//! near `v = a - (2·E₁ - E₂)`, where `a` is the wanted amplitude and `E₁`,
//! `E₂` the errors of the last two pulses' phasors (complex, computed from
//! their integer edges), so that the error of the carrier's complex
//! envelope is shaped by `(1 - z⁻¹)²`: nothing at the carrier, rising
//! towards ±fc/2. (Shaping the two edges' times instead leaves the
//! phasor's dependence on them nonlinear, and the shaped noise
//! intermodulates back next to the carrier: 25-60 dB more noise in the
//! audio band, simulated.) The pulse only needs to be chosen roughly,
//! since its error is measured afterwards and fed back: the width of
//! `|v|` from a table of the amplitude law's inverse, `w = P·asin(|v|)/π`,
//! moved by about `arg v`.
//!
//! The phase of every clock cycle is exact: an accumulator advances by
//! `K = 2³²·fc / f_clk` per cycle ([`Pulses::k`]), so the carrier is
//! exactly `f_clk·K / 2³²`, and the crystal error is corrected by changing
//! `K`. Pulse centres are due where that phase is 0. Each edge's phasor
//! comes from a sine table corrected to first order ([`sin_cos`]): the
//! table alone would add noise in the audio band and, on a steady carrier,
//! lines.
//!
//! The noise shaping needs room above the wanted amplitude: at the full
//! square wave every phase correction lowers the amplitude (by `θ²/2`),
//! and the loop would wind up. The full output is therefore
//! `1 - 10·(π/P)²` of the square wave's ([`Tables::full`]): -0.01 dB at
//! 999 kHz from 264 MHz, -0.03 dB at 1.6 MHz.
//!
//! Simulated (exact phasors of the integer edges), the error within
//! ±5 kHz of the carrier is -98 dB re the carrier on long wave (153 kHz),
//! -106 dB at 999 kHz and -111 dB at 1.6 MHz for a 1 kHz tone at 90 %
//! depth, -102 dB for 4.5 kHz at 100 %, from a 252 MHz clock; a steady
//! carrier shows no line above -100 dBc within 20 kHz.
//!
//! Everything is integer arithmetic on 32 bits that the Cortex-M0+ does
//! natively (no wide products, no division per pulse): amplitudes and
//! phasors in units of [`ONE`] (the full square wave), positions in clock
//! cycles with 8 fractional bits.

/// Peak of [`SIN`].
pub const SIN_SCALE: i32 = 32767;
/// The unit of amplitudes and phasors: the full square wave's fundamental.
pub const ONE: i32 = 2 * SIN_SCALE;
/// Entries of the sine table per carrier period (log2). With the
/// first-order correction ([`sin_cos`]) a finer one would gain nothing.
pub const SIN_BITS: u32 = 12;
pub const SIN_LEN: usize = 1 << SIN_BITS;
/// Entries of the width tables per unit amplitude (log2), plus one.
pub const WIDTH_BITS: u32 = 10;
pub const WIDTH_LEN: usize = (1 << WIDTH_BITS) + 1;
/// The state machine's low phase lasts the count plus this many cycles.
pub const MIN_LOW: i32 = 4;
/// The carrier period's range (clock cycles) the arithmetic allows: at
/// least 24 for the pulses' room, at most 4095 for the fixed point.
pub const MIN_PERIOD: u32 = 24;
pub const MAX_PERIOD: u32 = 4095;
/// Extra fractional bits of the amplitude ramp between samples.
pub const A_FRAC: u32 = 8;

/// `sin(2π i / SIN_LEN)` in units of [`SIN_SCALE`].
pub static SIN: [i16; SIN_LEN] = sin_table();

/// `asin(i / (WIDTH_LEN - 1)) · 2 / π`, in units of `1 << 16`: the width
/// of amplitude `i / 1024` as a fraction of the full square wave's.
pub static ASIN: [u32; WIDTH_LEN] = asin_table();

/// The state machine's word for a pulse rising `r` and falling `f` clock
/// cycles after the last falling edge.
#[inline(always)]
pub fn word(r: i32, f: i32) -> u32 {
    ((f - r) as u32) << 16 | (r - MIN_LOW) as u32
}

/// The pulse quantiser's state. `#[repr(C)]`: the firmware's assembly
/// kernel works on it.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Pulses {
    /// The errors of the last two pulses' phasors (in-phase, quadrature).
    pub e1r: i32,
    pub e1i: i32,
    pub e2r: i32,
    pub e2i: i32,
    /// The wanted amplitude of the last pulse and its change per pulse,
    /// with [`A_FRAC`] more fractional bits.
    pub a: i32,
    pub da: i32,
    /// Carrier phase at the last falling edge, the time origin of the next
    /// pulse.
    pub phase: u32,
    /// When the next pulse's centre is due, in clock cycles after the last
    /// falling edge (Q8).
    pub centre: i32,
    /// Carrier phase advance per clock cycle (2³² = one period).
    pub k: u32,
    /// The carrier period in clock cycles, in sixteenths.
    pub period_q4: u32,
    /// The largest centre shift (Q8): a quarter period.
    pub max_shift: i32,
    /// Bound of the errors fed back.
    pub emax: i32,
    /// The amplitude the current sample ends at (units of [`ONE`]).
    pub target: i32,
}

/// What the quantiser looks up, which depends on the carrier period.
#[repr(C)]
#[derive(Debug, Clone)]
pub struct Tables {
    /// For amplitude `i / 1024`: the pulse width (clock cycles, Q8), and
    /// the centre shift per quadrature error ([`Tables::compute`]).
    pub choice: [[i32; 2]; WIDTH_LEN],
    /// The full output (units of [`ONE`]): the amplitude of sample 65535.
    pub full: i32,
}

impl Tables {
    pub const fn new() -> Tables {
        Tables { choice: [[0; 2]; WIDTH_LEN], full: 0 }
    }

    /// Fill the tables for a carrier period of `period_q4` sixteenths of a
    /// clock cycle. (A crystal correction changes the period by too little
    /// to matter.)
    pub fn compute(&mut self, period_q4: u32) {
        let p = period_q4.max(16) as u64;
        for (i, (c, &s)) in self.choice.iter_mut().zip(ASIN.iter()).enumerate() {
            // The width in Q8: P/2 at full output.
            c[0] = ((s as u64 * p * 8 + (1 << 15)) >> 16) as i32;
            // A quadrature error q at amplitude a asks for the pulse to
            // move by about -q/a · P/2π cycles (later pulses turn the
            // phasor clockwise); the quantiser computes ((q >> 4) · shift)
            // >> 8 in Q8 cycles, so shift = -P·2^20 / (2π a) =
            // -period_q4·2^16 / (2π a), with 2π ≈ 710/113.
            let a = ((i.max(1) as u64 * ONE as u64) >> WIDTH_BITS).max(1);
            c[1] = -(((p << 16) * 113 / (710 * a)) as i32);
        }
        // 10·(π/P)² below the square wave (10·π²·256 ≈ 25266).
        self.full = ONE - (25266 * ONE as u64 / (p * p)).min(ONE as u64 / 2) as i32;
    }
}

impl Default for Tables {
    fn default() -> Self {
        Self::new()
    }
}

impl Pulses {
    /// A quantiser for a carrier of `k` (phase per clock cycle) with no
    /// output yet; the first pulse's centre is due half a period after
    /// the start.
    pub fn new(k: u32) -> Pulses {
        let mut p = Pulses {
            k: 0,
            phase: 1 << 31,
            centre: 0,
            e1r: 0,
            e1i: 0,
            e2r: 0,
            e2i: 0,
            a: 0,
            da: 0,
            period_q4: 0,
            max_shift: 0,
            emax: 0,
            target: 0,
        };
        p.retune(k);
        p.centre = p.period_q4 as i32 * 8;
        p
    }

    /// A new carrier frequency (a crystal correction), from the next pulse
    /// on.
    pub fn retune(&mut self, k: u32) {
        self.k = k;
        self.period_q4 = period_q4(k);
        self.max_shift = self.period_q4 as i32 * 4;
        // Several times what one pulse makes (an edge off by half a cycle
        // moves the phasor by π/2P).
        self.emax = ((32 * 16 * ONE as u64 / self.period_q4.max(1) as u64) as i32).clamp(ONE / 32, ONE / 4);
    }

    /// Set up the next sample's `cycles` pulses, which end at `sample`
    /// (65535: the full output, [`Tables::full`]); `inv` is
    /// [`inverse`]`(cycles)`. The amplitude moves linearly from the last
    /// sample's and lands exactly on this one's (the step's rounding moves
    /// the start by a few 2⁻²⁴ of the full output).
    #[inline(always)]
    pub fn steer(&mut self, sample: u16, cycles: u32, inv: i32, tables: &Tables) {
        self.target = ((sample as u32 * tables.full as u32) >> 16) as i32;
        // |target - a| ≤ ONE < 2^16 and inv ≤ 2^15: the product fits.
        self.da = ((self.target - (self.a >> A_FRAC)) * inv) >> (15 - A_FRAC);
        self.a = (self.target << A_FRAC) - cycles as i32 * self.da;
    }

    /// The next pulse: its state machine word ([`word`]).
    #[inline(always)]
    pub fn pulse(&mut self, tables: &Tables, sin: &[i16; SIN_LEN]) -> u32 {
        self.a += self.da;
        let vr = (self.a >> A_FRAC) - 2 * self.e1r + self.e2r;
        let vi = self.e2i - 2 * self.e1i;
        // The pulse whose phasor is nearest v: the width of |v| (max +
        // 3/8 min, within 7 %), centred about vi / |v| radians off (up to
        // 0.17 P), … or, where v points backwards (a pulse's in-phase part
        // could never be paid back there), a quarter period off, for vi
        // alone. Only there, and beyond the full output, can the errors
        // grow (wind up): bound them.
        let (i, shift, bound) = if vr > 0 {
            let y = vi.abs();
            let (hi, lo) = if vr > y { (vr, y) } else { (y, vr) };
            let i = (hi + (lo >> 2) + (lo >> 3) + 32) >> 6;
            let (i, bound) = if i < 1 << WIDTH_BITS { (i, false) } else { (1 << WIDTH_BITS, true) };
            (i as usize, ((vi >> 4) * tables.choice[i as usize][1]) >> 8, bound)
        } else {
            let i = ((vi.abs() + 32) >> 6).min(1 << WIDTH_BITS) as usize;
            (i, if vi > 0 { -self.max_shift } else { self.max_shift }, true)
        };
        let w = (tables.choice[i][0] + 128) >> 8;
        // As close as the width's parity allows.
        let r = ((self.centre + shift - (w << 7) + 128) >> 8).max(MIN_LOW);
        // The phasor of that pulse, from its edges' phases.
        let ar = self.phase.wrapping_add((r as u32).wrapping_mul(self.k));
        let af = ar.wrapping_add((w as u32).wrapping_mul(self.k));
        let (sr, cr) = sin_cos(sin, ar);
        let (sf, cf) = sin_cos(sin, af);
        let (mut er, mut ei) = (sf - sr - vr, cf - cr - vi);
        if bound {
            (er, ei) = (er.clamp(-self.emax, self.emax), ei.clamp(-self.emax, self.emax));
        }
        (self.e2r, self.e2i, self.e1r, self.e1i) = (self.e1r, self.e1i, er, ei);
        self.phase = af;
        // The next centre is a period after this one; the falling edge is
        // less than half a period past this one (in 2^-15 periods).
        let past = (self.phase as i32) >> 17;
        self.centre = ((((1 << 15) - past) as u32 * self.period_q4) >> 11) as i32;
        word(r, r + w)
    }
}

/// `(1 << 15) / cycles`, rounded, for [`Pulses::steer`].
pub fn inverse(cycles: u32) -> i32 {
    let c = cycles.max(1);
    (((1u32 << 15) + c / 2) / c) as i32
}

/// The carrier period in sixteenths of a clock cycle for phase step `k`.
pub fn period_q4(k: u32) -> u32 {
    ((1u64 << 36) / k.max(1) as u64).min(u32::MAX as u64) as u32
}

/// `(sin, cos)` of the phase `p` (2³² = 2π), in units of [`SIN_SCALE`]:
/// the table entry at or below `p`, corrected to first order for the rest
/// of the phase (the second-order error is below 1.2·10⁻⁶; the entries'
/// rounding, 1.5·10⁻⁵, remains; truncating the corrections biases both
/// edges of a pulse alike, which cancels). The entry alone would be off by
/// up to 1.5·10⁻³ (with 2^14 entries 4·10⁻⁴: noise in the audio band at
/// -88 dB on long wave, and on a steady carrier whose period is close to
/// whole clock cycles lines up to -79 dBc, the edges hitting the same
/// entries over and over).
#[inline(always)]
pub fn sin_cos(sin: &[i16; SIN_LEN], p: u32) -> (i32, i32) {
    const SHIFT: u32 = 32 - SIN_BITS;
    let i = (p >> SHIFT) as usize;
    let (s, c) = (sin[i] as i32, sin[(i + SIN_LEN / 4) & (SIN_LEN - 1)] as i32);
    // The phase past the entry (its top 16 bits) in radians, Q24: b·16·2π
    // / 2^8 ≈ b·3217 / 2^13.
    let b = ((((p >> (SHIFT - 16)) & 0xffff) * 3217) >> 13) as i32;
    let s = s + ((b * c) >> 24);
    (s, c - ((b * s) >> 24))
}

// Tables at compile time ---------------------------------------------------

const PI: f64 = core::f64::consts::PI;

/// `(sin x, cos x)` for `|x| ≤ π/4` (Taylor series to x¹⁸).
const fn sin_cos_small(x: f64) -> (f64, f64) {
    let x2 = x * x;
    let (mut s, mut c) = (x, 1.0);
    let (mut ts, mut tc) = (x, 1.0);
    let mut n = 1;
    while n < 10 {
        ts = -ts * x2 / ((2 * n) as f64 * (2 * n + 1) as f64);
        tc = -tc * x2 / ((2 * n - 1) as f64 * (2 * n) as f64);
        s += ts;
        c += tc;
        n += 1;
    }
    (s, c)
}

/// `sin(2π·n/d)` for `0 ≤ n < d`, `d` a multiple of 8.
const fn sin_frac(n: u64, d: u64) -> f64 {
    // Octant and the angle within it (|x| ≤ π/4 from the nearest multiple of π/2).
    let quarter = d / 4;
    let q = (n + quarter / 2) / quarter; // the nearest multiple of π/2
    let x = 2.0 * PI * (n as f64 - (q * quarter) as f64) / d as f64;
    let (s, c) = sin_cos_small(x);
    match q % 4 {
        0 => s,
        1 => c,
        2 => -s,
        _ => -c,
    }
}

const fn sin_table() -> [i16; SIN_LEN] {
    let mut t = [0i16; SIN_LEN];
    let mut i = 0;
    while i < SIN_LEN {
        let v = sin_frac(i as u64, SIN_LEN as u64) * SIN_SCALE as f64;
        t[i] = round(v) as i16;
        i += 1;
    }
    t
}

const fn round(v: f64) -> i64 {
    if v >= 0.0 { (v + 0.5) as i64 } else { -((-v + 0.5) as i64) }
}

/// `sin(y·π/2)` for `0 ≤ y ≤ 1`.
const fn sin_quarter(y: f64) -> f64 {
    if y <= 0.5 { sin_cos_small(y * PI / 2.0).0 } else { sin_cos_small((1.0 - y) * PI / 2.0).1 }
}

const fn asin_table() -> [u32; WIDTH_LEN] {
    let mut t = [0u32; WIDTH_LEN];
    let mut i = 0;
    while i < WIDTH_LEN {
        let a = i as f64 / (WIDTH_LEN - 1) as f64;
        // Bisection for sin(y·π/2) = a.
        let (mut lo, mut hi) = (0.0, 1.0);
        let mut n = 0;
        while n < 60 {
            let mid = (lo + hi) / 2.0;
            if sin_quarter(mid) < a {
                lo = mid;
            } else {
                hi = mid;
            }
            n += 1;
        }
        t[i] = round((lo + hi) / 2.0 * 65536.0) as u32;
        i += 1;
    }
    t
}

#[cfg(test)]
mod tests {
    extern crate std;
    use std::vec::Vec;

    use super::*;

    type C = (f64, f64);

    fn fft(x: &mut [C]) {
        let n = x.len();
        let mut j = 0;
        for i in 1..n {
            let mut bit = n >> 1;
            while j & bit != 0 {
                j ^= bit;
                bit >>= 1;
            }
            j |= bit;
            if i < j {
                x.swap(i, j);
            }
        }
        let mut len = 2;
        while len <= n {
            let ang = -2.0 * PI / len as f64;
            for start in (0..n).step_by(len) {
                for k in 0..len / 2 {
                    let (wr, wi) = ((ang * k as f64).cos(), (ang * k as f64).sin());
                    let (ar, ai) = x[start + k];
                    let (br, bi) = x[start + k + len / 2];
                    let (tr, ti) = (br * wr - bi * wi, br * wi + bi * wr);
                    x[start + k] = (ar + tr, ai + ti);
                    x[start + k + len / 2] = (ar - tr, ai - ti);
                }
            }
            len <<= 1;
        }
    }

    /// A transmission: per pulse the amplitude asked for, the phasor the
    /// pulse has (exact, from its integer edges) and its word.
    struct Run {
        wanted: Vec<f64>,
        got: Vec<C>,
        words: Vec<u32>,
        /// The carrier (Hz, at the end), and the clock cycles in all.
        fc: f64,
        ticks: u64,
        period: f64,
    }

    const TWO32: f64 = 4294967296.0;

    /// Run the quantiser at clock `clk` for carrier `fc`, `cycles` pulses
    /// per sample, over the samples `s(n)` (0-1 of the full output); from
    /// sample `retune.0` on, the carrier is `retune.1` times higher.
    fn run(clk: f64, fc: f64, cycles: u32, samples: usize, s: impl Fn(usize) -> f64, retune: (usize, f64)) -> Run {
        let mut k = (fc / clk * TWO32).round() as u32;
        let mut p = Pulses::new(k);
        let mut t = Tables::new();
        t.compute(p.period_q4);
        let inv = inverse(cycles);
        let (mut wanted, mut got, mut words) = (Vec::new(), Vec::new(), Vec::new());
        // The phase at the last falling edge, kept apart from the quantiser's.
        let mut phase = p.phase;
        let (mut ticks, mut prev) = (0u64, 0.0);
        for n in 0..samples {
            if n == retune.0 {
                k = (fc * retune.1 / clk * TWO32).round() as u32;
                p.retune(k);
            }
            let sample = (s(n).clamp(0.0, 1.0) * 65535.0).round() as u16;
            p.steer(sample, cycles, inv, &t);
            let target = p.target as f64 / ONE as f64;
            for c in 0..cycles {
                let w = p.pulse(&t, &SIN);
                words.push(w);
                let r = (w & 0xffff) + MIN_LOW as u32;
                let f = r + (w >> 16);
                let angle = |n: u32| phase.wrapping_add(n.wrapping_mul(k)) as f64 / TWO32 * 2.0 * PI;
                let (ar, af) = (angle(r), angle(f));
                // (e^{-jα_r} - e^{-jα_f}) / 2j
                got.push(((af.sin() - ar.sin()) / 2.0, (af.cos() - ar.cos()) / 2.0));
                wanted.push(prev + (target - prev) * (c + 1) as f64 / cycles as f64);
                phase = phase.wrapping_add(f.wrapping_mul(k));
                ticks += f as u64;
            }
            prev = target;
        }
        assert_eq!(phase, p.phase);
        Run { wanted, got, words, fc: clk * k as f64 / TWO32, ticks, period: TWO32 / k as f64 }
    }

    impl Run {
        /// Spectrum of the error (or of the phasors) over the last 2^17
        /// pulses (Hann window), as (Hz from the carrier, power re the
        /// carrier).
        fn spectrum(&self, error: bool) -> Vec<(f64, f64)> {
            let n = 1 << 17;
            let off = self.got.len() - n;
            let w = |i: usize| 0.5 - 0.5 * (2.0 * PI * i as f64 / n as f64).cos();
            let mut e: Vec<C> = (0..n)
                .map(|i| {
                    let (g, a) = (self.got[off + i], if error { self.wanted[off + i] } else { 0.0 });
                    ((g.0 - a) * w(i), g.1 * w(i))
                })
                .collect();
            let carrier: f64 = (0..n).map(|i| self.wanted[off + i] * w(i)).sum();
            fft(&mut e);
            let df = self.fc / n as f64;
            e.iter()
                .enumerate()
                .map(|(i, &(re, im))| {
                    let f = if i <= n / 2 { i as f64 * df } else { (i as f64 - n as f64) * df };
                    (f, (re * re + im * im) / (carrier * carrier))
                })
                .collect()
        }

        /// Error power within `lo..=hi` Hz of the carrier (both sides), re
        /// the carrier (dB).
        fn error_db(&self, lo: f64, hi: f64) -> f64 {
            let p: f64 = self.spectrum(true).iter().filter(|(f, _)| (lo..=hi).contains(&f.abs())).map(|(_, p)| p).sum();
            10.0 * p.log10()
        }
    }

    /// A tone of `hz` at `depth`, its peaks at the full output.
    fn tone(fc: f64, cycles: u32, depth: f64, hz: f64) -> impl Fn(usize) -> f64 {
        let fs = fc / cycles as f64;
        let car = 1.0 / (1.0 + depth);
        move |n| car * (1.0 + depth * (2.0 * PI * hz * n as f64 / fs).sin())
    }

    fn cycles_for(fc: f64) -> u32 {
        (fc / 100e3).round().max(1.0) as u32
    }

    #[test]
    fn tables() {
        for (i, &s) in SIN.iter().enumerate() {
            let want = (2.0 * PI * i as f64 / SIN_LEN as f64).sin() * SIN_SCALE as f64;
            assert!((s as f64 - want).abs() <= 0.5 + 1e-9, "{i}: {s} {want}");
        }
        assert_eq!((ASIN[0], ASIN[WIDTH_LEN - 1]), (0, 65536));
        for (i, &s) in ASIN.iter().enumerate() {
            let want = (i as f64 / (WIDTH_LEN - 1) as f64).asin() * 2.0 / PI * 65536.0;
            assert!((s as f64 - want).abs() <= 0.5 + 1e-6, "{i}: {s} {want}");
        }
        let mut t = Tables::new();
        t.compute(16 * 252);
        assert_eq!(t.choice[WIDTH_LEN - 1][0], 126 * 256);
        assert!((t.full as f64 / ONE as f64 - (1.0 - 10.0 * (PI / 252.0).powi(2))).abs() < 1e-4);
    }

    #[test]
    fn words_count_the_state_machine_cycles() {
        assert_eq!(word(MIN_LOW, MIN_LOW), 0);
        assert_eq!(word(100, 150), 50 << 16 | 96);
    }

    /// The modulation comes out with little noise next to the carrier,
    /// from long wave to the 75 m band (simulated with exact phasors).
    #[test]
    fn quiet_next_to_the_carrier() {
        for (clk, fc, depth, hz, limit) in [
            (252e6f64, 153e3f64, 0.9f64, 1000.0f64, -95.0f64),
            (252e6, 999e3, 0.9, 1000.0, -103.0),
            (252e6, 999e3, 1.0, 4500.0, -95.0),
            (252e6, 1.602e6, 0.9, 1000.0, -105.0),
            (252e6, 3.95e6, 0.9, 1000.0, -95.0),
            (144e6, 999e3, 0.9, 1000.0, -103.0),
            // The top of the medium wave band from the Pico's 264 MHz.
            (264e6, 1.7e6, 0.9, 1000.0, -100.0),
            (264e6, 1.7e6, 1.0, 4500.0, -88.0),
        ] {
            let cycles = cycles_for(fc);
            let r = run(clk, fc, cycles, (1 << 17) / cycles as usize + 400, tone(fc, cycles, depth, hz), (0, 1.0));
            let db = r.error_db(20.0, 5000.0);
            assert!(db < limit, "{fc} Hz from {clk} Hz, {hz} Hz at {depth}: in-band error {db:.1} dB");
        }
    }

    /// A steady carrier repeats no pattern that shows up near it.
    #[test]
    fn steady_carrier_without_lines() {
        // Among them periods of whole clock cycles (600 kHz, 1.5 MHz).
        for (clk, fc) in [(252e6f64, 999e3f64), (252e6, 1.5e6), (252e6, 600e3), (252e6, 3.9e6), (252e6, 1.008e6)] {
            let cycles = cycles_for(fc);
            let r = run(clk, fc, cycles, (1 << 17) / cycles as usize + 400, |_| 0.526, (0, 1.0));
            // Past the window's main lobe (two bins).
            let bin = r.fc / (1 << 17) as f64;
            let worst = r
                .spectrum(false)
                .iter()
                .filter(|(f, _)| (2.5 * bin..=20e3).contains(&f.abs()))
                .map(|&(_, p)| 10.0 * p.log10())
                .fold(-300.0, f64::max);
            assert!(worst < -100.0, "{fc} Hz: a line at {worst:.1} dBc within 20 kHz");
        }
    }

    /// No amplitude, no pulses; the full output; and back to nothing (the
    /// loop then pays its remaining error back with a few pulses of a
    /// cycle or two, far from the carrier).
    #[test]
    fn silence_and_full_output() {
        let (clk, fc) = (252e6, 999e3);
        let cycles = cycles_for(fc);
        let step = |n: usize| {
            if n < 2000 {
                0.0
            } else if n < 4000 {
                1.0
            } else {
                0.0
            }
        };
        let r = run(clk, fc, cycles, 6000, step, (0, 1.0));
        let c = cycles as usize;
        assert!(r.words[..2000 * c].iter().all(|w| w >> 16 == 0), "pulses without amplitude");
        let full = 1.0 - 10.0 * (PI / r.period).powi(2);
        let width = r.period * full.asin() / PI;
        // (The amplitude law is flat at the top: the widths spread.)
        for (i, &w) in r.words[2100 * c..4000 * c].iter().enumerate() {
            let w = (w >> 16) as f64;
            assert!(w >= width - 30.0 && w <= r.period / 2.0 + 1.0, "{i}: width {w} at full output ({width:.1})");
        }
        assert!(r.words[4100 * c..].iter().all(|w| w >> 16 <= 2), "pulses after the amplitude went");
        let left = r.got[4100 * c..].iter().fold((0.0, 0.0), |s, z| (s.0 + z.0, s.1 + z.1));
        let n = (r.got.len() - 4100 * c) as f64;
        let nz = r.words[4100 * c..].iter().filter(|w| **w >> 16 != 0).count();
        assert!(
            left.0.hypot(left.1) / n < 1e-4,
            "a carrier after the amplitude went: {:?}, {nz} pulses of {n}",
            (left.0 / n, left.1 / n)
        );
        // The full output's phasor.
        let got = r.got[2100 * c..4000 * c].iter().map(|z| z.0).sum::<f64>() / (1900 * c) as f64;
        assert!((got - full).abs() < 1e-4, "{got} {full}");
    }

    /// The pulses keep the period on average (no drift against the exact
    /// carrier), leave the state machine room, and follow a retune.
    #[test]
    fn timing_and_retune() {
        let (clk, fc) = (252e6, 1.234567e6);
        let cycles = cycles_for(fc);
        let samples = 20_000;
        let r = run(clk, fc, cycles, samples, tone(fc, cycles, 0.9, 3000.0), (samples / 2, 1.0 + 150e-6));
        for &w in &r.words {
            assert!((w & 0xffff) as i32 + MIN_LOW >= MIN_LOW && (w >> 16) as f64 <= r.period / 2.0 + 4.0);
        }
        // The last pulse ended within a fraction of a period of where the
        // carrier's cycles (at the retuned frequency since the middle) end.
        let first = (samples / 2 * cycles as usize) as f64 * TWO32 / (fc / clk * TWO32).round();
        let second = (r.words.len() - samples / 2 * cycles as usize) as f64 * r.period;
        let ideal = first + second;
        assert!((r.ticks as f64 - ideal).abs() < r.period / 2.0, "{} vs {ideal}", r.ticks);
        // After the retune, as quiet as before.
        assert!(r.error_db(20.0, 5000.0) < -90.0);
    }
}
