//! Sub-audio signalling for tone and code squelch: a CTCSS tone or a DCS
//! code, sent below the voice band (under 300 Hz) all the time, so that a
//! receiver set to it opens its squelch for this transmitter only.
//!
//! * **CTCSS**: one of the tones of EIA/TIA-603, 67.0–254.1 Hz.
//! * **DCS**: a 23-bit word repeated at 134.4 bit/s. Its 12 data bits are
//!   the code's three octal digits (9 bits, the last digit in the lowest
//!   bits) and the fixed bits `100`, protected by a Golay (23, 12) code with
//!   the generator polynomial `0xC75`; the word is sent from its lowest bit
//!   up. These words reproduce the published aliases, e.g. 023 is received
//!   as 340 or 766 when a receiver locks onto another bit of it, and
//!   inverted as 047, 375 or 707 (the tests check them). A 1 bit deviates the
//!   carrier upwards; an inverted code (`023I`) sends the complement, for a
//!   receiver that sees the polarity the other way. The bits change with a
//!   raised-cosine transition over one bit period, which leaves 52 dB less
//!   power above 300 Hz than below.

use std::fmt;

use txengine_dsp::{PhaseAccumulator, SineTable};

/// The CTCSS tones of EIA/TIA-603 (Hz).
pub const CTCSS_TONES: [f64; 50] = [
    67.0, 69.3, 71.9, 74.4, 77.0, 79.7, 82.5, 85.4, 88.5, 91.5, 94.8, 97.4, 100.0, 103.5, 107.2, 110.9, 114.8, 118.8,
    123.0, 127.3, 131.8, 136.5, 141.3, 146.2, 151.4, 156.7, 159.8, 162.2, 165.5, 167.9, 171.3, 173.8, 177.3, 179.9,
    183.5, 186.2, 189.9, 192.8, 196.6, 199.5, 203.5, 206.5, 210.7, 218.1, 225.7, 229.1, 233.6, 241.8, 250.3, 254.1,
];

/// Lowest and highest CTCSS tone accepted (Hz): the voice high-pass's stop
/// band reaches up to the highest standard tone.
pub const CTCSS_RANGE: (f64, f64) = (60.0, 254.1);

/// Whether `f` is a standard CTCSS tone.
pub fn is_standard_tone(f: f64) -> bool {
    CTCSS_TONES.iter().any(|&t| (t - f).abs() < 0.05)
}

/// DCS bit rate (bit/s).
pub const DCS_BIT_RATE: f64 = 134.4;
/// Bits in a DCS word.
pub const DCS_WORD_BITS: usize = 23;

/// Generator polynomial of the DCS Golay (23, 12) code.
const GOLAY_POLY: u32 = 0xc75;

/// The DCS word of 12 data bits: the 11 check bits above the data, so
/// that the whole word, sent from its lowest bit, is data then check bits.
fn golay(data: u32) -> u32 {
    let data = data & 0xfff;
    let mut r = data;
    for _ in 0..12 {
        if r & 1 != 0 {
            r ^= GOLAY_POLY;
        }
        r >>= 1;
    }
    (r << 12) | data
}

/// A DCS code: three octal digits, normal or inverted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DcsCode {
    /// 0o000–0o777.
    pub code: u16,
    pub inverted: bool,
}

impl DcsCode {
    /// `023`, `023N`, `023I` or the same with a leading `D` (`D023N`).
    pub fn parse(s: &str) -> Result<DcsCode, String> {
        let t = s.trim().to_ascii_uppercase();
        let t = t.strip_prefix('D').unwrap_or(&t);
        let (digits, inverted) = match t.as_bytes().last() {
            Some(b'N') => (&t[..t.len() - 1], false),
            Some(b'I') => (&t[..t.len() - 1], true),
            _ => (t, false),
        };
        if digits.len() != 3 || !digits.bytes().all(|b| (b'0'..=b'7').contains(&b)) {
            return Err(format!(
                "a DCS code is three octal digits, optionally with N or I (e.g. 023, 754I), got '{s}'"
            ));
        }
        let code = u16::from_str_radix(digits, 8).expect("octal digits");
        Ok(DcsCode { code, inverted })
    }

    /// The 23-bit word as sent: the first bit in bit 0, a 1 bit deviating
    /// upwards.
    pub fn word(&self) -> u32 {
        let w = golay(0x800 | self.code as u32);
        if self.inverted { !w & ((1 << DCS_WORD_BITS) - 1) } else { w }
    }
}

impl fmt::Display for DcsCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "D{:03o}{}", self.code, if self.inverted { 'I' } else { 'N' })
    }
}

/// What is sent below the voice band.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Subaudio {
    Off,
    /// A tone (Hz).
    Ctcss(f64),
    Dcs(DcsCode),
}

impl Subaudio {
    /// Check a tone's range (the code needs no check).
    pub fn check(&self) -> Result<(), String> {
        match *self {
            Subaudio::Ctcss(f) if !(CTCSS_RANGE.0..=CTCSS_RANGE.1 + 0.01).contains(&f) => {
                Err(format!("a CTCSS tone must be {:.0}-{:.1} Hz, got {f} Hz", CTCSS_RANGE.0, CTCSS_RANGE.1))
            }
            _ => Ok(()),
        }
    }
}

impl fmt::Display for Subaudio {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Subaudio::Off => f.write_str("off"),
            Subaudio::Ctcss(t) => write!(f, "CTCSS {t:.1} Hz"),
            Subaudio::Dcs(c) => write!(f, "DCS {c}"),
        }
    }
}

/// Renders the sub-audio signal, as a fraction (±1) of its deviation.
#[derive(Debug, Clone)]
pub struct SubaudioGenerator {
    what: Subaudio,
    table: SineTable,
    /// The CTCSS tone.
    tone: PhaseAccumulator,
    /// DCS: the word, the bit being sent and how far into it (0-1), and
    /// how far each sample advances.
    word: u32,
    bit: usize,
    frac: f64,
    bit_step: f64,
}

impl SubaudioGenerator {
    /// For output samples at `fs` Hz.
    pub fn new(what: Subaudio, fs: f64) -> SubaudioGenerator {
        let tone = match what {
            Subaudio::Ctcss(f) => f,
            _ => 0.0,
        };
        let word = match what {
            Subaudio::Dcs(c) => c.word(),
            _ => 0,
        };
        SubaudioGenerator {
            what,
            table: SineTable::new(),
            tone: PhaseAccumulator::new(tone, fs),
            word,
            bit: 0,
            frac: 0.0,
            bit_step: DCS_BIT_RATE / fs,
        }
    }

    pub fn what(&self) -> Subaudio {
        self.what
    }

    /// The samples now leave at `fs` Hz (a crystal error): keep the tone
    /// and the bit rate true, continuing from where they are.
    pub fn set_rate(&mut self, fs: f64) {
        if let Subaudio::Ctcss(f) = self.what {
            self.tone.set_frequency(f, fs);
        }
        self.bit_step = DCS_BIT_RATE / fs;
    }

    /// Bit `k` of the word (wrapping) as ±1.
    #[inline]
    fn level(&self, k: usize) -> f32 {
        if self.word >> (k % DCS_WORD_BITS) & 1 != 0 { 1.0 } else { -1.0 }
    }

    /// `sin(π x)` for `x` in −1..1.
    #[inline]
    fn sin_pi(&self, x: f64) -> f32 {
        self.table.sin((x * 2f64.powi(31)) as i64 as u32)
    }

    /// Add `scale` times the signal to `out`.
    pub fn add(&mut self, out: &mut [f32], scale: f32) {
        match self.what {
            Subaudio::Off => {}
            Subaudio::Ctcss(_) => {
                for o in out.iter_mut() {
                    *o += scale * self.table.sin(self.tone.harmonic_frac(1));
                    self.tone.advance();
                }
            }
            Subaudio::Dcs(_) => {
                for o in out.iter_mut() {
                    // Each change of bit is a raised-cosine step over one
                    // bit period, centred on the bit boundary.
                    let b = self.level(self.bit);
                    let v = if self.frac < 0.5 {
                        let a = self.level(self.bit + DCS_WORD_BITS - 1);
                        a + (b - a) * 0.5 * (1.0 + self.sin_pi(self.frac))
                    } else {
                        let c = self.level(self.bit + 1);
                        b + (c - b) * 0.5 * (1.0 + self.sin_pi(self.frac - 1.0))
                    };
                    *o += scale * v;
                    self.frac += self.bit_step;
                    if self.frac >= 1.0 {
                        self.frac -= 1.0;
                        self.bit = (self.bit + 1) % DCS_WORD_BITS;
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rotations(w: u32) -> impl Iterator<Item = u32> {
        let m = (1u32 << DCS_WORD_BITS) - 1;
        (0..DCS_WORD_BITS as u32).map(move |k| ((w >> k) | (w << (DCS_WORD_BITS as u32 - k))) & m)
    }

    /// The codes whose words (normal polarity) are rotations of `w`.
    fn aliases(w: u32) -> Vec<u16> {
        let mut v: Vec<u16> =
            (0..0o1000).filter(|&c| rotations(w).any(|r| r == DcsCode { code: c, inverted: false }.word())).collect();
        v.sort();
        v
    }

    #[test]
    fn dcs_words_match_the_published_aliases() {
        fn d(s: &str) -> u32 {
            DcsCode::parse(s).unwrap().word()
        }
        // 023 received from another starting bit, and inverted.
        assert_eq!(aliases(d("023")), vec![0o023, 0o340, 0o766]);
        assert_eq!(aliases(d("023I")), vec![0o047, 0o375, 0o707]);
        // Pairs of codes that are each other's inversion.
        for (a, b) in [("025", 0o244), ("026", 0o464), ("031", 0o627), ("032", 0o051), ("054", 0o413)] {
            assert!(aliases(d(&format!("{a}I"))).contains(&b), "{a}");
        }
    }

    #[test]
    fn dcs_word_layout() {
        let w = DcsCode::parse("D023N").unwrap().word();
        // The code's bits first (lowest digit lowest), then 0, 0, 1.
        assert_eq!(w & 0x1ff, 0o023);
        assert_eq!((w >> 9) & 7, 0b100);
        assert_eq!(w, 0x763813);
    }

    #[test]
    fn dcs_parsing() {
        assert_eq!(DcsCode::parse("023").unwrap(), DcsCode { code: 0o23, inverted: false });
        assert_eq!(DcsCode::parse("d754i").unwrap(), DcsCode { code: 0o754, inverted: true });
        assert_eq!(DcsCode::parse("D411N").unwrap().to_string(), "D411N");
        for bad in ["23", "0238", "089", "D02", "023X", ""] {
            assert!(DcsCode::parse(bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn ctcss_tones() {
        assert!(is_standard_tone(88.5) && is_standard_tone(254.1) && !is_standard_tone(88.0));
        assert!(Subaudio::Ctcss(254.1).check().is_ok());
        assert!(Subaudio::Ctcss(300.0).check().is_err());
    }

    /// The bits come out at the bit rate, at their centres at full level.
    #[test]
    fn dcs_waveform() {
        let fs = 19_000.0;
        let code = DcsCode::parse("165").unwrap();
        let mut g = SubaudioGenerator::new(Subaudio::Dcs(code), fs);
        let n = (fs * 2.0 * DCS_WORD_BITS as f64 / DCS_BIT_RATE) as usize;
        let mut x = vec![0f32; n];
        g.add(&mut x, 1.0);
        for k in 0..2 * DCS_WORD_BITS {
            let centre = ((k as f64 + 0.5) / DCS_BIT_RATE * fs).round() as usize;
            let want = if code.word() >> (k % DCS_WORD_BITS) & 1 != 0 { 1.0 } else { -1.0 };
            assert!((x[centre] - want).abs() < 1e-3, "bit {k}: {}", x[centre]);
        }
        assert!(x.iter().all(|v| v.abs() <= 1.0 + 1e-6));
    }
}
