//! Independent RDS demodulator and decoder for verification.
//!
//! Takes an FM multiplex signal (as produced by txengine, or demodulated
//! from an over-the-air capture) and recovers RDS groups and station data.
//! The implementation deliberately shares nothing with the transmit path
//! except the block code tables, so it can catch encoder mistakes.

use crate::charset::decode_char;
use crate::group::{Group, GroupType, Offset, syndrome};

/// Result of decoding a signal.
#[derive(Debug, Clone, Default)]
pub struct Decoded {
    pub groups: Vec<Group>,
    /// Blocks received with a correct checkword / total blocks after sync.
    pub blocks_ok: usize,
    pub blocks_total: usize,
    pub station: Station,
    /// Estimated RDS injection (peak deviation of the all-zero reference
    /// sine) in the units of the input signal.
    pub level: f64,
    /// Measured 57 kHz carrier frequency offset (Hz).
    pub carrier_offset: f64,
    /// Bit positions of erroneous blocks.
    pub errors_at: Vec<usize>,
    /// For each entry of `groups`: time (seconds from the start of the
    /// signal) at which its last bit ended.
    pub group_end_times: Vec<f64>,
}

/// Station data accumulated from decoded groups.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Station {
    pub pi: Option<u16>,
    pub pty: Option<u8>,
    pub tp: Option<bool>,
    pub ta: Option<bool>,
    pub ms: Option<bool>,
    pub di: [Option<bool>; 4],
    pub ps: [Option<char>; 8],
    pub rt: Vec<Option<char>>,
    pub rt_ab: Option<bool>,
    pub ptyn: [Option<char>; 8],
    pub af: Vec<u8>,
    pub ecc: Option<u8>,
    pub lic: Option<u16>,
    /// (MJD, hour, minute, local offset in half hours).
    pub ct: Vec<(u32, u8, u8, i8)>,
    pub group_counts: std::collections::BTreeMap<String, usize>,
}

impl Station {
    pub fn ps_string(&self) -> String {
        self.ps.iter().map(|c| c.unwrap_or('_')).collect()
    }

    /// RadioText up to the carriage return.
    pub fn rt_string(&self) -> String {
        let mut s = String::new();
        for c in &self.rt {
            match c {
                Some('\r') => break,
                Some(c) => s.push(*c),
                None => s.push('_'),
            }
        }
        s.trim_end().to_string()
    }

    pub fn ptyn_string(&self) -> String {
        self.ptyn.iter().map(|c| c.unwrap_or('_')).collect()
    }

    fn apply(&mut self, g: &Group) {
        let b = g.blocks;
        let gt = g.group_type();
        *self.group_counts.entry(gt.to_string()).or_default() += 1;
        self.pi = Some(b[0]);
        self.tp = Some(b[1] & (1 << 10) != 0);
        self.pty = Some(((b[1] >> 5) & 0x1f) as u8);
        match (gt.code, gt.b) {
            (0, _) => {
                let seg = (b[1] & 3) as usize;
                self.ta = Some(b[1] & 0x10 != 0);
                self.ms = Some(b[1] & 0x8 != 0);
                self.di[3 - seg] = Some(b[1] & 0x4 != 0);
                self.ps[2 * seg] = Some(decode_char((b[3] >> 8) as u8));
                self.ps[2 * seg + 1] = Some(decode_char(b[3] as u8));
                if !gt.b {
                    for c in [(b[2] >> 8) as u8, b[2] as u8] {
                        if (1..=204).contains(&c) && !self.af.contains(&c) {
                            self.af.push(c);
                        }
                    }
                }
            }
            (2, false) => {
                let ab = b[1] & 0x10 != 0;
                if self.rt_ab != Some(ab) {
                    self.rt.clear();
                    self.rt_ab = Some(ab);
                }
                let seg = (b[1] & 0xf) as usize;
                if self.rt.len() < 64 {
                    self.rt.resize(64, None);
                }
                for (i, c) in [(b[2] >> 8), b[2], (b[3] >> 8), b[3]].iter().enumerate() {
                    self.rt[4 * seg + i] = Some(decode_char(*c as u8));
                }
            }
            (4, false) => {
                let mjd = ((b[1] as u32 & 3) << 15) | (b[2] as u32 >> 1);
                let hour = (((b[2] & 1) << 4) | (b[3] >> 12)) as u8;
                let min = ((b[3] >> 6) & 0x3f) as u8;
                let off = (b[3] & 0x1f) as i8 * if b[3] & 0x20 != 0 { -1 } else { 1 };
                self.ct.push((mjd, hour, min, off));
            }
            (10, false) => {
                let seg = (b[1] & 1) as usize;
                for (i, c) in [(b[2] >> 8), b[2], (b[3] >> 8), b[3]].iter().enumerate() {
                    self.ptyn[4 * seg + i] = Some(decode_char(*c as u8));
                }
            }
            (1, false) => match (b[2] >> 12) & 7 {
                0 => self.ecc = Some(b[2] as u8),
                3 => self.lic = Some(b[2] & 0xfff),
                _ => {}
            },
            _ => {}
        }
    }
}

/// Block synchroniser over a stream of data bits.
///
/// Acquisition requires two blocks with valid syndromes whose distance and
/// block sequence agree (as RDS receivers do), so a chance syndrome match in
/// noise or before the signal starts does not produce a false lock.
#[derive(Debug, Default)]
pub struct BlockSync {
    reg: u32,
    nbits: usize,
    synced: bool,
    expect: usize,
    bits_since: usize,
    blocks: [u16; 4],
    good: [bool; 4],
    /// Recent syndrome matches while searching: (bit position, block index).
    candidates: Vec<(usize, usize)>,
    pub out: Vec<Group>,
    pub blocks_ok: usize,
    pub blocks_total: usize,
    /// Bit positions of blocks received with errors (diagnostics).
    pub errors_at: Vec<usize>,
    /// Index of the last bit of every complete group in `out`.
    pub group_end_bits: Vec<usize>,
    bad_run: usize,
}

fn block_index(syn: u16) -> Option<(usize, bool)> {
    match syn {
        x if x == Offset::A.word() => Some((0, false)),
        x if x == Offset::B.word() => Some((1, false)),
        x if x == Offset::C.word() => Some((2, false)),
        x if x == Offset::Cp.word() => Some((2, true)),
        x if x == Offset::D.word() => Some((3, false)),
        _ => None,
    }
}

impl BlockSync {
    pub fn push(&mut self, bit: bool) {
        self.reg = ((self.reg << 1) | bit as u32) & 0x3ff_ffff;
        self.nbits += 1;
        if self.nbits < 26 {
            return;
        }
        if !self.synced {
            let Some((idx, _)) = block_index(syndrome(self.reg)) else { return };
            let pos = self.nbits;
            let confirmed = self.candidates.iter().any(|&(p, i)| {
                let d = pos - p;
                d.is_multiple_of(26) && (1..=6).contains(&(d / 26)) && (i + d / 26) % 4 == idx
            });
            self.candidates.retain(|&(p, _)| pos - p <= 6 * 26);
            self.candidates.push((pos, idx));
            if !confirmed {
                return;
            }
            // Locked on this block; start a fresh group at it.
            self.synced = true;
            self.candidates.clear();
            self.bad_run = 0;
            self.good = [false; 4];
            self.expect = idx;
            self.bits_since = 0;
            self.accept(true);
            return;
        }
        self.bits_since += 1;
        if self.bits_since < 26 {
            return;
        }
        self.bits_since = 0;
        let ok = block_index(syndrome(self.reg)).is_some_and(|(idx, _)| idx == self.expect);
        self.accept(ok);
    }

    fn accept(&mut self, ok: bool) {
        self.blocks_total += 1;
        if ok {
            self.blocks_ok += 1;
            self.bad_run = 0;
        } else {
            self.errors_at.push(self.nbits);
            self.bad_run += 1;
            if self.bad_run > 10 {
                self.synced = false;
                self.bad_run = 0;
                return;
            }
        }
        self.blocks[self.expect] = (self.reg >> 10) as u16;
        self.good[self.expect] = ok;
        if self.expect == 3 {
            if self.good.iter().all(|&g| g) {
                self.out.push(Group { blocks: self.blocks });
                self.group_end_bits.push(self.nbits - 1);
            }
            self.good = [false; 4];
        }
        self.expect = (self.expect + 1) % 4;
    }
}

/// Demodulate and decode RDS from an MPX signal sampled at `fs`.
pub fn decode_mpx(mpx: &[f32], fs: f64) -> Decoded {
    let bits = demodulate(mpx, fs);
    let mut sync = BlockSync::default();
    for &b in &bits.bits {
        sync.push(b);
    }
    let mut station = Station::default();
    for g in &sync.out {
        station.apply(g);
    }
    let group_end_times = sync
        .group_end_bits
        .iter()
        .map(|&b| bits.bit_times.get(b).copied().unwrap_or(f64::NAN) + 1.0 / 1187.5)
        .collect();
    let blocks_ok = sync.blocks_ok;
    let blocks_total = sync.blocks_total;
    Decoded {
        groups: sync.out,
        blocks_ok,
        blocks_total,
        station,
        level: bits.level,
        carrier_offset: bits.carrier_offset,
        errors_at: sync.errors_at,
        group_end_times,
    }
}

struct Demodulated {
    bits: Vec<bool>,
    /// Start time of each bit (seconds from the start of the input).
    bit_times: Vec<f64>,
    level: f64,
    carrier_offset: f64,
}

fn demodulate(mpx: &[f32], fs: f64) -> Demodulated {
    // 1. Mix the 57 kHz subcarrier to DC and decimate to ~19 kHz with a
    //    sharp channel filter (the stereo subband mixes down to 4-34 kHz).
    let dec = (fs / 19_000.0).floor().max(1.0) as usize;
    let fd = fs / dec as f64;
    let taps = txengine_dsp::FirDesign::lowpass(fs, 2_500.0, 4_200.0, 75.0);
    let w0 = 2.0 * std::f64::consts::PI * 57_000.0 / fs;
    let mut bb: Vec<(f64, f64)> = Vec::with_capacity(mpx.len() / dec + 1);
    let n = taps.len();
    let mut i = n;
    while i < mpx.len() {
        let (mut re, mut im) = (0.0, 0.0);
        for (k, &h) in taps.iter().enumerate() {
            let j = i - k;
            let ph = w0 * j as f64;
            let v = mpx[j] as f64 * h;
            re += v * ph.cos();
            im -= v * ph.sin();
        }
        bb.push((2.0 * re, 2.0 * im));
        i += dec;
    }

    // 2. Costas loop (BPSK) on the decimated baseband.
    let (bw, zeta) = (8.0 / fd, 0.707);
    let wn = 2.0 * std::f64::consts::PI * bw;
    let (kp, ki) = (2.0 * zeta * wn, wn * wn);
    let (mut phase, mut freq) = (0.0f64, 0.0f64);
    let mut base = Vec::with_capacity(bb.len());
    let mut power = 0.0;
    let (mut fsum, mut fcount) = (0.0, 0usize);
    for (k, &(re, im)) in bb.iter().enumerate() {
        let (s, c) = phase.sin_cos();
        let (i0, q0) = (re * c + im * s, im * c - re * s);
        let p = i0 * i0 + q0 * q0 + 1e-30;
        let err = i0 * q0 / p;
        freq += ki * err;
        phase += freq + kp * err;
        base.push(i0);
        if k > bb.len() / 4 {
            power += p;
            fsum += freq;
            fcount += 1;
        }
    }
    // Average frequency after the loop settled: the subcarrier offset, which
    // also scales the bit clock (subcarrier / 48).
    let carrier_offset = fsum / fcount.max(1) as f64 * fd / (2.0 * std::f64::consts::PI);
    // The baseband is the shaped data signal; its RMS is 1/sqrt(2) of the
    // all-zero reference sine amplitude (EN 50067 1.3).
    let level = (power / (bb.len() - bb.len() / 4 - 1).max(1) as f64).sqrt() * std::f64::consts::SQRT_2;

    // 3. Symbol timing: sample both lobes of each biphase symbol (t and
    //    t + Tb/2) and track the mid-symbol zero crossing (t + Tb/4).
    let sps = fd / (1187.5 * (1.0 + carrier_offset / 57_000.0));
    let at = |t: f64| -> f64 {
        let i = t.floor() as usize;
        if i + 1 >= base.len() {
            return 0.0;
        }
        let f = t - i as f64;
        base[i] * (1.0 - f) + base[i + 1] * f
    };
    let skip = (base.len() / 8) as f64; // let the Costas loop settle
    let mut best = (0.0, 0.0);
    for k in 0..64 {
        let off = skip + sps * k as f64 / 64.0;
        let e: f64 = (0..100).map(|n| (at(off + n as f64 * sps) - at(off + n as f64 * sps + sps / 2.0)).abs()).sum();
        if e > best.1 {
            best = (off, e);
        }
    }
    let mut t = best.0;
    let mut rate = sps;
    let mut bits = Vec::new();
    let mut bit_times = Vec::new();
    // Input time of decimated sample m: channel filter delay compensated.
    let t0 = n as f64 - (n as f64 - 1.0) / 2.0;
    let mut prev = 0.0;
    while t + sps < base.len() as f64 {
        let a = at(t);
        let b = at(t + sps / 2.0);
        let mid = at(t + sps / 4.0);
        let y = a - b;
        // Sampling early → the mid-symbol sample still carries the first
        // lobe's sign → positive error → move later (second order loop).
        let e = (mid * y.signum() / (y.abs() + 1e-30)).clamp(-1.0, 1.0);
        bits.push((y > 0.0) != (prev > 0.0));
        bit_times.push((t0 + t * dec as f64) / fs);
        rate += 2e-5 * e * sps;
        t += rate + 0.01 * e * sps;
        prev = y;
    }
    Demodulated { bits, bit_times, level, carrier_offset }
}

impl GroupType {
    /// Parse e.g. "2A".
    pub fn parse(s: &str) -> Option<GroupType> {
        let (num, ver) = s.split_at(s.len().checked_sub(1)?);
        let code: u8 = num.parse().ok()?;
        match ver {
            "A" | "a" => Some(GroupType::a(code)),
            "B" | "b" => Some(GroupType::b(code)),
            _ => None,
        }
    }
}
