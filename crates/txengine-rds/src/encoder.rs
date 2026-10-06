//! RDS group generation: station data and group scheduling.

use crate::charset;
use crate::group::{Group, GroupType};

/// Duration of one group (104 bits at 1187.5 bit/s) in seconds.
pub const GROUP_SECONDS: f64 = 104.0 / 1187.5;

/// Decoder identification bits (EN 50067 3.2.1.5).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct DecoderInfo {
    /// d0: stereo transmission.
    pub stereo: bool,
    /// d1: artificial head (binaural) recording.
    pub artificial_head: bool,
    /// d2: compressed.
    pub compressed: bool,
    /// d3: PTY is switched dynamically.
    pub dynamic_pty: bool,
}

impl DecoderInfo {
    /// The DI bit carried in PS segment `seg` (d3 in segment 0 ... d0 in 3).
    fn bit_for_segment(self, seg: usize) -> bool {
        match seg {
            0 => self.dynamic_pty,
            1 => self.compressed,
            2 => self.artificial_head,
            _ => self.stereo,
        }
    }
}

/// Errors from validating station data.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RdsError(pub String);

impl std::fmt::Display for RdsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for RdsError {}

/// AF code (EN 50067 table 10) of a VHF frequency in MHz.
pub fn af_code(mhz: f64) -> Result<u8, RdsError> {
    let code = ((mhz - 87.5) * 10.0).round();
    if (1.0..=204.0).contains(&code) && ((mhz - 87.5) * 10.0 - code).abs() < 0.01 {
        Ok(code as u8)
    } else {
        Err(RdsError(format!("alternative frequency {mhz} MHz is not on the 87.6-107.9 MHz 100 kHz raster")))
    }
}

/// Station data carried in RDS.
#[derive(Debug, Clone, PartialEq)]
pub struct RdsParams {
    pub pi: u16,
    pub ps: [u8; 8],
    /// RadioText (already encoded, at most 64 characters).
    pub rt: Vec<u8>,
    pub pty: u8,
    pub tp: bool,
    pub ta: bool,
    /// Music (true) / speech (false).
    pub ms: bool,
    pub di: DecoderInfo,
    /// Alternative frequency codes (AF method A, at most 25).
    pub af: Vec<u8>,
    pub ptyn: Option<[u8; 8]>,
    /// Extended country code (group 1A variant 0).
    pub ecc: Option<u8>,
    /// Language identification code (group 1A variant 3).
    pub lic: Option<u16>,
    /// Transmit clock time (group 4A).
    pub ct: bool,
}

impl Default for RdsParams {
    fn default() -> Self {
        RdsParams {
            pi: 0x1234,
            ps: *b"txengine",
            rt: Vec::new(),
            pty: 0,
            tp: false,
            ta: false,
            ms: true,
            di: DecoderInfo::default(),
            af: Vec::new(),
            ptyn: None,
            ecc: None,
            lic: None,
            ct: true,
        }
    }
}

/// Encode a fixed 8-character field (PS, PTYN), space padded.
pub fn encode8(text: &str) -> Result<[u8; 8], RdsError> {
    let enc = charset::encode(text);
    if enc.len() > 8 {
        return Err(RdsError(format!("'{text}' is longer than 8 characters")));
    }
    let mut out = [b' '; 8];
    out[..enc.len()].copy_from_slice(&enc);
    Ok(out)
}

/// Encode RadioText (at most 64 characters).
pub fn encode_rt(text: &str) -> Result<Vec<u8>, RdsError> {
    let enc = charset::encode(text.trim_end());
    if enc.len() > 64 {
        return Err(RdsError(format!("RadioText is {} characters long; the maximum is 64", enc.len())));
    }
    Ok(enc)
}

/// Provides the local time offset from UTC in seconds at a Unix time.
pub type LocalOffsetFn = Box<dyn Fn(f64) -> i32 + Send>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Slot {
    Basic,
    RadioText,
    PtyName,
    Slow,
}

/// Generates the RDS group sequence.
pub struct RdsEncoder {
    params: RdsParams,
    ps_seg: usize,
    rt_seg: usize,
    rt_ab: bool,
    rt_segments: usize,
    ptyn_seg: usize,
    ptyn_ab: bool,
    af_pairs: Vec<(u8, u8)>,
    af_pos: usize,
    slow_variant: usize,
    credits: [f64; 4],
    last_ct_minute: Option<i64>,
    local_offset: LocalOffsetFn,
    groups_sent: u64,
}

impl std::fmt::Debug for RdsEncoder {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RdsEncoder").field("params", &self.params).finish_non_exhaustive()
    }
}

const WEIGHTS: [f64; 4] = [8.0, 4.0, 1.0, 1.0];

impl RdsEncoder {
    pub fn new(params: RdsParams) -> Result<RdsEncoder, RdsError> {
        let mut e = RdsEncoder {
            params: RdsParams::default(),
            ps_seg: 0,
            rt_seg: 0,
            rt_ab: false,
            rt_segments: 0,
            ptyn_seg: 0,
            ptyn_ab: false,
            af_pairs: Vec::new(),
            af_pos: 0,
            slow_variant: 0,
            credits: [0.0; 4],
            last_ct_minute: None,
            local_offset: Box::new(|_| 0),
            groups_sent: 0,
        };
        e.set_params(params)?;
        Ok(e)
    }

    /// Replace all station data at once (A/B flags toggle where content changed).
    pub fn set_params(&mut self, p: RdsParams) -> Result<(), RdsError> {
        if p.pty > 31 {
            return Err(RdsError(format!("PTY {} out of range 0-31", p.pty)));
        }
        if p.rt.len() > 64 {
            return Err(RdsError("RadioText longer than 64 characters".into()));
        }
        if p.af.len() > 25 || p.af.iter().any(|&c| !(1..=204).contains(&c)) {
            return Err(RdsError("AF list must hold at most 25 valid codes".into()));
        }
        if p.rt != self.params.rt {
            self.rt_ab = !self.rt_ab;
            self.rt_seg = 0;
        }
        if p.ptyn != self.params.ptyn {
            self.ptyn_ab = !self.ptyn_ab;
            self.ptyn_seg = 0;
        }
        if p.ps != self.params.ps {
            self.ps_seg = 0;
        }
        self.rt_segments = if p.rt.is_empty() {
            0
        } else if p.rt.len() == 64 {
            16
        } else {
            // Text plus the terminating carriage return.
            (p.rt.len() + 1).div_ceil(4)
        };
        self.af_pairs = af_pairs(&p.af);
        self.af_pos = 0;
        self.params = p;
        Ok(())
    }

    pub fn params(&self) -> &RdsParams {
        &self.params
    }

    /// Install the function used to obtain the local time offset for CT.
    pub fn set_local_offset_fn(&mut self, f: LocalOffsetFn) {
        self.local_offset = f;
    }

    pub fn groups_sent(&self) -> u64 {
        self.groups_sent
    }

    fn block2(&self, gt: GroupType) -> u16 {
        gt.bits() | (self.params.tp as u16) << 10 | (self.params.pty as u16) << 5
    }

    /// Produce the next group. `end_time` is the Unix time at which the
    /// group's last bit will be transmitted, used to place clock-time groups
    /// on the minute edge (EN 50067 3.1.5.6 note 4).
    pub fn next_group(&mut self, end_time: f64) -> Group {
        self.groups_sent += 1;
        if let Some(g) = self.clock_time_group(end_time) {
            return g;
        }
        let available = [
            true,
            self.rt_segments > 0,
            self.params.ptyn.is_some(),
            self.params.ecc.is_some() || self.params.lic.is_some(),
        ];
        let total: f64 = (0..4).filter(|&i| available[i]).map(|i| WEIGHTS[i]).sum();
        for i in 0..4 {
            if available[i] {
                self.credits[i] += WEIGHTS[i];
            } else {
                self.credits[i] = 0.0;
            }
        }
        let pick = (0..4).filter(|&i| available[i]).max_by(|&a, &b| self.credits[a].total_cmp(&self.credits[b])).unwrap();
        self.credits[pick] -= total;
        match [Slot::Basic, Slot::RadioText, Slot::PtyName, Slot::Slow][pick] {
            Slot::Basic => self.group_0a(),
            Slot::RadioText => self.group_2a(),
            Slot::PtyName => self.group_10a(),
            Slot::Slow => self.group_1a(),
        }
    }

    fn group_0a(&mut self) -> Group {
        let p = &self.params;
        let seg = self.ps_seg;
        let b2 = self.block2(GroupType::a(0))
            | (p.ta as u16) << 4
            | (p.ms as u16) << 3
            | (p.di.bit_for_segment(seg) as u16) << 2
            | seg as u16;
        let b3 = if self.af_pairs.is_empty() {
            // "No AF exists" followed by the filler code.
            (224 << 8) | 205
        } else {
            let (a, b) = self.af_pairs[self.af_pos];
            self.af_pos = (self.af_pos + 1) % self.af_pairs.len();
            (a as u16) << 8 | b as u16
        };
        let b4 = (p.ps[2 * seg] as u16) << 8 | p.ps[2 * seg + 1] as u16;
        self.ps_seg = (seg + 1) % 4;
        Group { blocks: [p.pi, b2, b3, b4] }
    }

    fn rt_char(&self, i: usize) -> u8 {
        let rt = &self.params.rt;
        match i.cmp(&rt.len()) {
            std::cmp::Ordering::Less => rt[i],
            std::cmp::Ordering::Equal => 0x0d,
            std::cmp::Ordering::Greater => b' ',
        }
    }

    fn group_2a(&mut self) -> Group {
        let seg = self.rt_seg;
        let b2 = self.block2(GroupType::a(2)) | (self.rt_ab as u16) << 4 | seg as u16;
        let c = |i| self.rt_char(4 * seg + i) as u16;
        let (b3, b4) = (c(0) << 8 | c(1), c(2) << 8 | c(3));
        self.rt_seg = (seg + 1) % self.rt_segments.max(1);
        Group { blocks: [self.params.pi, b2, b3, b4] }
    }

    fn group_10a(&mut self) -> Group {
        let name = self.params.ptyn.unwrap_or([b' '; 8]);
        let seg = self.ptyn_seg;
        let b2 = self.block2(GroupType::a(10)) | (self.ptyn_ab as u16) << 4 | seg as u16;
        let c = |i: usize| name[4 * seg + i] as u16;
        self.ptyn_seg = (seg + 1) % 2;
        Group { blocks: [self.params.pi, b2, c(0) << 8 | c(1), c(2) << 8 | c(3)] }
    }

    fn group_1a(&mut self) -> Group {
        // Alternate between the configured slow labelling codes.
        let variants: Vec<u16> = [
            self.params.ecc.map(|ecc| ecc as u16),
            self.params.lic.map(|lic| (3 << 12) | (lic & 0xfff)),
        ]
        .into_iter()
        .flatten()
        .collect();
        let b3 = variants[self.slow_variant % variants.len()];
        self.slow_variant = self.slow_variant.wrapping_add(1);
        // Block 4: programme item number; day 0 means "no valid PIN".
        Group { blocks: [self.params.pi, self.block2(GroupType::a(1)), b3, 0] }
    }

    fn clock_time_group(&mut self, end_time: f64) -> Option<Group> {
        if !self.params.ct || !end_time.is_finite() || end_time < 0.0 {
            return None;
        }
        // The minute edge closest to the end of this group.
        let minute = (end_time / 60.0).round() as i64;
        let edge = minute as f64 * 60.0;
        if (end_time - edge).abs() > GROUP_SECONDS / 2.0 || self.last_ct_minute == Some(minute) {
            return None;
        }
        self.last_ct_minute = Some(minute);
        let secs = minute * 60;
        let mjd = secs.div_euclid(86_400) + 40_587;
        let sod = secs.rem_euclid(86_400);
        let (hour, min) = ((sod / 3600) as u16, ((sod / 60) % 60) as u16);
        let offset = (self.local_offset)(edge);
        let half_hours = (offset.unsigned_abs() / 1800).min(31) as u16;
        let sign = (offset < 0) as u16;
        let mjd = mjd as u32;
        let b2 = self.block2(GroupType::a(4)) | ((mjd >> 15) & 0x3) as u16;
        let b3 = (((mjd & 0x7fff) as u16) << 1) | (hour >> 4);
        let b4 = (hour & 0xf) << 12 | min << 6 | sign << 5 | half_hours;
        Some(Group { blocks: [self.params.pi, b2, b3, b4] })
    }
}

/// AF method A pair sequence: (#AF, AF1), (AF2, AF3), ... padded with filler.
fn af_pairs(codes: &[u8]) -> Vec<(u8, u8)> {
    if codes.is_empty() {
        return Vec::new();
    }
    let mut seq = vec![224 + codes.len() as u8];
    seq.extend_from_slice(codes);
    if seq.len() % 2 == 1 {
        seq.push(205);
    }
    seq.chunks(2).map(|c| (c[0], c[1])).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn enc(p: RdsParams) -> RdsEncoder {
        RdsEncoder::new(p).unwrap()
    }

    #[test]
    fn ps_and_di_segments() {
        let mut e = enc(RdsParams {
            pi: 0x2abc,
            ps: encode8("RADIO 1").unwrap(),
            pty: 10,
            tp: true,
            di: DecoderInfo { stereo: true, ..Default::default() },
            ct: false,
            ..Default::default()
        });
        let mut ps = [0u8; 8];
        let mut di = 0u8;
        for _ in 0..8 {
            let g = e.next_group(f64::NAN);
            assert_eq!(g.blocks[0], 0x2abc);
            assert_eq!(g.group_type(), GroupType::a(0));
            assert_eq!((g.blocks[1] >> 5) & 0x1f, 10);
            assert!(g.blocks[1] & (1 << 10) != 0);
            assert_eq!(g.blocks[2], 0xe0cd, "no AF -> 224 + filler");
            let seg = (g.blocks[1] & 3) as usize;
            ps[2 * seg] = (g.blocks[3] >> 8) as u8;
            ps[2 * seg + 1] = g.blocks[3] as u8;
            di |= (((g.blocks[1] >> 2) & 1) as u8) << (3 - seg);
        }
        assert_eq!(&ps, b"RADIO 1 ");
        assert_eq!(di, 0b0001);
    }

    #[test]
    fn radiotext_terminated_and_ab_toggles() {
        let mut e = enc(RdsParams { rt: encode_rt("Hello world").unwrap(), ct: false, ..Default::default() });
        let mut segs = std::collections::BTreeMap::new();
        let mut ab = None;
        for _ in 0..40 {
            let g = e.next_group(f64::NAN);
            if g.group_type() == GroupType::a(2) {
                segs.insert(g.blocks[1] & 0xf, (g.blocks[2], g.blocks[3]));
                ab = Some(g.blocks[1] & 0x10);
            }
        }
        // 11 chars + CR = 12 -> 3 segments.
        assert_eq!(segs.len(), 3);
        assert_eq!(segs[&2], (u16::from_be_bytes([b'r', b'l']), u16::from_be_bytes([b'd', 0x0d])));
        let mut p = e.params().clone();
        p.rt = encode_rt("Second").unwrap();
        e.set_params(p).unwrap();
        loop {
            let g = e.next_group(f64::NAN);
            if g.group_type() == GroupType::a(2) {
                assert_ne!(Some(g.blocks[1] & 0x10), ab);
                assert_eq!(g.blocks[1] & 0xf, 0, "new text starts at segment 0");
                break;
            }
        }
    }

    #[test]
    fn af_method_a() {
        let codes: Vec<u8> = [89.3, 99.5, 101.7].iter().map(|&f| af_code(f).unwrap()).collect();
        assert_eq!(af_pairs(&codes), vec![(227, 18), (120, 142)]);
        let codes4: Vec<u8> = [89.3, 99.5, 101.7, 107.9].iter().map(|&f| af_code(f).unwrap()).collect();
        assert_eq!(af_pairs(&codes4), vec![(228, 18), (120, 142), (204, 205)]);
        assert!(af_code(108.0).is_err());
        assert!(af_code(87.55).is_err());
    }

    #[test]
    fn clock_time_on_minute_edge() {
        let mut e = enc(RdsParams::default());
        e.set_local_offset_fn(Box::new(|_| 7200));
        // 2026-10-05 21:43:00 UTC = 1791236580.
        let edge = 1_791_236_580.0;
        let mut t = edge - 1.0;
        let mut ct = None;
        while t < edge + 1.0 {
            let g = e.next_group(t);
            if g.group_type() == GroupType::a(4) {
                assert!(ct.is_none(), "exactly one CT group per minute");
                ct = Some((t, g));
            }
            t += GROUP_SECONDS;
        }
        let (t_end, g) = ct.expect("CT group sent");
        assert!((t_end - edge).abs() <= GROUP_SECONDS / 2.0 + 1e-9);
        let mjd = ((g.blocks[1] as u32 & 3) << 15) | (g.blocks[2] as u32 >> 1);
        assert_eq!(mjd, 61318); // 2026-10-05
        let hour = ((g.blocks[2] & 1) << 4) | (g.blocks[3] >> 12);
        let min = (g.blocks[3] >> 6) & 0x3f;
        assert_eq!((hour, min), (21, 43));
        assert_eq!(g.blocks[3] & 0x3f, 4, "+2h = 4 half hours, positive");
    }

    #[test]
    fn scheduler_mix() {
        let mut e = enc(RdsParams {
            rt: encode_rt("x").unwrap(),
            ptyn: Some(encode8("Football").unwrap()),
            ecc: Some(0xe2),
            ct: false,
            ..Default::default()
        });
        let mut count = std::collections::HashMap::new();
        for _ in 0..1400 {
            *count.entry(e.next_group(f64::NAN).group_type().code).or_insert(0) += 1;
        }
        assert_eq!(count[&0], 800);
        assert_eq!(count[&2], 400);
        assert_eq!(count[&10], 100);
        assert_eq!(count[&1], 100);
    }
}
