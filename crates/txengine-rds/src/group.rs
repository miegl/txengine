//! RDS baseband coding (EN 50067 clause 2): blocks, checkwords, groups.

/// Generator polynomial g(x) = x^10 + x^8 + x^7 + x^5 + x^4 + x^3 + 1.
const POLY: u32 = 0b101_1011_1001;

/// Offset words (EN 50067 Annex A).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Offset {
    A,
    B,
    C,
    /// C' – block 3 of version B groups.
    Cp,
    D,
}

impl Offset {
    pub const fn word(self) -> u16 {
        match self {
            Offset::A => 0x0fc,
            Offset::B => 0x198,
            Offset::C => 0x168,
            Offset::Cp => 0x350,
            Offset::D => 0x1b4,
        }
    }
}

/// The 10-bit checkword of a 16-bit information word (before the offset).
pub fn checkword(info: u16) -> u16 {
    let mut reg: u32 = (info as u32) << 10;
    for bit in (10..26).rev() {
        if reg & (1 << bit) != 0 {
            reg ^= POLY << (bit - 10);
        }
    }
    reg as u16 & 0x3ff
}

/// A complete 26-bit block: information word followed by checkword + offset.
pub fn encode_block(info: u16, offset: Offset) -> u32 {
    ((info as u32) << 10) | (checkword(info) ^ offset.word()) as u32
}

/// Syndrome of a received 26-bit block (H matrix multiplication). Equals the
/// offset word of the block if it was received without error.
pub fn syndrome(block: u32) -> u16 {
    let info = (block >> 10) as u16;
    let check = (block & 0x3ff) as u16;
    checkword(info) ^ check
}

/// Group type, e.g. 0A = `GroupType { code: 0, b: false }`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GroupType {
    pub code: u8,
    pub b: bool,
}

impl GroupType {
    pub const fn a(code: u8) -> Self {
        GroupType { code, b: false }
    }
    pub const fn b(code: u8) -> Self {
        GroupType { code, b: true }
    }
    /// The 5 most significant bits of block 2.
    pub const fn bits(self) -> u16 {
        ((self.code as u16) << 12) | ((self.b as u16) << 11)
    }
}

impl std::fmt::Display for GroupType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}{}", self.code, if self.b { 'B' } else { 'A' })
    }
}

/// One RDS group (four 16-bit information words).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Group {
    pub blocks: [u16; 4],
}

impl Group {
    pub fn group_type(&self) -> GroupType {
        GroupType { code: (self.blocks[1] >> 12) as u8, b: self.blocks[1] & (1 << 11) != 0 }
    }

    /// The 104 bits of the group, most significant bit first, with
    /// checkwords and offsets (C' in block 3 of version B groups).
    pub fn bits(&self) -> [bool; 104] {
        let offsets = [Offset::A, Offset::B, if self.group_type().b { Offset::Cp } else { Offset::C }, Offset::D];
        let mut out = [false; 104];
        for (i, (&info, &off)) in self.blocks.iter().zip(&offsets).enumerate() {
            let blk = encode_block(info, off);
            for j in 0..26 {
                out[i * 26 + j] = blk & (1 << (25 - j)) != 0;
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn checkword_matches_annex_b() {
        // Annex B: the check bits of each of the 16 unit vectors (rows of G).
        let rows = [
            0x077, 0x2e7, 0x3af, 0x30b, 0x359, 0x370, 0x1b8, 0x0dc, 0x06e, 0x037, 0x2c7, 0x3bf, 0x303, 0x35d, 0x372,
            0x1b9,
        ];
        for (i, &r) in rows.iter().enumerate() {
            assert_eq!(checkword(1 << (15 - i)), r, "row {i}");
        }
    }

    #[test]
    fn syndrome_identifies_offsets() {
        for (off, info) in [(Offset::A, 0x1234u16), (Offset::B, 0x0408), (Offset::Cp, 0xe20d), (Offset::D, 0x4142)] {
            assert_eq!(syndrome(encode_block(info, off)), off.word());
        }
    }
}
