//! RDS character set (EN 50067 Annex E, figure E.1: EBU Latin-based
//! repertoire, code table 00) – conversion from Unicode.

/// Characters of code points 0x80..=0xFF.
const HIGH: [char; 128] = [
    'á', 'à', 'é', 'è', 'í', 'ì', 'ó', 'ò', 'ú', 'ù', 'Ñ', 'Ç', 'Ş', 'ß', '¡', 'Ĳ', //
    'â', 'ä', 'ê', 'ë', 'î', 'ï', 'ô', 'ö', 'û', 'ü', 'ñ', 'ç', 'ş', 'ğ', 'ı', 'ĳ', //
    'ª', 'α', '©', '‰', 'Ğ', 'ě', 'ň', 'ő', 'π', '€', '£', '$', '←', '↑', '→', '↓', //
    'º', '¹', '²', '³', '±', 'İ', 'ń', 'ű', 'µ', '¿', '÷', '°', '¼', '½', '¾', '§', //
    'Á', 'À', 'É', 'È', 'Í', 'Ì', 'Ó', 'Ò', 'Ú', 'Ù', 'Ř', 'Č', 'Š', 'Ž', 'Đ', 'Ŀ', //
    'Â', 'Ä', 'Ê', 'Ë', 'Î', 'Ï', 'Ô', 'Ö', 'Û', 'Ü', 'ř', 'č', 'š', 'ž', 'đ', 'ŀ', //
    'Ã', 'Å', 'Æ', 'Œ', 'ŷ', 'Ý', 'Õ', 'Ø', 'Þ', 'Ŋ', 'Ŕ', 'Ć', 'Ś', 'Ź', 'Ŧ', 'ð', //
    'ã', 'å', 'æ', 'œ', 'ŵ', 'ý', 'õ', 'ø', 'þ', 'ŋ', 'ŕ', 'ć', 'ś', 'ź', 'ŧ', ' ', //
];

/// Encode one character, if it is representable.
pub fn encode_char(c: char) -> Option<u8> {
    match c {
        // The code table differs from ASCII at these positions.
        '$' => Some(0xab),
        '¤' => Some(0x24),
        '^' | '`' | '~' => None,
        '―' => Some(0x5e),
        '‖' | '║' => Some(0x60),
        '¯' => Some(0x7e),
        // Glyph variants of the same code points used by some tables.
        'β' => Some(0x8d),
        'ǧ' => Some(0x9d),
        'Ǧ' => Some(0xa4),
        '\n' => Some(0x0a),
        '\r' => Some(0x0d),
        ' '..='}' => Some(c as u8),
        _ => HIGH.iter().position(|&h| h == c).map(|i| 0x80 + i as u8),
    }
}

/// Closest representable substitute for characters outside the repertoire.
fn fallback(c: char) -> u8 {
    // Strip common diacritics not present in the table.
    let base = match c {
        'ą' | 'ă' | 'ā' => 'a',
        'Ą' | 'Ă' | 'Ā' => 'A',
        'ę' | 'ē' | 'ė' => 'e',
        'Ę' | 'Ē' | 'Ė' => 'E',
        'ł' => 'l',
        'Ł' => 'L',
        'ů' | 'ū' | 'ų' => 'u',
        'Ů' | 'Ū' | 'Ų' => 'U',
        'ť' | 'ţ' | 'ț' => 't',
        'Ť' | 'Ţ' | 'Ț' => 'T',
        'ď' => 'd',
        'Ď' => 'D',
        'ĺ' | 'ľ' | 'ļ' => 'l',
        'Ĺ' | 'Ľ' | 'Ļ' => 'L',
        'ș' => 'ş',
        'Ș' => 'Ş',
        'ż' => 'z',
        'Ż' => 'Z',
        'ő' => 'ö',
        '\u{2018}' | '\u{2019}' | '\u{201a}' | '´' => '\'',
        '\u{201c}' | '\u{201d}' | '\u{201e}' => '"',
        '\u{2013}' | '\u{2014}' => '-',
        '…' => '.',
        '\t' => ' ',
        _ => '?',
    };
    encode_char(base).unwrap_or(b'?')
}

/// Encode text for PS/RT/PTYN. Characters outside the repertoire are
/// approximated (diacritics dropped) or replaced by `?`.
pub fn encode(text: &str) -> Vec<u8> {
    text.chars().map(|c| encode_char(c).unwrap_or_else(|| fallback(c))).collect()
}

/// Decode back to Unicode (used by the verification decoder).
pub fn decode_char(b: u8) -> char {
    match b {
        0x24 => '¤',
        0x5e => '―',
        0x60 => '‖',
        0x7e => '¯',
        0x0a => '\n',
        0x0d => '\r',
        0x20..=0x7d => b as char,
        0x80..=0xff => HIGH[(b - 0x80) as usize],
        _ => ' ',
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn czech_roundtrip() {
        let s = "Příliš žluťoučký kůň úpěl ďábelské ódy";
        let enc = encode(s);
        let dec: String = enc.iter().map(|&b| decode_char(b)).collect();
        // ť, ů, ď are not in the repertoire and are approximated (ň is).
        assert_eq!(dec, "Příliš žlutoučký kuň úpěl dábelské ódy");
    }

    #[test]
    fn ascii_differences() {
        assert_eq!(encode("$5"), vec![0xab, b'5']);
        assert_eq!(encode("€"), vec![0xa9]);
        assert_eq!(encode("~"), vec![b'?']);
    }
}
