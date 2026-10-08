//! Debug: decode RDS from a mono float/16-bit WAV MPX file (100 kHz full scale).
use txengine_rds::decoder::{BlockSync, decode_mpx};

fn main() {
    let path = std::env::args().nth(1).expect("file");
    let b = std::fs::read(&path).unwrap();
    // Minimal WAV parser: find "fmt " and "data".
    let mut i = 12;
    let (mut fmt, mut rate, mut bits, mut data) = (0u16, 0u32, 0u16, &b[..0]);
    while i + 8 <= b.len() {
        let id = &b[i..i + 4];
        let len = u32::from_le_bytes(b[i + 4..i + 8].try_into().unwrap()) as usize;
        let body = &b[i + 8..(i + 8 + len).min(b.len())];
        if id == b"fmt " {
            fmt = u16::from_le_bytes([body[0], body[1]]);
            rate = u32::from_le_bytes(body[4..8].try_into().unwrap());
            bits = u16::from_le_bytes([body[14], body[15]]);
        } else if id == b"data" {
            data = body;
        }
        i += 8 + len + (len & 1);
    }
    let x: Vec<f32> = if fmt == 3 {
        data.chunks_exact(4).map(|c| f32::from_le_bytes(c.try_into().unwrap()) * 100e3).collect()
    } else {
        assert_eq!(bits, 16);
        data.chunks_exact(2).map(|c| i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0 * 100e3).collect()
    };
    let d = decode_mpx(&x, rate as f64);
    println!(
        "{} groups, blocks ok {}/{} level {:.0} offset {:.2}",
        d.groups.len(),
        d.blocks_ok,
        d.blocks_total,
        d.level,
        d.carrier_offset
    );
    println!("errors at bits {:?} (of {} bits)", d.errors_at, x.len() as f64 / rate as f64 * 1187.5);
    let _ = BlockSync::default();
}
