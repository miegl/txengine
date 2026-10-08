//! Transmit a test tone through the RP1 FM backend (Raspberry Pi 5).
//!
//! `rp1_tone [freq_mhz] [tone_hz] [deviation_hz] [seconds]`

use std::time::{Duration, Instant};

use txengine_core::RfSink;
use txengine_hal::rp1::fm::{Rp1FmConfig, Rp1FmTransmitter};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let a: Vec<String> = std::env::args().collect();
    let freq: f64 = a.get(1).map(|s| s.parse()).transpose()?.unwrap_or(87.6);
    let tone: f64 = a.get(2).map(|s| s.parse()).transpose()?.unwrap_or(1000.0);
    let dev: f64 = a.get(3).map(|s| s.parse()).transpose()?.unwrap_or(50_000.0);
    let secs: f64 = a.get(4).map(|s| s.parse()).transpose()?.unwrap_or(10.0);
    let square = a.get(5).is_some_and(|s| s == "square");
    // Multi-tone: `tone` = 0 selects a fixed set of probe tones, each at `dev`.
    let multi: Vec<f64> = if tone == 0.0 {
        vec![
            1003.0, 5003.0, 10007.0, 15013.0, 19009.0, 23011.0, 30011.0, 38017.0, 45007.0, 53003.0, 57007.0, 61001.0,
            67003.0, 75011.0, 85009.0, 95003.0,
        ]
    } else {
        vec![]
    };

    let cfg = Rp1FmConfig { carrier_hz: freq * 1e6, ..Default::default() };
    let mut tx = Rp1FmTransmitter::new(&cfg, 50e6)?;
    println!("{}", tx.info().description);
    let fs = tx.info().sample_rate;
    let mut n: u64 = 0;
    let mut buf = vec![0f32; 65536];
    let render = |out: &mut [f32], n: &mut u64| {
        for s in out.iter_mut() {
            let t = *n as f64 / fs;
            let v = if multi.is_empty() {
                let v = (2.0 * std::f64::consts::PI * tone * t).sin();
                if square { v.signum() } else { v }
            } else {
                multi.iter().enumerate().map(|(i, f)| (2.0 * std::f64::consts::PI * f * t + i as f64).sin()).sum()
            };
            *s = (dev * v) as f32;
            *n += 1;
        }
    };
    let p = tx.poll()?;
    render(&mut buf[..p.writable], &mut n);
    tx.write(&buf[..p.writable])?;
    tx.start()?;
    let t0 = Instant::now();
    let mut last = Instant::now();
    while t0.elapsed().as_secs_f64() < secs {
        let p = tx.poll()?;
        if p.writable > 0 {
            let w = p.writable.min(buf.len());
            render(&mut buf[..w], &mut n);
            tx.write(&buf[..w])?;
        }
        if last.elapsed() > Duration::from_secs(1) {
            last = Instant::now();
            println!(
                "t={:.1}s consumed {} written {} underruns {} rate {:.1}",
                t0.elapsed().as_secs_f64(),
                p.consumed,
                p.written,
                p.underruns,
                p.consumed as f64 / t0.elapsed().as_secs_f64()
            );
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    tx.stop()?;
    println!("stopped");
    Ok(())
}
