//! Time the status request of a Pico (txengine firmware): the clock
//! discipline uses it to time the sample position, and drops readings that
//! take longer than 0.5 ms.
//!
//!     cargo run --release -p txengine-hal --example pico_status_rtt [SERIAL]

use std::time::Instant;

use txengine_hal::pico;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let which = std::env::args().nth(1);
    let usb = pico::find(which.as_deref())?;
    let p = pico::Pico::open(&usb)?;
    println!("{}", p.describe());
    let mut us: Vec<f64> = (0..2000)
        .map(|_| {
            let t0 = Instant::now();
            p.status().expect("status");
            t0.elapsed().as_secs_f64() * 1e6
        })
        .collect();
    us.sort_by(f64::total_cmp);
    let q = |f: f64| us[((us.len() - 1) as f64 * f) as usize];
    let under = us.iter().filter(|&&u| u <= 500.0).count();
    println!(
        "status round trip: min {:.0} µs, median {:.0}, 90 % {:.0}, max {:.0}; {:.0} % within 500 µs",
        q(0.0),
        q(0.5),
        q(0.9),
        q(1.0),
        100.0 * under as f64 / us.len() as f64
    );
    Ok(())
}
