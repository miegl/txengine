//! Output a static carrier from the RP1 audio PLL (no DMA), optionally
//! rewriting the fractional divider from the CPU at a given rate.
//!
//! `rp1_static <freq_mhz> <offset_hz> <seconds> [cpu_rewrite_hz]`

use std::time::{Duration, Instant};

use txengine_hal::rp1::Rp1;
use txengine_hal::rp1::clocks::{ClockGen, Pll, PllId};
use txengine_hal::rp1::fm::{Rp1Pll, plan_clocks};
use txengine_hal::rp1::gpio::{Drive, Gpio};
use txengine_hal::rp1::regs;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let a: Vec<String> = std::env::args().collect();
    let freq: f64 = a.get(1).map(|s| s.parse()).transpose()?.unwrap_or(87.6);
    let offset: f64 = a.get(2).map(|s| s.parse()).transpose()?.unwrap_or(0.0);
    let secs: f64 = a.get(3).map(|s| s.parse()).transpose()?.unwrap_or(5.0);
    let rewrite: f64 = a.get(4).map(|s| s.parse()).transpose()?.unwrap_or(0.0);

    let rp1 = Rp1::open()?;
    let clocks = rp1.clocks();
    let set = rp1.block_set(regs::CLOCKS_MAIN);
    let clr = rp1.block_clr(regs::CLOCKS_MAIN);
    let pll = Pll::new(&clocks, PllId::Audio, 50e6);
    let gp = ClockGen::new(&clocks, &set, &clr, 0);
    let gpio = Gpio::new(rp1.block(regs::IO_BANK0), rp1.block(regs::PADS_BANK0));
    let (ps, gs, pin) = (pll.snapshot(), gp.snapshot(), gpio.snapshot(4)?);

    let plan = plan_clocks(Rp1Pll::Audio, 0, freq * 1e6, 50e6, 100e3)?;
    let frac = (plan.frac_center + offset * plan.frac_per_hz).round() as u32;
    println!("plan: int {} frac {:#x} (offset {offset} Hz), N {}", plan.fbdiv_int, frac, plan.n);
    pll.configure(plan.fbdiv_int, frac, plan.div1, plan.div2)?;
    gp.configure(plan.auxsrc, plan.gp_div, 0)?;
    gp.enable_output();
    gpio.set_function_output(4, 0, Drive::Ma12)?;

    let t0 = Instant::now();
    let reg = pll.frac_reg_offset();
    let mut writes = 0u64;
    if rewrite > 0.0 {
        let period = Duration::from_secs_f64(1.0 / rewrite);
        let mut next = Instant::now();
        while t0.elapsed().as_secs_f64() < secs {
            clocks.write(reg, frac);
            writes += 1;
            next += period;
            while Instant::now() < next {
                std::hint::spin_loop();
            }
        }
    } else {
        std::thread::sleep(Duration::from_secs_f64(secs));
    }
    println!("done ({writes} CPU rewrites), locked {}", pll.is_locked());
    gpio.restore(&pin);
    gp.restore(&gs);
    pll.restore(&ps)?;
    Ok(())
}
