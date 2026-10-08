//! Phase-noise experiments on the Raspberry Pi 5: drive GPIO4 from one clock
//! path, unmodulated, for a while, then restore every register.
//!
//!     rp1_pn dump
//!     rp1_pn off                                   GPIO4 off, pll_audio back to the kernel's setting
//!     rp1_pn xosc <div> <secs>                     clk_gp0 from the 50 MHz crystal
//!     rp1_pn sys <div> <secs>                      clk_gp0 from pll_sys (as it runs)
//!     rp1_pn audio <int> <frac> <div1> <div2> <secs> [pwr] [gp_div]
//!
//! `audio` programs pll_audio directly: VCO = 50 MHz x (int + frac / 2^24),
//! output VCO / div1 / div2. `pwr` (hex) is written to PLL_PWR as is
//! (bit 1 DACPD, bit 2 DSMPD, ...); by default DSMPD is set for frac = 0,
//! as the kernel does, and everything else is powered.
use std::time::Duration;

use txengine_hal::rp1::Rp1;
use txengine_hal::rp1::clocks::{ClockGen, Pll, PllId};
use txengine_hal::rp1::gpio::{Drive, Gpio};
use txengine_hal::rp1::regs::{self, clk::*};

fn dump(c: &txengine_hal::mmio::Mmio) {
    for (name, base) in [("sys", PLL_SYS), ("audio", PLL_AUDIO), ("video", PLL_VIDEO)] {
        println!(
            "pll_{name}: cs {:#010x} pwr {:#04x} int {} frac {:#08x} prim {:#07x} sec {:#x}",
            c.read(base + PLL_CS),
            c.read(base + PLL_PWR),
            c.read(base + PLL_FBDIV_INT),
            c.read(base + PLL_FBDIV_FRAC),
            c.read(base + PLL_PRIM),
            c.read(base + PLL_SEC)
        );
    }
    println!(
        "gp0: ctrl {:#x} div_int {} div_frac {:#x}",
        c.read(gp_ctrl(0)),
        c.read(gp_div_int(0)),
        c.read(gp_div_frac(0))
    );
}

fn arg<T: std::str::FromStr>(a: &[String], i: usize, what: &str) -> T {
    a.get(i).and_then(|s| s.parse().ok()).unwrap_or_else(|| panic!("missing or invalid {what}"))
}

fn hex(s: &str) -> u32 {
    u32::from_str_radix(s.trim_start_matches("0x"), 16).expect("hex value")
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let rp1 = Rp1::open()?;
    let clocks = rp1.clocks();
    if a.first().map(String::as_str) == Some("dump") {
        dump(&clocks);
        let (io, pads) = (rp1.block(regs::IO_BANK0), rp1.block(regs::PADS_BANK0));
        println!(
            "gpio4: ctrl {:#x} pad {:#x}, GPCLK_OE {:#x}",
            io.read(regs::gpio::ctrl(4)),
            pads.read(regs::gpio::pad(4)),
            clocks.read(GPCLK_OE_CTRL)
        );
        return Ok(());
    }
    if a.first().map(String::as_str) == Some("off") {
        // Stop GPCLK0, take GPIO4 off the clock (NULL function, output
        // disabled) and put pll_audio back to the kernel's 1536 MHz / 5 / 5.
        let set = rp1.block_set(regs::CLOCKS_MAIN);
        let clr = rp1.block_clr(regs::CLOCKS_MAIN);
        ClockGen::new(&clocks, &set, &clr, 0).disable();
        let (io, pads) = (rp1.block(regs::IO_BANK0), rp1.block(regs::PADS_BANK0));
        io.write(regs::gpio::ctrl(4), (io.read(regs::gpio::ctrl(4)) & !regs::gpio::CTRL_FUNCSEL_MASK) | 0x1f);
        pads.write(regs::gpio::pad(4), pads.read(regs::gpio::pad(4)) | regs::gpio::PAD_OD);
        clocks.write(PLL_AUDIO + PLL_PWR, 0);
        clocks.write(PLL_AUDIO + PLL_FBDIV_INT, 30);
        clocks.write(PLL_AUDIO + PLL_FBDIV_FRAC, 0xb851ec);
        clocks.write(PLL_AUDIO + PLL_PRIM, 0x55000);
        Pll::new(&clocks, PllId::Audio, 50e6).wait_lock()?;
        dump(&clocks);
        println!(
            "gpio4: ctrl {:#x} pad {:#x}, GPCLK_OE {:#x}",
            io.read(regs::gpio::ctrl(4)),
            pads.read(regs::gpio::pad(4)),
            clocks.read(GPCLK_OE_CTRL)
        );
        return Ok(());
    }
    let set = rp1.block_set(regs::CLOCKS_MAIN);
    let clr = rp1.block_clr(regs::CLOCKS_MAIN);
    let pll = Pll::new(&clocks, PllId::Audio, 50e6);
    let gp = ClockGen::new(&clocks, &set, &clr, 0);
    let gpio = Gpio::new(rp1.block(regs::IO_BANK0), rp1.block(regs::PADS_BANK0));
    let (ps, gs, pin) = (pll.snapshot(), gp.snapshot(), gpio.snapshot(4)?);
    dump(&clocks);

    let secs: f64;
    match a.first().map(String::as_str) {
        Some("xosc") => {
            let div: u32 = arg(&a, 1, "divider");
            secs = arg(&a, 2, "seconds");
            gp.configure(0, div, 0)?;
            println!("clk_gp0 = xosc / {div} = {:.6} MHz", 50.0 / div as f64);
        }
        Some("sys") => {
            let div: u32 = arg(&a, 1, "divider");
            secs = arg(&a, 2, "seconds");
            gp.configure(6, div, 0)?;
            println!("clk_gp0 = pll_sys / {div}");
        }
        Some("audio") => {
            let (int, frac) = (arg::<u32>(&a, 1, "int"), hex(&a[2]));
            let (d1, d2): (u32, u32) = (arg(&a, 3, "div1"), arg(&a, 4, "div2"));
            secs = arg(&a, 5, "seconds");
            let pwr = a.get(6).map(|s| hex(s)).unwrap_or(if frac == 0 { PLL_PWR_DSMPD } else { 0 });
            let base = PLL_AUDIO;
            clocks.write(base + PLL_PWR, pwr);
            clocks.write(base + PLL_FBDIV_INT, int);
            clocks.write(base + PLL_FBDIV_FRAC, frac);
            let prim = (clocks.read(base + PLL_PRIM) & !(PLL_PRIM_DIV1_MASK | PLL_PRIM_DIV2_MASK))
                | (d1 << PLL_PRIM_DIV1_SHIFT)
                | (d2 << PLL_PRIM_DIV2_SHIFT);
            clocks.write(base + PLL_PRIM, prim);
            pll.wait_lock()?;
            let gdiv: u32 = a.get(7).map(|s| s.parse()).transpose()?.unwrap_or(1);
            gp.configure(7, gdiv, 0)?;
            let vco = 50.0 * (int as f64 + frac as f64 / (1u64 << 24) as f64);
            println!(
                "pll_audio VCO {vco:.6} MHz / {d1} / {d2} / {gdiv} = {:.6} MHz, pwr {pwr:#x}",
                vco / (d1 * d2 * gdiv) as f64
            );
        }
        _ => {
            return Err(
                "usage: rp1_pn dump | xosc <div> <s> | sys <div> <s> | audio <int> <frac> <d1> <d2> <s> [pwr] [gp_div]"
                    .into(),
            );
        }
    }
    gp.enable_output();
    // PN_DRIVE: pad drive level 0-7 (default 7 = 12 mA); PN_SLEW=slow clears SLEWFAST.
    let drive = std::env::var("PN_DRIVE").ok().and_then(|v| v.parse().ok()).unwrap_or(7);
    gpio.set_function_output(4, 0, Drive::from_level(drive))?;
    if std::env::var("PN_SLEW").is_ok_and(|v| v == "slow") {
        let pads = rp1.block(regs::PADS_BANK0);
        let off = regs::gpio::pad(4);
        pads.write(off, pads.read(off) & !regs::gpio::PAD_SLEWFAST);
    }
    println!("pad {:#x}", rp1.block(regs::PADS_BANK0).read(regs::gpio::pad(4)));
    dump(&clocks);
    std::thread::sleep(Duration::from_secs_f64(secs));
    println!("locked {}", pll.is_locked());
    gpio.restore(&pin);
    gp.restore(&gs);
    pll.restore(&ps)?;
    Ok(())
}
