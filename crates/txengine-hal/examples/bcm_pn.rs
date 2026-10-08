//! Phase-noise experiments on the BCM283x/BCM2711 boards: drive GPIO4 from
//! one clock path through GPCLK0 (integer division, no MASH, unmodulated)
//! for a while, then restore every register.
//!
//!     bcm_pn dump
//!     bcm_pn lock <ki> <count>                         PLLA lock behaviour (no output)
//!     bcm_pn osc <gp_div> <secs>                       GPCLK0 from the crystal oscillator
//!     bcm_pn per <a|c|d> <gp_div> <secs> [per_div]     from a PLL's PER channel as it runs
//!     bcm_pn plla <ndiv> <frac> <per_div> <gp_div> <secs> [ki kp ka]
//!
//! `plla` retunes PLLA (VCO = osc x (ndiv + frac / 2^20)) and optionally sets
//! its loop filter gains (ANA1 KI bits 19-21, KP bits 15-18; ANA3 KA bits
//! 7-9; Linux uses KI 2, KP 8, KA 2). The pads get drive PN_DRIVE (default 7)
//! and unlimited slew unless PN_SLEW=limited; PN_PREDIV=0/1 sets PLLA's
//! feedback pre-divider.
use std::time::Duration;

use txengine_hal::bcm::cprman::{Cprman, Pll, PllId};
use txengine_hal::bcm::regs::{self, cm::*};
use txengine_hal::bcm::Bcm;
use txengine_hal::platform;

fn arg<T: std::str::FromStr>(a: &[String], i: usize, what: &str) -> T {
    a.get(i).and_then(|s| s.parse().ok()).unwrap_or_else(|| panic!("missing or invalid {what}"))
}

fn dump(c: &Cprman, osc: f64) {
    for (name, cmr, ctrl, frac, ana0, per) in [
        ("PLLA", PLLA, A2W_PLLA_CTRL, A2W_PLLA_FRAC, A2W_PLLA_ANA0, A2W_PLLA_PER),
        ("PLLC", PLLC, A2W_PLLC_CTRL, A2W_PLLC_FRAC, A2W_PLLC_ANA0, A2W_PLLC_PER),
        ("PLLD", PLLD, A2W_PLLD_CTRL, A2W_PLLD_FRAC, A2W_PLLD_ANA0, A2W_PLLD_PER),
    ] {
        let (ct, fr) = (c.read(ctrl), c.read(frac) & A2W_FRAC_MASK);
        let ana: Vec<String> = (0..4).map(|i| format!("{:#08x}", c.read(ana0 + 4 * i))).collect();
        let a1 = c.read(ana0 + 4);
        println!(
            "{name}: cm {:#05x} ctrl {ct:#07x} (ndiv {} pdiv {}) frac {fr:#07x} -> VCO {:.3} MHz (x{} prediv) per {:#x} ana [{}] KI {} KP {} KA {}",
            c.read(cmr),
            ct & A2W_CTRL_NDIV_MASK,
            (ct & A2W_CTRL_PDIV_MASK) >> A2W_CTRL_PDIV_SHIFT,
            osc / 1e6 * ((ct & A2W_CTRL_NDIV_MASK) as f64 + fr as f64 / (1 << 20) as f64) * if a1 & ANA1_FB_PREDIV != 0 { 2.0 } else { 1.0 },
            if a1 & ANA1_FB_PREDIV != 0 { 2 } else { 1 },
            c.read(per),
            ana.join(" "),
            (a1 >> 19) & 7,
            (a1 >> 15) & 15,
            (c.read(ana0 + 12) >> 7) & 7
        );
    }
    println!("GP0: ctl {:#x} div {:#x}", c.read(GP0), c.read(div(GP0)));
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let plat = platform::detect()?;
    let b = Bcm::open(&plat)?;
    let cm = Cprman::new(b.cm.clone());
    let osc = b.osc_hz;
    dump(&cm, osc);
    if a.first().map(String::as_str) == Some("dump") {
        return Ok(());
    }
    let plla = Pll::new(&cm, PllId::A, osc, b.bcm2711);
    let saved_pll = plla.snapshot();
    if a.first().map(String::as_str) == Some("lock") {
        // Lock behaviour of PLLA (no clock output): with loop gain KI,
        // `n` retunes alternating between 2050 and 2350 MHz (lock time),
        // then `n` steps of +-2 MHz at the VCO (the largest FM deviation)
        // with the lock bit sampled for 1 ms after each.
        let (ki, n): (u32, usize) = (arg(&a, 1, "ki"), arg(&a, 2, "count"));
        let r = (|| -> Result<(), Box<dyn std::error::Error>> {
            plla.set_loop_ki(ki)?;
            let unit = osc * plla.fb_mult();
            let word = |vco: f64| {
                let u = vco / unit;
                (u.floor() as u32, ((u - u.floor()) * (1 << 20) as f64) as u32)
            };
            let set = |vco: f64| {
                let (nd, fr) = word(vco);
                cm.write(A2W_PLLA_FRAC, fr);
                let c = cm.read(A2W_PLLA_CTRL);
                cm.write(A2W_PLLA_CTRL, (c & !A2W_CTRL_NDIV_MASK) | nd);
            };
            let mut times = Vec::new();
            let mut failed = 0;
            for i in 0..n {
                set(if i % 2 == 0 { 2350e6 } else { 2050e6 });
                let t = std::time::Instant::now();
                // The lock bit may lag the write: wait for it to drop first.
                while plla.is_locked() && t.elapsed() < Duration::from_micros(200) {}
                while !plla.is_locked() && t.elapsed() < Duration::from_millis(100) {}
                if plla.is_locked() { times.push(t.elapsed().as_secs_f64() * 1e6) } else { failed += 1 }
                std::thread::sleep(Duration::from_millis(2));
            }
            times.sort_by(f64::total_cmp);
            let pct = |q: f64| times.get(((times.len() as f64 - 1.0) * q) as usize).copied().unwrap_or(f64::NAN);
            println!("KI {ki}: {} large steps, lock after {:.0} / {:.0} / {:.0} us (min/median/max), {failed} failed", n, pct(0.0), pct(0.5), pct(1.0));
            set(2277.6e6);
            plla.wait_lock()?;
            let mut drops = 0;
            for i in 0..n {
                set(2277.6e6 + if i % 2 == 0 { 2e6 } else { -2e6 });
                let t = std::time::Instant::now();
                let mut lost = false;
                while t.elapsed() < Duration::from_millis(1) {
                    lost |= !plla.is_locked();
                }
                drops += lost as usize;
            }
            println!("KI {ki}: {n} steps of 2 MHz, lock bit dropped after {drops}");
            Ok(())
        })();
        plla.restore(&saved_pll)?;
        dump(&cm, osc);
        return r;
    }
    let (gp_ctl, gp_div) = (cm.read(GP0), cm.read(div(GP0)));
    let (fsel_reg, sh) = (regs::gpio::fsel(4), regs::gpio::fsel_shift(4));
    let fsel = b.gpio.read(fsel_reg);
    let pads = b.pads.read(regs::pads::GPIO_0_27);
    let mut pll_changed = false;

    let run = |pll_changed: &mut bool| -> Result<(), Box<dyn std::error::Error>> {
        let (src, gdiv, secs): (u32, u32, f64) = match a.first().map(String::as_str) {
            Some("osc") => (SRC_OSC, arg(&a, 1, "gp_div"), arg(&a, 2, "seconds")),
            Some("per") => {
                let (id, src) = match a.get(1).map(String::as_str) {
                    Some("a") => (PllId::A, SRC_PLLA_PER),
                    Some("c") => (PllId::C, SRC_PLLC_PER),
                    Some("d") => (PllId::D, SRC_PLLD_PER),
                    _ => return Err("per needs a, c or d".into()),
                };
                if let Some(p) = a.get(4) {
                    if id != PllId::A {
                        return Err("only PLLA's PER divider may be changed".into());
                    }
                    plla.set_per(p.parse()?)?;
                    *pll_changed = true;
                }
                let pll = Pll::new(&cm, id, osc, b.bcm2711);
                println!("{} VCO {:.3} MHz, PER /{:?}", id.name(), pll.vco_hz() / 1e6, pll.per_divider());
                (src, arg(&a, 2, "gp_div"), arg(&a, 3, "seconds"))
            }
            Some("plla") => {
                let (ndiv, frac): (u32, u32) = (arg(&a, 1, "ndiv"), u32::from_str_radix(a[2].trim_start_matches("0x"), 16)?);
                let (per, gdiv): (u32, u32) = (arg(&a, 3, "per_div"), arg(&a, 4, "gp_div"));
                let secs: f64 = arg(&a, 5, "seconds");
                *pll_changed = true;
                if a.len() >= 9 {
                    let (ki, kp, ka): (u32, u32, u32) = (arg(&a, 6, "ki"), arg(&a, 7, "kp"), arg(&a, 8, "ka"));
                    let base = A2W_PLLA_ANA0;
                    let mut ana = [0, 1, 2, 3].map(|i| cm.read(base + 4 * i));
                    ana[1] = (ana[1] & !((7 << 19) | (15 << 15))) | (ki << 19) | (kp << 15);
                    ana[3] = (ana[3] & !(7 << 7)) | (ka << 7);
                    for i in (0..4).rev() {
                        cm.write(base + 4 * i, ana[i]);
                    }
                }
                // PN_PREDIV=0/1: feedback pre-divider (x2, used above 1.75 GHz);
                // written before the dividers, as Linux does when turning it off.
                if let Ok(v) = std::env::var("PN_PREDIV") {
                    // The four ANA registers latch together on the write of
                    // ANA0; write all of them, highest first.
                    let mut ana = [0, 1, 2, 3].map(|i| cm.read(A2W_PLLA_ANA0 + 4 * i));
                    ana[1] = if v == "1" { ana[1] | ANA1_FB_PREDIV } else { ana[1] & !ANA1_FB_PREDIV };
                    for i in (0..4).rev() {
                        cm.write(A2W_PLLA_ANA0 + 4 * i, ana[i]);
                    }
                }
                plla.retune(ndiv, frac)?;
                plla.set_per(per)?;
                let vco = plla.vco_hz();
                println!("PLLA VCO {:.4} MHz / {per} / {gdiv} = {:.6} MHz", vco / 1e6, vco / (per * gdiv) as f64 / 1e6);
                (SRC_PLLA_PER, gdiv, secs)
            }
            _ => return Err("usage: bcm_pn dump | osc <gp_div> <s> | per <a|c|d> <gp_div> <s> [per_div] | plla <ndiv> <frac> <per> <gp_div> <s> [ki kp ka]".into()),
        };
        cm.clock_start(GP0, src, gdiv, 0, 0)?;
        b.gpio.modify(fsel_reg, 7 << sh, regs::gpio::ALT0 << sh);
        // Pad settings for the test: PN_DRIVE (0-7, default 7), PN_SLEW=limited.
        let drive: u32 = std::env::var("PN_DRIVE").ok().and_then(|v| v.parse().ok()).unwrap_or(7);
        let mut pv = (pads & !regs::pads::DRIVE_MASK & 0xff_ffff) | drive;
        if std::env::var("PN_SLEW").is_ok_and(|v| v == "limited") {
            pv &= !regs::pads::SLEW_UNLIMITED;
        } else {
            pv |= regs::pads::SLEW_UNLIMITED;
        }
        b.pads.write(regs::pads::GPIO_0_27, regs::pads::PASSWORD | pv);
        println!("pads {:#x} -> {pv:#x}", pads & 0xff_ffff);
        dump(&cm, osc);
        std::thread::sleep(Duration::from_secs_f64(secs));
        Ok(())
    };
    let result = run(&mut pll_changed);
    println!("PLLA locked {}", plla.is_locked());
    b.gpio.write(fsel_reg, fsel);
    b.pads.write(regs::pads::GPIO_0_27, regs::pads::PASSWORD | (pads & 0xff_ffff));
    cm.clock_stop(GP0)?;
    cm.write(div(GP0), gp_div);
    cm.write(GP0, gp_ctl & !ENAB);
    if gp_ctl & ENAB != 0 {
        cm.write(GP0, gp_ctl);
    }
    if pll_changed {
        plla.restore(&saved_pll)?;
    }
    dump(&cm, osc);
    result
}
