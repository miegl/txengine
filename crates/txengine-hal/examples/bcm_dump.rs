//! Dump the clock, PWM and DMA registers used by the BCM transmitter
//! (debugging aid; run as root while txengine is running).
//!
//!     bcm_dump [dma channel] [--watch seconds]
//!
//! With `--watch`, polls the PLL, clock generator and pin registers every
//! millisecond and prints every change (and GPCLK0 as measured by the
//! frequency counter once per second), to catch the carrier chain being
//! changed behind txengine's back.
use std::time::{Duration, Instant};

use txengine_hal::bcm::cprman::Cprman;
use txengine_hal::bcm::{Bcm, regs};
use txengine_hal::platform;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let ch: usize = args.first().filter(|a| !a.starts_with("--")).map(|s| s.parse()).transpose()?.unwrap_or(14);
    let watch: Option<f64> = args.iter().position(|a| a == "--watch").and_then(|i| args.get(i + 1)).map(|s| s.parse()).transpose()?;
    let plat = platform::detect()?;
    let b = Bcm::open(&plat)?;
    if let Some(secs) = watch {
        use regs::cm::*;
        let c = Cprman::new(b.cm.clone());
        let regs: &[(&str, usize)] = &[
            ("CM_PLLA", PLLA), ("PLLA_CTRL", A2W_PLLA_CTRL), ("PLLA_ANA0", A2W_PLLA_ANA0), ("PLLA_ANA1", A2W_PLLA_ANA0 + 4),
            ("PLLA_ANA2", A2W_PLLA_ANA0 + 8), ("PLLA_ANA3", A2W_PLLA_ANA0 + 12), ("PLLA_PER", A2W_PLLA_PER), ("PLLA_CORE", A2W_PLLA_CORE),
            ("PLLA_DSI0", A2W_PLLA_DSI0), ("PLLA_CCP2", A2W_PLLA_CCP2), ("CM_PLLC", PLLC), ("PLLC_CTRL", A2W_PLLC_CTRL),
            ("PLLC_FRAC", A2W_PLLC_FRAC), ("PLLC_PER", A2W_PLLC_PER), ("PLLC_CORE0", A2W_PLLC_CORE0), ("LOCK", LOCK),
            ("XOSC", A2W_XOSC_CTRL), ("VPUCTL", VPU), ("VPUDIV", div(VPU)), ("V3DCTL", V3D), ("V3DDIV", div(V3D)),
            ("H264CTL", H264), ("ISPCTL", ISP), ("GP0CTL", GP0), ("GP0DIV", div(GP0)),
        ];
        let read_all = |out: &mut Vec<u32>| {
            out.clear();
            out.extend(regs.iter().map(|&(_, o)| b.cm.read(o)));
            out.push(b.gpio.read(regs::gpio::fsel(4)));
            out.push(b.pads.read(regs::pads::GPIO_0_27));
        };
        let names: Vec<&str> = regs.iter().map(|r| r.0).chain(["GPFSEL0", "PADS0"]).collect();
        let (mut prev, mut cur) = (Vec::new(), Vec::new());
        read_all(&mut prev);
        let t0 = Instant::now();
        println!("{}", names.iter().zip(&prev).map(|(n, v)| format!("{n}={v:#x}")).collect::<Vec<_>>().join(" "));
        let mut last_measure = Instant::now();
        while t0.elapsed().as_secs_f64() < secs {
            read_all(&mut cur);
            let changes: Vec<String> = names
                .iter()
                .zip(prev.iter().zip(&cur))
                .filter(|(_, (a, b))| a != b)
                .map(|(n, (a, b))| format!("{n} {a:#x}->{b:#x}"))
                .collect();
            if !changes.is_empty() {
                println!("{:8.3} s  {}", t0.elapsed().as_secs_f64(), changes.join("  "));
                std::mem::swap(&mut prev, &mut cur);
            }
            if last_measure.elapsed() > Duration::from_secs(1) {
                last_measure = Instant::now();
                let hz = c.measure(TCNT_GP0, plat.osc_hz, 0.001).unwrap_or(f64::NAN);
                println!("{:8.3} s  GP0 {:.3} MHz", t0.elapsed().as_secs_f64(), hz / 1e6);
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        return Ok(());
    }
    let cm = |name: &str, off: usize| println!("{name:10} {:#010x}", b.cm.read(off));
    cm("PWMCTL", regs::cm::PWM);
    cm("PWMDIV", regs::cm::div(regs::cm::PWM));
    cm("GP0CTL", regs::cm::GP0);
    cm("GP0DIV", regs::cm::div(regs::cm::GP0));
    cm("PLLA", regs::cm::PLLA);
    cm("PLLA_CTRL", regs::cm::A2W_PLLA_CTRL);
    cm("PLLA_FRAC", regs::cm::A2W_PLLA_FRAC);
    cm("PLLA_PER", regs::cm::A2W_PLLA_PER);
    cm("LOCK", regs::cm::LOCK);
    for (name, off) in [("CTL", 0x0), ("STA", 0x4), ("DMAC", 0x8), ("RNG1", 0x10), ("DAT1", 0x14)] {
        println!("PWM {name:6} {:#010x}", b.pwm.read(off));
    }
    let base = ch * 0x100;
    for (name, off) in [("CS", 0x0), ("CONBLK", 0x4), ("TI", 0x8), ("SRC", 0xc), ("DST", 0x10), ("LEN", 0x14), ("NEXT", 0x1c), ("DEBUG", 0x20)] {
        println!("DMA{ch} {name:6} {:#010x}", b.dma.read(base + off));
    }
    println!("DMA ENABLE {:#010x}  INT_STATUS {:#010x}", b.dma.read(0xff0), b.dma.read(0xfe0));
    Ok(())
}
