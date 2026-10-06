//! Dump the clock, PWM and DMA registers used by the BCM transmitter
//! (debugging aid; run as root while txengine is running).
//!
//!     bcm_dump [dma channel]
use txengine_hal::bcm::{Bcm, regs};
use txengine_hal::platform;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ch: usize = std::env::args().nth(1).map(|s| s.parse()).transpose()?.unwrap_or(14);
    let plat = platform::detect()?;
    let b = Bcm::open(&plat)?;
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
