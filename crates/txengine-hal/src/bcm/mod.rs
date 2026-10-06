//! BCM2835/6/7 and BCM2711 support (Raspberry Pi 2, 3, 4, Zero 2 W).

pub mod cprman;
pub mod dma;
pub mod fm;
pub mod regs;

use crate::error::{HalError, Result};
use crate::mmio::Mmio;
use crate::platform::{Platform, Soc};
use crate::sys::Mapping;

/// Mapped peripheral blocks.
#[derive(Debug, Clone)]
pub struct Bcm {
    pub cm: Mmio,
    pub dma: Mmio,
    pub gpio: Mmio,
    pub pads: Mmio,
    pub pwm: Mmio,
    pub pcm: Mmio,
    pub bcm2711: bool,
    pub soc: Soc,
    pub osc_hz: f64,
}

fn map(plat: &Platform, offset: usize, len: usize, what: &str) -> Result<Mmio> {
    let phys = plat.periph_phys_base + offset as u64;
    let m = Mapping::map_physical(phys, len, true, what).map_err(|e| {
        HalError::Unsupported(format!(
            "cannot map {what} registers ({e}). Run as root; kernels built with \
             CONFIG_IO_STRICT_DEVMEM need the `iomem=relaxed` boot parameter"
        ))
    })?;
    Ok(Mmio::new(m, regs::BUS_PERIPH_BASE as u64 + offset as u64))
}

impl Bcm {
    pub fn open(plat: &Platform) -> Result<Bcm> {
        if !plat.soc.has_bcm_peripherals() {
            return Err(HalError::Unsupported(format!("{} has no BCM283x peripherals", plat.soc)));
        }
        Ok(Bcm {
            cm: map(plat, regs::CM, regs::CM_LEN, "clock manager")?,
            dma: map(plat, regs::DMA, 0x1000, "DMA")?,
            gpio: map(plat, regs::GPIO, 0x1000, "GPIO")?,
            pads: map(plat, regs::PADS, 0x1000, "pad control")?,
            pwm: map(plat, regs::PWM, 0x1000, "PWM")?,
            pcm: map(plat, regs::PCM, 0x1000, "PCM")?,
            bcm2711: plat.soc == Soc::Bcm2711,
            soc: plat.soc,
            osc_hz: plat.osc_hz,
        })
    }
}

/// GPIOs that can carry a GPCLK: (gpio, gpclk index, ALT function code).
pub const GPCLK_PINS: &[(u32, usize, u32)] = &[
    (4, 0, regs::gpio::ALT0),
    (20, 0, regs::gpio::ALT5),
    (32, 0, regs::gpio::ALT0),
    (34, 0, regs::gpio::ALT0),
    (5, 1, regs::gpio::ALT0),
    (21, 1, regs::gpio::ALT5),
    (42, 1, regs::gpio::ALT0),
    (44, 1, regs::gpio::ALT0),
    (6, 2, regs::gpio::ALT0),
    (43, 2, regs::gpio::ALT0),
];

pub fn gpclk_for_pin(gpio: u32) -> Option<(usize, u32)> {
    GPCLK_PINS.iter().find(|p| p.0 == gpio).map(|p| (p.1, p.2))
}
