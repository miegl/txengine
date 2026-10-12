//! BCM2835/6/7 and BCM2711 support (Raspberry Pi 2, 3, 4, Zero 2 W).

pub mod cprman;
pub mod dma;
pub mod fm;
pub mod regs;

use std::time::Duration;

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
        // SAFETY: geteuid has no preconditions.
        let root = unsafe { libc::geteuid() } == 0;
        let denied = matches!(&e, HalError::Io { source, .. } if source.raw_os_error() == Some(libc::EPERM));
        let hint = if !root {
            "txengine must run as root"
        } else if denied {
            // The register block belongs to a kernel driver and the kernel
            // was built with CONFIG_IO_STRICT_DEVMEM.
            "this kernel blocks /dev/mem access to registers that a driver owns; \
             add `iomem=relaxed` to the kernel command line and reboot"
        } else {
            "kernels built with CONFIG_IO_STRICT_DEVMEM need the `iomem=relaxed` boot parameter"
        };
        HalError::Unsupported(format!("cannot map the {what} registers ({e}): {hint}"))
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

/// Pad control register of the bank `gpio` is in.
fn pads_reg(gpio: u32) -> usize {
    if gpio < 28 { regs::pads::GPIO_0_27 } else { regs::pads::GPIO_28_45 }
}

/// A GPIO's function and its bank's pad control, as found before use.
#[derive(Debug, Clone, Copy)]
pub struct PinState {
    gpio: u32,
    fsel: u32,
    pads: u32,
}

impl PinState {
    pub fn save(bcm: &Bcm, gpio: u32) -> PinState {
        PinState {
            gpio,
            fsel: (bcm.gpio.read(regs::gpio::fsel(gpio)) >> regs::gpio::fsel_shift(gpio)) & 7,
            pads: bcm.pads.read(pads_reg(gpio)),
        }
    }

    /// Put the function and the pad control back (`gpio`, `pads`: the
    /// register blocks).
    pub fn restore(&self, gpio: &Mmio, pads: &Mmio) {
        let sh = regs::gpio::fsel_shift(self.gpio);
        gpio.modify(regs::gpio::fsel(self.gpio), 7 << sh, self.fsel << sh);
        pads.write(pads_reg(self.gpio), regs::pads::PASSWORD | (self.pads & 0xff_ffff));
    }
}

/// Switch `pin` to the function `alt`, after setting its bank's pads to
/// the drive strength `drive` (0-7: 2-16 mA) with unlimited slew rate
/// (`gpio`, `pads`: the register blocks).
pub fn set_pin_function(gpio: &Mmio, pads: &Mmio, pin: u32, alt: u32, drive: u8) {
    use regs::pads::{HYST, PASSWORD, SLEW_UNLIMITED};
    pads.write(pads_reg(pin), PASSWORD | SLEW_UNLIMITED | HYST | drive as u32);
    let sh = regs::gpio::fsel_shift(pin);
    gpio.modify(regs::gpio::fsel(pin), 7 << sh, alt << sh);
}

/// Write a PWM register and wait until it reads back: every write must pass
/// the PWM's clock-domain synchroniser before the next one, and with a slow
/// PWM clock successive writes get lost (`STA.BERR`).
pub fn pwm_write_confirmed(pwm: &Mmio, reg: usize, v: u32) -> Result<()> {
    pwm.write(reg, v);
    crate::sys::wait_until(Duration::from_millis(5), "PWM register write", || pwm.read(reg) == v)?;
    Ok(())
}
