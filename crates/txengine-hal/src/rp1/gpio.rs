//! RP1 GPIO bank 0: function selection and pad control.

use super::regs::gpio::*;
use crate::error::{HalError, Result};
use crate::mmio::Mmio;

/// GPIOs that can carry a GPCLK output: (gpio, gpclk index, function select).
pub const GPCLK_PINS: &[(u32, usize, u32)] = &[
    (4, 0, 0),  // a0: GPCLK0
    (5, 1, 0),  // a0: GPCLK1
    (6, 2, 0),  // a0: GPCLK2
    (18, 1, 8), // a8: GPCLK1
    (20, 0, 3), // a3: GPCLK0
    (21, 1, 3), // a3: GPCLK1
];

/// GPIOs that can carry a PWM0 channel: (gpio, channel, function select).
pub const PWM0_PINS: &[(u32, usize, u32)] = &[
    (12, 0, 0), // a0: PWM0_CHAN0
    (13, 1, 0), // a0: PWM0_CHAN1
    (18, 2, 3), // a3: PWM0_CHAN2
    (19, 3, 3), // a3: PWM0_CHAN3
];

/// Look up the PWM0 routing of a pin: (channel, function select).
pub fn pwm0_for_pin(gpio: u32) -> Option<(usize, u32)> {
    PWM0_PINS.iter().find(|p| p.0 == gpio).map(|p| (p.1, p.2))
}

/// Look up the GPCLK routing of a pin.
pub fn gpclk_for_pin(gpio: u32) -> Option<(usize, u32)> {
    GPCLK_PINS.iter().find(|p| p.0 == gpio).map(|p| (p.1, p.2))
}

/// Saved state of one pin.
#[derive(Debug, Clone, Copy)]
pub struct PinRegs {
    pub gpio: u32,
    pub ctrl: u32,
    pub pad: u32,
}

/// Pad drive strength.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Drive {
    Ma2 = 0,
    Ma4 = 1,
    Ma8 = 2,
    Ma12 = 3,
}

impl Drive {
    /// Map PiFmAdv-style power levels 0..=7 (2..16 mA) to RP1's four steps.
    pub fn from_level(level: u8) -> Drive {
        match level {
            0..=1 => Drive::Ma2,
            2..=3 => Drive::Ma4,
            4..=5 => Drive::Ma8,
            _ => Drive::Ma12,
        }
    }
    pub fn milliamps(self) -> u32 {
        [2, 4, 8, 12][self as usize]
    }
}

#[derive(Debug, Clone)]
pub struct Gpio {
    io: Mmio,
    pads: Mmio,
}

impl Gpio {
    pub fn new(io: Mmio, pads: Mmio) -> Gpio {
        Gpio { io, pads }
    }

    fn check(gpio: u32) -> Result<usize> {
        if gpio > 27 {
            return Err(HalError::Config(format!("GPIO{gpio} is not on RP1 bank 0")));
        }
        Ok(gpio as usize)
    }

    pub fn snapshot(&self, gpio: u32) -> Result<PinRegs> {
        let g = Self::check(gpio)?;
        Ok(PinRegs { gpio, ctrl: self.io.read(ctrl(g)), pad: self.pads.read(pad(g)) })
    }

    pub fn restore(&self, s: &PinRegs) {
        let g = s.gpio as usize;
        // Function first (so the pin never drives a stale function with the
        // output enabled), then the pad.
        self.io.write(ctrl(g), s.ctrl);
        self.pads.write(pad(g), s.pad);
    }

    /// Route peripheral function `funcsel` to the pin as an output with the
    /// given drive strength and fast slew rate.
    pub fn set_function_output(&self, gpio: u32, funcsel: u32, drive: Drive) -> Result<()> {
        let g = Self::check(gpio)?;
        let p =
            (self.pads.read(pad(g)) & !(PAD_OD | PAD_DRIVE_MASK)) | PAD_SLEWFAST | ((drive as u32) << PAD_DRIVE_SHIFT);
        self.pads.write(pad(g), p);
        let c = (self.io.read(ctrl(g)) & !(CTRL_FUNCSEL_MASK | CTRL_OUTOVER_MASK | CTRL_OEOVER_MASK))
            | (funcsel & CTRL_FUNCSEL_MASK);
        self.io.write(ctrl(g), c);
        let (rc, rp) = (self.io.read(ctrl(g)), self.pads.read(pad(g)));
        if rc != c || rp != p {
            return Err(HalError::Hardware(format!(
                "GPIO{gpio}: read-back mismatch (ctrl {rc:#x} != {c:#x}, pad {rp:#x} != {p:#x})"
            )));
        }
        Ok(())
    }
}
