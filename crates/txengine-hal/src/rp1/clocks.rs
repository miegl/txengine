//! RP1 clock manager: PLLs, general purpose clock generators and the
//! frequency counters used to verify them.

use std::time::Duration;

use log::debug;

use super::regs::clk::*;
use crate::error::{HalError, Result};
use crate::mmio::Mmio;
use crate::sys::wait_until;

const LOCK_TIMEOUT: Duration = Duration::from_millis(100);

/// One of RP1's three fractional-N PLLs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PllId {
    Sys,
    Audio,
    Video,
}

impl PllId {
    fn base(self) -> usize {
        match self {
            PllId::Sys => PLL_SYS,
            PllId::Audio => PLL_AUDIO,
            PllId::Video => PLL_VIDEO,
        }
    }
    pub fn name(self) -> &'static str {
        match self {
            PllId::Sys => "pll_sys",
            PllId::Audio => "pll_audio",
            PllId::Video => "pll_video",
        }
    }
}

/// Raw register snapshot of a PLL (for restoring on exit).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PllRegs {
    pub cs: u32,
    pub pwr: u32,
    pub fbdiv_int: u32,
    pub fbdiv_frac: u32,
    pub prim: u32,
    pub sec: u32,
}

/// Handle to one PLL.
#[derive(Debug, Clone)]
pub struct Pll {
    clk: Mmio,
    pub id: PllId,
    xosc_hz: f64,
}

impl Pll {
    pub fn new(clocks: &Mmio, id: PllId, xosc_hz: f64) -> Pll {
        Pll { clk: clocks.clone(), id, xosc_hz }
    }

    fn reg(&self, r: usize) -> usize {
        self.id.base() + r
    }

    pub fn read(&self, r: usize) -> u32 {
        self.clk.read(self.reg(r))
    }

    pub fn snapshot(&self) -> PllRegs {
        PllRegs {
            cs: self.read(PLL_CS),
            pwr: self.read(PLL_PWR),
            fbdiv_int: self.read(PLL_FBDIV_INT),
            fbdiv_frac: self.read(PLL_FBDIV_FRAC),
            prim: self.read(PLL_PRIM),
            sec: self.read(PLL_SEC),
        }
    }

    pub fn is_locked(&self) -> bool {
        self.read(PLL_CS) & PLL_CS_LOCK != 0
    }

    pub fn refdiv(&self) -> u32 {
        (self.read(PLL_CS) & PLL_CS_REFDIV_MASK).max(1)
    }

    /// Reference frequency at the phase detector.
    pub fn ref_hz(&self) -> f64 {
        self.xosc_hz / self.refdiv() as f64
    }

    /// Current VCO frequency computed from the dividers.
    pub fn vco_hz(&self) -> f64 {
        let int = self.read(PLL_FBDIV_INT) as f64;
        let frac = self.read(PLL_FBDIV_FRAC) as f64 / (1u64 << PLL_FRAC_BITS) as f64;
        self.ref_hz() * (int + frac)
    }

    /// Primary output post-dividers (div1, div2).
    pub fn prim_divs(&self) -> (u32, u32) {
        let p = self.read(PLL_PRIM);
        ((p & PLL_PRIM_DIV1_MASK) >> PLL_PRIM_DIV1_SHIFT, (p & PLL_PRIM_DIV2_MASK) >> PLL_PRIM_DIV2_SHIFT)
    }

    /// Program the feedback divider (VCO = ref * (int + frac / 2^24)) and
    /// the primary post dividers, then wait for lock.
    ///
    /// The PLL keeps running throughout (as the kernel's set_rate does), so
    /// consumers see a frequency step rather than a stopped clock.
    pub fn configure(&self, fbdiv_int: u32, fbdiv_frac: u32, div1: u32, div2: u32) -> Result<()> {
        if !(16..=320).contains(&fbdiv_int) || fbdiv_frac >= (1 << PLL_FRAC_BITS) {
            return Err(HalError::Config(format!(
                "{}: feedback divider {fbdiv_int}+{fbdiv_frac}/2^24 out of range",
                self.id.name()
            )));
        }
        if !(1..=7).contains(&div1) || !(1..=7).contains(&div2) {
            return Err(HalError::Config(format!("{}: post dividers {div1}x{div2} out of range", self.id.name())));
        }
        // Fractional mode needs the delta-sigma modulator powered (DSMPD = 0);
        // VCO, post dividers and the PLL itself must be powered up.
        let pwr = self.read(PLL_PWR) & !(PLL_PWR_PD | PLL_PWR_DSMPD | PLL_PWR_POSTDIVPD | PLL_PWR_VCOPD);
        self.clk.write(self.reg(PLL_PWR), pwr);
        self.clk.write(self.reg(PLL_FBDIV_INT), fbdiv_int);
        self.clk.write(self.reg(PLL_FBDIV_FRAC), fbdiv_frac);
        let prim = (self.read(PLL_PRIM) & !(PLL_PRIM_DIV1_MASK | PLL_PRIM_DIV2_MASK))
            | (div1 << PLL_PRIM_DIV1_SHIFT)
            | (div2 << PLL_PRIM_DIV2_SHIFT);
        self.clk.write(self.reg(PLL_PRIM), prim);
        self.wait_lock()?;
        // Read back: the registers must hold exactly what we wrote.
        let (rint, rfrac) = (self.read(PLL_FBDIV_INT), self.read(PLL_FBDIV_FRAC));
        if rint != fbdiv_int || rfrac != fbdiv_frac || self.read(PLL_PRIM) != prim {
            return Err(HalError::Hardware(format!(
                "{}: divider read-back mismatch (int {rint} frac {rfrac:#x})",
                self.id.name()
            )));
        }
        debug!(
            "{}: VCO {:.6} MHz (fbdiv {fbdiv_int} + {fbdiv_frac:#08x}/2^24), prim /{div1}/{div2}, locked",
            self.id.name(),
            self.vco_hz() / 1e6
        );
        Ok(())
    }

    pub fn wait_lock(&self) -> Result<()> {
        wait_until(LOCK_TIMEOUT, &format!("{} lock", self.id.name()), || self.is_locked()).map(|_| ())
    }

    /// Offset of the FBDIV_FRAC register within the clock manager (the
    /// register the DMA engine writes to modulate the carrier).
    pub fn frac_reg_offset(&self) -> usize {
        self.reg(PLL_FBDIV_FRAC)
    }

    /// Restore a snapshot taken with [`Pll::snapshot`].
    pub fn restore(&self, s: &PllRegs) -> Result<()> {
        self.clk.write(self.reg(PLL_FBDIV_INT), s.fbdiv_int);
        self.clk.write(self.reg(PLL_FBDIV_FRAC), s.fbdiv_frac);
        self.clk.write(self.reg(PLL_PRIM), s.prim);
        self.clk.write(self.reg(PLL_PWR), s.pwr);
        if s.cs & PLL_CS_LOCK != 0 && s.pwr & PLL_PWR_PD == 0 {
            self.wait_lock()?;
        }
        Ok(())
    }
}

/// A clock generator of the clock manager: a general purpose one
/// (`clk_gp0` .. `clk_gp5`, which can drive GPIO pads) or a peripheral's
/// (`clk_pwm0`). They share the register layout: CTRL, DIV_INT, DIV_FRAC,
/// SEL.
#[derive(Debug, Clone)]
pub struct ClockGen {
    clk: Mmio,
    clk_set: Mmio,
    clk_clr: Mmio,
    name: &'static str,
    /// Offset of the CTRL register.
    ctrl: usize,
    /// Its bit in GPCLK_OE_CTRL (pad output enable), if it has one.
    oe: Option<u32>,
}

/// Raw snapshot of a clock generator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClockGenRegs {
    pub ctrl: u32,
    pub div_int: u32,
    pub div_frac: u32,
    pub oe: bool,
}

impl ClockGen {
    /// General purpose clock generator `n` (0..=5).
    pub fn new(clocks: &Mmio, clocks_set: &Mmio, clocks_clr: &Mmio, n: usize) -> ClockGen {
        const NAMES: [&str; 6] = ["clk_gp0", "clk_gp1", "clk_gp2", "clk_gp3", "clk_gp4", "clk_gp5"];
        ClockGen {
            clk: clocks.clone(),
            clk_set: clocks_set.clone(),
            clk_clr: clocks_clr.clone(),
            name: NAMES[n],
            ctrl: gp_ctrl(n),
            oe: Some(1 << n),
        }
    }

    /// The clock of the PWM0 block.
    pub fn pwm0(clocks: &Mmio, clocks_set: &Mmio, clocks_clr: &Mmio) -> ClockGen {
        ClockGen {
            clk: clocks.clone(),
            clk_set: clocks_set.clone(),
            clk_clr: clocks_clr.clone(),
            name: "clk_pwm0",
            ctrl: CLK_PWM0_CTRL,
            oe: None,
        }
    }

    pub fn name(&self) -> &'static str {
        self.name
    }

    fn div_int(&self) -> usize {
        self.ctrl + 0x4
    }

    fn div_frac(&self) -> usize {
        self.ctrl + 0x8
    }

    pub fn snapshot(&self) -> ClockGenRegs {
        ClockGenRegs {
            ctrl: self.clk.read(self.ctrl),
            div_int: self.clk.read(self.div_int()),
            div_frac: self.clk.read(self.div_frac()),
            oe: self.oe.is_some_and(|b| self.clk.read(GPCLK_OE_CTRL) & b != 0),
        }
    }

    pub fn is_enabled(&self) -> bool {
        self.clk.read(self.ctrl) & CTRL_ENABLE != 0
    }

    /// The one-hot source selection status register.
    pub fn selected(&self) -> u32 {
        self.clk.read(self.ctrl + 0xc)
    }

    /// Stop the clock and disable its pad output.
    pub fn disable(&self) {
        if let Some(b) = self.oe {
            self.clk_clr.write(GPCLK_OE_CTRL, b);
        }
        self.clk_clr.write(self.ctrl, CTRL_ENABLE);
    }

    /// Select auxiliary source `auxsrc`, program an integer (+ 16 bit
    /// fractional) divider and enable the output. Verifies that the
    /// glitchless mux has switched to the requested source.
    pub fn configure(&self, auxsrc: u32, div_int: u32, div_frac16: u32) -> Result<()> {
        if div_int == 0 || div_int > 0xffff || div_frac16 > 0xffff {
            return Err(HalError::Config(format!("{}: invalid divider {div_int}.{div_frac16}", self.name)));
        }
        self.disable();
        let ctrl = (self.clk.read(self.ctrl) & !(CTRL_AUXSRC_MASK | CTRL_ENABLE))
            | ((auxsrc << CTRL_AUXSRC_SHIFT) & CTRL_AUXSRC_MASK);
        self.clk.write(self.ctrl, ctrl);
        self.clk.write(self.div_int(), div_int);
        self.clk.write(self.div_frac(), div_frac16 << DIV_FRAC_SHIFT);
        self.clk_set.write(self.ctrl, CTRL_ENABLE);
        wait_until(Duration::from_millis(10), &format!("{} start", self.name), || {
            self.clk.read(self.ctrl) & CTRL_ENABLED != 0
        })?;
        let ctrl_rb = self.clk.read(self.ctrl);
        if (ctrl_rb & CTRL_AUXSRC_MASK) >> CTRL_AUXSRC_SHIFT != auxsrc || ctrl_rb & CTRL_ENABLE == 0 {
            return Err(HalError::Hardware(format!("{}: control read-back {ctrl_rb:#x}", self.name)));
        }
        Ok(())
    }

    /// Enable the clock's output to the GPIO pads.
    pub fn enable_output(&self) {
        if let Some(b) = self.oe {
            self.clk_set.write(GPCLK_OE_CTRL, b);
        }
    }

    pub fn restore(&self, s: &ClockGenRegs) {
        self.disable();
        self.clk.write(self.div_int(), s.div_int);
        self.clk.write(self.div_frac(), s.div_frac);
        self.clk.write(self.ctrl, s.ctrl);
        if s.oe {
            self.enable_output();
        }
    }
}

/// Frequency counter source selector for a clock (`FC_NUM(idx, src)` in the kernel).
#[derive(Debug, Clone, Copy)]
pub struct FcSource {
    pub idx: usize,
    pub src: u32,
}

impl FcSource {
    pub const fn gp(n: usize) -> FcSource {
        FcSource { idx: n, src: 1 }
    }
    pub const PLL_SYS: FcSource = FcSource { idx: 0, src: 2 };
    pub const PLL_AUDIO: FcSource = FcSource { idx: 4, src: 2 };
    pub const PLL_AUDIO_PRI_PH: FcSource = FcSource { idx: 5, src: 1 };
    pub const PWM0: FcSource = FcSource { idx: 0, src: 5 };
    pub const PLL_VIDEO: FcSource = FcSource { idx: 3, src: 2 };
}

/// Measure a clock with the hardware frequency counter. Returns Hz.
/// The counter's resolution is 1/32 kHz.
pub fn measure(clocks: &Mmio, fc: FcSource, ref_hz: f64) -> Result<f64> {
    let off = fc.idx * FC_SIZE;
    wait_until(Duration::from_millis(100), "frequency counter idle", || {
        clocks.read(off + FC0_STATUS) & FC_STATUS_RUNNING == 0
    })?;
    clocks.write(off + FC0_REF_KHZ, (ref_hz / 1000.0).round() as u32);
    clocks.write(off + FC0_MIN_KHZ, 0);
    clocks.write(off + FC0_MAX_KHZ, 0x1ff_ffff);
    clocks.write(off + FC0_INTERVAL, 8);
    clocks.write(off + FC0_DELAY, 7);
    clocks.write(off + FC0_SRC, fc.src);
    let res = wait_until(Duration::from_millis(100), "frequency counter result", || {
        clocks.read(off + FC0_STATUS) & FC_STATUS_DONE != 0
    });
    let result = clocks.read(off + FC0_RESULT);
    clocks.write(off + FC0_SRC, 0);
    res?;
    Ok(result as f64 / (1u32 << FC_RESULT_FRAC_BITS) as f64 * 1000.0)
}
