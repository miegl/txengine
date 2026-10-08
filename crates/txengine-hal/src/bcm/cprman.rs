//! BCM283x/BCM2711 clock manager ("CPRMAN"): clock generators, PLLs and the
//! frequency counter. Every write carries the 0x5A password; every state
//! change is confirmed through the BUSY / LOCK status bits.

use std::time::Duration;

use log::{debug, warn};

use super::regs::cm::*;
use crate::error::{HalError, Result};
use crate::mmio::Mmio;
use crate::sys::wait_until;

const TIMEOUT: Duration = Duration::from_millis(100);

/// The clock manager register block.
#[derive(Debug, Clone)]
pub struct Cprman {
    regs: Mmio,
}

impl Cprman {
    pub fn new(regs: Mmio) -> Cprman {
        Cprman { regs }
    }

    #[inline]
    pub fn read(&self, off: usize) -> u32 {
        self.regs.read(off)
    }

    /// Password-protected write (the top byte of every register).
    #[inline]
    pub fn write(&self, off: usize, val: u32) {
        self.regs.write(off, PASSWORD | (val & 0x00ff_ffff));
    }

    /// Address of a register as seen by the DMA engine.
    pub fn bus_addr(off: usize) -> u32 {
        super::regs::BUS_PERIPH_BASE + (super::regs::CM + off) as u32
    }

    /// Stop the clock generator at `ctl` and wait until it is idle.
    pub fn clock_stop(&self, ctl: usize) -> Result<()> {
        let c = self.read(ctl);
        if c & ENAB != 0 {
            self.write(ctl, c & !ENAB);
        }
        if wait_until(TIMEOUT, "clock generator stop", || self.read(ctl) & BUSY == 0).is_err() {
            // A generator whose source has stopped never clears BUSY; KILL
            // resets it (may glitch, but the output is being stopped anyway).
            warn!("clock generator at {ctl:#x} did not stop; killing it");
            self.write(ctl, (self.read(ctl) & !ENAB) | KILL);
            wait_until(TIMEOUT, "clock generator kill", || self.read(ctl) & BUSY == 0)?;
            self.write(ctl, self.read(ctl) & !KILL);
        }
        Ok(())
    }

    /// Program and start a clock generator: source, integer/fractional
    /// divider and MASH order. Glitch free: stops it first.
    pub fn clock_start(&self, ctl: usize, src: u32, divi: u32, divf: u32, mash: u32) -> Result<()> {
        if !(1..=0xfff).contains(&divi) || divf > 0xfff || mash > 3 {
            return Err(HalError::Config(format!("invalid clock divider {divi}.{divf} / MASH {mash}")));
        }
        self.clock_stop(ctl)?;
        self.write(div(ctl), (divi << DIVI_SHIFT) | divf);
        let base = (src & SRC_MASK) | (mash << MASH_SHIFT);
        self.write(ctl, base);
        self.write(ctl, base | ENAB);
        wait_until(TIMEOUT, "clock generator start", || self.read(ctl) & BUSY != 0)?;
        let (rc, rd) = (self.read(ctl) & 0x7ff, self.read(div(ctl)) & 0xff_ffff);
        if rc & (SRC_MASK | MASH_MASK | ENAB) != base | ENAB || rd != (divi << DIVI_SHIFT) | divf {
            return Err(HalError::Hardware(format!("clock generator {ctl:#x}: read-back ctl {rc:#x} div {rd:#x}")));
        }
        Ok(())
    }

    /// Measure a clock with the frequency counter (`tcnt_mux` as in the
    /// Linux driver) over `seconds` of the crystal; returns Hz, or 0 if the
    /// clock is not running.
    ///
    /// The count register sits in the measured clock's domain: when that
    /// clock is stopped, KILL cannot clear it and the previous result reads
    /// back (seen on a Zero 2 W: a disabled GPCLK "measured" 87.6 MHz). Two
    /// gate times expose a stale count, as it does not grow with the gate.
    pub fn measure(&self, tcnt_mux: u32, osc_hz: f64, seconds: f64) -> Result<f64> {
        let short = self.count(tcnt_mux, osc_hz, seconds / 2.0)?;
        let long = self.count(tcnt_mux, osc_hz, seconds)?;
        if (long as f64) < 1.5 * short as f64 || long == 0 {
            debug!("frequency counter: {short} then {long} counts; the clock is not running");
            return Ok(0.0);
        }
        Ok(long as f64 / seconds)
    }

    fn count(&self, tcnt_mux: u32, osc_hz: f64, seconds: f64) -> Result<u32> {
        self.write(TCNTCTL, KILL);
        self.write(TCNTCTL, (tcnt_mux & SRC_MASK) | ((tcnt_mux >> 4) << TCNT_SRC1_SHIFT));
        let count = (osc_hz * seconds) as u32;
        self.write(OSCCOUNT, count);
        std::thread::sleep(Duration::from_secs_f64(seconds));
        wait_until(TIMEOUT, "frequency counter", || self.read(OSCCOUNT) == 0)?;
        wait_until(TIMEOUT, "frequency counter idle", || self.read(TCNTCTL) & BUSY == 0)?;
        let n = self.read(TCNTCNT);
        self.write(TCNTCTL, 0);
        Ok(n)
    }
}

/// One of the PLLs a GPCLK can be fed from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PllId {
    A,
    C,
    D,
}

struct PllRegs {
    cm: usize,
    ctrl: usize,
    frac: usize,
    ana0: usize,
    per: usize,
    lock_bit: u32,
    xosc_enable: u32,
    gpclk_src: u32,
}

impl PllId {
    fn regs(self) -> PllRegs {
        match self {
            PllId::A => PllRegs {
                cm: PLLA,
                ctrl: A2W_PLLA_CTRL,
                frac: A2W_PLLA_FRAC,
                ana0: A2W_PLLA_ANA0,
                per: A2W_PLLA_PER,
                lock_bit: 1 << 8,
                xosc_enable: 1 << 6,
                gpclk_src: SRC_PLLA_PER,
            },
            PllId::C => PllRegs {
                cm: PLLC,
                ctrl: A2W_PLLC_CTRL,
                frac: A2W_PLLC_FRAC,
                ana0: A2W_PLLC_ANA0,
                per: A2W_PLLC_PER,
                lock_bit: 1 << 10,
                xosc_enable: 1 << 0,
                gpclk_src: SRC_PLLC_PER,
            },
            PllId::D => PllRegs {
                cm: PLLD,
                ctrl: A2W_PLLD_CTRL,
                frac: A2W_PLLD_FRAC,
                ana0: A2W_PLLD_ANA0,
                per: A2W_PLLD_PER,
                lock_bit: 1 << 11,
                xosc_enable: 1 << 4,
                gpclk_src: SRC_PLLD_PER,
            },
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            PllId::A => "PLLA",
            PllId::C => "PLLC",
            PllId::D => "PLLD",
        }
    }

    /// GPCLK source number of the PLL's PER channel.
    pub fn gpclk_source(self) -> u32 {
        self.regs().gpclk_src
    }

    /// All channels of this PLL other than PER: (name, A2W register,
    /// VPU-mux source number if a CM clock can select it, LOAD bit in the
    /// CM_PLLx register).
    fn other_channels(self) -> &'static [(&'static str, usize, Option<u32>, u32)] {
        match self {
            PllId::A => &[
                ("PLLA_CORE", A2W_PLLA_CORE, Some(4), 1 << 4),
                ("PLLA_DSI0", A2W_PLLA_DSI0, None, 1 << 0),
                ("PLLA_CCP2", A2W_PLLA_CCP2, None, 1 << 2),
            ],
            PllId::C => &[
                ("PLLC_CORE0", A2W_PLLC_CORE0, Some(5), 1 << 0),
                ("PLLC_CORE1", A2W_PLLC_CORE1, Some(8), 1 << 2),
                ("PLLC_CORE2", A2W_PLLC_CORE2, Some(9), 1 << 4),
            ],
            PllId::D => &[
                ("PLLD_CORE", A2W_PLLD_CORE, Some(6), 1 << 4),
                ("PLLD_DSI0", A2W_PLLD_DSI0, None, 1 << 0),
                ("PLLD_DSI1", A2W_PLLD_DSI1, None, 1 << 2),
            ],
        }
    }
}

/// Saved PLL state.
#[derive(Debug, Clone, Copy)]
pub struct PllState {
    pub cm: u32,
    pub ctrl: u32,
    pub frac: u32,
    pub ana: [u32; 4],
    pub per: u32,
    /// The other channels' dividers, as listed by `other_channels`.
    pub channels: [u32; 3],
    pub xosc: u32,
}

/// PLL settings that determine its PER output, apart from the fraction
/// (see [`Pll::config`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PllConfig {
    pub cm: u32,
    pub ctrl: u32,
    pub ana: [u32; 4],
    pub per: u32,
    /// The other channels' dividers (not compared by [`Pll::matches`]).
    pub channels: [u32; 3],
}

/// Peripheral-mux clocks (sources 4/5/6 are PLLA/C/D PER).
const PER_CLOCKS: &[(&str, usize)] = &[
    ("cam0", CAM0),
    ("cam1", CAM1),
    ("ccp2", CCP2),
    ("dsi0e", DSI0E),
    ("dpi", DPI),
    ("gp0", GP0),
    ("gp1", GP0 + 8),
    ("gp2", GP0 + 16),
    ("hsm", HSM),
    ("pcm", PCM),
    ("pwm", PWM),
    ("slim", SLIM),
    ("smi", SMI),
    ("uart", UART),
    ("vec", VEC),
    ("dsi1e", DSI1E),
    ("dft", DFT),
    ("aveo", AVEO),
    ("emmc", EMMC),
];
/// VPU-mux clocks (4 = PLLA_CORE, 5 = PLLC_CORE0, 6 = PLLD_CORE, 8/9 = PLLC_CORE1/2).
const VPU_CLOCKS: &[(&str, usize)] = &[("vpu", VPU), ("h264", H264), ("isp", ISP), ("v3d", V3D), ("sdram", SDRAM)];
/// The VPU-mux clocks of the GPU's video and 3D blocks: idle unless video
/// is decoded or encoded, the camera is used or 3D is drawn.
const GPU_CLOCKS: &[&str] = &["h264", "isp", "v3d"];

/// Other users of a PLL (see [`Pll::users`]).
#[derive(Debug, Clone, Default)]
pub struct PllUsers {
    /// Enabled CM clocks selecting its PER channel.
    pub per: Vec<String>,
    /// Its enabled non-PER channels, with the VPU-mux clocks using them.
    pub other: Vec<String>,
    /// Every one of them feeds only the GPU's blocks ([`GPU_CLOCKS`]), or
    /// nothing: no clock of the VPU, the SD card, the UARTs or a display.
    pub gpu_only: bool,
}

impl PllUsers {
    pub fn is_empty(&self) -> bool {
        self.per.is_empty() && self.other.is_empty()
    }

    /// All users, for messages.
    pub fn names(&self) -> Vec<String> {
        self.per.iter().chain(&self.other).cloned().collect()
    }
}

/// Handle to a PLL.
#[derive(Debug, Clone)]
pub struct Pll {
    cm: Cprman,
    pub id: PllId,
    osc_hz: f64,
    /// BCM2835/6/7 have a feedback pre-divider bit; BCM2711 does not.
    has_prediv: bool,
    emmc2: bool,
}

impl Pll {
    pub fn new(cm: &Cprman, id: PllId, osc_hz: f64, bcm2711: bool) -> Pll {
        Pll { cm: cm.clone(), id, osc_hz, has_prediv: !bcm2711, emmc2: bcm2711 }
    }

    pub fn snapshot(&self) -> PllState {
        let r = self.id.regs();
        PllState {
            cm: self.cm.read(r.cm),
            ctrl: self.cm.read(r.ctrl),
            frac: self.cm.read(r.frac),
            ana: [0, 1, 2, 3].map(|i| self.cm.read(r.ana0 + 4 * i)),
            per: self.cm.read(r.per),
            channels: self.channel_dividers(),
            xosc: self.cm.read(A2W_XOSC_CTRL),
        }
    }

    pub fn is_powered(&self) -> bool {
        self.cm.read(self.id.regs().ctrl) & A2W_CTRL_PWRDN == 0
    }

    pub fn is_locked(&self) -> bool {
        self.cm.read(LOCK) & self.id.regs().lock_bit != 0
    }

    /// Multiplier from crystal to VCO for one unit of NDIV (2 when the
    /// feedback pre-divider is engaged).
    pub fn fb_mult(&self) -> f64 {
        if self.has_prediv && self.cm.read(self.id.regs().ana0 + 4) & ANA1_FB_PREDIV != 0 { 2.0 } else { 1.0 }
    }

    /// Current VCO frequency.
    pub fn vco_hz(&self) -> f64 {
        let r = self.id.regs();
        let ctrl = self.cm.read(r.ctrl);
        let ndiv = (ctrl & A2W_CTRL_NDIV_MASK) as f64;
        let pdiv = (((ctrl & A2W_CTRL_PDIV_MASK) >> A2W_CTRL_PDIV_SHIFT) as f64).max(1.0);
        let frac = (self.cm.read(r.frac) & A2W_FRAC_MASK) as f64 / (1u32 << A2W_FRAC_BITS) as f64;
        self.osc_hz * self.fb_mult() * (ndiv + frac) / pdiv
    }

    /// PER channel divider (0 means 256); None if the channel is disabled.
    pub fn per_divider(&self) -> Option<u32> {
        let v = self.cm.read(self.id.regs().per);
        if v & A2W_CHANNEL_DISABLE != 0 {
            return None;
        }
        Some(match v & A2W_CHANNEL_DIV_MASK {
            0 => 256,
            d => d,
        })
    }

    /// Other users of this PLL: enabled CM clocks selecting its PER channel
    /// (excluding `own_gpclk_ctl`) and enabled non-PER channels.
    pub fn users(&self, own_gpclk_ctl: usize) -> PllUsers {
        let mut per_users = Vec::new();
        let mut other = Vec::new();
        let mut gpu_only = true;
        let src = self.id.gpclk_source();
        for &(name, ctl) in PER_CLOCKS.iter().chain(self.emmc2.then_some(&("emmc2", EMMC2))) {
            if ctl == own_gpclk_ctl {
                continue;
            }
            let c = self.cm.read(ctl);
            if c & (ENAB | BUSY) != 0 && c & SRC_MASK == src {
                per_users.push(name.to_string());
            }
        }
        for &(cname, reg, vpu_src, _) in self.id.other_channels() {
            if self.cm.read(reg) & A2W_CHANNEL_DISABLE != 0 {
                continue;
            }
            let mut users: Vec<&str> = Vec::new();
            if let Some(s) = vpu_src {
                for &(name, ctl) in VPU_CLOCKS {
                    let c = self.cm.read(ctl);
                    // The VPU clock has no ENAB bit and is always running.
                    if (c & (ENAB | BUSY) != 0 || ctl == VPU) && c & SRC_MASK == s {
                        users.push(name);
                    }
                }
            } else {
                // DSI and CCP2 channels clock a display or camera directly.
                gpu_only = false;
            }
            gpu_only &= users.iter().all(|u| GPU_CLOCKS.contains(u));
            other.push(if users.is_empty() { cname.to_string() } else { format!("{cname} ({})", users.join(", ")) });
        }
        gpu_only &= per_users.is_empty();
        PllUsers { per: per_users, other, gpu_only }
    }

    /// Bring a powered-down PLL up (Linux `bcm2835_pll_on`).
    pub fn power_up(&self, ndiv: u32, frac: u32) -> Result<()> {
        let r = self.id.regs();
        self.cm.write(A2W_XOSC_CTRL, self.cm.read(A2W_XOSC_CTRL) | r.xosc_enable);
        // Loop filter defaults (KI=2, KP=8, KA=2), pre-divider off.
        let mut ana = [0, 1, 2, 3].map(|i| self.cm.read(r.ana0 + 4 * i));
        ana[1] = (ana[1] & !(ANA1_KI_MASK | ANA1_KP_MASK)) | (2 << ANA1_KI_SHIFT) | (8 << ANA1_KP_SHIFT);
        if self.has_prediv {
            ana[1] &= !ANA1_FB_PREDIV;
        }
        ana[3] = (ana[3] & !ANA3_KA_MASK) | (2 << ANA3_KA_SHIFT);
        for i in (0..4).rev() {
            self.cm.write(r.ana0 + 4 * i, ana[i]);
        }
        self.cm.write(r.frac, frac);
        let ctrl = (self.cm.read(r.ctrl) & !(A2W_CTRL_NDIV_MASK | A2W_CTRL_PDIV_MASK | A2W_CTRL_PWRDN))
            | ndiv
            | (1 << A2W_CTRL_PDIV_SHIFT);
        self.cm.write(r.ctrl, ctrl);
        self.cm.write(r.cm, self.cm.read(r.cm) & !PLL_ANARST);
        self.wait_lock()?;
        self.cm.write(r.ctrl, self.cm.read(r.ctrl) | A2W_CTRL_PRST_DISABLE);
        Ok(())
    }

    /// Retune a running PLL (NDIV + 20-bit fraction), keeping PDIV and the
    /// analogue settings, and wait for lock.
    pub fn retune(&self, ndiv: u32, frac: u32) -> Result<()> {
        if !(1..=A2W_CTRL_NDIV_MASK).contains(&ndiv) || frac > A2W_FRAC_MASK {
            return Err(HalError::Config(format!("{}: invalid NDIV {ndiv} / FRAC {frac:#x}", self.id.name())));
        }
        let r = self.id.regs();
        self.cm.write(r.frac, frac);
        let ctrl = (self.cm.read(r.ctrl) & !(A2W_CTRL_NDIV_MASK | A2W_CTRL_PDIV_MASK)) | ndiv | (1 << A2W_CTRL_PDIV_SHIFT);
        self.cm.write(r.ctrl, ctrl);
        self.wait_lock()?;
        let (rc, rf) = (self.cm.read(r.ctrl) & A2W_CTRL_NDIV_MASK, self.cm.read(r.frac) & A2W_FRAC_MASK);
        if rc != ndiv || rf != frac {
            return Err(HalError::Hardware(format!("{}: read-back NDIV {rc} FRAC {rf:#x}", self.id.name())));
        }
        debug!("{}: VCO {:.4} MHz (NDIV {ndiv} + {frac:#07x}/2^20), locked", self.id.name(), self.vco_hz() / 1e6);
        Ok(())
    }

    pub fn wait_lock(&self) -> Result<()> {
        wait_until(TIMEOUT, &format!("{} lock", self.id.name()), || self.is_locked()).map(|_| ())
    }

    /// Integral gain of the loop filter (ANA1 KI, 0-7).
    pub fn loop_ki(&self) -> u32 {
        (self.cm.read(self.id.regs().ana0 + 4) & ANA1_KI_MASK) >> ANA1_KI_SHIFT
    }

    /// Set the integral gain of the loop filter of a running PLL and wait
    /// for lock. The four ANA registers take effect together when ANA0 is
    /// written (reading one before that returns the old value), so all of
    /// them are written, ANA3 first, as Linux does.
    pub fn set_loop_ki(&self, ki: u32) -> Result<()> {
        if ki > 7 {
            return Err(HalError::Config(format!("{}: loop gain KI {ki} out of range 0-7", self.id.name())));
        }
        let r = self.id.regs();
        let mut ana = [0, 1, 2, 3].map(|i| self.cm.read(r.ana0 + 4 * i));
        ana[1] = (ana[1] & !ANA1_KI_MASK) | (ki << ANA1_KI_SHIFT);
        for i in (0..4).rev() {
            self.cm.write(r.ana0 + 4 * i, ana[i]);
        }
        self.wait_lock()?;
        if self.loop_ki() != ki {
            return Err(HalError::Hardware(format!("{}: loop gain KI read back {} instead of {ki}", self.id.name(), self.loop_ki())));
        }
        debug!("{}: loop gain KI {ki}, locked", self.id.name());
        Ok(())
    }

    /// BCM2711 (Raspberry Pi 4) rather than BCM2835/6/7.
    pub fn is_bcm2711(&self) -> bool {
        !self.has_prediv
    }

    /// Program and enable the PER channel divider (Linux `bcm2835_pll_divider_*`).
    pub fn set_per(&self, div: u32) -> Result<()> {
        if !(1..=256).contains(&div) {
            return Err(HalError::Config(format!("PLL channel divider {div} out of range")));
        }
        let r = self.id.regs();
        self.cm.write(r.per, div & A2W_CHANNEL_DIV_MASK);
        let c = self.cm.read(r.cm);
        self.cm.write(r.cm, c | PLL_LOADPER);
        self.cm.write(r.cm, (c & !PLL_LOADPER) & !PLL_HOLDPER);
        if self.per_divider() != Some(div) {
            return Err(HalError::Hardware(format!("{}_PER divider read-back mismatch", self.id.name())));
        }
        Ok(())
    }

    /// Register offset of the fractional divider (the DMA target).
    pub fn frac_offset(&self) -> usize {
        self.id.regs().frac
    }

    /// The configuration that keeps the PER output running, except the
    /// fraction (which the DMA engine rewrites continuously).
    pub fn config(&self) -> PllConfig {
        let r = self.id.regs();
        PllConfig {
            cm: self.cm.read(r.cm),
            ctrl: self.cm.read(r.ctrl),
            ana: [0, 1, 2, 3].map(|i| self.cm.read(r.ana0 + 4 * i)),
            per: self.cm.read(r.per),
            channels: self.channel_dividers(),
        }
    }

    fn channel_dividers(&self) -> [u32; 3] {
        [0, 1, 2].map(|i| self.cm.read(self.id.other_channels()[i].1))
    }

    /// Put the other channels' dividers back where they differ from
    /// `saved` (a bus collision can overwrite them, see `crate::cpufreq`).
    fn restore_channels(&self, saved: &[u32; 3]) {
        let cm = self.id.regs().cm;
        for (&(name, reg, _, load), &want) in self.id.other_channels().iter().zip(saved) {
            if self.cm.read(reg) != want {
                warn!("{name} was changed while transmitting; restoring it");
                self.cm.write(reg, want);
                let c = self.cm.read(cm);
                self.cm.write(cm, c | load);
                self.cm.write(cm, c & !load);
            }
        }
    }

    /// Whether the PLL still runs as configured in `good` (and is locked).
    /// Only the fields that determine the PER output are compared, so
    /// changes the firmware makes to other channels are left alone.
    pub fn matches(&self, good: &PllConfig) -> bool {
        let now = self.config();
        let ctrl_mask = A2W_CTRL_NDIV_MASK | A2W_CTRL_PDIV_MASK | A2W_CTRL_PWRDN;
        let per_mask = A2W_CHANNEL_DIV_MASK | A2W_CHANNEL_DISABLE;
        let cm_mask = PLL_ANARST | PLL_HOLDPER;
        self.is_locked()
            && now.ctrl & ctrl_mask == good.ctrl & ctrl_mask
            && now.per & per_mask == good.per & per_mask
            && now.cm & cm_mask == good.cm & cm_mask
            && now.ana == good.ana
    }

    /// Bring the PLL back to `good` after someone else changed it (the
    /// fraction is left to the DMA engine): analogue settings, control
    /// (reset and relock if it does not lock by itself), then the PER divider.
    pub fn repair(&self, good: &PllConfig) -> Result<()> {
        let r = self.id.regs();
        if self.config().ana != good.ana {
            for i in (0..4).rev() {
                self.cm.write(r.ana0 + 4 * i, good.ana[i]);
            }
        }
        self.cm.write(r.ctrl, good.ctrl);
        self.cm.write(r.cm, good.cm & !(PLL_ANARST | PLL_LOADPER));
        if wait_until(Duration::from_millis(5), "PLL relock", || self.is_locked()).is_err() {
            // As Linux bcm2835_pll_on: reset, release, lock, then stop the
            // post-divider from being reset again.
            debug!("{} does not relock; resetting it", self.id.name());
            self.cm.write(r.cm, self.cm.read(r.cm) | PLL_ANARST);
            self.cm.write(r.ctrl, good.ctrl & !A2W_CTRL_PRST_DISABLE);
            self.cm.write(r.cm, self.cm.read(r.cm) & !PLL_ANARST);
            self.wait_lock()?;
            self.cm.write(r.ctrl, good.ctrl);
        }
        self.cm.write(r.per, good.per);
        let c = self.cm.read(r.cm);
        self.cm.write(r.cm, c | PLL_LOADPER);
        self.cm.write(r.cm, c & !PLL_LOADPER);
        self.restore_channels(&good.channels);
        if !self.matches(good) {
            return Err(HalError::Hardware(format!("{}: could not restore its configuration", self.id.name())));
        }
        Ok(())
    }

    /// Restore a snapshot.
    pub fn restore(&self, s: &PllState) -> Result<()> {
        let r = self.id.regs();
        self.cm.write(r.per, s.per);
        let c = self.cm.read(r.cm);
        self.cm.write(r.cm, c | PLL_LOADPER);
        self.cm.write(r.cm, c & !PLL_LOADPER);
        for i in (0..4).rev() {
            self.cm.write(r.ana0 + 4 * i, s.ana[i]);
        }
        self.cm.write(r.frac, s.frac);
        self.cm.write(r.ctrl, s.ctrl);
        self.cm.write(r.cm, s.cm);
        if s.ctrl & A2W_CTRL_PWRDN == 0 && s.cm & PLL_ANARST == 0 {
            self.wait_lock()?;
        }
        self.restore_channels(&s.channels);
        self.cm.write(A2W_XOSC_CTRL, s.xosc);
        Ok(())
    }
}
