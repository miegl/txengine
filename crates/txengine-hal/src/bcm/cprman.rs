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

/// Smallest integer divider of a clock generator. Set to 1, GPCLK0 and the
/// PWM clock divide by 4096 instead, from any source and with or without
/// MASH (counted on a Raspberry Pi 2 and a Zero 2 W from 19.2 MHz to
/// 1.125 GHz). Linux never divides these clocks by less than 2 either.
pub const MIN_DIVI: u32 = 2;

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
    /// divider and MASH order (a generator without a MASH control, see
    /// [`has_mash`], only with a whole divider). Glitch free: stops it
    /// first.
    pub fn clock_start(&self, ctl: usize, src: u32, divi: u32, divf: u32, mash: u32) -> Result<()> {
        if !(MIN_DIVI..=0xfff).contains(&divi) || divf > 0xfff || mash > 3 {
            return Err(HalError::Config(format!("invalid clock divider {divi}.{divf} / MASH {mash}")));
        }
        if !has_mash(ctl) && divf != 0 {
            return Err(HalError::Config(format!(
                "clock generator {ctl:#x} divides by whole numbers only (asked {divi}.{divf})"
            )));
        }
        self.clock_stop(ctl)?;
        self.write(div(ctl), (divi << DIVI_SHIFT) | divf);
        let base = (src & SRC_MASK) | (mash << MASH_SHIFT);
        self.write(ctl, base);
        self.write(ctl, base | ENAB);
        wait_until(TIMEOUT, "clock generator start", || self.read(ctl) & BUSY != 0)?;
        let (rc, rd) = (self.read(ctl) & 0x7ff, self.read(div(ctl)) & 0xff_ffff);
        let checked = SRC_MASK | ENAB | if has_mash(ctl) { MASH_MASK } else { 0 };
        if rc & checked != (base | ENAB) & checked || rd != (divi << DIVI_SHIFT) | divf {
            return Err(HalError::Hardware(format!("clock generator {ctl:#x}: read-back ctl {rc:#x} div {rd:#x}")));
        }
        Ok(())
    }

    /// Put back the control and divider registers of the clock generator at
    /// `ctl`, as read before it was used (`saved`: control, divider).
    pub fn restore_clock(&self, ctl: usize, saved: (u32, u32)) {
        let _ = self.clock_stop(ctl);
        self.write(div(ctl), saved.1);
        self.write(ctl, saved.0 & !ENAB);
        if saved.0 & ENAB != 0 {
            self.write(ctl, saved.0);
        }
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
        self.count_start(tcnt_mux, (osc_hz * seconds) as u32);
        std::thread::sleep(Duration::from_secs_f64(seconds));
        let mut n = None;
        wait_until(TIMEOUT, "frequency counter", || {
            n = self.count_result();
            n.is_some()
        })?;
        Ok(n.expect("count finished"))
    }

    /// Start counting a clock's cycles over `osc_cycles` cycles of the
    /// crystal, without waiting ([`Cprman::count_result`] collects it).
    /// The count can be stale when the clock is stopped (see
    /// [`Cprman::measure`]).
    pub fn count_start(&self, tcnt_mux: u32, osc_cycles: u32) {
        self.write(TCNTCTL, KILL);
        self.write(TCNTCTL, (tcnt_mux & SRC_MASK) | ((tcnt_mux >> 4) << TCNT_SRC1_SHIFT));
        self.write(OSCCOUNT, osc_cycles);
    }

    /// The count started by [`Cprman::count_start`], or None while it runs.
    pub fn count_result(&self) -> Option<u32> {
        if self.read(OSCCOUNT) != 0 || self.read(TCNTCTL) & BUSY != 0 {
            return None;
        }
        let n = self.read(TCNTCNT);
        self.write(TCNTCTL, 0);
        Some(n)
    }
}

/// Counts a clock with the frequency counter in the background, one gate
/// per [`ClockWatch::check`], to notice when it runs at the wrong
/// frequency. The gate alternates between 5 and 10 ms: a stopped clock
/// leaves the previous count in the register, which then does not fit the
/// gate.
#[derive(Debug, Clone)]
pub struct ClockWatch {
    tcnt_mux: u32,
    osc_hz: f64,
    expected_hz: f64,
    /// Gate (crystal cycles) of the count in progress.
    gate: Option<u32>,
}

impl ClockWatch {
    /// Watch the clock `tcnt_mux` (see [`Cprman::measure`]), which should
    /// run at `expected_hz`, from a crystal of `osc_hz`.
    pub fn new(tcnt_mux: u32, osc_hz: f64, expected_hz: f64) -> ClockWatch {
        ClockWatch { tcnt_mux, osc_hz, expected_hz, gate: None }
    }

    /// Collect the count in progress if it has finished, and start the
    /// next. Returns the frequency of a finished count that is off by more
    /// than modulation and crystal correction can move a clock's average
    /// (0.5 %; a PLL feedback divider step is at least 0.8 %).
    pub fn check(&mut self, cm: &Cprman) -> Option<f64> {
        let short = (self.osc_hz * 0.005) as u32;
        let mut wrong = None;
        let gate = match self.gate {
            Some(gate) => {
                let n = cm.count_result()?;
                let hz = n as f64 * self.osc_hz / gate as f64;
                if (hz - self.expected_hz).abs() > 0.005 * self.expected_hz {
                    wrong = Some(hz);
                }
                if gate == short { 2 * short } else { short }
            }
            None => short,
        };
        cm.count_start(self.tcnt_mux, gate);
        self.gate = Some(gate);
        wrong
    }
}

/// One of the PLLs a GPCLK can be fed from: PLLA, PLLC and PLLD through
/// their PER channel, PLLH (the HDMI PLL of the BCM2835/6/7, which the
/// BCM2711 does not have) through its AUX channel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PllId {
    A,
    C,
    D,
    H,
}

struct PllRegs {
    cm: usize,
    ctrl: usize,
    frac: usize,
    ana0: usize,
    /// The channel that feeds the clock generators: PER, or PLLH's AUX.
    per: usize,
    lock_bit: u32,
    xosc_enable: u32,
    gpclk_src: u32,
    /// The CM_PLLx bits that load and hold `per` (PLLH's have no hold).
    load_per: u32,
    hold_per: u32,
    /// ANA1's feedback pre-divider bit (BCM2835/6/7).
    prediv: u32,
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
                load_per: PLL_LOADPER,
                hold_per: PLL_HOLDPER,
                prediv: ANA1_FB_PREDIV,
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
                load_per: PLL_LOADPER,
                hold_per: PLL_HOLDPER,
                prediv: ANA1_FB_PREDIV,
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
                load_per: PLL_LOADPER,
                hold_per: PLL_HOLDPER,
                prediv: ANA1_FB_PREDIV,
            },
            // Its reference is enabled with PLLC's.
            PllId::H => PllRegs {
                cm: PLLH,
                ctrl: A2W_PLLH_CTRL,
                frac: A2W_PLLH_FRAC,
                ana0: A2W_PLLH_ANA0,
                per: A2W_PLLH_AUX,
                lock_bit: 1 << 12,
                xosc_enable: 1 << 0,
                gpclk_src: SRC_PLLH_AUX,
                load_per: PLLH_LOADAUX,
                hold_per: 0,
                prediv: ANA1_PLLH_FB_PREDIV,
            },
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            PllId::A => "PLLA",
            PllId::C => "PLLC",
            PllId::D => "PLLD",
            PllId::H => "PLLH",
        }
    }

    /// The name of the channel that feeds the clock generators.
    pub fn per_name(self) -> &'static str {
        match self {
            PllId::A => "PLLA_PER",
            PllId::C => "PLLC_PER",
            PllId::D => "PLLD_PER",
            PllId::H => "PLLH_AUX",
        }
    }

    /// The PLLs of the SoC.
    pub fn all(bcm2711: bool) -> &'static [PllId] {
        if bcm2711 { &[PllId::A, PllId::C, PllId::D] } else { &[PllId::A, PllId::C, PllId::D, PllId::H] }
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
            PllId::H => {
                &[("PLLH_RCAL", A2W_PLLH_RCAL, None, PLLH_LOADRCAL), ("PLLH_PIX", A2W_PLLH_PIX, None, PLLH_LOADPIX)]
            }
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
    /// The other channels' dividers, as listed by `other_channels` (0
    /// where a PLL has fewer).
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

/// Peripheral-mux clocks (sources 4/5/6 are PLLA/C/D PER, 7 PLLH AUX).
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

/// How far down the VCO of a PLL whose only users are the GPU's video and
/// 3D blocks (normally PLLA) may be moved on the BCM2836/7: those blocks
/// then run up to this much slower, and nothing else depends on them. From
/// PLLA's 2250 MHz this reaches every FM carrier in 87.5–108 MHz (at most
/// −12 % is needed; ±3 % reaches 76 % of the band), and with the full
/// −25 % every narrow-band FM channel of CB, the 10, 6, 4, 2 and 1.25 m
/// amateur bands and marine VHF, and of 148–174 MHz all but those within
/// 50 kHz of 153.6, 166.4 and 172.8 MHz (where every division puts the
/// fraction at a whole number); −15 % left 17 marine channels out
/// (156.3–159.1 MHz). Planners
/// prefer the least change, so a VCO this far down only serves carriers
/// that need it. The loop's response and noise measure the same from 1927
/// to 2365 MHz (−14 % to +5 %). Upwards, which would overclock the
/// blocks, the general limit applies.
pub const GPU_PLL_MAX_DECREASE: f64 = 0.25;

/// The PLLs to try for a clock generator at `own_ctl`, best first: an idle
/// PLL is ideal (no other user, or not running). Otherwise, on the
/// BCM2836/7, PLLA, which normally clocks only the GPU's blocks and may
/// move furthest (see [`GPU_PLL_MAX_DECREASE`]), then PLLC, which clocks
/// the VPU, the SD card and the WiFi. On the BCM2711 PLLC (on a Raspberry
/// Pi 4 idle: the firmware parks its channels), then PLLA, which clocks the
/// VPU. PLLH comes last, for the carriers the others cannot make: the DMA
/// engine writes its fraction more slowly (179-186 k writes/s on a Pi 2 and
/// a Zero 2 W, PLLA 228-231 k), which lowers FM's sample rate to 152 kS/s.
pub fn pll_preference(cm: &Cprman, osc_hz: f64, bcm2711: bool, own_ctl: usize) -> Vec<PllId> {
    let idle = [PllId::A, PllId::C, PllId::D].into_iter().find(|&id| {
        let p = Pll::new(cm, id, osc_hz, bcm2711);
        !(p.is_powered() && p.is_locked()) || p.users(own_ctl).is_empty()
    });
    let shared = if bcm2711 { [PllId::C, PllId::A] } else { [PllId::A, PllId::C] };
    let h = (!bcm2711).then_some(PllId::H);
    idle.into_iter().chain(shared).chain(h).collect()
}

/// Fastest input planned for a BCM2835/6/7 clock generator (the firmware
/// runs them at up to 1 GHz). Fed from a PLL undivided, GPCLK0 and the PWM
/// clock counted exact up to 1.8 GHz on a Zero 2 W, and on a Raspberry Pi 2
/// GPCLK0 up to 1.8 GHz, the PWM clock up to 1.6 GHz (0.2 % fast at
/// 1.8 GHz); from 2.0 GHz both failed on both boards.
pub const MAX_PER_HZ: f64 = 1.5e9;

/// Fastest input planned for the BCM2711's PWM and PCM clocks: on a
/// Raspberry Pi 4 they counted exact up to 1.32 GHz, the most measured.
const BCM2711_MAX_INPUT_HZ: f64 = 1.25e9;

/// Fastest input planned for a BCM2711 GPCLK. Fed faster, its output goes
/// wrong, at even divisors first: on a Raspberry Pi 4 GPCLK0 counted exact
/// up to 662.5 MHz, at 675 MHz exact or 0.015 % slow, from 687.5 MHz
/// 0.06 % slow, at 725 MHz 0.5 %; from 750 MHz the divisor 4 gave nothing,
/// from 864 MHz every divisor beyond 3 about 0.22 MHz. (The PWM clock was
/// exact from 1320 MHz, the PCM clock from 750 MHz.)
const BCM2711_MAX_GPCLK_INPUT_HZ: f64 = 600e6;

/// Fastest input planned for a GPCLK.
pub fn max_gpclk_input_hz(bcm2711: bool) -> f64 {
    if bcm2711 { BCM2711_MAX_GPCLK_INPUT_HZ } else { MAX_PER_HZ }
}

/// The VCO frequencies txengine plans a BCM2835/6/7 PLL for that nothing
/// else uses (Hz), with its feedback pre-divider engaged. Retuned step by
/// step and counted, PLLA locked from 0.4 GHz up to 2.8 GHz on a Raspberry
/// Pi 2 (3.0 GHz failed) and to 3.0 GHz on a Zero 2 W (3.2 GHz failed),
/// PLLH from 0.4 to 3.0 GHz on both without the pre-divider and from 0.8
/// to 3.0 GHz with it; the pre-divider made no difference to PLLA. A tone
/// modulated the Zero 2 W's PLLA equally cleanly from 0.73 to 2.91 GHz.
/// (Linux keeps them within 0.6-2.4 GHz and engages the pre-divider above
/// 1.75 GHz.) The pre-divider doubles the VCO per unit of the feedback
/// divider, which halves the fraction the modulation sweeps: without it,
/// from PLLH 25 FM carriers could not be modulated at ±100 kHz, with it
/// every one PLLA reaches.
pub const BCM283X_VCO_RANGE: (f64, f64) = (0.8e9, 2.6e9);

/// The same for a running BCM2711 PLL, by its ANA1 VCO-range bits 13-14,
/// which txengine leaves as the firmware set them. On a Raspberry Pi 4's
/// PLLC: with bit 14 (the firmware's setting of PLLA, PLLB and PLLC) it
/// locked and counted exactly from 1.08 GHz to beyond 6.4 GHz, though a
/// jump from 2.59 to 1.16 GHz did not lock; with neither (PLLD's) from 1.25
/// to 3.8 GHz; with bit 13 alone from 1.05 to 2.2 GHz. A tone modulated
/// PLLC as cleanly from 1.46 to 4.66 GHz as at 2.62 GHz, within the
/// spread of the reception. With the GPCLK input limited to 600 MHz, PLLC
/// reaches every FM carrier but those within 140 kHz of 108 MHz (twice
/// the crystal), and every narrow-band channel of CB, the 10, 6, 4, 2 and
/// 1.25 m bands and 148–174 MHz but those within 50 kHz of 54 and
/// 162 MHz.
pub(crate) fn bcm2711_vco_range(ana1: u32) -> (f64, f64) {
    if ana1 & ANA1_BCM2711_RANGE_WIDE != 0 {
        (1.4e9, 4.0e9)
    } else if ana1 & ANA1_BCM2711_RANGE_LOW != 0 {
        (1.2e9, 2.0e9)
    } else {
        (1.4e9, 3.6e9)
    }
}

/// What a frequency planner may do with a PLL, given who else uses it: the
/// VCO frequencies and PER dividers it may choose.
#[derive(Debug, Clone)]
pub struct PllRoom {
    /// Powered up and locked when planning.
    pub running: bool,
    /// Its other users (none if it is not running).
    pub users: PllUsers,
    /// The VCO frequency now (0 if not running).
    pub vco_old: f64,
    /// The VCO per unit of NDIV, in crystal periods: 2 with the feedback
    /// pre-divider engaged (which is never changed on a running PLL).
    pub fb: f64,
    /// The PER dividers allowed: only the current one if another clock
    /// uses the PER channel.
    pub per_choices: Vec<u32>,
    /// The PER divider now, if the channel is enabled.
    pub per_now: Option<u32>,
    /// Allowed VCO range.
    pub vco_min: f64,
    pub vco_max: f64,
    /// Largest relative decrease and increase of a shared VCO (for messages).
    pub max_decrease: f64,
    pub max_increase: f64,
    /// Fastest PER channel output the planned clock generator takes.
    pub max_per_hz: f64,
    /// Why a planner cannot use the PLL at all, if it cannot.
    pub unusable: Option<&'static str>,
    /// The planner engages the feedback pre-divider (`fb` 2): a BCM2835/6/7
    /// PLL that nothing else uses (see [`BCM283X_VCO_RANGE`]).
    pub engage_prediv: bool,
}

impl PllRoom {
    /// The room on `pll` for a planner that drives the clock generator at
    /// `own_ctl` (not counted as a user), moving a shared VCO by at most
    /// `max_change` (relative; see [`GPU_PLL_MAX_DECREASE`] for a PLL that
    /// only clocks the GPU).
    pub fn of(pll: &Pll, own_ctl: usize, max_change: f64) -> PllRoom {
        let max_per_hz = if (0..3).any(|n| gp(n) == own_ctl) {
            max_gpclk_input_hz(pll.is_bcm2711())
        } else if pll.is_bcm2711() {
            BCM2711_MAX_INPUT_HZ
        } else {
            MAX_PER_HZ
        };
        if pll.is_bcm2711() && pll.id == PllId::H {
            return PllRoom::unusable("does not exist on the BCM2711", max_per_hz);
        }
        let running = pll.is_powered() && pll.is_locked();
        if pll.is_bcm2711() && !running {
            return PllRoom::unusable("is not running, and txengine starts PLLs on the BCM2835/6/7 only", max_per_hz);
        }
        let users = if running { pll.users(own_ctl) } else { PllUsers::default() };
        if pll.id == PllId::H && !users.other.is_empty() {
            return PllRoom::unusable("clocks the HDMI output", max_per_hz);
        }
        let max_decrease = if users.gpu_only { max_change.max(GPU_PLL_MAX_DECREASE) } else { max_change };
        let engage_prediv = !pll.is_bcm2711() && users.is_empty();
        let fb = if engage_prediv { 2.0 } else { pll.fb_mult() };
        let vco_old = if running { pll.vco_hz() } else { 0.0 };
        let per_now = pll.per_divider();
        // A PER channel used by another clock must keep its divider.
        let per_choices = match (per_now, users.per.is_empty()) {
            (Some(d), false) => vec![d],
            _ => (1..=8).collect(),
        };
        // VCO limits: a PLL that others use moves only a little from the
        // frequency it runs at, with its pre-divider left as it is; an idle
        // one anywhere the hardware allows.
        let (vco_min, vco_max) = if running && !users.is_empty() {
            (vco_old * (1.0 - max_decrease), vco_old * (1.0 + max_change))
        } else if pll.is_bcm2711() {
            bcm2711_vco_range(pll.ana1())
        } else {
            BCM283X_VCO_RANGE
        };
        PllRoom {
            running,
            users,
            vco_old,
            fb,
            per_choices,
            per_now,
            vco_min,
            vco_max,
            max_decrease,
            max_increase: max_change,
            max_per_hz,
            unusable: None,
            engage_prediv,
        }
    }

    /// A PLL no planner may use, for the reason `why`.
    fn unusable(why: &'static str, max_per_hz: f64) -> PllRoom {
        PllRoom {
            running: false,
            users: PllUsers::default(),
            vco_old: 0.0,
            fb: 1.0,
            per_choices: Vec::new(),
            per_now: None,
            vco_min: 0.0,
            vco_max: 0.0,
            max_decrease: 0.0,
            max_increase: 0.0,
            max_per_hz,
            unusable: Some(why),
            engage_prediv: false,
        }
    }

    /// A planner's preference for `vco` on a PLL that nothing else uses: the
    /// middle of its range, away from the edges. (Near the bottom it would
    /// divide little: PLLH kept near its 864 MHz, divided by 10 for
    /// 87.6 MHz, put the phase noise 1 MHz out 10 dB above PLLA's, divided
    /// by 26, on a Zero 2 W.)
    pub fn idle_score(&self, vco: f64) -> f64 {
        if self.users.is_empty() { (vco - (self.vco_min + self.vco_max) / 2.0).abs() / 1e9 } else { 0.0 }
    }

    /// Whether the VCO may run at `vco` with the PER divider `per` (one of
    /// [`PllRoom::per_choices`]).
    pub fn allows(&self, vco: f64, per: u32) -> bool {
        (self.vco_min..=self.vco_max).contains(&vco) && vco / per as f64 <= self.max_per_hz
    }

    /// The relative change of the VCO's frequency to `vco` for the PLL's
    /// other users (0 if it has none).
    pub fn change(&self, vco: f64) -> f64 {
        if self.users.is_empty() { 0.0 } else { (vco - self.vco_old).abs() / self.vco_old }
    }

    /// The error for a planner that found nothing in this room.
    pub fn exhausted(&self, id: PllId, what: &str) -> HalError {
        if let Some(why) = self.unusable {
            return HalError::Config(format!("{} {why}; choose another --pll", id.name()));
        }
        if let [per] = self.per_choices[..]
            && self.vco_min / per as f64 > self.max_per_hz
        {
            return HalError::Config(format!(
                "{}, which also drives {}, runs at {:.0} MHz (/{per}), faster than the clock generator takes \
                 ({:.0} MHz); choose another --pll",
                id.per_name(),
                self.users.per.join(", "),
                self.vco_old / per as f64 / 1e6,
                self.max_per_hz / 1e6
            ));
        }
        HalError::Config(format!(
            "{} cannot produce {what} with its VCO within -{:.0} % / +{:.0} % of {:.1} MHz \
             (it also drives: {}); choose another --pll",
            id.name(),
            self.max_decrease * 100.0,
            self.max_increase * 100.0,
            self.vco_old / 1e6,
            self.users.names().join(", ")
        ))
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
        if self.has_prediv && self.ana1() & self.id.regs().prediv != 0 { 2.0 } else { 1.0 }
    }

    fn ana1(&self) -> u32 {
        self.cm.read(self.id.regs().ana0 + 4)
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
    /// (excluding `own_gpclk_ctl`), its CORE channels that clocks select,
    /// and its other enabled channels, which clock a display or camera
    /// directly (DSI, CCP2, HDMI), unless the firmware parked them at the
    /// largest divider (256): on a Raspberry Pi 4, PLLA's DSI0 and CCP2
    /// channels at 12.7 MHz, on the Pi 2 and Zero 2 W without a display
    /// PLLH's HDMI channels at 3.4 MHz.
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
            let v = self.cm.read(reg);
            if v & A2W_CHANNEL_DISABLE != 0 {
                continue;
            }
            if let Some(s) = vpu_src {
                let users: Vec<&str> = VPU_CLOCKS
                    .iter()
                    .filter(|&&(_, ctl)| {
                        let c = self.cm.read(ctl);
                        // The VPU clock has no ENAB bit and is always running.
                        (c & (ENAB | BUSY) != 0 || ctl == VPU) && c & SRC_MASK == s
                    })
                    .map(|&(name, _)| name)
                    .collect();
                if users.is_empty() {
                    continue;
                }
                gpu_only &= users.iter().all(|u| GPU_CLOCKS.contains(u));
                other.push(format!("{cname} ({})", users.join(", ")));
            } else if v & A2W_CHANNEL_DIV_MASK != 0 {
                gpu_only = false;
                other.push(cname.to_string());
            }
        }
        gpu_only &= per_users.is_empty();
        PllUsers { per: per_users, other, gpu_only }
    }

    /// Bring a powered-down PLL up (Linux `bcm2835_pll_on`), with its
    /// feedback pre-divider engaged (see [`BCM283X_VCO_RANGE`]).
    pub fn power_up(&self, ndiv: u32, frac: u32) -> Result<()> {
        let r = self.id.regs();
        self.cm.write(A2W_XOSC_CTRL, self.cm.read(A2W_XOSC_CTRL) | r.xosc_enable);
        // Linux's loop filter settings (KI 2, KP 8, KA 2; PLLH: KI 2, KP 6,
        // KA 2).
        let mut ana = [0, 1, 2, 3].map(|i| self.cm.read(r.ana0 + 4 * i));
        if self.id == PllId::H {
            ana[0] = (ana[0] & !(PLLH_ANA0_KA_MASK | PLLH_ANA0_KI_LOW_MASK))
                | (2 << PLLH_ANA0_KA_SHIFT)
                | (2 << PLLH_ANA0_KI_LOW_SHIFT);
            ana[1] = (ana[1] & !(PLLH_ANA1_KI_HIGH_MASK | PLLH_ANA1_KP_MASK)) | (6 << PLLH_ANA1_KP_SHIFT);
        } else {
            ana[1] = (ana[1] & !(ANA1_KI_MASK | ANA1_KP_MASK)) | (2 << ANA1_KI_SHIFT) | (8 << ANA1_KP_SHIFT);
            ana[3] = (ana[3] & !ANA3_KA_MASK) | (2 << ANA3_KA_SHIFT);
        }
        if self.has_prediv {
            ana[1] |= r.prediv;
        }
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

    /// Engage the feedback pre-divider of a running BCM2835/6/7 PLL (which
    /// doubles its VCO until it is retuned) and wait for lock.
    pub fn engage_prediv(&self) -> Result<()> {
        let r = self.id.regs();
        if !self.has_prediv || self.ana1() & r.prediv != 0 {
            return Ok(());
        }
        let mut ana = [0, 1, 2, 3].map(|i| self.cm.read(r.ana0 + 4 * i));
        ana[1] |= r.prediv;
        for i in (0..4).rev() {
            self.cm.write(r.ana0 + 4 * i, ana[i]);
        }
        self.wait_lock()
    }

    /// Retune a running PLL (NDIV + 20-bit fraction), keeping PDIV and the
    /// analogue settings, and wait for lock.
    pub fn retune(&self, ndiv: u32, frac: u32) -> Result<()> {
        if !(1..=A2W_CTRL_NDIV_MASK).contains(&ndiv) || frac > A2W_FRAC_MASK {
            return Err(HalError::Config(format!("{}: invalid NDIV {ndiv} / FRAC {frac:#x}", self.id.name())));
        }
        let r = self.id.regs();
        self.cm.write(r.frac, frac);
        let ctrl =
            (self.cm.read(r.ctrl) & !(A2W_CTRL_NDIV_MASK | A2W_CTRL_PDIV_MASK)) | ndiv | (1 << A2W_CTRL_PDIV_SHIFT);
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

    /// Integral gain of the loop filter (0-7: ANA1 KI; PLLH's low two bits
    /// in ANA0, its high bit in ANA1).
    pub fn loop_ki(&self) -> u32 {
        let ana0 = self.id.regs().ana0;
        if self.id == PllId::H {
            ((self.cm.read(ana0) & PLLH_ANA0_KI_LOW_MASK) >> PLLH_ANA0_KI_LOW_SHIFT)
                | ((self.cm.read(ana0 + 4) & PLLH_ANA1_KI_HIGH_MASK) << 2)
        } else {
            (self.cm.read(ana0 + 4) & ANA1_KI_MASK) >> ANA1_KI_SHIFT
        }
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
        if self.id == PllId::H {
            ana[0] = (ana[0] & !PLLH_ANA0_KI_LOW_MASK) | ((ki & 3) << PLLH_ANA0_KI_LOW_SHIFT);
            ana[1] = (ana[1] & !PLLH_ANA1_KI_HIGH_MASK) | (ki >> 2);
        } else {
            ana[1] = (ana[1] & !ANA1_KI_MASK) | (ki << ANA1_KI_SHIFT);
        }
        for i in (0..4).rev() {
            self.cm.write(r.ana0 + 4 * i, ana[i]);
        }
        self.wait_lock()?;
        if self.loop_ki() != ki {
            return Err(HalError::Hardware(format!(
                "{}: loop gain KI read back {} instead of {ki}",
                self.id.name(),
                self.loop_ki()
            )));
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
        self.cm.write(r.cm, c | r.load_per);
        self.cm.write(r.cm, (c & !r.load_per) & !r.hold_per);
        if self.per_divider() != Some(div) {
            return Err(HalError::Hardware(format!("{} divider read-back mismatch", self.id.per_name())));
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
        [0, 1, 2].map(|i| self.id.other_channels().get(i).map_or(0, |c| self.cm.read(c.1)))
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
        let cm_mask = PLL_ANARST | self.id.regs().hold_per;
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
        self.cm.write(r.cm, good.cm & !(PLL_ANARST | r.load_per));
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
        self.cm.write(r.cm, c | r.load_per);
        self.cm.write(r.cm, c & !r.load_per);
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
        self.cm.write(r.cm, c | r.load_per);
        self.cm.write(r.cm, c & !r.load_per);
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
