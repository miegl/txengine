//! The raised system clock of the PIO method and AM: while they transmit,
//! the system clock runs from `PLL_USB` at 240-264 MHz (chosen by the
//! host), the core voltage is raised to 1.20 V and the flash interface
//! slowed down to keep its clock where boot put it: on the RP2350 on the
//! fly, on the RP2040 with core 0 parked in RAM, as its SSI cannot change
//! while code runs from flash. USB keeps running from `PLL_SYS`.

use embassy_rp::pac;
use txengine_pico_protocol::{Fault, SysClock};

use crate::rf::{self, XOSC_HZ};

/// System clocks accepted: high enough to keep the FM band clear of the
/// PIO method's carrier image (`B - f`), low enough that the RP2040's flash
/// (clocked at half the system clock) stays within 133 MHz.
pub const MIN_HZ: u32 = 240_000_000;
pub const MAX_HZ: u32 = 264_000_000;

/// Core voltage while transmitting (1.20 V; the same encoding on both chips).
const VSEL_1V20: u8 = 0b1101;

/// Breadcrumbs of the clock switch (see `crate::crumb`).
const STEP: u32 = 0x434b_0000;

#[inline(always)]
fn crumb(step: u32) {
    crate::crumb(step);
}

/// The clock of a setting.
pub fn hz(clock: &SysClock) -> u32 {
    clock.hz(XOSC_HZ)
}

pub fn validate(clock: &SysClock) -> bool {
    let fref = XOSC_HZ / clock.refdiv.max(1) as u32;
    let vco = fref as u64 * clock.fbdiv as u64;
    (1..=63).contains(&clock.refdiv)
        && fref >= 5_000_000
        && (16..=320).contains(&clock.fbdiv)
        && (750_000_000..=1_600_000_000).contains(&vco)
        && (1..=7).contains(&clock.post_div1)
        && (1..=7).contains(&clock.post_div2)
        && (MIN_HZ..=MAX_HZ).contains(&hz(clock))
}

/// What [`raise`] changed outside the transmitter's own hardware.
pub struct Saved {
    vsel: u8,
    /// The flash interface's clock divider.
    flash_div: u16,
}

/// Raise the core voltage, start `PLL_USB` at `clock`, slow the flash
/// interface down and run the system clock from `PLL_USB`.
pub fn raise(clock: &SysClock) -> Result<Saved, Fault> {
    let saved = Saved { vsel: set_vsel(VSEL_1V20), flash_div: flash_div() };
    crumb(STEP | 3);
    busy_wait_us(2_000);
    crumb(STEP | 4);
    rf::reset_pll_usb();
    let p = pac::PLL_USB;
    p.pwr().write(|w| {
        w.set_pd(true);
        w.set_vcopd(true);
        w.set_postdivpd(true);
        w.set_dsmpd(true);
    });
    p.cs().write(|w| w.set_refdiv(clock.refdiv));
    p.fbdiv_int().write(|w| w.set_fbdiv_int(clock.fbdiv));
    p.pwr().write(|w| {
        w.set_postdivpd(true);
        w.set_dsmpd(true);
    });
    let t0 = embassy_time::Instant::now();
    while !p.cs().read().lock() {
        if t0.elapsed() > embassy_time::Duration::from_millis(10) {
            restore(&saved);
            return Err(Fault::PllLock);
        }
    }
    p.prim().write(|w| {
        w.set_postdiv1(clock.post_div1);
        w.set_postdiv2(clock.post_div2);
    });
    p.pwr().modify(|w| w.set_postdivpd(false));
    crumb(STEP | 5);
    // Keep the flash's clock where boot put it: scale its divider with the
    // system clock (an even divider on the RP2040).
    let div = (saved.flash_div as u32 * hz(clock)).div_ceil(rf::SYS_HZ);
    let div = if cfg!(feature = "rp2040") { div.next_multiple_of(2) } else { div };
    crumb(STEP | 6 | (div << 8));
    switch_clock(pac::clocks::vals::ClkSysCtrlAuxsrc::CLKSRC_PLL_USB, div.min(254) as u16, true);
    crumb(STEP | 7);
    Ok(saved)
}

/// Undo [`raise`]: the system clock back on `PLL_SYS` with the flash's
/// divider, the voltage, and `PLL_USB` off.
pub fn restore(saved: &Saved) {
    switch_clock(pac::clocks::vals::ClkSysCtrlAuxsrc::CLKSRC_PLL_SYS, saved.flash_div, false);
    set_vsel(saved.vsel);
    pac::PLL_USB.pwr().write(|w| {
        w.set_pd(true);
        w.set_vcopd(true);
        w.set_postdivpd(true);
        w.set_dsmpd(true);
    });
}

/// Take the system clock off `PLL_USB` if it runs from it (the raised
/// clock), before the PLL is powered down other than by [`restore`]: else
/// both cores stop with it, and no watchdog reset follows. The flash
/// divider and the voltage stay raised, which the slower clock tolerates.
pub fn leave_pll_usb() {
    use pac::clocks::vals::{ClkSysCtrlAuxsrc, ClkSysCtrlSrc};
    let c = pac::CLOCKS.clk_sys_ctrl().read();
    if c.src() == ClkSysCtrlSrc::CLKSRC_CLK_SYS_AUX && c.auxsrc() == ClkSysCtrlAuxsrc::CLKSRC_PLL_USB {
        clk_sys_from(ClkSysCtrlAuxsrc::CLKSRC_PLL_SYS);
    }
}

/// The flash interface's clock divider.
fn flash_div() -> u16 {
    #[cfg(feature = "rp2040")]
    {
        pac::SSI.baudr().read().sckdv()
    }
    #[cfg(feature = "rp2350")]
    {
        pac::QMI.mem(0).timing().read().clkdiv() as u16
    }
}

/// Switch the system clock to `src` and the flash divider to `div`: the
/// divider first when the clock rises, after it when it falls. On the
/// RP2350 the divider changes on the fly; the RP2040's SSI must be disabled
/// to change it, so nothing may run from flash meanwhile: core 1 parks in
/// RAM and core 0 makes the change, also from RAM ([`service_switch`]).
fn switch_clock(src: pac::clocks::vals::ClkSysCtrlAuxsrc, div: u16, rising: bool) {
    #[cfg(feature = "rp2040")]
    {
        use core::sync::atomic::Ordering;
        SWITCH_SRC.store(src as u32, Ordering::Relaxed);
        SWITCH_DIV.store(div as u32, Ordering::Relaxed);
        SWITCH_RISING.store(rising as u32, Ordering::Relaxed);
        crumb(STEP | 0x30);
        wait_for_switch();
        crumb(STEP | 0x34);
    }
    #[cfg(feature = "rp2350")]
    {
        let set_div = || {
            pac::QMI.mem(0).timing().modify(|w| w.set_clkdiv(div as u8));
            // A read of mapped flash, so that the new divider is in effect
            // before the system clock changes.
            // SAFETY: flash is mapped there.
            unsafe { core::ptr::read_volatile(0x1000_0000 as *const u32) };
            cortex_m::asm::dsb();
            cortex_m::asm::isb();
        };
        if rising {
            set_div();
            clk_sys_from(src);
        } else {
            clk_sys_from(src);
            set_div();
        }
    }
}

/// The switch core 1 asks core 0 for: 1 requested, 2 done.
#[cfg(feature = "rp2040")]
static SWITCH: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
#[cfg(feature = "rp2040")]
static SWITCH_SRC: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
#[cfg(feature = "rp2040")]
static SWITCH_DIV: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
#[cfg(feature = "rp2040")]
static SWITCH_RISING: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// Core 1: request the switch and wait for it in RAM.
#[cfg(feature = "rp2040")]
#[unsafe(link_section = ".data.wait_for_switch")]
#[inline(never)]
fn wait_for_switch() {
    use core::sync::atomic::Ordering;
    SWITCH.store(1, Ordering::Release);
    cortex_m::asm::sev();
    while SWITCH.load(Ordering::Acquire) != 2 {}
    SWITCH.store(0, Ordering::Release);
}

/// Core 0: make a switch core 1 asked for, from RAM with interrupts off.
/// Called while core 0 waits for the engine and every few milliseconds.
#[cfg(feature = "rp2040")]
#[unsafe(link_section = ".data.service_switch")]
#[inline(never)]
pub fn service_switch() {
    use core::sync::atomic::Ordering;
    if SWITCH.load(Ordering::Acquire) != 1 {
        return;
    }
    cortex_m::interrupt::disable();
    let src = pac::clocks::vals::ClkSysCtrlAuxsrc::from_bits(SWITCH_SRC.load(Ordering::Relaxed) as u8);
    let div = SWITCH_DIV.load(Ordering::Relaxed) as u16;
    let set_div = || {
        let ssi = pac::SSI;
        while ssi.sr().read().busy() {}
        ssi.ssienr().write(|w| w.set_ssi_en(false));
        ssi.baudr().write(|w| w.set_sckdv(div));
        ssi.ssienr().write(|w| w.set_ssi_en(true));
    };
    if SWITCH_RISING.load(Ordering::Relaxed) != 0 {
        set_div();
        clk_sys_from(src);
    } else {
        clk_sys_from(src);
        set_div();
    }
    SWITCH.store(2, Ordering::Release);
    // SAFETY: interrupts were on (this runs in thread mode).
    unsafe { cortex_m::interrupt::enable() };
}

/// The RP2350 changes its flash divider on the fly: nothing to do.
#[cfg(feature = "rp2350")]
pub fn service_switch() {}

/// Switch the system clock's auxiliary source through the glitchless
/// reference clock.
#[inline(always)]
fn clk_sys_from(src: pac::clocks::vals::ClkSysCtrlAuxsrc) {
    use pac::clocks::vals::ClkSysCtrlSrc;
    let c = pac::CLOCKS;
    c.clk_sys_ctrl().modify(|w| w.set_src(ClkSysCtrlSrc::CLK_REF));
    while selected(&c) & 1 == 0 {}
    c.clk_sys_ctrl().modify(|w| w.set_auxsrc(src));
    c.clk_sys_ctrl().modify(|w| w.set_src(ClkSysCtrlSrc::CLKSRC_CLK_SYS_AUX));
    while selected(&c) & 2 == 0 {}
}

/// `CLK_SYS_SELECTED` (one-hot: bit 0 the reference, bit 1 the auxiliary
/// source).
#[inline(always)]
fn selected(c: &pac::clocks::Clocks) -> u32 {
    #[cfg(feature = "rp2040")]
    {
        c.clk_sys_selected().read()
    }
    #[cfg(feature = "rp2350")]
    {
        c.clk_sys_selected().read().0
    }
}

/// Set the core voltage; returns the previous setting.
fn set_vsel(v: u8) -> u8 {
    #[cfg(feature = "rp2040")]
    {
        let r = pac::VREG_AND_CHIP_RESET.vreg();
        let old = r.read().vsel();
        r.modify(|w| w.set_vsel(v));
        old
    }
    #[cfg(feature = "rp2350")]
    {
        // POWMAN registers take a password in the top half. The regulator
        // ignores VSEL (which still reads back as written) until software
        // control is unlocked, which then stays so until a reset.
        const PASSWORD: u32 = 0x5afe << 16;
        pac::POWMAN.vreg_ctrl().modify(|w| {
            w.0 = (w.0 & 0xffff) | PASSWORD;
            w.set_unlock(true);
        });
        let r = pac::POWMAN.vreg();
        // Writes are ignored while an update is in progress.
        while r.read().update_in_progress() {}
        let old = r.read().vsel();
        r.modify(|w| {
            w.0 = (w.0 & 0xffff) | PASSWORD;
            w.set_vsel(v);
        });
        while r.read().update_in_progress() {}
        old
    }
}

fn busy_wait_us(us: u64) {
    let t0 = embassy_time::Instant::now();
    while t0.elapsed() < embassy_time::Duration::from_micros(us) {}
}
