//! txengine RF front end for the Raspberry Pi Pico (RP2040) and Pico 2
//! (RP2350): the host renders the samples, the Pico turns them into a
//! carrier modulated in frequency ([`rf`]: the PLL method, [`direct`]: the
//! PIO method) or in amplitude ([`am`]). See `txengine-pico-protocol`.
//!
//! Core 0 runs the USB device; core 1 runs the RF engine alone ([`rf`]).
//! The system clock (144 MHz) and the USB clock (144 / 3 MHz) both come
//! from `PLL_SYS`, which leaves `PLL_USB` for the carrier, or, raised to
//! 240-264 MHz, for the system clock while the PIO method or AM transmits
//! ([`clock`]).

#![no_std]
#![no_main]

mod am;
mod clock;
mod direct;
mod rf;
mod shared;
mod usb;
mod usb_driver;

use core::fmt::Write as _;
use core::ptr::addr_of_mut;

use embassy_executor::Spawner;
use embassy_rp::clocks::{ClockConfig, PllConfig, UsbClkConfig, UsbClkSrc};
use embassy_rp::multicore::{Stack, spawn_core1};
use embassy_rp::usb::InterruptHandler;
use embassy_rp::{bind_interrupts, peripherals};
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    // The guard first: it acts on what embassy-rp's handler then clears.
    USBCTRL_IRQ => usb_driver::Ep0Guard, InterruptHandler<peripherals::USB>;
});

static mut CORE1_STACK: Stack<8192> = Stack::new();

/// Core 0 feeds the watchdog; a hang reboots the firmware (and stops the
/// carrier).
pub const WATCHDOG_TIMEOUT: embassy_time::Duration = embassy_time::Duration::from_secs(1);
/// Set when the firmware reboots itself (from either core): core 0 stops
/// feeding the watchdog, on which the boot ROM counts the reboot down.
pub static REBOOTING: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);
/// The breadcrumb left in watchdog scratch 0 before the last watchdog reset.
pub static WATCHDOG_NOTE: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// The firmware's version: `git describe` of its source tree (build.rs).
pub const VERSION: &str = env!("TXENGINE_VERSION");
const _: () = assert!(VERSION.len() <= txengine_pico_protocol::VERSION_MAX);

/// What `picotool info` shows about the firmware, also in BOOTSEL mode.
#[unsafe(link_section = ".bi_entries")]
#[used]
static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 3] = {
    use embassy_rp::binary_info::{self as bi, consts};
    [
        bi::rp_cargo_bin_name!(),
        bi::env!(consts::TAG_RASPBERRY_PI, consts::ID_RP_PROGRAM_VERSION_STRING, "TXENGINE_VERSION"),
        bi::rp_program_description!(c"txengine RF front end: an FM and AM transmitter on USB"),
    ]
};

fn clock_config() -> ClockConfig {
    let mut c = ClockConfig::crystal(rf::XOSC_HZ);
    let xosc = c.xosc.as_mut().unwrap();
    // 12 MHz × 120 = 1440 MHz, / 5 / 2 = 144 MHz.
    xosc.sys_pll = Some(PllConfig { refdiv: 1, fbdiv: 120, post_div1: 5, post_div2: 2 });
    // PLL_USB belongs to the transmitter.
    xosc.usb_pll = None;
    c.usb_clk = Some(UsbClkConfig { src: UsbClkSrc::PllSys, div: 3, phase: 0 });
    // Neither the ADC nor the RTC is used: their clocks stay off.
    c.adc_clk = None;
    #[cfg(feature = "rp2040")]
    {
        c.rtc_clk = None;
    }
    c
}

/// A serial number unique to the board, for telling several apart.
fn serial_number(p: &mut embassy_rp::Peripherals) -> &'static str {
    static SERIAL: StaticCell<heapless_str::Str> = StaticCell::new();
    let s = SERIAL.init(heapless_str::Str::new());
    #[cfg(feature = "rp2040")]
    {
        let mut id = [0u8; 8];
        let mut flash = embassy_rp::flash::Flash::<_, embassy_rp::flash::Blocking, { 2 * 1024 * 1024 }>::new_blocking(
            p.FLASH.reborrow(),
        );
        if flash.blocking_unique_id(&mut id).is_ok() {
            for b in id {
                let _ = write!(s, "{b:02X}");
            }
        }
    }
    #[cfg(feature = "rp2350")]
    {
        let _ = &p;
        if let Ok(id) = embassy_rp::otp::get_chipid() {
            let _ = write!(s, "{id:016X}");
        }
    }
    s.as_str()
}

/// A tiny fixed-capacity string.
mod heapless_str {
    pub struct Str {
        buf: [u8; 24],
        len: usize,
    }

    impl Str {
        pub const fn new() -> Str {
            Str { buf: [0; 24], len: 0 }
        }

        pub fn as_str(&self) -> &str {
            core::str::from_utf8(&self.buf[..self.len]).unwrap_or("")
        }
    }

    impl core::fmt::Write for Str {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            let b = s.as_bytes();
            let end = self.len + b.len();
            if end > self.buf.len() {
                return Err(core::fmt::Error);
            }
            self.buf[self.len..end].copy_from_slice(b);
            self.len = end;
            Ok(())
        }
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = embassy_rp::init(embassy_rp::config::Config::new(clock_config()));
    let serial = serial_number(&mut p);
    // The assembly kernels must match the Rust arithmetic.
    assert!(rf::self_test() && direct::self_test() && am::self_test());
    // Reserve what the RF engine programs directly.
    let _rf = (p.PIO0, p.DMA_CH0, p.DMA_CH1, p.DMA_CH2, p.DMA_CH3, p.PIN_21);
    spawn_core1(
        p.CORE1,
        // SAFETY: the stack is handed to core 1 once.
        unsafe { &mut *addr_of_mut!(CORE1_STACK) },
        || rf::run(),
    );
    let mut watchdog = embassy_rp::watchdog::Watchdog::new(p.WATCHDOG);
    if watchdog.reset_reason() == Some(embassy_rp::watchdog::ResetReason::TimedOut) {
        WATCHDOG_NOTE.store(watchdog.get_scratch(0), core::sync::atomic::Ordering::Relaxed);
    }
    watchdog.set_scratch(0, 0);
    watchdog.start(WATCHDOG_TIMEOUT);
    // embassy-rp selects what a timeout resets with the RP2040's bits on the
    // RP2350 too, where they stand for the oscillators, the clocks, the
    // peripherals and part of the RAM, but not the cores: a hang would
    // leave the cores running with USB reset under them, never rebooting.
    // Everything but the oscillators, as the Pico SDK selects.
    #[cfg(feature = "rp2350")]
    {
        let mut sel = embassy_rp::pac::psm::regs::Wdsel(0x01ff_ffff);
        sel.set_rosc(false);
        sel.set_xosc(false);
        embassy_rp::pac::PSM.wdsel().write_value(sel);
    }
    let driver = usb_driver::Driver::new(p.USB, Irqs);
    usb::run(driver, serial, watchdog).await;
}

/// Leave a breadcrumb in watchdog scratch 0, read back after a watchdog
/// reset (`Info::watchdog_note`): a tag in the top half, a step below.
#[inline(always)]
pub fn crumb(code: u32) {
    embassy_rp::pac::WATCHDOG.scratch0().write_value(code);
}

/// Switch the carrier off and reboot into the USB bootloader (from either
/// core).
pub fn reboot_to_bootsel() -> ! {
    REBOOTING.store(true, core::sync::atomic::Ordering::Release);
    rf::output_off(None);
    embassy_rp::rom_data::reset_to_usb_boot(0, 0);
    #[allow(clippy::empty_loop)]
    loop {}
}

/// A panic switches the carrier off and returns to the bootloader, so the
/// board can be reflashed without touching it.
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    reboot_to_bootsel()
}

#[cortex_m_rt::exception]
unsafe fn HardFault(_frame: &cortex_m_rt::ExceptionFrame) -> ! {
    reboot_to_bootsel()
}
