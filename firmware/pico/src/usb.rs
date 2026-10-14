//! The USB device (core 0): the txengine interface and the Pico SDK's
//! reset interface.

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_time::{Duration, Instant, Timer};
use embassy_usb::control::{InResponse, OutResponse, Recipient, Request as ControlRequest, RequestType};
use embassy_usb::driver::{Endpoint as _, EndpointOut};
use embassy_usb::{Builder, Handler};
use static_cell::StaticCell;
use txengine_pico_protocol as proto;
use txengine_pico_protocol::{Config, Fault, Info, PioConfig, Request, Retune, State, Status};

use crate::shared::{Command, Method, SHARED};
use crate::usb_driver::Driver;
use crate::{direct, rf};

/// Set to reboot into the bootloader once the control transfer is done.
static REBOOT_BOOTSEL: AtomicBool = AtomicBool::new(false);
static REBOOT_FLASH: AtomicBool = AtomicBool::new(false);

struct Control {
    txengine_if: u16,
    reset_if: u16,
    buf: [u8; 64],
}

/// Wait (briefly) for the engine to reach a state.
fn wait_state(ok: impl Fn(State) -> bool, ms: u64) -> bool {
    let t0 = Instant::now();
    while !ok(SHARED.state()) {
        // The engine may need core 0 to switch the clocks (PIO method).
        direct::service_switch();
        if t0.elapsed() > Duration::from_millis(ms) {
            return false;
        }
    }
    true
}

/// A new configuration is in place: start over from it.
fn configured(r: Retune) {
    SHARED.retune.set(r);
    SHARED.samples.clear();
    SHARED.received.set(0);
    SHARED.set_fault(Fault::None);
    SHARED.set_state(State::Configured);
}

fn stop_engine() {
    SHARED.send(Command::Stop);
    wait_state(|s| s == State::Idle, 100);
    SHARED.samples.clear();
    SHARED.received.set(0);
}

impl Control {
    fn out(&mut self, req: Request, data: &[u8]) -> bool {
        match req {
            Request::Configure => {
                let Some(cfg) = Config::decode(data) else { return false };
                if !matches!(SHARED.state(), State::Idle | State::Configured) || !rf::validate(&cfg) {
                    return false;
                }
                SHARED.config.set(cfg);
                SHARED.set_method(Method::Pll);
                configured(Retune { center: cfg.center, scale: cfg.scale });
                true
            }
            Request::ConfigurePio => {
                let Some(cfg) = PioConfig::decode(data) else { return false };
                if !matches!(SHARED.state(), State::Idle | State::Configured) || !direct::validate(&cfg) {
                    return false;
                }
                SHARED.pio_config.set(cfg);
                SHARED.set_method(Method::Pio);
                configured(Retune { center: cfg.center, scale: cfg.scale });
                true
            }
            Request::Start => {
                if SHARED.state() != State::Configured {
                    return false;
                }
                SHARED.send(Command::Start);
                wait_state(|s| matches!(s, State::Running | State::Fault), 100) && SHARED.state() == State::Running
            }
            Request::Stop => {
                stop_engine();
                true
            }
            Request::Retune => {
                let Some(r) = Retune::decode(data) else { return false };
                SHARED.retune.set(r);
                true
            }
            Request::Bootsel => {
                stop_engine();
                REBOOT_BOOTSEL.store(true, Ordering::Release);
                true
            }
            Request::Info | Request::Status | Request::Version => false,
        }
    }

    fn in_(&mut self, req: Request) -> Option<&[u8]> {
        match req {
            Request::Info => {
                let info = Info {
                    protocol: proto::PROTOCOL_VERSION,
                    chip: if cfg!(feature = "rp2040") { proto::Chip::Rp2040 } else { proto::Chip::Rp2350 },
                    xosc_hz: rf::XOSC_HZ,
                    sys_hz: rf::SYS_HZ,
                    buffer_samples: crate::shared::RING_SAMPLES as u32,
                    max_update_hz: rf::MAX_UPDATE_HZ,
                    gpio_mask: rf::gpio_mask(),
                    pio_min_hz: direct::MIN_HZ,
                    pio_max_hz: direct::MAX_HZ,
                    watchdog_note: crate::WATCHDOG_NOTE.load(Ordering::Relaxed),
                };
                let b = self.buf.first_chunk_mut::<{ Info::LEN }>().unwrap();
                info.encode(b);
                Some(&self.buf[..Info::LEN])
            }
            Request::Status => {
                let p = SHARED.progress.get();
                let status = Status {
                    state: SHARED.state(),
                    fault: SHARED.fault(),
                    received: SHARED.received.get(),
                    consumed: p.consumed,
                    underruns: p.underruns,
                    min_lead: SHARED.min_lead.swap_or_reset(),
                };
                let b = self.buf.first_chunk_mut::<{ Status::LEN }>().unwrap();
                status.encode(b);
                Some(&self.buf[..Status::LEN])
            }
            Request::Version => Some(crate::VERSION.as_bytes()),
            _ => None,
        }
    }
}

trait SwapOrReset {
    fn swap_or_reset(&self) -> u32;
}

impl SwapOrReset for core::sync::atomic::AtomicU32 {
    /// Read and start over (no atomic swap on the Cortex-M0+).
    fn swap_or_reset(&self) -> u32 {
        let v = self.load(Ordering::Relaxed);
        self.store(u32::MAX, Ordering::Relaxed);
        v
    }
}

impl Handler for Control {
    fn control_out(&mut self, req: ControlRequest, data: &[u8]) -> Option<OutResponse> {
        if req.recipient != Recipient::Interface {
            return None;
        }
        if req.index == self.reset_if && matches!(req.request_type, RequestType::Class | RequestType::Vendor) {
            return Some(match req.request {
                proto::RESET_REQUEST_BOOTSEL => {
                    stop_engine();
                    REBOOT_BOOTSEL.store(true, Ordering::Release);
                    OutResponse::Accepted
                }
                proto::RESET_REQUEST_FLASH => {
                    stop_engine();
                    REBOOT_FLASH.store(true, Ordering::Release);
                    OutResponse::Accepted
                }
                _ => OutResponse::Rejected,
            });
        }
        if req.index != self.txengine_if || req.request_type != RequestType::Vendor {
            return None;
        }
        let ok = Request::from_u8(req.request).is_some_and(|r| self.out(r, data));
        Some(if ok { OutResponse::Accepted } else { OutResponse::Rejected })
    }

    fn control_in<'a>(&'a mut self, req: ControlRequest, _buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        if req.recipient != Recipient::Interface
            || req.index != self.txengine_if
            || req.request_type != RequestType::Vendor
        {
            return None;
        }
        Some(match Request::from_u8(req.request).and_then(|r| self.in_(r)) {
            Some(b) => InResponse::Accepted(&b[..b.len().min(req.length as usize)]),
            None => InResponse::Rejected,
        })
    }

    fn reset(&mut self) {
        // The host reset the bus or rebooted: stop transmitting.
        stop_engine();
    }

    fn configured(&mut self, configured: bool) {
        if !configured {
            stop_engine();
        }
    }
}

/// Run the USB device forever.
pub async fn run(driver: Driver<'static>, serial: &'static str, mut watchdog: embassy_rp::watchdog::Watchdog) -> ! {
    let mut config = embassy_usb::Config::new(
        proto::VID,
        if cfg!(feature = "rp2040") { proto::PID_RP2040 } else { proto::PID_RP2350 },
    );
    config.manufacturer = Some("txengine");
    config.product = Some("txengine RF front end");
    config.serial_number = Some(serial);
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    static CONFIG_DESC: StaticCell<[u8; 128]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 32]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static CONTROL: StaticCell<Control> = StaticCell::new();
    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESC.init([0; 128]),
        BOS_DESC.init([0; 32]),
        &mut [],
        CONTROL_BUF.init([0; 64]),
    );

    let mut func = builder.function(proto::INTERFACE_CLASS, proto::INTERFACE_SUBCLASS, proto::INTERFACE_PROTOCOL);
    let mut iface = func.interface();
    let txengine_if = iface.interface_number();
    let mut alt = iface.alt_setting(proto::INTERFACE_CLASS, proto::INTERFACE_SUBCLASS, proto::INTERFACE_PROTOCOL, None);
    let mut ep = alt.endpoint_bulk_out(None, proto::SAMPLES_PACKET as u16);
    drop(func);

    let mut func = builder.function(
        proto::RESET_INTERFACE_CLASS,
        proto::RESET_INTERFACE_SUBCLASS,
        proto::RESET_INTERFACE_PROTOCOL,
    );
    let mut iface = func.interface();
    let reset_if = iface.interface_number();
    iface.alt_setting(
        proto::RESET_INTERFACE_CLASS,
        proto::RESET_INTERFACE_SUBCLASS,
        proto::RESET_INTERFACE_PROTOCOL,
        None,
    );
    drop(func);

    let control =
        CONTROL.init(Control { txengine_if: txengine_if.0 as u16, reset_if: reset_if.0 as u16, buf: [0; 64] });
    builder.handler(control);
    let mut usb = builder.build();

    let samples = async {
        let mut buf = [0u8; proto::SAMPLES_PACKET];
        loop {
            ep.wait_enabled().await;
            loop {
                // Leave the packet with the host (it is NAKed) until there is room.
                while SHARED.samples.free() < proto::SAMPLES_PACKET / 2 {
                    Timer::after_micros(100).await;
                }
                match ep.read(&mut buf).await {
                    Ok(n) => {
                        let n = n & !1;
                        SHARED.samples.push_le(&buf[..n]);
                        SHARED.received.set(SHARED.received.get() + (n / 2) as u64);
                    }
                    Err(_) => break,
                }
            }
        }
    };

    let reboot = async {
        loop {
            Timer::after_millis(10).await;
            if !crate::REBOOTING.load(Ordering::Acquire) {
                watchdog.feed(crate::WATCHDOG_TIMEOUT);
            }
            direct::service_switch();
            if REBOOT_BOOTSEL.load(Ordering::Acquire) {
                Timer::after_millis(20).await;
                crate::reboot_to_bootsel();
            }
            if REBOOT_FLASH.load(Ordering::Acquire) {
                Timer::after_millis(20).await;
                cortex_m::peripheral::SCB::sys_reset();
            }
        }
    };

    embassy_futures::join::join3(usb.run(), samples, reboot).await;
    unreachable!()
}
