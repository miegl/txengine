//! Raspberry Pi Pico (RP2040) and Pico 2 (RP2350) as a USB RF front end,
//! running the txengine firmware (`firmware/pico`): txengine renders the
//! samples and plans the clocks, the Pico synthesises and modulates the
//! carrier. The protocol is in `txengine-pico-protocol`.

pub mod fm;
pub mod plan;

use std::io;

pub use txengine_pico_protocol as protocol;
use txengine_pico_protocol::{self as proto, Config, Info, PioConfig, Request, Retune, Status};

use crate::usbfs::{self, Device, DeviceInfo};
use crate::{HalError, Result};

/// The Picos running the txengine firmware.
pub fn list() -> Vec<DeviceInfo> {
    usbfs::find(proto::VID, &[proto::PID_RP2040, proto::PID_RP2350])
        .into_iter()
        .filter(|d| {
            d.interface_class(0) == Some((proto::INTERFACE_CLASS, proto::INTERFACE_SUBCLASS, proto::INTERFACE_PROTOCOL))
        })
        .collect()
}

/// Pick the Pico whose serial number or USB port (`3-1.2`) is `which`, or
/// the only one.
pub fn find(which: Option<&str>) -> Result<DeviceInfo> {
    let all = list();
    let describe = |all: &[DeviceInfo]| {
        all.iter().map(|d| format!("{} (port {})", d.serial, d.port())).collect::<Vec<_>>().join(", ")
    };
    match which {
        Some(w) => all
            .iter()
            .find(|d| d.serial.eq_ignore_ascii_case(w) || d.port() == w)
            .cloned()
            .ok_or_else(|| HalError::Config(format!("no Pico '{w}' (found: {})", describe(&all)))),
        None => match all.len() {
            0 => Err(HalError::Unsupported(
                "no Raspberry Pi Pico with the txengine firmware is connected (see firmware/pico)".into(),
            )),
            1 => Ok(all[0].clone()),
            _ => {
                Err(HalError::Config(format!("several Picos are connected, pick one with --pico: {}", describe(&all))))
            }
        },
    }
}

/// An open connection to the firmware.
#[derive(Debug)]
pub struct Pico {
    dev: Device,
    interface: u16,
    pub usb: DeviceInfo,
    pub info: Info,
}

fn io_err(what: &str, e: io::Error) -> HalError {
    HalError::io(format!("Pico: {what}"), e)
}

impl Pico {
    pub fn open(usb: &DeviceInfo) -> Result<Pico> {
        let interface = 0;
        let dev = Device::open(usb, interface)?;
        let mut pico = Pico {
            dev,
            interface: interface as u16,
            usb: usb.clone(),
            info: Info {
                protocol: 0,
                chip: proto::Chip::Rp2040,
                xosc_hz: 0,
                sys_hz: 0,
                buffer_samples: 0,
                max_update_hz: 0,
                gpio_mask: 0,
                pio_min_hz: 0,
                pio_max_hz: 0,
                watchdog_note: 0,
            },
        };
        let mut b = [0u8; Info::LEN];
        let n = pico.request_in(Request::Info, &mut b).map_err(|e| io_err("reading its description", e))?;
        pico.info = Info::decode(&b[..n]).ok_or_else(|| HalError::Hardware("Pico: malformed description".into()))?;
        if pico.info.protocol != proto::PROTOCOL_VERSION {
            return Err(HalError::Unsupported(format!(
                "the Pico's firmware speaks protocol {}, this txengine {}: update the firmware",
                pico.info.protocol,
                proto::PROTOCOL_VERSION
            )));
        }
        Ok(pico)
    }

    fn request_out(&self, r: Request, data: &[u8]) -> io::Result<()> {
        self.dev.control_out(usbfs::TYPE_VENDOR | usbfs::RECIPIENT_INTERFACE, r as u8, 0, self.interface, data)
    }

    fn request_in(&self, r: Request, buf: &mut [u8]) -> io::Result<usize> {
        self.dev.control_in(usbfs::TYPE_VENDOR | usbfs::RECIPIENT_INTERFACE, r as u8, 0, self.interface, buf)
    }

    pub fn status(&self) -> Result<Status> {
        let mut b = [0u8; Status::LEN];
        let n = self.request_in(Request::Status, &mut b).map_err(|e| io_err("reading its status", e))?;
        Status::decode(&b[..n]).ok_or_else(|| HalError::Hardware("Pico: malformed status".into()))
    }

    /// Send a configuration; the firmware refuses one out of range (a stall).
    pub fn configure(&self, c: &Config) -> Result<()> {
        let mut b = [0u8; Config::LEN];
        c.encode(&mut b);
        self.request_out(Request::Configure, &b).map_err(|e| match e.raw_os_error() {
            Some(libc::EPIPE) => HalError::Config("the Pico refused the configuration".into()),
            _ => io_err("configuring", e),
        })
    }

    /// Send a configuration of the PIO method.
    pub fn configure_pio(&self, c: &PioConfig) -> Result<()> {
        let mut b = [0u8; PioConfig::LEN];
        c.encode(&mut b);
        self.request_out(Request::ConfigurePio, &b).map_err(|e| match e.raw_os_error() {
            Some(libc::EPIPE) => HalError::Config("the Pico refused the configuration".into()),
            _ => io_err("configuring", e),
        })
    }

    pub fn start(&self) -> Result<()> {
        self.request_out(Request::Start, &[]).map_err(|e| match e.raw_os_error() {
            Some(libc::EPIPE) => {
                let why = self.status().map(|s| s.fault.describe()).unwrap_or("unknown");
                HalError::Hardware(format!("the Pico did not start: {why}"))
            }
            _ => io_err("starting", e),
        })
    }

    pub fn stop(&self) -> Result<()> {
        self.request_out(Request::Stop, &[]).map_err(|e| io_err("stopping", e))
    }

    pub fn retune(&self, r: &Retune) -> Result<()> {
        let mut b = [0u8; Retune::LEN];
        r.encode(&mut b);
        self.request_out(Request::Retune, &b).map_err(|e| io_err("retuning", e))
    }

    /// Reboot into the USB bootloader (for flashing).
    pub fn bootsel(&self) -> Result<()> {
        self.request_out(Request::Bootsel, &[]).map_err(|e| io_err("rebooting", e))
    }

    /// Queue samples (`i16` little endian) for the bulk endpoint.
    pub fn send(&mut self, bytes: Vec<u8>) -> Result<()> {
        self.dev.submit_bulk_out(proto::SAMPLES_ENDPOINT, bytes).map_err(|e| io_err("sending samples", e))
    }

    /// Collect finished transfers; an error means samples were lost.
    pub fn reap(&mut self) -> Result<usize> {
        self.dev.reap().map_err(|e| io_err("sending samples", e))
    }

    /// Cancel the samples still on their way.
    pub fn discard(&mut self) {
        self.dev.discard_all();
    }

    pub fn describe(&self) -> String {
        format!("Raspberry Pi Pico ({}, serial {}, port {})", self.info.chip.name(), self.usb.serial, self.usb.port())
    }
}
