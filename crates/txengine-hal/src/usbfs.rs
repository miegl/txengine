//! A minimal USB host interface over Linux usbfs (`/dev/bus/usb`): finding
//! devices in sysfs, control transfers, and asynchronous bulk OUT transfers.
//! No libusb, so the static binaries stay self-contained.

use std::ffi::c_void;
use std::fs::{self, File, OpenOptions};
use std::io;
use std::os::fd::AsRawFd;
use std::path::{Path, PathBuf};

use crate::{HalError, Result};

const fn ioc(dir: u32, nr: u32, size: usize) -> u32 {
    (dir << 30) | ((size as u32) << 16) | ((b'U' as u32) << 8) | nr
}
const IOC_NONE: u32 = 0;
const IOC_WRITE: u32 = 1;
const IOC_READ: u32 = 2;

#[repr(C)]
struct CtrlTransfer {
    request_type: u8,
    request: u8,
    value: u16,
    index: u16,
    length: u16,
    timeout_ms: u32,
    data: *mut c_void,
}

#[repr(C)]
struct Urb {
    kind: u8,
    endpoint: u8,
    status: i32,
    flags: u32,
    buffer: *mut c_void,
    buffer_length: i32,
    actual_length: i32,
    start_frame: i32,
    number_of_packets: i32,
    error_count: i32,
    signr: u32,
    usercontext: *mut c_void,
}

const USBDEVFS_CONTROL: u32 = ioc(IOC_READ | IOC_WRITE, 0, size_of::<CtrlTransfer>());
const USBDEVFS_SUBMITURB: u32 = ioc(IOC_READ, 10, size_of::<Urb>());
const USBDEVFS_DISCARDURB: u32 = ioc(IOC_NONE, 11, 0);
const USBDEVFS_REAPURBNDELAY: u32 = ioc(IOC_WRITE, 13, size_of::<*mut c_void>());
const USBDEVFS_CLAIMINTERFACE: u32 = ioc(IOC_READ, 15, size_of::<u32>());
const USBDEVFS_RELEASEINTERFACE: u32 = ioc(IOC_READ, 16, size_of::<u32>());
const URB_TYPE_BULK: u8 = 3;

/// `bmRequestType` bits.
pub const DIR_IN: u8 = 0x80;
pub const TYPE_VENDOR: u8 = 0x40;
pub const TYPE_CLASS: u8 = 0x20;
pub const RECIPIENT_INTERFACE: u8 = 0x01;

/// A USB device as sysfs describes it.
#[derive(Debug, Clone)]
pub struct DeviceInfo {
    /// sysfs directory, e.g. `/sys/bus/usb/devices/3-1.2`.
    pub sysfs: PathBuf,
    pub vid: u16,
    pub pid: u16,
    pub bus: u32,
    pub dev: u32,
    pub manufacturer: String,
    pub product: String,
    pub serial: String,
}

impl DeviceInfo {
    /// Bus port path (`3-1.2`), stable while the device stays in its port.
    pub fn port(&self) -> String {
        self.sysfs.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default()
    }

    /// `(class, subclass, protocol)` of interface `n`.
    pub fn interface_class(&self, n: u8) -> Option<(u8, u8, u8)> {
        let dir = self.sysfs.join(format!("{}:1.{n}", self.port()));
        let hex = |f: &str| u8::from_str_radix(read_trim(&dir.join(f))?.as_str(), 16).ok();
        Some((hex("bInterfaceClass")?, hex("bInterfaceSubClass")?, hex("bInterfaceProtocol")?))
    }
}

fn read_trim(p: &Path) -> Option<String> {
    fs::read_to_string(p).ok().map(|s| s.trim().to_string())
}

/// All USB devices with vendor `vid` and one of the product IDs.
pub fn find(vid: u16, pids: &[u16]) -> Vec<DeviceInfo> {
    let Ok(dir) = fs::read_dir("/sys/bus/usb/devices") else { return Vec::new() };
    let mut found: Vec<DeviceInfo> = dir
        .flatten()
        .filter_map(|e| {
            let p = e.path();
            let hex = |f: &str| u16::from_str_radix(read_trim(&p.join(f))?.as_str(), 16).ok();
            let dec = |f: &str| read_trim(&p.join(f))?.parse::<u32>().ok();
            let (v, pid) = (hex("idVendor")?, hex("idProduct")?);
            if v != vid || !pids.contains(&pid) {
                return None;
            }
            Some(DeviceInfo {
                vid: v,
                pid,
                bus: dec("busnum").unwrap_or(0),
                dev: dec("devnum").unwrap_or(0),
                manufacturer: read_trim(&p.join("manufacturer")).unwrap_or_default(),
                product: read_trim(&p.join("product")).unwrap_or_default(),
                serial: read_trim(&p.join("serial")).unwrap_or_default(),
                sysfs: p.clone(),
            })
        })
        .collect();
    found.sort_by_key(|d| (d.bus, d.dev));
    found
}

/// An open device with one claimed interface.
#[derive(Debug)]
pub struct Device {
    file: File,
    interface: u32,
    /// Bulk transfers submitted and not yet reaped (boxed: the kernel holds
    /// pointers to the URB and its buffer until they are reaped, while the
    /// vector may move).
    #[allow(clippy::vec_box)]
    in_flight: Vec<Box<Transfer>>,
}

#[derive(Debug)]
struct Transfer {
    urb: Urb,
    data: Vec<u8>,
}

// SAFETY: the raw pointers in `Urb` point into the same boxed `Transfer`.
unsafe impl Send for Transfer {}

impl std::fmt::Debug for Urb {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Urb(ep {:#x}, status {}, {} B)", self.endpoint, self.status, self.buffer_length)
    }
}

impl Device {
    /// Open `info` and claim `interface`.
    pub fn open(info: &DeviceInfo, interface: u32) -> Result<Device> {
        let path = format!("/dev/bus/usb/{:03}/{:03}", info.bus, info.dev);
        let file = OpenOptions::new().read(true).write(true).open(&path).map_err(|e| {
            let hint = if e.kind() == io::ErrorKind::PermissionDenied {
                " (run as root, or install a udev rule that gives your user access)"
            } else {
                ""
            };
            HalError::io(format!("opening {path}{hint}"), e)
        })?;
        let mut n = interface;
        // SAFETY: the ioctl reads an unsigned int.
        if unsafe { libc::ioctl(file.as_raw_fd(), USBDEVFS_CLAIMINTERFACE as _, &mut n) } < 0 {
            return Err(HalError::last_os(format!("claiming interface {interface} of {path}")));
        }
        Ok(Device { file, interface, in_flight: Vec::new() })
    }

    fn control(&self, request_type: u8, request: u8, value: u16, index: u16, data: &mut [u8]) -> io::Result<usize> {
        let mut c = CtrlTransfer {
            request_type,
            request,
            value,
            index,
            length: data.len() as u16,
            timeout_ms: 1000,
            data: data.as_mut_ptr().cast(),
        };
        // SAFETY: `c` describes `data`, which outlives the synchronous call.
        let n = unsafe { libc::ioctl(self.file.as_raw_fd(), USBDEVFS_CONTROL as _, &mut c) };
        if n < 0 { Err(io::Error::last_os_error()) } else { Ok(n as usize) }
    }

    /// Control transfer to the device (data stage OUT).
    pub fn control_out(&self, request_type: u8, request: u8, value: u16, index: u16, data: &[u8]) -> io::Result<()> {
        let mut buf = data.to_vec();
        self.control(request_type & !DIR_IN, request, value, index, &mut buf).map(|_| ())
    }

    /// Control transfer from the device; returns the bytes received.
    pub fn control_in(
        &self,
        request_type: u8,
        request: u8,
        value: u16,
        index: u16,
        buf: &mut [u8],
    ) -> io::Result<usize> {
        self.control(request_type | DIR_IN, request, value, index, buf)
    }

    /// Queue a bulk OUT transfer of `data` to `endpoint`.
    pub fn submit_bulk_out(&mut self, endpoint: u8, data: Vec<u8>) -> io::Result<()> {
        let mut t = Box::new(Transfer {
            urb: Urb {
                kind: URB_TYPE_BULK,
                endpoint,
                status: 0,
                flags: 0,
                buffer: std::ptr::null_mut(),
                buffer_length: data.len() as i32,
                actual_length: 0,
                start_frame: 0,
                number_of_packets: 0,
                error_count: 0,
                signr: 0,
                usercontext: std::ptr::null_mut(),
            },
            data,
        });
        t.urb.buffer = t.data.as_mut_ptr().cast();
        t.urb.usercontext = (&mut *t as *mut Transfer).cast();
        // SAFETY: the URB and its buffer live in the box, which stays in
        // `in_flight` (and is not moved) until the kernel returns it.
        if unsafe { libc::ioctl(self.file.as_raw_fd(), USBDEVFS_SUBMITURB as _, &mut t.urb) } < 0 {
            return Err(io::Error::last_os_error());
        }
        self.in_flight.push(t);
        Ok(())
    }

    /// Collect finished bulk transfers without waiting; returns how many
    /// finished, or the first one's error.
    pub fn reap(&mut self) -> io::Result<usize> {
        let mut done = 0;
        let mut first_err = None;
        loop {
            let mut urb: *mut Urb = std::ptr::null_mut();
            // SAFETY: the kernel writes the address of a URB we submitted.
            if unsafe { libc::ioctl(self.file.as_raw_fd(), USBDEVFS_REAPURBNDELAY as _, &mut urb) } < 0 {
                let e = io::Error::last_os_error();
                if e.raw_os_error() == Some(libc::EAGAIN) {
                    break;
                }
                return Err(e);
            }
            // SAFETY: `urb` is the URB of a `Transfer` we own, which its user
            // context points to (the field order of `Transfer` is the
            // compiler's, so `urb` need not be its address).
            let (status, ctx) = unsafe { ((*urb).status, (*urb).usercontext as *const Transfer) };
            self.in_flight.retain(|t| !std::ptr::eq(&**t, ctx));
            done += 1;
            if status != 0 && first_err.is_none() {
                first_err = Some(io::Error::from_raw_os_error(-status));
            }
        }
        match first_err {
            Some(e) => Err(e),
            None => Ok(done),
        }
    }

    /// Transfers submitted and not yet reaped.
    pub fn in_flight(&self) -> usize {
        self.in_flight.len()
    }

    /// Cancel every transfer in flight and wait until the kernel returns them.
    pub fn discard_all(&mut self) {
        for t in &mut self.in_flight {
            // SAFETY: the URB was submitted on this file; discarding a
            // finished one fails harmlessly.
            unsafe { libc::ioctl(self.file.as_raw_fd(), USBDEVFS_DISCARDURB as _, &mut t.urb) };
        }
        let t0 = std::time::Instant::now();
        while !self.in_flight.is_empty() && t0.elapsed() < std::time::Duration::from_millis(500) {
            if self.reap().is_err() {
                // Discarded URBs report an error status; keep collecting.
                continue;
            }
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
    }
}

impl Drop for Device {
    fn drop(&mut self) {
        self.discard_all();
        if !self.in_flight.is_empty() {
            // The kernel still holds pointers into these: never free them.
            std::mem::forget(std::mem::take(&mut self.in_flight));
        }
        let mut n = self.interface;
        // SAFETY: the ioctl reads an unsigned int.
        unsafe { libc::ioctl(self.file.as_raw_fd(), USBDEVFS_RELEASEINTERFACE as _, &mut n) };
    }
}
