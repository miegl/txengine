//! VideoCore firmware mailbox (`/dev/vcio`) and GPU memory allocation.

use std::os::fd::{AsRawFd, OwnedFd};
use std::path::Path;

use log::debug;

use super::DmaMemory;
use crate::error::{HalError, Result};
use crate::sys::{Mapping, open, page_size, round_up};

/// `_IOWR(100, 0, char *)` – the size field depends on the pointer width of
/// the calling process.
const IOCTL_MBOX_PROPERTY: u32 = (3 << 30) | ((std::mem::size_of::<*mut u8>() as u32) << 16) | (100 << 8);

const TAG_ALLOCATE_MEMORY: u32 = 0x0003_000c;
const TAG_LOCK_MEMORY: u32 = 0x0003_000d;
const TAG_UNLOCK_MEMORY: u32 = 0x0003_000e;
const TAG_RELEASE_MEMORY: u32 = 0x0003_000f;
const TAG_GET_FIRMWARE_REVISION: u32 = 0x0000_0001;
const TAG_GET_THROTTLED: u32 = 0x0003_0046;

const RESPONSE_SUCCESS: u32 = 0x8000_0000;

/// Allocation flags (see the firmware's mailbox property interface).
pub mod flags {
    /// 0xC0000000 bus alias: uncached, bypasses the VideoCore L2.
    pub const DIRECT: u32 = 1 << 2;
    /// 0x80000000 bus alias: non-allocating in L2 but coherent.
    pub const COHERENT: u32 = 2 << 2;
    /// Initialise the buffer to zero.
    pub const ZERO: u32 = 1 << 4;
    /// Keep the allocation locked (permalock hint).
    pub const HINT_PERMALOCK: u32 = 1 << 6;
}

/// Handle to the firmware property mailbox.
pub struct Mailbox {
    fd: OwnedFd,
}

impl std::fmt::Debug for Mailbox {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("Mailbox")
    }
}

impl Mailbox {
    pub fn open() -> Result<Mailbox> {
        let fd = open(Path::new("/dev/vcio"), libc::O_RDWR).map_err(|e| {
            HalError::Unsupported(format!(
                "cannot open the VideoCore mailbox /dev/vcio ({e}); is this a Raspberry Pi \
                 running a kernel with the bcm2835 mailbox driver?"
            ))
        })?;
        Ok(Mailbox { fd })
    }

    /// Issue a single-tag property request. Returns the response value words.
    pub fn property(&self, tag: u32, request: &[u32], response_words: usize) -> Result<Vec<u32>> {
        let value_words = request.len().max(response_words);
        let mut buf = vec![0u32; 6 + value_words];
        let total = buf.len();
        buf[0] = (total * 4) as u32;
        buf[1] = 0; // process request
        buf[2] = tag;
        buf[3] = (value_words * 4) as u32;
        buf[4] = 0; // request indicator
        buf[5..5 + request.len()].copy_from_slice(request);
        buf[total - 1] = 0; // end tag

        // SAFETY: the buffer is a well-formed property message of the stated size.
        let rc = unsafe { libc::ioctl(self.fd.as_raw_fd(), IOCTL_MBOX_PROPERTY as _, buf.as_mut_ptr()) };
        if rc < 0 {
            return Err(HalError::last_os(format!("mailbox property {tag:#x}")));
        }
        if buf[1] != RESPONSE_SUCCESS {
            return Err(HalError::Hardware(format!("mailbox property {tag:#x} failed (response code {:#x})", buf[1])));
        }
        if buf[4] & 0x8000_0000 == 0 {
            return Err(HalError::Hardware(format!("mailbox tag {tag:#x} not handled by firmware")));
        }
        Ok(buf[5..5 + response_words].to_vec())
    }

    pub fn firmware_revision(&self) -> Result<u32> {
        Ok(self.property(TAG_GET_FIRMWARE_REVISION, &[], 1)?[0])
    }

    /// Power and thermal state as reported by `vcgencmd get_throttled`.
    pub fn throttled(&self) -> Result<u32> {
        Ok(self.property(TAG_GET_THROTTLED, &[0], 1)?[0])
    }
}

/// Memory allocated by the VideoCore firmware, mapped uncached.
pub struct VcMemory {
    mbox: Mailbox,
    handle: u32,
    bus_addr: u32,
    map: Option<Mapping>,
    len: usize,
}

impl std::fmt::Debug for VcMemory {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.describe())
    }
}

impl VcMemory {
    /// Allocate `len` bytes of uncached (`DIRECT`) GPU memory.
    pub fn alloc(len: usize) -> Result<VcMemory> {
        let mbox = Mailbox::open()?;
        let len = round_up(len, page_size());
        let handle =
            mbox.property(TAG_ALLOCATE_MEMORY, &[len as u32, page_size() as u32, flags::DIRECT | flags::ZERO], 1)?[0];
        if handle == 0 {
            return Err(HalError::Hardware(format!(
                "VideoCore could not allocate {} KiB (increase gpu_mem?)",
                len / 1024
            )));
        }
        let mut mem = VcMemory { mbox, handle, bus_addr: 0, map: None, len };
        mem.bus_addr = mem.mbox.property(TAG_LOCK_MEMORY, &[handle], 1)?[0];
        if mem.bus_addr == 0 {
            return Err(HalError::Hardware("VideoCore could not lock the allocation".into()));
        }
        let phys = (mem.bus_addr & 0x3fff_ffff) as u64;
        mem.map = Some(Mapping::map_physical(phys, len, true, "VideoCore memory")?);
        debug!("{}", mem.describe());
        Ok(mem)
    }

    /// A token that can free this allocation from the shutdown guardian
    /// (a forked child) after it has stopped the DMA engine.
    pub fn release_token(&self) -> Result<VcRelease> {
        Ok(VcRelease { mbox: Mailbox { fd: crate::sys::dup_fd(&self.mbox.fd)? }, handle: self.handle })
    }
}

/// See [`VcMemory::release_token`].
#[derive(Debug)]
pub struct VcRelease {
    mbox: Mailbox,
    handle: u32,
}

impl VcRelease {
    /// Descriptor the guardian must keep open.
    pub fn fd(&self) -> std::os::fd::RawFd {
        self.mbox.fd.as_raw_fd()
    }

    /// Unlock and free the allocation.
    pub fn release(&self) {
        let _ = self.mbox.property(TAG_UNLOCK_MEMORY, &[self.handle], 1);
        let _ = self.mbox.property(TAG_RELEASE_MEMORY, &[self.handle], 1);
    }
}

impl Drop for VcMemory {
    fn drop(&mut self) {
        self.map = None;
        if self.handle != 0 {
            if self.bus_addr != 0 {
                let _ = self.mbox.property(TAG_UNLOCK_MEMORY, &[self.handle], 1);
            }
            let _ = self.mbox.property(TAG_RELEASE_MEMORY, &[self.handle], 1);
        }
    }
}

impl DmaMemory for VcMemory {
    fn as_ptr(&self) -> *mut u8 {
        self.map.as_ref().expect("mapped").as_ptr()
    }
    fn len(&self) -> usize {
        self.len
    }
    fn dma_addr_at(&self, offset: usize) -> u64 {
        assert!(offset < self.len);
        self.bus_addr as u64 + offset as u64
    }
    fn chunk_size(&self) -> usize {
        self.len
    }
    fn flush(&self, _offset: usize, _len: usize) {
        crate::mmio::barrier::dsb();
    }
    fn invalidate(&self, _offset: usize, _len: usize) {
        crate::mmio::barrier::dsb();
    }
    fn describe(&self) -> String {
        format!("VideoCore memory: {} KiB at bus {:#010x} (uncached)", self.len / 1024, self.bus_addr)
    }
}
