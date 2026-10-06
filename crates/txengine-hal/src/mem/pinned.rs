//! Pinned anonymous memory for non-coherent PCIe bus masters (RP1 DMA).
//!
//! The memory is ordinary process memory whose pages are *long-term pinned*
//! by registering them as io_uring fixed buffers (`FOLL_LONGTERM`), which
//! prevents the kernel from migrating them (compaction, NUMA balancing, CMA
//! evacuation) for as long as the registration exists. Physical addresses
//! come from `/proc/self/pagemap`; each page is physically contiguous, so
//! the buffer is a scatter-gather list with page-sized chunks.
//!
//! CPU writes are pushed to the point of coherency with `DC CVAC` because
//! PCIe on the BCM2712 does not snoop the CPU caches.

use std::os::fd::{FromRawFd, OwnedFd};

use log::{debug, warn};

use super::{DmaMemory, cache, pagemap::Pagemap};
use crate::error::{HalError, Result};
use crate::sys::{page_size, round_up};

const IORING_REGISTER_BUFFERS: u32 = 0;

/// Pinned, page-granular DMA memory.
#[derive(Debug)]
pub struct PinnedMemory {
    ptr: *mut u8,
    len: usize,
    page: usize,
    /// Physical address of every page.
    pages: Vec<u64>,
    dma_offset: u64,
    /// The io_uring instance holding the pin (None: only mlock'ed).
    _pin: Option<OwnedFd>,
}

// SAFETY: the buffer is plain memory owned by this object.
unsafe impl Send for PinnedMemory {}
// SAFETY: see above; synchronisation of the contents is the user's job.
unsafe impl Sync for PinnedMemory {}

impl PinnedMemory {
    /// Allocate `len` bytes (rounded up to whole pages). `dma_offset` is
    /// added to physical addresses to form bus addresses.
    pub fn alloc(len: usize, dma_offset: u64) -> Result<PinnedMemory> {
        if !cache::AVAILABLE {
            return Err(HalError::Unsupported(
                "DMA memory for the RP1 needs CPU cache maintenance, which 32-bit user space \
                 cannot perform; use the 64-bit (aarch64) build of txengine on the Raspberry Pi 5"
                    .into(),
            ));
        }
        let page = page_size();
        let len = round_up(len, page);
        // SAFETY: fresh anonymous mapping.
        let ptr = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                len,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_PRIVATE | libc::MAP_ANONYMOUS | libc::MAP_POPULATE,
                -1,
                0,
            )
        };
        if ptr == libc::MAP_FAILED {
            return Err(HalError::last_os("allocate DMA memory"));
        }
        let ptr = ptr.cast::<u8>();
        let mut mem = PinnedMemory { ptr, len, page, pages: Vec::new(), dma_offset, _pin: None };

        // Never share these pages with a forked child (copy-on-write would
        // give the parent fresh pages the DMA engine does not know about).
        // SAFETY: valid mapping.
        if unsafe { libc::madvise(ptr.cast(), len, libc::MADV_DONTFORK) } != 0 {
            return Err(HalError::last_os("madvise(MADV_DONTFORK)"));
        }
        // SAFETY: valid mapping.
        if unsafe { libc::mlock(ptr.cast(), len) } != 0 {
            return Err(HalError::last_os("mlock DMA memory"));
        }
        // Touch every page so it is backed by its final physical page.
        for off in (0..len).step_by(page) {
            // SAFETY: inside the mapping.
            unsafe { ptr.add(off).write_volatile(0) };
        }
        mem._pin = match pin_pages(ptr, len) {
            Ok(fd) => Some(fd),
            Err(e) => {
                warn!("cannot long-term pin DMA memory ({e}); relying on mlock and verifying physical addresses");
                None
            }
        };

        let pm = Pagemap::open()?;
        mem.pages = (0..len / page)
            .map(|i| pm.phys(ptr as usize + i * page))
            .collect::<Result<_>>()?;
        // SAFETY: inside our mapping.
        unsafe { cache::clean_invalidate(ptr, len) };
        debug!("{}", mem.describe());
        Ok(mem)
    }

    /// Descriptor holding the long-term pin (must stay open while a DMA
    /// engine may access the memory).
    pub fn pin_fd(&self) -> Option<std::os::fd::RawFd> {
        use std::os::fd::AsRawFd;
        self._pin.as_ref().map(|f| f.as_raw_fd())
    }

    /// Re-read the page table and confirm no page has moved.
    pub fn verify_pinned(&self) -> Result<()> {
        let pm = Pagemap::open()?;
        for (i, &p) in self.pages.iter().enumerate() {
            let now = pm.phys(self.ptr as usize + i * self.page)?;
            if now != p {
                return Err(HalError::Hardware(format!(
                    "DMA page {i} migrated from {p:#x} to {now:#x}"
                )));
            }
        }
        Ok(())
    }
}

/// Long-term pin pages by registering them as io_uring fixed buffers.
fn pin_pages(ptr: *mut u8, len: usize) -> Result<OwnedFd> {
    let mut params = [0u8; 120]; // struct io_uring_params
    // SAFETY: io_uring_setup with a zeroed parameter block.
    let fd = unsafe { libc::syscall(libc::SYS_io_uring_setup, 1u32, params.as_mut_ptr()) };
    if fd < 0 {
        return Err(HalError::last_os("io_uring_setup"));
    }
    // SAFETY: the kernel returned a new descriptor we now own.
    let ring = unsafe { OwnedFd::from_raw_fd(fd as i32) };
    // A single fixed buffer is limited to 1 GiB; ours are far smaller.
    let iov = libc::iovec { iov_base: ptr.cast(), iov_len: len };
    // SAFETY: registering one valid iovec describing our mapping.
    let rc = unsafe {
        libc::syscall(libc::SYS_io_uring_register, fd as i32, IORING_REGISTER_BUFFERS, &iov as *const libc::iovec, 1u32)
    };
    if rc < 0 {
        return Err(HalError::last_os("io_uring_register(BUFFERS)"));
    }
    Ok(ring)
}

impl Drop for PinnedMemory {
    fn drop(&mut self) {
        // Unpin (close the ring) before unmapping.
        self._pin = None;
        // SAFETY: we own the mapping.
        unsafe {
            libc::munmap(self.ptr.cast(), self.len);
        }
    }
}

impl DmaMemory for PinnedMemory {
    fn as_ptr(&self) -> *mut u8 {
        self.ptr
    }
    fn len(&self) -> usize {
        self.len
    }
    fn dma_addr_at(&self, offset: usize) -> u64 {
        assert!(offset < self.len);
        self.pages[offset / self.page] + (offset % self.page) as u64 + self.dma_offset
    }
    fn chunk_size(&self) -> usize {
        self.page
    }
    fn flush(&self, offset: usize, len: usize) {
        assert!(offset + len <= self.len);
        // SAFETY: range checked against our mapping.
        unsafe { cache::clean(self.ptr.add(offset), len) }
    }
    fn invalidate(&self, offset: usize, len: usize) {
        assert!(offset + len <= self.len);
        // SAFETY: range checked against our mapping.
        unsafe { cache::clean_invalidate(self.ptr.add(offset), len) }
    }
    fn describe(&self) -> String {
        format!(
            "pinned memory: {} KiB in {} pages of {} KiB ({}), cached with explicit maintenance",
            self.len / 1024,
            self.pages.len(),
            self.page / 1024,
            if self._pin.is_some() { "io_uring long-term pin" } else { "mlock only" }
        )
    }
}
