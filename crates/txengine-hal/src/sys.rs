//! Thin safe wrappers around the OS facilities the hardware layer needs.

use std::ffi::CString;
use std::fs::File;
use std::io;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd, RawFd};
use std::path::Path;
use std::ptr::NonNull;
use std::time::{Duration, Instant};

use crate::error::{HalError, Result};

/// System page size (16 KiB on the Raspberry Pi 5 kernel, 4 KiB elsewhere).
pub fn page_size() -> usize {
    // SAFETY: sysconf has no preconditions.
    let ps = unsafe { libc::sysconf(libc::_SC_PAGESIZE) };
    if ps > 0 { ps as usize } else { 4096 }
}

pub fn round_up(v: usize, align: usize) -> usize {
    v.div_ceil(align) * align
}

/// Open a file with raw `open(2)` flags.
pub fn open(path: &Path, flags: libc::c_int) -> Result<OwnedFd> {
    let cpath = CString::new(path.as_os_str().as_encoded_bytes())
        .map_err(|_| HalError::Config(format!("invalid path {}", path.display())))?;
    // SAFETY: cpath is a valid NUL terminated string.
    let fd = unsafe { libc::open(cpath.as_ptr(), flags | libc::O_CLOEXEC) };
    if fd < 0 {
        return Err(HalError::last_os(format!("open {}", path.display())));
    }
    // SAFETY: fd is a freshly opened, owned descriptor.
    Ok(unsafe { OwnedFd::from_raw_fd(fd) })
}

/// A memory mapping that is unmapped on drop.
#[derive(Debug)]
pub struct Mapping {
    ptr: NonNull<u8>,
    len: usize,
}

// SAFETY: the mapping is plain memory/device memory; synchronisation of the
// contents is the responsibility of the users (volatile register access).
unsafe impl Send for Mapping {}
// SAFETY: see above.
unsafe impl Sync for Mapping {}

impl Mapping {
    /// `mmap(2)` `len` bytes of `fd` at `offset` (must be page aligned) with
    /// read/write access, shared.
    pub fn map(fd: RawFd, offset: u64, len: usize, what: &str) -> Result<Mapping> {
        let len = round_up(len, page_size());
        // SAFETY: we request a fresh mapping; the kernel validates fd/offset.
        let ptr = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                len,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                fd,
                offset as libc::off_t,
            )
        };
        if ptr == libc::MAP_FAILED {
            return Err(HalError::last_os(format!("mmap {what} (offset {offset:#x}, len {len:#x})")));
        }
        Ok(Mapping { ptr: NonNull::new(ptr.cast()).expect("mmap returned NULL"), len })
    }

    /// Map a physical address range through `/dev/mem`.
    ///
    /// `uncached` selects `O_SYNC`, which yields a device (for MMIO) or
    /// non-cacheable (for RAM) mapping.
    pub fn map_physical(phys: u64, len: usize, uncached: bool, what: &str) -> Result<Mapping> {
        if !phys.is_multiple_of(page_size() as u64) {
            return Err(HalError::Config(format!("{what}: physical address {phys:#x} is not page aligned")));
        }
        let flags = libc::O_RDWR | if uncached { libc::O_SYNC } else { 0 };
        let fd = open(Path::new("/dev/mem"), flags)?;
        Mapping::map(fd.as_raw_fd(), phys, len, what)
    }

    pub fn as_ptr(&self) -> *mut u8 {
        self.ptr.as_ptr()
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

impl Drop for Mapping {
    fn drop(&mut self) {
        // SAFETY: ptr/len describe a mapping we created and still own.
        unsafe {
            libc::munmap(self.ptr.as_ptr().cast(), self.len);
        }
    }
}

/// Read a small text/binary file (sysfs, procfs, device-tree).
pub fn read_file(path: impl AsRef<Path>) -> io::Result<Vec<u8>> {
    std::fs::read(path)
}

/// Read a sysfs/procfs text attribute, trimmed.
pub fn read_trimmed(path: impl AsRef<Path>) -> io::Result<String> {
    Ok(String::from_utf8_lossy(&std::fs::read(path)?).trim().to_string())
}

/// Poll `cond` until it returns true or `timeout` expires.
///
/// Spins briefly first (most hardware conditions resolve within
/// microseconds), then yields the CPU between attempts.
pub fn wait_until(timeout: Duration, what: &str, mut cond: impl FnMut() -> bool) -> Result<Duration> {
    let start = Instant::now();
    let mut spins = 0u32;
    loop {
        if cond() {
            return Ok(start.elapsed());
        }
        let elapsed = start.elapsed();
        if elapsed >= timeout {
            // Re-check once more: we may have been descheduled.
            if cond() {
                return Ok(elapsed);
            }
            return Err(HalError::Timeout(format!("{what} (after {:?})", elapsed)));
        }
        spins += 1;
        if spins < 1000 {
            std::hint::spin_loop();
        } else if spins < 2000 {
            std::thread::yield_now();
        } else {
            std::thread::sleep(Duration::from_micros(50));
        }
    }
}

/// Sleep for a short, bounded hardware settling time.
///
/// Only used where the hardware offers no status bit to poll (documented at
/// each call site).
pub fn settle(d: Duration) {
    std::thread::sleep(d);
}

/// Duplicate a descriptor into an owned one (used to keep heap buffers alive).
pub fn dup_fd(fd: &impl AsRawFd) -> Result<OwnedFd> {
    // SAFETY: fcntl on a valid descriptor.
    let nfd = unsafe { libc::fcntl(fd.as_raw_fd(), libc::F_DUPFD_CLOEXEC, 0) };
    if nfd < 0 {
        return Err(HalError::last_os("dup"));
    }
    // SAFETY: nfd is a new, owned descriptor.
    Ok(unsafe { OwnedFd::from_raw_fd(nfd) })
}

/// Open a file read-only as `File`, with context.
pub fn open_file(path: &Path) -> Result<File> {
    File::open(path).map_err(|e| HalError::io(format!("open {}", path.display()), e))
}
