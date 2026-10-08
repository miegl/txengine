//! Virtual to physical address translation via `/proc/self/pagemap`.
//!
//! Requires `CAP_SYS_ADMIN` (the kernel reports PFN 0 otherwise).

use std::fs::File;
use std::os::unix::fs::FileExt;
use std::path::Path;

use crate::error::{HalError, Result};
use crate::sys::{open_file, page_size};

const PM_PRESENT: u64 = 1 << 63;
const PM_PFN_MASK: u64 = (1 << 55) - 1;

pub struct Pagemap {
    file: File,
    page_size: usize,
}

impl Pagemap {
    pub fn open() -> Result<Pagemap> {
        Ok(Pagemap { file: open_file(Path::new("/proc/self/pagemap"))?, page_size: page_size() })
    }

    /// Physical address of a mapped, resident virtual address.
    pub fn phys(&self, virt: usize) -> Result<u64> {
        let v = virt;
        let mut entry = [0u8; 8];
        let off = (v / self.page_size * 8) as u64;
        self.file.read_exact_at(&mut entry, off).map_err(|e| HalError::io("read /proc/self/pagemap", e))?;
        let e = u64::from_le_bytes(entry);
        if e & PM_PRESENT == 0 {
            return Err(HalError::Hardware(format!("page at {virt:#x} is not resident")));
        }
        let pfn = e & PM_PFN_MASK;
        if pfn == 0 {
            return Err(HalError::Config(
                "pagemap reports PFN 0: physical addresses need CAP_SYS_ADMIN (run as root)".into(),
            ));
        }
        Ok(pfn * self.page_size as u64 + (v % self.page_size) as u64)
    }

    /// Physical address of `[virt, virt + len)`, verifying the range is
    /// physically contiguous.
    pub fn contiguous_phys(&self, virt: usize, len: usize) -> Result<u64> {
        let base = self.phys(virt)?;
        let mut off = self.page_size - (virt % self.page_size);
        while off < len {
            let p = self.phys(virt + off)?;
            if p != base + off as u64 {
                return Err(HalError::Hardware(format!(
                    "buffer is not physically contiguous (offset {off:#x}: {p:#x} != {:#x})",
                    base + off as u64
                )));
            }
            off += self.page_size;
        }
        Ok(base)
    }
}
