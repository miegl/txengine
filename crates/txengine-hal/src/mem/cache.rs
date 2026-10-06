//! CPU data cache maintenance from user space.
//!
//! On AArch64 Linux enables EL0 cache maintenance by virtual address
//! (`SCTLR_EL1.UCI`), so `DC CVAC`/`DC CIVAC` can clean/invalidate lines to
//! the point of coherency – exactly what is needed before a non-coherent bus
//! master (the RP1 DMA engine behind PCIe) reads memory written by the CPU.
//! AArch32 user space has no such instructions.

/// Whether user-space cache maintenance to the point of coherency is available.
pub const AVAILABLE: bool = cfg!(target_arch = "aarch64");

/// Smallest data cache line size in bytes (from `CTR_EL0.DminLine`).
pub fn dcache_line() -> usize {
    #[cfg(target_arch = "aarch64")]
    {
        let ctr: u64;
        // SAFETY: CTR_EL0 is readable from EL0 on Linux (SCTLR_EL1.UCT).
        unsafe { core::arch::asm!("mrs {}, ctr_el0", out(reg) ctr, options(nomem, nostack)) };
        4usize << ((ctr >> 16) & 0xf)
    }
    #[cfg(not(target_arch = "aarch64"))]
    {
        64
    }
}

/// Clean `[ptr, ptr + len)` to the point of coherency and wait for completion.
///
/// # Safety
/// The range must be mapped in this process.
pub unsafe fn clean(ptr: *const u8, len: usize) {
    #[cfg(target_arch = "aarch64")]
    {
        let line = dcache_line();
        let mut a = (ptr as usize) & !(line - 1);
        let end = ptr as usize + len;
        while a < end {
            // SAFETY: caller guarantees the address is mapped.
            unsafe { core::arch::asm!("dc cvac, {}", in(reg) a, options(nostack, preserves_flags)) };
            a += line;
        }
        crate::mmio::barrier::dsb();
    }
    #[cfg(not(target_arch = "aarch64"))]
    {
        let _ = (ptr, len);
        crate::mmio::barrier::dsb();
    }
}

/// Clean and invalidate `[ptr, ptr + len)` to the point of coherency.
///
/// # Safety
/// The range must be mapped in this process.
pub unsafe fn clean_invalidate(ptr: *const u8, len: usize) {
    #[cfg(target_arch = "aarch64")]
    {
        let line = dcache_line();
        let mut a = (ptr as usize) & !(line - 1);
        let end = ptr as usize + len;
        while a < end {
            // SAFETY: caller guarantees the address is mapped.
            unsafe { core::arch::asm!("dc civac, {}", in(reg) a, options(nostack, preserves_flags)) };
            a += line;
        }
        crate::mmio::barrier::dsb();
    }
    #[cfg(not(target_arch = "aarch64"))]
    {
        let _ = (ptr, len);
        crate::mmio::barrier::dsb();
    }
}
