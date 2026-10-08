//! Volatile access to memory-mapped peripheral registers.

use std::fmt;
use std::sync::Arc;

use crate::sys::Mapping;

/// A window of memory-mapped registers.
///
/// Cloning is cheap; all clones share the underlying mapping. Accesses are
/// bounds- and alignment-checked (a wrong offset is a programming error and
/// panics instead of touching unrelated hardware).
#[derive(Clone)]
pub struct Mmio {
    map: Arc<Mapping>,
    offset: usize,
    len: usize,
    /// Address of this window in the device's own address map (for logs).
    pub device_base: u64,
}

impl fmt::Debug for Mmio {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Mmio({:#x}+{:#x})", self.device_base, self.len)
    }
}

impl Mmio {
    pub fn new(map: Mapping, device_base: u64) -> Self {
        let len = map.len();
        Mmio { map: Arc::new(map), offset: 0, len, device_base }
    }

    /// A sub-window starting at `offset` of `len` bytes.
    pub fn window(&self, offset: usize, len: usize) -> Mmio {
        assert!(
            offset.checked_add(len).is_some_and(|end| end <= self.len),
            "MMIO window {offset:#x}+{len:#x} outside {self:?}"
        );
        Mmio { map: self.map.clone(), offset: self.offset + offset, len, device_base: self.device_base + offset as u64 }
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    #[inline]
    fn ptr(&self, off: usize) -> *mut u32 {
        assert!(off.is_multiple_of(4) && off + 4 <= self.len, "register {off:#x} outside {self:?}");
        // SAFETY: bounds checked above; the mapping outlives self.
        unsafe { self.map.as_ptr().add(self.offset + off).cast() }
    }

    #[inline]
    pub fn read(&self, off: usize) -> u32 {
        // SAFETY: ptr() validated the offset; the mapping outlives self.
        unsafe { load32(self.ptr(off)) }
    }

    #[inline]
    pub fn write(&self, off: usize, val: u32) {
        // SAFETY: ptr() validated the offset; the mapping outlives self.
        unsafe { store32(self.ptr(off), val) }
    }

    /// Read-modify-write: clear `mask`, then set `val & mask`.
    #[inline]
    pub fn modify(&self, off: usize, mask: u32, val: u32) {
        let v = self.read(off);
        self.write(off, (v & !mask) | (val & mask));
    }

    /// Read a 64-bit register as two 32-bit accesses (low word first).
    #[inline]
    pub fn read64(&self, off: usize) -> u64 {
        let lo = self.read(off) as u64;
        let hi = self.read(off + 4) as u64;
        (hi << 32) | lo
    }

    /// Read a 64-bit register that may be changing (e.g. a DMA address):
    /// re-reads until the high word is stable across the low word read.
    #[inline]
    pub fn read64_consistent(&self, off: usize) -> u64 {
        loop {
            let hi1 = self.read(off + 4);
            let lo = self.read(off);
            let hi2 = self.read(off + 4);
            if hi1 == hi2 {
                return ((hi1 as u64) << 32) | lo as u64;
            }
        }
    }

    /// Write a 64-bit register as two 32-bit accesses (low word first).
    #[inline]
    pub fn write64(&self, off: usize, val: u64) {
        self.write(off, val as u32);
        self.write(off + 4, (val >> 32) as u32);
    }
}

/// 32-bit load from device memory through a general-purpose register.
///
/// A plain volatile read may be compiled into a load straight into a
/// floating-point register when the value is converted to a float next
/// (`ldr s0, [x8, #0xc4]`). On the BCM2837 (Cortex-A53; Pi 3, Zero 2 W)
/// such a load from a peripheral at an address ≡ 4 (mod 8) returns the word
/// 4 bytes below it: reading the frequency counter's result gave its control
/// register. Linux' `readl()` uses an explicit `ldr` for the same reason.
///
/// # Safety
/// `p` must be valid for a 4-byte aligned read.
#[inline(always)]
pub unsafe fn load32(p: *const u32) -> u32 {
    #[cfg(target_arch = "aarch64")]
    // SAFETY: the caller guarantees `p` is readable.
    unsafe {
        let v: u32;
        core::arch::asm!("ldr {v:w}, [{p}]", p = in(reg) p, v = out(reg) v, options(nostack, preserves_flags));
        v
    }
    #[cfg(target_arch = "arm")]
    // SAFETY: the caller guarantees `p` is readable.
    unsafe {
        let v: u32;
        core::arch::asm!("ldr {v}, [{p}]", p = in(reg) p, v = out(reg) v, options(nostack, preserves_flags));
        v
    }
    #[cfg(not(any(target_arch = "aarch64", target_arch = "arm")))]
    // SAFETY: the caller guarantees `p` is readable.
    unsafe {
        p.read_volatile()
    }
}

/// 32-bit store to device memory from a general-purpose register (see
/// [`load32`]; a floating-point store could likewise hit the wrong word).
///
/// # Safety
/// `p` must be valid for a 4-byte aligned write.
#[inline(always)]
pub unsafe fn store32(p: *mut u32, v: u32) {
    #[cfg(target_arch = "aarch64")]
    // SAFETY: the caller guarantees `p` is writable.
    unsafe {
        core::arch::asm!("str {v:w}, [{p}]", p = in(reg) p, v = in(reg) v, options(nostack, preserves_flags));
    }
    #[cfg(target_arch = "arm")]
    // SAFETY: the caller guarantees `p` is writable.
    unsafe {
        core::arch::asm!("str {v}, [{p}]", p = in(reg) p, v = in(reg) v, options(nostack, preserves_flags));
    }
    #[cfg(not(any(target_arch = "aarch64", target_arch = "arm")))]
    // SAFETY: the caller guarantees `p` is writable.
    unsafe {
        p.write_volatile(v)
    }
}

/// Memory barriers for ordering CPU accesses against DMA engines.
pub mod barrier {
    /// Full system data synchronisation barrier: all preceding memory
    /// accesses (including to non-cacheable DMA buffers and device
    /// registers) complete before any following access.
    #[inline]
    pub fn dsb() {
        #[cfg(target_arch = "aarch64")]
        // SAFETY: barrier instruction, no memory effects beyond ordering.
        unsafe {
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
        }
        #[cfg(target_arch = "arm")]
        // SAFETY: barrier instruction, available at PL0 on ARMv7.
        unsafe {
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
        }
        #[cfg(not(any(target_arch = "aarch64", target_arch = "arm")))]
        std::sync::atomic::fence(std::sync::atomic::Ordering::SeqCst);
    }
}
