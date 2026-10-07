//! Memory that peripheral DMA engines can access.
//!
//! Two providers exist:
//!
//! * [`mailbox::VcMemory`] – BCM283x/BCM2711 (Pi 1–4). Memory is allocated
//!   by the VideoCore firmware outside of Linux' RAM and accessed through an
//!   uncached `/dev/mem` mapping, so no cache maintenance is required.
//! * [`pinned::PinnedMemory`] – BCM2712 (Pi 5). Ordinary process memory,
//!   long-term pinned so it cannot migrate, used as a scatter-gather list of
//!   pages. The RP1 DMA engine reads it over PCIe, which is *not* cache
//!   coherent, so CPU writes are cleaned to the point of coherency explicitly.

pub mod cache;
pub mod mailbox;
pub mod pagemap;
pub mod pinned;

use std::fmt;

/// A buffer accessible by a DMA engine.
///
/// The buffer is physically contiguous within each aligned chunk of
/// [`DmaMemory::chunk_size`] bytes; structures the DMA engine reads in one
/// go must not cross a chunk boundary.
pub trait DmaMemory: Send + Sync + fmt::Debug {
    /// CPU virtual address of the buffer start.
    fn as_ptr(&self) -> *mut u8;
    /// Buffer length in bytes.
    fn len(&self) -> usize;
    fn is_empty(&self) -> bool {
        self.len() == 0
    }
    /// Address of the byte at `offset` as seen by the DMA engine.
    fn dma_addr_at(&self, offset: usize) -> u64;
    /// Granularity of physical contiguity (the whole length if contiguous).
    fn chunk_size(&self) -> usize;
    /// Make CPU writes to `[offset, offset + len)` visible to the device.
    /// Includes the necessary barrier.
    fn flush(&self, offset: usize, len: usize);
    /// Discard CPU cached copies of `[offset, offset + len)` so that data
    /// written by the device becomes visible to the CPU.
    fn invalidate(&self, offset: usize, len: usize);
    /// Short description for logs.
    fn describe(&self) -> String;
}

/// Typed, bounds-checked helpers on top of [`DmaMemory`].
pub trait DmaMemoryExt: DmaMemory {
    /// Write a `u32` at byte `offset` (volatile; no flush).
    fn write_u32(&self, offset: usize, v: u32) {
        assert!(offset.is_multiple_of(4) && offset + 4 <= self.len());
        // SAFETY: bounds and alignment checked; the buffer is owned by self.
        unsafe { self.as_ptr().add(offset).cast::<u32>().write_volatile(v) }
    }
    /// Read a `u32` at byte `offset` (volatile; no invalidate).
    fn read_u32(&self, offset: usize) -> u32 {
        assert!(offset.is_multiple_of(4) && offset + 4 <= self.len());
        // SAFETY: bounds and alignment checked; the buffer is owned by self.
        unsafe { self.as_ptr().add(offset).cast::<u32>().read_volatile() }
    }
    fn write_u64(&self, offset: usize, v: u64) {
        self.write_u32(offset, v as u32);
        self.write_u32(offset + 4, (v >> 32) as u32);
    }
    fn read_u64(&self, offset: usize) -> u64 {
        self.read_u32(offset) as u64 | (self.read_u32(offset + 4) as u64) << 32
    }
    /// Copy `src` into the buffer at byte `offset` (as `u32` words, no flush).
    ///
    /// One 32-bit store per word: a plain copy lets the compiler merge
    /// neighbouring words into wider stores at 4-byte aligned addresses, and
    /// unaligned stores fault on the Device memory type that AArch64 uses
    /// for uncached `/dev/mem` mappings (SIGBUS).
    fn write_words(&self, offset: usize, src: &[u32]) {
        assert!(offset.is_multiple_of(4) && offset + src.len() * 4 <= self.len());
        let dst = self.as_ptr().wrapping_add(offset).cast::<u32>();
        for (k, &w) in src.iter().enumerate() {
            // SAFETY: bounds and alignment checked above; the buffer is owned by self.
            unsafe { dst.add(k).write_volatile(w) }
        }
    }
    /// Fill `[offset, offset+len)` with zero bytes (no flush).
    fn zero(&self, offset: usize, len: usize) {
        assert!(offset + len <= self.len());
        // Byte-wise volatile stores: `memset` may use `DC ZVA`, which is not
        // permitted on non-cacheable mappings.
        for i in 0..len / 4 {
            self.write_u32(offset + i * 4, 0);
        }
    }
}

impl<T: DmaMemory + ?Sized> DmaMemoryExt for T {}
