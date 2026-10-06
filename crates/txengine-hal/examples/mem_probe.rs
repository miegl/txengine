//! Probe DMA memory allocation strategies on the running board.

use txengine_hal::mem::mailbox::{Mailbox, VcMemory};
use txengine_hal::mem::pagemap::Pagemap;

fn main() {
    match Mailbox::open().and_then(|m| m.firmware_revision()) {
        Ok(rev) => println!("mailbox: firmware revision {rev:#x}"),
        Err(e) => println!("mailbox: {e}"),
    }
    match VcMemory::alloc(65536) {
        Ok(m) => println!("mailbox alloc: {m:?}"),
        Err(e) => println!("mailbox alloc failed: {e}"),
    }

    // io_uring buffer registration as a long-term page pin.
    let len = 8 * 16384usize;
    // SAFETY: anonymous mapping.
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
    assert_ne!(ptr, libc::MAP_FAILED);
    // SAFETY: valid mapping.
    unsafe {
        libc::madvise(ptr, len, libc::MADV_DONTFORK);
        std::ptr::write_bytes(ptr as *mut u8, 0x5a, len);
    }
    let mut params = [0u8; 120];
    // SAFETY: io_uring_setup with a zeroed params struct.
    let ring = unsafe { libc::syscall(libc::SYS_io_uring_setup, 1u32, params.as_mut_ptr()) };
    println!("io_uring_setup -> {ring}");
    let iov = libc::iovec { iov_base: ptr, iov_len: len };
    // SAFETY: registering one valid iovec.
    let rc = unsafe { libc::syscall(libc::SYS_io_uring_register, ring as i32, 0u32, &iov as *const _, 1u32) };
    println!("io_uring_register(BUFFERS) -> {rc} ({})", std::io::Error::last_os_error());

    let pm = Pagemap::open().unwrap();
    let ps = 16384;
    let before: Vec<u64> = (0..len / ps).map(|i| pm.phys(ptr as usize + i * ps).unwrap()).collect();
    println!("phys pages: {:x?}", before);
    // Ask the kernel to compact memory, which migrates movable pages.
    let _ = std::fs::write("/proc/sys/vm/compact_memory", "1");
    std::thread::sleep(std::time::Duration::from_millis(500));
    let after: Vec<u64> = (0..len / ps).map(|i| pm.phys(ptr as usize + i * ps).unwrap()).collect();
    println!("after compaction unchanged: {}", before == after);
}
