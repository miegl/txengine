//! State shared between the USB side (core 0) and the RF engine (core 1).
//!
//! Only 32-bit atomic loads and stores are used (the RP2040's Cortex-M0+
//! has no atomic read-modify-write); 64-bit values are published with a
//! sequence counter.

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicU8, AtomicU32, Ordering, fence};

use txengine_pico_protocol::{Config, Fault, PioConfig, Retune, State};

/// Capacity of the sample buffer (samples, a power of two).
pub const RING_SAMPLES: usize = 1 << 15;

/// Samples from the host: written by core 0, read by core 1. The indices
/// run freely and wrap at 2^32.
pub struct SampleRing {
    buf: UnsafeCell<[i16; RING_SAMPLES]>,
    head: AtomicU32,
    tail: AtomicU32,
}

// SAFETY: one producer writes slots outside [tail, head) and then publishes
// them by advancing `head`; one consumer reads slots inside it and then
// releases them by advancing `tail`.
unsafe impl Sync for SampleRing {}

impl SampleRing {
    const fn new() -> SampleRing {
        SampleRing { buf: UnsafeCell::new([0; RING_SAMPLES]), head: AtomicU32::new(0), tail: AtomicU32::new(0) }
    }

    /// Free slots (producer side).
    pub fn free(&self) -> usize {
        RING_SAMPLES - self.head.load(Ordering::Relaxed).wrapping_sub(self.tail.load(Ordering::Acquire)) as usize
    }

    /// Append samples given as little-endian bytes; `bytes.len() / 2` must
    /// not exceed [`SampleRing::free`]. Producer only.
    pub fn push_le(&self, bytes: &[u8]) {
        let head = self.head.load(Ordering::Relaxed);
        let buf = self.buf.get() as *mut i16;
        for (i, b) in bytes.chunks_exact(2).enumerate() {
            let at = head.wrapping_add(i as u32) as usize % RING_SAMPLES;
            // SAFETY: the slot is free (see `free`), so the consumer does not read it.
            unsafe { buf.add(at).write_volatile(i16::from_le_bytes([b[0], b[1]])) };
        }
        self.head.store(head.wrapping_add((bytes.len() / 2) as u32), Ordering::Release);
    }

    /// Take the oldest sample. Consumer only.
    #[inline(always)]
    pub fn pop(&self) -> Option<i16> {
        let tail = self.tail.load(Ordering::Relaxed);
        if tail == self.head.load(Ordering::Acquire) {
            return None;
        }
        // SAFETY: the slot was published by the producer and is not reused
        // before `tail` moves past it.
        let s = unsafe { (self.buf.get() as *const i16).add(tail as usize % RING_SAMPLES).read_volatile() };
        self.tail.store(tail.wrapping_add(1), Ordering::Release);
        Some(s)
    }

    /// Discard everything. Only while the consumer is idle.
    pub fn clear(&self) {
        self.tail.store(self.head.load(Ordering::Relaxed), Ordering::Release);
    }
}

/// A value written by one core and read by the other, guarded by a
/// sequence counter (odd while being written).
pub struct Published<T: Copy> {
    seq: AtomicU32,
    value: UnsafeCell<T>,
}

// SAFETY: a single writer; readers retry until they see the same even
// sequence number before and after reading.
unsafe impl<T: Copy + Send> Sync for Published<T> {}

impl<T: Copy> Published<T> {
    const fn new(v: T) -> Published<T> {
        Published { seq: AtomicU32::new(0), value: UnsafeCell::new(v) }
    }

    /// Writer side.
    #[inline(always)]
    pub fn set(&self, v: T) {
        let s = self.seq.load(Ordering::Relaxed);
        self.seq.store(s.wrapping_add(1), Ordering::Relaxed);
        fence(Ordering::Release);
        // SAFETY: single writer; readers detect the concurrent write.
        unsafe { self.value.get().write_volatile(v) };
        fence(Ordering::Release);
        self.seq.store(s.wrapping_add(2), Ordering::Release);
    }

    /// Reader side.
    pub fn get(&self) -> T {
        loop {
            let s0 = self.seq.load(Ordering::Acquire);
            // SAFETY: a torn read is discarded below.
            let v = unsafe { self.value.get().read_volatile() };
            fence(Ordering::Acquire);
            if s0 & 1 == 0 && self.seq.load(Ordering::Relaxed) == s0 {
                return v;
            }
        }
    }

    /// Sequence number, which changes with every [`Published::set`].
    #[inline(always)]
    pub fn version(&self) -> u32 {
        self.seq.load(Ordering::Acquire)
    }
}

/// What core 0 asks core 1 to do.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Command {
    None = 0,
    Start = 1,
    Stop = 2,
}

/// How the carrier is made.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Method {
    Pll = 0,
    Pio = 1,
}

/// Progress reported by the RF engine.
#[derive(Debug, Clone, Copy)]
pub struct Progress {
    pub consumed: u64,
    pub underruns: u32,
}

pub struct Shared {
    pub samples: SampleRing,
    /// Samples received since the last stop.
    pub received: Published<u64>,
    /// Written by core 0 while the engine is idle, with `method`.
    pub config: Published<Config>,
    pub pio_config: Published<PioConfig>,
    method: AtomicU8,
    /// Written by core 0 at any time; the engine applies it at the next sample.
    pub retune: Published<Retune>,
    command: AtomicU8,
    state: AtomicU8,
    fault: AtomicU8,
    pub progress: Published<Progress>,
    /// Smallest lead of the modulator (updates) since core 0 last reset it to `u32::MAX`.
    pub min_lead: AtomicU32,
}

pub static SHARED: Shared = Shared {
    samples: SampleRing::new(),
    received: Published::new(0),
    config: Published::new(Config {
        refdiv: 1,
        post_div1: 1,
        post_div2: 1,
        gpout_div: 1,
        gpio: 0,
        drive_ma: 2,
        base: 0,
        update_div: 1,
        sample_updates: 6,
        interpolate: false,
        center: 0,
        scale: 0,
        phase: 0,
    }),
    pio_config: Published::new(PioConfig {
        refdiv: 1,
        fbdiv: 0,
        post_div1: 1,
        post_div2: 1,
        gpio: 0,
        drive_ma: 2,
        ratio: 0,
        center: 0,
        scale: 0,
        sample_words: 1,
        interpolate: false,
    }),
    method: AtomicU8::new(Method::Pll as u8),
    retune: Published::new(Retune { center: 0, scale: 0 }),
    command: AtomicU8::new(Command::None as u8),
    state: AtomicU8::new(State::Idle as u8),
    fault: AtomicU8::new(Fault::None as u8),
    progress: Published::new(Progress { consumed: 0, underruns: 0 }),
    min_lead: AtomicU32::new(u32::MAX),
};

impl Shared {
    pub fn command(&self) -> Command {
        match self.command.load(Ordering::Acquire) {
            1 => Command::Start,
            2 => Command::Stop,
            _ => Command::None,
        }
    }

    /// Ask the engine to do something and wake it.
    pub fn send(&self, c: Command) {
        self.command.store(c as u8, Ordering::Release);
        cortex_m::asm::sev();
    }

    /// The engine took the command.
    pub fn take_command(&self) {
        self.command.store(Command::None as u8, Ordering::Release);
    }

    pub fn method(&self) -> Method {
        if self.method.load(Ordering::Acquire) == Method::Pio as u8 { Method::Pio } else { Method::Pll }
    }

    pub fn set_method(&self, m: Method) {
        self.method.store(m as u8, Ordering::Release);
    }

    pub fn state(&self) -> State {
        match self.state.load(Ordering::Acquire) {
            1 => State::Configured,
            2 => State::Running,
            3 => State::Fault,
            _ => State::Idle,
        }
    }

    pub fn set_state(&self, s: State) {
        self.state.store(s as u8, Ordering::Release);
    }

    pub fn fault(&self) -> Fault {
        Fault::from_u8(self.fault.load(Ordering::Acquire))
    }

    pub fn set_fault(&self, f: Fault) {
        self.fault.store(f as u8, Ordering::Release);
    }
}
