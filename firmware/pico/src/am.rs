//! AM: one pulse per carrier cycle from a PIO state machine, whose width
//! sets the carrier's amplitude, at the raised system clock ([`clock`]).
//! Core 1 runs the quantiser of `txengine_pico_protocol::am`, which places
//! every pulse's edges on clock cycles with noise shaping in the phasor
//! domain, and writes one word per pulse to a ring that DMA feeds to the
//! state machine:
//!
//! ```text
//!  samples ─► core 1 (quantiser) ─► word ring ──DMA──► PIO TX FIFO ─► state machine ─► GPIO
//! ```
//!
//! The state machine drives the pin by side-set: low for `x + 4` cycles,
//! then high for `y` (none for `y = 0`), from a word `y << 16 | x`:
//!
//! ```text
//!  hi:  jmp y-- hi   side 1
//!  top: out x, 16    side 0   ; ◄ wrap target
//!       out y, 16    side 0
//!  lo:  jmp x-- lo   side 0
//!       jmp y-- hi   side 0   ; y = 0: on to the next word (wrap)
//! ```

use core::ptr::{addr_of, addr_of_mut};

use embassy_rp::pac;
use txengine_pico_protocol::am::{self, Pulses, SIN_LEN, Tables};
use txengine_pico_protocol::{AmConfig, Fault};

use crate::clock;
use crate::rf::{self, FUNCSEL_NULL, FUNCSEL_PIO0, SM};
use crate::shared::{Command, Progress, SHARED};

/// What core 1 needs (system clock cycles, with 10 % to spare): per pulse
/// (carrier cycle), and per sample on top, measured as the highest carriers
/// each chip keeps up with at 50, 100 and 200 kS/s: about 120 and 280 on the
/// RP2040 (the assembly kernel), 131 and 157 on the RP2350 (the compiled
/// Rust, which keeps part of the state on the stack).
pub const PULSE_CYCLES: u16 = if cfg!(feature = "rp2040") { 132 } else { 144 };
pub const SAMPLE_CYCLES: u16 = if cfg!(feature = "rp2040") { 310 } else { 173 };

/// Words (pulses) in the ring: the first half of `rf::WORD_RING`, 4 ms at
/// 1 MHz (all of it would hold most of the latency on long wave).
const RING_WORDS: usize = rf::WORD_RING_WORDS / 2;

/// What the quantiser looks up, in RAM (core 1 reads it for every pulse):
/// the period's tables, and the sine table: as it is for the Rust
/// quantiser, packed as (sin, cos) pairs for the assembly one.
static mut TABLES: Tables = Tables::new();
#[cfg(feature = "rp2350")]
static mut SIN: [i16; SIN_LEN] = [0; SIN_LEN];
#[cfg(feature = "rp2040")]
static mut SINCOS: [u32; SIN_LEN] = [0; SIN_LEN];

/// The quantiser's state and where its tables are (`#[repr(C)]`: the
/// assembly kernel's view of it).
#[repr(C)]
struct Kernel {
    p: Pulses,
    choice: *const [i32; 2],
    sincos: *const u32,
}

// The offsets the assembly kernel uses.
const _: () = {
    use core::mem::offset_of;
    assert!(offset_of!(Pulses, e1r) == 0 && offset_of!(Pulses, e1i) == 4);
    assert!(offset_of!(Pulses, e2r) == 8 && offset_of!(Pulses, e2i) == 12);
    assert!(offset_of!(Pulses, a) == 16 && offset_of!(Pulses, da) == 20);
    assert!(offset_of!(Pulses, phase) == 24 && offset_of!(Pulses, centre) == 28);
    assert!(offset_of!(Pulses, k) == 32 && offset_of!(Pulses, period_q4) == 36);
    assert!(offset_of!(Pulses, max_shift) == 40 && offset_of!(Pulses, emax) == 44);
    assert!(offset_of!(Kernel, choice) == 52);
};

// The quantiser of `txengine_pico_protocol::am::Pulses::pulse` for the
// Cortex-M0+, which the compiler spills to the stack all over (about 280
// cycles a pulse with the per-sample work). `txengine_am_pulses(k, out, n)`
// writes `n` (at least one) pulses' words to `out`. Runs from RAM.
//
// It uses the core's interpolators (set up by `setup_interp`), which sit on
// the single-cycle I/O port: interpolator 0 turns a phase into the address
// of its sine table entry (lane 0) and the phase past the entry (lane 1),
// and the spare registers of both hold what the loop keeps, read and
// written in one cycle instead of two: the last two errors (interpolator 1
// ACCUM0, ACCUM1, BASE0, BASE1), the amplitude step (its BASE2), the period
// (interpolator 0 ACCUM1) and the constant 3217 (its BASE2).
//
// Registers across the loop: r7 the interpolators, r8 the amplitude, r9 the
// choice table, r10 the phase, r11 k, r12 the centre, lr the next word; the
// end of the words and the kernel on the stack. The common path chooses
// the pulse from |v| and the quadrature error; two rare ones (v pointing
// backwards, |v| beyond the full output) join a copy of the rest that
// bounds the errors.
#[cfg(feature = "rp2040")]
core::arch::global_asm!(
    // (sin, cos) of the phase \p, into \s and \c; \p, \t and \b are clobbered.
    ".macro SINCOS p, s, c, t, b",
    "    str \\p, [r7, #0x00]", // interpolator 0 ACCUM0
    "    ldr \\t, [r7, #0x20]", // PEEK_LANE0: the entry's address
    "    ldr \\b, [r7, #0x24]", // PEEK_LANE1: the phase past it (16 bits)
    "    ldr \\t, [\\t]",
    "    sxth \\s, \\t",
    "    asrs \\c, \\t, #16",
    "    ldr \\t, [r7, #0x10]", // 3217
    "    muls \\b, \\t",
    "    lsrs \\b, \\b, #13", // in radians (Q24)
    "    movs \\t, \\c",
    "    muls \\t, \\b",
    "    asrs \\t, \\t, #24",
    "    adds \\s, \\s, \\t",
    "    muls \\b, \\s",
    "    asrs \\b, \\b, #24",
    "    subs \\c, \\c, \\b",
    ".endm",
    // The rest of a pulse: from the width (Q8) in r1 and the shift in r0,
    // with vr in r4 and vi in r5, to the next one's centre.
    ".macro TAIL bound",
    "    adds r1, #128",
    "    asrs r1, r1, #8", // w
    "    lsls r2, r1, #7",
    "    subs r0, r0, r2",
    "    add r0, r12",
    "    adds r0, #128",
    "    asrs r0, r0, #8", // r
    "    cmp r0, #4",
    "    bge 1f",
    "    movs r0, #4",
    "1:",
    "    lsls r2, r1, #16",
    "    adds r2, r2, r0",
    "    subs r2, #4", // the word
    "    mov r3, lr",
    "    stmia r3!, {{r2}}",
    "    mov lr, r3",
    "    mov r2, r11",
    "    muls r0, r2",     // r k
    "    muls r1, r2",     // w k
    "    add r0, r10",     // ar
    "    adds r1, r1, r0", // af
    "    mov r10, r1",
    "    SINCOS r0, r2, r3, r1, r6",
    "    adds r4, r4, r2", // vr + sr
    "    adds r5, r5, r3", // vi + cr
    "    mov r0, r10",
    "    SINCOS r0, r2, r3, r1, r6",
    "    subs r4, r2, r4", // er
    "    subs r5, r3, r5", // ei
    ".if \\bound",
    "    ldr r0, [sp, #4]",
    "    ldr r0, [r0, #44]", // emax
    "    negs r1, r0",
    "    cmp r4, r0",
    "    ble 2f",
    "    mov r4, r0",
    "2:",
    "    cmp r4, r1",
    "    bge 3f",
    "    mov r4, r1",
    "3:",
    "    cmp r5, r0",
    "    ble 4f",
    "    mov r5, r0",
    "4:",
    "    cmp r5, r1",
    "    bge 5f",
    "    mov r5, r1",
    "5:",
    ".endif",
    "    str r4, [r7, #0x40]", // e1r
    "    str r5, [r7, #0x44]", // e1i
    // The next centre: ((2^15 - (af >> 17)) period_q4) >> 11.
    "    mov r0, r10",
    "    asrs r0, r0, #17",
    "    movs r1, #1",
    "    lsls r1, r1, #15",
    "    subs r1, r1, r0",
    "    ldr r0, [r7, #0x04]", // period_q4
    "    muls r1, r0",
    "    lsrs r1, r1, #11",
    "    mov r12, r1",
    ".endm",
    ".section .data.txengine_am_pulses,\"ax\",%progbits",
    ".global txengine_am_pulses",
    ".type txengine_am_pulses,%function",
    ".thumb_func",
    ".p2align 2",
    "txengine_am_pulses:",
    "    push {{r4-r7, lr}}",
    "    mov r3, r8",
    "    mov r4, r9",
    "    mov r5, r10",
    "    mov r6, r11",
    "    push {{r3-r6}}",
    "    sub sp, #8",
    "    str r0, [sp, #4]", // the kernel
    "    lsls r2, r2, #2",
    "    adds r2, r1, r2",
    "    str r2, [sp, #0]", // the end
    "    mov lr, r1",
    "    ldr r7, =0xd0000080", // the interpolators
    "    ldm r0!, {{r1-r6}}",  // e1r e1i e2r e2i a da
    "    str r1, [r7, #0x40]",
    "    str r2, [r7, #0x44]",
    "    str r3, [r7, #0x48]",
    "    str r4, [r7, #0x4c]",
    "    mov r8, r5",
    "    str r6, [r7, #0x50]",
    "    ldm r0!, {{r1-r4}}", // phase centre k period_q4
    "    mov r10, r1",
    "    mov r12, r2",
    "    mov r11, r3",
    "    str r4, [r7, #0x04]",
    "    ldr r0, [sp, #4]",
    "    ldr r0, [r0, #52]", // choice
    "    mov r9, r0",
    "    b 20f",
    // v points backwards: a pulse a quarter period off, for vi alone.
    "10:",
    "    asrs r1, r5, #31",
    "    adds r0, r5, r1",
    "    eors r0, r1",
    "    adds r0, #32",
    "    asrs r0, r0, #6",
    "    movs r1, #1",
    "    lsls r1, r1, #10",
    "    cmp r0, r1",
    "    ble 11f",
    "    mov r0, r1",
    "11:",
    "    mov r1, r9",
    "    lsls r0, r0, #3",
    "    ldr r1, [r1, r0]", // width
    "    ldr r0, [sp, #4]",
    "    ldr r0, [r0, #40]", // max_shift
    "    cmp r5, #0",
    "    ble 12f",
    "    negs r0, r0",
    "12:",
    "    b 30f",
    // |v| beyond the full output.
    "13:",
    "    mov r1, r9",
    "    movs r0, #1",
    "    lsls r0, r0, #13",
    "    adds r1, r1, r0",
    "    ldr r0, [r1, #4]",
    "    ldr r1, [r1, #0]",
    "    asrs r2, r5, #4",
    "    muls r0, r2",
    "    asrs r0, r0, #8",
    "    b 30f",
    // |vi| ≥ vr: |v| ≈ |vi| + 3/8 vr.
    "14:",
    "    lsrs r1, r4, #2",
    "    lsrs r2, r4, #3",
    "    adds r0, r0, r1",
    "    adds r0, r0, r2",
    "    b 22f",
    // The loop.
    "20:",
    "    ldr r0, [r7, #0x50]",
    "    add r8, r0",          // a += da
    "    ldr r0, [r7, #0x40]", // e1r
    "    ldr r1, [r7, #0x44]", // e1i
    "    ldr r2, [r7, #0x48]", // e2r
    "    ldr r3, [r7, #0x4c]", // e2i
    "    str r0, [r7, #0x48]",
    "    str r1, [r7, #0x4c]", // e2 = e1
    "    mov r4, r8",
    "    asrs r4, r4, #8",
    "    adds r4, r4, r2",
    "    lsls r0, r0, #1",
    "    subs r4, r4, r0", // vr
    "    lsls r1, r1, #1",
    "    subs r5, r3, r1", // vi
    "    cmp r4, #0",
    "    ble 10b",
    "    asrs r1, r5, #31",
    "    adds r0, r5, r1",
    "    eors r0, r1", // y = |vi|
    "    cmp r4, r0",
    "    ble 14b",
    "    lsrs r1, r0, #2",
    "    lsrs r0, r0, #3",
    "    adds r0, r0, r1",
    "    adds r0, r0, r4", // vr + 3/8 y
    "22:",
    "    adds r0, #32",
    "    asrs r0, r0, #6", // i
    "    lsrs r1, r0, #10",
    "    bne 13b",
    "    mov r1, r9",
    "    lsls r0, r0, #3",
    "    adds r1, r1, r0",
    "    ldr r0, [r1, #4]",
    "    ldr r1, [r1, #0]", // width
    "    asrs r2, r5, #4",
    "    muls r0, r2",
    "    asrs r0, r0, #8", // shift
    "    TAIL 0",
    "40:",
    "    ldr r0, [sp, #0]",
    "    cmp r0, lr",
    "    beq 41f",
    "    b 20b",
    "41:",
    "    ldr r0, [sp, #4]",
    "    ldr r1, [r7, #0x40]",
    "    ldr r2, [r7, #0x44]",
    "    ldr r3, [r7, #0x48]",
    "    ldr r4, [r7, #0x4c]",
    "    mov r5, r8",
    "    stmia r0!, {{r1-r5}}", // e1r e1i e2r e2i a
    "    adds r0, #4",
    "    mov r1, r10",
    "    mov r2, r12",
    "    stmia r0!, {{r1, r2}}", // phase centre
    "    add sp, #8",
    "    pop {{r3-r6}}",
    "    mov r8, r3",
    "    mov r9, r4",
    "    mov r10, r5",
    "    mov r11, r6",
    "    pop {{r4-r7, pc}}",
    "30:",
    "    TAIL 1",
    "    b 40b",
    ".ltorg",
    ".size txengine_am_pulses, . - txengine_am_pulses",
);

/// Set up the calling core's interpolators for `txengine_am_pulses`.
#[cfg(feature = "rp2040")]
fn setup_interp(sincos: *const u32) {
    let i0 = pac::SIO.interp(0);
    // Lane 0: (p >> 18) & 0x3ffc, the entry's offset, plus the table.
    i0.ctrl_lane0().write_value(pac::sio::regs::Interp1ctrlLane0(18 | 2 << 5 | 13 << 10));
    // Lane 1, from ACCUM0 as well: (p >> 4) & 0xffff.
    i0.ctrl_lane1().write_value(pac::sio::regs::Interp1ctrlLane1(4 | 15 << 10 | 1 << 16));
    i0.base0().write_value(sincos as u32);
    i0.base1().write_value(0);
    i0.base2().write_value(3217);
    let i1 = pac::SIO.interp(1);
    i1.ctrl_lane0().write_value(pac::sio::regs::Interp1ctrlLane0(0));
    i1.ctrl_lane1().write_value(pac::sio::regs::Interp1ctrlLane1(0));
}

#[cfg(feature = "rp2040")]
unsafe extern "C" {
    fn txengine_am_pulses(k: *mut Kernel, out: *mut u32, n: u32);
}

impl Kernel {
    /// Write `out.len()` pulses.
    #[inline(always)]
    fn pulses(&mut self, out: &mut [u32]) {
        #[cfg(feature = "rp2040")]
        // SAFETY: the kernel writes exactly `out.len()` (at least one)
        // words, and reads its tables (`choice` with WIDTH_LEN pairs,
        // `sincos` with SIN_LEN words).
        unsafe {
            txengine_am_pulses(self, out.as_mut_ptr(), out.len() as u32)
        };
        #[cfg(feature = "rp2350")]
        // SAFETY: only core 1 uses the tables.
        self.pulses_portable(unsafe { &*addr_of!(SIN) }, out);
    }

    /// The same in Rust.
    #[inline(always)]
    fn pulses_portable(&mut self, sin: &[i16; SIN_LEN], out: &mut [u32]) {
        // SAFETY: only core 1 uses the tables.
        let tables = unsafe { &*addr_of!(TABLES) };
        for o in out {
            *o = self.p.pulse(tables, sin);
        }
    }
}

/// Fill the tables for a carrier of `k`.
fn fill_tables(k: u32) {
    // SAFETY: only core 1 uses the tables, and it is idle here (or core 1
    // does not run yet).
    unsafe {
        (*addr_of_mut!(TABLES)).compute(am::period_q4(k));
        #[cfg(feature = "rp2350")]
        (*addr_of_mut!(SIN)).copy_from_slice(&am::SIN);
        #[cfg(feature = "rp2040")]
        for (i, e) in (*addr_of_mut!(SINCOS)).iter_mut().enumerate() {
            let c = am::SIN[(i + SIN_LEN / 4) % SIN_LEN];
            *e = (c as u32) << 16 | am::SIN[i] as u16 as u32;
        }
    }
}

impl Kernel {
    fn new(k: u32) -> Kernel {
        // SAFETY: only the address is taken.
        let choice = unsafe { addr_of!(TABLES.choice) } as *const [i32; 2];
        #[cfg(feature = "rp2040")]
        let sincos = addr_of!(SINCOS) as *const u32;
        #[cfg(feature = "rp2350")]
        let sincos = core::ptr::null();
        Kernel { p: Pulses::new(k), choice, sincos }
    }
}

/// Check the assembly kernel against the Rust quantiser over carriers,
/// amplitudes and transitions. Before core 1 starts (it fills the tables).
#[cfg(feature = "rp2040")]
pub fn self_test() -> bool {
    let mut ok = true;
    // Periods of 64.2, 252.3 and 1647.3 cycles.
    for k in [66_899_000u32, 17_022_000, 2_607_000] {
        fill_tables(k);
        let (mut a, mut b) = (Kernel::new(k), Kernel::new(k));
        setup_interp(a.sincos);
        let (mut wa, mut wb) = ([0u32; 8], [0u32; 8]);
        // SAFETY: core 1 does not run yet.
        let tables = unsafe { &*addr_of!(TABLES) };
        for (n, s) in [0u16, 0, 30_000, 65_535, 65_535, 34_000, 40_000, 2_000, 0, 1, 33_000].into_iter().enumerate() {
            let c = 1 + n as u32 % 8;
            a.p.steer(s, c, am::inverse(c), tables);
            b.p.steer(s, c, am::inverse(c), tables);
            a.pulses(&mut wa[..c as usize]);
            b.pulses_portable(&am::SIN, &mut wb[..c as usize]);
            ok &= wa == wb && a.p == b.p;
        }
    }
    ok
}

#[cfg(feature = "rp2350")]
pub fn self_test() -> bool {
    true
}

/// DMA channel (a ping-pong pair with the next one on the RP2040).
const CH: usize = 0;

/// Breadcrumbs (see `crate::crumb`).
const STEP: u32 = 0x414d_0000;

#[inline(always)]
fn crumb(step: u32) {
    crate::crumb(STEP | step);
}

/// The carrier period (clock cycles) of a configuration.
fn period(cfg: &AmConfig) -> u32 {
    am::period_q4(cfg.k) / 16
}

pub fn validate(cfg: &AmConfig) -> bool {
    clock::validate(&cfg.clock())
        && cfg.gpio < 30
        && matches!(cfg.drive_ma, 2 | 4 | 8 | 12)
        && cfg.k != 0
        && (am::MIN_PERIOD..=am::MAX_PERIOD).contains(&period(cfg))
        && cfg.cycles >= 1
        && cfg.cycles as usize <= RING_WORDS / 4
        && cfg.sustainable(PULSE_CYCLES, SAMPLE_CYCLES)
}

pub struct Transmitter {
    cfg: AmConfig,
    k: Kernel,
    /// `am::inverse(cycles)`.
    inv: i32,
    retune_version: u32,
    /// Words written to the ring, and read by the DMA (both wrapping).
    written: u32,
    read: u32,
    last_offset: u32,
    /// Samples taken from the host's buffer.
    taken: u64,
    underruns: u32,
    /// Consecutive samples the host did not deliver.
    missing: u32,
    /// The samples' recent mean (Q8), which an underrun holds.
    mean: u32,
    saved: Option<clock::Saved>,
}

impl Transmitter {
    pub fn start(cfg: &AmConfig) -> Result<Transmitter, Fault> {
        crumb(1);
        fill_tables(cfg.k);
        // A correction sent before the start counts from the start.
        let retune_version = SHARED.retune.version();
        let k = match SHARED.retune.get().center {
            0 => cfg.k,
            k => k,
        };
        let k = Kernel::new(k);
        #[cfg(feature = "rp2040")]
        setup_interp(k.sincos);
        let mut t = Transmitter {
            cfg: *cfg,
            k,
            inv: am::inverse(cfg.cycles as u32),
            retune_version,
            written: 0,
            read: 0,
            last_offset: 0,
            taken: 0,
            underruns: 0,
            missing: 0,
            mean: 0,
            saved: None,
        };
        // The samples sent ahead, from no carrier on (the ring holds at
        // most `RING_WORDS - 1` unread words); the mean if there are too few.
        let c = cfg.cycles as u32;
        while t.written + c < RING_WORDS as u32 {
            let s = match SHARED.samples.pop() {
                Some(s) => t.took(s as u16),
                None => (t.mean >> 8) as u16,
            };
            t.pulses(s);
        }
        SHARED.progress.set(Progress { consumed: 0, underruns: 0 });

        crumb(2);
        t.saved = Some(clock::raise(&cfg.clock())?);
        crumb(3);
        start_pio(cfg);
        // SAFETY: only the address is taken.
        let ring = unsafe { addr_of_mut!(rf::WORD_RING.0) } as u32;
        start_dma(ring);
        crumb(4);
        pac::PIO0.ctrl().modify(|w| w.set_sm_enable(1 << SM));
        crumb(5);
        Ok(t)
    }

    /// Count a sample taken from the host and follow its mean.
    #[inline(always)]
    fn took(&mut self, s: u16) -> u16 {
        self.taken += 1;
        self.missing = 0;
        // A time constant of 1024 samples (10 ms at 100 kS/s).
        self.mean = (self.mean as i32 + (((s as i32) << 8) - self.mean as i32) / 1024) as u32;
        s
    }

    /// The pulses of a sample into the ring.
    #[inline(always)]
    fn pulses(&mut self, s: u16) {
        let c = self.cfg.cycles as u32;
        // SAFETY: only core 1 uses the tables.
        self.k.p.steer(s, c, self.inv, unsafe { &*addr_of!(TABLES) });
        let mut left = c as usize;
        while left > 0 {
            // Up to the end of the ring, then on from its start.
            let at = self.written as usize % RING_WORDS;
            let n = left.min(RING_WORDS - at);
            // SAFETY: in bounds; core 1 is the only writer, and these words
            // were read by the DMA a ring ago (the caller keeps the lead
            // below a ring).
            let out =
                unsafe { core::slice::from_raw_parts_mut((addr_of_mut!(rf::WORD_RING.0) as *mut u32).add(at), n) };
            self.k.pulses(out);
            self.written = self.written.wrapping_add(n as u32);
            left -= n;
        }
    }

    /// One sample per iteration when there is room.
    #[unsafe(link_section = ".data.am_run")]
    #[inline(never)]
    pub fn run(&mut self) -> Result<(), Fault> {
        let c = self.cfg.cycles as u32;
        // Samples per second: the host is lost after that many missing.
        let host_lost = ((clock::hz(&self.cfg.clock()) as u64 * self.cfg.k as u64) >> 32) as u32 / c;
        let mut min_lead = u32::MAX;
        // Samples since the progress was last published.
        let mut since = 0u32;
        // The state machine stalls when its FIFO runs dry: a gap in the
        // carrier. Clear the flag the start left.
        pac::PIO0.fdebug().write(|w| w.set_txstall(1 << SM));
        crumb(6);
        loop {
            if pac::PIO0.fdebug().read().txstall() & (1 << SM) != 0 {
                return Err(Fault::Stall);
            }
            if let Some(offset) = read_offset() {
                self.read = self.read.wrapping_add(offset.wrapping_sub(self.last_offset) % RING_WORDS as u32);
                self.last_offset = offset;
            }
            let lead = self.written.wrapping_sub(self.read) as i32;
            if lead <= 0 {
                return Err(Fault::Late);
            }
            let lead = lead as u32;
            min_lead = min_lead.min(lead);

            let full = (RING_WORDS as u32 - 1).saturating_sub(lead) < c;
            if full || since >= 32 {
                // Publish (also while catching up, so that the host keeps
                // sending), and look at the commands.
                since = 0;
                if min_lead != u32::MAX {
                    let m = SHARED.min_lead.load(core::sync::atomic::Ordering::Relaxed);
                    SHARED.min_lead.store(m.min(min_lead), core::sync::atomic::Ordering::Relaxed);
                    min_lead = u32::MAX;
                }
                let consumed = self.taken.saturating_sub((lead / c) as u64);
                SHARED.progress.set(Progress { consumed, underruns: self.underruns });
                if SHARED.command() == Command::Stop {
                    return Ok(());
                }
                if full {
                    continue;
                }
            }
            since += 1;

            if SHARED.retune.version() != self.retune_version {
                self.retune_version = SHARED.retune.version();
                let k = SHARED.retune.get().center;
                if k != 0 {
                    self.k.p.retune(k);
                }
            }
            let s = match SHARED.samples.pop() {
                Some(s) => self.took(s as u16),
                None => {
                    // The host is late: hold the carrier at its recent mean.
                    if self.missing == 0 {
                        self.underruns += 1;
                    }
                    self.missing += 1;
                    if self.missing > host_lost {
                        return Err(Fault::HostLost);
                    }
                    (self.mean >> 8) as u16
                }
            };
            self.pulses(s);
        }
    }

    pub fn stop(&mut self) {
        crumb(20);
        pac::PIO0.ctrl().modify(|w| w.set_sm_enable(0));
        rf::abort_dma(&dma_channels());
        pac::IO_BANK0.gpio(self.cfg.gpio as usize).ctrl().write(|w| w.set_funcsel(FUNCSEL_NULL));
        if let Some(saved) = self.saved.take() {
            clock::restore(&saved);
        }
        crate::crumb(0);
    }
}

fn start_pio(cfg: &AmConfig) {
    rf::reset_pio0();
    let pio = pac::PIO0;
    let prg = pio::pio_asm!(
        ".side_set 1",
        "hi:",
        "    jmp y-- hi    side 1",
        ".wrap_target",
        "    out x, 16     side 0",
        "    out y, 16     side 0",
        "lo:",
        "    jmp x-- lo    side 0",
        "    jmp y-- hi    side 0",
        ".wrap",
    );
    for (i, &c) in prg.program.code.iter().enumerate() {
        pio.instr_mem(i).write_value(pac::pio::regs::InstrMem(c as u32));
    }
    let sm = pio.sm(SM);
    sm.clkdiv().write(|w| {
        w.set_int(1);
        w.set_frac(0);
    });
    sm.execctrl().write(|w| {
        w.set_wrap_bottom(prg.program.wrap.target);
        w.set_wrap_top(prg.program.wrap.source);
    });
    sm.shiftctrl().write(|w| {
        // x from the low half of a word, y from the high half.
        w.set_out_shiftdir(true);
        w.set_autopull(true);
        w.set_pull_thresh(0); // 32
        w.set_fjoin_tx(true);
    });
    sm.pinctrl().write(|w| {
        w.set_sideset_base(cfg.gpio);
        w.set_sideset_count(1);
        w.set_set_base(cfg.gpio);
        w.set_set_count(1);
    });
    let exec = |i: u16| sm.instr().write(|w| w.set_instr(i));
    exec(pio::InstructionOperands::SET { destination: pio::SetDestination::PINDIRS, data: 1 }.encode());
    exec(
        pio::InstructionOperands::JMP { condition: pio::JmpCondition::Always, address: prg.program.wrap.target }
            .encode(),
    );
    rf::configure_pad(cfg.gpio, cfg.drive_ma);
    pac::IO_BANK0.gpio(cfg.gpio as usize).ctrl().write(|w| w.set_funcsel(FUNCSEL_PIO0));
}

fn dma_channels() -> [usize; if cfg!(feature = "rp2040") { 2 } else { 1 }] {
    #[cfg(feature = "rp2040")]
    {
        [CH, CH + 1]
    }
    #[cfg(feature = "rp2350")]
    {
        [CH]
    }
}

fn start_dma(ring: u32) {
    let txf = pac::PIO0.txf(SM).as_ptr() as u32;
    let treq = pac::dma::vals::TreqSel::from_bits(SM as u8);
    rf::start_ring_dma(CH, ring, txf, treq, (RING_WORDS * 4).trailing_zeros() as u8, RING_WORDS as u32);
}

/// The DMA's position in the ring (words), if known right now.
#[inline(always)]
fn read_offset() -> Option<u32> {
    // SAFETY: only the address is taken.
    let ring = unsafe { addr_of_mut!(rf::WORD_RING.0) } as u32;
    rf::ring_read_offset(CH, ring, RING_WORDS as u32)
}
