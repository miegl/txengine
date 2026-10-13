//! The RF engine, which runs alone on core 1.
//!
//! The carrier is `PLL_USB` (the system and USB clocks run from `PLL_SYS`),
//! divided by its post dividers and a clock generator onto a GPIO. Its
//! feedback divider is the modulation input: a MASH 1-1 sigma-delta
//! modulator turns the wanted fractional divider into 5-bit codes, one per
//! update, packed six to a word into a ring in RAM. The pipeline is all
//! hardware from there:
//!
//! ```text
//!  code ring ──DMA──► PIO TX FIFO ─► state machine ─► RX FIFO ──DMA──► PLL_USB.FBDIV_INT
//!  (core 1 fills)      paced: one update every `update_div` reference cycles
//! ```
//!
//! The state machine takes a code, prepends the high bits of the divider
//! (`FBDIV = (high << 5) | code`) and pushes it, then waits out the rest
//! of the update period: its clock (`clk_sys`) is locked to the same
//! crystal as the PLL's reference, so the writes are evenly spaced in
//! reference cycles. The core only has to keep the ring ahead of the DMA.

use core::ptr::addr_of_mut;

use embassy_rp::pac;
use txengine_pico_protocol::sigma_delta::Mash11;
use txengine_pico_protocol::{Config, FRAC_BITS, Fault, State};

use crate::direct;
use crate::shared::{Command, Method, Progress, SHARED};

/// Words in the code ring (a power of two; 6 codes of 5 bits per word,
/// in its top 30 bits).
const RING_WORDS: usize = 1024;
const CODES_PER_WORD: u32 = Config::CODES_PER_WORD as u32;

#[repr(C, align(4096))]
struct CodeRing([u32; RING_WORDS]);

/// The DMA reads it as a ring: aligned to its size.
static mut CODES: CodeRing = CodeRing([0; RING_WORDS]);

/// DMA channels: codes → PIO, PIO → `FBDIV_INT`. On the RP2040, whose
/// transfer counts run out, each is a pair that chains to each other.
const CH_CODES: usize = 0;
const CH_FBDIV: usize = 2;
pub const SM: usize = 0;

/// System clock, set up in `main` (`PLL_SYS` 1440 MHz / 10).
pub const SYS_HZ: u32 = 144_000_000;
/// Nominal crystal.
pub const XOSC_HZ: u32 = 12_000_000;
/// Highest update rate: the PLL needs each FBDIV for four crystal cycles
/// ([`Config::MIN_UPDATE_XOSC_CYCLES`]). The kernels could compute twice that.
pub const MAX_UPDATE_HZ: u32 = XOSC_HZ / Config::MIN_UPDATE_XOSC_CYCLES;

/// The modulator's state, as the assembly kernels see it.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Kernel {
    x: u32,
    dx: i32,
    acc1: u32,
    acc2: u32,
    c2_prev: u32,
    bias: u32,
}

// The MASH 1-1 of `txengine_pico_protocol::sigma_delta` for the Cortex-M0+,
// which the compiler spills to the stack: everything in registers, 15
// cycles per update. `txengine_mash_words(k, out, words)` writes `words`
// words of six 5-bit codes each to `out`. Runs from RAM.
#[cfg(feature = "rp2040")]
core::arch::global_asm!(
    ".section .data.txengine_mash_words,\"ax\",%progbits",
    ".global txengine_mash_words",
    ".type txengine_mash_words,%function",
    ".thumb_func",
    ".p2align 2",
    "txengine_mash_words:",
    "    push {{r4-r7, lr}}",
    "    mov r3, r8",
    "    mov r4, r9",
    "    mov r5, r10",
    "    mov r6, r11",
    "    push {{r3-r6}}",
    "    mov r10, r0", // state
    "    mov r11, r2", // words left
    "    mov r7, r1",  // out
    "    ldr r3, [r0, #4]",
    "    mov r8, r3", // dx
    "    ldr r3, [r0, #20]",
    "    mov r9, r3",        // bias
    "    ldr r1, [r0, #8]",  // acc1
    "    ldr r2, [r0, #12]", // acc2
    "    ldr r3, [r0, #16]", // c2_prev
    "    ldr r0, [r0, #0]",  // x
    "1:",
    ".rept 6",
    "    add r0, r8",       // x += dx
    "    adds r5, r1, r0",  // s1 = acc1 + x
    "    lsrs r6, r5, #30", // c1
    "    lsls r1, r5, #2",
    "    lsrs r1, r1, #2",  // acc1 = s1 mod 2^30
    "    adds r2, r2, r1",  // s2 = acc2 + acc1
    "    lsrs r5, r2, #30", // c2
    "    lsls r2, r2, #2",
    "    lsrs r2, r2, #2", // acc2 = s2 mod 2^30
    "    adds r6, r6, r5",
    "    subs r6, r6, r3", // c1 + c2 - c2_prev
    "    movs r3, r5",     // c2_prev = c2
    "    add r6, r9",      // + bias
    "    lsls r4, r4, #5",
    "    orrs r4, r6", // w = w << 5 | code
    ".endr",
    "    lsls r4, r4, #2", // codes in the top 30 bits
    "    stmia r7!, {{r4}}",
    "    mov r5, r11",
    "    subs r5, #1",
    "    mov r11, r5",
    "    bne 1b",
    "    mov r5, r10",
    "    str r0, [r5, #0]",
    "    str r1, [r5, #8]",
    "    str r2, [r5, #12]",
    "    str r3, [r5, #16]",
    "    pop {{r3-r6}}",
    "    mov r8, r3",
    "    mov r9, r4",
    "    mov r10, r5",
    "    mov r11, r6",
    "    pop {{r4-r7, pc}}",
    ".size txengine_mash_words, . - txengine_mash_words",
);

// The same for the Cortex-M33 (Thumb-2: three-operand arithmetic, shifted
// operands, `bic` with an immediate): 11 cycles per update. `c2_prev` is
// kept as `bias - c2_prev`.
#[cfg(feature = "rp2350")]
core::arch::global_asm!(
    ".section .data.txengine_mash_words,\"ax\",%progbits",
    ".global txengine_mash_words",
    ".type txengine_mash_words,%function",
    ".thumb_func",
    ".p2align 2",
    "txengine_mash_words:",
    "    push {{r4-r11, lr}}",
    "    mov r11, r0",       // state
    "    mov r10, r2",       // words left
    "    mov r7, r1",        // out
    "    ldr r8, [r0, #4]",  // dx
    "    ldr r9, [r0, #20]", // bias
    "    ldr r1, [r0, #8]",  // acc1
    "    ldr r2, [r0, #12]", // acc2
    "    ldr r3, [r0, #16]",
    "    sub r3, r9, r3",   // q = bias - c2_prev
    "    ldr r0, [r0, #0]", // x
    "1:",
    ".rept 6",
    "    add r0, r0, r8",          // x += dx
    "    add r5, r1, r0",          // s1 = acc1 + x
    "    lsrs r6, r5, #30",        // c1
    "    bic r1, r5, #0xc0000000", // acc1 = s1 mod 2^30
    "    add r2, r2, r1",          // s2 = acc2 + acc1
    "    lsrs r5, r2, #30",        // c2
    "    bic r2, r2, #0xc0000000", // acc2 = s2 mod 2^30
    "    add r6, r6, r3",          // c1 + bias - c2_prev
    "    add r6, r6, r5",          // + c2
    "    sub r3, r9, r5",          // q = bias - c2
    "    orr r4, r6, r4, lsl #5",  // w = w << 5 | code
    ".endr",
    "    lsl r4, r4, #2", // codes in the top 30 bits
    "    str r4, [r7], #4",
    "    subs r10, r10, #1",
    "    bne 1b",
    "    str r0, [r11, #0]",
    "    str r1, [r11, #8]",
    "    str r2, [r11, #12]",
    "    sub r3, r9, r3",
    "    str r3, [r11, #16]",
    "    pop {{r4-r11, pc}}",
    ".size txengine_mash_words, . - txengine_mash_words",
);

unsafe extern "C" {
    fn txengine_mash_words(k: *mut Kernel, out: *mut u32, words: u32);
}

impl Kernel {
    fn new(x: u32, bias: u32) -> Kernel {
        Kernel { x, dx: 0, acc1: 0, acc2: 0, c2_prev: 0, bias }
    }

    /// Write `out.len()` words of codes.
    #[inline(always)]
    fn words(&mut self, out: &mut [u32]) {
        // SAFETY: the kernel writes exactly `out.len()` words and reads `self`.
        unsafe { txengine_mash_words(self, out.as_mut_ptr(), out.len() as u32) };
    }

    /// The same in Rust, with [`Mash11`]'s arithmetic.
    #[inline(always)]
    fn words_portable(&mut self, out: &mut [u32]) {
        let mut m = Mash11::with_bias(self.bias);
        m.set_state(self.acc1, self.acc2, self.c2_prev);
        let mut x = self.x;
        for o in out {
            let mut w = 0u32;
            for _ in 0..CODES_PER_WORD {
                x = x.wrapping_add_signed(self.dx);
                w = (w << 5) | m.code(x);
            }
            *o = w << 2;
        }
        self.x = x;
        (self.acc1, self.acc2, self.c2_prev) = m.state();
    }
}

/// Check the assembly kernel against [`Mash11`]; a mismatch would
/// transmit garbage.
pub fn self_test() -> bool {
    let mut a = Kernel::new(0x5a5a_5a5a & Config::X_MAX, 3);
    let mut b = a;
    let (mut wa, mut wb) = ([0u32; 16], [0u32; 16]);
    for dx in [0, 12_345_678, -98_765] {
        a.dx = dx;
        b.dx = dx;
        a.words(&mut wa);
        b.words_portable(&mut wb);
        if wa != wb || a != b {
            return false;
        }
    }
    true
}

/// Without samples for this long, the host is gone (seconds).
const HOST_LOST_S: u32 = 1;

/// GPIOs with a clock output, and its number.
pub fn gpout_of(gpio: u8) -> Option<usize> {
    match gpio {
        21 => Some(0),
        23 => Some(1),
        24 => Some(2),
        25 => Some(3),
        #[cfg(feature = "rp2350")]
        13 => Some(0),
        #[cfg(feature = "rp2350")]
        15 => Some(1),
        _ => None,
    }
}

pub fn gpio_mask() -> u32 {
    (0..30).filter(|&g| gpout_of(g).is_some()).fold(0, |m, g| m | 1 << g)
}

/// The state machine's loop starts after the synchronisation.
const LOOP_START: u8 = 3;
/// Its padding nops (each delays up to 31 more cycles).
const PAD_SLOTS: usize = 4;
/// The pin that carries the crystal clock while the state machine
/// synchronises to it: the Pico's LED (GPIO 25), not on the header.
const SYNC_GPIO: usize = 25;
const SYNC_GPOUT: usize = 3;

/// GPIO function: clock output.
#[cfg(feature = "rp2040")]
const FUNCSEL_GPCK: u8 = 8;
#[cfg(feature = "rp2350")]
const FUNCSEL_GPCK: u8 = 9;
pub const FUNCSEL_NULL: u8 = 0x1f;

/// State machine cycles per update.
fn update_cycles(cfg: &Config) -> u32 {
    cfg.update_div as u32 * (SYS_HZ / (XOSC_HZ / cfg.refdiv as u32))
}

/// Check a configuration against the hardware's limits.
pub fn validate(cfg: &Config) -> bool {
    let fref = XOSC_HZ / cfg.refdiv.max(1) as u32;
    let vco = |n: u32| fref as u64 * n as u64;
    let codes_fit = Config::codes_fit(cfg.base);
    let base = cfg.base as u32;
    (1..=63).contains(&cfg.refdiv)
        && XOSC_HZ.is_multiple_of(cfg.refdiv as u32)
        && fref >= 5_000_000
        && SYS_HZ.is_multiple_of(fref)
        && (1..=7).contains(&cfg.post_div1)
        && (1..=7).contains(&cfg.post_div2)
        && cfg.gpout_div >= 1
        && gpout_of(cfg.gpio).is_some()
        && matches!(cfg.drive_ma, 2 | 4 | 8 | 12)
        && base >= 16
        && base + 4 <= 320
        && vco(base) >= 750_000_000
        && vco(base + 4) <= 1_600_000_000
        && codes_fit
        && cfg.update_div as u32 * cfg.refdiv as u32 >= Config::MIN_UPDATE_XOSC_CYCLES
        && fref / cfg.update_div as u32 <= MAX_UPDATE_HZ
        && cfg.phase < update_cycles(cfg).min(32) as u8
        && (3 + PAD_SLOTS as u32..=3 + 32 * PAD_SLOTS as u32).contains(&update_cycles(cfg))
        && cfg.sample_updates >= Config::CODES_PER_WORD
        && cfg.sample_updates.is_multiple_of(Config::CODES_PER_WORD)
        && cfg.center <= Config::X_MAX
        && cfg.scale <= Config::SCALE_MAX
}

/// Core 1: wait for commands, transmit.
pub fn run() -> ! {
    loop {
        match SHARED.command() {
            Command::Start => {
                SHARED.take_command();
                if SHARED.state() != State::Configured {
                    SHARED.set_fault(Fault::BadConfig);
                    SHARED.set_state(State::Fault);
                    continue;
                }
                let end = match SHARED.method() {
                    Method::Pll => {
                        let cfg = SHARED.config.get();
                        transmit(validate(&cfg), || Transmitter::start(&cfg), |t| t.run(), |t| t.stop())
                    }
                    Method::Pio => {
                        let cfg = SHARED.pio_config.get();
                        transmit(direct::validate(&cfg), || direct::Transmitter::start(&cfg), |t| t.run(), |t| t.stop())
                    }
                };
                match end {
                    Ok(()) => SHARED.set_state(State::Idle),
                    Err(f) => {
                        SHARED.set_fault(f);
                        SHARED.set_state(State::Fault);
                    }
                }
            }
            Command::Stop => {
                SHARED.take_command();
                SHARED.set_fault(Fault::None);
                SHARED.set_state(State::Idle);
            }
            Command::None => cortex_m::asm::wfe(),
        }
    }
}

/// Start a transmitter, run it until stopped, and stop it.
fn transmit<T>(
    valid: bool,
    start: impl FnOnce() -> Result<T, Fault>,
    run: impl FnOnce(&mut T) -> Result<(), Fault>,
    stop: impl FnOnce(&mut T),
) -> Result<(), Fault> {
    if !valid {
        return Err(Fault::BadConfig);
    }
    let mut t = start()?;
    SHARED.set_state(State::Running);
    let end = run(&mut t);
    stop(&mut t);
    end
}

pub fn reset_pll_usb() {
    pac::RESETS.reset().modify(|w| w.set_pll_usb(true));
    pac::RESETS.reset().modify(|w| w.set_pll_usb(false));
    while !pac::RESETS.reset_done().read().pll_usb() {}
}

pub fn reset_pio0() {
    pac::RESETS.reset().modify(|w| w.set_pio0(true));
    pac::RESETS.reset().modify(|w| w.set_pio0(false));
    while !pac::RESETS.reset_done().read().pio0() {}
}

/// Switch the carrier off: everything `start` touched, in reverse.
pub fn output_off(gpio: Option<u8>) {
    if let Some(g) = gpio {
        pac::IO_BANK0.gpio(g as usize).ctrl().write(|w| w.set_funcsel(FUNCSEL_NULL));
    }
    for n in 0..4 {
        pac::CLOCKS.clk_gpout_ctrl(n).modify(|w| {
            if w.auxsrc() == pac::clocks::vals::ClkGpoutCtrlAuxsrc::CLKSRC_PLL_USB {
                w.set_enable(false);
            }
        });
    }
    pac::PLL_USB.pwr().write(|w| {
        w.set_pd(true);
        w.set_vcopd(true);
        w.set_postdivpd(true);
        w.set_dsmpd(true);
    });
}

struct Transmitter {
    cfg: Config,
    /// The modulator; its `x` is the input at the end of the last sample.
    k: Kernel,
    center: u32,
    scale: u32,
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
}

impl Transmitter {
    fn start(cfg: &Config) -> Result<Transmitter, Fault> {
        // A correction sent before the start counts from the start.
        let retune_version = SHARED.retune.version();
        let r = SHARED.retune.get();
        let (center, scale) = (r.center.min(Config::X_MAX), r.scale.min(Config::SCALE_MAX));
        let mut t = Transmitter {
            cfg: *cfg,
            k: Kernel::new(center | 1, 1 + (cfg.base & 0x1f) as u32),
            center,
            scale,
            retune_version,
            written: 0,
            read: 0,
            last_offset: 0,
            taken: 0,
            underruns: 0,
            missing: 0,
        };
        // The carrier at its centre until the first samples are read; the
        // ring holds at most `RING_WORDS - 1` unread words.
        let words_per_sample = cfg.sample_updates as u32 / CODES_PER_WORD;
        for _ in 0..(RING_WORDS as u32 - 1) / words_per_sample {
            t.fill(0, words_per_sample);
        }
        SHARED.progress.set(Progress { consumed: 0, underruns: 0 });

        start_pll(cfg)?;
        start_pio(cfg);
        // SAFETY: only core 1 uses the ring, and the DMA only reads it.
        let ring = unsafe { addr_of_mut!(CODES.0) } as u32;
        start_dma(ring);
        start_output(cfg);
        // Go: the state machine synchronises and starts pulling codes.
        sync_and_enable()?;
        Ok(t)
    }

    /// Produce `words` words of codes, `x` moving by `dx` per update.
    #[inline(always)]
    fn fill(&mut self, dx: i32, words: u32) {
        self.k.dx = dx;
        let mut left = words as usize;
        while left > 0 {
            // Up to the end of the ring, then on from its start.
            let at = self.written as usize % RING_WORDS;
            let n = left.min(RING_WORDS - at);
            // SAFETY: in bounds; core 1 is the only writer, and these words
            // were read by the DMA a ring ago (the caller keeps the lead
            // below a ring).
            let out = unsafe { core::slice::from_raw_parts_mut((addr_of_mut!(CODES.0) as *mut u32).add(at), n) };
            self.k.words(out);
            self.written = self.written.wrapping_add(n as u32);
            left -= n;
        }
    }

    /// The modulator loop: one sample per iteration when there is room.
    #[unsafe(link_section = ".data.rf_run")]
    #[inline(never)]
    fn run(&mut self) -> Result<(), Fault> {
        let updates = self.cfg.sample_updates as u32;
        let words_per_sample = updates / CODES_PER_WORD;
        let interpolate = self.cfg.interpolate;
        let host_lost = HOST_LOST_S * (XOSC_HZ / self.cfg.refdiv as u32 / self.cfg.update_div as u32) / updates;
        let mut min_lead = u32::MAX;
        // Interpolation moves by whole steps per update and may undershoot
        // the target by up to a sample's worth of updates: keep that inside
        // the modulator's input range.
        let (lo, hi) = (updates, Config::X_MAX - updates);
        loop {
            if let Some(offset) = codes_read_offset() {
                self.read = self.read.wrapping_add(offset.wrapping_sub(self.last_offset) % RING_WORDS as u32);
                self.last_offset = offset;
            }
            let lead = self.written.wrapping_sub(self.read) as i32;
            if lead <= 0 {
                return Err(Fault::Late);
            }
            let lead = lead as u32;
            min_lead = min_lead.min(lead);

            if (RING_WORDS as u32 - 1).saturating_sub(lead) < words_per_sample {
                // The ring is full: publish, and look at the commands.
                if min_lead != u32::MAX {
                    let m = SHARED.min_lead.load(core::sync::atomic::Ordering::Relaxed);
                    SHARED.min_lead.store(m.min(min_lead * CODES_PER_WORD), core::sync::atomic::Ordering::Relaxed);
                    min_lead = u32::MAX;
                }
                let consumed = self.taken.saturating_sub((lead / words_per_sample) as u64);
                SHARED.progress.set(Progress { consumed, underruns: self.underruns });
                if SHARED.command() == Command::Stop {
                    return Ok(());
                }
                continue;
            }

            if SHARED.retune.version() != self.retune_version {
                self.retune_version = SHARED.retune.version();
                let r = SHARED.retune.get();
                (self.center, self.scale) = (r.center.min(Config::X_MAX), r.scale.min(Config::SCALE_MAX));
            }
            let target = match SHARED.samples.pop() {
                Some(s) => {
                    self.taken += 1;
                    self.missing = 0;
                    Config::x_of(self.center, self.scale, s)
                }
                None => {
                    // The host is late: hold the carrier at its centre.
                    if self.missing == 0 {
                        self.underruns += 1;
                    }
                    self.missing += 1;
                    if self.missing > host_lost {
                        return Err(Fault::HostLost);
                    }
                    self.center
                }
            }
            .clamp(lo, hi)
                // An odd input never repeats: no idle tones on a steady carrier.
                | 1;
            let dx = if interpolate { (target as i32).wrapping_sub(self.k.x as i32) / updates as i32 } else { 0 };
            if !interpolate {
                self.k.x = target;
            }
            self.fill(dx, words_per_sample);
            // Exactly on the sample's value at its end.
            self.k.x = target;
        }
    }

    fn stop(&mut self) {
        output_off(Some(self.cfg.gpio));
        stop_pio_dma();
    }
}

fn start_pll(cfg: &Config) -> Result<(), Fault> {
    // The RP2350 only lets the cores at PLL_USB by default; the DMA writes
    // its feedback divider. (ACCESSCTRL writes carry a password.)
    #[cfg(feature = "rp2350")]
    {
        let a = pac::ACCESSCTRL.pll_usb();
        let v = a.read().0 & 0xff;
        a.write_value(pac::accessctrl::regs::Access(0xacce_0000 | v | 1 << 6));
    }
    reset_pll_usb();
    let p = pac::PLL_USB;
    p.pwr().write(|w| {
        w.set_pd(true);
        w.set_vcopd(true);
        w.set_postdivpd(true);
        w.set_dsmpd(true);
    });
    p.cs().write(|w| w.set_refdiv(cfg.refdiv));
    p.fbdiv_int().write(|w| w.set_fbdiv_int(cfg.base + 1 + (cfg.center >> FRAC_BITS) as u16));
    p.pwr().write(|w| {
        w.set_postdivpd(true);
        w.set_dsmpd(true);
    });
    // Lock takes tens of microseconds; give it 10 ms.
    let t0 = embassy_time::Instant::now();
    while !p.cs().read().lock() {
        if t0.elapsed() > embassy_time::Duration::from_millis(10) {
            output_off(None);
            return Err(Fault::PllLock);
        }
    }
    p.prim().write(|w| {
        w.set_postdiv1(cfg.post_div1);
        w.set_postdiv2(cfg.post_div2);
    });
    p.pwr().modify(|w| w.set_postdivpd(false));
    Ok(())
}

fn start_pio(cfg: &Config) {
    reset_pio0();
    let pio = pac::PIO0;
    // Once, wait for a rising edge of the crystal clock (on SYNC_GPIO
    // while starting), then FBDIV = (Y << 5) | code, one per period: the
    // period is a whole number of reference cycles, so every write lands at
    // the same point of one.
    let prg = pio::pio_asm!(
        "    wait 0 gpio 25",
        "    wait 1 gpio 25",
        "    nop",
        ".wrap_target",
        "    out x, 5",
        "    in y, 7",
        "    in x, 5",
        "    nop",
        "    nop",
        "    nop",
        "    nop",
        ".wrap",
    );
    let mut code = prg.program.code;
    code[2] |= (cfg.phase as u16) << 8;
    // Pad the loop to the update period with the nops' delays.
    let mut pad = update_cycles(cfg) - PAD_SLOTS as u32 - 3;
    for slot in &mut code[6..6 + PAD_SLOTS] {
        let d = pad.min(31);
        *slot |= (d as u16) << 8;
        pad -= d;
    }
    for (i, &c) in code.iter().enumerate() {
        pio.instr_mem(i).write_value(pac::pio::regs::InstrMem(c as u32));
    }
    let sm = pio.sm(SM);
    sm.clkdiv().write(|w| {
        w.set_int(1);
        w.set_frac(0);
    });
    sm.execctrl().write(|w| {
        w.set_wrap_bottom(LOOP_START);
        w.set_wrap_top(code.len() as u8 - 1);
    });
    sm.shiftctrl().write(|w| {
        // Codes come most significant first; FBDIV is assembled leftwards.
        w.set_out_shiftdir(false);
        w.set_in_shiftdir(false);
        w.set_autopull(true);
        w.set_pull_thresh(30);
        w.set_autopush(true);
        w.set_push_thresh(12);
    });
    // Y = high bits of FBDIV; then empty the OSR so the first `out` pulls.
    pio.txf(SM).write_value((cfg.base >> 5) as u32);
    let exec = |i: u16| sm.instr().write(|w| w.set_instr(i));
    exec(pio::InstructionOperands::PULL { if_empty: false, block: true }.encode());
    exec(
        pio::InstructionOperands::MOV {
            destination: pio::MovDestination::Y,
            op: pio::MovOperation::None,
            source: pio::MovSource::OSR,
        }
        .encode(),
    );
    exec(pio::InstructionOperands::OUT { destination: pio::OutDestination::NULL, bit_count: 32 }.encode());
    exec(pio::InstructionOperands::JMP { condition: pio::JmpCondition::Always, address: 0 }.encode());
}

/// Put the crystal clock on SYNC_GPIO, enable the state machine, and take
/// the clock away once it has seen the edge.
fn sync_and_enable() -> Result<(), Fault> {
    let c = pac::CLOCKS;
    let pad = pac::PADS_BANK0.gpio(SYNC_GPIO);
    let io = pac::IO_BANK0.gpio(SYNC_GPIO).ctrl();
    let (pad_before, io_before) = (pad.read(), io.read());
    c.clk_gpout_ctrl(SYNC_GPOUT).write(|w| w.set_enable(false));
    c.clk_gpout_div(SYNC_GPOUT).write(|w| {
        w.set_int(1);
        w.set_frac(0);
    });
    c.clk_gpout_ctrl(SYNC_GPOUT).write(|w| {
        w.set_auxsrc(pac::clocks::vals::ClkGpoutCtrlAuxsrc::XOSC_CLKSRC);
        w.set_enable(true);
    });
    pad.write(|w| {
        w.set_ie(true);
        w.set_drive(pac::pads::vals::Drive::_2M_A);
        #[cfg(feature = "rp2350")]
        w.set_iso(false);
    });
    io.write(|w| w.set_funcsel(FUNCSEL_GPCK));
    pac::PIO0.ctrl().modify(|w| w.set_sm_enable(1 << SM));
    let t0 = embassy_time::Instant::now();
    let synced = loop {
        if pac::PIO0.sm(SM).addr().read().addr() >= LOOP_START {
            break true;
        }
        if t0.elapsed() > embassy_time::Duration::from_millis(10) {
            break false;
        }
    };
    io.write_value(io_before);
    pad.write_value(pad_before);
    c.clk_gpout_ctrl(SYNC_GPOUT).write(|w| w.set_enable(false));
    if synced { Ok(()) } else { Err(Fault::Sync) }
}

fn dma_ctrl(w: &mut pac::dma::regs::CtrlTrig, chain_to: usize, treq: pac::dma::vals::TreqSel, ring_bits: u8) {
    w.set_en(true);
    w.set_high_priority(true);
    w.set_data_size(pac::dma::vals::DataSize::SIZE_WORD);
    w.set_incr_read(ring_bits != 0);
    w.set_incr_write(false);
    w.set_ring_size(ring_bits);
    w.set_ring_sel(false);
    w.set_chain_to(chain_to as u8);
    w.set_treq_sel(treq);
    w.set_irq_quiet(true);
}

/// A DMA channel that runs for ever, paced by `treq`, from `read` (a ring
/// of `ring_words` words aligned to `1 << ring_bits` bytes, or with
/// `ring_bits` 0 a fixed address) to the fixed address `write`. On the
/// RP2040, whose transfer counts run out, it is a pair (`ch`, `ch + 1`)
/// that trigger each other, each a whole number of rings long (so that the
/// read address is back at the ring's start).
pub fn start_ring_dma(ch: usize, read: u32, write: u32, treq: pac::dma::vals::TreqSel, ring_bits: u8, ring_words: u32) {
    pac::RESETS.reset().modify(|w| w.set_dma(false));
    while !pac::RESETS.reset_done().read().dma() {}
    #[cfg(feature = "rp2040")]
    {
        let count = if ring_bits != 0 { ring_words << (31 - ring_words.ilog2()).min(20) } else { u32::MAX };
        for (c, other) in [(ch + 1, ch), (ch, ch + 1)] {
            let r = pac::DMA.ch(c);
            r.read_addr().write_value(read);
            r.write_addr().write_value(write);
            r.trans_count().write_value(count);
            let mut v = pac::dma::regs::CtrlTrig(0);
            dma_ctrl(&mut v, other, treq, ring_bits);
            if c == ch {
                r.ctrl_trig().write_value(v);
            } else {
                r.al1_ctrl().write_value(v.0);
            }
        }
    }
    #[cfg(feature = "rp2350")]
    {
        let _ = ring_words;
        let r = pac::DMA.ch(ch);
        r.read_addr().write_value(read);
        r.write_addr().write_value(write);
        r.trans_count().write(|w| {
            w.set_mode(pac::dma::vals::TransCountMode::ENDLESS);
            w.set_count(1);
        });
        let mut v = pac::dma::regs::CtrlTrig(0);
        dma_ctrl(&mut v, ch, treq, ring_bits);
        r.ctrl_trig().write_value(v);
    }
}

fn start_dma(ring: u32) {
    let txf = pac::PIO0.txf(SM).as_ptr() as u32;
    let rxf = pac::PIO0.rxf(SM).as_ptr() as u32;
    let fbdiv = pac::PLL_USB.fbdiv_int().as_ptr() as u32;
    let treq_tx = pac::dma::vals::TreqSel::from_bits(SM as u8);
    let treq_rx = pac::dma::vals::TreqSel::from_bits(4 + SM as u8);
    start_ring_dma(CH_FBDIV, rxf, fbdiv, treq_rx, 0, 0);
    start_ring_dma(CH_CODES, ring, txf, treq_tx, (RING_WORDS * 4).trailing_zeros() as u8, RING_WORDS as u32);
}

/// Position (words) in its ring of a channel started by [`start_ring_dma`],
/// if known right now (on the RP2040 not while the pair hands over).
#[inline(always)]
pub fn ring_read_offset(ch: usize, ring: u32, ring_words: u32) -> Option<u32> {
    #[cfg(feature = "rp2040")]
    {
        for c in [ch, ch + 1] {
            if pac::DMA.ch(c).ctrl_trig().read().busy() {
                return Some(pac::DMA.ch(c).read_addr().read().wrapping_sub(ring) / 4 % ring_words);
            }
        }
        None
    }
    #[cfg(feature = "rp2350")]
    {
        Some(pac::DMA.ch(ch).read_addr().read().wrapping_sub(ring) / 4 % ring_words)
    }
}

/// Position of the codes channel in the ring (words), if known right now.
#[inline(always)]
fn codes_read_offset() -> Option<u32> {
    // SAFETY: only the address is taken.
    let ring = unsafe { addr_of_mut!(CODES.0) } as u32;
    ring_read_offset(CH_CODES, ring, RING_WORDS as u32)
}

/// Pad of an output: drive strength, fast slew, input off.
pub fn configure_pad(gpio: u8, drive_ma: u8) {
    pac::PADS_BANK0.gpio(gpio as usize).write(|w| {
        w.set_drive(match drive_ma {
            2 => pac::pads::vals::Drive::_2M_A,
            4 => pac::pads::vals::Drive::_4M_A,
            8 => pac::pads::vals::Drive::_8M_A,
            _ => pac::pads::vals::Drive::_12M_A,
        });
        w.set_slewfast(true);
        w.set_ie(false);
        w.set_od(false);
        #[cfg(feature = "rp2350")]
        w.set_iso(false);
    });
}

fn start_output(cfg: &Config) {
    let n = gpout_of(cfg.gpio).unwrap();
    let c = pac::CLOCKS;
    c.clk_gpout_ctrl(n).write(|w| w.set_enable(false));
    c.clk_gpout_div(n).write(|w| {
        // 24 integer bits on the RP2040, 16 on the RP2350.
        #[cfg(feature = "rp2040")]
        w.set_int(cfg.gpout_div as u32);
        #[cfg(feature = "rp2350")]
        w.set_int(cfg.gpout_div);
        w.set_frac(0);
    });
    c.clk_gpout_ctrl(n).write(|w| {
        w.set_auxsrc(pac::clocks::vals::ClkGpoutCtrlAuxsrc::CLKSRC_PLL_USB);
        w.set_dc50(cfg.gpout_div % 2 == 1);
        w.set_enable(true);
    });
    configure_pad(cfg.gpio, cfg.drive_ma);
    pac::IO_BANK0.gpio(cfg.gpio as usize).ctrl().write(|w| w.set_funcsel(FUNCSEL_GPCK));
}

fn stop_pio_dma() {
    pac::PIO0.ctrl().modify(|w| w.set_sm_enable(0));
    #[cfg(feature = "rp2040")]
    abort_dma(&[CH_CODES, CH_CODES + 1, CH_FBDIV, CH_FBDIV + 1]);
    #[cfg(feature = "rp2350")]
    abort_dma(&[CH_CODES, CH_FBDIV]);
}

/// Stop DMA channels: disable first, then abort (RP2040-E13).
pub fn abort_dma(chans: &[usize]) {
    let mut mask = 0u16;
    for &ch in chans {
        pac::DMA.ch(ch).al1_ctrl().write_value(0);
        mask |= 1 << ch;
    }
    pac::DMA.chan_abort().write(|w| w.set_chan_abort(mask));
    while pac::DMA.chan_abort().read().chan_abort() != 0 {}
}
