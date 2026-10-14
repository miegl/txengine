//! The USB protocol between txengine and its RP2040/RP2350 firmware.
//!
//! The firmware turns a Raspberry Pi Pico into an RF front end: the host
//! renders the samples (instantaneous carrier frequency offsets) and plans
//! the clocks, the Pico synthesises the carrier and modulates it.
//!
//! Two ways to make the carrier ([`Config`], [`PioConfig`]):
//!
//! * **PLL**: the PLL that is not clocking the system makes the carrier;
//!   its integer feedback divider (`FBDIV_INT`) is switched at a fraction of
//!   the reference rate by a second-order sigma-delta modulator
//!   ([`sigma_delta::Mash11`]), whose average is the fractional divider the
//!   hardware lacks. The PLL's loop filter smooths the switching.
//! * **PIO**: a PIO state machine puts out a bit stream at the system clock,
//!   which runs at 240-264 MHz from the other PLL while it does: a square
//!   wave whose edges fall on the nearest bit. Cleaner near the carrier, but
//!   the square wave's harmonics alias, some of them into the FM band.
//!
//! The device has one vendor interface ([`INTERFACE_CLASS`]…) with a bulk OUT
//! endpoint ([`SAMPLES_ENDPOINT`]) for the samples, `i16` little endian, one
//! after another; control requests to that interface ([`Request`]) configure,
//! start, stop, retune and query it. A second interface is the Pico SDK's
//! reset interface ([`RESET_INTERFACE_CLASS`]…), so `picotool reboot -f -u`
//! can put the device back into its USB bootloader.
//!
//! All multi-byte fields are little endian.

#![no_std]

pub mod am;
pub mod sigma_delta;

/// Version of this protocol; [`Info::protocol`] must match.
pub const PROTOCOL_VERSION: u16 = 2;

/// Longest firmware version string ([`Request::Version`]), in bytes.
pub const VERSION_MAX: usize = 64;

/// USB vendor ID (Raspberry Pi).
pub const VID: u16 = 0x2e8a;
/// USB product IDs. These are the Pico SDK's IDs for USB stdio devices,
/// which picotool's udev rules make accessible to the logged-in user and
/// which `picotool reboot -f` recognises.
pub const PID_RP2040: u16 = 0x000a;
pub const PID_RP2350: u16 = 0x0009;

/// The txengine interface: vendor class, subclass `'T'`.
pub const INTERFACE_CLASS: u8 = 0xff;
pub const INTERFACE_SUBCLASS: u8 = 0x54;
pub const INTERFACE_PROTOCOL: u8 = 0x01;

/// The Pico SDK's reset interface (`pico/usb_reset_interface.h`).
pub const RESET_INTERFACE_CLASS: u8 = 0xff;
pub const RESET_INTERFACE_SUBCLASS: u8 = 0x00;
pub const RESET_INTERFACE_PROTOCOL: u8 = 0x01;
/// Reset interface request: reboot into the USB bootloader.
pub const RESET_REQUEST_BOOTSEL: u8 = 0x01;
/// Reset interface request: reboot into the application.
pub const RESET_REQUEST_FLASH: u8 = 0x02;

/// Bulk OUT endpoint carrying the samples.
pub const SAMPLES_ENDPOINT: u8 = 0x01;
/// Its maximum packet size (USB full speed).
pub const SAMPLES_PACKET: usize = 64;

/// Fixed-point scale of the sigma-delta input: `1 << FRAC_BITS` is one
/// step of the feedback divider.
pub const FRAC_BITS: u32 = 30;

/// Vendor control requests to the txengine interface (`wIndex` = its
/// interface number).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Request {
    /// IN: [`Info`].
    Info = 0x01,
    /// OUT: [`Config`]. Only while idle.
    Configure = 0x02,
    /// OUT, no data: start transmitting the configured carrier. The samples
    /// already received are emitted first.
    Start = 0x03,
    /// OUT, no data: stop, switch the output off and discard the samples.
    Stop = 0x04,
    /// IN: [`Status`].
    Status = 0x05,
    /// OUT: [`Retune`], applied from the next sample on.
    Retune = 0x06,
    /// OUT, no data: reboot into the USB bootloader.
    Bootsel = 0x07,
    /// OUT: [`PioConfig`]. Only while idle.
    ConfigurePio = 0x08,
    /// IN: the firmware's version, UTF-8, at most [`VERSION_MAX`] bytes:
    /// `git describe` of its source tree, like txengine's.
    Version = 0x09,
}

impl Request {
    pub fn from_u8(v: u8) -> Option<Request> {
        Some(match v {
            0x01 => Request::Info,
            0x02 => Request::Configure,
            0x03 => Request::Start,
            0x04 => Request::Stop,
            0x05 => Request::Status,
            0x06 => Request::Retune,
            0x07 => Request::Bootsel,
            0x08 => Request::ConfigurePio,
            0x09 => Request::Version,
            _ => return None,
        })
    }
}

/// The microcontroller.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Chip {
    Rp2040 = 1,
    Rp2350 = 2,
}

impl Chip {
    pub fn name(self) -> &'static str {
        match self {
            Chip::Rp2040 => "RP2040",
            Chip::Rp2350 => "RP2350",
        }
    }

    /// The Raspberry Pi board the firmware is built for.
    pub fn board(self) -> &'static str {
        match self {
            Chip::Rp2040 => "Pico",
            Chip::Rp2350 => "Pico 2",
        }
    }
}

/// Little-endian field reader.
struct Reader<'a>(&'a [u8]);

impl Reader<'_> {
    fn take<const N: usize>(&mut self) -> Option<[u8; N]> {
        let (head, rest) = self.0.split_first_chunk::<N>()?;
        self.0 = rest;
        Some(*head)
    }
    fn u8(&mut self) -> Option<u8> {
        self.take::<1>().map(|b| b[0])
    }
    fn u16(&mut self) -> Option<u16> {
        self.take().map(u16::from_le_bytes)
    }
    fn u32(&mut self) -> Option<u32> {
        self.take().map(u32::from_le_bytes)
    }
    fn u64(&mut self) -> Option<u64> {
        self.take().map(u64::from_le_bytes)
    }
}

/// Little-endian field writer.
struct Writer<'a> {
    buf: &'a mut [u8],
    at: usize,
}

impl Writer<'_> {
    fn put(&mut self, b: &[u8]) {
        self.buf[self.at..self.at + b.len()].copy_from_slice(b);
        self.at += b.len();
    }
}

/// What the device is ([`Request::Info`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Info {
    pub protocol: u16,
    pub chip: Chip,
    /// Nominal crystal frequency, the PLL reference (Hz).
    pub xosc_hz: u32,
    /// System clock (Hz).
    pub sys_hz: u32,
    /// Capacity of the sample buffer (samples).
    pub buffer_samples: u32,
    /// Highest sigma-delta update rate the firmware sustains (Hz).
    pub max_update_hz: u32,
    /// GPIOs the carrier can be put on (bit n = GPIO n).
    pub gpio_mask: u32,
    /// Bit rates (Hz) the PIO method accepts.
    pub pio_min_hz: u32,
    pub pio_max_hz: u32,
    /// If the watchdog reset the firmware (a hang), the step it had reached
    /// (a breadcrumb for debugging); 0 otherwise.
    pub watchdog_note: u32,
}

impl Info {
    pub const LEN: usize = 35;

    pub fn encode(&self, buf: &mut [u8; Self::LEN]) {
        let mut w = Writer { buf, at: 0 };
        w.put(&self.protocol.to_le_bytes());
        w.put(&[self.chip as u8]);
        w.put(&self.xosc_hz.to_le_bytes());
        w.put(&self.sys_hz.to_le_bytes());
        w.put(&self.buffer_samples.to_le_bytes());
        w.put(&self.max_update_hz.to_le_bytes());
        w.put(&self.gpio_mask.to_le_bytes());
        w.put(&self.pio_min_hz.to_le_bytes());
        w.put(&self.pio_max_hz.to_le_bytes());
        w.put(&self.watchdog_note.to_le_bytes());
    }

    pub fn decode(b: &[u8]) -> Option<Info> {
        let mut r = Reader(b);
        Some(Info {
            protocol: r.u16()?,
            chip: match r.u8()? {
                1 => Chip::Rp2040,
                2 => Chip::Rp2350,
                _ => return None,
            },
            xosc_hz: r.u32()?,
            sys_hz: r.u32()?,
            buffer_samples: r.u32()?,
            max_update_hz: r.u32()?,
            gpio_mask: r.u32()?,
            pio_min_hz: r.u32()?,
            pio_max_hz: r.u32()?,
            watchdog_note: r.u32()?,
        })
    }
}

/// A setting of `PLL_USB` as the system clock: `xosc / refdiv · fbdiv /
/// (post_div1 · post_div2)` (the PIO method).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SysClock {
    pub refdiv: u8,
    pub fbdiv: u16,
    pub post_div1: u8,
    pub post_div2: u8,
}

impl SysClock {
    /// The clock (Hz) from a crystal of `xosc_hz`.
    pub fn hz(&self, xosc_hz: u32) -> u32 {
        let vco = (xosc_hz / self.refdiv.max(1) as u32) as u64 * self.fbdiv as u64;
        (vco / (self.post_div1.max(1) as u64 * self.post_div2.max(1) as u64)) as u32
    }
}

/// The PIO method ([`Request::ConfigurePio`]).
///
/// `PLL_USB` runs at `xosc / refdiv · fbdiv` and, divided by `post_div1 ·
/// post_div2`, clocks the system and the state machine: that is the bit
/// rate `B`. The pin puts out one bit per cycle, least significant first,
/// 32 per word. Word `n` is the pattern of a square wave at `ratio · B`
/// starting at phase `φ(n)` (in cycles, 2³² = one cycle), where `φ` advances
/// by `center + s · scale` per word for the sample `s` (i16) being sent.
/// Samples last `sample_words` words; with `interpolate` the advance moves
/// linearly from one sample's value to the next.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PioConfig {
    pub refdiv: u8,
    pub fbdiv: u16,
    pub post_div1: u8,
    pub post_div2: u8,
    pub gpio: u8,
    pub drive_ma: u8,
    /// Carrier / bit rate (2³² = 1), below ½.
    pub ratio: u32,
    pub center: u32,
    /// Phase advance per word and sample unit; at most [`PioConfig::SCALE_MAX`].
    pub scale: u32,
    pub sample_words: u8,
    pub interpolate: bool,
}

impl PioConfig {
    pub const LEN: usize = 22;
    /// `s · scale` then fits an `i32`.
    pub const SCALE_MAX: u32 = 0xffff;
    /// Phase-indexed patterns in the firmware's table (log2).
    pub const TABLE_BITS: u32 = 12;

    pub fn encode(&self, buf: &mut [u8; Self::LEN]) {
        let mut w = Writer { buf, at: 0 };
        w.put(&[self.refdiv]);
        w.put(&self.fbdiv.to_le_bytes());
        w.put(&[self.post_div1, self.post_div2, self.gpio, self.drive_ma]);
        w.put(&self.ratio.to_le_bytes());
        w.put(&self.center.to_le_bytes());
        w.put(&self.scale.to_le_bytes());
        w.put(&[self.sample_words, self.interpolate as u8]);
    }

    pub fn clock(&self) -> SysClock {
        SysClock { refdiv: self.refdiv, fbdiv: self.fbdiv, post_div1: self.post_div1, post_div2: self.post_div2 }
    }

    pub fn decode(b: &[u8]) -> Option<PioConfig> {
        let mut r = Reader(b);
        Some(PioConfig {
            refdiv: r.u8()?,
            fbdiv: r.u16()?,
            post_div1: r.u8()?,
            post_div2: r.u8()?,
            gpio: r.u8()?,
            drive_ma: r.u8()?,
            ratio: r.u32()?,
            center: r.u32()?,
            scale: r.u32()?,
            sample_words: r.u8()?,
            interpolate: r.u8()? != 0,
        })
    }

    /// The word for start phase `phase`: bit `j` (sent `j`-th) is the
    /// square wave (high for the first half cycle) at `phase + j · ratio`,
    /// taken at the middle of `phase`'s table bin.
    pub fn pattern(ratio: u32, bin: u32) -> u32 {
        let start = (bin << (32 - Self::TABLE_BITS)) | (1 << (31 - Self::TABLE_BITS));
        (0..32).fold(0, |w, j| w | ((start.wrapping_add(ratio.wrapping_mul(j)) < 1 << 31) as u32) << j)
    }
}

/// The carrier and its modulation ([`Request::Configure`]).
///
/// The PLL runs at `xosc / refdiv · FBDIV`, and the carrier is that divided
/// by `post_div1 · post_div2 · gpout_div`. Every `update_div` reference
/// cycles `FBDIV` is set to `base + code`, where `code` (0-4) is the output
/// of the sigma-delta modulator plus one, for the input `x` (in units of
/// `1 << FRAC_BITS` per step, within `0 ..= 2 << FRAC_BITS`): the average
/// of `FBDIV` is `base + 1 + x`. A sample `s` (i16) sets
/// `x = center + s · scale`; samples last `sample_updates` updates (a
/// multiple of [`Config::CODES_PER_WORD`]), and with `interpolate` `x`
/// moves linearly from one sample's value to the next instead of holding
/// it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    pub refdiv: u8,
    pub post_div1: u8,
    pub post_div2: u8,
    pub gpout_div: u16,
    /// Output GPIO.
    pub gpio: u8,
    /// Pad drive strength in mA (2, 4, 8 or 12).
    pub drive_ma: u8,
    /// `FBDIV` for code 0. The firmware prepends the high bits of `FBDIV`
    /// to 5-bit codes, so `base % 32 + 4` must not exceed 31.
    pub base: u16,
    /// Reference cycles per sigma-delta update.
    pub update_div: u8,
    /// Updates per sample, a multiple of [`Config::CODES_PER_WORD`].
    pub sample_updates: u8,
    pub interpolate: bool,
    pub center: u32,
    /// `x` per unit of a sample; at most [`Config::SCALE_MAX`].
    pub scale: u32,
    /// When `FBDIV` is written, in system clock cycles after a rising edge
    /// of the reference (the firmware aligns its writes to the crystal);
    /// see [`Config::PHASE`].
    pub phase: u8,
}

impl Config {
    pub const LEN: usize = 21;

    pub fn encode(&self, buf: &mut [u8; Self::LEN]) {
        let mut w = Writer { buf, at: 0 };
        w.put(&[self.refdiv, self.post_div1, self.post_div2]);
        w.put(&self.gpout_div.to_le_bytes());
        w.put(&[self.gpio, self.drive_ma]);
        w.put(&self.base.to_le_bytes());
        w.put(&[self.update_div, self.sample_updates, self.interpolate as u8]);
        w.put(&self.center.to_le_bytes());
        w.put(&self.scale.to_le_bytes());
        w.put(&[self.phase]);
    }

    pub fn decode(b: &[u8]) -> Option<Config> {
        let mut r = Reader(b);
        Some(Config {
            refdiv: r.u8()?,
            post_div1: r.u8()?,
            post_div2: r.u8()?,
            gpout_div: r.u16()?,
            gpio: r.u8()?,
            drive_ma: r.u8()?,
            base: r.u16()?,
            update_div: r.u8()?,
            sample_updates: r.u8()?,
            interpolate: r.u8()? != 0,
            center: r.u32()?,
            scale: r.u32()?,
            phase: r.u8()?,
        })
    }

    /// The highest `x` a sample may set (inclusive).
    pub const X_MAX: u32 = 2 << FRAC_BITS;
    /// The write phase to use. The PLL takes a new `FBDIV` over into its
    /// reference clock domain with a small window, about 3 system clock
    /// cycles (21 ns at 144 MHz) wide, in which a change is captured wrongly:
    /// on both chips, at 144 MHz, writes 1-4 cycles after the edge (with this
    /// firmware's latency) smear the carrier, 5-11 and 0 are clean.
    pub const PHASE: u8 = 8;
    /// A new `FBDIV` must also stay for at least this many crystal cycles
    /// (3 or fewer smear the carrier on both chips, at any phase, also with
    /// the reference divided by 2).
    pub const MIN_UPDATE_XOSC_CYCLES: u32 = 4;

    /// Bits of a code, and codes per 32-bit word (the firmware packs them).
    pub const CODE_BITS: u32 = 5;
    pub const CODES_PER_WORD: u8 = 6;

    /// Whether the codes `0..=4` added to `base` fit its low 5 bits.
    pub fn codes_fit(base: u16) -> bool {
        (base & 0x1f) + 4 <= 0x1f
    }

    /// The largest scale: `s · scale` then fits an `i32`.
    pub const SCALE_MAX: u32 = 0xffff;

    /// `x` for sample `s`, clamped to the modulator's input range.
    /// `scale` must not exceed [`Config::SCALE_MAX`].
    #[inline(always)]
    pub fn x_of(center: u32, scale: u32, s: i16) -> u32 {
        // A 32-bit multiply: the Cortex-M0+ has no 64-bit one.
        let x = center as i64 + (s as i32 * scale as i32) as i64;
        x.clamp(0, Self::X_MAX as i64) as u32
    }
}

/// A new centre and scale ([`Request::Retune`]), e.g. for a corrected
/// crystal error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Retune {
    pub center: u32,
    pub scale: u32,
}

impl Retune {
    pub const LEN: usize = 8;

    pub fn encode(&self, buf: &mut [u8; Self::LEN]) {
        let mut w = Writer { buf, at: 0 };
        w.put(&self.center.to_le_bytes());
        w.put(&self.scale.to_le_bytes());
    }

    pub fn decode(b: &[u8]) -> Option<Retune> {
        let mut r = Reader(b);
        Some(Retune { center: r.u32()?, scale: r.u32()? })
    }
}

/// State of the transmitter.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    /// Output off; accepts [`Request::Configure`].
    Idle = 0,
    /// Configured, waiting for [`Request::Start`].
    Configured = 1,
    /// Transmitting.
    Running = 2,
    /// Stopped by an error ([`Status::fault`]); [`Request::Stop`] clears it.
    Fault = 3,
}

/// Why the device stopped or refused a configuration.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fault {
    None = 0,
    /// A configuration value is out of range.
    BadConfig = 1,
    /// The PLL did not lock.
    PllLock = 2,
    /// No samples for too long while running: the host is gone.
    HostLost = 3,
    /// The modulator fell behind the hardware.
    Late = 4,
    /// The modulator did not see the crystal clock to align to.
    Sync = 5,
    /// The state machine ran out of data (the DMA fell behind).
    Stall = 6,
}

impl Fault {
    pub fn from_u8(v: u8) -> Fault {
        match v {
            1 => Fault::BadConfig,
            2 => Fault::PllLock,
            3 => Fault::HostLost,
            4 => Fault::Late,
            5 => Fault::Sync,
            6 => Fault::Stall,
            _ => Fault::None,
        }
    }

    pub fn describe(self) -> &'static str {
        match self {
            Fault::None => "none",
            Fault::BadConfig => "configuration out of range",
            Fault::PllLock => "the PLL did not lock",
            Fault::HostLost => "no samples from the host",
            Fault::Late => "the modulator fell behind the hardware",
            Fault::Sync => "the modulator could not align to the crystal clock",
            Fault::Stall => "the state machine ran out of data",
        }
    }
}

/// Progress of the transmitter ([`Request::Status`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Status {
    pub state: State,
    pub fault: Fault,
    /// Samples received since [`Request::Start`] (or since the last stop).
    pub received: u64,
    /// Samples whose emission has started.
    pub consumed: u64,
    /// Number of times the buffer ran empty while running.
    pub underruns: u32,
    /// Smallest lead of the modulator over the hardware since the last
    /// status request, in sigma-delta updates.
    pub min_lead: u32,
}

impl Status {
    pub const LEN: usize = 26;

    pub fn encode(&self, buf: &mut [u8; Self::LEN]) {
        let mut w = Writer { buf, at: 0 };
        w.put(&[self.state as u8, self.fault as u8]);
        w.put(&self.received.to_le_bytes());
        w.put(&self.consumed.to_le_bytes());
        w.put(&self.underruns.to_le_bytes());
        w.put(&self.min_lead.to_le_bytes());
    }

    pub fn decode(b: &[u8]) -> Option<Status> {
        let mut r = Reader(b);
        Some(Status {
            state: match r.u8()? {
                0 => State::Idle,
                1 => State::Configured,
                2 => State::Running,
                3 => State::Fault,
                _ => return None,
            },
            fault: Fault::from_u8(r.u8()?),
            received: r.u64()?,
            consumed: r.u64()?,
            underruns: r.u32()?,
            min_lead: r.u32()?,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips() {
        let i = Info {
            protocol: 1,
            chip: Chip::Rp2350,
            xosc_hz: 12_000_000,
            sys_hz: 144_000_000,
            buffer_samples: 32768,
            max_update_hz: 6_000_000,
            gpio_mask: 1 << 21,
            pio_min_hz: 240_000_000,
            pio_max_hz: 264_000_000,
            watchdog_note: 0,
        };
        let mut b = [0; Info::LEN];
        i.encode(&mut b);
        assert_eq!(Info::decode(&b), Some(i));

        let c = Config {
            refdiv: 1,
            post_div1: 4,
            post_div2: 4,
            gpout_div: 1,
            gpio: 21,
            drive_ma: 12,
            base: 116,
            update_div: 2,
            sample_updates: 18,
            interpolate: true,
            center: 0x5555_5555,
            scale: 12345,
            phase: 7,
        };
        let mut b = [0; Config::LEN];
        c.encode(&mut b);
        assert_eq!(Config::decode(&b), Some(c));

        let p = PioConfig {
            refdiv: 1,
            fbdiv: 126,
            post_div1: 6,
            post_div2: 1,
            gpio: 21,
            drive_ma: 12,
            ratio: 0x5900_0000,
            center: 0x2000_0000,
            scale: 1636,
            sample_words: 42,
            interpolate: true,
        };
        let mut b = [0; PioConfig::LEN];
        p.encode(&mut b);
        assert_eq!(PioConfig::decode(&b), Some(p));

        let s = Status {
            state: State::Running,
            fault: Fault::None,
            received: 1 << 40,
            consumed: (1 << 40) - 5,
            underruns: 3,
            min_lead: 777,
        };
        let mut b = [0; Status::LEN];
        s.encode(&mut b);
        assert_eq!(Status::decode(&b), Some(s));
        assert_eq!(Status::decode(&b[..Status::LEN - 1]), None);
    }

    #[test]
    fn patterns_are_square_waves() {
        // A quarter of the bit rate: 1100 repeated, from phase 0.
        assert_eq!(PioConfig::pattern(1 << 30, 0), 0x3333_3333);
        // Half a cycle later the pattern is inverted.
        let half = 1 << (PioConfig::TABLE_BITS - 1);
        assert_eq!(PioConfig::pattern(1 << 30, half), !0x3333_3333);
    }

    #[test]
    fn x_is_clamped() {
        assert_eq!(Config::x_of(1 << 30, 1000, 100), (1 << 30) + 100_000);
        assert_eq!(Config::x_of(1000, 1000, -2), 0);
        assert_eq!(Config::x_of(Config::X_MAX - 5, 1000, 1), Config::X_MAX);
    }
}
