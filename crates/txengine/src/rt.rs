//! Operating-system helpers: real-time scheduling, signals, single-instance
//! lock, local time offset.

use std::os::fd::{AsRawFd, OwnedFd};
use std::sync::atomic::{AtomicBool, AtomicI32, Ordering};

use log::{debug, warn};

static STOP: AtomicBool = AtomicBool::new(false);
static SIGNAL: AtomicI32 = AtomicI32::new(0);

extern "C" fn on_signal(sig: libc::c_int) {
    STOP.store(true, Ordering::SeqCst);
    SIGNAL.store(sig, Ordering::SeqCst);
}

/// Install handlers so SIGINT/SIGTERM/SIGHUP/SIGQUIT request a clean stop.
pub fn install_signal_handlers() {
    // SAFETY: the handler only touches atomics (async-signal-safe).
    unsafe {
        for sig in [libc::SIGINT, libc::SIGTERM, libc::SIGHUP, libc::SIGQUIT] {
            let mut sa: libc::sigaction = std::mem::zeroed();
            sa.sa_sigaction = on_signal as *const () as usize;
            libc::sigemptyset(&mut sa.sa_mask);
            libc::sigaction(sig, &sa, std::ptr::null_mut());
        }
        // A closed control socket or stdout pipe must not kill the transmitter.
        libc::signal(libc::SIGPIPE, libc::SIG_IGN);
    }
}

pub fn stop_requested() -> bool {
    STOP.load(Ordering::Relaxed)
}

pub fn request_stop() {
    STOP.store(true, Ordering::SeqCst);
}

pub fn stop_signal() -> Option<i32> {
    match SIGNAL.load(Ordering::Relaxed) {
        0 => None,
        s => Some(s),
    }
}

/// Lock memory and give the calling thread real-time priority.
pub fn make_realtime(priority: i32) {
    // SAFETY: plain syscalls.
    unsafe {
        if libc::mlockall(libc::MCL_CURRENT | libc::MCL_FUTURE) != 0 {
            warn!("mlockall failed ({}); page faults may cause glitches", std::io::Error::last_os_error());
        }
        if priority > 0 {
            // musl's sched_setscheduler() is a stub (ENOSYS); the pthread
            // call sets the policy of this thread only, which is what we want.
            let mut p: libc::sched_param = std::mem::zeroed();
            p.sched_priority = priority;
            let rc = libc::pthread_setschedparam(libc::pthread_self(), libc::SCHED_FIFO, &p);
            if rc != 0 {
                warn!(
                    "cannot use SCHED_FIFO ({}); running with normal priority",
                    std::io::Error::from_raw_os_error(rc)
                );
            } else {
                debug!("transmit thread runs SCHED_FIFO priority {priority}");
            }
        }
    }
}

/// Exclusive instance lock: two transmitters fighting over the same PLL and
/// DMA tick would corrupt each other's output.
/// `name` tells transmitters apart (`txengine` for the Pi's own hardware,
/// one per Pico).
pub fn instance_lock(name: &str) -> anyhow::Result<OwnedFd> {
    use std::ffi::CString;
    use std::os::fd::FromRawFd;
    let name: String = name.chars().filter(|c| c.is_ascii_alphanumeric() || *c == '-').collect();
    let path = CString::new(format!("/run/{name}.lock"))?;
    let alt = CString::new(format!("/tmp/{name}.lock"))?;
    // SAFETY: plain syscalls with valid C strings.
    let fd = unsafe {
        let mut fd = libc::open(path.as_ptr(), libc::O_RDWR | libc::O_CREAT | libc::O_CLOEXEC, 0o644);
        if fd < 0 {
            fd = libc::open(alt.as_ptr(), libc::O_RDWR | libc::O_CREAT | libc::O_CLOEXEC, 0o644);
        }
        if fd < 0 {
            anyhow::bail!("cannot create the instance lock file: {}", std::io::Error::last_os_error());
        }
        OwnedFd::from_raw_fd(fd)
    };
    // SAFETY: valid descriptor.
    if unsafe { libc::flock(fd.as_raw_fd(), libc::LOCK_EX | libc::LOCK_NB) } != 0 {
        anyhow::bail!("another txengine instance is already transmitting");
    }
    Ok(fd)
}

/// Local time offset from UTC (seconds) at Unix time `t`, from the system
/// time zone (handles DST transitions).
#[allow(deprecated)] // libc::time_t on 32-bit musl (still 32 bit in the libc crate)
pub fn local_offset(t: f64) -> i32 {
    let tt = t as libc::time_t;
    // SAFETY: localtime_r writes into our struct.
    unsafe {
        let mut tm: libc::tm = std::mem::zeroed();
        if libc::localtime_r(&tt, &mut tm).is_null() {
            return 0;
        }
        tm.tm_gmtoff as i32
    }
}

/// Current Unix time in seconds.
pub fn unix_now() -> f64 {
    std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0)
}

// The raw system calls below take the kernel's structures for the native
// word size (the 32-bit time ones on 32-bit ARM, whatever the C library's
// time64 ABI), which libc's types match when `time_t` is a `long`.
#[allow(deprecated)] // libc::time_t on 32-bit musl (still 32 bit in the libc crate)
const _: () = assert!(std::mem::size_of::<libc::time_t>() == std::mem::size_of::<libc::c_long>());

/// Seconds on the raw monotonic clock (`CLOCK_MONOTONIC_RAW`: the CPU's
/// timer, never adjusted by NTP), since boot.
pub fn monotonic_raw() -> f64 {
    let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
    // SAFETY: the kernel fills a `timespec` of the native layout (see above).
    unsafe { libc::syscall(libc::SYS_clock_gettime, libc::CLOCK_MONOTONIC_RAW, &mut ts as *mut libc::timespec) };
    ts.tv_sec as f64 + ts.tv_nsec as f64 * 1e-9
}

/// The kernel's NTP state of the system clock.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KernelClock {
    /// An NTP daemon (chrony, systemd-timesyncd, ...) keeps the system clock
    /// on true time.
    pub synchronised: bool,
    /// Frequency correction of the system clock against the raw clock
    /// (1e-6: the system clock runs 1 ppm faster), without the extra
    /// slewing with which the kernel removes time offsets.
    pub frequency: f64,
}

/// Ask the kernel about the system clock; `None` if it cannot be asked.
///
/// Synchronisation: the kernel's own flag (`STA_UNSYNC`) is not enough, as
/// chrony clears it only with `rtcsync` (which also starts the kernel's
/// 11-minute RTC updates). Every NTP daemon does set the kernel's maximum
/// error, which the kernel then grows by 0.5 ms per second (it starts at
/// 16 s at boot), so a small maximum error means a daemon has disciplined
/// the clock within the last quarter of an hour.
pub fn kernel_clock() -> Option<KernelClock> {
    // SAFETY: a plain C structure, valid when zeroed (modes 0: read only).
    let mut t: libc::timex = unsafe { std::mem::zeroed() };
    // SAFETY: the kernel fills a `timex` of the native layout (see above).
    let state = unsafe { libc::syscall(libc::SYS_adjtimex, &mut t as *mut libc::timex) };
    if state < 0 {
        return None;
    }
    // SAFETY: plain call.
    let user_hz = unsafe { libc::sysconf(libc::_SC_CLK_TCK) }.max(1) as f64;
    // `tick`: µs added per 1/USER_HZ s (coarse); `freq`: ppm · 2^16 (fine).
    let frequency = t.tick as f64 * user_hz * 1e-6 * (1.0 + t.freq as f64 / 65536.0 * 1e-6) - 1.0;
    let kernel_synced = state != libc::TIME_ERROR as libc::c_long && t.status & libc::STA_UNSYNC == 0;
    log::trace!(
        "system clock: kernel state {state}, status {:#x}, frequency {:+.3} ppm, maximum error {} µs, estimated error {} µs",
        t.status,
        frequency * 1e6,
        t.maxerror,
        t.esterror
    );
    Some(KernelClock { synchronised: kernel_synced || (t.maxerror as i64) < 500_000, frequency })
}
