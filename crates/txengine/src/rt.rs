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
                warn!("cannot use SCHED_FIFO ({}); running with normal priority", std::io::Error::from_raw_os_error(rc));
            } else {
                debug!("transmit thread runs SCHED_FIFO priority {priority}");
            }
        }
    }
}

/// Exclusive instance lock: two transmitters fighting over the same PLL and
/// DMA tick would corrupt each other's output.
pub fn instance_lock() -> anyhow::Result<OwnedFd> {
    use std::os::fd::FromRawFd;
    let path = c"/run/txengine.lock";
    let alt = c"/tmp/txengine.lock";
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
