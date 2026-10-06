//! Fail-safe hardware shutdown when the transmitter process dies abruptly.
//!
//! DMA engines keep running when the process that programmed them dies:
//! the carrier would stay on (replaying the last buffer on BCM283x), and on
//! the Raspberry Pi 5 the DMA engine could touch memory that has been
//! returned to the kernel. Ordinary `Drop`/signal handling cannot cover
//! `SIGKILL`, the OOM killer or a crash.
//!
//! The guardian is a small child process forked while the transmitter is
//! still single threaded. It inherits the register mappings and the
//! descriptors that keep DMA memory alive, and blocks on a pipe. If the
//! parent exits without dismissing it (pipe EOF), it runs the emergency
//! shutdown routine and only then releases the memory by exiting.

use std::os::fd::{AsRawFd, FromRawFd, OwnedFd, RawFd};

use log::{debug, warn};

use crate::error::{HalError, Result};

/// Message the parent sends to dismiss the guardian after a clean shutdown.
const DISMISS: u8 = b'D';

/// Handle held by the parent.
#[derive(Debug)]
pub struct Guardian {
    pipe_w: Option<OwnedFd>,
    pid: libc::pid_t,
}

impl Guardian {
    /// Fork the guardian.
    ///
    /// `cleanup` runs in the child if the parent dies. `keep_fds` are the
    /// descriptors the child must keep open (memory pins, mailbox); all
    /// others are closed in the child. Must be called while the process is
    /// single threaded.
    pub fn spawn(keep_fds: &[RawFd], cleanup: impl FnOnce()) -> Result<Guardian> {
        let mut fds = [0 as libc::c_int; 2];
        // SAFETY: pipe2 writes two descriptors into the array.
        if unsafe { libc::pipe2(fds.as_mut_ptr(), libc::O_CLOEXEC) } != 0 {
            return Err(HalError::last_os("guardian pipe"));
        }
        // SAFETY: freshly created descriptors we own.
        let (r, w) = unsafe { (OwnedFd::from_raw_fd(fds[0]), OwnedFd::from_raw_fd(fds[1])) };

        // SAFETY: fork; the child only runs `child_main`, which never returns.
        let pid = unsafe { libc::fork() };
        if pid < 0 {
            return Err(HalError::last_os("fork guardian"));
        }
        if pid == 0 {
            drop(w);
            child_main(r.as_raw_fd(), keep_fds, cleanup);
        }
        drop(r);
        drop(cleanup);
        debug!("shutdown guardian started (pid {pid})");
        Ok(Guardian { pipe_w: Some(w), pid })
    }

    /// Tell the guardian the hardware has been shut down cleanly; it exits
    /// without touching anything.
    pub fn dismiss(&mut self) {
        let Some(w) = self.pipe_w.take() else { return };
        // SAFETY: writing one byte from a valid buffer to our pipe.
        let n = unsafe { libc::write(w.as_raw_fd(), [DISMISS].as_ptr().cast(), 1) };
        if n != 1 {
            warn!("could not dismiss the shutdown guardian; it will re-run the shutdown sequence");
        }
        drop(w);
        let mut status = 0;
        // SAFETY: waiting for our own child.
        unsafe { libc::waitpid(self.pid, &mut status, 0) };
    }
}

impl Drop for Guardian {
    fn drop(&mut self) {
        // Dropping without dismissing (e.g. a panic before shutdown) closes
        // the pipe, which triggers the emergency shutdown in the child.
        if let Some(w) = self.pipe_w.take() {
            drop(w);
            let mut status = 0;
            // SAFETY: waiting for our own child.
            unsafe { libc::waitpid(self.pid, &mut status, 0) };
        }
    }
}

fn child_main(pipe_r: RawFd, keep_fds: &[RawFd], cleanup: impl FnOnce()) -> ! {
    // SAFETY: plain libc calls in the freshly forked, single-threaded child.
    unsafe {
        // Terminal signals are meant for the transmitter; the guardian must
        // outlive it to do its job.
        for sig in [libc::SIGINT, libc::SIGTERM, libc::SIGHUP, libc::SIGQUIT, libc::SIGPIPE, libc::SIGTSTP] {
            libc::signal(sig, libc::SIG_IGN);
        }
        libc::prctl(libc::PR_SET_NAME, c"txengine-guard".as_ptr() as libc::c_ulong, 0, 0, 0);
        close_other_fds(pipe_r, keep_fds);
    }
    let mut buf = [0u8; 1];
    let n = loop {
        // SAFETY: reading into a valid one byte buffer.
        let n = unsafe { libc::read(pipe_r, buf.as_mut_ptr().cast(), 1) };
        if n < 0 && std::io::Error::last_os_error().kind() == std::io::ErrorKind::Interrupted {
            continue;
        }
        break n;
    };
    if !(n == 1 && buf[0] == DISMISS) {
        eprintln!("txengine-guard: transmitter exited without shutting down; stopping the hardware");
        cleanup();
    }
    // SAFETY: terminate the child without running the parent's destructors.
    unsafe { libc::_exit(0) }
}

/// Close every descriptor except stderr, the pipe and `keep`.
///
/// # Safety
/// Must only be called in the guardian child.
unsafe fn close_other_fds(pipe_r: RawFd, keep: &[RawFd]) {
    let fds: Vec<RawFd> = std::fs::read_dir("/proc/self/fd")
        .map(|d| d.flatten().filter_map(|e| e.file_name().to_str()?.parse().ok()).collect())
        .unwrap_or_default();
    for fd in fds {
        if fd != 2 && fd != pipe_r && !keep.contains(&fd) {
            // SAFETY: closing descriptors in our own process.
            unsafe { libc::close(fd) };
        }
    }
}
