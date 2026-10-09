//! Runtime control channels: a named pipe (fire and forget) and a Unix
//! socket with replies.
//!
//! Commands are plain text lines such as `PS MyRadio`, `RT Now playing...`,
//! `TA ON`, `PTY 10`. They are forwarded to the transmit thread, which
//! applies them between rendering chunks (so RDS changes take effect at
//! group boundaries, never in the middle of a group).

use std::io::{BufRead, BufReader, Write};
use std::os::unix::fs::FileTypeExt;
use std::os::unix::net::UnixListener;
use std::path::{Path, PathBuf};
use std::sync::mpsc::{Receiver, Sender, channel};

use anyhow::{Context, bail};
use log::{info, warn};

/// A command line and where to send the reply.
#[derive(Debug)]
pub struct ControlMsg {
    pub line: String,
    pub reply: Option<Sender<String>>,
}

/// Longest accepted command line (RT is 64 characters, UTF-8 up to 4 bytes each).
const MAX_LINE: usize = 1024;

/// Read lines, splitting overlong input safely (never on a partial UTF-8
/// character); returns None at EOF.
fn read_line(r: &mut impl BufRead, buf: &mut Vec<u8>) -> std::io::Result<Option<String>> {
    buf.clear();
    let n = std::io::Read::take(&mut *r, MAX_LINE as u64).read_until(b'\n', buf)?;
    if n == 0 {
        return Ok(None);
    }
    if buf.last() != Some(&b'\n') && n == MAX_LINE {
        // Discard the rest of an overlong line.
        let mut sink = Vec::new();
        r.read_until(b'\n', &mut sink)?;
        warn!("control: ignoring a line longer than {MAX_LINE} bytes");
        return Ok(Some(String::new()));
    }
    Ok(Some(String::from_utf8_lossy(buf).trim_end_matches(['\n', '\r']).to_string()))
}

/// A control channel's file, removed when the transmission ends if
/// txengine created it.
#[derive(Debug)]
pub struct Endpoint(Option<PathBuf>);

impl Drop for Endpoint {
    fn drop(&mut self) {
        if let Some(p) = self.0.take() {
            let _ = std::fs::remove_file(p);
        }
    }
}

/// Read commands from a FIFO, creating it if needed.
pub fn spawn_fifo(path: &Path, tx: Sender<ControlMsg>) -> anyhow::Result<Endpoint> {
    let endpoint = match std::fs::metadata(path) {
        Ok(m) if m.file_type().is_fifo() => Endpoint(None),
        Ok(_) => bail!("{} exists and is not a named pipe", path.display()),
        Err(_) => {
            let c = std::ffi::CString::new(path.as_os_str().as_encoded_bytes())?;
            // SAFETY: valid C string.
            if unsafe { libc::mkfifo(c.as_ptr(), 0o660) } != 0 {
                bail!("cannot create FIFO {}: {}", path.display(), std::io::Error::last_os_error());
            }
            info!("created control FIFO {}", path.display());
            Endpoint(Some(path.to_path_buf()))
        }
    };
    // Opening read-write keeps a writer attached, so the reader never sees
    // EOF when a client (e.g. `echo ... > fifo`) closes its end.
    let f = std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .open(path)
        .with_context(|| format!("open {}", path.display()))?;
    let p = path.to_path_buf();
    std::thread::Builder::new().name("txengine-ctl".into()).spawn(move || {
        let mut r = BufReader::new(f);
        let mut buf = Vec::new();
        loop {
            match read_line(&mut r, &mut buf) {
                Ok(Some(line)) if line.trim().is_empty() => {}
                Ok(Some(line)) => {
                    if tx.send(ControlMsg { line, reply: None }).is_err() {
                        return;
                    }
                }
                Ok(None) => return,
                Err(e) => {
                    warn!("control FIFO {}: {e}", p.display());
                    return;
                }
            }
        }
    })?;
    info!("reading commands from {}", path.display());
    Ok(endpoint)
}

/// Accept connections on a Unix socket; every command gets one reply line
/// (`OK ...` or `ERR ...`).
pub fn spawn_socket(path: &Path, tx: Sender<ControlMsg>) -> anyhow::Result<Endpoint> {
    if let Ok(m) = std::fs::symlink_metadata(path) {
        if m.file_type().is_socket() {
            std::fs::remove_file(path).ok();
        } else {
            bail!("{} exists and is not a socket", path.display());
        }
    }
    let listener = UnixListener::bind(path).with_context(|| format!("bind {}", path.display()))?;
    let endpoint = Endpoint(Some(path.to_path_buf()));
    std::thread::Builder::new().name("txengine-sock".into()).spawn(move || {
        for conn in listener.incoming() {
            let Ok(stream) = conn else { continue };
            let tx = tx.clone();
            let _ = std::thread::Builder::new().name("txengine-client".into()).spawn(move || {
                let Ok(mut w) = stream.try_clone() else { return };
                let mut r = BufReader::new(stream);
                let mut buf = Vec::new();
                while let Ok(Some(line)) = read_line(&mut r, &mut buf) {
                    if line.trim().is_empty() {
                        continue;
                    }
                    let (rtx, rrx) = channel();
                    if tx.send(ControlMsg { line, reply: Some(rtx) }).is_err() {
                        return;
                    }
                    let reply = rrx.recv().unwrap_or_else(|_| "ERR transmitter stopped".into());
                    if writeln!(w, "{reply}").is_err() {
                        return;
                    }
                }
            });
        }
    })?;
    info!("control socket listening on {}", path.display());
    Ok(endpoint)
}

/// Control channel endpoints for the engine.
pub fn channel_pair() -> (Sender<ControlMsg>, Receiver<ControlMsg>) {
    channel()
}
