//! Minimal stderr logger with relative timestamps.

use std::io::Write;
use std::time::Instant;

use log::{Level, LevelFilter, Log, Metadata, Record};

struct Logger {
    start: Instant,
    level: LevelFilter,
}

impl Log for Logger {
    fn enabled(&self, m: &Metadata) -> bool {
        m.level() <= self.level
    }

    fn log(&self, r: &Record) {
        if !self.enabled(r.metadata()) {
            return;
        }
        let t = self.start.elapsed().as_secs_f64();
        let tag = match r.level() {
            Level::Error => "ERROR",
            Level::Warn => "WARN ",
            Level::Info => "INFO ",
            Level::Debug => "DEBUG",
            Level::Trace => "TRACE",
        };
        let module = r.target().split("::").next().unwrap_or("");
        let mut e = std::io::stderr().lock();
        if self.level >= LevelFilter::Debug {
            let _ = writeln!(e, "[{t:9.3}] {tag} {module}: {}", r.args());
        } else {
            let _ = writeln!(e, "[{t:9.3}] {tag} {}", r.args());
        }
    }

    fn flush(&self) {}
}

pub fn init(verbosity: u8, quiet: bool) {
    let level = if quiet {
        LevelFilter::Warn
    } else {
        match verbosity {
            0 => LevelFilter::Info,
            1 => LevelFilter::Debug,
            _ => LevelFilter::Trace,
        }
    };
    let logger = Box::new(Logger { start: Instant::now(), level });
    if log::set_boxed_logger(logger).is_ok() {
        log::set_max_level(level);
    }
}
