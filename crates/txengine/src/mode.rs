//! Running a transmission mode: transmitting, or rendering to a file. A mode
//! plugs in by implementing [`Mode`] for its command line options.

use std::time::Duration;

use anyhow::Context;
use log::info;
use txengine_core::{Modulator, RfSink};

use crate::cli::{ControlArgs, HardwareArgs, OutputArgs, RenderFormat};
use crate::{clock, control, engine, hw, rt, sinks};

/// A transmission mode, set up from its command line options.
pub trait Mode {
    /// Largest carrier frequency offset the mode needs from the
    /// transmitter, in Hz.
    fn max_offset_hz(&self) -> f64;

    /// Check the options before anything is opened. `carrier_mhz`: the
    /// carrier frequency, when transmitting.
    fn check(&self, carrier_mhz: Option<f64>) -> anyhow::Result<()>;

    /// Create the modulator for an output running at `sample_rate` (in real
    /// time for a transmitter).
    fn modulator(&self, sample_rate: f64, realtime: bool) -> anyhow::Result<Box<dyn Modulator>>;
}

pub fn transmit(hw: &HardwareArgs, ctl: &ControlArgs, mode: &dyn Mode) -> anyhow::Result<()> {
    // Validate everything that does not need the hardware first.
    mode.check(Some(hw.freq))?;
    let _lock = rt::instance_lock()?;
    rt::install_signal_handlers();

    // The hardware is opened while still single threaded: it forks the
    // shutdown guardian.
    let mut sink = hw::open_transmitter(hw, mode.max_offset_hz())?;
    let mut m = mode.modulator(sink.info().sample_rate, true)?;
    m.configure(sink.info()).map_err(anyhow::Error::msg)?;
    let mut clock = clock::ClockDiscipline::new(hw.ppm, Box::new(clock::SystemClock::default()));

    let control = if ctl.ctl.is_some() || ctl.ctl_socket.is_some() {
        let (tx, rx) = control::channel_pair();
        if let Some(p) = &ctl.ctl {
            control::spawn_fifo(p, tx.clone())?;
        }
        if let Some(p) = &ctl.ctl_socket {
            control::spawn_socket(p, tx)?;
        }
        Some(rx)
    } else {
        None
    };

    rt::make_realtime(ctl.rt_priority);
    let opts = engine::EngineOptions {
        duration: ctl.duration,
        status_interval: (ctl.status_interval > 0.0).then(|| Duration::from_secs_f64(ctl.status_interval)),
        start_time: None,
    };
    let summary = engine::run(sink.as_mut(), m.as_mut(), Some(&mut clock), control, &opts);
    // Always release the hardware, even after an error.
    sink.stop().ok();
    let summary = summary?;
    info!(
        "stopped ({}): {:.1} s, {} samples, {} underruns",
        summary.reason, summary.seconds, summary.samples, summary.underruns
    );
    Ok(())
}

pub fn render(out: &OutputArgs, mode: &dyn Mode) -> anyhow::Result<()> {
    mode.check(None)?;
    let fs = out.sample_rate;
    let mut sink: Box<dyn RfSink> = match out.format {
        RenderFormat::Deviation => {
            let p = out.output.as_ref().context("--output is required")?;
            Box::new(sinks::FileSink::deviation_wav(p, fs, out.hold)?)
        }
        RenderFormat::Iq => {
            let p = out.output.as_ref().context("--output is required")?;
            Box::new(sinks::FileSink::iq(p, fs, out.hold)?)
        }
        RenderFormat::Null => Box::new(sinks::FileSink::null(fs)),
    };
    let mut m = mode.modulator(fs, false)?;
    m.configure(sink.info()).map_err(anyhow::Error::msg)?;
    let start = out.start_time.unwrap_or_else(rt::unix_now);
    let opts = engine::EngineOptions { duration: None, status_interval: None, start_time: Some(start) };
    // Render exactly `duration` seconds.
    let total = (out.duration * fs) as u64;
    if let Some(p) = &out.output {
        // Record the timeline so `analyze` can check clock-time placement.
        std::fs::write(format!("{}.json", p.display()), format!("{{\"start_time\": {start:.6}}}\n"))?;
    }
    let mut limited = Limited { inner: sink.as_mut(), left: total };
    let t = std::time::Instant::now();
    let s = engine::run(&mut limited, m.as_mut(), None, None, &opts)?;
    info!(
        "rendered {:.2} s of signal in {:.2} s ({:.1}x real time)",
        s.samples as f64 / fs,
        t.elapsed().as_secs_f64(),
        s.samples as f64 / fs / t.elapsed().as_secs_f64().max(1e-9)
    );
    let meters = m.take_meters();
    if !meters.is_empty() {
        info!("{meters}");
    }
    Ok(())
}

/// Wraps a software sink so that it accepts exactly `left` more samples and
/// then requests a stop.
struct Limited<'a> {
    inner: &'a mut dyn RfSink,
    left: u64,
}

impl RfSink for Limited<'_> {
    fn info(&self) -> &txengine_core::SinkInfo {
        self.inner.info()
    }
    fn start(&mut self) -> Result<(), txengine_core::SinkError> {
        self.inner.start()
    }
    fn poll(&mut self) -> Result<txengine_core::SinkPoll, txengine_core::SinkError> {
        let mut p = self.inner.poll()?;
        p.writable = p.writable.min(self.left as usize);
        if self.left == 0 {
            rt::request_stop();
        }
        Ok(p)
    }
    fn write(&mut self, samples: &[f32]) -> Result<(), txengine_core::SinkError> {
        self.left = self.left.saturating_sub(samples.len() as u64);
        self.inner.write(samples)
    }
    fn stop(&mut self) -> Result<(), txengine_core::SinkError> {
        self.inner.stop()
    }
}
