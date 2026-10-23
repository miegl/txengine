//! Running a transmission mode: transmitting, or rendering to a file. A mode
//! plugs in by implementing [`Mode`] for its command line options. Also the
//! modes' hardware tools: the plan for a mode, and raw test tones.

use std::time::Duration;

use anyhow::Context;
use log::info;
use txengine_core::{Modulator, RfSink, SinkInfo};

use crate::cli::{ControlArgs, HardwareArgs, OutputArgs, ProbeArgs, RenderFormat, ToneArgs};
use crate::{clock, control, engine, hw, rt, sinks};

/// A transmission mode, set up from its command line options.
pub trait Mode {
    /// What the mode needs from the transmitter.
    fn output(&self) -> hw::Output;

    /// Check the options before anything is opened. `carrier_hz`: the
    /// carrier frequency, when transmitting.
    fn check(&self, carrier_hz: Option<f64>) -> anyhow::Result<()>;

    /// Create the modulator for the output `sink` (a transmitter, or a file).
    fn modulator(&self, sink: &SinkInfo) -> anyhow::Result<Box<dyn Modulator>>;
}

pub fn transmit(hw: &HardwareArgs, ctl: &ControlArgs, mode: &dyn Mode) -> anyhow::Result<()> {
    // Validate everything that does not need the hardware first.
    mode.check(Some(hw.freq_hz))?;
    let device = hw::select(hw)?;
    let _lock = rt::instance_lock(&device.lock_name()?)?;
    rt::install_signal_handlers();

    // The hardware is opened while still single threaded: it forks the
    // shutdown guardian.
    let mut sink = device.open(hw, mode.output())?;
    let mut m = mode.modulator(sink.info())?;
    m.configure(sink.info()).map_err(anyhow::Error::msg)?;
    let mut clock = clock::ClockDiscipline::new(hw.ppm, Box::new(clock::SystemClock::default()));

    // The control channels' files live as long as the transmission.
    let mut _endpoints = Vec::new();
    let control = if ctl.ctl.is_some() || ctl.ctl_socket.is_some() {
        let (tx, rx) = control::channel_pair();
        if let Some(p) = &ctl.ctl {
            _endpoints.push(control::spawn_fifo(p, tx.clone())?);
        }
        if let Some(p) = &ctl.ctl_socket {
            _endpoints.push(control::spawn_socket(p, tx)?);
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
    let format = mode.output().format();
    let mut sink: Box<dyn RfSink> = match out.format {
        RenderFormat::Wav => {
            let p = out.output.as_ref().context("--output is required")?;
            // A full-scale sample is the largest deviation the mode plans for.
            let full_scale = match mode.output() {
                hw::Output::Frequency { max_offset_hz } => max_offset_hz,
                hw::Output::Amplitude => 1.0,
            };
            Box::new(sinks::FileSink::wav(p, fs, format, full_scale, out.hold)?)
        }
        RenderFormat::Iq => {
            let p = out.output.as_ref().context("--output is required")?;
            Box::new(sinks::FileSink::iq(p, fs, format, out.hold)?)
        }
        RenderFormat::Null => Box::new(sinks::FileSink::null(fs, format)),
    };
    let mut m = mode.modulator(sink.info())?;
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

/// Show the hardware and its plan for a mode that needs `out`.
pub fn probe(a: &ProbeArgs, out: hw::Output) -> anyhow::Result<()> {
    hw::select(&a.hw)?.probe(&a.hw, out)
}

/// Transmit raw test tones from the transmitter planned, as for an FM mode,
/// for carrier offsets up to `max_offset_hz`, so that the measurement
/// applies to that mode.
pub fn tone(a: &ToneArgs, max_offset_hz: f64) -> anyhow::Result<()> {
    if a.tones.iter().any(|&f| !(0.0..=120_000.0).contains(&f)) {
        anyhow::bail!("--tones must be within 0-120000 Hz");
    }
    if a.deviation * a.tones.len() as f64 > max_offset_hz {
        anyhow::bail!("the tones together may deviate at most {max_offset_hz:.0} Hz in this mode");
    }
    let device = hw::select(&a.hw)?;
    let _lock = rt::instance_lock(&device.lock_name()?)?;
    rt::install_signal_handlers();
    let mut sink = device.open(&a.hw, hw::Output::Frequency { max_offset_hz })?;
    let fs = sink.info().sample_rate;
    if a.tones.iter().any(|&f| f >= fs / 2.0) {
        anyhow::bail!("tones must be below half the sample rate ({:.0} Hz)", fs / 2.0);
    }
    info!("tones {:?} Hz at ±{:.0} Hz each, sample rate {:.3} kS/s", a.tones, a.deviation, fs / 1e3);
    // Only the carrier is corrected: the tones are raw test signals.
    let mut clock = clock::ClockDiscipline::new(a.hw.ppm, Box::new(clock::SystemClock::default()));
    clock.attach(sink.as_mut())?;
    rt::make_realtime(30);
    let table = txengine_dsp::SineTable::new();
    let mut osc: Vec<_> = a.tones.iter().map(|&f| txengine_dsp::PhaseAccumulator::new(f, fs)).collect();
    let dev = a.deviation as f32;
    let mut buf = vec![0f32; 4096];
    let mut fill = |buf: &mut [f32]| {
        for x in buf.iter_mut() {
            *x = osc
                .iter_mut()
                .map(|o| {
                    let v = table.sin(o.harmonic_frac(1));
                    o.advance();
                    dev * v
                })
                .sum();
        }
    };
    let p = sink.poll()?;
    let k = p.writable.min(buf.len());
    fill(&mut buf[..k]);
    sink.write(&buf[..k])?;
    sink.start()?;
    let start = std::time::Instant::now();
    let result = (|| -> anyhow::Result<u64> {
        while !rt::stop_requested() && start.elapsed().as_secs_f64() < a.duration {
            let p = clock.poll(sink.as_mut())?;
            if p.writable >= 1024 {
                let k = p.writable.min(buf.len());
                fill(&mut buf[..k]);
                sink.write(&buf[..k])?;
            } else {
                std::thread::sleep(Duration::from_millis(2));
            }
        }
        Ok(sink.poll()?.underruns)
    })();
    sink.stop().ok();
    info!("stopped after {:.1} s, {} underruns; {}", start.elapsed().as_secs_f64(), result?, clock.describe());
    Ok(())
}
