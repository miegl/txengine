//! txengine – FM/RDS transmitter for the Raspberry Pi.

mod analyze;
mod cli;
mod clock;
mod control;
mod engine;
mod hw;
mod logger;
mod programme;
mod rt;
mod sinks;

use std::time::Duration;

use anyhow::Context;
use clap::Parser;
use log::{error, info};
use txengine_core::{Modulator, RfSink};

use cli::{Cli, Command, RenderFormat};

fn main() {
    let cli = Cli::parse();
    logger::init(cli.verbose, cli.quiet);
    if !matches!(cli.command, Command::Tx(_)) {
        // Reporting commands: end quietly when piped into e.g. `head`.
        // SAFETY: resetting a signal disposition.
        unsafe { libc::signal(libc::SIGPIPE, libc::SIG_DFL) };
    }
    let result = match &cli.command {
        Command::Tx(a) => transmit(a),
        Command::Render(a) => render(a),
        Command::Probe(a) => hw::probe(&a.hw),
        Command::Analyze(a) => analyze::run(a),
        Command::Tone(a) => tone(a),
    };
    if let Err(e) = result {
        error!("{e:#}");
        std::process::exit(1);
    }
}

fn transmit(a: &cli::TxArgs) -> anyhow::Result<()> {
    // Validate everything that does not need the hardware first.
    programme::mpx_config(&a.prog)?;
    programme::rds_params(&a.prog)?;
    let _lock = rt::instance_lock()?;
    rt::install_signal_handlers();

    // The hardware is opened while still single threaded: it forks the
    // shutdown guardian.
    let mut sink = hw::open_transmitter(&a.hw)?;
    let mut m = programme::build_modulator(&a.prog, sink.info().sample_rate, true)?;
    m.configure(sink.info()).map_err(anyhow::Error::msg)?;
    let mut clock = clock::ClockDiscipline::new(a.hw.ppm, Box::new(clock::SystemClock::default()));

    let control = if a.ctl.is_some() || a.ctl_socket.is_some() {
        let (tx, rx) = control::channel_pair();
        if let Some(p) = &a.ctl {
            control::spawn_fifo(p, tx.clone())?;
        }
        if let Some(p) = &a.ctl_socket {
            control::spawn_socket(p, tx)?;
        }
        Some(rx)
    } else {
        None
    };

    rt::make_realtime(a.rt_priority);
    let opts = engine::EngineOptions {
        duration: a.duration,
        status_interval: (a.status_interval > 0.0).then(|| Duration::from_secs_f64(a.status_interval)),
        start_time: None,
    };
    let summary = engine::run(sink.as_mut(), &mut m, Some(&mut clock), control, &opts);
    // Always release the hardware, even after an error.
    sink.stop().ok();
    let summary = summary?;
    info!(
        "stopped ({}): {:.1} s, {} samples, {} underruns",
        summary.reason, summary.seconds, summary.samples, summary.underruns
    );
    Ok(())
}

fn tone(a: &cli::ToneArgs) -> anyhow::Result<()> {
    if a.tones.iter().any(|&f| !(0.0..=120_000.0).contains(&f)) {
        anyhow::bail!("--tones must be within 0-120000 Hz");
    }
    let _lock = rt::instance_lock()?;
    rt::install_signal_handlers();
    let mut sink = hw::open_transmitter(&a.hw)?;
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
            *x = osc.iter_mut().map(|o| {
                let v = table.sin(o.harmonic_frac(1));
                o.advance();
                dev * v
            }).sum();
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

fn render(a: &cli::RenderArgs) -> anyhow::Result<()> {
    let fs = a.sample_rate;
    let mut sink: Box<dyn RfSink> = match a.format {
        RenderFormat::Mpx => {
            let p = a.output.as_ref().context("--output is required")?;
            Box::new(sinks::FileSink::mpx_wav(p, fs, a.hold)?)
        }
        RenderFormat::Iq => {
            let p = a.output.as_ref().context("--output is required")?;
            Box::new(sinks::FileSink::iq(p, fs, a.hold)?)
        }
        RenderFormat::Null => Box::new(sinks::FileSink::null(fs)),
    };
    let mut prog = a.prog.clone();
    if prog.audio.is_some() && prog.on_end.is_none() {
        // A rendered file should end with the input.
        prog.on_end = Some(cli::OnEnd::Stop);
    }
    let mut m = programme::build_modulator(&prog, fs, false)?;
    m.configure(sink.info()).map_err(anyhow::Error::msg)?;
    let start = a.start_time.unwrap_or_else(rt::unix_now);
    let opts = engine::EngineOptions { duration: None, status_interval: None, start_time: Some(start) };
    // Render exactly `duration` seconds.
    let total = (a.duration * fs) as u64;
    if let Some(p) = &a.output {
        // Record the timeline so `analyze` can check clock-time placement.
        std::fs::write(format!("{}.json", p.display()), format!("{{\"start_time\": {start:.6}}}\n"))?;
    }
    let mut limited = Limited { inner: sink.as_mut(), left: total };
    let t = std::time::Instant::now();
    let s = engine::run(&mut limited, &mut m, None, None, &opts)?;
    info!(
        "rendered {:.2} s of signal in {:.2} s ({:.1}x real time)",
        s.samples as f64 / fs,
        t.elapsed().as_secs_f64(),
        s.samples as f64 / fs / t.elapsed().as_secs_f64().max(1e-9)
    );
    let st = m.take_stats().unwrap_or_default();
    info!(
        "peak deviation {:.1} kHz, MPX power {:+.2} dBr (last 60 s), HF limiter {:+.1} dB, limiter {:+.1} dB, clipped {}",
        st.peak_deviation / 1e3,
        st.mpx_power_dbr,
        -st.hf_limiter_db,
        20.0 * (st.limiter_gain.max(1e-6) as f64).log10(),
        st.clipped
    );
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
