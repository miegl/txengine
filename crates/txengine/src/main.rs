//! txengine – FM/RDS transmitter for the Raspberry Pi.

mod cli;
mod clock;
mod control;
mod engine;
mod hw;
mod logger;
mod mode;
mod rt;
mod sinks;
mod wfm;

use std::time::Duration;

use clap::Parser;
use log::{error, info};

use cli::{Cli, Command};

fn main() {
    let cli = Cli::parse();
    logger::init(cli.verbose, cli.quiet);
    // Reporting commands end quietly when piped into e.g. `head`;
    // transmitting ones ignore SIGPIPE again (`rt::install_signal_handlers`).
    // SAFETY: resetting a signal disposition.
    unsafe { libc::signal(libc::SIGPIPE, libc::SIG_DFL) };
    let result = match &cli.command {
        Command::Wfm(c) => wfm::run(c),
        Command::Probe(a) => hw::select(&a.hw).and_then(|d| d.probe(&a.hw, wfm::MAX_OFFSET_HZ)),
        Command::Devices => hw::list_devices(),
        Command::Tone(a) => tone(a),
    };
    if let Err(e) = result {
        error!("{e:#}");
        std::process::exit(1);
    }
}

fn tone(a: &cli::ToneArgs) -> anyhow::Result<()> {
    if a.tones.iter().any(|&f| !(0.0..=120_000.0).contains(&f)) {
        anyhow::bail!("--tones must be within 0-120000 Hz");
    }
    let device = hw::select(&a.hw)?;
    let _lock = rt::instance_lock(&device.lock_name()?)?;
    rt::install_signal_handlers();
    // Planned as for WFM, so that the measured response applies to it.
    let mut sink = device.open(&a.hw, wfm::MAX_OFFSET_HZ)?;
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
