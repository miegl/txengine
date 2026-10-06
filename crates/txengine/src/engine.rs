//! The transmit loop: keeps the sink's buffer filled with rendered samples.

use std::sync::mpsc::Receiver;
use std::time::{Duration, Instant};

use log::{info, warn};
use txengine_core::{CommandError, Modulator, RenderContext, RfSink};
use txengine_fm::WfmModulator;

use crate::control::ControlMsg;
use crate::rt;

pub struct EngineOptions {
    pub duration: Option<f64>,
    pub status_interval: Option<Duration>,
    /// Unix time of the first sample for non-real-time outputs.
    pub start_time: Option<f64>,
}

/// Summary returned when the engine stops.
#[derive(Debug, Default)]
pub struct EngineSummary {
    pub samples: u64,
    pub seconds: f64,
    pub underruns: u64,
    pub reason: String,
}

fn handle_command(m: &mut WfmModulator, msg: ControlMsg, status: &dyn Fn() -> String) {
    let line = msg.line.trim();
    let reply = match line.to_ascii_uppercase().as_str() {
        "QUIT" | "STOP" | "EXIT" => {
            rt::request_stop();
            "OK stopping".to_string()
        }
        "STATUS" => {
            let mut s = m.command("STATUS").unwrap_or_default();
            s.push(' ');
            s.push_str(&status());
            format!("OK {s}")
        }
        _ => match m.command(line) {
            Ok(r) => {
                info!("{r}");
                format!("OK {r}")
            }
            Err(e @ CommandError::Unknown(_)) | Err(e @ CommandError::Invalid(_)) => {
                warn!("control: {line}: {e}");
                format!("ERR {e}")
            }
        },
    };
    if let Some(tx) = msg.reply {
        let _ = tx.send(reply);
    }
}

/// Run until stopped. The sink must not have been started yet.
pub fn run(
    sink: &mut dyn RfSink,
    m: &mut WfmModulator,
    control: Option<Receiver<ControlMsg>>,
    opts: &EngineOptions,
) -> anyhow::Result<EngineSummary> {
    let info = sink.info().clone();
    let fs = info.sample_rate;
    let mut buf = vec![0f32; info.buffer_len.max(1 << 14)];
    // Render in chunks of ~4 ms: small enough to keep latency and command
    // response low, large enough to be efficient.
    let chunk_min = ((fs * 0.004) as usize).max(256);

    // Emission time of sample k: for hardware, derived from the buffer fill
    // at render time; for files, a virtual timeline from `t_start`.
    let t_start = opts.start_time.unwrap_or_else(rt::unix_now);
    // Pre-fill the whole buffer before starting the output.
    let p = sink.poll()?;
    let n = p.writable.min(buf.len());
    let ctx = RenderContext { first_sample: 0, sample_rate: fs, emit_time_unix: Some(if info.realtime { rt::unix_now() } else { t_start }) };
    m.render(&ctx, &mut buf[..n]);
    sink.write(&buf[..n])?;
    sink.start()?;
    let start = Instant::now();
    info!("transmitting");

    let mut last_status = Instant::now();
    let mut last_consumed = 0u64;
    let mut last_underruns = 0u64;
    let mut summary = EngineSummary::default();
    let reason;
    loop {
        if rt::stop_requested() {
            reason = match rt::stop_signal() {
                Some(s) => format!("signal {s}"),
                None => "stop requested".into(),
            };
            break;
        }
        if let Some(d) = opts.duration
            && start.elapsed().as_secs_f64() >= d {
                reason = "duration reached".into();
                break;
            }
        if m.finished() {
            reason = "end of input".into();
            break;
        }
        if let Some(rx) = &control {
            while let Ok(msg) = rx.try_recv() {
                let consumed = last_consumed;
                handle_command(m, msg, &|| format!("samples={consumed} underruns={last_underruns}"));
            }
        }

        let p = sink.poll()?;
        if p.underruns != last_underruns {
            warn!("output underrun (#{}) – the transmitter could not keep up", p.underruns);
            last_underruns = p.underruns;
        }
        last_consumed = p.consumed;
        if p.writable >= chunk_min || (!info.realtime && p.writable > 0) {
            let n = p.writable.min(buf.len());
            // Sample `written` leaves the antenna after everything still
            // queued in the hardware buffer.
            let emit = Some(if info.realtime {
                rt::unix_now() + (p.written.saturating_sub(p.consumed)) as f64 / fs
            } else {
                t_start + p.written as f64 / fs
            });
            let ctx = RenderContext { first_sample: p.written, sample_rate: fs, emit_time_unix: emit };
            m.render(&ctx, &mut buf[..n]);
            sink.write(&buf[..n])?;
        } else if info.realtime {
            let wait = (chunk_min - p.writable) as f64 / fs;
            std::thread::sleep(Duration::from_secs_f64(wait.clamp(0.0005, 0.02)));
        }

        if let Some(iv) = opts.status_interval
            && last_status.elapsed() >= iv
        {
            let dt = last_status.elapsed().as_secs_f64();
            let st = m.take_stats().unwrap_or_default();
            let fields = m.status().fields;
            let field = |name: &str| fields.iter().find(|(k, _)| k == name).map(|(_, v)| v.clone());
            let audio = field("audio_buffer").map(|v| format!(", audio buffer {v}")).unwrap_or_default();
            let drift = field("audio_drift_ppm").map(|v| format!(", input drift {v} ppm")).unwrap_or_default();
            info!(
                "status: {:.1} s, rate {:.1} S/s, peak deviation {:.1} kHz, limiter {:+.1} dB, clipped {}, audio underruns {}, output underruns {}{audio}{drift}",
                start.elapsed().as_secs_f64(),
                (p.consumed - summary.samples) as f64 / dt,
                st.peak_deviation / 1e3,
                20.0 * (st.limiter_gain.max(1e-6) as f64).log10(),
                st.clipped,
                st.audio_underruns,
                p.underruns,
            );
            summary.samples = p.consumed;
            last_status = Instant::now();
        }
    }
    let p = sink.poll()?;
    sink.stop()?;
    summary.samples = p.consumed.max(p.written * (!info.realtime) as u64);
    summary.seconds = start.elapsed().as_secs_f64();
    summary.underruns = p.underruns;
    summary.reason = reason;
    Ok(summary)
}
