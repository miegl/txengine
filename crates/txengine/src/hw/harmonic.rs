//! Transmitting on a harmonic of the carrier (`--harmonic N`).
//!
//! The pin's square wave has harmonics at whole multiples of its frequency,
//! and a frequency modulation moves the `n`-th of them `n` times as far as
//! the fundamental. A carrier above what an output makes directly can
//! therefore be transmitted on a harmonic: the hardware makes `f / n`, and
//! every offset is divided by `n` on its way to it. The fundamental stays
//! far stronger than the wanted harmonic, so the output must be band-pass
//! filtered at `f`.

use txengine_core::{RfSink, SinkError, SinkInfo, SinkPoll};

/// A frequency-modulating sink seen at the `n`-th harmonic of its carrier.
pub struct HarmonicSink {
    inner: Box<dyn RfSink>,
    n: f32,
    info: SinkInfo,
    /// The samples scaled to the fundamental.
    scaled: Vec<f32>,
}

impl HarmonicSink {
    pub fn new(inner: Box<dyn RfSink>, n: u32) -> HarmonicSink {
        let mut info = inner.info().clone();
        let k = n as f64;
        info.description = format!(
            "{}; transmitting on harmonic {n} of {:.6} MHz: {:.6} MHz",
            info.description,
            info.carrier_hz / 1e6,
            info.carrier_hz * k / 1e6
        );
        info.carrier_hz *= k;
        info.max_offset_hz *= k;
        info.resolution_hz *= k;
        // Engine writes are at most this long; reserved here, not in the
        // real-time path.
        let scaled = Vec::with_capacity(info.buffer_len.max(1 << 14));
        HarmonicSink { inner, n: n as f32, info, scaled }
    }
}

impl RfSink for HarmonicSink {
    fn info(&self) -> &SinkInfo {
        &self.info
    }

    fn start(&mut self) -> Result<(), SinkError> {
        self.inner.start()
    }

    fn poll(&mut self) -> Result<SinkPoll, SinkError> {
        self.inner.poll()
    }

    fn write(&mut self, samples: &[f32]) -> Result<(), SinkError> {
        let k = 1.0 / self.n;
        self.scaled.clear();
        self.scaled.extend(samples.iter().map(|&s| s * k));
        self.inner.write(&self.scaled)
    }

    fn stop(&mut self) -> Result<(), SinkError> {
        self.inner.stop()
    }

    /// A crystal error moves every harmonic by the same fraction.
    fn set_clock_error_ppm(&mut self, ppm: f64) -> Result<(), SinkError> {
        self.inner.set_clock_error_ppm(ppm)
    }
}

#[cfg(test)]
mod tests {
    use std::sync::{Arc, Mutex};

    use txengine_core::SampleFormat;

    use super::*;

    /// Records what reaches the hardware.
    struct Fake {
        info: SinkInfo,
        written: Arc<Mutex<Vec<f32>>>,
        ppm: Arc<Mutex<f64>>,
    }

    impl RfSink for Fake {
        fn info(&self) -> &SinkInfo {
            &self.info
        }
        fn start(&mut self) -> Result<(), SinkError> {
            Ok(())
        }
        fn poll(&mut self) -> Result<SinkPoll, SinkError> {
            Ok(SinkPoll::default())
        }
        fn write(&mut self, samples: &[f32]) -> Result<(), SinkError> {
            self.written.lock().unwrap().extend_from_slice(samples);
            Ok(())
        }
        fn stop(&mut self) -> Result<(), SinkError> {
            Ok(())
        }
        fn set_clock_error_ppm(&mut self, ppm: f64) -> Result<(), SinkError> {
            *self.ppm.lock().unwrap() = ppm;
            Ok(())
        }
    }

    #[test]
    fn the_harmonic_carries_the_modulation() {
        let info = SinkInfo {
            description: "fake".into(),
            sample_rate: 190e3,
            carrier_hz: 148.666e6,
            max_offset_hz: 3_400.0,
            resolution_hz: 2.0,
            max_clock_error_ppm: 200.0,
            paced_by_carrier: false,
            buffer_len: 4096,
            format: SampleFormat::FrequencyOffset,
            realtime: true,
            zero_order_hold: true,
            response_db: Vec::new(),
        };
        let (written, ppm) = (Arc::new(Mutex::new(Vec::new())), Arc::new(Mutex::new(0.0)));
        let fake = Fake { info, written: written.clone(), ppm: ppm.clone() };
        let mut h = HarmonicSink::new(Box::new(fake), 3);
        let i = h.info();
        assert_eq!((i.carrier_hz, i.max_offset_hz, i.resolution_hz, i.sample_rate), (445.998e6, 10_200.0, 6.0, 190e3));
        h.write(&[3000.0, -1500.0, 0.0]).unwrap();
        assert_eq!(*written.lock().unwrap(), vec![1000.0, -500.0, 0.0]);
        h.set_clock_error_ppm(-54.0).unwrap();
        assert_eq!(*ppm.lock().unwrap(), -54.0);
    }
}
