//! Stereo-linked look-ahead peak limiter.
//!
//! The signal is processed in frames of `F` samples (for the FM multiplex:
//! one frame per audio-rate sample, i.e. the `F` output samples of the
//! interpolator). The caller supplies each frame's peak; the gain
//! computation runs once per frame, and the gain is interpolated linearly
//! across the samples of a frame, so it is continuous.
//!
//! With `req(i)` the gain frame `i` needs, the held gain is the minimum of
//! `req` over the last `L + 1` frames and the frame gain `g(j)` is the mean
//! of the held gain over frames `j .. j + L - 1`. Every term of that mean,
//! and every term of the mean for `g(j + 1)`, includes `req(j)` in its
//! minimum, so all samples of frame `j` (gains between `g(j)` and `g(j + 1)`)
//! are guaranteed to stay below the threshold. Frames leave the limiter `L`
//! frames late. A release follower slows the gain recovery.

#[derive(Debug, Clone)]
pub struct FrameLimiter {
    threshold: f32,
    frame: usize,
    /// Look-ahead `L` in frames.
    look: usize,
    /// Delay line, `L + 1` slots of `[a; frame] [b; frame]`.
    delay: Vec<f32>,
    slot: usize,
    /// Monotonic queue (ring) of (frame index, required gain).
    q_idx: Vec<u64>,
    q_val: Vec<f32>,
    q_head: usize,
    q_len: usize,
    boxbuf: Vec<f32>,
    boxsum: f32,
    bpos: usize,
    n: u64,
    follower: f32,
    release: f32,
    gain: f32,
    min_gain: f32,
}

impl FrameLimiter {
    /// `frame_rate`: frames per second; `lookahead` and `release` in seconds.
    pub fn new(frame: usize, frame_rate: f64, threshold: f32, lookahead: f64, release: f64) -> FrameLimiter {
        assert!(frame >= 1);
        let look = ((lookahead * frame_rate).round() as usize).max(1);
        FrameLimiter {
            threshold,
            frame,
            look,
            delay: vec![0.0; (look + 1) * 2 * frame],
            slot: 0,
            q_idx: vec![0; look + 2],
            q_val: vec![1.0; look + 2],
            q_head: 0,
            q_len: 0,
            boxbuf: vec![1.0; look],
            boxsum: look as f32,
            bpos: 0,
            n: 0,
            follower: 1.0,
            release: (1.0 - (-1.0 / (release * frame_rate)).exp()) as f32,
            gain: 1.0,
            min_gain: 1.0,
        }
    }

    /// Latency in samples.
    pub fn latency(&self) -> usize {
        self.look * self.frame
    }

    /// Lowest gain applied since the last call (for metering).
    pub fn take_min_gain(&mut self) -> f32 {
        std::mem::replace(&mut self.min_gain, 1.0)
    }

    /// Next frame gain for a frame whose peak is `peak`.
    #[inline]
    fn frame_gain(&mut self, peak: f32) -> f32 {
        let req = if peak > self.threshold { self.threshold / peak } else { 1.0 };
        let cap = self.q_idx.len();
        let wrap = |i: usize| if i >= cap { i - cap } else { i };
        // Sliding minimum over frames n - L ..= n.
        while self.q_len > 0 {
            let back = wrap(self.q_head + self.q_len - 1);
            if self.q_val[back] < req {
                break;
            }
            self.q_len -= 1;
        }
        let tail = wrap(self.q_head + self.q_len);
        self.q_idx[tail] = self.n;
        self.q_val[tail] = req;
        self.q_len += 1;
        if self.q_idx[self.q_head] + (self.look as u64) < self.n {
            self.q_head = wrap(self.q_head + 1);
            self.q_len -= 1;
        }
        let held = self.q_val[self.q_head];
        self.n += 1;

        // Mean of the held gain over the last L frames.
        self.boxsum += held - self.boxbuf[self.bpos];
        self.boxbuf[self.bpos] = held;
        self.bpos += 1;
        if self.bpos == self.look {
            self.bpos = 0;
            // Re-sum to cancel accumulated rounding.
            self.boxsum = self.boxbuf.iter().sum();
        }
        let avg = crate::fmin(self.boxsum / self.look as f32, 1.0);

        // Attack instantly to the averaged gain, release slowly.
        if avg < self.follower {
            self.follower = avg;
        } else {
            self.follower += (avg - self.follower) * self.release;
        }
        crate::fmin(self.follower, avg)
    }

    /// Push one frame (`a`, `b`, each `frame` samples, with `peak` the
    /// largest value the threshold applies to) and write the limited frame
    /// from `L` frames earlier to `a_out` / `b_out`.
    #[inline]
    pub fn process(&mut self, peak: f32, a: &[f32], b: &[f32], a_out: &mut [f32], b_out: &mut [f32]) {
        let f = self.frame;
        let g1 = self.frame_gain(peak);
        let g0 = self.gain;
        self.gain = g1;
        self.min_gain = crate::fmin(self.min_gain, g0);

        let slots = self.look + 1;
        let out_slot = if self.slot + 1 == slots { 0 } else { self.slot + 1 };
        let (w, r) = (self.slot * 2 * f, out_slot * 2 * f);
        let step = (g1 - g0) / f as f32;
        for p in 0..f {
            self.delay[w + p] = a[p];
            self.delay[w + f + p] = b[p];
            let g = g0 + step * p as f32;
            a_out[p] = self.delay[r + p] * g;
            b_out[p] = self.delay[r + f + p] * g;
        }
        self.slot = out_slot;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(frame: usize, look: f64, input: impl Fn(usize) -> (f32, f32), n: usize) -> (Vec<(f32, f32)>, FrameLimiter) {
        let fr = 48_000.0;
        let mut lim = FrameLimiter::new(frame, fr, 1.0, look, 0.1);
        let mut out = Vec::new();
        let (mut a, mut b) = (vec![0.0; frame], vec![0.0; frame]);
        let (mut oa, mut ob) = (vec![0.0; frame], vec![0.0; frame]);
        for k in 0..n / frame {
            let mut peak: f32 = 0.0;
            for p in 0..frame {
                (a[p], b[p]) = input(k * frame + p);
                peak = peak.max(a[p].abs().max(b[p].abs()));
            }
            lim.process(peak, &a, &b, &mut oa, &mut ob);
            out.extend(oa.iter().zip(&ob).map(|(&x, &y)| (x, y)));
        }
        (out, lim)
    }

    #[test]
    fn never_exceeds_threshold() {
        for frame in [1, 5] {
            let noise = |n: usize| {
                let seed = (n as u32).wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                (seed >> 8) as f32 / (1 << 24) as f32 - 0.5
            };
            let x = |n: usize| {
                let burst = if (n / 5000).is_multiple_of(3) { 4.0 } else { 0.3 };
                let v = burst * ((n as f32 * 0.05).sin() + noise(n));
                (v, -0.7 * v)
            };
            let (out, _) = run(frame, 0.0015, x, 200_000);
            let worst = out.iter().fold(0f32, |m, &(a, b)| m.max(a.abs().max(b.abs())));
            assert!(worst <= 1.0 + 1e-5, "frame {frame}: peak {worst}");
        }
    }

    #[test]
    fn transparent_below_threshold() {
        let (out, lim) = run(5, 0.001, |n| (0.5 * (n as f32 * 0.1).sin(), 0.0), 1000);
        let lat = lim.latency();
        for (n, &(a, _)) in out.iter().enumerate().skip(lat) {
            assert!((a - 0.5 * ((n - lat) as f32 * 0.1).sin()).abs() < 1e-6);
        }
    }
}
