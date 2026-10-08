//! Look-ahead limiters.
//!
//! [`LookaheadGain`] turns the gain each frame requires into a smooth gain
//! that never exceeds the requirement of the frame it is applied to. With
//! `req(i)` the gain frame `i` needs, the held gain is the minimum of `req`
//! over the last `L + 1` frames and the frame gain `g(j)` is the mean of the
//! held gain over frames `j .. j + L - 1`. Every term of that mean, and every
//! term of the mean for `g(j + 1)`, includes `req(j)` in its minimum, so
//! `g(j)` and `g(j + 1)` are both at most `req(j)`. Frames are delayed by
//! `L` frames to meet their gain. A release follower slows the recovery.
//!
//! [`PreemphasisLimiter`] uses it to limit a pre-emphasised programme,
//! reducing the boost of the pre-emphasis before the whole programme.

/// Look-ahead gain computer (see the module documentation).
#[derive(Debug, Clone)]
pub struct LookaheadGain {
    /// Look-ahead `L` in frames.
    look: usize,
    /// Monotonic queue (ring) of (frame index, required gain).
    q_idx: Vec<u64>,
    q_val: Vec<f32>,
    q_head: usize,
    q_len: usize,
    boxbuf: Vec<f32>,
    boxsum: f32,
    bpos: usize,
    n: u64,
    inv_look: f32,
    follower: f32,
    release: f32,
    /// The last output was 1 for a requirement of 1: the queue holds a
    /// single entry of 1, the box only ones and the follower is 1, so
    /// further requirements of 1 change nothing but the queue entry's index.
    settled: bool,
}

impl LookaheadGain {
    /// `frame_rate`: frames per second; `lookahead` and `release` in seconds.
    pub fn new(frame_rate: f64, lookahead: f64, release: f64) -> LookaheadGain {
        let look = ((lookahead * frame_rate).round() as usize).max(1);
        LookaheadGain {
            look,
            q_idx: vec![0; look + 2],
            q_val: vec![1.0; look + 2],
            q_head: 0,
            q_len: 0,
            boxbuf: vec![1.0; look],
            boxsum: look as f32,
            bpos: 0,
            n: 0,
            inv_look: 1.0 / look as f32,
            follower: 1.0,
            release: (1.0 - (-1.0 / (release * frame_rate)).exp()) as f32,
            settled: true,
        }
    }

    /// Look-ahead `L` in frames: the delay the caller must apply.
    pub fn look(&self) -> usize {
        self.look
    }

    /// Push the gain `req` (at most 1) the newest frame needs; returns the
    /// gain for the frame `L` frames earlier.
    #[inline]
    pub fn next(&mut self, req: f32) -> f32 {
        if req >= 1.0 && self.settled {
            if self.q_len > 0 {
                self.q_idx[self.q_head] = self.n;
            }
            self.n += 1;
            return 1.0;
        }
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
        let avg = crate::fmin(self.boxsum * self.inv_look, 1.0);

        // Attack instantly to the averaged gain, release slowly. Close to
        // the target the release step drops below the resolution of f32
        // and the follower would stall short of it: it then jumps there.
        if avg < self.follower {
            self.follower = avg;
        } else {
            let next = self.follower + (avg - self.follower) * self.release;
            self.follower = if next == self.follower { avg } else { next };
        }
        let g = crate::fmin(self.follower, avg);
        self.settled = req >= 1.0 && g >= 1.0;
        g
    }
}

/// Look-ahead limiter for a pre-emphasised stereo programme, in M/S form.
///
/// The programme comes split into its flat part `x` and the boost the
/// pre-emphasis adds, `d` (the pre-emphasised programme is `x + d`), both
/// at the output rate, so that true (inter-sample) peaks are seen. It is
/// processed in frames of `F` samples (for the FM multiplex: the `F`
/// interpolator outputs per audio-rate sample); both gains are computed once
/// per frame and interpolated linearly across it.
///
/// Two gains, each from a [`LookaheadGain`]:
///
/// * The HF gain `h` scales only the boost: `x + h·d`. A high-frequency
///   overload thus reduces the pre-emphasis for a moment, which a receiver
///   (after de-emphasis) hears as less treble, not as the whole programme
///   ducking. Per sample and channel (`L = M + S`, `R = M - S`), `h` must
///   satisfy `|x + h·d| <= max(T, |x|)`: the boost may push no sample over
///   the threshold `T`, nor make a flat sample that is already over it any
///   larger. `x + h·d` is linear in `h` and within bounds at `h = 0`, so
///   every gain below the requirement satisfies it too, and the smoothed
///   gain (at most the requirement of the frame it is applied to) does.
/// * The peak gain `g` scales everything: `g·(x + h·d)`. With the HF gain
///   in place, every sample is within `max(T, |x|)`, so `g` only has to
///   bring the flat peak down to `T`: it acts only when the flat programme
///   itself exceeds the threshold.
///
/// For a programme without pre-emphasis, `d = 0` and this is a plain peak
/// limiter on `|M| + |S|` (the peak of `L` and `R`).
#[derive(Debug, Clone)]
pub struct PreemphasisLimiter {
    threshold: f32,
    frame: usize,
    hf: LookaheadGain,
    peak: LookaheadGain,
    /// Delay line of `L` frames of `[xm, xs, dm, ds]`; each frame's slot is
    /// read (the frame from `L` frames ago) before it is overwritten.
    delay: Vec<[f32; 4]>,
    slot: usize,
    /// Gains `(h, g)` of the previous frame.
    last: (f32, f32),
    min_hf: f32,
    min_peak: f32,
}

impl PreemphasisLimiter {
    /// `frame_rate`: frames per second; `lookahead` and the releases in seconds.
    pub fn new(
        frame: usize,
        frame_rate: f64,
        threshold: f32,
        lookahead: f64,
        hf_release: f64,
        peak_release: f64,
    ) -> PreemphasisLimiter {
        assert!(frame >= 1);
        let hf = LookaheadGain::new(frame_rate, lookahead, hf_release);
        PreemphasisLimiter {
            threshold,
            frame,
            delay: vec![[0.0; 4]; hf.look() * frame],
            peak: LookaheadGain::new(frame_rate, lookahead, peak_release),
            hf,
            slot: 0,
            last: (1.0, 1.0),
            min_hf: 1.0,
            min_peak: 1.0,
        }
    }

    /// Latency in samples.
    pub fn latency(&self) -> usize {
        self.hf.look() * self.frame
    }

    /// Lowest HF gain (fraction of the boost passed) and lowest peak gain
    /// applied since the last call (for metering).
    pub fn take_min_gains(&mut self) -> (f32, f32) {
        (std::mem::replace(&mut self.min_hf, 1.0), std::mem::replace(&mut self.min_peak, 1.0))
    }

    /// Largest HF gain `h` with `|x + h·d| <= max(T, |x|)` for one sample.
    #[inline(always)]
    fn required(&self, x: f32, d: f32) -> f32 {
        let lim = crate::fmax(self.threshold, x.abs());
        if (x + d).abs() <= lim {
            return 1.0;
        }
        // x + d lies beyond ±lim on the side d points to, so |d| > 0 and
        // the result is in [0, 1).
        let xs = if d >= 0.0 { x } else { -x };
        (lim - xs) / d.abs()
    }

    /// Push one frame (flat `xm`, `xs` and boost `dm`, `ds`, each `F`
    /// samples) and write the limited pre-emphasised frame from `L` frames
    /// earlier to `m_out` / `s_out`.
    #[inline]
    pub fn process(&mut self, xm: &[f32], xs: &[f32], dm: &[f32], ds: &[f32], m_out: &mut [f32], s_out: &mut [f32]) {
        let f = self.frame;
        let (xm, xs, dm, ds) = (&xm[..f], &xs[..f], &dm[..f], &ds[..f]);
        // Peaks of the boosted and the flat programme, max(|L|, |R|) =
        // |M| + |S|, compared on the bit patterns (ordered like the
        // magnitudes): float compares are slow on 32-bit ARM.
        let (mut boosted, mut flat) = (0u32, 0u32);
        for (((&xm, &xs), &dm), &ds) in xm.iter().zip(xs).zip(dm).zip(ds) {
            boosted = boosted.max(((xm + dm).abs() + (xs + ds).abs()).to_bits());
            flat = flat.max((xm.abs() + xs.abs()).to_bits());
        }
        let t = self.threshold;
        let mut req_hf = 1.0;
        if boosted > t.to_bits() {
            for (((&xm, &xs), &dm), &ds) in xm.iter().zip(xs).zip(dm).zip(ds) {
                let (l, r) = (self.required(xm + xs, dm + ds), self.required(xm - xs, dm - ds));
                req_hf = crate::fmin(req_hf, crate::fmin(l, r));
            }
        }
        let req_peak = if flat > t.to_bits() { t / f32::from_bits(flat) } else { 1.0 };

        let (h1, g1) = (self.hf.next(req_hf), self.peak.next(req_peak));
        let (h0, g0) = self.last;
        self.last = (h1, g1);
        self.min_hf = crate::fmin(self.min_hf, h0);
        self.min_peak = crate::fmin(self.min_peak, g0);

        let slots = self.delay.len() / f;
        let slot = &mut self.delay[self.slot * f..(self.slot + 1) * f];
        self.slot = if self.slot + 1 == slots { 0 } else { self.slot + 1 };
        let (hs, gs) = ((h1 - h0) / f as f32, (g1 - g0) / f as f32);
        let (mut h, mut g) = (h0, g0);
        let inputs = xm.iter().zip(xs).zip(dm).zip(ds);
        for (((d, (((&xm, &xs), &dm), &ds)), m), s) in
            slot.iter_mut().zip(inputs).zip(m_out.iter_mut()).zip(s_out.iter_mut())
        {
            let [om, os, odm, ods] = *d;
            *m = g * (om + h * odm);
            *s = g * (os + h * ods);
            *d = [xm, xs, dm, ds];
            h += hs;
            g += gs;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn noise(n: usize) -> f32 {
        let seed = (n as u32).wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (seed >> 8) as f32 / (1 << 24) as f32 - 0.5
    }

    /// One sample of (xm, xs, dm, ds).
    type Input = fn(usize) -> [f32; 4];
    /// An output sample (m, s) and the input sample it belongs to.
    type Output = ([f32; 2], [f32; 4]);

    fn run(frame: usize, look: f64, input: Input, n: usize) -> (Vec<Output>, PreemphasisLimiter) {
        let mut lim = PreemphasisLimiter::new(frame, 48_000.0, 1.0, look, 0.05, 0.1);
        let lat = lim.latency();
        let mut ch = vec![vec![0f32; frame]; 4];
        let (mut om, mut os) = (vec![0f32; frame], vec![0f32; frame]);
        let mut out = Vec::new();
        for k in 0..n / frame {
            for p in 0..frame {
                for (c, v) in ch.iter_mut().zip(input(k * frame + p)) {
                    c[p] = v;
                }
            }
            lim.process(&ch[0], &ch[1], &ch[2], &ch[3], &mut om, &mut os);
            for p in 0..frame {
                let i = k * frame + p;
                if i >= lat {
                    out.push(([om[p], os[p]], input(i - lat)));
                }
            }
        }
        (out, lim)
    }

    fn peak(out: &[Output]) -> f32 {
        out.iter().fold(0f32, |m, ([a, b], _)| m.max(a.abs() + b.abs()))
    }

    #[test]
    fn plain_peak_limiter_never_exceeds_threshold() {
        for frame in [1, 5] {
            let x: Input = |n| {
                let burst = if (n / 5000).is_multiple_of(3) { 4.0 } else { 0.3 };
                let v = burst * ((n as f32 * 0.05).sin() + noise(n));
                [0.15 * v, 0.85 * v, 0.0, 0.0]
            };
            let (out, _) = run(frame, 0.0015, x, 200_000);
            assert!(peak(&out) <= 1.0 + 1e-5, "frame {frame}: peak {}", peak(&out));
        }
    }

    #[test]
    fn transparent_below_threshold() {
        let x: Input = |n| [0.4 * (n as f32 * 0.1).sin(), 0.1, 0.3 * (n as f32 * 1.3).sin(), -0.1];
        let (out, mut lim) = run(5, 0.001, x, 5000);
        for ([m, s], v) in out {
            assert_eq!([m, s], [v[0] + v[2], v[1] + v[3]]);
        }
        assert_eq!(lim.take_min_gains(), (1.0, 1.0));
    }

    /// Flat part: a low tone within full scale; boost: overloading noise bursts.
    fn hf_overload(n: usize) -> [f32; 4] {
        let loud = (n / 3000).is_multiple_of(2);
        let x = 0.5 * (n as f32 * 0.01).sin();
        let k = if loud { 3.0 } else { 0.1 };
        [x, -0.3 * x, k * noise(n), k * 0.7 * noise(n + 77)]
    }

    #[test]
    fn hf_overload_reduces_only_the_boost() {
        for frame in [1, 5] {
            let (out, mut lim) = run(frame, 0.0015, hf_overload, 200_000);
            assert!(peak(&out) <= 1.0 + 1e-5, "frame {frame}: peak {}", peak(&out));
            // m = xm + h dm, s = xs + h ds with 0 <= h <= 1 (the flat part
            // is never scaled).
            for ([m, s], [xm, xs, dm, ds]) in &out {
                for (y, x, d) in [(m, xm, dm), (s, xs, ds)] {
                    if d.abs() > 1e-3 {
                        let h = (y - x) / d;
                        assert!((-1e-3..=1.0 + 1e-3).contains(&h), "h {h}");
                    }
                }
            }
            let (h, g) = lim.take_min_gains();
            assert!(h < 0.5, "the HF gain must act ({h})");
            assert_eq!(g, 1.0, "the peak gain must not act");
        }
    }

    #[test]
    fn gains_recover_to_unity() {
        // An f32 release follower can stall just short of 1: after an
        // overload both gains must come back to exactly 1.
        let mut lim = PreemphasisLimiter::new(5, 48_000.0, 1.0, 0.0015, 0.05, 0.1);
        let frame = |lim: &mut PreemphasisLimiter, k: usize, flat: f32, boost: f32| {
            let (mut om, mut os) = ([0f32; 5], [0f32; 5]);
            let x = [flat * (k as f32 * 0.05).sin(); 5];
            let d = [boost * (k as f32 * 0.7).sin(); 5];
            lim.process(&x, &[0.0; 5], &d, &[0.0; 5], &mut om, &mut os);
        };
        for k in 0..2_000 {
            frame(&mut lim, k, 2.0, 2.0);
        }
        let (h, g) = lim.take_min_gains();
        assert!(h < 1.0 && g < 1.0);
        for k in 0..100_000 {
            frame(&mut lim, k, 0.3, 0.1);
        }
        lim.take_min_gains();
        for k in 0..1_000 {
            frame(&mut lim, k, 0.3, 0.1);
        }
        assert_eq!(lim.take_min_gains(), (1.0, 1.0));
    }

    #[test]
    fn flat_overs_are_not_enlarged_by_the_boost() {
        // The flat part alone exceeds the threshold: the peak gain brings it
        // down, and the boost may not add to it.
        let x: Input = |n| {
            let v = 1.5 * (n as f32 * 0.002).sin();
            [v, 0.0, 0.4 * (n as f32 * 0.9).sin(), 0.0]
        };
        let (out, mut lim) = run(5, 0.0015, x, 100_000);
        assert!(peak(&out) <= 1.0 + 1e-5, "peak {}", peak(&out));
        let (_, g) = lim.take_min_gains();
        assert!(g < 0.7);
    }
}
