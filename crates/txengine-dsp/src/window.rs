//! Window functions.

/// Zeroth order modified Bessel function of the first kind.
pub fn bessel_i0(x: f64) -> f64 {
    let mut sum = 1.0;
    let mut term = 1.0;
    let q = x * x / 4.0;
    for k in 1..200 {
        term *= q / (k as f64 * k as f64);
        sum += term;
        if term < sum * 1e-17 {
            break;
        }
    }
    sum
}

/// Kaiser window of length `n` with shape parameter `beta`.
pub fn kaiser(n: usize, beta: f64) -> Vec<f64> {
    if n == 1 {
        return vec![1.0];
    }
    let denom = bessel_i0(beta);
    let m = (n - 1) as f64;
    (0..n)
        .map(|i| {
            let r = 2.0 * i as f64 / m - 1.0;
            bessel_i0(beta * (1.0 - r * r).max(0.0).sqrt()) / denom
        })
        .collect()
}

/// Kaiser `beta` for a desired stop-band attenuation in dB.
pub fn kaiser_beta(atten_db: f64) -> f64 {
    if atten_db > 50.0 {
        0.1102 * (atten_db - 8.7)
    } else if atten_db >= 21.0 {
        0.5842 * (atten_db - 21.0).powf(0.4) + 0.07886 * (atten_db - 21.0)
    } else {
        0.0
    }
}

/// Number of taps for a Kaiser-windowed FIR with the given attenuation and
/// transition width (normalised to the sample rate, i.e. Hz / fs).
pub fn kaiser_length(atten_db: f64, transition: f64) -> usize {
    let n = (atten_db - 7.95) / (2.285 * 2.0 * std::f64::consts::PI * transition);
    (n.ceil() as usize + 1) | 1
}
