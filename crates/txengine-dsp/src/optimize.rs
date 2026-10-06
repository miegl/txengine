//! Tiny derivative-free minimiser used for filter fitting at start-up.

/// Nelder–Mead simplex minimisation of `f` starting at `x0` with initial
/// step `step`. Returns the best point and its value.
pub fn nelder_mead(f: impl Fn(&[f64]) -> f64, x0: &[f64], step: f64, iters: usize) -> (Vec<f64>, f64) {
    let n = x0.len();
    let mut simplex: Vec<Vec<f64>> = vec![x0.to_vec()];
    for i in 0..n {
        let mut p = x0.to_vec();
        p[i] += step;
        simplex.push(p);
    }
    let mut vals: Vec<f64> = simplex.iter().map(|p| f(p)).collect();
    for _ in 0..iters {
        // Order.
        let mut idx: Vec<usize> = (0..=n).collect();
        idx.sort_by(|&a, &b| vals[a].total_cmp(&vals[b]));
        simplex = idx.iter().map(|&i| simplex[i].clone()).collect();
        vals = idx.iter().map(|&i| vals[i]).collect();
        if (vals[n] - vals[0]).abs() < 1e-14 {
            break;
        }
        let centroid: Vec<f64> = (0..n).map(|j| simplex[..n].iter().map(|p| p[j]).sum::<f64>() / n as f64).collect();
        let along = |t: f64| -> Vec<f64> { (0..n).map(|j| centroid[j] + t * (simplex[n][j] - centroid[j])).collect() };
        let xr = along(-1.0);
        let fr = f(&xr);
        if fr < vals[0] {
            let xe = along(-2.0);
            let fe = f(&xe);
            if fe < fr {
                simplex[n] = xe;
                vals[n] = fe;
            } else {
                simplex[n] = xr;
                vals[n] = fr;
            }
        } else if fr < vals[n - 1] {
            simplex[n] = xr;
            vals[n] = fr;
        } else {
            let xc = if fr < vals[n] { along(-0.5) } else { along(0.5) };
            let fc = f(&xc);
            if fc < vals[n].min(fr) {
                simplex[n] = xc;
                vals[n] = fc;
            } else {
                // Shrink towards the best point.
                let (best, rest) = simplex.split_at_mut(1);
                for (p, v) in rest.iter_mut().zip(vals[1..].iter_mut()) {
                    for (x, b) in p.iter_mut().zip(&best[0]) {
                        *x = b + 0.5 * (*x - b);
                    }
                    *v = f(p);
                }
            }
        }
    }
    let best = (0..=n).min_by(|&a, &b| vals[a].total_cmp(&vals[b])).unwrap();
    (simplex[best].clone(), vals[best])
}

#[cfg(test)]
mod tests {
    #[test]
    fn finds_quadratic_minimum() {
        let (x, v) = super::nelder_mead(|p| (p[0] - 3.0).powi(2) + (p[1] + 1.0).powi(2) + 2.0, &[0.0, 0.0], 1.0, 500);
        assert!((x[0] - 3.0).abs() < 1e-5 && (x[1] + 1.0).abs() < 1e-5 && (v - 2.0).abs() < 1e-9);
    }
}
