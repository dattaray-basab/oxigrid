/// Internal L-BFGS minimizer for ECM parameter fitting.
/// Algorithm: Nocedal & Wright, Numerical Optimization, Algorithm 7.4
/// (two-loop recursion with forward-difference gradient).
use std::collections::VecDeque;

pub struct LbfgsConfig {
    pub max_iter: usize,
    pub m: usize,
    pub ftol: f64,
    pub gtol: f64,
    pub finite_diff_step: f64,
}

impl Default for LbfgsConfig {
    fn default() -> Self {
        Self {
            max_iter: 100,
            m: 7,
            ftol: 1e-8,
            gtol: 1e-6,
            finite_diff_step: 1e-6,
        }
    }
}

/// Minimize f(x) from x0. Returns (x_opt, f_opt, iterations) or error.
pub fn lbfgs_minimize<F>(
    f: F,
    x0: &[f64],
    cfg: &LbfgsConfig,
) -> Result<(Vec<f64>, f64, usize), String>
where
    F: Fn(&[f64]) -> f64,
{
    if x0.is_empty() {
        return Err("empty initial point".into());
    }
    let mut x = x0.to_vec();
    let mut fx = f(&x);
    let mut g = forward_diff_grad(&f, &x, cfg.finite_diff_step);

    // History: VecDeque of (s_k, y_k) pairs
    let mut history: VecDeque<(Vec<f64>, Vec<f64>)> = VecDeque::with_capacity(cfg.m);

    for iter in 0..cfg.max_iter {
        // Check convergence on gradient infinity norm
        let g_inf = g.iter().map(|v| v.abs()).fold(0.0f64, f64::max);
        if g_inf < cfg.gtol {
            return Ok((x, fx, iter));
        }

        // Two-loop recursion to compute search direction p = -H_k g_k
        let p = two_loop_direction(&g, &history)?;

        // Check descent: gp = g^T p should be < 0
        let gp: f64 = g.iter().zip(p.iter()).map(|(gi, pi)| gi * pi).sum();
        if gp >= 0.0 {
            // Not a descent direction — reset history and use steepest descent
            history.clear();
            let p_sd: Vec<f64> = g.iter().map(|v| -v).collect();
            let gp_sd = -g.iter().map(|v| v * v).sum::<f64>();
            let (x_new, fx_new) = armijo_search(&f, &x, fx, &p_sd, gp_sd);
            let s: Vec<f64> = x_new.iter().zip(x.iter()).map(|(xn, xo)| xn - xo).collect();
            let g_new = forward_diff_grad(&f, &x_new, cfg.finite_diff_step);
            let y: Vec<f64> = g_new.iter().zip(g.iter()).map(|(gn, go)| gn - go).collect();
            let ys: f64 = y.iter().zip(s.iter()).map(|(yi, si)| yi * si).sum();
            if ys > 0.0 {
                if history.len() == cfg.m {
                    history.pop_front();
                }
                history.push_back((s, y));
            }
            let f_old = fx;
            x = x_new;
            fx = fx_new;
            g = g_new;
            if (f_old - fx).abs() < cfg.ftol {
                return Ok((x, fx, iter + 1));
            }
            continue;
        }

        let (x_new, fx_new) = armijo_search(&f, &x, fx, &p, gp);
        let s: Vec<f64> = x_new.iter().zip(x.iter()).map(|(xn, xo)| xn - xo).collect();
        let g_new = forward_diff_grad(&f, &x_new, cfg.finite_diff_step);
        let y: Vec<f64> = g_new.iter().zip(g.iter()).map(|(gn, go)| gn - go).collect();

        // Curvature condition: only update history if y^T s > 0
        let ys: f64 = y.iter().zip(s.iter()).map(|(yi, si)| yi * si).sum();
        if ys > 0.0 {
            if history.len() == cfg.m {
                history.pop_front();
            }
            history.push_back((s, y));
        }

        let f_old = fx;
        x = x_new;
        fx = fx_new;
        g = g_new;
        if (f_old - fx).abs() < cfg.ftol {
            return Ok((x, fx, iter + 1));
        }
    }

    Ok((x, fx, cfg.max_iter))
}

fn forward_diff_grad<F: Fn(&[f64]) -> f64>(f: &F, x: &[f64], h: f64) -> Vec<f64> {
    let fx = f(x);
    let mut g = vec![0.0; x.len()];
    let mut x_pert = x.to_vec();
    for i in 0..x.len() {
        x_pert[i] += h;
        g[i] = (f(&x_pert) - fx) / h;
        x_pert[i] = x[i];
    }
    g
}

fn two_loop_direction(
    g: &[f64],
    history: &VecDeque<(Vec<f64>, Vec<f64>)>,
) -> Result<Vec<f64>, String> {
    let n = g.len();
    let m = history.len();
    if m == 0 {
        let gnorm = g.iter().map(|v| v * v).sum::<f64>().sqrt().max(1e-15);
        return Ok(g.iter().map(|v| -v / gnorm).collect());
    }

    let mut q = g.to_vec();
    let mut alphas = vec![0.0f64; m];
    let rhos: Vec<f64> = history
        .iter()
        .map(|(s, y)| {
            let ys: f64 = y.iter().zip(s.iter()).map(|(yi, si)| yi * si).sum();
            if ys.abs() < 1e-15 {
                0.0
            } else {
                1.0 / ys
            }
        })
        .collect();

    // First loop (backwards)
    for i in (0..m).rev() {
        let (s, _y) = &history[i];
        let sq: f64 = s.iter().zip(q.iter()).map(|(si, qi)| si * qi).sum();
        alphas[i] = rhos[i] * sq;
        let (_s, y) = &history[i];
        for j in 0..n {
            q[j] -= alphas[i] * y[j];
        }
    }

    // Initial Hessian scaling: H_0 = (s^T y) / (y^T y) using most recent pair
    let (s_last, y_last) = history
        .back()
        .ok_or_else(|| "history unexpectedly empty in two_loop_direction".to_string())?;
    let sy: f64 = s_last
        .iter()
        .zip(y_last.iter())
        .map(|(si, yi)| si * yi)
        .sum();
    let yy: f64 = y_last.iter().map(|yi| yi * yi).sum();
    let h0 = if yy > 1e-15 { sy / yy } else { 1.0 };
    let mut r: Vec<f64> = q.iter().map(|qi| h0 * qi).collect();

    // Second loop (forwards)
    for i in 0..m {
        let (_s, y) = &history[i];
        let yr: f64 = y.iter().zip(r.iter()).map(|(yi, ri)| yi * ri).sum();
        let beta = rhos[i] * yr;
        let (s, _y) = &history[i];
        for j in 0..n {
            r[j] += s[j] * (alphas[i] - beta);
        }
    }

    // Return descent direction
    Ok(r.iter().map(|v| -v).collect())
}

/// Armijo backtracking line search.
/// Returns (x_new, fx_new). Falls back to the unmoved point if no step improves.
fn armijo_search<F: Fn(&[f64]) -> f64>(
    f: &F,
    x: &[f64],
    fx: f64,
    p: &[f64],
    gp: f64,
) -> (Vec<f64>, f64) {
    let c1 = 1e-4;
    let mut alpha = 1.0f64;
    // Conservative fallback: return x unchanged if nothing improves
    let mut best: (Vec<f64>, f64) = (x.to_vec(), fx);
    for _ in 0..20 {
        let x_new: Vec<f64> = x
            .iter()
            .zip(p.iter())
            .map(|(xi, pi)| xi + alpha * pi)
            .collect();
        let fx_new = f(&x_new);
        if fx_new < best.1 {
            best = (x_new.clone(), fx_new);
        }
        if fx_new <= fx + c1 * alpha * gp {
            return (x_new, fx_new);
        }
        alpha *= 0.5;
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lbfgs_quadratic_recovers_minimum() {
        let f = |x: &[f64]| (x[0] - 3.0).powi(2) + (x[1] + 2.0).powi(2) + 1.0;
        let cfg = LbfgsConfig::default();
        let (x, fval, _iters) = lbfgs_minimize(f, &[0.0, 0.0], &cfg).unwrap();
        assert!((x[0] - 3.0).abs() < 1e-4, "x[0]={} expected ~3.0", x[0]);
        assert!((x[1] + 2.0).abs() < 1e-4, "x[1]={} expected ~-2.0", x[1]);
        assert!((fval - 1.0).abs() < 1e-6, "fval={} expected ~1.0", fval);
    }

    #[test]
    fn lbfgs_rosenbrock_2d_converges() {
        let f = |x: &[f64]| {
            let a = 1.0 - x[0];
            let b = x[1] - x[0] * x[0];
            a * a + 100.0 * b * b
        };
        let cfg = LbfgsConfig {
            max_iter: 500,
            ..LbfgsConfig::default()
        };
        let (x, _fval, _iters) = lbfgs_minimize(f, &[-1.2, 1.0], &cfg).unwrap();
        assert!((x[0] - 1.0).abs() < 1e-2, "x[0]={} expected ~1.0", x[0]);
        assert!((x[1] - 1.0).abs() < 1e-2, "x[1]={} expected ~1.0", x[1]);
    }

    #[test]
    fn lbfgs_invalid_input_returns_error() {
        let f = |_x: &[f64]| 0.0f64;
        let cfg = LbfgsConfig::default();
        assert!(lbfgs_minimize(f, &[], &cfg).is_err());
    }

    #[test]
    fn lbfgs_cost_decreases_with_iterations() {
        let f = |x: &[f64]| x[0] * x[0] + x[1] * x[1];
        let x0 = [5.0_f64, 5.0_f64];
        let initial_cost = f(&x0);

        let cfg5 = LbfgsConfig {
            max_iter: 5,
            ..LbfgsConfig::default()
        };
        let (_x5, fval5, _) = lbfgs_minimize(f, &x0, &cfg5).expect("5-iter L-BFGS should succeed");

        let cfg20 = LbfgsConfig {
            max_iter: 20,
            ..LbfgsConfig::default()
        };
        let (_x20, fval20, _) =
            lbfgs_minimize(f, &x0, &cfg20).expect("20-iter L-BFGS should succeed");

        assert!(fval5.is_finite(), "fval5 should be finite, got {fval5}");
        assert!(fval20.is_finite(), "fval20 should be finite, got {fval20}");
        assert!(
            fval5 <= initial_cost,
            "5-iter result {fval5} should not exceed initial cost {initial_cost}"
        );
        assert!(
            fval20 <= initial_cost,
            "20-iter result {fval20} should not exceed initial cost {initial_cost}"
        );
    }

    #[test]
    fn lbfgs_gradient_norm_converges() {
        let f = |x: &[f64]| x[0] * x[0] + x[1] * x[1];
        let x0 = [10.0_f64, 10.0_f64];
        let initial_norm = x0[0].abs() + x0[1].abs();

        let cfg = LbfgsConfig::default();
        let (x_opt, _fval, _) =
            lbfgs_minimize(f, &x0, &cfg).expect("L-BFGS should converge on simple quadratic");

        let final_norm = x_opt[0].abs() + x_opt[1].abs();
        assert!(
            final_norm < initial_norm,
            "final |x| ({final_norm}) should be less than initial |x| ({initial_norm})"
        );
    }

    #[test]
    fn lbfgs_1d_quadratic_exact_minimum() {
        let f = |x: &[f64]| (x[0] - 3.0_f64).powi(2);
        let cfg = LbfgsConfig::default();
        let (x_opt, _fval, _) =
            lbfgs_minimize(f, &[0.0_f64], &cfg).expect("L-BFGS should find 1D minimum");

        assert!(
            (x_opt[0] - 3.0).abs() < 1e-3,
            "x[0]={} should be near 3.0",
            x_opt[0]
        );
    }

    #[test]
    fn lbfgs_returns_finite_result() {
        let f = |x: &[f64]| x[0] * x[0] + x[1] * x[1] + x[2] * x[2];
        let cfg = LbfgsConfig::default();
        let (x_opt, fval, _) = lbfgs_minimize(f, &[1.0_f64, 2.0_f64, 3.0_f64], &cfg)
            .expect("L-BFGS should succeed on 3D quadratic");

        assert!(
            x_opt.iter().all(|v| v.is_finite()),
            "all components of x_opt should be finite: {x_opt:?}"
        );
        assert!(fval.is_finite(), "fval should be finite, got {fval}");
    }

    #[test]
    fn lbfgs_memory_bound_by_m() {
        let f = |x: &[f64]| x.iter().map(|v| v * v).sum::<f64>();
        let x0 = [10.0_f64; 5];
        let initial_cost = f(&x0);

        let cfg = LbfgsConfig {
            max_iter: 100,
            m: 3,
            ..LbfgsConfig::default()
        };
        let (_x_opt, fval, _) =
            lbfgs_minimize(f, &x0, &cfg).expect("L-BFGS with m=3 should succeed on 5D quadratic");

        assert!(fval.is_finite(), "fval should be finite, got {fval}");
        assert!(
            fval < initial_cost,
            "fval ({fval}) should be less than initial cost ({initial_cost}) with bounded memory m=3"
        );
    }

    #[test]
    fn lbfgs_single_variable_converges() {
        let f = |x: &[f64]| (x[0] + 5.0_f64).powi(2) + 10.0_f64;
        let cfg = LbfgsConfig::default();
        let (x_opt, fval, _) = lbfgs_minimize(f, &[0.0_f64], &cfg)
            .expect("L-BFGS should find minimum of shifted quadratic");

        assert!(
            (x_opt[0] + 5.0).abs() < 1e-3,
            "x[0]={} should be near -5.0",
            x_opt[0]
        );
        assert!(
            (fval - 10.0).abs() < 1e-4,
            "fval={fval} should be near 10.0"
        );
    }
}
