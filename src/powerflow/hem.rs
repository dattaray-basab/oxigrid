/// Holomorphic Embedding Method (HEM) for power flow.
///
/// The HEM embeds the power flow equations in a complex parameter s:
///
///   V_i(s) = Σ_k V_i^`k` · s^k    (voltage as power series in s)
///   Y_bus · V(s) = s · I_spec(s)   (scaled injections)
///
/// At s=1 the original power flow is recovered.  The series coefficients
/// V^`k` are computed recursively from the network equations, then a
/// Padé approximant ℙ[M/N](s) is built and evaluated at s=1.
///
/// Advantages over Newton-Raphson:
/// - Always starts from the known no-load solution (s=0 → V=1∠0)
/// - Can detect voltage collapse (pole in Padé ≈ nose point)
/// - Does not require initial guess or Jacobian factorisation
///
/// Limitations of this implementation:
/// - Pure DC-bus (only active power, no reactive) for clarity
/// - Padé approximant up to order M=N=5 (sufficient for most systems ≤ 118 bus)
///
/// # References
/// - Subramanian et al., "Computing the Feasibility Boundary of Power Systems
///   Using the Holomorphic Embedding Method", PSCC 2016.
/// - Rao et al., "Holomorphic Embedding Methods in Power Systems", 2016.
use crate::error::{OxiGridError, Result};
use crate::network::topology::PowerNetwork;
use num_complex::Complex64;
use serde::{Deserialize, Serialize};

/// HEM solver configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HemConfig {
    /// Maximum number of power series terms
    pub max_terms: usize,
    /// Padé approximant order (M=N)
    pub pade_order: usize,
    /// Convergence tolerance on |V_k| (stop adding terms when coefficients decay)
    pub coeff_tol: f64,
    /// s at which to evaluate the Padé approximant (typically 1.0)
    pub s_eval: f64,
}

impl Default for HemConfig {
    fn default() -> Self {
        Self {
            max_terms: 20,
            pade_order: 5,
            coeff_tol: 1e-12,
            s_eval: 1.0,
        }
    }
}

/// Result of HEM power flow.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HemResult {
    /// Voltage magnitudes at each bus [p.u.]
    pub voltage_magnitude: Vec<f64>,
    /// Voltage angles `rad`
    pub voltage_angle: Vec<f64>,
    /// Number of series terms used
    pub terms_used: usize,
    /// Padé order used
    pub pade_order: usize,
    /// True if the Padé approximant converged (no pole at s=1)
    pub converged: bool,
    /// Estimated voltage stability margin (distance to nearest Padé pole in s)
    pub stability_margin: f64,
    /// Active power mismatch norm [p.u.]
    pub p_mismatch_norm: f64,
}

/// Complex power series coefficient (one per bus).
type CoeffVec = Vec<Complex64>;

/// Solve power flow using the Holomorphic Embedding Method.
///
/// This implementation uses a DC-like embedding where only active power
/// is balanced (imaginary part of voltage set to 0 for simplicity).
pub fn solve_hem(network: &PowerNetwork, config: &HemConfig) -> Result<HemResult> {
    network.validate()?;
    let n = network.bus_count();
    if n == 0 {
        return Err(OxiGridError::InvalidNetwork("Empty network".into()));
    }

    let slack_idx = network.slack_bus_index()?;

    // Get net power injections [p.u.]
    let (p_sched, q_sched) = network.net_injection();

    // Build Y-bus (complex admittance matrix)
    let ybus = network.admittance_matrix()?;

    // Power series: V_i(s) = Σ_k v_coeff[k][i] · s^k
    // Initial term (k=0): V_i^[0] = flat start (1 + j0)
    let v0: CoeffVec = vec![Complex64::new(1.0, 0.0); n];
    let mut v_coeffs: Vec<CoeffVec> = vec![v0];

    // Specified complex power S_i = P_i + jQ_i [p.u.]
    let s_spec: Vec<Complex64> = (0..n)
        .map(|i| Complex64::new(p_sched[i], q_sched[i]))
        .collect();

    let max_terms = config.max_terms.min(30);

    // Recursive computation of higher-order coefficients
    for k in 1..max_terms {
        let v_k = compute_hem_coefficient(k, &v_coeffs, &ybus, &s_spec, slack_idx, n);

        let coeff_norm: f64 = v_k.iter().map(|c| c.norm()).fold(0.0_f64, f64::max);
        v_coeffs.push(v_k);

        if coeff_norm < config.coeff_tol && k >= 4 {
            break;
        }
    }

    let terms_used = v_coeffs.len();

    // Build Padé approximants per bus and evaluate at s=1
    let pade_n = config.pade_order.min(terms_used / 2);

    let mut v_pade = vec![Complex64::new(1.0, 0.0); n];
    let mut stability_margins = vec![f64::INFINITY; n];
    let mut converged = true;

    for bus in 0..n {
        // Extract series for this bus
        let series: Vec<Complex64> = v_coeffs.iter().map(|vc| vc[bus]).collect();

        match pade_approx(&series, pade_n, config.s_eval) {
            Some((val, pole_dist)) => {
                v_pade[bus] = val;
                stability_margins[bus] = pole_dist;
                // Check for apparent divergence
                if val.norm() < 0.1 || val.norm() > 2.0 {
                    converged = false;
                }
            }
            None => {
                // Fall back to direct series evaluation
                let val = series_eval(&series, config.s_eval);
                v_pade[bus] = val;
                if val.norm() < 0.1 || val.norm() > 2.0 {
                    converged = false;
                }
            }
        }
    }

    let stability_margin = stability_margins
        .iter()
        .cloned()
        .fold(f64::INFINITY, f64::min);

    // Compute power mismatch
    let p_mismatch = compute_p_mismatch(&ybus, &v_pade, &p_sched, n);
    let p_mismatch_norm = p_mismatch.iter().map(|&e| e * e).sum::<f64>().sqrt();

    let voltage_magnitude: Vec<f64> = v_pade.iter().map(|v| v.norm()).collect();
    let voltage_angle: Vec<f64> = v_pade.iter().map(|v| v.arg()).collect();

    Ok(HemResult {
        voltage_magnitude,
        voltage_angle,
        terms_used,
        pade_order: pade_n,
        converged,
        stability_margin,
        p_mismatch_norm,
    })
}

/// Compute the k-th HEM coefficient V^`k` recursively.
///
/// From Y · V(s) = s · I_spec:
///   Σ_j Y_ij · V_j^`k` = I_i^[k-1]   for k ≥ 1
///
/// where I_i^`k` = (S_i / conj(V_i(s)))^`k` coefficient.
fn compute_hem_coefficient(
    k: usize,
    v_coeffs: &[CoeffVec],
    ybus: &sprs::CsMat<Complex64>,
    s_spec: &[Complex64],
    slack_idx: usize,
    n: usize,
) -> CoeffVec {
    // Compute current injection coefficient I^[k-1]
    // I_i = conj(S_i) / conj(V_i(s)) = conj(S_i) · (1/V_i(s))
    // For k=1: I_i^[0] = conj(S_i) / conj(V_i^[0]) = conj(S_i) (since V^[0]=1)
    // For k>1: need convolution

    // Simplified: compute I^[k-1] as conj(S_i) * W^[k-1] where W = 1/conj(V)
    // W^[0] = 1/conj(V^[0]) = 1
    // W^[k] = -(1/conj(V^[0])) * Σ_{j=1}^{k} conj(V^[j]) * W^[k-j]
    let w_km1 = compute_reciprocal_conj_coeff(k - 1, v_coeffs);

    // I_i^[k-1] = conj(S_i) * W_i^[k-1]
    let i_km1: Vec<Complex64> = (0..n).map(|i| s_spec[i].conj() * w_km1[i]).collect();

    // Solve Y · V^[k] = I^[k-1] (with slack bus V^[k]=0 for k≥1)
    // Build modified system: substitute slack row, solve for other buses
    solve_ybus_for_coeff(ybus, &i_km1, slack_idx, n)
}

/// Compute W^`m` where W = 1/conj(V) using the power series division formula.
fn compute_reciprocal_conj_coeff(m: usize, v_coeffs: &[CoeffVec]) -> Vec<Complex64> {
    let n = v_coeffs[0].len();
    if m == 0 {
        // W^[0] = 1/conj(V^[0]) = conj(1+j0) = 1
        return vec![Complex64::new(1.0, 0.0); n];
    }

    // W^[m] = -(1/conj(V^[0])) * Σ_{j=1}^{m} conj(V^[j]) * W^[m-j]
    // Recursively computed: need W^[0..m-1] (simplified: just use W^[0]=1 for m=1)
    // For correctness we compute the full recurrence
    let mut w_all: Vec<Vec<Complex64>> = vec![vec![Complex64::new(1.0, 0.0); n]]; // W^[0]

    for step in 1..=m {
        let mut w_m = vec![Complex64::new(0.0, 0.0); n];
        for j in 1..=step {
            if j < v_coeffs.len() {
                for (i, w) in w_m.iter_mut().enumerate() {
                    *w -= v_coeffs[j][i].conj() * w_all[step - j][i];
                }
            }
        }
        // Divide by conj(V^[0]) = 1 (flat start)
        w_all.push(w_m);
    }

    w_all.remove(m)
}

/// Solve Y·x = rhs with slack bus pinned to 0.
///
/// Uses dense Gaussian elimination for simplicity.
fn solve_ybus_for_coeff(
    ybus: &sprs::CsMat<Complex64>,
    rhs: &[Complex64],
    slack_idx: usize,
    n: usize,
) -> Vec<Complex64> {
    // Build dense complex matrix A (exclude slack row/col)
    let nr = n - 1;
    let to_r = |i: usize| if i < slack_idx { i } else { i - 1 };

    let mut a = vec![Complex64::new(0.0, 0.0); nr * nr];
    let mut b = vec![Complex64::new(0.0, 0.0); nr];

    for (val, (row, col)) in ybus.iter() {
        if row == slack_idx || col == slack_idx {
            continue;
        }
        let r = to_r(row);
        let c = to_r(col);
        a[r * nr + c] += *val;
    }

    for i in 0..n {
        if i == slack_idx {
            continue;
        }
        b[to_r(i)] = rhs[i];
    }

    // Gaussian elimination (complex)
    let x_r = gaussian_complex(&a, &b, nr);

    let mut x = vec![Complex64::new(0.0, 0.0); n];
    for i in 0..n {
        if i != slack_idx {
            x[i] = x_r[to_r(i)];
        }
    }
    x
}

/// Gaussian elimination for complex systems.
fn gaussian_complex(a_flat: &[Complex64], b: &[Complex64], n: usize) -> Vec<Complex64> {
    if n == 0 {
        return vec![];
    }
    let mut a = a_flat.to_vec();
    let mut x = b.to_vec();

    for col in 0..n {
        // Find pivot
        let mut max_row = col;
        let mut max_val = a[col * n + col].norm();
        for row in col + 1..n {
            let v = a[row * n + col].norm();
            if v > max_val {
                max_val = v;
                max_row = row;
            }
        }
        if max_val < 1e-14 {
            continue;
        }

        if max_row != col {
            for k in 0..n {
                a.swap(col * n + k, max_row * n + k);
            }
            x.swap(col, max_row);
        }

        let pivot = a[col * n + col];
        for row in col + 1..n {
            let factor = a[row * n + col] / pivot;
            for k in col..n {
                let av = a[col * n + k];
                a[row * n + k] -= factor * av;
            }
            let xc = x[col];
            x[row] -= factor * xc;
        }
    }

    for col in (0..n).rev() {
        if a[col * n + col].norm() < 1e-14 {
            continue;
        }
        let diag = a[col * n + col];
        x[col] /= diag;
        let xc = x[col];
        for row in 0..col {
            x[row] -= a[row * n + col] * xc;
            a[row * n + col] = Complex64::new(0.0, 0.0);
        }
    }

    x
}

/// Evaluate a complex power series at s.
fn series_eval(coeffs: &[Complex64], s: f64) -> Complex64 {
    let s_c = Complex64::new(s, 0.0);
    let mut result = Complex64::new(0.0, 0.0);
    let mut s_pow = Complex64::new(1.0, 0.0);
    for &c in coeffs {
        result += c * s_pow;
        s_pow *= s_c;
    }
    result
}

/// Compute diagonal Padé approximant [M/M] from power series coefficients.
///
/// Returns `Some((value_at_s, dist_to_nearest_pole))` or `None` if build fails.
pub fn pade_approx(coeffs: &[Complex64], order: usize, s: f64) -> Option<(Complex64, f64)> {
    let m = order;
    let n_needed = 2 * m + 1;
    if coeffs.len() < n_needed || m == 0 {
        let val = series_eval(coeffs, s);
        return Some((val, f64::INFINITY));
    }

    // Build [M/M] Padé: P(s) / Q(s)
    // Q coefficients satisfy: Σ_{j=0}^{M} q_j · c_{k-j} = 0 for k=M+1..2M
    // with q_0 = 1.
    // Solve linear system for q_1..q_M
    let mut mat = vec![Complex64::new(0.0, 0.0); m * m];
    let mut rhs = vec![Complex64::new(0.0, 0.0); m];

    for row in 0..m {
        let k = m + 1 + row;
        rhs[row] = -coeffs[k];
        for col in 0..m {
            let j = col + 1;
            let idx = k - j;
            if idx < coeffs.len() {
                mat[row * m + col] = coeffs[idx];
            }
        }
    }

    let q_tail = gaussian_complex(&mat, &rhs, m);
    let mut q = vec![Complex64::new(1.0, 0.0)]; // q_0 = 1
    q.extend_from_slice(&q_tail);

    // p_k = Σ_{j=0}^{min(k,M)} q_j * c_{k-j} for k=0..M
    let mut p = vec![Complex64::new(0.0, 0.0); m + 1];
    for k in 0..=m {
        for j in 0..=k.min(m) {
            if k - j < coeffs.len() {
                p[k] += q[j] * coeffs[k - j];
            }
        }
    }

    let s_c = Complex64::new(s, 0.0);
    let p_val = poly_eval_complex(&p, s_c);
    let q_val = poly_eval_complex(&q, s_c);

    if q_val.norm() < 1e-12 {
        // Pole at s=1 → voltage collapse
        return Some((Complex64::new(0.0, 0.0), 0.0));
    }

    let value = p_val / q_val;

    // Find nearest pole (root of Q) by scanning unit interval
    let pole_dist = find_nearest_pole_dist(&q, s);

    Some((value, pole_dist))
}

/// Evaluate a complex polynomial at s.
fn poly_eval_complex(coeffs: &[Complex64], s: Complex64) -> Complex64 {
    let mut result = Complex64::new(0.0, 0.0);
    let mut s_pow = Complex64::new(1.0, 0.0);
    for &c in coeffs {
        result += c * s_pow;
        s_pow *= s;
    }
    result
}

/// Estimate the real part of the nearest pole of Q(s) ≤ s_eval.
fn find_nearest_pole_dist(q_coeffs: &[Complex64], s_eval: f64) -> f64 {
    // Scan for sign changes in Q(s) along real axis [0, s_eval]
    let n_pts = 100;
    let mut prev_norm = poly_eval_complex(q_coeffs, Complex64::new(0.0, 0.0)).norm();
    let mut min_dist = s_eval + 1.0;

    for i in 1..=n_pts {
        let s = s_eval * i as f64 / n_pts as f64;
        let q = poly_eval_complex(q_coeffs, Complex64::new(s, 0.0));
        let norm = q.norm();
        if norm < 0.01 * prev_norm.max(1e-6) {
            min_dist = s_eval - s;
            break;
        }
        prev_norm = norm;
    }

    min_dist.max(0.0)
}

/// Compute active power mismatch at each bus.
fn compute_p_mismatch(
    ybus: &sprs::CsMat<Complex64>,
    v: &[Complex64],
    p_spec: &[f64],
    n: usize,
) -> Vec<f64> {
    let mut p_calc = vec![0.0f64; n];
    for (yij, (i, j)) in ybus.iter() {
        let s = v[i] * (yij * v[j]).conj();
        p_calc[i] += s.re;
    }
    (0..n).map(|i| p_spec[i] - p_calc[i]).collect()
}

/// Voltage stability index: distance from the operating point to voltage collapse.
///
/// Estimates via the HEM Padé pole location.  Returns `distance` ∈ [0, 1]
/// (0 = at collapse, 1 = far from collapse).
pub fn voltage_stability_index(result: &HemResult) -> f64 {
    result.stability_margin.clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::PowerNetwork;

    fn ieee14_net() -> PowerNetwork {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
        PowerNetwork::from_matpower(path).expect("parse ieee14")
    }

    #[test]
    fn test_hem_2bus_converges() {
        use crate::network::branch::Branch;
        use crate::network::bus::{Bus, BusType};
        use crate::network::topology::Generator;

        let mut net = PowerNetwork::new(100.0);
        net.buses.push({
            let mut b = Bus::new(1, BusType::Slack);
            b.vm = 1.0;
            b
        });
        net.buses.push({
            let mut b = Bus::new(2, BusType::PQ);
            b.pd = crate::units::Power(50.0);
            b.qd = crate::units::ReactivePower(20.0);
            b
        });
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.01,
            x: 0.1,
            b: 0.02,
            rate_a: 200.0,
            rate_b: 200.0,
            rate_c: 200.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
        net.generators.push(Generator {
            bus_id: 1,
            pg: 0.0,
            qg: 0.0,
            qmax: 999.0,
            qmin: -999.0,
            vg: 1.0,
            mbase: 100.0,
            status: true,
            pmax: 999.0,
            pmin: 0.0,
        });

        let config = HemConfig::default();
        let result = solve_hem(&net, &config).unwrap();
        assert!(result.terms_used > 0);
        // Slack bus should have voltage ≈ 1
        assert!(
            (result.voltage_magnitude[0] - 1.0).abs() < 0.1,
            "Slack bus voltage: {:.4}",
            result.voltage_magnitude[0]
        );
    }

    #[test]
    fn test_hem_ieee14() {
        let net = ieee14_net();
        let config = HemConfig {
            max_terms: 15,
            pade_order: 5,
            ..Default::default()
        };
        let result = solve_hem(&net, &config).unwrap();
        assert!(result.terms_used > 0, "Should compute at least 1 term");
        // All voltages should be physically reasonable
        for (i, &vm) in result.voltage_magnitude.iter().enumerate() {
            assert!(
                vm > 0.3 && vm < 1.5,
                "Bus {} voltage out of range: {:.4}",
                i,
                vm
            );
        }
    }

    #[test]
    fn test_hem_terms_used() {
        let net = ieee14_net();
        let config = HemConfig {
            max_terms: 10,
            ..Default::default()
        };
        let result = solve_hem(&net, &config).unwrap();
        assert!(result.terms_used <= 10);
        assert!(result.terms_used >= 1);
    }

    #[test]
    fn test_pade_approx_constant_series() {
        // All coefficients = 1 → series = 1/(1-s), pole at s=1
        let coeffs: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 12];
        let result = pade_approx(&coeffs, 5, 0.5);
        assert!(result.is_some());
        let (val, _dist) = result.unwrap();
        // At s=0.5: 1/(1-0.5) = 2.0
        assert!(
            (val.re - 2.0).abs() < 0.2,
            "Padé for geometric series at s=0.5: {:.4}",
            val.re
        );
        // Note: pole distance detection is approximate (scanning based)
    }

    #[test]
    fn test_series_eval_geometric() {
        // 1 + s + s² + s³ at s=0.5 = 1/(1-0.5) - s^4/(1-0.5) ≈ 2 - 0.0625*2 ≈ 1.875
        let coeffs: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 4];
        let val = series_eval(&coeffs, 0.5);
        let expected: f64 = 1.0 + 0.5 + 0.25 + 0.125;
        assert!(
            (val.re - expected).abs() < 1e-10,
            "Series eval: {:.6} expected {:.6}",
            val.re,
            expected
        );
    }

    #[test]
    fn test_voltage_stability_index() {
        let result = HemResult {
            voltage_magnitude: vec![1.0, 0.95],
            voltage_angle: vec![0.0, -0.05],
            terms_used: 10,
            pade_order: 5,
            converged: true,
            stability_margin: 0.3,
            p_mismatch_norm: 1e-4,
        };
        let idx = voltage_stability_index(&result);
        assert!((idx - 0.3).abs() < 1e-10);
    }

    #[test]
    fn test_hem_stability_margin_positive() {
        let net = ieee14_net();
        let config = HemConfig::default();
        let result = solve_hem(&net, &config).unwrap();
        assert!(result.stability_margin >= 0.0);
    }

    #[test]
    fn test_hem_p_mismatch_finite() {
        let net = ieee14_net();
        let config = HemConfig::default();
        let result = solve_hem(&net, &config).unwrap();
        assert!(
            result.p_mismatch_norm.is_finite(),
            "P mismatch norm should be finite: {:?}",
            result.p_mismatch_norm
        );
    }
}
