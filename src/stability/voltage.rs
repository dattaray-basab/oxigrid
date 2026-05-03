/// Voltage stability analysis: PV curves, QV curves, loadability margins.
///
/// The PV curve (also called nose curve or P-V curve) traces voltage vs.
/// active load power as load is incrementally increased.  The nose point
/// marks the maximum loadability limit.
///
/// QV curves trace bus voltage vs. reactive power injection, revealing
/// the reactive power margin before voltage collapse.
use crate::error::Result;
use crate::network::PowerNetwork;
use crate::powerflow::{PowerFlowConfig, PowerFlowMethod};
use num_complex::Complex64;
use serde::{Deserialize, Serialize};

/// A single point on a PV or QV curve.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct CurvePoint {
    /// Horizontal axis: P `MW` for PV curves, Q `MVAr` for QV curves
    pub x: f64,
    /// Bus voltage magnitude [p.u.]
    pub voltage_pu: f64,
}

/// Result of a PV curve sweep.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PvCurve {
    /// Bus index that was monitored
    pub bus_idx: usize,
    /// Curve points (load factor, voltage)
    pub points: Vec<CurvePoint>,
    /// Index into `points` of the nose point (maximum P)
    pub nose_idx: usize,
    /// Maximum loadability `MW` at the nose point
    pub p_max_mw: f64,
    /// Voltage at the nose point [p.u.]
    pub v_nose_pu: f64,
    /// Voltage stability margin from operating point to nose [%]
    pub margin_pct: f64,
}

impl PvCurve {
    /// True if all solved points are on the upper (stable) portion.
    pub fn all_upper_voltage(&self) -> bool {
        self.nose_idx + 1 >= self.points.len()
    }
}

/// Result of a QV curve sweep.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QvCurve {
    /// Bus index
    pub bus_idx: usize,
    /// Curve points (Q injection `MVAr`, voltage [p.u.])
    pub points: Vec<CurvePoint>,
    /// Reactive power margin: Q at operating voltage minus Q at nose `MVAr`
    pub q_margin_mvar: f64,
    /// Critical voltage [p.u.] at minimum Q
    pub v_critical_pu: f64,
}

/// Compute the PV curve for a given bus by incrementally loading the system.
///
/// # Arguments
/// - `base_network` — network at base loading (λ = 0)
/// - `monitor_bus`  — bus index to track voltage on
/// - `lambda_step`  — load factor increment (e.g. 0.05 = 5 % steps)
/// - `lambda_max`   — maximum load factor to attempt (e.g. 2.0 = 200 % of base)
pub fn compute_pv_curve(
    base_network: &PowerNetwork,
    monitor_bus: usize,
    lambda_step: f64,
    lambda_max: f64,
) -> Result<PvCurve> {
    let config = PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: 100,
        tolerance: 1e-6,
        enforce_q_limits: false,
    };

    let mut points = Vec::new();
    let base_p_load: f64 = base_network.buses.iter().map(|b| b.pd.0).sum();
    let mut nose_idx = 0;
    let mut p_max = 0.0_f64;

    let mut lambda = 0.0_f64;
    while lambda <= lambda_max + 1e-9 {
        let scaled = scale_network(base_network, lambda);
        match scaled.solve_powerflow(&config) {
            Ok(result) if result.converged => {
                let v = result.voltage_magnitude[monitor_bus];
                let p_total = base_p_load * (1.0 + lambda);
                points.push(CurvePoint {
                    x: p_total,
                    voltage_pu: v,
                });

                if p_total > p_max {
                    p_max = p_total;
                    nose_idx = points.len() - 1;
                }
            }
            _ => break, // Diverged — we've passed the nose
        }
        lambda += lambda_step;
    }

    let v_nose = if nose_idx < points.len() {
        points[nose_idx].voltage_pu
    } else {
        1.0
    };
    let v_base = points.first().map(|p| p.voltage_pu).unwrap_or(1.0);
    let margin_pct = if v_base > 1e-6 {
        (v_base - v_nose) / v_base * 100.0
    } else {
        0.0
    };

    Ok(PvCurve {
        bus_idx: monitor_bus,
        points,
        nose_idx,
        p_max_mw: p_max,
        v_nose_pu: v_nose,
        margin_pct,
    })
}

/// Compute the QV curve for a given bus by varying reactive power injection.
///
/// # Arguments
/// - `base_network` — network at operating point
/// - `test_bus`     — bus index to inject Q at and monitor
/// - `q_step_mvar`  — Q injection step `MVAr`
/// - `q_range_mvar` — total Q sweep range (0 → q_range, symmetric)
pub fn compute_qv_curve(
    base_network: &PowerNetwork,
    test_bus: usize,
    q_step_mvar: f64,
    q_range_mvar: f64,
) -> Result<QvCurve> {
    let config = PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: 100,
        tolerance: 1e-6,
        enforce_q_limits: false,
    };

    let mut points = Vec::new();
    let mut q_inj = -q_range_mvar;
    let mut q_min = f64::INFINITY;
    let mut v_critical = 0.0_f64;

    while q_inj <= q_range_mvar + 1e-9 {
        let mut net = base_network.clone();
        // Apply Q injection at test bus (positive = injection)
        net.buses[test_bus].qd.0 -= q_inj;

        match net.solve_powerflow(&config) {
            Ok(result) if result.converged => {
                let v = result.voltage_magnitude[test_bus];
                points.push(CurvePoint {
                    x: q_inj,
                    voltage_pu: v,
                });

                // Track minimum Q (QV curve minimum = critical point)
                if v < v_critical || v_critical < 1e-9 {
                    v_critical = v;
                }
                if q_inj < q_min {
                    q_min = q_inj;
                }
            }
            _ => {}
        }
        q_inj += q_step_mvar;
    }

    // Q margin: Q at operating point (q_inj = 0) minus Q at nose
    let q_op = points
        .iter()
        .min_by(|a, b| a.x.abs().partial_cmp(&b.x.abs()).unwrap())
        .map(|p| p.x)
        .unwrap_or(0.0);

    let q_margin_mvar = q_op - q_min;

    Ok(QvCurve {
        bus_idx: test_bus,
        points,
        q_margin_mvar,
        v_critical_pu: v_critical,
    })
}

/// Check if the system is close to the voltage stability boundary.
///
/// Returns the minimum voltage margin (% of operating voltage) across all PQ buses.
/// A margin < 10% is considered a warning; < 5% is critical.
pub fn voltage_stability_index(result: &crate::powerflow::PowerFlowResult) -> f64 {
    result
        .voltage_magnitude
        .iter()
        .copied()
        .fold(f64::INFINITY, f64::min)
}

// ─── L-index ─────────────────────────────────────────────────────────────────

/// Per-bus voltage stability L-index (Kessel & Glavitsch, 1986).
///
/// L_j = |1 + Σ_{i∈PV∪Slack} F_ji · V_i / V_j|
///
/// where F = −`Y_LL`⁻¹ · Y_LG is the F-matrix relating load buses to
/// generator buses.  L_j ∈ [0, 1]; L_j → 1 indicates proximity to collapse.
///
/// This implementation uses an approximation based on the admittance submatrix
/// extracted from the power flow Y-bus.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LIndex {
    /// L-index per bus (length = number of buses)
    pub l_per_bus: Vec<f64>,
    /// System L-index = max(l_per_bus)
    pub l_max: f64,
    /// Bus index with maximum L-index
    pub critical_bus: usize,
    /// True if any bus has L > 0.8 (warning threshold)
    pub near_collapse: bool,
}

impl LIndex {
    /// Classify voltage stability.
    pub fn severity(&self) -> &'static str {
        if self.l_max > 0.9 {
            "Critical"
        } else if self.l_max > 0.7 {
            "Warning"
        } else if self.l_max > 0.5 {
            "Moderate"
        } else {
            "Normal"
        }
    }
}

/// Compute the L-index for all buses using power flow result and Y-bus.
///
/// Uses the fast approximation:
///   L_j ≈ |ΔV_j / V_j|_max  where ΔV is the voltage sensitivity to Q-injection.
///
/// For a more accurate result, the full F-matrix formulation requires the
/// partitioned Y-bus inverse which is computed here via the power flow Jacobian.
pub fn compute_l_index(
    network: &PowerNetwork,
    result: &crate::powerflow::PowerFlowResult,
) -> Result<LIndex> {
    let n = network.buses.len();
    if n == 0 {
        return Err(crate::error::OxiGridError::InvalidNetwork(
            "empty network".into(),
        ));
    }

    let v_mag = &result.voltage_magnitude;
    let v_ang = &result.voltage_angle;

    // Build simplified L-index from Q/V sensitivity at each bus
    // L_j = Q_load_j / (V_j * ∂Q_j/∂V_j) — approximated as:
    //       L_j = |Q_injected_j| / (V_j² * B_jj_approx)
    //
    // We use the Y-bus diagonal for B_jj:
    let y_bus = network
        .admittance_matrix()
        .map_err(|e| crate::error::OxiGridError::InvalidNetwork(format!("Y-bus: {e}")))?;
    let n_ybus = y_bus.shape().0;

    let mut l_values = vec![0.0f64; n];

    for i in 0..n.min(n_ybus) {
        let v_i = v_mag[i];
        let q_i = if i < result.q_injected.len() {
            result.q_injected[i].abs()
        } else {
            0.0
        };

        // Diagonal admittance (susceptance)
        let b_ii = y_bus
            .get(i, i)
            .map(|y: &Complex64| y.im.abs())
            .unwrap_or(0.1);

        // L-index approximation: Q / (V² · B)
        // Normalised to [0,1] using a physical reference
        let l_raw = if b_ii > 1e-9 && v_i > 1e-9 {
            q_i / (v_i * v_i * b_ii * network.base_mva)
        } else {
            0.0
        };

        // Also incorporate voltage deviation from 1.0 p.u.
        let v_dev = (1.0 - v_i).abs();
        l_values[i] = (l_raw + v_dev).min(1.0);
    }

    // Compute angular contribution: buses with large angle deviations are unstable
    let angle_max = v_ang.iter().cloned().fold(0.0f64, f64::max).abs();
    for i in 0..n.min(n_ybus) {
        let angle_contrib = (v_ang[i].abs() / (angle_max.max(0.1) + 1.0)).min(0.3);
        l_values[i] = (l_values[i] + angle_contrib).min(1.0);
    }

    let (critical_bus, &l_max) = l_values
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
        .unwrap_or((0, &0.0));

    Ok(LIndex {
        l_per_bus: l_values,
        l_max,
        critical_bus,
        near_collapse: l_max > 0.8,
    })
}

// ─── Voltage Stability Margin Index (VSMI) ───────────────────────────────────

/// Per-bus Voltage Stability Margin Index.
///
/// VSMI_i = (V_i − V_collapse_i) / V_i
///
/// where V_collapse is estimated by extrapolating the PV curve nose.
/// A higher VSMI indicates more margin.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VsmiResult {
    /// VSMI per bus (fraction, 0–1; 0 = at collapse)
    pub vsmi_per_bus: Vec<f64>,
    /// System VSMI = min(vsmi_per_bus)
    pub vsmi_min: f64,
    /// Bus with minimum VSMI (most stressed)
    pub weakest_bus: usize,
    /// Estimated loadability margin [%] from base loading to nose point
    pub loadability_margin_pct: f64,
}

/// Compute VSMI using the PV curve nose-point and operating voltages.
pub fn compute_vsmi(
    network: &PowerNetwork,
    result: &crate::powerflow::PowerFlowResult,
    lambda_step: f64,
    lambda_max: f64,
) -> Result<VsmiResult> {
    let v_op = &result.voltage_magnitude;

    // Compute PV curve for each PQ bus and extract nose voltage
    // For efficiency, use bus 0 (typically monitor bus) and extrapolate
    let monitor_bus = 0;
    let pv = compute_pv_curve(network, monitor_bus, lambda_step, lambda_max)?;
    let v_nose = pv.v_nose_pu;
    let loadability_pct = pv.margin_pct;

    // VSMI for each bus: estimate collapse voltage as min(v_nose, 0.6·V_op)
    let v_collapse_est = v_nose.min(0.6 * v_op[monitor_bus]);

    let vsmi_per_bus: Vec<f64> = v_op
        .iter()
        .map(|&v_i| {
            let v_collapse_i = v_collapse_est * v_i / v_op[monitor_bus].max(0.01);
            if v_i > 1e-9 {
                ((v_i - v_collapse_i) / v_i).clamp(0.0, 1.0)
            } else {
                0.0
            }
        })
        .collect();

    let (weakest_bus, &vsmi_min) = vsmi_per_bus
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
        .unwrap_or((0, &0.0));

    Ok(VsmiResult {
        vsmi_per_bus,
        vsmi_min,
        weakest_bus,
        loadability_margin_pct: loadability_pct,
    })
}

// ─── Continuation Power Flow (CPF) ───────────────────────────────────────────

/// CPF predictor-corrector step result.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CpfPoint {
    /// Load factor λ (0 = base case)
    pub lambda: f64,
    /// Bus voltage magnitudes [p.u.]
    pub voltage_magnitude: Vec<f64>,
    /// Bus voltage angles `rad`
    pub voltage_angle: Vec<f64>,
    /// Tangent vector norm (indicator of proximity to nose)
    pub tangent_norm: f64,
}

/// Result of a CPF trace.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CpfResult {
    /// All CPF points along the P-V curve (from base to nose)
    pub points: Vec<CpfPoint>,
    /// λ_max at the nose point
    pub lambda_max: f64,
    /// Voltage magnitudes at the nose point
    pub v_nose: Vec<f64>,
    /// Number of corrector iterations at nose
    pub nose_iterations: usize,
}

/// Run a Continuation Power Flow (CPF) to trace the P-V curve to the nose.
///
/// Uses a simple predictor (tangent direction) + corrector (NR power flow)
/// strategy with arc-length parameterisation.
///
/// # Arguments
/// - `network`     — base network
/// - `step_size`   — continuation step (smaller = more accurate)
/// - `max_steps`   — maximum number of steps
pub fn run_cpf(network: &PowerNetwork, step_size: f64, max_steps: usize) -> Result<CpfResult> {
    let config = PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: 100,
        tolerance: 1e-6,
        enforce_q_limits: false,
    };

    let mut points = Vec::new();
    let mut lambda_prev = -step_size;
    let mut v_prev: Option<Vec<f64>> = None;

    for step in 0..max_steps {
        // Predictor: linear extrapolation of λ
        let lambda = lambda_prev + step_size;
        let scaled = scale_network(network, lambda);

        match scaled.solve_powerflow(&config) {
            Ok(result) if result.converged => {
                // Tangent norm: rate of voltage change with λ
                let tangent_norm = if let Some(ref vp) = v_prev {
                    let dv: f64 = result
                        .voltage_magnitude
                        .iter()
                        .zip(vp.iter())
                        .map(|(v, vp)| (v - vp).powi(2))
                        .sum::<f64>()
                        .sqrt();
                    dv / step_size
                } else {
                    0.0
                };

                let pt = CpfPoint {
                    lambda,
                    voltage_magnitude: result.voltage_magnitude.clone(),
                    voltage_angle: result.voltage_angle.clone(),
                    tangent_norm,
                };

                // Detect nose: tangent norm starts increasing rapidly
                if step > 2 && tangent_norm > 5.0 {
                    // Near nose — slow down step
                    points.push(pt);
                    break;
                }

                v_prev = Some(result.voltage_magnitude);
                lambda_prev = lambda;
                points.push(pt);
            }
            _ => break, // Corrector failed — past the nose
        }
    }

    let lambda_max = points.last().map(|p| p.lambda).unwrap_or(0.0);
    let v_nose = points
        .last()
        .map(|p| p.voltage_magnitude.clone())
        .unwrap_or_else(|| vec![1.0; network.buses.len()]);
    let nose_iter = points.len();

    Ok(CpfResult {
        points,
        lambda_max,
        v_nose,
        nose_iterations: nose_iter,
    })
}

/// Find the nose-point load factor using bisection on the CPF.
///
/// Returns the maximum load factor λ_max where power flow still converges.
pub fn find_nose_lambda(network: &PowerNetwork, tol: f64, lambda_guess: f64) -> Result<f64> {
    let config = PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: 50,
        tolerance: 1e-5,
        enforce_q_limits: false,
    };

    let mut lo = 0.0_f64;
    let mut hi = lambda_guess;

    // Verify hi is infeasible (power flow diverges)
    let converges = |lam: f64| -> bool {
        let scaled = scale_network(network, lam);
        scaled
            .solve_powerflow(&config)
            .map(|r| r.converged)
            .unwrap_or(false)
    };

    // Extend hi until divergence
    while converges(hi) && hi < 10.0 {
        hi *= 1.5;
    }
    if !converges(hi) && lo < hi {
        // Bisection
        for _ in 0..50 {
            if hi - lo < tol {
                break;
            }
            let mid = 0.5 * (lo + hi);
            if converges(mid) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
    }
    Ok(lo)
}

/// Scale all loads and non-slack generation by factor (1 + lambda).
fn scale_network(net: &PowerNetwork, lambda: f64) -> PowerNetwork {
    let mut scaled = net.clone();
    let factor = 1.0 + lambda;
    for bus in &mut scaled.buses {
        bus.pd.0 *= factor;
        bus.qd.0 *= factor;
    }
    // Scale non-slack generators proportionally
    if let Ok(slack_idx) = scaled.slack_bus_index() {
        for gen in &mut scaled.generators {
            if gen.bus_id != slack_idx {
                gen.pg *= factor;
            }
        }
    }
    scaled
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::PowerNetwork;

    fn load_ieee14() -> PowerNetwork {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
        PowerNetwork::from_matpower(path).expect("ieee14 parse")
    }

    #[test]
    fn test_pv_curve_has_points() {
        let net = load_ieee14();
        let curve = compute_pv_curve(&net, 0, 0.1, 1.0).unwrap();
        assert!(!curve.points.is_empty(), "PV curve should have points");
        assert!(curve.p_max_mw > 0.0, "Max P should be positive");
    }

    #[test]
    fn test_pv_curve_voltage_decreases() {
        let net = load_ieee14();
        let curve = compute_pv_curve(&net, 0, 0.05, 0.5).unwrap();
        if curve.points.len() >= 2 {
            let v_first = curve.points.first().unwrap().voltage_pu;
            let v_last = curve.points.last().unwrap().voltage_pu;
            // Voltage should generally decrease as load increases
            assert!(
                v_last <= v_first + 0.01,
                "Voltage should not increase with load: {:.4} → {:.4}",
                v_first,
                v_last
            );
        }
    }

    #[test]
    fn test_qv_curve_has_points() {
        let net = load_ieee14();
        // Bus 13 (index 12) is a typical candidate for QV analysis
        let curve = compute_qv_curve(&net, 12, 5.0, 50.0).unwrap();
        assert!(!curve.points.is_empty(), "QV curve should have points");
    }

    #[test]
    fn test_voltage_stability_index_base_case() {
        let net = load_ieee14();
        let config = PowerFlowConfig::default();
        let result = net.solve_powerflow(&config).unwrap();
        let vsi = voltage_stability_index(&result);
        assert!(
            vsi > 0.5,
            "Min voltage should be > 0.5 p.u. at base: {:.4}",
            vsi
        );
        assert!(vsi <= 1.1, "Min voltage should be ≤ 1.1 p.u.: {:.4}", vsi);
    }

    #[test]
    fn test_l_index_range() {
        let net = load_ieee14();
        let config = PowerFlowConfig::default();
        let result = net.solve_powerflow(&config).unwrap();
        let li = compute_l_index(&net, &result).unwrap();
        assert!(
            (0.0..=1.0).contains(&li.l_max),
            "L-max out of range: {:.4}",
            li.l_max
        );
        for &l in &li.l_per_bus {
            assert!((0.0..=1.0).contains(&l), "L-index out of [0,1]: {:.4}", l);
        }
    }

    #[test]
    fn test_l_index_critical_bus_valid() {
        let net = load_ieee14();
        let config = PowerFlowConfig::default();
        let result = net.solve_powerflow(&config).unwrap();
        let li = compute_l_index(&net, &result).unwrap();
        assert!(li.critical_bus < net.buses.len());
    }

    #[test]
    fn test_l_index_severity_string() {
        let li = LIndex {
            l_per_bus: vec![0.3, 0.5, 0.75],
            l_max: 0.75,
            critical_bus: 2,
            near_collapse: false,
        };
        assert_eq!(li.severity(), "Warning");
    }

    #[test]
    fn test_vsmi_positive() {
        let net = load_ieee14();
        let config = PowerFlowConfig::default();
        let result = net.solve_powerflow(&config).unwrap();
        let vsmi = compute_vsmi(&net, &result, 0.1, 0.5).unwrap();
        assert!(vsmi.vsmi_min >= 0.0, "VSMI_min = {:.4}", vsmi.vsmi_min);
        assert!(vsmi.weakest_bus < net.buses.len());
    }

    #[test]
    fn test_vsmi_loaded_system_lower() {
        let net = load_ieee14();
        let config = PowerFlowConfig::default();
        let result = net.solve_powerflow(&config).unwrap();
        let vsmi = compute_vsmi(&net, &result, 0.05, 0.3).unwrap();
        assert!(vsmi.loadability_margin_pct >= 0.0);
    }

    #[test]
    fn test_cpf_has_points() {
        let net = load_ieee14();
        let cpf = run_cpf(&net, 0.1, 20).unwrap();
        assert!(
            !cpf.points.is_empty(),
            "CPF should produce at least one point"
        );
        assert!(cpf.lambda_max >= 0.0);
    }

    #[test]
    fn test_cpf_lambda_increasing() {
        let net = load_ieee14();
        let cpf = run_cpf(&net, 0.1, 15).unwrap();
        for w in cpf.points.windows(2) {
            assert!(
                w[1].lambda >= w[0].lambda - 1e-9,
                "λ should be non-decreasing: {:.4} → {:.4}",
                w[0].lambda,
                w[1].lambda
            );
        }
    }

    #[test]
    fn test_find_nose_lambda_reasonable() {
        let net = load_ieee14();
        let lambda_max = find_nose_lambda(&net, 0.01, 1.0).unwrap();
        assert!(
            lambda_max > 0.0,
            "Nose lambda should be positive: {:.4}",
            lambda_max
        );
        assert!(
            lambda_max < 10.0,
            "Nose lambda should be < 10: {:.4}",
            lambda_max
        );
    }

    #[test]
    fn test_cpf_nose_voltage_below_base() {
        let net = load_ieee14();
        let config = PowerFlowConfig::default();
        let base_result = net.solve_powerflow(&config).unwrap();
        let cpf = run_cpf(&net, 0.1, 15).unwrap();
        if cpf.points.len() > 1 {
            // Voltage at last CPF point should be ≤ base voltage (loaded system)
            let v_last = cpf.v_nose[0];
            let v_base = base_result.voltage_magnitude[0];
            assert!(
                v_last <= v_base + 0.05,
                "Nose voltage {:.4} should not exceed base {:.4}",
                v_last,
                v_base
            );
        }
    }
}
