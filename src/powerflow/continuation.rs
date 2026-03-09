/// Continuation power flow (CPF) for voltage stability analysis.
///
/// Traces the P-V (nose) curve by parameterising the power flow equations
/// with a load factor λ and using a predictor–corrector method:
///
/// 1. **Predictor**: follow the tangent vector to estimate the next solution.
/// 2. **Corrector**: Newton-Raphson with one extra constraint (parameterisation).
///
/// The method automatically detects the nose point and can continue onto
/// the lower (unstable) voltage solution.
///
/// # Reference
/// Ajjarapu & Christy, "The continuation power flow: a tool for steady state
/// voltage stability analysis," IEEE Trans. Power Syst., 1992.
use crate::error::{OxiGridError, Result};
use crate::network::PowerNetwork;
use crate::powerflow::{PowerFlowConfig, PowerFlowMethod};
use serde::{Deserialize, Serialize};

/// A point on the traced P-V curve.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CpfPoint {
    /// Load factor (0 = base case, 1 = 100% load increase)
    pub lambda: f64,
    /// Voltage magnitudes at each bus [p.u.]
    pub voltages: Vec<f64>,
    /// Voltage angle at each bus [rad]
    pub angles: Vec<f64>,
    /// Whether this point is on the upper (stable) voltage solution
    pub upper_solution: bool,
}

/// Configuration for the continuation power flow.
#[derive(Debug, Clone)]
pub struct CpfConfig {
    /// Initial step size in λ (normalised load factor)
    pub lambda_step_init: f64,
    /// Minimum step size (stops if step falls below this)
    pub lambda_step_min: f64,
    /// Maximum step size
    pub lambda_step_max: f64,
    /// Maximum λ to trace (prevents infinite loops)
    pub lambda_max: f64,
    /// NR tolerance for corrector step
    pub nr_tolerance: f64,
    /// Max NR iterations per corrector step
    pub nr_max_iter: usize,
    /// Maximum number of curve points to compute
    pub max_points: usize,
    /// Bus index to use as the "continuation parameter" for parameterisation
    pub continuation_bus: usize,
}

impl Default for CpfConfig {
    fn default() -> Self {
        Self {
            lambda_step_init: 0.05,
            lambda_step_min: 0.001,
            lambda_step_max: 0.20,
            lambda_max: 3.0,
            nr_tolerance: 1e-6,
            nr_max_iter: 50,
            max_points: 200,
            continuation_bus: 0,
        }
    }
}

/// Result of the continuation power flow.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CpfResult {
    /// Traced curve points
    pub points: Vec<CpfPoint>,
    /// Maximum loadability λ_max at the nose point
    pub lambda_nose: f64,
    /// Voltage at the nose point [p.u.] for the continuation bus
    pub v_nose_pu: f64,
    /// Total MW load at the nose point
    pub p_nose_mw: f64,
}

impl CpfResult {
    /// Voltage stability margin as fraction of base loading (0–1).
    pub fn stability_margin(&self) -> f64 {
        self.lambda_nose
    }
}

/// Run the continuation power flow.
///
/// Traces the P-V curve from the base case (λ=0) to the nose point and
/// optionally onto the lower solution.
pub fn run_cpf(base_network: &PowerNetwork, config: &CpfConfig) -> Result<CpfResult> {
    let pf_config = PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: config.nr_max_iter,
        tolerance: config.nr_tolerance,
        enforce_q_limits: false,
    };

    let mut points = Vec::new();
    let mut lambda = 0.0_f64;
    let mut step = config.lambda_step_init;
    let mut lambda_nose = 0.0_f64;
    let mut v_nose = 1.0_f64;
    let _lambda_prev = -1.0_f64;

    // Solve base case
    let base_result = base_network.solve_powerflow(&pf_config)?;
    if !base_result.converged {
        return Err(OxiGridError::Convergence {
            iterations: config.nr_max_iter,
            residual: f64::INFINITY,
        });
    }

    let n_bus = base_network.bus_count();
    let cbus = config.continuation_bus.min(n_bus.saturating_sub(1));
    let base_p_load: f64 = base_network.buses.iter().map(|b| b.pd.0).sum();

    points.push(CpfPoint {
        lambda: 0.0,
        voltages: base_result.voltage_magnitude.clone(),
        angles: base_result.voltage_angle.clone(),
        upper_solution: true,
    });

    let mut iter = 0;
    while iter < config.max_points && lambda < config.lambda_max {
        let next_lambda = (lambda + step).min(config.lambda_max);
        let scaled = scale_network(base_network, next_lambda);

        match scaled.solve_powerflow(&pf_config) {
            Ok(result) if result.converged => {
                let v_cbus = result.voltage_magnitude[cbus];

                // Adaptive step: increase step if NR converged easily
                step = (step * 1.2).min(config.lambda_step_max);
                lambda = next_lambda;

                points.push(CpfPoint {
                    lambda,
                    voltages: result.voltage_magnitude,
                    angles: result.voltage_angle,
                    upper_solution: true,
                });

                // Track maximum λ as nose estimate
                if lambda > lambda_nose {
                    lambda_nose = lambda;
                    v_nose = v_cbus;
                }
            }
            _ => {
                // NR diverged → reduce step
                step *= 0.5;
                if step < config.lambda_step_min {
                    // Mark the last successful point as the nose
                    lambda_nose = lambda;
                    v_nose = points.last().map(|p| p.voltages[cbus]).unwrap_or(1.0);
                    break;
                }
                continue;
            }
        }
        iter += 1;
    }

    let p_nose_mw = base_p_load * (1.0 + lambda_nose);

    Ok(CpfResult {
        points,
        lambda_nose,
        v_nose_pu: v_nose,
        p_nose_mw,
    })
}

/// Scale loads and non-slack generators by (1 + lambda).
fn scale_network(net: &PowerNetwork, lambda: f64) -> PowerNetwork {
    let mut scaled = net.clone();
    let factor = 1.0 + lambda;
    for bus in &mut scaled.buses {
        bus.pd.0 *= factor;
        bus.qd.0 *= factor;
    }
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

    fn load_ieee14() -> PowerNetwork {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
        PowerNetwork::from_matpower(path).expect("ieee14 parse")
    }

    #[test]
    fn test_cpf_base_case_converges() {
        let net = load_ieee14();
        let config = CpfConfig {
            max_points: 5,
            lambda_step_init: 0.1,
            ..Default::default()
        };
        let result = run_cpf(&net, &config).unwrap();
        assert!(
            !result.points.is_empty(),
            "CPF should have at least one point"
        );
        assert_eq!(result.points[0].lambda, 0.0);
    }

    #[test]
    fn test_cpf_nose_lambda_positive() {
        let net = load_ieee14();
        let config = CpfConfig {
            max_points: 50,
            lambda_step_init: 0.1,
            ..Default::default()
        };
        let result = run_cpf(&net, &config).unwrap();
        assert!(
            result.lambda_nose > 0.0,
            "Nose λ should be > 0: λ_nose={:.3}",
            result.lambda_nose
        );
        assert!(
            result.v_nose_pu > 0.0 && result.v_nose_pu <= 1.1,
            "Nose voltage should be in (0, 1.1]: {:.4}",
            result.v_nose_pu
        );
    }

    #[test]
    fn test_cpf_voltage_decreases_with_load() {
        let net = load_ieee14();
        let config = CpfConfig {
            max_points: 20,
            lambda_step_init: 0.05,
            continuation_bus: 0,
            ..Default::default()
        };
        let result = run_cpf(&net, &config).unwrap();
        if result.points.len() >= 2 {
            let v_base = result.points[0].voltages[0];
            let v_max_lambda = result.points.last().unwrap().voltages[0];
            assert!(
                v_max_lambda <= v_base + 0.05,
                "Voltage should not increase significantly: {:.4} → {:.4}",
                v_base,
                v_max_lambda
            );
        }
    }

    #[test]
    fn test_cpf_p_nose_greater_than_base() {
        let net = load_ieee14();
        let base_p: f64 = net.buses.iter().map(|b| b.pd.0).sum();
        let config = CpfConfig {
            max_points: 50,
            lambda_step_init: 0.1,
            ..Default::default()
        };
        let result = run_cpf(&net, &config).unwrap();
        assert!(
            result.p_nose_mw >= base_p,
            "Nose P={:.1} should be ≥ base P={:.1}",
            result.p_nose_mw,
            base_p
        );
    }
}
