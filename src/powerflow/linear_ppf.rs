//! Linear sensitivity-based probabilistic power flow.
//!
//! Uses numerical sensitivity ∂V/∂P and ∂F/∂P via one perturbed solve per input.
//! Variances propagate analytically: Var\[y\] = Σ_k (∂y/∂P_k)² · Var\[ΔP_k\].

use crate::error::{OxiGridError, Result};
use crate::network::PowerNetwork;
use crate::powerflow::probabilistic::{
    normal_icdf, OutputStats, PpfConfig, PpfMethod, PpfResult, UncertainInjection,
};
use crate::powerflow::{PowerFlowConfig, PowerFlowMethod};

/// Linear (sensitivity-based) PPF.
///
/// Uses numerical sensitivity ∂V/∂P and ∂F/∂P via one perturbed solve per input.
/// Variances propagate analytically: Var\[y\] = Σ_k (∂y/∂P_k)² · Var\[ΔP_k\].
pub fn run_linear_ppf(
    network: &PowerNetwork,
    injections: &[UncertainInjection],
    config: &PpfConfig,
) -> Result<PpfResult> {
    let n_buses = network.buses.len();
    let n_branches = network.branches.len();
    let eps = 1.0f64; // 1 MW perturbation

    // Linear sensitivity requires AC voltage magnitudes — always use Newton-Raphson.
    let ac_pf_config = PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        warm_start: None,
        ..config.pf_config.clone()
    };
    let base_result = network.solve_powerflow(&ac_pf_config)?;
    if !base_result.converged {
        return Err(OxiGridError::Convergence {
            iterations: config.pf_config.max_iter,
            residual: f64::INFINITY,
        });
    }

    let mut v_sensitivity: Vec<Vec<f64>> = Vec::new();
    let mut f_sensitivity: Vec<Vec<f64>> = Vec::new();

    for inj in injections {
        let mut net_p = network.clone();
        if let Some(bus) = net_p.buses.iter_mut().find(|b| b.id == inj.bus_id) {
            bus.pd = crate::units::Power(bus.pd.0 - eps);
        }
        let res_p = net_p.solve_powerflow(&ac_pf_config)?;

        let dv: Vec<f64> = res_p
            .voltage_magnitude
            .iter()
            .zip(&base_result.voltage_magnitude)
            .map(|(&vp, &vb)| (vp - vb) / eps)
            .collect();
        v_sensitivity.push(dv);

        let df: Vec<f64> = res_p
            .branch_flows
            .iter()
            .zip(&base_result.branch_flows)
            .map(|(fp, fb)| (fp.p_from_mw - fb.p_from_mw) / eps)
            .collect();
        f_sensitivity.push(df);
    }

    let mut v_var = vec![0.0f64; n_buses];
    let mut f_var = vec![0.0f64; n_branches];
    for (k, inj) in injections.iter().enumerate() {
        let var_k = inj.delta_p.variance();
        for (i, vv) in v_var.iter_mut().enumerate() {
            if i < v_sensitivity[k].len() {
                *vv += v_sensitivity[k][i].powi(2) * var_k;
            }
        }
        for (i, fv) in f_var.iter_mut().enumerate() {
            if i < f_sensitivity[k].len() {
                *fv += f_sensitivity[k][i].powi(2) * var_k;
            }
        }
    }

    let make_stats = |means: &[f64], variances: &[f64]| -> Vec<OutputStats> {
        means
            .iter()
            .zip(variances)
            .map(|(&mu, &var)| {
                let std_dev = var.sqrt();
                OutputStats {
                    n_samples: injections.len() + 1,
                    mean: mu,
                    std_dev,
                    p05: mu + std_dev * normal_icdf(0.05),
                    p25: mu + std_dev * normal_icdf(0.25),
                    p50: mu,
                    p75: mu + std_dev * normal_icdf(0.75),
                    p95: mu + std_dev * normal_icdf(0.95),
                    p99: mu + std_dev * normal_icdf(0.99),
                    min: mu - 4.0 * std_dev,
                    max: mu + 4.0 * std_dev,
                }
            })
            .collect()
    };

    let f_means: Vec<f64> = base_result
        .branch_flows
        .iter()
        .map(|b| b.p_from_mw)
        .collect();
    let p_vars = vec![0.0f64; n_buses];

    Ok(PpfResult {
        method: PpfMethod::LinearSensitivity,
        n_samples: injections.len() + 1,
        voltage_stats: make_stats(&base_result.voltage_magnitude, &v_var),
        p_injection_stats: make_stats(&base_result.p_injected, &p_vars),
        branch_flow_stats: make_stats(&f_means, &f_var),
        n_diverged: 0,
    })
}
