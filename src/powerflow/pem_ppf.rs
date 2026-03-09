//! Point Estimate Method (PEM) probabilistic power flow.
//!
//! Implements Hong's 2m+1 scheme: for each of m uncertain inputs, evaluate at
//! mean ± k·σ concentration points, plus the all-mean base case.
//! Total = 2m+1 deterministic power flows.

use crate::error::Result;
use crate::network::PowerNetwork;
use crate::powerflow::probabilistic::{
    normal_icdf, OutputStats, PpfConfig, PpfMethod, PpfResult, UncertainInjection,
};

/// Run a Point Estimate Method (PEM) probabilistic power flow.
///
/// Uses Hong's 2m+1 scheme: for each of m uncertain inputs, evaluate at
/// mean ± k·σ concentration points, plus the all-mean base case.
/// Total = 2m+1 deterministic power flows.
pub fn run_pem_ppf(
    network: &PowerNetwork,
    injections: &[UncertainInjection],
    config: &PpfConfig,
) -> Result<PpfResult> {
    let m = injections.len();
    let n_buses = network.buses.len();
    let n_branches = network.branches.len();
    let n_evaluations = 2 * m + 1;

    let mut v_wsum = vec![0.0f64; n_buses];
    let mut v_wxsum = vec![0.0f64; n_buses];
    let mut v_wx2sum = vec![0.0f64; n_buses];
    let mut p_wsum = vec![0.0f64; n_buses];
    let mut p_wxsum = vec![0.0f64; n_buses];
    let mut p_wx2sum = vec![0.0f64; n_buses];
    let mut f_wsum = vec![0.0f64; n_branches];
    let mut f_wxsum = vec![0.0f64; n_branches];
    let mut f_wx2sum = vec![0.0f64; n_branches];
    let mut n_diverged = 0usize;

    let w_pair = 1.0 / (2.0 * m.max(1) as f64);
    let w_base = if m > 0 {
        (m as f64 - 1.0) / m as f64
    } else {
        1.0
    };

    // Base case (all-mean)
    let base_result = evaluate_pem_point(network, injections, &[], config)?;
    if base_result.converged {
        accumulate_weighted(
            &base_result,
            w_base,
            n_buses,
            n_branches,
            &mut v_wsum,
            &mut v_wxsum,
            &mut v_wx2sum,
            &mut p_wsum,
            &mut p_wxsum,
            &mut p_wx2sum,
            &mut f_wsum,
            &mut f_wxsum,
            &mut f_wx2sum,
        );
    } else {
        n_diverged += 1;
    }

    // 2m concentration points
    for (k, inj) in injections.iter().enumerate() {
        let (mu, var, skew, _kurt) = inj.delta_p.moments();
        let sigma = var.sqrt().max(1e-9);
        let lambda = skew / 2.0;
        let xi_plus = lambda + (m as f64 - lambda * lambda).max(0.0).sqrt();
        let xi_minus = lambda - (m as f64 - lambda * lambda).max(0.0).sqrt();

        for xi in [xi_plus, xi_minus] {
            let delta = mu + xi * sigma;
            let perturbation = vec![(k, delta)];
            match evaluate_pem_point(network, injections, &perturbation, config) {
                Ok(res) if res.converged => {
                    accumulate_weighted(
                        &res,
                        w_pair,
                        n_buses,
                        n_branches,
                        &mut v_wsum,
                        &mut v_wxsum,
                        &mut v_wx2sum,
                        &mut p_wsum,
                        &mut p_wxsum,
                        &mut p_wx2sum,
                        &mut f_wsum,
                        &mut f_wxsum,
                        &mut f_wx2sum,
                    );
                }
                _ => n_diverged += 1,
            }
        }
    }

    let stats_from_moments = |wsum: &[f64], wxsum: &[f64], wx2sum: &[f64]| -> Vec<OutputStats> {
        wsum.iter()
            .zip(wxsum)
            .zip(wx2sum)
            .map(|((&w, &wx), &wx2)| {
                let mean = if w > 1e-12 { wx / w } else { 0.0 };
                let e_x2 = if w > 1e-12 { wx2 / w } else { 0.0 };
                let var = (e_x2 - mean * mean).max(0.0);
                let std_dev = var.sqrt();
                OutputStats {
                    n_samples: n_evaluations,
                    mean,
                    std_dev,
                    p05: mean + std_dev * normal_icdf(0.05),
                    p25: mean + std_dev * normal_icdf(0.25),
                    p50: mean,
                    p75: mean + std_dev * normal_icdf(0.75),
                    p95: mean + std_dev * normal_icdf(0.95),
                    p99: mean + std_dev * normal_icdf(0.99),
                    min: mean - 4.0 * std_dev,
                    max: mean + 4.0 * std_dev,
                }
            })
            .collect()
    };

    Ok(PpfResult {
        method: PpfMethod::PointEstimate,
        n_samples: n_evaluations,
        voltage_stats: stats_from_moments(&v_wsum, &v_wxsum, &v_wx2sum),
        p_injection_stats: stats_from_moments(&p_wsum, &p_wxsum, &p_wx2sum),
        branch_flow_stats: stats_from_moments(&f_wsum, &f_wxsum, &f_wx2sum),
        n_diverged,
    })
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

#[allow(clippy::too_many_arguments)]
pub(crate) fn accumulate_weighted(
    res: &crate::powerflow::PowerFlowResult,
    w: f64,
    n_buses: usize,
    n_branches: usize,
    v_wsum: &mut [f64],
    v_wxsum: &mut [f64],
    v_wx2sum: &mut [f64],
    p_wsum: &mut [f64],
    p_wxsum: &mut [f64],
    p_wx2sum: &mut [f64],
    f_wsum: &mut [f64],
    f_wxsum: &mut [f64],
    f_wx2sum: &mut [f64],
) {
    for (i, &v) in res.voltage_magnitude.iter().enumerate() {
        if i < n_buses {
            v_wsum[i] += w;
            v_wxsum[i] += w * v;
            v_wx2sum[i] += w * v * v;
        }
    }
    for (i, &p) in res.p_injected.iter().enumerate() {
        if i < n_buses {
            p_wsum[i] += w;
            p_wxsum[i] += w * p;
            p_wx2sum[i] += w * p * p;
        }
    }
    for (i, bf) in res.branch_flows.iter().enumerate() {
        if i < n_branches {
            let fp = bf.p_from_mw;
            f_wsum[i] += w;
            f_wxsum[i] += w * fp;
            f_wx2sum[i] += w * fp * fp;
        }
    }
}

pub(crate) fn evaluate_pem_point(
    network: &PowerNetwork,
    injections: &[UncertainInjection],
    perturbations: &[(usize, f64)],
    config: &PpfConfig,
) -> Result<crate::powerflow::PowerFlowResult> {
    let mut net_copy = network.clone();
    for (k, inj) in injections.iter().enumerate() {
        let mu = inj.delta_p.mean();
        let delta = perturbations
            .iter()
            .find(|(idx, _)| *idx == k)
            .map(|(_, v)| *v)
            .unwrap_or(mu);
        if let Some(bus) = net_copy.buses.iter_mut().find(|b| b.id == inj.bus_id) {
            bus.pd = crate::units::Power(bus.pd.0 - delta);
        }
    }
    net_copy.solve_powerflow(&config.pf_config)
}
