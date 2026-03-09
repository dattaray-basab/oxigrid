//! Monte Carlo and Latin Hypercube Sampling probabilistic power flow.
//!
//! Provides `run_mc_ppf` (random sampling) and `run_lhs_ppf` (stratified
//! Latin Hypercube Sampling) implementations for probabilistic power flow analysis.

use crate::error::Result;
use crate::network::PowerNetwork;
use crate::powerflow::probabilistic::{
    dummy_stats, lhs_sample, Lcg64, OutputStats, PpfConfig, PpfMethod, PpfResult,
    UncertainInjection,
};

/// Run a Monte Carlo probabilistic power flow.
///
/// For each sample: perturb injections → solve power flow → record outputs.
pub fn run_mc_ppf(
    network: &PowerNetwork,
    injections: &[UncertainInjection],
    config: &PpfConfig,
) -> Result<PpfResult> {
    let n_buses = network.buses.len();
    let n_branches = network.branches.len();
    let n = config.n_samples;

    let mut v_samples: Vec<Vec<f64>> = vec![Vec::with_capacity(n); n_buses];
    let mut p_samples: Vec<Vec<f64>> = vec![Vec::with_capacity(n); n_buses];
    let mut f_samples: Vec<Vec<f64>> = vec![Vec::with_capacity(n); n_branches];
    let mut n_diverged = 0usize;

    let mut rng = Lcg64::new(config.seed);

    for _ in 0..n {
        let mut net_copy = network.clone();
        for inj in injections {
            let delta_p = inj.delta_p.icdf(rng.next_f64());
            if let Some(bus) = net_copy.buses.iter_mut().find(|b| b.id == inj.bus_id) {
                bus.pd = crate::units::Power(bus.pd.0 - delta_p);
                if let Some(dq_dist) = &inj.delta_q {
                    let delta_q = dq_dist.icdf(rng.next_f64());
                    bus.qd = crate::units::ReactivePower(bus.qd.0 - delta_q);
                }
            }
        }
        match net_copy.solve_powerflow(&config.pf_config) {
            Ok(res) if res.converged => {
                for (i, &v) in res.voltage_magnitude.iter().enumerate() {
                    if i < n_buses {
                        v_samples[i].push(v);
                    }
                }
                for (i, &p) in res.p_injected.iter().enumerate() {
                    if i < n_buses {
                        p_samples[i].push(p);
                    }
                }
                for (i, bf) in res.branch_flows.iter().enumerate() {
                    if i < n_branches {
                        f_samples[i].push(bf.p_from_mw);
                    }
                }
            }
            _ => n_diverged += 1,
        }
    }

    let voltage_stats: Vec<OutputStats> = v_samples
        .into_iter()
        .map(|s| {
            if s.is_empty() {
                dummy_stats()
            } else {
                OutputStats::from_samples(s)
            }
        })
        .collect();
    let p_injection_stats: Vec<OutputStats> = p_samples
        .into_iter()
        .map(|s| {
            if s.is_empty() {
                dummy_stats()
            } else {
                OutputStats::from_samples(s)
            }
        })
        .collect();
    let branch_flow_stats: Vec<OutputStats> = f_samples
        .into_iter()
        .map(|s| {
            if s.is_empty() {
                dummy_stats()
            } else {
                OutputStats::from_samples(s)
            }
        })
        .collect();

    Ok(PpfResult {
        method: PpfMethod::MonteCarlo,
        n_samples: n,
        voltage_stats,
        p_injection_stats,
        branch_flow_stats,
        n_diverged,
    })
}

/// Run a Latin Hypercube Sampling probabilistic power flow.
///
/// Stratifies each input dimension into `n_samples` equal-probability bins,
/// randomly samples one value per stratum, and shuffles across dimensions.
pub fn run_lhs_ppf(
    network: &PowerNetwork,
    injections: &[UncertainInjection],
    config: &PpfConfig,
) -> Result<PpfResult> {
    let n = config.n_samples;
    let m = injections.len();
    let mut rng = Lcg64::new(config.seed);

    let lhs_matrix = lhs_sample(m, n, &mut rng);

    let samples_per_inj: Vec<Vec<f64>> = injections
        .iter()
        .zip(lhs_matrix.iter())
        .map(|(inj, u_row)| u_row.iter().map(|&u| inj.delta_p.icdf(u)).collect())
        .collect();

    let n_buses = network.buses.len();
    let n_branches = network.branches.len();
    let mut v_samples: Vec<Vec<f64>> = vec![Vec::with_capacity(n); n_buses];
    let mut p_samples: Vec<Vec<f64>> = vec![Vec::with_capacity(n); n_buses];
    let mut f_samples: Vec<Vec<f64>> = vec![Vec::with_capacity(n); n_branches];
    let mut n_diverged = 0usize;

    #[allow(clippy::needless_range_loop)]
    for s_idx in 0..n {
        let mut net_copy = network.clone();
        for (i, inj) in injections.iter().enumerate() {
            let delta_p = samples_per_inj[i][s_idx];
            if let Some(bus) = net_copy.buses.iter_mut().find(|b| b.id == inj.bus_id) {
                bus.pd = crate::units::Power(bus.pd.0 - delta_p);
            }
        }
        match net_copy.solve_powerflow(&config.pf_config) {
            Ok(res) if res.converged => {
                for (i, &v) in res.voltage_magnitude.iter().enumerate() {
                    if i < n_buses {
                        v_samples[i].push(v);
                    }
                }
                for (i, &p) in res.p_injected.iter().enumerate() {
                    if i < n_buses {
                        p_samples[i].push(p);
                    }
                }
                for (i, bf) in res.branch_flows.iter().enumerate() {
                    if i < n_branches {
                        f_samples[i].push(bf.p_from_mw);
                    }
                }
            }
            _ => n_diverged += 1,
        }
    }

    let voltage_stats: Vec<OutputStats> = v_samples
        .into_iter()
        .map(|s| {
            if s.is_empty() {
                dummy_stats()
            } else {
                OutputStats::from_samples(s)
            }
        })
        .collect();
    let p_injection_stats: Vec<OutputStats> = p_samples
        .into_iter()
        .map(|s| {
            if s.is_empty() {
                dummy_stats()
            } else {
                OutputStats::from_samples(s)
            }
        })
        .collect();
    let branch_flow_stats: Vec<OutputStats> = f_samples
        .into_iter()
        .map(|s| {
            if s.is_empty() {
                dummy_stats()
            } else {
                OutputStats::from_samples(s)
            }
        })
        .collect();

    Ok(PpfResult {
        method: PpfMethod::LatinHypercube,
        n_samples: n,
        voltage_stats,
        p_injection_stats,
        branch_flow_stats,
        n_diverged,
    })
}
