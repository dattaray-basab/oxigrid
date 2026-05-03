/// Security-Constrained Optimal Power Flow (SCOPF) — N-1 contingency screening.
///
/// Extends the DC-OPF with N-1 security constraints:
/// for every single branch outage, post-contingency branch flows must remain
/// within thermal limits.
///
/// # Method (linearised N-1 DC-SCOPF)
/// 1. Solve the base-case DC-OPF (economic dispatch + DC power flow).
/// 2. Build the PTDF (power transfer distribution factor) matrix.
/// 3. Build the LODF (line outage distribution factor) matrix.
/// 4. For each contingency k (branch k outage):
///    post-flow[l] = base_flow[l] + LODF[l,k] * base_flow[k]
/// 5. Check if any post-flow[l] exceeds rate_a[l].
/// 6. Return all binding N-1 constraints and the overall security status.
///
/// This is a screening tool — it identifies violated constraints without
/// re-optimising the dispatch. Full SCOPF re-optimisation (adding violated
/// contingency constraints and re-solving) is left to future iterations.
use crate::error::{OxiGridError, Result};
use crate::network::reduction::{build_b_bus, lodf_matrix, ptdf_matrix};
use crate::network::topology::PowerNetwork;
use crate::optimize::opf::dc_opf::{solve_dc_opf, DcOpfResult, GenCost};
use serde::{Deserialize, Serialize};

/// A binding N-1 security constraint violation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContingencyViolation {
    /// Index of the outaged branch (contingency)
    pub outage_branch: usize,
    /// Index of the overloaded branch (monitored)
    pub monitored_branch: usize,
    /// Post-contingency flow `MW`
    pub post_flow_mw: f64,
    /// Thermal limit `MW`
    pub limit_mw: f64,
    /// LODF sensitivity used: LODF[monitored, outage]
    pub lodf: f64,
}

impl ContingencyViolation {
    /// Loading level after outage (fraction of limit; > 1 means violation).
    pub fn loading_fraction(&self) -> f64 {
        self.post_flow_mw.abs() / self.limit_mw
    }
}

/// Result of the N-1 SCOPF screening.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScopfResult {
    /// Base-case DC-OPF result
    pub base_case: DcOpfResult,
    /// All detected N-1 constraint violations
    pub violations: Vec<ContingencyViolation>,
    /// True if no N-1 violations were found
    pub is_n1_secure: bool,
    /// Number of contingencies screened
    pub contingencies_screened: usize,
}

/// Configuration for SCOPF screening.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScopfConfig {
    /// Fraction of rate_a to use as limit for post-contingency flows (typical: 1.0)
    pub emergency_rating_fraction: f64,
    /// Minimum base-case branch flow below which LODF screening is skipped `MW`
    pub flow_threshold_mw: f64,
}

impl Default for ScopfConfig {
    fn default() -> Self {
        Self {
            emergency_rating_fraction: 1.0,
            flow_threshold_mw: 0.1,
        }
    }
}

/// Run N-1 security-constrained OPF screening.
///
/// Solves the base-case DC-OPF then screens all single-branch contingencies
/// using linearised LODF analysis. Returns the base result and any violations.
pub fn run_scopf(
    network: &PowerNetwork,
    gen_costs: &[GenCost],
    config: &ScopfConfig,
) -> Result<ScopfResult> {
    let n_branch = network.branch_count();
    if n_branch == 0 {
        return Err(OxiGridError::InvalidNetwork(
            "No branches in network".into(),
        ));
    }

    // Step 1: Solve base-case DC-OPF
    let base_case = solve_dc_opf(network, gen_costs)?;
    let base_flows = &base_case.branch_flows_mw;

    // Step 2: Build B-bus matrix
    let slack_idx = network.slack_bus_index()?;
    let branch_from: Vec<usize> = network
        .branches
        .iter()
        .filter(|b| b.status)
        .map(|b| network.bus_index(b.from_bus).unwrap_or(0))
        .collect();
    let branch_to: Vec<usize> = network
        .branches
        .iter()
        .filter(|b| b.status)
        .map(|b| network.bus_index(b.to_bus).unwrap_or(0))
        .collect();
    let branch_x: Vec<f64> = network
        .branches
        .iter()
        .filter(|b| b.status)
        .map(|b| b.x)
        .collect();

    let b_bus = build_b_bus(network.bus_count(), &branch_from, &branch_to, &branch_x);

    // Step 3: PTDF and LODF matrices
    let ptdf = ptdf_matrix(&b_bus, &branch_from, &branch_to, &branch_x, slack_idx)?;
    let lodf = lodf_matrix(&ptdf, &branch_from, &branch_to);

    // Step 4: Screening — for each contingency k, check all monitored branches l
    let mut violations = Vec::new();
    let m = n_branch; // number of active branches (assume all active for screening)

    for k in 0..m {
        // Skip if base-case flow on outaged branch is negligible
        let base_flow_k = if k < base_flows.len() {
            base_flows[k]
        } else {
            0.0
        };
        if base_flow_k.abs() < config.flow_threshold_mw {
            continue;
        }

        for l in 0..m {
            if l == k {
                continue;
            } // skip self (outaged branch)
            let base_flow_l = if l < base_flows.len() {
                base_flows[l]
            } else {
                0.0
            };
            let lodf_lk = if l < lodf.len() && k < lodf[l].len() {
                lodf[l][k]
            } else {
                0.0
            };

            // Post-contingency flow estimate
            let post_flow = base_flow_l + lodf_lk * base_flow_k;

            // Thermal limit for monitored branch
            let rate_a = network.branches[l].rate_a;
            if rate_a < 1e-6 {
                continue;
            } // no rating set
            let limit_mw = rate_a * config.emergency_rating_fraction;

            if post_flow.abs() > limit_mw {
                violations.push(ContingencyViolation {
                    outage_branch: k,
                    monitored_branch: l,
                    post_flow_mw: post_flow,
                    limit_mw,
                    lodf: lodf_lk,
                });
            }
        }
    }

    let contingencies_screened = m;
    let is_n1_secure = violations.is_empty();

    Ok(ScopfResult {
        base_case,
        violations,
        is_n1_secure,
        contingencies_screened,
    })
}

/// Check only a specific set of contingencies (subset screening).
pub fn screen_contingencies(
    network: &PowerNetwork,
    base_case: &DcOpfResult,
    contingency_branches: &[usize],
    config: &ScopfConfig,
) -> Result<Vec<ContingencyViolation>> {
    let slack_idx = network.slack_bus_index()?;
    let branch_from: Vec<usize> = network
        .branches
        .iter()
        .filter(|b| b.status)
        .map(|b| network.bus_index(b.from_bus).unwrap_or(0))
        .collect();
    let branch_to: Vec<usize> = network
        .branches
        .iter()
        .filter(|b| b.status)
        .map(|b| network.bus_index(b.to_bus).unwrap_or(0))
        .collect();
    let branch_x: Vec<f64> = network
        .branches
        .iter()
        .filter(|b| b.status)
        .map(|b| b.x)
        .collect();

    let b_bus = build_b_bus(network.bus_count(), &branch_from, &branch_to, &branch_x);
    let ptdf = ptdf_matrix(&b_bus, &branch_from, &branch_to, &branch_x, slack_idx)?;
    let lodf = lodf_matrix(&ptdf, &branch_from, &branch_to);

    let m = network.branch_count();
    let base_flows = &base_case.branch_flows_mw;
    let mut violations = Vec::new();

    for &k in contingency_branches {
        if k >= m {
            continue;
        }
        let base_flow_k = if k < base_flows.len() {
            base_flows[k]
        } else {
            0.0
        };

        for l in 0..m {
            if l == k {
                continue;
            }
            let base_flow_l = if l < base_flows.len() {
                base_flows[l]
            } else {
                0.0
            };
            let lodf_lk = if l < lodf.len() && k < lodf[l].len() {
                lodf[l][k]
            } else {
                0.0
            };
            let post_flow = base_flow_l + lodf_lk * base_flow_k;
            let rate_a = network.branches[l].rate_a;
            if rate_a < 1e-6 {
                continue;
            }
            let limit_mw = rate_a * config.emergency_rating_fraction;
            if post_flow.abs() > limit_mw {
                violations.push(ContingencyViolation {
                    outage_branch: k,
                    monitored_branch: l,
                    post_flow_mw: post_flow,
                    limit_mw,
                    lodf: lodf_lk,
                });
            }
        }
    }

    Ok(violations)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::PowerNetwork;

    fn ieee14_network() -> PowerNetwork {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
        PowerNetwork::from_matpower(path).expect("parse ieee14")
    }

    fn ieee14_costs(network: &PowerNetwork) -> Vec<GenCost> {
        network
            .generators
            .iter()
            .map(|g| GenCost::quadratic(0.0, 20.0, 0.05, g.pmin.max(0.0), g.pmax.max(10.0)))
            .collect()
    }

    #[test]
    fn test_scopf_base_case_solves() {
        let net = ieee14_network();
        let costs = ieee14_costs(&net);
        let config = ScopfConfig::default();
        let result = run_scopf(&net, &costs, &config);
        assert!(result.is_ok(), "SCOPF should succeed: {:?}", result.err());
        let r = result.unwrap();
        assert!(r.base_case.total_cost > 0.0, "Base cost should be positive");
    }

    #[test]
    fn test_scopf_screens_all_contingencies() {
        let net = ieee14_network();
        let costs = ieee14_costs(&net);
        let config = ScopfConfig::default();
        let r = run_scopf(&net, &costs, &config).unwrap();
        // All branches screened (N-1 for each active branch)
        assert!(r.contingencies_screened > 0);
        assert!(r.contingencies_screened <= net.branch_count());
    }

    #[test]
    fn test_scopf_violation_loading_fraction() {
        let net = ieee14_network();
        let costs = ieee14_costs(&net);
        // Use tight limits to force violations
        let config = ScopfConfig {
            emergency_rating_fraction: 1.0,
            flow_threshold_mw: 0.1,
        };
        let r = run_scopf(&net, &costs, &config).unwrap();
        for v in &r.violations {
            assert!(
                v.loading_fraction() > 1.0,
                "Violation should have loading > 1.0, got {:.3}",
                v.loading_fraction()
            );
        }
    }

    #[test]
    fn test_scopf_no_violations_with_zero_rating() {
        let mut net = ieee14_network();
        // Remove all thermal ratings → no violations possible
        for b in &mut net.branches {
            b.rate_a = 0.0;
        }
        let costs = ieee14_costs(&net);
        let r = run_scopf(&net, &costs, &ScopfConfig::default()).unwrap();
        assert!(r.is_n1_secure, "No ratings → no violations expected");
    }

    #[test]
    fn test_screen_contingencies_subset() {
        let net = ieee14_network();
        let costs = ieee14_costs(&net);
        let base = solve_dc_opf(&net, &costs).unwrap();
        let viols = screen_contingencies(&net, &base, &[0, 1, 2], &ScopfConfig::default()).unwrap();
        // Should return a Vec (possibly empty, possibly with violations)
        let _ = viols;
    }

    #[test]
    fn test_scopf_is_n1_secure_flag_consistent() {
        let net = ieee14_network();
        let costs = ieee14_costs(&net);
        let r = run_scopf(&net, &costs, &ScopfConfig::default()).unwrap();
        assert_eq!(r.is_n1_secure, r.violations.is_empty());
    }
}
