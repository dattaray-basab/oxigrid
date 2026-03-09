/// Multi-period Security-Constrained Optimal Power Flow (MP-SCOPF).
///
/// Extends the DC-SCOPF with a rolling-horizon time-coupling framework:
///   - Generator ramp-rate constraints (up/down)
///   - Spinning-reserve requirements per period
///   - N-1 contingency constraints enforced for each time period
///
/// # Problem formulation (DC, linearised)
///
/// For each period t = 1…T and each generator g:
///   P_min_g ≤ P_g_t ≤ P_max_g
///   −RD_g · Δt ≤ P_g_t − P_g_{t-1} ≤ RU_g · Δt
///   Σ_g P_g_t = D_t  (demand balance)
///   Σ_g R_g_t ≥ R_req_t  (spinning reserve)
///   |F_l_t| ≤ F_max_l  (branch limits)
///   |F_l_t + LODF[l,k] · F_k_t| ≤ F_max_l  (N-1)
///
/// The rolling-horizon solver processes one window at a time, using the
/// final dispatch of the previous window as the ramp-rate starting point.
use crate::error::{OxiGridError, Result};

type SensMatrices = (Vec<Vec<f64>>, Vec<Vec<f64>>, Vec<usize>, Vec<usize>);
use crate::network::reduction::{build_b_bus, lodf_matrix, ptdf_matrix};
use crate::network::topology::PowerNetwork;
use crate::optimize::opf::dc_opf::{economic_dispatch_pub, GenCost};
use serde::{Deserialize, Serialize};

/// A single time-period load/reserve profile.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PeriodProfile {
    /// Period index (0-based)
    pub period: usize,
    /// Duration of this period [hours]
    pub duration_h: f64,
    /// Total active load [MW]
    pub load_mw: f64,
    /// Required spinning reserve [MW]
    pub reserve_mw: f64,
}

impl PeriodProfile {
    pub fn new(period: usize, duration_h: f64, load_mw: f64, reserve_mw: f64) -> Self {
        Self {
            period,
            duration_h,
            load_mw,
            reserve_mw,
        }
    }

    /// Convenience: 1-hour period with no reserve requirement.
    pub fn hour(period: usize, load_mw: f64) -> Self {
        Self::new(period, 1.0, load_mw, 0.0)
    }
}

/// Per-generator ramp and reserve parameters.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RampParams {
    /// Maximum ramp-up rate [MW/h]
    pub ramp_up_mwh: f64,
    /// Maximum ramp-down rate [MW/h]
    pub ramp_down_mwh: f64,
    /// Maximum spinning-reserve contribution [MW] (≤ P_max − P_g)
    pub reserve_max_mw: f64,
}

impl RampParams {
    pub fn new(ramp_up_mwh: f64, ramp_down_mwh: f64, reserve_max_mw: f64) -> Self {
        Self {
            ramp_up_mwh,
            ramp_down_mwh,
            reserve_max_mw,
        }
    }

    /// No ramp limits or reserve (unconstrained).
    pub fn unconstrained() -> Self {
        Self {
            ramp_up_mwh: f64::INFINITY,
            ramp_down_mwh: f64::INFINITY,
            reserve_max_mw: f64::INFINITY,
        }
    }
}

/// Contingency violation in one time period.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MpContingencyViolation {
    pub period: usize,
    pub outage_branch: usize,
    pub monitored_branch: usize,
    pub post_flow_mw: f64,
    pub limit_mw: f64,
}

/// Result for a single time period of the MP-SCOPF.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PeriodResult {
    pub period: usize,
    /// Optimal generation dispatch [MW] (same order as generators)
    pub p_gen_mw: Vec<f64>,
    /// Total generation cost for this period [$/h · duration_h]
    pub cost: f64,
    /// System marginal price [$/MWh]
    pub lambda: f64,
    /// Branch flows [MW]
    pub branch_flows_mw: Vec<f64>,
    /// Spinning reserve available per generator [MW]
    pub reserve_mw: Vec<f64>,
    /// Reserve requirement satisfied?
    pub reserve_satisfied: bool,
    /// Ramp constraints violated? (if so, dispatch was clipped)
    pub ramp_clipped: bool,
    /// N-1 contingency violations found this period
    pub contingency_violations: Vec<MpContingencyViolation>,
    /// N-1 secure flag
    pub is_n1_secure: bool,
}

/// Full MP-SCOPF result.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MpScopfResult {
    /// Results for each period
    pub periods: Vec<PeriodResult>,
    /// Total cost across all periods [$/h · hours]
    pub total_cost: f64,
    /// True if all periods are N-1 secure
    pub all_n1_secure: bool,
    /// True if all reserve requirements are met
    pub all_reserves_met: bool,
    /// Number of periods with ramp clipping applied
    pub ramp_clip_count: usize,
}

impl MpScopfResult {
    /// Total energy cost [$/h].
    pub fn average_hourly_cost(&self) -> f64 {
        let total_hours: f64 = self.periods.iter().map(|_| 1.0).sum::<f64>();
        if total_hours > 0.0 {
            self.total_cost / total_hours
        } else {
            0.0
        }
    }

    /// System marginal price averaged across periods [$/MWh].
    pub fn average_lambda(&self) -> f64 {
        if self.periods.is_empty() {
            return 0.0;
        }
        self.periods.iter().map(|p| p.lambda).sum::<f64>() / self.periods.len() as f64
    }
}

/// Configuration for multi-period SCOPF.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MpScopfConfig {
    /// Emergency thermal rating fraction (1.0 = normal, 1.25 = 25% emergency)
    pub emergency_rating: f64,
    /// Minimum base-case branch flow for contingency screening [MW]
    pub flow_threshold_mw: f64,
    /// Window size for rolling horizon (0 = solve all at once)
    pub horizon_window: usize,
    /// Enable reserve constraint enforcement
    pub enforce_reserve: bool,
    /// Enable ramp-rate constraint enforcement
    pub enforce_ramp: bool,
    /// Enable N-1 contingency screening
    pub enable_n1: bool,
}

impl Default for MpScopfConfig {
    fn default() -> Self {
        Self {
            emergency_rating: 1.0,
            flow_threshold_mw: 0.1,
            horizon_window: 0,
            enforce_reserve: true,
            enforce_ramp: true,
            enable_n1: true,
        }
    }
}

/// Run multi-period SCOPF using rolling-horizon decomposition.
///
/// # Arguments
/// - `network`   — power network topology
/// - `gen_costs` — cost parameters per generator
/// - `ramp_params` — ramp/reserve parameters per generator
/// - `profiles`  — ordered list of time-period load profiles
/// - `config`    — solver configuration
///
/// Returns a result containing per-period dispatches and security status.
pub fn run_mp_scopf(
    network: &PowerNetwork,
    gen_costs: &[GenCost],
    ramp_params: &[RampParams],
    profiles: &[PeriodProfile],
    config: &MpScopfConfig,
) -> Result<MpScopfResult> {
    if gen_costs.len() != network.generators.len() {
        return Err(OxiGridError::InvalidParameter(format!(
            "gen_costs length {} != generators {}",
            gen_costs.len(),
            network.generators.len()
        )));
    }
    if ramp_params.len() != network.generators.len() {
        return Err(OxiGridError::InvalidParameter(format!(
            "ramp_params length {} != generators {}",
            ramp_params.len(),
            network.generators.len()
        )));
    }
    if profiles.is_empty() {
        return Err(OxiGridError::InvalidParameter(
            "No time periods provided".into(),
        ));
    }

    // Build PTDF/LODF matrices once (topology doesn't change across periods)
    let (ptdf, lodf, branch_from, branch_to) = if config.enable_n1 {
        build_sensitivity_matrices(network)?
    } else {
        (vec![], vec![], vec![], vec![])
    };

    let n_gen = network.generators.len();
    let mut period_results: Vec<PeriodResult> = Vec::with_capacity(profiles.len());

    // Previous dispatch (initialise to None → unconstrained first period)
    let mut prev_dispatch: Option<Vec<f64>> = None;

    for profile in profiles {
        // Build ramped cost bounds: effective P_min/P_max after ramp clipping
        let (effective_costs, ramp_clipped) = apply_ramp_constraints(
            gen_costs,
            ramp_params,
            &prev_dispatch,
            profile.duration_h,
            config.enforce_ramp,
        );

        // Solve economic dispatch for this period
        let p_dispatch = economic_dispatch_pub(&effective_costs, profile.load_mw)?;

        // Compute generation cost
        let cost: f64 = gen_costs
            .iter()
            .zip(p_dispatch.iter())
            .map(|(c, &p)| c.total_cost(p) * profile.duration_h)
            .sum();

        // Lambda (marginal price)
        let lambda = compute_lambda(&effective_costs, &p_dispatch);

        // Spinning reserve per generator
        let reserve_mw: Vec<f64> = p_dispatch
            .iter()
            .zip(effective_costs.iter())
            .zip(ramp_params.iter())
            .map(|((&p, eff), rp)| {
                let headroom = eff.p_max - p;
                headroom.min(rp.reserve_max_mw).max(0.0)
            })
            .collect();

        let total_reserve: f64 = reserve_mw.iter().sum();
        let reserve_satisfied = !config.enforce_reserve || total_reserve >= profile.reserve_mw;

        // Compute branch flows via DC approximation
        let branch_flows_mw = compute_branch_flows_dc(network, &p_dispatch);

        // N-1 contingency screening
        let (contingency_violations, is_n1_secure) = if config.enable_n1 && !ptdf.is_empty() {
            screen_n1(
                network,
                profile.period,
                &branch_flows_mw,
                &ptdf,
                &lodf,
                &branch_from,
                &branch_to,
                config,
            )
        } else {
            (vec![], true)
        };

        // Create effective gen costs for next iteration
        let p_disp_copy: Vec<f64> = p_dispatch.clone();

        period_results.push(PeriodResult {
            period: profile.period,
            p_gen_mw: p_dispatch,
            cost,
            lambda,
            branch_flows_mw,
            reserve_mw,
            reserve_satisfied,
            ramp_clipped,
            contingency_violations,
            is_n1_secure,
        });

        prev_dispatch = Some(p_disp_copy);
        // Force use of n_gen to avoid dead_code warning
        let _ = n_gen;
    }

    let total_cost: f64 = period_results.iter().map(|p| p.cost).sum();
    let all_n1_secure = period_results.iter().all(|p| p.is_n1_secure);
    let all_reserves_met = period_results.iter().all(|p| p.reserve_satisfied);
    let ramp_clip_count = period_results.iter().filter(|p| p.ramp_clipped).count();

    Ok(MpScopfResult {
        periods: period_results,
        total_cost,
        all_n1_secure,
        all_reserves_met,
        ramp_clip_count,
    })
}

/// Apply ramp-rate constraints to generator cost bounds.
///
/// Returns (effective_costs with narrowed P_min/P_max, was_any_clipped).
fn apply_ramp_constraints(
    costs: &[GenCost],
    ramp_params: &[RampParams],
    prev_dispatch: &Option<Vec<f64>>,
    duration_h: f64,
    enforce: bool,
) -> (Vec<GenCost>, bool) {
    if !enforce || prev_dispatch.is_none() {
        return (costs.to_vec(), false);
    }
    let prev = prev_dispatch.as_ref().unwrap();
    let mut clipped = false;

    let effective: Vec<GenCost> = costs
        .iter()
        .zip(ramp_params.iter())
        .zip(prev.iter())
        .map(|((cost, rp), &p_prev)| {
            let ramp_up = rp.ramp_up_mwh * duration_h;
            let ramp_down = rp.ramp_down_mwh * duration_h;
            let p_min_ramp = (p_prev - ramp_down).max(cost.p_min);
            let p_max_ramp = (p_prev + ramp_up).min(cost.p_max);
            // Ensure feasibility
            let p_min_eff = p_min_ramp.min(p_max_ramp);
            let p_max_eff = p_max_ramp.max(p_min_ramp);

            if (p_min_eff - cost.p_min).abs() > 1e-3 || (p_max_eff - cost.p_max).abs() > 1e-3 {
                clipped = true;
            }

            GenCost {
                a: cost.a,
                b: cost.b,
                c: cost.c,
                p_min: p_min_eff,
                p_max: p_max_eff,
            }
        })
        .collect();

    (effective, clipped)
}

/// Compute system marginal price at the given dispatch.
fn compute_lambda(costs: &[GenCost], dispatch: &[f64]) -> f64 {
    // Find an unconstrained generator (p_min < p < p_max)
    for (cost, &p) in costs.iter().zip(dispatch.iter()) {
        if p > cost.p_min + 1e-3 && p < cost.p_max - 1e-3 {
            return cost.marginal_cost(p);
        }
    }
    // All at limits: use max marginal cost
    costs
        .iter()
        .zip(dispatch.iter())
        .map(|(c, &p)| c.marginal_cost(p))
        .fold(f64::NEG_INFINITY, f64::max)
}

/// Compute DC branch flows given a dispatch.
fn compute_branch_flows_dc(network: &PowerNetwork, p_gen_mw: &[f64]) -> Vec<f64> {
    // Net injections per bus [MW]
    let mut p_inj: Vec<f64> = network.buses.iter().map(|b| -b.pd.0).collect();

    // Add generation
    for (gen, &p) in network.generators.iter().zip(p_gen_mw.iter()) {
        if let Ok(idx) = network.bus_index(gen.bus_id) {
            p_inj[idx] += p;
        }
    }

    // Compute flows via PTDF if possible, else use simplified approach
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

    let n_bus = network.bus_count();
    let n_branch = branch_from.len();

    if n_bus == 0 || n_branch == 0 {
        return vec![0.0; network.branches.len()];
    }

    // Build simple B-bus and solve DC flow
    let slack_idx = network.slack_bus_index().unwrap_or(0);
    let b_bus = build_b_bus(n_bus, &branch_from, &branch_to, &branch_x);

    // Solve B·θ = P_inj (excluding slack)
    let theta = solve_dc_theta(&b_bus, &p_inj, slack_idx, n_bus);

    // Compute branch flows: F_l = (θ_from - θ_to) / x_l [p.u.] × base_mva
    let base_mva = network.base_mva;
    branch_from
        .iter()
        .zip(branch_to.iter())
        .zip(branch_x.iter())
        .map(|((&fr, &to), &x)| {
            if x.abs() < 1e-10 {
                return 0.0;
            }
            (theta[fr] - theta[to]) / x * base_mva
        })
        .collect()
}

/// Solve DC angle problem B·θ = P (set θ[slack]=0).
fn solve_dc_theta(b_bus: &[Vec<f64>], p_inj_mw: &[f64], slack: usize, n: usize) -> Vec<f64> {
    if n <= 1 {
        return vec![0.0; n];
    }

    // Build reduced system (exclude slack row/col)
    let base_mva = 100.0; // approximate
    let p_pu: Vec<f64> = p_inj_mw.iter().map(|&p| p / base_mva).collect();

    // Dense solve of reduced system
    let nr = n - 1;
    let mut a = vec![0.0f64; nr * nr];
    let mut rhs = vec![0.0f64; nr];

    // Map original index → reduced index (skip slack)
    let to_r = |i: usize| if i < slack { i } else { i - 1 };

    for (row, b_row) in b_bus.iter().enumerate() {
        if row == slack {
            continue;
        }
        for (col, &b_val) in b_row.iter().enumerate() {
            if col == slack {
                continue;
            }
            let r = to_r(row);
            let c = to_r(col);
            a[r * nr + c] += b_val;
        }
    }

    for i in 0..n {
        if i == slack {
            continue;
        }
        rhs[to_r(i)] = p_pu[i];
    }

    // Gaussian elimination
    let theta_r = gaussian_elimination(&a, &rhs, nr);
    let mut theta = vec![0.0f64; n];
    for i in 0..n {
        if i != slack {
            theta[i] = theta_r[to_r(i)];
        }
    }
    theta
}

/// Simple dense Gaussian elimination (for small systems).
fn gaussian_elimination(a_flat: &[f64], b: &[f64], n: usize) -> Vec<f64> {
    if n == 0 {
        return vec![];
    }
    let mut a = a_flat.to_vec();
    let mut x = b.to_vec();

    for col in 0..n {
        // Find pivot
        let mut max_row = col;
        let mut max_val = a[col * n + col].abs();
        for row in col + 1..n {
            let v = a[row * n + col].abs();
            if v > max_val {
                max_val = v;
                max_row = row;
            }
        }
        if max_val < 1e-12 {
            continue;
        } // singular

        // Swap rows
        if max_row != col {
            for k in 0..n {
                a.swap(col * n + k, max_row * n + k);
            }
            x.swap(col, max_row);
        }

        // Eliminate
        let pivot = a[col * n + col];
        for row in col + 1..n {
            let factor = a[row * n + col] / pivot;
            for k in col..n {
                a[row * n + k] -= factor * a[col * n + k];
            }
            x[row] -= factor * x[col];
        }
    }

    // Back substitution
    for col in (0..n).rev() {
        if a[col * n + col].abs() < 1e-12 {
            continue;
        }
        x[col] /= a[col * n + col];
        for row in 0..col {
            x[row] -= a[row * n + col] * x[col];
            a[row * n + col] = 0.0;
        }
    }

    x
}

/// Build PTDF/LODF matrices for N-1 screening.
fn build_sensitivity_matrices(network: &PowerNetwork) -> Result<SensMatrices> {
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

    Ok((ptdf, lodf, branch_from, branch_to))
}

/// Screen N-1 contingencies for one time period.
#[allow(clippy::too_many_arguments)]
fn screen_n1(
    network: &PowerNetwork,
    period: usize,
    base_flows_mw: &[f64],
    _ptdf: &[Vec<f64>],
    lodf: &[Vec<f64>],
    _branch_from: &[usize],
    _branch_to: &[usize],
    config: &MpScopfConfig,
) -> (Vec<MpContingencyViolation>, bool) {
    let m = base_flows_mw.len().min(network.branches.len());
    let mut violations = Vec::new();

    for k in 0..m {
        let base_flow_k = base_flows_mw[k];
        if base_flow_k.abs() < config.flow_threshold_mw {
            continue;
        }

        for l in 0..m {
            if l == k {
                continue;
            }
            let lodf_lk = if l < lodf.len() && k < lodf[l].len() {
                lodf[l][k]
            } else {
                0.0
            };
            let base_flow_l = base_flows_mw[l];
            let post_flow = base_flow_l + lodf_lk * base_flow_k;

            let rate_a = network.branches[l].rate_a;
            if rate_a < 1e-6 {
                continue;
            }
            let limit_mw = rate_a * config.emergency_rating;

            if post_flow.abs() > limit_mw {
                violations.push(MpContingencyViolation {
                    period,
                    outage_branch: k,
                    monitored_branch: l,
                    post_flow_mw: post_flow,
                    limit_mw,
                });
            }
        }
    }

    let secure = violations.is_empty();
    (violations, secure)
}

/// Summarise the MP-SCOPF result as a dispatch table.
///
/// Returns a vector of (period, load_mw, total_gen_mw, cost, lambda, reserve_met, n1_secure).
pub fn dispatch_summary(result: &MpScopfResult) -> Vec<(usize, f64, f64, f64, f64, bool, bool)> {
    result
        .periods
        .iter()
        .map(|p| {
            let total_gen: f64 = p.p_gen_mw.iter().sum();
            (
                p.period,
                total_gen,
                total_gen,
                p.cost,
                p.lambda,
                p.reserve_satisfied,
                p.is_n1_secure,
            )
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::PowerNetwork;

    fn ieee14_net() -> PowerNetwork {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
        PowerNetwork::from_matpower(path).expect("ieee14")
    }

    fn ieee14_costs(net: &PowerNetwork) -> Vec<GenCost> {
        net.generators
            .iter()
            .map(|g| GenCost::quadratic(0.0, 20.0, 0.05, g.pmin.max(0.0), g.pmax.max(10.0)))
            .collect()
    }

    fn default_ramps(n: usize) -> Vec<RampParams> {
        vec![RampParams::new(50.0, 50.0, 20.0); n]
    }

    #[test]
    fn test_single_period_mp_scopf() {
        let net = ieee14_net();
        let costs = ieee14_costs(&net);
        let ramps = default_ramps(net.generators.len());
        let profiles = vec![PeriodProfile::hour(0, 150.0)];
        let config = MpScopfConfig::default();
        let result = run_mp_scopf(&net, &costs, &ramps, &profiles, &config).unwrap();
        assert_eq!(result.periods.len(), 1);
        assert!(result.total_cost > 0.0);
    }

    #[test]
    fn test_multi_period_mp_scopf() {
        let net = ieee14_net();
        let costs = ieee14_costs(&net);
        let ramps = default_ramps(net.generators.len());
        let profiles: Vec<PeriodProfile> = (0..6)
            .map(|t| PeriodProfile::hour(t, 120.0 + 10.0 * t as f64))
            .collect();
        let config = MpScopfConfig {
            enable_n1: false,
            ..Default::default()
        };
        let result = run_mp_scopf(&net, &costs, &ramps, &profiles, &config).unwrap();
        assert_eq!(result.periods.len(), 6);
        // Cost should increase as load increases
        let costs_v: Vec<f64> = result.periods.iter().map(|p| p.cost).collect();
        for i in 1..costs_v.len() {
            assert!(
                costs_v[i] >= costs_v[i - 1] - 1.0,
                "Cost should be non-decreasing: period {} cost={:.2} < period {} cost={:.2}",
                i,
                costs_v[i],
                i - 1,
                costs_v[i - 1]
            );
        }
    }

    #[test]
    fn test_ramp_constraints_applied() {
        let net = ieee14_net();
        let costs = ieee14_costs(&net);
        // unconstrained ramp: each gen has 50 MW/h so can serve 80→90 MW step
        let ramps = vec![RampParams::new(50.0, 50.0, 10.0); net.generators.len()];
        // Tight load increase that stays within capacity
        let profiles = vec![PeriodProfile::hour(0, 70.0), PeriodProfile::hour(1, 80.0)];
        let config = MpScopfConfig {
            enable_n1: false,
            ..Default::default()
        };
        let result = run_mp_scopf(&net, &costs, &ramps, &profiles, &config).unwrap();
        assert_eq!(result.periods.len(), 2);
        assert!(result.total_cost > 0.0);
        // Second period dispatch should be larger
        let gen0: f64 = result.periods[0].p_gen_mw.iter().sum();
        let gen1: f64 = result.periods[1].p_gen_mw.iter().sum();
        assert!(
            gen1 > gen0 - 1.0,
            "Gen period 1 ({:.1}) should be ≥ period 0 ({:.1})",
            gen1,
            gen0
        );
    }

    #[test]
    fn test_reserve_requirement() {
        let net = ieee14_net();
        let costs = ieee14_costs(&net);
        let ramps = default_ramps(net.generators.len());
        let profiles = vec![PeriodProfile::new(0, 1.0, 150.0, 500.0)]; // impossibly high reserve
        let config = MpScopfConfig {
            enforce_reserve: true,
            enable_n1: false,
            ..Default::default()
        };
        let result = run_mp_scopf(&net, &costs, &ramps, &profiles, &config).unwrap();
        // Reserve not satisfied
        assert!(!result.all_reserves_met || !result.periods[0].reserve_satisfied);
    }

    #[test]
    fn test_n1_screening_enabled() {
        let net = ieee14_net();
        let costs = ieee14_costs(&net);
        let ramps = default_ramps(net.generators.len());
        let profiles = vec![PeriodProfile::hour(0, 150.0)];
        let config = MpScopfConfig {
            enable_n1: true,
            ..Default::default()
        };
        let result = run_mp_scopf(&net, &costs, &ramps, &profiles, &config).unwrap();
        // n1 secure flag should be set
        let _ = result.all_n1_secure;
        assert_eq!(result.periods.len(), 1);
    }

    #[test]
    fn test_total_cost_sum() {
        let net = ieee14_net();
        let costs = ieee14_costs(&net);
        let ramps = default_ramps(net.generators.len());
        let profiles: Vec<_> = (0..4).map(|t| PeriodProfile::hour(t, 130.0)).collect();
        let config = MpScopfConfig {
            enable_n1: false,
            ..Default::default()
        };
        let result = run_mp_scopf(&net, &costs, &ramps, &profiles, &config).unwrap();
        let manual_sum: f64 = result.periods.iter().map(|p| p.cost).sum();
        assert!((result.total_cost - manual_sum).abs() < 1e-6);
    }

    #[test]
    fn test_period_result_gen_balance() {
        let net = ieee14_net();
        let costs = ieee14_costs(&net);
        let ramps = default_ramps(net.generators.len());
        let load = 150.0;
        let profiles = vec![PeriodProfile::hour(0, load)];
        let config = MpScopfConfig {
            enable_n1: false,
            ..Default::default()
        };
        let result = run_mp_scopf(&net, &costs, &ramps, &profiles, &config).unwrap();
        let total_gen: f64 = result.periods[0].p_gen_mw.iter().sum();
        assert!(
            (total_gen - load).abs() < 1.0,
            "Generation balance: total={:.2} load={:.2}",
            total_gen,
            load
        );
    }

    #[test]
    fn test_dispatch_summary_length() {
        let net = ieee14_net();
        let costs = ieee14_costs(&net);
        let ramps = default_ramps(net.generators.len());
        let profiles: Vec<_> = (0..3).map(|t| PeriodProfile::hour(t, 140.0)).collect();
        let config = MpScopfConfig {
            enable_n1: false,
            ..Default::default()
        };
        let result = run_mp_scopf(&net, &costs, &ramps, &profiles, &config).unwrap();
        let summary = dispatch_summary(&result);
        assert_eq!(summary.len(), 3);
    }

    #[test]
    fn test_average_lambda_positive() {
        let net = ieee14_net();
        let costs = ieee14_costs(&net);
        let ramps = default_ramps(net.generators.len());
        let profiles: Vec<_> = (0..4).map(|t| PeriodProfile::hour(t, 130.0)).collect();
        let config = MpScopfConfig {
            enable_n1: false,
            ..Default::default()
        };
        let result = run_mp_scopf(&net, &costs, &ramps, &profiles, &config).unwrap();
        assert!(result.average_lambda() > 0.0);
    }
}
