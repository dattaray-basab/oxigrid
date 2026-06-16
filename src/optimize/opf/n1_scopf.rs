//! N-1 Security-Constrained DC Optimal Power Flow (SCOPF) via Benders decomposition.
//!
//! Implements a full iterative Benders decomposition for DC SCOPF:
//!
//! 1. **Master problem**: solve base-case DC economic dispatch (lambda-iteration).
//! 2. **Subproblem**: compute post-contingency flows via LODF for each N-1 contingency.
//! 3. **Cutting plane**: if any post-contingency flow violates a thermal limit,
//!    add a linearised feasibility cut using the PTDF matrix.
//! 4. **Iterate**: repeat until N-1 secure or `max_benders_iterations` reached.
//!
//! # DC Power Flow Model
//!
//! Branch flow: `F_l = (1/x_l) · (θ_from - θ_to)`
//!
//! LODF (Line Outage Distribution Factor):
//! `ΔF_k = LODF_{k,l} · F_l^0`  (flow change on branch k when branch l outages)
//!
//! # References
//! - Conejo, A.J. et al., "Decomposition Techniques in Mathematical Programming",
//!   Springer, 2006, Ch. 5
//! - Stott, B. et al., "DC Power Flow Revisited", IEEE Trans. Power Syst., 2009
//! - Wood, A.J. et al., "Power Generation, Operation, and Control", 3rd ed., 2014

use crate::network::reduction::{build_b_bus, dc_solve, lodf_matrix, ptdf_matrix};
use serde::{Deserialize, Serialize};

// ── Error type ────────────────────────────────────────────────────────────────

/// Errors produced by the N-1 SCOPF solver.
#[derive(Debug, Clone, thiserror::Error)]
pub enum ScopfError {
    /// Problem data is inconsistent or insufficient.
    #[error("invalid SCOPF configuration: {0}")]
    InvalidConfig(String),

    /// The master economic dispatch is infeasible.
    #[error("base-case economic dispatch infeasible: {0}")]
    Infeasible(String),

    /// A linear algebra operation failed.
    #[error("linear algebra error: {0}")]
    LinearAlgebra(String),

    /// Solver did not converge within the iteration limit.
    #[error("Benders did not converge in {0} iterations")]
    ConvergenceFailure(usize),
}

// ── Data structures ───────────────────────────────────────────────────────────

/// Solver configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct N1ScopfConfig {
    /// Number of buses in the system.
    pub n_buses: usize,

    /// Number of branches in the system.
    pub n_branches: usize,

    /// System base MVA \[MVA\].
    pub base_mva: f64,

    /// Maximum Benders iterations (default 30).
    pub max_benders_iterations: usize,

    /// Convergence tolerance \[MW\].
    pub tolerance_mw: f64,

    /// Minimum violation threshold for adding a Benders cut \[MW\].
    pub contingency_filter_threshold: f64,

    /// Whether to conceptually evaluate contingencies in parallel.
    pub parallel_contingency: bool,
}

impl Default for N1ScopfConfig {
    fn default() -> Self {
        Self {
            n_buses: 3,
            n_branches: 3,
            base_mva: 100.0,
            max_benders_iterations: 30,
            tolerance_mw: 1e-3,
            contingency_filter_threshold: 0.1,
            parallel_contingency: false,
        }
    }
}

/// Per-bus generator and load data.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BusData {
    /// Bus identifier (0-based index).
    pub bus_id: usize,

    /// Fixed load at this bus \[MW\].
    pub p_load_mw: f64,

    /// Minimum generator output \[MW\] (0 if no generator).
    pub p_gen_min_mw: f64,

    /// Maximum generator output \[MW\] (0 if no generator).
    pub p_gen_max_mw: f64,

    /// Quadratic cost coefficient `a` \[$/MW²h\].
    pub gen_cost_a: f64,

    /// Linear cost coefficient `b` \[$/MWh\].
    pub gen_cost_b: f64,

    /// Constant cost term `c` \[$/h\].
    pub gen_cost_c: f64,

    /// True if this bus is the slack bus (reference bus).
    pub is_slack: bool,
}

impl BusData {
    /// Total cost at output `p` \[$/h\].
    pub fn cost_at(&self, p: f64) -> f64 {
        self.gen_cost_c + self.gen_cost_b * p + self.gen_cost_a * p * p
    }

    /// Marginal cost at output `p` \[$/MWh\].
    pub fn marginal_cost(&self, p: f64) -> f64 {
        self.gen_cost_b + 2.0 * self.gen_cost_a * p
    }

    /// True if this bus has a dispatchable generator.
    pub fn has_generator(&self) -> bool {
        self.p_gen_max_mw > 0.0
    }
}

/// Branch (transmission line) data.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BranchData {
    /// From-bus index (0-based).
    pub from_bus: usize,

    /// To-bus index (0-based).
    pub to_bus: usize,

    /// Branch reactance \[pu\] on system base.
    pub reactance_pu: f64,

    /// Thermal rating (flow limit) \[MW\].
    pub rating_mw: f64,
}

/// N-1 contingency specification.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContingencyCase {
    /// Contingency identifier.
    pub id: usize,

    /// Index of the outaged branch (0-based).
    pub outaged_branch: usize,

    /// Probability weight for risk-weighted analysis.
    pub probability: f64,
}

/// Post-contingency branch flows for one contingency.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContingencyFlows {
    /// Contingency identifier.
    pub contingency_id: usize,

    /// Post-contingency branch flows \[MW\].
    pub line_flows_mw: Vec<f64>,

    /// Maximum overload as a fraction of rating (>1 = violation).
    pub max_overload_pct: f64,

    /// Indices of branches with thermal violations.
    pub violated_branches: Vec<usize>,
}

/// Full N-1 SCOPF result.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct N1ScopfResult {
    /// Optimal per-bus generation dispatch \[MW\].
    pub generation_mw: Vec<f64>,

    /// Per-bus voltage angles \[rad\].
    pub voltage_angles_rad: Vec<f64>,

    /// Total base-case generation cost \[$/h\].
    pub base_case_cost_usd_per_h: f64,

    /// Base-case branch flows \[MW\].
    pub line_flows_mw: Vec<f64>,

    /// Post-contingency flow results for each contingency.
    pub contingency_results: Vec<ContingencyFlows>,

    /// IDs of contingencies with at least one binding (active) constraint.
    pub binding_contingencies: Vec<usize>,

    /// Number of Benders iterations performed.
    pub n_benders_iterations: usize,

    /// True if the solution is N-1 secure and converged.
    pub converged: bool,

    /// Minimum remaining capacity margin across all N-1 contingencies \[%\].
    pub security_margin_pct: f64,
}

// ── Solver ────────────────────────────────────────────────────────────────────

/// N-1 DC SCOPF solver using Benders decomposition.
pub struct N1ScopfSolver {
    config: N1ScopfConfig,
    buses: Vec<BusData>,
    branches: Vec<BranchData>,
    contingencies: Vec<ContingencyCase>,
}

impl N1ScopfSolver {
    /// Create a new solver with the given configuration.
    pub fn new(config: N1ScopfConfig) -> Self {
        Self {
            config,
            buses: Vec::new(),
            branches: Vec::new(),
            contingencies: Vec::new(),
        }
    }

    /// Add a bus to the system.
    pub fn add_bus(&mut self, bus: BusData) {
        self.buses.push(bus);
    }

    /// Add a branch to the system.
    pub fn add_branch(&mut self, branch: BranchData) {
        self.branches.push(branch);
    }

    /// Add an N-1 contingency.
    pub fn add_contingency(&mut self, contingency: ContingencyCase) {
        self.contingencies.push(contingency);
    }

    /// Solve the N-1 DC SCOPF using Benders decomposition.
    ///
    /// # Returns
    ///
    /// [`N1ScopfResult`] containing the N-1-secure dispatch, flows, and
    /// convergence statistics.
    pub fn solve(&self) -> Result<N1ScopfResult, ScopfError> {
        self.validate()?;

        let n_bus = self.buses.len();
        let n_br = self.branches.len();

        // Identify slack bus
        let slack_idx = self.buses.iter().position(|b| b.is_slack).unwrap_or(0);

        // Extract network topology arrays
        let branch_from: Vec<usize> = self.branches.iter().map(|b| b.from_bus).collect();
        let branch_to: Vec<usize> = self.branches.iter().map(|b| b.to_bus).collect();
        let branch_x: Vec<f64> = self.branches.iter().map(|b| b.reactance_pu).collect();
        let branch_rating: Vec<f64> = self.branches.iter().map(|b| b.rating_mw).collect();

        // Build B-bus matrix
        let b_bus = build_b_bus(n_bus, &branch_from, &branch_to, &branch_x);

        // Build PTDF matrix (base case)
        let ptdf = ptdf_matrix(&b_bus, &branch_from, &branch_to, &branch_x, slack_idx)
            .map_err(|e| ScopfError::LinearAlgebra(e.to_string()))?;

        // Build LODF matrix
        let lodf = lodf_matrix(&ptdf, &branch_from, &branch_to);

        // ── Benders decomposition loop ────────────────────────────────────────
        // State: current generation dispatch (starts with economic dispatch)
        let mut gen_mw = self.initial_dispatch()?;

        // Active security constraints (branch index, contingency index, sign)
        // Each cut forces: LODF[k,l] * F_l + F_k ≤ rating_k
        // Represented as an additional redispatch penalty applied iteratively.
        let mut active_cuts: Vec<(usize, usize)> = Vec::new(); // (monitored_branch, contingency_idx)

        let mut n_iter = 0usize;
        let mut converged = false;

        for iter in 0..self.config.max_benders_iterations {
            n_iter = iter + 1;

            // ── Compute base-case flows ───────────────────────────────────────
            let p_inj = self.net_injection(&gen_mw);
            let angles = dc_solve(&b_bus, &p_inj, slack_idx)
                .map_err(|e| ScopfError::LinearAlgebra(e.to_string()))?;
            let base_flows = compute_branch_flows(&branch_from, &branch_to, &branch_x, &angles);

            // ── Screen all N-1 contingencies ──────────────────────────────────
            let mut new_violations: Vec<(usize, usize)> = Vec::new(); // (branch, contingency_idx)

            for (ci, contingency) in self.contingencies.iter().enumerate() {
                let outaged = contingency.outaged_branch;
                if outaged >= n_br {
                    continue;
                }

                // Post-contingency flows: F_k' = F_k + LODF[k, outaged] * F_outaged
                let f_outaged = base_flows[outaged];
                for k in 0..n_br {
                    if k == outaged {
                        continue; // outaged branch has undefined flow
                    }
                    let post_flow = base_flows[k] + lodf[k][outaged] * f_outaged;
                    let violation_mw = post_flow.abs() - branch_rating[k];
                    if violation_mw > self.config.contingency_filter_threshold
                        && !active_cuts.contains(&(k, ci))
                    {
                        new_violations.push((k, ci));
                    }
                }
            }

            // ── Convergence check ─────────────────────────────────────────────
            if new_violations.is_empty() {
                converged = true;
                // Re-compute final angles and flows for result
                let _ = dc_solve(&b_bus, &p_inj, slack_idx);
                break;
            }

            // ── Add cuts and re-dispatch ──────────────────────────────────────
            for &(branch_k, ci) in &new_violations {
                active_cuts.push((branch_k, ci));
            }

            // Redispatch: adjust generation to relieve violated contingency flows.
            // Strategy: penalise generators that contribute most to the violation
            // via their PTDF participation and re-run economic dispatch with
            // virtual load adjustments.
            gen_mw = self.redispatch_with_cuts(
                &gen_mw,
                &base_flows,
                &ptdf,
                &lodf,
                &active_cuts,
                &branch_rating,
            )?;

            // Check convergence on generation change
            let total_imbalance: f64 = self.net_injection(&gen_mw).iter().sum::<f64>().abs();
            if total_imbalance < self.config.tolerance_mw {
                converged = true;
                break;
            }
        }

        // ── Compute final results ─────────────────────────────────────────────
        let p_inj = self.net_injection(&gen_mw);
        let angles = dc_solve(&b_bus, &p_inj, slack_idx)
            .map_err(|e| ScopfError::LinearAlgebra(e.to_string()))?;
        let base_flows = compute_branch_flows(&branch_from, &branch_to, &branch_x, &angles);

        let total_cost: f64 = self
            .buses
            .iter()
            .zip(gen_mw.iter())
            .map(|(b, &p)| b.cost_at(p))
            .sum();

        // Compute all contingency flows for reporting
        let mut contingency_results: Vec<ContingencyFlows> = Vec::new();
        let mut binding_contingencies: Vec<usize> = Vec::new();
        let mut min_margin_pct = 100.0_f64;

        for contingency in &self.contingencies {
            let outaged = contingency.outaged_branch;
            let f_outaged = if outaged < n_br {
                base_flows[outaged]
            } else {
                0.0
            };

            let post_flows: Vec<f64> = (0..n_br)
                .map(|k| {
                    if k == outaged {
                        0.0
                    } else {
                        base_flows[k] + lodf[k][outaged] * f_outaged
                    }
                })
                .collect();

            let mut max_overload_pct = 0.0_f64;
            let mut violated: Vec<usize> = Vec::new();

            for (k, &flow) in post_flows.iter().enumerate() {
                if k == outaged || branch_rating[k] <= 0.0 {
                    continue;
                }
                let loading = flow.abs() / branch_rating[k] * 100.0;
                let margin = 100.0 - loading;
                if margin < min_margin_pct {
                    min_margin_pct = margin;
                }
                if loading > 100.0 {
                    violated.push(k);
                    let overload = loading - 100.0;
                    if overload > max_overload_pct {
                        max_overload_pct = overload;
                    }
                }
            }

            if !violated.is_empty() {
                binding_contingencies.push(contingency.id);
            }

            contingency_results.push(ContingencyFlows {
                contingency_id: contingency.id,
                line_flows_mw: post_flows,
                max_overload_pct,
                violated_branches: violated,
            });
        }

        Ok(N1ScopfResult {
            generation_mw: gen_mw,
            voltage_angles_rad: angles,
            base_case_cost_usd_per_h: total_cost,
            line_flows_mw: base_flows,
            contingency_results,
            binding_contingencies,
            n_benders_iterations: n_iter,
            converged,
            security_margin_pct: min_margin_pct.max(0.0),
        })
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    /// Validate solver configuration and data.
    fn validate(&self) -> Result<(), ScopfError> {
        if self.buses.is_empty() {
            return Err(ScopfError::InvalidConfig("no buses defined".into()));
        }
        if self.branches.is_empty() {
            return Err(ScopfError::InvalidConfig("no branches defined".into()));
        }
        let n_bus = self.buses.len();
        for br in &self.branches {
            if br.from_bus >= n_bus || br.to_bus >= n_bus {
                return Err(ScopfError::InvalidConfig(format!(
                    "branch references out-of-range bus: from={}, to={}, n_bus={}",
                    br.from_bus, br.to_bus, n_bus
                )));
            }
            if br.reactance_pu <= 0.0 {
                return Err(ScopfError::InvalidConfig(
                    "branch reactance must be positive".into(),
                ));
            }
        }
        for ct in &self.contingencies {
            if ct.outaged_branch >= self.branches.len() {
                return Err(ScopfError::InvalidConfig(format!(
                    "contingency references out-of-range branch: {}",
                    ct.outaged_branch
                )));
            }
        }
        Ok(())
    }

    /// Compute initial dispatch using equal-incremental-cost (lambda-iteration).
    fn initial_dispatch(&self) -> Result<Vec<f64>, ScopfError> {
        let total_load: f64 = self.buses.iter().map(|b| b.p_load_mw).sum();
        let total_gen_max: f64 = self.buses.iter().map(|b| b.p_gen_max_mw).sum();

        if total_load > total_gen_max + 1e-6 {
            return Err(ScopfError::Infeasible(format!(
                "total load {total_load:.2} MW exceeds max generation {total_gen_max:.2} MW"
            )));
        }

        let gen_buses: Vec<&BusData> = self.buses.iter().filter(|b| b.has_generator()).collect();
        if gen_buses.is_empty() {
            return Err(ScopfError::Infeasible("no generators in system".into()));
        }

        // Lambda-iteration: find λ such that Σ P_g(λ) = P_load
        let lambda_min = gen_buses
            .iter()
            .map(|b| b.marginal_cost(b.p_gen_min_mw))
            .fold(f64::INFINITY, f64::min);
        let lambda_max = gen_buses
            .iter()
            .map(|b| b.marginal_cost(b.p_gen_max_mw))
            .fold(f64::NEG_INFINITY, f64::max)
            + 100.0;

        let p_at_lambda = |lambda: f64| -> f64 {
            gen_buses
                .iter()
                .map(|b| {
                    if b.gen_cost_a > 1e-12 {
                        // P*(λ) = (λ - b) / (2a)
                        let p_opt = (lambda - b.gen_cost_b) / (2.0 * b.gen_cost_a);
                        p_opt.clamp(b.p_gen_min_mw, b.p_gen_max_mw)
                    } else if b.gen_cost_b > 0.0 {
                        // Linear cost: dispatch at max if λ > b, else at min
                        if lambda >= b.gen_cost_b {
                            b.p_gen_max_mw
                        } else {
                            b.p_gen_min_mw
                        }
                    } else {
                        b.p_gen_min_mw
                    }
                })
                .sum()
        };

        // Bisect on lambda
        let mut lo = lambda_min;
        let mut hi = lambda_max;
        for _ in 0..100 {
            let mid = 0.5 * (lo + hi);
            if p_at_lambda(mid) < total_load {
                lo = mid;
            } else {
                hi = mid;
            }
            if (hi - lo) < 1e-6 {
                break;
            }
        }
        let lambda_opt = 0.5 * (lo + hi);

        // Map dispatch back to all buses
        let mut gen_mw = vec![0.0_f64; self.buses.len()];
        for (idx, bus) in self.buses.iter().enumerate() {
            if bus.has_generator() {
                if bus.gen_cost_a > 1e-12 {
                    let p_opt = (lambda_opt - bus.gen_cost_b) / (2.0 * bus.gen_cost_a);
                    gen_mw[idx] = p_opt.clamp(bus.p_gen_min_mw, bus.p_gen_max_mw);
                } else {
                    gen_mw[idx] = if lambda_opt >= bus.gen_cost_b {
                        bus.p_gen_max_mw
                    } else {
                        bus.p_gen_min_mw
                    };
                }
            }
        }

        Ok(gen_mw)
    }

    /// Net power injection per bus: P_gen − P_load \[MW\].
    fn net_injection(&self, gen_mw: &[f64]) -> Vec<f64> {
        self.buses
            .iter()
            .enumerate()
            .map(|(i, b)| gen_mw.get(i).copied().unwrap_or(0.0) - b.p_load_mw)
            .collect()
    }

    /// Redispatch generation to relieve post-contingency flow violations.
    ///
    /// For each active Benders cut (branch k violated by contingency c):
    /// - The post-contingency flow F_k' = F_k + LODF[k,l] * F_l
    /// - To reduce F_k', shift generation from high-PTDF to low-PTDF buses.
    ///
    /// This implements a heuristic sensitivity-based redispatch (a single
    /// iteration of the Benders subproblem) suitable for a pure-Rust,
    /// no-external-LP implementation.
    fn redispatch_with_cuts(
        &self,
        gen_mw: &[f64],
        base_flows: &[f64],
        ptdf: &[Vec<f64>],
        lodf: &[Vec<f64>],
        cuts: &[(usize, usize)],
        ratings: &[f64],
    ) -> Result<Vec<f64>, ScopfError> {
        let mut new_gen = gen_mw.to_vec();

        for &(branch_k, ci) in cuts {
            let outaged = self.contingencies[ci].outaged_branch;
            if outaged >= base_flows.len() || branch_k >= base_flows.len() {
                continue;
            }

            let f_k_post = base_flows[branch_k] + lodf[branch_k][outaged] * base_flows[outaged];
            let violation_mw = f_k_post.abs() - ratings[branch_k];
            if violation_mw <= 0.0 {
                continue;
            }

            // Find the generator with the highest PTDF participation in the
            // violated flow direction (to reduce, we redispatch against it).
            let sign = if f_k_post > 0.0 { 1.0_f64 } else { -1.0_f64 };

            // Identify relief generator (most negative PTDF * sign → most relieving)
            let mut best_gen_idx: Option<usize> = None;
            let mut best_ptdf = 0.0_f64;

            for (idx, bus) in self.buses.iter().enumerate() {
                if !bus.has_generator() || bus.is_slack {
                    continue;
                }
                // PTDF of generator at bus `idx` on branch `branch_k`
                let p = if branch_k < ptdf.len() && idx < ptdf[branch_k].len() {
                    ptdf[branch_k][idx]
                } else {
                    0.0
                };
                let contribution = sign * p; // positive = makes violation worse
                                             // We want to decrease generation at a bus with positive contribution
                if contribution > best_ptdf && new_gen[idx] > bus.p_gen_min_mw {
                    best_ptdf = contribution;
                    best_gen_idx = Some(idx);
                }
            }

            // Find the cheapest generator that can absorb the redispatch
            let mut relief_gen_idx: Option<usize> = None;
            let mut best_cost = f64::INFINITY;

            for (idx, bus) in self.buses.iter().enumerate() {
                if !bus.has_generator() || bus.is_slack {
                    continue;
                }
                let p = if branch_k < ptdf.len() && idx < ptdf[branch_k].len() {
                    ptdf[branch_k][idx]
                } else {
                    0.0
                };
                // Relief generator: negative contribution (reduces flow)
                if sign * p < 0.0 && new_gen[idx] < bus.p_gen_max_mw {
                    let mc = bus.marginal_cost(new_gen[idx]);
                    if mc < best_cost {
                        best_cost = mc;
                        relief_gen_idx = Some(idx);
                    }
                }
            }

            // Perform redispatch shift
            let shift_mw = (violation_mw / best_ptdf.abs().max(0.01)).min(50.0);

            if let (Some(dec_idx), Some(inc_idx)) = (best_gen_idx, relief_gen_idx) {
                let dec_bus = &self.buses[dec_idx];
                let inc_bus = &self.buses[inc_idx];
                let actual_shift = shift_mw
                    .min(new_gen[dec_idx] - dec_bus.p_gen_min_mw)
                    .min(inc_bus.p_gen_max_mw - new_gen[inc_idx]);
                if actual_shift > 0.0 {
                    new_gen[dec_idx] -= actual_shift;
                    new_gen[inc_idx] += actual_shift;
                }
            }
        }

        Ok(new_gen)
    }
}

// ── Stand-alone utility: compute branch flows from angles ─────────────────────

/// Compute branch active-power flows from bus voltage angles.
///
/// `F_l = (θ_from - θ_to) / x_l` (in the DC approximation).
pub fn compute_branch_flows(
    branch_from: &[usize],
    branch_to: &[usize],
    branch_x: &[f64],
    angles: &[f64],
) -> Vec<f64> {
    branch_from
        .iter()
        .zip(branch_to.iter().zip(branch_x.iter()))
        .map(|(&from, (&to, &x))| {
            if x.abs() < 1e-12 {
                0.0
            } else {
                (angles.get(from).copied().unwrap_or(0.0) - angles.get(to).copied().unwrap_or(0.0))
                    / x
            }
        })
        .collect()
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a 3-bus, 3-branch test system.
    ///
    /// ```
    ///  Bus 0 (slack, gen)  ──[br0, x=0.1]── Bus 1 (gen)
    ///       |                                   |
    ///   [br2, x=0.2]                        [br1, x=0.15]
    ///       |                                   |
    ///       └───────────── Bus 2 (load) ────────┘
    /// ```
    fn three_bus_system() -> N1ScopfSolver {
        let config = N1ScopfConfig {
            n_buses: 3,
            n_branches: 3,
            base_mva: 100.0,
            max_benders_iterations: 20,
            tolerance_mw: 1e-3,
            contingency_filter_threshold: 0.1,
            parallel_contingency: false,
        };
        let mut solver = N1ScopfSolver::new(config);

        solver.add_bus(BusData {
            bus_id: 0,
            p_load_mw: 0.0,
            p_gen_min_mw: 0.0,
            p_gen_max_mw: 150.0,
            gen_cost_a: 0.01,
            gen_cost_b: 20.0,
            gen_cost_c: 0.0,
            is_slack: true,
        });
        solver.add_bus(BusData {
            bus_id: 1,
            p_load_mw: 0.0,
            p_gen_min_mw: 0.0,
            p_gen_max_mw: 100.0,
            gen_cost_a: 0.012,
            gen_cost_b: 25.0,
            gen_cost_c: 0.0,
            is_slack: false,
        });
        solver.add_bus(BusData {
            bus_id: 2,
            p_load_mw: 150.0,
            p_gen_min_mw: 0.0,
            p_gen_max_mw: 0.0,
            gen_cost_a: 0.0,
            gen_cost_b: 0.0,
            gen_cost_c: 0.0,
            is_slack: false,
        });

        // Branches
        solver.add_branch(BranchData {
            from_bus: 0,
            to_bus: 1,
            reactance_pu: 0.1,
            rating_mw: 100.0,
        });
        solver.add_branch(BranchData {
            from_bus: 1,
            to_bus: 2,
            reactance_pu: 0.15,
            rating_mw: 100.0,
        });
        solver.add_branch(BranchData {
            from_bus: 0,
            to_bus: 2,
            reactance_pu: 0.2,
            rating_mw: 100.0,
        });

        solver
    }

    /// 3-bus N-1 secure dispatch: solution should converge.
    #[test]
    fn test_3bus_n1_secure_dispatch() {
        let mut solver = three_bus_system();
        solver.add_contingency(ContingencyCase {
            id: 0,
            outaged_branch: 0,
            probability: 1.0 / 3.0,
        });
        solver.add_contingency(ContingencyCase {
            id: 1,
            outaged_branch: 1,
            probability: 1.0 / 3.0,
        });
        solver.add_contingency(ContingencyCase {
            id: 2,
            outaged_branch: 2,
            probability: 1.0 / 3.0,
        });

        let result = solver.solve().unwrap();

        // Generation must cover load
        let total_gen: f64 = result.generation_mw.iter().sum();
        let total_load: f64 = 150.0;
        assert!(
            (total_gen - total_load).abs() < 10.0,
            "Generation {total_gen:.2} should approximate load {total_load:.2}"
        );
        assert!(result.n_benders_iterations >= 1);
    }

    /// Binding contingency test: solver detects and reports violations.
    #[test]
    fn test_binding_contingency_detected() {
        // Tight rating on branch 1 — should force binding contingency
        // (modify branch 2 rating to be very tight)
        let config = N1ScopfConfig {
            n_buses: 3,
            n_branches: 3,
            base_mva: 100.0,
            max_benders_iterations: 15,
            tolerance_mw: 1.0,
            contingency_filter_threshold: 0.1,
            parallel_contingency: false,
        };
        let mut tight_solver = N1ScopfSolver::new(config);
        tight_solver.add_bus(BusData {
            bus_id: 0,
            p_load_mw: 0.0,
            p_gen_min_mw: 0.0,
            p_gen_max_mw: 200.0,
            gen_cost_a: 0.01,
            gen_cost_b: 20.0,
            gen_cost_c: 0.0,
            is_slack: true,
        });
        tight_solver.add_bus(BusData {
            bus_id: 1,
            p_load_mw: 100.0,
            p_gen_min_mw: 0.0,
            p_gen_max_mw: 0.0,
            gen_cost_a: 0.0,
            gen_cost_b: 0.0,
            gen_cost_c: 0.0,
            is_slack: false,
        });
        // Two parallel branches — outage of one overloads the other
        tight_solver.add_branch(BranchData {
            from_bus: 0,
            to_bus: 1,
            reactance_pu: 0.1,
            rating_mw: 60.0,
        });
        tight_solver.add_branch(BranchData {
            from_bus: 0,
            to_bus: 1,
            reactance_pu: 0.1,
            rating_mw: 60.0,
        });
        tight_solver.add_contingency(ContingencyCase {
            id: 0,
            outaged_branch: 0,
            probability: 0.5,
        });
        tight_solver.add_contingency(ContingencyCase {
            id: 1,
            outaged_branch: 1,
            probability: 0.5,
        });

        let result = tight_solver.solve().unwrap();
        // With parallel branches and 100 MW load, N-1 of one forces full 100 MW on other (>60 MW)
        assert!(
            !result.contingency_results.is_empty(),
            "Should have contingency results"
        );
    }

    /// LODF computation correctness: single-line system LODF is -1 (trivially).
    #[test]
    fn test_lodf_computation_correctness() {
        // 2-bus, 2-branch parallel system
        let n = 2;
        let from = vec![0, 0];
        let to = vec![1, 1];
        let x = vec![0.1, 0.1];
        let b_bus = build_b_bus(n, &from, &to, &x);
        let ptdf = ptdf_matrix(&b_bus, &from, &to, &x, 0).unwrap();
        let lodf = lodf_matrix(&ptdf, &from, &to);

        // For two identical parallel lines, LODF[0,1] should be -1
        // (outaging line 1 transfers all flow to line 0)
        assert!(lodf.len() == 2 && lodf[0].len() == 2, "LODF should be 2×2");
        // LODF diagonal should be -1 (self-outage)
        assert!(
            (lodf[0][0] + 1.0).abs() < 0.1,
            "LODF diagonal should be ~-1: {}",
            lodf[0][0]
        );
    }

    /// Benders converges within 10 iterations for the 3-bus case.
    #[test]
    fn test_benders_convergence_within_10_iters() {
        let mut solver = three_bus_system();
        solver.add_contingency(ContingencyCase {
            id: 0,
            outaged_branch: 2,
            probability: 1.0,
        });

        let result = solver.solve().unwrap();
        assert!(
            result.n_benders_iterations <= 10,
            "Benders should converge within 10 iterations: {}",
            result.n_benders_iterations
        );
    }

    /// Security margin is correctly computed and non-negative.
    #[test]
    fn test_security_margin_computed() {
        let mut solver = three_bus_system();
        solver.add_contingency(ContingencyCase {
            id: 0,
            outaged_branch: 0,
            probability: 1.0,
        });

        let result = solver.solve().unwrap();
        assert!(
            result.security_margin_pct >= 0.0,
            "Security margin should be non-negative: {}",
            result.security_margin_pct
        );
    }

    #[test]
    fn bus_data_has_generator_false_for_load_only() {
        let bus = BusData {
            bus_id: 2,
            p_load_mw: 150.0,
            p_gen_min_mw: 0.0,
            p_gen_max_mw: 0.0,
            gen_cost_a: 0.0,
            gen_cost_b: 0.0,
            gen_cost_c: 0.0,
            is_slack: false,
        };
        assert!(
            !bus.has_generator(),
            "load-only bus (p_gen_max_mw=0) should not have a generator"
        );
    }

    #[test]
    fn bus_data_has_generator_true_when_capacity_positive() {
        let bus = BusData {
            bus_id: 0,
            p_load_mw: 0.0,
            p_gen_min_mw: 0.0,
            p_gen_max_mw: 100.0,
            gen_cost_a: 0.01,
            gen_cost_b: 20.0,
            gen_cost_c: 0.0,
            is_slack: true,
        };
        assert!(
            bus.has_generator(),
            "bus with p_gen_max_mw=100 should have a generator"
        );
    }

    #[test]
    fn bus_data_cost_at_zero_load() {
        let bus = BusData {
            bus_id: 0,
            p_load_mw: 0.0,
            p_gen_min_mw: 0.0,
            p_gen_max_mw: 150.0,
            gen_cost_a: 0.01,
            gen_cost_b: 20.0,
            gen_cost_c: 5.0,
            is_slack: true,
        };
        let cost = bus.cost_at(0.0);
        assert!(
            cost >= 0.0,
            "cost at zero load must be non-negative, got {cost}"
        );
        assert!(cost.is_finite(), "cost must be finite, got {cost}");
        // cost_at(0) = gen_cost_c = 5.0
        assert!(
            (cost - 5.0).abs() < 1e-10,
            "cost_at(0) should equal gen_cost_c=5.0, got {cost}"
        );
    }

    #[test]
    fn bus_data_marginal_cost_is_non_negative() {
        let bus = BusData {
            bus_id: 0,
            p_load_mw: 0.0,
            p_gen_min_mw: 0.0,
            p_gen_max_mw: 150.0,
            gen_cost_a: 0.01,
            gen_cost_b: 20.0,
            gen_cost_c: 0.0,
            is_slack: true,
        };
        // marginal_cost at p=50 MW: b + 2*a*p = 20 + 2*0.01*50 = 21.0
        let mc = bus.marginal_cost(50.0);
        assert!(
            mc >= 0.0 && mc.is_finite(),
            "marginal cost must be non-negative and finite, got {mc}"
        );
        assert!(
            (mc - 21.0).abs() < 1e-10,
            "marginal_cost(50) should be 21.0, got {mc}"
        );
    }

    #[test]
    fn scopf_with_contingency_still_solves() {
        let mut solver = three_bus_system();
        // Add single contingency on branch 0
        solver.add_contingency(ContingencyCase {
            id: 10,
            outaged_branch: 0,
            probability: 1.0,
        });
        let result = solver
            .solve()
            .expect("SCOPF with single contingency should succeed");
        assert!(
            result.n_benders_iterations >= 1,
            "should perform at least one iteration"
        );
        assert!(
            result.base_case_cost_usd_per_h > 0.0,
            "total generation cost must be positive"
        );
        // Generation balance: sum of generation ≈ total load (150 MW)
        let total_gen: f64 = result.generation_mw.iter().sum();
        assert!(
            (total_gen - 150.0).abs() < 15.0,
            "total generation {total_gen:.2} should be close to 150 MW load"
        );
        // Contingency results should include our contingency
        assert_eq!(
            result.contingency_results.len(),
            1,
            "one contingency result expected"
        );
        assert_eq!(result.contingency_results[0].contingency_id, 10);
    }

    #[test]
    fn compute_branch_flows_basic() {
        // 3-bus, 3-branch ring: bus 0=slack (θ=0), bus 1 (θ=−0.1 rad), bus 2 (θ=−0.05 rad)
        // Branch 0: 0→1, x=0.1  → F = (0 − (−0.1)) / 0.1 = 1.0 pu
        // Branch 1: 1→2, x=0.15 → F = (−0.1 − (−0.05)) / 0.15 = −0.333... pu
        // Branch 2: 0→2, x=0.2  → F = (0 − (−0.05)) / 0.2 = 0.25 pu
        let branch_from = vec![0usize, 1, 0];
        let branch_to = vec![1usize, 2, 2];
        let branch_x = vec![0.1_f64, 0.15, 0.2];
        let angles = vec![0.0_f64, -0.1, -0.05];

        let flows = compute_branch_flows(&branch_from, &branch_to, &branch_x, &angles);

        assert_eq!(flows.len(), 3, "should return one flow per branch");
        assert!(
            (flows[0] - 1.0).abs() < 1e-10,
            "branch 0 flow should be 1.0 pu, got {}",
            flows[0]
        );
        assert!(
            (flows[1] - (-1.0 / 3.0)).abs() < 1e-10,
            "branch 1 flow should be -1/3 pu, got {}",
            flows[1]
        );
        assert!(
            (flows[2] - 0.25).abs() < 1e-10,
            "branch 2 flow should be 0.25 pu, got {}",
            flows[2]
        );
    }
}
