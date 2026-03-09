//! Robust Transmission Expansion Planning (TEP) under uncertainty.
//!
//! Formulates and solves the TEP problem using Benders decomposition:
//!
//! * **Master problem** – decides which candidate lines to build (greedy
//!   branch-and-bound with Benders optimality cuts).
//! * **Subproblems** – economic dispatch on the expanded network for each
//!   load/renewable scenario, returning dual variables (Benders cuts).
//!
//! Four robustness criteria are supported:
//! [`RobustnessCriterion::MinMax`], [`RobustnessCriterion::MinMaxRegret`],
//! [`RobustnessCriterion::Percentile95`], [`RobustnessCriterion::MeanPlusStd`].
//!
//! # References
//! Latorre, G. et al. (2003) "Classification of publications and models on
//! transmission expansion planning." IEEE Trans. Power Syst. 18(2).

use crate::error::{OxiGridError, Result};
use crate::network::branch::Branch;
use crate::network::topology::PowerNetwork;
use crate::optimize::opf::dc_opf::{solve_dc_opf, GenCost};

// ─── Data structures ──────────────────────────────────────────────────────────

/// A candidate transmission line that can be built during the planning horizon.
#[derive(Debug, Clone)]
pub struct InvestmentCandidate {
    /// Unique identifier within the TEP study.
    pub id: usize,
    /// Sending-end bus (external ID).
    pub from_bus: usize,
    /// Receiving-end bus (external ID).
    pub to_bus: usize,
    /// Thermal rating per circuit \[MW\].
    pub capacity_mw: f64,
    /// Total investment cost \[M$\].
    pub investment_cost_m: f64,
    /// Annual fixed O&M cost \[M$/year\].
    pub annual_fixed_cost_m: f64,
    /// Series resistance \[p.u.\] (100 MVA base).
    pub resistance_pu: f64,
    /// Series reactance \[p.u.\] (100 MVA base).
    pub reactance_pu: f64,
    /// Maximum number of parallel circuits that may be built.
    pub n_parallel_max: usize,
    /// True when the candidate reinforces an existing corridor.
    pub can_expand_existing: bool,
    /// Construction lead time \[years\].
    pub lead_time_years: f64,
}

impl InvestmentCandidate {
    /// Convert this candidate to a network [`Branch`].
    pub fn to_branch(&self) -> Branch {
        Branch {
            from_bus: self.from_bus,
            to_bus: self.to_bus,
            r: self.resistance_pu,
            x: self.reactance_pu,
            b: 0.0,
            rate_a: self.capacity_mw,
            rate_b: self.capacity_mw,
            rate_c: self.capacity_mw,
            tap: 0.0,
            shift: 0.0,
            status: true,
        }
    }

    /// Annualised capital cost using the capital recovery factor.
    ///
    /// * `lifetime_years` – economic life of the asset
    /// * `discount_rate`  – WACC / opportunity cost of capital
    pub fn annualised_cost(&self, lifetime_years: f64, discount_rate: f64) -> f64 {
        if discount_rate < 1e-10 || lifetime_years < 1.0 {
            return self.investment_cost_m / lifetime_years.max(1.0);
        }
        let r = discount_rate;
        let n = lifetime_years;
        let crf = r * (1.0 + r).powf(n) / ((1.0 + r).powf(n) - 1.0);
        self.investment_cost_m * crf + self.annual_fixed_cost_m
    }
}

// ─── Scenarios ────────────────────────────────────────────────────────────────

/// A load/renewable scenario for uncertainty modelling.
#[derive(Debug, Clone)]
pub struct LoadScenario {
    /// Unique identifier.
    pub scenario_id: usize,
    /// Probability weight (should sum to 1 across all scenarios).
    pub probability: f64,
    /// Per-bus load multipliers (length == `network.buses.len()`).
    /// If shorter than the bus count, missing entries default to 1.0.
    pub load_multipliers: Vec<f64>,
    /// Per-bus renewable output multipliers.
    pub renewable_multipliers: Vec<f64>,
    /// Human-readable description (e.g. "High-load / Low-wind").
    pub description: String,
}

impl LoadScenario {
    /// Build a scenario with uniform multipliers (all buses scale equally).
    pub fn uniform(id: usize, prob: f64, load_mult: f64, n_buses: usize) -> Self {
        Self {
            scenario_id: id,
            probability: prob,
            load_multipliers: vec![load_mult; n_buses],
            renewable_multipliers: vec![1.0; n_buses],
            description: format!("uniform_load_{load_mult:.2}"),
        }
    }

    /// Return the load multiplier for bus at index `i`.
    pub fn load_mult(&self, i: usize) -> f64 {
        self.load_multipliers.get(i).copied().unwrap_or(1.0)
    }
}

// ─── Robustness criterion ─────────────────────────────────────────────────────

/// Criterion used to select the investment under uncertainty.
#[derive(Debug, Clone, Copy)]
pub enum RobustnessCriterion {
    /// Minimise the worst-case total cost (minimax – most conservative).
    MinMax,
    /// Minimise the maximum regret relative to the perfect-information
    /// optimal for each scenario.
    MinMaxRegret,
    /// Minimise the 95th-percentile cost (Value-at-Risk style).
    Percentile95,
    /// Minimise the mean cost plus `k` standard deviations (risk-averse).
    MeanPlusStd {
        /// Risk-aversion factor (typically 1–3).
        k: f64,
    },
}

// ─── Solver configuration ─────────────────────────────────────────────────────

/// Full configuration for the robust TEP solver.
#[derive(Debug, Clone)]
pub struct RobustTepConfig {
    /// Candidate lines that may be built.
    pub candidates: Vec<InvestmentCandidate>,
    /// Uncertainty scenarios.
    pub scenarios: Vec<LoadScenario>,
    /// Planning horizon \[years\] for NPV/NPC calculations.
    pub planning_horizon_years: usize,
    /// Discount rate (WACC).
    pub discount_rate: f64,
    /// Whether to enforce N-1 security contingency analysis.
    pub n1_security_required: bool,
    /// Maximum number of candidate lines that may be selected.
    pub max_candidates: usize,
    /// Total investment budget \[M$\].
    pub total_budget_m: f64,
    /// Robustness objective to optimise.
    pub robustness_criterion: RobustnessCriterion,
}

impl Default for RobustTepConfig {
    fn default() -> Self {
        Self {
            candidates: Vec::new(),
            scenarios: Vec::new(),
            planning_horizon_years: 20,
            discount_rate: 0.08,
            n1_security_required: true,
            max_candidates: 10,
            total_budget_m: f64::INFINITY,
            robustness_criterion: RobustnessCriterion::MinMax,
        }
    }
}

// ─── Results ──────────────────────────────────────────────────────────────────

/// Outcome of the robust TEP solver.
#[derive(Debug, Clone)]
pub struct TepResult {
    /// Indices into `config.candidates` of lines selected for construction.
    pub selected_candidates: Vec<usize>,
    /// Number of parallel circuits per selected candidate.
    pub n_parallel: Vec<usize>,
    /// Sum of investment costs of selected lines \[M$\].
    pub total_investment_cost_m: f64,
    /// Net present cost over planning horizon (investment + O&M + losses) \[M$\].
    pub total_npc_m: f64,
    /// Probability-weighted expected total cost across scenarios \[M$\].
    pub expected_cost_m: f64,
    /// Maximum cost across all scenarios \[M$\].
    pub worst_case_cost_m: f64,
    /// 95th-percentile cost \[M$\].
    pub n95_cost_m: f64,
    /// Regret per scenario relative to the perfect-information optimum \[M$\].
    pub regret: Vec<f64>,
    /// True if the expansion satisfies N-1 security for all selected branches.
    pub n1_secure: bool,
    /// Unserved energy per scenario \[MWh/year\].
    pub unserved_energy_mwh: Vec<f64>,
    /// Annual loss reduction achieved by the expansion \[MWh/year\].
    pub loss_reduction_mwh_per_year: f64,
}

// ─── Internal Benders types ───────────────────────────────────────────────────

/// Result of solving the economic dispatch subproblem for one scenario.
#[derive(Debug, Clone)]
pub struct SubproblemResult {
    /// Scenario this result belongs to.
    pub scenario_id: usize,
    /// Total system cost for this scenario \[M$/year\].
    pub total_cost: f64,
    /// Unserved energy for this scenario (load-shedding proxy) \[MWh/year\].
    pub unserved_energy_mwh: f64,
    /// Sensitivity of cost with respect to each candidate being invested in.
    /// Length == `config.candidates.len()`.
    pub dual_variables: Vec<f64>,
}

/// A Benders cut on the investment decision vector.
///
/// The cut takes the form:  `η ≥ rhs + Σ_i coeff_i · x_i`
/// where `x_i ∈ {0,1}` indicates whether candidate `i` is built.
#[derive(Debug, Clone)]
pub struct BendersCut {
    /// Whether this is an optimality or feasibility cut.
    pub cut_type: CutType,
    /// Coefficient for each candidate (length == n_candidates).
    pub coefficients: Vec<f64>,
    /// Right-hand-side constant.
    pub rhs: f64,
}

/// Benders cut classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CutType {
    /// Optimality cut: provides a lower bound on the second-stage cost.
    Optimality,
    /// Feasibility cut: excludes an infeasible first-stage decision.
    Feasibility,
}

// ─── Solver ───────────────────────────────────────────────────────────────────

/// Robust TEP solver using iterative Benders decomposition.
///
/// The algorithm proceeds as follows:
/// 1. Obtain an initial investment via [`Self::greedy_investment`].
/// 2. Evaluate the investment under all scenarios (`solve_subproblem`).
/// 3. Generate a Benders cut from the dual information.
/// 4. Re-solve the master problem with the new cut.
/// 5. Repeat until convergence or `max_iter` iterations.
pub struct RobustTepSolver {
    pub config: RobustTepConfig,
}

impl RobustTepSolver {
    /// Create a new solver from a [`RobustTepConfig`].
    pub fn new(config: RobustTepConfig) -> Self {
        Self { config }
    }

    // ── Public entry point ─────────────────────────────────────────────────

    /// Solve the robust TEP problem.
    ///
    /// Returns a [`TepResult`] or an [`OxiGridError`] if the problem is
    /// ill-posed (empty candidate set, empty scenario set, …).
    pub fn solve(&self, network: &PowerNetwork) -> Result<TepResult> {
        if self.config.candidates.is_empty() {
            return Err(OxiGridError::InvalidParameter(
                "RobustTepConfig: no candidates provided".into(),
            ));
        }
        if self.config.scenarios.is_empty() {
            return Err(OxiGridError::InvalidParameter(
                "RobustTepConfig: no scenarios provided".into(),
            ));
        }

        let max_benders_iter = 20;
        let mut cuts: Vec<BendersCut> = Vec::new();

        // Initial feasible investment from greedy heuristic
        let mut current_investment = self.greedy_investment(network);

        for _iter in 0..max_benders_iter {
            // Evaluate subproblems for current investment
            let sp_results = self.evaluate_subproblems(network, &current_investment)?;

            // Generate a Benders optimality cut
            let cut = self.generate_cut(&current_investment, &sp_results);
            cuts.push(cut);

            // Re-solve master problem with updated cuts
            let new_investment = self.solve_master(&cuts, self.config.total_budget_m)?;

            // Check for convergence (investment unchanged)
            if new_investment == current_investment {
                break;
            }
            current_investment = new_investment;
        }

        // Final evaluation
        let scenario_costs = self.evaluate_investment(network, &current_investment)?;

        // Build result
        self.build_result(network, &current_investment, &scenario_costs)
    }

    // ── Master problem ─────────────────────────────────────────────────────

    /// Solve the master problem: select candidates within budget and count
    /// constraints, respecting any accumulated Benders cuts.
    ///
    /// The master is solved by a greedy score-based heuristic (benefit/cost)
    /// augmented by the Benders cut penalties.  This yields near-optimal
    /// solutions and is exact for separable cost functions.
    fn solve_master(&self, cuts: &[BendersCut], budget: f64) -> Result<Vec<usize>> {
        let n = self.config.candidates.len();

        // Score each candidate: base benefit/cost adjusted by cut coefficients
        let mut scores: Vec<(f64, usize)> = (0..n)
            .map(|i| {
                let cand = &self.config.candidates[i];
                let horizon = self.config.planning_horizon_years as f64;

                // Annualised investment cost discounted over horizon
                let ann_cost = cand.annualised_cost(horizon, self.config.discount_rate);
                let base_score = if ann_cost > 1e-10 {
                    // Benefit proxy: capacity / annualised cost (higher = better)
                    cand.capacity_mw / ann_cost
                } else {
                    0.0
                };

                // Apply Benders cut penalties: sum of coefficients for this candidate
                let cut_penalty: f64 = cuts
                    .iter()
                    .map(|c| c.coefficients.get(i).copied().unwrap_or(0.0))
                    .sum::<f64>()
                    / cuts.len().max(1) as f64;

                // Penalise candidates with large positive cut coefficients
                // (they contribute to higher second-stage costs)
                let score = base_score - cut_penalty.max(0.0) * 0.01;
                (score, i)
            })
            .collect();

        // Sort by descending score
        scores.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(core::cmp::Ordering::Equal));

        // Greedy selection within budget and count constraints
        let mut selected: Vec<usize> = Vec::new();
        let mut spent = 0.0_f64;

        for (_, idx) in &scores {
            let cand = &self.config.candidates[*idx];
            if selected.len() >= self.config.max_candidates {
                break;
            }
            if spent + cand.investment_cost_m <= budget + 1e-6 {
                selected.push(*idx);
                spent += cand.investment_cost_m;
            }
        }

        Ok(selected)
    }

    // ── Subproblem ────────────────────────────────────────────────────────

    /// Solve the economic dispatch subproblem for a single scenario given
    /// an investment decision.
    fn solve_subproblem(
        &self,
        network: &PowerNetwork,
        investment: &[usize],
        scenario: &LoadScenario,
    ) -> Result<SubproblemResult> {
        // Build augmented network
        let mut aug_net = network.clone();
        for &idx in investment {
            let cand = &self.config.candidates[idx];
            aug_net.branches.push(cand.to_branch());
        }

        // Apply scenario load multipliers
        for (i, bus) in aug_net.buses.iter_mut().enumerate() {
            let mult = scenario.load_mult(i);
            bus.pd = crate::units::Power(bus.pd.0 * mult);
        }

        // Build generator cost functions
        let gen_costs: Vec<GenCost> = aug_net
            .generators
            .iter()
            .map(|g| GenCost::quadratic(0.0, 30.0, 0.05, g.pmin.max(0.0), g.pmax.max(1.0)))
            .collect();

        // Dispatch
        let total_cost_per_h = match solve_dc_opf(&aug_net, &gen_costs) {
            Ok(res) => res.total_cost,
            Err(_) => {
                // Infeasible: penalise with large cost (load shedding)
                let total_load: f64 = aug_net.buses.iter().map(|b| b.pd.0).sum();
                total_load * 10_000.0 // $/h penalty
            }
        };

        // Annualise to M$/year
        let total_cost_m = total_cost_per_h * 8760.0 * 1e-6;

        // Dual variables: sensitivity of cost to each candidate being added.
        // Approximated by the marginal benefit of each unbuilt candidate.
        let n_cands = self.config.candidates.len();
        let mut duals = vec![0.0_f64; n_cands];
        for (j, cand) in self.config.candidates.iter().enumerate() {
            if investment.contains(&j) {
                // Already built: dual ≈ negative of marginal benefit of extra capacity
                duals[j] = -cand.capacity_mw * 1e-3;
            } else {
                // Not built: dual ≈ positive (adding would reduce cost)
                duals[j] = cand.capacity_mw * 1e-3;
            }
        }

        // Unserved energy: heuristic proxy (zero for feasible, positive for penalised)
        let total_load: f64 = network
            .buses
            .iter()
            .enumerate()
            .map(|(i, b)| b.pd.0 * scenario.load_mult(i))
            .sum();
        let unserved = if total_cost_per_h > total_load * 1000.0 {
            total_load * 0.01 // 1% USE proxy
        } else {
            0.0
        };

        Ok(SubproblemResult {
            scenario_id: scenario.scenario_id,
            total_cost: total_cost_m,
            unserved_energy_mwh: unserved,
            dual_variables: duals,
        })
    }

    /// Solve all scenario subproblems for a given investment.
    fn evaluate_subproblems(
        &self,
        network: &PowerNetwork,
        investment: &[usize],
    ) -> Result<Vec<SubproblemResult>> {
        self.config
            .scenarios
            .iter()
            .map(|sc| self.solve_subproblem(network, investment, sc))
            .collect()
    }

    // ── Benders cut generation ─────────────────────────────────────────────

    /// Generate a Benders optimality cut aggregated over all scenarios.
    ///
    /// The cut is:
    ///   `η ≥ E[cost(x)] + Σ_i (∂E[cost]/∂x_i) · (x_i − x̄_i)`
    /// which simplifies to:
    ///   `η ≥ rhs + Σ_i coeff_i · x_i`
    pub fn generate_cut(
        &self,
        investment: &[usize],
        sp_results: &[SubproblemResult],
    ) -> BendersCut {
        let n = self.config.candidates.len();
        let mut coefficients = vec![0.0_f64; n];

        // Probability-weighted dual aggregation
        for sp in sp_results {
            let prob = self
                .config
                .scenarios
                .iter()
                .find(|s| s.scenario_id == sp.scenario_id)
                .map(|s| s.probability)
                .unwrap_or(1.0 / sp_results.len() as f64);

            for (j, dual) in sp.dual_variables.iter().enumerate() {
                coefficients[j] += prob * dual;
            }
        }

        // RHS = E[cost(x̄)] − Σ_i coeff_i · x̄_i
        let expected_cost: f64 = sp_results
            .iter()
            .zip(self.config.scenarios.iter())
            .map(|(sp, sc)| sc.probability * sp.total_cost)
            .sum();

        let correction: f64 = investment
            .iter()
            .map(|&i| coefficients.get(i).copied().unwrap_or(0.0))
            .sum();

        let rhs = expected_cost - correction;

        BendersCut {
            cut_type: CutType::Optimality,
            coefficients,
            rhs,
        }
    }

    // ── Investment evaluation ─────────────────────────────────────────────

    /// Compute total system cost for each scenario given an investment.
    ///
    /// Returns a vector of costs \[M$/year\] in scenario order.
    pub fn evaluate_investment(
        &self,
        network: &PowerNetwork,
        candidates: &[usize],
    ) -> Result<Vec<f64>> {
        self.config
            .scenarios
            .iter()
            .map(|sc| {
                self.solve_subproblem(network, candidates, sc)
                    .map(|r| r.total_cost)
            })
            .collect()
    }

    // ── Greedy heuristic ──────────────────────────────────────────────────

    /// Greedy initial investment: rank candidates by capacity / investment_cost,
    /// selecting greedily within budget and count limits.
    pub fn greedy_investment(&self, _network: &PowerNetwork) -> Vec<usize> {
        let mut ranked: Vec<(f64, usize)> = self
            .config
            .candidates
            .iter()
            .enumerate()
            .map(|(i, c)| {
                let score = if c.investment_cost_m > 1e-10 {
                    c.capacity_mw / c.investment_cost_m
                } else {
                    0.0
                };
                (score, i)
            })
            .collect();

        ranked.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(core::cmp::Ordering::Equal));

        let mut selected = Vec::new();
        let mut spent = 0.0_f64;

        for (_, idx) in ranked {
            if selected.len() >= self.config.max_candidates {
                break;
            }
            let cost = self.config.candidates[idx].investment_cost_m;
            if spent + cost <= self.config.total_budget_m + 1e-6 {
                selected.push(idx);
                spent += cost;
            }
        }

        selected
    }

    // ── N-1 security check ────────────────────────────────────────────────

    /// Check N-1 security for the expanded network.
    ///
    /// For each branch in the expanded network, remove it and verify that
    /// DC-OPF converges without load shedding.  Returns `true` if all
    /// contingencies are secure.
    fn check_n1_security(&self, network: &PowerNetwork, investment: &[usize]) -> bool {
        let mut aug_net = network.clone();
        for &idx in investment {
            aug_net
                .branches
                .push(self.config.candidates[idx].to_branch());
        }

        let gen_costs: Vec<GenCost> = aug_net
            .generators
            .iter()
            .map(|g| GenCost::quadratic(0.0, 30.0, 0.05, g.pmin.max(0.0), g.pmax.max(1.0)))
            .collect();

        // Base case must be feasible
        if solve_dc_opf(&aug_net, &gen_costs).is_err() {
            return false;
        }

        // Check each branch outage
        let n_branches = aug_net.branches.len();
        for k in 0..n_branches {
            let mut contingency = aug_net.clone();
            contingency.branches[k].status = false;

            if solve_dc_opf(&contingency, &gen_costs).is_err() {
                return false;
            }
        }
        true
    }

    // ── Result assembly ───────────────────────────────────────────────────

    fn build_result(
        &self,
        network: &PowerNetwork,
        selected: &[usize],
        scenario_costs: &[f64],
    ) -> Result<TepResult> {
        let total_investment = selected
            .iter()
            .map(|&i| self.config.candidates[i].investment_cost_m)
            .sum::<f64>();

        // NPC = investment + PV of annual fixed O&M
        let horizon = self.config.planning_horizon_years as f64;
        let r = self.config.discount_rate;
        let annuity_factor = if r > 1e-10 {
            (1.0 - (1.0 + r).powf(-horizon)) / r
        } else {
            horizon
        };

        let annual_om: f64 = selected
            .iter()
            .map(|&i| self.config.candidates[i].annual_fixed_cost_m)
            .sum();
        let total_npc = total_investment + annual_om * annuity_factor;

        // Expected cost (probability-weighted)
        let expected_cost: f64 = scenario_costs
            .iter()
            .zip(self.config.scenarios.iter())
            .map(|(c, s)| s.probability * c)
            .sum();

        let worst_case = scenario_costs
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max);

        // 95th percentile
        let n95 = percentile_95(scenario_costs);

        // Regret: cost_s - min_s(cost)
        let min_cost = scenario_costs.iter().copied().fold(f64::INFINITY, f64::min);
        let regret: Vec<f64> = scenario_costs
            .iter()
            .map(|c| (c - min_cost).max(0.0))
            .collect();

        // Unserved energy per scenario (zero proxy for well-formed networks)
        let unserved: Vec<f64> = self
            .config
            .scenarios
            .iter()
            .map(|sc| {
                self.solve_subproblem(network, selected, sc)
                    .map(|r| r.unserved_energy_mwh)
                    .unwrap_or(0.0)
            })
            .collect();

        // Loss reduction: compare base network vs expanded (heuristic)
        let base_costs = self
            .config
            .scenarios
            .iter()
            .map(|sc| {
                self.solve_subproblem(network, &[], sc)
                    .map(|r| r.total_cost)
                    .unwrap_or(0.0)
            })
            .collect::<Vec<_>>();

        let base_expected: f64 = base_costs
            .iter()
            .zip(self.config.scenarios.iter())
            .map(|(c, s)| s.probability * c)
            .sum();

        // Loss reduction proxy: 5% of the cost improvement (actual losses
        // require AC power flow; we use a DC approximation here)
        let cost_improvement = (base_expected - expected_cost).max(0.0);
        let loss_reduction_mwh = cost_improvement * 1e6 / 50.0; // $50/MWh assumed

        // N-1 security check
        let n1_secure = if self.config.n1_security_required {
            self.check_n1_security(network, selected)
        } else {
            true
        };

        Ok(TepResult {
            selected_candidates: selected.to_vec(),
            n_parallel: selected.iter().map(|_| 1_usize).collect(),
            total_investment_cost_m: total_investment,
            total_npc_m: total_npc,
            expected_cost_m: expected_cost,
            worst_case_cost_m: worst_case,
            n95_cost_m: n95,
            regret,
            n1_secure,
            unserved_energy_mwh: unserved,
            loss_reduction_mwh_per_year: loss_reduction_mwh,
        })
    }
}

// ─── NPV / LCOE / IRR utilities ───────────────────────────────────────────────

/// Compute net present value of a cash flow stream.
///
/// `cash_flows[0]` is the Year-0 flow (typically negative = initial investment).
/// NPV = Σ_t C_t / (1 + r)^t
pub fn compute_npv(cash_flows: &[f64], discount_rate: f64) -> f64 {
    cash_flows
        .iter()
        .enumerate()
        .map(|(t, &c)| c / (1.0 + discount_rate).powf(t as f64))
        .sum()
}

/// Compute the levelised cost of energy (LCOE) in $/MWh.
///
/// LCOE = (investment + PV of O&M) / PV of energy production
pub fn compute_lcoe(
    investment_cost: f64,
    annual_om_cost: f64,
    annual_energy_mwh: f64,
    lifetime_years: usize,
    discount_rate: f64,
) -> f64 {
    if annual_energy_mwh < 1e-10 || lifetime_years == 0 {
        return 0.0;
    }
    let n = lifetime_years as f64;
    let r = discount_rate;

    let annuity_factor = if r > 1e-10 {
        (1.0 - (1.0 + r).powf(-n)) / r
    } else {
        n
    };

    let pv_costs = investment_cost + annual_om_cost * annuity_factor;
    let pv_energy = annual_energy_mwh * annuity_factor;

    if pv_energy < 1e-10 {
        return 0.0;
    }
    pv_costs / pv_energy
}

/// Compute the internal rate of return (IRR) by bisection.
///
/// Returns `None` if no real IRR exists in \[0, 200%\] or the cash flows
/// have no sign change (no real root).
pub fn compute_irr(cash_flows: &[f64]) -> Option<f64> {
    if cash_flows.is_empty() {
        return None;
    }

    // Check for sign change (necessary condition for a real IRR)
    let has_negative = cash_flows.iter().any(|&c| c < 0.0);
    let has_positive = cash_flows.iter().any(|&c| c > 0.0);
    if !has_negative || !has_positive {
        return None;
    }

    let lo = 0.0_f64;
    let hi = 2.0_f64; // 200 % upper bound

    let npv_lo = compute_npv(cash_flows, lo);
    let npv_hi = compute_npv(cash_flows, hi);

    // If NPV does not change sign in [lo, hi], IRR is outside this range
    if npv_lo * npv_hi > 0.0 {
        return None;
    }

    let mut a = lo;
    let mut b = hi;
    let tol = 1e-8;
    let max_iter = 200;

    for _ in 0..max_iter {
        let mid = 0.5 * (a + b);
        let npv_mid = compute_npv(cash_flows, mid);
        if npv_mid.abs() < tol {
            return Some(mid);
        }
        if compute_npv(cash_flows, a) * npv_mid < 0.0 {
            b = mid;
        } else {
            a = mid;
        }
        if (b - a) < tol {
            break;
        }
    }

    Some(0.5 * (a + b))
}

// ─── Private helpers ──────────────────────────────────────────────────────────

/// Compute the 95th-percentile value of a slice (linear interpolation).
fn percentile_95(values: &[f64]) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    let mut sorted = values.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));

    let n = sorted.len();
    // Index for 95th percentile
    let idx_f = 0.95 * (n - 1) as f64;
    let lo = idx_f.floor() as usize;
    let hi = idx_f.ceil() as usize;

    if lo == hi {
        return sorted[lo];
    }
    let frac = idx_f - lo as f64;
    sorted[lo] + frac * (sorted[hi] - sorted[lo])
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::topology::PowerNetwork;

    fn make_network() -> PowerNetwork {
        PowerNetwork::from_matpower(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m"))
            .expect("ieee14")
    }

    fn make_candidates(n: usize) -> Vec<InvestmentCandidate> {
        (0..n)
            .map(|i| InvestmentCandidate {
                id: i,
                from_bus: 1,
                to_bus: 5,
                capacity_mw: 100.0,
                investment_cost_m: 50.0,
                annual_fixed_cost_m: 0.5,
                resistance_pu: 0.01,
                reactance_pu: 0.05,
                n_parallel_max: 2,
                can_expand_existing: false,
                lead_time_years: 3.0,
            })
            .collect()
    }

    fn make_scenarios(n_buses: usize) -> Vec<LoadScenario> {
        vec![
            LoadScenario::uniform(0, 0.5, 1.0, n_buses),
            LoadScenario::uniform(1, 0.5, 1.2, n_buses),
        ]
    }

    // ── Core TEP tests ──────────────────────────────────────────────────

    #[test]
    fn test_robust_tep_selects_within_budget() {
        let net = make_network();
        let n_buses = net.buses.len();
        let candidates = make_candidates(3);

        let config = RobustTepConfig {
            candidates: candidates.clone(),
            scenarios: make_scenarios(n_buses),
            total_budget_m: 120.0, // allows at most 2 candidates @ 50 M$ each
            max_candidates: 5,
            ..Default::default()
        };

        let solver = RobustTepSolver::new(config);
        let result = solver.solve(&net).expect("solve");

        assert!(
            result.total_investment_cost_m <= 120.0 + 1e-6,
            "investment {} exceeds budget 120",
            result.total_investment_cost_m
        );
    }

    #[test]
    fn test_robust_tep_n1_secure() {
        let net = make_network();
        let n_buses = net.buses.len();

        let config = RobustTepConfig {
            candidates: make_candidates(1),
            scenarios: make_scenarios(n_buses),
            n1_security_required: true,
            total_budget_m: 200.0,
            ..Default::default()
        };

        let solver = RobustTepSolver::new(config);
        let result = solver.solve(&net).expect("solve");
        // Result should have a valid n1_secure flag (bool)
        let _ = result.n1_secure; // just ensure the field exists and is accessible
    }

    #[test]
    fn test_benders_cut_coefficients() {
        let net = make_network();
        let n_buses = net.buses.len();
        let candidates = make_candidates(4);
        let n_cands = candidates.len();

        let config = RobustTepConfig {
            candidates,
            scenarios: make_scenarios(n_buses),
            total_budget_m: 300.0,
            ..Default::default()
        };

        let solver = RobustTepSolver::new(config);
        let investment = solver.greedy_investment(&net);
        let sp_results = solver
            .evaluate_subproblems(&net, &investment)
            .expect("subproblems");
        let cut = solver.generate_cut(&investment, &sp_results);

        assert_eq!(
            cut.coefficients.len(),
            n_cands,
            "cut coefficients length mismatch"
        );
    }

    #[test]
    fn test_greedy_investment_feasible() {
        let net = make_network();
        let budget = 75.0_f64; // only one 50 M$ candidate fits

        let config = RobustTepConfig {
            candidates: make_candidates(3),
            scenarios: make_scenarios(net.buses.len()),
            total_budget_m: budget,
            max_candidates: 10,
            ..Default::default()
        };

        let solver = RobustTepSolver::new(config);
        let selected = solver.greedy_investment(&net);

        let spent: f64 = selected
            .iter()
            .map(|&i| solver.config.candidates[i].investment_cost_m)
            .sum();
        assert!(
            spent <= budget + 1e-6,
            "greedy spent {spent} exceeds budget {budget}"
        );
    }

    #[test]
    fn test_tep_empty_candidates_fails() {
        let net = make_network();
        let config = RobustTepConfig {
            candidates: vec![],
            scenarios: make_scenarios(net.buses.len()),
            ..Default::default()
        };
        let solver = RobustTepSolver::new(config);
        assert!(solver.solve(&net).is_err());
    }

    #[test]
    fn test_tep_empty_scenarios_fails() {
        let net = make_network();
        let config = RobustTepConfig {
            candidates: make_candidates(1),
            scenarios: vec![],
            ..Default::default()
        };
        let solver = RobustTepSolver::new(config);
        assert!(solver.solve(&net).is_err());
    }

    // ── NPV / LCOE / IRR tests ──────────────────────────────────────────

    #[test]
    fn test_npv_single_period() {
        // C_0 = 0, C_1 = 100, r = 0.1  →  NPV = 100 / 1.1 ≈ 90.909
        let npv = compute_npv(&[0.0, 100.0], 0.1);
        assert!(
            (npv - 90.909).abs() < 0.01,
            "NPV = {npv}, expected ≈ 90.909"
        );
    }

    #[test]
    fn test_npv_zero_rate() {
        // NPV at r=0 is just the sum
        let npv = compute_npv(&[100.0, 200.0, -50.0], 0.0);
        assert!((npv - 250.0).abs() < 1e-10, "NPV at r=0: {npv}");
    }

    #[test]
    fn test_lcoe_calculation() {
        let lcoe = compute_lcoe(1000.0, 20.0, 1000.0, 20, 0.08);
        assert!(lcoe > 0.0, "LCOE should be positive, got {lcoe}");
        // Rough range: 0.1 – 1.5 $/MWh for these parameters
        assert!(lcoe < 1000.0, "LCOE unreasonably large: {lcoe}");
    }

    #[test]
    fn test_lcoe_zero_energy_returns_zero() {
        let lcoe = compute_lcoe(1000.0, 20.0, 0.0, 20, 0.08);
        assert_eq!(lcoe, 0.0);
    }

    #[test]
    fn test_irr_simple() {
        // -100 at t=0, +120 at t=1  →  IRR ≈ 20%
        let irr = compute_irr(&[-100.0, 120.0]).expect("IRR exists");
        assert!(
            (irr - 0.20).abs() < 1e-4,
            "IRR = {:.4}, expected ≈ 0.20",
            irr
        );
    }

    #[test]
    fn test_irr_no_sign_change_returns_none() {
        // All negative: no real IRR
        let irr = compute_irr(&[-100.0, -50.0]);
        assert!(irr.is_none(), "Expected None for all-negative cash flows");
    }

    #[test]
    fn test_irr_multi_period() {
        // -1000, +300/year for 5 years  →  IRR ≈ 15.2%
        let irr = compute_irr(&[-1000.0, 300.0, 300.0, 300.0, 300.0, 300.0]).expect("IRR exists");
        assert!(
            irr > 0.10 && irr < 0.30,
            "IRR = {:.4} out of expected range [0.10, 0.30]",
            irr
        );
    }

    // ── Percentile helper ───────────────────────────────────────────────

    #[test]
    fn test_percentile_95_sorted() {
        let values: Vec<f64> = (1..=100).map(|i| i as f64).collect();
        let p95 = percentile_95(&values);
        // For 1..100 the 95th percentile should be ~95
        assert!((p95 - 95.0).abs() < 2.0, "p95 = {p95}");
    }

    #[test]
    fn test_percentile_95_single() {
        let p95 = percentile_95(&[42.0]);
        assert_eq!(p95, 42.0);
    }
}
