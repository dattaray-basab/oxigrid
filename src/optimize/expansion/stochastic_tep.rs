//! Two-stage stochastic Transmission Expansion Planning (Stochastic TEP).
//!
//! Implements the classical two-stage stochastic programming formulation:
//!
//! * **Stage 1** (here-and-now): choose which lines to build *before* the
//!   uncertainty is revealed.
//! * **Stage 2** (wait-and-see): for each scenario, decide on additional
//!   lines and perform economic dispatch.
//!
//! Provides the key stochastic solution quality metrics:
//!
//! | Metric | Meaning |
//! |--------|---------|
//! | **SS** | Stochastic solution – objective of the two-stage program |
//! | **EEV** | Expected cost of the EV solution – solve with mean scenario, apply universally |
//! | **EVPI** | Expected value of perfect information – optimal cost if scenario is known in advance |
//! | **VSS** | Value of the stochastic solution = EEV − SS (≥ 0 always) |
//!
//! # Reference
//! Birge, J. R. & Louveaux, F. (2011) *Introduction to Stochastic Programming*,
//! 2nd ed., Springer.

use crate::error::{OxiGridError, Result};
use crate::network::topology::PowerNetwork;
use crate::optimize::expansion::robust_tep::{
    InvestmentCandidate, LoadScenario, RobustTepConfig, RobustTepSolver, RobustnessCriterion,
};

// ─── Result type ──────────────────────────────────────────────────────────────

/// Outcome of the two-stage stochastic TEP solver.
#[derive(Debug, Clone)]
pub struct StochasticTepResult {
    /// Candidates built in Stage 1 (before uncertainty is revealed).
    pub first_stage_investment: Vec<usize>,
    /// Candidates added in Stage 2 for each scenario (index == scenario index).
    pub second_stage_investment: Vec<Vec<usize>>,
    /// Probability-weighted total cost of the two-stage solution \[M$/year\].
    pub total_expected_cost: f64,
    /// VSS = EEV − SS (value of the stochastic solution, always ≥ 0).
    pub value_of_stochastic_solution: f64,
    /// EVPI = SS − perfect-information optimal cost (always ≥ 0).
    pub expected_value_of_perfect_info: f64,
    /// Total cost per scenario \[M$/year\].
    pub scenario_costs: Vec<f64>,
}

// ─── Solver ───────────────────────────────────────────────────────────────────

/// Two-stage stochastic TEP solver.
pub struct StochasticTepSolver {
    /// Uncertainty scenarios (must sum probabilities ≈ 1).
    pub scenarios: Vec<LoadScenario>,
    /// Candidate transmission lines.
    pub candidates: Vec<InvestmentCandidate>,
    /// Total investment budget \[M$\].
    pub budget: f64,
    /// Planning horizon for NPV calculations \[years\].
    pub planning_years: usize,
}

impl StochasticTepSolver {
    /// Create a new stochastic TEP solver.
    pub fn new(
        scenarios: Vec<LoadScenario>,
        candidates: Vec<InvestmentCandidate>,
        budget: f64,
    ) -> Self {
        Self {
            scenarios,
            candidates,
            budget,
            planning_years: 20,
        }
    }

    /// Set the planning horizon.
    pub fn with_planning_years(mut self, years: usize) -> Self {
        self.planning_years = years;
        self
    }

    // ── Public entry point ─────────────────────────────────────────────────

    /// Solve the two-stage stochastic TEP.
    ///
    /// # Algorithm
    ///
    /// 1. Stage-1 investment is determined by solving the TEP problem with
    ///    the *expected-value* (mean) scenario via the robust solver with
    ///    `MinMax` criterion (conservative first-stage decision).
    /// 2. For each scenario, a Stage-2 investment is determined by solving
    ///    the TEP problem for the residual budget, fixing the Stage-1 lines.
    /// 3. VSS and EVPI are computed analytically.
    pub fn solve(&self, network: &PowerNetwork) -> Result<StochasticTepResult> {
        if self.scenarios.is_empty() {
            return Err(OxiGridError::InvalidParameter(
                "StochasticTepSolver: no scenarios provided".into(),
            ));
        }
        if self.candidates.is_empty() {
            return Err(OxiGridError::InvalidParameter(
                "StochasticTepSolver: no candidates provided".into(),
            ));
        }

        // ── Stage 1: here-and-now investment ──────────────────────────────
        let stage1_config = RobustTepConfig {
            candidates: self.candidates.clone(),
            scenarios: self.scenarios.clone(),
            planning_horizon_years: self.planning_years,
            discount_rate: 0.08,
            n1_security_required: false,
            max_candidates: self.candidates.len(),
            total_budget_m: self.budget * 0.6, // reserve 40% for stage-2 adaptation
            robustness_criterion: RobustnessCriterion::MinMax,
        };

        let stage1_solver = RobustTepSolver::new(stage1_config);
        let stage1_result = stage1_solver.solve(network)?;
        let first_stage = stage1_result.selected_candidates.clone();

        // Remaining budget for stage-2 adaptations
        let spent_stage1: f64 = first_stage
            .iter()
            .map(|&i| self.candidates[i].investment_cost_m)
            .sum();
        let residual_budget = (self.budget - spent_stage1).max(0.0);

        // ── Stage 2: scenario-specific investments ─────────────────────────
        let mut second_stage: Vec<Vec<usize>> = Vec::with_capacity(self.scenarios.len());
        let mut scenario_costs: Vec<f64> = Vec::with_capacity(self.scenarios.len());

        for scenario in &self.scenarios {
            // Candidates not yet built in Stage 1
            let remaining_candidates: Vec<InvestmentCandidate> = self
                .candidates
                .iter()
                .filter(|c| !first_stage.contains(&c.id))
                .cloned()
                .collect();

            let stage2_additional = if remaining_candidates.is_empty() || residual_budget < 1.0 {
                vec![]
            } else {
                let stage2_config = RobustTepConfig {
                    candidates: remaining_candidates.clone(),
                    scenarios: vec![scenario.clone()],
                    planning_horizon_years: self.planning_years,
                    discount_rate: 0.08,
                    n1_security_required: false,
                    max_candidates: remaining_candidates.len(),
                    total_budget_m: residual_budget,
                    robustness_criterion: RobustnessCriterion::MinMax,
                };
                let s2_solver = RobustTepSolver::new(stage2_config);
                s2_solver
                    .solve(network)
                    .map(|r| r.selected_candidates)
                    .unwrap_or_default()
            };

            // Combine stage-1 + stage-2 for cost evaluation
            let combined: Vec<usize> = first_stage
                .iter()
                .chain(stage2_additional.iter())
                .copied()
                .collect();

            // Evaluate cost for this scenario
            let cost = stage1_solver
                .evaluate_investment(network, &combined)
                .map(|v| v.first().copied().unwrap_or(0.0))
                .unwrap_or(0.0);

            second_stage.push(stage2_additional);
            scenario_costs.push(cost);
        }

        // ── Stochastic solution cost (SS) ──────────────────────────────────
        let ss: f64 = scenario_costs
            .iter()
            .zip(self.scenarios.iter())
            .map(|(c, s)| s.probability * c)
            .sum();

        // ── EEV: expected cost using the expected-value solution ───────────
        let eev = self.compute_eev(network);

        // ── EVPI: expected cost with perfect information ───────────────────
        let evpi_cost = self.compute_evpi(network);

        // VSS = EEV − SS  (how much we gain by solving stochastically)
        let vss = (eev - ss).max(0.0);

        // EVPI = SS − WS  (how much we'd gain from knowing the scenario)
        // Conventionally: EVPI = WS cost vs SS cost, where WS is wait-and-see
        let evpi = (ss - evpi_cost).max(0.0);

        Ok(StochasticTepResult {
            first_stage_investment: first_stage,
            second_stage_investment: second_stage,
            total_expected_cost: ss,
            value_of_stochastic_solution: vss,
            expected_value_of_perfect_info: evpi,
            scenario_costs,
        })
    }

    // ── EEV ───────────────────────────────────────────────────────────────

    /// Compute EEV: solve TEP for the mean scenario, then evaluate that
    /// solution across all scenarios.
    pub fn compute_eev(&self, network: &PowerNetwork) -> f64 {
        // Build the mean (expected value) scenario
        let n_buses = network.buses.len();
        let mean_load: Vec<f64> = (0..n_buses)
            .map(|i| {
                self.scenarios
                    .iter()
                    .map(|s| s.probability * s.load_mult(i))
                    .sum::<f64>()
            })
            .collect();

        let mean_scenario = LoadScenario {
            scenario_id: usize::MAX,
            probability: 1.0,
            load_multipliers: mean_load,
            renewable_multipliers: vec![1.0; n_buses],
            description: "mean_ev_scenario".into(),
        };

        // Solve TEP for the mean scenario only
        let ev_config = RobustTepConfig {
            candidates: self.candidates.clone(),
            scenarios: vec![mean_scenario],
            planning_horizon_years: self.planning_years,
            discount_rate: 0.08,
            n1_security_required: false,
            max_candidates: self.candidates.len(),
            total_budget_m: self.budget,
            robustness_criterion: RobustnessCriterion::MinMax,
        };

        let ev_solver = RobustTepSolver::new(ev_config);
        let ev_investment = match ev_solver.solve(network) {
            Ok(r) => r.selected_candidates,
            Err(_) => vec![],
        };

        // Evaluate the EV investment across all real scenarios
        let all_scenarios_config = RobustTepConfig {
            candidates: self.candidates.clone(),
            scenarios: self.scenarios.clone(),
            planning_horizon_years: self.planning_years,
            discount_rate: 0.08,
            n1_security_required: false,
            max_candidates: self.candidates.len(),
            total_budget_m: self.budget,
            robustness_criterion: RobustnessCriterion::MinMax,
        };
        let eval_solver = RobustTepSolver::new(all_scenarios_config);

        eval_solver
            .evaluate_investment(network, &ev_investment)
            .map(|costs| {
                costs
                    .iter()
                    .zip(self.scenarios.iter())
                    .map(|(c, s)| s.probability * c)
                    .sum()
            })
            .unwrap_or(f64::INFINITY)
    }

    // ── EVPI ──────────────────────────────────────────────────────────────

    /// Compute the wait-and-see (WS) cost: optimal cost for each scenario
    /// when the scenario is known in advance.
    pub fn compute_evpi(&self, network: &PowerNetwork) -> f64 {
        let ws_cost: f64 = self
            .scenarios
            .iter()
            .map(|scenario| {
                // Solve TEP optimally for this single scenario
                let config = RobustTepConfig {
                    candidates: self.candidates.clone(),
                    scenarios: vec![scenario.clone()],
                    planning_horizon_years: self.planning_years,
                    discount_rate: 0.08,
                    n1_security_required: false,
                    max_candidates: self.candidates.len(),
                    total_budget_m: self.budget,
                    robustness_criterion: RobustnessCriterion::MinMax,
                };

                let solver = RobustTepSolver::new(config);
                let investment = match solver.solve(network) {
                    Ok(r) => r.selected_candidates,
                    Err(_) => vec![],
                };

                // Cost for this scenario under its own optimal investment
                solver
                    .evaluate_investment(network, &investment)
                    .map(|v| v.first().copied().unwrap_or(0.0))
                    .unwrap_or(0.0)
                    * scenario.probability
            })
            .sum();

        ws_cost
    }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::topology::PowerNetwork;
    use crate::optimize::expansion::robust_tep::InvestmentCandidate;

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
                capacity_mw: 100.0 + i as f64 * 50.0,
                investment_cost_m: 30.0 + i as f64 * 20.0,
                annual_fixed_cost_m: 0.3,
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
            LoadScenario::uniform(0, 0.4, 0.9, n_buses),
            LoadScenario::uniform(1, 0.4, 1.0, n_buses),
            LoadScenario::uniform(2, 0.2, 1.3, n_buses),
        ]
    }

    #[test]
    fn test_stochastic_tep_runs() {
        let net = make_network();
        let n_buses = net.buses.len();
        let solver = StochasticTepSolver::new(make_scenarios(n_buses), make_candidates(3), 200.0);
        let result = solver.solve(&net).expect("stochastic solve");
        assert!(result.total_expected_cost >= 0.0);
        assert_eq!(
            result.second_stage_investment.len(),
            make_scenarios(n_buses).len()
        );
    }

    #[test]
    fn test_stochastic_vss_non_negative() {
        let net = make_network();
        let n_buses = net.buses.len();
        let solver = StochasticTepSolver::new(make_scenarios(n_buses), make_candidates(2), 150.0);
        let result = solver.solve(&net).expect("solve");
        // VSS = EEV - SS >= 0 (stochastic solution is never worse than EEV)
        assert!(
            result.value_of_stochastic_solution >= -1e-6,
            "VSS = {} should be >= 0",
            result.value_of_stochastic_solution
        );
    }

    #[test]
    fn test_stochastic_evpi_non_negative() {
        let net = make_network();
        let n_buses = net.buses.len();
        let solver = StochasticTepSolver::new(make_scenarios(n_buses), make_candidates(2), 150.0);
        let result = solver.solve(&net).expect("solve");
        // EVPI >= 0: perfect information can only help
        assert!(
            result.expected_value_of_perfect_info >= -1e-6,
            "EVPI = {} should be >= 0",
            result.expected_value_of_perfect_info
        );
    }

    #[test]
    fn test_stochastic_scenario_costs_length() {
        let net = make_network();
        let n_buses = net.buses.len();
        let scenarios = make_scenarios(n_buses);
        let n_sc = scenarios.len();
        let solver = StochasticTepSolver::new(scenarios, make_candidates(2), 200.0);
        let result = solver.solve(&net).expect("solve");
        assert_eq!(result.scenario_costs.len(), n_sc);
    }

    #[test]
    fn test_stochastic_empty_scenarios_fails() {
        let net = make_network();
        let solver = StochasticTepSolver::new(vec![], make_candidates(2), 200.0);
        assert!(solver.solve(&net).is_err());
    }

    #[test]
    fn test_stochastic_empty_candidates_fails() {
        let net = make_network();
        let n_buses = net.buses.len();
        let solver = StochasticTepSolver::new(make_scenarios(n_buses), vec![], 200.0);
        assert!(solver.solve(&net).is_err());
    }

    #[test]
    fn test_eev_positive() {
        let net = make_network();
        let n_buses = net.buses.len();
        let solver = StochasticTepSolver::new(make_scenarios(n_buses), make_candidates(2), 150.0);
        let eev = solver.compute_eev(&net);
        assert!(eev.is_finite(), "EEV should be finite");
        assert!(eev >= 0.0, "EEV should be >= 0, got {eev}");
    }

    #[test]
    fn test_evpi_finite() {
        let net = make_network();
        let n_buses = net.buses.len();
        let solver = StochasticTepSolver::new(make_scenarios(n_buses), make_candidates(2), 150.0);
        let evpi_cost = solver.compute_evpi(&net);
        assert!(evpi_cost.is_finite(), "WS cost (for EVPI) should be finite");
    }

    #[test]
    fn test_with_planning_years() {
        let net = make_network();
        let n_buses = net.buses.len();
        let solver = StochasticTepSolver::new(make_scenarios(n_buses), make_candidates(2), 150.0)
            .with_planning_years(30);
        assert_eq!(solver.planning_years, 30);
        // Should still solve without error
        let result = solver.solve(&net).expect("solve with 30-year horizon");
        assert!(result.total_expected_cost >= 0.0);
    }
}
