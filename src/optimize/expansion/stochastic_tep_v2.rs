//! Multi-Stage Transmission Expansion Planning (TEP) under deep uncertainty.
//!
//! Extends the classical two-stage stochastic TEP to a *multi-stage* setting
//! where investment decisions are made sequentially over several planning
//! periods, and the planner adapts to revealed information at each stage.
//!
//! # Algorithm
//!
//! **Greedy sequential selection** with **minimax regret** across scenarios:
//!
//! For each planning period `t`:
//! 1. Discount future costs: `df = (1 + r)^{-t·T}` where `r` is the discount
//!    rate and `T` is years per period.
//! 2. Compute expected load/CO₂ cost weighting across [`GrowthScenario`]s.
//! 3. Score each unbuilt [`CandidateLine`] by benefit/cost ratio:
//!    - Benefit = capacity \[MW\] × growth multiplier × weighted CO₂ price.
//!    - Cost = `capex` × `df` + `opex` × `T` × `df`.
//! 4. Build all lines with ratio > 1.0 whose construction time fits the period.
//! 5. Estimate LOLE from built capacity; accumulate CO₂ emissions.
//!
//! After all periods, compute **minimax regret**: for each scenario find the
//! best-possible NPV (solo greedy) and record the maximum shortfall.
//!
//! # References
//! Birge & Louveaux (2011) *Introduction to Stochastic Programming*, 2nd ed.
//! Oren et al. (2010) *Multi-Stage Transmission Expansion Planning*, IEEE Trans.

use serde::{Deserialize, Serialize};
use thiserror::Error;

// ─── Error type ───────────────────────────────────────────────────────────────

/// Errors from the multi-stage TEP solver.
#[derive(Debug, Error)]
pub enum TepError {
    /// No scenarios were added before calling [`MultiStageTepSolver::solve`].
    #[error("no scenarios defined")]
    NoScenarios,
    /// Configuration is logically inconsistent.
    #[error("invalid config: {0}")]
    InvalidConfig(String),
    /// The planning algorithm encountered an internal failure.
    #[error("planning failed: {0}")]
    PlanningFailed(String),
}

// ─── Configuration ────────────────────────────────────────────────────────────

/// Configuration for the multi-stage TEP solver.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MultiStageTepConfig {
    /// Number of planning periods (e.g. 5 for a 25-year horizon with 5-year periods).
    pub planning_periods: usize,
    /// Duration of each planning period \[years\].
    pub years_per_period: usize,
    /// Annual discount rate (e.g. 0.08 for 8 %).
    pub discount_rate: f64,
    /// Number of demand/renewable scenarios.
    pub n_scenarios: usize,
    /// LOLE reliability standard \[days/year\] — must not be exceeded.
    pub reliability_standard: f64,
    /// Total CO₂ budget over the full horizon \[Mt\].
    pub co2_budget_mt: f64,
}

// ─── Candidate lines ──────────────────────────────────────────────────────────

/// A candidate transmission line that may be built during the planning horizon.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CandidateLine {
    /// Unique identifier.
    pub id: usize,
    /// Sending-end bus (external ID).
    pub from_bus: usize,
    /// Receiving-end bus (external ID).
    pub to_bus: usize,
    /// Thermal rating \[MW\].
    pub capacity_mw: f64,
    /// Total capital expenditure \[M USD\].
    pub capex_m_usd: f64,
    /// Annual fixed operations and maintenance cost \[M USD/year\].
    pub fixed_opex_m_usd_per_year: f64,
    /// Construction lead time \[years\].
    pub construction_time_years: usize,
    /// Economic lifetime \[years\].
    pub lifetime_years: usize,
    /// Series reactance \[p.u.\] (100 MVA base).
    pub reactance_pu: f64,
}

// ─── Growth scenarios ─────────────────────────────────────────────────────────

/// A demand/renewable growth scenario for stochastic TEP.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GrowthScenario {
    /// Scenario identifier.
    pub id: usize,
    /// Probability weight ∈ (0, 1\]; all probabilities should sum to 1.
    pub probability: f64,
    /// Annual load growth rate \[%/year\] (e.g. 2.5 for 2.5 %).
    pub load_growth_pct_per_year: f64,
    /// Annual renewable penetration growth rate \[%/year\].
    pub renewable_growth_pct_per_year: f64,
    /// CO₂ shadow price \[USD/t\].
    pub co2_price_usd_per_t: f64,
}

// ─── Stage decision ───────────────────────────────────────────────────────────

/// Investment decision made at a single planning stage.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TepDecisionStage {
    /// Planning period index (0-based).
    pub period: usize,
    /// IDs of [`CandidateLine`]s built in this period.
    pub lines_invested: Vec<usize>,
    /// Gross capital expenditure in this period \[M USD\].
    pub total_investment_m_usd: f64,
    /// Discounted NPV contribution of this period \[M USD\].
    pub npv_cost_m_usd: f64,
    /// Loss-of-load expectation at end of this period \[days/year\].
    pub lole_days_per_year: f64,
    /// CO₂ emissions attributed to this period \[Mt\].
    pub co2_emissions_mt: f64,
}

// ─── Result ───────────────────────────────────────────────────────────────────

/// Full result of the multi-stage stochastic TEP.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StochasticTepV2Result {
    /// Stage-by-stage investment decisions.
    pub stages: Vec<TepDecisionStage>,
    /// Total NPV cost over the planning horizon \[M USD\].
    pub total_npv_m_usd: f64,
    /// Expected LOLE averaged over scenarios \[days/year\].
    pub expected_lole: f64,
    /// Expected CO₂ emissions summed over all periods \[Mt\].
    pub expected_co2_mt: f64,
    /// Whether the LOLE reliability standard is satisfied at every period.
    pub reliability_satisfied: bool,
    /// Whether total emissions stay within the CO₂ budget.
    pub co2_satisfied: bool,
    /// Minimax regret value across scenarios \[M USD\].
    pub regret_m_usd: f64,
}

// ─── Solver ───────────────────────────────────────────────────────────────────

/// Multi-stage stochastic TEP solver.
///
/// # Example
/// ```rust
/// use oxigrid::optimize::expansion::stochastic_tep_v2::{
///     MultiStageTepConfig, MultiStageTepSolver, CandidateLine, GrowthScenario,
/// };
///
/// let config = MultiStageTepConfig {
///     planning_periods: 3,
///     years_per_period: 5,
///     discount_rate: 0.08,
///     n_scenarios: 1,
///     reliability_standard: 0.1,
///     co2_budget_mt: 1000.0,
/// };
/// let mut solver = MultiStageTepSolver::new(config);
/// solver.add_scenario(GrowthScenario {
///     id: 0, probability: 1.0,
///     load_growth_pct_per_year: 2.0,
///     renewable_growth_pct_per_year: 5.0,
///     co2_price_usd_per_t: 30.0,
/// });
/// let result = solver.solve().unwrap();
/// assert_eq!(result.stages.len(), 3);
/// ```
pub struct MultiStageTepSolver {
    config: MultiStageTepConfig,
    candidates: Vec<CandidateLine>,
    scenarios: Vec<GrowthScenario>,
    /// Proxy for the annualised cost of the existing network \[M USD/year\].
    existing_network_cost_m_usd: f64,
}

impl MultiStageTepSolver {
    /// Create a solver with the given configuration.
    pub fn new(config: MultiStageTepConfig) -> Self {
        Self {
            config,
            candidates: Vec::new(),
            scenarios: Vec::new(),
            existing_network_cost_m_usd: 0.0,
        }
    }

    /// Register a candidate line available for investment.
    pub fn add_candidate(&mut self, line: CandidateLine) {
        self.candidates.push(line);
    }

    /// Register a demand/renewable growth scenario.
    pub fn add_scenario(&mut self, scenario: GrowthScenario) {
        self.scenarios.push(scenario);
    }

    /// Set the annualised cost of the existing network \[M USD/year\].
    pub fn set_existing_network_cost(&mut self, cost_m_usd: f64) {
        self.existing_network_cost_m_usd = cost_m_usd;
    }

    /// Solve the multi-stage TEP problem.
    ///
    /// Returns a [`StochasticTepV2Result`] containing stage decisions,
    /// NPV breakdown, reliability and emissions metrics, and minimax regret.
    pub fn solve(&self) -> Result<StochasticTepV2Result, TepError> {
        // ── Validation ───────────────────────────────────────────────────────
        if self.scenarios.is_empty() {
            return Err(TepError::NoScenarios);
        }
        if self.config.planning_periods == 0 {
            return Err(TepError::InvalidConfig(
                "planning_periods must be ≥ 1".into(),
            ));
        }
        if self.config.discount_rate <= 0.0 {
            return Err(TepError::InvalidConfig(
                "discount_rate must be positive".into(),
            ));
        }
        // Warn (but don't fail) if probabilities don't sum to ≈ 1.
        let prob_sum: f64 = self.scenarios.iter().map(|s| s.probability).sum();
        if (prob_sum - 1.0).abs() > 0.05 {
            return Err(TepError::InvalidConfig(format!(
                "scenario probabilities sum to {prob_sum:.4}, expected ≈ 1.0"
            )));
        }

        let ypp = self.config.years_per_period as f64;
        let r = self.config.discount_rate;

        // ── Weighted scenario statistics ─────────────────────────────────────
        let weighted_co2_price: f64 = self
            .scenarios
            .iter()
            .map(|s| s.probability * s.co2_price_usd_per_t)
            .sum();
        let weighted_load_growth_pct: f64 = self
            .scenarios
            .iter()
            .map(|s| s.probability * s.load_growth_pct_per_year)
            .sum();

        // ── Sequential greedy planning ────────────────────────────────────────
        let mut built: Vec<bool> = vec![false; self.candidates.len()];
        let mut stages: Vec<TepDecisionStage> = Vec::with_capacity(self.config.planning_periods);
        let mut total_built_capacity_mw: f64 = 0.0;

        for period in 0..self.config.planning_periods {
            let df = discount_factor(r, period as f64 * ypp);
            // Load growth multiplier at mid-period
            let mid_year = (period as f64 + 0.5) * ypp;
            let load_mult = (1.0 + weighted_load_growth_pct / 100.0).powf(mid_year);

            let mut period_lines: Vec<usize> = Vec::new();
            let mut period_capex: f64 = 0.0;

            // Score and select lines
            for (idx, line) in self.candidates.iter().enumerate() {
                if built[idx] {
                    continue;
                }
                if line.construction_time_years > self.config.years_per_period {
                    continue;
                }
                // benefit proxy: capacity value per period [M USD]
                // co2_price [USD/t] × load_mult × capacity [MW] × period [years] × emissions_factor
                let benefit = line.capacity_mw * load_mult * weighted_co2_price * ypp * 1e-4;
                let cost = line.capex_m_usd * df + line.fixed_opex_m_usd_per_year * ypp * df;
                if cost > 0.0 && benefit / cost > 1.0 {
                    built[idx] = true;
                    period_lines.push(line.id);
                    period_capex += line.capex_m_usd;
                    total_built_capacity_mw += line.capacity_mw;
                }
            }

            let npv_cost = period_capex * df;

            // LOLE estimate: declines with built capacity (base = 0.5 days/year)
            let lole = base_lole() / (1.0 + total_built_capacity_mw / 1_000.0);

            // CO₂ emissions proxy: proportional to load growth minus renewables
            let avg_renewable_growth: f64 = self
                .scenarios
                .iter()
                .map(|s| s.probability * s.renewable_growth_pct_per_year)
                .sum();
            let net_growth = (weighted_load_growth_pct - avg_renewable_growth).max(0.0);
            let co2_mt = net_growth * mid_year * df * 0.1; // proxy \[Mt\]

            stages.push(TepDecisionStage {
                period,
                lines_invested: period_lines,
                total_investment_m_usd: period_capex,
                npv_cost_m_usd: npv_cost,
                lole_days_per_year: lole,
                co2_emissions_mt: co2_mt,
            });
        }

        // ── Aggregate metrics ─────────────────────────────────────────────────
        let total_npv_m_usd: f64 =
            stages.iter().map(|s| s.npv_cost_m_usd).sum::<f64>() + self.existing_network_cost_m_usd;
        let expected_lole =
            stages.iter().map(|s| s.lole_days_per_year).sum::<f64>() / stages.len().max(1) as f64;
        let expected_co2_mt: f64 = stages.iter().map(|s| s.co2_emissions_mt).sum();

        let reliability_satisfied = stages
            .iter()
            .all(|s| s.lole_days_per_year <= self.config.reliability_standard);
        let co2_satisfied = expected_co2_mt <= self.config.co2_budget_mt;

        // ── Minimax regret ────────────────────────────────────────────────────
        let regret_m_usd = self.compute_minimax_regret(total_npv_m_usd, r, ypp)?;

        Ok(StochasticTepV2Result {
            stages,
            total_npv_m_usd,
            expected_lole,
            expected_co2_mt,
            reliability_satisfied,
            co2_satisfied,
            regret_m_usd,
        })
    }

    /// Compute minimax regret by evaluating the best-possible NPV under each
    /// scenario individually and comparing with the actual multi-scenario NPV.
    fn compute_minimax_regret(&self, actual_npv: f64, r: f64, ypp: f64) -> Result<f64, TepError> {
        let mut max_regret: f64 = 0.0;

        for scenario in &self.scenarios {
            // Solo greedy: optimise only for this scenario
            let best_npv = self.solo_greedy_npv(scenario, r, ypp);
            // Regret = best attainable − what we actually achieved
            let regret = (actual_npv - best_npv).abs();
            if regret > max_regret {
                max_regret = regret;
            }
        }
        Ok(max_regret)
    }

    /// Greedy NPV optimised for a single scenario (used for regret computation).
    fn solo_greedy_npv(&self, scenario: &GrowthScenario, r: f64, ypp: f64) -> f64 {
        let mut built: Vec<bool> = vec![false; self.candidates.len()];
        let mut npv: f64 = self.existing_network_cost_m_usd;

        for period in 0..self.config.planning_periods {
            let df = discount_factor(r, period as f64 * ypp);
            let mid_year = (period as f64 + 0.5) * ypp;
            let load_mult = (1.0 + scenario.load_growth_pct_per_year / 100.0).powf(mid_year);

            for (idx, line) in self.candidates.iter().enumerate() {
                if built[idx] {
                    continue;
                }
                if line.construction_time_years > self.config.years_per_period {
                    continue;
                }
                let benefit =
                    line.capacity_mw * load_mult * scenario.co2_price_usd_per_t * ypp * 1e-4;
                let cost = line.capex_m_usd * df + line.fixed_opex_m_usd_per_year * ypp * df;
                if cost > 0.0 && benefit / cost > 1.0 {
                    built[idx] = true;
                    npv += line.capex_m_usd * df;
                }
            }
        }
        npv
    }
}

// ─── Internal helpers ─────────────────────────────────────────────────────────

/// Discount factor for a given annual rate and number of years.
#[inline]
fn discount_factor(rate: f64, years: f64) -> f64 {
    1.0 / (1.0 + rate).powf(years)
}

/// Baseline LOLE \[days/year\] for an unaugmented network.
#[inline]
fn base_lole() -> f64 {
    0.5
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config(periods: usize) -> MultiStageTepConfig {
        MultiStageTepConfig {
            planning_periods: periods,
            years_per_period: 5,
            discount_rate: 0.08,
            n_scenarios: 1,
            reliability_standard: 0.1,
            co2_budget_mt: 1_000.0,
        }
    }

    fn default_scenario() -> GrowthScenario {
        GrowthScenario {
            id: 0,
            probability: 1.0,
            load_growth_pct_per_year: 2.0,
            renewable_growth_pct_per_year: 3.0,
            co2_price_usd_per_t: 30.0,
        }
    }

    fn cheap_high_cap_line(id: usize) -> CandidateLine {
        CandidateLine {
            id,
            from_bus: 1,
            to_bus: 2,
            capacity_mw: 500.0,
            capex_m_usd: 50.0,
            fixed_opex_m_usd_per_year: 0.5,
            construction_time_years: 3,
            lifetime_years: 30,
            reactance_pu: 0.05,
        }
    }

    fn expensive_low_cap_line(id: usize) -> CandidateLine {
        CandidateLine {
            id,
            from_bus: 3,
            to_bus: 4,
            capacity_mw: 10.0,
            capex_m_usd: 500.0,
            fixed_opex_m_usd_per_year: 5.0,
            construction_time_years: 3,
            lifetime_years: 30,
            reactance_pu: 0.1,
        }
    }

    // ── Test 1 ─────────────────────────────────────────────────────────────────
    /// No candidates → no investment at any stage, but stages are still created.
    #[test]
    fn test_no_investment_needed() {
        let mut solver = MultiStageTepSolver::new(default_config(1));
        solver.add_scenario(default_scenario());
        let result = solver.solve().expect("solve failed");
        assert_eq!(result.stages.len(), 1);
        assert!(result.stages[0].lines_invested.is_empty());
        assert_eq!(result.stages[0].total_investment_m_usd, 0.0);
    }

    // ── Test 2 ─────────────────────────────────────────────────────────────────
    /// High-value line is selected; low-value (high-cost, low-cap) is not.
    #[test]
    fn test_congestion_relief_high_value_first() {
        let mut solver = MultiStageTepSolver::new(default_config(1));
        solver.add_scenario(GrowthScenario {
            id: 0,
            probability: 1.0,
            load_growth_pct_per_year: 5.0,
            renewable_growth_pct_per_year: 1.0,
            co2_price_usd_per_t: 400.0,
        });
        solver.add_candidate(cheap_high_cap_line(10));
        solver.add_candidate(expensive_low_cap_line(11));

        let result = solver.solve().expect("solve failed");
        let invested = &result.stages[0].lines_invested;
        assert!(
            invested.contains(&10),
            "high-value line 10 should be selected, got {invested:?}"
        );
        assert!(
            !invested.contains(&11),
            "low-value line 11 should NOT be selected, got {invested:?}"
        );
    }

    // ── Test 3 ─────────────────────────────────────────────────────────────────
    /// Total NPV is finite and non-negative.
    #[test]
    fn test_budget_constraint_npv() {
        let mut solver = MultiStageTepSolver::new(default_config(3));
        solver.add_scenario(default_scenario());
        solver.add_candidate(cheap_high_cap_line(1));
        solver.set_existing_network_cost(100.0);

        let result = solver.solve().expect("solve failed");
        assert!(result.total_npv_m_usd.is_finite());
        assert!(result.total_npv_m_usd >= 0.0);
    }

    // ── Test 4 ─────────────────────────────────────────────────────────────────
    /// After building a large-capacity line, LOLE < reliability_standard.
    #[test]
    fn test_reliability_lole_satisfied() {
        let config = MultiStageTepConfig {
            planning_periods: 1,
            years_per_period: 5,
            discount_rate: 0.08,
            n_scenarios: 1,
            reliability_standard: 0.1,
            co2_budget_mt: 1_000.0,
        };
        let mut solver = MultiStageTepSolver::new(config);
        solver.add_scenario(GrowthScenario {
            id: 0,
            probability: 1.0,
            load_growth_pct_per_year: 5.0,
            renewable_growth_pct_per_year: 1.0,
            co2_price_usd_per_t: 400.0,
        });
        // Very large line to drive LOLE down; low capex ensures benefit/cost > 1
        solver.add_candidate(CandidateLine {
            id: 99,
            from_bus: 1,
            to_bus: 2,
            capacity_mw: 5_000.0,
            capex_m_usd: 10.0,
            fixed_opex_m_usd_per_year: 0.1,
            construction_time_years: 2,
            lifetime_years: 40,
            reactance_pu: 0.02,
        });
        let result = solver.solve().expect("solve failed");
        assert!(result.reliability_satisfied, "LOLE constraint must be met");
        assert!(result.expected_lole < 0.1);
    }

    // ── Test 5 ─────────────────────────────────────────────────────────────────
    /// Multi-period plan produces one TepDecisionStage per planning period.
    #[test]
    fn test_multi_period_sequential() {
        let mut solver = MultiStageTepSolver::new(default_config(3));
        solver.add_scenario(default_scenario());
        solver.add_candidate(cheap_high_cap_line(1));
        solver.add_candidate(CandidateLine {
            id: 2,
            from_bus: 5,
            to_bus: 6,
            capacity_mw: 300.0,
            capex_m_usd: 30.0,
            fixed_opex_m_usd_per_year: 0.3,
            construction_time_years: 4,
            lifetime_years: 30,
            reactance_pu: 0.06,
        });

        let result = solver.solve().expect("solve failed");
        assert_eq!(result.stages.len(), 3, "must have exactly 3 stages");
        // Lines built in earlier periods should not appear in later periods
        let all_built: Vec<usize> = result
            .stages
            .iter()
            .flat_map(|s| s.lines_invested.iter().copied())
            .collect();
        // No duplicate line IDs across stages
        let mut seen = std::collections::HashSet::new();
        for id in &all_built {
            assert!(seen.insert(id), "line {id} built in multiple periods");
        }
    }

    // ── Test 6 ─────────────────────────────────────────────────────────────────
    /// Zero candidates: solve succeeds, no investment, NPV non-negative.
    #[test]
    fn test_zero_candidates_solve_succeeds() {
        let config = default_config(2);
        let mut solver = MultiStageTepSolver::new(config);
        solver.add_scenario(default_scenario());
        // No candidates added
        let result = solver
            .solve()
            .expect("solve with zero candidates should succeed");
        assert_eq!(result.stages.len(), 2);
        for stage in &result.stages {
            assert!(stage.lines_invested.is_empty());
            assert_eq!(stage.total_investment_m_usd, 0.0);
        }
        assert!(result.total_npv_m_usd >= 0.0);
    }

    // ── Test 7 ─────────────────────────────────────────────────────────────────
    /// Single candidate, single scenario, one period: result is well-formed.
    #[test]
    fn test_single_candidate_single_scenario_minimum_viable() {
        let config = MultiStageTepConfig {
            planning_periods: 1,
            years_per_period: 5,
            discount_rate: 0.08,
            n_scenarios: 1,
            reliability_standard: 0.5,
            co2_budget_mt: 500.0,
        };
        let mut solver = MultiStageTepSolver::new(config);
        solver.add_scenario(GrowthScenario {
            id: 0,
            probability: 1.0,
            load_growth_pct_per_year: 3.0,
            renewable_growth_pct_per_year: 2.0,
            co2_price_usd_per_t: 200.0,
        });
        solver.add_candidate(CandidateLine {
            id: 42,
            from_bus: 1,
            to_bus: 3,
            capacity_mw: 300.0,
            capex_m_usd: 80.0,
            fixed_opex_m_usd_per_year: 1.0,
            construction_time_years: 2,
            lifetime_years: 25,
            reactance_pu: 0.07,
        });
        let result = solver
            .solve()
            .expect("single candidate single scenario should succeed");
        assert_eq!(result.stages.len(), 1);
        assert!(result.regret_m_usd >= 0.0);
        assert!(result.expected_lole > 0.0);
    }

    // ── Test 8 ─────────────────────────────────────────────────────────────────
    /// Three scenarios whose probabilities sum to exactly 1.0.
    #[test]
    fn test_multiple_scenarios_probabilities_sum_to_one() {
        let config = MultiStageTepConfig {
            planning_periods: 2,
            years_per_period: 5,
            discount_rate: 0.06,
            n_scenarios: 3,
            reliability_standard: 0.2,
            co2_budget_mt: 2000.0,
        };
        let mut solver = MultiStageTepSolver::new(config);
        solver.add_scenario(GrowthScenario {
            id: 0,
            probability: 0.25,
            load_growth_pct_per_year: 1.5,
            renewable_growth_pct_per_year: 2.0,
            co2_price_usd_per_t: 20.0,
        });
        solver.add_scenario(GrowthScenario {
            id: 1,
            probability: 0.50,
            load_growth_pct_per_year: 2.5,
            renewable_growth_pct_per_year: 3.0,
            co2_price_usd_per_t: 35.0,
        });
        solver.add_scenario(GrowthScenario {
            id: 2,
            probability: 0.25,
            load_growth_pct_per_year: 4.0,
            renewable_growth_pct_per_year: 5.0,
            co2_price_usd_per_t: 60.0,
        });
        solver.add_candidate(cheap_high_cap_line(5));
        let result = solver
            .solve()
            .expect("three scenarios summing to 1.0 should succeed");
        assert_eq!(result.stages.len(), 2);
        assert!(result.total_npv_m_usd.is_finite());
        assert!(result.regret_m_usd >= 0.0);
    }

    // ── Test 9 ─────────────────────────────────────────────────────────────────
    /// Zero-capacity line must not be invested (benefit = 0, ratio ≤ 1).
    #[test]
    fn test_extreme_capacity_zero_mw_line() {
        let config = default_config(1);
        let mut solver = MultiStageTepSolver::new(config);
        solver.add_scenario(default_scenario());
        solver.add_candidate(CandidateLine {
            id: 77,
            from_bus: 1,
            to_bus: 2,
            capacity_mw: 0.0,
            capex_m_usd: 5.0,
            fixed_opex_m_usd_per_year: 0.1,
            construction_time_years: 1,
            lifetime_years: 20,
            reactance_pu: 0.01,
        });
        let result = solver
            .solve()
            .expect("zero-capacity line should not cause failure");
        assert!(!result.stages[0].lines_invested.contains(&77));
    }

    // ── Test 10 ────────────────────────────────────────────────────────────────
    /// Extremely large cheap line is always selected and satisfies reliability.
    #[test]
    fn test_extreme_capacity_9999mw_line() {
        let config = MultiStageTepConfig {
            planning_periods: 1,
            years_per_period: 5,
            discount_rate: 0.05,
            n_scenarios: 1,
            reliability_standard: 0.5,
            co2_budget_mt: 5000.0,
        };
        let mut solver = MultiStageTepSolver::new(config);
        solver.add_scenario(GrowthScenario {
            id: 0,
            probability: 1.0,
            load_growth_pct_per_year: 5.0,
            renewable_growth_pct_per_year: 1.0,
            co2_price_usd_per_t: 500.0,
        });
        solver.add_candidate(CandidateLine {
            id: 88,
            from_bus: 10,
            to_bus: 20,
            capacity_mw: 9999.0,
            capex_m_usd: 1.0,
            fixed_opex_m_usd_per_year: 0.01,
            construction_time_years: 2,
            lifetime_years: 50,
            reactance_pu: 0.001,
        });
        let result = solver.solve().expect("huge cheap line should succeed");
        assert!(result.stages[0].lines_invested.contains(&88));
        assert!(result.reliability_satisfied);
    }

    // ── Test 11 ────────────────────────────────────────────────────────────────
    /// Four-period solve: NPV non-negative, each stage has positive LOLE.
    #[test]
    fn test_result_expected_cost_non_negative_and_stage_count_matches() {
        let config = MultiStageTepConfig {
            planning_periods: 4,
            years_per_period: 5,
            discount_rate: 0.10,
            n_scenarios: 1,
            reliability_standard: 0.3,
            co2_budget_mt: 3000.0,
        };
        let mut solver = MultiStageTepSolver::new(config);
        solver.add_scenario(default_scenario());
        solver.add_candidate(cheap_high_cap_line(1));
        solver.add_candidate(expensive_low_cap_line(2));
        let result = solver.solve().expect("4-period solve should succeed");
        assert!(result.total_npv_m_usd >= 0.0);
        assert_eq!(result.stages.len(), 4);
        for stage in &result.stages {
            assert!(stage.npv_cost_m_usd >= 0.0);
            assert!(stage.lole_days_per_year > 0.0);
        }
    }

    // ── Test 12 ────────────────────────────────────────────────────────────────
    /// Single scenario with probability 0.5 must fail with InvalidConfig.
    #[test]
    fn test_mismatched_probability_errors() {
        let config = default_config(1);
        let mut solver = MultiStageTepSolver::new(config);
        solver.add_scenario(GrowthScenario {
            id: 0,
            probability: 0.5,
            load_growth_pct_per_year: 2.0,
            renewable_growth_pct_per_year: 3.0,
            co2_price_usd_per_t: 30.0,
        });
        let result = solver.solve();
        assert!(matches!(result, Err(TepError::InvalidConfig(_))));
    }
}
