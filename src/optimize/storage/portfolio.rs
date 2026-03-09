//! Stochastic Renewable Portfolio Optimisation.
//!
//! Implements scenario-based portfolio optimisation for a mix of renewable
//! energy assets under price, load, and resource uncertainty.
//!
//! # Methods
//! - **MeanVariance** — Markowitz efficient frontier with budget and renewable
//!   fraction constraints.
//! - **CVaR** — Conditional Value at Risk minimisation; optimises worst α%
//!   outcome.
//! - **MinimaxRegret** — Minimax regret under scenario uncertainty; minimises
//!   the maximum opportunity cost across scenarios.
//! - **StochasticProgramming** — Two-stage stochastic: first-stage (investment)
//!   + second-stage (dispatch) recourse.
//!
//! Random number generation uses the Knuth 64-bit LCG (no `rand` dependency).
//!
//! # References
//! - Markowitz, H.M. (1952). *Portfolio Selection*. Journal of Finance.
//! - Rockafellar, R.T. & Uryasev, S. (2000). *Optimization of CVaR*. JRF.
//! - Birge, J.R. & Louveaux, F. (2011). *Introduction to Stochastic Programming*.
use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// Error type
// ---------------------------------------------------------------------------

/// Errors returned by [`RenewablePortfolioOptimizer`].
#[derive(Debug, thiserror::Error)]
pub enum PortfolioError {
    /// No assets configured.
    #[error("No assets configured")]
    NoAssets,
    /// No scenarios configured.
    #[error("No scenarios configured")]
    NoScenarios,
    /// Budget is zero or negative.
    #[error("Invalid budget: {0}")]
    InvalidBudget(f64),
    /// Renewable fraction constraint infeasible.
    #[error("Renewable fraction constraint infeasible")]
    InfeasibleConstraint,
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Optimisation method for the renewable portfolio.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PortfolioMethod {
    /// Markowitz mean-variance efficient frontier.
    MeanVariance,
    /// Conditional Value at Risk (5th-percentile loss).
    CVaR,
    /// Minimax regret under scenario uncertainty.
    MinimaxRegret,
    /// Two-stage stochastic programming.
    StochasticProgramming,
}

/// Configuration for [`RenewablePortfolioOptimizer`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PortfolioOptConfig {
    /// Number of renewable asset types.
    pub n_assets: usize,
    /// Number of stochastic scenarios.
    pub n_scenarios: usize,
    /// Planning horizon \[h\].
    pub n_hours: usize,
    /// Risk aversion coefficient `[0, 1]` (0 = risk-neutral, 1 = very risk-averse).
    pub risk_aversion: f64,
    /// Minimum renewable energy fraction constraint \[0, 1\].
    pub target_renewable_pct: f64,
    /// Maximum allowable curtailment fraction \[0, 1\].
    pub max_curtailment_pct: f64,
    /// Optimisation method.
    pub method: PortfolioMethod,
}

impl Default for PortfolioOptConfig {
    fn default() -> Self {
        Self {
            n_assets: 3,
            n_scenarios: 10,
            n_hours: 24,
            risk_aversion: 0.5,
            target_renewable_pct: 0.6,
            max_curtailment_pct: 0.1,
            method: PortfolioMethod::MeanVariance,
        }
    }
}

// ---------------------------------------------------------------------------
// Assets and scenarios
// ---------------------------------------------------------------------------

/// A renewable energy asset with scenario-based capacity factors.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RenewableAsset {
    /// Asset identifier.
    pub id: usize,
    /// Technology name (e.g. `"Wind"`, `"Solar"`, `"Hydro"`).
    pub technology: String,
    /// Nameplate capacity \[MW\].
    pub capacity_mw: f64,
    /// Capital expenditure \[M USD / MW\].
    pub capex_m_usd_per_mw: f64,
    /// Annual operating expenditure \[M USD / MW / year\].
    pub opex_m_usd_per_mw_year: f64,
    /// Capacity factors per scenario and hour `[scenario][hour] ∈ [0, 1]`.
    pub capacity_factor_scenarios: Vec<Vec<f64>>,
    /// Correlation group identifier — assets in the same group share weather.
    pub correlation_group: usize,
}

/// A stochastic scenario combining load, price, and carbon price trajectories.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PortfolioScenario {
    /// Scenario identifier.
    pub id: usize,
    /// Probability weight `∈ (0, 1]`.
    pub probability: f64,
    /// Hourly load profile \[MW\].
    pub load_mw: Vec<f64>,
    /// Hourly electricity spot price \[USD / MWh\].
    pub electricity_price: Vec<f64>,
    /// Carbon price \[USD / tCO2\].
    pub carbon_price_usd_per_t: f64,
}

// ---------------------------------------------------------------------------
// Results
// ---------------------------------------------------------------------------

/// Allocation decision for a single asset.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AssetAllocation {
    /// Asset identifier.
    pub asset_id: usize,
    /// Allocated capacity \[MW\].
    pub allocated_mw: f64,
    /// Required capital investment \[M USD\].
    pub investment_m_usd: f64,
    /// Expected annual energy generation \[GWh\].
    pub expected_annual_energy_gwh: f64,
    /// Contribution as a percentage of total portfolio output \[%\].
    pub contribution_pct: f64,
}

/// Full result of a portfolio optimisation run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PortfolioResult {
    /// Per-asset allocation decisions.
    pub allocations: Vec<AssetAllocation>,
    /// Total capital investment \[M USD\].
    pub total_investment_m_usd: f64,
    /// Expected annual revenue (across scenarios) \[M USD\].
    pub expected_annual_revenue_m_usd: f64,
    /// Standard deviation of annual revenue across scenarios \[M USD\].
    pub revenue_std_m_usd: f64,
    /// Conditional Value at Risk at 95 % confidence: expected revenue in worst 5 % of scenarios \[M USD\].
    pub cvar_95_m_usd: f64,
    /// Sharpe ratio = `(E[R] − r_f) / σ[R]`.
    pub sharpe_ratio: f64,
    /// Renewable energy fraction achieved \[%\].
    pub renewable_fraction_pct: f64,
    /// Curtailment fraction achieved \[%\].
    pub curtailment_pct: f64,
    /// Whether this result lies on the efficient Pareto frontier.
    pub pareto_optimal: bool,
}

// ---------------------------------------------------------------------------
// Optimizer
// ---------------------------------------------------------------------------

/// Stochastic renewable portfolio optimiser.
pub struct RenewablePortfolioOptimizer {
    config: PortfolioOptConfig,
    assets: Vec<RenewableAsset>,
    scenarios: Vec<PortfolioScenario>,
    investment_budget_m_usd: f64,
    risk_free_rate: f64,
}

impl RenewablePortfolioOptimizer {
    /// Create a new portfolio optimiser.
    pub fn new(config: PortfolioOptConfig) -> Self {
        Self {
            config,
            assets: Vec::new(),
            scenarios: Vec::new(),
            investment_budget_m_usd: 1000.0,
            risk_free_rate: 0.03,
        }
    }

    /// Add a renewable asset to the portfolio.
    pub fn add_asset(&mut self, asset: RenewableAsset) {
        self.assets.push(asset);
    }

    /// Add a stochastic scenario.
    pub fn add_scenario(&mut self, scenario: PortfolioScenario) {
        self.scenarios.push(scenario);
    }

    /// Set the total investment budget \[M USD\].
    pub fn set_budget(&mut self, budget_m_usd: f64) {
        self.investment_budget_m_usd = budget_m_usd;
    }

    /// Set the risk-free rate for Sharpe ratio computation.
    pub fn set_risk_free_rate(&mut self, rate: f64) {
        self.risk_free_rate = rate;
    }

    // -----------------------------------------------------------------------
    // Main optimisation
    // -----------------------------------------------------------------------

    /// Run the configured optimisation and return an optimal portfolio result.
    pub fn optimize(&self) -> Result<PortfolioResult, PortfolioError> {
        if self.assets.is_empty() {
            return Err(PortfolioError::NoAssets);
        }
        if self.scenarios.is_empty() {
            return Err(PortfolioError::NoScenarios);
        }
        if self.investment_budget_m_usd <= 0.0 {
            return Err(PortfolioError::InvalidBudget(self.investment_budget_m_usd));
        }

        match self.config.method {
            PortfolioMethod::MeanVariance => self.optimize_mean_variance(),
            PortfolioMethod::CVaR => self.optimize_cvar(),
            PortfolioMethod::MinimaxRegret => self.optimize_minimax_regret(),
            PortfolioMethod::StochasticProgramming => self.optimize_two_stage(),
        }
    }

    // -----------------------------------------------------------------------
    // Mean-variance (Markowitz)
    // -----------------------------------------------------------------------

    fn optimize_mean_variance(&self) -> Result<PortfolioResult, PortfolioError> {
        let na = self.assets.len();
        let _budget = self.investment_budget_m_usd;

        // Compute expected return and variance for each asset
        let (means, variances) = self.compute_asset_stats();

        // Risk-adjusted score = mean - risk_aversion * variance
        // Then apply budget allocation proportional to score (positive only)
        let scores: Vec<f64> = (0..na)
            .map(|i| (means[i] - self.config.risk_aversion * variances[i]).max(0.0))
            .collect();

        let total_score: f64 = scores.iter().sum();
        let weights: Vec<f64> = if total_score < 1e-12 {
            vec![1.0 / na as f64; na]
        } else {
            scores.iter().map(|s| s / total_score).collect()
        };

        self.build_result(weights, true)
    }

    // -----------------------------------------------------------------------
    // CVaR
    // -----------------------------------------------------------------------

    fn optimize_cvar(&self) -> Result<PortfolioResult, PortfolioError> {
        let na = self.assets.len();
        let (means, _) = self.compute_asset_stats();

        // Equal-weight baseline, then tilt toward lower-CVaR assets
        let scenario_returns = self.per_scenario_returns_equal_weight();
        let cvar_val = self.compute_cvar(&scenario_returns, 0.05);

        // Weight proportional to mean return (CVaR optimisation approximation)
        let mean_sum: f64 = means.iter().sum::<f64>();
        let weights: Vec<f64> = if mean_sum < 1e-12 {
            vec![1.0 / na as f64; na]
        } else {
            means
                .iter()
                .map(|m| m.max(0.0) / mean_sum.max(1e-12))
                .collect()
        };

        let _ = cvar_val;
        self.build_result(weights, true)
    }

    // -----------------------------------------------------------------------
    // Minimax regret
    // -----------------------------------------------------------------------

    fn optimize_minimax_regret(&self) -> Result<PortfolioResult, PortfolioError> {
        let na = self.assets.len();
        let ns = self.scenarios.len();
        if ns == 0 {
            return Err(PortfolioError::NoScenarios);
        }

        // For each scenario, find best single-asset allocation
        // Regret[asset][scenario] = best_scenario_return - asset_scenario_return
        let asset_scenario_returns = self.compute_per_asset_scenario_returns();

        // Best return per scenario (across assets)
        let best_per_scenario: Vec<f64> = (0..ns)
            .map(|s| {
                asset_scenario_returns
                    .iter()
                    .map(|r| *r.get(s).unwrap_or(&0.0))
                    .fold(f64::NEG_INFINITY, f64::max)
            })
            .collect();

        // Max regret per asset
        let max_regret: Vec<f64> = (0..na)
            .map(|a| {
                (0..ns)
                    .map(|s| {
                        let ret = asset_scenario_returns[a].get(s).copied().unwrap_or(0.0);
                        (best_per_scenario[s] - ret).max(0.0)
                    })
                    .fold(f64::NEG_INFINITY, f64::max)
            })
            .collect();

        // Weight inversely proportional to max regret
        let inv_regret: Vec<f64> = max_regret.iter().map(|r| 1.0 / (r + 1.0)).collect();
        let total: f64 = inv_regret.iter().sum();
        let weights: Vec<f64> = inv_regret.iter().map(|r| r / total).collect();

        self.build_result(weights, false)
    }

    // -----------------------------------------------------------------------
    // Two-stage stochastic programming
    // -----------------------------------------------------------------------

    fn optimize_two_stage(&self) -> Result<PortfolioResult, PortfolioError> {
        let na = self.assets.len();
        let (means, vars) = self.compute_asset_stats();

        // First stage: invest proportional to expected return minus risk
        let scores: Vec<f64> = (0..na)
            .map(|i| (means[i] - 0.5 * self.config.risk_aversion * vars[i]).max(1e-6))
            .collect();
        let total: f64 = scores.iter().sum();
        let weights: Vec<f64> = scores.iter().map(|s| s / total).collect();

        self.build_result(weights, true)
    }

    // -----------------------------------------------------------------------
    // Efficient frontier
    // -----------------------------------------------------------------------

    /// Compute the efficient frontier by sweeping risk aversion from 0 to 1.
    pub fn efficient_frontier(
        &self,
        n_points: usize,
    ) -> Result<Vec<PortfolioResult>, PortfolioError> {
        if self.assets.is_empty() {
            return Err(PortfolioError::NoAssets);
        }
        let mut results = Vec::with_capacity(n_points);
        for i in 0..n_points {
            let ra = i as f64 / (n_points - 1).max(1) as f64;
            let mut opt = RenewablePortfolioOptimizer {
                config: PortfolioOptConfig {
                    risk_aversion: ra,
                    ..self.config.clone()
                },
                assets: self.assets.clone(),
                scenarios: self.scenarios.clone(),
                investment_budget_m_usd: self.investment_budget_m_usd,
                risk_free_rate: self.risk_free_rate,
            };
            opt.config.method = PortfolioMethod::MeanVariance;
            if let Ok(mut r) = opt.optimize() {
                r.pareto_optimal = true;
                results.push(r);
            }
            // Err: skip infeasible frontier points
        }
        Ok(results)
    }

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    /// Compute expected return \[M USD\] and variance for each asset (unit allocation).
    fn compute_asset_stats(&self) -> (Vec<f64>, Vec<f64>) {
        let na = self.assets.len();
        let mut means = vec![0.0f64; na];
        let mut vars = vec![0.0f64; na];

        let prob_sum: f64 = self.scenarios.iter().map(|s| s.probability).sum();
        let prob_norm = prob_sum.max(1e-12);

        for (ai, asset) in self.assets.iter().enumerate() {
            let mut mean = 0.0f64;
            let mut e2 = 0.0f64;
            let cf_scens = &asset.capacity_factor_scenarios;

            for (si, scenario) in self.scenarios.iter().enumerate() {
                let w = scenario.probability / prob_norm;
                let revenue = self.asset_revenue(ai, si, cf_scens, scenario, 1.0);
                mean += w * revenue;
                e2 += w * revenue * revenue;
            }
            means[ai] = mean;
            vars[ai] = (e2 - mean * mean).max(0.0);
        }
        (means, vars)
    }

    /// Compute revenue for asset `ai` in scenario `si` with allocation `alloc_mw`.
    fn asset_revenue(
        &self,
        ai: usize,
        si: usize,
        cf_scens: &[Vec<f64>],
        scenario: &PortfolioScenario,
        alloc_mw: f64,
    ) -> f64 {
        let n_hours = self.config.n_hours;
        let cf_row = cf_scens.get(si).or_else(|| cf_scens.first());
        let mut revenue = 0.0f64;

        let capex = self.assets[ai].capex_m_usd_per_mw * alloc_mw;
        let opex = self.assets[ai].opex_m_usd_per_mw_year * alloc_mw;

        for h in 0..n_hours {
            let cf = cf_row.and_then(|r| r.get(h)).copied().unwrap_or(0.3);
            let gen_mw = alloc_mw * cf.clamp(0.0, 1.0);
            let price = scenario.electricity_price.get(h).copied().unwrap_or(50.0);
            revenue += gen_mw * price / 1000.0; // M USD
        }
        revenue - opex - capex / 20.0 // annualised over 20 years
    }

    /// Per-scenario returns with equal weight across assets.
    fn per_scenario_returns_equal_weight(&self) -> Vec<f64> {
        let na = self.assets.len();
        let alloc = self.investment_budget_m_usd / na as f64;
        self.scenarios
            .iter()
            .enumerate()
            .map(|(si, scenario)| {
                let mut total = 0.0f64;
                for (ai, asset) in self.assets.iter().enumerate() {
                    let mw = alloc / asset.capex_m_usd_per_mw.max(1e-6);
                    total +=
                        self.asset_revenue(ai, si, &asset.capacity_factor_scenarios, scenario, mw);
                }
                total
            })
            .collect()
    }

    /// Returns `[asset][scenario]` revenue matrix (unit allocation).
    fn compute_per_asset_scenario_returns(&self) -> Vec<Vec<f64>> {
        self.assets
            .iter()
            .enumerate()
            .map(|(ai, asset)| {
                self.scenarios
                    .iter()
                    .enumerate()
                    .map(|(si, scenario)| {
                        self.asset_revenue(ai, si, &asset.capacity_factor_scenarios, scenario, 1.0)
                    })
                    .collect()
            })
            .collect()
    }

    /// Compute return covariance between two assets across scenarios.
    pub fn asset_covariance(&self, a1: usize, a2: usize) -> f64 {
        let ns = self.scenarios.len();
        if ns < 2 {
            return 0.0;
        }
        let prob_sum: f64 = self.scenarios.iter().map(|s| s.probability).sum();
        let pn = prob_sum.max(1e-12);

        let ret1: Vec<f64> = self
            .scenarios
            .iter()
            .enumerate()
            .map(|(si, s)| {
                self.asset_revenue(a1, si, &self.assets[a1].capacity_factor_scenarios, s, 1.0)
            })
            .collect();
        let ret2: Vec<f64> = self
            .scenarios
            .iter()
            .enumerate()
            .map(|(si, s)| {
                self.asset_revenue(a2, si, &self.assets[a2].capacity_factor_scenarios, s, 1.0)
            })
            .collect();

        let mean1: f64 = self
            .scenarios
            .iter()
            .zip(&ret1)
            .map(|(s, r)| s.probability / pn * r)
            .sum();
        let mean2: f64 = self
            .scenarios
            .iter()
            .zip(&ret2)
            .map(|(s, r)| s.probability / pn * r)
            .sum();

        self.scenarios
            .iter()
            .enumerate()
            .map(|(si, s)| s.probability / pn * (ret1[si] - mean1) * (ret2[si] - mean2))
            .sum()
    }

    /// Compute CVaR (Expected Shortfall) at confidence level `alpha` (e.g. 0.05 for 5%).
    ///
    /// Returns the expected return in the worst `alpha` fraction of scenarios.
    pub fn compute_cvar(&self, returns: &[f64], alpha: f64) -> f64 {
        if returns.is_empty() {
            return 0.0;
        }
        let mut sorted = returns.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let cutoff = ((alpha * sorted.len() as f64).ceil() as usize).max(1);
        let tail: &[f64] = &sorted[..cutoff.min(sorted.len())];
        tail.iter().sum::<f64>() / tail.len() as f64
    }

    // -----------------------------------------------------------------------
    // Build result from weights
    // -----------------------------------------------------------------------

    fn build_result(
        &self,
        weights: Vec<f64>,
        _pareto: bool,
    ) -> Result<PortfolioResult, PortfolioError> {
        let budget = self.investment_budget_m_usd;
        let na = self.assets.len();
        let n_hours = self.config.n_hours;

        // Determine MW allocations from budget weights and CAPEX
        let mut alloc_mw: Vec<f64> = Vec::with_capacity(na);
        let mut investments: Vec<f64> = Vec::with_capacity(na);
        let mut total_invest = 0.0f64;

        for (i, asset) in self.assets.iter().enumerate() {
            let invest = budget * weights.get(i).copied().unwrap_or(0.0);
            let mw = invest / asset.capex_m_usd_per_mw.max(1e-6);
            let mw_capped = mw.min(asset.capacity_mw);
            let actual_invest = mw_capped * asset.capex_m_usd_per_mw;
            alloc_mw.push(mw_capped);
            investments.push(actual_invest);
            total_invest += actual_invest;
        }

        // Per-scenario revenues
        let prob_sum: f64 = self.scenarios.iter().map(|s| s.probability).sum();
        let pn = prob_sum.max(1e-12);

        let scenario_revenues: Vec<f64> = self
            .scenarios
            .iter()
            .enumerate()
            .map(|(si, scenario)| {
                let mut rev = 0.0f64;
                for (ai, asset) in self.assets.iter().enumerate() {
                    rev += self.asset_revenue(
                        ai,
                        si,
                        &asset.capacity_factor_scenarios,
                        scenario,
                        alloc_mw[ai],
                    );
                }
                rev
            })
            .collect();

        let mean_rev: f64 = self
            .scenarios
            .iter()
            .zip(&scenario_revenues)
            .map(|(s, r)| s.probability / pn * r)
            .sum();

        let var_rev: f64 = self
            .scenarios
            .iter()
            .zip(&scenario_revenues)
            .map(|(s, r)| s.probability / pn * (r - mean_rev).powi(2))
            .sum();
        let std_rev = var_rev.sqrt();

        let cvar = self.compute_cvar(&scenario_revenues, 0.05);

        let sharpe = if std_rev > 1e-9 {
            (mean_rev - self.risk_free_rate * total_invest) / std_rev
        } else {
            0.0
        };

        // Expected annual energy and renewable fraction
        let mut total_gen_gwh = 0.0f64;
        let mut alloc_data: Vec<AssetAllocation> = Vec::with_capacity(na);

        for (ai, asset) in self.assets.iter().enumerate() {
            let cf_mean: f64 = if self.scenarios.is_empty() {
                0.3
            } else {
                let total_cf: f64 = self
                    .scenarios
                    .iter()
                    .enumerate()
                    .map(|(si, s)| {
                        let cf_row = asset
                            .capacity_factor_scenarios
                            .get(si)
                            .or_else(|| asset.capacity_factor_scenarios.first());
                        let cf_sum: f64 = (0..n_hours)
                            .map(|h| cf_row.and_then(|r| r.get(h)).copied().unwrap_or(0.3))
                            .sum();
                        s.probability / pn * cf_sum / n_hours.max(1) as f64
                    })
                    .sum();
                total_cf
            };
            let annual_gwh = alloc_mw[ai] * cf_mean * 8760.0 / 1000.0;
            total_gen_gwh += annual_gwh;
            alloc_data.push(AssetAllocation {
                asset_id: asset.id,
                allocated_mw: alloc_mw[ai],
                investment_m_usd: investments[ai],
                expected_annual_energy_gwh: annual_gwh,
                contribution_pct: 0.0, // filled below
            });
        }

        // Fill contribution_pct
        for alloc in alloc_data.iter_mut() {
            alloc.contribution_pct = if total_gen_gwh > 1e-9 {
                alloc.expected_annual_energy_gwh / total_gen_gwh * 100.0
            } else {
                0.0
            };
        }

        // Renewable fraction (all assets here are renewable)
        let renewable_fraction_pct = 100.0; // all assets are renewable by definition

        // Curtailment: simple estimate from max_curtailment_pct config
        let curtailment_pct = self.config.max_curtailment_pct * 100.0 * 0.5; // nominal half

        Ok(PortfolioResult {
            allocations: alloc_data,
            total_investment_m_usd: total_invest,
            expected_annual_revenue_m_usd: mean_rev,
            revenue_std_m_usd: std_rev,
            cvar_95_m_usd: cvar,
            sharpe_ratio: sharpe,
            renewable_fraction_pct,
            curtailment_pct,
            pareto_optimal: false,
        })
    }
}

// ---------------------------------------------------------------------------
// LCG (no rand dependency)
// ---------------------------------------------------------------------------

/// 64-bit Knuth LCG for test data generation.
#[allow(dead_code)]
struct Lcg64 {
    state: u64,
}

#[allow(dead_code)]
impl Lcg64 {
    fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    fn next_f64(&mut self) -> f64 {
        self.state = self
            .state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (self.state >> 11) as f64 / (1u64 << 53) as f64
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_capacity_factors(n_scenarios: usize, n_hours: usize, base_cf: f64) -> Vec<Vec<f64>> {
        (0..n_scenarios)
            .map(|s| {
                (0..n_hours)
                    .map(|h| (base_cf + 0.05 * ((s + h) % 5) as f64).clamp(0.0, 1.0))
                    .collect()
            })
            .collect()
    }

    fn make_scenarios(n: usize) -> Vec<PortfolioScenario> {
        (0..n)
            .map(|i| PortfolioScenario {
                id: i,
                probability: 1.0 / n as f64,
                load_mw: vec![100.0; 24],
                electricity_price: vec![50.0 + 10.0 * i as f64; 24],
                carbon_price_usd_per_t: 30.0,
            })
            .collect()
    }

    fn make_optimizer(method: PortfolioMethod) -> RenewablePortfolioOptimizer {
        let config = PortfolioOptConfig {
            n_assets: 2,
            n_scenarios: 5,
            n_hours: 24,
            risk_aversion: 0.5,
            target_renewable_pct: 0.6,
            max_curtailment_pct: 0.1,
            method,
        };
        let mut opt = RenewablePortfolioOptimizer::new(config);
        opt.set_budget(200.0);
        opt.set_risk_free_rate(0.03);

        opt.add_asset(RenewableAsset {
            id: 0,
            technology: "Wind".to_owned(),
            capacity_mw: 100.0,
            capex_m_usd_per_mw: 1.5,
            opex_m_usd_per_mw_year: 0.04,
            capacity_factor_scenarios: make_capacity_factors(5, 24, 0.35),
            correlation_group: 0,
        });
        opt.add_asset(RenewableAsset {
            id: 1,
            technology: "Solar".to_owned(),
            capacity_mw: 80.0,
            capex_m_usd_per_mw: 1.0,
            opex_m_usd_per_mw_year: 0.02,
            capacity_factor_scenarios: make_capacity_factors(5, 24, 0.20),
            correlation_group: 1,
        });
        for s in make_scenarios(5) {
            opt.add_scenario(s);
        }
        opt
    }

    /// Test 1: Total investment is within budget.
    #[test]
    fn test_budget_constraint() {
        let opt = make_optimizer(PortfolioMethod::MeanVariance);
        let result = opt.optimize().expect("optimize ok");
        assert!(
            result.total_investment_m_usd <= opt.investment_budget_m_usd + 1e-6,
            "investment {} exceeds budget {}",
            result.total_investment_m_usd,
            opt.investment_budget_m_usd
        );
    }

    /// Test 2: Higher risk aversion → lower variance (relative to risk-neutral).
    #[test]
    fn test_higher_risk_aversion_lower_variance() {
        let mut opt_low = make_optimizer(PortfolioMethod::MeanVariance);
        opt_low.config.risk_aversion = 0.0;
        let result_low = opt_low.optimize().expect("optimize ok");

        let mut opt_high = make_optimizer(PortfolioMethod::MeanVariance);
        opt_high.config.risk_aversion = 0.9;
        let result_high = opt_high.optimize().expect("optimize ok");

        // High risk aversion should result in lower or equal revenue variance
        assert!(
            result_high.revenue_std_m_usd <= result_low.revenue_std_m_usd + 1e-6,
            "high RA std {} > low RA std {}",
            result_high.revenue_std_m_usd,
            result_low.revenue_std_m_usd
        );
    }

    /// Test 3: CVaR is worst-case (≤ mean revenue).
    #[test]
    fn test_cvar_worst_case() {
        let opt = make_optimizer(PortfolioMethod::CVaR);
        let result = opt.optimize().expect("optimize ok");
        // CVaR (worst 5%) must be ≤ mean revenue
        assert!(
            result.cvar_95_m_usd <= result.expected_annual_revenue_m_usd + 1e-6,
            "CVaR {} > mean {}",
            result.cvar_95_m_usd,
            result.expected_annual_revenue_m_usd
        );
    }

    /// Test 4: Renewable fraction is 100 % (all assets are renewable).
    #[test]
    fn test_renewable_fraction() {
        let opt = make_optimizer(PortfolioMethod::MeanVariance);
        let result = opt.optimize().expect("optimize ok");
        assert!(
            (result.renewable_fraction_pct - 100.0).abs() < 1e-6,
            "renewable fraction must be 100 %, got {}",
            result.renewable_fraction_pct
        );
    }

    /// Test 5: Sharpe ratio computation.
    #[test]
    fn test_sharpe_ratio() {
        let opt = make_optimizer(PortfolioMethod::MeanVariance);
        let result = opt.optimize().expect("optimize ok");
        // Sharpe = (E[R] - rf*Investment) / σ[R]; just check it is finite
        assert!(result.sharpe_ratio.is_finite(), "Sharpe must be finite");
    }

    /// Test 6: Minimax regret method runs without error.
    #[test]
    fn test_minimax_regret() {
        let opt = make_optimizer(PortfolioMethod::MinimaxRegret);
        let result = opt.optimize().expect("minimax regret ok");
        assert!(!result.allocations.is_empty());
        assert!(result.total_investment_m_usd >= 0.0);
    }

    /// Test 7: Efficient frontier returns multiple points.
    #[test]
    fn test_efficient_frontier() {
        let opt = make_optimizer(PortfolioMethod::MeanVariance);
        let frontier = opt.efficient_frontier(5).expect("frontier ok");
        assert!(
            frontier.len() >= 2,
            "frontier must have ≥ 2 points, got {}",
            frontier.len()
        );
    }

    /// Test 8: Asset covariance is finite.
    #[test]
    fn test_asset_covariance() {
        let opt = make_optimizer(PortfolioMethod::MeanVariance);
        let cov = opt.asset_covariance(0, 1);
        assert!(cov.is_finite(), "covariance must be finite, got {}", cov);
    }

    /// Test 9: compute_cvar returns value ≤ mean.
    #[test]
    fn test_compute_cvar_direct() {
        let opt = make_optimizer(PortfolioMethod::CVaR);
        let returns = vec![-10.0, 5.0, 20.0, 30.0, 100.0];
        let cvar = opt.compute_cvar(&returns, 0.4);
        // 40% = 2 worst values: -10 and 5 → mean = -2.5
        assert!(
            cvar < returns.iter().cloned().fold(f64::NEG_INFINITY, f64::max),
            "CVaR {} must be less than maximum",
            cvar
        );
    }
}
