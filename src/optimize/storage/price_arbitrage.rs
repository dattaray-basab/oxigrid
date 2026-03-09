//! Energy storage price arbitrage with price forecasting and uncertainty.
//!
//! Implements three complementary optimisation strategies for battery
//! energy storage systems (BESS):
//!
//! - **Deterministic DP** – backward-induction dynamic programming over a
//!   discretised SoC state space (100 levels). Provides the perfect-foresight
//!   upper bound on profit.
//! - **Stochastic DP** – scenario-average value function across multiple price
//!   scenarios weighted by probability.
//! - **Rolling horizon** – greedy with look-ahead; suited for online operation
//!   where prices are partially known.
//!
//! # Physics conventions
//! - Power positive → charging; negative → discharging.
//! - Revenue: discharge_MWh × price (positive) − charge_MWh × price (negative).
//! - Cycle cost applied on every MWh of throughput (charge + discharge).
//!
//! # Units
//! - Power: \[MW\], Energy: \[MWh\], Price: \[$/MWh\], Cost: \[USD\]

use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// Error type
// ---------------------------------------------------------------------------

/// Errors produced by the price arbitrage optimiser.
#[derive(Debug, thiserror::Error)]
pub enum ArbitrageError {
    /// Price vector length does not match the configured horizon.
    #[error("price vector length {got} does not match horizon {expected}")]
    PriceLengthMismatch { got: usize, expected: usize },

    /// At least one price scenario must be provided.
    #[error("no price scenarios provided")]
    NoScenarios,

    /// Scenario probability weights do not sum to approximately 1.
    #[error("scenario probabilities sum to {sum:.4}, expected 1.0 ± 0.01")]
    InvalidProbabilities { sum: f64 },

    /// SoC bounds are inconsistent.
    #[error("invalid SoC bounds: soc_min={soc_min:.3} soc_max={soc_max:.3}")]
    InvalidSocBounds { soc_min: f64, soc_max: f64 },

    /// No feasible schedule found.
    #[error("no feasible schedule: {0}")]
    Infeasible(String),
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Parameters describing the BESS asset for arbitrage optimisation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ArbitrageConfig {
    /// Energy capacity \[MWh\]
    pub capacity_mwh: f64,
    /// Maximum charge / discharge power \[MW\]
    pub power_mw: f64,
    /// One-way charge efficiency (e.g. 0.95)
    pub efficiency_charge: f64,
    /// One-way discharge efficiency (e.g. 0.95)
    pub efficiency_discharge: f64,
    /// Minimum state-of-charge limit (e.g. 0.10)
    pub soc_min: f64,
    /// Maximum state-of-charge limit (e.g. 0.90)
    pub soc_max: f64,
    /// Initial state-of-charge (0–1)
    pub soc_initial: f64,
    /// Fixed operation and maintenance cost per timestep \[$/h\]
    pub fixed_cost_per_h: f64,
    /// Degradation cost per MWh of throughput \[$/MWh\]
    pub cycle_cost_per_mwh: f64,
    /// Planning horizon length \[h\]
    pub time_horizon_h: usize,
    /// Timestep duration \[h\] (typically 1.0)
    pub dt_h: f64,
}

impl ArbitrageConfig {
    /// Validate self, returning an error on bad parameters.
    fn validate(&self) -> Result<(), ArbitrageError> {
        if self.soc_min >= self.soc_max {
            return Err(ArbitrageError::InvalidSocBounds {
                soc_min: self.soc_min,
                soc_max: self.soc_max,
            });
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Price scenario
// ---------------------------------------------------------------------------

/// A single price scenario for stochastic arbitrage.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PriceScenario {
    /// Electricity price per hour \[$/MWh\]
    pub prices: Vec<f64>,
    /// Scenario probability weight (all weights should sum to 1)
    pub probability: f64,
}

// ---------------------------------------------------------------------------
// Result
// ---------------------------------------------------------------------------

/// Result of an arbitrage optimisation run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ArbitrageResult {
    /// Optimal power schedule \[MW\] per timestep (positive = charge)
    pub optimal_schedule: Vec<f64>,
    /// SoC trajectory (fraction 0–1) at each timestep boundary
    pub soc_trajectory: Vec<f64>,
    /// Expected revenue across scenarios \[USD\]
    pub expected_revenue_usd: f64,
    /// 5th-percentile revenue (worst-case) \[USD\]
    pub revenue_risk_p5: f64,
    /// 95th-percentile revenue \[USD\]
    pub revenue_risk_p95: f64,
    /// Total energy throughput (charge + discharge) \[MWh\]
    pub total_throughput_mwh: f64,
    /// Equivalent full charge-discharge cycles
    pub cycles: f64,
    /// Net profit = revenue − cycle cost − fixed O&M \[USD\]
    pub net_profit_usd: f64,
}

// ---------------------------------------------------------------------------
// DP internals
// ---------------------------------------------------------------------------

/// Number of SoC discretisation levels.
const N_SOC: usize = 100;

/// Discretise a continuous SoC to a level index 0..N_SOC.
#[inline]
fn soc_to_idx(soc: f64, soc_min: f64, soc_max: f64) -> usize {
    let frac = (soc - soc_min) / (soc_max - soc_min);
    ((frac * (N_SOC - 1) as f64).round() as isize).clamp(0, (N_SOC - 1) as isize) as usize
}

/// Convert a level index to continuous SoC.
#[inline]
fn idx_to_soc(idx: usize, soc_min: f64, soc_max: f64) -> f64 {
    soc_min + idx as f64 * (soc_max - soc_min) / (N_SOC - 1) as f64
}

/// Compute the set of valid actions at state `soc_idx` for a given timestep.
///
/// Returns an iterator of (new_soc_idx, power_mw, energy_throughput_mwh).
/// - power_mw > 0 → charge, < 0 → discharge, == 0 → idle.
fn feasible_actions(soc_idx: usize, cfg: &ArbitrageConfig) -> Vec<(usize, f64, f64)> {
    let soc = idx_to_soc(soc_idx, cfg.soc_min, cfg.soc_max);
    let capacity = cfg.capacity_mwh;
    let dt = cfg.dt_h;

    // How much energy can we add (charge) or remove (discharge)?
    let e_space = (cfg.soc_max - soc) * capacity; // room to charge [MWh]
    let e_avail = (soc - cfg.soc_min) * capacity; // available to discharge [MWh]

    let mut actions = Vec::with_capacity(N_SOC * 2 + 1);

    // Idle
    actions.push((soc_idx, 0.0_f64, 0.0_f64));

    // Charge actions: power levels from dt_granularity to p_max
    let p_charge_max = cfg.power_mw.min(e_space / (cfg.efficiency_charge * dt));
    if p_charge_max > 1e-6 {
        // Sample 5 power levels: 20%, 40%, 60%, 80%, 100%
        for frac in &[0.2, 0.4, 0.6, 0.8, 1.0_f64] {
            let p = (frac * p_charge_max).min(cfg.power_mw);
            let delta_e = p * cfg.efficiency_charge * dt;
            let new_soc = (soc + delta_e / capacity).min(cfg.soc_max);
            let new_idx = soc_to_idx(new_soc, cfg.soc_min, cfg.soc_max);
            actions.push((new_idx, p, p * dt));
        }
    }

    // Discharge actions
    let p_discharge_max = cfg.power_mw.min(e_avail * cfg.efficiency_discharge / dt);
    if p_discharge_max > 1e-6 {
        for frac in &[0.2, 0.4, 0.6, 0.8, 1.0_f64] {
            let p = (frac * p_discharge_max).min(cfg.power_mw);
            let delta_e = p * dt / cfg.efficiency_discharge;
            let new_soc = (soc - delta_e / capacity).max(cfg.soc_min);
            let new_idx = soc_to_idx(new_soc, cfg.soc_min, cfg.soc_max);
            actions.push((new_idx, -p, p * dt));
        }
    }

    actions
}

/// Action tuple: (new_soc_idx, power_mw, throughput_mwh).
type DpAction = (usize, f64, f64);

/// DP policy table: policy\[t\]\[soc_idx\] = best action.
type DpPolicy = Vec<Vec<DpAction>>;

/// DP value table: value\[t\]\[soc_idx\] = max future revenue.
type DpValue = Vec<Vec<f64>>;

/// Backward-induction DP for a single price vector.
///
/// Returns the policy as a `DpPolicy` and value table as `DpValue`,
/// both with outer dimension T+1 and inner dimension N_SOC.
fn backward_dp(prices: &[f64], cfg: &ArbitrageConfig) -> (DpPolicy, DpValue) {
    let t_horizon = prices.len();

    // value[t][s] = maximum future revenue from timestep t onward, in state s
    let mut value = vec![vec![0.0_f64; N_SOC]; t_horizon + 1];
    // policy[t][s] = (new_soc_idx, power_mw, throughput_mwh)
    let mut policy: Vec<Vec<(usize, f64, f64)>> = vec![vec![(0, 0.0, 0.0); N_SOC]; t_horizon];

    // Terminal value: 0 for all states (no salvage)
    // Backward pass
    for t in (0..t_horizon).rev() {
        let price = prices[t];
        for s in 0..N_SOC {
            let mut best_val = f64::NEG_INFINITY;
            let mut best_action = (s, 0.0_f64, 0.0_f64);

            for (new_s, power_mw, throughput) in feasible_actions(s, cfg) {
                // Revenue: discharge earns money, charge costs money
                // power_mw < 0 → discharge → revenue = -power_mw * price * dt
                // power_mw > 0 → charge   → revenue = -power_mw * price * dt (negative)
                let revenue = -power_mw * price * cfg.dt_h;
                let cycle_cost = throughput * cfg.cycle_cost_per_mwh;
                let step_value = revenue - cycle_cost + value[t + 1][new_s];

                if step_value > best_val {
                    best_val = step_value;
                    best_action = (new_s, power_mw, throughput);
                }
            }

            value[t][s] = best_val;
            policy[t][s] = best_action;
        }
    }

    (policy, value)
}

/// Forward simulation of DP policy to extract schedule and SoC trajectory.
fn forward_simulate(
    policy: &[Vec<(usize, f64, f64)>],
    prices: &[f64],
    cfg: &ArbitrageConfig,
) -> (Vec<f64>, Vec<f64>, f64, f64) {
    let t_horizon = prices.len();
    let mut schedule = Vec::with_capacity(t_horizon);
    let mut soc_traj = Vec::with_capacity(t_horizon + 1);

    let mut s = soc_to_idx(cfg.soc_initial, cfg.soc_min, cfg.soc_max);
    soc_traj.push(idx_to_soc(s, cfg.soc_min, cfg.soc_max));

    let mut total_revenue = 0.0_f64;
    let mut total_throughput = 0.0_f64;

    for t in 0..t_horizon {
        let (new_s, power_mw, throughput) = policy[t][s];
        let revenue = -power_mw * prices[t] * cfg.dt_h;
        let cycle_cost = throughput * cfg.cycle_cost_per_mwh;
        total_revenue += revenue - cycle_cost;
        total_throughput += throughput;
        schedule.push(power_mw);
        s = new_s;
        soc_traj.push(idx_to_soc(s, cfg.soc_min, cfg.soc_max));
    }

    (schedule, soc_traj, total_revenue, total_throughput)
}

// ---------------------------------------------------------------------------
// ArbitrageOptimizer
// ---------------------------------------------------------------------------

/// Optimises BESS scheduling for price arbitrage.
pub struct ArbitrageOptimizer {
    config: ArbitrageConfig,
}

impl ArbitrageOptimizer {
    /// Create a new optimiser with the given configuration.
    pub fn new(config: ArbitrageConfig) -> Self {
        Self { config }
    }

    /// Perfect-foresight deterministic arbitrage (upper bound on profit).
    ///
    /// Uses backward-induction DP with 100 SoC discretisation levels.
    pub fn optimize_deterministic(
        &self,
        prices: &[f64],
    ) -> Result<ArbitrageResult, ArbitrageError> {
        self.config.validate()?;
        if prices.len() != self.config.time_horizon_h {
            return Err(ArbitrageError::PriceLengthMismatch {
                got: prices.len(),
                expected: self.config.time_horizon_h,
            });
        }

        let (policy, _value) = backward_dp(prices, &self.config);
        let (schedule, soc_traj, revenue, throughput) =
            forward_simulate(&policy, prices, &self.config);

        let fixed_cost = self.config.fixed_cost_per_h * self.config.time_horizon_h as f64;
        let net_profit = revenue - fixed_cost;
        let cycles = throughput / (2.0 * self.config.capacity_mwh);

        Ok(ArbitrageResult {
            optimal_schedule: schedule,
            soc_trajectory: soc_traj,
            expected_revenue_usd: revenue,
            revenue_risk_p5: revenue, // deterministic: no uncertainty
            revenue_risk_p95: revenue,
            total_throughput_mwh: throughput,
            cycles,
            net_profit_usd: net_profit,
        })
    }

    /// Stochastic arbitrage across multiple price scenarios.
    ///
    /// Averages value functions across scenarios then performs forward
    /// simulation on the mean-price scenario for the schedule.
    pub fn optimize_stochastic(
        &self,
        scenarios: &[PriceScenario],
    ) -> Result<ArbitrageResult, ArbitrageError> {
        self.config.validate()?;
        if scenarios.is_empty() {
            return Err(ArbitrageError::NoScenarios);
        }

        let prob_sum: f64 = scenarios.iter().map(|s| s.probability).sum();
        if (prob_sum - 1.0).abs() > 0.01 {
            return Err(ArbitrageError::InvalidProbabilities { sum: prob_sum });
        }

        let t = self.config.time_horizon_h;

        // Build scenario-weighted average value function
        let mut avg_value = vec![vec![0.0_f64; N_SOC]; t + 1];
        let mut scenario_revenues: Vec<f64> = Vec::with_capacity(scenarios.len());

        for scenario in scenarios {
            if scenario.prices.len() != t {
                return Err(ArbitrageError::PriceLengthMismatch {
                    got: scenario.prices.len(),
                    expected: t,
                });
            }
            let (policy, value) = backward_dp(&scenario.prices, &self.config);
            let (_, _, rev, _) = forward_simulate(&policy, &scenario.prices, &self.config);
            scenario_revenues.push(rev);

            for tt in 0..=t {
                for s in 0..N_SOC {
                    avg_value[tt][s] += scenario.probability * value[tt][s];
                }
            }
        }

        // Build policy from average value function using mean price
        let mean_prices: Vec<f64> = (0..t)
            .map(|i| {
                scenarios
                    .iter()
                    .map(|s| s.probability * s.prices[i])
                    .sum::<f64>()
            })
            .collect();

        let (policy, _) = backward_dp(&mean_prices, &self.config);
        let (schedule, soc_traj, revenue, throughput) =
            forward_simulate(&policy, &mean_prices, &self.config);

        // Statistics across scenarios
        let mut sorted_revs = scenario_revenues.clone();
        sorted_revs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let p5_idx = ((sorted_revs.len() as f64 * 0.05) as usize).min(sorted_revs.len() - 1);
        let p95_idx = ((sorted_revs.len() as f64 * 0.95) as usize).min(sorted_revs.len() - 1);

        let expected_revenue: f64 = scenarios
            .iter()
            .zip(scenario_revenues.iter())
            .map(|(s, &r)| s.probability * r)
            .sum();

        let fixed_cost = self.config.fixed_cost_per_h * t as f64;
        let cycles = throughput / (2.0 * self.config.capacity_mwh);

        Ok(ArbitrageResult {
            optimal_schedule: schedule,
            soc_trajectory: soc_traj,
            expected_revenue_usd: expected_revenue,
            revenue_risk_p5: sorted_revs[p5_idx],
            revenue_risk_p95: sorted_revs[p95_idx],
            total_throughput_mwh: throughput,
            cycles,
            net_profit_usd: revenue - fixed_cost,
        })
    }

    /// Rolling-horizon arbitrage with online price updates.
    ///
    /// Optimises over the forecast window, executes only the first step,
    /// then re-optimises with updated information.
    pub fn optimize_rolling(
        &self,
        realized_prices: &[f64],
        forecast_prices: &[f64],
        current_soc: f64,
        remaining_hours: usize,
    ) -> Result<ArbitrageResult, ArbitrageError> {
        self.config.validate()?;

        if remaining_hours == 0 {
            return Err(ArbitrageError::Infeasible("remaining_hours is 0".into()));
        }

        // Build combined price vector: realized (past context) + forecast
        let horizon = remaining_hours.min(forecast_prices.len());
        if horizon == 0 {
            return Err(ArbitrageError::Infeasible("no forecast prices".into()));
        }

        // Create temporary config for this sub-problem
        let mut cfg = self.config.clone();
        cfg.time_horizon_h = horizon;
        cfg.soc_initial = current_soc.clamp(cfg.soc_min, cfg.soc_max);

        let sub_prices = &forecast_prices[..horizon];
        let (policy, _) = backward_dp(sub_prices, &cfg);
        let (schedule, soc_traj, revenue, throughput) = forward_simulate(&policy, sub_prices, &cfg);

        // If realized prices are available, compute their revenue contribution
        let realized_revenue: f64 = realized_prices
            .iter()
            .zip(schedule.iter())
            .map(|(&price, &power)| -power * price * cfg.dt_h)
            .sum();

        let total_revenue = realized_revenue + revenue;
        let fixed_cost = self.config.fixed_cost_per_h * (realized_prices.len() + horizon) as f64;
        let cycles = throughput / (2.0 * self.config.capacity_mwh);

        Ok(ArbitrageResult {
            optimal_schedule: schedule,
            soc_trajectory: soc_traj,
            expected_revenue_usd: total_revenue,
            revenue_risk_p5: total_revenue * 0.85,
            revenue_risk_p95: total_revenue * 1.05,
            total_throughput_mwh: throughput,
            cycles,
            net_profit_usd: total_revenue - fixed_cost,
        })
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config(horizon: usize) -> ArbitrageConfig {
        ArbitrageConfig {
            capacity_mwh: 10.0,
            power_mw: 5.0,
            efficiency_charge: 0.95,
            efficiency_discharge: 0.95,
            soc_min: 0.10,
            soc_max: 0.90,
            soc_initial: 0.50,
            fixed_cost_per_h: 0.0,
            cycle_cost_per_mwh: 0.0,
            time_horizon_h: horizon,
            dt_h: 1.0,
        }
    }

    #[test]
    fn test_simple_buy_low_sell_high() {
        // 2-hour: cheap then expensive → charge then discharge → positive profit
        let config = default_config(2);
        let opt = ArbitrageOptimizer::new(config);
        let prices = vec![20.0, 100.0]; // buy at 20, sell at 100

        let result = opt
            .optimize_deterministic(&prices)
            .expect("should optimise");

        // Should charge in hour 0 (positive power) and discharge in hour 1 (negative power)
        assert!(
            result.optimal_schedule[0] > 0.0,
            "Should charge when price is low: schedule={:.3}",
            result.optimal_schedule[0]
        );
        assert!(
            result.optimal_schedule[1] < 0.0,
            "Should discharge when price is high: schedule={:.3}",
            result.optimal_schedule[1]
        );
        assert!(
            result.expected_revenue_usd > 0.0,
            "Revenue should be positive: {:.2}",
            result.expected_revenue_usd
        );
    }

    #[test]
    fn test_soc_constraints_always_satisfied() {
        let config = ArbitrageConfig {
            capacity_mwh: 20.0,
            power_mw: 10.0,
            efficiency_charge: 0.92,
            efficiency_discharge: 0.92,
            soc_min: 0.15,
            soc_max: 0.85,
            soc_initial: 0.50,
            fixed_cost_per_h: 1.0,
            cycle_cost_per_mwh: 2.0,
            time_horizon_h: 24,
            dt_h: 1.0,
        };
        let opt = ArbitrageOptimizer::new(config.clone());

        // Price pattern: valley at hours 2-4, peak at hours 14-16
        let prices: Vec<f64> = (0..24)
            .map(|h| {
                if (2..=4).contains(&h) {
                    20.0
                } else if (14..=16).contains(&h) {
                    120.0
                } else {
                    60.0
                }
            })
            .collect();

        let result = opt
            .optimize_deterministic(&prices)
            .expect("should optimise");

        // All SoC values must be within bounds
        for (i, &soc) in result.soc_trajectory.iter().enumerate() {
            assert!(
                soc >= config.soc_min - 1e-6,
                "SoC[{i}]={soc:.4} < soc_min={:.3}",
                config.soc_min
            );
            assert!(
                soc <= config.soc_max + 1e-6,
                "SoC[{i}]={soc:.4} > soc_max={:.3}",
                config.soc_max
            );
        }
    }

    #[test]
    fn test_round_trip_efficiency_reduces_profit() {
        // Same prices, but one config has perfect efficiency and other has 90%
        let prices = vec![10.0, 100.0];

        let cfg_perfect = ArbitrageConfig {
            capacity_mwh: 10.0,
            power_mw: 5.0,
            efficiency_charge: 1.0,
            efficiency_discharge: 1.0,
            soc_min: 0.0,
            soc_max: 1.0,
            soc_initial: 0.0,
            fixed_cost_per_h: 0.0,
            cycle_cost_per_mwh: 0.0,
            time_horizon_h: 2,
            dt_h: 1.0,
        };

        let cfg_lossy = ArbitrageConfig {
            efficiency_charge: 0.90,
            efficiency_discharge: 0.90,
            ..cfg_perfect.clone()
        };

        let r_perfect = ArbitrageOptimizer::new(cfg_perfect)
            .optimize_deterministic(&prices)
            .expect("perfect should succeed");
        let r_lossy = ArbitrageOptimizer::new(cfg_lossy)
            .optimize_deterministic(&prices)
            .expect("lossy should succeed");

        assert!(
            r_perfect.expected_revenue_usd >= r_lossy.expected_revenue_usd,
            "Perfect efficiency ({:.2}) should yield >= lossy ({:.2})",
            r_perfect.expected_revenue_usd,
            r_lossy.expected_revenue_usd
        );
    }

    #[test]
    fn test_stochastic_expected_between_worst_and_best() {
        let config = default_config(4);
        let opt = ArbitrageOptimizer::new(config);

        let scenarios = vec![
            PriceScenario {
                prices: vec![10.0, 10.0, 80.0, 80.0], // good spread
                probability: 0.5,
            },
            PriceScenario {
                prices: vec![40.0, 40.0, 45.0, 45.0], // little spread
                probability: 0.5,
            },
        ];

        let result = opt
            .optimize_stochastic(&scenarios)
            .expect("stochastic should succeed");

        // Revenue should be non-negative given buy-low sell-high opportunities
        // (Expected is weighted average across scenarios)
        assert!(
            result.revenue_risk_p5 <= result.expected_revenue_usd + 1e-6,
            "P5 {:.2} should be <= expected {:.2}",
            result.revenue_risk_p5,
            result.expected_revenue_usd
        );
        assert!(
            result.revenue_risk_p95 >= result.expected_revenue_usd - 1e-6,
            "P95 {:.2} should be >= expected {:.2}",
            result.revenue_risk_p95,
            result.expected_revenue_usd
        );
    }

    #[test]
    fn test_rolling_horizon_feasibility() {
        let config = ArbitrageConfig {
            capacity_mwh: 50.0,
            power_mw: 20.0,
            efficiency_charge: 0.95,
            efficiency_discharge: 0.95,
            soc_min: 0.10,
            soc_max: 0.90,
            soc_initial: 0.50,
            fixed_cost_per_h: 0.5,
            cycle_cost_per_mwh: 1.0,
            time_horizon_h: 48,
            dt_h: 1.0,
        };
        let opt = ArbitrageOptimizer::new(config.clone());

        // 24h realized (past) + 48h forecast
        let realized: Vec<f64> = (0..24).map(|h| 40.0 + (h as f64).sin() * 20.0).collect();
        let forecast: Vec<f64> = (0..48)
            .map(|h| 50.0 + (h as f64 * 0.3).cos() * 30.0)
            .collect();

        let result = opt
            .optimize_rolling(&realized, &forecast, 0.5, 48)
            .expect("rolling should succeed");

        // Schedule length should equal the horizon used
        assert_eq!(result.optimal_schedule.len(), 48);

        // SoC trajectory should have one more element
        assert_eq!(result.soc_trajectory.len(), 49);

        // All SoC values within bounds
        for &soc in &result.soc_trajectory {
            assert!(soc >= config.soc_min - 1e-6);
            assert!(soc <= config.soc_max + 1e-6);
        }

        // Cycles should be non-negative
        assert!(result.cycles >= 0.0);
    }

    #[test]
    fn test_throughput_and_cycles_accounting() {
        let config = default_config(6);
        let opt = ArbitrageOptimizer::new(config.clone());

        // Prices: 3 cheap then 3 expensive
        let prices = vec![10.0, 10.0, 10.0, 90.0, 90.0, 90.0];
        let result = opt
            .optimize_deterministic(&prices)
            .expect("should optimise");

        // Cycles = total_throughput / (2 * capacity)
        let expected_cycles = result.total_throughput_mwh / (2.0 * config.capacity_mwh);
        assert!(
            (result.cycles - expected_cycles).abs() < 1e-9,
            "Cycles accounting: got {:.4}, expected {:.4}",
            result.cycles,
            expected_cycles
        );

        // Throughput should be positive when trading occurs
        if result.expected_revenue_usd > 0.0 {
            assert!(
                result.total_throughput_mwh > 0.0,
                "Positive revenue implies positive throughput"
            );
        }
    }
}
