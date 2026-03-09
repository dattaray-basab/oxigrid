//! Grid asset health modelling and risk-based maintenance optimisation.
//!
//! Implements the bathtub curve failure rate model, remaining useful life estimation,
//! health index aggregation, and a risk-prioritised maintenance scheduler.
//!
//! # References
//! - CIGRE TB 858, "Asset Management Frameworks for Transmission Systems" (2021)
//! - IEEE Std C57.140, "Guide for the Evaluation and Reconditioning of Liquid Immersed Power Transformers"
//! - Nelson, "Accelerated Testing", Wiley 1990 (Weibull / bathtub curve theory)

use crate::error::{OxiGridError, Result};

// ─────────────────────────────────────────────────────────────────────────────
// Asset Types
// ─────────────────────────────────────────────────────────────────────────────

/// Classification of power system assets for maintenance modelling.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AssetType {
    /// Power transformer (distribution or transmission)
    Transformer,
    /// Overhead transmission or distribution line
    OverheadLine,
    /// Underground XLPE or paper-insulated cable
    UndergroundCable,
    /// Air-blast, SF6, or vacuum circuit breaker
    CircuitBreaker,
    /// Shunt capacitor bank
    Capacitor,
    /// Shunt or series reactor
    Reactor,
    /// Synchronous or inverter-based generator
    Generator,
}

impl AssetType {
    /// Nominal design lifetime (years) for this asset class (typical industry values).
    pub fn nominal_lifetime_years(&self) -> f64 {
        match self {
            AssetType::Transformer => 40.0,
            AssetType::OverheadLine => 50.0,
            AssetType::UndergroundCable => 35.0,
            AssetType::CircuitBreaker => 30.0,
            AssetType::Capacitor => 20.0,
            AssetType::Reactor => 35.0,
            AssetType::Generator => 30.0,
        }
    }

    /// Base failure rate (failures/year) during the constant-hazard (random) phase.
    pub fn base_failure_rate(&self) -> f64 {
        match self {
            AssetType::Transformer => 0.005,
            AssetType::OverheadLine => 0.02,
            AssetType::UndergroundCable => 0.01,
            AssetType::CircuitBreaker => 0.015,
            AssetType::Capacitor => 0.03,
            AssetType::Reactor => 0.008,
            AssetType::Generator => 0.04,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Asset Health Model
// ─────────────────────────────────────────────────────────────────────────────

/// Asset health record and failure-rate model.
///
/// The failure rate follows the classic bathtub curve:
/// ```text
/// λ(t) = λ_early · exp(−k_early · t)   [infant mortality]
///       + λ_constant                    [random failures]
///       + λ_wear  · exp(k_wear · max(0, t − t_wear))  [wear-out]
/// ```
#[derive(Debug, Clone)]
pub struct AssetHealth {
    /// Unique asset identifier
    pub asset_id: usize,
    /// Asset classification
    pub asset_type: AssetType,
    /// Calendar year the asset was installed
    pub installation_year: u32,
    /// Design lifetime (years)
    pub design_lifetime_years: u32,
    /// Current age (fractional years)
    pub current_age_years: f64,
    /// Aggregate health index (0 = failed, 1 = new condition)
    pub health_index: f64,
    /// Current failure rate (failures/year) — may be overridden from field data
    pub failure_rate_per_year: f64,
    /// Time since last maintenance (years)
    pub last_maintenance: f64,
    /// Number of failures recorded over the asset's service life
    pub n_failures_lifetime: usize,
    /// Condition scores from inspection (0–1 each); may be empty
    pub condition_scores: Vec<f64>,
}

/// Bathtub curve parameters for failure rate modelling.
#[derive(Debug, Clone)]
struct BathtubParams {
    /// Infant-mortality peak rate (failures/year at t=0)
    lambda_early: f64,
    /// Decay constant for infant-mortality phase (1/year)
    k_early: f64,
    /// Constant (random) failure rate (failures/year)
    lambda_constant: f64,
    /// Wear-out onset rate (failures/year at t = t_wear)
    lambda_wear: f64,
    /// Growth constant for wear-out phase (1/year)
    k_wear: f64,
    /// Age at which wear-out begins (years)
    t_wear: f64,
}

impl AssetHealth {
    /// Construct a new asset health record from essential parameters.
    pub fn new(
        asset_id: usize,
        asset_type: AssetType,
        installation_year: u32,
        current_age_years: f64,
        health_index: f64,
    ) -> Self {
        let design_lifetime = asset_type.nominal_lifetime_years() as u32;
        let failure_rate = asset_type.base_failure_rate();
        Self {
            asset_id,
            asset_type,
            installation_year,
            design_lifetime_years: design_lifetime,
            current_age_years: current_age_years.max(0.0),
            health_index: health_index.clamp(0.0, 1.0),
            failure_rate_per_year: failure_rate,
            last_maintenance: current_age_years,
            n_failures_lifetime: 0,
            condition_scores: Vec::new(),
        }
    }

    /// Compute the instantaneous failure rate using the bathtub curve model.
    ///
    /// λ(t) = λ_early·e^{−k_early·t} + λ_constant + λ_wear·e^{k_wear·(t−t_wear)}
    pub fn failure_rate(&self) -> f64 {
        let p = self.bathtub_params();
        let t = self.current_age_years;
        let infant = p.lambda_early * (-p.k_early * t).exp();
        let constant = p.lambda_constant;
        let wearout = p.lambda_wear * (p.k_wear * (t - p.t_wear).max(0.0)).exp();
        // Scale by health index degradation factor: poor health → higher failure rate
        let health_factor = 1.0 + (1.0 - self.health_index.clamp(0.0, 1.0)) * 2.0;
        (infant + constant + wearout) * health_factor
    }

    /// Estimated remaining useful life (years).
    ///
    /// RUL = (design_lifetime − current_age) / degradation_factor
    /// where degradation_factor accounts for accumulated damage via the health index.
    pub fn remaining_useful_life(&self) -> f64 {
        let design = self.design_lifetime_years as f64;
        let age = self.current_age_years;
        if age >= design {
            return 0.0;
        }
        // Degradation factor: HI=1 → factor=1, HI=0 → factor=3 (worn-out asset degrades faster)
        let degradation_factor = 1.0 + (1.0 - self.health_index.clamp(0.0, 1.0)) * 2.0;
        let rul = (design - age) / degradation_factor;
        rul.max(0.0)
    }

    /// Compute the weighted health index from condition factor scores.
    ///
    /// HI = Σ(weight_i × score_i) / Σ weight_i
    ///
    /// Returns 1.0 (new) if either slice is empty or weights sum to zero.
    pub fn compute_health_index(condition_scores: &[f64], weights: &[f64]) -> f64 {
        if condition_scores.is_empty() || weights.is_empty() {
            return 1.0;
        }
        let n = condition_scores.len().min(weights.len());
        let sum_w: f64 = weights[..n].iter().sum();
        if sum_w < 1e-12 {
            return 1.0;
        }
        let weighted_sum: f64 = condition_scores[..n]
            .iter()
            .zip(weights[..n].iter())
            .map(|(s, w)| s.clamp(0.0, 1.0) * w)
            .sum();
        (weighted_sum / sum_w).clamp(0.0, 1.0)
    }

    /// Probability that the asset fails within the next `n_years` years.
    ///
    /// Uses the exponential (Poisson) approximation:
    /// P(T < t + n | T > t) = 1 − exp(−λ · n)
    /// where λ is the current instantaneous failure rate.
    pub fn failure_probability(&self, n_years: f64) -> f64 {
        if n_years <= 0.0 {
            return 0.0;
        }
        let lambda = self.failure_rate();
        (1.0 - (-lambda * n_years).exp()).clamp(0.0, 1.0)
    }

    /// Private: derive bathtub curve parameters from asset type and age.
    fn bathtub_params(&self) -> BathtubParams {
        let base = self.asset_type.base_failure_rate();
        let life = self.design_lifetime_years as f64;
        BathtubParams {
            lambda_early: base * 3.0, // infant mortality: 3× base at t=0
            k_early: 0.5,             // decays quickly in first ~2 years
            lambda_constant: base,
            lambda_wear: base * 4.0, // wear-out peak: 4× base
            k_wear: 0.3,             // accelerates gradually
            t_wear: life * 0.7,      // wear-out begins at 70% of design life
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Maintenance Strategy and Actions
// ─────────────────────────────────────────────────────────────────────────────

/// Maintenance decision strategy.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MaintenanceStrategy {
    /// Corrective maintenance: repair only after failure (run-to-failure).
    Corrective,
    /// Time-based preventive maintenance at a fixed interval (years).
    Preventive { interval_years: f64 },
    /// Condition-based maintenance: triggered when health index falls below a threshold.
    ConditionBased,
    /// Predictive maintenance: triggered when failure probability within 1 year exceeds threshold.
    Predictive { threshold: f64 },
}

/// Classification of maintenance intervention.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActionType {
    /// Visual inspection and testing (no physical work)
    Inspection,
    /// Cleaning, lubrication, minor adjustments
    MinorMaintenance,
    /// Detailed overhaul (rewinding, insulation replacement, etc.)
    MajorOverhaul,
    /// Complete asset replacement
    Replacement,
}

impl ActionType {
    /// Fractional health-index improvement expected from this action type.
    pub fn health_improvement(&self) -> f64 {
        match self {
            ActionType::Inspection => 0.0, // inspection alone doesn't restore health
            ActionType::MinorMaintenance => 0.05,
            ActionType::MajorOverhaul => 0.30,
            ActionType::Replacement => 1.0, // restored to as-new
        }
    }

    /// Typical duration (days) for each action type.
    pub fn typical_duration_days(&self) -> f64 {
        match self {
            ActionType::Inspection => 0.5,
            ActionType::MinorMaintenance => 1.0,
            ActionType::MajorOverhaul => 5.0,
            ActionType::Replacement => 14.0,
        }
    }
}

/// A planned maintenance action.
#[derive(Debug, Clone)]
pub struct MaintenanceAction {
    /// Asset this action applies to
    pub asset_id: usize,
    /// Type of intervention
    pub action_type: ActionType,
    /// Planned execution year (fractional, e.g. 2026.5 = mid-2026)
    pub scheduled_year: f64,
    /// Estimated direct cost (USD or same currency unit)
    pub cost: f64,
    /// Expected improvement to health index (p.u.)
    pub health_improvement: f64,
    /// Expected outage duration for this action (days)
    pub duration_days: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Maintenance Optimiser
// ─────────────────────────────────────────────────────────────────────────────

/// Risk-based maintenance optimiser for a fleet of assets.
pub struct MaintenanceOptimizer {
    /// Fleet of assets under management
    pub assets: Vec<AssetHealth>,
    /// Maintenance decision strategy
    pub strategy: MaintenanceStrategy,
    /// Planning horizon (years)
    pub planning_horizon_years: usize,
    /// Annual maintenance budget (USD)
    pub annual_budget: f64,
}

/// Output of the maintenance optimisation: scheduled actions and KPIs.
#[derive(Debug, Clone)]
pub struct MaintenanceSchedule {
    /// Prioritised and scheduled maintenance actions within budget
    pub actions: Vec<MaintenanceAction>,
    /// Total cost of all scheduled actions
    pub total_cost: f64,
    /// Estimated post-maintenance fleet-level SAIFI (weighted by failure rate reduction)
    pub expected_reliability: f64,
    /// Fraction of initial risk mitigated by the planned actions (0–1)
    pub risk_reduction: f64,
    /// Asset IDs whose maintenance was deferred due to budget constraints
    pub deferred_actions: Vec<usize>,
}

impl MaintenanceOptimizer {
    /// Create a new optimiser.
    pub fn new(
        assets: Vec<AssetHealth>,
        strategy: MaintenanceStrategy,
        horizon: usize,
        budget: f64,
    ) -> Self {
        Self {
            assets,
            strategy,
            planning_horizon_years: horizon,
            annual_budget: budget,
        }
    }

    /// Optimise the maintenance schedule using risk-based prioritisation.
    ///
    /// Algorithm:
    /// 1. For each asset, compute risk score = failure_rate × consequence_cost.
    /// 2. Rank assets by risk score (descending).
    /// 3. For each year in the horizon, select maintenance actions within the annual budget.
    /// 4. Schedule the most appropriate action type for each asset based on health / strategy.
    pub fn optimize(&self) -> Result<MaintenanceSchedule> {
        if self.planning_horizon_years == 0 {
            return Err(OxiGridError::InvalidParameter(
                "Planning horizon must be ≥ 1 year".to_string(),
            ));
        }
        if self.annual_budget < 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "Annual budget must be non-negative".to_string(),
            ));
        }

        let total_budget = self.annual_budget * self.planning_horizon_years as f64;

        // Compute risk score for each asset: λ × consequence (proxy from health degradation)
        let mut risk_scores: Vec<(usize, f64)> = self
            .assets
            .iter()
            .enumerate()
            .map(|(i, a)| {
                let consequence = (1.0 - a.health_index) * 1_000_000.0 + 10_000.0;
                let risk = a.failure_rate() * consequence;
                (i, risk)
            })
            .collect();

        // Sort by risk descending
        risk_scores.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        let initial_total_risk: f64 = risk_scores.iter().map(|(_, r)| r).sum();

        let mut scheduled = Vec::new();
        let mut deferred = Vec::new();
        let mut spent = 0.0_f64;
        let mut mitigated_risk = 0.0_f64;

        for (asset_idx, risk) in &risk_scores {
            let asset = &self.assets[*asset_idx];

            // Determine appropriate action type based on strategy and health
            let action_type = self.select_action_type(asset);

            // Estimate action cost based on asset type and action severity
            let cost = self.estimate_cost(asset, action_type);

            // Check if this fits within budget
            if spent + cost > total_budget {
                deferred.push(asset.asset_id);
                continue;
            }

            // Schedule in the optimal year (earliest year where risk is highest)
            let scheduled_year = self.select_scheduled_year(asset, action_type);

            let health_imp = action_type.health_improvement();
            let duration = action_type.typical_duration_days();

            scheduled.push(MaintenanceAction {
                asset_id: asset.asset_id,
                action_type,
                scheduled_year,
                cost,
                health_improvement: health_imp,
                duration_days: duration,
            });

            spent += cost;
            mitigated_risk += risk;
        }

        // Compute post-maintenance expected fleet SAIFI (normalised, not per-customer)
        let n_assets = self.assets.len() as f64;
        let pre_maintenance_saifi: f64 =
            self.assets.iter().map(|a| a.failure_rate()).sum::<f64>() / n_assets.max(1.0);
        let reduction_fraction = if initial_total_risk > 0.0 {
            mitigated_risk / initial_total_risk
        } else {
            0.0
        };
        // Expected SAIFI reduced proportionally to risk reduction (simplified)
        let expected_saifi = pre_maintenance_saifi * (1.0 - reduction_fraction * 0.6).max(0.0);

        Ok(MaintenanceSchedule {
            actions: scheduled,
            total_cost: spent,
            expected_reliability: expected_saifi,
            risk_reduction: reduction_fraction.clamp(0.0, 1.0),
            deferred_actions: deferred,
        })
    }

    /// Compute the cost-risk tradeoff curve over a range of budget levels.
    ///
    /// Returns a vector of `(budget, risk_reduction_fraction)` pairs,
    /// enabling decision-makers to visualise the marginal benefit of additional spending.
    pub fn risk_cost_curve(&self, budget_range: &[f64]) -> Vec<(f64, f64)> {
        budget_range
            .iter()
            .filter_map(|&budget| {
                if budget < 0.0 {
                    return None;
                }
                let temporary_optimizer = MaintenanceOptimizer::new(
                    self.assets.clone(),
                    self.strategy,
                    self.planning_horizon_years,
                    budget,
                );
                temporary_optimizer
                    .optimize()
                    .ok()
                    .map(|sched| (budget, sched.risk_reduction))
            })
            .collect()
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    /// Determine the appropriate action type for an asset given the strategy.
    fn select_action_type(&self, asset: &AssetHealth) -> ActionType {
        match self.strategy {
            MaintenanceStrategy::Corrective => {
                // Only replace after failure (high failure probability → replacement)
                if asset.failure_probability(1.0) > 0.5 {
                    ActionType::Replacement
                } else {
                    ActionType::Inspection
                }
            }
            MaintenanceStrategy::Preventive { interval_years } => {
                // Based on time since last maintenance
                if asset.last_maintenance >= interval_years * 2.0 {
                    ActionType::MajorOverhaul
                } else if asset.last_maintenance >= interval_years {
                    ActionType::MinorMaintenance
                } else {
                    ActionType::Inspection
                }
            }
            MaintenanceStrategy::ConditionBased => {
                // Based on health index thresholds
                if asset.health_index < 0.3 {
                    ActionType::Replacement
                } else if asset.health_index < 0.6 {
                    ActionType::MajorOverhaul
                } else if asset.health_index < 0.85 {
                    ActionType::MinorMaintenance
                } else {
                    ActionType::Inspection
                }
            }
            MaintenanceStrategy::Predictive { threshold } => {
                // Based on failure probability within next year
                let fp = asset.failure_probability(1.0);
                if fp > threshold * 2.0 {
                    ActionType::Replacement
                } else if fp > threshold {
                    ActionType::MajorOverhaul
                } else if fp > threshold * 0.5 {
                    ActionType::MinorMaintenance
                } else {
                    ActionType::Inspection
                }
            }
        }
    }

    /// Estimate maintenance cost (USD) for a given asset and action type.
    fn estimate_cost(&self, asset: &AssetHealth, action: ActionType) -> f64 {
        // Base costs by asset type (order-of-magnitude industry estimates in USD)
        let base_cost = match asset.asset_type {
            AssetType::Transformer => 500_000.0,
            AssetType::OverheadLine => 50_000.0,
            AssetType::UndergroundCable => 150_000.0,
            AssetType::CircuitBreaker => 100_000.0,
            AssetType::Capacitor => 20_000.0,
            AssetType::Reactor => 80_000.0,
            AssetType::Generator => 300_000.0,
        };
        let multiplier = match action {
            ActionType::Inspection => 0.005,
            ActionType::MinorMaintenance => 0.02,
            ActionType::MajorOverhaul => 0.15,
            ActionType::Replacement => 1.0,
        };
        base_cost * multiplier
    }

    /// Select the optimal year to schedule maintenance.
    fn select_scheduled_year(&self, asset: &AssetHealth, action: ActionType) -> f64 {
        // Schedule high-urgency actions sooner
        let urgency = match action {
            ActionType::Replacement => 0.0,
            ActionType::MajorOverhaul => 0.25,
            ActionType::MinorMaintenance => 0.5,
            ActionType::Inspection => 1.0,
        };
        let base_year = 2026.0; // current year
        base_year + urgency * (self.planning_horizon_years as f64).min(5.0) * asset.health_index
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_asset(id: usize, asset_type: AssetType, age: f64, health: f64) -> AssetHealth {
        let mut a = AssetHealth::new(id, asset_type, 2000, age, health);
        a.last_maintenance = age * 0.5; // maintained halfway through life
        a
    }

    // ── Bathtub Curve Tests ───────────────────────────────────────────────────

    #[test]
    fn test_asset_health_bathtub_curve_infant_mortality() {
        // Very young asset (t < 2 years) should have higher failure rate than mid-life
        let young = make_asset(1, AssetType::Transformer, 0.1, 0.99);
        let mature = make_asset(2, AssetType::Transformer, 15.0, 0.85);
        // Infant-mortality dominates at t≈0; constant phase lower
        let rate_young = young.failure_rate();
        let rate_mature = make_asset(3, AssetType::Transformer, 15.0, 1.0).failure_rate();
        // With HI=1.0 (no health penalty), infant-mortality at t=0.1 should exceed constant phase
        assert!(
            rate_young > rate_mature,
            "Infant mortality phase: young asset (λ={rate_young:.5}) should have higher rate than mature (λ={rate_mature:.5})"
        );
        let _ = mature; // suppress unused warning
    }

    #[test]
    fn test_asset_health_bathtub_curve_wearout() {
        // Old asset near end of life should have higher failure rate than mid-life
        let mid_life = make_asset(1, AssetType::Transformer, 20.0, 0.8);
        let old_asset = make_asset(2, AssetType::Transformer, 38.0, 0.3);
        assert!(
            old_asset.failure_rate() > mid_life.failure_rate(),
            "Wear-out phase: old asset should have higher failure rate than mid-life"
        );
    }

    #[test]
    fn test_asset_failure_rate_positive() {
        let asset = make_asset(1, AssetType::CircuitBreaker, 10.0, 0.7);
        assert!(
            asset.failure_rate() > 0.0,
            "Failure rate must always be positive"
        );
    }

    // ── Remaining Useful Life Tests ───────────────────────────────────────────

    #[test]
    fn test_remaining_useful_life_new_asset() {
        let new_asset = make_asset(1, AssetType::Transformer, 0.0, 1.0);
        let rul = new_asset.remaining_useful_life();
        let design = AssetType::Transformer.nominal_lifetime_years();
        // New asset (health=1, age=0) → RUL ≈ design_lifetime / 1.0 (no degradation)
        assert!(
            (rul - design).abs() < 0.1,
            "New transformer RUL ≈ {design} years, got {rul:.2}"
        );
    }

    #[test]
    fn test_remaining_useful_life_old_asset() {
        let design = AssetType::Transformer.nominal_lifetime_years() as u32;
        let old_asset = AssetHealth {
            asset_id: 1,
            asset_type: AssetType::Transformer,
            installation_year: 1980,
            design_lifetime_years: design,
            current_age_years: design as f64,
            health_index: 0.0,
            failure_rate_per_year: 0.05,
            last_maintenance: 5.0,
            n_failures_lifetime: 3,
            condition_scores: vec![],
        };
        let rul = old_asset.remaining_useful_life();
        assert!(
            rul <= 0.0,
            "Asset at design lifetime: RUL should be 0, got {rul:.2}"
        );
    }

    #[test]
    fn test_remaining_useful_life_monotone_with_age() {
        // RUL should decrease as age increases (all else equal)
        let a10 = make_asset(1, AssetType::OverheadLine, 10.0, 0.9);
        let a20 = make_asset(2, AssetType::OverheadLine, 20.0, 0.9);
        assert!(
            a10.remaining_useful_life() > a20.remaining_useful_life(),
            "Older asset should have smaller RUL"
        );
    }

    // ── Health Index Tests ────────────────────────────────────────────────────

    #[test]
    fn test_health_index_equal_weights() {
        let scores = vec![0.8, 0.6, 0.9];
        let weights = vec![1.0, 1.0, 1.0];
        let hi = AssetHealth::compute_health_index(&scores, &weights);
        let expected = (0.8 + 0.6 + 0.9) / 3.0;
        assert!(
            (hi - expected).abs() < 1e-9,
            "Equal-weight HI: expected {expected:.4}, got {hi:.4}"
        );
    }

    #[test]
    fn test_health_index_weighted() {
        let scores = vec![0.5, 0.9];
        let weights = vec![3.0, 1.0]; // score[0] is 3× more important
        let hi = AssetHealth::compute_health_index(&scores, &weights);
        let expected = (3.0 * 0.5 + 1.0 * 0.9) / 4.0; // = 0.6
        assert!(
            (hi - expected).abs() < 1e-9,
            "Weighted HI: expected {expected:.4}, got {hi:.4}"
        );
    }

    #[test]
    fn test_health_index_empty_returns_one() {
        let hi = AssetHealth::compute_health_index(&[], &[]);
        assert!((hi - 1.0).abs() < 1e-9, "Empty scores: HI should be 1.0");
    }

    // ── Failure Probability Tests ─────────────────────────────────────────────

    #[test]
    fn test_failure_probability_zero_years() {
        let asset = make_asset(1, AssetType::Generator, 10.0, 0.7);
        assert_eq!(
            asset.failure_probability(0.0),
            0.0,
            "Zero horizon: P(fail) = 0"
        );
    }

    #[test]
    fn test_failure_probability_in_range() {
        let asset = make_asset(1, AssetType::Transformer, 15.0, 0.5);
        let p = asset.failure_probability(5.0);
        assert!(
            p > 0.0 && p < 1.0,
            "Failure probability must be in (0,1), got {p:.4}"
        );
    }

    #[test]
    fn test_failure_probability_increases_with_horizon() {
        let asset = make_asset(1, AssetType::CircuitBreaker, 25.0, 0.4);
        let p1 = asset.failure_probability(1.0);
        let p5 = asset.failure_probability(5.0);
        assert!(p5 > p1, "Longer horizon → higher failure probability");
    }

    // ── Maintenance Optimizer Tests ───────────────────────────────────────────

    fn make_fleet() -> Vec<AssetHealth> {
        vec![
            make_asset(1, AssetType::Transformer, 35.0, 0.3), // high risk
            make_asset(2, AssetType::OverheadLine, 10.0, 0.9), // low risk
            make_asset(3, AssetType::CircuitBreaker, 25.0, 0.5),
            make_asset(4, AssetType::Capacitor, 15.0, 0.7),
        ]
    }

    #[test]
    fn test_maintenance_schedule_within_budget() {
        let assets = make_fleet();
        let optimizer =
            MaintenanceOptimizer::new(assets, MaintenanceStrategy::ConditionBased, 5, 1_000_000.0);
        let schedule = optimizer.optimize().expect("optimize should succeed");
        let total_budget = 1_000_000.0 * 5.0;
        assert!(
            schedule.total_cost <= total_budget + 1e-6,
            "Total cost {:.0} should not exceed budget {total_budget:.0}",
            schedule.total_cost
        );
    }

    #[test]
    fn test_maintenance_schedule_not_empty_for_degraded_fleet() {
        let assets = make_fleet();
        let optimizer =
            MaintenanceOptimizer::new(assets, MaintenanceStrategy::ConditionBased, 5, 10_000_000.0);
        let schedule = optimizer.optimize().expect("optimize should succeed");
        assert!(
            !schedule.actions.is_empty(),
            "Degraded fleet should have at least one action scheduled"
        );
    }

    #[test]
    fn test_risk_cost_curve_monotone_non_decreasing() {
        let assets = make_fleet();
        let optimizer = MaintenanceOptimizer::new(
            assets,
            MaintenanceStrategy::ConditionBased,
            5,
            0.0, // will be overridden per point
        );
        let budgets: Vec<f64> = vec![0.0, 10_000.0, 50_000.0, 200_000.0, 1_000_000.0, 5_000_000.0];
        let curve = optimizer.risk_cost_curve(&budgets);
        // Risk reduction should be non-decreasing as budget increases
        for w in curve.windows(2) {
            let (b0, r0) = w[0];
            let (b1, r1) = w[1];
            assert!(
                r1 >= r0 - 1e-9,
                "Risk reduction should be non-decreasing: budget {b0:.0}→{b1:.0}, reduction {r0:.4}→{r1:.4}"
            );
        }
    }

    #[test]
    fn test_risk_cost_curve_zero_budget_zero_reduction() {
        let assets = make_fleet();
        let optimizer =
            MaintenanceOptimizer::new(assets, MaintenanceStrategy::ConditionBased, 5, 0.0);
        let curve = optimizer.risk_cost_curve(&[0.0]);
        if let Some(&(_, reduction)) = curve.first() {
            assert!(
                reduction < 1e-6,
                "Zero budget → zero risk reduction, got {reduction:.4}"
            );
        }
    }

    #[test]
    fn test_preventive_strategy_schedules_maintenance() {
        let assets = vec![make_asset(1, AssetType::Transformer, 20.0, 0.6)];
        let optimizer = MaintenanceOptimizer::new(
            assets,
            MaintenanceStrategy::Preventive {
                interval_years: 5.0,
            },
            10,
            5_000_000.0,
        );
        let schedule = optimizer.optimize().expect("optimize");
        assert!(
            !schedule.actions.is_empty(),
            "Preventive strategy should schedule maintenance"
        );
    }

    #[test]
    fn test_predictive_strategy() {
        let assets = vec![make_asset(1, AssetType::Generator, 28.0, 0.2)];
        let optimizer = MaintenanceOptimizer::new(
            assets,
            MaintenanceStrategy::Predictive { threshold: 0.05 },
            5,
            5_000_000.0,
        );
        let schedule = optimizer.optimize().expect("optimize");
        assert!(
            !schedule.actions.is_empty(),
            "High-risk old generator should trigger predictive maintenance"
        );
    }

    #[test]
    fn test_risk_reduction_in_range() {
        let assets = make_fleet();
        let optimizer =
            MaintenanceOptimizer::new(assets, MaintenanceStrategy::ConditionBased, 5, 5_000_000.0);
        let schedule = optimizer.optimize().expect("optimize");
        assert!(
            schedule.risk_reduction >= 0.0 && schedule.risk_reduction <= 1.0,
            "Risk reduction must be in [0,1], got {:.4}",
            schedule.risk_reduction
        );
    }
}
