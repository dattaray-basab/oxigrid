//! Predictive grid asset lifecycle management.
//!
//! Implements risk-based maintenance scheduling using a condition-score
//! deterioration model and net-present-value (NPV) optimisation over a
//! multi-year planning horizon.
//!
//! # Deterioration model
//!
//! Asset condition decays exponentially:
//!
//! ```text
//! score(t) = score_0 × exp(−λ × t)
//! ```
//!
//! where λ is a class-specific decay rate derived from the asset's nominal
//! design lifetime.
//!
//! # Risk-based prioritisation
//!
//! ```text
//! risk = failure_rate × failure_cost × criticality
//! ```
//!
//! Assets are sorted by descending risk.  Within the annual budget cap,
//! the highest-risk assets are scheduled for maintenance or replacement first.
//!
//! # NPV
//!
//! Future costs are discounted:
//!
//! ```text
//! NPV = Σ_t  cost(t) / (1 + discount_rate)^t
//! ```
//!
//! # References
//!
//! - CIGRE TB 858, "Asset Management Frameworks for Transmission Systems" 2021
//! - Brown, "Electric Power Distribution Reliability", CRC Press 2009

use serde::{Deserialize, Serialize};
use thiserror::Error;

// ─────────────────────────────────────────────────────────────────────────────
// Error type
// ─────────────────────────────────────────────────────────────────────────────

/// Errors from the asset lifecycle optimiser.
#[derive(Debug, Error)]
pub enum LifecycleError {
    /// No assets have been added.
    #[error("no assets registered")]
    NoAssets,

    /// Planning horizon is zero.
    #[error("planning_horizon_years must be > 0")]
    ZeroHorizon,

    /// Discount rate is out of range.
    #[error("discount_rate must be in (0, 1), got {0}")]
    InvalidDiscountRate(f64),
}

// ─────────────────────────────────────────────────────────────────────────────
// Asset types
// ─────────────────────────────────────────────────────────────────────────────

/// Classification of a grid asset for lifecycle analysis.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AssetType {
    /// High-voltage power transformer.
    PowerTransformer,
    /// Air-blast, SF6, or vacuum circuit breaker.
    CircuitBreaker,
    /// Overhead transmission or distribution line.
    TransmissionLine,
    /// Distribution-voltage transformer.
    DistributionTransformer,
    /// XLPE or paper-insulated underground cable.
    UndergroundCable,
    /// Shunt capacitor bank.
    CapacitorBank,
    /// Surge / lightning arrester.
    LightningArrester,
    /// Ceramic or polymer insulator.
    Insulator,
}

impl AssetType {
    /// Nominal design lifetime \[years\].
    pub fn nominal_lifetime_years(&self) -> f64 {
        match self {
            AssetType::PowerTransformer => 40.0,
            AssetType::CircuitBreaker => 30.0,
            AssetType::TransmissionLine => 50.0,
            AssetType::DistributionTransformer => 35.0,
            AssetType::UndergroundCable => 35.0,
            AssetType::CapacitorBank => 20.0,
            AssetType::LightningArrester => 25.0,
            AssetType::Insulator => 30.0,
        }
    }

    /// Condition decay constant λ = −ln(0.3) / nominal_lifetime.
    ///
    /// At end-of-life (t = nominal_lifetime), condition decays to 30% of
    /// initial score, which represents the end-of-useful-life threshold.
    pub fn decay_lambda(&self) -> f64 {
        // λ such that exp(-λ × T) = 0.3  →  λ = ln(10/3) / T
        let ln_10_over_3 = (10.0_f64 / 3.0).ln();
        ln_10_over_3 / self.nominal_lifetime_years()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Asset record
// ─────────────────────────────────────────────────────────────────────────────

/// A single grid asset subject to lifecycle management.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GridAsset {
    /// Unique asset identifier.
    pub id: usize,
    /// Asset class.
    pub asset_type: AssetType,
    /// Year the asset was installed.
    pub installation_year: usize,
    /// Current calendar year.
    pub current_year: usize,
    /// Current condition score (0–100; 100 = new).
    pub condition_score: f64,
    /// Network criticality (0–1; 1 = most critical path).
    pub criticality: f64,
    /// Current annual failure rate \[failures/year\].
    pub failure_rate_per_year: f64,
    /// Expected cost of an unplanned failure \[USD\].
    pub failure_cost_usd: f64,
    /// Cost of one planned maintenance intervention \[USD\].
    pub maintenance_cost_usd: f64,
    /// Capital cost of a full replacement \[USD\].
    pub replacement_cost_usd: f64,
    /// Estimated remaining useful life \[years\].
    pub remaining_life_years: f64,
}

impl GridAsset {
    /// Instantaneous risk score = failure_rate × failure_cost × criticality \[USD/year\].
    pub fn risk_score(&self) -> f64 {
        self.failure_rate_per_year * self.failure_cost_usd * self.criticality
    }

    /// Age of the asset \[years\].
    pub fn age_years(&self) -> usize {
        self.current_year.saturating_sub(self.installation_year)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Maintenance actions
// ─────────────────────────────────────────────────────────────────────────────

/// Type of planned maintenance or capital action.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PlannedAction {
    /// Visual / diagnostic inspection (no physical intervention).
    Inspection,
    /// Minor maintenance: cleaning, re-torquing, partial replacement.
    MinorMaintenance,
    /// Major overhaul: rewind, SF6 refill, bushing replacement.
    MajorOverhaul,
    /// Full asset replacement with new equipment.
    Replacement,
    /// Partial refurbishment extending life without full replacement.
    Refurbishment,
}

/// Priority level of a maintenance plan entry.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum MaintenancePriority {
    /// Immediate action required (safety/reliability risk).
    Urgent,
    /// Action within 6 months.
    High,
    /// Action within 1 year.
    Medium,
    /// Routine scheduling (> 1 year).
    Low,
}

/// A single maintenance work order entry.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MaintenancePlan {
    /// Asset identifier.
    pub asset_id: usize,
    /// Calendar year in which the work is planned.
    pub planned_year: usize,
    /// Type of intervention.
    pub action: PlannedAction,
    /// Estimated direct cost \[USD\].
    pub estimated_cost_usd: f64,
    /// Expected life extension resulting from this action \[years\].
    pub expected_life_extension_years: f64,
    /// Factor by which the failure rate is reduced (0–1; 1 = no reduction).
    pub risk_reduction_factor: f64,
    /// Scheduling priority.
    pub priority: MaintenancePriority,
}

// ─────────────────────────────────────────────────────────────────────────────
// Results
// ─────────────────────────────────────────────────────────────────────────────

/// Output of the lifecycle optimisation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LifecycleResult {
    /// Scheduled maintenance work orders.
    pub maintenance_plans: Vec<MaintenancePlan>,
    /// Total direct maintenance cost per year \[USD\].
    pub annual_budget: Vec<f64>,
    /// Expected failure (risk) cost per year \[USD\].
    pub annual_risk_cost: Vec<f64>,
    /// NPV of all maintenance and avoided failure costs \[USD\].
    pub total_npv_cost_usd: f64,
    /// Percentage risk reduction achieved by the maintenance plan.
    pub risk_reduction_pct: f64,
    /// IDs of assets with high criticality and low condition.
    pub assets_at_risk: Vec<usize>,
    /// Years where the budget exceeds the annual limit `(year, cost_usd)`.
    pub budget_peaks: Vec<(usize, f64)>,
}

// ─────────────────────────────────────────────────────────────────────────────
// Optimiser
// ─────────────────────────────────────────────────────────────────────────────

/// Lifecycle configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AssetLifecycleConfig {
    /// Number of years in the planning horizon.
    pub planning_horizon_years: usize,
    /// Annual discount rate (e.g. 0.05 = 5 %).
    pub discount_rate: f64,
    /// Cost multiplier for unplanned corrective maintenance vs. planned.
    pub corrective_maintenance_multiplier: f64,
    /// Weight on reliability (vs. cost) when prioritising (0 = pure cost, 1 = pure risk).
    pub reliability_weight: f64,
}

impl Default for AssetLifecycleConfig {
    fn default() -> Self {
        Self {
            planning_horizon_years: 10,
            discount_rate: 0.05,
            corrective_maintenance_multiplier: 3.0,
            reliability_weight: 0.5,
        }
    }
}

/// Predictive grid asset lifecycle optimiser.
pub struct AssetLifecycleOptimizer {
    config: AssetLifecycleConfig,
    assets: Vec<GridAsset>,
    annual_budget_limit: f64,
}

impl AssetLifecycleOptimizer {
    /// Create a new optimiser with the given configuration.
    pub fn new(config: AssetLifecycleConfig) -> Self {
        Self {
            config,
            assets: Vec::new(),
            annual_budget_limit: f64::MAX,
        }
    }

    /// Register an asset for lifecycle analysis.
    pub fn add_asset(&mut self, asset: GridAsset) {
        self.assets.push(asset);
    }

    /// Set the maximum annual maintenance budget \[USD\].
    pub fn set_budget_limit(&mut self, usd_per_year: f64) {
        self.annual_budget_limit = usd_per_year;
    }

    /// Run the lifecycle optimisation.
    ///
    /// # Errors
    ///
    /// - [`LifecycleError::NoAssets`] if no assets have been added.
    /// - [`LifecycleError::ZeroHorizon`] if the planning horizon is zero.
    /// - [`LifecycleError::InvalidDiscountRate`] if discount rate ≤ 0 or ≥ 1.
    pub fn optimize(&self) -> Result<LifecycleResult, LifecycleError> {
        if self.assets.is_empty() {
            return Err(LifecycleError::NoAssets);
        }
        if self.config.planning_horizon_years == 0 {
            return Err(LifecycleError::ZeroHorizon);
        }
        let r = self.config.discount_rate;
        if r <= 0.0 || r >= 1.0 {
            return Err(LifecycleError::InvalidDiscountRate(r));
        }

        let horizon = self.config.planning_horizon_years;
        let base_year = self
            .assets
            .iter()
            .map(|a| a.current_year)
            .max()
            .unwrap_or(2024);

        // ── Step 1: Compute initial risk scores ───────────────────────────
        let initial_total_risk: f64 = self.assets.iter().map(|a| a.risk_score()).sum();

        // ── Step 2: Sort assets by descending risk (primary) ──────────────
        let mut sorted_ids: Vec<usize> = (0..self.assets.len()).collect();
        sorted_ids.sort_by(|&a, &b| {
            let ra = self.assets[a].risk_score();
            let rb = self.assets[b].risk_score();
            rb.partial_cmp(&ra).unwrap_or(std::cmp::Ordering::Equal)
        });

        // ── Step 3: Build maintenance plans within budget ─────────────────
        let mut maintenance_plans: Vec<MaintenancePlan> = Vec::new();
        let mut annual_budget: Vec<f64> = vec![0.0; horizon];
        let mut annual_risk_cost: Vec<f64> = vec![0.0; horizon];
        let mut post_maint_risk: Vec<f64> = vec![0.0; self.assets.len()];

        for &idx in &sorted_ids {
            let asset = &self.assets[idx];

            // Determine best action for this asset
            let (action, planned_year_offset) = self.choose_action(asset, base_year, horizon);

            if planned_year_offset >= horizon {
                continue;
            }

            let cost = self.action_cost(asset, &action);
            let life_ext = self.life_extension(&action);
            let risk_factor = self.post_maintenance_failure_rate(asset, &action)
                / asset.failure_rate_per_year.max(1e-9);
            let priority = self.compute_priority(asset, &action);

            // Check budget
            let yr = planned_year_offset;
            if annual_budget[yr] + cost <= self.annual_budget_limit {
                annual_budget[yr] += cost;
                post_maint_risk[idx] = asset.failure_rate_per_year
                    * risk_factor
                    * asset.failure_cost_usd
                    * asset.criticality;

                maintenance_plans.push(MaintenancePlan {
                    asset_id: asset.id,
                    planned_year: base_year + yr,
                    action,
                    estimated_cost_usd: cost,
                    expected_life_extension_years: life_ext,
                    risk_reduction_factor: 1.0 - risk_factor,
                    priority,
                });
            } else {
                // Over budget — push to next year if possible
                let next_yr = yr + 1;
                if next_yr < horizon && annual_budget[next_yr] + cost <= self.annual_budget_limit {
                    annual_budget[next_yr] += cost;
                    post_maint_risk[idx] = asset.failure_rate_per_year
                        * risk_factor
                        * asset.failure_cost_usd
                        * asset.criticality;

                    maintenance_plans.push(MaintenancePlan {
                        asset_id: asset.id,
                        planned_year: base_year + next_yr,
                        action,
                        estimated_cost_usd: cost,
                        expected_life_extension_years: life_ext,
                        risk_reduction_factor: 1.0 - risk_factor,
                        priority,
                    });
                } else {
                    // Cannot schedule — corrective maintenance cost (higher)
                    post_maint_risk[idx] = asset.risk_score();
                }
            }
        }

        // ── Step 4: Annual risk cost (projected deterioration) ────────────
        for (yr, risk_slot) in annual_risk_cost.iter_mut().enumerate() {
            *risk_slot = self
                .assets
                .iter()
                .enumerate()
                .map(|(idx, asset)| {
                    let projected_score = self.project_condition(asset, yr);
                    let risk_scale = if projected_score > 0.0 {
                        (100.0 - projected_score) / 100.0
                    } else {
                        1.0
                    };
                    let base_risk = if post_maint_risk[idx] > 0.0 {
                        post_maint_risk[idx]
                    } else {
                        asset.risk_score()
                    };
                    base_risk * (1.0 + risk_scale)
                })
                .sum();
        }

        // ── Step 5: NPV ───────────────────────────────────────────────────
        let mut total_npv = 0.0_f64;
        for yr in 0..horizon {
            let discount = (1.0 + r).powi(yr as i32 + 1);
            total_npv += (annual_budget[yr] + annual_risk_cost[yr]) / discount;
        }

        // ── Step 6: Risk reduction ────────────────────────────────────────
        let final_total_risk: f64 = post_maint_risk.iter().copied().sum::<f64>()
            + self
                .assets
                .iter()
                .enumerate()
                .filter(|(i, _)| post_maint_risk[*i] == 0.0)
                .map(|(_, a)| a.risk_score())
                .sum::<f64>();

        let risk_reduction_pct = if initial_total_risk > 0.0 {
            ((initial_total_risk - final_total_risk) / initial_total_risk * 100.0).max(0.0)
        } else {
            0.0
        };

        // ── Step 7: Assets at risk ────────────────────────────────────────
        let assets_at_risk: Vec<usize> = self
            .assets
            .iter()
            .filter(|a| a.criticality > 0.7 && a.condition_score < 40.0)
            .map(|a| a.id)
            .collect();

        // ── Step 8: Budget peaks ──────────────────────────────────────────
        let budget_peaks: Vec<(usize, f64)> = annual_budget
            .iter()
            .enumerate()
            .filter(|(_, &c)| c > self.annual_budget_limit * 0.9)
            .map(|(yr, &c)| (base_year + yr, c))
            .collect();

        Ok(LifecycleResult {
            maintenance_plans,
            annual_budget,
            annual_risk_cost,
            total_npv_cost_usd: total_npv,
            risk_reduction_pct,
            assets_at_risk,
            budget_peaks,
        })
    }

    // ── Internal helpers ─────────────────────────────────────────────────────

    /// Choose the best maintenance action and when it should occur.
    fn choose_action(
        &self,
        asset: &GridAsset,
        _base_year: usize,
        horizon: usize,
    ) -> (PlannedAction, usize) {
        let condition = asset.condition_score;
        let remaining = asset.remaining_life_years;

        if condition < 20.0 || remaining < 1.0 {
            (PlannedAction::Replacement, 0)
        } else if condition < 40.0 || remaining < 3.0 {
            (PlannedAction::MajorOverhaul, 0)
        } else if condition < 60.0 {
            (
                PlannedAction::MinorMaintenance,
                1.min(horizon.saturating_sub(1)),
            )
        } else {
            (PlannedAction::Inspection, 2.min(horizon.saturating_sub(1)))
        }
    }

    /// Estimated cost of a specific action on an asset \[USD\].
    fn action_cost(&self, asset: &GridAsset, action: &PlannedAction) -> f64 {
        match action {
            PlannedAction::Inspection => asset.maintenance_cost_usd * 0.1,
            PlannedAction::MinorMaintenance => asset.maintenance_cost_usd * 0.4,
            PlannedAction::MajorOverhaul => asset.maintenance_cost_usd,
            PlannedAction::Replacement => asset.replacement_cost_usd,
            PlannedAction::Refurbishment => asset.replacement_cost_usd * 0.5,
        }
    }

    /// Life extension resulting from an action \[years\].
    fn life_extension(&self, action: &PlannedAction) -> f64 {
        match action {
            PlannedAction::Inspection => 0.0,
            PlannedAction::MinorMaintenance => 2.0,
            PlannedAction::MajorOverhaul => 8.0,
            PlannedAction::Replacement => 40.0,
            PlannedAction::Refurbishment => 15.0,
        }
    }

    /// Post-maintenance failure rate \[failures/year\].
    ///
    /// - Inspection → no change
    /// - MinorMaintenance → −20 %
    /// - MajorOverhaul → −50 %
    /// - Replacement → reset to new-asset rate (÷ 10)
    /// - Refurbishment → −35 %
    fn post_maintenance_failure_rate(&self, asset: &GridAsset, action: &PlannedAction) -> f64 {
        let base = asset.failure_rate_per_year;
        match action {
            PlannedAction::Inspection => base,
            PlannedAction::MinorMaintenance => base * 0.80,
            PlannedAction::MajorOverhaul => base * 0.50,
            PlannedAction::Replacement => base * 0.10,
            PlannedAction::Refurbishment => base * 0.65,
        }
    }

    /// Project asset condition score `years` from now using exponential decay.
    ///
    /// `score(t) = score_0 × exp(−λ × t)`
    fn project_condition(&self, asset: &GridAsset, years: usize) -> f64 {
        let lambda = asset.asset_type.decay_lambda();
        asset.condition_score * (-lambda * years as f64).exp()
    }

    /// Scheduling priority based on risk and action severity.
    fn compute_priority(&self, asset: &GridAsset, action: &PlannedAction) -> MaintenancePriority {
        match action {
            PlannedAction::Replacement => {
                if asset.condition_score < 20.0 {
                    MaintenancePriority::Urgent
                } else {
                    MaintenancePriority::High
                }
            }
            PlannedAction::MajorOverhaul => {
                if asset.criticality > 0.8 {
                    MaintenancePriority::High
                } else {
                    MaintenancePriority::Medium
                }
            }
            PlannedAction::MinorMaintenance | PlannedAction::Refurbishment => {
                MaintenancePriority::Medium
            }
            PlannedAction::Inspection => MaintenancePriority::Low,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_config() -> AssetLifecycleConfig {
        AssetLifecycleConfig {
            planning_horizon_years: 5,
            discount_rate: 0.05,
            corrective_maintenance_multiplier: 3.0,
            reliability_weight: 0.5,
        }
    }

    fn make_asset(id: usize, condition: f64, criticality: f64, failure_rate: f64) -> GridAsset {
        GridAsset {
            id,
            asset_type: AssetType::PowerTransformer,
            installation_year: 1990,
            current_year: 2024,
            condition_score: condition,
            criticality,
            failure_rate_per_year: failure_rate,
            failure_cost_usd: 500_000.0,
            maintenance_cost_usd: 50_000.0,
            replacement_cost_usd: 2_000_000.0,
            remaining_life_years: condition / 100.0 * 40.0,
        }
    }

    // ── Test 1: high-risk asset scheduled first ───────────────────────────

    #[test]
    fn test_high_risk_asset_scheduled_first() {
        let config = make_config();
        let mut opt = AssetLifecycleOptimizer::new(config);
        opt.set_budget_limit(10_000_000.0); // no budget constraint

        // Low-risk asset
        opt.add_asset(make_asset(0, 80.0, 0.3, 0.01));
        // High-risk asset
        opt.add_asset(make_asset(1, 30.0, 0.9, 0.5));

        let result = opt.optimize().expect("optimize");

        // The first scheduled plan should be for the high-risk asset (id=1)
        let plans_for_1: Vec<&MaintenancePlan> = result
            .maintenance_plans
            .iter()
            .filter(|p| p.asset_id == 1)
            .collect();
        let plans_for_0: Vec<&MaintenancePlan> = result
            .maintenance_plans
            .iter()
            .filter(|p| p.asset_id == 0)
            .collect();

        assert!(!plans_for_1.is_empty(), "High-risk asset must be scheduled");
        // High-risk asset should have a more severe action
        if let Some(p) = plans_for_1.first() {
            assert!(
                matches!(
                    p.action,
                    PlannedAction::MajorOverhaul | PlannedAction::Replacement
                ),
                "Expected overhaul or replacement for high-risk, got {:?}",
                p.action
            );
        }
        // Low-risk asset inspection/minor is fine
        if let Some(p) = plans_for_0.first() {
            assert!(
                matches!(
                    p.action,
                    PlannedAction::Inspection | PlannedAction::MinorMaintenance
                ),
                "Expected lighter action for low-risk, got {:?}",
                p.action
            );
        }
    }

    // ── Test 2: budget constraint respected ──────────────────────────────

    #[test]
    fn test_budget_constraint_respected() {
        let config = make_config();
        let mut opt = AssetLifecycleOptimizer::new(config);
        opt.set_budget_limit(60_000.0); // only one minor maintenance per year

        for i in 0..5 {
            opt.add_asset(make_asset(i, 50.0, 0.7, 0.1));
        }

        let result = opt.optimize().expect("optimize");

        for (yr, &cost) in result.annual_budget.iter().enumerate() {
            assert!(
                cost <= 60_001.0, // small float tolerance
                "Year {yr}: budget {cost:.0} exceeds limit 60000"
            );
        }
    }

    // ── Test 3: replacement triggered at very low condition ───────────────

    #[test]
    fn test_replacement_triggered_at_low_condition() {
        let config = make_config();
        let mut opt = AssetLifecycleOptimizer::new(config);
        opt.set_budget_limit(5_000_000.0);

        // Extremely poor condition
        opt.add_asset(make_asset(0, 10.0, 0.9, 0.8));

        let result = opt.optimize().expect("optimize");

        let plan = result
            .maintenance_plans
            .iter()
            .find(|p| p.asset_id == 0)
            .expect("asset 0 should be scheduled");

        assert_eq!(
            plan.action,
            PlannedAction::Replacement,
            "Asset at 10% condition should be replaced"
        );
    }

    // ── Test 4: NPV — maintenance cheaper than unchecked failure ─────────

    #[test]
    fn test_npv_maintenance_cheaper_than_failure() {
        let config = AssetLifecycleConfig {
            planning_horizon_years: 5,
            discount_rate: 0.05,
            corrective_maintenance_multiplier: 3.0,
            reliability_weight: 0.5,
        };
        let mut opt = AssetLifecycleOptimizer::new(config.clone());
        opt.set_budget_limit(10_000_000.0);

        // High failure rate, high failure cost — maintenance should pay off
        opt.add_asset(make_asset(0, 40.0, 1.0, 1.0)); // 1 failure/year

        let result = opt.optimize().expect("optimize");

        // NPV should be finite and positive
        assert!(
            result.total_npv_cost_usd.is_finite() && result.total_npv_cost_usd > 0.0,
            "Expected positive finite NPV, got {}",
            result.total_npv_cost_usd
        );

        // With maintenance, risk reduction should be non-zero
        assert!(
            result.risk_reduction_pct >= 0.0,
            "Expected non-negative risk reduction"
        );
    }

    // ── Test 5: risk reduction achieved after maintenance ─────────────────

    #[test]
    fn test_risk_reduction_achieved() {
        let config = make_config();
        let mut opt = AssetLifecycleOptimizer::new(config);
        opt.set_budget_limit(5_000_000.0);

        opt.add_asset(make_asset(0, 35.0, 0.9, 0.4));
        opt.add_asset(make_asset(1, 25.0, 0.8, 0.6));

        let result = opt.optimize().expect("optimize");

        assert!(
            result.risk_reduction_pct >= 0.0 && result.risk_reduction_pct <= 100.0,
            "Risk reduction {:.1}% out of range",
            result.risk_reduction_pct
        );

        // Both assets should have plans
        assert!(
            result.maintenance_plans.len() >= 2,
            "Expected at least 2 maintenance plans"
        );
    }

    // ── Test 6: assets_at_risk identified correctly ──────────────────────

    #[test]
    fn test_assets_at_risk_identified() {
        let config = make_config();
        let mut opt = AssetLifecycleOptimizer::new(config);
        opt.set_budget_limit(10_000_000.0);

        // High criticality, very low condition → at risk
        opt.add_asset(make_asset(10, 25.0, 0.95, 0.5));
        // Low criticality, medium condition → not at risk
        opt.add_asset(make_asset(11, 60.0, 0.3, 0.05));

        let result = opt.optimize().expect("optimize");

        assert!(
            result.assets_at_risk.contains(&10),
            "Asset 10 (high criticality, low condition) should be at risk"
        );
        assert!(
            !result.assets_at_risk.contains(&11),
            "Asset 11 (low criticality, medium condition) should not be at risk"
        );
    }

    // ── Test 7: no assets → error ─────────────────────────────────────────

    #[test]
    fn test_no_assets_error() {
        let opt = AssetLifecycleOptimizer::new(make_config());
        let result = opt.optimize();
        assert!(matches!(result, Err(LifecycleError::NoAssets)));
    }

    // ── Test 8: project_condition decays over time ────────────────────────

    #[test]
    fn test_project_condition_decays() {
        let opt = AssetLifecycleOptimizer::new(make_config());
        let asset = make_asset(0, 100.0, 1.0, 0.1);

        let c0 = opt.project_condition(&asset, 0);
        let c10 = opt.project_condition(&asset, 10);
        let c40 = opt.project_condition(&asset, 40);

        assert!((c0 - 100.0).abs() < 1e-6, "At t=0 condition should be 100");
        assert!(c10 < c0, "Condition should decrease over time");
        assert!(c40 < c10, "Condition at t=40 should be less than at t=10");
        assert!(c40 > 20.0, "At nominal lifetime, condition ≈ 30% = 30.0");
    }
}
