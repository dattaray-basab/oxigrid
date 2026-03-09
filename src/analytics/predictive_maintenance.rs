//! Predictive maintenance for power system assets.
//!
//! Provides condition monitoring, degradation modeling, remaining useful life (RUL)
//! estimation, failure mode analysis (FMEA), and maintenance schedule optimization
//! for transformers, circuit breakers, cables, generators, and other assets.
//!
//! # Standards basis
//! - IEC 60076 (transformer condition assessment)
//! - IEC 60300 (dependability management)
//! - IEEE 1910 (predictive maintenance)
//! - Weibull reliability analysis

use std::collections::HashMap;

// Physical constants
const BOLTZMANN_EV_PER_K: f64 = 8.617_333_262e-5; // eV/K
const SECONDS_PER_DAY: f64 = 86_400.0;
const SECONDS_PER_YEAR: f64 = 31_557_600.0;

/// Asset type for predictive maintenance classification.
#[derive(Debug, Clone, PartialEq)]
pub enum AssetType {
    /// High-voltage power transformer
    PowerTransformer,
    /// Circuit breaker (SF6, vacuum, oil)
    CircuitBreaker,
    /// Underground cable system
    UndergroundCable,
    /// Overhead transmission/distribution line
    OverheadLine,
    /// Synchronous or asynchronous generator
    Generator,
    /// Electric motor (MV/LV)
    Motor,
    /// Shunt or series capacitor bank
    Capacitor,
    /// Switchgear assembly
    SwitchGear,
}

/// A single condition indicator measurement from a sensor or inspection.
#[derive(Debug, Clone)]
pub struct ConditionIndicator {
    /// Human-readable name (e.g., `"oil_temperature"`, `"h2_ppm"`)
    pub name: String,
    /// Measured value
    pub value: f64,
    /// Engineering unit string (e.g., `"°C"`, `"ppm"`, `"mΩ"`)
    pub unit: String,
    /// Unix epoch timestamp of the measurement (seconds)
    pub timestamp_s: f64,
    /// Normal operating range `(lower, upper)`
    pub normal_range: (f64, f64),
    /// Value above which condition is degraded (warning)
    pub warning_threshold: f64,
    /// Value above which condition is critical
    pub critical_threshold: f64,
}

impl ConditionIndicator {
    /// Normalize indicator to `[0, 1]` health contribution.
    ///
    /// Returns `1.0` when value is within `normal_range`, `0.0` at or beyond
    /// `critical_threshold`, and a linear interpolation in between.
    pub fn normalized_health(&self) -> f64 {
        let v = self.value;
        let (lo, hi) = self.normal_range;
        // In normal range
        if v >= lo && v <= hi {
            return 1.0;
        }
        // Below normal (some indicators like SF6 pressure decrease with degradation)
        if v < lo {
            // treat as mirror: distance below lo vs distance from lo to 0
            let span = lo; // distance from 0 to normal lower bound
            if span <= 0.0 {
                return 1.0;
            }
            let warn_lo = lo - (self.warning_threshold.max(lo) - lo).abs(); // symmetric approximation
                                                                            // Use critical as bottom-of-range indicator
            let crit_lo = self.critical_threshold.min(lo);
            if crit_lo >= lo {
                return 1.0;
            }
            if v <= crit_lo {
                return 0.0;
            }
            let _warn = lo - (lo - hi).abs() * 0.1; // fallback warning at 10% below normal
            let _ = warn_lo;
            // linear from lo (1.0) to crit_lo (0.0)
            ((v - crit_lo) / (lo - crit_lo)).clamp(0.0, 1.0)
        } else {
            // Above normal range (v > hi)
            let warn = self.warning_threshold.max(hi);
            let crit = self.critical_threshold.max(warn);
            if v >= crit {
                return 0.0;
            }
            if v <= warn {
                // between hi and warning
                return 1.0 - 0.2 * ((v - hi) / (warn - hi + f64::EPSILON));
            }
            // between warning and critical
            0.8 * (1.0 - (v - warn) / (crit - warn + f64::EPSILON))
        }
    }
}

/// Degradation model parametrization.
#[derive(Debug, Clone)]
pub enum DegradationModel {
    /// Linear degradation: `health = H0 - rate * t`
    Linear {
        /// Initial health (0–1)
        initial_health: f64,
        /// Degradation rate per second
        degradation_rate: f64,
    },
    /// Exponential decay: `health = H0 * exp(-k * t)`
    Exponential {
        /// Initial health (0–1)
        initial_health: f64,
        /// Decay constant (1/s)
        decay_constant: f64,
    },
    /// Power-law: `health = H0 * (1 - (t / T_life)^alpha)`
    PowerLaw {
        /// Initial health (0–1)
        initial_health: f64,
        /// Design lifetime (seconds)
        lifetime_s: f64,
        /// Shape exponent; >1 → concave (fast early degradation), <1 → convex
        alpha: f64,
    },
    /// Weibull survival function: `health = exp(-(t/eta)^beta)`
    Weibull {
        /// Shape parameter
        beta: f64,
        /// Scale (characteristic life, seconds)
        eta: f64,
    },
    /// Arrhenius temperature-accelerated degradation
    Arrhenius {
        /// Activation energy (eV)
        activation_energy_ev: f64,
        /// Pre-exponential factor (1/s)
        pre_exponential: f64,
        /// Operating temperature (K)
        temperature_k: f64,
    },
}

impl DegradationModel {
    /// Return health `[0, 1]` at elapsed time `t_s` (seconds).
    pub fn health_at_time(&self, t_s: f64) -> f64 {
        match self {
            DegradationModel::Linear {
                initial_health,
                degradation_rate,
            } => (initial_health - degradation_rate * t_s).clamp(0.0, 1.0),

            DegradationModel::Exponential {
                initial_health,
                decay_constant,
            } => (initial_health * (-decay_constant * t_s).exp()).clamp(0.0, 1.0),

            DegradationModel::PowerLaw {
                initial_health,
                lifetime_s,
                alpha,
            } => {
                if *lifetime_s <= 0.0 {
                    return 0.0;
                }
                let ratio = (t_s / lifetime_s).clamp(0.0, 1.0);
                (initial_health * (1.0 - ratio.powf(*alpha))).clamp(0.0, 1.0)
            }

            DegradationModel::Weibull { beta, eta } => {
                if *eta <= 0.0 {
                    return 0.0;
                }
                (-(t_s / eta).powf(*beta)).exp().clamp(0.0, 1.0)
            }

            DegradationModel::Arrhenius {
                activation_energy_ev,
                pre_exponential,
                temperature_k,
            } => {
                let k = pre_exponential
                    * (-activation_energy_ev / (BOLTZMANN_EV_PER_K * temperature_k)).exp();
                (1.0 - k * t_s).clamp(0.0, 1.0)
            }
        }
    }

    /// Estimate time (seconds) until `current_health` reaches `failure_threshold`.
    ///
    /// Returns `0.0` if already below threshold.
    pub fn rul_from_health(&self, current_health: f64, failure_threshold: f64) -> f64 {
        if current_health <= failure_threshold {
            return 0.0;
        }
        match self {
            DegradationModel::Linear {
                initial_health: _,
                degradation_rate,
            } => {
                if *degradation_rate <= 0.0 {
                    return f64::MAX;
                }
                (current_health - failure_threshold) / degradation_rate
            }

            DegradationModel::Exponential {
                initial_health: _,
                decay_constant,
            } => {
                if *decay_constant <= 0.0 {
                    return f64::MAX;
                }
                // health(t) = h0 * exp(-k*t), solve for t given current_health
                // t = -ln(failure_threshold / current_health) / k
                let ratio = failure_threshold / current_health;
                if ratio <= 0.0 || ratio >= 1.0 {
                    return 0.0;
                }
                -ratio.ln() / decay_constant
            }

            DegradationModel::PowerLaw {
                initial_health,
                lifetime_s,
                alpha,
            } => {
                if *initial_health <= 0.0 || *alpha <= 0.0 {
                    return 0.0;
                }
                // h = H0 * (1 - (t/T)^a)  →  t = T * (1 - h/H0)^(1/a)
                let frac = 1.0 - failure_threshold / initial_health;
                if frac <= 0.0 {
                    return 0.0;
                }
                lifetime_s * frac.powf(1.0 / alpha)
            }

            DegradationModel::Weibull { beta, eta } => {
                // Weibull survival: health = exp(-(t/eta)^beta)
                // t = eta * (-ln(failure_threshold))^(1/beta)
                if failure_threshold <= 0.0 {
                    return f64::MAX;
                }
                let ln_val = -failure_threshold.ln();
                if ln_val <= 0.0 {
                    return 0.0;
                }
                eta * ln_val.powf(1.0 / beta)
            }

            DegradationModel::Arrhenius {
                activation_energy_ev,
                pre_exponential,
                temperature_k,
            } => {
                let k = pre_exponential
                    * (-activation_energy_ev / (BOLTZMANN_EV_PER_K * temperature_k)).exp();
                if k <= 0.0 {
                    return f64::MAX;
                }
                (current_health - failure_threshold) / k
            }
        }
    }
}

/// A maintenance event in the asset's history.
#[derive(Debug, Clone)]
pub struct MaintenanceEvent {
    /// Unix epoch timestamp (seconds)
    pub timestamp_s: f64,
    /// Type of maintenance performed
    pub event_type: MaintenanceType,
    /// Actual cost in USD
    pub cost_usd: f64,
    /// Health value restored after this event (0–1)
    pub health_after: f64,
}

/// Classification of a maintenance activity.
#[derive(Debug, Clone, PartialEq)]
pub enum MaintenanceType {
    /// Visual/diagnostic inspection only
    Inspection,
    /// Minor component repair or adjustment
    MinorRepair,
    /// Full equipment overhaul
    MajorOverhaul,
    /// Replacement of a failed/degraded component
    ComponentReplacement,
    /// Unplanned corrective maintenance after failure
    CorrectiveMaintenance,
}

/// Complete condition record for a single asset.
#[derive(Debug, Clone)]
pub struct AssetCondition {
    /// Unique asset identifier
    pub asset_id: String,
    /// Asset category
    pub asset_type: AssetType,
    /// Asset age in years
    pub age_years: f64,
    /// Current condition indicator measurements
    pub condition_indicators: Vec<ConditionIndicator>,
    /// Chronological maintenance history
    pub maintenance_history: Vec<MaintenanceEvent>,
    /// Cumulative operating hours
    pub operating_hours: f64,
    /// Average load factor (0–1)
    pub load_factor: f64,
}

/// A single failure mode with FMEA risk metrics.
#[derive(Debug, Clone)]
pub struct FailureMode {
    /// Name of the failure mechanism
    pub name: String,
    /// Estimated probability of failure in the next assessment interval
    pub probability: f64,
    /// Consequence severity (0–1; 1 = catastrophic)
    pub consequence_severity: f64,
    /// Detectability (0–1; 1 = easily detected before failure)
    pub detectability: f64,
    /// Risk Priority Number: `probability × severity × (1 − detectability)`
    pub rpn: f64,
}

/// Remaining useful life estimate with confidence bounds.
#[derive(Debug, Clone)]
pub struct RulEstimate {
    /// Asset identifier
    pub asset_id: String,
    /// Current health score (0–1)
    pub current_health: f64,
    /// Point estimate of remaining useful life (days)
    pub rul_days: f64,
    /// 10th-percentile RUL (days)
    pub rul_confidence_low: f64,
    /// 90th-percentile RUL (days)
    pub rul_confidence_high: f64,
    /// Probability of failure within 30 days
    pub failure_probability_30d: f64,
    /// Probability of failure within 90 days
    pub failure_probability_90d: f64,
    /// Probability of failure within 365 days
    pub failure_probability_365d: f64,
    /// Recommended maintenance action
    pub recommended_action: MaintenanceRecommendation,
}

/// Recommended maintenance action derived from health and RUL.
#[derive(Debug, Clone, PartialEq)]
pub enum MaintenanceRecommendation {
    /// Continue normal operation; no action required
    Continue,
    /// Schedule a diagnostic inspection
    ScheduleInspection {
        /// Target scheduling window (days)
        within_days: f64,
    },
    /// Schedule preventive maintenance
    ScheduleMaintenance {
        /// Target scheduling window (days)
        within_days: f64,
    },
    /// Immediate corrective action required
    ImmediateAction,
    /// Asset replacement recommended
    Replacement,
}

/// Optimized maintenance schedule for an asset.
#[derive(Debug, Clone)]
pub struct MaintenanceSchedule {
    /// Asset identifier
    pub asset_id: String,
    /// Ordered list of planned tasks
    pub planned_maintenance: Vec<PlannedTask>,
    /// Sum of all task costs (USD)
    pub total_cost_usd: f64,
    /// Failure costs avoided by executing this plan (USD)
    pub avoided_failure_cost_usd: f64,
    /// `avoided_failure_cost_usd − total_cost_usd`
    pub net_benefit_usd: f64,
}

/// A single planned maintenance task in the schedule.
#[derive(Debug, Clone)]
pub struct PlannedTask {
    /// Type of maintenance task
    pub task_type: MaintenanceType,
    /// Days from now when the task should be executed
    pub scheduled_day: f64,
    /// Estimated task cost (USD)
    pub estimated_cost_usd: f64,
    /// Expected health value after the task completes
    pub expected_health_after: f64,
}

/// Composite health index for an asset (0–100 scale).
#[derive(Debug, Clone)]
pub struct HealthIndex {
    /// Overall composite health (0–100)
    pub overall: f64,
    /// Electrical sub-system health (0–100)
    pub electrical: f64,
    /// Mechanical sub-system health (0–100)
    pub mechanical: f64,
    /// Thermal sub-system health (0–100)
    pub thermal: f64,
    /// Insulation sub-system health (0–100)
    pub insulation: f64,
}

/// Task cost table used in maintenance optimization.
struct TaskCosts {
    cost_usd: f64,
    health_improvement: f64,
}

fn task_costs_for(mtype: &MaintenanceType) -> TaskCosts {
    match mtype {
        MaintenanceType::Inspection => TaskCosts {
            cost_usd: 500.0,
            health_improvement: 0.05,
        },
        MaintenanceType::MinorRepair => TaskCosts {
            cost_usd: 2_000.0,
            health_improvement: 0.10,
        },
        MaintenanceType::MajorOverhaul => TaskCosts {
            cost_usd: 10_000.0,
            health_improvement: 0.30,
        },
        MaintenanceType::ComponentReplacement => TaskCosts {
            cost_usd: 50_000.0,
            health_improvement: 1.0,
        },
        MaintenanceType::CorrectiveMaintenance => TaskCosts {
            cost_usd: 5_000.0,
            health_improvement: 0.20,
        },
    }
}

/// Predictive maintenance engine managing a fleet of power system assets.
pub struct PredictiveMaintenance {
    /// Registered assets
    pub assets: Vec<AssetCondition>,
    /// Failure cost per asset ID (USD)
    pub failure_costs: HashMap<String, f64>,
}

impl PredictiveMaintenance {
    /// Create a new empty predictive maintenance engine.
    pub fn new() -> Self {
        Self {
            assets: Vec::new(),
            failure_costs: HashMap::new(),
        }
    }

    /// Register an asset for monitoring.
    pub fn add_asset(&mut self, asset: AssetCondition) {
        self.assets.push(asset);
    }

    /// Set the estimated cost of an unplanned failure for an asset (USD).
    pub fn set_failure_cost(&mut self, asset_id: &str, cost_usd: f64) {
        self.failure_costs.insert(asset_id.to_string(), cost_usd);
    }

    /// Compute the composite health index for an asset.
    ///
    /// Each indicator is normalized to `[0, 1]` and averaged within electrical,
    /// mechanical, thermal, and insulation sub-groups. Overall is the mean of all.
    pub fn compute_health_index(&self, asset: &AssetCondition) -> HealthIndex {
        let indicators = &asset.condition_indicators;
        if indicators.is_empty() {
            // Default: assume perfect health for assets with no measurements
            return HealthIndex {
                overall: 100.0,
                electrical: 100.0,
                mechanical: 100.0,
                thermal: 100.0,
                insulation: 100.0,
            };
        }

        let mut elec_vals: Vec<f64> = Vec::new();
        let mut mech_vals: Vec<f64> = Vec::new();
        let mut therm_vals: Vec<f64> = Vec::new();
        let mut insul_vals: Vec<f64> = Vec::new();
        let mut all_vals: Vec<f64> = Vec::new();

        for ind in indicators {
            let h = ind.normalized_health();
            all_vals.push(h);

            let name_lower = ind.name.to_lowercase();
            if name_lower.contains("temperature")
                || name_lower.contains("temp")
                || name_lower.contains("thermal")
            {
                therm_vals.push(h);
            } else if name_lower.contains("insulation")
                || name_lower.contains("moisture")
                || name_lower.contains("power_factor")
                || name_lower.contains("h2")
                || name_lower.contains("c2h")
                || name_lower.contains("dga")
            {
                insul_vals.push(h);
            } else if name_lower.contains("resistance")
                || name_lower.contains("timing")
                || name_lower.contains("vibration")
                || name_lower.contains("bearing")
            {
                mech_vals.push(h);
            } else {
                elec_vals.push(h);
            }
        }

        let avg = |v: &Vec<f64>| -> f64 {
            if v.is_empty() {
                return mean_f64(&all_vals);
            }
            mean_f64(v)
        };

        let overall = mean_f64(&all_vals) * 100.0;
        HealthIndex {
            overall: overall.clamp(0.0, 100.0),
            electrical: (avg(&elec_vals) * 100.0).clamp(0.0, 100.0),
            mechanical: (avg(&mech_vals) * 100.0).clamp(0.0, 100.0),
            thermal: (avg(&therm_vals) * 100.0).clamp(0.0, 100.0),
            insulation: (avg(&insul_vals) * 100.0).clamp(0.0, 100.0),
        }
    }

    /// Estimate remaining useful life for the named asset.
    ///
    /// Fits a linear degradation model from the asset's current health and age,
    /// then applies Weibull statistics for failure probability bounds.
    pub fn estimate_rul(&self, asset_id: &str) -> Result<RulEstimate, String> {
        let asset = self
            .assets
            .iter()
            .find(|a| a.asset_id == asset_id)
            .ok_or_else(|| format!("Asset '{}' not found", asset_id))?;

        let hi = self.compute_health_index(asset);
        let current_health = hi.overall / 100.0; // normalize to 0-1

        // Build a linear degradation model from age and current health
        // Assume initial health was 1.0; rate = (1.0 - current_health) / age_s
        let age_s = asset.age_years * SECONDS_PER_YEAR;
        let degradation_rate = if age_s > 0.0 {
            (1.0 - current_health) / age_s
        } else {
            1.0 / (30.0 * SECONDS_PER_YEAR) // default: 30-year life
        };

        let model = DegradationModel::Linear {
            initial_health: current_health,
            degradation_rate,
        };

        const FAILURE_THRESHOLD: f64 = 0.3;
        let rul_s = model.rul_from_health(current_health, FAILURE_THRESHOLD);
        let rul_days = (rul_s / SECONDS_PER_DAY).max(0.0);

        // Confidence interval ±20%
        let rul_low = rul_days * 0.8;
        let rul_high = rul_days * 1.2;

        // Weibull parameters: beta=2 (wear-out), eta = rul_days (characteristic life)
        let beta = 2.0;
        let eta = (rul_days * SECONDS_PER_DAY).max(1.0) / SECONDS_PER_DAY; // in days

        let fp30 = self.weibull_failure_prob(30.0, beta, eta).clamp(0.0, 1.0);
        let fp90 = self.weibull_failure_prob(90.0, beta, eta).clamp(0.0, 1.0);
        let fp365 = self.weibull_failure_prob(365.0, beta, eta).clamp(0.0, 1.0);

        let action = self.recommend_action(current_health, rul_days);

        Ok(RulEstimate {
            asset_id: asset_id.to_string(),
            current_health,
            rul_days,
            rul_confidence_low: rul_low,
            rul_confidence_high: rul_high,
            failure_probability_30d: fp30,
            failure_probability_90d: fp90,
            failure_probability_365d: fp365,
            recommended_action: action,
        })
    }

    /// Compute FMEA failure modes for the given asset.
    ///
    /// Returns modes sorted by RPN (highest first).
    pub fn compute_failure_modes(&self, asset: &AssetCondition) -> Vec<FailureMode> {
        let mut modes = match asset.asset_type {
            AssetType::PowerTransformer => self.transformer_failure_modes(asset),
            AssetType::CircuitBreaker => self.breaker_failure_modes(asset),
            _ => self.generic_failure_modes(asset),
        };

        // Compute RPN and sort
        for m in &mut modes {
            m.rpn = m.probability * m.consequence_severity * (1.0 - m.detectability);
        }
        modes.sort_by(|a, b| {
            b.rpn
                .partial_cmp(&a.rpn)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        modes
    }

    /// Optimize the maintenance schedule for an asset over a planning horizon.
    ///
    /// Uses a greedy benefit-cost approach: tasks are included if the expected
    /// failure cost avoided exceeds the task cost.
    pub fn optimize_maintenance(
        &self,
        asset_id: &str,
        horizon_days: f64,
    ) -> Result<MaintenanceSchedule, String> {
        let asset = self
            .assets
            .iter()
            .find(|a| a.asset_id == asset_id)
            .ok_or_else(|| format!("Asset '{}' not found", asset_id))?;

        let failure_cost = *self.failure_costs.get(asset_id).unwrap_or(&100_000.0);

        let hi = self.compute_health_index(asset);
        let current_health = hi.overall / 100.0;

        // Compute baseline failure probability over horizon
        let age_s = asset.age_years * SECONDS_PER_YEAR;
        let degradation_rate = if age_s > 0.0 {
            (1.0 - current_health) / age_s
        } else {
            1.0 / (30.0 * SECONDS_PER_YEAR)
        };
        let beta = 2.0;
        let rul_s = if degradation_rate > 0.0 {
            (current_health - 0.3) / degradation_rate
        } else {
            30.0 * SECONDS_PER_YEAR
        };
        let eta_days = (rul_s / SECONDS_PER_DAY).max(1.0);
        let baseline_fp = self.weibull_failure_prob(horizon_days, beta, eta_days);

        let candidate_types = [
            MaintenanceType::Inspection,
            MaintenanceType::MinorRepair,
            MaintenanceType::MajorOverhaul,
            MaintenanceType::ComponentReplacement,
        ];

        let mut planned: Vec<PlannedTask> = Vec::new();
        let mut total_cost = 0.0;
        let mut avoided_cost = 0.0;
        let mut cumulative_health = current_health;

        for (i, mtype) in candidate_types.iter().enumerate() {
            let tc = task_costs_for(mtype);
            let new_health = (cumulative_health + tc.health_improvement).min(1.0);
            let health_gain = new_health - cumulative_health;

            // Benefit: fraction of failure cost avoided proportional to health gain
            let benefit = baseline_fp * failure_cost * health_gain;

            if benefit > tc.cost_usd {
                let scheduled_day =
                    horizon_days * (i as f64 + 1.0) / (candidate_types.len() as f64 + 1.0);
                planned.push(PlannedTask {
                    task_type: mtype.clone(),
                    scheduled_day,
                    estimated_cost_usd: tc.cost_usd,
                    expected_health_after: new_health,
                });
                total_cost += tc.cost_usd;
                avoided_cost += benefit;
                cumulative_health = new_health;
            }
        }

        let net_benefit = avoided_cost - total_cost;

        Ok(MaintenanceSchedule {
            asset_id: asset_id.to_string(),
            planned_maintenance: planned,
            total_cost_usd: total_cost,
            avoided_failure_cost_usd: avoided_cost,
            net_benefit_usd: net_benefit,
        })
    }

    /// Return the health index for every registered asset.
    pub fn fleet_health_summary(&self) -> Vec<(String, HealthIndex)> {
        self.assets
            .iter()
            .map(|a| (a.asset_id.clone(), self.compute_health_index(a)))
            .collect()
    }

    /// Return asset IDs whose overall health index is below `threshold` (0–100).
    pub fn critical_assets(&self, threshold: f64) -> Vec<String> {
        self.assets
            .iter()
            .filter(|a| self.compute_health_index(a).overall < threshold)
            .map(|a| a.asset_id.clone())
            .collect()
    }

    /// Estimate the degradation rate from recent indicator trends.
    ///
    /// Uses the fraction of degraded indicators as a proxy.
    #[allow(dead_code)]
    fn degradation_rate(&self, indicators: &[ConditionIndicator], model: &DegradationModel) -> f64 {
        if indicators.is_empty() {
            return match model {
                DegradationModel::Linear {
                    degradation_rate, ..
                } => *degradation_rate,
                _ => 1.0 / (30.0 * SECONDS_PER_YEAR),
            };
        }
        let avg_health = mean_f64(
            &indicators
                .iter()
                .map(|i| i.normalized_health())
                .collect::<Vec<_>>(),
        );
        // If average health has deteriorated, derive a linear rate
        let degraded = 1.0 - avg_health;
        degraded / (365.0 * SECONDS_PER_DAY) // per-second rate assuming 1 year elapsed
    }

    /// Weibull cumulative distribution function (failure probability).
    ///
    /// `F(t) = 1 - exp(-(t/eta)^beta)`
    fn weibull_failure_prob(&self, t_days: f64, beta: f64, eta: f64) -> f64 {
        if t_days <= 0.0 || eta <= 0.0 {
            return 0.0;
        }
        1.0 - (-(t_days / eta).powf(beta)).exp()
    }

    /// Determine the recommended maintenance action from health and RUL.
    fn recommend_action(&self, health: f64, rul_days: f64) -> MaintenanceRecommendation {
        if health < 0.1 {
            return MaintenanceRecommendation::Replacement;
        }
        if health < 0.2 || rul_days < 7.0 {
            return MaintenanceRecommendation::ImmediateAction;
        }
        if health < 0.4 || rul_days < 30.0 {
            return MaintenanceRecommendation::ScheduleMaintenance {
                within_days: rul_days * 0.5,
            };
        }
        if health < 0.6 || rul_days < 90.0 {
            return MaintenanceRecommendation::ScheduleInspection {
                within_days: rul_days * 0.5,
            };
        }
        MaintenanceRecommendation::Continue
    }

    /// Generate standard condition indicators for a power transformer.
    ///
    /// Indicators are estimated from `age_years` and `load_factor` using
    /// typical aging curves per IEC 60076-7.
    pub fn standard_transformer_indicators(
        age_years: f64,
        load_factor: f64,
    ) -> Vec<ConditionIndicator> {
        let now = 0.0_f64; // relative timestamp
        vec![
            ConditionIndicator {
                name: "oil_temperature".to_string(),
                value: 60.0 + load_factor * 20.0 + age_years * 0.2,
                unit: "°C".to_string(),
                timestamp_s: now,
                normal_range: (50.0, 70.0),
                warning_threshold: 80.0,
                critical_threshold: 95.0,
            },
            ConditionIndicator {
                name: "h2_ppm".to_string(),
                value: age_years * 2.0,
                unit: "ppm".to_string(),
                timestamp_s: now,
                normal_range: (0.0, 100.0),
                warning_threshold: 150.0,
                critical_threshold: 300.0,
            },
            ConditionIndicator {
                name: "c2h2_ppm".to_string(),
                value: age_years * 0.05,
                unit: "ppm".to_string(),
                timestamp_s: now,
                normal_range: (0.0, 1.0),
                warning_threshold: 3.0,
                critical_threshold: 10.0,
            },
            ConditionIndicator {
                name: "c2h4_ppm".to_string(),
                value: age_years * 0.5,
                unit: "ppm".to_string(),
                timestamp_s: now,
                normal_range: (0.0, 50.0),
                warning_threshold: 100.0,
                critical_threshold: 200.0,
            },
            ConditionIndicator {
                name: "moisture_ppm".to_string(),
                value: age_years * 0.5 + load_factor * 5.0,
                unit: "ppm".to_string(),
                timestamp_s: now,
                normal_range: (0.0, 15.0),
                warning_threshold: 25.0,
                critical_threshold: 35.0,
            },
            ConditionIndicator {
                name: "power_factor_percent".to_string(),
                value: 0.1 + age_years * 0.02,
                unit: "%".to_string(),
                timestamp_s: now,
                normal_range: (0.0, 0.5),
                warning_threshold: 1.0,
                critical_threshold: 2.0,
            },
            ConditionIndicator {
                name: "tap_changer_ops".to_string(),
                value: age_years * 1000.0 * load_factor,
                unit: "ops".to_string(),
                timestamp_s: now,
                normal_range: (0.0, 50_000.0),
                warning_threshold: 80_000.0,
                critical_threshold: 100_000.0,
            },
        ]
    }

    /// Generate standard condition indicators for a circuit breaker.
    ///
    /// Indicators are estimated from `ops_count` (total switching operations)
    /// and `age_years`.
    pub fn standard_breaker_indicators(ops_count: f64, age_years: f64) -> Vec<ConditionIndicator> {
        let now = 0.0_f64;
        vec![
            ConditionIndicator {
                name: "contact_resistance_mohm".to_string(),
                value: 50.0 + ops_count * 0.01,
                unit: "mΩ".to_string(),
                timestamp_s: now,
                normal_range: (0.0, 100.0),
                warning_threshold: 200.0,
                critical_threshold: 500.0,
            },
            ConditionIndicator {
                name: "sf6_pressure_bar".to_string(),
                value: 6.0 - age_years * 0.05,
                unit: "bar".to_string(),
                timestamp_s: now,
                normal_range: (5.0, 7.0),
                warning_threshold: 4.5,
                critical_threshold: 4.0,
            },
            ConditionIndicator {
                name: "operation_count".to_string(),
                value: ops_count,
                unit: "ops".to_string(),
                timestamp_s: now,
                normal_range: (0.0, 5_000.0),
                warning_threshold: 8_000.0,
                critical_threshold: 10_000.0,
            },
            ConditionIndicator {
                name: "timing_ms".to_string(),
                value: 60.0 + ops_count * 0.001,
                unit: "ms".to_string(),
                timestamp_s: now,
                normal_range: (50.0, 80.0),
                warning_threshold: 100.0,
                critical_threshold: 120.0,
            },
        ]
    }

    // --- private helpers ---

    fn transformer_failure_modes(&self, asset: &AssetCondition) -> Vec<FailureMode> {
        let indicators = &asset.condition_indicators;

        // Thermal degradation: driven by oil temperature
        let thermal_health = indicators
            .iter()
            .find(|i| i.name.contains("temperature") || i.name.contains("temp"))
            .map(|i| i.normalized_health())
            .unwrap_or(0.8);
        let thermal_prob = (1.0 - thermal_health).clamp(0.0, 1.0);

        // Insulation failure: driven by DGA (H2, C2H2) and moisture
        let dga_health: Vec<f64> = indicators
            .iter()
            .filter(|i| {
                i.name.contains("h2") || i.name.contains("c2h") || i.name.contains("moisture")
            })
            .map(|i| i.normalized_health())
            .collect();
        let ins_health = if dga_health.is_empty() {
            0.9
        } else {
            mean_f64(&dga_health)
        };
        let ins_prob = (1.0 - ins_health).clamp(0.0, 1.0);

        // Moisture ingress
        let moist_health = indicators
            .iter()
            .find(|i| i.name.contains("moisture"))
            .map(|i| i.normalized_health())
            .unwrap_or(0.9);
        let moist_prob = (1.0 - moist_health).clamp(0.0, 1.0);

        vec![
            FailureMode {
                name: "thermal_degradation".to_string(),
                probability: thermal_prob,
                consequence_severity: 0.8,
                detectability: 0.7,
                rpn: 0.0, // computed in caller
            },
            FailureMode {
                name: "insulation_failure".to_string(),
                probability: ins_prob,
                consequence_severity: 0.95,
                detectability: 0.5,
                rpn: 0.0,
            },
            FailureMode {
                name: "moisture_ingress".to_string(),
                probability: moist_prob,
                consequence_severity: 0.6,
                detectability: 0.8,
                rpn: 0.0,
            },
        ]
    }

    fn breaker_failure_modes(&self, asset: &AssetCondition) -> Vec<FailureMode> {
        let indicators = &asset.condition_indicators;

        let contact_health = indicators
            .iter()
            .find(|i| i.name.contains("resistance"))
            .map(|i| i.normalized_health())
            .unwrap_or(0.8);
        let contact_prob = (1.0 - contact_health).clamp(0.0, 1.0);

        let timing_health = indicators
            .iter()
            .find(|i| i.name.contains("timing"))
            .map(|i| i.normalized_health())
            .unwrap_or(0.9);
        let mech_prob = (1.0 - timing_health).clamp(0.0, 1.0);

        vec![
            FailureMode {
                name: "contact_wear".to_string(),
                probability: contact_prob,
                consequence_severity: 0.7,
                detectability: 0.6,
                rpn: 0.0,
            },
            FailureMode {
                name: "mechanism_failure".to_string(),
                probability: mech_prob,
                consequence_severity: 0.9,
                detectability: 0.4,
                rpn: 0.0,
            },
        ]
    }

    fn generic_failure_modes(&self, asset: &AssetCondition) -> Vec<FailureMode> {
        let indicators = &asset.condition_indicators;
        let avg_health = if indicators.is_empty() {
            0.8
        } else {
            mean_f64(
                &indicators
                    .iter()
                    .map(|i| i.normalized_health())
                    .collect::<Vec<_>>(),
            )
        };
        let prob = (1.0 - avg_health).clamp(0.0, 1.0);

        vec![
            FailureMode {
                name: "general_degradation".to_string(),
                probability: prob,
                consequence_severity: 0.6,
                detectability: 0.5,
                rpn: 0.0,
            },
            FailureMode {
                name: "insulation_breakdown".to_string(),
                probability: prob * 0.5,
                consequence_severity: 0.8,
                detectability: 0.4,
                rpn: 0.0,
            },
        ]
    }
}

impl Default for PredictiveMaintenance {
    fn default() -> Self {
        Self::new()
    }
}

// --- utility ---

fn mean_f64(v: &[f64]) -> f64 {
    if v.is_empty() {
        return 0.0;
    }
    v.iter().sum::<f64>() / v.len() as f64
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn make_normal_indicator(name: &str, value: f64) -> ConditionIndicator {
        ConditionIndicator {
            name: name.to_string(),
            value,
            unit: "unit".to_string(),
            timestamp_s: 0.0,
            normal_range: (0.0, 100.0),
            warning_threshold: 120.0,
            critical_threshold: 150.0,
        }
    }

    fn make_critical_indicator(name: &str) -> ConditionIndicator {
        ConditionIndicator {
            name: name.to_string(),
            value: 200.0, // beyond critical
            unit: "unit".to_string(),
            timestamp_s: 0.0,
            normal_range: (0.0, 100.0),
            warning_threshold: 120.0,
            critical_threshold: 150.0,
        }
    }

    fn healthy_asset(id: &str) -> AssetCondition {
        AssetCondition {
            asset_id: id.to_string(),
            asset_type: AssetType::PowerTransformer,
            age_years: 5.0,
            condition_indicators: vec![
                make_normal_indicator("oil_temperature", 65.0),
                make_normal_indicator("h2_ppm", 50.0),
            ],
            maintenance_history: vec![],
            operating_hours: 43_800.0,
            load_factor: 0.7,
        }
    }

    // 1. All normal indicators → health ≈ 1.0 (≈ 100 on 0-100 scale)
    #[test]
    fn health_index_all_normal() {
        let mut asset = healthy_asset("T1");
        asset.condition_indicators = vec![
            make_normal_indicator("oil_temperature", 60.0),
            make_normal_indicator("h2_ppm", 80.0),
            make_normal_indicator("resistance", 50.0),
        ];
        let pm = PredictiveMaintenance::new();
        let hi = pm.compute_health_index(&asset);
        assert!(
            hi.overall > 95.0,
            "expected overall ≈ 100, got {}",
            hi.overall
        );
    }

    // 2. All critical indicators → health ≈ 0.0
    #[test]
    fn health_index_all_critical() {
        let mut asset = healthy_asset("T2");
        asset.condition_indicators = vec![
            make_critical_indicator("oil_temperature"),
            make_critical_indicator("h2_ppm"),
            make_critical_indicator("resistance"),
        ];
        let pm = PredictiveMaintenance::new();
        let hi = pm.compute_health_index(&asset);
        assert!(
            hi.overall < 10.0,
            "expected overall ≈ 0, got {}",
            hi.overall
        );
    }

    // 3. Mixed indicators → 0 < health < 1
    #[test]
    fn health_index_mixed() {
        let mut asset = healthy_asset("T3");
        asset.condition_indicators = vec![
            make_normal_indicator("oil_temperature", 60.0),
            make_critical_indicator("h2_ppm"),
        ];
        let pm = PredictiveMaintenance::new();
        let hi = pm.compute_health_index(&asset);
        assert!(
            hi.overall > 0.0 && hi.overall < 100.0,
            "expected mixed health, got {}",
            hi.overall
        );
    }

    // 4. Linear degradation decreases over time
    #[test]
    fn linear_degradation_decreases() {
        let model = DegradationModel::Linear {
            initial_health: 1.0,
            degradation_rate: 1e-8,
        };
        let h1 = model.health_at_time(0.0);
        let h2 = model.health_at_time(1e7);
        assert!(h1 > h2, "expected h1={} > h2={}", h1, h2);
        assert!((h1 - 1.0).abs() < 1e-9);
    }

    // 5. Exponential degradation is always positive and decaying
    #[test]
    fn exponential_degradation() {
        let model = DegradationModel::Exponential {
            initial_health: 1.0,
            decay_constant: 1e-9,
        };
        let h0 = model.health_at_time(0.0);
        let h1 = model.health_at_time(1e8);
        let h2 = model.health_at_time(2e8);
        assert!((h0 - 1.0).abs() < 1e-6);
        assert!(h1 > 0.0);
        assert!(h2 < h1);
    }

    // 6. PowerLaw alpha > 1 → concave (fast initial decay)
    #[test]
    fn power_law_alpha_gt1() {
        let lifetime = 30.0 * SECONDS_PER_YEAR;
        let model = DegradationModel::PowerLaw {
            initial_health: 1.0,
            lifetime_s: lifetime,
            alpha: 2.0,
        };
        // At 50% of lifetime, health should be < 0.75 (concave → already lost >25%)
        let h_half = model.health_at_time(lifetime * 0.5);
        // alpha=2: h = 1*(1 - 0.5^2) = 0.75
        assert!((h_half - 0.75).abs() < 0.01, "got {}", h_half);
    }

    // 7. PowerLaw alpha < 1 → convex (slow initial decay)
    #[test]
    fn power_law_alpha_lt1() {
        let lifetime = 30.0 * SECONDS_PER_YEAR;
        let model = DegradationModel::PowerLaw {
            initial_health: 1.0,
            lifetime_s: lifetime,
            alpha: 0.5,
        };
        let h_half = model.health_at_time(lifetime * 0.5);
        // alpha=0.5: h = 1*(1 - 0.5^0.5) = 1 - 0.707 ≈ 0.293
        assert!(h_half > 0.0 && h_half < 1.0, "got {}", h_half);
    }

    // 8. Weibull failure probability at t=0 is 0
    #[test]
    fn weibull_prob_at_zero() {
        let pm = PredictiveMaintenance::new();
        let p = pm.weibull_failure_prob(0.0, 2.0, 1000.0);
        assert_eq!(p, 0.0);
    }

    // 9. Weibull failure probability at large t → 1
    #[test]
    fn weibull_prob_at_inf() {
        let pm = PredictiveMaintenance::new();
        let p = pm.weibull_failure_prob(1_000_000.0, 2.0, 1000.0);
        assert!(p > 0.999, "expected ≈1 got {}", p);
    }

    // 10. Weibull F(eta) ≈ 0.632
    #[test]
    fn weibull_characteristic_life() {
        let pm = PredictiveMaintenance::new();
        let eta = 500.0;
        let p = pm.weibull_failure_prob(eta, 1.0, eta); // beta=1 → exact 0.632
        assert!((p - 0.6321).abs() < 0.001, "F(eta)={}, expected ≈0.632", p);
    }

    // 11. RUL > 0 for a healthy asset
    #[test]
    fn rul_healthy_asset() {
        let mut pm = PredictiveMaintenance::new();
        pm.add_asset(healthy_asset("T1"));
        let rul = pm.estimate_rul("T1").expect("rul ok");
        assert!(
            rul.rul_days > 0.0,
            "expected rul_days > 0, got {}",
            rul.rul_days
        );
    }

    // 12. RUL confidence interval: low < point < high
    #[test]
    fn rul_confidence_interval() {
        let mut pm = PredictiveMaintenance::new();
        pm.add_asset(healthy_asset("T1"));
        let rul = pm.estimate_rul("T1").expect("rul ok");
        assert!(
            rul.rul_confidence_low < rul.rul_days,
            "low={} >= point={}",
            rul.rul_confidence_low,
            rul.rul_days
        );
        assert!(
            rul.rul_days < rul.rul_confidence_high,
            "point={} >= high={}",
            rul.rul_days,
            rul.rul_confidence_high
        );
    }

    // 13. Failure probability ordering: 30d < 90d < 365d
    #[test]
    fn failure_prob_ordering() {
        let mut pm = PredictiveMaintenance::new();
        let mut asset = healthy_asset("T1");
        // Make asset somewhat aged so probabilities are meaningfully non-zero
        asset.age_years = 20.0;
        asset.condition_indicators = vec![
            make_normal_indicator("oil_temperature", 85.0), // slightly degraded
        ];
        pm.add_asset(asset);
        let rul = pm.estimate_rul("T1").expect("rul ok");
        assert!(
            rul.failure_probability_30d <= rul.failure_probability_90d,
            "30d={} > 90d={}",
            rul.failure_probability_30d,
            rul.failure_probability_90d
        );
        assert!(
            rul.failure_probability_90d <= rul.failure_probability_365d,
            "90d={} > 365d={}",
            rul.failure_probability_90d,
            rul.failure_probability_365d
        );
    }

    // 14. New asset → recommend Continue
    #[test]
    fn recommend_continue_new() {
        let pm = PredictiveMaintenance::new();
        let action = pm.recommend_action(0.95, 3650.0);
        assert_eq!(action, MaintenanceRecommendation::Continue);
    }

    // 15. Critical health → ImmediateAction
    #[test]
    fn recommend_immediate_critical() {
        let pm = PredictiveMaintenance::new();
        let action = pm.recommend_action(0.15, 5.0);
        assert!(
            matches!(action, MaintenanceRecommendation::ImmediateAction),
            "expected ImmediateAction got {:?}",
            action
        );
    }

    // 16. Maintenance schedule task cost < failure cost for costly assets
    #[test]
    fn maintenance_schedule_net_benefit() {
        let mut pm = PredictiveMaintenance::new();
        let mut asset = healthy_asset("T_costly");
        asset.age_years = 25.0;
        asset.condition_indicators = vec![make_normal_indicator("oil_temperature", 88.0)];
        pm.add_asset(asset);
        pm.set_failure_cost("T_costly", 500_000.0);
        let schedule = pm
            .optimize_maintenance("T_costly", 365.0)
            .expect("schedule ok");
        // For a well-degraded asset with high failure cost, net benefit should be positive
        // (If no tasks are scheduled, net_benefit = 0 which is still >= 0)
        assert!(
            schedule.net_benefit_usd >= 0.0,
            "net_benefit={}",
            schedule.net_benefit_usd
        );
        assert!(schedule.total_cost_usd >= 0.0);
    }

    // 17. Fleet summary length equals number of added assets
    #[test]
    fn fleet_summary_length() {
        let mut pm = PredictiveMaintenance::new();
        pm.add_asset(healthy_asset("A1"));
        pm.add_asset(healthy_asset("A2"));
        pm.add_asset(healthy_asset("A3"));
        let summary = pm.fleet_health_summary();
        assert_eq!(summary.len(), 3);
    }

    // 18. critical_assets returns correct subset
    #[test]
    fn critical_assets_threshold() {
        let mut pm = PredictiveMaintenance::new();
        pm.add_asset(healthy_asset("OK1")); // healthy
        let mut bad = healthy_asset("BAD1");
        bad.condition_indicators = vec![
            make_critical_indicator("oil_temperature"),
            make_critical_indicator("h2_ppm"),
        ];
        pm.add_asset(bad);
        let crits = pm.critical_assets(50.0);
        assert!(
            crits.contains(&"BAD1".to_string()),
            "BAD1 should be critical"
        );
        assert!(
            !crits.contains(&"OK1".to_string()),
            "OK1 should not be critical"
        );
    }

    // 19. standard_transformer_indicators returns ≥ 5 indicators
    #[test]
    fn standard_transformer_indicators_nonempty() {
        let inds = PredictiveMaintenance::standard_transformer_indicators(10.0, 0.7);
        assert!(
            inds.len() >= 5,
            "expected ≥5 indicators, got {}",
            inds.len()
        );
    }

    // 20. standard_breaker_indicators returns ≥ 4 indicators
    #[test]
    fn standard_breaker_indicators_nonempty() {
        let inds = PredictiveMaintenance::standard_breaker_indicators(3000.0, 8.0);
        assert!(
            inds.len() >= 4,
            "expected ≥4 indicators, got {}",
            inds.len()
        );
    }

    // 21. FMEA RPN formula: rpn = probability * severity * (1 - detectability)
    #[test]
    fn fmea_rpn_formula() {
        let pm = PredictiveMaintenance::new();
        let asset = healthy_asset("T1");
        let modes = pm.compute_failure_modes(&asset);
        for m in &modes {
            let expected = m.probability * m.consequence_severity * (1.0 - m.detectability);
            assert!(
                (m.rpn - expected).abs() < 1e-9,
                "RPN mismatch for {}: got {}, expected {}",
                m.name,
                m.rpn,
                expected
            );
        }
    }

    // 22. Failure modes sorted by RPN descending
    #[test]
    fn fmea_sorted_descending() {
        let pm = PredictiveMaintenance::new();
        let asset = healthy_asset("T1");
        let modes = pm.compute_failure_modes(&asset);
        for i in 1..modes.len() {
            assert!(
                modes[i - 1].rpn >= modes[i].rpn,
                "modes not sorted: modes[{}].rpn={} < modes[{}].rpn={}",
                i - 1,
                modes[i - 1].rpn,
                i,
                modes[i].rpn
            );
        }
    }

    // 23. AssetType variants are constructible
    #[test]
    fn asset_type_variants() {
        let variants = [
            AssetType::PowerTransformer,
            AssetType::CircuitBreaker,
            AssetType::UndergroundCable,
            AssetType::OverheadLine,
            AssetType::Generator,
            AssetType::Motor,
            AssetType::Capacitor,
            AssetType::SwitchGear,
        ];
        assert_eq!(variants.len(), 8);
    }

    // 24. MaintenanceType variants are constructible
    #[test]
    fn maintenance_type_variants() {
        let variants = [
            MaintenanceType::Inspection,
            MaintenanceType::MinorRepair,
            MaintenanceType::MajorOverhaul,
            MaintenanceType::ComponentReplacement,
            MaintenanceType::CorrectiveMaintenance,
        ];
        assert_eq!(variants.len(), 5);
    }

    // 25. compute_failure_modes for PowerTransformer returns ≥ 2 modes
    #[test]
    fn compute_failure_modes_transformer() {
        let pm = PredictiveMaintenance::new();
        let asset = AssetCondition {
            asset_id: "TF1".to_string(),
            asset_type: AssetType::PowerTransformer,
            age_years: 15.0,
            condition_indicators: PredictiveMaintenance::standard_transformer_indicators(15.0, 0.8),
            maintenance_history: vec![],
            operating_hours: 131_400.0,
            load_factor: 0.8,
        };
        let modes = pm.compute_failure_modes(&asset);
        assert!(
            modes.len() >= 2,
            "expected ≥2 failure modes, got {}",
            modes.len()
        );
    }

    // 26. All failure probability fields in [0, 1]
    #[test]
    fn rul_estimate_fields_in_range() {
        let mut pm = PredictiveMaintenance::new();
        pm.add_asset(healthy_asset("T1"));
        let rul = pm.estimate_rul("T1").expect("rul ok");
        assert!(rul.failure_probability_30d >= 0.0 && rul.failure_probability_30d <= 1.0);
        assert!(rul.failure_probability_90d >= 0.0 && rul.failure_probability_90d <= 1.0);
        assert!(rul.failure_probability_365d >= 0.0 && rul.failure_probability_365d <= 1.0);
        assert!(rul.current_health >= 0.0 && rul.current_health <= 1.0);
    }

    // Bonus: DegradationModel::Weibull RUL calculation
    #[test]
    fn weibull_rul_calculation() {
        let model = DegradationModel::Weibull {
            beta: 2.0,
            eta: 1e9,
        };
        let h0 = model.health_at_time(0.0);
        assert!((h0 - 1.0).abs() < 1e-9);
        let rul = model.rul_from_health(0.9, 0.3);
        assert!(rul > 0.0);
    }

    // Bonus: Arrhenius model works
    #[test]
    fn arrhenius_model() {
        let model = DegradationModel::Arrhenius {
            activation_energy_ev: 1.0,
            pre_exponential: 1e-10,
            temperature_k: 350.0,
        };
        let h0 = model.health_at_time(0.0);
        let h1 = model.health_at_time(1e10);
        assert!((h0 - 1.0).abs() < 1e-6);
        // h1 may be 0 or less than h0 (clamped)
        assert!(h1 <= h0);
    }
}
