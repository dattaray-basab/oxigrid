//! Digital Asset Lifecycle Management for power grid equipment.
//!
//! Provides digital asset records with inspection history, condition scoring,
//! predictive maintenance scheduling (Weibull failure model), DGA analysis
//! (Rogers Ratio method), Asset Health Index computation, and criticality-based
//! prioritization for capital planning.
//!
//! # Units
//! - Time: Unix epoch seconds \[s\], ages in \[years\]
//! - Costs: US Dollars \[USD\]
//! - Gas concentrations: parts-per-million \[ppm\]
//! - Condition score: 0–5 (0 = failed, 5 = new/excellent)
//! - Criticality score: 0–1 (1 = most critical)
//! - Health Index: 0–100

use std::collections::HashMap;

/// Seconds per Julian year (365.25 days).
const SECS_PER_YEAR: f64 = 365.25 * 24.0 * 3600.0;

/// Current reference epoch used when no external clock is supplied \[s\].
/// Corresponds to 2026-03-09 00:00:00 UTC.
const NOW_UNIX: f64 = 1_741_478_400.0 + 365.25 * 24.0 * 3600.0; // 2026-03-09

/// Value of Lost Load \[USD/MWh\].
const VOLL: f64 = 10_000.0;

/// Default energy not served per asset failure \[MWh\].
const DEFAULT_ENS_MWH: f64 = 100.0;

// ─────────────────────────────────────────────────────────────────────────────
// Asset classification
// ─────────────────────────────────────────────────────────────────────────────

/// Classification of a power grid asset, carrying class-specific nameplate data.
#[derive(Debug, Clone)]
pub enum AssetClass {
    /// Oil-filled or dry-type power transformer.
    PowerTransformer {
        /// Rated apparent power \[MVA\].
        mva_rating: f64,
        /// Nominal voltage (highest winding) \[kV\].
        voltage_kv: f64,
        /// IEC vector group (e.g. `"Dyn11"`).
        vector_group: String,
    },
    /// High-voltage circuit breaker (SF6, vacuum, oil).
    CircuitBreaker {
        /// Rated voltage \[kV\].
        rated_kv: f64,
        /// Rated normal current \[kA\].
        rated_ka: f64,
        /// Short-circuit interrupting capacity \[kA\].
        interrupting_capacity_ka: f64,
    },
    /// Overhead or underground transmission/distribution line.
    TransmissionLine {
        /// Operating voltage \[kV\].
        voltage_kv: f64,
        /// Route length \[km\].
        length_km: f64,
        /// Conductor type (e.g. `"ACSR 400mm²"`).
        conductor_type: String,
    },
    /// Tower, pole, or other overhead line support structure.
    OverheadLineStructure {
        /// Structure material / design class (e.g. `"Lattice Steel"`).
        structure_type: String,
        /// Nominal height above ground \[m\].
        height_m: f64,
    },
    /// Instrument transformer (potential or current).
    MeasuringTransformer {
        /// `"PT"` for voltage / potential transformer, `"CT"` for current transformer.
        pt_or_ct: String,
        /// Rated primary voltage \[kV\].
        rated_kv: f64,
    },
    /// Numerical or electromechanical protection relay.
    ProtectionRelay {
        /// Relay manufacturer.
        make: String,
        /// Relay model designation.
        model: String,
        /// Communication protocol (e.g. `"IEC 61850"`, `"DNP3"`).
        protocol: String,
    },
    /// Stationary battery bank (BESS or UPS duty).
    BatteryBank {
        /// Nominal energy capacity \[kWh\].
        capacity_kwh: f64,
        /// Electrochemical chemistry (e.g. `"VRLA"`, `"Li-NMC"`).
        chemistry: String,
        /// Number of cells in series per string.
        cells_in_series: u32,
    },
}

impl AssetClass {
    /// Characteristic life η used in the Weibull failure model \[years\].
    pub fn characteristic_life_years(&self) -> f64 {
        match self {
            AssetClass::PowerTransformer { .. } => 40.0,
            AssetClass::CircuitBreaker { .. } => 30.0,
            AssetClass::TransmissionLine { .. } => 50.0,
            AssetClass::OverheadLineStructure { .. } => 60.0,
            AssetClass::MeasuringTransformer { .. } => 35.0,
            AssetClass::ProtectionRelay { .. } => 25.0,
            AssetClass::BatteryBank { .. } => 15.0,
        }
    }

    /// Typical replacement cost \[USD\].
    pub fn replacement_cost_usd(&self) -> f64 {
        match self {
            AssetClass::PowerTransformer { .. } => 500_000.0,
            AssetClass::CircuitBreaker { .. } => 50_000.0,
            AssetClass::TransmissionLine { .. } => 200_000.0,
            AssetClass::OverheadLineStructure { .. } => 20_000.0,
            AssetClass::MeasuringTransformer { .. } => 15_000.0,
            AssetClass::ProtectionRelay { .. } => 10_000.0,
            AssetClass::BatteryBank { .. } => 100_000.0,
        }
    }

    /// Estimated preventive maintenance cost per event \[USD\].
    pub fn maintenance_cost_usd(&self) -> f64 {
        match self {
            AssetClass::PowerTransformer { .. } => 20_000.0,
            AssetClass::CircuitBreaker { .. } => 5_000.0,
            AssetClass::TransmissionLine { .. } => 15_000.0,
            AssetClass::OverheadLineStructure { .. } => 2_000.0,
            AssetClass::MeasuringTransformer { .. } => 1_500.0,
            AssetClass::ProtectionRelay { .. } => 1_000.0,
            AssetClass::BatteryBank { .. } => 8_000.0,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Inspection types and records
// ─────────────────────────────────────────────────────────────────────────────

/// Category of inspection performed on a grid asset.
#[derive(Debug, Clone)]
pub enum InspectionType {
    /// Unaided eye survey of physical condition.
    Visual,
    /// Infrared thermographic survey.
    Thermal {
        /// Whether a calibrated thermal camera was used (vs. spot pyrometer).
        camera_used: bool,
    },
    /// Dissolved Gas Analysis of transformer insulating oil.
    DGA,
    /// Partial Discharge detection (acoustic, UHF, or electrical).
    PartialDischarge,
    /// Insulation Resistance / Polarisation Index test.
    InsulationResistance,
    /// Contact Resistance measurement (μΩ).
    ContactResistance,
    /// Vibration / acoustic signature analysis.
    Vibration,
    /// Ultrasonic DGA (online, non-intrusive).
    UltrasonicDGA,
}

/// Severity classification assigned to a defect found during inspection.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum DefectSeverity {
    /// No defect observed.
    None,
    /// Cosmetic or negligible defect; monitor only.
    Minor,
    /// Notable degradation; schedule corrective action.
    Moderate,
    /// Substantial degradation; expedite maintenance.
    Significant,
    /// Imminent risk of failure; urgent action required.
    Critical,
    /// Asset has reached end of serviceable life; replacement required.
    EndOfLife,
}

/// Full record of a single asset inspection event.
#[derive(Debug, Clone)]
pub struct InspectionRecord {
    /// Unique identifier for this inspection event.
    pub inspection_id: String,
    /// Inspection date as Unix epoch \[s\].
    pub date_unix: f64,
    /// Type of inspection performed.
    pub inspection_type: InspectionType,
    /// Employee or contractor identifier of the inspector.
    pub inspector_id: String,
    /// Free-text findings noted during inspection.
    pub findings: Vec<String>,
    /// Overall condition score assigned (0.0 = failed, 5.0 = new/excellent).
    pub condition_score: f64,
    /// Recommended corrective or preventive actions.
    pub recommended_actions: Vec<String>,
    /// Highest severity defect identified.
    pub defect_severity: DefectSeverity,
}

// ─────────────────────────────────────────────────────────────────────────────
// Maintenance records
// ─────────────────────────────────────────────────────────────────────────────

/// Category of maintenance work performed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MaintenanceType {
    /// Repair after an observed failure or defect.
    Corrective,
    /// Scheduled time-based servicing.
    Preventive,
    /// Condition-based maintenance triggered by monitoring data.
    Predictive,
    /// Unplanned response to an emergency.
    Emergency,
    /// Major refurbishment restoring near-new condition.
    Overhaul,
}

/// Record of a maintenance intervention on a grid asset.
#[derive(Debug, Clone)]
pub struct MaintenanceRecord {
    /// Maintenance date as Unix epoch \[s\].
    pub date_unix: f64,
    /// Category of maintenance performed.
    pub maintenance_type: MaintenanceType,
    /// Total cost of the maintenance activity \[USD\].
    pub cost_usd: f64,
    /// Duration of asset outage associated with maintenance \[h\].
    pub downtime_h: f64,
    /// Organisation or individual who performed the work.
    pub performed_by: String,
    /// List of material parts or components replaced.
    pub parts_replaced: Vec<String>,
}

// ─────────────────────────────────────────────────────────────────────────────
// Digital asset record
// ─────────────────────────────────────────────────────────────────────────────

/// Complete digital record for a single power grid asset.
#[derive(Debug, Clone)]
pub struct DigitalAssetRecord {
    /// Unique asset identifier (e.g. plant tag number).
    pub asset_id: String,
    /// Asset classification with nameplate parameters.
    pub asset_class: AssetClass,
    /// Date the asset was commissioned / energised, as Unix epoch \[s\].
    pub installation_date_unix: f64,
    /// Original equipment manufacturer.
    pub manufacturer: String,
    /// Manufacturer model designation.
    pub model: String,
    /// Factory serial number.
    pub serial_number: String,
    /// Parent substation or switchyard identifier.
    pub substation_id: String,
    /// Network bus index to which the asset is connected.
    pub bus_id: usize,
    /// Chronological list of all inspection events.
    pub inspection_history: Vec<InspectionRecord>,
    /// Chronological list of all maintenance interventions.
    pub maintenance_history: Vec<MaintenanceRecord>,
    /// Cumulative energised operating time \[h\].
    pub operating_hours: f64,
    /// Average loading expressed as fraction of rated capacity (0–1).
    pub load_factor_avg: f64,
    /// Network criticality weighting factor (0 = non-critical, 1 = most critical).
    pub criticality_score: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Maintenance scheduler
// ─────────────────────────────────────────────────────────────────────────────

/// Fleet-level maintenance planning engine.
///
/// Holds a portfolio of [`DigitalAssetRecord`]s and exposes methods for
/// condition assessment, failure prediction, risk scoring, and capital planning.
#[derive(Debug, Clone)]
pub struct MaintenanceScheduler {
    /// Portfolio of assets under management.
    pub assets: Vec<DigitalAssetRecord>,
    /// Annual capital / O&M budget available for maintenance \[USD/yr\].
    pub budget_annual_usd: f64,
    /// Number of years over which to plan maintenance activities \[years\].
    pub planning_horizon_years: usize,
}

// ─────────────────────────────────────────────────────────────────────────────
// Output structs
// ─────────────────────────────────────────────────────────────────────────────

/// A single scheduled maintenance action within a [`MaintenancePlan`].
#[derive(Debug, Clone)]
pub struct ScheduledMaintenance {
    /// Asset targeted by this maintenance action.
    pub asset_id: String,
    /// Type of maintenance to be performed.
    pub maintenance_type: MaintenanceType,
    /// Calendar year in which maintenance is planned (1-indexed from current year).
    pub scheduled_year: usize,
    /// Estimated total cost of the maintenance activity \[USD\].
    pub estimated_cost_usd: f64,
    /// Risk score before the maintenance intervention.
    pub risk_before: f64,
    /// Estimated residual risk score after the maintenance intervention.
    pub risk_after: f64,
}

/// Output of [`MaintenanceScheduler::generate_maintenance_plan`].
#[derive(Debug, Clone)]
pub struct MaintenancePlan {
    /// Ordered list of scheduled maintenance actions (highest risk first).
    pub scheduled_actions: Vec<ScheduledMaintenance>,
    /// Sum of estimated costs for all scheduled actions \[USD\].
    pub total_cost_usd: f64,
    /// Estimated percentage reduction in total fleet risk after executing the plan.
    pub risk_reduction_pct: f64,
    /// Asset IDs that exceeded the risk threshold but could not be funded.
    pub assets_not_covered: Vec<String>,
}

/// Output of [`MaintenanceScheduler::dga_analysis`] (Dissolved Gas Analysis).
#[derive(Debug, Clone)]
pub struct DgaResult {
    /// Human-readable fault type classification.
    pub fault_type: String,
    /// Severity of the identified fault condition.
    pub severity: DefectSeverity,
    /// Rogers Ratios: (CH₄/H₂, C₂H₂/C₂H₄, C₂H₄/C₂H₆).
    pub rogers_ratios: (f64, f64, f64),
    /// Recommended corrective action.
    pub recommended_action: String,
}

/// Output of [`MaintenanceScheduler::asset_health_index`].
#[derive(Debug, Clone)]
pub struct AssetHealthReport {
    /// Asset identifier.
    pub asset_id: String,
    /// Composite health index (0 = end-of-life, 100 = new/excellent).
    pub health_index: f64,
    /// Qualitative condition band: `"Good"` / `"Fair"` / `"Poor"` / `"Critical"`.
    pub condition_band: String,
    /// Estimated remaining serviceable life \[years\].
    pub remaining_life_years: f64,
    /// `true` if the asset requires priority maintenance attention.
    pub priority_maintenance: bool,
}

/// A single asset replacement recommendation from [`MaintenanceScheduler::replacement_prioritization`].
#[derive(Debug, Clone)]
pub struct ReplacementRecommendation {
    /// Asset identifier.
    pub asset_id: String,
    /// Explanation of why replacement is recommended.
    pub reason: String,
    /// Urgency qualifier: `"Immediate"` / `"Within 1yr"` / `"Within 5yr"`.
    pub urgency: String,
    /// Estimated replacement cost \[USD\].
    pub replacement_cost_usd: f64,
    /// Return-on-investment score used for ranking (higher = replace sooner).
    pub roi_score: f64,
}

/// Summary statistics for the entire asset fleet.
#[derive(Debug, Clone)]
pub struct FleetReport {
    /// Population-mean asset age \[years\].
    pub mean_age_years: f64,
    /// Median current condition score across all assets (0–5 scale).
    pub median_condition: f64,
    /// Count of assets in each 1-point condition band: `"0-1"`, `"1-2"`, …, `"4-5"`.
    pub condition_distribution: Vec<(String, usize)>,
    /// Sum of replacement costs for all assets \[USD\].
    pub total_replacement_value_usd: f64,
    /// Number of assets whose risk score exceeds 0.1.
    pub high_risk_assets: usize,
}

// ─────────────────────────────────────────────────────────────────────────────
// MaintenanceScheduler implementation
// ─────────────────────────────────────────────────────────────────────────────

impl MaintenanceScheduler {
    /// Create a new scheduler with the given asset portfolio and financial parameters.
    pub fn new(
        assets: Vec<DigitalAssetRecord>,
        budget_annual_usd: f64,
        planning_horizon_years: usize,
    ) -> Self {
        Self {
            assets,
            budget_annual_usd,
            planning_horizon_years,
        }
    }

    /// Return the current condition score for `asset` \[0–5\].
    ///
    /// Starts from the most recent inspection score and applies a linear
    /// degradation of 0.1 per year since that inspection.  Returns a default
    /// of 3.0 if no inspections exist.
    pub fn current_condition_score(&self, asset: &DigitalAssetRecord) -> f64 {
        let latest = asset.inspection_history.iter().max_by(|a, b| {
            a.date_unix
                .partial_cmp(&b.date_unix)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        match latest {
            None => 3.0,
            Some(rec) => {
                let years_since = (NOW_UNIX - rec.date_unix) / SECS_PER_YEAR;
                let degraded = rec.condition_score - 0.1 * years_since.max(0.0);
                degraded.clamp(0.0, 5.0)
            }
        }
    }

    /// Estimate the probability of failure within `horizon_years` using a
    /// two-parameter Weibull model P(T < t) = 1 − exp(−(t/η)^β).
    ///
    /// β = 2.5 (typical for electrical HV equipment).
    /// η is the class-specific characteristic life, reduced by 20 % when the
    /// current condition score is below 3.0.
    pub fn predict_failure_probability(
        &self,
        asset: &DigitalAssetRecord,
        horizon_years: f64,
    ) -> f64 {
        let beta: f64 = 2.5;
        let mut eta = asset.asset_class.characteristic_life_years();

        let age = (NOW_UNIX - asset.installation_date_unix) / SECS_PER_YEAR;
        let condition = self.current_condition_score(asset);
        if condition < 3.0 {
            eta *= 0.8;
        }

        let t = (age + horizon_years).max(0.0);
        let p = 1.0 - (-(t / eta).powf(beta)).exp();
        p.clamp(0.0, 1.0)
    }

    /// Compute a dimensionless risk score for `asset`.
    ///
    /// risk = P(failure in 1 yr) × criticality × consequence \[MUSD\]
    pub fn calculate_risk_score(&self, asset: &DigitalAssetRecord) -> f64 {
        let failure_prob = self.predict_failure_probability(asset, 1.0);
        let consequence_usd = asset.asset_class.replacement_cost_usd() + DEFAULT_ENS_MWH * VOLL;
        failure_prob * asset.criticality_score * consequence_usd / 1e6
    }

    /// Build a risk-prioritised maintenance plan constrained by the available
    /// budget over `planning_horizon_years`.
    ///
    /// Assets with a risk score above `risk_threshold` are candidates.  They
    /// are sorted by risk (highest first) and scheduled until the budget is
    /// exhausted.  The risk is assumed to fall by 70 % after each maintenance
    /// event.
    pub fn generate_maintenance_plan(&self, risk_threshold: f64) -> MaintenancePlan {
        let total_budget = self.budget_annual_usd * self.planning_horizon_years as f64;

        // Collect candidate (asset, risk) pairs above threshold.
        let mut candidates: Vec<(&DigitalAssetRecord, f64)> = self
            .assets
            .iter()
            .map(|a| (a, self.calculate_risk_score(a)))
            .filter(|(_, r)| *r > risk_threshold)
            .collect();

        // Sort highest risk first.
        candidates.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        let total_risk_all: f64 = self
            .assets
            .iter()
            .map(|a| self.calculate_risk_score(a))
            .sum();

        let mut scheduled_actions: Vec<ScheduledMaintenance> = Vec::new();
        let mut total_cost_usd = 0.0;
        let mut total_risk_before = 0.0;
        let mut total_risk_after = 0.0;
        let mut assets_not_covered: Vec<String> = Vec::new();
        let mut remaining_budget = total_budget;

        for (asset, risk_before) in &candidates {
            let cost = asset.asset_class.maintenance_cost_usd();
            if cost <= remaining_budget {
                let risk_after = risk_before * 0.3;
                // Spread over horizon; assign to earliest available year slot.
                let year_slot = scheduled_actions.len() % self.planning_horizon_years.max(1) + 1;
                scheduled_actions.push(ScheduledMaintenance {
                    asset_id: asset.asset_id.clone(),
                    maintenance_type: MaintenanceType::Predictive,
                    scheduled_year: year_slot,
                    estimated_cost_usd: cost,
                    risk_before: *risk_before,
                    risk_after,
                });
                total_cost_usd += cost;
                total_risk_before += risk_before;
                total_risk_after += risk_after;
                remaining_budget -= cost;
            } else {
                assets_not_covered.push(asset.asset_id.clone());
            }
        }

        let risk_reduction_pct = if total_risk_all > 0.0 {
            (total_risk_before - total_risk_after) / total_risk_all * 100.0
        } else {
            0.0
        };

        MaintenancePlan {
            scheduled_actions,
            total_cost_usd,
            risk_reduction_pct,
            assets_not_covered,
        }
    }

    /// Diagnose transformer oil health using the Rogers Ratio method.
    ///
    /// `gas_ppm` must contain keys `"CH4"`, `"C2H2"`, `"C2H4"`, `"C2H6"`, `"H2"`.
    /// Returns `Err` if the asset is not a `PowerTransformer` or if a required
    /// gas key is missing.
    pub fn dga_analysis(
        &self,
        gas_ppm: &HashMap<String, f64>,
        asset: &DigitalAssetRecord,
    ) -> Result<DgaResult, String> {
        // Only valid for transformers.
        match &asset.asset_class {
            AssetClass::PowerTransformer { .. } => {}
            _ => {
                return Err(format!(
                    "DGA analysis only applicable to PowerTransformer, asset {} is {:?}",
                    asset.asset_id,
                    std::mem::discriminant(&asset.asset_class)
                ))
            }
        }

        let get = |key: &str| -> Result<f64, String> {
            gas_ppm
                .get(key)
                .copied()
                .ok_or_else(|| format!("Missing gas key '{}' in gas_ppm map", key))
        };

        let ch4 = get("CH4")?;
        let c2h2 = get("C2H2")?;
        let c2h4 = get("C2H4")?;
        let c2h6 = get("C2H6")?;
        let h2 = get("H2")?;

        // Avoid division by zero.
        let r1 = if h2 > 0.0 { ch4 / h2 } else { 0.0 };
        let r2 = if c2h4 > 0.0 { c2h2 / c2h4 } else { 0.0 };
        let r3 = if c2h6 > 0.0 { c2h4 / c2h6 } else { 0.0 };

        let (fault_type, severity, recommended_action) = if r2 > 1.0 && c2h2 > 5.0 {
            (
                "Electrical Discharge (Arcing)".to_string(),
                DefectSeverity::Critical,
                "Immediate de-energisation and internal inspection required.".to_string(),
            )
        } else if r2 > 0.1 && c2h2 > 1.0 {
            (
                "Low-Energy Electrical Discharge".to_string(),
                DefectSeverity::Significant,
                "Schedule urgent internal inspection within 30 days.".to_string(),
            )
        } else if r3 > 4.0 && r1 > 1.0 {
            (
                "High-Temperature Thermal Fault (>700°C)".to_string(),
                DefectSeverity::Critical,
                "Reduce loading immediately; schedule emergency inspection.".to_string(),
            )
        } else if r3 > 1.0 && r1 > 0.5 {
            (
                "Medium-Temperature Thermal Fault (300-700°C)".to_string(),
                DefectSeverity::Moderate,
                "Increase DGA sampling frequency; plan maintenance outage.".to_string(),
            )
        } else if r3 <= 1.0 && r1 < 0.5 {
            (
                "Normal Aging / Low Temperature Thermal".to_string(),
                DefectSeverity::Minor,
                "Continue routine monitoring; next DGA within 12 months.".to_string(),
            )
        } else {
            (
                "Normal".to_string(),
                DefectSeverity::None,
                "No action required; maintain standard monitoring schedule.".to_string(),
            )
        };

        Ok(DgaResult {
            fault_type,
            severity,
            rogers_ratios: (r1, r2, r3),
            recommended_action,
        })
    }

    /// Compute the Asset Health Index (AHI) for `asset` \[0–100\].
    ///
    /// AHI = 100 × (0.4 × condition_norm + 0.3 × age_score + 0.3 × maint_eff)
    ///
    /// where:
    /// - `condition_norm` = current_condition_score / 5
    /// - `age_score` = 1 − age / expected_life (clamped 0–1)
    /// - `maint_eff` = fraction of maintenance actions that are preventive or predictive
    pub fn asset_health_index(&self, asset: &DigitalAssetRecord) -> AssetHealthReport {
        let condition = self.current_condition_score(asset);
        let condition_norm = (condition / 5.0).clamp(0.0, 1.0);

        let expected_life = asset.asset_class.characteristic_life_years();
        let age =
            ((NOW_UNIX - asset.installation_date_unix) / SECS_PER_YEAR).clamp(0.0, expected_life);
        let age_score = (1.0 - age / expected_life).clamp(0.0, 1.0);

        let maint_eff = if asset.maintenance_history.is_empty() {
            0.0
        } else {
            let proactive = asset
                .maintenance_history
                .iter()
                .filter(|m| {
                    m.maintenance_type == MaintenanceType::Preventive
                        || m.maintenance_type == MaintenanceType::Predictive
                })
                .count();
            proactive as f64 / asset.maintenance_history.len() as f64
        };

        let ahi = (0.4 * condition_norm + 0.3 * age_score + 0.3 * maint_eff) * 100.0;
        let ahi = ahi.clamp(0.0, 100.0);

        let condition_band = if ahi >= 75.0 {
            "Good".to_string()
        } else if ahi >= 50.0 {
            "Fair".to_string()
        } else if ahi >= 25.0 {
            "Poor".to_string()
        } else {
            "Critical".to_string()
        };

        let remaining_life_years = (expected_life - age).clamp(0.0, expected_life);
        let priority_maintenance = ahi < 50.0;

        AssetHealthReport {
            asset_id: asset.asset_id.clone(),
            health_index: ahi,
            condition_band,
            remaining_life_years,
            priority_maintenance,
        }
    }

    /// Identify assets that should be replaced and rank them by ROI score
    /// within the given `budget` \[USD\].
    ///
    /// ROI score = (failure_prob × criticality) / (replacement_cost / 1e6)
    pub fn replacement_prioritization(&self, budget: f64) -> Vec<ReplacementRecommendation> {
        let mut candidates: Vec<(&DigitalAssetRecord, f64, f64, String, String)> = Vec::new();

        for asset in &self.assets {
            let condition = self.current_condition_score(asset);
            let latest_severity = asset
                .inspection_history
                .iter()
                .max_by(|a, b| {
                    a.date_unix
                        .partial_cmp(&b.date_unix)
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .map(|r| &r.defect_severity);

            let is_eol = latest_severity == Some(&DefectSeverity::EndOfLife);
            let is_critical_defect = latest_severity == Some(&DefectSeverity::Critical);
            let needs_replacement = condition < 2.0 || is_eol || is_critical_defect;

            if !needs_replacement {
                continue;
            }

            let failure_prob = self.predict_failure_probability(asset, 1.0);
            let replacement_cost = asset.asset_class.replacement_cost_usd();
            let roi_score =
                (failure_prob * asset.criticality_score) / (replacement_cost / 1e6).max(1e-9);

            let urgency = if condition < 1.0 || is_eol {
                "Immediate".to_string()
            } else if condition < 2.0 {
                "Within 1yr".to_string()
            } else {
                "Within 5yr".to_string()
            };

            let reason = if is_eol {
                "Asset has reached end-of-life per latest inspection.".to_string()
            } else if is_critical_defect {
                "Critical defect identified in latest inspection.".to_string()
            } else {
                format!(
                    "Condition score {:.2} below replacement threshold of 2.0.",
                    condition
                )
            };

            candidates.push((asset, roi_score, replacement_cost, reason, urgency));
        }

        // Sort by ROI score descending.
        candidates.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        let mut recommendations: Vec<ReplacementRecommendation> = Vec::new();
        let mut remaining_budget = budget;

        for (asset, roi_score, replacement_cost, reason, urgency) in candidates {
            if replacement_cost <= remaining_budget {
                recommendations.push(ReplacementRecommendation {
                    asset_id: asset.asset_id.clone(),
                    reason,
                    urgency,
                    replacement_cost_usd: replacement_cost,
                    roi_score,
                });
                remaining_budget -= replacement_cost;
            }
        }

        recommendations
    }

    /// Compute summary statistics for the entire managed fleet.
    pub fn fleet_statistics(&self) -> FleetReport {
        if self.assets.is_empty() {
            return FleetReport {
                mean_age_years: 0.0,
                median_condition: 0.0,
                condition_distribution: vec![
                    ("0-1".to_string(), 0),
                    ("1-2".to_string(), 0),
                    ("2-3".to_string(), 0),
                    ("3-4".to_string(), 0),
                    ("4-5".to_string(), 0),
                ],
                total_replacement_value_usd: 0.0,
                high_risk_assets: 0,
            };
        }

        // Mean age.
        let mean_age_years = self
            .assets
            .iter()
            .map(|a| (NOW_UNIX - a.installation_date_unix) / SECS_PER_YEAR)
            .sum::<f64>()
            / self.assets.len() as f64;

        // Condition scores.
        let mut scores: Vec<f64> = self
            .assets
            .iter()
            .map(|a| self.current_condition_score(a))
            .collect();
        scores.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let n = scores.len();
        let median_condition = if n % 2 == 0 {
            (scores[n / 2 - 1] + scores[n / 2]) / 2.0
        } else {
            scores[n / 2]
        };

        // Condition distribution.
        let bands = ["0-1", "1-2", "2-3", "3-4", "4-5"];
        let mut dist = [0usize; 5];
        for &s in &scores {
            let idx = if s < 1.0 {
                0
            } else if s < 2.0 {
                1
            } else if s < 3.0 {
                2
            } else if s < 4.0 {
                3
            } else {
                4
            };
            dist[idx] += 1;
        }
        let condition_distribution = bands
            .iter()
            .zip(dist.iter())
            .map(|(b, c)| (b.to_string(), *c))
            .collect();

        // Total replacement value.
        let total_replacement_value_usd = self
            .assets
            .iter()
            .map(|a| a.asset_class.replacement_cost_usd())
            .sum();

        // High-risk count.
        let high_risk_assets = self
            .assets
            .iter()
            .filter(|a| self.calculate_risk_score(a) > 0.1)
            .count();

        FleetReport {
            mean_age_years,
            median_condition,
            condition_distribution,
            total_replacement_value_usd,
            high_risk_assets,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Asset Digitization — Physical grid asset digital twin representations
// ─────────────────────────────────────────────────────────────────────────────

/// Physical asset types in the power grid.
#[derive(Debug, Clone)]
pub enum AssetCategory {
    Transformer {
        kva_rating: f64,
        primary_kv: f64,
        secondary_kv: f64,
    },
    Breaker {
        current_rating_ka: f64,
        interrupting_capacity_ka: f64,
    },
    TransmissionLine {
        length_km: f64,
        voltage_kv: f64,
    },
    Cable {
        length_km: f64,
        voltage_kv: f64,
        insulation_type: String,
    },
    Generator {
        rated_mw: f64,
        technology: String,
    },
    BusBar {
        voltage_kv: f64,
        current_rating_ka: f64,
    },
    ProtectionRelayAsset {
        model: String,
        function_codes: Vec<String>,
    },
    CapacitorBank {
        rated_mvar: f64,
        voltage_kv: f64,
    },
    MeasurementTransformer {
        ratio: f64,
        accuracy_class: String,
    },
    Battery {
        energy_kwh: f64,
        power_kw: f64,
    },
    Inverter {
        power_kw: f64,
        voltage_level: String,
    },
    SolarPanel {
        peak_power_wp: f64,
        cell_technology: String,
    },
    WindTurbineAsset {
        rated_kw: f64,
        hub_height_m: f64,
    },
}

impl AssetCategory {
    /// Return a string label matching the variant name for filtering.
    pub fn category_name(&self) -> &'static str {
        match self {
            AssetCategory::Transformer { .. } => "Transformer",
            AssetCategory::Breaker { .. } => "Breaker",
            AssetCategory::TransmissionLine { .. } => "TransmissionLine",
            AssetCategory::Cable { .. } => "Cable",
            AssetCategory::Generator { .. } => "Generator",
            AssetCategory::BusBar { .. } => "BusBar",
            AssetCategory::ProtectionRelayAsset { .. } => "ProtectionRelay",
            AssetCategory::CapacitorBank { .. } => "CapacitorBank",
            AssetCategory::MeasurementTransformer { .. } => "MeasurementTransformer",
            AssetCategory::Battery { .. } => "Battery",
            AssetCategory::Inverter { .. } => "Inverter",
            AssetCategory::SolarPanel { .. } => "SolarPanel",
            AssetCategory::WindTurbineAsset { .. } => "WindTurbine",
        }
    }

    /// Rough replacement value in million EUR.
    pub fn replacement_value_million_eur(&self) -> f64 {
        match self {
            AssetCategory::Transformer { kva_rating, .. } => {
                // rated_mva * 0.1 M€/MVA
                (kva_rating / 1000.0) * 0.1
            }
            AssetCategory::Breaker { .. } => 0.1,
            AssetCategory::TransmissionLine { length_km, .. } => length_km * 0.5,
            AssetCategory::Cable { length_km, .. } => length_km * 0.3,
            AssetCategory::Generator { rated_mw, .. } => rated_mw * 0.8,
            AssetCategory::BusBar { .. } => 0.05,
            AssetCategory::ProtectionRelayAsset { .. } => 0.02,
            AssetCategory::CapacitorBank { rated_mvar, .. } => rated_mvar * 0.01,
            AssetCategory::MeasurementTransformer { .. } => 0.015,
            AssetCategory::Battery { energy_kwh, .. } => energy_kwh * 0.0005,
            AssetCategory::Inverter { power_kw, .. } => power_kw * 0.0003,
            AssetCategory::SolarPanel { peak_power_wp, .. } => peak_power_wp * 0.000_001,
            AssetCategory::WindTurbineAsset { rated_kw, .. } => rated_kw * 0.001_5,
        }
    }

    /// Typical design life in years for aging analysis.
    pub fn design_life_years(&self) -> f64 {
        match self {
            AssetCategory::Transformer { .. } => 40.0,
            AssetCategory::Breaker { .. } => 30.0,
            AssetCategory::TransmissionLine { .. } => 50.0,
            AssetCategory::Cable { .. } => 40.0,
            AssetCategory::Generator { .. } => 35.0,
            AssetCategory::BusBar { .. } => 50.0,
            AssetCategory::ProtectionRelayAsset { .. } => 25.0,
            AssetCategory::CapacitorBank { .. } => 20.0,
            AssetCategory::MeasurementTransformer { .. } => 35.0,
            AssetCategory::Battery { .. } => 15.0,
            AssetCategory::Inverter { .. } => 20.0,
            AssetCategory::SolarPanel { .. } => 25.0,
            AssetCategory::WindTurbineAsset { .. } => 25.0,
        }
    }

    /// Rated power in MW (used for weighting fleet health score).
    pub fn rated_power_mw(&self) -> f64 {
        match self {
            AssetCategory::Transformer { kva_rating, .. } => kva_rating / 1000.0,
            AssetCategory::Breaker {
                current_rating_ka, ..
            } => *current_rating_ka,
            AssetCategory::TransmissionLine { length_km, .. } => length_km * 0.01,
            AssetCategory::Cable { length_km, .. } => length_km * 0.01,
            AssetCategory::Generator { rated_mw, .. } => *rated_mw,
            AssetCategory::BusBar {
                current_rating_ka, ..
            } => *current_rating_ka,
            AssetCategory::ProtectionRelayAsset { .. } => 0.001,
            AssetCategory::CapacitorBank { rated_mvar, .. } => *rated_mvar,
            AssetCategory::MeasurementTransformer { .. } => 0.001,
            AssetCategory::Battery { power_kw, .. } => power_kw / 1000.0,
            AssetCategory::Inverter { power_kw, .. } => power_kw / 1000.0,
            AssetCategory::SolarPanel { peak_power_wp, .. } => peak_power_wp / 1_000_000.0,
            AssetCategory::WindTurbineAsset { rated_kw, .. } => rated_kw / 1000.0,
        }
    }
}

/// Asset health risk level.
#[derive(Clone, Debug, PartialEq)]
pub enum RiskLevel {
    Low,
    Medium,
    High,
    Critical,
}

/// Asset health condition snapshot.
#[derive(Clone, Debug)]
pub struct AssetCondition {
    /// Overall health index 0–100 (100 = new).
    pub overall_health_index: f64,
    pub mechanical_condition: f64,
    pub electrical_condition: f64,
    pub insulation_condition: f64,
    /// Applicable to cooling-equipped assets.
    pub cooling_condition: f64,
    /// ISO 8601 date string.
    pub last_inspection_date: String,
    /// ISO 8601 date string, or `"OVERDUE"` / `"SOON"`.
    pub next_maintenance_due: String,
    /// Active defect codes.
    pub defect_codes: Vec<String>,
    pub risk_level: RiskLevel,
}

impl Default for AssetCondition {
    fn default() -> Self {
        Self {
            overall_health_index: 100.0,
            mechanical_condition: 100.0,
            electrical_condition: 100.0,
            insulation_condition: 100.0,
            cooling_condition: 100.0,
            last_inspection_date: "2026-01-01".to_string(),
            next_maintenance_due: "2027-01-01".to_string(),
            defect_codes: Vec::new(),
            risk_level: RiskLevel::Low,
        }
    }
}

/// Geographic / installation location of a physical asset.
#[derive(Debug, Clone)]
pub struct AssetLocation {
    pub latitude: f64,
    pub longitude: f64,
    pub bay: String,
    pub panel: String,
    pub rack: Option<String>,
}

/// Nameplate (rated) data for an asset.
#[derive(Debug, Clone)]
pub struct AssetNameplate {
    pub rated_voltage_kv: f64,
    pub rated_current_ka: f64,
    pub rated_power_mva: f64,
    pub frequency_hz: f64,
    /// IEC temperature class (e.g. `"B"`, `"F"`, `"H"`).
    pub temperature_class: String,
    /// IP rating (e.g. `"IP54"`).
    pub protection_class: String,
    pub weight_kg: f64,
}

/// Live telemetry snapshot for an asset.
#[derive(Debug, Clone)]
pub struct AssetTelemetry {
    /// Unix timestamp [s].
    pub timestamp: f64,
    pub current_ka: f64,
    pub voltage_kv: f64,
    pub power_mw: f64,
    pub temperature_c: f64,
    /// For rotating machines [mm/s].
    pub vibration_mm_per_s: f64,
    /// HV equipment partial discharge [pC].
    pub partial_discharge_pc: f64,
    /// Transformer oil temperature [°C].
    pub oil_temperature_c: f64,
    /// H₂ dissolved in oil — DGA indicator [ppm].
    pub dissolved_gas_h2_ppm: f64,
    /// GIS equipment SF₆ pressure [bar].
    pub sf6_pressure_bar: f64,
}

impl Default for AssetTelemetry {
    fn default() -> Self {
        Self {
            timestamp: 0.0,
            current_ka: 0.0,
            voltage_kv: 0.0,
            power_mw: 0.0,
            temperature_c: 20.0,
            vibration_mm_per_s: 0.0,
            partial_discharge_pc: 0.0,
            oil_temperature_c: 20.0,
            dissolved_gas_h2_ppm: 0.0,
            sf6_pressure_bar: 0.0,
        }
    }
}

/// Category of maintenance work.
#[derive(Debug, Clone)]
pub enum AssetMaintenanceType {
    Routine,
    Preventive,
    Corrective,
    Emergency,
    Overhaul,
}

/// Record of a single maintenance intervention.
#[derive(Debug, Clone)]
pub struct AssetMaintenanceRecord {
    pub date: String,
    pub maintenance_type: AssetMaintenanceType,
    pub work_order: String,
    pub performed_by: String,
    pub duration_hours: f64,
    pub cost_eur: f64,
    pub findings: String,
    pub corrective_actions: Vec<String>,
}

/// Record of a single asset failure event.
#[derive(Debug, Clone)]
pub struct FailureRecord {
    pub date: String,
    pub failure_mode: String,
    pub severity: RiskLevel,
    pub outage_duration_hours: f64,
    pub affected_customers: u32,
    pub repair_cost_eur: f64,
    pub root_cause: String,
    pub corrective_action: String,
}

/// Full digital twin of a single physical power grid asset.
#[derive(Debug, Clone)]
pub struct AssetDigitalTwin {
    pub asset_id: String,
    pub asset_name: String,
    pub category: AssetCategory,
    pub substation_id: String,
    /// ISO 8601 date.
    pub commissioning_date: String,
    pub manufacturer: String,
    pub model_number: String,
    pub serial_number: String,
    pub location: AssetLocation,
    pub condition: AssetCondition,
    pub nameplate: AssetNameplate,
    pub telemetry: AssetTelemetry,
    pub maintenance_history: Vec<AssetMaintenanceRecord>,
    pub failure_history: Vec<FailureRecord>,
    /// 0–1: how well the digital twin matches the physical asset.
    pub digital_twin_accuracy: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// AssetRegistry
// ─────────────────────────────────────────────────────────────────────────────

/// Fleet-level asset registry (collection of asset digital twins).
pub struct AssetRegistry {
    pub assets: Vec<AssetDigitalTwin>,
    pub substation_id: String,
    pub utility_name: String,
    /// Unix timestamp of last synchronisation [s].
    pub last_sync: f64,
}

/// Summary of fleet aging.
#[derive(Debug, Clone)]
pub struct AgingReport {
    pub total_assets: usize,
    pub beyond_half_life: usize,
    pub beyond_design_life: usize,
    pub avg_age_years: f64,
    pub oldest_asset_id: String,
    pub oldest_asset_age_years: f64,
    /// Estimated capital required for replacements within 5 years [M€].
    pub capital_replacement_5yr_million_eur: f64,
}

/// Parse an ISO 8601 date string `"YYYY-MM-DD"` into a Unix timestamp [s].
///
/// Returns `None` on parse failure.
fn parse_iso_date(date: &str) -> Option<f64> {
    let parts: Vec<&str> = date.split('-').collect();
    if parts.len() != 3 {
        return None;
    }
    let year: i64 = parts[0].parse().ok()?;
    let month: i64 = parts[1].parse().ok()?;
    let day: i64 = parts[2].parse().ok()?;
    // Compute Julian Day Number (JDN) using the proleptic Gregorian calendar.
    let y = if month <= 2 { year - 1 } else { year };
    let m = if month <= 2 { month + 12 } else { month };
    let jdn: i64 = 365 * y + y / 4 - y / 100 + y / 400 + (153 * m + 8) / 5 + day - 678_882;
    // JDN for 1970-01-01 (Unix epoch): y=1969, m=13, day=1
    //   = 365*1969 + 1969/4 - 1969/100 + 1969/400 + (153*13+8)/5 + 1 - 678882
    //   = 718685 + 492 - 19 + 4 + 399 + 1 - 678882 = 40680
    const JDN_1970: i64 = 40_680;
    let days_since_epoch = jdn - JDN_1970;
    Some(days_since_epoch as f64 * 86_400.0)
}

/// Compute asset age in years from a commissioning date ISO string.
///
/// Uses `NOW_UNIX` as the reference instant.
fn asset_age_years(commissioning_date: &str) -> f64 {
    match parse_iso_date(commissioning_date) {
        Some(unix) => ((NOW_UNIX - unix) / SECS_PER_YEAR).max(0.0),
        None => 0.0,
    }
}

impl AssetRegistry {
    /// Create an empty registry.
    pub fn new(substation_id: String, utility_name: String) -> Self {
        Self {
            assets: Vec::new(),
            substation_id,
            utility_name,
            last_sync: 0.0,
        }
    }

    /// Add an asset digital twin to the registry.
    pub fn add_asset(&mut self, asset: AssetDigitalTwin) {
        self.assets.push(asset);
    }

    /// Look up an asset by ID.
    pub fn get_asset(&self, asset_id: &str) -> Option<&AssetDigitalTwin> {
        self.assets.iter().find(|a| a.asset_id == asset_id)
    }

    /// Look up an asset by ID (mutable).
    pub fn get_asset_mut(&mut self, asset_id: &str) -> Option<&mut AssetDigitalTwin> {
        self.assets.iter_mut().find(|a| a.asset_id == asset_id)
    }

    /// Return assets whose category name matches `category_name`.
    ///
    /// Valid names: `"Transformer"`, `"Breaker"`, `"TransmissionLine"`,
    /// `"Cable"`, `"Generator"`, `"BusBar"`, `"ProtectionRelay"`,
    /// `"CapacitorBank"`, `"MeasurementTransformer"`, `"Battery"`,
    /// `"Inverter"`, `"SolarPanel"`, `"WindTurbine"`.
    pub fn assets_by_category(&self, category_name: &str) -> Vec<&AssetDigitalTwin> {
        self.assets
            .iter()
            .filter(|a| a.category.category_name() == category_name)
            .collect()
    }

    /// Return assets with `RiskLevel::High` or `RiskLevel::Critical`.
    pub fn high_risk_assets(&self) -> Vec<&AssetDigitalTwin> {
        self.assets
            .iter()
            .filter(|a| {
                matches!(
                    a.condition.risk_level,
                    RiskLevel::High | RiskLevel::Critical
                )
            })
            .collect()
    }

    /// Return assets where `condition.next_maintenance_due` equals
    /// `"OVERDUE"` or `"SOON"`.
    pub fn assets_due_maintenance(&self, _within_days: f64) -> Vec<&AssetDigitalTwin> {
        self.assets
            .iter()
            .filter(|a| {
                let due = &a.condition.next_maintenance_due;
                due == "OVERDUE" || due == "SOON"
            })
            .collect()
    }

    /// Fleet health score: average health index weighted by rated power.
    ///
    /// Returns 100.0 if the registry is empty.
    pub fn fleet_health_score(&self) -> f64 {
        if self.assets.is_empty() {
            return 100.0;
        }
        let mut weighted_sum = 0.0;
        let mut total_weight = 0.0;
        for asset in &self.assets {
            let w = asset.category.rated_power_mw().max(1e-9);
            weighted_sum += asset.condition.overall_health_index * w;
            total_weight += w;
        }
        if total_weight > 0.0 {
            weighted_sum / total_weight
        } else {
            100.0
        }
    }

    /// Total replacement value [M€] estimated from asset categories.
    pub fn total_replacement_value_million_eur(&self) -> f64 {
        self.assets
            .iter()
            .map(|a| a.category.replacement_value_million_eur())
            .sum()
    }

    /// Analyse asset aging across the fleet.
    pub fn aging_analysis(&self) -> AgingReport {
        let total_assets = self.assets.len();
        if total_assets == 0 {
            return AgingReport {
                total_assets: 0,
                beyond_half_life: 0,
                beyond_design_life: 0,
                avg_age_years: 0.0,
                oldest_asset_id: String::new(),
                oldest_asset_age_years: 0.0,
                capital_replacement_5yr_million_eur: 0.0,
            };
        }

        let mut beyond_half_life = 0usize;
        let mut beyond_design_life = 0usize;
        let mut total_age = 0.0_f64;
        let mut oldest_age = 0.0_f64;
        let mut oldest_id = String::new();
        let mut capital_5yr = 0.0_f64;

        for asset in &self.assets {
            let age = asset_age_years(&asset.commissioning_date);
            let design_life = asset.category.design_life_years();
            total_age += age;
            if age > design_life / 2.0 {
                beyond_half_life += 1;
            }
            if age > design_life {
                beyond_design_life += 1;
                // Overdue for replacement — add to 5-year capital estimate.
                capital_5yr += asset.category.replacement_value_million_eur();
            } else if age > design_life * 0.8 {
                // Within final 20 % of design life — likely to need replacement in 5 yr.
                capital_5yr += asset.category.replacement_value_million_eur();
            }
            if age > oldest_age {
                oldest_age = age;
                oldest_id = asset.asset_id.clone();
            }
        }

        AgingReport {
            total_assets,
            beyond_half_life,
            beyond_design_life,
            avg_age_years: total_age / total_assets as f64,
            oldest_asset_id: oldest_id,
            oldest_asset_age_years: oldest_age,
            capital_replacement_5yr_million_eur: capital_5yr,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// TwinSynchronizer
// ─────────────────────────────────────────────────────────────────────────────

/// Synchronisation status for a single asset digital twin.
#[derive(Debug, Clone)]
pub struct TwinSyncStatus {
    pub asset_id: String,
    /// Unix timestamp of last telemetry update [s].
    pub last_sync_time: f64,
    /// Seconds elapsed since last update.
    pub sync_gap_s: f64,
    /// Telemetry completeness [0–100 %].
    pub data_quality_pct: f64,
    /// Names of sensors with no valid data.
    pub missing_sensors: Vec<String>,
    /// `true` when `sync_gap_s` exceeds the stale threshold.
    pub is_stale: bool,
    /// Digital twin accuracy of the asset.
    pub accuracy: f64,
}

/// Engine for monitoring and updating digital twin synchronisation.
pub struct TwinSynchronizer {
    /// Seconds after which an asset twin is considered stale.
    pub stale_threshold_s: f64,
    /// Minimum acceptable accuracy score (0–1).
    pub min_accuracy_threshold: f64,
}

impl TwinSynchronizer {
    /// Create a new synchroniser with the given thresholds.
    pub fn new(stale_threshold_s: f64, min_accuracy_threshold: f64) -> Self {
        Self {
            stale_threshold_s,
            min_accuracy_threshold,
        }
    }

    /// Check synchronisation status of every asset in the registry.
    pub fn check_sync_status(
        &self,
        registry: &AssetRegistry,
        current_time: f64,
    ) -> Vec<TwinSyncStatus> {
        registry
            .assets
            .iter()
            .map(|asset| {
                let last_sync_time = asset.telemetry.timestamp;
                let sync_gap_s = (current_time - last_sync_time).max(0.0);
                let is_stale = sync_gap_s > self.stale_threshold_s;

                // Identify missing sensors (zero / default telemetry values used as proxy).
                let mut missing = Vec::new();
                if asset.telemetry.current_ka == 0.0 {
                    missing.push("current_ka".to_string());
                }
                if asset.telemetry.voltage_kv == 0.0 {
                    missing.push("voltage_kv".to_string());
                }
                if asset.telemetry.power_mw == 0.0 {
                    missing.push("power_mw".to_string());
                }
                if asset.telemetry.temperature_c == 0.0 || asset.telemetry.temperature_c == 20.0 {
                    // 20.0 is the default — could be genuine or uninitialised.
                    // Only flag as missing when timestamp is zero (never updated).
                    if last_sync_time == 0.0 {
                        missing.push("temperature_c".to_string());
                    }
                }

                let total_sensors = 7usize; // current, voltage, power, temp, vibration, PD, H2
                let missing_count = missing.len().min(total_sensors);
                let data_quality_pct =
                    ((total_sensors - missing_count) as f64 / total_sensors as f64) * 100.0;

                TwinSyncStatus {
                    asset_id: asset.asset_id.clone(),
                    last_sync_time,
                    sync_gap_s,
                    data_quality_pct,
                    missing_sensors: missing,
                    is_stale,
                    accuracy: asset.digital_twin_accuracy,
                }
            })
            .collect()
    }

    /// Update the telemetry for the named asset in the registry.
    ///
    /// Returns `OxiGridError::InvalidParameter` if the asset is not found.
    pub fn update_telemetry(
        registry: &mut AssetRegistry,
        asset_id: &str,
        telemetry: AssetTelemetry,
    ) -> Result<(), crate::error::OxiGridError> {
        match registry.get_asset_mut(asset_id) {
            Some(asset) => {
                asset.telemetry = telemetry;
                Ok(())
            }
            None => Err(crate::error::OxiGridError::InvalidParameter(format!(
                "asset '{}' not found in registry",
                asset_id
            ))),
        }
    }

    /// Compute a digital twin accuracy score for an asset.
    ///
    /// Starts at 1.0 and deducts:
    /// - `−0.05` per missing telemetry field (zero-value proxy)
    /// - `−0.1` if no maintenance record in the last year (based on empty history)
    /// - `−0.02` per active defect code
    pub fn compute_accuracy(asset: &AssetDigitalTwin) -> f64 {
        let mut score = 1.0_f64;

        // Deduct for uninitialised / default telemetry.
        if asset.telemetry.timestamp == 0.0 {
            score -= 0.05;
        }
        if asset.telemetry.current_ka == 0.0 {
            score -= 0.05;
        }
        if asset.telemetry.voltage_kv == 0.0 {
            score -= 0.05;
        }
        if asset.telemetry.power_mw == 0.0 {
            score -= 0.05;
        }
        if asset.telemetry.vibration_mm_per_s == 0.0 {
            score -= 0.05;
        }
        if asset.telemetry.partial_discharge_pc == 0.0 {
            score -= 0.05;
        }
        if asset.telemetry.dissolved_gas_h2_ppm == 0.0 {
            score -= 0.05;
        }

        // Deduct if no maintenance record in the last year.
        let one_year_s = SECS_PER_YEAR;
        let recent_maintenance = asset.maintenance_history.iter().any(|m| {
            // Use work_order as a proxy — if date is not parseable, assume old.
            parse_iso_date(&m.date)
                .map(|t| (NOW_UNIX - t) < one_year_s)
                .unwrap_or(false)
        });
        if !recent_maintenance {
            score -= 0.1;
        }

        // Deduct per active defect code.
        let defect_penalty = 0.02 * asset.condition.defect_codes.len() as f64;
        score -= defect_penalty;

        score.clamp(0.0, 1.0)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// CbmAssessor — Condition-Based Maintenance Assessment
// ─────────────────────────────────────────────────────────────────────────────

/// Condition-based maintenance assessor using rule-based telemetry analysis.
pub struct CbmAssessor;

impl CbmAssessor {
    /// Assess risk level from telemetry (rule-based).
    ///
    /// Rules:
    /// - Vibration > 7.1 mm/s (ISO 10816 Zone D) → `Critical`
    /// - H₂ in oil > 150 ppm → `Critical`
    /// - H₂ in oil > 50 ppm → `High`
    /// - PD > 1000 pC → `High`
    /// - Temperature > 120 °C → `High`
    /// - Temperature > 100 °C → `Medium`
    /// - Otherwise → `Low`
    pub fn assess_from_telemetry(telemetry: &AssetTelemetry) -> RiskLevel {
        if telemetry.vibration_mm_per_s > 7.1 {
            return RiskLevel::Critical;
        }
        if telemetry.dissolved_gas_h2_ppm > 150.0 {
            return RiskLevel::Critical;
        }
        if telemetry.dissolved_gas_h2_ppm > 50.0 {
            return RiskLevel::High;
        }
        if telemetry.partial_discharge_pc > 1000.0 {
            return RiskLevel::High;
        }
        if telemetry.temperature_c > 120.0 {
            return RiskLevel::High;
        }
        if telemetry.temperature_c > 100.0 {
            return RiskLevel::Medium;
        }
        RiskLevel::Low
    }

    /// Update the asset's condition risk level based on its current telemetry.
    pub fn update_condition(asset: &mut AssetDigitalTwin) {
        let risk = Self::assess_from_telemetry(&asset.telemetry);
        asset.condition.risk_level = risk;
    }

    /// Estimate remaining useful life [years].
    ///
    /// Uses linear degradation from health index:
    /// `RUL = (health - threshold_health) / (100 - threshold_health) * typical_life_years`
    ///
    /// Returns 0.0 when health is at or below threshold.
    pub fn estimate_rul(
        asset: &AssetDigitalTwin,
        threshold_health: f64,
        typical_life_years: f64,
    ) -> f64 {
        let health = asset.condition.overall_health_index;
        let span = 100.0 - threshold_health;
        if span <= 0.0 || health <= threshold_health {
            return 0.0;
        }
        let rul = ((health - threshold_health) / span) * typical_life_years;
        rul.max(0.0)
    }

    /// Recommend a maintenance interval in years using a simplified CBM criterion.
    ///
    /// Based on current health index:
    /// - Health ≥ 80 → 5 years
    /// - Health ≥ 60 → 3 years
    /// - Health ≥ 40 → 1 year
    /// - Health < 40  → 0.5 years (6 months)
    pub fn recommend_maintenance_interval(asset: &AssetDigitalTwin) -> f64 {
        let health = asset.condition.overall_health_index;
        if health >= 80.0 {
            5.0
        } else if health >= 60.0 {
            3.0
        } else if health >= 40.0 {
            1.0
        } else {
            0.5
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Fixed "current" time used in all tests so results are deterministic.
    /// Matches NOW_UNIX defined at module level (2026-03-09).
    const T_NOW: f64 = NOW_UNIX;

    fn make_transformer(age_years: f64, criticality: f64) -> DigitalAssetRecord {
        DigitalAssetRecord {
            asset_id: "TRF-001".to_string(),
            asset_class: AssetClass::PowerTransformer {
                mva_rating: 100.0,
                voltage_kv: 220.0,
                vector_group: "YNd11".to_string(),
            },
            installation_date_unix: T_NOW - age_years * SECS_PER_YEAR,
            manufacturer: "ABB".to_string(),
            model: "TRAFO-220".to_string(),
            serial_number: "SN-000001".to_string(),
            substation_id: "SS-01".to_string(),
            bus_id: 0,
            inspection_history: Vec::new(),
            maintenance_history: Vec::new(),
            operating_hours: age_years * 8760.0,
            load_factor_avg: 0.7,
            criticality_score: criticality,
        }
    }

    fn make_circuit_breaker(age_years: f64, criticality: f64) -> DigitalAssetRecord {
        DigitalAssetRecord {
            asset_id: "CB-001".to_string(),
            asset_class: AssetClass::CircuitBreaker {
                rated_kv: 145.0,
                rated_ka: 2.5,
                interrupting_capacity_ka: 40.0,
            },
            installation_date_unix: T_NOW - age_years * SECS_PER_YEAR,
            manufacturer: "Siemens".to_string(),
            model: "3AP1FI".to_string(),
            serial_number: "SN-CB-001".to_string(),
            substation_id: "SS-01".to_string(),
            bus_id: 1,
            inspection_history: Vec::new(),
            maintenance_history: Vec::new(),
            operating_hours: age_years * 8760.0,
            load_factor_avg: 0.5,
            criticality_score: criticality,
        }
    }

    fn add_inspection(
        asset: &mut DigitalAssetRecord,
        days_ago: f64,
        score: f64,
        severity: DefectSeverity,
    ) {
        asset.inspection_history.push(InspectionRecord {
            inspection_id: format!("INS-{}", asset.inspection_history.len()),
            date_unix: T_NOW - days_ago * 86400.0,
            inspection_type: InspectionType::Visual,
            inspector_id: "ENG-01".to_string(),
            findings: vec!["Routine visual check".to_string()],
            condition_score: score,
            recommended_actions: Vec::new(),
            defect_severity: severity,
        });
    }

    /// Condition score from a recent (30-day-old) inspection with score 4.0
    /// should remain very close to 4.0 after degradation.
    #[test]
    fn test_condition_score_recent_inspection() {
        let scheduler = MaintenanceScheduler::new(Vec::new(), 100_000.0, 5);
        let mut asset = make_transformer(5.0, 0.8);
        add_inspection(&mut asset, 30.0, 4.0, DefectSeverity::None);

        let score = scheduler.current_condition_score(&asset);
        // 30 days ≈ 0.082 years → degradation ≈ 0.008; score ≈ 3.99
        assert!((score - 4.0).abs() < 0.1, "Expected ~4.0, got {score:.4}");
    }

    /// A 40-year-old transformer should have a high failure probability (>0.5).
    #[test]
    fn test_failure_probability_old_transformer() {
        let scheduler = MaintenanceScheduler::new(Vec::new(), 100_000.0, 5);
        let asset = make_transformer(40.0, 0.9);
        let p = scheduler.predict_failure_probability(&asset, 1.0);
        assert!(
            p > 0.5,
            "Expected P > 0.5 for 40-year transformer, got {p:.4}"
        );
    }

    /// High failure probability × high criticality should yield a positive risk score.
    #[test]
    fn test_risk_score_high_probability_and_criticality() {
        let scheduler = MaintenanceScheduler::new(Vec::new(), 100_000.0, 5);
        let asset = make_transformer(38.0, 1.0);
        let risk = scheduler.calculate_risk_score(&asset);
        assert!(risk > 0.0, "Risk score should be positive, got {risk:.6}");
    }

    /// DGA with high C2H2 and C2H2/C2H4 ratio should classify as electrical arcing.
    #[test]
    fn test_dga_arcing_diagnosis() {
        let scheduler = MaintenanceScheduler::new(Vec::new(), 100_000.0, 5);
        let asset = make_transformer(10.0, 0.8);

        let mut gas: HashMap<String, f64> = HashMap::new();
        gas.insert("H2".to_string(), 100.0);
        gas.insert("CH4".to_string(), 80.0);
        gas.insert("C2H2".to_string(), 50.0); // high acetylene
        gas.insert("C2H4".to_string(), 30.0); // C2H2/C2H4 = 1.67 > 1.0
        gas.insert("C2H6".to_string(), 10.0);

        let result = scheduler
            .dga_analysis(&gas, &asset)
            .expect("DGA should succeed for transformer");
        assert_eq!(
            result.fault_type, "Electrical Discharge (Arcing)",
            "Expected arcing, got: {}",
            result.fault_type
        );
        assert_eq!(result.severity, DefectSeverity::Critical);
    }

    /// A brand-new asset (age ≈ 0, condition = 5) should yield AHI close to 100.
    #[test]
    fn test_ahi_new_asset() {
        let scheduler = MaintenanceScheduler::new(Vec::new(), 100_000.0, 5);
        // Install "now" → age ≈ 0
        let mut asset = make_transformer(0.01, 0.5);
        add_inspection(&mut asset, 1.0, 5.0, DefectSeverity::None);
        // Add preventive maintenance to maximise maint_eff.
        asset.maintenance_history.push(MaintenanceRecord {
            date_unix: T_NOW - 30.0 * 86400.0,
            maintenance_type: MaintenanceType::Preventive,
            cost_usd: 5_000.0,
            downtime_h: 4.0,
            performed_by: "OEM".to_string(),
            parts_replaced: Vec::new(),
        });

        let report = scheduler.asset_health_index(&asset);
        assert!(
            report.health_index > 90.0,
            "Expected AHI > 90 for new asset, got {:.2}",
            report.health_index
        );
    }

    /// With a zero budget the plan should schedule no actions.
    #[test]
    fn test_maintenance_plan_zero_budget() {
        let mut asset = make_transformer(30.0, 0.9);
        add_inspection(&mut asset, 365.0, 2.5, DefectSeverity::Moderate);
        let scheduler = MaintenanceScheduler::new(vec![asset], 0.0, 5);

        let plan = scheduler.generate_maintenance_plan(0.0);
        assert!(
            plan.scheduled_actions.is_empty(),
            "Zero budget should schedule no actions, got {}",
            plan.scheduled_actions.len()
        );
    }

    /// An asset with a condition score below 2.0 should appear in the
    /// replacement prioritisation list.
    #[test]
    fn test_replacement_priority_poor_condition() {
        let mut asset = make_circuit_breaker(28.0, 0.8);
        // Force condition below 2.0: inspection 15 years ago with score 3.5
        // → 3.5 - 0.1×15 = 2.0; add another low-score recent inspection.
        add_inspection(&mut asset, 10.0, 1.5, DefectSeverity::Significant);

        let scheduler = MaintenanceScheduler::new(vec![asset], 200_000.0, 5);

        let recs = scheduler.replacement_prioritization(1_000_000.0);
        assert!(
            !recs.is_empty(),
            "Expected at least one replacement recommendation for poor-condition asset"
        );
        assert_eq!(recs[0].asset_id, "CB-001");
    }

    /// Fleet mean age should equal the arithmetic mean of individual ages.
    #[test]
    fn test_fleet_mean_age_calculation() {
        let a1 = make_transformer(10.0, 0.7);
        let a2 = make_circuit_breaker(20.0, 0.5);
        let scheduler = MaintenanceScheduler::new(vec![a1, a2], 50_000.0, 3);

        let report = scheduler.fleet_statistics();
        let expected_mean = 15.0; // (10 + 20) / 2
        assert!(
            (report.mean_age_years - expected_mean).abs() < 0.5,
            "Expected mean age ~15 yr, got {:.4}",
            report.mean_age_years
        );
    }

    /// DGA on a non-transformer asset should return an error.
    #[test]
    fn test_dga_non_transformer_returns_error() {
        let scheduler = MaintenanceScheduler::new(Vec::new(), 50_000.0, 3);
        let asset = make_circuit_breaker(5.0, 0.5);

        let gas: HashMap<String, f64> = HashMap::new();
        let result = scheduler.dga_analysis(&gas, &asset);
        assert!(
            result.is_err(),
            "DGA on a circuit breaker should return Err"
        );
    }

    /// Budget exactly covering one maintenance action should schedule exactly one.
    #[test]
    fn test_maintenance_plan_budget_covers_one() {
        let mut xfmr = make_transformer(30.0, 0.9);
        add_inspection(&mut xfmr, 180.0, 2.0, DefectSeverity::Moderate);
        let mut cb = make_circuit_breaker(25.0, 0.8);
        add_inspection(&mut cb, 180.0, 2.5, DefectSeverity::Minor);

        // Transformer maintenance cost = 20_000; CB = 5_000.
        // Budget for 1 year = 22_000 → can afford xfmr (20k) but NOT both.
        let scheduler = MaintenanceScheduler::new(
            vec![xfmr, cb],
            22_000.0, // annual budget
            1,        // 1-year horizon → total budget = 22_000
        );

        let plan = scheduler.generate_maintenance_plan(0.0);
        assert_eq!(
            plan.scheduled_actions.len(),
            1,
            "Budget of $22k over 1yr should schedule exactly one transformer maintenance"
        );
    }

    // ─── AssetDigitalTwin / AssetRegistry tests ────────────────────────────

    fn make_twin(
        id: &str,
        category: AssetCategory,
        health: f64,
        risk: RiskLevel,
    ) -> AssetDigitalTwin {
        AssetDigitalTwin {
            asset_id: id.to_string(),
            asset_name: format!("Asset {}", id),
            category,
            substation_id: "SS-01".to_string(),
            commissioning_date: "2016-01-01".to_string(), // ~10 years old
            manufacturer: "TestCo".to_string(),
            model_number: "MDL-100".to_string(),
            serial_number: "SN-TEST-001".to_string(),
            location: AssetLocation {
                latitude: 59.44,
                longitude: 24.75,
                bay: "A1".to_string(),
                panel: "P1".to_string(),
                rack: None,
            },
            condition: AssetCondition {
                overall_health_index: health,
                mechanical_condition: health,
                electrical_condition: health,
                insulation_condition: health,
                cooling_condition: health,
                last_inspection_date: "2025-06-01".to_string(),
                next_maintenance_due: "2027-06-01".to_string(),
                defect_codes: Vec::new(),
                risk_level: risk,
            },
            nameplate: AssetNameplate {
                rated_voltage_kv: 110.0,
                rated_current_ka: 1.0,
                rated_power_mva: 50.0,
                frequency_hz: 50.0,
                temperature_class: "F".to_string(),
                protection_class: "IP54".to_string(),
                weight_kg: 5000.0,
            },
            telemetry: AssetTelemetry::default(),
            maintenance_history: Vec::new(),
            failure_history: Vec::new(),
            digital_twin_accuracy: 0.9,
        }
    }

    #[test]
    fn test_asset_digital_twin_creation() {
        let twin = make_twin(
            "T-001",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            90.0,
            RiskLevel::Low,
        );
        assert_eq!(twin.asset_id, "T-001");
        assert_eq!(twin.condition.overall_health_index, 90.0);
        assert_eq!(twin.digital_twin_accuracy, 0.9);
    }

    #[test]
    fn test_asset_registry_add_get() {
        let mut registry = AssetRegistry::new("SS-01".to_string(), "UtilityX".to_string());
        let twin = make_twin(
            "T-001",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            85.0,
            RiskLevel::Low,
        );
        registry.add_asset(twin);
        assert!(registry.get_asset("T-001").is_some());
        assert!(registry.get_asset("T-999").is_none());
    }

    #[test]
    fn test_assets_by_category_transformer() {
        let mut registry = AssetRegistry::new("SS-01".to_string(), "UtilityX".to_string());
        registry.add_asset(make_twin(
            "T-001",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            80.0,
            RiskLevel::Low,
        ));
        registry.add_asset(make_twin(
            "B-001",
            AssetCategory::Breaker {
                current_rating_ka: 2.5,
                interrupting_capacity_ka: 40.0,
            },
            75.0,
            RiskLevel::Low,
        ));
        let transformers = registry.assets_by_category("Transformer");
        assert_eq!(transformers.len(), 1);
        assert_eq!(transformers[0].asset_id, "T-001");
    }

    #[test]
    fn test_high_risk_assets_filter() {
        let mut registry = AssetRegistry::new("SS-01".to_string(), "UtilityX".to_string());
        registry.add_asset(make_twin(
            "T-001",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            90.0,
            RiskLevel::Low,
        ));
        registry.add_asset(make_twin(
            "T-002",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            40.0,
            RiskLevel::High,
        ));
        registry.add_asset(make_twin(
            "T-003",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            20.0,
            RiskLevel::Critical,
        ));
        let high_risk = registry.high_risk_assets();
        assert_eq!(high_risk.len(), 2);
    }

    #[test]
    fn test_fleet_health_score() {
        let mut registry = AssetRegistry::new("SS-01".to_string(), "UtilityX".to_string());
        // Generator 100 MW at health 80; Transformer 50 MVA at health 60.
        registry.add_asset(make_twin(
            "G-001",
            AssetCategory::Generator {
                rated_mw: 100.0,
                technology: "CCGT".to_string(),
            },
            80.0,
            RiskLevel::Low,
        ));
        registry.add_asset(make_twin(
            "T-001",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            60.0,
            RiskLevel::Medium,
        ));
        let score = registry.fleet_health_score();
        // Weighted: (80*100 + 60*50) / (100+50) = (8000+3000)/150 = 73.33
        assert!(
            (score - 73.33).abs() < 1.0,
            "Fleet health score ~73.3, got {:.2}",
            score
        );
    }

    #[test]
    fn test_fleet_health_score_perfect() {
        let mut registry = AssetRegistry::new("SS-01".to_string(), "UtilityX".to_string());
        registry.add_asset(make_twin(
            "T-001",
            AssetCategory::Transformer {
                kva_rating: 100_000.0,
                primary_kv: 220.0,
                secondary_kv: 110.0,
            },
            100.0,
            RiskLevel::Low,
        ));
        let score = registry.fleet_health_score();
        assert!(
            (score - 100.0).abs() < 1e-6,
            "Score should be 100.0, got {:.6}",
            score
        );
    }

    #[test]
    fn test_total_replacement_value() {
        let mut registry = AssetRegistry::new("SS-01".to_string(), "UtilityX".to_string());
        // 100 MVA transformer: 100 * 0.1 = 10 M€
        registry.add_asset(make_twin(
            "T-001",
            AssetCategory::Transformer {
                kva_rating: 100_000.0,
                primary_kv: 220.0,
                secondary_kv: 110.0,
            },
            85.0,
            RiskLevel::Low,
        ));
        // Breaker: 0.1 M€
        registry.add_asset(make_twin(
            "B-001",
            AssetCategory::Breaker {
                current_rating_ka: 2.5,
                interrupting_capacity_ka: 40.0,
            },
            85.0,
            RiskLevel::Low,
        ));
        let total = registry.total_replacement_value_million_eur();
        assert!(
            (total - 10.1).abs() < 0.01,
            "Expected 10.1 M€, got {:.4}",
            total
        );
    }

    #[test]
    fn test_aging_report_young_fleet() {
        let mut registry = AssetRegistry::new("SS-01".to_string(), "UtilityX".to_string());
        // Commissioning date 2 years ago (2024-01-01): age ~2 yr, design life 40 yr → not beyond half-life.
        let mut twin = make_twin(
            "T-001",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            95.0,
            RiskLevel::Low,
        );
        twin.commissioning_date = "2024-01-01".to_string();
        registry.add_asset(twin);
        let report = registry.aging_analysis();
        assert_eq!(report.total_assets, 1);
        assert_eq!(
            report.beyond_half_life, 0,
            "Young asset should not be beyond half-life"
        );
        assert_eq!(report.beyond_design_life, 0);
    }

    #[test]
    fn test_aging_report_old_fleet() {
        let mut registry = AssetRegistry::new("SS-01".to_string(), "UtilityX".to_string());
        // Commissioning date 45 years ago (1981-01-01): age ~45 yr > 40 yr design life.
        let mut twin = make_twin(
            "T-OLD",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            30.0,
            RiskLevel::High,
        );
        twin.commissioning_date = "1981-01-01".to_string();
        registry.add_asset(twin);
        let report = registry.aging_analysis();
        assert_eq!(
            report.beyond_design_life, 1,
            "45-year transformer should be beyond design life"
        );
        assert!(report.beyond_half_life >= 1);
        assert!(report.capital_replacement_5yr_million_eur > 0.0);
    }

    #[test]
    fn test_twin_synchronizer_creation() {
        let sync = TwinSynchronizer::new(300.0, 0.8);
        assert_eq!(sync.stale_threshold_s, 300.0);
        assert_eq!(sync.min_accuracy_threshold, 0.8);
    }

    #[test]
    fn test_sync_status_fresh() {
        let mut registry = AssetRegistry::new("SS-01".to_string(), "UtilityX".to_string());
        let mut twin = make_twin(
            "T-001",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            90.0,
            RiskLevel::Low,
        );
        twin.telemetry.timestamp = 1_000_000.0;
        registry.add_asset(twin);
        let sync = TwinSynchronizer::new(300.0, 0.8);
        // current_time = timestamp + 10 s → gap = 10 < 300 → not stale
        let statuses = sync.check_sync_status(&registry, 1_000_010.0);
        assert_eq!(statuses.len(), 1);
        assert!(!statuses[0].is_stale, "10 s gap should not be stale");
        assert!((statuses[0].sync_gap_s - 10.0).abs() < 1.0);
    }

    #[test]
    fn test_sync_status_stale() {
        let mut registry = AssetRegistry::new("SS-01".to_string(), "UtilityX".to_string());
        let mut twin = make_twin(
            "T-001",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            90.0,
            RiskLevel::Low,
        );
        twin.telemetry.timestamp = 1_000_000.0;
        registry.add_asset(twin);
        let sync = TwinSynchronizer::new(300.0, 0.8);
        // current_time = timestamp + 600 s → gap = 600 > 300 → stale
        let statuses = sync.check_sync_status(&registry, 1_000_600.0);
        assert!(statuses[0].is_stale, "600 s gap should be stale");
    }

    #[test]
    fn test_update_telemetry_success() {
        let mut registry = AssetRegistry::new("SS-01".to_string(), "UtilityX".to_string());
        registry.add_asset(make_twin(
            "T-001",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            85.0,
            RiskLevel::Low,
        ));
        let new_tel = AssetTelemetry {
            timestamp: 9_999_999.0,
            current_ka: 1.2,
            voltage_kv: 110.0,
            power_mw: 45.0,
            temperature_c: 75.0,
            vibration_mm_per_s: 1.5,
            partial_discharge_pc: 50.0,
            oil_temperature_c: 65.0,
            dissolved_gas_h2_ppm: 10.0,
            sf6_pressure_bar: 0.0,
        };
        let result = TwinSynchronizer::update_telemetry(&mut registry, "T-001", new_tel);
        assert!(result.is_ok());
        let asset = registry.get_asset("T-001").expect("asset should exist");
        assert_eq!(asset.telemetry.timestamp, 9_999_999.0);
        assert_eq!(asset.telemetry.current_ka, 1.2);
    }

    #[test]
    fn test_update_telemetry_not_found() {
        let mut registry = AssetRegistry::new("SS-01".to_string(), "UtilityX".to_string());
        let tel = AssetTelemetry::default();
        let result = TwinSynchronizer::update_telemetry(&mut registry, "NONEXISTENT", tel);
        assert!(result.is_err(), "Should return error for missing asset");
    }

    #[test]
    fn test_compute_accuracy_full_data() {
        // Asset with full telemetry and a recent maintenance record → high accuracy.
        let mut twin = make_twin(
            "T-001",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            90.0,
            RiskLevel::Low,
        );
        twin.telemetry = AssetTelemetry {
            timestamp: NOW_UNIX - 100.0,
            current_ka: 1.0,
            voltage_kv: 110.0,
            power_mw: 50.0,
            temperature_c: 75.0,
            vibration_mm_per_s: 1.0,
            partial_discharge_pc: 100.0,
            oil_temperature_c: 65.0,
            dissolved_gas_h2_ppm: 5.0,
            sf6_pressure_bar: 6.0,
        };
        twin.maintenance_history.push(AssetMaintenanceRecord {
            date: "2026-01-15".to_string(),
            maintenance_type: AssetMaintenanceType::Preventive,
            work_order: "WO-001".to_string(),
            performed_by: "TechTeam".to_string(),
            duration_hours: 4.0,
            cost_eur: 5000.0,
            findings: "All OK".to_string(),
            corrective_actions: Vec::new(),
        });
        let accuracy = TwinSynchronizer::compute_accuracy(&twin);
        // All 7 telemetry fields non-zero + recent maintenance → should be >= 0.9
        assert!(
            accuracy >= 0.9,
            "Full-data asset accuracy should be ≥ 0.9, got {:.4}",
            accuracy
        );
    }

    #[test]
    fn test_cbm_assess_normal() {
        let tel = AssetTelemetry {
            timestamp: 1_000_000.0,
            current_ka: 0.8,
            voltage_kv: 110.0,
            power_mw: 40.0,
            temperature_c: 70.0,
            vibration_mm_per_s: 1.0,
            partial_discharge_pc: 50.0,
            oil_temperature_c: 60.0,
            dissolved_gas_h2_ppm: 5.0,
            sf6_pressure_bar: 6.0,
        };
        assert_eq!(CbmAssessor::assess_from_telemetry(&tel), RiskLevel::Low);
    }

    #[test]
    fn test_cbm_assess_high_temperature() {
        let tel = AssetTelemetry {
            temperature_c: 125.0,
            ..AssetTelemetry::default()
        };
        assert_eq!(CbmAssessor::assess_from_telemetry(&tel), RiskLevel::High);
    }

    #[test]
    fn test_cbm_assess_high_pd() {
        let tel = AssetTelemetry {
            temperature_c: 60.0,
            partial_discharge_pc: 1500.0,
            ..AssetTelemetry::default()
        };
        assert_eq!(CbmAssessor::assess_from_telemetry(&tel), RiskLevel::High);
    }

    #[test]
    fn test_estimate_rul_new_asset() {
        let twin = make_twin(
            "T-001",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            95.0,
            RiskLevel::Low,
        );
        // health=95, threshold=20, typical=40 → RUL = (95-20)/(100-20)*40 = 75/80*40 = 37.5
        let rul = CbmAssessor::estimate_rul(&twin, 20.0, 40.0);
        assert!(
            (rul - 37.5).abs() < 0.1,
            "Expected RUL ~37.5, got {:.4}",
            rul
        );
    }

    #[test]
    fn test_estimate_rul_aged_asset() {
        let twin = make_twin(
            "T-OLD",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            15.0,
            RiskLevel::Critical,
        );
        // health=15 < threshold=20 → RUL = 0
        let rul = CbmAssessor::estimate_rul(&twin, 20.0, 40.0);
        assert_eq!(rul, 0.0, "Health below threshold should give RUL=0");
    }

    #[test]
    fn test_recommend_maintenance_interval() {
        let mut twin_new = make_twin(
            "T-001",
            AssetCategory::Transformer {
                kva_rating: 50_000.0,
                primary_kv: 110.0,
                secondary_kv: 20.0,
            },
            90.0,
            RiskLevel::Low,
        );
        twin_new.condition.overall_health_index = 90.0;
        assert_eq!(
            CbmAssessor::recommend_maintenance_interval(&twin_new),
            5.0,
            "Health ≥80 → 5yr"
        );

        let mut twin_mid = twin_new.clone();
        twin_mid.condition.overall_health_index = 65.0;
        assert_eq!(
            CbmAssessor::recommend_maintenance_interval(&twin_mid),
            3.0,
            "Health 60-79 → 3yr"
        );

        let mut twin_low = twin_new.clone();
        twin_low.condition.overall_health_index = 45.0;
        assert_eq!(
            CbmAssessor::recommend_maintenance_interval(&twin_low),
            1.0,
            "Health 40-59 → 1yr"
        );

        let mut twin_crit = twin_new.clone();
        twin_crit.condition.overall_health_index = 30.0;
        assert_eq!(
            CbmAssessor::recommend_maintenance_interval(&twin_crit),
            0.5,
            "Health <40 → 0.5yr"
        );
    }
}
