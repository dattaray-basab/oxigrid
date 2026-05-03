//! Auto-generated module
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

use super::functions::asset_age_years;
use super::types::{
    AgingReport, AssetClass, AssetDigitalTwin, AssetMaintenanceType, InspectionRecord,
    MaintenanceRecord, RiskLevel, ScheduledMaintenance,
};

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
            AssetCategory::Transformer { kva_rating, .. } => (kva_rating / 1000.0) * 0.1,
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
/// Synchronisation status for a single asset digital twin.
#[derive(Debug, Clone)]
pub struct TwinSyncStatus {
    pub asset_id: String,
    /// Unix timestamp of last telemetry update `s`.
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
/// Output of `MaintenanceScheduler::generate_maintenance_plan`.
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
/// Fleet-level asset registry (collection of asset digital twins).
pub struct AssetRegistry {
    pub assets: Vec<AssetDigitalTwin>,
    pub substation_id: String,
    pub utility_name: String,
    /// Unix timestamp of last synchronisation `s`.
    pub last_sync: f64,
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
                capital_5yr += asset.category.replacement_value_million_eur();
            } else if age > design_life * 0.8 {
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
