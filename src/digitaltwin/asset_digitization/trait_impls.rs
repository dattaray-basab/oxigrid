//! # AssetCondition - Trait Implementations
//!
//! This module contains trait implementations for `AssetCondition`.
//!
//! ## Implemented Traits
//!
//! - `Default`
//! - `Default`
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

use super::types::{AssetTelemetry, RiskLevel};
use super::types_3::AssetCondition;

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
