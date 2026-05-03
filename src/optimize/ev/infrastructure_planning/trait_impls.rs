//! # ChargerCost - Trait Implementations
//!
//! This module contains trait implementations for `ChargerCost`.
//!
//! ## Implemented Traits
//!
//! - `Default`
//! - `Default`
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

use super::types::{ChargerCost, InfrastructurePlanConfig};

impl Default for ChargerCost {
    fn default() -> Self {
        Self {
            level2_installed_usd: 5_000.0,
            dcfc_installed_usd: 50_000.0,
            annual_opex_pct: 0.05,
            electricity_rate_per_kwh: 0.12,
        }
    }
}

impl Default for InfrastructurePlanConfig {
    fn default() -> Self {
        Self {
            planning_horizon_years: 10,
            discount_rate: 0.07,
            charger_cost: ChargerCost::default(),
            reliability_target: 0.95,
            grid_upgrade_cost_per_kw: 500.0,
        }
    }
}
