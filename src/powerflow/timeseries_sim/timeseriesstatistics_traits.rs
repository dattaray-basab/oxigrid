//! # `TimeSeriesStatistics` - Trait Implementations
//!
//! This module contains trait implementations for `TimeSeriesStatistics`.
//!
//! ## Implemented Traits
//!
//! - `Default`
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

use super::types::TimeSeriesStatistics;

impl Default for TimeSeriesStatistics {
    fn default() -> Self {
        Self {
            n_timesteps: 0,
            n_converged: 0,
            convergence_rate: 0.0,
            max_voltage_pu: 1.0,
            min_voltage_pu: 1.0,
            avg_voltage_pu: 1.0,
            peak_load_mw: 0.0,
            avg_load_mw: 0.0,
            load_factor: 0.0,
            total_energy_twh: 0.0,
            renewable_fraction_pct: 0.0,
            total_curtailment_mwh: 0.0,
            total_losses_mwh: 0.0,
            max_branch_loading_pct: 0.0,
            avg_branch_loading_pct: 0.0,
            n_overload_hours: 0,
            n_voltage_violation_hours: 0,
            hosting_capacity_estimate_mw: 0.0,
        }
    }
}
