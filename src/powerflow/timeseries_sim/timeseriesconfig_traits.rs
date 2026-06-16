//! # `TimeSeriesConfig` - Trait Implementations
//!
//! This module contains trait implementations for `TimeSeriesConfig`.
//!
//! ## Implemented Traits
//!
//! - `Default`
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

use super::types::{StorageStrategy, TimeResolution, TimeSeriesConfig};

impl Default for TimeSeriesConfig {
    fn default() -> Self {
        Self {
            n_timesteps: 8760,
            resolution: TimeResolution::Hourly,
            voltage_lower_pu: 0.95,
            voltage_upper_pu: 1.05,
            max_pf_iterations: 20,
            pf_tolerance: 1e-4,
            enable_curtailment: true,
            storage_dispatch_strategy: StorageStrategy::ScheduledDispatch,
        }
    }
}
