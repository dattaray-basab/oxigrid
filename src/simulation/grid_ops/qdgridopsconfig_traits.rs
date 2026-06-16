//! # `QdGridOpsConfig` - Trait Implementations
//!
//! This module contains trait implementations for `QdGridOpsConfig`.
//!
//! ## Implemented Traits
//!
//! - `Default`
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

use super::types::QdGridOpsConfig;

impl Default for QdGridOpsConfig {
    fn default() -> Self {
        Self {
            n_buses: 10,
            base_mva: 100.0,
            nominal_frequency_hz: 50.0,
            frequency_deadband_hz: 0.02,
            ufls_threshold_hz: 47.5,
            ufls_shed_pct: vec![0.10, 0.15, 0.20],
            ovf_threshold_hz: 51.5,
            voltage_min_pu: 0.95,
            voltage_max_pu: 1.05,
            max_branch_loading_pct: 90.0,
        }
    }
}
