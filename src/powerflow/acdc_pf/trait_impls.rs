//! # AcDcPfConfig - Trait Implementations
//!
//! This module contains trait implementations for `AcDcPfConfig`.
//!
//! ## Implemented Traits
//!
//! - `Default`
//! - `Default`
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

use super::types::AcDcPfConfig;
use super::types_3::AcDcSequentialConfig;

impl Default for AcDcPfConfig {
    fn default() -> Self {
        Self {
            max_iterations: 50,
            tolerance_pu: 1e-6,
            base_mva: 100.0,
            base_kv_ac: 110.0,
            base_kv_dc: 320.0,
        }
    }
}

impl Default for AcDcSequentialConfig {
    fn default() -> Self {
        Self {
            n_ac_buses: 0,
            n_dc_buses: 0,
            tolerance: 1e-6,
            max_iterations: 50,
            base_mva: 100.0,
        }
    }
}
