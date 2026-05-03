//! # PssTuningConfig - Trait Implementations
//!
//! This module contains trait implementations for `PssTuningConfig`.
//!
//! ## Implemented Traits
//!
//! - `Default`
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

use super::types::PssType;
use super::types_3::PssTuningConfig;

impl Default for PssTuningConfig {
    fn default() -> Self {
        Self {
            target_damping: 0.05,
            target_gain_db: 20.0,
            freq_min_hz: 0.01,
            freq_max_hz: 10.0,
            n_freq_points: 100,
            pss_type: PssType::Pss1A,
        }
    }
}
