//! # `GridOpsConfig` - Trait Implementations
//!
//! This module contains trait implementations for `GridOpsConfig`.
//!
//! ## Implemented Traits
//!
//! - `Default`
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

use super::types_4::GridOpsConfig;

impl Default for GridOpsConfig {
    fn default() -> Self {
        Self {
            simulation_hours: 8760,
            dt_minutes: 60.0,
            operator_skill: 0.8,
            automation_level: 0.5,
            contingency_probability: 0.05,
            weather_events: true,
        }
    }
}
