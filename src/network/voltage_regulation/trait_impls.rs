//! # VoltageRegulationSystem - Trait Implementations
//!
//! This module contains trait implementations for `VoltageRegulationSystem`.
//!
//! ## Implemented Traits
//!
//! - `Default`
//! - `Default`
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

use super::types::VvoOptimizer;
use super::types_3::VoltageRegulationSystem;

impl Default for VoltageRegulationSystem {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for VvoOptimizer {
    fn default() -> Self {
        Self::new()
    }
}
