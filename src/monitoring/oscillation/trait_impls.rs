//! # OscillationMonitorConfig - Trait Implementations
//!
//! This module contains trait implementations for `OscillationMonitorConfig`.
//!
//! ## Implemented Traits
//!
//! - `Default`
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

use super::types::OscillationMonitorConfig;

impl Default for OscillationMonitorConfig {
    fn default() -> Self {
        Self {
            sampling_rate_hz: 50.0,
            analysis_window_s: 10.0,
            min_mode_energy: 0.001,
            frequency_range_hz: (0.1, 3.0),
            alarm_damping_threshold: 0.05,
            alarm_amplitude_threshold: 0.05,
        }
    }
}
