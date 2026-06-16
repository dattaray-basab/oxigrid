//! Quasi-steady-state time-series power flow simulation engine.
//!
//! Runs a DC-based quasi-static simulation over a time horizon (hours/days/years),
//! applying time-varying load and renewable generation profiles at each timestep.
//! Supports storage dispatch strategies, curtailment, and scenario analysis.
//!
//! # Usage
//! ```rust,ignore
//! let config = TimeSeriesConfig::default();
//! let mut sim = TimeSeriesSimulator::new(network, config);
//! let result = sim.run()?;
//! println!("Renewable fraction: {:.1}%", result.statistics.renewable_fraction_pct);
//! ```
//!
//! # Method
//! Each timestep is solved as an independent quasi-steady-state DC power flow
//! (B·θ = P). Voltage magnitudes are estimated via Q-sensitivity. Storage SoC
//! is tracked across timesteps. Curtailment is applied when voltage bounds are
//! violated and `enable_curtailment` is set.

pub mod functions;
pub mod timeseriesconfig_traits;
pub mod timeseriesstatistics_traits;
pub mod types;
pub mod types_4;

// Re-export all types
pub use types::*;
pub use types_4::*;
