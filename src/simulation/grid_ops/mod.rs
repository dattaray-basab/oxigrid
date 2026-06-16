//! Grid Operations Simulator.
//!
//! This module provides two simulation paradigms for grid operations:
//!
//! 1. **Legacy discrete-event operator-focused simulator** (`GridOpsSimulator`) —
//!    models operator skill, automation level, contingency rates, and workload
//!    metrics over multi-hour/year horizons.
//!
//! 2. **Event-driven quasi-dynamic simulator** (`GridOperationsSimulator`) —
//!    physics-based, timestep-accurate simulation with swing-equation frequency
//!    dynamics, AGC, UFLS, DC branch flows, and storage SoC tracking.
//!
//! # Quick Start (legacy)
//!
//! ```rust
//! use oxigrid::simulation::grid_ops::{GridOpsConfig, GridOpsSimulator};
//!
//! let config = GridOpsConfig {
//!     simulation_hours: 24,
//!     dt_minutes: 5.0,
//!     operator_skill: 0.85,
//!     automation_level: 0.7,
//!     contingency_probability: 0.02,
//!     weather_events: false,
//! };
//! let sim = GridOpsSimulator::new(config, 1000.0, 800.0);
//! let result = sim.simulate().expect("simulation failed");
//! assert!(result.system_reliability_pct > 90.0);
//! ```

pub mod constants;
pub mod functions;
pub mod gridopsconfig_traits;
pub mod qdgridopsconfig_traits;
pub mod types;
pub mod types_4;

// Re-export all types
pub use types::*;
pub use types_4::*;

#[cfg(test)]
mod tests;
