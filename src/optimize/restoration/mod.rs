//! Power system restoration: black-start sequence planning, energisation
//! scheduling, and reliability impact assessment.
//!
//! # Overview
//!
//! A system-wide blackout requires a carefully ordered restoration procedure.
//! This module implements:
//!
//! - **`black_start`** — Core planning algorithm: greedy heuristic that starts
//!   black-start-capable generators, cranks non-black-start units over
//!   transmission paths, and picks up load blocks respecting cold-load pickup
//!   transients and spinning-reserve margins.
//!
//! - **`sequence`** — Parallel path discovery and load-block ordering, plus
//!   SAIDI/SAIFI/ENS metrics for quantifying reliability impact.
//!
//! - **`constraints`** — Per-step voltage, frequency, thermal, reserve-margin,
//!   and back-feed constraint checkers.
//!
//! # Quick-start
//!
//! ```rust,ignore
//! use oxigrid::optimize::restoration::black_start::{
//!     BlackStartConfig, BlackStartPlanner, BlackStartUnit, LoadBlock,
//! };
//! use oxigrid::network::topology::PowerNetwork;
//!
//! let config = BlackStartConfig {
//!     black_start_units: vec![BlackStartUnit {
//!         gen_id: 0, bus: 1, p_rated_mw: 100.0, p_min_mw: 20.0,
//!         ramp_rate_mw_per_min: 5.0, crank_time_min: 15.0,
//!         max_crank_distance_km: 50.0, auxiliary_load_mw: 5.0, priority: 1,
//!     }],
//!     load_blocks: vec![LoadBlock {
//!         block_id: 0, buses: vec![2], base_demand_mw: 30.0,
//!         cold_load_pickup_factor: 2.5, cold_load_decay_min: 30.0,
//!         priority: 1, can_defer: false,
//!     }],
//!     ..Default::default()
//! };
//!
//! let planner = BlackStartPlanner::new(config);
//! let network = PowerNetwork::new(100.0); // build or load actual network
//! let plan = planner.plan(&network)?;
//! println!("Feasible: {}, total time: {:.0} min", plan.feasible, plan.total_time_min);
//! ```

pub mod adaptive_load_shedding;
pub mod black_start;
pub mod constraints;
pub mod sequence;

pub use black_start::{
    BlackStartConfig, BlackStartPlanner, BlackStartUnit, EnergizationPath, LoadBlock,
    RestorationAction, RestorationPlan, RestorationStep,
};
pub use constraints::{
    audit_plan, check_thermal_constraint, check_voltage_constraint, find_backfeed_violations,
    find_frequency_violations, find_reserve_violations, path_within_crank_range, ConstraintReport,
};
pub use sequence::{RestorationMetrics, RestorationSequencer};
