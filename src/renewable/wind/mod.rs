//! Wind energy modelling: turbine power curves, wake effects, and wind farm layout.
//!
//! - [`turbine`] — Power curve (piecewise-linear), Betz limit (16/27 Cp),
//!   Weibull annual energy production (AEP), log wind profile
//! - [`wake`]    — Jensen top-hat wake model, Frandsen wake model,
//!   square-sum superposition, meteorological wind convention
//! - [`farm`]    — Regular grid wind farm layout with wake-corrected power output
pub mod farm;
pub mod plant_ops;
pub mod spatial;
pub mod turbine;
pub mod wake;

pub use plant_ops::*;
