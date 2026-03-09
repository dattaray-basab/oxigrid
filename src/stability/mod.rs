//! Power system stability analysis: transient, small-signal, and voltage.
//!
//! # Modules
//! - [`transient`]    — Swing equation, RK4 time-domain simulation, SMIB fault analysis
//! - [`small_signal`] — Multi-machine state-space (A-matrix), Schur eigenvalue decomposition,
//!   inter-area and local oscillation mode identification
//! - [`voltage`]      — P-V curve, Q-V curve, voltage stability index (L-index)
//! - [`generator`]    — Classical, detailed (4th-order d-q axis), governor, and AVR models
pub mod fault_trajectory;
pub mod generator;
pub mod inertia_analysis;
pub mod load;
pub mod load_modeling;
pub use load_modeling::*;
pub mod modal;
pub mod multi_machine;
pub mod small_signal;
pub mod transient;
pub mod voltage;
pub use inertia_analysis::*;
pub mod pss_design;
pub use pss_design::*;
pub mod black_start_procedure;
pub use black_start_procedure::*;
pub mod tsmi;
pub use tsmi::*;
pub mod tvsa;
pub use tvsa::*;
pub mod grid_forming_stability;
pub use grid_forming_stability::*;
pub mod inter_area;
pub use inter_area::{
    IaGenerator, InterAreaAnalyzer, InterAreaMode, PronyMode, PronyResult, RingdownResult,
    SystemArea, TieLine, WadcDesign,
};
