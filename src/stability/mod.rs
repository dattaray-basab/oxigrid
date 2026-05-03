//! Power system stability analysis: transient, small-signal, and voltage.
//!
//! # Modules
//! - [`transient`]    — Swing equation, RK4 time-domain simulation, SMIB fault analysis
//! - [`small_signal`] — Multi-machine state-space (A-matrix), Schur eigenvalue decomposition,
//!   inter-area and local oscillation mode identification
//! - [`voltage`]      — P-V curve, Q-V curve, voltage stability index (L-index)
//! - [`generator`]    — Classical, detailed (4th-order d-q axis), governor, and AVR models
//!
//! ## Mathematical background
//!
//! **Swing equation** (classical generator model, SMIB):
//!
//! ```text
//! M · d²δ/dt² + D · dδ/dt = Pm − Pe(δ)
//! Pe(δ) = (E'·V / X'd) · sin(δ)
//! ```
//!
//! where M = 2H/ωs (inertia constant), D = damping, Pm = mechanical power,
//! Pe(δ) = electrical power, H = inertia in MWs/MVA, ωs = synchronous speed.
//!
//! **Small-signal stability**: The system A-matrix eigenvalues λ = σ ± jω
//! determine stability. System is stable iff Re(λ) < 0 for all eigenvalues.
//! Oscillation frequency f = ω/(2π); damping ratio ζ = −σ/|λ|.
//!
//! **Voltage stability**: The P-V nose point (bifurcation) is found via
//! continuation power flow. The L-index for bus i:
//!
//! ```text
//! L_i = |1 + Σ_{j ∈ G} F_ij · V_j / V_i|
//! ```
//!
//! L_i ∈ \[0, 1\]; L_i → 1 indicates voltage collapse proximity.
pub mod agc;
pub mod fault_trajectory;
pub mod generator;
pub mod inertia_analysis;
pub mod inertia_emulation;
pub mod inertia_estimation;
pub mod load;
pub mod load_modeling;
pub use load_modeling::*;
pub mod modal;
pub mod multi_machine;
pub mod multi_machine_detail;
pub mod probabilistic_transient;
pub mod small_signal;
pub mod small_signal_enhanced;
pub mod synthetic_inertia_control;
pub mod transient;
pub mod voltage;
pub mod voltage_stability;
pub use inertia_analysis::*;
pub mod pss_design;
pub mod pss_tuning;
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
pub mod indices;
pub use indices::{
    BranchStabilityData, BusStabilityData, BusType, StabilityAssessment, StabilityError,
    StabilityIndexCalculator, StabilityIndicesConfig, StabilityRisk, TransientStabilityIndices,
    TsMethod, VoltageStabilityIndices, VsMethod,
};
