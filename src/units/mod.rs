//! Physical units for energy systems modelling.
//!
//! All quantities use newtype wrappers to prevent unit confusion at compile time.
//! The inner `f64` stores the canonical SI-adjacent unit documented on each type.
//!
//! # Sub-modules
//! - [`electrical`] — Voltage (V), Current (A), Power (W), ReactivePower (VAr),
//!   Impedance (Ω), Frequency (Hz), PerUnit
//! - [`thermal`]    — Temperature (K), ThermalConductivity, HeatCapacity
//! - [`energy`]     — Energy (Wh), Capacity (Ah), StateOfCharge [0, 1]
//! - [`conversion`] — `base_impedance()`, `base_current()` per-unit helpers
//!
//! ## Mathematical background
//!
//! Per-unit normalization converts physical quantities to dimensionless ratios:
//!
//! ```text
//! Z_pu    = Z_actual / Z_base
//! Z_base  = V_base² / S_base        `Ω`
//! I_base  = S_base / (√3 · V_base)  `A` (three-phase)
//! Y_base  = 1 / Z_base              [S]
//! ```
//!
//! where S_base is the system MVA base and V_base is the line-to-line kV base.
//! All `PerUnit` conversions in this module follow these formulas.
pub mod conversion;
pub mod electrical;
pub mod energy;
pub mod thermal;

pub use conversion::*;
pub use electrical::*;
pub use energy::*;
pub use thermal::*;
