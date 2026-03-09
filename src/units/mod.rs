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
pub mod conversion;
pub mod electrical;
pub mod energy;
pub mod thermal;

pub use conversion::*;
pub use electrical::*;
pub use energy::*;
pub use thermal::*;
