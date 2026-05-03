//! Battery equivalent circuit models (ECMs).
//!
//! Three ECM complexities are provided, all implementing [`crate::battery::BatteryModel`]:
//!
//! | Model | Struct | Parameters |
//! |-------|--------|-----------|
//! | Rint  | [`RintModel`]  | OCV(SoC), R₀ |
//! | 1RC   | [`OneRcModel`] | OCV(SoC), R₀, R₁, C₁ |
//! | 2RC   | [`TwoRcModel`] | OCV(SoC), R₀, R₁, C₁, R₂, C₂ |
//!
//! All models support temperature-dependent R₀ and capacity derating.
mod lbfgs;
pub mod parameter;
pub mod parameter_id;
pub mod rc;
pub mod rint;

pub use parameter::ParameterSet;
pub use rc::{OneRcModel, TwoRcModel};
pub use rint::RintModel;
