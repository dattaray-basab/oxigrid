//! Power electronics converter modeling — average-value models.
//!
//! Provides average-value (state-space averaged) models for power electronic
//! converter circuits commonly used in grid-connected applications:
//!
//! | Module | Description |
//! |--------|-------------|
//! | [`converter`] | VSC / NPC / MMC / H-Bridge / Buck-Boost average models |
//!
//! # Control Architecture
//!
//! All converters use a cascaded control structure:
//! - **Inner loop**: PI current controller in the rotating dq frame
//! - **Outer loop**: Power/voltage PI controller that provides current setpoints
//!
//! # References
//! - Holmes & Lipo, "Pulse Width Modulation for Power Converters", 2003.
//! - Yazdani & Iravani, "Voltage-Source Converters in Power Systems", 2010.

pub mod converter;
pub mod converters;

pub use converter::{
    ConverterController, ConverterError, ConverterModel, ConverterResult, ConverterSimulator,
    ConverterState, ConverterTopology,
};
pub use converters::*;
