//! Advanced inverter control models for grid-connected and grid-forming applications.
//!
//! # Modules
//! - [`grid_forming`]  — Virtual Synchronous Machine (VSM) + droop control,
//!   microgrid simulator
//! - [`grid_following`] — PLL-based grid-following inverter with dq current control
//! - [`lcl_filter`]    — LCL output filter state-space dynamics (RK4)
//! - [`voc`]           — Virtual Oscillator Control (VOC): Van der Pol oscillator
//!   in αβ frame, single and parallel inverter simulation
pub mod grid_following;
pub mod grid_forming;
pub mod lcl_filter;
pub mod smart_inverter;
pub mod voc;

pub use grid_following::{GridFollowingInverter, PhaseLockedLoop, PllConfig, PllState};
pub use grid_forming::{
    MicrogridSimResult, MicrogridSimulator, VirtualSynchronousMachine, VsmConfig, VsmOutput,
    VsmState,
};
pub use lcl_filter::{LclFilter, LclState};
pub use smart_inverter::{
    InverterOutput, SmartInverter, SmartInverterConfig, SmartInverterMode, VoltVarCurve,
    VoltWattCurve,
};
pub use voc::{VocConfig, VocError, VocResult, VocSimulator, VocState};
