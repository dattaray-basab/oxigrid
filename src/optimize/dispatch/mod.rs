//! Generator dispatch and unit commitment.
//!
//! - [`economic`]    — Multi-period economic dispatch with generator ramp limits
//! - [`unit_commit`] — Priority-list unit commitment with minimum on/off time constraints
//! - [`eco_env`]     — Economic-environmental dispatch (EED) with Pareto front and cap-and-trade
pub mod decommitment;
pub mod eco_env;
pub use eco_env::*;
pub mod economic;
pub mod milp_uc;
pub mod op_planning;
pub mod ramp_product;
pub mod reserve_sharing;
pub mod stochastic_uc;
pub mod unit_commit;
