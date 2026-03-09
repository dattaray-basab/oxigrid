//! Optimisation modules: OPF, economic dispatch, microgrid EMS, battery storage.
//!
//! # Sub-modules
//!
//! | Module | Description |
//! |--------|-------------|
//! | [`opf`]       | DC-OPF (lambda-iteration), AC-OPF (SQP/penalty), N-1 SCOPF |
//! | [`dispatch`]  | Multi-period economic dispatch, priority-list unit commitment |
//! | [`microgrid`] | Rule-based EMS, islanding detection, P2P energy market |
//! | [`storage`]   | Price-arbitrage dispatch, battery sizing optimisation |
pub mod demand_response;
pub mod dispatch;
pub mod ev;
pub mod expansion;
pub mod hydrogen;
pub mod market;
pub mod mes;
pub mod microgrid;
pub mod multi_energy;
pub mod opf;
pub mod reliability;
pub mod restoration;
pub mod storage;
pub mod vpp;

pub use multi_energy::*;
