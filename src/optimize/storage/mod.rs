//! Battery storage optimisation: price arbitrage, sizing, and scheduling.
//!
//! - [`arbitrage`]    — Greedy price-based charge/discharge arbitrage scheduler
//! - [`sizing`]       — Storage sizing for peak shaving, solar shifting, backup, self-consumption
//! - [`multi_market`] — Multi-market co-optimization across energy, ancillary services, and capacity markets
pub mod arbitrage;
pub mod economic_dispatch;
pub mod multi_market;
pub mod sizing;
pub use economic_dispatch::*;
pub use multi_market::*;
