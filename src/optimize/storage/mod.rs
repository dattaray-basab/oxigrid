//! Battery storage optimisation: price arbitrage, sizing, and scheduling.
//!
//! - [`arbitrage`]                — Greedy price-based charge/discharge arbitrage scheduler
//! - [`sizing`]                   — Storage sizing for peak shaving, solar shifting, backup, self-consumption
//! - [`multi_market`]             — Multi-market co-optimization across energy, ancillary services, and capacity markets
//! - [`degradation_scheduling`]   — Degradation-aware storage scheduling (DP, rainflow, SEI)
//! - [`price_arbitrage`]          — Price arbitrage with deterministic/stochastic/rolling-horizon DP
pub mod arbitrage;
pub mod degradation_scheduling;
pub mod economic_dispatch;
pub mod multi_market;
pub mod portfolio;
pub mod price_arbitrage;
pub mod renewable_sizing;
pub mod sizing;
pub mod stochastic_dp;
pub mod thermal;
pub use economic_dispatch::*;
pub use multi_market::*;
