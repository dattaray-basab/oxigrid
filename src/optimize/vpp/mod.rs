//! Virtual Power Plant (VPP) aggregation and energy storage fleet coordination.
//!
//! This module implements:
//!
//! - **VPP Aggregator** ([`aggregator`]): Pools heterogeneous Distributed Energy
//!   Resources (batteries, EV fleets, demand response, CHP, P2G, biogas, PV+storage,
//!   wind+storage) into a single dispatchable entity.  Provides capability-envelope
//!   computation, merit-order dispatch, price-responsive forecast, and aggregate metrics.
//!
//! - **Storage Fleet Coordination** ([`fleet_coord`]): Coordinates multiple battery
//!   storage units to jointly deliver a power setpoint, with five algorithms:
//!   merit order, equal-SoC, pro-rata, priority-based, and multi-period dynamic
//!   programming.
//!
//! - **VPP Market Bidding** ([`bidding`]): Risk-adjusted price-volume bid generation
//!   for day-ahead energy and ancillary service markets, plus Value at Risk (VaR)
//!   estimation via Monte-Carlo simulation (LCG-based, no external RNG).
//!
//! # References
//! - Pudjianto, D. et al., "Virtual power plant and system integration of distributed
//!   energy resources", IET Renewable Power Generation 1(1), 2007.
//! - Giuntoli, M. & Poli, D., "Optimized Thermal and Electrical Scheduling of a Large
//!   Scale Virtual Power Plant in the Presence of Energy Storages", IEEE Trans. Smart
//!   Grid, 2013.
//! - Morales, J.M. et al., "Integrating Renewables in Electricity Markets", Springer, 2014.

pub mod advanced_fleet;
pub mod aggregator;
pub mod bidding;
pub mod derms;
pub mod fleet_coord;

pub use aggregator::{
    DerResource, DerState, DerType, VirtualPowerPlant, VppDispatchResult, VppEnvelope, VppMetrics,
};
pub use bidding::{coordinate_vpp_bids, MarketBid, MarketService, VppBiddingStrategy};
pub use fleet_coord::{
    CoordinationAlgorithm, FleetDispatch, FleetStorageUnit, StorageFleetCoordinator,
};
