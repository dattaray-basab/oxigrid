//! EV fleet smart charging and Vehicle-to-Grid (V2G) optimisation.
//!
//! # Sub-modules
//!
//! | Module      | Description                                                       |
//! |-------------|-------------------------------------------------------------------|
//! | [`charging`]| Single-vehicle charging: uncontrolled, TOU, V2G DP, freq. reg.   |
//! | [`fleet`]   | Fleet aggregation: valley filling, peak shaving, coordination     |
//! | [`v2g`]     | V2G aggregator, grid services, flexibility envelopes              |
//!
//! # Quick Start
//!
//! ```rust,no_run
//! use oxigrid::optimize::ev::charging::{EvSession, SmartCharger};
//! use oxigrid::optimize::ev::fleet::{EvFleet, FleetCharger, FleetAlgorithm};
//!
//! // 15-min TOU prices (96 slots for 24h)
//! let prices: Vec<f64> = (0..96)
//!     .map(|i| if i < 32 || i >= 76 { 0.05 } else { 0.25 })
//!     .collect();
//! let charger = SmartCharger::new(0.25, prices, 22.0);
//!
//! let session = EvSession {
//!     vehicle_id: 1,
//!     arrival_time: 18.0,
//!     departure_time: 31.0,
//!     soc_arrival: 0.3,
//!     soc_target: 0.8,
//!     battery_kwh: 60.0,
//!     max_charge_kw: 11.0,
//!     max_discharge_kw: 7.4,
//!     ..Default::default()
//! };
//!
//! // Single-vehicle TOU optimisation
//! let schedule = charger.tou_optimized(&session).unwrap();
//! println!("Net cost: ${:.2}", schedule.net_cost);
//! ```

pub mod charging;
pub mod fleet;
pub mod grid_integration;
pub mod infrastructure;
pub mod infrastructure_planning;
pub mod v2g;

pub use charging::{ChargingSchedule, EvSession, SmartCharger};
pub use fleet::{EvFleet, FleetAlgorithm, FleetCharger, FleetScheduleResult};
pub use grid_integration::{
    ChargingLocation as GridChargingLocation, ChargingNetworkOptimizer, CoordinationMode,
    CoordinationResult, DrActivationResult, EvChargingSession, EvDemandResponse, EvFleetProfile,
    EvGridImpact, GridEvSynergy, NetworkChargerType, SmartChargingCoordinator,
    V2gRevenueCalculator,
};
pub use infrastructure::{
    ChargerType as InfraChargerType, ChargingInfraConfig, ChargingStation as InfraChargingStation,
    ChargingTariff, EvArrival, InfraError, InfrastructureResult, SmartChargingInfrastructure,
};
pub use infrastructure_planning::{
    haversine_km, ChargerCost, ChargerSizing, ChargerType, ChargingDemandForecaster,
    ChargingLocation, ChargingPreference, ChargingStation, DemandForecastYear, DemandNode,
    DrPotential, EquityReport, EvDemandForecast, EvInfraPlanner, GridConstraint, GridImpact,
    GridViolation, InfrastructurePlan, InfrastructurePlanConfig, InfrastructurePlanner,
    LocationType, PlacementPlan, PlanChargerType, PlanMetrics, SiteAllocation,
};
pub use v2g::{
    compute_flexibility_envelope, FlexibilityEnvelope, GridService, V2gAggregator, V2gResult,
};
