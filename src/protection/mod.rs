//! Power system protection: fault analysis, relay models, and protection coordination.
//!
//! # Modules
//! - [`fault`]        — Z-bus fault current (3-phase, L-G, L-L), DC offset factor
//! - [`relay`]        — IEC 60255 / IEEE C37.112 overcurrent relay, Mho distance relay
//! - [`coordination`] — TCC curve coordination, CTI (coordination time interval) check
pub mod autorecloser;
pub mod coordination;
pub mod coordination_advanced;
pub mod coordination_optimizer;
pub mod coordination_study;
pub mod differential;
pub mod distance;
pub mod fault;
pub mod fault_asymmetric;
pub mod fault_current_limiter;
pub mod fault_symmetric;
pub mod hif;
pub mod ibr_protection;
pub mod iec60909;
pub mod microgrid_protection;
pub mod motor_protection;
pub mod protection_testing;
pub mod relay;
pub mod shortcircuit;
pub mod zone_protection;
pub use coordination_optimizer::*;
pub use fault_current_limiter::*;
pub use protection_testing::*;
pub use zone_protection::{
    CoordinationIssue, CoordinationResult, DifferentialZone, DifferentialZoneType,
    DistanceCharacteristic, DistanceRelay, DistanceZone, FaultLocation, ProtFaultType,
    ProtectionPerformance, ProtectionZoneMap, ZoneCoordinator, ZoneCoverage, ZoneDirectional,
};
