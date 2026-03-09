/// Grid code compliance modules for renewable energy generators.
///
/// Covers all major categories of grid code requirements that renewable
/// energy sources must satisfy for network connection and continued operation:
///
/// - [`lvrt`]               — Low Voltage Ride-Through envelope and reactive injection
/// - [`hvrt`]               — High Voltage Ride-Through envelope and reactive absorption
/// - [`frequency_response`] — Droop response, FFR, synthetic inertia, operating range
/// - [`reactive`]           — PQ-diagram capability and power factor requirements
/// - [`ramp`]               — Ramp rate limits and enforcement
///
/// # Standards covered
/// - IEC 61400-21 (wind turbines)
/// - ENTSO-E Requirements for Generators (RfG), 2016
/// - NERC PRC-024-2 / BAL-003-2
/// - German BDEW Medium-Voltage Guideline, 2008
pub mod frequency_response;
pub mod hvrt;
pub mod lvrt;
pub mod ramp;
pub mod reactive;

pub use frequency_response::{FfrRequirement, FrequencyResponseRequirement};
pub use hvrt::{HvrtEvent, HvrtProfile, HvrtResult};
pub use lvrt::{compute_lvrt_reactive_injection, LvrtEvent, LvrtProfile, LvrtResult};
pub use ramp::RampRateRequirement;
pub use reactive::{PqDiagram, ReactiveRequirement};
pub mod compliance_checker;
pub use compliance_checker::{
    interpolate_profile, ComplianceCategory, ComplianceChecker, ComplianceCheckerConfig,
    ComplianceError, ComplianceReport, ComplianceSeverity, ComplianceTest, GeneratorType,
    GridCodeStandard,
};
