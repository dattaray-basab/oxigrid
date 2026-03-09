//! Wide-area monitoring, PMU analytics, and substation automation.
//!
//! Provides a PMU-based Wide-Area Monitoring System (WAMS) for real-time
//! power system situational awareness including angle stability monitoring,
//! inter-area oscillation detection, and automatic alarm generation.
//!
//! Also includes a full Substation Automation System (SAS) with bay
//! controllers, IED data processing, SCADA reporting, and maintenance
//! diagnostics per IEC 61850 concepts.
pub mod equipment_health;
pub mod frequency_monitoring;
pub mod micro_scada;
pub mod oscillation;
pub mod substation;
pub mod wams;

pub use equipment_health::{
    BreakerHealthIndicators, DgaResult, EquipmentHealthConfig, EquipmentHealthMonitor, HealthError,
    HealthScore, MaintenanceAction, RiskLevel, TransformerHealthIndicators,
};
pub use frequency_monitoring::{
    FrequencyComplianceReport, FrequencyMeasurement, FrequencyMeter, FrequencyRelay,
    InertiaEstimator, NadirEstimate, NadirEstimator, RelayAction, RocofRelay, UflsScheme, UflsStep,
};
pub use micro_scada::{
    AlarmPriority, ControlCommand, ControlLog, MeasurementQuality, MicroScada, RtuDevice,
    RtuProtocol, ScadaAlarm, ScadaDisplayPage, ScadaKpis, TagBuilder, TagValue,
};
pub use oscillation::{
    AlarmLevel, AlertSeverity, DampingTrend, DetectedMode, FourierOscillationDetector, ModeType,
    OscillationAlarmLevel, OscillationAlert, OscillationError, OscillationMode, OscillationMonitor,
    OscillationMonitorConfig, OscillationMonitorResult, OscillationReport, PronyAnalyzer,
    PssTuningRecommendation, RingdownAnalyzer, RingdownResult, SpectrumResult,
    WamOscillationMonitor,
};
pub use substation::{
    Bay, BayType, BreakerStatus, BusbarConfig, EventSeverity, EventType, ProtectionStatus,
    SasError, ScadaReport as SubstationScadaReport, SubstationAutomationSystem, SubstationConfig,
    SubstationEvent,
};
pub use wams::{
    AlarmSeverity, AngleTrend, AngularStabilityIndex, InterAreaMode, PmuReading, WamsAlarm,
    WamsConfig, WamsError, WamsMonitor, WamsResult,
};
