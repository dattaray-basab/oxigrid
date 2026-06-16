//! Power system data analytics and Key Performance Indicator (KPI) computation.
//!
//! This module provides standard power system performance metrics covering
//! reliability, power quality, economics, and environmental impact.
//!
//! # Standard Indices
//!
//! - **SAIDI** (IEEE 1366) — System Average Interruption Duration Index
//! - **SAIFI** (IEEE 1366) — System Average Interruption Frequency Index
//! - **CAIDI** — Customer Average Interruption Duration = SAIDI / SAIFI
//! - **ASAI** — Average Service Availability Index = 1 − SAIDI / 8760
//! - **ENS** — Energy Not Supplied \[MWh\]
//! - **THD** — Total Harmonic Distortion per IEC 61000-3-2 / IEEE 519
//! - **LCOE** — Levelized Cost of Energy \[$/MWh\]
//! - **NPV** — Net Present Value \[USD\]
//! - **IRR** — Internal Rate of Return
pub mod carbon;
pub mod carbon_forecast;
pub mod energy_equity;
pub mod grid_health;
pub mod grid_operations;
pub mod metrics;
pub mod operational_analytics;
pub mod predictive_maintenance;

pub use carbon::{
    CarbonAccountant, CarbonAccountingConfig, CarbonAnalysisResult, CarbonError, CarbonMetrics,
    DispatchPoint, GeneratorType, MarginalEmissionMethod,
};
pub use carbon_forecast::{
    CarbonForecastConfig, CarbonForecastResult, CarbonIntensityForecaster, FuelType,
    GeneratorEmission, MarginalEmissionResult,
};
pub use energy_equity::{
    AffordabilityMethod, EnergyEquityAnalyzer, EnergyEquityConfig, EnergyEquityResult, EquityError,
    EquityMetrics, HouseholdGroup, HousingType, PolicyIntervention,
};
pub use grid_health::{
    components_below_threshold, weighted_average_score, CategoryAggregate, CategoryWeight,
    ComponentCategory, ComponentHealth, GridHealthError, GridHealthReport, GridHealthScorer,
    HealthStatus,
};
pub use grid_operations::{
    AlertSeverity, CongestionAnalyzer, DayType, DemandAnalytics, GeneratorFuelType,
    GeneratorPerformance, OperationalKpis, OperationsAlert, OperationsReport, RenewableMetrics,
    RenewableSummary,
};
pub use metrics::{
    AnalyticsError, EconomicMetrics, EnvironmentalMetrics, GridKpiDashboard, KpiInput,
    PowerQualityMetrics, SystemPerformanceMetrics,
};
pub use operational_analytics::{
    AnalyticsInterval, AnomalyScore, EfficiencyReport, OperationalAnalytics, OperationalDashboard,
    OperationalKpi, OperationalKpiCategory, TimeSeriesKpi, TrendDirection,
};
pub use predictive_maintenance::{
    AssetCondition, AssetType, ConditionIndicator, DegradationModel, FailureMode, HealthIndex,
    MaintenanceEvent, MaintenanceRecommendation, MaintenanceSchedule, MaintenanceType, PlannedTask,
    PredictiveMaintenance, RulEstimate,
};
