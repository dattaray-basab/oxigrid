/// Digital twin framework for real-time grid monitoring and state tracking.
///
/// Provides a live synchronised model of a power system that ingests SCADA
/// and PMU telemetry, runs state estimation, and continuously monitors the
/// grid against configurable alert thresholds.
///
/// # Modules
///
/// - [`twin`]      — `GridDigitalTwin` core engine
/// - [`telemetry`] — SCADA/PMU telemetry ingestion (`TelemetryBatch`, `ScadaPoint`)
/// - [`alert`]     — Real-time alert engine (`AlertEngine`, `TwinAlert`)
/// - [`replay`]    — Historical replay and what-if analysis (`GridReplay`)
///
/// # Quick-start Example
///
/// ```rust,ignore
/// use oxigrid::digitaltwin::twin::{GridDigitalTwin, TwinConfig};
/// use oxigrid::digitaltwin::telemetry::{TelemetryBatch, TelemetrySource, ScadaPoint, ScadaMeasType};
///
/// let net = PowerNetwork::new(100.0);
/// let mut twin = GridDigitalTwin::new(net, TwinConfig::default());
///
/// let mut batch = TelemetryBatch::new(TelemetrySource::Scada, 0);
/// batch.add_scada(ScadaPoint::new(0, 0, ScadaMeasType::VoltageMagnitude, 0, 1.02, 0));
/// let alerts = twin.ingest_telemetry(&batch).unwrap();
/// ```
pub mod alert;
pub mod asset_digitization;
pub mod realtime_sync;
pub mod replay;
pub mod telemetry;
pub mod twin;

pub use alert::{AlertCategory, AlertEngine, AlertSeverity, AlertThresholds, TwinAlert};
pub use asset_digitization::{
    AgingReport, AssetCategory, AssetClass, AssetCondition, AssetDigitalTwin, AssetHealthReport,
    AssetLocation, AssetMaintenanceRecord, AssetMaintenanceType, AssetNameplate, AssetRegistry,
    AssetTelemetry, CbmAssessor, DigitalAssetRecord, FailureRecord, InspectionRecord,
    MaintenancePlan, MaintenanceScheduler, RiskLevel, TwinSyncStatus, TwinSynchronizer,
};
pub use realtime_sync::{
    DigitalTwinSync, MeasurementUpdate, PredictionState, RealtimeSyncConfig, SeType, TwinError,
    TwinSyncState,
};
pub use replay::{GridKpi, GridReplay, TwinModification};
pub use telemetry::{
    compute_telemetry_stats, ScadaMeasType, ScadaPoint, TelemetryBatch, TelemetrySource,
    TelemetryStats,
};
pub use twin::{
    DataQuality, GridDigitalTwin, StateSource, TopologyChange, TopologyChangeType, TwinConfig,
    TwinDivergence, TwinState,
};
