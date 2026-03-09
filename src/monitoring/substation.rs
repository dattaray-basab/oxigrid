//! Substation Automation System (SAS).
//!
//! Implements bay controller, IED data processing, breaker operations,
//! protection status tracking, and SCADA report generation for substation
//! automation per IEC 61850 conceptual model.
//!
//! # Key Structures
//!
//! - [`SubstationAutomationSystem`] — top-level SAS controller
//! - [`Bay`] — individual bay with breaker, disconnectors, measurements
//! - [`BreakerStatus`] — contact wear, SF6 pressure, trip count
//! - [`ScadaReport`] — periodic SCADA telemetry snapshot
//!
//! # Units
//!
//! Currents in \[A\], powers in \[MW\] / \[MVAr\], voltages in \[pu\] or \[kV\],
//! pressures in \[bar\], timestamps in \[s\] (Unix epoch or relative).

use serde::{Deserialize, Serialize};
use thiserror::Error;

// ---------------------------------------------------------------------------
// Error type
// ---------------------------------------------------------------------------

/// Errors produced by the Substation Automation System.
#[derive(Debug, Error)]
pub enum SasError {
    /// Requested bay index was not found.
    #[error("bay not found: id={0}")]
    BayNotFound(usize),
    /// Synchronism check failed — close rejected for safety.
    #[error("synchronism check failed: close rejected")]
    SyncCheckFailed,
    /// Breaker is already in the requested state.
    #[error("breaker already in requested state")]
    BreakerStateConflict,
    /// Measurement value is invalid (NaN, out of range, etc.)
    #[error("invalid measurement: {0}")]
    InvalidMeasurement(String),
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Top-level substation configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SubstationConfig {
    /// Substation name (e.g. "Main 220 kV SS").
    pub name: String,
    /// Nominal voltage level \[kV\].
    pub voltage_kv: f64,
    /// Number of bays in the substation.
    pub n_bays: usize,
    /// Busbar arrangement scheme.
    pub busbar_configuration: BusbarConfig,
    /// IED polling interval \[ms\].
    pub ied_polling_interval_ms: f64,
    /// SCADA reporting interval \[s\].
    pub scada_report_interval_s: f64,
}

impl Default for SubstationConfig {
    fn default() -> Self {
        Self {
            name: "Unnamed Substation".into(),
            voltage_kv: 110.0,
            n_bays: 4,
            busbar_configuration: BusbarConfig::SingleBus,
            ied_polling_interval_ms: 100.0,
            scada_report_interval_s: 10.0,
        }
    }
}

/// Busbar switching arrangement.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BusbarConfig {
    /// Single busbar — simplest scheme.
    SingleBus,
    /// Double busbar with bus coupler.
    DoubleBus,
    /// 1.5-breaker scheme (breaker-and-a-half).
    OneAndHalf,
    /// Ring bus — all bays connected in a ring.
    RingBus,
}

// ---------------------------------------------------------------------------
// Bay
// ---------------------------------------------------------------------------

/// A substation bay containing primary equipment.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Bay {
    /// Unique bay identifier.
    pub id: usize,
    /// Human-readable name (e.g. "Line 1 Bay").
    pub name: String,
    /// Functional type of this bay.
    pub bay_type: BayType,
    /// Circuit breaker status.
    pub breaker_status: BreakerStatus,
    /// Disconnector positions (true = closed).
    pub disconnector_status: Vec<bool>,
    /// Measured line current \[A\].
    pub measured_current_a: f64,
    /// Measured active power \[MW\].
    pub measured_power_mw: f64,
    /// Measured reactive power \[MVAr\].
    pub measured_reactive_mvar: f64,
    /// Current protection / alarm status.
    pub protection_status: ProtectionStatus,
    /// Rated current for alarm thresholds \[A\] (0 = no limit).
    pub rated_current_a: f64,
}

impl Bay {
    /// Create a new bay with defaults.
    pub fn new(id: usize, name: impl Into<String>, bay_type: BayType) -> Self {
        Self {
            id,
            name: name.into(),
            bay_type,
            breaker_status: BreakerStatus::default(),
            disconnector_status: vec![true, true],
            measured_current_a: 0.0,
            measured_power_mw: 0.0,
            measured_reactive_mvar: 0.0,
            protection_status: ProtectionStatus::Normal,
            rated_current_a: 0.0,
        }
    }
}

/// Functional type of a substation bay.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BayType {
    /// Power transformer bay.
    Transformer,
    /// Outgoing feeder / line bay.
    FeederLine,
    /// Busbar section or coupler bay.
    Busbar,
    /// Capacitor bank bay.
    Capacitor,
    /// Shunt reactor bay.
    Reactor,
    /// Generator connection bay.
    Generator,
    /// Voltage transformer (VT) measurement bay.
    MeasurementVt,
}

/// Status and health of a circuit breaker.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BreakerStatus {
    /// `true` if breaker is closed (conducting).
    pub is_closed: bool,
    /// Cumulative number of trip operations.
    pub trip_count: usize,
    /// Timestamp of the last open/close operation \[s\].
    pub last_operation: Option<f64>,
    /// Contact mechanical wear as percentage \[%\] (0–100).
    pub contact_wear_pct: f64,
    /// SF6 gas pressure \[bar\].
    pub sf6_pressure_bar: f64,
}

impl Default for BreakerStatus {
    fn default() -> Self {
        Self {
            is_closed: true,
            trip_count: 0,
            last_operation: None,
            contact_wear_pct: 0.0,
            sf6_pressure_bar: 6.0,
        }
    }
}

/// Protection and alarm state of a bay.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ProtectionStatus {
    /// No active alarms or trips.
    Normal,
    /// An alarm condition is active.
    AlarmActive(String),
    /// Protection has tripped the breaker.
    ProtectionTripped {
        /// Time of trip \[s\].
        time_s: f64,
        /// Root cause description.
        cause: String,
    },
    /// Bay protection is under test.
    InTest,
    /// Bay is deliberately out of service.
    OutOfService,
}

// ---------------------------------------------------------------------------
// Events
// ---------------------------------------------------------------------------

/// A timestamped event logged by the SAS.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SubstationEvent {
    /// Event timestamp \[s\].
    pub timestamp_s: f64,
    /// Category of event.
    pub event_type: EventType,
    /// Bay that generated the event (if applicable).
    pub bay_id: Option<usize>,
    /// Free-text description.
    pub description: String,
    /// Severity classification.
    pub severity: EventSeverity,
}

/// Category of a substation event.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum EventType {
    /// Breaker open or close operation.
    BreakerOperation,
    /// Protection relay action.
    Protection,
    /// Alarm raised or cleared.
    Alarm,
    /// Measurement update or anomaly.
    Measurement,
    /// Operator control action.
    Control,
    /// Communication fault or restore.
    Communication,
}

/// Severity level of an event.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum EventSeverity {
    /// Informational — no action required.
    Information,
    /// Warning — monitor closely.
    Warning,
    /// Major — corrective action required.
    Major,
    /// Critical — immediate action required.
    Critical,
}

// ---------------------------------------------------------------------------
// SCADA report
// ---------------------------------------------------------------------------

/// Periodic SCADA telemetry snapshot of the substation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScadaReport {
    /// Report generation timestamp \[s\].
    pub timestamp_s: f64,
    /// Substation name.
    pub substation: String,
    /// Sum of all bay active loads \[MW\].
    pub total_load_mw: f64,
    /// Sum of all bay reactive loads \[MVAr\].
    pub total_reactive_mvar: f64,
    /// Estimated busbar voltage \[pu\].
    pub busbar_voltage_pu: f64,
    /// Number of active alarms.
    pub n_alarms: usize,
    /// Number of protection trips.
    pub n_trips: usize,
    /// Per-bay status: (bay_id, breaker_closed, power_mw).
    pub bay_statuses: Vec<(usize, bool, f64)>,
}

// ---------------------------------------------------------------------------
// Main SAS struct
// ---------------------------------------------------------------------------

/// Substation Automation System controller.
///
/// Manages bay state, IED data processing, breaker operations,
/// event logging, and SCADA report generation.
pub struct SubstationAutomationSystem {
    config: SubstationConfig,
    bays: Vec<Bay>,
    event_log: Vec<SubstationEvent>,
}

impl SubstationAutomationSystem {
    /// Create a new SAS with the given configuration and no bays.
    pub fn new(config: SubstationConfig) -> Self {
        Self {
            config,
            bays: Vec::new(),
            event_log: Vec::new(),
        }
    }

    /// Add a bay to the substation.
    pub fn add_bay(&mut self, bay: Bay) {
        self.bays.push(bay);
    }

    /// Find a bay by ID (mutable).
    fn find_bay_mut(&mut self, bay_id: usize) -> Result<&mut Bay, SasError> {
        self.bays
            .iter_mut()
            .find(|b| b.id == bay_id)
            .ok_or(SasError::BayNotFound(bay_id))
    }

    /// Find a bay by ID (immutable).
    fn find_bay(&self, bay_id: usize) -> Result<&Bay, SasError> {
        self.bays
            .iter()
            .find(|b| b.id == bay_id)
            .ok_or(SasError::BayNotFound(bay_id))
    }

    /// Process incoming IED current measurement for a bay.
    ///
    /// Updates the bay's `measured_current_a` and generates events when:
    /// - Current exceeds 2× rated → Warning alarm
    /// - Current exceeds 3× rated → Critical alarm and auto-trip
    ///
    /// Returns the list of events generated during this update.
    pub fn process_ied_data(
        &mut self,
        bay_id: usize,
        current_a: f64,
        timestamp_s: f64,
    ) -> Result<Vec<SubstationEvent>, SasError> {
        if current_a.is_nan() || current_a < 0.0 {
            return Err(SasError::InvalidMeasurement(format!(
                "current_a={current_a} is invalid"
            )));
        }

        // Snapshot rated current before mutable borrow
        let rated = {
            let bay = self.find_bay(bay_id)?;
            bay.rated_current_a
        };

        // Update measurement
        {
            let bay = self.find_bay_mut(bay_id)?;
            bay.measured_current_a = current_a;
        }

        let mut new_events: Vec<SubstationEvent> = Vec::new();

        // Generate events based on threshold multiples
        if rated > 0.0 {
            let ratio = current_a / rated;
            if ratio >= 3.0 {
                // Critical overcurrent → auto-trip
                let trip_event = SubstationEvent {
                    timestamp_s,
                    event_type: EventType::Protection,
                    bay_id: Some(bay_id),
                    description: format!(
                        "Critical overcurrent {current_a:.1} A ({:.1}× rated) — auto-trip",
                        ratio
                    ),
                    severity: EventSeverity::Critical,
                };
                new_events.push(trip_event.clone());
                self.event_log.push(trip_event);

                // Perform the trip
                let bay = self.find_bay_mut(bay_id)?;
                bay.breaker_status.is_closed = false;
                bay.breaker_status.trip_count += 1;
                bay.breaker_status.last_operation = Some(timestamp_s);
                bay.protection_status = ProtectionStatus::ProtectionTripped {
                    time_s: timestamp_s,
                    cause: format!("overcurrent {current_a:.1} A"),
                };
            } else if ratio >= 2.0 {
                let alarm_event = SubstationEvent {
                    timestamp_s,
                    event_type: EventType::Alarm,
                    bay_id: Some(bay_id),
                    description: format!(
                        "High overcurrent alarm {current_a:.1} A ({:.1}× rated)",
                        ratio
                    ),
                    severity: EventSeverity::Warning,
                };
                new_events.push(alarm_event.clone());
                self.event_log.push(alarm_event);

                let bay = self.find_bay_mut(bay_id)?;
                bay.protection_status =
                    ProtectionStatus::AlarmActive(format!("overcurrent {current_a:.1} A"));
            }
        }

        // Always log a measurement event at Information level
        let meas_event = SubstationEvent {
            timestamp_s,
            event_type: EventType::Measurement,
            bay_id: Some(bay_id),
            description: format!("IED update: current={current_a:.2} A"),
            severity: EventSeverity::Information,
        };
        new_events.push(meas_event.clone());
        self.event_log.push(meas_event);

        Ok(new_events)
    }

    /// Trip a breaker (open it) due to a protection or control action.
    ///
    /// Increments `trip_count`, updates `last_operation`, sets `ProtectionTripped`
    /// status, and appends an event to the log.
    pub fn trip_breaker(
        &mut self,
        bay_id: usize,
        cause: &str,
        timestamp_s: f64,
    ) -> Result<(), SasError> {
        let bay = self.find_bay_mut(bay_id)?;
        bay.breaker_status.is_closed = false;
        bay.breaker_status.trip_count += 1;
        bay.breaker_status.last_operation = Some(timestamp_s);
        bay.protection_status = ProtectionStatus::ProtectionTripped {
            time_s: timestamp_s,
            cause: cause.to_string(),
        };

        let event = SubstationEvent {
            timestamp_s,
            event_type: EventType::Protection,
            bay_id: Some(bay_id),
            description: format!("Breaker tripped: {cause}"),
            severity: EventSeverity::Major,
        };
        self.event_log.push(event);
        Ok(())
    }

    /// Close a breaker after a synchronism check.
    ///
    /// If `sync_ok` is `false`, the close is refused and [`SasError::SyncCheckFailed`]
    /// is returned — no state is changed. On success, `is_closed` becomes `true`.
    pub fn close_breaker(
        &mut self,
        bay_id: usize,
        sync_ok: bool,
        timestamp_s: f64,
    ) -> Result<(), SasError> {
        if !sync_ok {
            return Err(SasError::SyncCheckFailed);
        }

        let bay = self.find_bay_mut(bay_id)?;
        bay.breaker_status.is_closed = true;
        bay.breaker_status.last_operation = Some(timestamp_s);
        bay.protection_status = ProtectionStatus::Normal;

        let event = SubstationEvent {
            timestamp_s,
            event_type: EventType::BreakerOperation,
            bay_id: Some(bay_id),
            description: "Breaker closed (sync OK)".to_string(),
            severity: EventSeverity::Information,
        };
        self.event_log.push(event);
        Ok(())
    }

    /// Generate a SCADA telemetry report at the given timestamp.
    pub fn generate_report(&self, timestamp_s: f64) -> ScadaReport {
        let mut total_load_mw = 0.0_f64;
        let mut total_reactive_mvar = 0.0_f64;
        let mut n_alarms = 0usize;
        let mut n_trips = 0usize;
        let mut bay_statuses = Vec::with_capacity(self.bays.len());

        for bay in &self.bays {
            total_load_mw += bay.measured_power_mw;
            total_reactive_mvar += bay.measured_reactive_mvar;
            match &bay.protection_status {
                ProtectionStatus::AlarmActive(_) => n_alarms += 1,
                ProtectionStatus::ProtectionTripped { .. } => n_trips += 1,
                _ => {}
            }
            bay_statuses.push((bay.id, bay.breaker_status.is_closed, bay.measured_power_mw));
        }

        ScadaReport {
            timestamp_s,
            substation: self.config.name.clone(),
            total_load_mw,
            total_reactive_mvar,
            busbar_voltage_pu: self.estimate_busbar_voltage(),
            n_alarms,
            n_trips,
            bay_statuses,
        }
    }

    /// Estimate busbar voltage \[pu\] from measurement VT bays.
    ///
    /// Returns 1.0 (nominal) if no VT bays are present.
    pub fn estimate_busbar_voltage(&self) -> f64 {
        // Use VT bays if available; fall back to a simple power-weighted average
        // based on the assumption that higher power injection → closer to nominal.
        let vt_bays: Vec<&Bay> = self
            .bays
            .iter()
            .filter(|b| b.bay_type == BayType::MeasurementVt)
            .collect();

        if !vt_bays.is_empty() {
            // Average over VT measurements (stored in measured_power_mw as proxy for V [pu])
            let sum: f64 = vt_bays.iter().map(|b| b.measured_power_mw).sum();
            return sum / vt_bays.len() as f64;
        }

        // Simplified: no VT — return nominal voltage
        1.0
    }

    /// Identify bays requiring maintenance.
    ///
    /// Flags bays where:
    /// - Contact wear > 80 \[%\]
    /// - SF6 pressure < 4.5 \[bar\]
    /// - Trip count > 50
    ///
    /// Returns `(bay_id, reason)` pairs.
    pub fn check_maintenance_needs(&self) -> Vec<(usize, String)> {
        let mut needs = Vec::new();
        for bay in &self.bays {
            let bs = &bay.breaker_status;
            if bs.contact_wear_pct > 80.0 {
                needs.push((
                    bay.id,
                    format!(
                        "Contact wear {:.1}% exceeds 80% threshold",
                        bs.contact_wear_pct
                    ),
                ));
            }
            if bs.sf6_pressure_bar < 4.5 {
                needs.push((
                    bay.id,
                    format!(
                        "SF6 pressure {:.2} bar below 4.5 bar minimum",
                        bs.sf6_pressure_bar
                    ),
                ));
            }
            if bs.trip_count > 50 {
                needs.push((
                    bay.id,
                    format!(
                        "Trip count {} exceeds 50 — contact inspection due",
                        bs.trip_count
                    ),
                ));
            }
        }
        needs
    }

    /// Read-only access to the event log.
    pub fn event_log(&self) -> &[SubstationEvent] {
        &self.event_log
    }

    /// Read-only access to all bays.
    pub fn bays(&self) -> &[Bay] {
        &self.bays
    }

    /// Substation configuration.
    pub fn config(&self) -> &SubstationConfig {
        &self.config
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_sas() -> SubstationAutomationSystem {
        let config = SubstationConfig {
            name: "TestSS".into(),
            voltage_kv: 110.0,
            n_bays: 3,
            busbar_configuration: BusbarConfig::SingleBus,
            ied_polling_interval_ms: 100.0,
            scada_report_interval_s: 10.0,
        };
        let mut sas = SubstationAutomationSystem::new(config);

        let mut bay0 = Bay::new(0, "Line Bay 1", BayType::FeederLine);
        bay0.measured_power_mw = 20.0;
        bay0.measured_reactive_mvar = 5.0;
        bay0.rated_current_a = 500.0;
        sas.add_bay(bay0);

        let mut bay1 = Bay::new(1, "Transformer Bay", BayType::Transformer);
        bay1.measured_power_mw = 30.0;
        bay1.measured_reactive_mvar = 10.0;
        bay1.rated_current_a = 800.0;
        sas.add_bay(bay1);

        let mut bay2 = Bay::new(2, "Generator Bay", BayType::Generator);
        bay2.measured_power_mw = 50.0;
        bay2.measured_reactive_mvar = 15.0;
        bay2.rated_current_a = 1000.0;
        sas.add_bay(bay2);

        sas
    }

    /// Test 1: Trip breaker — status changes and event is logged.
    #[test]
    fn test_trip_breaker() {
        let mut sas = make_sas();

        // Bay 0 breaker starts closed
        assert!(sas.bays()[0].breaker_status.is_closed);

        sas.trip_breaker(0, "overcurrent", 100.0)
            .expect("trip must succeed");

        let bay = &sas.bays()[0];
        assert!(
            !bay.breaker_status.is_closed,
            "breaker must be open after trip"
        );
        assert_eq!(bay.breaker_status.trip_count, 1);
        assert_eq!(bay.breaker_status.last_operation, Some(100.0));

        match &bay.protection_status {
            ProtectionStatus::ProtectionTripped { cause, .. } => {
                assert_eq!(cause, "overcurrent");
            }
            other => panic!("expected ProtectionTripped, got {:?}", other),
        }

        let has_prot_event = sas
            .event_log()
            .iter()
            .any(|e| e.event_type == EventType::Protection && e.bay_id == Some(0));
        assert!(has_prot_event, "protection event must be logged");
    }

    /// Test 2: Close breaker with sync_ok=true succeeds.
    #[test]
    fn test_close_breaker_sync_ok() {
        let mut sas = make_sas();

        // First trip bay 0 so it's open
        sas.trip_breaker(0, "test trip", 50.0)
            .expect("trip must succeed");
        assert!(!sas.bays()[0].breaker_status.is_closed);

        // Close with sync OK
        sas.close_breaker(0, true, 60.0)
            .expect("close must succeed with sync OK");

        let bay = &sas.bays()[0];
        assert!(bay.breaker_status.is_closed, "breaker must be closed");
        assert_eq!(bay.breaker_status.last_operation, Some(60.0));

        match &bay.protection_status {
            ProtectionStatus::Normal => {}
            other => panic!("expected Normal after close, got {:?}", other),
        }
    }

    /// Test 3: Close without sync is rejected.
    #[test]
    fn test_close_breaker_no_sync_rejected() {
        let mut sas = make_sas();

        // Trip bay 0 first
        sas.trip_breaker(0, "test", 10.0).expect("trip OK");
        assert!(!sas.bays()[0].breaker_status.is_closed);

        // Attempt close without sync
        let result = sas.close_breaker(0, false, 20.0);
        assert!(
            matches!(result, Err(SasError::SyncCheckFailed)),
            "close without sync must be rejected"
        );

        // Breaker should still be open
        assert!(
            !sas.bays()[0].breaker_status.is_closed,
            "breaker must remain open"
        );
    }

    /// Test 4: SCADA report computes correct totals.
    #[test]
    fn test_scada_report_totals() {
        let mut sas = make_sas();

        // Trip bay 1 to create a trip count
        sas.trip_breaker(1, "test", 0.0).expect("trip OK");

        let report = sas.generate_report(200.0);

        assert_eq!(report.substation, "TestSS");
        assert!(
            (report.total_load_mw - 100.0).abs() < 1e-9,
            "total load must be 20+30+50=100 MW, got {}",
            report.total_load_mw
        );
        assert!(
            (report.total_reactive_mvar - 30.0).abs() < 1e-9,
            "total reactive must be 5+10+15=30 MVAr"
        );
        assert_eq!(report.n_trips, 1, "one trip should be counted");
        assert_eq!(report.bay_statuses.len(), 3);

        // Bay 1 should be open after trip
        let bay1_status = report.bay_statuses.iter().find(|(id, _, _)| *id == 1);
        assert!(bay1_status.is_some());
        assert!(!bay1_status.unwrap().1, "bay 1 breaker should be open");
    }

    /// Test 5: High wear is flagged in maintenance check.
    #[test]
    fn test_maintenance_high_wear_flagged() {
        let mut sas = make_sas();

        // Set bay 0 to high wear
        sas.bays[0].breaker_status.contact_wear_pct = 85.0;

        let needs = sas.check_maintenance_needs();
        let has_wear = needs
            .iter()
            .any(|(id, reason)| *id == 0 && reason.contains("wear"));
        assert!(has_wear, "high wear must be flagged for bay 0");
    }

    /// Test 6: Low SF6 pressure flagged.
    #[test]
    fn test_maintenance_low_sf6_flagged() {
        let mut sas = make_sas();

        // Set bay 2 to low SF6 pressure
        sas.bays[2].breaker_status.sf6_pressure_bar = 3.8;

        let needs = sas.check_maintenance_needs();
        let has_sf6 = needs
            .iter()
            .any(|(id, reason)| *id == 2 && reason.contains("SF6"));
        assert!(has_sf6, "low SF6 pressure must be flagged for bay 2");
    }

    /// Test 7: IED overcurrent at 3× rated triggers auto-trip.
    #[test]
    fn test_ied_critical_overcurrent_auto_trip() {
        let mut sas = make_sas();
        // Bay 0 rated_current_a = 500 A → 3× = 1500 A
        let events = sas.process_ied_data(0, 1600.0, 300.0).expect("process OK");

        // Must have a Critical protection event
        let has_critical = events.iter().any(|e| e.severity == EventSeverity::Critical);
        assert!(has_critical, "critical event must be generated at 3× rated");

        // Breaker must now be open
        assert!(
            !sas.bays()[0].breaker_status.is_closed,
            "breaker auto-tripped"
        );
        assert_eq!(sas.bays()[0].breaker_status.trip_count, 1);
    }

    /// Test 8: Bay not found returns error.
    #[test]
    fn test_bay_not_found_error() {
        let mut sas = make_sas();
        let result = sas.trip_breaker(99, "test", 0.0);
        assert!(
            matches!(result, Err(SasError::BayNotFound(99))),
            "must return BayNotFound for unknown bay"
        );
    }
}
