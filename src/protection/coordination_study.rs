//! Protection System Coordination Study.
//!
//! Provides a comprehensive tool for analysing the time–current coordination of
//! overcurrent protection schemes.  It supports all five standard IDMT curves
//! (IEC SI/VI/EI/LTI and ANSI CO-8), zone-based topology, automatic TDS tuning,
//! and generates a detailed [`CoordinationReport`] with recommended relay
//! adjustments.
//!
//! # Reference
//! - IEC 60255-151:2009 — Functional requirements for protection relays
//! - IEEE Std C37.112-1996 — Inverse-time characteristic equations
use serde::{Deserialize, Serialize};
use thiserror::Error;

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------

/// Errors produced by the coordination study engine.
#[derive(Debug, Error)]
pub enum StudyError {
    /// No zones have been added.
    #[error("coordination study has no protection zones")]
    NoZones,
    /// A zone references a relay that has not been added.
    #[error("zone {zone_id} references unknown relay {relay_id}")]
    UnknownRelay { zone_id: usize, relay_id: usize },
    /// Upstream relay is not registered.
    #[error("upstream relay {0} not found in relay list")]
    UnknownUpstreamRelay(usize),
    /// Fault current is zero or negative.
    #[error("fault current must be positive, got {0}")]
    InvalidFaultCurrent(f64),
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Global settings for one coordination study.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CoordinationStudyConfig {
    /// Nominal system voltage \[kV\]
    pub system_voltage_kv: f64,
    /// System base \[MVA\]
    pub base_mva: f64,
    /// Maximum fault current in the study \[kA\]
    pub max_fault_current_ka: f64,
    /// Minimum fault current checked for coordination \[kA\]
    pub min_fault_current_ka: f64,
    /// Required time grading margin between primary and backup \[ms\]
    pub grading_margin_ms: f64,
    /// Percentage margin above backup relay trip time for instantaneous setting \[%\]
    pub instantaneous_margin_pct: f64,
}

impl Default for CoordinationStudyConfig {
    fn default() -> Self {
        Self {
            system_voltage_kv: 11.0,
            base_mva: 100.0,
            max_fault_current_ka: 10.0,
            min_fault_current_ka: 0.3,
            grading_margin_ms: 300.0,
            instantaneous_margin_pct: 10.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Zone and element types
// ---------------------------------------------------------------------------

/// Classification of a protection zone in the hierarchy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ZoneType {
    /// Primary (fastest) protection zone.
    Main,
    /// First backup zone.
    Backup,
    /// Second backup zone.
    Tertiary,
}

/// The power system element protected by a zone.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ProtectedElement {
    /// Overhead or cable line.
    Line {
        /// Line length \[km\]
        length_km: f64,
        /// Line positive-sequence impedance \[pu\]
        impedance_pu: f64,
    },
    /// Power transformer.
    Transformer {
        /// Transformer rating \[MVA\]
        mva: f64,
        /// Leakage impedance \[%\]
        impedance_pct: f64,
    },
    /// Busbar.
    Bus {
        /// Busbar identifier
        busbar_id: usize,
    },
    /// Synchronous generator.
    Generator {
        /// Generator rating \[MVA\]
        mva: f64,
    },
}

/// One protection zone in the coordination hierarchy.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProtectionZone {
    /// Zone identifier.
    pub id: usize,
    /// Relay responsible for this zone.
    pub relay_id: usize,
    /// Zone classification (Main / Backup / Tertiary).
    pub zone_type: ZoneType,
    /// Protected element.
    pub protected_element: ProtectedElement,
    /// Upstream backup relay ID (None for the head zone).
    pub upstream_relay: Option<usize>,
    /// IDs of downstream (primary) relays this zone backs up.
    pub downstream_relays: Vec<usize>,
}

// ---------------------------------------------------------------------------
// Relay characteristics
// ---------------------------------------------------------------------------

/// Overcurrent relay type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RelayType {
    /// IDMT overcurrent relay.
    OvercurrentInverse,
    /// Definite-time overcurrent relay.
    OvercurrentDefiniteTime,
    /// Impedance / distance relay.
    Distance,
    /// Differential (percentage / high-restraint) relay.
    Differential,
    /// High-impedance busbar relay.
    HighImpedance,
}

/// Standard IDMT curve families.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum IdmtCurve {
    /// IEC Standard Inverse: `t = 0.14·TDS / (M^0.02 − 1)`
    StandardInverse,
    /// IEC Very Inverse: `t = 13.5·TDS / (M − 1)`
    VeryInverse,
    /// IEC Extremely Inverse: `t = 80·TDS / (M² − 1)`
    ExtremelyInverse,
    /// IEC Long-Time Inverse: `t = 120·TDS / (M − 1)`
    LongTimeInverse,
    /// ANSI CO-8 (US CO Inverse): `t = (5.95·TDS) / (M^2 − 1) + 0.18·TDS`
    UsCoInverse,
}

/// Complete relay settings for one relay in the study.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RelayCharacteristics {
    /// Relay identifier.
    pub relay_id: usize,
    /// Relay technology type.
    pub relay_type: RelayType,
    /// Time dial setting (TDS / TMS).
    pub time_dial: f64,
    /// Pickup current (primary side) \[A\]
    pub pickup_current_a: f64,
    /// Instantaneous overcurrent pickup \[A\] (0 = disabled)
    pub instantaneous_pickup_a: f64,
    /// Current transformer ratio (primary : secondary).
    pub ct_ratio: f64,
    /// IDMT curve family.
    pub curve_type: IdmtCurve,
}

// ---------------------------------------------------------------------------
// Results
// ---------------------------------------------------------------------------

/// Coordination check between one upstream–downstream relay pair.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CoordinationCheck {
    /// Upstream (backup) relay ID.
    pub upstream_relay: usize,
    /// Downstream (primary) relay ID.
    pub downstream_relay: usize,
    /// Fault current used for the check \[A\]
    pub fault_current_a: f64,
    /// Upstream relay trip time \[ms\]
    pub upstream_trip_time_ms: f64,
    /// Downstream relay trip time \[ms\]
    pub downstream_trip_time_ms: f64,
    /// Actual grading margin (upstream − downstream) \[ms\]
    pub margin_ms: f64,
    /// True when margin ≥ required grading margin.
    pub coordinated: bool,
    /// How much the margin is violated \[ms\] (None when coordinated).
    pub margin_violation: Option<f64>,
}

/// Full report produced by [`CoordinationStudy::run`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CoordinationReport {
    /// All upstream–downstream coordination checks.
    pub checks: Vec<CoordinationCheck>,
    /// True only when every check passes.
    pub all_coordinated: bool,
    /// Number of checks with coordination violations.
    pub total_violations: usize,
    /// Magnitude of the worst violation \[ms\] (0 when no violations).
    pub worst_violation_ms: f64,
    /// Tuning recommendations: `(relay_id, new_TDS, new_pickup_a)`.
    pub recommended_adjustments: Vec<(usize, f64, f64)>,
}

// ---------------------------------------------------------------------------
// Study engine
// ---------------------------------------------------------------------------

/// Coordination study engine.
///
/// Add zones with [`add_zone`](Self::add_zone) and relay settings with
/// [`add_relay`](Self::add_relay), then call [`run`](Self::run).
pub struct CoordinationStudy {
    config: CoordinationStudyConfig,
    zones: Vec<ProtectionZone>,
    relays: Vec<RelayCharacteristics>,
}

impl CoordinationStudy {
    /// Create a new study with the given configuration.
    pub fn new(config: CoordinationStudyConfig) -> Self {
        Self {
            config,
            zones: Vec::new(),
            relays: Vec::new(),
        }
    }

    /// Add a protection zone to the study.
    pub fn add_zone(&mut self, zone: ProtectionZone) {
        self.zones.push(zone);
    }

    /// Register relay characteristics.
    pub fn add_relay(&mut self, relay: RelayCharacteristics) {
        self.relays.push(relay);
    }

    // -----------------------------------------------------------------------
    // IDMT trip-time computation
    // -----------------------------------------------------------------------

    /// Compute IDMT operating time \[s\] for a given curve family.
    ///
    /// # Arguments
    /// * `curve` — IDMT curve family
    /// * `tds`   — time dial setting (dimensionless)
    /// * `m`     — current multiple = I_fault / I_pickup (must be > 1)
    ///
    /// Returns `f64::INFINITY` when `m ≤ 1` (relay does not operate).
    pub fn idmt_trip_time(curve: &IdmtCurve, tds: f64, m: f64) -> f64 {
        if m <= 1.0 {
            return f64::INFINITY;
        }
        match curve {
            IdmtCurve::StandardInverse => {
                // IEC SI: t = 0.14 * TDS / (M^0.02 - 1)
                tds * 0.14 / (m.powf(0.02) - 1.0)
            }
            IdmtCurve::VeryInverse => {
                // IEC VI: t = 13.5 * TDS / (M - 1)
                tds * 13.5 / (m - 1.0)
            }
            IdmtCurve::ExtremelyInverse => {
                // IEC EI: t = 80 * TDS / (M^2 - 1)
                tds * 80.0 / (m * m - 1.0)
            }
            IdmtCurve::LongTimeInverse => {
                // IEC LTI: t = 120 * TDS / (M - 1)
                tds * 120.0 / (m - 1.0)
            }
            IdmtCurve::UsCoInverse => {
                // ANSI CO-8: t = 5.95 * TDS / (M^2 - 1) + 0.18 * TDS
                tds * 5.95 / (m * m - 1.0) + 0.18 * tds
            }
        }
    }

    /// Compute trip time \[s\] for relay `r` at `fault_current_a`.
    ///
    /// Returns `f64::INFINITY` if the relay does not pick up.
    fn relay_trip_time_s(r: &RelayCharacteristics, fault_current_a: f64) -> f64 {
        // Check instantaneous first.
        if r.instantaneous_pickup_a > 0.0 && fault_current_a >= r.instantaneous_pickup_a {
            return 0.02; // nominal 20 ms instantaneous operate time
        }
        let m = fault_current_a / r.pickup_current_a;
        Self::idmt_trip_time(&r.curve_type, r.time_dial, m)
    }

    // -----------------------------------------------------------------------
    // Auto-tune TDS
    // -----------------------------------------------------------------------

    /// Compute the minimum TDS for `relay_id` such that its trip time is at
    /// least `required_margin_ms` ms above the downstream relay's trip time at
    /// `fault_current_a`.
    ///
    /// If no downstream relay is associated (head relay), returns the current TDS.
    pub fn auto_tune_tds(&self, relay_id: usize, fault_current_a: f64) -> f64 {
        // Find the relay.
        let Some(relay) = self.relays.iter().find(|r| r.relay_id == relay_id) else {
            return 1.0; // default
        };

        // Find all downstream relays through zones.
        let downstream_ids: Vec<usize> = self
            .zones
            .iter()
            .filter(|z| z.upstream_relay == Some(relay_id))
            .flat_map(|z| z.downstream_relays.iter().copied())
            .collect();

        if downstream_ids.is_empty() {
            return relay.time_dial;
        }

        // Find the latest downstream trip time.
        let margin_s = self.config.grading_margin_ms / 1000.0;
        let mut max_downstream_s = 0.0_f64;
        for &ds_id in &downstream_ids {
            if let Some(ds_relay) = self.relays.iter().find(|r| r.relay_id == ds_id) {
                let t = Self::relay_trip_time_s(ds_relay, fault_current_a);
                if t.is_finite() {
                    max_downstream_s = max_downstream_s.max(t);
                }
            }
        }

        // Required upstream trip time.
        let required_t = max_downstream_s + margin_s;

        // Invert the IDMT equation to find TDS.
        let m = fault_current_a / relay.pickup_current_a;
        if m <= 1.0 {
            return relay.time_dial;
        }
        let denominator = match relay.curve_type {
            IdmtCurve::StandardInverse => 0.14 / (m.powf(0.02) - 1.0),
            IdmtCurve::VeryInverse => 13.5 / (m - 1.0),
            IdmtCurve::ExtremelyInverse => 80.0 / (m * m - 1.0),
            IdmtCurve::LongTimeInverse => 120.0 / (m - 1.0),
            IdmtCurve::UsCoInverse => 5.95 / (m * m - 1.0) + 0.18,
        };
        if denominator <= 0.0 {
            return relay.time_dial;
        }
        // TDS = required_t / denominator
        (required_t / denominator).max(0.05)
    }

    // -----------------------------------------------------------------------
    // Core study run
    // -----------------------------------------------------------------------

    /// Run the full coordination study and return a [`CoordinationReport`].
    pub fn run(&self) -> Result<CoordinationReport, StudyError> {
        if self.zones.is_empty() {
            return Err(StudyError::NoZones);
        }

        // Validate that all zones reference registered relays.
        for zone in &self.zones {
            if !self.relays.iter().any(|r| r.relay_id == zone.relay_id) {
                return Err(StudyError::UnknownRelay {
                    zone_id: zone.id,
                    relay_id: zone.relay_id,
                });
            }
        }

        let max_i_a = self.config.max_fault_current_ka * 1000.0;
        let min_i_a = self.config.min_fault_current_ka * 1000.0;
        let margin_ms = self.config.grading_margin_ms;

        // Test at max and min fault currents; add midpoint for thoroughness.
        let test_currents = [max_i_a, (max_i_a + min_i_a) / 2.0, min_i_a];

        let mut checks: Vec<CoordinationCheck> = Vec::new();

        // For each zone, check coordination between the zone relay and its
        // upstream backup relay.
        for zone in &self.zones {
            let Some(upstream_relay_id) = zone.upstream_relay else {
                continue; // head zone — no upstream to coordinate with
            };

            let Some(upstream_relay) = self.relays.iter().find(|r| r.relay_id == upstream_relay_id)
            else {
                return Err(StudyError::UnknownUpstreamRelay(upstream_relay_id));
            };

            let Some(downstream_relay) = self.relays.iter().find(|r| r.relay_id == zone.relay_id)
            else {
                return Err(StudyError::UnknownRelay {
                    zone_id: zone.id,
                    relay_id: zone.relay_id,
                });
            };

            for &i_fault in &test_currents {
                let t_down_s = Self::relay_trip_time_s(downstream_relay, i_fault);
                let t_up_s = Self::relay_trip_time_s(upstream_relay, i_fault);

                // Only check when the downstream relay actually operates.
                if !t_down_s.is_finite() {
                    continue;
                }

                let t_down_ms = t_down_s * 1000.0;
                let t_up_ms = if t_up_s.is_finite() {
                    t_up_s * 1000.0
                } else {
                    f64::INFINITY
                };

                let actual_margin = t_up_ms - t_down_ms;
                let coordinated = actual_margin >= margin_ms;
                let margin_violation = if coordinated {
                    None
                } else {
                    Some(margin_ms - actual_margin)
                };

                checks.push(CoordinationCheck {
                    upstream_relay: upstream_relay_id,
                    downstream_relay: zone.relay_id,
                    fault_current_a: i_fault,
                    upstream_trip_time_ms: t_up_ms,
                    downstream_trip_time_ms: t_down_ms,
                    margin_ms: actual_margin,
                    coordinated,
                    margin_violation,
                });
            }
        }

        let total_violations = checks.iter().filter(|c| !c.coordinated).count();
        let worst_violation_ms = checks
            .iter()
            .filter_map(|c| c.margin_violation)
            .fold(0.0_f64, f64::max);
        let all_coordinated = total_violations == 0;

        // Generate recommendations for relays with violations.
        let mut recommended_adjustments: Vec<(usize, f64, f64)> = Vec::new();
        let violated_relay_ids: Vec<usize> = {
            let mut ids: Vec<usize> = checks
                .iter()
                .filter(|c| !c.coordinated)
                .map(|c| c.upstream_relay)
                .collect();
            ids.sort_unstable();
            ids.dedup();
            ids
        };

        for relay_id in violated_relay_ids {
            let new_tds = self.auto_tune_tds(relay_id, max_i_a);
            // Keep existing pickup; only TDS is adjusted.
            let existing_pickup = self
                .relays
                .iter()
                .find(|r| r.relay_id == relay_id)
                .map(|r| r.pickup_current_a)
                .unwrap_or(0.0);
            recommended_adjustments.push((relay_id, new_tds, existing_pickup));
        }

        Ok(CoordinationReport {
            checks,
            all_coordinated,
            total_violations,
            worst_violation_ms,
            recommended_adjustments,
        })
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: standard SI relay.
    fn si_relay(id: usize, pickup_a: f64, tds: f64) -> RelayCharacteristics {
        RelayCharacteristics {
            relay_id: id,
            relay_type: RelayType::OvercurrentInverse,
            time_dial: tds,
            pickup_current_a: pickup_a,
            instantaneous_pickup_a: 0.0,
            ct_ratio: 200.0,
            curve_type: IdmtCurve::StandardInverse,
        }
    }

    // ------------------------------------------------------------------
    // 1. IDMT curve correctness at M = 5
    // ------------------------------------------------------------------
    #[test]
    fn test_idmt_si_at_m5() {
        // IEC SI: t = 0.14 * TDS / (5^0.02 - 1)
        let tds = 1.0;
        let m = 5.0;
        let expected = 0.14 / (5.0_f64.powf(0.02) - 1.0);
        let got = CoordinationStudy::idmt_trip_time(&IdmtCurve::StandardInverse, tds, m);
        assert!(
            (got - expected).abs() < 1e-9,
            "SI curve mismatch: {got:.6} vs {expected:.6}"
        );
    }

    #[test]
    fn test_idmt_vi_at_m5() {
        // IEC VI: t = 13.5 / (5 - 1) = 3.375 s
        let got = CoordinationStudy::idmt_trip_time(&IdmtCurve::VeryInverse, 1.0, 5.0);
        let expected = 13.5 / 4.0;
        assert!(
            (got - expected).abs() < 1e-9,
            "VI: {got:.6} vs {expected:.6}"
        );
    }

    #[test]
    fn test_idmt_ei_at_m5() {
        // IEC EI: t = 80 / (25 - 1) = 80/24 ≈ 3.333 s
        let got = CoordinationStudy::idmt_trip_time(&IdmtCurve::ExtremelyInverse, 1.0, 5.0);
        let expected = 80.0 / 24.0;
        assert!(
            (got - expected).abs() < 1e-9,
            "EI: {got:.6} vs {expected:.6}"
        );
    }

    #[test]
    fn test_idmt_lti_at_m5() {
        // IEC LTI: t = 120 / (5 - 1) = 30 s
        let got = CoordinationStudy::idmt_trip_time(&IdmtCurve::LongTimeInverse, 1.0, 5.0);
        assert!((got - 30.0).abs() < 1e-9, "LTI: {got:.6}");
    }

    #[test]
    fn test_idmt_co_at_m5() {
        // ANSI CO-8: t = 5.95/(25-1) + 0.18 = 5.95/24 + 0.18
        let got = CoordinationStudy::idmt_trip_time(&IdmtCurve::UsCoInverse, 1.0, 5.0);
        let expected = 5.95 / 24.0 + 0.18;
        assert!(
            (got - expected).abs() < 1e-9,
            "CO-8: {got:.6} vs {expected:.6}"
        );
    }

    // ------------------------------------------------------------------
    // 2. Coordinated system — all checks pass
    // ------------------------------------------------------------------
    #[test]
    fn test_coordinated_system_passes() {
        let config = CoordinationStudyConfig {
            system_voltage_kv: 11.0,
            base_mva: 100.0,
            max_fault_current_ka: 5.0,
            min_fault_current_ka: 0.5,
            grading_margin_ms: 300.0,
            instantaneous_margin_pct: 10.0,
        };
        let mut study = CoordinationStudy::new(config);

        // Downstream relay: TDS=0.2, pickup=400 A
        study.add_relay(si_relay(1, 400.0, 0.2));
        // Upstream relay: TDS=0.6, pickup=300 A — deliberately slower
        study.add_relay(si_relay(2, 300.0, 0.6));

        study.add_zone(ProtectionZone {
            id: 1,
            relay_id: 1,
            zone_type: ZoneType::Main,
            protected_element: ProtectedElement::Line {
                length_km: 5.0,
                impedance_pu: 0.1,
            },
            upstream_relay: Some(2),
            downstream_relays: vec![],
        });

        let report = study.run().expect("study should succeed");
        assert!(
            report.all_coordinated,
            "Expected all coordinated; violations={:?}",
            report.total_violations
        );
        assert_eq!(report.total_violations, 0);
    }

    // ------------------------------------------------------------------
    // 3. Violation — grading margin not met
    // ------------------------------------------------------------------
    #[test]
    fn test_violation_detected() {
        let config = CoordinationStudyConfig {
            system_voltage_kv: 11.0,
            base_mva: 100.0,
            max_fault_current_ka: 5.0,
            min_fault_current_ka: 0.5,
            grading_margin_ms: 300.0,
            instantaneous_margin_pct: 10.0,
        };
        let mut study = CoordinationStudy::new(config);

        // Both relays have same TDS → zero grading margin → violation.
        study.add_relay(si_relay(1, 400.0, 0.2));
        study.add_relay(si_relay(2, 400.0, 0.2)); // same settings as downstream

        study.add_zone(ProtectionZone {
            id: 1,
            relay_id: 1,
            zone_type: ZoneType::Main,
            protected_element: ProtectedElement::Line {
                length_km: 5.0,
                impedance_pu: 0.1,
            },
            upstream_relay: Some(2),
            downstream_relays: vec![],
        });

        let report = study.run().expect("study should run");
        assert!(
            !report.all_coordinated,
            "Expected violations when both relays identical"
        );
        assert!(report.total_violations > 0);
        assert!(report.worst_violation_ms > 0.0);
    }

    // ------------------------------------------------------------------
    // 4. Auto-tune TDS achieves coordination
    // ------------------------------------------------------------------
    #[test]
    fn test_auto_tune_achieves_coordination() {
        let config = CoordinationStudyConfig {
            system_voltage_kv: 11.0,
            base_mva: 100.0,
            max_fault_current_ka: 5.0,
            min_fault_current_ka: 0.5,
            grading_margin_ms: 300.0,
            instantaneous_margin_pct: 10.0,
        };
        let mut study = CoordinationStudy::new(config);

        let ds_relay = si_relay(1, 400.0, 0.2);
        study.add_relay(ds_relay.clone());
        study.add_relay(si_relay(2, 300.0, 0.2)); // initially under-tuned upstream

        study.add_zone(ProtectionZone {
            id: 1,
            relay_id: 1,
            zone_type: ZoneType::Main,
            protected_element: ProtectedElement::Line {
                length_km: 5.0,
                impedance_pu: 0.1,
            },
            upstream_relay: Some(2),
            downstream_relays: vec![1],
        });

        let fault_a = 5000.0;
        let new_tds = study.auto_tune_tds(2, fault_a);

        // Verify: trip time with new_tds >= trip time of downstream + margin
        let m_up = fault_a / 300.0;
        let t_up = CoordinationStudy::idmt_trip_time(&IdmtCurve::StandardInverse, new_tds, m_up);

        let m_down = fault_a / 400.0;
        let t_down = CoordinationStudy::idmt_trip_time(
            &IdmtCurve::StandardInverse,
            ds_relay.time_dial,
            m_down,
        );

        let actual_margin_ms = (t_up - t_down) * 1000.0;
        assert!(
            actual_margin_ms >= 299.0, // allow 1 ms tolerance
            "Auto-tuned margin {actual_margin_ms:.2} ms < 300 ms required"
        );
    }

    // ------------------------------------------------------------------
    // 5. Instantaneous trip is faster than IDMT at high currents
    // ------------------------------------------------------------------
    #[test]
    fn test_instantaneous_faster_than_idmt_at_high_current() {
        let relay_idmt = RelayCharacteristics {
            relay_id: 10,
            relay_type: RelayType::OvercurrentInverse,
            time_dial: 1.0,
            pickup_current_a: 100.0,
            instantaneous_pickup_a: 0.0, // IDMT only
            ct_ratio: 100.0,
            curve_type: IdmtCurve::VeryInverse,
        };
        let relay_inst = RelayCharacteristics {
            relay_id: 11,
            relay_type: RelayType::OvercurrentInverse,
            time_dial: 1.0,
            pickup_current_a: 100.0,
            instantaneous_pickup_a: 1500.0, // kicks in at 1500 A
            ct_ratio: 100.0,
            curve_type: IdmtCurve::VeryInverse,
        };

        let i_high = 3000.0_f64;
        let t_idmt = CoordinationStudy::relay_trip_time_s(&relay_idmt, i_high);
        let t_inst = CoordinationStudy::relay_trip_time_s(&relay_inst, i_high);

        assert!(
            t_inst < t_idmt,
            "Instantaneous ({t_inst:.4} s) should be faster than IDMT ({t_idmt:.4} s)"
        );
        // Instantaneous should be ≈ 20 ms
        assert!(
            (t_inst - 0.02).abs() < 1e-6,
            "Instantaneous trip time should be 0.02 s, got {t_inst}"
        );
    }

    // ------------------------------------------------------------------
    // 6. No zones → StudyError::NoZones
    // ------------------------------------------------------------------
    #[test]
    fn test_empty_study_returns_error() {
        let study = CoordinationStudy::new(CoordinationStudyConfig::default());
        let err = study.run().unwrap_err();
        assert!(
            matches!(err, StudyError::NoZones),
            "Expected NoZones, got {err}"
        );
    }
}
