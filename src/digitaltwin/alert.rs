/// Real-time alert engine for the grid digital twin.
///
/// Monitors `TwinState` against configurable thresholds and generates
/// `TwinAlert` objects at severity levels Info → Warning → Critical → Emergency.
/// Includes 60-second deduplication and per-element suppression.
use crate::digitaltwin::twin::TwinState;
use serde::{Deserialize, Serialize};

// ─────────────────────────────────────────────────────────────────────────────
// Threshold configuration
// ─────────────────────────────────────────────────────────────────────────────

/// Per-quantity alert thresholds (all values in engineering units as noted).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlertThresholds {
    /// Voltage high warning \[pu\] (default 1.05)
    pub voltage_high_pu: f64,
    /// Voltage low warning \[pu\] (default 0.95)
    pub voltage_low_pu: f64,
    /// Voltage critical high \[pu\] (default 1.10)
    pub voltage_critical_high: f64,
    /// Voltage critical low \[pu\] (default 0.90)
    pub voltage_critical_low: f64,
    /// Branch thermal loading warning [%] (default 80)
    pub branch_loading_warning_pct: f64,
    /// Branch thermal loading critical [%] (default 100)
    pub branch_loading_critical_pct: f64,
    /// Frequency high warning \[Hz\] (default 50.2)
    pub frequency_high_hz: f64,
    /// Frequency low warning \[Hz\] (default 49.8)
    pub frequency_low_hz: f64,
    /// Frequency critical high \[Hz\] (default 50.5)
    pub frequency_critical_high: f64,
    /// Frequency critical low \[Hz\] (default 49.5)
    pub frequency_critical_low: f64,
    /// Rate-of-change-of-frequency (RoCoF) critical threshold [Hz/s] (default 1.0)
    pub rocof_critical: f64,
}

impl Default for AlertThresholds {
    fn default() -> Self {
        Self {
            voltage_high_pu: 1.05,
            voltage_low_pu: 0.95,
            voltage_critical_high: 1.10,
            voltage_critical_low: 0.90,
            branch_loading_warning_pct: 80.0,
            branch_loading_critical_pct: 100.0,
            frequency_high_hz: 50.2,
            frequency_low_hz: 49.8,
            frequency_critical_high: 50.5,
            frequency_critical_low: 49.5,
            rocof_critical: 1.0,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Alert types
// ─────────────────────────────────────────────────────────────────────────────

/// Severity level — ordering matches PartialOrd: Info < Warning < Critical < Emergency.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum AlertSeverity {
    Info,
    Warning,
    Critical,
    Emergency,
}

/// High-level category for routing / display.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AlertCategory {
    Voltage,
    Thermal,
    Frequency,
    Stability,
    DataQuality,
    Topology,
}

/// A single alert instance produced by `AlertEngine::check_state`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TwinAlert {
    /// Globally unique monotonic alert ID.
    pub alert_id: u64,
    /// Timestamp of the state snapshot that triggered the alert [µs since epoch].
    pub timestamp_us: i64,
    /// Severity level.
    pub severity: AlertSeverity,
    /// Functional category.
    pub category: AlertCategory,
    /// Human-readable description.
    pub description: String,
    /// Bus or branch index that triggered the alert (None = system-wide).
    pub affected_element: Option<usize>,
    /// Actual measured value at alert time.
    pub value: f64,
    /// Threshold that was violated.
    pub threshold: f64,
    /// Suggested operator action.
    pub recommended_action: String,
    /// Whether an operator has acknowledged this alert.
    pub acknowledged: bool,
}

// ─────────────────────────────────────────────────────────────────────────────
// Alert engine
// ─────────────────────────────────────────────────────────────────────────────

/// Last-triggered record used for 60-second deduplication.
#[derive(Debug, Clone)]
struct AlertRecord {
    category: AlertCategory,
    element: Option<usize>,
    last_triggered_us: i64,
}

/// Stateful alert engine.  Maintains active alert list, deduplication state,
/// and per-element suppression.
pub struct AlertEngine {
    /// Threshold configuration.
    pub thresholds: AlertThresholds,
    /// Monotonically increasing counter for `alert_id`.
    pub alert_counter: u64,
    /// Currently active (non-cleared) alerts.
    pub active_alerts: Vec<TwinAlert>,
    /// (category, element) pairs that are permanently suppressed.
    pub suppression_list: Vec<(AlertCategory, usize)>,
    /// Deduplication records.
    dedup: Vec<AlertRecord>,
}

impl AlertEngine {
    /// Create a new engine with the given thresholds.
    pub fn new(thresholds: AlertThresholds) -> Self {
        Self {
            thresholds,
            alert_counter: 0,
            active_alerts: Vec::new(),
            suppression_list: Vec::new(),
            dedup: Vec::new(),
        }
    }

    /// Inspect `state` and return any new alerts generated.
    ///
    /// Internal deduplication prevents re-alerting the same (category, element)
    /// condition within 60 seconds.
    pub fn check_state(&mut self, state: &TwinState, timestamp_us: i64) -> Vec<TwinAlert> {
        let mut new_alerts: Vec<TwinAlert> = Vec::new();

        // ── Voltage checks ────────────────────────────────────────────────
        for (bus_idx, &v) in state.voltage_magnitudes.iter().enumerate() {
            let (violated, severity, description, threshold) =
                if v > self.thresholds.voltage_critical_high {
                    (
                        true,
                        AlertSeverity::Emergency,
                        format!("Bus {bus_idx}: voltage {v:.4} pu exceeds critical-high threshold"),
                        self.thresholds.voltage_critical_high,
                    )
                } else if v < self.thresholds.voltage_critical_low {
                    (
                        true,
                        AlertSeverity::Emergency,
                        format!("Bus {bus_idx}: voltage {v:.4} pu below critical-low threshold"),
                        self.thresholds.voltage_critical_low,
                    )
                } else if v > self.thresholds.voltage_high_pu {
                    (
                        true,
                        AlertSeverity::Warning,
                        format!("Bus {bus_idx}: voltage {v:.4} pu exceeds high-limit"),
                        self.thresholds.voltage_high_pu,
                    )
                } else if v < self.thresholds.voltage_low_pu {
                    (
                        true,
                        AlertSeverity::Warning,
                        format!("Bus {bus_idx}: voltage {v:.4} pu below low-limit"),
                        self.thresholds.voltage_low_pu,
                    )
                } else {
                    (false, AlertSeverity::Info, String::new(), 0.0)
                };

            if violated
                && !self.is_suppressed(AlertCategory::Voltage, bus_idx)
                && self.should_alert(AlertCategory::Voltage, Some(bus_idx), timestamp_us)
            {
                let action = if v > 1.0 {
                    "Reduce generator Q output or adjust transformer tap".to_string()
                } else {
                    "Increase generator Q output or capacitor bank switching".to_string()
                };
                let alert = self.make_alert(
                    timestamp_us,
                    severity,
                    AlertCategory::Voltage,
                    description,
                    Some(bus_idx),
                    v,
                    threshold,
                    action,
                );
                self.record_dedup(AlertCategory::Voltage, Some(bus_idx), timestamp_us);
                self.active_alerts.push(alert.clone());
                new_alerts.push(alert);
            }
        }

        // ── Branch thermal (loading) checks ───────────────────────────────
        for (br_idx, &p_mw) in state.branch_flows_mw.iter().enumerate() {
            // Use a nominal rating of 100 MW if ratings are not stored in state.
            // Actual % is against a reference that the caller must embed in the state
            // via the `branch_flows_mw` sign convention — we treat 100 MW as 100%.
            // For a more precise comparison the twin should carry per-branch ratings;
            // here we compute a dimensionless proxy.
            let loading_pct = p_mw.abs();
            let (violated, severity, description, threshold) = if loading_pct
                > self.thresholds.branch_loading_critical_pct
            {
                (
                    true,
                    AlertSeverity::Critical,
                    format!("Branch {br_idx}: loading {loading_pct:.1}% exceeds emergency rating"),
                    self.thresholds.branch_loading_critical_pct,
                )
            } else if loading_pct > self.thresholds.branch_loading_warning_pct {
                (
                    true,
                    AlertSeverity::Warning,
                    format!("Branch {br_idx}: loading {loading_pct:.1}% exceeds warning rating"),
                    self.thresholds.branch_loading_warning_pct,
                )
            } else {
                (false, AlertSeverity::Info, String::new(), 0.0)
            };

            if violated
                && !self.is_suppressed(AlertCategory::Thermal, br_idx)
                && self.should_alert(AlertCategory::Thermal, Some(br_idx), timestamp_us)
            {
                let alert = self.make_alert(
                    timestamp_us,
                    severity,
                    AlertCategory::Thermal,
                    description,
                    Some(br_idx),
                    loading_pct,
                    threshold,
                    "Redispatch generation or open parallel path".to_string(),
                );
                self.record_dedup(AlertCategory::Thermal, Some(br_idx), timestamp_us);
                self.active_alerts.push(alert.clone());
                new_alerts.push(alert);
            }
        }

        // ── Frequency checks ──────────────────────────────────────────────
        let f = state.frequency_hz;
        let (f_violated, f_severity, f_desc, f_threshold) =
            if f > self.thresholds.frequency_critical_high {
                (
                    true,
                    AlertSeverity::Emergency,
                    format!("System frequency {f:.3} Hz exceeds critical-high"),
                    self.thresholds.frequency_critical_high,
                )
            } else if f < self.thresholds.frequency_critical_low {
                (
                    true,
                    AlertSeverity::Emergency,
                    format!("System frequency {f:.3} Hz below critical-low"),
                    self.thresholds.frequency_critical_low,
                )
            } else if f > self.thresholds.frequency_high_hz {
                (
                    true,
                    AlertSeverity::Warning,
                    format!("System frequency {f:.3} Hz high"),
                    self.thresholds.frequency_high_hz,
                )
            } else if f < self.thresholds.frequency_low_hz {
                (
                    true,
                    AlertSeverity::Warning,
                    format!("System frequency {f:.3} Hz low"),
                    self.thresholds.frequency_low_hz,
                )
            } else {
                (false, AlertSeverity::Info, String::new(), 0.0)
            };

        if f_violated && self.should_alert(AlertCategory::Frequency, None, timestamp_us) {
            let f_action = if f > 50.0 {
                "Reduce generation output or shed interruptible load".to_string()
            } else {
                "Increase generation output or activate fast reserve".to_string()
            };
            let alert = self.make_alert(
                timestamp_us,
                f_severity,
                AlertCategory::Frequency,
                f_desc,
                None,
                f,
                f_threshold,
                f_action,
            );
            self.record_dedup(AlertCategory::Frequency, None, timestamp_us);
            self.active_alerts.push(alert.clone());
            new_alerts.push(alert);
        }

        new_alerts
    }

    /// Acknowledge an alert by ID; returns `true` if found.
    pub fn acknowledge(&mut self, alert_id: u64) -> bool {
        for alert in &mut self.active_alerts {
            if alert.alert_id == alert_id {
                alert.acknowledged = true;
                return true;
            }
        }
        false
    }

    /// Permanently suppress a (category, element) pair from future alerting.
    pub fn suppress(&mut self, category: AlertCategory, element: usize) {
        if !self.suppression_list.contains(&(category, element)) {
            self.suppression_list.push((category, element));
        }
    }

    /// Return active alerts sorted descending by severity (Emergency first).
    pub fn active_by_severity(&self) -> Vec<&TwinAlert> {
        let mut refs: Vec<&TwinAlert> = self.active_alerts.iter().collect();
        refs.sort_by_key(|b| std::cmp::Reverse(b.severity));
        refs
    }

    // ── Private helpers ───────────────────────────────────────────────────

    #[allow(clippy::too_many_arguments)]
    fn make_alert(
        &mut self,
        timestamp_us: i64,
        severity: AlertSeverity,
        category: AlertCategory,
        description: String,
        affected_element: Option<usize>,
        value: f64,
        threshold: f64,
        recommended_action: String,
    ) -> TwinAlert {
        let id = self.alert_counter;
        self.alert_counter += 1;
        TwinAlert {
            alert_id: id,
            timestamp_us,
            severity,
            category,
            description,
            affected_element,
            value,
            threshold,
            recommended_action,
            acknowledged: false,
        }
    }

    /// Returns `true` if the (category, element) alert condition was NOT fired
    /// within the last 60 seconds.
    fn should_alert(&self, category: AlertCategory, element: Option<usize>, now_us: i64) -> bool {
        const DEDUP_WINDOW_US: i64 = 60_000_000; // 60 s
        for rec in &self.dedup {
            if rec.category == category && rec.element == element {
                return (now_us - rec.last_triggered_us) > DEDUP_WINDOW_US;
            }
        }
        true
    }

    fn record_dedup(&mut self, category: AlertCategory, element: Option<usize>, now_us: i64) {
        for rec in &mut self.dedup {
            if rec.category == category && rec.element == element {
                rec.last_triggered_us = now_us;
                return;
            }
        }
        self.dedup.push(AlertRecord {
            category,
            element,
            last_triggered_us: now_us,
        });
    }

    fn is_suppressed(&self, category: AlertCategory, element: usize) -> bool {
        self.suppression_list.contains(&(category, element))
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Unit tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::digitaltwin::twin::{DataQuality, StateSource, TwinState};

    fn make_state(voltages: Vec<f64>, freq: f64) -> TwinState {
        let n = voltages.len();
        TwinState {
            voltage_magnitudes: voltages,
            voltage_angles: vec![0.0; n],
            branch_flows_mw: vec![],
            branch_flows_mvar: vec![],
            generation_mw: vec![],
            load_mw: vec![0.0; n],
            frequency_hz: freq,
            timestamp_us: 1_000_000,
            data_quality: vec![DataQuality::Good; n],
            state_source: StateSource::PowerFlow,
        }
    }

    #[test]
    fn test_alert_voltage_violation() {
        let mut engine = AlertEngine::new(AlertThresholds::default());
        let state = make_state(vec![1.0, 0.92], 50.0);
        let alerts = engine.check_state(&state, 1_000_000);
        assert!(!alerts.is_empty(), "should generate low-voltage alert");
        assert!(alerts.iter().any(|a| a.category == AlertCategory::Voltage));
    }

    #[test]
    fn test_alert_deduplication() {
        let mut engine = AlertEngine::new(AlertThresholds::default());
        let state = make_state(vec![0.92], 50.0);
        let t1 = 1_000_000_i64;
        let alerts1 = engine.check_state(&state, t1);
        assert_eq!(alerts1.len(), 1);
        // Second call within 60 s should not re-alert.
        let t2 = t1 + 10_000_000; // +10 s
        let alerts2 = engine.check_state(&state, t2);
        assert!(
            alerts2.is_empty(),
            "deduplication within 60 s should suppress"
        );
        // After 61 s it should alert again.
        let t3 = t1 + 61_000_000;
        let alerts3 = engine.check_state(&state, t3);
        assert!(
            !alerts3.is_empty(),
            "alert should fire again after 60 s window"
        );
    }

    #[test]
    fn test_alert_severity_ordering() {
        assert!(AlertSeverity::Emergency > AlertSeverity::Critical);
        assert!(AlertSeverity::Critical > AlertSeverity::Warning);
        assert!(AlertSeverity::Warning > AlertSeverity::Info);
    }

    /// A freshly created AlertEngine has zero active alerts and a zero counter.
    #[test]
    fn alert_engine_starts_empty() {
        let engine = AlertEngine::new(AlertThresholds::default());
        assert!(
            engine.active_alerts.is_empty(),
            "new engine must have no active alerts"
        );
        assert_eq!(engine.alert_counter, 0, "alert counter must start at zero");
        assert!(
            engine.suppression_list.is_empty(),
            "suppression list must start empty"
        );
    }

    /// Acknowledging an alert by its ID must set `acknowledged = true`.
    #[test]
    fn alert_acknowledge_sets_flag() {
        let mut engine = AlertEngine::new(AlertThresholds::default());
        // Trigger a voltage alert to get an alert with a known ID.
        let state = make_state(vec![0.92], 50.0);
        let alerts = engine.check_state(&state, 1_000_000);
        assert_eq!(alerts.len(), 1, "should produce one alert");
        let alert_id = alerts[0].alert_id;

        let found = engine.acknowledge(alert_id);
        assert!(found, "acknowledge should return true for a known ID");

        let acked = engine.active_alerts.iter().find(|a| a.alert_id == alert_id);
        assert!(
            acked.map(|a| a.acknowledged).unwrap_or(false),
            "alert must be marked acknowledged"
        );
    }

    /// Suppressing a (category, element) pair must prevent that alert from firing.
    #[test]
    fn alert_suppression_blocks_future_alerts() {
        let mut engine = AlertEngine::new(AlertThresholds::default());
        // Suppress voltage alerts for bus 0 before any state is checked.
        engine.suppress(AlertCategory::Voltage, 0);

        let state = make_state(vec![0.85], 50.0); // well below threshold
        let alerts = engine.check_state(&state, 1_000_000);
        assert!(
            alerts
                .iter()
                .all(|a| !(a.category == AlertCategory::Voltage && a.affected_element == Some(0))),
            "suppressed (Voltage, bus 0) should not appear in alerts"
        );
    }

    /// `active_by_severity` must return alerts sorted with highest severity first.
    #[test]
    fn alert_active_by_severity_descending() {
        let mut engine = AlertEngine::new(AlertThresholds::default());
        // Trigger a Warning-level voltage alert (0.92 pu) at t=0.
        let state_warn = make_state(vec![0.92], 50.0);
        engine.check_state(&state_warn, 0);

        // Trigger an Emergency-level frequency alert at t=62M (past dedup window).
        let state_crit = make_state(vec![1.0], 49.3);
        engine.check_state(&state_crit, 62_000_000);

        let sorted = engine.active_by_severity();
        assert!(sorted.len() >= 2, "must have at least two active alerts");
        // Verify descending order.
        let severities: Vec<AlertSeverity> = sorted.iter().map(|a| a.severity).collect();
        let is_desc = severities.windows(2).all(|w| w[0] >= w[1]);
        assert!(
            is_desc,
            "active_by_severity must return alerts in descending severity order"
        );
    }

    /// Checking a state with no violations must produce zero new alerts.
    #[test]
    fn alert_no_violations_produces_no_alerts() {
        let mut engine = AlertEngine::new(AlertThresholds::default());
        // Nominal state: voltage 1.0 pu, frequency 50.0 Hz.
        let state = make_state(vec![1.0, 1.0], 50.0);
        let alerts = engine.check_state(&state, 1_000_000);
        assert!(
            alerts.is_empty(),
            "nominal state must not generate any alerts"
        );
    }

    /// Triggering the same condition for N buses must produce exactly N alerts
    /// (one per bus, each with a distinct alert_id).
    #[test]
    fn alert_batch_distinct_ids_per_bus() {
        let mut engine = AlertEngine::new(AlertThresholds::default());
        // Four buses all with low voltage — should produce four distinct alerts.
        let state = make_state(vec![0.92, 0.91, 0.90, 0.89], 50.0);
        let alerts = engine.check_state(&state, 1_000_000);

        // There should be alerts for all four buses (some may be Emergency level for 0.89/0.90).
        assert_eq!(alerts.len(), 4, "one alert per violating bus");

        // All alert IDs must be unique.
        let mut ids: Vec<u64> = alerts.iter().map(|a| a.alert_id).collect();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), 4, "all four alert IDs must be distinct");
    }
}
