//! Power Grid Intrusion Detection System (IDS).
//!
//! Detects cyber-attacks on grid measurements using a multi-layer approach:
//!
//! | Layer | Method | Detects |
//! |-------|--------|---------|
//! | 1 | Z-score (3σ) | Steady bias, sensor faults |
//! | 2 | CUSUM | Slow ramps, persistent drift |
//! | 3 | Physics check | P²+Q²>S², V out of bounds |
//! | 4 | Rate-of-change | Sudden jumps (replay, FDI) |
//! | 5 | Cross-bus correlation | Coordinated FDI |
//!
//! # Usage
//! ```rust
//! use oxigrid::security::ids::{GridIds, GridIdsConfig, GridMeasurement};
//!
//! let config = GridIdsConfig {
//!     detection_window_s: 60.0,
//!     baseline_period_s: 300.0,
//!     sensitivity: 0.5,
//!     correlation_threshold: 0.9,
//! };
//! let mut ids = GridIds::new(config);
//! // train on clean data …
//! ```
//!
//! # References
//! Liu, Y. et al. (2011) "False Data Injection Attacks against State Estimation
//! in Electric Power Grids", *ACM Trans. Inf. Syst. Secur.* 14(1).

use serde::{Deserialize, Serialize};
use thiserror::Error;

// ─── Error ────────────────────────────────────────────────────────────────────

/// Errors from the grid IDS.
#[derive(Debug, Error)]
pub enum IdsError {
    /// The IDS has not been trained yet.
    #[error("IDS not trained: call train() before detect()")]
    NotTrained,
    /// Insufficient measurement data.
    #[error("insufficient measurements: need at least {0}")]
    InsufficientData(usize),
    /// Invalid configuration parameter.
    #[error("invalid IDS config: {0}")]
    InvalidConfig(String),
}

// ─── Configuration ────────────────────────────────────────────────────────────

/// Configuration for the power grid IDS.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GridIdsConfig {
    /// Rolling detection window \[s\].
    pub detection_window_s: f64,
    /// Baseline training period \[s\].
    pub baseline_period_s: f64,
    /// Sensitivity ∈ \[0, 1\] — higher values lower the alarm threshold
    /// (more sensitive but more false positives).
    pub sensitivity: f64,
    /// Pearson correlation threshold for cross-bus consistency checks.
    pub correlation_threshold: f64,
}

// ─── Attack signatures ────────────────────────────────────────────────────────

/// Describes a known attack pattern for reference/documentation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum AttackSignature {
    /// A constant offset injected into one measurement channel \[p.u.\].
    SteadyBias {
        /// Target bus.
        bus: usize,
        /// Measurement type (e.g. `"voltage_pu"`).
        measurement: String,
        /// Injected bias magnitude \[p.u.\].
        bias_pu: f64,
    },
    /// A linearly increasing injection \[p.u./s\].
    RampInjection {
        /// Target bus.
        bus: usize,
        /// Rate of change \[p.u./s\].
        rate_pu_per_s: f64,
    },
    /// Replaying a historical window of measurements \[s\].
    ReplayWindow {
        /// Duration of the replayed segment \[s\].
        duration_s: f64,
    },
    /// Simultaneous false-data injection on multiple buses.
    CoordinatedFdi {
        /// Set of target bus IDs.
        buses: Vec<usize>,
    },
    /// High-frequency noise injected at one measurement \[p.u.\].
    HighFrequencyNoise {
        /// Target bus.
        bus: usize,
        /// Peak noise amplitude \[p.u.\].
        amplitude: f64,
    },
}

// ─── Measurement ──────────────────────────────────────────────────────────────

/// A single grid measurement sample.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GridMeasurement {
    /// Measurement timestamp \[s\] (wall-clock or simulation time).
    pub timestamp_s: f64,
    /// Bus identifier this measurement belongs to.
    pub bus_id: usize,
    /// Measurement type tag (e.g. `"P_MW"`, `"Q_MVAR"`, `"V_pu"`).
    pub measurement_type: String,
    /// Measured value in consistent per-unit or engineering units.
    pub value: f64,
    /// Data quality flag (0 = bad, 255 = perfect).
    pub quality: u8,
}

// ─── Alert types ──────────────────────────────────────────────────────────────

/// Classification of a detected anomaly.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum IdsAlertType {
    /// Statistical bias consistent with false data injection.
    FalseDataInjection,
    /// Measurement sequence repeated from a historical window.
    ReplayAttack,
    /// Unexpected communication gap or burst.
    CommunicationAnomaly,
    /// Measurements from different sensors are mutually inconsistent.
    MeasurementInconsistency,
    /// Multiple buses showing correlated anomalies simultaneously.
    CoordinatedManipulation,
    /// Anomaly pattern consistent with hardware/software sensor fault.
    SensorFault,
}

/// A single IDS alert.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IdsAlert {
    /// Timestamp at which the alert was raised \[s\].
    pub timestamp_s: f64,
    /// Alert classification.
    pub alert_type: IdsAlertType,
    /// Confidence \[0, 1\] that this is a genuine attack.
    pub confidence: f64,
    /// Indices (into the input measurement slice) of affected measurements.
    pub affected_measurements: Vec<usize>,
    /// Recommended operator action.
    pub recommended_action: String,
}

// ─── IDS result ───────────────────────────────────────────────────────────────

/// Aggregate result of a [`GridIds::detect`] call.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IdsResult {
    /// All alerts raised.
    pub alerts: Vec<IdsAlert>,
    /// Number of measurements classified as anomalous.
    pub n_anomalous_measurements: usize,
    /// `true` if at least one high-confidence attack alert was raised.
    pub attack_detected: bool,
    /// Primary attack type (highest-confidence alert).
    pub attack_type: Option<IdsAlertType>,
    /// Timestamp of the first alert \[s\].
    pub detection_time_s: Option<f64>,
    /// Fraction of alerts estimated to be false positives.
    pub false_positive_estimate: f64,
}

// ─── Per-channel baseline stats ───────────────────────────────────────────────

/// Baseline statistics for one measurement channel (bus + type pair).
#[derive(Debug, Clone)]
struct ChannelStats {
    bus_id: usize,
    measurement_type: String,
    mean: f64,
    std: f64,
    /// EWMA state for online baseline tracking.
    ewma: f64,
}

// ─── IDS engine ───────────────────────────────────────────────────────────────

/// Power grid Intrusion Detection System.
pub struct GridIds {
    config: GridIdsConfig,
    /// Baseline stats per channel, indexed by (bus_id, measurement_type).
    baseline_stats: Vec<ChannelStats>,
}

impl GridIds {
    /// Create a new IDS with the given configuration.
    pub fn new(config: GridIdsConfig) -> Self {
        Self {
            config,
            baseline_stats: Vec::new(),
        }
    }

    // ── Training ─────────────────────────────────────────────────────────────

    /// Train the baseline from a set of known-clean measurements.
    ///
    /// Groups measurements by `(bus_id, measurement_type)` and computes mean
    /// and standard deviation for each channel.
    pub fn train(&mut self, clean_measurements: &[GridMeasurement]) -> Result<(), IdsError> {
        if clean_measurements.is_empty() {
            return Err(IdsError::InsufficientData(1));
        }
        self.baseline_stats.clear();

        // Group by channel
        let mut channels: std::collections::HashMap<(usize, String), Vec<f64>> =
            std::collections::HashMap::new();
        for m in clean_measurements {
            channels
                .entry((m.bus_id, m.measurement_type.clone()))
                .or_default()
                .push(m.value);
        }

        for ((bus_id, mtype), values) in channels {
            let mean = stat_mean(&values);
            let std = stat_std(&values, mean);
            self.baseline_stats.push(ChannelStats {
                bus_id,
                measurement_type: mtype,
                mean,
                std,
                ewma: mean,
            });
        }
        Ok(())
    }

    // ── Detection ────────────────────────────────────────────────────────────

    /// Detect anomalies in a batch of streaming measurements.
    ///
    /// Returns an [`IdsResult`] summarising all alerts.
    pub fn detect(&self, measurements: &[GridMeasurement]) -> Result<IdsResult, IdsError> {
        if self.baseline_stats.is_empty() {
            return Err(IdsError::NotTrained);
        }
        if measurements.is_empty() {
            return Ok(IdsResult {
                alerts: Vec::new(),
                n_anomalous_measurements: 0,
                attack_detected: false,
                attack_type: None,
                detection_time_s: None,
                false_positive_estimate: 0.0,
            });
        }

        // z-score threshold scales inversely with sensitivity
        let z_thresh = z_score_threshold(self.config.sensitivity);
        // CUSUM threshold
        let cusum_thresh = cusum_threshold(self.config.sensitivity);

        let mut anomalous_indices: Vec<usize> = Vec::new();
        let mut alerts: Vec<IdsAlert> = Vec::new();

        // ── Layer 1 & 2: z-score + CUSUM per measurement ─────────────────────
        let mut cusum_pos_state: Vec<f64> = vec![0.0; measurements.len()];
        let mut cusum_neg_state: Vec<f64> = vec![0.0; measurements.len()];

        for (idx, m) in measurements.iter().enumerate() {
            if let Some(stats) = self.find_channel(m.bus_id, &m.measurement_type) {
                let z = z_score(m.value, stats.mean, stats.std);

                // Layer 1: z-score
                if z.abs() > z_thresh {
                    anomalous_indices.push(idx);
                    let confidence = (z.abs() / z_thresh - 1.0).min(1.0) * 0.8;
                    alerts.push(IdsAlert {
                        timestamp_s: m.timestamp_s,
                        alert_type: IdsAlertType::FalseDataInjection,
                        confidence,
                        affected_measurements: vec![idx],
                        recommended_action: format!(
                            "Verify measurement on bus {} (z={:.2})",
                            m.bus_id, z
                        ),
                    });
                }

                // Layer 2: CUSUM — accumulate across measurements in order
                let slack = 0.5 * stats.std.max(1e-9);
                let cp = if idx == 0 {
                    (m.value - stats.mean - slack).max(0.0)
                } else {
                    (cusum_pos_state[idx - 1] + m.value - stats.mean - slack).max(0.0)
                };
                let cn = if idx == 0 {
                    (-(m.value - stats.mean) - slack).max(0.0)
                } else {
                    (cusum_neg_state[idx - 1] - (m.value - stats.mean) - slack).max(0.0)
                };
                cusum_pos_state[idx] = cp;
                cusum_neg_state[idx] = cn;

                if cp > cusum_thresh || cn > cusum_thresh {
                    if !anomalous_indices.contains(&idx) {
                        anomalous_indices.push(idx);
                    }
                    let cusum_val = cp.max(cn);
                    let confidence = (cusum_val / cusum_thresh - 1.0).clamp(0.0, 1.0) * 0.7;
                    alerts.push(IdsAlert {
                        timestamp_s: m.timestamp_s,
                        alert_type: IdsAlertType::FalseDataInjection,
                        confidence,
                        affected_measurements: vec![idx],
                        recommended_action: format!(
                            "CUSUM alarm on bus {} (cusum={:.2})",
                            m.bus_id, cusum_val
                        ),
                    });
                }
            }
        }

        // ── Layer 3: Physics consistency (P²+Q²>S²) ─────────────────────────
        let phys_anomalies = self.check_physics_consistency(measurements);
        for &idx in &phys_anomalies {
            if !anomalous_indices.contains(&idx) {
                anomalous_indices.push(idx);
            }
            alerts.push(IdsAlert {
                timestamp_s: measurements[idx].timestamp_s,
                alert_type: IdsAlertType::MeasurementInconsistency,
                confidence: 0.9,
                affected_measurements: vec![idx],
                recommended_action: format!(
                    "Physics violation at bus {} — P²+Q²>S²",
                    measurements[idx].bus_id
                ),
            });
        }

        // ── Layer 4: Rate-of-change (temporal jump detection) ────────────────
        let roc_anomalies = self.check_rate_of_change(measurements);
        for &idx in &roc_anomalies {
            if !anomalous_indices.contains(&idx) {
                anomalous_indices.push(idx);
            }
            alerts.push(IdsAlert {
                timestamp_s: measurements[idx].timestamp_s,
                alert_type: IdsAlertType::ReplayAttack,
                confidence: 0.75,
                affected_measurements: vec![idx],
                recommended_action: format!(
                    "Temporal jump detected at bus {} — possible replay",
                    measurements[idx].bus_id
                ),
            });
        }

        // ── Layer 5: Cross-bus correlation ────────────────────────────────────
        let corr_anomalies = self.check_correlation(measurements);
        if !corr_anomalies.is_empty() {
            let mut aff = corr_anomalies.clone();
            for &idx in &aff {
                if !anomalous_indices.contains(&idx) {
                    anomalous_indices.push(idx);
                }
            }
            aff.sort_unstable();
            aff.dedup();
            alerts.push(IdsAlert {
                timestamp_s: measurements.first().map(|m| m.timestamp_s).unwrap_or(0.0),
                alert_type: IdsAlertType::CoordinatedManipulation,
                confidence: 0.85,
                affected_measurements: aff,
                recommended_action: "Coordinated FDI suspected — cross-bus correlation anomaly"
                    .into(),
            });
        }

        // ── Aggregate ─────────────────────────────────────────────────────────
        anomalous_indices.sort_unstable();
        anomalous_indices.dedup();

        let attack_detected = alerts.iter().any(|a| a.confidence >= 0.7);
        let attack_type = alerts
            .iter()
            .max_by(|a, b| {
                a.confidence
                    .partial_cmp(&b.confidence)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|a| a.alert_type.clone());
        let detection_time_s = alerts
            .iter()
            .min_by(|a, b| {
                a.timestamp_s
                    .partial_cmp(&b.timestamp_s)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|a| a.timestamp_s);

        // False-positive estimate: fraction of alerts below high-confidence threshold
        let fp_est = if alerts.is_empty() {
            0.0
        } else {
            alerts.iter().filter(|a| a.confidence < 0.5).count() as f64 / alerts.len() as f64
        };

        Ok(IdsResult {
            alerts,
            n_anomalous_measurements: anomalous_indices.len(),
            attack_detected,
            attack_type,
            detection_time_s,
            false_positive_estimate: fp_est,
        })
    }

    // ── Online baseline update ────────────────────────────────────────────────

    /// Update the baseline with a new measurement using EWMA to track slow drift.
    ///
    /// `α = sensitivity * 0.1` so a higher sensitivity tracks faster.
    pub fn update_baseline(&mut self, measurement: &GridMeasurement) {
        let alpha = (self.config.sensitivity * 0.1).clamp(0.001, 0.5);
        for ch in &mut self.baseline_stats {
            if ch.bus_id == measurement.bus_id
                && ch.measurement_type == measurement.measurement_type
            {
                ch.ewma = alpha * measurement.value + (1.0 - alpha) * ch.ewma;
                // Update mean slowly toward EWMA
                ch.mean = 0.99 * ch.mean + 0.01 * ch.ewma;
            }
        }
    }

    // ── Physical consistency check ────────────────────────────────────────────

    /// Check that P²+Q² ≤ S²_apparent for each bus group.
    ///
    /// Returns indices of measurements involved in a physics violation.
    fn check_physics_consistency(&self, measurements: &[GridMeasurement]) -> Vec<usize> {
        // Group by bus: find P, Q, S (or V*I approximation)
        use std::collections::HashMap;
        let mut by_bus: HashMap<usize, Vec<(usize, &GridMeasurement)>> = HashMap::new();
        for (idx, m) in measurements.iter().enumerate() {
            by_bus.entry(m.bus_id).or_default().push((idx, m));
        }

        let mut violations: Vec<usize> = Vec::new();
        for entries in by_bus.values() {
            let p_opt = entries
                .iter()
                .find(|(_, m)| m.measurement_type.contains('P'));
            let q_opt = entries
                .iter()
                .find(|(_, m)| m.measurement_type.contains('Q'));
            let s_opt = entries
                .iter()
                .find(|(_, m)| m.measurement_type.contains('S'));

            if let (Some((pi, pm)), Some((qi, qm)), Some((si, sm))) = (p_opt, q_opt, s_opt) {
                let p2q2 = pm.value.powi(2) + qm.value.powi(2);
                let s2 = sm.value.powi(2);
                if p2q2 > s2 * 1.01 {
                    // 1% tolerance
                    violations.push(*pi);
                    violations.push(*qi);
                    violations.push(*si);
                }
            }
        }
        violations
    }

    /// Check cross-measurement correlation: buses that historically co-move
    /// should still co-move; divergence signals coordinated manipulation.
    fn check_correlation(&self, measurements: &[GridMeasurement]) -> Vec<usize> {
        // Simple approach: compare all pairs of same-type measurements across buses.
        // Flag if a pair that should correlate shows anti-correlation.
        let types: &[&str] = &["P", "Q", "V"];
        let mut anomalous: Vec<usize> = Vec::new();

        for mtype in types {
            let group: Vec<(usize, f64)> = measurements
                .iter()
                .enumerate()
                .filter(|(_, m)| m.measurement_type.contains(mtype))
                .map(|(i, m)| (i, m.value))
                .collect();

            if group.len() < 2 {
                continue;
            }

            // Normalise: compute z-scores relative to baseline
            let stats_for: Vec<Option<&ChannelStats>> = group
                .iter()
                .map(|(i, _)| {
                    let m = &measurements[*i];
                    self.find_channel(m.bus_id, &m.measurement_type)
                })
                .collect();

            let z_scores: Vec<f64> = group
                .iter()
                .zip(stats_for.iter())
                .map(|((_, v), stats_opt)| {
                    if let Some(st) = stats_opt {
                        z_score(*v, st.mean, st.std)
                    } else {
                        0.0
                    }
                })
                .collect();

            // Detect if some z-scores have opposite signs with large magnitude
            let pos: Vec<usize> = z_scores
                .iter()
                .enumerate()
                .filter(|(_, &z)| z > 2.0)
                .map(|(i, _)| i)
                .collect();
            let neg: Vec<usize> = z_scores
                .iter()
                .enumerate()
                .filter(|(_, &z)| z < -2.0)
                .map(|(i, _)| i)
                .collect();

            // If we have simultaneous large pos and large neg deviations across
            // buses for the same measurement type, flag coordinated FDI.
            if !pos.is_empty() && !neg.is_empty() {
                for i in &pos {
                    anomalous.push(group[*i].0);
                }
                for i in &neg {
                    anomalous.push(group[*i].0);
                }
            }
        }
        anomalous
    }

    // ── Temporal rate-of-change check ─────────────────────────────────────────

    /// Detect sudden jumps inconsistent with physical rate-of-change limits.
    fn check_rate_of_change(&self, measurements: &[GridMeasurement]) -> Vec<usize> {
        use std::collections::HashMap;
        // Group by (bus, type) and check consecutive differences
        type ChannelKey = (usize, String);
        type ChannelEntries = Vec<(usize, f64, f64)>;
        let mut by_channel: HashMap<ChannelKey, ChannelEntries> = HashMap::new();
        for (idx, m) in measurements.iter().enumerate() {
            by_channel
                .entry((m.bus_id, m.measurement_type.clone()))
                .or_default()
                .push((idx, m.timestamp_s, m.value));
        }

        let mut anomalous: Vec<usize> = Vec::new();
        for ((bus_id, mtype), mut entries) in by_channel {
            entries.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
            if entries.len() < 2 {
                continue;
            }
            if let Some(stats) = self.find_channel(bus_id, &mtype) {
                // Max physically plausible rate of change: 5σ/s
                let max_roc = stats.std * 5.0 + 1e-6;
                for i in 1..entries.len() {
                    let dt = (entries[i].1 - entries[i - 1].1).abs();
                    if dt < 1e-9 {
                        continue;
                    }
                    let dv = (entries[i].2 - entries[i - 1].2).abs();
                    if dv / dt > max_roc {
                        anomalous.push(entries[i].0);
                    }
                }
            }
        }
        anomalous
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    fn find_channel(&self, bus_id: usize, mtype: &str) -> Option<&ChannelStats> {
        self.baseline_stats
            .iter()
            .find(|s| s.bus_id == bus_id && s.measurement_type == mtype)
    }
}

// ─── Statistics helpers ───────────────────────────────────────────────────────

fn stat_mean(v: &[f64]) -> f64 {
    if v.is_empty() {
        return 0.0;
    }
    v.iter().sum::<f64>() / v.len() as f64
}

fn stat_std(v: &[f64], mean: f64) -> f64 {
    if v.len() < 2 {
        // Use relative floor: 1 % of |mean| or 1.0, whichever is larger.
        return (mean.abs() * 0.01).max(1.0);
    }
    let var = v.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / v.len() as f64;
    let raw = var.sqrt();
    // Enforce a relative floor: at least 0.5 % of |mean| to avoid near-zero std
    // amplifying tiny measurement noise into false alarms.
    raw.max(mean.abs() * 0.005).max(1e-9)
}

fn z_score(value: f64, mean: f64, std: f64) -> f64 {
    (value - mean) / std.max(1e-9)
}

/// Compute z-score threshold from sensitivity ∈ \[0, 1\].
///
/// At sensitivity=0 → threshold=4.0 (very few alarms).
/// At sensitivity=1 → threshold=1.5 (many alarms).
fn z_score_threshold(sensitivity: f64) -> f64 {
    4.0 - sensitivity.clamp(0.0, 1.0) * 2.5
}

/// CUSUM alarm threshold scaled by sensitivity.
fn cusum_threshold(sensitivity: f64) -> f64 {
    10.0 - sensitivity.clamp(0.0, 1.0) * 7.0
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> GridIdsConfig {
        GridIdsConfig {
            detection_window_s: 60.0,
            baseline_period_s: 300.0,
            sensitivity: 0.5,
            correlation_threshold: 0.9,
        }
    }

    fn make_measurement(t: f64, bus: usize, mtype: &str, value: f64) -> GridMeasurement {
        GridMeasurement {
            timestamp_s: t,
            bus_id: bus,
            measurement_type: mtype.to_string(),
            value,
            quality: 255,
        }
    }

    fn build_clean_measurements(n: usize) -> Vec<GridMeasurement> {
        (0..n)
            .map(|i| make_measurement(i as f64, 1, "P_MW", 100.0 + (i as f64 % 3_f64) * 0.5))
            .collect()
    }

    // ── Test 1 ─────────────────────────────────────────────────────────────────
    /// Clean data must not produce any alerts.
    #[test]
    fn test_clean_data_no_false_alarms() {
        let mut ids = GridIds::new(default_config());
        let clean: Vec<GridMeasurement> = (0..100)
            .map(|i| make_measurement(i as f64, 1, "P_MW", 100.0))
            .collect();
        ids.train(&clean).expect("train failed");

        // Test measurements equal to baseline (no deviation)
        let test: Vec<GridMeasurement> = (100..110)
            .map(|i| make_measurement(i as f64, 1, "P_MW", 100.0))
            .collect();
        let result = ids.detect(&test).expect("detect failed");
        assert!(
            !result.attack_detected,
            "clean data should not trigger attack flag"
        );
    }

    // ── Test 2 ─────────────────────────────────────────────────────────────────
    /// A steady bias >> 3σ is detected by z-score.
    #[test]
    fn test_steady_bias_detected_by_z_score() {
        let mut ids = GridIds::new(GridIdsConfig {
            sensitivity: 0.5,
            ..default_config()
        });
        let clean: Vec<GridMeasurement> = (0..100)
            .map(|i| make_measurement(i as f64, 1, "P_MW", 100.0))
            .collect();
        ids.train(&clean).expect("train failed");

        // Inject a large bias (50 units >> any std in clean data which is 0)
        let biased: Vec<GridMeasurement> = vec![make_measurement(200.0, 1, "P_MW", 200.0)];
        let result = ids.detect(&biased).expect("detect failed");
        assert!(
            result.n_anomalous_measurements > 0,
            "steady bias must be flagged"
        );
    }

    // ── Test 3 ─────────────────────────────────────────────────────────────────
    /// Replay attack detected by temporal jump in rate-of-change check.
    #[test]
    fn test_replay_detected_by_temporal_anomaly() {
        let mut ids = GridIds::new(default_config());
        // Train with slowly varying data
        let clean: Vec<GridMeasurement> = (0..100)
            .map(|i| make_measurement(i as f64, 1, "P_MW", 100.0 + i as f64 * 0.01))
            .collect();
        ids.train(&clean).expect("train failed");

        // Two consecutive samples with a huge jump (simulated replay desync)
        let test = vec![
            make_measurement(200.0, 1, "P_MW", 100.5),
            make_measurement(201.0, 1, "P_MW", 500.0), // jump >> max_roc
        ];
        let result = ids.detect(&test).expect("detect failed");
        assert!(
            result.n_anomalous_measurements > 0,
            "replay jump must be flagged"
        );
    }

    // ── Test 4 ─────────────────────────────────────────────────────────────────
    /// Physics inconsistency: P²+Q² > S² is flagged.
    #[test]
    fn test_physics_inconsistency_flagged() {
        let mut ids = GridIds::new(default_config());
        let clean = vec![
            make_measurement(0.0, 1, "P_MW", 80.0),
            make_measurement(0.0, 1, "Q_MVAR", 60.0),
            make_measurement(0.0, 1, "S_MVA", 100.0), // consistent: 80²+60²=100²
        ];
        ids.train(&clean).expect("train failed");

        // Now inject P=90, Q=60, S=100 → P²+Q² = 8100+3600 = 11700 > 10000 = S²
        let test = vec![
            make_measurement(1.0, 1, "P_MW", 90.0),
            make_measurement(1.0, 1, "Q_MVAR", 60.0),
            make_measurement(1.0, 1, "S_MVA", 100.0),
        ];
        let result = ids.detect(&test).expect("detect failed");
        let has_physics_alert = result
            .alerts
            .iter()
            .any(|a| a.alert_type == IdsAlertType::MeasurementInconsistency);
        assert!(has_physics_alert, "P²+Q²>S² must trigger physics alert");
    }

    // ── Test 5 ─────────────────────────────────────────────────────────────────
    /// Coordinated FDI across buses: simultaneous divergent z-scores flagged.
    #[test]
    fn test_coordinated_fdi_cross_bus() {
        let mut ids = GridIds::new(GridIdsConfig {
            sensitivity: 0.8,
            ..default_config()
        });
        // Baseline: bus 1 and bus 2 both at P=100
        let clean: Vec<GridMeasurement> = (0..50)
            .flat_map(|i| {
                vec![
                    make_measurement(i as f64, 1, "P_MW", 100.0),
                    make_measurement(i as f64, 2, "P_MW", 100.0),
                ]
            })
            .collect();
        ids.train(&clean).expect("train failed");

        // Coordinated attack: bus 1 P soars, bus 2 P plummets
        let attack = vec![
            make_measurement(200.0, 1, "P_MW", 200.0), // large positive deviation
            make_measurement(200.0, 2, "P_MW", 0.0),   // large negative deviation
        ];
        let result = ids.detect(&attack).expect("detect failed");
        assert!(
            result.n_anomalous_measurements > 0,
            "coordinated FDI must be detected"
        );
    }

    // ── Test 6: update_baseline doesn't crash ─────────────────────────────────
    #[test]
    fn test_update_baseline_ewma() {
        let mut ids = GridIds::new(default_config());
        let clean = build_clean_measurements(20);
        ids.train(&clean).expect("train failed");
        // Should not panic
        ids.update_baseline(&make_measurement(999.0, 1, "P_MW", 100.5));
    }
}
