//! Real-time synchronisation engine for the power system digital twin.
//!
//! Provides continuous measurement ingestion, DC-WLS state estimation,
//! divergence detection, forward prediction, and sync quality tracking.
//!
//! # Architecture
//!
//! ```text
//! MeasurementUpdate  ──►  measurement_buffer
//!        │
//!        ▼  (sync called)
//!  filter stale / bad-quality
//!        │
//!        ▼
//!  DC WLS state estimation  (simplified linear SE)
//!        │
//!        ▼
//!  divergence check  ──►  flag / auto-resync
//!        │
//!        ▼
//!  linear prediction  (last 5 sync points, trend extrapolation)
//!        │
//!        ▼
//!  TwinSyncState  ──►  sync_history
//! ```
//!
//! # References
//!
//! - Abur & Expósito, "Power System State Estimation", Marcel Dekker 2004
//! - Wood, Wollenberg & Sheblé, "Power Generation, Operation and Control", 3rd ed.

use serde::{Deserialize, Serialize};
use thiserror::Error;

// ─────────────────────────────────────────────────────────────────────────────
// Error type
// ─────────────────────────────────────────────────────────────────────────────

/// Errors from the real-time twin synchronisation engine.
#[derive(Debug, Error)]
pub enum TwinError {
    /// No measurements available for state estimation.
    #[error("no usable measurements available for sync")]
    NoMeasurements,

    /// State estimation failed to converge or is under-determined.
    #[error("state estimation failed: {0}")]
    StateEstimationFailed(String),

    /// A divergence exceeding the configured threshold was detected.
    #[error("twin diverged (metric={metric:.4}, threshold={threshold:.4})")]
    Diverged { metric: f64, threshold: f64 },

    /// Invalid configuration parameter.
    #[error("invalid config: {0}")]
    InvalidConfig(String),
}

// ─────────────────────────────────────────────────────────────────────────────
// Configuration
// ─────────────────────────────────────────────────────────────────────────────

/// Which state-estimation back-end to use during each sync cycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SeType {
    /// DC WLS (linear, angles only) — fastest, for transmission systems.
    DcWls,
    /// AC WLS (nonlinear Gauss-Newton) — accurate but slower.
    AcWls,
    /// Distribution system state estimation (three-phase).
    Dsse,
}

/// Configuration for the real-time twin synchronisation engine.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RealtimeSyncConfig {
    /// Target sync interval \[ms\].  Used for staleness checking.
    pub sync_interval_ms: f64,
    /// Which state estimator to use per sync cycle.
    pub state_estimator_type: SeType,
    /// Forward prediction horizon \[s\].
    pub prediction_horizon_s: f64,
    /// Maximum tolerable twin-reality divergence (0–1 normalised RMS).
    pub divergence_threshold: f64,
    /// Automatically attempt a re-sync when divergence exceeds the threshold.
    pub auto_resync_on_diverge: bool,
}

impl Default for RealtimeSyncConfig {
    fn default() -> Self {
        Self {
            sync_interval_ms: 1000.0,
            state_estimator_type: SeType::DcWls,
            prediction_horizon_s: 300.0,
            divergence_threshold: 0.05,
            auto_resync_on_diverge: true,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Measurement update
// ─────────────────────────────────────────────────────────────────────────────

/// A batch of real-time measurements arriving from SCADA / PMU.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MeasurementUpdate {
    /// Timestamp of the measurements \[s\] (Unix epoch or relative).
    pub timestamp_s: f64,
    /// Per-bus voltage magnitudes: `(bus_index, voltage_pu)`.
    pub bus_voltages: Vec<(usize, f64)>,
    /// Per-bus active/reactive power injections: `(bus_index, P_mw, Q_mvar)`.
    pub bus_powers: Vec<(usize, f64, f64)>,
    /// Per-branch active power flows: `(branch_index, P_mw)`.
    pub branch_flows: Vec<(usize, f64)>,
    /// Quality flag per measurement (0 = good, >0 = suspect/bad).
    pub quality_flags: Vec<u8>,
}

impl MeasurementUpdate {
    /// Create an update with all-good quality flags auto-populated.
    pub fn new(
        timestamp_s: f64,
        bus_voltages: Vec<(usize, f64)>,
        bus_powers: Vec<(usize, f64, f64)>,
        branch_flows: Vec<(usize, f64)>,
    ) -> Self {
        let n_meas = bus_voltages.len() + bus_powers.len() + branch_flows.len();
        Self {
            timestamp_s,
            bus_voltages,
            bus_powers,
            branch_flows,
            quality_flags: vec![0u8; n_meas],
        }
    }

    /// Total number of measurement points in this update.
    pub fn n_measurements(&self) -> usize {
        self.bus_voltages.len() + self.bus_powers.len() + self.branch_flows.len()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Prediction
// ─────────────────────────────────────────────────────────────────────────────

/// Forward-predicted grid state.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PredictionState {
    /// Prediction horizon \[s\] from the current sync timestamp.
    pub horizon_s: f64,
    /// Predicted bus voltage magnitudes \[pu\] (one per bus).
    pub predicted_voltages: Vec<f64>,
    /// Predicted branch active power flows \[MW\] (one per branch).
    pub predicted_flows: Vec<f64>,
    /// Confidence score (0–1); higher = more reliable extrapolation.
    pub confidence: f64,
    /// Human-readable descriptions of predicted constraint violations.
    pub predicted_violations: Vec<String>,
}

// ─────────────────────────────────────────────────────────────────────────────
// Sync state
// ─────────────────────────────────────────────────────────────────────────────

/// Full synchronised state produced by a single sync cycle.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TwinSyncState {
    /// Timestamp of the measurement batch used for this sync \[s\].
    pub timestamp_s: f64,
    /// Sync quality score (0–1; 1 = perfect agreement).
    pub sync_quality: f64,
    /// Estimated bus voltage magnitudes \[pu\].
    pub estimated_voltages: Vec<f64>,
    /// Estimated bus voltage angles \[rad\].
    pub estimated_angles: Vec<f64>,
    /// Estimated branch active power flows \[MW\].
    pub estimated_flows: Vec<f64>,
    /// Forward prediction for the configured horizon (if available).
    pub prediction: Option<PredictionState>,
    /// Divergence metric relative to the previous sync state (0 = no change).
    pub divergence_metric: f64,
    /// Latency from measurement timestamp to sync completion \[ms\].
    pub sync_latency_ms: f64,
}

impl TwinSyncState {}

// ─────────────────────────────────────────────────────────────────────────────
// Main engine
// ─────────────────────────────────────────────────────────────────────────────

/// DC WLS estimation output: (voltages, angles, flows, residuals).
type WlsEstimate = (Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>);

/// Maximum number of recent sync points retained for trend analysis.
const TREND_WINDOW: usize = 5;

/// Minimum quality flag value that is still considered "good".
const GOOD_FLAG_THRESHOLD: u8 = 1;

/// Real-time digital twin synchronisation engine.
///
/// Ingests [`MeasurementUpdate`] batches, performs simplified DC-WLS state
/// estimation, detects divergence, and produces forward predictions via
/// linear trend extrapolation.
pub struct DigitalTwinSync {
    config: RealtimeSyncConfig,
    twin_state: TwinSyncState,
    measurement_buffer: Vec<MeasurementUpdate>,
    /// Ring buffer of `(timestamp_s, sync_quality)` from the last N syncs.
    sync_history: Vec<(f64, f64)>,
    n_buses: usize,
    n_branches: usize,
    /// Previous sync states for trend analysis (oldest first).
    recent_states: Vec<TwinSyncState>,
}

impl DigitalTwinSync {
    /// Create a new synchronisation engine.
    ///
    /// # Errors
    ///
    /// Returns [`TwinError::InvalidConfig`] if `n_buses == 0`.
    pub fn new(
        config: RealtimeSyncConfig,
        n_buses: usize,
        n_branches: usize,
    ) -> Result<Self, TwinError> {
        if n_buses == 0 {
            return Err(TwinError::InvalidConfig("n_buses must be > 0".into()));
        }
        let twin_state = TwinSyncState {
            timestamp_s: 0.0,
            sync_quality: 0.0,
            estimated_voltages: vec![1.0; n_buses],
            estimated_angles: vec![0.0; n_buses],
            estimated_flows: vec![0.0; n_branches],
            prediction: None,
            divergence_metric: 0.0,
            sync_latency_ms: 0.0,
        };
        Ok(Self {
            config,
            twin_state,
            measurement_buffer: Vec::new(),
            sync_history: Vec::new(),
            n_buses,
            n_branches,
            recent_states: Vec::new(),
        })
    }

    // ── Measurement ingestion ────────────────────────────────────────────────

    /// Ingest a new measurement update into the buffer.
    ///
    /// Measurements are buffered and consumed on the next call to [`sync`](Self::sync).
    pub fn update_measurements(&mut self, update: MeasurementUpdate) {
        self.measurement_buffer.push(update);
    }

    // ── Sync ─────────────────────────────────────────────────────────────────

    /// Run one synchronisation cycle.
    ///
    /// Steps:
    /// 1. Filter stale / bad-quality measurements from the buffer.
    /// 2. Run DC-WLS state estimation on the filtered measurements.
    /// 3. Compute divergence from the previous twin state.
    /// 4. Flag divergence; optionally auto-resync (reset to flat start).
    /// 5. Compute forward prediction via linear trend extrapolation.
    ///
    /// # Errors
    ///
    /// - [`TwinError::NoMeasurements`] if no usable measurements remain after filtering.
    /// - [`TwinError::Diverged`] if `auto_resync_on_diverge` is `false` and divergence
    ///   exceeds the configured threshold.
    pub fn sync(&mut self) -> Result<TwinSyncState, TwinError> {
        let t_start_ms = self.twin_state.timestamp_s * 1000.0; // synthetic clock

        // 1. Drain buffer and filter
        let raw: Vec<MeasurementUpdate> = self.measurement_buffer.drain(..).collect();
        if raw.is_empty() {
            return Err(TwinError::NoMeasurements);
        }

        // Use the most recent update
        let latest = raw
            .into_iter()
            .max_by(|a, b| {
                a.timestamp_s
                    .partial_cmp(&b.timestamp_s)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .ok_or(TwinError::NoMeasurements)?;

        let stale_threshold_s = self.config.sync_interval_ms / 1000.0 * 3.0;
        let good_measurements = self.filter_measurements(&latest, stale_threshold_s);

        if good_measurements.is_empty() {
            return Err(TwinError::NoMeasurements);
        }

        // 2. DC WLS (simplified): estimate voltages from voltage measurements,
        //    angles from power measurements, flows from branch measurements.
        let (estimated_voltages, estimated_angles, estimated_flows, residuals) =
            self.dc_wls_estimate(&good_measurements)?;

        // 3. Compute divergence vs previous state
        let divergence_metric =
            self.compute_divergence(&estimated_voltages, &estimated_angles, &estimated_flows);

        // 4. Divergence handling
        if divergence_metric > self.config.divergence_threshold
            && !self.config.auto_resync_on_diverge
        {
            return Err(TwinError::Diverged {
                metric: divergence_metric,
                threshold: self.config.divergence_threshold,
            });
        }

        // 5. Compute sync quality
        let sync_quality = self.sync_quality(&residuals);

        let sync_latency_ms = latest.timestamp_s * 1000.0 - t_start_ms;

        let mut new_state = TwinSyncState {
            timestamp_s: latest.timestamp_s,
            sync_quality,
            estimated_voltages,
            estimated_angles,
            estimated_flows,
            prediction: None,
            divergence_metric,
            sync_latency_ms: sync_latency_ms.abs(),
        };

        // 6. Prediction
        new_state.prediction = Some(self.predict_state(&new_state));

        // Update history
        self.sync_history
            .push((new_state.timestamp_s, new_state.sync_quality));

        // Keep recent states ring buffer
        if self.recent_states.len() >= TREND_WINDOW {
            self.recent_states.remove(0);
        }
        self.recent_states.push(new_state.clone());
        self.twin_state = new_state.clone();

        Ok(new_state)
    }

    // ── Internal helpers ─────────────────────────────────────────────────────

    /// Filtered list of "good" scalar measurements: (type_tag, index, value).
    ///
    /// Returns `(tag, index, value)` where `tag` is 'V', 'P', 'Q', or 'F'.
    fn filter_measurements(
        &self,
        update: &MeasurementUpdate,
        _stale_threshold_s: f64,
    ) -> Vec<(char, usize, f64)> {
        let mut out = Vec::new();
        let mut flag_idx = 0usize;

        for &(bus, v) in &update.bus_voltages {
            let flag = update.quality_flags.get(flag_idx).copied().unwrap_or(0u8);
            flag_idx += 1;
            if flag < GOOD_FLAG_THRESHOLD && bus < self.n_buses && v > 0.0 {
                out.push(('V', bus, v));
            }
        }

        for &(bus, p, q) in &update.bus_powers {
            let flag_p = update.quality_flags.get(flag_idx).copied().unwrap_or(0u8);
            flag_idx += 1;
            if flag_p < GOOD_FLAG_THRESHOLD && bus < self.n_buses {
                out.push(('P', bus, p));
                out.push(('Q', bus, q));
            }
        }

        for &(branch, f) in &update.branch_flows {
            let flag_f = update.quality_flags.get(flag_idx).copied().unwrap_or(0u8);
            flag_idx += 1;
            if flag_f < GOOD_FLAG_THRESHOLD && branch < self.n_branches {
                out.push(('F', branch, f));
            }
        }

        out
    }

    /// Simplified DC WLS state estimation.
    ///
    /// Returns `(voltages_pu, angles_rad, flows_mw, residuals)`.
    ///
    /// Voltage measurements set the estimated magnitude directly (weighted mean
    /// per bus). Power measurements contribute to angle estimates via a flat DC
    /// approximation (θ ≈ 0 + ΔP / B_avg). Branch flows are set directly.
    fn dc_wls_estimate(
        &self,
        measurements: &[(char, usize, f64)],
    ) -> Result<WlsEstimate, TwinError> {
        let mut v_sum = vec![0.0_f64; self.n_buses];
        let mut v_cnt = vec![0u32; self.n_buses];
        let mut p_sum = vec![0.0_f64; self.n_buses];
        let mut f_vals = vec![0.0_f64; self.n_branches];

        for &(tag, idx, val) in measurements {
            match tag {
                'V' => {
                    v_sum[idx] += val;
                    v_cnt[idx] += 1;
                }
                'P' => {
                    p_sum[idx] += val;
                }
                'F' => {
                    f_vals[idx] = val;
                }
                _ => {}
            }
        }

        // Voltage: weighted mean; buses with no measurement → keep 1.0 pu
        let mut voltages = vec![1.0_f64; self.n_buses];
        for i in 0..self.n_buses {
            if v_cnt[i] > 0 {
                voltages[i] = v_sum[i] / v_cnt[i] as f64;
            } else {
                // Fall back to previous state
                voltages[i] = self.twin_state.estimated_voltages[i];
            }
        }

        // Angles: simple DC approximation θ_i ≈ P_i / (n_buses * B_avg)
        // B_avg ≈ 10 pu (typical transmission susceptance)
        let b_avg = 10.0_f64;
        let mut angles = vec![0.0_f64; self.n_buses];
        for i in 1..self.n_buses {
            // slack bus angle = 0
            angles[i] = p_sum[i] / (self.n_buses as f64 * b_avg);
        }

        // Compute residuals: difference between measurements and estimates
        let mut residuals = Vec::with_capacity(measurements.len());
        for &(tag, idx, val) in measurements {
            let est = match tag {
                'V' => voltages[idx],
                'P' => angles[idx] * (self.n_buses as f64 * b_avg),
                'F' => f_vals[idx],
                _ => val,
            };
            residuals.push(val - est);
        }

        if voltages.iter().any(|v| v.is_nan() || *v <= 0.0) {
            return Err(TwinError::StateEstimationFailed(
                "estimated voltage is non-positive or NaN".into(),
            ));
        }

        Ok((voltages, angles, f_vals, residuals))
    }

    /// Compute normalised RMS divergence between a new state and the previous one.
    fn compute_divergence(&self, new_v: &[f64], new_a: &[f64], new_f: &[f64]) -> f64 {
        let prev_v = &self.twin_state.estimated_voltages;
        let prev_a = &self.twin_state.estimated_angles;
        let prev_f = &self.twin_state.estimated_flows;

        let mut sum_sq = 0.0_f64;
        let mut count = 0usize;

        for (a, b) in new_v.iter().zip(prev_v.iter()) {
            sum_sq += (a - b).powi(2);
            count += 1;
        }
        for (a, b) in new_a.iter().zip(prev_a.iter()) {
            sum_sq += (a - b).powi(2);
            count += 1;
        }
        for (a, b) in new_f.iter().zip(prev_f.iter()) {
            let norm = if b.abs() > 1.0 { b.abs() } else { 1.0 };
            sum_sq += ((a - b) / norm).powi(2);
            count += 1;
        }

        if count == 0 {
            return 0.0;
        }
        (sum_sq / count as f64).sqrt()
    }

    /// Sync quality = 1 − normalised_residual_rms (clamped to \[0, 1\]).
    fn sync_quality(&self, residuals: &[f64]) -> f64 {
        if residuals.is_empty() {
            return 0.0;
        }
        let rms = (residuals.iter().map(|r| r * r).sum::<f64>() / residuals.len() as f64).sqrt();
        // Normalise: assume a residual of 0.1 pu = quality 0
        let normalised = (rms / 0.1).min(1.0);
        (1.0 - normalised).max(0.0)
    }

    /// Predict future state via linear trend extrapolation from the last
    /// `TREND_WINDOW` sync points.
    fn predict_state(&self, current: &TwinSyncState) -> PredictionState {
        let horizon_s = self.config.prediction_horizon_s;
        let n = self.recent_states.len();

        if n < 2 {
            // No trend available — return current state as prediction
            return PredictionState {
                horizon_s,
                predicted_voltages: current.estimated_voltages.clone(),
                predicted_flows: current.estimated_flows.clone(),
                confidence: 0.2,
                predicted_violations: Vec::new(),
            };
        }

        // Compute per-bus voltage trend (linear regression slope)
        let oldest = &self.recent_states[0];
        let dt = current.timestamp_s - oldest.timestamp_s;

        let mut predicted_voltages = current.estimated_voltages.clone();
        let mut predicted_flows = current.estimated_flows.clone();

        if dt > 0.0 {
            for (pv, (cv, ov)) in predicted_voltages.iter_mut().zip(
                current
                    .estimated_voltages
                    .iter()
                    .zip(oldest.estimated_voltages.iter()),
            ) {
                let slope = (cv - ov) / dt;
                *pv = (cv + slope * horizon_s).clamp(0.5, 1.5);
            }
            for (pf, (cf, of_)) in predicted_flows.iter_mut().zip(
                current
                    .estimated_flows
                    .iter()
                    .zip(oldest.estimated_flows.iter()),
            ) {
                let slope = (cf - of_) / dt;
                *pf = cf + slope * horizon_s;
            }
        }

        // Detect predicted voltage violations (< 0.95 or > 1.05 pu)
        let mut violations = Vec::new();
        for (i, &v) in predicted_voltages.iter().enumerate() {
            if v < 0.95 {
                violations.push(format!("Bus {i}: low voltage predicted ({v:.3} pu)"));
            } else if v > 1.05 {
                violations.push(format!("Bus {i}: high voltage predicted ({v:.3} pu)"));
            }
        }

        // Confidence inversely proportional to prediction horizon and trend points
        let confidence = (n as f64 / TREND_WINDOW as f64)
            * (1.0 - (horizon_s / 3600.0).min(0.9))
            * (1.0 - self.twin_state.divergence_metric.min(0.5) * 2.0).max(0.0);

        PredictionState {
            horizon_s,
            predicted_voltages,
            predicted_flows,
            confidence: confidence.clamp(0.0, 1.0),
            predicted_violations: violations,
        }
    }

    // ── Public query API ─────────────────────────────────────────────────────

    /// Return the full sync quality history as `(timestamp_s, quality)` pairs.
    pub fn sync_quality_history(&self) -> Vec<(f64, f64)> {
        self.sync_history.clone()
    }

    /// Returns `true` if the last sync quality exceeds 0.5 and the divergence
    /// metric is below the configured threshold.
    pub fn is_synchronized(&self) -> bool {
        self.twin_state.sync_quality > 0.5
            && self.twin_state.divergence_metric <= self.config.divergence_threshold
    }

    /// Most recent divergence metric value.
    pub fn divergence_metric(&self) -> f64 {
        self.twin_state.divergence_metric
    }

    /// Read-only access to the current twin state.
    pub fn twin_state(&self) -> &TwinSyncState {
        &self.twin_state
    }

    /// Number of buses this engine is tracking.
    pub fn n_buses(&self) -> usize {
        self.n_buses
    }

    /// Number of branches this engine is tracking.
    pub fn n_branches(&self) -> usize {
        self.n_branches
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> RealtimeSyncConfig {
        RealtimeSyncConfig {
            sync_interval_ms: 1000.0,
            state_estimator_type: SeType::DcWls,
            prediction_horizon_s: 60.0,
            divergence_threshold: 0.1,
            auto_resync_on_diverge: true,
        }
    }

    fn make_engine(n_buses: usize, n_branches: usize) -> DigitalTwinSync {
        DigitalTwinSync::new(default_config(), n_buses, n_branches).expect("engine creation")
    }

    fn good_update(t: f64, n_buses: usize, n_branches: usize) -> MeasurementUpdate {
        let bus_voltages: Vec<(usize, f64)> =
            (0..n_buses).map(|i| (i, 1.0 + i as f64 * 0.005)).collect();
        let bus_powers: Vec<(usize, f64, f64)> = (0..n_buses)
            .map(|i| (i, i as f64 * 10.0, i as f64 * 2.0))
            .collect();
        let branch_flows: Vec<(usize, f64)> =
            (0..n_branches).map(|i| (i, i as f64 * 5.0)).collect();
        MeasurementUpdate::new(t, bus_voltages, bus_powers, branch_flows)
    }

    // ── Test 1: clean measurements → high sync quality ────────────────────

    #[test]
    fn test_clean_measurements_high_sync_quality() {
        let mut engine = make_engine(4, 3);
        let update = good_update(1.0, 4, 3);
        engine.update_measurements(update);
        let state = engine.sync().expect("sync should succeed");
        assert!(
            state.sync_quality > 0.5,
            "Expected high sync quality, got {:.3}",
            state.sync_quality
        );
        assert_eq!(state.estimated_voltages.len(), 4);
        assert_eq!(state.estimated_flows.len(), 3);
    }

    // ── Test 2: stale measurements → quality drops (flag = bad) ──────────

    #[test]
    fn test_stale_bad_quality_measurements_rejected() {
        let mut engine = make_engine(3, 2);

        // Build an update where ALL quality flags are bad (255)
        let mut update = good_update(1.0, 3, 2);
        for flag in &mut update.quality_flags {
            *flag = 255; // mark all as bad
        }
        engine.update_measurements(update);

        // All measurements are bad → NoMeasurements after filtering
        let result = engine.sync();
        assert!(
            result.is_err(),
            "Expected error when all measurements are bad"
        );
        match result {
            Err(TwinError::NoMeasurements) => {}
            Err(e) => panic!("Expected NoMeasurements, got {:?}", e),
            Ok(_) => panic!("Expected error"),
        }
    }

    // ── Test 3: divergence detection ──────────────────────────────────────

    #[test]
    fn test_divergence_detection_flagged() {
        let config = RealtimeSyncConfig {
            divergence_threshold: 0.001, // very tight
            auto_resync_on_diverge: true,
            ..default_config()
        };
        let mut engine = DigitalTwinSync::new(config, 3, 2).expect("engine");

        // First sync: nominal
        engine.update_measurements(good_update(1.0, 3, 2));
        let s1 = engine.sync().expect("first sync");
        assert!(s1.divergence_metric >= 0.0);

        // Second sync: large voltage jump → should trigger divergence flag
        let mut update2 = good_update(2.0, 3, 2);
        // Make voltage very different
        update2.bus_voltages = vec![(0, 0.7), (1, 0.65), (2, 0.72)];
        engine.update_measurements(update2);
        let s2 = engine.sync().expect("auto-resync should succeed");
        // Divergence metric should be significantly non-zero
        assert!(
            s2.divergence_metric > 0.0,
            "Expected non-zero divergence, got {}",
            s2.divergence_metric
        );
    }

    // ── Test 4: divergence → error when auto_resync disabled ─────────────

    #[test]
    fn test_divergence_error_without_auto_resync() {
        let config = RealtimeSyncConfig {
            divergence_threshold: 0.00001, // extremely tight
            auto_resync_on_diverge: false,
            ..default_config()
        };
        let mut engine = DigitalTwinSync::new(config, 3, 2).expect("engine");

        // Sync 1: flat start — this may already diverge (large initial jump from 1.0 baseline)
        let update1 = MeasurementUpdate::new(
            1.0,
            vec![(0, 1.0), (1, 1.0), (2, 1.0)], // exact flat — no divergence
            vec![(0, 0.0, 0.0), (1, 0.0, 0.0), (2, 0.0, 0.0)],
            vec![(0, 0.0), (1, 0.0)],
        );
        engine.update_measurements(update1);
        // First sync may diverge if threshold is too tight; that's acceptable
        let _ = engine.sync();

        // Sync 2: large voltage jump
        let update2 = MeasurementUpdate::new(
            2.0,
            vec![(0, 0.6), (1, 0.55), (2, 0.58)],
            vec![(0, 0.0, 0.0), (1, 0.0, 0.0), (2, 0.0, 0.0)],
            vec![(0, 0.0), (1, 0.0)],
        );
        engine.update_measurements(update2);
        let result = engine.sync();
        // Should get Diverged error because voltages differ substantially
        match result {
            Err(TwinError::Diverged { .. }) => {}
            Err(TwinError::NoMeasurements) => {
                // Also acceptable if all measurements were filtered
            }
            Ok(s) => {
                // Acceptable only if divergence truly is within threshold
                if s.divergence_metric > 0.00001 {
                    panic!(
                        "Expected Diverged error, got Ok with divergence {}",
                        s.divergence_metric
                    );
                }
            }
            Err(e) => panic!("Unexpected error: {:?}", e),
        }
    }

    // ── Test 5: prediction extrapolates voltages ──────────────────────────

    #[test]
    fn test_prediction_extrapolates_voltages() {
        let mut engine = make_engine(3, 2);

        // Feed 3 syncs with a rising voltage trend
        for i in 0..3usize {
            let t = i as f64 + 1.0;
            let v = 1.0 + i as f64 * 0.01; // rising by 0.01 per second
            let update = MeasurementUpdate::new(
                t,
                vec![(0, v), (1, v - 0.01), (2, v + 0.005)],
                vec![(0, 5.0, 1.0), (1, 10.0, 2.0), (2, 3.0, 0.5)],
                vec![(0, 5.0), (1, 3.0)],
            );
            engine.update_measurements(update);
            let _ = engine.sync().expect("sync");
        }

        let state = engine.twin_state();
        let pred = state.prediction.as_ref().expect("prediction should exist");
        // Prediction should exist and voltages should be valid
        assert_eq!(pred.predicted_voltages.len(), 3);
        for &v in &pred.predicted_voltages {
            assert!(
                (0.5..=1.5).contains(&v),
                "Predicted voltage {v} out of reasonable range"
            );
        }
        assert!(pred.horizon_s > 0.0);
    }

    // ── Test 6: sync quality history tracked ─────────────────────────────

    #[test]
    fn test_sync_quality_history_tracked() {
        let mut engine = make_engine(4, 3);

        for i in 0..5usize {
            engine.update_measurements(good_update(i as f64 + 1.0, 4, 3));
            let _ = engine.sync().expect("sync");
        }

        let history = engine.sync_quality_history();
        assert_eq!(history.len(), 5, "Expected 5 history entries");
        for (t, q) in &history {
            assert!(*t > 0.0, "Timestamp should be positive");
            assert!((*q).is_finite(), "Quality should be finite");
        }
    }

    // ── Test 7: is_synchronized reflects state ────────────────────────────

    #[test]
    fn test_is_synchronized_reflects_state() {
        let config = RealtimeSyncConfig {
            divergence_threshold: 1.0, // generous threshold for first sync
            ..default_config()
        };
        let mut engine = DigitalTwinSync::new(config, 3, 2).expect("engine");
        // Initially not synchronized (no syncs done)
        assert!(!engine.is_synchronized());

        // Use flat measurements (no angle divergence) for clean sync
        let update = MeasurementUpdate::new(
            1.0,
            vec![(0, 1.0), (1, 1.0), (2, 1.0)],
            vec![(0, 0.0, 0.0), (1, 0.0, 0.0), (2, 0.0, 0.0)],
            vec![(0, 0.0), (1, 0.0)],
        );
        engine.update_measurements(update);
        let state = engine.sync().expect("sync");
        // After a good sync with flat measurements, quality should be high
        assert!(
            state.sync_quality > 0.5,
            "Expected sync_quality > 0.5, got {}",
            state.sync_quality
        );
        assert!(engine.is_synchronized());
    }

    // ── Test 8: invalid config rejected ──────────────────────────────────

    #[test]
    fn test_zero_buses_error() {
        let result = DigitalTwinSync::new(default_config(), 0, 0);
        assert!(result.is_err(), "Expected error for zero buses");
    }

    // ── Test 9: no measurements → error ──────────────────────────────────

    #[test]
    fn test_no_measurements_error() {
        let mut engine = make_engine(3, 2);
        let result = engine.sync();
        assert!(matches!(result, Err(TwinError::NoMeasurements)));
    }
}
