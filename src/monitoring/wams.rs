//! Wide-Area Monitoring System (WAMS) based on Phasor Measurement Units (PMUs).
//!
//! Implements real-time monitoring of:
//! - Angular stability index from GPS-synchronized voltage phasors
//! - Inter-area oscillation detection via AR-based Prony analysis
//! - Frequency coherency grouping via K-means clustering
//! - Voltage stability L-index proxy
//! - Alarm generation with severity classification
//!
//! # Reference
//! Phadke, A.G., Thorp, J.S. (2008). "Synchronized Phasor Measurements and Their
//! Applications". Springer.
use serde::{Deserialize, Serialize};
use thiserror::Error;

// ---------------------------------------------------------------------------
// Error type
// ---------------------------------------------------------------------------

/// Errors from the WAMS analysis pipeline.
#[derive(Debug, Error)]
pub enum WamsError {
    /// Fewer than the minimum required good-quality PMU readings.
    #[error("insufficient PMU readings: need at least {0}")]
    InsufficientData(usize),
    /// Configuration parameter is invalid.
    #[error("invalid WAMS configuration: {0}")]
    InvalidConfig(String),
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// WAMS system configuration parameters.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WamsConfig {
    /// PMU data reporting rate \[Hz\] (typically 25, 50, or 60 frames per second).
    pub pmu_reporting_rate_hz: f64,
    /// Expected communication latency \[ms\].
    pub latency_ms: f64,
    /// GPS time synchronisation accuracy \[μs\] (typically < 1 μs).
    pub gps_sync_accuracy_us: f64,
    /// Normalised residual threshold for bad-data detection.
    pub bad_data_threshold: f64,
    /// Prony analysis window length \[s\].
    pub mode_meter_window_s: f64,
}

impl Default for WamsConfig {
    fn default() -> Self {
        Self {
            pmu_reporting_rate_hz: 50.0,
            latency_ms: 100.0,
            gps_sync_accuracy_us: 1.0,
            bad_data_threshold: 3.0,
            mode_meter_window_s: 10.0,
        }
    }
}

// ---------------------------------------------------------------------------
// PMU reading
// ---------------------------------------------------------------------------

/// A single GPS-synchronised measurement frame from one PMU.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PmuReading {
    /// Unique PMU identifier.
    pub pmu_id: usize,
    /// Index of the bus where this PMU is installed.
    pub bus_idx: usize,
    /// Absolute timestamp \[s\] (GPS epoch or POSIX time).
    pub timestamp_s: f64,
    /// Voltage magnitude \[pu\].
    pub voltage_magnitude_pu: f64,
    /// GPS-synchronised voltage angle \[deg\].
    pub voltage_angle_deg: f64,
    /// Instantaneous frequency measurement \[Hz\].
    pub frequency_hz: f64,
    /// Rate of change of frequency (ROCOF) \[Hz/s\].
    pub rocof_hz_per_s: f64,
    /// Active power flow \[MW\].
    pub p_mw: f64,
    /// Reactive power flow \[MVAr\].
    pub q_mvar: f64,
    /// Quality flag: 0 = OK; any other value indicates a measurement fault.
    pub quality_flag: u8,
}

// ---------------------------------------------------------------------------
// Stability index
// ---------------------------------------------------------------------------

/// Angular stability trend indicator.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum AngleTrend {
    /// All ROCOFs < 0.1 \[Hz/s\] and angle spread < 5 \[deg\].
    Stable,
    /// Angle spread growing slowly; max ROCOF ∈ \[0.1, 1.0\] \[Hz/s\].
    IncreasingSlowly,
    /// Max ROCOF > 1.0 \[Hz/s\] — alarm condition.
    IncreasingFast,
    /// Inter-area oscillation detected; mode frequency 0.1–2.0 \[Hz\].
    Oscillating,
}

/// Summary angular-stability index computed from a snapshot of PMU readings.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AngularStabilityIndex {
    /// Maximum pairwise angle difference |θ_i − θ_j| \[deg\].
    pub max_angle_diff_deg: f64,
    /// Standard deviation of all bus voltage angles \[deg\] (angle spread).
    pub angle_spread_deg: f64,
    /// Bus pair (i, j) achieving the maximum angle difference.
    pub critical_pair: (usize, usize),
    /// Stability margin: `1 − |Δθ| / 90°` clamped to \[0, 1\].
    pub stability_margin: f64,
    /// Qualitative trend indicator.
    pub trend: AngleTrend,
}

// ---------------------------------------------------------------------------
// Inter-area mode
// ---------------------------------------------------------------------------

/// An inter-area oscillation mode identified by Prony analysis.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InterAreaMode {
    /// Mode frequency \[Hz\] (typical inter-area range: 0.1–2.0 \[Hz\]).
    pub frequency_hz: f64,
    /// Modal damping ratio (< 0.05 is alarm threshold).
    pub damping_ratio: f64,
    /// Indices of buses that participate in this mode.
    pub participating_buses: Vec<usize>,
    /// Normalised per-bus modal participation (same length as `participating_buses`).
    pub mode_shape: Vec<f64>,
}

// ---------------------------------------------------------------------------
// WAMS result
// ---------------------------------------------------------------------------

/// Comprehensive WAMS analysis result for one measurement snapshot.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WamsResult {
    /// Computed angular stability index.
    pub angular_stability: AngularStabilityIndex,
    /// Inter-area modes identified by Prony analysis.
    pub detected_modes: Vec<InterAreaMode>,
    /// Coherent generator groups identified by K-means on frequency.
    pub frequency_coherency_groups: Vec<Vec<usize>>,
    /// Minimum voltage magnitude across all good-quality buses (proxy L-index).
    pub voltage_stability_index: f64,
    /// Alarms generated by threshold checks.
    pub alarms: Vec<WamsAlarm>,
}

// ---------------------------------------------------------------------------
// Alarms
// ---------------------------------------------------------------------------

/// Alarm severity level.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum AlarmSeverity {
    /// Informational: operator awareness needed.
    Advisory,
    /// Warning: corrective action recommended.
    Alert,
    /// Critical: immediate action required.
    Emergency,
}

/// A single WAMS alarm event.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WamsAlarm {
    /// Timestamp at which the alarm was triggered \[s\].
    pub timestamp_s: f64,
    /// Alarm severity classification.
    pub severity: AlarmSeverity,
    /// Human-readable description of the alarm condition.
    pub description: String,
    /// Buses involved in or affected by this alarm.
    pub affected_buses: Vec<usize>,
}

// ---------------------------------------------------------------------------
// WamsMonitor
// ---------------------------------------------------------------------------

/// Wide-Area Monitoring System analyser.
///
/// Processes a snapshot of [`PmuReading`] frames and returns a [`WamsResult`]
/// containing stability indices, detected inter-area modes, coherency groups,
/// and any active alarms.
pub struct WamsMonitor {
    config: WamsConfig,
}

impl WamsMonitor {
    /// Construct a new `WamsMonitor` with the given configuration.
    pub fn new(config: WamsConfig) -> Self {
        Self { config }
    }

    /// Analyse a snapshot of PMU readings.
    ///
    /// Steps performed:
    /// 1. Filter out readings with non-zero `quality_flag`.
    /// 2. Compute [`AngularStabilityIndex`].
    /// 3. Prony analysis for inter-area modes (AR(2) per bus).
    /// 4. K-means coherency grouping on frequency measurements.
    /// 5. Voltage stability index (minimum voltage magnitude).
    /// 6. Generate alarms from threshold checks.
    ///
    /// Returns [`WamsError::InsufficientData`] when fewer than 2 good readings remain.
    pub fn analyze(&self, readings: &[PmuReading]) -> Result<WamsResult, WamsError> {
        if self.config.pmu_reporting_rate_hz <= 0.0 {
            return Err(WamsError::InvalidConfig(
                "pmu_reporting_rate_hz must be positive".to_string(),
            ));
        }

        // 1. Filter bad-quality readings
        let good: Vec<&PmuReading> = readings.iter().filter(|r| r.quality_flag == 0).collect();

        if good.len() < 2 {
            return Err(WamsError::InsufficientData(2));
        }

        // 2. Angular stability index
        let angular_stability = compute_angle_stability(&good);

        // 3. Prony / AR-based inter-area mode detection
        let detected_modes = detect_inter_area_modes(&good, self.config.pmu_reporting_rate_hz);

        // 4. K-means coherency grouping (k=2)
        let frequency_coherency_groups = kmeans_coherency(&good, 2);

        // 5. Voltage stability index: min voltage magnitude
        let voltage_stability_index = good
            .iter()
            .map(|r| r.voltage_magnitude_pu)
            .fold(f64::INFINITY, f64::min);
        let voltage_stability_index = if voltage_stability_index.is_infinite() {
            1.0
        } else {
            voltage_stability_index
        };

        // 6. Alarms
        let timestamp_s = good
            .iter()
            .map(|r| r.timestamp_s)
            .fold(f64::NEG_INFINITY, f64::max);
        let alarms = generate_alarms(
            &angular_stability,
            &detected_modes,
            voltage_stability_index,
            timestamp_s,
            &good,
        );

        Ok(WamsResult {
            angular_stability,
            detected_modes,
            frequency_coherency_groups,
            voltage_stability_index,
            alarms,
        })
    }
}

// ---------------------------------------------------------------------------
// Internal: angular stability
// ---------------------------------------------------------------------------

fn compute_angle_stability(good: &[&PmuReading]) -> AngularStabilityIndex {
    let angles: Vec<f64> = good.iter().map(|r| r.voltage_angle_deg).collect();
    let n = angles.len();

    // Max pairwise difference and critical pair
    let mut max_diff = 0.0_f64;
    let mut critical_pair = (good[0].bus_idx, good[0].bus_idx);
    for i in 0..n {
        for j in (i + 1)..n {
            let diff = (angles[i] - angles[j]).abs();
            if diff > max_diff {
                max_diff = diff;
                critical_pair = (good[i].bus_idx, good[j].bus_idx);
            }
        }
    }

    // Angle spread (std dev)
    let mean_angle = angles.iter().sum::<f64>() / n as f64;
    let variance = angles
        .iter()
        .map(|&a| (a - mean_angle).powi(2))
        .sum::<f64>()
        / n as f64;
    let angle_spread_deg = variance.sqrt();

    // Stability margin
    let stability_margin = (1.0 - max_diff / 90.0).clamp(0.0, 1.0);

    // Trend: use ROCOF
    let max_rocof = good
        .iter()
        .map(|r| r.rocof_hz_per_s.abs())
        .fold(0.0_f64, f64::max);

    let trend = if max_rocof > 1.0 {
        AngleTrend::IncreasingFast
    } else if max_rocof > 0.1 || angle_spread_deg >= 5.0 {
        AngleTrend::IncreasingSlowly
    } else {
        AngleTrend::Stable
    };

    AngularStabilityIndex {
        max_angle_diff_deg: max_diff,
        angle_spread_deg,
        critical_pair,
        stability_margin,
        trend,
    }
}

// ---------------------------------------------------------------------------
// Internal: Prony / AR(2) inter-area mode detection
// ---------------------------------------------------------------------------

/// Detect inter-area oscillation modes using AR(2) fit on per-bus angle signals.
///
/// For each bus with ≥ 3 readings, an AR(2) model is fitted to the angle time
/// series. The companion matrix eigenvalues give the damped-sinusoid poles.
/// Only poles whose frequency falls in the inter-area range 0.1–2.0 \[Hz\] are
/// retained.
fn detect_inter_area_modes(good: &[&PmuReading], rate_hz: f64) -> Vec<InterAreaMode> {
    let dt = 1.0 / rate_hz.max(1.0);

    // Group readings by bus_idx, sorted by timestamp
    let mut bus_angles: std::collections::HashMap<usize, Vec<(f64, f64)>> =
        std::collections::HashMap::new();
    for r in good {
        bus_angles
            .entry(r.bus_idx)
            .or_default()
            .push((r.timestamp_s, r.voltage_angle_deg));
    }
    for vals in bus_angles.values_mut() {
        vals.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    }

    let mut modes: Vec<InterAreaMode> = Vec::new();

    for (&bus, readings) in &bus_angles {
        if readings.len() < 3 {
            continue;
        }
        let signal: Vec<f64> = readings.iter().map(|&(_, a)| a).collect();
        if let Some((freq, damp)) = ar2_mode_extract(&signal, dt) {
            // Only accept inter-area range 0.1–2.0 Hz with non-trivial damping
            if (0.1..=2.0).contains(&freq) && damp.is_finite() {
                // Check if this mode already exists (within 0.1 Hz)
                let existing = modes
                    .iter_mut()
                    .find(|m| (m.frequency_hz - freq).abs() < 0.15);
                if let Some(m) = existing {
                    m.participating_buses.push(bus);
                    m.mode_shape.push(1.0); // will normalise later
                                            // Update damping as average
                    let n = m.participating_buses.len() as f64;
                    m.damping_ratio = (m.damping_ratio * (n - 1.0) + damp) / n;
                } else {
                    modes.push(InterAreaMode {
                        frequency_hz: freq,
                        damping_ratio: damp,
                        participating_buses: vec![bus],
                        mode_shape: vec![1.0],
                    });
                }
            }
        }
    }

    // Normalise mode shapes
    for mode in &mut modes {
        let mag: f64 = mode.mode_shape.iter().map(|&v| v * v).sum::<f64>().sqrt();
        if mag > 1e-12 {
            for v in &mut mode.mode_shape {
                *v /= mag;
            }
        }
    }

    modes
}

/// Fit AR(2) to a signal and extract the dominant damped-sinusoid pole.
///
/// Returns `(frequency_hz, damping_ratio)` or `None` if the signal is too
/// short or the model is degenerate.
fn ar2_mode_extract(signal: &[f64], dt: f64) -> Option<(f64, f64)> {
    if signal.len() < 4 {
        return None;
    }

    // Compute mean and centre
    let mean = signal.iter().sum::<f64>() / signal.len() as f64;
    let x: Vec<f64> = signal.iter().map(|&s| s - mean).collect();

    // Autocorrelations r[0], r[1], r[2]
    let n = x.len() as f64;
    let r0 = x.iter().map(|&v| v * v).sum::<f64>() / n;
    if r0 < 1e-30 {
        return None;
    }
    let r1 = x[1..]
        .iter()
        .zip(x.iter())
        .map(|(&a, &b)| a * b)
        .sum::<f64>()
        / n;
    let r2 = x[2..]
        .iter()
        .zip(x.iter())
        .map(|(&a, &b)| a * b)
        .sum::<f64>()
        / n;

    // Yule-Walker AR(2): [r0 r1; r1 r0] [phi1; phi2] = [r1; r2]
    let det = r0 * r0 - r1 * r1;
    if det.abs() < 1e-30 {
        return None;
    }
    let phi1 = (r1 * r0 - r2 * r1) / det;
    let phi2 = (r0 * r2 - r1 * r1) / det;

    // Roots of characteristic polynomial: z² - phi1*z - phi2 = 0
    // z = (phi1 ± sqrt(phi1² + 4*phi2)) / 2
    let disc = phi1 * phi1 + 4.0 * phi2;

    // Complex root branch (oscillatory)
    if disc < 0.0 {
        // z = phi1/2 ± j*sqrt(-disc)/2
        let re = phi1 / 2.0;
        let im = (-disc).sqrt() / 2.0;
        let r = (re * re + im * im).sqrt();
        let theta = im.atan2(re); // angle in radians per sample
        if r < 1e-12 || theta.abs() < 1e-12 {
            return None;
        }
        let freq = theta.abs() / (2.0 * std::f64::consts::PI * dt);
        // Damping ratio: ζ = -ln(r) / sqrt(ln²(r) + θ²)
        let ln_r = r.ln();
        let denom = (ln_r * ln_r + theta * theta).sqrt();
        let damping = if denom > 1e-12 { -ln_r / denom } else { 0.0 };
        Some((freq, damping))
    } else {
        // Real roots → no oscillatory mode
        None
    }
}

// ---------------------------------------------------------------------------
// Internal: K-means coherency grouping
// ---------------------------------------------------------------------------

/// Group buses into `k` coherent groups using K-means on frequency measurements.
///
/// Uses simple Lloyd's algorithm with LCG-seeded initialisation (min/max split).
fn kmeans_coherency(good: &[&PmuReading], k: usize) -> Vec<Vec<usize>> {
    if good.is_empty() || k == 0 {
        return vec![];
    }

    let freqs: Vec<f64> = good.iter().map(|r| r.frequency_hz).collect();
    let n = freqs.len();
    let actual_k = k.min(n);

    // Initialise centroids: spread between min and max
    let f_min = freqs.iter().cloned().fold(f64::INFINITY, f64::min);
    let f_max = freqs.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let mut centroids: Vec<f64> = (0..actual_k)
        .map(|i| {
            if actual_k == 1 {
                (f_min + f_max) / 2.0
            } else {
                f_min + (f_max - f_min) * (i as f64) / (actual_k - 1) as f64
            }
        })
        .collect();

    let mut assignments = vec![0usize; n];

    for _ in 0..30 {
        // Assignment step
        let mut changed = false;
        for (i, &f) in freqs.iter().enumerate() {
            let nearest = centroids
                .iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| {
                    (f - *a)
                        .abs()
                        .partial_cmp(&(f - *b).abs())
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .map(|(idx, _)| idx)
                .unwrap_or(0);
            if assignments[i] != nearest {
                assignments[i] = nearest;
                changed = true;
            }
        }

        // Update step
        let mut sums = vec![0.0f64; actual_k];
        let mut counts = vec![0usize; actual_k];
        for (i, &a) in assignments.iter().enumerate() {
            sums[a] += freqs[i];
            counts[a] += 1;
        }
        for c in 0..actual_k {
            if counts[c] > 0 {
                centroids[c] = sums[c] / counts[c] as f64;
            }
        }

        if !changed {
            break;
        }
    }

    // Build result groups as bus indices
    let mut groups: Vec<Vec<usize>> = vec![Vec::new(); actual_k];
    for (i, &a) in assignments.iter().enumerate() {
        groups[a].push(good[i].bus_idx);
    }
    // Remove empty groups
    groups.retain(|g| !g.is_empty());
    groups
}

// ---------------------------------------------------------------------------
// Internal: alarm generation
// ---------------------------------------------------------------------------

fn generate_alarms(
    stability: &AngularStabilityIndex,
    modes: &[InterAreaMode],
    v_min: f64,
    timestamp_s: f64,
    good: &[&PmuReading],
) -> Vec<WamsAlarm> {
    let mut alarms: Vec<WamsAlarm> = Vec::new();

    let affected: Vec<usize> = good.iter().map(|r| r.bus_idx).collect();

    // Angle spread thresholds
    if stability.max_angle_diff_deg > 60.0 {
        alarms.push(WamsAlarm {
            timestamp_s,
            severity: AlarmSeverity::Emergency,
            description: format!(
                "Critical angle spread: {:.1} deg > 60 deg — loss-of-synchronism risk",
                stability.max_angle_diff_deg
            ),
            affected_buses: vec![stability.critical_pair.0, stability.critical_pair.1],
        });
    } else if stability.max_angle_diff_deg > 45.0 {
        alarms.push(WamsAlarm {
            timestamp_s,
            severity: AlarmSeverity::Alert,
            description: format!(
                "High angle spread: {:.1} deg > 45 deg",
                stability.max_angle_diff_deg
            ),
            affected_buses: vec![stability.critical_pair.0, stability.critical_pair.1],
        });
    } else if stability.max_angle_diff_deg > 30.0 {
        alarms.push(WamsAlarm {
            timestamp_s,
            severity: AlarmSeverity::Advisory,
            description: format!(
                "Elevated angle spread: {:.1} deg > 30 deg",
                stability.max_angle_diff_deg
            ),
            affected_buses: vec![stability.critical_pair.0, stability.critical_pair.1],
        });
    }

    // Low damping modes
    for mode in modes {
        if mode.damping_ratio < 0.05 {
            alarms.push(WamsAlarm {
                timestamp_s,
                severity: AlarmSeverity::Alert,
                description: format!(
                    "Inter-area mode at {:.3} Hz has low damping ratio {:.3} (< 5%)",
                    mode.frequency_hz, mode.damping_ratio
                ),
                affected_buses: mode.participating_buses.clone(),
            });
        }
    }

    // Low voltage
    if v_min < 0.9 {
        alarms.push(WamsAlarm {
            timestamp_s,
            severity: AlarmSeverity::Advisory,
            description: format!("Low bus voltage: {:.3} pu < 0.9 pu", v_min),
            affected_buses: affected,
        });
    }

    alarms
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[allow(clippy::too_many_arguments)]
    fn make_reading(
        pmu_id: usize,
        bus_idx: usize,
        timestamp_s: f64,
        voltage_magnitude_pu: f64,
        voltage_angle_deg: f64,
        frequency_hz: f64,
        rocof_hz_per_s: f64,
        quality_flag: u8,
    ) -> PmuReading {
        PmuReading {
            pmu_id,
            bus_idx,
            timestamp_s,
            voltage_magnitude_pu,
            voltage_angle_deg,
            frequency_hz,
            rocof_hz_per_s,
            p_mw: 0.0,
            q_mvar: 0.0,
            quality_flag,
        }
    }

    fn flat_readings(n: usize) -> Vec<PmuReading> {
        (0..n)
            .map(|i| make_reading(i, i, i as f64 * 0.02, 1.0, 0.0, 50.0, 0.0, 0))
            .collect()
    }

    /// Test 1: Flat system — all angles ≈ 0, all frequencies ≈ 50 Hz → Stable, no Emergency.
    #[test]
    fn test_flat_system_stable() {
        let readings = flat_readings(5);
        let monitor = WamsMonitor::new(WamsConfig::default());
        let result = monitor.analyze(&readings).expect("analysis failed");

        assert_eq!(result.angular_stability.trend, AngleTrend::Stable);
        assert!(
            result.angular_stability.max_angle_diff_deg < 1.0,
            "max angle diff should be ~0"
        );
        let has_emergency = result
            .alarms
            .iter()
            .any(|a| a.severity == AlarmSeverity::Emergency);
        assert!(
            !has_emergency,
            "no Emergency alarm expected for flat system"
        );
    }

    /// Test 2: Growing angle spread → IncreasingFast alarm generated.
    #[test]
    fn test_growing_angle_spread_alarm() {
        let readings = vec![
            make_reading(0, 0, 0.0, 1.0, 0.0, 50.0, 2.5, 0),
            make_reading(1, 1, 0.02, 1.0, 35.0, 50.0, 2.5, 0),
            make_reading(2, 2, 0.04, 1.0, 70.0, 50.0, 2.5, 0),
        ];
        let monitor = WamsMonitor::new(WamsConfig::default());
        let result = monitor.analyze(&readings).expect("analysis failed");

        assert!(
            result.angular_stability.max_angle_diff_deg > 60.0,
            "expected angle diff > 60 deg"
        );
        let has_emergency = result
            .alarms
            .iter()
            .any(|a| a.severity == AlarmSeverity::Emergency);
        assert!(
            has_emergency,
            "Emergency alarm expected for spread > 60 deg"
        );
        assert_eq!(result.angular_stability.trend, AngleTrend::IncreasingFast);
    }

    /// Test 3: Synthetic oscillating signal at ~0.5 Hz → mode detected in 0.1–2 Hz range.
    ///
    /// Uses 200 samples (4 s at 50 Hz = 2 complete cycles) to ensure the AR(2) model
    /// has enough stationary data to identify the oscillatory pole.
    #[test]
    fn test_oscillation_mode_detected() {
        let rate = 50.0_f64;
        let dt = 1.0 / rate;
        let osc_freq = 0.5_f64; // Hz
        let n = 200usize; // 200 samples = 4 s = 2 full cycles → AR(2) detects mode

        // Generate repeated readings from one bus with sinusoidal angle variation
        let readings: Vec<PmuReading> = (0..n)
            .map(|i| {
                let t = i as f64 * dt;
                let angle = 5.0 * (2.0 * std::f64::consts::PI * osc_freq * t).sin();
                make_reading(0, 0, t, 1.0, angle, 50.0, 0.05, 0)
            })
            .collect();

        // Add a second bus so we have ≥ 2 good readings overall
        let mut all_readings = readings;
        all_readings.push(make_reading(1, 1, 0.0, 1.0, 0.0, 50.0, 0.0, 0));

        let cfg = WamsConfig {
            pmu_reporting_rate_hz: rate,
            ..WamsConfig::default()
        };
        let monitor = WamsMonitor::new(cfg);
        let result = monitor.analyze(&all_readings).expect("analysis failed");

        // We expect at least one mode in the inter-area range
        let has_mode = result
            .detected_modes
            .iter()
            .any(|m| m.frequency_hz >= 0.1 && m.frequency_hz <= 2.0);
        assert!(has_mode, "expected an inter-area mode to be detected");
    }

    /// Test 4: Bad-quality PMU data is filtered out.
    #[test]
    fn test_bad_quality_filtered() {
        // 3 bad readings + 2 good readings
        let readings = vec![
            make_reading(0, 0, 0.0, 1.0, 70.0, 50.0, 0.0, 1), // bad
            make_reading(1, 1, 0.0, 1.0, 80.0, 50.0, 0.0, 2), // bad
            make_reading(2, 2, 0.0, 1.0, 90.0, 50.0, 0.0, 3), // bad
            make_reading(3, 3, 0.0, 1.0, 0.0, 50.0, 0.0, 0),  // good
            make_reading(4, 4, 0.0, 1.0, 1.0, 50.0, 0.0, 0),  // good
        ];
        let monitor = WamsMonitor::new(WamsConfig::default());
        let result = monitor.analyze(&readings).expect("analysis failed");

        // Only good readings used: angle diff should be ~1 deg, not 90 deg
        assert!(
            result.angular_stability.max_angle_diff_deg < 5.0,
            "bad readings must be filtered: max_angle_diff should be ~1 deg, got {:.2}",
            result.angular_stability.max_angle_diff_deg
        );
    }

    /// Test 5: Coherency grouping — two well-separated frequency groups identified.
    #[test]
    fn test_coherency_grouping_two_groups() {
        let readings = vec![
            make_reading(0, 0, 0.0, 1.0, 0.0, 49.5, 0.0, 0),
            make_reading(1, 1, 0.0, 1.0, 1.0, 49.5, 0.0, 0),
            make_reading(2, 2, 0.0, 1.0, 2.0, 50.5, 0.0, 0),
            make_reading(3, 3, 0.0, 1.0, 3.0, 50.5, 0.0, 0),
        ];
        let monitor = WamsMonitor::new(WamsConfig::default());
        let result = monitor.analyze(&readings).expect("analysis failed");

        assert_eq!(
            result.frequency_coherency_groups.len(),
            2,
            "expected 2 coherency groups, got {}",
            result.frequency_coherency_groups.len()
        );
        // Each group should have 2 buses
        for g in &result.frequency_coherency_groups {
            assert_eq!(g.len(), 2, "each group should have 2 buses");
        }
    }

    /// Test 6: Emergency alarm on angle spread > 60°.
    #[test]
    fn test_emergency_alarm_above_60_deg() {
        let readings = vec![
            make_reading(0, 0, 0.0, 1.0, 0.0, 50.0, 0.0, 0),
            make_reading(1, 1, 0.0, 1.0, 65.0, 50.0, 0.0, 0),
        ];
        let monitor = WamsMonitor::new(WamsConfig::default());
        let result = monitor.analyze(&readings).expect("analysis failed");

        let has_emergency = result
            .alarms
            .iter()
            .any(|a| a.severity == AlarmSeverity::Emergency);
        assert!(
            has_emergency,
            "Emergency alarm must be raised when max angle diff > 60 deg"
        );
    }
}
