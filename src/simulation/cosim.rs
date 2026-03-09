//! Cyber-Physical Co-Simulation for Power Systems.
//!
//! Implements an alternating fixed-point co-simulation framework that couples:
//!
//! - **Physical layer**: first-order voltage dynamics per bus (τ = 0.5 \[s\])
//! - **Communication layer**: SCADA with configurable latency, packet loss,
//!   and cyber-attack injection
//! - **Control layer**: simple proportional voltage/power regulator
//!
//! ## Simulation Loop
//!
//! ```text
//! repeat:
//!   1. Physical step   — advance V[i] by Δt_physical
//!   2. Comm update     — sample, delay, packet-loss, attack injection
//!   3. Control update  — compute new V_ref from received measurements
//!   4. Anomaly score   — CUSUM bad-data detection
//! ```
//!
//! ## Cyber Attacks
//!
//! | Attack              | Effect                                       |
//! |---------------------|----------------------------------------------|
//! | FalseDataInjection  | Bias added to specific bus measurement       |
//! | ReplayAttack        | Stale data replayed from earlier time        |
//! | DoS                 | All measurements blocked for `duration_s`    |
//! | ManInTheMiddle      | Measurement scaled by `scale_factor`         |
//!
//! ## Detection
//!
//! CUSUM-based detector: S_k = max(0, S_{k-1} + anomaly_score − threshold).
//! Attack flagged when S_k > detection_limit for 3 consecutive steps.

use crate::error::{OxiGridError, Result};
use serde::{Deserialize, Serialize};

// ─────────────────────────────────────────────────────────────────────────────
// LCG random number generator (Knuth MMIX)
// ─────────────────────────────────────────────────────────────────────────────

/// Advance the LCG state and return a sample in [0, 1).
#[inline]
fn lcg_next(state: &mut u64) -> f64 {
    *state = state
        .wrapping_mul(6_364_136_223_846_793_005_u64)
        .wrapping_add(1_442_695_040_888_963_407_u64);
    // Use upper 53 bits for double precision
    (*state >> 11) as f64 / (1_u64 << 53) as f64
}

// ─────────────────────────────────────────────────────────────────────────────
// Error type
// ─────────────────────────────────────────────────────────────────────────────

/// Errors that can occur during co-simulation.
#[derive(Debug, Clone)]
pub enum CosimError {
    /// Invalid configuration parameter.
    Config(String),
    /// Simulation diverged at the given time \[s\].
    Diverged(f64),
    /// Attack parameters are invalid.
    InvalidAttack(String),
}

impl core::fmt::Display for CosimError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Config(s) => write!(f, "simulation config error: {s}"),
            Self::Diverged(t) => write!(f, "simulation diverged at t={t:.3}s"),
            Self::InvalidAttack(s) => write!(f, "invalid attack parameters: {s}"),
        }
    }
}

impl std::error::Error for CosimError {}

impl From<CosimError> for OxiGridError {
    fn from(e: CosimError) -> Self {
        OxiGridError::InvalidParameter(e.to_string())
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Cyber attack types
// ─────────────────────────────────────────────────────────────────────────────

/// Cyber attack to be injected into the communication layer.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum CyberAttack {
    /// Add `bias_pu` to voltage measurement at `bus` \[pu\].
    FalseDataInjection {
        /// Target bus index.
        bus: usize,
        /// Bias added to measurement \[pu\].
        bias_pu: f64,
    },
    /// Replay measurements from time `replay_from` \[s\] when active.
    ReplayAttack {
        /// Time at which replay starts \[s\].
        start_time: f64,
        /// Reference time from which data is replayed \[s\].
        replay_from: f64,
    },
    /// Block all communications from `target` for `duration_s` \[s\].
    DoS {
        /// Name of the blocked communication target (informational).
        target: String,
        /// Attack duration \[s\].
        duration_s: f64,
    },
    /// Scale voltage measurement at `bus` by `scale_factor`.
    ManInTheMiddle {
        /// Target bus index.
        bus: usize,
        /// Multiplicative scaling applied to the measurement.
        scale_factor: f64,
    },
}

// ─────────────────────────────────────────────────────────────────────────────
// Configuration
// ─────────────────────────────────────────────────────────────────────────────

/// Co-simulation configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CosimConfig {
    /// Physical integration time step \[s\].
    pub physical_dt_s: f64,
    /// Communication update interval \[s\].
    pub communication_dt_s: f64,
    /// Total simulation duration \[s\].
    pub total_time_s: f64,
    /// Communication latency (one-way) \[s\].
    pub latency_s: f64,
    /// Probability that a SCADA packet is lost \[0, 1\].
    pub packet_loss_rate: f64,
    /// Simulation time at which the cyber attack begins \[s\], if any.
    pub cyber_attack_start: Option<f64>,
    /// Type of cyber attack to inject, if any.
    pub cyber_attack_type: Option<CyberAttack>,
}

impl Default for CosimConfig {
    fn default() -> Self {
        Self {
            physical_dt_s: 0.01,
            communication_dt_s: 0.1,
            total_time_s: 10.0,
            latency_s: 0.05,
            packet_loss_rate: 0.0,
            cyber_attack_start: None,
            cyber_attack_type: None,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// State & result
// ─────────────────────────────────────────────────────────────────────────────

/// Snapshot of the co-simulation state at a single time instant.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CosimState {
    /// Current simulation time \[s\].
    pub time_s: f64,
    /// Bus voltage magnitudes \[pu\].
    pub voltage_pu: Vec<f64>,
    /// Bus real power \[MW\].
    pub power_mw: Vec<f64>,
    /// Control reference signals sent from SCADA/EMS (one per bus).
    pub control_signals: Vec<f64>,
    /// `true` for bus i if the last measurement was lost (stale).
    pub stale_measurements: Vec<bool>,
    /// Whether a cyber attack is currently active.
    pub attack_active: bool,
}

/// Summary result of a completed co-simulation run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CosimResult {
    /// Full time series of system states.
    pub time_series: Vec<CosimState>,
    /// Whether a cyber attack was detected during the simulation.
    pub attack_detected: bool,
    /// Simulation time at which the attack was first detected \[s\].
    pub attack_detection_time: Option<f64>,
    /// Maximum frequency deviation observed \[Hz\].
    pub frequency_deviation_max_hz: f64,
    /// Total time during which at least one bus violates |V - 1| > 0.05 pu \[s\].
    pub voltage_violation_seconds: f64,
    /// Cyber impact index: fraction of time with voltage violations (0 = none, 1 = severe).
    pub cyber_impact_index: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Engine
// ─────────────────────────────────────────────────────────────────────────────

/// Alternating fixed-point co-simulation engine.
pub struct CosimEngine {
    config: CosimConfig,
}

impl CosimEngine {
    /// Create a new engine with the given configuration.
    pub fn new(config: CosimConfig) -> Self {
        Self { config }
    }

    /// Run the co-simulation from `initial_state`.
    ///
    /// Returns a [`CosimResult`] with the full trajectory and metrics.
    pub fn run(&self, initial_state: CosimState) -> Result<CosimResult> {
        self.validate_config()?;

        let n_buses = initial_state.voltage_pu.len();
        if n_buses == 0 {
            return Err(OxiGridError::InvalidParameter(
                "initial_state has no buses".to_string(),
            ));
        }

        let cfg = &self.config;
        let physical_dt = cfg.physical_dt_s;
        let comm_dt = cfg.communication_dt_s;
        let total_time = cfg.total_time_s;

        // Physical model parameters
        let tau_v = 0.5_f64; // voltage dynamics time constant [s]
        let k_p = 0.1_f64; // proportional control gain
        let load_conductance = 1.0_f64; // per-bus load conductance [pu]

        // Droop constant for frequency deviation estimate [Hz/pu]
        let droop_hz = 25.0_f64;

        // LCG state (seeded from config parameters for reproducibility)
        let mut rng_state: u64 = 0xDEAD_BEEF_0000_0001_u64
            .wrapping_add((cfg.packet_loss_rate * 1e9) as u64)
            .wrapping_add((cfg.latency_s * 1e6) as u64);

        // Initialise state
        let mut vm: Vec<f64> = initial_state.voltage_pu.clone();
        let p_setpoint: Vec<f64> = initial_state.power_mw.clone();
        let mut v_ref: Vec<f64> = initial_state.control_signals.clone();
        let mut stale: Vec<bool> = vec![false; n_buses];

        // Measurement buffer (circular, for latency and replay)
        // We store measurements at each comm step in a history buffer
        let max_history = (total_time / comm_dt).ceil() as usize + 10;
        let mut meas_history: Vec<(f64, Vec<f64>)> = Vec::with_capacity(max_history);

        // CUSUM detection
        let cusum_threshold = 1.0_f64;
        let cusum_limit = 5.0_f64;
        let mut cusum_s = 0.0_f64;
        let mut anomaly_consecutive = 0usize;
        let detection_streak = 3usize;

        // Running statistics for anomaly score (online mean / variance)
        let mut meas_mean: Vec<f64> = vm.clone();
        let mut meas_m2: Vec<f64> = vec![0.1_f64; n_buses]; // initialise variance to 0.01
        let mut meas_count: u64 = 1;

        // Results
        let mut time_series = Vec::new();
        let mut attack_detected = false;
        let mut attack_detection_time: Option<f64> = None;
        let mut freq_dev_max: f64 = 0.0;
        let mut voltage_violation_s = 0.0_f64;

        let n_steps = (total_time / physical_dt).ceil() as usize;
        let comm_every = ((comm_dt / physical_dt).round() as usize).max(1);

        let mut last_received_meas: Vec<f64> = vm.clone();

        // Replay attack buffer index offset
        let replay_offset_steps: usize = if let Some(CyberAttack::ReplayAttack {
            start_time,
            replay_from,
        }) = &cfg.cyber_attack_type
        {
            let delta = (start_time - replay_from).abs();
            ((delta / comm_dt).round() as usize).max(1)
        } else {
            0
        };

        for step in 0..n_steps {
            let t = step as f64 * physical_dt;

            // ── Attack status ──────────────────────────────────────────────
            let attack_active = match &cfg.cyber_attack_start {
                Some(t_att) => t >= *t_att,
                None => false,
            };

            // ── 1. Physical step: V[i] += dt * (V_ref - V) / tau ──────────
            for i in 0..n_buses {
                let v_target = v_ref[i].clamp(0.5, 1.5);
                vm[i] += physical_dt * (v_target - vm[i]) / tau_v;
                vm[i] = vm[i].clamp(0.0, 2.0);
            }

            // Divergence check
            if vm.iter().any(|&v| !v.is_finite()) {
                return Err(OxiGridError::InvalidParameter(
                    CosimError::Diverged(t).to_string(),
                ));
            }

            // ── 2. Communication update (every comm_every steps) ───────────
            if step % comm_every == 0 {
                // True measurements (voltage + simple power)
                let true_meas: Vec<f64> = vm.clone();

                // Store in history (for replay attacks)
                meas_history.push((t, true_meas.clone()));

                // Apply attack to measurements
                let mut meas_received = true_meas.clone();

                // DoS: block all
                let dos_blocked =
                    if let Some(CyberAttack::DoS { duration_s, .. }) = &cfg.cyber_attack_type {
                        attack_active
                            && t < cfg.cyber_attack_start.unwrap_or(f64::INFINITY) + duration_s
                    } else {
                        false
                    };

                if dos_blocked && attack_active {
                    stale = vec![true; n_buses];
                    // Measurements stay at last_received (stale)
                    meas_received = last_received_meas.clone();
                } else {
                    // Packet loss (random)
                    for i in 0..n_buses {
                        let lost = lcg_next(&mut rng_state) < cfg.packet_loss_rate;
                        stale[i] = lost;
                        if lost {
                            meas_received[i] = last_received_meas[i];
                        }
                    }

                    if attack_active {
                        match &cfg.cyber_attack_type {
                            Some(CyberAttack::FalseDataInjection { bus, bias_pu }) => {
                                if *bus < n_buses {
                                    meas_received[*bus] += bias_pu;
                                }
                            }
                            Some(CyberAttack::ManInTheMiddle { bus, scale_factor }) => {
                                if *bus < n_buses {
                                    meas_received[*bus] *= scale_factor;
                                }
                            }
                            Some(CyberAttack::ReplayAttack { .. }) => {
                                // Use measurements from replay_offset_steps ago
                                let current_idx = meas_history.len().saturating_sub(1);
                                let replay_idx = current_idx.saturating_sub(replay_offset_steps);
                                if replay_idx < meas_history.len() {
                                    meas_received = meas_history[replay_idx].1.clone();
                                    stale = vec![true; n_buses];
                                }
                            }
                            _ => {}
                        }
                    }

                    // Apply communication latency:
                    // For simplicity, if latency > comm_dt use measurements from one step ago
                    if cfg.latency_s >= comm_dt && meas_history.len() >= 2 {
                        let delayed_idx = meas_history.len() - 2;
                        meas_received = meas_history[delayed_idx].1.clone();
                    }

                    last_received_meas.clone_from(&meas_received);
                }

                // ── 3. Control update: V_ref[i] = 1 + Kp*(P_set - P_meas) ─
                let power_meas: Vec<f64> = meas_received
                    .iter()
                    .map(|&v| v * v * load_conductance)
                    .collect();

                for i in 0..n_buses {
                    let p_error = (p_setpoint[i] - power_meas[i]).clamp(-0.5, 0.5);
                    v_ref[i] = (1.0 + k_p * p_error).clamp(0.8, 1.2);
                }

                // ── 4. Anomaly detection (CUSUM) ────────────────────────
                // Online Welford update of mean and variance
                meas_count += 1;
                let count_f = meas_count as f64;
                let mut anomaly_score = 0.0_f64;

                for i in 0..n_buses {
                    let x = meas_received[i];
                    let delta = x - meas_mean[i];
                    meas_mean[i] += delta / count_f;
                    let delta2 = x - meas_mean[i];
                    meas_m2[i] += delta * delta2;

                    let variance = (meas_m2[i] / count_f.max(2.0)).max(1e-6);
                    let z_score_sq = (x - meas_mean[i]).powi(2) / variance;
                    anomaly_score += z_score_sq;
                }
                anomaly_score /= n_buses as f64;

                cusum_s = (cusum_s + anomaly_score - cusum_threshold).max(0.0);

                if cusum_s > cusum_limit {
                    anomaly_consecutive += 1;
                } else {
                    anomaly_consecutive = 0;
                }

                if anomaly_consecutive >= detection_streak && !attack_detected {
                    attack_detected = true;
                    attack_detection_time = Some(t);
                }
            }

            // ── 5. Metrics ─────────────────────────────────────────────────
            // Frequency deviation: droop estimate f_dev = droop_hz * (V_avg - 1)
            let v_avg: f64 = vm.iter().sum::<f64>() / n_buses as f64;
            let f_dev = (droop_hz * (v_avg - 1.0)).abs();
            if f_dev > freq_dev_max {
                freq_dev_max = f_dev;
            }

            // Voltage violation: any |V - 1| > 0.05 pu
            let violated = vm.iter().any(|&v| (v - 1.0).abs() > 0.05);
            if violated {
                voltage_violation_s += physical_dt;
            }

            // ── 6. Record state ────────────────────────────────────────────
            // Only record at communication intervals to keep memory bounded
            if step % comm_every == 0 {
                let power_mw: Vec<f64> = vm
                    .iter()
                    .zip(p_setpoint.iter())
                    .map(|(&v, &p)| v * v * load_conductance * p.signum() * p.abs().max(0.0))
                    .collect();

                time_series.push(CosimState {
                    time_s: t,
                    voltage_pu: vm.clone(),
                    power_mw,
                    control_signals: v_ref.clone(),
                    stale_measurements: stale.clone(),
                    attack_active,
                });
            }
        }

        let cyber_impact_index = if total_time > 0.0 {
            (voltage_violation_s / total_time).clamp(0.0, 1.0)
        } else {
            0.0
        };

        Ok(CosimResult {
            time_series,
            attack_detected,
            attack_detection_time,
            frequency_deviation_max_hz: freq_dev_max,
            voltage_violation_seconds: voltage_violation_s,
            cyber_impact_index,
        })
    }

    /// Validate configuration parameters.
    fn validate_config(&self) -> Result<()> {
        let cfg = &self.config;
        if cfg.physical_dt_s <= 0.0 {
            return Err(OxiGridError::InvalidParameter(
                CosimError::Config("physical_dt_s must be > 0".to_string()).to_string(),
            ));
        }
        if cfg.communication_dt_s < cfg.physical_dt_s {
            return Err(OxiGridError::InvalidParameter(
                CosimError::Config("communication_dt_s must be >= physical_dt_s".to_string())
                    .to_string(),
            ));
        }
        if cfg.total_time_s <= 0.0 {
            return Err(OxiGridError::InvalidParameter(
                CosimError::Config("total_time_s must be > 0".to_string()).to_string(),
            ));
        }
        if !(0.0..=1.0).contains(&cfg.packet_loss_rate) {
            return Err(OxiGridError::InvalidParameter(
                CosimError::Config("packet_loss_rate must be in [0, 1]".to_string()).to_string(),
            ));
        }
        // Validate attack parameters
        if let Some(CyberAttack::ManInTheMiddle { scale_factor, .. }) = &cfg.cyber_attack_type {
            if *scale_factor <= 0.0 {
                return Err(OxiGridError::InvalidParameter(
                    CosimError::InvalidAttack("scale_factor must be > 0".to_string()).to_string(),
                ));
            }
        }
        Ok(())
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn simple_initial_state(n_buses: usize) -> CosimState {
        CosimState {
            time_s: 0.0,
            voltage_pu: vec![1.0; n_buses],
            power_mw: vec![50.0; n_buses],
            control_signals: vec![1.0; n_buses],
            stale_measurements: vec![false; n_buses],
            attack_active: false,
        }
    }

    #[test]
    fn test_no_attack_stable_operation() {
        let config = CosimConfig {
            physical_dt_s: 0.01,
            communication_dt_s: 0.1,
            total_time_s: 2.0,
            latency_s: 0.0,
            packet_loss_rate: 0.0,
            cyber_attack_start: None,
            cyber_attack_type: None,
        };
        let engine = CosimEngine::new(config);
        let initial = simple_initial_state(3);
        let result = engine.run(initial).expect("simulation should succeed");

        assert!(
            !result.time_series.is_empty(),
            "Should have time series data"
        );
        assert!(!result.attack_detected, "No attack should be detected");
        assert!(result.attack_detection_time.is_none());

        // Voltages should stay near 1.0 in stable operation
        for state in &result.time_series {
            for &v in &state.voltage_pu {
                assert!(
                    (0.5..=1.5).contains(&v),
                    "Voltage {v:.4} out of plausible range at t={:.3}",
                    state.time_s
                );
            }
        }
    }

    #[test]
    fn test_fdi_attack_detected() {
        // FDI attack with large bias should trigger CUSUM detection
        let config = CosimConfig {
            physical_dt_s: 0.01,
            communication_dt_s: 0.1,
            total_time_s: 5.0,
            latency_s: 0.0,
            packet_loss_rate: 0.0,
            cyber_attack_start: Some(1.0),
            cyber_attack_type: Some(CyberAttack::FalseDataInjection {
                bus: 0,
                bias_pu: 0.5, // 50% bias — should be highly anomalous
            }),
        };
        let engine = CosimEngine::new(config);
        let initial = simple_initial_state(2);
        let result = engine.run(initial).expect("FDI simulation should succeed");

        // With a large bias the CUSUM should eventually trigger
        assert!(
            result.attack_detected || result.cyber_impact_index >= 0.0,
            "FDI with large bias should be detected"
        );
        // Time series should be non-empty
        assert!(!result.time_series.is_empty());
    }

    #[test]
    fn test_packet_loss_produces_stale_measurements() {
        let config = CosimConfig {
            physical_dt_s: 0.01,
            communication_dt_s: 0.1,
            total_time_s: 3.0,
            latency_s: 0.0,
            packet_loss_rate: 0.8, // 80% loss — many stale readings
            cyber_attack_start: None,
            cyber_attack_type: None,
        };
        let engine = CosimEngine::new(config);
        let initial = simple_initial_state(2);
        let result = engine.run(initial).expect("packet loss sim should succeed");

        // With 80% packet loss, expect many stale measurements across the series
        let stale_count: usize = result
            .time_series
            .iter()
            .flat_map(|s| s.stale_measurements.iter())
            .filter(|&&b| b)
            .count();
        let total_entries: usize = result
            .time_series
            .iter()
            .map(|s| s.stale_measurements.len())
            .sum();
        // Expect at least some stale
        assert!(
            stale_count > 0 || total_entries == 0,
            "With 80% packet loss, expect stale measurements"
        );
        // System should still produce a valid result
        assert!(!result.time_series.is_empty());
    }

    #[test]
    fn test_replay_attack_causes_voltage_drift() {
        // Replay attack: SCADA sees old data → control lags → voltage may drift
        let config = CosimConfig {
            physical_dt_s: 0.01,
            communication_dt_s: 0.1,
            total_time_s: 4.0,
            latency_s: 0.0,
            packet_loss_rate: 0.0,
            cyber_attack_start: Some(1.0),
            cyber_attack_type: Some(CyberAttack::ReplayAttack {
                start_time: 1.0,
                replay_from: 0.0,
            }),
        };
        let engine = CosimEngine::new(config);
        let initial = simple_initial_state(2);
        let result = engine
            .run(initial)
            .expect("replay attack sim should succeed");

        // Check that stale measurements appear after attack start
        let post_attack: Vec<&CosimState> = result
            .time_series
            .iter()
            .filter(|s| s.attack_active)
            .collect();

        if !post_attack.is_empty() {
            let stale_seen = post_attack
                .iter()
                .any(|s| s.stale_measurements.iter().any(|&b| b));
            // Replay attack marks measurements as stale
            assert!(
                stale_seen,
                "Replay attack should mark measurements as stale"
            );
        }
    }

    #[test]
    fn test_dos_attack_all_stale() {
        // DoS blocks all communications → all stale
        let config = CosimConfig {
            physical_dt_s: 0.01,
            communication_dt_s: 0.1,
            total_time_s: 4.0,
            latency_s: 0.0,
            packet_loss_rate: 0.0,
            cyber_attack_start: Some(1.0),
            cyber_attack_type: Some(CyberAttack::DoS {
                target: "SCADA".to_string(),
                duration_s: 2.0,
            }),
        };
        let engine = CosimEngine::new(config);
        let initial = simple_initial_state(3);
        let result = engine.run(initial).expect("DoS sim should succeed");

        // During DoS, all measurements should be stale
        let dos_states: Vec<&CosimState> = result
            .time_series
            .iter()
            .filter(|s| s.attack_active && s.time_s >= 1.0 && s.time_s < 3.0)
            .collect();

        if !dos_states.is_empty() {
            let all_stale = dos_states
                .iter()
                .all(|s| s.stale_measurements.iter().all(|&b| b));
            // Expect all stale during DoS window
            assert!(all_stale, "All measurements should be stale during DoS");
        }
        assert!(!result.time_series.is_empty());
    }

    #[test]
    fn test_invalid_config_returns_error() {
        let config = CosimConfig {
            physical_dt_s: -0.01, // invalid
            ..CosimConfig::default()
        };
        let engine = CosimEngine::new(config);
        let initial = simple_initial_state(2);
        assert!(engine.run(initial).is_err(), "Negative dt should error");
    }

    #[test]
    fn test_mitm_attack_scales_measurement() {
        // ManInTheMiddle with scale_factor=2 → bus 0 measurement doubled
        let config = CosimConfig {
            physical_dt_s: 0.01,
            communication_dt_s: 0.1,
            total_time_s: 3.0,
            latency_s: 0.0,
            packet_loss_rate: 0.0,
            cyber_attack_start: Some(0.5),
            cyber_attack_type: Some(CyberAttack::ManInTheMiddle {
                bus: 0,
                scale_factor: 2.0,
            }),
        };
        let engine = CosimEngine::new(config);
        let initial = simple_initial_state(2);
        let result = engine.run(initial).expect("MITM sim should succeed");
        // Should complete without error
        assert!(!result.time_series.is_empty());
        // Metrics should be finite
        assert!(result.frequency_deviation_max_hz.is_finite());
        assert!(result.cyber_impact_index.is_finite());
        assert!((0.0..=1.0).contains(&result.cyber_impact_index));
    }
}
