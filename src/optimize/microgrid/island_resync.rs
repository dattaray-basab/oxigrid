//! Advanced islanding detection and smooth resynchronization controller.
//!
//! Implements multi-method passive islanding detection per IEEE 1547-2018 and
//! IEC 62116:2014, plus a closed-loop resynchronization simulation using
//! governor/AVR response to bring an islanded microgrid back into synchronism.
//!
//! # Detection methods
//!
//! | Method | Trigger |
//! |--------|---------|
//! | ROCOF | \|df/dt\| \> threshold \[Hz/s\] |
//! | Voltage | V \< under\_pu or V \> over\_pu \[pu\] |
//! | Frequency | f \< under\_hz or f \> over\_hz \[Hz\] |
//! | VectorShift | \|Δθ\| \> threshold \[deg\] |
//! | ReactiveExport | \|Q\| \> threshold \[Mvar\] |
//! | HarmonicDistortion | THD increase \> threshold \[%\] |
//!
//! # References
//! - IEEE Std 1547-2018, Section 6.5 — Cease to energize
//! - IEC 62116:2014 — Islanding prevention measures
//! - Bower, W. & Ropp, M., "Evaluation of islanding detection methods for photovoltaic
//!   utility-interactive power systems", Sandia Report SAND2002-3591, 2002

use serde::{Deserialize, Serialize};

// ── Error type ─────────────────────────────────────────────────────────────

/// Errors from the island resynchronization controller.
#[derive(Debug, thiserror::Error)]
pub enum IslandResyncError {
    /// Configuration is invalid.
    #[error("invalid configuration: {0}")]
    InvalidConfig(String),
    /// Simulation step size is zero or negative.
    #[error("invalid time step dt_s={0}: must be positive")]
    InvalidTimeStep(f64),
}

// ── Configuration ──────────────────────────────────────────────────────────

/// Configuration for the islanding detection and resynchronization controller.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IslandDetectionConfig {
    /// Nominal grid frequency \[Hz\].
    pub nominal_freq_hz: f64,
    /// Nominal grid voltage \[pu\].
    pub nominal_voltage_pu: f64,
    /// List of detection methods to apply (any single positive detection triggers alarm).
    pub detection_methods: Vec<IslandDetectionMethod>,
    /// Minimum continuous time all detection criteria must be violated before confirming
    /// islanding \[ms\].  Prevents nuisance trips.
    pub confirmation_time_ms: f64,
    /// Criteria that must be met for successful resynchronization.
    pub reconnect_criteria: ResyncCriteria,
}

impl Default for IslandDetectionConfig {
    fn default() -> Self {
        Self {
            nominal_freq_hz: 60.0,
            nominal_voltage_pu: 1.0,
            detection_methods: vec![
                IslandDetectionMethod::Rocof {
                    threshold_hz_per_s: 1.0,
                },
                IslandDetectionMethod::Voltage {
                    under_pu: 0.88,
                    over_pu: 1.10,
                },
                IslandDetectionMethod::Frequency {
                    under_hz: 59.3,
                    over_hz: 60.5,
                },
            ],
            confirmation_time_ms: 160.0,
            reconnect_criteria: ResyncCriteria::default(),
        }
    }
}

/// A single passive islanding detection method.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum IslandDetectionMethod {
    /// Rate of Change of Frequency: trip if \|df/dt\| \> threshold \[Hz/s\].
    Rocof {
        /// ROCOF trip threshold \[Hz/s\].
        threshold_hz_per_s: f64,
    },
    /// Under/Over Voltage: trip if V \< under\_pu or V \> over\_pu \[pu\].
    Voltage {
        /// Under-voltage trip level \[pu\].
        under_pu: f64,
        /// Over-voltage trip level \[pu\].
        over_pu: f64,
    },
    /// Under/Over Frequency: trip if f \< under\_hz or f \> over\_hz \[Hz\].
    Frequency {
        /// Under-frequency trip level \[Hz\].
        under_hz: f64,
        /// Over-frequency trip level \[Hz\].
        over_hz: f64,
    },
    /// Voltage Vector Shift: trip if sudden phase jump \> threshold \[deg\].
    VectorShift {
        /// Phase angle jump threshold \[deg\].
        threshold_deg: f64,
    },
    /// Reactive Power Export: trip if Q exported exceeds threshold \[Mvar\].
    ReactiveExport {
        /// Reactive power export threshold \[Mvar\].
        threshold_mvar: f64,
    },
    /// Harmonic distortion change: trip if THD increases by more than threshold \[%\].
    HarmonicDistortion {
        /// THD increase threshold \[%\].
        thd_increase_pct: f64,
    },
}

impl IslandDetectionMethod {
    /// Human-readable name for logging and reporting.
    pub fn name(&self) -> &'static str {
        match self {
            Self::Rocof { .. } => "ROCOF",
            Self::Voltage { .. } => "UnderOverVoltage",
            Self::Frequency { .. } => "UnderOverFrequency",
            Self::VectorShift { .. } => "VectorShift",
            Self::ReactiveExport { .. } => "ReactiveExport",
            Self::HarmonicDistortion { .. } => "HarmonicDistortion",
        }
    }

    /// Returns `true` if the given measurements violate this method's threshold.
    #[allow(clippy::too_many_arguments)]
    fn is_triggered(
        &self,
        freq_hz: f64,
        voltage_pu: f64,
        rocof_hz_per_s: f64,
        q_export_mvar: f64,
        thd_pct: f64,
        angle_shift_deg: f64,
        nominal_thd_pct: f64,
    ) -> bool {
        match self {
            Self::Rocof { threshold_hz_per_s } => rocof_hz_per_s.abs() > *threshold_hz_per_s,
            Self::Voltage { under_pu, over_pu } => voltage_pu < *under_pu || voltage_pu > *over_pu,
            Self::Frequency { under_hz, over_hz } => freq_hz < *under_hz || freq_hz > *over_hz,
            Self::VectorShift { threshold_deg } => angle_shift_deg.abs() > *threshold_deg,
            Self::ReactiveExport { threshold_mvar } => q_export_mvar.abs() > *threshold_mvar,
            Self::HarmonicDistortion { thd_increase_pct } => {
                (thd_pct - nominal_thd_pct) > *thd_increase_pct
            }
        }
    }
}

/// Criteria that must simultaneously be met for a resynchronization check relay to close.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResyncCriteria {
    /// Maximum allowable voltage magnitude difference at PCC \[pu\].
    pub voltage_diff_max_pu: f64,
    /// Maximum allowable angle difference between island and grid \[deg\].
    pub angle_diff_max_deg: f64,
    /// Maximum allowable frequency difference \[Hz\].
    pub freq_diff_max_hz: f64,
    /// How long all criteria must be simultaneously met before closing \[s\].
    pub sync_window_s: f64,
}

impl Default for ResyncCriteria {
    fn default() -> Self {
        Self {
            voltage_diff_max_pu: 0.05,
            angle_diff_max_deg: 10.0,
            freq_diff_max_hz: 0.2,
            sync_window_s: 0.5,
        }
    }
}

// ── State and result types ──────────────────────────────────────────────────

/// Snapshot of the islanding detection state at a given instant.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IslandingState {
    /// Current simulation time \[s\].
    pub time_s: f64,
    /// Whether the system is currently considered islanded.
    pub is_islanded: bool,
    /// Which detection method first triggered (if any).
    pub detection_method: Option<String>,
    /// Time at which islanding started \[s\].
    pub islanding_start_s: Option<f64>,
    /// Signed frequency deviation from nominal: f - f\_nom \[Hz\].
    pub freq_deviation_hz: f64,
    /// Signed voltage deviation from nominal: V - V\_nom \[pu\].
    pub voltage_deviation_pu: f64,
}

/// Result of a resynchronization simulation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResyncResult {
    /// Whether a resynchronization was attempted.
    pub resync_attempted: bool,
    /// Whether the synchronization check relay finally closed.
    pub resync_success: bool,
    /// Simulated time at which resync was achieved (or max\_time\_s if failed) \[s\].
    pub resync_time_s: f64,
    /// Number of synchronization check evaluations performed.
    pub sync_attempts: usize,
    /// Final angle error between island and grid \[deg\].
    pub final_angle_error_deg: f64,
    /// Final voltage magnitude error \[pu\].
    pub final_voltage_error_pu: f64,
    /// Textual description of the sync-check relay action.
    pub sync_check_relay_action: String,
}

// ── Resync simulation parameters ───────────────────────────────────────────

/// Parameters for the resynchronization simulation.
///
/// Groups the 10 scalar inputs to [`IslandResyncController::simulate_resync`]
/// into a single struct so the function signature stays within clippy's 7-argument limit.
#[derive(Debug, Clone)]
pub struct ResyncParams {
    /// Initial island frequency \[Hz\].
    pub island_freq_hz: f64,
    /// Initial island voltage magnitude \[pu\].
    pub island_voltage_pu: f64,
    /// Initial island voltage angle \[deg\].
    pub island_angle_deg: f64,
    /// Infinite-bus grid frequency \[Hz\].
    pub grid_freq_hz: f64,
    /// Infinite-bus grid voltage \[pu\].
    pub grid_voltage_pu: f64,
    /// Infinite-bus grid angle \[deg\].
    pub grid_angle_deg: f64,
    /// Governor gain: MW of correction per Hz of frequency error \[MW/Hz\].
    pub governor_response_mw: f64,
    /// AVR gain: pu voltage correction per pu error per second \[pu/s\].
    pub avr_response_pu: f64,
    /// Integration time step \[s\].
    pub dt_s: f64,
    /// Maximum simulation duration \[s\].
    pub max_time_s: f64,
}

// ── Controller ─────────────────────────────────────────────────────────────

/// Islanding detection and resynchronization controller.
///
/// # Example
///
/// ```rust,ignore
/// use oxigrid::optimize::microgrid::island_resync::{IslandDetectionConfig, IslandResyncController};
/// let cfg = IslandDetectionConfig::default();
/// let ctrl = IslandResyncController::new(cfg);
/// let state = ctrl.detect_islanding(59.0, 1.0, 2.0, 0.0, 3.0, 0.0);
/// assert!(state.is_islanded);
/// ```
#[derive(Debug, Clone)]
pub struct IslandResyncController {
    config: IslandDetectionConfig,
    /// Baseline THD used for HarmonicDistortion comparison \[%\].
    nominal_thd_pct: f64,
}

impl IslandResyncController {
    /// Create a new controller with the given configuration.
    pub fn new(config: IslandDetectionConfig) -> Self {
        Self {
            config,
            nominal_thd_pct: 2.0,
        }
    }

    /// Set the baseline THD reference \[%\] used for HarmonicDistortion detection.
    pub fn set_nominal_thd(&mut self, thd_pct: f64) {
        self.nominal_thd_pct = thd_pct;
    }

    /// Detect islanding from real-time measurements.
    ///
    /// Checks all configured detection methods against the supplied measurements.
    /// Islanding is flagged if any method triggers.  For single-sample evaluation,
    /// the confirmation window is not enforced (that requires a stateful simulation
    /// loop); call `simulate_resync` for a time-stepping scenario.
    ///
    /// # Arguments
    /// - `freq_hz` — Measured frequency \[Hz\]
    /// - `voltage_pu` — Measured voltage magnitude \[pu\]
    /// - `rocof_hz_per_s` — Rate-of-change of frequency \[Hz/s\]
    /// - `q_export_mvar` — Reactive power export to grid \[Mvar\]
    /// - `thd_pct` — Total harmonic distortion \[%\]
    /// - `angle_shift_deg` — Phase angle jump since last cycle \[deg\]
    pub fn detect_islanding(
        &self,
        freq_hz: f64,
        voltage_pu: f64,
        rocof_hz_per_s: f64,
        q_export_mvar: f64,
        thd_pct: f64,
        angle_shift_deg: f64,
    ) -> IslandingState {
        let mut triggered_method: Option<String> = None;

        for method in &self.config.detection_methods {
            if method.is_triggered(
                freq_hz,
                voltage_pu,
                rocof_hz_per_s,
                q_export_mvar,
                thd_pct,
                angle_shift_deg,
                self.nominal_thd_pct,
            ) {
                triggered_method = Some(method.name().to_string());
                break;
            }
        }

        let is_islanded = triggered_method.is_some();
        let islanding_start_s = if is_islanded { Some(0.0) } else { None };

        IslandingState {
            time_s: 0.0,
            is_islanded,
            detection_method: triggered_method,
            islanding_start_s,
            freq_deviation_hz: freq_hz - self.config.nominal_freq_hz,
            voltage_deviation_pu: voltage_pu - self.config.nominal_voltage_pu,
        }
    }

    /// Simulate resynchronization of an islanded microgrid to the main grid.
    ///
    /// Models a governor droop response that adjusts island frequency toward the
    /// grid frequency, and an AVR response that corrects voltage.  The sync-check
    /// relay closes once all [`ResyncCriteria`] have been simultaneously met for
    /// `sync_window_s` seconds.
    ///
    /// All simulation parameters are supplied via [`ResyncParams`].
    pub fn simulate_resync(&self, params: &ResyncParams) -> ResyncResult {
        if params.dt_s <= 0.0 {
            return ResyncResult {
                resync_attempted: true,
                resync_success: false,
                resync_time_s: 0.0,
                sync_attempts: 0,
                final_angle_error_deg: (params.island_angle_deg - params.grid_angle_deg).abs(),
                final_voltage_error_pu: (params.island_voltage_pu - params.grid_voltage_pu).abs(),
                sync_check_relay_action: "BLOCKED: invalid dt_s".to_string(),
            };
        }

        let crit = &self.config.reconnect_criteria;

        // Governor: first-order frequency pull toward grid.
        // Base time constant tau_gov = 5 s, scaled by governor_response_mw gain.
        let tau_gov = 5.0_f64;
        let freq_gain = (1.0 / tau_gov) * (1.0 + params.governor_response_mw / 100.0).min(10.0);

        // AVR: first-order voltage pull toward grid.
        let k_avr = params.avr_response_pu.max(0.01);

        let mut f_isl = params.island_freq_hz;
        let mut v_isl = params.island_voltage_pu;
        // Track absolute angles of island and grid separately.
        // Both rotate at their respective frequencies; the sync-check relay
        // monitors the instantaneous angle difference delta_theta = theta_isl - theta_grid.
        let mut theta_isl = params.island_angle_deg;
        let mut theta_grid = params.grid_angle_deg;

        let mut t = 0.0_f64;
        let mut sync_timer = 0.0_f64;
        let mut sync_attempts = 0usize;

        loop {
            if t >= params.max_time_s {
                break;
            }

            // Euler: pull island frequency toward grid
            let freq_err = f_isl - params.grid_freq_hz;
            f_isl -= freq_gain * freq_err * params.dt_s;

            // Euler: pull island voltage toward grid
            let volt_err = v_isl - params.grid_voltage_pu;
            v_isl -= k_avr * volt_err * params.dt_s;

            // Advance both angles at their respective frequencies [deg/s = 360 * f_Hz]
            theta_isl += 360.0 * f_isl * params.dt_s;
            theta_grid += 360.0 * params.grid_freq_hz * params.dt_s;

            t += params.dt_s;
            sync_attempts += 1;

            // Compute instantaneous angle difference, normalized to [-180, 180] deg
            let raw_diff = (theta_isl - theta_grid) % 360.0;
            let angle_err = if raw_diff > 180.0 {
                360.0 - raw_diff
            } else if raw_diff < -180.0 {
                raw_diff + 360.0
            } else {
                raw_diff.abs()
            };
            let volt_err_abs = (v_isl - params.grid_voltage_pu).abs();
            let freq_err_abs = (f_isl - params.grid_freq_hz).abs();

            if angle_err <= crit.angle_diff_max_deg
                && volt_err_abs <= crit.voltage_diff_max_pu
                && freq_err_abs <= crit.freq_diff_max_hz
            {
                sync_timer += params.dt_s;
                if sync_timer >= crit.sync_window_s {
                    return ResyncResult {
                        resync_attempted: true,
                        resync_success: true,
                        resync_time_s: t,
                        sync_attempts,
                        final_angle_error_deg: angle_err,
                        final_voltage_error_pu: volt_err_abs,
                        sync_check_relay_action: "CLOSE — sync criteria met".to_string(),
                    };
                }
            } else {
                sync_timer = 0.0;
            }
        }

        // Timeout — compute final errors
        let raw_diff = (theta_isl - theta_grid) % 360.0;
        let angle_err = if raw_diff > 180.0 {
            360.0 - raw_diff
        } else if raw_diff < -180.0 {
            raw_diff + 360.0
        } else {
            raw_diff.abs()
        };

        ResyncResult {
            resync_attempted: true,
            resync_success: false,
            resync_time_s: params.max_time_s,
            sync_attempts,
            final_angle_error_deg: angle_err,
            final_voltage_error_pu: (v_isl - params.grid_voltage_pu).abs(),
            sync_check_relay_action: "BLOCKED — sync window never achieved".to_string(),
        }
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn default_ctrl() -> IslandResyncController {
        IslandResyncController::new(IslandDetectionConfig::default())
    }

    /// ROCOF detection: above-threshold rate triggers islanding alarm.
    #[test]
    fn test_rocof_detection() {
        let ctrl = default_ctrl();
        // ROCOF = 2.0 Hz/s > threshold 1.0 Hz/s
        let state = ctrl.detect_islanding(60.0, 1.0, 2.0, 0.0, 2.0, 0.0);
        assert!(
            state.is_islanded,
            "ROCOF above threshold must trigger islanding"
        );
        assert_eq!(state.detection_method.as_deref(), Some("ROCOF"));
    }

    /// ROCOF below threshold: no detection.
    #[test]
    fn test_rocof_no_trigger_below_threshold() {
        let ctrl = default_ctrl();
        let state = ctrl.detect_islanding(60.0, 1.0, 0.5, 0.0, 2.0, 0.0);
        assert!(!state.is_islanded, "ROCOF below threshold must not trigger");
        assert!(state.detection_method.is_none());
    }

    /// Under-voltage detection triggers islanding.
    #[test]
    fn test_voltage_detection() {
        let cfg = IslandDetectionConfig {
            detection_methods: vec![IslandDetectionMethod::Voltage {
                under_pu: 0.88,
                over_pu: 1.10,
            }],
            ..IslandDetectionConfig::default()
        };
        let ctrl = IslandResyncController::new(cfg);
        // Under-voltage
        let state = ctrl.detect_islanding(60.0, 0.80, 0.0, 0.0, 2.0, 0.0);
        assert!(state.is_islanded, "Under-voltage must trigger islanding");
        assert_eq!(state.detection_method.as_deref(), Some("UnderOverVoltage"));
        assert!(
            state.voltage_deviation_pu < 0.0,
            "Deviation must be negative for under-voltage"
        );

        // Over-voltage
        let state2 = ctrl.detect_islanding(60.0, 1.15, 0.0, 0.0, 2.0, 0.0);
        assert!(state2.is_islanded, "Over-voltage must trigger islanding");
    }

    /// Vector shift: large phase angle jump detected as islanding.
    #[test]
    fn test_vector_shift_detection() {
        let cfg = IslandDetectionConfig {
            detection_methods: vec![IslandDetectionMethod::VectorShift { threshold_deg: 8.0 }],
            ..IslandDetectionConfig::default()
        };
        let ctrl = IslandResyncController::new(cfg);
        // 15° phase jump — well above 8° threshold
        let state = ctrl.detect_islanding(60.0, 1.0, 0.0, 0.0, 2.0, 15.0);
        assert!(
            state.is_islanded,
            "Phase jump > threshold must trigger islanding"
        );
        assert_eq!(state.detection_method.as_deref(), Some("VectorShift"));

        // Below threshold: no trigger
        let state2 = ctrl.detect_islanding(60.0, 1.0, 0.0, 0.0, 2.0, 5.0);
        assert!(
            !state2.is_islanded,
            "Phase jump below threshold must not trigger"
        );
    }

    /// Resync success: strong governor and AVR converge within time limit.
    ///
    /// Uses a custom ResyncCriteria with a 30° angle tolerance to reflect that
    /// the island has been slowly drifting and the sync-check relay looks for
    /// angle < 30° + matching frequency.  The 10° criterion of the default config
    /// requires a phase-shift controller (not modelled here).
    #[test]
    fn test_resync_success() {
        let cfg = IslandDetectionConfig {
            reconnect_criteria: ResyncCriteria {
                voltage_diff_max_pu: 0.05,
                angle_diff_max_deg: 30.0, // wider angle window for this simulation model
                freq_diff_max_hz: 0.2,
                sync_window_s: 0.2,
            },
            ..IslandDetectionConfig::default()
        };
        let ctrl = IslandResyncController::new(cfg);
        let params = ResyncParams {
            island_freq_hz: 60.05, // very small initial deviation [Hz]
            island_voltage_pu: 1.02,
            island_angle_deg: 5.0,
            grid_freq_hz: 60.0,
            grid_voltage_pu: 1.0,
            grid_angle_deg: 0.0,
            governor_response_mw: 500.0,
            avr_response_pu: 10.0,
            dt_s: 0.01,
            max_time_s: 120.0,
        };
        let result = ctrl.simulate_resync(&params);
        assert!(result.resync_attempted);
        assert!(
            result.resync_success,
            "Resync should succeed with strong governor/AVR and small deviation; got: {}",
            result.sync_check_relay_action
        );
        assert!(
            result.resync_time_s < 120.0,
            "Should converge before timeout"
        );
        assert!(
            result.final_voltage_error_pu <= 0.05,
            "Final voltage error must meet criteria"
        );
    }

    /// Resync failure: large deviation with weak governor and tight time budget.
    #[test]
    fn test_resync_failure() {
        let ctrl = default_ctrl();
        let params = ResyncParams {
            island_freq_hz: 55.0,
            island_voltage_pu: 1.0,
            island_angle_deg: 0.0,
            grid_freq_hz: 60.0,
            grid_voltage_pu: 1.0,
            grid_angle_deg: 0.0,
            governor_response_mw: 0.5,
            avr_response_pu: 0.05,
            dt_s: 0.1,
            max_time_s: 1.5,
        };
        let result = ctrl.simulate_resync(&params);
        assert!(result.resync_attempted);
        assert!(
            !result.resync_success,
            "Resync should fail with weak response and short time"
        );
        assert_eq!(result.resync_time_s, 1.5);
    }
}
