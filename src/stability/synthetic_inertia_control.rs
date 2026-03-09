//! Synthetic Inertia Control (SIC) for grid-connected inverters.
//!
//! Implements Virtual Synchronous Machine (VSM), Fast Frequency Response (FFR),
//! ROCOF-triggered control, and grid code compliance assessment.
//!
//! # Units
//! - Inertia constant H: \[MJ/MVA\]
//! - Frequency: \[Hz\]
//! - ROCOF: \[Hz/s\]
//! - Power: \[pu\]
//! - Time: \[s\]
//! - Angles: \[rad\]

/// SIC control method selection.
#[derive(Debug, Clone, PartialEq)]
pub enum SicMethod {
    /// VSM: emulate synchronous machine swing equation.
    VirtualSynchronousMachine,
    /// df/dt triggered response (derivative control).
    DerivativeControl,
    /// Frequency band (Drapeau bandwidth) controller.
    DrapeauBandwidth,
    /// Power Synchronization Control for weak grids.
    PowerSynchronization,
    /// Combined VSM + fast FFR hybrid strategy.
    CombinedVsmFfr,
}

/// Virtual Synchronous Machine parameters.
#[derive(Debug, Clone)]
pub struct VsmParameters {
    /// Virtual inertia constant H \[MJ/MVA\] (default 5.0).
    pub virtual_inertia_mj_mva: f64,
    /// Virtual damping coefficient D (default 20.0).
    pub virtual_damping: f64,
    /// Frequency droop percentage (default 4.0 %).
    pub droop_pct: f64,
    /// Maximum power output \[pu\] (default 1.0).
    pub p_max_pu: f64,
    /// Minimum power output \[pu\] (default 0.0).
    pub p_min_pu: f64,
}

impl Default for VsmParameters {
    fn default() -> Self {
        Self {
            virtual_inertia_mj_mva: 5.0,
            virtual_damping: 20.0,
            droop_pct: 4.0,
            p_max_pu: 1.0,
            p_min_pu: 0.0,
        }
    }
}

/// Fast Frequency Response parameters.
#[derive(Debug, Clone)]
pub struct FfrParameters {
    /// Activate FFR if |df/dt| exceeds this threshold \[Hz/s\] (default 0.5).
    pub rocof_trigger_hz_per_s: f64,
    /// Activate FFR if frequency drops below this value \[Hz\] (default 59.5).
    pub frequency_trigger_hz: f64,
    /// Maximum FFR power output \[pu\] (default 1.0).
    pub max_power_pu: f64,
    /// Duration to hold FFR at full power \[s\] (default 10.0).
    pub hold_time_s: f64,
    /// Ramp-back rate after hold period \[pu/s\] (default 0.1).
    pub release_rate_pu_per_s: f64,
}

impl Default for FfrParameters {
    fn default() -> Self {
        Self {
            rocof_trigger_hz_per_s: 0.5,
            frequency_trigger_hz: 59.5,
            max_power_pu: 1.0,
            hold_time_s: 10.0,
            release_rate_pu_per_s: 0.1,
        }
    }
}

/// Configuration for the SIC controller.
#[derive(Debug, Clone)]
pub struct SicConfig {
    /// Control method.
    pub method: SicMethod,
    /// Rated active power \[MW\].
    pub rated_mw: f64,
    /// Rated apparent power \[MVA\].
    pub rated_mva: f64,
    /// Nominal grid frequency \[Hz\] (default 60.0).
    pub f_nominal_hz: f64,
    /// Measurement window for ROCOF estimation \[s\] (default 0.1).
    pub measurement_window_s: f64,
    /// VSM parameters (required for VSM-based methods).
    pub vsm: Option<VsmParameters>,
    /// FFR parameters (required for FFR-based methods).
    pub ffr: Option<FfrParameters>,
    /// Frequency deadband — no response if |Δf| < deadband \[Hz\] (default 0.02).
    pub deadband_hz: f64,
}

impl Default for SicConfig {
    fn default() -> Self {
        Self {
            method: SicMethod::VirtualSynchronousMachine,
            rated_mw: 10.0,
            rated_mva: 10.0,
            f_nominal_hz: 60.0,
            measurement_window_s: 0.1,
            vsm: Some(VsmParameters::default()),
            ffr: Some(FfrParameters::default()),
            deadband_hz: 0.02,
        }
    }
}

/// Output produced by one SIC controller update step.
#[derive(Debug, Clone)]
pub struct SicOutput {
    /// SIC power contribution \[pu\].
    pub p_sic_pu: f64,
    /// Total power including SIC \[pu\].
    pub p_total_pu: f64,
    /// Measured ROCOF \[Hz/s\].
    pub rocof_hz_per_s: f64,
    /// Whether FFR is currently active.
    pub ffr_active: bool,
    /// Virtual rotor speed \[pu\] (1.0 = synchronous).
    pub vsm_omega_pu: f64,
    /// Human-readable label for the active strategy.
    pub method_active: String,
}

/// Frequency nadir prediction with and without SIC.
#[derive(Debug, Clone)]
pub struct NadirPrediction {
    /// Frequency nadir without SIC \[Hz\].
    pub f_nadir_without_sic_hz: f64,
    /// Frequency nadir with SIC \[Hz\].
    pub f_nadir_with_sic_hz: f64,
    /// Improvement due to SIC \[Hz\].
    pub improvement_hz: f64,
    /// Estimated time to nadir \[s\].
    pub time_to_nadir_s: f64,
}

/// Grid-code compliance report for a SIC configuration.
#[derive(Debug, Clone)]
pub struct SicComplianceReport {
    /// Grid code identifier (e.g., "ENTSO-E", "GB", "ERCOT").
    pub grid_code: String,
    /// Overall compliance flag.
    pub compliant: bool,
    /// Response time within code requirement.
    pub response_time_ok: bool,
    /// Hold time within code requirement.
    pub hold_time_ok: bool,
    /// Trigger threshold within code requirement.
    pub trigger_threshold_ok: bool,
    /// List of non-compliance descriptions.
    pub issues: Vec<String>,
}

/// Weak-grid compatibility assessment.
#[derive(Debug, Clone)]
pub struct WeakGridAssessment {
    /// Short-circuit ratio at the point of connection.
    pub scr: f64,
    /// Descriptive grid strength label.
    pub grid_strength: String,
    /// Recommended SIC method for this SCR.
    pub recommended_method: SicMethod,
    /// Qualitative stability risk.
    pub stability_risk: String,
}

/// Error type for SIC operations.
#[derive(Debug)]
pub enum SicError {
    /// Insufficient frequency history samples.
    InsufficientHistory,
    /// Missing required parameter block.
    MissingParameters(String),
    /// Invalid numeric input.
    InvalidInput(String),
}

impl std::fmt::Display for SicError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InsufficientHistory => write!(f, "insufficient frequency history"),
            Self::MissingParameters(s) => write!(f, "missing parameters: {s}"),
            Self::InvalidInput(s) => write!(f, "invalid input: {s}"),
        }
    }
}

/// Synthetic Inertia Controller.
///
/// Maintains frequency history and VSM rotor state across timesteps.
/// Call [`update`](SicController::update) at each simulation step.
#[derive(Debug, Clone)]
pub struct SicController {
    /// Public configuration.
    pub config: SicConfig,
    /// Rolling frequency measurement buffer \[Hz\].
    f_history: Vec<f64>,
    /// Timestamp buffer aligned with `f_history` \[s\].
    t_history: Vec<f64>,
    /// Accumulated simulation time \[s\].
    sim_time_s: f64,
    /// Current SIC power output \[pu\].
    p_output_pu: f64,
    /// FFR activation flag.
    ffr_active: bool,
    /// Elapsed time since FFR activation \[s\].
    ffr_timer_s: f64,
    /// Virtual rotor angle \[rad\].
    vsm_angle_rad: f64,
    /// Virtual rotor speed in \[pu\] (1.0 = synchronous).
    vsm_omega_pu: f64,
    /// Pre-disturbance setpoint used by VSM \[pu\].
    p_setpoint_pu: f64,
}

impl SicController {
    /// Create a new controller with the given configuration.
    pub fn new(config: SicConfig) -> Self {
        Self {
            config,
            f_history: Vec::new(),
            t_history: Vec::new(),
            sim_time_s: 0.0,
            p_output_pu: 0.0,
            ffr_active: false,
            ffr_timer_s: 0.0,
            vsm_angle_rad: 0.0,
            vsm_omega_pu: 1.0,
            p_setpoint_pu: 0.0,
        }
    }

    // ------------------------------------------------------------------
    // Public API
    // ------------------------------------------------------------------

    /// Step the controller forward by `dt_s` seconds.
    ///
    /// # Arguments
    /// * `f_hz`           – measured grid frequency \[Hz\]
    /// * `p_available_pu` – maximum power the source can deliver right now \[pu\]
    /// * `dt_s`           – time step \[s\]
    pub fn update(&mut self, f_hz: f64, p_available_pu: f64, dt_s: f64) -> SicOutput {
        // Append measurement
        self.f_history.push(f_hz);
        self.t_history.push(self.sim_time_s);
        self.sim_time_s += dt_s;

        // Prune history older than measurement_window_s * 2 to bound memory
        let window = self.config.measurement_window_s * 2.0;
        let cutoff = self.sim_time_s - window;
        while self.t_history.len() > 2 && self.t_history.first().copied().unwrap_or(0.0) < cutoff {
            self.t_history.remove(0);
            self.f_history.remove(0);
        }

        let f0 = self.config.f_nominal_hz;
        let delta_f = f_hz - f0;

        // Measure ROCOF
        let rocof = Self::calculate_rocof_from_history(&self.f_history, &self.t_history);

        // Deadband check — no action for tiny deviations
        if delta_f.abs() < self.config.deadband_hz {
            self.vsm_omega_pu = 1.0 + delta_f / f0;
            let out = p_available_pu.min(self.p_output_pu.max(0.0));
            return SicOutput {
                p_sic_pu: 0.0,
                p_total_pu: out,
                rocof_hz_per_s: rocof,
                ffr_active: self.ffr_active,
                vsm_omega_pu: self.vsm_omega_pu,
                method_active: "Deadband".to_string(),
            };
        }

        let (p_sic, method_label) = match &self.config.method {
            SicMethod::VirtualSynchronousMachine => {
                let p = self.update_vsm(delta_f, dt_s);
                (p, "VSM".to_string())
            }
            SicMethod::DerivativeControl => {
                let p = self.update_derivative(rocof);
                (p, "DerivativeControl".to_string())
            }
            SicMethod::DrapeauBandwidth => {
                let p = self.update_drapeau(delta_f, f0);
                (p, "DrapeauBandwidth".to_string())
            }
            SicMethod::PowerSynchronization => {
                let p = self.update_psc(delta_f, dt_s);
                (p, "PowerSynchronization".to_string())
            }
            SicMethod::CombinedVsmFfr => {
                let p_vsm = self.update_vsm(delta_f, dt_s);
                let p_ffr = self.update_ffr(f_hz, rocof, dt_s);
                (p_vsm + p_ffr, "CombinedVsmFfr".to_string())
            }
        };

        // For non-combined methods also run FFR if configured
        let p_ffr_extra =
            if self.config.method != SicMethod::CombinedVsmFfr && self.config.ffr.is_some() {
                self.update_ffr(f_hz, rocof, dt_s)
            } else {
                0.0
            };

        let p_total_sic = p_sic + p_ffr_extra;
        // Clamp to [0, p_available_pu]
        let p_total_sic_clamped = p_total_sic.clamp(0.0, p_available_pu);

        self.p_output_pu = p_total_sic_clamped;

        SicOutput {
            p_sic_pu: p_total_sic_clamped,
            p_total_pu: p_total_sic_clamped,
            rocof_hz_per_s: rocof,
            ffr_active: self.ffr_active,
            vsm_omega_pu: self.vsm_omega_pu,
            method_active: method_label,
        }
    }

    /// Calculate ROCOF \[Hz/s\] by linear regression on the last `window_s` seconds
    /// of frequency measurements.
    ///
    /// Returns 0.0 when fewer than 2 samples are available.
    pub fn calculate_rocof(freq_history: &[f64], window_s: f64) -> f64 {
        if freq_history.len() < 2 {
            return 0.0;
        }
        // Reconstruct uniform time axis (assume unit spacing, then rescale)
        let n = freq_history.len();
        let dt = window_s / (n - 1) as f64;
        let times: Vec<f64> = (0..n).map(|i| i as f64 * dt).collect();
        linear_regression_slope(&times, freq_history)
    }

    /// Integrate the swing equation by one Euler step.
    ///
    /// `delta_p_load` is the per-unit load disturbance (positive = increased load),
    /// `omega_pu` is the current rotor speed in pu,
    /// `dt_s` is the timestep \[s\].
    ///
    /// Returns `(new_omega_pu, new_delta_rad)`.
    pub fn vsm_swing_equation(
        &self,
        delta_p_load: f64,
        omega_pu: f64,
        dt_s: f64,
    ) -> Result<(f64, f64), SicError> {
        let vsm = self
            .config
            .vsm
            .as_ref()
            .ok_or_else(|| SicError::MissingParameters("VsmParameters".to_string()))?;

        let h = vsm.virtual_inertia_mj_mva;
        let d = vsm.virtual_damping;
        let omega_0 = 2.0 * std::f64::consts::PI * self.config.f_nominal_hz;

        let delta_omega = omega_pu - 1.0;
        // Mechanical power = setpoint, electrical power = setpoint + disturbance
        let pm = self.p_setpoint_pu;
        let pe = pm + delta_p_load;

        // dω/dt = (Pm - Pe - D·Δω) / (2H)
        let d_omega_dt = (pm - pe - d * delta_omega) / (2.0 * h);
        let new_omega = omega_pu + d_omega_dt * dt_s;

        // dδ/dt = ω_0 · Δω
        let d_delta_dt = omega_0 * delta_omega;
        let new_delta = self.vsm_angle_rad + d_delta_dt * dt_s;

        Ok((new_omega, new_delta))
    }

    /// Estimate the effective inertia contribution of the SIC unit.
    ///
    /// `p_delta_pu` – power injection change \[pu\]
    /// `df_dt_hz_per_s` – measured ROCOF \[Hz/s\]
    ///
    /// Returns effective H \[MJ/MVA\] at rated MVA.
    /// Returns 0 when ROCOF is near zero to avoid division by zero.
    pub fn estimate_inertia_contribution(&self, p_delta_pu: f64, df_dt_hz_per_s: f64) -> f64 {
        if df_dt_hz_per_s.abs() < 1e-9 {
            return 0.0;
        }
        let f0 = self.config.f_nominal_hz;
        let s_rated = self.config.rated_mva;
        // H_eff = -P_delta / (2 * df/dt) * f0 / S_rated  (power in MW)
        let p_delta_mw = p_delta_pu * self.config.rated_mw;
        -p_delta_mw / (2.0 * df_dt_hz_per_s) * f0 / s_rated
    }

    /// Predict frequency nadir with and without this SIC unit.
    ///
    /// # Arguments
    /// * `disturbance_mw`   – generation loss \[MW\]
    /// * `total_inertia_mj` – total system inertia **without** this SIC unit \[MJ\]
    ///
    /// Uses the simplified equal-area / swing equation approximation:
    /// `Δf_nadir ≈ disturbance · Δt / (2 · H_total)`
    pub fn frequency_nadir_prediction(
        &self,
        disturbance_mw: f64,
        total_inertia_mj: f64,
    ) -> Result<NadirPrediction, SicError> {
        if disturbance_mw <= 0.0 {
            return Err(SicError::InvalidInput(
                "disturbance_mw must be positive".to_string(),
            ));
        }
        if total_inertia_mj <= 0.0 {
            return Err(SicError::InvalidInput(
                "total_inertia_mj must be positive".to_string(),
            ));
        }
        let f0 = self.config.f_nominal_hz;

        // Governor response assumed to start arresting frequency at ~10 s
        let governor_response_s = 10.0;

        // Using the simplified equal-area / swing-equation nadir model:
        //   ROCOF_0 = -P_loss * f0 / (2 * H_total_MJ)    [Hz/s]
        //   t_nadir ≈ H_total / (P_loss * k_gov)          [s], k_gov is governor gain
        //   Δf_nadir = |ROCOF_0| * t_nadir * 0.5          (triangle area approximation)
        //   => Δf = P_loss * f0 / (2 * H_total_MJ) * t_nadir * 0.5
        //
        // t_nadir depends on governor: t_nadir = sqrt(2 * H_total * Δf_limit / (P_loss * k_gov))
        // For a simple model we use: t_nadir = sqrt(H_total_MJ / (P_loss * governor_response_s))
        let k_gov = 1.0 / governor_response_s; // governor gain [pu/s]
        let t_nadir = (total_inertia_mj / (disturbance_mw * k_gov * f0)).sqrt();
        let rocof_0 = disturbance_mw * f0 / (2.0 * total_inertia_mj);
        let df_without = rocof_0 * t_nadir * 0.5;
        let f_nadir_without = f0 - df_without;

        // Effective SIC inertia contribution
        let vsm_h = self
            .config
            .vsm
            .as_ref()
            .map(|v| v.virtual_inertia_mj_mva)
            .unwrap_or(0.0);
        let sic_inertia_mj = vsm_h * self.config.rated_mva;
        let total_with_sic = total_inertia_mj + sic_inertia_mj;

        let t_nadir_sic = (total_with_sic / (disturbance_mw * k_gov * f0)).sqrt();
        let rocof_0_sic = disturbance_mw * f0 / (2.0 * total_with_sic);
        let df_with = rocof_0_sic * t_nadir_sic * 0.5;
        let f_nadir_with = f0 - df_with;

        Ok(NadirPrediction {
            f_nadir_without_sic_hz: f_nadir_without,
            f_nadir_with_sic_hz: f_nadir_with,
            improvement_hz: f_nadir_with - f_nadir_without,
            time_to_nadir_s: t_nadir,
        })
    }

    /// Check whether the SIC configuration meets a named grid code.
    ///
    /// Supported codes: `"ENTSO-E"`, `"GB"`, `"ERCOT"`.
    pub fn grid_code_compliance(&self, grid_code: &str) -> SicComplianceReport {
        let ffr = match &self.config.ffr {
            Some(p) => p,
            None => {
                return SicComplianceReport {
                    grid_code: grid_code.to_string(),
                    compliant: false,
                    response_time_ok: false,
                    hold_time_ok: false,
                    trigger_threshold_ok: false,
                    issues: vec!["No FfrParameters configured".to_string()],
                };
            }
        };

        let mut issues = Vec::new();

        match grid_code {
            "ENTSO-E" => {
                // FFR must respond within 0.5 s, hold 10 s, release over 20 s
                // We approximate response_time as measurement_window_s
                let response_time_ok = self.config.measurement_window_s <= 0.5;
                let hold_time_ok = ffr.hold_time_s >= 10.0;
                // Trigger at 49.0 Hz (or equivalent) for 60 Hz system scaled
                let trigger_hz = if self.config.f_nominal_hz < 55.0 {
                    49.0
                } else {
                    59.0
                };
                let trigger_threshold_ok = ffr.frequency_trigger_hz <= trigger_hz + 1.0;
                let release_ok = ffr.release_rate_pu_per_s <= 0.05 + 1e-9;
                if !response_time_ok {
                    issues.push(format!(
                        "measurement_window_s {:.3} > 0.5 s (ENTSO-E FFR response requirement)",
                        self.config.measurement_window_s
                    ));
                }
                if !hold_time_ok {
                    issues.push(format!(
                        "hold_time_s {:.1} < 10 s (ENTSO-E requirement)",
                        ffr.hold_time_s
                    ));
                }
                if !trigger_threshold_ok {
                    issues.push(format!(
                        "frequency_trigger_hz {:.2} above code limit {:.2}",
                        ffr.frequency_trigger_hz,
                        trigger_hz + 1.0
                    ));
                }
                if !release_ok {
                    issues.push(format!(
                        "release_rate_pu_per_s {:.3} > 0.05 (ENTSO-E 20 s release requirement)",
                        ffr.release_rate_pu_per_s
                    ));
                }
                let compliant =
                    response_time_ok && hold_time_ok && trigger_threshold_ok && release_ok;
                SicComplianceReport {
                    grid_code: grid_code.to_string(),
                    compliant,
                    response_time_ok,
                    hold_time_ok,
                    trigger_threshold_ok,
                    issues,
                }
            }
            "GB" => {
                // FFR response within 1 s at 49.7 Hz trigger
                let response_time_ok = self.config.measurement_window_s <= 1.0;
                let hold_time_ok = ffr.hold_time_s >= 30.0;
                let trigger_threshold_ok = ffr.frequency_trigger_hz <= 49.8;
                if !response_time_ok {
                    issues.push(format!(
                        "measurement_window_s {:.3} > 1.0 s (GB FFR response requirement)",
                        self.config.measurement_window_s
                    ));
                }
                if !hold_time_ok {
                    issues.push(format!(
                        "hold_time_s {:.1} < 30 s (GB requirement)",
                        ffr.hold_time_s
                    ));
                }
                if !trigger_threshold_ok {
                    issues.push(format!(
                        "frequency_trigger_hz {:.2} > 49.8 Hz (GB requirement)",
                        ffr.frequency_trigger_hz
                    ));
                }
                let compliant = response_time_ok && hold_time_ok && trigger_threshold_ok;
                SicComplianceReport {
                    grid_code: grid_code.to_string(),
                    compliant,
                    response_time_ok,
                    hold_time_ok,
                    trigger_threshold_ok,
                    issues,
                }
            }
            "ERCOT" => {
                // 100 ms response, 10 min hold
                let response_time_ok = self.config.measurement_window_s <= 0.1;
                let hold_time_ok = ffr.hold_time_s >= 600.0;
                let trigger_threshold_ok = ffr.frequency_trigger_hz <= 59.7;
                if !response_time_ok {
                    issues.push(format!(
                        "measurement_window_s {:.3} > 0.1 s (ERCOT requirement)",
                        self.config.measurement_window_s
                    ));
                }
                if !hold_time_ok {
                    issues.push(format!(
                        "hold_time_s {:.1} < 600 s / 10 min (ERCOT requirement)",
                        ffr.hold_time_s
                    ));
                }
                if !trigger_threshold_ok {
                    issues.push(format!(
                        "frequency_trigger_hz {:.2} > 59.7 Hz (ERCOT requirement)",
                        ffr.frequency_trigger_hz
                    ));
                }
                let compliant = response_time_ok && hold_time_ok && trigger_threshold_ok;
                SicComplianceReport {
                    grid_code: grid_code.to_string(),
                    compliant,
                    response_time_ok,
                    hold_time_ok,
                    trigger_threshold_ok,
                    issues,
                }
            }
            other => SicComplianceReport {
                grid_code: other.to_string(),
                compliant: false,
                response_time_ok: false,
                hold_time_ok: false,
                trigger_threshold_ok: false,
                issues: vec![format!("Unknown grid code: {other}")],
            },
        }
    }

    /// Assess compatibility with weak grids and recommend a control method.
    ///
    /// `scr` – short-circuit ratio at the point of connection (dimensionless).
    pub fn assess_weak_grid_compatibility(&self, scr: f64) -> WeakGridAssessment {
        let (grid_strength, recommended_method, stability_risk) = if scr >= 3.0 {
            (
                "Strong".to_string(),
                SicMethod::VirtualSynchronousMachine,
                "Low".to_string(),
            )
        } else if scr >= 1.5 {
            (
                "Moderate".to_string(),
                SicMethod::VirtualSynchronousMachine,
                "Medium".to_string(),
            )
        } else if scr >= 1.0 {
            (
                "Weak".to_string(),
                SicMethod::PowerSynchronization,
                "High".to_string(),
            )
        } else {
            (
                "Very Weak".to_string(),
                SicMethod::PowerSynchronization,
                "High".to_string(),
            )
        };
        WeakGridAssessment {
            scr,
            grid_strength,
            recommended_method,
            stability_risk,
        }
    }

    // ------------------------------------------------------------------
    // Private helpers
    // ------------------------------------------------------------------

    /// Compute ROCOF from internal history using timestamps.
    fn calculate_rocof_from_history(f_history: &[f64], t_history: &[f64]) -> f64 {
        if f_history.len() < 2 || t_history.len() < 2 {
            return 0.0;
        }
        linear_regression_slope(t_history, f_history)
    }

    /// VSM control update: integrates swing equation and returns power increment \[pu\].
    fn update_vsm(&mut self, delta_f: f64, dt_s: f64) -> f64 {
        let vsm = match &self.config.vsm {
            Some(v) => v.clone(),
            None => return 0.0,
        };
        let f0 = self.config.f_nominal_hz;
        let omega_0 = 2.0 * std::f64::consts::PI * f0;

        let delta_omega = delta_f / f0; // pu deviation
        let h = vsm.virtual_inertia_mj_mva;
        let d = vsm.virtual_damping;

        // Droop contribution (proportional to Δf)
        let droop_gain = 1.0 / (vsm.droop_pct / 100.0 * f0);
        let p_droop = -droop_gain * delta_f;

        // Damping contribution
        let p_damp = -d * delta_omega;

        // Inertial contribution (approximated via derivative of omega)
        let prev_omega = self.vsm_omega_pu;
        let new_omega = 1.0 + delta_omega;
        let d_omega_dt = (new_omega - prev_omega) / dt_s.max(1e-9);
        let p_inertia = -2.0 * h * d_omega_dt;

        self.vsm_omega_pu = new_omega;

        // Update virtual rotor angle
        self.vsm_angle_rad += omega_0 * delta_omega * dt_s;

        let p_total = p_droop + p_damp + p_inertia;
        p_total.clamp(vsm.p_min_pu, vsm.p_max_pu)
    }

    /// Derivative (ROCOF) control: power proportional to -df/dt.
    fn update_derivative(&self, rocof: f64) -> f64 {
        let vsm = match &self.config.vsm {
            Some(v) => v,
            None => return 0.0,
        };
        // Gain: H / f0  (standard inertia emulation formula P = -2H/f0 * df/dt)
        let h = vsm.virtual_inertia_mj_mva;
        let f0 = self.config.f_nominal_hz;
        let p = -2.0 * h / f0 * rocof;
        p.clamp(vsm.p_min_pu, vsm.p_max_pu)
    }

    /// Drapeau bandwidth controller: power proportional to Δf within a band.
    fn update_drapeau(&self, delta_f: f64, f0: f64) -> f64 {
        let vsm = match &self.config.vsm {
            Some(v) => v,
            None => return 0.0,
        };
        // Simple proportional with gain inversely proportional to droop
        let gain = 1.0 / (vsm.droop_pct / 100.0 * f0);
        let p = -gain * delta_f;
        p.clamp(vsm.p_min_pu, vsm.p_max_pu)
    }

    /// Power Synchronization Control for weak grids: angle-droop based.
    fn update_psc(&mut self, delta_f: f64, dt_s: f64) -> f64 {
        let vsm = match &self.config.vsm {
            Some(v) => v.clone(),
            None => return 0.0,
        };
        let f0 = self.config.f_nominal_hz;
        let omega_0 = 2.0 * std::f64::consts::PI * f0;

        // Integrate angle from frequency deviation
        self.vsm_angle_rad += omega_0 * (delta_f / f0) * dt_s;

        // Active power reference from angle (simplified PSC)
        let p = -vsm.virtual_damping / 100.0 * self.vsm_angle_rad;
        p.clamp(vsm.p_min_pu, vsm.p_max_pu)
    }

    /// FFR logic: activate on trigger, hold, then ramp down.
    /// Returns additional FFR power \[pu\].
    fn update_ffr(&mut self, f_hz: f64, rocof: f64, dt_s: f64) -> f64 {
        let ffr = match self.config.ffr.clone() {
            Some(p) => p,
            None => return 0.0,
        };

        // Activation conditions
        let freq_trigger = f_hz < ffr.frequency_trigger_hz;
        let rocof_trigger = rocof.abs() > ffr.rocof_trigger_hz_per_s;

        if !self.ffr_active {
            if freq_trigger || rocof_trigger {
                self.ffr_active = true;
                self.ffr_timer_s = 0.0;
            } else {
                return 0.0;
            }
        }

        // Accumulate timer
        self.ffr_timer_s += dt_s;

        // Hold phase
        if self.ffr_timer_s <= ffr.hold_time_s {
            ffr.max_power_pu
        } else {
            // Ramp-down phase
            let elapsed_since_hold = self.ffr_timer_s - ffr.hold_time_s;
            let remaining =
                (ffr.max_power_pu - ffr.release_rate_pu_per_s * elapsed_since_hold).max(0.0);
            if remaining <= 0.0 {
                self.ffr_active = false;
                self.ffr_timer_s = 0.0;
            }
            remaining
        }
    }
}

// ------------------------------------------------------------------
// Standalone utilities
// ------------------------------------------------------------------

/// Compute linear regression slope (dy/dx) for paired (x, y) samples.
///
/// Returns 0.0 if fewer than 2 points or denominator is near zero.
fn linear_regression_slope(x: &[f64], y: &[f64]) -> f64 {
    let n = x.len().min(y.len());
    if n < 2 {
        return 0.0;
    }
    let n_f = n as f64;
    let sx: f64 = x[..n].iter().sum();
    let sy: f64 = y[..n].iter().sum();
    let sxy: f64 = x[..n]
        .iter()
        .zip(y[..n].iter())
        .map(|(xi, yi)| xi * yi)
        .sum();
    let sxx: f64 = x[..n].iter().map(|xi| xi * xi).sum();
    let denom = n_f * sxx - sx * sx;
    if denom.abs() < 1e-15 {
        return 0.0;
    }
    (n_f * sxy - sx * sy) / denom
}

// ------------------------------------------------------------------
// Tests
// ------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config_60hz() -> SicConfig {
        SicConfig {
            method: SicMethod::VirtualSynchronousMachine,
            rated_mw: 10.0,
            rated_mva: 10.0,
            f_nominal_hz: 60.0,
            measurement_window_s: 0.1,
            vsm: Some(VsmParameters::default()),
            ffr: Some(FfrParameters::default()),
            deadband_hz: 0.02,
        }
    }

    // ----------------------------------------------------------------
    // 1. ROCOF: linear frequency ramp → correct df/dt
    // ----------------------------------------------------------------
    #[test]
    fn test_rocof_linear_ramp() {
        // Frequency drops at exactly -1.0 Hz/s
        let n = 101usize;
        let dt = 0.01_f64;
        let freq: Vec<f64> = (0..n).map(|i| 60.0 - i as f64 * dt).collect();
        let window = (n - 1) as f64 * dt; // 1.0 s
        let rocof = SicController::calculate_rocof(&freq, window);
        assert!(
            (rocof - (-1.0)).abs() < 0.01,
            "ROCOF should be -1.0 Hz/s, got {rocof:.4}"
        );
    }

    // ----------------------------------------------------------------
    // 2. VSM: frequency deviation causes power injection
    // ----------------------------------------------------------------
    #[test]
    fn test_vsm_power_injection_on_freq_drop() {
        let config = default_config_60hz();
        let mut ctrl = SicController::new(config);

        // Stable at 60.0 Hz for a few steps, then drop to 59.0 Hz
        for _ in 0..5 {
            ctrl.update(60.0, 1.0, 0.02);
        }
        let out = ctrl.update(59.0, 1.0, 0.02);
        // Frequency drop → VSM should inject positive power
        assert!(
            out.p_sic_pu > 0.0,
            "VSM must inject power on frequency drop, got {:.4}",
            out.p_sic_pu
        );
    }

    // ----------------------------------------------------------------
    // 3. FFR trigger: f < 59.5 Hz → FFR activates
    // ----------------------------------------------------------------
    #[test]
    fn test_ffr_activates_on_frequency_trigger() {
        let mut config = default_config_60hz();
        config.method = SicMethod::DerivativeControl;
        let mut ctrl = SicController::new(config);

        // Drive frequency below trigger (59.5 Hz)
        for _ in 0..3 {
            ctrl.update(60.0, 1.0, 0.02);
        }
        let out = ctrl.update(59.3, 1.0, 0.02);
        assert!(out.ffr_active, "FFR should activate when f < 59.5 Hz");
        assert!(
            out.p_sic_pu > 0.0,
            "Power output should be > 0 when FFR active"
        );
    }

    // ----------------------------------------------------------------
    // 4. FFR hold: active for hold_time_s
    // ----------------------------------------------------------------
    #[test]
    fn test_ffr_hold_duration() {
        let mut config = default_config_60hz();
        config.method = SicMethod::DerivativeControl;
        if let Some(ref mut ffr) = config.ffr {
            ffr.hold_time_s = 5.0;
        }
        let mut ctrl = SicController::new(config);

        // Trigger FFR
        ctrl.update(59.0, 1.0, 0.02);

        // Advance 4.5 s (within hold_time = 5 s)
        let steps = (4.5 / 0.02) as usize;
        let mut out = ctrl.update(59.0, 1.0, 0.02);
        for _ in 1..steps {
            out = ctrl.update(59.0, 1.0, 0.02);
        }
        assert!(
            out.ffr_active,
            "FFR should still be active within hold_time_s"
        );
    }

    // ----------------------------------------------------------------
    // 5. FFR release: ramps down after hold
    // ----------------------------------------------------------------
    #[test]
    fn test_ffr_release_after_hold() {
        let mut config = default_config_60hz();
        config.method = SicMethod::DerivativeControl;
        if let Some(ref mut ffr) = config.ffr {
            ffr.hold_time_s = 1.0;
            ffr.release_rate_pu_per_s = 0.5; // quick release for test
        }
        let mut ctrl = SicController::new(config);

        // Trigger FFR
        ctrl.update(59.0, 1.0, 0.02);

        // Advance past hold time
        let steps_past_hold = (2.0 / 0.02) as usize;
        let mut last_ffr_power = 0.0_f64;
        for _ in 0..steps_past_hold {
            let out = ctrl.update(59.0, 1.0, 0.02);
            last_ffr_power = out.p_sic_pu;
        }
        // After 2 s total (1 s hold + 1 s release at 0.5 pu/s), output should be ~0.5 pu
        // or less (could be ramping down)
        assert!(
            last_ffr_power < 1.0,
            "FFR power should ramp down after hold_time; got {last_ffr_power:.4}"
        );
    }

    // ----------------------------------------------------------------
    // 6. Deadband: tiny Δf < deadband → no response
    // ----------------------------------------------------------------
    #[test]
    fn test_deadband_no_response() {
        let config = default_config_60hz();
        let mut ctrl = SicController::new(config);

        // Frequency within deadband (0.02 Hz)
        let out = ctrl.update(60.01, 1.0, 0.02);
        assert_eq!(
            out.p_sic_pu, 0.0,
            "No SIC response expected within deadband"
        );
    }

    // ----------------------------------------------------------------
    // 7. Nadir prediction: with SIC → better nadir than without
    // ----------------------------------------------------------------
    #[test]
    fn test_nadir_prediction_improvement() {
        let config = default_config_60hz();
        let ctrl = SicController::new(config);

        // 50 MW disturbance in a 2000 MJ system
        let pred = ctrl
            .frequency_nadir_prediction(50.0, 2000.0)
            .expect("nadir prediction");

        assert!(
            pred.f_nadir_with_sic_hz > pred.f_nadir_without_sic_hz,
            "SIC should raise the nadir: with={:.4} Hz, without={:.4} Hz",
            pred.f_nadir_with_sic_hz,
            pred.f_nadir_without_sic_hz
        );
        assert!(
            pred.improvement_hz > 0.0,
            "improvement_hz should be positive"
        );
    }

    // ----------------------------------------------------------------
    // 8. Compliance: ENTSO-E hold_time=10s, response_time=0.1s → compliant
    // ----------------------------------------------------------------
    #[test]
    fn test_entso_e_compliance_pass() {
        let mut config = default_config_60hz();
        config.measurement_window_s = 0.1; // ≤ 0.5 s requirement
        if let Some(ref mut ffr) = config.ffr {
            ffr.hold_time_s = 10.0; // ≥ 10 s
            ffr.release_rate_pu_per_s = 0.05; // ≤ 0.05 pu/s (20 s release)
            ffr.frequency_trigger_hz = 59.5; // within allowed range
        }
        let ctrl = SicController::new(config);
        let report = ctrl.grid_code_compliance("ENTSO-E");

        assert!(
            report.compliant,
            "Should be ENTSO-E compliant; issues: {:?}",
            report.issues
        );
        assert!(report.response_time_ok);
        assert!(report.hold_time_ok);
        assert!(report.trigger_threshold_ok);
    }

    // ----------------------------------------------------------------
    // 9. Swing equation returns reasonable (omega, delta)
    // ----------------------------------------------------------------
    #[test]
    fn test_swing_equation_basic() {
        let config = default_config_60hz();
        let mut ctrl = SicController::new(config);
        ctrl.vsm_omega_pu = 1.0;

        // Small load increase → frequency should drop (omega < 1 after step)
        let (new_omega, new_delta) = ctrl
            .vsm_swing_equation(0.1, 1.0, 0.02)
            .expect("swing equation");

        assert!(
            new_omega < 1.0,
            "Load increase should reduce omega; got {new_omega:.6}"
        );
        // Delta should remain near zero for this tiny step
        assert!(
            new_delta.abs() < 0.01,
            "Delta should be small; got {new_delta:.6}"
        );
    }

    // ----------------------------------------------------------------
    // 10. Weak-grid assessment recommends PSC for low SCR
    // ----------------------------------------------------------------
    #[test]
    fn test_weak_grid_psc_recommended() {
        let config = default_config_60hz();
        let ctrl = SicController::new(config);

        let assessment = ctrl.assess_weak_grid_compatibility(0.8);
        assert_eq!(assessment.grid_strength, "Very Weak");
        assert_eq!(
            assessment.recommended_method,
            SicMethod::PowerSynchronization
        );
        assert_eq!(assessment.stability_risk, "High");

        let strong = ctrl.assess_weak_grid_compatibility(5.0);
        assert_eq!(strong.grid_strength, "Strong");
        assert_eq!(strong.stability_risk, "Low");
    }

    // ----------------------------------------------------------------
    // 11. Estimate inertia contribution is non-negative for power injection
    // ----------------------------------------------------------------
    #[test]
    fn test_estimate_inertia_contribution() {
        let config = default_config_60hz();
        let ctrl = SicController::new(config);

        // When frequency is falling (negative df/dt), power injection (positive) should yield +H
        let h_eff = ctrl.estimate_inertia_contribution(0.1, -1.0);
        assert!(
            h_eff > 0.0,
            "Effective H should be positive; got {h_eff:.4}"
        );
    }

    // ----------------------------------------------------------------
    // 12. CombinedVsmFfr mode produces output >= pure VSM output
    // ----------------------------------------------------------------
    #[test]
    fn test_combined_vsm_ffr_output() {
        let mut config_vsm = default_config_60hz();
        config_vsm.method = SicMethod::VirtualSynchronousMachine;
        let mut ctrl_vsm = SicController::new(config_vsm);
        for _ in 0..3 {
            ctrl_vsm.update(60.0, 1.0, 0.02);
        }
        let out_vsm = ctrl_vsm.update(59.0, 1.0, 0.02);

        let mut config_comb = default_config_60hz();
        config_comb.method = SicMethod::CombinedVsmFfr;
        let mut ctrl_comb = SicController::new(config_comb);
        for _ in 0..3 {
            ctrl_comb.update(60.0, 1.0, 0.02);
        }
        let out_comb = ctrl_comb.update(59.0, 1.0, 0.02);

        assert!(
            out_comb.p_sic_pu >= out_vsm.p_sic_pu,
            "Combined mode should produce at least as much power as pure VSM: comb={:.4} vsm={:.4}",
            out_comb.p_sic_pu,
            out_vsm.p_sic_pu
        );
    }
}
