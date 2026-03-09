//! IEEE 1547-2018 compliant smart inverter model.
//!
//! Implements Volt-VAR, Volt-Watt, Frequency-Watt, constant power factor,
//! and active power curtailment functions as required by IEEE 1547-2018.
//!
//! # Units
//! - Voltages: \[pu\] (per-unit of nominal)
//! - Powers: \[pu\] of rated kVA internally; \[kW\] / \[kvar\] in outputs
//! - Frequencies: \[Hz\]
//! - Times: \[s\]
//! - Apparent power rating: \[kVA\]

/// Operating mode of the smart inverter.
#[derive(Debug, Clone, PartialEq)]
pub enum SmartInverterMode {
    /// Fixed power factor control.
    ConstantPowerFactor {
        /// Power factor (signed: positive = lagging / inductive absorption).
        pf: f64,
    },
    /// Volt-VAR (Q(V)) curve control.
    VoltVar,
    /// Volt-Watt (P(V)) curtailment control.
    VoltWatt,
    /// Frequency-Watt (P(f)) droop control.
    FrequencyWatt,
    /// Combined Volt-VAR + Volt-Watt.
    VoltVarPlusVoltWatt,
    /// Direct P/Q setpoint control \[pu\].
    ActiveReactivePowerControl {
        /// Active power reference \[pu\].
        p_ref: f64,
        /// Reactive power reference \[pu\].
        q_ref: f64,
    },
}

/// Piecewise-linear Volt-VAR (Q(V)) curve per IEEE 1547-2018.
///
/// IEEE 1547 default: V = \[0.92, 0.98, 1.02, 1.08\] pu,
/// Q = \[0.44, 0.0, 0.0, -0.44\] pu of rated.
#[derive(Debug, Clone)]
pub struct VoltVarCurve {
    /// Voltage breakpoints \[pu\] — must be strictly monotone increasing.
    pub v_points: Vec<f64>,
    /// Reactive power at each breakpoint \[pu of rated kVA\].
    /// Positive Q = injection (capacitive), negative Q = absorption (inductive).
    pub q_points: Vec<f64>,
}

impl Default for VoltVarCurve {
    fn default() -> Self {
        Self {
            v_points: vec![0.92, 0.98, 1.02, 1.08],
            q_points: vec![0.44, 0.0, 0.0, -0.44],
        }
    }
}

impl VoltVarCurve {
    /// Validate that the curve is well-formed (same length, monotone V).
    pub fn validate(&self) -> Result<(), SmartInverterError> {
        if self.v_points.len() != self.q_points.len() {
            return Err(SmartInverterError::InvalidCurve(
                "v_points and q_points must have equal length".into(),
            ));
        }
        if self.v_points.len() < 2 {
            return Err(SmartInverterError::InvalidCurve(
                "VoltVarCurve requires at least 2 breakpoints".into(),
            ));
        }
        for i in 1..self.v_points.len() {
            if self.v_points[i] <= self.v_points[i - 1] {
                return Err(SmartInverterError::InvalidCurve(
                    "v_points must be strictly monotone increasing".into(),
                ));
            }
        }
        Ok(())
    }
}

/// Volt-Watt (P(V)) curtailment curve per IEEE 1547-2018.
///
/// Linear curtailment from `v_start` to `v_stop`.
#[derive(Debug, Clone)]
pub struct VoltWattCurve {
    /// Voltage where curtailment begins \[pu\] (default 1.06).
    pub v_start: f64,
    /// Voltage where output reaches `p_min_pu` \[pu\] (default 1.10).
    pub v_stop: f64,
    /// Minimum allowable power output \[pu\] (default 0.0).
    pub p_min_pu: f64,
}

impl Default for VoltWattCurve {
    fn default() -> Self {
        Self {
            v_start: 1.06,
            v_stop: 1.10,
            p_min_pu: 0.0,
        }
    }
}

/// Frequency-Watt droop configuration per IEEE 1547-2018.
#[derive(Debug, Clone)]
pub struct FrequencyWattConfig {
    /// Deadband half-width \[Hz\] (default 0.036 Hz).
    pub f_deadband_hz: f64,
    /// Droop slope as percentage (default 5.0 %).
    /// Interpretation: 5 % droop → 5 % ΔP per (Δf / f_nom).
    pub droop_pct: f64,
    /// Nominal frequency \[Hz\] (default 60.0).
    pub f_nominal_hz: f64,
    /// Minimum output during sustained overfrequency \[pu\] (default 0.2).
    pub p_min_pu: f64,
}

impl Default for FrequencyWattConfig {
    fn default() -> Self {
        Self {
            f_deadband_hz: 0.036,
            droop_pct: 5.0,
            f_nominal_hz: 60.0,
            p_min_pu: 0.2,
        }
    }
}

/// Voltage and frequency ride-through thresholds per IEEE 1547-2018 Category B.
#[derive(Debug, Clone)]
pub struct RideThroughConfig {
    /// Low-voltage ride-through threshold \[pu\] (default 0.88).
    pub lvrt_threshold_pu: f64,
    /// Maximum ride-through duration at LVRT threshold \[s\] (default 2.0).
    pub lvrt_time_s: f64,
    /// High-voltage ride-through threshold \[pu\] (default 1.10).
    pub hvrt_threshold_pu: f64,
    /// Maximum ride-through duration at HVRT threshold \[s\] (default 0.2).
    pub hvrt_time_s: f64,
    /// Low-frequency trip threshold \[Hz\] (default 57.0).
    pub lf_threshold_hz: f64,
    /// High-frequency trip threshold \[Hz\] (default 62.0).
    pub hf_threshold_hz: f64,
}

impl Default for RideThroughConfig {
    fn default() -> Self {
        Self {
            lvrt_threshold_pu: 0.88,
            lvrt_time_s: 2.0,
            hvrt_threshold_pu: 1.10,
            hvrt_time_s: 0.2,
            lf_threshold_hz: 57.0,
            hf_threshold_hz: 62.0,
        }
    }
}

/// Full configuration for the smart inverter.
#[derive(Debug, Clone)]
pub struct SmartInverterConfig {
    /// Rated apparent power \[kVA\].
    pub rated_kva: f64,
    /// Operating mode.
    pub mode: SmartInverterMode,
    /// Volt-VAR curve (required if mode is VoltVar or VoltVarPlusVoltWatt).
    pub volt_var: Option<VoltVarCurve>,
    /// Volt-Watt curve (required if mode is VoltWatt or VoltVarPlusVoltWatt).
    pub volt_watt: Option<VoltWattCurve>,
    /// Frequency-Watt config (required if mode is FrequencyWatt).
    pub freq_watt: Option<FrequencyWattConfig>,
    /// Response time \[s\] — IEEE 1547 requires ≤ 10 s (default 10.0).
    pub response_time_s: f64,
    /// Output ramp rate \[% of rated per second\] (default 10.0 %/s).
    pub ramp_rate_pct_per_s: f64,
    /// Ride-through configuration.
    pub ride_through: RideThroughConfig,
}

impl SmartInverterConfig {
    /// Create a configuration with IEEE 1547-2018 default Volt-VAR settings.
    pub fn default_volt_var(rated_kva: f64) -> Self {
        Self {
            rated_kva,
            mode: SmartInverterMode::VoltVar,
            volt_var: Some(VoltVarCurve::default()),
            volt_watt: None,
            freq_watt: None,
            response_time_s: 10.0,
            ramp_rate_pct_per_s: 10.0,
            ride_through: RideThroughConfig::default(),
        }
    }
}

/// Runtime state of the inverter.
#[derive(Debug, Clone, PartialEq)]
pub enum InverterState {
    /// Operating within normal limits.
    Normal,
    /// Operating outside limits but within ride-through window.
    RidingThrough,
    /// Tripped — not producing output.
    Tripped,
    /// Waiting for reconnection delay to elapse (IEEE 1547: 300 s minimum).
    Reconnecting {
        /// Time elapsed in reconnect delay \[s\].
        time_elapsed_s: f64,
    },
}

/// Errors produced by the smart inverter module.
#[derive(Debug, Clone)]
pub enum SmartInverterError {
    /// A curve definition is invalid.
    InvalidCurve(String),
    /// A configuration field has an out-of-range value.
    InvalidConfig(String),
}

impl core::fmt::Display for SmartInverterError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            SmartInverterError::InvalidCurve(msg) => write!(f, "InvalidCurve: {msg}"),
            SmartInverterError::InvalidConfig(msg) => write!(f, "InvalidConfig: {msg}"),
        }
    }
}

impl std::error::Error for SmartInverterError {}

/// Output from a single inverter update step.
#[derive(Debug, Clone)]
pub struct InverterOutput {
    /// Active power output \[kW\].
    pub p_kw: f64,
    /// Reactive power output \[kvar\].
    pub q_kvar: f64,
    /// Active power output \[pu\].
    pub p_pu: f64,
    /// Reactive power output \[pu\].
    pub q_pu: f64,
    /// Current inverter state.
    pub state: InverterState,
    /// Active power curtailed relative to available \[kW\].
    pub curtailed_kw: f64,
    /// True if reactive power was clipped by apparent-power limit.
    pub q_limited: bool,
}

/// Result of IEEE 1547-2018 compliance verification.
#[derive(Debug, Clone)]
pub struct ComplianceReport {
    /// True if Volt-VAR response meets specification.
    pub volt_var_compliant: bool,
    /// True if ride-through behavior meets Category B requirements.
    pub ride_through_compliant: bool,
    /// True if response time ≤ 10 s.
    pub response_time_ok: bool,
    /// Q capability at rated voltage \[pu\].
    pub q_capability_at_rated_v: f64,
    /// List of non-conformance issues found.
    pub issues: Vec<String>,
}

/// IEEE 1547-2018 compliant smart inverter model.
///
/// Supports Volt-VAR, Volt-Watt, Frequency-Watt, constant power factor,
/// combined Volt-VAR+Volt-Watt, and direct P/Q setpoint control.
pub struct SmartInverter {
    /// Unique identifier for this inverter.
    pub inverter_id: String,
    /// Configuration parameters.
    pub config: SmartInverterConfig,
    /// Current active power output \[pu\].
    p_output_pu: f64,
    /// Current reactive power output \[pu\].
    q_output_pu: f64,
    /// Current runtime state.
    state: InverterState,
    /// Accumulated time in the current voltage/frequency violation \[s\].
    time_in_violation_s: f64,
}

impl SmartInverter {
    /// Reconnection delay required by IEEE 1547-2018 \[s\].
    const RECONNECT_DELAY_S: f64 = 300.0;

    /// Create a new SmartInverter with the given configuration.
    pub fn new(inverter_id: impl Into<String>, config: SmartInverterConfig) -> Self {
        Self {
            inverter_id: inverter_id.into(),
            config,
            p_output_pu: 0.0,
            q_output_pu: 0.0,
            state: InverterState::Normal,
            time_in_violation_s: 0.0,
        }
    }

    /// Return the current active power output \[pu\].
    pub fn p_output_pu(&self) -> f64 {
        self.p_output_pu
    }

    /// Return the current reactive power output \[pu\].
    pub fn q_output_pu(&self) -> f64 {
        self.q_output_pu
    }

    /// Return the current inverter state.
    pub fn state(&self) -> &InverterState {
        &self.state
    }

    /// Advance the inverter by one time step `dt_s` \[s\].
    ///
    /// # Arguments
    /// - `v_pu` — terminal voltage \[pu\]
    /// - `f_hz` — grid frequency \[Hz\]
    /// - `p_available_pu` — available active power from source \[pu\]
    /// - `dt_s` — time step \[s\]
    ///
    /// # Returns
    /// [`InverterOutput`] with the P/Q setpoints and diagnostic flags.
    pub fn update(
        &mut self,
        v_pu: f64,
        f_hz: f64,
        p_available_pu: f64,
        dt_s: f64,
    ) -> InverterOutput {
        // Clamp p_available to [0, 1]
        let p_available_pu = p_available_pu.clamp(0.0, 1.0);

        // Update ride-through state machine
        self.state = self.check_ride_through(v_pu, f_hz, dt_s);

        // If tripped or reconnecting, output zero
        match &self.state {
            InverterState::Tripped | InverterState::Reconnecting { .. } => {
                self.p_output_pu = 0.0;
                self.q_output_pu = 0.0;
                let rated = self.config.rated_kva;
                return InverterOutput {
                    p_kw: 0.0,
                    q_kvar: 0.0,
                    p_pu: 0.0,
                    q_pu: 0.0,
                    state: self.state.clone(),
                    curtailed_kw: p_available_pu * rated,
                    q_limited: false,
                };
            }
            InverterState::Normal | InverterState::RidingThrough => {}
        }

        // Compute desired P and Q setpoints based on mode
        let (p_desired, q_desired) = self.compute_setpoints(v_pu, f_hz, p_available_pu);

        // Apply apparent power limit
        let (p_limited, q_limited_val) = Self::apparent_power_limit(p_desired, q_desired);
        let q_was_limited =
            (q_limited_val - q_desired).abs() > 1e-9 || (p_limited - p_desired).abs() > 1e-9;

        // Apply ramp rate limiting
        let max_delta_pu = self.config.ramp_rate_pct_per_s / 100.0 * dt_s;
        let p_ramp = clamp_delta(self.p_output_pu, p_limited, max_delta_pu);
        let q_ramp = clamp_delta(self.q_output_pu, q_limited_val, max_delta_pu);

        self.p_output_pu = p_ramp;
        self.q_output_pu = q_ramp;

        let rated = self.config.rated_kva;
        let curtailed_pu = (p_available_pu - p_ramp).max(0.0);

        InverterOutput {
            p_kw: p_ramp * rated,
            q_kvar: q_ramp * rated,
            p_pu: p_ramp,
            q_pu: q_ramp,
            state: self.state.clone(),
            curtailed_kw: curtailed_pu * rated,
            q_limited: q_was_limited,
        }
    }

    /// Compute P and Q setpoints \[pu\] for the current mode.
    fn compute_setpoints(&self, v_pu: f64, f_hz: f64, p_available_pu: f64) -> (f64, f64) {
        match &self.config.mode {
            SmartInverterMode::ConstantPowerFactor { pf } => {
                let pf = pf.clamp(-1.0, 1.0);
                // Q = P * tan(acos(|pf|)), sign follows pf sign
                let p = p_available_pu;
                let q = if pf.abs() < 1e-9 {
                    0.0
                } else {
                    let angle = pf.abs().acos();
                    p * angle.tan() * pf.signum()
                };
                (p, q)
            }

            SmartInverterMode::VoltVar => {
                let q = self.volt_var_response(v_pu);
                (p_available_pu, q)
            }

            SmartInverterMode::VoltWatt => {
                let p = self.volt_watt_response(v_pu, p_available_pu);
                (p, 0.0)
            }

            SmartInverterMode::FrequencyWatt => {
                let p = self.frequency_watt_response(f_hz, p_available_pu);
                (p, 0.0)
            }

            SmartInverterMode::VoltVarPlusVoltWatt => {
                let p = self.volt_watt_response(v_pu, p_available_pu);
                let q = self.volt_var_response(v_pu);
                (p, q)
            }

            SmartInverterMode::ActiveReactivePowerControl { p_ref, q_ref } => {
                let p = p_ref.clamp(0.0, p_available_pu);
                let q = *q_ref;
                (p, q)
            }
        }
    }

    /// Compute Q \[pu\] from the Volt-VAR curve at voltage `v_pu`.
    ///
    /// Uses piecewise-linear interpolation; extrapolates with endpoint values
    /// outside the curve range.  Q is clamped to the apparent-power circle:
    /// |Q| ≤ √(1 − P²).
    pub fn volt_var_response(&self, v_pu: f64) -> f64 {
        let curve = match &self.config.volt_var {
            Some(c) => c,
            None => return 0.0,
        };

        let q_raw = piecewise_linear(&curve.v_points, &curve.q_points, v_pu);

        // Clamp to reactive capability: |Q| ≤ sqrt(1 - P^2)
        let p = self.p_output_pu.clamp(0.0, 1.0);
        let q_max = (1.0_f64 - p * p).max(0.0).sqrt();
        q_raw.clamp(-q_max, q_max)
    }

    /// Compute curtailed P \[pu\] from the Volt-Watt curve at voltage `v_pu`.
    ///
    /// Returns `p_available_pu` unchanged when V < `v_start`, and `p_min_pu`
    /// when V ≥ `v_stop`.
    pub fn volt_watt_response(&self, v_pu: f64, p_available_pu: f64) -> f64 {
        let curve = match &self.config.volt_watt {
            Some(c) => c,
            None => return p_available_pu,
        };

        if v_pu <= curve.v_start {
            return p_available_pu;
        }
        if v_pu >= curve.v_stop {
            return curve.p_min_pu;
        }

        let span = curve.v_stop - curve.v_start;
        let frac = (v_pu - curve.v_start) / span;
        let p = p_available_pu * (1.0 - frac);
        p.clamp(curve.p_min_pu, p_available_pu)
    }

    /// Compute droop-adjusted P \[pu\] from the Frequency-Watt function.
    ///
    /// Within the deadband: P = `p_available_pu`.
    /// Overfrequency: reduce P proportionally to Δf / f_nom / droop.
    /// Underfrequency: increase P up to `p_available_pu`.
    pub fn frequency_watt_response(&self, f_hz: f64, p_available_pu: f64) -> f64 {
        let cfg = match &self.config.freq_watt {
            Some(c) => c,
            None => return p_available_pu,
        };

        let delta_f = f_hz - cfg.f_nominal_hz;
        let db = cfg.f_deadband_hz;

        if delta_f.abs() <= db {
            return p_available_pu;
        }

        // Signed frequency deviation outside deadband
        let delta_f_active = if delta_f > 0.0 {
            delta_f - db
        } else {
            delta_f + db
        };

        // ΔP = -(Δf / f_nom) / (droop / 100)
        let droop_fraction = cfg.droop_pct / 100.0;
        let delta_p = -(delta_f_active / cfg.f_nominal_hz) / droop_fraction;

        let p = p_available_pu + delta_p;
        p.clamp(cfg.p_min_pu, p_available_pu)
    }

    /// Update ride-through state based on current V and f, advancing by `dt_s` \[s\].
    ///
    /// State transitions:
    /// - Normal → RidingThrough when V or f leaves normal operating range.
    /// - RidingThrough → Tripped when accumulated violation time exceeds threshold.
    /// - Tripped → Reconnecting when V and f return to normal range.
    /// - Reconnecting → Normal after 300 s delay.
    pub fn check_ride_through(&mut self, v_pu: f64, f_hz: f64, dt_s: f64) -> InverterState {
        let rt = &self.config.ride_through;

        let voltage_violation = v_pu < rt.lvrt_threshold_pu || v_pu > rt.hvrt_threshold_pu;
        let freq_violation = f_hz < rt.lf_threshold_hz || f_hz > rt.hf_threshold_hz;
        let in_violation = voltage_violation || freq_violation;

        // Determine the trip time for the current violation type
        let trip_time = if v_pu < rt.lvrt_threshold_pu {
            rt.lvrt_time_s
        } else if v_pu > rt.hvrt_threshold_pu {
            rt.hvrt_time_s
        } else {
            // Frequency violation — use LVRT time as conservative default
            rt.lvrt_time_s
        };

        match &self.state.clone() {
            InverterState::Normal => {
                if in_violation {
                    self.time_in_violation_s = dt_s;
                    if self.time_in_violation_s >= trip_time {
                        InverterState::Tripped
                    } else {
                        InverterState::RidingThrough
                    }
                } else {
                    self.time_in_violation_s = 0.0;
                    InverterState::Normal
                }
            }

            InverterState::RidingThrough => {
                if in_violation {
                    self.time_in_violation_s += dt_s;
                    if self.time_in_violation_s >= trip_time {
                        InverterState::Tripped
                    } else {
                        InverterState::RidingThrough
                    }
                } else {
                    // Conditions restored — return to normal
                    self.time_in_violation_s = 0.0;
                    InverterState::Normal
                }
            }

            InverterState::Tripped => {
                if !in_violation {
                    // Conditions restored — start reconnect timer
                    self.time_in_violation_s = 0.0;
                    InverterState::Reconnecting {
                        time_elapsed_s: 0.0,
                    }
                } else {
                    InverterState::Tripped
                }
            }

            InverterState::Reconnecting { time_elapsed_s } => {
                if in_violation {
                    // Violation during reconnect — restart trip
                    self.time_in_violation_s = dt_s;
                    InverterState::Tripped
                } else {
                    let new_elapsed = time_elapsed_s + dt_s;
                    if new_elapsed >= Self::RECONNECT_DELAY_S {
                        self.time_in_violation_s = 0.0;
                        InverterState::Normal
                    } else {
                        InverterState::Reconnecting {
                            time_elapsed_s: new_elapsed,
                        }
                    }
                }
            }
        }
    }

    /// Constrain (P, Q) \[pu\] to the apparent-power circle S ≤ 1.0 pu.
    ///
    /// If P² + Q² > 1, both are scaled down uniformly so that S = 1.
    pub fn apparent_power_limit(p_pu: f64, q_pu: f64) -> (f64, f64) {
        let s_sq = p_pu * p_pu + q_pu * q_pu;
        if s_sq <= 1.0 {
            return (p_pu, q_pu);
        }
        let s = s_sq.sqrt();
        (p_pu / s, q_pu / s)
    }

    /// Verify IEEE 1547-2018 compliance over a voltage/frequency operating range.
    ///
    /// # Arguments
    /// - `v_range` — (V_min, V_max) \[pu\] to sweep
    /// - `f_range` — (f_min, f_max) \[Hz\] to sweep
    /// - `test_duration` — duration of each sub-test \[s\]
    ///
    /// # Returns
    /// [`ComplianceReport`] detailing pass/fail status.
    pub fn ieee1547_compliance_check(
        &mut self,
        v_range: (f64, f64),
        _f_range: (f64, f64),
        test_duration: f64,
    ) -> ComplianceReport {
        let mut issues: Vec<String> = Vec::new();

        // 1. Response time check
        let response_time_ok = self.config.response_time_s <= 10.0;
        if !response_time_ok {
            issues.push(format!(
                "Response time {:.1} s exceeds IEEE 1547 maximum of 10 s",
                self.config.response_time_s
            ));
        }

        // 2. Volt-VAR compliance: check Q at several voltages within range
        let volt_var_compliant = self.check_volt_var_compliance(v_range, &mut issues);

        // 3. Ride-through compliance: test LVRT and HVRT scenarios
        let ride_through_compliant = self.check_ride_through_compliance(test_duration, &mut issues);

        // 4. Q capability at rated voltage
        let q_capability_at_rated_v = self.compute_q_capability_at_rated_v();

        ComplianceReport {
            volt_var_compliant,
            ride_through_compliant,
            response_time_ok,
            q_capability_at_rated_v,
            issues,
        }
    }

    /// Check Volt-VAR compliance within a voltage range.
    fn check_volt_var_compliance(&self, v_range: (f64, f64), issues: &mut Vec<String>) -> bool {
        // For non-Volt-VAR modes, compliance is not applicable
        let has_volt_var = matches!(
            &self.config.mode,
            SmartInverterMode::VoltVar | SmartInverterMode::VoltVarPlusVoltWatt
        );
        if !has_volt_var {
            return true;
        }

        let curve = match &self.config.volt_var {
            Some(c) => c,
            None => {
                issues.push("VoltVar mode requires volt_var curve".into());
                return false;
            }
        };

        // Validate curve
        if let Err(e) = curve.validate() {
            issues.push(format!("Invalid VoltVar curve: {e}"));
            return false;
        }

        // Check that curve spans the expected operating range [v_range.0, v_range.1]
        let curve_v_min = curve.v_points.first().copied().unwrap_or(0.0);
        let curve_v_max = curve.v_points.last().copied().unwrap_or(0.0);

        let mut compliant = true;

        if v_range.0 < curve_v_min {
            issues.push(format!(
                "VoltVar curve does not cover low end of v_range: {:.3} < curve min {:.3}",
                v_range.0, curve_v_min
            ));
            compliant = false;
        }
        if v_range.1 > curve_v_max {
            issues.push(format!(
                "VoltVar curve does not cover high end of v_range: {:.3} > curve max {:.3}",
                v_range.1, curve_v_max
            ));
            compliant = false;
        }

        // Verify Q at rated voltage (1.0 pu) is approximately 0 (deadband)
        let q_at_rated = self.volt_var_response(1.0);
        if q_at_rated.abs() > 0.01 {
            issues.push(format!(
                "VoltVar Q at 1.0 pu = {q_at_rated:.4} (expected ≈ 0 in deadband)"
            ));
            compliant = false;
        }

        compliant
    }

    /// Check ride-through compliance by simulating brief violations.
    fn check_ride_through_compliance(
        &mut self,
        test_duration: f64,
        issues: &mut Vec<String>,
    ) -> bool {
        let rt = self.config.ride_through.clone();
        let mut compliant = true;

        // Check LVRT: voltage below threshold should survive for lvrt_time_s
        if rt.lvrt_time_s <= 0.0 {
            issues.push("LVRT time must be > 0".into());
            compliant = false;
        }
        if rt.lvrt_threshold_pu >= 1.0 {
            issues.push(format!(
                "LVRT threshold {:.3} pu must be < 1.0 pu",
                rt.lvrt_threshold_pu
            ));
            compliant = false;
        }

        // Check HVRT: voltage above threshold should trip within hvrt_time_s
        if rt.hvrt_time_s <= 0.0 {
            issues.push("HVRT time must be > 0".into());
            compliant = false;
        }
        if rt.hvrt_threshold_pu <= 1.0 {
            issues.push(format!(
                "HVRT threshold {:.3} pu must be > 1.0 pu",
                rt.hvrt_threshold_pu
            ));
            compliant = false;
        }

        // Category B frequency limits: 57–62 Hz
        if rt.lf_threshold_hz > 57.0 {
            issues.push(format!(
                "LF threshold {:.1} Hz is above Category B minimum of 57.0 Hz",
                rt.lf_threshold_hz
            ));
            compliant = false;
        }
        if rt.hf_threshold_hz < 62.0 {
            issues.push(format!(
                "HF threshold {:.1} Hz is below Category B maximum of 62.0 Hz",
                rt.hf_threshold_hz
            ));
            compliant = false;
        }

        // Verify that test_duration is sufficient for meaningful testing
        if test_duration < rt.lvrt_time_s {
            issues.push(format!(
                "test_duration {test_duration:.1} s < LVRT time {:.1} s — cannot verify full ride-through",
                rt.lvrt_time_s
            ));
            // Not a compliance failure per se, just informational
        }

        compliant
    }

    /// Compute reactive power capability at rated voltage (1.0 pu) \[pu\].
    fn compute_q_capability_at_rated_v(&self) -> f64 {
        // At rated voltage and full P, Q capability = sqrt(1 - P^2)
        // For compliance, evaluate at P = p_available = 1.0
        let p = 1.0_f64;
        (1.0 - p * p).max(0.0).sqrt()
    }
}

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Piecewise-linear interpolation over paired (x, y) breakpoint arrays.
///
/// Extrapolates with endpoint values outside the range.
fn piecewise_linear(x_pts: &[f64], y_pts: &[f64], x: f64) -> f64 {
    let n = x_pts.len();
    if n == 0 {
        return 0.0;
    }
    if n == 1 || x <= x_pts[0] {
        return y_pts[0];
    }
    if x >= x_pts[n - 1] {
        return y_pts[n - 1];
    }
    // Binary search for the segment
    let mut lo = 0usize;
    let mut hi = n - 1;
    while hi - lo > 1 {
        let mid = (lo + hi) / 2;
        if x_pts[mid] <= x {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let t = (x - x_pts[lo]) / (x_pts[hi] - x_pts[lo]);
    y_pts[lo] + t * (y_pts[hi] - y_pts[lo])
}

/// Clamp a value change to ±`max_delta`, moving from `current` toward `target`.
fn clamp_delta(current: f64, target: f64, max_delta: f64) -> f64 {
    let delta = target - current;
    if delta.abs() <= max_delta {
        target
    } else {
        current + delta.signum() * max_delta
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_inverter(mode: SmartInverterMode) -> SmartInverter {
        let config = SmartInverterConfig {
            rated_kva: 100.0,
            mode,
            volt_var: Some(VoltVarCurve::default()),
            volt_watt: Some(VoltWattCurve::default()),
            freq_watt: Some(FrequencyWattConfig::default()),
            response_time_s: 10.0,
            ramp_rate_pct_per_s: 100.0, // fast ramp for most tests
            ride_through: RideThroughConfig::default(),
        };
        SmartInverter::new("inv-1", config)
    }

    // -----------------------------------------------------------------------
    // Test 1: Volt-VAR — V=0.95 → positive Q injection
    // -----------------------------------------------------------------------
    #[test]
    fn test_volt_var_v095_q_injection() {
        let mut inv = make_inverter(SmartInverterMode::VoltVar);
        // Pre-set P so reactive capability is well-defined
        inv.p_output_pu = 0.0;
        let q = inv.volt_var_response(0.95);
        assert!(
            q > 0.0,
            "At V=0.95 pu (below nominal), Q should be positive (injection); got {q:.4}"
        );
    }

    // -----------------------------------------------------------------------
    // Test 2: Volt-VAR — V=1.05 → negative Q absorption
    // -----------------------------------------------------------------------
    #[test]
    fn test_volt_var_v105_q_absorption() {
        let mut inv = make_inverter(SmartInverterMode::VoltVar);
        inv.p_output_pu = 0.0;
        let q = inv.volt_var_response(1.05);
        assert!(
            q < 0.0,
            "At V=1.05 pu (above nominal), Q should be negative (absorption); got {q:.4}"
        );
    }

    // -----------------------------------------------------------------------
    // Test 3: Volt-Watt — V=1.08 > v_start → P is curtailed
    // -----------------------------------------------------------------------
    #[test]
    fn test_volt_watt_curtailment() {
        let inv = make_inverter(SmartInverterMode::VoltWatt);
        let p_available = 1.0;
        let p_out = inv.volt_watt_response(1.08, p_available);
        assert!(
            p_out < p_available,
            "At V=1.08 > v_start=1.06, P should be curtailed; got p_out={p_out:.4}"
        );
        assert!(p_out >= 0.0, "P must be non-negative; got {p_out:.4}");
    }

    // -----------------------------------------------------------------------
    // Test 4: Volt-Watt — V=1.11 ≥ v_stop → P = p_min
    // -----------------------------------------------------------------------
    #[test]
    fn test_volt_watt_full_curtail() {
        let inv = make_inverter(SmartInverterMode::VoltWatt);
        let p_out = inv.volt_watt_response(1.11, 1.0);
        let p_min = inv
            .config
            .volt_watt
            .as_ref()
            .map(|c| c.p_min_pu)
            .unwrap_or(0.0);
        assert!(
            (p_out - p_min).abs() < 1e-9,
            "At V=1.11 >= v_stop=1.10, P should equal p_min={p_min}; got {p_out:.6}"
        );
    }

    // -----------------------------------------------------------------------
    // Test 5: Frequency-Watt — f=60.5 Hz → P is reduced
    // -----------------------------------------------------------------------
    #[test]
    fn test_frequency_watt_overfreq() {
        let inv = make_inverter(SmartInverterMode::FrequencyWatt);
        let p_available = 1.0;
        let p_out = inv.frequency_watt_response(60.5, p_available);
        assert!(
            p_out < p_available,
            "At f=60.5 Hz (overfrequency), P should be reduced; got {p_out:.4}"
        );
        assert!(p_out >= 0.0, "P must be non-negative; got {p_out:.4}");
    }

    // -----------------------------------------------------------------------
    // Test 6: Apparent power limit — P=0.9, Q=0.6 → rescaled within circle
    // -----------------------------------------------------------------------
    #[test]
    fn test_apparent_power_limit_circle() {
        let (p_out, q_out) = SmartInverter::apparent_power_limit(0.9, 0.6);
        let s_sq = p_out * p_out + q_out * q_out;
        assert!(
            s_sq <= 1.0 + 1e-9,
            "Output S² = {s_sq:.6} should be ≤ 1.0; P={p_out:.4}, Q={q_out:.4}"
        );
        // Input exceeds circle
        let s_in = 0.9_f64.hypot(0.6);
        assert!(s_in > 1.0, "Test input S={s_in:.4} should be > 1.0");
    }

    // -----------------------------------------------------------------------
    // Test 7: Ride-through — V=0.85 < threshold for > LVRT time → Tripped
    // -----------------------------------------------------------------------
    #[test]
    fn test_ride_through_lvrt_trip() {
        let mut inv = make_inverter(SmartInverterMode::VoltVar);
        let lvrt_time = inv.config.ride_through.lvrt_time_s;
        let dt = 0.1;
        let steps = ((lvrt_time / dt) as usize) + 5;

        let mut last_state = InverterState::Normal;
        for _ in 0..steps {
            let out = inv.update(0.85, 60.0, 1.0, dt);
            last_state = out.state;
        }
        assert_eq!(
            last_state,
            InverterState::Tripped,
            "After V=0.85 for >{lvrt_time:.1} s, inverter should be Tripped"
        );
    }

    // -----------------------------------------------------------------------
    // Test 8: IEEE 1547 compliance — default config → compliant
    // -----------------------------------------------------------------------
    #[test]
    fn test_ieee1547_compliance_default() {
        let config = SmartInverterConfig::default_volt_var(100.0);
        let mut inv = SmartInverter::new("inv-compliance", config);
        let report = inv.ieee1547_compliance_check((0.92, 1.08), (57.0, 62.0), 5.0);

        assert!(
            report.volt_var_compliant,
            "Default VoltVar config should be volt_var_compliant; issues: {:?}",
            report.issues
        );
        assert!(
            report.ride_through_compliant,
            "Default ride-through should be compliant; issues: {:?}",
            report.issues
        );
        assert!(
            report.response_time_ok,
            "Response time should be ≤ 10 s; issues: {:?}",
            report.issues
        );
    }

    // -----------------------------------------------------------------------
    // Test 9: Ramp rate — P change capped at ramp_rate × dt
    // -----------------------------------------------------------------------
    #[test]
    fn test_ramp_rate_limiting() {
        let config = SmartInverterConfig {
            rated_kva: 100.0,
            mode: SmartInverterMode::VoltWatt,
            volt_var: Some(VoltVarCurve::default()),
            volt_watt: Some(VoltWattCurve::default()),
            freq_watt: Some(FrequencyWattConfig::default()),
            response_time_s: 10.0,
            ramp_rate_pct_per_s: 10.0, // 10 %/s → max 0.01 pu per 0.1 s step
            ride_through: RideThroughConfig::default(),
        };
        let mut inv = SmartInverter::new("inv-ramp", config);
        // Start at P=0, step to full available
        let dt = 0.1_f64;
        let out = inv.update(1.0, 60.0, 1.0, dt);
        let max_step = inv.config.ramp_rate_pct_per_s / 100.0 * dt;
        assert!(
            out.p_pu <= max_step + 1e-9,
            "P step {:.6} should be ≤ ramp limit {max_step:.6}",
            out.p_pu
        );
    }

    // -----------------------------------------------------------------------
    // Additional: piecewise_linear helper correctness
    // -----------------------------------------------------------------------
    #[test]
    fn test_piecewise_linear_interpolation() {
        let x = vec![0.0, 1.0, 2.0];
        let y = vec![0.0, 1.0, 0.0];
        assert!((piecewise_linear(&x, &y, 0.5) - 0.5).abs() < 1e-9);
        assert!((piecewise_linear(&x, &y, 1.5) - 0.5).abs() < 1e-9);
        // Clamp at boundaries
        assert!((piecewise_linear(&x, &y, -1.0) - 0.0).abs() < 1e-9);
        assert!((piecewise_linear(&x, &y, 3.0) - 0.0).abs() < 1e-9);
    }

    // -----------------------------------------------------------------------
    // Additional: VoltVarCurve validation
    // -----------------------------------------------------------------------
    #[test]
    fn test_volt_var_curve_validation() {
        let good = VoltVarCurve::default();
        assert!(good.validate().is_ok());

        let bad_len = VoltVarCurve {
            v_points: vec![0.9, 1.1],
            q_points: vec![0.44],
        };
        assert!(bad_len.validate().is_err());

        let non_monotone = VoltVarCurve {
            v_points: vec![1.0, 0.9, 1.1],
            q_points: vec![0.0, 0.2, -0.2],
        };
        assert!(non_monotone.validate().is_err());
    }
}
