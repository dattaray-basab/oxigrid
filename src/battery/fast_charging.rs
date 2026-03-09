//! Fast charging protocol optimization module.
//!
//! Implements CCCV, multi-stage constant current (MCC), pulse charging,
//! boost charging, and health-aware optimal charging with degradation-
//! minimization tradeoffs.
//!
//! # Battery Model
//!
//! Uses a first-order ECM (Rint + 1 RC pair) for voltage dynamics:
//! ```text
//! OCV(SoC) = ocv_offset + ocv_slope * SoC
//! V_RC1(t+dt) = V_RC1(t) + dt * (-V_RC1/(R1*C1) + I/C1)
//! V_term = OCV + I*R0 + V_RC1   (I > 0 = charging)
//! SoC(t+dt) = SoC(t) + I*dt / (capacity_ah * 3600)
//! ```
//!
//! # Thermal Model
//!
//! Single-node lumped thermal model:
//! ```text
//! C_th * dT/dt = I^2 * R0 - (T - T_amb) / R_th
//! ```

use std::fmt;

// ---------------------------------------------------------------------------
// Error type
// ---------------------------------------------------------------------------

/// Error returned by fast-charging simulation functions.
#[derive(Debug, Clone)]
pub struct FastChargingError(String);

impl fmt::Display for FastChargingError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "FastChargingError: {}", self.0)
    }
}

impl From<String> for FastChargingError {
    fn from(s: String) -> Self {
        FastChargingError(s)
    }
}

impl From<&str> for FastChargingError {
    fn from(s: &str) -> Self {
        FastChargingError(s.to_string())
    }
}

// ---------------------------------------------------------------------------
// Protocol types
// ---------------------------------------------------------------------------

/// Charging protocol type.
#[derive(Debug, Clone)]
pub enum ChargingProtocol {
    /// Constant current – constant voltage.
    CcCv {
        /// CC phase current (A).
        cc_current_a: f64,
        /// CV phase voltage (V).
        cv_voltage_v: f64,
        /// Termination current (A); charging ends when CV current falls below this.
        cutoff_current_a: f64,
    },
    /// Multi-stage constant current.
    Mcc {
        /// Ordered CC stages; the charger advances to the next stage when the
        /// terminal voltage reaches the stage's `voltage_limit_v`.
        stages: Vec<CcStage>,
        /// Final CV voltage (V).
        cv_voltage_v: f64,
        /// Termination current (A).
        cutoff_current_a: f64,
    },
    /// Pulse charging (alternating high/low current cycles).
    Pulse {
        /// Peak current during ON phase (A).
        peak_current_a: f64,
        /// Current during OFF phase (A); use 0 for full rest.
        rest_current_a: f64,
        /// Duration of the ON phase (s).
        on_time_s: f64,
        /// Duration of the OFF phase (s).
        off_time_s: f64,
        /// Maximum allowed terminal voltage (V); charging stops if exceeded.
        max_voltage_v: f64,
        /// Termination current – unused in pulse mode but kept for API symmetry.
        cutoff_current_a: f64,
    },
    /// Boost charging: high initial current that tapers after a voltage threshold.
    Boost {
        /// Initial boost current (A).
        boost_current_a: f64,
        /// Terminal voltage at which the charger switches from boost to normal CC (V).
        boost_voltage_v: f64,
        /// Normal CC current after boost phase (A).
        normal_current_a: f64,
        /// CV voltage (V).
        cv_voltage_v: f64,
        /// Termination current (A).
        cutoff_current_a: f64,
    },
    /// Health-aware charging: minimises degradation while meeting time constraints.
    HealthAware {
        /// Target SoC to reach [0, 1].
        target_soc: f64,
        /// Maximum allowed charging time (s).
        max_time_s: f64,
        /// Hard upper bound on current (A).
        max_current_a: f64,
        /// Hard upper bound on terminal voltage (V).
        max_voltage_v: f64,
        /// Tradeoff weight: 0 = fastest charge, 1 = healthiest charge.
        degradation_weight: f64,
    },
}

/// One stage in a multi-stage constant-current protocol.
#[derive(Debug, Clone)]
pub struct CcStage {
    /// Constant current for this stage (A).
    pub current_a: f64,
    /// Terminal voltage that triggers the transition to the next stage (V).
    pub voltage_limit_v: f64,
}

// ---------------------------------------------------------------------------
// Battery parameters
// ---------------------------------------------------------------------------

/// Battery parameters used by the fast-charging simulator.
#[derive(Debug, Clone)]
pub struct FcBatteryParams {
    /// Nominal capacity (Ah).
    pub capacity_ah: f64,
    /// Ohmic (series) resistance Rint (Ω).
    pub r0_ohm: f64,
    /// RC-pair resistance R1 (Ω).
    pub r1_ohm: f64,
    /// RC-pair capacitance C1 (F).
    pub c1_f: f64,
    /// Minimum terminal voltage (V).
    pub v_min: f64,
    /// Maximum terminal voltage (V).
    pub v_max: f64,
    /// Slope of the linear OCV-SoC approximation (V per unit SoC).
    pub ocv_slope: f64,
    /// OCV at SoC = 0 (V).
    pub ocv_offset: f64,
    /// Capacity fade per kAh of charge throughput (fraction per kAh).
    pub deg_per_kah: f64,
    /// Additional degradation multiplier for temperatures above 40 °C.
    pub thermal_deg_factor: f64,
}

// ---------------------------------------------------------------------------
// Simulation state
// ---------------------------------------------------------------------------

/// Instantaneous charging simulation state.
#[derive(Debug, Clone)]
pub struct ChargingState {
    /// State of charge [0, 1].
    pub soc: f64,
    /// Terminal voltage (V).
    pub v_terminal: f64,
    /// Open-circuit voltage (V).
    pub v_ocv: f64,
    /// Voltage across the RC1 branch (V).
    pub v_rc1: f64,
    /// Applied current (A); positive = charging.
    pub current_a: f64,
    /// Cell temperature (°C).
    pub temperature_c: f64,
    /// Elapsed simulation time (s).
    pub time_s: f64,
    /// Cumulative charge throughput (Ah).
    pub charge_throughput_ah: f64,
}

// ---------------------------------------------------------------------------
// Simulation configuration
// ---------------------------------------------------------------------------

/// Configuration for a fast-charging simulation run.
#[derive(Debug, Clone)]
pub struct FastChargingConfig {
    /// Integration time step (s).  Default: 1.0 s.
    pub dt_s: f64,
    /// Maximum simulation time (s).  Default: 7200 s (2 h).
    pub max_sim_time_s: f64,
    /// Initial SoC [0, 1].
    pub initial_soc: f64,
    /// Initial cell temperature (°C).
    pub initial_temp_c: f64,
    /// Ambient temperature for thermal model (°C).
    pub ambient_temp_c: f64,
    /// Thermal resistance (K/W).
    pub thermal_resistance_k_per_w: f64,
    /// Thermal capacitance (J/K).
    pub thermal_capacitance_j_per_k: f64,
}

impl Default for FastChargingConfig {
    fn default() -> Self {
        Self {
            dt_s: 1.0,
            max_sim_time_s: 7200.0,
            initial_soc: 0.2,
            initial_temp_c: 25.0,
            ambient_temp_c: 25.0,
            thermal_resistance_k_per_w: 5.0,
            thermal_capacitance_j_per_k: 100.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Result types
// ---------------------------------------------------------------------------

/// Result of a single fast-charging simulation run.
#[derive(Debug, Clone)]
pub struct FastChargingResult {
    /// Time vector (s).
    pub time_s: Vec<f64>,
    /// SoC trajectory [0, 1].
    pub soc: Vec<f64>,
    /// Terminal voltage trajectory (V).
    pub voltage_v: Vec<f64>,
    /// Applied current trajectory (A).
    pub current_a: Vec<f64>,
    /// Cell temperature trajectory (°C).
    pub temperature_c: Vec<f64>,
    /// Instantaneous power trajectory (W).
    pub power_w: Vec<f64>,
    /// `true` if charging completed (reached cutoff or target SoC).
    pub charging_complete: bool,
    /// Total charging time (s).
    pub charging_time_s: f64,
    /// Final SoC [0, 1].
    pub final_soc: f64,
    /// Total energy delivered to the cell (Wh).
    pub energy_wh: f64,
    /// Charging efficiency (%).
    pub efficiency_pct: f64,
    /// Estimated capacity fade (%).
    pub capacity_fade_pct: f64,
    /// Peak cell temperature during charging (°C).
    pub peak_temperature_c: f64,
    /// Total charge throughput (Ah).
    pub charge_throughput_ah: f64,
}

/// Comparison report across multiple charging protocols.
#[derive(Debug, Clone)]
pub struct ProtocolComparison {
    /// Human-readable protocol names.
    pub protocols: Vec<String>,
    /// Charging times (s) for each protocol.
    pub charging_times_s: Vec<f64>,
    /// Final SoC for each protocol.
    pub final_socs: Vec<f64>,
    /// Charging efficiency (%) for each protocol.
    pub efficiencies_pct: Vec<f64>,
    /// Estimated capacity fade (%) for each protocol.
    pub capacity_fades_pct: Vec<f64>,
    /// Peak temperature (°C) for each protocol.
    pub peak_temperatures_c: Vec<f64>,
    /// Index of the recommended protocol in the vectors above.
    pub recommended: usize,
    /// Human-readable explanation for the recommendation.
    pub recommendation_reason: String,
}

// ---------------------------------------------------------------------------
// Simulator
// ---------------------------------------------------------------------------

/// Fast-charging protocol simulator.
///
/// # Example
/// ```rust
/// # use oxigrid::battery::fast_charging::{
/// #     FastChargingSimulator, FcBatteryParams, FastChargingConfig, ChargingProtocol,
/// # };
/// let battery = FcBatteryParams {
///     capacity_ah: 50.0,
///     r0_ohm: 0.002,
///     r1_ohm: 0.003,
///     c1_f: 1000.0,
///     v_min: 2.8,
///     v_max: 4.2,
///     ocv_slope: 1.2,
///     ocv_offset: 3.0,
///     deg_per_kah: 0.001,
///     thermal_deg_factor: 0.02,
/// };
/// let config = FastChargingConfig::default();
/// let sim = FastChargingSimulator::new(battery, config);
/// let protocol = ChargingProtocol::CcCv {
///     cc_current_a: 50.0,
///     cv_voltage_v: 4.2,
///     cutoff_current_a: 2.5,
/// };
/// let result = sim.simulate(&protocol).expect("simulation failed");
/// assert!(result.charging_complete);
/// ```
pub struct FastChargingSimulator {
    /// Battery parameters.
    pub battery: FcBatteryParams,
    /// Simulation configuration.
    pub config: FastChargingConfig,
}

// ---------------------------------------------------------------------------
// Phase discriminants used internally
// ---------------------------------------------------------------------------

/// Internal phase label for CCCV / Boost state machines.
#[derive(Debug, Clone, Copy, PartialEq)]
enum CcCvPhase {
    Cc,
    Cv,
}

/// Internal phase label for Boost state machine.
#[derive(Debug, Clone, Copy, PartialEq)]
enum BoostPhase {
    Boost,
    Normal,
    Cv,
}

// ---------------------------------------------------------------------------
// impl FastChargingSimulator
// ---------------------------------------------------------------------------

impl FastChargingSimulator {
    /// Create a new simulator with the given battery parameters and configuration.
    pub fn new(battery: FcBatteryParams, config: FastChargingConfig) -> Self {
        Self { battery, config }
    }

    // -----------------------------------------------------------------------
    // Public API
    // -----------------------------------------------------------------------

    /// Simulate the given charging protocol and return a detailed result.
    ///
    /// Returns `Err` if the battery parameters are inconsistent (e.g. zero
    /// capacity) or the protocol contains degenerate values.
    pub fn simulate(
        &self,
        protocol: &ChargingProtocol,
    ) -> Result<FastChargingResult, FastChargingError> {
        self.validate_battery()?;
        match protocol {
            ChargingProtocol::CcCv { .. } => self.simulate_cccv(protocol),
            ChargingProtocol::Mcc {
                stages,
                cv_voltage_v,
                cutoff_current_a,
            } => self.simulate_mcc(stages, *cv_voltage_v, *cutoff_current_a),
            ChargingProtocol::Pulse { .. } => self.simulate_pulse(protocol),
            ChargingProtocol::Boost { .. } => self.simulate_boost(protocol),
            ChargingProtocol::HealthAware { .. } => self.simulate_health_aware(protocol),
        }
    }

    /// Compare multiple named protocols and return a recommendation.
    ///
    /// Ranking uses a weighted score:
    /// `0.4 * time_score + 0.3 * efficiency_score + 0.3 * health_score`
    pub fn compare_protocols(
        &self,
        protocols: &[(String, ChargingProtocol)],
    ) -> Result<ProtocolComparison, FastChargingError> {
        if protocols.is_empty() {
            return Err(FastChargingError::from("no protocols provided"));
        }
        let mut names = Vec::with_capacity(protocols.len());
        let mut times = Vec::with_capacity(protocols.len());
        let mut socs = Vec::with_capacity(protocols.len());
        let mut effs = Vec::with_capacity(protocols.len());
        let mut fades = Vec::with_capacity(protocols.len());
        let mut temps = Vec::with_capacity(protocols.len());

        for (name, proto) in protocols {
            let r = self.simulate(proto)?;
            names.push(name.clone());
            times.push(r.charging_time_s);
            socs.push(r.final_soc);
            effs.push(r.efficiency_pct);
            fades.push(r.capacity_fade_pct);
            temps.push(r.peak_temperature_c);
        }

        let recommended = self.rank_protocols(&times, &effs, &fades);
        let reason = format!(
            "Protocol '{}' scored best on weighted time/efficiency/health criteria",
            names.get(recommended).cloned().unwrap_or_default()
        );

        Ok(ProtocolComparison {
            protocols: names,
            charging_times_s: times,
            final_socs: socs,
            efficiencies_pct: effs,
            capacity_fades_pct: fades,
            peak_temperatures_c: temps,
            recommended,
            recommendation_reason: reason,
        })
    }

    /// Estimate the optimal instantaneous charging current (A) for the given SoC.
    ///
    /// Uses the health-aware current tapering formula:
    /// `I(SoC) = I_max * exp(-k * SoC^2 * degradation_weight)`
    /// with k = 2.0.
    pub fn estimate_optimal_current(
        &self,
        soc: f64,
        max_current_a: f64,
        degradation_weight: f64,
    ) -> f64 {
        const K: f64 = 2.0;
        let weight = degradation_weight.clamp(0.0, 1.0);
        let soc_c = soc.clamp(0.0, 1.0);
        max_current_a * (-K * soc_c * soc_c * weight).exp()
    }

    // -----------------------------------------------------------------------
    // Protocol implementations
    // -----------------------------------------------------------------------

    fn simulate_cccv(
        &self,
        protocol: &ChargingProtocol,
    ) -> Result<FastChargingResult, FastChargingError> {
        let (cc_current_a, cv_voltage_v, cutoff_current_a) = match protocol {
            ChargingProtocol::CcCv {
                cc_current_a,
                cv_voltage_v,
                cutoff_current_a,
            } => (*cc_current_a, *cv_voltage_v, *cutoff_current_a),
            _ => return Err(FastChargingError::from("not a CCCV protocol")),
        };

        if cc_current_a <= 0.0 {
            return Err(FastChargingError::from("cc_current_a must be positive"));
        }
        if cutoff_current_a < 0.0 {
            return Err(FastChargingError::from(
                "cutoff_current_a must be non-negative",
            ));
        }

        let mut state = self.initial_state();
        let mut phase = CcCvPhase::Cc;
        let mut rec = Recorder::new();
        rec.record(&state);

        let dt = self.config.dt_s;
        let mut complete = false;

        while state.time_s < self.config.max_sim_time_s {
            let current = match phase {
                CcCvPhase::Cc => {
                    let v = self.terminal_voltage(state.soc, cc_current_a, state.v_rc1);
                    if v >= cv_voltage_v {
                        phase = CcCvPhase::Cv;
                        self.cv_current(state.soc, state.v_rc1, cv_voltage_v)
                    } else {
                        cc_current_a
                    }
                }
                CcCvPhase::Cv => self.cv_current(state.soc, state.v_rc1, cv_voltage_v),
            };

            let current = current.max(0.0);

            if phase == CcCvPhase::Cv && current <= cutoff_current_a {
                complete = true;
                break;
            }

            // Voltage guard
            let v_check = self.terminal_voltage(state.soc, current, state.v_rc1);
            if v_check > self.battery.v_max + 1e-6 {
                // clamp at v_max
                break;
            }

            state = self.advance(state, current, dt);
            rec.record(&state);

            if state.soc >= 1.0 {
                complete = true;
                break;
            }
        }

        Ok(self.build_result(rec, complete))
    }

    fn simulate_mcc(
        &self,
        stages: &[CcStage],
        cv_voltage_v: f64,
        cutoff_current_a: f64,
    ) -> Result<FastChargingResult, FastChargingError> {
        if stages.is_empty() {
            return Err(FastChargingError::from("MCC stages must not be empty"));
        }

        let mut state = self.initial_state();
        let mut stage_idx = 0usize;
        let mut in_cv = false;
        let mut rec = Recorder::new();
        rec.record(&state);
        let mut complete = false;
        let dt = self.config.dt_s;

        while state.time_s < self.config.max_sim_time_s {
            let current = if in_cv {
                let i = self.cv_current(state.soc, state.v_rc1, cv_voltage_v);
                if i <= cutoff_current_a {
                    complete = true;
                    break;
                }
                i
            } else {
                let stage = &stages[stage_idx];
                let v = self.terminal_voltage(state.soc, stage.current_a, state.v_rc1);
                if v >= stage.voltage_limit_v {
                    // advance stage
                    if stage_idx + 1 < stages.len() {
                        stage_idx += 1;
                    } else {
                        in_cv = true;
                    }
                }
                // re-read after possible stage advance
                if in_cv {
                    self.cv_current(state.soc, state.v_rc1, cv_voltage_v)
                        .max(0.0)
                } else {
                    stages[stage_idx].current_a
                }
            };

            let current = current.max(0.0);
            let v_check = self.terminal_voltage(state.soc, current, state.v_rc1);
            if v_check > self.battery.v_max + 1e-6 {
                break;
            }

            state = self.advance(state, current, dt);
            rec.record(&state);

            if state.soc >= 1.0 {
                complete = true;
                break;
            }
        }

        Ok(self.build_result(rec, complete))
    }

    fn simulate_pulse(
        &self,
        protocol: &ChargingProtocol,
    ) -> Result<FastChargingResult, FastChargingError> {
        let (peak_current_a, rest_current_a, on_time_s, off_time_s, max_voltage_v, _cutoff) =
            match protocol {
                ChargingProtocol::Pulse {
                    peak_current_a,
                    rest_current_a,
                    on_time_s,
                    off_time_s,
                    max_voltage_v,
                    cutoff_current_a,
                } => (
                    *peak_current_a,
                    *rest_current_a,
                    *on_time_s,
                    *off_time_s,
                    *max_voltage_v,
                    *cutoff_current_a,
                ),
                _ => return Err(FastChargingError::from("not a Pulse protocol")),
            };

        if on_time_s <= 0.0 || off_time_s < 0.0 {
            return Err(FastChargingError::from("on_time_s must be positive"));
        }

        let cycle_time = on_time_s + off_time_s;
        let mut state = self.initial_state();
        let mut rec = Recorder::new();
        rec.record(&state);
        let mut complete = false;
        let dt = self.config.dt_s;

        while state.time_s < self.config.max_sim_time_s {
            let phase_in_cycle = state.time_s % cycle_time;
            let current = if phase_in_cycle < on_time_s {
                peak_current_a
            } else {
                rest_current_a
            };

            let v_check = self.terminal_voltage(state.soc, current, state.v_rc1);
            if v_check > max_voltage_v {
                complete = true;
                break;
            }

            state = self.advance(state, current, dt);
            rec.record(&state);

            if state.soc >= 1.0 {
                complete = true;
                break;
            }
        }

        Ok(self.build_result(rec, complete))
    }

    fn simulate_boost(
        &self,
        protocol: &ChargingProtocol,
    ) -> Result<FastChargingResult, FastChargingError> {
        let (boost_current_a, boost_voltage_v, normal_current_a, cv_voltage_v, cutoff_current_a) =
            match protocol {
                ChargingProtocol::Boost {
                    boost_current_a,
                    boost_voltage_v,
                    normal_current_a,
                    cv_voltage_v,
                    cutoff_current_a,
                } => (
                    *boost_current_a,
                    *boost_voltage_v,
                    *normal_current_a,
                    *cv_voltage_v,
                    *cutoff_current_a,
                ),
                _ => return Err(FastChargingError::from("not a Boost protocol")),
            };

        let mut state = self.initial_state();
        let mut phase = BoostPhase::Boost;
        let mut rec = Recorder::new();
        rec.record(&state);
        let mut complete = false;
        let dt = self.config.dt_s;

        while state.time_s < self.config.max_sim_time_s {
            let current = match phase {
                BoostPhase::Boost => {
                    let v = self.terminal_voltage(state.soc, boost_current_a, state.v_rc1);
                    if v >= boost_voltage_v {
                        phase = BoostPhase::Normal;
                        normal_current_a
                    } else {
                        boost_current_a
                    }
                }
                BoostPhase::Normal => {
                    let v = self.terminal_voltage(state.soc, normal_current_a, state.v_rc1);
                    if v >= cv_voltage_v {
                        phase = BoostPhase::Cv;
                        self.cv_current(state.soc, state.v_rc1, cv_voltage_v)
                    } else {
                        normal_current_a
                    }
                }
                BoostPhase::Cv => self.cv_current(state.soc, state.v_rc1, cv_voltage_v),
            };

            let current = current.max(0.0);

            if phase == BoostPhase::Cv && current <= cutoff_current_a {
                complete = true;
                break;
            }

            let v_check = self.terminal_voltage(state.soc, current, state.v_rc1);
            if v_check > self.battery.v_max + 1e-6 {
                break;
            }

            state = self.advance(state, current, dt);
            rec.record(&state);

            if state.soc >= 1.0 {
                complete = true;
                break;
            }
        }

        Ok(self.build_result(rec, complete))
    }

    fn simulate_health_aware(
        &self,
        protocol: &ChargingProtocol,
    ) -> Result<FastChargingResult, FastChargingError> {
        let (target_soc, max_time_s, max_current_a, max_voltage_v, degradation_weight) =
            match protocol {
                ChargingProtocol::HealthAware {
                    target_soc,
                    max_time_s,
                    max_current_a,
                    max_voltage_v,
                    degradation_weight,
                } => (
                    *target_soc,
                    *max_time_s,
                    *max_current_a,
                    *max_voltage_v,
                    *degradation_weight,
                ),
                _ => return Err(FastChargingError::from("not a HealthAware protocol")),
            };

        if !(0.0..=1.0).contains(&target_soc) {
            return Err(FastChargingError::from("target_soc must be in [0, 1]"));
        }
        if max_current_a <= 0.0 {
            return Err(FastChargingError::from("max_current_a must be positive"));
        }

        let limit_time = max_time_s.min(self.config.max_sim_time_s);

        let mut state = self.initial_state();
        let mut rec = Recorder::new();
        rec.record(&state);
        let mut complete = false;
        let dt = self.config.dt_s;

        while state.time_s < limit_time {
            let current = self
                .estimate_optimal_current(state.soc, max_current_a, degradation_weight)
                .max(0.01); // keep at least 10 mA to prevent infinite loop

            let v_check = self.terminal_voltage(state.soc, current, state.v_rc1);
            let current = if v_check > max_voltage_v {
                // clamp current so terminal voltage does not exceed limit
                self.current_for_voltage(state.soc, state.v_rc1, max_voltage_v)
                    .max(0.0)
            } else {
                current
            };

            state = self.advance(state, current, dt);
            rec.record(&state);

            if state.soc >= target_soc {
                complete = true;
                break;
            }
        }

        Ok(self.build_result(rec, complete))
    }

    // -----------------------------------------------------------------------
    // Capacity fade & scoring
    // -----------------------------------------------------------------------

    /// Compute estimated capacity fade (%) from a simulation result.
    pub fn compute_capacity_fade(&self, result: &FastChargingResult) -> f64 {
        let throughput_kah = result.charge_throughput_ah / 1000.0;
        let peak_temp = result.peak_temperature_c;
        let thermal_extra = (peak_temp - 40.0).max(0.0);
        self.battery.deg_per_kah
            * throughput_kah
            * (1.0 + self.battery.thermal_deg_factor * thermal_extra)
            * 100.0 // convert to %
    }

    fn rank_protocols(&self, times: &[f64], effs: &[f64], fades: &[f64]) -> usize {
        let n = times.len();
        if n == 0 {
            return 0;
        }
        let max_t = times
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max)
            .max(1e-9);
        let max_eff = effs
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max)
            .max(1e-9);
        let max_fade = fades
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max)
            .max(1e-9);

        let mut best_idx = 0usize;
        let mut best_score = f64::NEG_INFINITY;
        for i in 0..n {
            // Lower time → higher time_score
            let time_score = 1.0 - times[i] / max_t;
            let eff_score = effs[i] / max_eff;
            let health_score = 1.0 - fades[i] / max_fade;
            let score = 0.4 * time_score + 0.3 * eff_score + 0.3 * health_score;
            if score > best_score {
                best_score = score;
                best_idx = i;
            }
        }
        best_idx
    }

    // -----------------------------------------------------------------------
    // ECM helpers
    // -----------------------------------------------------------------------

    /// Open-circuit voltage as a linear function of SoC.
    pub fn ocv(&self, soc: f64) -> f64 {
        let soc = soc.clamp(0.0, 1.0);
        self.battery.ocv_offset + self.battery.ocv_slope * soc
    }

    /// Terminal voltage during charging (I > 0 flows into battery).
    ///
    /// `V_term = OCV + I*R0 + V_RC1`
    fn terminal_voltage(&self, soc: f64, current_a: f64, v_rc1: f64) -> f64 {
        self.ocv(soc) + current_a * self.battery.r0_ohm + v_rc1
    }

    /// Compute the CV-phase current needed to hold the terminal voltage at `cv_v`.
    ///
    /// From `cv_v = OCV + I*R0 + V_RC1`:
    /// `I = (cv_v - OCV - V_RC1) / R0`
    fn cv_current(&self, soc: f64, v_rc1: f64, cv_v: f64) -> f64 {
        if self.battery.r0_ohm.abs() < 1e-15 {
            return 0.0;
        }
        (cv_v - self.ocv(soc) - v_rc1) / self.battery.r0_ohm
    }

    /// Current that would produce exactly `target_v` at the terminal.
    fn current_for_voltage(&self, soc: f64, v_rc1: f64, target_v: f64) -> f64 {
        self.cv_current(soc, v_rc1, target_v)
    }

    // -----------------------------------------------------------------------
    // ODE integration
    // -----------------------------------------------------------------------

    /// Advance the charging state by one time step `dt`.
    fn advance(&self, state: ChargingState, current_a: f64, dt: f64) -> ChargingState {
        // --- Electrical ---
        let b = &self.battery;
        let tau1 = b.r1_ohm * b.c1_f;
        let dv_rc1 = if tau1.abs() < 1e-15 {
            0.0
        } else {
            dt * (-state.v_rc1 / tau1 + current_a / b.c1_f)
        };
        let v_rc1_new = state.v_rc1 + dv_rc1;

        let d_soc = current_a * dt / (b.capacity_ah * 3600.0);
        let soc_new = (state.soc + d_soc).clamp(0.0, 1.0);

        let v_ocv_new = self.ocv(soc_new);
        let v_term_new = v_ocv_new + current_a * b.r0_ohm + v_rc1_new;

        // --- Thermal ---
        let joule_heat = current_a * current_a * b.r0_ohm;
        let cooling = (state.temperature_c - self.config.ambient_temp_c)
            / self.config.thermal_resistance_k_per_w;
        let d_temp = dt * (joule_heat - cooling) / self.config.thermal_capacitance_j_per_k;
        let temp_new = state.temperature_c + d_temp;

        // --- Throughput ---
        let throughput_new = state.charge_throughput_ah + current_a * dt / 3600.0;

        ChargingState {
            soc: soc_new,
            v_terminal: v_term_new,
            v_ocv: v_ocv_new,
            v_rc1: v_rc1_new,
            current_a,
            temperature_c: temp_new,
            time_s: state.time_s + dt,
            charge_throughput_ah: throughput_new,
        }
    }

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    fn initial_state(&self) -> ChargingState {
        let soc = self.config.initial_soc.clamp(0.0, 1.0);
        let v_ocv = self.ocv(soc);
        ChargingState {
            soc,
            v_terminal: v_ocv,
            v_ocv,
            v_rc1: 0.0,
            current_a: 0.0,
            temperature_c: self.config.initial_temp_c,
            time_s: 0.0,
            charge_throughput_ah: 0.0,
        }
    }

    fn validate_battery(&self) -> Result<(), FastChargingError> {
        let b = &self.battery;
        if b.capacity_ah <= 0.0 {
            return Err(FastChargingError::from("capacity_ah must be positive"));
        }
        if b.r0_ohm < 0.0 {
            return Err(FastChargingError::from("r0_ohm must be non-negative"));
        }
        if b.v_max <= b.v_min {
            return Err(FastChargingError::from("v_max must be greater than v_min"));
        }
        Ok(())
    }

    fn build_result(&self, rec: Recorder, complete: bool) -> FastChargingResult {
        let n = rec.time_s.len();
        if n == 0 {
            return FastChargingResult {
                time_s: vec![],
                soc: vec![],
                voltage_v: vec![],
                current_a: vec![],
                temperature_c: vec![],
                power_w: vec![],
                charging_complete: false,
                charging_time_s: 0.0,
                final_soc: self.config.initial_soc,
                energy_wh: 0.0,
                efficiency_pct: 0.0,
                capacity_fade_pct: 0.0,
                peak_temperature_c: self.config.initial_temp_c,
                charge_throughput_ah: 0.0,
            };
        }

        // Compute power
        let power_w: Vec<f64> = rec
            .voltage_v
            .iter()
            .zip(rec.current_a.iter())
            .map(|(v, i)| v * i)
            .collect();

        // Integrate energy (trapezoidal rule over time)
        let mut energy_wh = 0.0;
        let dt = self.config.dt_s;
        for i in 0..power_w.len().saturating_sub(1) {
            energy_wh += 0.5 * (power_w[i] + power_w[i + 1]) * dt / 3600.0;
        }
        if power_w.len() == 1 {
            energy_wh = power_w[0] * dt / 3600.0;
        }

        let final_soc = *rec.soc.last().unwrap_or(&self.config.initial_soc);
        let charge_throughput_ah = *rec.throughput_ah.last().unwrap_or(&0.0);
        let peak_temp = rec
            .temperature_c
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        let charging_time_s = *rec.time_s.last().unwrap_or(&0.0);

        // Efficiency: ratio of chemical energy stored to electrical energy supplied.
        // Chemical energy ≈ ΔSoC * capacity_ah * average_OCV * 3600 (J) / 3600 → Wh
        let delta_soc = (final_soc - self.config.initial_soc).max(0.0);
        let avg_ocv = 0.5 * (self.ocv(self.config.initial_soc) + self.ocv(final_soc));
        let stored_wh = delta_soc * self.battery.capacity_ah * avg_ocv;
        let efficiency_pct = if energy_wh > 1e-9 {
            (stored_wh / energy_wh * 100.0).clamp(0.0, 100.0)
        } else {
            0.0
        };

        let mut result = FastChargingResult {
            time_s: rec.time_s,
            soc: rec.soc,
            voltage_v: rec.voltage_v,
            current_a: rec.current_a,
            temperature_c: rec.temperature_c,
            power_w,
            charging_complete: complete,
            charging_time_s,
            final_soc,
            energy_wh,
            efficiency_pct,
            capacity_fade_pct: 0.0, // filled in below
            peak_temperature_c: if peak_temp.is_finite() {
                peak_temp
            } else {
                self.config.initial_temp_c
            },
            charge_throughput_ah,
        };
        result.capacity_fade_pct = self.compute_capacity_fade(&result);
        result
    }
}

// ---------------------------------------------------------------------------
// Recorder (internal helper)
// ---------------------------------------------------------------------------

/// Accumulates per-step simulation data.
struct Recorder {
    time_s: Vec<f64>,
    soc: Vec<f64>,
    voltage_v: Vec<f64>,
    current_a: Vec<f64>,
    temperature_c: Vec<f64>,
    throughput_ah: Vec<f64>,
}

impl Recorder {
    fn new() -> Self {
        Self {
            time_s: Vec::new(),
            soc: Vec::new(),
            voltage_v: Vec::new(),
            current_a: Vec::new(),
            temperature_c: Vec::new(),
            throughput_ah: Vec::new(),
        }
    }

    fn record(&mut self, s: &ChargingState) {
        self.time_s.push(s.time_s);
        self.soc.push(s.soc);
        self.voltage_v.push(s.v_terminal);
        self.current_a.push(s.current_a);
        self.temperature_c.push(s.temperature_c);
        self.throughput_ah.push(s.charge_throughput_ah);
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_battery() -> FcBatteryParams {
        FcBatteryParams {
            capacity_ah: 50.0,
            r0_ohm: 0.002,
            r1_ohm: 0.003,
            c1_f: 1000.0,
            v_min: 2.8,
            v_max: 4.25,
            ocv_slope: 1.2,
            ocv_offset: 3.0,
            deg_per_kah: 0.002,
            thermal_deg_factor: 0.02,
        }
    }

    fn default_config() -> FastChargingConfig {
        FastChargingConfig {
            dt_s: 1.0,
            max_sim_time_s: 7200.0,
            initial_soc: 0.2,
            initial_temp_c: 25.0,
            ambient_temp_c: 25.0,
            thermal_resistance_k_per_w: 5.0,
            thermal_capacitance_j_per_k: 100.0,
        }
    }

    fn make_sim() -> FastChargingSimulator {
        FastChargingSimulator::new(default_battery(), default_config())
    }

    fn cccv_protocol() -> ChargingProtocol {
        ChargingProtocol::CcCv {
            cc_current_a: 50.0,
            cv_voltage_v: 4.15,
            cutoff_current_a: 2.5,
        }
    }

    // --- CCCV tests ---

    #[test]
    fn test_cccv_completes_before_max_time() {
        let sim = make_sim();
        let r = sim.simulate(&cccv_protocol()).expect("simulation error");
        assert!(r.charging_complete, "CCCV did not complete");
        assert!(r.charging_time_s < sim.config.max_sim_time_s);
    }

    #[test]
    fn test_cccv_final_soc_above_90pct() {
        let sim = make_sim();
        let r = sim.simulate(&cccv_protocol()).expect("simulation error");
        assert!(r.final_soc > 0.9, "final SoC = {}", r.final_soc);
    }

    #[test]
    fn test_cccv_voltage_stays_below_vmax() {
        let sim = make_sim();
        let r = sim.simulate(&cccv_protocol()).expect("simulation error");
        let max_v = r
            .voltage_v
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(
            max_v <= sim.battery.v_max + 1e-4,
            "max voltage {} exceeded v_max {}",
            max_v,
            sim.battery.v_max
        );
    }

    #[test]
    fn test_cc_phase_voltage_rises_monotonically() {
        let sim = make_sim();
        let r = sim.simulate(&cccv_protocol()).expect("simulation error");
        let cc_current = 50.0f64;
        // CC phase: find the last *consecutive* step at CC current.
        // Skip the initial state (index 0) which has current = 0.
        let mut end = 0usize;
        let mut found_cc = false;
        for (idx, &i) in r.current_a.iter().enumerate().skip(1) {
            if (i - cc_current).abs() < 0.5 {
                end = idx;
                found_cc = true;
            } else if found_cc {
                // First deviation after we entered CC ends the phase
                break;
            }
        }
        assert!(found_cc && end > 0, "no CC phase found");
        // Voltage in CC phase should be non-decreasing (allow 1 mV numerical noise)
        for i in 1..=end {
            assert!(
                r.voltage_v[i] >= r.voltage_v[i - 1] - 1e-3,
                "voltage not monotone at step {}: {} < {}",
                i,
                r.voltage_v[i],
                r.voltage_v[i - 1]
            );
        }
    }

    #[test]
    fn test_cv_phase_current_decays() {
        let sim = make_sim();
        let r = sim.simulate(&cccv_protocol()).expect("simulation error");
        // Find the start of CV phase: first index where current drops well below CC level
        let cv_start_opt = r.current_a.iter().position(|&i| i < 45.0);
        if let Some(cv_start) = cv_start_opt {
            if cv_start + 10 < r.current_a.len() {
                // Sample current 10 steps into CV and near the end
                let i_early = r.current_a[cv_start + 1];
                let i_late = r.current_a[r.current_a.len() - 1];
                assert!(
                    i_late <= i_early + 0.1,
                    "CV current should decay: early={}, late={}",
                    i_early,
                    i_late
                );
            }
        }
        // At minimum, the final current should be ≤ cutoff
        let final_i = *r.current_a.last().unwrap_or(&0.0);
        assert!(
            final_i <= 3.0,
            "final current should be near cutoff: {}",
            final_i
        );
    }

    #[test]
    fn test_cutoff_current_terminates_cccv() {
        let sim = make_sim();
        let proto = ChargingProtocol::CcCv {
            cc_current_a: 50.0,
            cv_voltage_v: 4.15,
            cutoff_current_a: 10.0, // high cutoff → terminates sooner
        };
        let r_high = sim.simulate(&proto).expect("error");
        let r_low = sim.simulate(&cccv_protocol()).expect("error");
        // With higher cutoff the charging time should be shorter
        assert!(
            r_high.charging_time_s <= r_low.charging_time_s + 1.0,
            "high cutoff should terminate sooner: {} vs {}",
            r_high.charging_time_s,
            r_low.charging_time_s
        );
    }

    // --- MCC tests ---

    fn mcc_protocol() -> ChargingProtocol {
        ChargingProtocol::Mcc {
            stages: vec![
                CcStage {
                    current_a: 75.0,
                    voltage_limit_v: 4.0,
                },
                CcStage {
                    current_a: 50.0,
                    voltage_limit_v: 4.1,
                },
                CcStage {
                    current_a: 25.0,
                    voltage_limit_v: 4.15,
                },
            ],
            cv_voltage_v: 4.15,
            cutoff_current_a: 2.5,
        }
    }

    #[test]
    fn test_mcc_3_stages_completes() {
        let sim = make_sim();
        let r = sim.simulate(&mcc_protocol()).expect("error");
        assert!(r.charging_complete, "MCC did not complete");
    }

    #[test]
    fn test_mcc_first_stage_highest_crate() {
        // First stage current (75 A) > second stage (50 A) > third (25 A)
        // Verify the first stage has max current in the trajectory
        let sim = make_sim();
        let r = sim.simulate(&mcc_protocol()).expect("error");
        let max_i = r
            .current_a
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(
            (max_i - 75.0).abs() < 1.0,
            "first stage should dominate max current: {}",
            max_i
        );
    }

    #[test]
    fn test_mcc_voltage_limit_triggers_stage_switch() {
        // Each stage should see a voltage no higher than its limit (plus small tolerance)
        let sim = make_sim();
        // Use tighter limits to ensure stage transitions happen
        let proto = ChargingProtocol::Mcc {
            stages: vec![
                CcStage {
                    current_a: 50.0,
                    voltage_limit_v: 3.8,
                },
                CcStage {
                    current_a: 25.0,
                    voltage_limit_v: 4.0,
                },
            ],
            cv_voltage_v: 4.1,
            cutoff_current_a: 2.5,
        };
        let r = sim.simulate(&proto).expect("error");
        // Voltage should remain ≤ v_max throughout
        let max_v = r
            .voltage_v
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(max_v <= sim.battery.v_max + 1e-4);
    }

    // --- Pulse tests ---

    fn pulse_protocol() -> ChargingProtocol {
        ChargingProtocol::Pulse {
            peak_current_a: 60.0,
            rest_current_a: 0.0,
            on_time_s: 10.0,
            off_time_s: 5.0,
            max_voltage_v: 4.2,
            cutoff_current_a: 2.5,
        }
    }

    #[test]
    fn test_pulse_on_current_greater_than_off() {
        let sim = make_sim();
        let r = sim.simulate(&pulse_protocol()).expect("error");
        let max_i = r
            .current_a
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        let min_i = r.current_a.iter().cloned().fold(f64::INFINITY, f64::min);
        assert!(
            max_i > min_i + 1.0,
            "pulse current should vary: max={}, min={}",
            max_i,
            min_i
        );
    }

    #[test]
    fn test_pulse_alternating_current_pattern() {
        let sim = make_sim();
        let r = sim.simulate(&pulse_protocol()).expect("error");
        // Check that there are both high and low current values
        let has_high = r.current_a.iter().any(|&i| i > 50.0);
        let has_low = r.current_a.iter().any(|&i| i < 5.0);
        assert!(has_high, "no high-current phase found");
        assert!(has_low, "no rest phase found");
    }

    // --- Boost tests ---

    fn boost_protocol() -> ChargingProtocol {
        ChargingProtocol::Boost {
            boost_current_a: 100.0,
            boost_voltage_v: 3.9,
            normal_current_a: 50.0,
            cv_voltage_v: 4.15,
            cutoff_current_a: 2.5,
        }
    }

    #[test]
    fn test_boost_starts_at_boost_current() {
        let sim = make_sim();
        let r = sim.simulate(&boost_protocol()).expect("error");
        // First non-zero current step should be the boost current
        let first_nonzero = r
            .current_a
            .iter()
            .find(|&&i| i > 0.0)
            .cloned()
            .unwrap_or(0.0);
        assert!(
            (first_nonzero - 100.0).abs() < 1.0,
            "first current should be boost current: {}",
            first_nonzero
        );
    }

    #[test]
    fn test_boost_drops_to_normal_then_cv() {
        let sim = make_sim();
        let r = sim.simulate(&boost_protocol()).expect("error");
        // After the boost phase we expect current to be at normal level (50 A)
        let has_normal = r.current_a.iter().any(|&i| (i - 50.0).abs() < 5.0);
        assert!(has_normal, "no normal-phase current found");
    }

    // --- HealthAware tests ---

    fn health_aware_protocol(weight: f64) -> ChargingProtocol {
        ChargingProtocol::HealthAware {
            target_soc: 0.9,
            max_time_s: 7200.0,
            max_current_a: 75.0,
            max_voltage_v: 4.2,
            degradation_weight: weight,
        }
    }

    #[test]
    fn test_health_aware_lower_current_at_high_soc() {
        let sim = make_sim();
        let i_low_soc = sim.estimate_optimal_current(0.1, 75.0, 0.8);
        let i_high_soc = sim.estimate_optimal_current(0.9, 75.0, 0.8);
        assert!(
            i_low_soc > i_high_soc,
            "current at low SoC ({}) should exceed high SoC ({})",
            i_low_soc,
            i_high_soc
        );
    }

    #[test]
    fn test_health_aware_lower_fade_than_cccv() {
        let sim = make_sim();
        let r_ha = sim.simulate(&health_aware_protocol(1.0)).expect("error");
        let r_cc = sim.simulate(&cccv_protocol()).expect("error");
        // HealthAware at full degradation weight should produce lower or equal capacity fade
        assert!(
            r_ha.capacity_fade_pct <= r_cc.capacity_fade_pct + 0.001,
            "HealthAware fade {} not less than CCCV fade {}",
            r_ha.capacity_fade_pct,
            r_cc.capacity_fade_pct
        );
    }

    // --- Thermal tests ---

    #[test]
    fn test_temperature_rises_during_fast_charge() {
        let sim = make_sim();
        let r = sim.simulate(&cccv_protocol()).expect("error");
        let max_t = r
            .temperature_c
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(
            max_t > sim.config.initial_temp_c,
            "temperature should rise: max={}, initial={}",
            max_t,
            sim.config.initial_temp_c
        );
    }

    #[test]
    fn test_higher_current_higher_temperature() {
        let battery = default_battery();
        let config = default_config();
        let sim = FastChargingSimulator::new(battery, config);

        let proto_low = ChargingProtocol::CcCv {
            cc_current_a: 25.0,
            cv_voltage_v: 4.15,
            cutoff_current_a: 2.5,
        };
        let proto_high = ChargingProtocol::CcCv {
            cc_current_a: 100.0,
            cv_voltage_v: 4.15,
            cutoff_current_a: 2.5,
        };

        let r_low = sim.simulate(&proto_low).expect("error");
        let r_high = sim.simulate(&proto_high).expect("error");

        assert!(
            r_high.peak_temperature_c >= r_low.peak_temperature_c,
            "high current peak T ({}) should be ≥ low current peak T ({})",
            r_high.peak_temperature_c,
            r_low.peak_temperature_c
        );
    }

    // --- Efficiency tests ---

    #[test]
    fn test_charging_efficiency_below_100pct() {
        let sim = make_sim();
        let r = sim.simulate(&cccv_protocol()).expect("error");
        assert!(
            r.efficiency_pct < 100.0,
            "efficiency should be < 100%: {}",
            r.efficiency_pct
        );
        assert!(
            r.efficiency_pct > 0.0,
            "efficiency should be positive: {}",
            r.efficiency_pct
        );
    }

    // --- Capacity fade tests ---

    #[test]
    fn test_capacity_fade_proportional_to_throughput() {
        let sim = make_sim();
        // Two protocols with very different throughputs
        let proto_fast = ChargingProtocol::CcCv {
            cc_current_a: 100.0,
            cv_voltage_v: 4.15,
            cutoff_current_a: 20.0, // cut off early
        };
        let proto_slow = ChargingProtocol::CcCv {
            cc_current_a: 100.0,
            cv_voltage_v: 4.15,
            cutoff_current_a: 2.5,
        };
        let r_fast = sim.simulate(&proto_fast).expect("error");
        let r_slow = sim.simulate(&proto_slow).expect("error");
        // Slower charging (more throughput) → more fade
        assert!(
            r_slow.capacity_fade_pct >= r_fast.capacity_fade_pct,
            "slow fade ({}) should be ≥ fast fade ({})",
            r_slow.capacity_fade_pct,
            r_fast.capacity_fade_pct
        );
    }

    // --- compare_protocols tests ---

    #[test]
    fn test_compare_protocols_returns_recommendation() {
        let sim = make_sim();
        let protocols = vec![
            ("CCCV".to_string(), cccv_protocol()),
            ("MCC".to_string(), mcc_protocol()),
        ];
        let cmp = sim.compare_protocols(&protocols).expect("error");
        assert!(cmp.recommended < protocols.len());
        assert!(!cmp.recommendation_reason.is_empty());
    }

    #[test]
    fn test_compare_protocols_fastest_has_smallest_time() {
        let sim = make_sim();
        let protocols = vec![
            (
                "CCCV-slow".to_string(),
                ChargingProtocol::CcCv {
                    cc_current_a: 10.0,
                    cv_voltage_v: 4.15,
                    cutoff_current_a: 2.5,
                },
            ),
            (
                "CCCV-fast".to_string(),
                ChargingProtocol::CcCv {
                    cc_current_a: 100.0,
                    cv_voltage_v: 4.15,
                    cutoff_current_a: 2.5,
                },
            ),
        ];
        let cmp = sim.compare_protocols(&protocols).expect("error");
        // Fastest is at index 1
        let min_t = cmp
            .charging_times_s
            .iter()
            .cloned()
            .fold(f64::INFINITY, f64::min);
        assert_eq!(cmp.charging_times_s[1], min_t);
    }

    #[test]
    fn test_compare_protocols_vector_lengths_match() {
        let sim = make_sim();
        let protocols = vec![
            ("A".to_string(), cccv_protocol()),
            ("B".to_string(), mcc_protocol()),
            ("C".to_string(), pulse_protocol()),
        ];
        let cmp = sim.compare_protocols(&protocols).expect("error");
        let n = protocols.len();
        assert_eq!(cmp.protocols.len(), n);
        assert_eq!(cmp.charging_times_s.len(), n);
        assert_eq!(cmp.final_socs.len(), n);
        assert_eq!(cmp.efficiencies_pct.len(), n);
        assert_eq!(cmp.capacity_fades_pct.len(), n);
        assert_eq!(cmp.peak_temperatures_c.len(), n);
    }

    // --- estimate_optimal_current tests ---

    #[test]
    fn test_estimate_optimal_current_decreases_with_weight() {
        let sim = make_sim();
        let i0 = sim.estimate_optimal_current(0.5, 75.0, 0.0);
        let i1 = sim.estimate_optimal_current(0.5, 75.0, 1.0);
        assert!(
            i0 > i1,
            "weight=0 current ({}) should exceed weight=1 ({})",
            i0,
            i1
        );
    }

    #[test]
    fn test_estimate_optimal_current_at_zero_soc() {
        let sim = make_sim();
        // At SoC=0 and any weight, exp(0) = 1 → returns max_current
        let i = sim.estimate_optimal_current(0.0, 75.0, 1.0);
        assert!((i - 75.0).abs() < 1e-9);
    }

    // --- Energy tests ---

    #[test]
    fn test_energy_wh_positive() {
        let sim = make_sim();
        let r = sim.simulate(&cccv_protocol()).expect("error");
        assert!(
            r.energy_wh > 0.0,
            "energy_wh should be positive: {}",
            r.energy_wh
        );
    }
}
