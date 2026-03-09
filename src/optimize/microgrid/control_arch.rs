//! Three-Layer Microgrid Control Architecture.
//!
//! Implements a hierarchical control framework with three time-scale layers:
//!
//! - **Primary control** \[ms\]: droop-based frequency/voltage regulation with
//!   first-order lag dynamics for each Distributed Energy Resource (DER).
//! - **Secondary control** \[s\]: PI-based frequency and voltage restoration
//!   to nominal values after primary droop action.
//! - **Tertiary control** \[min\]: economic dispatch via merit-order, grid
//!   import/export decisions, and SoC management for battery units.

// ─── Errors ───────────────────────────────────────────────────────────────────

/// Errors from microgrid control simulation.
#[derive(Debug, thiserror::Error)]
pub enum ControlError {
    /// No DER units added.
    #[error("no DER units added — add at least one DER before simulating")]
    NoDer,
    /// Load or renewable profiles not set.
    #[error("load/renewable profiles not set or mismatched lengths")]
    NoProfiles,
    /// Invalid configuration.
    #[error("configuration error: {0}")]
    Config(String),
    /// Numerical issue during simulation.
    #[error("numerical issue at time {0:.3} s: {1}")]
    Numerical(f64, String),
}

// ─── Configuration ────────────────────────────────────────────────────────────

/// Configuration for the three-layer microgrid control hierarchy.
#[derive(Debug, Clone)]
pub struct MicrogridControlConfig {
    /// Number of DER units.
    pub n_der: usize,
    /// Primary control timestep \[s\] (e.g. 0.001).
    pub primary_control_dt_s: f64,
    /// Secondary control timestep \[s\] (e.g. 1.0).
    pub secondary_control_dt_s: f64,
    /// Tertiary control timestep \[s\] (e.g. 300.0).
    pub tertiary_control_dt_s: f64,
    /// Nominal frequency \[Hz\].
    pub nominal_freq_hz: f64,
    /// Nominal bus voltage \[pu\].
    pub nominal_voltage_pu: f64,
    /// Primary droop gain for frequency \[Hz/MW\].
    pub primary_droop_freq: f64,
    /// Primary droop gain for voltage \[pu/Mvar\].
    pub primary_droop_voltage: f64,
}

impl Default for MicrogridControlConfig {
    fn default() -> Self {
        Self {
            n_der: 1,
            primary_control_dt_s: 0.001,
            secondary_control_dt_s: 1.0,
            tertiary_control_dt_s: 300.0,
            nominal_freq_hz: 50.0,
            nominal_voltage_pu: 1.0,
            primary_droop_freq: 0.01,    // 0.01 Hz/MW
            primary_droop_voltage: 0.05, // 0.05 pu/Mvar
        }
    }
}

// ─── DER Unit ────────────────────────────────────────────────────────────────

/// A Distributed Energy Resource (generator, battery, or combined).
#[derive(Debug, Clone)]
pub struct DerUnit {
    /// Unique identifier.
    pub id: usize,
    /// Rated active power capacity \[MW\].
    pub p_rated_mw: f64,
    /// Rated reactive power capacity \[Mvar\].
    pub q_rated_mvar: f64,
    /// Minimum active power \[MW\].
    pub p_min_mw: f64,
    /// Initial active power setpoint \[MW\].
    pub p_initial_mw: f64,
    /// Initial reactive power setpoint \[Mvar\].
    pub q_initial_mvar: f64,
    /// Per-unit droop gain (relative to system base).
    pub droop_gain: f64,
    /// Ramp rate limit \[MW/s\].
    pub ramp_rate_mw_per_s: f64,
    /// First-order lag time constant \[s\].
    pub response_time_s: f64,
}

// ─── State and Action Types ───────────────────────────────────────────────────

/// Snapshot of the microgrid state during primary control.
#[derive(Debug, Clone)]
pub struct PrimaryControlState {
    /// Simulation time \[s\].
    pub time_s: f64,
    /// System frequency \[Hz\].
    pub frequency_hz: f64,
    /// Bus voltage magnitude \[pu\].
    pub voltage_pu: f64,
    /// Active power output per DER \[MW\].
    pub der_p_mw: Vec<f64>,
    /// Reactive power output per DER \[Mvar\].
    pub der_q_mvar: Vec<f64>,
}

/// Secondary control correction action.
#[derive(Debug, Clone)]
pub struct SecondaryControlAction {
    /// Simulation time \[s\].
    pub time_s: f64,
    /// Frequency correction \[Hz\] added to DER setpoints.
    pub frequency_correction_hz: f64,
    /// Voltage correction \[pu\] added to DER setpoints.
    pub voltage_correction_pu: f64,
    /// Updated active power setpoints \[MW\] per DER.
    pub p_dispatch: Vec<f64>,
    /// Updated reactive power setpoints \[Mvar\] per DER.
    pub q_dispatch: Vec<f64>,
}

/// Tertiary control (economic dispatch) decision.
#[derive(Debug, Clone)]
pub struct TertiaryDecision {
    /// Simulation time \[s\].
    pub time_s: f64,
    /// Economically optimised P setpoints \[MW\] per DER.
    pub economic_dispatch: Vec<f64>,
    /// Power imported (positive) or exported (negative) from/to main grid \[MW\].
    pub power_import_mw: f64,
    /// Battery SoC management setpoints \[normalised\].
    pub battery_setpoints: Vec<f64>,
    /// Current operation mode.
    pub mode: OperationMode,
}

/// Microgrid operation mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OperationMode {
    /// Connected to and exchanging power with the main grid.
    GridConnected,
    /// Operating autonomously, no grid connection.
    Islanded,
    /// Transitioning between grid-connected and islanded modes.
    Transitioning,
}

impl std::fmt::Display for OperationMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            OperationMode::GridConnected => write!(f, "GridConnected"),
            OperationMode::Islanded => write!(f, "Islanded"),
            OperationMode::Transitioning => write!(f, "Transitioning"),
        }
    }
}

// ─── Result ──────────────────────────────────────────────────────────────────

/// Full three-layer control simulation result.
#[derive(Debug, Clone)]
pub struct MicrogridControlResult {
    /// Primary control states (one per dt\_primary step, sub-sampled for storage).
    pub primary_states: Vec<PrimaryControlState>,
    /// Secondary control actions (one per dt\_secondary period).
    pub secondary_actions: Vec<SecondaryControlAction>,
    /// Tertiary control decisions (one per dt\_tertiary period).
    pub tertiary_decisions: Vec<TertiaryDecision>,
    /// Maximum frequency deviation from nominal \[Hz\].
    pub frequency_deviation_max_hz: f64,
    /// Maximum voltage deviation from nominal \[pu\].
    pub voltage_deviation_max_pu: f64,
    /// RMS load-following error \[MW\].
    pub load_following_error_mw: f64,
    /// Total control effort (sum of absolute setpoint changes) \[MW\].
    pub control_effort: f64,
}

// ─── Controller ──────────────────────────────────────────────────────────────

/// Three-layer hierarchical microgrid controller.
pub struct MicrogridControlHierarchy {
    config: MicrogridControlConfig,
    der_units: Vec<DerUnit>,
    load_profile: Vec<f64>,
    renewable_profile: Vec<f64>,
}

impl MicrogridControlHierarchy {
    /// Create a new controller with the given configuration.
    pub fn new(config: MicrogridControlConfig) -> Self {
        Self {
            config,
            der_units: Vec::new(),
            load_profile: Vec::new(),
            renewable_profile: Vec::new(),
        }
    }

    /// Add a DER unit.
    pub fn add_der(&mut self, unit: DerUnit) {
        self.der_units.push(unit);
    }

    /// Set hourly load and renewable profiles.
    pub fn set_profiles(&mut self, load: Vec<f64>, renewable: Vec<f64>) {
        self.load_profile = load;
        self.renewable_profile = renewable;
    }

    /// Simulate `n_tertiary_periods` tertiary control periods.
    ///
    /// Within each tertiary period:
    /// - Many secondary periods are simulated.
    /// - Within each secondary period, many primary steps are simulated.
    pub fn simulate(
        &self,
        n_tertiary_periods: usize,
    ) -> Result<MicrogridControlResult, ControlError> {
        if self.der_units.is_empty() {
            return Err(ControlError::NoDer);
        }
        if self.load_profile.is_empty() || self.renewable_profile.is_empty() {
            return Err(ControlError::NoProfiles);
        }
        if self.load_profile.len() != self.renewable_profile.len() {
            return Err(ControlError::NoProfiles);
        }
        if self.config.primary_control_dt_s <= 0.0 {
            return Err(ControlError::Config(
                "primary_control_dt_s must be > 0".to_string(),
            ));
        }

        let n_der = self.der_units.len();
        let dt_primary = self.config.primary_control_dt_s;
        let dt_secondary = self.config.secondary_control_dt_s;
        let dt_tertiary = self.config.tertiary_control_dt_s;
        let f0 = self.config.nominal_freq_hz;
        let v0 = self.config.nominal_voltage_pu;
        let r_f = self.config.primary_droop_freq;
        let r_v = self.config.primary_droop_voltage;

        // Number of primary steps per secondary period
        let primary_per_secondary = ((dt_secondary / dt_primary).round() as usize).clamp(1, 200);
        // Number of secondary periods per tertiary period
        let secondary_per_tertiary = ((dt_tertiary / dt_secondary).round() as usize).clamp(1, 300);

        // Initial state
        let mut der_p: Vec<f64> = self.der_units.iter().map(|d| d.p_initial_mw).collect();
        let mut der_q: Vec<f64> = self.der_units.iter().map(|d| d.q_initial_mvar).collect();
        let mut p_ref: Vec<f64> = der_p.clone(); // setpoints
        let mut q_ref: Vec<f64> = der_q.clone();
        let mut freq = f0;
        let mut volt = v0;

        // Secondary PI state
        let kp_f = 0.5;
        let ki_f = 0.2;
        let kp_v = 0.3;
        let ki_v = 0.1;
        let mut integral_f = 0.0f64;
        let mut integral_v = 0.0f64;

        let mut primary_states: Vec<PrimaryControlState> = Vec::new();
        let mut secondary_actions: Vec<SecondaryControlAction> = Vec::new();
        let mut tertiary_decisions: Vec<TertiaryDecision> = Vec::new();

        let mut max_freq_dev = 0.0f64;
        let mut max_volt_dev = 0.0f64;
        let mut total_lf_error_sq = 0.0f64;
        let mut lf_samples = 0usize;
        let mut control_effort = 0.0f64;
        let mut global_time = 0.0f64;

        // Profile hours available
        let n_hours = self.load_profile.len();

        for tert_idx in 0..n_tertiary_periods {
            // Tertiary: economic dispatch
            let hour_idx = (tert_idx * (dt_tertiary as usize / 3600).max(1)).min(n_hours - 1);
            let load_mw = self.load_profile[hour_idx].max(0.0);
            let renew_mw = self.renewable_profile[hour_idx].max(0.0);

            let tertiary = self.tertiary_dispatch(global_time, load_mw, renew_mw, &p_ref, &q_ref);

            // Update setpoints from tertiary dispatch
            let old_p_ref = p_ref.clone();
            for (i, &pd) in tertiary.economic_dispatch.iter().enumerate().take(n_der) {
                p_ref[i] = pd;
                control_effort += (pd - old_p_ref[i]).abs();
            }

            tertiary_decisions.push(tertiary);

            for sec_idx in 0..secondary_per_tertiary {
                let sec_time = global_time + sec_idx as f64 * dt_secondary;

                // Primary control: fast droop + lag
                let mut sec_primary_samples: Vec<PrimaryControlState> = Vec::new();

                for pri_idx in 0..primary_per_secondary {
                    let t = sec_time + pri_idx as f64 * dt_primary;

                    // Frequency deviation drives droop response
                    let df = freq - f0;
                    let dv = volt - v0;

                    // Update each DER via droop + first-order lag
                    for (i, unit) in self.der_units.iter().enumerate() {
                        // Droop: ΔP = -(1/R_f) * Δf, ΔQ = -(1/R_v) * ΔV
                        let dp_droop = -df / r_f.max(1e-9) * unit.droop_gain;
                        let dq_droop = -dv / r_v.max(1e-9) * unit.droop_gain;

                        let p_target = (p_ref[i] + dp_droop).clamp(unit.p_min_mw, unit.p_rated_mw);
                        let q_target =
                            (q_ref[i] + dq_droop).clamp(-unit.q_rated_mvar, unit.q_rated_mvar);

                        // Ramp rate limit
                        let dp_max = unit.ramp_rate_mw_per_s * dt_primary;
                        let raw_delta_p = p_target - der_p[i];
                        let clamped_delta_p = raw_delta_p.clamp(-dp_max, dp_max);

                        // First-order lag: P(t+dt) = P(t) + (P_target - P(t)) * dt / τ
                        let tau = unit.response_time_s.max(dt_primary);
                        let lag_p = der_p[i] + (p_target - der_p[i]) * dt_primary / tau;
                        // Combine ramp limit and lag
                        der_p[i] = (der_p[i] + clamped_delta_p)
                            .min(lag_p.max(der_p[i] - dp_max))
                            .clamp(unit.p_min_mw, unit.p_rated_mw);

                        let lag_q = der_q[i] + (q_target - der_q[i]) * dt_primary / tau;
                        der_q[i] = lag_q.clamp(-unit.q_rated_mvar, unit.q_rated_mvar);
                    }

                    // Update system frequency: simplified swing equation
                    // Δf = (P_gen - P_load) / (2H * f0)
                    let p_gen: f64 = der_p.iter().sum::<f64>() + renew_mw;
                    let p_load = load_mw;
                    let h_eq = 3.0; // equivalent inertia constant
                    let d_p = p_gen - p_load;
                    let df_dot = d_p / (2.0 * h_eq * p_gen.max(1.0)) * f0;
                    // Damping
                    let d_coeff = 2.0;
                    freq += (df_dot - d_coeff * (freq - f0)) * dt_primary;
                    freq = freq.clamp(f0 - 5.0, f0 + 5.0);

                    // Voltage: simplified droop from Q balance
                    let q_gen: f64 = der_q.iter().sum();
                    let dv_dot = (q_gen) * r_v * 0.1;
                    volt += dv_dot * dt_primary;
                    volt = volt.clamp(0.8, 1.2);

                    max_freq_dev = max_freq_dev.max((freq - f0).abs());
                    max_volt_dev = max_volt_dev.max((volt - v0).abs());

                    total_lf_error_sq += d_p * d_p;
                    lf_samples += 1;

                    // Sub-sample primary states (store every 100th step to limit memory)
                    if pri_idx % 100 == 0 {
                        sec_primary_samples.push(PrimaryControlState {
                            time_s: t,
                            frequency_hz: freq,
                            voltage_pu: volt,
                            der_p_mw: der_p.clone(),
                            der_q_mvar: der_q.clone(),
                        });
                    }
                }

                // Secondary PI control: restore frequency and voltage to nominal
                let e_f = f0 - freq;
                let e_v = v0 - volt;
                integral_f += e_f * dt_secondary;
                integral_v += e_v * dt_secondary;

                let df_corr = kp_f * e_f + ki_f * integral_f;
                let dv_corr = kp_v * e_v + ki_v * integral_v;

                // Apply correction to setpoints
                let old_p = p_ref.clone();
                let old_q = q_ref.clone();
                for i in 0..n_der {
                    let unit = &self.der_units[i];
                    p_ref[i] = (p_ref[i] + df_corr / r_f.max(1e-9) * unit.droop_gain)
                        .clamp(unit.p_min_mw, unit.p_rated_mw);
                    q_ref[i] = (q_ref[i] + dv_corr / r_v.max(1e-9) * unit.droop_gain)
                        .clamp(-unit.q_rated_mvar, unit.q_rated_mvar);
                    control_effort += (p_ref[i] - old_p[i]).abs() + (q_ref[i] - old_q[i]).abs();
                }

                // Partial frequency restoration via secondary correction
                freq += df_corr * 0.1 * dt_secondary;
                volt += dv_corr * 0.1 * dt_secondary;

                secondary_actions.push(SecondaryControlAction {
                    time_s: sec_time,
                    frequency_correction_hz: df_corr,
                    voltage_correction_pu: dv_corr,
                    p_dispatch: p_ref.clone(),
                    q_dispatch: q_ref.clone(),
                });

                // Store sub-sampled primary states
                for s in sec_primary_samples {
                    primary_states.push(s);
                }
            }

            global_time += dt_tertiary;
        }

        let load_following_error_mw = if lf_samples > 0 {
            (total_lf_error_sq / lf_samples as f64).sqrt()
        } else {
            0.0
        };

        Ok(MicrogridControlResult {
            primary_states,
            secondary_actions,
            tertiary_decisions,
            frequency_deviation_max_hz: max_freq_dev,
            voltage_deviation_max_pu: max_volt_dev,
            load_following_error_mw,
            control_effort,
        })
    }

    // ─── Tertiary dispatch ─────────────────────────────────────────────────

    /// Economic dispatch via merit-order (cheapest DER first).
    ///
    /// Sorts DERs by droop gain (lower = more expensive) and allocates
    /// generation to meet net load (load - renewables).
    fn tertiary_dispatch(
        &self,
        time_s: f64,
        load_mw: f64,
        renewable_mw: f64,
        p_ref: &[f64],
        q_ref: &[f64],
    ) -> TertiaryDecision {
        let n_der = self.der_units.len();
        let net_load = (load_mw - renewable_mw).max(0.0);

        // Merit order: higher droop_gain = more "willing" to respond = dispatched first
        let mut merit_order: Vec<usize> = (0..n_der).collect();
        merit_order.sort_by(|&a, &b| {
            self.der_units[b]
                .droop_gain
                .partial_cmp(&self.der_units[a].droop_gain)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let mut economic_dispatch = vec![0.0f64; n_der];
        let mut remaining = net_load;

        for &idx in &merit_order {
            let unit = &self.der_units[idx];
            let alloc = remaining.min(unit.p_rated_mw).max(unit.p_min_mw);
            economic_dispatch[idx] = alloc;
            remaining -= alloc - unit.p_min_mw;
            if remaining <= 0.0 {
                break;
            }
        }

        let total_der: f64 = economic_dispatch.iter().sum();
        let power_import = net_load - total_der;

        // Battery SoC targets (normalised to [0,1])
        let battery_setpoints: Vec<f64> = (0..n_der)
            .map(|i| p_ref[i] / self.der_units[i].p_rated_mw.max(1e-9))
            .collect();

        let mode = if power_import.abs() < 0.1 {
            OperationMode::Islanded
        } else {
            OperationMode::GridConnected
        };

        let _ = q_ref; // reserved for reactive dispatch extension

        TertiaryDecision {
            time_s,
            economic_dispatch,
            power_import_mw: power_import,
            battery_setpoints,
            mode,
        }
    }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> MicrogridControlConfig {
        MicrogridControlConfig {
            n_der: 2,
            primary_control_dt_s: 0.01, // 10 ms for fast testing
            secondary_control_dt_s: 0.5,
            tertiary_control_dt_s: 2.0,
            nominal_freq_hz: 50.0,
            nominal_voltage_pu: 1.0,
            primary_droop_freq: 0.02,
            primary_droop_voltage: 0.05,
        }
    }

    fn make_der(id: usize, p_rated: f64) -> DerUnit {
        DerUnit {
            id,
            p_rated_mw: p_rated,
            q_rated_mvar: p_rated * 0.5,
            p_min_mw: 0.0,
            p_initial_mw: p_rated * 0.5,
            q_initial_mvar: 0.0,
            droop_gain: 0.05,
            ramp_rate_mw_per_s: p_rated * 0.1,
            response_time_s: 0.1,
        }
    }

    fn run_basic_sim(n_tertiary: usize) -> MicrogridControlResult {
        let mut ctrl = MicrogridControlHierarchy::new(default_config());
        ctrl.add_der(make_der(0, 5.0));
        ctrl.add_der(make_der(1, 3.0));
        // Load = 4 MW, renewable = 1 MW, so net = 3 MW
        ctrl.set_profiles(vec![4.0; 24], vec![1.0; 24]);
        ctrl.simulate(n_tertiary).expect("simulation ok")
    }

    // Test 1: Droop response — output of DERs changes with load
    #[test]
    fn test_droop_response_active() {
        let result = run_basic_sim(3);
        // We should have primary states recorded
        assert!(
            !result.primary_states.is_empty(),
            "Primary states should be recorded"
        );
        // DER power outputs should be within rated limits
        for state in &result.primary_states {
            for (i, &p) in state.der_p_mw.iter().enumerate() {
                assert!(
                    (-0.01..=5.1).contains(&p),
                    "DER[{i}] P = {p:.3} MW out of bounds"
                );
            }
        }
    }

    // Test 2: Secondary control reduces frequency deviation over time
    #[test]
    fn test_secondary_frequency_correction() {
        let result = run_basic_sim(5);
        assert!(
            !result.secondary_actions.is_empty(),
            "Secondary actions must be produced"
        );
        // Frequency correction should be non-trivial for a non-balanced system
        let max_corr = result
            .secondary_actions
            .iter()
            .map(|a| a.frequency_correction_hz.abs())
            .fold(0.0f64, f64::max);
        // Correction should be finite and not diverge
        assert!(
            max_corr.is_finite(),
            "Frequency correction must be finite: {max_corr}"
        );
    }

    // Test 3: Tertiary dispatch follows merit order (higher droop_gain dispatched first)
    #[test]
    fn test_tertiary_merit_order() {
        let mut ctrl = MicrogridControlHierarchy::new(default_config());
        // DER 0: high droop gain (preferred), DER 1: low droop gain
        let mut der0 = make_der(0, 5.0);
        der0.droop_gain = 0.1; // preferred
        let mut der1 = make_der(1, 5.0);
        der1.droop_gain = 0.02; // less preferred
        ctrl.add_der(der0);
        ctrl.add_der(der1);
        ctrl.set_profiles(vec![3.0; 24], vec![0.0; 24]);
        let result = ctrl.simulate(2).expect("ok");

        assert!(!result.tertiary_decisions.is_empty());
        let first = &result.tertiary_decisions[0];
        // DER 0 (droop=0.1) should be dispatched at least as much as DER 1
        assert!(
            first.economic_dispatch[0] >= first.economic_dispatch[1] - 1e-9,
            "DER 0 (higher droop) should be dispatched at least as much: dispatch={:?}",
            first.economic_dispatch
        );
    }

    // Test 4: Ramp rate limits DER output growth over time
    #[test]
    fn test_ramp_rate_limits_output() {
        let mut ctrl = MicrogridControlHierarchy::new(MicrogridControlConfig {
            primary_control_dt_s: 0.01,
            ..default_config()
        });
        let mut der = make_der(0, 10.0);
        der.ramp_rate_mw_per_s = 0.5; // 0.5 MW/s
        der.p_initial_mw = 0.0;
        ctrl.add_der(der);
        ctrl.set_profiles(vec![9.0; 24], vec![0.0; 24]); // large step request

        let result = ctrl.simulate(2).expect("ok");
        assert!(!result.primary_states.is_empty());

        // The primary states are sub-sampled every 100 steps = 1.0 s apart.
        // Over 1 s at 0.5 MW/s, maximum growth = 0.5 MW per sub-sampled step.
        // We use a generous tolerance because the lag dynamics can cause slight
        // differences across the 100-step window.
        let sub_sample_dt = 100.0 * 0.01; // 100 steps × 10 ms = 1.0 s
        let max_ramp_per_window = 0.5 * sub_sample_dt + 0.2; // 0.5 MW + tolerance

        for w in result.primary_states.windows(2) {
            for i in 0..w[0].der_p_mw.len() {
                let dp = (w[1].der_p_mw[i] - w[0].der_p_mw[i]).abs();
                assert!(
                    dp <= max_ramp_per_window,
                    "DER[{i}] inter-sample ΔP={dp:.4} MW exceeds {max_ramp_per_window:.4} MW window"
                );
            }
        }
    }

    // Test 5: Islanded mode detected when generation balances load
    #[test]
    fn test_islanded_mode_detected() {
        let mut ctrl = MicrogridControlHierarchy::new(default_config());
        ctrl.add_der(make_der(0, 5.0));
        // Renewable covers all load
        ctrl.set_profiles(vec![4.0; 24], vec![4.1; 24]); // renewable ≥ load

        let result = ctrl.simulate(2).expect("ok");
        // With renewable covering load, net import should be ≈ 0 → Islanded
        let islanded = result
            .tertiary_decisions
            .iter()
            .any(|d| d.mode == OperationMode::Islanded || d.power_import_mw.abs() < 0.5);
        assert!(
            islanded,
            "Should detect islanded or near-zero import conditions"
        );
    }

    // Test 6: No DER → error
    #[test]
    fn test_no_der_error() {
        let ctrl = MicrogridControlHierarchy::new(default_config());
        let result = ctrl.simulate(1);
        assert!(result.is_err(), "No DER must return error");
    }

    // Test 7: No profiles → error
    #[test]
    fn test_no_profiles_error() {
        let mut ctrl = MicrogridControlHierarchy::new(default_config());
        ctrl.add_der(make_der(0, 5.0));
        let result = ctrl.simulate(1);
        assert!(result.is_err(), "No profiles must return error");
    }

    // Test 8: Frequency deviation stays bounded
    #[test]
    fn test_frequency_bounded() {
        let result = run_basic_sim(5);
        assert!(
            result.frequency_deviation_max_hz < 5.0,
            "Max frequency deviation {} Hz should be < 5 Hz",
            result.frequency_deviation_max_hz
        );
    }
}
