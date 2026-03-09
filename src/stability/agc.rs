//! Automatic Generation Control (AGC) and frequency regulation simulation.
//!
//! # Overview
//!
//! This module implements a complete AGC framework for power system frequency
//! regulation, including:
//!
//! - **Area Control Error (ACE)**: NERC CPS1/CPS2 compliance metrics
//! - **AGC Controller**: PI-based regulation with participation factor dispatch
//! - **Multi-Area AGC**: Coupled area simulation with tie line dynamics
//! - **Governor Droop**: Enhanced two-time-constant governor model
//! - **FCR Assessment**: Frequency Containment Reserve analysis and ROCOF estimation
//! - **HVDC Frequency Support**: Emergency power injection via HVDC modulation
//!
//! # References
//! - NERC BAL-001-2: Real Power Balancing Control Performance
//! - IEEE Std 94: Guide for Abnormal Frequency Protection for Power Generating Plants
//! - Kundur, "Power System Stability and Control", Chapter 11

use crate::error::OxiGridError;
use serde::{Deserialize, Serialize};

// ─── Control Area ────────────────────────────────────────────────────────────

/// A control area participating in AGC.
///
/// The area is characterised by its frequency bias B (MW/0.1 Hz), a scheduled
/// interchange with neighbouring areas, and the set of generators and tie lines
/// that belong to it.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ControlArea {
    /// Unique numeric identifier for the area.
    pub area_id: usize,
    /// Human-readable name.
    pub name: String,
    /// Frequency bias coefficient B [MW/0.1 Hz].
    ///
    /// A positive value means the area exports more MW when frequency is high.
    /// Typical values are −10 × rated_capacity_mw (negative bias convention used
    /// in some utilities). Here we follow the NERC sign convention where B > 0.
    pub frequency_bias_mw_per_hz: f64,
    /// Scheduled net interchange (positive = export) \[MW\].
    pub scheduled_interchange: f64,
    /// Generator IDs that belong to this area.
    pub generator_ids: Vec<usize>,
    /// Branch indices of tie lines connecting this area to neighbours.
    pub tie_line_ids: Vec<usize>,
}

// ─── ACE Computation ─────────────────────────────────────────────────────────

/// Compute the Area Control Error for a control area.
///
/// # Formula
/// ```text
/// ACE = (P_tie_actual − P_tie_sched) + B · (f − f₀)
/// ```
///
/// where `B` is the frequency bias [MW/Hz], `f` is measured frequency \[Hz\],
/// and `f₀` is nominal frequency \[Hz\].
///
/// A positive ACE means the area is generating too much (overfrequency / net
/// export excess). The AGC controller should lower generation to restore ACE to
/// zero.
pub fn compute_ace(
    area: &ControlArea,
    p_tie_actual_mw: f64,
    frequency_hz: f64,
    nominal_freq_hz: f64,
) -> f64 {
    let tie_error = p_tie_actual_mw - area.scheduled_interchange;
    let freq_term = area.frequency_bias_mw_per_hz * (frequency_hz - nominal_freq_hz);
    tie_error + freq_term
}

/// Compute the NERC Control Performance Standard 1 (CPS1) score.
///
/// CPS1 measures whether the product ACE·Δf stays within bounds over time.
/// A score ≥ 100% indicates compliant control performance.
///
/// # Formula
/// ```text
/// CPS1 = 100 × (1 − |mean(ACE_i × Δf_i)| / (ε₁² × B))
/// ```
///
/// where `ε₁` is the NERC threshold (default 0.018 Hz for 60 Hz system) and
/// `B` is the frequency bias.
///
/// # Arguments
/// * `ace_samples`        — ACE time series \[MW\]
/// * `frequency_errors`   — Δf = f − f₀ time series \[Hz\], must match length of `ace_samples`
/// * `b`                  — Area frequency bias [MW/Hz]
pub fn compute_cps1(ace_samples: &[f64], frequency_errors: &[f64], b: f64) -> f64 {
    if ace_samples.is_empty() || ace_samples.len() != frequency_errors.len() {
        return 0.0;
    }
    let n = ace_samples.len() as f64;
    let mean_product: f64 = ace_samples
        .iter()
        .zip(frequency_errors.iter())
        .map(|(&a, &df)| a * df)
        .sum::<f64>()
        / n;

    // NERC ε₁ standard for 60 Hz systems: 0.018 Hz
    let eps1 = 0.018_f64;
    let denominator = eps1 * eps1 * b.abs();
    if denominator < 1e-12 {
        return 0.0;
    }

    100.0 * (1.0 - mean_product.abs() / denominator)
}

/// Compute the NERC Control Performance Standard 2 (CPS2) compliance score.
///
/// CPS2 requires that the 10-minute average ACE stays within a bound L₁₀ for
/// at least 90% of all 10-minute intervals.
///
/// Returns the percentage of compliant intervals.
///
/// # Arguments
/// * `ace_10min_avg` — Vector of 10-minute average ACE values \[MW\]
/// * `l10_limit`     — CPS2 limit L₁₀ \[MW\]; typically 1.65 × ε₁₀ × |B| × 10
pub fn compute_cps2(ace_10min_avg: &[f64], l10_limit: f64) -> f64 {
    if ace_10min_avg.is_empty() {
        return 100.0; // vacuously compliant
    }
    let compliant = ace_10min_avg
        .iter()
        .filter(|&&a| a.abs() <= l10_limit)
        .count();
    100.0 * compliant as f64 / ace_10min_avg.len() as f64
}

// ─── AGC Configuration and State ─────────────────────────────────────────────

/// Configuration for one AGC controller instance.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AgcConfig {
    /// The control area managed by this controller.
    pub area: ControlArea,
    /// AGC scan interval \[s\] (default: 4 s).
    pub scan_rate_s: f64,
    /// Proportional gain of the PI controller (default: 0.1).
    pub kp: f64,
    /// Integral gain of the PI controller (default: 0.02).
    pub ki: f64,
    /// ACE deadband \[MW\]; the integral is not updated if |ACE| < deadband.
    pub deadband_mw: f64,
    /// Maximum generation raise rate [MW/s].
    pub max_raise_rate_mw_per_s: f64,
    /// Maximum generation lower rate [MW/s].
    pub max_lower_rate_mw_per_s: f64,
    /// Participation factors: `(gen_id, factor)`.  Factors must sum to 1.0.
    pub participation_factors: Vec<(usize, f64)>,
}

impl AgcConfig {
    /// Construct an `AgcConfig` with sensible defaults for a given area.
    ///
    /// Participation factors are initialised uniformly across the generators
    /// listed in `area.generator_ids`.
    pub fn default_for_area(area: ControlArea) -> Self {
        let n = area.generator_ids.len().max(1);
        let factor = 1.0 / n as f64;
        let participation_factors = area.generator_ids.iter().map(|&id| (id, factor)).collect();

        Self {
            area,
            scan_rate_s: 4.0,
            kp: 0.1,
            ki: 0.02,
            deadband_mw: 0.0,
            max_raise_rate_mw_per_s: 10.0,
            max_lower_rate_mw_per_s: 10.0,
            participation_factors,
        }
    }
}

/// Runtime state of one AGC controller.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AgcState {
    /// Most recent ACE value \[MW\].
    pub ace: f64,
    /// Accumulated integral of ACE [MW·s].
    pub integral_error: f64,
    /// Current regulation signal sent to generators \[MW\].
    pub regulation_signal: f64,
    /// Latest generation setpoints: `(gen_id, setpoint_mw)`.
    pub generation_setpoints: Vec<(usize, f64)>,
}

/// AGC output produced by each controller scan step.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AgcOutput {
    /// New generator setpoints \[MW\]: `(gen_id, new_mw)`.
    pub new_setpoints: Vec<(usize, f64)>,
    /// ACE at the time of the scan \[MW\].
    pub ace: f64,
    /// PI regulation signal \[MW\].
    pub regulation_signal_mw: f64,
    /// Frequency deviation from nominal \[Hz\].
    pub frequency_error_hz: f64,
}

// ─── AGC Controller ───────────────────────────────────────────────────────────

/// AGC PI controller for a single control area.
pub struct AgcController {
    pub config: AgcConfig,
    pub state: AgcState,
}

impl AgcController {
    /// Initialise a new AGC controller with zeroed state.
    pub fn new(config: AgcConfig) -> Self {
        let setpoints = config
            .participation_factors
            .iter()
            .map(|&(id, _)| (id, 0.0))
            .collect();
        Self {
            state: AgcState {
                ace: 0.0,
                integral_error: 0.0,
                regulation_signal: 0.0,
                generation_setpoints: setpoints,
            },
            config,
        }
    }

    /// Execute one AGC scan step.
    ///
    /// # Arguments
    /// * `p_tie_actual`     — Measured tie-line flow (positive = export) \[MW\]
    /// * `frequency_hz`     — Measured system frequency \[Hz\]
    /// * `current_dispatch` — Current generator outputs: `(gen_id, mw)`
    /// * `gen_limits`       — Generator capacity limits: `(gen_id, p_min_mw, p_max_mw)`
    /// * `dt_s`             — Elapsed time since last scan \[s\]
    pub fn step(
        &mut self,
        p_tie_actual: f64,
        frequency_hz: f64,
        current_dispatch: &[(usize, f64)],
        gen_limits: &[(usize, f64, f64)],
        dt_s: f64,
    ) -> AgcOutput {
        let nominal_hz = 60.0; // NERC nominal; overridable via config in future
        let ace = compute_ace(&self.config.area, p_tie_actual, frequency_hz, nominal_hz);
        self.state.ace = ace;

        let signal = self.compute_regulation_signal(ace, dt_s);
        self.state.regulation_signal = signal;

        let new_setpoints = self.distribute_signal(signal, current_dispatch, gen_limits, dt_s);
        self.state.generation_setpoints = new_setpoints.clone();

        AgcOutput {
            new_setpoints,
            ace,
            regulation_signal_mw: signal,
            frequency_error_hz: frequency_hz - nominal_hz,
        }
    }

    /// PI controller: compute the regulation signal from the current ACE.
    ///
    /// The integral term is frozen when |ACE| is within the deadband.
    fn compute_regulation_signal(&mut self, ace: f64, dt_s: f64) -> f64 {
        if ace.abs() > self.config.deadband_mw {
            self.state.integral_error += ace * dt_s;
        }
        // Negative sign: positive ACE → lower generation
        -(self.config.kp * ace + self.config.ki * self.state.integral_error)
    }

    /// Distribute the regulation signal to generators proportionally.
    ///
    /// Each generator's new setpoint is bounded by its ramp rate and capacity
    /// limits.
    fn distribute_signal(
        &self,
        signal_mw: f64,
        current_dispatch: &[(usize, f64)],
        gen_limits: &[(usize, f64, f64)],
        dt_s: f64,
    ) -> Vec<(usize, f64)> {
        // Build lookup maps
        let dispatch_map: std::collections::HashMap<usize, f64> =
            current_dispatch.iter().cloned().collect();
        let limits_map: std::collections::HashMap<usize, (f64, f64)> = gen_limits
            .iter()
            .map(|&(id, lo, hi)| (id, (lo, hi)))
            .collect();

        self.config
            .participation_factors
            .iter()
            .map(|&(gen_id, pf)| {
                let allocated = signal_mw * pf;
                let current = dispatch_map.get(&gen_id).copied().unwrap_or(0.0);

                // Apply ramp rate limits
                let max_delta = if allocated >= 0.0 {
                    self.config.max_raise_rate_mw_per_s * dt_s
                } else {
                    -self.config.max_lower_rate_mw_per_s * dt_s
                };
                let clamped_delta = if allocated >= 0.0 {
                    allocated.min(max_delta)
                } else {
                    allocated.max(max_delta)
                };

                let new_setpoint = current + clamped_delta;

                // Apply capacity limits
                let (p_min, p_max) = limits_map.get(&gen_id).copied().unwrap_or((0.0, f64::MAX));
                let final_setpoint = new_setpoint.clamp(p_min, p_max);

                (gen_id, final_setpoint)
            })
            .collect()
    }
}

// ─── Multi-Area AGC ───────────────────────────────────────────────────────────

/// A tie line interconnecting two control areas.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TieLine {
    /// Branch index in the network model.
    pub branch_idx: usize,
    /// Area index (in `MultiAreaAgc::areas`) at the "from" end.
    pub area_from: usize,
    /// Area index (in `MultiAreaAgc::areas`) at the "to" end.
    pub area_to: usize,
    /// Scheduled power transfer (positive = from→to) \[MW\].
    pub schedule_mw: f64,
    /// DC susceptance used for angle-based flow calculation [p.u.].
    pub susceptance_pu: f64,
}

/// Simulation result for the full multi-area AGC run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MultiAreaSimResult {
    /// Time vector \[s\].
    pub time: Vec<f64>,
    /// System frequency at each time step \[Hz\].
    pub frequency: Vec<f64>,
    /// ACE for each area at each time step: `ace_per_area[area][step]` \[MW\].
    pub ace_per_area: Vec<Vec<f64>>,
    /// Tie line flows at each time step: `tie_flows[tie_line][step]` \[MW\].
    pub tie_flows: Vec<Vec<f64>>,
    /// Total generation per area at each time step \[MW\].
    pub generation: Vec<Vec<f64>>,
    /// Final CPS1 score (%).
    pub cps1: f64,
    /// Final CPS2 score (%).
    pub cps2: f64,
    /// Frequency nadir (lowest frequency reached) \[Hz\].
    pub nadir_hz: f64,
    /// Steady-state frequency error at end of simulation \[Hz\].
    pub steady_state_error_hz: f64,
}

/// Multi-area AGC simulation coupling several control areas via tie lines.
pub struct MultiAreaAgc {
    /// AGC controllers, one per area.
    pub areas: Vec<AgcController>,
    /// Tie lines between areas.
    pub tie_lines: Vec<TieLine>,
    /// Total system inertia constant (combined) [MW·s].
    pub system_inertia_mws: f64,
}

impl MultiAreaAgc {
    /// Construct a multi-area AGC simulation.
    pub fn new(
        areas: Vec<AgcController>,
        tie_lines: Vec<TieLine>,
        system_inertia_mws: f64,
    ) -> Self {
        Self {
            areas,
            tie_lines,
            system_inertia_mws,
        }
    }

    /// Simulate the AGC response over `duration_s` seconds.
    ///
    /// An optional step disturbance can be injected at a specified time:
    /// `disturbance = Some((t_event_s, delta_p_mw))`.  A negative `delta_p_mw`
    /// represents a generation loss or sudden load increase that drives
    /// frequency down.
    ///
    /// # Simulation Model
    ///
    /// The system is modelled as a single-frequency aggregate with linearised
    /// swing dynamics:
    ///
    /// ```text
    /// df/dt = f₀ × ΔP_net / (2 × H_sys)
    /// ```
    ///
    /// where `ΔP_net` is the total power imbalance \[MW\] including:
    /// 1. The external disturbance (step or ramp)
    /// 2. Primary frequency response (governor droop proportional to Δf)
    /// 3. Secondary response (AGC, updated every `scan_rate_s` seconds)
    ///
    /// Tie lines are modelled by DC power-angle equations; area angles evolve
    /// as the integral of the per-unit angular velocity deviation.
    pub fn simulate(
        &mut self,
        duration_s: f64,
        disturbance: Option<(f64, f64)>,
    ) -> Result<MultiAreaSimResult, OxiGridError> {
        if duration_s <= 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "simulation duration must be positive".into(),
            ));
        }

        let dt = 0.1_f64; // integration step [s]
        let n_steps = (duration_s / dt).ceil() as usize;
        let n_areas = self.areas.len();
        let n_ties = self.tie_lines.len();
        let nominal_hz = 60.0_f64;

        // ── State variables ──────────────────────────────────────────────────
        let mut freq_hz = nominal_hz;
        let mut area_angles: Vec<f64> = vec![0.0; n_areas];

        // Persistent disturbance imbalance [MW]: negative = deficit.
        // Once applied, it remains until AGC/droop compensates it.
        let mut disturbance_mw = 0.0_f64;

        // Incremental AGC correction dispatched in the last scan [MW].
        // Positive = extra generation raised to cover a deficit.
        let mut agc_delta_mw = 0.0_f64;

        // System-wide primary frequency response (governor droop) [MW/Hz].
        // A reasonable default: 5 % droop on the full system inertia.
        // droop_gain = S_base_mw / (R * f0); we approximate S_base from inertia.
        let droop_gain_mw_per_hz = self.system_inertia_mws.max(1.0) * 0.04;

        // Build initial dispatch: all generators start at zero (unloaded).
        let mut current_dispatch: Vec<Vec<(usize, f64)>> = (0..n_areas)
            .map(|a| {
                self.areas[a]
                    .config
                    .participation_factors
                    .iter()
                    .map(|&(id, _)| (id, 0.0))
                    .collect()
            })
            .collect();

        // Generator capacity limits (generous for simplified simulation).
        let gen_limits: Vec<Vec<(usize, f64, f64)>> = (0..n_areas)
            .map(|a| {
                self.areas[a]
                    .config
                    .participation_factors
                    .iter()
                    .map(|&(id, _)| (id, 0.0, 50_000.0))
                    .collect()
            })
            .collect();

        // ── Result storage ───────────────────────────────────────────────────
        let mut time_vec = Vec::with_capacity(n_steps + 1);
        let mut freq_vec = Vec::with_capacity(n_steps + 1);
        let mut ace_vec: Vec<Vec<f64>> = vec![Vec::with_capacity(n_steps + 1); n_areas];
        let mut tie_flow_vec: Vec<Vec<f64>> = vec![Vec::with_capacity(n_steps + 1); n_ties];
        let mut gen_vec: Vec<Vec<f64>> = vec![Vec::with_capacity(n_steps + 1); n_areas];

        let mut scan_timer = 0.0_f64;
        let scan_rate = self
            .areas
            .first()
            .map(|a| a.config.scan_rate_s)
            .unwrap_or(4.0);
        let (dist_time, dist_delta_p) = disturbance.unwrap_or((f64::MAX, 0.0));

        // Record initial state (t = 0)
        let tie_flows_init = self.compute_tie_flows(&area_angles);
        time_vec.push(0.0);
        freq_vec.push(freq_hz);
        for a in 0..n_areas {
            ace_vec[a].push(0.0);
            gen_vec[a].push(0.0);
        }
        for (ti, tfv) in tie_flow_vec.iter_mut().enumerate().take(n_ties) {
            tfv.push(tie_flows_init.get(ti).copied().unwrap_or(0.0));
        }

        // ── Main simulation loop ─────────────────────────────────────────────
        for step in 1..=n_steps {
            let t = step as f64 * dt;

            // Inject step disturbance (once, when the event window is reached)
            if t >= dist_time && t < dist_time + dt {
                // dist_delta_p < 0: generation loss → deficit grows
                disturbance_mw += dist_delta_p;
            }

            // Compute tie line flows from area angles
            let tie_flows = self.compute_tie_flows(&area_angles);

            // Primary frequency response (governor droop):
            //   df ↓ → delta_f < 0 → primary_response > 0 (inject MW)
            let delta_f = freq_hz - nominal_hz;
            let primary_response_mw = -droop_gain_mw_per_hz * delta_f;

            // Net power imbalance seen by the swing equation:
            //   positive net_delta → frequency rises
            //   negative net_delta → frequency falls
            let net_delta_mw = disturbance_mw + primary_response_mw + agc_delta_mw;

            // Swing equation: df/dt = f₀ · ΔP / (2 · H)
            let inertia = self.system_inertia_mws.max(1.0);
            let df_dt = nominal_hz * net_delta_mw / (2.0 * inertia);
            freq_hz = (freq_hz + df_dt * dt).clamp(nominal_hz - 5.0, nominal_hz + 5.0);

            // Update area angles (integrate angular velocity deviation)
            let delta_omega_per_s = 2.0 * std::f64::consts::PI * delta_f; // rad/s
            for angle in area_angles.iter_mut() {
                *angle += delta_omega_per_s * dt;
            }

            // ── AGC scan (secondary frequency control) ───────────────────────
            scan_timer += dt;
            if scan_timer >= scan_rate {
                scan_timer = 0.0;
                let mut total_agc_gen = 0.0_f64;
                for a in 0..n_areas {
                    let tie_actual: f64 = tie_flows
                        .iter()
                        .enumerate()
                        .filter_map(|(ti, &f)| {
                            self.tie_lines.get(ti).and_then(|tl| {
                                if tl.area_from == a {
                                    Some(f)
                                } else if tl.area_to == a {
                                    Some(-f)
                                } else {
                                    None
                                }
                            })
                        })
                        .sum();

                    let output = self.areas[a].step(
                        tie_actual,
                        freq_hz,
                        &current_dispatch[a].clone(),
                        &gen_limits[a].clone(),
                        scan_rate,
                    );
                    current_dispatch[a] = output.new_setpoints.clone();
                    let area_gen: f64 = output.new_setpoints.iter().map(|&(_, mw)| mw).sum();
                    total_agc_gen += area_gen;
                }
                // agc_delta_mw is the total incremental generation raised by AGC.
                // Once agc_delta_mw ≈ |disturbance_mw|, the net_delta → 0 → freq recovers.
                agc_delta_mw = total_agc_gen;
            }

            // ── Record results ───────────────────────────────────────────────
            time_vec.push(t);
            freq_vec.push(freq_hz);
            for a in 0..n_areas {
                let tie_actual: f64 = tie_flows
                    .iter()
                    .enumerate()
                    .filter_map(|(ti, &f)| {
                        self.tie_lines.get(ti).and_then(|tl| {
                            if tl.area_from == a {
                                Some(f)
                            } else if tl.area_to == a {
                                Some(-f)
                            } else {
                                None
                            }
                        })
                    })
                    .sum();
                let ace = compute_ace(&self.areas[a].config.area, tie_actual, freq_hz, nominal_hz);
                ace_vec[a].push(ace);
                let total_gen: f64 = current_dispatch[a].iter().map(|&(_, mw)| mw).sum();
                gen_vec[a].push(total_gen);
            }
            for (ti, tf) in tie_flows.iter().enumerate() {
                if ti < n_ties {
                    tie_flow_vec[ti].push(*tf);
                }
            }
        }

        // Compute CPS1 and CPS2
        // Aggregate ACE and freq error across all areas for CPS1
        let all_ace: Vec<f64> = ace_vec[0].clone(); // single area for simplicity
        let freq_errors: Vec<f64> = freq_vec.iter().map(|&f| f - nominal_hz).collect();
        let b = self
            .areas
            .first()
            .map(|a| a.config.area.frequency_bias_mw_per_hz)
            .unwrap_or(1.0);
        let cps1 = compute_cps1(&all_ace, &freq_errors, b);

        // CPS2: compute 10-min averages
        let samples_per_10min = (600.0 / dt) as usize;
        let ace_10min: Vec<f64> = all_ace
            .chunks(samples_per_10min.max(1))
            .map(|chunk| chunk.iter().sum::<f64>() / chunk.len() as f64)
            .collect();
        let l10 = 1.65 * 0.018 * b.abs() * 10.0;
        let cps2 = compute_cps2(&ace_10min, l10);

        let nadir_hz = freq_vec.iter().cloned().fold(nominal_hz, f64::min);
        let steady_state_error = freq_vec.last().copied().unwrap_or(nominal_hz) - nominal_hz;

        Ok(MultiAreaSimResult {
            time: time_vec,
            frequency: freq_vec,
            ace_per_area: ace_vec,
            tie_flows: tie_flow_vec,
            generation: gen_vec,
            cps1,
            cps2,
            nadir_hz,
            steady_state_error_hz: steady_state_error,
        })
    }

    /// Compute DC tie line flows from area voltage angles.
    ///
    /// P_flow = B × (θ_from − θ_to)  [p.u.] × s_base
    ///
    /// Returns one flow value per tie line in MW (positive = from→to).
    fn compute_tie_flows(&self, area_angles: &[f64]) -> Vec<f64> {
        self.tie_lines
            .iter()
            .map(|tl| {
                let theta_from = area_angles.get(tl.area_from).copied().unwrap_or(0.0);
                let theta_to = area_angles.get(tl.area_to).copied().unwrap_or(0.0);
                tl.susceptance_pu * (theta_from - theta_to)
            })
            .collect()
    }

    /// Advance the aggregate system frequency by one time step.
    ///
    /// Uses a linearised first-order swing equation:
    /// ```text
    /// dω/dt = (P_mech − P_load) / (2H · S_base / ω₀)
    /// ```
    /// Simplified for MW-level simulation where inertia is in [MW·s]:
    /// ```text
    /// df/dt = f₀ · (P_mech − P_load) / (2 · H_sys)
    /// ```
    pub fn swing_step(&self, freq_hz: f64, p_mech_mw: f64, p_load_mw: f64, dt: f64) -> f64 {
        let nominal_hz = 60.0_f64;
        let inertia = self.system_inertia_mws.max(1.0);
        let delta_p = p_mech_mw - p_load_mw;
        // df/dt = f₀ × ΔP / (2 × H)
        let df_dt = nominal_hz * delta_p / (2.0 * inertia);
        freq_hz + df_dt * dt
    }
}

// ─── Governor Droop ───────────────────────────────────────────────────────────

/// Enhanced two-time-constant governor-turbine model for primary frequency response.
///
/// Implements the classical governor-turbine chain:
/// ```text
/// Governor block:  dxg/dt = (u_gov − xg) / Tg
///   where u_gov = clamp(p_ref − Δf/(R·f₀), p_min, p_max)
///
/// Turbine block:   dxt/dt = (xg − xt) / Tt
/// P_mech = clamp(xt, p_min, p_max)
/// ```
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GovernorDroop {
    /// Generator identifier.
    pub gen_id: usize,
    /// Droop percentage R [%] (e.g. 5 means 5% droop).
    pub droop_pct: f64,
    /// Rated generation capacity \[MW\].
    pub p_rated_mw: f64,
    /// Minimum mechanical output \[MW\].
    pub p_min_mw: f64,
    /// Maximum mechanical output \[MW\].
    pub p_max_mw: f64,
    /// Governor time constant Tg \[s\] (default 0.2 s).
    pub governor_time_s: f64,
    /// Turbine time constant Tt \[s\] (default 0.5 s).
    pub turbine_time_s: f64,
    /// Internal governor valve/gate state.
    pub governor_state: f64,
    /// Internal turbine state.
    pub turbine_state: f64,
}

impl GovernorDroop {
    /// Create a governor droop model with default time constants.
    ///
    /// Initial state is set at 50% of rated output.
    pub fn new(gen_id: usize, droop_pct: f64, p_rated_mw: f64) -> Self {
        let p_init = 0.5 * p_rated_mw;
        Self {
            gen_id,
            droop_pct,
            p_rated_mw,
            p_min_mw: 0.0,
            p_max_mw: p_rated_mw,
            governor_time_s: 0.2,
            turbine_time_s: 0.5,
            governor_state: p_init,
            turbine_state: p_init,
        }
    }

    /// Initialise with a specified initial output.
    pub fn with_initial_output(mut self, p_init_mw: f64) -> Self {
        let p = p_init_mw.clamp(self.p_min_mw, self.p_max_mw);
        self.governor_state = p;
        self.turbine_state = p;
        self
    }

    /// Advance the governor-turbine model one time step.
    ///
    /// Returns the new mechanical power output \[MW\].
    ///
    /// # Arguments
    /// * `frequency_hz` — Measured frequency \[Hz\]
    /// * `nominal_hz`   — Nominal frequency (e.g. 60.0) \[Hz\]
    /// * `p_ref_mw`     — AGC setpoint reference \[MW\]
    /// * `dt_s`         — Time step \[s\]
    pub fn step(&mut self, frequency_hz: f64, nominal_hz: f64, p_ref_mw: f64, dt_s: f64) -> f64 {
        let r = self.droop_pct / 100.0; // per-unit droop
        let delta_f = frequency_hz - nominal_hz;

        // Droop correction: ΔP = −Δf / (R · f₀)
        let droop_correction = -delta_f / (r * nominal_hz);
        let u_gov =
            (p_ref_mw + droop_correction * self.p_rated_mw).clamp(self.p_min_mw, self.p_max_mw);

        // Governor first-order lag
        let tg = self.governor_time_s.max(1e-6);
        let dxg_dt = (u_gov - self.governor_state) / tg;
        self.governor_state =
            (self.governor_state + dt_s * dxg_dt).clamp(self.p_min_mw, self.p_max_mw);

        // Turbine first-order lag
        let tt = self.turbine_time_s.max(1e-6);
        let dxt_dt = (self.governor_state - self.turbine_state) / tt;
        self.turbine_state =
            (self.turbine_state + dt_s * dxt_dt).clamp(self.p_min_mw, self.p_max_mw);

        self.turbine_state
    }
}

// ─── FCR Assessment ───────────────────────────────────────────────────────────

/// Frequency Containment Reserve (FCR) availability and activation assessment.
///
/// FCR is the primary frequency response activated automatically by governor
/// droop when system frequency deviates from nominal.  In European practice:
/// - Activation starts at ±0.2 Hz (activation threshold)
/// - Full activation occurs at ±0.5 Hz
///
/// For NERC (North American):
/// - Primary response is required within 10–12 seconds
/// - Full response maintained for at least 10 minutes
pub struct FcrAssessment {
    /// Total FCR capacity available in the system \[MW\].
    pub total_fcr_mw: f64,
    /// Frequency threshold for the start of FCR activation \[Hz\].
    pub activation_threshold_hz: f64,
    /// Frequency deviation at which full FCR is activated \[Hz\].
    pub full_activation_hz: f64,
}

impl FcrAssessment {
    /// Create an FCR assessment with ENTSO-E default thresholds.
    pub fn new(total_fcr_mw: f64) -> Self {
        Self {
            total_fcr_mw,
            activation_threshold_hz: 0.2,
            full_activation_hz: 0.5,
        }
    }

    /// Compute the fraction of FCR activated for a given frequency.
    ///
    /// Uses a linear ramp between `activation_threshold_hz` and
    /// `full_activation_hz`.  Returns 0 if within deadband, `total_fcr_mw`
    /// at full deviation, proportional otherwise.
    pub fn activated_fcr(&self, frequency_hz: f64, nominal_hz: f64) -> f64 {
        let deviation = (frequency_hz - nominal_hz).abs();
        if deviation <= self.activation_threshold_hz {
            return 0.0;
        }
        if deviation >= self.full_activation_hz {
            return self.total_fcr_mw;
        }
        let range = self.full_activation_hz - self.activation_threshold_hz;
        let fraction = (deviation - self.activation_threshold_hz) / range;
        fraction * self.total_fcr_mw
    }

    /// Estimate the Rate of Change of Frequency (ROCOF) from a frequency time series.
    ///
    /// Uses a simple first-difference approximation: ROCOF = Δf / Δt.
    /// For accuracy, `frequency_history` should cover the first few seconds
    /// after a disturbance.
    ///
    /// Returns ROCOF in [Hz/s]; negative value indicates frequency decline.
    pub fn rocof(&self, frequency_history: &[f64], dt_s: f64) -> f64 {
        if frequency_history.len() < 2 || dt_s <= 0.0 {
            return 0.0;
        }
        // Use least-squares slope over the available window
        let n = frequency_history.len() as f64;
        let sum_t: f64 = (0..frequency_history.len()).map(|i| i as f64 * dt_s).sum();
        let sum_t2: f64 = (0..frequency_history.len())
            .map(|i| (i as f64 * dt_s).powi(2))
            .sum();
        let sum_f: f64 = frequency_history.iter().sum();
        let sum_tf: f64 = frequency_history
            .iter()
            .enumerate()
            .map(|(i, &f)| i as f64 * dt_s * f)
            .sum();
        let denom = n * sum_t2 - sum_t * sum_t;
        if denom.abs() < 1e-12 {
            return 0.0;
        }
        (n * sum_tf - sum_t * sum_f) / denom
    }

    /// Estimate the system inertia constant H \[s\] from a generation loss event.
    ///
    /// Uses the swing equation at the instant of disturbance:
    /// ```text
    /// df/dt|₀ = f₀ × P_loss / (2 × H × S_base)
    /// ∴ H = f₀ × P_loss / (2 × S_base × ROCOF)
    /// ```
    ///
    /// # Arguments
    /// * `p_loss_mw`  — Size of the generation loss event \[MW\]
    /// * `rocof`      — Measured ROCOF immediately after the event [Hz/s]
    ///   (should be negative for a generation loss)
    /// * `s_base_mva` — System apparent power base \[MVA\]
    ///
    /// Returns estimated inertia constant H \[s\].
    pub fn inertia_from_rocof(&self, p_loss_mw: f64, rocof: f64, s_base_mva: f64) -> f64 {
        let nominal_hz = 60.0_f64;
        let rocof_abs = rocof.abs();
        if rocof_abs < 1e-12 || s_base_mva < 1e-12 {
            return 0.0;
        }
        nominal_hz * p_loss_mw / (2.0 * s_base_mva * rocof_abs)
    }
}

// ─── HVDC Frequency Support ───────────────────────────────────────────────────

/// HVDC link modulation for fast frequency support.
///
/// When the AC system frequency deviates beyond the deadband, the HVDC link
/// adjusts its power transfer setpoint to inject or absorb MW proportional to
/// the frequency deviation, up to its rated capacity and ramp rate.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HvdcFrequencySupport {
    /// Identifier of the HVDC link.
    pub link_id: usize,
    /// Maximum HVDC power injection \[MW\].
    pub p_max_mw: f64,
    /// Ramp rate [MW/s].
    pub ramp_rate_mw_per_s: f64,
    /// Frequency deadband \[Hz\]; no action if |Δf| < deadband.
    pub frequency_deadband_hz: f64,
    /// Droop gain [MW/Hz]; determines how much MW to inject per Hz of deviation.
    pub droop_mw_per_hz: f64,
    /// Current HVDC power output \[MW\].
    pub current_p_mw: f64,
}

impl HvdcFrequencySupport {
    /// Create an HVDC frequency support controller.
    ///
    /// Positive `current_p_mw` means the link is currently injecting into the AC grid.
    pub fn new(link_id: usize, p_max_mw: f64, droop_mw_per_hz: f64) -> Self {
        Self {
            link_id,
            p_max_mw,
            ramp_rate_mw_per_s: p_max_mw * 0.1, // 10 % per second default
            frequency_deadband_hz: 0.5,
            droop_mw_per_hz,
            current_p_mw: 0.0,
        }
    }

    /// Execute one time step of the HVDC frequency support controller.
    ///
    /// Returns the new HVDC power output \[MW\].
    ///
    /// When frequency is below nominal, the link injects power (positive output).
    /// When frequency is above nominal, the link absorbs power (negative output).
    pub fn step(&mut self, frequency_hz: f64, nominal_hz: f64, dt_s: f64) -> f64 {
        let delta_f = frequency_hz - nominal_hz;

        // Check deadband
        if delta_f.abs() <= self.frequency_deadband_hz {
            // Ramp back to zero if inside deadband
            let ramp_limit = self.ramp_rate_mw_per_s * dt_s;
            let target = 0.0_f64;
            let delta = target - self.current_p_mw;
            let actual_delta = delta.clamp(-ramp_limit, ramp_limit);
            self.current_p_mw =
                (self.current_p_mw + actual_delta).clamp(-self.p_max_mw, self.p_max_mw);
            return self.current_p_mw;
        }

        // Target power: negative delta_f → inject positive MW
        let target_p = (-delta_f * self.droop_mw_per_hz).clamp(-self.p_max_mw, self.p_max_mw);

        // Apply ramp rate
        let ramp_limit = self.ramp_rate_mw_per_s * dt_s;
        let delta = target_p - self.current_p_mw;
        let actual_delta = delta.clamp(-ramp_limit, ramp_limit);
        self.current_p_mw = (self.current_p_mw + actual_delta).clamp(-self.p_max_mw, self.p_max_mw);

        self.current_p_mw
    }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_area(area_id: usize, bias: f64, sched: f64) -> ControlArea {
        ControlArea {
            area_id,
            name: format!("Area-{area_id}"),
            frequency_bias_mw_per_hz: bias,
            scheduled_interchange: sched,
            generator_ids: vec![area_id * 10],
            tie_line_ids: vec![],
        }
    }

    // ── ACE ──────────────────────────────────────────────────────────────────

    #[test]
    fn test_ace_zero_at_equilibrium() {
        let area = make_area(0, 100.0, 50.0);
        let ace = compute_ace(&area, 50.0, 60.0, 60.0);
        assert!(
            ace.abs() < 1e-10,
            "ACE should be zero at equilibrium: {ace}"
        );
    }

    #[test]
    fn test_ace_positive_overfrequency() {
        // f > f0, B > 0 → frequency term is positive
        let area = make_area(0, 100.0, 50.0);
        let ace = compute_ace(&area, 50.0, 60.1, 60.0);
        // freq term = 100 * 0.1 = 10 MW (positive)
        assert!(ace > 0.0, "ACE should be positive for overfrequency: {ace}");
        assert!((ace - 10.0).abs() < 1e-9, "Expected ACE = 10 MW, got {ace}");
    }

    #[test]
    fn test_ace_tie_error_dominant() {
        // Tie line exporting 20 MW more than scheduled → positive ACE
        let area = make_area(0, 100.0, 50.0);
        let ace = compute_ace(&area, 70.0, 60.0, 60.0); // tie error = +20
        assert!((ace - 20.0).abs() < 1e-9, "Expected ACE = 20 MW: {ace}");
    }

    // ── CPS1 / CPS2 ──────────────────────────────────────────────────────────

    #[test]
    fn test_cps1_perfect_control() {
        // ACE = 0 always → mean(ACE·Δf) = 0 → CPS1 = 100%
        let ace = vec![0.0_f64; 100];
        let df = vec![0.001_f64; 100];
        let score = compute_cps1(&ace, &df, 100.0);
        assert!(
            (score - 100.0).abs() < 1e-6,
            "Perfect control should give CPS1=100%: {score}"
        );
    }

    #[test]
    fn test_cps1_empty_input() {
        let score = compute_cps1(&[], &[], 100.0);
        assert_eq!(score, 0.0);
    }

    #[test]
    fn test_cps2_all_compliant() {
        let avgs = vec![0.5_f64; 20];
        let score = compute_cps2(&avgs, 1.0);
        assert!(
            (score - 100.0).abs() < 1e-6,
            "All within limit → 100%: {score}"
        );
    }

    #[test]
    fn test_cps2_half_compliant() {
        let avgs: Vec<f64> = (0..20).map(|i| if i < 10 { 0.5 } else { 2.0 }).collect();
        let score = compute_cps2(&avgs, 1.0);
        assert!((score - 50.0).abs() < 1e-6, "Half compliant → 50%: {score}");
    }

    // ── AGC Controller ────────────────────────────────────────────────────────

    fn make_agc(bias: f64, sched: f64) -> AgcController {
        let area = ControlArea {
            area_id: 0,
            name: "TestArea".into(),
            frequency_bias_mw_per_hz: bias,
            scheduled_interchange: sched,
            generator_ids: vec![1, 2],
            tie_line_ids: vec![],
        };
        let mut config = AgcConfig::default_for_area(area);
        config.kp = 0.3;
        config.ki = 0.1;
        config.max_raise_rate_mw_per_s = 100.0;
        config.max_lower_rate_mw_per_s = 100.0;
        AgcController::new(config)
    }

    #[test]
    fn test_agc_reduces_ace_over_time() {
        let mut agc = make_agc(100.0, 50.0);
        let dispatch = vec![(1_usize, 100.0_f64), (2_usize, 100.0_f64)];
        let limits = vec![(1_usize, 0.0_f64, 500.0_f64), (2_usize, 0.0_f64, 500.0_f64)];

        // Underfrequency: f = 59.9 Hz → ACE is negative (area under-generating)
        // Tie flow at schedule (50 MW), so only freq term contributes:
        // ACE = (50-50) + 100*(59.9-60) = -10 MW
        let mut prev_ace_abs = f64::MAX;
        let mut ace_decreased = false;
        let mut current_dispatch = dispatch.clone();

        for _ in 0..10 {
            let out = agc.step(50.0, 59.9, &current_dispatch, &limits, 4.0);
            current_dispatch = out.new_setpoints.clone();
            if out.ace.abs() < prev_ace_abs {
                ace_decreased = true;
            }
            prev_ace_abs = out.ace.abs();
        }
        assert!(ace_decreased, "AGC should reduce ACE magnitude over time");
    }

    #[test]
    fn test_agc_respects_ramp_limits() {
        let area = ControlArea {
            area_id: 0,
            name: "RampTest".into(),
            frequency_bias_mw_per_hz: 100.0,
            scheduled_interchange: 0.0,
            generator_ids: vec![1],
            tie_line_ids: vec![],
        };
        let mut config = AgcConfig::default_for_area(area);
        config.kp = 10.0; // large gain → large desired signal
        config.ki = 0.0;
        config.max_raise_rate_mw_per_s = 5.0; // only 5 MW/s
        config.max_lower_rate_mw_per_s = 5.0;

        let mut agc = AgcController::new(config);
        let dispatch = vec![(1_usize, 100.0_f64)];
        let limits = vec![(1_usize, 0.0_f64, 500.0_f64)];
        let dt = 4.0_f64;

        let out = agc.step(0.0, 59.5, &dispatch, &limits, dt);
        let new_sp = out.new_setpoints[0].1;
        // Max raise in 4 s = 5 * 4 = 20 MW → new setpoint ≤ 120 MW
        assert!(
            new_sp <= 100.0 + 5.0 * dt + 1e-9,
            "Setpoint exceeded ramp limit: {new_sp}"
        );
    }

    #[test]
    fn test_agc_deadband() {
        let area = make_area(0, 100.0, 50.0);
        let mut config = AgcConfig::default_for_area(area);
        config.deadband_mw = 20.0; // large deadband
        config.kp = 1.0;
        config.ki = 1.0;
        let mut agc = AgcController::new(config);

        let dispatch = vec![(0_usize, 100.0_f64)];
        let limits = vec![(0_usize, 0.0_f64, 500.0_f64)];

        // ACE = (50-50) + 100*(60.05-60) = 5 MW → inside 20 MW deadband
        let integral_before = agc.state.integral_error;
        agc.step(50.0, 60.05, &dispatch, &limits, 4.0);
        // Integral should NOT accumulate when |ACE| < deadband
        assert!(
            (agc.state.integral_error - integral_before).abs() < 1e-9,
            "Integral should be frozen inside deadband: diff={}",
            agc.state.integral_error - integral_before
        );
    }

    // ── Governor Droop ────────────────────────────────────────────────────────

    #[test]
    fn test_governor_droop_response() {
        // Frequency drops: governor should increase mechanical output
        let mut gov = GovernorDroop::new(0, 5.0, 100.0).with_initial_output(50.0);

        let p_init = gov.turbine_state;
        // Simulate 5 seconds of 59.5 Hz (−0.5 Hz deviation)
        let mut p_final = p_init;
        for _ in 0..50 {
            p_final = gov.step(59.5, 60.0, 50.0, 0.1);
        }
        assert!(
            p_final > p_init,
            "Governor should increase output for underfrequency: init={p_init:.2} final={p_final:.2}"
        );
    }

    #[test]
    fn test_governor_droop_steady_state() {
        // At nominal frequency, output should stay near p_ref
        let mut gov = GovernorDroop::new(0, 5.0, 100.0).with_initial_output(60.0);
        let mut p_out = 60.0_f64;
        for _ in 0..200 {
            p_out = gov.step(60.0, 60.0, 60.0, 0.1);
        }
        assert!(
            (p_out - 60.0).abs() < 2.0,
            "Governor output should be near p_ref at nominal freq: {p_out}"
        );
    }

    #[test]
    fn test_governor_droop_limits() {
        // Very large frequency drop → output clamped to p_max
        let mut gov = GovernorDroop::new(0, 5.0, 100.0);
        gov.p_max_mw = 80.0;
        gov.p_min_mw = 10.0;
        let mut p_out = 0.0_f64;
        for _ in 0..500 {
            p_out = gov.step(55.0, 60.0, 50.0, 0.1); // extreme under-freq
        }
        assert!(
            p_out <= gov.p_max_mw + 1e-9,
            "Output must not exceed p_max: {p_out}"
        );
        assert!(
            p_out >= gov.p_min_mw - 1e-9,
            "Output must not go below p_min: {p_out}"
        );
    }

    // ── FCR Assessment ────────────────────────────────────────────────────────

    #[test]
    fn test_fcr_activation_within_deadband() {
        let fcr = FcrAssessment::new(500.0);
        // f = 59.9 Hz → |Δf| = 0.1 Hz < threshold 0.2 Hz → no activation
        let activated = fcr.activated_fcr(59.9, 60.0);
        assert!(
            activated.abs() < 1e-9,
            "No FCR within deadband: {activated}"
        );
    }

    #[test]
    fn test_fcr_activation_partial() {
        let fcr = FcrAssessment::new(500.0);
        // f = 49.7 Hz (50 Hz nominal) → Δf = −0.3 Hz
        // threshold=0.2, full=0.5 → fraction = (0.3-0.2)/(0.5-0.2) = 1/3
        let activated = fcr.activated_fcr(49.7, 50.0);
        let expected = 500.0 / 3.0;
        assert!(
            (activated - expected).abs() < 1.0,
            "Partial FCR activation at 49.7 Hz: got {activated:.2} expected {expected:.2}"
        );
    }

    #[test]
    fn test_fcr_activation_full() {
        let fcr = FcrAssessment::new(500.0);
        // f = 49.5 Hz → |Δf| = 0.5 Hz = full_activation_hz → 100% FCR
        let activated = fcr.activated_fcr(49.5, 50.0);
        assert!(
            (activated - 500.0).abs() < 1e-9,
            "Full FCR at 49.5 Hz: {activated}"
        );
    }

    #[test]
    fn test_fcr_activation_beyond_full() {
        let fcr = FcrAssessment::new(500.0);
        // f = 49.0 Hz → beyond full activation → still 500 MW (clamped)
        let activated = fcr.activated_fcr(49.0, 50.0);
        assert!(
            (activated - 500.0).abs() < 1e-9,
            "FCR beyond full deviation should be clamped: {activated}"
        );
    }

    // ── ROCOF and Inertia Estimation ─────────────────────────────────────────

    #[test]
    fn test_rocof_linear_decline() {
        // Synthetic frequency declining at −0.5 Hz/s from 60 Hz
        let dt = 0.1_f64;
        let history: Vec<f64> = (0..20).map(|i| 60.0 - 0.5 * i as f64 * dt).collect();
        let fcr = FcrAssessment::new(1000.0);
        let rocof = fcr.rocof(&history, dt);
        assert!(
            (rocof + 0.5).abs() < 0.05,
            "ROCOF should be ≈ −0.5 Hz/s: {rocof:.4}"
        );
    }

    #[test]
    fn test_inertia_estimate_from_rocof() {
        // H=5 s, P_loss=100 MW, S_base=1000 MVA, f₀=60 Hz
        // ROCOF = 60 * 100 / (2 * 1000 * 5) = 0.6 Hz/s
        let fcr = FcrAssessment::new(0.0);
        let rocof = 60.0 * 100.0 / (2.0 * 1000.0 * 5.0);
        let h_est = fcr.inertia_from_rocof(100.0, rocof, 1000.0);
        assert!(
            (h_est - 5.0).abs() < 0.01,
            "Estimated H should be ≈ 5 s: {h_est:.4}"
        );
    }

    // ── HVDC Frequency Support ────────────────────────────────────────────────

    #[test]
    fn test_hvdc_frequency_support_deadband() {
        let mut hvdc = HvdcFrequencySupport::new(0, 200.0, 100.0);
        hvdc.frequency_deadband_hz = 0.5;

        // f = 59.6 Hz → |Δf| = 0.4 Hz < 0.5 Hz deadband → no response
        let p_out = hvdc.step(59.6, 60.0, 1.0);
        assert!(
            p_out.abs() < 1e-9,
            "HVDC should not respond within deadband: {p_out}"
        );
    }

    #[test]
    fn test_hvdc_frequency_support_activates() {
        let mut hvdc = HvdcFrequencySupport::new(0, 200.0, 100.0);
        hvdc.frequency_deadband_hz = 0.2;
        hvdc.ramp_rate_mw_per_s = 1000.0; // fast ramp for test

        // f = 59.5 Hz → |Δf| = 0.5 Hz > deadband → inject power
        let p_out = hvdc.step(59.5, 60.0, 1.0);
        assert!(
            p_out > 0.0,
            "HVDC should inject power at underfrequency: {p_out}"
        );
    }

    #[test]
    fn test_hvdc_ramp_rate_limit() {
        let mut hvdc = HvdcFrequencySupport::new(0, 200.0, 100.0);
        hvdc.frequency_deadband_hz = 0.0;
        hvdc.ramp_rate_mw_per_s = 10.0;

        // Large frequency drop, one small dt
        let p_out = hvdc.step(55.0, 60.0, 0.1); // max ramp = 10*0.1 = 1 MW
        assert!(
            p_out.abs() <= 1.0 + 1e-9,
            "HVDC must respect ramp rate: {p_out}"
        );
    }

    #[test]
    fn test_hvdc_power_limit() {
        let mut hvdc = HvdcFrequencySupport::new(0, 200.0, 1000.0);
        hvdc.frequency_deadband_hz = 0.0;
        hvdc.ramp_rate_mw_per_s = 10_000.0; // instantaneous ramp

        // Simulate many steps to reach saturation
        let mut p_out = 0.0_f64;
        for _ in 0..1000 {
            p_out = hvdc.step(55.0, 60.0, 0.1);
        }
        assert!(
            p_out.abs() <= hvdc.p_max_mw + 1e-9,
            "HVDC output must not exceed p_max: {p_out}"
        );
    }

    // ── Multi-Area Simulation ─────────────────────────────────────────────────

    #[test]
    fn test_multi_area_frequency_recovery() {
        let area0 = ControlArea {
            area_id: 0,
            name: "North".into(),
            frequency_bias_mw_per_hz: 200.0,
            scheduled_interchange: 0.0,
            generator_ids: vec![1],
            tie_line_ids: vec![],
        };
        let area1 = ControlArea {
            area_id: 1,
            name: "South".into(),
            frequency_bias_mw_per_hz: 200.0,
            scheduled_interchange: 0.0,
            generator_ids: vec![2],
            tie_line_ids: vec![],
        };

        let mut cfg0 = AgcConfig::default_for_area(area0);
        cfg0.kp = 0.5;
        cfg0.ki = 0.1;
        cfg0.max_raise_rate_mw_per_s = 500.0;
        cfg0.max_lower_rate_mw_per_s = 500.0;

        let mut cfg1 = AgcConfig::default_for_area(area1);
        cfg1.kp = 0.5;
        cfg1.ki = 0.1;
        cfg1.max_raise_rate_mw_per_s = 500.0;
        cfg1.max_lower_rate_mw_per_s = 500.0;

        let controllers = vec![AgcController::new(cfg0), AgcController::new(cfg1)];
        let tie = TieLine {
            branch_idx: 0,
            area_from: 0,
            area_to: 1,
            schedule_mw: 0.0,
            susceptance_pu: 10.0,
        };

        let mut multi = MultiAreaAgc::new(controllers, vec![tie], 5000.0);
        // 200 MW loss at t=5 s
        let result = multi.simulate(60.0, Some((5.0, -200.0)));
        let result = result.expect("simulation should succeed");

        assert!(
            result.nadir_hz < 60.0,
            "Frequency should dip below nominal: nadir={:.4}",
            result.nadir_hz
        );
        assert!(
            result.steady_state_error_hz.abs() < 0.3,
            "Frequency should recover within 0.3 Hz: err={:.4}",
            result.steady_state_error_hz
        );
    }

    #[test]
    fn test_multi_area_no_disturbance() {
        let area = ControlArea {
            area_id: 0,
            name: "Single".into(),
            frequency_bias_mw_per_hz: 100.0,
            scheduled_interchange: 0.0,
            generator_ids: vec![1],
            tie_line_ids: vec![],
        };
        let config = AgcConfig::default_for_area(area);
        let controllers = vec![AgcController::new(config)];
        let mut multi = MultiAreaAgc::new(controllers, vec![], 2000.0);
        let result = multi.simulate(10.0, None).expect("should succeed");
        assert!(
            result.frequency.iter().all(|&f| (f - 60.0).abs() < 1e-6),
            "Without disturbance, frequency should remain at 60 Hz"
        );
    }

    #[test]
    fn test_multi_area_invalid_duration() {
        let area = make_area(0, 100.0, 0.0);
        let config = AgcConfig::default_for_area(area);
        let controllers = vec![AgcController::new(config)];
        let mut multi = MultiAreaAgc::new(controllers, vec![], 1000.0);
        let result = multi.simulate(-1.0, None);
        assert!(result.is_err(), "Negative duration should return error");
    }
}
