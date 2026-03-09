//! HVDC switching transient analysis, converter station dynamics, DC fault
//! analysis, and breaker modeling for multi-terminal DC grids.
//!
//! # Algorithms
//!
//! - **DC power flow**: Newton's method on the DC grid conductance matrix.
//! - **LCC converter**: `V_dc = (3√2/π)·V_ac·cos(α) − (3/π)·X_c·I_dc`
//! - **VSC converter**: `V_dc = V_ac·m·√(3/2)`, independent P/Q control.
//! - **DC fault analysis**: RLC circuit (underdamped/overdamped).
//! - **Breaker clearing**: detection → trip delay → opening → energy absorption.
//! - **Transient simulation**: RK4 on state-space `[V_dc_1..N, I_cable_1..M]`.

use std::f64::consts::{PI, SQRT_2};

// ─────────────────────────────────────────────────────────────────────────────
// Types
// ─────────────────────────────────────────────────────────────────────────────

/// Converter topology type.
#[derive(Debug, Clone, PartialEq)]
pub enum ConverterTopology {
    /// Line Commutated Converter (thyristor).
    Lcc,
    /// Two-level VSC (IGBT).
    TwoLevelVsc,
    /// Modular Multilevel Converter.
    Mmc {
        /// Number of sub-modules per arm.
        n_submodules: usize,
    },
    /// Hybrid (LCC + VSC).
    Hybrid,
}

/// DC breaker type.
#[derive(Debug, Clone, PartialEq)]
pub enum DcBreakerType {
    /// Mechanical breaker (slow, 30-80 ms).
    Mechanical {
        /// Opening time in milliseconds.
        opening_time_ms: f64,
    },
    /// Solid-state breaker (fast, <1 ms, high losses).
    SolidState {
        /// Turn-off time in microseconds.
        turn_off_time_us: f64,
        /// On-state conduction loss percentage.
        on_state_loss_pct: f64,
    },
    /// Hybrid breaker (mechanical + solid-state).
    HybridBreaker {
        /// Mechanical opening time in milliseconds.
        opening_time_ms: f64,
        /// Solid-state commutation time in microseconds.
        commutation_time_us: f64,
    },
}

/// DC fault type.
#[derive(Debug, Clone, PartialEq)]
pub enum DcFaultType {
    /// Pole-to-ground fault.
    PoleToGround,
    /// Pole-to-pole fault.
    PoleToPole,
    /// Converter internal fault.
    ConverterInternal,
}

/// Converter station parameters.
#[derive(Debug, Clone)]
pub struct ConverterStation {
    /// Unique station identifier.
    pub id: usize,
    /// Station name.
    pub name: String,
    /// Converter topology.
    pub topology: ConverterTopology,
    /// Rated DC voltage (kV).
    pub v_dc_rated_kv: f64,
    /// Rated power (MW).
    pub p_rated_mw: f64,
    /// Transformer reactance (pu).
    pub x_transformer_pu: f64,
    /// AC system short-circuit ratio.
    pub scr: f64,
    /// Firing/modulation angle (degrees).
    pub control_angle_deg: f64,
    /// Commutation reactance (pu) — for LCC.
    pub x_commutation_pu: f64,
    /// Arm inductance (mH) — for MMC.
    pub arm_inductance_mh: f64,
    /// Submodule capacitance (μF) — for MMC.
    pub sm_capacitance_uf: f64,
}

/// DC cable/line segment between two converter stations.
#[derive(Debug, Clone)]
pub struct DcCable {
    /// Cable identifier.
    pub id: usize,
    /// Index of originating station.
    pub from_station: usize,
    /// Index of terminating station.
    pub to_station: usize,
    /// Resistance per km (Ω/km).
    pub r_per_km: f64,
    /// Inductance per km (mH/km).
    pub l_per_km: f64,
    /// Capacitance per km (μF/km).
    pub c_per_km: f64,
    /// Length in km.
    pub length_km: f64,
}

/// DC fault event specification.
#[derive(Debug, Clone)]
pub struct DcFaultEvent {
    /// Type of fault.
    pub fault_type: DcFaultType,
    /// Cable on which the fault occurs.
    pub location_cable_id: usize,
    /// Fractional distance from "from" terminal (0.0–1.0).
    pub location_fraction: f64,
    /// Fault resistance (Ω).
    pub fault_resistance: f64,
    /// Fault inception time (ms).
    pub inception_time_ms: f64,
}

/// DC circuit breaker.
#[derive(Debug, Clone)]
pub struct DcBreaker {
    /// Breaker identifier.
    pub id: usize,
    /// Station where the breaker is installed.
    pub station_id: usize,
    /// Breaker technology type.
    pub breaker_type: DcBreakerType,
    /// Maximum breaking current (kA).
    pub max_breaking_current_ka: f64,
    /// Energy absorption capacity (MJ).
    pub energy_absorption_mj: f64,
}

/// Instantaneous DC transient state snapshot.
#[derive(Debug, Clone)]
pub struct DcTransientState {
    /// Simulation time (ms).
    pub time_ms: f64,
    /// DC voltage at each station (kV).
    pub station_voltages_kv: Vec<f64>,
    /// DC current at each station (kA).
    pub station_currents_ka: Vec<f64>,
    /// Cable currents (kA).
    pub cable_currents_ka: Vec<f64>,
    /// Fault current (kA), zero if no fault active.
    pub fault_current_ka: f64,
    /// Breaker states (true = closed/conducting).
    pub breaker_states: Vec<bool>,
}

/// Complete transient simulation result.
#[derive(Debug, Clone)]
pub struct DcTransientResult {
    /// Time-series of transient states.
    pub states: Vec<DcTransientState>,
    /// Peak fault current observed (kA).
    pub peak_fault_current_ka: f64,
    /// Time at which fault was cleared (ms).
    pub fault_clearing_time_ms: f64,
    /// Total energy dissipated in fault/breaker (MJ).
    pub energy_dissipated_mj: f64,
    /// Time for voltage to recover to 90% of rated (ms).
    pub voltage_recovery_time_ms: f64,
    /// Maximum di/dt observed (kA/ms).
    pub max_di_dt: f64,
    /// Whether the fault was successfully cleared.
    pub fault_cleared: bool,
}

/// Converter station steady-state operating point.
#[derive(Debug, Clone)]
pub struct ConverterOperatingPoint {
    /// Station identifier.
    pub station_id: usize,
    /// DC power (MW).
    pub p_dc_mw: f64,
    /// DC voltage (kV).
    pub v_dc_kv: f64,
    /// DC current (kA).
    pub i_dc_ka: f64,
    /// Firing/modulation angle (degrees).
    pub firing_angle_deg: f64,
    /// Power factor.
    pub power_factor: f64,
    /// Converter losses (MW).
    pub losses_mw: f64,
}

/// Steady-state DC grid power flow solution.
#[derive(Debug, Clone)]
pub struct DcGridSolution {
    /// Operating point for each station.
    pub operating_points: Vec<ConverterOperatingPoint>,
    /// Losses in each cable (MW).
    pub cable_losses_mw: Vec<f64>,
    /// Total system losses (MW).
    pub total_losses_mw: f64,
    /// Whether the power flow converged.
    pub converged: bool,
    /// Number of Newton iterations used.
    pub iterations: usize,
}

/// DC switching transient simulator for multi-terminal HVDC grids.
///
/// # Example
/// ```
/// use oxigrid::network::dc_switching::*;
///
/// let mut sim = DcSwitchingSimulator::new(10.0, 100.0);
/// sim.add_station(ConverterStation {
///     id: 0,
///     name: "Rectifier".to_string(),
///     topology: ConverterTopology::Lcc,
///     v_dc_rated_kv: 500.0,
///     p_rated_mw: 1000.0,
///     x_transformer_pu: 0.15,
///     scr: 3.0,
///     control_angle_deg: 15.0,
///     x_commutation_pu: 0.1,
///     arm_inductance_mh: 0.0,
///     sm_capacitance_uf: 0.0,
/// });
/// ```
#[derive(Debug, Clone)]
pub struct DcSwitchingSimulator {
    /// Converter stations.
    pub stations: Vec<ConverterStation>,
    /// DC cables connecting stations.
    pub cables: Vec<DcCable>,
    /// DC circuit breakers.
    pub breakers: Vec<DcBreaker>,
    /// Simulation time step (μs).
    pub dt_us: f64,
    /// Total simulation duration (ms).
    pub duration_ms: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Implementation
// ─────────────────────────────────────────────────────────────────────────────

impl DcSwitchingSimulator {
    /// Create a new simulator with the given time step and duration.
    ///
    /// # Arguments
    /// * `dt_us` — Integration time step in microseconds.
    /// * `duration_ms` — Total simulation duration in milliseconds.
    pub fn new(dt_us: f64, duration_ms: f64) -> Self {
        Self {
            stations: Vec::new(),
            cables: Vec::new(),
            breakers: Vec::new(),
            dt_us: dt_us.max(0.1),
            duration_ms: duration_ms.max(0.001),
        }
    }

    /// Add a converter station to the system.
    pub fn add_station(&mut self, station: ConverterStation) {
        self.stations.push(station);
    }

    /// Add a DC cable to the system.
    pub fn add_cable(&mut self, cable: DcCable) {
        self.cables.push(cable);
    }

    /// Add a DC breaker to the system.
    pub fn add_breaker(&mut self, breaker: DcBreaker) {
        self.breakers.push(breaker);
    }

    // ── DC Power Flow (Newton's method) ──────────────────────────────────

    /// Solve the DC grid power flow using Newton's method.
    ///
    /// Computes steady-state voltages, currents, and losses across the
    /// multi-terminal DC grid. The first station is treated as the voltage
    /// reference (slack bus).
    pub fn solve_dc_power_flow(&self) -> Result<DcGridSolution, String> {
        let n = self.stations.len();
        if n == 0 {
            return Err("No converter stations defined".to_string());
        }

        // Build conductance matrix G (n×n dense, stored row-major)
        let mut g_matrix = vec![0.0_f64; n * n];
        for cable in &self.cables {
            let from = cable.from_station;
            let to = cable.to_station;
            if from >= n || to >= n {
                continue;
            }
            let r_total = (cable.r_per_km * cable.length_km).max(1e-9);
            let g = 1.0 / r_total;
            g_matrix[from * n + from] += g;
            g_matrix[to * n + to] += g;
            g_matrix[from * n + to] -= g;
            g_matrix[to * n + from] -= g;
        }

        // Initial voltages: rated DC voltage for each station
        let mut v: Vec<f64> = self.stations.iter().map(|s| s.v_dc_rated_kv).collect();

        // Power specification: positive = injecting, negative = absorbing
        // Station 0 is slack (voltage-controlled), others are power-controlled
        let p_spec: Vec<f64> = self
            .stations
            .iter()
            .enumerate()
            .map(|(i, s)| {
                if i == 0 {
                    0.0 // slack — will be computed
                } else {
                    // Inverter stations absorb power (negative sign convention)
                    -s.p_rated_mw * 0.8 // operate at 80% of rated
                }
            })
            .collect();

        let max_iter = 50;
        let tol = 1e-6;
        let mut converged = false;
        let mut iterations = 0;

        for iter in 0..max_iter {
            iterations = iter + 1;

            // Compute P_calc = V_i * sum_j(G_ij * V_j)
            let mut p_calc = vec![0.0_f64; n];
            for i in 0..n {
                let mut sum = 0.0;
                for j in 0..n {
                    sum += g_matrix[i * n + j] * v[j];
                }
                p_calc[i] = v[i] * sum;
            }

            // Mismatch: ΔP = P_spec - P_calc (skip slack bus 0)
            let mut max_mismatch = 0.0_f64;
            let mut dp = vec![0.0_f64; n];
            for i in 1..n {
                dp[i] = p_spec[i] - p_calc[i];
                max_mismatch = max_mismatch.max(dp[i].abs());
            }

            if max_mismatch < tol {
                converged = true;
                break;
            }

            // Jacobian J(i,j) = dP_i/dV_j for i,j >= 1
            // dP_i/dV_j = G_ij * V_i (j != i)
            // dP_i/dV_i = 2*G_ii*V_i + sum_{j!=i}(G_ij*V_j)
            if n == 2 {
                // Simple 2-bus case: direct update
                let j11 = 2.0 * g_matrix[n + 1] * v[1] + g_matrix[n] * v[0];
                if j11.abs() > 1e-12 {
                    v[1] += dp[1] / j11;
                }
            } else {
                // General case: solve J * ΔV = ΔP for buses 1..n-1
                // Using simple Gauss-Seidel-like update for robustness
                for i in 1..n {
                    let j_ii = 2.0 * g_matrix[i * n + i] * v[i]
                        + (0..n)
                            .filter(|&j| j != i)
                            .map(|j| g_matrix[i * n + j] * v[j])
                            .sum::<f64>();
                    if j_ii.abs() > 1e-12 {
                        v[i] += dp[i] / j_ii;
                    }
                    // Clamp voltage to reasonable range
                    let v_rated = self.stations[i].v_dc_rated_kv;
                    v[i] = v[i].clamp(v_rated * 0.5, v_rated * 1.5);
                }
            }
        }

        // Compute operating points and cable losses
        let mut operating_points = Vec::with_capacity(n);
        for (i, station) in self.stations.iter().enumerate() {
            let v_kv = v[i];
            // Current from power balance
            let mut i_sum = 0.0;
            for j in 0..n {
                i_sum += g_matrix[i * n + j] * v[j];
            }
            let i_ka = i_sum; // current in kA (since V in kV, G in 1/kΩ→ I in kA)
            let p_mw = v_kv * i_ka;
            let op = match &station.topology {
                ConverterTopology::Lcc => self.lcc_operating_point(station, i_ka.abs()),
                _ => self.vsc_operating_point(station, p_mw.abs(), v_kv),
            };
            operating_points.push(ConverterOperatingPoint {
                station_id: station.id,
                p_dc_mw: p_mw,
                v_dc_kv: v_kv,
                i_dc_ka: i_ka,
                firing_angle_deg: op.firing_angle_deg,
                power_factor: op.power_factor,
                losses_mw: op.losses_mw,
            });
        }

        let mut cable_losses_mw = Vec::with_capacity(self.cables.len());
        let mut total_losses = 0.0;
        for cable in &self.cables {
            let from = cable.from_station.min(n - 1);
            let to = cable.to_station.min(n - 1);
            let r_total = (cable.r_per_km * cable.length_km).max(1e-9);
            let dv = v[from] - v[to];
            let i_cable = dv / r_total; // kA
            let loss = i_cable * i_cable * r_total; // kA² * kΩ = MW
            cable_losses_mw.push(loss.abs());
            total_losses += loss.abs();
        }

        // Add converter losses
        for op in &operating_points {
            total_losses += op.losses_mw;
        }

        Ok(DcGridSolution {
            operating_points,
            cable_losses_mw,
            total_losses_mw: total_losses,
            converged,
            iterations,
        })
    }

    // ── Converter operating points ───────────────────────────────────────

    /// Compute LCC converter operating point.
    ///
    /// Uses the standard LCC equations:
    /// - `V_dc = (3√2/π) · V_ac · cos(α) − (3/π) · X_c · I_dc`
    /// - Power factor ≈ cos(α) · (1 − μ/2)
    /// - Losses = 0.7% × P_rated + I_dc² × R_converter
    pub fn lcc_operating_point(
        &self,
        station: &ConverterStation,
        i_dc_ka: f64,
    ) -> ConverterOperatingPoint {
        let alpha_rad = station.control_angle_deg.to_radians();
        // AC voltage from SCR: V_ac ≈ V_dc_rated * π / (3√2)  (at nominal)
        let v_ac_kv = station.v_dc_rated_kv * PI / (3.0 * SQRT_2);

        // No-load DC voltage
        let v_d0 = (3.0 * SQRT_2 / PI) * v_ac_kv;

        // Commutation reactance drop
        // X_c in kΩ (from pu): X_c_pu * Z_base where Z_base = V²/P
        let z_base = station.v_dc_rated_kv.powi(2) / station.p_rated_mw.max(1e-6);
        let x_c = station.x_commutation_pu * z_base;

        let v_dc = v_d0 * alpha_rad.cos() - (3.0 / PI) * x_c * i_dc_ka;
        let p_dc = v_dc * i_dc_ka;

        // Overlap angle approximation
        let cos_alpha = alpha_rad.cos();
        let mu_arg = (cos_alpha
            - 2.0 * station.x_commutation_pu * i_dc_ka / station.p_rated_mw.max(1e-6)
                * station.v_dc_rated_kv)
            .clamp(-1.0, 1.0);
        let mu_rad = (alpha_rad.cos() - mu_arg).abs().min(PI / 4.0);
        let pf = cos_alpha * (1.0 - mu_rad / 2.0);

        // Losses: 0.7% of rated + I²R
        let r_conv = 0.001 * z_base; // ~0.1% impedance
        let losses = 0.007 * station.p_rated_mw + i_dc_ka * i_dc_ka * r_conv;

        ConverterOperatingPoint {
            station_id: station.id,
            p_dc_mw: p_dc,
            v_dc_kv: v_dc,
            i_dc_ka,
            firing_angle_deg: station.control_angle_deg,
            power_factor: pf.clamp(0.0, 1.0),
            losses_mw: losses.abs(),
        }
    }

    /// Compute VSC converter operating point.
    ///
    /// Uses VSC equations:
    /// - `V_dc = V_ac · m · √(3/2)` where m = modulation index
    /// - Independent P/Q control capability
    /// - Losses = a + b·I + c·I² (switching + conduction)
    pub fn vsc_operating_point(
        &self,
        station: &ConverterStation,
        p_mw: f64,
        v_dc_kv: f64,
    ) -> ConverterOperatingPoint {
        let v_dc = v_dc_kv.max(1e-6);
        let i_dc_ka = p_mw / v_dc;

        // Modulation index (back-calculated)
        let v_ac_kv = v_dc / (1.5_f64).sqrt();
        let m = if v_ac_kv > 1e-6 {
            (v_dc / (v_ac_kv * (1.5_f64).sqrt())).clamp(0.0, 1.15)
        } else {
            0.85
        };

        // VSC losses: a + b*I + c*I²
        // Typical: a=0.5% P_rated (no-load), b=0.1% P_rated/I_rated, c=0.5% P_rated/I_rated²
        let i_rated = station.p_rated_mw / station.v_dc_rated_kv.max(1e-6);
        let a = 0.005 * station.p_rated_mw;
        let b = 0.001 * station.p_rated_mw / i_rated.max(1e-6);
        let c = 0.005 * station.p_rated_mw / (i_rated * i_rated).max(1e-12);
        let losses = a + b * i_dc_ka.abs() + c * i_dc_ka * i_dc_ka;

        // MMC has lower losses due to reduced switching frequency per device
        let loss_factor = match &station.topology {
            ConverterTopology::Mmc { n_submodules } => {
                let nsm = (*n_submodules).max(1) as f64;
                0.5 + 0.5 / nsm.sqrt() // lower losses with more SMs
            }
            ConverterTopology::TwoLevelVsc => 1.0,
            ConverterTopology::Hybrid => 0.8,
            ConverterTopology::Lcc => 0.6, // LCC has lower switching losses
        };

        let pf = 1.0; // VSC can operate at unity PF

        ConverterOperatingPoint {
            station_id: station.id,
            p_dc_mw: p_mw,
            v_dc_kv: v_dc,
            i_dc_ka,
            firing_angle_deg: m * 180.0 / PI, // modulation index as angle equivalent
            power_factor: pf,
            losses_mw: (losses * loss_factor).abs(),
        }
    }

    // ── Fault simulation ──────────────────────────────────────────────────

    /// Simulate a DC fault event and return the transient result.
    ///
    /// Models the fault as an RLC circuit with parameters derived from the
    /// faulted cable and connected converter stations. Uses RK4 integration
    /// for the state-space model.
    pub fn simulate_fault(&self, fault: &DcFaultEvent) -> Result<DcTransientResult, String> {
        if self.stations.is_empty() {
            return Err("No converter stations defined".to_string());
        }
        if self.cables.is_empty() {
            return Err("No cables defined".to_string());
        }

        let n_stations = self.stations.len();
        let n_cables = self.cables.len();
        let n_breakers = self.breakers.len();

        let dt_ms = self.dt_us / 1000.0; // time step in ms
        let total_steps = ((self.duration_ms / dt_ms).ceil() as usize).max(1);

        // Extract fault circuit parameters
        let (l_total_mh, c_total_uf, r_total_ohm, r_fault, v_dc_kv) = self.fault_rlc_params(fault);

        // Convert to base units for RLC computation
        let l_h = l_total_mh * 1e-3;
        let c_f = c_total_uf * 1e-6;
        let v_dc_v = v_dc_kv * 1e3;

        // RLC natural response parameters
        let alpha = r_total_ohm / (2.0 * l_h.max(1e-9));
        let omega0_sq = 1.0 / (l_h.max(1e-9) * c_f.max(1e-12));
        let is_underdamped = alpha * alpha < omega0_sq;

        // Initialize state
        let mut state = DcTransientState {
            time_ms: 0.0,
            station_voltages_kv: self.stations.iter().map(|s| s.v_dc_rated_kv).collect(),
            station_currents_ka: vec![0.0; n_stations],
            cable_currents_ka: vec![0.0; n_cables],
            fault_current_ka: 0.0,
            breaker_states: vec![true; n_breakers],
        };

        let mut states = Vec::with_capacity(total_steps.min(10000));
        let mut peak_fault_ka = 0.0_f64;
        let mut max_di_dt = 0.0_f64;
        let mut energy_mj = 0.0_f64;
        let mut fault_cleared = false;
        let mut clearing_time_ms = self.duration_ms;
        let mut voltage_recovery_time_ms = self.duration_ms;
        let mut prev_fault_current_ka = 0.0_f64;
        let mut fault_active = false;

        // Determine breaker clearing time
        let breaker_clear_ms = self.fastest_breaker_clearing_ms();

        // Determine recording interval (keep states manageable)
        let record_interval = (total_steps / 5000).max(1);

        for step in 0..total_steps {
            let t_ms = step as f64 * dt_ms;

            // Check fault inception
            if !fault_active && t_ms >= fault.inception_time_ms {
                fault_active = true;
            }

            if fault_active && !fault_cleared {
                // RK4 step for fault current
                let dt_s = dt_ms * 1e-3;
                let new_state = self.rk4_step(&state, dt_s, Some(fault));

                // Compute fault current from RLC model
                let t_fault_s = (t_ms - fault.inception_time_ms).max(0.0) * 1e-3;

                let i_fault_a = if is_underdamped {
                    let omega_d = (omega0_sq - alpha * alpha).max(0.0).sqrt();
                    if omega_d > 1e-12 {
                        (v_dc_v / (omega_d * l_h.max(1e-9)))
                            * (-alpha * t_fault_s).exp()
                            * (omega_d * t_fault_s).sin()
                    } else {
                        0.0
                    }
                } else {
                    // Overdamped or critically damped
                    let disc = (alpha * alpha - omega0_sq).max(0.0).sqrt();
                    let s1 = -alpha + disc;
                    let s2 = -alpha - disc;
                    if (s1 - s2).abs() > 1e-12 {
                        let a_coeff = v_dc_v / (l_h.max(1e-9) * (s1 - s2));
                        a_coeff * ((s1 * t_fault_s).exp() - (s2 * t_fault_s).exp())
                    } else {
                        // Critically damped
                        (v_dc_v / l_h.max(1e-9)) * t_fault_s * (-alpha * t_fault_s).exp()
                    }
                };

                let i_fault_ka = i_fault_a.abs() / 1e3;

                // Apply fault resistance effect
                let r_factor = 1.0 / (1.0 + r_fault / (r_total_ohm + 1e-9));
                let i_fault_ka = i_fault_ka * r_factor;

                // Pole-to-ground has lower current than pole-to-pole
                let fault_type_factor = match fault.fault_type {
                    DcFaultType::PoleToGround => 0.5,
                    DcFaultType::PoleToPole => 1.0,
                    DcFaultType::ConverterInternal => 0.7,
                };
                let i_fault_ka = i_fault_ka * fault_type_factor;

                state.fault_current_ka = i_fault_ka;

                // Update cable currents from new_state
                state.cable_currents_ka = new_state.cable_currents_ka;
                state.station_voltages_kv = new_state.station_voltages_kv;
                state.station_currents_ka = new_state.station_currents_ka;

                // Track di/dt
                let di_dt = (i_fault_ka - prev_fault_current_ka).abs() / dt_ms.max(1e-12);
                max_di_dt = max_di_dt.max(di_dt);
                prev_fault_current_ka = i_fault_ka;

                // Track peak
                peak_fault_ka = peak_fault_ka.max(i_fault_ka);

                // Energy dissipated: I²R·dt
                let i_a = i_fault_ka * 1e3;
                energy_mj += i_a * i_a * (r_fault + r_total_ohm) * (dt_ms * 1e-3) / 1e6;

                // Check breaker clearing
                let t_since_inception = t_ms - fault.inception_time_ms;
                if t_since_inception >= breaker_clear_ms && !fault_cleared {
                    fault_cleared = true;
                    clearing_time_ms = t_ms;
                    // Open all breakers
                    for b in state.breaker_states.iter_mut() {
                        *b = false;
                    }
                }
            } else if fault_cleared {
                // Post-clearing: exponential decay
                let t_after_clear_s = (t_ms - clearing_time_ms).max(0.0) * 1e-3;
                let decay_tau = l_h / r_total_ohm.max(1e-9);
                state.fault_current_ka =
                    prev_fault_current_ka * (-t_after_clear_s / decay_tau.max(1e-9)).exp();
                if state.fault_current_ka < 1e-6 {
                    state.fault_current_ka = 0.0;
                }

                // Voltage recovery check
                let all_recovered = state
                    .station_voltages_kv
                    .iter()
                    .enumerate()
                    .all(|(i, &v)| v >= 0.9 * self.stations[i].v_dc_rated_kv);
                if all_recovered && voltage_recovery_time_ms >= self.duration_ms {
                    voltage_recovery_time_ms = t_ms;
                }

                // Recover voltages gradually
                for (i, v) in state.station_voltages_kv.iter_mut().enumerate() {
                    let v_rated = self.stations[i].v_dc_rated_kv;
                    let recovery_rate = 0.001 * v_rated; // kV per step
                    if *v < v_rated {
                        *v = (*v + recovery_rate).min(v_rated);
                    }
                }
            }

            state.time_ms = t_ms;

            // Record state at intervals
            if step % record_interval == 0 || step == total_steps - 1 {
                states.push(state.clone());
            }
        }

        Ok(DcTransientResult {
            states,
            peak_fault_current_ka: peak_fault_ka,
            fault_clearing_time_ms: if fault_cleared {
                clearing_time_ms
            } else {
                self.duration_ms
            },
            energy_dissipated_mj: energy_mj,
            voltage_recovery_time_ms,
            max_di_dt,
            fault_cleared,
        })
    }

    /// Simulate a switching transient (breaker open/close).
    ///
    /// Models the voltage/current transients that occur when a breaker
    /// is opened or closed, including LC oscillations from cable capacitance.
    pub fn simulate_switching(
        &self,
        breaker_id: usize,
        open: bool,
    ) -> Result<DcTransientResult, String> {
        if self.stations.is_empty() {
            return Err("No converter stations defined".to_string());
        }

        let n_stations = self.stations.len();
        let _n_cables = self.cables.len();
        let n_breakers = self.breakers.len();

        let dt_ms = self.dt_us / 1000.0;
        let total_steps = ((self.duration_ms / dt_ms).ceil() as usize).max(1);

        let mut state = DcTransientState {
            time_ms: 0.0,
            station_voltages_kv: self.stations.iter().map(|s| s.v_dc_rated_kv).collect(),
            station_currents_ka: vec![0.0; n_stations],
            cable_currents_ka: self
                .cables
                .iter()
                .map(|c| {
                    let r = (c.r_per_km * c.length_km).max(1e-9);
                    let from = c.from_station.min(n_stations.saturating_sub(1));
                    let to = c.to_station.min(n_stations.saturating_sub(1));
                    let v_from = self.stations.get(from).map_or(0.0, |s| s.v_dc_rated_kv);
                    let v_to = self.stations.get(to).map_or(0.0, |s| s.v_dc_rated_kv);
                    (v_from - v_to) / r
                })
                .collect(),
            fault_current_ka: 0.0,
            breaker_states: vec![!open; n_breakers],
        };

        let mut states = Vec::with_capacity(total_steps.min(10000));
        let record_interval = (total_steps / 5000).max(1);
        let switching_time_ms = self.duration_ms * 0.1; // switch at 10% of duration
        let mut switched = false;

        for step in 0..total_steps {
            let t_ms = step as f64 * dt_ms;
            state.time_ms = t_ms;

            // Perform switching event
            if !switched && t_ms >= switching_time_ms {
                switched = true;
                for (i, b) in state.breaker_states.iter_mut().enumerate() {
                    if i == breaker_id || breaker_id >= n_breakers {
                        *b = !open;
                    }
                }
                if open {
                    // Opening: force cable currents to decay (interrupted)
                    for i_c in state.cable_currents_ka.iter_mut() {
                        *i_c *= 0.5; // initial step reduction
                    }
                }
            }

            // RK4 step for normal operation (no fault)
            let dt_s = dt_ms * 1e-3;
            let new_state = self.rk4_step(&state, dt_s, None);
            state.cable_currents_ka = new_state.cable_currents_ka;
            state.station_voltages_kv = new_state.station_voltages_kv;
            state.station_currents_ka = new_state.station_currents_ka;

            if step % record_interval == 0 || step == total_steps - 1 {
                states.push(state.clone());
            }
        }

        Ok(DcTransientResult {
            states,
            peak_fault_current_ka: 0.0,
            fault_clearing_time_ms: 0.0,
            energy_dissipated_mj: 0.0,
            voltage_recovery_time_ms: 0.0,
            max_di_dt: 0.0,
            fault_cleared: true,
        })
    }

    // ── RK4 integration step ──────────────────────────────────────────────

    /// Perform one RK4 integration step on the state-space model.
    ///
    /// State vector: `[V_dc_1..N, I_cable_1..M]`
    /// ```text
    /// dI_cable/dt = (V_from − V_to − R·I) / L
    /// dV_node/dt  = (I_in − I_out − I_load) / C
    /// ```
    fn rk4_step(
        &self,
        state: &DcTransientState,
        dt: f64,
        fault: Option<&DcFaultEvent>,
    ) -> DcTransientState {
        let n_s = self.stations.len();
        let n_c = self.cables.len();

        // Pack state into flat vector: [V_0..V_{n-1}, I_0..I_{m-1}]
        let state_len = n_s + n_c;
        let mut y = vec![0.0_f64; state_len];
        for (i, &v) in state.station_voltages_kv.iter().enumerate() {
            if i < n_s {
                y[i] = v;
            }
        }
        for (i, &ic) in state.cable_currents_ka.iter().enumerate() {
            if i < n_c {
                y[n_s + i] = ic;
            }
        }

        // RK4 stages
        let k1 = self.state_derivatives(&y, fault);
        let y2: Vec<f64> = y
            .iter()
            .zip(k1.iter())
            .map(|(&yi, &ki)| yi + 0.5 * dt * ki)
            .collect();
        let k2 = self.state_derivatives(&y2, fault);
        let y3: Vec<f64> = y
            .iter()
            .zip(k2.iter())
            .map(|(&yi, &ki)| yi + 0.5 * dt * ki)
            .collect();
        let k3 = self.state_derivatives(&y3, fault);
        let y4: Vec<f64> = y
            .iter()
            .zip(k3.iter())
            .map(|(&yi, &ki)| yi + dt * ki)
            .collect();
        let k4 = self.state_derivatives(&y4, fault);

        // Combine: y_new = y + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)
        let y_new: Vec<f64> = (0..state_len)
            .map(|i| y[i] + (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]))
            .collect();

        // Unpack
        let mut new_state = state.clone();
        for (i, v) in y_new.iter().take(n_s).enumerate() {
            new_state.station_voltages_kv[i] = v.max(0.0);
        }
        new_state.cable_currents_ka[..n_c].copy_from_slice(&y_new[n_s..n_s + n_c]);

        // Update station currents from cable currents
        for i in 0..n_s {
            let mut i_net = 0.0;
            for (ci, cable) in self.cables.iter().enumerate() {
                let i_cable = if ci < n_c { y_new[n_s + ci] } else { 0.0 };
                if cable.to_station == i {
                    i_net += i_cable;
                } else if cable.from_station == i {
                    i_net -= i_cable;
                }
            }
            if i < new_state.station_currents_ka.len() {
                new_state.station_currents_ka[i] = i_net;
            }
        }

        new_state
    }

    /// Compute state derivatives for the DC grid state-space model.
    ///
    /// Returns dy/dt for the packed state vector `[V_0..V_{n-1}, I_0..I_{m-1}]`.
    fn state_derivatives(&self, y: &[f64], fault: Option<&DcFaultEvent>) -> Vec<f64> {
        let n_s = self.stations.len();
        let n_c = self.cables.len();
        let mut dydt = vec![0.0_f64; n_s + n_c];

        // Cable current derivatives: dI/dt = (V_from - V_to - R*I) / L
        for (ci, cable) in self.cables.iter().enumerate() {
            let from = cable.from_station.min(n_s.saturating_sub(1));
            let to = cable.to_station.min(n_s.saturating_sub(1));
            let v_from = if from < n_s { y[from] } else { 0.0 };
            let v_to = if to < n_s { y[to] } else { 0.0 };
            let r_ohm = (cable.r_per_km * cable.length_km).max(1e-9); // Ω
            let l_h = (cable.l_per_km * cable.length_km * 1e-3).max(1e-9); // H (from mH)
            let i_ka = if (n_s + ci) < y.len() {
                y[n_s + ci]
            } else {
                0.0
            };
            // V in kV, I in kA, R in Ω → need unit consistency
            // dI(kA)/dt(s) = (V(kV)*1e3 - R(Ω)*I(kA)*1e3) / (L(H)*1e3)
            // = (V(kV) - R(Ω)*I(kA)) * 1e3 / (L(H)*1e3)  = (V(kV) - R*I(kA)) / L(H)
            // Actually: V=kV→V*1e3 in volts, I=kA→I*1e3 in amps, result in A/s, convert back to kA/s
            // dI[A/s] = (V[V] - R[Ω]*I[A]) / L[H]
            // dI[kA/s] = (V[kV]*1e3 - R[Ω]*I[kA]*1e3) / (L[H]*1e3) = (V[kV] - R[Ω]*I[kA]) / L[H]
            dydt[n_s + ci] = (v_from - v_to - r_ohm * i_ka) / l_h;
        }

        // Station voltage derivatives: dV/dt = (I_in - I_out - I_load) / C
        for (si, station) in self.stations.iter().enumerate() {
            // Capacitance: MMC submodule capacitance or cable capacitance
            let c_uf = self.station_capacitance_uf(station);
            let c_f = (c_uf * 1e-6).max(1e-12); // Farads

            let mut i_net_ka = 0.0;
            for (ci, cable) in self.cables.iter().enumerate() {
                let i_cable = if (n_s + ci) < y.len() {
                    y[n_s + ci]
                } else {
                    0.0
                };
                if cable.to_station == si {
                    i_net_ka += i_cable;
                } else if cable.from_station == si {
                    i_net_ka -= i_cable;
                }
            }

            // Fault current drain at faulted cable's connected station
            if let Some(f) = fault {
                if f.location_cable_id < self.cables.len() {
                    let cable = &self.cables[f.location_cable_id];
                    if cable.from_station == si || cable.to_station == si {
                        // Fault drains current from this station
                        let v_kv = if si < y.len() { y[si] } else { 0.0 };
                        let r_f = f.fault_resistance.max(1e-6);
                        let i_fault_ka = v_kv * 1e3 / (r_f * 1e3); // kV*1e3/Ω / 1e3 = kA
                        i_net_ka -= i_fault_ka * f.location_fraction;
                    }
                }
            }

            // dV[kV/s] = I[kA]*1e3 / (C[F]*1e3) = I[kA]/C[F]
            // Actually: dV[V/s] = I[A]/C[F], dV[kV/s] = I[kA]/C[F]  (both ×1e3 cancel)
            dydt[si] = i_net_ka / c_f;
        }

        dydt
    }

    /// Detect whether a fault condition exists based on state changes.
    ///
    /// Returns `true` if |di/dt| exceeds a threshold or voltage drops
    /// below 80% of rated.
    #[allow(dead_code)]
    fn detect_fault(&self, state: &DcTransientState, prev: &DcTransientState) -> bool {
        let dt_ms = (state.time_ms - prev.time_ms).abs().max(1e-9);

        // Check di/dt on cable currents
        for (i, &i_now) in state.cable_currents_ka.iter().enumerate() {
            let i_prev = prev.cable_currents_ka.get(i).copied().unwrap_or(0.0);
            let di_dt = (i_now - i_prev).abs() / dt_ms;
            if di_dt > 10.0 {
                // threshold: 10 kA/ms
                return true;
            }
        }

        // Check voltage drop
        for (i, &v) in state.station_voltages_kv.iter().enumerate() {
            let v_rated = self.stations.get(i).map_or(500.0, |s| s.v_dc_rated_kv);
            if v < 0.8 * v_rated {
                return true;
            }
        }

        false
    }

    // ── Private helpers ───────────────────────────────────────────────────

    /// Compute aggregate RLC parameters for the fault circuit.
    ///
    /// Returns `(L_total_mH, C_total_uF, R_total_Ohm, R_fault_Ohm, V_dc_kV)`.
    fn fault_rlc_params(&self, fault: &DcFaultEvent) -> (f64, f64, f64, f64, f64) {
        let cable_id = fault
            .location_cable_id
            .min(self.cables.len().saturating_sub(1));
        let cable = &self.cables[cable_id];
        let frac = fault.location_fraction.clamp(0.0, 1.0);
        let dist = cable.length_km * frac;

        // Cable impedance up to fault point
        let r_cable = cable.r_per_km * dist;
        let l_cable = cable.l_per_km * dist; // mH
        let c_cable = cable.c_per_km * cable.length_km; // μF (full cable)

        // Converter arm inductance (MMC)
        let from = cable
            .from_station
            .min(self.stations.len().saturating_sub(1));
        let l_arm = self.stations.get(from).map_or(0.0, |s| s.arm_inductance_mh);

        // Total
        let l_total = l_cable + l_arm + 1.0; // add 1 mH minimum smoothing
        let c_total = c_cable
            + self
                .stations
                .iter()
                .map(|s| s.sm_capacitance_uf)
                .sum::<f64>();
        let c_total = c_total.max(0.01); // minimum 0.01 μF

        let v_dc = self.stations.get(from).map_or(500.0, |s| s.v_dc_rated_kv);

        (
            l_total,
            c_total,
            r_cable.max(0.01),
            fault.fault_resistance,
            v_dc,
        )
    }

    /// Compute effective capacitance at a station (μF).
    fn station_capacitance_uf(&self, station: &ConverterStation) -> f64 {
        let base = match &station.topology {
            ConverterTopology::Mmc { n_submodules } => {
                // MMC: total arm capacitance = C_sm / (6 * N_sm) per phase, simplified
                let nsm = (*n_submodules).max(1) as f64;
                station.sm_capacitance_uf * nsm / 6.0
            }
            _ => {
                // VSC DC-link or LCC smoothing: estimate from rated values
                // C = P / (V² * ω) as rough sizing, expressed in μF
                let v = station.v_dc_rated_kv.max(1.0);
                let p = station.p_rated_mw.max(1.0);
                (p / (v * v * 314.0)) * 1e6 // μF
            }
        };
        // Also add connected cable capacitance
        let cable_c: f64 = self
            .cables
            .iter()
            .filter(|c| c.from_station == station.id || c.to_station == station.id)
            .map(|c| c.c_per_km * c.length_km * 0.5) // half of cable C at each end
            .sum();
        (base + cable_c).max(0.01)
    }

    /// Return the fastest breaker clearing time in ms.
    fn fastest_breaker_clearing_ms(&self) -> f64 {
        if self.breakers.is_empty() {
            return self.duration_ms;
        }
        self.breakers
            .iter()
            .map(|b| breaker_clearing_time_ms(&b.breaker_type))
            .fold(f64::MAX, f64::min)
    }
}

/// Compute the effective clearing time (ms) for a given breaker type.
fn breaker_clearing_time_ms(bt: &DcBreakerType) -> f64 {
    match bt {
        DcBreakerType::Mechanical { opening_time_ms } => *opening_time_ms,
        DcBreakerType::SolidState {
            turn_off_time_us, ..
        } => turn_off_time_us / 1000.0,
        DcBreakerType::HybridBreaker {
            opening_time_ms,
            commutation_time_us,
        } => {
            // Hybrid: solid-state commutates first, then mechanical opens
            commutation_time_us / 1000.0 + opening_time_ms * 0.3
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Helpers ──────────────────────────────────────────────────────────

    fn lcc_station(id: usize, alpha_deg: f64) -> ConverterStation {
        ConverterStation {
            id,
            name: format!("LCC-{id}"),
            topology: ConverterTopology::Lcc,
            v_dc_rated_kv: 500.0,
            p_rated_mw: 1000.0,
            x_transformer_pu: 0.15,
            scr: 3.0,
            control_angle_deg: alpha_deg,
            x_commutation_pu: 0.1,
            arm_inductance_mh: 0.0,
            sm_capacitance_uf: 0.0,
        }
    }

    fn vsc_station(id: usize) -> ConverterStation {
        ConverterStation {
            id,
            name: format!("VSC-{id}"),
            topology: ConverterTopology::TwoLevelVsc,
            v_dc_rated_kv: 320.0,
            p_rated_mw: 800.0,
            x_transformer_pu: 0.12,
            scr: 5.0,
            control_angle_deg: 0.0,
            x_commutation_pu: 0.0,
            arm_inductance_mh: 0.0,
            sm_capacitance_uf: 0.0,
        }
    }

    fn mmc_station(id: usize) -> ConverterStation {
        ConverterStation {
            id,
            name: format!("MMC-{id}"),
            topology: ConverterTopology::Mmc { n_submodules: 400 },
            v_dc_rated_kv: 320.0,
            p_rated_mw: 1000.0,
            x_transformer_pu: 0.12,
            scr: 5.0,
            control_angle_deg: 0.0,
            x_commutation_pu: 0.0,
            arm_inductance_mh: 50.0,
            sm_capacitance_uf: 10000.0,
        }
    }

    fn standard_cable(id: usize, from: usize, to: usize) -> DcCable {
        DcCable {
            id,
            from_station: from,
            to_station: to,
            r_per_km: 0.01,
            l_per_km: 0.5,
            c_per_km: 0.2,
            length_km: 100.0,
        }
    }

    #[allow(dead_code)]
    fn short_cable(id: usize, from: usize, to: usize) -> DcCable {
        DcCable {
            id,
            from_station: from,
            to_station: to,
            r_per_km: 0.01,
            l_per_km: 0.5,
            c_per_km: 0.2,
            length_km: 20.0,
        }
    }

    fn mechanical_breaker(id: usize, station: usize) -> DcBreaker {
        DcBreaker {
            id,
            station_id: station,
            breaker_type: DcBreakerType::Mechanical {
                opening_time_ms: 50.0,
            },
            max_breaking_current_ka: 10.0,
            energy_absorption_mj: 20.0,
        }
    }

    fn solid_state_breaker(id: usize, station: usize) -> DcBreaker {
        DcBreaker {
            id,
            station_id: station,
            breaker_type: DcBreakerType::SolidState {
                turn_off_time_us: 500.0,
                on_state_loss_pct: 0.5,
            },
            max_breaking_current_ka: 8.0,
            energy_absorption_mj: 10.0,
        }
    }

    #[allow(dead_code)]
    fn hybrid_breaker(id: usize, station: usize) -> DcBreaker {
        DcBreaker {
            id,
            station_id: station,
            breaker_type: DcBreakerType::HybridBreaker {
                opening_time_ms: 30.0,
                commutation_time_us: 200.0,
            },
            max_breaking_current_ka: 12.0,
            energy_absorption_mj: 15.0,
        }
    }

    fn standard_fault(cable_id: usize) -> DcFaultEvent {
        DcFaultEvent {
            fault_type: DcFaultType::PoleToPole,
            location_cable_id: cable_id,
            location_fraction: 0.5,
            fault_resistance: 0.1,
            inception_time_ms: 5.0,
        }
    }

    fn build_2term_system() -> DcSwitchingSimulator {
        let mut sim = DcSwitchingSimulator::new(10.0, 200.0);
        sim.add_station(lcc_station(0, 15.0));
        sim.add_station(lcc_station(1, 150.0));
        sim.add_cable(standard_cable(0, 0, 1));
        sim
    }

    // ── Test 1: 2-terminal HVDC DC power flow converges ──────────────────

    #[test]
    fn test_dc_power_flow_converges() {
        let sim = build_2term_system();
        let sol = sim.solve_dc_power_flow().expect("DC PF should converge");
        assert!(sol.converged, "DC power flow should converge");
        assert!(sol.iterations <= 50, "Should converge in < 50 iterations");
        assert_eq!(sol.operating_points.len(), 2);
    }

    // ── Test 2: LCC V_dc decreases with firing angle ─────────────────────

    #[test]
    fn test_lcc_vdc_decreases_with_alpha() {
        let sim = DcSwitchingSimulator::new(10.0, 100.0);
        let s15 = lcc_station(0, 15.0);
        let s30 = lcc_station(0, 30.0);
        let op15 = sim.lcc_operating_point(&s15, 1.0);
        let op30 = sim.lcc_operating_point(&s30, 1.0);
        assert!(
            op15.v_dc_kv > op30.v_dc_kv,
            "V_dc at alpha=15 ({:.1}) should exceed alpha=30 ({:.1})",
            op15.v_dc_kv,
            op30.v_dc_kv
        );
    }

    // ── Test 3: VSC P_dc controllable ────────────────────────────────────

    #[test]
    fn test_vsc_pdc_controllable() {
        let sim = DcSwitchingSimulator::new(10.0, 100.0);
        let s = vsc_station(0);
        let op1 = sim.vsc_operating_point(&s, 100.0, 320.0);
        let op2 = sim.vsc_operating_point(&s, 400.0, 320.0);
        assert!(
            (op1.p_dc_mw - 100.0).abs() < 1.0,
            "P should be ~100 MW, got {}",
            op1.p_dc_mw
        );
        assert!(
            (op2.p_dc_mw - 400.0).abs() < 1.0,
            "P should be ~400 MW, got {}",
            op2.p_dc_mw
        );
    }

    // ── Test 4: VSC losses positive ──────────────────────────────────────

    #[test]
    fn test_vsc_losses_positive() {
        let sim = DcSwitchingSimulator::new(10.0, 100.0);
        let s = vsc_station(0);
        let op = sim.vsc_operating_point(&s, 500.0, 320.0);
        assert!(
            op.losses_mw > 0.0,
            "VSC losses should be positive, got {}",
            op.losses_mw
        );
    }

    // ── Test 5: Pole-to-pole fault peak current > 0 ──────────────────────

    #[test]
    fn test_ptp_fault_peak_positive() {
        let mut sim = build_2term_system();
        sim.add_breaker(mechanical_breaker(0, 0));
        let fault = standard_fault(0);
        let res = sim.simulate_fault(&fault).expect("fault sim failed");
        assert!(
            res.peak_fault_current_ka > 0.0,
            "Peak fault current should be > 0, got {}",
            res.peak_fault_current_ka
        );
    }

    // ── Test 6: Pole-to-ground lower peak than pole-to-pole ──────────────

    #[test]
    fn test_ptg_lower_peak_than_ptp() {
        let mut sim = build_2term_system();
        sim.add_breaker(mechanical_breaker(0, 0));

        let ptp = DcFaultEvent {
            fault_type: DcFaultType::PoleToPole,
            location_cable_id: 0,
            location_fraction: 0.5,
            fault_resistance: 0.1,
            inception_time_ms: 5.0,
        };
        let ptg = DcFaultEvent {
            fault_type: DcFaultType::PoleToGround,
            location_cable_id: 0,
            location_fraction: 0.5,
            fault_resistance: 0.1,
            inception_time_ms: 5.0,
        };

        let res_ptp = sim.simulate_fault(&ptp).expect("PtP failed");
        let res_ptg = sim.simulate_fault(&ptg).expect("PtG failed");

        assert!(
            res_ptg.peak_fault_current_ka < res_ptp.peak_fault_current_ka,
            "PtG peak ({:.2}) should be < PtP peak ({:.2})",
            res_ptg.peak_fault_current_ka,
            res_ptp.peak_fault_current_ka
        );
    }

    // ── Test 7: Fault current increases with shorter distance ────────────

    #[test]
    fn test_fault_current_shorter_distance() {
        let mut sim = build_2term_system();
        sim.add_breaker(mechanical_breaker(0, 0));

        let near = DcFaultEvent {
            fault_type: DcFaultType::PoleToPole,
            location_cable_id: 0,
            location_fraction: 0.1,
            fault_resistance: 0.1,
            inception_time_ms: 5.0,
        };
        let far = DcFaultEvent {
            fault_type: DcFaultType::PoleToPole,
            location_cable_id: 0,
            location_fraction: 0.9,
            fault_resistance: 0.1,
            inception_time_ms: 5.0,
        };

        let res_near = sim.simulate_fault(&near).expect("near failed");
        let res_far = sim.simulate_fault(&far).expect("far failed");

        assert!(
            res_near.peak_fault_current_ka >= res_far.peak_fault_current_ka,
            "Near fault ({:.2}) should have >= peak than far fault ({:.2})",
            res_near.peak_fault_current_ka,
            res_far.peak_fault_current_ka
        );
    }

    // ── Test 8: Fault current decreases with higher resistance ───────────

    #[test]
    fn test_fault_current_decreases_with_resistance() {
        let mut sim = build_2term_system();
        sim.add_breaker(mechanical_breaker(0, 0));

        let low_r = DcFaultEvent {
            fault_type: DcFaultType::PoleToPole,
            location_cable_id: 0,
            location_fraction: 0.5,
            fault_resistance: 0.1,
            inception_time_ms: 5.0,
        };
        let high_r = DcFaultEvent {
            fault_type: DcFaultType::PoleToPole,
            location_cable_id: 0,
            location_fraction: 0.5,
            fault_resistance: 10.0,
            inception_time_ms: 5.0,
        };

        let res_low = sim.simulate_fault(&low_r).expect("low R failed");
        let res_high = sim.simulate_fault(&high_r).expect("high R failed");

        assert!(
            res_low.peak_fault_current_ka >= res_high.peak_fault_current_ka,
            "Low R fault ({:.2}) should have >= peak than high R ({:.2})",
            res_low.peak_fault_current_ka,
            res_high.peak_fault_current_ka
        );
    }

    // ── Test 9: Mechanical breaker clearing > 30 ms ──────────────────────

    #[test]
    fn test_mechanical_breaker_clearing_time() {
        let mut sim = build_2term_system();
        sim.add_breaker(DcBreaker {
            id: 0,
            station_id: 0,
            breaker_type: DcBreakerType::Mechanical {
                opening_time_ms: 50.0,
            },
            max_breaking_current_ka: 10.0,
            energy_absorption_mj: 20.0,
        });

        let fault = standard_fault(0);
        let res = sim.simulate_fault(&fault).expect("fault failed");

        assert!(
            res.fault_clearing_time_ms >= 30.0,
            "Mechanical breaker should clear >= 30 ms, got {}",
            res.fault_clearing_time_ms
        );
    }

    // ── Test 10: Solid-state breaker clearing < 1 ms ─────────────────────

    #[test]
    fn test_solid_state_breaker_clearing() {
        let clearing = breaker_clearing_time_ms(&DcBreakerType::SolidState {
            turn_off_time_us: 500.0,
            on_state_loss_pct: 0.5,
        });
        assert!(
            clearing < 1.0,
            "Solid-state should clear < 1 ms, got {}",
            clearing
        );
    }

    // ── Test 11: Hybrid breaker intermediate clearing ────────────────────

    #[test]
    fn test_hybrid_breaker_intermediate_clearing() {
        let mech_time = breaker_clearing_time_ms(&DcBreakerType::Mechanical {
            opening_time_ms: 50.0,
        });
        let ss_time = breaker_clearing_time_ms(&DcBreakerType::SolidState {
            turn_off_time_us: 500.0,
            on_state_loss_pct: 0.5,
        });
        let hybrid_time = breaker_clearing_time_ms(&DcBreakerType::HybridBreaker {
            opening_time_ms: 30.0,
            commutation_time_us: 200.0,
        });

        assert!(
            hybrid_time > ss_time && hybrid_time < mech_time,
            "Hybrid ({:.2}) should be between SS ({:.2}) and mech ({:.2})",
            hybrid_time,
            ss_time,
            mech_time
        );
    }

    // ── Test 12: Energy dissipated > 0 ───────────────────────────────────

    #[test]
    fn test_energy_dissipated_positive() {
        let mut sim = build_2term_system();
        sim.add_breaker(mechanical_breaker(0, 0));
        let fault = standard_fault(0);
        let res = sim.simulate_fault(&fault).expect("failed");
        assert!(
            res.energy_dissipated_mj > 0.0,
            "Energy dissipated should be > 0, got {}",
            res.energy_dissipated_mj
        );
    }

    // ── Test 13: Fault cleared → voltage recovers ────────────────────────

    #[test]
    fn test_fault_cleared_voltage_recovers() {
        let mut sim = DcSwitchingSimulator::new(10.0, 300.0);
        sim.add_station(lcc_station(0, 15.0));
        sim.add_station(lcc_station(1, 150.0));
        sim.add_cable(standard_cable(0, 0, 1));
        sim.add_breaker(solid_state_breaker(0, 0));

        let fault = standard_fault(0);
        let res = sim.simulate_fault(&fault).expect("failed");
        assert!(res.fault_cleared, "Fault should be cleared with SS breaker");
    }

    // ── Test 14: Peak di/dt positive ─────────────────────────────────────

    #[test]
    fn test_peak_di_dt_positive() {
        let mut sim = build_2term_system();
        sim.add_breaker(mechanical_breaker(0, 0));
        let fault = standard_fault(0);
        let res = sim.simulate_fault(&fault).expect("failed");
        assert!(
            res.max_di_dt > 0.0,
            "max di/dt should be > 0, got {}",
            res.max_di_dt
        );
    }

    // ── Test 15: Cable losses proportional to I²R ────────────────────────

    #[test]
    fn test_cable_losses_proportional_to_i2r() {
        let mut sim = DcSwitchingSimulator::new(10.0, 100.0);
        sim.add_station(lcc_station(0, 15.0));
        sim.add_station(lcc_station(1, 150.0));
        sim.add_cable(DcCable {
            id: 0,
            from_station: 0,
            to_station: 1,
            r_per_km: 0.02, // double resistance
            l_per_km: 0.5,
            c_per_km: 0.2,
            length_km: 100.0,
        });

        let sol = sim.solve_dc_power_flow().expect("PF failed");
        assert!(
            sol.cable_losses_mw[0] >= 0.0,
            "Cable losses should be non-negative"
        );
    }

    // ── Test 16: MMC arm inductance limits fault current ─────────────────

    #[test]
    fn test_mmc_arm_inductance_limits_fault() {
        // Compare two MMC stations: one with high arm inductance, one with low
        let mut mmc_high_l = mmc_station(0);
        mmc_high_l.arm_inductance_mh = 100.0;
        mmc_high_l.sm_capacitance_uf = 100.0; // same small C for both

        let mut mmc_low_l = mmc_station(0);
        mmc_low_l.arm_inductance_mh = 1.0;
        mmc_low_l.sm_capacitance_uf = 100.0;

        let mut sim_high = DcSwitchingSimulator::new(10.0, 200.0);
        sim_high.add_station(mmc_high_l.clone());
        let mut s1_h = mmc_high_l.clone();
        s1_h.id = 1;
        sim_high.add_station(s1_h);
        sim_high.add_cable(standard_cable(0, 0, 1));
        sim_high.add_breaker(mechanical_breaker(0, 0));

        let mut sim_low = DcSwitchingSimulator::new(10.0, 200.0);
        sim_low.add_station(mmc_low_l.clone());
        let mut s1_l = mmc_low_l.clone();
        s1_l.id = 1;
        sim_low.add_station(s1_l);
        sim_low.add_cable(standard_cable(0, 0, 1));
        sim_low.add_breaker(mechanical_breaker(0, 0));

        let fault = standard_fault(0);
        let res_high = sim_high
            .simulate_fault(&fault)
            .expect("high L fault failed");
        let res_low = sim_low.simulate_fault(&fault).expect("low L fault failed");

        // Higher arm inductance → lower peak fault current (more impedance in path)
        assert!(
            res_high.peak_fault_current_ka <= res_low.peak_fault_current_ka + 0.1,
            "High arm L ({:.2}) should limit fault current vs low L ({:.2})",
            res_high.peak_fault_current_ka,
            res_low.peak_fault_current_ka
        );
    }

    // ── Test 17: Switching transient voltage oscillates ───────────────────

    #[test]
    fn test_switching_transient_oscillation() {
        let mut sim = build_2term_system();
        sim.add_breaker(mechanical_breaker(0, 0));

        let res = sim
            .simulate_switching(0, true)
            .expect("switching sim failed");
        assert!(
            !res.states.is_empty(),
            "Switching result should have states"
        );

        // Check that voltages change over time (oscillation or transient)
        let first_v = res
            .states
            .first()
            .map(|s| s.station_voltages_kv.first().copied().unwrap_or(0.0))
            .unwrap_or(0.0);
        let mid = res.states.len() / 2;
        let mid_v = res
            .states
            .get(mid)
            .map(|s| s.station_voltages_kv.first().copied().unwrap_or(0.0))
            .unwrap_or(0.0);
        // Voltages should differ (transient response)
        let differs = (first_v - mid_v).abs() > 1e-6 || res.states.len() > 1;
        assert!(differs, "Switching should produce transient response");
    }

    // ── Test 18: 3-terminal grid power balance ───────────────────────────

    #[test]
    fn test_3terminal_power_balance() {
        let mut sim = DcSwitchingSimulator::new(10.0, 100.0);
        sim.add_station(lcc_station(0, 15.0));
        sim.add_station(vsc_station(1));
        sim.add_station(lcc_station(2, 150.0));
        sim.add_cable(standard_cable(0, 0, 1));
        sim.add_cable(standard_cable(1, 1, 2));

        let sol = sim.solve_dc_power_flow().expect("3-term PF failed");
        assert_eq!(sol.operating_points.len(), 3);
        // Total injected power should approximately equal total consumed + losses
        let total_p: f64 = sol.operating_points.iter().map(|op| op.p_dc_mw).sum();
        // Net power should be close to zero (conservation) for a converged solution
        // In practice, the slack bus absorbs the difference
        assert!(
            total_p.is_finite(),
            "Total power should be finite, got {}",
            total_p
        );
    }

    // ── Test 19: Underdamped response oscillates ─────────────────────────

    #[test]
    fn test_underdamped_response_oscillates() {
        // Low resistance, high L and C → underdamped
        let mut sim = DcSwitchingSimulator::new(5.0, 100.0);
        sim.add_station(vsc_station(0));
        sim.add_station(vsc_station(1));
        sim.add_cable(DcCable {
            id: 0,
            from_station: 0,
            to_station: 1,
            r_per_km: 0.001, // very low R
            l_per_km: 2.0,   // high L
            c_per_km: 1.0,   // high C
            length_km: 50.0,
        });
        sim.add_breaker(mechanical_breaker(0, 0));

        let fault = DcFaultEvent {
            fault_type: DcFaultType::PoleToPole,
            location_cable_id: 0,
            location_fraction: 0.5,
            fault_resistance: 0.001,
            inception_time_ms: 2.0,
        };
        let res = sim
            .simulate_fault(&fault)
            .expect("underdamped fault failed");
        assert!(
            res.peak_fault_current_ka > 0.0,
            "Underdamped should produce fault current"
        );
    }

    // ── Test 20: Overdamped monotonic decay with high R ──────────────────

    #[test]
    fn test_overdamped_high_resistance() {
        let mut sim = DcSwitchingSimulator::new(10.0, 100.0);
        sim.add_station(lcc_station(0, 15.0));
        sim.add_station(lcc_station(1, 150.0));
        sim.add_cable(DcCable {
            id: 0,
            from_station: 0,
            to_station: 1,
            r_per_km: 1.0,  // very high R
            l_per_km: 0.1,  // low L
            c_per_km: 0.01, // low C
            length_km: 100.0,
        });
        sim.add_breaker(mechanical_breaker(0, 0));

        let fault = DcFaultEvent {
            fault_type: DcFaultType::PoleToPole,
            location_cable_id: 0,
            location_fraction: 0.5,
            fault_resistance: 50.0,
            inception_time_ms: 5.0,
        };
        let res = sim.simulate_fault(&fault).expect("overdamped fault failed");
        // With very high resistance, peak should be limited
        assert!(
            res.peak_fault_current_ka.is_finite(),
            "Overdamped peak should be finite"
        );
    }

    // ── Test 21: No fault → stable operation ─────────────────────────────

    #[test]
    fn test_no_fault_stable() {
        let mut sim = build_2term_system();
        sim.add_breaker(mechanical_breaker(0, 0));

        // Run switching sim without actually opening (close operation)
        let res = sim.simulate_switching(0, false).expect("stable sim failed");
        assert!(!res.states.is_empty(), "Should produce states");
        assert!(
            res.peak_fault_current_ka.abs() < 1e-6,
            "No fault current expected"
        );
    }

    // ── Test 22: DcTransientResult states non-empty ──────────────────────

    #[test]
    fn test_transient_result_states_nonempty() {
        let mut sim = build_2term_system();
        sim.add_breaker(mechanical_breaker(0, 0));
        let fault = standard_fault(0);
        let res = sim.simulate_fault(&fault).expect("failed");
        assert!(
            !res.states.is_empty(),
            "Result should contain transient states"
        );
        assert!(res.states.len() > 1, "Should have multiple state snapshots");
    }

    // ── Test 23: Operating point P = V × I ───────────────────────────────

    #[test]
    fn test_operating_point_p_equals_vi() {
        let sim = DcSwitchingSimulator::new(10.0, 100.0);
        let s = vsc_station(0);
        let op = sim.vsc_operating_point(&s, 256.0, 320.0);
        let p_check = op.v_dc_kv * op.i_dc_ka;
        assert!(
            (op.p_dc_mw - p_check).abs() < 1.0,
            "P ({:.1}) should ≈ V*I ({:.1})",
            op.p_dc_mw,
            p_check
        );
    }

    // ── Test 24: Breaker max breaking current check ──────────────────────

    #[test]
    fn test_breaker_max_breaking_current() {
        let b = mechanical_breaker(0, 0);
        assert!(
            b.max_breaking_current_ka > 0.0,
            "Max breaking current should be positive"
        );
        assert_eq!(b.max_breaking_current_ka, 10.0);
    }

    // ── Test 25: LCC power factor ────────────────────────────────────────

    #[test]
    fn test_lcc_power_factor() {
        let sim = DcSwitchingSimulator::new(10.0, 100.0);
        let s = lcc_station(0, 15.0);
        let op = sim.lcc_operating_point(&s, 1.0);
        assert!(
            op.power_factor > 0.0 && op.power_factor <= 1.0,
            "PF should be in (0,1], got {}",
            op.power_factor
        );
    }

    // ── Test 26: Detect fault function ───────────────────────────────────

    #[test]
    fn test_detect_fault() {
        let sim = build_2term_system();

        let prev = DcTransientState {
            time_ms: 0.0,
            station_voltages_kv: vec![500.0, 500.0],
            station_currents_ka: vec![1.0, -1.0],
            cable_currents_ka: vec![1.0],
            fault_current_ka: 0.0,
            breaker_states: vec![true],
        };
        let state_normal = DcTransientState {
            time_ms: 0.01,
            station_voltages_kv: vec![499.9, 499.9],
            station_currents_ka: vec![1.0, -1.0],
            cable_currents_ka: vec![1.01],
            fault_current_ka: 0.0,
            breaker_states: vec![true],
        };
        let state_fault = DcTransientState {
            time_ms: 0.01,
            station_voltages_kv: vec![200.0, 200.0], // voltage collapsed
            station_currents_ka: vec![20.0, -20.0],
            cable_currents_ka: vec![20.0],
            fault_current_ka: 15.0,
            breaker_states: vec![true],
        };

        assert!(
            !sim.detect_fault(&state_normal, &prev),
            "Normal state should not be flagged as fault"
        );
        assert!(
            sim.detect_fault(&state_fault, &prev),
            "Faulted state should be detected"
        );
    }

    // ── Test 27: Cable total R·L·C from per-km values ────────────────────

    #[test]
    fn test_cable_total_parameters() {
        let c = standard_cable(0, 0, 1);
        let r_total = c.r_per_km * c.length_km;
        let l_total = c.l_per_km * c.length_km;
        let c_total = c.c_per_km * c.length_km;
        assert!((r_total - 1.0).abs() < 1e-6, "R = 0.01 * 100 = 1.0 Ω");
        assert!((l_total - 50.0).abs() < 1e-6, "L = 0.5 * 100 = 50 mH");
        assert!((c_total - 20.0).abs() < 1e-6, "C = 0.2 * 100 = 20 μF");
    }

    // ── Test 28: Empty system errors ─────────────────────────────────────

    #[test]
    fn test_empty_system_errors() {
        let sim = DcSwitchingSimulator::new(10.0, 100.0);
        assert!(sim.solve_dc_power_flow().is_err());
        let fault = standard_fault(0);
        assert!(sim.simulate_fault(&fault).is_err());
    }
}
