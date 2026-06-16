/// Virtual Synchronous Machine (VSM) and droop control for grid-forming inverters.
///
/// The VSM emulates synchronous machine dynamics so that a power-electronics-based
/// inverter contributes inertia and damping to the grid.  The swing equation is
/// integrated with 4th-order Runge-Kutta (RK4).
///
/// # References
/// - Zhong & Weiss, "Synchronverters: Inverters That Mimic Synchronous Generators",
///   IEEE Trans. Ind. Electron., 2011.
/// - Driesen & Visscher, "Virtual Synchronous Generators", IEEE PES GM, 2008.
use crate::error::{OxiGridError, Result};
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

// ─────────────────────────────────────────────────────────
// VSM configuration
// ─────────────────────────────────────────────────────────

/// Configuration parameters for a Virtual Synchronous Machine.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VsmConfig {
    /// Rated apparent power \[MVA\]
    pub rated_power_mva: f64,
    /// Rated line voltage \[kV\]
    pub rated_voltage_kv: f64,
    /// Virtual inertia \[kg·m²\]; mapped internally to H (inertia constant in seconds)
    /// via  H = J·ω_ref² / (2·S_rated_J)  (S in joules = MVA·1e6)
    pub inertia_constant_j: f64,
    /// Damping coefficient \[pu\] relative to rated power
    pub damping_d: f64,
    /// Active power (frequency) droop gain [Hz/MW]
    pub droop_kp: f64,
    /// Reactive power (voltage) droop gain [V/MVAr] (expressed in pu/pu)
    pub droop_kq: f64,
    /// Voltage reference \[pu\]
    pub v_ref: f64,
    /// Angular frequency reference [rad/s] (typically 2π·50 or 2π·60)
    pub omega_ref: f64,
    /// Simulation timestep \[s\]
    pub dt: f64,
}

impl VsmConfig {
    /// Typical 1 MVA grid-forming inverter at 50 Hz.
    pub fn default_1mva_50hz() -> Self {
        Self {
            rated_power_mva: 1.0,
            rated_voltage_kv: 0.4,
            inertia_constant_j: 100.0,
            damping_d: 0.5,
            droop_kp: 0.04,
            droop_kq: 0.05,
            v_ref: 1.0,
            omega_ref: 2.0 * PI * 50.0,
            dt: 1e-3,
        }
    }

    /// Inertia constant H \[s\] derived from J \[kg·m²\].
    ///
    /// H = J·ω_ref² / (2·S_rated)   where S_rated is in \[J\]
    pub fn inertia_h_s(&self) -> f64 {
        let s_j = self.rated_power_mva * 1.0e6; // joules
        self.inertia_constant_j * self.omega_ref * self.omega_ref / (2.0 * s_j)
    }

    /// Angular momentum M = 2H/ω_ref [s²/rad] used in swing equation.
    pub fn angular_momentum_m(&self) -> f64 {
        let h = self.inertia_h_s();
        2.0 * h / self.omega_ref
    }
}

// ─────────────────────────────────────────────────────────
// VSM state
// ─────────────────────────────────────────────────────────

/// Instantaneous state variables of a Virtual Synchronous Machine.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VsmState {
    /// Angular frequency of virtual rotor [rad/s]
    pub omega: f64,
    /// Virtual rotor angle \[rad\]
    pub delta: f64,
    /// d-axis output voltage component \[pu\]
    pub v_d: f64,
    /// q-axis output voltage component \[pu\]
    pub v_q: f64,
    /// Filtered active power output \[MW\]
    pub p_out: f64,
    /// Filtered reactive power output \[MVAr\]
    pub q_out: f64,
    /// Excitation (field) voltage \[pu\] — drives v_d, v_q via AVR droop
    pub e_fd: f64,
}

impl VsmState {
    /// Initialise at the operating point defined by `v_ref` and `omega_ref`.
    pub fn new(v_ref: f64, omega_ref: f64) -> Self {
        Self {
            omega: omega_ref,
            delta: 0.0,
            v_d: v_ref,
            v_q: 0.0,
            p_out: 0.0,
            q_out: 0.0,
            e_fd: v_ref,
        }
    }
}

// ─────────────────────────────────────────────────────────
// VSM output
// ─────────────────────────────────────────────────────────

/// Outputs produced by one VSM timestep.
#[derive(Debug, Clone)]
pub struct VsmOutput {
    /// d-axis voltage reference for inner current/voltage controller \[pu\]
    pub v_ref_d: f64,
    /// q-axis voltage reference \[pu\]
    pub v_ref_q: f64,
    /// Current angular frequency of virtual rotor [rad/s]
    pub omega: f64,
    /// Current virtual rotor angle \[rad\]
    pub delta: f64,
    /// Active power setpoint issued by droop law \[MW\]
    pub p_setpoint: f64,
    /// Reactive power setpoint issued by voltage droop \[MVAr\]
    pub q_setpoint: f64,
}

// ─────────────────────────────────────────────────────────
// Virtual Synchronous Machine
// ─────────────────────────────────────────────────────────

/// Grid-forming inverter controller modelled as a Virtual Synchronous Machine.
///
/// The VSM maps the swing-equation dynamics of a synchronous generator onto a
/// power-electronics inverter so that the inverter provides virtual inertia and
/// participates in primary frequency / voltage regulation.
///
/// # Dynamics
///
/// ```text
/// M·dω/dt  = P_set − P_out − D·(ω − ω_ref)          [swing]
/// dδ/dt    = ω − ω_ref                                [rotor angle]
/// P_set    = P_rated − K_p·(ω − ω_ref)/(2π)          [f-droop]
/// E_fd     = V_ref − K_q·(Q_out − Q_ref)              [v-droop]
/// τ_f·dP/dt = P_meas − P_out                          [LPF on power]
/// τ_f·dQ/dt = Q_meas − Q_out                          [LPF on power]
/// ```
pub struct VirtualSynchronousMachine {
    /// Configuration parameters (read-only after construction)
    pub config: VsmConfig,
    /// Mutable state variables
    pub state: VsmState,
    /// Low-pass filter time constant for P/Q measurement \[s\] (default 0.02 s)
    pub tau_f: f64,
    /// Active power rated operating point \[MW\] (= rated_power_mva at unity p.f.)
    pub p_rated_mw: f64,
    /// Reactive power reference \[MVAr\] (typically 0 for unity power factor)
    pub q_ref_mvar: f64,
}

impl VirtualSynchronousMachine {
    /// Create a VSM with the given configuration.
    ///
    /// The machine starts at `v_ref`, `omega_ref`, zero power, zero angle.
    pub fn new(config: VsmConfig) -> Self {
        let v_ref = config.v_ref;
        let omega_ref = config.omega_ref;
        let p_rated = config.rated_power_mva;
        Self {
            state: VsmState::new(v_ref, omega_ref),
            config,
            tau_f: 0.02,
            p_rated_mw: p_rated,
            q_ref_mvar: 0.0,
        }
    }

    // ── Internal: compute instantaneous power setpoints ──────────────────────

    fn p_setpoint_mw(&self) -> f64 {
        let delta_omega = self.state.omega - self.config.omega_ref;
        let freq_dev_hz = delta_omega / (2.0 * PI);
        self.p_rated_mw - self.config.droop_kp * freq_dev_hz
    }

    fn e_fd_ref(&self) -> f64 {
        let q_err = self.state.q_out - self.q_ref_mvar;
        (self.config.v_ref - self.config.droop_kq * q_err).max(0.0)
    }

    // ── Internal: RK4 for swing equation ─────────────────────────────────────

    /// Compute derivatives [dω, dδ, dp, dq] at the current state given
    /// instantaneous measurements.
    fn derivatives(
        &self,
        omega: f64,
        _delta: f64,
        p_out: f64,
        q_out: f64,
        p_meas: f64,
        q_meas: f64,
    ) -> (f64, f64, f64, f64) {
        let m = self.config.angular_momentum_m();
        let d = self.config.damping_d;
        let omega_ref = self.config.omega_ref;

        // Droop setpoint at *current* omega
        let delta_omega = omega - omega_ref;
        let freq_dev_hz = delta_omega / (2.0 * PI);
        let p_set = self.p_rated_mw - self.config.droop_kp * freq_dev_hz;

        let d_omega = (p_set - p_out - d * delta_omega) / m;
        let d_delta = omega - omega_ref;

        // First-order LPF on power measurements
        let d_p = (p_meas - p_out) / self.tau_f;
        let d_q = (q_meas - q_out) / self.tau_f;

        (d_omega, d_delta, d_p, d_q)
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /// Advance the VSM state by one timestep using RK4.
    ///
    /// # Arguments
    /// * `p_meas`   — measured active power injection at PCC \[MW\]
    /// * `q_meas`   — measured reactive power injection at PCC \[MVAr\]
    /// * `v_pcc_d`  — d-axis PCC voltage \[pu\]
    /// * `v_pcc_q`  — q-axis PCC voltage \[pu\]
    ///
    /// # Returns
    /// [`VsmOutput`] with updated voltage references and frequency.
    pub fn step(&mut self, p_meas: f64, q_meas: f64, v_pcc_d: f64, v_pcc_q: f64) -> VsmOutput {
        let dt = self.config.dt;

        // Capture current state
        let omega0 = self.state.omega;
        let delta0 = self.state.delta;
        let p0 = self.state.p_out;
        let q0 = self.state.q_out;

        // ── RK4 integration ──────────────────────────────────────────────────
        let (k1_w, k1_d, k1_p, k1_q) = self.derivatives(omega0, delta0, p0, q0, p_meas, q_meas);

        let (k2_w, k2_d, k2_p, k2_q) = self.derivatives(
            omega0 + 0.5 * dt * k1_w,
            delta0 + 0.5 * dt * k1_d,
            p0 + 0.5 * dt * k1_p,
            q0 + 0.5 * dt * k1_q,
            p_meas,
            q_meas,
        );

        let (k3_w, k3_d, k3_p, k3_q) = self.derivatives(
            omega0 + 0.5 * dt * k2_w,
            delta0 + 0.5 * dt * k2_d,
            p0 + 0.5 * dt * k2_p,
            q0 + 0.5 * dt * k2_q,
            p_meas,
            q_meas,
        );

        let (k4_w, k4_d, k4_p, k4_q) = self.derivatives(
            omega0 + dt * k3_w,
            delta0 + dt * k3_d,
            p0 + dt * k3_p,
            q0 + dt * k3_q,
            p_meas,
            q_meas,
        );

        self.state.omega += dt / 6.0 * (k1_w + 2.0 * k2_w + 2.0 * k3_w + k4_w);
        self.state.delta += dt / 6.0 * (k1_d + 2.0 * k2_d + 2.0 * k3_d + k4_d);
        self.state.p_out += dt / 6.0 * (k1_p + 2.0 * k2_p + 2.0 * k3_p + k4_p);
        self.state.q_out += dt / 6.0 * (k1_q + 2.0 * k2_q + 2.0 * k3_q + k4_q);

        // ── Update voltage reference via field excitation ─────────────────────
        let e_fd = self.e_fd_ref();
        self.state.e_fd = e_fd;

        // The VSM produces a voltage at the virtual rotor angle relative to PCC.
        // In the dq frame aligned to PCC voltage, the inverter voltage reference is:
        //   v_ref_d = E_fd · cos(δ) + v_pcc_d
        //   v_ref_q = E_fd · sin(δ)
        // (simplified; inner voltage controller tracks this reference)
        let v_ref_d = e_fd * self.state.delta.cos() + v_pcc_d * 0.0; // PCC already in dq
        let v_ref_q = e_fd * self.state.delta.sin() + v_pcc_q * 0.0;

        // Store for external query
        self.state.v_d = v_ref_d;
        self.state.v_q = v_ref_q;

        let p_set = self.p_setpoint_mw();

        VsmOutput {
            v_ref_d,
            v_ref_q,
            omega: self.state.omega,
            delta: self.state.delta,
            p_setpoint: p_set,
            q_setpoint: self.q_ref_mvar,
        }
    }

    /// Current dq voltage reference from the VSM.
    pub fn voltage_reference(&self) -> (f64, f64) {
        (self.state.v_d, self.state.v_q)
    }

    /// Frequency deviation from nominal \[Hz\].
    pub fn frequency_deviation_hz(&self) -> f64 {
        (self.state.omega - self.config.omega_ref) / (2.0 * PI)
    }

    /// Current frequency \[Hz\].
    pub fn frequency_hz(&self) -> f64 {
        self.state.omega / (2.0 * PI)
    }
}

// ─────────────────────────────────────────────────────────
// Microgrid simulator
// ─────────────────────────────────────────────────────────

/// Simulation result for a microgrid with multiple VSM-based inverters.
#[derive(Debug, Clone)]
pub struct MicrogridSimResult {
    /// Time vector \[s\]
    pub time: Vec<f64>,
    /// Frequency trajectory per inverter \[Hz\] — outer index = inverter, inner = time
    pub frequency_hz: Vec<Vec<f64>>,
    /// Active power sharing per inverter \[MW\]
    pub power_sharing: Vec<Vec<f64>>,
    /// PCC voltage magnitude \[pu\] (approximated as mean E_fd across inverters)
    pub voltage_pcc: Vec<f64>,
    /// Frequency nadir \[Hz\]
    pub nadir_hz: f64,
    /// Time to recover within 0.5 % of nominal frequency \[s\]; `f64::NAN` if never
    pub recovery_time_s: f64,
}

/// Simulate a microgrid formed by multiple VSM-based grid-forming inverters.
///
/// The inverters share a common PCC.  Power balance is maintained by distributing
/// the total load according to each inverter's droop characteristic.  The PCC
/// voltage is modelled as the average field excitation voltage across all inverters
/// (adequate for radial microgrids with negligible line impedances).
pub struct MicrogridSimulator {
    /// Participating grid-forming inverters
    pub inverters: Vec<VirtualSynchronousMachine>,
    /// Integration timestep \[s\]
    pub dt: f64,
    /// Total simulation duration \[s\]
    pub duration_s: f64,
}

impl MicrogridSimulator {
    /// Create a new microgrid simulator.
    pub fn new(inverters: Vec<VirtualSynchronousMachine>, dt: f64, duration_s: f64) -> Self {
        Self {
            inverters,
            dt,
            duration_s,
        }
    }

    /// Simulate a sudden active-power load step.
    ///
    /// # Arguments
    /// * `step_time_s` — time at which the load step occurs \[s\]
    /// * `delta_p_mw`  — magnitude of load increase \[MW\] (positive = more load)
    ///
    /// # Returns
    /// [`MicrogridSimResult`] with per-inverter and aggregate trajectories.
    pub fn simulate_load_step(
        &mut self,
        step_time_s: f64,
        delta_p_mw: f64,
    ) -> Result<MicrogridSimResult> {
        let n_inv = self.inverters.len();
        if n_inv == 0 {
            return Err(OxiGridError::InvalidParameter(
                "microgrid must have at least one inverter".into(),
            ));
        }
        if self.dt <= 0.0 || self.duration_s <= 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "dt and duration_s must be positive".into(),
            ));
        }

        let n_steps = (self.duration_s / self.dt).ceil() as usize + 1;

        let mut time = Vec::with_capacity(n_steps);
        let mut freq_hz: Vec<Vec<f64>> = vec![Vec::with_capacity(n_steps); n_inv];
        let mut power: Vec<Vec<f64>> = vec![Vec::with_capacity(n_steps); n_inv];
        let mut voltage_pcc = Vec::with_capacity(n_steps);

        // Each inverter contributes proportionally to its rated power.
        let total_rated: f64 = self.inverters.iter().map(|inv| inv.p_rated_mw).sum();

        // Initialise extra load to zero; applied after step_time_s
        let mut extra_load_mw = 0.0_f64;

        for step in 0..n_steps {
            let t = step as f64 * self.dt;

            // Trigger load step
            if t >= step_time_s && extra_load_mw < delta_p_mw * 0.5 {
                extra_load_mw = delta_p_mw;
            }

            // Distribute extra load by rated-power share
            let pcc_v_sum: f64 = self.inverters.iter().map(|inv| inv.state.e_fd).sum();
            let pcc_v_avg = pcc_v_sum / n_inv as f64;

            for (i, inv) in self.inverters.iter_mut().enumerate() {
                let share = if total_rated > 0.0 {
                    inv.p_rated_mw / total_rated
                } else {
                    1.0 / n_inv as f64
                };
                // Demanded power = the inverter's rated operating point plus its
                // proportional share of the stepped load. Using a fixed setpoint
                // (not `p_out`) is essential: tying it to `p_out` makes the
                // power-LPF target track the state, so `p_out` either never moves
                // (pre-step) or ramps without bound (post-step), collapsing ω.
                let p_meas = inv.p_rated_mw + share * extra_load_mw;
                let q_meas = inv.state.q_out;

                // PCC voltage in dq frame — simplified: d-axis = pcc_v_avg, q=0
                let out = inv.step(p_meas, q_meas, pcc_v_avg, 0.0);

                freq_hz[i].push(out.omega / (2.0 * PI));
                power[i].push(inv.state.p_out);
            }

            voltage_pcc.push(pcc_v_avg);
            time.push(t);
        }

        // Compute nadir and recovery time
        let nominal_hz = self.inverters[0].config.omega_ref / (2.0 * PI);
        let tolerance = nominal_hz * 0.005; // 0.5 %

        let nadir_hz = freq_hz
            .iter()
            .flat_map(|f| f.iter().copied())
            .fold(f64::INFINITY, f64::min);

        let recovery_time_s = self.find_recovery_time(&freq_hz, &time, nominal_hz, tolerance);

        Ok(MicrogridSimResult {
            time,
            frequency_hz: freq_hz,
            power_sharing: power,
            voltage_pcc,
            nadir_hz,
            recovery_time_s,
        })
    }

    /// Find the first time all inverters are within `tolerance` of nominal after
    /// the nadir has occurred.
    fn find_recovery_time(
        &self,
        freq_hz: &[Vec<f64>],
        time: &[f64],
        nominal_hz: f64,
        tolerance: f64,
    ) -> f64 {
        // Find time index of nadir first
        let mut nadir_idx = 0_usize;
        let mut nadir_val = f64::INFINITY;
        let n = time.len();
        for k in 0..n {
            let f_k: f64 = freq_hz.iter().map(|fv| fv[k]).fold(f64::INFINITY, f64::min);
            if f_k < nadir_val {
                nadir_val = f_k;
                nadir_idx = k;
            }
        }
        // Search after nadir
        for k in nadir_idx..n {
            let all_ok = freq_hz
                .iter()
                .all(|fv| (fv[k] - nominal_hz).abs() <= tolerance);
            if all_ok {
                return time[k];
            }
        }
        f64::NAN
    }
}

// ─────────────────────────────────────────────────────────
// Unit tests
// ─────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_vsm(h_s: Option<f64>, droop_kp: Option<f64>) -> VirtualSynchronousMachine {
        let omega_ref = 2.0 * PI * 50.0;
        let s_mva = 1.0_f64;
        // Convert H [s] → J [kg·m²]:  J = 2·H·S_J / ω_ref²
        let h = h_s.unwrap_or(5.0);
        let s_j = s_mva * 1e6;
        let j_kgm2 = 2.0 * h * s_j / (omega_ref * omega_ref);

        VsmConfig {
            rated_power_mva: s_mva,
            rated_voltage_kv: 0.4,
            inertia_constant_j: j_kgm2,
            damping_d: 0.5,
            droop_kp: droop_kp.unwrap_or(0.04),
            droop_kq: 0.05,
            v_ref: 1.0,
            omega_ref,
            dt: 1e-3,
        }
        .into()
    }

    // Helper: convert VsmConfig into a VirtualSynchronousMachine directly
    impl From<VsmConfig> for VirtualSynchronousMachine {
        fn from(cfg: VsmConfig) -> Self {
            VirtualSynchronousMachine::new(cfg)
        }
    }

    /// At steady-state (P_meas = P_rated, no disturbance), ω should remain ≈ ω_ref.
    #[test]
    fn test_vsm_steady_state() {
        let mut vsm = make_vsm(None, None);
        let p_rated = vsm.p_rated_mw;
        let omega_ref = vsm.config.omega_ref;

        // Run 5 s with constant rated load
        let steps = (5.0 / vsm.config.dt) as usize;
        for _ in 0..steps {
            vsm.step(p_rated, 0.0, 1.0, 0.0);
        }

        let dev = (vsm.state.omega - omega_ref).abs();
        assert!(
            dev < 0.01,
            "steady-state ω deviation = {:.4e} rad/s (expected < 0.01)",
            dev
        );
    }

    /// A +10 % load step should cause ω to drop initially then be restored by droop.
    #[test]
    fn test_vsm_droop_response() {
        let mut vsm = make_vsm(None, None);
        let p_rated = vsm.p_rated_mw;
        let omega_ref = vsm.config.omega_ref;
        let dt = vsm.config.dt;

        // Settle for 1 s at rated operating point
        let settle = (1.0 / dt) as usize;
        for _ in 0..settle {
            vsm.step(p_rated, 0.0, 1.0, 0.0);
        }

        // Apply +10 % load step and record ω over next 4 s
        let step_load = p_rated * 1.10;
        let sim = (4.0 / dt) as usize;
        let mut omega_min = f64::INFINITY;
        let mut omega_final = omega_ref;
        for k in 0..sim {
            let out = vsm.step(step_load, 0.0, 1.0, 0.0);
            if out.omega < omega_min {
                omega_min = out.omega;
            }
            if k == sim - 1 {
                omega_final = out.omega;
            }
        }

        // Frequency should have dropped below nominal
        assert!(
            omega_min < omega_ref,
            "ω should dip below ω_ref after load step; nadir = {:.4} rad/s",
            omega_min
        );
        // Droop brings it back towards reference (within 2 Hz = 12.6 rad/s)
        assert!(
            (omega_final - omega_ref).abs() < 2.0 * 2.0 * PI,
            "final ω deviation = {:.4} rad/s",
            (omega_final - omega_ref).abs()
        );
        // P_out should have increased towards the step load
        assert!(
            vsm.state.p_out > p_rated,
            "P_out should increase after load step; P_out = {:.4} MW",
            vsm.state.p_out
        );
    }

    /// Frequency nadir after a 0.2 pu load step should stay above 49.0 Hz.
    #[test]
    fn test_vsm_frequency_nadir() {
        let mut vsm = make_vsm(None, None);
        let p_rated = vsm.p_rated_mw;
        let dt = vsm.config.dt;

        let step_load = p_rated * 1.20; // 0.2 pu extra = 20 % step
        let sim = (5.0 / dt) as usize;
        let mut omega_min = f64::INFINITY;
        for _ in 0..sim {
            let out = vsm.step(step_load, 0.0, 1.0, 0.0);
            if out.omega < omega_min {
                omega_min = out.omega;
            }
        }
        let f_nadir = omega_min / (2.0 * PI);
        assert!(
            f_nadir > 49.0,
            "frequency nadir = {:.3} Hz (expected > 49.0 Hz)",
            f_nadir
        );
    }

    /// Higher inertia (H=5 s) should produce a lower ROCOF than low inertia (H=1 s)
    /// after the same load step.
    #[test]
    fn test_vsm_inertia_scaling() {
        let measure_rocof = |h: f64| -> f64 {
            let mut vsm = make_vsm(Some(h), None);
            let p_step = vsm.p_rated_mw * 1.05;
            let dt = vsm.config.dt;
            let omega_ref = vsm.config.omega_ref;
            // One step after load step
            let out = vsm.step(p_step, 0.0, 1.0, 0.0);
            (out.omega - omega_ref).abs() / dt
        };

        let rocof_high = measure_rocof(5.0);
        let rocof_low = measure_rocof(1.0);
        assert!(
            rocof_high < rocof_low,
            "Higher H should give lower ROCOF: ROCOF(H=5)={:.4}, ROCOF(H=1)={:.4}",
            rocof_high,
            rocof_low
        );
    }

    /// Two equal inverters sharing a load step should each carry roughly half.
    #[test]
    fn test_microgrid_power_sharing() {
        let vsm1 = make_vsm(None, None);
        let vsm2 = make_vsm(None, None);
        let dt = 1e-3;
        let mut sim = MicrogridSimulator::new(vec![vsm1, vsm2], dt, 3.0);
        let result = sim
            .simulate_load_step(0.5, 0.4)
            .expect("simulation should succeed");

        // At end of simulation, each inverter should carry roughly half the step
        let n = result.time.len();
        let p1_final = result.power_sharing[0][n - 1];
        let p2_final = result.power_sharing[1][n - 1];
        let ratio = if p2_final.abs() > 1e-9 {
            p1_final / p2_final
        } else {
            1.0
        };
        assert!(
            (ratio - 1.0).abs() < 0.05,
            "Power sharing ratio = {:.4} (expected within 5% of 1.0)",
            ratio
        );
    }

    #[test]
    fn test_frequency_deviation_hz_initially_zero() {
        let vsm = make_vsm(None, None);
        assert!(
            vsm.frequency_deviation_hz().abs() < 1e-9,
            "frequency deviation should be zero at construction; got {}",
            vsm.frequency_deviation_hz()
        );
    }

    #[test]
    fn test_frequency_hz_at_nominal() {
        let vsm = make_vsm(None, None);
        assert!(
            (vsm.frequency_hz() - 50.0).abs() < 0.001,
            "frequency should be 50.0 Hz at construction; got {}",
            vsm.frequency_hz()
        );
    }

    #[test]
    fn test_vsm_step_returns_consistent_omega() {
        let mut vsm = make_vsm(None, None);
        let p = vsm.p_rated_mw;
        let out = vsm.step(p, 0.0, 1.0, 0.0);
        assert_eq!(
            out.omega, vsm.state.omega,
            "VsmOutput.omega must match state.omega after step"
        );
    }

    #[test]
    fn test_vsm_rotor_angle_accumulates() {
        let mut vsm = make_vsm(None, None);
        let p = vsm.p_rated_mw;
        for _ in 0..100 {
            vsm.step(p, 0.0, 1.0, 0.0);
        }
        assert!(
            vsm.state.delta.abs() < 0.1,
            "delta should remain near 0 at rated load; got {}",
            vsm.state.delta
        );
    }

    #[test]
    fn test_vsm_voltage_reference_at_zero_angle() {
        let vsm = make_vsm(None, None);
        let (vd, vq) = vsm.voltage_reference();
        assert!(
            (vd - 1.0).abs() < 1e-9,
            "initial v_d should be v_ref=1.0; got {}",
            vd
        );
        assert!(vq.abs() < 1e-9, "initial v_q should be 0.0; got {}", vq);
    }

    #[test]
    fn test_vsm_higher_droop_kp_larger_frequency_restore() {
        let run = |kp: f64| -> f64 {
            let mut vsm = make_vsm(None, Some(kp));
            let p_step = vsm.p_rated_mw * 1.10;
            let steps = (3.0 / vsm.config.dt) as usize;
            for _ in 0..steps {
                vsm.step(p_step, 0.0, 1.0, 0.0);
            }
            vsm.state.omega
        };
        let omega_ref = 2.0 * PI * 50.0;
        let omega_low = run(0.02);
        let omega_high = run(0.08);
        assert!(
            (omega_high - omega_ref).abs() < (omega_low - omega_ref).abs(),
            "higher droop_kp should restore frequency closer to ref: low={:.4} high={:.4}",
            omega_low,
            omega_high
        );
    }

    #[test]
    fn test_vsm_reactive_power_voltage_droop() {
        let mut vsm = make_vsm(None, None);
        let p = vsm.p_rated_mw;
        let steps = (2.0 / vsm.config.dt) as usize;
        for _ in 0..steps {
            vsm.step(p, 0.5, 1.0, 0.0);
        }
        assert!(
            vsm.state.e_fd < 1.0,
            "reactive load should lower E_fd via voltage droop; e_fd = {:.4}",
            vsm.state.e_fd
        );
    }

    #[test]
    fn test_microgrid_nadir_above_49hz() {
        let vsm = make_vsm(None, None);
        let mut sim = MicrogridSimulator::new(vec![vsm], 1e-3, 5.0);
        let result = sim
            .simulate_load_step(0.5, 0.1)
            .expect("simulation should succeed");
        assert!(
            result.nadir_hz > 49.0,
            "nadir_hz = {:.3} Hz, expected > 49.0 Hz",
            result.nadir_hz
        );
    }
}
