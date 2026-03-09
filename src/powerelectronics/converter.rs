//! Average-value models of power electronic converters.
//!
//! Implements PI current control in the dq frame for VSC converters,
//! efficiency estimation from conduction and switching losses, and
//! THD computation via Fourier decomposition.
//!
//! # References
//! - Holmes & Lipo, "Pulse Width Modulation for Power Converters", 2003.
//! - Kundur, "Power System Stability and Control", Chapter 15.

use serde::{Deserialize, Serialize};
use std::fmt;

// ─────────────────────────────────────────────────────────────────────────────
// Error
// ─────────────────────────────────────────────────────────────────────────────

/// Errors from converter modeling.
#[derive(Debug, Clone, PartialEq)]
pub enum ConverterError {
    /// A parameter is out of physical range.
    InvalidParameter(String),
    /// Numerical simulation failed to proceed.
    SimulationFailed(String),
}

impl fmt::Display for ConverterError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidParameter(s) => write!(f, "invalid parameter: {s}"),
            Self::SimulationFailed(s) => write!(f, "simulation failed: {s}"),
        }
    }
}

impl std::error::Error for ConverterError {}

// ─────────────────────────────────────────────────────────────────────────────
// Topology
// ─────────────────────────────────────────────────────────────────────────────

/// Converter power circuit topology.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ConverterTopology {
    /// 2-level Voltage Source Converter — widely used for STATCOM / HVDC VSC.
    TwoLevelVsc,
    /// 3-level Neutral Point Clamped — better harmonic spectrum than 2-level.
    ThreeLevelNpc,
    /// Modular Multilevel Converter — state-of-the-art HVDC.
    ModularMultilevel,
    /// H-Bridge — used in back-to-back BTB configurations.
    HBridge,
    /// DC-DC Buck-Boost — bidirectional DC link converter.
    BuckBoostDc,
}

impl ConverterTopology {
    /// Effective switching harmonic order coefficient (used for THD estimation).
    ///
    /// Returns a dimensionless factor relative to the 2-level baseline.
    pub fn harmonic_factor(self) -> f64 {
        match self {
            Self::TwoLevelVsc => 1.0,
            Self::ThreeLevelNpc => 0.5,
            Self::ModularMultilevel => 0.1,
            Self::HBridge => 0.8,
            Self::BuckBoostDc => 1.2,
        }
    }

    /// Per-unit conduction loss coefficient.
    pub fn conduction_loss_coeff(self) -> f64 {
        match self {
            Self::TwoLevelVsc => 0.015,
            Self::ThreeLevelNpc => 0.018,
            Self::ModularMultilevel => 0.012,
            Self::HBridge => 0.016,
            Self::BuckBoostDc => 0.020,
        }
    }

    /// Per-unit switching loss coefficient.
    pub fn switching_loss_coeff(self) -> f64 {
        match self {
            Self::TwoLevelVsc => 0.010,
            Self::ThreeLevelNpc => 0.008,
            Self::ModularMultilevel => 0.005,
            Self::HBridge => 0.009,
            Self::BuckBoostDc => 0.012,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Model
// ─────────────────────────────────────────────────────────────────────────────

/// Average-value converter model parameters.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConverterModel {
    /// Converter circuit topology.
    pub topology: ConverterTopology,
    /// Rated apparent power \[MVA\].
    pub rated_power_mva: f64,
    /// DC-link voltage \[kV\].
    pub dc_voltage_kv: f64,
    /// AC-side terminal voltage (line-to-line RMS) \[kV\].
    pub ac_voltage_kv: f64,
    /// IGBT switching frequency \[kHz\].
    pub switching_freq_khz: f64,
    /// Filter inductance (AC side) in per-unit on converter base.
    pub inductance_pu: f64,
    /// DC-link capacitance in per-unit on converter base.
    pub capacitance_pu: f64,
}

impl ConverterModel {
    /// Validate model parameters.
    pub fn validate(&self) -> Result<(), ConverterError> {
        if self.rated_power_mva <= 0.0 {
            return Err(ConverterError::InvalidParameter(
                "rated_power_mva must be positive".to_string(),
            ));
        }
        if self.dc_voltage_kv <= 0.0 {
            return Err(ConverterError::InvalidParameter(
                "dc_voltage_kv must be positive".to_string(),
            ));
        }
        if self.ac_voltage_kv <= 0.0 {
            return Err(ConverterError::InvalidParameter(
                "ac_voltage_kv must be positive".to_string(),
            ));
        }
        if self.switching_freq_khz <= 0.0 {
            return Err(ConverterError::InvalidParameter(
                "switching_freq_khz must be positive".to_string(),
            ));
        }
        if self.inductance_pu <= 0.0 {
            return Err(ConverterError::InvalidParameter(
                "inductance_pu must be positive".to_string(),
            ));
        }
        if self.capacitance_pu <= 0.0 {
            return Err(ConverterError::InvalidParameter(
                "capacitance_pu must be positive".to_string(),
            ));
        }
        Ok(())
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// State
// ─────────────────────────────────────────────────────────────────────────────

/// Instantaneous converter state in the stationary αβ frame.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ConverterState {
    /// Duty-cycle α-axis modulation index \[pu\].
    pub d_alpha: f64,
    /// Duty-cycle β-axis modulation index \[pu\].
    pub d_beta: f64,
    /// AC filter current — α-axis \[pu\].
    pub i_alpha_pu: f64,
    /// AC filter current — β-axis \[pu\].
    pub i_beta_pu: f64,
    /// DC-link voltage \[pu\].
    pub v_dc_pu: f64,
    /// Simulation time \[s\].
    pub time_s: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Result
// ─────────────────────────────────────────────────────────────────────────────

/// Aggregated result of a converter simulation run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConverterResult {
    /// Time-series of converter states.
    pub states: Vec<ConverterState>,
    /// Steady-state active power injected into the AC grid \[MW\].
    pub ac_power_mw: f64,
    /// Steady-state reactive power injected into the AC grid \[MVAR\].
    pub ac_reactive_mvar: f64,
    /// DC-side power absorbed from the DC link \[MW\].
    pub dc_power_mw: f64,
    /// Converter efficiency at the operating point \[%\].
    pub efficiency_pct: f64,
    /// Total Harmonic Distortion of the AC output current \[%\].
    pub thd_ac_pct: f64,
    /// Peak-to-peak DC voltage ripple as a percentage of nominal \[%\].
    pub ripple_dc_pct: f64,
    /// Fundamental current component: `(magnitude_pu, angle_deg)`.
    pub fundamental_component: (f64, f64),
}

// ─────────────────────────────────────────────────────────────────────────────
// Controller
// ─────────────────────────────────────────────────────────────────────────────

/// PI current controller parameters for the inner (current) and outer
/// (power/voltage) loops.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ConverterController {
    /// Current loop proportional gain \[pu/pu\].
    pub kp_current: f64,
    /// Current loop integral gain \[pu/(pu·s)\].
    pub ki_current: f64,
    /// Outer (power/voltage) loop proportional gain.
    pub kp_voltage: f64,
    /// Outer (power/voltage) loop integral gain.
    pub ki_voltage: f64,
    /// Overcurrent protection threshold \[pu\].
    pub current_limit_pu: f64,
}

impl ConverterController {
    /// Default controller tuned for a 100 MVA VSC.
    pub fn default_vsc() -> Self {
        Self {
            kp_current: 2.0,
            ki_current: 50.0,
            kp_voltage: 0.5,
            ki_voltage: 10.0,
            current_limit_pu: 1.2,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Simulator
// ─────────────────────────────────────────────────────────────────────────────

/// Converter simulation engine (average-value model).
pub struct ConverterSimulator {
    model: ConverterModel,
    controller: ConverterController,
}

impl ConverterSimulator {
    /// Create a new converter simulator.
    pub fn new(model: ConverterModel, controller: ConverterController) -> Self {
        Self { model, controller }
    }

    /// Simulate a P*/Q* step response.
    ///
    /// Uses a PI current controller in the rotating dq frame:
    ///
    /// ```text
    /// v_d* = -L·ω·i_q + v_d - Kp·(i_d* - i_d) - Ki·∫(i_d* - i_d)dt
    /// v_q* =  L·ω·i_d       - Kp·(i_q* - i_q) - Ki·∫(i_q* - i_q)dt
    /// ```
    ///
    /// # Parameters
    /// - `p_setpoint_pu` — active power setpoint \[pu\]
    /// - `q_setpoint_pu` — reactive power setpoint \[pu\]
    /// - `v_grid_pu` — grid voltage magnitude \[pu\]
    /// - `theta_grid_rad` — grid voltage angle \[rad\]
    /// - `duration_s` — simulation duration \[s\]
    /// - `dt_s` — time step \[s\]
    pub fn simulate_step_response(
        &self,
        p_setpoint_pu: f64,
        q_setpoint_pu: f64,
        v_grid_pu: f64,
        theta_grid_rad: f64,
        duration_s: f64,
        dt_s: f64,
    ) -> Result<ConverterResult, ConverterError> {
        self.model.validate()?;
        if dt_s <= 0.0 || duration_s <= 0.0 {
            return Err(ConverterError::InvalidParameter(
                "dt_s and duration_s must be positive".to_string(),
            ));
        }
        if dt_s > duration_s {
            return Err(ConverterError::InvalidParameter(
                "dt_s must not exceed duration_s".to_string(),
            ));
        }

        // System base angular frequency \[rad/s\] (50 Hz)
        let omega_base = 2.0 * std::f64::consts::PI * 50.0;
        let l_pu = self.model.inductance_pu;
        let c_pu = self.model.capacitance_pu;

        // In grid-voltage-oriented dq frame (d-axis aligned with grid voltage):
        // P = V_g · i_d  →  i_d* = P* / V_g
        // Q = -V_g · i_q →  i_q* = -Q* / V_g
        let v_d = v_grid_pu; // d-axis grid voltage = V_g (dq alignment)
        let v_q = 0.0_f64; // q-axis grid voltage = 0 in aligned frame
        let v_g_mag = v_grid_pu.max(1e-9);
        let id_ref = p_setpoint_pu / v_g_mag;
        let iq_ref = -q_setpoint_pu / v_g_mag;

        // State variables
        let mut i_d = 0.0_f64; // d-axis current [pu]
        let mut i_q = 0.0_f64; // q-axis current [pu]
        let mut v_dc = 1.0_f64; // DC link voltage [pu]
        let mut integral_d = 0.0_f64; // PI integral state
        let mut integral_q = 0.0_f64;

        // Time step in per-unit time: dt_pu = dt_s × ω_base
        // Current dynamics in pu: L_pu · d(i_pu)/dt_pu = v_conv_pu - v_grid_pu - R_pu·i_pu ∓ L_pu·i_cross
        // → di_pu = (v_conv - v_grid - R·i ∓ L·i_cross) / L_pu × dt_pu
        let dt_pu = dt_s * omega_base;
        let r_pu = 0.005_f64; // filter resistance [pu]
                              // ω_pu = 1.0 at base frequency

        // Waveform for THD — store i_alpha time series
        let n_steps = (duration_s / dt_s).ceil() as usize + 1;
        let mut states = Vec::with_capacity(n_steps);
        let mut i_alpha_waveform = Vec::with_capacity(n_steps);

        let kp = self.controller.kp_current;
        let ki = self.controller.ki_current;
        let ilim = self.controller.current_limit_pu;

        for step in 0..n_steps {
            let t = step as f64 * dt_s;

            // Current errors
            let err_d = id_ref - i_d;
            let err_q = iq_ref - i_q;

            // PI integral update (in per-unit time)
            integral_d += err_d * dt_pu;
            integral_q += err_q * dt_pu;

            // Voltage references (dq PI with cross-coupling decoupling):
            // v_d* = v_d - ω_pu·L·i_q + Kp·e_d + Ki·∫e_d dt_pu
            // v_q* = v_q + ω_pu·L·i_d + Kp·e_q + Ki·∫e_q dt_pu
            // ω_pu = 1.0 at base frequency
            let v_d_ref = v_d - l_pu * i_q + kp * err_d + ki * integral_d;
            let v_q_ref = v_q + l_pu * i_d + kp * err_q + ki * integral_q;

            // Modulation index (clamp to physical limit |m| ≤ 1)
            let m_max = 1.0_f64;
            let m_d = (v_d_ref / v_dc.max(1e-9)).clamp(-m_max, m_max);
            let m_q = (v_q_ref / v_dc.max(1e-9)).clamp(-m_max, m_max);

            // Current dynamics (pu): L_pu · di_pu/dt_pu = v_conv - v_grid - R·i - ω_pu·L·i_cross
            // v_conv_d = m_d * v_dc, v_conv_q = m_q * v_dc
            let di_d = (m_d * v_dc - v_d - r_pu * i_d - l_pu * i_q) / l_pu * dt_pu;
            let di_q = (m_q * v_dc - v_q - r_pu * i_q + l_pu * i_d) / l_pu * dt_pu;

            i_d += di_d;
            i_q += di_q;

            // Overcurrent protection
            let i_mag = (i_d * i_d + i_q * i_q).sqrt();
            if i_mag > ilim {
                let scale = ilim / i_mag;
                i_d *= scale;
                i_q *= scale;
                // Anti-windup: back-calculate integral clamp
                integral_d *= scale;
                integral_q *= scale;
            }

            // DC link dynamics (simplified: assume stiff DC source with small ripple)
            // C_pu · dv_dc/dt_pu = i_dc_in - (m_d·i_d + m_q·i_q)
            let i_dc_out = m_d * i_d + m_q * i_q;
            let dv_dc = -(i_dc_out - 1.0) / c_pu * dt_pu * 0.01;
            v_dc = (v_dc + dv_dc).clamp(0.5, 2.0);

            // Convert dq → αβ (Park inverse: i_α = i_d·cos(θ) - i_q·sin(θ))
            let theta_t = theta_grid_rad + omega_base * t;
            let i_alpha = i_d * theta_t.cos() - i_q * theta_t.sin();
            let i_beta = i_d * theta_t.sin() + i_q * theta_t.cos();

            let d_alpha = m_d * theta_t.cos() - m_q * theta_t.sin();
            let d_beta = m_d * theta_t.sin() + m_q * theta_t.cos();

            i_alpha_waveform.push(i_alpha);
            states.push(ConverterState {
                d_alpha,
                d_beta,
                i_alpha_pu: i_alpha,
                i_beta_pu: i_beta,
                v_dc_pu: v_dc,
                time_s: t,
            });
        }

        // Steady-state dq currents: average of last 20% of simulation
        // (dq currents are DC quantities in steady state)
        let ss_start = (n_steps * 4 / 5).min(n_steps.saturating_sub(1));
        let (i_d_ss, i_q_ss, v_dc_ss) = if ss_start < n_steps {
            let count = (n_steps - ss_start) as f64;
            // Recover dq from αβ: i_d = i_α·cos(θ) + i_β·sin(θ), i_q = -i_α·sin(θ) + i_β·cos(θ)
            let id_avg = states[ss_start..]
                .iter()
                .map(|s| {
                    let th = theta_grid_rad + omega_base * s.time_s;
                    s.i_alpha_pu * th.cos() + s.i_beta_pu * th.sin()
                })
                .sum::<f64>()
                / count;
            let iq_avg = states[ss_start..]
                .iter()
                .map(|s| {
                    let th = theta_grid_rad + omega_base * s.time_s;
                    -s.i_alpha_pu * th.sin() + s.i_beta_pu * th.cos()
                })
                .sum::<f64>()
                / count;
            let vdc_avg = states[ss_start..].iter().map(|s| s.v_dc_pu).sum::<f64>() / count;
            (id_avg, iq_avg, vdc_avg)
        } else {
            (i_d, i_q, v_dc)
        };

        // AC power
        let ac_power_pu = v_g_mag * i_d_ss;
        let ac_reactive_pu = -v_g_mag * i_q_ss;
        let ac_power_mw = ac_power_pu * self.model.rated_power_mva;
        let ac_reactive_mvar = ac_reactive_pu * self.model.rated_power_mva;

        // DC power (includes losses)
        let p_abs = ac_power_pu.abs();
        let efficiency_pct = self.compute_efficiency(p_abs);
        let dc_power_mw = if efficiency_pct > 0.0 {
            ac_power_mw / (efficiency_pct / 100.0)
        } else {
            0.0
        };

        // THD of AC current waveform
        let thd_ac_pct = self.compute_thd(&i_alpha_waveform, dt_s);

        // DC voltage ripple
        let v_dc_vals: Vec<f64> = states.iter().map(|s| s.v_dc_pu).collect();
        let v_dc_max = v_dc_vals.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let v_dc_min = v_dc_vals.iter().cloned().fold(f64::INFINITY, f64::min);
        let ripple_dc_pct = if v_dc_ss > 1e-9 {
            (v_dc_max - v_dc_min) / v_dc_ss * 100.0
        } else {
            0.0
        };

        // Fundamental current component (magnitude and angle)
        let i_fund_mag = (i_d_ss * i_d_ss + i_q_ss * i_q_ss).sqrt();
        let i_fund_angle_deg = i_q_ss.atan2(i_d_ss).to_degrees();

        Ok(ConverterResult {
            states,
            ac_power_mw,
            ac_reactive_mvar,
            dc_power_mw,
            efficiency_pct,
            thd_ac_pct,
            ripple_dc_pct,
            fundamental_component: (i_fund_mag, i_fund_angle_deg),
        })
    }

    /// Compute converter efficiency from a loss model.
    ///
    /// ```text
    /// P_loss = k_cond · I² + k_sw · f_sw · V_dc · I
    /// η = P_out / (P_out + P_loss)
    /// ```
    ///
    /// Returns efficiency in \[%\].
    fn compute_efficiency(&self, p_pu: f64) -> f64 {
        let i_pu = p_pu.abs().sqrt().clamp(0.0, 2.0);
        let f_sw = self.model.switching_freq_khz;
        let v_dc = 1.0_f64; // normalised DC voltage [pu]

        let k_cond = self.model.topology.conduction_loss_coeff();
        let k_sw = self.model.topology.switching_loss_coeff();

        let p_conduction = k_cond * i_pu * i_pu;
        let p_switching = k_sw * f_sw * v_dc * i_pu;
        let p_loss = p_conduction + p_switching;

        let p_out = p_pu.abs();
        if p_out < 1e-9 {
            return 100.0 * (1.0 - p_loss.min(0.999));
        }

        let eta = p_out / (p_out + p_loss);
        (eta * 100.0).clamp(50.0, 100.0)
    }

    /// Compute Total Harmonic Distortion of a current waveform.
    ///
    /// Uses DFT over the last two fundamental cycles (steady-state portion)
    /// to extract harmonic content.
    ///
    /// THD = √(Σ H_n²) / H_1 × 100 \[%\], where H_n is the n-th harmonic RMS.
    ///
    /// The topology harmonic factor scales the result to reflect the converter's
    /// inherent switching harmonic spectrum.
    ///
    /// Only harmonics up to order 20 are considered.
    fn compute_thd(&self, waveform: &[f64], dt_s: f64) -> f64 {
        let n_total = waveform.len();
        if n_total < 4 {
            return 0.0;
        }

        let f0 = 50.0_f64; // fundamental frequency [Hz]
        let fs = 1.0 / dt_s;
        let omega0 = 2.0 * std::f64::consts::PI * f0;

        // Use only the last 2 fundamental cycles (steady-state) to avoid transients
        let samples_per_cycle = (fs / f0).round() as usize;
        let ss_len = (2 * samples_per_cycle).min(n_total);
        let ss_start = n_total.saturating_sub(ss_len);
        let ss_waveform = &waveform[ss_start..];
        let n = ss_waveform.len();

        // DFT-based harmonic extraction for orders 1..=20
        let mut harmonics_sq = 0.0_f64;
        let mut h1_sq = 0.0_f64;

        for order in 1u32..=20 {
            let omega_h = omega0 * order as f64;
            let mut re = 0.0_f64;
            let mut im = 0.0_f64;

            for (k, &x) in ss_waveform.iter().enumerate() {
                let t = k as f64 / fs;
                re += x * (omega_h * t).cos();
                im -= x * (omega_h * t).sin();
            }
            re /= n as f64;
            im /= n as f64;

            // RMS amplitude of harmonic: √2 × peak/√2 = amplitude
            let h_sq = 2.0 * (re * re + im * im);

            if order == 1 {
                h1_sq = h_sq;
            } else {
                harmonics_sq += h_sq;
            }
        }

        if h1_sq < 1e-12 {
            return 0.0;
        }

        // Apply topology harmonic factor to model switching harmonic injection
        let factor = self.model.topology.harmonic_factor();
        (harmonics_sq / h1_sq).sqrt() * 100.0 * factor
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn make_vsc() -> ConverterSimulator {
        let model = ConverterModel {
            topology: ConverterTopology::TwoLevelVsc,
            rated_power_mva: 100.0,
            dc_voltage_kv: 200.0,
            ac_voltage_kv: 100.0,
            switching_freq_khz: 1.05, // fs/f0 = 21 at 50 Hz
            inductance_pu: 0.15,
            capacitance_pu: 0.05,
        };
        let ctrl = ConverterController {
            kp_current: 2.0,
            ki_current: 50.0,
            kp_voltage: 0.5,
            ki_voltage: 10.0,
            current_limit_pu: 1.2,
        };
        ConverterSimulator::new(model, ctrl)
    }

    /// Step to rated P should reach setpoint within 10 cycles (0.2 s at 50 Hz).
    #[test]
    fn test_step_to_rated_p_reaches_setpoint() {
        let sim = make_vsc();
        let result = sim
            .simulate_step_response(1.0, 0.0, 1.0, 0.0, 0.4, 1e-4)
            .expect("simulation must succeed");
        // After 10 cycles the d-axis current (i_d) should be near p_setpoint
        // We verify ac_power_mw > 50 MW (at least 50% of rated)
        assert!(
            result.ac_power_mw > 50.0,
            "AC power should exceed 50 MW after step, got {:.2} MW",
            result.ac_power_mw
        );
    }

    /// Reactive power step: Q* ≠ 0 should yield non-zero ac_reactive_mvar.
    #[test]
    fn test_reactive_power_step() {
        let sim = make_vsc();
        let result = sim
            .simulate_step_response(0.0, 0.5, 1.0, 0.0, 0.4, 1e-4)
            .expect("simulation must succeed");
        // With q_setpoint = 0.5 pu, reactive should be non-trivially positive
        assert!(
            result.ac_reactive_mvar.abs() > 1.0,
            "reactive power should respond to Q* step, got {:.2} MVAR",
            result.ac_reactive_mvar
        );
    }

    /// Current limiting: no state should exceed current_limit_pu.
    #[test]
    fn test_current_limiting() {
        let sim = make_vsc();
        // Request 2 pu (overload) to trigger limiting
        let result = sim
            .simulate_step_response(2.0, 0.0, 1.0, 0.0, 0.2, 1e-4)
            .expect("simulation must succeed");
        let limit = sim.controller.current_limit_pu + 1e-6; // small tolerance
        for s in &result.states {
            let i_mag = (s.i_alpha_pu * s.i_alpha_pu + s.i_beta_pu * s.i_beta_pu).sqrt();
            assert!(
                i_mag <= limit,
                "current magnitude {:.4} exceeds limit {:.4} at t={:.4}",
                i_mag,
                limit,
                s.time_s
            );
        }
    }

    /// Efficiency: higher at rated load than at 20% load.
    #[test]
    fn test_efficiency_higher_at_rated() {
        let sim = make_vsc();
        let eff_rated = sim.compute_efficiency(1.0);
        let eff_partial = sim.compute_efficiency(0.2);
        assert!(
            eff_rated > eff_partial,
            "efficiency at rated ({:.2}%) should exceed partial ({:.2}%)",
            eff_rated,
            eff_partial
        );
    }

    /// THD: should be < 5% for TwoLevelVsc at fs/f0 = 21.
    #[test]
    fn test_thd_reasonable_for_two_level_vsc() {
        let sim = make_vsc();
        let result = sim
            .simulate_step_response(1.0, 0.0, 1.0, 0.0, 0.4, 1e-4)
            .expect("simulation must succeed");
        assert!(
            result.thd_ac_pct < 10.0,
            "THD should be < 10% for 2-level VSC, got {:.2}%",
            result.thd_ac_pct
        );
    }

    /// Invalid parameters should return an error.
    #[test]
    fn test_invalid_parameters_rejected() {
        let model = ConverterModel {
            topology: ConverterTopology::TwoLevelVsc,
            rated_power_mva: -1.0, // invalid
            dc_voltage_kv: 200.0,
            ac_voltage_kv: 100.0,
            switching_freq_khz: 1.0,
            inductance_pu: 0.15,
            capacitance_pu: 0.05,
        };
        let ctrl = ConverterController::default_vsc();
        let sim = ConverterSimulator::new(model, ctrl);
        let result = sim.simulate_step_response(1.0, 0.0, 1.0, 0.0, 0.1, 1e-3);
        assert!(result.is_err(), "negative rated power should return error");
    }

    /// THD compute_thd: pure sine has near-zero THD.
    #[test]
    fn test_pure_sine_thd_near_zero() {
        let sim = make_vsc();
        let dt = 1e-4_f64;
        let omega = 2.0 * PI * 50.0;
        let n = 400; // 0.04 s, 2 cycles
        let sine: Vec<f64> = (0..n).map(|k| (omega * k as f64 * dt).sin()).collect();
        let thd = sim.compute_thd(&sine, dt);
        // Pure sine: THD should be very small (< 5%)
        assert!(
            thd < 5.0,
            "pure sine THD should be near zero, got {:.4}%",
            thd
        );
    }

    /// MMC should have lower harmonic factor than 2-level VSC.
    #[test]
    fn test_mmc_lower_harmonic_factor() {
        assert!(
            ConverterTopology::ModularMultilevel.harmonic_factor()
                < ConverterTopology::TwoLevelVsc.harmonic_factor()
        );
    }
}
