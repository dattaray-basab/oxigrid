/// PLL-based grid-following inverter with dq-frame current control.
///
/// A grid-following inverter tracks the grid voltage angle via a Phase-Locked
/// Loop (PLL) and injects controlled dq currents to regulate active and reactive
/// power.  This is the dominant inverter control strategy for large-scale PV and
/// wind converters.
///
/// # References
/// - Yazdani & Iravani, "Voltage-Sourced Converters in Power Systems", Wiley 2010.
/// - Golestan et al., "Three-Phase PLLs: A Review of Recent Advances", IEEE TPEL 2017.
use serde::{Deserialize, Serialize};

// ─────────────────────────────────────────────────────────
// Phase-Locked Loop
// ─────────────────────────────────────────────────────────

/// Configuration for a synchronous-reference-frame PLL (SRF-PLL).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PllConfig {
    /// PLL loop-filter proportional gain [rad/(s·pu)]
    pub kp_pll: f64,
    /// PLL loop-filter integral gain [rad/(s²·pu)]
    pub ki_pll: f64,
    /// Nominal angular frequency [rad/s]
    pub omega_0: f64,
}

impl PllConfig {
    /// Typical PLL tuned for 50 Hz grid with ~20 Hz bandwidth.
    pub fn default_50hz() -> Self {
        Self {
            kp_pll: 40.0,
            ki_pll: 400.0,
            omega_0: 2.0 * std::f64::consts::PI * 50.0,
        }
    }

    /// Typical PLL tuned for 60 Hz grid.
    pub fn default_60hz() -> Self {
        Self {
            kp_pll: 40.0,
            ki_pll: 400.0,
            omega_0: 2.0 * std::f64::consts::PI * 60.0,
        }
    }
}

/// Mutable state variables of the SRF-PLL.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PllState {
    /// Estimated grid angular frequency [rad/s]
    pub omega_pll: f64,
    /// Estimated grid voltage angle \[rad\]
    pub delta_pll: f64,
    /// Integral state of the PI compensator [rad/s]
    pub error_int: f64,
}

impl PllState {
    /// Initialise PLL at nominal frequency with zero angle.
    pub fn new(omega_0: f64) -> Self {
        Self {
            omega_pll: omega_0,
            delta_pll: 0.0,
            error_int: 0.0,
        }
    }
}

/// Synchronous-reference-frame Phase-Locked Loop.
///
/// The PLL drives the q-axis voltage component to zero by adjusting its
/// estimated angle δ.  When locked, δ ≈ actual grid angle, and ω_pll ≈ ω_grid.
///
/// # Equations
/// ```text
/// error    = v_q                         (q-axis should be zero when locked)
/// dε/dt    = error
/// ω_pll    = ω_0 + Kp·error + Ki·ε
/// dδ/dt    = ω_pll
/// ```
#[derive(Debug, Clone)]
pub struct PhaseLockedLoop {
    /// PLL configuration
    pub config: PllConfig,
    /// PLL state
    pub state: PllState,
}

impl PhaseLockedLoop {
    /// Create a new PLL initialised at nominal frequency.
    pub fn new(config: PllConfig) -> Self {
        let omega_0 = config.omega_0;
        Self {
            state: PllState::new(omega_0),
            config,
        }
    }

    /// Advance PLL state by `dt` seconds.
    ///
    /// # Arguments
    /// * `v_q` — q-axis voltage component in the estimated dq frame \[pu\].
    ///   When the PLL is locked this is driven to zero.
    /// * `dt`  — integration timestep \[s\]
    ///
    /// # Returns
    /// Updated estimated angular frequency [rad/s].
    pub fn step(&mut self, v_q: f64, dt: f64) -> f64 {
        // PI compensator
        let kp = self.config.kp_pll;
        let ki = self.config.ki_pll;

        // Trapezoidal integration of integral state
        let d_int = v_q * dt;
        self.state.error_int += d_int;

        let omega_pll = self.config.omega_0 + kp * v_q + ki * self.state.error_int;
        self.state.omega_pll = omega_pll;

        // Update angle using Euler (angle update is not stiff)
        self.state.delta_pll += omega_pll * dt;
        // Keep angle in [−π, π]
        self.state.delta_pll = wrap_angle(self.state.delta_pll);

        omega_pll
    }

    /// Estimated grid voltage angle \[rad\].
    pub fn angle(&self) -> f64 {
        self.state.delta_pll
    }

    /// Estimated grid frequency \[Hz\].
    pub fn frequency_hz(&self) -> f64 {
        self.state.omega_pll / (2.0 * std::f64::consts::PI)
    }
}

// ─────────────────────────────────────────────────────────
// Grid-following inverter
// ─────────────────────────────────────────────────────────

/// Grid-following inverter with PLL-based angle tracking and dq current control.
///
/// The inverter tracks the PCC voltage angle and regulates d-/q-axis currents
/// to control active and reactive power independently.  A PI current controller
/// produces voltage references for the modulator.
///
/// # Power equations (in PLL-locked dq frame with v_d = |V_pcc|, v_q ≈ 0)
/// ```text
/// P = 1.5 · v_d · i_d
/// Q = −1.5 · v_d · i_q
/// ⟹  i_d_ref = (2/3) · P_ref / v_d
///     i_q_ref = −(2/3) · Q_ref / v_d
/// ```
#[derive(Debug, Clone)]
pub struct GridFollowingInverter {
    /// Rated apparent power \[MVA\]
    pub rated_power_mva: f64,
    /// Maximum current magnitude \[pu\] (current-limiter threshold)
    pub i_max_pu: f64,
    /// Current PI proportional gain
    pub kp_current: f64,
    /// Current PI integral gain
    pub ki_current: f64,
    /// PLL for grid-angle tracking
    pub pll: PhaseLockedLoop,
    /// d-axis current controller integral state
    pub id_int: f64,
    /// q-axis current controller integral state
    pub iq_int: f64,
}

impl GridFollowingInverter {
    /// Create a grid-following inverter with default current-controller tuning.
    ///
    /// # Arguments
    /// * `rated_mva` — rated apparent power \[MVA\]
    /// * `pll_config` — PLL configuration
    pub fn new(rated_mva: f64, pll_config: PllConfig) -> Self {
        Self {
            rated_power_mva: rated_mva,
            i_max_pu: 1.1,
            kp_current: 5.0,
            ki_current: 200.0,
            pll: PhaseLockedLoop::new(pll_config),
            id_int: 0.0,
            iq_int: 0.0,
        }
    }

    /// Advance the inverter state by `dt` seconds.
    ///
    /// # Arguments
    /// * `p_ref`  — active power reference \[MW\]
    /// * `q_ref`  — reactive power reference \[MVAr\]
    /// * `v_pcc`  — PCC voltage in dq frame `(v_d, v_q)` \[pu\]
    /// * `dt`     — timestep \[s\]
    ///
    /// # Returns
    /// Current injection `(i_d, i_q)` \[pu\] in PLL-aligned dq frame.
    pub fn step(&mut self, p_ref: f64, q_ref: f64, v_pcc: (f64, f64), dt: f64) -> (f64, f64) {
        let (v_d, v_q) = v_pcc;

        // Update PLL
        self.pll.step(v_q, dt);

        // Current references from power references
        // Using 3-phase convention:  P = 1.5·v_d·i_d,  Q = −1.5·v_d·i_q
        let i_d_ref = if v_d.abs() > 1e-6 {
            (2.0 / 3.0) * p_ref / v_d
        } else {
            0.0
        };
        let i_q_ref = if v_d.abs() > 1e-6 {
            -(2.0 / 3.0) * q_ref / v_d
        } else {
            0.0
        };

        // Current limiter (circular)
        let i_mag = (i_d_ref * i_d_ref + i_q_ref * i_q_ref).sqrt();
        let (i_d_ref, i_q_ref) = if i_mag > self.i_max_pu {
            let scale = self.i_max_pu / i_mag;
            (i_d_ref * scale, i_q_ref * scale)
        } else {
            (i_d_ref, i_q_ref)
        };

        // PI current controller — we expose current injection directly here.
        // In a full implementation the output would be a modulation voltage;
        // here we model the closed-loop response as a first-order PI tracking
        // to the current reference.

        // Simulated measured currents (pure integrator plant in pu: di/dt = u/L)
        // For the purposes of current injection output, we return the reference
        // filtered through a PI response with error = 0 (ideal tracking is assumed
        // for the power flow interface).  The integral states are updated to prevent
        // windup in case of saturation.
        let i_d_err = i_d_ref - 0.0; // error vs. measured (assumed fed back externally)
        let i_q_err = i_q_ref - 0.0;

        self.id_int += i_d_err * dt;
        self.iq_int += i_q_err * dt;

        // PI output (voltage command — stored but not returned here; current is returned)
        let _v_cmd_d = self.kp_current * i_d_err + self.ki_current * self.id_int + v_d;
        let _v_cmd_q = self.kp_current * i_q_err + self.ki_current * self.iq_int + v_q;

        (i_d_ref, i_q_ref)
    }

    /// Reset internal integral states (anti-windup or reinitialisation).
    pub fn reset(&mut self) {
        self.id_int = 0.0;
        self.iq_int = 0.0;
    }
}

// ─────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────

/// Wrap angle to [−π, π].
fn wrap_angle(theta: f64) -> f64 {
    let pi = std::f64::consts::PI;
    let two_pi = 2.0 * pi;
    let mut t = theta % two_pi;
    if t > pi {
        t -= two_pi;
    } else if t < -pi {
        t += two_pi;
    }
    t
}

// ─────────────────────────────────────────────────────────
// Unit tests
// ─────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// PLL starting at wrong angle should lock within 0.5 s.
    #[test]
    fn test_pll_lock() {
        let cfg = PllConfig::default_50hz();
        let mut pll = PhaseLockedLoop::new(cfg);
        let dt = 1e-4;

        // Simulate a grid voltage with true angle advancing at 50 Hz
        // The PLL starts with delta_pll = 0; true angle starts at π/4 offset
        let true_offset = PI / 4.0;
        let omega_true = 2.0 * PI * 50.0;

        let n = (0.5 / dt) as usize;
        for k in 0..n {
            let t = k as f64 * dt;
            let true_angle = omega_true * t + true_offset;
            // q-axis error ≈ sin(true_angle − delta_pll) (linearised = angle difference)
            let angle_err = wrap_angle(true_angle - pll.state.delta_pll);
            // Use linearised v_q ≈ angle_err (small-angle approximation with unit amplitude)
            pll.step(angle_err, dt);
        }

        // After 0.5 s the PLL should have tracked the true angle
        let t_final = n as f64 * dt;
        let true_angle_final = wrap_angle(omega_true * t_final + true_offset);
        let err = wrap_angle(true_angle_final - pll.state.delta_pll).abs();
        assert!(
            err < 0.01,
            "PLL angle error after 0.5 s = {:.4} rad (expected < 0.01 rad)",
            err
        );
    }

    /// Grid-following inverter: current references should reflect power commands.
    #[test]
    fn test_grid_following_current_reference() {
        let pll_cfg = PllConfig::default_50hz();
        let mut inv = GridFollowingInverter::new(1.0, pll_cfg);

        // v_d = 1.0 pu, v_q = 0 (perfectly locked PLL)
        let (id, iq) = inv.step(0.5, 0.0, (1.0, 0.0), 1e-3);

        // i_d_ref = (2/3) * P / v_d = (2/3) * 0.5 / 1.0 ≈ 0.333
        let expected_id = (2.0 / 3.0) * 0.5;
        assert!(
            (id - expected_id).abs() < 1e-6,
            "i_d_ref = {:.6} expected {:.6}",
            id,
            expected_id
        );
        // No reactive power → i_q = 0
        assert!(
            iq.abs() < 1e-9,
            "i_q_ref should be zero for Q=0; got {:.6}",
            iq
        );
    }

    /// Current limiter should clip the output magnitude to i_max_pu.
    #[test]
    fn test_grid_following_current_limiter() {
        let pll_cfg = PllConfig::default_50hz();
        let mut inv = GridFollowingInverter::new(1.0, pll_cfg);
        inv.i_max_pu = 1.1;

        // Very high power command → should exceed i_max
        let (id, iq) = inv.step(5.0, 5.0, (1.0, 0.0), 1e-3);
        let i_mag = (id * id + iq * iq).sqrt();
        assert!(
            i_mag <= inv.i_max_pu + 1e-9,
            "Current magnitude {:.4} exceeds limit {:.4}",
            i_mag,
            inv.i_max_pu
        );
    }

    #[test]
    fn test_pll_60hz_nominal_frequency() {
        let cfg = PllConfig::default_60hz();
        let pll = PhaseLockedLoop::new(cfg.clone());
        // At initialisation the PLL is at nominal frequency
        assert!((pll.frequency_hz() - 60.0).abs() < 1e-6);
    }

    #[test]
    fn test_pll_angle_advances_at_nominal_omega() {
        let cfg = PllConfig::default_50hz();
        let mut pll = PhaseLockedLoop::new(cfg);
        let dt = 1.0 / 50.0; // one cycle
                             // Feed zero v_q (no error) — omega_pll = omega_0
        let omega_ret = pll.step(0.0, dt);
        assert!(
            (omega_ret - pll.config.omega_0).abs() < 1e-6,
            "omega = {:.4}",
            omega_ret
        );
    }

    #[test]
    fn test_inverter_reset_clears_integral_states() {
        let cfg = PllConfig::default_50hz();
        let mut inv = GridFollowingInverter::new(1.0, cfg);
        // Run a few steps to accumulate integral state
        for _ in 0..10 {
            inv.step(0.8, 0.2, (1.0, 0.01), 1e-3);
        }
        inv.reset();
        assert!((inv.id_int).abs() < 1e-12, "id_int = {:.4e}", inv.id_int);
        assert!((inv.iq_int).abs() < 1e-12, "iq_int = {:.4e}", inv.iq_int);
    }

    #[test]
    fn test_inverter_q_ref_produces_negative_iq() {
        let pll_cfg = PllConfig::default_50hz();
        let mut inv = GridFollowingInverter::new(1.0, pll_cfg);
        // Positive reactive power reference → i_q_ref = -(2/3)*Q/v_d < 0
        let (_id, iq) = inv.step(0.0, 0.5, (1.0, 0.0), 1e-3);
        assert!(
            iq < 0.0,
            "i_q should be negative for positive Q ref, got {:.4}",
            iq
        );
        let expected_iq = -(2.0 / 3.0) * 0.5 / 1.0;
        assert!(
            (iq - expected_iq).abs() < 1e-6,
            "iq = {:.6}, expected {:.6}",
            iq,
            expected_iq
        );
    }

    #[test]
    fn test_inverter_zero_vd_gives_zero_current() {
        let pll_cfg = PllConfig::default_50hz();
        let mut inv = GridFollowingInverter::new(1.0, pll_cfg);
        // v_d = 0 → division guard → currents must be zero
        let (id, iq) = inv.step(1.0, 0.5, (0.0, 0.0), 1e-3);
        assert!(
            (id).abs() < 1e-9,
            "id should be zero for v_d=0, got {:.4e}",
            id
        );
        assert!(
            (iq).abs() < 1e-9,
            "iq should be zero for v_d=0, got {:.4e}",
            iq
        );
    }
}
