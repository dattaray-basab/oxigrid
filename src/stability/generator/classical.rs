/// Classical generator model for power system stability studies.
///
/// The "classical" model represents a synchronous generator as a constant
/// voltage source E' behind a transient reactance X'd.  This is the
/// simplest model used in transient stability simulation.
///
/// # State variables
/// - δ (rotor angle, rad)
/// - ω (rotor speed deviation from synchronous, rad/s)
///
/// # Swing equation
///   M·dω/dt = Tm − Te − D·ω
///   dδ/dt   = ω
///
/// where M = 2H/ωs, Te = E'·V·sin(δ−θ)/X'd_total
use serde::{Deserialize, Serialize};

/// Parameters for the classical generator model.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ClassicalGeneratorParams {
    /// Inertia constant H [s]
    pub h: f64,
    /// Damping coefficient D [p.u.]
    pub d: f64,
    /// Transient reactance X'd [p.u.]
    pub x_d_prime: f64,
    /// Internal voltage magnitude E' [p.u.] (constant in classical model)
    pub e_prime: f64,
    /// Nominal frequency [Hz]
    pub freq_hz: f64,
    /// Machine MVA rating
    pub mva_rating: f64,
}

impl ClassicalGeneratorParams {
    /// Typical large steam generator (600 MW class).
    pub fn steam_600mw() -> Self {
        Self {
            h: 6.0,
            d: 2.0,
            x_d_prime: 0.20,
            e_prime: 1.05,
            freq_hz: 60.0,
            mva_rating: 600.0,
        }
    }

    /// Typical hydro generator (200 MW class).
    pub fn hydro_200mw() -> Self {
        Self {
            h: 4.0,
            d: 1.5,
            x_d_prime: 0.28,
            e_prime: 1.02,
            freq_hz: 60.0,
            mva_rating: 200.0,
        }
    }

    /// Typical gas turbine generator (100 MW class).
    pub fn gas_turbine_100mw() -> Self {
        Self {
            h: 3.0,
            d: 1.0,
            x_d_prime: 0.15,
            e_prime: 1.03,
            freq_hz: 60.0,
            mva_rating: 100.0,
        }
    }

    /// Angular momentum M = 2H / ωs [s²/rad].
    pub fn m(&self) -> f64 {
        let omega_s = 2.0 * std::f64::consts::PI * self.freq_hz;
        2.0 * self.h / omega_s
    }

    /// Synchronous speed [rad/s].
    pub fn omega_s(&self) -> f64 {
        2.0 * std::f64::consts::PI * self.freq_hz
    }
}

/// State of the classical generator model at a point in time.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ClassicalGeneratorState {
    /// Rotor angle [rad]
    pub delta: f64,
    /// Rotor speed deviation from synchronous [rad/s]
    pub omega: f64,
    /// Electrical torque [p.u.]
    pub te: f64,
    /// Mechanical torque input [p.u.]
    pub tm: f64,
    /// Time [s]
    pub time_s: f64,
}

/// Classical generator model integrator.
pub struct ClassicalGenerator {
    pub params: ClassicalGeneratorParams,
    /// Current rotor angle [rad]
    pub delta: f64,
    /// Current rotor speed deviation [rad/s]
    pub omega: f64,
    /// Mechanical torque setpoint [p.u.]
    pub tm: f64,
    /// Simulation time [s]
    pub time_s: f64,
}

impl ClassicalGenerator {
    /// Create a generator at steady-state equilibrium.
    ///
    /// `delta0` — initial rotor angle [rad]
    /// `tm`     — mechanical torque [p.u.] (= steady-state Pe)
    pub fn new(params: ClassicalGeneratorParams, delta0: f64, tm: f64) -> Self {
        Self {
            params,
            delta: delta0,
            omega: 0.0,
            tm,
            time_s: 0.0,
        }
    }

    /// Electrical torque for SMIB configuration.
    ///
    /// Te = E'·V_inf·sin(δ) / X_total
    pub fn te_smib(&self, v_inf: f64, x_total: f64) -> f64 {
        self.params.e_prime * v_inf * self.delta.sin() / x_total
    }

    /// Advance the generator state by dt using RK4.
    ///
    /// `te` — electrical torque [p.u.] at current state.
    pub fn step_rk4(&mut self, te: f64, dt: f64) {
        let m = self.params.m();
        let d = self.params.d;
        let tm = self.tm;

        let f_delta = |omega: f64| -> f64 { omega };
        let f_omega = |omega: f64| -> f64 { (tm - te - d * omega) / m };

        let k1_d = f_delta(self.omega);
        let k1_w = f_omega(self.omega);
        let k2_d = f_delta(self.omega + 0.5 * dt * k1_w);
        let k2_w = f_omega(self.omega + 0.5 * dt * k1_w);
        let k3_d = f_delta(self.omega + 0.5 * dt * k2_w);
        let k3_w = f_omega(self.omega + 0.5 * dt * k2_w);
        let k4_d = f_delta(self.omega + dt * k3_w);
        let k4_w = f_omega(self.omega + dt * k3_w);

        self.delta += dt / 6.0 * (k1_d + 2.0 * k2_d + 2.0 * k3_d + k4_d);
        self.omega += dt / 6.0 * (k1_w + 2.0 * k2_w + 2.0 * k3_w + k4_w);
        self.time_s += dt;
    }

    /// Current state snapshot.
    pub fn state(&self, te: f64) -> ClassicalGeneratorState {
        ClassicalGeneratorState {
            delta: self.delta,
            omega: self.omega,
            te,
            tm: self.tm,
            time_s: self.time_s,
        }
    }

    /// Simulate SMIB response to a fault (zero Te during fault, then recovery).
    ///
    /// Returns list of states at each time step.
    #[allow(clippy::too_many_arguments)]
    pub fn simulate_smib_fault(
        &mut self,
        v_inf: f64,
        x_pre: f64,
        x_during: f64,
        x_post: f64,
        t_fault: f64,
        t_clear: f64,
        t_end: f64,
        dt: f64,
    ) -> Vec<ClassicalGeneratorState> {
        let mut states = Vec::new();
        while self.time_s < t_end {
            let x = if self.time_s < t_fault {
                x_pre
            } else if self.time_s < t_clear {
                x_during
            } else {
                x_post
            };
            let te = self.te_smib(v_inf, x);
            states.push(self.state(te));
            self.step_rk4(te, dt);
        }
        states
    }

    /// Reset to initial conditions.
    pub fn reset(&mut self, delta0: f64) {
        self.delta = delta0;
        self.omega = 0.0;
        self.time_s = 0.0;
    }

    /// Critical clearing angle [rad] for equal-area criterion (SMIB).
    ///
    /// δ_cc = arccos((π − 2·δ_s)·sin(δ_s) − cos(δ_s))
    /// where δ_s is the steady-state rotor angle.
    pub fn critical_clearing_angle_smib(delta_s: f64) -> f64 {
        let arg = (std::f64::consts::PI - 2.0 * delta_s) * delta_s.sin() - delta_s.cos();
        arg.clamp(-1.0, 1.0).acos()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_m_positive() {
        let p = ClassicalGeneratorParams::steam_600mw();
        assert!(p.m() > 0.0);
    }

    #[test]
    fn test_omega_s_60hz() {
        let p = ClassicalGeneratorParams::steam_600mw();
        let expected = 2.0 * std::f64::consts::PI * 60.0;
        assert!((p.omega_s() - expected).abs() < 1e-10);
    }

    #[test]
    fn test_te_smib_at_90deg_max() {
        // At δ = π/2, Te = E'V/X is maximum
        let p = ClassicalGeneratorParams::steam_600mw();
        let mut gen = ClassicalGenerator::new(p.clone(), std::f64::consts::FRAC_PI_2, 0.9);
        let te_90 = gen.te_smib(1.0, 0.3);
        gen.delta = std::f64::consts::FRAC_PI_4;
        let te_45 = gen.te_smib(1.0, 0.3);
        assert!(te_90 > te_45, "Te should be maximum at δ=90°");
    }

    #[test]
    fn test_rk4_stable_equilibrium() {
        // At equilibrium (ω=0, Te=Tm), generator should stay near equilibrium
        let p = ClassicalGeneratorParams::steam_600mw();
        let delta0 = 0.5_f64; // ~28.6°
        let tm = 0.8;
        let v_inf = 1.0;
        let x_total = p.e_prime * v_inf * delta0.sin() / tm;
        let mut gen = ClassicalGenerator::new(p, delta0, tm);
        let te = gen.te_smib(v_inf, x_total);
        for _ in 0..100 {
            gen.step_rk4(te, 0.01);
        }
        // At exact equilibrium with constant Te=Tm, should not diverge
        assert!(gen.delta.is_finite());
        assert!(gen.omega.is_finite());
    }

    #[test]
    fn test_critical_clearing_angle() {
        let delta_s = 0.4; // rad
        let dcc = ClassicalGenerator::critical_clearing_angle_smib(delta_s);
        assert!(
            dcc > delta_s,
            "Critical clearing angle > steady-state angle"
        );
        assert!(dcc < std::f64::consts::PI);
    }

    #[test]
    fn test_simulate_smib_fault_returns_states() {
        let p = ClassicalGeneratorParams::steam_600mw();
        let mut gen = ClassicalGenerator::new(p, 0.5, 0.8);
        let states = gen.simulate_smib_fault(1.0, 0.3, 0.8, 0.35, 0.1, 0.2, 1.0, 0.01);
        assert!(!states.is_empty());
        assert!(states.iter().all(|s| s.delta.is_finite()));
    }
}
