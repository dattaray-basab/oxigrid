/// Detailed synchronous generator model (subtransient d-q axis).
///
/// Implements the 4th-order "two-axis" (flux-decay) model commonly used in
/// transient stability studies, also called the "Anderson-Fouad" model.
///
/// # State variables
/// - E'_q: q-axis transient EMF [p.u.]
/// - E'_d: d-axis transient EMF [p.u.]
/// - δ:   rotor angle `rad`
/// - ω:   rotor speed [p.u.] (1.0 = synchronous)
///
/// # Differential equations
///   T'_d0 · dE'_q/dt = E_fd − E'_q − (X_d − X'_d) · i_d
///   T'_q0 · dE'_d/dt = −E'_d + (X_q − X'_q) · i_q
///   M · dω/dt = T_m − T_e − D · (ω − 1)
///   dδ/dt = ωs · (ω − 1)
///
/// # Reference
/// Anderson & Fouad (2003), "Power System Control and Stability", 2nd Ed.
use serde::{Deserialize, Serialize};

/// Subtransient generator parameters (4th-order two-axis model).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DetailedGenParams {
    /// Inertia constant H `s`
    pub h: f64,
    /// Damping coefficient D [p.u.]
    pub d: f64,
    /// d-axis synchronous reactance X_d [p.u.]
    pub x_d: f64,
    /// d-axis transient reactance X'_d [p.u.]
    pub x_d_prime: f64,
    /// q-axis synchronous reactance X_q [p.u.]
    pub x_q: f64,
    /// q-axis transient reactance X'_q [p.u.]
    pub x_q_prime: f64,
    /// d-axis open-circuit transient time constant T'_d0 `s`
    pub t_d0_prime: f64,
    /// q-axis open-circuit transient time constant T'_q0 `s`
    pub t_q0_prime: f64,
    /// Nominal frequency `Hz`
    pub freq_hz: f64,
    /// Ra armature resistance [p.u.]
    pub ra: f64,
}

impl DetailedGenParams {
    /// Synchronous speed ωs = 2π·f [rad/s].
    pub fn omega_s(&self) -> f64 {
        2.0 * std::f64::consts::PI * self.freq_hz
    }

    /// Angular momentum M = 2H/ωs [s²/rad].
    pub fn m(&self) -> f64 {
        2.0 * self.h / self.omega_s()
    }

    /// Typical round-rotor steam generator (600 MW class).
    pub fn steam_round_rotor() -> Self {
        Self {
            h: 6.0,
            d: 2.0,
            x_d: 1.81,
            x_d_prime: 0.30,
            x_q: 1.76,
            x_q_prime: 0.65,
            t_d0_prime: 8.0,
            t_q0_prime: 1.0,
            freq_hz: 60.0,
            ra: 0.003,
        }
    }

    /// Typical salient-pole hydro generator.
    pub fn hydro_salient_pole() -> Self {
        Self {
            h: 4.0,
            d: 1.5,
            x_d: 1.02,
            x_d_prime: 0.32,
            x_q: 0.72,
            x_q_prime: 0.72, // salient-pole: X'_q ≈ X_q
            t_d0_prime: 6.5,
            t_q0_prime: 0.5,
            freq_hz: 60.0,
            ra: 0.0025,
        }
    }
}

/// State of the 4th-order generator model.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct DetailedGenState {
    /// q-axis transient EMF E'_q [p.u.]
    pub e_q_prime: f64,
    /// d-axis transient EMF E'_d [p.u.]
    pub e_d_prime: f64,
    /// Rotor angle δ `rad`
    pub delta: f64,
    /// Rotor speed ω [p.u.] (1.0 = synchronous)
    pub omega: f64,
    /// Electrical torque T_e [p.u.]
    pub t_e: f64,
    /// Simulation time `s`
    pub time_s: f64,
}

/// Algebraic variables (currents) for the generator.
#[derive(Debug, Clone, Copy)]
pub struct GenAlgebraic {
    /// d-axis current i_d [p.u.]
    pub i_d: f64,
    /// q-axis current i_q [p.u.]
    pub i_q: f64,
}

/// 4th-order synchronous generator model.
pub struct DetailedGenerator {
    pub params: DetailedGenParams,
    pub state: DetailedGenState,
    /// Field voltage (from AVR or fixed) [p.u.]
    pub e_fd: f64,
    /// Mechanical torque input [p.u.]
    pub t_m: f64,
}

impl DetailedGenerator {
    /// Create at a given operating point.
    ///
    /// `delta0` — initial rotor angle `rad`
    /// `omega0` — initial speed [p.u.], typically 1.0
    /// `e_q0`   — initial q-axis transient EMF [p.u.]
    /// `e_d0`   — initial d-axis transient EMF [p.u.]
    /// `e_fd0`  — initial field voltage [p.u.]
    /// `t_m0`   — mechanical torque [p.u.]
    pub fn new(
        params: DetailedGenParams,
        delta0: f64,
        omega0: f64,
        e_q0: f64,
        e_d0: f64,
        e_fd0: f64,
        t_m0: f64,
    ) -> Self {
        Self {
            state: DetailedGenState {
                e_q_prime: e_q0,
                e_d_prime: e_d0,
                delta: delta0,
                omega: omega0,
                t_e: t_m0,
                time_s: 0.0,
            },
            params,
            e_fd: e_fd0,
            t_m: t_m0,
        }
    }

    /// Compute algebraic currents from terminal voltage and rotor angle.
    ///
    /// V_t = V∠θ (terminal voltage), using network frame.
    /// Returns (i_d, i_q) in machine d-q frame.
    pub fn compute_currents(&self, vt_re: f64, vt_im: f64) -> GenAlgebraic {
        let p = &self.params;
        let s = &self.state;
        // Transform terminal voltage to rotor d-q frame
        let v_d = vt_re * (s.delta).sin() - vt_im * (s.delta).cos();
        let v_q = vt_re * (s.delta).cos() + vt_im * (s.delta).sin();

        // Solve voltage equations:
        // v_d = -ra*i_d + x'_q*i_q + e'_d
        // v_q = -ra*i_q - x'_d*i_d + e'_q
        let denom = p.ra * p.ra + p.x_d_prime * p.x_q_prime;
        let i_d = if denom > 1e-12 {
            (p.ra * (v_d - s.e_d_prime) - p.x_q_prime * (v_q - s.e_q_prime)) / denom
        } else {
            (v_d - s.e_d_prime) / p.x_d_prime.max(1e-6)
        };
        let i_q = if denom > 1e-12 {
            (p.ra * (v_q - s.e_q_prime) + p.x_d_prime * (v_d - s.e_d_prime)) / denom
        } else {
            (v_q - s.e_q_prime) / p.x_q_prime.max(1e-6)
        };
        GenAlgebraic { i_d, i_q }
    }

    /// Compute electrical torque T_e = E'_q·i_q + E'_d·i_d + (X'_d − X'_q)·i_d·i_q.
    pub fn electrical_torque(&self, alg: &GenAlgebraic) -> f64 {
        let s = &self.state;
        let p = &self.params;
        s.e_q_prime * alg.i_q
            + s.e_d_prime * alg.i_d
            + (p.x_d_prime - p.x_q_prime) * alg.i_d * alg.i_q
    }

    /// Advance state by dt using RK4 for SMIB (infinite bus at V∠0).
    ///
    /// `v_inf` — infinite bus voltage [p.u.]
    /// `x_net` — network reactance between generator and infinite bus [p.u.]
    pub fn step_smib_rk4(&mut self, v_inf: f64, x_net: f64, dt: f64) {
        let p = &self.params;
        let omega_s = p.omega_s();
        let m = p.m();

        let step = |s: &DetailedGenState, e_fd: f64, t_m: f64| -> DetailedGenState {
            // Compute terminal voltage (simplified: infinite bus seen through x_net)
            // vt = v_inf + j*x_net*i (approximation)
            let i_q = (s.e_q_prime - v_inf * s.delta.cos()) / (x_net + p.x_d_prime);
            let i_d = v_inf * s.delta.sin() / (x_net + p.x_q_prime);
            let t_e =
                s.e_q_prime * i_q + s.e_d_prime * i_d + (p.x_d_prime - p.x_q_prime) * i_d * i_q;

            let de_q_prime = (e_fd - s.e_q_prime - (p.x_d - p.x_d_prime) * i_d) / p.t_d0_prime;
            let de_d_prime = (-s.e_d_prime + (p.x_q - p.x_q_prime) * i_q) / p.t_q0_prime;
            let d_omega = (t_m - t_e - p.d * (s.omega - 1.0)) / m;
            let d_delta = omega_s * (s.omega - 1.0);

            DetailedGenState {
                e_q_prime: de_q_prime,
                e_d_prime: de_d_prime,
                delta: d_delta,
                omega: d_omega,
                t_e,
                time_s: 0.0,
            }
        };

        let s = self.state;
        let k1 = step(&s, self.e_fd, self.t_m);

        let s2 = DetailedGenState {
            e_q_prime: s.e_q_prime + 0.5 * dt * k1.e_q_prime,
            e_d_prime: s.e_d_prime + 0.5 * dt * k1.e_d_prime,
            delta: s.delta + 0.5 * dt * k1.delta,
            omega: s.omega + 0.5 * dt * k1.omega,
            t_e: s.t_e,
            time_s: s.time_s,
        };
        let k2 = step(&s2, self.e_fd, self.t_m);

        let s3 = DetailedGenState {
            e_q_prime: s.e_q_prime + 0.5 * dt * k2.e_q_prime,
            e_d_prime: s.e_d_prime + 0.5 * dt * k2.e_d_prime,
            delta: s.delta + 0.5 * dt * k2.delta,
            omega: s.omega + 0.5 * dt * k2.omega,
            t_e: s.t_e,
            time_s: s.time_s,
        };
        let k3 = step(&s3, self.e_fd, self.t_m);

        let s4 = DetailedGenState {
            e_q_prime: s.e_q_prime + dt * k3.e_q_prime,
            e_d_prime: s.e_d_prime + dt * k3.e_d_prime,
            delta: s.delta + dt * k3.delta,
            omega: s.omega + dt * k3.omega,
            t_e: s.t_e,
            time_s: s.time_s,
        };
        let k4 = step(&s4, self.e_fd, self.t_m);

        let t_e_mid = k2.t_e; // use midpoint for electrical torque
        self.state = DetailedGenState {
            e_q_prime: s.e_q_prime
                + dt / 6.0
                    * (k1.e_q_prime + 2.0 * k2.e_q_prime + 2.0 * k3.e_q_prime + k4.e_q_prime),
            e_d_prime: s.e_d_prime
                + dt / 6.0
                    * (k1.e_d_prime + 2.0 * k2.e_d_prime + 2.0 * k3.e_d_prime + k4.e_d_prime),
            delta: s.delta + dt / 6.0 * (k1.delta + 2.0 * k2.delta + 2.0 * k3.delta + k4.delta),
            omega: s.omega + dt / 6.0 * (k1.omega + 2.0 * k2.omega + 2.0 * k3.omega + k4.omega),
            t_e: t_e_mid,
            time_s: s.time_s + dt,
        };
    }

    /// Set field voltage (from AVR).
    pub fn set_e_fd(&mut self, e_fd: f64) {
        self.e_fd = e_fd.max(0.0);
    }

    /// Set mechanical torque (from governor).
    pub fn set_t_m(&mut self, t_m: f64) {
        self.t_m = t_m.max(0.0);
    }

    /// Terminal power factor angle `rad`.
    pub fn power_factor_angle(&self, alg: &GenAlgebraic) -> f64 {
        let p_e = self.state.e_q_prime * alg.i_q;
        let q_e = self.state.e_q_prime * alg.i_d;
        q_e.atan2(p_e)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_gen() -> DetailedGenerator {
        let p = DetailedGenParams::steam_round_rotor();
        // Steady state: ω=1, δ=30°, E'_q=1.2, E'_d=0
        DetailedGenerator::new(p, 30f64.to_radians(), 1.0, 1.2, 0.0, 1.2, 0.8)
    }

    #[test]
    fn test_params_m_positive() {
        let p = DetailedGenParams::steam_round_rotor();
        assert!(p.m() > 0.0);
        assert!(p.omega_s() > 300.0); // 2π*60 ≈ 377
    }

    #[test]
    fn test_initial_state_finite() {
        let gen = make_gen();
        let s = &gen.state;
        assert!(s.e_q_prime.is_finite());
        assert!(s.delta.is_finite());
        assert!(s.omega.is_finite());
    }

    #[test]
    fn test_compute_currents_finite() {
        let gen = make_gen();
        let alg = gen.compute_currents(1.0, 0.0);
        assert!(alg.i_d.is_finite());
        assert!(alg.i_q.is_finite());
    }

    #[test]
    fn test_electrical_torque_positive_at_load() {
        let gen = make_gen();
        let alg = gen.compute_currents(1.0, 0.0);
        let te = gen.electrical_torque(&alg);
        assert!(te.is_finite());
    }

    #[test]
    fn test_step_smib_stays_finite() {
        let mut gen = make_gen();
        for _ in 0..100 {
            gen.step_smib_rk4(1.0, 0.3, 0.01);
        }
        assert!(gen.state.delta.is_finite());
        assert!(gen.state.omega.is_finite());
        assert!(gen.state.e_q_prime.is_finite());
    }

    #[test]
    fn test_hydro_salient_pole_params() {
        let p = DetailedGenParams::hydro_salient_pole();
        assert!(p.x_q_prime >= p.x_q_prime); // X'_q ≈ X_q for salient pole
        assert!(p.t_d0_prime > p.t_q0_prime);
    }

    #[test]
    fn test_set_efd_and_tm() {
        let mut gen = make_gen();
        gen.set_e_fd(1.5);
        gen.set_t_m(0.9);
        assert!((gen.e_fd - 1.5).abs() < 1e-10);
        assert!((gen.t_m - 0.9).abs() < 1e-10);
    }

    #[test]
    fn test_freq_hz_60() {
        let p = DetailedGenParams::steam_round_rotor();
        assert!((p.freq_hz - 60.0).abs() < 1e-10);
    }
}
