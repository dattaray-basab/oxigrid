/// IEEE Type 1 Automatic Voltage Regulator (EXDC1 model).
///
/// State equations (Euler or RK4 integration):
///
///   Vf   = Rf − (Kf/Tf)·Efd                [rate feedback]
///   dRf/dt = (−Rf + (Kf/Tf)·Efd) / Tf
///   Ve   = Vref − Vt − Vf                  [voltage error]
///   dVr/dt = (−Vr + KA·Ve) / TA            [amplifier, with limiter]
///   Se   = AE·exp(BE·|Efd|)·sign(Efd)      [saturation]
///   dEfd/dt = (Vr − (KE + Se)·Efd) / TE   [exciter]
///
/// # Reference
/// IEEE Std 421.5-2016 "IEEE Recommended Practice for Excitation System Models",
/// Model EXDC1 (identical to DC1A in older standards).
use serde::{Deserialize, Serialize};

/// Parameters for the IEEE Type 1 DC exciter (EXDC1 / DC1A).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Avr1Params {
    /// Voltage regulator gain KA [p.u./p.u.]
    pub ka: f64,
    /// Voltage regulator time constant TA `s`
    pub ta: f64,
    /// Exciter gain KE [p.u.]
    pub ke: f64,
    /// Exciter time constant TE `s`
    pub te: f64,
    /// Rate feedback gain KF [p.u.]
    pub kf: f64,
    /// Rate feedback time constant TF `s`
    pub tf: f64,
    /// Maximum regulator output Vrmax [p.u.]
    pub vr_max: f64,
    /// Minimum regulator output Vrmin [p.u.]
    pub vr_min: f64,
    /// Saturation coefficient AE [p.u.]
    pub a_e: f64,
    /// Saturation coefficient BE [1/p.u.]
    pub b_e: f64,
}

impl Avr1Params {
    /// Typical fast-acting DC exciter for a steam turbine generator.
    pub fn steam_typical() -> Self {
        Self {
            ka: 46.0,
            ta: 0.06,
            ke: -0.047,
            te: 0.46,
            kf: 0.1,
            tf: 1.0,
            vr_max: 1.0,
            vr_min: -0.95,
            a_e: 0.0039,
            b_e: 1.555,
        }
    }

    /// Slow-response DC exciter for a hydro unit.
    pub fn hydro_slow() -> Self {
        Self {
            ka: 20.0,
            ta: 0.20,
            ke: 1.0,
            te: 0.95,
            kf: 0.06,
            tf: 1.0,
            vr_max: 3.5,
            vr_min: 0.0,
            a_e: 0.0,
            b_e: 0.0,
        }
    }

    /// Saturation function Se(Efd) = AE · exp(BE · Efd) for Efd > 0, else 0.
    ///
    /// In the EXDC1 model saturation is only defined for positive field voltage.
    pub fn saturation(&self, efd: f64) -> f64 {
        if self.a_e < 1e-12 || efd <= 0.0 {
            return 0.0;
        }
        self.a_e * (self.b_e * efd).exp()
    }
}

/// State variables for the IEEE Type 1 AVR.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Avr1State {
    /// Field voltage output Efd [p.u.]
    pub efd: f64,
    /// Amplifier output Vr [p.u.]
    pub vr: f64,
    /// Rate feedback state Rf [p.u.]
    pub rf: f64,
    /// Voltage reference Vref [p.u.]
    pub vref: f64,
    /// Current simulation time `s`
    pub time_s: f64,
}

impl Avr1State {
    /// Initialise AVR at a steady-state operating point.
    ///
    /// Given terminal voltage `vt` and field voltage `efd0`, derive
    /// the steady-state Vr and Rf.
    pub fn from_steady_state(efd0: f64, vt: f64, params: &Avr1Params) -> Self {
        // At steady state dRf/dt = 0 → Rf = (Kf/Tf)*Efd
        let rf0 = (params.kf / params.tf) * efd0;
        // At steady state Vf = Rf - (Kf/Tf)*Efd = 0
        // dVr/dt = 0 → Vr = KA*(Vref - Vt - Vf) = KA*(Vref - Vt)
        // dEfd/dt = 0 → Vr = (KE + Se)*Efd
        let se0 = params.saturation(efd0);
        let vr0 = (params.ke + se0) * efd0;
        // Infer Vref: Vr = KA*(Vref - Vt) → Vref = Vt + Vr/KA
        let vref0 = vt + vr0 / params.ka;
        Self {
            efd: efd0,
            vr: vr0,
            rf: rf0,
            vref: vref0,
            time_s: 0.0,
        }
    }

    /// Compute state derivatives [dEfd/dt, dVr/dt, dRf/dt].
    fn derivatives(&self, vt: f64, params: &Avr1Params) -> (f64, f64, f64) {
        let vf = self.rf - (params.kf / params.tf) * self.efd;
        let ve = self.vref - vt - vf;
        let se = params.saturation(self.efd);

        let defd_dt = (self.vr - (params.ke + se) * self.efd) / params.te;
        let dvr_dt_unclamped = (-self.vr + params.ka * ve) / params.ta;
        // Apply limiter: if Vr at limit and derivative would push further, clamp
        let at_limit = (self.vr >= params.vr_max && dvr_dt_unclamped > 0.0)
            || (self.vr <= params.vr_min && dvr_dt_unclamped < 0.0);
        let dvr_dt = if at_limit { 0.0 } else { dvr_dt_unclamped };
        let drf_dt = (-self.rf + (params.kf / params.tf) * self.efd) / params.tf;

        (defd_dt, dvr_dt, drf_dt)
    }

    /// Advance by one time step using RK4.
    ///
    /// `vt` is the terminal voltage magnitude [p.u.] (assumed constant over step).
    pub fn step_rk4(&mut self, vt: f64, dt: f64, params: &Avr1Params) {
        let (k1e, k1r, k1f) = self.derivatives(vt, params);

        let s2 = Avr1State {
            efd: self.efd + 0.5 * dt * k1e,
            vr: (self.vr + 0.5 * dt * k1r).clamp(params.vr_min, params.vr_max),
            rf: self.rf + 0.5 * dt * k1f,
            ..*self
        };
        let (k2e, k2r, k2f) = s2.derivatives(vt, params);

        let s3 = Avr1State {
            efd: self.efd + 0.5 * dt * k2e,
            vr: (self.vr + 0.5 * dt * k2r).clamp(params.vr_min, params.vr_max),
            rf: self.rf + 0.5 * dt * k2f,
            ..*self
        };
        let (k3e, k3r, k3f) = s3.derivatives(vt, params);

        let s4 = Avr1State {
            efd: self.efd + dt * k3e,
            vr: (self.vr + dt * k3r).clamp(params.vr_min, params.vr_max),
            rf: self.rf + dt * k3f,
            ..*self
        };
        let (k4e, k4r, k4f) = s4.derivatives(vt, params);

        self.efd += dt / 6.0 * (k1e + 2.0 * k2e + 2.0 * k3e + k4e);
        self.vr = (self.vr + dt / 6.0 * (k1r + 2.0 * k2r + 2.0 * k3r + k4r))
            .clamp(params.vr_min, params.vr_max);
        self.rf += dt / 6.0 * (k1f + 2.0 * k2f + 2.0 * k3f + k4f);
        self.time_s += dt;
    }

    /// Simulate AVR response to a terminal voltage step.
    ///
    /// Returns `(times, efd_trace)` for plotting.
    pub fn simulate_voltage_step(
        efd0: f64,
        vt_initial: f64,
        vt_final: f64,
        t_step: f64,
        t_end: f64,
        dt: f64,
        params: &Avr1Params,
    ) -> (Vec<f64>, Vec<f64>) {
        let mut state = Avr1State::from_steady_state(efd0, vt_initial, params);
        let mut times = Vec::new();
        let mut efds = Vec::new();

        while state.time_s <= t_end {
            let vt = if state.time_s >= t_step {
                vt_final
            } else {
                vt_initial
            };
            times.push(state.time_s);
            efds.push(state.efd);
            state.step_rk4(vt, dt, params);
        }
        (times, efds)
    }
}

/// Combined AVR + Generator (SMIB context).
///
/// Holds AVR params and state together for convenience.
pub struct AvrGenerator {
    pub params: Avr1Params,
    pub state: Avr1State,
}

impl AvrGenerator {
    /// Create from steady-state operating point.
    pub fn new(params: Avr1Params, efd0: f64, vt0: f64) -> Self {
        let state = Avr1State::from_steady_state(efd0, vt0, &params);
        Self { params, state }
    }

    /// Step AVR given terminal voltage.
    pub fn step(&mut self, vt: f64, dt: f64) {
        self.state.step_rk4(vt, dt, &self.params);
    }

    /// Current field voltage [p.u.].
    pub fn efd(&self) -> f64 {
        self.state.efd
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_steady_state_initialisation() {
        let params = Avr1Params::steam_typical();
        let efd0 = 1.2;
        let vt0 = 1.0;
        let state = Avr1State::from_steady_state(efd0, vt0, &params);
        // Verify: at t=0, derivatives should be near zero
        let (de, dv, dr) = state.derivatives(vt0, &params);
        assert!(de.abs() < 1e-6, "dEfd/dt = {de:.2e} at steady state");
        assert!(dv.abs() < 1e-6, "dVr/dt = {dv:.2e} at steady state");
        assert!(dr.abs() < 1e-9, "dRf/dt = {dr:.2e} at steady state");
    }

    #[test]
    fn test_voltage_step_response() {
        let params = Avr1Params::steam_typical();
        // Step up voltage: AVR should reduce Efd to bring V back down
        let (times, efds) =
            Avr1State::simulate_voltage_step(1.2, 1.0, 1.05, 0.5, 5.0, 0.01, &params);
        assert!(!times.is_empty());
        // Efd should change in response to the voltage step
        let efd_initial = efds[0];
        let efd_final = *efds.last().unwrap();
        // A voltage increase should cause Efd to decrease
        assert!(
            efd_final < efd_initial,
            "Efd should decrease for voltage increase: {efd_initial:.4} → {efd_final:.4}"
        );
    }

    #[test]
    fn test_avr_generator_step() {
        let params = Avr1Params::hydro_slow();
        let mut avr = AvrGenerator::new(params, 1.1, 1.0);
        let efd_init = avr.efd();
        // Simulate 1 second
        for _ in 0..100 {
            avr.step(1.0, 0.01);
        }
        // Should remain near steady state with constant Vt
        assert!(
            (avr.efd() - efd_init).abs() < 0.01,
            "Efd drifted from SS: {efd_init:.4} → {:.4}",
            avr.efd()
        );
    }

    #[test]
    fn test_saturation_function() {
        let params = Avr1Params::steam_typical();
        // Se(0) = 0 (for small Efd values, saturation is minimal)
        assert!(params.saturation(0.0).abs() < 0.01);
        // Se grows with Efd
        assert!(params.saturation(2.0) > params.saturation(1.0));
    }

    #[test]
    fn test_vr_limiter() {
        let params = Avr1Params::steam_typical();
        // Start with Vr at max; a further increase should be blocked
        let state = Avr1State {
            efd: 1.2,
            vr: params.vr_max,
            rf: 0.12,
            vref: 1.0,
            time_s: 0.0,
        };
        // With Vt very low, Ve is large positive → dVr/dt would be > 0, clamped
        let (_, dvr, _) = state.derivatives(0.5, &params);
        assert!(
            dvr <= 0.0,
            "Vr limiter should block positive dVr when at max: dvr = {dvr:.4}"
        );
    }
}
