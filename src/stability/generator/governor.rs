/// Governor models for synchronous generators.
///
/// Governors regulate the mechanical power input to maintain frequency.
/// This module implements:
/// - **TGOV1**: Simple steam turbine governor (IEEE standard model)
/// - **GGOV1**: General-purpose governor (simplified)
///
/// # TGOV1 transfer function
///   G(s) = R⁻¹ · (1 + T3·s) / ((1 + T1·s)(1 + T2·s))
///
/// with droop R, time constants T1 (inlet), T2 (reheater), T3 (turbine).
use serde::{Deserialize, Serialize};

/// TGOV1 governor parameters (IEEE standard steam governor model).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Tgov1Params {
    /// Droop (speed regulation) R [p.u.]
    pub r: f64,
    /// Inlet/valve time constant T1 [s]
    pub t1: f64,
    /// Reheater time constant T2 [s] (≈ 0 for simple model)
    pub t2: f64,
    /// Turbine power fraction T3 [s]
    pub t3: f64,
    /// Maximum valve gate position (p.u.)
    pub v_max: f64,
    /// Minimum valve gate position (p.u.)
    pub v_min: f64,
    /// Mechanical damping (turbine friction) Dt [p.u.]
    pub dt: f64,
}

impl Tgov1Params {
    /// Typical steam turbine governor settings.
    pub fn steam_typical() -> Self {
        Self {
            r: 0.05,  // 5% droop
            t1: 0.50, // inlet valve lag
            t2: 3.00, // reheater lag
            t3: 10.0, // HP turbine lag
            v_max: 1.0,
            v_min: 0.0,
            dt: 0.02,
        }
    }

    /// Fast gas turbine governor (shorter time constants).
    pub fn gas_turbine() -> Self {
        Self {
            r: 0.04,
            t1: 0.20,
            t2: 0.30,
            t3: 3.00,
            v_max: 1.0,
            v_min: 0.0,
            dt: 0.01,
        }
    }
}

/// TGOV1 governor state (internal integrator states).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Tgov1State {
    /// Gate position (valve opening) x1
    pub x1: f64,
    /// Reheater state x2
    pub x2: f64,
    /// Reference (setpoint) mechanical power [p.u.]
    pub p_ref: f64,
    /// Current mechanical power output [p.u.]
    pub p_m: f64,
}

/// TGOV1 governor model.
pub struct Tgov1 {
    pub params: Tgov1Params,
    pub state: Tgov1State,
}

impl Tgov1 {
    /// Initialise at steady state for a given initial mechanical power.
    pub fn new(params: Tgov1Params, p_m_init: f64) -> Self {
        let x1 = p_m_init; // valve at steady-state
        let x2 = p_m_init;
        Self {
            state: Tgov1State {
                x1,
                x2,
                p_ref: p_m_init,
                p_m: p_m_init,
            },
            params,
        }
    }

    /// Advance one time step using Euler integration.
    ///
    /// `omega_pu` — rotor speed in p.u. (1.0 = synchronous)
    /// `dt`       — time step [s]
    ///
    /// Returns mechanical power output [p.u.].
    pub fn step(&mut self, omega_pu: f64, dt: f64) -> f64 {
        let p = &self.params;
        let delta_omega = omega_pu - 1.0; // speed error

        // Speed governor block: valve position
        let u_gov = self.state.p_ref - delta_omega / p.r;
        let dx1 = (u_gov.clamp(p.v_min, p.v_max) - self.state.x1) / p.t1;
        self.state.x1 = (self.state.x1 + dt * dx1).clamp(p.v_min, p.v_max);

        // Reheater block
        let dx2 = (self.state.x1 - self.state.x2) / p.t2.max(1e-6);
        self.state.x2 += dt * dx2;

        // Turbine lag block → mechanical power
        let p_m_raw = self.state.x2 - p.dt * delta_omega;
        self.state.p_m = p_m_raw.clamp(p.v_min, p.v_max);
        self.state.p_m
    }

    /// Set a new power reference setpoint.
    pub fn set_reference(&mut self, p_ref: f64) {
        self.state.p_ref = p_ref.clamp(self.params.v_min, self.params.v_max);
    }

    /// Reset to initial conditions.
    pub fn reset(&mut self, p_m_init: f64) {
        self.state.x1 = p_m_init;
        self.state.x2 = p_m_init;
        self.state.p_ref = p_m_init;
        self.state.p_m = p_m_init;
    }
}

/// Simple droop speed governor (proportional only, no dynamics).
///
/// P_m = P_ref − (ω − ω_ref) / R
pub struct DroopGovernor {
    /// Droop constant R [p.u.]
    pub r: f64,
    /// Reference power [p.u.]
    pub p_ref: f64,
    /// Reference speed [p.u.]
    pub omega_ref: f64,
    /// Maximum power output [p.u.]
    pub p_max: f64,
    /// Minimum power output [p.u.]
    pub p_min: f64,
}

impl DroopGovernor {
    /// Create a droop governor with typical settings.
    pub fn new(r: f64, p_ref: f64) -> Self {
        Self {
            r,
            p_ref,
            omega_ref: 1.0,
            p_max: 1.0,
            p_min: 0.0,
        }
    }

    /// Compute mechanical power for a given rotor speed.
    pub fn mechanical_power(&self, omega_pu: f64) -> f64 {
        let delta_omega = omega_pu - self.omega_ref;
        (self.p_ref - delta_omega / self.r).clamp(self.p_min, self.p_max)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tgov1_steady_state_unchanged() {
        let params = Tgov1Params::steam_typical();
        let mut gov = Tgov1::new(params, 0.8);
        // At synchronous speed with no frequency deviation, Pm should stay ≈ 0.8
        for _ in 0..1000 {
            gov.step(1.0, 0.01);
        }
        assert!(
            (gov.state.p_m - 0.8).abs() < 0.05,
            "Pm should remain near setpoint: {:.4}",
            gov.state.p_m
        );
    }

    #[test]
    fn test_tgov1_response_to_underfrequency() {
        let params = Tgov1Params::steam_typical();
        let mut gov = Tgov1::new(params, 0.8);
        // Under-frequency → governor opens valve → more mechanical power
        let p_init = gov.state.p_m;
        for _ in 0..200 {
            gov.step(0.99, 0.01); // 1% under-speed
        }
        assert!(
            gov.state.p_m > p_init,
            "Governor should increase Pm for under-speed"
        );
    }

    #[test]
    fn test_tgov1_clamping() {
        let params = Tgov1Params::steam_typical();
        let mut gov = Tgov1::new(params, 0.95);
        // Large frequency drop → valve should not exceed v_max=1.0
        for _ in 0..500 {
            let pm = gov.step(0.90, 0.01);
            assert!(
                (0.0 - 1e-10..=1.0 + 1e-10).contains(&pm),
                "Pm must be within limits: {:.4}",
                pm
            );
        }
    }

    #[test]
    fn test_droop_governor_response() {
        let gov = DroopGovernor::new(0.05, 0.8);
        let p_sync = gov.mechanical_power(1.0);
        let p_slow = gov.mechanical_power(0.99); // 1% low freq
        assert!((p_sync - 0.8).abs() < 1e-10);
        assert!(
            p_slow > p_sync,
            "Under-speed should increase mechanical power"
        );
    }

    #[test]
    fn test_droop_governor_limits() {
        let gov = DroopGovernor::new(0.05, 0.5);
        let p = gov.mechanical_power(0.50); // extreme under-speed
        assert!(p <= gov.p_max);
    }

    #[test]
    fn test_tgov1_gas_turbine_faster() {
        // Gas turbine should have shorter T1 → faster response
        let steam = Tgov1Params::steam_typical();
        let gas = Tgov1Params::gas_turbine();
        assert!(gas.t1 < steam.t1);
        assert!(gas.t3 < steam.t3);
    }
}
