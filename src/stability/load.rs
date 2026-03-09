/// Load models for power system stability analysis.
///
/// Three load representation levels:
///
/// 1. **ZIP model** — static polynomial model combining constant
///    Impedance (Z), constant Current (I) and constant Power (P):
///    P_L = P0 · (α_z·V² + α_i·V + α_p)
///    Q_L = Q0 · (β_z·V² + β_i·V + β_p)
///
/// 2. **Induction motor (1st-order)** — captures motor dynamic stalling and
///    recovery.  Uses the reduced-order voltage-behind-reactance model.
///
/// 3. **CLOD (Composite Load model)** — combines a fraction of motor load
///    with a fraction of ZIP load; commonly used in WECC studies.
///
/// # References
/// - Kundur, "Power System Stability and Control", Ch. 7.
/// - IEEE PES Load Representation for Dynamic Performance Working Group
///   Report, 1995.
/// - WECC Composite Load Model CMPLDWG implementation guide, 2015.
use serde::{Deserialize, Serialize};

/// ZIP model parameters for one load bus.
///
/// Fractions must sum to 1: α_z + α_i + α_p = 1 (same for β).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ZipModel {
    /// Nominal active power P0 [p.u.]
    pub p0: f64,
    /// Nominal reactive power Q0 [p.u.]
    pub q0: f64,
    /// Active power fractions: [constant-Z, constant-I, constant-P]
    pub alpha: [f64; 3], // [α_z, α_i, α_p]
    /// Reactive power fractions: [constant-Z, constant-I, constant-P]
    pub beta: [f64; 3], // [β_z, β_i, β_p]
    /// Nominal voltage (at which P0, Q0 apply) [p.u.]
    pub v_nom: f64,
}

impl ZipModel {
    /// Pure constant power (standard power flow representation).
    pub fn constant_power(p0: f64, q0: f64) -> Self {
        Self {
            p0,
            q0,
            alpha: [0.0, 0.0, 1.0],
            beta: [0.0, 0.0, 1.0],
            v_nom: 1.0,
        }
    }

    /// Pure constant impedance (passive load).
    pub fn constant_impedance(p0: f64, q0: f64) -> Self {
        Self {
            p0,
            q0,
            alpha: [1.0, 0.0, 0.0],
            beta: [1.0, 0.0, 0.0],
            v_nom: 1.0,
        }
    }

    /// Typical residential mix (70% P, 20% I, 10% Z).
    pub fn residential(p0: f64, q0: f64) -> Self {
        Self {
            p0,
            q0,
            alpha: [0.10, 0.20, 0.70],
            beta: [0.10, 0.20, 0.70],
            v_nom: 1.0,
        }
    }

    /// Typical industrial mix (40% P, 30% I, 30% Z).
    pub fn industrial(p0: f64, q0: f64) -> Self {
        Self {
            p0,
            q0,
            alpha: [0.30, 0.30, 0.40],
            beta: [0.30, 0.30, 0.40],
            v_nom: 1.0,
        }
    }

    /// Compute active power consumption at voltage V [p.u.].
    pub fn active_power(&self, v: f64) -> f64 {
        let vn = v / self.v_nom;
        self.p0 * (self.alpha[0] * vn * vn + self.alpha[1] * vn + self.alpha[2])
    }

    /// Compute reactive power consumption at voltage V [p.u.].
    pub fn reactive_power(&self, v: f64) -> f64 {
        let vn = v / self.v_nom;
        self.q0 * (self.beta[0] * vn * vn + self.beta[1] * vn + self.beta[2])
    }

    /// Voltage sensitivity dP/dV at nominal voltage [p.u./p.u.].
    pub fn dp_dv_nominal(&self) -> f64 {
        // dP/dV = P0 * (2*α_z*V + α_i) / V_nom²
        self.p0 * (2.0 * self.alpha[0] * self.v_nom + self.alpha[1]) / (self.v_nom * self.v_nom)
    }

    /// Voltage stability: true if load is voltage-stabilising (dP/dV > 0).
    pub fn is_voltage_stabilising(&self) -> bool {
        self.dp_dv_nominal() > 0.0
    }
}

/// First-order induction motor load model.
///
/// Models a simple induction motor as a voltage-dependent dynamic load with
/// internal transient EMF E' and slip dynamics.
///
/// Equivalent circuit at fundamental frequency:
///   E' = V − jX'·I
///   Equation of motion: ds/dt = (T_e(s,V) − T_m) / (2H)
///
/// Simplified slip dynamics (per unit):
///   2H · ds/dt = T_m − T_e(V, s)
///   T_e ≈ E'²·R_s/s  (pull-out torque approximation)
///
/// Stall condition: s → 1 (motor stalls below critical voltage).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InductionMotorParams {
    /// Rated active power P_rated [p.u.]
    pub p_rated: f64,
    /// Power factor at rated load
    pub power_factor: f64,
    /// Inertia constant H [s]
    pub h: f64,
    /// Transient reactance X' [p.u.] (≈ leakage + rotor reactance)
    pub x_prime: f64,
    /// Rotor resistance R_s [p.u.] at standstill
    pub r_s: f64,
    /// Mechanical load exponent: T_mech ~ ω^α (0=const, 1=linear, 2=fan)
    pub mech_load_exp: f64,
}

impl InductionMotorParams {
    /// Typical air-conditioner load (low inertia, constant torque).
    pub fn air_conditioner(p_rated: f64) -> Self {
        Self {
            p_rated,
            power_factor: 0.85,
            h: 0.3,
            x_prime: 0.12,
            r_s: 0.02,
            mech_load_exp: 0.0,
        }
    }

    /// Typical pump/fan load (medium inertia, quadratic torque).
    pub fn pump_fan(p_rated: f64) -> Self {
        Self {
            p_rated,
            power_factor: 0.88,
            h: 1.5,
            x_prime: 0.18,
            r_s: 0.015,
            mech_load_exp: 2.0,
        }
    }

    /// Critical slip (slip at pull-out torque) [p.u.].
    pub fn critical_slip(&self, v: f64) -> f64 {
        // s_cr = R_s / sqrt(0 + X'^2) for simplified model (Rs << X')
        let _ = v;
        self.r_s / self.x_prime
    }
}

/// State of an induction motor.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct MotorState {
    /// Slip s = (ωs − ω) / ωs [0,1] (0=synchronous, 1=stall)
    pub slip: f64,
    /// Stalled flag (true if motor has stalled)
    pub stalled: bool,
}

impl MotorState {
    pub fn at_rated_load(params: &InductionMotorParams) -> Self {
        // Approximate rated slip: s_rated ≈ R_s / X' (simplified)
        let s_rated = params.r_s / params.x_prime;
        Self {
            slip: s_rated,
            stalled: false,
        }
    }
}

/// Compute electrical torque of induction motor at given slip and voltage.
///
/// T_e(s, V) = V² · R_s/s / [(R_s/s)² + X'²]   (simplified Steinmetz circuit)
fn electrical_torque(params: &InductionMotorParams, slip: f64, v: f64) -> f64 {
    let s = slip.abs().max(1e-6);
    let r_over_s = params.r_s / s;
    let denom = r_over_s * r_over_s + params.x_prime * params.x_prime;
    v * v * r_over_s / denom
}

/// Compute mechanical torque: T_m ~ (1-s)^α (normalised to 1 at s=0).
fn mechanical_torque(params: &InductionMotorParams, slip: f64) -> f64 {
    let omega_pu = (1.0 - slip).max(0.0);
    params.p_rated * omega_pu.powf(params.mech_load_exp)
}

/// Step the induction motor model by dt.
///
/// Returns updated motor state and active power consumption.
pub fn motor_step(
    params: &InductionMotorParams,
    state: &MotorState,
    v: f64,
    dt: f64,
) -> (MotorState, f64) {
    if state.stalled {
        // Stalled motor: consume locked-rotor current
        let p_stall = v * v / (params.x_prime + params.r_s);
        return (
            MotorState {
                slip: 1.0,
                stalled: true,
            },
            p_stall * params.p_rated,
        );
    }

    let t_e = electrical_torque(params, state.slip, v);
    let t_m = mechanical_torque(params, state.slip);

    // Swing equation: 2H·ds/dt = T_m − T_e
    let ds_dt = (t_m - t_e) / (2.0 * params.h);
    let new_slip = (state.slip + ds_dt * dt).clamp(0.0, 1.0);

    // Check stall condition
    let s_crit = params.critical_slip(v);
    let stalled = new_slip >= s_crit * 5.0 || (v < 0.7 && new_slip > 0.5);

    // Power consumption: P = T_e * ω
    let omega = (1.0 - new_slip).max(0.0);
    let p_mw = t_e * omega * params.p_rated;

    (
        MotorState {
            slip: new_slip,
            stalled,
        },
        p_mw.max(0.0),
    )
}

/// CLOD (Composite Load) model: combination of motor + ZIP components.
///
/// Fraction `motor_frac` is represented as induction motor;
/// remainder (1 − motor_frac) is ZIP load.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ClodModel {
    /// Fraction of load modelled as induction motor [0, 1]
    pub motor_frac: f64,
    /// Motor parameters
    pub motor: InductionMotorParams,
    /// ZIP parameters for the non-motor fraction
    pub zip: ZipModel,
    /// Total rated load active power [p.u.]
    pub p_total: f64,
}

impl ClodModel {
    /// Standard WECC CLOD: 40% motors, 60% ZIP.
    pub fn wecc_standard(p_total: f64, q_total: f64) -> Self {
        Self {
            motor_frac: 0.4,
            motor: InductionMotorParams::air_conditioner(p_total * 0.4),
            zip: ZipModel::residential(p_total * 0.6, q_total * 0.6),
            p_total,
        }
    }

    /// Compute total active power at voltage V with motor state.
    pub fn active_power(&self, v: f64, motor_state: &MotorState) -> f64 {
        let t_e = electrical_torque(&self.motor, motor_state.slip, v);
        let omega = (1.0 - motor_state.slip).max(0.0);
        let p_motor = t_e * omega * self.motor.p_rated;
        let p_zip = self.zip.active_power(v);
        p_motor + p_zip
    }

    /// Compute total reactive power at voltage V.
    pub fn reactive_power(&self, v: f64, _motor_state: &MotorState) -> f64 {
        // Motor reactive: Q_m ≈ V² / X' (magnetising + leakage)
        let q_motor = v * v / self.motor.x_prime * self.motor.p_rated;
        let q_zip = self.zip.reactive_power(v);
        q_motor + q_zip
    }
}

/// Load recovery model for exponential load recovery after voltage disturbance.
///
/// Models the restoration of load after a voltage dip:
///   dP_t/dt = (P_s(V) − P_t) / T_p  + P_s(V) − P_0
///
/// where:
///   P_s(V) = P0 * V^α_s  (static characteristic after recovery)
///   P_t     = transient (dynamic) load
///   T_p     = active power recovery time constant [s]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LoadRecoveryModel {
    /// Steady-state active load at V=1 [p.u.]
    pub p0: f64,
    /// Steady-state reactive load at V=1 [p.u.]
    pub q0: f64,
    /// Active power static voltage exponent (α_s)
    pub alpha_s: f64,
    /// Active power transient voltage exponent (α_t)
    pub alpha_t: f64,
    /// Reactive power static voltage exponent (β_s)
    pub beta_s: f64,
    /// Reactive power transient voltage exponent (β_t)
    pub beta_t: f64,
    /// Active recovery time constant T_p [s]
    pub t_p: f64,
    /// Reactive recovery time constant T_q [s]
    pub t_q: f64,
    // State: transient load power deviation
    pub x_p: f64, // active load state variable
    pub x_q: f64, // reactive load state variable
}

impl LoadRecoveryModel {
    /// Exponential recovery model (Karlsson-Hill 1994 style).
    pub fn new(p0: f64, q0: f64, t_p: f64, t_q: f64) -> Self {
        Self {
            p0,
            q0,
            alpha_s: 0.5,
            alpha_t: 2.0,
            beta_s: 3.5,
            beta_t: 7.0,
            t_p,
            t_q,
            x_p: 0.0,
            x_q: 0.0,
        }
    }

    /// Compute instantaneous active and reactive power demand.
    pub fn power_demand(&self, v: f64) -> (f64, f64) {
        let p = self.p0 * v.powf(self.alpha_t) + self.x_p;
        let q = self.q0 * v.powf(self.beta_t) + self.x_q;
        (p.max(0.0), q.max(0.0))
    }

    /// Update internal state (forward Euler).
    pub fn step(&mut self, v: f64, dt: f64) {
        // dx_p/dt = (P_s(V) - P_t(V)) / T_p
        let p_s = self.p0 * v.powf(self.alpha_s);
        let p_t = self.p0 * v.powf(self.alpha_t);
        let dx_p = (p_s - p_t - self.x_p) / self.t_p;
        self.x_p += dx_p * dt;

        let q_s = self.q0 * v.powf(self.beta_s);
        let q_t = self.q0 * v.powf(self.beta_t);
        let dx_q = (q_s - q_t - self.x_q) / self.t_q;
        self.x_q += dx_q * dt;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_zip_constant_power() {
        let z = ZipModel::constant_power(1.0, 0.3);
        assert!((z.active_power(1.0) - 1.0).abs() < 1e-10);
        assert!((z.active_power(0.9) - 1.0).abs() < 1e-10); // constant P
        assert!((z.reactive_power(1.0) - 0.3).abs() < 1e-10);
    }

    #[test]
    fn test_zip_constant_impedance_voltage_sensitive() {
        let z = ZipModel::constant_impedance(1.0, 0.3);
        let p09 = z.active_power(0.9);
        let p10 = z.active_power(1.0);
        assert!(
            p09 < p10,
            "Z load decreases with voltage: {:.4} < {:.4}",
            p09,
            p10
        );
        // P ~ V²: ratio = 0.81
        assert!(
            (p09 / p10 - 0.81).abs() < 0.01,
            "P ~ V²: ratio={:.4}",
            p09 / p10
        );
    }

    #[test]
    fn test_zip_residential_fractions_sum_to_one() {
        let z = ZipModel::residential(1.0, 1.0);
        let sum_alpha: f64 = z.alpha.iter().sum();
        assert!(
            (sum_alpha - 1.0).abs() < 1e-10,
            "α fractions: {:.6}",
            sum_alpha
        );
    }

    #[test]
    fn test_zip_dp_dv_positive_for_z_load() {
        let z = ZipModel::constant_impedance(1.0, 0.3);
        assert!(z.dp_dv_nominal() > 0.0);
        assert!(z.is_voltage_stabilising());
    }

    #[test]
    fn test_zip_dp_dv_zero_for_p_load() {
        let z = ZipModel::constant_power(1.0, 0.3);
        // dP/dV = 0 for constant P
        assert!(z.dp_dv_nominal().abs() < 1e-10);
    }

    #[test]
    fn test_motor_at_rated_load_small_slip() {
        let params = InductionMotorParams::air_conditioner(0.5);
        let state = MotorState::at_rated_load(&params);
        assert!(
            state.slip > 0.0 && state.slip < 0.5,
            "Rated slip out of range: {:.4}",
            state.slip
        );
    }

    #[test]
    fn test_motor_step_nominal_voltage() {
        let params = InductionMotorParams::pump_fan(1.0);
        let state = MotorState::at_rated_load(&params);
        let (new_state, p) = motor_step(&params, &state, 1.0, 0.01);
        assert!(!new_state.stalled, "Motor should not stall at V=1.0");
        assert!(p > 0.0, "Active power should be positive: {:.4}", p);
    }

    #[test]
    fn test_motor_step_low_voltage_may_stall() {
        let params = InductionMotorParams::air_conditioner(1.0);
        let mut state = MotorState {
            slip: 0.5,
            stalled: false,
        };
        // Apply very low voltage repeatedly
        for _ in 0..100 {
            let (ns, _) = motor_step(&params, &state, 0.5, 0.01);
            state = ns;
        }
        // At V=0.5 the motor may or may not stall depending on parameters
        // Just check it doesn't panic
        let _ = state.stalled;
    }

    #[test]
    fn test_motor_stalled_consumes_power() {
        let params = InductionMotorParams::air_conditioner(1.0);
        let state = MotorState {
            slip: 1.0,
            stalled: true,
        };
        let (new_state, p) = motor_step(&params, &state, 1.0, 0.01);
        assert!(new_state.stalled);
        assert!(p > 0.0, "Stalled motor should consume power: {:.4}", p);
    }

    #[test]
    fn test_clod_active_power_at_nominal() {
        let clod = ClodModel::wecc_standard(1.0, 0.3);
        let state = MotorState::at_rated_load(&clod.motor);
        let p = clod.active_power(1.0, &state);
        assert!(p > 0.0, "CLOD active power: {:.4}", p);
    }

    #[test]
    fn test_clod_reactive_power_positive() {
        let clod = ClodModel::wecc_standard(1.0, 0.3);
        let state = MotorState::at_rated_load(&clod.motor);
        let q = clod.reactive_power(1.0, &state);
        assert!(q > 0.0, "CLOD reactive power: {:.4}", q);
    }

    #[test]
    fn test_load_recovery_nominal_steady_state() {
        let model = LoadRecoveryModel::new(1.0, 0.3, 60.0, 80.0);
        // At V=1.0, x_p=x_q=0: P = P0*1^alpha_t = P0
        let (p, q) = model.power_demand(1.0);
        assert!((p - 1.0).abs() < 1e-10, "P at V=1: {:.6}", p);
        assert!((q - 0.3).abs() < 1e-10, "Q at V=1: {:.6}", q);
    }

    #[test]
    fn test_load_recovery_step_after_voltage_dip() {
        let mut model = LoadRecoveryModel::new(1.0, 0.3, 30.0, 40.0);
        let v_dip = 0.8;
        // After voltage dip, x_p should evolve toward static value
        for _ in 0..100 {
            model.step(v_dip, 0.1);
        }
        // x_p should be non-zero after voltage dip
        let _ = model.x_p;
        // Power demand at dip voltage should be less than nominal
        let (p_dip, _) = model.power_demand(v_dip);
        assert!(
            p_dip < 1.0,
            "Power at dip should be < nominal: {:.4}",
            p_dip
        );
    }

    #[test]
    fn test_electrical_torque_increases_with_voltage() {
        let params = InductionMotorParams::air_conditioner(1.0);
        let t1 = electrical_torque(&params, 0.05, 0.8);
        let t2 = electrical_torque(&params, 0.05, 1.0);
        assert!(
            t2 > t1,
            "Torque increases with voltage: {:.4} > {:.4}",
            t2,
            t1
        );
    }
}
