//! Dynamic load modeling for power system stability analysis.
//!
//! Implements ZIP, Exponential, Induction Motor, and Composite (WECC) load models
//! for time-domain transient stability simulations.
//!
//! # Models
//! 1. **ZIP model** — polynomial combination of constant Impedance, Current, Power
//! 2. **Exponential model** — voltage + frequency dependent algebraic model
//! 3. **Induction motor (3rd-order)** — slip dynamics with Euler integration
//! 4. **Composite (WECC CMPLDWG)** — four motor classes + static ZIP
//! 5. **Parameter fitting** — least-squares identification from measurement data
//!
//! # References
//! - Kundur, "Power System Stability and Control", Ch. 7 & 15.
//! - IEEE PES Load Representation for Dynamic Performance WG Report, 1995.
//! - WECC CMPLDWG Composite Load Model Implementation Guide, 2015.

use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// Internal LCG — used only in tests for deterministic synthetic data.
// ---------------------------------------------------------------------------
#[cfg(test)]
struct Lcg {
    state: u64,
}

#[cfg(test)]
impl Lcg {
    fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    fn next_f64(&mut self) -> f64 {
        self.state = self
            .state
            .wrapping_mul(6_364_136_223_846_793_005_u64)
            .wrapping_add(1_442_695_040_888_963_407_u64);
        (self.state >> 11) as f64 / (1_u64 << 53) as f64
    }
}

// ---------------------------------------------------------------------------
// Enums
// ---------------------------------------------------------------------------

/// Classification of load model types used in stability simulations.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum LoadModelType {
    /// Constant power — P and Q independent of voltage.
    ConstantPower,
    /// Constant current — P proportional to V.
    ConstantCurrent,
    /// Constant impedance — P proportional to V².
    ConstantImpedance,
    /// ZIP polynomial combination of Z, I, P components.
    Zip,
    /// Exponential voltage/frequency dependent load.
    Exponential,
    /// Third-order induction motor dynamic model.
    InductionMotor,
    /// Composite load combining ZIP and multiple motor groups.
    Composite,
    /// WECC composite load model (CMPLDWG).
    Wecc,
}

/// Classification of induction motor load types for parameter selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MotorLoadType {
    /// Single-phase motors (air conditioners, refrigerators).
    SinglePhase,
    /// Three-phase motors (small industrial drives).
    ThreePhase,
    /// High-inertia motors (large fans, mills).
    HighInertia,
    /// Low-inertia motors (pumps, compressors with unloading).
    LowInertia,
    /// Chiller compressor motors (HVAC systems).
    ChillerCompressor,
    /// Conveyor belt drive motors.
    ConveyorBelt,
    /// Pump motors (centrifugal, affinity laws apply).
    Pump,
    /// Fan motors (centrifugal, cube-law torque).
    Fan,
}

// ---------------------------------------------------------------------------
// ZIP Model
// ---------------------------------------------------------------------------

/// ZIP load model: `P = P0*(Zp·V² + Ip·V + Pp)`, `Q = Q0*(Zq·V² + Iq·V + Pq)`.
///
/// Constraint: `Zp + Ip + Pp = 1` and `Zq + Iq + Pq = 1`.
///
/// # Example
/// ```
/// use oxigrid::stability::load_modeling::ZipModel;
/// let z = ZipModel::new_typical_residential();
/// let p = z.compute_p(0.95);
/// ```
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ZipModel {
    /// Constant impedance fraction for active power (Zp).
    pub zp: f64,
    /// Constant current fraction for active power (Ip).
    pub ip: f64,
    /// Constant power fraction for active power (Pp). Constraint: `zp + ip + pp = 1`.
    pub pp: f64,
    /// Constant impedance fraction for reactive power (Zq).
    pub zq: f64,
    /// Constant current fraction for reactive power (Iq).
    pub iq: f64,
    /// Constant power fraction for reactive power (Pq). Constraint: `zq + iq + pq = 1`.
    pub pq: f64,
    /// Nominal active power `MW`.
    pub p0_mw: f64,
    /// Nominal reactive power `Mvar`.
    pub q0_mvar: f64,
}

impl ZipModel {
    /// Create a typical residential load ZIP model.
    ///
    /// Fractions: Zp=0.10, Ip=0.30, Pp=0.60; Zq=0.15, Iq=0.35, Pq=0.50.
    pub fn new_typical_residential() -> Self {
        Self {
            zp: 0.10,
            ip: 0.30,
            pp: 0.60,
            zq: 0.15,
            iq: 0.35,
            pq: 0.50,
            p0_mw: 1.0,
            q0_mvar: 0.3,
        }
    }

    /// Create a typical commercial load ZIP model.
    ///
    /// Fractions: Zp=0.05, Ip=0.35, Pp=0.60; Zq=0.10, Iq=0.40, Pq=0.50.
    pub fn new_typical_commercial() -> Self {
        Self {
            zp: 0.05,
            ip: 0.35,
            pp: 0.60,
            zq: 0.10,
            iq: 0.40,
            pq: 0.50,
            p0_mw: 1.0,
            q0_mvar: 0.4,
        }
    }

    /// Create a typical industrial load ZIP model.
    ///
    /// Fractions: Zp=0.20, Ip=0.20, Pp=0.60; Zq=0.20, Iq=0.30, Pq=0.50.
    pub fn new_typical_industrial() -> Self {
        Self {
            zp: 0.20,
            ip: 0.20,
            pp: 0.60,
            zq: 0.20,
            iq: 0.30,
            pq: 0.50,
            p0_mw: 1.0,
            q0_mvar: 0.5,
        }
    }

    /// Compute active power consumption at the given per-unit voltage.
    ///
    /// `P = P0 * (Zp·V² + Ip·V + Pp)`
    pub fn compute_p(&self, v_pu: f64) -> f64 {
        self.p0_mw * (self.zp * v_pu * v_pu + self.ip * v_pu + self.pp)
    }

    /// Compute reactive power consumption at the given per-unit voltage.
    ///
    /// `Q = Q0 * (Zq·V² + Iq·V + Pq)`
    pub fn compute_q(&self, v_pu: f64) -> f64 {
        self.q0_mvar * (self.zq * v_pu * v_pu + self.iq * v_pu + self.pq)
    }

    /// Compute the displacement power factor at nominal voltage.
    ///
    /// Returns `P0 / sqrt(P0² + Q0²)`, or 1.0 if both are negligible.
    pub fn power_factor(&self) -> f64 {
        let s = (self.p0_mw * self.p0_mw + self.q0_mvar * self.q0_mvar).sqrt();
        if s < 1e-12 {
            1.0
        } else {
            self.p0_mw / s
        }
    }
}

// ---------------------------------------------------------------------------
// Exponential Model
// ---------------------------------------------------------------------------

/// Exponential voltage- and frequency-dependent static load model.
///
/// `P = P0 · (V/V0)^alpha · (f/f0)^alpha_f`
/// `Q = Q0 · (V/V0)^beta  · (f/f0)^beta_f`
///
/// where V0 = f0 = 1.0 pu (nominal).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExponentialModel {
    /// Nominal active power `MW`.
    pub p0_mw: f64,
    /// Nominal reactive power `Mvar`.
    pub q0_mvar: f64,
    /// Voltage exponent for active power (alpha). Typical: 0.0–2.0.
    pub alpha: f64,
    /// Voltage exponent for reactive power (beta). Typical: 1.0–3.0.
    pub beta: f64,
    /// Frequency exponent for active power (alpha_f). Typical: 1.0.
    pub alpha_f: f64,
    /// Frequency exponent for reactive power (beta_f). Typical: -1.0.
    pub beta_f: f64,
}

impl ExponentialModel {
    /// Create a new exponential load model.
    pub fn new(p0_mw: f64, q0_mvar: f64, alpha: f64, beta: f64, alpha_f: f64, beta_f: f64) -> Self {
        Self {
            p0_mw,
            q0_mvar,
            alpha,
            beta,
            alpha_f,
            beta_f,
        }
    }

    /// Compute active power at the given per-unit voltage and frequency.
    ///
    /// `P = P0 · V^alpha · f^alpha_f`
    pub fn compute_p(&self, v_pu: f64, f_pu: f64) -> f64 {
        let v = v_pu.max(1e-12);
        let f = f_pu.max(1e-12);
        self.p0_mw * v.powf(self.alpha) * f.powf(self.alpha_f)
    }

    /// Compute reactive power at the given per-unit voltage and frequency.
    ///
    /// `Q = Q0 · V^beta · f^beta_f`
    pub fn compute_q(&self, v_pu: f64, f_pu: f64) -> f64 {
        let v = v_pu.max(1e-12);
        let f = f_pu.max(1e-12);
        self.q0_mvar * v.powf(self.beta) * f.powf(self.beta_f)
    }

    /// Compute the static voltage sensitivity dP/dV at the given voltage.
    ///
    /// `dP/dV = alpha · P0 · V^(alpha-1)`
    pub fn static_sensitivity_dp_dv(&self, v_pu: f64) -> f64 {
        let v = v_pu.max(1e-12);
        self.alpha * self.p0_mw * v.powf(self.alpha - 1.0)
    }
}

// ---------------------------------------------------------------------------
// Induction Motor Model
// ---------------------------------------------------------------------------

/// Third-order induction motor dynamic load model.
///
/// Uses the classical reduced-order voltage-behind-reactance formulation.
/// Slip dynamics are integrated via Euler's method from the swing equation:
/// `2H · ds/dt = T_mech - T_elec`
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InductionMotorModel {
    /// Unique identifier for this motor instance.
    pub id: usize,
    /// Motor type classification (determines default parameters).
    pub motor_type: MotorLoadType,
    /// Motor inertia constant H `seconds`.
    pub h_motor_s: f64,
    /// Stator resistance `pu`.
    pub rs: f64,
    /// Stator leakage reactance `pu`.
    pub xs: f64,
    /// Magnetizing reactance `pu`.
    pub xm: f64,
    /// Rotor resistance `pu`.
    pub rr: f64,
    /// Rotor leakage reactance `pu`.
    pub xr: f64,
    /// Rated active power `MW`.
    pub p_rated_mw: f64,
    /// Current slip. 0 = synchronous speed, 1 = standstill. Typical: 0.01–0.05.
    pub slip: f64,
    /// Terminal voltage `pu`.
    pub v_terminal_pu: f64,
    /// Voltage threshold below which stall is checked `pu`. Default: 0.6.
    pub stall_voltage_pu: f64,
    /// Whether the motor is currently stalled.
    pub stalled: bool,
}

impl InductionMotorModel {
    /// Create a new induction motor with type-appropriate default parameters.
    ///
    /// R, X values are chosen per WECC composite load model guidelines.
    pub fn new(id: usize, motor_type: MotorLoadType, h_s: f64, p_rated_mw: f64) -> Self {
        let (rs, xs, xm, rr, xr) = Self::typical_params(motor_type);
        Self {
            id,
            motor_type,
            h_motor_s: h_s,
            rs,
            xs,
            xm,
            rr,
            xr,
            p_rated_mw,
            slip: 0.03,
            v_terminal_pu: 1.0,
            stall_voltage_pu: 0.6,
            stalled: false,
        }
    }

    /// Return typical (Rs, Xs, Xm, Rr, Xr) per-unit parameters for the given motor type.
    fn typical_params(motor_type: MotorLoadType) -> (f64, f64, f64, f64, f64) {
        match motor_type {
            MotorLoadType::SinglePhase => (0.040, 0.100, 2.00, 0.035, 0.080),
            MotorLoadType::ThreePhase => (0.030, 0.100, 2.50, 0.030, 0.080),
            MotorLoadType::HighInertia => (0.020, 0.080, 3.00, 0.020, 0.060),
            MotorLoadType::LowInertia => (0.050, 0.120, 2.00, 0.045, 0.100),
            MotorLoadType::ChillerCompressor => (0.030, 0.090, 2.80, 0.025, 0.070),
            MotorLoadType::ConveyorBelt => (0.030, 0.100, 2.50, 0.030, 0.080),
            MotorLoadType::Pump => (0.025, 0.080, 2.80, 0.025, 0.070),
            MotorLoadType::Fan => (0.025, 0.080, 2.80, 0.020, 0.060),
        }
    }

    /// Compute electrical air-gap torque at the given slip and terminal voltage.
    ///
    /// Based on Thevenin-equivalent circuit: `Te = (Rr/s) · |I_rotor|²`.
    fn electrical_torque_at(&self, slip: f64, v_pu: f64) -> f64 {
        let s = if slip.abs() < 1e-9 {
            1e-9_f64.copysign(slip + 1e-9)
        } else {
            slip
        };
        // Simplified equivalent: series combination Rs + j·Xs + Rr/s
        let x_eq = self.xs + self.xm * self.xr / (self.xm + self.xr).max(1e-12);
        let r_eq = self.rs + self.rr / s;
        let denom = r_eq * r_eq + x_eq * x_eq;
        if denom < 1e-14 {
            return 0.0;
        }
        let i_sq = v_pu * v_pu / denom;
        (self.rr / s) * i_sq
    }

    /// Compute mechanical load torque as a function of slip.
    ///
    /// Fan/pump: cube-law `Tm = (1-s)²`; conveyor: mixed; others: constant.
    fn mechanical_torque_at(&self, slip: f64) -> f64 {
        let speed = (1.0 - slip).max(0.0);
        match self.motor_type {
            MotorLoadType::Fan | MotorLoadType::Pump => speed * speed,
            MotorLoadType::ConveyorBelt => 0.5 + 0.5 * speed,
            _ => 0.8, // approximately constant-torque load
        }
    }

    /// Compute the torque-speed characteristic curve.
    ///
    /// Returns 101 `(speed_pu, torque_pu)` pairs sampled at unit terminal voltage.
    pub fn compute_torque_speed(&self) -> Vec<(f64, f64)> {
        (0..=100)
            .map(|i| {
                let slip = i as f64 / 100.0;
                let speed = 1.0 - slip;
                let torque = self.electrical_torque_at(slip, 1.0);
                (speed, torque)
            })
            .collect()
    }

    /// Compute electrical active and reactive power from current slip and voltage.
    ///
    /// Returns `(P_mw, Q_mvar)`.
    pub fn compute_electrical_power(&self) -> (f64, f64) {
        let s = if self.slip.abs() < 1e-9 {
            1e-9
        } else {
            self.slip
        };
        let v = self.v_terminal_pu;

        let te = self.electrical_torque_at(s, v);
        // Shaft power = air-gap power × mechanical speed (1-s at pu base)
        let p_shaft = te * (1.0 - s);

        // Magnetising + leakage reactive consumption
        let x_eq = self.xs + self.xm * self.xr / (self.xm + self.xr).max(1e-12);
        let r_eq = self.rs + self.rr / s;
        let denom = r_eq * r_eq + x_eq * x_eq;
        let i_sq = if denom < 1e-14 { 0.0 } else { v * v / denom };
        let q_total = (self.xs + self.xr) * i_sq + v * v / self.xm.max(1e-12);

        (p_shaft * self.p_rated_mw, q_total * self.p_rated_mw)
    }

    /// Integrate the slip swing equation by one Euler step.
    ///
    /// `2H · ds/dt = T_mech - T_elec`
    pub fn step_euler(&mut self, v_pu: f64, dt_s: f64) {
        self.v_terminal_pu = v_pu;
        if self.stalled {
            self.slip = 1.0;
            return;
        }
        let te = self.electrical_torque_at(self.slip, v_pu);
        let tm = self.mechanical_torque_at(self.slip);
        let ds_dt = (tm - te) / (2.0 * self.h_motor_s.max(1e-9));
        self.slip = (self.slip + ds_dt * dt_s).clamp(0.0, 1.0);
        self.check_stall();
    }

    /// Detect stall condition.
    ///
    /// Sets `stalled = true` when terminal voltage < `stall_voltage_pu` AND slip > 0.2.
    pub fn check_stall(&mut self) {
        if self.v_terminal_pu < self.stall_voltage_pu && self.slip > 0.2 {
            self.stalled = true;
            self.slip = 1.0;
        }
    }
}

// ---------------------------------------------------------------------------
// Composite Load Model (WECC)
// ---------------------------------------------------------------------------

/// WECC Composite Load Model combining static ZIP and four motor groups.
///
/// Motor groups:
/// - A: single-phase (residential A/C)
/// - B: small three-phase motors
/// - C: large motor loads (compressors, chillers)
/// - D: electronic / discharge lighting (represented as constant-power ZIP)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CompositeLoadModel {
    /// Fraction of total load modelled as static ZIP.
    pub zip_fraction: f64,
    /// Fraction from single-phase (type A) motors.
    pub motor_a_fraction: f64,
    /// Fraction from three-phase small (type B) motors.
    pub motor_b_fraction: f64,
    /// Fraction from large compressor (type C) motors.
    pub motor_c_fraction: f64,
    /// Fraction from electronic/discharge lighting (type D).
    pub motor_d_fraction: f64,
    /// ZIP model representing the static fraction.
    pub zip_model: ZipModel,
    /// Type-A motor instances.
    pub motors_a: Vec<InductionMotorModel>,
    /// Type-B motor instances.
    pub motors_b: Vec<InductionMotorModel>,
    /// Type-C motor instances.
    pub motors_c: Vec<InductionMotorModel>,
    /// Total nominal active power `MW`.
    pub total_p_mw: f64,
    /// Total nominal reactive power `Mvar`.
    pub total_q_mvar: f64,
}

impl CompositeLoadModel {
    /// Create a WECC composite load model.
    ///
    /// `motor_fractions` = `[frac_a, frac_b, frac_c, frac_d]`.
    /// Fractions should ideally sum to 1 with `zip_fraction`.
    pub fn new(
        total_p_mw: f64,
        total_q_mvar: f64,
        zip_fraction: f64,
        motor_fractions: [f64; 4],
    ) -> Self {
        let [frac_a, frac_b, frac_c, frac_d] = motor_fractions;

        let mut zip = ZipModel::new_typical_commercial();
        zip.p0_mw = total_p_mw * zip_fraction;
        zip.q0_mvar = total_q_mvar * zip_fraction;

        let motor_a =
            InductionMotorModel::new(0, MotorLoadType::SinglePhase, 0.5, total_p_mw * frac_a);
        let motor_b =
            InductionMotorModel::new(1, MotorLoadType::ThreePhase, 1.0, total_p_mw * frac_b);
        let motor_c = InductionMotorModel::new(
            2,
            MotorLoadType::ChillerCompressor,
            2.0,
            total_p_mw * frac_c,
        );

        Self {
            zip_fraction,
            motor_a_fraction: frac_a,
            motor_b_fraction: frac_b,
            motor_c_fraction: frac_c,
            motor_d_fraction: frac_d,
            zip_model: zip,
            motors_a: vec![motor_a],
            motors_b: vec![motor_b],
            motors_c: vec![motor_c],
            total_p_mw,
            total_q_mvar,
        }
    }

    /// Compute aggregate `(P_mw, Q_mvar)` for all load components.
    pub fn compute_total_load(&self, v_pu: f64, f_pu: f64) -> (f64, f64) {
        let p_zip = self.zip_model.compute_p(v_pu);
        let q_zip = self.zip_model.compute_q(v_pu);

        let (p_motors, q_motors) = self
            .motors_a
            .iter()
            .chain(self.motors_b.iter())
            .chain(self.motors_c.iter())
            .fold((0.0_f64, 0.0_f64), |(pa, qa), m| {
                let (p, q) = m.compute_electrical_power();
                (pa + p, qa + q)
            });

        // Type-D: near-constant power with mild frequency dependence
        let p_d = self.total_p_mw * self.motor_d_fraction * f_pu;
        let q_d = self.total_q_mvar * self.motor_d_fraction * 0.1;

        (p_zip + p_motors + p_d, q_zip + q_motors + q_d)
    }

    /// Evaluate stall conditions for all motors under the given voltage.
    pub fn check_stall_cascade(&mut self, v_pu: f64) {
        for m in self
            .motors_a
            .iter_mut()
            .chain(self.motors_b.iter_mut())
            .chain(self.motors_c.iter_mut())
        {
            m.v_terminal_pu = v_pu;
            m.check_stall();
        }
    }
}

// ---------------------------------------------------------------------------
// Load State
// ---------------------------------------------------------------------------

/// Dynamic state variables for a single load component.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LoadState {
    /// Motor slip (0 = synchronous, 1 = standstill).
    pub slip: f64,
    /// D-axis transient voltage behind reactance `pu`.
    pub ed_prime: f64,
    /// Q-axis transient voltage behind reactance `pu`.
    pub eq_prime: f64,
    /// Current active power demand `MW`.
    pub p_mw: f64,
    /// Current reactive power demand `Mvar`.
    pub q_mvar: f64,
}

impl Default for LoadState {
    fn default() -> Self {
        Self {
            slip: 0.03,
            ed_prime: 0.0,
            eq_prime: 1.0,
            p_mw: 0.0,
            q_mvar: 0.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Load Simulator
// ---------------------------------------------------------------------------

/// Time-domain load simulator for composite dynamic load models.
///
/// Advances all motor states via Euler integration and computes aggregate
/// load power at each simulation timestep.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LoadSimulator {
    /// The composite load model being simulated.
    pub composite_model: CompositeLoadModel,
    /// Per-motor dynamic state (indexed to match motor ordering A→B→C).
    pub states: Vec<LoadState>,
    /// Simulation timestep `s`. Default: 0.01 s (one cycle at 50 Hz).
    pub dt_s: f64,
    /// System frequency `pu`. Default: 1.0.
    pub frequency_pu: f64,
    /// Snapshot of motor slips at construction time (used by `reset`).
    initial_slips: Vec<f64>,
}

impl LoadSimulator {
    /// Create a new load simulator.
    pub fn new(model: CompositeLoadModel, dt_s: f64) -> Self {
        let n = model.motors_a.len() + model.motors_b.len() + model.motors_c.len();
        let initial_slips: Vec<f64> = model
            .motors_a
            .iter()
            .chain(model.motors_b.iter())
            .chain(model.motors_c.iter())
            .map(|m| m.slip)
            .collect();

        Self {
            composite_model: model,
            states: vec![LoadState::default(); n],
            dt_s,
            frequency_pu: 1.0,
            initial_slips,
        }
    }

    /// Advance all motors by one timestep and return aggregate `(P_mw, Q_mvar)`.
    pub fn step(&mut self, v_pu: f64, f_pu: f64) -> (f64, f64) {
        self.frequency_pu = f_pu;
        let dt = self.dt_s;

        for m in self
            .composite_model
            .motors_a
            .iter_mut()
            .chain(self.composite_model.motors_b.iter_mut())
            .chain(self.composite_model.motors_c.iter_mut())
        {
            m.step_euler(v_pu, dt);
        }

        self.composite_model.check_stall_cascade(v_pu);
        self.composite_model.compute_total_load(v_pu, f_pu)
    }

    /// Simulate a voltage sag event followed by voltage recovery.
    ///
    /// Applies `v_sag_pu` for `n_steps` Euler steps each of duration
    /// `duration_s / n_steps`, then restores voltage to 1.0 for an equal span.
    ///
    /// Returns `[(time_s, P_mw, Q_mvar)]` — 2 × n_steps entries.
    pub fn simulate_voltage_sag(
        &mut self,
        v_sag_pu: f64,
        duration_s: f64,
        n_steps: usize,
    ) -> Vec<(f64, f64, f64)> {
        let n = n_steps.max(1);
        let dt = duration_s / n as f64;
        self.dt_s = dt;

        let mut out = Vec::with_capacity(2 * n);

        for i in 0..n {
            let t = i as f64 * dt;
            let (p, q) = self.step(v_sag_pu, self.frequency_pu);
            out.push((t, p, q));
        }

        for i in 0..n {
            let t = duration_s + i as f64 * dt;
            let (p, q) = self.step(1.0, self.frequency_pu);
            out.push((t, p, q));
        }

        out
    }

    /// Reset the simulator to initial operating conditions.
    pub fn reset(&mut self) {
        #![allow(clippy::explicit_counter_loop)]
        let initial_slips = self.initial_slips.clone();
        let mut idx = 0usize;

        for m in self
            .composite_model
            .motors_a
            .iter_mut()
            .chain(self.composite_model.motors_b.iter_mut())
            .chain(self.composite_model.motors_c.iter_mut())
        {
            m.slip = initial_slips.get(idx).copied().unwrap_or(0.03);
            m.stalled = false;
            m.v_terminal_pu = 1.0;
            idx += 1;
        }

        let n = self.states.len();
        self.states = vec![LoadState::default(); n];
        self.frequency_pu = 1.0;
    }
}

// ---------------------------------------------------------------------------
// Load Model Fitter
// ---------------------------------------------------------------------------

/// Parameter identification for load models from measurement data.
///
/// # Methods
/// - [`LoadModelFitter::fit_zip_from_measurements`] — least-squares normal equations for ZIP
/// - [`LoadModelFitter::fit_exponential_from_measurements`] — log-linear regression for exponential
pub struct LoadModelFitter;

impl LoadModelFitter {
    /// Fit a ZIP model to voltage-P-Q measurement data via least squares.
    ///
    /// Solves `P(V) = c0·V² + c1·V + c2` and normalises to recover Zp, Ip, Pp.
    /// Falls back to a typical residential model if fewer than 3 measurements are
    /// provided or the normal equations are singular.
    pub fn fit_zip_from_measurements(
        voltages: &[f64],
        p_measured: &[f64],
        q_measured: &[f64],
    ) -> ZipModel {
        let n = voltages.len().min(p_measured.len()).min(q_measured.len());
        if n < 3 {
            return ZipModel::new_typical_residential();
        }

        let (zp_p0, ip_p0, pp_p0) = Self::ls_quadratic(voltages, p_measured, n);
        let (zq_q0, iq_q0, pq_q0) = Self::ls_quadratic(voltages, q_measured, n);

        let p0 = (zp_p0 + ip_p0 + pp_p0).max(1e-9);
        let q0 = (zq_q0 + iq_q0 + pq_q0).abs().max(1e-9);

        let zp = (zp_p0 / p0).clamp(0.0, 1.0);
        let ip = ((ip_p0 / p0).clamp(0.0, 1.0 - zp)).max(0.0);
        let pp = (1.0 - zp - ip).clamp(0.0, 1.0);

        let zq = (zq_q0 / q0).clamp(0.0, 1.0);
        let iq = ((iq_q0 / q0).clamp(0.0, 1.0 - zq)).max(0.0);
        let pq = (1.0 - zq - iq).clamp(0.0, 1.0);

        ZipModel {
            zp,
            ip,
            pp,
            zq,
            iq,
            pq,
            p0_mw: p0,
            q0_mvar: q0,
        }
    }

    /// Solve least-squares quadratic fit `b ≈ c0·V² + c1·V + c2` via 3×3 normal equations.
    fn ls_quadratic(voltages: &[f64], b: &[f64], n: usize) -> (f64, f64, f64) {
        let mut ata = [[0.0_f64; 3]; 3];
        let mut atb = [0.0_f64; 3];

        for i in 0..n {
            let v = voltages[i];
            let row = [v * v, v, 1.0_f64];
            let bi = b[i];
            for r in 0..3 {
                atb[r] += row[r] * bi;
                for c in 0..3 {
                    ata[r][c] += row[r] * row[c];
                }
            }
        }

        let mean_b = b[..n].iter().copied().sum::<f64>() / n as f64;
        Self::gauss3(&ata, &atb).unwrap_or((0.0, 0.0, mean_b))
    }

    /// Gaussian elimination with partial pivoting on a 3×3 augmented system.
    #[allow(clippy::needless_range_loop)]
    fn gauss3(a: &[[f64; 3]; 3], b: &[f64; 3]) -> Option<(f64, f64, f64)> {
        let mut aug = [
            [a[0][0], a[0][1], a[0][2], b[0]],
            [a[1][0], a[1][1], a[1][2], b[1]],
            [a[2][0], a[2][1], a[2][2], b[2]],
        ];

        for col in 0..3 {
            // Partial pivot
            let mut max_row = col;
            let mut max_val = aug[col][col].abs();
            for row in (col + 1)..3 {
                if aug[row][col].abs() > max_val {
                    max_val = aug[row][col].abs();
                    max_row = row;
                }
            }
            aug.swap(col, max_row);

            let pivot = aug[col][col];
            if pivot.abs() < 1e-14 {
                return None;
            }
            for row in (col + 1)..3 {
                let factor = aug[row][col] / pivot;
                for k in col..4 {
                    let v = aug[col][k];
                    aug[row][k] -= factor * v;
                }
            }
        }

        // Back-substitution
        let x2 = aug[2][3] / aug[2][2];
        let x1 = (aug[1][3] - aug[1][2] * x2) / aug[1][1];
        let x0 = (aug[0][3] - aug[0][2] * x2 - aug[0][1] * x1) / aug[0][0];
        Some((x0, x1, x2))
    }

    /// Fit an exponential load model via log-linear regression.
    ///
    /// Performs `ln(P/P0) = alpha · ln(V)` OLS regression where P0 is estimated
    /// at V ≈ 1.0. Frequency exponents are set to defaults (alpha_f=1.0, beta_f=-1.0).
    ///
    /// Falls back to `alpha=1.0` if regression is ill-conditioned.
    pub fn fit_exponential_from_measurements(
        voltages: &[f64],
        p_measured: &[f64],
    ) -> ExponentialModel {
        let n = voltages.len().min(p_measured.len());
        if n < 2 {
            return ExponentialModel::new(1.0, 0.3, 1.0, 2.0, 1.0, -1.0);
        }

        let p0 = Self::p_at_nominal(voltages, p_measured, n);

        // Log-linear regression: y = alpha * x, y = ln(P/P0), x = ln(V)
        let mut sum_x = 0.0_f64;
        let mut sum_xx = 0.0_f64;
        let mut sum_xy = 0.0_f64;
        let mut cnt = 0usize;

        for i in 0..n {
            let v = voltages[i];
            let p = p_measured[i];
            if v > 1e-9 && p > 1e-12 && p0 > 1e-12 {
                let x = v.ln();
                let y = (p / p0).ln();
                sum_x += x;
                sum_xx += x * x;
                sum_xy += x * y;
                cnt += 1;
            }
        }

        // OLS slope: beta = (n*Σxy - Σx*Σy) / (n*Σx² - (Σx)²)
        // where y = ln(P/P0), x = ln(V)
        // Equivalently, use slope-through-origin on centered data, or standard OLS.
        let alpha = if cnt > 1 {
            let n_f = cnt as f64;
            // Collect sum_y to apply centred formula
            let sum_y: f64 = {
                let mut sy = 0.0_f64;
                for i in 0..n {
                    let v = voltages[i];
                    let p = p_measured[i];
                    if v > 1e-9 && p > 1e-12 && p0 > 1e-12 {
                        sy += (p / p0).ln();
                    }
                }
                sy
            };
            let denom = n_f * sum_xx - sum_x * sum_x;
            if denom.abs() > 1e-14 {
                ((n_f * sum_xy - sum_x * sum_y) / denom).clamp(-8.0, 8.0)
            } else if sum_xx.abs() > 1e-14 {
                (sum_xy / sum_xx).clamp(-8.0, 8.0)
            } else {
                1.0
            }
        } else if sum_xx.abs() > 1e-14 {
            (sum_xy / sum_xx).clamp(-8.0, 8.0)
        } else {
            1.0
        };

        ExponentialModel::new(p0, p0 * 0.3, alpha, (alpha * 1.5).max(0.5), 1.0, -1.0)
    }

    /// Estimate P0 as the measurement at the voltage nearest to 1.0 pu.
    #[allow(clippy::needless_range_loop)]
    fn p_at_nominal(voltages: &[f64], p_measured: &[f64], n: usize) -> f64 {
        let mut best_idx = 0usize;
        let mut best_dist = (voltages[0] - 1.0).abs();
        for i in 1..n {
            let d = (voltages[i] - 1.0).abs();
            if d < best_dist {
                best_dist = d;
                best_idx = i;
            }
        }
        p_measured[best_idx].max(1e-12)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // ── ZIP ──────────────────────────────────────────────────────────────────

    #[test]
    fn test_zip_fractions_sum_to_one() {
        let z = ZipModel::new_typical_residential();
        let sp = z.zp + z.ip + z.pp;
        let sq = z.zq + z.iq + z.pq;
        assert!((sp - 1.0).abs() < 1e-10, "P fractions sum = {sp}");
        assert!((sq - 1.0).abs() < 1e-10, "Q fractions sum = {sq}");
    }

    #[test]
    fn test_zip_compute_p_nominal() {
        let mut z = ZipModel::new_typical_residential();
        z.p0_mw = 10.0;
        let p = z.compute_p(1.0);
        assert!((p - 10.0).abs() < 1e-9, "P at V=1.0: {p}");
    }

    #[test]
    fn test_zip_compute_p_low_voltage() {
        let mut z = ZipModel::new_typical_residential();
        z.p0_mw = 10.0;
        let p1 = z.compute_p(1.0);
        let p09 = z.compute_p(0.9);
        assert!(p09 < p1, "P(0.9)={p09} should be < P(1.0)={p1}");
    }

    #[test]
    fn test_zip_constant_power_behavior() {
        let z = ZipModel {
            zp: 0.0,
            ip: 0.0,
            pp: 1.0,
            zq: 0.0,
            iq: 0.0,
            pq: 1.0,
            p0_mw: 5.0,
            q0_mvar: 2.0,
        };
        for &v in &[0.7_f64, 0.9, 1.0, 1.1, 1.2] {
            let p = z.compute_p(v);
            assert!((p - 5.0).abs() < 1e-9, "Constant P at V={v}: {p}");
        }
    }

    #[test]
    fn test_zip_constant_impedance_behavior() {
        let z = ZipModel {
            zp: 1.0,
            ip: 0.0,
            pp: 0.0,
            zq: 1.0,
            iq: 0.0,
            pq: 0.0,
            p0_mw: 4.0,
            q0_mvar: 1.0,
        };
        let p08 = z.compute_p(0.8);
        let p10 = z.compute_p(1.0);
        let ratio = p08 / p10;
        assert!((ratio - 0.64).abs() < 1e-9, "Constant-Z ratio: {ratio}");
    }

    #[test]
    fn test_zip_residential_typical() {
        let z = ZipModel::new_typical_residential();
        assert!(z.zp >= 0.0 && z.ip >= 0.0 && z.pp >= 0.0);
        assert!((z.zp + z.ip + z.pp - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_zip_commercial_typical() {
        let z = ZipModel::new_typical_commercial();
        assert!(z.pp > 0.5, "Commercial: Pp={}", z.pp);
        assert!((z.zp + z.ip + z.pp - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_zip_power_factor() {
        let mut z = ZipModel::new_typical_residential();
        z.p0_mw = 3.0;
        z.q0_mvar = 4.0;
        let pf = z.power_factor();
        assert!((pf - 0.6).abs() < 1e-9, "PF: {pf}");
    }

    // ── Exponential ──────────────────────────────────────────────────────────

    #[test]
    fn test_exponential_model_nominal() {
        let m = ExponentialModel::new(10.0, 4.0, 1.5, 2.0, 1.0, -1.0);
        let p = m.compute_p(1.0, 1.0);
        let q = m.compute_q(1.0, 1.0);
        assert!((p - 10.0).abs() < 1e-9, "P at V=f=1: {p}");
        assert!((q - 4.0).abs() < 1e-9, "Q at V=f=1: {q}");
    }

    #[test]
    fn test_exponential_model_sensitivity() {
        let m = ExponentialModel::new(10.0, 4.0, 1.5, 2.0, 1.0, -1.0);
        let sens = m.static_sensitivity_dp_dv(1.0);
        assert!((sens - 15.0).abs() < 1e-6, "dP/dV at V=1: {sens}");
    }

    #[test]
    fn test_static_sensitivity() {
        let m = ExponentialModel::new(5.0, 2.0, 2.0, 3.0, 1.0, -1.0);
        let s = m.static_sensitivity_dp_dv(1.0);
        // alpha * P0 * 1^(alpha-1) = 2 * 5 = 10
        assert!((s - 10.0).abs() < 1e-9, "dP/dV: {s}");
    }

    // ── Induction Motor ──────────────────────────────────────────────────────

    #[test]
    fn test_motor_torque_speed_curve() {
        let m = InductionMotorModel::new(0, MotorLoadType::Pump, 1.5, 2.0);
        let curve = m.compute_torque_speed();
        assert_eq!(curve.len(), 101);
        // At slip=0 (first entry), torque should be ~0
        let (_, t_zero_slip) = curve[0];
        assert!(
            t_zero_slip.abs() < 0.1,
            "Torque at zero slip: {t_zero_slip}"
        );
        let max_t = curve.iter().map(|&(_, t)| t).fold(0.0_f64, f64::max);
        assert!(max_t > 0.0, "Max torque should be positive: {max_t}");
    }

    #[test]
    fn test_motor_step_euler() {
        let mut m = InductionMotorModel::new(0, MotorLoadType::Fan, 2.0, 5.0);
        let s0 = m.slip;
        for _ in 0..100 {
            m.step_euler(0.7, 0.01);
        }
        let changed = (m.slip - s0).abs() > 1e-6 || m.stalled;
        assert!(changed, "Slip should change under reduced voltage");
    }

    #[test]
    fn test_motor_stall_detection() {
        let mut m = InductionMotorModel::new(0, MotorLoadType::SinglePhase, 0.5, 1.0);
        m.slip = 0.25;
        m.v_terminal_pu = 0.55;
        m.check_stall();
        assert!(m.stalled, "Motor should stall at V=0.55, slip=0.25");
    }

    // ── Composite ────────────────────────────────────────────────────────────

    #[test]
    fn test_composite_total_load_nominal() {
        let model = CompositeLoadModel::new(10.0, 4.0, 0.3, [0.3, 0.2, 0.1, 0.1]);
        let (p, _q) = model.compute_total_load(1.0, 1.0);
        assert!(p > 0.0 && p < 25.0, "Total P at V=1.0: {p}");
    }

    #[test]
    fn test_composite_stall_cascade() {
        let mut model = CompositeLoadModel::new(10.0, 4.0, 0.2, [0.3, 0.2, 0.1, 0.2]);
        // Force all motors to high slip first
        for m in model
            .motors_a
            .iter_mut()
            .chain(model.motors_b.iter_mut())
            .chain(model.motors_c.iter_mut())
        {
            m.slip = 0.5;
        }
        model.check_stall_cascade(0.5);
        let any_stalled = model
            .motors_a
            .iter()
            .chain(model.motors_b.iter())
            .chain(model.motors_c.iter())
            .any(|m| m.stalled);
        assert!(
            any_stalled,
            "At least one motor should stall at V=0.5 with slip=0.5"
        );
    }

    // ── LoadSimulator ────────────────────────────────────────────────────────

    #[test]
    fn test_load_simulator_step() {
        let model = CompositeLoadModel::new(10.0, 4.0, 0.3, [0.3, 0.2, 0.1, 0.1]);
        let mut sim = LoadSimulator::new(model, 0.01);
        let (p, q) = sim.step(1.0, 1.0);
        assert!(p > 0.0, "P: {p}");
        assert!(q >= 0.0, "Q: {q}");
    }

    #[test]
    fn test_voltage_sag_simulation() {
        let model = CompositeLoadModel::new(10.0, 4.0, 0.3, [0.3, 0.2, 0.1, 0.1]);
        let mut sim = LoadSimulator::new(model, 0.01);
        let res = sim.simulate_voltage_sag(0.7, 0.5, 50);
        assert_eq!(res.len(), 100, "Expected 2×50 results");
        // Times are non-decreasing
        for w in res.windows(2) {
            assert!(w[1].0 >= w[0].0 - 1e-12, "Times non-decreasing");
        }
    }

    #[test]
    fn test_voltage_sag_recovery() {
        let model = CompositeLoadModel::new(10.0, 4.0, 0.4, [0.2, 0.2, 0.1, 0.1]);
        let mut sim = LoadSimulator::new(model, 0.005);
        let res = sim.simulate_voltage_sag(0.8, 0.2, 40);
        let p_mid = res[20].1;
        let p_end = res[res.len() - 1].1;
        assert!(p_mid >= 0.0, "P during sag: {p_mid}");
        assert!(p_end >= 0.0, "P after recovery: {p_end}");
    }

    // ── LoadModelFitter ──────────────────────────────────────────────────────

    #[test]
    fn test_fit_zip_from_measurements() {
        let known = ZipModel {
            zp: 0.20,
            ip: 0.30,
            pp: 0.50,
            zq: 0.10,
            iq: 0.40,
            pq: 0.50,
            p0_mw: 5.0,
            q0_mvar: 2.0,
        };
        let voltages: Vec<f64> = (0..=20).map(|i| 0.8 + 0.02 * i as f64).collect();
        let pm: Vec<f64> = voltages.iter().map(|&v| known.compute_p(v)).collect();
        let qm: Vec<f64> = voltages.iter().map(|&v| known.compute_q(v)).collect();

        let fitted = LoadModelFitter::fit_zip_from_measurements(&voltages, &pm, &qm);
        assert!((fitted.p0_mw - 5.0).abs() < 0.5, "P0 = {}", fitted.p0_mw);
        assert!(
            fitted.zp >= 0.0 && fitted.zp <= 1.0,
            "Zp in range: {}",
            fitted.zp
        );
        assert!(
            (fitted.zp + fitted.ip + fitted.pp - 1.0).abs() < 1e-6,
            "P fractions sum"
        );
    }

    #[test]
    fn test_fit_exponential_log_linear() {
        let p0 = 8.0_f64;
        let alpha_true = 1.5_f64;
        let voltages: Vec<f64> = (0..=20).map(|i| 0.8 + 0.01 * i as f64).collect();
        let pm: Vec<f64> = voltages.iter().map(|&v| p0 * v.powf(alpha_true)).collect();

        let fitted = LoadModelFitter::fit_exponential_from_measurements(&voltages, &pm);
        assert!(
            (fitted.alpha - alpha_true).abs() < 0.5,
            "alpha fitted={} true={alpha_true}",
            fitted.alpha
        );
        assert!(fitted.p0_mw > 0.0, "P0: {}", fitted.p0_mw);
    }

    #[test]
    fn test_fit_exponential_noise_robustness() {
        let mut lcg = Lcg::new(42);
        let p0 = 6.0_f64;
        let alpha_true = 1.2_f64;
        let voltages: Vec<f64> = (0..=30).map(|i| 0.85 + 0.005 * i as f64).collect();
        let pm: Vec<f64> = voltages
            .iter()
            .map(|&v| p0 * v.powf(alpha_true) * (1.0 + 0.01 * (lcg.next_f64() - 0.5)))
            .collect();

        let fitted = LoadModelFitter::fit_exponential_from_measurements(&voltages, &pm);
        assert!(
            (fitted.alpha - alpha_true).abs() < 0.5,
            "Noisy alpha fitted={} true={alpha_true}",
            fitted.alpha
        );
    }
}
