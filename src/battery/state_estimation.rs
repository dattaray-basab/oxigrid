//! Battery Pack State Estimation via Extended Kalman Filter (EKF).
//!
//! This module provides SoC/SoH tracking and adaptive ECM parameter identification
//! for large-scale battery packs using a 2RC Thevenin equivalent circuit model.
//!
//! # Model
//!
//! State vector: `x = [SoC, V_RC1, V_RC2]`
//!
//! Terminal voltage:
//! ```text
//! V_t = OCV(SoC) - I·R0 - V_RC1 - V_RC2
//! ```
//!
//! # Units
//!
//! | Symbol | Unit |
//! |--------|------|
//! | Current | \[A\] (positive = discharge) |
//! | Voltage | \[V\] |
//! | Capacity | \[Ah\] |
//! | Resistance | \[Ω\] |
//! | Capacitance | \[F\] |
//! | Time | \[s\] |

use serde::{Deserialize, Serialize};

// ─────────────────────────────────────────────────────────────────────────────
// Error type
// ─────────────────────────────────────────────────────────────────────────────

/// Errors produced by the battery state estimator.
#[derive(Debug, Clone, PartialEq)]
pub enum StateEstimationError {
    /// OCV-SoC table is empty or has fewer than two entries.
    EmptyOcvTable,
    /// A matrix inversion failed (singular matrix).
    SingularMatrix,
    /// A parameter is outside its physical bounds.
    ParameterOutOfBounds(String),
}

impl core::fmt::Display for StateEstimationError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::EmptyOcvTable => write!(f, "OCV-SoC table is empty or too small"),
            Self::SingularMatrix => write!(f, "Matrix inversion failed: singular"),
            Self::ParameterOutOfBounds(msg) => write!(f, "Parameter out of bounds: {msg}"),
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Battery chemistry
// ─────────────────────────────────────────────────────────────────────────────

/// Battery cell chemistry, used to select built-in OCV-SoC tables and
/// default ECM parameters.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BatteryChemistryType {
    /// Lithium Iron Phosphate — flat OCV plateau ~3.2–3.35 V.
    Lfp,
    /// Nickel Manganese Cobalt Oxide — sloped OCV 3.0–4.2 V.
    Nmc,
    /// Nickel Cobalt Aluminium Oxide — high energy density 3.0–4.2 V.
    Nca,
    /// Lithium Titanate Oxide — low voltage 1.5–2.7 V, very flat.
    Lto,
}

// ─────────────────────────────────────────────────────────────────────────────
// ECM parameters
// ─────────────────────────────────────────────────────────────────────────────

/// Equivalent Circuit Model (2RC Thevenin) parameters.
///
/// ```text
///   R0         R1         R2
///  ──┤├──┬──┤├──┤ ├──┬──┤├──┤ ├──┬──
///        │   (C1)       │   (C2)   │
///       OCV            ...        V_t
/// ```
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EcmParameters {
    /// Series (ohmic) resistance \[Ω\].
    pub r0_ohm: f64,
    /// RC-pair 1 resistance \[Ω\].
    pub r1_ohm: f64,
    /// RC-pair 1 capacitance \[F\].
    pub c1_f: f64,
    /// RC-pair 2 resistance \[Ω\].
    pub r2_ohm: f64,
    /// RC-pair 2 capacitance \[F\].
    pub c2_f: f64,
    /// OCV-SoC look-up table: sorted `(SoC ∈ [0,1], OCV \[V\])` pairs.
    pub ocv_soc_table: Vec<(f64, f64)>,
}

impl EcmParameters {
    /// Validate that all resistances and capacitances are positive and finite,
    /// and that the OCV table has at least two entries.
    pub fn validate(&self) -> Result<(), StateEstimationError> {
        if self.r0_ohm <= 0.0 || !self.r0_ohm.is_finite() {
            return Err(StateEstimationError::ParameterOutOfBounds(
                "r0_ohm must be positive and finite".into(),
            ));
        }
        if self.r1_ohm <= 0.0 || !self.r1_ohm.is_finite() {
            return Err(StateEstimationError::ParameterOutOfBounds(
                "r1_ohm must be positive and finite".into(),
            ));
        }
        if self.c1_f <= 0.0 || !self.c1_f.is_finite() {
            return Err(StateEstimationError::ParameterOutOfBounds(
                "c1_f must be positive and finite".into(),
            ));
        }
        if self.r2_ohm <= 0.0 || !self.r2_ohm.is_finite() {
            return Err(StateEstimationError::ParameterOutOfBounds(
                "r2_ohm must be positive and finite".into(),
            ));
        }
        if self.c2_f <= 0.0 || !self.c2_f.is_finite() {
            return Err(StateEstimationError::ParameterOutOfBounds(
                "c2_f must be positive and finite".into(),
            ));
        }
        if self.ocv_soc_table.len() < 2 {
            return Err(StateEstimationError::EmptyOcvTable);
        }
        Ok(())
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Estimator configuration
// ─────────────────────────────────────────────────────────────────────────────

/// Configuration for [`BatteryPackEstimator`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BatteryStateEstimatorConfig {
    /// Nominal cell / pack capacity \[Ah\].
    pub capacity_ah: f64,
    /// Nominal terminal voltage \[V\] (for reference only).
    pub nominal_voltage_v: f64,
    /// EKF sampling interval \[s\] (default 1.0).
    pub dt_s: f64,
    /// Process noise covariance scalar Q (default 1e-6).
    pub q_noise: f64,
    /// Measurement noise covariance scalar R (default 1e-4).
    pub r_noise: f64,
    /// Initial SoC estimate ∈ \[0, 1\].
    pub initial_soc: f64,
    /// Initial SoH estimate ∈ \[0, 1\].
    pub initial_soh: f64,
    /// Cell chemistry (selects default OCV curve if not overridden).
    pub chemistry: BatteryChemistryType,
}

impl Default for BatteryStateEstimatorConfig {
    fn default() -> Self {
        Self {
            capacity_ah: 100.0,
            nominal_voltage_v: 3.6,
            dt_s: 1.0,
            q_noise: 1e-6,
            r_noise: 1e-4,
            initial_soc: 0.8,
            initial_soh: 1.0,
            chemistry: BatteryChemistryType::Nmc,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Output structures
// ─────────────────────────────────────────────────────────────────────────────

/// Comprehensive pack-level state estimate returned by [`BatteryPackEstimator::update`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PackStateEstimate {
    /// State of Charge ∈ \[0, 1\].
    pub soc: f64,
    /// State of Health ∈ \[0, 1\].
    pub soh: f64,
    /// Model-predicted terminal voltage \[V\].
    pub voltage_model_v: f64,
    /// Measured − model voltage residual \[V\].
    pub voltage_residual_v: f64,
    /// RC-pair 1 over-voltage \[V\].
    pub v_rc1_v: f64,
    /// RC-pair 2 over-voltage \[V\].
    pub v_rc2_v: f64,
    /// EKF innovation (same as voltage_residual before state update) \[V\].
    pub innovation: f64,
    /// Trace of the EKF covariance matrix — overall estimation uncertainty.
    pub covariance_trace: f64,
}

/// Intermediate result from the EKF measurement-update step.
#[derive(Debug, Clone)]
pub struct EkfResult {
    /// Updated state vector `[SoC, V_RC1, V_RC2]`.
    pub state: [f64; 3],
    /// Innovation (pre-fit residual) \[V\].
    pub innovation: f64,
    /// Kalman gain vector `[K_SoC, K_V_RC1, K_V_RC2]`.
    pub kalman_gain: [f64; 3],
    /// Updated 3×3 error covariance matrix P.
    pub covariance: [[f64; 3]; 3],
}

/// Result of the sanity-check on a state estimate.
#[derive(Debug, Clone)]
pub struct ValidationResult {
    /// `true` if all checks pass.
    pub valid: bool,
    /// `true` if SoC ∈ \[0, 1\].
    pub soc_in_range: bool,
    /// Model voltage residual \[V\].
    pub voltage_residual_v: f64,
    /// `true` if |residual| ≤ 50 mV.
    pub residual_ok: bool,
}

// ─────────────────────────────────────────────────────────────────────────────
// 3×3 matrix helpers (no external linear-algebra crate)
// ─────────────────────────────────────────────────────────────────────────────

type Mat3 = [[f64; 3]; 3];

/// Identity matrix.
#[inline]
fn mat3_identity() -> Mat3 {
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
}

/// Matrix multiply A × B.
#[inline]
fn mat3_mul(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut c = [[0.0_f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                c[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    c
}

/// Matrix transpose.
#[inline]
fn mat3_transpose(a: &Mat3) -> Mat3 {
    let mut t = [[0.0_f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            t[j][i] = a[i][j];
        }
    }
    t
}

/// Scale every element of a matrix.
#[inline]
fn mat3_scale(a: &Mat3, s: f64) -> Mat3 {
    let mut r = *a;
    for row in r.iter_mut() {
        for v in row.iter_mut() {
            *v *= s;
        }
    }
    r
}

/// Element-wise matrix addition.
#[inline]
fn mat3_add(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut r = [[0.0_f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            r[i][j] = a[i][j] + b[i][j];
        }
    }
    r
}

/// Element-wise matrix subtraction.
#[inline]
fn mat3_sub(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut r = [[0.0_f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            r[i][j] = a[i][j] - b[i][j];
        }
    }
    r
}

/// Outer product of two 3-vectors: result[i][j] = a[i] * b[j].
#[inline]
fn vec3_outer(a: &[f64; 3], b: &[f64; 3]) -> Mat3 {
    let mut m = [[0.0_f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            m[i][j] = a[i] * b[j];
        }
    }
    m
}

/// Matrix × column-vector.
#[inline]
fn mat3_vec_mul(a: &Mat3, v: &[f64; 3]) -> [f64; 3] {
    let mut r = [0.0_f64; 3];
    for i in 0..3 {
        for j in 0..3 {
            r[i] += a[i][j] * v[j];
        }
    }
    r
}

/// Row-vector × Matrix: result[j] = sum_i h[i] * a[i][j].
#[allow(dead_code)]
#[inline]
fn vec3_mat3_mul(h: &[f64; 3], a: &Mat3) -> [f64; 3] {
    let mut r = [0.0_f64; 3];
    for j in 0..3 {
        for i in 0..3 {
            r[j] += h[i] * a[i][j];
        }
    }
    r
}

/// Dot product of two 3-vectors.
#[inline]
fn vec3_dot(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Trace of a 3×3 matrix.
#[inline]
fn mat3_trace(a: &Mat3) -> f64 {
    a[0][0] + a[1][1] + a[2][2]
}

// ─────────────────────────────────────────────────────────────────────────────
// Process-noise covariance builder
// ─────────────────────────────────────────────────────────────────────────────

/// Build a diagonal 3×3 process-noise matrix scaled by `q`.
///
/// The noise on the RC voltages is one order of magnitude smaller than on SoC
/// because they are driven by the (known) input current.
#[inline]
fn build_q_matrix(q: f64) -> Mat3 {
    [[q, 0.0, 0.0], [0.0, q * 0.1, 0.0], [0.0, 0.0, q * 0.1]]
}

// ─────────────────────────────────────────────────────────────────────────────
// Main estimator
// ─────────────────────────────────────────────────────────────────────────────

/// EKF-based battery pack state estimator using a 2RC Thevenin ECM.
///
/// # State vector
/// `x = [SoC, V_RC1 \[V\], V_RC2 \[V\]]`
///
/// # Usage
/// ```rust,ignore
/// let mut est = BatteryPackEstimator::new(config, ecm_params)?;
/// let estimate = est.update(current_a, voltage_v)?;
/// ```
#[derive(Debug, Clone)]
pub struct BatteryPackEstimator {
    /// Estimator configuration.
    pub config: BatteryStateEstimatorConfig,
    /// ECM parameters (may be updated adaptively).
    pub ecm_params: EcmParameters,
    /// EKF state vector `[SoC, V_RC1, V_RC2]`.
    state: [f64; 3],
    /// 3×3 error covariance matrix P.
    covariance: Mat3,
    /// Current State of Health estimate ∈ \[0, 1\].
    soh: f64,
    /// Equivalent full-cycle count (accumulated).
    cycle_count: f64,
    /// Total charge throughput \[Ah\].
    q_throughput_ah: f64,
}

impl BatteryPackEstimator {
    // ── Construction ──────────────────────────────────────────────────────────

    /// Create a new estimator.
    ///
    /// Returns an error if the ECM parameters fail validation.
    pub fn new(
        config: BatteryStateEstimatorConfig,
        ecm_params: EcmParameters,
    ) -> Result<Self, StateEstimationError> {
        ecm_params.validate()?;

        let soc0 = config.initial_soc.clamp(0.0, 1.0);
        let soh0 = config.initial_soh.clamp(0.0, 1.0);

        // Initialise P as a scaled identity — moderate uncertainty.
        let p0 = mat3_scale(&mat3_identity(), 0.01);

        Ok(Self {
            state: [soc0, 0.0, 0.0],
            covariance: p0,
            soh: soh0,
            cycle_count: 0.0,
            q_throughput_ah: 0.0,
            config,
            ecm_params,
        })
    }

    /// Create an estimator with default ECM parameters for the given chemistry.
    pub fn with_chemistry(
        config: BatteryStateEstimatorConfig,
    ) -> Result<Self, StateEstimationError> {
        let ecm_params = Self::default_ecm_for_chemistry(config.chemistry);
        Self::new(config, ecm_params)
    }

    // ── OCV helpers ───────────────────────────────────────────────────────────

    /// Linearly interpolate OCV from the estimator's own table.
    pub fn ocv_at_soc(&self, soc: f64) -> Result<f64, StateEstimationError> {
        Self::ocv_from_soc(soc, &self.ecm_params.ocv_soc_table)
    }

    /// Linearly interpolate OCV at `soc` from `table`.
    ///
    /// `soc` is clamped to \[0, 1\] before interpolation.
    pub fn ocv_from_soc(soc: f64, table: &[(f64, f64)]) -> Result<f64, StateEstimationError> {
        if table.len() < 2 {
            return Err(StateEstimationError::EmptyOcvTable);
        }
        let soc = soc.clamp(0.0, 1.0);

        // Fast path: out-of-range clamping.
        if soc <= table[0].0 {
            return Ok(table[0].1);
        }
        if soc >= table[table.len() - 1].0 {
            return Ok(table[table.len() - 1].1);
        }

        // Binary search for the bracket.
        let pos = table.partition_point(|&(s, _)| s <= soc);
        let (s0, v0) = table[pos - 1];
        let (s1, v1) = table[pos];
        let alpha = (soc - s0) / (s1 - s0);
        Ok(v0 + alpha * (v1 - v0))
    }

    /// Numerical derivative dOCV/dSoC at `soc` using the estimator's table.
    fn docv_dsoc(&self, soc: f64) -> Result<f64, StateEstimationError> {
        let eps = 1e-4_f64;
        let soc_c = soc.clamp(eps, 1.0 - eps);
        let v_hi = Self::ocv_from_soc(soc_c + eps, &self.ecm_params.ocv_soc_table)?;
        let v_lo = Self::ocv_from_soc(soc_c - eps, &self.ecm_params.ocv_soc_table)?;
        Ok((v_hi - v_lo) / (2.0 * eps))
    }

    // ── EKF predict step ──────────────────────────────────────────────────────

    /// EKF prediction step.
    ///
    /// Propagates the state and covariance forward by `dt_s` \[s\] given
    /// `current_a` \[A\] (positive = discharge).
    ///
    /// Returns the predicted state vector `[SoC, V_RC1, V_RC2]`.
    pub fn ekf_predict(&mut self, current_a: f64, dt_s: f64) -> [f64; 3] {
        let cap_ah = self.config.capacity_ah;
        let soh = self.soh;

        let r1 = self.ecm_params.r1_ohm;
        let c1 = self.ecm_params.c1_f;
        let r2 = self.ecm_params.r2_ohm;
        let c2 = self.ecm_params.c2_f;

        // Time constants.
        let tau1 = r1 * c1;
        let tau2 = r2 * c2;

        let exp1 = (-dt_s / tau1).exp();
        let exp2 = (-dt_s / tau2).exp();

        let soc = self.state[0];
        let vrc1 = self.state[1];
        let vrc2 = self.state[2];

        // State transition (discrete-time exact ZOH):
        // SoC[k+1]  = SoC[k]  - I·dt / (3600·C_nom·SoH)
        // V_RC1[k+1]= V_RC1[k]·exp(-dt/τ1) + R1·(1-exp(-dt/τ1))·I
        // V_RC2[k+1]= V_RC2[k]·exp(-dt/τ2) + R2·(1-exp(-dt/τ2))·I
        let soc_next = soc - current_a * dt_s / (3600.0 * cap_ah * soh);
        let vrc1_next = vrc1 * exp1 + r1 * (1.0 - exp1) * current_a;
        let vrc2_next = vrc2 * exp2 + r2 * (1.0 - exp2) * current_a;

        self.state = [soc_next, vrc1_next, vrc2_next];

        // State-transition Jacobian F (3×3).
        // Only the diagonal terms are non-trivial; cross terms vanish.
        let f: Mat3 = [[1.0, 0.0, 0.0], [0.0, exp1, 0.0], [0.0, 0.0, exp2]];

        // Covariance prediction: P = F·P·F^T + Q
        let q = build_q_matrix(self.config.q_noise);
        let ft = mat3_transpose(&f);
        let fp = mat3_mul(&f, &self.covariance);
        let fpft = mat3_mul(&fp, &ft);
        self.covariance = mat3_add(&fpft, &q);

        self.state
    }

    // ── EKF update step ───────────────────────────────────────────────────────

    /// EKF measurement-update step.
    ///
    /// `measured_voltage_v` \[V\] is the observed terminal voltage.
    /// `current_a` \[A\] is the applied current (positive = discharge).
    ///
    /// Returns an [`EkfResult`] containing the corrected state, innovation, and
    /// updated covariance.
    pub fn ekf_update(
        &mut self,
        measured_voltage_v: f64,
        current_a: f64,
    ) -> Result<EkfResult, StateEstimationError> {
        let soc = self.state[0];
        let vrc1 = self.state[1];
        let vrc2 = self.state[2];

        let ocv = Self::ocv_from_soc(soc, &self.ecm_params.ocv_soc_table)?;
        let v_model = ocv - current_a * self.ecm_params.r0_ohm - vrc1 - vrc2;

        // Innovation (pre-fit residual).
        let innovation = measured_voltage_v - v_model;

        // Measurement Jacobian H = [∂V_t/∂SoC, ∂V_t/∂V_RC1, ∂V_t/∂V_RC2]
        //                        = [dOCV/dSoC,  -1,            -1          ]
        let docv = self.docv_dsoc(soc)?;
        let h: [f64; 3] = [docv, -1.0, -1.0];

        // Innovation covariance: S = H·P·H^T + R  (scalar for 1-D measurement)
        let ph = mat3_vec_mul(&self.covariance, &h); // P·H^T (column)
        let s = vec3_dot(&h, &ph) + self.config.r_noise;

        if s.abs() < 1e-15 {
            return Err(StateEstimationError::SingularMatrix);
        }

        // Kalman gain: K = P·H^T / S
        let k: [f64; 3] = [ph[0] / s, ph[1] / s, ph[2] / s];

        // State update: x = x_pred + K·y
        let soc_new = (soc + k[0] * innovation).clamp(0.0, 1.0);
        let vrc1_new = vrc1 + k[1] * innovation;
        let vrc2_new = vrc2 + k[2] * innovation;
        self.state = [soc_new, vrc1_new, vrc2_new];

        // Covariance update: P = (I - K·H)·P
        let kh = vec3_outer(&k, &h);
        let i_kh = mat3_sub(&mat3_identity(), &kh);
        self.covariance = mat3_mul(&i_kh, &self.covariance);

        Ok(EkfResult {
            state: self.state,
            innovation,
            kalman_gain: k,
            covariance: self.covariance,
        })
    }

    // ── SoH update ────────────────────────────────────────────────────────────

    /// Incremental SoH degradation from Coulomb throughput.
    ///
    /// `cycle_degradation_per_ah` \[1/Ah\] is the fractional capacity loss
    /// per unit of charge throughput (absolute value of current × time).
    ///
    /// Returns the updated SoH, clamped to \[0, 1\].
    pub fn estimate_soh(
        current_soh: f64,
        capacity_fade: f64,
        cycle_degradation_per_ah: f64,
    ) -> f64 {
        (current_soh - cycle_degradation_per_ah * capacity_fade.abs()).clamp(0.0, 1.0)
    }

    // ── Adaptive R0 update ────────────────────────────────────────────────────

    /// Adaptive update of the series resistance R0 using a forgetting-factor
    /// RLS-like rule.
    ///
    /// If `|innovation|` is above the threshold (10 mV) and `|current_a|` is
    /// non-negligible, R0 is nudged toward the implied impedance estimate.
    ///
    /// R0 is bounded to `[0.001, 0.1] Ω`.
    ///
    /// Returns the updated [`EcmParameters`].
    pub fn adaptive_parameter_update(
        &mut self,
        innovation: f64,
        current_a: f64,
        _dt_s: f64,
    ) -> EcmParameters {
        const INNOVATION_THRESHOLD: f64 = 0.010; // 10 mV
        const ALPHA: f64 = 0.05; // learning rate
        const R0_MIN: f64 = 0.001; // \[Ω\]
        const R0_MAX: f64 = 0.100; // \[Ω\]

        if innovation.abs() > INNOVATION_THRESHOLD && current_a.abs() > 0.1 {
            // Implied R0 from the voltage drop discrepancy.
            let r0_implied = (innovation.abs() / current_a.abs()).clamp(R0_MIN, R0_MAX);
            let r0_new = self.ecm_params.r0_ohm + ALPHA * (r0_implied - self.ecm_params.r0_ohm);
            self.ecm_params.r0_ohm = r0_new.clamp(R0_MIN, R0_MAX);
        }

        self.ecm_params.clone()
    }

    // ── Main update ───────────────────────────────────────────────────────────

    /// Combined predict + update + SoH step.
    ///
    /// Call this once per sampling interval with the measured terminal voltage
    /// and applied current.
    ///
    /// `current_a` \[A\] positive = discharge.
    /// `voltage_v` \[V\] measured terminal voltage.
    ///
    /// Returns a [`PackStateEstimate`] on success.
    pub fn update(
        &mut self,
        current_a: f64,
        voltage_v: f64,
    ) -> Result<PackStateEstimate, StateEstimationError> {
        let dt = self.config.dt_s;

        // 1. Prediction step.
        self.ekf_predict(current_a, dt);

        // 2. Measurement-update step.
        let ekf_result = self.ekf_update(voltage_v, current_a)?;

        // 3. Adaptive parameter update.
        self.adaptive_parameter_update(ekf_result.innovation, current_a, dt);

        // 4. SoH update from Coulomb throughput.
        //    Typical li-ion: ~0.0001 % SoH loss per Ah of throughput (≈1000 cycles at 100 Ah).
        let dq_ah = (current_a * dt / 3600.0).abs();
        self.q_throughput_ah += dq_ah;
        // 1e-7 /Ah gives ~1000 equivalent full cycles until SoH = 0 for 100 Ah pack.
        const DEGRAD_PER_AH: f64 = 1e-7;
        self.soh = Self::estimate_soh(self.soh, dq_ah, DEGRAD_PER_AH);
        // Equivalent full cycles.
        self.cycle_count += dq_ah / (self.config.capacity_ah * 2.0);

        // 5. Compute final model voltage with updated state.
        let soc = ekf_result.state[0];
        let vrc1 = ekf_result.state[1];
        let vrc2 = ekf_result.state[2];
        let ocv = Self::ocv_from_soc(soc, &self.ecm_params.ocv_soc_table)?;
        let v_model = ocv - current_a * self.ecm_params.r0_ohm - vrc1 - vrc2;
        let residual = voltage_v - v_model;

        Ok(PackStateEstimate {
            soc,
            soh: self.soh,
            voltage_model_v: v_model,
            voltage_residual_v: residual,
            v_rc1_v: vrc1,
            v_rc2_v: vrc2,
            innovation: ekf_result.innovation,
            covariance_trace: mat3_trace(&ekf_result.covariance),
        })
    }

    // ── Validation ────────────────────────────────────────────────────────────

    /// Sanity-check an estimate against physical constraints.
    ///
    /// Flags the estimate as invalid if SoC ∉ \[0, 1\] or |voltage_residual| > 50 mV.
    pub fn validate_estimate(
        &self,
        state: &[f64; 3],
        measured_voltage: f64,
        current_a: f64,
    ) -> Result<ValidationResult, StateEstimationError> {
        let soc = state[0];
        let vrc1 = state[1];
        let vrc2 = state[2];

        let soc_in_range = (0.0..=1.0).contains(&soc);

        let ocv = Self::ocv_from_soc(soc.clamp(0.0, 1.0), &self.ecm_params.ocv_soc_table)?;
        let v_model = ocv - current_a * self.ecm_params.r0_ohm - vrc1 - vrc2;
        let residual = measured_voltage - v_model;

        const RESIDUAL_LIMIT: f64 = 0.050; // 50 mV
        let residual_ok = residual.abs() <= RESIDUAL_LIMIT;

        Ok(ValidationResult {
            valid: soc_in_range && residual_ok,
            soc_in_range,
            voltage_residual_v: residual,
            residual_ok,
        })
    }

    // ── Accessors ─────────────────────────────────────────────────────────────

    /// Current SoC estimate.
    pub fn soc(&self) -> f64 {
        self.state[0]
    }

    /// Current SoH estimate.
    pub fn soh(&self) -> f64 {
        self.soh
    }

    /// Total charge throughput \[Ah\].
    pub fn q_throughput_ah(&self) -> f64 {
        self.q_throughput_ah
    }

    /// Equivalent full-cycle count.
    pub fn cycle_count(&self) -> f64 {
        self.cycle_count
    }

    /// Covariance trace (estimation uncertainty).
    pub fn covariance_trace(&self) -> f64 {
        mat3_trace(&self.covariance)
    }

    // ── Built-in OCV tables ───────────────────────────────────────────────────

    /// Return a built-in OCV-SoC table for the specified chemistry.
    ///
    /// Tables are (SoC, OCV \[V\]) pairs sampled at representative operating points.
    pub fn generate_ocv_table(chemistry: BatteryChemistryType) -> Vec<(f64, f64)> {
        match chemistry {
            BatteryChemistryType::Lfp => vec![
                // Flat plateau characteristic of LFP; sharp drops near 0 % and 100 %.
                (0.00, 3.000),
                (0.05, 3.100),
                (0.10, 3.180),
                (0.15, 3.220),
                (0.20, 3.250),
                (0.30, 3.265),
                (0.40, 3.278),
                (0.50, 3.290),
                (0.60, 3.300),
                (0.70, 3.310),
                (0.80, 3.325),
                (0.85, 3.340),
                (0.90, 3.360),
                (0.95, 3.400),
                (1.00, 3.650),
            ],
            BatteryChemistryType::Nmc => vec![
                // Roughly linear between 3.0 V (empty) and 4.2 V (full).
                (0.00, 3.000),
                (0.05, 3.400),
                (0.10, 3.520),
                (0.20, 3.620),
                (0.30, 3.680),
                (0.40, 3.720),
                (0.50, 3.760),
                (0.60, 3.810),
                (0.70, 3.860),
                (0.80, 3.920),
                (0.90, 3.980),
                (0.95, 4.060),
                (1.00, 4.200),
            ],
            BatteryChemistryType::Nca => vec![
                // Slightly higher energy than NMC; peak ~4.2 V.
                (0.00, 2.950),
                (0.05, 3.350),
                (0.10, 3.500),
                (0.20, 3.600),
                (0.30, 3.660),
                (0.40, 3.710),
                (0.50, 3.750),
                (0.60, 3.800),
                (0.70, 3.860),
                (0.80, 3.930),
                (0.90, 4.000),
                (0.95, 4.080),
                (1.00, 4.200),
            ],
            BatteryChemistryType::Lto => vec![
                // Li4Ti5O12 anode; very flat 2.3 V plateau, 1.5–2.7 V range.
                (0.00, 1.500),
                (0.05, 2.100),
                (0.10, 2.250),
                (0.20, 2.310),
                (0.30, 2.320),
                (0.40, 2.330),
                (0.50, 2.335),
                (0.60, 2.340),
                (0.70, 2.345),
                (0.80, 2.350),
                (0.90, 2.360),
                (0.95, 2.400),
                (1.00, 2.700),
            ],
        }
    }

    // ── Default ECM per chemistry ─────────────────────────────────────────────

    /// Return default ECM parameters (2RC Thevenin) for the given chemistry.
    pub fn default_ecm_for_chemistry(chemistry: BatteryChemistryType) -> EcmParameters {
        let ocv_table = Self::generate_ocv_table(chemistry);
        match chemistry {
            BatteryChemistryType::Lfp => EcmParameters {
                r0_ohm: 0.005,
                r1_ohm: 0.003,
                c1_f: 3000.0,
                r2_ohm: 0.002,
                c2_f: 30000.0,
                ocv_soc_table: ocv_table,
            },
            BatteryChemistryType::Nmc => EcmParameters {
                r0_ohm: 0.006,
                r1_ohm: 0.004,
                c1_f: 2500.0,
                r2_ohm: 0.002,
                c2_f: 25000.0,
                ocv_soc_table: ocv_table,
            },
            BatteryChemistryType::Nca => EcmParameters {
                r0_ohm: 0.005,
                r1_ohm: 0.003,
                c1_f: 2800.0,
                r2_ohm: 0.002,
                c2_f: 28000.0,
                ocv_soc_table: ocv_table,
            },
            BatteryChemistryType::Lto => EcmParameters {
                r0_ohm: 0.003,
                r1_ohm: 0.002,
                c1_f: 5000.0,
                r2_ohm: 0.001,
                c2_f: 50000.0,
                ocv_soc_table: ocv_table,
            },
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a default NMC estimator starting at SoC = 0.8, SoH = 1.0.
    fn make_nmc_estimator(soc: f64) -> BatteryPackEstimator {
        let config = BatteryStateEstimatorConfig {
            capacity_ah: 100.0,
            nominal_voltage_v: 3.7,
            dt_s: 1.0,
            q_noise: 1e-6,
            r_noise: 1e-4,
            initial_soc: soc,
            initial_soh: 1.0,
            chemistry: BatteryChemistryType::Nmc,
        };
        BatteryPackEstimator::with_chemistry(config).expect("default estimator should be valid")
    }

    // ── Test 1: EKF predict – SoC decreases during discharge ─────────────────
    #[test]
    fn test_ekf_predict_soc_decreases_on_discharge() {
        let mut est = make_nmc_estimator(0.8);
        let soc_before = est.soc();

        // Discharge at 50 A for 1 s.
        est.ekf_predict(50.0, 1.0);

        let soc_after = est.soc();
        assert!(
            soc_after < soc_before,
            "SoC should decrease during discharge: {soc_before} → {soc_after}"
        );

        // Sanity-check magnitude: ΔSoC ≈ 50/(3600·100·1) ≈ 1.39e-4
        let expected_delta = 50.0 / (3600.0 * 100.0 * 1.0);
        let actual_delta = soc_before - soc_after;
        assert!(
            (actual_delta - expected_delta).abs() < 1e-6,
            "ΔSoC mismatch: expected {expected_delta:.2e}, got {actual_delta:.2e}"
        );
    }

    // ── Test 2: EKF predict – SoC increases during charging ──────────────────
    #[test]
    fn test_ekf_predict_soc_increases_on_charge() {
        let mut est = make_nmc_estimator(0.5);
        let soc_before = est.soc();

        // Charge at -30 A for 1 s (negative = charge convention).
        est.ekf_predict(-30.0, 1.0);

        assert!(
            est.soc() > soc_before,
            "SoC should increase during charging"
        );
    }

    // ── Test 3: EKF update – innovation drives state toward truth ────────────
    #[test]
    fn test_ekf_update_innovation_drives_correction() {
        let mut est = make_nmc_estimator(0.5);

        // Manually move state to a "wrong" SoC.
        est.state[0] = 0.45;

        // Observed voltage corresponds to SoC ≈ 0.50 on the NMC curve:
        // V ≈ OCV(0.50) - I·R0 = 3.76 - 10·0.006 = 3.70
        let current_a = 10.0;
        let v_measured = 3.70;

        let result = est
            .ekf_update(v_measured, current_a)
            .expect("EKF update should succeed");

        // Innovation should be positive (model underestimated voltage).
        assert!(
            result.innovation.abs() > 0.0,
            "Non-zero innovation expected"
        );

        // SoC should shift from 0.45 toward a higher value.
        assert!(
            result.state[0] > 0.45,
            "State SoC should increase after positive innovation: {}",
            result.state[0]
        );
    }

    // ── Test 4: OCV interpolation at SoC = 0.5 ───────────────────────────────
    #[test]
    fn test_ocv_interpolation_at_mid_soc() {
        let table = BatteryPackEstimator::generate_ocv_table(BatteryChemistryType::Nmc);
        let ocv = BatteryPackEstimator::ocv_from_soc(0.5, &table)
            .expect("Should interpolate successfully");

        // NMC table has (0.50, 3.760) exactly.
        assert!(
            (ocv - 3.760).abs() < 1e-6,
            "OCV at SoC=0.50 should be 3.760 V, got {ocv}"
        );
    }

    // ── Test 5: LFP OCV is nearly flat between 20 % and 80 % ─────────────────
    #[test]
    fn test_ocv_lfp_plateau_is_flat() {
        let table = BatteryPackEstimator::generate_ocv_table(BatteryChemistryType::Lfp);

        let ocv20 = BatteryPackEstimator::ocv_from_soc(0.20, &table).expect("interp ok");
        let ocv50 = BatteryPackEstimator::ocv_from_soc(0.50, &table).expect("interp ok");
        let ocv80 = BatteryPackEstimator::ocv_from_soc(0.80, &table).expect("interp ok");

        // Max spread in plateau should be < 100 mV.
        let spread = ocv80 - ocv20;
        assert!(
            spread < 0.100,
            "LFP plateau too wide: {spread:.4} V between 20% and 80% SoC"
        );

        // All plateau voltages should be in 3.2–3.4 V range.
        for &ocv in &[ocv20, ocv50, ocv80] {
            assert!(
                (3.20..=3.40).contains(&ocv),
                "LFP plateau OCV out of expected range: {ocv}"
            );
        }
    }

    // ── Test 6: SoH decreases with Coulomb throughput ────────────────────────
    #[test]
    fn test_soh_decreases_with_throughput() {
        // Start at SoH = 1.0, accumulate 1000 Ah of throughput.
        let soh_start = 1.0_f64;
        let cap_fade_ah = 1000.0_f64; // 1000 Ah throughput
        const DEGRAD: f64 = 1e-7; // same constant used in update()

        let soh_end = BatteryPackEstimator::estimate_soh(soh_start, cap_fade_ah, DEGRAD);

        assert!(soh_end < soh_start, "SoH should decrease after throughput");
        assert!(
            soh_end > 0.0,
            "SoH should remain positive for moderate throughput"
        );

        // With 1000 Ah throughput: ΔSOH = 1e-7 * 1000 = 1e-4 → SoH ≈ 0.9999
        let expected = (soh_start - DEGRAD * cap_fade_ah).clamp(0.0, 1.0);
        assert!(
            (soh_end - expected).abs() < 1e-12,
            "SoH mismatch: {soh_end} vs {expected}"
        );
    }

    // ── Test 7: Adaptive R0 adjusts on large innovation ──────────────────────
    #[test]
    fn test_adaptive_r0_adjusts_on_large_innovation() {
        let mut est = make_nmc_estimator(0.8);
        let r0_before = est.ecm_params.r0_ohm;

        // Large innovation (50 mV) with significant current (20 A) → R0 should change.
        est.adaptive_parameter_update(0.050, 20.0, 1.0);
        let r0_after = est.ecm_params.r0_ohm;

        assert!(
            (r0_after - r0_before).abs() > 1e-10,
            "R0 should change after large innovation: before={r0_before}, after={r0_after}"
        );

        // R0 must stay within physical bounds.
        assert!(r0_after >= 0.001, "R0 below minimum: {r0_after}");
        assert!(r0_after <= 0.100, "R0 above maximum: {r0_after}");
    }

    // ── Test 8: Validation rejects SoC > 1 ───────────────────────────────────
    #[test]
    fn test_validation_rejects_out_of_range_soc() {
        let est = make_nmc_estimator(0.5);
        let bad_state = [1.1_f64, 0.0, 0.0]; // SoC = 110% — impossible

        let result = est
            .validate_estimate(&bad_state, 3.76, 0.0)
            .expect("validate should not error");

        assert!(!result.valid, "Estimate with SoC=1.1 should be invalid");
        assert!(
            !result.soc_in_range,
            "soc_in_range should be false for SoC=1.1"
        );
    }

    // ── Test 9: Full charge-discharge cycle – SoC returns near initial ────────
    //
    // Strategy: use Coulomb counting alone to verify that SoC integration is
    // internally consistent.  We simulate the terminal voltage using the full
    // 2RC model state so that the EKF sees near-zero innovations and does not
    // apply large corrections.
    #[test]
    fn test_full_cycle_soc_recovery() {
        let mut est = make_nmc_estimator(0.50);

        let cap = 100.0_f64; // Ah
        let current_charge = -10.0_f64; // 10 A charge (negative convention)
        let current_disch = 10.0_f64; // 10 A discharge
                                      // Use fewer steps to avoid accumulating numerical issues; 360 s = 6 min.
                                      // ΔSoC per step: |I|·dt / (3600·C) = 10·1/(3600·100) = 2.78e-5
                                      // Total 360 steps → ΔSoC ≈ 0.01 (1 %).
        let steps = 360_usize;

        // Simulate terminal voltage using the CURRENT internal model state so
        // that the EKF innovation stays near zero and the SoC integrates cleanly.
        /// Compute the predicted terminal voltage that the EKF measurement
        /// update will see, given the current state, so that the innovation
        /// is effectively zero and EKF does not perturb the Coulomb-counted SoC.
        #[allow(clippy::too_many_arguments)]
        fn predicted_vt(
            soc_now: f64,
            vrc1_now: f64,
            vrc2_now: f64,
            current_a: f64,
            dt: f64,
            r0: f64,
            r1: f64,
            c1: f64,
            r2: f64,
            c2: f64,
            cap_ah: f64,
            soh: f64,
            ocv_table: &[(f64, f64)],
        ) -> f64 {
            // Replicate the predict equations to get the post-predict state.
            let tau1 = r1 * c1;
            let tau2 = r2 * c2;
            let exp1 = (-dt / tau1).exp();
            let exp2 = (-dt / tau2).exp();
            let soc_p = soc_now - current_a * dt / (3600.0 * cap_ah * soh);
            let vrc1_p = vrc1_now * exp1 + r1 * (1.0 - exp1) * current_a;
            let vrc2_p = vrc2_now * exp2 + r2 * (1.0 - exp2) * current_a;
            let ocv_p = BatteryPackEstimator::ocv_from_soc(soc_p, ocv_table).unwrap_or(3.76);
            ocv_p - current_a * r0 - vrc1_p - vrc2_p
        }

        // Charge for `steps` seconds.
        for _ in 0..steps {
            let v_sim = predicted_vt(
                est.state[0],
                est.state[1],
                est.state[2],
                current_charge,
                est.config.dt_s,
                est.ecm_params.r0_ohm,
                est.ecm_params.r1_ohm,
                est.ecm_params.c1_f,
                est.ecm_params.r2_ohm,
                est.ecm_params.c2_f,
                est.config.capacity_ah,
                est.soh,
                &est.ecm_params.ocv_soc_table,
            );
            let _ = est.update(current_charge, v_sim);
        }

        let soc_after_charge = est.soc();

        // Discharge for the same duration.
        for _ in 0..steps {
            let v_sim = predicted_vt(
                est.state[0],
                est.state[1],
                est.state[2],
                current_disch,
                est.config.dt_s,
                est.ecm_params.r0_ohm,
                est.ecm_params.r1_ohm,
                est.ecm_params.c1_f,
                est.ecm_params.r2_ohm,
                est.ecm_params.c2_f,
                est.config.capacity_ah,
                est.soh,
                &est.ecm_params.ocv_soc_table,
            );
            let _ = est.update(current_disch, v_sim);
        }

        let soc_after_cycle = est.soc();

        // Expected ΔSoC per direction: 10 A × 360 s / (3600·100 Ah) = 0.01
        let expected_delta = current_disch * (steps as f64) / (3600.0 * cap);

        assert!(
            soc_after_charge > 0.50,
            "SoC should rise during charging: {soc_after_charge:.4}"
        );
        assert!(
            (soc_after_cycle - 0.50).abs() < 0.02,
            "SoC should return near 0.50 after symmetric cycle, got {soc_after_cycle:.4}"
        );
        // Charge increment should be ~expected_delta.
        assert!(
            (soc_after_charge - 0.50 - expected_delta).abs() < 0.005,
            "Charge SoC increment wrong: expected Δ{expected_delta:.4}, actual Δ{:.4}",
            soc_after_charge - 0.50
        );
    }

    // ── Test 10: OCV table with empty list returns error ──────────────────────
    #[test]
    fn test_ocv_from_soc_empty_table_returns_error() {
        let table: Vec<(f64, f64)> = vec![];
        let result = BatteryPackEstimator::ocv_from_soc(0.5, &table);
        assert!(
            matches!(result, Err(StateEstimationError::EmptyOcvTable)),
            "Expected EmptyOcvTable error, got {result:?}"
        );
    }

    // ── Test 11: EcmParameters::validate catches bad parameters ───────────────
    #[test]
    fn test_ecm_parameters_validate_rejects_negatives() {
        let bad = EcmParameters {
            r0_ohm: -0.001,
            r1_ohm: 0.003,
            c1_f: 3000.0,
            r2_ohm: 0.002,
            c2_f: 30000.0,
            ocv_soc_table: vec![(0.0, 3.0), (1.0, 4.2)],
        };
        assert!(
            bad.validate().is_err(),
            "Negative R0 should fail validation"
        );
    }

    // ── Test 12: Covariance trace decreases (or stays bounded) after many updates ──
    #[test]
    fn test_covariance_trace_bounded_after_updates() {
        let mut est = make_nmc_estimator(0.8);

        // Run 100 update steps with a consistent simulated measurement.
        for _ in 0..100 {
            let soc = est.soc();
            let ocv =
                BatteryPackEstimator::ocv_from_soc(soc, &est.ecm_params.ocv_soc_table.clone())
                    .unwrap_or(3.76);
            let v_meas = ocv - 5.0 * est.ecm_params.r0_ohm;
            let _ = est.update(5.0, v_meas);
        }

        // Covariance trace should be small (well converged) after 100 consistent steps.
        let trace = est.covariance_trace();
        assert!(
            trace < 0.01,
            "Covariance trace should be small after convergence: {trace}"
        );
    }
}
