/// Online battery ECM parameter identification.
///
/// Implements two complementary online estimation algorithms:
///
/// 1. **Recursive Least Squares (RLS)** — updates the 2RC Thevenin parameter
///    vector θ = [R0, R1, C1, R2, C2] on every new current/voltage sample.
///    Uses a forgetting factor λ ∈ (0, 1] to track slowly-varying parameters.
///
/// 2. **Dual Extended Kalman Filter (DEKF)** — runs two coupled EKFs in
///    parallel: one estimates the SoC (state EKF) and one estimates the
///    ECM parameters (parameter EKF).  The estimates are exchanged at each
///    step, enabling simultaneous state and parameter tracking.
///
/// 3. **OCV-SoC hysteresis model** — captures the hysteresis between the
///    charge and discharge OCV curves using a first-order exponential model.
///
/// # References
/// - Plett, "Battery Management Systems Vol. 1", Chapter 4 (RLS, DEKF).
/// - Hu et al., "A comparative study of equivalent circuit models for
///   Li-ion batteries", J. Power Sources, 2012.
use serde::{Deserialize, Serialize};

/// RLS estimated parameter vector for a 2RC Thevenin model.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EcmParams {
    /// Ohmic resistance R0 [Ω]
    pub r0: f64,
    /// RC pair 1 resistance R1 [Ω]
    pub r1: f64,
    /// RC pair 1 capacitance C1 [F]
    pub c1: f64,
    /// RC pair 2 resistance R2 [Ω]
    pub r2: f64,
    /// RC pair 2 capacitance C2 [F]
    pub c2: f64,
}

impl EcmParams {
    /// Typical LFP cell at 25°C.
    pub fn lfp_typical() -> Self {
        Self {
            r0: 1.5e-3,
            r1: 1.0e-3,
            c1: 40_000.0,
            r2: 2.0e-3,
            c2: 5_000.0,
        }
    }

    /// Time constants τ1 = R1·C1, τ2 = R2·C2 [s].
    pub fn time_constants(&self) -> (f64, f64) {
        (self.r1 * self.c1, self.r2 * self.c2)
    }

    /// Total DC resistance R0 + R1 + R2 [Ω].
    pub fn r_total(&self) -> f64 {
        self.r0 + self.r1 + self.r2
    }
}

/// Recursive Least Squares estimator for ECM parameters.
///
/// Regresses the terminal voltage model:
///   V_t = OCV(SoC) - R0·I - V_RC1 - V_RC2
///
/// In linearised form (with known OCV subtracted):
///   y_k = V_t_k − OCV_k = −R0·I_k − α1·V_RC1_{k-1} − β1·I_k − α2·V_RC2_{k-1} − β2·I_k
///
/// The regression vector φ_k is constructed from past measurements,
/// and θ = [a1, b1, a2, b2, r0]' is updated via:
///   K_k = P_{k-1}·φ_k / (λ + φ_k'·P_{k-1}·φ_k)
///   θ_k = θ_{k-1} + K_k·(y_k − φ_k'·θ_{k-1})
///   P_k = (I − K_k·φ_k')·P_{k-1} / λ
pub struct RlsEstimator {
    /// Forgetting factor λ ∈ (0.9, 1]
    pub lambda: f64,
    /// Parameter vector [a1, b1, a2, b2, r0] (5 params)
    theta: Vec<f64>,
    /// Covariance matrix P (5×5)
    p_cov: Vec<f64>, // row-major 5×5
    /// Previous RC voltages
    v_rc1_prev: f64,
    v_rc2_prev: f64,
    /// Previous current
    i_prev: f64,
    /// Discretisation time step [s]
    dt: f64,
    /// Number of updates performed
    pub update_count: usize,
}

impl RlsEstimator {
    /// Create a new RLS estimator.
    ///
    /// - `initial`  — starting parameter guess
    /// - `lambda`   — forgetting factor (0.95–0.999 typical)
    /// - `dt`       — sample period [s]
    /// - `p0_diag` — initial covariance diagonal (large = uncertain)
    pub fn new(initial: &EcmParams, lambda: f64, dt: f64, p0_diag: f64) -> Self {
        let n = 5;
        let mut p_cov = vec![0.0f64; n * n];
        for i in 0..n {
            p_cov[i * n + i] = p0_diag;
        }

        // Discrete-time coefficients: α = exp(−dt/(R·C)), β = R·(1−α)
        let alpha1 = (-dt / (initial.r1 * initial.c1)).exp();
        let beta1 = initial.r1 * (1.0 - alpha1);
        let alpha2 = (-dt / (initial.r2 * initial.c2)).exp();
        let beta2 = initial.r2 * (1.0 - alpha2);

        Self {
            lambda,
            theta: vec![alpha1, beta1, alpha2, beta2, initial.r0],
            p_cov,
            v_rc1_prev: 0.0,
            v_rc2_prev: 0.0,
            i_prev: 0.0,
            dt,
            update_count: 0,
        }
    }

    /// Update the RLS estimate with a new measurement.
    ///
    /// - `current_a`    — measured current [A] (positive = discharge)
    /// - `voltage_v`    — measured terminal voltage [V]
    /// - `ocv_v`        — estimated OCV at current SoC [V]
    ///
    /// Returns the updated parameter estimate.
    pub fn update(&mut self, current_a: f64, voltage_v: f64, ocv_v: f64) -> EcmParams {
        let n = 5;
        // Regression vector φ = [V_RC1_{k-1}, I_{k-1}, V_RC2_{k-1}, I_{k-1}, -I_k]
        let phi = [
            self.v_rc1_prev, // multiplied by a1
            self.i_prev,     // multiplied by b1 (positive current → negative RC voltage)
            self.v_rc2_prev, // multiplied by a2
            self.i_prev,     // multiplied by b2
            -current_a,      // multiplied by r0
        ];

        // Measurement: y = V_t - OCV  (RC voltage contribution is negative)
        let y = voltage_v - ocv_v;

        // Innovation: ε = y - φ'·θ
        let y_hat: f64 = phi.iter().zip(self.theta.iter()).map(|(p, t)| p * t).sum();
        let epsilon = y - y_hat;

        // Gain: K = P·φ / (λ + φ'·P·φ)
        let p_phi = mat5_vec_mul(&self.p_cov, &phi, n);
        let phi_p_phi: f64 = phi.iter().zip(p_phi.iter()).map(|(p, pp)| p * pp).sum();
        let denom = self.lambda + phi_p_phi;

        let gain: Vec<f64> = p_phi.iter().map(|v| v / denom).collect();

        // Update theta
        for (t, &g) in self.theta.iter_mut().zip(gain.iter()) {
            *t += g * epsilon;
        }

        // Update covariance P = (I - K·φ') · P / λ
        let mut p_new = vec![0.0f64; n * n];
        for i in 0..n {
            for j in 0..n {
                let kphi_ij = gain[i] * phi[j];
                let delta = if i == j { 1.0 } else { 0.0 };
                for k in 0..n {
                    p_new[i * n + j] +=
                        (delta - kphi_ij) * self.p_cov[k * n + j] * if k == i { 1.0 } else { 0.0 };
                }
                // Simplified row-by-row update
                let _ = kphi_ij;
            }
        }
        // Direct update formula: P = (P - K·φ'·P) / λ
        let k_phi_p: Vec<f64> = {
            let mut m = vec![0.0f64; n * n];
            for i in 0..n {
                for j in 0..n {
                    for (k, &phi_k) in phi.iter().enumerate() {
                        m[i * n + j] += gain[i] * phi_k * self.p_cov[k * n + j];
                    }
                }
            }
            m
        };
        for i in 0..n {
            for j in 0..n {
                self.p_cov[i * n + j] = (self.p_cov[i * n + j] - k_phi_p[i * n + j]) / self.lambda;
            }
        }

        // Update RC voltages for next step
        let a1 = self.theta[0].clamp(0.0, 1.0);
        let b1 = self.theta[1].max(0.0);
        let a2 = self.theta[2].clamp(0.0, 1.0);
        let b2 = self.theta[3].max(0.0);

        self.v_rc1_prev = a1 * self.v_rc1_prev + b1 * self.i_prev;
        self.v_rc2_prev = a2 * self.v_rc2_prev + b2 * self.i_prev;
        self.i_prev = current_a;
        self.update_count += 1;

        self.extract_params()
    }

    /// Extract physical ECM parameters from the internal regressor coefficients.
    ///
    /// θ = [a1, b1, a2, b2, r0]
    /// a1 = exp(−dt/(R1·C1)), b1 = R1·(1−a1)
    /// → τ1 = −dt/ln(a1), R1 = b1/(1−a1), C1 = τ1/R1
    pub fn extract_params(&self) -> EcmParams {
        let a1 = self.theta[0].clamp(1e-6, 1.0 - 1e-6);
        let b1 = self.theta[1].max(1e-9);
        let a2 = self.theta[2].clamp(1e-6, 1.0 - 1e-6);
        let b2 = self.theta[3].max(1e-9);
        let r0 = self.theta[4].max(1e-9);

        let r1 = b1 / (1.0 - a1);
        let tau1 = -self.dt / a1.ln();
        let c1 = if r1 > 1e-12 { tau1 / r1 } else { 1.0 };

        let r2 = b2 / (1.0 - a2);
        let tau2 = -self.dt / a2.ln();
        let c2 = if r2 > 1e-12 { tau2 / r2 } else { 1.0 };

        EcmParams { r0, r1, c1, r2, c2 }
    }
}

/// Simple 5×5 matrix-vector multiply.
fn mat5_vec_mul(m: &[f64], v: &[f64; 5], n: usize) -> Vec<f64> {
    (0..n)
        .map(|i| (0..n).map(|j| m[i * n + j] * v[j]).sum())
        .collect()
}

/// OCV-SoC hysteresis model (first-order exponential).
///
/// The OCV hysteresis is approximated as:
///   OCV(SoC, h) = OCV_avg(SoC) + h · M(SoC)
///
/// where h is a hysteresis state that decays exponentially toward ±1:
///   dh/dt = γ · |I| / Q_nom · (sgn(I) − h)
///
/// with γ = hysteresis decay constant, Q_nom = nominal capacity [Ah].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HysteresisModel {
    /// Hysteresis state h ∈ [−1, +1]
    pub h: f64,
    /// Decay constant γ (typical 1–10)
    pub gamma: f64,
    /// Nominal cell capacity [Ah]
    pub q_nom_ah: f64,
    /// Peak hysteresis voltage [V] at each SoC breakpoint
    pub m_soc: Vec<(f64, f64)>, // (SoC fraction, M voltage [V])
}

impl HysteresisModel {
    /// Create a hysteresis model with uniform hysteresis magnitude M.
    pub fn uniform(gamma: f64, q_nom_ah: f64, m_v: f64) -> Self {
        Self {
            h: 0.0,
            gamma,
            q_nom_ah,
            m_soc: vec![(0.0, m_v), (0.5, m_v), (1.0, m_v)],
        }
    }

    /// Create with an SoC-dependent hysteresis profile.
    pub fn with_profile(gamma: f64, q_nom_ah: f64, m_soc: Vec<(f64, f64)>) -> Self {
        Self {
            h: 0.0,
            gamma,
            q_nom_ah,
            m_soc,
        }
    }

    /// Update the hysteresis state given the current and time step.
    ///
    /// - `current_a` — cell current [A] (positive = discharge)
    /// - `dt_s`      — time step [s]
    pub fn update(&mut self, current_a: f64, dt_s: f64) {
        let sign_i = if current_a > 1e-6 {
            1.0
        } else if current_a < -1e-6 {
            -1.0
        } else {
            0.0
        };
        // Euler step: h += γ · |I| / (3600 · Q) · (sgn(I) − h) · dt
        let dhdt = self.gamma * current_a.abs() / (3600.0 * self.q_nom_ah) * (sign_i - self.h);
        self.h = (self.h + dhdt * dt_s).clamp(-1.0, 1.0);
    }

    /// Interpolate hysteresis magnitude M at given SoC.
    pub fn m_at_soc(&self, soc: f64) -> f64 {
        if self.m_soc.is_empty() {
            return 0.0;
        }
        if soc <= self.m_soc[0].0 {
            return self.m_soc[0].1;
        }
        if soc >= self.m_soc[self.m_soc.len() - 1].0 {
            return self.m_soc[self.m_soc.len() - 1].1;
        }

        for i in 0..self.m_soc.len() - 1 {
            let (s0, m0) = self.m_soc[i];
            let (s1, m1) = self.m_soc[i + 1];
            if soc >= s0 && soc <= s1 {
                let t = (soc - s0) / (s1 - s0);
                return m0 + t * (m1 - m0);
            }
        }
        0.0
    }

    /// Compute hysteresis voltage correction [V] at given SoC.
    pub fn voltage_correction(&self, soc: f64) -> f64 {
        self.h * self.m_at_soc(soc)
    }
}

/// Dual Extended Kalman Filter for joint SoC + parameter estimation.
///
/// State EKF: x = [SoC, V_RC1, V_RC2], estimates state given parameters.
/// Parameter EKF: p = [R0, R1, C1, R2, C2], estimates params given state.
///
/// The two EKFs exchange estimates at each step:
///   1. State EKF uses current parameter estimate to predict next state.
///   2. Parameter EKF uses current state estimate to predict next output.
pub struct DualEkf {
    /// State EKF
    pub state_ekf: StateEkf,
    /// Parameter EKF
    pub param_ekf: ParamEkf,
    /// Discretisation time step [s]
    pub dt: f64,
    /// Total cell capacity [Ah]
    pub capacity_ah: f64,
}

impl DualEkf {
    /// Construct from initial guesses.
    pub fn new(initial_soc: f64, initial_params: &EcmParams, dt: f64, capacity_ah: f64) -> Self {
        Self {
            state_ekf: StateEkf::new(initial_soc, dt, capacity_ah),
            param_ekf: ParamEkf::new(initial_params, dt),
            dt,
            capacity_ah,
        }
    }

    /// Run one step of the dual EKF.
    ///
    /// - `current_a` — measured current [A]
    /// - `voltage_v` — measured terminal voltage [V]
    /// - `ocv_fn`    — OCV-SoC function
    ///
    /// Returns (SoC estimate, parameter estimate).
    pub fn update(
        &mut self,
        current_a: f64,
        voltage_v: f64,
        ocv_fn: &dyn Fn(f64) -> f64,
    ) -> (f64, EcmParams) {
        // Step 1: State EKF prediction using current param estimate
        let params = self.param_ekf.params();
        self.state_ekf.predict(current_a, &params, self.dt);

        // Step 2: State EKF update
        let soc = self.state_ekf.update(voltage_v, current_a, ocv_fn, &params);

        // Step 3: Param EKF prediction (parameters change slowly)
        self.param_ekf.predict();

        // Step 4: Param EKF update using current state estimate
        let est_params =
            self.param_ekf
                .update(voltage_v, current_a, soc, &self.state_ekf.state, ocv_fn);

        (soc, est_params)
    }
}

/// State EKF: estimates [SoC, V_RC1, V_RC2].
pub struct StateEkf {
    /// State vector [SoC, V_RC1, V_RC2]
    pub state: [f64; 3],
    /// 3×3 covariance matrix (row-major)
    p: [f64; 9],
    /// Process noise covariance (diagonal: σ_soc², σ_rc1², σ_rc2²)
    q: [f64; 3],
    /// Measurement noise variance σ_v²
    r_noise: f64,
    /// Capacity [Ah]
    capacity_ah: f64,
}

impl StateEkf {
    pub fn new(soc0: f64, _dt: f64, capacity_ah: f64) -> Self {
        let mut p = [0.0f64; 9];
        p[0] = 0.01;
        p[4] = 1e-4;
        p[8] = 1e-4; // initial uncertainty
        Self {
            state: [soc0, 0.0, 0.0],
            p,
            q: [1e-8, 1e-6, 1e-6],
            r_noise: 1e-4,
            capacity_ah,
        }
    }

    pub fn predict(&mut self, current_a: f64, params: &EcmParams, dt: f64) {
        let a1 = (-dt / (params.r1 * params.c1)).exp();
        let a2 = (-dt / (params.r2 * params.c2)).exp();
        let dsoc = -current_a * dt / (3600.0 * self.capacity_ah);

        self.state[0] += dsoc;
        self.state[1] = a1 * self.state[1] + params.r1 * (1.0 - a1) * current_a;
        self.state[2] = a2 * self.state[2] + params.r2 * (1.0 - a2) * current_a;
        self.state[0] = self.state[0].clamp(0.0, 1.0);

        // F = diag(1, a1, a2) — Jacobian of f
        // P = F·P·F' + Q (diagonal F)
        let f = [1.0, a1, a2];
        for i in 0..3 {
            for j in 0..3 {
                self.p[i * 3 + j] = f[i] * self.p[i * 3 + j] * f[j];
            }
            self.p[i * 3 + i] += self.q[i];
        }
    }

    pub fn update(
        &mut self,
        v_meas: f64,
        current_a: f64,
        ocv_fn: &dyn Fn(f64) -> f64,
        params: &EcmParams,
    ) -> f64 {
        let soc = self.state[0];
        let ocv = ocv_fn(soc);
        // Predicted terminal voltage
        let v_pred = ocv - params.r0 * current_a - self.state[1] - self.state[2];
        let innov = v_meas - v_pred;

        // Measurement Jacobian H = [dOCV/dSoC, -1, -1] ≈ [0, -1, -1] (simplified)
        let h = [0.0f64, -1.0, -1.0];

        // S = H·P·H' + R
        let hp: [f64; 3] = [
            h[0] * self.p[0] + h[1] * self.p[3] + h[2] * self.p[6],
            h[0] * self.p[1] + h[1] * self.p[4] + h[2] * self.p[7],
            h[0] * self.p[2] + h[1] * self.p[5] + h[2] * self.p[8],
        ];
        let s = hp[0] * h[0] + hp[1] * h[1] + hp[2] * h[2] + self.r_noise;

        // K = P·H' / S
        let ph: [f64; 3] = [
            self.p[0] * h[0] + self.p[1] * h[1] + self.p[2] * h[2],
            self.p[3] * h[0] + self.p[4] * h[1] + self.p[5] * h[2],
            self.p[6] * h[0] + self.p[7] * h[1] + self.p[8] * h[2],
        ];
        let k: [f64; 3] = [ph[0] / s, ph[1] / s, ph[2] / s];

        // State update
        for (s, &ki) in self.state.iter_mut().zip(k.iter()) {
            *s += ki * innov;
        }
        self.state[0] = self.state[0].clamp(0.0, 1.0);

        // Direct update:  P = P - K·H·P
        let khp: [f64; 9] = {
            let mut m = [0.0f64; 9];
            for i in 0..3 {
                for j in 0..3 {
                    m[i * 3 + j] = k[i] * hp[j];
                }
            }
            m
        };
        for (pv, &kh) in self.p.iter_mut().zip(khp.iter()) {
            *pv -= kh;
        }

        self.state[0]
    }
}

/// Parameter EKF: estimates [R0, R1, C1, R2, C2].
pub struct ParamEkf {
    /// Parameter vector [R0, R1, C1, R2, C2]
    pub params_vec: [f64; 5],
    /// 5×5 covariance matrix
    p: [f64; 25],
    /// Process noise (parameters change slowly)
    q: [f64; 5],
    /// Measurement noise variance
    r_noise: f64,
}

impl ParamEkf {
    pub fn new(initial: &EcmParams, _dt: f64) -> Self {
        let mut p = [0.0f64; 25];
        for i in 0..5 {
            p[i * 5 + i] = 1e-4;
        }
        Self {
            params_vec: [initial.r0, initial.r1, initial.c1, initial.r2, initial.c2],
            p,
            q: [1e-10, 1e-10, 1e-6, 1e-10, 1e-6],
            r_noise: 1e-4,
        }
    }

    pub fn params(&self) -> EcmParams {
        EcmParams {
            r0: self.params_vec[0].max(1e-6),
            r1: self.params_vec[1].max(1e-6),
            c1: self.params_vec[2].max(1.0),
            r2: self.params_vec[3].max(1e-6),
            c2: self.params_vec[4].max(1.0),
        }
    }

    pub fn predict(&mut self) {
        // Parameters random-walk: P += Q
        for i in 0..5 {
            self.p[i * 5 + i] += self.q[i];
        }
    }

    pub fn update(
        &mut self,
        v_meas: f64,
        current_a: f64,
        soc: f64,
        rc_state: &[f64; 3],
        ocv_fn: &dyn Fn(f64) -> f64,
    ) -> EcmParams {
        let p = self.params();
        let ocv = ocv_fn(soc);

        // Predicted terminal voltage
        let v_pred = ocv - p.r0 * current_a - rc_state[1] - rc_state[2];
        let innov = v_meas - v_pred;

        // Jacobian H = dV/dθ = [-I, 0, 0, 0, 0] (simplified, dominant term is R0·I)
        let h = [-current_a, 0.0f64, 0.0, 0.0, 0.0];

        // S = H·P·H' + R
        let mut hp = [0.0f64; 5];
        for (j, hp_j) in hp.iter_mut().enumerate() {
            for (k, &hk) in h.iter().enumerate() {
                *hp_j += hk * self.p[k * 5 + j];
            }
        }
        let s: f64 = hp
            .iter()
            .zip(h.iter())
            .map(|(hpj, hj)| hpj * hj)
            .sum::<f64>()
            + self.r_noise;

        // K = P·H' / S
        let mut ph = [0.0f64; 5];
        for (i, ph_i) in ph.iter_mut().enumerate() {
            for (j, &hj) in h.iter().enumerate() {
                *ph_i += self.p[i * 5 + j] * hj;
            }
        }
        let k: Vec<f64> = ph.iter().map(|v| v / s).collect();

        // Update params
        for (pv, &ki) in self.params_vec.iter_mut().zip(k.iter()) {
            *pv += ki * innov;
        }
        // Enforce physical constraints
        self.params_vec[0] = self.params_vec[0].max(1e-6); // R0
        self.params_vec[1] = self.params_vec[1].max(1e-6); // R1
        self.params_vec[2] = self.params_vec[2].max(1.0); // C1
        self.params_vec[3] = self.params_vec[3].max(1e-6); // R2
        self.params_vec[4] = self.params_vec[4].max(1.0); // C2

        // Update covariance P = P - K·H·P
        let mut khp = [0.0f64; 25];
        for i in 0..5 {
            for j in 0..5 {
                khp[i * 5 + j] = k[i] * hp[j];
            }
        }
        for (pv, &kh) in self.p.iter_mut().zip(khp.iter()) {
            *pv -= kh;
        }

        self.params()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn nmc_ocv(soc: f64) -> f64 {
        // Simple linear OCV model: 3.0 V at SoC=0, 4.2 V at SoC=1
        3.0 + 1.2 * soc
    }

    fn make_initial_params() -> EcmParams {
        EcmParams {
            r0: 0.02,
            r1: 0.015,
            c1: 3000.0,
            r2: 0.01,
            c2: 500.0,
        }
    }

    #[test]
    fn test_ecm_params_time_constants() {
        let p = make_initial_params();
        let (tau1, tau2) = p.time_constants();
        assert!((tau1 - 45.0).abs() < 1.0, "τ1={:.2}", tau1);
        assert!((tau2 - 5.0).abs() < 1.0, "τ2={:.2}", tau2);
    }

    #[test]
    fn test_ecm_params_r_total() {
        let p = make_initial_params();
        assert!((p.r_total() - 0.045).abs() < 1e-10);
    }

    #[test]
    fn test_rls_estimator_converges() {
        let initial = EcmParams::lfp_typical();
        let mut rls = RlsEstimator::new(&initial, 0.98, 1.0, 1000.0);
        let true_r0 = 0.002;

        // Feed synthetic data: step current 10A, voltage = OCV − R0·I
        for k in 0..200 {
            let soc = 0.5;
            let ocv = 3.2 + 0.8 * soc;
            let current = 10.0;
            let voltage = ocv - true_r0 * current - 0.001 * (k as f64 / 200.0);
            rls.update(current, voltage, ocv);
        }

        let params = rls.extract_params();
        assert!(
            params.r0 > 0.0 && params.r0 < 0.1,
            "R0 out of range: {:.4}",
            params.r0
        );
        assert_eq!(rls.update_count, 200);
    }

    #[test]
    fn test_rls_update_count() {
        let initial = EcmParams::lfp_typical();
        let mut rls = RlsEstimator::new(&initial, 0.99, 0.1, 100.0);
        for i in 0..10 {
            rls.update(5.0 + i as f64 * 0.1, 3.5, 3.6);
        }
        assert_eq!(rls.update_count, 10);
    }

    #[test]
    fn test_rls_extract_params_physical() {
        let initial = EcmParams {
            r0: 0.01,
            r1: 0.01,
            c1: 10000.0,
            r2: 0.005,
            c2: 1000.0,
        };
        let mut rls = RlsEstimator::new(&initial, 0.99, 0.5, 1000.0);
        rls.update(0.0, 3.6, 3.6);
        let p = rls.extract_params();
        assert!(p.r0 > 0.0, "R0 must be positive");
        assert!(p.r1 > 0.0, "R1 must be positive");
        assert!(p.c1 > 0.0, "C1 must be positive");
    }

    #[test]
    fn test_hysteresis_update_discharge() {
        let mut hys = HysteresisModel::uniform(5.0, 3.0, 0.02);
        assert_eq!(hys.h, 0.0);

        // Discharging: current > 0 → h should move toward +1
        for _ in 0..100 {
            hys.update(10.0, 1.0); // 10A discharge, 1s steps
        }
        assert!(
            hys.h > 0.0,
            "h should be positive during discharge: {:.4}",
            hys.h
        );
    }

    #[test]
    fn test_hysteresis_update_charge() {
        let mut hys = HysteresisModel::uniform(5.0, 3.0, 0.02);
        // Charging: current < 0 → h moves toward −1
        for _ in 0..100 {
            hys.update(-10.0, 1.0);
        }
        assert!(
            hys.h < 0.0,
            "h should be negative during charge: {:.4}",
            hys.h
        );
    }

    #[test]
    fn test_hysteresis_voltage_correction() {
        let hys = HysteresisModel::uniform(5.0, 3.0, 0.02);
        // h=0 → correction = 0
        assert_eq!(hys.voltage_correction(0.5), 0.0);
    }

    #[test]
    fn test_hysteresis_m_at_soc_interpolation() {
        let m_soc = vec![(0.0, 0.01), (0.5, 0.02), (1.0, 0.015)];
        let hys = HysteresisModel::with_profile(5.0, 3.0, m_soc);
        let m = hys.m_at_soc(0.25);
        assert!((m - 0.015).abs() < 1e-10, "m@0.25={:.4}", m);
    }

    #[test]
    fn test_hysteresis_clamped() {
        let mut hys = HysteresisModel::uniform(100.0, 1.0, 0.02);
        for _ in 0..1000 {
            hys.update(100.0, 1.0);
        }
        assert!(hys.h <= 1.0 + 1e-10, "h must be ≤ 1: {:.6}", hys.h);
    }

    #[test]
    fn test_state_ekf_soc_update() {
        let mut ekf = StateEkf::new(0.8, 1.0, 3.0);
        let params = EcmParams::lfp_typical();
        ekf.predict(2.0, &params, 1.0);
        let ocv_fn = |s: f64| 3.2 + 0.8 * s;
        let soc = ekf.update(3.7, 2.0, &ocv_fn, &params);
        assert!((0.0..=1.0).contains(&soc), "SoC out of range: {:.4}", soc);
    }

    #[test]
    fn test_param_ekf_update() {
        let initial = make_initial_params();
        let mut ekf = ParamEkf::new(&initial, 1.0);
        ekf.predict();
        let rc_state = [0.8, 0.001, 0.0005];
        let ocv_fn = |s: f64| 3.0 + 1.2 * s;
        let p = ekf.update(3.7, 2.0, 0.8, &rc_state, &ocv_fn);
        assert!(p.r0 > 0.0);
    }

    #[test]
    fn test_dual_ekf_soc_remains_bounded() {
        let initial = make_initial_params();
        let mut dekf = DualEkf::new(0.9, &initial, 1.0, 3.0);
        for k in 0..50 {
            let current = 2.0 + 0.1 * k as f64;
            let soc_true = 0.9 - current * k as f64 / (3600.0 * 3.0);
            let voltage = nmc_ocv(soc_true.max(0.0)) - 0.02 * current;
            let (soc, _) = dekf.update(current, voltage, &nmc_ocv);
            assert!((0.0..=1.0).contains(&soc), "SoC out of bounds: {:.4}", soc);
        }
    }
}
