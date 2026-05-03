//! Distribution System State Estimation (DSSE).
//!
//! Distribution networks differ from transmission in three key ways that affect
//! state estimation:
//!
//! 1. **Radial topology** — tree-structured feeders with a single substation slack bus.
//! 2. **High R/X ratio** — series resistance dominates; the P/Q decoupling assumption
//!    used in transmission WLS breaks down, so a full coupled formulation is required.
//! 3. **Heterogeneous measurement quality** — smart meters, SCADA, pseudo-measurements
//!    from load profiles and virtual zero-injection measurements span several orders of
//!    magnitude in accuracy.
//!
//! # Algorithm
//!
//! The module implements Weighted Least Squares (WLS) state estimation via the
//! Gauss–Newton method.  The state vector is `x = [V₀ … Vₙ₋₁, θ₀ … θₙ₋₁]` (all n buses).
//!
//! Each iteration solves the *normal equations*:
//!
//! ```text
//! (Hᵀ W H) Δx = Hᵀ W r
//! ```
//!
//! where `H` is the Jacobian of the measurement functions, `W = diag(1/σ²)` is the
//! weight matrix, and `r = z − h(x)` is the residual vector.
//!
//! The Jacobian is computed numerically via forward finite differences (ε = 1 × 10⁻⁶).
//!
//! # Bad Data Detection
//!
//! After convergence the Largest Normalised Residual (LNR) test flags measurements
//! whose `|r_i| / σ_i` exceeds `config.bad_data_threshold` (default 3.0).
//!
//! A χ²-statistic `Σ rᵢ² / σᵢ²` is also returned for a global bad-data check.

use serde::{Deserialize, Serialize};

use crate::error::OxiGridError;

// ─────────────────────────────────────────────────────────────────────────────
// Measurement types
// ─────────────────────────────────────────────────────────────────────────────

/// Measurement type understood by the DSSE solver.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DsseMeasurementType {
    /// Active power injection P at a bus (MW).
    ActivePowerInjection,
    /// Reactive power injection Q at a bus (MVAr).
    ReactivePowerInjection,
    /// Active power flow P on a branch (MW).
    ActivePowerFlow,
    /// Reactive power flow Q on a branch (MVAr).
    ReactivePowerFlow,
    /// Voltage magnitude |V| at a bus (pu).
    VoltageMagnitude,
    /// Current magnitude |I| on a branch (pu).
    CurrentMagnitude,
    /// Active power reading from a smart meter (high accuracy).
    SmartMeterPower,
    /// Load-profile estimate — pseudo-measurement (low accuracy).
    PseudoMeasurement,
    /// Zero-injection constraint at a T-junction (very high accuracy).
    VirtualMeasurement,
}

// ─────────────────────────────────────────────────────────────────────────────
// Measurement
// ─────────────────────────────────────────────────────────────────────────────

/// A single DSSE measurement with its noise model.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DsseMeasurement {
    /// Unique measurement identifier.
    pub id: usize,
    /// Kind of measured quantity.
    pub meas_type: DsseMeasurementType,
    /// Bus or branch index the measurement refers to.
    pub element: usize,
    /// Measured value (engineering units matching the measurement type).
    pub value: f64,
    /// Measurement variance σ² (same units as value²).
    pub variance: f64,
}

impl DsseMeasurement {
    /// Standard deviation σ = √variance.
    #[inline]
    pub fn std_dev(&self) -> f64 {
        self.variance.sqrt()
    }

    /// WLS weight w = 1 / σ² (clamped away from zero).
    #[inline]
    pub fn weight(&self) -> f64 {
        1.0 / self.variance.max(1e-12)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Network elements
// ─────────────────────────────────────────────────────────────────────────────

/// A distribution feeder branch modelled as a series RL element.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DsseBranch {
    /// From-bus index.
    pub from: usize,
    /// To-bus index.
    pub to: usize,
    /// Series resistance (Ω).
    pub r_ohm: f64,
    /// Series reactance (Ω).
    pub x_ohm: f64,
    /// Thermal current rating (A).
    pub rating_a: f64,
    /// Physical length (km).
    pub length_km: f64,
}

/// A distribution bus.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DsseBus {
    /// Bus index (0-based).
    pub id: usize,
    /// `true` for the HV/MV substation slack bus (voltage reference).
    pub is_substation: bool,
    /// Nominal voltage (kV).
    pub v_base_kv: f64,
    /// Nominal active load used to initialise pseudo-measurements (MW).
    pub load_p_mw: f64,
    /// Nominal reactive load (MVAr).
    pub load_q_mvar: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Configuration
// ─────────────────────────────────────────────────────────────────────────────

/// Solver configuration for `DistributionStateEstimator`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DsseConfig {
    /// Number of buses in the network.
    pub n_buses: usize,
    /// System base apparent power (MVA).
    pub base_mva: f64,
    /// System base voltage (kV).
    pub base_kv: f64,
    /// Maximum Gauss–Newton iterations.
    pub max_iterations: usize,
    /// Convergence tolerance: ‖Δx‖_∞ < tol.
    pub convergence_tol: f64,
    /// Normalised-residual threshold for the LNR bad-data test.
    pub bad_data_threshold: f64,
    /// If `true`, use branch-current form (BIM); otherwise use nodal (NIM).
    /// Both paths use the same nodal WLS formulation currently.
    pub use_branch_current_form: bool,
}

impl Default for DsseConfig {
    fn default() -> Self {
        Self {
            n_buses: 2,
            base_mva: 1.0,
            base_kv: 10.0,
            max_iterations: 30,
            convergence_tol: 1e-4,
            bad_data_threshold: 3.0,
            use_branch_current_form: false,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Result
// ─────────────────────────────────────────────────────────────────────────────

/// Solution returned by `DistributionStateEstimator::estimate`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DsseResult {
    /// Whether the solver converged within `max_iterations`.
    pub converged: bool,
    /// Number of Gauss–Newton iterations performed.
    pub iterations: usize,
    /// Estimated bus voltage magnitudes (pu).
    pub voltage_magnitudes: Vec<f64>,
    /// Estimated bus voltage angles (rad).
    pub voltage_angles: Vec<f64>,
    /// Estimated branch current magnitudes (pu).
    pub branch_currents: Vec<f64>,
    /// Estimated branch active power flows (MW).
    pub branch_p_flow: Vec<f64>,
    /// Estimated branch reactive power flows (MVAr).
    pub branch_q_flow: Vec<f64>,
    /// Raw measurement residuals r = z − h(x̂).
    pub measurement_residuals: Vec<f64>,
    /// Normalised residuals |r_i| / σ_i.
    pub normalized_residuals: Vec<f64>,
    /// IDs of measurements flagged by the LNR test.
    pub bad_data_suspected: Vec<usize>,
    /// WLS objective J = Σ rᵢ² / σᵢ².
    pub objective: f64,
    /// Maximum normalised residual across all measurements.
    pub max_normalized_residual: f64,
    /// χ²-statistic = Σ rᵢ² wᵢ (same as objective).
    pub chi_squared_statistic: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Solver
// ─────────────────────────────────────────────────────────────────────────────

/// Distribution System State Estimator using WLS Gauss–Newton iterations.
#[derive(Debug, Clone)]
pub struct DistributionStateEstimator {
    /// All buses in the network.
    pub buses: Vec<DsseBus>,
    /// All branches in the network.
    pub branches: Vec<DsseBranch>,
    /// All available measurements.
    pub measurements: Vec<DsseMeasurement>,
    /// Solver configuration.
    pub config: DsseConfig,
}

impl DistributionStateEstimator {
    /// Construct a new estimator.
    pub fn new(
        buses: Vec<DsseBus>,
        branches: Vec<DsseBranch>,
        measurements: Vec<DsseMeasurement>,
        config: DsseConfig,
    ) -> Self {
        Self {
            buses,
            branches,
            measurements,
            config,
        }
    }

    // ── Per-unit conversion helpers ──────────────────────────────────────────

    /// Base impedance Z_base = V_base² / S_base  `Ω`.
    ///
    /// V_base is converted from kV to V; S_base from MVA to VA.
    pub fn z_base(&self) -> f64 {
        let v_base_v = self.config.base_kv * 1_000.0;
        let s_base_va = self.config.base_mva * 1_000_000.0;
        (v_base_v * v_base_v) / s_base_va
    }

    /// Convert a series impedance from Ω to per-unit.
    fn to_pu_impedance(&self, r_ohm: f64, x_ohm: f64) -> (f64, f64) {
        let zb = self.z_base().max(1e-12);
        (r_ohm / zb, x_ohm / zb)
    }

    // ── Y-bus construction ───────────────────────────────────────────────────

    /// Build the nodal admittance matrix as separate conductance G and
    /// susceptance B matrices (both n_buses × n_buses, row-major `Vec<Vec<f64>>`).
    fn build_ybus(&self) -> (Vec<Vec<f64>>, Vec<Vec<f64>>) {
        let n = self.config.n_buses;
        let mut g = vec![vec![0.0_f64; n]; n];
        let mut b = vec![vec![0.0_f64; n]; n];

        for br in &self.branches {
            let i = br.from;
            let j = br.to;
            if i >= n || j >= n {
                continue;
            }
            let (r_pu, x_pu) = self.to_pu_impedance(br.r_ohm, br.x_ohm);
            let denom = (r_pu * r_pu + x_pu * x_pu).max(1e-18);
            let g_ij = r_pu / denom;
            let b_ij = -x_pu / denom;

            g[i][i] += g_ij;
            g[j][j] += g_ij;
            g[i][j] -= g_ij;
            g[j][i] -= g_ij;

            b[i][i] += b_ij;
            b[j][j] += b_ij;
            b[i][j] -= b_ij;
            b[j][i] -= b_ij;
        }

        (g, b)
    }

    // ── State initialisation ─────────────────────────────────────────────────

    /// Flat start: V = 1 pu, θ = 0 rad for every bus.
    fn flat_start(&self) -> (Vec<f64>, Vec<f64>) {
        let n = self.config.n_buses;
        (vec![1.0_f64; n], vec![0.0_f64; n])
    }

    // ── Measurement functions h(x) ───────────────────────────────────────────

    /// Evaluate the measurement function vector h(x) for the current state.
    fn compute_measurement_functions(
        &self,
        v: &[f64],
        theta: &[f64],
        g_mat: &[Vec<f64>],
        b_mat: &[Vec<f64>],
    ) -> Vec<f64> {
        let n = self.config.n_buses;
        let mut h = Vec::with_capacity(self.measurements.len());

        for meas in &self.measurements {
            let elem = meas.element;
            let val = match meas.meas_type {
                // ── Active power injection (and meter / pseudo variants) ──
                DsseMeasurementType::ActivePowerInjection
                | DsseMeasurementType::SmartMeterPower
                | DsseMeasurementType::PseudoMeasurement => {
                    if elem >= n {
                        0.0
                    } else {
                        let mut p = 0.0_f64;
                        for j in 0..n {
                            let dth = theta[elem] - theta[j];
                            p += v[elem]
                                * v[j]
                                * (g_mat[elem][j] * dth.cos() + b_mat[elem][j] * dth.sin());
                        }
                        p
                    }
                }

                // ── Reactive power injection ──
                DsseMeasurementType::ReactivePowerInjection => {
                    if elem >= n {
                        0.0
                    } else {
                        let mut q = 0.0_f64;
                        for j in 0..n {
                            let dth = theta[elem] - theta[j];
                            q += v[elem]
                                * v[j]
                                * (g_mat[elem][j] * dth.sin() - b_mat[elem][j] * dth.cos());
                        }
                        q
                    }
                }

                // ── Voltage magnitude ──
                DsseMeasurementType::VoltageMagnitude | DsseMeasurementType::VirtualMeasurement => {
                    if elem < n {
                        v[elem]
                    } else {
                        0.0
                    }
                }

                // ── Active branch power flow ──
                DsseMeasurementType::ActivePowerFlow => {
                    self.branch_p_flow_for_index(elem, v, theta, g_mat, b_mat)
                }

                // ── Reactive branch power flow ──
                DsseMeasurementType::ReactivePowerFlow => {
                    self.branch_q_flow_for_index(elem, v, theta, g_mat, b_mat)
                }

                // ── Current magnitude (approximated from S = V·I*) ──
                DsseMeasurementType::CurrentMagnitude => {
                    let p = self.branch_p_flow_for_index(elem, v, theta, g_mat, b_mat);
                    let q = self.branch_q_flow_for_index(elem, v, theta, g_mat, b_mat);
                    if elem < self.branches.len() {
                        let fr = self.branches[elem].from;
                        let vi = if fr < n { v[fr] } else { 1.0 };
                        let s = (p * p + q * q).sqrt();
                        s / vi.max(0.01)
                    } else {
                        0.0
                    }
                }
            };
            h.push(val);
        }
        h
    }

    /// Active power flow on branch `br_idx` (from-bus end).
    fn branch_p_flow_for_index(
        &self,
        br_idx: usize,
        v: &[f64],
        theta: &[f64],
        g_mat: &[Vec<f64>],
        b_mat: &[Vec<f64>],
    ) -> f64 {
        let n = self.config.n_buses;
        if br_idx >= self.branches.len() {
            return 0.0;
        }
        let br = &self.branches[br_idx];
        let i = br.from;
        let j = br.to;
        if i >= n || j >= n {
            return 0.0;
        }
        let dth = theta[i] - theta[j];
        // Use off-diagonal Y-bus entries for the branch coupling term.
        let g_self = g_mat[i][i];
        let g_off = -g_mat[i][j]; // branch conductance magnitude
        let b_off = -b_mat[i][j]; // branch susceptance magnitude
        v[i] * v[i] * g_self - v[i] * v[j] * (g_off * dth.cos() + b_off * dth.sin())
    }

    /// Reactive power flow on branch `br_idx` (from-bus end).
    fn branch_q_flow_for_index(
        &self,
        br_idx: usize,
        v: &[f64],
        theta: &[f64],
        g_mat: &[Vec<f64>],
        b_mat: &[Vec<f64>],
    ) -> f64 {
        let n = self.config.n_buses;
        if br_idx >= self.branches.len() {
            return 0.0;
        }
        let br = &self.branches[br_idx];
        let i = br.from;
        let j = br.to;
        if i >= n || j >= n {
            return 0.0;
        }
        let dth = theta[i] - theta[j];
        let b_self = b_mat[i][i];
        let g_off = -g_mat[i][j];
        let b_off = -b_mat[i][j];
        -v[i] * v[i] * b_self - v[i] * v[j] * (g_off * dth.sin() - b_off * dth.cos())
    }

    // ── Jacobian ─────────────────────────────────────────────────────────────

    /// Return the index of the slack (substation) bus, defaulting to bus 0.
    fn slack_bus_index(&self) -> usize {
        self.buses
            .iter()
            .find(|b| b.is_substation)
            .map(|b| b.id)
            .unwrap_or(0)
    }

    /// Compute the Jacobian H (m × state_len) numerically via forward finite differences.
    ///
    /// State order: `[V₀ … Vₙ₋₁,  θ_i for i ≠ slack]`  (length 2n−1).
    ///
    /// The slack bus angle is held fixed at 0 and excluded from the state,
    /// so the gain matrix (Hᵀ W H) remains non-singular even when only voltage
    /// magnitude measurements are supplied.
    fn compute_jacobian(
        &self,
        v: &[f64],
        theta: &[f64],
        g_mat: &[Vec<f64>],
        b_mat: &[Vec<f64>],
    ) -> Vec<Vec<f64>> {
        let n = self.config.n_buses;
        let slack = self.slack_bus_index();
        let m = self.measurements.len();
        // state = n voltage magnitudes + (n-1) non-slack angles → 2n-1 columns
        let state_len = 2 * n - 1;
        let eps = 1e-6_f64;

        let h0 = self.compute_measurement_functions(v, theta, g_mat, b_mat);
        let mut jac = vec![vec![0.0_f64; state_len]; m];

        // Perturb V components (columns 0..n)
        let mut v_pert = v.to_vec();
        for col in 0..n {
            v_pert[col] += eps;
            let h_pert = self.compute_measurement_functions(&v_pert, theta, g_mat, b_mat);
            for row in 0..m {
                jac[row][col] = (h_pert[row] - h0[row]) / eps;
            }
            v_pert[col] -= eps;
        }

        // Perturb non-slack θ components (columns n..2n-1)
        // Mapping: state column n + k  →  bus angle index  bus_k  (skipping slack)
        let mut th_pert = theta.to_vec();
        let mut state_col = n; // first θ state column
        for bus_idx in 0..n {
            if bus_idx == slack {
                continue; // skip slack — angle is fixed
            }
            th_pert[bus_idx] += eps;
            let h_pert = self.compute_measurement_functions(v, &th_pert, g_mat, b_mat);
            for row in 0..m {
                jac[row][state_col] = (h_pert[row] - h0[row]) / eps;
            }
            th_pert[bus_idx] -= eps;
            state_col += 1;
        }

        jac
    }

    // ── Weight matrix ────────────────────────────────────────────────────────

    /// Build the diagonal weight vector W (length m), where W_i = 1 / σᵢ².
    fn build_weight_matrix(&self) -> Vec<f64> {
        self.measurements.iter().map(|m| m.weight()).collect()
    }

    // ── Normal equations solver ──────────────────────────────────────────────

    /// Solve (Hᵀ W H) Δx = Hᵀ W r via Gaussian elimination with partial pivoting.
    ///
    /// Returns `Err(LinearAlgebra(...))` if the gain matrix is (near-)singular.
    #[allow(clippy::needless_range_loop)]
    fn solve_normal_equations(
        h: &[Vec<f64>],
        w: &[f64],
        r: &[f64],
    ) -> Result<Vec<f64>, OxiGridError> {
        let m = h.len();
        if m == 0 {
            return Err(OxiGridError::LinearAlgebra(
                "empty measurement set".to_string(),
            ));
        }
        let state_len = h[0].len();
        if state_len == 0 {
            return Err(OxiGridError::LinearAlgebra(
                "empty state vector".to_string(),
            ));
        }

        // Build gain matrix A = Hᵀ W H  (state_len × state_len)
        let mut a = vec![vec![0.0_f64; state_len]; state_len];
        // Build right-hand side b = Hᵀ W r  (state_len)
        let mut rhs = vec![0.0_f64; state_len];

        for k in 0..m {
            let wk = w[k];
            let rk = r[k];
            for i in 0..state_len {
                rhs[i] += h[k][i] * wk * rk;
                for j in 0..state_len {
                    a[i][j] += h[k][i] * wk * h[k][j];
                }
            }
        }

        // Gaussian elimination with partial pivoting on augmented matrix [A | rhs]
        let mut aug: Vec<Vec<f64>> = (0..state_len)
            .map(|i| {
                let mut row = a[i].clone();
                row.push(rhs[i]);
                row
            })
            .collect();

        for col in 0..state_len {
            // Find pivot row
            let mut pivot_row = col;
            let mut pivot_val = aug[col][col].abs();
            for row in (col + 1)..state_len {
                let v = aug[row][col].abs();
                if v > pivot_val {
                    pivot_val = v;
                    pivot_row = row;
                }
            }

            aug.swap(col, pivot_row);

            let pivot = aug[col][col];
            if pivot.abs() < 1e-14 {
                return Err(OxiGridError::LinearAlgebra(format!(
                    "gain matrix is singular at column {col} (pivot = {pivot:.3e})"
                )));
            }

            // Normalise pivot row
            let inv_pivot = 1.0 / pivot;
            for elem in aug[col].iter_mut() {
                *elem *= inv_pivot;
            }

            // Eliminate column in all other rows
            for row in 0..state_len {
                if row == col {
                    continue;
                }
                let factor = aug[row][col];
                if factor.abs() < 1e-16 {
                    continue;
                }
                for k in 0..=state_len {
                    let sub = factor * aug[col][k];
                    aug[row][k] -= sub;
                }
            }
        }

        // Extract solution from last column
        let dx: Vec<f64> = (0..state_len).map(|i| aug[i][state_len]).collect();
        Ok(dx)
    }

    // ── Residual analysis ────────────────────────────────────────────────────

    /// Compute normalised residuals: |r_i| / σ_i (simplified LNR form).
    fn compute_normalized_residuals(&self, residuals: &[f64]) -> Vec<f64> {
        self.measurements
            .iter()
            .zip(residuals.iter())
            .map(|(meas, &res)| res.abs() / meas.std_dev().max(1e-12))
            .collect()
    }

    /// Identify measurements whose normalised residual exceeds the LNR threshold.
    ///
    /// Returns a vector of measurement *IDs* (not indices).
    fn identify_bad_data(&self, normalized_residuals: &[f64]) -> Vec<usize> {
        self.measurements
            .iter()
            .zip(normalized_residuals.iter())
            .filter_map(|(meas, &nr)| {
                if nr > self.config.bad_data_threshold {
                    Some(meas.id)
                } else {
                    None
                }
            })
            .collect()
    }

    /// χ²-statistic = Σ rᵢ² × wᵢ  (equals the WLS objective J).
    fn chi_squared_test(&self, residuals: &[f64]) -> f64 {
        let w = self.build_weight_matrix();
        residuals
            .iter()
            .zip(w.iter())
            .map(|(&res, &wi)| res * res * wi)
            .sum()
    }

    // ── Branch current computation ───────────────────────────────────────────

    /// Compute branch current magnitudes (pu) from the estimated state.
    fn compute_branch_currents(&self, v: &[f64], theta: &[f64]) -> Vec<f64> {
        let n = self.config.n_buses;
        self.branches
            .iter()
            .map(|br| {
                let i = br.from;
                let j = br.to;
                if i >= n || j >= n {
                    return 0.0;
                }
                let (r_pu, x_pu) = self.to_pu_impedance(br.r_ohm, br.x_ohm);
                let z_sq = (r_pu * r_pu + x_pu * x_pu).max(1e-18);

                // Phasor difference ΔV = V_i∠θ_i − V_j∠θ_j
                let dv_real = v[i] * theta[i].cos() - v[j] * theta[j].cos();
                let dv_imag = v[i] * theta[i].sin() - v[j] * theta[j].sin();

                // I = ΔV / (r + jx) — multiply numerator & denominator by (r − jx)
                let i_real = (r_pu * dv_real + x_pu * dv_imag) / z_sq;
                let i_imag = (r_pu * dv_imag - x_pu * dv_real) / z_sq;

                (i_real * i_real + i_imag * i_imag).sqrt()
            })
            .collect()
    }

    // ── Main solver ──────────────────────────────────────────────────────────

    /// Run the WLS Gauss–Newton state estimation.
    ///
    /// Returns a [`DsseResult`] on success, or an [`OxiGridError`] if the network
    /// configuration is invalid or the linear solve fails irrecoverably.
    pub fn estimate(&mut self) -> Result<DsseResult, OxiGridError> {
        // ── Validate inputs ──
        if self.config.n_buses < 2 {
            return Err(OxiGridError::InvalidNetwork(
                "DSSE requires at least 2 buses".to_string(),
            ));
        }
        if self.measurements.is_empty() {
            return Err(OxiGridError::InvalidNetwork(
                "no measurements provided".to_string(),
            ));
        }
        if !self.buses.iter().any(|b| b.is_substation) {
            return Err(OxiGridError::InvalidNetwork(
                "at least one bus must be marked as substation (slack)".to_string(),
            ));
        }

        let n = self.config.n_buses;
        let m = self.measurements.len();
        let slack = self.slack_bus_index();

        // ── Build Y-bus ──
        let (g_mat, b_mat) = self.build_ybus();

        // ── Flat start ──
        let (mut v, mut theta) = self.flat_start();

        let w = self.build_weight_matrix();

        let mut converged = false;
        let mut iters = 0_usize;

        // ── Gauss–Newton iterations ──
        for iter in 0..self.config.max_iterations {
            iters = iter + 1;

            // h(x)
            let h_vec = self.compute_measurement_functions(&v, &theta, &g_mat, &b_mat);

            // r = z − h(x)
            let residuals: Vec<f64> = self
                .measurements
                .iter()
                .zip(h_vec.iter())
                .map(|(meas, &hk)| meas.value - hk)
                .collect();

            // Jacobian H (m × (2n-1))
            let jac = self.compute_jacobian(&v, &theta, &g_mat, &b_mat);

            // Normal equations → Δx  (length 2n-1)
            let dx = Self::solve_normal_equations(&jac, &w, &residuals)?;

            // ── State update ──
            // dx[0..n]         → ΔV for each bus
            // dx[n..2n-1]      → Δθ for non-slack buses (in bus-index order, skipping slack)
            for i in 0..n {
                v[i] += dx[i];
                // Clamp voltage magnitude to [0.5, 1.5] pu for numerical stability
                v[i] = v[i].clamp(0.5, 1.5);
            }
            // Apply angle updates to non-slack buses
            let mut state_col = n;
            for (bus_idx, theta_val) in theta.iter_mut().enumerate() {
                if bus_idx == slack {
                    continue;
                }
                *theta_val += dx[state_col];
                state_col += 1;
            }
            // Keep slack angle fixed
            theta[slack] = 0.0;

            // ── Convergence check ──
            let inf_norm = dx.iter().map(|d: &f64| d.abs()).fold(0.0_f64, f64::max);
            if inf_norm < self.config.convergence_tol {
                converged = true;
                break;
            }
        }

        // ── Post-processing ──
        let h_final = self.compute_measurement_functions(&v, &theta, &g_mat, &b_mat);
        let final_residuals: Vec<f64> = self
            .measurements
            .iter()
            .zip(h_final.iter())
            .map(|(meas, &hk)| meas.value - hk)
            .collect();

        let normalized = self.compute_normalized_residuals(&final_residuals);
        let bad_data = self.identify_bad_data(&normalized);
        let chi2 = self.chi_squared_test(&final_residuals);

        let max_nr = normalized.iter().cloned().fold(0.0_f64, f64::max);

        let objective: f64 = final_residuals
            .iter()
            .zip(w.iter())
            .map(|(&res, &wi)| res * res * wi)
            .sum();

        // Branch currents
        let branch_currents = self.compute_branch_currents(&v, &theta);

        // Branch flows (scaled to MW / MVAr via base_mva)
        let mut branch_p_flow = Vec::with_capacity(self.branches.len());
        let mut branch_q_flow = Vec::with_capacity(self.branches.len());
        for idx in 0..self.branches.len() {
            let p = self.branch_p_flow_for_index(idx, &v, &theta, &g_mat, &b_mat);
            let q = self.branch_q_flow_for_index(idx, &v, &theta, &g_mat, &b_mat);
            branch_p_flow.push(p * self.config.base_mva);
            branch_q_flow.push(q * self.config.base_mva);
        }

        debug_assert_eq!(final_residuals.len(), m);
        debug_assert_eq!(normalized.len(), m);

        Ok(DsseResult {
            converged,
            iterations: iters,
            voltage_magnitudes: v,
            voltage_angles: theta,
            branch_currents,
            branch_p_flow,
            branch_q_flow,
            measurement_residuals: final_residuals,
            normalized_residuals: normalized,
            bad_data_suspected: bad_data,
            objective,
            max_normalized_residual: max_nr,
            chi_squared_statistic: chi2,
        })
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Helpers ──────────────────────────────────────────────────────────────

    fn make_bus(id: usize, is_sub: bool) -> DsseBus {
        DsseBus {
            id,
            is_substation: is_sub,
            v_base_kv: 10.0,
            load_p_mw: 0.1,
            load_q_mvar: 0.05,
        }
    }

    fn make_branch(from: usize, to: usize, r: f64, x: f64) -> DsseBranch {
        DsseBranch {
            from,
            to,
            r_ohm: r,
            x_ohm: x,
            rating_a: 200.0,
            length_km: 1.0,
        }
    }

    fn make_meas(
        id: usize,
        meas_type: DsseMeasurementType,
        element: usize,
        value: f64,
        variance: f64,
    ) -> DsseMeasurement {
        DsseMeasurement {
            id,
            meas_type,
            element,
            value,
            variance,
        }
    }

    fn default_config(n: usize) -> DsseConfig {
        DsseConfig {
            n_buses: n,
            base_mva: 1.0,
            base_kv: 10.0,
            max_iterations: 30,
            convergence_tol: 1e-4,
            bad_data_threshold: 3.0,
            use_branch_current_form: false,
        }
    }

    // ── Tests ─────────────────────────────────────────────────────────────────

    #[test]
    fn test_dsse_bus_creation() {
        let bus = make_bus(2, true);
        assert_eq!(bus.id, 2);
        assert!(bus.is_substation);
        assert!((bus.v_base_kv - 10.0).abs() < 1e-10);
        assert!((bus.load_p_mw - 0.1).abs() < 1e-10);
        assert!((bus.load_q_mvar - 0.05).abs() < 1e-10);
    }

    #[test]
    fn test_dsse_branch_creation() {
        let br = make_branch(0, 1, 5.0, 3.0);
        assert_eq!(br.from, 0);
        assert_eq!(br.to, 1);
        assert!((br.r_ohm - 5.0).abs() < 1e-10);
        assert!((br.x_ohm - 3.0).abs() < 1e-10);
        assert!((br.rating_a - 200.0).abs() < 1e-10);
        assert!((br.length_km - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_measurement_weight_voltage() {
        // variance = 1e-4  →  weight = 10_000
        let m = make_meas(0, DsseMeasurementType::VoltageMagnitude, 0, 1.0, 1e-4);
        let expected = 1.0 / 1e-4;
        assert!(
            (m.weight() - expected).abs() < 1.0,
            "weight = {}",
            m.weight()
        );
    }

    #[test]
    fn test_measurement_weight_pseudo() {
        // variance = 0.01  →  weight = 100
        let m = make_meas(1, DsseMeasurementType::PseudoMeasurement, 1, 0.05, 0.01);
        assert!((m.weight() - 100.0).abs() < 1e-8);
    }

    #[test]
    fn test_z_base_computation() {
        // V_base = 10 kV, S_base = 1 MVA  →  Z_base = (10_000)² / 1_000_000 = 100 Ω
        let cfg = default_config(2);
        let se = DistributionStateEstimator::new(vec![], vec![], vec![], cfg);
        let zb = se.z_base();
        assert!((zb - 100.0).abs() < 1e-6, "expected 100 Ω, got {zb}");
    }

    #[test]
    fn test_pu_impedance_conversion() {
        // z_base = 100 Ω  →  5 Ω → 0.05 pu,  3 Ω → 0.03 pu
        let cfg = default_config(2);
        let se = DistributionStateEstimator::new(vec![], vec![], vec![], cfg);
        let (r_pu, x_pu) = se.to_pu_impedance(5.0, 3.0);
        assert!((r_pu - 0.05).abs() < 1e-10, "r_pu = {r_pu}");
        assert!((x_pu - 0.03).abs() < 1e-10, "x_pu = {x_pu}");
    }

    #[test]
    fn test_build_ybus_2bus() {
        // 2 buses, 1 branch with r=10Ω, x=0 → pure resistive
        // z_base = 100 Ω → r_pu = 0.1 → g = 10, b = 0
        let buses = vec![make_bus(0, true), make_bus(1, false)];
        let branches = vec![make_branch(0, 1, 10.0, 0.0)];
        let cfg = default_config(2);
        let se = DistributionStateEstimator::new(buses, branches, vec![], cfg);
        let (g, b) = se.build_ybus();

        let g_diag = g[0][0];
        let g_off = g[0][1];
        assert!(g_diag > 0.0, "diagonal G must be positive, got {g_diag}");
        assert!(g_off < 0.0, "off-diagonal G must be negative, got {g_off}");
        // Row sum = 0 for lossless network
        assert!((g_diag + g_off).abs() < 1e-10);
        assert!((b[0][0] + b[0][1]).abs() < 1e-10);
    }

    #[test]
    fn test_build_ybus_3bus() {
        let buses = vec![make_bus(0, true), make_bus(1, false), make_bus(2, false)];
        let branches = vec![make_branch(0, 1, 5.0, 2.0), make_branch(1, 2, 5.0, 2.0)];
        let cfg = default_config(3);
        let se = DistributionStateEstimator::new(buses, branches, vec![], cfg);
        let (g, _b) = se.build_ybus();

        // Bus 1 (middle) is connected to both 0 and 2  →  higher self-conductance
        assert!(g[1][1] > g[0][0], "middle bus should have higher self-G");
        assert_eq!(g.len(), 3);
        assert_eq!(g[0].len(), 3);
        // Symmetry
        assert!((g[0][1] - g[1][0]).abs() < 1e-12);
    }

    #[test]
    fn test_flat_start_initialization() {
        let cfg = default_config(4);
        let se = DistributionStateEstimator::new(vec![], vec![], vec![], cfg);
        let (v, theta) = se.flat_start();
        assert_eq!(v.len(), 4);
        assert_eq!(theta.len(), 4);
        assert!(v.iter().all(|&vi| (vi - 1.0).abs() < 1e-15));
        assert!(theta.iter().all(|&t| t.abs() < 1e-15));
    }

    #[test]
    fn test_measurement_function_voltage_magnitude() {
        let buses = vec![make_bus(0, true), make_bus(1, false)];
        let branches = vec![make_branch(0, 1, 5.0, 2.0)];
        let meas = vec![make_meas(
            0,
            DsseMeasurementType::VoltageMagnitude,
            1,
            1.05,
            1e-4,
        )];
        let cfg = default_config(2);
        let se = DistributionStateEstimator::new(buses, branches, meas, cfg);
        let (g, b) = se.build_ybus();
        let v = vec![1.0, 1.05];
        let theta = vec![0.0, 0.0];
        let h = se.compute_measurement_functions(&v, &theta, &g, &b);
        assert_eq!(h.len(), 1);
        assert!((h[0] - 1.05).abs() < 1e-10, "h[0] = {}", h[0]);
    }

    #[test]
    fn test_measurement_function_power_injection() {
        // At flat start (all V=1, theta=0): P_i = Σ_j G_ij  which equals 0 (Y-bus row sum = 0)
        let buses = vec![make_bus(0, true), make_bus(1, false)];
        let branches = vec![make_branch(0, 1, 5.0, 2.0)];
        let meas = vec![make_meas(
            0,
            DsseMeasurementType::ActivePowerInjection,
            0,
            0.0,
            0.01,
        )];
        let cfg = default_config(2);
        let se = DistributionStateEstimator::new(buses, branches, meas, cfg);
        let (g, b) = se.build_ybus();
        let (v, theta) = se.flat_start();
        let h = se.compute_measurement_functions(&v, &theta, &g, &b);
        assert!(h[0].abs() < 1e-10, "expected ~0, got {}", h[0]);
    }

    #[test]
    fn test_jacobian_numerical_finite_diff() {
        let buses = vec![make_bus(0, true), make_bus(1, false)];
        let branches = vec![make_branch(0, 1, 5.0, 2.0)];
        let meas = vec![
            make_meas(0, DsseMeasurementType::VoltageMagnitude, 0, 1.0, 1e-4),
            make_meas(1, DsseMeasurementType::VoltageMagnitude, 1, 1.0, 1e-4),
            make_meas(2, DsseMeasurementType::ActivePowerInjection, 0, 0.0, 0.01),
        ];
        let cfg = default_config(2);
        let se = DistributionStateEstimator::new(buses, branches, meas, cfg);
        let (g, b) = se.build_ybus();
        let (v, theta) = se.flat_start();
        let jac = se.compute_jacobian(&v, &theta, &g, &b);

        let m = 3;
        let state_len = 2 * 2 - 1; // 2n-1 = 3 (slack angle excluded)
        assert_eq!(jac.len(), m);
        assert!(jac.iter().all(|row| row.len() == state_len));
    }

    #[test]
    fn test_weight_matrix_construction() {
        let meas = vec![
            make_meas(0, DsseMeasurementType::VoltageMagnitude, 0, 1.0, 0.0001),
            make_meas(1, DsseMeasurementType::PseudoMeasurement, 1, 0.0, 0.04),
        ];
        let cfg = default_config(2);
        let se = DistributionStateEstimator::new(vec![], vec![], meas, cfg);
        let w = se.build_weight_matrix();
        assert_eq!(w.len(), 2);
        assert!((w[0] - 10_000.0).abs() < 1.0, "w[0] = {}", w[0]);
        assert!((w[1] - 25.0).abs() < 0.01, "w[1] = {}", w[1]);
    }

    #[test]
    fn test_normal_equations_solve_simple() {
        // H = I_2, w = [2, 3], r = [4, 6]
        // A = Hᵀ W H = diag(2, 3)
        // b = Hᵀ W r = [8, 18]
        // solution: dx = [4, 6]
        let h = vec![vec![1.0, 0.0], vec![0.0, 1.0]];
        let w = vec![2.0, 3.0];
        let r = vec![4.0, 6.0];
        let dx = DistributionStateEstimator::solve_normal_equations(&h, &w, &r)
            .expect("solve should succeed");
        assert_eq!(dx.len(), 2);
        assert!((dx[0] - 4.0).abs() < 1e-8, "dx[0] = {}", dx[0]);
        assert!((dx[1] - 6.0).abs() < 1e-8, "dx[1] = {}", dx[1]);
    }

    #[test]
    fn test_estimate_2bus_no_noise() {
        // 2-bus: bus 0 (substation, V=1.0), bus 1 (load, V=0.98)
        // Branch: 10 Ω + 5 Ω  (r_pu=0.1, x_pu=0.05)
        let buses = vec![make_bus(0, true), make_bus(1, false)];
        let branches = vec![make_branch(0, 1, 10.0, 5.0)];
        let meas = vec![
            make_meas(0, DsseMeasurementType::VoltageMagnitude, 0, 1.0, 1e-6),
            make_meas(1, DsseMeasurementType::VoltageMagnitude, 1, 0.98, 1e-6),
            make_meas(2, DsseMeasurementType::ActivePowerInjection, 0, 0.0, 1e-4),
            make_meas(3, DsseMeasurementType::ReactivePowerInjection, 0, 0.0, 1e-4),
        ];
        let cfg = default_config(2);
        let mut se = DistributionStateEstimator::new(buses, branches, meas, cfg);
        let result = se.estimate().expect("estimate failed");

        assert!(result.converged, "should converge");
        assert_eq!(result.voltage_magnitudes.len(), 2);
        assert!(
            (result.voltage_magnitudes[0] - 1.0).abs() < 0.05,
            "V[0] = {}",
            result.voltage_magnitudes[0]
        );
        assert!(
            (result.voltage_magnitudes[1] - 0.98).abs() < 0.05,
            "V[1] = {}",
            result.voltage_magnitudes[1]
        );
    }

    #[test]
    fn test_estimate_3bus_radial() {
        let buses = vec![make_bus(0, true), make_bus(1, false), make_bus(2, false)];
        let branches = vec![make_branch(0, 1, 5.0, 2.0), make_branch(1, 2, 5.0, 2.0)];
        let meas = vec![
            make_meas(0, DsseMeasurementType::VoltageMagnitude, 0, 1.0, 1e-6),
            make_meas(1, DsseMeasurementType::VoltageMagnitude, 1, 0.99, 1e-6),
            make_meas(2, DsseMeasurementType::VoltageMagnitude, 2, 0.97, 1e-6),
            make_meas(3, DsseMeasurementType::ActivePowerInjection, 1, 0.0, 0.01),
        ];
        let cfg = default_config(3);
        let mut se = DistributionStateEstimator::new(buses, branches, meas, cfg);
        let result = se.estimate().expect("estimate failed");

        assert!(result.converged, "3-bus should converge");
        assert_eq!(result.voltage_magnitudes.len(), 3);
        assert_eq!(result.branch_currents.len(), 2);
        assert_eq!(result.branch_p_flow.len(), 2);
    }

    #[test]
    fn test_normalized_residuals_clean() {
        let meas = vec![
            make_meas(0, DsseMeasurementType::VoltageMagnitude, 0, 1.0, 1e-4),
            make_meas(1, DsseMeasurementType::VoltageMagnitude, 1, 0.98, 1e-4),
        ];
        let cfg = default_config(2);
        let se = DistributionStateEstimator::new(vec![], vec![], meas, cfg);
        let residuals = vec![0.0, 0.0];
        let nr = se.compute_normalized_residuals(&residuals);
        assert!(nr.iter().all(|&r| r.abs() < 1e-15));
    }

    #[test]
    fn test_bad_data_detection_large_residual() {
        let meas = vec![
            make_meas(10, DsseMeasurementType::VoltageMagnitude, 0, 1.0, 1e-4),
            make_meas(11, DsseMeasurementType::PseudoMeasurement, 1, 0.0, 0.04),
        ];
        let cfg = default_config(2);
        let se = DistributionStateEstimator::new(vec![], vec![], meas, cfg);
        // residual = 0.5 → |0.5| / sqrt(1e-4) = 0.5 / 0.01 = 50 >> 3.0
        let residuals = vec![0.5, 0.001];
        let nr = se.compute_normalized_residuals(&residuals);
        let bad = se.identify_bad_data(&nr);
        assert!(bad.contains(&10), "measurement 10 should be flagged");
        assert!(!bad.contains(&11), "measurement 11 should not be flagged");
    }

    #[test]
    fn test_chi_squared_statistic() {
        // variance = 1.0  →  w = 1.0 for meas 0
        // variance = 4.0  →  w = 0.25 for meas 1
        // residuals = [2.0, 4.0]
        // chi2 = 2² * 1.0 + 4² * 0.25 = 4 + 4 = 8
        let meas = vec![
            make_meas(0, DsseMeasurementType::VoltageMagnitude, 0, 1.0, 1.0),
            make_meas(1, DsseMeasurementType::VoltageMagnitude, 1, 1.0, 4.0),
        ];
        let cfg = default_config(2);
        let se = DistributionStateEstimator::new(vec![], vec![], meas, cfg);
        let residuals = vec![2.0, 4.0];
        let chi2 = se.chi_squared_test(&residuals);
        assert!((chi2 - 8.0).abs() < 1e-8, "chi2 = {chi2}");
    }

    #[test]
    fn test_branch_current_computation() {
        // bus 0 and bus 1 at same voltage (1∠0) → ΔV = 0 → I = 0
        let buses = vec![make_bus(0, true), make_bus(1, false)];
        let branches = vec![make_branch(0, 1, 10.0, 0.0)]; // r=10Ω → r_pu=0.1
        let cfg = default_config(2);
        let se = DistributionStateEstimator::new(buses, branches, vec![], cfg);
        let v = vec![1.0, 1.0];
        let theta = vec![0.0, 0.0];
        let currents = se.compute_branch_currents(&v, &theta);
        assert_eq!(currents.len(), 1);
        assert!(currents[0].abs() < 1e-10, "current = {}", currents[0]);
    }

    // ── Additional tests beyond the minimum 20 ────────────────────────────────

    #[test]
    fn test_branch_current_nonzero() {
        // bus 0: 1.0∠0, bus 1: 0.9∠0
        // r_pu = 0.1 (10Ω / 100Ω),  x_pu = 0
        // ΔV_real = 0.1  →  I_real = 0.1 / 0.01 = 10 pu
        let buses = vec![make_bus(0, true), make_bus(1, false)];
        let branches = vec![make_branch(0, 1, 10.0, 0.0)];
        let cfg = default_config(2);
        let se = DistributionStateEstimator::new(buses, branches, vec![], cfg);
        let v = vec![1.0, 0.9];
        let theta = vec![0.0, 0.0];
        let currents = se.compute_branch_currents(&v, &theta);
        assert_eq!(currents.len(), 1);
        assert!(currents[0] > 0.0, "current should be nonzero");
    }

    #[test]
    fn test_std_dev() {
        let m = make_meas(0, DsseMeasurementType::VoltageMagnitude, 0, 1.0, 0.0025);
        assert!((m.std_dev() - 0.05).abs() < 1e-10);
    }

    #[test]
    fn test_dsse_config_defaults() {
        let cfg = DsseConfig::default();
        assert_eq!(cfg.max_iterations, 30);
        assert!((cfg.convergence_tol - 1e-4).abs() < 1e-15);
        assert!((cfg.bad_data_threshold - 3.0).abs() < 1e-10);
        assert!(!cfg.use_branch_current_form);
    }

    #[test]
    fn test_estimate_invalid_no_substation() {
        let buses = vec![make_bus(0, false), make_bus(1, false)];
        let branches = vec![make_branch(0, 1, 5.0, 2.0)];
        let meas = vec![make_meas(
            0,
            DsseMeasurementType::VoltageMagnitude,
            0,
            1.0,
            1e-4,
        )];
        let cfg = default_config(2);
        let mut se = DistributionStateEstimator::new(buses, branches, meas, cfg);
        assert!(
            se.estimate().is_err(),
            "should fail without a substation bus"
        );
    }

    #[test]
    fn test_estimate_invalid_no_measurements() {
        let buses = vec![make_bus(0, true), make_bus(1, false)];
        let branches = vec![make_branch(0, 1, 5.0, 2.0)];
        let cfg = default_config(2);
        let mut se = DistributionStateEstimator::new(buses, branches, vec![], cfg);
        assert!(se.estimate().is_err(), "should fail without measurements");
    }
}
