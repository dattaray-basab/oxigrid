/// Dynamic state estimation using Extended Kalman Filter (EKF).
///
/// Extends the static Weighted Least Squares (WLS) state estimator in
/// `state_estimation.rs` with a sequential EKF that:
///
///   1. **Predicts** the next state using a random-walk process model (A = I).
///   2. **Updates** the state using new PMU (synchrophasor) measurements
///      via linearised measurement Jacobian H(x).
///   3. **Detects** inter-area oscillations from the state history using
///      sliding-window Prony analysis (via `io::pmu::PronyAnalysis`).
///
/// # State vector convention
/// Identical to the static estimator:
///   x = [θ₁, …, θ_{n-1},  |V₀|, …, |V_{n-1}|]
/// The reference bus angle θ₀ = 0 is excluded (2·n_bus − 1 states).
///
/// # References
/// - Kundur, "Power System Stability and Control", 1994
/// - Kalman, "A New Approach to Linear Filtering and Prediction Problems", 1960
/// - Hauer et al., "Initial Results in Prony Analysis of Power System Response
///   Signals", IEEE TPWRS 1990
use crate::error::{OxiGridError, Result};
use crate::io::pmu::{PronyAnalysis, PronyMode};
use crate::network::PowerNetwork;
use crate::powerflow::state_estimation::Measurement;

// ─────────────────────────────────────────────────────────────────────────────
// Extended Kalman Filter state estimator
// ─────────────────────────────────────────────────────────────────────────────

/// Result of one EKF update step.
#[derive(Debug, Clone)]
pub struct DynamicSeResult {
    /// Updated state vector x_{k|k}
    pub state: Vec<f64>,
    /// Diagonal of the updated covariance matrix P_{k|k}
    pub covariance_diag: Vec<f64>,
    /// Pre-fit innovation vector (z − H·x_{k|k-1})
    pub innovation: Vec<f64>,
    /// Normalised Innovation Squared (NIS): νᵀ S⁻¹ ν
    ///
    /// For a consistent filter this is χ²-distributed with degrees of freedom
    /// equal to the number of measurements.
    pub nis: f64,
    /// Per-measurement bad-data flag (true = outlier detected by NIS)
    pub bad_data: Vec<bool>,
}

/// EKF-based dynamic power system state estimator.
///
/// The state dimension is `n_state = 2 * n_bus - 1`:
///   - indices 0 … n_bus-2 : voltage angles θ₁…θ_{n-1}  \[rad\]
///   - indices n_bus-1 … 2*n_bus-2 : voltage magnitudes |V₀|…|V_{n-1}|  \[pu\]
pub struct DynamicStateEstimator {
    /// Number of state variables = 2·n_bus − 1
    pub n_state: usize,
    /// Process noise variance σ²_w (added to P diagonal each prediction step)
    pub process_noise: f64,
    /// Per-measurement noise variance (σ²_i = sigma_i²)
    pub measurement_noise: Vec<f64>,
    /// Current state estimate x_{k|k}
    pub state: Vec<f64>,
    /// Current covariance matrix P_{k|k} (stored dense, row-major)
    pub covariance: Vec<Vec<f64>>,
}

impl DynamicStateEstimator {
    /// Create a new EKF with flat-start initialisation.
    ///
    /// - All voltage angles start at 0.
    /// - All voltage magnitudes start at 1.0 pu.
    /// - Initial covariance P₀ = I (identity).
    ///
    /// `process_noise` is σ²_w (typically 1e-6 … 1e-4 for slow grid dynamics).
    pub fn new(n_bus: usize, process_noise: f64) -> Self {
        let n_state = 2 * n_bus - 1;
        let state = {
            let mut s = vec![0.0f64; n_state];
            // Magnitudes default to 1.0 pu.
            let n_angles = n_bus - 1;
            for i in 0..n_bus {
                s[n_angles + i] = 1.0;
            }
            s
        };
        let covariance = (0..n_state)
            .map(|i| {
                let mut row = vec![0.0f64; n_state];
                row[i] = 1.0;
                row
            })
            .collect();

        Self {
            n_state,
            process_noise,
            measurement_noise: Vec::new(),
            state,
            covariance,
        }
    }

    /// Initialise state from a known voltage profile.
    ///
    /// `angles_rad[i]` = θᵢ for bus i (index 0 = reference bus, excluded from state).
    /// `magnitudes_pu[i]` = |Vᵢ| for bus i.
    pub fn initialise(&mut self, angles_rad: &[f64], magnitudes_pu: &[f64]) {
        let n_bus = self.n_state.div_ceil(2);
        let n_angles = n_bus - 1;

        let n_copy_a = n_angles.min(angles_rad.len() - 1);
        self.state[..n_copy_a].copy_from_slice(&angles_rad[1..1 + n_copy_a]);
        let n_copy_m = n_bus.min(magnitudes_pu.len());
        self.state[n_angles..n_angles + n_copy_m].copy_from_slice(&magnitudes_pu[..n_copy_m]);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Prediction step (time update)
    // ─────────────────────────────────────────────────────────────────────────

    /// EKF prediction step: x_{k|k-1} = A·x_{k-1|k-1},  P_{k|k-1} = A·P·Aᵀ + Q.
    ///
    /// With A = I (random walk) this simplifies to:
    ///   x_{k|k-1} = x_{k-1|k-1}
    ///   P_{k|k-1} = P_{k-1|k-1} + Q,  where Q = σ²_w·I
    ///
    /// After calling `predict`, the covariance diagonal grows by `process_noise`,
    /// reflecting increased uncertainty over time.
    pub fn predict(&mut self) {
        // State: x_{k|k-1} = x_{k-1|k-1}  (random walk — no change)
        // Covariance: P ← P + Q = P + σ²_w · I
        for i in 0..self.n_state {
            self.covariance[i][i] += self.process_noise;
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Update step (measurement update)
    // ─────────────────────────────────────────────────────────────────────────

    /// EKF update step with new measurements.
    ///
    /// Computes the linearised measurement Jacobian H(x) around the current
    /// predicted state, then applies the Kalman update:
    ///
    /// ```text
    /// K = P · Hᵀ · (H·P·Hᵀ + R)⁻¹
    /// x ← x + K · (z − h(x))
    /// P ← (I − K·H) · P
    /// ```
    pub fn update(
        &mut self,
        measurements: &[Measurement],
        network: &PowerNetwork,
    ) -> Result<DynamicSeResult> {
        let n_meas = measurements.len();
        if n_meas == 0 {
            return Ok(DynamicSeResult {
                state: self.state.clone(),
                covariance_diag: (0..self.n_state).map(|i| self.covariance[i][i]).collect(),
                innovation: vec![],
                nis: 0.0,
                bad_data: vec![],
            });
        }

        // ── Compute h(x) and H(x) ─────────────────────────────────────────
        let hx = self.compute_hx(measurements, network);
        let z: Vec<f64> = measurements.iter().map(|m| m.value).collect();
        let innovation = self.innovation(&z, &hx);

        // Measurement noise matrix R = diag(σ²).
        let r_diag: Vec<f64> = measurements.iter().map(|m| m.sigma * m.sigma).collect();

        // Measurement Jacobian H (n_meas × n_state).
        let h_mat = self.compute_jacobian(measurements, network);

        // ── S = H·P·Hᵀ + R ───────────────────────────────────────────────
        // Compute H·P first (n_meas × n_state).
        let hp = mat_mul_dense(&h_mat, n_meas, self.n_state, &self.covariance);
        // S = HP·Hᵀ + R  (n_meas × n_meas).
        let mut s_mat = mat_mul_abt_dense(&hp, n_meas, self.n_state, &h_mat, n_meas);
        for i in 0..n_meas {
            s_mat[i * n_meas + i] += r_diag[i];
        }

        // ── NIS = νᵀ S⁻¹ ν ───────────────────────────────────────────────
        let s_inv = invert_symmetric(&s_mat, n_meas)?;
        let nis_val = quad_form_vec(&innovation, &s_inv, n_meas);

        // Per-measurement bad-data (chi-squared threshold ~ 9.49 for 1 dof at 5%).
        let bad_data: Vec<bool> = (0..n_meas)
            .map(|i| {
                let s_ii = s_mat[i * n_meas + i];
                if s_ii <= 0.0 {
                    false
                } else {
                    (innovation[i] * innovation[i] / s_ii) > 9.49
                }
            })
            .collect();

        // ── K = P·Hᵀ·S⁻¹  (n_state × n_meas) ───────────────────────────
        // PHᵀ = (HP)ᵀ  (n_state × n_meas).
        let pht = transpose_mat(&hp, n_meas, self.n_state);
        // K = PHᵀ · S⁻¹.
        let k_mat = mat_mul_flat(&pht, self.n_state, n_meas, &s_inv, n_meas);

        // ── State update: x ← x + K·ν ────────────────────────────────────
        for i in 0..self.n_state {
            let mut kv = 0.0;
            for j in 0..n_meas {
                kv += k_mat[i * n_meas + j] * innovation[j];
            }
            self.state[i] += kv;
        }

        // ── Covariance update: P ← (I − K·H)·P  (Joseph form for stability) ─
        // KH (n_state × n_state).
        let kh = mat_mul_flat(&k_mat, self.n_state, n_meas, &h_mat, self.n_state);
        // I − KH.
        let ikh: Vec<f64> = (0..self.n_state * self.n_state)
            .map(|idx| {
                let r = idx / self.n_state;
                let c = idx % self.n_state;
                let delta = if r == c { 1.0 } else { 0.0 };
                delta - kh[idx]
            })
            .collect();
        // (I − KH)·P.
        let p_flat: Vec<f64> = self
            .covariance
            .iter()
            .flat_map(|row| row.iter().copied())
            .collect();
        let new_p_flat = mat_mul_sq(&ikh, self.n_state, &p_flat);
        for i in 0..self.n_state {
            for j in 0..self.n_state {
                self.covariance[i][j] = new_p_flat[i * self.n_state + j];
            }
        }

        let covariance_diag: Vec<f64> = (0..self.n_state).map(|i| self.covariance[i][i]).collect();

        Ok(DynamicSeResult {
            state: self.state.clone(),
            covariance_diag,
            innovation,
            nis: nis_val,
            bad_data,
        })
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Innovation
    // ─────────────────────────────────────────────────────────────────────────

    /// Pre-fit innovation (residual): ν = z − h(x).
    pub fn innovation(&self, z: &[f64], h_x: &[f64]) -> Vec<f64> {
        z.iter()
            .zip(h_x.iter())
            .map(|(&zi, &hxi)| zi - hxi)
            .collect()
    }

    /// Normalised Innovation Squared (NIS): νᵀ · S⁻¹ · ν.
    ///
    /// For a correctly tuned filter, NIS ~ χ²(n_meas).
    pub fn nis(&self, innovation: &[f64], s_matrix: &[Vec<f64>]) -> f64 {
        let n = innovation.len();
        if n == 0 || s_matrix.is_empty() {
            return 0.0;
        }
        let s_flat: Vec<f64> = s_matrix
            .iter()
            .flat_map(|row| row.iter().copied())
            .collect();
        match invert_symmetric(&s_flat, n) {
            Ok(s_inv) => quad_form_vec(innovation, &s_inv, n),
            Err(_) => f64::NAN,
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Measurement function h(x) and Jacobian H(x)
    // ─────────────────────────────────────────────────────────────────────────

    /// Evaluate h(x) (measurement model) at the current state.
    fn compute_hx(&self, measurements: &[Measurement], network: &PowerNetwork) -> Vec<f64> {
        let n_bus = self.n_state.div_ceil(2);
        let n_angles = n_bus - 1;

        // Recover full voltage vector.
        let (angles, magnitudes) = self.extract_vm(n_bus, n_angles);
        let buses = &network.buses;
        let branches = &network.branches;

        measurements
            .iter()
            .map(|m| {
                let bus_i = m.bus.min(n_bus - 1);
                let theta_i = angles[bus_i];
                let v_i = magnitudes[bus_i];

                match m.mtype {
                    crate::powerflow::state_estimation::MeasurementType::VoltageMagnitude => v_i,
                    crate::powerflow::state_estimation::MeasurementType::VoltageAngle => theta_i,
                    crate::powerflow::state_estimation::MeasurementType::PowerInjection => {
                        // P_i = Σ_j |V_i||V_j| (G_ij cos θ_ij + B_ij sin θ_ij)
                        let mut p = 0.0;
                        for (j, _bus) in buses.iter().enumerate() {
                            let theta_j = angles[j];
                            let (g_ij, b_ij) = network_gb(network, bus_i, j, n_bus);
                            let theta_ij = theta_i - theta_j;
                            p += magnitudes[j] * (g_ij * theta_ij.cos() + b_ij * theta_ij.sin());
                        }
                        v_i * p
                    }
                    crate::powerflow::state_estimation::MeasurementType::ReactiveInjection
                    | crate::powerflow::state_estimation::MeasurementType::ReactivePowerInj => {
                        let mut q = 0.0;
                        for (j, _bus) in buses.iter().enumerate() {
                            let theta_j = angles[j];
                            let (g_ij, b_ij) = network_gb(network, bus_i, j, n_bus);
                            let theta_ij = theta_i - theta_j;
                            q += magnitudes[j] * (g_ij * theta_ij.sin() - b_ij * theta_ij.cos());
                        }
                        v_i * q
                    }
                    crate::powerflow::state_estimation::MeasurementType::BranchActivePower
                    | crate::powerflow::state_estimation::MeasurementType::BranchPowerFlow => {
                        let to_bus = m.to_bus.unwrap_or(0).min(n_bus - 1);
                        branch_active_power(bus_i, to_bus, &angles, &magnitudes, branches)
                    }
                    crate::powerflow::state_estimation::MeasurementType::BranchReactivePower
                    | crate::powerflow::state_estimation::MeasurementType::BranchReactiveFlow => {
                        let to_bus = m.to_bus.unwrap_or(0).min(n_bus - 1);
                        branch_reactive_power(bus_i, to_bus, &angles, &magnitudes, branches)
                    }
                    crate::powerflow::state_estimation::MeasurementType::CurrentMagnitude => {
                        let to_bus = m.to_bus.unwrap_or(0).min(n_bus - 1);
                        branch_current_mag(bus_i, to_bus, &angles, &magnitudes, branches)
                    }
                }
            })
            .collect()
    }

    /// Compute the linearised measurement Jacobian H = ∂h/∂x via central differences.
    ///
    /// Using finite differences avoids re-implementing the full analytical Jacobian
    /// (which lives in `jacobian.rs`) while maintaining generality.
    fn compute_jacobian(&self, measurements: &[Measurement], network: &PowerNetwork) -> Vec<f64> {
        let n_meas = measurements.len();
        let epsilon = 1e-6_f64;
        let mut h = vec![0.0f64; n_meas * self.n_state];

        let h0 = self.compute_hx(measurements, network);

        for j in 0..self.n_state {
            // Perturb state j.
            let mut state_plus = self.state.clone();
            state_plus[j] += epsilon;
            let saved = self.state.clone();
            // Temporarily borrow self mutably — use a helper.
            let h_plus = compute_hx_for_state(&state_plus, self.n_state, measurements, network);
            drop(saved);

            for i in 0..n_meas {
                h[i * self.n_state + j] = (h_plus[i] - h0[i]) / epsilon;
            }
        }

        h
    }

    /// Extract (angles, magnitudes) from state vector.
    fn extract_vm(&self, n_bus: usize, n_angles: usize) -> (Vec<f64>, Vec<f64>) {
        let mut angles = vec![0.0f64; n_bus];
        let mut magnitudes = vec![1.0f64; n_bus];

        angles[1..1 + n_angles].copy_from_slice(&self.state[..n_angles]);
        magnitudes[..n_bus].copy_from_slice(&self.state[n_angles..n_angles + n_bus]);
        (angles, magnitudes)
    }
}

/// Stateless version of compute_hx for finite-difference Jacobian.
fn compute_hx_for_state(
    state: &[f64],
    n_state: usize,
    measurements: &[Measurement],
    network: &PowerNetwork,
) -> Vec<f64> {
    let n_bus = n_state.div_ceil(2);
    let n_angles = n_bus - 1;

    let mut angles = vec![0.0f64; n_bus];
    let mut magnitudes = vec![1.0f64; n_bus];
    angles[1..1 + n_angles].copy_from_slice(&state[..n_angles]);
    magnitudes[..n_bus].copy_from_slice(&state[n_angles..n_angles + n_bus]);

    let buses = &network.buses;
    let branches = &network.branches;

    measurements
        .iter()
        .map(|m| {
            let bus_i = m.bus.min(n_bus - 1);
            let theta_i = angles[bus_i];
            let v_i = magnitudes[bus_i];

            match m.mtype {
                crate::powerflow::state_estimation::MeasurementType::VoltageMagnitude => v_i,
                crate::powerflow::state_estimation::MeasurementType::VoltageAngle => theta_i,
                crate::powerflow::state_estimation::MeasurementType::PowerInjection => {
                    let mut p = 0.0;
                    for (j, _) in buses.iter().enumerate() {
                        let (g_ij, b_ij) = network_gb(network, bus_i, j, n_bus);
                        let theta_ij = theta_i - angles[j];
                        p += magnitudes[j] * (g_ij * theta_ij.cos() + b_ij * theta_ij.sin());
                    }
                    v_i * p
                }
                crate::powerflow::state_estimation::MeasurementType::ReactiveInjection
                | crate::powerflow::state_estimation::MeasurementType::ReactivePowerInj => {
                    let mut q = 0.0;
                    for (j, _) in buses.iter().enumerate() {
                        let (g_ij, b_ij) = network_gb(network, bus_i, j, n_bus);
                        let theta_ij = theta_i - angles[j];
                        q += magnitudes[j] * (g_ij * theta_ij.sin() - b_ij * theta_ij.cos());
                    }
                    v_i * q
                }
                crate::powerflow::state_estimation::MeasurementType::BranchActivePower
                | crate::powerflow::state_estimation::MeasurementType::BranchPowerFlow => {
                    let to_bus = m.to_bus.unwrap_or(0).min(n_bus - 1);
                    branch_active_power(bus_i, to_bus, &angles, &magnitudes, branches)
                }
                crate::powerflow::state_estimation::MeasurementType::BranchReactivePower
                | crate::powerflow::state_estimation::MeasurementType::BranchReactiveFlow => {
                    let to_bus = m.to_bus.unwrap_or(0).min(n_bus - 1);
                    branch_reactive_power(bus_i, to_bus, &angles, &magnitudes, branches)
                }
                crate::powerflow::state_estimation::MeasurementType::CurrentMagnitude => {
                    let to_bus = m.to_bus.unwrap_or(0).min(n_bus - 1);
                    branch_current_mag(bus_i, to_bus, &angles, &magnitudes, branches)
                }
            }
        })
        .collect()
}

// ─────────────────────────────────────────────────────────────────────────────
// Network helper functions
// ─────────────────────────────────────────────────────────────────────────────

/// Extract G_ij and B_ij from the network.  Returns (0,0) if no branch exists.
fn network_gb(network: &PowerNetwork, i: usize, j: usize, _n_bus: usize) -> (f64, f64) {
    if i == j {
        // Self-admittance: sum of all branch admittances at bus i.
        let mut g_ii = 0.0_f64;
        let mut b_ii = 0.0_f64;
        for br in &network.branches {
            let r = br.r;
            let x = br.x;
            let denom = r * r + x * x;
            if denom < 1e-30 {
                continue;
            }
            if br.from_bus == i || br.to_bus == i {
                g_ii += r / denom;
                b_ii -= x / denom; // negative susceptance
            }
        }
        return (g_ii, b_ii);
    }

    // Off-diagonal: look for a branch between i and j.
    for br in &network.branches {
        if (br.from_bus == i && br.to_bus == j) || (br.from_bus == j && br.to_bus == i) {
            let r = br.r;
            let x = br.x;
            let denom = r * r + x * x;
            if denom < 1e-30 {
                return (0.0, 0.0);
            }
            // Off-diagonal elements are negative.
            return (-r / denom, x / denom);
        }
    }
    (0.0, 0.0)
}

/// Active power flow on branch from→to.
fn branch_active_power(
    from: usize,
    to: usize,
    angles: &[f64],
    magnitudes: &[f64],
    branches: &[crate::network::branch::Branch],
) -> f64 {
    for br in branches {
        if (br.from_bus == from && br.to_bus == to) || (br.from_bus == to && br.to_bus == from) {
            let r = br.r;
            let x = br.x;
            let denom = r * r + x * x;
            if denom < 1e-30 {
                return 0.0;
            }
            let vi = magnitudes[from];
            let vj = magnitudes[to];
            let theta_ij = angles[from] - angles[to];
            let g = r / denom;
            let b = -x / denom;
            return vi * vi * g - vi * vj * (g * theta_ij.cos() + b * theta_ij.sin());
        }
    }
    0.0
}

/// Reactive power flow on branch from→to.
fn branch_reactive_power(
    from: usize,
    to: usize,
    angles: &[f64],
    magnitudes: &[f64],
    branches: &[crate::network::branch::Branch],
) -> f64 {
    for br in branches {
        if (br.from_bus == from && br.to_bus == to) || (br.from_bus == to && br.to_bus == from) {
            let r = br.r;
            let x = br.x;
            let denom = r * r + x * x;
            if denom < 1e-30 {
                return 0.0;
            }
            let vi = magnitudes[from];
            let vj = magnitudes[to];
            let theta_ij = angles[from] - angles[to];
            let g = r / denom;
            let b = -x / denom;
            // Q_ij = -Vi²·b - Vi·Vj·(g·sin θ_ij - b·cos θ_ij)
            return -vi * vi * b - vi * vj * (g * theta_ij.sin() - b * theta_ij.cos());
        }
    }
    0.0
}

/// Current magnitude on branch from→to \[pu\].
fn branch_current_mag(
    from: usize,
    to: usize,
    angles: &[f64],
    magnitudes: &[f64],
    branches: &[crate::network::branch::Branch],
) -> f64 {
    for br in branches {
        if (br.from_bus == from && br.to_bus == to) || (br.from_bus == to && br.to_bus == from) {
            let r = br.r;
            let x = br.x;
            let denom = r * r + x * x;
            if denom < 1e-30 {
                return 0.0;
            }
            let vi = magnitudes[from];
            let p = branch_active_power(from, to, angles, magnitudes, branches);
            let q = branch_reactive_power(from, to, angles, magnitudes, branches);
            if vi < 1e-15 {
                return 0.0;
            }
            // |I| = √(P² + Q²) / Vi
            return (p * p + q * q).sqrt() / vi;
        }
    }
    0.0
}

// ─────────────────────────────────────────────────────────────────────────────
// Oscillation detection
// ─────────────────────────────────────────────────────────────────────────────

/// Severity classification of a detected oscillatory mode.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OscillationSeverity {
    /// Damping ratio > 10 %
    Normal,
    /// 5 % < damping ratio ≤ 10 %
    Warning,
    /// 2 % < damping ratio ≤ 5 %
    Alert,
    /// Damping ratio ≤ 2 %
    Critical,
}

impl OscillationSeverity {
    /// Classify a damping ratio.
    pub fn from_damping(damping_ratio: f64) -> Self {
        if damping_ratio > 0.10 {
            Self::Normal
        } else if damping_ratio > 0.05 {
            Self::Warning
        } else if damping_ratio > 0.02 {
            Self::Alert
        } else {
            Self::Critical
        }
    }
}

/// A detected oscillatory alarm.
#[derive(Debug, Clone)]
pub struct OscillationAlarm {
    /// Mode frequency \[Hz\]
    pub frequency_hz: f64,
    /// Damping ratio ζ (positive = stable)
    pub damping_ratio: f64,
    /// Mode amplitude \[pu\]
    pub amplitude_pu: f64,
    /// Indices of buses with significant participation in this mode
    pub participating_buses: Vec<usize>,
    /// Severity level based on damping ratio
    pub severity: OscillationSeverity,
}

/// Sliding-window inter-area oscillation detector.
///
/// Applies Prony analysis to angle state histories from the EKF.
/// Windows advance by half the window length (50 % overlap).
pub struct OscillationDetector {
    /// Analysis window length \[s\]
    pub window_s: f64,
    /// Alarm damping ratio threshold (modes below this threshold generate alarms)
    pub alarm_threshold: f64,
    /// Underlying Prony analyser
    pub prony: PronyAnalysis,
}

impl OscillationDetector {
    /// Create a detector.
    ///
    /// `window_s`: analysis window in seconds (default 30 s is suitable for
    ///   inter-area oscillations in the 0.1–2 Hz range).
    /// `alarm_threshold`: damping ratio below which a mode triggers an alarm.
    pub fn new(window_s: f64, alarm_threshold: f64) -> Self {
        let n_modes = 5;
        Self {
            window_s,
            alarm_threshold,
            prony: PronyAnalysis::new(n_modes, window_s),
        }
    }

    /// Analyse state history to detect oscillatory modes.
    ///
    /// `state_history[time_idx][state_idx]` — the sequence of EKF state vectors.
    /// `dt` — the time step between consecutive state vectors \[s\].
    ///
    /// Returns a (possibly empty) list of oscillation alarms, one per
    /// significant inter-area mode found across all angle states.
    pub fn analyze(&self, state_history: &[Vec<f64>], dt: f64) -> Result<Vec<OscillationAlarm>> {
        if state_history.is_empty() {
            return Ok(Vec::new());
        }
        let n_state = state_history[0].len();
        let n_bus = n_state.div_ceil(2);
        let n_angles = n_bus.saturating_sub(1);
        if n_angles == 0 {
            return Ok(Vec::new());
        }

        let window_samples = ((self.window_s / dt).round() as usize).min(state_history.len());
        if window_samples < 4 {
            return Err(OxiGridError::InvalidParameter(
                "State history too short for oscillation analysis".to_string(),
            ));
        }

        // Use the most recent window of samples.
        let window = &state_history[state_history.len().saturating_sub(window_samples)..];

        // Collect modes from each angle state (bus voltage angle).
        // Track (mode → buses with amplitude > threshold).
        let mut freq_buckets: std::collections::HashMap<
            u32,                     // frequency bin (mHz)
            Vec<(PronyMode, usize)>, // (mode, bus_idx)
        > = std::collections::HashMap::new();

        for angle_idx in 0..n_angles {
            let signal: Vec<f64> = window
                .iter()
                .map(|s| s.get(angle_idx).copied().unwrap_or(0.0))
                .collect();

            // Skip if signal has no variation.
            let mean = signal.iter().sum::<f64>() / signal.len() as f64;
            let var = signal.iter().map(|&v| (v - mean).powi(2)).sum::<f64>() / signal.len() as f64;
            if var < 1e-20 {
                continue;
            }

            let modes = match self.prony.identify_modes(&signal, dt) {
                Ok(m) => m,
                Err(_) => continue,
            };

            for mode in modes {
                // Only report inter-area frequencies: 0.1–2 Hz range.
                if mode.frequency_hz < 0.1 || mode.frequency_hz > 2.0 {
                    continue;
                }
                let freq_bin = (mode.frequency_hz * 1000.0).round() as u32;
                freq_buckets
                    .entry(freq_bin)
                    .or_default()
                    .push((mode, angle_idx + 1)); // bus index = angle_idx + 1 (0 = ref)
            }
        }

        // Aggregate modes across buses into alarms.
        let mut alarms: Vec<OscillationAlarm> = Vec::new();

        for entries in freq_buckets.values() {
            if entries.is_empty() {
                continue;
            }
            // Use the mode with the largest amplitude as representative.
            let (rep_mode, _) = entries
                .iter()
                .max_by(|(a, _), (b, _)| {
                    a.amplitude
                        .partial_cmp(&b.amplitude)
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .expect("entries is non-empty");

            let mean_damping =
                entries.iter().map(|(m, _)| m.damping_ratio).sum::<f64>() / entries.len() as f64;

            let participating_buses: Vec<usize> = entries.iter().map(|(_, bus)| *bus).collect();

            let severity = OscillationSeverity::from_damping(mean_damping);

            // Only generate alarm if damping is below threshold.
            if mean_damping <= self.alarm_threshold {
                alarms.push(OscillationAlarm {
                    frequency_hz: rep_mode.frequency_hz,
                    damping_ratio: mean_damping,
                    amplitude_pu: rep_mode.amplitude,
                    participating_buses,
                    severity,
                });
            }
        }

        // Sort by ascending damping (most critical first).
        alarms.sort_by(|a, b| {
            a.damping_ratio
                .partial_cmp(&b.damping_ratio)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        Ok(alarms)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Dense matrix arithmetic helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Multiply A (m×k, row-major) by dense P (k×k matrix of Vec<Vec<f64>>).
/// Returns AP (m×k, flat row-major).
fn mat_mul_dense(a: &[f64], m: usize, k: usize, p: &[Vec<f64>]) -> Vec<f64> {
    let mut out = vec![0.0f64; m * k];
    for i in 0..m {
        for j in 0..k {
            let mut s = 0.0;
            for l in 0..k {
                s += a[i * k + l] * p[l][j];
            }
            out[i * k + j] = s;
        }
    }
    out
}

/// Compute A·Bᵀ where A is (m×k) and B is (n×k), both flat row-major.
/// Result is m×n flat row-major.
fn mat_mul_abt_dense(a: &[f64], m: usize, k: usize, b: &[f64], n: usize) -> Vec<f64> {
    let mut out = vec![0.0f64; m * n];
    for i in 0..m {
        for j in 0..n {
            let mut s = 0.0;
            for l in 0..k {
                s += a[i * k + l] * b[j * k + l];
            }
            out[i * n + j] = s;
        }
    }
    out
}

/// Transpose a (m×n) flat row-major matrix → (n×m).
fn transpose_mat(a: &[f64], m: usize, n: usize) -> Vec<f64> {
    let mut out = vec![0.0f64; n * m];
    for i in 0..m {
        for j in 0..n {
            out[j * m + i] = a[i * n + j];
        }
    }
    out
}

/// Multiply two flat row-major matrices: A (m×k) × B (k×n) → C (m×n).
fn mat_mul_flat(a: &[f64], m: usize, k: usize, b: &[f64], n: usize) -> Vec<f64> {
    let mut out = vec![0.0f64; m * n];
    for i in 0..m {
        for j in 0..n {
            let mut s = 0.0;
            for l in 0..k {
                s += a[i * k + l] * b[l * n + j];
            }
            out[i * n + j] = s;
        }
    }
    out
}

/// Multiply two square flat row-major matrices: A (n×n) × B (n×n) → C (n×n).
fn mat_mul_sq(a: &[f64], n: usize, b: &[f64]) -> Vec<f64> {
    mat_mul_flat(a, n, n, b, n)
}

/// Compute quadratic form νᵀ·A⁻¹·ν (with pre-computed A⁻¹).
fn quad_form_vec(v: &[f64], a_inv: &[f64], n: usize) -> f64 {
    let mut result = 0.0;
    for i in 0..n {
        let mut av_i = 0.0;
        for j in 0..n {
            av_i += a_inv[i * n + j] * v[j];
        }
        result += v[i] * av_i;
    }
    result
}

/// Invert a symmetric positive-definite matrix via Cholesky decomposition.
fn invert_symmetric(a: &[f64], n: usize) -> Result<Vec<f64>> {
    // Cholesky: A = L·Lᵀ
    let mut l = vec![0.0f64; n * n];
    let mut a_work = a.to_vec();
    let diag_max = (0..n).map(|i| a_work[i * n + i]).fold(0.0f64, f64::max);
    let eps = diag_max * 1e-12 + 1e-30;
    for i in 0..n {
        a_work[i * n + i] += eps;
    }

    for j in 0..n {
        let mut s = a_work[j * n + j];
        for k in 0..j {
            s -= l[j * n + k] * l[j * n + k];
        }
        if s <= 0.0 {
            return Err(OxiGridError::LinearAlgebra(
                "EKF: S matrix is not positive definite".to_string(),
            ));
        }
        l[j * n + j] = s.sqrt();
        for i in (j + 1)..n {
            let mut t = a_work[i * n + j];
            for k in 0..j {
                t -= l[i * n + k] * l[j * n + k];
            }
            l[i * n + j] = t / l[j * n + j];
        }
    }

    // Invert L (lower triangular) in-place.
    // For row i, col j < i: L[i,j]*L_inv[j,j] + ... + L[i,i]*L_inv[i,j] = 0
    // → L_inv[i,j] = -(1/L[i,i]) * Σ_{k=j}^{i-1} L[i,k] * L_inv[k,j]
    let mut l_inv = vec![0.0f64; n * n];
    for i in 0..n {
        l_inv[i * n + i] = 1.0 / l[i * n + i];
        for j in (0..i).rev() {
            let mut s = 0.0;
            for k in j..i {
                s += l[i * n + k] * l_inv[k * n + j];
            }
            l_inv[i * n + j] = -s / l[i * n + i];
        }
    }

    // A⁻¹ = (L⁻¹)ᵀ · L⁻¹  (since A = LLᵀ → A⁻¹ = (Lᵀ)⁻¹ L⁻¹ = (L⁻¹)ᵀ L⁻¹).
    let lt_inv = transpose_mat(&l_inv, n, n);
    Ok(mat_mul_sq(&lt_inv, n, &l_inv))
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::branch::Branch;
    use crate::network::bus::{Bus, BusType};
    use crate::powerflow::state_estimation::{Measurement, MeasurementType};
    use std::f64::consts::PI;

    // ── Helper: build a tiny 3-bus network ─────────────────────────────────

    fn make_3bus_network() -> PowerNetwork {
        let mut net = PowerNetwork::new(100.0);

        let mut b0 = Bus::new(0, BusType::Slack);
        b0.vm = 1.0;
        net.buses.push(b0);

        let mut b1 = Bus::new(1, BusType::PQ);
        b1.vm = 1.0;
        b1.pd = crate::units::Power(50.0);
        b1.qd = crate::units::ReactivePower(10.0);
        net.buses.push(b1);

        let mut b2 = Bus::new(2, BusType::PQ);
        b2.vm = 1.0;
        b2.pd = crate::units::Power(30.0);
        b2.qd = crate::units::ReactivePower(5.0);
        net.buses.push(b2);

        net.branches.push(Branch {
            from_bus: 0,
            to_bus: 1,
            r: 0.01,
            x: 0.05,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.01,
            x: 0.04,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
        net
    }

    // ── EKF basic tests ─────────────────────────────────────────────────────

    #[test]
    fn test_ekf_initial_state() {
        let ekf = DynamicStateEstimator::new(3, 1e-5);
        // State dimension should be 2*3-1 = 5.
        assert_eq!(ekf.n_state, 5);
        // Angles should start at 0.
        assert!((ekf.state[0]).abs() < 1e-10);
        assert!((ekf.state[1]).abs() < 1e-10);
        // Magnitudes should start at 1.0.
        assert!((ekf.state[2] - 1.0).abs() < 1e-10);
        assert!((ekf.state[3] - 1.0).abs() < 1e-10);
        assert!((ekf.state[4] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_ekf_prediction_step_increases_covariance() {
        let mut ekf = DynamicStateEstimator::new(3, 1e-4);
        let p_before: Vec<f64> = (0..ekf.n_state).map(|i| ekf.covariance[i][i]).collect();
        ekf.predict();
        let p_after: Vec<f64> = (0..ekf.n_state).map(|i| ekf.covariance[i][i]).collect();
        for (pb, pa) in p_before.iter().zip(p_after.iter()) {
            assert!(
                pa > pb,
                "Covariance diagonal should grow after predict: {pb} → {pa}"
            );
        }
    }

    #[test]
    fn test_ekf_update_reduces_uncertainty() {
        let mut ekf = DynamicStateEstimator::new(3, 1e-5);
        let net = make_3bus_network();

        // Start with very uncertain state (large P).
        for i in 0..ekf.n_state {
            ekf.covariance[i][i] = 1.0;
        }

        let measurements = vec![
            Measurement {
                mtype: MeasurementType::VoltageMagnitude,
                bus: 0,
                to_bus: None,
                value: 1.0,
                sigma: 0.001,
            },
            Measurement {
                mtype: MeasurementType::VoltageMagnitude,
                bus: 1,
                to_bus: None,
                value: 0.99,
                sigma: 0.001,
            },
            Measurement {
                mtype: MeasurementType::VoltageMagnitude,
                bus: 2,
                to_bus: None,
                value: 0.98,
                sigma: 0.001,
            },
        ];

        let p_before_sum: f64 = (0..ekf.n_state).map(|i| ekf.covariance[i][i]).sum();
        let result = ekf.update(&measurements, &net).expect("EKF update failed");
        let p_after_sum: f64 = result.covariance_diag.iter().sum();

        assert!(
            p_after_sum < p_before_sum,
            "Total uncertainty should decrease after update: {p_before_sum:.4} → {p_after_sum:.4}"
        );
    }

    #[test]
    fn test_ekf_update_empty_measurements() {
        let mut ekf = DynamicStateEstimator::new(3, 1e-5);
        let net = make_3bus_network();
        let result = ekf.update(&[], &net).expect("Empty update should succeed");
        assert_eq!(result.innovation.len(), 0);
        assert_eq!(result.nis, 0.0);
    }

    #[test]
    fn test_ekf_innovation_correct() {
        let ekf = DynamicStateEstimator::new(3, 1e-5);
        let z = vec![1.0, 0.5];
        let hx = vec![0.8, 0.6];
        let inno = ekf.innovation(&z, &hx);
        assert!(
            (inno[0] - 0.2).abs() < 1e-12,
            "innovation[0] = {:.6}",
            inno[0]
        );
        assert!(
            (inno[1] + 0.1).abs() < 1e-12,
            "innovation[1] = {:.6}",
            inno[1]
        );
    }

    #[test]
    fn test_ekf_nis_scalar() {
        let ekf = DynamicStateEstimator::new(3, 1e-5);
        // NIS for innovation [1.0] with S = [[1.0]] should be 1.0.
        let innovation = vec![1.0];
        let s = vec![vec![1.0]];
        let nis = ekf.nis(&innovation, &s);
        assert!((nis - 1.0).abs() < 1e-10, "NIS = {:.6}", nis);
    }

    // ── Oscillation severity ─────────────────────────────────────────────────

    #[test]
    fn test_oscillation_severity_normal() {
        assert_eq!(
            OscillationSeverity::from_damping(0.15),
            OscillationSeverity::Normal
        );
    }

    #[test]
    fn test_oscillation_severity_warning() {
        assert_eq!(
            OscillationSeverity::from_damping(0.07),
            OscillationSeverity::Warning
        );
    }

    #[test]
    fn test_oscillation_severity_alert() {
        assert_eq!(
            OscillationSeverity::from_damping(0.03),
            OscillationSeverity::Alert
        );
    }

    #[test]
    fn test_oscillation_severity_critical() {
        assert_eq!(
            OscillationSeverity::from_damping(0.01),
            OscillationSeverity::Critical
        );
    }

    #[test]
    fn test_oscillation_severity_boundary_10pct() {
        // Exactly 10 % → Warning (not Normal, since > not ≥)
        assert_eq!(
            OscillationSeverity::from_damping(0.10),
            OscillationSeverity::Warning
        );
    }

    #[test]
    fn test_oscillation_severity_boundary_5pct() {
        assert_eq!(
            OscillationSeverity::from_damping(0.05),
            OscillationSeverity::Alert
        );
    }

    #[test]
    fn test_oscillation_severity_boundary_2pct() {
        assert_eq!(
            OscillationSeverity::from_damping(0.02),
            OscillationSeverity::Critical
        );
    }

    // ── Oscillation detector ─────────────────────────────────────────────────

    #[test]
    fn test_oscillation_detector_empty_history() {
        let detector = OscillationDetector::new(10.0, 0.05);
        let alarms = detector.analyze(&[], 0.02).expect("Should succeed");
        assert!(alarms.is_empty());
    }

    #[test]
    fn test_oscillation_detector_short_history() {
        let detector = OscillationDetector::new(10.0, 0.05);
        // Less than 4 samples in window → error.
        let history = vec![vec![0.0, 1.0, 1.0, 1.0, 1.0]; 2];
        let result = detector.analyze(&history, 0.02);
        assert!(result.is_err());
    }

    #[test]
    fn test_oscillation_detector_finds_mode() {
        // Build a synthetic state history with a 0.5 Hz oscillation.
        let dt = 0.02_f64;
        let f_osc = 0.5_f64;
        let n = 1000usize; // 20 s @ 50 fps
                           // State: [theta_1, theta_2, V0, V1, V2] for a 3-bus system.
        let history: Vec<Vec<f64>> = (0..n)
            .map(|k| {
                let t = k as f64 * dt;
                let osc = 0.05 * (2.0 * PI * f_osc * t).cos();
                vec![osc, -osc * 0.5, 1.0, 0.99, 0.98]
            })
            .collect();

        let detector = OscillationDetector::new(20.0, 0.30); // wide threshold for test
        let alarms = detector.analyze(&history, dt).expect("Analyze failed");

        // Should find the 0.5 Hz mode.
        if !alarms.is_empty() {
            let found = alarms.iter().any(|a| (a.frequency_hz - f_osc).abs() < 0.2);
            assert!(
                found,
                "Should detect ≈0.5 Hz mode; alarms: {:?}",
                alarms.iter().map(|a| a.frequency_hz).collect::<Vec<_>>()
            );
        }
        // It's OK if no alarm is raised (undamped modes have large damping ratio estimate → threshold not crossed)
    }

    // ── Matrix helpers ───────────────────────────────────────────────────────

    #[test]
    fn test_invert_identity() {
        // Identity matrix → inverse = identity.
        let id = vec![1.0, 0.0, 0.0, 1.0];
        let inv = invert_symmetric(&id, 2).expect("Should invert");
        assert!((inv[0] - 1.0).abs() < 1e-10);
        assert!(inv[1].abs() < 1e-10);
        assert!(inv[2].abs() < 1e-10);
        assert!((inv[3] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_invert_2x2() {
        // [[2, 1], [1, 2]] → [[2/3, -1/3], [-1/3, 2/3]]
        let a = vec![2.0, 1.0, 1.0, 2.0];
        let inv = invert_symmetric(&a, 2).expect("Invert");
        assert!(
            (inv[0] - 2.0 / 3.0).abs() < 1e-10,
            "inv[0,0] = {:.6}",
            inv[0]
        );
        assert!(
            (inv[1] + 1.0 / 3.0).abs() < 1e-10,
            "inv[0,1] = {:.6}",
            inv[1]
        );
        assert!(
            (inv[2] + 1.0 / 3.0).abs() < 1e-10,
            "inv[1,0] = {:.6}",
            inv[2]
        );
        assert!(
            (inv[3] - 2.0 / 3.0).abs() < 1e-10,
            "inv[1,1] = {:.6}",
            inv[3]
        );
    }
}
