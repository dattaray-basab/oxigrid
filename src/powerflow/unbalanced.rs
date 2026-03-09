//! Three-phase unbalanced power flow for distribution networks.
//!
//! Handles single-phase loads, asymmetric line impedances, and delta/wye
//! transformer connections — all critical for accurate distribution-system
//! modelling.
//!
//! # Method
//!
//! The solver builds a 3n×3n phase-frame admittance matrix (where n is the
//! number of buses), then applies full Newton-Raphson iterations with a
//! 6-component mismatch vector per non-slack bus [ΔP_a, ΔP_b, ΔP_c,
//! ΔQ_a, ΔQ_b, ΔQ_c].
//!
//! # References
//!
//! * Kersting, W. H. (2017). *Distribution System Modeling and Analysis*.
//! * IEEE PES Distribution Test Feeder Working Group — 13-bus test case.

use crate::error::{OxiGridError, Result};
use crate::network::bus::BusType;
use num_complex::Complex64;
use oxiblas_lapack::lu::Lu;
use oxiblas_matrix::Mat;
use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Data structures
// ---------------------------------------------------------------------------

/// Three-phase bus for unbalanced distribution analysis.
#[derive(Debug, Clone)]
pub struct ThreePhaseBus {
    /// External bus identifier (1-based).
    pub id: usize,
    /// Human-readable bus name.
    pub name: String,
    /// Bus type (Slack / PQ — PV is not directly supported in unbalanced flow).
    pub bus_type: BusType,
    /// Per-phase complex voltage phasors [A, B, C] in per-unit.
    pub v: [Complex64; 3],
    /// Per-phase scheduled real-power injections [A, B, C] in MW.
    /// Positive = generation minus load (net injection).
    pub p_inj: [f64; 3],
    /// Per-phase scheduled reactive-power injections [A, B, C] in MVAr.
    pub q_inj: [f64; 3],
    /// Phase connectivity flags: `true` means the phase is present.
    pub phases: [bool; 3],
}

impl ThreePhaseBus {
    /// Construct a three-phase PQ bus with flat-start voltages (1∠0°, 1∠−120°, 1∠+120°).
    pub fn new_pq(id: usize, name: impl Into<String>) -> Self {
        Self {
            id,
            name: name.into(),
            bus_type: BusType::PQ,
            v: balanced_init_voltages(),
            p_inj: [0.0; 3],
            q_inj: [0.0; 3],
            phases: [true; 3],
        }
    }

    /// Construct a slack bus with fixed balanced voltages.
    pub fn new_slack(id: usize, name: impl Into<String>, v_pu: f64) -> Self {
        let a = Complex64::from_polar(v_pu, 0.0);
        let b = Complex64::from_polar(v_pu, -2.0 * PI / 3.0);
        let c = Complex64::from_polar(v_pu, 2.0 * PI / 3.0);
        Self {
            id,
            name: name.into(),
            bus_type: BusType::Slack,
            v: [a, b, c],
            p_inj: [0.0; 3],
            q_inj: [0.0; 3],
            phases: [true; 3],
        }
    }
}

/// Three-phase distribution-line branch with asymmetric series-impedance
/// and shunt-admittance matrices.
#[derive(Debug, Clone)]
pub struct ThreePhaseBranch {
    /// Internal index of the from-bus.
    pub from_bus: usize,
    /// Internal index of the to-bus.
    pub to_bus: usize,
    /// 3×3 series impedance matrix (Ω per km × line length) — phase frame.
    pub z_series: [[Complex64; 3]; 3],
    /// 3×3 shunt admittance matrix (total, split 50/50 at each end).
    pub y_shunt: [[Complex64; 3]; 3],
    /// Line length \[km\].
    pub length_km: f64,
    /// Three-phase kVA thermal rating (0 = unlimited).
    pub rating_kva: f64,
    /// Which phases are physically present on this segment.
    pub phases: [bool; 3],
}

impl ThreePhaseBranch {
    /// Construct a simple three-phase line from per-unit impedances.
    ///
    /// `z_diag` — per-phase self-impedance (Ω/km × km), same on all phases.
    /// `z_off`  — mutual impedance between phases.
    pub fn three_phase_symmetric(
        from_bus: usize,
        to_bus: usize,
        z_diag: Complex64,
        z_off: Complex64,
        length_km: f64,
        rating_kva: f64,
    ) -> Self {
        let z = [
            [z_diag, z_off, z_off],
            [z_off, z_diag, z_off],
            [z_off, z_off, z_diag],
        ];
        Self {
            from_bus,
            to_bus,
            z_series: z,
            y_shunt: [[Complex64::new(0.0, 0.0); 3]; 3],
            length_km,
            rating_kva,
            phases: [true; 3],
        }
    }
}

/// Transformer connection type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransformerConnection {
    WyeWye,
    WyeDelta,
    DeltaWye,
    DeltaDelta,
}

/// Three-phase transformer model.
#[derive(Debug, Clone)]
pub struct ThreePhaseTransformer {
    /// Internal index of the primary bus.
    pub from_bus: usize,
    /// Internal index of the secondary bus.
    pub to_bus: usize,
    /// Winding connection (Wye-Wye, Wye-Delta, etc.).
    pub connection: TransformerConnection,
    /// Rated three-phase apparent power \[kVA\].
    pub kva_rating: f64,
    /// Primary side line-to-line voltage \[kV\].
    pub v_primary_kv: f64,
    /// Secondary side line-to-line voltage \[kV\].
    pub v_secondary_kv: f64,
    /// Leakage impedance in per-unit on transformer base.
    pub z_pu: Complex64,
    /// Off-nominal turns ratio (1.0 = nominal).
    pub tap_ratio: f64,
}

impl ThreePhaseTransformer {
    /// Phase shift introduced by the winding connection \[radians\].
    /// Wye-Delta introduces −30° (standard American convention),
    /// Delta-Wye introduces +30°, others introduce 0°.
    pub fn phase_shift_rad(&self) -> f64 {
        match self.connection {
            TransformerConnection::WyeDelta => -PI / 6.0,
            TransformerConnection::DeltaWye => PI / 6.0,
            _ => 0.0,
        }
    }
}

/// Complete three-phase distribution network.
#[derive(Debug, Clone)]
pub struct ThreePhaseNetwork {
    /// All buses in the network (order defines the internal bus index).
    pub buses: Vec<ThreePhaseBus>,
    /// All line segments (branches).
    pub branches: Vec<ThreePhaseBranch>,
    /// All transformers.
    pub transformers: Vec<ThreePhaseTransformer>,
    /// Nominal base line-to-line voltage \[kV\].
    pub base_kv: f64,
    /// Nominal three-phase base power \[kVA\].
    pub base_kva: f64,
    /// Internal index of the slack bus.
    pub slack_bus: usize,
}

impl ThreePhaseNetwork {
    /// Return the number of buses.
    pub fn n_buses(&self) -> usize {
        self.buses.len()
    }
}

// ---------------------------------------------------------------------------
// Symmetrical-component operator
// ---------------------------------------------------------------------------

/// a = exp(j·2π/3)  — the Fortescue operator.
#[inline]
fn fortescue_a() -> Complex64 {
    Complex64::from_polar(1.0, 2.0 * PI / 3.0)
}

// ---------------------------------------------------------------------------
// Helper: balanced initial voltages
// ---------------------------------------------------------------------------

fn balanced_init_voltages() -> [Complex64; 3] {
    [
        Complex64::from_polar(1.0, 0.0),
        Complex64::from_polar(1.0, -2.0 * PI / 3.0),
        Complex64::from_polar(1.0, 2.0 * PI / 3.0),
    ]
}

// ---------------------------------------------------------------------------
// 3×3 matrix inversion (complex)
// ---------------------------------------------------------------------------

/// Invert a 3×3 complex matrix using Cramer's rule.
///
/// Returns `Err` if the matrix is numerically singular.
fn invert_3x3(m: &[[Complex64; 3]; 3]) -> Result<[[Complex64; 3]; 3]> {
    // Co-factors
    let c00 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
    let c01 = -(m[1][0] * m[2][2] - m[1][2] * m[2][0]);
    let c02 = m[1][0] * m[2][1] - m[1][1] * m[2][0];
    let c10 = -(m[0][1] * m[2][2] - m[0][2] * m[2][1]);
    let c11 = m[0][0] * m[2][2] - m[0][2] * m[2][0];
    let c12 = -(m[0][0] * m[2][1] - m[0][1] * m[2][0]);
    let c20 = m[0][1] * m[1][2] - m[0][2] * m[1][1];
    let c21 = -(m[0][0] * m[1][2] - m[0][2] * m[1][0]);
    let c22 = m[0][0] * m[1][1] - m[0][1] * m[1][0];

    let det = m[0][0] * c00 + m[0][1] * c01 + m[0][2] * c02;
    if det.norm() < 1e-30 {
        return Err(OxiGridError::LinearAlgebra(
            "3×3 branch impedance matrix is singular".to_string(),
        ));
    }
    let inv_det = Complex64::new(1.0, 0.0) / det;
    Ok([
        [c00 * inv_det, c10 * inv_det, c20 * inv_det],
        [c01 * inv_det, c11 * inv_det, c21 * inv_det],
        [c02 * inv_det, c12 * inv_det, c22 * inv_det],
    ])
}

// ---------------------------------------------------------------------------
// Three-phase admittance matrix (3n × 3n)
// ---------------------------------------------------------------------------

/// Build the full 3n×3n phase-frame admittance matrix for the network.
///
/// The matrix is stored as a dense `Vec<Vec<Complex64>>` with row/column
/// ordering: [bus0_a, bus0_b, bus0_c, bus1_a, bus1_b, bus1_c, …].
///
/// For each branch the 3×3 series admittance matrix is computed as
/// Y_s = Z_s^{-1}, then assembled into the 3n×3n Y-bus using the
/// standard primitive-network stamp:
///
/// ```text
/// Y_bus[3·i+p][3·i+q] += Y_s[p][q] + Y_sh[p][q]/2
/// Y_bus[3·j+p][3·j+q] += Y_s[p][q] + Y_sh[p][q]/2
/// Y_bus[3·i+p][3·j+q] -= Y_s[p][q]
/// Y_bus[3·j+p][3·i+q] -= Y_s[p][q]
/// ```
///
/// Transformers are stamped as ideal π-sections with appropriate tap/phase.
pub fn build_three_phase_ybus(network: &ThreePhaseNetwork) -> Result<Vec<Vec<Complex64>>> {
    let n = network.n_buses();
    let dim = 3 * n;
    let zero = Complex64::new(0.0, 0.0);
    let mut ybus = vec![vec![zero; dim]; dim];

    // ── Stamp branches ──────────────────────────────────────────────────────
    for branch in &network.branches {
        let i = branch.from_bus;
        let j = branch.to_bus;
        if i >= n || j >= n {
            return Err(OxiGridError::InvalidNetwork(format!(
                "Branch references out-of-range bus ({} or {}), n={}",
                i, j, n
            )));
        }

        // Series admittance matrix Y_s = Z_s^{-1}
        let y_s = invert_3x3(&branch.z_series)?;

        // Stamp self and mutual terms
        for p in 0..3 {
            if !branch.phases[p] {
                continue;
            }
            for q in 0..3 {
                if !branch.phases[q] {
                    continue;
                }
                let y_sh_half = branch.y_shunt[p][q] * 0.5;
                // From-bus diagonal block
                ybus[3 * i + p][3 * i + q] += y_s[p][q] + y_sh_half;
                // To-bus diagonal block
                ybus[3 * j + p][3 * j + q] += y_s[p][q] + y_sh_half;
                // Off-diagonal blocks (negative series admittance)
                ybus[3 * i + p][3 * j + q] -= y_s[p][q];
                ybus[3 * j + p][3 * i + q] -= y_s[p][q];
            }
        }
    }

    // ── Stamp transformers ──────────────────────────────────────────────────
    for xfmr in &network.transformers {
        let i = xfmr.from_bus;
        let j = xfmr.to_bus;
        if i >= n || j >= n {
            return Err(OxiGridError::InvalidNetwork(format!(
                "Transformer references out-of-range bus ({} or {}), n={}",
                i, j, n
            )));
        }

        // Convert per-unit leakage impedance to actual admittance on system base.
        // y_pu = 1 / z_pu (on transformer kVA base, converted to system base below)
        let kva_base_ratio = network.base_kva / xfmr.kva_rating;
        let y_series_pu = if xfmr.z_pu.norm() > 1e-30 {
            Complex64::new(1.0, 0.0) / xfmr.z_pu * kva_base_ratio
        } else {
            Complex64::new(1e6, 0.0) // ideal (lossless) transformer
        };

        // Effective complex turns ratio including phase shift
        let phi = xfmr.phase_shift_rad();
        let a_tap = Complex64::from_polar(xfmr.tap_ratio, phi);

        // π-model for each phase independently (diagonal transformer model)
        // Y_ii += y / |a|²,  Y_jj += y,  Y_ij -= y/a*,  Y_ji -= y/a
        let a_tap_sq = xfmr.tap_ratio * xfmr.tap_ratio;
        for ph in 0..3 {
            let row_i = 3 * i + ph;
            let row_j = 3 * j + ph;
            ybus[row_i][row_i] += y_series_pu / a_tap_sq;
            ybus[row_j][row_j] += y_series_pu;
            ybus[row_i][row_j] -= y_series_pu / a_tap.conj();
            ybus[row_j][row_i] -= y_series_pu / a_tap;
        }
    }

    Ok(ybus)
}

// ---------------------------------------------------------------------------
// Power injection calculation
// ---------------------------------------------------------------------------

/// Compute per-phase scheduled power injections as flat vectors.
///
/// `ThreePhaseBus::p_inj` and `q_inj` are stored in per-unit (already
/// normalised on the system kVA base).  This function simply unpacks them
/// into a flat 3n vector with ordering [bus0_a, bus0_b, bus0_c, bus1_a, …].
fn scheduled_injections(network: &ThreePhaseNetwork) -> (Vec<f64>, Vec<f64>) {
    let n = network.n_buses();
    let mut p = vec![0.0f64; 3 * n];
    let mut q = vec![0.0f64; 3 * n];
    for (bus_idx, bus) in network.buses.iter().enumerate() {
        for ph in 0..3 {
            // p_inj / q_inj are already in per-unit on network.base_kva
            p[3 * bus_idx + ph] = bus.p_inj[ph];
            q[3 * bus_idx + ph] = bus.q_inj[ph];
        }
    }
    (p, q)
}

/// Compute calculated power injections from the current voltage vector and Y-bus.
///
/// S_i = V_i · conj(sum_j Y_ij · V_j)
fn calculate_power_3ph(ybus: &[Vec<Complex64>], voltages: &[Complex64]) -> (Vec<f64>, Vec<f64>) {
    let dim = voltages.len();
    let mut p = vec![0.0f64; dim];
    let mut q = vec![0.0f64; dim];
    for i in 0..dim {
        let mut i_inj = Complex64::new(0.0, 0.0);
        for j in 0..dim {
            i_inj += ybus[i][j] * voltages[j];
        }
        let s = voltages[i] * i_inj.conj();
        p[i] = s.re;
        q[i] = s.im;
    }
    (p, q)
}

// ---------------------------------------------------------------------------
// Jacobian assembly for 3-phase NR (rectangular form)
// ---------------------------------------------------------------------------

/// Build the Jacobian for the three-phase NR iteration in rectangular form.
///
/// **State vector**: [Δe_1, …, Δe_n, Δf_1, …, Δf_n] where V_i = e_i + j·f_i,
/// indexing only the free DOFs (non-slack, active-phase entries of the 3n vector).
///
/// **Mismatch vector**: [ΔP_1, …, ΔP_n, ΔQ_1, …, ΔQ_n]
///
/// **Jacobian blocks**:
/// ```text
///     [∂P/∂e   ∂P/∂f]
/// J = [               ]
///     [∂Q/∂e   ∂Q/∂f]
/// ```
///
/// Analytic derivatives (i ≠ j):
/// ```text
///   ∂P_i/∂e_j = G_ij·e_i + B_ij·f_i    (off-diag)
///   ∂P_i/∂f_j = G_ij·f_i - B_ij·e_i    (off-diag)
///   ∂Q_i/∂e_j = G_ij·f_i - B_ij·e_i    (off-diag)
///   ∂Q_i/∂f_j = -G_ij·e_i - B_ij·f_i   (off-diag)
///
///   ∂P_i/∂e_i = G_ii·e_i - B_ii·f_i + P_i/e_i  ... simplified via current injection:
///   ∂P_i/∂e_i = I_Re_i + G_ii·e_i + B_ii·f_i   (where I_Re = Re(I_inj_i))
///   ∂P_i/∂f_i = -I_Im_i - B_ii·e_i + G_ii·f_i
///   ∂Q_i/∂e_i = I_Im_i + G_ii·f_i - B_ii·e_i
///   ∂Q_i/∂f_i = I_Re_i - G_ii·e_i - B_ii·f_i
/// ```
///
/// The derivation uses `P_i = e_i·I_Re + f_i·I_Im` and `Q_i = f_i·I_Re - e_i·I_Im`.
fn build_jacobian_3ph(
    ybus: &[Vec<Complex64>],
    voltages: &[Complex64],
    free_dof: &[usize],
) -> Vec<Vec<f64>> {
    let ndof = free_dof.len();
    let dim = voltages.len();

    // Compute current injections I = Y·V
    let mut i_inj = vec![Complex64::new(0.0, 0.0); dim];
    for i in 0..dim {
        for j in 0..dim {
            i_inj[i] += ybus[i][j] * voltages[j];
        }
    }

    let mut jac = vec![vec![0.0f64; 2 * ndof]; 2 * ndof];

    for (r, &ri) in free_dof.iter().enumerate() {
        let ei = voltages[ri].re;
        let fi = voltages[ri].im;
        let i_re = i_inj[ri].re;
        let i_im = i_inj[ri].im;
        let g_ii = ybus[ri][ri].re;
        let b_ii = ybus[ri][ri].im;

        // ── Diagonal blocks ─────────────────────────────────────────────────
        // P_i = e_i·I_re_i + f_i·I_im_i
        // Q_i = f_i·I_re_i - e_i·I_im_i
        // where I_re_i = Σ(G_ij·e_j - B_ij·f_j),  I_im_i = Σ(B_ij·e_j + G_ij·f_j)
        //
        // ∂I_re_i/∂e_i = G_ii,  ∂I_re_i/∂f_i = -B_ii
        // ∂I_im_i/∂e_i = B_ii,  ∂I_im_i/∂f_i =  G_ii
        //
        // ∂P_i/∂e_i = I_re_i + e_i·G_ii + f_i·B_ii
        jac[r][r] = i_re + g_ii * ei + b_ii * fi;
        // ∂P_i/∂f_i = I_im_i - e_i·B_ii + f_i·G_ii
        jac[r][ndof + r] = i_im - b_ii * ei + g_ii * fi;
        // ∂Q_i/∂e_i = f_i·G_ii - I_im_i - e_i·B_ii
        jac[ndof + r][r] = fi * g_ii - i_im - ei * b_ii;
        // ∂Q_i/∂f_i = I_re_i - f_i·B_ii - e_i·G_ii
        jac[ndof + r][ndof + r] = i_re - fi * b_ii - ei * g_ii;

        for (c, &ci) in free_dof.iter().enumerate() {
            if r == c {
                continue;
            }
            let g_ij = ybus[ri][ci].re;
            let b_ij = ybus[ri][ci].im;

            // Off-diagonal (j ≠ i): only the self-term of Y contributes extra.
            // ∂P_i/∂e_j = e_i·G_ij + f_i·B_ij
            jac[r][c] = ei * g_ij + fi * b_ij;
            // ∂P_i/∂f_j = f_i·G_ij - e_i·B_ij
            jac[r][ndof + c] = fi * g_ij - ei * b_ij;
            // ∂Q_i/∂e_j = f_i·G_ij - e_i·B_ij  (same as ∂P_i/∂f_j)
            jac[ndof + r][c] = fi * g_ij - ei * b_ij;
            // ∂Q_i/∂f_j = -(e_i·G_ij + f_i·B_ij)  (negation of ∂P_i/∂e_j)
            jac[ndof + r][ndof + c] = -(ei * g_ij + fi * b_ij);
        }
    }

    jac
}

// ---------------------------------------------------------------------------
// Linear solve for the Jacobian system (dense, via oxiblas-lapack)
// ---------------------------------------------------------------------------

/// Solve `J·Δx = b` using oxiblas-lapack dense LU.
fn solve_dense_lu(jac: &[Vec<f64>], rhs: &[f64]) -> Result<Vec<f64>> {
    let n = rhs.len();
    if n == 0 {
        return Ok(vec![]);
    }

    // Flatten row-major into a single slice for oxiblas_matrix::Mat
    let mut flat = Vec::with_capacity(n * n);
    for row in jac.iter().take(n) {
        flat.extend_from_slice(&row[..n]);
    }
    let a = Mat::<f64>::from_slice(n, n, &flat);
    let b = Mat::<f64>::from_slice(n, 1, rhs);

    let lu = Lu::compute(a.as_ref())
        .map_err(|e| OxiGridError::LinearAlgebra(format!("LU compute: {}", e)))?;
    let x = lu
        .solve(b.as_ref())
        .map_err(|e| OxiGridError::LinearAlgebra(format!("LU solve: {}", e)))?;

    Ok((0..n).map(|i| x[(i, 0)]).collect())
}

// ---------------------------------------------------------------------------
// Voltage unbalance factor
// ---------------------------------------------------------------------------

/// Compute the voltage unbalance factor (VUF) for three-phase voltages.
///
/// VUF [%] = |V_negative| / |V_positive| × 100
///
/// where the positive- and negative-sequence components are extracted using
/// the symmetrical-components transformation:
///
/// ```text
/// V₊ = (Va + a·Vb + a²·Vc) / 3       a = exp(j·2π/3)
/// V₋ = (Va + a²·Vb + a·Vc) / 3
/// ```
pub fn compute_vuf(voltages: &[Complex64; 3]) -> f64 {
    let a = fortescue_a(); // exp(j·2π/3)
    let a2 = a * a;

    let v_pos = (voltages[0] + a * voltages[1] + a2 * voltages[2]) / 3.0;
    let v_neg = (voltages[0] + a2 * voltages[1] + a * voltages[2]) / 3.0;

    let v_pos_mag = v_pos.norm();
    if v_pos_mag < 1e-12 {
        return 0.0;
    }
    (v_neg.norm() / v_pos_mag) * 100.0
}

// ---------------------------------------------------------------------------
// Branch-flow results
// ---------------------------------------------------------------------------

/// Per-phase power-flow results for a single three-phase branch.
#[derive(Debug, Clone)]
pub struct ThreePhaseBranchFlow {
    /// Index into `ThreePhaseNetwork::branches`.
    pub branch_idx: usize,
    /// Apparent power at the from-end per phase \[kVA\] (complex).
    pub s_from: [Complex64; 3],
    /// Apparent power at the to-end per phase \[kVA\] (complex).
    pub s_to: [Complex64; 3],
    /// Branch current per phase \[A\] (complex, if base kV is known).
    pub current: [Complex64; 3],
    /// Maximum phase loading as a percentage of `rating_kva`.
    pub loading_pct: f64,
}

// ---------------------------------------------------------------------------
// Solver and result types
// ---------------------------------------------------------------------------

/// Configuration for the three-phase unbalanced Newton-Raphson solver.
#[derive(Debug, Clone)]
pub struct UnbalancedPowerFlow {
    /// Maximum number of Newton-Raphson iterations.
    pub max_iter: usize,
    /// Convergence tolerance on the power mismatch [p.u.].
    pub tolerance: f64,
    /// System base line-to-line voltage \[kV\].
    pub base_kv: f64,
    /// System base three-phase power \[kVA\].
    pub base_kva: f64,
}

impl Default for UnbalancedPowerFlow {
    fn default() -> Self {
        Self {
            max_iter: 50,
            tolerance: 1e-6,
            base_kv: 4.16,
            base_kva: 1000.0,
        }
    }
}

/// Results from a three-phase unbalanced power flow solve.
#[derive(Debug, Clone)]
pub struct UnbalancedResult {
    /// Final complex voltages per bus and phase [p.u.].
    pub voltages: Vec<[Complex64; 3]>,
    /// Branch power flows.
    pub power_flows: Vec<ThreePhaseBranchFlow>,
    /// Total real-power losses per phase \[MW\] (A, B, C).
    pub losses: [f64; 3],
    /// Voltage unbalance factor (VUF) per bus [%].
    pub voltage_unbalance: Vec<f64>,
    /// Whether the solver converged.
    pub converged: bool,
    /// Number of NR iterations performed.
    pub n_iterations: usize,
    /// Maximum power mismatch at the final iteration [p.u.].
    pub max_mismatch: f64,
}

impl UnbalancedPowerFlow {
    /// Solve the three-phase unbalanced power flow for the given network.
    ///
    /// The algorithm:
    /// 1. Build the 3n×3n phase-frame Y-bus.
    /// 2. Initialise voltages from bus initial values.
    /// 3. Newton-Raphson iteration until convergence or max_iter.
    /// 4. Compute branch flows, losses, and VUF.
    pub fn solve(&self, network: &ThreePhaseNetwork) -> Result<UnbalancedResult> {
        let n = network.n_buses();
        if n == 0 {
            return Err(OxiGridError::InvalidNetwork(
                "Network has no buses".to_string(),
            ));
        }

        // ── 1. Build Y-bus ──────────────────────────────────────────────────
        let ybus = build_three_phase_ybus(network)?;

        // ── 2. Initialise voltage vector (length 3n) ────────────────────────
        let mut voltages: Vec<Complex64> = network
            .buses
            .iter()
            .flat_map(|bus| bus.v.iter().copied())
            .collect();

        // ── 3. Scheduled injections (per-unit on system base) ───────────────
        let (p_sched, q_sched) = scheduled_injections(network);

        // ── 4. Identify free DOFs (exclude all phases of the slack bus) ──────
        // DOF indexing: row/col in the 3n system maps to bus·3 + phase.
        let free_dof: Vec<usize> = (0..n)
            .flat_map(|bus_idx| {
                let bus = &network.buses[bus_idx];
                (0..3_usize).filter_map(move |ph| {
                    if bus.bus_type == BusType::Slack || !bus.phases[ph] {
                        None
                    } else {
                        Some(bus_idx * 3 + ph)
                    }
                })
            })
            .collect();

        let ndof = free_dof.len();
        let mut converged = false;
        let mut n_iterations = 0_usize;
        let mut max_mismatch = f64::MAX;

        // ── 5. Newton-Raphson loop ───────────────────────────────────────────
        for _iter in 0..self.max_iter {
            let (p_calc, q_calc) = calculate_power_3ph(&ybus, &voltages);

            // Mismatch vector [ΔP_free..., ΔQ_free...]
            let mut mismatch = Vec::with_capacity(2 * ndof);
            for &dof in &free_dof {
                mismatch.push(p_sched[dof] - p_calc[dof]);
            }
            for &dof in &free_dof {
                mismatch.push(q_sched[dof] - q_calc[dof]);
            }

            max_mismatch = mismatch.iter().map(|x| x.abs()).fold(0.0_f64, f64::max);

            if max_mismatch < self.tolerance {
                converged = true;
                break;
            }

            // Build Jacobian
            let jac = build_jacobian_3ph(&ybus, &voltages, &free_dof);

            // Solve J·Δx = mismatch (where mismatch = f(x) = P_sched - P_calc)
            // The NR update is x_{k+1} = x_k + α·Δx, where α ∈ (0,1] is found
            // via backtracking line-search to guarantee ‖f(x+αΔx)‖ < ‖f(x)‖.
            let dx = solve_dense_lu(&jac, &mismatch)?;

            // ── Backtracking line search (Armijo condition) ───────────────────
            let mut alpha = 1.0_f64;
            let norm_f = max_mismatch;

            'line_search: for _ in 0..8 {
                // Trial update
                let mut v_trial = voltages.clone();
                for (k, &dof) in free_dof.iter().enumerate() {
                    let de = alpha * dx[k];
                    let df = alpha * dx[ndof + k];
                    let new_v = Complex64::new(v_trial[dof].re + de, v_trial[dof].im + df);
                    v_trial[dof] = if new_v.norm() < 0.5 {
                        Complex64::from_polar(0.5, new_v.arg())
                    } else {
                        new_v
                    };
                }
                let (p_trial, q_trial) = calculate_power_3ph(&ybus, &v_trial);
                let norm_f_trial = free_dof
                    .iter()
                    .map(|&dof| {
                        let dp = (p_sched[dof] - p_trial[dof]).abs();
                        let dq = (q_sched[dof] - q_trial[dof]).abs();
                        dp.max(dq)
                    })
                    .fold(0.0_f64, f64::max);

                if norm_f_trial < norm_f {
                    // Accept this step
                    voltages = v_trial;
                    break 'line_search;
                }
                alpha *= 0.5;
            }
            // If no improvement found, take the smallest step anyway
            if alpha < 1.0 / 256.0 {
                for (k, &dof) in free_dof.iter().enumerate() {
                    let de = alpha * dx[k];
                    let df = alpha * dx[ndof + k];
                    let new_v = Complex64::new(voltages[dof].re + de, voltages[dof].im + df);
                    voltages[dof] = if new_v.norm() < 0.5 {
                        Complex64::from_polar(0.5, new_v.arg())
                    } else {
                        new_v
                    };
                }
            }

            n_iterations += 1;
        }

        // ── 6. Reconstruct per-bus voltage arrays ────────────────────────────
        let bus_voltages: Vec<[Complex64; 3]> = (0..n)
            .map(|i| [voltages[3 * i], voltages[3 * i + 1], voltages[3 * i + 2]])
            .collect();

        // ── 7. Compute branch flows ──────────────────────────────────────────
        let base_mva = network.base_kva / 1000.0;
        let power_flows = compute_three_phase_branch_flows(network, &voltages, base_mva);

        // ── 8. Losses per phase ──────────────────────────────────────────────
        // Total injected power sums to zero in lossless assumption; losses =
        // sum of branch (s_from + s_to) per phase.
        let mut losses = [0.0f64; 3];
        for flow in &power_flows {
            for (ph, loss) in losses.iter_mut().enumerate() {
                // s_from + s_to = loss (MW) on this branch
                *loss += (flow.s_from[ph] + flow.s_to[ph]).re;
            }
        }

        // ── 9. VUF per bus ───────────────────────────────────────────────────
        let voltage_unbalance: Vec<f64> = bus_voltages.iter().map(compute_vuf).collect();

        Ok(UnbalancedResult {
            voltages: bus_voltages,
            power_flows,
            losses,
            voltage_unbalance,
            converged,
            n_iterations,
            max_mismatch,
        })
    }
}

// ---------------------------------------------------------------------------
// Branch flow computation
// ---------------------------------------------------------------------------

fn compute_three_phase_branch_flows(
    network: &ThreePhaseNetwork,
    voltages: &[Complex64],
    base_mva: f64,
) -> Vec<ThreePhaseBranchFlow> {
    let base_kva = base_mva * 1000.0;
    let mut flows = Vec::with_capacity(network.branches.len());

    for (branch_idx, branch) in network.branches.iter().enumerate() {
        let i = branch.from_bus;
        let j = branch.to_bus;

        // Series admittance matrix
        let y_s = match invert_3x3(&branch.z_series) {
            Ok(y) => y,
            Err(_) => {
                // Push a zeroed flow entry and continue
                flows.push(ThreePhaseBranchFlow {
                    branch_idx,
                    s_from: [Complex64::new(0.0, 0.0); 3],
                    s_to: [Complex64::new(0.0, 0.0); 3],
                    current: [Complex64::new(0.0, 0.0); 3],
                    loading_pct: 0.0,
                });
                continue;
            }
        };

        let mut s_from = [Complex64::new(0.0, 0.0); 3];
        let mut s_to = [Complex64::new(0.0, 0.0); 3];
        let mut current = [Complex64::new(0.0, 0.0); 3];

        for p in 0..3 {
            if !branch.phases[p] {
                continue;
            }

            let vi_p = voltages[3 * i + p];
            let vj_p = voltages[3 * j + p];

            // Phase-frame current: I_from_p = Σ_q Y_s[p][q] · (V_i_q - V_j_q)
            let mut i_from_p = Complex64::new(0.0, 0.0);
            let mut i_to_p = Complex64::new(0.0, 0.0);
            for q in 0..3 {
                if branch.phases[q] {
                    let dv = voltages[3 * i + q] - voltages[3 * j + q];
                    i_from_p += y_s[p][q] * dv;
                    // Shunt at from end
                    i_from_p += branch.y_shunt[p][q] * 0.5 * voltages[3 * i + q];
                }
            }
            for q in 0..3 {
                if branch.phases[q] {
                    let dv = voltages[3 * j + q] - voltages[3 * i + q];
                    i_to_p += y_s[p][q] * dv;
                    // Shunt at to end
                    i_to_p += branch.y_shunt[p][q] * 0.5 * voltages[3 * j + q];
                }
            }

            current[p] = i_from_p;
            // Apparent power in p.u., then scale to kVA
            s_from[p] = vi_p * i_from_p.conj() * base_kva;
            s_to[p] = vj_p * i_to_p.conj() * base_kva;
        }

        // Loading: max phase |s_from| / rating
        let max_s = s_from.iter().map(|s| s.norm()).fold(0.0_f64, f64::max);
        let loading_pct = if branch.rating_kva > 0.0 {
            max_s / branch.rating_kva * 100.0
        } else {
            0.0
        };

        flows.push(ThreePhaseBranchFlow {
            branch_idx,
            s_from,
            s_to,
            current,
            loading_pct,
        });
    }

    flows
}

// ---------------------------------------------------------------------------
// IEEE 13-bus distribution test feeder
// ---------------------------------------------------------------------------

/// Construct the IEEE 13-bus unbalanced distribution test feeder.
///
/// This is the canonical test case from the IEEE PES Distribution Test
/// Feeder Working Group.  Bus 650 is the slack (substation LV bus).
/// All impedance values are converted to the system per-unit base.
///
/// Topology (from IEEE 13-bus specification):
/// ```text
/// 650 ── 632 ── 671 ── 692 ── 675
///         |      |      |
///        633    684    611
///         |      |
///        634    652
///         |
///        645 ── 646
/// ```
///
/// Line configurations from the IEEE working group (ohms/mile → system pu):
///
/// Config 601 (3-phase), 602 (3-phase), 603/604 (2-phase BC), 605 (1-phase C).
pub fn ieee13_bus_feeder() -> ThreePhaseNetwork {
    // Bus IDs as used in the IEEE reference:
    // 650, 632, 633, 634, 645, 646, 671, 692, 675, 684, 611, 652
    // We assign internal indices 0..11

    // Base: 4.16 kV (line-to-line), 1000 kVA three-phase
    // Z_base = V_base² / S_base = (4.16e3)² / (1000e3) = 17.3056 Ω
    let base_kv: f64 = 4.16;
    let base_kva: f64 = 1000.0;
    let z_base = (base_kv * 1000.0).powi(2) / (base_kva * 1000.0); // Ω per phase

    // ── Bus definitions ──────────────────────────────────────────────────────
    // Bus 650 (slack at 1.0 p.u.), others flat-start
    //  idx  id   name
    //  0   650  slack
    //  1   632
    //  2   633
    //  3   634
    //  4   645
    //  5   646
    //  6   671
    //  7   692
    //  8   675
    //  9   684
    // 10   611  phase C only
    // 11   652  phase A only

    let mut buses: Vec<ThreePhaseBus> = Vec::new();

    // Bus 650 — slack, 3-phase, 1.0 p.u.
    buses.push(ThreePhaseBus::new_slack(650, "650", 1.0));

    // PQ load buses — all initially 3-phase unless noted
    let pq_ids = [632, 633, 634, 645, 646, 671, 692, 675, 684, 611, 652];
    for &id in &pq_ids {
        buses.push(ThreePhaseBus::new_pq(id, id.to_string()));
    }

    // Bus 4 (645): phases B and C only (branch 632→645 is config 603 two-phase BC)
    buses[4].phases = [false, true, true];
    buses[4].v[0] = Complex64::new(0.0, 0.0);

    // Bus 5 (646): phases B and C only (branch 645→646 is config 603 two-phase BC)
    buses[5].phases = [false, true, true];
    buses[5].v[0] = Complex64::new(0.0, 0.0);

    // Bus 9 (684): phases A and C only (connected to 671 via config 604)
    buses[9].phases = [true, false, true];
    buses[9].v[1] = Complex64::new(0.0, 0.0);

    // Bus 10 (611): phase C only
    buses[10].phases = [false, false, true];
    buses[10].v[0] = Complex64::new(0.0, 0.0);
    buses[10].v[1] = Complex64::new(0.0, 0.0);

    // Bus 11 (652): phase A only
    buses[11].phases = [true, false, false];
    buses[11].v[1] = Complex64::new(0.0, 0.0);
    buses[11].v[2] = Complex64::new(0.0, 0.0);

    // ── Load injections (kW, kVAr per phase, sign convention: load is negative injection) ──
    // Values from IEEE 13-bus reference (spot loads in kW+jkVAr)
    // Divided by base_kva/3 per phase for per-unit conversion.
    // All loads are negative net injections (load consumes power).

    let to_pu = |kw: f64| -> f64 { -kw / base_kva }; // 3-phase base

    // Bus 634 (idx 3): 3-phase balanced load 160+j110 kVA total
    buses[3].p_inj = [to_pu(160.0 / 3.0); 3];
    buses[3].q_inj = [to_pu(110.0 / 3.0); 3];

    // Bus 645 (idx 4): phase B 170+j125 kVAr
    buses[4].p_inj = [0.0, to_pu(170.0), 0.0];
    buses[4].q_inj = [0.0, to_pu(125.0), 0.0];

    // Bus 646 (idx 5): phase B 230+j132 kVAr
    buses[5].p_inj = [0.0, to_pu(230.0), 0.0];
    buses[5].q_inj = [0.0, to_pu(132.0), 0.0];

    // Bus 671 (idx 6): 3-phase unbalanced (385+j220, 385+j220, 385+j220) kW
    buses[6].p_inj = [to_pu(385.0); 3];
    buses[6].q_inj = [to_pu(220.0); 3];

    // Bus 675 (idx 8): unbalanced (485+j190, 68+j60, 290+j212) kW
    buses[8].p_inj = [to_pu(485.0), to_pu(68.0), to_pu(290.0)];
    buses[8].q_inj = [to_pu(190.0), to_pu(60.0), to_pu(212.0)];

    // Bus 692 (idx 7): phase C 170+j151 kVAr
    buses[7].p_inj = [0.0, 0.0, to_pu(170.0)];
    buses[7].q_inj = [0.0, 0.0, to_pu(151.0)];

    // Bus 611 (idx 10): phase C 170+j80 kVAr
    buses[10].p_inj = [0.0, 0.0, to_pu(170.0)];
    buses[10].q_inj = [0.0, 0.0, to_pu(80.0)];

    // Bus 652 (idx 11): phase A 128+j86 kVAr
    buses[11].p_inj = [to_pu(128.0), 0.0, 0.0];
    buses[11].q_inj = [to_pu(86.0), 0.0, 0.0];

    // ── Line configuration matrices (ohms/mile × miles, then /z_base for p.u.) ──
    // IEEE 13-bus feeder uses overhead line configurations.
    // Config 601: 3-phase (typical Kersting values, ohms/mile)
    let r601_mi = Complex64::new(0.3465, 0.0);
    let x601_mi = Complex64::new(0.0, 1.0179);
    let r601_mut = Complex64::new(0.1560, 0.0);
    let x601_mut = Complex64::new(0.0, 0.5017);
    let z601_diag = r601_mi + x601_mi;
    let z601_off = r601_mut + x601_mut;

    // Config 602: 3-phase
    let z602_diag = Complex64::new(0.7526, 1.1814);
    let z602_off = Complex64::new(0.1580, 0.4236);

    // Config 603: 2-phase B-C
    let z603_bc = Complex64::new(1.3294, 1.3471);
    let z603_off = Complex64::new(0.2066, 0.4591);

    // Config 604: 2-phase A-C
    let z604_ac = Complex64::new(1.3238, 1.3569);
    let z604_off = Complex64::new(0.2091, 0.4591);

    // Config 605: 1-phase C
    let z605_c = Complex64::new(1.3292, 1.3475);

    // Helper to make a per-unit impedance matrix (ohms/mile × miles / z_base)
    let z_pu = |z_ohm: Complex64, miles: f64| -> Complex64 { z_ohm * miles / z_base };

    // ── Branch list ──────────────────────────────────────────────────────────
    // (from_idx, to_idx, config, miles)
    let mut branches: Vec<ThreePhaseBranch> = Vec::new();

    // 650→632: config 601, 2000 ft = 0.3788 mi, 3-phase
    {
        let miles = 2000.0 / 5280.0;
        let zd = z_pu(z601_diag, miles);
        let zo = z_pu(z601_off, miles);
        let mut b = ThreePhaseBranch::three_phase_symmetric(0, 1, zd, zo, miles * 1.609, 1000.0);
        // Adjust: correct diagonal includes both R and X
        b.z_series = [[zd, zo, zo], [zo, zd, zo], [zo, zo, zd]];
        branches.push(b);
    }

    // 632→671: config 601, 2000 ft
    {
        let miles = 2000.0 / 5280.0;
        let zd = z_pu(z601_diag, miles);
        let zo = z_pu(z601_off, miles);
        let b = ThreePhaseBranch {
            from_bus: 1,
            to_bus: 6,
            z_series: [[zd, zo, zo], [zo, zd, zo], [zo, zo, zd]],
            y_shunt: [[Complex64::new(0.0, 0.0); 3]; 3],
            length_km: miles * 1.609,
            rating_kva: 1000.0,
            phases: [true; 3],
        };
        branches.push(b);
    }

    // 632→633: config 602, 500 ft
    {
        let miles = 500.0 / 5280.0;
        let zd = z_pu(z602_diag, miles);
        let zo = z_pu(z602_off, miles);
        let b = ThreePhaseBranch {
            from_bus: 1,
            to_bus: 2,
            z_series: [[zd, zo, zo], [zo, zd, zo], [zo, zo, zd]],
            y_shunt: [[Complex64::new(0.0, 0.0); 3]; 3],
            length_km: miles * 1.609,
            rating_kva: 500.0,
            phases: [true; 3],
        };
        branches.push(b);
    }

    // 671→692: config 601, short section (switch — approx 0 length)
    // In the IEEE feeder 692 is essentially the same bus as 671 (a switch).
    // We use a very short line to avoid division by zero.
    {
        let miles = 0.001;
        let zd = z_pu(z601_diag, miles);
        let zo = z_pu(z601_off, miles);
        let b = ThreePhaseBranch {
            from_bus: 6,
            to_bus: 7,
            z_series: [[zd, zo, zo], [zo, zd, zo], [zo, zo, zd]],
            y_shunt: [[Complex64::new(0.0, 0.0); 3]; 3],
            length_km: miles * 1.609,
            rating_kva: 1000.0,
            phases: [true; 3],
        };
        branches.push(b);
    }

    // 692→675: config 606, 800 ft  (use 601 approximation for 3-phase)
    {
        let miles = 800.0 / 5280.0;
        let zd = z_pu(z601_diag, miles);
        let zo = z_pu(z601_off, miles);
        let b = ThreePhaseBranch {
            from_bus: 7,
            to_bus: 8,
            z_series: [[zd, zo, zo], [zo, zd, zo], [zo, zo, zd]],
            y_shunt: [[Complex64::new(0.0, 0.0); 3]; 3],
            length_km: miles * 1.609,
            rating_kva: 500.0,
            phases: [true; 3],
        };
        branches.push(b);
    }

    // 645→646: config 603, 300 ft, phases B-C
    {
        let miles = 300.0 / 5280.0;
        let z_bc = z_pu(z603_bc, miles);
        let z_off = z_pu(z603_off, miles);
        let zero = Complex64::new(0.0, 0.0);
        let b = ThreePhaseBranch {
            from_bus: 4,
            to_bus: 5,
            z_series: [
                [Complex64::new(1.0, 0.0), zero, zero], // phase A not present; use 1.0 to avoid singularity
                [zero, z_bc, z_off],
                [zero, z_off, z_bc],
            ],
            y_shunt: [[zero; 3]; 3],
            length_km: miles * 1.609,
            rating_kva: 500.0,
            phases: [false, true, true],
        };
        branches.push(b);
    }

    // 632→645: config 603, 500 ft, phases B-C
    {
        let miles = 500.0 / 5280.0;
        let z_bc = z_pu(z603_bc, miles);
        let z_off = z_pu(z603_off, miles);
        let zero = Complex64::new(0.0, 0.0);
        let b = ThreePhaseBranch {
            from_bus: 1,
            to_bus: 4,
            z_series: [
                [Complex64::new(1.0, 0.0), zero, zero],
                [zero, z_bc, z_off],
                [zero, z_off, z_bc],
            ],
            y_shunt: [[zero; 3]; 3],
            length_km: miles * 1.609,
            rating_kva: 500.0,
            phases: [false, true, true],
        };
        branches.push(b);
    }

    // 671→684: config 604, 300 ft, phases A-C
    {
        let miles = 300.0 / 5280.0;
        let z_ac = z_pu(z604_ac, miles);
        let z_off = z_pu(z604_off, miles);
        let zero = Complex64::new(0.0, 0.0);
        let b = ThreePhaseBranch {
            from_bus: 6,
            to_bus: 9,
            z_series: [
                [z_ac, zero, z_off],
                [zero, Complex64::new(1.0, 0.0), zero],
                [z_off, zero, z_ac],
            ],
            y_shunt: [[zero; 3]; 3],
            length_km: miles * 1.609,
            rating_kva: 500.0,
            phases: [true, false, true],
        };
        branches.push(b);
    }

    // 684→611: config 605, 300 ft, phase C only
    {
        let miles = 300.0 / 5280.0;
        let z_c = z_pu(z605_c, miles);
        let zero = Complex64::new(0.0, 0.0);
        let b = ThreePhaseBranch {
            from_bus: 9,
            to_bus: 10,
            z_series: [
                [Complex64::new(1.0, 0.0), zero, zero],
                [zero, Complex64::new(1.0, 0.0), zero],
                [zero, zero, z_c],
            ],
            y_shunt: [[zero; 3]; 3],
            length_km: miles * 1.609,
            rating_kva: 300.0,
            phases: [false, false, true],
        };
        branches.push(b);
    }

    // 684→652: config 607, 800 ft, phase A only (use z604 A diagonal)
    {
        let miles = 800.0 / 5280.0;
        let z_a = z_pu(z604_ac, miles);
        let zero = Complex64::new(0.0, 0.0);
        let b = ThreePhaseBranch {
            from_bus: 9,
            to_bus: 11,
            z_series: [
                [z_a, zero, zero],
                [zero, Complex64::new(1.0, 0.0), zero],
                [zero, zero, Complex64::new(1.0, 0.0)],
            ],
            y_shunt: [[zero; 3]; 3],
            length_km: miles * 1.609,
            rating_kva: 300.0,
            phases: [true, false, false],
        };
        branches.push(b);
    }

    // ── Transformer: 633→634 (Wye-Wye step-down, 500 kVA, 4.16kV / 0.48kV) ─
    // In per-unit: z_pu = 0.01 + j0.08 on transformer base, same pu on system
    let transformer = ThreePhaseTransformer {
        from_bus: 2, // 633
        to_bus: 3,   // 634
        connection: TransformerConnection::WyeWye,
        kva_rating: 500.0,
        v_primary_kv: 4.16,
        v_secondary_kv: 0.48,
        z_pu: Complex64::new(0.01, 0.08),
        tap_ratio: 1.0,
    };

    ThreePhaseNetwork {
        buses,
        branches,
        transformers: vec![transformer],
        base_kv,
        base_kva,
        slack_bus: 0,
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_three_phase_ybus_dimensions() {
        let net = ieee13_bus_feeder();
        let ybus = build_three_phase_ybus(&net).expect("Y-bus build should succeed");
        let expected = 3 * net.buses.len();
        assert_eq!(ybus.len(), expected, "Y-bus row count mismatch");
        for row in &ybus {
            assert_eq!(row.len(), expected, "Y-bus column count mismatch");
        }
    }

    #[test]
    fn test_three_phase_balanced_equals_single_phase() {
        // Balanced 2-bus three-phase network: results should show very small VUF
        let n_buses = 2;
        let base_kv = 11.0;
        let base_kva = 10_000.0;
        let z_base = (base_kv * 1e3_f64).powi(2) / (base_kva * 1e3_f64);

        // Symmetric line: z = (0.5 + j1.0) Ω, mutual = (0.1 + j0.3) Ω
        let z_d = Complex64::new(0.5, 1.0) / z_base;
        let z_m = Complex64::new(0.1, 0.3) / z_base;

        let branch = ThreePhaseBranch {
            from_bus: 0,
            to_bus: 1,
            z_series: [[z_d, z_m, z_m], [z_m, z_d, z_m], [z_m, z_m, z_d]],
            y_shunt: [[Complex64::new(0.0, 0.0); 3]; 3],
            length_km: 1.0,
            rating_kva: 10_000.0,
            phases: [true; 3],
        };

        // Balanced load: 1000 kW + j500 kVAr per phase
        let p_pu = -1000.0 / base_kva;
        let q_pu = -500.0 / base_kva;

        let mut load_bus = ThreePhaseBus::new_pq(2, "Load");
        load_bus.p_inj = [p_pu; 3];
        load_bus.q_inj = [q_pu; 3];

        let net = ThreePhaseNetwork {
            buses: vec![ThreePhaseBus::new_slack(1, "Slack", 1.0), load_bus],
            branches: vec![branch],
            transformers: vec![],
            base_kv,
            base_kva,
            slack_bus: 0,
        };

        let solver = UnbalancedPowerFlow {
            max_iter: 50,
            tolerance: 1e-8,
            base_kv,
            base_kva,
        };

        let result = solver
            .solve(&net)
            .expect("Balanced 2-bus solve should succeed");
        assert!(
            result.converged,
            "Balanced 2-bus did not converge (iter={}, mismatch={:.2e})",
            result.n_iterations, result.max_mismatch
        );

        // VUF should be very small for a balanced network
        let vuf_load = result.voltage_unbalance[1];
        assert!(
            vuf_load < 1.0,
            "VUF {:.4}% too large for balanced load bus",
            vuf_load
        );

        // All three phase magnitudes at load bus should be within 1% of each other
        let v_load = &result.voltages[1];
        let mags: Vec<f64> = v_load.iter().map(|v| v.norm()).collect();
        let mag_max = mags.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let mag_min = mags.iter().cloned().fold(f64::INFINITY, f64::min);
        assert!(
            (mag_max - mag_min) < 0.01,
            "Phase voltage magnitudes diverge: {:?}",
            mags
        );

        let _ = n_buses; // suppress unused variable warning
    }

    #[test]
    fn test_ieee13_bus_convergence() {
        let net = ieee13_bus_feeder();
        let pf = UnbalancedPowerFlow {
            max_iter: 200,
            tolerance: 1e-6,
            base_kv: 4.16,
            base_kva: 1000.0,
        };
        let result = pf.solve(&net);
        assert!(result.is_ok(), "solve() returned Err: {:?}", result.err());
        let r = result.expect("solve should succeed");
        assert!(
            r.converged,
            "IEEE 13-bus did not converge (iter={}, mismatch={:.2e})",
            r.n_iterations, r.max_mismatch
        );
    }

    #[test]
    fn test_vuf_balanced() {
        let a = Complex64::new(1.0, 0.0);
        let b = Complex64::from_polar(1.0, -2.0 * PI / 3.0);
        let c = Complex64::from_polar(1.0, 2.0 * PI / 3.0);
        let vuf = compute_vuf(&[a, b, c]);
        assert!(
            vuf < 0.1,
            "VUF for balanced voltages should be ~0, got {:.4}%",
            vuf
        );
    }

    #[test]
    fn test_vuf_unbalanced() {
        // Heavily unbalanced: phase A at 1.0, B and C at 0.8
        let a = Complex64::new(1.0, 0.0);
        let b = Complex64::from_polar(0.8, -2.0 * PI / 3.0);
        let c = Complex64::from_polar(0.8, 2.0 * PI / 3.0);
        let vuf = compute_vuf(&[a, b, c]);
        assert!(
            vuf > 0.1,
            "VUF for unbalanced voltages should be > 0.1%, got {:.4}%",
            vuf
        );
    }

    #[test]
    fn test_transformer_wye_delta_phase_shift() {
        // Wye-Delta transformer should introduce -30° phase shift
        let xfmr = ThreePhaseTransformer {
            from_bus: 0,
            to_bus: 1,
            connection: TransformerConnection::WyeDelta,
            kva_rating: 1000.0,
            v_primary_kv: 4.16,
            v_secondary_kv: 0.48,
            z_pu: Complex64::new(0.01, 0.06),
            tap_ratio: 1.0,
        };

        let phi = xfmr.phase_shift_rad();
        assert!(
            (phi - (-PI / 6.0)).abs() < 1e-10,
            "WyeDelta phase shift should be -30°, got {:.4} rad",
            phi
        );

        // Delta-Wye should be +30°
        let xfmr_dw = ThreePhaseTransformer {
            connection: TransformerConnection::DeltaWye,
            ..xfmr
        };
        let phi_dw = xfmr_dw.phase_shift_rad();
        assert!(
            (phi_dw - (PI / 6.0)).abs() < 1e-10,
            "DeltaWye phase shift should be +30°, got {:.4} rad",
            phi_dw
        );
    }

    #[test]
    fn test_ybus_symmetry() {
        // A network with balanced symmetric impedances should yield a symmetric Y-bus
        let net = ieee13_bus_feeder();
        let ybus = build_three_phase_ybus(&net).expect("Y-bus build");
        let n = ybus.len();
        #[allow(clippy::needless_range_loop)]
        for i in 0..n {
            for j in 0..n {
                let diff = (ybus[i][j] - ybus[j][i]).norm();
                assert!(
                    diff < 1e-10,
                    "Y-bus not symmetric at ({},{}) vs ({},{}): diff={:.2e}",
                    i,
                    j,
                    j,
                    i,
                    diff
                );
            }
        }
    }

    #[test]
    fn test_compute_vuf_zero_for_perfectly_balanced() {
        // Exact Fortescue balanced set
        for v_mag in [0.9, 1.0, 1.05] {
            let va = Complex64::from_polar(v_mag, 0.0);
            let vb = Complex64::from_polar(v_mag, -2.0 * PI / 3.0);
            let vc = Complex64::from_polar(v_mag, 2.0 * PI / 3.0);
            let vuf = compute_vuf(&[va, vb, vc]);
            assert!(
                vuf < 1e-8,
                "VUF={:.2e}% for v_mag={} should be ~0",
                vuf,
                v_mag
            );
        }
    }

    #[test]
    fn test_ieee13_bus_feeder_structure() {
        let net = ieee13_bus_feeder();
        assert_eq!(
            net.buses.len(),
            12,
            "IEEE 13-bus feeder has 12 buses (650 + 11 load buses)"
        );
        assert!(!net.branches.is_empty(), "feeder must have branches");
        assert!(
            !net.transformers.is_empty(),
            "feeder must have at least one transformer"
        );
        assert_eq!(net.slack_bus, 0, "slack bus is index 0 (bus 650)");
        assert_eq!(
            net.buses[0].bus_type,
            BusType::Slack,
            "bus 650 must be slack"
        );
    }

    #[test]
    /// Verify that the analytical Jacobian matches the numerical (finite-difference) Jacobian.
    fn test_jacobian_matches_numerical() {
        let base_kv = 11.0_f64;
        let base_kva = 10_000.0_f64;
        let z_base = (base_kv * 1e3_f64).powi(2) / (base_kva * 1e3_f64);
        let z_d = Complex64::new(0.5, 1.0) / z_base;
        let z_m = Complex64::new(0.1, 0.3) / z_base;
        let branch = ThreePhaseBranch {
            from_bus: 0,
            to_bus: 1,
            z_series: [[z_d, z_m, z_m], [z_m, z_d, z_m], [z_m, z_m, z_d]],
            y_shunt: [[Complex64::new(0.0, 0.0); 3]; 3],
            length_km: 1.0,
            rating_kva: 10_000.0,
            phases: [true; 3],
        };
        let mut load_bus = ThreePhaseBus::new_pq(2, "Load");
        load_bus.p_inj = [-0.1; 3];
        load_bus.q_inj = [-0.05; 3];
        let net = ThreePhaseNetwork {
            buses: vec![ThreePhaseBus::new_slack(1, "Slack", 1.0), load_bus],
            branches: vec![branch],
            transformers: vec![],
            base_kv,
            base_kva,
            slack_bus: 0,
        };

        let ybus = build_three_phase_ybus(&net).expect("Y-bus");
        let voltages: Vec<Complex64> = net.buses.iter().flat_map(|b| b.v.iter().copied()).collect();
        let free_dof: Vec<usize> = vec![3, 4, 5];
        let ndof = free_dof.len();

        let jac = build_jacobian_3ph(&ybus, &voltages, &free_dof);
        let h = 1e-6_f64;

        // Check all Jacobian entries against finite differences
        for (r, &ri) in free_dof.iter().enumerate() {
            for (c, &ci) in free_dof.iter().enumerate() {
                // ∂P_ri/∂e_ci
                let mut vp = voltages.clone();
                vp[ci] += Complex64::new(h, 0.0);
                let mut vm = voltages.clone();
                vm[ci] -= Complex64::new(h, 0.0);
                let (pp, qp) = calculate_power_3ph(&ybus, &vp);
                let (pm, qm) = calculate_power_3ph(&ybus, &vm);
                let dp_de_num = (pp[ri] - pm[ri]) / (2.0 * h);
                let dq_de_num = (qp[ri] - qm[ri]) / (2.0 * h);

                // ∂P_ri/∂f_ci
                let mut vfp = voltages.clone();
                vfp[ci] += Complex64::new(0.0, h);
                let mut vfm = voltages.clone();
                vfm[ci] -= Complex64::new(0.0, h);
                let (pfp, qfp) = calculate_power_3ph(&ybus, &vfp);
                let (pfm, qfm) = calculate_power_3ph(&ybus, &vfm);
                let dp_df_num = (pfp[ri] - pfm[ri]) / (2.0 * h);
                let dq_df_num = (qfp[ri] - qfm[ri]) / (2.0 * h);

                let tol = 1e-4;
                assert!(
                    (jac[r][c] - dp_de_num).abs() < tol,
                    "J[{r}][{c}] dP/de: analytic={:.6} numerical={:.6}",
                    jac[r][c],
                    dp_de_num
                );
                assert!(
                    (jac[r][ndof + c] - dp_df_num).abs() < tol,
                    "J[{r}][{}] dP/df: analytic={:.6} numerical={:.6}",
                    ndof + c,
                    jac[r][ndof + c],
                    dp_df_num
                );
                assert!(
                    (jac[ndof + r][c] - dq_de_num).abs() < tol,
                    "J[{}][{c}] dQ/de: analytic={:.6} numerical={:.6}",
                    ndof + r,
                    jac[ndof + r][c],
                    dq_de_num
                );
                assert!(
                    (jac[ndof + r][ndof + c] - dq_df_num).abs() < tol,
                    "J[{}][{}] dQ/df: analytic={:.6} numerical={:.6}",
                    ndof + r,
                    ndof + c,
                    jac[ndof + r][ndof + c],
                    dq_df_num
                );
            }
        }
    }

    #[test]
    fn test_invert_3x3_identity() {
        let one = Complex64::new(1.0, 0.0);
        let zero = Complex64::new(0.0, 0.0);
        let eye = [[one, zero, zero], [zero, one, zero], [zero, zero, one]];
        let inv = invert_3x3(&eye).expect("identity inverse must succeed");
        #[allow(clippy::needless_range_loop)]
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (inv[i][j].re - expected).abs() < 1e-12,
                    "inv[{i}][{j}].re = {:.2e}, expected {expected}",
                    inv[i][j].re
                );
                assert!(
                    inv[i][j].im.abs() < 1e-12,
                    "inv[{i}][{j}].im = {:.2e}, expected 0",
                    inv[i][j].im
                );
            }
        }
    }
}
