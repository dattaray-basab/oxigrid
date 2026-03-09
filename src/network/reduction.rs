/// Network reduction — Kron reduction and Ward equivalents.
///
/// Kron reduction eliminates interior (load) buses from the Y-bus to produce
/// an equivalent network containing only the retained (generator/boundary) buses.
///
/// # Algorithm
/// Partition the n×n admittance matrix Y into boundary (b) and interior (i) sets:
///
///   Y = [[Ybb, Ybi],
///        [Yib, Yii]]
///
/// Kron-reduced admittance matrix:
///
///   Yr = Ybb − Ybi · Yii⁻¹ · Yib
///
/// The (r×r) reduced matrix gives the equivalent impedance seen from the boundary.
///
/// # Reference
/// Bergen & Vittal, "Power Systems Analysis", 2nd ed., Ch. 11.
use crate::error::{OxiGridError, Result};
use nalgebra::{DMatrix, DVector};
use num_complex::Complex64;
use sprs::CsMat;

/// Kron-reduce a sparse Y-bus, eliminating `interior` bus indices.
///
/// Returns the reduced dense Y-bus over the retained buses.
/// The retained bus ordering follows the sorted complement of `interior`.
pub fn kron_reduce(ybus: &CsMat<Complex64>, interior: &[usize]) -> Result<Vec<Vec<Complex64>>> {
    let n = ybus.rows();
    if n != ybus.cols() {
        return Err(OxiGridError::InvalidNetwork("Y-bus must be square".into()));
    }

    let mut interior_set: Vec<usize> = interior.to_vec();
    interior_set.sort();
    interior_set.dedup();

    let retained: Vec<usize> = (0..n).filter(|i| !interior_set.contains(i)).collect();
    let nb = retained.len();
    let ni = interior_set.len();

    if ni == 0 {
        return dense_ybus(ybus, &retained);
    }
    if nb == 0 {
        return Err(OxiGridError::InvalidNetwork(
            "No retained buses after reduction".into(),
        ));
    }

    // Extract dense sub-matrices
    let ybb = extract_submatrix(ybus, &retained, &retained);
    let ybi = extract_submatrix(ybus, &retained, &interior_set);
    let yib = extract_submatrix(ybus, &interior_set, &retained);
    let yii = extract_submatrix(ybus, &interior_set, &interior_set);

    // Solve Yii * X = Yib  (X = Yii^-1 * Yib)
    let yii_mat = to_nalgebra(&yii, ni, ni);
    let yib_mat = to_nalgebra(&yib, ni, nb);

    let lu = yii_mat.lu();
    let x = lu
        .solve(&yib_mat)
        .ok_or_else(|| OxiGridError::LinearAlgebra("Yii is singular in Kron reduction".into()))?;

    // Yr = Ybb - Ybi * X
    let ybi_mat = to_nalgebra(&ybi, nb, ni);
    let yr_mat = to_nalgebra(&ybb, nb, nb) - ybi_mat * x;

    // Convert back to Vec<Vec<Complex64>>
    let mut yr = vec![vec![Complex64::new(0.0, 0.0); nb]; nb];
    for i in 0..nb {
        for j in 0..nb {
            yr[i][j] = yr_mat[(i, j)];
        }
    }
    Ok(yr)
}

/// Retained bus indices after Kron reduction.
pub fn retained_buses(n: usize, interior: &[usize]) -> Vec<usize> {
    let mut interior_set = interior.to_vec();
    interior_set.sort();
    interior_set.dedup();
    (0..n).filter(|i| !interior_set.contains(i)).collect()
}

/// Extract power transfer distribution factors (PTDF) matrix.
///
/// PTDF[l, k] = sensitivity of branch `l` real power flow to bus `k` injection.
/// Computed using the DC power flow B matrix.
///
/// PTDF = Xf · B^-1_red
///
/// where Xf is the branch-bus reactance sensitivity matrix and B_red is the
/// reduced susceptance matrix (slack bus removed).
pub fn ptdf_matrix(
    b_bus: &[Vec<f64>],
    branch_from: &[usize],
    branch_to: &[usize],
    branch_x: &[f64],
    slack_idx: usize,
) -> Result<Vec<Vec<f64>>> {
    let n = b_bus.len();
    let m = branch_from.len();

    // Reduced B matrix (remove slack row/col)
    let n_red = n - 1;
    let mut b_red = DMatrix::<f64>::zeros(n_red, n_red);
    let bus_map: Vec<usize> = (0..n).filter(|&i| i != slack_idx).collect();

    for (ri, &i) in bus_map.iter().enumerate() {
        for (rj, &j) in bus_map.iter().enumerate() {
            b_red[(ri, rj)] = b_bus[i][j];
        }
    }

    let lu = b_red.lu();
    // Solve B_red * X = I  → X = B_red^-1
    let identity = DMatrix::<f64>::identity(n_red, n_red);
    let b_inv = lu
        .solve(&identity)
        .ok_or_else(|| OxiGridError::LinearAlgebra("B matrix singular in PTDF".into()))?;

    // PTDF[l, k] = (1/x_l) * (B_inv[from_l, k] - B_inv[to_l, k])
    let mut ptdf = vec![vec![0.0_f64; n]; m];
    for (l, (&from, (&to, &x_l))) in branch_from
        .iter()
        .zip(branch_to.iter().zip(branch_x.iter()))
        .enumerate()
    {
        for (ri, &k) in bus_map.iter().enumerate() {
            let from_row = bus_map
                .iter()
                .position(|&b| b == from)
                .map(|p| b_inv[(p, ri)])
                .unwrap_or(0.0);
            let to_row = bus_map
                .iter()
                .position(|&b| b == to)
                .map(|p| b_inv[(p, ri)])
                .unwrap_or(0.0);
            ptdf[l][k] = (from_row - to_row) / x_l;
        }
    }
    Ok(ptdf)
}

/// Build DC B-bus matrix (purely imaginary parts, no slack).
///
/// B_ij = -1/x_ij for connected buses, B_ii = sum of 1/x_ij for all connected.
pub fn build_b_bus(
    n: usize,
    branch_from: &[usize],
    branch_to: &[usize],
    branch_x: &[f64],
) -> Vec<Vec<f64>> {
    let mut b = vec![vec![0.0_f64; n]; n];
    for ((&from, &to), &x) in branch_from
        .iter()
        .zip(branch_to.iter())
        .zip(branch_x.iter())
    {
        if x.abs() < 1e-12 {
            continue;
        }
        let b_val = 1.0 / x;
        b[from][from] += b_val;
        b[to][to] += b_val;
        b[from][to] -= b_val;
        b[to][from] -= b_val;
    }
    b
}

// ── Helpers ──────────────────────────────────────────────────────────────────

fn extract_submatrix(
    ybus: &CsMat<Complex64>,
    rows: &[usize],
    cols: &[usize],
) -> Vec<Vec<Complex64>> {
    let mut m = vec![vec![Complex64::new(0.0, 0.0); cols.len()]; rows.len()];
    for (ri, &r) in rows.iter().enumerate() {
        for (ci, &c) in cols.iter().enumerate() {
            // Extract element (r, c) from sparse matrix
            if let Some(row_vec) = ybus.outer_view(r) {
                for (&col_idx, &val) in row_vec.indices().iter().zip(row_vec.data().iter()) {
                    if col_idx == c {
                        m[ri][ci] = val;
                        break;
                    }
                }
            }
        }
    }
    m
}

fn dense_ybus(ybus: &CsMat<Complex64>, retained: &[usize]) -> Result<Vec<Vec<Complex64>>> {
    let nb = retained.len();
    let mut y = vec![vec![Complex64::new(0.0, 0.0); nb]; nb];
    for (ri, &r) in retained.iter().enumerate() {
        if let Some(row_vec) = ybus.outer_view(r) {
            for (&col_idx, &val) in row_vec.indices().iter().zip(row_vec.data().iter()) {
                if let Some(ci) = retained.iter().position(|&c| c == col_idx) {
                    y[ri][ci] = val;
                }
            }
        }
    }
    Ok(y)
}

fn to_nalgebra(m: &[Vec<Complex64>], rows: usize, cols: usize) -> DMatrix<Complex64> {
    let mut mat = DMatrix::<Complex64>::zeros(rows, cols);
    for i in 0..rows {
        for j in 0..cols {
            mat[(i, j)] = m[i][j];
        }
    }
    mat
}

/// Lossless DC power transfer sensitivity: line outage distribution factor (LODF).
///
/// LODF[l, k] = change in flow on line `l` as a fraction of pre-outage flow on line `k`.
/// LODF[l, k] = (PTDF[l, from_k] - PTDF[l, to_k]) / (1 - (PTDF[k, from_k] - PTDF[k, to_k]))
pub fn lodf_matrix(ptdf: &[Vec<f64>], branch_from: &[usize], branch_to: &[usize]) -> Vec<Vec<f64>> {
    let m = ptdf.len();
    let mut lodf = vec![vec![0.0_f64; m]; m];
    for l in 0..m {
        for k in 0..m {
            if l == k {
                lodf[l][k] = -1.0; // Self-LODF convention
                continue;
            }
            let ptdf_lk_from = ptdf[l][branch_from[k]];
            let ptdf_lk_to = ptdf[l][branch_to[k]];
            let ptdf_kk_from = ptdf[k][branch_from[k]];
            let ptdf_kk_to = ptdf[k][branch_to[k]];
            let denom = 1.0 - (ptdf_kk_from - ptdf_kk_to);
            if denom.abs() < 1e-12 {
                lodf[l][k] = 0.0; // Disconnected or radial
            } else {
                lodf[l][k] = (ptdf_lk_from - ptdf_lk_to) / denom;
            }
        }
    }
    lodf
}

/// Solve DC power flow: B' · θ = P, returning bus angle vector.
///
/// Uses DVector solve. Slack bus (slack_idx) is fixed at 0 rad.
pub fn dc_solve(b_bus: &[Vec<f64>], p_inj: &[f64], slack_idx: usize) -> Result<Vec<f64>> {
    let n = b_bus.len();
    let n_red = n - 1;
    let bus_map: Vec<usize> = (0..n).filter(|&i| i != slack_idx).collect();

    let mut b_red = DMatrix::<f64>::zeros(n_red, n_red);
    for (ri, &i) in bus_map.iter().enumerate() {
        for (rj, &j) in bus_map.iter().enumerate() {
            b_red[(ri, rj)] = b_bus[i][j];
        }
    }

    let p_red = DVector::<f64>::from_iterator(n_red, bus_map.iter().map(|&i| p_inj[i]));
    let lu = b_red.lu();
    let theta_red = lu
        .solve(&p_red)
        .ok_or_else(|| OxiGridError::LinearAlgebra("B matrix singular in DC solve".into()))?;

    let mut theta = vec![0.0_f64; n];
    for (ri, &i) in bus_map.iter().enumerate() {
        theta[i] = theta_red[ri];
    }
    Ok(theta)
}

// ═══════════════════════════════════════════════════════════════════════════
// Ward Equivalent, Extended Ward, REI Equivalent, and Coherency Analysis
// ═══════════════════════════════════════════════════════════════════════════

/// Bus classification for Ward reduction.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BusRole {
    /// Buses retained in the equivalent network.
    Internal,
    /// Buses to be eliminated (Gaussian elimination on Y-bus).
    External,
    /// Buses connecting internal to external regions.
    Boundary,
}

/// Configuration for computing a Ward equivalent.
#[derive(Debug, Clone)]
pub struct WardEquivalentConfig {
    /// Indices of buses to retain verbatim in the equivalent.
    pub internal_buses: Vec<usize>,
    /// Indices of buses to eliminate.
    pub external_buses: Vec<usize>,
    /// Indices of boundary buses (interface between internal and external).
    pub boundary_buses: Vec<usize>,
    /// Whether to transfer external load injections to boundary buses.
    pub include_load_transfer: bool,
    /// System MVA base.
    pub base_mva: f64,
}

/// Result of a static Ward equivalent computation.
#[derive(Debug, Clone)]
pub struct WardEquivalentResult {
    /// Reduced Y matrix over internal + boundary buses, stored as (G, B) pairs.
    pub y_ward: Vec<Vec<(f64, f64)>>,
    /// Equivalent real-power injection at each boundary bus (MW).
    pub p_ward_injections: Vec<f64>,
    /// Equivalent reactive injection at each boundary bus (MVAr).
    pub q_ward_injections: Vec<f64>,
    /// Number of buses in the original system.
    pub n_buses_reduced: usize,
    /// Number of buses in the equivalent.
    pub n_buses_equivalent: usize,
    /// Fraction of buses eliminated: 1 − n_equivalent/n_original.
    pub reduction_ratio: f64,
}

/// Result of an Extended Ward (XW) equivalent computation.
///
/// Adds voltage-dependent current sources at boundary buses to improve
/// accuracy under varying load levels.
#[derive(Debug, Clone)]
pub struct ExtendedWardEquivalentResult {
    /// Underlying static Ward equivalent.
    pub ward_result: WardEquivalentResult,
    /// Per-boundary-bus voltage correction factors (pu).
    pub voltage_correction: Vec<f64>,
    /// Reactive compensation injections at boundary buses (MVAr).
    pub q_compensation_mvar: Vec<f64>,
}

/// REI (Radial Equivalent Independent) group specification.
#[derive(Debug, Clone)]
pub struct ReiEquivalent {
    /// Indices of generator buses to aggregate.
    pub generator_buses: Vec<usize>,
    /// Indices of load buses to aggregate.
    pub load_buses: Vec<usize>,
}

/// Result of an REI equivalent computation.
#[derive(Debug, Clone)]
pub struct ReiResult {
    /// Index of the fictitious REI aggregation bus.
    pub rei_bus_id: usize,
    /// Voltage magnitude of the REI bus (pu), set by power balance.
    pub rei_voltage_pu: f64,
    /// Total active power of the REI equivalent (MW).
    pub rei_p_mw: f64,
    /// Total reactive power of the REI equivalent (MVAr).
    pub rei_q_mvar: f64,
    /// Spoke admittances: (original_bus_idx, G_spoke, B_spoke).
    pub spoke_admittances: Vec<(usize, f64, f64)>,
    /// Tie admittances between pairs of original buses: ((i, j), G_tie, B_tie).
    pub tie_admittances: Vec<((usize, usize), f64, f64)>,
}

impl ReiEquivalent {
    /// Compute the REI equivalent for the given admittance sub-matrix.
    ///
    /// `y_sub` is the (n×n) admittance sub-matrix over `buses` (combined
    /// generator + load buses), stored as (G, B) pairs.
    /// `p_inj` and `q_inj` are the net active/reactive injections at each bus (MW, MVAr).
    ///
    /// The REI bus aggregates injections via a star network of spoke admittances.
    /// Spoke admittances are chosen proportional to the self-admittance of each bus.
    pub fn compute(
        &self,
        y_sub: &[Vec<(f64, f64)>],
        p_inj: &[f64],
        q_inj: &[f64],
        next_bus_id: usize,
    ) -> crate::error::Result<ReiResult> {
        let all_buses: Vec<usize> = self
            .generator_buses
            .iter()
            .chain(self.load_buses.iter())
            .copied()
            .collect();
        let n = all_buses.len();
        if n == 0 {
            return Err(crate::error::OxiGridError::InvalidParameter(
                "REI equivalent requires at least one bus".into(),
            ));
        }
        if y_sub.len() != n || y_sub.iter().any(|row| row.len() != n) {
            return Err(crate::error::OxiGridError::InvalidParameter(
                "y_sub dimension mismatch for REI equivalent".into(),
            ));
        }
        if p_inj.len() != n || q_inj.len() != n {
            return Err(crate::error::OxiGridError::InvalidParameter(
                "p_inj / q_inj length must equal number of REI buses".into(),
            ));
        }

        // Aggregate injections
        let rei_p: f64 = p_inj.iter().sum();
        let rei_q: f64 = q_inj.iter().sum();

        // Spoke admittances: proportional to |Y_ii| self-admittance
        // Y_spoke_i = Y_ii / n  (equal-share heuristic; avoids zero spokes)
        let mut spokes = Vec::with_capacity(n);
        for (local_i, &orig_bus) in all_buses.iter().enumerate() {
            let (g_ii, b_ii) = y_sub[local_i][local_i];
            // Use magnitude of self-admittance / n as spoke conductance
            let mag = (g_ii * g_ii + b_ii * b_ii).sqrt();
            let g_spoke = if mag > 1e-12 { g_ii / (n as f64) } else { 0.0 };
            let b_spoke = if mag > 1e-12 { b_ii / (n as f64) } else { 0.0 };
            spokes.push((orig_bus, g_spoke, b_spoke));
        }

        // Tie admittances: retain off-diagonal elements as tie lines
        let mut ties = Vec::new();
        for i in 0..n {
            for j in (i + 1)..n {
                let (g_ij, b_ij) = y_sub[i][j];
                if g_ij.abs() > 1e-14 || b_ij.abs() > 1e-14 {
                    ties.push(((all_buses[i], all_buses[j]), -g_ij, -b_ij));
                }
            }
        }

        // REI bus voltage: 1.0 pu by convention (equivalent replaces external
        // injections; exact voltage computed in the full power flow)
        let rei_voltage_pu = 1.0_f64;

        Ok(ReiResult {
            rei_bus_id: next_bus_id,
            rei_voltage_pu,
            rei_p_mw: rei_p,
            rei_q_mvar: rei_q,
            spoke_admittances: spokes,
            tie_admittances: ties,
        })
    }
}

// ─── Complex arithmetic helpers (scalar operations on (re, im) tuples) ────────

/// Multiply two complex numbers: (a+jb)·(c+jd) = (ac-bd) + j(ad+bc).
#[inline]
fn cmul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Divide two complex numbers: (a+jb)/(c+jd).
#[inline]
fn cdiv(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let denom = b.0 * b.0 + b.1 * b.1;
    if denom < 1e-300 {
        (f64::INFINITY, f64::INFINITY)
    } else {
        (
            (a.0 * b.0 + a.1 * b.1) / denom,
            (a.1 * b.0 - a.0 * b.1) / denom,
        )
    }
}

/// Invert a complex scalar: 1/(a+jb) = (a-jb)/(a²+b²).
#[inline]
fn cinv(re: f64, im: f64) -> (f64, f64) {
    cdiv((1.0, 0.0), (re, im))
}

/// Add two complex numbers.
#[inline]
fn cadd(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

/// Subtract two complex numbers.
#[inline]
fn csub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

/// Negate a complex number.
#[inline]
#[allow(dead_code)]
fn cneg(a: (f64, f64)) -> (f64, f64) {
    (-a.0, -a.1)
}

// ─── KronReducer ──────────────────────────────────────────────────────────────

/// Sequential Kron (Gaussian) reducer operating on a dense complex admittance
/// matrix stored as `Vec<Vec<(f64, f64)>>` (G, B) pairs.
///
/// Each call to [`KronReducer::eliminate_bus`] removes one row/column from
/// the matrix using the Kron formula:
///
/// ```text
/// Y_ij_new = Y_ij − Y_ik · Y_kj / Y_kk   (i, j ≠ k)
/// ```
#[derive(Debug, Clone)]
pub struct KronReducer {
    /// Current number of buses (decreases after each elimination).
    pub n_buses: usize,
    /// Working admittance matrix (shrinks as buses are eliminated).
    pub y_full: Vec<Vec<(f64, f64)>>,
    /// Tracks which original bus indices remain active.
    active_buses: Vec<usize>,
}

impl KronReducer {
    /// Construct from an n×n admittance matrix given as (G, B) pairs.
    pub fn new(n_buses: usize, y_full: Vec<Vec<(f64, f64)>>) -> Self {
        let active_buses: Vec<usize> = (0..n_buses).collect();
        Self {
            n_buses,
            y_full,
            active_buses,
        }
    }

    /// Eliminate bus with *original* index `bus` from the working matrix.
    ///
    /// The local index within the current (possibly reduced) matrix is found
    /// by looking up `bus` in `active_buses`.
    pub fn eliminate_bus(&mut self, bus: usize) -> crate::error::Result<()> {
        // Find local index of the bus in the current working matrix
        let local_k = self
            .active_buses
            .iter()
            .position(|&b| b == bus)
            .ok_or_else(|| {
                crate::error::OxiGridError::InvalidParameter(format!(
                    "Bus {bus} not found in active buses for Kron elimination"
                ))
            })?;

        let n = self.y_full.len();
        let y_kk = self.y_full[local_k][local_k];
        let denom = y_kk.0 * y_kk.0 + y_kk.1 * y_kk.1;
        if denom < 1e-300 {
            return Err(crate::error::OxiGridError::LinearAlgebra(format!(
                "Y_kk is singular for bus {bus} during Kron elimination"
            )));
        }

        // Build new (n-1) × (n-1) matrix
        let mut new_y: Vec<Vec<(f64, f64)>> = Vec::with_capacity(n - 1);
        for i in 0..n {
            if i == local_k {
                continue;
            }
            let mut row = Vec::with_capacity(n - 1);
            for j in 0..n {
                if j == local_k {
                    continue;
                }
                let y_ij = self.y_full[i][j];
                let y_ik = self.y_full[i][local_k];
                let y_kj = self.y_full[local_k][j];
                // Y_ij_new = Y_ij − Y_ik * Y_kj / Y_kk
                let correction = cdiv(cmul(y_ik, y_kj), y_kk);
                row.push(csub(y_ij, correction));
            }
            new_y.push(row);
        }

        self.y_full = new_y;
        self.active_buses.remove(local_k);
        self.n_buses -= 1;
        Ok(())
    }

    /// Eliminate multiple buses in the given order.
    pub fn eliminate_buses(&mut self, buses: &[usize]) -> crate::error::Result<()> {
        for &bus in buses {
            self.eliminate_bus(bus)?;
        }
        Ok(())
    }

    /// Return the current reduced admittance matrix.
    pub fn reduced_matrix(&self) -> Vec<Vec<(f64, f64)>> {
        self.y_full.clone()
    }

    /// Compute the Thévenin impedance between bus pair (i, j) in the *current*
    /// reduced network.
    ///
    /// Requires inverting the current Y matrix to obtain Z = Y⁻¹, then:
    /// `Z_th = Z_ii + Z_jj − 2·Z_ij`
    pub fn thevenin_impedance(
        &self,
        bus_i: usize,
        bus_j: usize,
    ) -> crate::error::Result<(f64, f64)> {
        let li = self
            .active_buses
            .iter()
            .position(|&b| b == bus_i)
            .ok_or_else(|| {
                crate::error::OxiGridError::InvalidParameter(format!(
                    "Bus {bus_i} not in reduced network"
                ))
            })?;
        let lj = self
            .active_buses
            .iter()
            .position(|&b| b == bus_j)
            .ok_or_else(|| {
                crate::error::OxiGridError::InvalidParameter(format!(
                    "Bus {bus_j} not in reduced network"
                ))
            })?;

        let z = complex_matrix_inverse(&self.y_full)?;
        let z_th = csub(cadd(z[li][li], z[lj][lj]), cmul((2.0, 0.0), z[li][lj]));
        Ok(z_th)
    }
}

/// Invert an n×n complex matrix via Gaussian elimination with partial pivoting.
fn complex_matrix_inverse(mat: &[Vec<(f64, f64)>]) -> crate::error::Result<Vec<Vec<(f64, f64)>>> {
    let n = mat.len();
    if n == 0 {
        return Err(crate::error::OxiGridError::LinearAlgebra(
            "Cannot invert empty matrix".into(),
        ));
    }

    // Augment [mat | I]
    let mut aug: Vec<Vec<(f64, f64)>> = (0..n)
        .map(|i| {
            let mut row: Vec<(f64, f64)> = mat[i].to_vec();
            for j in 0..n {
                row.push(if i == j { (1.0, 0.0) } else { (0.0, 0.0) });
            }
            row
        })
        .collect();

    for col in 0..n {
        // Partial pivot: find row with largest |Y_kk|
        let pivot_row = (col..n)
            .max_by(|&a, &b| {
                let mag_a = aug[a][col].0 * aug[a][col].0 + aug[a][col].1 * aug[a][col].1;
                let mag_b = aug[b][col].0 * aug[b][col].0 + aug[b][col].1 * aug[b][col].1;
                mag_a
                    .partial_cmp(&mag_b)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .ok_or_else(|| {
                crate::error::OxiGridError::LinearAlgebra(
                    "Empty range during matrix inversion".into(),
                )
            })?;

        aug.swap(col, pivot_row);

        let pivot = aug[col][col];
        let pivot_mag = pivot.0 * pivot.0 + pivot.1 * pivot.1;
        if pivot_mag < 1e-300 {
            return Err(crate::error::OxiGridError::LinearAlgebra(
                "Singular matrix: cannot invert".into(),
            ));
        }
        let pivot_inv = cinv(pivot.0, pivot.1);

        // Scale pivot row
        for val in aug[col].iter_mut() {
            *val = cmul(*val, pivot_inv);
        }

        // Eliminate column from all other rows
        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = aug[row][col];
            if factor.0.abs() < 1e-300 && factor.1.abs() < 1e-300 {
                continue;
            }
            #[allow(clippy::needless_range_loop)]
            for j in 0..2 * n {
                let sub = cmul(factor, aug[col][j]);
                aug[row][j] = csub(aug[row][j], sub);
            }
        }
    }

    // Extract inverse from augmented matrix
    let inv: Vec<Vec<(f64, f64)>> = aug.iter().map(|row| row[n..].to_vec()).collect();
    Ok(inv)
}

// ─── WardEquivalent ──────────────────────────────────────────────────────────

/// Computes static and extended Ward equivalents for large power systems.
///
/// The Ward equivalent eliminates all *external* buses via Schur complement
/// on the partitioned admittance matrix, retaining only internal + boundary
/// buses.  External load injections are transferred to boundary buses.
///
/// # Partition
///
/// ```text
/// Y = [ Y_ii  Y_ib  Y_ie ]   i = internal
///     [ Y_bi  Y_bb  Y_be ]   b = boundary
///     [ Y_ei  Y_eb  Y_ee ]   e = external
/// ```
///
/// Reduced matrix:
/// ```text
/// Y_ward_ii = Y_ii − Y_ie · Y_ee⁻¹ · Y_ei
/// Y_ward_ib = Y_ib − Y_ie · Y_ee⁻¹ · Y_eb
/// Y_ward_bb = Y_bb − Y_be · Y_ee⁻¹ · Y_eb
/// ```
///
/// Boundary injection transfer:
/// ```text
/// ΔI_boundary = −Y_be · Y_ee⁻¹ · I_external
/// ```
#[derive(Debug, Clone)]
pub struct WardEquivalent {
    /// Total number of buses in the original system.
    pub n_buses: usize,
    /// Full admittance matrix of the original system (n×n), (G, B) pairs.
    pub y_matrix: Vec<Vec<(f64, f64)>>,
    /// Configuration specifying bus partitions and options.
    pub config: WardEquivalentConfig,
}

impl WardEquivalent {
    /// Construct a new Ward equivalent solver.
    pub fn new(
        n_buses: usize,
        y_matrix: Vec<Vec<(f64, f64)>>,
        config: WardEquivalentConfig,
    ) -> Self {
        Self {
            n_buses,
            y_matrix,
            config,
        }
    }

    /// Compute the static Ward equivalent.
    ///
    /// `p_ext` and `q_ext` are the net active/reactive injections (MW, MVAr)
    /// at the external buses, in the same order as `config.external_buses`.
    pub fn compute(
        &self,
        p_ext: &[f64],
        q_ext: &[f64],
    ) -> crate::error::Result<WardEquivalentResult> {
        let int_b = &self.config.internal_buses;
        let ext_b = &self.config.external_buses;
        let bnd_b = &self.config.boundary_buses;

        if p_ext.len() != ext_b.len() || q_ext.len() != ext_b.len() {
            return Err(crate::error::OxiGridError::InvalidParameter(
                "p_ext/q_ext length must match external_buses count".into(),
            ));
        }

        let n_ext = ext_b.len();
        let n_int = int_b.len();
        let n_bnd = bnd_b.len();
        let n_equiv = n_int + n_bnd;

        // All retained buses: internal first, then boundary
        let retained: Vec<usize> = int_b.iter().chain(bnd_b.iter()).copied().collect();

        if n_ext == 0 {
            // Nothing to eliminate: return full sub-matrix
            let y_ward = self.extract_submatrix_gb(&retained, &retained);
            let reduction_ratio = 0.0;
            return Ok(WardEquivalentResult {
                y_ward,
                p_ward_injections: vec![0.0; n_bnd],
                q_ward_injections: vec![0.0; n_bnd],
                n_buses_reduced: self.n_buses,
                n_buses_equivalent: n_equiv,
                reduction_ratio,
            });
        }

        // ── Extract sub-matrices ──────────────────────────────────────────────
        // Y_ee (external × external)
        let y_ee = self.extract_submatrix_gb(ext_b, ext_b);
        // Y_ei (external × internal)
        let y_ei = self.extract_submatrix_gb(ext_b, int_b);
        // Y_eb (external × boundary)
        let y_eb = self.extract_submatrix_gb(ext_b, bnd_b);
        // Y_ie (internal × external)
        let y_ie = self.extract_submatrix_gb(int_b, ext_b);
        // Y_be (boundary × external)
        let y_be = self.extract_submatrix_gb(bnd_b, ext_b);
        // Base matrices
        let y_ii = self.extract_submatrix_gb(int_b, int_b);
        let y_ib = self.extract_submatrix_gb(int_b, bnd_b);
        let y_bi = self.extract_submatrix_gb(bnd_b, int_b);
        let y_bb = self.extract_submatrix_gb(bnd_b, bnd_b);

        // ── Solve Y_ee * X_ei = Y_ei  →  X_ei = Y_ee^{-1} * Y_ei ────────────
        let x_ei = Self::solve_rhs_matrix(&y_ee, &y_ei)?;
        // Solve Y_ee * X_eb = Y_eb  →  X_eb = Y_ee^{-1} * Y_eb
        let x_eb = Self::solve_rhs_matrix(&y_ee, &y_eb)?;

        // ── Compute reduced sub-matrices ──────────────────────────────────────
        // Y_ward_ii = Y_ii − Y_ie * X_ei
        let y_ward_ii = Self::mat_sub(&y_ii, &Self::mat_mul(&y_ie, &x_ei));
        // Y_ward_ib = Y_ib − Y_ie * X_eb
        let y_ward_ib = Self::mat_sub(&y_ib, &Self::mat_mul(&y_ie, &x_eb));
        // Y_ward_bi = Y_bi − Y_be * X_ei
        let y_ward_bi = Self::mat_sub(&y_bi, &Self::mat_mul(&y_be, &x_ei));
        // Y_ward_bb = Y_bb − Y_be * X_eb
        let y_ward_bb = Self::mat_sub(&y_bb, &Self::mat_mul(&y_be, &x_eb));

        // ── Assemble (n_int + n_bnd) × (n_int + n_bnd) Ward Y matrix ─────────
        let mut y_ward = vec![vec![(0.0_f64, 0.0_f64); n_equiv]; n_equiv];
        // Top-left: ii block
        for i in 0..n_int {
            for j in 0..n_int {
                y_ward[i][j] = y_ward_ii[i][j];
            }
        }
        // Top-right: ib block
        for i in 0..n_int {
            for j in 0..n_bnd {
                y_ward[i][n_int + j] = y_ward_ib[i][j];
            }
        }
        // Bottom-left: bi block
        for i in 0..n_bnd {
            for j in 0..n_int {
                y_ward[n_int + i][j] = y_ward_bi[i][j];
            }
        }
        // Bottom-right: bb block
        for i in 0..n_bnd {
            for j in 0..n_bnd {
                y_ward[n_int + i][n_int + j] = y_ward_bb[i][j];
            }
        }

        // ── Transfer external injections to boundary buses ────────────────────
        // I_ext as complex vector (P + jQ, in per-unit current approximation)
        let base = self.config.base_mva;
        let i_ext: Vec<(f64, f64)> = (0..n_ext)
            .map(|k| (p_ext[k] / base, q_ext[k] / base))
            .collect();

        // X_be = Y_ee^{-1} * column-by-column is already in x_eb rows from boundary perspective
        // We need Y_be * Y_ee^{-1} * I_ext = Y_be * (Y_ee^{-1} * I_ext)
        let x_i_ext = Self::solve_linear(&y_ee, &i_ext)?;
        // ΔI_boundary = −Y_be * x_i_ext
        let delta_i_bnd = Self::mat_vec_mul(&y_be, &x_i_ext);

        let p_ward_injections: Vec<f64> = delta_i_bnd.iter().map(|&(re, _im)| -re * base).collect();
        let q_ward_injections: Vec<f64> = delta_i_bnd.iter().map(|&(_re, im)| -im * base).collect();

        let n_orig = self.n_buses;
        let reduction_ratio = 1.0 - (n_equiv as f64) / (n_orig as f64).max(1.0);

        Ok(WardEquivalentResult {
            y_ward,
            p_ward_injections,
            q_ward_injections,
            n_buses_reduced: n_orig,
            n_buses_equivalent: n_equiv,
            reduction_ratio,
        })
    }

    /// Compute the Extended Ward equivalent with voltage-dependent correction.
    ///
    /// `v_ext` holds the voltage magnitudes (pu) at the external buses.
    /// The correction captures first-order sensitivity of boundary currents
    /// to deviations of external voltages from 1.0 pu.
    pub fn compute_extended(
        &self,
        p_ext: &[f64],
        q_ext: &[f64],
        v_ext: &[f64],
    ) -> crate::error::Result<ExtendedWardEquivalentResult> {
        if v_ext.len() != self.config.external_buses.len() {
            return Err(crate::error::OxiGridError::InvalidParameter(
                "v_ext length must match external_buses count".into(),
            ));
        }

        let ward_result = self.compute(p_ext, q_ext)?;
        let n_bnd = self.config.boundary_buses.len();

        // Voltage correction: for each boundary bus, compute weighted average
        // of external voltage deviations scaled by Y_be admittance magnitudes.
        let y_be =
            self.extract_submatrix_gb(&self.config.boundary_buses, &self.config.external_buses);
        let n_ext = self.config.external_buses.len();

        let mut voltage_correction = vec![0.0_f64; n_bnd];
        let mut q_compensation_mvar = vec![0.0_f64; n_bnd];

        for i in 0..n_bnd {
            let mut weight_sum = 0.0_f64;
            let mut v_corr = 0.0_f64;
            let mut q_corr = 0.0_f64;
            for k in 0..n_ext {
                let (g_ik, b_ik) = y_be[i][k];
                let admittance_mag = (g_ik * g_ik + b_ik * b_ik).sqrt();
                let delta_v = v_ext[k] - 1.0; // deviation from nominal
                v_corr += admittance_mag * delta_v;
                // Q correction: ΔQ ≈ −B_ik · ΔV_k (linearised)
                q_corr += -b_ik * delta_v * self.config.base_mva;
                weight_sum += admittance_mag;
            }
            if weight_sum > 1e-12 {
                voltage_correction[i] = v_corr / weight_sum;
            }
            q_compensation_mvar[i] = q_corr;
        }

        Ok(ExtendedWardEquivalentResult {
            ward_result,
            voltage_correction,
            q_compensation_mvar,
        })
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    /// Extract a sub-matrix of `y_matrix` for given row and column bus indices.
    fn extract_submatrix_gb(&self, rows: &[usize], cols: &[usize]) -> Vec<Vec<(f64, f64)>> {
        rows.iter()
            .map(|&r| {
                cols.iter()
                    .map(|&c| {
                        if r < self.y_matrix.len() && c < self.y_matrix[r].len() {
                            self.y_matrix[r][c]
                        } else {
                            (0.0, 0.0)
                        }
                    })
                    .collect()
            })
            .collect()
    }

    /// Solve the complex linear system A·x = b using Gaussian elimination
    /// with partial pivoting.
    ///
    /// `a` is an n×n complex matrix; `b` is an n-vector of complex numbers.
    /// Returns the solution vector `x`.
    pub fn solve_linear(
        a: &[Vec<(f64, f64)>],
        b: &[(f64, f64)],
    ) -> crate::error::Result<Vec<(f64, f64)>> {
        let n = a.len();
        if n == 0 || b.len() != n {
            return Err(crate::error::OxiGridError::LinearAlgebra(
                "Dimension mismatch in solve_linear".into(),
            ));
        }

        // Augment [A | b]
        let mut aug: Vec<Vec<(f64, f64)>> = a
            .iter()
            .zip(b.iter())
            .map(|(row, &rhs)| {
                let mut r = row.to_vec();
                r.push(rhs);
                r
            })
            .collect();

        for col in 0..n {
            // Partial pivot
            let pivot_row = (col..n)
                .max_by(|&ra, &rb| {
                    let ma = aug[ra][col].0 * aug[ra][col].0 + aug[ra][col].1 * aug[ra][col].1;
                    let mb = aug[rb][col].0 * aug[rb][col].0 + aug[rb][col].1 * aug[rb][col].1;
                    ma.partial_cmp(&mb).unwrap_or(std::cmp::Ordering::Equal)
                })
                .ok_or_else(|| {
                    crate::error::OxiGridError::LinearAlgebra(
                        "Empty range in solve_linear pivot".into(),
                    )
                })?;
            aug.swap(col, pivot_row);

            let pivot = aug[col][col];
            let pmag = pivot.0 * pivot.0 + pivot.1 * pivot.1;
            if pmag < 1e-300 {
                return Err(crate::error::OxiGridError::LinearAlgebra(
                    "Singular system in Ward equivalent solve".into(),
                ));
            }
            let pivot_inv = cinv(pivot.0, pivot.1);
            for val in aug[col].iter_mut() {
                *val = cmul(*val, pivot_inv);
            }
            for row in 0..n {
                if row == col {
                    continue;
                }
                let fac = aug[row][col];
                if fac.0.abs() < 1e-300 && fac.1.abs() < 1e-300 {
                    continue;
                }
                #[allow(clippy::needless_range_loop)]
                for j in 0..=n {
                    let sub = cmul(fac, aug[col][j]);
                    aug[row][j] = csub(aug[row][j], sub);
                }
            }
        }

        Ok(aug.iter().map(|row| row[n]).collect())
    }

    /// Solve A·X = B for a right-hand-side matrix B (multiple right-hand sides).
    fn solve_rhs_matrix(
        a: &[Vec<(f64, f64)>],
        b: &[Vec<(f64, f64)>],
    ) -> crate::error::Result<Vec<Vec<(f64, f64)>>> {
        let n_cols = if b.is_empty() { 0 } else { b[0].len() };
        let mut result = Vec::with_capacity(a.len());

        // Solve column by column
        for col in 0..n_cols {
            let rhs: Vec<(f64, f64)> = b.iter().map(|row| row[col]).collect();
            let x_col = Self::solve_linear(a, &rhs)?;
            result.push(x_col);
        }

        // Transpose: result is currently (cols × rows), need (rows × cols)
        let n_rows = a.len();
        let mut out = vec![vec![(0.0_f64, 0.0_f64); n_cols]; n_rows];
        for (col_idx, col_vec) in result.iter().enumerate() {
            for (row_idx, &val) in col_vec.iter().enumerate() {
                out[row_idx][col_idx] = val;
            }
        }
        Ok(out)
    }

    /// Complex matrix-vector multiply: y = A·x.
    pub fn mat_vec_mul(a: &[Vec<(f64, f64)>], x: &[(f64, f64)]) -> Vec<(f64, f64)> {
        a.iter()
            .map(|row| {
                row.iter()
                    .zip(x.iter())
                    .fold((0.0_f64, 0.0_f64), |acc, (&a_ij, &x_j)| {
                        cadd(acc, cmul(a_ij, x_j))
                    })
            })
            .collect()
    }

    /// Complex matrix-matrix multiply: C = A·B.
    fn mat_mul(a: &[Vec<(f64, f64)>], b: &[Vec<(f64, f64)>]) -> Vec<Vec<(f64, f64)>> {
        if a.is_empty() || b.is_empty() {
            return Vec::new();
        }
        let n_rows = a.len();
        let n_cols = b[0].len();
        let n_inner = b.len();
        let mut c = vec![vec![(0.0_f64, 0.0_f64); n_cols]; n_rows];
        for i in 0..n_rows {
            for j in 0..n_cols {
                let mut sum = (0.0_f64, 0.0_f64);
                for k in 0..n_inner {
                    sum = cadd(sum, cmul(a[i][k], b[k][j]));
                }
                c[i][j] = sum;
            }
        }
        c
    }

    /// Complex matrix subtraction: C = A − B.
    fn mat_sub(a: &[Vec<(f64, f64)>], b: &[Vec<(f64, f64)>]) -> Vec<Vec<(f64, f64)>> {
        a.iter()
            .zip(b.iter())
            .map(|(row_a, row_b)| {
                row_a
                    .iter()
                    .zip(row_b.iter())
                    .map(|(&x, &y)| csub(x, y))
                    .collect()
            })
            .collect()
    }

    /// Scalar complex inverse (exposed for direct use).
    pub fn complex_inv(re: f64, im: f64) -> (f64, f64) {
        cinv(re, im)
    }
}

// ─── CoherencyAnalyzer ───────────────────────────────────────────────────────

/// Identifies coherent generator groups from rotor-angle time trajectories.
///
/// Two generators are considered coherent if the Pearson correlation of their
/// rotor angle trajectories exceeds `coherency_threshold`.  Coherent generators
/// are grouped using agglomerative (single-linkage) clustering.
#[derive(Debug, Clone)]
pub struct CoherencyAnalyzer {
    /// Indices of generator buses in the power network.
    pub generator_buses: Vec<usize>,
    /// Correlation threshold for coherency classification (default 0.95).
    pub coherency_threshold: f64,
}

/// A group of coherent generators that can be aggregated into a single
/// dynamic equivalent machine.
#[derive(Debug, Clone)]
pub struct CoherencyGroup {
    /// Sequential group identifier.
    pub group_id: usize,
    /// Original bus indices belonging to this coherent group.
    pub buses: Vec<usize>,
    /// Aggregate inertia constant H (s): H_agg = Σ(H_i · S_i) / Σ S_i.
    pub aggregate_inertia_s: f64,
    /// Total generating capacity (MW) of the group.
    pub aggregate_capacity_mw: f64,
    /// Dominant inter-area swing mode frequency (Hz), estimated from FFT of
    /// mean angle deviation.
    pub swing_mode_frequency_hz: f64,
}

impl CoherencyAnalyzer {
    /// Construct with specified generator buses and correlation threshold.
    pub fn new(generator_buses: Vec<usize>, coherency_threshold: f64) -> Self {
        Self {
            generator_buses,
            coherency_threshold,
        }
    }

    /// Identify coherent groups from rotor-angle trajectories.
    ///
    /// `angle_trajectories[i]` is the rotor angle time series (radians) for
    /// generator `i`, sampled at intervals of `dt_s` seconds.
    ///
    /// Inertia constants and capacities default to 1.0 when not provided (use
    /// [`CoherencyGroup::aggregate_inertia_s`] as a relative index in that case).
    pub fn identify_groups(
        &self,
        angle_trajectories: &[Vec<f64>],
        dt_s: f64,
    ) -> Vec<CoherencyGroup> {
        let n_gen = self.generator_buses.len();
        if n_gen == 0 || angle_trajectories.is_empty() {
            return Vec::new();
        }
        let n_traj = angle_trajectories.len().min(n_gen);

        // Compute pairwise correlations
        let mut corr = vec![vec![0.0_f64; n_traj]; n_traj];
        for i in 0..n_traj {
            corr[i][i] = 1.0;
            for j in (i + 1)..n_traj {
                let c = Self::pearson_correlation(&angle_trajectories[i], &angle_trajectories[j]);
                corr[i][j] = c;
                corr[j][i] = c;
            }
        }

        // Agglomerative single-linkage clustering
        // Each generator starts in its own group
        let mut group_of: Vec<usize> = (0..n_traj).collect();
        let mut n_groups = n_traj;

        // Merge pairs whose correlation exceeds threshold
        for i in 0..n_traj {
            for j in (i + 1)..n_traj {
                if corr[i][j] >= self.coherency_threshold {
                    // Merge group_of[j] into group_of[i]
                    let old_group = group_of[j];
                    let new_group = group_of[i];
                    if old_group != new_group {
                        for g in group_of.iter_mut() {
                            if *g == old_group {
                                *g = new_group;
                            }
                        }
                        n_groups = n_groups.saturating_sub(1);
                    }
                }
            }
        }

        // Collect group members
        let mut unique_ids: Vec<usize> = group_of.to_vec();
        unique_ids.sort();
        unique_ids.dedup();

        let uniform_inertia = vec![1.0_f64; n_traj];
        let uniform_capacity = vec![1.0_f64; n_traj];

        unique_ids
            .into_iter()
            .enumerate()
            .map(|(gid, group_id_val)| {
                let members: Vec<usize> = (0..n_traj)
                    .filter(|&k| group_of[k] == group_id_val)
                    .collect();
                let bus_indices: Vec<usize> =
                    members.iter().map(|&k| self.generator_buses[k]).collect();

                let agg_inertia =
                    Self::aggregate_inertia(&uniform_inertia, &uniform_capacity, &members);
                let agg_capacity: f64 = members.iter().map(|&k| uniform_capacity[k]).sum();

                // Estimate swing frequency from mean angle deviation
                let swing_hz = if dt_s > 0.0 {
                    Self::estimate_swing_frequency(&members, angle_trajectories, dt_s)
                } else {
                    0.0
                };

                CoherencyGroup {
                    group_id: gid,
                    buses: bus_indices,
                    aggregate_inertia_s: agg_inertia,
                    aggregate_capacity_mw: agg_capacity,
                    swing_mode_frequency_hz: swing_hz,
                }
            })
            .collect()
    }

    /// Compute the Pearson correlation coefficient between two equal-length
    /// time series.  Returns 0.0 if either series has zero variance.
    pub fn pearson_correlation(x: &[f64], y: &[f64]) -> f64 {
        let n = x.len().min(y.len());
        if n < 2 {
            return 0.0;
        }
        let n_f = n as f64;
        let mean_x: f64 = x[..n].iter().sum::<f64>() / n_f;
        let mean_y: f64 = y[..n].iter().sum::<f64>() / n_f;

        let mut cov = 0.0_f64;
        let mut var_x = 0.0_f64;
        let mut var_y = 0.0_f64;
        for i in 0..n {
            let dx = x[i] - mean_x;
            let dy = y[i] - mean_y;
            cov += dx * dy;
            var_x += dx * dx;
            var_y += dy * dy;
        }
        let denom = (var_x * var_y).sqrt();
        if denom < 1e-14 {
            0.0
        } else {
            cov / denom
        }
    }

    /// Aggregate inertia constant: H_agg = Σ(H_i · S_i) / Σ S_i.
    ///
    /// `indices` selects which generators (by position) to include.
    pub fn aggregate_inertia(
        inertia_constants: &[f64],
        capacities_mva: &[f64],
        indices: &[usize],
    ) -> f64 {
        let mut weighted_sum = 0.0_f64;
        let mut total_s = 0.0_f64;
        for &idx in indices {
            if idx < inertia_constants.len() && idx < capacities_mva.len() {
                weighted_sum += inertia_constants[idx] * capacities_mva[idx];
                total_s += capacities_mva[idx];
            }
        }
        if total_s < 1e-12 {
            0.0
        } else {
            weighted_sum / total_s
        }
    }

    /// Estimate dominant swing frequency from the mean angle deviation of the group.
    ///
    /// Uses zero-crossing counting on the mean angle deviation from steady state
    /// (first sample) to approximate the primary oscillation frequency.
    fn estimate_swing_frequency(members: &[usize], trajectories: &[Vec<f64>], dt_s: f64) -> f64 {
        if members.is_empty() || trajectories.is_empty() {
            return 0.0;
        }
        let len = trajectories[members[0]].len();
        if len < 4 {
            return 0.0;
        }

        // Compute mean angle trajectory for this group
        let mean_traj: Vec<f64> = (0..len)
            .map(|t| {
                let s: f64 = members
                    .iter()
                    .filter(|&&k| k < trajectories.len() && t < trajectories[k].len())
                    .map(|&k| trajectories[k][t])
                    .sum();
                let cnt = members
                    .iter()
                    .filter(|&&k| k < trajectories.len() && t < trajectories[k].len())
                    .count();
                if cnt > 0 {
                    s / cnt as f64
                } else {
                    0.0
                }
            })
            .collect();

        // Deviation from first sample (steady state reference)
        let ref_val = mean_traj[0];
        let deviation: Vec<f64> = mean_traj.iter().map(|&v| v - ref_val).collect();

        // Count zero crossings
        let mut crossings = 0usize;
        for i in 1..deviation.len() {
            if deviation[i - 1] * deviation[i] < 0.0 {
                crossings += 1;
            }
        }

        // Each full oscillation = 2 zero crossings
        let total_time = (len - 1) as f64 * dt_s;
        if total_time < 1e-9 || crossings == 0 {
            0.0
        } else {
            (crossings as f64 / 2.0) / total_time
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::PowerNetwork;

    #[test]
    fn test_build_b_bus_simple() {
        // Simple 3-bus network: 0-1 (x=0.1), 1-2 (x=0.2), 0-2 (x=0.4)
        let from = [0, 1, 0];
        let to = [1, 2, 2];
        let x = [0.1, 0.2, 0.4];
        let b = build_b_bus(3, &from, &to, &x);

        assert!((b[0][0] - (10.0 + 2.5)).abs() < 1e-9); // 1/0.1 + 1/0.4
        assert!((b[1][1] - (10.0 + 5.0)).abs() < 1e-9); // 1/0.1 + 1/0.2
        assert!((b[0][1] - (-10.0)).abs() < 1e-9);
        assert!((b[1][0] - (-10.0)).abs() < 1e-9);
    }

    #[test]
    fn test_dc_solve_3bus() {
        let from = [0, 1, 0];
        let to = [1, 2, 2];
        let x = [0.1, 0.2, 0.4];
        let b = build_b_bus(3, &from, &to, &x);

        // Bus 0 = slack, bus 1 injects 0.5 pu, bus 2 consumes 0.5 pu
        let p = [0.0, 0.5, -0.5];
        let theta = dc_solve(&b, &p, 0).unwrap();

        // Slack = 0
        assert!(theta[0].abs() < 1e-10);
        // Angles should be non-trivial
        assert!(theta[1].abs() > 1e-6);
        assert!(theta[2].abs() > 1e-6);
    }

    #[test]
    fn test_kron_reduce_two_bus() {
        // 3-bus: retain buses 0 and 2, eliminate bus 1
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
        if let Ok(net) = PowerNetwork::from_matpower(path) {
            let ybus = net.admittance_matrix().unwrap();
            // Eliminate buses 2..13, retain bus 0 and 1
            let interior: Vec<usize> = (2..14).collect();
            let yr = kron_reduce(&ybus, &interior).unwrap();
            // Reduced matrix should be 2×2
            assert_eq!(yr.len(), 2);
            assert_eq!(yr[0].len(), 2);
            // Diagonal should be non-zero
            assert!(yr[0][0].norm() > 0.0);
            assert!(yr[1][1].norm() > 0.0);
        }
    }

    #[test]
    fn test_retained_buses() {
        let retained = retained_buses(5, &[1, 3]);
        assert_eq!(retained, vec![0, 2, 4]);
    }

    #[test]
    fn test_ptdf_3bus() {
        let from = [0, 1, 0];
        let to = [1, 2, 2];
        let x = [0.1, 0.2, 0.4];
        let b = build_b_bus(3, &from, &to, &x);
        let ptdf = ptdf_matrix(&b, &from, &to, &x, 0).unwrap();
        // PTDF should be n_branch × n_bus
        assert_eq!(ptdf.len(), 3);
        assert_eq!(ptdf[0].len(), 3);
        // PTDF at slack bus = 0
        for row in &ptdf {
            assert!(row[0].abs() < 1e-10, "PTDF at slack != 0: {}", row[0]);
        }
    }

    // ── KronReducer tests ────────────────────────────────────────────────────

    /// Helper: build a 2×2 Y matrix with one shunt and one branch.
    fn make_2x2_y() -> Vec<Vec<(f64, f64)>> {
        // Bus 0-1 branch: y_12 = 1/(0+j0.1) = 0 - j10
        // Self-admittance = j10 each
        vec![
            vec![(0.0, 10.0), (0.0, -10.0)],
            vec![(0.0, -10.0), (0.0, 10.0)],
        ]
    }

    fn make_3x3_y() -> Vec<Vec<(f64, f64)>> {
        // 3-bus ring: all branches y = 1/(0+j0.1) = -j10
        vec![
            vec![(0.0, 20.0), (0.0, -10.0), (0.0, -10.0)],
            vec![(0.0, -10.0), (0.0, 20.0), (0.0, -10.0)],
            vec![(0.0, -10.0), (0.0, -10.0), (0.0, 20.0)],
        ]
    }

    #[test]
    fn test_kron_eliminate_bus_2x2() {
        let y = make_2x2_y();
        let mut kr = KronReducer::new(2, y);
        // Eliminate bus 1
        kr.eliminate_bus(1).expect("eliminate_bus should succeed");
        let red = kr.reduced_matrix();
        assert_eq!(red.len(), 1, "Should have 1 bus left");
        // After eliminating bus 1: Y_00_new = Y_00 - Y_01*Y_10/Y_11
        // = j10 - (-j10)*(-j10)/(j10) = j10 - (-100)/(j10) = j10 - j10 = 0
        // (shunt to ground consumed by Kron)
        let (g00, b00) = red[0][0];
        assert!(g00.abs() < 1e-9, "G_00 should be ~0, got {g00}");
        assert!(b00.abs() < 1e-9, "B_00 should be ~0, got {b00}");
    }

    #[test]
    fn test_kron_eliminate_bus_3x3() {
        let y = make_3x3_y();
        let mut kr = KronReducer::new(3, y);
        // Eliminate bus 1 (middle bus of the ring)
        kr.eliminate_bus(1).expect("eliminate_bus should succeed");
        let red = kr.reduced_matrix();
        assert_eq!(red.len(), 2, "Should have 2 buses left after eliminating 1");
        assert_eq!(red[0].len(), 2);
        // Off-diagonal should be non-zero (coupling through eliminated bus)
        let (g01, b01) = red[0][1];
        assert!(
            g01.abs() > 1e-9 || b01.abs() > 1e-9,
            "Off-diagonal should be non-zero: ({g01}, {b01})"
        );
    }

    #[test]
    fn test_kron_reduce_passive_node() {
        // A passive (zero-injection) node in a 3-bus network is eliminated.
        // Buses 0 and 2 retain; bus 1 is passive.
        let y = make_3x3_y();
        let mut kr = KronReducer::new(3, y);
        kr.eliminate_bus(1).expect("eliminate passive node");
        let red = kr.reduced_matrix();
        // Reduced to 2-bus network
        assert_eq!(red.len(), 2);
        // Active buses should still be 0 and 2
        assert_eq!(kr.active_buses, vec![0, 2]);
    }

    #[test]
    fn test_kron_reduced_matrix_size() {
        let y = make_3x3_y();
        let mut kr = KronReducer::new(3, y);
        kr.eliminate_buses(&[1, 2]).expect("eliminate two buses");
        let red = kr.reduced_matrix();
        assert_eq!(red.len(), 1, "Should be 1×1 after eliminating 2 of 3 buses");
        assert_eq!(red[0].len(), 1);
    }

    // ── Complex arithmetic tests ─────────────────────────────────────────────

    #[test]
    fn test_complex_multiplication() {
        // (1+2j) * (3+4j) = 3+4j+6j+8j² = (3-8)+(4+6)j = -5+10j
        let result = cmul((1.0, 2.0), (3.0, 4.0));
        assert!((result.0 - (-5.0)).abs() < 1e-12, "Real part: {}", result.0);
        assert!((result.1 - 10.0).abs() < 1e-12, "Imag part: {}", result.1);
    }

    #[test]
    fn test_complex_division() {
        // (1+2j) / (1+j) = ((1+2)+(2-1)j) / (1+1) = (3+j) / 2 = 1.5+0.5j
        let result = cdiv((1.0, 2.0), (1.0, 1.0));
        assert!((result.0 - 1.5).abs() < 1e-12, "Real: {}", result.0);
        assert!((result.1 - 0.5).abs() < 1e-12, "Imag: {}", result.1);
    }

    #[test]
    fn test_complex_inverse() {
        // 1/(3+4j) = (3-4j)/25 = 0.12 - 0.16j
        let (re, im) = cinv(3.0, 4.0);
        assert!((re - 0.12).abs() < 1e-12, "Re(inv): {re}");
        assert!((im - (-0.16)).abs() < 1e-12, "Im(inv): {im}");
        // Exposed API
        let (re2, im2) = WardEquivalent::complex_inv(3.0, 4.0);
        assert!((re2 - 0.12).abs() < 1e-12);
        assert!((im2 - (-0.16)).abs() < 1e-12);
    }

    #[test]
    fn test_thevenin_impedance_simple() {
        // 2-bus system: Y = [[y, -y],[-y, y]] with y = 0 + j10
        // Z = Y^{-1} is singular for lossless 2-bus (floating reference);
        // use a shunt to make it invertible.
        // Y_diag = j10 + j1 (shunt), off = -j10
        let y = vec![
            vec![(0.0, 11.0), (0.0, -10.0)],
            vec![(0.0, -10.0), (0.0, 11.0)],
        ];
        let kr = KronReducer::new(2, y);
        let z_th = kr
            .thevenin_impedance(0, 1)
            .expect("Thévenin impedance should succeed");
        // Z_th = Z_00 + Z_11 - 2*Z_01; should be non-zero
        let z_mag = (z_th.0 * z_th.0 + z_th.1 * z_th.1).sqrt();
        assert!(
            z_mag > 1e-6,
            "Thévenin impedance should be non-zero: {z_mag}"
        );
    }

    // ── Ward equivalent tests ────────────────────────────────────────────────

    fn make_ward_config_4bus() -> WardEquivalentConfig {
        // 4-bus: 0,1 internal; 2 boundary; 3 external
        WardEquivalentConfig {
            internal_buses: vec![0, 1],
            external_buses: vec![3],
            boundary_buses: vec![2],
            include_load_transfer: true,
            base_mva: 100.0,
        }
    }

    fn make_4bus_y() -> Vec<Vec<(f64, f64)>> {
        // Simple 4-bus Y-bus (mostly imaginary): each branch y = -j5, shunts j15
        vec![
            vec![(0.0, 15.0), (0.0, -5.0), (0.0, -5.0), (0.0, -5.0)],
            vec![(0.0, -5.0), (0.0, 15.0), (0.0, -5.0), (0.0, -5.0)],
            vec![(0.0, -5.0), (0.0, -5.0), (0.0, 15.0), (0.0, -5.0)],
            vec![(0.0, -5.0), (0.0, -5.0), (0.0, -5.0), (0.0, 15.0)],
        ]
    }

    #[test]
    fn test_ward_config_creation() {
        let config = make_ward_config_4bus();
        assert_eq!(config.internal_buses, vec![0, 1]);
        assert_eq!(config.external_buses, vec![3]);
        assert_eq!(config.boundary_buses, vec![2]);
        assert!((config.base_mva - 100.0).abs() < 1e-9);
    }

    #[test]
    fn test_ward_partition() {
        let config = make_ward_config_4bus();
        let y = make_4bus_y();
        let ward = WardEquivalent::new(4, y, config);
        // Extract sub-matrices and verify shapes
        let y_ee =
            ward.extract_submatrix_gb(&ward.config.external_buses, &ward.config.external_buses);
        let y_bi =
            ward.extract_submatrix_gb(&ward.config.boundary_buses, &ward.config.internal_buses);
        assert_eq!(y_ee.len(), 1, "Y_ee should be 1×1");
        assert_eq!(y_bi.len(), 1, "Y_bi should be 1 row");
        assert_eq!(y_bi[0].len(), 2, "Y_bi should have 2 cols (internal buses)");
    }

    #[test]
    fn test_ward_equivalent_1bus_external() {
        let config = make_ward_config_4bus();
        let y = make_4bus_y();
        let ward = WardEquivalent::new(4, y, config);
        let p_ext = [50.0]; // 50 MW load at external bus
        let q_ext = [20.0]; // 20 MVAr
        let result = ward
            .compute(&p_ext, &q_ext)
            .expect("Ward compute should succeed");
        // Equivalent has 3 buses (2 internal + 1 boundary)
        assert_eq!(result.n_buses_equivalent, 3);
        assert_eq!(result.n_buses_reduced, 4);
        assert_eq!(result.y_ward.len(), 3);
        assert_eq!(result.y_ward[0].len(), 3);
    }

    #[test]
    fn test_ward_equivalent_2bus_external() {
        // 5-bus: 0 internal, 1 boundary, 2,3 external, 4 internal
        let config = WardEquivalentConfig {
            internal_buses: vec![0, 4],
            external_buses: vec![2, 3],
            boundary_buses: vec![1],
            include_load_transfer: true,
            base_mva: 100.0,
        };
        // Build 5-bus Y-bus (fully connected, each branch -j2, diagonal j8)
        let y: Vec<Vec<(f64, f64)>> = (0..5)
            .map(|i| {
                (0..5)
                    .map(|j| if i == j { (0.0, 8.0) } else { (0.0, -2.0) })
                    .collect()
            })
            .collect();
        let ward = WardEquivalent::new(5, y, config);
        let p_ext = [30.0, 20.0];
        let q_ext = [10.0, 5.0];
        let result = ward.compute(&p_ext, &q_ext).expect("Ward 2-external");
        // 2 internal + 1 boundary = 3 buses in equivalent
        assert_eq!(result.n_buses_equivalent, 3);
        assert_eq!(result.y_ward.len(), 3);
    }

    #[test]
    fn test_ward_injection_transfer() {
        let config = make_ward_config_4bus();
        let y = make_4bus_y();
        let ward = WardEquivalent::new(4, y, config);
        let p_ext = [100.0];
        let q_ext = [50.0];
        let result = ward.compute(&p_ext, &q_ext).expect("Ward injection");
        // Boundary injections should be non-trivially transferred
        assert_eq!(result.p_ward_injections.len(), 1);
        assert_eq!(result.q_ward_injections.len(), 1);
        // The injection magnitudes should reflect the transferred load
        // (absolute values may differ depending on Y_be, but should be finite)
        assert!(result.p_ward_injections[0].is_finite());
        assert!(result.q_ward_injections[0].is_finite());
    }

    #[test]
    fn test_ward_reduction_ratio() {
        let config = make_ward_config_4bus();
        let y = make_4bus_y();
        let ward = WardEquivalent::new(4, y, config);
        let result = ward.compute(&[0.0], &[0.0]).expect("Ward ratio");
        // 4 → 3 buses: ratio = 1 - 3/4 = 0.25
        assert!(
            (result.reduction_ratio - 0.25).abs() < 1e-9,
            "Reduction ratio: {}",
            result.reduction_ratio
        );
    }

    #[test]
    fn test_extended_ward_voltage_correction() {
        let config = make_ward_config_4bus();
        let y = make_4bus_y();
        let ward = WardEquivalent::new(4, y, config);
        let p_ext = [50.0];
        let q_ext = [20.0];
        let v_ext = [0.95]; // external bus at 0.95 pu
        let result = ward
            .compute_extended(&p_ext, &q_ext, &v_ext)
            .expect("Extended Ward");
        // Voltage correction should reflect the −0.05 pu deviation
        assert_eq!(result.voltage_correction.len(), 1);
        assert!(
            result.voltage_correction[0] < 0.0,
            "Correction should be negative for under-voltage: {}",
            result.voltage_correction[0]
        );
        // Q compensation should be non-zero
        assert_eq!(result.q_compensation_mvar.len(), 1);
    }

    // ── CoherencyAnalyzer tests ──────────────────────────────────────────────

    #[test]
    fn test_coherency_analyzer_identical_trajectories() {
        // Two generators with identical angle trajectories → same group
        let traj = vec![
            vec![0.0_f64, 0.1, 0.2, 0.15, 0.05, 0.0],
            vec![0.0_f64, 0.1, 0.2, 0.15, 0.05, 0.0],
        ];
        let analyzer = CoherencyAnalyzer::new(vec![0, 1], 0.95);
        let groups = analyzer.identify_groups(&traj, 0.02);
        // Both should be in the same group
        assert_eq!(groups.len(), 1, "Identical trajectories → 1 group");
        assert_eq!(groups[0].buses.len(), 2);
    }

    #[test]
    fn test_coherency_analyzer_opposite_trajectories() {
        // Two generators with opposite oscillations → different groups
        let base: Vec<f64> = (0..100).map(|k| (k as f64 * 0.1).sin()).collect();
        let opp: Vec<f64> = base.iter().map(|&v| -v).collect();
        let traj = vec![base, opp];
        let analyzer = CoherencyAnalyzer::new(vec![0, 1], 0.95);
        let groups = analyzer.identify_groups(&traj, 0.01);
        // Should form 2 separate groups (correlation = −1 < 0.95)
        assert_eq!(groups.len(), 2, "Opposite trajectories → 2 groups");
    }

    #[test]
    fn test_pearson_correlation_perfect() {
        // Perfect positive correlation: y = 2x + 3
        let x: Vec<f64> = (0..50).map(|k| k as f64).collect();
        let y: Vec<f64> = x.iter().map(|&v| 2.0 * v + 3.0).collect();
        let c = CoherencyAnalyzer::pearson_correlation(&x, &y);
        assert!(
            (c - 1.0).abs() < 1e-10,
            "Perfect correlation should be 1.0, got {c}"
        );
    }

    #[test]
    fn test_pearson_correlation_none() {
        // Zero correlation: constant y
        let x: Vec<f64> = (0..50).map(|k| k as f64).collect();
        let y: Vec<f64> = vec![5.0_f64; 50];
        let c = CoherencyAnalyzer::pearson_correlation(&x, &y);
        assert!(
            c.abs() < 1e-10,
            "Zero-variance y → correlation 0.0, got {c}"
        );
    }

    #[test]
    fn test_aggregate_inertia_equal_capacity() {
        // Equal capacities: H_agg = mean of inertia constants
        let h = vec![4.0, 6.0, 5.0];
        let s = vec![100.0, 100.0, 100.0];
        let idx = vec![0, 1, 2];
        let h_agg = CoherencyAnalyzer::aggregate_inertia(&h, &s, &idx);
        assert!(
            (h_agg - 5.0).abs() < 1e-10,
            "H_agg should be 5.0 for equal capacities, got {h_agg}"
        );
    }
}
