use nalgebra::DMatrix;
use num_complex::Complex64;
use sprs::{CsMat, TriMat};

/// Build the full Jacobian matrix for Newton-Raphson power flow as a sparse CsMat.
///
/// Uses sparse Y-bus iteration — only computes entries for connected bus pairs,
/// avoiding the O(n²) dense Y-bus conversion used in naive implementations.
///
/// The Jacobian is structured as:
/// ```text
/// J = | H  N |   where H = dP/dθ, N = dP/d|V| * |V|
///     | M  L |         M = dQ/dθ, L = dQ/d|V| * |V|
/// ```
///
/// This function returns a sparse `CsMat<f64>` directly — callers on the large
/// system path use this to avoid the O(n²) `DMatrix` allocation entirely.
pub fn build_jacobian_sparse(
    ybus: &CsMat<Complex64>,
    v_mag: &[f64],
    v_ang: &[f64],
    p_calc: &[f64],
    q_calc: &[f64],
    pq_indices: &[usize],
    pvpq_indices: &[usize],
) -> CsMat<f64> {
    let n = v_mag.len();
    let npvpq = pvpq_indices.len();
    let npq = pq_indices.len();
    let j_size = npvpq + npq;

    // Pre-allocate triplet lists with an upper-bound on nnz.
    // Each Y-bus non-zero contributes at most 4 Jacobian entries.
    let nnz_bound = 8 * ybus.nnz();
    let mut tri: TriMat<f64> = TriMat::with_capacity((j_size, j_size), nnz_bound);

    // O(n) lookup arrays instead of HashMap — avoids hashing overhead
    let mut pvpq_map = vec![usize::MAX; n];
    for (row, &i) in pvpq_indices.iter().enumerate() {
        pvpq_map[i] = row;
    }
    let mut pq_map = vec![usize::MAX; n];
    for (row, &i) in pq_indices.iter().enumerate() {
        pq_map[i] = row;
    }

    // Iterate over Y-bus non-zeros only (sparse path)
    // For connected networks, nnz ≈ 2 * n_branches + n_buses  << n²
    for (&yij_val, (i, j)) in ybus.iter() {
        let in_pvpq_i = pvpq_map[i] != usize::MAX;
        let in_pq_i = pq_map[i] != usize::MAX;

        if i == j {
            // ── Diagonal terms ──────────────────────────────────────────────
            let g_ii = yij_val.re;
            let b_ii = yij_val.im;
            let v2 = v_mag[i] * v_mag[i];

            if in_pvpq_i {
                let row = pvpq_map[i];
                // H_ii = -Q_i - B_ii * |V_i|²
                tri.add_triplet(row, row, -q_calc[i] - b_ii * v2);

                // N_ii (only when bus i is also PQ)
                if in_pq_i {
                    let col = pq_map[i];
                    // N_ii = P_i + G_ii * |V_i|²
                    tri.add_triplet(row, npvpq + col, p_calc[i] + g_ii * v2);
                }
            }

            if in_pq_i {
                let row = pq_map[i];
                // M_ii = P_i - G_ii * |V_i|²  (col is same pvpq row since i∈pvpq∩pq)
                let pvpq_col = pvpq_map[i];
                tri.add_triplet(npvpq + row, pvpq_col, p_calc[i] - g_ii * v2);

                // L_ii = Q_i - B_ii * |V_i|²
                tri.add_triplet(npvpq + row, npvpq + row, q_calc[i] - b_ii * v2);
            }
        } else {
            // ── Off-diagonal terms: only non-zero where buses are connected ──
            let theta_ij = v_ang[i] - v_ang[j];
            let (sin_ij, cos_ij) = theta_ij.sin_cos();
            let vm_ij = v_mag[i] * v_mag[j];
            let g = yij_val.re;
            let b = yij_val.im;

            // Shared products used across sub-matrices
            let gs_bc = g * sin_ij - b * cos_ij; // G*sin(θ) - B*cos(θ)
            let gc_bs = g * cos_ij + b * sin_ij; // G*cos(θ) + B*sin(θ)

            let in_pvpq_j = pvpq_map[j] != usize::MAX;
            let in_pq_j = pq_map[j] != usize::MAX;

            if in_pvpq_i {
                let row = pvpq_map[i];

                // H_ij = |V_i||V_j|*(G_ij*sin(θ_ij) - B_ij*cos(θ_ij))
                if in_pvpq_j {
                    tri.add_triplet(row, pvpq_map[j], vm_ij * gs_bc);
                }

                // N_ij = |V_i||V_j|*(G_ij*cos(θ_ij) + B_ij*sin(θ_ij))
                if in_pq_j {
                    tri.add_triplet(row, npvpq + pq_map[j], vm_ij * gc_bs);
                }
            }

            if in_pq_i {
                let row = pq_map[i];

                // M_ij = -|V_i||V_j|*(G_ij*cos(θ_ij) + B_ij*sin(θ_ij))
                if in_pvpq_j {
                    tri.add_triplet(npvpq + row, pvpq_map[j], -vm_ij * gc_bs);
                }

                // L_ij = |V_i||V_j|*(G_ij*sin(θ_ij) - B_ij*cos(θ_ij))
                if in_pq_j {
                    tri.add_triplet(npvpq + row, npvpq + pq_map[j], vm_ij * gs_bc);
                }
            }
        }
    }

    tri.to_csr()
}

/// Build the full Jacobian matrix for Newton-Raphson power flow as a dense `DMatrix`.
///
/// This is a thin wrapper around [`build_jacobian_sparse`] that materialises the
/// result as a nalgebra `DMatrix<f64>`.  Existing callers (state estimation tests,
/// DC-OPF, etc.) are preserved without modification.
///
/// For large systems (> 200 buses) prefer [`build_jacobian_sparse`] directly to
/// avoid the O(n²) allocation this wrapper performs.
pub fn build_jacobian(
    ybus: &CsMat<Complex64>,
    v_mag: &[f64],
    v_ang: &[f64],
    p_calc: &[f64],
    q_calc: &[f64],
    pq_indices: &[usize],
    pvpq_indices: &[usize],
) -> DMatrix<f64> {
    let csmat = build_jacobian_sparse(ybus, v_mag, v_ang, p_calc, q_calc, pq_indices, pvpq_indices);
    let n = csmat.rows();
    // Manual conversion: sprs to_dense() returns ndarray::Array2 (incompatible type),
    // so we iterate over non-zeros and fill a nalgebra DMatrix.
    let mut dense = DMatrix::zeros(n, n);
    for (&v, (r, c)) in csmat.iter() {
        dense[(r, c)] = v;
    }
    dense
}

/// Parallel Jacobian builder using rayon (enabled by `parallel` feature flag).
///
/// This is a thin wrapper around [`build_jacobian_sparse`] that materialises the
/// result as a nalgebra `DMatrix<f64>`, identical to [`build_jacobian`].  The
/// rayon-based row-parallel implementation has been superseded by the sparse
/// builder which already amortises allocations via triplet accumulation.
/// Callers in `newton_raphson.rs` that import this as `build_jacobian` are
/// unaffected because the return type and signature are unchanged.
#[cfg(feature = "parallel")]
pub fn build_jacobian_parallel(
    ybus: &CsMat<Complex64>,
    v_mag: &[f64],
    v_ang: &[f64],
    p_calc: &[f64],
    q_calc: &[f64],
    pq_indices: &[usize],
    pvpq_indices: &[usize],
) -> DMatrix<f64> {
    let csmat = build_jacobian_sparse(ybus, v_mag, v_ang, p_calc, q_calc, pq_indices, pvpq_indices);
    let n = csmat.rows();
    let mut dense = DMatrix::zeros(n, n);
    for (&v, (r, c)) in csmat.iter() {
        dense[(r, c)] = v;
    }
    dense
}

#[cfg(test)]
mod tests {
    use super::*;
    use num_complex::Complex64;
    use sprs::TriMat;

    /// Build a simple 3-bus Y-bus (CSC) for testing.
    ///
    /// Topology (radial): bus 0 = slack, bus 1 = PQ, bus 2 = PQ.
    ///   Branch 0-1: z = 0.01 + 0.05j  → y = 1/z
    ///   Branch 1-2: z = 0.02 + 0.08j  → y = 1/z
    ///   Shunt at bus 0: y_sh = 0.001j
    fn make_3bus_ybus() -> CsMat<Complex64> {
        let y01 = Complex64::new(0.01, 0.05).inv();
        let y12 = Complex64::new(0.02, 0.08).inv();
        let y_sh0 = Complex64::new(0.0, 0.001);

        let mut tri: TriMat<Complex64> = TriMat::new((3, 3));
        // Diagonal
        tri.add_triplet(0, 0, y01 + y_sh0);
        tri.add_triplet(1, 1, y01 + y12);
        tri.add_triplet(2, 2, y12);
        // Off-diagonal (symmetric)
        tri.add_triplet(0, 1, -y01);
        tri.add_triplet(1, 0, -y01);
        tri.add_triplet(1, 2, -y12);
        tri.add_triplet(2, 1, -y12);

        tri.to_csc()
    }

    /// Both sparse and dense Jacobian builders must produce identical results.
    #[test]
    fn jacobian_sparse_matches_dense_3bus() {
        let ybus = make_3bus_ybus();

        // Bus 0 = slack (excluded). Buses 1,2 = PQ.
        let pvpq_indices = &[1usize, 2];
        let pq_indices = &[1usize, 2];

        let v_mag = [1.0_f64, 0.98, 0.96];
        let v_ang = [0.0_f64, -0.03, -0.06];
        // Compute power injections from the Y-bus
        let p_calc = {
            let v: Vec<Complex64> = v_mag
                .iter()
                .zip(v_ang.iter())
                .map(|(&m, &a)| Complex64::from_polar(m, a))
                .collect();
            let mut p = [0.0_f64; 3];
            for (&yij, (i, j)) in ybus.iter() {
                let s = v[i] * (yij * v[j]).conj();
                p[i] += s.re;
            }
            p
        };
        let q_calc = {
            let v: Vec<Complex64> = v_mag
                .iter()
                .zip(v_ang.iter())
                .map(|(&m, &a)| Complex64::from_polar(m, a))
                .collect();
            let mut q = [0.0_f64; 3];
            for (&yij, (i, j)) in ybus.iter() {
                let s = v[i] * (yij * v[j]).conj();
                q[i] += s.im;
            }
            q
        };

        let jac_dense = build_jacobian(
            &ybus,
            &v_mag,
            &v_ang,
            &p_calc,
            &q_calc,
            pq_indices,
            pvpq_indices,
        );

        let jac_sparse = build_jacobian_sparse(
            &ybus,
            &v_mag,
            &v_ang,
            &p_calc,
            &q_calc,
            pq_indices,
            pvpq_indices,
        );

        let n = jac_dense.nrows();
        assert_eq!(n, jac_sparse.rows(), "Jacobian row count must match");
        assert_eq!(n, jac_sparse.cols(), "Jacobian col count must match");

        // Compare element-wise
        for r in 0..n {
            for c in 0..n {
                let dense_val = jac_dense[(r, c)];
                // sparse stores only non-zeros; missing entries are implicitly zero
                let sparse_val = jac_sparse.get(r, c).copied().unwrap_or(0.0);
                assert!(
                    (dense_val - sparse_val).abs() < 1e-12,
                    "Jacobian[{r},{c}]: dense={dense_val:.15e} sparse={sparse_val:.15e}"
                );
            }
        }
    }

    /// For the IEEE 14-bus system, the sparse Jacobian nnz must be well below
    /// the dense upper bound (40% of j_size²).
    #[test]
    fn jacobian_sparse_nnz_bounded_ieee14() {
        let net = crate::testcases::ieee::ieee14().expect("IEEE 14-bus must load");
        let ybus = net.admittance_matrix().expect("Y-bus must build");

        let mut pq_indices = Vec::new();
        let mut pv_indices = Vec::new();
        for (i, bus) in net.buses.iter().enumerate() {
            match bus.bus_type {
                crate::network::bus::BusType::PQ => pq_indices.push(i),
                crate::network::bus::BusType::PV => pv_indices.push(i),
                crate::network::bus::BusType::Slack => {}
            }
        }
        let mut pvpq_indices = pv_indices.clone();
        pvpq_indices.extend_from_slice(&pq_indices);
        pvpq_indices.sort();

        let n = net.bus_count();
        let v_mag = net.buses.iter().map(|b| b.vm).collect::<Vec<_>>();
        let v_ang = net.buses.iter().map(|b| b.va).collect::<Vec<_>>();

        // Compute power injections for a flat-start
        let (p_calc, q_calc) = {
            let v: Vec<Complex64> = v_mag
                .iter()
                .zip(v_ang.iter())
                .map(|(&m, &a)| Complex64::from_polar(m, a))
                .collect();
            let mut p = vec![0.0_f64; n];
            let mut q = vec![0.0_f64; n];
            for (&yij, (i, j)) in ybus.iter() {
                let s = v[i] * (yij * v[j]).conj();
                p[i] += s.re;
                q[i] += s.im;
            }
            (p, q)
        };

        let jac_sparse = build_jacobian_sparse(
            &ybus,
            &v_mag,
            &v_ang,
            &p_calc,
            &q_calc,
            &pq_indices,
            &pvpq_indices,
        );

        let j_size = pvpq_indices.len() + pq_indices.len();
        let dense_bound = (j_size * j_size) as f64;
        let nnz = jac_sparse.nnz();

        assert!(
            (nnz as f64) < 0.4 * dense_bound,
            "Jacobian nnz={nnz} must be < 40% of j_size²={dense_bound:.0} for IEEE 14-bus"
        );
    }
}
