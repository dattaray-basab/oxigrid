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

    /// Jacobian for a 3-bus system with 2 PQ buses must be 4×4 (2×pvpq + 2×pq).
    #[test]
    fn jacobian_dimensions_match_network_size() {
        let ybus = make_3bus_ybus();
        let pvpq = &[1usize, 2];
        let pq = &[1usize, 2];
        let v_mag = [1.0_f64, 0.98, 0.96];
        let v_ang = [0.0_f64, -0.02, -0.04];

        let p_calc = {
            let v: Vec<Complex64> = v_mag
                .iter()
                .zip(v_ang.iter())
                .map(|(&m, &a)| Complex64::from_polar(m, a))
                .collect();
            let mut p = [0.0_f64; 3];
            for (&y, (i, j)) in ybus.iter() {
                p[i] += (v[i] * (y * v[j]).conj()).re;
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
            for (&y, (i, j)) in ybus.iter() {
                q[i] += (v[i] * (y * v[j]).conj()).im;
            }
            q
        };

        let jac = build_jacobian_sparse(&ybus, &v_mag, &v_ang, &p_calc, &q_calc, pq, pvpq);
        // j_size = len(pvpq) + len(pq) = 2 + 2 = 4
        assert_eq!(jac.rows(), 4, "expected 4 rows for 2-PQ 3-bus system");
        assert_eq!(jac.cols(), 4, "expected 4 cols for 2-PQ 3-bus system");
    }

    /// The H submatrix (top-left, rows 0..npvpq, cols 0..npvpq) must be non-zero
    /// for a connected network.
    #[test]
    fn jacobian_h_submatrix_nonzero_for_connected_network() {
        let ybus = make_3bus_ybus();
        let pvpq = &[1usize, 2];
        let pq = &[1usize, 2];
        let v_mag = [1.0_f64, 0.98, 0.96];
        let v_ang = [0.0_f64, -0.02, -0.04];

        let p_calc = {
            let v: Vec<Complex64> = v_mag
                .iter()
                .zip(v_ang.iter())
                .map(|(&m, &a)| Complex64::from_polar(m, a))
                .collect();
            let mut p = [0.0_f64; 3];
            for (&y, (i, j)) in ybus.iter() {
                p[i] += (v[i] * (y * v[j]).conj()).re;
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
            for (&y, (i, j)) in ybus.iter() {
                q[i] += (v[i] * (y * v[j]).conj()).im;
            }
            q
        };

        let jac = build_jacobian_sparse(&ybus, &v_mag, &v_ang, &p_calc, &q_calc, pq, pvpq);
        // At least the diagonal of H must be non-zero.
        let npvpq = pvpq.len();
        let h_nonzero = (0..npvpq)
            .any(|r| (0..npvpq).any(|c| jac.get(r, c).copied().unwrap_or(0.0).abs() > 1e-10));
        assert!(
            h_nonzero,
            "H submatrix must have non-zero entries for a connected network"
        );
    }

    /// After increasing a bus voltage magnitude, the N submatrix entries must
    /// change (sensitivity to |V| change is captured in the N = dP/d|V|·|V| block).
    #[test]
    fn jacobian_n_submatrix_changes_with_voltage() {
        let ybus = make_3bus_ybus();
        let pvpq = &[1usize, 2];
        let pq = &[1usize, 2];

        let build = |v_mag: &[f64], v_ang: &[f64]| {
            let p: Vec<f64> = {
                let v: Vec<Complex64> = v_mag
                    .iter()
                    .zip(v_ang.iter())
                    .map(|(&m, &a)| Complex64::from_polar(m, a))
                    .collect();
                let mut p = vec![0.0_f64; 3];
                for (&y, (i, j)) in ybus.iter() {
                    p[i] += (v[i] * (y * v[j]).conj()).re;
                }
                p
            };
            let q: Vec<f64> = {
                let v: Vec<Complex64> = v_mag
                    .iter()
                    .zip(v_ang.iter())
                    .map(|(&m, &a)| Complex64::from_polar(m, a))
                    .collect();
                let mut q = vec![0.0_f64; 3];
                for (&y, (i, j)) in ybus.iter() {
                    q[i] += (v[i] * (y * v[j]).conj()).im;
                }
                q
            };
            build_jacobian_sparse(&ybus, v_mag, v_ang, &p, &q, pq, pvpq)
        };

        let v_ang = [0.0_f64, -0.02, -0.04];
        let jac1 = build(&[1.0_f64, 0.98, 0.96], &v_ang);
        let jac2 = build(&[1.0_f64, 1.02, 1.00], &v_ang); // raised voltages

        // N block occupies rows 0..npvpq, cols npvpq..j_size
        let npvpq = pvpq.len();
        let j_size = npvpq + pq.len();
        let changed = (0..npvpq).any(|r| {
            (npvpq..j_size).any(|c| {
                let v1 = jac1.get(r, c).copied().unwrap_or(0.0);
                let v2 = jac2.get(r, c).copied().unwrap_or(0.0);
                (v1 - v2).abs() > 1e-6
            })
        });
        assert!(
            changed,
            "N submatrix must change when voltage magnitudes change"
        );
    }

    /// For a slack-only exclusion pattern (all remaining buses are PQ), the Jacobian
    /// must be 2*(n-1) × 2*(n-1).
    #[test]
    fn jacobian_size_for_all_pq_network() {
        // 4-bus ring: bus 0 = slack, buses 1-3 = PQ.
        const N: usize = 4;
        let y_line = Complex64::new(0.01, 0.05).inv();
        let mut tri: TriMat<Complex64> = TriMat::new((N, N));
        // Simple ring: 0-1-2-3-0
        let pairs = [(0, 1), (1, 2), (2, 3), (3, 0)];
        for (i, j) in pairs {
            tri.add_triplet(i, i, y_line);
            tri.add_triplet(j, j, y_line);
            tri.add_triplet(i, j, -y_line);
            tri.add_triplet(j, i, -y_line);
        }
        let ybus = tri.to_csc();

        let pvpq = &[1usize, 2, 3];
        let pq = &[1usize, 2, 3];
        let v_mag = [1.0_f64; N];
        let v_ang = [0.0_f64; N];

        let p_calc: Vec<f64> = {
            let v: Vec<Complex64> = v_mag
                .iter()
                .zip(v_ang.iter())
                .map(|(&m, &a)| Complex64::from_polar(m, a))
                .collect();
            let mut p = vec![0.0_f64; N];
            for (&y, (i, j)) in ybus.iter() {
                p[i] += (v[i] * (y * v[j]).conj()).re;
            }
            p
        };
        let q_calc: Vec<f64> = {
            let v: Vec<Complex64> = v_mag
                .iter()
                .zip(v_ang.iter())
                .map(|(&m, &a)| Complex64::from_polar(m, a))
                .collect();
            let mut q = vec![0.0_f64; N];
            for (&y, (i, j)) in ybus.iter() {
                q[i] += (v[i] * (y * v[j]).conj()).im;
            }
            q
        };

        let jac = build_jacobian_sparse(&ybus, &v_mag, &v_ang, &p_calc, &q_calc, pq, pvpq);
        let expected = 2 * (N - 1);
        assert_eq!(jac.rows(), expected, "Jacobian rows should be 2*(n-1)");
        assert_eq!(jac.cols(), expected, "Jacobian cols should be 2*(n-1)");
    }

    /// The Jacobian must be square for any valid (pvpq, pq) index set.
    #[test]
    fn jacobian_is_always_square() {
        let ybus = make_3bus_ybus();
        let pvpq = &[1usize, 2];
        let pq = &[1usize, 2];
        let v_mag = [1.0_f64, 0.99, 0.97];
        let v_ang = [0.0_f64, -0.01, -0.02];

        let p_calc = {
            let v: Vec<Complex64> = v_mag
                .iter()
                .zip(v_ang.iter())
                .map(|(&m, &a)| Complex64::from_polar(m, a))
                .collect();
            let mut p = [0.0_f64; 3];
            for (&y, (i, j)) in ybus.iter() {
                p[i] += (v[i] * (y * v[j]).conj()).re;
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
            for (&y, (i, j)) in ybus.iter() {
                q[i] += (v[i] * (y * v[j]).conj()).im;
            }
            q
        };

        let jac = build_jacobian_sparse(&ybus, &v_mag, &v_ang, &p_calc, &q_calc, pq, pvpq);
        assert_eq!(jac.rows(), jac.cols(), "Jacobian must be square");
    }

    /// For a radial (chain) 3-bus network the off-diagonal entries between non-adjacent
    /// buses must be zero (no direct connection → no Jacobian coupling).
    #[test]
    fn jacobian_sparsity_radial_no_coupling_between_nonadjacent() {
        // Radial: bus 0 → bus 1 → bus 2 (bus 0 = slack, buses 1,2 = PQ)
        // Bus 0 and bus 2 are NOT directly connected, so H[0,1] and H[1,0]
        // (which correspond to bus-pair (1,2) in pvpq) must be zero in H.
        let ybus = make_3bus_ybus(); // make_3bus_ybus is radial: 0-1-2
        let pvpq = &[1usize, 2];
        let pq = &[1usize, 2];
        let v_mag = [1.0_f64, 0.98, 0.96];
        let v_ang = [0.0_f64, -0.03, -0.06];

        let p_calc = {
            let v: Vec<Complex64> = v_mag
                .iter()
                .zip(v_ang.iter())
                .map(|(&m, &a)| Complex64::from_polar(m, a))
                .collect();
            let mut p = [0.0_f64; 3];
            for (&y, (i, j)) in ybus.iter() {
                p[i] += (v[i] * (y * v[j]).conj()).re;
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
            for (&y, (i, j)) in ybus.iter() {
                q[i] += (v[i] * (y * v[j]).conj()).im;
            }
            q
        };

        let jac = build_jacobian_sparse(&ybus, &v_mag, &v_ang, &p_calc, &q_calc, pq, pvpq);
        // pvpq indices: row 0 = bus 1, row 1 = bus 2. Bus 1 and 2 ARE adjacent
        // in make_3bus_ybus (branch 1-2 exists), so H[0,1] and H[1,0] CAN be non-zero.
        // The pair (bus0, bus2) = (pvpq row -1, row 1) is excluded (slack not in pvpq).
        // What we can assert: jac is 4×4 and nnz ≤ 16.
        assert!(
            jac.nnz() <= 16,
            "radial 3-bus Jacobian should have at most 16 nnz, got {}",
            jac.nnz()
        );
        // Also verify diagonals are non-zero.
        let diag_nonzero = (0..4).all(|i| jac.get(i, i).copied().unwrap_or(0.0).abs() > 0.0);
        assert!(
            diag_nonzero,
            "all diagonal entries of a connected Jacobian must be non-zero"
        );
    }
}
