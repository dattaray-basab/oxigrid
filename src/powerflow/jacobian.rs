use nalgebra::DMatrix;
use num_complex::Complex64;
use sprs::CsMat;

/// Build the full Jacobian matrix for Newton-Raphson power flow.
///
/// Uses sparse Y-bus iteration — only computes entries for connected bus pairs,
/// avoiding the O(n²) dense Y-bus conversion used in naive implementations.
///
/// The Jacobian is structured as:
/// ```text
/// J = | H  N |   where H = dP/dθ, N = dP/d|V| * |V|
///     | M  L |         M = dQ/dθ, L = dQ/d|V| * |V|
/// ```
pub fn build_jacobian(
    ybus: &CsMat<Complex64>,
    v_mag: &[f64],
    v_ang: &[f64],
    p_calc: &[f64],
    q_calc: &[f64],
    pq_indices: &[usize],
    pvpq_indices: &[usize],
) -> DMatrix<f64> {
    let n = v_mag.len();
    let npvpq = pvpq_indices.len();
    let npq = pq_indices.len();
    let j_size = npvpq + npq;

    let mut jac = DMatrix::zeros(j_size, j_size);

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
                jac[(row, row)] = -q_calc[i] - b_ii * v2;

                // N_ii (only when bus i is also PQ)
                if in_pq_i {
                    let col = pq_map[i];
                    // N_ii = P_i + G_ii * |V_i|²
                    jac[(row, npvpq + col)] = p_calc[i] + g_ii * v2;
                }
            }

            if in_pq_i {
                let row = pq_map[i];
                // M_ii = P_i - G_ii * |V_i|²  (col is same pvpq row since i∈pvpq∩pq)
                let pvpq_col = pvpq_map[i];
                jac[(npvpq + row, pvpq_col)] = p_calc[i] - g_ii * v2;

                // L_ii = Q_i - B_ii * |V_i|²
                jac[(npvpq + row, npvpq + row)] = q_calc[i] - b_ii * v2;
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
                    jac[(row, pvpq_map[j])] = vm_ij * gs_bc;
                }

                // N_ij = |V_i||V_j|*(G_ij*cos(θ_ij) + B_ij*sin(θ_ij))
                if in_pq_j {
                    jac[(row, npvpq + pq_map[j])] = vm_ij * gc_bs;
                }
            }

            if in_pq_i {
                let row = pq_map[i];

                // M_ij = -|V_i||V_j|*(G_ij*cos(θ_ij) + B_ij*sin(θ_ij))
                if in_pvpq_j {
                    jac[(npvpq + row, pvpq_map[j])] = -vm_ij * gc_bs;
                }

                // L_ij = |V_i||V_j|*(G_ij*sin(θ_ij) - B_ij*cos(θ_ij))
                if in_pq_j {
                    jac[(npvpq + row, npvpq + pq_map[j])] = vm_ij * gs_bc;
                }
            }
        }
    }

    jac
}

/// Parallel Jacobian builder using rayon (enabled by `parallel` feature flag).
///
/// Converts Y-bus to CSR for row-parallel access, then dispatches each
/// Jacobian row to a rayon thread. Useful for systems with > 200 buses.
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
    use rayon::prelude::*;

    let n = v_mag.len();
    let npvpq = pvpq_indices.len();
    let npq = pq_indices.len();
    let j_size = npvpq + npq;

    let mut pvpq_map = vec![usize::MAX; n];
    for (row, &i) in pvpq_indices.iter().enumerate() {
        pvpq_map[i] = row;
    }
    let mut pq_map = vec![usize::MAX; n];
    for (row, &i) in pq_indices.iter().enumerate() {
        pq_map[i] = row;
    }

    let ybus_csr = ybus.to_csr();

    // Compute each Jacobian row in parallel
    // Row layout: first npvpq rows = dP (pvpq buses), next npq rows = dQ (pq buses)
    let rows: Vec<Vec<f64>> = (0..j_size)
        .into_par_iter()
        .map(|jac_row| {
            let mut row_data = vec![0.0f64; j_size];

            // Determine which physical bus this Jacobian row corresponds to
            let (bus_i, is_p_row) = if jac_row < npvpq {
                (pvpq_indices[jac_row], true)
            } else {
                (pq_indices[jac_row - npvpq], false)
            };

            let in_pq_i = pq_map[bus_i] != usize::MAX;
            let v2 = v_mag[bus_i] * v_mag[bus_i];

            // Iterate over Y-bus row bus_i
            for (j, &yij_val) in ybus_csr
                .outer_view(bus_i)
                .expect("CSR outer view valid")
                .iter()
            {
                let g = yij_val.re;
                let b = yij_val.im;

                if bus_i == j {
                    if is_p_row {
                        // H_ii diagonal
                        row_data[jac_row] = -q_calc[bus_i] - b * v2;
                        // N_ii (if PQ bus)
                        if in_pq_i {
                            row_data[npvpq + pq_map[bus_i]] = p_calc[bus_i] + g * v2;
                        }
                    } else {
                        // M_ii (col = pvpq_map[bus_i])
                        row_data[pvpq_map[bus_i]] = p_calc[bus_i] - g * v2;
                        // L_ii diagonal
                        row_data[npvpq + pac_map_from_pq_row(jac_row, npvpq)] =
                            q_calc[bus_i] - b * v2;
                    }
                } else {
                    let theta_ij = v_ang[bus_i] - v_ang[j];
                    let (sin_ij, cos_ij) = theta_ij.sin_cos();
                    let vm_ij = v_mag[bus_i] * v_mag[j];
                    let gs_bc = g * sin_ij - b * cos_ij;
                    let gc_bs = g * cos_ij + b * sin_ij;

                    let in_pvpq_j = pvpq_map[j] != usize::MAX;
                    let in_pq_j = pq_map[j] != usize::MAX;

                    if is_p_row {
                        if in_pvpq_j {
                            row_data[pvpq_map[j]] = vm_ij * gs_bc;
                        }
                        if in_pq_j {
                            row_data[npvpq + pq_map[j]] = vm_ij * gc_bs;
                        }
                    } else {
                        if in_pvpq_j {
                            row_data[pvpq_map[j]] = -vm_ij * gc_bs;
                        }
                        if in_pq_j {
                            row_data[npvpq + pq_map[j]] = vm_ij * gs_bc;
                        }
                    }
                }
            }

            row_data
        })
        .collect();

    let mut jac = DMatrix::zeros(j_size, j_size);
    for (jac_row, row_data) in rows.iter().enumerate() {
        for (col, &val) in row_data.iter().enumerate() {
            jac[(jac_row, col)] = val;
        }
    }
    jac
}

#[cfg(feature = "parallel")]
#[inline(always)]
fn pac_map_from_pq_row(jac_row: usize, npvpq: usize) -> usize {
    jac_row - npvpq
}
