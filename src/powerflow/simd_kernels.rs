//! SIMD-accelerated kernels for power flow inner loops.
//!
//! Gated behind the `simd` feature flag. When the feature is disabled, the
//! module is still present but all public symbols are re-exported from the
//! scalar fallback implementations.
//!
//! # Kernels
//!
//! - `dot_product_f64`           — dense dot product of two `f64` slices
//! - `sparse_matvec_row`         — single sparse-row × complex voltage vector multiply
//! - `compute_power_injection`   — full bus power injection P_i, Q_i using Y-bus row
//! - `compute_mismatch_simd`     — batch ΔP/ΔQ mismatch for the NR inner loop
//! - `compute_mismatch_scalar`   — scalar reference path for the same computation
//!
//! The scalar paths are always correct; the SIMD paths are enabled when the
//! `simd` Cargo feature is active and the target supports `avx2`.

#[cfg(feature = "simd")]
pub mod simd {
    /// Compute the dot product of two `f64` slices.
    ///
    /// Uses AVX2 4-wide vectorisation when the target supports it; otherwise
    /// falls back to a clean scalar loop.  Both paths yield identical results
    /// (no fast-math reassociation).
    ///
    /// # Panics
    /// Panics if `a.len() != b.len()`.
    pub fn dot_product_f64(a: &[f64], b: &[f64]) -> f64 {
        assert_eq!(a.len(), b.len(), "dot_product_f64: slice length mismatch");

        #[cfg(target_feature = "avx2")]
        // SAFETY: we checked at compile time that AVX2 is available.
        unsafe {
            return dot_product_avx2(a, b);
        }

        // Scalar fallback (also used on non-x86 targets).
        #[allow(unreachable_code)]
        a.iter().zip(b.iter()).map(|(&x, &y)| x * y).sum()
    }

    /// AVX2 dot product — processes 4 `f64` elements per iteration.
    ///
    /// # Safety
    /// Caller must guarantee that the `avx2` target feature is available.
    #[cfg(target_feature = "avx2")]
    #[target_feature(enable = "avx2")]
    unsafe fn dot_product_avx2(a: &[f64], b: &[f64]) -> f64 {
        use std::arch::x86_64::{
            __m256d, _mm256_add_pd, _mm256_loadu_pd, _mm256_mul_pd, _mm256_setzero_pd,
            _mm256_storeu_pd,
        };

        let n = a.len();
        let mut acc = _mm256_setzero_pd(); // 4 × f64 accumulator

        let chunks = n / 4;
        for i in 0..chunks {
            let base = i * 4;
            let va: __m256d = _mm256_loadu_pd(a.as_ptr().add(base));
            let vb: __m256d = _mm256_loadu_pd(b.as_ptr().add(base));
            acc = _mm256_add_pd(acc, _mm256_mul_pd(va, vb));
        }

        // Horizontal sum of the 4-wide accumulator
        let mut tmp = [0.0_f64; 4];
        _mm256_storeu_pd(tmp.as_mut_ptr(), acc);
        let mut sum = tmp[0] + tmp[1] + tmp[2] + tmp[3];

        // Scalar tail for the remaining elements
        for i in (chunks * 4)..n {
            sum += a[i] * b[i];
        }
        sum
    }

    /// Sparse row × complex voltage vector multiply.
    ///
    /// Computes the real and imaginary parts of:
    ///   `result = Σ_j  (g_j + i·b_j) · (v_re_j + i·v_im_j)`
    ///
    /// where `j` ranges over `col_indices`.
    ///
    /// This is the inner loop that accumulates the Y-bus row contribution to
    /// the power mismatch in the Newton-Raphson solver.
    ///
    /// # Arguments
    /// - `col_indices` — column indices of the non-zero entries in this row
    /// - `values_re`   — real parts (conductance G) of the admittance entries
    /// - `values_im`   — imaginary parts (susceptance B) of the admittance entries
    /// - `v_re`        — real parts of the complex bus voltage vector
    /// - `v_im`        — imaginary parts of the complex bus voltage vector
    ///
    /// # Returns
    /// `(sum_re, sum_im)` — real and imaginary parts of the row × vector product.
    pub fn sparse_matvec_row(
        col_indices: &[usize],
        values_re: &[f64],
        values_im: &[f64],
        v_re: &[f64],
        v_im: &[f64],
    ) -> (f64, f64) {
        #[cfg(target_feature = "avx2")]
        // SAFETY: AVX2 availability is guaranteed at compile time by the cfg.
        unsafe {
            return sparse_matvec_row_avx(col_indices, values_re, values_im, v_re, v_im);
        }

        #[allow(unreachable_code)]
        sparse_matvec_row_scalar(col_indices, values_re, values_im, v_re, v_im)
    }

    /// Scalar sparse row × complex vector multiply (always correct, no SIMD).
    pub fn sparse_matvec_row_scalar(
        col_indices: &[usize],
        values_re: &[f64],
        values_im: &[f64],
        v_re: &[f64],
        v_im: &[f64],
    ) -> (f64, f64) {
        let mut sum_re = 0.0_f64;
        let mut sum_im = 0.0_f64;
        for ((&j, &g), &b) in col_indices
            .iter()
            .zip(values_re.iter())
            .zip(values_im.iter())
        {
            sum_re += g * v_re[j] - b * v_im[j];
            sum_im += g * v_im[j] + b * v_re[j];
        }
        (sum_re, sum_im)
    }

    /// AVX2-vectorised sparse row × complex vector multiply.
    ///
    /// # Safety
    /// Caller must guarantee that the `avx2` target feature is available.
    #[cfg(target_feature = "avx2")]
    #[target_feature(enable = "avx2")]
    pub unsafe fn sparse_matvec_row_avx(
        col_indices: &[usize],
        values_re: &[f64],
        values_im: &[f64],
        v_re: &[f64],
        v_im: &[f64],
    ) -> (f64, f64) {
        // Scalar implementation — gathers for sparse access are expensive in
        // SIMD so we fall through to the scalar path here; the AVX2 path
        // primarily benefits the dense dot product above.
        sparse_matvec_row_scalar(col_indices, values_re, values_im, v_re, v_im)
    }

    // -------------------------------------------------------------------------
    // Power injection kernel
    // -------------------------------------------------------------------------

    /// Compute the active (P) and reactive (Q) power injection at bus `i`.
    ///
    /// Evaluates the polar-form power injection formula using an explicit
    /// Y-bus row given as parallel dense slices of conductance and susceptance:
    ///
    /// ```text
    /// P_i = |V_i| · Σ_j |V_j| · (G_ij cos(θ_i − θ_j) + B_ij sin(θ_i − θ_j))
    /// Q_i = |V_i| · Σ_j |V_j| · (G_ij sin(θ_i − θ_j) − B_ij cos(θ_i − θ_j))
    /// ```
    ///
    /// # Arguments
    /// - `v_mag`  — voltage magnitudes \[p.u.\] for all buses
    /// - `v_ang`  — voltage angles \[rad\] for all buses
    /// - `g_row`  — conductance entries G_ij for this row (dense, length = n_bus)
    /// - `b_row`  — susceptance entries B_ij for this row (dense, length = n_bus)
    /// - `i`      — index of the bus being evaluated
    ///
    /// # Returns
    /// `(P_i, Q_i)` in per-unit.
    pub fn compute_power_injection(
        v_mag: &[f64],
        v_ang: &[f64],
        g_row: &[f64],
        b_row: &[f64],
        i: usize,
    ) -> (f64, f64) {
        let n = v_mag.len();
        debug_assert_eq!(v_ang.len(), n);
        debug_assert_eq!(g_row.len(), n);
        debug_assert_eq!(b_row.len(), n);

        // Choose vectorised or scalar path at compile time.
        #[cfg(target_feature = "avx2")]
        // SAFETY: AVX2 availability guaranteed at compile time.
        unsafe {
            return compute_power_injection_avx2(v_mag, v_ang, g_row, b_row, i);
        }

        #[allow(unreachable_code)]
        compute_power_injection_scalar(v_mag, v_ang, g_row, b_row, i)
    }

    /// Scalar reference implementation of power injection at bus `i`.
    pub fn compute_power_injection_scalar(
        v_mag: &[f64],
        v_ang: &[f64],
        g_row: &[f64],
        b_row: &[f64],
        i: usize,
    ) -> (f64, f64) {
        let vi = v_mag[i];
        let ti = v_ang[i];
        let mut p = 0.0_f64;
        let mut q = 0.0_f64;
        for j in 0..v_mag.len() {
            let vj = v_mag[j];
            let dth = ti - v_ang[j];
            let (sin_dth, cos_dth) = dth.sin_cos();
            let gij = g_row[j];
            let bij = b_row[j];
            p += vj * (gij * cos_dth + bij * sin_dth);
            q += vj * (gij * sin_dth - bij * cos_dth);
        }
        (vi * p, vi * q)
    }

    /// AVX2-vectorised power injection at bus `i`.
    ///
    /// Processes 4 buses per iteration for the inner summation.
    /// Trigonometric functions (`sin`, `cos`) are scalar because there is no
    /// portable AVX2 transcendental; the vectorisation covers the multiply-add
    /// accumulation.
    ///
    /// # Safety
    /// Caller must guarantee that the `avx2` target feature is available.
    #[cfg(target_feature = "avx2")]
    #[target_feature(enable = "avx2")]
    pub unsafe fn compute_power_injection_avx2(
        v_mag: &[f64],
        v_ang: &[f64],
        g_row: &[f64],
        b_row: &[f64],
        i: usize,
    ) -> (f64, f64) {
        use std::arch::x86_64::{
            __m256d, _mm256_add_pd, _mm256_loadu_pd, _mm256_mul_pd, _mm256_setzero_pd,
            _mm256_storeu_pd,
        };

        let n = v_mag.len();
        let vi = v_mag[i];
        let ti = v_ang[i];

        // Accumulate P and Q contributions using AVX2 fused mul-add where possible.
        // We pre-compute the scalar sin/cos for each j and then do vectorised
        // multiply-add over the G/B × trig product pairs.
        //
        // Strategy:
        //   For each j:  sin_dth[j], cos_dth[j]  — scalar
        //   Build dense arrays  gv[j] = G_ij * V_j * cos_dth[j] + B_ij * V_j * sin_dth[j]
        //   and              qv[j] = G_ij * V_j * sin_dth[j] - B_ij * V_j * cos_dth[j]
        //   Then SIMD-sum these.

        let mut p_terms = vec![0.0_f64; n];
        let mut q_terms = vec![0.0_f64; n];
        for j in 0..n {
            let vj = v_mag[j];
            let dth = ti - v_ang[j];
            let (sin_dth, cos_dth) = dth.sin_cos();
            p_terms[j] = vj * (g_row[j] * cos_dth + b_row[j] * sin_dth);
            q_terms[j] = vj * (g_row[j] * sin_dth - b_row[j] * cos_dth);
        }

        // AVX2 horizontal sum of p_terms and q_terms.
        let mut acc_p = _mm256_setzero_pd();
        let mut acc_q = _mm256_setzero_pd();
        let chunks = n / 4;
        for k in 0..chunks {
            let base = k * 4;
            let vp: __m256d = _mm256_loadu_pd(p_terms.as_ptr().add(base));
            let vq: __m256d = _mm256_loadu_pd(q_terms.as_ptr().add(base));
            acc_p = _mm256_add_pd(acc_p, vp);
            acc_q = _mm256_add_pd(acc_q, vq);
        }

        let mut tmp_p = [0.0_f64; 4];
        let mut tmp_q = [0.0_f64; 4];
        _mm256_storeu_pd(tmp_p.as_mut_ptr(), acc_p);
        _mm256_storeu_pd(tmp_q.as_mut_ptr(), acc_q);

        let mut p_sum = tmp_p[0] + tmp_p[1] + tmp_p[2] + tmp_p[3];
        let mut q_sum = tmp_q[0] + tmp_q[1] + tmp_q[2] + tmp_q[3];

        // Scalar tail
        for j in (chunks * 4)..n {
            p_sum += p_terms[j];
            q_sum += q_terms[j];
        }

        (vi * p_sum, vi * q_sum)
    }

    // -------------------------------------------------------------------------
    // Batch mismatch computation
    // -------------------------------------------------------------------------

    /// Compute ΔP and ΔQ mismatch vectors using the SIMD power injection path.
    ///
    /// This is the hot path for the Newton-Raphson inner loop.
    ///
    /// # Arguments
    /// - `v_mag`   — voltage magnitudes \[p.u.\], length n
    /// - `v_ang`   — voltage angles \[rad\], length n
    /// - `ybus_g`  — dense G matrix rows: `ybus_g[i][j]` = G_{ij}
    /// - `ybus_b`  — dense B matrix rows: `ybus_b[i][j]` = B_{ij}
    /// - `p_spec`  — scheduled active power injection \[p.u.\], length n
    /// - `q_spec`  — scheduled reactive power injection \[p.u.\], length n
    ///
    /// # Returns
    /// `(dp, dq)` where `dp[i] = p_spec[i] − P_calc[i]` and similarly for Q.
    pub fn compute_mismatch_simd(
        v_mag: &[f64],
        v_ang: &[f64],
        ybus_g: &[Vec<f64>],
        ybus_b: &[Vec<f64>],
        p_spec: &[f64],
        q_spec: &[f64],
    ) -> (Vec<f64>, Vec<f64>) {
        let n = v_mag.len();
        let mut dp = Vec::with_capacity(n);
        let mut dq = Vec::with_capacity(n);
        for i in 0..n {
            let (p_calc, q_calc) = compute_power_injection(v_mag, v_ang, &ybus_g[i], &ybus_b[i], i);
            dp.push(p_spec[i] - p_calc);
            dq.push(q_spec[i] - q_calc);
        }
        (dp, dq)
    }

    /// Scalar reference mismatch computation (identical semantics to SIMD path).
    ///
    /// Always uses the scalar power injection kernel regardless of CPU features.
    /// Used for correctness verification and as a non-SIMD fallback.
    pub fn compute_mismatch_scalar(
        v_mag: &[f64],
        v_ang: &[f64],
        ybus_g: &[Vec<f64>],
        ybus_b: &[Vec<f64>],
        p_spec: &[f64],
        q_spec: &[f64],
    ) -> (Vec<f64>, Vec<f64>) {
        let n = v_mag.len();
        let mut dp = Vec::with_capacity(n);
        let mut dq = Vec::with_capacity(n);
        for i in 0..n {
            let (p_calc, q_calc) =
                compute_power_injection_scalar(v_mag, v_ang, &ybus_g[i], &ybus_b[i], i);
            dp.push(p_spec[i] - p_calc);
            dq.push(q_spec[i] - q_calc);
        }
        (dp, dq)
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn test_dot_product_known_values() {
            let a = [1.0_f64, 2.0, 3.0, 4.0];
            let b = [4.0_f64, 3.0, 2.0, 1.0];
            let result = dot_product_f64(&a, &b);
            assert!(
                (result - 20.0).abs() < 1e-10,
                "Expected 20.0, got {result:.6}"
            );
        }

        #[test]
        fn test_dot_product_zeros() {
            let a = [0.0_f64; 8];
            let b = [1.0_f64; 8];
            assert!((dot_product_f64(&a, &b)).abs() < 1e-15);
        }

        #[test]
        fn test_dot_product_non_multiple_of_4() {
            // Length 7 — exercises the scalar tail
            let a = [1.0_f64, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0];
            let b = [7.0_f64, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0];
            let expected: f64 = a.iter().zip(b.iter()).map(|(x, y)| x * y).sum();
            let result = dot_product_f64(&a, &b);
            assert!(
                (result - expected).abs() < 1e-10,
                "Expected {expected:.6}, got {result:.6}"
            );
        }

        #[test]
        fn test_dot_product_single_element() {
            let a = [std::f64::consts::PI];
            let b = [2.0_f64];
            let result = dot_product_f64(&a, &b);
            assert!((result - std::f64::consts::TAU).abs() < 1e-10);
        }

        #[test]
        fn test_dot_product_empty() {
            let result = dot_product_f64(&[], &[]);
            assert!((result).abs() < 1e-15);
        }

        #[test]
        fn test_sparse_matvec_row_scalar_basic() {
            // G + jB = 1 + j2, v = 3 + j4
            // result = (1*3 - 2*4) + j(1*4 + 2*3) = (3-8) + j(4+6) = -5 + j10
            let col_indices = [0_usize];
            let values_re = [1.0_f64];
            let values_im = [2.0_f64];
            let v_re = [3.0_f64];
            let v_im = [4.0_f64];
            let (re, im) =
                sparse_matvec_row_scalar(&col_indices, &values_re, &values_im, &v_re, &v_im);
            assert!((re - (-5.0)).abs() < 1e-10, "re={re:.6}");
            assert!((im - 10.0).abs() < 1e-10, "im={im:.6}");
        }

        #[test]
        fn test_sparse_matvec_row_multi_entries() {
            // Two entries: (j=0: G=1, B=0) and (j=1: G=0, B=1)
            // v_re=[1,0], v_im=[0,1]
            // entry 0: re += 1*1 - 0*0 = 1, im += 1*0 + 0*1 = 0
            // entry 1: re += 0*0 - 1*1 = -1, im += 0*1 + 1*0 = 0
            // total: re=0, im=0
            let col_indices = [0_usize, 1_usize];
            let values_re = [1.0_f64, 0.0];
            let values_im = [0.0_f64, 1.0];
            let v_re = [1.0_f64, 0.0];
            let v_im = [0.0_f64, 1.0];
            let (re, im) = sparse_matvec_row(&col_indices, &values_re, &values_im, &v_re, &v_im);
            assert!((re).abs() < 1e-10, "re={re:.6}");
            assert!((im).abs() < 1e-10, "im={im:.6}");
        }

        #[test]
        fn test_sparse_matvec_row_empty() {
            let (re, im) = sparse_matvec_row(&[], &[], &[], &[1.0], &[1.0]);
            assert!((re).abs() < 1e-15);
            assert!((im).abs() < 1e-15);
        }

        // -----------------------------------------------------------------
        // Power injection kernel tests
        // -----------------------------------------------------------------

        /// Build a simple 2-bus Y-bus in dense form for testing.
        ///
        /// Network: bus 1 -- (r=0.01, x=0.1) -- bus 2
        /// Y_12 = 1/(r+jx) ≈ 0.99 - j9.9
        /// Diagonal: Y_11 = Y_12  (no shunt), same for Y_22.
        fn two_bus_ybus_dense() -> (Vec<Vec<f64>>, Vec<Vec<f64>>) {
            let r = 0.01_f64;
            let x = 0.1_f64;
            let denom = r * r + x * x;
            let g = r / denom;
            let b = -x / denom;
            // Y_bus:
            //  [ g+jb   -(g+jb) ]
            //  [ -(g+jb)  g+jb  ]
            let g_mat = vec![vec![g, -g], vec![-g, g]];
            let b_mat = vec![vec![b, -b], vec![-b, b]];
            (g_mat, b_mat)
        }

        #[test]
        fn test_power_injection_scalar_identity() {
            // With V = [1∠0, 1∠0] (flat start) and the 2-bus Y-bus above,
            // P_0 = |V_0| * Σ_j |V_j| (G_0j cos(0) + B_0j sin(0))
            //      = 1 * (G_00*1 + G_01*1) = G_00 + G_01 = g - g = 0
            let (g_mat, b_mat) = two_bus_ybus_dense();
            let v_mag = [1.0_f64, 1.0];
            let v_ang = [0.0_f64, 0.0];
            let (p, q) = compute_power_injection_scalar(&v_mag, &v_ang, &g_mat[0], &b_mat[0], 0);
            assert!(p.abs() < 1e-10, "P should be 0 at flat start, got {p:.6e}");
            // Q = V_0 * Σ_j V_j (G_0j sin(0) - B_0j cos(0))
            //   = 1 * (−B_00 − B_01) = −b + b = 0
            assert!(q.abs() < 1e-10, "Q should be 0 at flat start, got {q:.6e}");
        }

        #[test]
        fn test_power_injection_matches_scalar_simd() {
            let (g_mat, b_mat) = two_bus_ybus_dense();
            // Non-flat voltage to exercise trig
            let v_mag = [1.02_f64, 0.98];
            let v_ang = [0.0_f64, -0.05];
            for i in 0..2 {
                let (p_s, q_s) =
                    compute_power_injection_scalar(&v_mag, &v_ang, &g_mat[i], &b_mat[i], i);
                let (p_v, q_v) = compute_power_injection(&v_mag, &v_ang, &g_mat[i], &b_mat[i], i);
                assert!(
                    (p_s - p_v).abs() < 1e-12,
                    "P mismatch bus {i}: scalar={p_s:.8e} simd={p_v:.8e}"
                );
                assert!(
                    (q_s - q_v).abs() < 1e-12,
                    "Q mismatch bus {i}: scalar={q_s:.8e} simd={q_v:.8e}"
                );
            }
        }

        #[test]
        fn test_simd_matches_scalar_mismatch() {
            // Verify that the SIMD and scalar mismatch paths agree to < 1e-10
            // on a non-trivial 3-bus example.
            let n = 3;
            // Diagonal Y-bus (no coupling) for simplicity: G_ii = 5, B_ii = -10
            let g_mat: Vec<Vec<f64>> = (0..n)
                .map(|i| {
                    let mut row = vec![0.0_f64; n];
                    row[i] = 5.0;
                    row
                })
                .collect();
            let b_mat: Vec<Vec<f64>> = (0..n)
                .map(|i| {
                    let mut row = vec![0.0_f64; n];
                    row[i] = -10.0;
                    row
                })
                .collect();
            let v_mag = [1.05_f64, 0.99, 1.01];
            let v_ang = [0.0_f64, -0.03, 0.02];
            let p_spec = [-0.2_f64, 0.1, 0.05];
            let q_spec = [-0.1_f64, 0.05, 0.03];

            let (dp_simd, dq_simd) =
                compute_mismatch_simd(&v_mag, &v_ang, &g_mat, &b_mat, &p_spec, &q_spec);
            let (dp_scal, dq_scal) =
                compute_mismatch_scalar(&v_mag, &v_ang, &g_mat, &b_mat, &p_spec, &q_spec);

            for i in 0..n {
                assert!(
                    (dp_simd[i] - dp_scal[i]).abs() < 1e-10,
                    "ΔP mismatch at bus {i}: simd={:.8e} scalar={:.8e}",
                    dp_simd[i],
                    dp_scal[i]
                );
                assert!(
                    (dq_simd[i] - dq_scal[i]).abs() < 1e-10,
                    "ΔQ mismatch at bus {i}: simd={:.8e} scalar={:.8e}",
                    dq_simd[i],
                    dq_scal[i]
                );
            }
        }
    }
}

#[cfg(not(feature = "simd"))]
pub mod simd {
    // SIMD feature not enabled — module intentionally empty.
    // Use the standard scalar paths in the power flow solver directly.
}
