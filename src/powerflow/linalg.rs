//! Linear algebra backend abstraction for power flow solvers.
//!
//! # Backend selection
//!
//! Use `select_backend(n)` to obtain the most performant backend for a system
//! of size `n`. The function respects the `simd` feature flag and performs a
//! runtime AVX2 check before dispatching to the SIMD path.
//!
//! # Mathematical background
//!
//! The Newton-Raphson power flow inner loop solves:
//!   J · Δx = -f(x)
//! where J is the (2n-2) × (2n-2) Jacobian matrix (dense or sparse depending
//! on network size), Δx = [Δθ; ΔV/V], and f(x) = [ΔP; ΔQ] is the power mismatch.
use crate::error::Result;
use crate::powerflow::sparse_lu::{CrsMatrix, IterativeRefinement, NalgebraDenseLu, SprsLu};
use nalgebra::DMatrix;

/// Abstraction over linear algebra operations used in the NR inner loop.
pub trait LinearAlgebraBackend: Send + Sync {
    /// Solve A·x = b (A is a dense nalgebra matrix).
    fn solve_dense(&self, a: &DMatrix<f64>, b: &[f64]) -> Result<Vec<f64>>;

    /// Dot product of two equal-length slices.
    fn dot(&self, a: &[f64], b: &[f64]) -> f64 {
        a.iter().zip(b.iter()).map(|(&x, &y)| x * y).sum()
    }

    /// AXPY: y += alpha * x (in-place).
    fn axpy(&self, alpha: f64, x: &[f64], y: &mut [f64]) {
        for (yi, &xi) in y.iter_mut().zip(x.iter()) {
            *yi += alpha * xi;
        }
    }

    /// Sparse matrix-vector product y = M·x using the scalar CRS path.
    fn matvec_csr(&self, m: &CrsMatrix, x: &[f64], y: &mut [f64]) {
        for (r, yi) in y.iter_mut().enumerate() {
            *yi = 0.0;
            for k in m.row_ptr[r]..m.row_ptr[r + 1] {
                *yi += m.values[k] * x[m.col_idx[k]];
            }
        }
    }

    /// Human-readable backend name for diagnostics.
    fn name(&self) -> &'static str;
}

impl LinearAlgebraBackend for NalgebraDenseLu {
    fn solve_dense(&self, a: &DMatrix<f64>, b: &[f64]) -> Result<Vec<f64>> {
        use crate::powerflow::sparse_lu::LinearSolver;
        <Self as LinearSolver>::solve_dense(self, a, b)
    }

    fn name(&self) -> &'static str {
        "nalgebra-dense-lu"
    }
}

impl LinearAlgebraBackend for IterativeRefinement<SprsLu> {
    fn solve_dense(&self, a: &DMatrix<f64>, b: &[f64]) -> Result<Vec<f64>> {
        use crate::powerflow::sparse_lu::LinearSolver;
        <Self as LinearSolver>::solve_dense(self, a, b)
    }

    fn name(&self) -> &'static str {
        "sprs-lu-refined"
    }
}

/// SIMD-accelerated backend (AVX2).
///
/// Only compiled when both the `simd` feature flag is active and the target
/// architecture is x86_64.  On all other configurations `select_backend` will
/// fall through to the scalar paths.
#[cfg(all(feature = "simd", target_arch = "x86_64"))]
pub struct SimdAvx2Backend {
    inner: NalgebraDenseLu,
}

#[cfg(all(feature = "simd", target_arch = "x86_64"))]
impl Default for SimdAvx2Backend {
    fn default() -> Self {
        Self {
            inner: NalgebraDenseLu,
        }
    }
}

#[cfg(all(feature = "simd", target_arch = "x86_64"))]
impl LinearAlgebraBackend for SimdAvx2Backend {
    fn solve_dense(&self, a: &DMatrix<f64>, b: &[f64]) -> Result<Vec<f64>> {
        use crate::powerflow::sparse_lu::LinearSolver;
        LinearSolver::solve_dense(&self.inner, a, b)
    }

    fn dot(&self, a: &[f64], b: &[f64]) -> f64 {
        crate::powerflow::simd_kernels::simd::dot_product_f64(a, b)
    }

    fn axpy(&self, alpha: f64, x: &[f64], y: &mut [f64]) {
        crate::powerflow::simd_kernels::simd::axpy_f64(alpha, x, y);
    }

    fn name(&self) -> &'static str {
        "simd-avx2"
    }
}

/// Threshold (number of buses) below which dense LU is used.
const SPARSE_THRESHOLD: usize = 200;

/// Select the best `LinearAlgebraBackend` for a system of `n` equations.
///
/// Decision logic:
/// 1. If the `simd` feature is active and the CPU supports AVX2 → `SimdAvx2Backend`.
/// 2. Else if n > `SPARSE_THRESHOLD` → `IterativeRefinement<SprsLu>`.
/// 3. Else → `NalgebraDenseLu`.
pub fn select_backend(n: usize) -> Box<dyn LinearAlgebraBackend> {
    #[cfg(all(feature = "simd", target_arch = "x86_64"))]
    if std::is_x86_feature_detected!("avx2") {
        return Box::new(SimdAvx2Backend::default());
    }

    if n > SPARSE_THRESHOLD {
        Box::new(IterativeRefinement::new(SprsLu, 2, 1e-10))
    } else {
        Box::new(NalgebraDenseLu)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::DMatrix;

    fn identity(n: usize) -> DMatrix<f64> {
        DMatrix::identity(n, n)
    }

    #[test]
    fn test_nalgebra_backend_identity() {
        let backend = NalgebraDenseLu;
        let a = identity(3);
        let b = vec![1.0, 2.0, 3.0];
        let x = backend.solve_dense(&a, &b).expect("solve failed");
        for (xi, bi) in x.iter().zip(b.iter()) {
            assert!((xi - bi).abs() < 1e-12);
        }
    }

    #[test]
    fn test_iterative_refinement_backend() {
        let backend = IterativeRefinement::new(SprsLu, 2, 1e-10);
        let a = identity(5);
        let b = vec![5.0, 4.0, 3.0, 2.0, 1.0];
        let x = backend.solve_dense(&a, &b).expect("solve failed");
        for (xi, bi) in x.iter().zip(b.iter()) {
            assert!((xi - bi).abs() < 1e-10);
        }
    }

    #[test]
    fn test_default_dot() {
        let backend = NalgebraDenseLu;
        let a = vec![1.0, 2.0, 3.0];
        let b = vec![3.0, 2.0, 1.0];
        assert!((backend.dot(&a, &b) - 10.0).abs() < 1e-12);
    }

    #[test]
    fn test_default_axpy() {
        let backend = NalgebraDenseLu;
        let x = vec![1.0, 2.0, 3.0];
        let mut y = vec![0.0, 1.0, 2.0];
        backend.axpy(2.0, &x, &mut y);
        assert!((y[0] - 2.0).abs() < 1e-12);
        assert!((y[1] - 5.0).abs() < 1e-12);
        assert!((y[2] - 8.0).abs() < 1e-12);
    }

    #[test]
    fn test_matvec_csr() {
        let backend = NalgebraDenseLu;
        let triplets = vec![(0, 0, 2.0), (0, 1, 1.0), (1, 0, 3.0), (1, 1, 4.0)];
        let m = CrsMatrix::from_triplets(2, 2, &triplets);
        let x = vec![1.0, 2.0];
        let mut y = vec![0.0; 2];
        backend.matvec_csr(&m, &x, &mut y);
        assert!((y[0] - 4.0).abs() < 1e-12); // 2*1 + 1*2
        assert!((y[1] - 11.0).abs() < 1e-12); // 3*1 + 4*2
    }

    #[test]
    fn test_select_backend_small() {
        let backend = select_backend(50);
        let a = identity(3);
        let b = vec![1.0, 2.0, 3.0];
        let x = backend.solve_dense(&a, &b).expect("solve failed");
        for (xi, bi) in x.iter().zip(b.iter()) {
            assert!((xi - bi).abs() < 1e-12);
        }
    }

    #[test]
    fn test_select_backend_large() {
        let backend = select_backend(300);
        let a = identity(3);
        let b = vec![3.0, 2.0, 1.0];
        let x = backend.solve_dense(&a, &b).expect("solve failed");
        for (xi, bi) in x.iter().zip(b.iter()) {
            assert!((xi - bi).abs() < 1e-12);
        }
    }

    #[test]
    fn test_backend_names() {
        let dense = NalgebraDenseLu;
        assert_eq!(dense.name(), "nalgebra-dense-lu");
        let refined = IterativeRefinement::new(SprsLu, 2, 1e-10);
        assert_eq!(refined.name(), "sprs-lu-refined");
    }

    #[test]
    fn test_solve_2x2_non_trivial_system() {
        let backend = NalgebraDenseLu;
        let a = DMatrix::from_row_slice(2, 2, &[2.0, 1.0, 1.0, 3.0]);
        let b = vec![5.0, 10.0];
        let x = backend.solve_dense(&a, &b).expect("solve failed");
        assert!((x[0] - 1.0).abs() < 1e-10);
        assert!((x[1] - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_residual_norm_after_solve() {
        let backend = NalgebraDenseLu;
        let a = DMatrix::from_row_slice(3, 3, &[4.0, 1.0, 0.0, 1.0, 3.0, 1.0, 0.0, 1.0, 2.0]);
        let b = vec![1.0, 2.0, 3.0];
        let x = backend.solve_dense(&a, &b).expect("solve failed");
        let r0 = 4.0 * x[0] + 1.0 * x[1] + 0.0 * x[2] - b[0];
        let r1 = 1.0 * x[0] + 3.0 * x[1] + 1.0 * x[2] - b[1];
        let r2 = 0.0 * x[0] + 1.0 * x[1] + 2.0 * x[2] - b[2];
        assert!(r0.abs() < 1e-9);
        assert!(r1.abs() < 1e-9);
        assert!(r2.abs() < 1e-9);
    }

    #[test]
    fn test_dot_product_empty_slices() {
        let backend = NalgebraDenseLu;
        let result = backend.dot(&[], &[]);
        assert_eq!(result, 0.0);
    }

    #[test]
    fn test_axpy_with_alpha_zero_is_noop() {
        let backend = NalgebraDenseLu;
        let x = vec![1.0, 2.0, 3.0];
        let mut y = vec![4.0, 5.0, 6.0];
        backend.axpy(0.0, &x, &mut y);
        assert!((y[0] - 4.0).abs() < 1e-12);
        assert!((y[1] - 5.0).abs() < 1e-12);
        assert!((y[2] - 6.0).abs() < 1e-12);
    }

    #[test]
    fn test_matvec_csr_diagonal_matrix() {
        let backend = NalgebraDenseLu;
        let triplets = vec![(0, 0, 2.0), (1, 1, 3.0), (2, 2, 5.0)];
        let m = CrsMatrix::from_triplets(3, 3, &triplets);
        let x = vec![1.0, 2.0, 3.0];
        let mut y = vec![0.0; 3];
        backend.matvec_csr(&m, &x, &mut y);
        assert!((y[0] - 2.0).abs() < 1e-12);
        assert!((y[1] - 6.0).abs() < 1e-12);
        assert!((y[2] - 15.0).abs() < 1e-12);
    }
}
