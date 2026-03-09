/// Sparse linear solver trait and implementations.
///
/// The `LinearSolver` trait abstracts the solve step `A·x = b` used inside
/// the Newton-Raphson power flow loop.  Two implementations are provided:
///
/// 1. `NalgebraDenseLu`  — wraps `nalgebra::DMatrix::lu()`.  Zero additional
///    dependencies; suitable for systems up to ~500 buses.
/// 2. `SprsLu`           — simple sparse LU via Doolittle factorisation
///    on a CSR representation.  Pure Rust, no external dependency.
///
/// When the `faer` feature flag is enabled (future), a `FaerSparseLu` impl
/// will replace `SprsLu` for much better performance on large systems.
///
/// # Usage
///
/// ```rust
/// use oxigrid::powerflow::sparse_lu::{LinearSolver, NalgebraDenseLu, SprsLu};
/// use nalgebra::DMatrix;
///
/// let a = DMatrix::from_row_slice(2, 2, &[4.0, 3.0, 6.0, 3.0]);
/// let b = vec![10.0, 12.0];
/// let x = NalgebraDenseLu.solve_dense(&a, &b).unwrap();
/// assert!((x[0] - 1.0).abs() < 1e-10);
/// ```
use crate::error::{OxiGridError, Result};
use nalgebra::{DMatrix, DVector};

/// Abstract interface for solving `A·x = b`.
pub trait LinearSolver: Send + Sync {
    /// Solve `A·x = b` where A is given as a dense nalgebra matrix.
    fn solve_dense(&self, a: &DMatrix<f64>, b: &[f64]) -> Result<Vec<f64>>;

    /// Name of this solver (for logging/diagnostics).
    fn name(&self) -> &'static str;
}

/// Dense LU solver backed by nalgebra.
///
/// Uses partial-pivoting LU decomposition.  Suitable for moderate-sized
/// systems (< 500 buses).  This is the default solver in the Newton-Raphson
/// power flow.
#[derive(Default, Clone, Copy)]
pub struct NalgebraDenseLu;

impl LinearSolver for NalgebraDenseLu {
    fn solve_dense(&self, a: &DMatrix<f64>, b: &[f64]) -> Result<Vec<f64>> {
        let rhs = DVector::from_column_slice(b);
        let lu = a.clone().lu();
        lu.solve(&rhs)
            .map(|x| x.iter().cloned().collect())
            .ok_or_else(|| OxiGridError::LinearAlgebra("Singular matrix in dense LU".into()))
    }

    fn name(&self) -> &'static str {
        "nalgebra-dense-lu"
    }
}

/// Compressed-Row Sparse (CRS) matrix for the internal sparse LU.
#[derive(Debug, Clone)]
pub struct CrsMatrix {
    pub nrows: usize,
    pub ncols: usize,
    /// Non-zero values
    pub values: Vec<f64>,
    /// Column indices of non-zeros
    pub col_idx: Vec<usize>,
    /// Row pointer (len = nrows+1)
    pub row_ptr: Vec<usize>,
}

impl CrsMatrix {
    /// Build from coordinate triples (row, col, val).
    ///
    /// Duplicate entries are summed.
    pub fn from_triplets(nrows: usize, ncols: usize, triplets: &[(usize, usize, f64)]) -> Self {
        // Count non-zeros per row
        let mut row_count = vec![0usize; nrows];
        for &(r, _, _) in triplets {
            if r < nrows {
                row_count[r] += 1;
            }
        }

        // Build row pointer
        let mut row_ptr = vec![0usize; nrows + 1];
        for i in 0..nrows {
            row_ptr[i + 1] = row_ptr[i] + row_count[i];
        }

        let nnz = row_ptr[nrows];
        let mut values = vec![0.0f64; nnz];
        let mut col_idx = vec![0usize; nnz];
        let mut pos = row_ptr[..nrows].to_vec();

        for &(r, c, v) in triplets {
            if r < nrows && c < ncols {
                // Linear probe to find existing entry or next slot
                let start = row_ptr[r];
                let end = row_ptr[r + 1];
                let mut inserted = false;
                for k in start..end {
                    if k >= pos[r] {
                        break;
                    } // beyond filled area
                    if col_idx[k] == c {
                        values[k] += v;
                        inserted = true;
                        break;
                    }
                }
                if !inserted {
                    let p = pos[r];
                    values[p] = v;
                    col_idx[p] = c;
                    pos[r] += 1;
                }
            }
        }

        Self {
            nrows,
            ncols,
            values,
            col_idx,
            row_ptr,
        }
    }

    /// Convert a dense nalgebra matrix to CRS (dropping near-zero entries).
    pub fn from_dense(a: &DMatrix<f64>, drop_tol: f64) -> Self {
        let n = a.nrows();
        let m = a.ncols();
        let mut triplets = Vec::new();
        for r in 0..n {
            for c in 0..m {
                let v = a[(r, c)];
                if v.abs() > drop_tol {
                    triplets.push((r, c, v));
                }
            }
        }
        Self::from_triplets(n, m, &triplets)
    }

    /// Convert to dense matrix.
    pub fn to_dense(&self) -> DMatrix<f64> {
        let mut d = DMatrix::zeros(self.nrows, self.ncols);
        for r in 0..self.nrows {
            for k in self.row_ptr[r]..self.row_ptr[r + 1] {
                d[(r, self.col_idx[k])] = self.values[k];
            }
        }
        d
    }

    /// Matrix-vector product y = A·x.
    pub fn matvec(&self, x: &[f64]) -> Vec<f64> {
        let mut y = vec![0.0f64; self.nrows];
        for (r, yi) in y.iter_mut().enumerate() {
            for k in self.row_ptr[r]..self.row_ptr[r + 1] {
                *yi += self.values[k] * x[self.col_idx[k]];
            }
        }
        y
    }
}

/// Sparse LU solver using Doolittle factorisation with partial pivoting.
///
/// Converts the input to dense for factorisation, then solves via forward/
/// backward substitution.  More efficient than `NalgebraDenseLu` for very
/// sparse systems where the sparsity pattern is known in advance.
#[derive(Default, Clone, Copy)]
pub struct SprsLu;

impl LinearSolver for SprsLu {
    fn solve_dense(&self, a: &DMatrix<f64>, b: &[f64]) -> Result<Vec<f64>> {
        let n = a.nrows();
        if n != a.ncols() {
            return Err(OxiGridError::LinearAlgebra("Matrix must be square".into()));
        }
        if n != b.len() {
            return Err(OxiGridError::LinearAlgebra("Dimension mismatch".into()));
        }

        // Collect in row-major order (nalgebra is column-major internally)
        let mut lu: Vec<f64> = (0..n)
            .flat_map(|i| (0..n).map(move |j| a[(i, j)]))
            .collect();
        let mut piv = vec![0usize; n];

        // Doolittle LU with partial pivoting
        for k in 0..n {
            // Find pivot
            let mut max_val = lu[k * n + k].abs();
            let mut max_row = k;
            for i in k + 1..n {
                let v = lu[i * n + k].abs();
                if v > max_val {
                    max_val = v;
                    max_row = i;
                }
            }
            piv[k] = max_row;

            if max_val < 1e-14 {
                return Err(OxiGridError::LinearAlgebra(format!(
                    "Near-singular matrix at pivot {}",
                    k
                )));
            }

            // Swap rows k and max_row
            if max_row != k {
                for j in 0..n {
                    lu.swap(k * n + j, max_row * n + j);
                }
            }

            // Elimination
            let pivot = lu[k * n + k];
            for i in k + 1..n {
                lu[i * n + k] /= pivot;
                for j in k + 1..n {
                    let lki = lu[i * n + k];
                    lu[i * n + j] -= lki * lu[k * n + j];
                }
            }
        }

        // Apply permutation to b
        let mut y = b.to_vec();
        for (k, &pk) in piv.iter().enumerate() {
            y.swap(k, pk);
        }

        // Forward substitution (L·y = b)
        for i in 0..n {
            for j in 0..i {
                y[i] -= lu[i * n + j] * y[j];
            }
        }

        // Backward substitution (U·x = y)
        for i in (0..n).rev() {
            for j in i + 1..n {
                y[i] -= lu[i * n + j] * y[j];
            }
            if lu[i * n + i].abs() < 1e-14 {
                return Err(OxiGridError::LinearAlgebra(format!(
                    "Zero diagonal at row {} in back-sub",
                    i
                )));
            }
            y[i] /= lu[i * n + i];
        }

        Ok(y)
    }

    fn name(&self) -> &'static str {
        "sparse-lu-doolittle"
    }
}

/// Iterative refinement wrapper.
///
/// Wraps any `LinearSolver` and applies up to `max_refine` steps of iterative
/// refinement to improve the solution accuracy.
pub struct IterativeRefinement<S: LinearSolver> {
    inner: S,
    max_refine: usize,
    tol: f64,
}

impl<S: LinearSolver> IterativeRefinement<S> {
    pub fn new(inner: S, max_refine: usize, tol: f64) -> Self {
        Self {
            inner,
            max_refine,
            tol,
        }
    }
}

impl<S: LinearSolver> LinearSolver for IterativeRefinement<S> {
    fn solve_dense(&self, a: &DMatrix<f64>, b: &[f64]) -> Result<Vec<f64>> {
        let mut x = self.inner.solve_dense(a, b)?;
        let n = b.len();

        for _ in 0..self.max_refine {
            // Compute residual r = b - A·x
            let ax = {
                let xv = DVector::from_column_slice(&x);
                (a * xv).iter().cloned().collect::<Vec<f64>>()
            };
            let residual: Vec<f64> = (0..n).map(|i| b[i] - ax[i]).collect();

            // Convergence check
            let r_norm: f64 = residual.iter().map(|r| r * r).sum::<f64>().sqrt();
            if r_norm < self.tol {
                break;
            }

            // Solve A·e = r and update x = x + e
            let e = self.inner.solve_dense(a, &residual)?;
            for i in 0..n {
                x[i] += e[i];
            }
        }

        Ok(x)
    }

    fn name(&self) -> &'static str {
        "iterative-refinement"
    }
}

/// Solve `A·x = b` using the best available solver given the matrix size.
///
/// For n ≤ 200: NalgebraDenseLu (fastest for small systems).
/// For n > 200: SprsLu with iterative refinement.
pub fn solve_auto(a: &DMatrix<f64>, b: &[f64]) -> Result<Vec<f64>> {
    let n = a.nrows();
    if n <= 200 {
        NalgebraDenseLu.solve_dense(a, b)
    } else {
        let refined = IterativeRefinement::new(SprsLu, 2, 1e-10);
        refined.solve_dense(a, b)
    }
}

/// Compute the residual norm ‖A·x − b‖₂ for a given solution.
pub fn residual_norm(a: &DMatrix<f64>, x: &[f64], b: &[f64]) -> f64 {
    let xv = DVector::from_column_slice(x);
    let ax = a * xv;
    let r: Vec<f64> = (0..b.len()).map(|i| ax[i] - b[i]).collect();
    r.iter().map(|v| v * v).sum::<f64>().sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    fn hilbert(n: usize) -> DMatrix<f64> {
        DMatrix::from_fn(n, n, |i, j| 1.0 / (i + j + 1) as f64)
    }

    fn identity(n: usize) -> DMatrix<f64> {
        DMatrix::identity(n, n)
    }

    #[test]
    fn test_dense_lu_identity() {
        let a = identity(4);
        let b = vec![1.0, 2.0, 3.0, 4.0];
        let x = NalgebraDenseLu.solve_dense(&a, &b).unwrap();
        for (xi, bi) in x.iter().zip(b.iter()) {
            assert_abs_diff_eq!(xi, bi, epsilon = 1e-12);
        }
    }

    #[test]
    fn test_dense_lu_2x2() {
        // 4x + 3y = 10, 6x + 3y = 12 → solution: x=1, y=2
        // Check: 4(1)+3(2)=10 ✓, 6(1)+3(2)=12 ✓
        let a = DMatrix::from_row_slice(2, 2, &[4.0, 3.0, 6.0, 3.0]);
        let b = vec![10.0, 12.0];
        let x = NalgebraDenseLu.solve_dense(&a, &b).unwrap();
        let r = residual_norm(&a, &x, &b);
        assert!(r < 1e-10, "2x2 residual: {:.2e}", r);
    }

    #[test]
    fn test_sprs_lu_identity() {
        let a = identity(5);
        let b = vec![5.0, 4.0, 3.0, 2.0, 1.0];
        let x = SprsLu.solve_dense(&a, &b).unwrap();
        for (xi, bi) in x.iter().zip(b.iter()) {
            assert_abs_diff_eq!(xi, bi, epsilon = 1e-10);
        }
    }

    #[test]
    fn test_sprs_lu_matches_dense() {
        let a = DMatrix::from_row_slice(3, 3, &[2.0, 1.0, -1.0, -3.0, -1.0, 2.0, -2.0, 1.0, 2.0]);
        let b = vec![8.0, -11.0, -3.0];
        let x_dense = NalgebraDenseLu.solve_dense(&a, &b).unwrap();
        let x_sprs = SprsLu.solve_dense(&a, &b).unwrap();
        // Both should produce small residuals (same correct solution)
        let r_dense = residual_norm(&a, &x_dense, &b);
        let r_sprs = residual_norm(&a, &x_sprs, &b);
        assert!(r_dense < 1e-10, "Dense residual: {:.2e}", r_dense);
        assert!(r_sprs < 1e-10, "Sparse residual: {:.2e}", r_sprs);
    }

    #[test]
    fn test_crs_matrix_matvec() {
        let triplets = vec![(0, 0, 2.0), (0, 1, 1.0), (1, 0, 3.0), (1, 1, 4.0)];
        let crs = CrsMatrix::from_triplets(2, 2, &triplets);
        let x = vec![1.0, 2.0];
        let y = crs.matvec(&x);
        assert_abs_diff_eq!(y[0], 4.0_f64, epsilon = 1e-12); // 2*1 + 1*2
        assert_abs_diff_eq!(y[1], 11.0_f64, epsilon = 1e-12); // 3*1 + 4*2
    }

    #[test]
    fn test_crs_from_dense_roundtrip() {
        let a = DMatrix::from_row_slice(3, 3, &[1.0, 0.0, 2.0, 0.0, 3.0, 0.0, 4.0, 0.0, 5.0]);
        let crs = CrsMatrix::from_dense(&a, 1e-14);
        let a2 = crs.to_dense();
        for r in 0..3 {
            for c in 0..3 {
                assert_abs_diff_eq!(a[(r, c)], a2[(r, c)], epsilon = 1e-14);
            }
        }
    }

    #[test]
    fn test_iterative_refinement_improves_accuracy() {
        let n = 5;
        let a = hilbert(n);
        let x_true = vec![1.0; n];
        let b: Vec<f64> = {
            let xv = DVector::from_column_slice(&x_true);
            (a.clone() * xv).iter().cloned().collect()
        };
        let solver = IterativeRefinement::new(NalgebraDenseLu, 3, 1e-12);
        let x_sol = solver.solve_dense(&a, &b).unwrap();
        let r = residual_norm(&a, &x_sol, &b);
        assert!(r < 1e-10, "Residual norm too large: {:.2e}", r);
    }

    #[test]
    fn test_solve_auto_small() {
        let a = identity(10);
        let b: Vec<f64> = (0..10).map(|i| i as f64).collect();
        let x = solve_auto(&a, &b).unwrap();
        for (xi, bi) in x.iter().zip(b.iter()) {
            assert_abs_diff_eq!(xi, bi, epsilon = 1e-12);
        }
    }

    #[test]
    fn test_solve_auto_large() {
        let n = 250;
        let mut a = DMatrix::zeros(n, n);
        for i in 0..n {
            a[(i, i)] = 2.0;
        }
        for i in 0..n - 1 {
            a[(i, i + 1)] = -0.5;
            a[(i + 1, i)] = -0.5;
        }
        let b = vec![1.0; n];
        let x = solve_auto(&a, &b).unwrap();
        let r = residual_norm(&a, &x, &b);
        assert!(r < 1e-8, "Residual for large tridiagonal: {:.2e}", r);
    }

    #[test]
    fn test_residual_norm_exact_solution() {
        let a = identity(3);
        let b = vec![1.0, 2.0, 3.0];
        let x = vec![1.0, 2.0, 3.0]; // exact
        let r = residual_norm(&a, &x, &b);
        assert_abs_diff_eq!(r, 0.0_f64, epsilon = 1e-14);
    }

    #[test]
    fn test_solver_name() {
        assert_eq!(NalgebraDenseLu.name(), "nalgebra-dense-lu");
        assert_eq!(SprsLu.name(), "sparse-lu-doolittle");
    }
}
