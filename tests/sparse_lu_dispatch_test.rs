//! Tests for LinearAlgebraBackend trait dispatch and correctness.
#![cfg(feature = "powerflow")]
use nalgebra::DMatrix;
use oxigrid::powerflow::linalg::{select_backend, LinearAlgebraBackend};
use oxigrid::powerflow::sparse_lu::NalgebraDenseLu;

fn identity(n: usize) -> DMatrix<f64> {
    DMatrix::identity(n, n)
}

fn residual_norm(a: &DMatrix<f64>, x: &[f64], b: &[f64]) -> f64 {
    let xv = nalgebra::DVector::from_column_slice(x);
    let ax = a * xv;
    let r: Vec<f64> = (0..b.len()).map(|i| (ax[i] - b[i]).powi(2)).collect();
    r.iter().sum::<f64>().sqrt()
}

#[test]
fn test_select_backend_small_is_dense() {
    let b = select_backend(50);
    let a = identity(50);
    let rhs = vec![1.0f64; 50];
    let x = b.solve_dense(&a, &rhs).expect("solve failed");
    assert_eq!(x.len(), 50);
    for xi in &x {
        assert!((xi - 1.0).abs() < 1e-10);
    }
}

#[test]
fn test_select_backend_large_solves_correctly() {
    let n = 300;
    let b_backend = select_backend(n);
    // Tridiagonal system
    let mut a = DMatrix::zeros(n, n);
    for i in 0..n {
        a[(i, i)] = 2.0;
    }
    for i in 0..n - 1 {
        a[(i, i + 1)] = -0.5;
        a[(i + 1, i)] = -0.5;
    }
    let rhs = vec![1.0f64; n];
    let x = b_backend.solve_dense(&a, &rhs).expect("large solve failed");
    let r = residual_norm(&a, &x, &rhs);
    assert!(r < 1e-8, "Large system residual too large: {:.3e}", r);
}

#[test]
fn test_nalgebra_dense_lu_backend() {
    let backend: &dyn LinearAlgebraBackend = &NalgebraDenseLu;
    let a = DMatrix::from_row_slice(2, 2, &[4.0, 3.0, 6.0, 3.0]);
    let b = vec![10.0f64, 12.0];
    let x = backend.solve_dense(&a, &b).expect("2x2 solve");
    assert!((x[0] - 1.0).abs() < 1e-10, "x[0]={}", x[0]);
    assert!((x[1] - 2.0).abs() < 1e-10, "x[1]={}", x[1]);
}

#[test]
fn test_backend_dot_product() {
    let backend = select_backend(10);
    let a = vec![1.0, 2.0, 3.0, 4.0];
    let b = vec![4.0, 3.0, 2.0, 1.0];
    let d = backend.dot(&a, &b);
    assert!((d - 20.0).abs() < 1e-12, "dot={}", d);
}

#[test]
fn test_backend_axpy() {
    let backend = select_backend(10);
    let x = vec![1.0, 2.0, 3.0];
    let mut y = vec![0.0, 1.0, 2.0];
    backend.axpy(2.0, &x, &mut y);
    assert!((y[0] - 2.0).abs() < 1e-12);
    assert!((y[1] - 5.0).abs() < 1e-12);
    assert!((y[2] - 8.0).abs() < 1e-12);
}

#[test]
fn test_nr_ieee14_still_converges() {
    // End-to-end regression: NR must still converge on IEEE-14 after the backend wiring.
    use oxigrid::network::PowerNetwork;
    use oxigrid::powerflow::{PowerFlowConfig, PowerFlowMethod};
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
    let net = PowerNetwork::from_matpower(path).expect("ieee14 parse");
    let cfg = PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: 50,
        tolerance: 1e-8,
        enforce_q_limits: false,
    };
    let res = net.solve_powerflow(&cfg).expect("NR solve");
    assert!(
        res.converged,
        "IEEE-14 NR did not converge after backend wiring"
    );
}
