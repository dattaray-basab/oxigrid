#![cfg(feature = "powerflow")]
use oxigrid::network::reduction::{
    build_b_bus, dc_solve, kron_reduce, lodf_matrix, ptdf_matrix, retained_buses,
};
use oxigrid::network::PowerNetwork;

// ── B-bus construction ────────────────────────────────────────────────────────

#[test]
fn test_build_b_bus_single_line() {
    // Single line 0-1, x = 0.1
    let b = build_b_bus(2, &[0], &[1], &[0.1]);
    assert!((b[0][0] - 10.0).abs() < 1e-9, "B00 = 1/x = 10");
    assert!((b[1][1] - 10.0).abs() < 1e-9, "B11 = 1/x = 10");
    assert!((b[0][1] + 10.0).abs() < 1e-9, "B01 = -1/x = -10");
    assert!((b[1][0] + 10.0).abs() < 1e-9, "B10 = -1/x = -10");
}

#[test]
fn test_build_b_bus_3bus() {
    // 3-bus: 0-1 (x=0.1), 1-2 (x=0.2), 0-2 (x=0.4)
    let from = [0, 1, 0];
    let to = [1, 2, 2];
    let x = [0.1, 0.2, 0.4];
    let b = build_b_bus(3, &from, &to, &x);
    assert!((b[0][0] - 12.5).abs() < 1e-9, "B00 = 10 + 2.5 = 12.5");
    assert!((b[1][1] - 15.0).abs() < 1e-9, "B11 = 10 + 5 = 15");
    assert!((b[2][2] - 7.5).abs() < 1e-9, "B22 = 5 + 2.5 = 7.5");
    assert!((b[0][1] + 10.0).abs() < 1e-9);
    assert!((b[1][2] + 5.0).abs() < 1e-9);
    assert!((b[0][2] + 2.5).abs() < 1e-9);
}

#[test]
fn test_build_b_bus_symmetric() {
    let from = [0, 1, 0];
    let to = [1, 2, 2];
    let x = [0.1, 0.2, 0.4];
    let b = build_b_bus(3, &from, &to, &x);
    for (i, b_row) in b.iter().enumerate() {
        for (j, b_val) in b_row.iter().enumerate() {
            assert!(
                (*b_val - b[j][i]).abs() < 1e-12,
                "B[{i}][{j}] ≠ B[{j}][{i}]"
            );
        }
    }
}

#[test]
fn test_build_b_bus_zero_reactance_skipped() {
    // Branch with x=0 should be skipped (division by zero guard)
    let b = build_b_bus(2, &[0], &[1], &[0.0]);
    assert!(
        b[0][0].abs() < 1e-9,
        "Zero-reactance branch should be ignored"
    );
}

// ── DC power flow solve ───────────────────────────────────────────────────────

#[test]
fn test_dc_solve_slack_angle_zero() {
    let from = [0, 1];
    let to = [1, 2];
    let x = [0.1, 0.2];
    let b = build_b_bus(3, &from, &to, &x);
    let p = [0.0, 0.5, -0.5];
    let theta = dc_solve(&b, &p, 0).unwrap();
    assert!(
        theta[0].abs() < 1e-12,
        "Slack angle must be 0: {}",
        theta[0]
    );
}

#[test]
fn test_dc_solve_power_balance() {
    let from = [0, 1, 0];
    let to = [1, 2, 2];
    let x = [0.1, 0.2, 0.4];
    let b = build_b_bus(3, &from, &to, &x);
    let p_inj = [0.0, 0.5, -0.5];
    let theta = dc_solve(&b, &p_inj, 0).unwrap();

    // Verify: P_ij = (θ_i - θ_j) / x_ij → sum of flows out of each bus = injection
    let p01 = (theta[0] - theta[1]) / x[0];
    let p12 = (theta[1] - theta[2]) / x[1];
    let p02 = (theta[0] - theta[2]) / x[2];
    let _net_0 = p01 + p02; // flows out of bus 0 (slack, not checked)
    let net_1 = -p01 + p12; // flows out of bus 1 (net injection = -net_out)
    let net_2 = -p12 - p02; // flows out of bus 2
                            // Bus 1: net_injection = 0.5 → power balanced
    assert!((net_1 - p_inj[1]).abs() < 1e-9, "Bus 1 balance: {net_1:.4}");
    assert!((net_2 - p_inj[2]).abs() < 1e-9, "Bus 2 balance: {net_2:.4}");
}

#[test]
fn test_dc_solve_radial_network() {
    // Radial: 0-1 (x=0.1), 1-2 (x=0.1)
    let b = build_b_bus(3, &[0, 1], &[1, 2], &[0.1, 0.1]);
    let p = [0.0, 0.0, -1.0]; // bus 0 slack, bus 2 loads 1 p.u.
    let theta = dc_solve(&b, &p, 0).unwrap();
    // Flow on 0-1 = 1.0 pu → θ0 - θ1 = 1.0 * 0.1 = 0.1 rad
    assert!(
        (theta[0] - theta[1] - 0.1).abs() < 1e-6,
        "θ0-θ1 = {:.4}",
        theta[0] - theta[1]
    );
    // Flow on 1-2 = 1.0 pu → θ1 - θ2 = 0.1 rad
    assert!(
        (theta[1] - theta[2] - 0.1).abs() < 1e-6,
        "θ1-θ2 = {:.4}",
        theta[1] - theta[2]
    );
}

// ── Retained buses ────────────────────────────────────────────────────────────

#[test]
fn test_retained_buses_basic() {
    let r = retained_buses(5, &[1, 3]);
    assert_eq!(r, vec![0, 2, 4]);
}

#[test]
fn test_retained_buses_none_interior() {
    let r = retained_buses(4, &[]);
    assert_eq!(r, vec![0, 1, 2, 3]);
}

#[test]
fn test_retained_buses_all_interior() {
    let r = retained_buses(3, &[0, 1, 2]);
    assert_eq!(r, Vec::<usize>::new());
}

#[test]
fn test_retained_buses_duplicates_ignored() {
    let r = retained_buses(4, &[1, 1, 2]);
    assert_eq!(r, vec![0, 3]);
}

// ── Kron reduction ────────────────────────────────────────────────────────────

#[test]
fn test_kron_reduce_no_interior() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
    if let Ok(net) = PowerNetwork::from_matpower(path) {
        let ybus = net.admittance_matrix().unwrap();
        let yr = kron_reduce(&ybus, &[]).unwrap();
        // No reduction: should return full matrix
        assert_eq!(yr.len(), net.bus_count());
    }
}

#[test]
fn test_kron_reduce_dimensions() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
    if let Ok(net) = PowerNetwork::from_matpower(path) {
        let ybus = net.admittance_matrix().unwrap();
        let n = net.bus_count();
        // Retain first 4 buses, eliminate the rest
        let interior: Vec<usize> = (4..n).collect();
        let yr = kron_reduce(&ybus, &interior).unwrap();
        assert_eq!(yr.len(), 4, "Reduced matrix should have 4 rows");
        assert_eq!(yr[0].len(), 4, "Reduced matrix should have 4 cols");
    }
}

#[test]
fn test_kron_reduce_symmetric() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
    if let Ok(net) = PowerNetwork::from_matpower(path) {
        let ybus = net.admittance_matrix().unwrap();
        let interior: Vec<usize> = (2..14).collect();
        let yr = kron_reduce(&ybus, &interior).unwrap();
        // Reduced Y-bus should be symmetric (for reciprocal network)
        for (i, yr_row) in yr.iter().enumerate() {
            for (j, yr_val) in yr_row.iter().enumerate() {
                let diff = (*yr_val - yr[j][i]).norm();
                assert!(diff < 1e-8, "Yr[{i}][{j}] ≠ Yr[{j}][{i}]: diff={diff:.2e}");
            }
        }
    }
}

#[test]
fn test_kron_reduce_diagonal_nonzero() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
    if let Ok(net) = PowerNetwork::from_matpower(path) {
        let ybus = net.admittance_matrix().unwrap();
        let interior: Vec<usize> = (2..14).collect();
        let yr = kron_reduce(&ybus, &interior).unwrap();
        for (i, row) in yr.iter().enumerate() {
            assert!(
                row[i].norm() > 0.0,
                "Diagonal Yr[{i}][{i}] should be nonzero"
            );
        }
    }
}

// ── PTDF matrix ───────────────────────────────────────────────────────────────

#[test]
fn test_ptdf_shape() {
    let from = [0usize, 1, 0];
    let to = [1usize, 2, 2];
    let x = [0.1_f64, 0.2, 0.4];
    let b = build_b_bus(3, &from, &to, &x);
    let ptdf = ptdf_matrix(&b, &from, &to, &x, 0).unwrap();
    assert_eq!(ptdf.len(), 3, "PTDF rows = n_branch");
    assert_eq!(ptdf[0].len(), 3, "PTDF cols = n_bus");
}

#[test]
fn test_ptdf_slack_bus_zero() {
    let from = [0usize, 1, 0];
    let to = [1usize, 2, 2];
    let x = [0.1_f64, 0.2, 0.4];
    let b = build_b_bus(3, &from, &to, &x);
    let ptdf = ptdf_matrix(&b, &from, &to, &x, 0).unwrap();
    for row in &ptdf {
        assert!(
            row[0].abs() < 1e-10,
            "PTDF at slack bus (idx 0) must be 0, got {}",
            row[0]
        );
    }
}

#[test]
fn test_ptdf_flow_on_parallel_branch() {
    // 3-bus network with 3 lines forming a mesh
    let from = [0usize, 1, 0];
    let to = [1usize, 2, 2];
    let x = [0.1_f64, 0.2, 0.4];
    let b = build_b_bus(3, &from, &to, &x);
    let ptdf = ptdf_matrix(&b, &from, &to, &x, 0).unwrap();
    // All PTDFs must be finite
    for row in &ptdf {
        for &v in row {
            assert!(v.is_finite(), "PTDF value should be finite: {v}");
        }
    }
    // PTDF at slack bus = 0 (already tested, but verify)
    for row in &ptdf {
        assert!(row[0].abs() < 1e-10, "PTDF at slack = {}", row[0]);
    }
    // PTDFs at non-slack buses should be non-trivial (at least one per row)
    for (l, row) in ptdf.iter().enumerate() {
        let max_abs = row.iter().map(|v| v.abs()).fold(0.0_f64, f64::max);
        assert!(max_abs > 1e-6, "PTDF row {l} should have nonzero entries");
    }
}

// ── LODF matrix ───────────────────────────────────────────────────────────────

#[test]
fn test_lodf_diagonal_is_minus_one() {
    let from = [0usize, 1, 0];
    let to = [1usize, 2, 2];
    let x = [0.1_f64, 0.2, 0.4];
    let b = build_b_bus(3, &from, &to, &x);
    let ptdf = ptdf_matrix(&b, &from, &to, &x, 0).unwrap();
    let lodf = lodf_matrix(&ptdf, &from, &to);
    for (l, lodf_row) in lodf.iter().enumerate().take(3) {
        assert!(
            (lodf_row[l] + 1.0).abs() < 1e-9,
            "LODF[{l}][{l}] should be -1"
        );
    }
}

#[test]
fn test_lodf_shape() {
    let from = [0usize, 1];
    let to = [1usize, 2];
    let x = [0.1_f64, 0.2];
    let b = build_b_bus(3, &from, &to, &x);
    let ptdf = ptdf_matrix(&b, &from, &to, &x, 0).unwrap();
    let lodf = lodf_matrix(&ptdf, &from, &to);
    assert_eq!(lodf.len(), 2);
    assert_eq!(lodf[0].len(), 2);
}

// ── IEEE 14-bus integration ───────────────────────────────────────────────────

#[test]
fn test_dc_solve_ieee14() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
    if let Ok(net) = PowerNetwork::from_matpower(path) {
        let n = net.bus_count();
        let slack_idx = net.slack_bus_index().unwrap();
        let branch_from: Vec<usize> = net
            .branches
            .iter()
            .map(|b| net.bus_index(b.from_bus).unwrap())
            .collect();
        let branch_to: Vec<usize> = net
            .branches
            .iter()
            .map(|b| net.bus_index(b.to_bus).unwrap())
            .collect();
        let branch_x: Vec<f64> = net.branches.iter().map(|b| b.x).collect();
        let b = build_b_bus(n, &branch_from, &branch_to, &branch_x);
        let (p_inj, _) = net.net_injection();
        let theta = dc_solve(&b, &p_inj, slack_idx).unwrap();
        assert!(
            theta[slack_idx].abs() < 1e-12,
            "Slack angle = {}",
            theta[slack_idx]
        );
        // All angles should be finite
        for &t in &theta {
            assert!(t.is_finite(), "Angle is not finite: {t}");
        }
    }
}

#[test]
fn test_ptdf_ieee14() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
    if let Ok(net) = PowerNetwork::from_matpower(path) {
        let n = net.bus_count();
        let slack_idx = net.slack_bus_index().unwrap();
        let branch_from: Vec<usize> = net
            .branches
            .iter()
            .map(|b| net.bus_index(b.from_bus).unwrap())
            .collect();
        let branch_to: Vec<usize> = net
            .branches
            .iter()
            .map(|b| net.bus_index(b.to_bus).unwrap())
            .collect();
        let branch_x: Vec<f64> = net.branches.iter().map(|b| b.x).collect();
        let b = build_b_bus(n, &branch_from, &branch_to, &branch_x);
        let ptdf = ptdf_matrix(&b, &branch_from, &branch_to, &branch_x, slack_idx).unwrap();
        // Shape check
        assert_eq!(ptdf.len(), net.branch_count());
        assert_eq!(ptdf[0].len(), n);
        // Slack column should be zero
        for row in &ptdf {
            assert!(row[slack_idx].abs() < 1e-9);
        }
    }
}
