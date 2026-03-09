#![cfg(feature = "powerflow")]
use oxigrid::powerflow::state_estimation::{
    detect_bad_data, DcStateEstimator, Measurement, MeasurementType,
};

// ── 3-bus test helper ─────────────────────────────────────────────────────────
// Topology: 0(slack)─1 (x=0.1 pu), 1─2 (x=0.2 pu)
// B-bus diagonal:  B00=10, B11=15, B22=5
//        off-diag: B01=B10=-10, B12=B21=-5
fn three_bus_estimator() -> DcStateEstimator {
    let b_bus = vec![
        vec![10.0, -10.0, 0.0],
        vec![-10.0, 15.0, -5.0],
        vec![0.0, -5.0, 5.0],
    ];
    DcStateEstimator::new(3, 0, b_bus, vec![0, 1], vec![1, 2], vec![0.1, 0.2])
}

// ── Measurement constructors ──────────────────────────────────────────────────

#[test]
fn test_measurement_types() {
    let m1 = Measurement::power_injection(0, 0.5, 0.01);
    assert_eq!(m1.mtype, MeasurementType::PowerInjection);
    assert_eq!(m1.bus, 0);
    assert!(m1.to_bus.is_none());

    let m2 = Measurement::voltage(1, 1.02, 0.005);
    assert_eq!(m2.mtype, MeasurementType::VoltageMagnitude);

    let m3 = Measurement::branch_flow(0, 1, 0.3, 0.01);
    assert_eq!(m3.mtype, MeasurementType::BranchActivePower);
    assert_eq!(m3.to_bus, Some(1));
}

#[test]
fn test_measurement_weight() {
    let m = Measurement::power_injection(0, 0.5, 0.02);
    assert!(
        (m.weight() - 2500.0).abs() < 1e-6,
        "1/0.02² = 2500, got {}",
        m.weight()
    );
}

// ── State estimation: exact measurements ─────────────────────────────────────

#[test]
fn test_dc_se_exact_branch_flows() {
    // True angles: θ = [0, -0.05, -0.15] rad
    // P_01 = (0 - (-0.05)) / 0.1 = 0.5 pu
    // P_12 = ((-0.05) - (-0.15)) / 0.2 = 0.5 pu
    let est = three_bus_estimator();
    let meas = vec![
        Measurement::branch_flow(0, 1, 0.5, 0.01),
        Measurement::branch_flow(1, 2, 0.5, 0.01),
        Measurement::power_injection(2, -0.5, 0.01), // P2 = -0.5 pu (load)
    ];
    let result = est.estimate(&meas).unwrap();
    assert!(result.converged);
    assert!(
        result.theta[0].abs() < 1e-9,
        "Slack angle ≠ 0: {}",
        result.theta[0]
    );
    assert!(
        (result.theta[1] - (-0.05)).abs() < 1e-4,
        "θ₁ = {:.4}, expected -0.05",
        result.theta[1]
    );
    assert!(
        (result.theta[2] - (-0.15)).abs() < 1e-4,
        "θ₂ = {:.4}, expected -0.15",
        result.theta[2]
    );
}

#[test]
fn test_dc_se_residuals_near_zero_exact() {
    let est = three_bus_estimator();
    let meas = vec![
        Measurement::branch_flow(0, 1, 0.5, 0.01),
        Measurement::branch_flow(1, 2, 0.5, 0.01),
        Measurement::power_injection(2, -0.5, 0.01),
    ];
    let result = est.estimate(&meas).unwrap();
    for (i, &r) in result.residuals.iter().enumerate() {
        assert!(
            r.abs() < 1e-6,
            "Residual [{i}] = {r:.2e} should be near 0 for exact measurements"
        );
    }
}

// ── State estimation: redundant measurements ──────────────────────────────────

#[test]
fn test_dc_se_overdetermined() {
    // 4 measurements for 2 unknowns (θ₁, θ₂)
    let est = three_bus_estimator();
    let meas = vec![
        Measurement::branch_flow(0, 1, 0.5, 0.01),
        Measurement::branch_flow(1, 2, 0.5, 0.01),
        Measurement::power_injection(1, 0.0, 0.01),
        Measurement::power_injection(2, -0.5, 0.01),
    ];
    let result = est.estimate(&meas).unwrap();
    assert!(result.converged);
    assert_eq!(result.dof, 2, "dof = n_meas - n_states = 4 - 2 = 2");
    // Angles should still be close to true values
    assert!((result.theta[1] - (-0.05)).abs() < 5e-3);
    assert!((result.theta[2] - (-0.15)).abs() < 5e-3);
}

#[test]
fn test_dc_se_normalised_chi2_small_for_consistent_data() {
    let est = three_bus_estimator();
    let meas = vec![
        Measurement::branch_flow(0, 1, 0.5, 0.01),
        Measurement::branch_flow(1, 2, 0.5, 0.01),
        Measurement::power_injection(1, 0.0, 0.01),
        Measurement::power_injection(2, -0.5, 0.01),
    ];
    let result = est.estimate(&meas).unwrap();
    let chi2_norm = result.normalised_chi2();
    assert!(
        chi2_norm < 10.0,
        "Chi² / dof = {chi2_norm:.4} should be small for consistent data"
    );
}

// ── State estimation: noisy measurements ─────────────────────────────────────

#[test]
fn test_dc_se_noisy_converges() {
    let est = three_bus_estimator();
    // Add small noise to measurements
    let meas = vec![
        Measurement::branch_flow(0, 1, 0.502, 0.01), // slight noise
        Measurement::branch_flow(1, 2, 0.498, 0.01),
        Measurement::power_injection(1, 0.003, 0.01),
        Measurement::power_injection(2, -0.498, 0.01),
    ];
    let result = est.estimate(&meas).unwrap();
    assert!(result.converged);
    // Despite noise, angles should be close to true values
    assert!((result.theta[1] - (-0.05)).abs() < 0.01);
    assert!((result.theta[2] - (-0.15)).abs() < 0.01);
}

// ── Bad data detection ────────────────────────────────────────────────────────

#[test]
fn test_bad_data_flags_outlier() {
    let meas = vec![
        Measurement::power_injection(0, 0.5, 0.01),
        Measurement::power_injection(1, 100.0, 0.01), // bad: 10000σ from true
    ];
    let residuals = [0.001_f64, 1.0];
    let bad = detect_bad_data(&residuals, &meas, 3.0);
    assert_eq!(bad, vec![1], "Should flag measurement 1 as bad data");
}

#[test]
fn test_bad_data_no_false_positives() {
    let meas = vec![
        Measurement::power_injection(0, 0.5, 0.01),
        Measurement::power_injection(1, -0.3, 0.01),
    ];
    let residuals = [0.005_f64, -0.003]; // 0.5σ and 0.3σ — within threshold
    let bad = detect_bad_data(&residuals, &meas, 3.0);
    assert!(
        bad.is_empty(),
        "No bad data should be detected for small residuals"
    );
}

#[test]
fn test_bad_data_threshold_boundary() {
    let meas = vec![Measurement::power_injection(0, 1.0, 0.01)];
    // residual = 0.03 = exactly 3σ — should NOT be flagged (> not >=)
    let residuals = [0.03_f64];
    let bad = detect_bad_data(&residuals, &meas, 3.0);
    assert!(
        bad.is_empty(),
        "Residual exactly at threshold should not be flagged"
    );
}

// ── Under-determined / edge cases ────────────────────────────────────────────

#[test]
fn test_dc_se_underdetermined_returns_error() {
    let est = three_bus_estimator();
    // Only 1 measurement for 2 unknowns → under-determined
    let meas = vec![Measurement::branch_flow(0, 1, 0.5, 0.01)];
    assert!(
        est.estimate(&meas).is_err(),
        "Under-determined system should return error"
    );
}

#[test]
fn test_dc_se_exactly_determined() {
    // Exactly 2 measurements for 2 unknowns
    let est = three_bus_estimator();
    let meas = vec![
        Measurement::branch_flow(0, 1, 0.5, 0.01),
        Measurement::branch_flow(1, 2, 0.5, 0.01),
    ];
    let result = est.estimate(&meas).unwrap();
    assert!(result.converged);
    assert_eq!(result.dof, 0); // exactly determined
}

// ── IEEE 14-bus state estimation ──────────────────────────────────────────────

#[test]
fn test_dc_se_ieee14_from_power_flow() {
    use oxigrid::network::reduction::build_b_bus;
    use oxigrid::network::PowerNetwork;

    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
    if let Ok(net) = PowerNetwork::from_matpower(path) {
        let n = net.bus_count();
        let slack_idx = net.slack_bus_index().unwrap();

        // Build B-bus
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
        let b_bus = build_b_bus(n, &branch_from, &branch_to, &branch_x);

        // Build measurements: one injection per bus (from nominal p.u. dispatch)
        let (p_inj, _) = net.net_injection();
        let mut meas: Vec<Measurement> = p_inj
            .iter()
            .enumerate()
            .map(|(i, &p)| Measurement::power_injection(i, p, 0.01))
            .collect();
        // Also add branch flows for redundancy
        for (&from, &to) in branch_from.iter().zip(branch_to.iter()) {
            // Approximate flow from flat start
            meas.push(Measurement::branch_flow(from, to, 0.0, 0.1));
        }

        let est = DcStateEstimator::new(n, slack_idx, b_bus, branch_from, branch_to, branch_x);
        let result = est.estimate(&meas);
        assert!(
            result.is_ok(),
            "IEEE 14-bus SE should succeed: {:?}",
            result.err()
        );
        let r = result.unwrap();
        assert!(r.converged);
        // Slack angle should be zero
        assert!(
            r.theta[slack_idx].abs() < 1e-9,
            "Slack angle ≠ 0: {}",
            r.theta[slack_idx]
        );
    }
}
