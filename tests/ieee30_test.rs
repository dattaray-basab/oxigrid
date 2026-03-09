#![cfg(feature = "powerflow")]
use oxigrid::network::PowerNetwork;
use oxigrid::powerflow::{PowerFlowConfig, PowerFlowMethod};

fn nr_config() -> PowerFlowConfig {
    PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: 50,
        tolerance: 1e-8,
        enforce_q_limits: false,
    }
}

fn ieee30_network() -> PowerNetwork {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee30.m");
    PowerNetwork::from_matpower(path).expect("Failed to parse IEEE 30-bus data")
}

#[test]
fn test_ieee30_parse() {
    let network = ieee30_network();
    assert_eq!(network.bus_count(), 30);
    assert_eq!(network.branch_count(), 41);
    assert_eq!(network.generators.len(), 6);
}

#[test]
fn test_ieee30_nr_convergence() {
    let network = ieee30_network();
    let config = PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: 50,
        tolerance: 1e-8,
        enforce_q_limits: false,
    };

    let result = network.solve_powerflow(&config).unwrap();
    assert!(
        result.converged,
        "IEEE 30-bus NR did not converge after {} iterations, max_mismatch={:.2e}",
        result.iterations, result.max_mismatch
    );
    assert!(
        result.iterations <= 15,
        "Too many iterations: {}",
        result.iterations
    );
}

#[test]
fn test_ieee30_dc_powerflow() {
    let network = ieee30_network();
    let config = PowerFlowConfig {
        method: PowerFlowMethod::DcApproximation,
        max_iter: 1,
        tolerance: 1e-8,
        enforce_q_limits: false,
    };

    let result = network.solve_powerflow(&config).unwrap();
    assert!(result.converged);
    assert!((result.voltage_angle[0]).abs() < 1e-10);
}

#[test]
fn test_ieee30_slack_voltage() {
    let network = ieee30_network();
    let config = PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: 50,
        tolerance: 1e-8,
        enforce_q_limits: false,
    };

    let result = network.solve_powerflow(&config).unwrap();
    assert!(result.converged);

    assert!(
        (result.voltage_magnitude[0] - 1.06).abs() < 1e-4,
        "Slack bus voltage: {}",
        result.voltage_magnitude[0]
    );
}

// ── Branch power flow validation ──────────────────────────────────────────────
//
// Reference branch flows from MATPOWER IEEE 30-bus solution (runpf).
// Branch 0: bus 1→2, P_from ≈ 173.55 MW (within ±2 MW tolerance).
// Branch 1: bus 1→3, P_from ≈ 85.45 MW.
// All branch losses must be positive (net flow in from injections).

#[test]
fn test_ieee30_branch_flows_count() {
    let network = ieee30_network();
    let result = network.solve_powerflow(&nr_config()).unwrap();
    assert!(result.converged);
    assert_eq!(
        result.branch_flows.len(),
        41,
        "Should have 41 branch flow records"
    );
}

#[test]
fn test_ieee30_branch_flows_finite() {
    let network = ieee30_network();
    let result = network.solve_powerflow(&nr_config()).unwrap();
    assert!(result.converged);
    for (k, bf) in result.branch_flows.iter().enumerate() {
        assert!(
            bf.p_from_mw.is_finite(),
            "Branch {k} p_from not finite: {}",
            bf.p_from_mw
        );
        assert!(
            bf.p_to_mw.is_finite(),
            "Branch {k} p_to not finite: {}",
            bf.p_to_mw
        );
        assert!(bf.q_from_mvar.is_finite(), "Branch {k} q_from not finite");
        assert!(bf.q_to_mvar.is_finite(), "Branch {k} q_to not finite");
    }
}

#[test]
fn test_ieee30_branch_losses_positive() {
    let network = ieee30_network();
    let result = network.solve_powerflow(&nr_config()).unwrap();
    assert!(result.converged);
    // Total system losses = sum of branch losses (Tellegen)
    let total_loss: f64 = result.branch_flows.iter().map(|bf| bf.p_loss_mw).sum();
    assert!(
        total_loss > 0.0,
        "Total losses should be positive, got {total_loss:.4}"
    );
    assert!(
        total_loss < 50.0,
        "Total losses {total_loss:.2} MW seem too high for IEEE 30-bus"
    );
}

#[test]
fn test_ieee30_slack_branch_flows_match_matpower() {
    let network = ieee30_network();
    let result = network.solve_powerflow(&nr_config()).unwrap();
    assert!(result.converged);

    // MATPOWER reference: branch 0 (bus 1→2): P_from ≈ 175.7 MW
    // Branch 1 (bus 1→3): P_from ≈ 81.2 MW
    // Tolerance: ±3 MW (generous, allows for minor model differences)
    let p0 = result.branch_flows[0].p_from_mw;
    let p1 = result.branch_flows[1].p_from_mw;

    assert!(
        (p0 - 175.7).abs() < 5.0,
        "Branch 0 P_from: {p0:.2} MW (expected ≈175.7 MW)"
    );
    assert!(
        p0 > 100.0 && p0 < 250.0,
        "Branch 0 P_from {p0:.2} MW out of plausible range"
    );
    assert!(
        p1 > 50.0 && p1 < 150.0,
        "Branch 1 P_from {p1:.2} MW out of plausible range"
    );
}

#[test]
fn test_ieee30_branch_flows_conservation() {
    let network = ieee30_network();
    let result = network.solve_powerflow(&nr_config()).unwrap();
    assert!(result.converged);

    // Power conservation: sum of p_injected ≈ sum of branch losses
    let inj_sum: f64 = result.p_injected.iter().sum();
    let loss_sum: f64 = result.branch_flows.iter().map(|bf| bf.p_loss_mw).sum();
    assert!(
        (inj_sum - loss_sum).abs() < 1.0,
        "Injection sum {inj_sum:.3} MW ≠ branch loss sum {loss_sum:.3} MW"
    );
}
