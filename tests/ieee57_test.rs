#![cfg(feature = "powerflow")]
use oxigrid::network::PowerNetwork;
use oxigrid::powerflow::{PowerFlowConfig, PowerFlowMethod};

fn ieee57_network() -> PowerNetwork {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee57.m");
    PowerNetwork::from_matpower(path).expect("Failed to parse IEEE 57-bus data")
}

#[test]
fn test_ieee57_parse() {
    let network = ieee57_network();
    assert_eq!(network.bus_count(), 57, "bus count");
    assert_eq!(network.branch_count(), 80, "branch count");
    assert_eq!(network.generators.len(), 7, "generator count");
}

#[test]
fn test_ieee57_nr_convergence() {
    let network = ieee57_network();
    let config = PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: 50,
        tolerance: 1e-8,
        enforce_q_limits: false,
    };
    let result = network.solve_powerflow(&config).unwrap();
    assert!(
        result.converged,
        "IEEE 57-bus NR did not converge after {} iterations, max_mismatch={:.2e}",
        result.iterations, result.max_mismatch
    );
    assert!(
        result.iterations <= 15,
        "Too many iterations: {}",
        result.iterations
    );
}

#[test]
fn test_ieee57_slack_voltage() {
    let network = ieee57_network();
    let config = PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: 50,
        tolerance: 1e-8,
        enforce_q_limits: false,
    };
    let result = network.solve_powerflow(&config).unwrap();
    assert!(result.converged);
    // Slack bus 1: V = 1.04 p.u.
    assert!(
        (result.voltage_magnitude[0] - 1.04).abs() < 1e-6,
        "Slack V = {:.6}",
        result.voltage_magnitude[0]
    );
    // Slack bus angle = 0
    assert!(result.voltage_angle[0].abs() < 1e-10);
}

#[test]
fn test_ieee57_pv_voltages_held() {
    let network = ieee57_network();
    let config = PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: 50,
        tolerance: 1e-8,
        enforce_q_limits: false,
    };
    let result = network.solve_powerflow(&config).unwrap();
    assert!(result.converged);

    // PV bus voltage setpoints from the .m file
    let pv_expected = [
        (1, 1.010),  // bus 2
        (2, 0.985),  // bus 3
        (5, 0.980),  // bus 6
        (7, 1.005),  // bus 8
        (8, 0.980),  // bus 9
        (11, 1.015), // bus 12
    ];
    for (idx, v_exp) in pv_expected {
        let diff = (result.voltage_magnitude[idx] - v_exp).abs();
        assert!(
            diff < 1e-4,
            "PV bus {} Vm={:.5} expected={:.3} diff={:.6}",
            idx + 1,
            result.voltage_magnitude[idx],
            v_exp,
            diff
        );
    }
}

#[test]
fn test_ieee57_dc_powerflow() {
    let network = ieee57_network();
    let config = PowerFlowConfig {
        method: PowerFlowMethod::DcApproximation,
        max_iter: 1,
        tolerance: 1e-8,
        enforce_q_limits: false,
    };
    let result = network.solve_powerflow(&config).unwrap();
    assert!(result.converged);
    // DC: all voltage magnitudes = 1.0
    for vm in &result.voltage_magnitude {
        assert!((vm - 1.0).abs() < 1e-10);
    }
    // Slack bus angle = 0
    assert!(result.voltage_angle[0].abs() < 1e-10);
}

// ── Branch flow validation ────────────────────────────────────────────────────

fn nr_config() -> PowerFlowConfig {
    PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: 50,
        tolerance: 1e-8,
        enforce_q_limits: false,
    }
}

#[test]
fn test_ieee57_branch_flows_count() {
    let network = ieee57_network();
    let result = network.solve_powerflow(&nr_config()).unwrap();
    assert!(result.converged);
    assert_eq!(
        result.branch_flows.len(),
        80,
        "Should have 80 branch flow records"
    );
}

#[test]
fn test_ieee57_branch_flows_finite() {
    let network = ieee57_network();
    let result = network.solve_powerflow(&nr_config()).unwrap();
    assert!(result.converged);
    for (k, bf) in result.branch_flows.iter().enumerate() {
        assert!(bf.p_from_mw.is_finite(), "Branch {k} p_from not finite");
        assert!(bf.p_to_mw.is_finite(), "Branch {k} p_to not finite");
        assert!(bf.q_from_mvar.is_finite(), "Branch {k} q_from not finite");
        assert!(bf.q_to_mvar.is_finite(), "Branch {k} q_to not finite");
    }
}

#[test]
fn test_ieee57_total_losses_positive() {
    let network = ieee57_network();
    let result = network.solve_powerflow(&nr_config()).unwrap();
    assert!(result.converged);
    let total_loss: f64 = result.branch_flows.iter().map(|bf| bf.p_loss_mw).sum();
    assert!(
        total_loss > 0.0,
        "Total losses should be positive, got {total_loss:.4}"
    );
    // IEEE 57-bus: typical losses ~26 MW (within generous bounds)
    assert!(
        total_loss < 80.0,
        "Total losses {total_loss:.2} MW seem too high"
    );
}

#[test]
fn test_ieee57_branch_flows_conservation() {
    let network = ieee57_network();
    let result = network.solve_powerflow(&nr_config()).unwrap();
    assert!(result.converged);

    // Injection sum ≈ branch loss sum (Tellegen's theorem)
    let inj_sum: f64 = result.p_injected.iter().sum();
    let loss_sum: f64 = result.branch_flows.iter().map(|bf| bf.p_loss_mw).sum();
    assert!(
        (inj_sum - loss_sum).abs() < 1.0,
        "Injection sum {inj_sum:.3} MW ≠ branch loss sum {loss_sum:.3} MW"
    );
}

#[test]
fn test_ieee57_slack_branch_flow_plausible() {
    let network = ieee57_network();
    let result = network.solve_powerflow(&nr_config()).unwrap();
    assert!(result.converged);

    // Branch 0: bus 1→2. Plausibility check: large main feeder, should carry > 20 MW
    let p0 = result.branch_flows[0].p_from_mw;
    assert!(
        p0.abs() > 10.0 && p0.is_finite(),
        "Branch 0 P_from {p0:.2} MW implausible"
    );
}
