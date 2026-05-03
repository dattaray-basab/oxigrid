//! IEEE 300-bus power flow integration test.
//!
//! Loads the synthetic IEEE 300-bus MATPOWER file and validates that
//! the Newton-Raphson power flow converges with physically plausible results.
#![cfg(feature = "powerflow")]

use oxigrid::network::PowerNetwork;
use oxigrid::powerflow::{PowerFlowConfig, PowerFlowMethod};

fn ieee300_network() -> PowerNetwork {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee300.m");
    PowerNetwork::from_matpower(path).expect("Failed to parse IEEE 300-bus MATPOWER data")
}

fn nr_config() -> PowerFlowConfig {
    PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: 80,
        tolerance: 1e-8,
        enforce_q_limits: false,
    }
}

#[test]
fn test_ieee300_parse_bus_count() {
    let network = ieee300_network();
    assert_eq!(network.bus_count(), 300, "IEEE-300 should have 300 buses");
}

#[test]
fn test_ieee300_nr_converges() {
    let network = ieee300_network();
    let result = network
        .solve_powerflow(&nr_config())
        .expect("IEEE-300 NR solve failed");
    assert!(
        result.converged,
        "IEEE-300 NR did not converge in 80 iterations (max_mismatch={:.3e})",
        result.max_mismatch
    );
    assert!(
        result.max_mismatch < 1e-6,
        "Mismatch too large: {:.3e}",
        result.max_mismatch
    );
}

#[test]
fn test_ieee300_slack_voltage() {
    let network = ieee300_network();
    let result = network
        .solve_powerflow(&nr_config())
        .expect("IEEE-300 NR solve failed");
    assert!(result.converged);
    // Slack bus (bus 1) has Vg = 1.04 p.u. in case300
    let slack_idx = network
        .slack_bus_index()
        .expect("IEEE-300 slack bus not found");
    let v_slack = result.voltage_magnitude[slack_idx];
    assert!(
        (v_slack - 1.04).abs() < 1e-6,
        "Slack bus voltage {:.6} p.u., expected 1.04",
        v_slack
    );
    assert!(
        result.voltage_angle[slack_idx].abs() < 1e-10,
        "Slack bus angle {:.2e} rad, expected 0",
        result.voltage_angle[slack_idx]
    );
}

#[test]
fn test_ieee300_branch_flows_count() {
    let network = ieee300_network();
    let result = network
        .solve_powerflow(&nr_config())
        .expect("IEEE-300 NR solve failed");
    assert!(result.converged);
    // Branch count is whatever the MATPOWER file provides; must be nonzero
    assert!(
        !result.branch_flows.is_empty(),
        "No branch flows returned for IEEE-300"
    );
}

#[test]
fn test_ieee300_branch_flows_finite() {
    let network = ieee300_network();
    let result = network
        .solve_powerflow(&nr_config())
        .expect("IEEE-300 NR solve failed");
    assert!(result.converged);
    for (k, bf) in result.branch_flows.iter().enumerate() {
        assert!(bf.p_from_mw.is_finite(), "Branch {k} p_from not finite");
        assert!(bf.p_to_mw.is_finite(), "Branch {k} p_to not finite");
        assert!(bf.q_from_mvar.is_finite(), "Branch {k} q_from not finite");
        assert!(bf.q_to_mvar.is_finite(), "Branch {k} q_to not finite");
    }
}

#[test]
fn test_ieee300_voltages_in_range() {
    let network = ieee300_network();
    let result = network
        .solve_powerflow(&nr_config())
        .expect("IEEE-300 NR solve failed");
    assert!(result.converged);
    for (i, &vm) in result.voltage_magnitude.iter().enumerate() {
        assert!(
            vm > 0.8 && vm < 1.2,
            "Bus {} voltage {:.4} p.u. outside [0.8, 1.2] range",
            i + 1,
            vm
        );
    }
}

#[test]
fn test_ieee300_dc_powerflow() {
    let network = ieee300_network();
    let config = PowerFlowConfig {
        method: PowerFlowMethod::DcApproximation,
        max_iter: 1,
        tolerance: 1e-8,
        enforce_q_limits: false,
    };
    let result = network
        .solve_powerflow(&config)
        .expect("IEEE-300 DC solve failed");
    assert!(result.converged, "IEEE-300 DC power flow failed");
    // DC: all voltage magnitudes = 1.0
    for (i, vm) in result.voltage_magnitude.iter().enumerate() {
        assert!(
            (vm - 1.0).abs() < 1e-10,
            "Bus {} DC voltage magnitude = {:.6}, expected 1.0",
            i + 1,
            vm
        );
    }
    let slack_idx = network
        .slack_bus_index()
        .expect("IEEE-300 slack bus not found");
    assert!(
        result.voltage_angle[slack_idx].abs() < 1e-10,
        "Slack bus DC angle {:.2e}, expected 0",
        result.voltage_angle[slack_idx]
    );
}
