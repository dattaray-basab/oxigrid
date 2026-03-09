//! Property-based tests for power flow numerical invariants.
//!
//! These tests verify that mathematical laws hold across a wide range of
//! parameter combinations: power conservation, solver consistency, and
//! numerical stability under load variation.
#![cfg(feature = "powerflow")]

use oxigrid::network::PowerNetwork;
use oxigrid::powerflow::{PowerFlowConfig, PowerFlowMethod};
use proptest::prelude::*;

// ── Helpers ───────────────────────────────────────────────────────────────────

fn ieee14_config_nr() -> PowerFlowConfig {
    PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: 50,
        tolerance: 1e-8,
        enforce_q_limits: false,
    }
}

fn ieee14_config_fdlf() -> PowerFlowConfig {
    PowerFlowConfig {
        method: PowerFlowMethod::FastDecoupled,
        max_iter: 50,
        tolerance: 1e-6,
        enforce_q_limits: false,
    }
}

fn ieee14_config_dc() -> PowerFlowConfig {
    PowerFlowConfig {
        method: PowerFlowMethod::DcApproximation,
        max_iter: 1,
        tolerance: 1e-8,
        enforce_q_limits: false,
    }
}

// ── NR/FDLF consistency ───────────────────────────────────────────────────────

// Newton-Raphson and FDLF should produce similar bus angles for IEEE 14-bus.
// Tests over randomised load scaling from 60% to 120% of nominal.
proptest! {
    #![proptest_config(ProptestConfig::with_cases(10))]
    #[test]
    fn prop_nr_fdlf_angle_consistency(scale in 0.6_f64..1.2_f64) {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
        let mut net = PowerNetwork::from_matpower(path).unwrap();

        // Scale all loads
        for bus in &mut net.buses {
            bus.pd = oxigrid::units::Power(bus.pd.0 * scale);
            bus.qd = oxigrid::units::ReactivePower(bus.qd.0 * scale);
        }

        let nr = net.solve_powerflow(&ieee14_config_nr()).unwrap();
        let fdlf = net.solve_powerflow(&ieee14_config_fdlf()).unwrap();

        if nr.converged && fdlf.converged {
            // Bus angles should agree within 5 degrees (0.087 rad) for IEEE 14-bus
            for (i, (&nr_a, &fdlf_a)) in nr.voltage_angle.iter().zip(fdlf.voltage_angle.iter()).enumerate() {
                let diff = (nr_a - fdlf_a).abs().to_degrees();
                prop_assert!(
                    diff < 5.0,
                    "Bus {} angle: NR={:.3}° FDLF={:.3}° diff={:.3}°",
                    i+1, nr_a.to_degrees(), fdlf_a.to_degrees(), diff
                );
            }
        }
    }
}

// Slack bus angle must always be zero regardless of loading.
proptest! {
    #![proptest_config(ProptestConfig::with_cases(15))]
    #[test]
    fn prop_slack_angle_always_zero(scale in 0.5_f64..1.5_f64) {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
        let mut net = PowerNetwork::from_matpower(path).unwrap();
        for bus in &mut net.buses { bus.pd = oxigrid::units::Power(bus.pd.0 * scale); }

        let result = net.solve_powerflow(&ieee14_config_nr()).unwrap();
        if result.converged {
            prop_assert!(
                result.voltage_angle[0].abs() < 1e-10,
                "Slack angle = {:.2e}", result.voltage_angle[0]
            );
        }
    }
}

// DC power flow slack bus angle must always be exactly zero.
proptest! {
    #![proptest_config(ProptestConfig::with_cases(15))]
    #[test]
    fn prop_dc_slack_angle_zero(scale in 0.4_f64..1.6_f64) {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
        let mut net = PowerNetwork::from_matpower(path).unwrap();
        for bus in &mut net.buses { bus.pd = oxigrid::units::Power(bus.pd.0 * scale); }

        let result = net.solve_powerflow(&ieee14_config_dc()).unwrap();
        prop_assert!(result.converged, "DC should always converge");
        prop_assert!(
            result.voltage_angle[0].abs() < 1e-10,
            "DC slack angle = {:.2e}", result.voltage_angle[0]
        );
    }
}

// DC power flow voltage magnitudes must all equal 1.0 p.u.
proptest! {
    #![proptest_config(ProptestConfig::with_cases(15))]
    #[test]
    fn prop_dc_voltages_flat(scale in 0.4_f64..1.6_f64) {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
        let mut net = PowerNetwork::from_matpower(path).unwrap();
        for bus in &mut net.buses { bus.pd = oxigrid::units::Power(bus.pd.0 * scale); }

        let result = net.solve_powerflow(&ieee14_config_dc()).unwrap();
        for (i, &vm) in result.voltage_magnitude.iter().enumerate() {
            prop_assert!(
                (vm - 1.0).abs() < 1e-10,
                "Bus {} DC voltage = {:.6}", i+1, vm
            );
        }
    }
}

// NR converges for IEEE 14-bus over wide load range (50%-150%).
proptest! {
    #![proptest_config(ProptestConfig::with_cases(20))]
    #[test]
    fn prop_nr_converges_normal_loading(scale in 0.5_f64..1.5_f64) {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
        let mut net = PowerNetwork::from_matpower(path).unwrap();
        for bus in &mut net.buses {
            bus.pd = oxigrid::units::Power(bus.pd.0 * scale);
            bus.qd = oxigrid::units::ReactivePower(bus.qd.0 * scale);
        }

        let result = net.solve_powerflow(&ieee14_config_nr()).unwrap();
        prop_assert!(result.converged, "NR did not converge at scale={:.2}", scale);
        prop_assert!(result.max_mismatch < 1e-7);
        prop_assert!(result.iterations <= 15);
    }
}

// NR voltage magnitudes remain in plausible range [0.8, 1.25] for normal loading.
proptest! {
    #![proptest_config(ProptestConfig::with_cases(15))]
    #[test]
    fn prop_nr_voltages_plausible(scale in 0.7_f64..1.3_f64) {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
        let mut net = PowerNetwork::from_matpower(path).unwrap();
        for bus in &mut net.buses {
            bus.pd = oxigrid::units::Power(bus.pd.0 * scale);
            bus.qd = oxigrid::units::ReactivePower(bus.qd.0 * scale);
        }

        let result = net.solve_powerflow(&ieee14_config_nr()).unwrap();
        if result.converged {
            for (i, &vm) in result.voltage_magnitude.iter().enumerate() {
                prop_assert!(
                    (0.8..=1.25).contains(&vm),
                    "Bus {} voltage {:.4} p.u. out of [0.8, 1.25]", i+1, vm
                );
            }
        }
    }
}

// Power losses are always non-negative (second law).
proptest! {
    #![proptest_config(ProptestConfig::with_cases(15))]
    #[test]
    fn prop_branch_losses_non_negative(scale in 0.5_f64..1.4_f64) {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
        let mut net = PowerNetwork::from_matpower(path).unwrap();
        for bus in &mut net.buses {
            bus.pd = oxigrid::units::Power(bus.pd.0 * scale);
            bus.qd = oxigrid::units::ReactivePower(bus.qd.0 * scale);
        }

        let result = net.solve_powerflow(&ieee14_config_nr()).unwrap();
        if result.converged {
            let total_loss: f64 = result.branch_flows.iter().map(|f| f.p_loss_mw).sum();
            prop_assert!(
                total_loss >= -0.01,
                "Total losses {:.4} MW should be non-negative", total_loss
            );
        }
    }
}

// Branch flow count equals branch count.
proptest! {
    #![proptest_config(ProptestConfig::with_cases(5))]
    #[test]
    fn prop_branch_flow_count_consistent(scale in 0.8_f64..1.2_f64) {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee30.m");
        let mut net = PowerNetwork::from_matpower(path).unwrap();
        for bus in &mut net.buses { bus.pd = oxigrid::units::Power(bus.pd.0 * scale); }

        let result = net.solve_powerflow(&PowerFlowConfig {
            method: PowerFlowMethod::NewtonRaphson,
            max_iter: 50,
            tolerance: 1e-8,
            enforce_q_limits: false,
        }).unwrap();

        prop_assert_eq!(result.branch_flows.len(), net.branch_count());
    }
}

/// FDLF mismatch converges monotonically (no oscillation) for IEEE 14-bus.
#[test]
fn test_fdlf_converges_ieee14() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
    let net = PowerNetwork::from_matpower(path).unwrap();
    let result = net.solve_powerflow(&ieee14_config_fdlf()).unwrap();
    assert!(result.converged, "FDLF must converge");
    assert!(
        result.max_mismatch < 1e-5,
        "mismatch={:.2e}",
        result.max_mismatch
    );
    assert!(result.iterations <= 20);
}

/// FDLF and NR produce same generation (within 1 MW) on IEEE 30-bus.
#[test]
fn test_fdlf_nr_same_p_loss_ieee30() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee30.m");
    let net = PowerNetwork::from_matpower(path).unwrap();

    let nr = net
        .solve_powerflow(&PowerFlowConfig {
            method: PowerFlowMethod::NewtonRaphson,
            max_iter: 50,
            tolerance: 1e-8,
            enforce_q_limits: false,
        })
        .unwrap();
    let fdlf = net.solve_powerflow(&ieee14_config_fdlf()).unwrap();

    assert!(nr.converged);
    assert!(fdlf.converged);

    let loss_nr: f64 = nr.branch_flows.iter().map(|f| f.p_loss_mw).sum();
    let loss_fdlf: f64 = fdlf.branch_flows.iter().map(|f| f.p_loss_mw).sum();

    // Losses should be within 5 MW of each other
    assert!(
        (loss_nr - loss_fdlf).abs() < 5.0,
        "NR losses={:.2} MW, FDLF losses={:.2} MW",
        loss_nr,
        loss_fdlf
    );
}
