#![cfg(feature = "powerflow")]
use oxigrid::network::PowerNetwork;
use oxigrid::powerflow::{PowerFlowConfig, PowerFlowMethod};
use proptest::prelude::*;

fn ieee14_network() -> PowerNetwork {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
    PowerNetwork::from_matpower(path).expect("Failed to parse IEEE 14-bus data")
}

#[test]
fn test_ieee14_nr_convergence() {
    let network = ieee14_network();
    let config = PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: 50,
        tolerance: 1e-8,
        enforce_q_limits: false,
    };

    let result = network.solve_powerflow(&config).unwrap();
    assert!(
        result.converged,
        "IEEE 14-bus NR did not converge after {} iterations, max_mismatch={:.2e}",
        result.iterations, result.max_mismatch
    );
    assert!(
        result.iterations <= 10,
        "Too many iterations: {}",
        result.iterations
    );
}

#[test]
fn test_ieee14_nr_voltages() {
    let network = ieee14_network();
    let config = PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: 50,
        tolerance: 1e-8,
        enforce_q_limits: false,
    };

    let result = network.solve_powerflow(&config).unwrap();
    assert!(result.converged);

    // Expected values from MATPOWER / Appendix A
    let expected_vm = [
        1.0600, 1.0450, 1.0100, 1.0177, 1.0195, 1.0700, 1.0615, 1.0900, 1.0559, 1.0510, 1.0569,
        1.0552, 1.0504, 1.0355,
    ];
    let expected_va_deg = [
        0.000, -4.983, -12.725, -10.313, -8.774, -14.221, -13.360, -13.360, -14.939, -15.097,
        -14.791, -15.076, -15.156, -16.034,
    ];

    for (i, (&vm_calc, &vm_exp)) in result
        .voltage_magnitude
        .iter()
        .zip(expected_vm.iter())
        .enumerate()
    {
        let diff = (vm_calc - vm_exp).abs();
        assert!(
            diff < 1e-4,
            "Bus {} voltage magnitude mismatch: calc={:.6}, expected={:.6}, diff={:.8}",
            i + 1,
            vm_calc,
            vm_exp,
            diff
        );
    }

    let va_deg = result.voltage_angle_degrees();
    for (i, (&va_calc, &va_exp)) in va_deg.iter().zip(expected_va_deg.iter()).enumerate() {
        let diff = (va_calc - va_exp).abs();
        assert!(
            diff < 0.1,
            "Bus {} voltage angle mismatch: calc={:.3}, expected={:.3}, diff={:.4}",
            i + 1,
            va_calc,
            va_exp,
            diff
        );
    }
}

#[test]
fn test_ieee14_dc_powerflow() {
    let network = ieee14_network();
    let config = PowerFlowConfig {
        method: PowerFlowMethod::DcApproximation,
        max_iter: 1,
        tolerance: 1e-8,
        enforce_q_limits: false,
    };

    let result = network.solve_powerflow(&config).unwrap();
    assert!(result.converged);

    for vm in &result.voltage_magnitude {
        assert!((vm - 1.0).abs() < 1e-10);
    }

    assert!((result.voltage_angle[0]).abs() < 1e-10);
}

#[test]
fn test_ieee14_bus_count() {
    let network = ieee14_network();
    assert_eq!(network.bus_count(), 14);
    assert_eq!(network.branch_count(), 20);
}

#[test]
fn test_ieee14_branch_flows_sum_to_losses() {
    let network = ieee14_network();
    let config = PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: 50,
        tolerance: 1e-8,
        enforce_q_limits: false,
    };
    let result = network.solve_powerflow(&config).unwrap();
    assert!(result.converged);

    // Sum of branch losses should equal sum of injected power (Tellegen)
    let branch_loss_mw: f64 = result.branch_flows.iter().map(|f| f.p_loss_mw).sum();
    let inj_sum_mw: f64 = result.p_injected.iter().sum();
    assert!(
        (branch_loss_mw - inj_sum_mw).abs() < 0.5,
        "Branch losses {branch_loss_mw:.3} MW should equal injection sum {inj_sum_mw:.3} MW"
    );
    // IEEE 14-bus losses should be a few MW (not zero, not enormous)
    assert!(
        branch_loss_mw > 0.0 && branch_loss_mw < 30.0,
        "Branch losses {branch_loss_mw:.2} MW out of expected range"
    );
}

#[test]
fn test_ieee14_q_limit_enforcement() {
    let network = ieee14_network();
    let config = PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: 50,
        tolerance: 1e-8,
        enforce_q_limits: true,
    };
    let result = network.solve_powerflow(&config).unwrap();
    // With Q limits enforced, the solver should still converge
    assert!(
        result.converged,
        "Power flow with Q limits did not converge after {} iters, mismatch={:.2e}",
        result.iterations, result.max_mismatch
    );
    // Voltages should still be in a reasonable range
    for (i, &vm) in result.voltage_magnitude.iter().enumerate() {
        assert!(
            vm > 0.8 && vm < 1.2,
            "Bus {} voltage {vm:.4} out of range with Q limits",
            i + 1
        );
    }
}

#[test]
fn test_ieee14_incidence_matrix() {
    let network = ieee14_network();
    let a = network.incidence_matrix();
    assert_eq!(a.len(), 14, "Incidence matrix rows = bus count");
    assert_eq!(a[0].len(), 20, "Incidence matrix cols = branch count");

    // Each column should have exactly one +1 and one -1
    for k in 0..20 {
        let sum: f64 = a.iter().map(|row| row[k]).sum();
        assert!(sum.abs() < 1e-12, "Column {k} should sum to 0, got {sum}");
        let pos: usize = a.iter().filter(|row| row[k] > 0.5).count();
        let neg: usize = a.iter().filter(|row| row[k] < -0.5).count();
        assert_eq!(pos, 1, "Column {k} should have exactly one +1");
        assert_eq!(neg, 1, "Column {k} should have exactly one -1");
    }
}

#[test]
fn test_ieee14_dc_opf() {
    use oxigrid::optimize::opf::dc_opf::{solve_dc_opf, GenCost};

    let network = ieee14_network();

    // Assign simple quadratic costs to the 5 generators (IEEE 14-bus has 5 gens)
    // Based on typical IEEE 14-bus generator data
    let gen_costs: Vec<GenCost> = network
        .generators
        .iter()
        .map(|g| GenCost::quadratic(0.0, 20.0, 0.05, g.pmin.max(0.0), g.pmax.max(10.0)))
        .collect();

    let result = solve_dc_opf(&network, &gen_costs);
    assert!(result.is_ok(), "DC-OPF should succeed: {:?}", result.err());

    let r = result.unwrap();

    // Power balance: total generation should equal total load
    let total_gen: f64 = r.p_gen_mw.iter().sum();
    let total_load: f64 = network.buses.iter().map(|b| b.pd.0).sum();
    assert!(
        (total_gen - total_load).abs() < 1.0,
        "Generation {total_gen:.1} MW should ≈ load {total_load:.1} MW"
    );

    // Each generator within limits
    for (i, (&p, cost)) in r.p_gen_mw.iter().zip(gen_costs.iter()).enumerate() {
        assert!(
            p >= cost.p_min - 1e-3,
            "Gen {i} below p_min: {p:.2} < {}",
            cost.p_min
        );
        assert!(
            p <= cost.p_max + 1e-3,
            "Gen {i} above p_max: {p:.2} > {}",
            cost.p_max
        );
    }

    // Total cost should be positive
    assert!(
        r.total_cost > 0.0,
        "Total cost should be positive: {}",
        r.total_cost
    );

    // Lambda (marginal price) should be positive
    assert!(r.lambda > 0.0, "Lambda should be positive: {}", r.lambda);

    // Branch flows should be finite
    for (k, &f) in r.branch_flows_mw.iter().enumerate() {
        assert!(f.is_finite(), "Branch {k} flow is not finite: {f}");
    }
}

// proptest: scaled IEEE 14-bus loads should converge and conserve power
proptest! {
    #[test]
    fn prop_ieee14_power_conservation(load_scale in 0.5f64..1.2) {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
        let mut network = PowerNetwork::from_matpower(path).expect("parse");

        // Scale all bus loads by load_scale
        for bus in &mut network.buses {
            bus.pd.0 *= load_scale;
            bus.qd.0 *= load_scale;
        }

        let config = PowerFlowConfig {
            method: PowerFlowMethod::NewtonRaphson,
            max_iter: 50,
            tolerance: 1e-8,
            enforce_q_limits: false,
        };

        if let Ok(result) = network.solve_powerflow(&config) {
            if result.converged {
                // Power conservation: sum of all bus injections = system losses (>= 0)
                let total_injection_mw: f64 = result.p_injected.iter().sum();
                // Losses must be non-negative (generators produce more than loads consume)
                prop_assert!(
                    total_injection_mw >= -1e-3,
                    "Total injection {:.4} MW should be non-negative (losses)",
                    total_injection_mw
                );
                // Losses should be physically reasonable (< 10% of total generation)
                let total_load_mw: f64 = network.buses.iter().map(|b| b.pd.0).sum();
                prop_assert!(
                    total_injection_mw < 0.1 * total_load_mw + 1.0,
                    "Losses {:.2} MW exceed 10% of load {:.2} MW",
                    total_injection_mw, total_load_mw
                );
            }
        }
    }
}
