//! Integration tests for `src/testcases/` module.
//!
//! Verifies bus/branch counts, connectivity, and basic structural invariants
//! for all IEEE standard cases, synthetic generators, distribution cases,
//! and the benchmark suite.

use oxigrid::testcases::{
    benchmark::power_flow_benchmarks,
    distribution::{ieee33, ieee69, lv_european_residential, mv_urban_feeder},
    ieee::{ieee118, ieee14, ieee30, ieee300, ieee57, pegase89, rts96},
    synthetic::{generate_synthetic_network, NetworkTopology, SyntheticNetworkConfig},
};

// ---------------------------------------------------------------------------
// IEEE standard test cases
// ---------------------------------------------------------------------------

#[test]
fn test_ieee14_bus_count() {
    let net = ieee14().expect("ieee14 must build");
    assert_eq!(net.buses.len(), 14, "IEEE 14 must have 14 buses");
    assert!(
        net.branches.len() >= 16,
        "IEEE 14 must have at least 16 branches, got {}",
        net.branches.len()
    );
}

#[test]
fn test_ieee14_generator_count() {
    let net = ieee14().expect("ieee14 must build");
    assert_eq!(net.generators.len(), 5, "IEEE 14 has 5 generators");
}

#[test]
fn test_ieee14_has_slack() {
    use oxigrid::network::bus::BusType;
    let net = ieee14().expect("ieee14 must build");
    let slack_count = net
        .buses
        .iter()
        .filter(|b| b.bus_type == BusType::Slack)
        .count();
    assert_eq!(slack_count, 1, "IEEE 14 must have exactly 1 slack bus");
}

#[test]
fn test_ieee14_connected() {
    let net = ieee14().expect("ieee14 must build");
    assert!(net.is_connected(), "IEEE 14 must be connected");
}

#[test]
fn test_ieee14_valid() {
    let net = ieee14().expect("ieee14 must build");
    net.validate().expect("IEEE 14 must pass validation");
}

#[test]
fn test_ieee30_bus_count() {
    let net = ieee30().expect("ieee30 must build");
    assert_eq!(net.buses.len(), 30, "IEEE 30 must have 30 buses");
    assert!(
        net.branches.len() >= 36,
        "IEEE 30 must have at least 36 branches"
    );
}

#[test]
fn test_ieee30_valid() {
    let net = ieee30().expect("ieee30 must build");
    net.validate().expect("IEEE 30 must pass validation");
}

#[test]
fn test_ieee30_connected() {
    let net = ieee30().expect("ieee30 must build");
    assert!(net.is_connected());
}

#[test]
fn test_ieee57_bus_count() {
    let net = ieee57().expect("ieee57 must build");
    assert_eq!(net.buses.len(), 57, "IEEE 57 must have 57 buses");
    assert!(
        net.branches.len() >= 70,
        "IEEE 57 must have at least 70 branches, got {}",
        net.branches.len()
    );
}

#[test]
fn test_ieee57_valid() {
    let net = ieee57().expect("ieee57 must build");
    net.validate().expect("IEEE 57 must pass validation");
}

#[test]
fn test_ieee118_bus_count() {
    let net = ieee118().expect("ieee118 must build");
    assert_eq!(net.buses.len(), 118, "IEEE 118 must have 118 buses");
}

#[test]
fn test_ieee118_valid() {
    let net = ieee118().expect("ieee118 must build");
    net.validate().expect("IEEE 118 must pass validation");
}

#[test]
fn test_ieee300_bus_count() {
    let net = ieee300().expect("ieee300 must build");
    assert_eq!(net.buses.len(), 300, "IEEE 300 must have 300 buses");
}

#[test]
fn test_rts96_bus_count() {
    let net = rts96().expect("rts96 must build");
    assert_eq!(net.buses.len(), 73, "RTS-96 must have 73 buses");
}

#[test]
fn test_pegase89_bus_count() {
    let net = pegase89().expect("pegase89 must build");
    assert_eq!(net.buses.len(), 89, "PEGASE 89 must have 89 buses");
}

// ---------------------------------------------------------------------------
// IEEE 14-bus power flow convergence
// ---------------------------------------------------------------------------

#[test]
fn test_ieee14_power_flow_converges() {
    use oxigrid::powerflow::newton_raphson::NewtonRaphsonSolver;
    use oxigrid::powerflow::{PowerFlowConfig, PowerFlowMethod, PowerFlowSolver};

    let net = ieee14().expect("ieee14 must build");
    let cfg = PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: 50,
        tolerance: 1e-6,
        enforce_q_limits: false,
    };
    let solver = NewtonRaphsonSolver;
    let result = solver.solve(&net, &cfg).expect("power flow must not error");
    assert!(
        result.converged,
        "IEEE 14 power flow must converge, max_mismatch={:.2e}",
        result.max_mismatch
    );
    assert!(result.iterations <= 20, "Should converge in ≤20 iterations");

    // Voltage magnitudes in expected range
    for vm in &result.voltage_magnitude {
        assert!(
            *vm >= 0.9 && *vm <= 1.1,
            "IEEE 14 voltage out of normal range: {vm:.4} p.u."
        );
    }
}

// ---------------------------------------------------------------------------
// Synthetic network generation
// ---------------------------------------------------------------------------

#[test]
fn test_synthetic_ring_topology() {
    let config = SyntheticNetworkConfig {
        n_buses: 10,
        topology: NetworkTopology::Ring,
        ..Default::default()
    };
    let net = generate_synthetic_network(&config).expect("ring must generate");
    assert_eq!(net.buses.len(), 10, "Ring must have 10 buses");
    // Ring: exactly n branches (one per bus)
    assert_eq!(
        net.branches.len(),
        10,
        "Ring topology must have 10 branches, got {}",
        net.branches.len()
    );
    assert!(net.is_connected(), "Ring must be connected");
}

#[test]
fn test_synthetic_radial_topology() {
    let config = SyntheticNetworkConfig {
        n_buses: 15,
        n_generators: 1,
        topology: NetworkTopology::Radial,
        ..Default::default()
    };
    let net = generate_synthetic_network(&config).expect("radial must generate");
    assert_eq!(net.buses.len(), 15);
    // Spanning tree: exactly n-1 branches
    assert!(
        net.branches.len() >= 14,
        "Radial must have >= 14 branches, got {}",
        net.branches.len()
    );
    assert!(net.is_connected(), "Radial must be connected");
}

#[test]
fn test_synthetic_meshed_connected() {
    let config = SyntheticNetworkConfig {
        n_buses: 20,
        topology: NetworkTopology::Meshed,
        seed: 100,
        ..Default::default()
    };
    let net = generate_synthetic_network(&config).expect("meshed must generate");
    assert_eq!(net.buses.len(), 20);
    // Meshed: at least n-1 branches to be connected
    assert!(
        net.branches.len() >= net.buses.len() - 1,
        "Meshed must have >= n-1 branches"
    );
    assert!(net.is_connected(), "Meshed must be connected");
}

#[test]
fn test_synthetic_geographic_topology() {
    let config = SyntheticNetworkConfig {
        n_buses: 16,
        topology: NetworkTopology::Geographic,
        ..Default::default()
    };
    let net = generate_synthetic_network(&config).expect("geographic must generate");
    assert_eq!(net.buses.len(), 16);
    assert!(net.is_connected(), "Geographic must be connected");
}

#[test]
fn test_synthetic_small_world() {
    let config = SyntheticNetworkConfig {
        n_buses: 30,
        topology: NetworkTopology::SmallWorld,
        ..Default::default()
    };
    let net = generate_synthetic_network(&config).expect("small-world must generate");
    assert_eq!(net.buses.len(), 30, "Small-world must have 30 buses");
    assert!(net.is_connected(), "Small-world must be connected");
}

#[test]
fn test_scale_free_degree_distribution() {
    let config = SyntheticNetworkConfig {
        n_buses: 50,
        topology: NetworkTopology::ScaleFree,
        seed: 777,
        ..Default::default()
    };
    let net = generate_synthetic_network(&config).expect("scale-free must generate");
    assert_eq!(net.buses.len(), 50);

    // Scale-free: at least one high-degree bus (degree >= 4)
    let max_degree = (1..=50).map(|id| net.degree(id)).max().unwrap_or(0);
    assert!(
        max_degree >= 4,
        "Scale-free network should have at least one hub with degree ≥ 4, max was {max_degree}"
    );
}

#[test]
fn test_synthetic_validation_passes() {
    let config = SyntheticNetworkConfig {
        n_buses: 25,
        n_generators: 4,
        topology: NetworkTopology::Meshed,
        seed: 12345,
        ..Default::default()
    };
    let net = generate_synthetic_network(&config).expect("must generate");
    net.validate()
        .expect("generated network must pass validation");
}

#[test]
fn test_synthetic_error_on_zero_buses() {
    let config = SyntheticNetworkConfig {
        n_buses: 1,
        ..Default::default()
    };
    let result = generate_synthetic_network(&config);
    assert!(result.is_err(), "n_buses < 2 must return error");
}

#[test]
fn test_synthetic_reproducible_with_same_seed() {
    let config = SyntheticNetworkConfig {
        n_buses: 20,
        topology: NetworkTopology::Meshed,
        seed: 999,
        ..Default::default()
    };
    let net1 = generate_synthetic_network(&config).expect("first generation");
    let net2 = generate_synthetic_network(&config).expect("second generation");
    assert_eq!(
        net1.buses.len(),
        net2.buses.len(),
        "Same seed must produce same bus count"
    );
    assert_eq!(
        net1.branches.len(),
        net2.branches.len(),
        "Same seed must produce same branch count"
    );
}

// ---------------------------------------------------------------------------
// Distribution test cases
// ---------------------------------------------------------------------------

#[test]
fn test_ieee33_distribution() {
    let net = ieee33().expect("ieee33 must build");
    assert_eq!(net.buses.len(), 33, "IEEE 33 must have 33 buses");
    assert_eq!(net.branches.len(), 32, "IEEE 33 must have 32 main branches");
    // Total load check: ≈3715 kW = 3.715 MW
    let total_load: f64 = net.buses.iter().map(|b| b.pd.0).sum();
    assert!(
        (total_load - 3.715).abs() < 0.1,
        "IEEE 33 total load ≈ 3.715 MW, got {total_load:.3} MW"
    );
}

#[test]
fn test_ieee33_connected() {
    let net = ieee33().expect("ieee33 must build");
    assert!(net.is_connected(), "IEEE 33 must be connected");
}

#[test]
fn test_ieee33_valid() {
    let net = ieee33().expect("ieee33 must build");
    net.validate().expect("IEEE 33 must pass validation");
}

#[test]
fn test_ieee69_bus_count() {
    let net = ieee69().expect("ieee69 must build");
    assert_eq!(net.buses.len(), 69, "IEEE 69 must have 69 buses");
}

#[test]
fn test_ieee69_connected() {
    let net = ieee69().expect("ieee69 must build");
    assert!(net.is_connected(), "IEEE 69 must be connected");
}

#[test]
fn test_ieee69_valid() {
    let net = ieee69().expect("ieee69 must build");
    net.validate().expect("IEEE 69 must pass validation");
}

#[test]
fn test_lv_european_feeder() {
    let net = lv_european_residential(20).expect("LV feeder must build");
    // 1 substation + 20 customers = 21 buses
    assert!(
        net.buses.len() >= 20,
        "LV feeder must have >= 20 buses, got {}",
        net.buses.len()
    );
    // 0.4 kV network
    for bus in &net.buses {
        assert!(
            (bus.base_kv.0 - 0.4).abs() < 0.01,
            "LV feeder buses should be 0.4 kV"
        );
    }
    assert!(net.is_connected(), "LV feeder must be connected");
}

#[test]
fn test_mv_urban_feeder() {
    let net = mv_urban_feeder(10).expect("MV feeder must build");
    assert_eq!(
        net.buses.len(),
        10,
        "MV feeder must have requested bus count"
    );
    // 11 kV network
    for bus in &net.buses {
        assert!(
            (bus.base_kv.0 - 11.0).abs() < 0.1,
            "MV feeder buses should be 11 kV"
        );
    }
}

#[test]
fn test_mv_urban_feeder_normally_open_switch() {
    let net = mv_urban_feeder(8).expect("MV feeder must build");
    // Last branch is normally open (tie switch)
    let tie = net.branches.last().expect("must have branches");
    assert!(
        !tie.status,
        "Last branch (tie switch) must be normally open"
    );
}

// ---------------------------------------------------------------------------
// Benchmark suite
// ---------------------------------------------------------------------------

#[test]
fn test_benchmark_suite_runs() {
    let benchmarks = power_flow_benchmarks();
    assert!(!benchmarks.is_empty(), "Benchmark suite must not be empty");
    // Should have at least the 3 standard cases
    assert!(
        benchmarks.len() >= 3,
        "Expected >= 3 benchmarks, got {}",
        benchmarks.len()
    );
}

#[test]
fn test_benchmark_networks_valid() {
    let benchmarks = power_flow_benchmarks();
    for scenario in &benchmarks {
        scenario
            .network
            .validate()
            .unwrap_or_else(|e| panic!("Benchmark '{}' invalid: {e}", scenario.name));
    }
}

#[test]
fn test_benchmark_networks_connected() {
    let benchmarks = power_flow_benchmarks();
    for scenario in &benchmarks {
        assert!(
            scenario.network.is_connected(),
            "Benchmark '{}' must be connected",
            scenario.name
        );
    }
}

#[test]
fn test_validate_all_benchmarks() {
    use oxigrid::testcases::benchmark::validate_all_benchmarks;
    let reports = validate_all_benchmarks();
    assert!(!reports.is_empty(), "Must produce at least one report");

    // Log any failures for diagnostic purposes
    let failures: Vec<_> = reports.iter().filter(|r| !r.passed).collect();
    for f in &failures {
        eprintln!(
            "Benchmark '{}' note: converged={}, v_err={:.4}, l_err={:.2} MW, notes: {:?}",
            f.scenario_name, f.actual_converged, f.voltage_error_pu, f.losses_error_mw, f.notes
        );
    }

    // IEEE 14-bus and IEEE 30-bus must both converge
    let must_converge = ["IEEE 14-Bus", "IEEE 30-Bus"];
    for name in must_converge {
        if let Some(report) = reports.iter().find(|r| r.scenario_name == name) {
            assert!(
                report.actual_converged,
                "Benchmark '{}' must converge",
                report.scenario_name
            );
        }
    }
}
