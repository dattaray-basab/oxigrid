#![cfg(feature = "powerflow")]
use oxigrid::network::PowerNetwork;
use oxigrid::powerflow::{PowerFlowConfig, PowerFlowMethod};

fn ieee118_network() -> PowerNetwork {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee118.m");
    PowerNetwork::from_matpower(path).expect("Failed to parse IEEE 118-bus data")
}

#[test]
fn test_ieee118_parse() {
    let network = ieee118_network();
    assert_eq!(network.bus_count(), 118, "bus count");
    assert_eq!(network.branch_count(), 185, "branch count");
    assert_eq!(network.generators.len(), 54, "generator count");
}

#[test]
fn test_ieee118_nr_convergence() {
    let network = ieee118_network();
    let config = PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: 50,
        tolerance: 1e-8,
        enforce_q_limits: false,
    };
    let result = network.solve_powerflow(&config).unwrap();
    assert!(
        result.converged,
        "IEEE 118-bus NR did not converge after {} iterations, max_mismatch={:.2e}",
        result.iterations, result.max_mismatch
    );
    assert!(
        result.iterations <= 20,
        "Too many iterations: {}",
        result.iterations
    );
}

#[test]
fn test_ieee118_slack_voltage() {
    let network = ieee118_network();
    let config = PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: 50,
        tolerance: 1e-8,
        enforce_q_limits: false,
    };
    let result = network.solve_powerflow(&config).unwrap();
    assert!(result.converged);

    // Slack bus is bus 69 (index 68): Vg = 1.035 p.u.
    let slack_idx = network.slack_bus_index().unwrap();
    assert!(
        (result.voltage_magnitude[slack_idx] - 1.035).abs() < 1e-6,
        "Slack bus voltage = {:.6}, expected 1.035",
        result.voltage_magnitude[slack_idx]
    );
    assert!(
        result.voltage_angle[slack_idx].abs() < 1e-10,
        "Slack bus angle = {:.2e}, expected 0",
        result.voltage_angle[slack_idx]
    );
}

#[test]
fn test_ieee118_dc_powerflow() {
    let network = ieee118_network();
    let config = PowerFlowConfig {
        method: PowerFlowMethod::DcApproximation,
        max_iter: 1,
        tolerance: 1e-8,
        enforce_q_limits: false,
    };
    let result = network.solve_powerflow(&config).unwrap();
    assert!(result.converged, "IEEE 118-bus DC power flow failed");
    // DC: all voltage magnitudes = 1.0
    for (i, vm) in result.voltage_magnitude.iter().enumerate() {
        assert!(
            (vm - 1.0).abs() < 1e-10,
            "Bus {} DC voltage magnitude = {:.6}, expected 1.0",
            i + 1,
            vm
        );
    }
    // Slack bus angle = 0
    let slack_idx = network.slack_bus_index().unwrap();
    assert!(result.voltage_angle[slack_idx].abs() < 1e-10);
}

#[test]
fn test_ieee118_incidence_matrix() {
    let network = ieee118_network();
    let a = network.incidence_matrix();
    assert_eq!(a.len(), 118, "incidence matrix rows");
    assert_eq!(a[0].len(), 185, "incidence matrix columns");

    // Each column must have exactly one +1 and one -1, rest zeros
    for k in 0..185 {
        let pos: usize = a.iter().filter(|row| row[k] > 0.5).count();
        let neg: usize = a.iter().filter(|row| row[k] < -0.5).count();
        assert_eq!(pos, 1, "branch {} should have exactly 1 from-bus", k);
        assert_eq!(neg, 1, "branch {} should have exactly 1 to-bus", k);
    }
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
fn test_ieee118_branch_flows_count() {
    let network = ieee118_network();
    let result = network.solve_powerflow(&nr_config()).unwrap();
    assert!(result.converged);
    assert_eq!(
        result.branch_flows.len(),
        185,
        "Should have 185 branch flow records"
    );
}

#[test]
fn test_ieee118_branch_flows_finite() {
    let network = ieee118_network();
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
fn test_ieee118_total_losses_plausible() {
    let network = ieee118_network();
    let result = network.solve_powerflow(&nr_config()).unwrap();
    assert!(result.converged);
    let total_loss: f64 = result.branch_flows.iter().map(|bf| bf.p_loss_mw).sum();
    // IEEE 118-bus: typical losses ~130 MW
    assert!(total_loss > 0.0, "Losses should be positive");
    assert!(
        total_loss < 300.0,
        "Losses {total_loss:.2} MW seem too high"
    );
}

#[test]
fn test_ieee118_fdlf_convergence() {
    let network = ieee118_network();
    let config = PowerFlowConfig {
        method: PowerFlowMethod::FastDecoupled,
        max_iter: 50,
        tolerance: 1e-6,
        enforce_q_limits: false,
    };
    let result = network.solve_powerflow(&config).unwrap();
    assert!(result.converged, "IEEE 118-bus FDLF did not converge");
    assert!(result.max_mismatch < 1e-5);
}

#[test]
fn test_ieee118_branch_flows_conservation() {
    let network = ieee118_network();
    let result = network.solve_powerflow(&nr_config()).unwrap();
    assert!(result.converged);

    let inj_sum: f64 = result.p_injected.iter().sum();
    let loss_sum: f64 = result.branch_flows.iter().map(|bf| bf.p_loss_mw).sum();
    assert!(
        (inj_sum - loss_sum).abs() < 2.0,
        "Injection sum {inj_sum:.3} ≠ branch loss sum {loss_sum:.3}"
    );
}

// ── Network utility methods ───────────────────────────────────────────────────

#[test]
fn test_ieee118_total_load() {
    let network = ieee118_network();
    let load = network.total_load_mw();
    // IEEE 118-bus total load ≈ 4242 MW
    assert!(
        load > 3000.0 && load < 6000.0,
        "Total load {load:.1} MW out of expected range"
    );
}

#[test]
fn test_ieee118_installed_capacity() {
    let network = ieee118_network();
    let cap = network.installed_capacity_mw();
    assert!(
        cap > network.total_load_mw(),
        "Installed capacity should exceed load"
    );
}

#[test]
fn test_ieee118_reserve_margin_positive() {
    let network = ieee118_network();
    let margin = network.reserve_margin();
    assert!(
        margin > 0.0,
        "Reserve margin {margin:.3} should be positive"
    );
}

#[test]
fn test_ieee118_bus_type_counts() {
    let network = ieee118_network();
    let n_slack = network
        .buses
        .iter()
        .filter(|b| b.bus_type == oxigrid::network::BusType::Slack)
        .count();
    let n_pv = network.n_pv_buses();
    let n_pq = network.n_pq_buses();
    assert_eq!(n_slack, 1, "Exactly 1 slack bus");
    assert_eq!(n_slack + n_pv + n_pq, 118, "All buses accounted for");
    assert!(n_pv > 0, "Should have PV buses");
    assert!(n_pq > 0, "Should have PQ buses");
}
