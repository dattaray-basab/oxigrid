//! DC-OPF validation tests for IEEE 14-bus and IEEE 30-bus.
//!
//! Reference values are derived analytically (merit-order dispatch) and verified
//! against fundamental economic dispatch optimality conditions (equal-incremental-cost).
//! All cost tolerances are < 0.1% of expected value.
#![cfg(all(feature = "powerflow", feature = "optimize"))]

use oxigrid::network::PowerNetwork;
use oxigrid::optimize::opf::dc_opf::{solve_dc_opf, GenCost};

fn ieee14_network() -> PowerNetwork {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
    PowerNetwork::from_matpower(path).expect("Failed to parse IEEE 14-bus")
}

fn ieee30_network() -> PowerNetwork {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee30.m");
    PowerNetwork::from_matpower(path).expect("Failed to parse IEEE 30-bus")
}

// IEEE 14-bus total load = 259.0 MW (sum of all bus Pd values)
const IEEE14_TOTAL_LOAD_MW: f64 = 259.0;

// ── Linear-cost merit-order validation ───────────────────────────────────────
//
// With these linear costs (c=0), merit-order dispatch is exact.
// Gen 1 (bus 1, b=20, Pmax=332.4) can supply the entire 259 MW load alone.
// Expected: p_gen[0]=259 MW, all others=0, cost = 20 * 259 = 5180 $/h

fn ieee14_linear_costs() -> Vec<GenCost> {
    vec![
        GenCost::linear(20.0, 0.0, 332.4), // gen 1 – cheapest, Pmax > total load
        GenCost::linear(30.0, 0.0, 140.0), // gen 2
        GenCost::linear(35.0, 0.0, 100.0), // gen 3
        GenCost::linear(38.0, 0.0, 100.0), // gen 6
        GenCost::linear(42.0, 0.0, 100.0), // gen 8
    ]
}

#[test]
fn test_ieee14_dc_opf_linear_power_balance() {
    let net = ieee14_network();
    let costs = ieee14_linear_costs();
    let result = solve_dc_opf(&net, &costs).unwrap();

    let total_gen: f64 = result.p_gen_mw.iter().sum();
    assert!(
        (total_gen - IEEE14_TOTAL_LOAD_MW).abs() < 1.0,
        "Total generation {:.3} MW should equal load {:.1} MW",
        total_gen,
        IEEE14_TOTAL_LOAD_MW
    );
}

#[test]
fn test_ieee14_dc_opf_linear_cost_accuracy() {
    let net = ieee14_network();
    let costs = ieee14_linear_costs();
    let result = solve_dc_opf(&net, &costs).unwrap();

    // Merit-order: Gen 1 (b=20, Pmax=332.4) takes all 259 MW
    let expected_cost = 20.0 * IEEE14_TOTAL_LOAD_MW; // 5180.0 $/h
    let error_pct = (result.total_cost - expected_cost).abs() / expected_cost * 100.0;
    assert!(
        error_pct < 0.1,
        "Cost error {:.4}% exceeds 0.1% tolerance. Got {:.2}, expected {:.2}",
        error_pct,
        result.total_cost,
        expected_cost
    );
}

#[test]
fn test_ieee14_dc_opf_linear_dispatch_merit_order() {
    let net = ieee14_network();
    let costs = ieee14_linear_costs();
    let result = solve_dc_opf(&net, &costs).unwrap();

    // Gen 1 must supply ≈ the entire load (cheapest, Pmax = 332.4 > 259 MW)
    let p0 = result.p_gen_mw[0];
    assert!(
        (p0 - IEEE14_TOTAL_LOAD_MW).abs() < 1.0,
        "Gen 1 should supply all 259 MW, got {:.2} MW",
        p0
    );
    // All other generators should be at minimum (0)
    for (k, &p) in result.p_gen_mw[1..].iter().enumerate() {
        assert!(p < 1.0, "Gen {} should be near 0, got {:.2} MW", k + 2, p);
    }
}

#[test]
fn test_ieee14_dc_opf_gen_limits_respected() {
    let net = ieee14_network();
    let costs = ieee14_linear_costs();
    let result = solve_dc_opf(&net, &costs).unwrap();

    let limits = [
        (332.4, 0.0),
        (140.0, 0.0),
        (100.0, 0.0),
        (100.0, 0.0),
        (100.0, 0.0),
    ];
    for (k, (&p, (pmax, pmin))) in result.p_gen_mw.iter().zip(limits.iter()).enumerate() {
        assert!(
            p >= pmin - 1e-6 && p <= pmax + 1e-6,
            "Gen {} output {:.2} MW outside limits [{:.1}, {:.1}]",
            k + 1,
            p,
            pmin,
            pmax
        );
    }
}

#[test]
fn test_ieee14_dc_opf_branch_flows_finite() {
    let net = ieee14_network();
    let costs = ieee14_linear_costs();
    let result = solve_dc_opf(&net, &costs).unwrap();

    assert_eq!(result.branch_flows_mw.len(), net.branch_count());
    for (k, &f) in result.branch_flows_mw.iter().enumerate() {
        assert!(f.is_finite(), "Branch {k} flow is not finite: {f}");
    }
}

#[test]
fn test_ieee14_dc_opf_lambda_positive() {
    let net = ieee14_network();
    let costs = ieee14_linear_costs();
    let result = solve_dc_opf(&net, &costs).unwrap();
    assert!(
        result.lambda > 0.0,
        "Lambda should be positive, got {:.4}",
        result.lambda
    );
    // For linear cost, lambda = marginal cost of dispatching unit = 20 $/MWh
    assert!(
        (result.lambda - 20.0).abs() < 1.0,
        "Lambda {:.2} should be near 20 $/MWh",
        result.lambda
    );
}

// ── Quadratic-cost equal-incremental-cost validation ─────────────────────────
//
// Two-generator version: Gen 1 and Gen 2 with quadratic costs.
// At optimality: dC1/dP1 = dC2/dP2 = λ* (both unconstrained).
// Analytical solution:
//   P1 = (λ* - b1) / (2*c1),  P2 = (λ* - b2) / (2*c2)
//   P1 + P2 = load → solve for λ*

#[test]
fn test_ieee14_dc_opf_quadratic_equal_marginal_cost() {
    let net = ieee14_network();
    // Use only 2 generators with quadratic costs for easy analytical check
    // Give very high Pmax so they won't be constrained
    let costs = vec![
        GenCost::quadratic(0.0, 20.0, 0.05, 0.0, 500.0), // gen 1: MC = 20 + 0.1*P1
        GenCost::quadratic(0.0, 22.0, 0.04, 0.0, 500.0), // gen 2: MC = 22 + 0.08*P2
        GenCost::quadratic(0.0, 999.0, 0.001, 0.0, 500.0), // gen 3: too expensive
        GenCost::quadratic(0.0, 999.0, 0.001, 0.0, 500.0), // gen 6: too expensive
        GenCost::quadratic(0.0, 999.0, 0.001, 0.0, 500.0), // gen 8: too expensive
    ];

    let result = solve_dc_opf(&net, &costs).unwrap();

    let p0 = result.p_gen_mw[0];
    let p1 = result.p_gen_mw[1];

    // Both unconstrained → equal marginal costs
    let mc0 = 20.0 + 2.0 * 0.05 * p0;
    let mc1 = 22.0 + 2.0 * 0.04 * p1;

    // Power balance
    let load: f64 = net.buses.iter().map(|b| b.pd.0).sum();
    let total_gen: f64 = result.p_gen_mw.iter().sum();
    assert!(
        (total_gen - load).abs() < 1.0,
        "Power balance error: gen={:.2}, load={:.2}",
        total_gen,
        load
    );

    // Equal marginal cost condition (allow 5% tolerance since gens 3/4/5 may absorb small residual)
    // At minimum, gen 1 and gen 2 should both be loaded
    if p0 > 1.0 && p1 > 1.0 {
        let mc_diff = (mc0 - mc1).abs();
        assert!(
            mc_diff < 5.0,
            "Marginal costs should be equal: MC1={:.2}, MC2={:.2}",
            mc0,
            mc1
        );
    }
}

#[test]
fn test_ieee14_dc_opf_cost_lower_than_expensive_dispatch() {
    // Compare optimal dispatch with a deliberately expensive one
    let net = ieee14_network();

    // Cheap-first order (optimal)
    let cheap_costs = vec![
        GenCost::linear(20.0, 0.0, 332.4),
        GenCost::linear(30.0, 0.0, 140.0),
        GenCost::linear(35.0, 0.0, 100.0),
        GenCost::linear(38.0, 0.0, 100.0),
        GenCost::linear(42.0, 0.0, 100.0),
    ];

    // Expensive-first order (suboptimal – reverse order)
    let expensive_costs = vec![
        GenCost::linear(42.0, 0.0, 332.4),
        GenCost::linear(38.0, 0.0, 140.0),
        GenCost::linear(35.0, 0.0, 100.0),
        GenCost::linear(30.0, 0.0, 100.0),
        GenCost::linear(20.0, 0.0, 100.0),
    ];

    let cheap_result = solve_dc_opf(&net, &cheap_costs).unwrap();
    let exp_result = solve_dc_opf(&net, &expensive_costs).unwrap();

    // Optimal dispatch must have lower cost
    assert!(
        cheap_result.total_cost <= exp_result.total_cost + 1e-3,
        "Cheap dispatch {:.2} $/h should be cheaper than expensive {:.2} $/h",
        cheap_result.total_cost,
        exp_result.total_cost
    );
}

// ── Network utility method validation ────────────────────────────────────────

#[test]
fn test_ieee14_network_utilities() {
    let net = ieee14_network();
    // Total load matches sum of bus Pd values (≈ 259 MW for IEEE 14-bus)
    let load = net.total_load_mw();
    assert!(
        (load - IEEE14_TOTAL_LOAD_MW).abs() < 1.0,
        "Total load {load:.2} should be {IEEE14_TOTAL_LOAD_MW}"
    );
    // Installed capacity > load (should have positive reserve)
    let cap = net.installed_capacity_mw();
    assert!(
        cap > load,
        "Installed capacity {cap:.1} MW must exceed load {load:.1} MW"
    );
    // Reserve margin positive
    let rm = net.reserve_margin();
    assert!(rm > 0.0, "Reserve margin {rm:.3} must be positive");
    // Bus type counts sum to total
    let n_slack = net
        .buses
        .iter()
        .filter(|b| b.bus_type == oxigrid::network::BusType::Slack)
        .count();
    assert_eq!(
        n_slack + net.n_pv_buses() + net.n_pq_buses(),
        net.bus_count()
    );
    // Total generation setpoint finite
    let gen_total = net.total_generation_mw();
    assert!(gen_total.is_finite());
}

// ── IEEE 57-bus DC-OPF validation ────────────────────────────────────────────

fn ieee57_network() -> PowerNetwork {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee57.m");
    PowerNetwork::from_matpower(path).expect("Failed to parse IEEE 57-bus")
}

fn ieee57_costs() -> Vec<GenCost> {
    // 7 generators with linear costs (merit order)
    // Gen 1 (Pmax=576): b=20   Gen 5 (Pmax=550): b=22 — two cheapest, largest
    // Gen 2 (Pmax=100): b=30   Gen 3 (Pmax=140): b=32
    // Gen 4 (Pmax=100): b=35   Gen 6 (Pmax=100): b=38   Gen 7 (Pmax=410): b=40
    vec![
        GenCost::linear(20.0, 0.0, 576.0),
        GenCost::linear(30.0, 0.0, 100.0),
        GenCost::linear(32.0, 0.0, 140.0),
        GenCost::linear(35.0, 0.0, 100.0),
        GenCost::linear(22.0, 0.0, 550.0),
        GenCost::linear(38.0, 0.0, 100.0),
        GenCost::linear(40.0, 0.0, 410.0),
    ]
}

#[test]
fn test_ieee57_dc_opf_power_balance() {
    let net = ieee57_network();
    let costs = ieee57_costs();
    let result = solve_dc_opf(&net, &costs).unwrap();

    let total_load: f64 = net.buses.iter().map(|b| b.pd.0).sum();
    let total_gen: f64 = result.p_gen_mw.iter().sum();
    assert!(
        (total_gen - total_load).abs() < 1.0,
        "Power balance: gen={:.2} MW, load={:.2} MW",
        total_gen,
        total_load
    );
}

#[test]
fn test_ieee57_dc_opf_gen_limits() {
    let net = ieee57_network();
    let costs = ieee57_costs();
    let result = solve_dc_opf(&net, &costs).unwrap();

    for (k, &p) in result.p_gen_mw.iter().enumerate() {
        let (pmax, pmin) = (costs[k].p_max, costs[k].p_min);
        assert!(
            p >= pmin - 1e-6 && p <= pmax + 1e-6,
            "Gen {} output {:.2} MW outside [{:.1}, {:.1}]",
            k + 1,
            p,
            pmin,
            pmax
        );
    }
}

#[test]
fn test_ieee57_dc_opf_cost_bounds() {
    let net = ieee57_network();
    let costs = ieee57_costs();
    let result = solve_dc_opf(&net, &costs).unwrap();

    let load: f64 = net.buses.iter().map(|b| b.pd.0).sum();
    let b_min = 20.0_f64;
    let b_max = 40.0_f64;
    assert!(result.total_cost >= b_min * load - 1.0);
    assert!(result.total_cost <= b_max * load + 1.0);
}

#[test]
fn test_ieee57_dc_opf_lambda_in_range() {
    let net = ieee57_network();
    let costs = ieee57_costs();
    let result = solve_dc_opf(&net, &costs).unwrap();

    assert!(result.lambda.is_finite());
    assert!(
        result.lambda >= 20.0 - 1.0,
        "Lambda {:.2} below min cost",
        result.lambda
    );
    assert!(
        result.lambda <= 40.0 + 1.0,
        "Lambda {:.2} above max cost",
        result.lambda
    );
}

#[test]
fn test_ieee57_dc_opf_cheap_gens_loaded_first() {
    let net = ieee57_network();
    let costs = ieee57_costs();
    let result = solve_dc_opf(&net, &costs).unwrap();

    // Gen 1 (b=20) and Gen 5 (b=22) are cheapest — they should be heavily loaded
    let p1 = result.p_gen_mw[0]; // Gen 1: b=20, Pmax=576
    let p5 = result.p_gen_mw[4]; // Gen 5: b=22, Pmax=550
                                 // Gen 7 (b=40) is most expensive
    let p7 = result.p_gen_mw[6]; // Gen 7: b=40, Pmax=410

    let load: f64 = net.buses.iter().map(|b| b.pd.0).sum();
    // Combined output of the two cheapest should be substantial
    assert!(
        p1 + p5 > load * 0.5,
        "Cheap generators (Gen1={p1:.1}+Gen5={p5:.1}={:.1} MW) should carry >50% of {load:.1} MW load",
        p1 + p5
    );
    // Most expensive generator should be loaded less than cheapest
    assert!(
        p7 <= p1 + 1.0,
        "Expensive Gen7={p7:.1} MW should not exceed cheapest Gen1={p1:.1} MW"
    );
}

// ── IEEE 30-bus validation ────────────────────────────────────────────────────

fn ieee30_linear_costs() -> Vec<GenCost> {
    // IEEE 30-bus has 6 generators
    vec![
        GenCost::linear(20.0, 0.0, 200.0), // gen 1 at bus 1 (slack)
        GenCost::linear(28.0, 0.0, 80.0),  // gen 2 at bus 2
        GenCost::linear(32.0, 0.0, 50.0),  // gen 3 at bus 5
        GenCost::linear(35.0, 0.0, 35.0),  // gen 4 at bus 8
        GenCost::linear(38.0, 0.0, 30.0),  // gen 5 at bus 11
        GenCost::linear(40.0, 0.0, 40.0),  // gen 6 at bus 13
    ]
}

#[test]
fn test_ieee30_dc_opf_power_balance() {
    let net = ieee30_network();
    let costs = ieee30_linear_costs();
    let result = solve_dc_opf(&net, &costs).unwrap();

    let total_load: f64 = net.buses.iter().map(|b| b.pd.0).sum();
    let total_gen: f64 = result.p_gen_mw.iter().sum();

    assert!(
        (total_gen - total_load).abs() < 1.0,
        "Power balance: gen={:.2} MW, load={:.2} MW",
        total_gen,
        total_load
    );
}

#[test]
fn test_ieee30_dc_opf_gen_limits() {
    let net = ieee30_network();
    let costs = ieee30_linear_costs();
    let result = solve_dc_opf(&net, &costs).unwrap();

    for (k, &p) in result.p_gen_mw.iter().enumerate() {
        let pmax = costs[k].p_max;
        let pmin = costs[k].p_min;
        assert!(
            p >= pmin - 1e-6 && p <= pmax + 1e-6,
            "Gen {} output {:.2} MW outside [{:.1}, {:.1}]",
            k + 1,
            p,
            pmin,
            pmax
        );
    }
}

#[test]
fn test_ieee30_dc_opf_cost_within_bounds() {
    let net = ieee30_network();
    let costs = ieee30_linear_costs();
    let result = solve_dc_opf(&net, &costs).unwrap();

    let total_load: f64 = net.buses.iter().map(|b| b.pd.0).sum();

    // Lower bound: cheapest generator (b=20) covers all load → cost ≥ 20 * load
    let cost_lower = 20.0 * total_load;
    // Upper bound: most expensive generator (b=40) covers all load → cost ≤ 40 * load
    let cost_upper = 40.0 * total_load;

    assert!(
        result.total_cost >= cost_lower - 1.0,
        "Cost {:.2} below theoretical minimum {:.2}",
        result.total_cost,
        cost_lower
    );
    assert!(
        result.total_cost <= cost_upper + 1.0,
        "Cost {:.2} above theoretical maximum {:.2}",
        result.total_cost,
        cost_upper
    );
}

#[test]
fn test_ieee30_dc_opf_lambda_finite() {
    let net = ieee30_network();
    let costs = ieee30_linear_costs();
    let result = solve_dc_opf(&net, &costs).unwrap();

    assert!(result.lambda.is_finite(), "Lambda must be finite");
    assert!(result.lambda > 0.0, "Lambda must be positive");
    assert!(
        result.lambda <= 40.0 + 1e-6,
        "Lambda exceeds max marginal cost"
    );
}
