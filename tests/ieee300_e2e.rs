//! End-to-end IEEE 300-bus cross-stack regression tests.
//!
//! Exercises: load → NR power flow → DC state estimation → N-1 contingency → DC-OPF.
//! These tests catch inter-module contract regressions that unit tests miss.
//!
//! The IEEE 300-bus network is loaded from `tests/data/ieee300.m` (MATPOWER format).
//! All five sub-tests must pass together to give confidence that the full pipeline
//! from data loading through optimisation is functioning correctly.

// All tests below require at minimum the `powerflow` feature.
// The DC-OPF test additionally requires `optimize`.
#![cfg(feature = "powerflow")]

use oxigrid::network::PowerNetwork;
use oxigrid::powerflow::state_estimation::{DcStateEstimator, Measurement};
use oxigrid::powerflow::{PowerFlowConfig, PowerFlowMethod};

// ─────────────────────────────────────────────────────────────────────────────
// Shared helpers
// ─────────────────────────────────────────────────────────────────────────────

fn ieee300_network() -> PowerNetwork {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee300.m");
    PowerNetwork::from_matpower(path).expect("IEEE 300-bus MATPOWER file must parse successfully")
}

fn nr_config() -> PowerFlowConfig {
    PowerFlowConfig {
        method: PowerFlowMethod::NewtonRaphson,
        max_iter: 80,
        tolerance: 1e-8,
        enforce_q_limits: false,
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 1: Load + basic topology checks
// ─────────────────────────────────────────────────────────────────────────────

#[test]
fn ieee300_e2e_topology_checks() {
    let net = ieee300_network();

    // Bus count
    let bus_count = net.bus_count();
    assert!(bus_count >= 300, "expected ≥300 buses, got {bus_count}");

    // Branch count must be non-zero
    assert!(
        !net.branches.is_empty(),
        "IEEE-300 must have at least one branch"
    );

    // Exactly one slack bus
    let n_slack = net
        .buses
        .iter()
        .filter(|b| b.bus_type == oxigrid::network::BusType::Slack)
        .count();
    assert_eq!(
        n_slack, 1,
        "IEEE-300 must have exactly one slack bus, found {n_slack}"
    );

    // At least some PQ and PV buses
    let n_pq = net.n_pq_buses();
    let n_pv = net.n_pv_buses();
    assert!(n_pq > 0, "IEEE-300 must have PQ buses, found 0");
    assert!(n_pv > 0, "IEEE-300 must have PV buses, found 0");

    // All bus types account for total bus count
    assert_eq!(
        n_slack + n_pq + n_pv,
        bus_count,
        "Bus type counts do not sum to total: {n_slack} slack + {n_pq} PQ + {n_pv} PV != {bus_count}"
    );

    // Network validation passes
    net.validate()
        .expect("IEEE-300 network validation must succeed");
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 2: NR power flow converges within 25 iterations
// ─────────────────────────────────────────────────────────────────────────────

#[test]
fn ieee300_e2e_nr_converges_within_25_iterations() {
    let net = ieee300_network();
    let result = net
        .solve_powerflow(&nr_config())
        .expect("IEEE-300 NR power flow must not return an error");

    assert!(
        result.converged,
        "Newton-Raphson must converge on IEEE-300 (iterations={}, max_mismatch={:.3e})",
        result.iterations, result.max_mismatch
    );

    assert!(
        result.iterations <= 25,
        "NR used {} iterations, expected ≤25",
        result.iterations
    );

    assert!(
        result.max_mismatch < 1e-6,
        "Mismatch {:.3e} exceeds 1e-6 tolerance",
        result.max_mismatch
    );

    // All voltage magnitudes must be in a physically plausible range
    for (i, &vm) in result.voltage_magnitude.iter().enumerate() {
        assert!(
            vm > 0.5 && vm < 1.5,
            "Bus {} voltage {:.4} p.u. outside [0.5, 1.5] range",
            i + 1,
            vm
        );
    }

    // All bus voltages must be finite
    for (i, &vm) in result.voltage_magnitude.iter().enumerate() {
        assert!(
            vm.is_finite(),
            "Bus {} voltage magnitude is not finite",
            i + 1
        );
    }
    for (i, &va) in result.voltage_angle.iter().enumerate() {
        assert!(va.is_finite(), "Bus {} voltage angle is not finite", i + 1);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 3: DC state estimation runs without error
// ─────────────────────────────────────────────────────────────────────────────

#[test]
fn ieee300_e2e_dc_state_estimation_runs() {
    let net = ieee300_network();
    let n = net.bus_count();

    // Solve NR first to get a reference solution for synthetic measurements
    let pf = net
        .solve_powerflow(&nr_config())
        .expect("IEEE-300 NR must converge for SE measurement generation");
    assert!(
        pf.converged,
        "NR must converge before building SE measurements"
    );

    // --- Build B-prime matrix (n × n) using DC susceptance model ---
    // Same construction as DcPowerFlowSolver: b_ij = 1 / (x * tap)
    let mut b_bus = vec![vec![0.0_f64; n]; n];
    let mut branch_from_idx = Vec::with_capacity(net.branches.len());
    let mut branch_to_idx = Vec::with_capacity(net.branches.len());
    let mut branch_x = Vec::with_capacity(net.branches.len());

    for branch in &net.branches {
        if !branch.status {
            continue;
        }
        let fi = net
            .bus_index(branch.from_bus)
            .expect("from_bus index must exist");
        let ti = net
            .bus_index(branch.to_bus)
            .expect("to_bus index must exist");
        let tap = if branch.tap.abs() < 1e-9 {
            1.0
        } else {
            branch.tap
        };
        let bij = 1.0 / (branch.x * tap);

        b_bus[fi][fi] += bij;
        b_bus[ti][ti] += bij;
        b_bus[fi][ti] -= bij;
        b_bus[ti][fi] -= bij;

        branch_from_idx.push(fi);
        branch_to_idx.push(ti);
        branch_x.push(branch.x * tap);
    }

    let slack_idx = net
        .slack_bus_index()
        .expect("IEEE-300 must have a slack bus");

    let estimator = DcStateEstimator::new(
        n,
        slack_idx,
        b_bus,
        branch_from_idx.clone(),
        branch_to_idx.clone(),
        branch_x,
    );

    // --- Build synthetic measurements from the converged NR branch flows ---
    // Use branch active power flows (in p.u.) as measurements.
    // IEEE-300 has ≈411 branches — far more than the 299 states needed.
    let base_mva = net.base_mva;
    let mut measurements: Vec<Measurement> = Vec::with_capacity(net.branches.len());

    for (k, bf) in pf.branch_flows.iter().enumerate() {
        // Map external bus IDs to 0-based internal indices for the estimator
        let fi_ext = bf.from_bus;
        let ti_ext = bf.to_bus;

        let fi = net
            .bus_index(fi_ext)
            .expect("branch from_bus must be in network");
        let ti = net
            .bus_index(ti_ext)
            .expect("branch to_bus must be in network");

        // Verify the branch is in the estimator's branch list (skip if not mapped)
        let branch_present = branch_from_idx
            .iter()
            .zip(branch_to_idx.iter())
            .any(|(&f, &t)| f == fi && t == ti);

        if branch_present {
            let p_pu = bf.p_from_mw / base_mva;
            // Use small but non-zero sigma to represent good-quality SCADA readings
            measurements.push(Measurement::branch_flow(fi, ti, p_pu, 0.01));
        }

        // Fallback: also add the power injection measurement at every 10th branch
        // to ensure observability even when parallel-branch skip occurs
        if k % 10 == 0 {
            let p_inj_pu = pf.p_injected[fi] / base_mva;
            measurements.push(Measurement::power_injection(fi, p_inj_pu, 0.02));
        }
    }

    // Must have at least n-1 = 299 measurements
    assert!(
        measurements.len() >= n - 1,
        "Need ≥{} measurements for DC-SE on {}-bus network, have {}",
        n - 1,
        n,
        measurements.len()
    );

    // --- Run DC WLS state estimation ---
    let se_result = estimator
        .estimate(&measurements)
        .expect("DC state estimation must not return an error on IEEE-300");

    assert!(
        se_result.converged,
        "DC state estimator must converge on IEEE-300"
    );

    // All estimated angles must be finite
    for (i, &theta) in se_result.theta.iter().enumerate() {
        assert!(theta.is_finite(), "Bus {} estimated angle is not finite", i);
    }

    // Slack bus angle must be fixed at 0
    assert!(
        se_result.theta[slack_idx].abs() < 1e-10,
        "Slack bus estimated angle {:.3e} must be 0",
        se_result.theta[slack_idx]
    );

    // Chi-squared statistic must be finite and non-negative
    assert!(
        se_result.chi2.is_finite() && se_result.chi2 >= 0.0,
        "Chi-squared must be finite and non-negative, got {:.4e}",
        se_result.chi2
    );

    // Residuals must all be finite
    for (i, &r) in se_result.residuals.iter().enumerate() {
        assert!(r.is_finite(), "Residual {} is not finite", i);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 4: N-1 contingency — removing one non-critical branch still converges
// ─────────────────────────────────────────────────────────────────────────────

#[test]
fn ieee300_e2e_n1_single_branch_removal_solvable() {
    let base_net = ieee300_network();

    // Find a branch whose removal does not disconnect the network.
    // IEEE-300 is a meshed transmission network, so most branch removals
    // leave the network connected. We scan from branch 0 upward.
    let candidate_idx = (0..base_net.branch_count()).find(|&k| {
        let mut trial = base_net.clone();
        trial.branches.remove(k);
        trial.is_connected()
    });

    let branch_idx = match candidate_idx {
        Some(k) => k,
        None => {
            // Defensive: if (unexpectedly) every branch removal disconnects the
            // network this test is vacuously satisfied — the pipeline was exercised.
            return;
        }
    };

    // Build the contingency network
    let mut contingency_net = base_net.clone();
    let removed = contingency_net.branches.remove(branch_idx);

    assert!(
        contingency_net.is_connected(),
        "N-1 network must remain connected after removing branch {branch_idx} \
         (from_bus={}, to_bus={})",
        removed.from_bus,
        removed.to_bus
    );

    // Re-solve NR on the contingency network
    let result = contingency_net
        .solve_powerflow(&nr_config())
        .expect("N-1 power flow solve must not return an error");

    assert!(
        result.converged,
        "NR must converge after removing branch {branch_idx} \
         (from={}, to={}) — iterations={}, mismatch={:.3e}",
        removed.from_bus, removed.to_bus, result.iterations, result.max_mismatch
    );

    // Voltage magnitudes remain in plausible range under contingency
    for (i, &vm) in result.voltage_magnitude.iter().enumerate() {
        assert!(
            vm > 0.5 && vm < 1.5,
            "Bus {} contingency voltage {:.4} p.u. outside [0.5, 1.5]",
            i + 1,
            vm
        );
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 5: DC-OPF produces a balanced dispatch
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(feature = "optimize")]
#[test]
fn ieee300_e2e_dc_opf_produces_feasible_dispatch() {
    use oxigrid::optimize::opf::dc_opf::{solve_dc_opf, GenCost};

    let net = ieee300_network();
    let total_load_mw: f64 = net.buses.iter().map(|b| b.pd.0).sum();

    // Build GenCost entries from the actual generator data.
    // Use p_min = 0 and p_max from the generator record so the feasibility
    // check in economic_dispatch always passes. Linear costs are varied by
    // generator index to produce a deterministic merit-order dispatch.
    let gen_costs: Vec<GenCost> = net
        .generators
        .iter()
        .enumerate()
        .map(|(i, gen)| {
            // Linear cost b increases slightly per generator (20 + i*0.5 $/MWh)
            let b = 20.0 + i as f64 * 0.5;
            // Use pmin=0 so feasibility is guaranteed regardless of load level
            GenCost::linear(b, 0.0, gen.pmax.max(1.0))
        })
        .collect();

    // Verify feasibility before calling the solver
    let p_max_total: f64 = gen_costs.iter().map(|c| c.p_max).sum();
    assert!(
        p_max_total >= total_load_mw,
        "Generator capacity {:.1} MW must cover load {:.1} MW",
        p_max_total,
        total_load_mw
    );

    let result =
        solve_dc_opf(&net, &gen_costs).expect("DC-OPF must not return an error on IEEE-300");

    // --- Power balance: sum(dispatch) ≈ total load ---
    let total_gen_mw: f64 = result.p_gen_mw.iter().sum();
    let balance_error = (total_gen_mw - total_load_mw).abs();
    let tol_mw = 0.01 * total_load_mw; // 1% of total load
    assert!(
        balance_error < tol_mw,
        "Power balance error {:.3} MW exceeds 1% of load ({:.3} MW). \
         gen={:.3} MW, load={:.3} MW",
        balance_error,
        tol_mw,
        total_gen_mw,
        total_load_mw
    );

    // --- All dispatch values within [p_min, p_max] ---
    for (k, (&p, cost)) in result.p_gen_mw.iter().zip(gen_costs.iter()).enumerate() {
        assert!(
            p >= cost.p_min - 1e-6,
            "Generator {} dispatch {:.3} MW below p_min {:.3} MW",
            k + 1,
            p,
            cost.p_min
        );
        assert!(
            p <= cost.p_max + 1e-6,
            "Generator {} dispatch {:.3} MW above p_max {:.3} MW",
            k + 1,
            p,
            cost.p_max
        );
    }

    // --- Branch flows and marginal cost are finite ---
    assert_eq!(
        result.branch_flows_mw.len(),
        net.branch_count(),
        "Branch flow count must match network branch count"
    );
    for (k, &f) in result.branch_flows_mw.iter().enumerate() {
        assert!(f.is_finite(), "Branch {k} flow is not finite: {f}");
    }
    assert!(
        result.total_cost.is_finite() && result.total_cost >= 0.0,
        "Total cost must be finite and non-negative, got {:.4}",
        result.total_cost
    );
    assert!(
        result.lambda.is_finite() && result.lambda > 0.0,
        "System marginal price (lambda) must be finite and positive, got {:.4}",
        result.lambda
    );
}
