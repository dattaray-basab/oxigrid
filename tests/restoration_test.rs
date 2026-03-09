//! Integration tests for the power system restoration module.
//!
//! Tests cover: feasibility, cold-load pickup decay, reserve margin,
//! priority ordering, frequency nadir, SAIDI comparison, ENS positivity,
//! and load-block priority ordering via the sequencer.

#![cfg(feature = "optimize")]

use oxigrid::network::branch::Branch;
use oxigrid::network::bus::{Bus, BusType};
use oxigrid::network::topology::{Generator, PowerNetwork};
use oxigrid::optimize::restoration::black_start::{
    BlackStartConfig, BlackStartPlanner, BlackStartUnit, LoadBlock,
};
use oxigrid::optimize::restoration::sequence::{RestorationMetrics, RestorationSequencer};
use oxigrid::units::electrical::{Power, ReactivePower, Voltage};

// ─────────────────────────────────────────────────────────────────────────────
// Shared test network builder
// ─────────────────────────────────────────────────────────────────────────────

/// Build a simple 3-bus test network:
///
/// ```text
///  Bus 1 (Slack, BS gen)  ──branch0──  Bus 2 (PV, gen)  ──branch1──  Bus 3 (PQ, load)
/// ```
fn make_three_bus_network() -> PowerNetwork {
    let mut net = PowerNetwork::new(100.0);

    // Bus 1 — Slack (black-start generator)
    let mut b1 = Bus::new(1, BusType::Slack);
    b1.base_kv = Voltage(110_000.0); // 110 kV in Volts
    b1.pd = Power(0.0);
    b1.qd = ReactivePower(0.0);
    net.buses.push(b1);

    // Bus 2 — PV (non-black-start generator)
    let mut b2 = Bus::new(2, BusType::PV);
    b2.base_kv = Voltage(110_000.0);
    b2.pd = Power(0.0);
    b2.qd = ReactivePower(0.0);
    net.buses.push(b2);

    // Bus 3 — PQ (load bus)
    let mut b3 = Bus::new(3, BusType::PQ);
    b3.base_kv = Voltage(110_000.0);
    b3.pd = Power(30.0e6); // 30 MW in Watts
    b3.qd = ReactivePower(10.0e6); // 10 MVAr in VAr
    net.buses.push(b3);

    net.branches.push(Branch {
        from_bus: 1,
        to_bus: 2,
        r: 0.01,
        x: 0.05,
        b: 0.02,
        rate_a: 200.0,
        rate_b: 250.0,
        rate_c: 300.0,
        tap: 0.0,
        shift: 0.0,
        status: true,
    });
    net.branches.push(Branch {
        from_bus: 2,
        to_bus: 3,
        r: 0.01,
        x: 0.04,
        b: 0.015,
        rate_a: 150.0,
        rate_b: 200.0,
        rate_c: 250.0,
        tap: 0.0,
        shift: 0.0,
        status: true,
    });

    // Generator at bus 1 (black-start unit)
    net.generators.push(Generator {
        bus_id: 1,
        pg: 80.0,
        qg: 20.0,
        qmax: 50.0,
        qmin: -20.0,
        vg: 1.0,
        mbase: 100.0,
        status: true,
        pmax: 100.0,
        pmin: 10.0,
    });

    // Generator at bus 2 (non-black-start)
    net.generators.push(Generator {
        bus_id: 2,
        pg: 40.0,
        qg: 10.0,
        qmax: 30.0,
        qmin: -15.0,
        vg: 1.0,
        mbase: 100.0,
        status: true,
        pmax: 60.0,
        pmin: 5.0,
    });

    net
}

/// Build a BlackStartConfig for the 3-bus network.
fn make_simple_config() -> BlackStartConfig {
    BlackStartConfig {
        black_start_units: vec![BlackStartUnit {
            gen_id: 0,
            bus: 1,
            p_rated_mw: 100.0,
            p_min_mw: 10.0,
            ramp_rate_mw_per_min: 5.0,
            crank_time_min: 15.0,
            max_crank_distance_km: 200.0,
            auxiliary_load_mw: 3.0,
            priority: 1,
        }],
        load_blocks: vec![
            LoadBlock {
                block_id: 0,
                buses: vec![3],
                base_demand_mw: 20.0,
                cold_load_pickup_factor: 2.5,
                cold_load_decay_min: 30.0,
                priority: 1,
                can_defer: false,
            },
            LoadBlock {
                block_id: 1,
                buses: vec![2],
                base_demand_mw: 10.0,
                cold_load_pickup_factor: 2.0,
                cold_load_decay_min: 25.0,
                priority: 2,
                can_defer: true,
            },
        ],
        max_restoration_time_min: 240.0,
        frequency_tolerance_hz: 0.5,
        voltage_tolerance_pu: 0.05,
        max_generator_loading_pct: 80.0,
        reserve_margin_pct: 20.0,
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[test]
fn test_black_start_plan_feasible() {
    let network = make_three_bus_network();
    let config = make_simple_config();
    let planner = BlackStartPlanner::new(config);

    let plan = planner.plan(&network).expect("plan should succeed");

    assert!(
        plan.feasible,
        "Plan should be feasible; bottlenecks: {:?}",
        plan.bottlenecks
    );
    assert!(
        plan.total_time_min <= 240.0,
        "Should complete within 4 hours, got {:.1} min",
        plan.total_time_min
    );
    assert!(
        plan.restored_load_pct > 0.99,
        "All load should be restored, got {:.2}",
        plan.restored_load_pct
    );
}

#[test]
fn test_cold_load_pickup_decays() {
    let block = LoadBlock {
        block_id: 0,
        buses: vec![1],
        base_demand_mw: 100.0,
        cold_load_pickup_factor: 2.5,
        cold_load_decay_min: 30.0,
        priority: 1,
        can_defer: false,
    };
    let config = BlackStartConfig::default();
    let planner = BlackStartPlanner::new(config);

    // At t=0 after restore: demand ≈ factor × base
    let at_zero = planner.cold_load_pickup(&block, 0.0, 0.0);
    assert!(
        (at_zero - 250.0).abs() < 1e-6,
        "Expected 250 MW at t=0, got {:.4}",
        at_zero
    );

    // After 100 min the demand should be close to base
    let at_100 = planner.cold_load_pickup(&block, 0.0, 100.0);
    assert!(
        at_100 < 110.0,
        "Expected near-base demand at t=100 min, got {:.2} MW",
        at_100
    );
    assert!(
        at_100 > 100.0,
        "Demand should still be slightly above base at t=100 min, got {:.2}",
        at_100
    );
}

#[test]
fn test_restoration_respects_reserve() {
    let network = make_three_bus_network();
    let config = make_simple_config();
    let reserve_pct = config.reserve_margin_pct;
    let planner = BlackStartPlanner::new(config);

    let plan = planner.plan(&network).expect("plan should succeed");

    for step in &plan.steps {
        let required_reserve = step.available_generation_mw * reserve_pct / 100.0;
        let actual_reserve = step.available_generation_mw - step.connected_load_mw;
        assert!(
            actual_reserve >= required_reserve - 1e-6,
            "Step {} at {:.1} min violates reserve: available={:.1}, load={:.1}, reserve={:.1}",
            step.step_id,
            step.time_min,
            step.available_generation_mw,
            step.connected_load_mw,
            required_reserve
        );
    }
}

#[test]
fn test_critical_loads_restored_first() {
    let network = make_three_bus_network();
    let config = BlackStartConfig {
        black_start_units: vec![BlackStartUnit {
            gen_id: 0,
            bus: 1,
            p_rated_mw: 200.0,
            p_min_mw: 10.0,
            ramp_rate_mw_per_min: 10.0,
            crank_time_min: 10.0,
            max_crank_distance_km: 500.0,
            auxiliary_load_mw: 2.0,
            priority: 1,
        }],
        load_blocks: vec![
            LoadBlock {
                block_id: 0,
                buses: vec![3],
                base_demand_mw: 20.0,
                cold_load_pickup_factor: 2.0,
                cold_load_decay_min: 30.0,
                priority: 1, // critical
                can_defer: false,
            },
            LoadBlock {
                block_id: 1,
                buses: vec![2],
                base_demand_mw: 10.0,
                cold_load_pickup_factor: 2.0,
                cold_load_decay_min: 30.0,
                priority: 2, // non-critical
                can_defer: true,
            },
        ],
        max_restoration_time_min: 240.0,
        frequency_tolerance_hz: 0.5,
        voltage_tolerance_pu: 0.05,
        max_generator_loading_pct: 80.0,
        reserve_margin_pct: 10.0,
    };

    let planner = BlackStartPlanner::new(config);
    let plan = planner.plan(&network).expect("plan should succeed");

    use oxigrid::optimize::restoration::black_start::RestorationAction;

    // Find the step times at which priority-1 and priority-2 blocks appear
    let mut priority1_time: Option<f64> = None;
    let mut priority2_time: Option<f64> = None;

    for step in &plan.steps {
        if let RestorationAction::PickupLoadBlock { block_id, .. } = &step.action {
            if *block_id == 0 && priority1_time.is_none() {
                priority1_time = Some(step.time_min);
            }
            if *block_id == 1 && priority2_time.is_none() {
                priority2_time = Some(step.time_min);
            }
        }
    }

    if let (Some(t1), Some(t2)) = (priority1_time, priority2_time) {
        assert!(
            t1 <= t2,
            "Priority-1 block should be restored no later than priority-2 (t1={:.1}, t2={:.1})",
            t1,
            t2
        );
    }
    // If only priority-1 was restored (priority-2 deferred), that is also acceptable
}

#[test]
fn test_frequency_response_nadir() {
    let config = BlackStartConfig::default();
    let planner = BlackStartPlanner::new(config);

    // Simulate a 50 MW load pickup with 500 MWs inertia and 100 MW/Hz governor
    let freq_traj = planner.simulate_frequency_response(50.0, 500.0, 100.0, 0.1, 30.0);

    assert!(
        !freq_traj.is_empty(),
        "Frequency trajectory should not be empty"
    );

    let nadir = freq_traj.iter().cloned().fold(f64::INFINITY, f64::min);

    // After a load step the frequency should dip below 50 Hz
    assert!(
        nadir < 50.0,
        "Frequency nadir should be below 50 Hz, got {:.3} Hz",
        nadir
    );
    // But not collapse (should stay above 48 Hz with these parameters)
    assert!(
        nadir > 48.0,
        "Frequency nadir should stay above 48 Hz, got {:.3} Hz",
        nadir
    );
}

#[test]
fn test_saidi_decreases_with_faster_restoration() {
    let network = make_three_bus_network();
    let customers_per_block = vec![1000usize, 500];

    // Slow config: large crank time
    let slow_config = BlackStartConfig {
        black_start_units: vec![BlackStartUnit {
            gen_id: 0,
            bus: 1,
            p_rated_mw: 200.0,
            p_min_mw: 10.0,
            ramp_rate_mw_per_min: 1.0, // slow ramp
            crank_time_min: 60.0,      // slow crank
            max_crank_distance_km: 500.0,
            auxiliary_load_mw: 2.0,
            priority: 1,
        }],
        load_blocks: vec![
            LoadBlock {
                block_id: 0,
                buses: vec![3],
                base_demand_mw: 10.0,
                cold_load_pickup_factor: 1.5,
                cold_load_decay_min: 30.0,
                priority: 1,
                can_defer: false,
            },
            LoadBlock {
                block_id: 1,
                buses: vec![2],
                base_demand_mw: 5.0,
                cold_load_pickup_factor: 1.5,
                cold_load_decay_min: 30.0,
                priority: 2,
                can_defer: true,
            },
        ],
        max_restoration_time_min: 480.0,
        frequency_tolerance_hz: 1.0,
        voltage_tolerance_pu: 0.1,
        max_generator_loading_pct: 80.0,
        reserve_margin_pct: 10.0,
    };

    // Fast config: same but fast ramp and short crank time
    let mut fast_config = slow_config.clone();
    fast_config.black_start_units[0].ramp_rate_mw_per_min = 20.0;
    fast_config.black_start_units[0].crank_time_min = 5.0;

    let slow_plan = BlackStartPlanner::new(slow_config)
        .plan(&network)
        .expect("slow plan should succeed");
    let fast_plan = BlackStartPlanner::new(fast_config)
        .plan(&network)
        .expect("fast plan should succeed");

    let metrics = RestorationMetrics::new(1500, 0.0);
    let saidi_slow = metrics.compute_saidi(&slow_plan, &customers_per_block);
    let saidi_fast = metrics.compute_saidi(&fast_plan, &customers_per_block);

    assert!(
        saidi_fast <= saidi_slow,
        "Faster restoration should give lower or equal SAIDI: fast={:.2}, slow={:.2}",
        saidi_fast,
        saidi_slow
    );
}

#[test]
fn test_ens_positive() {
    let network = make_three_bus_network();
    let config = make_simple_config();
    let planner = BlackStartPlanner::new(config.clone());
    let plan = planner.plan(&network).expect("plan should succeed");

    let metrics = RestorationMetrics::new(1000, 0.0);
    let ens = metrics.compute_ens(&plan, &config.load_blocks);

    assert!(
        ens > 0.0,
        "ENS should be positive whenever restoration takes time, got {:.4} MWh",
        ens
    );
}

#[test]
fn test_load_block_ordering_by_priority() {
    let blocks = vec![
        LoadBlock {
            block_id: 10,
            buses: vec![1],
            base_demand_mw: 5.0,
            cold_load_pickup_factor: 2.0,
            cold_load_decay_min: 30.0,
            priority: 3, // lowest
            can_defer: true,
        },
        LoadBlock {
            block_id: 11,
            buses: vec![2],
            base_demand_mw: 5.0,
            cold_load_pickup_factor: 2.0,
            cold_load_decay_min: 30.0,
            priority: 1, // highest
            can_defer: false,
        },
        LoadBlock {
            block_id: 12,
            buses: vec![3],
            base_demand_mw: 5.0,
            cold_load_pickup_factor: 2.0,
            cold_load_decay_min: 30.0,
            priority: 2, // middle
            can_defer: true,
        },
    ];

    let sequencer = RestorationSequencer::new(2);
    // Give ample headroom (100 MW available, 5 MW reserve)
    let order = sequencer.order_load_blocks(&blocks, 100.0, 5.0);

    // The result should contain at minimum the highest-priority block first
    assert!(!order.is_empty(), "Order should have at least one block");
    let first_id = order[0];
    let first_block = blocks.iter().find(|b| b.block_id == first_id).unwrap();
    let min_priority = blocks.iter().map(|b| b.priority).min().unwrap_or(1);
    assert_eq!(
        first_block.priority, min_priority,
        "First block should have highest priority (lowest priority number)"
    );
}

#[test]
fn test_no_black_start_units_returns_error() {
    let network = make_three_bus_network();
    let config = BlackStartConfig {
        black_start_units: vec![],
        load_blocks: vec![],
        ..Default::default()
    };
    let planner = BlackStartPlanner::new(config);
    let result = planner.plan(&network);
    assert!(
        result.is_err(),
        "Should return error when no BS units configured"
    );
}
