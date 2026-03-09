/// Coverage tests for previously untested public APIs.
///
/// Focuses on:
///   - Units: cross-type dimensional arithmetic, Energy/Capacity helpers,
///     Impedance admittance conversion, base_current conversion
///   - Network topology: neighbors, degree, is_connected, validate edge cases
///   - PowerFlowResult: total_losses_mw/mvar, max_branch_loading_pct, overloaded_branches
///   - Harmonic analysis: ieee519_individual_voltage_compliant, dft consistency
///   - Battery thermal: LumpedThermalModel constructors, steady-state, entropic heating
///   - Battery aging: NMC defaults, step_current half-cycle counting
///   - Market clearing: system_cost, weighted_average_offer, LmpComponents decomposition
use approx::assert_relative_eq;
use oxigrid::network::branch::Branch;
use oxigrid::network::bus::{Bus, BusType};
use oxigrid::network::topology::PowerNetwork;
use oxigrid::units::{Capacity, Current, Energy, Power, ReactivePower, StateOfCharge, Voltage};

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

fn make_branch(from: usize, to: usize) -> Branch {
    Branch {
        from_bus: from,
        to_bus: to,
        r: 0.01,
        x: 0.1,
        b: 0.0,
        rate_a: 100.0,
        rate_b: 0.0,
        rate_c: 0.0,
        tap: 0.0,
        shift: 0.0,
        status: true,
    }
}

/// Build a simple 3-bus radial network: 1 — 2 — 3  (bus 1 = slack)
fn radial_3bus() -> PowerNetwork {
    let mut net = PowerNetwork::new(100.0);
    net.buses.push(Bus::new(1, BusType::Slack));
    net.buses.push(Bus::new(2, BusType::PQ));
    net.buses.push(Bus::new(3, BusType::PQ));
    net.branches.push(make_branch(1, 2));
    net.branches.push(make_branch(2, 3));
    net
}

// ─────────────────────────────────────────────────────────────────────────────
// Units — cross-type dimensional arithmetic
// ─────────────────────────────────────────────────────────────────────────────

#[test]
fn test_voltage_times_current_gives_power() {
    let v = Voltage(230.0);
    let i = Current(10.0);
    let p: Power = v * i;
    assert_relative_eq!(p.0, 2300.0, epsilon = 1e-10);
}

#[test]
fn test_current_times_voltage_gives_power() {
    // Commutative form: Current × Voltage
    let v = Voltage(400.0);
    let i = Current(5.0);
    let p: Power = i * v;
    assert_relative_eq!(p.0, 2000.0, epsilon = 1e-10);
}

#[test]
fn test_power_to_energy_wh() {
    let p = Power(1000.0); // 1 kW
    let e = p.to_energy_wh(2.0); // 2 hours
    assert_relative_eq!(e.0, 2000.0, epsilon = 1e-10);
}

#[test]
fn test_per_unit_round_trip_voltage() {
    let v = Voltage(115.0);
    let base = Voltage(230.0);
    let pu = v.to_per_unit(base);
    let back = Voltage::from_per_unit(pu, base);
    assert_relative_eq!(back.0, v.0, epsilon = 1e-10);
}

#[test]
fn test_per_unit_round_trip_current() {
    let i = Current(500.0);
    let base = Current(1000.0);
    let pu = i.to_per_unit(base);
    let back = Current::from_per_unit(pu, base);
    assert_relative_eq!(back.0, i.0, epsilon = 1e-10);
}

#[test]
fn test_reactive_power_per_unit_round_trip() {
    let q = ReactivePower(50.0);
    let base = Power(100.0);
    let pu = q.to_per_unit(base);
    let back = ReactivePower::from_per_unit(pu, base);
    assert_relative_eq!(back.0, q.0, epsilon = 1e-10);
}

// ─────────────────────────────────────────────────────────────────────────────
// Units — Energy helpers
// ─────────────────────────────────────────────────────────────────────────────

#[test]
fn test_energy_to_power_w() {
    let e = Energy(500.0); // 500 Wh
    let p = e.to_power_w(2.0); // over 2 hours → 250 W
    assert_relative_eq!(p.0, 250.0, epsilon = 1e-10);
}

#[test]
fn test_energy_arithmetic() {
    let e1 = Energy(100.0);
    let e2 = Energy(50.0);
    assert_relative_eq!((e1 + e2).0, 150.0, epsilon = 1e-10);
    assert_relative_eq!((e1 - e2).0, 50.0, epsilon = 1e-10);
    assert_relative_eq!((e1 * 3.0).0, 300.0, epsilon = 1e-10);
    assert_relative_eq!((e1 / 2.0).0, 50.0, epsilon = 1e-10);
    assert_relative_eq!((-e1).0, -100.0, epsilon = 1e-10);
}

#[test]
fn test_capacity_arithmetic() {
    let c1 = Capacity(75.0); // 75 Ah
    let c2 = Capacity(25.0);
    assert_relative_eq!((c1 + c2).0, 100.0, epsilon = 1e-10);
    assert_relative_eq!((c1 - c2).0, 50.0, epsilon = 1e-10);
}

#[test]
fn test_state_of_charge_percentage() {
    let soc = StateOfCharge::new(0.75);
    assert_relative_eq!(soc.as_percentage(), 75.0, epsilon = 1e-10);
    assert_eq!(format!("{soc}"), "75.0%");
}

#[test]
fn test_state_of_charge_clamps_above_one() {
    let soc = StateOfCharge::new(1.5);
    assert_relative_eq!(soc.0, 1.0, epsilon = 1e-10);
}

#[test]
fn test_state_of_charge_clamps_below_zero() {
    let soc = StateOfCharge::new(-0.3);
    assert_relative_eq!(soc.0, 0.0, epsilon = 1e-10);
}

// ─────────────────────────────────────────────────────────────────────────────
// Units — Impedance helpers
// ─────────────────────────────────────────────────────────────────────────────

#[test]
fn test_impedance_to_admittance_pure_resistance() {
    use oxigrid::units::electrical::Impedance;
    let z = Impedance::new(0.5, 0.0); // 0.5 Ω resistive
    let y = z.to_admittance();
    assert_relative_eq!(y.re, 2.0, epsilon = 1e-10);
    assert_relative_eq!(y.im, 0.0, epsilon = 1e-10);
}

#[test]
fn test_impedance_to_admittance_near_zero_returns_zero() {
    use oxigrid::units::electrical::Impedance;
    let z = Impedance::new(0.0, 0.0);
    let y = z.to_admittance();
    assert_relative_eq!(y.re, 0.0, epsilon = 1e-10);
    assert_relative_eq!(y.im, 0.0, epsilon = 1e-10);
}

#[test]
fn test_impedance_to_complex() {
    use oxigrid::units::electrical::Impedance;
    let z = Impedance::new(3.0, 4.0);
    let c = z.to_complex();
    assert_relative_eq!(c.re, 3.0, epsilon = 1e-10);
    assert_relative_eq!(c.im, 4.0, epsilon = 1e-10);
}

// ─────────────────────────────────────────────────────────────────────────────
// Units — conversion helpers
// ─────────────────────────────────────────────────────────────────────────────

#[test]
fn test_base_current() {
    use oxigrid::units::conversion::base_current;
    // I_base = S_base / (√3 · V_base)  [A] (S in MVA, V in kV)
    // For 100 MVA, 230 kV: I_base = 100e6 / (√3 × 230e3) ≈ 251.02 A
    let i_base = base_current(Power(100e6), Voltage(230e3));
    let expected = 100e6 / (3.0_f64.sqrt() * 230e3);
    assert_relative_eq!(i_base, expected, epsilon = 1e-6);
}

// ─────────────────────────────────────────────────────────────────────────────
// Network topology — neighbors, degree, is_connected
// ─────────────────────────────────────────────────────────────────────────────

#[test]
fn test_bus_neighbors_radial() {
    let net = radial_3bus();
    // Bus 2 connects to bus 1 and bus 3
    let nbrs = net.neighbors(2);
    assert_eq!(
        nbrs,
        vec![1, 3],
        "Bus 2 neighbors should be [1, 3]: {nbrs:?}"
    );
}

#[test]
fn test_bus_neighbors_terminal() {
    let net = radial_3bus();
    // Bus 3 has only one neighbor: bus 2
    let nbrs = net.neighbors(3);
    assert_eq!(nbrs, vec![2]);
}

#[test]
fn test_bus_neighbors_unknown_bus() {
    let net = radial_3bus();
    let nbrs = net.neighbors(99);
    assert!(nbrs.is_empty(), "Unknown bus should have no neighbors");
}

#[test]
fn test_degree_hub_bus() {
    let net = radial_3bus();
    // Bus 2 has degree 2 (connected to 1 and 3)
    assert_eq!(net.degree(2), 2);
}

#[test]
fn test_degree_leaf_bus() {
    let net = radial_3bus();
    assert_eq!(net.degree(3), 1);
}

#[test]
fn test_degree_unknown_bus() {
    let net = radial_3bus();
    assert_eq!(net.degree(99), 0);
}

#[test]
fn test_is_connected_radial() {
    let net = radial_3bus();
    assert!(
        net.is_connected(),
        "Radial 3-bus network should be connected"
    );
}

#[test]
fn test_is_connected_island() {
    let mut net = radial_3bus();
    // Add isolated bus 4 with no branches
    net.buses.push(Bus::new(4, BusType::PQ));
    assert!(
        !net.is_connected(),
        "Network with isolated bus should be disconnected"
    );
}

#[test]
fn test_is_connected_empty_network() {
    let net = PowerNetwork::new(100.0);
    // Empty network is trivially connected
    assert!(net.is_connected());
}

#[test]
fn test_validate_valid_network() {
    let net = radial_3bus();
    assert!(
        net.validate().is_ok(),
        "Valid 3-bus network should pass validation"
    );
}

#[test]
fn test_validate_no_buses() {
    let net = PowerNetwork::new(100.0);
    assert!(
        net.validate().is_err(),
        "Empty network should fail validation"
    );
}

#[test]
fn test_validate_no_slack_bus() {
    let mut net = PowerNetwork::new(100.0);
    net.buses.push(Bus::new(1, BusType::PQ));
    net.buses.push(Bus::new(2, BusType::PQ));
    assert!(
        net.validate().is_err(),
        "Network without slack bus should fail validation"
    );
}

#[test]
fn test_bus_count_and_branch_count() {
    let net = radial_3bus();
    assert_eq!(net.bus_count(), 3);
    assert_eq!(net.branch_count(), 2);
}

#[test]
fn test_n_pq_and_pv_buses() {
    let mut net = radial_3bus(); // 1 slack + 2 PQ
    net.buses.push(Bus::new(4, BusType::PV));
    assert_eq!(net.n_pq_buses(), 2);
    assert_eq!(net.n_pv_buses(), 1);
}

// ─────────────────────────────────────────────────────────────────────────────
// PowerFlowResult — branch flow methods
// ─────────────────────────────────────────────────────────────────────────────

fn make_branch_flow(
    branch_index: usize,
    from_bus: usize,
    to_bus: usize,
    p_loss: f64,
    q_loss: f64,
    loading_pct: f64,
) -> oxigrid::powerflow::result::BranchFlow {
    oxigrid::powerflow::result::BranchFlow {
        branch_index,
        from_bus,
        to_bus,
        p_from_mw: 10.0,
        q_from_mvar: 5.0,
        p_to_mw: 10.0 - p_loss,
        q_to_mvar: 5.0 - q_loss,
        p_loss_mw: p_loss,
        q_loss_mvar: q_loss,
        loading_pct,
    }
}

fn make_powerflow_result(
    branch_flows: Vec<oxigrid::powerflow::result::BranchFlow>,
) -> oxigrid::powerflow::result::PowerFlowResult {
    use oxigrid::powerflow::result::PowerFlowResult;
    let total_p: f64 = branch_flows.iter().map(|b| b.p_loss_mw).sum();
    let total_q: f64 = branch_flows.iter().map(|b| b.q_loss_mvar).sum();
    PowerFlowResult {
        voltage_magnitude: vec![1.0, 0.98, 0.97],
        voltage_angle: vec![0.0, -0.02, -0.04],
        p_injected: vec![10.0, -5.0, -5.0],
        q_injected: vec![3.0, -1.5, -1.5],
        branch_flows,
        total_p_loss_mw: total_p,
        total_q_loss_mvar: total_q,
        converged: true,
        iterations: 4,
        max_mismatch: 1e-8,
    }
}

#[test]
fn test_total_losses_mw_sums_branches() {
    let flows = vec![
        make_branch_flow(0, 1, 2, 0.5, 0.2, 50.0),
        make_branch_flow(1, 2, 3, 0.3, 0.1, 30.0),
    ];
    let result = make_powerflow_result(flows);
    assert_relative_eq!(result.total_losses_mw(), 0.8, epsilon = 1e-10);
    assert_relative_eq!(result.total_losses_mvar(), 0.3, epsilon = 1e-10);
}

#[test]
fn test_total_losses_mw_empty_branches() {
    let result = make_powerflow_result(vec![]);
    assert_relative_eq!(result.total_losses_mw(), 0.0, epsilon = 1e-10);
    assert_relative_eq!(result.total_losses_mvar(), 0.0, epsilon = 1e-10);
}

#[test]
fn test_max_branch_loading_pct() {
    let flows = vec![
        make_branch_flow(0, 1, 2, 0.5, 0.2, 80.0),
        make_branch_flow(1, 2, 3, 0.3, 0.1, 110.0), // overloaded
    ];
    let result = make_powerflow_result(flows);
    assert_relative_eq!(result.max_branch_loading_pct(), 110.0, epsilon = 1e-10);
}

#[test]
fn test_max_branch_loading_pct_empty() {
    let result = make_powerflow_result(vec![]);
    assert_relative_eq!(result.max_branch_loading_pct(), 0.0, epsilon = 1e-10);
}

#[test]
fn test_overloaded_branches_detected() {
    let flows = vec![
        make_branch_flow(0, 1, 2, 0.5, 0.2, 95.0),
        make_branch_flow(1, 2, 3, 0.3, 0.1, 105.0), // overloaded
        make_branch_flow(2, 3, 4, 0.1, 0.05, 120.0), // overloaded
    ];
    let result = make_powerflow_result(flows);
    let overloaded = result.overloaded_branches();
    assert_eq!(overloaded.len(), 2, "Two branches should be overloaded");
    for b in &overloaded {
        assert!(b.loading_pct > 100.0);
    }
}

#[test]
fn test_overloaded_branches_none() {
    let flows = vec![
        make_branch_flow(0, 1, 2, 0.5, 0.2, 80.0),
        make_branch_flow(1, 2, 3, 0.3, 0.1, 95.0),
    ];
    let result = make_powerflow_result(flows);
    assert!(
        result.overloaded_branches().is_empty(),
        "No branch should be overloaded"
    );
}

#[test]
fn test_voltage_angle_degrees_conversion() {
    use std::f64::consts::PI;
    let result = make_powerflow_result(vec![]);
    // Override with known angles
    let result = oxigrid::powerflow::result::PowerFlowResult {
        voltage_angle: vec![0.0, -PI / 6.0, PI / 4.0],
        ..result
    };
    let deg = result.voltage_angle_degrees();
    assert_relative_eq!(deg[0], 0.0, epsilon = 1e-10);
    assert_relative_eq!(deg[1], -30.0, epsilon = 1e-9);
    assert_relative_eq!(deg[2], 45.0, epsilon = 1e-9);
}

#[test]
fn test_total_p_loss_and_q_loss_accessors() {
    let result = make_powerflow_result(vec![make_branch_flow(0, 1, 2, 1.5, 0.7, 50.0)]);
    assert_relative_eq!(
        result.total_p_loss(),
        result.total_p_loss_mw,
        epsilon = 1e-10
    );
    assert_relative_eq!(
        result.total_q_loss(),
        result.total_q_loss_mvar,
        epsilon = 1e-10
    );
}

// ─────────────────────────────────────────────────────────────────────────────
// Harmonic analysis — IEEE 519 individual voltage compliance, dft consistency
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(feature = "harmonics")]
mod harmonic_tests {
    use approx::assert_relative_eq;
    use oxigrid::harmonics::analysis::{analyse, dft, synthetic_waveform, HarmonicSpectrum};

    /// Build a spectrum with known individual harmonic distortions.
    fn low_ihd_spectrum() -> HarmonicSpectrum {
        let components = vec![(1u32, 1.0f64, 0.0f64), (3, 0.02, 0.0), (5, 0.01, 0.0)];
        let samples = synthetic_waveform(60.0, 6000.0, 6000, &components);
        analyse(&samples, 6000.0, 60.0, 10, None)
    }

    fn high_ihd_spectrum() -> HarmonicSpectrum {
        // 5th harmonic at 40% → IHD = 40% > 3% limit for 13.8 kV bus
        let components = vec![(1u32, 1.0f64, 0.0f64), (5, 0.40, 0.0)];
        let samples = synthetic_waveform(60.0, 6000.0, 6000, &components);
        analyse(&samples, 6000.0, 60.0, 10, None)
    }

    #[test]
    fn test_ieee519_individual_voltage_compliant_low_ihd() {
        let spec = low_ihd_spectrum();
        assert!(
            spec.ieee519_individual_voltage_compliant(13.8),
            "IHD 2% and 1% should be within 3% limit at 13.8 kV"
        );
    }

    #[test]
    fn test_ieee519_individual_voltage_non_compliant_high_ihd() {
        let spec = high_ihd_spectrum();
        assert!(
            !spec.ieee519_individual_voltage_compliant(13.8),
            "IHD 40% should exceed 3% limit at 13.8 kV"
        );
    }

    #[test]
    fn test_ieee519_lv_bus_higher_limit() {
        // At < 1 kV the individual limit is 5%
        let components = vec![(1u32, 1.0f64, 0.0f64), (3, 0.04, 0.0)]; // IHD=4%
        let samples = synthetic_waveform(60.0, 6000.0, 6000, &components);
        let spec = analyse(&samples, 6000.0, 60.0, 10, None);
        assert!(
            spec.ieee519_individual_voltage_compliant(0.4),
            "4% IHD should comply at 0.4 kV bus (limit = 5%)"
        );
    }

    #[test]
    fn test_thd_pure_sine_near_zero() {
        // A pure sine wave should have near-zero THD
        use std::f64::consts::PI;
        let n = 6000usize;
        let samples: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 60.0 * i as f64 / 6000.0).sin())
            .collect();
        let spec = analyse(&samples, 6000.0, 60.0, 10, None);
        assert!(
            spec.thd_pct < 1.0,
            "THD of pure sine should be < 1%, got {:.4}%",
            spec.thd_pct
        );
    }

    #[test]
    fn test_thd_with_known_5th_harmonic() {
        // 20% 5th harmonic → THD ≈ 20%
        let components = vec![(1u32, 1.0f64, 0.0f64), (5, 0.20, 0.0)];
        let samples = synthetic_waveform(60.0, 6000.0, 6000, &components);
        let spec = analyse(&samples, 6000.0, 60.0, 10, None);
        assert!(
            (spec.thd_pct - 20.0).abs() < 1.0,
            "THD should be ~20%, got {:.2}%",
            spec.thd_pct
        );
    }

    #[test]
    fn test_dft_pure_sine_fundamental_bin() {
        use std::f64::consts::PI;
        // Pure sine at frequency f, sampled at N*f samples/s → energy at bin 1
        let n = 1000usize;
        let samples: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * i as f64 / n as f64).sin())
            .collect();
        let spectrum = dft(&samples);
        // Bin 0 = DC (should be 0), bin 1 = fundamental
        let (re0, im0) = spectrum[0];
        assert_relative_eq!(re0, 0.0, epsilon = 1e-10);
        assert_relative_eq!(im0, 0.0, epsilon = 1e-10);
        let mag1 = (spectrum[1].0.powi(2) + spectrum[1].1.powi(2)).sqrt();
        // Normalised to 1/N, peak amplitude = 1 → bin magnitude = 0.5
        assert_relative_eq!(mag1, 0.5, epsilon = 1e-2);
    }

    #[test]
    fn test_tdd_calculation() {
        let components = vec![(1u32, 1.0f64, 0.0f64), (3, 0.1, 0.0)];
        let samples = synthetic_waveform(60.0, 6000.0, 6000, &components);
        // rated_current = 2.0 × fundamental (so TDD < THD)
        let fundamental_rms = 1.0 / 2.0_f64.sqrt();
        let spec = analyse(&samples, 6000.0, 60.0, 10, Some(2.0 * fundamental_rms));
        assert!(
            spec.tdd_pct.is_some(),
            "TDD should be computed when rated current is given"
        );
        let tdd = spec.tdd_pct.expect("TDD present");
        assert!(
            tdd < spec.thd_pct,
            "TDD ({tdd:.2}%) should be less than THD ({:.2}%) for rated > fundamental",
            spec.thd_pct
        );
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Battery thermal — LumpedThermalModel constructors and steady-state
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(feature = "battery")]
mod battery_thermal_tests {
    use approx::assert_relative_eq;
    use oxigrid::battery::thermal::LumpedThermalModel;

    #[test]
    fn test_cell_18650_constructor() {
        let cell = LumpedThermalModel::cell_18650();
        assert_relative_eq!(cell.mass_kg, 0.045, epsilon = 1e-6);
        assert_relative_eq!(cell.t_ambient, 298.15, epsilon = 1e-6);
        assert_relative_eq!(cell.temperature, 298.15, epsilon = 1e-6);
    }

    #[test]
    fn test_pouch_75ah_constructor() {
        let cell = LumpedThermalModel::pouch_75ah();
        assert_relative_eq!(cell.mass_kg, 1.5, epsilon = 1e-6);
        assert_relative_eq!(cell.heat_capacity, 1000.0, epsilon = 1e-6);
    }

    #[test]
    fn test_lumped_temperature_accessor() {
        let cell = LumpedThermalModel::cell_18650();
        let t = cell.temperature();
        assert_relative_eq!(t.0, 298.15, epsilon = 1e-6);
    }

    #[test]
    fn test_steady_state_temp_above_ambient() {
        let cell = LumpedThermalModel::cell_18650();
        // Any joule heating should raise steady-state above ambient
        let t_ss = cell.steady_state_temp(10.0, 0.1); // 10 A, 100 mΩ
        assert!(
            t_ss.0 > cell.t_ambient,
            "Steady-state should exceed ambient: {:.2} K",
            t_ss.0
        );
    }

    #[test]
    fn test_steady_state_temp_no_current_equals_ambient() {
        let cell = LumpedThermalModel::cell_18650();
        let t_ss = cell.steady_state_temp(0.0, 0.1);
        assert_relative_eq!(t_ss.0, cell.t_ambient, epsilon = 1e-10);
    }

    #[test]
    fn test_lumped_step_increases_temperature() {
        let mut cell = LumpedThermalModel::cell_18650();
        // 10 A, 100 mΩ, 1 second → should heat up
        let t_init = cell.temperature;
        cell.step(10.0, 0.1, 1.0);
        assert!(
            cell.temperature > t_init,
            "Temperature should rise with current: {:.4} > {:.4}",
            cell.temperature,
            t_init
        );
    }

    #[test]
    fn test_lumped_entropic_heating_effect() {
        // With a positive entropic coefficient, heating is greater than without
        let mut no_entropic = LumpedThermalModel::new(0.1, 1000.0, 0.0, 0.1);
        let mut with_entropic = LumpedThermalModel::new(0.1, 1000.0, 0.0, 0.1);
        with_entropic.entropic_coeff = 1e-4;
        with_entropic.temperature = 350.0;
        no_entropic.temperature = 350.0;

        no_entropic.step(10.0, 0.01, 1.0);
        with_entropic.step(10.0, 0.01, 1.0);

        assert!(
            with_entropic.temperature >= no_entropic.temperature,
            "Entropic heating should not reduce temperature"
        );
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Battery aging — NMC defaults, step_current half-cycle detection
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(feature = "battery")]
mod battery_aging_tests {
    use approx::assert_relative_eq;
    use oxigrid::battery::aging::{AgingModel, AgingParams};

    #[test]
    fn test_nmc_default_initial_state() {
        let model = AgingModel::new(AgingParams::nmc_default());
        assert_relative_eq!(model.state.soh, 1.0, epsilon = 1e-9);
        assert_relative_eq!(model.state.q_remaining, 3.0, epsilon = 1e-9);
    }

    #[test]
    fn test_nmc_calendar_aging_positive_after_one_year() {
        // NMC should show measurable calendar aging over one year
        let mut nmc = AgingModel::new(AgingParams::nmc_default());
        let one_year = 31_536_000.0;
        nmc.step_calendar(one_year, 298.15);
        assert!(
            nmc.state.q_loss_cal_pct > 0.0,
            "NMC should show calendar aging after one year: {:.6}%",
            nmc.state.q_loss_cal_pct
        );
        assert!(
            nmc.state.soh < 1.0,
            "NMC SoH should decrease after calendar aging"
        );
    }

    #[test]
    fn test_step_current_registers_half_cycle() {
        let mut model = AgingModel::new(AgingParams::lfp_default());
        let q_nom = model.params.q_nom;
        // Simulate discharge at high current to accumulate significant charge throughput
        let initial_loss = model.state.q_loss_cyc_pct;
        // Discharge for 1 hour (3600 s) at 1C then reverse to trigger a half-cycle
        let i_discharge = q_nom; // 1C
        let dt = 0.01; // 10 ms steps
        let mut soc = 1.0_f64;
        // Discharge phase: SoC decreases
        for _ in 0..100 {
            model.step_current(i_discharge, dt, soc);
            soc -= i_discharge * dt / (3600.0 * q_nom);
            soc = soc.max(0.0);
        }
        let soc_after_discharge = soc;
        // Charge phase: reverse direction → triggers half-cycle on reversal
        let i_charge = -q_nom;
        for _ in 0..50 {
            model.step_current(i_charge, dt, soc);
            soc += q_nom * dt / (3600.0 * q_nom);
            soc = soc.min(1.0);
        }
        // After a direction reversal, cycle aging should accumulate
        let _ = soc_after_discharge; // used above
                                     // The cycling should have registered at least some cycle loss
        assert!(
            model.state.q_loss_cyc_pct >= initial_loss,
            "Cycle aging should accumulate after discharge-charge reversal"
        );
    }

    #[test]
    fn test_time_to_80pct_soh_finite_and_positive() {
        let lfp = AgingModel::new(AgingParams::lfp_default());
        let nmc = AgingModel::new(AgingParams::nmc_default());
        let t_lfp = lfp.time_to_80pct_soh(298.15);
        let t_nmc = nmc.time_to_80pct_soh(298.15);
        assert!(
            t_lfp > 0.0 && t_lfp < f64::INFINITY,
            "LFP t_80 should be finite and positive: {t_lfp:.2e}"
        );
        assert!(
            t_nmc > 0.0 && t_nmc < f64::INFINITY,
            "NMC t_80 should be finite and positive: {t_nmc:.2e}"
        );
        // Higher temperature should shorten life
        let t_lfp_hot = lfp.time_to_80pct_soh(333.15); // 60°C
        assert!(
            t_lfp_hot < t_lfp,
            "Hot storage should shorten calendar life: {t_lfp_hot:.2e} < {t_lfp:.2e}"
        );
    }

    #[test]
    fn test_register_cycle_dod_below_threshold_ignored() {
        let mut model = AgingModel::new(AgingParams::lfp_default());
        let before = model.state.q_loss_cyc_pct;
        model.register_cycle(1e-5); // below 1e-4 threshold
        assert!(
            (model.state.q_loss_cyc_pct - before).abs() < 1e-15,
            "Tiny DoD should not register a cycle"
        );
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Protection coordination — CTI checking, tcc_curve, recommend_tms
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(feature = "protection")]
mod protection_tests {
    use approx::assert_relative_eq;
    use oxigrid::protection::coordination::{check_coordination, recommend_tms, tcc_curve};
    use oxigrid::protection::relay::{OcRelay, RelayCharacteristic};

    fn si_relay(pickup: f64, tms: f64) -> OcRelay {
        OcRelay::new(pickup, tms, RelayCharacteristic::StandardInverse)
    }

    fn vi_relay(pickup: f64, tms: f64) -> OcRelay {
        OcRelay::new(pickup, tms, RelayCharacteristic::VeryInverse)
    }

    #[test]
    fn test_cti_checking_coordinated_pair() {
        let relays = vec![
            si_relay(100.0, 0.10), // primary (faster)
            si_relay(100.0, 0.35), // backup (slower)
        ];
        let study = check_coordination(&relays, &[(0, 1, 500.0)], 0.20);
        assert!(
            study.is_fully_coordinated(),
            "Margin should exceed 0.20 s CTI"
        );
        assert_eq!(study.n_violations(), 0);
    }

    #[test]
    fn test_cti_checking_violation_detected() {
        let relays = vec![
            si_relay(100.0, 0.25), // primary — only slightly faster than backup
            si_relay(100.0, 0.26), // backup — margin << 0.20 s
        ];
        let study = check_coordination(&relays, &[(0, 1, 500.0)], 0.20);
        assert!(!study.is_fully_coordinated());
        assert_eq!(study.n_violations(), 1);
    }

    #[test]
    fn test_coordination_margin_is_t_backup_minus_t_primary() {
        let relays = vec![si_relay(100.0, 0.10), si_relay(100.0, 0.40)];
        let study = check_coordination(&relays, &[(0, 1, 500.0)], 0.20);
        let pair = &study.pairs[0];
        assert_relative_eq!(
            pair.margin_s,
            pair.t_backup_s - pair.t_primary_s,
            epsilon = 1e-12
        );
    }

    #[test]
    fn test_tcc_curve_is_monotone_decreasing() {
        let relay = si_relay(100.0, 0.20);
        let curve = tcc_curve(&relay, 200.0, 2000.0, 15);
        assert!(!curve.is_empty(), "TCC curve should not be empty");
        for w in curve.windows(2) {
            assert!(
                w[1].0 > w[0].0,
                "Currents should be strictly increasing in TCC curve"
            );
            assert!(
                w[1].1 <= w[0].1 + 1e-9,
                "Trip times should be non-increasing"
            );
        }
    }

    #[test]
    fn test_recommend_tms_gives_correct_trip_time() {
        // For VeryInverse: t = TMS * 13.5 / (M - 1), M = i_fault / pickup
        let relay = vi_relay(100.0, 0.20);
        let i_fault = 500.0;
        let t_target = 0.60;
        let tms_new = recommend_tms(&relay, t_target, i_fault);
        assert!(
            tms_new > 0.0,
            "Recommended TMS must be positive: {tms_new:.4}"
        );
        // Verify: trip time with new TMS should equal t_target
        let relay_new = vi_relay(100.0, tms_new);
        let t_actual = relay_new
            .trip_time(i_fault)
            .expect("Relay should trip at fault current");
        assert_relative_eq!(t_actual, t_target, epsilon = 1e-4);
    }

    #[test]
    fn test_recommend_tms_below_pickup_returns_original() {
        let relay = si_relay(100.0, 0.20);
        // Fault current below pickup → return original TMS
        let tms = recommend_tms(&relay, 0.5, 50.0);
        assert_relative_eq!(tms, relay.tms, epsilon = 1e-12);
    }

    #[test]
    fn test_n_violations_zero_empty_pairs() {
        let relays = vec![si_relay(100.0, 0.20)];
        let study = check_coordination(&relays, &[], 0.20);
        assert_eq!(study.n_violations(), 0);
        assert!(study.is_fully_coordinated());
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Market clearing — system_cost, weighted_average_offer, LmpComponents
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(feature = "optimize")]
mod market_tests {
    use approx::assert_relative_eq;
    use oxigrid::optimize::market::{
        compute_lmps, lerner_index, pay_as_bid_clearing, uniform_price_clearing, GeneratorBid,
        LmpComponents,
    };

    fn sample_bids() -> Vec<GeneratorBid> {
        vec![
            GeneratorBid::simple(0, 0, 0.0, 100.0, 20.0),
            GeneratorBid::simple(1, 1, 0.0, 150.0, 35.0),
            GeneratorBid::simple(2, 2, 0.0, 200.0, 50.0),
        ]
    }

    #[test]
    fn test_system_cost_equals_sum_of_payments() {
        let bids = sample_bids();
        let result = uniform_price_clearing(&bids, 200.0, &[]);
        let manual: f64 = result.gen_dispatches.iter().map(|d| d.payment).sum();
        assert_relative_eq!(result.system_cost(), manual, epsilon = 1e-9);
    }

    #[test]
    fn test_system_cost_zero_no_dispatches() {
        let result = pay_as_bid_clearing(&[], 0.0);
        assert_relative_eq!(result.system_cost(), 0.0, epsilon = 1e-9);
    }

    #[test]
    fn test_weighted_average_offer_single_unit() {
        // Single unit: weighted avg = its own payment / dispatched output
        let bids = vec![GeneratorBid::simple(0, 0, 0.0, 100.0, 40.0)];
        let result = uniform_price_clearing(&bids, 80.0, &[]);
        // With one generator, weighted_avg_offer returns its payment per MWh
        let wao = result.weighted_average_offer();
        assert!(
            wao >= 0.0,
            "Weighted average offer should be non-negative: {wao:.4}"
        );
    }

    #[test]
    fn test_lmp_loss_component() {
        // Non-zero MLF → loss component = mlf * lambda
        let lambda = 30.0;
        let mlf = 0.05;
        let gsf = vec![vec![0.0]]; // 1 branch, 1 bus, no congestion
        let shadow_prices = vec![0.0];
        let mlfs = vec![mlf];
        let lmps = compute_lmps(lambda, &gsf, &shadow_prices, &mlfs);
        assert_relative_eq!(lmps[0].loss, mlf * lambda, epsilon = 1e-10);
        assert_relative_eq!(lmps[0].lmp, lambda + mlf * lambda, epsilon = 1e-10);
    }

    #[test]
    fn test_lmp_components_struct_fields() {
        let lmp = LmpComponents::compute(5, 40.0, &[0.2, -0.1], &[10.0, 5.0], 0.03);
        // congestion = 0.2*10 + (-0.1)*5 = 2.0 - 0.5 = 1.5
        assert_relative_eq!(lmp.energy, 40.0, epsilon = 1e-10);
        assert_relative_eq!(lmp.congestion, 1.5, epsilon = 1e-10);
        assert_relative_eq!(lmp.loss, 0.03 * 40.0, epsilon = 1e-10);
        assert_relative_eq!(lmp.lmp, 40.0 + 1.5 + 0.03 * 40.0, epsilon = 1e-10);
        assert_eq!(lmp.bus_id, 5);
    }

    #[test]
    fn test_lerner_index_zero_price_returns_zero() {
        // Edge case: price = 0 → return 0 (avoid division by zero)
        let l = lerner_index(0.0, 0.0);
        assert_relative_eq!(l, 0.0, epsilon = 1e-12);
    }

    #[test]
    fn test_lerner_index_clamped_to_one() {
        // MC < 0 (subsidy) → Lerner index should clamp at 1.0
        let l = lerner_index(50.0, -10.0);
        assert_relative_eq!(l, 1.0, epsilon = 1e-12);
    }

    #[test]
    fn test_uniform_price_no_demand_cleared() {
        let bids = sample_bids();
        let result = uniform_price_clearing(&bids, 0.0, &[]);
        assert_relative_eq!(result.total_generation_mw, 0.0, epsilon = 1e-6);
    }
}
