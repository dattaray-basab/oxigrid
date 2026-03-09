#![cfg(feature = "stability")]
use oxigrid::stability::generator::avr::{Avr1Params, Avr1State, AvrGenerator};
use oxigrid::stability::generator::classical::{ClassicalGenerator, ClassicalGeneratorParams};
use oxigrid::stability::generator::governor::{DroopGovernor, Tgov1, Tgov1Params};
use oxigrid::stability::small_signal::SmallSignalModel;
use oxigrid::stability::transient::{ClassicalGen, GenState, TransientSim};
use std::f64::consts::PI;

// ── Classical Generator (SMIB) ───────────────────────────────────────────────

#[test]
fn test_classical_generator_steady_state() {
    let params = ClassicalGeneratorParams::steam_600mw();
    let tm = 0.8;
    let delta0 = 0.5;
    let mut gen = ClassicalGenerator::new(params, delta0, tm);
    // Compute te so that te == tm → no acceleration
    let te = gen.te_smib(1.0, 0.4);
    let state0 = gen.state(te);
    gen.step_rk4(state0.tm, 0.001);
    let state1 = gen.state(state0.tm);
    // Rotor angle should barely change
    assert!(
        (state1.delta - state0.delta).abs() < 1e-3,
        "Angle drifted too much: Δδ = {:.4e}",
        state1.delta - state0.delta
    );
}

#[test]
fn test_critical_clearing_angle() {
    let delta_s = 0.4_f64;
    let delta_cc = ClassicalGenerator::critical_clearing_angle_smib(delta_s);
    assert!(
        delta_cc > delta_s,
        "CCA {:.3} must be > δs {:.3}",
        delta_cc,
        delta_s
    );
    assert!(
        delta_cc < PI - delta_s,
        "CCA {:.3} must be < π−δs {:.3}",
        delta_cc,
        PI - delta_s
    );
}

#[test]
fn test_smib_fault_simulation() {
    let params = ClassicalGeneratorParams::steam_600mw();
    // Equilibrium angle for Tm=0.8, E'=1.05, V=1.0, x=0.4:
    // delta_eq = arcsin(Tm*x/(E'*V)) = arcsin(0.8*0.4/1.05) ≈ 0.309 rad
    let delta_eq = (0.8_f64 * 0.4 / 1.05).asin();
    let mut gen = ClassicalGenerator::new(params, delta_eq, 0.8);
    let states = gen.simulate_smib_fault(1.0, 0.4, 0.8, 0.4, 0.1, 0.2, 2.0, 0.001);
    assert!(!states.is_empty(), "No states returned");
    // Simulation should run to near t_end without diverging
    let last = states.last().unwrap();
    assert!(
        last.time_s >= 1.9,
        "Simulation ended too early: {:.3}",
        last.time_s
    );
    assert!(last.delta.is_finite(), "Rotor angle diverged to infinity");
}

// ── TGOV1 Governor ───────────────────────────────────────────────────────────

#[test]
fn test_governor_droop_steady_state() {
    let gov = DroopGovernor::new(0.05, 1.0);
    let pm = gov.mechanical_power(1.0);
    assert!((pm - 1.0).abs() < 1e-10, "Pm at rated speed = {pm:.6}");
}

#[test]
fn test_governor_droop_under_speed() {
    let gov = DroopGovernor::new(0.05, 0.8);
    let pm_rated = gov.mechanical_power(1.0);
    let pm_under = gov.mechanical_power(0.98);
    assert!(
        pm_under > pm_rated,
        "Governor should increase output at low speed: {pm_under:.4} vs {pm_rated:.4}"
    );
}

#[test]
fn test_tgov1_step_response() {
    let params = Tgov1Params::steam_typical();
    let mut gov = Tgov1::new(params, 0.8);
    let pm_init = gov.state.p_m;
    // Under-speed: governor should increase power over time
    for _ in 0..200 {
        gov.step(0.99, 0.01);
    }
    assert!(
        gov.state.p_m > pm_init,
        "TGOV1 should increase Pm on under-speed: {:.4} vs {:.4}",
        gov.state.p_m,
        pm_init
    );
}

#[test]
fn test_tgov1_valve_limits() {
    let params = Tgov1Params::steam_typical();
    let v_max = params.v_max;
    let v_min = params.v_min;
    let mut gov = Tgov1::new(params, 0.0);
    // Extreme under-speed: valve should saturate at v_max
    for _ in 0..1000 {
        gov.step(0.5, 0.01);
    }
    assert!(
        gov.state.p_m <= v_max + 1e-6,
        "Governor exceeded valve max: {:.4} > {:.4}",
        gov.state.p_m,
        v_max
    );
    assert!(
        gov.state.p_m >= v_min - 1e-6,
        "Governor below valve min: {:.4} < {:.4}",
        gov.state.p_m,
        v_min
    );
}

// ── AVR ──────────────────────────────────────────────────────────────────────

#[test]
fn test_avr_initialisation() {
    let params = Avr1Params::steam_typical();
    let state = Avr1State::from_steady_state(1.2, 1.0, &params);
    assert!(state.efd > 0.0, "Efd should be positive: {}", state.efd);
    assert!(state.vref > 0.0, "Vref should be positive: {}", state.vref);
}

#[test]
fn test_avr_voltage_step_reduces_efd() {
    let params = Avr1Params::steam_typical();
    let (_, efds) = Avr1State::simulate_voltage_step(1.2, 1.0, 1.05, 0.5, 5.0, 0.005, &params);
    let efd0 = efds[0];
    let efd_end = *efds.last().unwrap();
    assert!(
        efd_end < efd0,
        "Efd should decrease for Vt step up: {efd0:.4} → {efd_end:.4}"
    );
}

#[test]
fn test_avr_generator_stable() {
    let params = Avr1Params::hydro_slow();
    let mut avr = AvrGenerator::new(params, 1.1, 1.0);
    let efd0 = avr.efd();
    for _ in 0..200 {
        avr.step(1.0, 0.01);
    }
    assert!(
        (avr.efd() - efd0).abs() < 0.02,
        "AVR drifted: {efd0:.4} → {:.4}",
        avr.efd()
    );
}

#[test]
fn test_avr_saturation_nonzero() {
    let params = Avr1Params::steam_typical();
    // Saturation should grow with field voltage
    let se1 = params.saturation(1.0);
    let se2 = params.saturation(2.0);
    assert!(
        se2 > se1,
        "Saturation should increase with Efd: se(1)={se1:.4} se(2)={se2:.4}"
    );
}

// ── Small-Signal Stability ───────────────────────────────────────────────────

#[test]
fn test_smib_small_signal_stable() {
    let model = SmallSignalModel::smib(6.0, 2.0, 60.0, 1.5);
    let modes = model.oscillation_modes();
    let osc: Vec<_> = modes.iter().filter(|m| m.is_oscillatory()).collect();
    assert!(!osc.is_empty(), "Expected at least one oscillatory mode");
    for m in &osc {
        assert!(
            m.is_stable(),
            "Oscillatory mode should be stable: σ={:.4}",
            m.sigma
        );
        assert!(
            m.freq_hz > 0.3 && m.freq_hz < 5.0,
            "Electromechanical mode freq out of range: {:.3} Hz",
            m.freq_hz
        );
    }
}

#[test]
fn test_two_machine_modes() {
    // Use small damping so modes are underdamped/oscillatory
    let m1 = 6.0 / (2.0 * PI * 60.0);
    let m2 = 4.0 / (2.0 * PI * 60.0);
    let ks = 2.0;
    let model = SmallSignalModel::new(
        vec![m1, m2],
        vec![0.05, 0.05], // small damping → oscillatory modes
        vec![vec![ks, -ks / 2.0], vec![-ks / 2.0, ks]],
    );
    let modes = model.oscillation_modes();
    assert!(
        !modes.is_empty(),
        "Two-machine system should have oscillation modes"
    );
}

#[test]
fn test_damping_ratio_positive_with_damping() {
    let model = SmallSignalModel::smib(6.0, 3.0, 60.0, 2.0);
    let modes = model.oscillation_modes();
    for m in modes.iter().filter(|m| m.is_oscillatory()) {
        assert!(
            m.damping_ratio > 0.0,
            "Damping ratio should be positive: ζ={:.4}",
            m.damping_ratio
        );
    }
}

#[test]
fn test_small_signal_eigenvalues_count() {
    // n generators → 2n eigenvalues (n angle + n speed states)
    let model = SmallSignalModel::smib(6.0, 2.0, 60.0, 1.5);
    let eigs = model.eigenvalues();
    assert_eq!(eigs.len(), 2, "SMIB should have 2 eigenvalues");
}

// ── Transient Stability Simulator ────────────────────────────────────────────

#[test]
fn test_transient_sim_single_machine() {
    let gen = ClassicalGen::thermal_unit();
    let v_inf = 1.0;
    let x_tot = gen.xd_prime + 0.3;
    let sim = TransientSim::smib(gen, v_inf, x_tot);
    let initial = vec![GenState::new(0.4)];
    let results = sim.run(initial, 0.002, 0.5);
    assert!(!results.is_empty(), "Simulation produced no results");
    assert!(results.last().unwrap().time >= 0.49);
}

#[test]
fn test_transient_sim_angle_bounded() {
    let gen = ClassicalGen::thermal_unit();
    let sim = TransientSim::smib(gen, 1.0, 0.5);
    let initial = vec![GenState::new(0.4)];
    let results = sim.run(initial, 0.002, 1.0);
    // Rotor angle should stay within ±2π (not go to infinity)
    let max_delta: f64 = results
        .iter()
        .map(|r| r.gen_states[0].delta.abs())
        .fold(0.0_f64, f64::max);
    assert!(
        max_delta < 2.0 * PI,
        "Rotor angle diverged: {max_delta:.4} rad"
    );
}
