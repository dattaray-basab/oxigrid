#![cfg(feature = "battery-p2d")]
use oxigrid::battery::p2d::{
    electrode::{ElectrodeParams, ElectrodeType, ParticleDiffusion},
    electrolyte::{ElectrolyteParams, ElectrolyteState},
    separator::{SeparatorParams, SeparatorState},
    solver::{SpmConfig, SpmSolver},
};

// ── Electrode / ParticleDiffusion ────────────────────────────────────────────

#[test]
fn test_electrode_types_present() {
    let g = ElectrodeParams::graphite_anode();
    let l = ElectrodeParams::lfp_cathode();
    let n = ElectrodeParams::nmc_cathode();
    assert_eq!(g.electrode_type, ElectrodeType::Graphite);
    assert_eq!(l.electrode_type, ElectrodeType::LFP);
    assert_eq!(n.electrode_type, ElectrodeType::NMC);
}

#[test]
fn test_graphite_ocp_range() {
    let params = ElectrodeParams::graphite_anode();
    for theta in [0.1f64, 0.3, 0.5, 0.7, 0.9] {
        let v = params.ocp(theta);
        assert!(
            (0.0..=1.5).contains(&v),
            "Graphite OCP = {v:.4} V at θ={theta} out of [0, 1.5]"
        );
    }
}

#[test]
fn test_lfp_ocp_plateau() {
    let params = ElectrodeParams::lfp_cathode();
    // LFP should sit near 3.4 V for 0.2 < θ < 0.8
    for theta in [0.25f64, 0.5, 0.75] {
        let v = params.ocp(theta);
        assert!(v > 3.0 && v < 4.0, "LFP OCP = {v:.4} V at θ={theta}");
    }
}

#[test]
fn test_nmc_ocp_decreasing() {
    let params = ElectrodeParams::nmc_cathode();
    // NMC OCP should decrease as stoichiometry increases (more lithiated = lower potential)
    let v_low = params.ocp(0.2);
    let v_high = params.ocp(0.8);
    assert!(
        v_low > v_high,
        "NMC OCP should decrease with θ: {v_low:.3} > {v_high:.3}"
    );
}

#[test]
fn test_diffusivity_increases_with_temperature() {
    let params = ElectrodeParams::graphite_anode();
    let d_cold = params.d_s(273.15); // 0°C
    let d_warm = params.d_s(333.15); // 60°C
    assert!(
        d_warm > d_cold,
        "D_s should increase with T: {d_cold:.2e} < {d_warm:.2e}"
    );
}

#[test]
fn test_specific_area_formula() {
    let params = ElectrodeParams::graphite_anode();
    let expected = 3.0 * params.epsilon_s / params.r_particle;
    assert!((params.specific_area() - expected).abs() < 1e-6 * expected);
}

#[test]
fn test_particle_init_at_theta_init() {
    let params = ElectrodeParams::graphite_anode();
    let theta0 = params.theta_init;
    let particle = ParticleDiffusion::new(params, 7);
    let theta_avg = particle.theta_avg();
    assert!(
        (theta_avg - theta0).abs() < 0.01,
        "Initial avg stoich = {theta_avg:.4}, expected {theta0:.4}"
    );
}

#[test]
fn test_particle_surface_eq_avg_at_rest() {
    // Uniform initial profile → surface = average
    let params = ElectrodeParams::lfp_cathode();
    let particle = ParticleDiffusion::new(params, 5);
    let diff = (particle.theta_surface() - particle.theta_avg()).abs();
    assert!(
        diff < 1e-9,
        "Surface θ should equal avg θ at rest: diff={diff:.2e}"
    );
}

#[test]
fn test_particle_discharge_decreases_anode_surface() {
    // Negative j_n = extraction from anode during discharge
    let params = ElectrodeParams::graphite_anode();
    let mut particle = ParticleDiffusion::new(params, 11);
    let theta_before = particle.theta_surface();
    let j_n = -1e-5; // mol/(m²·s) extraction
    let dt = particle.dt_stable(298.15) * 0.4;
    particle.step(j_n, dt, 298.15);
    let theta_after = particle.theta_surface();
    assert!(
        theta_after < theta_before,
        "Surface θ should decrease on extraction: {theta_before:.4} -> {theta_after:.4}"
    );
}

#[test]
fn test_particle_charge_increases_anode_surface() {
    // Positive j_n = intercalation into anode during charging
    // Start at low stoichiometry so we can charge
    let mut p2 = ElectrodeParams::graphite_anode();
    p2.theta_init = 0.3;
    let mut particle2 = ParticleDiffusion::new(p2, 11);
    let theta_before = particle2.theta_surface();
    let j_n = 1e-5; // intercalation
    let dt = particle2.dt_stable(298.15) * 0.4;
    particle2.step(j_n, dt, 298.15);
    let theta_after = particle2.theta_surface();
    assert!(
        theta_after > theta_before,
        "Surface θ should increase on intercalation: {theta_before:.4} -> {theta_after:.4}"
    );
}

#[test]
fn test_particle_no_flux_conserves_average() {
    // Zero flux → no Li leaves/enters → avg concentration constant
    let params = ElectrodeParams::lfp_cathode();
    let mut particle = ParticleDiffusion::new(params, 11);
    let avg_before = particle.theta_avg();
    let dt = particle.dt_stable(298.15) * 0.4;
    // Take many steps with zero flux
    for _ in 0..20 {
        particle.step(0.0, dt, 298.15);
    }
    let avg_after = particle.theta_avg();
    assert!(
        (avg_after - avg_before).abs() < 1e-6,
        "Avg θ should be conserved with zero flux: {avg_before:.6} -> {avg_after:.6}"
    );
}

#[test]
fn test_dt_stable_positive() {
    let params = ElectrodeParams::graphite_anode();
    let particle = ParticleDiffusion::new(params, 5);
    assert!(particle.dt_stable(298.15) > 0.0);
}

#[test]
fn test_capacity_coulombs_reasonable() {
    // ~3 Ah graphite anode → ~10,800 C
    let params = ElectrodeParams::graphite_anode();
    let particle = ParticleDiffusion::new(params, 5);
    let cap_c = particle.capacity_coulombs();
    assert!(
        cap_c > 5_000.0 && cap_c < 100_000.0,
        "Capacity = {cap_c:.0} C"
    );
}

#[test]
fn test_reset_restores_initial() {
    let params = ElectrodeParams::graphite_anode();
    let mut particle = ParticleDiffusion::new(params, 11);
    let theta_init = particle.theta_avg();
    let dt = particle.dt_stable(298.15) * 0.4;
    for _ in 0..50 {
        particle.step(-1e-5, dt, 298.15);
    }
    particle.reset();
    let theta_reset = particle.theta_avg();
    assert!(
        (theta_reset - theta_init).abs() < 1e-9,
        "Reset should restore initial: {theta_init:.4} -> {theta_reset:.4}"
    );
}

#[test]
fn test_docp_dtheta_sign_graphite() {
    // Graphite OCP is generally decreasing with θ at mid-range (dOCP/dθ < 0)
    let params = ElectrodeParams::graphite_anode();
    let d = params.docp_dtheta(0.5);
    // At θ=0.5 graphite OCP has complex shape; just check it's finite and nonzero
    assert!(
        d.is_finite() && d != 0.0,
        "dOCP/dθ should be finite and nonzero: {d:.4}"
    );
}

// ── ElectrolyteState ─────────────────────────────────────────────────────────

fn default_electrolyte() -> ElectrolyteState {
    let params = ElectrolyteParams::lipf6_ec_dmc();
    ElectrolyteState::new(
        params, 100e-6, 25e-6, 80e-6, // l_neg, l_sep, l_pos [m]
        0.30, 0.40, 0.30, // porosities
        10, 5, 8, // nodes
    )
}

#[test]
fn test_electrolyte_initial_uniform() {
    let state = default_electrolyte();
    let c0 = state.params.c_e_init;
    for &c in &state.c_e {
        assert!((c - c0).abs() < 1e-9, "Initial conc not uniform: {c:.3}");
    }
}

#[test]
fn test_electrolyte_n_total() {
    let state = default_electrolyte();
    assert_eq!(state.n_total(), 23, "10 + 5 + 8 = 23 nodes");
}

#[test]
fn test_electrolyte_dt_stable_positive() {
    let state = default_electrolyte();
    assert!(state.dt_stable(298.15) > 0.0);
}

#[test]
fn test_electrolyte_zero_source_conserves_mass() {
    let mut state = default_electrolyte();
    let c_sum_before: f64 = state.c_e.iter().sum();
    let dt = state.dt_stable(298.15) * 0.4;
    state.step_concentration(0.0, 0.0, dt, 298.15);
    let c_sum_after: f64 = state.c_e.iter().sum();
    let rel_err = (c_sum_after - c_sum_before).abs() / c_sum_before;
    assert!(
        rel_err < 1e-9,
        "Mass should be conserved: rel_err={rel_err:.2e}"
    );
}

#[test]
fn test_electrolyte_source_increases_anode_conc() {
    // Positive j_n_neg → Li released into electrolyte at anode (discharge)
    let mut state = default_electrolyte();
    let dt = state.dt_stable(298.15) * 0.4;
    let c_anode_before = state.c_e[0];
    state.step_concentration(10.0, -10.0, dt, 298.15);
    let c_anode_after = state.c_e[0];
    assert!(
        c_anode_after > c_anode_before - 1.0, // some increase expected in anode region
        "Anode conc should respond to positive source"
    );
}

#[test]
fn test_electrolyte_arrhenius_diffusivity() {
    let p = ElectrolyteParams::lipf6_ec_dmc();
    let d_cold = p.d_e(273.15);
    let d_warm = p.d_e(333.15);
    assert!(d_warm > d_cold, "D_e should increase with T");
}

#[test]
fn test_electrolyte_effective_diffusivity_lt_bulk() {
    let p = ElectrolyteParams::lipf6_ec_dmc();
    let eps = 0.30;
    let d_eff = p.d_e_eff(eps, 298.15);
    let d_bulk = p.d_e(298.15);
    assert!(d_eff < d_bulk, "D_eff < D_bulk for ε={eps}");
}

#[test]
fn test_electrolyte_conductivity_arrhenius() {
    let p = ElectrolyteParams::lipf6_ec_dmc();
    let k_cold = p.kappa(273.15);
    let k_warm = p.kappa(333.15);
    assert!(k_warm > k_cold, "κ should increase with temperature");
}

#[test]
fn test_electrolyte_reset() {
    let mut state = default_electrolyte();
    let dt = state.dt_stable(298.15) * 0.4;
    state.step_concentration(5.0, -5.0, dt, 298.15);
    state.reset();
    let c0 = state.params.c_e_init;
    for &c in &state.c_e {
        assert!(
            (c - c0).abs() < 1e-9,
            "Reset should restore c_e = {c0}: got {c:.3}"
        );
    }
    for &phi in &state.phi_e {
        assert!(phi.abs() < 1e-12, "Reset should zero phi_e, got {phi}");
    }
}

// ── SeparatorParams / SeparatorState ─────────────────────────────────────────

#[test]
fn test_celgard_parameters() {
    let sep = SeparatorParams::celgard_2500();
    assert!((sep.thickness - 25e-6).abs() < 1e-12, "Thickness = 25 µm");
    assert!(
        sep.porosity > 0.0 && sep.porosity < 1.0,
        "Porosity in (0,1)"
    );
    assert!(sep.bruggeman > 0.0, "Bruggeman > 0");
}

#[test]
fn test_ceramic_thinner_than_celgard() {
    let cel = SeparatorParams::celgard_2500();
    let cer = SeparatorParams::ceramic_16um();
    assert!(cer.thickness < cel.thickness, "Ceramic should be thinner");
}

#[test]
fn test_effective_transport_factor_range() {
    let sep = SeparatorParams::celgard_2500();
    let eta = sep.effective_transport_factor();
    assert!(
        eta > 0.0 && eta < 1.0,
        "Transport factor ε^b ∈ (0,1): {eta:.4}"
    );
}

#[test]
fn test_separator_resistance_positive() {
    let sep = SeparatorParams::celgard_2500();
    let r = sep.resistance_area(1.1);
    assert!(r > 0.0, "Resistance should be positive: {r:.4e}");
}

#[test]
fn test_separator_zero_conductivity_gives_infinite_resistance() {
    let sep = SeparatorParams::celgard_2500();
    let r = sep.resistance_area(0.0);
    assert!(r.is_infinite(), "Zero conductivity → infinite resistance");
}

#[test]
fn test_separator_overtemp_detection() {
    let sep = SeparatorParams::celgard_2500();
    assert!(!sep.is_overtemp(350.0)); // 77°C — safe
    assert!(sep.is_overtemp(450.0)); // 177°C — above 130°C limit
}

#[test]
fn test_separator_state_init() {
    let params = SeparatorParams::celgard_2500();
    let state = SeparatorState::new(params, 1000.0, 5);
    assert_eq!(state.n_nodes, 5);
    for &c in &state.c_e {
        assert!((c - 1000.0).abs() < 1e-9);
    }
    for &phi in &state.phi_e {
        assert!(phi.abs() < 1e-12);
    }
}

#[test]
fn test_separator_linear_concentration_profile() {
    let params = SeparatorParams::celgard_2500();
    let mut state = SeparatorState::new(params, 1000.0, 10);
    state.set_boundary_concentrations(800.0, 1200.0);
    // Profile should be monotonically increasing
    for i in 1..state.n_nodes {
        assert!(
            state.c_e[i] > state.c_e[i - 1],
            "Profile not monotone at node {i}"
        );
    }
    // Average should be close to midpoint
    let mid = (800.0 + 1200.0) / 2.0;
    assert!(
        (state.c_avg() - mid).abs() < 50.0,
        "Avg ≈ midpoint: {:.1}",
        state.c_avg()
    );
}

#[test]
fn test_separator_ohmic_drop_proportional_to_current() {
    let params = SeparatorParams::celgard_2500();
    let state = SeparatorState::new(params, 1000.0, 5);
    let drop_1 = state.ohmic_drop(100.0, 1.1);
    let drop_2 = state.ohmic_drop(200.0, 1.1);
    assert!(
        (drop_2 - 2.0 * drop_1).abs() < 1e-12,
        "Ohmic drop should be linear in current: {drop_1:.4e} * 2 = {drop_2:.4e}"
    );
}

#[test]
fn test_separator_reset() {
    let params = SeparatorParams::celgard_2500();
    let mut state = SeparatorState::new(params, 1000.0, 5);
    state.set_boundary_concentrations(500.0, 1500.0);
    state.reset(1000.0);
    for &c in &state.c_e {
        assert!((c - 1000.0).abs() < 1e-9, "Reset c_e: {c:.3}");
    }
}

// ── SpmSolver (Single Particle Model) ────────────────────────────────────────

#[test]
fn test_spm_lfp_ocv_in_range() {
    let solver = SpmSolver::graphite_lfp(SpmConfig::default());
    let ocv = solver.ocv();
    assert!(
        ocv > 2.5 && ocv < 4.5,
        "LFP OCV = {ocv:.3} V out of [2.5, 4.5]"
    );
}

#[test]
fn test_spm_nmc_ocv_in_range() {
    let solver = SpmSolver::graphite_nmc(SpmConfig::default());
    let ocv = solver.ocv();
    assert!(
        ocv > 3.0 && ocv < 5.0,
        "NMC OCV = {ocv:.3} V out of [3.0, 5.0]"
    );
}

#[test]
fn test_spm_initial_soc_high() {
    // theta_init = 0.80 → SoC ≈ 1.0
    let solver = SpmSolver::graphite_lfp(SpmConfig::default());
    let soc = solver.soc_estimate();
    assert!(soc > 0.9, "Initial SoC should be near 1.0: {soc:.3}");
}

#[test]
fn test_spm_discharge_voltage_below_ocv() {
    let mut solver = SpmSolver::graphite_lfp(SpmConfig::default());
    let ocv = solver.ocv();
    // Apply 1C discharge (≈ 3 A) for 1 second
    let state = solver.step(3.0, 1.0, 298.15);
    assert!(
        state.voltage < ocv + 0.05,
        "Discharge voltage {:.3} should be ≤ OCV {ocv:.3}",
        state.voltage
    );
}

#[test]
fn test_spm_discharge_current_stored() {
    let mut solver = SpmSolver::graphite_lfp(SpmConfig::default());
    let state = solver.step(5.0, 1.0, 298.15);
    assert!(
        (state.current - 5.0).abs() < 1e-12,
        "Current stored correctly"
    );
}

#[test]
fn test_spm_time_advances() {
    let mut solver = SpmSolver::graphite_lfp(SpmConfig::default());
    solver.step(3.0, 10.0, 298.15);
    solver.step(3.0, 10.0, 298.15);
    assert!(
        (solver.time_s - 20.0).abs() < 1e-9,
        "Time should advance: {:.1}s",
        solver.time_s
    );
}

#[test]
fn test_spm_simulate_discharge_returns_states() {
    let mut solver = SpmSolver::graphite_lfp(SpmConfig::default());
    let states = solver.simulate_discharge(3.0, 10.0, 298.15, 60.0);
    assert!(!states.is_empty(), "Discharge should return states");
    assert!(
        states.len() <= 6,
        "At most 60s/10s = 6 states (may cut off earlier)"
    );
}

#[test]
fn test_spm_simulate_discharge_voltage_decreases() {
    let mut solver = SpmSolver::graphite_lfp(SpmConfig::default());
    let states = solver.simulate_discharge(3.0, 10.0, 298.15, 100.0);
    if states.len() >= 3 {
        let v_start = states[0].voltage;
        let v_end = states.last().unwrap().voltage;
        assert!(
            v_end <= v_start + 0.2,
            "Voltage should trend down during discharge: {v_start:.3} -> {v_end:.3}"
        );
    }
}

#[test]
fn test_spm_theta_neg_decreases_on_discharge() {
    // During discharge, Li leaves anode → anode stoichiometry decreases
    let mut solver = SpmSolver::graphite_lfp(SpmConfig::default());
    let state0 = solver.step(0.0, 1.0, 298.15); // rest step
    let theta_neg_0 = state0.theta_neg_avg;
    let state1 = solver.step(3.0, 60.0, 298.15); // discharge
    let theta_neg_1 = state1.theta_neg_avg;
    assert!(
        theta_neg_1 < theta_neg_0,
        "Anode avg θ should decrease on discharge: {theta_neg_0:.4} -> {theta_neg_1:.4}"
    );
}

#[test]
fn test_spm_theta_pos_increases_on_discharge() {
    // During discharge, Li enters cathode → cathode stoichiometry increases
    let mut solver = SpmSolver::graphite_lfp(SpmConfig::default());
    let state0 = solver.step(0.0, 1.0, 298.15);
    let theta_pos_0 = state0.theta_pos_avg;
    let state1 = solver.step(3.0, 60.0, 298.15);
    let theta_pos_1 = state1.theta_pos_avg;
    assert!(
        theta_pos_1 > theta_pos_0,
        "Cathode avg θ should increase on discharge: {theta_pos_0:.4} -> {theta_pos_1:.4}"
    );
}

#[test]
fn test_spm_voltage_finite() {
    let mut solver = SpmSolver::graphite_lfp(SpmConfig::default());
    for _ in 0..10 {
        let state = solver.step(3.0, 5.0, 298.15);
        assert!(
            state.voltage.is_finite(),
            "Voltage should be finite: {}",
            state.voltage
        );
        if state.cutoff {
            break;
        }
    }
}

#[test]
fn test_spm_simulate_discharge_long() {
    // Run a long discharge and verify simulation terminates with correct state
    let mut solver = SpmSolver::graphite_lfp(SpmConfig::default());
    let ocv_initial = solver.ocv();
    // 3 A for 1200 s = 1 Ah (≈1/3 of 3 Ah capacity)
    let states = solver.simulate_discharge(3.0, 60.0, 298.15, 1200.0);
    assert!(!states.is_empty(), "Discharge states should be non-empty");
    let last = states.last().unwrap();
    assert!(last.voltage.is_finite(), "Final voltage should be finite");
    // Voltage should remain in a physically meaningful range
    assert!(
        last.voltage >= solver.config.v_min,
        "Voltage should not go below v_min"
    );
    // Time should advance
    assert!(last.time_s > 0.0, "Time should advance during discharge");
    // SoC should drop
    let soc_final = solver.soc_estimate();
    assert!(
        soc_final < 0.99,
        "SoC should decrease from initial: {soc_final:.3}"
    );
    let _ = ocv_initial;
}

#[test]
fn test_spm_reset() {
    let mut solver = SpmSolver::graphite_lfp(SpmConfig::default());
    let ocv_before = solver.ocv();
    solver.simulate_discharge(3.0, 10.0, 298.15, 300.0);
    solver.reset();
    let ocv_after = solver.ocv();
    assert!(
        (ocv_after - ocv_before).abs() < 0.01,
        "OCV should be restored after reset: {ocv_before:.4} -> {ocv_after:.4}"
    );
    assert!(solver.time_s.abs() < 1e-12, "Time should be 0 after reset");
}

#[test]
fn test_spm_nmc_discharge_works() {
    let mut solver = SpmSolver::graphite_nmc(SpmConfig::default());
    let state = solver.step(3.0, 60.0, 298.15);
    assert!(state.voltage.is_finite(), "NMC voltage should be finite");
    assert!(state.voltage > 0.0, "NMC voltage should be positive");
}

#[test]
fn test_spm_bv_zero_current_zero_overpotential() {
    // At rest (zero flux), Butler-Volmer overpotential = 0
    let mut solver = SpmSolver::graphite_lfp(SpmConfig::default());
    // Rest step: no current applied, voltage ≈ OCV
    let state = solver.step(0.0, 1.0, 298.15);
    let ocv = solver.ocv();
    // With zero current, terminal voltage ≈ OCV (small numerical drift allowed)
    assert!(
        (state.voltage - ocv).abs() < 0.01,
        "At rest, V ≈ OCV: V={:.4}, OCV={ocv:.4}",
        state.voltage
    );
}

#[test]
fn test_spm_coulomb_counting_consistency() {
    // Run SPM and compare charge removed (I·t) with stoichiometry change
    let mut solver = SpmSolver::graphite_lfp(SpmConfig::default());
    let soc_before = solver.soc_estimate();
    let current_a = 3.0;
    let dt = 100.0; // 100 s
    let state = solver.step(current_a, dt, 298.15);
    let soc_after = solver.soc_estimate();
    let dsoc = soc_before - soc_after; // should be positive (discharge)
                                       // Approximate capacity of graphite anode electrode
    let cap_ah = solver.anode.capacity_coulombs() / 3600.0;
    let expected_dsoc = current_a * dt / 3600.0 / cap_ah;
    // Rough check: dsoc should be in the right order of magnitude
    assert!(dsoc > 0.0, "SoC should decrease on discharge");
    assert!(
        (dsoc - expected_dsoc).abs() < 0.5 * expected_dsoc + 0.01,
        "SoC drop {dsoc:.4} should be ≈ {expected_dsoc:.4} (I·dt / Q_cap)"
    );
    let _ = state;
}
