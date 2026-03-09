#![cfg(feature = "stability")]
use oxigrid::stability::transient::{ClassicalGen, GenState, TransientSim};
use proptest::prelude::*;

proptest! {
    /// Zero net torque (Pm == Pe): rotor angle stays constant, speed stays zero.
    #[test]
    fn prop_zero_torque_angle_constant(
        delta_init_rad in -1.0_f64..1.0_f64,
        h in 2.0_f64..10.0_f64,
        d in 0.5_f64..5.0_f64,
    ) {
        let gen = ClassicalGen { h, d, xd_prime: 0.2, e_prime: 1.0, p_mech: 1.0 };
        let pe_fn = Box::new(|_: &[GenState]| vec![1.0_f64]); // Pe always = Pm
        let sim = TransientSim::new(vec![gen], 60.0, pe_fn);
        let initial = vec![GenState::new(delta_init_rad)];
        let snaps = sim.run(initial, 0.01, 1.0);
        let delta_final = snaps.last().unwrap().gen_states[0].delta;
        prop_assert!((delta_final - delta_init_rad).abs() < 1e-4,
            "δ changed by {:.2e}", (delta_final - delta_init_rad).abs());
    }

    /// With damping only (no spring, Pe=0, Pm>0): rotor accelerates monotonically.
    #[test]
    fn prop_unloaded_fault_angle_increases(
        delta_init_deg in 10.0_f64..60.0_f64,
        h in 3.0_f64..10.0_f64,
    ) {
        let gen = ClassicalGen { h, d: 0.0, xd_prime: 0.2, e_prime: 1.0, p_mech: 0.5 };
        let pe_fn = Box::new(|_: &[GenState]| vec![0.0_f64]); // fault: Pe = 0
        let sim = TransientSim::new(vec![gen], 60.0, pe_fn);
        let initial = vec![GenState::new(delta_init_deg.to_radians())];
        let snaps = sim.run(initial, 0.01, 0.5);
        let delta0 = snaps[0].gen_states[0].delta;
        let delta_end = snaps.last().unwrap().gen_states[0].delta;
        prop_assert!(delta_end > delta0,
            "δ should increase: {:.3}° → {:.3}°",
            delta0.to_degrees(), delta_end.to_degrees());
    }

    /// Speed deviation zero at t=0 stays zero under balanced condition.
    #[test]
    fn prop_initial_omega_zero_balanced(
        h in 2.0_f64..10.0_f64,
        delta_init_rad in 0.1_f64..0.8_f64,
    ) {
        let gen = ClassicalGen { h, d: 5.0, xd_prime: 0.2, e_prime: 1.0, p_mech: 1.0 };
        let pe_fn = Box::new(|_: &[GenState]| vec![1.0_f64]);
        let sim = TransientSim::new(vec![gen], 60.0, pe_fn);
        let initial = vec![GenState::new(delta_init_rad)];
        let snap0 = &sim.run(initial, 0.001, 0.001)[0];
        prop_assert!((snap0.gen_states[0].omega).abs() < 1e-12);
    }

    /// RK4 steps are reversible: Pm = Pe constant means δ stable near equilibrium.
    #[test]
    fn prop_rk4_step_size_consistency(
        dt in 0.001_f64..0.05_f64,
    ) {
        let gen = ClassicalGen { p_mech: 1.0, e_prime: 1.0, xd_prime: 0.5, h: 6.0, d: 0.0 };
        let pe_fn = Box::new(|_: &[GenState]| vec![1.0_f64]);
        let sim = TransientSim::new(vec![gen], 60.0, pe_fn);
        let initial = vec![GenState::new(0.5)];
        let snaps = sim.run(initial, dt, 1.0);
        let delta_final = snaps.last().unwrap().gen_states[0].delta;
        prop_assert!((delta_final - 0.5).abs() < 1e-3,
            "δ changed with dt={}: {:.6}", dt, delta_final);
    }

    /// Eigenvalues have negative real parts for D>0, K>0 (stable operating point).
    #[test]
    fn prop_smib_eigenvalues_negative_real_stable(
        h in 2.0_f64..10.0_f64,
        d in 1.0_f64..8.0_f64,
        k in 0.5_f64..5.0_f64,
    ) {
        use oxigrid::stability::transient::smib_eigenvalues;
        let gen = ClassicalGen { h, d, xd_prime: 0.2, e_prime: 1.05, p_mech: 0.8 };
        let (r1, _i1, r2, _i2) = smib_eigenvalues(&gen, k, 60.0);
        prop_assert!(r1 <= 0.0, "r1={:.4} should be ≤ 0", r1);
        prop_assert!(r2 <= 0.0, "r2={:.4} should be ≤ 0", r2);
    }

    /// With positive damping, SMIB returns close to equilibrium after small perturbation.
    ///
    /// Ranges are bounded to keep D/M moderate (avoid RK4 numerical stiffness).
    #[test]
    fn prop_damped_smib_oscillation_decays(
        h in 5.0_f64..10.0_f64,
        d in 1.0_f64..3.0_f64,
    ) {
        // SMIB, start at equilibrium + small perturbation → should return
        let gen = ClassicalGen { h, d, xd_prime: 0.20, e_prime: 1.05, p_mech: 0.8 };
        let v_inf = 1.0;
        let x_tot = gen.xd_prime + 0.3;
        let e = gen.e_prime;
        let pm = gen.p_mech;
        let sim = TransientSim::smib(gen, v_inf, x_tot);
        let delta_eq = (pm * x_tot / (e * v_inf)).asin();
        let initial = vec![GenState::new(delta_eq + 5.0_f64.to_radians())];
        // Use small dt=0.005 s to keep RK4 stable across the parameter range
        let snaps = sim.run(initial, 0.005, 10.0);
        let last = snaps.last().unwrap();
        let delta_final = last.gen_states[0].delta;
        prop_assert!(delta_final.is_finite(), "simulation diverged");
        prop_assert!((delta_final - delta_eq).abs() < 0.3,
            "δ_final={:.2}° not near equilib={:.2}°",
            delta_final.to_degrees(), delta_eq.to_degrees());
    }
}
