//! Tests for the `Thermal1DAxial` 1D axial finite-difference thermal model.
//!
//! Feature-gated behind `battery`.

#![cfg(feature = "battery")]

use oxigrid::battery::thermal::{Axial1DConfig, AxialBoundary, Thermal1DAxial};

/// Build a default valid Thermal1DAxial with convective BCs on both ends.
fn make_default(n_nodes: usize) -> Thermal1DAxial {
    Thermal1DAxial::new(Axial1DConfig {
        n_nodes,
        length_m: 0.065,
        cross_area_m2: 4.91e-5,
        density: 2500.0,
        cp: 1000.0,
        k: 1.5,
        t_ambient_k: 298.15,
        bc_left: AxialBoundary::Convective { h_conv: 10.0 },
        bc_right: AxialBoundary::Convective { h_conv: 10.0 },
    })
    .expect("valid construction should not fail")
}

/// Uniform heat generation vector for all nodes.
fn uniform_q(model: &Thermal1DAxial, watts_per_node: f64) -> Vec<f64> {
    vec![watts_per_node; model.n_nodes]
}

// ─── Construction tests ───────────────────────────────────────────────────────

#[test]
fn test_model_construction_valid() {
    let model = make_default(10);
    assert_eq!(model.n_nodes, 10);
    assert_eq!(model.temperatures.len(), 10);
    // All temperatures should be initialised at ambient.
    for &t in &model.temperatures {
        assert!((t - 298.15).abs() < 1e-9);
    }
}

#[test]
fn test_model_construction_invalid_n_nodes() {
    // Single node is not enough for a spatial gradient.
    let result = Thermal1DAxial::new(Axial1DConfig {
        n_nodes: 1,
        length_m: 0.065,
        cross_area_m2: 4.91e-5,
        density: 2500.0,
        cp: 1000.0,
        k: 1.5,
        t_ambient_k: 298.15,
        bc_left: AxialBoundary::Convective { h_conv: 10.0 },
        bc_right: AxialBoundary::Convective { h_conv: 10.0 },
    });
    assert!(result.is_err(), "n_nodes=1 must return Err");
}

#[test]
fn test_model_construction_invalid_k() {
    // k ≤ 0 is physically invalid.
    let result = Thermal1DAxial::new(Axial1DConfig {
        n_nodes: 10,
        length_m: 0.065,
        cross_area_m2: 4.91e-5,
        density: 2500.0,
        cp: 1000.0,
        k: -1.0, // bad k
        t_ambient_k: 298.15,
        bc_left: AxialBoundary::Convective { h_conv: 10.0 },
        bc_right: AxialBoundary::Convective { h_conv: 10.0 },
    });
    assert!(result.is_err(), "k<=0 must return Err");

    let result_zero = Thermal1DAxial::new(Axial1DConfig {
        n_nodes: 10,
        length_m: 0.065,
        cross_area_m2: 4.91e-5,
        density: 2500.0,
        cp: 1000.0,
        k: 0.0, // zero k
        t_ambient_k: 298.15,
        bc_left: AxialBoundary::Convective { h_conv: 10.0 },
        bc_right: AxialBoundary::Convective { h_conv: 10.0 },
    });
    assert!(result_zero.is_err(), "k=0 must return Err");
}

// ─── Stability / correctness tests ───────────────────────────────────────────

#[test]
fn test_cfl_no_blowup_with_large_dt() {
    // A large external dt (20× CFL) must be handled by adaptive sub-stepping
    // without producing non-finite or astronomically large temperatures.
    let mut model = make_default(10);
    let dt_cfl = model.dt_cfl();
    let q = uniform_q(&model, 0.01); // small heat source

    model.step(dt_cfl * 20.0, &q).expect("step should succeed");

    for &t in &model.temperatures {
        assert!(
            t.is_finite(),
            "temperature must be finite after large dt step"
        );
        assert!(t < 1e6, "temperature must be physically reasonable");
    }
}

#[test]
fn test_dirichlet_left_bc() {
    // With Dirichlet BC at left end (273 K) and zero heat generation,
    // the implicit solver enforces T[0] = T_fixed exactly (identity row).
    // After many steps the left node should converge to 273 K.
    let t_fixed = 273.0_f64;
    let mut model = Thermal1DAxial::new(Axial1DConfig {
        n_nodes: 10,
        length_m: 0.065,
        cross_area_m2: 4.91e-5,
        density: 2500.0,
        cp: 1000.0,
        k: 1.5,
        t_ambient_k: 298.15,
        bc_left: AxialBoundary::Dirichlet { t_fixed_k: t_fixed },
        bc_right: AxialBoundary::Convective { h_conv: 10.0 },
    })
    .expect("valid construction");

    let q = vec![0.0_f64; model.n_nodes]; // no heat generation

    // Run 200 implicit steps of 1 s each — well past steady state.
    for _ in 0..200 {
        model
            .step_implicit(1.0, &q)
            .expect("step_implicit should succeed");
    }

    let t0 = model.temperature_at(0).expect("node 0 must exist");

    assert!(
        (t0 - t_fixed).abs() < 2.0,
        "left node {t0:.3} K should be near {t_fixed} K (Dirichlet, implicit solver)"
    );
}

#[test]
fn test_implicit_step_no_blowup() {
    // The implicit solver must produce finite temperatures even with dt >> dt_cfl.
    let mut model = make_default(10);
    let q = uniform_q(&model, 0.5);

    model
        .step_implicit(1.0, &q)
        .expect("step_implicit should succeed with dt=1 s");

    for &t in &model.temperatures {
        assert!(t.is_finite(), "implicit step must not produce NaN/Inf");
        assert!(t < 1e6, "implicit step must remain physically bounded");
    }
}

#[test]
fn test_max_min_temperature() {
    // After some heat generation, max should be ≥ min, and at least some
    // node should be warmer than ambient.
    let mut model = make_default(10);
    let init_max = model.max_temperature();
    let q = uniform_q(&model, 1.0); // moderate heat per node

    for _ in 0..50 {
        model.step(model.dt_cfl(), &q).expect("step should succeed");
    }

    let max_t = model.max_temperature();
    let min_t = model.min_temperature();

    assert!(
        max_t >= min_t,
        "max temperature must be >= min temperature (got max={max_t:.2}, min={min_t:.2})"
    );
    assert!(
        max_t > init_max,
        "max temperature should rise due to heating (init={init_max:.2}, after={max_t:.2})"
    );
}
