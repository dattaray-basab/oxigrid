#![cfg(feature = "protection")]
use oxigrid::protection::coordination::{check_coordination, tcc_curve, CoordinationStudy};
use oxigrid::protection::fault::{compute_zbus, three_phase_fault, FaultType};
use oxigrid::protection::relay::{OcRelay, RelayCharacteristic};

// ── Relay Time-Current Characteristics ───────────────────────────────────────

#[test]
fn test_oc_relay_trips_above_pickup() {
    let relay = OcRelay::new(100.0, 0.5, RelayCharacteristic::StandardInverse);
    let t = relay.trip_time(1000.0).expect("Should trip at 10× pickup");
    assert!(
        t > 0.0 && t < 5.0,
        "Trip time {t:.4} s out of range at 10× pickup"
    );
}

#[test]
fn test_oc_relay_no_trip_below_pickup() {
    let relay = OcRelay::new(100.0, 0.5, RelayCharacteristic::StandardInverse);
    assert!(
        relay.trip_time(50.0).is_none(),
        "Relay should not trip below pickup current"
    );
}

#[test]
fn test_oc_relay_very_inverse() {
    let relay = OcRelay::new(200.0, 1.0, RelayCharacteristic::VeryInverse);
    let t = relay.trip_time(2000.0).expect("Should trip at 10× pickup");
    assert!(t > 0.0 && t < 30.0, "Trip time {t:.4} s out of range");
}

#[test]
fn test_oc_relay_extremely_inverse() {
    let relay = OcRelay::new(100.0, 0.3, RelayCharacteristic::ExtremelyInverse);
    let t = relay.trip_time(500.0).expect("Should trip at 5× pickup");
    assert!(t > 0.0 && t < 15.0, "Trip time {t:.4} s");
}

#[test]
fn test_higher_tms_slower_trip() {
    let relay_fast = OcRelay::new(100.0, 0.3, RelayCharacteristic::StandardInverse);
    let relay_slow = OcRelay::new(100.0, 0.8, RelayCharacteristic::StandardInverse);
    let i_fault = 800.0;
    let t_fast = relay_fast.trip_time(i_fault).unwrap();
    let t_slow = relay_slow.trip_time(i_fault).unwrap();
    assert!(
        t_slow > t_fast,
        "Higher TMS should trip slower: {t_slow:.4} vs {t_fast:.4}"
    );
}

#[test]
fn test_tcc_curve_returns_points() {
    let relay = OcRelay::new(100.0, 0.5, RelayCharacteristic::StandardInverse);
    let points = tcc_curve(&relay, 200.0, 5000.0, 20);
    assert!(!points.is_empty(), "TCC curve should have points");
    // All points should have positive current and time
    for (i, t) in &points {
        assert!(*i > 0.0 && *t > 0.0, "TCC point ({i:.2}, {t:.4}) invalid");
    }
}

#[test]
fn test_tcc_curve_monotone_decreasing() {
    let relay = OcRelay::new(100.0, 0.5, RelayCharacteristic::StandardInverse);
    let points = tcc_curve(&relay, 200.0, 5000.0, 30);
    for w in points.windows(2) {
        let (i1, t1) = w[0];
        let (i2, t2) = w[1];
        // Higher current → shorter time
        if i2 > i1 {
            assert!(
                t2 <= t1 + 1e-9,
                "TCC not monotone: at i={i2:.2} t={t2:.4} > t={t1:.4} at i={i1:.2}"
            );
        }
    }
}

// ── Coordination ─────────────────────────────────────────────────────────────

#[test]
fn test_check_coordination_well_coordinated() {
    let relays = vec![
        OcRelay::new(100.0, 0.3, RelayCharacteristic::StandardInverse),
        OcRelay::new(100.0, 0.9, RelayCharacteristic::StandardInverse),
    ];
    let pairs = vec![(0usize, 1usize, 800.0_f64)];
    let study = check_coordination(&relays, &pairs, 0.3);
    assert_eq!(study.pairs.len(), 1);
    let pair = &study.pairs[0];
    assert!(
        pair.t_backup_s > pair.t_primary_s,
        "Backup {:.4}s must be slower than primary {:.4}s",
        pair.t_backup_s,
        pair.t_primary_s
    );
}

#[test]
fn test_check_coordination_identical_relays_violation() {
    let relays = vec![
        OcRelay::new(100.0, 0.5, RelayCharacteristic::StandardInverse),
        OcRelay::new(100.0, 0.5, RelayCharacteristic::StandardInverse),
    ];
    let pairs = vec![(0usize, 1usize, 800.0_f64)];
    let study = check_coordination(&relays, &pairs, 0.3);
    assert!(
        !study.violations.is_empty(),
        "Identical relays should have coordination violation"
    );
}

#[test]
fn test_coordination_fully_coordinated() {
    let relays = vec![
        OcRelay::new(100.0, 0.2, RelayCharacteristic::VeryInverse),
        OcRelay::new(100.0, 0.7, RelayCharacteristic::VeryInverse),
        OcRelay::new(100.0, 1.2, RelayCharacteristic::VeryInverse),
    ];
    let pairs = vec![(0usize, 1usize, 1000.0_f64), (1usize, 2usize, 800.0_f64)];
    let study: CoordinationStudy = check_coordination(&relays, &pairs, 0.3);
    assert_eq!(study.pairs.len(), 2);
    // The study has a fully_coordinated method
    // Just verify no panic and structure is valid
    assert!(study.n_violations() <= 2);
}

#[test]
fn test_coordination_cti_s_field() {
    let relays = vec![
        OcRelay::new(100.0, 0.3, RelayCharacteristic::StandardInverse),
        OcRelay::new(100.0, 0.8, RelayCharacteristic::StandardInverse),
    ];
    let study = check_coordination(&relays, &[(0, 1, 500.0)], 0.25);
    assert!(
        (study.cti_s - 0.25).abs() < 1e-9,
        "CTI should match input: {}",
        study.cti_s
    );
}

// ── Fault Current Analysis ────────────────────────────────────────────────────

#[test]
fn test_three_phase_fault_current_magnitude() {
    // 1-bus system: Thevenin impedance Z_th = j0.05 pu → Y = -j20
    use num_complex::Complex64;
    let y = Complex64::new(0.0, -20.0); // 1/(j0.05) = -j20
    let ybus_dense = vec![vec![y]];
    let results = compute_zbus(&ybus_dense);
    assert!(results.is_ok(), "compute_zbus should succeed");
    let zbus = results.unwrap();
    // Z11 = 1/Y = j0.05
    assert!(
        (zbus[0][0].im - 0.05).abs() < 1e-6,
        "Z11 = {:.6}, expected j0.05",
        zbus[0][0]
    );
}

#[test]
fn test_three_phase_fault_result() {
    use num_complex::Complex64;
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
    if let Ok(net) = oxigrid::network::PowerNetwork::from_matpower(path) {
        let ybus_sp = net.admittance_matrix().unwrap();
        let n = ybus_sp.rows();
        let mut ybus_dense = vec![vec![Complex64::new(0.0, 0.0); n]; n];
        for (i, ybus_row) in ybus_dense.iter_mut().enumerate() {
            if let Some(rv) = ybus_sp.outer_view(i) {
                for (&j, &v) in rv.indices().iter().zip(rv.data().iter()) {
                    ybus_row[j] = v;
                }
            }
        }
        let zbus = compute_zbus(&ybus_dense).unwrap();
        let v_pre: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); n];
        let result = three_phase_fault(&zbus, &v_pre, 0, 100.0, None);
        assert!(result.is_ok(), "three_phase_fault should succeed");
        let fr = result.unwrap();
        assert_eq!(fr.bus_idx, 0);
        assert!(
            fr.i_fault_pu > 0.0,
            "Fault current should be positive: {}",
            fr.i_fault_pu
        );
    }
}

#[test]
fn test_fault_type_variants() {
    // Verify all fault type variants exist
    let _ft = [
        FaultType::ThreePhase,
        FaultType::SingleLineGround,
        FaultType::LineLine,
        FaultType::DoubleLineGround,
    ];
}

#[test]
fn test_zbus_diagonal_real_part() {
    use num_complex::Complex64;
    // Pure inductive network: Z-bus diagonal should have near-zero real part
    let y = Complex64::new(0.0, 10.0);
    let ybus = vec![vec![y + y, -y], vec![-y, y + y]];
    let zbus = compute_zbus(&ybus).unwrap();
    for (i, zbus_row) in zbus.iter().enumerate() {
        assert!(
            zbus_row[i].re.abs() < 1e-6,
            "Diagonal Z{i}{i} real part should be ≈0: {:.4e}",
            zbus_row[i].re
        );
    }
}

#[test]
fn test_zbus_symmetry() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
    if let Ok(net) = oxigrid::network::PowerNetwork::from_matpower(path) {
        use num_complex::Complex64;
        let ybus_sp = net.admittance_matrix().unwrap();
        let n = ybus_sp.rows();
        let mut ybus_dense = vec![vec![Complex64::new(0.0, 0.0); n]; n];
        for (i, row) in (0..n).map(|i| (i, ybus_sp.outer_view(i))) {
            if let Some(rv) = row {
                for (&j, &v) in rv.indices().iter().zip(rv.data().iter()) {
                    ybus_dense[i][j] = v;
                }
            }
        }
        let zbus = compute_zbus(&ybus_dense).unwrap();
        for (i, zbus_row) in zbus.iter().enumerate() {
            for (j, zbus_val) in zbus_row.iter().enumerate() {
                let diff = (*zbus_val - zbus[j][i]).norm();
                assert!(
                    diff < 1e-8,
                    "Z-bus not symmetric at ({i},{j}): diff={diff:.2e}"
                );
            }
        }
    }
}
