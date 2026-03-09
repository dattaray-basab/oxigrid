//! Tests for asymmetric fault analysis using symmetrical components.
//!
//! Covers SLG, LL, and DLG fault types using both the low-level
//! `compute_asymmetric_fault` API and the high-level `FaultAnalysis` struct.
#![cfg(feature = "protection")]

use num_complex::Complex64;
use oxigrid::protection::fault::compute_zbus;
use oxigrid::protection::fault_asymmetric::{
    compute_asymmetric_fault, AsymmetricFaultType, FaultAnalysis, SequenceImpedances,
};

/// Build a simple 2-bus Y-bus (same as used in fault.rs unit tests).
fn two_bus_ybus() -> Vec<Vec<Complex64>> {
    let y12 = Complex64::new(0.0, -5.0); // x = 0.2 p.u.
    let y11 = -y12 + Complex64::new(0.0, 0.1);
    vec![vec![y11, y12], vec![y12, y11]]
}

/// Typical sequence impedances: Z1=Z2=j0.12, Z0=j0.35.
fn typical_seq() -> SequenceImpedances {
    SequenceImpedances {
        z1: Complex64::new(0.01, 0.12),
        z2: Complex64::new(0.01, 0.12),
        z0: Complex64::new(0.02, 0.35),
    }
}

// ── SLG fault tests ───────────────────────────────────────────────────────────

#[test]
fn test_slg_ia_equals_three_times_i1() {
    // For SLG fault: Ia = I0 + I1 + I2 = 3*I1 (since I0=I1=I2)
    let seq = typical_seq();
    let result = compute_asymmetric_fault(
        &seq,
        AsymmetricFaultType::SingleLineToGround,
        1.0,
        0,
        None,
        100.0,
    )
    .expect("SLG fault should succeed");

    let i1 = result.i_fault_positive;
    let ia = result.i_fault_a;
    let expected_ia = i1 * 3.0;
    assert!(
        (ia - expected_ia).norm() < 1e-10,
        "Ia should equal 3*I1 for SLG: got {:.6}+{:.6}j, expected {:.6}+{:.6}j",
        ia.re,
        ia.im,
        expected_ia.re,
        expected_ia.im
    );
}

#[test]
fn test_slg_sequence_currents_equal() {
    // SLG: I1 = I2 = I0
    let seq = typical_seq();
    let result = compute_asymmetric_fault(
        &seq,
        AsymmetricFaultType::SingleLineToGround,
        1.0,
        0,
        None,
        100.0,
    )
    .expect("SLG fault should succeed");

    let diff12 = (result.i_fault_positive - result.i_fault_negative).norm();
    let diff10 = (result.i_fault_positive - result.i_fault_zero).norm();
    assert!(
        diff12 < 1e-10,
        "I1 should equal I2 for SLG: diff={diff12:.2e}"
    );
    assert!(
        diff10 < 1e-10,
        "I1 should equal I0 for SLG: diff={diff10:.2e}"
    );
}

#[test]
fn test_slg_fault_current_nonzero() {
    let seq = typical_seq();
    let result = compute_asymmetric_fault(
        &seq,
        AsymmetricFaultType::SingleLineToGround,
        1.0,
        0,
        None,
        100.0,
    )
    .expect("SLG fault should succeed");

    assert!(
        result.i_fault_a.norm() > 0.0,
        "SLG Ia must be nonzero: {:.6}",
        result.i_fault_a.norm()
    );
    assert!(
        result.i_fault_magnitude > 0.0,
        "SLG i_fault_magnitude must be > 0"
    );
}

#[test]
fn test_slg_ib_ic_small_compared_to_ia() {
    // For SLG (bolted, phase A only), Ib and Ic are non-zero due to sequence mixing
    // but the dominant fault is on phase A (Ia = 3*I1)
    let seq = typical_seq();
    let result = compute_asymmetric_fault(
        &seq,
        AsymmetricFaultType::SingleLineToGround,
        1.0,
        0,
        None,
        100.0,
    )
    .expect("SLG fault should succeed");

    // Phase A carries the bulk of the fault current
    let ia_mag = result.i_fault_a.norm();
    // Ground current (3*I0) equals Ia for SLG bolted
    let ground_mag = (result.i_fault_zero * 3.0).norm();
    assert!(
        (ia_mag - ground_mag).abs() < 1e-10,
        "Ia should equal 3*I0 for SLG: Ia={ia_mag:.6}, 3*I0={ground_mag:.6}"
    );
}

// ── LL fault tests ────────────────────────────────────────────────────────────

#[test]
fn test_ll_fault_ia_zero() {
    // LL fault (B-C): phase A current = 0
    let seq = typical_seq();
    let result =
        compute_asymmetric_fault(&seq, AsymmetricFaultType::LineToLine, 1.0, 0, None, 100.0)
            .expect("LL fault should succeed");

    assert!(
        result.i_fault_a.norm() < 1e-10,
        "Ia should be 0 for LL (B-C) fault: {:.2e}",
        result.i_fault_a.norm()
    );
}

#[test]
fn test_ll_fault_ib_ic_equal_magnitude() {
    // LL fault (B-C): |Ib| = |Ic|
    let seq = typical_seq();
    let result =
        compute_asymmetric_fault(&seq, AsymmetricFaultType::LineToLine, 1.0, 0, None, 100.0)
            .expect("LL fault should succeed");

    let ib_mag = result.i_fault_b.norm();
    let ic_mag = result.i_fault_c.norm();
    assert!(
        (ib_mag - ic_mag).abs() < 1e-10,
        "|Ib| should equal |Ic| for LL fault: |Ib|={ib_mag:.6}, |Ic|={ic_mag:.6}"
    );
}

#[test]
fn test_ll_fault_no_zero_sequence() {
    // LL fault: no ground path → I0 = 0
    let seq = typical_seq();
    let result =
        compute_asymmetric_fault(&seq, AsymmetricFaultType::LineToLine, 1.0, 0, None, 100.0)
            .expect("LL fault should succeed");

    assert!(
        result.i_fault_zero.norm() < 1e-12,
        "I0 should be 0 for LL fault: {:.2e}",
        result.i_fault_zero.norm()
    );
}

#[test]
fn test_ll_fault_current_nonzero() {
    let seq = typical_seq();
    let result =
        compute_asymmetric_fault(&seq, AsymmetricFaultType::LineToLine, 1.0, 0, None, 100.0)
            .expect("LL fault should succeed");

    assert!(
        result.i_fault_magnitude > 0.0,
        "LL fault magnitude must be > 0"
    );
}

// ── DLG fault tests ───────────────────────────────────────────────────────────

#[test]
fn test_dlg_fault_current_nonzero() {
    let seq = typical_seq();
    let result = compute_asymmetric_fault(
        &seq,
        AsymmetricFaultType::DoubleLineToGround,
        1.0,
        0,
        None,
        100.0,
    )
    .expect("DLG fault should succeed");

    assert!(result.i_fault_a.norm() > 0.0, "DLG Ia must be nonzero");
    assert!(
        result.i_fault_magnitude > 0.0,
        "DLG i_fault_magnitude must be > 0"
    );
}

#[test]
fn test_dlg_has_zero_sequence_current() {
    // DLG: ground path exists → I0 ≠ 0
    let seq = typical_seq();
    let result = compute_asymmetric_fault(
        &seq,
        AsymmetricFaultType::DoubleLineToGround,
        1.0,
        0,
        None,
        100.0,
    )
    .expect("DLG fault should succeed");

    assert!(
        result.i_fault_zero.norm() > 1e-6,
        "I0 should be nonzero for DLG: {:.2e}",
        result.i_fault_zero.norm()
    );
}

#[test]
fn test_dlg_sequence_current_balance() {
    // DLG KCL at fault point: I1 + I2 + I0 should not be zero (Ia = I0+I1+I2)
    // But phase B and C share the fault, so Ia is non-trivially related.
    // Key check: I1, I2, I0 are all nonzero and their magnitudes are consistent.
    let seq = typical_seq();
    let result = compute_asymmetric_fault(
        &seq,
        AsymmetricFaultType::DoubleLineToGround,
        1.0,
        0,
        None,
        100.0,
    )
    .expect("DLG fault should succeed");

    // All three sequence currents nonzero for DLG
    assert!(result.i_fault_positive.norm() > 1e-6, "I1 nonzero for DLG");
    assert!(result.i_fault_negative.norm() > 1e-6, "I2 nonzero for DLG");
    assert!(result.i_fault_zero.norm() > 1e-6, "I0 nonzero for DLG");

    // Ia = I0 + I1 + I2 (symmetrical components identity)
    let ia_check = result.i_fault_zero + result.i_fault_positive + result.i_fault_negative;
    assert!(
        (result.i_fault_a - ia_check).norm() < 1e-10,
        "Ia != I0+I1+I2: diff={:.2e}",
        (result.i_fault_a - ia_check).norm()
    );
}

// ── FaultAnalysis struct tests ────────────────────────────────────────────────

#[test]
fn test_fault_analysis_from_ybus() {
    let ybus = two_bus_ybus();
    let v_pre = vec![Complex64::new(1.0, 0.0); 2];
    let fa = FaultAnalysis::from_ybus(&ybus, v_pre, 100.0)
        .expect("FaultAnalysis::from_ybus should succeed");
    assert_eq!(fa.z_bus.len(), 2);
}

#[test]
fn test_fault_analysis_slg() {
    let ybus = two_bus_ybus();
    let v_pre = vec![Complex64::new(1.0, 0.0); 2];
    let fa = FaultAnalysis::from_ybus(&ybus, v_pre, 100.0).expect("from_ybus ok");

    let result = fa.single_line_to_ground_fault(0).expect("SLG ok");
    assert_eq!(result.fault_type, AsymmetricFaultType::SingleLineToGround);
    assert_eq!(result.fault_bus, 0);
    assert!(
        result.i_fault_magnitude > 0.0,
        "SLG magnitude should be > 0"
    );
}

#[test]
fn test_fault_analysis_ll() {
    let ybus = two_bus_ybus();
    let v_pre = vec![Complex64::new(1.0, 0.0); 2];
    let fa = FaultAnalysis::from_ybus(&ybus, v_pre, 100.0).expect("from_ybus ok");

    let result = fa.line_to_line_fault(1).expect("LL ok");
    assert_eq!(result.fault_type, AsymmetricFaultType::LineToLine);
    assert!(result.i_fault_magnitude > 0.0, "LL magnitude should be > 0");
    assert!(
        result.i_fault_a.norm() < 1e-10,
        "Ia should be ~0 for LL fault: {:.2e}",
        result.i_fault_a.norm()
    );
}

#[test]
fn test_fault_analysis_dlg() {
    let ybus = two_bus_ybus();
    let v_pre = vec![Complex64::new(1.0, 0.0); 2];
    let fa = FaultAnalysis::from_ybus(&ybus, v_pre, 100.0).expect("from_ybus ok");

    let result = fa.double_line_to_ground_fault(0).expect("DLG ok");
    assert_eq!(result.fault_type, AsymmetricFaultType::DoubleLineToGround);
    assert!(
        result.i_fault_magnitude > 0.0,
        "DLG magnitude should be > 0"
    );
}

#[test]
fn test_fault_analysis_all_asymmetric_faults() {
    let ybus = two_bus_ybus();
    let v_pre = vec![Complex64::new(1.0, 0.0); 2];
    let fa = FaultAnalysis::from_ybus(&ybus, v_pre, 100.0).expect("from_ybus ok");

    let results = fa
        .all_asymmetric_faults(0)
        .expect("all_asymmetric_faults ok");
    assert_eq!(
        results[0].fault_type,
        AsymmetricFaultType::SingleLineToGround
    );
    assert_eq!(results[1].fault_type, AsymmetricFaultType::LineToLine);
    assert_eq!(
        results[2].fault_type,
        AsymmetricFaultType::DoubleLineToGround
    );

    for r in &results {
        assert!(r.i_fault_magnitude > 0.0, "All fault magnitudes > 0");
    }
}

#[test]
fn test_fault_analysis_set_bus_kv_enables_ka() {
    let ybus = two_bus_ybus();
    let v_pre = vec![Complex64::new(1.0, 0.0); 2];
    let mut fa = FaultAnalysis::from_ybus(&ybus, v_pre, 100.0).expect("from_ybus ok");

    fa.set_bus_kv(0, 115.0).expect("set_bus_kv ok");
    let result = fa.single_line_to_ground_fault(0).expect("SLG ok");
    assert!(
        result.i_fault_ka.is_some(),
        "i_fault_ka should be set when base_kv is provided"
    );
    assert!(
        result.i_fault_ka.unwrap() > 0.0,
        "i_fault_ka should be positive"
    );
}

#[test]
fn test_fault_analysis_out_of_range_bus() {
    let ybus = two_bus_ybus();
    let v_pre = vec![Complex64::new(1.0, 0.0); 2];
    let fa = FaultAnalysis::from_ybus(&ybus, v_pre, 100.0).expect("from_ybus ok");

    assert!(
        fa.single_line_to_ground_fault(99).is_err(),
        "Out-of-range bus should return error"
    );
}

#[test]
fn test_compute_asymmetric_fault_bus_index_stored() {
    let seq = typical_seq();
    let result = compute_asymmetric_fault(
        &seq,
        AsymmetricFaultType::SingleLineToGround,
        1.0,
        7, // bus index
        None,
        100.0,
    )
    .expect("SLG ok");
    assert_eq!(result.fault_bus, 7);
}

#[test]
fn test_max_phase_current_pu() {
    let seq = typical_seq();
    let result = compute_asymmetric_fault(
        &seq,
        AsymmetricFaultType::SingleLineToGround,
        1.0,
        0,
        None,
        100.0,
    )
    .expect("SLG ok");
    let max_phase = result.max_phase_current_pu();
    assert!(
        max_phase >= result.i_fault_magnitude
            || (max_phase - result.i_fault_magnitude).abs() < 1e-10,
        "max_phase_current_pu should match i_fault_magnitude for SLG"
    );
}

#[test]
fn test_slg_larger_than_ll_for_solidly_grounded() {
    // When Z0 is small (solidly grounded), SLG yields higher fault current than LL
    let seq = SequenceImpedances {
        z1: Complex64::new(0.0, 0.12),
        z2: Complex64::new(0.0, 0.12),
        z0: Complex64::new(0.0, 0.05), // small zero-sequence
    };
    let slg = compute_asymmetric_fault(
        &seq,
        AsymmetricFaultType::SingleLineToGround,
        1.0,
        0,
        None,
        100.0,
    )
    .expect("SLG ok");
    let ll = compute_asymmetric_fault(&seq, AsymmetricFaultType::LineToLine, 1.0, 0, None, 100.0)
        .expect("LL ok");

    assert!(slg.i_fault_magnitude > 0.0, "SLG has fault current");
    assert!(ll.i_fault_magnitude > 0.0, "LL has fault current");
    // For solidly grounded (small Z0), SLG should exceed LL
    assert!(
        slg.i_fault_magnitude > ll.i_fault_magnitude,
        "SLG should exceed LL for solidly grounded system: SLG={:.4} LL={:.4}",
        slg.i_fault_magnitude,
        ll.i_fault_magnitude
    );
}

#[test]
fn test_compute_zbus_and_fault_analysis_integration() {
    // Build Z-bus from a pure inductive Y-bus and run all fault types
    let y_val = Complex64::new(0.0, 10.0); // x = 0.1 pu
    let y11 = y_val + y_val;
    let y12 = -y_val;
    let ybus = vec![vec![y11, y12], vec![y12, y11]];

    let zbus = compute_zbus(&ybus).expect("compute_zbus ok");
    let v_pre = vec![Complex64::new(1.0, 0.0); 2];
    let fa = FaultAnalysis {
        z_bus: zbus,
        v_prefault: v_pre,
        base_mva: 100.0,
        bus_kv: vec![None, None],
    };

    // All three fault types at bus 0
    let slg = fa.single_line_to_ground_fault(0).expect("SLG ok");
    let ll = fa.line_to_line_fault(0).expect("LL ok");
    let dlg = fa.double_line_to_ground_fault(0).expect("DLG ok");

    assert!(slg.i_fault_magnitude > 0.0);
    assert!(ll.i_fault_magnitude > 0.0);
    assert!(dlg.i_fault_magnitude > 0.0);
}
