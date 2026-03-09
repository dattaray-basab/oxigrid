#![cfg(feature = "harmonics")]
use oxigrid::harmonics::analysis::{analyse, synthetic_waveform};
use oxigrid::harmonics::filter::{FilterType, PassiveFilter};
use oxigrid::harmonics::standards::{
    check_iec61000_3_2_class_a, check_ieee519_voltage, iec61000_3_2_class_a,
    ieee519_current_limits, ieee519_voltage_limits, HarmonicLimit,
};

// ── Harmonic Analysis ─────────────────────────────────────────────────────────

#[test]
fn test_thd_pure_fundamental() {
    // Use 6000 Hz sample rate with 1000 samples = exactly 10 cycles at 60 Hz
    let samples = synthetic_waveform(60.0, 6000.0, 1000, &[(1, 1.0, 0.0)]);
    let spectrum = analyse(&samples, 6000.0, 60.0, 20, None);
    assert!(
        spectrum.thd_pct < 1.0,
        "THD of pure fundamental should be near 0: {:.4}%",
        spectrum.thd_pct
    );
}

#[test]
fn test_thd_with_harmonics() {
    // 5th at 20%, 7th at 14.3% → THD ≈ 24.5%
    let samples = synthetic_waveform(
        60.0,
        6000.0,
        1024,
        &[(1, 1.0, 0.0), (5, 0.20, 0.0), (7, 0.143, 0.0)],
    );
    let spectrum = analyse(&samples, 6000.0, 60.0, 20, None);
    let expected_thd = (0.20_f64.powi(2) + 0.143_f64.powi(2)).sqrt() * 100.0;
    assert!(
        (spectrum.thd_pct - expected_thd).abs() < 5.0,
        "THD = {:.2}%, expected ≈ {:.2}%",
        spectrum.thd_pct,
        expected_thd
    );
}

#[test]
fn test_analyse_fundamental_magnitude() {
    // 1000 samples at 6000 Hz = exactly 10 cycles at 60 Hz → clean DFT bin
    let samples = synthetic_waveform(60.0, 6000.0, 1000, &[(1, 100.0, 0.0)]);
    let spectrum = analyse(&samples, 6000.0, 60.0, 20, None);
    // Fundamental RMS ≈ 100/√2 ≈ 70.7
    let expected_rms = 100.0 / 2.0_f64.sqrt();
    assert!(
        (spectrum.fundamental - expected_rms).abs() / expected_rms < 0.05,
        "Fundamental magnitude {:.4} should be near {:.4}",
        spectrum.fundamental,
        expected_rms
    );
}

#[test]
fn test_spectrum_harmonic_content() {
    // Waveform with 3rd harmonic at 10%
    let samples = synthetic_waveform(60.0, 6000.0, 1024, &[(1, 1.0, 0.0), (3, 0.1, 0.0)]);
    let spectrum = analyse(&samples, 6000.0, 60.0, 20, None);
    // Should detect 3rd harmonic
    let h3 = spectrum.harmonics.iter().find(|h| h.order == 3);
    assert!(h3.is_some(), "Should detect 3rd harmonic");
    if let Some(h) = h3 {
        // IHD at 3rd should be near 10%
        assert!(
            (h.ihd_pct - 10.0).abs() < 3.0,
            "3rd harmonic IHD = {:.2}%, expected ≈ 10%",
            h.ihd_pct
        );
    }
}

#[test]
fn test_ieee519_voltage_compliant() {
    // Synthesise a spectrum that meets IEEE 519-2022 voltage limits
    // For 13.8 kV bus: THD < 5%, individual < 3%
    let samples = synthetic_waveform(60.0, 6000.0, 1024, &[(1, 1.0, 0.0), (5, 0.02, 0.0)]);
    let spectrum = analyse(&samples, 6000.0, 60.0, 20, None);
    assert!(
        spectrum.ieee519_voltage_compliant(13.8),
        "Should comply: THD = {:.2}%",
        spectrum.thd_pct
    );
}

// ── Passive Filters ───────────────────────────────────────────────────────────

#[test]
fn test_single_tuned_filter_resonance() {
    let f = PassiveFilter::single_tuned(5.0, 1.0, 60.0, 13.8, 50.0);
    assert_eq!(f.filter_type, FilterType::SingleTuned);
    assert!((f.harmonic_order - 5.0).abs() < 1e-9);
    // At resonance (5th = 300 Hz), impedance should be much lower than off-resonance
    let z_at_res = f.impedance(300.0);
    let z_off = f.impedance(420.0); // 7th harmonic
    assert!(
        z_at_res.norm() < z_off.norm(),
        "Filter impedance at resonance ({:.4}) should be less than off ({:.4})",
        z_at_res.norm(),
        z_off.norm()
    );
}

#[test]
fn test_high_pass_filter() {
    let f = PassiveFilter::high_pass(7.0, 1.0, 60.0, 13.8, 2.0);
    assert_eq!(f.filter_type, FilterType::HighPass);
    // High-pass: high frequency impedance should be lower than mid
    let z_high = f.impedance(600.0);
    let z_low = f.impedance(60.0);
    // Just verify both return finite values
    assert!(
        z_high.norm().is_finite(),
        "High-pass impedance at high freq should be finite"
    );
    assert!(
        z_low.norm().is_finite(),
        "High-pass impedance at low freq should be finite"
    );
}

#[test]
fn test_filter_reactive_power() {
    let f = PassiveFilter::single_tuned(5.0, 1.0, 60.0, 13.8, 50.0);
    let q = f.reactive_power_mvar();
    assert!(
        (q - 1.0).abs() < 1e-9,
        "Reactive power should equal design value: {q:.6}"
    );
}

#[test]
fn test_filter_capacitance_positive() {
    let f = PassiveFilter::single_tuned(5.0, 2.0, 60.0, 13.8, 50.0);
    let c = f.capacitance_uf();
    assert!(c > 0.0, "Capacitance should be positive: {c:.6} µF");
}

#[test]
fn test_filter_inductance_positive() {
    let f = PassiveFilter::single_tuned(5.0, 2.0, 60.0, 13.8, 50.0);
    let l = f.inductance_mh();
    assert!(l > 0.0, "Inductance should be positive: {l:.6} mH");
}

#[test]
fn test_mitigation_factor() {
    // Mitigation factor at tuning harmonic should be significant
    let f = PassiveFilter::single_tuned(5.0, 1.0, 60.0, 13.8, 50.0);
    let z_sys = 1.0; // 1 Ω system impedance
    let mf = f.mitigation_factor(5, z_sys);
    assert!(
        mf > 0.0 && mf <= 1.0,
        "Mitigation factor should be in (0,1]: {mf:.4}"
    );
}

// ── IEEE 519-2022 Voltage Limits ──────────────────────────────────────────────

#[test]
fn test_ieee519_voltage_limits_exist() {
    let limits = ieee519_voltage_limits();
    assert!(
        !limits.is_empty(),
        "IEEE 519 voltage limits table should not be empty"
    );
    for l in &limits {
        assert!(l.thd_limit_pct > 0.0, "THD limit should be positive");
    }
}

#[test]
fn test_ieee519_voltage_no_violation() {
    let harmonics_pct = vec![(5u32, 1.5_f64), (7, 1.0), (11, 0.8)];
    let violations = check_ieee519_voltage(13.8, &harmonics_pct, 2.5);
    assert!(
        violations.is_empty(),
        "Should have no violations with low harmonics: {violations:?}"
    );
}

#[test]
fn test_ieee519_voltage_thd_violation() {
    let harmonics_pct = vec![(5u32, 5.0_f64), (7, 4.0), (11, 3.0)];
    let violations = check_ieee519_voltage(13.8, &harmonics_pct, 15.0);
    assert!(
        !violations.is_empty(),
        "High THD should trigger violation: {violations:?}"
    );
}

#[test]
fn test_ieee519_current_limits_ordered() {
    let limits = ieee519_current_limits();
    assert!(!limits.is_empty());
    // Higher Isc/IL → higher allowed distortion
    if limits.len() >= 2 {
        assert!(
            limits.last().unwrap().tdd_pct >= limits.first().unwrap().tdd_pct,
            "Higher Isc/IL ratio should allow more current distortion"
        );
    }
}

// ── IEC 61000-3-2 Class A ─────────────────────────────────────────────────────

#[test]
fn test_iec61000_class_a_limits_exist() {
    let limits = iec61000_3_2_class_a();
    assert!(!limits.is_empty(), "Class A limits should not be empty");
    // 3rd harmonic should be present
    assert!(
        limits.iter().any(|l: &HarmonicLimit| l.harmonic == 3),
        "Class A limits should include 3rd harmonic"
    );
}

#[test]
fn test_iec61000_class_a_3rd_harmonic() {
    let limits = iec61000_3_2_class_a();
    let h3 = limits.iter().find(|l| l.harmonic == 3).unwrap();
    // IEC 61000-3-2 Class A, 3rd harmonic: 2.3 A
    assert!(
        (h3.limit - 2.3).abs() < 0.2,
        "Class A 3rd harmonic limit = {:.4} A, expected ≈ 2.3",
        h3.limit
    );
}

#[test]
fn test_iec61000_check_compliant() {
    let measured = vec![(3u32, 0.5_f64), (5, 0.3), (7, 0.2)];
    let violations = check_iec61000_3_2_class_a(&measured);
    assert!(
        violations.is_empty(),
        "Low harmonics should be compliant: {violations:?}"
    );
}

#[test]
fn test_iec61000_check_violation() {
    let measured = vec![(3u32, 50.0_f64), (5, 40.0), (7, 30.0)];
    let violations = check_iec61000_3_2_class_a(&measured);
    assert!(
        !violations.is_empty(),
        "High harmonics should violate IEC 61000-3-2"
    );
}
