#![cfg(feature = "battery")]
/// LFP chemistry validation tests.
///
/// Tests specific behaviours of LFP cells:
/// - Flat OCV plateau (3.2-3.4 V range)
/// - Energy conservation across charge-discharge cycle
/// - Thermal rise under load
use oxigrid::battery::ecm::{OneRcModel, RintModel};
use oxigrid::battery::thermal::LumpedThermalModel;
use oxigrid::battery::OcvSocCurve;
use oxigrid::units::{Current, StateOfCharge, Temperature};

#[test]
fn test_lfp_ocv_plateau_flat() {
    let curve = OcvSocCurve::lfp_default();
    // LFP has a very flat plateau between ~20% and 80% SoC
    let v20 = curve.ocv(0.20);
    let v80 = curve.ocv(0.80);
    // Should span < 150 mV (LFP characteristic)
    assert!(
        (v80 - v20) < 0.15,
        "LFP plateau too steep: v20={:.3}V v80={:.3}V diff={:.3}V",
        v20, v80, v80 - v20
    );
}

#[test]
fn test_lfp_rint_energy_conservation() {
    let curve = OcvSocCurve::lfp_default();
    let capacity = 10.0_f64; // 10 Ah
    let r0 = 0.003;
    let mut model = RintModel::new(curve.clone(), r0, capacity).with_soc(0.0);

    // Charge from 0% to ~100% at 1C
    let charge_current = Current(-capacity);
    let dt = 1.0;
    let mut energy_in = 0.0_f64;

    for _ in 0..3700 {
        let state = model.step(charge_current, dt, Temperature(298.15));
        energy_in += state.voltage.0 * charge_current.0.abs() * dt / 3600.0; // Wh
        if state.soc.0 > 0.99 {
            break;
        }
    }
    let soc_charged = model.soc;

    // Discharge back to 0%
    let discharge_current = Current(capacity);
    let mut energy_out = 0.0_f64;

    for _ in 0..3700 {
        let state = model.step(discharge_current, dt, Temperature(298.15));
        energy_out += state.voltage.0 * discharge_current.0 * dt / 3600.0; // Wh
        if state.soc.0 < 0.01 {
            break;
        }
    }

    // Coulombic efficiency: energy_out / energy_in should be > 90%
    let eta = energy_out / energy_in;
    assert!(
        eta > 0.90,
        "Round-trip efficiency = {:.1}%, expected > 90%",
        eta * 100.0
    );
    let _ = soc_charged;
}

#[test]
fn test_lfp_terminal_voltage_under_load() {
    let model = RintModel::new(OcvSocCurve::lfp_default(), 0.005, 75.0).with_soc(0.5);
    // At 50% SoC, OCV ≈ 3.32 V; at 1C (75A), voltage drop = 75 * 0.005 = 0.375 V
    let v = model.terminal_voltage(
        StateOfCharge::new(0.5),
        Current(75.0),
        Temperature(298.15),
    );
    let ocv = OcvSocCurve::lfp_default().ocv(0.5);
    assert!((v.0 - (ocv - 75.0 * 0.005)).abs() < 1e-6);
}

#[test]
fn test_lfp_1rc_rest_relaxation() {
    let curve = OcvSocCurve::lfp_default();
    let mut model = OneRcModel::new(curve, 0.002, 0.008, 5000.0, 75.0);
    // Discharge pulse: 5 seconds at 2C
    for _ in 0..5 {
        model.step(Current(150.0), 1.0, Temperature(298.15));
    }
    let v_rc1_end_of_pulse = model.v_rc1;
    // Rest: 300 seconds (τ1 = 0.008*5000 = 40s → should largely decay)
    for _ in 0..300 {
        model.step(Current(0.0), 1.0, Temperature(298.15));
    }
    // RC1 should have decayed to < 5% of pulse value
    assert!(
        model.v_rc1.abs() < v_rc1_end_of_pulse.abs() * 0.05,
        "RC1 not relaxing: {:.4} vs {:.4}",
        model.v_rc1, v_rc1_end_of_pulse
    );
}

#[test]
fn test_thermal_rise_lfp_cell() {
    let mut thermal = LumpedThermalModel::new(0.25, 1050.0, 12.0, 0.025);
    let initial = thermal.temperature;
    // 50A discharge (typical BEV rate), R_eff = 3 mΩ
    for _ in 0..600 {
        thermal.step(50.0, 0.003, 1.0);
    }
    // Should heat up by at least 1°C but less than 50°C
    let rise = thermal.temperature - initial;
    assert!(rise > 1.0, "No thermal rise observed: {:.2} K", rise);
    assert!(rise < 50.0, "Excessive thermal rise: {:.2} K", rise);
}
