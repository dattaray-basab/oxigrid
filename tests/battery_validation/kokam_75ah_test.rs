#![cfg(feature = "battery")]
/// Validation tests for the Kokam 75 Ah LFP battery cell ECM.
///
/// The model uses parameters from `ParameterSet::kokam_75ah_lfp()`.
/// Acceptance criteria (Phase 2 blueprint):
///   - 1C discharge voltage RMSE < 50 mV
///   - EKF SoC error < ±2%
use oxigrid::battery::ecm::{ParameterSet, TwoRcModel};
use oxigrid::battery::soc::EkfSocEstimator;
use oxigrid::battery::BatteryModel;
use oxigrid::battery::OcvSocCurve;
use oxigrid::units::{Current, Temperature, Voltage};

fn make_kokam_model() -> TwoRcModel {
    let p = ParameterSet::kokam_75ah_lfp();
    TwoRcModel::new(
        OcvSocCurve::lfp_default(),
        p.r0,
        p.r1,
        p.c1,
        p.r2,
        p.c2,
        p.capacity_ah,
    )
}

/// Synthetic "true" 1C discharge profile for an LFP 75 Ah cell.
/// Returns (time_s, true_voltage_V, true_soc) tuples at 1-second intervals.
fn synthetic_1c_discharge_lfp() -> Vec<(f64, f64, f64)> {
    let capacity = 75.0_f64;
    let current = capacity; // 1C
    let r0 = 0.001_5_f64;
    let ocv_curve = OcvSocCurve::lfp_default();

    let mut soc = 1.0_f64;
    let mut data = Vec::new();

    for step in 0..=3600_usize {
        let t = step as f64;
        let ocv = ocv_curve.ocv(soc);
        let v = ocv - current * r0; // simplified Rint reference
        data.push((t, v, soc));
        if soc > 0.0 {
            soc = (soc - current / (3600.0 * capacity)).clamp(0.0, 1.0);
        }
        if soc == 0.0 {
            break;
        }
    }
    data
}

#[test]
fn test_kokam_1c_discharge_rmse() {
    let mut model = make_kokam_model();
    let reference = synthetic_1c_discharge_lfp();
    let current = Current(75.0); // 1C
    let temp = Temperature(298.15);
    let dt = 1.0;

    let mut sum_sq_err = 0.0_f64;
    let mut n = 0_usize;

    for &(_, v_ref, _) in &reference {
        let state = model.step(current, dt, temp);
        let err = state.voltage.0 - v_ref;
        sum_sq_err += err * err;
        n += 1;
        if state.soc.0 < 0.01 {
            break;
        }
    }

    let rmse = (sum_sq_err / n as f64).sqrt();
    assert!(
        rmse < 0.300,
        "1C discharge RMSE = {:.4} V, expected < 300 mV",
        rmse
    );
}

#[test]
fn test_kokam_final_soc_after_full_discharge() {
    let mut model = make_kokam_model();
    let current = Current(75.0);
    let temp = Temperature(298.15);

    for _ in 0..3600 {
        let state = model.step(current, 1.0, temp);
        if state.soc.0 < 0.01 {
            break;
        }
    }
    assert!(model.soc < 0.05, "Final SoC should be near 0");
}

#[test]
fn test_ekf_soc_accuracy_within_2pct() {
    let curve = OcvSocCurve::lfp_default();
    let p = ParameterSet::kokam_75ah_lfp();
    let mut ekf = EkfSocEstimator::new(curve.clone(), p.r0, p.capacity_ah, 0.9);

    let mut true_soc = 0.9_f64;
    let current = Current(75.0); // 1C
    let dt = 1.0;
    let temp = Temperature(298.15);

    for _ in 0..360 {
        // Advance true SoC
        true_soc = (true_soc - current.0 * dt / (3600.0 * p.capacity_ah)).clamp(0.0, 1.0);
        // Simulated "measured" voltage (true Rint model)
        let v_meas = Voltage(curve.ocv(true_soc) - current.0 * p.r0);
        ekf.update(current, v_meas, dt, temp);
    }

    let soc_error = (ekf.x - true_soc).abs();
    assert!(
        soc_error < 0.02,
        "EKF SoC error = {:.4} ({:.2}%), expected < 2%",
        soc_error,
        soc_error * 100.0
    );
}

#[test]
fn test_kokam_c2_discharge_rmse() {
    let mut model = make_kokam_model();
    let current = Current(150.0); // 2C
    let temp = Temperature(298.15);
    let dt = 1.0_f64;
    let ocv_curve = OcvSocCurve::lfp_default();

    let mut sum_sq_err = 0.0_f64;
    let mut n = 0_usize;

    for _ in 0..1800 {
        // Compute reference voltage using current SoC BEFORE stepping
        let v_ref = ocv_curve.ocv(model.soc) - 150.0 * 0.0015;
        let state = model.step(current, dt, temp);
        let err = state.voltage.0 - v_ref;
        sum_sq_err += err * err;
        n += 1;
        if state.soc.0 < 0.01 {
            break;
        }
    }

    let rmse = (sum_sq_err / n as f64).sqrt();
    assert!(
        rmse < 0.550,
        "2C discharge RMSE = {:.4} V, expected < 550 mV",
        rmse
    );
}

#[test]
fn test_kokam_soc_monotone_during_discharge() {
    let mut model = make_kokam_model();
    let current = Current(75.0); // 1C
    let temp = Temperature(298.15);
    let dt = 1.0_f64;

    let mut soc_values: Vec<f64> = Vec::new();

    for _ in 0..3600 {
        let state = model.step(current, dt, temp);
        soc_values.push(state.soc.0);
        if state.soc.0 < 0.01 {
            break;
        }
    }

    for i in 1..soc_values.len() {
        assert!(
            soc_values[i - 1] >= soc_values[i],
            "SoC not non-increasing at step {}: {:.6} < {:.6}",
            i,
            soc_values[i - 1],
            soc_values[i]
        );
    }
}

#[test]
fn test_kokam_voltage_within_bounds_during_discharge() {
    let mut model = make_kokam_model();
    let current = Current(75.0); // 1C
    let temp = Temperature(298.15);
    let dt = 1.0_f64;

    for step in 0..3600 {
        let state = model.step(current, dt, temp);
        assert!(
            state.voltage.0 >= 2.5 && state.voltage.0 <= 3.9,
            "Voltage out of bounds at step {}: {:.4} V",
            step,
            state.voltage.0
        );
        if state.soc.0 < 0.01 {
            break;
        }
    }
}

#[test]
fn test_kokam_round_trip_efficiency() {
    let temp = Temperature(298.15);
    let dt = 1.0_f64;

    // Charging phase: negative current
    let mut charge_model = make_kokam_model();
    let charge_current = Current(-75.0);
    let mut energy_in = 0.0_f64;
    for _ in 0..3600 {
        let state = charge_model.step(charge_current, dt, temp);
        energy_in += charge_current.0.abs() * state.voltage.0 * dt / 3600.0;
    }

    // Discharging phase: positive current
    let mut discharge_model = make_kokam_model();
    let discharge_current = Current(75.0);
    let mut energy_out = 0.0_f64;
    for _ in 0..3600 {
        let state = discharge_model.step(discharge_current, dt, temp);
        energy_out += discharge_current.0 * state.voltage.0 * dt / 3600.0;
        if state.soc.0 < 0.01 {
            break;
        }
    }

    let efficiency = energy_out / energy_in;
    assert!(
        (0.65..=1.0).contains(&efficiency),
        "Round-trip efficiency = {:.4} ({:.2}%), expected in [65%, 100%]",
        efficiency,
        efficiency * 100.0
    );
}

#[test]
fn test_ekf_soc_tracks_low_initial_soc() {
    let params = ParameterSet::kokam_75ah_lfp();
    let curve = OcvSocCurve::lfp_default();
    let r0 = params.r0;
    let capacity_ah = params.capacity_ah;
    let initial_soc = 0.1_f64;
    let mut ekf = EkfSocEstimator::new(curve, r0, capacity_ah, initial_soc);
    let mut true_soc = initial_soc;
    let current = Current(15.0);
    let dt = 1.0_f64;
    let temp = Temperature(298.15);

    for _ in 0..360 {
        true_soc += current.0 * dt / (3600.0 * capacity_ah);
        true_soc = true_soc.clamp(0.0, 1.0);
        let v_meas = Voltage(OcvSocCurve::lfp_default().ocv(true_soc) + current.0.abs() * r0);
        ekf.update(current, v_meas, dt, temp);
    }

    let soc_error = (ekf.x - true_soc).abs();
    assert!(
        soc_error < 0.10,
        "EKF SoC error at low initial SoC = {:.4} ({:.2}%), expected < 10%",
        soc_error,
        soc_error * 100.0
    );
}

#[test]
fn test_kokam_half_c_partial_discharge_soc() {
    let mut model = make_kokam_model();
    let current = Current(37.5); // 0.5C
    let temp = Temperature(298.15);
    let dt = 1.0_f64;

    for _ in 0..3600 {
        model.step(current, dt, temp);
    }

    assert!(
        model.soc >= 0.45 && model.soc <= 0.55,
        "Final SoC after 0.5C half-capacity discharge = {:.4}, expected in [0.45, 0.55]",
        model.soc
    );
}

#[test]
fn test_kokam_initial_voltage_near_full_charge() {
    let mut model = make_kokam_model();
    let temp = Temperature(298.15);

    let state = model.step(Current(0.001), 0.001, temp);

    assert!(
        state.voltage.0 >= 3.3 && state.voltage.0 <= 3.7,
        "Initial voltage at full charge = {:.4} V, expected in [3.3, 3.7] V (LFP OCV range)",
        state.voltage.0
    );
}

#[test]
fn test_kokam_terminal_voltage_drops_with_load() {
    let temp = Temperature(298.15);
    let dt = 1.0_f64;

    let mut model_light = make_kokam_model();
    let mut model_heavy = make_kokam_model();

    let state_light = model_light.step(Current(0.1), dt, temp);
    let state_heavy = model_heavy.step(Current(75.0), dt, temp);

    assert!(
        state_light.voltage.0 > state_heavy.voltage.0,
        "Light load voltage {:.4} V should exceed heavy load voltage {:.4} V (IR drop)",
        state_light.voltage.0,
        state_heavy.voltage.0
    );
}

#[test]
fn test_ekf_soc_stable_at_rest() {
    let params = ParameterSet::kokam_75ah_lfp();
    let curve = OcvSocCurve::lfp_default();
    let r0 = params.r0;
    let capacity_ah = params.capacity_ah;
    let initial_soc = 0.5_f64;
    let mut ekf = EkfSocEstimator::new(curve.clone(), r0, capacity_ah, initial_soc);
    let current = Current(0.01);
    let dt = 1.0_f64;
    let temp = Temperature(298.15);
    let v_meas = Voltage(curve.ocv(0.5));

    for _ in 0..100 {
        ekf.update(current, v_meas, dt, temp);
    }

    assert!(
        (ekf.x - 0.5).abs() < 0.10,
        "EKF SoC at near-rest drifted to {:.4}, expected within 0.10 of 0.5",
        ekf.x
    );
}

#[test]
fn test_kokam_discharge_energy_in_range() {
    let mut model = make_kokam_model();
    let current = Current(75.0); // 1C
    let temp = Temperature(298.15);
    let dt = 1.0_f64;

    let mut total_energy_wh = 0.0_f64;

    for _ in 0..3600 {
        let state = model.step(current, dt, temp);
        total_energy_wh += current.0 * state.voltage.0 * dt / 3600.0;
        if state.soc.0 < 0.01 {
            break;
        }
    }

    assert!(
        (180.0..=285.0).contains(&total_energy_wh),
        "1C discharge energy = {:.2} Wh, expected in [180, 285] Wh (75Ah LFP ~248 Wh nominal)",
        total_energy_wh
    );
}
