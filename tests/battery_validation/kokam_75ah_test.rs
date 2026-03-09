#![cfg(feature = "battery")]
/// Validation tests for the Kokam 75 Ah LFP battery cell ECM.
///
/// The model uses parameters from `ParameterSet::kokam_75ah_lfp()`.
/// Acceptance criteria (Phase 2 blueprint):
///   - 1C discharge voltage RMSE < 50 mV
///   - EKF SoC error < ±2%
use oxigrid::battery::ecm::{ParameterSet, TwoRcModel};
use oxigrid::battery::soc::EkfSocEstimator;
use oxigrid::battery::OcvSocCurve;
use oxigrid::units::{Current, Temperature, Voltage};

fn make_kokam_model() -> TwoRcModel {
    let p = ParameterSet::kokam_75ah_lfp();
    TwoRcModel::new(
        OcvSocCurve::lfp_default(),
        p.r0, p.r1, p.c1, p.r2, p.c2, p.capacity_ah,
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
        rmse < 0.050,
        "1C discharge RMSE = {:.4} V, expected < 50 mV",
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
