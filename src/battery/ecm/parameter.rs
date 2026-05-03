/// Battery ECM parameter identification.
///
/// Provides parameter sets for common battery chemistries and
/// ECM parameter identification via L-BFGS offline batch fitting.
use serde::{Deserialize, Serialize};

use super::lbfgs::{lbfgs_minimize, LbfgsConfig};

/// Complete parameter set for a 2RC Thevenin model.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ParameterSet {
    pub r0: f64,          // [Ω] ohmic resistance
    pub r1: f64,          // [Ω] RC pair 1 resistance
    pub c1: f64,          // [F] RC pair 1 capacitance
    pub r2: f64,          // [Ω] RC pair 2 resistance
    pub c2: f64,          // [F] RC pair 2 capacitance
    pub capacity_ah: f64, // [Ah] nominal capacity
    /// Optional temperature coefficients: (dR0/dT, dR1/dT, dR2/dT) [Ω/K]
    pub temp_coeffs: Option<(f64, f64, f64)>,
}

impl ParameterSet {
    /// Typical 75 Ah LFP cell parameters.
    pub fn kokam_75ah_lfp() -> Self {
        Self {
            r0: 0.001_5,
            r1: 0.001_0,
            c1: 40_000.0,
            r2: 0.002_0,
            c2: 5_000.0,
            capacity_ah: 75.0,
            temp_coeffs: Some((-5e-6, -3e-6, -4e-6)),
        }
    }

    /// Typical 3 Ah NMC cell parameters.
    pub fn nmc_3ah() -> Self {
        Self {
            r0: 0.020,
            r1: 0.015,
            c1: 3_000.0,
            r2: 0.010,
            c2: 500.0,
            capacity_ah: 3.0,
            temp_coeffs: None,
        }
    }

    /// Fit a 2RC model to pulse discharge data via L-BFGS batch optimization.
    ///
    /// `data` is a slice of (time_s, current_A, voltage_V) tuples.
    /// Returns fitted parameters or an error string.
    ///
    /// The heuristic estimates provide a warm-start initial point; L-BFGS
    /// then minimizes the mean-squared voltage residual over all samples.
    pub fn fit_from_pulse_data(data: &[(f64, f64, f64)], capacity_ah: f64) -> Result<Self, String> {
        if data.len() < 10 {
            return Err("Need at least 10 data points for parameter fitting".into());
        }

        // Warm-start initial guesses from heuristics
        let r0_est = estimate_r0_from_pulse(data);
        let (r1_est, c1_est, r2_est, c2_est) = estimate_rc_from_relaxation(data);

        let init = [
            r0_est.ln(),
            r1_est.ln(),
            c1_est.ln(),
            r2_est.ln(),
            c2_est.ln(),
        ];

        let cfg = LbfgsConfig {
            max_iter: 500,
            ..LbfgsConfig::default()
        };

        let (x_opt, _f_opt, _) = lbfgs_minimize(|x| ecm_simulate_loss(data, x), &init, &cfg)
            .map_err(|e| format!("L-BFGS failed: {e}"))?;

        // Canonicalise: pair-1 is the slow branch (τ1 ≥ τ2).
        // The 2RC loss is symmetric under (R1,C1) ↔ (R2,C2), so L-BFGS may
        // return either labelling. Sorting by time-constant removes the ambiguity.
        let (mut r1, mut c1) = (x_opt[1].exp(), x_opt[2].exp());
        let (mut r2, mut c2) = (x_opt[3].exp(), x_opt[4].exp());
        if r1 * c1 < r2 * c2 {
            std::mem::swap(&mut r1, &mut r2);
            std::mem::swap(&mut c1, &mut c2);
        }

        Ok(Self {
            r0: x_opt[0].exp(),
            r1,
            c1,
            r2,
            c2,
            capacity_ah,
            temp_coeffs: None,
        })
    }
}

/// Simulate a 2RC ECM over `data` and return mean-squared voltage error.
///
/// `log_params` = [ln r0, ln r1, ln c1, ln r2, ln c2] — log-space encoding
/// guarantees strictly positive values throughout the L-BFGS optimization.
fn ecm_simulate_loss(data: &[(f64, f64, f64)], log_params: &[f64]) -> f64 {
    if data.is_empty() || log_params.len() < 5 {
        return f64::INFINITY;
    }
    let r0 = log_params[0].exp();
    let r1 = log_params[1].exp();
    let c1 = log_params[2].exp();
    let r2 = log_params[3].exp();
    let c2 = log_params[4].exp();

    // OCV: average voltage from the leading pre-pulse rest segment only.
    // Using post-pulse rest would be biased (RC voltage still non-zero).
    // Fall back to ohmic-corrected first sample when no leading rest exists.
    let mut pre_pulse_vs: Vec<f64> = Vec::new();
    for &(_, i, v) in data.iter() {
        if i.abs() < 1e-6 {
            pre_pulse_vs.push(v);
        } else {
            break; // stop at first non-rest sample
        }
    }
    let ocv = if pre_pulse_vs.is_empty() {
        data[0].2 + r0 * data[0].1
    } else {
        pre_pulse_vs.iter().sum::<f64>() / pre_pulse_vs.len() as f64
    };

    // Initialise t_prev one step before the first sample so the first
    // Euler integration step uses the correct dt instead of dt ≈ 0.
    let dt0 = if data.len() > 1 {
        data[1].0 - data[0].0
    } else {
        0.1
    };
    let mut t_prev = data[0].0 - dt0;
    let mut v_rc1 = 0.0f64;
    let mut v_rc2 = 0.0f64;
    let mut loss = 0.0f64;

    for &(t, i_load, v_meas) in data.iter() {
        let dt = (t - t_prev).max(1e-9);
        t_prev = t;
        v_rc1 += dt * (i_load / c1 - v_rc1 / (r1 * c1));
        v_rc2 += dt * (i_load / c2 - v_rc2 / (r2 * c2));
        let v_sim = ocv - i_load * r0 - v_rc1 - v_rc2;
        let err = v_sim - v_meas;
        loss += err * err;
    }
    loss / data.len() as f64
}

fn estimate_r0_from_pulse(data: &[(f64, f64, f64)]) -> f64 {
    // Find first current step
    for i in 1..data.len() {
        let di = (data[i].1 - data[i - 1].1).abs();
        if di > 0.1 {
            let dv = (data[i].2 - data[i - 1].2).abs();
            if di > 1e-9 {
                return dv / di;
            }
        }
    }
    0.02 // fallback
}

fn estimate_rc_from_relaxation(data: &[(f64, f64, f64)]) -> (f64, f64, f64, f64) {
    // Find the POST-pulse relaxation segment. The `pulse_seen` flag ensures we
    // skip any leading pre-pulse rest samples — otherwise the heuristic would
    // operate on idle data instead of actual RC decay.
    let mut pulse_seen = false;
    let mut rest_start_opt: Option<usize> = None;
    for (idx, &(_, i, _)) in data.iter().enumerate() {
        if i.abs() > 0.5 {
            pulse_seen = true;
        } else if pulse_seen && i.abs() < 0.01 {
            rest_start_opt = Some(idx);
            break;
        }
    }
    let rest_start = match rest_start_opt {
        Some(rs) if rs + 5 < data.len() => rs,
        _ => return (0.015, 3000.0, 0.010, 500.0),
    };

    let v0 = data[rest_start].2;
    let v_inf = data[data.len() - 1].2;
    let dv = (v_inf - v0).abs().max(1e-12);

    // Pulse current magnitude (from last active sample before rest).
    let i_pulse = if rest_start > 0 {
        data[rest_start - 1].1.abs().max(1e-9)
    } else {
        1.0
    };

    // Estimate effective time constant from a log-decay over the rest window.
    // v(t) ≈ v_inf - dv·exp(-t/τ_eff). Use ¼-point of the rest window.
    let rest_end = data.len() - 1;
    let t_rest_start = data[rest_start].0;
    let quarter_idx = rest_start + ((rest_end - rest_start) / 4).max(1);
    let v_q = data[quarter_idx.min(rest_end)].2;
    let t_q = data[quarter_idx.min(rest_end)].0;
    let decay_q = ((v_inf - v_q) / dv).clamp(1e-9, 1.0 - 1e-9);
    let tau_eff = (-(t_q - t_rest_start) / decay_q.ln()).abs().max(1e-3);

    // Split into two RC pairs: slow (τ1 = 1.5 × τ_eff) and fast (τ2 = 0.4 × τ_eff).
    // The R values are split equally from the total RC voltage drop.
    let tau1 = tau_eff * 1.5;
    let tau2 = (tau_eff * 0.4).max(1e-3);
    let r_total = dv / i_pulse;
    let r1 = (r_total * 0.5).max(1e-9);
    let r2 = (r_total * 0.5).max(1e-9);
    let c1 = tau1 / r1;
    let c2 = tau2 / r2;

    (r1, c1, r2, c2)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_kokam_parameters() {
        let p = ParameterSet::kokam_75ah_lfp();
        assert!(p.r0 > 0.0);
        assert!(p.capacity_ah > 0.0);
        assert!(p.c1 > 0.0);
        assert!(p.temp_coeffs.is_some());
    }

    #[test]
    fn test_nmc_parameters() {
        let p = ParameterSet::nmc_3ah();
        assert!((p.capacity_ah - 3.0).abs() < 1e-9);
    }

    #[test]
    fn ecm_fit_recovers_synthetic_lfp_within_tolerance() {
        let r0_true = 0.001f64;
        let r1_true = 0.002f64;
        let c1_true = 5000.0f64;
        let r2_true = 0.003f64;
        let c2_true = 2000.0f64;
        let ocv = 3.6f64;
        let dt = 0.1f64;
        let i_load = 10.0f64;

        let mut data = Vec::new();

        // Pre-pulse rest (20 samples, I=0): provides OCV identification.
        for k in 0..20usize {
            let t = (k as f64 - 20.0) * dt; // t = -2.0, -1.9, ..., -0.1
            data.push((t, 0.0, ocv));
        }

        // Pulse discharge (200 samples): current step at t=0 identifies R0.
        let (mut v_rc1, mut v_rc2) = (0.0f64, 0.0f64);
        for k in 0..200usize {
            let t = k as f64 * dt;
            v_rc1 += dt * (i_load / c1_true - v_rc1 / (r1_true * c1_true));
            v_rc2 += dt * (i_load / c2_true - v_rc2 / (r2_true * c2_true));
            let v_meas = ocv - i_load * r0_true - v_rc1 - v_rc2;
            data.push((t, i_load, v_meas));
        }

        // Post-pulse rest (100 samples = 10s ≈ 1×τ1): sufficient to observe
        // RC decay for identifiable parameter combinations.
        let t_pulse_end = 200.0 * dt;
        for k in 0..100usize {
            let t = t_pulse_end + k as f64 * dt;
            v_rc1 += dt * (0.0_f64 / c1_true - v_rc1 / (r1_true * c1_true));
            v_rc2 += dt * (0.0_f64 / c2_true - v_rc2 / (r2_true * c2_true));
            let v_meas = ocv - v_rc1 - v_rc2;
            data.push((t, 0.0, v_meas));
        }

        let params = ParameterSet::fit_from_pulse_data(&data, 75.0)
            .expect("fit_from_pulse_data should succeed on clean synthetic data");

        // 2RC pairs with τ1/τ2 ≈ 10s/6s ≈ 1.67 are weakly separated; individual
        // R1/C1 lie along a τ1=R1·C1 hyperbola with near-flat loss. We therefore
        // test only the physically identifiable combinations: R0, τ1, and R1+R2.
        assert!(
            (params.r0 - r0_true).abs() / r0_true < 0.15,
            "R0: got {}, expected {} (±15%)",
            params.r0,
            r0_true
        );
        let tau1_fit = params.r1 * params.c1;
        let tau1_true = r1_true * c1_true;
        assert!(
            (tau1_fit - tau1_true).abs() / tau1_true < 0.20,
            "τ1=R1·C1: got {}, expected {} (±20%)",
            tau1_fit,
            tau1_true
        );
        let r_sum_fit = params.r1 + params.r2;
        let r_sum_true = r1_true + r2_true;
        assert!(
            (r_sum_fit - r_sum_true).abs() / r_sum_true < 0.20,
            "R1+R2: got {}, expected {} (±20%)",
            r_sum_fit,
            r_sum_true
        );
    }

    #[test]
    fn ecm_fit_better_than_heuristic_initial() {
        let r0_true = 0.001f64;
        let r1_true = 0.002f64;
        let c1_true = 5000.0f64;
        let r2_true = 0.003f64;
        let c2_true = 2000.0f64;
        let ocv = 3.6f64;
        let dt = 0.1f64;
        let i_load = 10.0f64;

        let mut data = Vec::new();

        // Pre-pulse rest (20 samples, I=0)
        for k in 0..20usize {
            let t = (k as f64 - 20.0) * dt;
            data.push((t, 0.0, ocv));
        }

        // Pulse discharge (200 samples)
        let (mut v_rc1, mut v_rc2) = (0.0f64, 0.0f64);
        for k in 0..200usize {
            let t = k as f64 * dt;
            v_rc1 += dt * (i_load / c1_true - v_rc1 / (r1_true * c1_true));
            v_rc2 += dt * (i_load / c2_true - v_rc2 / (r2_true * c2_true));
            let v_meas = ocv - i_load * r0_true - v_rc1 - v_rc2;
            data.push((t, i_load, v_meas));
        }

        // Post-pulse rest (100 samples = 10s ≈ 1×τ1): sufficient for heuristic
        // warm-start and loss-improvement verification.
        let t_pulse_end = 200.0 * dt;
        for k in 0..100usize {
            let t = t_pulse_end + k as f64 * dt;
            v_rc1 += dt * (0.0_f64 / c1_true - v_rc1 / (r1_true * c1_true));
            v_rc2 += dt * (0.0_f64 / c2_true - v_rc2 / (r2_true * c2_true));
            let v_meas = ocv - v_rc1 - v_rc2;
            data.push((t, 0.0, v_meas));
        }

        // Measure the loss at the heuristic initial point
        let r0_h = estimate_r0_from_pulse(&data);
        let (r1_h, c1_h, r2_h, c2_h) = estimate_rc_from_relaxation(&data);
        let initial_log = [r0_h.ln(), r1_h.ln(), c1_h.ln(), r2_h.ln(), c2_h.ln()];
        let initial_loss = ecm_simulate_loss(&data, &initial_log);

        let params = ParameterSet::fit_from_pulse_data(&data, 75.0).expect("fit should succeed");
        let final_log = [
            params.r0.ln(),
            params.r1.ln(),
            params.c1.ln(),
            params.r2.ln(),
            params.c2.ln(),
        ];
        let final_loss = ecm_simulate_loss(&data, &final_log);

        assert!(
            final_loss < initial_loss * 0.5,
            "L-BFGS final loss {} should be < 50% of initial heuristic loss {}",
            final_loss,
            initial_loss
        );
    }
}
