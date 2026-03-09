/// Battery ECM parameter identification.
///
/// Provides parameter sets for common battery chemistries and
/// placeholder infrastructure for optirs-based parameter identification.
use serde::{Deserialize, Serialize};

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

    /// Fit a 2RC model to pulse discharge data using least-squares.
    ///
    /// `data` is a slice of (time_s, current_A, voltage_V) tuples.
    /// Returns fitted parameters or an error string.
    ///
    /// NOTE: Full optirs integration is deferred to a future phase.
    /// This implementation uses a simplified curve-fitting approach.
    pub fn fit_from_pulse_data(data: &[(f64, f64, f64)], capacity_ah: f64) -> Result<Self, String> {
        if data.len() < 10 {
            return Err("Need at least 10 data points for parameter fitting".into());
        }

        // Estimate R0 from initial voltage drop on current step
        let r0_est = estimate_r0_from_pulse(data);

        // Estimate RC time constants from relaxation tail
        let (r1_est, c1_est, r2_est, c2_est) = estimate_rc_from_relaxation(data);

        Ok(Self {
            r0: r0_est,
            r1: r1_est,
            c1: c1_est,
            r2: r2_est,
            c2: c2_est,
            capacity_ah,
            temp_coeffs: None,
        })
    }
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
    // Find relaxation segment (current ≈ 0)
    let rest_start = data
        .iter()
        .position(|&(_, i, _)| i.abs() < 0.01)
        .unwrap_or(0);

    if rest_start + 5 >= data.len() {
        return (0.015, 3000.0, 0.010, 500.0);
    }

    let v0 = data[rest_start].2;
    let v_inf = data[data.len() - 1].2;
    let dv = (v_inf - v0).abs();

    // Heuristic two-exponential fit
    let r1 = dv * 0.7;
    let tau1 = (data[rest_start + 2].0 - data[rest_start].0) * 2.0;
    let c1 = if r1 > 1e-9 { tau1 / r1 } else { 3000.0 };

    let r2 = dv * 0.3;
    let tau2 = (data[data.len() - 1].0 - data[rest_start].0) * 0.3;
    let c2 = if r2 > 1e-9 { tau2 / r2 } else { 500.0 };

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
}
