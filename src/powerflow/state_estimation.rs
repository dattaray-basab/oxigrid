/// Weighted Least Squares (WLS) power system state estimation.
///
/// The state estimation problem:
///
///   z = h(x) + e
///
/// where z is the measurement vector, x is the state vector (bus voltage
/// angles and magnitudes), h(x) is the nonlinear measurement function,
/// and e is Gaussian noise with covariance R = diag(σ²).
///
/// WLS minimises J(x) = (z − h(x))ᵀ R⁻¹ (z − h(x)).
///
/// Solution via Gauss-Newton iterations:
///   Gx = Hᵀ W H  (gain matrix)
///   Δx = Gx⁻¹ Hᵀ W (z − h(x))
///   x ← x + Δx
///
/// # DC State Estimation (linear)
/// For DC power flow approximation the measurement equation is linear:
///   z_P = B' · θ + e
///
/// WLS closed-form: θ = (Hᵀ W H)⁻¹ Hᵀ W z
///
/// # Reference
/// Abur & Expósito, "Power System State Estimation: Theory and Implementation",
/// Marcel Dekker, 2004.
use crate::error::{OxiGridError, Result};
use nalgebra::{DMatrix, DVector};
use serde::{Deserialize, Serialize};

/// Type of power system measurement.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum MeasurementType {
    /// Active power injection at a bus [p.u.]
    PowerInjection,
    /// Reactive power injection at a bus [p.u.]
    ReactiveInjection,
    /// Voltage magnitude at a bus [p.u.]
    VoltageMagnitude,
    /// Active power flow on a branch (from-bus side) [p.u.]
    BranchActivePower,
    /// Reactive power flow on a branch (from-bus side) [p.u.]
    BranchReactivePower,
}

/// A single measurement with its noise model.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Measurement {
    /// Measurement type
    pub mtype: MeasurementType,
    /// Bus index (for injection/voltage) or from-bus index (for flow)
    pub bus: usize,
    /// To-bus index (only used for BranchActivePower / BranchReactivePower)
    pub to_bus: Option<usize>,
    /// Measured value [p.u.]
    pub value: f64,
    /// Standard deviation of measurement noise σ [p.u.]
    pub sigma: f64,
}

impl Measurement {
    /// Measurement weight W = 1/σ².
    pub fn weight(&self) -> f64 {
        1.0 / (self.sigma * self.sigma)
    }

    /// Power injection at given bus.
    pub fn power_injection(bus: usize, value: f64, sigma: f64) -> Self {
        Self {
            mtype: MeasurementType::PowerInjection,
            bus,
            to_bus: None,
            value,
            sigma,
        }
    }

    /// Voltage magnitude at given bus.
    pub fn voltage(bus: usize, value: f64, sigma: f64) -> Self {
        Self {
            mtype: MeasurementType::VoltageMagnitude,
            bus,
            to_bus: None,
            value,
            sigma,
        }
    }

    /// Branch active power flow from `from_bus` to `to_bus`.
    pub fn branch_flow(from_bus: usize, to_bus: usize, value: f64, sigma: f64) -> Self {
        Self {
            mtype: MeasurementType::BranchActivePower,
            bus: from_bus,
            to_bus: Some(to_bus),
            value,
            sigma,
        }
    }
}

/// DC state estimation result.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DcSeResult {
    /// Estimated bus voltage angles θ `rad`
    pub theta: Vec<f64>,
    /// Measurement residuals z − H·θ
    pub residuals: Vec<f64>,
    /// Chi-squared statistic for bad-data detection
    pub chi2: f64,
    /// Degrees of freedom = n_measurements − n_states
    pub dof: usize,
    /// True if the system was solvable
    pub converged: bool,
}

impl DcSeResult {
    /// Normalised chi-squared (chi2 / dof). Values >> 1 indicate bad data.
    pub fn normalised_chi2(&self) -> f64 {
        if self.dof == 0 {
            return 0.0;
        }
        self.chi2 / self.dof as f64
    }
}

/// DC Weighted Least Squares state estimator.
///
/// Uses the DC power flow approximation: P = B' · θ.
/// Supports bus power injection and branch flow measurements.
pub struct DcStateEstimator {
    /// Number of buses
    pub n_bus: usize,
    /// Slack bus index (angle fixed to 0)
    pub slack_idx: usize,
    /// B' matrix (n × n DC susceptance)
    pub b_bus: Vec<Vec<f64>>,
    /// Branch from-bus indices
    pub branch_from: Vec<usize>,
    /// Branch to-bus indices
    pub branch_to: Vec<usize>,
    /// Branch reactances [p.u.]
    pub branch_x: Vec<f64>,
}

impl DcStateEstimator {
    /// Create estimator from network data.
    pub fn new(
        n_bus: usize,
        slack_idx: usize,
        b_bus: Vec<Vec<f64>>,
        branch_from: Vec<usize>,
        branch_to: Vec<usize>,
        branch_x: Vec<f64>,
    ) -> Self {
        Self {
            n_bus,
            slack_idx,
            b_bus,
            branch_from,
            branch_to,
            branch_x,
        }
    }

    /// Solve the DC WLS state estimation problem.
    ///
    /// Returns estimated angles and diagnostics.
    pub fn estimate(&self, measurements: &[Measurement]) -> Result<DcSeResult> {
        let n_red = self.n_bus - 1;
        let nm = measurements.len();

        if nm < n_red {
            return Err(OxiGridError::InvalidNetwork(format!(
                "Under-determined: {nm} measurements for {n_red} states"
            )));
        }

        // Bus index mapping (slack removed)
        let bus_map: Vec<usize> = (0..self.n_bus).filter(|&i| i != self.slack_idx).collect();

        // Build H matrix (nm × n_red) and z, W vectors
        let mut h_mat = DMatrix::<f64>::zeros(nm, n_red);
        let mut z_vec = DVector::<f64>::zeros(nm);
        let mut w_diag = DVector::<f64>::zeros(nm);

        for (mi, meas) in measurements.iter().enumerate() {
            z_vec[mi] = meas.value;
            w_diag[mi] = meas.weight();

            match meas.mtype {
                MeasurementType::PowerInjection => {
                    // P_i = Σ_j B_ij * θ_j  (sum over non-slack j)
                    let bus = meas.bus;
                    for (ri, &j) in bus_map.iter().enumerate() {
                        h_mat[(mi, ri)] = self.b_bus[bus][j];
                    }
                }
                MeasurementType::BranchActivePower => {
                    // P_lk = (θ_from − θ_to) / x_l
                    let from = meas.bus;
                    let to = meas.to_bus.unwrap_or(0);
                    let br_idx = self
                        .branch_from
                        .iter()
                        .zip(self.branch_to.iter())
                        .position(|(&f, &t)| f == from && t == to)
                        .ok_or_else(|| {
                            OxiGridError::InvalidNetwork(format!("Branch {from}→{to} not found"))
                        })?;
                    let x_l = self.branch_x[br_idx];
                    if let Some(fi) = bus_map.iter().position(|&b| b == from) {
                        h_mat[(mi, fi)] = 1.0 / x_l;
                    }
                    if let Some(ti) = bus_map.iter().position(|&b| b == to) {
                        h_mat[(mi, ti)] = -1.0 / x_l;
                    }
                }
                // DC estimator ignores reactive/voltage measurements
                _ => {}
            }
        }

        // WLS: Gain G = Hᵀ W H
        let w_mat = DMatrix::<f64>::from_diagonal(&w_diag);
        let ht = h_mat.transpose();
        let g = &ht * &w_mat * &h_mat;
        let rhs = &ht * &w_mat * &z_vec;

        let lu = g.clone().lu();
        let theta_red = lu
            .solve(&rhs)
            .ok_or_else(|| OxiGridError::LinearAlgebra("Gain matrix G is singular".into()))?;

        // Reconstruct full angle vector
        let mut theta = vec![0.0_f64; self.n_bus];
        for (ri, &i) in bus_map.iter().enumerate() {
            theta[i] = theta_red[ri];
        }

        // Compute residuals and chi-squared
        let h_theta = &h_mat * &theta_red;
        let residuals: Vec<f64> = (0..nm).map(|i| z_vec[i] - h_theta[i]).collect();

        let chi2: f64 = residuals
            .iter()
            .zip(w_diag.iter())
            .map(|(r, &w)| r * r * w)
            .sum();

        let dof = nm.saturating_sub(n_red);

        Ok(DcSeResult {
            theta,
            residuals,
            chi2,
            dof,
            converged: true,
        })
    }
}

/// Detect bad data using the Largest Normalised Residual (LNR) test.
///
/// A measurement is flagged if its normalised residual exceeds `threshold`
/// (commonly 3.0 for a 3σ test).
pub fn detect_bad_data(
    residuals: &[f64],
    measurements: &[Measurement],
    threshold: f64,
) -> Vec<usize> {
    residuals
        .iter()
        .zip(measurements.iter())
        .enumerate()
        .filter_map(|(i, (r, m))| {
            let normalised = r.abs() / m.sigma;
            if normalised > threshold {
                Some(i)
            } else {
                None
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn simple_3bus_estimator() -> DcStateEstimator {
        // 3-bus network: 0(slack)-1 (x=0.1), 1-2 (x=0.2)
        let b_bus = vec![
            vec![10.0, -10.0, 0.0],
            vec![-10.0, 15.0, -5.0],
            vec![0.0, -5.0, 5.0],
        ];
        DcStateEstimator::new(3, 0, b_bus, vec![0, 1], vec![1, 2], vec![0.1, 0.2])
    }

    #[test]
    fn test_dc_se_exact_measurements() {
        let est = simple_3bus_estimator();
        // True angles: θ = [0, -0.05, -0.15] rad
        // P_inj: bus1 = B[1,:]*θ = -10*0 + 15*(-0.05) + (-5)*(-0.15) = -0.75 + 0.75 = 0.0?
        // Let's use branch flow measurements instead
        // P_01 = (θ0 - θ1)/x01 = (0 - (-0.05))/0.1 = 0.5 pu
        // P_12 = (θ1 - θ2)/x12 = ((-0.05) - (-0.15))/0.2 = 0.5 pu
        let meas = vec![
            Measurement::branch_flow(0, 1, 0.5, 0.01),
            Measurement::branch_flow(1, 2, 0.5, 0.01),
            Measurement::power_injection(1, 0.0, 0.01), // bus 1: balanced
        ];
        let result = est.estimate(&meas).unwrap();
        assert!(result.converged);
        assert!(
            result.theta[0].abs() < 1e-9,
            "slack angle ≠ 0: {}",
            result.theta[0]
        );
        assert!(
            (result.theta[1] - (-0.05)).abs() < 1e-4,
            "θ1 = {:.4}, expected -0.05",
            result.theta[1]
        );
        assert!(
            (result.theta[2] - (-0.15)).abs() < 1e-4,
            "θ2 = {:.4}, expected -0.15",
            result.theta[2]
        );
    }

    #[test]
    fn test_dc_se_redundant_measurements() {
        let est = simple_3bus_estimator();
        // Overdetermined: 4 measurements for 2 states
        let meas = vec![
            Measurement::branch_flow(0, 1, 0.5, 0.01),
            Measurement::branch_flow(1, 2, 0.5, 0.01),
            Measurement::power_injection(1, 0.0, 0.01),
            Measurement::power_injection(2, -0.5, 0.01),
        ];
        let result = est.estimate(&meas).unwrap();
        assert!(result.converged);
        assert!(result.dof == 2); // 4 meas - 2 states
    }

    #[test]
    fn test_bad_data_detection() {
        let meas = vec![
            Measurement::power_injection(0, 0.5, 0.01),
            Measurement::power_injection(1, 100.0, 0.01), // bad: 10000σ error
        ];
        let residuals = [0.001, 1.0]; // normalised: 0.1, 100
        let bad = detect_bad_data(&residuals, &meas, 3.0);
        assert_eq!(bad, vec![1]);
    }

    #[test]
    fn test_measurement_weight() {
        let m = Measurement::power_injection(0, 0.5, 0.02);
        assert!((m.weight() - 2500.0).abs() < 1e-6); // 1/0.02^2
    }

    #[test]
    fn test_under_determined_returns_error() {
        let est = simple_3bus_estimator();
        // Only 1 measurement for 2 states
        let meas = vec![Measurement::branch_flow(0, 1, 0.5, 0.01)];
        assert!(est.estimate(&meas).is_err());
    }
}
