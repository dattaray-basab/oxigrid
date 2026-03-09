//! Bad Data Processing for Power System State Estimation.
//!
//! Implements the standard pipeline for detecting and identifying bad
//! measurements in a Weighted Least Squares (WLS) state estimator:
//!
//! 1. **Chi-squared test** — overall goodness-of-fit: `J(x) = rᵀ W r ~ χ²(m−n)`
//! 2. **Largest Normalized Residual (LNR) test** — identify the single most
//!    suspicious measurement by comparing `|rᵢ / √Ωᵢᵢ|` against a threshold.
//! 3. **Hat-matrix diagonal** — leverage analysis: measurements with high
//!    `Sᵢᵢ` (diagonal of `H(HᵀWH)⁻¹HᵀW`) exert large influence on the
//!    estimated state and may mask bad data even with small residuals.
//!
//! # Algorithm
//!
//! Given residuals `r = z − h(x̂)`, measurement Jacobian `H`, and diagonal
//! weight matrix `W = diag(1/σᵢ²)`:
//!
//! ```text
//! Gain matrix:      G = HᵀWH                     [n×n]
//! Sensitivity:      S = H G⁻¹ Hᵀ W               [m×m, diagonal extracted]
//! Residual cov:     Ω = W⁻¹ − H G⁻¹ Hᵀ          [m×m, diagonal extracted]
//! Normalized res.:  rN_i = rᵢ / √Ωᵢᵢ
//! Chi-squared stat: J = rᵀ W r
//! ```
//!
//! # References
//! - Abur & Expósito, *Power System State Estimation: Theory and Implementation*,
//!   Marcel Dekker, 2004.
//! - Schweppe & Wildes, "Power system static state estimation", IEEE Trans. PAS 1970.

use serde::{Deserialize, Serialize};
use thiserror::Error;

// ── Error ─────────────────────────────────────────────────────────────────────

/// Errors produced by the bad-data processor.
#[derive(Debug, Error)]
pub enum BadDataError {
    /// Measurement vector and H matrix row count are inconsistent.
    #[error("measurement count {meas} does not match H matrix rows {h_rows}")]
    DimensionMismatch { meas: usize, h_rows: usize },

    /// Weight vector length does not match measurement count.
    #[error("weight vector length {w_len} does not match measurement count {meas}")]
    WeightLengthMismatch { w_len: usize, meas: usize },

    /// H matrix is empty or has no columns (no state variables).
    #[error("H matrix has no columns (no state variables)")]
    EmptyStateSpace,

    /// Gain matrix G = HᵀWH is singular (system under-determined).
    #[error("gain matrix G = HᵀWH is singular or nearly singular")]
    SingularGainMatrix,

    /// Fewer measurements than state variables — system is under-determined.
    #[error("under-determined system: {meas} measurements < {states} states")]
    Underdetermined { meas: usize, states: usize },

    /// Numerical issue in residual covariance computation.
    #[error("numerical error: {0}")]
    Numerical(String),
}

// ── Configuration ─────────────────────────────────────────────────────────────

/// Configuration for the bad-data detection pipeline.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BadDataConfig {
    /// Chi-squared test threshold (e.g. 3.84 at 95 % confidence for 1 DOF).
    ///
    /// The overall objective function `J(x) = rᵀWr` is compared against
    /// `chi2_threshold * (m − n)` where `m` is measurement count and `n` is
    /// state count.
    pub chi2_threshold: f64,
    /// Largest Normalized Residual detection threshold (typically 3.0).
    pub lnr_threshold: f64,
    /// Hypothesis test significance level α (e.g. 0.05 for 95 %).
    pub hypothesis_test_alpha: f64,
    /// Maximum number of bad measurements to flag per iteration.
    pub max_bad_data: usize,
    /// Hat-matrix diagonal threshold for leverage point detection (e.g. 0.5).
    pub leverage_threshold: f64,
}

impl Default for BadDataConfig {
    fn default() -> Self {
        Self {
            chi2_threshold: 3.84,
            lnr_threshold: 3.0,
            hypothesis_test_alpha: 0.05,
            max_bad_data: 5,
            leverage_threshold: 0.5,
        }
    }
}

// ── Measurement ───────────────────────────────────────────────────────────────

/// A single metered measurement with its estimated value and residual.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Measurement {
    /// Unique measurement identifier.
    pub id: usize,
    /// Measured value `z` (in per-unit, MW, kV, etc.).
    pub value: f64,
    /// Standard deviation `σ` of the measurement noise.
    pub std_dev: f64,
    /// Estimated value `h(x̂)` from the state estimator.
    pub estimated: f64,
    /// Residual `r = z − h(x̂)`.
    pub residual: f64,
}

impl Measurement {
    /// Weight `wᵢ = 1 / σᵢ²` for WLS.
    pub fn weight(&self) -> f64 {
        if self.std_dev.abs() < f64::EPSILON {
            0.0
        } else {
            1.0 / (self.std_dev * self.std_dev)
        }
    }
}

// ── Actions ───────────────────────────────────────────────────────────────────

/// Recommended corrective action for a suspicious measurement.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum BadDataAction {
    /// Remove the measurement from the estimator.
    Exclude {
        /// Measurement identifier.
        measurement_id: usize,
    },
    /// Replace with an interpolated or default substitute.
    Replace {
        /// Measurement identifier.
        measurement_id: usize,
        /// Substitute value to use.
        substitute_value: f64,
    },
    /// Flag for manual investigation.
    Investigate {
        /// Measurement identifier.
        measurement_id: usize,
        /// Human-readable reason.
        reason: String,
    },
    /// Measurement is accepted as valid.
    Accept,
}

// ── Result ────────────────────────────────────────────────────────────────────

/// Output of the bad-data detection pipeline.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BadDataResult {
    /// Indices of measurements suspected as bad (into the input slice).
    pub suspected_bad: Vec<usize>,
    /// Normalized residual `rNᵢ = rᵢ / √Ωᵢᵢ` for every measurement.
    pub normalized_residuals: Vec<f64>,
    /// Diagonal of the hat (sensitivity) matrix `Sᵢᵢ` for every measurement.
    pub sensitivity_matrix_diagonal: Vec<f64>,
    /// Chi-squared objective value `J = rᵀWr`.
    pub overall_chi2: f64,
    /// Whether the chi-squared test passed (J within acceptable range).
    pub chi2_test_passed: bool,
    /// Indices of high-leverage measurements (Sᵢᵢ > leverage_threshold).
    pub leverage_points: Vec<usize>,
    /// Recommended action for each measurement (same length as input).
    pub recommended_action: Vec<BadDataAction>,
}

// ── Processor ─────────────────────────────────────────────────────────────────

/// Bad-data processor implementing LNR test, chi-squared test, and hat-matrix analysis.
pub struct BadDataProcessor {
    config: BadDataConfig,
}

impl BadDataProcessor {
    /// Create a new processor with the given configuration.
    pub fn new(config: BadDataConfig) -> Self {
        Self { config }
    }

    /// Run the full bad-data detection pipeline.
    ///
    /// # Arguments
    /// * `measurements` — slice of measurements with residuals already computed.
    /// * `h_matrix` — Jacobian `H` as row-major `[m][n]` (m measurements, n states).
    /// * `weight_matrix_diag` — diagonal of `W = R⁻¹`, length m.
    pub fn process(
        &self,
        measurements: &[Measurement],
        h_matrix: &[Vec<f64>],
        weight_matrix_diag: &[f64],
    ) -> Result<BadDataResult, BadDataError> {
        let m = measurements.len();

        if h_matrix.len() != m {
            return Err(BadDataError::DimensionMismatch {
                meas: m,
                h_rows: h_matrix.len(),
            });
        }
        if weight_matrix_diag.len() != m {
            return Err(BadDataError::WeightLengthMismatch {
                w_len: weight_matrix_diag.len(),
                meas: m,
            });
        }
        if m == 0 {
            return Err(BadDataError::EmptyStateSpace);
        }

        let n = h_matrix[0].len();
        if n == 0 {
            return Err(BadDataError::EmptyStateSpace);
        }
        if m < n {
            return Err(BadDataError::Underdetermined { meas: m, states: n });
        }

        // Extract residuals
        let residuals: Vec<f64> = measurements.iter().map(|m| m.residual).collect();

        // Chi-squared test
        let (chi2, chi2_passed) = self.chi2_test(&residuals, weight_matrix_diag, n);

        // Hat matrix diagonal and residual covariance diagonal
        let s_diag = self.hat_matrix_diagonal(h_matrix, weight_matrix_diag)?;

        // Residual covariance diagonal: Ωᵢᵢ = 1/wᵢ − Sᵢᵢ/wᵢ = (1 − Sᵢᵢ)/wᵢ
        let omega_diag: Vec<f64> = (0..m)
            .map(|i| {
                let w = weight_matrix_diag[i];
                if w > f64::EPSILON {
                    (1.0 - s_diag[i]) / w
                } else {
                    0.0
                }
            })
            .collect();

        // Normalized residuals
        let normalized = self.normalized_residuals(&residuals, &omega_diag);

        // LNR test — iterative flagging up to max_bad_data
        let mut suspected_bad = Vec::new();
        let mut excluded = vec![false; m];
        for _ in 0..self.config.max_bad_data {
            // Work on non-excluded measurements
            let subset_normalized: Vec<(usize, f64)> = normalized
                .iter()
                .enumerate()
                .filter(|(i, _)| !excluded[*i])
                .map(|(i, &v)| (i, v))
                .collect();
            match self.lnr_test_indexed(&subset_normalized) {
                Some(idx) => {
                    suspected_bad.push(idx);
                    excluded[idx] = true;
                }
                None => break,
            }
        }

        // Leverage points
        let leverage_points: Vec<usize> = s_diag
            .iter()
            .enumerate()
            .filter(|(_, &s)| s > self.config.leverage_threshold)
            .map(|(i, _)| i)
            .collect();

        // Recommended actions
        let recommended_action =
            self.recommend_actions(measurements, &suspected_bad, &normalized, &s_diag);

        Ok(BadDataResult {
            suspected_bad,
            normalized_residuals: normalized,
            sensitivity_matrix_diagonal: s_diag,
            overall_chi2: chi2,
            chi2_test_passed: chi2_passed,
            leverage_points,
            recommended_action,
        })
    }

    /// Compute normalized residuals: `rNᵢ = rᵢ / √max(Ωᵢᵢ, ε)`.
    fn normalized_residuals(&self, residuals: &[f64], omega_diag: &[f64]) -> Vec<f64> {
        residuals
            .iter()
            .zip(omega_diag.iter())
            .map(|(&r, &omega)| {
                let denom = omega.max(0.0).sqrt();
                if denom < f64::EPSILON {
                    r.abs() // fallback: no normalization if omega ~ 0
                } else {
                    r / denom
                }
            })
            .collect()
    }

    /// LNR test on a pre-filtered indexed list.
    ///
    /// Returns the global index of the measurement with the largest `|rN|`
    /// exceeding `lnr_threshold`, or `None` if no measurement exceeds the threshold.
    fn lnr_test_indexed(&self, indexed: &[(usize, f64)]) -> Option<usize> {
        indexed
            .iter()
            .max_by(|a, b| {
                a.1.abs()
                    .partial_cmp(&b.1.abs())
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .and_then(|(idx, rn)| {
                if rn.abs() > self.config.lnr_threshold {
                    Some(*idx)
                } else {
                    None
                }
            })
    }

    /// LNR test on the full normalized residual vector.
    ///
    /// Returns the index of the measurement with `|rN|` exceeding the threshold,
    /// or `None` if all measurements pass.
    pub fn lnr_test(&self, normalized: &[f64]) -> Option<usize> {
        let indexed: Vec<(usize, f64)> = normalized.iter().cloned().enumerate().collect();
        self.lnr_test_indexed(&indexed)
    }

    /// Chi-squared test: `J(x) = rᵀWr ~ χ²(m−n)`.
    ///
    /// Returns `(J, passed)` where `passed` is true if `J ≤ chi2_threshold * (m−n)`.
    pub fn chi2_test(&self, residuals: &[f64], weights: &[f64], n_states: usize) -> (f64, bool) {
        let j: f64 = residuals
            .iter()
            .zip(weights.iter())
            .map(|(&r, &w)| r * r * w)
            .sum();
        let dof = residuals.len().saturating_sub(n_states) as f64;
        let threshold = self.config.chi2_threshold * dof.max(1.0);
        (j, j <= threshold)
    }

    /// Compute the diagonal of the hat (sensitivity) matrix `S = H(HᵀWH)⁻¹HᵀW`.
    ///
    /// Uses the formula `Sᵢᵢ = hᵢᵀ G⁻¹ hᵢ wᵢ` where `G = HᵀWH`.
    pub fn hat_matrix_diagonal(&self, h: &[Vec<f64>], w: &[f64]) -> Result<Vec<f64>, BadDataError> {
        let m = h.len();
        if m == 0 {
            return Ok(Vec::new());
        }
        let n = h[0].len();
        if n == 0 {
            return Err(BadDataError::EmptyStateSpace);
        }

        // Build G = HᵀWH [n×n]
        let mut g = vec![vec![0.0f64; n]; n];
        for (i, row) in h.iter().enumerate() {
            let wi = w[i];
            for j in 0..n {
                for k in 0..n {
                    g[j][k] += wi * row[j] * row[k];
                }
            }
        }

        // Invert G using Gaussian elimination with partial pivoting
        let g_inv = invert_matrix(&g).ok_or(BadDataError::SingularGainMatrix)?;

        // Sᵢᵢ = wᵢ * hᵢᵀ G⁻¹ hᵢ
        let s_diag: Vec<f64> = h
            .iter()
            .enumerate()
            .map(|(i, row)| {
                let wi = w[i];
                let mut quad = 0.0f64;
                for j in 0..n {
                    let mut g_inv_h_j = 0.0f64;
                    for k in 0..n {
                        g_inv_h_j += g_inv[j][k] * row[k];
                    }
                    quad += row[j] * g_inv_h_j;
                }
                (wi * quad).clamp(0.0, 1.0)
            })
            .collect();

        Ok(s_diag)
    }

    /// Generate recommended actions based on LNR and leverage analysis.
    fn recommend_actions(
        &self,
        measurements: &[Measurement],
        suspected_bad: &[usize],
        normalized: &[f64],
        s_diag: &[f64],
    ) -> Vec<BadDataAction> {
        measurements
            .iter()
            .enumerate()
            .map(|(i, meas)| {
                let is_bad = suspected_bad.contains(&i);
                let rn = normalized.get(i).copied().unwrap_or(0.0).abs();
                let s = s_diag.get(i).copied().unwrap_or(0.0);

                if is_bad && rn > self.config.lnr_threshold * 2.0 {
                    BadDataAction::Exclude {
                        measurement_id: meas.id,
                    }
                } else if is_bad && s > self.config.leverage_threshold {
                    BadDataAction::Investigate {
                        measurement_id: meas.id,
                        reason: format!(
                            "High leverage (S={s:.3}) with normalized residual {rn:.3}"
                        ),
                    }
                } else if is_bad {
                    BadDataAction::Replace {
                        measurement_id: meas.id,
                        substitute_value: meas.estimated,
                    }
                } else {
                    BadDataAction::Accept
                }
            })
            .collect()
    }
}

// ── Linear algebra helper ─────────────────────────────────────────────────────

/// Invert an `n×n` matrix using Gaussian elimination with partial pivoting.
///
/// Returns `None` if the matrix is singular (pivot < `1e-14`).
fn invert_matrix(a: &[Vec<f64>]) -> Option<Vec<Vec<f64>>> {
    let n = a.len();
    // Build augmented matrix [A | I]
    let mut aug: Vec<Vec<f64>> = (0..n)
        .map(|i| {
            let mut row = a[i].clone();
            for j in 0..n {
                row.push(if i == j { 1.0 } else { 0.0 });
            }
            row
        })
        .collect();

    for col in 0..n {
        // Partial pivoting
        let pivot_row = (col..n).max_by(|&r1, &r2| {
            aug[r1][col]
                .abs()
                .partial_cmp(&aug[r2][col].abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        })?;
        aug.swap(col, pivot_row);

        let pivot = aug[col][col];
        if pivot.abs() < 1e-14 {
            return None;
        }

        // Scale pivot row
        let pivot_row_ref = &mut aug[col];
        for val in pivot_row_ref.iter_mut() {
            *val /= pivot;
        }

        // Eliminate column
        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = aug[row][col];
            // Clone pivot row to avoid borrow conflict
            let pivot_row_clone: Vec<f64> = aug[col].clone();
            for (j, pv) in pivot_row_clone.iter().enumerate() {
                aug[row][j] -= factor * pv;
            }
        }
    }

    // Extract right half
    Some(aug.into_iter().map(|row| row[n..].to_vec()).collect())
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_config() -> BadDataConfig {
        BadDataConfig {
            chi2_threshold: 3.84,
            lnr_threshold: 3.0,
            hypothesis_test_alpha: 0.05,
            max_bad_data: 5,
            leverage_threshold: 0.5,
        }
    }

    /// Build a simple identity-H system: H = I_m, W = I_m.
    /// With H = I (square), the hat matrix S = I, so Ω = 0 for diagonal.
    /// Use a rectangular system (m > n) for more typical behavior.
    fn make_clean_system(
        n_meas: usize,
        n_states: usize,
    ) -> (Vec<Measurement>, Vec<Vec<f64>>, Vec<f64>) {
        // H is an overdetermined system: each measurement observes one state
        let mut h = vec![vec![0.0f64; n_states]; n_meas];
        let mut meas = Vec::new();
        let mut w = Vec::new();

        #[allow(clippy::needless_range_loop)]
        for i in 0..n_meas {
            let state_idx = i % n_states;
            h[i][state_idx] = 1.0;
            let true_val = (state_idx + 1) as f64 * 0.1;
            let std_dev = 0.01;
            meas.push(Measurement {
                id: i,
                value: true_val,
                std_dev,
                estimated: true_val,
                residual: 0.0, // clean: residual = 0
            });
            w.push(1.0 / (std_dev * std_dev));
        }
        (meas, h, w)
    }

    /// Chi-squared test passes for clean (zero-residual) measurements.
    #[test]
    fn test_clean_measurements_chi2_passes() {
        let (meas, h, w) = make_clean_system(6, 3);
        let proc = BadDataProcessor::new(make_config());
        let result = proc.process(&meas, &h, &w).expect("process should succeed");
        // J = 0 with zero residuals → must pass
        assert!(
            result.chi2_test_passed,
            "Chi-squared test should pass for clean measurements; J={}",
            result.overall_chi2
        );
        assert!(
            result.suspected_bad.is_empty(),
            "No bad data expected for clean measurements"
        );
    }

    /// Injecting a large error on one measurement makes the LNR test flag it.
    #[test]
    fn test_injected_bad_measurement_identified_by_lnr() {
        let (mut meas, h, w) = make_clean_system(6, 3);
        // Inject gross error on measurement 4
        let bad_idx = 4;
        meas[bad_idx].residual = 10.0; // huge residual

        let proc = BadDataProcessor::new(make_config());
        let result = proc.process(&meas, &h, &w).expect("process should succeed");

        assert!(
            result.suspected_bad.contains(&bad_idx),
            "Expected measurement {bad_idx} to be flagged; got {:?}",
            result.suspected_bad
        );
    }

    /// LNR normalized residual for bad measurement should be the largest.
    #[test]
    fn test_lnr_normalized_residual_is_largest_for_bad_measurement() {
        let (mut meas, h, w) = make_clean_system(8, 4);
        let bad_idx = 2;
        meas[bad_idx].residual = 5.0;

        let proc = BadDataProcessor::new(make_config());
        let result = proc.process(&meas, &h, &w).expect("process");

        let bad_rn = result.normalized_residuals[bad_idx].abs();
        let max_others = result
            .normalized_residuals
            .iter()
            .enumerate()
            .filter(|(i, _)| *i != bad_idx)
            .map(|(_, &v)| v.abs())
            .fold(0.0f64, f64::max);

        assert!(
            bad_rn >= max_others,
            "Bad measurement normalized residual {bad_rn:.3} should be the largest; others max={max_others:.3}"
        );
    }

    /// High-leverage measurement (Sᵢᵢ close to 1 for single-observation state) is flagged.
    #[test]
    fn test_high_leverage_point_flagged() {
        // Build system where one measurement is the only observer of a state
        // H = [[1,0],[1,0],[0,1]], with state 1 observed only by row 2 → high leverage
        let h = vec![
            vec![1.0, 0.0],
            vec![1.0, 0.0],
            vec![0.0, 1.0], // sole observer → high leverage
        ];
        let std_dev = 0.01f64;
        let w_diag = vec![1.0 / (std_dev * std_dev); 3];
        let meas: Vec<Measurement> = (0..3)
            .map(|i| Measurement {
                id: i,
                value: 1.0,
                std_dev,
                estimated: 1.0,
                residual: 0.0,
            })
            .collect();

        let mut config = make_config();
        config.leverage_threshold = 0.8; // tight threshold
        let proc = BadDataProcessor::new(config);
        let result = proc.process(&meas, &h, &w_diag).expect("process");

        // Measurement 2 (sole observer of state 1) should have high leverage
        let s2 = result.sensitivity_matrix_diagonal[2];
        assert!(
            s2 > 0.8,
            "Sole-observer measurement should have high leverage Sᵢᵢ > 0.8; got {s2:.4}"
        );
    }

    /// Recommend Exclude action for measurement with very high normalized residual.
    #[test]
    fn test_recommend_exclude_for_very_high_lnr() {
        let (mut meas, h, w) = make_clean_system(6, 3);
        let bad_idx = 1;
        // Very large residual → should trigger Exclude (rN > 2*lnr_threshold = 6.0)
        meas[bad_idx].residual = 100.0;

        let proc = BadDataProcessor::new(make_config());
        let result = proc.process(&meas, &h, &w).expect("process");

        let action = &result.recommended_action[bad_idx];
        assert!(
            matches!(action, BadDataAction::Exclude { .. }),
            "Expected Exclude action for very high LNR; got {:?}",
            action
        );
    }

    /// Chi-squared test fails when total weighted residual is large.
    #[test]
    fn test_chi2_test_fails_for_large_residuals() {
        let (mut meas, h, w) = make_clean_system(6, 3);
        // Inject moderate errors on multiple measurements
        for item in meas.iter_mut().take(6) {
            item.residual = 1.0;
        }

        let proc = BadDataProcessor::new(make_config());
        let result = proc.process(&meas, &h, &w).expect("process");

        // J = sum(1^2 * 1/0.01^2) * 6 = 6 * 10000 = 60000 >> threshold
        assert!(
            !result.chi2_test_passed,
            "Chi-squared test should fail for large residuals; J={}",
            result.overall_chi2
        );
    }

    /// Dimension mismatch returns correct error, not a panic.
    #[test]
    fn test_dimension_mismatch_returns_error() {
        let meas = vec![Measurement {
            id: 0,
            value: 1.0,
            std_dev: 0.01,
            estimated: 1.0,
            residual: 0.0,
        }];
        // H has 2 rows but only 1 measurement
        let h = vec![vec![1.0], vec![1.0]];
        let w = vec![1000.0];
        let proc = BadDataProcessor::new(make_config());
        let result = proc.process(&meas, &h, &w);
        assert!(
            matches!(result, Err(BadDataError::DimensionMismatch { .. })),
            "Expected DimensionMismatch error"
        );
    }
}
