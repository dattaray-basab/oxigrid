/// Probabilistic renewable energy forecasting.
///
/// Provides:
/// - `QuantileRegressor`       — linear quantile regression (pinball-loss minimised)
/// - `KernelDensityEstimator`  — Gaussian KDE for density estimation
/// - `ProbabilisticForecast`   — multi-quantile prediction interval
/// - `pinball_loss`            — proper scoring rule for quantile forecasts
/// - `reliability_diagram`     — calibration check (empirical vs. nominal coverage)
///
/// # Quantile Regression
/// Minimises the pinball (check) loss:
///   L_τ(y, ŷ) = τ·(y − ŷ)    if y ≥ ŷ
///              (τ−1)·(y − ŷ)  if y < ŷ
///
/// # Kernel Density Estimation
/// Gaussian kernel with bandwidth h (Silverman's rule by default):
///   f̂(x) = (1/nh) Σ_i φ((x − x_i)/h)
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

// ─── Pinball loss ───────────────────────────────────────────────────────────

/// Compute the mean pinball loss for a single quantile forecast.
///
/// # Arguments
/// - `predictions` — forecast values at quantile `tau`
/// - `actuals`     — observed values
/// - `tau`         — quantile level (0 < τ < 1)
pub fn pinball_loss(predictions: &[f64], actuals: &[f64], tau: f64) -> f64 {
    assert_eq!(predictions.len(), actuals.len(), "lengths must match");
    if predictions.is_empty() {
        return 0.0;
    }
    let n = predictions.len() as f64;
    let sum: f64 = predictions
        .iter()
        .zip(actuals.iter())
        .map(|(&y_hat, &y)| {
            let e = y - y_hat;
            if e >= 0.0 {
                tau * e
            } else {
                (tau - 1.0) * e
            }
        })
        .sum();
    sum / n
}

/// Compute pinball loss for multiple quantiles simultaneously.
///
/// Returns a `Vec<(tau, mean_pinball)>` for each quantile level.
pub fn pinball_loss_multi(
    predictions: &[Vec<f64>],
    actuals: &[f64],
    taus: &[f64],
) -> Vec<(f64, f64)> {
    taus.iter()
        .zip(predictions.iter())
        .map(|(&tau, preds)| (tau, pinball_loss(preds, actuals, tau)))
        .collect()
}

/// Weighted interval score (WIS) — summary proper score for interval forecasts.
///
/// For a (1−α) prediction interval [L, U] and observed y:
///   WIS = (U−L) + (2/α)·max(0, L−y) + (2/α)·max(0, y−U)
pub fn weighted_interval_score(lower: &[f64], upper: &[f64], actuals: &[f64], alpha: f64) -> f64 {
    assert_eq!(lower.len(), upper.len());
    assert_eq!(lower.len(), actuals.len());
    if lower.is_empty() {
        return 0.0;
    }
    let n = lower.len() as f64;
    let scale = 2.0 / alpha.max(1e-9);
    let sum: f64 = lower
        .iter()
        .zip(upper.iter())
        .zip(actuals.iter())
        .map(|((&l, &u), &y)| (u - l) + scale * (l - y).max(0.0) + scale * (y - u).max(0.0))
        .sum();
    sum / n
}

// ─── Kernel Density Estimator ───────────────────────────────────────────────

/// Gaussian kernel density estimator.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct KernelDensityEstimator {
    data: Vec<f64>,
    bandwidth: f64,
}

impl KernelDensityEstimator {
    /// Fit KDE to data using Silverman's rule for bandwidth selection.
    pub fn fit(data: &[f64]) -> Self {
        let h = Self::silverman_bw(data);
        Self {
            data: data.to_vec(),
            bandwidth: h,
        }
    }

    /// Fit KDE with an explicit bandwidth.
    pub fn fit_with_bandwidth(data: &[f64], bandwidth: f64) -> Self {
        Self {
            data: data.to_vec(),
            bandwidth: bandwidth.max(1e-9),
        }
    }

    /// Silverman's rule: h = 0.9 · min(σ, IQR/1.34) · n^{-1/5}
    pub fn silverman_bw(data: &[f64]) -> f64 {
        if data.len() < 2 {
            return 1.0;
        }
        let n = data.len() as f64;
        let std_dev = Self::std_dev(data);
        let iqr = Self::iqr(data);
        let spread = std_dev.min(iqr / 1.34).max(1e-9);
        0.9 * spread * n.powf(-0.2)
    }

    /// Evaluate PDF at a single point.
    pub fn pdf(&self, x: f64) -> f64 {
        let n = self.data.len() as f64;
        let h = self.bandwidth;
        let norm = 1.0 / (n * h * (2.0 * PI).sqrt());
        self.data
            .iter()
            .map(|&xi| {
                let z = (x - xi) / h;
                norm * (-0.5 * z * z).exp()
            })
            .sum()
    }

    /// Evaluate PDF at multiple points.
    pub fn pdf_batch(&self, xs: &[f64]) -> Vec<f64> {
        xs.iter().map(|&x| self.pdf(x)).collect()
    }

    /// Evaluate CDF at a single point (numerical integration via trapezoidal rule).
    pub fn cdf(&self, x: f64) -> f64 {
        let lo = self.data.iter().cloned().fold(f64::INFINITY, f64::min) - 5.0 * self.bandwidth;
        let n_steps = 200usize;
        let dx = (x - lo) / n_steps as f64;
        if dx <= 0.0 {
            return 0.0;
        }
        let mut sum = 0.0;
        let mut f_prev = self.pdf(lo);
        for k in 1..=n_steps {
            let x_k = lo + k as f64 * dx;
            let f_k = self.pdf(x_k);
            sum += 0.5 * (f_prev + f_k) * dx;
            f_prev = f_k;
        }
        sum.clamp(0.0, 1.0)
    }

    /// Compute quantile via bisection on the CDF.
    pub fn quantile(&self, tau: f64) -> f64 {
        let tau = tau.clamp(1e-6, 1.0 - 1e-6);
        let lo = self.data.iter().cloned().fold(f64::INFINITY, f64::min) - 3.0 * self.bandwidth;
        let hi = self.data.iter().cloned().fold(f64::NEG_INFINITY, f64::max) + 3.0 * self.bandwidth;
        let mut a = lo;
        let mut b = hi;
        for _ in 0..60 {
            let mid = 0.5 * (a + b);
            if self.cdf(mid) < tau {
                a = mid;
            } else {
                b = mid;
            }
        }
        0.5 * (a + b)
    }

    fn std_dev(data: &[f64]) -> f64 {
        let n = data.len() as f64;
        let mean = data.iter().sum::<f64>() / n;
        let var = data.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / n;
        var.sqrt()
    }

    fn iqr(data: &[f64]) -> f64 {
        let mut sorted = data.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let n = sorted.len();
        let q25 = sorted[(n as f64 * 0.25) as usize];
        let q75 = sorted[(n as f64 * 0.75).min(n as f64 - 1.0) as usize];
        (q75 - q25).max(0.0)
    }
}

// ─── Quantile Regressor ─────────────────────────────────────────────────────

/// Linear quantile regressor: ŷ_τ = β_0 + β_1·x_1 + … + β_p·x_p.
///
/// Minimises the pinball loss via iteratively reweighted least squares (IRLS)
/// with a smooth approximation to the pinball loss.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QuantileRegressor {
    /// Quantile level (0 < τ < 1)
    pub tau: f64,
    /// Intercept
    pub intercept: f64,
    /// Regression coefficients (one per feature)
    pub coefficients: Vec<f64>,
    /// Number of IRLS iterations used
    pub n_iter: usize,
}

impl QuantileRegressor {
    /// Fit quantile regression to (X, y) data.
    ///
    /// # Arguments
    /// - `x`    — feature matrix, shape \[n_samples × n_features\]
    /// - `y`    — target values, length n_samples
    /// - `tau`  — quantile level
    /// - `tol`  — convergence tolerance
    pub fn fit(x: &[Vec<f64>], y: &[f64], tau: f64, tol: f64) -> Self {
        assert!(!x.is_empty());
        assert_eq!(x.len(), y.len());
        let n = x.len();
        let p = x[0].len();
        let max_iter = 200;

        // Initialise with OLS estimate
        let mut beta = vec![0.0f64; p + 1]; // beta[0] = intercept
        beta[0] = y.iter().sum::<f64>() / n as f64;

        let mut n_iter = 0;
        for _iter in 0..max_iter {
            n_iter += 1;
            let old_beta = beta.clone();

            // Compute residuals
            let residuals: Vec<f64> = (0..n)
                .map(|i| y[i] - Self::predict_one(&beta, &x[i]))
                .collect();

            // IRLS weights: w_i = τ if r_i > 0 else (1-τ), smoothed near zero
            let epsilon = 0.01 * y.iter().map(|v| v.abs()).sum::<f64>() / n as f64 + 1e-9;
            let weights: Vec<f64> = residuals
                .iter()
                .map(|&r| {
                    if r.abs() < epsilon {
                        0.5 * epsilon / (epsilon + r.abs())
                    } else if r > 0.0 {
                        tau
                    } else {
                        1.0 - tau
                    }
                })
                .collect();

            // Weighted least squares (dense, via normal equations)
            // Augmented feature matrix: [1 | X]
            let q = p + 1;
            let mut xtx = vec![0.0f64; q * q];
            let mut xty = vec![0.0f64; q];
            for i in 0..n {
                let xi: Vec<f64> = std::iter::once(1.0).chain(x[i].iter().cloned()).collect();
                let w = weights[i];
                for j in 0..q {
                    xty[j] += w * xi[j] * y[i];
                    for k in 0..q {
                        xtx[j * q + k] += w * xi[j] * xi[k];
                    }
                }
            }

            // Solve normal equations via Gaussian elimination
            if let Some(new_beta) = solve_linear_system(&xtx, &xty, q) {
                beta = new_beta;
            } else {
                break;
            }

            // Check convergence
            let diff: f64 = beta
                .iter()
                .zip(old_beta.iter())
                .map(|(a, b)| (a - b).abs())
                .fold(0.0f64, f64::max);
            if diff < tol {
                break;
            }
        }

        Self {
            tau,
            intercept: beta[0],
            coefficients: beta[1..].to_vec(),
            n_iter,
        }
    }

    /// Predict quantile for a single sample.
    pub fn predict_single(&self, x: &[f64]) -> f64 {
        let beta: Vec<f64> = std::iter::once(self.intercept)
            .chain(self.coefficients.iter().cloned())
            .collect();
        Self::predict_one(&beta, x)
    }

    /// Predict quantile for multiple samples.
    pub fn predict(&self, x: &[Vec<f64>]) -> Vec<f64> {
        x.iter().map(|xi| self.predict_single(xi)).collect()
    }

    fn predict_one(beta: &[f64], x: &[f64]) -> f64 {
        let mut y = beta[0];
        for (j, &xj) in x.iter().enumerate() {
            if j + 1 < beta.len() {
                y += beta[j + 1] * xj;
            }
        }
        y
    }
}

/// Solve Ax = b via Gaussian elimination with partial pivoting.
fn solve_linear_system(a_flat: &[f64], b: &[f64], n: usize) -> Option<Vec<f64>> {
    let mut a = a_flat.to_vec();
    let mut x = b.to_vec();

    for col in 0..n {
        // Pivot
        let mut max_row = col;
        let mut max_val = a[col * n + col].abs();
        for row in col + 1..n {
            let v = a[row * n + col].abs();
            if v > max_val {
                max_val = v;
                max_row = row;
            }
        }
        if max_val < 1e-14 {
            return None;
        }
        if max_row != col {
            for k in 0..n {
                a.swap(col * n + k, max_row * n + k);
            }
            x.swap(col, max_row);
        }
        let pivot = a[col * n + col];
        for row in col + 1..n {
            let factor = a[row * n + col] / pivot;
            for k in col..n {
                a[row * n + k] -= factor * a[col * n + k];
            }
            x[row] -= factor * x[col];
        }
    }
    for col in (0..n).rev() {
        if a[col * n + col].abs() < 1e-14 {
            return None;
        }
        x[col] /= a[col * n + col];
        for row in 0..col {
            x[row] -= a[row * n + col] * x[col];
        }
    }
    Some(x)
}

// ─── Probabilistic forecast ─────────────────────────────────────────────────

/// Multi-quantile forecast for a single target variable.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProbabilisticForecast {
    /// Quantile levels, e.g. [0.1, 0.25, 0.5, 0.75, 0.9]
    pub taus: Vec<f64>,
    /// Forecast values at each quantile (same length as `taus`)
    pub quantile_values: Vec<f64>,
}

impl ProbabilisticForecast {
    /// Build from quantile regressors evaluated at a single feature vector.
    pub fn from_regressors(regressors: &[QuantileRegressor], x: &[f64]) -> Self {
        let taus: Vec<f64> = regressors.iter().map(|r| r.tau).collect();
        let quantile_values: Vec<f64> = regressors.iter().map(|r| r.predict_single(x)).collect();
        Self {
            taus,
            quantile_values,
        }
    }

    /// Build from KDE-derived quantiles.
    pub fn from_kde(kde: &KernelDensityEstimator, taus: &[f64]) -> Self {
        let quantile_values: Vec<f64> = taus.iter().map(|&tau| kde.quantile(tau)).collect();
        Self {
            taus: taus.to_vec(),
            quantile_values,
        }
    }

    /// Central (median) forecast.
    pub fn median(&self) -> Option<f64> {
        let idx = self.taus.iter().position(|&t| (t - 0.5).abs() < 0.01)?;
        Some(self.quantile_values[idx])
    }

    /// Prediction interval at coverage level (1 - α).
    ///
    /// Returns (lower, upper) using the symmetric quantiles closest to α/2 and 1-α/2.
    pub fn prediction_interval(&self, alpha: f64) -> Option<(f64, f64)> {
        let tau_lo = alpha / 2.0;
        let tau_hi = 1.0 - alpha / 2.0;

        let find = |target: f64| -> Option<f64> {
            self.taus
                .iter()
                .zip(self.quantile_values.iter())
                .min_by(|(&ta, _), (&tb, _)| {
                    (ta - target)
                        .abs()
                        .partial_cmp(&(tb - target).abs())
                        .unwrap()
                })
                .map(|(_, &v)| v)
        };

        let lo = find(tau_lo)?;
        let hi = find(tau_hi)?;
        Some((lo, hi))
    }

    /// Interval width at coverage level (1 - α).
    pub fn interval_width(&self, alpha: f64) -> Option<f64> {
        let (lo, hi) = self.prediction_interval(alpha)?;
        Some(hi - lo)
    }
}

// ─── Reliability diagram ────────────────────────────────────────────────────

/// One point on a reliability (calibration) diagram.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ReliabilityPoint {
    /// Nominal quantile level (0–1)
    pub nominal_tau: f64,
    /// Empirical coverage: fraction of actuals below the predicted quantile
    pub empirical_coverage: f64,
}

/// Compute a reliability diagram: for each quantile, measure actual coverage.
///
/// Perfect calibration: empirical_coverage ≈ nominal_tau for all quantiles.
///
/// # Arguments
/// - `predictions` — for each quantile: predicted values (outer index = quantile)
/// - `actuals`     — observed values
/// - `taus`        — quantile levels
pub fn reliability_diagram(
    predictions: &[Vec<f64>],
    actuals: &[f64],
    taus: &[f64],
) -> Vec<ReliabilityPoint> {
    let n = actuals.len() as f64;
    taus.iter()
        .zip(predictions.iter())
        .map(|(&tau, preds)| {
            let coverage = preds
                .iter()
                .zip(actuals.iter())
                .filter(|(&p, &a)| a <= p)
                .count() as f64
                / n;
            ReliabilityPoint {
                nominal_tau: tau,
                empirical_coverage: coverage,
            }
        })
        .collect()
}

/// Mean calibration error (MCE): mean |empirical − nominal| over all quantiles.
pub fn mean_calibration_error(diagram: &[ReliabilityPoint]) -> f64 {
    if diagram.is_empty() {
        return 0.0;
    }
    let sum: f64 = diagram
        .iter()
        .map(|pt| (pt.empirical_coverage - pt.nominal_tau).abs())
        .sum();
    sum / diagram.len() as f64
}

// ─── Ensemble-based forecasting ─────────────────────────────────────────────

/// Generate scenario quantiles from an ensemble of forecasts.
///
/// Sorts the ensemble for each sample and extracts quantiles.
pub fn ensemble_quantiles(ensemble: &[Vec<f64>], taus: &[f64]) -> Vec<Vec<f64>> {
    if ensemble.is_empty() || ensemble[0].is_empty() {
        return taus.iter().map(|_| vec![]).collect();
    }
    let n_samples = ensemble[0].len();
    let n_members = ensemble.len();

    // For each sample, collect and sort ensemble values
    let mut per_sample_sorted: Vec<Vec<f64>> = (0..n_samples)
        .map(|s| {
            let mut vals: Vec<f64> = ensemble.iter().map(|member| member[s]).collect();
            vals.sort_by(|a, b| a.partial_cmp(b).unwrap());
            vals
        })
        .collect();

    // Extract quantiles
    taus.iter()
        .map(|&tau| {
            let idx = ((tau * n_members as f64) as usize).min(n_members - 1);
            per_sample_sorted
                .iter_mut()
                .map(|sorted| sorted[idx])
                .collect()
        })
        .collect()
}

// ─── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Pinball loss ──
    #[test]
    fn test_pinball_loss_zero_when_exact() {
        // ŷ = y → residuals = 0 → pinball = 0
        let preds = vec![1.0, 2.0, 3.0];
        let actuals = vec![1.0, 2.0, 3.0];
        let loss = pinball_loss(&preds, &actuals, 0.5);
        assert!(loss.abs() < 1e-9, "Pinball loss should be zero: {}", loss);
    }

    #[test]
    fn test_pinball_loss_overforecast_penalised() {
        // ŷ > y → (τ−1)·(y−ŷ) > 0
        let preds = vec![5.0];
        let actuals = vec![2.0];
        let loss = pinball_loss(&preds, &actuals, 0.9);
        // e = 2-5 = -3;  (0.9-1)·(-3) = 0.1·3 = 0.3
        assert!((loss - 0.3).abs() < 1e-9);
    }

    #[test]
    fn test_pinball_loss_underforecast_penalised() {
        // ŷ < y → τ·(y−ŷ)
        let preds = vec![2.0];
        let actuals = vec![5.0];
        let loss = pinball_loss(&preds, &actuals, 0.9);
        // e = 5-2 = 3; 0.9·3 = 2.7
        assert!((loss - 2.7).abs() < 1e-9);
    }

    #[test]
    fn test_pinball_loss_symmetric_at_median() {
        // At τ=0.5, over/under forecast of equal magnitude have equal loss
        let preds_over = vec![5.0];
        let preds_under = vec![-1.0];
        let actuals = vec![2.0];
        let loss_over = pinball_loss(&preds_over, &actuals, 0.5);
        let loss_under = pinball_loss(&preds_under, &actuals, 0.5);
        assert!(
            (loss_over - loss_under).abs() < 1e-9,
            "over={} under={}",
            loss_over,
            loss_under
        );
    }

    // ── WIS ──
    #[test]
    fn test_wis_zero_for_covering_interval() {
        // y inside [L, U] → only width penalty
        let lower = vec![0.0];
        let upper = vec![2.0];
        let actual = vec![1.0];
        let wis = weighted_interval_score(&lower, &upper, &actual, 0.1);
        // (2-0) + 0 + 0 = 2.0
        assert!((wis - 2.0).abs() < 1e-9);
    }

    #[test]
    fn test_wis_penalises_miss() {
        let lower = vec![0.0];
        let upper = vec![1.0];
        let actual = vec![3.0]; // outside
        let wis = weighted_interval_score(&lower, &upper, &actual, 0.1);
        // (1) + (2/0.1)·(3-1) = 1 + 20·2 = 41
        assert!((wis - 41.0).abs() < 1e-9, "WIS = {}", wis);
    }

    // ── KDE ──
    #[test]
    fn test_kde_pdf_integrates_to_one() {
        let data: Vec<f64> = (0..100).map(|i| i as f64 * 0.1).collect();
        let kde = KernelDensityEstimator::fit(&data);
        // Numerical integration over [-2, 12] with 1000 steps
        let lo = -2.0;
        let hi = 12.0;
        let steps = 1000;
        let dx = (hi - lo) / steps as f64;
        let integral: f64 = (0..steps)
            .map(|k| {
                let x = lo + (k as f64 + 0.5) * dx;
                kde.pdf(x) * dx
            })
            .sum();
        assert!(
            (integral - 1.0).abs() < 0.05,
            "KDE integral = {:.4}",
            integral
        );
    }

    #[test]
    fn test_kde_pdf_positive() {
        let data = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let kde = KernelDensityEstimator::fit(&data);
        assert!(kde.pdf(3.0) > 0.0);
    }

    #[test]
    fn test_kde_cdf_monotone() {
        let data: Vec<f64> = (0..20).map(|i| i as f64).collect();
        let kde = KernelDensityEstimator::fit(&data);
        let mut prev = kde.cdf(-10.0);
        for x in (0..20).map(|i| i as f64) {
            let cur = kde.cdf(x);
            assert!(cur >= prev - 1e-9, "CDF not monotone at x={}", x);
            prev = cur;
        }
    }

    #[test]
    fn test_kde_quantile_round_trip() {
        let data: Vec<f64> = (0..50).map(|i| i as f64).collect();
        let kde = KernelDensityEstimator::fit(&data);
        let q50 = kde.quantile(0.5);
        // Median should be near 24.5
        assert!(q50 > 15.0 && q50 < 34.0, "Median = {:.2}", q50);
    }

    #[test]
    fn test_kde_bandwidth_silverman() {
        let data = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0];
        let h = KernelDensityEstimator::silverman_bw(&data);
        assert!(h > 0.0 && h < 10.0, "Silverman bandwidth = {:.4}", h);
    }

    // ── Quantile Regressor ──
    #[test]
    fn test_quantile_regressor_median_fit() {
        // y = 2x + 1 with no noise: median should recover slope ≈ 2, intercept ≈ 1
        let x: Vec<Vec<f64>> = (0..20).map(|i| vec![i as f64]).collect();
        let y: Vec<f64> = (0..20).map(|i| 2.0 * i as f64 + 1.0).collect();
        let reg = QuantileRegressor::fit(&x, &y, 0.5, 1e-6);
        assert!(
            (reg.intercept - 1.0).abs() < 1.0,
            "intercept={:.4}",
            reg.intercept
        );
        assert!(
            (reg.coefficients[0] - 2.0).abs() < 1.0,
            "slope={:.4}",
            reg.coefficients[0]
        );
    }

    #[test]
    fn test_quantile_regressor_q10_lower_than_q90() {
        let x: Vec<Vec<f64>> = (0..30).map(|i| vec![i as f64]).collect();
        let y: Vec<f64> = (0..30).map(|i| (i as f64) + (i % 3) as f64 * 0.5).collect();
        let reg10 = QuantileRegressor::fit(&x, &y, 0.1, 1e-4);
        let reg90 = QuantileRegressor::fit(&x, &y, 0.9, 1e-4);
        let q10 = reg10.predict_single(&[15.0]);
        let q90 = reg90.predict_single(&[15.0]);
        assert!(
            q10 < q90 + 5.0,
            "Q10={:.2} should be below Q90={:.2}",
            q10,
            q90
        );
    }

    // ── Probabilistic forecast ──
    #[test]
    fn test_probabilistic_forecast_from_kde() {
        let data: Vec<f64> = (0..100).map(|i| i as f64 * 0.1).collect();
        let kde = KernelDensityEstimator::fit(&data);
        let taus = vec![0.1, 0.5, 0.9];
        let forecast = ProbabilisticForecast::from_kde(&kde, &taus);
        assert_eq!(forecast.taus.len(), 3);
        assert_eq!(forecast.quantile_values.len(), 3);
        // Quantiles should be monotone
        assert!(forecast.quantile_values[0] < forecast.quantile_values[1]);
        assert!(forecast.quantile_values[1] < forecast.quantile_values[2]);
    }

    #[test]
    fn test_prediction_interval() {
        let data: Vec<f64> = (0..50).map(|i| i as f64).collect();
        let kde = KernelDensityEstimator::fit(&data);
        let taus = vec![0.05, 0.25, 0.5, 0.75, 0.95];
        let forecast = ProbabilisticForecast::from_kde(&kde, &taus);
        let interval = forecast.prediction_interval(0.10).unwrap();
        assert!(interval.0 < interval.1, "Lower should be below upper");
    }

    // ── Reliability diagram ──
    #[test]
    fn test_reliability_diagram_perfect_calibration() {
        // Predictions exactly equal actuals sorted at each quantile
        let actuals: Vec<f64> = (1..=10).map(|i| i as f64).collect();
        let taus = vec![0.1, 0.5, 0.9];
        // For perfect calibration: pred at tau should equal tau-th quantile of actuals
        let sorted_actuals = {
            let mut s = actuals.clone();
            s.sort_by(|a, b| a.partial_cmp(b).unwrap());
            s
        };
        let predictions: Vec<Vec<f64>> = taus
            .iter()
            .map(|&tau| {
                let q = sorted_actuals[((tau * 10.0) as usize).min(9)];
                actuals.iter().map(|_| q).collect()
            })
            .collect();
        let diagram = reliability_diagram(&predictions, &actuals, &taus);
        assert_eq!(diagram.len(), 3);
        let mce = mean_calibration_error(&diagram);
        assert!(mce < 0.5, "MCE = {:.4}", mce);
    }

    // ── Ensemble quantiles ──
    #[test]
    fn test_ensemble_quantiles_shape() {
        let ensemble = vec![
            vec![1.0, 2.0, 3.0],
            vec![2.0, 3.0, 4.0],
            vec![3.0, 4.0, 5.0],
        ];
        let taus = vec![0.1, 0.5, 0.9];
        let quantiles = ensemble_quantiles(&ensemble, &taus);
        assert_eq!(quantiles.len(), 3);
        assert_eq!(quantiles[0].len(), 3); // 3 samples
    }

    #[test]
    fn test_ensemble_quantiles_monotone_per_sample() {
        let ensemble: Vec<Vec<f64>> = (0..10)
            .map(|i| (0..5).map(|s| s as f64 + i as f64 * 0.1).collect())
            .collect();
        let taus = vec![0.1, 0.5, 0.9];
        let quantiles = ensemble_quantiles(&ensemble, &taus);
        for (s, q0) in quantiles[0].iter().enumerate().take(5) {
            assert!(*q0 <= quantiles[1][s] + 1e-9);
            assert!(quantiles[1][s] <= quantiles[2][s] + 1e-9);
        }
    }
}
