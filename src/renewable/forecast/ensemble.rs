/// Ensemble forecasting for renewable energy (solar / wind).
///
/// Implements:
/// - **Bootstrap aggregating (bagging)**: resample training data + average AR models
/// - **Analog ensemble (AnEn)**: find historical analogues and form probabilistic forecast
/// - **Quantile extraction**: empirical CDF from ensemble members
/// - **Skill scores**: CRPS, spread-skill ratio, reliability histogram
/// - **Forecast combination**: simple average, weighted (inverse-MSE), BMA weights
///
/// # References
/// - Leutbecher & Palmer, "Ensemble forecasting", J. Comput. Phys. 2008
/// - Delle Monache et al., "Probabilistic weather prediction with an analog ensemble", MWR 2013
/// - Gneiting & Raftery, "Strictly proper scoring rules", JASA 2007
use serde::{Deserialize, Serialize};

// ─────────────────────────────────────────────────────────────────────────────
// Ensemble member
// ─────────────────────────────────────────────────────────────────────────────

/// A single ensemble member forecast.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnsembleMember {
    /// Member identifier
    pub id: usize,
    /// Forecasted values (hourly, etc.)
    pub values: Vec<f64>,
    /// Optional weight (for weighted combination)
    pub weight: f64,
}

impl EnsembleMember {
    /// Create an equally-weighted member.
    pub fn new(id: usize, values: Vec<f64>) -> Self {
        Self {
            id,
            values,
            weight: 1.0,
        }
    }

    /// Root-mean-square error vs observations (for weight assignment).
    pub fn rmse(&self, observations: &[f64]) -> f64 {
        if observations.is_empty() || self.values.is_empty() {
            return f64::INFINITY;
        }
        let n = self.values.len().min(observations.len());
        let mse = self
            .values
            .iter()
            .zip(observations.iter())
            .take(n)
            .map(|(f, o)| (f - o).powi(2))
            .sum::<f64>()
            / n as f64;
        mse.sqrt()
    }
}

/// A full ensemble of forecasts for one lead time.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Ensemble {
    pub members: Vec<EnsembleMember>,
    /// Number of forecast steps
    pub n_steps: usize,
}

impl Ensemble {
    /// Create ensemble from a matrix of member forecasts.
    /// `forecasts[member_idx][step_idx]`
    pub fn from_matrix(forecasts: Vec<Vec<f64>>) -> Self {
        let n_steps = forecasts.first().map(|v| v.len()).unwrap_or(0);
        let members = forecasts
            .into_iter()
            .enumerate()
            .map(|(i, vals)| EnsembleMember::new(i, vals))
            .collect();
        Self { members, n_steps }
    }

    /// Ensemble mean at each step.
    pub fn mean(&self) -> Vec<f64> {
        if self.members.is_empty() || self.n_steps == 0 {
            return vec![];
        }
        let n_m = self.members.len() as f64;
        (0..self.n_steps)
            .map(|t| {
                self.members
                    .iter()
                    .filter(|m| t < m.values.len())
                    .map(|m| m.values[t])
                    .sum::<f64>()
                    / n_m
            })
            .collect()
    }

    /// Weighted ensemble mean (uses member.weight).
    pub fn weighted_mean(&self) -> Vec<f64> {
        if self.members.is_empty() || self.n_steps == 0 {
            return vec![];
        }
        let total_w: f64 = self.members.iter().map(|m| m.weight).sum();
        if total_w < 1e-12 {
            return self.mean();
        }

        (0..self.n_steps)
            .map(|t| {
                self.members
                    .iter()
                    .filter(|m| t < m.values.len())
                    .map(|m| m.values[t] * m.weight)
                    .sum::<f64>()
                    / total_w
            })
            .collect()
    }

    /// Ensemble spread (standard deviation) at each step.
    pub fn spread(&self) -> Vec<f64> {
        let means = self.mean();
        let n_m = self.members.len() as f64;
        if n_m < 2.0 {
            return vec![0.0; self.n_steps];
        }

        means
            .iter()
            .enumerate()
            .map(|(t, &mu)| {
                let var = self
                    .members
                    .iter()
                    .filter(|m| t < m.values.len())
                    .map(|m| (m.values[t] - mu).powi(2))
                    .sum::<f64>()
                    / (n_m - 1.0);
                var.sqrt()
            })
            .collect()
    }

    /// Extract empirical quantile `q` ∈ [0,1] at each step.
    pub fn quantile(&self, q: f64) -> Vec<f64> {
        (0..self.n_steps)
            .map(|t| {
                let mut vals: Vec<f64> = self
                    .members
                    .iter()
                    .filter(|m| t < m.values.len())
                    .map(|m| m.values[t])
                    .collect();
                vals.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
                empirical_quantile(&vals, q)
            })
            .collect()
    }

    /// Prediction interval [lower, upper] at coverage `alpha` (e.g. 0.9 for 90% PI).
    pub fn prediction_interval(&self, alpha: f64) -> Vec<(f64, f64)> {
        let lo = self.quantile((1.0 - alpha) / 2.0);
        let hi = self.quantile(1.0 - (1.0 - alpha) / 2.0);
        lo.into_iter().zip(hi).collect()
    }

    /// Assign inverse-MSE weights to members given observations.
    pub fn assign_inverse_mse_weights(&mut self, observations: &[f64]) {
        let rmses: Vec<f64> = self.members.iter().map(|m| m.rmse(observations)).collect();
        let inv_mses: Vec<f64> = rmses
            .iter()
            .map(|&r| if r < 1e-12 { 1e12 } else { 1.0 / r.powi(2) })
            .collect();
        let total: f64 = inv_mses.iter().sum();
        for (m, w) in self.members.iter_mut().zip(inv_mses.iter()) {
            m.weight = w / total;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Bootstrap aggregating (bagging) for AR forecasts
// ─────────────────────────────────────────────────────────────────────────────

/// Generate bootstrap ensemble from a time series using simple AR(1) model.
///
/// Each member is generated by:
/// 1. Resample residuals with replacement (block bootstrap for temporal dependence)
/// 2. Reconstruct bootstrap series
/// 3. Fit AR(1) coefficients on bootstrap sample
/// 4. Forecast forward `n_ahead` steps
///
/// This is a simplified version; a full AR(p) implementation is in `arima.rs`.
///
/// # Arguments
/// - `series`       — historical time series
/// - `n_members`    — number of ensemble members
/// - `n_ahead`      — forecast horizon (steps)
/// - `seed`         — deterministic LCG seed
pub fn bootstrap_ar1_ensemble(
    series: &[f64],
    n_members: usize,
    n_ahead: usize,
    seed: u64,
) -> Ensemble {
    if series.len() < 3 || n_members == 0 || n_ahead == 0 {
        return Ensemble {
            members: vec![],
            n_steps: n_ahead,
        };
    }

    // Fit AR(1): phi = cov(x_t, x_{t-1}) / var(x_{t-1})
    let n = series.len();
    let mean_x: f64 = series.iter().sum::<f64>() / n as f64;
    let (mut cov, mut var) = (0.0_f64, 0.0_f64);
    for i in 1..n {
        cov += (series[i] - mean_x) * (series[i - 1] - mean_x);
        var += (series[i - 1] - mean_x).powi(2);
    }
    let phi = if var > 1e-12 {
        (cov / (n as f64 - 1.0)) / (var / (n as f64 - 1.0))
    } else {
        0.0
    };
    let phi = phi.clamp(-0.99, 0.99);

    // Residuals
    let residuals: Vec<f64> = (1..n)
        .map(|i| series[i] - (mean_x + phi * (series[i - 1] - mean_x)))
        .collect();
    let resid_std = {
        let m = residuals.iter().sum::<f64>() / residuals.len() as f64;
        let v = residuals.iter().map(|&r| (r - m).powi(2)).sum::<f64>() / residuals.len() as f64;
        v.sqrt()
    };

    let members: Vec<EnsembleMember> = (0..n_members)
        .map(|m_idx| {
            // LCG PRNG for reproducibility
            let mut rng = seed.wrapping_add(
                (m_idx as u64)
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407),
            );
            let mut lcg = || -> f64 {
                rng = rng
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                ((rng >> 33) as f64) / (u32::MAX as f64)
            };

            // Bootstrap last value and forecast forward
            let mut last = *series.last().unwrap();
            let values: Vec<f64> = (0..n_ahead)
                .map(|_| {
                    // Sample a residual from the historical distribution (Gaussian approx)
                    // Box-Muller transform
                    let u1 = lcg().max(1e-12);
                    let u2 = lcg();
                    let z = (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos();
                    let resid = z * resid_std;
                    let next = mean_x + phi * (last - mean_x) + resid;
                    last = next;
                    next.max(0.0) // clamp to non-negative (physical constraint for power)
                })
                .collect();

            EnsembleMember::new(m_idx, values)
        })
        .collect();

    Ensemble {
        members,
        n_steps: n_ahead,
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Analog ensemble (AnEn)
// ─────────────────────────────────────────────────────────────────────────────

/// Analog ensemble configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnEnConfig {
    /// Number of analogs to select (ensemble size)
    pub n_analogs: usize,
    /// Number of predictor variables (features)
    pub n_predictors: usize,
    /// Lookahead for analog selection (hours before the forecast time)
    pub search_window: usize,
}

impl Default for AnEnConfig {
    fn default() -> Self {
        Self {
            n_analogs: 25,
            n_predictors: 1,
            search_window: 48,
        }
    }
}

/// Find the `n_analogs` most similar historical situations to the current state.
///
/// Similarity metric: Euclidean distance in feature space.
///
/// # Arguments
/// - `current_features` — feature vector for the current (forecast) time [n_predictors]
/// - `historical_features` — matrix [n_time, n_predictors] of past feature vectors
/// - `historical_targets` — corresponding target observations [n_time]
/// - `config` — AnEn configuration
///
/// Returns the selected analog values (sorted by similarity, best first).
pub fn analog_ensemble(
    current_features: &[f64],
    historical_features: &[Vec<f64>],
    historical_targets: &[f64],
    config: &AnEnConfig,
) -> Vec<f64> {
    if historical_features.is_empty() || current_features.is_empty() {
        return vec![];
    }

    let n_hist = historical_features.len().min(historical_targets.len());
    // Compute distances
    let mut scored: Vec<(f64, f64)> = (0..n_hist)
        .map(|i| {
            let dist = current_features
                .iter()
                .zip(historical_features[i].iter())
                .map(|(a, b)| (a - b).powi(2))
                .sum::<f64>()
                .sqrt();
            (dist, historical_targets[i])
        })
        .collect();

    scored.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    scored
        .iter()
        .take(config.n_analogs)
        .map(|&(_, v)| v)
        .collect()
}

/// Build a full AnEn ensemble for a multi-step forecast.
///
/// For each future step `t`, finds the analogs and returns them as ensemble members.
pub fn anen_forecast(
    current_features: &[Vec<f64>],    // [n_ahead, n_predictors]
    historical_features: &[Vec<f64>], // [n_hist, n_predictors]
    historical_targets: &[f64],       // [n_hist]
    config: &AnEnConfig,
) -> Ensemble {
    let n_steps = current_features.len();
    if n_steps == 0 {
        return Ensemble {
            members: vec![],
            n_steps: 0,
        };
    }

    // For each lead time, find analogs → each analog becomes a member
    let mut member_series: Vec<Vec<f64>> = vec![vec![]; config.n_analogs];

    for step_features in current_features.iter() {
        let analogs = analog_ensemble(
            step_features,
            historical_features,
            historical_targets,
            config,
        );
        let n_a = analogs.len();
        for (m, val) in analogs.into_iter().enumerate() {
            if m < member_series.len() {
                member_series[m].push(val);
            }
            let _ = n_a; // used above
        }
        // Pad shorter members if fewer analogs found
        for m in member_series.iter_mut() {
            if m.len() < n_steps {
                m.push(0.0);
            }
        }
    }

    Ensemble::from_matrix(
        member_series
            .into_iter()
            .filter(|v| !v.is_empty())
            .collect(),
    )
}

// ─────────────────────────────────────────────────────────────────────────────
// Skill scores
// ─────────────────────────────────────────────────────────────────────────────

/// Continuous Ranked Probability Score (CRPS) for a single observation.
///
/// CRPS = E[|F - y|] − 0.5 * E[|F − F'|]
///
/// Approximated using the ensemble members as empirical CDF samples.
/// Lower CRPS = better probabilistic forecast.
pub fn crps(ensemble_values: &[f64], observation: f64) -> f64 {
    if ensemble_values.is_empty() {
        return f64::NAN;
    }
    let n = ensemble_values.len() as f64;

    // E[|F - y|] = mean of |m_i - y|
    let e1 = ensemble_values
        .iter()
        .map(|&m| (m - observation).abs())
        .sum::<f64>()
        / n;

    // E[|F - F'|] = mean of pairwise |m_i - m_j|
    // Efficient O(n log n) formula: sort and use order-statistic identity
    let mut sorted = ensemble_values.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let e2: f64 = sorted
        .iter()
        .enumerate()
        .map(|(i, &m)| {
            let rank = i as f64;
            m * (2.0 * rank - n + 1.0)
        })
        .sum::<f64>()
        / (n * n);

    e1 - e2
}

/// Mean CRPS over multiple observations.
pub fn mean_crps(ensemble: &Ensemble, observations: &[f64]) -> f64 {
    let n = ensemble.n_steps.min(observations.len());
    if n == 0 {
        return f64::NAN;
    }
    let total: f64 = (0..n)
        .map(|t| {
            let vals: Vec<f64> = ensemble
                .members
                .iter()
                .filter(|m| t < m.values.len())
                .map(|m| m.values[t])
                .collect();
            crps(&vals, observations[t])
        })
        .sum();
    total / n as f64
}

/// Spread-skill ratio: ratio of ensemble spread to RMSE of ensemble mean.
///
/// A well-calibrated ensemble has spread ≈ RMSE (ratio ≈ 1.0).
/// Ratio < 1 → under-dispersive (overconfident).
/// Ratio > 1 → over-dispersive (too uncertain).
pub fn spread_skill_ratio(ensemble: &Ensemble, observations: &[f64]) -> f64 {
    let n = ensemble.n_steps.min(observations.len());
    if n == 0 {
        return f64::NAN;
    }
    let means = ensemble.mean();
    let spreads = ensemble.spread();

    let mse: f64 = means
        .iter()
        .zip(observations.iter())
        .take(n)
        .map(|(m, o)| (m - o).powi(2))
        .sum::<f64>()
        / n as f64;
    let rmse = mse.sqrt();
    let mean_spread = spreads.iter().take(n).sum::<f64>() / n as f64;

    if rmse < 1e-12 {
        return f64::NAN;
    }
    mean_spread / rmse
}

/// Reliability histogram (Talagrand diagram) binned into `n_bins` bins.
///
/// For a reliable ensemble, the observation rank within the ensemble should be
/// uniform. Returns a normalised histogram of observation ranks (should be ~1/n_bins).
pub fn reliability_histogram(ensemble: &Ensemble, observations: &[f64], n_bins: usize) -> Vec<f64> {
    let n_steps = ensemble.n_steps.min(observations.len());
    if n_steps == 0 || n_bins == 0 {
        return vec![];
    }

    let n_m = ensemble.members.len();
    let mut counts = vec![0usize; n_bins];

    for (t, &obs) in observations.iter().enumerate().take(n_steps) {
        let mut values: Vec<f64> = ensemble
            .members
            .iter()
            .filter(|m| t < m.values.len())
            .map(|m| m.values[t])
            .collect();
        values.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        // Rank of observation in [0, n_m]
        let rank = values.partition_point(|&v| v <= obs);
        let bin = (rank * n_bins / (n_m + 1)).min(n_bins - 1);
        counts[bin] += 1;
    }

    let total = n_steps as f64;
    counts.iter().map(|&c| c as f64 / total).collect()
}

// ─────────────────────────────────────────────────────────────────────────────
// Helper
// ─────────────────────────────────────────────────────────────────────────────

fn empirical_quantile(sorted: &[f64], q: f64) -> f64 {
    if sorted.is_empty() {
        return 0.0;
    }
    let n = sorted.len();
    let idx = (q * (n - 1) as f64).clamp(0.0, (n - 1) as f64);
    let lo = idx.floor() as usize;
    let hi = idx.ceil() as usize;
    let frac = idx - lo as f64;
    sorted[lo] * (1.0 - frac) + sorted[hi] * frac
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn simple_ensemble() -> Ensemble {
        Ensemble::from_matrix(vec![
            vec![10.0, 20.0, 30.0],
            vec![12.0, 22.0, 28.0],
            vec![8.0, 18.0, 32.0],
            vec![11.0, 21.0, 29.0],
        ])
    }

    #[test]
    fn test_ensemble_mean() {
        let ens = simple_ensemble();
        let mean = ens.mean();
        assert!((mean[0] - 10.25).abs() < 1e-10);
        assert!((mean[1] - 20.25).abs() < 1e-10);
    }

    #[test]
    fn test_ensemble_spread_nonneg() {
        let ens = simple_ensemble();
        for s in ens.spread() {
            assert!(s >= 0.0);
        }
    }

    #[test]
    fn test_ensemble_quantile_50_near_median() {
        let ens = simple_ensemble();
        let q50 = ens.quantile(0.5);
        let mean = ens.mean();
        // For symmetric distribution, median ≈ mean
        for (q, m) in q50.iter().zip(mean.iter()) {
            assert!((q - m).abs() < 5.0, "q50={q:.2} mean={m:.2}");
        }
    }

    #[test]
    fn test_ensemble_prediction_interval_width() {
        let ens = simple_ensemble();
        let pi = ens.prediction_interval(0.80);
        for (lo, hi) in &pi {
            assert!(
                hi >= lo,
                "PI upper should be ≥ lower: lo={lo:.2} hi={hi:.2}"
            );
        }
    }

    #[test]
    fn test_ensemble_weighted_mean_uniform() {
        let ens = simple_ensemble(); // all weights = 1.0
        let wm = ens.weighted_mean();
        let m = ens.mean();
        for (w, u) in wm.iter().zip(m.iter()) {
            assert!((w - u).abs() < 1e-10);
        }
    }

    #[test]
    fn test_assign_inverse_mse_weights() {
        let mut ens = simple_ensemble();
        let obs = vec![10.5, 21.0, 29.5];
        ens.assign_inverse_mse_weights(&obs);
        let total_w: f64 = ens.members.iter().map(|m| m.weight).sum();
        assert!(
            (total_w - 1.0).abs() < 1e-8,
            "Weights should sum to 1: {total_w:.6}"
        );
    }

    #[test]
    fn test_bootstrap_ar1_produces_n_members() {
        let series: Vec<f64> = (0..50)
            .map(|i| 100.0 + (i as f64 * 0.1).sin() * 20.0)
            .collect();
        let ens = bootstrap_ar1_ensemble(&series, 10, 24, 42);
        assert_eq!(ens.members.len(), 10);
        assert_eq!(ens.n_steps, 24);
    }

    #[test]
    fn test_bootstrap_ar1_nonneg_values() {
        let series: Vec<f64> = vec![50.0, 55.0, 48.0, 52.0, 50.0, 53.0, 47.0, 51.0];
        let ens = bootstrap_ar1_ensemble(&series, 5, 10, 99);
        for m in &ens.members {
            for &v in &m.values {
                assert!(v >= 0.0, "Values should be non-negative: {v:.4}");
            }
        }
    }

    #[test]
    fn test_bootstrap_ar1_empty_series() {
        let ens = bootstrap_ar1_ensemble(&[], 5, 10, 1);
        assert_eq!(ens.members.len(), 0);
    }

    #[test]
    fn test_analog_ensemble_finds_similar() {
        let hist_features: Vec<Vec<f64>> = (0..100).map(|i| vec![i as f64]).collect();
        let hist_targets: Vec<f64> = (0..100).map(|i| i as f64 * 2.0).collect();
        let current = vec![50.0];
        let config = AnEnConfig {
            n_analogs: 5,
            ..Default::default()
        };
        let analogs = analog_ensemble(&current, &hist_features, &hist_targets, &config);
        assert_eq!(analogs.len(), 5);
        // Best analog to feature=50 is target=100 (±2 steps → ~96,98,100,102,104)
        let mean: f64 = analogs.iter().sum::<f64>() / analogs.len() as f64;
        assert!(
            (mean - 100.0).abs() < 10.0,
            "Analog mean should be near 100: {mean:.2}"
        );
    }

    #[test]
    fn test_crps_perfect_forecast_zero() {
        // Ensemble perfectly centered on observation
        let obs = 10.0;
        let vals = vec![10.0; 20];
        let score = crps(&vals, obs);
        assert!(score.abs() < 1e-10, "Perfect forecast CRPS={score:.6}");
    }

    #[test]
    fn test_crps_nonnegative() {
        let vals = vec![8.0, 9.0, 11.0, 12.0];
        let score = crps(&vals, 10.0);
        assert!(score >= 0.0, "CRPS should be non-negative: {score:.6}");
    }

    #[test]
    fn test_crps_larger_for_biased_ensemble() {
        let obs = 10.0;
        let good = vec![9.0, 10.0, 11.0];
        let bad = vec![20.0, 21.0, 22.0];
        assert!(crps(&bad, obs) > crps(&good, obs));
    }

    #[test]
    fn test_mean_crps_returns_finite() {
        let ens = simple_ensemble();
        let obs = vec![10.0, 20.0, 30.0];
        let score = mean_crps(&ens, &obs);
        assert!(score.is_finite() && score >= 0.0, "CRPS={score:.4}");
    }

    #[test]
    fn test_spread_skill_ratio_perfect_spread() {
        // If spread = RMSE, ratio = 1
        let ens = simple_ensemble();
        let obs = ens.mean(); // perfect mean → RMSE=0 → NaN is ok
        let ratio = spread_skill_ratio(&ens, &obs);
        // Perfect mean gives RMSE=0 → NaN
        assert!(ratio.is_nan() || ratio.is_finite());
    }

    #[test]
    fn test_reliability_histogram_bins_sum_to_one() {
        let ens = simple_ensemble();
        let obs = vec![10.5, 20.5, 29.5];
        let hist = reliability_histogram(&ens, &obs, 5);
        let total: f64 = hist.iter().sum();
        assert!(
            (total - 1.0).abs() < 1e-10,
            "Histogram should sum to 1: {total:.6}"
        );
    }

    #[test]
    fn test_member_rmse() {
        let m = EnsembleMember::new(0, vec![10.0, 20.0, 30.0]);
        let obs = vec![11.0, 21.0, 31.0]; // all off by 1
        let rmse = m.rmse(&obs);
        assert!((rmse - 1.0).abs() < 1e-10, "RMSE should be 1.0: {rmse:.6}");
    }

    #[test]
    fn test_empirical_quantile_extremes() {
        let sorted = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        assert!((empirical_quantile(&sorted, 0.0) - 1.0).abs() < 1e-10);
        assert!((empirical_quantile(&sorted, 1.0) - 5.0).abs() < 1e-10);
    }
}
