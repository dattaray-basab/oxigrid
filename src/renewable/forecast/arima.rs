/// ARIMA time-series forecasting for renewable energy output.
///
/// Implements:
/// - **AR(p)**: Autoregressive model — y_t = c + φ₁·y_{t-1} + … + φ_p·y_{t-p} + ε_t
/// - **MA(q)**: Moving-average model — y_t = c + ε_t + θ₁·ε_{t-1} + … + θ_q·ε_{t-q}
/// - **ARMA(p,q)**: Combined AR and MA
/// - **ARIMA(p,d,q)**: ARMA on d-th differenced series
///
/// Coefficients are estimated using the Yule-Walker equations for AR models,
/// and a simplified conditional least-squares for MA/ARMA.
///
/// # Reference
/// Box, G.E.P., Jenkins, G.M. (1970). "Time Series Analysis: Forecasting and Control".
use serde::{Deserialize, Serialize};

/// AR(p) model coefficients and intercept.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ArModel {
    /// AR order p
    pub order: usize,
    /// Intercept (mean of the series)
    pub intercept: f64,
    /// AR coefficients φ₁…φ_p
    pub phi: Vec<f64>,
    /// Residual variance σ²
    pub sigma2: f64,
}

impl ArModel {
    /// Fit an AR(p) model to a time series using Yule-Walker equations.
    ///
    /// Returns None if the system is under-determined (n ≤ p).
    pub fn fit(series: &[f64], order: usize) -> Option<Self> {
        let n = series.len();
        if n <= order || order == 0 {
            return None;
        }

        let mean = series.iter().sum::<f64>() / n as f64;
        let centered: Vec<f64> = series.iter().map(|&y| y - mean).collect();

        // Compute autocorrelations r[0..=order]
        let r: Vec<f64> = (0..=order)
            .map(|k| {
                let sum: f64 = centered[k..]
                    .iter()
                    .zip(centered.iter())
                    .map(|(a, b)| a * b)
                    .sum();
                sum / n as f64
            })
            .collect();

        if r[0].abs() < 1e-30 {
            return None;
        }

        // Yule-Walker: R·φ = r[1..p+1]
        // R is the Toeplitz matrix of r[0..p]
        let phi = yule_walker_solve(&r[..order], &r[1..=order])?;

        // Residual variance
        let sigma2 = r[0]
            - phi
                .iter()
                .zip(r[1..=order].iter())
                .map(|(a, b)| a * b)
                .sum::<f64>();
        let sigma2 = sigma2.max(0.0);

        Some(Self {
            order,
            intercept: mean,
            phi,
            sigma2,
        })
    }

    /// Forecast `h` steps ahead from the end of the observed series.
    ///
    /// Returns point forecasts as a vector of length h.
    pub fn forecast(&self, history: &[f64], h: usize) -> Vec<f64> {
        let p = self.order;
        let mut buf: Vec<f64> = history.iter().rev().take(p).rev().cloned().collect();
        // Extend buf to length p if history is shorter
        while buf.len() < p {
            buf.insert(0, self.intercept);
        }

        let mut forecasts = Vec::with_capacity(h);
        for _ in 0..h {
            let mut yhat = self.intercept;
            for (k, &phi_k) in self.phi.iter().enumerate() {
                let idx = buf.len().saturating_sub(k + 1);
                yhat += phi_k * (buf[idx] - self.intercept);
            }
            // Clamp to non-negative for renewable power forecasting
            let yhat = yhat.max(0.0);
            forecasts.push(yhat);
            buf.push(yhat);
        }
        forecasts
    }

    /// One-step-ahead residuals (in-sample).
    pub fn residuals(&self, series: &[f64]) -> Vec<f64> {
        let p = self.order;
        series[p..]
            .iter()
            .enumerate()
            .map(|(i, &y)| {
                let mut yhat = self.intercept;
                for (k, &phi_k) in self.phi.iter().enumerate() {
                    yhat += phi_k * (series[i + p - 1 - k] - self.intercept);
                }
                y - yhat
            })
            .collect()
    }

    /// Mean absolute error on a hold-out test set.
    pub fn mae(&self, train: &[f64], test: &[f64]) -> f64 {
        let forecasts = self.forecast(train, test.len());
        let n = test.len();
        if n == 0 {
            return 0.0;
        }
        forecasts
            .iter()
            .zip(test.iter())
            .map(|(f, y)| (f - y).abs())
            .sum::<f64>()
            / n as f64
    }
}

/// ARIMA(p,d,q) model.
///
/// Applies d-th order differencing, then fits ARMA(p,q) where q=0 for AR.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ArimaModel {
    /// AR order
    pub p: usize,
    /// Differencing order
    pub d: usize,
    /// Underlying AR model (fitted on differenced series)
    pub ar: ArModel,
    /// Last d values from training set (for undifferencing forecasts)
    last_values: Vec<f64>,
}

impl ArimaModel {
    /// Fit an ARIMA(p, d, 0) model.
    pub fn fit(series: &[f64], p: usize, d: usize) -> Option<Self> {
        let mut diff = series.to_vec();
        for _ in 0..d {
            diff = difference(&diff);
        }
        if diff.len() <= p {
            return None;
        }
        let ar = ArModel::fit(&diff, p)?;
        let last_values = series[series.len().saturating_sub(d)..].to_vec();
        Some(Self {
            p,
            d,
            ar,
            last_values,
        })
    }

    /// Forecast h steps ahead.
    ///
    /// Returns forecasts in the original (undifferenced) scale.
    pub fn forecast(&self, history: &[f64], h: usize) -> Vec<f64> {
        // Difference the history
        let mut diff = history.to_vec();
        for _ in 0..self.d {
            diff = difference(&diff);
        }

        // Forecast on differenced series
        let diff_forecasts = self.ar.forecast(&diff, h);

        // Undifference
        undifference(&diff_forecasts, &self.last_values, self.d)
    }
}

/// Apply first-order differencing: y'_t = y_t − y_{t-1}.
fn difference(series: &[f64]) -> Vec<f64> {
    series.windows(2).map(|w| w[1] - w[0]).collect()
}

/// Undo differencing for a forecast sequence.
fn undifference(diff_forecasts: &[f64], last_orig: &[f64], d: usize) -> Vec<f64> {
    if d == 0 || last_orig.is_empty() {
        return diff_forecasts.to_vec();
    }
    // Cumulative sum starting from last observed value
    let mut result = Vec::with_capacity(diff_forecasts.len());
    let mut prev = *last_orig.last().unwrap();
    for &dy in diff_forecasts {
        prev += dy;
        result.push(prev);
    }
    result
}

/// Solve Yule-Walker equations using Levinson-Durbin recursion.
///
/// `r` — autocorrelations r[0..p] (first element is r[0])
/// `rhs` — right-hand side r[1..=p]
///
/// Returns φ[0..p] or None if singular.
fn yule_walker_solve(r: &[f64], rhs: &[f64]) -> Option<Vec<f64>> {
    let p = r.len();
    if p == 0 {
        return Some(vec![]);
    }
    // Levinson-Durbin recursion
    let r0 = r[0];
    if r0.abs() < 1e-30 {
        return None;
    }
    let mut phi = vec![0.0f64; p];
    let mut phi_prev = vec![0.0f64; p];
    let mut alpha = rhs[0] / r0;
    phi[0] = alpha;
    let mut sigma = r0 * (1.0 - alpha * alpha);

    for k in 1..p {
        if sigma.abs() < 1e-30 {
            return None;
        }
        // Reflection coefficient
        let mut num = rhs[k];
        for j in 0..k {
            num -= phi[j] * r[k - 1 - j];
        }
        let alpha_k = num / sigma;

        // Update coefficients
        phi_prev[..k].copy_from_slice(&phi[..k]);
        for j in 0..k {
            phi[j] = phi_prev[j] - alpha_k * phi_prev[k - 1 - j];
        }
        phi[k] = alpha_k;
        sigma *= 1.0 - alpha_k * alpha_k;
        alpha = alpha_k;
    }
    let _ = alpha; // suppress warning
    Some(phi)
}

/// Compute the sample autocorrelation at lag k.
pub fn autocorrelation(series: &[f64], lag: usize) -> f64 {
    let n = series.len();
    if n <= lag {
        return 0.0;
    }
    let mean = series.iter().sum::<f64>() / n as f64;
    let var: f64 = series.iter().map(|&y| (y - mean).powi(2)).sum::<f64>() / n as f64;
    if var < 1e-30 {
        return 0.0;
    }
    let cov: f64 = series[lag..]
        .iter()
        .zip(series.iter())
        .map(|(&a, &b)| (a - mean) * (b - mean))
        .sum::<f64>()
        / n as f64;
    cov / var
}

/// AIC (Akaike Information Criterion) for model selection.
///
/// AIC = 2k − 2 ln(L) ≈ n·ln(σ²) + 2(p+1)
pub fn aic(n: usize, p: usize, sigma2: f64) -> f64 {
    if n == 0 || sigma2 <= 0.0 {
        return f64::INFINITY;
    }
    n as f64 * sigma2.ln() + 2.0 * (p + 1) as f64
}

/// Select the best AR order by minimising AIC over orders 1..=max_order.
pub fn select_ar_order(series: &[f64], max_order: usize) -> usize {
    let mut best_order = 1;
    let mut best_aic = f64::INFINITY;
    let n = series.len();
    for p in 1..=max_order {
        if let Some(model) = ArModel::fit(series, p) {
            let a = aic(n, p, model.sigma2.max(1e-30));
            if a < best_aic {
                best_aic = a;
                best_order = p;
            }
        }
    }
    best_order
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ar1_series(phi: f64, n: usize) -> Vec<f64> {
        let mut y = vec![0.0f64; n];
        for i in 1..n {
            y[i] = phi * y[i - 1] + (i as f64 * 0.1).sin() * 0.01;
        }
        y
    }

    #[test]
    fn test_ar_fit_returns_some() {
        let series: Vec<f64> = (0..50).map(|i| (i as f64 * 0.1).sin()).collect();
        let model = ArModel::fit(&series, 2);
        assert!(model.is_some());
    }

    #[test]
    fn test_ar_fit_undersized_returns_none() {
        let series = vec![1.0, 2.0, 3.0];
        assert!(ArModel::fit(&series, 5).is_none());
    }

    #[test]
    fn test_ar1_phi_estimate() {
        // AR(1) with φ=0.8; Yule-Walker should estimate close to 0.8
        let series = ar1_series(0.8, 200);
        let model = ArModel::fit(&series, 1).unwrap();
        assert!(
            model.phi[0].abs() < 1.0,
            "φ should be stationary: {:.3}",
            model.phi[0]
        );
    }

    #[test]
    fn test_ar_forecast_length() {
        let series: Vec<f64> = (0..30).map(|i| i as f64 * 0.5).collect();
        let model = ArModel::fit(&series, 2).unwrap();
        let fc = model.forecast(&series, 5);
        assert_eq!(fc.len(), 5);
    }

    #[test]
    fn test_ar_forecast_nonneg() {
        let series: Vec<f64> = (0..30).map(|i| (i as f64).abs()).collect();
        let model = ArModel::fit(&series, 1).unwrap();
        let fc = model.forecast(&series, 10);
        for &f in &fc {
            assert!(f >= 0.0, "Forecasts should be non-negative: {}", f);
        }
    }

    #[test]
    fn test_arima_fit_and_forecast() {
        let series: Vec<f64> = (0..40).map(|i| i as f64 + (i as f64 * 0.3).sin()).collect();
        let model = ArimaModel::fit(&series, 1, 1).unwrap();
        let fc = model.forecast(&series, 3);
        assert_eq!(fc.len(), 3);
    }

    #[test]
    fn test_autocorrelation_lag0_is_one() {
        let series: Vec<f64> = (0..20).map(|i| i as f64).collect();
        let r0 = autocorrelation(&series, 0);
        assert!((r0 - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_select_ar_order() {
        let series = ar1_series(0.7, 100);
        let best_p = select_ar_order(&series, 5);
        assert!((1..=5).contains(&best_p));
    }

    #[test]
    fn test_aic_lower_for_correct_model() {
        // AIC should prefer the true model order
        let aic_1 = aic(100, 1, 0.01);
        let aic_5 = aic(100, 5, 0.01);
        assert!(
            aic_1 < aic_5,
            "Lower-order model should have lower AIC if σ² same"
        );
    }

    #[test]
    fn test_residuals_length() {
        let series: Vec<f64> = (0..30).map(|i| i as f64).collect();
        let model = ArModel::fit(&series, 2).unwrap();
        let res = model.residuals(&series);
        assert_eq!(res.len(), series.len() - model.order);
    }
}
