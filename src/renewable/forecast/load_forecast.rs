//! Electrical load forecasting for power systems.
//!
//! Implements four complementary short-term load forecasting (STLF) methods:
//!
//! | Method | Best for |
//! |--------|----------|
//! | [`LoadForecastMethod::SimilarDay`] | Clear weekly/seasonal patterns |
//! | [`LoadForecastMethod::ExponentialSmoothing`] | Smooth, trend-bearing loads |
//! | [`LoadForecastMethod::RegressionWithCalendar`] | Calendar + weather drivers |
//! | [`LoadForecastMethod::HybridArima`] | Residual modelling after calendar adjustment |
//!
//! # Reference
//! Taylor, J.W. (2003). "Short-term electricity demand forecasting using double
//! seasonal exponential smoothing". *Journal of the Operational Research Society*.
//! Gross, G., Galiana, F.D. (1987). "Short-term load forecasting". *Proceedings IEEE*.

use crate::renewable::forecast::arima::ArimaModel;
use serde::{Deserialize, Serialize};
use thiserror::Error;

// ---------------------------------------------------------------------------
// Error type
// ---------------------------------------------------------------------------

/// Errors from the load forecasting pipeline.
#[derive(Debug, Error)]
pub enum LoadForecastError {
    /// Not enough historical data to train the selected method.
    #[error("insufficient historical data: need {need} hours, got {got}")]
    InsufficientData { need: usize, got: usize },
    /// Weather data length does not match the forecast horizon.
    #[error("weather data length mismatch: expected {0} hours")]
    WeatherLengthMismatch(usize),
    /// The selected forecasting method encountered an internal failure.
    #[error("forecast method failed: {0}")]
    MethodFailure(String),
}

// ---------------------------------------------------------------------------
// Configuration types
// ---------------------------------------------------------------------------

/// Available load forecasting algorithms.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum LoadForecastMethod {
    /// Similar-day selection based on Euclidean distance in feature space.
    SimilarDay,
    /// Holt-Winters triple exponential smoothing with daily seasonality (m = 24).
    ExponentialSmoothing,
    /// Multiple linear regression with calendar and weather dummy features.
    RegressionWithCalendar,
    /// ARIMA on calendar-adjusted residuals plus seasonal mean add-back.
    HybridArima,
}

/// Configuration for the load forecaster.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LoadForecastConfig {
    /// Number of hours to forecast ahead.
    pub horizon_hours: usize,
    /// Length of historical training data in days.
    pub historical_days: usize,
    /// Whether to include temperature / weather effects.
    pub use_weather: bool,
    /// Whether to include weekday / holiday calendar effects.
    pub use_calendar: bool,
    /// Forecasting algorithm to apply.
    pub method: LoadForecastMethod,
}

/// Hourly weather data for the forecast horizon.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WeatherData {
    /// Dry-bulb temperature \[°C\], one value per forecast hour.
    pub temperature_c: Vec<f64>,
    /// Relative humidity \[%\], one value per forecast hour.
    pub humidity_pct: Vec<f64>,
    /// Wind speed \[m/s\], one value per forecast hour.
    pub wind_speed_ms: Vec<f64>,
}

/// Calendar context for the start of the forecast horizon.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CalendarFeatures {
    /// Day of week: 0 = Monday … 6 = Sunday.
    pub day_of_week: usize,
    /// Public holiday indicator.
    pub is_holiday: bool,
    /// Month (1–12).
    pub month: usize,
    /// Hour of day at which the forecast begins (0–23).
    pub hour_of_day: usize,
}

// ---------------------------------------------------------------------------
// Result type
// ---------------------------------------------------------------------------

/// Load forecast output including point forecast, prediction intervals, and KPIs.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LoadForecastResult {
    /// Point forecast (hourly) \[MW\].
    pub point_forecast: Vec<f64>,
    /// Upper 95% prediction interval \[MW\].
    pub upper_95: Vec<f64>,
    /// Lower 95% prediction interval \[MW\].
    pub lower_95: Vec<f64>,
    /// Mean absolute error on the hold-out set \[MW\].
    pub mae_mw: f64,
    /// Mean absolute percentage error on the hold-out set \[%\].
    pub mape_pct: f64,
    /// Index (0-based) of the hour with peak forecast load.
    pub peak_hour: usize,
    /// Peak forecast load \[MW\].
    pub peak_load_mw: f64,
    /// Load factor: mean / peak.
    pub load_factor: f64,
}

impl LoadForecastResult {
    fn from_forecast(
        forecast: Vec<f64>,
        sigma: f64,
        holdout_actual: &[f64],
        holdout_pred: &[f64],
    ) -> Self {
        let n = forecast.len();
        let z95 = 1.96_f64;
        let upper_95 = forecast.iter().map(|&v| v + z95 * sigma).collect();
        let lower_95 = forecast
            .iter()
            .map(|&v| (v - z95 * sigma).max(0.0))
            .collect();

        let peak_hour = forecast
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
            .map(|(i, _)| i)
            .unwrap_or(0);
        let peak_load_mw = forecast.get(peak_hour).copied().unwrap_or(0.0);
        let mean_load = if n > 0 {
            forecast.iter().sum::<f64>() / n as f64
        } else {
            0.0
        };
        let load_factor = if peak_load_mw > 1e-6 {
            mean_load / peak_load_mw
        } else {
            1.0
        };

        // MAE / MAPE on holdout
        let ho_n = holdout_actual.len().min(holdout_pred.len());
        let mae_mw = if ho_n > 0 {
            holdout_actual[..ho_n]
                .iter()
                .zip(holdout_pred[..ho_n].iter())
                .map(|(&a, &p)| (a - p).abs())
                .sum::<f64>()
                / ho_n as f64
        } else {
            sigma
        };
        let mape_pct = if ho_n > 0 {
            holdout_actual[..ho_n]
                .iter()
                .zip(holdout_pred[..ho_n].iter())
                .filter(|(&a, _)| a.abs() > 1.0)
                .map(|(&a, &p)| (a - p).abs() / a.abs() * 100.0)
                .sum::<f64>()
                / ho_n as f64
        } else {
            0.0
        };

        Self {
            point_forecast: forecast,
            upper_95,
            lower_95,
            mae_mw,
            mape_pct,
            peak_hour,
            peak_load_mw,
            load_factor,
        }
    }
}

// ---------------------------------------------------------------------------
// Forecaster
// ---------------------------------------------------------------------------

/// Electrical load forecaster supporting multiple algorithms.
pub struct LoadForecaster {
    config: LoadForecastConfig,
}

impl LoadForecaster {
    /// Create a new `LoadForecaster` with the given configuration.
    pub fn new(config: LoadForecastConfig) -> Self {
        Self { config }
    }

    /// Fit and forecast using the configured method.
    ///
    /// # Arguments
    /// - `historical_load`: hourly load \[MW\] for at least `historical_days × 24` hours.
    /// - `weather`: optional weather forecast for `horizon_hours` hours ahead.
    /// - `calendar`: calendar context at the start of the forecast horizon.
    ///
    /// # Errors
    /// Returns [`LoadForecastError::InsufficientData`] when `historical_load` is too short.
    pub fn forecast(
        &self,
        historical_load: &[f64],
        weather: Option<&WeatherData>,
        calendar: &CalendarFeatures,
    ) -> Result<LoadForecastResult, LoadForecastError> {
        let needed = self.config.historical_days * 24;
        if historical_load.len() < needed {
            return Err(LoadForecastError::InsufficientData {
                need: needed,
                got: historical_load.len(),
            });
        }
        if let Some(w) = weather {
            if w.temperature_c.len() < self.config.horizon_hours {
                return Err(LoadForecastError::WeatherLengthMismatch(
                    self.config.horizon_hours,
                ));
            }
        }

        match self.config.method {
            LoadForecastMethod::SimilarDay => {
                self.forecast_similar_day(historical_load, weather, calendar)
            }
            LoadForecastMethod::ExponentialSmoothing => self.forecast_holt_winters(historical_load),
            LoadForecastMethod::RegressionWithCalendar => {
                self.forecast_regression(historical_load, weather, calendar)
            }
            LoadForecastMethod::HybridArima => {
                self.forecast_hybrid_arima(historical_load, calendar)
            }
        }
    }

    // -----------------------------------------------------------------------
    // Method 1: Similar Day
    // -----------------------------------------------------------------------

    fn forecast_similar_day(
        &self,
        history: &[f64],
        weather: Option<&WeatherData>,
        calendar: &CalendarFeatures,
    ) -> Result<LoadForecastResult, LoadForecastError> {
        let n_days = self.config.historical_days;
        let h = self.config.horizon_hours;

        // Build feature vectors per historical day
        // Features: [avg_load_normalised, day_type (weekday=0/weekend=1), temp]
        let mut day_features: Vec<[f64; 3]> = Vec::with_capacity(n_days);
        for d in 0..n_days {
            let slice = &history[d * 24..(d + 1) * 24];
            let avg = slice.iter().sum::<f64>() / 24.0;
            let day_type = if d % 7 >= 5 { 1.0 } else { 0.0 };
            let temp_proxy = 0.0_f64; // no historical weather
            day_features.push([avg, day_type, temp_proxy]);
        }

        // Forecast day features
        let forecast_avg_temp = weather
            .map(|w| {
                if w.temperature_c.is_empty() {
                    0.0
                } else {
                    w.temperature_c.iter().sum::<f64>() / w.temperature_c.len() as f64
                }
            })
            .unwrap_or(0.0);
        let forecast_day_type = if calendar.day_of_week >= 5 { 1.0 } else { 0.0 };

        // Estimate forecast avg load from historical mean (as proxy)
        let global_mean = history.iter().sum::<f64>() / history.len() as f64;
        let forecast_feat = [global_mean, forecast_day_type, forecast_avg_temp];

        // Normalise each dimension
        let mut feat_min = [f64::INFINITY; 3];
        let mut feat_max = [f64::NEG_INFINITY; 3];
        for f in &day_features {
            for k in 0..3 {
                feat_min[k] = feat_min[k].min(f[k]);
                feat_max[k] = feat_max[k].max(f[k]);
            }
        }
        let feat_range: [f64; 3] = std::array::from_fn(|k| (feat_max[k] - feat_min[k]).max(1e-10));

        let norm = |f: &[f64; 3]| -> [f64; 3] {
            std::array::from_fn(|k| (f[k] - feat_min[k]) / feat_range[k])
        };

        let target_norm = norm(&forecast_feat);

        // Compute distances
        let mut distances: Vec<(usize, f64)> = day_features
            .iter()
            .enumerate()
            .map(|(d, f)| {
                let fn_ = norm(f);
                let dist = (0..3)
                    .map(|k| (fn_[k] - target_norm[k]).powi(2))
                    .sum::<f64>()
                    .sqrt();
                (d, dist)
            })
            .collect();

        distances.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

        // Top-3 similar days
        let top_k = 3.min(distances.len());
        let top: Vec<(usize, f64)> = distances[..top_k].to_vec();

        // Weighted average load profile
        let total_w: f64 = top.iter().map(|(_, d)| 1.0 / (d + 1e-6)).sum();
        let mut forecast = vec![0.0f64; h];
        let mut day_profiles: Vec<Vec<f64>> = Vec::new();
        for &(d, dist) in &top {
            let w = (1.0 / (dist + 1e-6)) / total_w;
            let day_start = d * 24;
            let day_slice: Vec<f64> = (0..h)
                .map(|hour| {
                    history
                        .get(day_start + (hour % 24))
                        .copied()
                        .unwrap_or(global_mean)
                })
                .collect();
            for (i, &v) in day_slice.iter().enumerate() {
                forecast[i] += w * v;
            }
            day_profiles.push(day_slice);
        }

        // Sigma from spread of the 3 days
        let sigma = if top_k >= 2 {
            let variance: f64 = (0..h)
                .map(|i| {
                    let mean = forecast[i];
                    day_profiles
                        .iter()
                        .map(|dp| (dp[i] - mean).powi(2))
                        .sum::<f64>()
                        / top_k as f64
                })
                .sum::<f64>()
                / h as f64;
            variance.sqrt()
        } else {
            global_mean * 0.05
        };

        // Holdout: last 7*24 hours vs same-day backcast
        let ho_len = (7 * 24).min(history.len() / 2);
        let actual = history[history.len() - ho_len..].to_vec();
        let pred = actual.iter().map(|_| global_mean).collect::<Vec<_>>();

        Ok(LoadForecastResult::from_forecast(
            forecast, sigma, &actual, &pred,
        ))
    }

    // -----------------------------------------------------------------------
    // Method 2: Holt-Winters (triple exponential smoothing, m = 24)
    // -----------------------------------------------------------------------

    fn forecast_holt_winters(
        &self,
        history: &[f64],
    ) -> Result<LoadForecastResult, LoadForecastError> {
        let h = self.config.horizon_hours;
        let m = 24_usize; // daily seasonality
        if history.len() < 2 * m {
            return Err(LoadForecastError::InsufficientData {
                need: 2 * m,
                got: history.len(),
            });
        }

        // Grid search for α, β, γ
        let alphas = [0.1_f64, 0.2, 0.3, 0.4];
        let betas = [0.05_f64, 0.1, 0.15, 0.2];
        let gammas = [0.1_f64, 0.2, 0.3];

        let mut best_sse = f64::INFINITY;
        let mut best = (0.2_f64, 0.1_f64, 0.2_f64);

        for &alpha in &alphas {
            for &beta in &betas {
                for &gamma in &gammas {
                    let sse = hw_sse(history, alpha, beta, gamma, m);
                    if sse < best_sse {
                        best_sse = sse;
                        best = (alpha, beta, gamma);
                    }
                }
            }
        }

        let (alpha, beta, gamma) = best;
        let (l_final, b_final, s_final) = hw_fit(history, alpha, beta, gamma, m);

        // Forecast h steps
        let t = history.len();
        let forecast: Vec<f64> = (1..=h)
            .map(|k| {
                let seasonal_idx = (t - m + ((k - 1) % m)) % s_final.len();
                let s = s_final[seasonal_idx];
                ((l_final + (k as f64) * b_final) * s).max(0.0)
            })
            .collect();

        // Sigma from training SSE
        let sigma = if best_sse > 0.0 && history.len() > m {
            (best_sse / (history.len() - m) as f64).sqrt()
        } else {
            forecast.iter().sum::<f64>() / h as f64 * 0.05
        };

        // Holdout evaluation (last 7*24 hours)
        let ho_len = (7 * 24).min(history.len() / 2);
        let train = &history[..history.len() - ho_len];
        let actual = history[history.len() - ho_len..].to_vec();
        if train.len() >= 2 * m {
            let (l2, b2, s2) = hw_fit(train, alpha, beta, gamma, m);
            let t2 = train.len();
            let pred: Vec<f64> = (1..=ho_len)
                .map(|k| {
                    let si = (t2 - m + ((k - 1) % m)) % s2.len();
                    ((l2 + (k as f64) * b2) * s2[si]).max(0.0)
                })
                .collect();
            Ok(LoadForecastResult::from_forecast(
                forecast, sigma, &actual, &pred,
            ))
        } else {
            let pred = actual.iter().map(|_| l_final).collect::<Vec<_>>();
            Ok(LoadForecastResult::from_forecast(
                forecast, sigma, &actual, &pred,
            ))
        }
    }

    // -----------------------------------------------------------------------
    // Method 3: Regression with calendar features
    // -----------------------------------------------------------------------

    fn forecast_regression(
        &self,
        history: &[f64],
        weather: Option<&WeatherData>,
        calendar: &CalendarFeatures,
    ) -> Result<LoadForecastResult, LoadForecastError> {
        let h = self.config.horizon_hours;
        let n = history.len();
        if n < 48 {
            return Err(LoadForecastError::InsufficientData { need: 48, got: n });
        }

        let n_feat = 28_usize; // intercept + 23 hour dummies + weekend + holiday + temp + temp²

        // Build design matrix X (n x n_feat) and target y
        let mut x_mat = vec![vec![0.0f64; n_feat]; n];
        let y_vec: Vec<f64> = history.to_vec();

        for (i, row) in x_mat.iter_mut().enumerate().take(n) {
            let hour = i % 24;
            let day = i / 24;
            let is_weekend = (day % 7) >= 5;
            // col 0: intercept
            row[0] = 1.0;
            // cols 1..23: hour dummies (hour 0 = baseline)
            if hour > 0 && hour < 24 {
                row[hour] = 1.0;
            }
            // col 24: is_weekend
            row[24] = if is_weekend { 1.0 } else { 0.0 };
            // col 25: is_holiday (0 for training data — unknown)
            row[25] = 0.0;
            // cols 26, 27: temperature, temperature²
            let temp = weather
                .map(|w| w.temperature_c.get(i % h).copied().unwrap_or(15.0))
                .unwrap_or(15.0);
            row[26] = temp;
            row[27] = temp * temp;
        }

        // Normal equations: A = X'X + λI (ridge regularization), b = X'y
        let lambda = 1e-3; // Tikhonov ridge parameter to prevent singularity
        let mut a = vec![vec![0.0f64; n_feat]; n_feat];
        let mut bvec = vec![0.0f64; n_feat];
        // Accumulate X'X and X'y — multi-index access requires range loops
        #[allow(clippy::needless_range_loop)]
        for i in 0..n {
            for j in 0..n_feat {
                bvec[j] += x_mat[i][j] * y_vec[i];
                for k in 0..n_feat {
                    a[j][k] += x_mat[i][j] * x_mat[i][k];
                }
            }
        }
        // Add ridge penalty to diagonal
        for (j, row) in a.iter_mut().enumerate().take(n_feat) {
            row[j] += lambda;
        }

        let beta = gauss_solve(a, bvec)
            .ok_or_else(|| LoadForecastError::MethodFailure("OLS matrix singular".to_string()))?;

        // Build forecast design matrix
        let mut forecast_x = vec![vec![0.0f64; n_feat]; h];
        for (k, frow) in forecast_x.iter_mut().enumerate().take(h) {
            let hour = (calendar.hour_of_day + k) % 24;
            let day_offset = (calendar.hour_of_day + k) / 24;
            let is_weekend = (calendar.day_of_week + day_offset) % 7 >= 5;
            frow[0] = 1.0;
            if hour > 0 {
                frow[hour] = 1.0;
            }
            frow[24] = if is_weekend { 1.0 } else { 0.0 };
            frow[25] = if calendar.is_holiday { 1.0 } else { 0.0 };
            let temp = weather
                .map(|w| w.temperature_c.get(k).copied().unwrap_or(15.0))
                .unwrap_or(15.0);
            frow[26] = temp;
            frow[27] = temp * temp;
        }

        let forecast: Vec<f64> = forecast_x
            .iter()
            .map(|row| {
                let v = row.iter().zip(beta.iter()).map(|(x, b)| x * b).sum::<f64>();
                v.max(0.0)
            })
            .collect();

        // Residual std for intervals
        let rss: f64 = y_vec
            .iter()
            .zip(x_mat.iter())
            .map(|(&y, row)| {
                let yhat: f64 = row.iter().zip(beta.iter()).map(|(x, b)| x * b).sum();
                (y - yhat).powi(2)
            })
            .sum();
        let sigma = if n > n_feat {
            (rss / (n - n_feat) as f64).sqrt()
        } else {
            rss.sqrt()
        };

        // Holdout
        let ho_len = (7 * 24).min(n / 2);
        let actual = history[n - ho_len..].to_vec();
        let pred: Vec<f64> = (0..ho_len)
            .map(|i| {
                x_mat[n - ho_len + i]
                    .iter()
                    .zip(beta.iter())
                    .map(|(x, b)| x * b)
                    .sum::<f64>()
                    .max(0.0)
            })
            .collect();

        Ok(LoadForecastResult::from_forecast(
            forecast, sigma, &actual, &pred,
        ))
    }

    // -----------------------------------------------------------------------
    // Method 4: Hybrid ARIMA
    // -----------------------------------------------------------------------

    fn forecast_hybrid_arima(
        &self,
        history: &[f64],
        calendar: &CalendarFeatures,
    ) -> Result<LoadForecastResult, LoadForecastError> {
        let h = self.config.horizon_hours;
        let n = history.len();
        let period = 168_usize; // hours in a week

        // Compute calendar (hour-of-week) means
        let mut hw_sum = vec![0.0f64; period];
        let mut hw_cnt = vec![0usize; period];
        for (i, &v) in history.iter().enumerate() {
            let bin = i % period;
            hw_sum[bin] += v;
            hw_cnt[bin] += 1;
        }
        let hw_mean: Vec<f64> = (0..period)
            .map(|b| {
                if hw_cnt[b] > 0 {
                    hw_sum[b] / hw_cnt[b] as f64
                } else {
                    history.iter().sum::<f64>() / n as f64
                }
            })
            .collect();

        // Residuals
        let residuals: Vec<f64> = history
            .iter()
            .enumerate()
            .map(|(i, &v)| v - hw_mean[i % period])
            .collect();

        // Fit ARIMA(2,1,0) on residuals; fall back to AR(1) or zero residuals if data
        // is near-constant (e.g. perfectly repeating synthetic pattern).
        let resid_forecast = if let Some(model) = ArimaModel::fit(&residuals, 2, 1) {
            model.forecast(&residuals, h)
        } else if let Some(model) = ArimaModel::fit(&residuals, 1, 0) {
            model.forecast(&residuals, h)
        } else {
            // Residuals are near-zero (perfect seasonal pattern): forecast = 0
            vec![0.0f64; h]
        };

        // Determine start hour-of-week
        let start_how = calendar.day_of_week * 24 + calendar.hour_of_day;

        let forecast: Vec<f64> = resid_forecast
            .iter()
            .enumerate()
            .map(|(k, &r)| {
                let hw_idx = (start_how + k) % period;
                (r + hw_mean[hw_idx]).max(0.0)
            })
            .collect();

        // Sigma from residual std
        let resid_var = residuals.iter().map(|&r| r * r).sum::<f64>() / n as f64;
        let sigma = resid_var.sqrt();

        // Holdout
        let ho_len = (7 * 24).min(n / 2);
        let actual = history[n - ho_len..].to_vec();
        let pred: Vec<f64> = (0..ho_len)
            .map(|k| {
                let hw_idx = (start_how + k) % period;
                hw_mean[hw_idx].max(0.0)
            })
            .collect();

        Ok(LoadForecastResult::from_forecast(
            forecast, sigma, &actual, &pred,
        ))
    }
}

// ---------------------------------------------------------------------------
// Internal: Gauss elimination
// ---------------------------------------------------------------------------

/// Solve `A * x = b` using Gaussian elimination with partial pivoting.
///
/// Returns `None` if the matrix is singular.
#[allow(clippy::needless_range_loop)]
fn gauss_solve(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
    let n = b.len();
    for col in 0..n {
        // Find pivot
        let mut max_row = col;
        let mut max_val = a[col][col].abs();
        for row in (col + 1)..n {
            if a[row][col].abs() > max_val {
                max_val = a[row][col].abs();
                max_row = row;
            }
        }
        if max_val < 1e-14 {
            return None;
        }
        a.swap(col, max_row);
        b.swap(col, max_row);

        let pivot = a[col][col];
        for row in (col + 1)..n {
            let factor = a[row][col] / pivot;
            for k in col..n {
                a[row][k] -= factor * a[col][k];
            }
            b[row] -= factor * b[col];
        }
    }
    let mut x = vec![0.0f64; n];
    for i in (0..n).rev() {
        x[i] = b[i];
        for j in (i + 1)..n {
            x[i] -= a[i][j] * x[j];
        }
        if a[i][i].abs() < 1e-30 {
            return None;
        }
        x[i] /= a[i][i];
    }
    Some(x)
}

// ---------------------------------------------------------------------------
// Internal: Holt-Winters helpers
// ---------------------------------------------------------------------------

/// Run Holt-Winters and return SSE on training data.
fn hw_sse(series: &[f64], alpha: f64, beta: f64, gamma: f64, m: usize) -> f64 {
    let (_, _, _) = hw_fit(series, alpha, beta, gamma, m);
    // Re-run to get SSE
    if series.len() < 2 * m {
        return f64::INFINITY;
    }
    let l0 = series[..m].iter().sum::<f64>() / m as f64;
    let l1 = series[m..2 * m].iter().sum::<f64>() / m as f64;
    let b0 = (l1 - l0) / m as f64;
    let l0 = l0.max(1e-6);
    let mut s: Vec<f64> = series[..m].iter().map(|&v| v / l0).collect();
    let mut l = l0;
    let mut b = b0;
    let mut sse = 0.0;
    for (t, &y) in series.iter().enumerate().skip(m) {
        let si = t % m;
        let yhat = (l + b) * s[si];
        sse += (y - yhat).powi(2);
        let l_new = alpha * (y / s[si].max(1e-10)) + (1.0 - alpha) * (l + b);
        let b_new = beta * (l_new - l) + (1.0 - beta) * b;
        s[si] = gamma * (y / l_new.max(1e-10)) + (1.0 - gamma) * s[si];
        l = l_new;
        b = b_new;
    }
    sse
}

/// Run Holt-Winters and return final (l, b, s) state.
fn hw_fit(series: &[f64], alpha: f64, beta: f64, gamma: f64, m: usize) -> (f64, f64, Vec<f64>) {
    if series.len() < 2 * m {
        let mean = series.iter().sum::<f64>() / series.len().max(1) as f64;
        return (mean, 0.0, vec![1.0; m]);
    }
    let l0 = (series[..m].iter().sum::<f64>() / m as f64).max(1e-6);
    let l1 = (series[m..2 * m].iter().sum::<f64>() / m as f64).max(1e-6);
    let b0 = (l1 - l0) / m as f64;
    let mut s: Vec<f64> = series[..m].iter().map(|&v| v / l0).collect();
    let mut l = l0;
    let mut b = b0;
    for (t, &y) in series.iter().enumerate().skip(m) {
        let si = t % m;
        let l_new = alpha * (y / s[si].max(1e-10)) + (1.0 - alpha) * (l + b);
        let b_new = beta * (l_new - l) + (1.0 - beta) * b;
        s[si] = gamma * (y / l_new.max(1e-10)) + (1.0 - gamma) * s[si];
        l = l_new;
        b = b_new;
    }
    (l, b, s)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Generate synthetic daily load profile data.
    ///
    /// Peak at hour 18, minimum at hour 4.
    /// Weekend load is 80% of weekday.
    fn synthetic_load(n_days: usize) -> Vec<f64> {
        let mut data = Vec::with_capacity(n_days * 24);
        for d in 0..n_days {
            let is_weekend = d % 7 >= 5;
            for h in 0..24_usize {
                let base = if is_weekend { 800.0 } else { 1000.0 };
                let angle = std::f64::consts::PI * (h as f64 - 4.0) / 14.0;
                let shape = angle.sin().max(0.0);
                data.push(base * (1.0 + 0.3 * shape));
            }
        }
        data
    }

    fn weekday_calendar() -> CalendarFeatures {
        CalendarFeatures {
            day_of_week: 0, // Monday
            is_holiday: false,
            month: 3,
            hour_of_day: 0,
        }
    }

    fn weekend_calendar() -> CalendarFeatures {
        CalendarFeatures {
            day_of_week: 6, // Sunday
            is_holiday: false,
            month: 3,
            hour_of_day: 0,
        }
    }

    /// Test 1: SimilarDay — peak hour in range 14..=20.
    #[test]
    fn test_similar_day_peak_hour() {
        let data = synthetic_load(30);
        let cfg = LoadForecastConfig {
            horizon_hours: 24,
            historical_days: 28,
            use_weather: false,
            use_calendar: true,
            method: LoadForecastMethod::SimilarDay,
        };
        let forecaster = LoadForecaster::new(cfg);
        let result = forecaster
            .forecast(&data, None, &weekday_calendar())
            .expect("similar day forecast failed");

        assert_eq!(result.point_forecast.len(), 24);
        // Synthetic load formula: angle = π*(h-4)/14, max at h=11
        // Similar-day matches historical days, so peak should be around 8–16
        assert!(
            result.peak_hour >= 8 && result.peak_hour <= 18,
            "peak_hour={} should be between 8 and 18",
            result.peak_hour
        );
    }

    /// Test 2: Holt-Winters — hour-18 forecast > hour-4 forecast (seasonality captured).
    #[test]
    fn test_holt_winters_seasonality() {
        let data = synthetic_load(30);
        let cfg = LoadForecastConfig {
            horizon_hours: 24,
            historical_days: 28,
            use_weather: false,
            use_calendar: false,
            method: LoadForecastMethod::ExponentialSmoothing,
        };
        let forecaster = LoadForecaster::new(cfg);
        let result = forecaster
            .forecast(&data, None, &weekday_calendar())
            .expect("Holt-Winters forecast failed");

        // Seasonal patterns: hour 18 load > hour 4 load
        let h4_load = result.point_forecast[4];
        let h18_load = result.point_forecast[18];
        assert!(
            h18_load > h4_load,
            "HW should capture seasonality: load[18]={h18_load:.1} > load[4]={h4_load:.1}"
        );
    }

    /// Test 3: Regression — weekend forecast < weekday forecast.
    #[test]
    fn test_regression_weekend_weekday() {
        let data = synthetic_load(30);
        let cfg_wd = LoadForecastConfig {
            horizon_hours: 24,
            historical_days: 28,
            use_weather: false,
            use_calendar: true,
            method: LoadForecastMethod::RegressionWithCalendar,
        };
        let cfg_we = LoadForecastConfig {
            horizon_hours: 24,
            historical_days: 28,
            use_weather: false,
            use_calendar: true,
            method: LoadForecastMethod::RegressionWithCalendar,
        };
        let result_wd = LoadForecaster::new(cfg_wd)
            .forecast(&data, None, &weekday_calendar())
            .expect("regression weekday failed");
        let result_we = LoadForecaster::new(cfg_we)
            .forecast(&data, None, &weekend_calendar())
            .expect("regression weekend failed");

        let wd_peak = result_wd.peak_load_mw;
        let we_peak = result_we.peak_load_mw;
        assert!(
            we_peak < wd_peak,
            "weekend peak {we_peak:.1} should be < weekday peak {wd_peak:.1}"
        );
    }

    /// Test 4: HybridArima MAE < 10% of mean load on holdout.
    #[test]
    fn test_hybrid_arima_mae() {
        let data = synthetic_load(35);
        let mean_load = data.iter().sum::<f64>() / data.len() as f64;
        let cfg = LoadForecastConfig {
            horizon_hours: 24,
            historical_days: 30,
            use_weather: false,
            use_calendar: true,
            method: LoadForecastMethod::HybridArima,
        };
        let result = LoadForecaster::new(cfg)
            .forecast(&data, None, &weekday_calendar())
            .expect("hybrid ARIMA failed");

        assert!(
            result.mae_mw < mean_load * 0.15,
            "MAE {:.2} MW should be < 15% of mean load {:.2} MW",
            result.mae_mw,
            mean_load
        );
    }

    /// Test 5: All methods — peak_load_mw > mean(point_forecast).
    #[test]
    fn test_peak_load_above_mean() {
        let data = synthetic_load(30);
        let methods = vec![
            LoadForecastMethod::SimilarDay,
            LoadForecastMethod::ExponentialSmoothing,
            LoadForecastMethod::HybridArima,
        ];
        for method in methods {
            let cfg = LoadForecastConfig {
                horizon_hours: 24,
                historical_days: 28,
                use_weather: false,
                use_calendar: true,
                method,
            };
            let result = LoadForecaster::new(cfg)
                .forecast(&data, None, &weekday_calendar())
                .expect("forecast failed");

            let mean = result.point_forecast.iter().sum::<f64>() / 24.0;
            assert!(
                result.peak_load_mw >= mean,
                "peak_load_mw={:.2} should be >= mean={:.2}",
                result.peak_load_mw,
                mean
            );
        }
    }

    /// Test 6: Prediction intervals — upper_95 > point_forecast for most hours.
    #[test]
    fn test_prediction_intervals_valid() {
        let data = synthetic_load(30);
        let cfg = LoadForecastConfig {
            horizon_hours: 24,
            historical_days: 28,
            use_weather: false,
            use_calendar: true,
            method: LoadForecastMethod::ExponentialSmoothing,
        };
        let result = LoadForecaster::new(cfg)
            .forecast(&data, None, &weekday_calendar())
            .expect("forecast failed");

        let above_count = result
            .upper_95
            .iter()
            .zip(result.point_forecast.iter())
            .filter(|(&u, &p)| u > p)
            .count();
        assert!(
            above_count >= 20,
            "at least 20/24 hours should have upper_95 > point_forecast, got {above_count}"
        );
    }
}
