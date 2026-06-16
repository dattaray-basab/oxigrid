/// Neural network forecasting bridge for renewable energy prediction.
///
/// Provides a trait-based interface that allows swapping between:
///   - Built-in statistical models (persistence, ARIMA) — always available
///   - External ML runtimes (torsh, trustformers, tract/onnx) — via feature flags
///
/// # Architecture
///
/// ```text
/// ForecastModel (trait)
///       │
///       ├── PersistenceBridge   — wraps persistence.rs
///       ├── ArimaBridge         — wraps arima.rs
///       ├── EnsembleBridge      — weighted combination of models
///       └── ExternalNnBridge    — polynomial fallback + native-runtime hook (torsh/trustformers)
/// ```
///
/// # Usage
///
/// ```rust,ignore
/// use oxigrid::renewable::forecast::nn_bridge::{ForecastModel, EnsembleBridge};
/// let ensemble = EnsembleBridge::equal_weight(vec![...]);
/// let forecast = ensemble.predict(&history, 24);
/// ```
use crate::renewable::forecast::arima::{select_ar_order, ArimaModel};
use crate::renewable::forecast::persistence::DiurnalPersistence;
use serde::{Deserialize, Serialize};

// ── Core trait ────────────────────────────────────────────────────────────────

/// A univariate time-series forecaster.
pub trait ForecastModel: Send + Sync {
    /// Name of this model (for logging/selection).
    fn name(&self) -> &str;

    /// Fit the model to historical observations.
    ///
    /// `history` — ordered time series (oldest first, hourly values assumed)
    fn fit(&mut self, history: &[f64]);

    /// Produce a multi-step forecast.
    ///
    /// `history` — recent observations to condition on
    /// `horizon` — number of steps ahead to predict
    ///
    /// Returns a vector of `horizon` point forecasts.
    fn predict(&self, history: &[f64], horizon: usize) -> Vec<f64>;

    /// Predict with uncertainty intervals (default: point ± std_dev estimate).
    ///
    /// Returns `(lower, point, upper)` vectors, each of length `horizon`.
    fn predict_intervals(
        &self,
        history: &[f64],
        horizon: usize,
        coverage: f64,
    ) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        let point = self.predict(history, horizon);
        // Default: simple ±k·σ where σ = std of recent residuals
        let sigma = estimate_residual_std(history, &point);
        let z = normal_quantile(0.5 + coverage / 2.0);
        let lower = point.iter().map(|&p| p - z * sigma).collect();
        let upper = point.iter().map(|&p| p + z * sigma).collect();
        (lower, point, upper)
    }

    /// Compute skill score relative to persistence on held-out data.
    fn skill_score(&self, history: &[f64], test_horizon: usize) -> f64 {
        if history.len() <= test_horizon + 1 {
            return 0.0;
        }
        let n = history.len() - test_horizon;
        let train = &history[..n];
        let actual = &history[n..];
        let forecast = self.predict(train, test_horizon);
        let mae_model: f64 = forecast
            .iter()
            .zip(actual)
            .map(|(f, a)| (f - a).abs())
            .sum::<f64>()
            / test_horizon as f64;
        // Persistence benchmark
        let last = *train.last().unwrap_or(&0.0);
        let mae_persist: f64 =
            actual.iter().map(|&a| (a - last).abs()).sum::<f64>() / test_horizon as f64;
        if mae_persist < 1e-10 {
            0.0
        } else {
            1.0 - mae_model / mae_persist
        }
    }
}

// ── Persistence bridge ────────────────────────────────────────────────────────

/// Wraps the built-in persistence forecaster.
pub struct PersistenceBridge {
    pub use_diurnal: bool,
}

impl PersistenceBridge {
    pub fn naive() -> Self {
        Self { use_diurnal: false }
    }
    pub fn diurnal() -> Self {
        Self { use_diurnal: true }
    }
}

impl ForecastModel for PersistenceBridge {
    fn name(&self) -> &str {
        if self.use_diurnal {
            "persistence-diurnal"
        } else {
            "persistence-naive"
        }
    }

    fn fit(&mut self, _history: &[f64]) {}

    fn predict(&self, history: &[f64], horizon: usize) -> Vec<f64> {
        if history.is_empty() {
            return vec![0.0; horizon];
        }
        if self.use_diurnal && history.len() >= 24 {
            // Load the last 24 values into the diurnal buffer
            let mut dp = DiurnalPersistence::new(24);
            let start = history.len().saturating_sub(24);
            for &v in &history[start..] {
                dp.update(v);
            }
            let period_forecast = dp.forecast_next_period();
            period_forecast.into_iter().cycle().take(horizon).collect()
        } else {
            let last = *history
                .last()
                .expect("invariant: history non-empty after branch guard");
            vec![last; horizon]
        }
    }
}

// ── ARIMA bridge ──────────────────────────────────────────────────────────────

/// Wraps the built-in ARIMA model with AIC-based order selection.
pub struct ArimaBridge {
    /// Maximum AR order to try during auto-selection.
    pub max_p: usize,
    /// Differencing order.
    pub d: usize,
    fitted_model: Option<ArimaModel>,
}

impl ArimaBridge {
    pub fn new(max_p: usize, d: usize) -> Self {
        Self {
            max_p,
            d,
            fitted_model: None,
        }
    }

    pub fn auto() -> Self {
        Self::new(4, 1)
    }
}

impl ForecastModel for ArimaBridge {
    fn name(&self) -> &str {
        "arima"
    }

    fn fit(&mut self, history: &[f64]) {
        let p = select_ar_order(history, self.max_p);
        self.fitted_model = ArimaModel::fit(history, p, self.d);
    }

    fn predict(&self, history: &[f64], horizon: usize) -> Vec<f64> {
        if history.is_empty() {
            return vec![0.0; horizon];
        }
        match &self.fitted_model {
            Some(model) => model.forecast(history, horizon),
            None => {
                let last = *history
                    .last()
                    .expect("invariant: history non-empty after branch guard");
                vec![last; horizon]
            }
        }
    }
}

// ── Ensemble bridge ───────────────────────────────────────────────────────────

/// Weighted ensemble combining multiple `ForecastModel`s.
pub struct EnsembleBridge {
    models: Vec<Box<dyn ForecastModel>>,
    weights: Vec<f64>,
    pub name: String,
}

impl EnsembleBridge {
    /// Create an ensemble with explicit weights (will be normalised to sum=1).
    pub fn new(models: Vec<Box<dyn ForecastModel>>, weights: Vec<f64>) -> Self {
        assert_eq!(
            models.len(),
            weights.len(),
            "models and weights must have same length"
        );
        let sum: f64 = weights.iter().sum();
        let weights = if sum > 1e-10 {
            weights.iter().map(|w| w / sum).collect()
        } else {
            vec![1.0 / models.len() as f64; models.len()]
        };
        let names: Vec<&str> = models.iter().map(|m| m.name()).collect();
        let name = format!("ensemble({})", names.join("+"));
        Self {
            models,
            weights,
            name,
        }
    }

    /// Equal-weight ensemble.
    pub fn equal_weight(models: Vec<Box<dyn ForecastModel>>) -> Self {
        let n = models.len();
        Self::new(models, vec![1.0 / n as f64; n])
    }

    /// Fit all member models.
    pub fn fit_all(&mut self, history: &[f64]) {
        for model in &mut self.models {
            model.fit(history);
        }
    }
}

impl ForecastModel for EnsembleBridge {
    fn name(&self) -> &str {
        &self.name
    }

    fn fit(&mut self, history: &[f64]) {
        self.fit_all(history);
    }

    fn predict(&self, history: &[f64], horizon: usize) -> Vec<f64> {
        let mut result = vec![0.0_f64; horizon];
        for (model, &w) in self.models.iter().zip(self.weights.iter()) {
            for (r, f) in result.iter_mut().zip(model.predict(history, horizon)) {
                *r += w * f;
            }
        }
        result
    }
}

// ── External NN bridge (polynomial fallback + native-runtime hook) ───────────

/// Metadata describing an external neural network model.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NnModelSpec {
    /// Model identifier/name
    pub name: String,
    /// Input window length (number of historical steps needed)
    pub input_len: usize,
    /// Output horizon this model produces
    pub output_len: usize,
    /// Model architecture description
    pub architecture: String,
    /// Path to model file (ONNX, SafeTensors, etc.)
    pub model_path: Option<String>,
}

impl NnModelSpec {
    pub fn new(name: &str, input_len: usize, output_len: usize, architecture: &str) -> Self {
        Self {
            name: name.to_string(),
            input_len,
            output_len,
            architecture: architecture.to_string(),
            model_path: None,
        }
    }

    pub fn with_path(mut self, path: &str) -> Self {
        self.model_path = Some(path.to_string());
        self
    }
}

/// JSON-serialisable config for the polynomial regression fallback model.
///
/// File format (example):
/// ```json
/// {
///   "coefficients": [200.0, 15.0, -0.5, 0.1],
///   "window": 24
/// }
/// ```
///
/// `coefficients[0]` is the bias term; `coefficients[1]` scales the history
/// mean; `coefficients[2]` scales the linear trend slope (units per step);
/// `coefficients[3]` (if present) scales the step index within the forecast
/// horizon (0-based).  Additional coefficients are ignored.
///
/// `window` is the number of recent history points used to compute mean and
/// trend.  If `window` is 0 or larger than the history length, all available
/// history is used.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PolyModelConfig {
    /// Polynomial coefficients: `[bias, mean_coef, slope_coef, horizon_coef?]`.
    pub coefficients: Vec<f64>,
    /// Number of recent history points used to compute features.
    pub window: usize,
}

impl Default for PolyModelConfig {
    fn default() -> Self {
        // Trivial identity: predict the history mean, ignoring slope/horizon.
        Self {
            coefficients: vec![0.0, 1.0],
            window: 24,
        }
    }
}

/// Bridge for external neural network runtimes with a polynomial regression
/// fallback.
///
/// When an ONNX/tract/torsh runtime becomes available it can be wired in under
/// a feature flag.  Until then the bridge loads a [`PolyModelConfig`] from the
/// JSON file at `spec.model_path` and evaluates a lightweight polynomial on
/// recent history features (mean and trend slope) to produce non-trivial
/// forecasts.
///
/// If `spec.model_path` is `None` or the file cannot be parsed, `try_load`
/// returns `false` and `predict` delegates to the ARIMA fallback.
pub struct ExternalNnBridge {
    pub spec: NnModelSpec,
    fallback: Box<dyn ForecastModel>,
    is_loaded: bool,
    /// Polynomial config loaded from `spec.model_path`.
    poly_config: Option<PolyModelConfig>,
}

impl ExternalNnBridge {
    pub fn new(spec: NnModelSpec) -> Self {
        let fallback: Box<dyn ForecastModel> = Box::new(ArimaBridge::auto());
        Self {
            spec,
            fallback,
            is_loaded: false,
            poly_config: None,
        }
    }

    /// Attempt to load the polynomial model config from `spec.model_path`.
    ///
    /// Returns `true` and sets `is_loaded` when the JSON config is read and
    /// parsed successfully.  Returns `false` (using ARIMA fallback) when:
    /// - `spec.model_path` is `None`
    /// - the file cannot be opened or read
    /// - the JSON cannot be deserialised into [`PolyModelConfig`]
    ///
    /// When a native ONNX/tract runtime is wired in under a feature flag, this
    /// method is the entry point to load the session instead.
    pub fn try_load(&mut self) -> bool {
        let path = match &self.spec.model_path {
            Some(p) => p.clone(),
            None => {
                self.is_loaded = false;
                return false;
            }
        };

        let contents = match std::fs::read_to_string(&path) {
            Ok(s) => s,
            Err(_) => {
                self.is_loaded = false;
                return false;
            }
        };

        match serde_json::from_str::<PolyModelConfig>(&contents) {
            Ok(cfg) => {
                self.poly_config = Some(cfg);
                self.is_loaded = true;
                true
            }
            Err(_) => {
                self.is_loaded = false;
                false
            }
        }
    }

    pub fn is_loaded(&self) -> bool {
        self.is_loaded
    }

    /// Compute (mean, slope) features over the last `window` points of `history`.
    ///
    /// `slope` is the ordinary-least-squares slope of the windowed series
    /// (units per step).  Returns `(0.0, 0.0)` when `window_data` is empty.
    fn compute_features(history: &[f64], window: usize) -> (f64, f64) {
        if history.is_empty() {
            return (0.0, 0.0);
        }
        let effective_window = if window == 0 || window > history.len() {
            history.len()
        } else {
            window
        };
        let slice = &history[history.len() - effective_window..];
        let n = slice.len() as f64;
        let mean = slice.iter().sum::<f64>() / n;

        // OLS slope: β = Σ(x_i - x̄)(y_i - ȳ) / Σ(x_i - x̄)²
        // x_i = i (0-based), x̄ = (n-1)/2
        let x_mean = (n - 1.0) / 2.0;
        let mut num = 0.0_f64;
        let mut den = 0.0_f64;
        for (i, &y) in slice.iter().enumerate() {
            let dx = i as f64 - x_mean;
            num += dx * (y - mean);
            den += dx * dx;
        }
        let slope = if den.abs() < 1e-12 { 0.0 } else { num / den };
        (mean, slope)
    }
}

impl ForecastModel for ExternalNnBridge {
    fn name(&self) -> &str {
        &self.spec.name
    }

    fn fit(&mut self, history: &[f64]) {
        if !self.is_loaded {
            self.fallback.fit(history);
        }
    }

    /// Produce a multi-step forecast.
    ///
    /// When the polynomial model is loaded the prediction at step `h` (0-based)
    /// is:
    ///
    /// ```text
    /// ŷ_h = coef[0]
    ///     + coef[1] * mean
    ///     + coef[2] * slope          (if present)
    ///     + coef[3] * (h as f64)     (if present)
    /// ```
    ///
    /// where `mean` and `slope` are computed over the last `config.window`
    /// observations.
    fn predict(&self, history: &[f64], horizon: usize) -> Vec<f64> {
        if !self.is_loaded {
            return self.fallback.predict(history, horizon);
        }

        let cfg = match &self.poly_config {
            Some(c) => c,
            // is_loaded should not be true without poly_config, but be safe.
            None => return self.fallback.predict(history, horizon),
        };

        let (mean, slope) = Self::compute_features(history, cfg.window);
        let coefficients = &cfg.coefficients;

        (0..horizon)
            .map(|h| {
                let mut y = coefficients.first().copied().unwrap_or(0.0);
                if let Some(&c1) = coefficients.get(1) {
                    y += c1 * mean;
                }
                if let Some(&c2) = coefficients.get(2) {
                    y += c2 * slope;
                }
                if let Some(&c3) = coefficients.get(3) {
                    y += c3 * h as f64;
                }
                y
            })
            .collect()
    }
}

// ── Helpers ───────────────────────────────────────────────────────────────────

fn estimate_residual_std(history: &[f64], forecast: &[f64]) -> f64 {
    if history.len() < 2 {
        return 1.0;
    }
    // Use recent diffs as proxy for forecast uncertainty
    let diffs: Vec<f64> = history.windows(2).map(|w| (w[1] - w[0]).abs()).collect();
    let mean = diffs.iter().sum::<f64>() / diffs.len() as f64;
    let horizon_factor = (1.0 + forecast.len() as f64 * 0.05).sqrt(); // grows with horizon
    (mean * horizon_factor).max(0.0)
}

/// Approximate normal distribution quantile (Beasley-Springer-Moro).
fn normal_quantile(p: f64) -> f64 {
    let p = p.clamp(1e-9, 1.0 - 1e-9);
    // Rational approximation (Abramowitz & Stegun 26.2.17)
    let t = if p < 0.5 {
        (-2.0 * p.ln()).sqrt()
    } else {
        (-2.0 * (1.0 - p).ln()).sqrt()
    };
    let c = [2.515_517, 0.802_853, 0.010_328];
    let d = [1.432_788, 0.189_269, 0.001_308];
    let num = c[0] + c[1] * t + c[2] * t * t;
    let den = 1.0 + d[0] * t + d[1] * t * t + d[2] * t * t * t;
    let x = t - num / den;
    if p < 0.5 {
        -x
    } else {
        x
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn solar_history() -> Vec<f64> {
        // 48 hourly values: diurnal solar pattern (2 days)
        (0..48)
            .map(|h| {
                let hr = h % 24;
                if (6..=18).contains(&hr) {
                    500.0
                        * ((hr as f64 - 12.0) / 6.0 * std::f64::consts::FRAC_PI_2)
                            .cos()
                            .powi(2)
                } else {
                    0.0
                }
            })
            .collect()
    }

    #[test]
    fn test_persistence_naive_predict() {
        let hist = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let m = PersistenceBridge::naive();
        let f = m.predict(&hist, 3);
        assert_eq!(f.len(), 3);
        assert!(
            (f[0] - 5.0).abs() < 1e-10,
            "Naive persistence should return last value"
        );
        assert!((f[1] - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_persistence_diurnal_predict() {
        let hist = solar_history();
        let m = PersistenceBridge::diurnal();
        let f = m.predict(&hist, 6);
        assert_eq!(f.len(), 6);
        // All finite
        for &v in &f {
            assert!(v.is_finite());
        }
    }

    #[test]
    fn test_arima_bridge_fit_predict() {
        let hist: Vec<f64> = (0..50)
            .map(|i| i as f64 * 0.5 + (i as f64 * 0.3).sin())
            .collect();
        let mut m = ArimaBridge::auto();
        m.fit(&hist);
        let f = m.predict(&hist, 5);
        assert_eq!(f.len(), 5);
        for &v in &f {
            assert!(v.is_finite(), "Forecast value not finite: {v}");
        }
    }

    #[test]
    fn test_ensemble_equal_weight() {
        let hist: Vec<f64> = (0..30).map(|i| i as f64).collect();
        let models: Vec<Box<dyn ForecastModel>> = vec![
            Box::new(PersistenceBridge::naive()),
            Box::new(PersistenceBridge::naive()),
        ];
        let m = EnsembleBridge::equal_weight(models);
        let f = m.predict(&hist, 3);
        assert_eq!(f.len(), 3);
        // Both models are naive persistence → result = last value
        assert!((f[0] - 29.0).abs() < 1e-9);
    }

    #[test]
    fn test_ensemble_weighted_sum() {
        let hist = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        // Model A predicts 4.0, Model B predicts 4.0 → ensemble = 4.0
        let models: Vec<Box<dyn ForecastModel>> = vec![
            Box::new(PersistenceBridge::naive()),
            Box::new(PersistenceBridge::naive()),
        ];
        let m = EnsembleBridge::new(models, vec![0.7, 0.3]);
        let f = m.predict(&hist, 1);
        assert!((f[0] - 4.0).abs() < 1e-9);
    }

    #[test]
    fn test_skill_score_persistence_vs_itself() {
        let hist: Vec<f64> = (0..20).map(|i| i as f64).collect();
        let m = PersistenceBridge::naive();
        let ss = m.skill_score(&hist, 5);
        // Skill of persistence vs persistence = 0 or near 0
        assert!(ss.is_finite());
        assert!(ss <= 1.0);
    }

    #[test]
    fn test_predict_intervals_correct_length() {
        let hist: Vec<f64> = (0..30).map(|i| (i as f64).sin() * 100.0 + 200.0).collect();
        let m = PersistenceBridge::naive();
        let (lo, pt, hi) = m.predict_intervals(&hist, 6, 0.90);
        assert_eq!(lo.len(), 6);
        assert_eq!(pt.len(), 6);
        assert_eq!(hi.len(), 6);
        for i in 0..6 {
            assert!(lo[i] <= pt[i], "lower should be <= point at step {i}");
            assert!(pt[i] <= hi[i], "point should be <= upper at step {i}");
        }
    }

    #[test]
    fn test_external_nn_bridge_falls_back() {
        let spec = NnModelSpec::new("solar-lstm", 48, 24, "LSTM-128");
        let mut bridge = ExternalNnBridge::new(spec);
        assert!(!bridge.try_load(), "No runtime available yet");
        let hist: Vec<f64> = (0..50).map(|i| i as f64).collect();
        bridge.fit(&hist);
        let f = bridge.predict(&hist, 5);
        assert_eq!(f.len(), 5);
    }

    #[test]
    fn test_nn_model_spec_builder() {
        let spec = NnModelSpec::new("wind-transformer", 72, 48, "Transformer")
            .with_path("/models/wind.onnx");
        assert_eq!(spec.input_len, 72);
        assert_eq!(spec.output_len, 48);
        assert!(spec.model_path.is_some());
    }

    #[test]
    fn test_normal_quantile_symmetry() {
        let z95 = normal_quantile(0.975);
        let z95_neg = normal_quantile(0.025);
        assert!(
            (z95 + z95_neg).abs() < 0.01,
            "Quantile should be symmetric: {z95:.4} vs {z95_neg:.4}"
        );
        assert!((z95 - 1.96).abs() < 0.05, "z0.975 ≈ 1.96, got {z95:.4}");
    }

    /// Test that a polynomial model loaded from a JSON config produces non-zero
    /// predictions consistent with the config coefficients.
    ///
    /// Config: `{"coefficients": [10.0, 1.0, 0.5, 0.1], "window": 5}`
    ///   ŷ_h = 10.0 + 1.0*mean + 0.5*slope + 0.1*h
    #[test]
    fn test_external_nn_bridge_poly_model_loaded() {
        use std::io::Write;

        // Write a temporary JSON config.
        let tmp_dir = std::env::temp_dir();
        let cfg_path = tmp_dir.join("oxigrid_test_poly_model.json");
        {
            let mut f = std::fs::File::create(&cfg_path).expect("should create temp file");
            f.write_all(b"{\"coefficients\":[10.0,1.0,0.5,0.1],\"window\":5}")
                .expect("should write config");
        }

        let spec = NnModelSpec::new("poly-test", 5, 3, "polynomial")
            .with_path(cfg_path.to_str().expect("path is valid UTF-8"));

        let mut bridge = ExternalNnBridge::new(spec);
        assert!(bridge.try_load(), "poly model should load from JSON file");
        assert!(bridge.is_loaded());

        // History: constant 100.0 → mean=100, slope=0
        let hist = vec![100.0_f64; 10];
        let f = bridge.predict(&hist, 3);
        assert_eq!(f.len(), 3);

        // Expected: 10.0 + 1.0*100.0 + 0.5*0.0 + 0.1*h = 110.0 + 0.1*h
        for (h, &val) in f.iter().enumerate() {
            let expected = 110.0 + 0.1 * h as f64;
            assert!(
                (val - expected).abs() < 1e-9,
                "step {h}: expected {expected:.3}, got {val:.3}"
            );
        }

        // Clean up.
        let _ = std::fs::remove_file(&cfg_path);
    }

    /// Test that a bridge with a nonexistent model path falls back to ARIMA.
    #[test]
    fn test_external_nn_bridge_missing_path_falls_back() {
        let spec = NnModelSpec::new("poly-missing", 5, 3, "polynomial")
            .with_path("/nonexistent/path/to/model.json");
        let mut bridge = ExternalNnBridge::new(spec);
        assert!(!bridge.try_load(), "should fail when file does not exist");
        assert!(!bridge.is_loaded());

        let hist: Vec<f64> = (0..20).map(|i| i as f64).collect();
        bridge.fit(&hist);
        let f = bridge.predict(&hist, 4);
        assert_eq!(f.len(), 4);
        for &v in &f {
            assert!(v.is_finite(), "fallback forecast should be finite");
        }
    }
}
