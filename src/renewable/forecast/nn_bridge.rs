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
///       └── ExternalNnBridge    — placeholder for torsh/trustformers
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
            let last = *history.last().unwrap();
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
                let last = *history.last().unwrap();
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

// ── External NN bridge (placeholder) ─────────────────────────────────────────

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

/// Placeholder bridge for external neural network runtimes.
///
/// When torsh/trustformers/tract-onnx are available, this struct
/// holds the loaded model and dispatches inference. Until then,
/// it falls back to a statistical model.
pub struct ExternalNnBridge {
    pub spec: NnModelSpec,
    fallback: Box<dyn ForecastModel>,
    is_loaded: bool,
}

impl ExternalNnBridge {
    pub fn new(spec: NnModelSpec) -> Self {
        let fallback: Box<dyn ForecastModel> = Box::new(ArimaBridge::auto());
        Self {
            spec,
            fallback,
            is_loaded: false,
        }
    }

    /// Attempt to load the model from disk (no-op until runtime crate available).
    pub fn try_load(&mut self) -> bool {
        // TODO: when tract/ort/torsh available, load model here
        // For now always returns false (uses fallback)
        self.is_loaded = false;
        false
    }

    pub fn is_loaded(&self) -> bool {
        self.is_loaded
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

    fn predict(&self, history: &[f64], horizon: usize) -> Vec<f64> {
        if self.is_loaded {
            // TODO: call runtime inference
            vec![0.0; horizon]
        } else {
            self.fallback.predict(history, horizon)
        }
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
}
