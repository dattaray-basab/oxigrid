/// Persistence forecast model.
///
/// The simplest forecast: the prediction for time t+k is the observation
/// at time t (or the same time the previous day for diurnal patterns).
///
/// Used as a benchmark — any useful model should beat persistence.
use serde::{Deserialize, Serialize};

/// Naive persistence model: forecast[t+1] = observation[t].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PersistenceForecast {
    /// Last observed value
    pub last_value: f64,
    /// Sliding window of recent observations (for RMSE tracking)
    history: Vec<f64>,
    max_history: usize,
}

impl PersistenceForecast {
    pub fn new(initial_value: f64) -> Self {
        Self {
            last_value: initial_value,
            history: vec![initial_value],
            max_history: 48,
        }
    }

    /// Update with a new observation and return the forecast for next step.
    pub fn update(&mut self, observation: f64) -> f64 {
        self.history.push(observation);
        if self.history.len() > self.max_history {
            self.history.remove(0);
        }
        self.last_value = observation;
        observation // next-step persistence forecast
    }

    /// k-step ahead persistence forecast (all equal to last observation).
    pub fn forecast_k_steps(&self, k: usize) -> Vec<f64> {
        vec![self.last_value; k]
    }

    /// Root mean square error over available history (leave-one-out on sliding window).
    pub fn rmse(&self) -> Option<f64> {
        let n = self.history.len();
        if n < 2 {
            return None;
        }
        let mse: f64 = self
            .history
            .windows(2)
            .map(|w| (w[1] - w[0]).powi(2))
            .sum::<f64>()
            / (n - 1) as f64;
        Some(mse.sqrt())
    }
}

/// Diurnal persistence: forecast for hour h tomorrow = observation at hour h today.
///
/// Better for solar irradiance or load which has a strong 24-h cycle.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DiurnalPersistence {
    pub period: usize,
    buffer: Vec<f64>,
    cursor: usize,
}

impl DiurnalPersistence {
    /// `period` is the cycle length in samples (e.g., 24 for hourly solar).
    pub fn new(period: usize) -> Self {
        Self {
            period,
            buffer: vec![0.0; period],
            cursor: 0,
        }
    }

    /// Update with new observation; returns the diurnal-persistence forecast
    /// (the value from `period` steps ago).
    pub fn update(&mut self, observation: f64) -> f64 {
        let forecast = self.buffer[self.cursor];
        self.buffer[self.cursor] = observation;
        self.cursor = (self.cursor + 1) % self.period;
        forecast
    }

    /// Forecast for the next `period` steps using today's profile.
    pub fn forecast_next_period(&self) -> Vec<f64> {
        let mut out = Vec::with_capacity(self.period);
        for i in 0..self.period {
            let idx = (self.cursor + i) % self.period;
            out.push(self.buffer[idx]);
        }
        out
    }
}

/// Compute forecast skill score relative to persistence baseline.
///
/// Skill = 1 − RMSE_model / RMSE_persistence   (higher is better; 1.0 = perfect).
pub fn skill_score(model_rmse: f64, persistence_rmse: f64) -> f64 {
    if persistence_rmse < 1e-12 {
        return 0.0;
    }
    1.0 - model_rmse / persistence_rmse
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_persistence_forecast_equals_last_obs() {
        let mut fc = PersistenceForecast::new(100.0);
        let next = fc.update(120.0);
        assert_eq!(next, 120.0);
        let fcs = fc.forecast_k_steps(5);
        assert!(fcs.iter().all(|&v| v == 120.0));
    }

    #[test]
    fn test_persistence_rmse_constant_series() {
        let mut fc = PersistenceForecast::new(50.0);
        for _ in 0..10 {
            fc.update(50.0); // constant → RMSE = 0
        }
        assert_eq!(fc.rmse().unwrap(), 0.0);
    }

    #[test]
    fn test_persistence_rmse_step_change() {
        let mut fc = PersistenceForecast::new(0.0);
        fc.update(10.0); // error = 10
        fc.update(10.0); // error = 0
        let rmse = fc.rmse().unwrap();
        assert!(rmse > 0.0 && rmse <= 10.0, "rmse={}", rmse);
    }

    #[test]
    fn test_diurnal_returns_previous_period() {
        let mut dp = DiurnalPersistence::new(3);
        dp.update(10.0);
        dp.update(20.0);
        dp.update(30.0);
        // Now cursor wraps around: next update returns the values from one period ago
        assert_eq!(dp.update(11.0), 10.0);
        assert_eq!(dp.update(21.0), 20.0);
    }

    #[test]
    fn test_skill_score_perfect_model() {
        assert_eq!(skill_score(0.0, 10.0), 1.0);
    }

    #[test]
    fn test_skill_score_worse_than_persistence() {
        assert!(skill_score(15.0, 10.0) < 0.0);
    }
}
