//! Statistical anomaly detection for grid measurement time-series.
//!
//! Provides:
//! - [`GridAnomalyDetector`] — z-score, EWMA, and CUSUM-based point/batch detection
//! - [`MeasurementCorrelationAnalyzer`] — Pearson correlation matrix and change detection

use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Compute mean of a slice.  Returns `0.0` on empty input.
fn mean(v: &[f64]) -> f64 {
    if v.is_empty() {
        return 0.0;
    }
    v.iter().sum::<f64>() / v.len() as f64
}

/// Compute population standard deviation.  Returns `0.0` on fewer than 2 elements.
fn std_dev(v: &[f64]) -> f64 {
    if v.len() < 2 {
        return 0.0;
    }
    let m = mean(v);
    let variance = v.iter().map(|x| (x - m).powi(2)).sum::<f64>() / v.len() as f64;
    variance.sqrt()
}

// ---------------------------------------------------------------------------
// GridAnomalyDetector
// ---------------------------------------------------------------------------

/// Detects statistical anomalies in a scalar grid measurement stream.
///
/// Three complementary methods are used:
/// - **Z-score** against a rolling window of historical values
/// - **EWMA** (Exponentially Weighted Moving Average) tracking
/// - **CUSUM** change-point test
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GridAnomalyDetector {
    /// Number of most-recent history samples used for rolling statistics.
    pub window_size: usize,
    /// Z-score alarm threshold (default `3.0` ≈ 0.3 % false-alarm rate).
    pub z_score_threshold: f64,
    /// EWMA smoothing factor α ∈ (0, 1].  Smaller = more smoothing.
    pub ewma_alpha: f64,
}

/// Result of a single anomaly-detection call.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnomalyResult {
    /// `true` if the value is classified as anomalous.
    pub is_anomaly: bool,
    /// Overall suspicion score (= `|z_score|`).
    pub score: f64,
    /// Standard deviations from the rolling mean.
    pub z_score: f64,
    /// Current EWMA estimate.
    pub ewma_value: f64,
    /// Upper control limit (mean + threshold × std).
    pub upper_bound: f64,
    /// Lower control limit (mean − threshold × std).
    pub lower_bound: f64,
    /// `true` if a CUSUM change point was detected in the history.
    pub change_point: bool,
}

impl GridAnomalyDetector {
    /// Construct a detector.
    ///
    /// # Arguments
    /// * `window_size` — rolling window length
    /// * `threshold`   — z-score alarm threshold (e.g. `3.0`)
    pub fn new(window_size: usize, threshold: f64) -> Self {
        Self {
            window_size,
            z_score_threshold: threshold,
            ewma_alpha: 0.1,
        }
    }

    /// Detect whether `value` is anomalous relative to `history`.
    ///
    /// The last `window_size` points of `history` are used for rolling statistics.
    pub fn detect_point(&self, value: f64, history: &[f64]) -> AnomalyResult {
        // Take the most recent window.
        let window: &[f64] = if history.len() > self.window_size {
            &history[history.len() - self.window_size..]
        } else {
            history
        };

        let m = mean(window);
        let raw_std = std_dev(window);

        // When the history is perfectly uniform (std ≈ 0), a measurement that is very
        // close to the mean should not be flagged.  Use an absolute-deviation check in
        // that case: flag only if the deviation exceeds 1e-6 (effectively zero).
        let (z_score, is_anomaly) = if raw_std < 1e-10 {
            let abs_dev = (value - m).abs();
            // Treat any value within 1e-4 of the constant history as normal.
            let anomaly = abs_dev > 1e-4;
            // Report a z_score proportional to deviation scaled by threshold so that
            // callers see a finite number.
            let z = if anomaly {
                self.z_score_threshold + abs_dev
            } else {
                abs_dev * self.z_score_threshold
            };
            (z, anomaly)
        } else {
            let z = (value - m) / raw_std;
            (z, z.abs() > self.z_score_threshold)
        };

        let s = if raw_std < 1e-10 { 1e-10 } else { raw_std };

        // EWMA over history.
        let ewma_value = compute_ewma(history, self.ewma_alpha);

        // CUSUM change-point over history.
        let k = 0.5 * s;
        let h = 5.0 * s;
        let change_point = self.cusum_test(history, k, h).is_some();

        AnomalyResult {
            is_anomaly,
            score: z_score.abs(),
            z_score,
            ewma_value,
            upper_bound: m + self.z_score_threshold * s,
            lower_bound: m - self.z_score_threshold * s,
            change_point,
        }
    }

    /// Detect anomalies across all measurements simultaneously.
    ///
    /// `history[i]` is the time-series for measurement channel `i`.
    /// Returns one [`AnomalyResult`] per channel.
    pub fn detect_batch(&self, values: &[f64], history: &[Vec<f64>]) -> Vec<AnomalyResult> {
        values
            .iter()
            .enumerate()
            .map(|(i, &v)| {
                let h = history.get(i).map(|x| x.as_slice()).unwrap_or(&[]);
                self.detect_point(v, h)
            })
            .collect()
    }

    /// CUSUM change-point detection on a scalar series.
    ///
    /// Uses the two-sided CUSUM algorithm:
    /// ```text
    /// S⁺`t` = max(0, S⁺[t-1] + x[t] - μ - k)
    /// S⁻`t` = max(0, S⁻[t-1] - x[t] + μ - k)
    /// alarm when S⁺`t` > h  or  S⁻`t` > h
    /// ```
    ///
    /// # Returns
    /// The first sample index at which an alarm fires, or `None`.
    pub fn cusum_test(&self, series: &[f64], threshold_k: f64, threshold_h: f64) -> Option<usize> {
        if series.is_empty() {
            return None;
        }
        let mu = mean(series);
        let mut s_pos = 0.0_f64;
        let mut s_neg = 0.0_f64;
        for (t, &x) in series.iter().enumerate() {
            s_pos = (s_pos + x - mu - threshold_k).max(0.0);
            s_neg = (s_neg - x + mu - threshold_k).max(0.0);
            if s_pos > threshold_h || s_neg > threshold_h {
                return Some(t);
            }
        }
        None
    }
}

/// Compute EWMA over a slice.  `S_0 = values[0]`, `S_t = α x_t + (1−α) S_{t-1}`.
/// Returns `0.0` on empty input.
fn compute_ewma(values: &[f64], alpha: f64) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    let mut s = values[0];
    for &x in &values[1..] {
        s = alpha * x + (1.0 - alpha) * s;
    }
    s
}

// ---------------------------------------------------------------------------
// MeasurementCorrelationAnalyzer
// ---------------------------------------------------------------------------

/// Analyses pairwise correlations among measurement channels to detect coordinated attacks.
///
/// A coordinated FDI attack on multiple sensors often breaks the expected correlation
/// structure; this analyzer computes the Frobenius distance between a current correlation
/// matrix and a baseline.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MeasurementCorrelationAnalyzer {
    /// Number of measurement channels.
    pub n_measurements: usize,
    /// Number of most-recent time steps used for rolling correlation.
    pub correlation_window: usize,
}

impl MeasurementCorrelationAnalyzer {
    /// Create an analyzer for `n_measurements` channels with a rolling `window`.
    pub fn new(n_measurements: usize, window: usize) -> Self {
        Self {
            n_measurements,
            correlation_window: window,
        }
    }

    /// Compute the `n × n` Pearson correlation matrix from measurement history.
    ///
    /// `history[i][t]` is the value of measurement channel `i` at time `t`.
    /// The last `correlation_window` time steps are used.
    pub fn compute_correlation(&self, history: &[Vec<f64>]) -> Vec<Vec<f64>> {
        let n = self.n_measurements.min(history.len());
        let mut corr = vec![vec![0.0_f64; n]; n];
        for i in 0..n {
            for j in 0..n {
                corr[i][j] = if i == j {
                    1.0
                } else {
                    pearson_correlation_windowed(&history[i], &history[j], self.correlation_window)
                };
            }
        }
        corr
    }

    /// Detect a structural change in measurement correlations.
    ///
    /// Computes the current correlation matrix from `current_window` and returns
    /// the Frobenius norm of the difference against `baseline_correlation`.
    pub fn detect_correlation_change(
        &self,
        current_window: &[Vec<f64>],
        baseline_correlation: &[Vec<f64>],
    ) -> f64 {
        let current = self.compute_correlation(current_window);
        let n = baseline_correlation.len().min(current.len());
        let mut sq_sum = 0.0_f64;
        for i in 0..n {
            let brow = &baseline_correlation[i];
            let crow = &current[i];
            for j in 0..brow.len().min(crow.len()) {
                let diff = crow[j] - brow[j];
                sq_sum += diff * diff;
            }
        }
        sq_sum.sqrt()
    }
}

/// Pearson correlation using the last `window` time steps.
fn pearson_correlation_windowed(a: &[f64], b: &[f64], window: usize) -> f64 {
    let len = a.len().min(b.len());
    if len == 0 {
        return 0.0;
    }
    let start = len.saturating_sub(window);
    let a_w = &a[start..];
    let b_w = &b[start..];
    let ma = mean(a_w);
    let mb = mean(b_w);
    let num: f64 = a_w
        .iter()
        .zip(b_w.iter())
        .map(|(x, y)| (x - ma) * (y - mb))
        .sum();
    let da: f64 = a_w.iter().map(|x| (x - ma).powi(2)).sum::<f64>().sqrt();
    let db: f64 = b_w.iter().map(|y| (y - mb).powi(2)).sum::<f64>().sqrt();
    let denom = da * db;
    if denom < 1e-15 {
        0.0
    } else {
        (num / denom).clamp(-1.0, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn z_score_detects_outlier() {
        let detector = GridAnomalyDetector::new(20, 3.0);
        let history: Vec<f64> = (0..20).map(|i| i as f64).collect();
        let result = detector.detect_point(200.0, &history);
        assert!(result.is_anomaly);
        assert!(result.z_score > 3.0);
    }

    #[test]
    fn z_score_passes_normal() {
        let detector = GridAnomalyDetector::new(20, 3.0);
        // Use varying history so std is non-zero, and pick a value well within 3σ.
        let history: Vec<f64> = (0..20).map(|i| 9.0 + (i as f64) * 0.1).collect(); // 9.0..10.9, mean≈9.95, std≈0.59
        let value = 10.0_f64; // within 1σ of mean
        let result = detector.detect_point(value, &history);
        assert!(
            !result.is_anomaly,
            "Value {} should be normal; z={}",
            value, result.z_score
        );
    }

    #[test]
    fn cusum_detects_step_change() {
        let detector = GridAnomalyDetector::new(20, 3.0);
        let mut series: Vec<f64> = vec![0.0; 10];
        series.extend(vec![5.0; 10]);
        let s = std_dev(&series[..10]).max(0.01);
        let result = detector.cusum_test(&series, 0.5 * s, 5.0 * s);
        assert!(result.is_some());
    }

    #[test]
    fn correlation_matrix_diagonal_is_one() {
        let analyzer = MeasurementCorrelationAnalyzer::new(3, 10);
        let history = vec![
            vec![1.0, 2.0, 3.0, 4.0],
            vec![2.0, 3.0, 4.0, 5.0],
            vec![0.5, 1.5, 2.5, 3.5],
        ];
        let corr = analyzer.compute_correlation(&history);
        for (i, row) in corr.iter().enumerate().take(3) {
            assert!((row[i] - 1.0).abs() < 1e-10);
        }
    }

    #[test]
    fn detect_point_empty_history() {
        let detector = GridAnomalyDetector::new(10, 3.0);
        // With empty history mean=0, std=0; value=0 is within 1e-4 of mean so not anomalous.
        let result = detector.detect_point(0.0, &[]);
        assert!(
            !result.is_anomaly,
            "value equal to mean of empty history (0.0) should not flag an anomaly"
        );
        // Sanity: a value far from zero should be flagged.
        let result_far = detector.detect_point(100.0, &[]);
        assert!(
            result_far.is_anomaly,
            "value far from mean of empty history should be flagged as anomaly"
        );
    }

    #[test]
    fn detect_batch_returns_correct_count() {
        let detector = GridAnomalyDetector::new(10, 3.0);
        let values = vec![1.0, 2.0, 3.0];
        let history = vec![
            vec![0.0, 1.0, 2.0, 3.0],
            vec![0.5, 1.5, 2.5, 3.5],
            vec![1.0, 2.0, 3.0, 4.0],
        ];
        let results = detector.detect_batch(&values, &history);
        assert_eq!(
            results.len(),
            3,
            "detect_batch should return one result per value"
        );
    }

    #[test]
    fn ewma_tracks_trend() {
        let rising: Vec<f64> = (0..10).map(|i| i as f64).collect();
        let ewma_val = compute_ewma(&rising, 0.3);
        assert!(
            ewma_val > rising[0],
            "EWMA on rising series ({ewma_val}) should exceed first element ({})",
            rising[0]
        );
    }

    #[test]
    fn mean_empty_returns_zero() {
        let result = mean(&[]);
        assert_eq!(result, 0.0, "mean of empty slice should be 0.0");
    }

    #[test]
    fn std_dev_single_returns_zero() {
        let result = std_dev(&[5.0]);
        assert_eq!(
            result, 0.0,
            "std_dev of a single-element slice should be 0.0"
        );
    }

    #[test]
    fn cusum_no_change_returns_none() {
        let detector = GridAnomalyDetector::new(10, 3.0);
        // Perfectly flat series — CUSUM should never accumulate enough to trigger.
        let series = vec![1.0_f64; 30];
        let result = detector.cusum_test(&series, 1e6, 1e9);
        assert!(
            result.is_none(),
            "CUSUM on a flat series with very large thresholds should return None"
        );
    }

    #[test]
    fn correlation_perfectly_correlated_channels() {
        let analyzer = MeasurementCorrelationAnalyzer::new(2, 5);
        let series: Vec<f64> = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let history = vec![series.clone(), series.clone()];
        let corr = analyzer.compute_correlation(&history);
        assert!(
            (corr[0][1] - 1.0).abs() < 1e-10,
            "two identical series should have Pearson correlation 1.0, got {}",
            corr[0][1]
        );
    }

    #[test]
    fn detect_correlation_change_identical_returns_near_zero() {
        let analyzer = MeasurementCorrelationAnalyzer::new(3, 4);
        let baseline_window = vec![
            vec![1.0, 2.0, 3.0, 4.0],
            vec![4.0, 3.0, 2.0, 1.0],
            vec![1.0, 1.0, 2.0, 2.0],
        ];
        let baseline_corr = analyzer.compute_correlation(&baseline_window);
        let frobenius = analyzer.detect_correlation_change(&baseline_window, &baseline_corr);
        assert!(
            frobenius < 1e-9,
            "Frobenius norm of (baseline - baseline) should be ~0, got {frobenius}"
        );
    }

    #[test]
    fn detect_point_empty_history_zero_value() {
        let detector = GridAnomalyDetector::new(20, 3.0);
        let result = detector.detect_point(0.0, &[]);
        // With empty history mean=0, value=0 → deviation=0 → not anomalous.
        assert!(!result.is_anomaly);
    }

    #[test]
    fn ewma_value_approaches_latest() {
        let mut detector = GridAnomalyDetector::new(20, 3.0);
        detector.ewma_alpha = 0.9;
        // History ends at 100.0; with alpha=0.9 EWMA should be very close to 100.
        let history: Vec<f64> = (0..20).map(|i| if i < 19 { 0.0 } else { 100.0 }).collect();
        let result = detector.detect_point(100.0, &history);
        assert!(
            result.ewma_value > 80.0,
            "EWMA should be close to 100; got {}",
            result.ewma_value
        );
    }

    #[test]
    fn detect_batch_returns_per_channel() {
        let detector = GridAnomalyDetector::new(10, 3.0);
        let values = vec![1.0, 2.0, 3.0];
        let history = vec![
            vec![1.0, 1.0, 1.0, 1.0, 1.0],
            vec![2.0, 2.0, 2.0, 2.0, 2.0],
            vec![3.0, 3.0, 3.0, 3.0, 3.0],
        ];
        let results = detector.detect_batch(&values, &history);
        assert_eq!(results.len(), 3);
    }

    #[test]
    fn cusum_no_alarm_constant_series() {
        let detector = GridAnomalyDetector::new(20, 3.0);
        let series = vec![1.0_f64; 20];
        // With a perfectly constant series the CUSUM accumulator stays at 0.
        let result = detector.cusum_test(&series, 0.5, 1.0);
        assert!(
            result.is_none(),
            "Constant series should not trigger CUSUM alarm"
        );
    }

    #[test]
    fn detect_point_bounds_contain_normal() {
        let detector = GridAnomalyDetector::new(20, 3.0);
        let history: Vec<f64> = (0..20).map(|i| i as f64).collect(); // mean=9.5, std≈5.77
        let value = 9.5_f64; // right at the mean
        let result = detector.detect_point(value, &history);
        assert!(
            result.lower_bound <= value && value <= result.upper_bound,
            "Normal value {} should be inside [{}, {}]",
            value,
            result.lower_bound,
            result.upper_bound
        );
    }

    #[test]
    fn score_is_abs_z_score() {
        let detector = GridAnomalyDetector::new(20, 3.0);
        let history: Vec<f64> = (0..20).map(|i| i as f64).collect();
        // Use a value that is below the mean to get a negative z_score.
        let result = detector.detect_point(-5.0, &history);
        assert!(
            (result.score - result.z_score.abs()).abs() < 1e-10,
            "score {} should equal |z_score| {}",
            result.score,
            result.z_score.abs()
        );
    }

    #[test]
    fn correlation_change_zero_for_identical_window() {
        let analyzer = MeasurementCorrelationAnalyzer::new(3, 10);
        let data = vec![
            vec![1.0, 2.0, 3.0, 4.0, 5.0],
            vec![5.0, 4.0, 3.0, 2.0, 1.0],
            vec![2.0, 2.0, 3.0, 3.0, 4.0],
        ];
        let baseline = analyzer.compute_correlation(&data);
        // Comparing the same window against its own baseline should yield ~0.
        let change = analyzer.detect_correlation_change(&data, &baseline);
        assert!(
            change < 1e-10,
            "Correlation change for identical windows should be ~0, got {}",
            change
        );
    }
}
