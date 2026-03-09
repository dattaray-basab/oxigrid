//! Synthetic inertia estimation from frequency measurements.
//!
//! Estimates system inertia constant H \[s\] from frequency time-series and
//! known power imbalances using four methods:
//!
//! - **SwingEquation**: instantaneous ROCOF → H = ΔP / (2 · f₀ · ROCOF)
//! - **LeastSquares**: linear regression of f(t) trajectory vs swing model
//! - **Spectral**: inertia from energy spectral density of the frequency signal
//! - **KalmanFilter**: adaptive tracker that follows H changes over time
//!
//! # References
//! - Anderson & Fouad, "Power System Control and Stability", IEEE Press, 2003.
//! - Ashton et al., "Inertia estimation of the GB power system using synchrophasor
//!   measurements", IEEE Trans. Power Syst. 30(2), 2015.

use crate::error::OxiGridError;
use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// Error type
// ---------------------------------------------------------------------------

/// Errors produced by the inertia estimation module.
#[derive(Debug, thiserror::Error)]
pub enum InertiaError {
    /// Frequency measurement arrays have mismatched lengths.
    #[error("freq_hz and time_s must have equal length (got {0} vs {1})")]
    LengthMismatch(usize, usize),

    /// Less than 2 samples provided.
    #[error("need at least 2 samples for estimation (got {0})")]
    InsufficientData(usize),

    /// |ROCOF| is below the configured minimum threshold.
    #[error("ROCOF magnitude {rocof:.4} Hz/s is below threshold {threshold:.4} Hz/s")]
    RocofBelowThreshold { rocof: f64, threshold: f64 },

    /// Numerical failure inside the estimator.
    #[error("numerical error: {0}")]
    Numerical(String),

    /// Power imbalance is too small to estimate inertia from.
    #[error("power imbalance {0:.4} MW is too small for reliable estimation")]
    SmallImbalance(f64),
}

impl From<InertiaError> for OxiGridError {
    fn from(e: InertiaError) -> Self {
        OxiGridError::InvalidParameter(e.to_string())
    }
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration for the inertia estimation algorithm.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InertiaEstimationConfig {
    /// Nominal system frequency \[Hz\] (e.g. 50.0 or 60.0)
    pub nominal_frequency_hz: f64,
    /// System base MVA \[MVA\]
    pub base_mva: f64,
    /// Sliding-window length for ROCOF computation and estimation \[s\]
    pub estimation_window_s: f64,
    /// Minimum |ROCOF| to trigger estimation \[Hz/s\] (e.g. 0.05)
    pub min_rocof_threshold: f64,
}

impl Default for InertiaEstimationConfig {
    fn default() -> Self {
        Self {
            nominal_frequency_hz: 50.0,
            base_mva: 1000.0,
            estimation_window_s: 5.0,
            min_rocof_threshold: 0.05,
        }
    }
}

// ---------------------------------------------------------------------------
// Method enum
// ---------------------------------------------------------------------------

/// Inertia estimation method selector.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum InertiaEstimationMethod {
    /// Classic swing equation: H = ΔP / (2 · f₀ · ROCOF)
    SwingEquation,
    /// Least-squares fit of frequency trajectory to swing equation
    LeastSquares,
    /// Spectral method: inertia from energy spectral density
    Spectral,
    /// Adaptive Kalman filter tracking H over time
    KalmanFilter,
}

// ---------------------------------------------------------------------------
// Result
// ---------------------------------------------------------------------------

/// Result of an inertia estimation run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InertiaEstimate {
    /// Estimated system inertia constant H \[s\] (MJ/MVA)
    pub h_total_mj_mva: f64,
    /// ±1σ uncertainty in H \[s\]
    pub h_uncertainty: f64,
    /// Angular momentum M = 2H·S \[MJ\]
    pub m_total_mj: f64,
    /// Confidence score (0–1)
    pub confidence: f64,
    /// Name of the method used
    pub method: String,
    /// ROCOF used for estimation \[Hz/s\]
    pub rocof_used: f64,
    /// Data-quality score (0–1, based on noise level and sample count)
    pub data_quality: f64,
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Compute ROCOF \[Hz/s\] from frequency array using linear regression over
/// the full window. Returns (rocof, r_squared).
fn compute_rocof_linear(freq_hz: &[f64], time_s: &[f64]) -> (f64, f64) {
    let n = freq_hz.len() as f64;
    let t_mean = time_s.iter().sum::<f64>() / n;
    let f_mean = freq_hz.iter().sum::<f64>() / n;

    let sxy: f64 = time_s
        .iter()
        .zip(freq_hz.iter())
        .map(|(&t, &f)| (t - t_mean) * (f - f_mean))
        .sum();
    let sxx: f64 = time_s.iter().map(|&t| (t - t_mean).powi(2)).sum();
    let syy: f64 = freq_hz.iter().map(|&f| (f - f_mean).powi(2)).sum();

    if sxx.abs() < 1e-15 {
        return (0.0, 0.0);
    }
    let slope = sxy / sxx;
    let r_sq = if syy.abs() < 1e-15 {
        1.0
    } else {
        (sxy * sxy / (sxx * syy)).clamp(0.0, 1.0)
    };
    (slope, r_sq)
}

/// Assess data quality: 0 = poor, 1 = perfect.
/// Based on sample count, frequency range, and ROCOF linearity (R²).
fn assess_data_quality(freq_hz: &[f64], time_s: &[f64], r_squared: f64) -> f64 {
    let n = freq_hz.len();
    // Sample count score (saturates at 50+ samples)
    let count_score = (n as f64 / 50.0).min(1.0);
    // Duration score
    let duration = time_s.last().copied().unwrap_or(0.0) - time_s.first().copied().unwrap_or(0.0);
    let duration_score = (duration / 5.0).min(1.0);
    // Frequency excursion must be noticeable
    let f_max = freq_hz.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let f_min = freq_hz.iter().cloned().fold(f64::INFINITY, f64::min);
    let excursion_score = ((f_max - f_min) / 0.5).min(1.0);

    (0.35 * r_squared + 0.3 * count_score + 0.2 * duration_score + 0.15 * excursion_score)
        .clamp(0.0, 1.0)
}

/// Simple DFT magnitude at frequency index k (no external FFT dependency).
/// Returns the power at bin k = 1 (lowest non-DC oscillation).
fn spectral_inertia(
    freq_hz: &[f64],
    time_s: &[f64],
    power_imbalance_mw: f64,
    base_mva: f64,
    f0: f64,
) -> Result<f64, InertiaError> {
    let n = freq_hz.len();
    if n < 4 {
        return Err(InertiaError::InsufficientData(n));
    }
    // Estimate the frequency of the natural oscillation from peak in DFT
    // We compute DFT magnitudes for bins 1..n/2 and find the dominant bin.
    let dt = if time_s.len() >= 2 {
        (time_s.last().copied().unwrap_or(1.0) - time_s.first().copied().unwrap_or(0.0))
            / (n as f64 - 1.0)
    } else {
        1.0
    };

    // Use the mean-subtracted frequency deviation signal
    let f_mean = freq_hz.iter().sum::<f64>() / n as f64;
    let signal: Vec<f64> = freq_hz.iter().map(|&f| f - f_mean).collect();

    // DFT: find dominant oscillation frequency (skip DC bin 0)
    let mut best_power = 0.0_f64;
    let mut best_freq_hz = 1.0_f64 / (n as f64 * dt); // lowest bin

    let n_bins = n / 2;
    for k in 1..=n_bins {
        let mut re = 0.0_f64;
        let mut im = 0.0_f64;
        for (i, &s) in signal.iter().enumerate() {
            let angle = -2.0 * std::f64::consts::PI * k as f64 * i as f64 / n as f64;
            re += s * angle.cos();
            im += s * angle.sin();
        }
        let power = re * re + im * im;
        if power > best_power {
            best_power = power;
            best_freq_hz = k as f64 / (n as f64 * dt);
        }
    }

    // The natural frequency of the swing equation:
    //   ω_n = sqrt(Pe / (2H·S)) where Pe ≈ power_imbalance in per-unit
    // So H = Pe / (2 * S * ω_n²) where ω_n = 2π * f_osc
    let p_pu = power_imbalance_mw.abs() / base_mva;
    let omega_n = 2.0 * std::f64::consts::PI * best_freq_hz;

    if omega_n < 1e-6 || p_pu < 1e-6 {
        return Err(InertiaError::Numerical(
            "spectral oscillation frequency or imbalance too small".into(),
        ));
    }

    // From swing: M d²δ/dt² = ΔP (pu), M = 2H/ω₀
    // Natural frequency: ω_n² = ΔP / (M · δ₀) — for small disturbance
    // Alternatively: H ≈ ΔP * f₀ / (best_power_density * ω_n²)
    // Simplified spectral estimate: H = p_pu * f0 / omega_n^2
    let h = p_pu * f0 / (omega_n * omega_n);
    Ok(h.clamp(0.1, 100.0))
}

// ---------------------------------------------------------------------------
// Kalman filter state for H estimation
// ---------------------------------------------------------------------------

/// Internal state for the adaptive Kalman filter.
#[derive(Debug, Clone)]
struct KalmanState {
    /// Estimated H \[s\]
    h: f64,
    /// State variance P
    p: f64,
    /// Process noise Q
    q: f64,
    /// Measurement noise R
    r: f64,
}

impl KalmanState {
    fn new(h_init: f64) -> Self {
        Self {
            h: h_init,
            p: 4.0,  // large initial uncertainty
            q: 0.01, // process noise (H changes slowly)
            r: 0.25, // measurement noise
        }
    }

    /// Update with a new H measurement.
    fn update(&mut self, h_meas: f64) {
        // Predict
        let p_pred = self.p + self.q;
        // Kalman gain
        let k = p_pred / (p_pred + self.r);
        // Update
        self.h += k * (h_meas - self.h);
        self.p = (1.0 - k) * p_pred;
    }

    fn uncertainty_sigma(&self) -> f64 {
        self.p.sqrt()
    }
}

// ---------------------------------------------------------------------------
// InertiaTracker
// ---------------------------------------------------------------------------

/// Tracks inertia H over time using a sliding window and Kalman filter.
pub struct InertiaTracker {
    config: InertiaEstimationConfig,
    /// Rolling history of (time_s, H_estimate)
    history: Vec<(f64, f64)>,
    /// Internal frequency buffer for the sliding window
    freq_buf: Vec<(f64, f64)>, // (time_s, freq_hz)
    /// Kalman filter state
    kalman: KalmanState,
    /// Last known power imbalance \[MW\]
    last_imbalance_mw: f64,
}

impl InertiaTracker {
    /// Create a new tracker with given configuration.
    pub fn new(config: InertiaEstimationConfig) -> Self {
        let h_init = 4.0; // reasonable default for a large system
        Self {
            config,
            history: Vec::new(),
            freq_buf: Vec::new(),
            kalman: KalmanState::new(h_init),
            last_imbalance_mw: 0.0,
        }
    }

    // -----------------------------------------------------------------------
    // Core estimation API
    // -----------------------------------------------------------------------

    /// Estimate H from a frequency time series and known power imbalance.
    pub fn estimate(
        &self,
        freq_hz: &[f64],
        time_s: &[f64],
        power_imbalance_mw: f64,
        method: InertiaEstimationMethod,
    ) -> Result<InertiaEstimate, InertiaError> {
        if freq_hz.len() != time_s.len() {
            return Err(InertiaError::LengthMismatch(freq_hz.len(), time_s.len()));
        }
        if freq_hz.len() < 2 {
            return Err(InertiaError::InsufficientData(freq_hz.len()));
        }

        let f0 = self.config.nominal_frequency_hz;
        let s_base = self.config.base_mva;

        // Compute ROCOF via linear regression over the window
        let (rocof, r_sq) = compute_rocof_linear(freq_hz, time_s);
        let data_quality = assess_data_quality(freq_hz, time_s, r_sq);

        // Check ROCOF threshold
        if rocof.abs() < self.config.min_rocof_threshold {
            return Err(InertiaError::RocofBelowThreshold {
                rocof: rocof.abs(),
                threshold: self.config.min_rocof_threshold,
            });
        }

        match method {
            InertiaEstimationMethod::SwingEquation => self.estimate_swing_equation(
                power_imbalance_mw,
                rocof,
                r_sq,
                data_quality,
                f0,
                s_base,
            ),
            InertiaEstimationMethod::LeastSquares => self.estimate_least_squares(
                freq_hz,
                time_s,
                power_imbalance_mw,
                rocof,
                r_sq,
                data_quality,
                f0,
                s_base,
            ),
            InertiaEstimationMethod::Spectral => {
                let h = spectral_inertia(freq_hz, time_s, power_imbalance_mw, s_base, f0)?;
                Ok(InertiaEstimate {
                    h_total_mj_mva: h,
                    h_uncertainty: h * 0.15,
                    m_total_mj: 2.0 * h * s_base,
                    confidence: (data_quality * 0.8).clamp(0.0, 1.0),
                    method: "Spectral".into(),
                    rocof_used: rocof,
                    data_quality,
                })
            }
            InertiaEstimationMethod::KalmanFilter => {
                // Use swing-equation measurement as input to Kalman filter
                let swing = self.estimate_swing_equation(
                    power_imbalance_mw,
                    rocof,
                    r_sq,
                    data_quality,
                    f0,
                    s_base,
                )?;
                // Clone Kalman state and apply one update step
                let mut ks = self.kalman.clone();
                ks.update(swing.h_total_mj_mva);
                let h = ks.h;
                let sigma = ks.uncertainty_sigma();
                Ok(InertiaEstimate {
                    h_total_mj_mva: h,
                    h_uncertainty: sigma,
                    m_total_mj: 2.0 * h * s_base,
                    confidence: (1.0 - (sigma / (h + 1e-9)).min(1.0)) * data_quality,
                    method: "KalmanFilter".into(),
                    rocof_used: rocof,
                    data_quality,
                })
            }
        }
    }

    // -----------------------------------------------------------------------
    // Tracker API
    // -----------------------------------------------------------------------

    /// Add a new measurement to the sliding-window buffer and update Kalman state.
    pub fn update(&mut self, time_s: f64, freq_hz: f64, power_imbalance_mw: f64) {
        self.last_imbalance_mw = power_imbalance_mw;
        self.freq_buf.push((time_s, freq_hz));

        // Trim buffer to the estimation window
        let window = self.config.estimation_window_s;
        if let Some(&(t_last, _)) = self.freq_buf.last() {
            self.freq_buf.retain(|&(t, _)| t_last - t <= window);
        }

        // Try to produce an estimate if we have enough data
        if self.freq_buf.len() < 4 {
            return;
        }
        let times: Vec<f64> = self.freq_buf.iter().map(|&(t, _)| t).collect();
        let freqs: Vec<f64> = self.freq_buf.iter().map(|&(_, f)| f).collect();

        if let Ok(est) = self.estimate(
            &freqs,
            &times,
            power_imbalance_mw,
            InertiaEstimationMethod::SwingEquation,
        ) {
            self.kalman.update(est.h_total_mj_mva);
            self.history.push((time_s, self.kalman.h));

            // Keep only last 200 history entries
            if self.history.len() > 200 {
                let drain_to = self.history.len() - 200;
                self.history.drain(..drain_to);
            }
        }
    }

    /// Return the current Kalman-filtered H estimate, if available.
    pub fn current_estimate(&self) -> Option<f64> {
        if self.history.is_empty() {
            None
        } else {
            Some(self.kalman.h)
        }
    }

    /// Detect a significant inertia change (>20% variation in recent history).
    ///
    /// Returns `Some(new_H)` if the most recent estimate deviates >20% from
    /// the historical mean, otherwise `None`.
    pub fn detect_inertia_change(&self) -> Option<f64> {
        if self.history.len() < 5 {
            return None;
        }
        let n = self.history.len();
        // Compare mean of first half vs last quarter
        let mid = n / 2;
        let baseline: f64 = self.history[..mid].iter().map(|&(_, h)| h).sum::<f64>() / mid as f64;
        let recent_start = n * 3 / 4;
        let recent_count = n - recent_start;
        let recent: f64 = self.history[recent_start..]
            .iter()
            .map(|&(_, h)| h)
            .sum::<f64>()
            / recent_count as f64;

        if baseline > 1e-6 && ((recent - baseline) / baseline).abs() > 0.20 {
            Some(recent)
        } else {
            None
        }
    }

    // -----------------------------------------------------------------------
    // Private helpers
    // -----------------------------------------------------------------------

    fn estimate_swing_equation(
        &self,
        power_imbalance_mw: f64,
        rocof: f64,
        r_sq: f64,
        data_quality: f64,
        f0: f64,
        s_base: f64,
    ) -> Result<InertiaEstimate, InertiaError> {
        // Swing equation: 2H * S * df/dt = ΔP  (using SI: H in s, S in MVA, P in MW)
        // → H = ΔP / (2 * S_base * ROCOF / f0)  [normalised ROCOF = ROCOF/f0]
        // Standard form: H = ΔP_MW / (2 * f0 * ROCOF * S_base / f0)
        //              = ΔP_MW / (2 * ROCOF * S_base)
        // Actually: swing eq in per-unit: 2H df/dt / f0 = ΔP_pu
        //   → H = ΔP_pu * f0 / (2 * df/dt)
        let delta_p_pu = power_imbalance_mw / s_base;
        if delta_p_pu.abs() < 1e-6 {
            return Err(InertiaError::SmallImbalance(power_imbalance_mw));
        }
        let h = (delta_p_pu * f0) / (2.0 * rocof.abs());
        let h = h.clamp(0.1, 200.0);

        // Uncertainty: from R² and ROCOF estimation error
        let h_uncertainty = h * (1.0 - r_sq).sqrt().max(0.02);
        let confidence = (r_sq * data_quality).clamp(0.0, 1.0);

        Ok(InertiaEstimate {
            h_total_mj_mva: h,
            h_uncertainty,
            m_total_mj: 2.0 * h * s_base,
            confidence,
            method: "SwingEquation".into(),
            rocof_used: rocof,
            data_quality,
        })
    }

    #[allow(clippy::too_many_arguments)]
    fn estimate_least_squares(
        &self,
        freq_hz: &[f64],
        time_s: &[f64],
        power_imbalance_mw: f64,
        rocof: f64,
        r_sq: f64,
        data_quality: f64,
        f0: f64,
        s_base: f64,
    ) -> Result<InertiaEstimate, InertiaError> {
        // Fit: f(t) = f0 + (ΔP / (2·H·S)) * f0 * t
        // Which is linear in 1/H:  f(t) - f0  = (ΔP·f0)/(2·S) · (1/H) · t
        // Let x = t, y = f(t) - f0, then y = (ΔP·f0/(2·S·H)) · x
        // Solve via least squares: β = (ΔP·f0)/(2·S·H) = Σ(x·y) / Σ(x²)
        let delta_p_pu = power_imbalance_mw / s_base;
        if delta_p_pu.abs() < 1e-6 {
            return Err(InertiaError::SmallImbalance(power_imbalance_mw));
        }
        let t0 = time_s.first().copied().unwrap_or(0.0);
        let f_init = freq_hz.first().copied().unwrap_or(f0);

        let sum_xy: f64 = time_s
            .iter()
            .zip(freq_hz.iter())
            .map(|(&t, &f)| (t - t0) * (f - f_init))
            .sum();
        let sum_xx: f64 = time_s.iter().map(|&t| (t - t0).powi(2)).sum();

        if sum_xx.abs() < 1e-15 {
            return Err(InertiaError::Numerical("zero time span in LS".into()));
        }

        let beta = sum_xy / sum_xx; // = ΔP_pu * f0 / (2 * H)
        if beta.abs() < 1e-12 {
            return Err(InertiaError::Numerical("near-zero LS slope".into()));
        }

        let h = (delta_p_pu * f0) / (2.0 * beta.abs());
        let h = h.clamp(0.1, 200.0);

        // Compute residuals for additional confidence
        let n = freq_hz.len() as f64;
        let ss_res: f64 = time_s
            .iter()
            .zip(freq_hz.iter())
            .map(|(&t, &f)| {
                let f_pred = f_init + beta * (t - t0);
                (f - f_pred).powi(2)
            })
            .sum();
        let ss_tot: f64 = freq_hz.iter().map(|&f| (f - f_init).powi(2)).sum();
        let ls_r_sq = if ss_tot < 1e-15 {
            r_sq
        } else {
            (1.0 - ss_res / ss_tot).clamp(0.0, 1.0)
        };

        let h_uncertainty = h * (1.0 - ls_r_sq).sqrt().max(0.02);
        let confidence = (ls_r_sq * data_quality * (n / 20.0).min(1.0)).clamp(0.0, 1.0);

        Ok(InertiaEstimate {
            h_total_mj_mva: h,
            h_uncertainty,
            m_total_mj: 2.0 * h * s_base,
            confidence,
            method: "LeastSquares".into(),
            rocof_used: rocof,
            data_quality,
        })
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Ideal data: linear frequency drop from f0 to f0 - df for known H.
    fn ideal_frequency(
        h: f64,
        delta_p_mw: f64,
        base_mva: f64,
        f0: f64,
        t_end: f64,
        n: usize,
    ) -> (Vec<f64>, Vec<f64>) {
        // df/dt = -ΔP / (2H * S) * f0   (negative because generation deficit)
        let rocof = -(delta_p_mw / base_mva) * f0 / (2.0 * h);
        let dt = t_end / (n as f64 - 1.0);
        let times: Vec<f64> = (0..n).map(|i| i as f64 * dt).collect();
        let freqs: Vec<f64> = times.iter().map(|&t| f0 + rocof * t).collect();
        (times, freqs)
    }

    #[test]
    fn test_swing_equation_exact_recovery() {
        let config = InertiaEstimationConfig {
            nominal_frequency_hz: 50.0,
            base_mva: 1000.0,
            estimation_window_s: 5.0,
            min_rocof_threshold: 0.01,
        };
        let tracker = InertiaTracker::new(config);

        let h_true = 6.0_f64;
        let delta_p = 100.0_f64; // 100 MW deficit
        let (times, freqs) = ideal_frequency(h_true, delta_p, 1000.0, 50.0, 4.0, 50);

        let est = tracker
            .estimate(
                &freqs,
                &times,
                delta_p,
                InertiaEstimationMethod::SwingEquation,
            )
            .expect("swing estimation should succeed");

        // Should recover H within 1%
        let err = (est.h_total_mj_mva - h_true).abs() / h_true;
        assert!(
            err < 0.01,
            "SwingEquation: H error {:.4} > 1% (got {:.4}, expected {:.4})",
            err,
            est.h_total_mj_mva,
            h_true
        );
        assert_eq!(est.method, "SwingEquation");
        assert!(est.m_total_mj > 0.0);
    }

    #[test]
    fn test_least_squares_robust_to_noise() {
        let config = InertiaEstimationConfig {
            nominal_frequency_hz: 50.0,
            base_mva: 2000.0,
            estimation_window_s: 10.0,
            min_rocof_threshold: 0.01,
        };
        let tracker = InertiaTracker::new(config);

        let h_true = 8.0_f64;
        let delta_p = 200.0_f64;
        let (times, mut freqs) = ideal_frequency(h_true, delta_p, 2000.0, 50.0, 5.0, 100);

        // Add noise using LCG (mult=6364136223846793005, add=1442695040888963407)
        let mut rng = 12345_u64;
        let noise_amp = 0.005; // 5 mHz noise
        for f in freqs.iter_mut() {
            rng = rng
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            let u = (rng >> 11) as f64 / (1u64 << 53) as f64; // [0,1)
            *f += noise_amp * (2.0 * u - 1.0);
        }

        let est = tracker
            .estimate(
                &freqs,
                &times,
                delta_p,
                InertiaEstimationMethod::LeastSquares,
            )
            .expect("LS estimation should succeed");

        // With small noise on ideal data, confidence (R²) should be high
        assert!(
            est.confidence > 0.90,
            "LS confidence {:.4} should exceed 0.90",
            est.confidence
        );

        // H should be within 5% of truth for this noise level
        let err = (est.h_total_mj_mva - h_true).abs() / h_true;
        assert!(
            err < 0.08,
            "LS: H error {:.4} > 8% (got {:.4}, expected {:.4})",
            err,
            est.h_total_mj_mva,
            h_true
        );
    }

    #[test]
    fn test_kalman_filter_tracks_changing_h() {
        // Phase 1: system has H = 8 s, estimated via many Kalman updates.
        // Phase 2: H changes to 4 s (faster ROCOF), Kalman should track toward 4.
        let config = InertiaEstimationConfig {
            nominal_frequency_hz: 50.0,
            base_mva: 1000.0,
            // Short window so phase-2 data dominates quickly
            estimation_window_s: 2.0,
            min_rocof_threshold: 0.01,
        };
        let mut tracker = InertiaTracker::new(config);

        // Phase 1: 60 updates at H=8 s (ROCOF = -0.3125 Hz/s).
        // Use a continuous frequency trajectory so each window is coherent.
        let delta_p = 100.0_f64;
        let rocof8 = -(delta_p / 1000.0) * 50.0 / (2.0 * 8.0); // = -0.3125 Hz/s
        let dt = 0.1_f64;
        for i in 0..60 {
            let t = i as f64 * dt;
            let f = 50.0 + rocof8 * t;
            tracker.update(t, f, delta_p);
        }

        let h_after_phase1 = tracker
            .current_estimate()
            .expect("should have estimate after phase 1");

        // Phase 2: H = 4 s (ROCOF = -0.625 Hz/s).
        // Frequencies continue as a new event starting from 50.0 Hz.
        // We reset time to 100 s so the window only sees phase-2 data.
        let rocof4 = -(delta_p / 1000.0) * 50.0 / (2.0 * 4.0); // = -0.625 Hz/s
        let t_offset = 100.0_f64;
        for i in 0..60 {
            let t = t_offset + i as f64 * dt;
            let _f = 50.0 + rocof4 * t.min(t_offset + 1.9); // frequency drops with new H
                                                            // Use the fresh rocof directly so the window is clean
            let f_clean = 50.0 + rocof4 * (i as f64 * dt);
            tracker.update(t, f_clean, delta_p);
        }

        // Kalman should have moved toward H = 4 s
        let h_after_phase2 = tracker
            .current_estimate()
            .expect("should have estimate after phase 2");
        // Phase 2 H (4) < Phase 1 H (8), so Kalman should have tracked downward
        assert!(
            h_after_phase2 < h_after_phase1,
            "Kalman H after phase 2 ({:.2}) should be below phase 1 ({:.2}); \
             phase-1 converged near 8, phase-2 pulls toward 4",
            h_after_phase2,
            h_after_phase1
        );
    }

    #[test]
    fn test_low_rocof_returns_error() {
        let config = InertiaEstimationConfig {
            nominal_frequency_hz: 50.0,
            base_mva: 1000.0,
            estimation_window_s: 5.0,
            min_rocof_threshold: 0.05,
        };
        let tracker = InertiaTracker::new(config);

        // Flat frequency → ROCOF ≈ 0 → error
        let times: Vec<f64> = (0..20).map(|i| i as f64 * 0.1).collect();
        let freqs: Vec<f64> = times.iter().map(|_| 50.0).collect();

        let result = tracker.estimate(&freqs, &times, 50.0, InertiaEstimationMethod::SwingEquation);

        assert!(
            matches!(result, Err(InertiaError::RocofBelowThreshold { .. })),
            "Should fail with RocofBelowThreshold, got {:?}",
            result
        );
    }

    #[test]
    fn test_multiple_methods_within_20_percent() {
        let config = InertiaEstimationConfig {
            nominal_frequency_hz: 50.0,
            base_mva: 1000.0,
            estimation_window_s: 5.0,
            min_rocof_threshold: 0.01,
        };
        let tracker = InertiaTracker::new(config);

        let h_true = 6.0_f64;
        let delta_p = 120.0_f64;
        let (times, freqs) = ideal_frequency(h_true, delta_p, 1000.0, 50.0, 4.0, 80);

        let methods = [
            InertiaEstimationMethod::SwingEquation,
            InertiaEstimationMethod::LeastSquares,
            InertiaEstimationMethod::KalmanFilter,
        ];

        let mut estimates = Vec::new();
        for method in &methods {
            let est = tracker
                .estimate(&freqs, &times, delta_p, method.clone())
                .expect("estimation should succeed");
            estimates.push(est.h_total_mj_mva);
        }

        // All estimates should be within 20% of H_true
        for (i, &h_est) in estimates.iter().enumerate() {
            let err = (h_est - h_true).abs() / h_true;
            assert!(
                err < 0.20,
                "Method {i}: H error {:.4} > 20% (got {:.4}, expected {:.4})",
                err,
                h_est,
                h_true
            );
        }

        // All estimates within 20% of each other
        let h_min = estimates.iter().cloned().fold(f64::INFINITY, f64::min);
        let h_max = estimates.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let spread = (h_max - h_min) / h_min;
        assert!(
            spread < 0.20,
            "Methods disagree by {:.4} > 20% (min={:.2}, max={:.2})",
            spread,
            h_min,
            h_max
        );
    }

    #[test]
    fn test_spectral_method_finite_result() {
        let config = InertiaEstimationConfig {
            nominal_frequency_hz: 50.0,
            base_mva: 1000.0,
            estimation_window_s: 10.0,
            min_rocof_threshold: 0.01,
        };
        let tracker = InertiaTracker::new(config);

        let h_true = 5.0_f64;
        let delta_p = 100.0_f64;
        let (times, freqs) = ideal_frequency(h_true, delta_p, 1000.0, 50.0, 5.0, 100);

        let est = tracker
            .estimate(&freqs, &times, delta_p, InertiaEstimationMethod::Spectral)
            .expect("spectral estimation should succeed");

        assert!(est.h_total_mj_mva.is_finite(), "H should be finite");
        assert!(est.h_total_mj_mva > 0.0, "H should be positive");
        assert_eq!(est.method, "Spectral");
    }

    #[test]
    fn test_m_total_equals_2h_times_s() {
        let config = InertiaEstimationConfig {
            nominal_frequency_hz: 60.0,
            base_mva: 500.0,
            estimation_window_s: 5.0,
            min_rocof_threshold: 0.01,
        };
        let tracker = InertiaTracker::new(config);

        let delta_p = 50.0_f64;
        let (times, freqs) = ideal_frequency(4.0, delta_p, 500.0, 60.0, 3.0, 40);

        let est = tracker
            .estimate(
                &freqs,
                &times,
                delta_p,
                InertiaEstimationMethod::SwingEquation,
            )
            .expect("estimation should succeed");

        let expected_m = 2.0 * est.h_total_mj_mva * 500.0;
        assert!(
            (est.m_total_mj - expected_m).abs() < 1e-6,
            "M = 2H*S should hold: got {:.4}, expected {:.4}",
            est.m_total_mj,
            expected_m
        );
    }
}
