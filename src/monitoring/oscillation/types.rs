//! Auto-generated module
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

use std::f64::consts::PI;
use thiserror::Error;

use super::functions::{
    companion_eigenvalues, compute_stability_index, estimate_amplitude, merge_modes,
    prony_fit_amplitude, prony_solve_ls, solve_linear_system,
};

/// Classification of an oscillatory mode by its frequency.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ModeType {
    /// Global system swing: f < 0.1 \[Hz\].
    GlobalMode,
    /// Inter-area oscillation: 0.1 ≤ f < 0.8 \[Hz\].
    InterAreaMode,
    /// Local area oscillation: 0.8 ≤ f < 2.0 \[Hz\].
    LocalAreaMode,
    /// Local plant mode (generator vs. transformer): 2.0 ≤ f < 3.0 \[Hz\].
    LocalPlantMode,
    /// Turbine-generator torsional mode: f ≥ 3.0 \[Hz\].
    TorsionalMode,
    /// Local mode (WAM): f > 0.7 Hz, involves 1-2 generators.
    Local,
    /// Inter-area mode (WAM): 0.1–0.7 Hz, involves generator groups.
    InterArea,
    /// Control-system interaction mode: f > 2 Hz.
    Control,
    /// Forced oscillation: driven by periodic external disturbance.
    ForcedOscillation,
}
/// Wide-Area Monitoring System for real-time oscillation tracking.
pub struct WamOscillationMonitor {
    /// Number of buses being monitored.
    pub n_buses: usize,
    /// PMU sampling rate \[Hz\].
    pub sampling_rate_hz: f64,
    /// Prony analyser instance.
    pub prony_analyzer: PronyAnalyzer,
    /// Fourier oscillation detector instance.
    pub fourier_detector: FourierOscillationDetector,
    /// Damping ratio threshold for alerts (default 0.05).
    pub alert_damping_threshold: f64,
    /// Amplitude threshold for alerts.
    pub alert_amplitude_threshold: f64,
    /// Rolling buffer of voltage angle measurements \[bus\]\[sample\].
    pub angle_buffers: Vec<Vec<f64>>,
    /// Rolling buffer of rotor speed / frequency deviation measurements \[bus\]\[sample\].
    pub speed_buffers: Vec<Vec<f64>>,
    /// Current timestamp \[s\].
    timestamp: f64,
}
impl WamOscillationMonitor {
    /// Create a new WAM oscillation monitor.
    pub fn new(n_buses: usize, sampling_rate_hz: f64) -> Self {
        let window = 256.min((sampling_rate_hz * 10.0) as usize).max(32);
        Self {
            n_buses,
            sampling_rate_hz,
            prony_analyzer: PronyAnalyzer::new(4, sampling_rate_hz, window),
            fourier_detector: FourierOscillationDetector::new(sampling_rate_hz, window),
            alert_damping_threshold: 0.05,
            alert_amplitude_threshold: 0.01,
            angle_buffers: vec![Vec::new(); n_buses],
            speed_buffers: vec![Vec::new(); n_buses],
            timestamp: 0.0,
        }
    }
    /// Ingest a new measurement sample from all buses.
    pub fn update(&mut self, bus_angles: &[f64], bus_speeds: &[f64], timestamp: f64) {
        self.timestamp = timestamp;
        let max_buf = self.prony_analyzer.window_size.max(64);
        for (i, buf) in self.angle_buffers.iter_mut().enumerate() {
            let val = bus_angles.get(i).copied().unwrap_or(0.0);
            buf.push(val);
            if buf.len() > max_buf {
                buf.remove(0);
            }
        }
        for (i, buf) in self.speed_buffers.iter_mut().enumerate() {
            let val = bus_speeds.get(i).copied().unwrap_or(0.0);
            buf.push(val);
            if buf.len() > max_buf {
                buf.remove(0);
            }
        }
    }
    /// Analyse the current measurement buffer and produce an oscillation report.
    pub fn analyze(&self) -> Result<OscillationReport, OscillationError> {
        if self.n_buses == 0 {
            return Err(OscillationError::InvalidConfig(
                "WAM monitor has no buses configured".to_string(),
            ));
        }
        let min_samples = self
            .angle_buffers
            .iter()
            .map(|b| b.len())
            .min()
            .unwrap_or(0);
        if min_samples < 4 {
            return Err(OscillationError::InvalidConfig(format!(
                "Insufficient samples in buffers: need ≥4, got {}",
                min_samples
            )));
        }
        let mut detected_modes: Vec<DetectedMode> = Vec::new();
        for bus_i in 0..self.n_buses {
            let signal = &self.angle_buffers[bus_i];
            if signal.len() < 4 {
                continue;
            }
            let analyzer = PronyAnalyzer::new(
                self.prony_analyzer.n_modes,
                self.sampling_rate_hz,
                signal.len(),
            );
            let modes = match analyzer.analyze(signal) {
                Ok(m) => m,
                Err(_) => continue,
            };
            for (freq, damp, amp, _phase) in modes {
                if !(0.05..=5.0).contains(&freq) {
                    continue;
                }
                let participation =
                    Self::compute_participation(&self.angle_buffers, freq, self.sampling_rate_hz);
                let n_participating = participation.iter().filter(|&&p| p > 0.1).count();
                let mode_type = Self::classify_wam_mode(freq, n_participating);
                let is_poorly_damped = damp < self.alert_damping_threshold;
                let confidence = if amp > 1e-6 {
                    (1.0 - (damp - 0.05).abs().min(0.5) / 0.5).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                detected_modes.push(DetectedMode {
                    frequency_hz: freq,
                    damping_ratio: damp,
                    amplitude: amp,
                    participation,
                    mode_type,
                    is_poorly_damped,
                    confidence,
                });
            }
        }
        if let Some(forced_freq) = self
            .fourier_detector
            .detect_forced_oscillation(&self.angle_buffers)
        {
            let already_present = detected_modes
                .iter()
                .any(|m| (m.frequency_hz - forced_freq).abs() < 0.05);
            if !already_present {
                let participation = Self::compute_participation(
                    &self.angle_buffers,
                    forced_freq,
                    self.sampling_rate_hz,
                );
                detected_modes.push(DetectedMode {
                    frequency_hz: forced_freq,
                    damping_ratio: 0.0,
                    amplitude: 0.01,
                    participation,
                    mode_type: ModeType::ForcedOscillation,
                    is_poorly_damped: true,
                    confidence: 0.7,
                });
            } else {
                if let Some(m) = detected_modes
                    .iter_mut()
                    .find(|m| (m.frequency_hz - forced_freq).abs() < 0.05)
                {
                    m.mode_type = ModeType::ForcedOscillation;
                }
            }
        }
        let poorly_damped_modes: Vec<usize> = detected_modes
            .iter()
            .enumerate()
            .filter(|(_, m)| m.is_poorly_damped)
            .map(|(i, _)| i)
            .collect();
        let mut alerts: Vec<OscillationAlert> = Vec::new();
        for mode in &detected_modes {
            let severity = if mode.damping_ratio < 0.0 {
                AlertSeverity::Critical
            } else if mode.damping_ratio < self.alert_damping_threshold {
                if mode.amplitude > self.alert_amplitude_threshold {
                    AlertSeverity::Warning
                } else {
                    AlertSeverity::Info
                }
            } else {
                continue;
            };
            let description = format!(
                "Mode at {:.3} Hz: ζ = {:.4}, A = {:.4}, type = {:?}",
                mode.frequency_hz, mode.damping_ratio, mode.amplitude, mode.mode_type
            );
            alerts.push(OscillationAlert {
                severity,
                mode_frequency_hz: mode.frequency_hz,
                damping_ratio: mode.damping_ratio,
                description,
            });
        }
        let mut recommended_pss_tuning: Vec<PssTuningRecommendation> = Vec::new();
        for &idx in &poorly_damped_modes {
            let mode = &detected_modes[idx];
            let max_part_bus = mode
                .participation
                .iter()
                .enumerate()
                .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
                .map(|(i, _)| i)
                .unwrap_or(0);
            recommended_pss_tuning.push(Self::recommend_pss_tuning(max_part_bus, mode));
        }
        let system_damping_index = Self::compute_damping_index(&detected_modes);
        Ok(OscillationReport {
            timestamp: self.timestamp,
            detected_modes,
            poorly_damped_modes,
            alerts,
            system_damping_index,
            recommended_pss_tuning,
        })
    }
    /// Compute the inter-machine swing signal between bus `i` and bus `j`.
    ///
    /// Returns the angle difference time series δ_i(t) − δ_j(t).
    pub fn compute_swing_mode(&self, bus_i: usize, bus_j: usize) -> Vec<f64> {
        let a = self
            .angle_buffers
            .get(bus_i)
            .map(|v| v.as_slice())
            .unwrap_or(&[]);
        let b = self
            .angle_buffers
            .get(bus_j)
            .map(|v| v.as_slice())
            .unwrap_or(&[]);
        let len = a.len().min(b.len());
        (0..len).map(|k| a[k] - b[k]).collect()
    }
    /// Compute participation factors: RMS band-power contribution of each bus
    /// to a mode at `mode_freq_hz`.
    pub(crate) fn compute_participation(
        signals: &[Vec<f64>],
        mode_freq_hz: f64,
        sampling_rate_hz: f64,
    ) -> Vec<f64> {
        let band_half = 0.1_f64;
        let mut participations: Vec<f64> = signals
            .iter()
            .map(|sig| {
                let n = sig.len();
                if n == 0 {
                    return 0.0;
                }
                let dt = 1.0 / sampling_rate_hz;
                let omega = 2.0 * PI * mode_freq_hz;
                let omega_low = 2.0 * PI * (mode_freq_hz - band_half).max(0.0);
                let omega_high = 2.0 * PI * (mode_freq_hz + band_half);
                let _ = (omega_low, omega_high);
                let mut re = 0.0f64;
                let mut im = 0.0f64;
                for (k, &xk) in sig.iter().enumerate() {
                    let t = k as f64 * dt;
                    re += xk * (omega * t).cos();
                    im += xk * (omega * t).sin();
                }
                ((re * re + im * im).sqrt() / n as f64).max(0.0)
            })
            .collect();
        let max_p = participations
            .iter()
            .cloned()
            .fold(0.0_f64, f64::max)
            .max(1e-15);
        for p in &mut participations {
            *p /= max_p;
        }
        participations
    }
    /// Classify a WAM mode based on frequency and number of participating buses.
    pub(crate) fn classify_wam_mode(freq_hz: f64, n_buses_participating: usize) -> ModeType {
        if freq_hz > 2.0 {
            ModeType::Control
        } else if freq_hz > 0.7 || n_buses_participating <= 2 {
            ModeType::Local
        } else {
            ModeType::InterArea
        }
    }
    /// Generate PSS lead-lag tuning recommendation for a bus/mode pair.
    ///
    /// Target phase advance ≈ 45° at mode frequency.
    /// T_lead = 1 / (2π·f · tan(45°/n_stages))
    pub(crate) fn recommend_pss_tuning(
        bus_id: usize,
        mode: &DetectedMode,
    ) -> PssTuningRecommendation {
        let f = mode.frequency_hz.max(0.01);
        let omega = 2.0 * PI * f;
        let n_stages = 2.0_f64;
        let angle_per_stage = (45.0_f64).to_radians() / n_stages;
        let t_lead = 1.0 / (omega * angle_per_stage.tan());
        let t_lag = t_lead / 10.0;
        let t_washout = 10.0 / omega;
        let threshold = 0.05_f64;
        let damping_deficit = (threshold - mode.damping_ratio).max(0.0_f64);
        let suggested_gain = (5.0_f64 * (1.0_f64 + 10.0_f64 * damping_deficit)).min(50.0_f64);
        let expected_improvement = (0.05_f64 * suggested_gain / 10.0_f64).min(0.2_f64);
        let _ = t_lag;
        PssTuningRecommendation {
            bus_id,
            target_frequency_hz: f,
            suggested_gain,
            suggested_washout_s: t_washout,
            expected_damping_improvement: expected_improvement,
        }
    }
    /// Compute composite system damping index as weighted average of mode damping ratios.
    ///
    /// Modes with higher amplitude receive greater weight.
    /// Returns value in \[0, 1\]: 1 = well damped, 0 = unstable.
    pub(crate) fn compute_damping_index(modes: &[DetectedMode]) -> f64 {
        if modes.is_empty() {
            return 1.0;
        }
        let total_weight: f64 = modes.iter().map(|m| m.amplitude.max(1e-10)).sum();
        if total_weight < 1e-20 {
            return 1.0;
        }
        let weighted_damp: f64 = modes
            .iter()
            .map(|m| m.damping_ratio * m.amplitude.max(1e-10))
            .sum::<f64>()
            / total_weight;
        ((weighted_damp + 0.1) / 0.2).clamp(0.0, 1.0)
    }
}
/// A single detected oscillatory mode.
#[derive(Debug, Clone)]
pub struct OscillationMode {
    /// Mode frequency \[Hz\].
    pub frequency_hz: f64,
    /// Damping ratio ζ (positive = decaying, negative = growing).
    pub damping_ratio: f64,
    /// Normalised mode amplitude \[pu\] (peak swing value).
    pub amplitude: f64,
    /// Relative mode energy (fraction of total signal energy).
    pub energy: f64,
    /// Mode classification.
    pub mode_type: ModeType,
    /// Indices of PMU channels that exhibit this mode with significant amplitude.
    pub participating_signals: Vec<usize>,
}
/// A detected oscillation mode extracted from PMU measurements.
#[derive(Debug, Clone)]
pub struct DetectedMode {
    /// Oscillation frequency \[Hz\].
    pub frequency_hz: f64,
    /// Damping ratio ζ (positive = stable, negative = unstable).
    pub damping_ratio: f64,
    /// Normalised amplitude (peak swing value).
    pub amplitude: f64,
    /// Bus participation factors (RMS contribution per bus).
    pub participation: Vec<f64>,
    /// Mode classification.
    pub mode_type: ModeType,
    /// True when ζ < 0.05 (poorly-damped threshold).
    pub is_poorly_damped: bool,
    /// Confidence indicator \[0, 1\]: 1 = high confidence.
    pub confidence: f64,
}
/// Prony method modal extractor for time-series PMU signals.
///
/// Decomposes a discrete-time signal into a sum of damped sinusoids:
/// x\[n\] = Σ Aᵢ · exp(σᵢ · n·Δt) · cos(2π·fᵢ · n·Δt + φᵢ)
pub struct PronyAnalyzer {
    /// Number of modes to extract.
    pub n_modes: usize,
    /// Measurement sampling rate \[Hz\].
    pub sampling_rate_hz: f64,
    /// Analysis window size in samples.
    pub window_size: usize,
}
impl PronyAnalyzer {
    /// Create a new Prony analyser.
    pub fn new(n_modes: usize, sampling_rate_hz: f64, window_size: usize) -> Self {
        Self {
            n_modes,
            sampling_rate_hz,
            window_size,
        }
    }
    /// Extract modal parameters from `signal` using the Prony method.
    ///
    /// Returns a vector of `(frequency_hz, damping_ratio, amplitude, phase_rad)` tuples.
    ///
    /// # Steps
    /// 1. Build Hankel matrix from signal samples.
    /// 2. Solve for AR coefficients via normal equations (least squares).
    /// 3. Find roots of characteristic polynomial via companion matrix + QR iteration.
    /// 4. Extract (σ, ω) from z = exp(λ·Δt).
    /// 5. Fit amplitudes via least squares over Vandermonde matrix.
    pub fn analyze(&self, signal: &[f64]) -> Result<Vec<(f64, f64, f64, f64)>, OscillationError> {
        let n = signal.len();
        if n < 4 {
            return Err(OscillationError::InvalidConfig(format!(
                "Prony analysis requires at least 4 samples, got {}",
                n
            )));
        }
        if self.n_modes == 0 {
            return Err(OscillationError::InvalidConfig(
                "n_modes must be at least 1".to_string(),
            ));
        }
        if self.sampling_rate_hz <= 0.0 {
            return Err(OscillationError::InvalidConfig(
                "sampling_rate_hz must be positive".to_string(),
            ));
        }
        let dt = 1.0 / self.sampling_rate_hz;
        let p = (self.n_modes * 2).min(n / 3).max(2);
        let rows = n - p;
        if rows < p {
            return Err(OscillationError::InvalidConfig(format!(
                "Signal too short for {} modes: need {} samples, got {}",
                self.n_modes,
                p * 2 + p,
                n
            )));
        }
        let mut hth = vec![0.0f64; p * p];
        let mut htb = vec![0.0f64; p];
        for k in 0..rows {
            let bk = signal[k + p];
            for i in 0..p {
                let xi = signal[k + p - 1 - i];
                htb[i] += xi * bk;
                for j in 0..p {
                    hth[i * p + j] += xi * signal[k + p - 1 - j];
                }
            }
        }
        let max_diag = (0..p)
            .map(|i| hth[i * p + i].abs())
            .fold(1e-20_f64, f64::max);
        for i in 0..p {
            hth[i * p + i] += max_diag * 1e-9;
        }
        let a_coeffs = prony_solve_ls(&hth, &htb, p).ok_or_else(|| {
            OscillationError::NumericalError("Prony normal-equation matrix is singular".to_string())
        })?;
        let eigenvalues = companion_eigenvalues(&a_coeffs, p);
        let mut results: Vec<(f64, f64, f64, f64)> = Vec::new();
        for (zr, zi) in &eigenvalues {
            let z_mag = (zr * zr + zi * zi).sqrt();
            if !(1e-10..=10.0).contains(&z_mag) {
                continue;
            }
            let sigma = z_mag.ln() / dt;
            let omega = zi.atan2(*zr) / dt;
            let freq_hz = omega.abs() / (2.0 * PI);
            if freq_hz < 1e-4 {
                continue;
            }
            let s_mag = (sigma * sigma + omega * omega).sqrt();
            let damping = if s_mag > 1e-10 { -sigma / s_mag } else { 1.0 };
            let (amplitude, phase) = prony_fit_amplitude(signal, sigma, omega, dt);
            results.push((freq_hz, damping, amplitude, phase));
        }
        results.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap_or(std::cmp::Ordering::Equal));
        results.truncate(self.n_modes);
        Ok(results)
    }
    /// Build the companion matrix for polynomial
    /// p(z) = z^n + a\[n-1\]·z^{n-1} + … + a\[0\]
    ///
    /// The companion matrix has the AR coefficients (negated) in the first row
    /// and 1s on the sub-diagonal.
    pub fn build_companion_matrix(coeffs: &[f64]) -> Vec<Vec<f64>> {
        let n = coeffs.len();
        if n == 0 {
            return Vec::new();
        }
        let mut mat = vec![vec![0.0f64; n]; n];
        for j in 0..n {
            mat[0][j] = -coeffs[n - 1 - j];
        }
        for i in 1..n {
            mat[i][i - 1] = 1.0;
        }
        mat
    }
    /// Power iteration to find the dominant (largest-magnitude) real eigenvalue.
    ///
    /// Operates on a flat row-major matrix.
    pub fn dominant_eigenvalue(matrix: &[Vec<f64>], max_iter: usize) -> f64 {
        let n = matrix.len();
        if n == 0 {
            return 0.0;
        }
        let mut v: Vec<f64> = vec![1.0; n];
        let mut eigenval = 0.0f64;
        for _ in 0..max_iter {
            let mut w: Vec<f64> = vec![0.0; n];
            for i in 0..n {
                for j in 0..n {
                    w[i] += matrix[i][j] * v[j];
                }
            }
            let norm: f64 = w.iter().map(|&x| x * x).sum::<f64>().sqrt();
            if norm < 1e-15 {
                break;
            }
            eigenval = norm;
            for i in 0..n {
                v[i] = w[i] / norm;
            }
        }
        let mut num = 0.0f64;
        let mut den = 0.0f64;
        let mut mv = vec![0.0f64; n];
        for i in 0..n {
            for j in 0..n {
                mv[i] += matrix[i][j] * v[j];
            }
        }
        for i in 0..n {
            num += v[i] * mv[i];
            den += v[i] * v[i];
        }
        if den > 1e-15 {
            num / den
        } else {
            eigenval
        }
    }
    /// Reconstruct signal from Prony modes.
    ///
    /// x\[n\] = Σ Aᵢ · exp(σᵢ · n·Δt) · cos(2π·fᵢ · n·Δt + φᵢ)
    pub fn reconstruct(&self, modes: &[(f64, f64, f64, f64)], n_samples: usize) -> Vec<f64> {
        if modes.is_empty() || n_samples == 0 || self.sampling_rate_hz <= 0.0 {
            return vec![0.0; n_samples];
        }
        let dt = 1.0 / self.sampling_rate_hz;
        (0..n_samples)
            .map(|k| {
                let t = k as f64 * dt;
                modes
                    .iter()
                    .map(|&(freq, damp, amp, phase)| {
                        let omega = 2.0 * PI * freq;
                        let sigma = -damp * omega;
                        amp * (sigma * t).exp() * (omega * t + phase).cos()
                    })
                    .sum::<f64>()
            })
            .collect()
    }
}
/// Result of post-disturbance ringdown analysis.
pub struct RingdownResult {
    /// Extracted modes: (frequency_hz, damping_ratio, amplitude, phase_rad).
    pub modes: Vec<(f64, f64, f64, f64)>,
    /// Time for dominant mode to decay to 1/e of initial amplitude \[s\].
    pub decay_time_s: f64,
    /// Settling time: time for all modes to reach <5% of initial amplitude \[s\].
    pub settling_time_s: f64,
    /// True if all extracted modes have positive damping ratio.
    pub is_stable: bool,
}
/// Post-disturbance ringdown analyser using the Prony method.
pub struct RingdownAnalyzer {
    /// Number of modes to extract.
    pub n_modes: usize,
    /// Measurement sampling rate \[Hz\].
    pub sampling_rate_hz: f64,
}
impl RingdownAnalyzer {
    /// Create a new ringdown analyser.
    pub fn new(n_modes: usize, sampling_rate_hz: f64) -> Self {
        Self {
            n_modes,
            sampling_rate_hz,
        }
    }
    /// Analyse a post-disturbance ringdown signal using the Prony method.
    pub fn analyze(&self, signal: &[f64]) -> Result<RingdownResult, OscillationError> {
        let n = signal.len();
        if n < 4 {
            return Err(OscillationError::InvalidConfig(format!(
                "Ringdown analysis requires at least 4 samples, got {}",
                n
            )));
        }
        let analyzer = PronyAnalyzer::new(self.n_modes, self.sampling_rate_hz, n);
        let modes = analyzer.analyze(signal)?;
        let is_stable = modes.iter().all(|&(_, damp, _, _)| damp >= 0.0);
        let decay_time_s = Self::estimate_decay_time(&modes);
        let settling_time_s = Self::estimate_settling_time(&modes);
        Ok(RingdownResult {
            modes,
            decay_time_s,
            settling_time_s,
            is_stable,
        })
    }
    /// Estimate decay time from the dominant (highest amplitude) mode.
    ///
    /// T_decay = 1 / |σ| = 1 / (2π · f · |ζ|)
    fn estimate_decay_time(modes: &[(f64, f64, f64, f64)]) -> f64 {
        if modes.is_empty() {
            return f64::INFINITY;
        }
        let dom = modes
            .iter()
            .max_by(|a, b| a.2.partial_cmp(&b.2).unwrap_or(std::cmp::Ordering::Equal));
        if let Some(&(freq, damp, _, _)) = dom {
            let sigma = (2.0 * PI * freq * damp.abs()).max(1e-10);
            1.0 / sigma
        } else {
            f64::INFINITY
        }
    }
    /// Estimate settling time: when all modes decay below 5% of initial amplitude.
    ///
    /// For mode i: t_settle_i = ln(20) / |σ_i|  (since exp(-|σ|·t) < 0.05 → t > ln(20)/|σ|)
    fn estimate_settling_time(modes: &[(f64, f64, f64, f64)]) -> f64 {
        if modes.is_empty() {
            return f64::INFINITY;
        }
        let ln20 = 20.0_f64.ln();
        modes
            .iter()
            .map(|&(freq, damp, _, _)| {
                let sigma = (2.0 * PI * freq * damp.abs()).max(1e-10);
                ln20 / sigma
            })
            .fold(0.0_f64, f64::max)
    }
}
/// Errors from the oscillation monitoring pipeline.
#[derive(Debug, Error)]
pub enum OscillationError {
    /// Not enough samples to perform Prony analysis.
    #[error("insufficient samples: need at least {required}, got {got}")]
    InsufficientSamples { required: usize, got: usize },
    /// Configuration parameter is invalid.
    #[error("invalid oscillation monitor configuration: {0}")]
    InvalidConfig(String),
    /// Numerical failure inside the Prony/QR solver.
    #[error("numerical error in Prony analysis: {0}")]
    NumericalError(String),
}
/// Configuration for the real-time oscillation monitor.
#[derive(Debug, Clone)]
pub struct OscillationMonitorConfig {
    /// PMU data rate \[Hz\] (frames per second; typical: 25, 30, 50, 60).
    pub sampling_rate_hz: f64,
    /// Analysis window length \[s\] — number of samples = rate × window.
    pub analysis_window_s: f64,
    /// Minimum relative mode energy to include in the result (e.g. 0.001).
    pub min_mode_energy: f64,
    /// Frequency range `(f_min, f_max)` \[Hz\] for inter-area mode search.
    pub frequency_range_hz: (f64, f64),
    /// Alarm threshold: raise advisory if damping ratio falls below this.
    pub alarm_damping_threshold: f64,
    /// Alarm threshold: raise advisory if mode amplitude exceeds this \[pu\].
    pub alarm_amplitude_threshold: f64,
}
/// An alarm associated with a particular oscillation mode.
#[derive(Debug, Clone)]
pub struct OscillationAlarmLevel {
    /// Frequency of the alarming mode \[Hz\].
    pub frequency_hz: f64,
    /// Current damping ratio of the alarming mode.
    pub damping_ratio: f64,
    /// Alarm severity.
    pub severity: AlarmLevel,
    /// Recent trend in the damping ratio.
    pub trend: DampingTrend,
}
/// Result of a single oscillation monitor analysis epoch.
#[derive(Debug, Clone)]
pub struct OscillationMonitorResult {
    /// All detected modes that exceeded the energy threshold.
    pub modes: Vec<OscillationMode>,
    /// Active alarms (if any).
    pub alarms: Vec<OscillationAlarmLevel>,
    /// The mode with the highest energy content (None if no modes).
    pub dominant_mode: Option<OscillationMode>,
    /// System-level stability index ∈ \[0, 1\]: 1 = fully stable, 0 = critical.
    pub system_stability_index: f64,
    /// Timestamp of this analysis epoch \[s\] (arbitrary reference).
    pub analysis_timestamp: f64,
}
/// Direction of change in damping over recent history.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DampingTrend {
    /// Damping ratio is increasing (improving stability).
    Improving,
    /// Damping ratio is approximately constant.
    Stable,
    /// Damping ratio is decreasing (deteriorating stability).
    Deteriorating,
}
/// Oscillation analysis report from the WAM monitor.
pub struct OscillationReport {
    /// Analysis timestamp \[s\].
    pub timestamp: f64,
    /// All detected modes.
    pub detected_modes: Vec<DetectedMode>,
    /// Indices into `detected_modes` for poorly-damped modes (ζ < threshold).
    pub poorly_damped_modes: Vec<usize>,
    /// Active oscillation alerts.
    pub alerts: Vec<OscillationAlert>,
    /// Composite system damping index \[0, 1\]: 1 = well damped.
    pub system_damping_index: f64,
    /// PSS tuning recommendations for affected buses.
    pub recommended_pss_tuning: Vec<PssTuningRecommendation>,
}
/// Alarm severity level.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum AlarmLevel {
    /// No action required.
    Normal,
    /// Awareness only — monitor closely.
    Advisory,
    /// Operator action recommended.
    Alert,
    /// Immediate operator action required.
    Emergency,
}
/// Real-time power system oscillation monitor.
#[derive(Debug, Clone)]
pub struct OscillationMonitor {
    config: OscillationMonitorConfig,
    /// Rolling history of dominant modes for trend detection.
    pub(crate) history: Vec<OscillationMode>,
}
impl OscillationMonitor {
    /// Create a new oscillation monitor.
    pub fn new(config: OscillationMonitorConfig) -> Self {
        Self {
            config,
            history: Vec::new(),
        }
    }
    /// Analyse a multi-channel PMU signal for oscillation modes.
    ///
    /// # Arguments
    ///
    /// * `signals` — `[channel][sample]` array of PMU measurements \[pu\].
    /// * `dt_s` — sample interval \[s\] (= 1 / sampling_rate_hz).
    ///
    /// # Returns
    ///
    /// [`OscillationMonitorResult`] containing all detected modes and alarms.
    pub fn analyze(
        &self,
        signals: &[Vec<f64>],
        dt_s: f64,
    ) -> Result<OscillationMonitorResult, OscillationError> {
        if signals.is_empty() {
            return Err(OscillationError::InsufficientSamples {
                required: 4,
                got: 0,
            });
        }
        if dt_s <= 0.0 {
            return Err(OscillationError::InvalidConfig(
                "dt_s must be positive".to_string(),
            ));
        }
        let n_samples = signals[0].len();
        let n_ch = signals.len();
        if n_samples < 4 {
            return Err(OscillationError::InsufficientSamples {
                required: 4,
                got: n_samples,
            });
        }
        let n_modes = (n_samples / 4).clamp(1, 8);
        let mut all_modes: Vec<OscillationMode> = Vec::new();
        for (ch, sig) in signals.iter().enumerate() {
            let mut modes = self.prony_analysis(sig, dt_s, n_modes);
            for m in &mut modes {
                m.participating_signals.push(ch);
            }
            all_modes.extend(modes);
        }
        let mut merged = merge_modes(&all_modes, n_ch);
        let (f_min, f_max) = self.config.frequency_range_hz;
        merged.retain(|m| {
            m.energy >= self.config.min_mode_energy
                && m.frequency_hz >= f_min
                && m.frequency_hz <= f_max
        });
        merged.sort_by(|a, b| {
            b.energy
                .partial_cmp(&a.energy)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        let alarms = self.generate_alarms(&merged);
        let dominant_mode = merged.first().cloned();
        let stability_index = compute_stability_index(&merged);
        Ok(OscillationMonitorResult {
            modes: merged,
            alarms,
            dominant_mode,
            system_stability_index: stability_index,
            analysis_timestamp: 0.0,
        })
    }
    /// Update mode history and detect deteriorating trends.
    ///
    /// Returns a list of alarms if trends are worsening. Keeps the last 10
    /// result epochs in the internal history buffer.
    pub fn update_and_check_trend(
        &mut self,
        result: &OscillationMonitorResult,
    ) -> Vec<OscillationAlarmLevel> {
        if let Some(ref dom) = result.dominant_mode {
            self.history.push(dom.clone());
        }
        if self.history.len() > 10 {
            self.history.drain(0..self.history.len() - 10);
        }
        if self.history.len() < 2 {
            return Vec::new();
        }
        let mut trend_alarms = Vec::new();
        if let Some(ref current) = result.dominant_mode {
            let trend = self.detect_damping_trend(current.frequency_hz);
            if trend == DampingTrend::Deteriorating {
                let severity = if current.damping_ratio < 0.0 {
                    AlarmLevel::Emergency
                } else if current.damping_ratio < self.config.alarm_damping_threshold {
                    AlarmLevel::Alert
                } else {
                    AlarmLevel::Advisory
                };
                trend_alarms.push(OscillationAlarmLevel {
                    frequency_hz: current.frequency_hz,
                    damping_ratio: current.damping_ratio,
                    severity,
                    trend,
                });
            }
        }
        trend_alarms
    }
    /// Decompose `signal` into `n_modes` complex exponential components.
    ///
    /// # Steps
    ///
    /// 1. Build Hankel data matrix from the signal.
    /// 2. Solve the linear prediction equations via least squares
    ///    (normal equations / Cholesky on small systems).
    /// 3. Build companion matrix and compute its eigenvalues (QR iterations).
    /// 4. Convert discrete-time poles to continuous-time (σ, ω) via log.
    /// 5. Solve Vandermonde system for amplitudes via back-substitution.
    fn prony_analysis(&self, signal: &[f64], dt_s: f64, n_modes: usize) -> Vec<OscillationMode> {
        let n = signal.len();
        if n < 4 || n_modes == 0 {
            return Vec::new();
        }
        let p = 4_usize.min(n / 3).max(2);
        let rows = n - p;
        let mut hth = vec![0.0f64; p * p];
        let mut htb = vec![0.0f64; p];
        let b_vec: Vec<f64> = (p..n).map(|k| signal[k]).collect();
        for k in 0..rows {
            for i in 0..p {
                let xi = signal[k + p - 1 - i];
                htb[i] += xi * b_vec[k];
                for j in 0..p {
                    let xj = signal[k + p - 1 - j];
                    hth[i * p + j] += xi * xj;
                }
            }
        }
        let max_diag = (0..p)
            .map(|i| hth[i * p + i].abs())
            .fold(1e-20_f64, f64::max);
        let lambda = max_diag * 1e-9;
        for i in 0..p {
            hth[i * p + i] += lambda;
        }
        let a_coeffs = match solve_linear_system(&hth, &htb, p) {
            Some(v) => v,
            None => return Vec::new(),
        };
        let eigenvalues = companion_eigenvalues(&a_coeffs, p);
        let total_energy: f64 = signal.iter().map(|&x| x * x).sum();
        let mut modes: Vec<OscillationMode> = Vec::new();
        for (zi_re, zi_im) in &eigenvalues {
            let zi_mag = (zi_re * zi_re + zi_im * zi_im).sqrt();
            if !(1e-10..=10.0).contains(&zi_mag) {
                continue;
            }
            let sigma = zi_mag.ln() / dt_s;
            let omega = zi_im.atan2(*zi_re) / dt_s;
            let freq_hz = omega.abs() / (2.0 * std::f64::consts::PI);
            if freq_hz < 1e-4 {
                continue;
            }
            let s_mag = (sigma * sigma + omega * omega).sqrt();
            let damping_ratio = if s_mag > 1e-10 { -sigma / s_mag } else { 1.0 };
            let amplitude = estimate_amplitude(signal, sigma, omega, dt_s);
            let mode_energy = if total_energy > 1e-30 {
                (amplitude * amplitude * (n as f64)) / total_energy
            } else {
                0.0
            };
            let mode_type = Self::classify_mode(freq_hz);
            modes.push(OscillationMode {
                frequency_hz: freq_hz,
                damping_ratio,
                amplitude,
                energy: mode_energy.min(1.0),
                mode_type,
                participating_signals: Vec::new(),
            });
        }
        modes
    }
    /// Classify a mode by its frequency into a [`ModeType`].
    ///
    /// | Range \[Hz\]   | Type            |
    /// |---------------|-----------------|
    /// | < 0.1         | GlobalMode      |
    /// | 0.1 – 0.8     | InterAreaMode   |
    /// | 0.8 – 2.0     | LocalAreaMode   |
    /// | 2.0 – 3.0     | LocalPlantMode  |
    /// | ≥ 3.0         | TorsionalMode   |
    pub fn classify_mode(freq_hz: f64) -> ModeType {
        if freq_hz < 0.1 {
            ModeType::GlobalMode
        } else if freq_hz < 0.8 {
            ModeType::InterAreaMode
        } else if freq_hz < 2.0 {
            ModeType::LocalAreaMode
        } else if freq_hz < 3.0 {
            ModeType::LocalPlantMode
        } else {
            ModeType::TorsionalMode
        }
    }
    fn generate_alarms(&self, modes: &[OscillationMode]) -> Vec<OscillationAlarmLevel> {
        let mut alarms = Vec::new();
        for mode in modes {
            let severity = if mode.damping_ratio < 0.0 {
                AlarmLevel::Emergency
            } else if mode.damping_ratio < self.config.alarm_damping_threshold {
                if mode.amplitude > self.config.alarm_amplitude_threshold {
                    AlarmLevel::Alert
                } else {
                    AlarmLevel::Advisory
                }
            } else {
                AlarmLevel::Normal
            };
            if severity != AlarmLevel::Normal {
                alarms.push(OscillationAlarmLevel {
                    frequency_hz: mode.frequency_hz,
                    damping_ratio: mode.damping_ratio,
                    severity,
                    trend: DampingTrend::Stable,
                });
            }
        }
        alarms
    }
    fn detect_damping_trend(&self, target_freq_hz: f64) -> DampingTrend {
        let tol = 0.2;
        let relevant: Vec<f64> = self
            .history
            .iter()
            .filter(|m| (m.frequency_hz - target_freq_hz).abs() < tol)
            .map(|m| m.damping_ratio)
            .collect();
        if relevant.len() < 2 {
            return DampingTrend::Stable;
        }
        let first_half_avg =
            relevant[..relevant.len() / 2].iter().sum::<f64>() / (relevant.len() / 2) as f64;
        let second_half_avg = relevant[relevant.len() / 2..].iter().sum::<f64>()
            / (relevant.len() - relevant.len() / 2) as f64;
        let delta = second_half_avg - first_half_avg;
        if delta > 0.005 {
            DampingTrend::Improving
        } else if delta < -0.005 {
            DampingTrend::Deteriorating
        } else {
            DampingTrend::Stable
        }
    }
}
/// Result of DFT-based spectrum analysis.
pub struct SpectrumResult {
    /// Frequency axis values \[Hz\] per DFT bin.
    pub frequencies: Vec<f64>,
    /// Normalised magnitude per bin.
    pub magnitudes: Vec<f64>,
    /// Frequency of the bin with maximum magnitude \[Hz\].
    pub dominant_frequency_hz: f64,
    /// Maximum magnitude value.
    pub dominant_magnitude: f64,
    /// Total power in the inter-area band 0.1–0.8 Hz.
    pub interarea_power: f64,
    /// Total power in the local-mode band 0.8–2.5 Hz.
    pub local_power: f64,
}
/// PSS lead-lag tuning recommendation for a specific bus.
#[derive(Debug, Clone)]
pub struct PssTuningRecommendation {
    /// Bus index.
    pub bus_id: usize,
    /// Target mode frequency for PSS tuning \[Hz\].
    pub target_frequency_hz: f64,
    /// Suggested PSS gain.
    pub suggested_gain: f64,
    /// Suggested washout time constant \[s\].
    pub suggested_washout_s: f64,
    /// Expected improvement in damping ratio.
    pub expected_damping_improvement: f64,
}
/// An individual oscillation alert raised by the WAM monitor.
#[derive(Debug, Clone)]
pub struct OscillationAlert {
    /// Alert severity.
    pub severity: AlertSeverity,
    /// Frequency of the alarming mode \[Hz\].
    pub mode_frequency_hz: f64,
    /// Current damping ratio.
    pub damping_ratio: f64,
    /// Human-readable description.
    pub description: String,
}
/// Fourier-based oscillation detector for real-time power system monitoring.
pub struct FourierOscillationDetector {
    /// Measurement sampling rate \[Hz\].
    pub sampling_rate_hz: f64,
    /// FFT window size in samples (power of 2 preferred for efficiency).
    pub fft_window_size: usize,
    /// Frequency resolution = sampling_rate / window_size \[Hz\].
    pub freq_resolution_hz: f64,
    /// Damping ratio threshold for "poorly damped" classification (default 0.05).
    pub poorly_damped_threshold: f64,
}
impl FourierOscillationDetector {
    /// Create a new Fourier oscillation detector.
    pub fn new(sampling_rate_hz: f64, fft_window_size: usize) -> Self {
        let freq_resolution_hz = if fft_window_size > 0 {
            sampling_rate_hz / fft_window_size as f64
        } else {
            sampling_rate_hz
        };
        Self {
            sampling_rate_hz,
            fft_window_size,
            freq_resolution_hz,
            poorly_damped_threshold: 0.05,
        }
    }
    /// Compute DFT-based spectrum for the given signal.
    ///
    /// Uses direct DFT (no FFT dependency): X\[k\] = Σ x\[n\] · exp(−j2π·k·n/N)
    /// Only the positive-frequency bins up to Nyquist are returned.
    pub fn analyze_spectrum(&self, signal: &[f64]) -> SpectrumResult {
        let n = signal.len().min(self.fft_window_size).max(1);
        let half = n / 2 + 1;
        let mut frequencies = Vec::with_capacity(half);
        let mut magnitudes = Vec::with_capacity(half);
        for k in 0..half {
            let freq_hz = k as f64 * self.sampling_rate_hz / n as f64;
            let mut re = 0.0f64;
            let mut im = 0.0f64;
            for (idx, &xn) in signal.iter().take(n).enumerate() {
                let angle = -2.0 * PI * k as f64 * idx as f64 / n as f64;
                re += xn * angle.cos();
                im += xn * angle.sin();
            }
            let mag = (re * re + im * im).sqrt() / n as f64;
            frequencies.push(freq_hz);
            magnitudes.push(mag);
        }
        let (dom_idx, &dom_mag) = magnitudes
            .iter()
            .enumerate()
            .skip(1)
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
            .unwrap_or((0, &0.0));
        let dominant_frequency_hz = frequencies.get(dom_idx).copied().unwrap_or(0.0);
        let interarea_power: f64 = frequencies
            .iter()
            .zip(magnitudes.iter())
            .filter(|(&f, _)| (0.1..=0.8).contains(&f))
            .map(|(_, &m)| m * m)
            .sum();
        let local_power: f64 = frequencies
            .iter()
            .zip(magnitudes.iter())
            .filter(|(&f, _)| f > 0.8 && f <= 2.5)
            .map(|(_, &m)| m * m)
            .sum();
        SpectrumResult {
            frequencies,
            magnitudes,
            dominant_frequency_hz,
            dominant_magnitude: dom_mag,
            interarea_power,
            local_power,
        }
    }
    /// Detect if signals contain a forced oscillation (narrow spectral peak).
    ///
    /// A forced oscillation is characterised by a single frequency where the
    /// spectral peak is >3 dB above its immediate neighbours in all channels.
    /// Returns the peak frequency \[Hz\] if a forced oscillation is detected.
    pub fn detect_forced_oscillation(&self, signals: &[Vec<f64>]) -> Option<f64> {
        if signals.is_empty() {
            return None;
        }
        let n = self
            .fft_window_size
            .min(signals.iter().map(|s| s.len()).min().unwrap_or(1))
            .max(4);
        let half = n / 2 + 1;
        let mut avg_mag = vec![0.0f64; half];
        for signal in signals {
            for (k, avg_val) in avg_mag.iter_mut().enumerate().take(half) {
                let mut re = 0.0f64;
                let mut im = 0.0f64;
                for (idx, &xn) in signal.iter().take(n).enumerate() {
                    let angle = -2.0 * PI * k as f64 * idx as f64 / n as f64;
                    re += xn * angle.cos();
                    im += xn * angle.sin();
                }
                *avg_val += (re * re + im * im).sqrt() / n as f64;
            }
        }
        let n_ch = signals.len() as f64;
        for m in &mut avg_mag {
            *m /= n_ch;
        }
        let threshold_db = 3.0_f64;
        let factor = 10.0_f64.powf(threshold_db / 20.0);
        for k in 1..half.saturating_sub(1) {
            let peak = avg_mag[k];
            let left = avg_mag[k - 1];
            let right = avg_mag[k + 1];
            if peak > factor * left && peak > factor * right && peak > 1e-12 {
                let freq_hz = k as f64 * self.sampling_rate_hz / n as f64;
                return Some(freq_hz);
            }
        }
        None
    }
}
/// Alert severity for oscillation events.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AlertSeverity {
    /// Informational — no immediate action needed.
    Info,
    /// Warning — operator should monitor closely.
    Warning,
    /// Critical — immediate operator action required.
    Critical,
}
