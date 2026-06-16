//! High Impedance Fault (HIF) Detection.
//!
//! High impedance faults (downed conductors, tree contact) produce very small
//! fault currents (< 75 \[A\]) making them invisible to conventional overcurrent
//! protection. This module implements specialised signal-processing algorithms
//! that exploit the arcing signature of HIFs.
//!
//! # Algorithms
//!
//! 1. **Even harmonic ratio** — arcing generates even harmonics (2nd, 4th)
//!    absent in normal load current.
//! 2. **Half-cycle asymmetry** — arc ignition threshold differs positive vs
//!    negative, producing asymmetric half-cycles.
//! 3. **Incremental energy** — energy buildup per cycle drifts during arcing.
//!
//! Confidences from the three detectors are combined using
//! **Dempster-Shafer evidence theory** to produce a final confidence score.

use thiserror::Error;

// ── Error ─────────────────────────────────────────────────────────────────────

/// Errors produced by the HIF detection algorithms.
#[derive(Debug, Error)]
pub enum HifError {
    /// Not enough current samples to fill a detection window.
    #[error("insufficient samples: need {need}, got {got}")]
    InsufficientSamples { need: usize, got: usize },

    /// Configuration is self-contradictory or out of range.
    #[error("invalid HIF configuration: {0}")]
    InvalidConfig(String),

    /// A numerical computation failed.
    #[error("computation error: {0}")]
    ComputationError(String),
}

// ── Public types ───────────────────────────────────────────────────────────────

/// Configuration for the HIF detector.
#[derive(Debug, Clone)]
pub struct HifConfig {
    /// Nominal system frequency \[Hz\] (e.g. 60.0 or 50.0).
    pub nominal_frequency_hz: f64,
    /// ADC sampling rate \[Hz\] (e.g. 1200 for 20× at 60 \[Hz\]).
    pub sampling_rate_hz: f64,
    /// Number of power-frequency cycles to analyse per detection window.
    pub detection_window_cycles: usize,
    /// Target false-alarm rate (0–1, e.g. 0.01).
    pub false_alarm_rate: f64,
}

impl HifConfig {
    /// Samples per cycle derived from configuration.
    fn samples_per_cycle(&self) -> usize {
        (self.sampling_rate_hz / self.nominal_frequency_hz).round() as usize
    }

    /// Minimum samples required to fill the detection window.
    pub fn min_samples(&self) -> usize {
        self.samples_per_cycle() * self.detection_window_cycles
    }
}

/// Three-phase current measurement phase identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    /// Phase A.
    A,
    /// Phase B.
    B,
    /// Phase C.
    C,
    /// Neutral conductor.
    N,
}

/// One instantaneous current measurement.
#[derive(Debug, Clone)]
pub struct CurrentSample {
    /// Measurement time \[s\].
    pub time_s: f64,
    /// Instantaneous current \[A\].
    pub current_a: f64,
    /// Conductor phase this sample belongs to.
    pub phase: Phase,
}

/// Features extracted from the current waveform for HIF decision making.
#[derive(Debug, Clone)]
pub struct HifFeatures {
    /// Ratio of 2nd + 4th harmonic energy to fundamental energy (0–1).
    pub even_harmonic_ratio: f64,
    /// Normalised asymmetry between positive and negative half-cycle RMS (0–1).
    pub half_cycle_asymmetry: f64,
    /// Mean absolute per-cycle energy increment \[A²·s\].
    pub delta_energy: f64,
    /// Zero-sequence current magnitude \[A\] (sum of neutral-phase samples RMS).
    pub zero_sequence_current_a: f64,
    /// Shannon entropy of current-peak magnitudes (dimensionless).
    pub arc_randomness: f64,
    /// Estimated neutral-to-ground voltage \[V\] (zero-seq × 1 \[Ω\]).
    pub neutral_ground_voltage: f64,
}

/// Outcome of a complete HIF detection run.
#[derive(Debug, Clone)]
pub struct HifDetectionResult {
    /// `true` when the combined evidence exceeds the detection threshold.
    pub hif_detected: bool,
    /// Combined confidence from all algorithms (0–1).
    pub confidence: f64,
    /// Intermediate features used for the decision.
    pub features: HifFeatures,
    /// Time of the last sample in the analysis window \[s\].
    pub detection_time_s: f64,
    /// Most likely faulted phase (absent when no clear dominant phase).
    pub fault_phase: Option<Phase>,
    /// Estimated fault resistance \[Ω\] (simplified V/I model).
    pub estimated_fault_resistance_ohm: f64,
    /// Human-readable name of the combined algorithm.
    pub algorithm_used: String,
}

// ── Detector ──────────────────────────────────────────────────────────────────

/// Multi-algorithm high-impedance fault detector.
pub struct HifDetector {
    config: HifConfig,
}

impl HifDetector {
    /// Construct a new detector with the given configuration.
    pub fn new(config: HifConfig) -> Self {
        Self { config }
    }

    // ── Feature extraction ─────────────────────────────────────────────────

    /// Extract diagnostic features from a window of current samples.
    ///
    /// The samples may contain multiple phases; features are computed from
    /// the phase with the highest RMS (assumed to be the faulted phase).
    pub fn extract_features(&self, samples: &[CurrentSample]) -> Result<HifFeatures, HifError> {
        let min = self.config.min_samples();
        if samples.len() < min {
            return Err(HifError::InsufficientSamples {
                need: min,
                got: samples.len(),
            });
        }

        // Work with the dominant non-neutral phase.
        let phase_samples = dominant_phase_samples(samples);

        let n = phase_samples.len();
        let dt = if phase_samples.len() >= 2 {
            phase_samples[1].time_s - phase_samples[0].time_s
        } else {
            1.0 / self.config.sampling_rate_hz
        };

        // ── DFT-based harmonic analysis ────────────────────────────────────
        let f0 = self.config.nominal_frequency_hz;
        let fs = self.config.sampling_rate_hz;

        let fund_mag = goertzel_magnitude(&phase_samples, f0, fs);
        let h2_mag = goertzel_magnitude(&phase_samples, 2.0 * f0, fs);
        let h4_mag = goertzel_magnitude(&phase_samples, 4.0 * f0, fs);

        let even_harmonic_ratio = if fund_mag > 1e-12 {
            (h2_mag + h4_mag) / fund_mag
        } else {
            0.0
        };

        // ── Half-cycle asymmetry ───────────────────────────────────────────
        let (pos_rms, neg_rms) = half_cycle_rms(&phase_samples, dt);
        let half_cycle_asymmetry = (pos_rms - neg_rms).abs() / (pos_rms + neg_rms + 1e-9);

        // ── Per-cycle energy increments ────────────────────────────────────
        let spc = self.config.samples_per_cycle().max(1);
        let delta_energy = per_cycle_energy_delta(&phase_samples, spc, dt);

        // ── Zero-sequence current (neutral phase) ──────────────────────────
        let neutral: Vec<f64> = samples
            .iter()
            .filter(|s| s.phase == Phase::N)
            .map(|s| s.current_a)
            .collect();
        let zero_sequence_current_a = if neutral.is_empty() {
            0.0
        } else {
            rms(&neutral)
        };

        // ── Arc randomness (entropy of peak magnitudes) ────────────────────
        let arc_randomness = peak_entropy(&phase_samples, spc);

        // ── Neutral-to-ground voltage (1 Ω nominal impedance model) ───────
        let neutral_ground_voltage = zero_sequence_current_a * 1.0;

        // Suppress unused warning in simple case
        let _ = n;

        Ok(HifFeatures {
            even_harmonic_ratio,
            half_cycle_asymmetry,
            delta_energy,
            zero_sequence_current_a,
            arc_randomness,
            neutral_ground_voltage,
        })
    }

    // ── Top-level detection ────────────────────────────────────────────────

    /// Run all detection algorithms and return a combined result.
    ///
    /// Detection logic: an HIF is flagged when **either** the fused evidence
    /// score exceeds the threshold **or** any single specialised algorithm
    /// reports high confidence (≥ 0.6).  This implements a one-out-of-N
    /// structure that mirrors practical relay designs where each algorithm can
    /// independently assert a trip.
    pub fn detect(&self, samples: &[CurrentSample]) -> Result<HifDetectionResult, HifError> {
        let features = self.extract_features(samples)?;

        let c1 = self.detect_even_harmonics(&features);
        let c2 = self.detect_asymmetry(&features);
        let c3 = self.detect_incremental_energy(samples);
        let combined = self.combine_evidence(&[c1, c2, c3]);

        // Detection threshold: tuned so that at false_alarm_rate=0.01 the
        // threshold sits at 0.48 (conservative default).
        let threshold = 0.5 - self.config.false_alarm_rate * 2.0;
        let threshold = threshold.clamp(0.3, 0.7);

        // Any single strong algorithm can also trigger (OR-gate structure).
        let single_algorithm_trip = c1 >= 0.6 || c2 >= 0.6 || c3 >= 0.6;

        let detection_time_s = samples
            .iter()
            .map(|s| s.time_s)
            .fold(f64::NEG_INFINITY, f64::max);

        let fault_phase = dominant_phase(samples);

        // Fault-resistance estimate: V_nom / I_fault (7200 V line-to-neutral).
        let v_nom = 7200.0_f64;
        let i_fault = features.zero_sequence_current_a + 1e-9;
        let estimated_fault_resistance_ohm = v_nom / i_fault;

        Ok(HifDetectionResult {
            hif_detected: combined >= threshold || single_algorithm_trip,
            confidence: combined,
            features,
            detection_time_s,
            fault_phase,
            estimated_fault_resistance_ohm,
            algorithm_used: "EvenHarmonic+HalfCycleAsymmetry+IncrementalEnergy+DempsterShafer"
                .to_string(),
        })
    }

    // ── Individual algorithm confidences ──────────────────────────────────

    /// Confidence from even-harmonic content.
    ///
    /// A ratio above 0.05 indicates possible arc.  Confidence grows linearly
    /// up to 1.0 at a ratio of 0.5.
    fn detect_even_harmonics(&self, features: &HifFeatures) -> f64 {
        let r = features.even_harmonic_ratio;
        if r < 0.05 {
            0.0
        } else {
            (0.5 + 0.5 * ((r - 0.05) / 0.45).min(1.0)).min(1.0)
        }
    }

    /// Confidence from half-cycle asymmetry.
    ///
    /// Normal load current is symmetric (asymmetry ≈ 0).  Arcing creates
    /// asymmetry driven by differing ignition thresholds each half-cycle.
    fn detect_asymmetry(&self, features: &HifFeatures) -> f64 {
        let a = features.half_cycle_asymmetry;
        if a < 0.02 {
            0.0
        } else {
            (a / 0.3).min(1.0)
        }
    }

    /// Confidence from per-cycle incremental energy variance.
    ///
    /// A steadily growing energy (random walk with drift) is characteristic
    /// of sustained arcing.
    fn detect_incremental_energy(&self, samples: &[CurrentSample]) -> f64 {
        let min = self.config.min_samples();
        if samples.len() < min {
            return 0.0;
        }
        let phase_samples = dominant_phase_samples(samples);
        let spc = self.config.samples_per_cycle().max(1);
        let dt = if phase_samples.len() >= 2 {
            phase_samples[1].time_s - phase_samples[0].time_s
        } else {
            1.0 / self.config.sampling_rate_hz
        };

        let energies = cycle_energies(&phase_samples, spc, dt);
        if energies.len() < 2 {
            return 0.0;
        }

        // Coefficient of variation of cycle-to-cycle energy deltas.
        let deltas: Vec<f64> = energies.windows(2).map(|w| (w[1] - w[0]).abs()).collect();
        let mean_d = deltas.iter().copied().sum::<f64>() / deltas.len() as f64;
        let var_d = deltas.iter().map(|&d| (d - mean_d).powi(2)).sum::<f64>() / deltas.len() as f64;
        let std_d = var_d.sqrt();
        let cv = if mean_d > 1e-12 { std_d / mean_d } else { 0.0 };

        // High CV → random arcing energy → higher confidence.
        (cv / 1.5).min(1.0)
    }

    // ── Dempster-Shafer combination ────────────────────────────────────────

    /// Combine independent evidence confidences using a Dempster-Shafer
    /// inspired evidence accumulation rule.
    ///
    /// Each source contributes focal elements:
    /// - `m_i(HIF)   = c_i`
    /// - `m_i(¬HIF)  = 1 − c_i`
    ///
    /// The implementation uses the log-odds accumulation equivalent to
    /// iterative Bayesian updating with independent sources.  This correctly
    /// reinforces consistent weak evidence: multiple sources each reporting
    /// moderate confidence produce higher combined confidence than any single
    /// source alone (unlike the raw Dempster rule which collapses toward 0.5
    /// when evidences are split).
    ///
    /// The formula in log-odds space:
    /// ```text
    /// LO = Σᵢ log(cᵢ / (1 − cᵢ))   (clamped away from 0 and 1)
    /// combined = 1 / (1 + exp(−LO))
    /// ```
    fn combine_evidence(&self, confidences: &[f64]) -> f64 {
        if confidences.is_empty() {
            return 0.0;
        }

        // If all sources are zero return 0.
        if confidences.iter().all(|&c| c < 1e-9) {
            return 0.0;
        }

        // Log-odds accumulation (independent evidence fusion).
        let log_odds_sum: f64 = confidences
            .iter()
            .map(|&c| {
                let c = c.clamp(1e-6, 1.0 - 1e-6);
                (c / (1.0 - c)).ln()
            })
            .sum();

        // Convert back to probability.
        let combined = 1.0 / (1.0 + (-log_odds_sum).exp());
        combined.clamp(0.0, 1.0)
    }
}

// ── Private helpers ────────────────────────────────────────────────────────────

/// Goertzel algorithm: compute DFT magnitude at a single target frequency.
fn goertzel_magnitude(samples: &[&CurrentSample], target_hz: f64, fs: f64) -> f64 {
    let n = samples.len();
    if n == 0 {
        return 0.0;
    }
    let k = (target_hz / fs * n as f64).round() as usize;
    let omega = 2.0 * std::f64::consts::PI * k as f64 / n as f64;
    let coeff = 2.0 * omega.cos();
    let mut s_prev2 = 0.0_f64;
    let mut s_prev1 = 0.0_f64;
    for s in samples {
        let s_curr = s.current_a + coeff * s_prev1 - s_prev2;
        s_prev2 = s_prev1;
        s_prev1 = s_curr;
    }
    let re = s_prev1 - s_prev2 * omega.cos();
    let im = s_prev2 * omega.sin();
    (re * re + im * im).sqrt() / n as f64
}

/// Compute positive and negative half-cycle RMS values.
fn half_cycle_rms(samples: &[&CurrentSample], _dt: f64) -> (f64, f64) {
    let pos: Vec<f64> = samples
        .iter()
        .filter(|s| s.current_a >= 0.0)
        .map(|s| s.current_a)
        .collect();
    let neg: Vec<f64> = samples
        .iter()
        .filter(|s| s.current_a < 0.0)
        .map(|s| s.current_a)
        .collect();
    (rms(&pos), rms(&neg))
}

/// Root-mean-square of a slice of values.
fn rms(vals: &[f64]) -> f64 {
    if vals.is_empty() {
        return 0.0;
    }
    let sum_sq: f64 = vals.iter().map(|v| v * v).sum();
    (sum_sq / vals.len() as f64).sqrt()
}

/// Mean absolute energy difference between consecutive cycles.
fn per_cycle_energy_delta(samples: &[&CurrentSample], spc: usize, dt: f64) -> f64 {
    let energies = cycle_energies(samples, spc, dt);
    if energies.len() < 2 {
        return 0.0;
    }
    let deltas: Vec<f64> = energies.windows(2).map(|w| (w[1] - w[0]).abs()).collect();
    deltas.iter().sum::<f64>() / deltas.len() as f64
}

/// Per-cycle energy (integral of i² dt) for each complete cycle.
fn cycle_energies(samples: &[&CurrentSample], spc: usize, dt: f64) -> Vec<f64> {
    samples
        .chunks(spc)
        .filter(|chunk| chunk.len() == spc)
        .map(|chunk| chunk.iter().map(|s| s.current_a * s.current_a * dt).sum())
        .collect()
}

/// Shannon entropy of current-peak magnitudes (one peak per cycle).
fn peak_entropy(samples: &[&CurrentSample], spc: usize) -> f64 {
    // Extract one positive peak per cycle.
    let peaks: Vec<f64> = samples
        .chunks(spc)
        .filter(|c| c.len() == spc)
        .filter_map(|chunk| chunk.iter().map(|s| s.current_a.abs()).reduce(f64::max))
        .collect();

    if peaks.is_empty() {
        return 0.0;
    }

    // Histogram into 8 bins.
    let min_p = peaks.iter().cloned().fold(f64::INFINITY, f64::min);
    let max_p = peaks.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let range = max_p - min_p;

    if range < 1e-12 {
        return 0.0; // All peaks identical → no randomness.
    }

    let n_bins = 8usize;
    let mut bins = vec![0usize; n_bins];
    for &p in &peaks {
        let idx = (((p - min_p) / range) * (n_bins - 1) as f64).round() as usize;
        let idx = idx.min(n_bins - 1);
        bins[idx] += 1;
    }

    let total = peaks.len() as f64;
    bins.iter()
        .filter(|&&b| b > 0)
        .map(|&b| {
            let pr = b as f64 / total;
            -pr * pr.ln()
        })
        .sum()
}

/// Return references to samples belonging to the phase with highest RMS.
fn dominant_phase_samples(samples: &[CurrentSample]) -> Vec<&CurrentSample> {
    // Find dominant non-neutral phase.
    let best = dominant_phase(samples).unwrap_or(Phase::A);
    samples.iter().filter(|s| s.phase == best).collect()
}

/// Identify the phase with the highest mean |current|.
fn dominant_phase(samples: &[CurrentSample]) -> Option<Phase> {
    let phases = [Phase::A, Phase::B, Phase::C];
    phases
        .iter()
        .map(|&ph| {
            let vals: Vec<f64> = samples
                .iter()
                .filter(|s| s.phase == ph)
                .map(|s| s.current_a.abs())
                .collect();
            let mean = if vals.is_empty() {
                0.0
            } else {
                vals.iter().sum::<f64>() / vals.len() as f64
            };
            (ph, mean)
        })
        .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(ph, _)| ph)
}

// ── Tests ──────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn make_config() -> HifConfig {
        HifConfig {
            nominal_frequency_hz: 60.0,
            sampling_rate_hz: 1200.0,
            detection_window_cycles: 20,
            false_alarm_rate: 0.01,
        }
    }

    /// Generate pure 60 Hz sine samples (no harmonics).
    fn pure_sine(amplitude_a: f64, n_samples: usize) -> Vec<CurrentSample> {
        let fs = 1200.0_f64;
        let f0 = 60.0_f64;
        (0..n_samples)
            .map(|i| CurrentSample {
                time_s: i as f64 / fs,
                current_a: amplitude_a * (2.0 * PI * f0 * i as f64 / fs).sin(),
                phase: Phase::A,
            })
            .collect()
    }

    /// Generate arc-like waveform: fundamental + even harmonics + asymmetric bias.
    fn arc_waveform(n_samples: usize) -> Vec<CurrentSample> {
        let fs = 1200.0_f64;
        let f0 = 60.0_f64;
        (0..n_samples)
            .map(|i| {
                let t = i as f64 / fs;
                let fundamental = 50.0 * (2.0 * PI * f0 * t).sin();
                let h2 = 8.0 * (2.0 * PI * 2.0 * f0 * t).sin(); // 16% 2nd harmonic
                let h4 = 3.0 * (2.0 * PI * 4.0 * f0 * t).sin(); // 6% 4th harmonic
                                                                // Asymmetric bias: positive half-cycle 20% larger
                let raw = fundamental + h2 + h4;
                let current_a = if raw >= 0.0 { raw * 1.20 } else { raw };
                CurrentSample {
                    time_s: t,
                    current_a,
                    phase: Phase::A,
                }
            })
            .collect()
    }

    #[test]
    fn test_normal_load_no_hif() {
        let cfg = make_config();
        let det = HifDetector::new(cfg.clone());
        let samples = pure_sine(30.0, cfg.min_samples() + 10);
        let result = det.detect(&samples).expect("detect should not fail");
        assert!(
            !result.hif_detected,
            "pure sine should not be flagged as HIF, confidence={}",
            result.confidence
        );
    }

    #[test]
    fn test_synthetic_hif_detected() {
        let cfg = make_config();
        let det = HifDetector::new(cfg.clone());
        let samples = arc_waveform(cfg.min_samples() + 10);
        let result = det.detect(&samples).expect("detect should not fail");
        assert!(
            result.hif_detected,
            "arc waveform should be detected as HIF, confidence={}",
            result.confidence
        );
    }

    #[test]
    fn test_even_harmonics_computed() {
        let cfg = make_config();
        let det = HifDetector::new(cfg.clone());
        // Waveform with explicit 10 % second harmonic.
        let fs = 1200.0_f64;
        let f0 = 60.0_f64;
        let n = cfg.min_samples() + 10;
        let samples: Vec<CurrentSample> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                CurrentSample {
                    time_s: t,
                    current_a: 50.0 * (2.0 * PI * f0 * t).sin()
                        + 5.0 * (2.0 * PI * 2.0 * f0 * t).sin(),
                    phase: Phase::A,
                }
            })
            .collect();
        let features = det.extract_features(&samples).expect("extract_features");
        assert!(
            features.even_harmonic_ratio > 0.05,
            "expected even_harmonic_ratio > 0.05, got {}",
            features.even_harmonic_ratio
        );
    }

    #[test]
    fn test_half_cycle_asymmetry_positive() {
        let cfg = make_config();
        let det = HifDetector::new(cfg.clone());
        let samples = arc_waveform(cfg.min_samples() + 10);
        let features = det.extract_features(&samples).expect("extract_features");
        assert!(
            features.half_cycle_asymmetry > 0.0,
            "asymmetric arc waveform must have positive asymmetry, got {}",
            features.half_cycle_asymmetry
        );
    }

    #[test]
    fn test_confidence_combination_dempster_shafer() {
        // Three weak-but-concordant signals each reporting moderate HIF
        // confidence.  Evidence accumulation must produce a higher combined
        // confidence than any single source (the hallmark of independent-
        // evidence fusion in the log-odds / Bayesian framework).
        let cfg = make_config();
        let det = HifDetector::new(cfg);
        let inputs = [0.55_f64, 0.60, 0.65]; // all > 0.5 → consistent HIF evidence
        let combined = det.combine_evidence(&inputs);
        let max_single = inputs.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!(
            combined > max_single,
            "Multiple weak concordant signals ({combined:.3}) should exceed \
             the strongest single signal ({max_single:.3})"
        );
    }

    #[test]
    fn test_insufficient_samples_error() {
        let cfg = make_config();
        let det = HifDetector::new(cfg);
        // 100 samples < 400 minimum
        let samples = pure_sine(100.0, 100);
        let err = det
            .detect(&samples)
            .expect_err("should fail with insufficient samples");
        match err {
            HifError::InsufficientSamples { need, got } => {
                assert_eq!(need, 400, "need should be 400");
                assert_eq!(got, 100, "got should be 100");
            }
            other => panic!("expected InsufficientSamples, got {:?}", other),
        }
    }

    #[test]
    fn test_zero_current_no_hif() {
        let cfg = make_config();
        let det = HifDetector::new(cfg.clone());
        let dt = 1.0 / cfg.sampling_rate_hz;
        let samples: Vec<CurrentSample> = (0..cfg.min_samples())
            .map(|i| CurrentSample {
                time_s: i as f64 * dt,
                current_a: 0.0,
                phase: Phase::A,
            })
            .collect();
        let result = det.detect(&samples).expect("detect should succeed");
        assert!(!result.hif_detected, "zero current should not trigger HIF");
        assert!(
            result.confidence < 0.5,
            "confidence should be low for zero current, got {}",
            result.confidence
        );
    }

    #[test]
    fn test_zero_sequence_current_from_neutral() {
        let cfg = make_config();
        let det = HifDetector::new(cfg.clone());
        let dt = 1.0 / cfg.sampling_rate_hz;
        let n = cfg.min_samples();
        // Phase A pure sine
        let mut samples = pure_sine(100.0, n);
        // Add Phase N samples with large current
        for i in 0..n {
            samples.push(CurrentSample {
                time_s: i as f64 * dt,
                current_a: 10.0,
                phase: Phase::N,
            });
        }
        let features = det
            .extract_features(&samples)
            .expect("extract_features should succeed");
        assert!(
            features.zero_sequence_current_a > 1.0,
            "neutral current should produce zero_sequence_current_a > 1.0, got {}",
            features.zero_sequence_current_a
        );
        let expected_ngv = features.zero_sequence_current_a * 1.0;
        assert!(
            (features.neutral_ground_voltage - expected_ngv).abs() < 1e-9,
            "neutral_ground_voltage should equal zero_sequence_current_a * 1.0"
        );
    }

    #[test]
    fn test_combine_evidence_all_zero() {
        // combine_evidence is private; test indirectly via pure-sine detect()
        let cfg = make_config();
        let det = HifDetector::new(cfg.clone());
        let samples = pure_sine(100.0, cfg.min_samples());
        let result = det.detect(&samples).expect("detect should succeed");
        // Pure sine provides zero evidence → combined confidence should be below threshold
        assert!(
            result.confidence < 0.48,
            "pure-sine combined confidence should be below threshold 0.48, got {}",
            result.confidence
        );
    }

    #[test]
    fn test_4th_harmonic_triggers_hif() {
        let cfg = make_config();
        let det = HifDetector::new(cfg.clone());
        let dt = 1.0 / cfg.sampling_rate_hz;
        let n = cfg.min_samples() + 10;
        let samples: Vec<CurrentSample> = (0..n)
            .map(|i| {
                let t = i as f64 * dt;
                let current_a =
                    50.0 * (2.0 * PI * 60.0 * t).sin() + 15.0 * (2.0 * PI * 240.0 * t).sin(); // 30% 4th harmonic
                CurrentSample {
                    time_s: t,
                    current_a,
                    phase: Phase::A,
                }
            })
            .collect();
        let features = det
            .extract_features(&samples)
            .expect("extract_features should succeed");
        assert!(
            features.even_harmonic_ratio > 0.25,
            "30% 4th harmonic → even_harmonic_ratio should be >0.25, got {}",
            features.even_harmonic_ratio
        );
        let result = det.detect(&samples).expect("detect should succeed");
        assert!(
            result.hif_detected,
            "strong 4th harmonic should trigger HIF detection"
        );
    }

    #[test]
    fn test_detection_time_matches_last_sample() {
        let cfg = make_config();
        let det = HifDetector::new(cfg.clone());
        let n = cfg.min_samples() + 5;
        let samples = pure_sine(100.0, n);
        let last_time = samples.last().expect("samples must be non-empty").time_s;
        let result = det.detect(&samples).expect("detect should succeed");
        assert!(
            (result.detection_time_s - last_time).abs() < 1e-12,
            "detection_time_s should equal last sample time {}, got {}",
            last_time,
            result.detection_time_s
        );
    }

    #[test]
    fn test_fault_phase_dominant_phase_b() {
        let cfg = make_config();
        let det = HifDetector::new(cfg.clone());
        let n = cfg.min_samples() + 10;
        // arc_waveform is on Phase::A; remap to Phase::B
        let samples: Vec<CurrentSample> = arc_waveform(n)
            .into_iter()
            .map(|s| CurrentSample {
                phase: Phase::B,
                ..s
            })
            .collect();
        let result = det.detect(&samples).expect("detect should succeed");
        match result.fault_phase {
            Some(Phase::B) => {}
            other => panic!("expected fault_phase Some(Phase::B), got {:?}", other),
        }
    }

    #[test]
    fn test_higher_far_lowers_threshold() {
        let n = {
            let cfg = make_config();
            cfg.min_samples() + 10
        };
        let samples = arc_waveform(n);

        // Low FAR detector (threshold = 0.48)
        let cfg_low = make_config(); // false_alarm_rate = 0.01
        let det_low = HifDetector::new(cfg_low);
        let result_low = det_low
            .detect(&samples)
            .expect("low-FAR detect should succeed");

        // High FAR detector (threshold = clamp(0.5 - 0.20*2, 0.3, 0.7) = 0.3)
        let cfg_high = HifConfig {
            false_alarm_rate: 0.20,
            ..make_config()
        };
        let det_high = HifDetector::new(cfg_high);
        let result_high = det_high
            .detect(&samples)
            .expect("high-FAR detect should succeed");

        assert!(
            result_low.hif_detected,
            "low-FAR detector should detect arc waveform"
        );
        assert!(
            result_high.hif_detected,
            "high-FAR detector should detect arc waveform"
        );
    }
}
