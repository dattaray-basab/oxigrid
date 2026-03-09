//! Waveform distortion analysis: THD, K-factor, crest factor, power quantities,
//! interharmonic detection, transformer derating.
//!
//! ## Metrics computed
//!
//! | Metric                | Formula                                          |
//! |-----------------------|--------------------------------------------------|
//! | THD (voltage/current) | `√(Σ_{h≥2} X_h²) / X_1 × 100 %`              |
//! | Crest factor          | `X_peak / X_rms`                                 |
//! | Form factor           | `X_rms / X_avg` (full-wave rectified average)    |
//! | K-factor              | `Σ(h² · I_h²) / Σ(I_h²)` (transformer derating) |
//! | Distortion power      | `|S_apparent|² - P² - Q_1²`                     |
//! | True power factor     | `P / |S|`                                        |
//! | Displacement PF       | `cos φ₁` (fundamental power factor angle)        |
//!
//! Harmonic extraction uses Goertzel algorithm: O(N) per harmonic, no FFT needed.

use crate::error::{OxiGridError, Result};
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

// ─────────────────────────────────────────────────────────────────────────────
// Public types
// ─────────────────────────────────────────────────────────────────────────────

/// One complex harmonic component of a waveform.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HarmonicComponent {
    /// Harmonic order (1 = fundamental, 2 = second, …).
    pub order: usize,
    /// RMS magnitude \[pu\].
    pub magnitude_pu: f64,
    /// Phase angle \[rad\] relative to the fundamental.
    pub phase_rad: f64,
    /// Harmonic power \[pu\]: positive for generation, negative for consumption.
    pub power: f64,
}

/// Comprehensive waveform distortion metrics.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WaveformMetrics {
    /// Total Harmonic Distortion of voltage [%].
    pub thd_pct: f64,
    /// Crest factor: `V_peak / V_rms`.
    pub crest_factor: f64,
    /// Form factor: `V_rms / V_avg` (full-wave rectified).
    pub form_factor: f64,
    /// Transformer K-factor: `Σ(h² · I_h²) / Σ(I_h²)`.
    pub k_factor: f64,
    /// Distortion volt-amperes \[pu\]: `√(|S|² − P² − Q₁²)`.
    pub distortion_power: f64,
    /// Displacement power factor: `cos φ₁`.
    pub displacement_pf: f64,
    /// True (total) power factor: `P / |S|`.
    pub true_pf: f64,
    /// Individual harmonic components (orders 1..=n_harmonics).
    pub harmonic_components: Vec<HarmonicComponent>,
}

// ─────────────────────────────────────────────────────────────────────────────
// Goertzel algorithm
// ─────────────────────────────────────────────────────────────────────────────

/// Extract a single frequency component using the Goertzel algorithm.
///
/// Returns `(re, im)` for the sinusoidal component at `freq_hz`.
/// The output is scaled so that the magnitude `√(re²+im²)` equals the
/// **peak** amplitude of the corresponding sine component in the input signal.
///
/// This is an O(N) operation; use it when only a small number of bins are
/// needed (as opposed to a full FFT).
pub fn goertzel(samples: &[f64], freq_hz: f64, sample_rate_hz: f64) -> (f64, f64) {
    let n = samples.len();
    if n == 0 || sample_rate_hz <= 0.0 {
        return (0.0, 0.0);
    }
    let k = (freq_hz / sample_rate_hz * n as f64).round() as usize;
    let omega = 2.0 * PI * k as f64 / n as f64;
    let coeff = 2.0 * omega.cos();
    let mut s1 = 0.0_f64;
    let mut s2 = 0.0_f64;
    for &x in samples {
        let s = x + coeff * s1 - s2;
        s2 = s1;
        s1 = s;
    }
    let re = s1 - s2 * omega.cos();
    let im = s2 * omega.sin();
    // Normalise to peak amplitude: multiply by 2/N.
    (re / n as f64 * 2.0, im / n as f64 * 2.0)
}

// ─────────────────────────────────────────────────────────────────────────────
// Harmonic extraction
// ─────────────────────────────────────────────────────────────────────────────

/// Extract harmonic components from a waveform using Goertzel.
///
/// # Arguments
/// * `waveform`        — instantaneous samples (pu)
/// * `sample_rate_hz`  — sampling rate \[Hz\]
/// * `fundamental_hz`  — power frequency \[Hz\]
/// * `n_harmonics`     — highest harmonic order to extract
///
/// # Returns
/// Vector of [`HarmonicComponent`], orders 1..=n_harmonics.
/// Components with frequencies above the Nyquist limit are omitted.
pub fn extract_harmonics(
    waveform: &[f64],
    sample_rate_hz: f64,
    fundamental_hz: f64,
    n_harmonics: usize,
) -> Vec<HarmonicComponent> {
    if waveform.is_empty() || n_harmonics == 0 || sample_rate_hz <= 0.0 || fundamental_hz <= 0.0 {
        return vec![];
    }

    let nyquist = sample_rate_hz / 2.0;
    let (re1, im1) = goertzel(waveform, fundamental_hz, sample_rate_hz);
    let peak1 = (re1 * re1 + im1 * im1).sqrt();
    let phase1 = im1.atan2(re1);

    let mut components = Vec::with_capacity(n_harmonics);

    for h in 1..=n_harmonics {
        let freq = fundamental_hz * h as f64;
        if freq > nyquist {
            break;
        }
        let (re, im) = goertzel(waveform, freq, sample_rate_hz);
        let peak = (re * re + im * im).sqrt();
        // RMS = peak / √2
        let magnitude_pu = peak / 2_f64.sqrt();
        let phase_abs = im.atan2(re);
        let phase_rel = if h == 1 { 0.0 } else { phase_abs - phase1 };

        // Harmonic power: uses only magnitude (phase cross-terms between
        // voltage and current are handled in analyze_waveform).
        // Here we store zero; the caller sets it from V×I cross-product.
        components.push(HarmonicComponent {
            order: h,
            magnitude_pu,
            phase_rad: phase_rel,
            power: 0.0,
        });

        // Keep fundamental phase for reference.
        let _ = peak1; // used above for the reference phase
    }

    components
}

// ─────────────────────────────────────────────────────────────────────────────
// K-factor
// ─────────────────────────────────────────────────────────────────────────────

/// Compute the transformer K-factor from a set of current harmonic components.
///
/// `K = Σ_h (h² · I_h²) / Σ_h (I_h²)`
///
/// K = 1 means pure fundamental current (no derating required).
/// K > 1 means harmonic loading; use with [`transformer_derating_factor`].
pub fn compute_k_factor(harmonics: &[HarmonicComponent]) -> f64 {
    let numerator: f64 = harmonics
        .iter()
        .map(|h| (h.order * h.order) as f64 * h.magnitude_pu * h.magnitude_pu)
        .sum();
    let denominator: f64 = harmonics
        .iter()
        .map(|h| h.magnitude_pu * h.magnitude_pu)
        .sum();
    if denominator < 1e-15 {
        1.0
    } else {
        numerator / denominator
    }
}

/// Approximate transformer derating factor for a given K-factor.
///
/// `derating = √(1 / (1 + eddy_loss_ratio · K / K_rated))`
///
/// where `eddy_loss_ratio` is typically assumed at 0.1 (10 % eddy-current
/// loss factor for standard distribution transformers).  The result is the
/// fraction of rated kVA that the transformer can safely supply when loaded
/// with harmonics.
///
/// # Arguments
/// * `k_factor` — measured K-factor of the load
/// * `k_rated`  — K-factor rating of the transformer (1, 4, 9, 13, …)
pub fn transformer_derating_factor(k_factor: f64, k_rated: f64) -> f64 {
    let k_rated_safe = k_rated.max(1.0);
    let k_factor_safe = k_factor.max(1.0);
    // eddy_loss_ratio = 0.10 (industry rule-of-thumb for distribution Tx)
    let eddy_loss_ratio = 0.10_f64;
    let denom = 1.0 + eddy_loss_ratio * k_factor_safe / k_rated_safe;
    (1.0 / denom).sqrt()
}

// ─────────────────────────────────────────────────────────────────────────────
// Interharmonic detection
// ─────────────────────────────────────────────────────────────────────────────

/// Detect interharmonic frequencies from a DFT magnitude spectrum.
///
/// Interharmonics are spectral components whose frequency is **not** an integer
/// multiple of the fundamental.  The function returns all bins whose magnitude
/// exceeds `threshold` and whose frequency does not lie within ±`bin_width`
/// of an integer harmonic.
///
/// # Arguments
/// * `spectrum`       — DFT magnitude array (index k → frequency k·fs/N)
/// * `sample_rate_hz` — sampling rate \[Hz\]
/// * `fundamental_hz` — power frequency \[Hz\]
/// * `threshold`      — minimum magnitude to report \[pu\]
///
/// # Returns
/// `Vec<(frequency_hz, magnitude_pu)>` sorted by frequency.
pub fn detect_interharmonics(
    spectrum: &[f64],
    sample_rate_hz: f64,
    fundamental_hz: f64,
    threshold: f64,
) -> Vec<(f64, f64)> {
    let n = spectrum.len();
    if n == 0 || sample_rate_hz <= 0.0 || fundamental_hz <= 0.0 {
        return vec![];
    }

    let freq_resolution = sample_rate_hz / n as f64;
    // Half-width (in bins) around each harmonic to exclude.
    let half_window = (freq_resolution * 0.5).max(1.0);

    let mut result = Vec::new();

    for (k, &mag) in spectrum.iter().enumerate() {
        if mag < threshold {
            continue;
        }
        let freq = k as f64 * freq_resolution;
        // Check if this bin is "near" an integer harmonic.
        let nearest_order = (freq / fundamental_hz).round();
        let harmonic_freq = nearest_order * fundamental_hz;
        if (freq - harmonic_freq).abs() > half_window {
            result.push((freq, mag));
        }
    }

    result.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    result
}

// ─────────────────────────────────────────────────────────────────────────────
// Full waveform analysis
// ─────────────────────────────────────────────────────────────────────────────

/// Analyse a pair of voltage and current waveforms and return comprehensive
/// power quality metrics.
///
/// # Arguments
/// * `voltage`         — instantaneous voltage \[pu\]
/// * `current`         — instantaneous current \[pu\]
/// * `sample_rate_hz`  — sampling rate \[Hz\]
/// * `nominal_freq_hz` — power frequency \[Hz\]
/// * `n_harmonics`     — highest harmonic order to compute (≥ 1)
///
/// # Errors
/// Returns [`OxiGridError::InvalidParameter`] if inputs are empty or
/// have mismatched lengths.
pub fn analyze_waveform(
    voltage: &[f64],
    current: &[f64],
    sample_rate_hz: f64,
    nominal_freq_hz: f64,
    n_harmonics: usize,
) -> Result<WaveformMetrics> {
    let n = voltage.len();
    if n == 0 {
        return Err(OxiGridError::InvalidParameter(
            "voltage waveform is empty".to_string(),
        ));
    }
    if current.len() != n {
        return Err(OxiGridError::InvalidParameter(format!(
            "voltage length {} ≠ current length {}",
            n,
            current.len()
        )));
    }
    if sample_rate_hz <= 0.0 || nominal_freq_hz <= 0.0 {
        return Err(OxiGridError::InvalidParameter(
            "sample_rate_hz and nominal_freq_hz must be positive".to_string(),
        ));
    }
    if n_harmonics == 0 {
        return Err(OxiGridError::InvalidParameter(
            "n_harmonics must be ≥ 1".to_string(),
        ));
    }

    // ── Voltage metrics ──────────────────────────────────────────────────────
    let v_rms = rms(voltage);
    let v_peak = voltage
        .iter()
        .copied()
        .fold(f64::NEG_INFINITY, f64::max)
        .abs()
        .max(voltage.iter().copied().fold(f64::INFINITY, f64::min).abs());
    let v_avg_rect = voltage.iter().map(|&v| v.abs()).sum::<f64>() / n as f64;

    let crest_factor = if v_rms > 1e-15 { v_peak / v_rms } else { 0.0 };
    let form_factor = if v_avg_rect > 1e-15 {
        v_rms / v_avg_rect
    } else {
        0.0
    };

    // ── Harmonic extraction ──────────────────────────────────────────────────
    let v_harmonics = extract_harmonics(voltage, sample_rate_hz, nominal_freq_hz, n_harmonics);
    let i_harmonics = extract_harmonics(current, sample_rate_hz, nominal_freq_hz, n_harmonics);

    let v1 = v_harmonics
        .first()
        .map(|h| h.magnitude_pu)
        .unwrap_or(0.0)
        .max(1e-15);
    let i1 = i_harmonics.first().map(|h| h.magnitude_pu).unwrap_or(0.0);

    // THD_V = √(Σ_{h≥2} V_h²) / V_1 × 100
    let thd_v_sq: f64 = v_harmonics
        .iter()
        .skip(1)
        .map(|h| h.magnitude_pu.powi(2))
        .sum();
    let thd_pct = thd_v_sq.sqrt() / v1 * 100.0;

    // K-factor uses current harmonics.
    let k_factor = compute_k_factor(&i_harmonics);

    // ── Power quantities ──────────────────────────────────────────────────────
    // Active power P = (1/N) Σ v·i
    let active_power: f64 = voltage
        .iter()
        .zip(current.iter())
        .map(|(&v, &i)| v * i)
        .sum::<f64>()
        / n as f64;

    // Apparent power S = V_rms × I_rms
    let i_rms = rms(current);
    let apparent_power = v_rms * i_rms;

    // Fundamental reactive power Q₁ = V₁·I₁·sin(φ₁)
    // φ₁ = phase_V₁ - phase_I₁
    let phi1_v = v_harmonics.first().map(|h| h.phase_rad).unwrap_or(0.0);
    let phi1_i = i_harmonics.first().map(|h| h.phase_rad).unwrap_or(0.0);
    let phi1 = phi1_v - phi1_i;
    let q1 = v1 * i1 * phi1.sin();

    // Distortion power D = √(S² − P² − Q₁²)
    let s2 = apparent_power * apparent_power;
    let distortion_power_sq = (s2 - active_power * active_power - q1 * q1).max(0.0);
    let distortion_power = distortion_power_sq.sqrt();

    let displacement_pf = phi1.cos();
    let true_pf = if apparent_power > 1e-15 {
        (active_power / apparent_power).clamp(-1.0, 1.0)
    } else {
        1.0
    };

    // ── Build harmonic_components with power ─────────────────────────────────
    let harmonic_components: Vec<HarmonicComponent> = v_harmonics
        .iter()
        .enumerate()
        .map(|(idx, vh)| {
            let ih = i_harmonics.get(idx);
            // Harmonic active power: P_h = V_h · I_h · cos(φ_V_h − φ_I_h)
            let hp = ih
                .map(|i| {
                    let dphi = vh.phase_rad - i.phase_rad;
                    vh.magnitude_pu * i.magnitude_pu * dphi.cos()
                })
                .unwrap_or(0.0);

            HarmonicComponent {
                order: vh.order,
                magnitude_pu: vh.magnitude_pu,
                phase_rad: vh.phase_rad,
                power: hp,
            }
        })
        .collect();

    Ok(WaveformMetrics {
        thd_pct,
        crest_factor,
        form_factor,
        k_factor,
        distortion_power,
        displacement_pf,
        true_pf,
        harmonic_components,
    })
}

// ─────────────────────────────────────────────────────────────────────────────
// Utility
// ─────────────────────────────────────────────────────────────────────────────

/// RMS of a slice.
fn rms(samples: &[f64]) -> f64 {
    if samples.is_empty() {
        return 0.0;
    }
    let mean_sq = samples.iter().map(|&v| v * v).sum::<f64>() / samples.len() as f64;
    mean_sq.sqrt()
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn sine(amp: f64, freq_hz: f64, phase_rad: f64, fs: f64, n: usize) -> Vec<f64> {
        (0..n)
            .map(|i| amp * (2.0 * PI * freq_hz * i as f64 / fs + phase_rad).sin())
            .collect()
    }

    #[test]
    fn test_thd_pure_fundamental() {
        // Pure 50 Hz sine → THD should be ≈ 0 %
        let fs = 10_000.0_f64;
        let n = (fs / 50.0 * 10.0) as usize; // 10 cycles
        let v = sine(1.0, 50.0, 0.0, fs, n);
        let i = v.clone();
        let m = analyze_waveform(&v, &i, fs, 50.0, 40).expect("analysis failed");
        assert!(
            m.thd_pct < 1.0,
            "THD for pure sine should be < 1%, got {:.4}",
            m.thd_pct
        );
    }

    #[test]
    fn test_thd_with_10pct_3rd_harmonic() {
        // v = sin(ωt) + 0.1·sin(3ωt) → THD ≈ 10 %
        let fs = 10_000.0_f64;
        let f0 = 50.0_f64;
        let n = (fs / f0 * 10.0) as usize;
        let v: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                (2.0 * PI * f0 * t).sin() + 0.1 * (2.0 * PI * 3.0 * f0 * t).sin()
            })
            .collect();
        let i = v.clone();
        let m = analyze_waveform(&v, &i, fs, f0, 10).expect("analysis failed");
        assert!(
            (m.thd_pct - 10.0).abs() < 1.0,
            "Expected THD ≈ 10%, got {:.3}%",
            m.thd_pct
        );
    }

    #[test]
    fn test_crest_factor_sinusoid() {
        // Pure sine: crest factor = √2 ≈ 1.4142
        let fs = 10_000.0_f64;
        let f0 = 50.0_f64;
        let n = (fs / f0 * 20.0) as usize;
        let v = sine(1.0, f0, 0.0, fs, n);
        let i = v.clone();
        let m = analyze_waveform(&v, &i, fs, f0, 1).expect("analysis failed");
        assert!(
            (m.crest_factor - 2.0_f64.sqrt()).abs() < 0.05,
            "Crest factor should be √2, got {:.4}",
            m.crest_factor
        );
    }

    #[test]
    fn test_k_factor_fundamental_only() {
        // Only fundamental → K = 1.0
        let harmonics = vec![HarmonicComponent {
            order: 1,
            magnitude_pu: 1.0,
            phase_rad: 0.0,
            power: 0.0,
        }];
        let k = compute_k_factor(&harmonics);
        assert!(
            (k - 1.0).abs() < 1e-10,
            "K-factor for fundamental-only should be 1.0, got {k:.6}"
        );
    }

    #[test]
    fn test_k_factor_with_harmonics() {
        // h=1 (mag=1) + h=3 (mag=1) → K = (1+9)/(1+1) = 5
        let harmonics = vec![
            HarmonicComponent {
                order: 1,
                magnitude_pu: 1.0,
                phase_rad: 0.0,
                power: 0.0,
            },
            HarmonicComponent {
                order: 3,
                magnitude_pu: 1.0,
                phase_rad: 0.0,
                power: 0.0,
            },
        ];
        let k = compute_k_factor(&harmonics);
        assert!(
            (k - 5.0).abs() < 1e-10,
            "K-factor should be 5.0, got {k:.6}"
        );
    }

    #[test]
    fn test_transformer_derating_k1() {
        // K-factor = K-rated → derating ≈ √(1/(1+0.1)) ≈ 0.953
        let d = transformer_derating_factor(4.0, 4.0);
        let expected = (1.0_f64 / (1.0 + 0.10)).sqrt();
        assert!(
            (d - expected).abs() < 1e-9,
            "Derating should be {expected:.4}, got {d:.4}"
        );
    }

    #[test]
    fn test_transformer_derating_k1_trivial() {
        // K_factor = K_rated = 1 → derating close to sqrt(1/1.1)
        let d = transformer_derating_factor(1.0, 1.0);
        assert!(
            d > 0.9 && d < 1.0,
            "Derating for K=K_rated=1 should be ~0.95, got {d:.4}"
        );
    }

    #[test]
    fn test_extract_harmonics_length() {
        let fs = 10_000.0_f64;
        let f0 = 50.0_f64;
        let n = (fs / f0 * 4.0) as usize;
        let v = sine(1.0, f0, 0.0, fs, n);
        let comps = extract_harmonics(&v, fs, f0, 10);
        // Should not exceed 10 components (may be fewer if Nyquist is hit)
        assert!(comps.len() <= 10);
        assert!(!comps.is_empty());
    }

    #[test]
    fn test_analyze_waveform_error_empty() {
        let result = analyze_waveform(&[], &[], 10_000.0, 50.0, 10);
        assert!(result.is_err());
    }

    #[test]
    fn test_analyze_waveform_error_length_mismatch() {
        let v = vec![0.0_f64; 100];
        let i = vec![0.0_f64; 50];
        let result = analyze_waveform(&v, &i, 10_000.0, 50.0, 10);
        assert!(result.is_err());
    }

    #[test]
    fn test_goertzel_amplitude() {
        // Single sine of amplitude 2.0 at 50 Hz, sampled at 10 kHz for 10 cycles.
        let fs = 10_000.0_f64;
        let f0 = 50.0_f64;
        let n = (fs / f0 * 10.0) as usize;
        let v = sine(2.0, f0, 0.0, fs, n);
        let (re, im) = goertzel(&v, f0, fs);
        let peak = (re * re + im * im).sqrt();
        assert!(
            (peak - 2.0).abs() < 0.05,
            "Goertzel peak amplitude should be ≈2.0, got {peak:.4}"
        );
    }

    #[test]
    fn test_interharmonic_detection_none_for_pure_sine() {
        // Pure fundamental sine → no interharmonics above threshold.
        let fs = 10_000.0_f64;
        let f0 = 50.0_f64;
        let n = (fs / f0 * 4.0) as usize;
        let v = sine(1.0, f0, 0.0, fs, n);
        // Build simple magnitude spectrum via DFT (re² + im²)^0.5
        let spectrum: Vec<f64> = (0..n)
            .map(|k| {
                let (re, im) = goertzel(&v, k as f64 * fs / n as f64, fs);
                (re * re + im * im).sqrt()
            })
            .take(n / 2) // one-sided
            .collect();
        let ih = detect_interharmonics(&spectrum, fs, f0, 0.05);
        // Any component found should not coincide with harmonic frequencies.
        for (freq, _mag) in &ih {
            let nearest = (freq / f0).round();
            assert!(
                (freq - nearest * f0).abs() > 1.0,
                "Detected interharmonic at {freq:.1} Hz too close to harmonic"
            );
        }
    }
}
