/// Harmonic analysis: THD, individual harmonic distortion, IEEE 519 compliance.
///
/// Uses a simple DFT (Goertzel algorithm) for single-frequency extraction and
/// a full-spectrum DFT for THD calculation.  For production use with large
/// datasets, replace with an FFT implementation.
use serde::{Deserialize, Serialize};

/// Harmonic order and its magnitude/phase.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct HarmonicComponent {
    /// Harmonic order (1 = fundamental, 2 = 2nd, …)
    pub order: u32,
    /// Magnitude (RMS, same units as input)
    pub magnitude: f64,
    /// Phase angle `rad`
    pub phase_rad: f64,
    /// Individual Harmonic Distortion [%] relative to fundamental
    pub ihd_pct: f64,
}

/// Result of a harmonic analysis.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HarmonicSpectrum {
    /// Fundamental frequency `Hz`
    pub fundamental_hz: f64,
    /// Fundamental magnitude (RMS)
    pub fundamental: f64,
    /// Harmonic components (orders 2..N)
    pub harmonics: Vec<HarmonicComponent>,
    /// Total Harmonic Distortion [%] = √(Σh≥2 V_h²) / V_1 × 100
    pub thd_pct: f64,
    /// Total Demand Distortion [%] (like THD but relative to rated/peak demand)
    pub tdd_pct: Option<f64>,
}

impl HarmonicSpectrum {
    /// Check IEEE 519-2022 voltage THD compliance at a given bus voltage `kV`.
    ///
    /// Returns `true` if THD ≤ limit.
    pub fn ieee519_voltage_compliant(&self, bus_kv: f64) -> bool {
        let limit_pct = if bus_kv <= 1.0 {
            8.0 // < 1 kV: 8%
        } else if bus_kv <= 69.0 {
            5.0 // 1–69 kV: 5%
        } else if bus_kv <= 161.0 {
            2.5 // 69–161 kV: 2.5%
        } else {
            1.5 // > 161 kV: 1.5%
        };
        self.thd_pct <= limit_pct
    }

    /// Check IEEE 519-2022 individual harmonic voltage limit.
    pub fn ieee519_individual_voltage_compliant(&self, bus_kv: f64) -> bool {
        let ihd_limit = if bus_kv <= 1.0 {
            5.0
        } else if bus_kv <= 69.0 {
            3.0
        } else {
            1.5
        };
        self.harmonics.iter().all(|h| h.ihd_pct <= ihd_limit)
    }
}

/// Compute the Discrete Fourier Transform (DFT) of `samples`.
///
/// Returns complex spectrum as `(re, im)` pairs for frequencies
/// 0, fs/N, 2·fs/N, …, (N-1)·fs/N.
pub fn dft(samples: &[f64]) -> Vec<(f64, f64)> {
    let n = samples.len();
    use std::f64::consts::PI;
    (0..n)
        .map(|k| {
            let (mut re, mut im) = (0.0_f64, 0.0_f64);
            for (j, &x) in samples.iter().enumerate() {
                let angle = -2.0 * PI * k as f64 * j as f64 / n as f64;
                re += x * angle.cos();
                im += x * angle.sin();
            }
            (re / n as f64, im / n as f64)
        })
        .collect()
}

/// Extract a single frequency bin using the Goertzel algorithm (efficient).
///
/// Returns (real, imag) for harmonic at `freq_hz` given `sample_rate_hz`.
pub fn goertzel(samples: &[f64], freq_hz: f64, sample_rate_hz: f64) -> (f64, f64) {
    use std::f64::consts::PI;
    let n = samples.len() as f64;
    let k = (freq_hz / sample_rate_hz * n).round() as usize;
    let omega = 2.0 * PI * k as f64 / n;
    let coeff = 2.0 * omega.cos();
    let (mut s1, mut s2) = (0.0_f64, 0.0_f64);
    for &x in samples {
        let s = x + coeff * s1 - s2;
        s2 = s1;
        s1 = s;
    }
    let re = s1 - s2 * omega.cos();
    let im = s2 * omega.sin();
    (re / n * 2.0, im / n * 2.0)
}

/// Analyse harmonics in a waveform.
///
/// # Arguments
/// - `samples`          — time-domain waveform (one full period recommended)
/// - `sample_rate_hz`   — sampling frequency `Hz`
/// - `fundamental_hz`   — fundamental frequency `Hz` (50 or 60)
/// - `max_order`        — highest harmonic order to compute
/// - `rated_current`    — optional rated current for TDD calculation
pub fn analyse(
    samples: &[f64],
    sample_rate_hz: f64,
    fundamental_hz: f64,
    max_order: u32,
    rated_current: Option<f64>,
) -> HarmonicSpectrum {
    // Fundamental
    let (re1, im1) = goertzel(samples, fundamental_hz, sample_rate_hz);
    let v1 = (re1 * re1 + im1 * im1).sqrt() / std::f64::consts::SQRT_2; // peak → rms
    let phase1 = im1.atan2(re1);
    let fundamental = v1.max(1e-12);

    // Harmonics
    let mut harmonics = Vec::new();
    let mut thd_sum_sq = 0.0_f64;

    for h in 2..=max_order {
        let freq = fundamental_hz * h as f64;
        let (re, im) = goertzel(samples, freq, sample_rate_hz);
        let mag = (re * re + im * im).sqrt() / std::f64::consts::SQRT_2;
        let ihd = mag / fundamental * 100.0;
        thd_sum_sq += mag * mag;
        harmonics.push(HarmonicComponent {
            order: h,
            magnitude: mag,
            phase_rad: im.atan2(re) - phase1,
            ihd_pct: ihd,
        });
    }

    let thd_pct = thd_sum_sq.sqrt() / fundamental * 100.0;
    let tdd_pct = rated_current.map(|i_rated| thd_sum_sq.sqrt() / i_rated.max(1e-12) * 100.0);

    HarmonicSpectrum {
        fundamental_hz,
        fundamental,
        harmonics,
        thd_pct,
        tdd_pct,
    }
}

/// Generate a test waveform with known harmonic content.
///
/// Returns samples of: A1·sin(2πf·t) + Σ Ah·sin(2π·h·f·t + φh)
pub fn synthetic_waveform(
    fundamental_hz: f64,
    sample_rate_hz: f64,
    n_samples: usize,
    components: &[(u32, f64, f64)], // (order, amplitude, phase_rad)
) -> Vec<f64> {
    use std::f64::consts::PI;
    (0..n_samples)
        .map(|i| {
            let t = i as f64 / sample_rate_hz;
            components
                .iter()
                .map(|&(h, amp, phi)| amp * (2.0 * PI * fundamental_hz * h as f64 * t + phi).sin())
                .sum()
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn pure_sine(freq_hz: f64, amp: f64, sample_rate: f64, n: usize) -> Vec<f64> {
        (0..n)
            .map(|i| amp * (2.0 * PI * freq_hz * i as f64 / sample_rate).sin())
            .collect()
    }

    #[test]
    fn test_pure_sine_thd_near_zero() {
        let samples = pure_sine(60.0, 1.0, 6000.0, 6000);
        let spec = analyse(&samples, 6000.0, 60.0, 10, None);
        assert!(spec.thd_pct < 1.0, "THD={:.4}%", spec.thd_pct);
        assert!(
            (spec.fundamental - 1.0 / 2_f64.sqrt()).abs() < 0.01,
            "V1_rms={:.4}",
            spec.fundamental
        );
    }

    #[test]
    fn test_thd_with_known_3rd_harmonic() {
        // V = sin(ωt) + 0.1·sin(3ωt) → THD ≈ 10%
        let components = vec![(1, 1.0, 0.0), (3, 0.1, 0.0)];
        let samples = synthetic_waveform(60.0, 6000.0, 6000, &components);
        let spec = analyse(&samples, 6000.0, 60.0, 10, None);
        assert!(
            (spec.thd_pct - 10.0).abs() < 0.5,
            "THD={:.2}%",
            spec.thd_pct
        );
    }

    #[test]
    fn test_goertzel_extracts_correct_amplitude() {
        let amp = 2.5;
        let samples = pure_sine(60.0, amp, 6000.0, 600);
        let (re, im) = goertzel(&samples, 60.0, 6000.0);
        let mag = (re * re + im * im).sqrt(); // goertzel returns peak amplitude
        assert!((mag - amp).abs() < 0.05, "mag={:.4} expected={}", mag, amp);
    }

    #[test]
    fn test_ieee519_compliant_low_thd() {
        let components = vec![(1, 1.0, 0.0), (3, 0.02, 0.0)];
        let samples = synthetic_waveform(60.0, 6000.0, 6000, &components);
        let spec = analyse(&samples, 6000.0, 60.0, 10, None);
        assert!(
            spec.ieee519_voltage_compliant(13.8),
            "THD={:.2}%",
            spec.thd_pct
        );
    }

    #[test]
    fn test_ieee519_non_compliant_high_thd() {
        // 20% 3rd harmonic → THD ≈ 20% > 5% limit
        let components = vec![(1, 1.0, 0.0), (3, 0.20, 0.0)];
        let samples = synthetic_waveform(60.0, 6000.0, 6000, &components);
        let spec = analyse(&samples, 6000.0, 60.0, 10, None);
        assert!(
            !spec.ieee519_voltage_compliant(13.8),
            "THD={:.2}%",
            spec.thd_pct
        );
    }
}
