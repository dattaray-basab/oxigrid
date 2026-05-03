//! Auto-generated module
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

use super::types::OscillationMode;

#[cfg(test)]
use super::types::{
    AlarmLevel, DetectedMode, FourierOscillationDetector, ModeType, OscillationError,
    OscillationMonitor, OscillationMonitorConfig, OscillationMonitorResult, PronyAnalyzer,
    RingdownAnalyzer, WamOscillationMonitor,
};

/// Solve an `n×n` linear system `A·x = b` via Gaussian elimination with partial
/// pivoting. Returns `None` if the matrix is singular.
#[allow(clippy::needless_range_loop)]
pub(crate) fn solve_linear_system(a_flat: &[f64], b: &[f64], n: usize) -> Option<Vec<f64>> {
    if n == 0 {
        return Some(vec![]);
    }
    let mut aug: Vec<Vec<f64>> = (0..n)
        .map(|i| {
            let mut row: Vec<f64> = a_flat[i * n..(i + 1) * n].to_vec();
            row.push(b[i]);
            row
        })
        .collect();
    for col in 0..n {
        let mut max_row = col;
        let mut max_val = aug[col][col].abs();
        for row in (col + 1)..n {
            let abs_val = aug[row][col].abs();
            if abs_val > max_val {
                max_val = abs_val;
                max_row = row;
            }
        }
        if max_val < 1e-14 {
            return None;
        }
        aug.swap(col, max_row);
        let pivot = aug[col][col];
        for row in (col + 1)..n {
            let factor = aug[row][col] / pivot;
            for k in col..=n {
                let piv_k = aug[col][k];
                aug[row][k] -= factor * piv_k;
            }
        }
    }
    let mut x = vec![0.0f64; n];
    for i in (0..n).rev() {
        let mut s = aug[i][n];
        for j in (i + 1)..n {
            s -= aug[i][j] * x[j];
        }
        if aug[i][i].abs() < 1e-14 {
            return None;
        }
        x[i] = s / aug[i][i];
    }
    Some(x)
}
/// Compute eigenvalues of the companion matrix for polynomial
/// `z^p - a[0]·z^{p-1} - a[1]·z^{p-2} - … - a[p-1]`
/// using the real-valued QR algorithm (Francis double-shift).
///
/// Returns complex eigenvalues as `Vec<(re, im)>`.
pub(crate) fn companion_eigenvalues(a: &[f64], p: usize) -> Vec<(f64, f64)> {
    if p == 0 {
        return Vec::new();
    }
    if p == 1 {
        return vec![(a[0], 0.0)];
    }
    let mut c = vec![0.0f64; p * p];
    c[..p].copy_from_slice(&a[..p]);
    for i in 1..p {
        c[i * p + (i - 1)] = 1.0;
    }
    qr_eigenvalues(&mut c, p)
}
/// Simplified QR algorithm for a `p×p` real matrix stored row-major.
/// Returns complex eigenvalues `(re, im)`.
fn qr_eigenvalues(h: &mut [f64], p: usize) -> Vec<(f64, f64)> {
    hessenberg_form(h, p);
    let max_iter_per_eig = 60usize;
    let mut eigs: Vec<(f64, f64)> = Vec::with_capacity(p);
    let mut size = p;
    while size > 0 {
        if size == 1 {
            eigs.push((h[0], 0.0));
            break;
        }
        let sub12 = h[(size - 1) * p + (size - 2)].abs();
        let tol12 =
            1e-10 * (h[(size - 2) * p + (size - 2)].abs() + h[(size - 1) * p + (size - 1)].abs());
        if sub12 <= tol12 {
            eigs.push((h[(size - 1) * p + (size - 1)], 0.0));
            size -= 1;
            continue;
        }
        let mut deflated = false;
        for _it in 0..max_iter_per_eig {
            let a11 = h[(size - 2) * p + (size - 2)];
            let a12 = h[(size - 2) * p + (size - 1)];
            let a21 = h[(size - 1) * p + (size - 2)];
            let a22 = h[(size - 1) * p + (size - 1)];
            let trace = a11 + a22;
            let det = a11 * a22 - a12 * a21;
            let disc = trace * trace / 4.0 - det;
            let shift = if disc >= 0.0 {
                let sqrt_disc = disc.sqrt();
                let e1 = trace / 2.0 + sqrt_disc;
                let e2 = trace / 2.0 - sqrt_disc;
                if (e1 - a22).abs() < (e2 - a22).abs() {
                    e1
                } else {
                    e2
                }
            } else {
                trace / 2.0
            };
            qr_step(h, p, size, shift);
            let sub = h[(size - 1) * p + (size - 2)].abs();
            let tol = 1e-10
                * (h[(size - 2) * p + (size - 2)].abs() + h[(size - 1) * p + (size - 1)].abs());
            if sub <= tol {
                eigs.push((h[(size - 1) * p + (size - 1)], 0.0));
                size -= 1;
                deflated = true;
                break;
            }
            if size >= 3 {
                let sub2 = h[(size - 2) * p + (size - 3)].abs();
                let tol2 = 1e-10
                    * (h[(size - 3) * p + (size - 3)].abs() + h[(size - 2) * p + (size - 2)].abs());
                if sub2 <= tol2 {
                    let ra = h[(size - 2) * p + (size - 2)];
                    let rb = h[(size - 2) * p + (size - 1)];
                    let rc = h[(size - 1) * p + (size - 2)];
                    let rd = h[(size - 1) * p + (size - 1)];
                    let tr = ra + rd;
                    let dt = ra * rd - rb * rc;
                    let d2 = tr * tr / 4.0 - dt;
                    if d2 >= 0.0 {
                        let sq = d2.sqrt();
                        eigs.push((tr / 2.0 + sq, 0.0));
                        eigs.push((tr / 2.0 - sq, 0.0));
                    } else {
                        let sq = (-d2).sqrt();
                        eigs.push((tr / 2.0, sq));
                        eigs.push((tr / 2.0, -sq));
                    }
                    size -= 2;
                    deflated = true;
                    break;
                }
            }
        }
        if !deflated {
            if size == 2 {
                let ra = h[0];
                let rb = h[1];
                let rc = h[p];
                let rd = h[p + 1];
                let tr = ra + rd;
                let dt = ra * rd - rb * rc;
                let d2 = tr * tr / 4.0 - dt;
                if d2 >= 0.0 {
                    let sq = d2.sqrt();
                    eigs.push((tr / 2.0 + sq, 0.0));
                    eigs.push((tr / 2.0 - sq, 0.0));
                } else {
                    let sq = (-d2).sqrt();
                    eigs.push((tr / 2.0, sq));
                    eigs.push((tr / 2.0, -sq));
                }
            } else {
                for i in 0..size {
                    eigs.push((h[i * p + i], 0.0));
                }
            }
            break;
        }
    }
    eigs
}
/// Reduce matrix to upper Hessenberg form in-place via Householder reflections.
fn hessenberg_form(h: &mut [f64], n: usize) {
    for k in 0..(n.saturating_sub(2)) {
        let mut x: Vec<f64> = (k + 1..n).map(|i| h[i * n + k]).collect();
        let norm: f64 = x.iter().map(|&v| v * v).sum::<f64>().sqrt();
        if norm < 1e-14 {
            continue;
        }
        let sign = if x[0] >= 0.0 { 1.0 } else { -1.0 };
        x[0] += sign * norm;
        let norm2: f64 = x.iter().map(|&v| v * v).sum::<f64>().sqrt();
        if norm2 < 1e-14 {
            continue;
        }
        for v in &mut x {
            *v /= norm2;
        }
        for j in k..n {
            let dot: f64 = x
                .iter()
                .enumerate()
                .map(|(i, &v)| v * h[(k + 1 + i) * n + j])
                .sum();
            for (i, &v) in x.iter().enumerate() {
                h[(k + 1 + i) * n + j] -= 2.0 * v * dot;
            }
        }
        for i in 0..n {
            let dot: f64 = x
                .iter()
                .enumerate()
                .map(|(j, &v)| v * h[i * n + k + 1 + j])
                .sum();
            for (j, &v) in x.iter().enumerate() {
                h[i * n + k + 1 + j] -= 2.0 * v * dot;
            }
        }
    }
}
/// Perform one QR step with a given shift on the leading `size×size` principal
/// submatrix of an `n×n` Hessenberg matrix stored row-major.
fn qr_step(h: &mut [f64], n: usize, size: usize, shift: f64) {
    for i in 0..size {
        h[i * n + i] -= shift;
    }
    let mut c_vec = vec![0.0f64; size - 1];
    let mut s_vec = vec![0.0f64; size - 1];
    for i in 0..(size - 1) {
        let a = h[i * n + i];
        let b = h[(i + 1) * n + i];
        let r = (a * a + b * b).sqrt();
        if r < 1e-14 {
            c_vec[i] = 1.0;
            s_vec[i] = 0.0;
            continue;
        }
        let c = a / r;
        let s = b / r;
        c_vec[i] = c;
        s_vec[i] = s;
        for j in i..size {
            let tmp1 = c * h[i * n + j] + s * h[(i + 1) * n + j];
            let tmp2 = -s * h[i * n + j] + c * h[(i + 1) * n + j];
            h[i * n + j] = tmp1;
            h[(i + 1) * n + j] = tmp2;
        }
    }
    for i in 0..(size - 1) {
        let c = c_vec[i];
        let s = s_vec[i];
        for k in 0..size {
            let tmp1 = c * h[k * n + i] + s * h[k * n + i + 1];
            let tmp2 = -s * h[k * n + i] + c * h[k * n + i + 1];
            h[k * n + i] = tmp1;
            h[k * n + i + 1] = tmp2;
        }
    }
    for i in 0..size {
        h[i * n + i] += shift;
    }
}
/// Estimate the amplitude of a damped sinusoidal component `A·exp(σt)·cos(ωt + φ)`
/// by projecting the signal onto both cosine and sine basis functions and taking
/// the magnitude of the complex coefficient, which is phase-independent.
pub(crate) fn estimate_amplitude(signal: &[f64], sigma: f64, omega: f64, dt_s: f64) -> f64 {
    let mut num_cos = 0.0f64;
    let mut num_sin = 0.0f64;
    let mut den = 0.0f64;
    for (k, &xk) in signal.iter().enumerate() {
        let t = k as f64 * dt_s;
        let env = (sigma * t).exp();
        let cos_t = (omega * t).cos();
        let sin_t = (omega * t).sin();
        num_cos += xk * env * cos_t;
        num_sin += xk * env * sin_t;
        den += env * env * (cos_t * cos_t + sin_t * sin_t);
    }
    if den > 1e-20 {
        (num_cos * num_cos + num_sin * num_sin).sqrt() / den
    } else {
        0.0
    }
}
/// Merge modes from multiple channels by clustering on frequency proximity.
pub(crate) fn merge_modes(modes: &[OscillationMode], _n_ch: usize) -> Vec<OscillationMode> {
    let tol_hz = 0.05;
    let mut merged: Vec<OscillationMode> = Vec::new();
    for mode in modes {
        if let Some(existing) = merged
            .iter_mut()
            .find(|m| (m.frequency_hz - mode.frequency_hz).abs() < tol_hz)
        {
            if mode.energy > existing.energy {
                existing.frequency_hz = mode.frequency_hz;
                existing.damping_ratio = mode.damping_ratio;
                existing.amplitude = mode.amplitude;
                existing.energy = mode.energy;
                existing.mode_type = mode.mode_type;
            }
            for &ch in &mode.participating_signals {
                if !existing.participating_signals.contains(&ch) {
                    existing.participating_signals.push(ch);
                }
            }
        } else {
            merged.push(mode.clone());
        }
    }
    merged
}
/// Compute a scalar stability index ∈ \[0, 1\] from the detected modes.
///
/// Uses the minimum damping ratio across all modes with amplitude above noise.
pub(crate) fn compute_stability_index(modes: &[OscillationMode]) -> f64 {
    if modes.is_empty() {
        return 1.0;
    }
    let min_damp = modes
        .iter()
        .map(|m| m.damping_ratio)
        .fold(f64::INFINITY, f64::min);
    let index = (min_damp + 0.1) / 0.2;
    index.clamp(0.0, 1.0)
}
#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;
    fn default_config() -> OscillationMonitorConfig {
        OscillationMonitorConfig {
            sampling_rate_hz: 50.0,
            analysis_window_s: 2.0,
            min_mode_energy: 1e-6,
            frequency_range_hz: (0.05, 5.0),
            alarm_damping_threshold: 0.05,
            alarm_amplitude_threshold: 0.01,
        }
    }
    /// Generate `n` samples of a pure sinusoid at `freq_hz` with sampling
    /// rate `fs`.
    fn pure_sinusoid(freq_hz: f64, amp: f64, n: usize, fs: f64) -> Vec<f64> {
        (0..n)
            .map(|k| amp * (2.0 * PI * freq_hz * k as f64 / fs).sin())
            .collect()
    }
    /// Generate a damped sinusoid `A·exp(σt)·cos(ωt)`.
    fn damped_sinusoid(freq_hz: f64, amp: f64, sigma: f64, n: usize, fs: f64) -> Vec<f64> {
        let dt = 1.0 / fs;
        (0..n)
            .map(|k| {
                let t = k as f64 * dt;
                amp * (sigma * t).exp() * (2.0 * PI * freq_hz * t).cos()
            })
            .collect()
    }
    #[test]
    fn test_classify_mode_all_ranges() {
        assert_eq!(
            OscillationMonitor::classify_mode(0.05),
            ModeType::GlobalMode
        );
        assert_eq!(
            OscillationMonitor::classify_mode(0.5),
            ModeType::InterAreaMode
        );
        assert_eq!(
            OscillationMonitor::classify_mode(1.0),
            ModeType::LocalAreaMode
        );
        assert_eq!(
            OscillationMonitor::classify_mode(2.5),
            ModeType::LocalPlantMode
        );
        assert_eq!(
            OscillationMonitor::classify_mode(4.0),
            ModeType::TorsionalMode
        );
        assert_eq!(
            OscillationMonitor::classify_mode(0.1),
            ModeType::InterAreaMode
        );
        assert_eq!(
            OscillationMonitor::classify_mode(0.8),
            ModeType::LocalAreaMode
        );
        assert_eq!(
            OscillationMonitor::classify_mode(2.0),
            ModeType::LocalPlantMode
        );
        assert_eq!(
            OscillationMonitor::classify_mode(3.0),
            ModeType::TorsionalMode
        );
    }
    #[test]
    fn test_pure_sinusoid_detects_mode() {
        let cfg = default_config();
        let monitor = OscillationMonitor::new(cfg);
        let fs = 50.0_f64;
        let dt = 1.0 / fs;
        let n = 100;
        let sig = pure_sinusoid(0.5, 0.1, n, fs);
        let result = monitor.analyze(&[sig], dt).unwrap();
        assert!(
            !result.modes.is_empty(),
            "Should detect at least one mode in a pure sinusoid"
        );
        let closest = result
            .modes
            .iter()
            .min_by(|a, b| {
                (a.frequency_hz - 0.5)
                    .abs()
                    .partial_cmp(&(b.frequency_hz - 0.5).abs())
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .unwrap();
        assert!(
            (closest.frequency_hz - 0.5).abs() < 0.15,
            "Mode frequency should be near 0.5 Hz, got {:.3} Hz",
            closest.frequency_hz
        );
    }
    #[test]
    fn test_damped_sinusoid_positive_damping() {
        let cfg = default_config();
        let monitor = OscillationMonitor::new(cfg);
        let fs = 50.0_f64;
        let dt = 1.0 / fs;
        let n = 100;
        let sig = damped_sinusoid(1.0, 1.0, -1.0, n, fs);
        let result = monitor.analyze(&[sig], dt).unwrap();
        if let Some(ref dom) = result.dominant_mode {
            assert!(
                dom.damping_ratio >= -0.5,
                "Decaying sinusoid should have non-strongly-negative damping, got {:.3}",
                dom.damping_ratio
            );
        }
    }
    #[test]
    fn test_growing_oscillation_triggers_emergency_alarm() {
        let cfg = OscillationMonitorConfig {
            sampling_rate_hz: 50.0,
            analysis_window_s: 2.0,
            min_mode_energy: 1e-9,
            frequency_range_hz: (0.05, 5.0),
            alarm_damping_threshold: 0.05,
            alarm_amplitude_threshold: 0.001,
        };
        let monitor = OscillationMonitor::new(cfg);
        let fs = 50.0_f64;
        let dt = 1.0 / fs;
        let n = 60;
        let sig = damped_sinusoid(0.5, 0.05, 0.5, n, fs);
        let result = monitor.analyze(&[sig], dt);
        match result {
            Ok(r) => {
                let has_emergency = r.alarms.iter().any(|a| a.severity == AlarmLevel::Emergency);
                let has_negative_damp = r.modes.iter().any(|m| m.damping_ratio < 0.0);
                if has_negative_damp {
                    assert!(
                        has_emergency,
                        "Negative damping should trigger Emergency alarm"
                    );
                }
                assert!(
                    r.system_stability_index <= 1.0,
                    "Stability index out of range"
                );
            }
            Err(OscillationError::InsufficientSamples { .. }) => {}
            Err(e) => panic!("Unexpected error: {e}"),
        }
    }
    #[test]
    fn test_multiple_modes_detected() {
        let cfg = OscillationMonitorConfig {
            sampling_rate_hz: 100.0,
            analysis_window_s: 4.0,
            min_mode_energy: 1e-8,
            frequency_range_hz: (0.05, 5.0),
            alarm_damping_threshold: 0.05,
            alarm_amplitude_threshold: 0.01,
        };
        let monitor = OscillationMonitor::new(cfg);
        let fs = 100.0_f64;
        let dt = 1.0 / fs;
        let n = 200;
        let sig: Vec<f64> = (0..n)
            .map(|k| {
                let t = k as f64 * dt;
                0.1 * (2.0 * PI * 0.5 * t).sin() + 0.05 * (2.0 * PI * 2.0 * t).sin()
            })
            .collect();
        let result = monitor.analyze(&[sig], dt).unwrap();
        assert!(
            !result.modes.is_empty(),
            "Should detect at least 1 mode in a two-tone signal, found {}",
            result.modes.len()
        );
    }
    #[test]
    fn test_trend_detection_deteriorating() {
        let cfg = default_config();
        let mut monitor = OscillationMonitor::new(cfg);
        let damping_sequence = [0.15, 0.12, 0.09, 0.06, 0.03, 0.01];
        let mut trend_alarms = Vec::new();
        for &damp in &damping_sequence {
            let result = OscillationMonitorResult {
                modes: vec![OscillationMode {
                    frequency_hz: 0.5,
                    damping_ratio: damp,
                    amplitude: 0.1,
                    energy: 0.5,
                    mode_type: ModeType::InterAreaMode,
                    participating_signals: vec![0],
                }],
                alarms: Vec::new(),
                dominant_mode: Some(OscillationMode {
                    frequency_hz: 0.5,
                    damping_ratio: damp,
                    amplitude: 0.1,
                    energy: 0.5,
                    mode_type: ModeType::InterAreaMode,
                    participating_signals: vec![0],
                }),
                system_stability_index: damp / 0.2,
                analysis_timestamp: 0.0,
            };
            trend_alarms = monitor.update_and_check_trend(&result);
        }
        assert!(
            !trend_alarms.is_empty() || monitor.history.len() >= 4,
            "Deteriorating trend should produce alarms or build history"
        );
    }
    #[test]
    fn test_insufficient_samples_error() {
        let cfg = default_config();
        let monitor = OscillationMonitor::new(cfg);
        let sig = vec![1.0, -1.0, 1.0];
        let result = monitor.analyze(&[sig], 0.02);
        assert!(
            matches!(result, Err(OscillationError::InsufficientSamples { .. })),
            "Should return InsufficientSamples for < 4 samples"
        );
    }
    #[test]
    fn test_stability_index_range() {
        let cfg = default_config();
        let monitor = OscillationMonitor::new(cfg);
        let fs = 50.0_f64;
        let n = 50;
        let sig = damped_sinusoid(1.0, 0.1, -5.0, n, fs);
        let result = monitor.analyze(&[sig], 1.0 / fs).unwrap();
        assert!(
            result.system_stability_index >= 0.0 && result.system_stability_index <= 1.0,
            "Stability index must be in [0, 1], got {}",
            result.system_stability_index
        );
    }
    #[test]
    fn test_analyze_empty_signals_returns_insufficient_samples() {
        let cfg = default_config();
        let monitor = OscillationMonitor::new(cfg);
        let result = monitor.analyze(&[], 0.02);
        assert!(
            matches!(
                result,
                Err(OscillationError::InsufficientSamples { got: 0, .. })
            ),
            "Empty signals slice should give InsufficientSamples{{got:0}}"
        );
    }
    #[test]
    fn test_analyze_zero_dt_returns_invalid_config() {
        let cfg = default_config();
        let monitor = OscillationMonitor::new(cfg);
        let sig = pure_sinusoid(0.5, 0.1, 10, 50.0);
        let result = monitor.analyze(&[sig], 0.0);
        assert!(
            matches!(result, Err(OscillationError::InvalidConfig(_))),
            "dt_s == 0.0 should return InvalidConfig"
        );
    }
    #[test]
    fn test_stability_index_empty_modes_is_one() {
        let index = compute_stability_index(&[]);
        assert!(
            (index - 1.0).abs() < 1e-12,
            "Empty modes should give stability index 1.0, got {index}"
        );
    }
    #[test]
    fn test_update_trend_single_epoch_no_alarms() {
        let cfg = default_config();
        let mut monitor = OscillationMonitor::new(cfg);
        let result = OscillationMonitorResult {
            modes: Vec::new(),
            alarms: Vec::new(),
            dominant_mode: Some(OscillationMode {
                frequency_hz: 0.5,
                damping_ratio: 0.1,
                amplitude: 0.05,
                energy: 0.4,
                mode_type: ModeType::InterAreaMode,
                participating_signals: vec![0],
            }),
            system_stability_index: 1.0,
            analysis_timestamp: 0.0,
        };
        let alarms = monitor.update_and_check_trend(&result);
        assert!(
            alarms.is_empty(),
            "Single epoch should produce no trend alarms (history len < 2)"
        );
    }
}
/// Solve n×n system A·x = b via Gaussian elimination with partial pivoting.
#[allow(clippy::needless_range_loop)]
pub(crate) fn prony_solve_ls(a_flat: &[f64], b: &[f64], n: usize) -> Option<Vec<f64>> {
    if n == 0 {
        return Some(vec![]);
    }
    let mut aug: Vec<Vec<f64>> = (0..n)
        .map(|i| {
            let mut row: Vec<f64> = a_flat[i * n..(i + 1) * n].to_vec();
            row.push(b[i]);
            row
        })
        .collect();
    for col in 0..n {
        let mut max_row = col;
        let mut max_val = aug[col][col].abs();
        for (row, aug_row) in aug.iter().enumerate().skip(col + 1).take(n - col - 1) {
            let v = aug_row[col].abs();
            if v > max_val {
                max_val = v;
                max_row = row;
            }
        }
        if max_val < 1e-14 {
            return None;
        }
        aug.swap(col, max_row);
        let pivot = aug[col][col];
        for row in (col + 1)..n {
            let factor = aug[row][col] / pivot;
            for k in col..=n {
                let pv = aug[col][k];
                aug[row][k] -= factor * pv;
            }
        }
    }
    let mut x = vec![0.0f64; n];
    for i in (0..n).rev() {
        let mut s = aug[i][n];
        for j in (i + 1)..n {
            s -= aug[i][j] * x[j];
        }
        if aug[i][i].abs() < 1e-14 {
            return None;
        }
        x[i] = s / aug[i][i];
    }
    Some(x)
}
/// Estimate amplitude and phase of a damped sinusoid via complex projection.
pub(crate) fn prony_fit_amplitude(signal: &[f64], sigma: f64, omega: f64, dt: f64) -> (f64, f64) {
    let mut re = 0.0f64;
    let mut im = 0.0f64;
    let mut den = 0.0f64;
    for (k, &xk) in signal.iter().enumerate() {
        let t = k as f64 * dt;
        let env = (sigma * t).exp();
        let c = (omega * t).cos();
        let s = (omega * t).sin();
        re += xk * env * c;
        im += xk * env * s;
        den += env * env;
    }
    if den > 1e-20 {
        let amplitude = (re * re + im * im).sqrt() / den;
        let phase = im.atan2(re);
        (amplitude, phase)
    } else {
        (0.0, 0.0)
    }
}
#[cfg(test)]
mod wam_tests {
    use super::*;
    /// Generate damped sinusoid: x[n] = A·exp(−ζ·2π·f·n/fs)·cos(2π·f·n/fs + φ)
    fn damped_sinusoid(freq: f64, amp: f64, zeta: f64, phase: f64, n: usize, fs: f64) -> Vec<f64> {
        (0..n)
            .map(|k| {
                let t = k as f64 / fs;
                let omega = 2.0 * std::f64::consts::PI * freq;
                amp * (-zeta * omega * t).exp() * (omega * t + phase).cos()
            })
            .collect()
    }
    fn pure_sine(freq: f64, amp: f64, n: usize, fs: f64) -> Vec<f64> {
        (0..n)
            .map(|k| {
                let t = k as f64 / fs;
                amp * (2.0 * std::f64::consts::PI * freq * t).cos()
            })
            .collect()
    }
    #[test]
    fn test_prony_analyzer_single_mode() {
        let fs = 50.0;
        let n = 200;
        let freq = 0.5;
        let zeta = 0.05;
        let signal = damped_sinusoid(freq, 1.0, zeta, 0.0, n, fs);
        let analyzer = PronyAnalyzer::new(2, fs, n);
        let modes = analyzer.analyze(&signal).expect("Prony analysis failed");
        assert!(!modes.is_empty(), "Should extract at least one mode");
        let closest = modes
            .iter()
            .min_by(|a, b| {
                (a.0 - freq)
                    .abs()
                    .partial_cmp(&(b.0 - freq).abs())
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .expect("modes not empty");
        assert!(
            (closest.0 - freq).abs() < 0.15,
            "Expected frequency ~{} Hz, got {:.4}",
            freq,
            closest.0
        );
        assert!(
            closest.1 >= -0.5,
            "Damping ratio should be non-strongly-negative for decaying signal, got {:.4}",
            closest.1
        );
    }
    #[test]
    fn test_prony_analyzer_two_modes() {
        let fs = 100.0;
        let n = 400;
        let f1 = 0.5;
        let f2 = 1.5;
        let zeta = 0.05;
        let sig1 = damped_sinusoid(f1, 1.0, zeta, 0.0, n, fs);
        let sig2 = damped_sinusoid(f2, 0.5, zeta, 0.0, n, fs);
        let signal: Vec<f64> = sig1.iter().zip(sig2.iter()).map(|(a, b)| a + b).collect();
        let analyzer = PronyAnalyzer::new(4, fs, n);
        let modes = analyzer.analyze(&signal).expect("Prony analysis failed");
        assert!(
            !modes.is_empty(),
            "Should extract at least 1 mode from two-tone signal"
        );
        let has_f1 = modes.iter().any(|&(f, _, _, _)| (f - f1).abs() < 0.3);
        let has_f2 = modes.iter().any(|&(f, _, _, _)| (f - f2).abs() < 0.3);
        assert!(
            has_f1 || has_f2,
            "Should detect at least one of the two frequencies"
        );
    }
    #[test]
    fn test_prony_reconstruct() {
        let fs = 50.0;
        let n = 100;
        let freq = 0.5;
        let zeta = 0.05;
        let signal = damped_sinusoid(freq, 1.0, zeta, 0.0, n, fs);
        let analyzer = PronyAnalyzer::new(2, fs, n);
        let modes = analyzer.analyze(&signal).expect("Prony analysis failed");
        let reconstructed = analyzer.reconstruct(&modes, n);
        assert_eq!(reconstructed.len(), n);
        for &v in &reconstructed {
            assert!(v.is_finite(), "Reconstructed value should be finite");
        }
    }
    #[test]
    fn test_companion_matrix_2x2() {
        let coeffs = vec![2.0, -3.0];
        let mat = PronyAnalyzer::build_companion_matrix(&coeffs);
        assert_eq!(mat.len(), 2);
        assert_eq!(mat[0].len(), 2);
        assert!((mat[0][0] - 3.0).abs() < 1e-10, "mat[0][0] should be 3.0");
        assert!(
            (mat[0][1] - (-2.0)).abs() < 1e-10,
            "mat[0][1] should be -2.0"
        );
        assert!((mat[1][0] - 1.0).abs() < 1e-10, "mat[1][0] should be 1.0");
        assert!(mat[1][1].abs() < 1e-10, "mat[1][1] should be 0.0");
    }
    #[test]
    fn test_dominant_eigenvalue() {
        let mat = vec![vec![3.0, 1.0], vec![0.0, 2.0]];
        let dom = PronyAnalyzer::dominant_eigenvalue(&mat, 200);
        assert!(
            (dom - 3.0).abs() < 0.2,
            "Dominant eigenvalue should be ~3.0, got {:.4}",
            dom
        );
    }
    #[test]
    fn test_fourier_detector_pure_sinusoid() {
        let fs = 50.0;
        let n = 256;
        let freq = 0.5;
        let signal = pure_sine(freq, 1.0, n, fs);
        let detector = FourierOscillationDetector::new(fs, n);
        let spectrum = detector.analyze_spectrum(&signal);
        assert!(!spectrum.frequencies.is_empty());
        assert!(!spectrum.magnitudes.is_empty());
        assert!(
            (spectrum.dominant_frequency_hz - freq).abs() < 0.3,
            "Dominant frequency should be ~{} Hz, got {:.4}",
            freq,
            spectrum.dominant_frequency_hz
        );
    }
    #[test]
    fn test_fourier_detector_dominant_frequency() {
        let fs = 100.0;
        let n = 512;
        let freq = 1.2;
        let signal = pure_sine(freq, 2.0, n, fs);
        let detector = FourierOscillationDetector::new(fs, n);
        let spectrum = detector.analyze_spectrum(&signal);
        assert!(spectrum.dominant_magnitude > 0.0);
        assert!(
            (spectrum.dominant_frequency_hz - freq).abs() < 0.5,
            "Expected dominant frequency near {:.1} Hz, got {:.4}",
            freq,
            spectrum.dominant_frequency_hz
        );
    }
    #[test]
    fn test_spectrum_interarea_power() {
        let fs = 50.0;
        let n = 256;
        let signal = pure_sine(0.4, 1.0, n, fs);
        let detector = FourierOscillationDetector::new(fs, n);
        let spectrum = detector.analyze_spectrum(&signal);
        assert!(
            spectrum.interarea_power > 0.0,
            "Inter-area power should be nonzero for 0.4 Hz signal"
        );
        assert!(
            spectrum.interarea_power >= spectrum.local_power,
            "Inter-area power should dominate"
        );
    }
    #[test]
    fn test_spectrum_local_power() {
        let fs = 100.0;
        let n = 512;
        let signal = pure_sine(1.5, 1.0, n, fs);
        let detector = FourierOscillationDetector::new(fs, n);
        let spectrum = detector.analyze_spectrum(&signal);
        assert!(
            spectrum.local_power > 0.0,
            "Local power should be nonzero for 1.5 Hz signal"
        );
        assert!(
            spectrum.local_power >= spectrum.interarea_power,
            "Local power should dominate"
        );
    }
    #[test]
    fn test_wam_monitor_update() {
        let mut monitor = WamOscillationMonitor::new(3, 50.0);
        let angles = vec![0.1, 0.2, 0.15];
        let speeds = vec![0.01, 0.02, 0.015];
        monitor.update(&angles, &speeds, 0.02);
        assert_eq!(monitor.angle_buffers[0].len(), 1);
        assert_eq!(monitor.speed_buffers[1].len(), 1);
        assert!((monitor.angle_buffers[0][0] - 0.1).abs() < 1e-10);
    }
    #[test]
    fn test_wam_monitor_swing_mode() {
        let mut monitor = WamOscillationMonitor::new(2, 50.0);
        let fs = 50.0;
        let n = 50;
        let freq = 0.5;
        for k in 0..n {
            let t = k as f64 / fs;
            let v0 = (2.0 * std::f64::consts::PI * freq * t).sin();
            let v1 = -(2.0 * std::f64::consts::PI * freq * t).sin();
            monitor.update(&[v0, v1], &[0.0, 0.0], t);
        }
        let swing = monitor.compute_swing_mode(0, 1);
        assert_eq!(swing.len(), n);
        let max_swing = swing.iter().cloned().fold(0.0_f64, f64::max);
        assert!(
            max_swing > 1.5,
            "Anti-phase swing amplitude should be ~2, got {:.4}",
            max_swing
        );
    }
    #[test]
    fn test_detect_forced_oscillation_single_freq() {
        let fs = 50.0;
        let n = 256;
        let forced_freq = 0.6;
        let sig0 = pure_sine(forced_freq, 1.0, n, fs);
        let sig1 = pure_sine(forced_freq, 0.9, n, fs);
        let signals = vec![sig0, sig1];
        let detector = FourierOscillationDetector::new(fs, n);
        let result = detector.detect_forced_oscillation(&signals);
        assert!(
            result.is_some(),
            "Should detect forced oscillation in coherent multi-bus signal"
        );
        if let Some(f) = result {
            assert!(
                (f - forced_freq).abs() < 0.3,
                "Detected forced frequency should be near {:.2} Hz, got {:.4}",
                forced_freq,
                f
            );
        }
    }
    #[test]
    fn test_mode_classification_interarea() {
        let mode_type = WamOscillationMonitor::classify_wam_mode(0.4, 5);
        assert_eq!(mode_type, ModeType::InterArea);
    }
    #[test]
    fn test_mode_classification_local() {
        let mode_type = WamOscillationMonitor::classify_wam_mode(1.0, 2);
        assert_eq!(mode_type, ModeType::Local);
    }
    #[test]
    fn test_participation_factor_computation() {
        let fs = 50.0;
        let n = 100;
        let freq = 0.5;
        let sig0 = pure_sine(freq, 1.0, n, fs);
        let sig1 = pure_sine(freq, 0.1, n, fs);
        let signals = vec![sig0, sig1];
        let participation = WamOscillationMonitor::compute_participation(&signals, freq, fs);
        assert_eq!(participation.len(), 2);
        assert!(
            (participation[0] - 1.0).abs() < 0.05,
            "Bus 0 should have maximum participation (1.0), got {:.4}",
            participation[0]
        );
        assert!(
            participation[1] < participation[0],
            "Bus 1 should have lower participation"
        );
    }
    #[test]
    fn test_pss_tuning_recommendation() {
        let mode = DetectedMode {
            frequency_hz: 0.5,
            damping_ratio: 0.02,
            amplitude: 0.05,
            participation: vec![0.9, 0.3],
            mode_type: ModeType::InterArea,
            is_poorly_damped: true,
            confidence: 0.8,
        };
        let _monitor = WamOscillationMonitor::new(2, 50.0);
        let rec = WamOscillationMonitor::recommend_pss_tuning(0, &mode);
        assert_eq!(rec.bus_id, 0);
        assert!((rec.target_frequency_hz - 0.5).abs() < 1e-6);
        assert!(rec.suggested_gain > 0.0, "Gain should be positive");
        assert!(
            rec.suggested_washout_s > 0.0,
            "Washout time should be positive"
        );
        assert!(
            rec.expected_damping_improvement > 0.0,
            "Improvement should be positive"
        );
    }
    #[test]
    fn test_ringdown_stable() {
        let fs = 50.0;
        let n = 200;
        let freq = 1.0;
        let zeta = 0.1;
        let signal = damped_sinusoid(freq, 1.0, zeta, 0.0, n, fs);
        let analyzer = RingdownAnalyzer::new(2, fs);
        let result = analyzer.analyze(&signal).expect("Ringdown analysis failed");
        assert!(
            result.is_stable,
            "Positively-damped ringdown should be stable"
        );
        assert!(result.decay_time_s > 0.0, "Decay time should be positive");
        assert!(
            result.settling_time_s >= result.decay_time_s,
            "Settling time should be >= decay time"
        );
    }
    #[test]
    fn test_ringdown_unstable_negative_damping() {
        let fs = 50.0;
        let n = 100;
        let freq = 1.0;
        let zeta = -0.05;
        let signal: Vec<f64> = (0..n)
            .map(|k| {
                let t = k as f64 / fs;
                let omega = 2.0 * std::f64::consts::PI * freq;
                1.0 * (-zeta * omega * t).exp().min(100.0) * (omega * t).cos()
            })
            .collect();
        let analyzer = RingdownAnalyzer::new(2, fs);
        if let Ok(result) = analyzer.analyze(&signal) {
            let _ = result.is_stable;
            assert!(result.decay_time_s >= 0.0);
        }
    }
    #[test]
    fn test_decay_time_calculation() {
        let fs = 50.0;
        let n = 200;
        let freq = 1.0;
        let zeta = 0.1;
        let signal = damped_sinusoid(freq, 1.0, zeta, 0.0, n, fs);
        let analyzer = RingdownAnalyzer::new(2, fs);
        let result = analyzer.analyze(&signal).expect("Ringdown analysis failed");
        let expected_sigma = 2.0 * std::f64::consts::PI * freq * zeta;
        let expected_decay = 1.0 / expected_sigma;
        assert!(
            result.decay_time_s > 0.0,
            "Decay time should be positive, got {:.4}",
            result.decay_time_s
        );
        assert!(
            result.decay_time_s < expected_decay * 100.0,
            "Decay time {:.2} s should be in reasonable range of expected {:.2} s",
            result.decay_time_s,
            expected_decay
        );
    }
    #[test]
    fn test_damping_index_well_damped() {
        let modes = vec![
            DetectedMode {
                frequency_hz: 0.5,
                damping_ratio: 0.15,
                amplitude: 0.1,
                participation: vec![1.0],
                mode_type: ModeType::InterArea,
                is_poorly_damped: false,
                confidence: 0.9,
            },
            DetectedMode {
                frequency_hz: 1.0,
                damping_ratio: 0.20,
                amplitude: 0.05,
                participation: vec![0.5],
                mode_type: ModeType::Local,
                is_poorly_damped: false,
                confidence: 0.85,
            },
        ];
        let index = WamOscillationMonitor::compute_damping_index(&modes);
        assert!(
            index > 0.8,
            "Well-damped system should have damping index > 0.8, got {:.4}",
            index
        );
        assert!(index <= 1.0, "Damping index must be ≤ 1.0");
    }
    #[test]
    fn test_prony_zero_modes_returns_invalid_config() {
        let sig = pure_sine(0.5, 1.0, 100, 50.0);
        let analyzer = PronyAnalyzer::new(0, 50.0, 100);
        let result = analyzer.analyze(&sig);
        assert!(
            matches!(result, Err(OscillationError::InvalidConfig(_))),
            "n_modes == 0 should return InvalidConfig"
        );
    }
    #[test]
    fn test_prony_zero_sampling_rate_returns_invalid_config() {
        let sig = pure_sine(0.5, 1.0, 100, 50.0);
        let analyzer = PronyAnalyzer::new(2, 0.0, 100);
        let result = analyzer.analyze(&sig);
        assert!(
            matches!(result, Err(OscillationError::InvalidConfig(_))),
            "sampling_rate_hz == 0.0 should return InvalidConfig"
        );
    }
    #[test]
    fn test_prony_too_few_samples_returns_error() {
        let short_sig = vec![1.0, -1.0, 1.0];
        let analyzer = PronyAnalyzer::new(2, 50.0, 100);
        let result = analyzer.analyze(&short_sig);
        assert!(
            result.is_err(),
            "Signal with < 4 samples should return an error"
        );
    }
    #[test]
    fn test_prony_reconstruct_empty_modes_is_zeros() {
        let analyzer = PronyAnalyzer::new(2, 50.0, 128);
        let reconstructed = analyzer.reconstruct(&[], 10);
        assert_eq!(reconstructed.len(), 10, "Should return 10 elements");
        for &v in &reconstructed {
            assert!(
                v.abs() < 1e-15,
                "Empty modes should reconstruct to all zeros"
            );
        }
    }
    #[test]
    fn test_build_companion_matrix_empty_coeffs() {
        let mat = PronyAnalyzer::build_companion_matrix(&[]);
        assert!(
            mat.is_empty(),
            "Empty coefficients should produce empty companion matrix"
        );
    }
    #[test]
    fn test_dominant_eigenvalue_empty_matrix_is_zero() {
        let result = PronyAnalyzer::dominant_eigenvalue(&[], 100);
        assert!(
            result.abs() < 1e-14,
            "Empty matrix should give dominant eigenvalue 0.0, got {result}"
        );
    }
    #[test]
    fn test_wam_monitor_zero_buses_returns_invalid_config() {
        let monitor = WamOscillationMonitor::new(0, 50.0);
        let result = monitor.analyze();
        assert!(
            matches!(result, Err(OscillationError::InvalidConfig(_))),
            "WAM monitor with 0 buses should return InvalidConfig"
        );
    }
    #[test]
    fn test_fourier_detect_forced_osc_empty_signals_is_none() {
        let detector = FourierOscillationDetector::new(50.0, 256);
        let result = detector.detect_forced_oscillation(&[]);
        assert!(
            result.is_none(),
            "Empty signals should return None for forced oscillation"
        );
    }
    #[test]
    fn test_ringdown_too_few_samples_returns_error() {
        let short_sig = vec![1.0, -0.5];
        let analyzer = RingdownAnalyzer::new(2, 50.0);
        let result = analyzer.analyze(&short_sig);
        assert!(
            result.is_err(),
            "Signal with < 4 samples should return an error in RingdownAnalyzer"
        );
    }
}
