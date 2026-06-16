//! Voltage sag/swell detection and characterisation per IEEE 1159-2019.
//!
//! ## Event taxonomy (IEEE 1159-2019, Table 1)
//!
//! | Event type             | Magnitude (pu) | Duration               |
//! |------------------------|----------------|------------------------|
//! | Instantaneous sag      | 0.1 – 0.9      | 0.5 – 30 cycles        |
//! | Momentary sag          | 0.1 – 0.9      | 30 cycles – 3 seconds  |
//! | Temporary sag          | 0.1 – 0.9      | 3 s – 1 min            |
//! | Instantaneous swell    | 1.1 – 1.8      | 0.5 – 30 cycles        |
//! | Momentary swell        | 1.1 – 1.8      | 30 cycles – 3 seconds  |
//! | Temporary swell        | 1.1 – 1.8      | 3 s – 1 min            |
//! | Interruption           | < 0.1          | any (classified below) |
//! | Undervoltage           | 0.8 – 0.9      | > 1 min (steady-state) |
//! | Overvoltage            | 1.1 – 1.2      | > 1 min (steady-state) |
//!
//! ## SEMI F47 ride-through envelope
//!
//! | Duration            | Min retained voltage |
//! |---------------------|----------------------|
//! | 0 – 20 ms           | 0 pu (any sag)       |
//! | 20 ms – 0.5 s       | 0.50 pu              |
//! | 0.5 s – 10 s        | 0.70 pu              |
//! | > 10 s              | 0.80 pu              |
//!
//! ## ITIC curve
//!
//! The ITIC (Information Technology Industry Council) curve defines an
//! acceptable region for power disturbances.  The implementation here
//! encodes the piecewise-linear boundary as look-up segments.

use serde::{Deserialize, Serialize};

// ─────────────────────────────────────────────────────────────────────────────
// Public types
// ─────────────────────────────────────────────────────────────────────────────

/// IEEE 1159-2019 voltage-event classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum VoltageEventType {
    /// 0.1 – 0.9 pu, 0.5 – 30 cycles.
    InstantaneousSag,
    /// 0.1 – 0.9 pu, 30 cycles – 3 s.
    MomentarySag,
    /// 0.1 – 0.9 pu, 3 s – 1 min.
    TemporarySag,
    /// 1.1 – 1.8 pu, 0.5 – 30 cycles.
    InstantaneousSwell,
    /// 1.1 – 1.8 pu, 30 cycles – 3 s.
    MomentarySwell,
    /// 1.1 – 1.8 pu, 3 s – 1 min.
    TemporarySwell,
    /// < 0.1 pu, any duration (IEC: < 3 min = short, else long).
    Interruption,
    /// 0.8 – 0.9 pu, > 1 min.
    Undervoltage,
    /// 1.1 – 1.2 pu, > 1 min.
    Overvoltage,
    /// Within normal operating range.
    Normal,
}

/// A fully characterised voltage event.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VoltageEvent {
    /// IEEE 1159 event type.
    pub event_type: VoltageEventType,
    /// Index of first abnormal sample.
    pub start_sample: usize,
    /// Index of last abnormal sample (inclusive).
    pub end_sample: usize,
    /// Duration expressed in power-frequency cycles.
    pub duration_cycles: f64,
    /// Characteristic magnitude \[pu\]: minimum for sags/interruptions,
    /// maximum for swells.
    pub magnitude_pu: f64,
    /// Retained voltage \[pu\] — equivalent to `magnitude_pu`.
    pub retained_voltage: f64,
    /// Voltage-sag energy [pu²·s], SEMI F47 / IEC 61000-4-11 metric:
    /// `Σ (1 − v²) · Δt` summed over the event window.
    pub energy_absorbed: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Half-cycle RMS envelope
// ─────────────────────────────────────────────────────────────────────────────

/// Compute the half-cycle RMS envelope from an instantaneous waveform.
///
/// Each output sample is the RMS of one half-cycle window of the input.  The
/// window is stepped by one half-cycle, so the output length is
/// `⌊samples * 2 * nominal_freq_hz / sample_rate_hz⌋`.
///
/// # Arguments
/// * `waveform`         — instantaneous voltage (pu)
/// * `sample_rate_hz`   — sampling rate \[Hz\]
/// * `nominal_freq_hz`  — power-frequency \[Hz\] (50 or 60)
///
/// # Returns
/// Per-unit RMS values, one per half-cycle.
pub fn half_cycle_rms(waveform: &[f64], sample_rate_hz: f64, nominal_freq_hz: f64) -> Vec<f64> {
    if waveform.is_empty() || sample_rate_hz <= 0.0 || nominal_freq_hz <= 0.0 {
        return vec![];
    }

    let half_cycle_samples = (sample_rate_hz / (2.0 * nominal_freq_hz)).round() as usize;
    if half_cycle_samples == 0 {
        return vec![];
    }

    let n_windows = waveform.len() / half_cycle_samples;
    let mut rms_envelope = Vec::with_capacity(n_windows);

    for w in 0..n_windows {
        let start = w * half_cycle_samples;
        let end = start + half_cycle_samples;
        let window = &waveform[start..end];
        let mean_sq = window.iter().map(|&v| v * v).sum::<f64>() / half_cycle_samples as f64;
        rms_envelope.push(mean_sq.sqrt());
    }

    rms_envelope
}

// ─────────────────────────────────────────────────────────────────────────────
// Event detection
// ─────────────────────────────────────────────────────────────────────────────

/// Detect IEEE 1159-2019 voltage events from a per-unit RMS envelope.
///
/// The function scans the envelope for contiguous regions that deviate from the
/// nominal (1.0 pu) beyond the supplied thresholds and classifies each region
/// according to its magnitude and duration.
///
/// # Arguments
/// * `v_rms_pu`               — per-unit RMS envelope (half-cycle samples)
/// * `sample_rate_hz`         — of the original waveform \[Hz\]
/// * `nominal_freq_hz`        — power frequency \[Hz\]
/// * `threshold_sag`          — lower boundary of normal band (default 0.9)
/// * `threshold_swell`        — upper boundary of normal band (default 1.1)
/// * `threshold_interruption` — below this → interruption (default 0.1)
pub fn detect_voltage_events(
    v_rms_pu: &[f64],
    sample_rate_hz: f64,
    nominal_freq_hz: f64,
    threshold_sag: f64,
    threshold_swell: f64,
    threshold_interruption: f64,
) -> Vec<VoltageEvent> {
    if v_rms_pu.is_empty() || sample_rate_hz <= 0.0 || nominal_freq_hz <= 0.0 {
        return vec![];
    }

    // Each half-cycle envelope sample corresponds to one half-cycle in time.
    // Duration in cycles = number_of_half_cycle_samples / 2.
    let half_cycle_samples = (sample_rate_hz / (2.0 * nominal_freq_hz)).round() as usize;
    let half_cycle_s = 1.0 / (2.0 * nominal_freq_hz);

    let mut events = Vec::new();
    let mut i = 0;

    while i < v_rms_pu.len() {
        let v = v_rms_pu[i];

        let is_sag = v < threshold_sag;
        let is_swell = v > threshold_swell;

        if !is_sag && !is_swell {
            i += 1;
            continue;
        }

        // Find the end of this contiguous anomaly.
        let start = i;
        let mut char_v = v; // characteristic voltage

        while i < v_rms_pu.len() {
            let vi = v_rms_pu[i];
            let still_anomaly = if is_sag {
                vi < threshold_sag
            } else {
                vi > threshold_swell
            };
            if !still_anomaly {
                break;
            }
            if is_sag {
                // For sags track the minimum retained voltage.
                if vi < char_v {
                    char_v = vi;
                }
            } else {
                // For swells track the maximum.
                if vi > char_v {
                    char_v = vi;
                }
            }
            i += 1;
        }

        let end = i - 1; // last anomalous sample index
        let n_env_samples = end - start + 1; // in half-cycle envelope samples

        // Duration in seconds and cycles.
        let duration_s = n_env_samples as f64 * half_cycle_s;
        let duration_cycles = n_env_samples as f64 * 0.5; // each env sample = ½ cycle

        // Voltage sag energy: Σ(1 − v²)·Δt (pu²·s).
        let energy_absorbed: f64 = v_rms_pu[start..=end]
            .iter()
            .map(|&vi| {
                let vi_clamped = vi.clamp(0.0, 1.0);
                (1.0 - vi_clamped * vi_clamped) * half_cycle_s
            })
            .sum();

        let event_type = if is_sag {
            classify_sag(char_v, duration_s, duration_cycles, threshold_interruption)
        } else {
            classify_swell(char_v, duration_s, duration_cycles)
        };

        // Convert envelope sample indices back to original waveform samples.
        let start_sample = start * half_cycle_samples;
        let end_sample = (end + 1) * half_cycle_samples - 1;

        events.push(VoltageEvent {
            event_type,
            start_sample,
            end_sample,
            duration_cycles,
            magnitude_pu: char_v,
            retained_voltage: char_v,
            energy_absorbed,
        });
    }

    events
}

// ─────────────────────────────────────────────────────────────────────────────
// Internal classifiers
// ─────────────────────────────────────────────────────────────────────────────

fn classify_sag(
    min_v: f64,
    duration_s: f64,
    duration_cycles: f64,
    threshold_interruption: f64,
) -> VoltageEventType {
    if min_v < threshold_interruption {
        return VoltageEventType::Interruption;
    }

    // Duration boundaries.
    const INSTANTANEOUS_MAX_CYCLES: f64 = 30.0;
    const MOMENTARY_MAX_S: f64 = 3.0;
    const TEMPORARY_MAX_S: f64 = 60.0;

    if duration_cycles <= 0.5 {
        // Below half-cycle threshold: treat as instantaneous
        VoltageEventType::InstantaneousSag
    } else if duration_cycles <= INSTANTANEOUS_MAX_CYCLES {
        VoltageEventType::InstantaneousSag
    } else if duration_s <= MOMENTARY_MAX_S {
        VoltageEventType::MomentarySag
    } else if duration_s <= TEMPORARY_MAX_S {
        VoltageEventType::TemporarySag
    } else {
        // > 1 min and 0.8–0.9 pu → undervoltage
        if (0.8..=0.9).contains(&min_v) {
            VoltageEventType::Undervoltage
        } else {
            VoltageEventType::TemporarySag
        }
    }
}

fn classify_swell(max_v: f64, duration_s: f64, duration_cycles: f64) -> VoltageEventType {
    const INSTANTANEOUS_MAX_CYCLES: f64 = 30.0;
    const MOMENTARY_MAX_S: f64 = 3.0;
    const TEMPORARY_MAX_S: f64 = 60.0;

    if duration_cycles <= INSTANTANEOUS_MAX_CYCLES {
        VoltageEventType::InstantaneousSwell
    } else if duration_s <= MOMENTARY_MAX_S {
        VoltageEventType::MomentarySwell
    } else if duration_s <= TEMPORARY_MAX_S {
        VoltageEventType::TemporarySwell
    } else {
        // > 1 min and 1.1–1.2 pu → overvoltage
        if (1.1..=1.2).contains(&max_v) {
            VoltageEventType::Overvoltage
        } else {
            VoltageEventType::TemporarySwell
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// ITIC curve
// ─────────────────────────────────────────────────────────────────────────────

/// Returns `true` when the event falls within the **acceptable** region of the
/// ITIC (Information Technology Industry Council) curve.
///
/// The curve is encoded as piecewise-linear segments on the (duration_ms,
/// voltage_pu) plane.  An event is acceptable if its retained voltage is above
/// the lower limit AND below the upper limit for its duration.
///
/// Reference: ITIC 2000 curve (updated from CBEMA 1987).
pub fn itic_compatible(event: &VoltageEvent, nominal_freq_hz: f64) -> bool {
    let duration_ms = event.duration_cycles / nominal_freq_hz * 1000.0;
    let v = event.retained_voltage;

    // Upper limit (overvoltage boundary)
    let upper = itic_upper_limit(duration_ms);
    // Lower limit (undervoltage / sag boundary)
    let lower = itic_lower_limit(duration_ms);

    v >= lower && v <= upper
}

/// ITIC upper boundary (overvoltage / swell limit).
fn itic_upper_limit(duration_ms: f64) -> f64 {
    // Piecewise linear: very short → very high tolerance, then decreases.
    if duration_ms <= 1.0 {
        f64::INFINITY // No upper limit for sub-microsecond or < 1 ms
    } else if duration_ms <= 3.0 {
        // Linear interpolation from ∞ to 2.0 pu at 3 ms is impractical;
        // use the ITIC specified value of 2.0 pu for the 1–3 ms region.
        2.0
    } else if duration_ms <= 20.0 {
        // 3 ms → 2.0 pu, 20 ms → 1.4 pu (linear)
        2.0 - (duration_ms - 3.0) / (20.0 - 3.0) * (2.0 - 1.4)
    } else if duration_ms <= 500.0 {
        // 20 ms → 1.4 pu, 500 ms → 1.2 pu
        1.4 - (duration_ms - 20.0) / (500.0 - 20.0) * (1.4 - 1.2)
    } else {
        // > 500 ms: 1.1 pu steady-state limit
        1.1
    }
}

/// ITIC lower boundary (sag / interruption limit).
///
/// Models the ITIC 2000 curve's "acceptable" lower voltage boundary:
///
/// | Duration       | Minimum voltage |
/// |----------------|-----------------|
/// | ≤ 8.33 ms      | 0 pu  (sub-half-cycle — no limit)     |
/// | 8.33 – 20 ms   | 0 – 0.7 pu (linear rise)              |
/// | 20 ms – 500 ms | 0.7 pu                                |
/// | 500 ms – 10 s  | 0.5 pu (ITIC no-damage region)        |
/// | > 10 s         | 0.9 pu (steady-state lower limit)     |
fn itic_lower_limit(duration_ms: f64) -> f64 {
    // Sub-half-cycle at 60 Hz (≈ 8.33 ms): no lower voltage limit.
    // This covers the "no-interruption" immune zone of the ITIC 2000 curve.
    let half_cycle_60hz_ms = 1000.0 / (2.0 * 60.0); // ≈ 8.333 ms
    if duration_ms <= half_cycle_60hz_ms {
        0.0
    } else if duration_ms <= 20.0 {
        // Linear rise from 0 at 8.33 ms to 0.7 at 20 ms.
        0.7 * (duration_ms - half_cycle_60hz_ms) / (20.0 - half_cycle_60hz_ms)
    } else if duration_ms <= 500.0 {
        // Flat 0.7 pu region (ITIC "acceptable" lower bound).
        0.7
    } else if duration_ms <= 10_000.0 {
        // ITIC no-damage region: 0.5 pu from 500 ms to 10 s.
        0.5
    } else {
        // Steady-state lower limit: 0.9 pu (EN 50160 compatible).
        0.9
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// SEMI F47
// ─────────────────────────────────────────────────────────────────────────────

/// Returns `true` when the event is within the SEMI F47 ride-through envelope.
///
/// SEMI F47-0706 defines minimum immunity requirements for semiconductor
/// fabrication equipment:
///
/// | Duration        | Minimum retained voltage |
/// |-----------------|--------------------------|
/// | 0 – 20 ms       | 0 pu (ride through any sag) |
/// | 20 ms – 200 ms  | 0.50 pu                  |
/// | 200 ms – 500 ms | 0.70 pu                  |
/// | 500 ms – 10 s   | 0.80 pu                  |
/// | > 10 s          | equipment may trip         |
///
/// An event is *compatible* if the retained voltage is above the minimum for
/// that duration.  Events lasting > 10 s are considered outside scope and
/// always return `false`.
pub fn semi_f47_compatible(event: &VoltageEvent, nominal_freq_hz: f64) -> bool {
    let duration_s = event.duration_cycles / nominal_freq_hz;
    let v = event.retained_voltage;

    if duration_s > 10.0 {
        // Beyond SEMI F47 ride-through window
        return false;
    }

    let min_v = semi_f47_min_voltage(duration_s);
    v >= min_v
}

fn semi_f47_min_voltage(duration_s: f64) -> f64 {
    let duration_ms = duration_s * 1000.0;
    if duration_ms <= 20.0 {
        0.0
    } else if duration_ms <= 200.0 {
        0.50
    } else if duration_ms <= 500.0 {
        0.70
    } else {
        0.80
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn pure_sine_pu(n_samples: usize, sample_rate_hz: f64, nominal_hz: f64) -> Vec<f64> {
        (0..n_samples)
            .map(|i| (2.0 * PI * nominal_hz * i as f64 / sample_rate_hz).sin())
            .collect()
    }

    #[test]
    fn test_half_cycle_rms_sinusoid() {
        // Pure sinusoid of amplitude 1 pu should yield RMS = 1/√2 ≈ 0.7071
        let fs = 10_000.0_f64;
        let f0 = 50.0_f64;
        let n = (fs * 10.0 / f0) as usize; // 10 full cycles
        let wave = pure_sine_pu(n, fs, f0);
        let rms = half_cycle_rms(&wave, fs, f0);
        assert!(!rms.is_empty(), "RMS envelope must not be empty");
        // Skip first sample (filter warm-up artefact)
        for &r in rms.iter().skip(1) {
            assert!(
                (r - 1.0_f64 / 2.0_f64.sqrt()).abs() < 0.01,
                "half-cycle RMS should be ~0.7071, got {r:.4}"
            );
        }
    }

    #[test]
    fn test_half_cycle_rms_empty() {
        assert!(half_cycle_rms(&[], 1000.0, 50.0).is_empty());
    }

    #[test]
    fn test_sag_detection_momentary() {
        // Construct an RMS envelope: 1 pu for 20 samples, then 0.7 pu for 70
        // samples (each = ½ cycle at 50 Hz → 35 cycles → MomentarySag),
        // then 1 pu for 20 samples.
        let mut env = vec![1.0_f64; 20];
        env.extend(vec![0.7; 70]); // 70 half-cycles = 35 cycles
        env.extend(vec![1.0; 20]);

        let events = detect_voltage_events(&env, 10_000.0, 50.0, 0.9, 1.1, 0.1);
        assert_eq!(events.len(), 1, "Expected exactly one sag event");
        let ev = &events[0];
        assert_eq!(
            ev.event_type,
            VoltageEventType::MomentarySag,
            "Expected MomentarySag, got {:?}",
            ev.event_type
        );
        assert!(
            (ev.magnitude_pu - 0.7).abs() < 1e-9,
            "magnitude should be 0.7"
        );
    }

    #[test]
    fn test_swell_detection() {
        let mut env = vec![1.0_f64; 20];
        env.extend(vec![1.25; 50]); // 25 cycles → InstantaneousSwell
        env.extend(vec![1.0; 20]);

        let events = detect_voltage_events(&env, 10_000.0, 50.0, 0.9, 1.1, 0.1);
        assert_eq!(events.len(), 1);
        let ev = &events[0];
        assert_eq!(
            ev.event_type,
            VoltageEventType::InstantaneousSwell,
            "Expected InstantaneousSwell, got {:?}",
            ev.event_type
        );
        assert!((ev.magnitude_pu - 1.25).abs() < 1e-9);
    }

    #[test]
    fn test_interruption_detection() {
        let mut env = vec![1.0_f64; 20];
        env.extend(vec![0.05; 40]); // below 0.1 → interruption
        env.extend(vec![1.0; 20]);

        let events = detect_voltage_events(&env, 10_000.0, 50.0, 0.9, 1.1, 0.1);
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].event_type, VoltageEventType::Interruption);
    }

    #[test]
    fn test_itic_normal_voltage() {
        // Normal operation (1.0 pu for many cycles) should be ITIC compatible.
        let event = VoltageEvent {
            event_type: VoltageEventType::Normal,
            start_sample: 0,
            end_sample: 1000,
            duration_cycles: 100.0,
            magnitude_pu: 1.0,
            retained_voltage: 1.0,
            energy_absorbed: 0.0,
        };
        assert!(itic_compatible(&event, 50.0));
    }

    #[test]
    fn test_itic_deep_short_sag_acceptable() {
        // Sub-cycle sag (0.5 cycles = ~8.3 ms at 60 Hz) — ITIC allows 0 pu
        // for very short durations.
        let event = VoltageEvent {
            event_type: VoltageEventType::InstantaneousSag,
            start_sample: 0,
            end_sample: 50,
            duration_cycles: 0.5,
            magnitude_pu: 0.0,
            retained_voltage: 0.0,
            energy_absorbed: 0.01,
        };
        assert!(
            itic_compatible(&event, 60.0),
            "Sub-cycle zero-volt sag should be ITIC acceptable"
        );
    }

    #[test]
    fn test_semi_f47_deep_short_sag() {
        // 0 pu for 10 ms (0.5 cycles at 50 Hz) → within 20 ms window → compatible.
        let event = VoltageEvent {
            event_type: VoltageEventType::InstantaneousSag,
            start_sample: 0,
            end_sample: 100,
            duration_cycles: 0.5,
            magnitude_pu: 0.0,
            retained_voltage: 0.0,
            energy_absorbed: 0.01,
        };
        assert!(semi_f47_compatible(&event, 50.0));
    }

    #[test]
    fn test_semi_f47_fails_long_deep_sag() {
        // 0.4 pu for 300 ms (15 cycles at 50 Hz) → min required 0.70 pu → fail.
        let event = VoltageEvent {
            event_type: VoltageEventType::TemporarySag,
            start_sample: 0,
            end_sample: 15000,
            duration_cycles: 15.0,
            magnitude_pu: 0.4,
            retained_voltage: 0.4,
            energy_absorbed: 0.5,
        };
        assert!(!semi_f47_compatible(&event, 50.0));
    }

    #[test]
    fn test_energy_absorbed_non_negative() {
        let mut env = vec![1.0_f64; 10];
        env.extend(vec![0.6; 30]);
        env.extend(vec![1.0; 10]);
        let events = detect_voltage_events(&env, 10_000.0, 50.0, 0.9, 1.1, 0.1);
        for ev in &events {
            assert!(
                ev.energy_absorbed >= 0.0,
                "Energy absorbed must be non-negative"
            );
        }
    }

    #[test]
    fn test_sag_detection_simple() {
        // 5 normal bookend samples + 40 sag samples + 5 normal = 50 total
        // 40 half-cycles = 20 cycles <= 30 => InstantaneousSag
        let mut env = vec![1.0_f64; 5];
        env.extend(vec![0.7_f64; 40]);
        env.extend(vec![1.0_f64; 5]);
        let events = detect_voltage_events(&env, 10_000.0, 50.0, 0.9, 1.1, 0.1);
        assert!(!events.is_empty(), "Expected at least one sag event");
        assert_eq!(
            events[0].event_type,
            VoltageEventType::InstantaneousSag,
            "40 half-cycle sag (20 cycles) should be InstantaneousSag"
        );
    }

    #[test]
    fn test_swell_detection_simple() {
        // 5 normal + 20 swell samples + 5 normal
        // 20 half-cycles = 10 cycles <= 30 => InstantaneousSwell
        let mut env = vec![1.0_f64; 5];
        env.extend(vec![1.2_f64; 20]);
        env.extend(vec![1.0_f64; 5]);
        let events = detect_voltage_events(&env, 10_000.0, 50.0, 0.9, 1.1, 0.1);
        assert!(!events.is_empty(), "Expected at least one swell event");
        assert_eq!(
            events[0].event_type,
            VoltageEventType::InstantaneousSwell,
            "20 half-cycle swell (10 cycles) should be InstantaneousSwell"
        );
    }

    #[test]
    fn test_interruption_simple() {
        // voltage 0.05 pu < 0.1 threshold => Interruption
        let mut env = vec![1.0_f64; 5];
        env.extend(vec![0.05_f64; 10]);
        env.extend(vec![1.0_f64; 5]);
        let events = detect_voltage_events(&env, 10_000.0, 50.0, 0.9, 1.1, 0.1);
        assert!(!events.is_empty(), "Expected an interruption event");
        assert_eq!(
            events[0].event_type,
            VoltageEventType::Interruption,
            "Voltage 0.05 pu < 0.1 threshold should be Interruption"
        );
    }

    #[test]
    fn test_half_cycle_rms_constant_dc() {
        // RMS of a constant waveform of value c is c
        let constant_val = 0.6_f64;
        let waveform: Vec<f64> = vec![constant_val; 10_000];
        let sample_rate = 10_000.0_f64;
        let nominal_freq = 50.0_f64;
        let rms_vals = half_cycle_rms(&waveform, sample_rate, nominal_freq);
        assert!(!rms_vals.is_empty(), "RMS output should not be empty");
        for (i, &v) in rms_vals.iter().enumerate() {
            assert!(
                (v - constant_val).abs() < 1e-9,
                "RMS at index {} should be ~0.6, got {}",
                i,
                v
            );
        }
    }

    #[test]
    fn test_itic_acceptable_moderate_sag() {
        // 5 cycles at 60 Hz = 83.33ms, retained 0.75 pu
        // itic lower at 83.33ms (20–500ms) = 0.7, upper >> 1.0
        // 0.75 >= 0.7 => compatible
        let event = VoltageEvent {
            event_type: VoltageEventType::InstantaneousSag,
            start_sample: 0,
            end_sample: 9,
            duration_cycles: 5.0,
            magnitude_pu: 0.75,
            retained_voltage: 0.75,
            energy_absorbed: 0.0,
        };
        assert!(
            itic_compatible(&event, 60.0),
            "5-cycle sag at 0.75 pu (83.3ms) should be ITIC compatible"
        );
    }

    #[test]
    fn test_itic_prohibited_long_deep_sag() {
        // 600 cycles at 60 Hz = 10,000ms = 10s, retained 0.4 pu
        // itic lower at 10,000ms (500ms–10s) = 0.5
        // 0.4 < 0.5 => NOT compatible
        let event = VoltageEvent {
            event_type: VoltageEventType::TemporarySag,
            start_sample: 0,
            end_sample: 1199,
            duration_cycles: 600.0,
            magnitude_pu: 0.4,
            retained_voltage: 0.4,
            energy_absorbed: 0.0,
        };
        assert!(
            !itic_compatible(&event, 60.0),
            "600-cycle deep sag at 0.4 pu (10s) should NOT be ITIC compatible"
        );
    }

    #[test]
    fn test_semi_f47_passes_short_sag() {
        // 0.5 cycles at 50 Hz = 10ms < 20ms threshold => min required = 0.0
        // retained 0.0 pu >= 0.0 => passes
        let event = VoltageEvent {
            event_type: VoltageEventType::InstantaneousSag,
            start_sample: 0,
            end_sample: 0,
            duration_cycles: 0.5,
            magnitude_pu: 0.0,
            retained_voltage: 0.0,
            energy_absorbed: 0.0,
        };
        assert!(
            semi_f47_compatible(&event, 50.0),
            "0.5-cycle interruption at 0.0 pu (10ms) should pass SEMI F47 (below 20ms threshold)"
        );
    }

    #[test]
    fn test_sag_boundary_exactly_09() {
        // voltage exactly 0.9 pu: is_sag = v < 0.9 => false (not < 0.9)
        // so no sag event should be detected
        let env = vec![0.9_f64; 50];
        let events = detect_voltage_events(&env, 10_000.0, 50.0, 0.9, 1.1, 0.1);
        assert!(
            events.is_empty(),
            "Voltage exactly at threshold 0.9 pu should not trigger a sag event"
        );
    }

    // ── New tests covering remaining requirements ──────────────────────────────

    #[test]
    fn test_sag_detection_instantaneous_below_09() {
        // 10 half-cycle samples at 0.7 pu (5 cycles) → InstantaneousSag
        let mut env = vec![1.0_f64; 10];
        env.extend(vec![0.7_f64; 10]); // 10 half-cycles = 5 cycles ≤ 30 → Instantaneous
        env.extend(vec![1.0_f64; 10]);
        let events = detect_voltage_events(&env, 10_000.0, 50.0, 0.9, 1.1, 0.1);
        assert!(
            !events.is_empty(),
            "Voltage 0.7 pu < 0.9 should trigger a sag event"
        );
        assert_eq!(
            events[0].event_type,
            VoltageEventType::InstantaneousSag,
            "5-cycle sag at 0.7 pu should be InstantaneousSag, got {:?}",
            events[0].event_type
        );
        assert!(
            events[0].magnitude_pu < 0.9,
            "Sag magnitude_pu must be < 0.9, got {}",
            events[0].magnitude_pu
        );
    }

    #[test]
    fn test_swell_detection_above_11() {
        // Swell at 1.15 pu for 6 half-cycle samples (3 cycles) → InstantaneousSwell
        let mut env = vec![1.0_f64; 10];
        env.extend(vec![1.15_f64; 6]); // 6 half-cycles = 3 cycles ≤ 30 → Instantaneous
        env.extend(vec![1.0_f64; 10]);
        let events = detect_voltage_events(&env, 10_000.0, 50.0, 0.9, 1.1, 0.1);
        assert!(
            !events.is_empty(),
            "Voltage 1.15 pu > 1.1 should trigger a swell event"
        );
        assert_eq!(
            events[0].event_type,
            VoltageEventType::InstantaneousSwell,
            "3-cycle swell at 1.15 pu should be InstantaneousSwell, got {:?}",
            events[0].event_type
        );
        assert!(
            events[0].magnitude_pu > 1.1,
            "Swell magnitude_pu must be > 1.1, got {}",
            events[0].magnitude_pu
        );
    }

    #[test]
    fn test_interruption_detection_below_01() {
        // Voltage 0.03 pu < 0.1 interruption threshold → Interruption
        let mut env = vec![1.0_f64; 10];
        env.extend(vec![0.03_f64; 8]); // below interruption threshold
        env.extend(vec![1.0_f64; 10]);
        let events = detect_voltage_events(&env, 10_000.0, 50.0, 0.9, 1.1, 0.1);
        assert!(
            !events.is_empty(),
            "Voltage 0.03 pu should trigger an interruption event"
        );
        assert_eq!(
            events[0].event_type,
            VoltageEventType::Interruption,
            "Voltage 0.03 pu < 0.1 threshold should be Interruption, got {:?}",
            events[0].event_type
        );
    }

    #[test]
    fn test_half_cycle_rms_all_ones() {
        // RMS of constant 1.0 waveform must be 1.0 per half-cycle window
        let waveform = vec![1.0_f64; 2000];
        let rms = half_cycle_rms(&waveform, 10_000.0, 50.0);
        assert!(
            !rms.is_empty(),
            "RMS of all-ones waveform should not be empty"
        );
        for (idx, &r) in rms.iter().enumerate() {
            assert!(
                (r - 1.0).abs() < 1e-12,
                "Half-cycle RMS at index {} should be 1.0, got {}",
                idx,
                r
            );
        }
    }

    #[test]
    fn test_itic_acceptable_region_normal_operation() {
        // 2 cycles at 60 Hz = 33.33 ms, retained 0.9 pu
        // itic_lower at 33.33 ms = 0.7 pu (flat region 20–500 ms)
        // 0.9 >= 0.7 and 0.9 <= 1.1 (upper at >500ms) → acceptable
        let event = VoltageEvent {
            event_type: VoltageEventType::InstantaneousSag,
            start_sample: 0,
            end_sample: 199,
            duration_cycles: 2.0,
            magnitude_pu: 0.9,
            retained_voltage: 0.9,
            energy_absorbed: 0.0,
        };
        assert!(
            itic_compatible(&event, 60.0),
            "2-cycle sag at 0.9 pu (33.33 ms) should be within ITIC acceptable region"
        );
    }

    #[test]
    fn test_semi_f47_boundary_200ms_passes_at_50pu() {
        // Duration exactly 200 ms at 50 Hz = 10 cycles
        // SEMI F47: 20–200 ms window requires ≥ 0.50 pu
        // retained = 0.50 pu → passes (boundary inclusive via >=)
        let event = VoltageEvent {
            event_type: VoltageEventType::InstantaneousSag,
            start_sample: 0,
            end_sample: 999,
            duration_cycles: 10.0,
            magnitude_pu: 0.50,
            retained_voltage: 0.50,
            energy_absorbed: 0.1,
        };
        assert!(
            semi_f47_compatible(&event, 50.0),
            "10-cycle sag at 0.5 pu (200 ms) should pass SEMI F47 (min = 0.50 at boundary)"
        );
    }

    #[test]
    fn test_event_duration_measurement() {
        // 20 half-cycle envelope samples = 10 cycles at 50 Hz
        let mut env = vec![1.0_f64; 5];
        env.extend(vec![0.6_f64; 20]); // 20 half-cycles = 10 cycles
        env.extend(vec![1.0_f64; 5]);
        let events = detect_voltage_events(&env, 10_000.0, 50.0, 0.9, 1.1, 0.1);
        assert_eq!(events.len(), 1, "Expected exactly one event");
        let ev = &events[0];
        assert!(
            (ev.duration_cycles - 10.0).abs() < 1e-9,
            "Duration should be 10.0 cycles (20 half-cycles), got {}",
            ev.duration_cycles
        );
    }

    #[test]
    fn test_swell_boundary_exactly_11_not_triggered() {
        // voltage exactly 1.1 pu: is_swell = v > 1.1 => false (not > 1.1)
        // no swell event should be detected
        let env = vec![1.1_f64; 50];
        let events = detect_voltage_events(&env, 10_000.0, 50.0, 0.9, 1.1, 0.1);
        assert!(
            events.is_empty(),
            "Voltage exactly at swell threshold 1.1 pu should not trigger a swell event"
        );
    }
}
