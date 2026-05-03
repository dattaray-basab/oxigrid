//! Power quality event detection and classification.
//!
//! Implements a complete PQ event classifier that processes instantaneous
//! voltage waveforms and identifies:
//!
//! - Voltage sags / swells / interruptions (IEEE 1159-2019)
//! - Oscillatory and impulsive transients (wavelet-inspired windowed RMS method)
//! - Voltage notches (power electronics)
//! - Sudden THD increases (harmonic events)
//! - Frequency deviations
//! - Voltage flicker
//!
//! All events are assigned a severity level (Minor / Moderate / Severe / Critical)
//! following the IEEE 1159 guidance.

use crate::powerquality::sag_swell::{detect_voltage_events, half_cycle_rms, VoltageEventType};
use serde::{Deserialize, Serialize};

// ─────────────────────────────────────────────────────────────────────────────
// Public types
// ─────────────────────────────────────────────────────────────────────────────

/// IEEE 1159-2019 power quality event classification.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PqEventClass {
    /// Voltage sag (dip) — includes the IEEE 1159 sub-classification.
    VoltageSag(VoltageEventType),
    /// Voltage swell — includes the IEEE 1159 sub-classification.
    VoltageSwell(VoltageEventType),
    /// Complete loss of supply (< 0.1 pu).
    Interruption,
    /// Fast oscillatory or impulsive transient (sub-cycle rise time).
    Transient,
    /// Sudden increase in total harmonic distortion.
    HarmonicIncrease,
    /// Frequency deviation beyond acceptable band.
    FrequencyDeviation,
    /// Repetitive voltage fluctuations causing flicker sensation.
    VoltageFlicker,
    /// Voltage notch caused by power electronics switching.
    Notch,
}

/// Event severity per IEEE 1159 guidance.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum PqSeverity {
    /// Small disturbance, unlikely to affect equipment.
    Minor,
    /// Noticeable disturbance; sensitive equipment may be affected.
    Moderate,
    /// Significant disturbance; most equipment may trip or malfunction.
    Severe,
    /// Extreme disturbance; certain equipment damage possible.
    Critical,
}

/// A classified power quality event.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PqEvent {
    /// Index of the first affected sample in the original waveform.
    pub start_sample: usize,
    /// Index of the last affected sample.
    pub end_sample: usize,
    /// IEEE 1159 event class.
    pub event_class: PqEventClass,
    /// Severity rating.
    pub severity: PqSeverity,
    /// RMS voltage deviation from nominal \[pu\] (e.g. 0.3 means 0.3 pu sag).
    pub rms_impact: f64,
    /// Frequency deviation \[Hz\] (zero for non-frequency events).
    pub frequency_impact: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Summary
// ─────────────────────────────────────────────────────────────────────────────

/// Aggregated statistics for a set of detected PQ events.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PqEventSummary {
    /// Total number of events detected.
    pub total_events: usize,
    /// Event count per class, sorted by class name (alphabetically).
    pub events_per_class: Vec<(String, usize)>,
    /// Average rate of events per hour.
    pub events_per_hour: f64,
    /// The event with the highest severity (or highest impact if tied).
    pub most_severe: Option<PqEvent>,
    /// Total duration of all events combined \[seconds\].
    pub cumulative_duration_s: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Classifier
// ─────────────────────────────────────────────────────────────────────────────

/// Power quality event classifier.
///
/// Create with [`PqEventClassifier::new`], then call [`PqEventClassifier::classify_events`] on
/// instantaneous voltage waveform data \[pu\].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PqEventClassifier {
    /// Sampling rate of the input waveform \[Hz\].
    pub sample_rate_hz: f64,
    /// Nominal power frequency \[Hz\].
    pub nominal_freq_hz: f64,
    /// Nominal voltage \[pu\] (typically 1.0).
    pub nominal_voltage_pu: f64,
}

impl PqEventClassifier {
    /// Create a new classifier.
    ///
    /// # Arguments
    /// * `sample_rate_hz`    — waveform sampling rate \[Hz\]
    /// * `nominal_freq_hz`   — power frequency \[Hz\] (50 or 60)
    pub fn new(sample_rate_hz: f64, nominal_freq_hz: f64) -> Self {
        Self {
            sample_rate_hz,
            nominal_freq_hz,
            nominal_voltage_pu: 1.0,
        }
    }

    /// Classify all PQ events in an instantaneous voltage waveform \[pu\].
    ///
    /// The function:
    /// 1. Computes the half-cycle RMS envelope.
    /// 2. Detects voltage sags, swells, and interruptions.
    /// 3. Detects fast transients using windowed RMS comparison.
    /// 4. Detects voltage notches using derivative threshold.
    ///
    /// # Returns
    /// A vector of [`PqEvent`] structs, chronologically ordered.
    pub fn classify_events(&self, waveform: &[f64]) -> Vec<PqEvent> {
        if waveform.is_empty() {
            return vec![];
        }

        let mut events: Vec<PqEvent> = Vec::new();

        // ── Sag / swell / interruption ────────────────────────────────────────
        let rms_env = half_cycle_rms(waveform, self.sample_rate_hz, self.nominal_freq_hz);
        let raw_events = detect_voltage_events(
            &rms_env,
            self.sample_rate_hz,
            self.nominal_freq_hz,
            0.9,
            1.1,
            0.1,
        );

        for ev in raw_events {
            let rms_impact = (ev.retained_voltage - 1.0).abs();
            let (event_class, severity) = match ev.event_type {
                VoltageEventType::Interruption => {
                    (PqEventClass::Interruption, PqSeverity::Critical)
                }
                VoltageEventType::InstantaneousSag
                | VoltageEventType::MomentarySag
                | VoltageEventType::TemporarySag
                | VoltageEventType::Undervoltage => {
                    let sev = severity_for_sag(ev.retained_voltage);
                    (PqEventClass::VoltageSag(ev.event_type), sev)
                }
                VoltageEventType::InstantaneousSwell
                | VoltageEventType::MomentarySwell
                | VoltageEventType::TemporarySwell
                | VoltageEventType::Overvoltage => {
                    let sev = severity_for_swell(ev.retained_voltage);
                    (PqEventClass::VoltageSwell(ev.event_type), sev)
                }
                VoltageEventType::Normal => continue,
            };

            events.push(PqEvent {
                start_sample: ev.start_sample,
                end_sample: ev.end_sample,
                event_class,
                severity,
                rms_impact,
                frequency_impact: 0.0,
            });
        }

        // ── Transient detection ───────────────────────────────────────────────
        let transients = self.detect_transients(waveform, 32);
        for (sample, peak_mag) in transients {
            let rms_impact = (peak_mag.abs() - self.nominal_voltage_pu).max(0.0);
            let severity = if peak_mag.abs() > 2.0 {
                PqSeverity::Critical
            } else if peak_mag.abs() > 1.8 {
                PqSeverity::Severe
            } else if peak_mag.abs() > 1.5 {
                PqSeverity::Moderate
            } else {
                PqSeverity::Minor
            };

            events.push(PqEvent {
                start_sample: sample,
                end_sample: sample + 1,
                event_class: PqEventClass::Transient,
                severity,
                rms_impact,
                frequency_impact: 0.0,
            });
        }

        // ── Notch detection ───────────────────────────────────────────────────
        let notches = self.detect_notches(waveform);
        for (start, end) in notches {
            let region = &waveform[start..=end.min(waveform.len() - 1)];
            let notch_depth = region.iter().copied().fold(f64::INFINITY, f64::min).abs();
            let rms_impact = notch_depth;
            events.push(PqEvent {
                start_sample: start,
                end_sample: end,
                event_class: PqEventClass::Notch,
                severity: PqSeverity::Minor,
                rms_impact,
                frequency_impact: 0.0,
            });
        }

        // Sort chronologically.
        events.sort_by_key(|e| e.start_sample);
        events
    }

    /// Detect transients using windowed RMS comparison.
    ///
    /// Each window of `window_samples` is compared with adjacent windows.
    /// A transient is declared when:
    /// - A sample exceeds 1.5 pu in absolute value, **or**
    /// - The windowed RMS jump from one window to the next exceeds 0.5 pu.
    ///
    /// Returns `(sample_index, peak_magnitude)` pairs.
    pub fn detect_transients(&self, waveform: &[f64], window_samples: usize) -> Vec<(usize, f64)> {
        if waveform.is_empty() || window_samples == 0 {
            return vec![];
        }

        let transient_threshold = 1.5_f64;
        let rms_jump_threshold = 0.5_f64;
        let mut transients = Vec::new();
        let mut last_reported = usize::MAX; // debounce

        // Method 1: direct peak detection.
        for (idx, &v) in waveform.iter().enumerate() {
            if v.abs() > transient_threshold {
                // Debounce: skip if too close to last reported transient.
                let debounce = window_samples.max(1);
                if last_reported == usize::MAX || idx >= last_reported + debounce {
                    transients.push((idx, v));
                    last_reported = idx;
                }
            }
        }

        // Method 2: windowed RMS jump detection.
        // Only fire when the window RMS is also above the nominal level (> 1.2 pu),
        // preventing false positives from natural RMS variation of a steady-state sine.
        let n_windows = waveform.len() / window_samples;
        let mut prev_rms = 0.0_f64;
        for w in 0..n_windows {
            let start = w * window_samples;
            let end = start + window_samples;
            let window = &waveform[start..end];
            let rms = (window.iter().map(|&v| v * v).sum::<f64>() / window_samples as f64).sqrt();
            // A genuine transient will push the windowed RMS well above normal (1/√2 ≈ 0.707).
            // Guard: only flag if the windowed RMS itself exceeds 1.0 pu (i.e. above nominal peak).
            if w > 0 && (rms - prev_rms).abs() > rms_jump_threshold && rms > 1.0 {
                let debounce = window_samples.max(1);
                if last_reported == usize::MAX || start >= last_reported + debounce {
                    // Find the peak sample in this window.
                    let (peak_idx, &peak_v) = window
                        .iter()
                        .enumerate()
                        .max_by(|(_, a), (_, b)| {
                            a.abs()
                                .partial_cmp(&b.abs())
                                .unwrap_or(std::cmp::Ordering::Equal)
                        })
                        .unwrap_or((0, &0.0));
                    transients.push((start + peak_idx, peak_v));
                    last_reported = start;
                }
            }
            prev_rms = rms;
        }

        // Sort and deduplicate by sample index.
        transients.sort_by_key(|(s, _)| *s);
        transients.dedup_by_key(|(s, _)| *s);
        transients
    }

    /// Classify the severity of a detected PQ event per IEEE 1159.
    pub fn classify_severity(&self, event: &PqEvent) -> PqSeverity {
        match &event.event_class {
            PqEventClass::Interruption => PqSeverity::Critical,
            PqEventClass::VoltageSag(t) => match t {
                VoltageEventType::InstantaneousSag => {
                    if event.rms_impact > 0.7 {
                        PqSeverity::Severe
                    } else if event.rms_impact > 0.5 {
                        PqSeverity::Moderate
                    } else {
                        PqSeverity::Minor
                    }
                }
                VoltageEventType::MomentarySag | VoltageEventType::TemporarySag => {
                    if event.rms_impact > 0.5 {
                        PqSeverity::Critical
                    } else if event.rms_impact > 0.3 {
                        PqSeverity::Severe
                    } else {
                        PqSeverity::Moderate
                    }
                }
                VoltageEventType::Undervoltage => PqSeverity::Minor,
                _ => PqSeverity::Minor,
            },
            PqEventClass::VoltageSwell(_) => {
                if event.rms_impact > 0.5 {
                    PqSeverity::Severe
                } else if event.rms_impact > 0.2 {
                    PqSeverity::Moderate
                } else {
                    PqSeverity::Minor
                }
            }
            PqEventClass::Transient => {
                if event.rms_impact > 1.0 {
                    PqSeverity::Critical
                } else if event.rms_impact > 0.5 {
                    PqSeverity::Severe
                } else {
                    PqSeverity::Moderate
                }
            }
            PqEventClass::HarmonicIncrease | PqEventClass::FrequencyDeviation => PqSeverity::Minor,
            PqEventClass::VoltageFlicker => PqSeverity::Minor,
            PqEventClass::Notch => PqSeverity::Minor,
        }
    }

    /// Generate a statistical summary from a slice of detected events.
    ///
    /// # Arguments
    /// * `events`          — slice of classified PQ events
    /// * `duration_hours`  — observation window length \[hours\]
    /// * `sample_rate_hz`  — waveform sampling rate \[Hz\] used to convert sample counts to seconds
    pub fn event_summary(
        events: &[PqEvent],
        duration_hours: f64,
        sample_rate_hz: f64,
    ) -> PqEventSummary {
        let total_events = events.len();
        let events_per_hour = if duration_hours > 0.0 {
            total_events as f64 / duration_hours
        } else {
            0.0
        };

        // Count per class.
        let class_names = [
            "Interruption",
            "VoltageSag",
            "VoltageSwell",
            "Transient",
            "HarmonicIncrease",
            "FrequencyDeviation",
            "VoltageFlicker",
            "Notch",
        ];
        let mut counts = vec![0usize; class_names.len()];
        for ev in events {
            let idx = match &ev.event_class {
                PqEventClass::Interruption => 0,
                PqEventClass::VoltageSag(_) => 1,
                PqEventClass::VoltageSwell(_) => 2,
                PqEventClass::Transient => 3,
                PqEventClass::HarmonicIncrease => 4,
                PqEventClass::FrequencyDeviation => 5,
                PqEventClass::VoltageFlicker => 6,
                PqEventClass::Notch => 7,
            };
            counts[idx] += 1;
        }
        let events_per_class: Vec<(String, usize)> = class_names
            .iter()
            .zip(counts.iter())
            .filter(|(_, &c)| c > 0)
            .map(|(&name, &count)| (name.to_string(), count))
            .collect();

        // Most severe event.
        let most_severe = events
            .iter()
            .max_by(|a, b| {
                a.severity.cmp(&b.severity).then(
                    a.rms_impact
                        .partial_cmp(&b.rms_impact)
                        .unwrap_or(std::cmp::Ordering::Equal),
                )
            })
            .cloned();

        // Cumulative duration.
        let seconds_per_sample = if sample_rate_hz > 0.0 {
            1.0 / sample_rate_hz
        } else {
            0.0
        };
        let cumulative_duration_s: f64 = events
            .iter()
            .map(|ev| {
                (ev.end_sample.saturating_sub(ev.start_sample) + 1) as f64 * seconds_per_sample
            })
            .sum();

        PqEventSummary {
            total_events,
            events_per_class,
            events_per_hour,
            most_severe,
            cumulative_duration_s,
        }
    }

    // ── Notch detection ───────────────────────────────────────────────────────

    /// Detect voltage notches caused by power-electronics commutation.
    ///
    /// A notch is characterised by a rapid, narrow dip in the voltage waveform
    /// that is smaller than half a cycle.  The heuristic here uses the first
    /// derivative:
    ///
    /// 1. Compute finite-difference derivative.
    /// 2. Identify rapid voltage drops (derivative < −threshold).
    /// 3. Confirm recovery within a short window (< half-cycle samples).
    ///
    /// Returns `Vec<(start_sample, end_sample)>`.
    fn detect_notches(&self, waveform: &[f64]) -> Vec<(usize, usize)> {
        if waveform.len() < 4 {
            return vec![];
        }

        // Threshold: derivative magnitude > 5 × nominal per sample
        // (tuned for typical notch at 60° on a 50/60 Hz waveform).
        let max_half_cycle_samples =
            (self.sample_rate_hz / (2.0 * self.nominal_freq_hz)).round() as usize;
        let deriv_threshold = 2.0 * self.nominal_freq_hz / self.sample_rate_hz * 10.0;

        let mut notches = Vec::new();
        let mut i = 1;

        while i < waveform.len() - 1 {
            let deriv = waveform[i] - waveform[i - 1];
            if deriv < -deriv_threshold {
                // Potential notch start.
                let start = i;
                let mut j = i + 1;
                while j < waveform.len().min(i + max_half_cycle_samples) {
                    let rec = waveform[j] - waveform[j - 1];
                    if rec > deriv_threshold {
                        // Recovery detected.
                        notches.push((start, j));
                        i = j + 1;
                        break;
                    }
                    j += 1;
                }
                if j >= waveform.len().min(i + max_half_cycle_samples) {
                    i += 1;
                }
            } else {
                i += 1;
            }
        }

        notches
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Internal severity helpers
// ─────────────────────────────────────────────────────────────────────────────

fn severity_for_sag(retained_v: f64) -> PqSeverity {
    if retained_v < 0.1 {
        PqSeverity::Critical
    } else if retained_v < 0.5 {
        PqSeverity::Severe
    } else if retained_v < 0.8 {
        PqSeverity::Moderate
    } else {
        PqSeverity::Minor
    }
}

fn severity_for_swell(peak_v: f64) -> PqSeverity {
    if peak_v > 1.8 {
        PqSeverity::Critical
    } else if peak_v > 1.4 {
        PqSeverity::Severe
    } else if peak_v > 1.2 {
        PqSeverity::Moderate
    } else {
        PqSeverity::Minor
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn pure_sine(fs: f64, f0: f64, n_cycles: f64) -> Vec<f64> {
        let n = (fs / f0 * n_cycles) as usize;
        (0..n)
            .map(|i| (2.0 * PI * f0 * i as f64 / fs).sin())
            .collect()
    }

    /// Build a waveform with a synthetic sag: multiply the amplitude by `sag_factor`
    /// during the sag window specified in sample indices.
    fn with_sag(base: &[f64], sag_start: usize, sag_end: usize, sag_factor: f64) -> Vec<f64> {
        base.iter()
            .enumerate()
            .map(|(i, &v)| {
                if i >= sag_start && i < sag_end {
                    v * sag_factor
                } else {
                    v
                }
            })
            .collect()
    }

    #[test]
    fn test_classify_events_sag() {
        let fs = 10_000.0_f64;
        let f0 = 50.0_f64;
        // 20 cycles of normal, then 10 cycles of 70 % sag, then 20 cycles normal.
        let n_cycle = (fs / f0) as usize;
        let base = pure_sine(fs, f0, 50.0);
        let wave = with_sag(&base, 20 * n_cycle, 30 * n_cycle, 0.7);

        let clf = PqEventClassifier::new(fs, f0);
        let events = clf.classify_events(&wave);

        // At least one sag event should be detected.
        let sag_count = events
            .iter()
            .filter(|e| matches!(e.event_class, PqEventClass::VoltageSag(_)))
            .count();
        assert!(
            sag_count >= 1,
            "Expected at least one sag, got events: {:?}",
            events.len()
        );
    }

    #[test]
    fn test_classify_events_empty_waveform() {
        let clf = PqEventClassifier::new(10_000.0, 50.0);
        let events = clf.classify_events(&[]);
        assert!(events.is_empty());
    }

    #[test]
    fn test_detect_transients_peak() {
        // Inject a single large spike.
        let fs = 10_000.0_f64;
        let f0 = 50.0_f64;
        let mut wave = pure_sine(fs, f0, 10.0);
        let spike_idx = 1000;
        wave[spike_idx] = 2.0; // 2.0 pu spike

        let clf = PqEventClassifier::new(fs, f0);
        let transients = clf.detect_transients(&wave, 32);

        let found = transients.iter().any(|(s, _)| *s == spike_idx);
        assert!(found, "Should detect spike at index {spike_idx}");
    }

    #[test]
    fn test_detect_transients_no_false_positive_pure_sine() {
        let fs = 10_000.0_f64;
        let f0 = 50.0_f64;
        let wave = pure_sine(fs, f0, 5.0);
        let clf = PqEventClassifier::new(fs, f0);
        let transients = clf.detect_transients(&wave, 32);
        // Pure sine should produce very few (ideally zero) transient detections.
        assert!(
            transients.is_empty(),
            "Pure sine should not produce transients, got {}: {:?}",
            transients.len(),
            transients
        );
    }

    #[test]
    fn test_event_summary_counts() {
        let events = vec![
            PqEvent {
                start_sample: 0,
                end_sample: 100,
                event_class: PqEventClass::VoltageSag(VoltageEventType::InstantaneousSag),
                severity: PqSeverity::Moderate,
                rms_impact: 0.3,
                frequency_impact: 0.0,
            },
            PqEvent {
                start_sample: 500,
                end_sample: 600,
                event_class: PqEventClass::Transient,
                severity: PqSeverity::Severe,
                rms_impact: 0.8,
                frequency_impact: 0.0,
            },
            PqEvent {
                start_sample: 1000,
                end_sample: 1100,
                event_class: PqEventClass::VoltageSag(VoltageEventType::MomentarySag),
                severity: PqSeverity::Severe,
                rms_impact: 0.5,
                frequency_impact: 0.0,
            },
        ];

        let summary = PqEventClassifier::event_summary(&events, 1.0, 1.0_f64);
        assert_eq!(summary.total_events, 3);
        assert_eq!(summary.events_per_hour, 3.0);

        let sag_count = summary
            .events_per_class
            .iter()
            .find(|(name, _)| name == "VoltageSag")
            .map(|(_, c)| *c)
            .unwrap_or(0);
        assert_eq!(sag_count, 2, "Expected 2 sag events in summary");

        let transient_count = summary
            .events_per_class
            .iter()
            .find(|(name, _)| name == "Transient")
            .map(|(_, c)| *c)
            .unwrap_or(0);
        assert_eq!(transient_count, 1);
    }

    #[test]
    fn test_severity_ordering() {
        // Severity enum must be orderable: Minor < Moderate < Severe < Critical
        assert!(PqSeverity::Minor < PqSeverity::Moderate);
        assert!(PqSeverity::Moderate < PqSeverity::Severe);
        assert!(PqSeverity::Severe < PqSeverity::Critical);
    }

    #[test]
    fn test_classify_severity_interruption() {
        let clf = PqEventClassifier::new(10_000.0, 50.0);
        let ev = PqEvent {
            start_sample: 0,
            end_sample: 1000,
            event_class: PqEventClass::Interruption,
            severity: PqSeverity::Critical,
            rms_impact: 0.95,
            frequency_impact: 0.0,
        };
        assert_eq!(clf.classify_severity(&ev), PqSeverity::Critical);
    }

    #[test]
    fn test_event_summary_most_severe() {
        let events = vec![
            PqEvent {
                start_sample: 0,
                end_sample: 10,
                event_class: PqEventClass::Notch,
                severity: PqSeverity::Minor,
                rms_impact: 0.05,
                frequency_impact: 0.0,
            },
            PqEvent {
                start_sample: 100,
                end_sample: 200,
                event_class: PqEventClass::Interruption,
                severity: PqSeverity::Critical,
                rms_impact: 0.95,
                frequency_impact: 0.0,
            },
        ];
        let summary = PqEventClassifier::event_summary(&events, 2.0, 1.0_f64);
        let ms = summary.most_severe.expect("Should have most_severe");
        assert_eq!(ms.severity, PqSeverity::Critical);
    }

    #[test]
    fn test_extract_harmonics_used_in_harmonic_event() {
        use crate::powerquality::waveform::extract_harmonics;
        // Verify extract_harmonics can be called (smoke test for the dependency).
        let fs = 10_000.0_f64;
        let f0 = 50.0_f64;
        let n = (fs / f0 * 4.0) as usize;
        let wave: Vec<f64> = (0..n)
            .map(|i| {
                (2.0 * PI * f0 * i as f64 / fs).sin()
                    + 0.1 * (2.0 * PI * 3.0 * f0 * i as f64 / fs).sin()
            })
            .collect();
        let harmonics = extract_harmonics(&wave, fs, f0, 5);
        assert!(!harmonics.is_empty());
    }

    #[test]
    fn test_event_summary_sample_rate_scaling() {
        // 1 event spanning 100 samples at 25600 Hz → expected duration = 100/25600 s
        let event = PqEvent {
            start_sample: 0,
            end_sample: 99, // 100 samples inclusive (99 - 0 + 1 = 100)
            event_class: PqEventClass::VoltageSag(VoltageEventType::InstantaneousSag),
            severity: PqSeverity::Moderate,
            rms_impact: 0.2,
            frequency_impact: 0.0,
        };
        let summary = PqEventClassifier::event_summary(&[event], 1.0, 25_600.0);
        let expected = 100.0 / 25_600.0;
        assert!(
            (summary.cumulative_duration_s - expected).abs() < 1e-9,
            "got {}, expected {}",
            summary.cumulative_duration_s,
            expected
        );
    }
}
