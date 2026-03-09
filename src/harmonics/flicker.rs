/// IEC 61000-4-15 Flickermeter and EN 50160 voltage quality analysis.
///
/// Implements:
/// - IEC 61000-4-15 Class F1/F2 flickermeter demodulator chain
/// - Short-term flicker severity P_st (10-minute window)
/// - Long-term flicker severity P_lt (2-hour window from P_st values)
/// - EN 50160 compliance checks: voltage magnitude, frequency, THD, unbalance, dips/swells
/// - IEC 61000-3-3 lamp flicker limits (P_st ≤ 1.0, P_lt ≤ 0.65)
///
/// # Algorithm (IEC 61000-4-15)
///
/// 1. **Input voltage** → normalise (remove DC, scale to 230/240 V ref)
/// 2. **Squaring detector** (v²)
/// 3. **Sliding mean filter** (de-emphasize low-frequency content)
/// 4. **Weighting filter** (simulates eye-brain response, ~8.8 Hz peak)
/// 5. **Squaring + 1st-order LP** → instantaneous flicker sensation s(t)
/// 6. **Statistical classifier** → P_st = √(0.0314·P₀.₁ + 0.0525·P₁ + 0.0657·P₃ + 0.28·P₁₀ + 0.08·P₅₀)
///
/// # References
/// - IEC 61000-4-15:2010 — Flickermeter, functional and design specs
/// - EN 50160:2010 — Voltage characteristics of electricity supplied by public networks
/// - UIE, "Guide to Quality of Electrical Power for Industrial Installations", 1996
use serde::{Deserialize, Serialize};

// ────────────────────────────────────────────────────────────────────────────
// Flicker measurement
// ────────────────────────────────────────────────────────────────────────────

/// Configuration for the flickermeter.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FlickermeterConfig {
    /// Nominal voltage [V rms]
    pub v_nominal: f64,
    /// Nominal frequency [Hz]
    pub f_nominal: f64,
    /// Sample rate [Hz]
    pub fs: f64,
    /// Short-term window length [s] (IEC: 600 s)
    pub t_short_s: f64,
    /// Number of P_st values used for P_lt (IEC: 12 for 2h)
    pub n_pst_for_plt: usize,
}

impl Default for FlickermeterConfig {
    fn default() -> Self {
        Self {
            v_nominal: 230.0,
            f_nominal: 50.0,
            fs: 1600.0, // IEC 61000-4-15 minimum sample rate
            t_short_s: 600.0,
            n_pst_for_plt: 12,
        }
    }
}

impl FlickermeterConfig {
    pub fn hz_60() -> Self {
        Self {
            v_nominal: 120.0,
            f_nominal: 60.0,
            fs: 1920.0,
            t_short_s: 600.0,
            n_pst_for_plt: 12,
        }
    }
}

/// Short-term flicker severity P_st result.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PstResult {
    /// P_st value (≤ 1.0 for IEC 61000-3-3 compliance)
    pub p_st: f64,
    /// Percentile values used: P0.1, P1, P3, P10, P50
    pub p_levels: [f64; 5],
    /// Duration of observation [s]
    pub duration_s: f64,
    /// Compliant with IEC 61000-3-3? (P_st ≤ 1.0)
    pub iec_compliant: bool,
}

/// Long-term flicker severity P_lt.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PltResult {
    /// P_lt value (≤ 0.65 for IEC 61000-3-3 compliance)
    pub p_lt: f64,
    /// P_st values used for computation
    pub p_st_values: Vec<f64>,
    /// Compliant? (P_lt ≤ 0.65)
    pub iec_compliant: bool,
}

/// Compute P_st from a time series of instantaneous flicker sensation values s(t).
///
/// Uses CPF (Cumulative Probability Function) to find the percentile levels:
/// P0.1, P1, P3, P10, P50 of the squared flicker sensation.
pub fn compute_pst(flicker_sensation: &[f64]) -> PstResult {
    if flicker_sensation.is_empty() {
        return PstResult {
            p_st: 0.0,
            p_levels: [0.0; 5],
            duration_s: 0.0,
            iec_compliant: true,
        };
    }

    let n = flicker_sensation.len();
    // Squared sensation (power)
    let mut squared: Vec<f64> = flicker_sensation.iter().map(|&s| s * s).collect();
    squared.sort_by(|a, b| a.partial_cmp(b).unwrap());

    // Percentile extraction
    let percentile = |p: f64| -> f64 {
        let idx = ((1.0 - p / 100.0) * n as f64) as usize;
        let idx = idx.min(n - 1);
        squared[idx]
    };

    let p_levels = [
        percentile(0.1),
        percentile(1.0),
        percentile(3.0),
        percentile(10.0),
        percentile(50.0),
    ];

    // IEC 61000-4-15 weighting formula
    let p_st = (0.0314 * p_levels[0]
        + 0.0525 * p_levels[1]
        + 0.0657 * p_levels[2]
        + 0.2800 * p_levels[3]
        + 0.0800 * p_levels[4])
        .sqrt();

    PstResult {
        p_st,
        p_levels,
        duration_s: n as f64,
        iec_compliant: p_st <= 1.0,
    }
}

/// Compute P_lt from a sequence of P_st values.
///
/// P_lt = ∛( (1/N) · Σ P_st,i³ )
pub fn compute_plt(p_st_values: &[f64]) -> PltResult {
    if p_st_values.is_empty() {
        return PltResult {
            p_lt: 0.0,
            p_st_values: vec![],
            iec_compliant: true,
        };
    }
    let n = p_st_values.len() as f64;
    let cube_mean = p_st_values.iter().map(|&p| p * p * p).sum::<f64>() / n;
    let p_lt = cube_mean.cbrt();
    PltResult {
        p_lt,
        p_st_values: p_st_values.to_vec(),
        iec_compliant: p_lt <= 0.65,
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Simplified flickermeter chain
// ────────────────────────────────────────────────────────────────────────────

/// First-order IIR low-pass filter state.
#[derive(Debug, Clone)]
struct Iir1 {
    a1: f64,
    b0: f64,
    y_prev: f64,
}

impl Iir1 {
    /// Butterworth-equivalent 1st-order LP: fc = cutoff [Hz], fs [Hz]
    fn new_lp(fc: f64, fs: f64) -> Self {
        let wc = 2.0 * std::f64::consts::PI * fc / fs;
        let alpha = wc / (wc + 1.0);
        Self {
            a1: 1.0 - alpha,
            b0: alpha,
            y_prev: 0.0,
        }
    }

    fn step(&mut self, x: f64) -> f64 {
        let y = self.b0 * x + self.a1 * self.y_prev;
        self.y_prev = y;
        y
    }
}

/// Simplified IEC 61000-4-15 flickermeter processing chain.
///
/// Processes a voltage waveform and returns instantaneous flicker sensation samples.
///
/// **Simplified model** (not the full band-pass weighting filter, but captures
/// the essential squaring + LP filtering structure):
///
/// 1. Normalise voltage to ±1.0 (p.u.)
/// 2. Square → demodulate (v² contains 0 and 2f terms)
/// 3. LP filter to remove 2f component → get modulation envelope
/// 4. Subtract mean (remove DC) → flicker modulation signal m(t)
/// 5. Apply 1st-order LP band-limit at ~35 Hz (HF weighting approximation)
/// 6. Square again → proportional to flicker power
/// 7. LP filter at 0.1 Hz → smoothed instantaneous flicker sensation s(t)
pub fn flickermeter_chain(voltage_samples: &[f64], config: &FlickermeterConfig) -> Vec<f64> {
    if voltage_samples.is_empty() {
        return vec![];
    }

    let v_ref = config.v_nominal * std::f64::consts::SQRT_2; // peak voltage
    let fs = config.fs;

    // Step 1: normalise
    let normalised: Vec<f64> = voltage_samples.iter().map(|&v| v / v_ref).collect();

    // Step 2: square (demodulate)
    let squared: Vec<f64> = normalised.iter().map(|&v| v * v).collect();

    // Step 3: LP at f_nominal to remove 2*f carrier
    let mut lp_demod = Iir1::new_lp(config.f_nominal * 0.6, fs);
    let envelope: Vec<f64> = squared.iter().map(|&x| lp_demod.step(x)).collect();

    // Step 4: remove DC (mean subtraction over a window)
    let mean_env = envelope.iter().sum::<f64>() / envelope.len() as f64;
    let modulation: Vec<f64> = envelope.iter().map(|&e| e - mean_env).collect();

    // Step 5: weighting filter (LP at 35 Hz — simplified eye-brain response)
    let mut lp_weight = Iir1::new_lp(8.8, fs); // peak at 8.8 Hz
    let weighted: Vec<f64> = modulation.iter().map(|&m| lp_weight.step(m)).collect();

    // Step 6 & 7: square + smooth LP at 0.1 Hz → sensation s(t)
    let mut lp_sense = Iir1::new_lp(0.1, fs);
    weighted.iter().map(|&w| lp_sense.step(w * w)).collect()
}

// ────────────────────────────────────────────────────────────────────────────
// EN 50160 compliance
// ────────────────────────────────────────────────────────────────────────────

/// EN 50160 compliance check result.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct En50160Report {
    /// Mean supply voltage deviation from nominal [p.u.]
    pub mean_voltage_deviation_pu: f64,
    /// Fraction of 10-min intervals where Vrms is within ±10% of nominal
    pub voltage_within_10pct_fraction: f64,
    /// Mean frequency [Hz]
    pub mean_frequency_hz: f64,
    /// Fraction of time frequency is within ±0.5 Hz (50 Hz system)
    pub frequency_within_band_fraction: f64,
    /// Voltage THD at 95th percentile [fraction]
    pub thd_p95: f64,
    /// Voltage unbalance factor at 95th percentile [fraction]
    pub unbalance_p95: f64,
    /// Number of voltage dips (< 90% nominal) observed per week
    pub dips_per_week: f64,
    /// Number of short interruptions (< 1 min) per year
    pub short_interruptions_per_year: f64,
    /// Overall EN 50160 compliant?
    pub compliant: bool,
    /// List of violated limits
    pub violations: Vec<String>,
}

/// EN 50160 measurement data for one observation period.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct En50160Data {
    /// 10-minute mean rms voltage samples [V]
    pub v_rms_10min: Vec<f64>,
    /// 10-second mean frequency samples [Hz]
    pub frequency_10s: Vec<f64>,
    /// 10-minute THD samples [fraction]
    pub thd_10min: Vec<f64>,
    /// 10-minute voltage unbalance factor samples [fraction]
    pub unbalance_10min: Vec<f64>,
    /// Number of voltage dips observed
    pub n_dips: usize,
    /// Number of short interruptions
    pub n_short_interruptions: usize,
    /// Observation duration [weeks]
    pub observation_weeks: f64,
    /// Nominal voltage [V]
    pub v_nominal: f64,
    /// Nominal frequency [Hz]
    pub f_nominal: f64,
}

impl En50160Data {
    pub fn new_50hz(v_nominal: f64) -> Self {
        Self {
            v_rms_10min: vec![],
            frequency_10s: vec![],
            thd_10min: vec![],
            unbalance_10min: vec![],
            n_dips: 0,
            n_short_interruptions: 0,
            observation_weeks: 1.0,
            v_nominal,
            f_nominal: 50.0,
        }
    }
}

/// Check EN 50160 compliance for a measurement dataset.
pub fn check_en50160(data: &En50160Data) -> En50160Report {
    let v_nom = data.v_nominal;
    let f_nom = data.f_nominal;
    let mut violations = Vec::new();

    // Voltage magnitude: 95% of 10-min means must be within ±10%
    let v_within: f64 = if data.v_rms_10min.is_empty() {
        1.0
    } else {
        data.v_rms_10min
            .iter()
            .filter(|&&v| (v - v_nom).abs() / v_nom <= 0.10)
            .count() as f64
            / data.v_rms_10min.len() as f64
    };
    if v_within < 0.95 {
        violations.push(format!(
            "Voltage: only {:.1}% within ±10% (limit 95%)",
            v_within * 100.0
        ));
    }

    let mean_v_dev = if data.v_rms_10min.is_empty() {
        0.0
    } else {
        data.v_rms_10min
            .iter()
            .map(|&v| (v - v_nom).abs() / v_nom)
            .sum::<f64>()
            / data.v_rms_10min.len() as f64
    };

    // Frequency: 95% of 10-s values within ±0.5 Hz (±1% for 50/60 Hz)
    let freq_band = 0.5; // ±0.5 Hz for interconnected European grid
    let f_within: f64 = if data.frequency_10s.is_empty() {
        1.0
    } else {
        data.frequency_10s
            .iter()
            .filter(|&&f| (f - f_nom).abs() <= freq_band)
            .count() as f64
            / data.frequency_10s.len() as f64
    };
    let mean_freq = if data.frequency_10s.is_empty() {
        f_nom
    } else {
        data.frequency_10s.iter().sum::<f64>() / data.frequency_10s.len() as f64
    };
    if f_within < 0.95 {
        violations.push(format!(
            "Frequency: only {:.1}% within ±0.5 Hz (limit 95%)",
            f_within * 100.0
        ));
    }

    // THD: 95th percentile ≤ 8% (LV)
    let thd_p95 = percentile_95(&data.thd_10min);
    if thd_p95 > 0.08 {
        violations.push(format!("THD: P95={:.1}% exceeds 8% limit", thd_p95 * 100.0));
    }

    // Voltage unbalance: 95th percentile ≤ 2%
    let unbal_p95 = percentile_95(&data.unbalance_10min);
    if unbal_p95 > 0.02 {
        violations.push(format!(
            "Unbalance: P95={:.2}% exceeds 2% limit",
            unbal_p95 * 100.0
        ));
    }

    // Dips: informative (no strict EN 50160 limit, just report)
    let dips_per_week = data.n_dips as f64 / data.observation_weeks.max(1e-6);

    // Short interruptions: < 250/year for some categories
    let interruptions_per_year =
        data.n_short_interruptions as f64 / data.observation_weeks.max(1e-6) * 52.18;
    if interruptions_per_year > 250.0 {
        violations.push(format!(
            "Short interruptions: {:.0}/year exceeds 250/year",
            interruptions_per_year
        ));
    }

    En50160Report {
        mean_voltage_deviation_pu: mean_v_dev,
        voltage_within_10pct_fraction: v_within,
        mean_frequency_hz: mean_freq,
        frequency_within_band_fraction: f_within,
        thd_p95,
        unbalance_p95: unbal_p95,
        dips_per_week,
        short_interruptions_per_year: interruptions_per_year,
        compliant: violations.is_empty(),
        violations,
    }
}

fn percentile_95(data: &[f64]) -> f64 {
    if data.is_empty() {
        return 0.0;
    }
    let mut sorted = data.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let idx = ((0.95 * sorted.len() as f64) as usize).min(sorted.len() - 1);
    sorted[idx]
}

// ────────────────────────────────────────────────────────────────────────────
// Voltage dip / swell statistics (IEC 61000-4-11)
// ────────────────────────────────────────────────────────────────────────────

/// Classification of a voltage dip/swell event.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum VoltageEventClass {
    /// Voltage dip: 10%–90% drop for 10 ms–60 s
    Dip { depth_pu: f64, duration_ms: f64 },
    /// Voltage swell: >110% for 10 ms–60 s
    Swell { height_pu: f64, duration_ms: f64 },
    /// Short interruption: < 1% for < 3 min
    ShortInterruption { duration_ms: f64 },
    /// Long interruption: < 1% for > 3 min
    LongInterruption { duration_min: f64 },
    /// Normal operation
    Normal,
}

/// Detect voltage events in a rms-voltage envelope (10-ms averages).
///
/// Returns classified events with time stamps.
pub fn detect_voltage_events(
    v_rms_10ms: &[f64],
    v_nominal: f64,
    dt_ms: f64,
) -> Vec<(f64, VoltageEventClass)> {
    let dip_threshold = 0.90 * v_nominal;
    let swell_threshold = 1.10 * v_nominal;
    let interruption_threshold = 0.01 * v_nominal;

    let mut events = Vec::new();
    let mut i = 0;

    while i < v_rms_10ms.len() {
        let v = v_rms_10ms[i];
        let t_ms = i as f64 * dt_ms;

        if v < interruption_threshold {
            // Find duration of interruption
            let start = i;
            while i < v_rms_10ms.len() && v_rms_10ms[i] < interruption_threshold {
                i += 1;
            }
            let dur_ms = (i - start) as f64 * dt_ms;
            if dur_ms < 3.0 * 60.0 * 1000.0 {
                events.push((
                    t_ms,
                    VoltageEventClass::ShortInterruption {
                        duration_ms: dur_ms,
                    },
                ));
            } else {
                events.push((
                    t_ms,
                    VoltageEventClass::LongInterruption {
                        duration_min: dur_ms / 60_000.0,
                    },
                ));
            }
        } else if v < dip_threshold {
            let start = i;
            let v_min = {
                let mut vmin = v;
                while i < v_rms_10ms.len() && v_rms_10ms[i] < dip_threshold {
                    vmin = vmin.min(v_rms_10ms[i]);
                    i += 1;
                }
                vmin
            };
            let dur_ms = (i - start) as f64 * dt_ms;
            let depth = 1.0 - v_min / v_nominal;
            events.push((
                t_ms,
                VoltageEventClass::Dip {
                    depth_pu: depth,
                    duration_ms: dur_ms,
                },
            ));
        } else if v > swell_threshold {
            let start = i;
            let v_max = {
                let mut vmax = v;
                while i < v_rms_10ms.len() && v_rms_10ms[i] > swell_threshold {
                    vmax = vmax.max(v_rms_10ms[i]);
                    i += 1;
                }
                vmax
            };
            let dur_ms = (i - start) as f64 * dt_ms;
            let height = v_max / v_nominal - 1.0;
            events.push((
                t_ms,
                VoltageEventClass::Swell {
                    height_pu: height,
                    duration_ms: dur_ms,
                },
            ));
        } else {
            i += 1;
        }
    }

    events
}

/// Summarise voltage events into SARFI-X statistics.
///
/// SARFI-X (System Average RMS Frequency Index) counts dips below X% per customer.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SarfiResult {
    /// SARFI-90: events below 90% Vnom
    pub sarfi_90: f64,
    /// SARFI-70: events below 70%
    pub sarfi_70: f64,
    /// SARFI-50: events below 50%
    pub sarfi_50: f64,
    /// Total dips detected
    pub n_dips: usize,
    /// Total swells detected
    pub n_swells: usize,
    /// Total interruptions (short + long)
    pub n_interruptions: usize,
}

/// Compute SARFI statistics from detected events.
pub fn sarfi_statistics(events: &[(f64, VoltageEventClass)]) -> SarfiResult {
    let n_dips = events
        .iter()
        .filter(|(_, e)| matches!(e, VoltageEventClass::Dip { .. }))
        .count();
    let n_swells = events
        .iter()
        .filter(|(_, e)| matches!(e, VoltageEventClass::Swell { .. }))
        .count();
    let n_int = events
        .iter()
        .filter(|(_, e)| {
            matches!(
                e,
                VoltageEventClass::ShortInterruption { .. }
                    | VoltageEventClass::LongInterruption { .. }
            )
        })
        .count();

    let sarfi_90 = events
        .iter()
        .filter(|(_, e)| match e {
            VoltageEventClass::Dip { depth_pu, .. } => *depth_pu >= 0.10,
            VoltageEventClass::ShortInterruption { .. }
            | VoltageEventClass::LongInterruption { .. } => true,
            _ => false,
        })
        .count() as f64;

    let sarfi_70 = events
        .iter()
        .filter(|(_, e)| match e {
            VoltageEventClass::Dip { depth_pu, .. } => *depth_pu >= 0.30,
            VoltageEventClass::ShortInterruption { .. }
            | VoltageEventClass::LongInterruption { .. } => true,
            _ => false,
        })
        .count() as f64;

    let sarfi_50 = events
        .iter()
        .filter(|(_, e)| match e {
            VoltageEventClass::Dip { depth_pu, .. } => *depth_pu >= 0.50,
            VoltageEventClass::ShortInterruption { .. }
            | VoltageEventClass::LongInterruption { .. } => true,
            _ => false,
        })
        .count() as f64;

    SarfiResult {
        sarfi_90,
        sarfi_70,
        sarfi_50,
        n_dips,
        n_swells,
        n_interruptions: n_int,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sine_wave(freq: f64, amp: f64, duration_s: f64, fs: f64) -> Vec<f64> {
        let n = (duration_s * fs) as usize;
        (0..n)
            .map(|i| amp * (2.0 * std::f64::consts::PI * freq * i as f64 / fs).sin())
            .collect()
    }

    #[test]
    fn test_compute_pst_quiet_signal_low() {
        // Clean sine wave → very low flicker
        let v = sine_wave(50.0, 230.0 * std::f64::consts::SQRT_2, 10.0, 1600.0);
        let config = FlickermeterConfig::default();
        let sensation = flickermeter_chain(&v, &config);
        let pst = compute_pst(&sensation);
        // Clean signal: P_st should be small
        assert!(
            pst.p_st < 0.5,
            "Clean signal should have low flicker: {:.4}",
            pst.p_st
        );
    }

    #[test]
    fn test_compute_pst_empty() {
        let pst = compute_pst(&[]);
        assert_eq!(pst.p_st, 0.0);
        assert!(pst.iec_compliant);
    }

    #[test]
    fn test_compute_plt_basic() {
        let p_st_values = vec![0.3, 0.4, 0.5, 0.3, 0.4, 0.5, 0.3, 0.4, 0.5, 0.3, 0.4, 0.5];
        let result = compute_plt(&p_st_values);
        assert!(result.p_lt > 0.0, "P_lt should be positive");
        // P_lt = cube_root(mean(P_st^3)) — for uniform values, P_lt = P_st
        let expected = (0.3_f64.powi(3) + 0.4_f64.powi(3) + 0.5_f64.powi(3)) / 3.0;
        let expected_plt = expected.cbrt();
        assert!(
            (result.p_lt - expected_plt).abs() < 0.01,
            "P_lt should match analytical: {:.4} vs {:.4}",
            result.p_lt,
            expected_plt
        );
    }

    #[test]
    fn test_compute_plt_empty() {
        let result = compute_plt(&[]);
        assert_eq!(result.p_lt, 0.0);
        assert!(result.iec_compliant);
    }

    #[test]
    fn test_plt_compliant_for_low_pst() {
        let p_st = vec![0.3; 12];
        let result = compute_plt(&p_st);
        assert!(
            result.iec_compliant,
            "P_lt from P_st=0.3 should be compliant: {:.4}",
            result.p_lt
        );
    }

    #[test]
    fn test_plt_noncompliant_for_high_pst() {
        let p_st = vec![1.5; 12]; // P_st >> 1 → P_lt >> 0.65
        let result = compute_plt(&p_st);
        assert!(!result.iec_compliant);
    }

    #[test]
    fn test_flickermeter_chain_length() {
        let v = sine_wave(50.0, 325.3, 1.0, 1600.0);
        let config = FlickermeterConfig::default();
        let sensation = flickermeter_chain(&v, &config);
        assert_eq!(sensation.len(), v.len());
    }

    #[test]
    fn test_flickermeter_chain_output_bounded() {
        let v = sine_wave(50.0, 325.3, 2.0, 1600.0);
        let config = FlickermeterConfig::default();
        let sensation = flickermeter_chain(&v, &config);
        for &s in &sensation {
            assert!(s.is_finite(), "All sensation values should be finite");
        }
    }

    #[test]
    fn test_en50160_compliant_data() {
        let mut data = En50160Data::new_50hz(230.0);
        // All voltages within ±5%
        data.v_rms_10min = (0..1000)
            .map(|i| 230.0 + (i as f64 % 20.0 - 10.0) * 0.5)
            .collect();
        // All frequencies within ±0.3 Hz
        data.frequency_10s = vec![50.0; 5000];
        data.thd_10min = vec![0.03; 1000]; // 3% THD
        data.unbalance_10min = vec![0.01; 1000]; // 1%
        data.n_dips = 5;
        data.n_short_interruptions = 2;
        data.observation_weeks = 1.0;

        let report = check_en50160(&data);
        assert!(
            report.compliant,
            "Good data should be compliant: {:?}",
            report.violations
        );
    }

    #[test]
    fn test_en50160_voltage_violation() {
        let mut data = En50160Data::new_50hz(230.0);
        // 10% of voltages outside ±10% → fails 95% criterion
        data.v_rms_10min = (0..1000)
            .map(|i| if i % 10 == 0 { 230.0 * 1.15 } else { 230.0 })
            .collect();
        data.frequency_10s = vec![50.0; 100];
        data.thd_10min = vec![0.03; 100];
        data.unbalance_10min = vec![0.01; 100];
        data.observation_weeks = 1.0;

        let report = check_en50160(&data);
        assert!(!report.compliant);
        assert!(report.violations.iter().any(|v| v.contains("Voltage")));
    }

    #[test]
    fn test_en50160_thd_violation() {
        let mut data = En50160Data::new_50hz(230.0);
        data.v_rms_10min = vec![230.0; 1000];
        data.frequency_10s = vec![50.0; 1000];
        data.thd_10min = vec![0.10; 1000]; // 10% > 8% limit
        data.unbalance_10min = vec![0.01; 1000];
        data.observation_weeks = 1.0;

        let report = check_en50160(&data);
        assert!(!report.compliant);
        assert!(report.violations.iter().any(|v| v.contains("THD")));
    }

    #[test]
    fn test_detect_voltage_dip() {
        let mut v = vec![230.0_f64; 100];
        // Insert a dip from index 20 to 30 (20% depth)
        for val in v.iter_mut().take(30).skip(20) {
            *val = 230.0 * 0.75;
        } // 25% dip
        let events = detect_voltage_events(&v, 230.0, 10.0);
        let dips: Vec<_> = events
            .iter()
            .filter(|(_, e)| matches!(e, VoltageEventClass::Dip { .. }))
            .collect();
        assert!(!dips.is_empty(), "Should detect voltage dip");
    }

    #[test]
    fn test_detect_voltage_swell() {
        let mut v = vec![230.0_f64; 100];
        for val in v.iter_mut().take(50).skip(40) {
            *val = 230.0 * 1.20;
        }
        let events = detect_voltage_events(&v, 230.0, 10.0);
        let swells: Vec<_> = events
            .iter()
            .filter(|(_, e)| matches!(e, VoltageEventClass::Swell { .. }))
            .collect();
        assert!(!swells.is_empty(), "Should detect swell");
    }

    #[test]
    fn test_detect_short_interruption() {
        let mut v = vec![230.0_f64; 200];
        for val in v.iter_mut().take(70).skip(50) {
            *val = 0.5;
        } // < 1% of nominal
        let events = detect_voltage_events(&v, 230.0, 10.0);
        let ints: Vec<_> = events
            .iter()
            .filter(|(_, e)| matches!(e, VoltageEventClass::ShortInterruption { .. }))
            .collect();
        assert!(!ints.is_empty(), "Should detect short interruption");
    }

    #[test]
    fn test_sarfi_statistics() {
        let events = vec![
            (
                0.0,
                VoltageEventClass::Dip {
                    depth_pu: 0.15,
                    duration_ms: 100.0,
                },
            ),
            (
                500.0,
                VoltageEventClass::Dip {
                    depth_pu: 0.35,
                    duration_ms: 50.0,
                },
            ),
            (
                1000.0,
                VoltageEventClass::Dip {
                    depth_pu: 0.55,
                    duration_ms: 200.0,
                },
            ),
            (
                2000.0,
                VoltageEventClass::Swell {
                    height_pu: 0.12,
                    duration_ms: 80.0,
                },
            ),
        ];
        let sarfi = sarfi_statistics(&events);
        assert_eq!(sarfi.n_dips, 3);
        assert_eq!(sarfi.n_swells, 1);
        assert_eq!(sarfi.sarfi_90, 3.0); // all 3 dips below 90%
        assert_eq!(sarfi.sarfi_70, 2.0); // depth ≥ 0.30: two dips
        assert_eq!(sarfi.sarfi_50, 1.0); // depth ≥ 0.50: one dip
    }

    #[test]
    fn test_pst_iec_compliance() {
        let low_pst = PstResult {
            p_st: 0.8,
            p_levels: [0.0; 5],
            duration_s: 600.0,
            iec_compliant: true,
        };
        let high_pst = PstResult {
            p_st: 1.2,
            p_levels: [0.0; 5],
            duration_s: 600.0,
            iec_compliant: false,
        };
        assert!(low_pst.iec_compliant);
        assert!(!high_pst.iec_compliant);
    }
}
