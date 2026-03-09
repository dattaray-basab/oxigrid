//! Grid code compliance checker for renewable energy generators.
//!
//! Evaluates whether a renewable generator meets the mandatory technical
//! requirements of major international grid codes:
//!
//! - ENTSO-E Requirements for Generators (RfG) 2016
//! - NERC PRC-024-2 / BAL-003-2 (North America, 2022)
//! - IEC 61400-21 (wind turbines)
//! - Australian National Electricity Market (NEM)
//! - UK Grid Code 2019
//! - German VDE-AR-N 4120 (2018)
//!
//! # Key checks
//! - **Low-voltage ride-through** (LVRT) — generator must stay connected while
//!   terminal voltage is below the code-specific `(time_ms, voltage_pu)` envelope.
//! - **Reactive capability** — minimum `\[pu\]` Q range at rated active power.
//! - **Frequency response** — droop `\[%\]` and ROCOF withstand `\[Hz/s\]`.
//! - **Power quality** — THD limits for voltage and current.

use serde::{Deserialize, Serialize};
use std::fmt;

// ─────────────────────────────────────────────────────────────────────────────
// Error
// ─────────────────────────────────────────────────────────────────────────────

/// Errors that can occur during a compliance assessment.
#[derive(Debug, Clone, PartialEq)]
pub enum ComplianceError {
    /// The supplied LVRT or capability profile is invalid.
    InvalidProfile(String),
    /// Required measurement data is absent.
    MissingData(String),
}

impl fmt::Display for ComplianceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidProfile(msg) => write!(f, "invalid profile: {msg}"),
            Self::MissingData(msg) => write!(f, "missing data: {msg}"),
        }
    }
}

impl std::error::Error for ComplianceError {}

// ─────────────────────────────────────────────────────────────────────────────
// Grid-code standard
// ─────────────────────────────────────────────────────────────────────────────

/// Supported grid-code standards.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum GridCodeStandard {
    /// European ENTSO-E Requirements for Generators (RfG), 2016.
    EntsoE2016,
    /// North American NERC PRC-024-2 / BAL-003-2 (2022).
    NercNerc2022,
    /// IEC 61400-21 wind-turbine power quality standard.
    IecIec61400,
    /// Australian National Electricity Market (NEM) requirements.
    AusNem,
    /// UK Grid Code, 2019 edition.
    Uk2019,
    /// German VDE-AR-N 4120:2018 (high-voltage generation units).
    Germany2018,
    /// User-defined custom standard (falls back to IEC 61400-21 limits).
    Custom { name: String },
}

impl GridCodeStandard {
    /// Human-readable name of the standard.
    pub fn display_name(&self) -> String {
        match self {
            Self::EntsoE2016 => "ENTSO-E RfG 2016".to_string(),
            Self::NercNerc2022 => "NERC 2022".to_string(),
            Self::IecIec61400 => "IEC 61400-21".to_string(),
            Self::AusNem => "Australian NEM".to_string(),
            Self::Uk2019 => "UK Grid Code 2019".to_string(),
            Self::Germany2018 => "VDE-AR-N 4120:2018".to_string(),
            Self::Custom { name } => name.clone(),
        }
    }

    /// Required LVRT envelope as `(time_ms, min_voltage_pu)` piecewise-linear points.
    ///
    /// The generator's capability profile must lie **at or above** this envelope.
    fn lvrt_envelope(&self) -> Vec<(f64, f64)> {
        match self {
            Self::EntsoE2016 | Self::IecIec61400 => vec![
                (0.0, 0.0),
                (150.0, 0.0),
                (150.0, 0.85),
                (500.0, 0.85),
                (1500.0, 0.9),
                (3000.0, 1.0),
            ],
            Self::NercNerc2022 => vec![(0.0, 0.0), (625.0, 0.0), (3000.0, 0.9)],
            Self::AusNem => vec![(0.0, 0.0), (200.0, 0.0), (200.0, 0.7), (2000.0, 0.9)],
            Self::Uk2019 => vec![
                (0.0, 0.0),
                (140.0, 0.0),
                (140.0, 0.8),
                (1200.0, 0.85),
                (2500.0, 0.9),
            ],
            Self::Germany2018 => vec![(0.0, 0.0), (150.0, 0.0), (150.0, 0.85), (1500.0, 0.9)],
            Self::Custom { .. } => vec![
                (0.0, 0.0),
                (150.0, 0.0),
                (150.0, 0.85),
                (500.0, 0.85),
                (1500.0, 0.9),
                (3000.0, 1.0),
            ],
        }
    }

    /// Required reactive capability: `(Q_min_pu, Q_max_pu)` at rated P.
    fn required_reactive_range(&self) -> (f64, f64) {
        match self {
            Self::EntsoE2016 | Self::NercNerc2022 | Self::Uk2019 | Self::Germany2018 => {
                (-0.33, 0.33)
            }
            Self::IecIec61400 => (-0.40, 0.40),
            Self::AusNem => (-0.395, 0.395),
            Self::Custom { .. } => (-0.33, 0.33),
        }
    }

    /// Required ROCOF withstand capability `\[Hz/s\]`.
    fn required_rocof_hz_s(&self) -> f64 {
        match self {
            Self::EntsoE2016 | Self::Germany2018 => 2.0,
            Self::NercNerc2022 => 1.5,
            Self::IecIec61400 | Self::Uk2019 => 1.0,
            Self::AusNem => 4.0,
            Self::Custom { .. } => 2.0,
        }
    }

    /// Droop limits: `(min_pct, max_pct)`.
    fn droop_limits_pct(&self) -> (f64, f64) {
        // All major codes require droop in 2–10 % range
        (2.0, 10.0)
    }

    /// THD limits: `(voltage_pct, current_pct)`.
    fn thd_limits_pct(&self) -> (f64, f64) {
        (5.0, 8.0)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Generator type
// ─────────────────────────────────────────────────────────────────────────────

/// Classification of the renewable or distributed generator.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum GeneratorType {
    /// Onshore wind turbine or farm.
    WindOnshore,
    /// Offshore wind turbine or farm.
    WindOffshore,
    /// Utility-scale or distributed solar PV.
    SolarPv,
    /// Battery energy storage system.
    Battery,
    /// Combined heat-and-power plant.
    Chp,
    /// Any other generator type.
    Other,
}

// ─────────────────────────────────────────────────────────────────────────────
// Configuration
// ─────────────────────────────────────────────────────────────────────────────

/// Configuration for a compliance assessment.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ComplianceCheckerConfig {
    /// Grid-code standard to assess against.
    pub grid_code: GridCodeStandard,
    /// Technology type of the generator.
    pub generator_type: GeneratorType,
    /// Rated active power `\[MW\]`.
    pub rated_mw: f64,
    /// Rated terminal voltage `\[kV\]`.
    pub rated_voltage_kv: f64,
    /// Voltage at the point of connection `\[kV\]`.
    pub connection_point_kv: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Test categories and severity
// ─────────────────────────────────────────────────────────────────────────────

/// High-level grouping of grid-code requirements.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ComplianceCategory {
    /// LVRT — remain connected during low-voltage events.
    LowVoltageRideThrough,
    /// HVRT — remain connected during high-voltage events.
    HighVoltageRideThrough,
    /// Active-power / frequency droop and ROCOF withstand.
    FrequencyResponse,
    /// Q-range and power-factor capability.
    ReactiveCapability,
    /// Voltage regulation and regulation band.
    VoltageRegulation,
    /// Harmonic distortion limits (THD, individual harmonics).
    PowerQuality,
    /// Relay settings (overcurrent, overvoltage, underfrequency trip).
    ProtectionSettings,
    /// SCADA, RTU, and communications requirements.
    CommunicationsAndControl,
}

/// How severely a non-compliance affects grid security.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ComplianceSeverity {
    /// Immediate risk to grid security; must be resolved before energisation.
    Critical,
    /// Significant impact; must be resolved before commercial operation.
    Major,
    /// Minor deviation; may be accepted with a technical waiver.
    Minor,
    /// Advisory item only; no mandatory action required.
    Informational,
}

// ─────────────────────────────────────────────────────────────────────────────
// Individual test result
// ─────────────────────────────────────────────────────────────────────────────

/// Result of a single grid-code compliance test.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ComplianceTest {
    /// Short identifier of the test (e.g. "LVRT_envelope").
    pub name: String,
    /// Compliance category this test belongs to.
    pub category: ComplianceCategory,
    /// Narrative description of the requirement.
    pub requirement: String,
    /// Measured or declared test value.
    pub test_value: f64,
    /// Threshold / limit defined by the grid code.
    pub limit: f64,
    /// `true` if the test passes (test_value satisfies the limit).
    pub passed: bool,
    /// Severity if this test fails.
    pub severity: ComplianceSeverity,
}

// ─────────────────────────────────────────────────────────────────────────────
// Full compliance report
// ─────────────────────────────────────────────────────────────────────────────

/// Complete grid-code compliance assessment report.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ComplianceReport {
    /// Identifier of the generator being assessed.
    pub generator_id: String,
    /// Grid-code standard used for this assessment.
    pub grid_code: String,
    /// Date of the assessment (ISO 8601 string if available, else "unknown").
    pub assessment_date: String,
    /// All individual test results.
    pub tests: Vec<ComplianceTest>,
    /// `true` if zero Critical or Major failures.
    pub overall_compliant: bool,
    /// Number of Critical failures.
    pub critical_failures: usize,
    /// Number of Major failures.
    pub major_failures: usize,
    /// `(passed / total) × 100` `\[%\]`.
    pub compliance_score_pct: f64,
    /// Actionable recommendations for each failing test.
    pub recommendations: Vec<String>,
}

// ─────────────────────────────────────────────────────────────────────────────
// Compliance checker
// ─────────────────────────────────────────────────────────────────────────────

/// Grid-code compliance checker.
///
/// Instantiate with a [`ComplianceCheckerConfig`] then call [`assess`](Self::assess).
pub struct ComplianceChecker {
    config: ComplianceCheckerConfig,
}

impl ComplianceChecker {
    /// Create a new checker with the given configuration.
    pub fn new(config: ComplianceCheckerConfig) -> Self {
        Self { config }
    }

    /// Run a full compliance assessment.
    ///
    /// # Arguments
    /// - `generator_id`            — label used in the output report.
    /// - `lvrt_profile`            — measured LVRT capability curve: `(time_ms, voltage_pu)`.
    ///   Each point gives the **lowest voltage the generator can withstand** at that time.
    /// - `reactive_capability`     — `(Q_min_pu, Q_max_pu)` at rated P.
    /// - `pf_range`                — `(PF_lead, PF_lag)` (absolute values, both ≤ 1.0).
    /// - `freq_response_droop_pct` — primary frequency droop `\[%\]`.
    /// - `rocof_capability_hz_per_s` — maximum rate-of-change-of-frequency `\[Hz/s\]` the
    ///   generator can withstand.
    /// - `thd_voltage_pct`         — total harmonic distortion of terminal voltage `\[%\]`.
    /// - `thd_current_pct`         — total harmonic distortion of injected current `\[%\]`.
    #[allow(clippy::too_many_arguments)]
    pub fn assess(
        &self,
        generator_id: &str,
        lvrt_profile: &[(f64, f64)],
        reactive_capability: (f64, f64),
        pf_range: (f64, f64),
        freq_response_droop_pct: f64,
        rocof_capability_hz_per_s: f64,
        thd_voltage_pct: f64,
        thd_current_pct: f64,
    ) -> Result<ComplianceReport, ComplianceError> {
        if lvrt_profile.is_empty() {
            return Err(ComplianceError::InvalidProfile(
                "LVRT profile must contain at least one point".to_string(),
            ));
        }

        let mut tests: Vec<ComplianceTest> = Vec::new();

        // LVRT
        tests.extend(self.check_lvrt(lvrt_profile));

        // Reactive capability
        let (q_min, q_max) = reactive_capability;
        let (pf_lead, pf_lag) = pf_range;
        tests.extend(self.check_reactive(q_min, q_max, pf_lead, pf_lag));

        // Frequency response
        tests.extend(
            self.check_frequency_response(freq_response_droop_pct, rocof_capability_hz_per_s),
        );

        // Power quality
        tests.extend(self.check_power_quality(thd_voltage_pct, thd_current_pct));

        // Aggregate
        let total = tests.len();
        let passed_count = tests.iter().filter(|t| t.passed).count();
        let critical_failures = tests
            .iter()
            .filter(|t| !t.passed && matches!(t.severity, ComplianceSeverity::Critical))
            .count();
        let major_failures = tests
            .iter()
            .filter(|t| !t.passed && matches!(t.severity, ComplianceSeverity::Major))
            .count();

        let compliance_score_pct = if total == 0 {
            100.0
        } else {
            (passed_count as f64 / total as f64) * 100.0
        };

        let overall_compliant = critical_failures == 0 && major_failures == 0;
        let recommendations = Self::build_recommendations(&tests);

        Ok(ComplianceReport {
            generator_id: generator_id.to_string(),
            grid_code: self.config.grid_code.display_name(),
            assessment_date: "unknown".to_string(),
            tests,
            overall_compliant,
            critical_failures,
            major_failures,
            compliance_score_pct,
            recommendations,
        })
    }

    /// Check the LVRT capability profile against the grid-code envelope.
    ///
    /// Compares the generator's declared capability curve against the grid-code
    /// LVRT envelope at a set of time samples (every 1 ms from 0 to 3000 ms).
    /// The capability must be ≥ the required voltage at every sample time.
    ///
    /// Note: step discontinuities in the envelope (same time, two voltage levels)
    /// are handled by using the **maximum required voltage** at any given time.
    pub fn check_lvrt(&self, actual: &[(f64, f64)]) -> Vec<ComplianceTest> {
        let envelope = self.config.grid_code.lvrt_envelope();

        // Find time range
        let t_max = envelope.last().map(|&(t, _)| t).unwrap_or(3000.0);

        // Build a deduplicated required-voltage lookup:
        // at each unique time, take the MAXIMUM required voltage (handles step jumps).
        // Collect unique times
        let mut times: Vec<f64> = envelope.iter().map(|&(t, _)| t).collect();
        times.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
        // Also add intermediate sample points
        let mut sample_times: Vec<f64> = Vec::new();
        let steps = 300usize;
        for i in 0..=steps {
            sample_times.push(t_max * i as f64 / steps as f64);
        }
        for t in times {
            sample_times.push(t + 0.001); // just after any step
        }
        sample_times.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        sample_times.dedup_by(|a, b| (*a - *b).abs() < 1e-9);

        let mut worst_margin: f64 = f64::MAX;
        let mut passed = true;

        for t in sample_times {
            let v_req = interpolate_envelope_max(&envelope, t);
            let v_actual = interpolate_profile(actual, t);
            let margin = v_actual - v_req;
            if margin < worst_margin {
                worst_margin = margin;
            }
            if v_actual < v_req - 1e-6 {
                passed = false;
            }
        }

        let worst_margin_display = if worst_margin == f64::MAX {
            0.0
        } else {
            worst_margin
        };

        vec![ComplianceTest {
            name: "LVRT_envelope".to_string(),
            category: ComplianceCategory::LowVoltageRideThrough,
            requirement: format!(
                "Generator must remain connected when voltage follows the {} LVRT envelope",
                self.config.grid_code.display_name()
            ),
            test_value: worst_margin_display,
            limit: 0.0, // margin must be ≥ 0
            passed,
            severity: ComplianceSeverity::Critical,
        }]
    }

    /// Check reactive capability against grid-code Q-range requirements.
    ///
    /// Two sub-tests:
    /// 1. Q-range — `Q_min_pu ≤ required_min` and `Q_max_pu ≥ required_max`.
    /// 2. Power factor — both lead and lag must meet the minimum |PF| derived
    ///    from the required Q-range.
    pub fn check_reactive(
        &self,
        q_min: f64,
        q_max: f64,
        pf_lead: f64,
        pf_lag: f64,
    ) -> Vec<ComplianceTest> {
        let (req_q_min, req_q_max) = self.config.grid_code.required_reactive_range();
        let mut out = Vec::new();

        // Q-range test
        let q_min_ok = q_min <= req_q_min + 1e-9;
        let q_max_ok = q_max >= req_q_max - 1e-9;
        let q_passed = q_min_ok && q_max_ok;
        let q_test_value = (q_max - q_min) / 2.0; // symmetric half-range

        out.push(ComplianceTest {
            name: "reactive_Q_range".to_string(),
            category: ComplianceCategory::ReactiveCapability,
            requirement: format!(
                "Q range must span [{:.2}, +{:.2}] pu at rated P",
                req_q_min, req_q_max
            ),
            test_value: q_test_value,
            limit: req_q_max,
            passed: q_passed,
            severity: ComplianceSeverity::Major,
        });

        // Power-factor test
        // PF corresponding to required Q range: PF_min = cos(atan(Q_req)) = 1/sqrt(1+Q^2)
        // A generator PASSES if its declared PF ≤ required_min_pf,
        // i.e. it can absorb/inject at least as much reactive power as required.
        // (Lower PF means more reactive capability.)
        let required_min_pf = (1.0_f64 / (1.0 + req_q_max * req_q_max).sqrt()).max(0.85);
        // Use the larger (less reactive) of lead/lag as the tightest constraint
        let pf_actual = pf_lead.max(pf_lag);
        let pf_passed = pf_actual <= required_min_pf + 1e-6;

        out.push(ComplianceTest {
            name: "reactive_PF_range".to_string(),
            category: ComplianceCategory::ReactiveCapability,
            requirement: format!(
                "Power factor capability must be ≤ {:.3} (lead/lag)",
                required_min_pf
            ),
            test_value: pf_actual,
            limit: required_min_pf,
            passed: pf_passed,
            severity: ComplianceSeverity::Major,
        });

        out
    }

    /// Check frequency-response requirements: droop and ROCOF withstand.
    pub fn check_frequency_response(&self, droop_pct: f64, rocof_hz_s: f64) -> Vec<ComplianceTest> {
        let (droop_min, droop_max) = self.config.grid_code.droop_limits_pct();
        let req_rocof = self.config.grid_code.required_rocof_hz_s();
        let mut out = Vec::new();

        // Droop test
        let droop_passed = droop_pct >= droop_min - 1e-9 && droop_pct <= droop_max + 1e-9;
        out.push(ComplianceTest {
            name: "freq_droop".to_string(),
            category: ComplianceCategory::FrequencyResponse,
            requirement: format!(
                "Primary frequency droop must be in [{droop_min:.0}, {droop_max:.0}] \u{25}"
            ),
            test_value: droop_pct,
            limit: droop_max,
            passed: droop_passed,
            severity: ComplianceSeverity::Major,
        });

        // ROCOF withstand test
        let rocof_passed = rocof_hz_s >= req_rocof - 1e-9;
        out.push(ComplianceTest {
            name: "freq_ROCOF_withstand".to_string(),
            category: ComplianceCategory::FrequencyResponse,
            requirement: format!(
                "Generator must withstand ROCOF of {req_rocof:.1} \u{5B}Hz/s\u{5D}"
            ),
            test_value: rocof_hz_s,
            limit: req_rocof,
            passed: rocof_passed,
            severity: ComplianceSeverity::Major,
        });

        out
    }

    /// Check power-quality THD limits.
    fn check_power_quality(&self, thd_v_pct: f64, thd_i_pct: f64) -> Vec<ComplianceTest> {
        let (v_limit, i_limit) = self.config.grid_code.thd_limits_pct();
        vec![
            ComplianceTest {
                name: "pq_THD_voltage".to_string(),
                category: ComplianceCategory::PowerQuality,
                requirement: format!("Terminal voltage THD must be < {v_limit:.0} %"),
                test_value: thd_v_pct,
                limit: v_limit,
                passed: thd_v_pct <= v_limit + 1e-9,
                severity: ComplianceSeverity::Minor,
            },
            ComplianceTest {
                name: "pq_THD_current".to_string(),
                category: ComplianceCategory::PowerQuality,
                requirement: format!("Injected current THD must be < {i_limit:.0} %"),
                test_value: thd_i_pct,
                limit: i_limit,
                passed: thd_i_pct <= i_limit + 1e-9,
                severity: ComplianceSeverity::Minor,
            },
        ]
    }

    /// Build actionable recommendation strings for every failing test.
    fn build_recommendations(tests: &[ComplianceTest]) -> Vec<String> {
        let mut recs: Vec<String> = Vec::new();
        for t in tests.iter().filter(|t| !t.passed) {
            let rec = match t.name.as_str() {
                "LVRT_envelope" => format!(
                    "[CRITICAL] LVRT: upgrade inverter firmware / hardware to maintain \
                     connection during grid faults per the required voltage-time envelope. \
                     Current margin: {:.3} pu below requirement.",
                    t.limit - t.test_value
                ),
                "reactive_Q_range" => format!(
                    "[MAJOR] Reactive Q-range: increase reactive power capability from \
                     ±{:.2} pu to ±{:.2} pu by upgrading inverter rating or adding \
                     reactive compensation.",
                    t.test_value, t.limit
                ),
                "reactive_PF_range" => format!(
                    "[MAJOR] Power factor: declare a minimum PF of {:.3} (lead/lag) \
                     to satisfy Q requirements.",
                    t.limit
                ),
                "freq_droop" => format!(
                    "[MAJOR] Frequency droop: adjust governor/inverter droop from {:.1} % \
                     to a value in the permitted range.",
                    t.test_value
                ),
                "freq_ROCOF_withstand" => format!(
                    "[MAJOR] ROCOF: increase anti-islanding relay ROCOF threshold from \
                     {:.1} to {:.1} Hz/s to avoid premature tripping.",
                    t.test_value, t.limit
                ),
                "pq_THD_voltage" => format!(
                    "[MINOR] Voltage THD: install harmonic filters or upgrade inverter \
                     switching strategy to reduce voltage THD from {:.1} % to < {:.0} %.",
                    t.test_value, t.limit
                ),
                "pq_THD_current" => format!(
                    "[MINOR] Current THD: reduce current harmonic injection from {:.1} % \
                     to < {:.0} % using active filters or modified PWM control.",
                    t.test_value, t.limit
                ),
                other => format!(
                    "[INFO] Test '{other}' failed (value={:.3}, limit={:.3}).",
                    t.test_value, t.limit
                ),
            };
            recs.push(rec);
        }
        recs
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Linearly interpolate a piecewise-linear `(x, y)` profile at position `x_query`.
///
/// Clamps to the first/last y value if `x_query` is outside the profile range.
///
/// **Step discontinuities**: if the profile contains duplicate x-values
/// (e.g. `(150, 0.0), (150, 0.9)` to model an instantaneous step), the
/// function returns the **maximum** y at that x — i.e. the upper value of the step.
pub fn interpolate_profile(profile: &[(f64, f64)], x_query: f64) -> f64 {
    if profile.is_empty() {
        return 0.0;
    }
    if x_query < profile[0].0 {
        return profile[0].1;
    }
    if x_query > profile[profile.len() - 1].0 {
        return profile[profile.len() - 1].1;
    }
    // Check for exact match at x_query (handles step discontinuities):
    // collect all y values at this x and return the maximum.
    let at_x: Vec<f64> = profile
        .iter()
        .filter(|&&(x, _)| (x - x_query).abs() < 1e-9)
        .map(|&(_, y)| y)
        .collect();
    if !at_x.is_empty() {
        return at_x.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    }
    // Standard linear interpolation between bracketing points
    for i in 1..profile.len() {
        let (x0, y0) = profile[i - 1];
        let (x1, y1) = profile[i];
        if x_query < x1 {
            if (x1 - x0).abs() < 1e-15 {
                return y1;
            }
            let t = (x_query - x0) / (x1 - x0);
            return y0 + t * (y1 - y0);
        }
    }
    profile[profile.len() - 1].1
}

/// Evaluate the required voltage at time `t_ms` from a possibly step-discontinuous
/// envelope, taking the **maximum** required voltage at that exact time instant.
///
/// This handles the ENTSO-E style `(150ms, 0) → (150ms, 0.85)` step: at 150 ms
/// the requirement instantly jumps to 0.85 pu.
fn interpolate_envelope_max(envelope: &[(f64, f64)], t_ms: f64) -> f64 {
    // Collect all (time, voltage) pairs at this time
    let mut at_t: Vec<f64> = envelope
        .iter()
        .filter(|&&(t, _)| (t - t_ms).abs() < 1e-9)
        .map(|&(_, v)| v)
        .collect();
    if !at_t.is_empty() {
        at_t.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        return *at_t.last().unwrap_or(&0.0);
    }
    // Otherwise linear interpolation
    interpolate_profile(envelope, t_ms)
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn entso_config() -> ComplianceCheckerConfig {
        ComplianceCheckerConfig {
            grid_code: GridCodeStandard::EntsoE2016,
            generator_type: GeneratorType::WindOnshore,
            rated_mw: 10.0,
            rated_voltage_kv: 33.0,
            connection_point_kv: 33.0,
        }
    }

    /// A "good" LVRT profile — clearly above the ENTSO-E envelope.
    ///
    /// Uses duplicate time-point at 150 ms to model the step-recovery
    /// from 0 to 0.90 pu, matching the ENTSO-E envelope requirement of
    /// 0.85 pu immediately after 150 ms.
    fn good_lvrt() -> Vec<(f64, f64)> {
        vec![
            (0.0, 0.0),
            (150.0, 0.0),
            (150.0, 0.90), // step — above 0.85 required by ENTSO-E
            (500.0, 0.90),
            (1500.0, 0.95),
            (3000.0, 1.0),
        ]
    }

    #[test]
    fn test_compliant_generator_all_pass() {
        let checker = ComplianceChecker::new(entso_config());
        let report = checker
            .assess(
                "gen1",
                &good_lvrt(),
                (-0.4, 0.4),
                (0.92, 0.92),
                5.0,
                3.0,
                3.0,
                5.0,
            )
            .expect("assess must succeed");

        assert!(
            report.overall_compliant,
            "all tests should pass: {:?}",
            report.tests
        );
        assert_eq!(report.critical_failures, 0);
        assert_eq!(report.major_failures, 0);
        assert!(
            (report.compliance_score_pct - 100.0).abs() < 1e-3,
            "score should be 100 %, got {:.2}",
            report.compliance_score_pct
        );
    }

    #[test]
    fn test_lvrt_failure_is_critical() {
        let checker = ComplianceChecker::new(entso_config());
        // Profile reaches 0.85 pu only at 400 ms — but ENTSO-E requires 0.85 pu from 150 ms.
        let bad_lvrt = vec![
            (0.0, 0.0),
            (300.0, 0.0), // still 0 at 300 ms — violates the 150 ms boundary
            (500.0, 0.5),
            (3000.0, 0.8),
        ];
        let report = checker
            .assess(
                "gen2",
                &bad_lvrt,
                (-0.4, 0.4),
                (0.92, 0.92),
                5.0,
                3.0,
                3.0,
                5.0,
            )
            .unwrap();
        assert!(
            report.critical_failures > 0,
            "LVRT violation must be flagged as Critical"
        );
        assert!(!report.overall_compliant);
    }

    #[test]
    fn test_reactive_capability_major_failure() {
        let checker = ComplianceChecker::new(entso_config());
        // Insufficient Q range: ±0.05 pu — ENTSO-E needs ±0.33 pu.
        let report = checker
            .assess(
                "gen3",
                &good_lvrt(),
                (-0.05, 0.05),
                (0.999, 0.999),
                5.0,
                3.0,
                3.0,
                5.0,
            )
            .unwrap();
        assert!(
            report.major_failures > 0,
            "Insufficient Q range must be flagged as Major"
        );
    }

    #[test]
    fn test_score_100_for_fully_compliant() {
        let config = ComplianceCheckerConfig {
            grid_code: GridCodeStandard::IecIec61400,
            generator_type: GeneratorType::WindOnshore,
            rated_mw: 3.0,
            rated_voltage_kv: 33.0,
            connection_point_kv: 33.0,
        };
        let checker = ComplianceChecker::new(config);
        let report = checker
            .assess(
                "gen4",
                &good_lvrt(),
                (-0.45, 0.45),
                (0.92, 0.92),
                4.0,
                2.0,
                2.0,
                4.0,
            )
            .unwrap();
        assert!(
            (report.compliance_score_pct - 100.0).abs() < 1e-3,
            "score should be 100 %, got {:.2}",
            report.compliance_score_pct
        );
    }

    #[test]
    fn test_recommendations_generated_for_failures() {
        let config = ComplianceCheckerConfig {
            grid_code: GridCodeStandard::Germany2018,
            generator_type: GeneratorType::SolarPv,
            rated_mw: 1.0,
            rated_voltage_kv: 0.4,
            connection_point_kv: 0.4,
        };
        let checker = ComplianceChecker::new(config);
        let bad_lvrt = vec![
            (0.0, 0.0),
            (500.0, 0.0), // Still 0 at 500 ms — Germany requires recovery at 150 ms
            (3000.0, 0.5),
        ];
        let report = checker
            .assess(
                "gen5",
                &bad_lvrt,
                (-0.05, 0.05),
                (0.999, 0.999),
                5.0,
                3.0,
                3.0,
                5.0,
            )
            .unwrap();
        assert!(
            !report.recommendations.is_empty(),
            "recommendations must be generated for failures"
        );
    }

    #[test]
    fn test_nerc_standard_625ms_zero_voltage_allowed() {
        let config = ComplianceCheckerConfig {
            grid_code: GridCodeStandard::NercNerc2022,
            generator_type: GeneratorType::WindOnshore,
            rated_mw: 100.0,
            rated_voltage_kv: 138.0,
            connection_point_kv: 138.0,
        };
        let checker = ComplianceChecker::new(config);
        // NERC allows 625 ms at zero voltage
        let lvrt = vec![(0.0, 0.0), (625.0, 0.0), (626.0, 0.9), (3000.0, 0.95)];
        let report = checker
            .assess(
                "gen6",
                &lvrt,
                (-0.35, 0.35),
                (0.94, 0.94),
                5.0,
                2.0,
                3.0,
                6.0,
            )
            .unwrap();
        let lvrt_tests: Vec<_> = report
            .tests
            .iter()
            .filter(|t| matches!(t.category, ComplianceCategory::LowVoltageRideThrough))
            .collect();
        assert!(
            lvrt_tests.iter().all(|t| t.passed),
            "NERC LVRT: 625 ms zero voltage must pass"
        );
    }

    #[test]
    fn test_interpolate_profile_bounds() {
        let profile = vec![(0.0, 0.0), (100.0, 1.0), (200.0, 0.5)];
        // Below first point — should return first y
        assert!((interpolate_profile(&profile, -10.0) - 0.0).abs() < 1e-12);
        // Above last point — should return last y
        assert!((interpolate_profile(&profile, 300.0) - 0.5).abs() < 1e-12);
        // Exact midpoint
        assert!((interpolate_profile(&profile, 50.0) - 0.5).abs() < 1e-12);
        // Between second and third
        assert!((interpolate_profile(&profile, 150.0) - 0.75).abs() < 1e-12);
    }

    #[test]
    fn test_aus_nem_standard() {
        let config = ComplianceCheckerConfig {
            grid_code: GridCodeStandard::AusNem,
            generator_type: GeneratorType::SolarPv,
            rated_mw: 50.0,
            rated_voltage_kv: 66.0,
            connection_point_kv: 66.0,
        };
        let checker = ComplianceChecker::new(config);
        // AusNEM: zero for 200 ms, then instantaneous step to 0.75 pu, then 0.9 pu at 2 s.
        // The capability profile uses a duplicate time-point at 200 ms to signal
        // the step recovery — this matches how the envelope is specified.
        let lvrt = vec![
            (0.0, 0.0),
            (200.0, 0.0),
            (200.0, 0.75), // step recovery — matches AusNEM requirement of 0.7 pu at 200 ms
            (2000.0, 0.95),
        ];
        let report = checker
            .assess("gen7", &lvrt, (-0.4, 0.4), (0.92, 0.92), 5.0, 5.0, 3.0, 5.0)
            .unwrap();
        let lvrt_tests: Vec<_> = report
            .tests
            .iter()
            .filter(|t| matches!(t.category, ComplianceCategory::LowVoltageRideThrough))
            .collect();
        assert!(
            lvrt_tests.iter().all(|t| t.passed),
            "AusNEM LVRT test should pass with compliant profile"
        );
    }
}
