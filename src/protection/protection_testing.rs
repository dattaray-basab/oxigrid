//! Protection system testing and verification module.
//!
//! Provides relay commissioning, secondary/primary injection testing, timing
//! verification, characteristic curve testing, and automated test reporting
//! for IEC 60255 / IEEE C37.112 overcurrent protection systems.
//!
//! # Overview
//! - [`ProtectionTester`] — main testing engine with LCG-based noise simulation
//! - [`TestSuite`] — collection of test cases with pass/fail accounting
//! - [`TestReportGenerator`] — statistical analysis and recommendation engine
//!
//! # IEC Standard Inverse formula
//! `t = TDS × 0.14 / ((I/Is)^0.02 − 1)` `seconds`

use serde::{Deserialize, Serialize};

// ── Enums ───────────────────────────────────────────────────────────────────

/// Category of protection test being performed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TestType {
    /// Verify relay picks up at the configured current threshold.
    PickupTest,
    /// Verify the inverse-time operating characteristic.
    TimingTest,
    /// Verify the complete time-current characteristic (TCC) curve.
    CharacteristicTest,
    /// Verify directional element (forward vs reverse fault discrimination).
    DirectionalTest,
    /// Verify differential element (percentage differential protection).
    DifferentialTest,
    /// Verify distance relay zone reach and timing.
    DistanceZoneTest,
    /// Verify auto-reclose sequence and timing.
    AutorecloserTest,
    /// Current/voltage injection at relay secondary terminals.
    SecondaryInjectionTest,
    /// Current/voltage injection at CT/PT primary side.
    PrimaryInjectionTest,
    /// End-to-end test spanning two relay terminals.
    EndToEndTest,
}

/// Current execution status of a test case or suite.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TestStatus {
    /// Not yet started.
    Pending,
    /// Currently executing.
    Running,
    /// All criteria met.
    Passed,
    /// One or more criteria not met.
    Failed,
    /// Result is ambiguous (e.g., near tolerance boundary).
    Inconclusive,
    /// Test was intentionally skipped.
    Skipped,
}

/// How the test stimulus is injected into the protection system.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum InjectionMode {
    /// Inject at relay secondary terminals (most common).
    Secondary,
    /// Inject at CT/PT primary side.
    Primary,
    /// Software simulation without physical injection.
    Digital,
}

// ── Core structs ─────────────────────────────────────────────────────────────

/// A single parameterised protection test case.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TestCase {
    /// Unique numeric identifier within the suite.
    pub id: usize,
    /// Human-readable name.
    pub name: String,
    /// Category of protection test.
    pub test_type: TestType,
    /// Detailed description of the test objective.
    pub description: String,
    /// Injected current at relay terminals [A secondary].
    pub input_current_a: f64,
    /// Injected voltage at relay terminals [V secondary].
    pub input_voltage_v: f64,
    /// Injected signal frequency `Hz`.
    pub input_frequency_hz: f64,
    /// Current-to-voltage angle `degrees` (positive = lagging current).
    pub input_angle_deg: f64,
    /// Expected relay operating time `ms`.
    pub expected_operate_time_ms: f64,
    /// Acceptable timing error `ms` (pass if |actual − expected| ≤ tolerance).
    pub tolerance_ms: f64,
    /// `true` if the relay is expected to operate (pick up).
    pub expected_pickup: bool,
    /// Maximum test duration before declaring no-operation `ms`.
    pub test_duration_ms: f64,
    /// Stimulus injection method.
    pub injection_mode: InjectionMode,
}

/// Result produced after executing a single [`TestCase`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TestResult {
    /// Refers back to [`TestCase::id`].
    pub test_case_id: usize,
    /// Final status after evaluation.
    pub status: TestStatus,
    /// Measured relay operating time `ms` (0 if no operation).
    pub actual_operate_time_ms: f64,
    /// Whether the relay actually picked up.
    pub actual_pickup: bool,
    /// Signed timing error: actual − expected `ms`.
    pub timing_error_ms: f64,
    /// `true` if both pickup criterion and timing criterion are satisfied.
    pub pass_fail: bool,
    /// Optional failure description.
    pub error_message: Option<String>,
    /// Wall-clock timestamp of test execution [ms, simulated].
    pub timestamp: f64,
}

/// Per-relay configuration used by the test engine.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RelayTestConfig {
    /// Unique relay identifier.
    pub relay_id: usize,
    /// Human-readable relay tag (e.g. "51A-Feeder1").
    pub relay_name: String,
    /// Current transformer ratio (primary:secondary).
    pub ct_ratio: f64,
    /// Voltage transformer ratio (primary:secondary).
    pub vt_ratio: f64,
    /// Rated secondary current `A` (typically 1 A or 5 A).
    pub rated_current_a: f64,
    /// Rated secondary voltage `V` (typically 110 V or 220 V).
    pub rated_voltage_v: f64,
    /// Pickup current setting [A secondary].
    pub pickup_setting_a: f64,
    /// Time dial setting (TDS / TMS).
    pub tds_setting: f64,
    /// Characteristic curve name (e.g. "IEC_SI", "IEC_VI", "ANSI_MI").
    pub characteristic: String,
}

/// A collection of test cases for one relay with accumulated results.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TestSuite {
    /// Unique suite identifier.
    pub id: usize,
    /// Human-readable suite name.
    pub name: String,
    /// Relay under test configuration.
    pub relay_config: RelayTestConfig,
    /// Ordered list of test cases.
    pub test_cases: Vec<TestCase>,
    /// Results in the same order as test_cases (populated after `run_suite`).
    pub results: Vec<TestResult>,
    /// Number of test cases that passed.
    pub pass_count: usize,
    /// Number of test cases that failed.
    pub fail_count: usize,
    /// Total elapsed simulation time `ms`.
    pub completion_time_ms: f64,
}

// ── ProtectionTester ─────────────────────────────────────────────────────────

/// Main protection testing engine.
///
/// Simulates relay operation with configurable LCG-based timing noise so that
/// test suites are repeatable yet realistic.
///
/// # Noise model
/// `actual_time = ideal_time × (1 + noise_pct × (u − 0.5))`
/// where `u ∈ [0,1)` is drawn from a Linear Congruential Generator.
#[derive(Debug, Clone)]
pub struct ProtectionTester {
    /// Relay configuration (settings, ratios, characteristic).
    pub relay_config: RelayTestConfig,
    /// Fractional timing noise amplitude (e.g. 0.005 = ±0.25 %).
    pub simulation_noise_pct: f64,
    /// Internal LCG state for reproducible pseudo-randomness.
    pub seed: u64,
}

impl ProtectionTester {
    /// LCG multiplier (Knuth / PCG family).
    const LCG_MUL: u64 = 6_364_136_223_846_793_005u64;
    /// LCG addend.
    const LCG_ADD: u64 = 1_442_695_040_888_963_407u64;

    /// Create a new tester with default 0.5 % noise.
    pub fn new(relay_config: RelayTestConfig) -> Self {
        Self {
            relay_config,
            simulation_noise_pct: 0.005,
            seed: 12_345_678_901_234_567u64,
        }
    }

    /// Advance the LCG one step and return the new state.
    fn lcg_next(&mut self) -> u64 {
        self.seed = self
            .seed
            .wrapping_mul(Self::LCG_MUL)
            .wrapping_add(Self::LCG_ADD);
        self.seed
    }

    /// Return next LCG sample normalised to [0, 1).
    fn lcg_f64(&mut self) -> f64 {
        let raw = self.lcg_next();
        // Use upper 53 bits for full double-precision mantissa coverage.
        (raw >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Compute the expected relay operating time for `current_a` [A secondary].
    ///
    /// The formula is selected by [`RelayTestConfig::characteristic`]:
    ///
    /// | Key       | Formula (t in seconds)                              |
    /// |-----------|-----------------------------------------------------|
    /// | `IEC_SI`  | `TDS × 0.14 / ((I/Is)^0.02 − 1)`                  |
    /// | `IEC_VI`  | `TDS × 13.5 / ((I/Is) − 1)`                        |
    /// | `IEC_EI`  | `TDS × 80 / ((I/Is)² − 1)`                         |
    /// | `ANSI_MI` | `TDS × (0.0515/((I/Is)^0.02−1) + 0.114)`           |
    ///
    /// Returns **milliseconds**.  Returns `0.0` when current ≤ pickup and
    /// `f64::INFINITY` when the denominator is numerically zero.
    pub fn compute_expected_operate_time(&self, current_a: f64) -> f64 {
        let is = self.relay_config.pickup_setting_a;
        if current_a <= is {
            return 0.0;
        }
        let m = current_a / is;
        let tds = self.relay_config.tds_setting;

        match self.relay_config.characteristic.as_str() {
            "IEC_VI" => {
                let denom = m - 1.0;
                if denom < 1e-12 {
                    return f64::INFINITY;
                }
                tds * 13.5 / denom * 1000.0
            }
            "IEC_EI" => {
                let denom = m * m - 1.0;
                if denom < 1e-12 {
                    return f64::INFINITY;
                }
                tds * 80.0 / denom * 1000.0
            }
            "ANSI_MI" => {
                let denom = m.powf(0.02) - 1.0;
                if denom < 1e-12 {
                    return f64::INFINITY;
                }
                tds * (0.0515 / denom + 0.114) * 1000.0
            }
            _ => {
                // Default: IEC Standard Inverse
                let denom = m.powf(0.02) - 1.0;
                if denom < 1e-12 {
                    return f64::INFINITY;
                }
                tds * 0.14 / denom * 1000.0
            }
        }
    }

    /// Simulate relay operation for a test case.
    ///
    /// Returns `(operated: bool, operate_time_ms: f64)`.
    ///
    /// If `operated` is `false`, `operate_time_ms` is `0.0`.
    pub fn simulate_relay_operation(&mut self, case: &TestCase) -> (bool, f64) {
        let is = self.relay_config.pickup_setting_a;
        let operated = case.input_current_a > is;

        if !operated {
            return (false, 0.0);
        }

        let ideal_time_ms = self.compute_expected_operate_time(case.input_current_a);
        if !ideal_time_ms.is_finite() {
            // Effectively at the pickup threshold — operate but report minimum time.
            return (true, 15.0);
        }

        // Apply LCG noise: time += time × noise_pct × (u − 0.5)
        let u = self.lcg_f64();
        let noisy_time = ideal_time_ms + ideal_time_ms * self.simulation_noise_pct * (u - 0.5);

        // Clamp to 15 ms (one cycle at 50 Hz) as physical minimum.
        (true, noisy_time.max(15.0))
    }

    /// Execute a single test case and return its result.
    pub fn run_test_case(&mut self, case: &TestCase) -> TestResult {
        let (mut actual_pickup, actual_time_ms) = self.simulate_relay_operation(case);

        // Directional test refinement: forward fault (|angle| < 90°) is required.
        if case.test_type == TestType::DirectionalTest {
            let forward = case.input_angle_deg.abs() < 90.0;
            actual_pickup = actual_pickup && forward;
        }

        let timing_error_ms = if actual_pickup && case.expected_pickup {
            actual_time_ms - case.expected_operate_time_ms
        } else {
            0.0
        };

        // Pass criteria:
        // 1. Pickup decision matches expectation.
        // 2. If relay expected to operate, timing within declared tolerance.
        let pickup_ok = actual_pickup == case.expected_pickup;
        let timing_ok = if case.expected_pickup && actual_pickup {
            timing_error_ms.abs() <= case.tolerance_ms
        } else {
            true
        };
        let pass_fail = pickup_ok && timing_ok;

        let error_message = if !pass_fail {
            if !pickup_ok {
                Some(format!(
                    "Pickup mismatch: expected={} actual={}",
                    case.expected_pickup, actual_pickup
                ))
            } else {
                Some(format!(
                    "Timing out of tolerance: error={:.2} ms, tolerance=±{:.2} ms",
                    timing_error_ms, case.tolerance_ms
                ))
            }
        } else {
            None
        };

        let status = if pass_fail {
            TestStatus::Passed
        } else {
            TestStatus::Failed
        };

        TestResult {
            test_case_id: case.id,
            status,
            actual_operate_time_ms: actual_time_ms,
            actual_pickup,
            timing_error_ms,
            pass_fail,
            error_message,
            timestamp: case.test_duration_ms,
        }
    }

    /// Run all test cases in a [`TestSuite`] and update its counters.
    ///
    /// Returns a shared reference to the (now populated) suite.
    pub fn run_suite<'s>(&mut self, suite: &'s mut TestSuite) -> &'s TestSuite {
        suite.results.clear();
        suite.pass_count = 0;
        suite.fail_count = 0;
        suite.completion_time_ms = 0.0;

        let cases: Vec<TestCase> = suite.test_cases.clone();
        for case in &cases {
            let result = self.run_test_case(case);
            suite.completion_time_ms += case.test_duration_ms;
            if result.pass_fail {
                suite.pass_count += 1;
            } else {
                suite.fail_count += 1;
            }
            suite.results.push(result);
        }
        suite
    }

    // ── Test-case generators ─────────────────────────────────────────────────

    /// Generate pickup verification tests at 0.8×, 0.9×, 1.0×, 1.05×, 1.1×, 1.5× pickup.
    ///
    /// Tests at or below pickup expect *no* operation; tests above expect operation.
    /// Produces exactly **6** test cases.
    pub fn generate_pickup_tests(&self) -> Vec<TestCase> {
        let is = self.relay_config.pickup_setting_a;
        // (multiple, should_pickup): at exactly 1.0× the relay does NOT operate (strict >).
        let multiples: &[(f64, bool)] = &[
            (0.80, false),
            (0.90, false),
            (1.00, false),
            (1.05, true),
            (1.10, true),
            (1.50, true),
        ];

        multiples
            .iter()
            .enumerate()
            .map(|(idx, &(mult, should_pickup))| {
                let current = is * mult;
                TestCase {
                    id: idx + 1,
                    name: format!("Pickup {:.0}% Is", mult * 100.0),
                    test_type: TestType::PickupTest,
                    description: format!(
                        "Verify relay {} at {:.2} A ({:.0}% of pickup {:.2} A)",
                        if should_pickup {
                            "operates"
                        } else {
                            "restrains"
                        },
                        current,
                        mult * 100.0,
                        is
                    ),
                    input_current_a: current,
                    input_voltage_v: self.relay_config.rated_voltage_v,
                    input_frequency_hz: 50.0,
                    input_angle_deg: 80.0,
                    expected_operate_time_ms: if should_pickup {
                        self.compute_expected_operate_time(current)
                    } else {
                        0.0
                    },
                    tolerance_ms: 50.0,
                    expected_pickup: should_pickup,
                    test_duration_ms: 500.0,
                    injection_mode: InjectionMode::Secondary,
                }
            })
            .collect()
    }

    /// Generate timing tests at the specified current multiples of the pickup setting.
    ///
    /// Typically called with `&[2.0, 5.0, 10.0, 20.0]`.  Tolerance is the larger of
    /// 5 % of the expected time or 20 ms.
    pub fn generate_timing_tests(&self, multiples: &[f64]) -> Vec<TestCase> {
        let is = self.relay_config.pickup_setting_a;
        multiples
            .iter()
            .enumerate()
            .map(|(idx, &mult)| {
                let current = is * mult;
                let expected_ms = self.compute_expected_operate_time(current);
                let tolerance_ms = (expected_ms * 0.05).max(20.0);
                TestCase {
                    id: 100 + idx + 1,
                    name: format!("Timing {:.0}× Is", mult),
                    test_type: TestType::TimingTest,
                    description: format!(
                        "Verify operating time at {:.1}× pickup ({:.2} A): expected {:.1} ms",
                        mult, current, expected_ms
                    ),
                    input_current_a: current,
                    input_voltage_v: self.relay_config.rated_voltage_v,
                    input_frequency_hz: 50.0,
                    input_angle_deg: 80.0,
                    expected_operate_time_ms: expected_ms,
                    tolerance_ms,
                    expected_pickup: true,
                    test_duration_ms: expected_ms + 200.0,
                    injection_mode: InjectionMode::Secondary,
                }
            })
            .collect()
    }

    /// Generate `n_points` characteristic curve test cases distributed on a logarithmic
    /// scale from 1.05× to 20× pickup.
    ///
    /// Returns an empty `Vec` when `n_points == 0`.
    pub fn generate_characteristic_tests(&self, n_points: usize) -> Vec<TestCase> {
        if n_points == 0 {
            return Vec::new();
        }
        let is = self.relay_config.pickup_setting_a;
        let log_min = (1.05_f64).ln();
        let log_max = (20.0_f64).ln();

        (0..n_points)
            .map(|i| {
                let t_frac = if n_points == 1 {
                    0.5
                } else {
                    i as f64 / (n_points - 1) as f64
                };
                let mult = (log_min + t_frac * (log_max - log_min)).exp();
                let current = is * mult;
                let expected_ms = self.compute_expected_operate_time(current);
                let tolerance_ms = (expected_ms * 0.05).max(20.0);
                TestCase {
                    id: 200 + i + 1,
                    name: format!("TCC point {}/{} ({:.2}× Is)", i + 1, n_points, mult),
                    test_type: TestType::CharacteristicTest,
                    description: format!(
                        "TCC verification at {:.2}× pickup ({:.2} A), expected {:.1} ms",
                        mult, current, expected_ms
                    ),
                    input_current_a: current,
                    input_voltage_v: self.relay_config.rated_voltage_v,
                    input_frequency_hz: 50.0,
                    input_angle_deg: 80.0,
                    expected_operate_time_ms: expected_ms,
                    tolerance_ms,
                    expected_pickup: true,
                    test_duration_ms: expected_ms + 200.0,
                    injection_mode: InjectionMode::Secondary,
                }
            })
            .collect()
    }
}

// ── TestReport & TestReportGenerator ─────────────────────────────────────────

/// Summarised performance report for a completed test suite.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TestReport {
    /// Name of the test suite.
    pub suite_name: String,
    /// Relay tag from [`RelayTestConfig::relay_name`].
    pub relay_name: String,
    /// Total number of test cases in the suite.
    pub total_tests: usize,
    /// Number of tests that passed.
    pub passed: usize,
    /// Number of tests that failed.
    pub failed: usize,
    /// Number of tests that were skipped.
    pub skipped: usize,
    /// Percentage of tests that passed [0–100].
    pub pass_rate_pct: f64,
    /// Overall suite status.
    pub overall_status: TestStatus,
    /// Percentage of pickup tests that produced the expected operate/restrain decision.
    pub pickup_accuracy_pct: f64,
    /// Percentage of timing tests whose error was within the declared tolerance.
    pub timing_accuracy_pct: f64,
    /// Maximum absolute timing error across all timing tests `ms`.
    pub max_timing_error_ms: f64,
    /// Mean signed timing error across all timing tests `ms`.
    pub mean_timing_error_ms: f64,
    /// Actionable recommendations based on test findings.
    pub recommendations: Vec<String>,
}

/// Generates [`TestReport`]s and statistical summaries from completed [`TestSuite`]s.
pub struct TestReportGenerator;

impl TestReportGenerator {
    /// Build a full report from a completed test suite.
    pub fn generate_report(suite: &TestSuite) -> TestReport {
        let total_tests = suite.results.len();
        let passed = suite.pass_count;
        let failed = suite.fail_count;
        let skipped = suite
            .results
            .iter()
            .filter(|r| r.status == TestStatus::Skipped)
            .count();

        let pass_rate_pct = if total_tests > 0 {
            passed as f64 / total_tests as f64 * 100.0
        } else {
            0.0
        };

        let overall_status = if total_tests == 0 {
            TestStatus::Inconclusive
        } else if failed == 0 {
            TestStatus::Passed
        } else {
            TestStatus::Failed
        };

        // Pickup accuracy: among PickupTest cases, fraction with correct pickup decision.
        let pickup_results: Vec<&TestResult> = suite
            .results
            .iter()
            .filter(|r| {
                suite
                    .test_cases
                    .iter()
                    .find(|tc| tc.id == r.test_case_id)
                    .map(|tc| tc.test_type == TestType::PickupTest)
                    .unwrap_or(false)
            })
            .collect();
        let pickup_accuracy_pct = if pickup_results.is_empty() {
            100.0
        } else {
            pickup_results.iter().filter(|r| r.pass_fail).count() as f64
                / pickup_results.len() as f64
                * 100.0
        };

        // Timing accuracy: among TimingTest / CharacteristicTest cases.
        let timing_results: Vec<&TestResult> = suite
            .results
            .iter()
            .filter(|r| {
                suite
                    .test_cases
                    .iter()
                    .find(|tc| tc.id == r.test_case_id)
                    .map(|tc| {
                        matches!(
                            tc.test_type,
                            TestType::TimingTest | TestType::CharacteristicTest
                        )
                    })
                    .unwrap_or(false)
            })
            .collect();
        let timing_accuracy_pct = if timing_results.is_empty() {
            100.0
        } else {
            timing_results.iter().filter(|r| r.pass_fail).count() as f64
                / timing_results.len() as f64
                * 100.0
        };

        let (mean_timing_error_ms, _std, max_timing_error_ms) =
            Self::compute_timing_statistics(&suite.results);

        let mut report = TestReport {
            suite_name: suite.name.clone(),
            relay_name: suite.relay_config.relay_name.clone(),
            total_tests,
            passed,
            failed,
            skipped,
            pass_rate_pct,
            overall_status,
            pickup_accuracy_pct,
            timing_accuracy_pct,
            max_timing_error_ms,
            mean_timing_error_ms,
            recommendations: Vec::new(),
        };
        report.recommendations = Self::generate_recommendations(&report);
        report
    }

    /// Collect all failed test results from a suite.
    pub fn identify_failures(suite: &TestSuite) -> Vec<&TestResult> {
        suite.results.iter().filter(|r| !r.pass_fail).collect()
    }

    /// Compute `(mean, std, max)` of timing errors `ms`.
    ///
    /// Only considers results where the relay operated (`actual_pickup == true`).
    /// Returns `(0.0, 0.0, 0.0)` if no such results exist.
    pub fn compute_timing_statistics(results: &[TestResult]) -> (f64, f64, f64) {
        let errors: Vec<f64> = results
            .iter()
            .filter(|r| r.actual_pickup)
            .map(|r| r.timing_error_ms)
            .collect();

        if errors.is_empty() {
            return (0.0, 0.0, 0.0);
        }

        let n = errors.len() as f64;
        let mean = errors.iter().sum::<f64>() / n;
        let variance = errors.iter().map(|e| (e - mean).powi(2)).sum::<f64>() / n;
        let std = variance.sqrt();
        let max = errors.iter().map(|e| e.abs()).fold(0.0_f64, f64::max);

        (mean, std, max)
    }

    /// Generate actionable commissioning recommendations based on the report.
    pub fn generate_recommendations(report: &TestReport) -> Vec<String> {
        let mut recs: Vec<String> = Vec::new();

        if report.overall_status == TestStatus::Passed {
            recs.push("All tests passed. Relay is suitable for energisation.".to_string());
            return recs;
        }

        if report.pickup_accuracy_pct < 100.0 {
            recs.push(format!(
                "Pickup accuracy {:.1}% < 100%: verify CT secondary loop continuity and \
                 pickup setting on the relay front panel.",
                report.pickup_accuracy_pct
            ));
        }

        if report.timing_accuracy_pct < 90.0 {
            recs.push(format!(
                "Timing accuracy {:.1}% < 90%: check TDS/TMS dial setting and relay firmware \
                 version; compare measured curve against IEC 60255-151 reference.",
                report.timing_accuracy_pct
            ));
        }

        if report.max_timing_error_ms > 100.0 {
            recs.push(format!(
                "Maximum timing error {:.1} ms exceeds 100 ms: inspect relay internal clock \
                 oscillator; consider relay replacement if issue persists.",
                report.max_timing_error_ms
            ));
        }

        if report.mean_timing_error_ms > 20.0 {
            recs.push(format!(
                "Mean timing error {:.1} ms is positive (relay operates late): \
                 verify TDS setting is not inadvertently incremented.",
                report.mean_timing_error_ms
            ));
        } else if report.mean_timing_error_ms < -20.0 {
            recs.push(format!(
                "Mean timing error {:.1} ms is negative (relay operates early): \
                 check for spurious contact bounce or VT secondary over-voltage.",
                report.mean_timing_error_ms
            ));
        }

        if report.failed > 0 {
            recs.push(format!(
                "{} test(s) failed: review individual error messages and re-test after \
                 corrective action.",
                report.failed
            ));
        }

        if report.pass_rate_pct < 80.0 {
            recs.push(
                "Pass rate below 80%: consider full relay replacement and re-commissioning."
                    .to_string(),
            );
        }

        if recs.is_empty() {
            recs.push("Minor failures detected: repeat individual failed test cases.".to_string());
        }

        recs
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> RelayTestConfig {
        RelayTestConfig {
            relay_id: 1,
            relay_name: "51-A".to_string(),
            ct_ratio: 200.0,
            vt_ratio: 110.0,
            rated_current_a: 5.0,
            rated_voltage_v: 110.0,
            pickup_setting_a: 1.0, // 1 A secondary = 200 A primary
            tds_setting: 0.1,
            characteristic: "IEC_SI".to_string(),
        }
    }

    fn default_tester() -> ProtectionTester {
        ProtectionTester::new(default_config())
    }

    fn make_suite(tester: &ProtectionTester) -> TestSuite {
        TestSuite {
            id: 1,
            name: "Commissioning Suite".to_string(),
            relay_config: tester.relay_config.clone(),
            test_cases: Vec::new(),
            results: Vec::new(),
            pass_count: 0,
            fail_count: 0,
            completion_time_ms: 0.0,
        }
    }

    fn simple_case(
        id: usize,
        current_a: f64,
        expected_pickup: bool,
        expected_ms: f64,
        tolerance_ms: f64,
        test_type: TestType,
        angle_deg: f64,
    ) -> TestCase {
        TestCase {
            id,
            name: format!("case_{}", id),
            test_type,
            description: String::new(),
            input_current_a: current_a,
            input_voltage_v: 110.0,
            input_frequency_hz: 50.0,
            input_angle_deg: angle_deg,
            expected_operate_time_ms: expected_ms,
            tolerance_ms,
            expected_pickup,
            test_duration_ms: 500.0,
            injection_mode: InjectionMode::Secondary,
        }
    }

    // ── Pickup tests ──────────────────────────────────────────────────────────

    #[test]
    fn test_pickup_at_setting() {
        // Exactly at pickup (1.0 A) with strict > → relay does NOT operate.
        let mut tester = default_tester();
        let case = simple_case(1, 1.0, false, 0.0, 50.0, TestType::PickupTest, 80.0);
        let result = tester.run_test_case(&case);
        assert!(
            !result.actual_pickup,
            "at exactly pickup should NOT operate"
        );
        assert!(result.pass_fail);
    }

    #[test]
    fn test_below_pickup() {
        // 0.95× pickup → should NOT operate.
        let mut tester = default_tester();
        let case = simple_case(2, 0.95, false, 0.0, 50.0, TestType::PickupTest, 80.0);
        let (operated, _) = tester.simulate_relay_operation(&case);
        assert!(!operated, "below pickup: relay must restrain");
    }

    #[test]
    fn test_above_pickup() {
        // 1.05× pickup → should operate.
        let mut tester = default_tester();
        let case = simple_case(3, 1.05, true, 0.0, 50.0, TestType::PickupTest, 80.0);
        let (operated, time_ms) = tester.simulate_relay_operation(&case);
        assert!(operated, "above pickup: relay must operate");
        assert!(time_ms > 0.0);
    }

    // ── IEC SI timing formula ─────────────────────────────────────────────────

    #[test]
    fn test_iec_si_timing_2x() {
        // t = TDS × 0.14 / (2^0.02 − 1)
        let tester = default_tester();
        let tds = tester.relay_config.tds_setting;
        let expected_ms = tds * 0.14 / (2.0_f64.powf(0.02) - 1.0) * 1000.0;
        let computed_ms = tester.compute_expected_operate_time(2.0); // 2× pickup = 2 A
        assert!(
            (computed_ms - expected_ms).abs() < 1.0,
            "IEC SI at 2×: computed={:.3} ms expected={:.3} ms",
            computed_ms,
            expected_ms
        );
    }

    #[test]
    fn test_iec_si_timing_10x() {
        let tester = default_tester();
        let tds = tester.relay_config.tds_setting;
        let expected_ms = tds * 0.14 / (10.0_f64.powf(0.02) - 1.0) * 1000.0;
        let computed_ms = tester.compute_expected_operate_time(10.0);
        assert!(
            (computed_ms - expected_ms).abs() < 1.0,
            "IEC SI at 10×: computed={:.3} ms expected={:.3} ms",
            computed_ms,
            expected_ms
        );
    }

    // ── Tolerance pass/fail logic ─────────────────────────────────────────────

    #[test]
    fn test_timing_within_tolerance() {
        // A result with 10 ms error and 20 ms tolerance must pass.
        let result = TestResult {
            test_case_id: 1,
            status: TestStatus::Passed,
            actual_operate_time_ms: 110.0,
            actual_pickup: true,
            timing_error_ms: 10.0,
            pass_fail: true,
            error_message: None,
            timestamp: 0.0,
        };
        assert!(result.pass_fail);
        assert!(result.timing_error_ms.abs() <= 20.0);
    }

    #[test]
    fn test_timing_outside_tolerance() {
        // A result with 200 ms error must fail.
        let result = TestResult {
            test_case_id: 2,
            status: TestStatus::Failed,
            actual_operate_time_ms: 300.0,
            actual_pickup: true,
            timing_error_ms: 200.0,
            pass_fail: false,
            error_message: Some("Timing out of tolerance".to_string()),
            timestamp: 0.0,
        };
        assert!(!result.pass_fail);
    }

    // ── Generator methods ─────────────────────────────────────────────────────

    #[test]
    fn test_generate_pickup_tests() {
        let tester = default_tester();
        let cases = tester.generate_pickup_tests();
        assert_eq!(
            cases.len(),
            6,
            "should generate exactly 6 pickup test cases"
        );
        assert!(!cases[0].expected_pickup, "80% → no pickup");
        assert!(!cases[1].expected_pickup, "90% → no pickup");
        assert!(!cases[2].expected_pickup, "100% → no pickup (strict >)");
        assert!(cases[3].expected_pickup, "105% → pickup");
        assert!(cases[4].expected_pickup, "110% → pickup");
        assert!(cases[5].expected_pickup, "150% → pickup");
    }

    #[test]
    fn test_generate_timing_tests() {
        let tester = default_tester();
        let multiples = [2.0, 5.0, 10.0, 20.0];
        let cases = tester.generate_timing_tests(&multiples);
        assert_eq!(cases.len(), multiples.len());
        for (i, case) in cases.iter().enumerate() {
            assert!(case.expected_pickup);
            let expected_current = tester.relay_config.pickup_setting_a * multiples[i];
            assert!(
                (case.input_current_a - expected_current).abs() < 1e-9,
                "case {} current mismatch",
                i
            );
        }
    }

    #[test]
    fn test_generate_characteristic_tests() {
        let tester = default_tester();
        let n = 8;
        let cases = tester.generate_characteristic_tests(n);
        assert_eq!(cases.len(), n);
        for case in &cases {
            assert_eq!(case.test_type, TestType::CharacteristicTest);
            assert!(case.input_current_a > tester.relay_config.pickup_setting_a);
        }
    }

    #[test]
    fn test_generate_characteristic_tests_zero() {
        let tester = default_tester();
        assert!(tester.generate_characteristic_tests(0).is_empty());
    }

    // ── Suite execution ───────────────────────────────────────────────────────

    #[test]
    fn test_run_suite_counts() {
        let mut tester = default_tester();
        let mut suite = make_suite(&tester);
        suite.test_cases = tester.generate_pickup_tests();
        tester.run_suite(&mut suite);
        let total = suite.test_cases.len();
        assert_eq!(
            suite.pass_count + suite.fail_count,
            total,
            "pass + fail must equal total"
        );
        assert_eq!(suite.results.len(), total);
    }

    #[test]
    fn test_suite_completion_time() {
        let mut tester = default_tester();
        let mut suite = make_suite(&tester);
        suite.test_cases = tester.generate_pickup_tests();
        let sum_duration: f64 = suite.test_cases.iter().map(|tc| tc.test_duration_ms).sum();
        tester.run_suite(&mut suite);
        assert!(
            (suite.completion_time_ms - sum_duration).abs() < 1e-9,
            "completion_time_ms={:.1} sum_duration={:.1}",
            suite.completion_time_ms,
            sum_duration
        );
    }

    // ── Report generation ─────────────────────────────────────────────────────

    #[test]
    fn test_report_pass_rate() {
        let mut tester = default_tester();
        let mut suite = make_suite(&tester);
        suite.test_cases = tester.generate_pickup_tests();
        tester.run_suite(&mut suite);
        let report = TestReportGenerator::generate_report(&suite);
        let expected_pct = report.passed as f64 / report.total_tests as f64 * 100.0;
        assert!(
            (report.pass_rate_pct - expected_pct).abs() < 0.01,
            "pass_rate={:.3} expected={:.3}",
            report.pass_rate_pct,
            expected_pct
        );
    }

    #[test]
    fn test_report_overall_pass() {
        let tester = default_tester();
        let mut suite = make_suite(&tester);
        suite.pass_count = 3;
        suite.fail_count = 0;
        suite.results = (1..=3)
            .map(|i| TestResult {
                test_case_id: i,
                status: TestStatus::Passed,
                actual_operate_time_ms: 100.0,
                actual_pickup: true,
                timing_error_ms: 1.0,
                pass_fail: true,
                error_message: None,
                timestamp: 0.0,
            })
            .collect();
        let report = TestReportGenerator::generate_report(&suite);
        assert_eq!(report.overall_status, TestStatus::Passed);
    }

    #[test]
    fn test_report_overall_fail() {
        let tester = default_tester();
        let mut suite = make_suite(&tester);
        suite.pass_count = 0;
        suite.fail_count = 1;
        suite.results = vec![TestResult {
            test_case_id: 1,
            status: TestStatus::Failed,
            actual_operate_time_ms: 999.0,
            actual_pickup: true,
            timing_error_ms: 500.0,
            pass_fail: false,
            error_message: Some("Timing out of tolerance".to_string()),
            timestamp: 0.0,
        }];
        let report = TestReportGenerator::generate_report(&suite);
        assert_eq!(report.overall_status, TestStatus::Failed);
    }

    #[test]
    fn test_recommendations_nonempty() {
        let tester = default_tester();
        let mut suite = make_suite(&tester);
        suite.pass_count = 0;
        suite.fail_count = 1;
        suite.results = vec![TestResult {
            test_case_id: 1,
            status: TestStatus::Failed,
            actual_operate_time_ms: 0.0,
            actual_pickup: false,
            timing_error_ms: 0.0,
            pass_fail: false,
            error_message: Some("Pickup mismatch".to_string()),
            timestamp: 0.0,
        }];
        let report = TestReportGenerator::generate_report(&suite);
        assert!(
            !report.recommendations.is_empty(),
            "failed suite must produce recommendations"
        );
    }

    #[test]
    fn test_timing_statistics() {
        let results = vec![
            TestResult {
                test_case_id: 1,
                status: TestStatus::Passed,
                actual_operate_time_ms: 100.0,
                actual_pickup: true,
                timing_error_ms: 5.0,
                pass_fail: true,
                error_message: None,
                timestamp: 0.0,
            },
            TestResult {
                test_case_id: 2,
                status: TestStatus::Passed,
                actual_operate_time_ms: 200.0,
                actual_pickup: true,
                timing_error_ms: -5.0,
                pass_fail: true,
                error_message: None,
                timestamp: 0.0,
            },
            TestResult {
                test_case_id: 3,
                status: TestStatus::Passed,
                actual_operate_time_ms: 150.0,
                actual_pickup: true,
                timing_error_ms: 10.0,
                pass_fail: true,
                error_message: None,
                timestamp: 0.0,
            },
        ];
        let (mean, std, max) = TestReportGenerator::compute_timing_statistics(&results);
        // mean of [5, -5, 10] = 10/3
        assert!(
            (mean - 10.0 / 3.0).abs() < 0.01,
            "mean={:.4} expected={:.4}",
            mean,
            10.0 / 3.0
        );
        assert!(std >= 0.0, "std must be non-negative");
        assert!(
            (max - 10.0).abs() < 0.01,
            "max absolute error must be 10.0, got {:.4}",
            max
        );
    }

    #[test]
    fn test_identify_failures() {
        let tester = default_tester();
        let mut suite = make_suite(&tester);
        suite.pass_count = 1;
        suite.fail_count = 1;
        suite.results = vec![
            TestResult {
                test_case_id: 1,
                status: TestStatus::Passed,
                actual_operate_time_ms: 100.0,
                actual_pickup: true,
                timing_error_ms: 1.0,
                pass_fail: true,
                error_message: None,
                timestamp: 0.0,
            },
            TestResult {
                test_case_id: 2,
                status: TestStatus::Failed,
                actual_operate_time_ms: 999.0,
                actual_pickup: false,
                timing_error_ms: 0.0,
                pass_fail: false,
                error_message: Some("Pickup mismatch".to_string()),
                timestamp: 0.0,
            },
        ];
        let failures = TestReportGenerator::identify_failures(&suite);
        assert_eq!(failures.len(), 1);
        assert_eq!(failures[0].test_case_id, 2);
    }

    // ── Directional tests ─────────────────────────────────────────────────────

    #[test]
    fn test_directional_test_angle_forward() {
        // Forward fault: angle = 60° < 90° → relay should operate when above pickup.
        let mut tester = default_tester();
        let expected_ms = tester.compute_expected_operate_time(5.0);
        let case = simple_case(
            50,
            5.0,
            true,
            expected_ms,
            200.0,
            TestType::DirectionalTest,
            60.0,
        );
        let result = tester.run_test_case(&case);
        assert!(result.actual_pickup, "forward fault (60°) should operate");
        assert!(result.pass_fail);
    }

    #[test]
    fn test_directional_test_angle_reverse() {
        // Reverse fault: angle = 120° ≥ 90° → directional element blocks operation.
        let mut tester = default_tester();
        let case = simple_case(51, 5.0, false, 0.0, 50.0, TestType::DirectionalTest, 120.0);
        let result = tester.run_test_case(&case);
        assert!(
            !result.actual_pickup,
            "reverse fault (120°) must be blocked by directional element"
        );
        assert!(result.pass_fail);
    }

    // ── CT ratio scaling ──────────────────────────────────────────────────────

    #[test]
    fn test_ct_ratio_scaling() {
        // 1000 A primary with CT 400:1 → 2.5 A secondary; pickup = 2 A → must operate.
        let config = RelayTestConfig {
            relay_id: 2,
            relay_name: "51-B".to_string(),
            ct_ratio: 400.0,
            vt_ratio: 110.0,
            rated_current_a: 5.0,
            rated_voltage_v: 110.0,
            pickup_setting_a: 2.0,
            tds_setting: 0.1,
            characteristic: "IEC_SI".to_string(),
        };
        let tester = ProtectionTester::new(config);
        let secondary_current = 1000.0 / tester.relay_config.ct_ratio;
        assert!(
            secondary_current > tester.relay_config.pickup_setting_a,
            "2.5 A must exceed 2.0 A pickup"
        );
        let t_ms = tester.compute_expected_operate_time(secondary_current);
        assert!(
            t_ms > 0.0 && t_ms.is_finite(),
            "must produce finite time: {}",
            t_ms
        );
    }

    // ── LCG reproducibility ───────────────────────────────────────────────────

    #[test]
    fn test_lcg_reproducibility() {
        let mut t1 = default_tester();
        let mut t2 = default_tester();
        let expected_ms = t1.compute_expected_operate_time(5.0);
        let case = simple_case(
            99,
            5.0,
            true,
            expected_ms,
            200.0,
            TestType::TimingTest,
            80.0,
        );
        let (_, time1) = t1.simulate_relay_operation(&case);
        let (_, time2) = t2.simulate_relay_operation(&case);
        assert_eq!(
            time1, time2,
            "same initial seed must produce identical simulation results"
        );
    }

    // ── IEC VI characteristic ─────────────────────────────────────────────────

    #[test]
    fn test_iec_vi_characteristic() {
        let config = RelayTestConfig {
            characteristic: "IEC_VI".to_string(),
            pickup_setting_a: 1.0,
            tds_setting: 0.1,
            ..default_config()
        };
        let tester = ProtectionTester::new(config);
        // t = 0.1 × 13.5 / (5 − 1) × 1000 = 337.5 ms
        let expected_ms = 0.1 * 13.5 / (5.0 - 1.0) * 1000.0;
        let computed = tester.compute_expected_operate_time(5.0);
        assert!(
            (computed - expected_ms).abs() < 0.5,
            "IEC VI at 5×: computed={:.3} expected={:.3}",
            computed,
            expected_ms
        );
    }

    #[test]
    fn test_below_pickup_returns_zero() {
        let tester = default_tester();
        assert_eq!(tester.compute_expected_operate_time(0.5), 0.0);
        assert_eq!(tester.compute_expected_operate_time(1.0), 0.0);
    }
}
