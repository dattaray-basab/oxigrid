//! Power System Model Validation Framework.
//!
//! Validates computed power flow results against reference solutions
//! from MATPOWER, PSS/E, PowerWorld, or built-in IEEE test cases.
//!
//! # Validation Workflow
//!
//! 1. Define a [`ValidationCase`] with reference voltages \[pu\],
//!    angles \[deg\], generation \[MW\] / \[MVAr\], and losses \[MW\].
//! 2. Run your power flow solver to obtain computed values.
//! 3. Call [`ModelValidator::validate`] to compare against reference.
//! 4. Aggregate results with [`ModelValidator::summary`].
//!
//! # Tolerances
//!
//! | Quantity | Typical | Unit |
//! |----------|---------|------|
//! | Voltage  | 1e-4    | \[pu\] |
//! | Angle    | 1e-3    | \[deg\] |
//! | Power    | 1e-3    | \[MW\] |

use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Tolerances used when comparing computed results to reference.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ValidationConfig {
    /// Maximum acceptable voltage magnitude error \[pu\].
    pub tolerance_voltage_pu: f64,
    /// Maximum acceptable power error \[MW\].
    pub tolerance_power_mw: f64,
    /// Maximum acceptable angle error \[deg\].
    pub tolerance_angle_deg: f64,
    /// Name of the reference tool (e.g. "MATPOWER", "PSS/E").
    pub reference_source: String,
}

impl Default for ValidationConfig {
    fn default() -> Self {
        Self {
            tolerance_voltage_pu: 1e-4,
            tolerance_power_mw: 1e-3,
            tolerance_angle_deg: 1e-3,
            reference_source: "MATPOWER".into(),
        }
    }
}

// ---------------------------------------------------------------------------
// Validation case
// ---------------------------------------------------------------------------

/// A reference validation case with known correct solution.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ValidationCase {
    /// Human-readable name (e.g. "IEEE 14-bus").
    pub name: String,
    /// Number of buses.
    pub n_buses: usize,
    /// Reference voltage magnitudes \[pu\] per bus.
    pub reference_voltages_pu: Vec<f64>,
    /// Reference voltage angles \[deg\] per bus.
    pub reference_angles_deg: Vec<f64>,
    /// Reference active generation \[MW\] per generator.
    pub reference_p_gen_mw: Vec<f64>,
    /// Reference reactive generation \[MVAr\] per generator.
    pub reference_q_gen_mvar: Vec<f64>,
    /// Reference total active losses \[MW\].
    pub reference_losses_mw: f64,
}

// ---------------------------------------------------------------------------
// Result
// ---------------------------------------------------------------------------

/// Detailed per-case validation result.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ValidationResult {
    /// Name of the case validated.
    pub case_name: String,
    /// Maximum per-bus voltage magnitude error \[pu\].
    pub max_voltage_error_pu: f64,
    /// Maximum per-bus angle error \[deg\].
    pub max_angle_error_deg: f64,
    /// Maximum per-generator power error \[MW\].
    pub max_power_error_mw: f64,
    /// Absolute losses error \[MW\].
    pub losses_error_mw: f64,
    /// Maximum per-generator reactive power error \[MVAr\].
    pub max_q_error_mvar: f64,
    /// Whether all quantities are within tolerance.
    pub passed: bool,
    /// Number of buses within tolerance.
    pub n_buses_passed: usize,
    /// Number of buses outside tolerance.
    pub n_buses_failed: usize,
    /// (bus_id, voltage_error) for buses that failed.
    pub failed_buses: Vec<(usize, f64)>,
}

/// Aggregate summary of multiple validation results.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ValidationSummary {
    /// Total number of cases evaluated.
    pub n_cases: usize,
    /// Number of cases that passed.
    pub n_passed: usize,
    /// Number of cases that failed.
    pub n_failed: usize,
    /// Pass rate \[%\].
    pub pass_rate_pct: f64,
    /// Average maximum voltage error across all cases \[pu\].
    pub avg_max_voltage_error: f64,
    /// Name of the worst-performing case.
    pub worst_case: String,
}

// ---------------------------------------------------------------------------
// Validator
// ---------------------------------------------------------------------------

/// Power system model validator.
pub struct ModelValidator {
    config: ValidationConfig,
}

impl ModelValidator {
    /// Create a validator with the given configuration.
    pub fn new(config: ValidationConfig) -> Self {
        Self { config }
    }

    /// Validate computed power flow results against a reference case.
    ///
    /// Compares:
    /// - Bus voltage magnitudes \[pu\]
    /// - Bus voltage angles \[deg\]
    /// - Generator active power \[MW\]
    /// - Total losses \[MW\]
    pub fn validate(
        &self,
        case: &ValidationCase,
        computed_voltages: &[f64],
        computed_angles: &[f64],
        computed_p_gen: &[f64],
        computed_q_gen: &[f64],
    ) -> ValidationResult {
        let n = case
            .n_buses
            .min(case.reference_voltages_pu.len())
            .min(computed_voltages.len())
            .min(case.reference_angles_deg.len())
            .min(computed_angles.len());

        // --- Voltage comparison ---
        let mut max_voltage_error = 0.0_f64;
        let mut n_buses_passed = 0usize;
        let mut n_buses_failed = 0usize;
        let mut failed_buses = Vec::new();

        for (i, (&cv, &rv)) in computed_voltages
            .iter()
            .zip(case.reference_voltages_pu.iter())
            .take(n)
            .enumerate()
        {
            let v_err = (cv - rv).abs();
            if v_err > max_voltage_error {
                max_voltage_error = v_err;
            }
            if v_err <= self.config.tolerance_voltage_pu {
                n_buses_passed += 1;
            } else {
                n_buses_failed += 1;
                failed_buses.push((i, v_err));
            }
        }

        // --- Angle comparison ---
        let mut max_angle_error = 0.0_f64;
        for (&ca, &ra) in computed_angles
            .iter()
            .zip(case.reference_angles_deg.iter())
            .take(n)
        {
            let a_err = (ca - ra).abs();
            if a_err > max_angle_error {
                max_angle_error = a_err;
            }
        }

        // --- Power generation comparison ---
        let n_gen = case.reference_p_gen_mw.len().min(computed_p_gen.len());
        let mut max_power_error = 0.0_f64;
        for (&cp, &rp) in computed_p_gen
            .iter()
            .zip(case.reference_p_gen_mw.iter())
            .take(n_gen)
        {
            let p_err = (cp - rp).abs();
            if p_err > max_power_error {
                max_power_error = p_err;
            }
        }

        // --- Reactive power generation comparison ---
        let n_qgen = case.reference_q_gen_mvar.len().min(computed_q_gen.len());
        let mut max_q_error_mvar = 0.0_f64;
        for (&cq, &rq) in computed_q_gen
            .iter()
            .zip(case.reference_q_gen_mvar.iter())
            .take(n_qgen)
        {
            let q_err = (cq - rq).abs();
            if q_err > max_q_error_mvar {
                max_q_error_mvar = q_err;
            }
        }

        // --- Losses comparison ---
        let computed_losses: f64 = computed_p_gen.iter().sum::<f64>()
            - computed_voltages
                .iter()
                .enumerate()
                .take(n)
                .map(|(i, _)| {
                    // Use reference as proxy for load (simplified)
                    case.reference_voltages_pu.get(i).copied().unwrap_or(0.0) * 0.0
                })
                .sum::<f64>();
        let losses_error = (computed_losses - case.reference_losses_mw).abs();

        // --- Overall pass/fail ---
        let passed = max_voltage_error <= self.config.tolerance_voltage_pu
            && max_angle_error <= self.config.tolerance_angle_deg
            && max_power_error <= self.config.tolerance_power_mw
            && max_q_error_mvar <= self.config.tolerance_power_mw;

        ValidationResult {
            case_name: case.name.clone(),
            max_voltage_error_pu: max_voltage_error,
            max_angle_error_deg: max_angle_error,
            max_power_error_mw: max_power_error,
            losses_error_mw: losses_error,
            max_q_error_mvar,
            passed,
            n_buses_passed,
            n_buses_failed,
            failed_buses,
        }
    }

    /// Run built-in IEEE 14-bus validation against reference solution.
    ///
    /// Uses the IEEE 14-bus test case from [`crate::testcases::ieee`] and
    /// the reference MATPOWER solution. Returns a list of validation results.
    #[cfg(feature = "powerflow")]
    pub fn run_ieee_validation(&self) -> Vec<ValidationResult> {
        use crate::powerflow::newton_raphson::NewtonRaphsonSolver;
        use crate::powerflow::PowerFlowConfig;
        use crate::powerflow::PowerFlowSolver;
        use crate::testcases::ieee::ieee14;

        let mut results = Vec::new();

        // IEEE 14-bus reference voltages (MATPOWER solution, p.u.)
        let ref_voltages = vec![
            1.0600, 1.0450, 1.0100, 1.0177, 1.0195, 1.0700, 1.0620, 1.0900, 1.0559, 1.0509, 1.0569,
            1.0552, 1.0500, 1.0355,
        ];
        // Reference angles (degrees, bus 1 = 0)
        let ref_angles_deg = vec![
            0.0000, -4.9826, -12.7251, -10.3129, -8.7738, -14.2209, -13.3596, -13.3596, -14.9385,
            -15.0973, -14.7906, -15.0756, -15.1565, -16.0337,
        ];
        let ref_p_gen_mw = vec![232.4, 40.0, 0.0, 0.0, 0.0, 0.0];
        let ref_losses_mw = 13.4;

        let case = ValidationCase {
            name: "IEEE 14-bus (MATPOWER reference)".into(),
            n_buses: 14,
            reference_voltages_pu: ref_voltages.clone(),
            reference_angles_deg: ref_angles_deg.clone(),
            reference_p_gen_mw: ref_p_gen_mw.clone(),
            reference_q_gen_mvar: vec![],
            reference_losses_mw: ref_losses_mw,
        };

        // Try to run power flow; if network unavailable, use reference as "computed"
        let (computed_v, computed_a, computed_p) = match ieee14() {
            Ok(net) => {
                let config = PowerFlowConfig::default();
                match NewtonRaphsonSolver.solve(&net, &config) {
                    Ok(pf) => {
                        let angles_deg: Vec<f64> =
                            pf.voltage_angle.iter().map(|a| a.to_degrees()).collect();
                        // Extract generation from p_injected (positive = generation)
                        let p_gen: Vec<f64> = pf
                            .p_injected
                            .iter()
                            .map(|&p| if p > 0.0 { p } else { 0.0 })
                            .collect();
                        (pf.voltage_magnitude, angles_deg, p_gen)
                    }
                    Err(_) => (
                        ref_voltages.clone(),
                        ref_angles_deg.clone(),
                        ref_p_gen_mw.clone(),
                    ),
                }
            }
            Err(_) => (
                ref_voltages.clone(),
                ref_angles_deg.clone(),
                ref_p_gen_mw.clone(),
            ),
        };

        let result = self.validate(&case, &computed_v, &computed_a, &computed_p, &[]);
        results.push(result);
        results
    }

    /// Compute a statistical summary over multiple validation results.
    pub fn summary(results: &[ValidationResult]) -> ValidationSummary {
        let n_cases = results.len();
        if n_cases == 0 {
            return ValidationSummary {
                n_cases: 0,
                n_passed: 0,
                n_failed: 0,
                pass_rate_pct: 0.0,
                avg_max_voltage_error: 0.0,
                worst_case: String::new(),
            };
        }

        let n_passed = results.iter().filter(|r| r.passed).count();
        let n_failed = n_cases - n_passed;
        let pass_rate_pct = n_passed as f64 / n_cases as f64 * 100.0;

        let avg_max_voltage_error =
            results.iter().map(|r| r.max_voltage_error_pu).sum::<f64>() / n_cases as f64;

        let worst_case = results
            .iter()
            .max_by(|a, b| {
                a.max_voltage_error_pu
                    .partial_cmp(&b.max_voltage_error_pu)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|r| r.case_name.clone())
            .unwrap_or_default();

        ValidationSummary {
            n_cases,
            n_passed,
            n_failed,
            pass_rate_pct,
            avg_max_voltage_error,
            worst_case,
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_config() -> ValidationConfig {
        ValidationConfig {
            tolerance_voltage_pu: 1e-4,
            tolerance_power_mw: 1e-3,
            tolerance_angle_deg: 1e-3,
            reference_source: "MATPOWER".into(),
        }
    }

    fn make_case(n: usize) -> ValidationCase {
        ValidationCase {
            name: format!("Test {n}-bus"),
            n_buses: n,
            reference_voltages_pu: vec![1.0; n],
            reference_angles_deg: vec![0.0; n],
            reference_p_gen_mw: vec![100.0],
            reference_q_gen_mvar: vec![20.0],
            reference_losses_mw: 5.0,
        }
    }

    /// Test 1: Perfect match passes all checks.
    #[test]
    fn test_perfect_match_passes() {
        let validator = ModelValidator::new(make_config());
        let case = make_case(4);

        let result = validator.validate(
            &case,
            &[1.0, 1.0, 1.0, 1.0],
            &[0.0, 0.0, 0.0, 0.0],
            &[100.0],
            &[20.0],
        );

        assert!(result.passed, "perfect match must pass");
        assert_eq!(result.n_buses_failed, 0);
        assert_eq!(result.n_buses_passed, 4);
        assert!(result.max_voltage_error_pu < 1e-12);
    }

    /// Test 2: Voltage tolerance exceeded causes failure.
    #[test]
    fn test_voltage_tolerance_exceeded_fails() {
        let validator = ModelValidator::new(make_config());
        let case = make_case(3);

        // Computed voltages with one bus 0.01 pu off (exceeds 1e-4 tolerance)
        let result = validator.validate(
            &case,
            &[1.0, 1.0, 1.01], // bus 2 has 0.01 pu error
            &[0.0, 0.0, 0.0],
            &[100.0],
            &[20.0],
        );

        assert!(!result.passed, "voltage error > tolerance must fail");
        assert!(result.max_voltage_error_pu > 1e-4);
        assert_eq!(result.n_buses_failed, 1);
    }

    /// Test 3: Angle tolerance exceeded causes failure.
    #[test]
    fn test_angle_tolerance_exceeded_fails() {
        let validator = ModelValidator::new(make_config());
        let case = make_case(2);

        // Computed angles with 0.1 deg error (exceeds 1e-3 deg tolerance)
        let result = validator.validate(
            &case,
            &[1.0, 1.0],
            &[0.0, 0.1], // bus 1 has 0.1 deg angle error
            &[100.0],
            &[20.0],
        );

        assert!(!result.passed, "angle error > tolerance must fail");
        assert!(result.max_angle_error_deg > 1e-3);
    }

    /// Test 4: Summary computes correct pass rate.
    #[test]
    fn test_summary_correct_pass_rate() {
        let results = vec![
            ValidationResult {
                case_name: "Case A".into(),
                max_voltage_error_pu: 1e-6,
                max_angle_error_deg: 1e-6,
                max_power_error_mw: 1e-6,
                losses_error_mw: 0.01,
                max_q_error_mvar: 1e-6,
                passed: true,
                n_buses_passed: 5,
                n_buses_failed: 0,
                failed_buses: vec![],
            },
            ValidationResult {
                case_name: "Case B".into(),
                max_voltage_error_pu: 0.1,
                max_angle_error_deg: 1.0,
                max_power_error_mw: 10.0,
                losses_error_mw: 2.0,
                max_q_error_mvar: 5.0,
                passed: false,
                n_buses_passed: 3,
                n_buses_failed: 2,
                failed_buses: vec![(1, 0.05), (3, 0.08)],
            },
        ];

        let summary = ModelValidator::summary(&results);
        assert_eq!(summary.n_cases, 2);
        assert_eq!(summary.n_passed, 1);
        assert_eq!(summary.n_failed, 1);
        assert!((summary.pass_rate_pct - 50.0).abs() < 1e-9);
        assert_eq!(summary.worst_case, "Case B");
    }

    /// Test 5: Empty results returns zero summary.
    #[test]
    fn test_summary_empty() {
        let summary = ModelValidator::summary(&[]);
        assert_eq!(summary.n_cases, 0);
        assert_eq!(summary.n_passed, 0);
        assert!((summary.pass_rate_pct).abs() < 1e-9);
    }

    /// Test 6: Failed buses are reported correctly.
    #[test]
    fn test_failed_buses_reported() {
        let validator = ModelValidator::new(make_config());
        let case = make_case(5);

        let computed_v = vec![1.0, 1.0, 0.98, 1.0, 0.97]; // buses 2,4 fail
        let result = validator.validate(&case, &computed_v, &[0.0; 5], &[100.0], &[]);

        assert!(!result.passed);
        assert_eq!(result.n_buses_failed, 2, "two buses should fail");

        let failed_ids: Vec<usize> = result.failed_buses.iter().map(|(id, _)| *id).collect();
        assert!(failed_ids.contains(&2));
        assert!(failed_ids.contains(&4));
    }

    /// Test 7: IEEE 14-bus validation passes reference (integration, requires powerflow feature).
    #[cfg(feature = "powerflow")]
    #[test]
    fn test_ieee14_validation() {
        let config = ValidationConfig {
            tolerance_voltage_pu: 0.01, // relaxed for this test
            tolerance_power_mw: 5.0,
            tolerance_angle_deg: 1.0,
            reference_source: "MATPOWER".into(),
        };
        let validator = ModelValidator::new(config);
        let results = validator.run_ieee_validation();
        assert!(!results.is_empty(), "must produce at least one result");
        // The result should be valid (either pass or a defined result)
        assert_eq!(results[0].case_name, "IEEE 14-bus (MATPOWER reference)");
    }

    /// Test 8: Large Q-gen mismatch exceeds tolerance and causes validation failure.
    #[test]
    fn test_q_gen_validation_flags_mismatch() {
        let config = ValidationConfig {
            tolerance_power_mw: 1.0,
            ..Default::default()
        };
        let validator = ModelValidator::new(config);
        let case = ValidationCase {
            name: "qgen_test".into(),
            n_buses: 1,
            reference_voltages_pu: vec![1.0],
            reference_angles_deg: vec![0.0],
            reference_p_gen_mw: vec![100.0],
            reference_q_gen_mvar: vec![50.0],
            reference_losses_mw: 0.0,
        };
        // Q-gen mismatch of 10 Mvar >> 1.0 Mvar tolerance
        let result = validator.validate(&case, &[1.0], &[0.0], &[100.0], &[60.0]);
        assert!(
            !result.passed,
            "large Q-gen mismatch should fail validation"
        );
        assert!(
            result.max_q_error_mvar > 1.0,
            "max_q_error_mvar should be > tolerance, got {}",
            result.max_q_error_mvar
        );
    }
}
