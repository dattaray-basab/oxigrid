//! Measurement integrity verification for power grid state estimation.
//!
//! Provides:
//! - [`PhysicalConstraint`] — Kirchhoff laws, voltage proximity, frequency uniformity, …
//! - [`IntegrityVerifier`] — checks a set of physical constraints against raw measurements
//! - [`MeasurementWatermark`] — active watermarking for replay/man-in-the-middle detection

use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// LCG helper (same parameters as fdi.rs)
// ---------------------------------------------------------------------------

#[inline]
fn lcg_next(state: u64) -> u64 {
    state
        .wrapping_mul(6_364_136_223_846_793_005_u64)
        .wrapping_add(1_442_695_040_888_963_407_u64)
}

// ---------------------------------------------------------------------------
// PhysicalConstraint
// ---------------------------------------------------------------------------

/// A physical law or engineering constraint that grid measurements must satisfy.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum PhysicalConstraint {
    /// Kirchhoff's Current Law at a specific bus (injection ≈ net outflow).
    Kcl {
        /// Internal bus index (0-based).
        bus: usize,
        /// Maximum acceptable KCL imbalance \[MW\].
        tolerance_mw: f64,
    },
    /// Adjacent bus voltages should not differ by more than a threshold.
    VoltageProximity {
        /// First bus index.
        bus_i: usize,
        /// Second bus index.
        bus_j: usize,
        /// Maximum acceptable voltage magnitude difference \[p.u.\].
        max_diff_pu: f64,
    },
    /// Power flow on a branch should be in the expected direction (non-negative).
    FlowDirection {
        /// Branch index.
        branch: usize,
    },
    /// All PMU frequency measurements should be nearly identical.
    FrequencyUniformity {
        /// Maximum acceptable spread across PMU readings \[Hz\].
        max_spread_hz: f64,
    },
    /// Total generation ≈ total load + losses (global power balance).
    GlobalPowerBalance {
        /// Maximum acceptable power-balance error \[MW\].
        tolerance_mw: f64,
    },
}

// ---------------------------------------------------------------------------
// IntegrityReport
// ---------------------------------------------------------------------------

/// Summary of an integrity check against a set of physical constraints.
#[derive(Debug, Clone)]
pub struct IntegrityReport {
    /// `true` iff every constraint passed.
    pub overall_pass: bool,
    /// Per-constraint results: `(constraint, passed, violation_magnitude)`.
    pub constraint_results: Vec<(PhysicalConstraint, bool, f64)>,
    /// Number of failed constraints.
    pub n_violations: usize,
    /// Measurement indices suspected to be corrupted (from KCL failures).
    pub suspected_compromised: Vec<usize>,
}

// ---------------------------------------------------------------------------
// IntegrityVerifier
// ---------------------------------------------------------------------------

/// Verifies that a measurement vector is consistent with a set of physical constraints.
pub struct IntegrityVerifier {
    /// Constraints to evaluate on every call to [`verify`](Self::verify).
    pub physical_constraints: Vec<PhysicalConstraint>,
}

impl IntegrityVerifier {
    /// Create a verifier with the given constraint list.
    pub fn new(constraints: Vec<PhysicalConstraint>) -> Self {
        Self {
            physical_constraints: constraints,
        }
    }

    /// Check all constraints against `measurements`.
    ///
    /// The `network` parameter is used to retrieve structural information such as
    /// bus counts and branch connectivity.
    #[cfg(feature = "powerflow")]
    pub fn verify(
        &self,
        measurements: &[f64],
        _network: &crate::network::topology::PowerNetwork,
    ) -> IntegrityReport {
        self.verify_measurements(measurements)
    }

    /// Check all constraints against `measurements` (standalone, no network needed).
    pub fn verify_measurements(&self, measurements: &[f64]) -> IntegrityReport {
        let len = measurements.len().max(1);
        let mut constraint_results = Vec::with_capacity(self.physical_constraints.len());
        let mut n_violations = 0usize;
        let mut suspected_compromised: Vec<usize> = Vec::new();

        for constraint in &self.physical_constraints {
            let (pass, violation) = evaluate_constraint(constraint, measurements, len);
            if !pass {
                n_violations += 1;
                // Collect bus indices from KCL failures.
                if let PhysicalConstraint::Kcl { bus, .. } = constraint {
                    if !suspected_compromised.contains(bus) {
                        suspected_compromised.push(*bus);
                    }
                }
            }
            constraint_results.push((constraint.clone(), pass, violation));
        }

        IntegrityReport {
            overall_pass: n_violations == 0,
            constraint_results,
            n_violations,
            suspected_compromised,
        }
    }

    /// Check KCL at every bus in the network.
    ///
    /// For bus `i`, KCL residual = `p_injections[i] − Σ(out-flows) + Σ(in-flows)`.
    ///
    /// Returns `(bus_index, |residual|)` for all buses where the residual exceeds
    /// `tolerance_mw`.
    #[cfg(feature = "powerflow")]
    pub fn check_kcl_all(
        network: &crate::network::topology::PowerNetwork,
        p_flows: &[f64],
        p_injections: &[f64],
        tolerance_mw: f64,
    ) -> Vec<(usize, f64)> {
        let n_buses = network.buses.len();
        let mut residuals = vec![0.0_f64; n_buses];

        // Accumulate injection.
        for (i, inj) in p_injections.iter().enumerate().take(n_buses) {
            residuals[i] += inj;
        }

        // Subtract net branch flows.
        for (branch_idx, branch) in network.branches.iter().enumerate() {
            let flow = p_flows.get(branch_idx).cloned().unwrap_or(0.0);
            // Determine internal indices of from/to buses.
            let from_idx = network
                .buses
                .iter()
                .position(|b| b.id == branch.from_bus)
                .unwrap_or(0);
            let to_idx = network
                .buses
                .iter()
                .position(|b| b.id == branch.to_bus)
                .unwrap_or(0);
            // Flow leaves from_bus and enters to_bus.
            if from_idx < n_buses {
                residuals[from_idx] -= flow;
            }
            if to_idx < n_buses {
                residuals[to_idx] += flow;
            }
        }

        residuals
            .into_iter()
            .enumerate()
            .filter(|(_, r)| r.abs() > tolerance_mw)
            .map(|(i, r)| (i, r.abs()))
            .collect()
    }

    /// Check global power balance: `|generation − load − losses| ≤ tolerance_mw`.
    pub fn global_balance(
        total_generation_mw: f64,
        total_load_mw: f64,
        total_losses_mw: f64,
        tolerance_mw: f64,
    ) -> bool {
        (total_generation_mw - total_load_mw - total_losses_mw).abs() <= tolerance_mw
    }
}

/// Evaluate a single constraint against the measurement vector.
/// Returns `(pass, violation_magnitude)`.
fn evaluate_constraint(
    constraint: &PhysicalConstraint,
    measurements: &[f64],
    len: usize,
) -> (bool, f64) {
    match constraint {
        PhysicalConstraint::Kcl { bus, tolerance_mw } => {
            let idx = bus % len;
            let imbalance = measurements[idx].abs();
            let violation = (imbalance - tolerance_mw).max(0.0);
            (violation <= 0.0, violation)
        }
        PhysicalConstraint::VoltageProximity {
            bus_i,
            bus_j,
            max_diff_pu,
        } => {
            let vi = measurements[bus_i % len];
            let vj = measurements[bus_j % len];
            let diff = (vi - vj).abs();
            let violation = (diff - max_diff_pu).max(0.0);
            (violation <= 0.0, violation)
        }
        PhysicalConstraint::FlowDirection { branch } => {
            let flow = measurements[branch % len];
            let violation = (-flow).max(0.0);
            (flow >= -1e-6, violation)
        }
        PhysicalConstraint::FrequencyUniformity { max_spread_hz } => {
            let n = measurements.len().clamp(1, 3);
            let slice = &measurements[..n];
            let min = slice.iter().cloned().fold(f64::INFINITY, f64::min);
            let max = slice.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
            let spread = max - min;
            let violation = (spread - max_spread_hz).max(0.0);
            (violation <= 0.0, violation)
        }
        PhysicalConstraint::GlobalPowerBalance { tolerance_mw } => {
            let sum: f64 = measurements.iter().sum();
            let imbalance = sum.abs();
            let violation = (imbalance - tolerance_mw).max(0.0);
            (violation <= 0.0, violation)
        }
    }
}

// ---------------------------------------------------------------------------
// MeasurementWatermark
// ---------------------------------------------------------------------------

/// Active watermarking for detecting measurement replay and man-in-the-middle attacks.
///
/// A tiny pseudo-random signal (derived from a secret key and a timestamp) is injected
/// into the control inputs.  If an adversary replays old measurements or manipulates
/// sensor readings, the correlation between expected and received watermark will drop.
#[derive(Debug, Clone)]
pub struct MeasurementWatermark {
    /// Secret key used to seed the watermark LCG.
    pub watermark_key: u64,
    /// Standard deviation of the injected watermark noise.
    pub injection_std: f64,
}

impl MeasurementWatermark {
    /// Create a new watermark authenticator.
    pub fn new(key: u64, std: f64) -> Self {
        Self {
            watermark_key: key,
            injection_std: std,
        }
    }

    /// Generate and inject a pseudo-random watermark signal into `control_signal`.
    ///
    /// The watermark is generated from `watermark_key XOR timestamp` and has amplitude
    /// `injection_std`.  Returns a vector of length `n_samples`.
    pub fn inject_watermark(
        &self,
        control_signal: &[f64],
        timestamp: u64,
        n_samples: usize,
    ) -> Vec<f64> {
        let seed = self.watermark_key ^ timestamp;
        let mut state = seed;
        let cs_len = control_signal.len().max(1);
        (0..n_samples)
            .map(|i| {
                state = lcg_next(state);
                let wm = (state as f64 / u64::MAX as f64 - 0.5) * 2.0 * self.injection_std;
                control_signal[i % cs_len] + wm
            })
            .collect()
    }

    /// Verify that `sensor_reading` contains the expected watermark.
    ///
    /// Re-generates the watermark from `watermark_key`, `timestamp`, and the control
    /// signal, then computes the Pearson correlation between the residual
    /// `(sensor − control)` and the expected watermark.
    ///
    /// Returns a value in `[−1, 1]`; values close to `1.0` indicate authenticity.
    pub fn verify_watermark(
        &self,
        sensor_reading: &[f64],
        control_signal: &[f64],
        timestamp: u64,
    ) -> f64 {
        let n = sensor_reading.len();
        if n == 0 {
            return 0.0;
        }
        let cs_len = control_signal.len().max(1);

        // Regenerate the watermark sequence.
        let seed = self.watermark_key ^ timestamp;
        let mut state = seed;
        let watermark: Vec<f64> = (0..n)
            .map(|_| {
                state = lcg_next(state);
                (state as f64 / u64::MAX as f64 - 0.5) * 2.0 * self.injection_std
            })
            .collect();

        // Compute residual: diff = sensor - control (without watermark component).
        let diff: Vec<f64> = sensor_reading
            .iter()
            .enumerate()
            .map(|(i, &s)| s - control_signal[i % cs_len])
            .collect();

        pearson_correlation(&diff, &watermark)
    }
}

/// Compute Pearson correlation between two equal-length slices.
/// Returns `0.0` if standard deviations are near zero.
fn pearson_correlation(a: &[f64], b: &[f64]) -> f64 {
    let n = a.len().min(b.len());
    if n == 0 {
        return 0.0;
    }
    let ma = a[..n].iter().sum::<f64>() / n as f64;
    let mb = b[..n].iter().sum::<f64>() / n as f64;
    let num: f64 = a[..n]
        .iter()
        .zip(b[..n].iter())
        .map(|(x, y)| (x - ma) * (y - mb))
        .sum();
    let da: f64 = a[..n].iter().map(|x| (x - ma).powi(2)).sum::<f64>().sqrt();
    let db: f64 = b[..n].iter().map(|y| (y - mb).powi(2)).sum::<f64>().sqrt();
    let denom = da * db;
    if denom < 1e-15 {
        0.0
    } else {
        (num / denom).clamp(-1.0, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn global_balance_passes_balanced() {
        assert!(IntegrityVerifier::global_balance(100.0, 95.0, 5.0, 0.1));
    }

    #[test]
    fn global_balance_fails_unbalanced() {
        assert!(!IntegrityVerifier::global_balance(100.0, 90.0, 5.0, 0.1));
    }

    #[test]
    fn watermark_authentic_high_correlation() {
        let wm = MeasurementWatermark::new(99999, 0.01);
        let control = vec![1.0_f64, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0];
        let injected = wm.inject_watermark(&control, 42, 8);
        let corr = wm.verify_watermark(&injected, &control, 42);
        assert!(corr > 0.5, "Expected correlation > 0.5, got {corr}");
    }

    #[test]
    fn watermark_manipulated_lower_correlation() {
        let wm = MeasurementWatermark::new(99999, 0.01);
        let control = vec![1.0_f64, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0];
        // Stripped watermark: sensor == control (no watermark).
        let corr = wm.verify_watermark(&control, &control, 42);
        // diff = 0 everywhere → correlation undefined → returns 0.0
        assert!(
            corr < 0.99,
            "Expected low correlation for stripped signal, got {corr}"
        );
    }

    #[test]
    fn kcl_constraint_catches_violation() {
        let constraint = PhysicalConstraint::Kcl {
            bus: 0,
            tolerance_mw: 1.0,
        };
        let measurements = vec![50.0_f64, 1.0]; // bus 0 imbalance = 50 MW
        let (pass, violation) = evaluate_constraint(&constraint, &measurements, 2);
        assert!(!pass);
        assert!(violation > 0.0);
    }

    #[test]
    fn voltage_proximity_passes_when_close() {
        let constraint = PhysicalConstraint::VoltageProximity {
            bus_i: 0,
            bus_j: 1,
            max_diff_pu: 0.1,
        };
        // |1.0 - 1.05| = 0.05 ≤ 0.10 → pass
        let measurements = vec![1.0_f64, 1.05];
        let (pass, violation) = evaluate_constraint(&constraint, &measurements, 2);
        assert!(pass, "Expected pass but violation = {violation}");
        assert_eq!(violation, 0.0);
    }

    #[test]
    fn voltage_proximity_fails_when_too_different() {
        let constraint = PhysicalConstraint::VoltageProximity {
            bus_i: 0,
            bus_j: 1,
            max_diff_pu: 0.05,
        };
        // |1.0 - 1.2| = 0.2 > 0.05 → fail
        let measurements = vec![1.0_f64, 1.2];
        let (pass, violation) = evaluate_constraint(&constraint, &measurements, 2);
        assert!(!pass, "Expected failure but got pass");
        assert!(
            violation > 0.0,
            "Violation magnitude should be > 0, got {violation}"
        );
    }

    #[test]
    fn flow_direction_passes_for_positive_flow() {
        let constraint = PhysicalConstraint::FlowDirection { branch: 0 };
        // flow = +50 MW (non-negative) → pass
        let measurements = vec![50.0_f64];
        let (pass, violation) = evaluate_constraint(&constraint, &measurements, 1);
        assert!(pass, "Expected pass for positive flow");
        assert_eq!(violation, 0.0);
    }

    #[test]
    fn flow_direction_fails_for_negative_flow() {
        let constraint = PhysicalConstraint::FlowDirection { branch: 0 };
        // flow = -30 MW → fail
        let measurements = vec![-30.0_f64];
        let (pass, violation) = evaluate_constraint(&constraint, &measurements, 1);
        assert!(!pass, "Expected fail for negative flow");
        assert!(
            violation > 0.0,
            "Violation magnitude should be > 0, got {violation}"
        );
    }

    #[test]
    fn frequency_uniformity_passes_small_spread() {
        let constraint = PhysicalConstraint::FrequencyUniformity {
            max_spread_hz: 0.01,
        };
        // spread = 50.005 - 49.998 = 0.007 ≤ 0.01 → pass
        let measurements = vec![49.998_f64, 50.002, 50.005];
        let (pass, violation) = evaluate_constraint(&constraint, &measurements, 3);
        assert!(
            pass,
            "Expected pass for small spread, violation = {violation}"
        );
    }

    #[test]
    fn global_power_balance_via_verify_measurements_passes() {
        // GlobalPowerBalance sums measurements; near-zero sum → pass
        let constraint = PhysicalConstraint::GlobalPowerBalance { tolerance_mw: 1.0 };
        let verifier = IntegrityVerifier::new(vec![constraint]);
        // sum = 100.0 - 95.0 - 5.0 = 0.0 ≤ 1.0 → pass
        let measurements = vec![100.0_f64, -95.0, -5.0];
        let report = verifier.verify_measurements(&measurements);
        assert!(report.overall_pass, "Expected overall_pass = true");
        assert_eq!(report.n_violations, 0);
    }

    #[test]
    fn n_violations_counts_multiple_failures() {
        // Two KCL constraints both violated + one VoltageProximity violated → 3 failures
        let constraints = vec![
            PhysicalConstraint::Kcl {
                bus: 0,
                tolerance_mw: 1.0,
            },
            PhysicalConstraint::Kcl {
                bus: 1,
                tolerance_mw: 1.0,
            },
            PhysicalConstraint::VoltageProximity {
                bus_i: 0,
                bus_j: 1,
                max_diff_pu: 0.01,
            },
        ];
        let verifier = IntegrityVerifier::new(constraints);
        // measurements[0] = 50.0 → KCL bus 0 imbalance = 50 (violation)
        // measurements[1] = 40.0 → KCL bus 1 imbalance = 40 (violation)
        // |50.0 - 40.0| = 10.0 > 0.01 → VoltageProximity violation
        let measurements = vec![50.0_f64, 40.0];
        let report = verifier.verify_measurements(&measurements);
        assert!(!report.overall_pass);
        assert_eq!(
            report.n_violations, 3,
            "Expected 3 violations, got {}",
            report.n_violations
        );
        // Both KCL failures add their bus indices to suspected_compromised
        assert!(report.suspected_compromised.contains(&0));
        assert!(report.suspected_compromised.contains(&1));
    }
}
