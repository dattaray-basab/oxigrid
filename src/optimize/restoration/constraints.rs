//! Restoration constraints: voltage, frequency, thermal limits, and switching rules.
//!
//! These constraints govern which restoration actions are permissible at any
//! given instant and ensure the re-energised system remains within safe bounds.

use crate::error::{OxiGridError, Result};
use crate::network::topology::PowerNetwork;
use crate::optimize::restoration::black_start::{
    BlackStartConfig, EnergizationPath, RestorationStep,
};

// ─────────────────────────────────────────────────────────────────────────────
// Voltage constraints
// ─────────────────────────────────────────────────────────────────────────────

/// Check that the voltage profile after energising `path` stays within the
/// acceptable band `[1 - tol, 1 + tol]` p.u.
///
/// Uses a very rough estimate: each km of line at rated current drops ~0.3 kV/km
/// at 110 kV.  A full AC power flow would replace this in production.
pub fn check_voltage_constraint(
    path: &EnergizationPath,
    nominal_voltage_kv: f64,
    tolerance_pu: f64,
) -> bool {
    // Estimated voltage drop per unit of line length (normalised)
    let drop_per_km = 0.003 / nominal_voltage_kv.max(1.0); // approximate
    let estimated_drop_pu = drop_per_km * path.total_length_km;
    estimated_drop_pu <= tolerance_pu
}

// ─────────────────────────────────────────────────────────────────────────────
// Frequency constraints
// ─────────────────────────────────────────────────────────────────────────────

/// Evaluate whether each step's recorded frequency stays within `tolerance_hz`
/// of 50 Hz.  Returns a list of step IDs that violate the constraint.
pub fn find_frequency_violations(steps: &[RestorationStep], tolerance_hz: f64) -> Vec<usize> {
    let f0 = 50.0_f64;
    steps
        .iter()
        .filter(|s| (s.frequency_hz - f0).abs() > tolerance_hz)
        .map(|s| s.step_id)
        .collect()
}

// ─────────────────────────────────────────────────────────────────────────────
// Thermal / loading constraints
// ─────────────────────────────────────────────────────────────────────────────

/// Check that no branch on `path` would be overloaded by the cold-load pickup
/// current implied by `pickup_mw` at `nominal_voltage_kv`.
///
/// Returns `Err` with the first overloaded branch index found, or `Ok(())`.
pub fn check_thermal_constraint(
    network: &PowerNetwork,
    path: &EnergizationPath,
    pickup_mw: f64,
    nominal_voltage_kv: f64,
) -> Result<()> {
    let current_ka = pickup_mw / (1.732 * nominal_voltage_kv.max(1.0));

    for &bi in &path.branch_sequence {
        let branch = network.branches.get(bi).ok_or_else(|| {
            OxiGridError::InvalidParameter(format!("Branch index {} out of range", bi))
        })?;

        if branch.rate_a > 0.0 {
            // rate_a is in MVA; convert to kA at given voltage
            let rating_ka = branch.rate_a / (1.732 * nominal_voltage_kv.max(1.0));
            if current_ka > rating_ka {
                return Err(OxiGridError::InvalidParameter(format!(
                    "Branch {} ({}→{}) overloaded: {:.3} kA > {:.3} kA rating",
                    bi, branch.from_bus, branch.to_bus, current_ka, rating_ka
                )));
            }
        }
    }
    Ok(())
}

// ─────────────────────────────────────────────────────────────────────────────
// Reserve margin constraints
// ─────────────────────────────────────────────────────────────────────────────

/// Verify that every restoration step maintains the required spinning reserve.
///
/// Returns indices of steps where the reserve margin is violated.
pub fn find_reserve_violations(steps: &[RestorationStep], reserve_margin_pct: f64) -> Vec<usize> {
    steps
        .iter()
        .filter(|s| {
            let required_reserve = s.available_generation_mw * reserve_margin_pct / 100.0;
            let actual_reserve = s.available_generation_mw - s.connected_load_mw;
            actual_reserve < required_reserve
        })
        .map(|s| s.step_id)
        .collect()
}

// ─────────────────────────────────────────────────────────────────────────────
// Cranking-distance constraint
// ─────────────────────────────────────────────────────────────────────────────

/// Return `true` if `path.total_length_km` is within the cranking range of
/// any configured black-start unit.
pub fn path_within_crank_range(path: &EnergizationPath, config: &BlackStartConfig) -> bool {
    config
        .black_start_units
        .iter()
        .any(|bs| path.total_length_km <= bs.max_crank_distance_km)
}

// ─────────────────────────────────────────────────────────────────────────────
// Switching sequence constraint (no back-feeding)
// ─────────────────────────────────────────────────────────────────────────────

/// Validate that no step attempts to energise a bus that is already in the
/// energised set (which would represent a back-feed and potentially a fault).
///
/// Returns a list of step IDs with back-feed violations.
pub fn find_backfeed_violations(steps: &[RestorationStep]) -> Vec<usize> {
    use crate::optimize::restoration::black_start::RestorationAction;
    use std::collections::HashSet;

    let mut energized: HashSet<usize> = HashSet::new();
    let mut violations = Vec::new();

    for step in steps {
        if let RestorationAction::EnergizePath { path } = &step.action {
            if energized.contains(&path.to_bus) {
                violations.push(step.step_id);
            } else {
                energized.insert(path.to_bus);
            }
            // from_bus should already be energized; mark it if not yet seen
            energized.insert(path.from_bus);
        }
    }
    violations
}

// ─────────────────────────────────────────────────────────────────────────────
// Aggregate constraint checker
// ─────────────────────────────────────────────────────────────────────────────

/// Result of a full constraint audit on a restoration plan.
#[derive(Debug)]
pub struct ConstraintReport {
    /// Step IDs with frequency violations.
    pub frequency_violations: Vec<usize>,
    /// Step IDs with reserve margin violations.
    pub reserve_violations: Vec<usize>,
    /// Step IDs with back-feed (re-energisation) violations.
    pub backfeed_violations: Vec<usize>,
    /// `true` if no violations were found.
    pub all_satisfied: bool,
}

/// Run all constraint checks on the given step list and config.
pub fn audit_plan(steps: &[RestorationStep], config: &BlackStartConfig) -> ConstraintReport {
    let freq_v = find_frequency_violations(steps, config.frequency_tolerance_hz);
    let rsv_v = find_reserve_violations(steps, config.reserve_margin_pct);
    let bf_v = find_backfeed_violations(steps);
    let all_ok = freq_v.is_empty() && rsv_v.is_empty() && bf_v.is_empty();
    ConstraintReport {
        frequency_violations: freq_v,
        reserve_violations: rsv_v,
        backfeed_violations: bf_v,
        all_satisfied: all_ok,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::branch::Branch;
    use crate::network::topology::PowerNetwork;
    use crate::optimize::restoration::black_start::{
        BlackStartConfig, BlackStartUnit, EnergizationPath, RestorationAction, RestorationStep,
    };

    // ── helpers ────────────────────────────────────────────────────────────────

    fn make_path(
        from_bus: usize,
        to_bus: usize,
        branch_seq: Vec<usize>,
        km: f64,
    ) -> EnergizationPath {
        EnergizationPath {
            from_bus,
            to_bus,
            branch_sequence: branch_seq,
            total_length_km: km,
            charging_current_mvar: 0.0,
            can_energize_at_t: 0.0,
        }
    }

    fn make_step(
        id: usize,
        freq_hz: f64,
        gen_mw: f64,
        load_mw: f64,
        action: RestorationAction,
    ) -> RestorationStep {
        RestorationStep {
            step_id: id,
            time_min: id as f64,
            action,
            available_generation_mw: gen_mw,
            connected_load_mw: load_mw,
            frequency_hz: freq_hz,
            notes: String::new(),
        }
    }

    fn make_branch(from: usize, to: usize, rate_a: f64) -> Branch {
        Branch {
            from_bus: from,
            to_bus: to,
            r: 0.01,
            x: 0.1,
            b: 0.0,
            rate_a,
            rate_b: 0.0,
            rate_c: 0.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        }
    }

    fn make_bs_unit(max_crank_km: f64) -> BlackStartUnit {
        BlackStartUnit {
            gen_id: 0,
            bus: 1,
            p_rated_mw: 100.0,
            p_min_mw: 10.0,
            ramp_rate_mw_per_min: 5.0,
            crank_time_min: 15.0,
            max_crank_distance_km: max_crank_km,
            auxiliary_load_mw: 2.0,
            priority: 1,
        }
    }

    // ── test 1: voltage constraint passes when estimated drop is below tolerance ──

    #[test]
    fn test_check_voltage_constraint_within_tolerance() {
        // Reason: short paths at high voltage should not exceed the tolerance band.
        let path = make_path(1, 2, vec![0], 5.0); // 5 km
                                                  // drop = (0.003 / 110.0) * 5 = ~0.000136 p.u. well under 0.05
        assert!(check_voltage_constraint(&path, 110.0, 0.05));
    }

    // ── test 2: voltage constraint fails when estimated drop exceeds tolerance ──

    #[test]
    fn test_check_voltage_constraint_exceeds_tolerance() {
        // Reason: a very long path at low voltage must be rejected.
        let path = make_path(1, 2, vec![0], 10_000.0); // 10 000 km
                                                       // drop = (0.003 / 1.0) * 10000 = 30 p.u. >> 0.05
        assert!(!check_voltage_constraint(&path, 0.0, 0.05));
    }

    // ── test 3: frequency violations correctly identifies out-of-band steps ──

    #[test]
    fn test_find_frequency_violations_identifies_bad_steps() {
        // Reason: steps outside ±0.5 Hz of 50 Hz must appear in the violation list.
        let action = RestorationAction::RampGenerator {
            gen_id: 0,
            target_mw: 50.0,
        };
        let steps = vec![
            make_step(1, 50.0, 100.0, 60.0, action.clone()), // ok
            make_step(2, 49.0, 100.0, 60.0, action.clone()), // violates (|49-50|=1 > 0.5)
            make_step(3, 50.3, 100.0, 60.0, action.clone()), // ok
        ];
        let violations = find_frequency_violations(&steps, 0.5);
        assert_eq!(violations, vec![2]);
    }

    // ── test 4: frequency violations returns empty list when all steps are in band ──

    #[test]
    fn test_find_frequency_violations_empty_when_all_ok() {
        // Reason: if every step is within tolerance, no violations should be reported.
        let action = RestorationAction::RampGenerator {
            gen_id: 0,
            target_mw: 50.0,
        };
        let steps = vec![
            make_step(1, 50.0, 100.0, 60.0, action.clone()),
            make_step(2, 50.4, 100.0, 60.0, action.clone()),
        ];
        let violations = find_frequency_violations(&steps, 0.5);
        assert!(violations.is_empty());
    }

    // ── test 5: thermal constraint returns Ok when branch is within rating ──

    #[test]
    fn test_check_thermal_constraint_within_rating_ok() {
        // Reason: pickup current below branch rating must not return an error.
        let mut net = PowerNetwork::new(100.0);
        net.branches.push(make_branch(1, 2, 1000.0)); // 1000 MVA rating
        let path = make_path(1, 2, vec![0], 10.0);
        // current_ka = 1.0 / (1.732 * 110) ≈ 0.00525 kA; rating_ka = 1000/(1.732*110) ≈ 5.25 kA
        let result = check_thermal_constraint(&net, &path, 1.0, 110.0);
        assert!(result.is_ok());
    }

    // ── test 6: thermal constraint returns Err when branch is overloaded ──

    #[test]
    fn test_check_thermal_constraint_overload_err() {
        // Reason: pickup current that exceeds the branch MVA rating must return an error.
        let mut net = PowerNetwork::new(100.0);
        net.branches.push(make_branch(1, 2, 0.001)); // 0.001 MVA — tiny rating
        let path = make_path(1, 2, vec![0], 10.0);
        // pickup_mw = 500 → large current; rating is negligible → overloaded
        let result = check_thermal_constraint(&net, &path, 500.0, 110.0);
        assert!(result.is_err());
    }

    // ── test 7: thermal constraint returns Err when branch index is out of range ──

    #[test]
    fn test_check_thermal_constraint_out_of_range_err() {
        // Reason: a branch_sequence that references a non-existent branch index must surface an error.
        let net = PowerNetwork::new(100.0); // no branches
        let path = make_path(1, 2, vec![99], 10.0); // index 99 does not exist
        let result = check_thermal_constraint(&net, &path, 10.0, 110.0);
        assert!(result.is_err());
    }

    // ── test 8: thermal constraint skips branches with rate_a == 0 (unlimited) ──

    #[test]
    fn test_check_thermal_constraint_unlimited_rating_skipped() {
        // Reason: rate_a == 0 means unlimited; even very high pickup must succeed.
        let mut net = PowerNetwork::new(100.0);
        net.branches.push(make_branch(1, 2, 0.0)); // rate_a = 0 → unlimited
        let path = make_path(1, 2, vec![0], 10.0);
        let result = check_thermal_constraint(&net, &path, 1_000_000.0, 110.0);
        assert!(result.is_ok());
    }

    // ── test 9: reserve violations correctly identifies steps with insufficient reserve ──

    #[test]
    fn test_find_reserve_violations_identifies_violations() {
        // Reason: when load consumes all generation, spinning reserve is negative and must be flagged.
        let action = RestorationAction::RampGenerator {
            gen_id: 0,
            target_mw: 50.0,
        };
        // Step 1: gen=100, load=90 → reserve=10, required=20 → violation
        // Step 2: gen=100, load=60 → reserve=40, required=20 → ok
        let steps = vec![
            make_step(1, 50.0, 100.0, 90.0, action.clone()),
            make_step(2, 50.0, 100.0, 60.0, action.clone()),
        ];
        let violations = find_reserve_violations(&steps, 20.0);
        assert_eq!(violations, vec![1]);
    }

    // ── test 10: path_within_crank_range returns false when no units are configured ──

    #[test]
    fn test_path_within_crank_range_empty_units() {
        // Reason: with no black-start units, any path must fail the crank-range check.
        let path = make_path(1, 2, vec![], 50.0);
        let config = BlackStartConfig::default(); // black_start_units: vec![]
        assert!(!path_within_crank_range(&path, &config));
    }

    // ── test 11: path_within_crank_range returns true when path fits unit range ──

    #[test]
    fn test_path_within_crank_range_within_range() {
        // Reason: a path shorter than the unit's max crank distance must be accepted.
        let path = make_path(1, 2, vec![], 40.0);
        let mut config = BlackStartConfig::default();
        config.black_start_units.push(make_bs_unit(100.0));
        assert!(path_within_crank_range(&path, &config));
    }

    // ── test 12: find_backfeed_violations detects duplicate energisation of a bus ──

    #[test]
    fn test_find_backfeed_violations_detects_duplicate() {
        // Reason: attempting to energise a bus that was already energised in a previous step is a backfeed violation.
        let path_a = make_path(1, 2, vec![], 10.0);
        let path_b = make_path(1, 2, vec![], 10.0); // to_bus=2 already energised
        let steps = vec![
            make_step(
                1,
                50.0,
                100.0,
                0.0,
                RestorationAction::EnergizePath { path: path_a },
            ),
            make_step(
                2,
                50.0,
                100.0,
                0.0,
                RestorationAction::EnergizePath { path: path_b },
            ),
        ];
        let violations = find_backfeed_violations(&steps);
        assert_eq!(violations, vec![2]);
    }

    // ── test 13: find_backfeed_violations returns empty when sequence is clean ──

    #[test]
    fn test_find_backfeed_violations_clean_sequence() {
        // Reason: a correctly ordered sequence that energises each bus exactly once must produce no violations.
        let steps = vec![
            make_step(
                1,
                50.0,
                100.0,
                0.0,
                RestorationAction::EnergizePath {
                    path: make_path(1, 2, vec![], 10.0),
                },
            ),
            make_step(
                2,
                50.0,
                100.0,
                0.0,
                RestorationAction::EnergizePath {
                    path: make_path(2, 3, vec![], 10.0),
                },
            ),
        ];
        let violations = find_backfeed_violations(&steps);
        assert!(violations.is_empty());
    }

    // ── test 14: audit_plan reports all_satisfied = true for a clean plan ──

    #[test]
    fn test_audit_plan_all_satisfied_clean() {
        // Reason: a plan where frequency, reserve, and backfeed are all compliant must set all_satisfied = true.
        let config = BlackStartConfig {
            frequency_tolerance_hz: 0.5,
            reserve_margin_pct: 20.0,
            ..BlackStartConfig::default()
        };
        let steps = vec![
            make_step(
                1,
                50.0,
                100.0,
                60.0,
                RestorationAction::EnergizePath {
                    path: make_path(1, 2, vec![], 10.0),
                },
            ),
            make_step(
                2,
                50.2,
                100.0,
                65.0,
                RestorationAction::EnergizePath {
                    path: make_path(2, 3, vec![], 10.0),
                },
            ),
        ];
        let report = audit_plan(&steps, &config);
        assert!(report.all_satisfied);
        assert!(report.frequency_violations.is_empty());
        assert!(report.reserve_violations.is_empty());
        assert!(report.backfeed_violations.is_empty());
    }

    // ── test 15: audit_plan all_satisfied = false when multiple constraints violated ──

    #[test]
    fn test_audit_plan_multiple_violations() {
        // Reason: simultaneous frequency, reserve, and backfeed violations must all be captured and all_satisfied set to false.
        let config = BlackStartConfig {
            frequency_tolerance_hz: 0.5,
            reserve_margin_pct: 20.0,
            ..BlackStartConfig::default()
        };

        let dup_path = make_path(1, 2, vec![], 5.0);
        let steps = vec![
            // step 1: frequency violation (48 Hz) + reserve violation (gen=100, load=95)
            make_step(
                1,
                48.0,
                100.0,
                95.0,
                RestorationAction::EnergizePath {
                    path: make_path(1, 2, vec![], 5.0),
                },
            ),
            // step 2: backfeed (to_bus=2 already energised) + reserve violation
            make_step(
                2,
                50.0,
                100.0,
                90.0,
                RestorationAction::EnergizePath { path: dup_path },
            ),
        ];
        let report = audit_plan(&steps, &config);
        assert!(!report.all_satisfied);
        assert!(!report.frequency_violations.is_empty());
        assert!(!report.reserve_violations.is_empty());
        assert!(!report.backfeed_violations.is_empty());
    }
}
