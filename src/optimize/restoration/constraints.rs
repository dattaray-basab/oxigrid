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
