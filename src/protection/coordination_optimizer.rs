//! Advanced protection coordination optimizer.
//!
//! Automatically sets relay coordination curves, time-dial settings, and pickup
//! currents to achieve optimal selectivity between primary and backup overcurrent
//! relays.
//!
//! # Reference
//! - IEC 60255-151: Functional requirements for protection relays
//! - IEEE Std C37.112-1996: Inverse-time characteristic equations
//! - "Power System Protection" — Anderson, IEEE Press

use serde::{Deserialize, Serialize};

// ─────────────────────────────────────────────────────────────────────────────
// Enums
// ─────────────────────────────────────────────────────────────────────────────

/// IEC and ANSI overcurrent relay characteristic curves.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RelayCharacteristic {
    /// IEC Standard Inverse: t = TDS · 0.14 / ((I/Is)^0.02 − 1)
    StandardInverseIec,
    /// IEC Very Inverse: t = TDS · 13.5 / ((I/Is)^1.0 − 1)
    VeryInverseIec,
    /// IEC Extremely Inverse: t = TDS · 80.0 / ((I/Is)^2.0 − 1)
    ExtremelyInverseIec,
    /// Definite Time: t = TDS (constant regardless of current magnitude)
    DefiniteTime,
    /// ANSI Standard Inverse: t = TDS · (0.0515/((I/Is)^0.02 − 1) + 0.1140)
    StandardInverseAnsi,
    /// ANSI Very Inverse: t = TDS · (19.61/((I/Is)^2.0 − 1) + 0.4910)
    VeryInverseAnsi,
    /// ANSI Extremely Inverse: t = TDS · (28.2/((I/Is)^2.0 − 1) + 0.1217)
    ExtremelyInverseAnsi,
    /// ANSI Long Time Inverse: t = TDS · (120.0/((I/Is)^1.0 − 1))
    LongTimeInverseAnsi,
}

/// Objective function for the coordination optimizer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CoordinationObjective {
    /// Minimize the sum of all primary relay operating times.
    MinimizeTotalOperatingTime,
    /// Maximize the number of coordinated relay pairs.
    MaximizeSelectivity,
    /// Minimize the backup relay operating times (fast backup).
    MinimizeBackupTime,
    /// Balance between fast primary operation and adequate backup margins.
    BalancedCoordination,
}

// ─────────────────────────────────────────────────────────────────────────────
// Structs
// ─────────────────────────────────────────────────────────────────────────────

/// An overcurrent relay with all settings required for coordination studies.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OvercurrentRelay {
    /// Unique relay identifier.
    pub id: usize,
    /// Human-readable relay name.
    pub name: String,
    /// Bus where the relay is physically located.
    pub location_bus: usize,
    /// Protected branch (from_bus, to_bus).
    pub protected_branch: (usize, usize),
    /// Relay curve characteristic.
    pub characteristic: RelayCharacteristic,
    /// Pickup current setting Is `A`.
    pub pickup_current_a: f64,
    /// Time-dial setting TDS (0.1–10.0).
    pub time_dial: f64,
    /// Instantaneous overcurrent pickup `A`, or `None` if disabled.
    pub instantaneous_pickup_a: Option<f64>,
    /// Current transformer ratio (e.g., 200/5 → 40.0).
    pub ct_ratio: f64,
    /// Maximum through-fault current seen at this relay location `A`.
    pub max_fault_current_a: f64,
    /// Minimum fault current for sensitivity check `A`.
    pub min_fault_current_a: f64,
}

impl OvercurrentRelay {
    /// Construct a new relay with sensible defaults (no instantaneous element).
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        id: usize,
        name: impl Into<String>,
        location_bus: usize,
        protected_branch: (usize, usize),
        characteristic: RelayCharacteristic,
        pickup_current_a: f64,
        time_dial: f64,
        ct_ratio: f64,
        max_fault_current_a: f64,
        min_fault_current_a: f64,
    ) -> Self {
        Self {
            id,
            name: name.into(),
            location_bus,
            protected_branch,
            characteristic,
            pickup_current_a,
            time_dial,
            instantaneous_pickup_a: None,
            ct_ratio,
            max_fault_current_a,
            min_fault_current_a,
        }
    }
}

/// Result for one primary–backup relay coordination pair.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CoordinationPair {
    /// ID of the primary (fastest, nearest) relay.
    pub primary_relay_id: usize,
    /// ID of the backup relay.
    pub backup_relay_id: usize,
    /// Fault current at the primary relay location used for the check `A`.
    pub fault_current_a: f64,
    /// Primary relay operating time at `fault_current_a` `s`.
    pub primary_time_s: f64,
    /// Backup relay operating time at `fault_current_a` `s`.
    pub backup_time_s: f64,
    /// Coordination time interval CTI = backup_time − primary_time `s`.
    pub coordination_time_interval_s: f64,
    /// `true` when CTI ≥ 0.3 s (or the configured minimum).
    pub is_coordinated: bool,
}

/// A fault injection point used to drive the coordination study.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FaultPoint {
    /// Bus at which the fault is applied.
    pub location_bus: usize,
    /// Fault current magnitude `A`.
    pub fault_current_a: f64,
    /// Distance from the relay to the fault `km`.
    pub distance_from_relay_km: f64,
}

/// Complete solution returned by the optimizer.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CoordinationSolution {
    /// Relay objects with optimized TDS (and other) settings.
    pub relay_settings: Vec<OvercurrentRelay>,
    /// Evaluated coordination pairs for the optimal settings.
    pub coordination_pairs: Vec<CoordinationPair>,
    /// Total number of primary–backup pairs examined.
    pub n_pairs_total: usize,
    /// Number of pairs that satisfy CTI ≥ cti_min.
    pub n_pairs_coordinated: usize,
    /// Fraction of pairs that are coordinated (0.0 – 1.0).
    pub selectivity_index: f64,
    /// Sum of primary relay operating times at their respective max fault currents `s`.
    pub total_operating_time_s: f64,
    /// Number of optimizer iterations performed.
    pub iterations: usize,
    /// `true` if the iterative solver converged within `max_iterations`.
    pub converged: bool,
}

// ─────────────────────────────────────────────────────────────────────────────
// Main optimizer
// ─────────────────────────────────────────────────────────────────────────────

/// Coefficient set (A, alpha) and optional additive term B for the time formula.
struct CurveCoeffs {
    /// Multiplier constant A.
    a: f64,
    /// Exponent alpha.
    alpha: f64,
    /// Additive constant B (used in ANSI formulas).
    b: f64,
    /// True for definite-time relays.
    is_definite: bool,
}

/// Protection coordination optimizer for overcurrent relay grading.
///
/// Uses iterative constraint-satisfaction (LP-inspired) to find the minimum
/// time-dial settings that satisfy CTI ≥ `cti_min_s` for every coordination
/// pair, while keeping TDS within [`tds_min`, `tds_max`].
pub struct ProtectionCoordinationOptimizer {
    /// All relays in the protection scheme.
    pub relays: Vec<OvercurrentRelay>,
    /// Coordination pairs as (primary_relay_id, backup_relay_id).
    pub coordination_pairs: Vec<(usize, usize)>,
    /// Fault injection points for analysis.
    pub fault_points: Vec<FaultPoint>,
    /// Optimization objective.
    pub objective: CoordinationObjective,
    /// Minimum required coordination time interval `s` (default 0.3 s).
    pub cti_min_s: f64,
    /// Maximum number of iterations before declaring non-convergence.
    pub max_iterations: usize,
    /// Lower bound for TDS.
    pub tds_min: f64,
    /// Upper bound for TDS.
    pub tds_max: f64,
}

impl ProtectionCoordinationOptimizer {
    /// Create a new optimizer with default parameters.
    ///
    /// # Arguments
    /// * `relays`       — relays participating in the grading study
    /// * `fault_points` — fault injection points used during analysis
    pub fn new(relays: Vec<OvercurrentRelay>, fault_points: Vec<FaultPoint>) -> Self {
        Self {
            relays,
            coordination_pairs: Vec::new(),
            fault_points,
            objective: CoordinationObjective::MinimizeTotalOperatingTime,
            cti_min_s: 0.3,
            max_iterations: 100,
            tds_min: 0.1,
            tds_max: 10.0,
        }
    }

    /// Register a primary → backup relay coordination pair by relay IDs.
    pub fn add_coordination_pair(&mut self, primary_id: usize, backup_id: usize) {
        self.coordination_pairs.push((primary_id, backup_id));
    }

    // ── Internal helpers ──────────────────────────────────────────────────

    /// Extract curve coefficients for a given characteristic.
    fn curve_coeffs(characteristic: RelayCharacteristic) -> CurveCoeffs {
        match characteristic {
            RelayCharacteristic::StandardInverseIec => CurveCoeffs {
                a: 0.14,
                alpha: 0.02,
                b: 0.0,
                is_definite: false,
            },
            RelayCharacteristic::VeryInverseIec => CurveCoeffs {
                a: 13.5,
                alpha: 1.0,
                b: 0.0,
                is_definite: false,
            },
            RelayCharacteristic::ExtremelyInverseIec => CurveCoeffs {
                a: 80.0,
                alpha: 2.0,
                b: 0.0,
                is_definite: false,
            },
            RelayCharacteristic::DefiniteTime => CurveCoeffs {
                a: 1.0,
                alpha: 0.0,
                b: 0.0,
                is_definite: true,
            },
            RelayCharacteristic::StandardInverseAnsi => CurveCoeffs {
                a: 0.0515,
                alpha: 0.02,
                b: 0.1140,
                is_definite: false,
            },
            RelayCharacteristic::VeryInverseAnsi => CurveCoeffs {
                a: 19.61,
                alpha: 2.0,
                b: 0.4910,
                is_definite: false,
            },
            RelayCharacteristic::ExtremelyInverseAnsi => CurveCoeffs {
                a: 28.2,
                alpha: 2.0,
                b: 0.1217,
                is_definite: false,
            },
            RelayCharacteristic::LongTimeInverseAnsi => CurveCoeffs {
                a: 120.0,
                alpha: 1.0,
                b: 0.0,
                is_definite: false,
            },
        }
    }

    /// Compute the relay operating time for a given fault current.
    ///
    /// Returns 1e9 (a large finite sentinel) when the relay will not operate
    /// (current ≤ pickup) or the formula denominator is ≤ 0.
    pub fn operating_time(relay: &OvercurrentRelay, fault_current_a: f64) -> f64 {
        // Current must exceed pickup
        if fault_current_a <= relay.pickup_current_a {
            return 1e9;
        }

        // Instantaneous element overrides inverse-time curve
        if let Some(inst_pickup) = relay.instantaneous_pickup_a {
            if fault_current_a >= inst_pickup {
                return 0.0;
            }
        }

        let coeffs = Self::curve_coeffs(relay.characteristic);

        if coeffs.is_definite {
            // Definite time: operating time = TDS regardless of current
            return relay.time_dial;
        }

        let m = fault_current_a / relay.pickup_current_a;
        let denom = m.powf(coeffs.alpha) - 1.0;
        if denom <= 0.0 {
            return 1e9;
        }

        relay.time_dial * (coeffs.a / denom + coeffs.b)
    }

    /// Compute the inverse TDS formula: given a desired operating time at a
    /// known fault current, return the required TDS.
    ///
    /// Returns `None` when no finite solution exists (e.g., current ≤ pickup).
    fn required_tds(
        characteristic: RelayCharacteristic,
        pickup_a: f64,
        fault_current_a: f64,
        target_time_s: f64,
    ) -> Option<f64> {
        if fault_current_a <= pickup_a || target_time_s <= 0.0 {
            return None;
        }

        let coeffs = Self::curve_coeffs(characteristic);

        if coeffs.is_definite {
            // For definite time, TDS = target time directly
            return Some(target_time_s);
        }

        let m = fault_current_a / pickup_a;
        let denom = m.powf(coeffs.alpha) - 1.0;
        if denom <= 0.0 {
            return None;
        }

        let time_per_unit = coeffs.a / denom + coeffs.b;
        if time_per_unit <= 0.0 {
            return None;
        }

        Some(target_time_s / time_per_unit)
    }

    // ── Public API ────────────────────────────────────────────────────────

    /// Compute coordination metrics for one (primary, backup) pair at a given fault current.
    pub fn compute_coordination_pair(
        primary: &OvercurrentRelay,
        backup: &OvercurrentRelay,
        fault_current_a: f64,
        cti_min_s: f64,
    ) -> CoordinationPair {
        let t_primary = Self::operating_time(primary, fault_current_a);
        let t_backup = Self::operating_time(backup, fault_current_a);
        let cti = t_backup - t_primary;
        CoordinationPair {
            primary_relay_id: primary.id,
            backup_relay_id: backup.id,
            fault_current_a,
            primary_time_s: t_primary,
            backup_time_s: t_backup,
            coordination_time_interval_s: cti,
            is_coordinated: cti >= cti_min_s,
        }
    }

    /// Evaluate all registered coordination pairs against the provided relay settings.
    pub fn check_coordination(&self, settings: &[OvercurrentRelay]) -> Vec<CoordinationPair> {
        let relay_ref = |id: usize| settings.iter().find(|r| r.id == id);

        self.coordination_pairs
            .iter()
            .filter_map(|&(pid, bid)| {
                let primary = relay_ref(pid)?;
                let backup = relay_ref(bid)?;
                // Use primary relay's max fault current for the coordination check
                let i_fault = primary.max_fault_current_a;
                Some(Self::compute_coordination_pair(
                    primary,
                    backup,
                    i_fault,
                    self.cti_min_s,
                ))
            })
            .collect()
    }

    /// LP-inspired iterative solver: minimize Σ TDS_i subject to CTI ≥ cti_min.
    ///
    /// Algorithm:
    /// 1. Initialize all TDS to `tds_min`.
    /// 2. For each pair, compute primary operating time and the TDS_backup
    ///    needed so that backup time = primary time + cti_min.
    /// 3. Update backup TDS (take max with current value to be monotone).
    /// 4. Clamp all TDS to [tds_min, tds_max].
    /// 5. Repeat until max TDS change < 1e-4 or max_iterations reached.
    pub fn solve_linear_programming(&self) -> CoordinationSolution {
        let mut settings: Vec<OvercurrentRelay> = self.relays.clone();

        // Initialize to minimum TDS
        for r in settings.iter_mut() {
            r.time_dial = self.tds_min;
        }

        let mut converged = false;
        let mut iterations = 0usize;

        for _iter in 0..self.max_iterations {
            iterations += 1;
            let mut max_change = 0.0_f64;

            // Process pairs in order — primary must be fixed before backup
            for &(pid, bid) in &self.coordination_pairs {
                // Snapshot primary
                let primary_tds;
                let primary_char;
                let primary_pickup;
                let primary_max_fault;
                {
                    let Some(primary) = settings.iter().find(|r| r.id == pid) else {
                        continue;
                    };
                    primary_tds = primary.time_dial;
                    primary_char = primary.characteristic;
                    primary_pickup = primary.pickup_current_a;
                    primary_max_fault = primary.max_fault_current_a;
                    let _ = primary_tds; // silence unused warning
                    let _ = primary_char;
                }

                // Compute primary operating time
                let t_primary = {
                    let Some(primary) = settings.iter().find(|r| r.id == pid) else {
                        continue;
                    };
                    Self::operating_time(primary, primary_max_fault)
                };

                if t_primary >= 1e8 {
                    // Primary doesn't operate → skip
                    continue;
                }

                let t_backup_required = t_primary + self.cti_min_s;

                // Solve for required TDS_backup
                let Some(backup) = settings.iter().find(|r| r.id == bid) else {
                    continue;
                };
                let backup_char = backup.characteristic;
                let backup_pickup = backup.pickup_current_a;
                let old_tds = backup.time_dial;

                let new_tds_opt = Self::required_tds(
                    backup_char,
                    backup_pickup,
                    primary_max_fault,
                    t_backup_required,
                );

                let new_tds = match new_tds_opt {
                    Some(t) => t.clamp(self.tds_min, self.tds_max),
                    None => self.tds_min,
                };

                // Take the maximum to ensure monotone increase
                let applied_tds = new_tds.max(old_tds).clamp(self.tds_min, self.tds_max);

                let change = (applied_tds - old_tds).abs();
                if change > max_change {
                    max_change = change;
                }

                // Apply update
                if let Some(backup_mut) = settings.iter_mut().find(|r| r.id == bid) {
                    backup_mut.time_dial = applied_tds;
                }

                // Suppress unused variable warnings for snapshotted values
                let _ = primary_tds;
                let _ = primary_char;
                let _ = primary_pickup;
            }

            if max_change < 1e-4 {
                converged = true;
                break;
            }
        }

        let pairs = self.check_coordination(&settings);
        let selectivity = self.compute_selectivity_index(&pairs);
        let n_coordinated = pairs.iter().filter(|p| p.is_coordinated).count();
        let total_time: f64 = settings
            .iter()
            .map(|r| Self::operating_time(r, r.max_fault_current_a))
            .filter(|&t| t < 1e8)
            .sum();

        CoordinationSolution {
            n_pairs_total: pairs.len(),
            n_pairs_coordinated: n_coordinated,
            selectivity_index: selectivity,
            total_operating_time_s: total_time,
            iterations,
            converged,
            relay_settings: settings,
            coordination_pairs: pairs,
        }
    }

    /// Sequential optimization: set backup relay TDS based on primary.
    ///
    /// Processes pairs in order; each backup relay's TDS is set to satisfy
    /// CTI ≥ cti_min at the primary relay's maximum fault current.
    pub fn solve_sequential_optimization(&self) -> CoordinationSolution {
        let mut settings: Vec<OvercurrentRelay> = self.relays.clone();

        // Start with minimum TDS
        for r in settings.iter_mut() {
            r.time_dial = self.tds_min;
        }

        let mut iterations = 0usize;

        for &(pid, bid) in &self.coordination_pairs {
            iterations += 1;

            let (t_primary, fault_current, backup_char, backup_pickup, old_tds) = {
                let Some(primary) = settings.iter().find(|r| r.id == pid) else {
                    continue;
                };
                let fc = primary.max_fault_current_a;
                let tp = Self::operating_time(primary, fc);
                let Some(backup) = settings.iter().find(|r| r.id == bid) else {
                    continue;
                };
                (
                    tp,
                    fc,
                    backup.characteristic,
                    backup.pickup_current_a,
                    backup.time_dial,
                )
            };

            if t_primary >= 1e8 {
                continue;
            }

            let t_required = t_primary + self.cti_min_s;

            let new_tds = Self::required_tds(backup_char, backup_pickup, fault_current, t_required)
                .unwrap_or(self.tds_min)
                .clamp(self.tds_min, self.tds_max);

            let applied = new_tds.max(old_tds).clamp(self.tds_min, self.tds_max);

            if let Some(bk) = settings.iter_mut().find(|r| r.id == bid) {
                bk.time_dial = applied;
            }
        }

        let pairs = self.check_coordination(&settings);
        let selectivity = self.compute_selectivity_index(&pairs);
        let n_coordinated = pairs.iter().filter(|p| p.is_coordinated).count();
        let total_time: f64 = settings
            .iter()
            .map(|r| Self::operating_time(r, r.max_fault_current_a))
            .filter(|&t| t < 1e8)
            .sum();

        CoordinationSolution {
            n_pairs_total: pairs.len(),
            n_pairs_coordinated: n_coordinated,
            selectivity_index: selectivity,
            total_operating_time_s: total_time,
            iterations,
            converged: true, // sequential always "completes"
            relay_settings: settings,
            coordination_pairs: pairs,
        }
    }

    /// Run the best available optimization method given the objective.
    pub fn optimize(&self) -> CoordinationSolution {
        match self.objective {
            CoordinationObjective::MinimizeTotalOperatingTime => self.solve_linear_programming(),
            CoordinationObjective::MaximizeSelectivity => self.solve_linear_programming(),
            CoordinationObjective::MinimizeBackupTime => self.solve_sequential_optimization(),
            CoordinationObjective::BalancedCoordination => {
                // Run both and pick the one with higher selectivity
                let lp = self.solve_linear_programming();
                let seq = self.solve_sequential_optimization();
                if lp.selectivity_index >= seq.selectivity_index {
                    lp
                } else {
                    seq
                }
            }
        }
    }

    /// Verify that each relay can detect its minimum fault current.
    ///
    /// Detection criterion: `min_fault_current_a > 1.5 × pickup_current_a`.
    ///
    /// Returns a vector of `(relay_id, can_detect)`.
    pub fn verify_sensitivity(&self, settings: &[OvercurrentRelay]) -> Vec<(usize, bool)> {
        settings
            .iter()
            .map(|r| {
                let sensitivity_margin = 1.5;
                let detects = r.min_fault_current_a > sensitivity_margin * r.pickup_current_a;
                (r.id, detects)
            })
            .collect()
    }

    /// Compute the selectivity index as the fraction of coordinated pairs.
    pub fn compute_selectivity_index(&self, pairs: &[CoordinationPair]) -> f64 {
        if pairs.is_empty() {
            return 1.0; // vacuously fully selective
        }
        let n_ok = pairs.iter().filter(|p| p.is_coordinated).count();
        n_ok as f64 / pairs.len() as f64
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Fault Coordination Analyzer
// ─────────────────────────────────────────────────────────────────────────────

/// Standalone analysis tool for time–current characteristics and coordination margin assessment.
pub struct FaultCoordinationAnalyzer;

impl FaultCoordinationAnalyzer {
    /// Compute the time–current characteristic curve for `relay` at the given currents.
    ///
    /// Returns a vector of `(current `A`, operating_time `s`)` pairs.
    /// Entries where the relay does not operate (time ≥ 1e8) are excluded.
    pub fn compute_time_current_curve(
        relay: &OvercurrentRelay,
        currents: &[f64],
    ) -> Vec<(f64, f64)> {
        currents
            .iter()
            .filter_map(|&i| {
                let t = ProtectionCoordinationOptimizer::operating_time(relay, i);
                if t < 1e8 {
                    Some((i, t))
                } else {
                    None
                }
            })
            .collect()
    }

    /// Find the maximum fault current at which primary and backup remain coordinated.
    ///
    /// Performs a binary search between `primary.pickup_current_a * 1.5` and
    /// `primary.max_fault_current_a`.  Returns a finite positive value.
    pub fn find_coordination_limit(
        primary: &OvercurrentRelay,
        backup: &OvercurrentRelay,
        cti_min_s: f64,
    ) -> f64 {
        let i_lo = primary.pickup_current_a * 1.5;
        let i_hi = primary.max_fault_current_a;

        if i_lo >= i_hi {
            return i_lo;
        }

        // Check whether fully coordinated at max fault
        let pair_hi = ProtectionCoordinationOptimizer::compute_coordination_pair(
            primary, backup, i_hi, cti_min_s,
        );
        if pair_hi.is_coordinated {
            return i_hi;
        }

        // Binary search for coordination limit
        let mut lo = i_lo;
        let mut hi = i_hi;
        for _ in 0..64 {
            let mid = 0.5 * (lo + hi);
            let p = ProtectionCoordinationOptimizer::compute_coordination_pair(
                primary, backup, mid, cti_min_s,
            );
            if p.is_coordinated {
                lo = mid;
            } else {
                hi = mid;
            }
            if hi - lo < 0.1 {
                break;
            }
        }
        lo
    }

    /// Compute the average CTI margin across all coordination pairs.
    ///
    /// Only considers pairs with finite backup times (CTI < 1e8).
    /// Returns 0.0 for an empty slice.
    pub fn assess_grading_margin(pairs: &[CoordinationPair]) -> f64 {
        let valid: Vec<f64> = pairs
            .iter()
            .map(|p| p.coordination_time_interval_s)
            .filter(|&cti| cti.is_finite() && cti < 1e8)
            .collect();
        if valid.is_empty() {
            return 0.0;
        }
        valid.iter().sum::<f64>() / valid.len() as f64
    }

    /// Return references to all pairs that do not satisfy the CTI requirement.
    pub fn identify_coordination_violations(pairs: &[CoordinationPair]) -> Vec<&CoordinationPair> {
        pairs.iter().filter(|p| !p.is_coordinated).collect()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Helpers ──────────────────────────────────────────────────────────────

    fn make_relay(
        id: usize,
        characteristic: RelayCharacteristic,
        pickup_a: f64,
        tds: f64,
        max_fault: f64,
        min_fault: f64,
    ) -> OvercurrentRelay {
        OvercurrentRelay {
            id,
            name: format!("R{}", id),
            location_bus: id,
            protected_branch: (id, id + 1),
            characteristic,
            pickup_current_a: pickup_a,
            time_dial: tds,
            instantaneous_pickup_a: None,
            ct_ratio: 200.0,
            max_fault_current_a: max_fault,
            min_fault_current_a: min_fault,
        }
    }

    // ── Formula verification ──────────────────────────────────────────────────

    #[test]
    fn test_iec_standard_inverse_formula() {
        // t = TDS * 0.14 / ((I/Is)^0.02 - 1)
        // At I/Is = 10, TDS = 1.0:
        // denom = 10^0.02 - 1 = 1.04713 - 1 = 0.04713
        // t ≈ 0.14 / 0.04713 ≈ 2.971 s
        let relay = make_relay(
            0,
            RelayCharacteristic::StandardInverseIec,
            100.0,
            1.0,
            5000.0,
            150.0,
        );
        let t = ProtectionCoordinationOptimizer::operating_time(&relay, 1000.0);
        let expected = 1.0 * 0.14 / (10.0_f64.powf(0.02) - 1.0);
        assert!(
            (t - expected).abs() < 1e-6,
            "IEC SI at I/Is=10: got {:.6}, expected {:.6}",
            t,
            expected
        );
    }

    #[test]
    fn test_iec_very_inverse_formula() {
        // t = TDS * 13.5 / ((I/Is)^1.0 - 1)
        // At I/Is = 5, TDS = 0.5:
        // t = 0.5 * 13.5 / (5 - 1) = 0.5 * 3.375 = 1.6875
        let relay = make_relay(
            1,
            RelayCharacteristic::VeryInverseIec,
            100.0,
            0.5,
            5000.0,
            150.0,
        );
        let t = ProtectionCoordinationOptimizer::operating_time(&relay, 500.0);
        let expected = 0.5 * 13.5 / (5.0 - 1.0);
        assert!(
            (t - expected).abs() < 1e-9,
            "IEC VI at I/Is=5: got {:.6}, expected {:.6}",
            t,
            expected
        );
    }

    #[test]
    fn test_iec_extremely_inverse_formula() {
        // t = TDS * 80.0 / ((I/Is)^2.0 - 1)
        // At I/Is = 3, TDS = 0.2:
        // t = 0.2 * 80 / (9 - 1) = 0.2 * 10 = 2.0
        let relay = make_relay(
            2,
            RelayCharacteristic::ExtremelyInverseIec,
            100.0,
            0.2,
            5000.0,
            150.0,
        );
        let t = ProtectionCoordinationOptimizer::operating_time(&relay, 300.0);
        let expected = 0.2 * 80.0 / (3.0_f64.powi(2) - 1.0);
        assert!(
            (t - expected).abs() < 1e-9,
            "IEC EI at I/Is=3: got {:.6}, expected {:.6}",
            t,
            expected
        );
    }

    #[test]
    fn test_definite_time_constant() {
        // For definite time, operating time = TDS regardless of current.
        let relay = make_relay(
            3,
            RelayCharacteristic::DefiniteTime,
            100.0,
            2.5,
            5000.0,
            150.0,
        );
        let t1 = ProtectionCoordinationOptimizer::operating_time(&relay, 200.0);
        let t2 = ProtectionCoordinationOptimizer::operating_time(&relay, 5000.0);
        assert!((t1 - 2.5).abs() < 1e-9, "DT at low current: got {:.6}", t1);
        assert!((t2 - 2.5).abs() < 1e-9, "DT at high current: got {:.6}", t2);
    }

    // ── Coordination pair checks ──────────────────────────────────────────────

    #[test]
    fn test_coordination_pair_coordinated() {
        // Primary: TDS=0.1, Backup: TDS=0.6 → CTI should be ≥ 0.3s
        let primary = make_relay(
            0,
            RelayCharacteristic::VeryInverseIec,
            100.0,
            0.1,
            1000.0,
            200.0,
        );
        let mut backup = make_relay(
            1,
            RelayCharacteristic::VeryInverseIec,
            100.0,
            0.6,
            1000.0,
            200.0,
        );
        backup.max_fault_current_a = 1000.0;
        let pair = ProtectionCoordinationOptimizer::compute_coordination_pair(
            &primary, &backup, 1000.0, 0.3,
        );
        assert!(
            pair.is_coordinated,
            "CTI={:.3} should be ≥ 0.3",
            pair.coordination_time_interval_s
        );
        assert!(pair.coordination_time_interval_s >= 0.3);
    }

    #[test]
    fn test_coordination_pair_not_coordinated() {
        // Primary: TDS=0.5, Backup: TDS=0.51 → CTI < 0.3s
        let primary = make_relay(
            0,
            RelayCharacteristic::VeryInverseIec,
            100.0,
            0.5,
            1000.0,
            200.0,
        );
        let backup = make_relay(
            1,
            RelayCharacteristic::VeryInverseIec,
            100.0,
            0.51,
            1000.0,
            200.0,
        );
        let pair = ProtectionCoordinationOptimizer::compute_coordination_pair(
            &primary, &backup, 1000.0, 0.3,
        );
        assert!(
            !pair.is_coordinated,
            "CTI={:.3} should be < 0.3",
            pair.coordination_time_interval_s
        );
    }

    // ── Optimization tests ────────────────────────────────────────────────────

    #[test]
    fn test_sequential_optimization_basic() {
        // Two relays: primary R0, backup R1.
        // After optimization, backup operating time ≥ primary + 0.3s.
        let r0 = make_relay(
            0,
            RelayCharacteristic::VeryInverseIec,
            100.0,
            0.1,
            1000.0,
            200.0,
        );
        let r1 = make_relay(
            1,
            RelayCharacteristic::VeryInverseIec,
            80.0,
            0.1,
            1000.0,
            150.0,
        );
        let mut opt = ProtectionCoordinationOptimizer::new(vec![r0, r1], vec![]);
        opt.add_coordination_pair(0, 1);

        let sol = opt.solve_sequential_optimization();
        assert!(!sol.relay_settings.is_empty());

        // Verify CTI for the pair
        let pair = &sol.coordination_pairs[0];
        assert!(
            pair.backup_time_s >= pair.primary_time_s + 0.3 - 1e-6,
            "backup={:.4} primary={:.4} CTI={:.4}",
            pair.backup_time_s,
            pair.primary_time_s,
            pair.coordination_time_interval_s
        );
    }

    #[test]
    fn test_lp_optimization_converges() {
        let r0 = make_relay(
            0,
            RelayCharacteristic::StandardInverseIec,
            100.0,
            0.1,
            2000.0,
            300.0,
        );
        let r1 = make_relay(
            1,
            RelayCharacteristic::StandardInverseIec,
            80.0,
            0.1,
            2000.0,
            200.0,
        );
        let mut opt = ProtectionCoordinationOptimizer::new(vec![r0, r1], vec![]);
        opt.add_coordination_pair(0, 1);

        let sol = opt.solve_linear_programming();
        assert!(
            sol.converged,
            "LP solver should converge for a 2-relay case"
        );
        assert!(sol.iterations > 0);
    }

    // ── Selectivity index ─────────────────────────────────────────────────────

    #[test]
    fn test_selectivity_index_all_coordinated() {
        let pairs = vec![
            CoordinationPair {
                primary_relay_id: 0,
                backup_relay_id: 1,
                fault_current_a: 1000.0,
                primary_time_s: 0.5,
                backup_time_s: 0.9,
                coordination_time_interval_s: 0.4,
                is_coordinated: true,
            },
            CoordinationPair {
                primary_relay_id: 1,
                backup_relay_id: 2,
                fault_current_a: 800.0,
                primary_time_s: 0.9,
                backup_time_s: 1.3,
                coordination_time_interval_s: 0.4,
                is_coordinated: true,
            },
        ];
        let opt = ProtectionCoordinationOptimizer::new(vec![], vec![]);
        let idx = opt.compute_selectivity_index(&pairs);
        assert!(
            (idx - 1.0).abs() < 1e-9,
            "all coordinated → index=1.0, got {:.4}",
            idx
        );
    }

    #[test]
    fn test_selectivity_index_none() {
        let pairs = vec![
            CoordinationPair {
                primary_relay_id: 0,
                backup_relay_id: 1,
                fault_current_a: 1000.0,
                primary_time_s: 0.5,
                backup_time_s: 0.7,
                coordination_time_interval_s: 0.2,
                is_coordinated: false,
            },
            CoordinationPair {
                primary_relay_id: 1,
                backup_relay_id: 2,
                fault_current_a: 800.0,
                primary_time_s: 0.9,
                backup_time_s: 1.0,
                coordination_time_interval_s: 0.1,
                is_coordinated: false,
            },
        ];
        let opt = ProtectionCoordinationOptimizer::new(vec![], vec![]);
        let idx = opt.compute_selectivity_index(&pairs);
        assert!(
            idx.abs() < 1e-9,
            "none coordinated → index=0.0, got {:.4}",
            idx
        );
    }

    // ── Sensitivity checks ────────────────────────────────────────────────────

    #[test]
    fn test_sensitivity_check_detection() {
        // min_fault = 300 A, pickup = 100 A → 300 > 1.5*100 = 150 → detects
        let relay = make_relay(
            0,
            RelayCharacteristic::VeryInverseIec,
            100.0,
            0.5,
            2000.0,
            300.0,
        );
        let opt = ProtectionCoordinationOptimizer::new(vec![relay.clone()], vec![]);
        let result = opt.verify_sensitivity(&[relay]);
        assert!(
            result[0].1,
            "relay should detect min fault current of 300 A"
        );
    }

    #[test]
    fn test_sensitivity_check_failure() {
        // min_fault = 140 A, pickup = 100 A → 140 < 1.5*100 = 150 → fails
        let relay = make_relay(
            0,
            RelayCharacteristic::VeryInverseIec,
            100.0,
            0.5,
            2000.0,
            140.0,
        );
        let opt = ProtectionCoordinationOptimizer::new(vec![relay.clone()], vec![]);
        let result = opt.verify_sensitivity(&[relay]);
        assert!(
            !result[0].1,
            "relay should NOT detect min fault current of 140 A with pickup 100 A"
        );
    }

    // ── TDS bounds ────────────────────────────────────────────────────────────

    #[test]
    fn test_tds_bounds_respected() {
        // Even if the required TDS would be very large, it must be ≤ 10.0
        let r0 = make_relay(
            0,
            RelayCharacteristic::StandardInverseIec,
            100.0,
            0.1,
            110.0, // Very close to pickup → very long operating time
            105.0,
        );
        let r1 = make_relay(
            1,
            RelayCharacteristic::StandardInverseIec,
            100.0,
            0.1,
            110.0,
            105.0,
        );
        let mut opt = ProtectionCoordinationOptimizer::new(vec![r0, r1], vec![]);
        opt.add_coordination_pair(0, 1);

        let sol = opt.solve_linear_programming();
        for r in &sol.relay_settings {
            assert!(
                r.time_dial >= opt.tds_min - 1e-9 && r.time_dial <= opt.tds_max + 1e-9,
                "TDS={:.3} out of bounds [{}, {}]",
                r.time_dial,
                opt.tds_min,
                opt.tds_max
            );
        }
    }

    // ── Time–current curve ────────────────────────────────────────────────────

    #[test]
    fn test_time_current_curve() {
        let relay = make_relay(
            0,
            RelayCharacteristic::VeryInverseIec,
            100.0,
            0.5,
            5000.0,
            150.0,
        );
        let currents: Vec<f64> = (2..=20).map(|x| x as f64 * 100.0).collect();
        let curve = FaultCoordinationAnalyzer::compute_time_current_curve(&relay, &currents);
        assert!(!curve.is_empty(), "curve should have entries");
        // Times should be monotonically decreasing as current increases
        for i in 1..curve.len() {
            assert!(
                curve[i].1 <= curve[i - 1].1 + 1e-9,
                "time not decreasing at I={:.1}: t_prev={:.4} t_curr={:.4}",
                curve[i].0,
                curve[i - 1].1,
                curve[i].1
            );
        }
    }

    // ── Coordination limit ────────────────────────────────────────────────────

    #[test]
    fn test_coordination_limit() {
        let primary = make_relay(
            0,
            RelayCharacteristic::VeryInverseIec,
            100.0,
            0.2,
            5000.0,
            200.0,
        );
        let backup = make_relay(
            1,
            RelayCharacteristic::VeryInverseIec,
            100.0,
            0.7,
            5000.0,
            200.0,
        );
        let limit = FaultCoordinationAnalyzer::find_coordination_limit(&primary, &backup, 0.3);
        assert!(
            limit > 0.0 && limit.is_finite(),
            "coordination limit should be finite positive, got {:.3}",
            limit
        );
    }

    // ── Grading margin ────────────────────────────────────────────────────────

    #[test]
    fn test_grading_margin() {
        let pairs = vec![
            CoordinationPair {
                primary_relay_id: 0,
                backup_relay_id: 1,
                fault_current_a: 1000.0,
                primary_time_s: 0.4,
                backup_time_s: 0.8,
                coordination_time_interval_s: 0.4,
                is_coordinated: true,
            },
            CoordinationPair {
                primary_relay_id: 1,
                backup_relay_id: 2,
                fault_current_a: 800.0,
                primary_time_s: 0.8,
                backup_time_s: 1.2,
                coordination_time_interval_s: 0.4,
                is_coordinated: true,
            },
        ];
        let margin = FaultCoordinationAnalyzer::assess_grading_margin(&pairs);
        assert!(
            margin > 0.0,
            "grading margin should be positive: {:.4}",
            margin
        );
        assert!(
            (margin - 0.4).abs() < 1e-9,
            "expected 0.4 s average, got {:.4}",
            margin
        );
    }

    // ── Violation identification ──────────────────────────────────────────────

    #[test]
    fn test_violations_identified() {
        let pairs = vec![
            CoordinationPair {
                primary_relay_id: 0,
                backup_relay_id: 1,
                fault_current_a: 1000.0,
                primary_time_s: 0.4,
                backup_time_s: 0.8,
                coordination_time_interval_s: 0.4,
                is_coordinated: true,
            },
            CoordinationPair {
                primary_relay_id: 1,
                backup_relay_id: 2,
                fault_current_a: 800.0,
                primary_time_s: 0.8,
                backup_time_s: 0.9,
                coordination_time_interval_s: 0.1,
                is_coordinated: false,
            },
        ];
        let violations = FaultCoordinationAnalyzer::identify_coordination_violations(&pairs);
        assert_eq!(violations.len(), 1, "exactly one violation expected");
        assert_eq!(violations[0].primary_relay_id, 1);
    }

    // ── 3-relay chain ─────────────────────────────────────────────────────────

    #[test]
    fn test_3_relay_chain() {
        // R0 (primary) → R1 (backup for R0) → R2 (backup for R1)
        // After LP optimization all pairs should be coordinated.
        let r0 = make_relay(
            0,
            RelayCharacteristic::VeryInverseIec,
            100.0,
            0.1,
            3000.0,
            200.0,
        );
        let r1 = make_relay(
            1,
            RelayCharacteristic::VeryInverseIec,
            80.0,
            0.1,
            3000.0,
            150.0,
        );
        let r2 = make_relay(
            2,
            RelayCharacteristic::VeryInverseIec,
            60.0,
            0.1,
            3000.0,
            120.0,
        );

        let mut opt = ProtectionCoordinationOptimizer::new(vec![r0, r1, r2], vec![]);
        opt.add_coordination_pair(0, 1);
        opt.add_coordination_pair(1, 2);

        let sol = opt.solve_linear_programming();
        assert!(
            sol.n_pairs_coordinated == sol.n_pairs_total,
            "all 3-relay-chain pairs should be coordinated: {}/{}",
            sol.n_pairs_coordinated,
            sol.n_pairs_total
        );
    }

    // ── Operating time above pickup ───────────────────────────────────────────

    #[test]
    fn test_operating_time_above_pickup() {
        let relay = make_relay(
            0,
            RelayCharacteristic::VeryInverseIec,
            500.0,
            0.5,
            5000.0,
            600.0,
        );
        // Below pickup → sentinel
        let t_below = ProtectionCoordinationOptimizer::operating_time(&relay, 400.0);
        assert!(
            t_below >= 1e9 - 1.0,
            "below pickup should return sentinel, got {:.4}",
            t_below
        );
        // At exactly pickup → sentinel
        let t_at = ProtectionCoordinationOptimizer::operating_time(&relay, 500.0);
        assert!(
            t_at >= 1e9 - 1.0,
            "at pickup should return sentinel, got {:.4}",
            t_at
        );
        // Above pickup → finite operating time
        let t_above = ProtectionCoordinationOptimizer::operating_time(&relay, 600.0);
        assert!(
            t_above < 1e8,
            "above pickup should return finite time, got {:.4}",
            t_above
        );
    }

    // ── Instantaneous element ─────────────────────────────────────────────────

    #[test]
    fn test_instantaneous_element() {
        let mut relay = make_relay(
            0,
            RelayCharacteristic::VeryInverseIec,
            100.0,
            1.0,
            5000.0,
            200.0,
        );
        relay.instantaneous_pickup_a = Some(2000.0);

        // Below instantaneous → inverse-time operates (takes several seconds)
        let t_normal = ProtectionCoordinationOptimizer::operating_time(&relay, 500.0);
        assert!(
            t_normal > 0.1,
            "should use inverse-time below inst pickup: {:.4}",
            t_normal
        );

        // Above instantaneous → instant trip (0.0 s)
        let t_inst = ProtectionCoordinationOptimizer::operating_time(&relay, 3000.0);
        assert!(
            t_inst == 0.0,
            "should be 0.0 s for instantaneous element, got {:.6}",
            t_inst
        );
    }

    // ── ANSI curves ───────────────────────────────────────────────────────────

    #[test]
    fn test_ansi_standard_inverse_formula() {
        // t = TDS * (0.0515/((I/Is)^0.02 - 1) + 0.1140)
        let relay = make_relay(
            0,
            RelayCharacteristic::StandardInverseAnsi,
            100.0,
            1.0,
            5000.0,
            200.0,
        );
        let i = 1000.0; // I/Is = 10
        let m = 10.0_f64;
        let expected = 1.0 * (0.0515 / (m.powf(0.02) - 1.0) + 0.1140);
        let t = ProtectionCoordinationOptimizer::operating_time(&relay, i);
        assert!(
            (t - expected).abs() < 1e-6,
            "ANSI SI at I/Is=10: got {:.6}, expected {:.6}",
            t,
            expected
        );
    }

    #[test]
    fn test_ansi_very_inverse_formula() {
        // t = TDS * (19.61/((I/Is)^2.0 - 1) + 0.4910)
        let relay = make_relay(
            0,
            RelayCharacteristic::VeryInverseAnsi,
            100.0,
            0.5,
            5000.0,
            200.0,
        );
        let i = 500.0; // I/Is = 5
        let m = 5.0_f64;
        let expected = 0.5 * (19.61 / (m.powi(2) - 1.0) + 0.4910);
        let t = ProtectionCoordinationOptimizer::operating_time(&relay, i);
        assert!(
            (t - expected).abs() < 1e-6,
            "ANSI VI at I/Is=5: got {:.6}, expected {:.6}",
            t,
            expected
        );
    }

    #[test]
    fn test_optimize_returns_solution() {
        let r0 = make_relay(
            0,
            RelayCharacteristic::ExtremelyInverseIec,
            100.0,
            0.1,
            2000.0,
            250.0,
        );
        let r1 = make_relay(
            1,
            RelayCharacteristic::ExtremelyInverseIec,
            80.0,
            0.1,
            2000.0,
            200.0,
        );
        let mut opt = ProtectionCoordinationOptimizer::new(vec![r0, r1], vec![]);
        opt.add_coordination_pair(0, 1);
        opt.objective = CoordinationObjective::BalancedCoordination;

        let sol = opt.optimize();
        assert_eq!(sol.relay_settings.len(), 2);
        assert!(sol.selectivity_index >= 0.0 && sol.selectivity_index <= 1.0);
    }
}
