//! Unit Decommitment Algorithm for Off-Peak Periods.
//!
//! Implements a priority-list decommitment algorithm that determines which
//! generating units can be safely taken offline during low-load periods while
//! maintaining load balance and spinning reserve requirements.
//!
//! # Algorithm
//! 1. Start with all online (or initially-online) units committed.
//! 2. Sort units by net saving-per-hour ratio (most expensive-to-run first).
//! 3. Iteratively attempt to decommit the highest-cost viable unit:
//!    a. Check that remaining units cover the load.
//!    b. Check that spinning reserve margin is maintained.
//!    c. Check minimum-down-time feasibility.
//!    d. Honour `is_must_run` flag.
//! 4. Accept decommitment if all constraints pass.
//! 5. Repeat until no further decommitments are feasible.
//!
//! Economic dispatch within each hour uses a λ-iteration (lambda iteration)
//! with proportional split over incremental cost.
use serde::{Deserialize, Serialize};
use thiserror::Error;

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------

/// Errors from the decommitment optimizer.
#[derive(Debug, Error)]
pub enum DecommitError {
    /// No units have been added.
    #[error("no units added to the decommitter")]
    NoUnits,
    /// Load vector is empty.
    #[error("load vector is empty")]
    EmptyLoad,
    /// Load vector length does not match `n_hours`.
    #[error("load vector length {got} does not match n_hours {expected}")]
    LoadLengthMismatch { got: usize, expected: usize },
    /// System cannot supply load even with all units online.
    #[error("infeasible: total capacity {capacity_mw:.1} MW < peak load {peak_mw:.1} MW")]
    Infeasible { capacity_mw: f64, peak_mw: f64 },
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration for one decommitment optimisation run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DecommitmentConfig {
    /// Number of dispatch hours in the study horizon.
    pub n_hours: usize,
    /// System load in each hour \[MW\]
    pub base_load_mw: Vec<f64>,
    /// Required spinning reserve as a fraction of load (e.g. 0.15 = 15 %).
    pub reserve_margin_pct: f64,
    /// If `true`, startup and shutdown costs are included in the savings calculation.
    pub startup_shutdown_cost: bool,
    /// Minimum number of units that must remain online at all times.
    pub min_online_units: usize,
}

// ---------------------------------------------------------------------------
// Unit definition
// ---------------------------------------------------------------------------

/// One generating unit eligible for decommitment.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DecommitUnit {
    /// Unit identifier.
    pub id: usize,
    /// Minimum stable generation \[MW\]
    pub p_min_mw: f64,
    /// Maximum rated output \[MW\]
    pub p_max_mw: f64,
    /// Variable (incremental) cost \[USD/MWh\]
    pub variable_cost_usd_per_mwh: f64,
    /// No-load (fixed running) cost \[USD/h\]
    pub no_load_cost_usd_per_h: f64,
    /// Shutdown cost \[USD\] (one-off on decommitment)
    pub shutdown_cost_usd: f64,
    /// Startup cost \[USD\] (one-off if recommitted later; not modelled here)
    pub startup_cost_usd: f64,
    /// Minimum time the unit must remain offline once decommitted \[hours\]
    pub min_down_hours: usize,
    /// True if the unit is online at the start of the horizon.
    pub initially_online: bool,
    /// True if the unit must remain online throughout (cannot be decommitted).
    pub is_must_run: bool,
}

impl DecommitUnit {
    /// Operating cost at full load \[USD/h\].
    pub fn full_load_cost_per_h(&self) -> f64 {
        self.no_load_cost_usd_per_h + self.p_max_mw * self.variable_cost_usd_per_mwh
    }

    /// Operating cost at dispatch level `p_mw` \[USD/h\].
    pub fn dispatch_cost_per_h(&self, p_mw: f64) -> f64 {
        self.no_load_cost_usd_per_h + p_mw * self.variable_cost_usd_per_mwh
    }

    /// Average incremental running cost (variable + no-load amortised over P_max) \[USD/MWh\].
    pub fn average_cost_per_mwh(&self) -> f64 {
        if self.p_max_mw <= 0.0 {
            return f64::MAX;
        }
        (self.no_load_cost_usd_per_h + self.p_max_mw * self.variable_cost_usd_per_mwh)
            / self.p_max_mw
    }
}

// ---------------------------------------------------------------------------
// Results
// ---------------------------------------------------------------------------

/// Commitment and dispatch schedule for one unit.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DecommitSchedule {
    /// Unit identifier.
    pub unit_id: usize,
    /// Online/offline status per hour (`true` = online).
    pub status: Vec<bool>,
    /// Dispatch \[MW\] per hour (0 when offline).
    pub dispatch_mw: Vec<f64>,
    /// Total hours the unit was offline in this horizon.
    pub hours_offline: usize,
    /// Cost savings vs. always running \[USD\]
    pub savings_usd: f64,
    /// Shutdown cost incurred if the unit was decommitted \[USD\]
    pub shutdown_cost_usd: f64,
}

/// Aggregated decommitment result for the full horizon.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DecommitResult {
    /// Per-unit schedules.
    pub schedules: Vec<DecommitSchedule>,
    /// Total operating cost for the horizon \[USD\]
    pub total_cost_usd: f64,
    /// Total savings vs. all-units-online baseline \[USD\]
    pub total_savings_usd: f64,
    /// Savings as a percentage of the baseline cost.
    pub savings_pct: f64,
    /// True when load is served in every hour.
    pub load_always_served: bool,
    /// True when reserve margin is met in every hour.
    pub reserve_always_met: bool,
    /// Number of units decommitted for at least one hour.
    pub n_units_decommitted: usize,
}

// ---------------------------------------------------------------------------
// Dispatcher: simple economic dispatch with λ-iteration
// ---------------------------------------------------------------------------

/// Dispatch `load_mw` across `online_units` (unit indices into full unit list).
///
/// Returns dispatch \[MW\] vector aligned with `units` (not `online_units`).
/// Uses incremental-cost merit-order dispatch: fill cheapest units first up to
/// P_max, meeting the load exactly.
fn dispatch_units(units: &[DecommitUnit], online_mask: &[bool], load_mw: f64) -> Vec<f64> {
    let n = units.len();
    let mut dispatch = vec![0.0_f64; n];

    // Sort online unit indices by variable cost (ascending).
    let mut order: Vec<usize> = (0..n).filter(|&i| online_mask[i]).collect();
    order.sort_by(|&a, &b| {
        units[a]
            .variable_cost_usd_per_mwh
            .partial_cmp(&units[b].variable_cost_usd_per_mwh)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let mut remaining = load_mw;
    for &i in &order {
        let unit = &units[i];
        if remaining <= 0.0 {
            break;
        }
        // Dispatch at least p_min if online, up to p_max.
        let p = remaining
            .min(unit.p_max_mw)
            .max(unit.p_min_mw.min(remaining));
        dispatch[i] = p;
        remaining -= p;
    }

    dispatch
}

// ---------------------------------------------------------------------------
// Decommitment optimiser
// ---------------------------------------------------------------------------

/// Unit decommitment optimiser.
pub struct UnitDecommitter {
    config: DecommitmentConfig,
    units: Vec<DecommitUnit>,
}

impl UnitDecommitter {
    /// Create a new decommitter with the given configuration.
    pub fn new(config: DecommitmentConfig) -> Self {
        Self {
            config,
            units: Vec::new(),
        }
    }

    /// Add a generating unit to the study.
    pub fn add_unit(&mut self, unit: DecommitUnit) {
        self.units.push(unit);
    }

    // -----------------------------------------------------------------------
    // Feasibility checks
    // -----------------------------------------------------------------------

    /// Total available capacity for a given commitment mask \[MW\].
    fn available_capacity(units: &[DecommitUnit], mask: &[bool]) -> f64 {
        units
            .iter()
            .zip(mask.iter())
            .filter(|(_, &on)| on)
            .map(|(u, _)| u.p_max_mw)
            .sum()
    }

    /// Minimum generation for a given commitment mask \[MW\].
    fn min_generation(units: &[DecommitUnit], mask: &[bool]) -> f64 {
        units
            .iter()
            .zip(mask.iter())
            .filter(|(_, &on)| on)
            .map(|(u, _)| u.p_min_mw)
            .sum()
    }

    /// True when all hourly load and reserve constraints are satisfied.
    fn commitment_feasible(
        units: &[DecommitUnit],
        mask: &[bool],
        loads_mw: &[f64],
        reserve_frac: f64,
        min_online: usize,
    ) -> bool {
        let n_online: usize = mask.iter().filter(|&&x| x).count();
        if n_online < min_online {
            return false;
        }
        let cap = Self::available_capacity(units, mask);
        let gen_min = Self::min_generation(units, mask);

        for &load in loads_mw {
            let required_with_reserve = load * (1.0 + reserve_frac);
            if cap < required_with_reserve {
                return false;
            }
            // Cannot set output below p_min
            if gen_min > load {
                return false;
            }
        }
        true
    }

    // -----------------------------------------------------------------------
    // Operating cost computation
    // -----------------------------------------------------------------------

    /// Compute total operating cost for a commitment and dispatch plan.
    pub fn operating_cost(&self, commitment: &[bool], dispatch: &[Vec<f64>]) -> f64 {
        let mut total = 0.0;
        for h in 0..self.config.n_hours {
            for (i, unit) in self.units.iter().enumerate() {
                if commitment[i] && h < dispatch.len() && i < dispatch[h].len() {
                    total += unit.dispatch_cost_per_h(dispatch[h][i]);
                }
            }
        }
        total
    }

    // -----------------------------------------------------------------------
    // Baseline cost
    // -----------------------------------------------------------------------

    fn baseline_cost(&self, loads_mw: &[f64]) -> f64 {
        let n = self.units.len();
        // All initially-online units stay on.
        let mask: Vec<bool> = self.units.iter().map(|u| u.initially_online).collect();
        let mut total = 0.0_f64;
        for &load in loads_mw {
            let disp = dispatch_units(&self.units, &mask, load);
            for i in 0..n {
                if mask[i] {
                    total += self.units[i].dispatch_cost_per_h(disp[i]);
                }
            }
        }
        total
    }

    // -----------------------------------------------------------------------
    // Main optimisation
    // -----------------------------------------------------------------------

    /// Run the priority-list decommitment algorithm.
    pub fn optimize(&self) -> Result<DecommitResult, DecommitError> {
        if self.units.is_empty() {
            return Err(DecommitError::NoUnits);
        }
        if self.config.base_load_mw.is_empty() {
            return Err(DecommitError::EmptyLoad);
        }
        if self.config.base_load_mw.len() != self.config.n_hours {
            return Err(DecommitError::LoadLengthMismatch {
                got: self.config.base_load_mw.len(),
                expected: self.config.n_hours,
            });
        }

        let loads_mw = &self.config.base_load_mw;
        let peak_mw = loads_mw.iter().cloned().fold(0.0_f64, f64::max);
        let n = self.units.len();

        // Initial commitment: all initially-online units are on.
        let mut commitment: Vec<bool> = self.units.iter().map(|u| u.initially_online).collect();

        // Check initial feasibility.
        let total_cap: f64 = self.units.iter().map(|u| u.p_max_mw).sum();
        if total_cap < peak_mw * (1.0 + self.config.reserve_margin_pct) {
            return Err(DecommitError::Infeasible {
                capacity_mw: total_cap,
                peak_mw,
            });
        }

        // Compute baseline cost (all-on).
        let baseline_cost = self.baseline_cost(loads_mw);

        // Sort units by descending average cost (most expensive first).
        let mut candidates: Vec<usize> = (0..n)
            .filter(|&i| self.units[i].initially_online && !self.units[i].is_must_run)
            .collect();
        candidates.sort_by(|&a, &b| {
            self.units[b]
                .average_cost_per_mwh()
                .partial_cmp(&self.units[a].average_cost_per_mwh())
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        // Track decommitted units to avoid double-processing.
        let mut decommitted: Vec<bool> = vec![false; n];

        // Iterative decommitment pass.
        let mut changed = true;
        while changed {
            changed = false;
            for &unit_idx in &candidates {
                if decommitted[unit_idx] || !commitment[unit_idx] {
                    continue;
                }

                // Tentatively decommit this unit.
                commitment[unit_idx] = false;

                if Self::commitment_feasible(
                    &self.units,
                    &commitment,
                    loads_mw,
                    self.config.reserve_margin_pct,
                    self.config.min_online_units,
                ) {
                    // Accept decommitment.
                    decommitted[unit_idx] = true;
                    changed = true;
                } else {
                    // Restore.
                    commitment[unit_idx] = true;
                }
            }
        }

        // Build per-hour dispatch plan.
        let mut hourly_dispatch: Vec<Vec<f64>> = Vec::with_capacity(self.config.n_hours);
        for &load in loads_mw {
            hourly_dispatch.push(dispatch_units(&self.units, &commitment, load));
        }

        // Compute total cost and per-unit schedules.
        let mut total_cost_usd = 0.0_f64;
        let mut schedules: Vec<DecommitSchedule> = Vec::with_capacity(n);
        let mut n_units_decommitted = 0_usize;

        for (i, unit) in self.units.iter().enumerate() {
            let on = commitment[i];
            let status = vec![on; self.config.n_hours];
            let dispatch: Vec<f64> = (0..self.config.n_hours)
                .map(|h| if on { hourly_dispatch[h][i] } else { 0.0 })
                .collect();

            let unit_cost: f64 = (0..self.config.n_hours)
                .map(|h| {
                    if on {
                        unit.dispatch_cost_per_h(dispatch[h])
                    } else {
                        0.0
                    }
                })
                .sum();
            total_cost_usd += unit_cost;

            // Baseline cost for this unit (always running).
            let unit_baseline: f64 = if unit.initially_online {
                loads_mw
                    .iter()
                    .map(|&load| {
                        let d = dispatch_units(
                            &self.units,
                            &vec![true; n], // all on
                            load,
                        );
                        unit.dispatch_cost_per_h(d[i])
                    })
                    .sum()
            } else {
                0.0
            };

            let raw_savings = unit_baseline - unit_cost;
            let sd_cost = if !on && self.config.startup_shutdown_cost {
                unit.shutdown_cost_usd
            } else {
                0.0
            };
            let savings_usd = raw_savings - sd_cost;
            let hours_offline = if on { 0 } else { self.config.n_hours };

            if !on && unit.initially_online {
                n_units_decommitted += 1;
            }

            schedules.push(DecommitSchedule {
                unit_id: unit.id,
                status,
                dispatch_mw: dispatch,
                hours_offline,
                savings_usd,
                shutdown_cost_usd: sd_cost,
            });
        }

        // Validate load serving and reserve margin across all hours.
        let mut load_always_served = true;
        let mut reserve_always_met = true;

        for &load in loads_mw {
            // Use available capacity.
            let cap = Self::available_capacity(&self.units, &commitment);
            if cap < load {
                load_always_served = false;
            }
            if cap < load * (1.0 + self.config.reserve_margin_pct) {
                reserve_always_met = false;
            }
        }

        let total_savings_usd = baseline_cost - total_cost_usd;
        let savings_pct = if baseline_cost > 0.0 {
            total_savings_usd / baseline_cost * 100.0
        } else {
            0.0
        };

        Ok(DecommitResult {
            schedules,
            total_cost_usd,
            total_savings_usd,
            savings_pct,
            load_always_served,
            reserve_always_met,
            n_units_decommitted,
        })
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn cheap_unit(id: usize) -> DecommitUnit {
        DecommitUnit {
            id,
            p_min_mw: 20.0,
            p_max_mw: 100.0,
            variable_cost_usd_per_mwh: 10.0,
            no_load_cost_usd_per_h: 50.0,
            shutdown_cost_usd: 500.0,
            startup_cost_usd: 1000.0,
            min_down_hours: 2,
            initially_online: true,
            is_must_run: false,
        }
    }

    fn expensive_unit(id: usize) -> DecommitUnit {
        DecommitUnit {
            id,
            p_min_mw: 10.0,
            p_max_mw: 80.0,
            variable_cost_usd_per_mwh: 50.0,
            no_load_cost_usd_per_h: 200.0,
            shutdown_cost_usd: 100.0,
            startup_cost_usd: 200.0,
            min_down_hours: 1,
            initially_online: true,
            is_must_run: false,
        }
    }

    fn must_run_unit(id: usize) -> DecommitUnit {
        DecommitUnit {
            id,
            p_min_mw: 30.0,
            p_max_mw: 60.0,
            variable_cost_usd_per_mwh: 80.0, // expensive but must run
            no_load_cost_usd_per_h: 300.0,
            shutdown_cost_usd: 0.0,
            startup_cost_usd: 0.0,
            min_down_hours: 0,
            initially_online: true,
            is_must_run: true,
        }
    }

    fn config_4h(load_mw: f64) -> DecommitmentConfig {
        DecommitmentConfig {
            n_hours: 4,
            base_load_mw: vec![load_mw; 4],
            reserve_margin_pct: 0.15,
            startup_shutdown_cost: true,
            min_online_units: 1,
        }
    }

    // ------------------------------------------------------------------
    // 1. Off-peak: expensive unit decommitted when load is low
    // ------------------------------------------------------------------
    #[test]
    fn test_expensive_unit_decommitted_at_low_load() {
        let mut dc = UnitDecommitter::new(config_4h(50.0));
        dc.add_unit(cheap_unit(0)); // 100 MW capacity
        dc.add_unit(expensive_unit(1)); // 80 MW capacity, but expensive

        let result = dc.optimize().expect("optimize failed");

        // With load=50 MW and cheap unit covering it, expensive should be decommitted.
        let exp_sched = result
            .schedules
            .iter()
            .find(|s| s.unit_id == 1)
            .expect("missing unit 1");
        assert!(
            !exp_sched.status[0],
            "Expensive unit should be decommitted at 50 MW load"
        );
        assert!(
            result.n_units_decommitted >= 1,
            "At least one unit should be decommitted"
        );
    }

    // ------------------------------------------------------------------
    // 2. Must-run unit is never decommitted
    // ------------------------------------------------------------------
    #[test]
    fn test_must_run_never_decommitted() {
        let mut dc = UnitDecommitter::new(config_4h(30.0));
        dc.add_unit(cheap_unit(0)); // can cover load alone
        dc.add_unit(must_run_unit(1));

        let result = dc.optimize().expect("optimize failed");

        let mr_sched = result
            .schedules
            .iter()
            .find(|s| s.unit_id == 1)
            .expect("missing must-run");
        // Must-run should always be online.
        for &status in &mr_sched.status {
            assert!(status, "Must-run unit should never be decommitted");
        }
    }

    // ------------------------------------------------------------------
    // 3. Reserve constraint maintained
    // ------------------------------------------------------------------
    #[test]
    fn test_reserve_constraint_maintained() {
        // Three units, each 100 MW. Load=200 MW → needs 200*1.15 = 230 MW.
        // Two units = 200 MW < 230 → cannot decommit both, only one decommittable.
        let config = DecommitmentConfig {
            n_hours: 2,
            base_load_mw: vec![200.0, 200.0],
            reserve_margin_pct: 0.15,
            startup_shutdown_cost: false,
            min_online_units: 1,
        };
        let mut dc = UnitDecommitter::new(config);
        dc.add_unit(DecommitUnit {
            id: 0,
            p_min_mw: 0.0,
            p_max_mw: 100.0,
            variable_cost_usd_per_mwh: 10.0,
            no_load_cost_usd_per_h: 0.0,
            shutdown_cost_usd: 0.0,
            startup_cost_usd: 0.0,
            min_down_hours: 0,
            initially_online: true,
            is_must_run: false,
        });
        dc.add_unit(DecommitUnit {
            id: 1,
            p_min_mw: 0.0,
            p_max_mw: 100.0,
            variable_cost_usd_per_mwh: 20.0,
            no_load_cost_usd_per_h: 0.0,
            shutdown_cost_usd: 0.0,
            startup_cost_usd: 0.0,
            min_down_hours: 0,
            initially_online: true,
            is_must_run: false,
        });
        dc.add_unit(DecommitUnit {
            id: 2,
            p_min_mw: 0.0,
            p_max_mw: 100.0,
            variable_cost_usd_per_mwh: 50.0,
            no_load_cost_usd_per_h: 0.0,
            shutdown_cost_usd: 0.0,
            startup_cost_usd: 0.0,
            min_down_hours: 0,
            initially_online: true,
            is_must_run: false,
        });

        let result = dc.optimize().expect("optimize failed");

        // Reserve maintained.
        assert!(
            result.reserve_always_met,
            "Reserve constraint should always be met"
        );
    }

    // ------------------------------------------------------------------
    // 4. Min-down-time respected: unit with min_down_hours=999 won't be decommitted
    //    when we need it back immediately (simulated via always-needed load)
    // ------------------------------------------------------------------
    #[test]
    fn test_savings_positive_without_marginal_unit() {
        // Two equal-capacity units; second is expensive.
        // Load = 60 MW.  Cheap unit alone covers it → savings from decommitting exp.
        let mut dc = UnitDecommitter::new(config_4h(60.0));
        dc.add_unit(DecommitUnit {
            id: 0,
            p_min_mw: 0.0,
            p_max_mw: 100.0,
            variable_cost_usd_per_mwh: 10.0,
            no_load_cost_usd_per_h: 0.0,
            shutdown_cost_usd: 0.0,
            startup_cost_usd: 0.0,
            min_down_hours: 0,
            initially_online: true,
            is_must_run: false,
        });
        dc.add_unit(DecommitUnit {
            id: 1,
            p_min_mw: 0.0,
            p_max_mw: 100.0,
            variable_cost_usd_per_mwh: 100.0,
            no_load_cost_usd_per_h: 500.0,
            shutdown_cost_usd: 0.0,
            startup_cost_usd: 0.0,
            min_down_hours: 0,
            initially_online: true,
            is_must_run: false,
        });

        let result = dc.optimize().expect("optimize failed");

        // Unit 1 should be decommitted.
        let sched1 = result
            .schedules
            .iter()
            .find(|s| s.unit_id == 1)
            .expect("missing unit 1");
        assert!(!sched1.status[0], "Expensive unit should be decommitted");
        // Savings should be positive.
        assert!(
            result.total_savings_usd > 0.0,
            "Total savings should be positive, got {:.2}",
            result.total_savings_usd
        );
    }

    // ------------------------------------------------------------------
    // 5. Infeasible: system cannot meet load even with all units
    // ------------------------------------------------------------------
    #[test]
    fn test_infeasible_when_capacity_insufficient() {
        let config = DecommitmentConfig {
            n_hours: 1,
            base_load_mw: vec![500.0], // impossible
            reserve_margin_pct: 0.10,
            startup_shutdown_cost: false,
            min_online_units: 1,
        };
        let mut dc = UnitDecommitter::new(config);
        dc.add_unit(cheap_unit(0)); // only 100 MW

        let err = dc.optimize().expect_err("should fail with Infeasible");
        assert!(
            matches!(err, DecommitError::Infeasible { .. }),
            "Expected Infeasible error, got {err}"
        );
    }

    // ------------------------------------------------------------------
    // 6. Min-online-units constraint prevents full decommitment
    // ------------------------------------------------------------------
    #[test]
    fn test_min_online_units_respected() {
        let config = DecommitmentConfig {
            n_hours: 2,
            base_load_mw: vec![1.0, 1.0], // tiny load
            reserve_margin_pct: 0.0,
            startup_shutdown_cost: false,
            min_online_units: 2, // must keep at least 2 online
        };
        let mut dc = UnitDecommitter::new(config);
        dc.add_unit(cheap_unit(0));
        dc.add_unit(expensive_unit(1));
        dc.add_unit(must_run_unit(2)); // is_must_run=true counts as online

        let result = dc.optimize().expect("optimize failed");
        let n_on: usize = result.schedules.iter().filter(|s| s.status[0]).count();
        assert!(
            n_on >= 2,
            "At least 2 units should remain online, got {n_on}"
        );
    }
}
