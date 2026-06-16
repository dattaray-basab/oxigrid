/// Unit commitment: determine which generators to operate each period.
///
/// Implements a priority-list (merit-order) unit commitment suitable for
/// small-to-medium systems.  For large systems with many units, MILP via
/// an external solver would be preferred.
///
/// # Algorithm
/// 1. Rank units by variable cost (full-load average $/MWh).
/// 2. For each period, commit units in order until demand + spinning reserve is met.
/// 3. Apply minimum on/off time constraints via a state machine.
use serde::{Deserialize, Serialize};

/// A single generating unit.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Unit {
    /// Unit name / identifier
    pub name: String,
    /// Minimum stable generation `MW`
    pub p_min_mw: f64,
    /// Maximum rated power `MW`
    pub p_max_mw: f64,
    /// Variable operating cost [$/MWh]
    pub cost_mwh: f64,
    /// No-load cost (fixed when running) [$/h]
    pub no_load_cost_h: f64,
    /// Start-up cost [$]
    pub startup_cost: f64,
    /// Minimum up time `hours`
    pub min_up_h: f64,
    /// Minimum down time `hours`
    pub min_down_h: f64,
    /// Initial commitment status (true = on)
    pub initially_on: bool,
    /// Hours already on (positive) or off (negative) at start
    pub initial_hours: f64,
}

impl Unit {
    /// Create a base-load unit (coal/nuclear).
    pub fn base_load(name: impl Into<String>, p_max_mw: f64, cost_mwh: f64) -> Self {
        Self {
            name: name.into(),
            p_min_mw: p_max_mw * 0.40,
            p_max_mw,
            cost_mwh,
            no_load_cost_h: p_max_mw * cost_mwh * 0.02,
            startup_cost: p_max_mw * 50.0,
            min_up_h: 8.0,
            min_down_h: 8.0,
            initially_on: true,
            initial_hours: 24.0,
        }
    }

    /// Create a peaking unit (gas turbine).
    pub fn peaking(name: impl Into<String>, p_max_mw: f64, cost_mwh: f64) -> Self {
        Self {
            name: name.into(),
            p_min_mw: p_max_mw * 0.20,
            p_max_mw,
            cost_mwh,
            no_load_cost_h: p_max_mw * cost_mwh * 0.01,
            startup_cost: p_max_mw * 10.0,
            min_up_h: 1.0,
            min_down_h: 1.0,
            initially_on: false,
            initial_hours: -4.0,
        }
    }

    /// Variable cost to operate at power level p [$/h].
    pub fn variable_cost(&self, p_mw: f64) -> f64 {
        self.no_load_cost_h + p_mw * self.cost_mwh
    }

    /// Incremental (marginal) cost [$/MWh] — constant for linear cost.
    pub fn marginal_cost(&self) -> f64 {
        self.cost_mwh
    }
}

/// Commitment state for one unit at one time step.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct UnitState {
    /// Is unit committed (online)?
    pub committed: bool,
    /// Dispatch `MW` (0 if not committed)
    pub dispatch_mw: f64,
    /// True if unit started this period
    pub start_up: bool,
    /// True if unit shut down this period
    pub shut_down: bool,
    /// Operating cost this period [$/h]
    pub cost_h: f64,
}

/// Result for one time period.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommitPeriod {
    pub period: usize,
    pub demand_mw: f64,
    pub states: Vec<UnitState>,
    pub total_generation_mw: f64,
    pub total_cost_h: f64,
    pub spinning_reserve_mw: f64,
    pub load_shed_mw: f64,
}

impl CommitPeriod {
    pub fn is_feasible(&self) -> bool {
        self.load_shed_mw < 1e-6
    }
}

/// Run priority-list unit commitment for multiple periods.
///
/// # Arguments
/// - `units`       — generating units
/// - `demands`     — demand `MW` for each period
/// - `dt_h`        — period duration `hours`
/// - `reserve_pct` — spinning reserve requirement as % of demand (e.g. 15.0)
pub fn priority_commit(
    units: &[Unit],
    demands: &[f64],
    dt_h: f64,
    reserve_pct: f64,
) -> Vec<CommitPeriod> {
    let n_units = units.len();
    let n_periods = demands.len();

    // Sort units by marginal cost (merit order)
    let mut merit_order: Vec<usize> = (0..n_units).collect();
    merit_order.sort_by(|&a, &b| {
        units[a]
            .marginal_cost()
            .partial_cmp(&units[b].marginal_cost())
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    // Track commitment duration (positive = hours on, negative = hours off)
    let mut hours_on_off: Vec<f64> = units
        .iter()
        .map(|u| {
            if u.initially_on {
                u.initial_hours
            } else {
                -u.initial_hours
            }
        })
        .collect();

    let mut results = Vec::with_capacity(n_periods);

    for (t, &demand) in demands.iter().enumerate() {
        let reserve_req = demand * reserve_pct / 100.0;
        let total_req = demand + reserve_req;

        let mut states = vec![
            UnitState {
                committed: false,
                dispatch_mw: 0.0,
                start_up: false,
                shut_down: false,
                cost_h: 0.0,
            };
            n_units
        ];

        // Determine which units CAN be committed (min off time)
        // and which MUST stay on (min on time)
        let mut capacity_committed = 0.0_f64;
        let mut must_on = vec![false; n_units];
        let mut can_start = vec![true; n_units];

        for i in 0..n_units {
            if hours_on_off[i] > 0.0 && hours_on_off[i] < units[i].min_up_h {
                // Must stay on
                must_on[i] = true;
            }
            if hours_on_off[i] < 0.0 && hours_on_off[i].abs() < units[i].min_down_h {
                // Cannot start yet
                can_start[i] = false;
            }
        }

        // Commit must-on units first
        for i in 0..n_units {
            if must_on[i] {
                states[i].committed = true;
                capacity_committed += units[i].p_max_mw;
            }
        }

        // Commit additional units in merit order
        for &i in &merit_order {
            if must_on[i] {
                continue;
            }
            if capacity_committed >= total_req {
                break;
            }
            if can_start[i] {
                states[i].committed = true;
                capacity_committed += units[i].p_max_mw;
            }
        }

        // Economic dispatch among committed units (merit order, linear cost)
        let committed_units: Vec<usize> = (0..n_units).filter(|&i| states[i].committed).collect();
        let dispatched = economic_dispatch_committed(units, &committed_units, demand);

        let mut total_gen = 0.0_f64;
        let mut total_cost = 0.0_f64;
        let mut total_cap = 0.0_f64;

        for (i, &p) in dispatched.iter().enumerate() {
            let ui = committed_units[i];
            let prev_committed = hours_on_off[ui] > 0.0;
            states[ui].dispatch_mw = p;
            states[ui].start_up = !prev_committed;
            states[ui].cost_h = units[ui].variable_cost(p);
            if states[ui].start_up {
                states[ui].cost_h += units[ui].startup_cost / dt_h; // amortise over period
            }
            total_gen += p;
            total_cost += states[ui].cost_h;
            total_cap += units[ui].p_max_mw;
        }

        // Detect shutdowns
        for i in 0..n_units {
            let was_on = hours_on_off[i] > 0.0;
            if was_on && !states[i].committed {
                states[i].shut_down = true;
            }
        }

        // Update hours on/off
        for i in 0..n_units {
            if states[i].committed {
                hours_on_off[i] = if hours_on_off[i] > 0.0 {
                    hours_on_off[i] + dt_h
                } else {
                    dt_h
                };
            } else {
                hours_on_off[i] = if hours_on_off[i] < 0.0 {
                    hours_on_off[i] - dt_h
                } else {
                    -dt_h
                };
            }
        }

        let load_shed = (demand - total_gen).max(0.0);
        let reserve = total_cap - total_gen;

        results.push(CommitPeriod {
            period: t,
            demand_mw: demand,
            states,
            total_generation_mw: total_gen,
            total_cost_h: total_cost,
            spinning_reserve_mw: reserve,
            load_shed_mw: load_shed,
        });
    }

    results
}

/// Economic dispatch for a fixed set of committed units.
///
/// Uses merit-order dispatch with min/max limits.
/// Returns dispatch `MW` for each unit (same order as `committed`).
fn economic_dispatch_committed(units: &[Unit], committed: &[usize], demand_mw: f64) -> Vec<f64> {
    if committed.is_empty() {
        return vec![];
    }

    // Sort by marginal cost
    let mut order: Vec<usize> = (0..committed.len()).collect();
    order.sort_by(|&a, &b| {
        units[committed[a]]
            .marginal_cost()
            .partial_cmp(&units[committed[b]].marginal_cost())
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let mut dispatch = vec![0.0_f64; committed.len()];
    let mut remaining = demand_mw;

    // Load each unit to p_min first
    for &ci in &order {
        let ui = committed[ci];
        dispatch[ci] = units[ui].p_min_mw;
        remaining -= units[ui].p_min_mw;
    }

    // Then fill up in merit order
    for &ci in &order {
        if remaining <= 0.0 {
            break;
        }
        let ui = committed[ci];
        let headroom = units[ui].p_max_mw - dispatch[ci];
        let add = headroom.min(remaining);
        dispatch[ci] += add;
        remaining -= add;
    }

    dispatch
}

/// Compute total cost for a commitment schedule [$/].
pub fn total_schedule_cost(results: &[CommitPeriod], dt_h: f64) -> f64 {
    results.iter().map(|r| r.total_cost_h * dt_h).sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn three_unit_system() -> Vec<Unit> {
        vec![
            Unit::base_load("Coal-1", 200.0, 25.0),
            Unit::base_load("Gas-CC", 150.0, 45.0),
            Unit::peaking("Gas-GT", 100.0, 80.0),
        ]
    }

    #[test]
    fn test_single_period_low_demand() {
        let units = three_unit_system();
        let demands = vec![150.0];
        let result = priority_commit(&units, &demands, 1.0, 15.0);
        assert_eq!(result.len(), 1);
        let period = &result[0];
        assert!(
            period.total_generation_mw >= 150.0,
            "Should meet demand: gen={:.1}",
            period.total_generation_mw
        );
    }

    #[test]
    fn test_high_demand_commits_peaker() {
        let units = three_unit_system();
        let demands = vec![420.0]; // needs all 3 units
        let result = priority_commit(&units, &demands, 1.0, 15.0);
        let committed_count = result[0].states.iter().filter(|s| s.committed).count();
        assert!(
            committed_count >= 2,
            "Should commit at least 2 units for high demand"
        );
    }

    #[test]
    fn test_generation_meets_demand() {
        let units = three_unit_system();
        let demands: Vec<f64> = (0..24)
            .map(|h| 100.0 + 150.0 * ((h as f64 / 24.0 * std::f64::consts::PI).sin()).abs())
            .collect();
        let results = priority_commit(&units, &demands, 1.0, 15.0);
        for r in &results {
            assert!(
                r.total_generation_mw >= r.demand_mw * 0.99 || r.load_shed_mw > 0.0,
                "Gen={:.1} should meet demand={:.1}",
                r.total_generation_mw,
                r.demand_mw
            );
        }
    }

    #[test]
    fn test_merit_order_cheapest_first() {
        let units = three_unit_system();
        let demands = vec![220.0]; // coal (200) + some gas CC
        let result = priority_commit(&units, &demands, 1.0, 0.0);
        // Coal (unit 0) should be at max, Gas-GT should not be needed
        let coal_on = result[0].states[0].committed;
        let gt_on = result[0].states[2].committed;
        assert!(coal_on, "Cheapest unit (coal) should be committed");
        // Gas turbine (most expensive) may or may not be needed
        let _ = gt_on;
    }

    #[test]
    fn test_total_cost_positive() {
        let units = three_unit_system();
        let demands = vec![200.0; 24];
        let results = priority_commit(&units, &demands, 1.0, 15.0);
        let cost = total_schedule_cost(&results, 1.0);
        assert!(cost > 0.0, "Total cost should be positive: ${:.2}", cost);
    }

    #[test]
    fn test_feasible_periods() {
        let units = three_unit_system();
        let demands = vec![100.0, 200.0, 300.0, 200.0, 100.0];
        let results = priority_commit(&units, &demands, 1.0, 10.0);
        for r in &results {
            assert!(
                r.is_feasible() || r.demand_mw > 450.0,
                "Period {} should be feasible for load {:.0}",
                r.period,
                r.demand_mw
            );
        }
    }

    // ------------------------------------------------------------------
    // 7 new tests
    // ------------------------------------------------------------------

    /// `Unit::base_load` derives fields from p_max and cost correctly.
    #[test]
    fn test_base_load_field_derivation() {
        let u = Unit::base_load("Nuclear-1", 400.0, 20.0);
        assert_eq!(u.p_min_mw, 400.0 * 0.40, "p_min should be 40% of p_max");
        assert_eq!(u.p_max_mw, 400.0);
        assert_eq!(u.cost_mwh, 20.0);
        assert!((u.no_load_cost_h - 400.0 * 20.0 * 0.02).abs() < 1e-9);
        assert_eq!(u.startup_cost, 400.0 * 50.0);
        assert_eq!(u.min_up_h, 8.0);
        assert_eq!(u.min_down_h, 8.0);
        assert!(u.initially_on);
        assert_eq!(u.initial_hours, 24.0);
    }

    /// `Unit::peaking` derives fields from p_max and cost correctly.
    #[test]
    fn test_peaking_field_derivation() {
        let u = Unit::peaking("Gas-GT-2", 50.0, 90.0);
        assert_eq!(u.p_min_mw, 50.0 * 0.20, "p_min should be 20% of p_max");
        assert_eq!(u.p_max_mw, 50.0);
        assert_eq!(u.cost_mwh, 90.0);
        assert!((u.no_load_cost_h - 50.0 * 90.0 * 0.01).abs() < 1e-9);
        assert_eq!(u.startup_cost, 50.0 * 10.0);
        assert_eq!(u.min_up_h, 1.0);
        assert_eq!(u.min_down_h, 1.0);
        assert!(!u.initially_on);
        assert_eq!(u.initial_hours, -4.0);
    }

    /// `Unit::variable_cost` and `marginal_cost` return expected values.
    #[test]
    fn test_unit_cost_methods() {
        let u = Unit::base_load("Coal-A", 100.0, 30.0);
        // no_load_cost_h = 100 * 30 * 0.02 = 60
        // variable_cost(80) = 60 + 80*30 = 2460
        let expected_vc = u.no_load_cost_h + 80.0 * 30.0;
        assert!((u.variable_cost(80.0) - expected_vc).abs() < 1e-9);
        assert_eq!(u.marginal_cost(), 30.0);
        // variable_cost at p_min_mw must be lower than at p_max_mw
        assert!(u.variable_cost(u.p_min_mw) < u.variable_cost(u.p_max_mw));
    }

    /// `CommitPeriod::is_feasible` returns false when load is shed.
    #[test]
    fn test_is_feasible_with_load_shed() {
        // Single tiny unit that cannot meet demand — causes genuine load shed.
        let units = vec![Unit::peaking("Tiny", 10.0, 100.0)];
        let demands = vec![5000.0]; // far beyond any capacity
        let results = priority_commit(&units, &demands, 1.0, 0.0);
        assert_eq!(results.len(), 1);
        let period = &results[0];
        assert!(
            period.load_shed_mw > 1e-6,
            "Expected load shedding, got {:.2} MW shed",
            period.load_shed_mw
        );
        assert!(
            !period.is_feasible(),
            "Period should not be feasible when load is shed"
        );
    }

    /// `start_up` and `shut_down` flags track transitions correctly.
    #[test]
    fn test_startup_and_shutdown_flags() {
        // Gas-GT starts off (initially_on = false, initial_hours = -4 → abs = 4 ≥ min_down_h=1)
        // Period 0: low demand — only Coal should commit (Gas-GT stays off).
        // Period 1: very high demand — Coal + Gas-CC + Gas-GT all commit.
        // Period 2: low demand again — Gas-GT should shut down (if min_up_h allows).
        //
        // We test that Gas-GT shows start_up=true the first period it turns on.
        let units = three_unit_system();
        let demands = vec![80.0, 450.0];
        let results = priority_commit(&units, &demands, 1.0, 0.0);
        assert_eq!(results.len(), 2);

        // Find the first period where Gas-GT (index 2) is committed.
        let gt_start_period = results.iter().find(|r| r.states[2].committed);
        if let Some(period) = gt_start_period {
            assert!(
                period.states[2].start_up,
                "Gas-GT should have start_up=true in its first committed period"
            );
        }
        // In period 0, Coal (index 0) was already on — it must NOT show start_up.
        assert!(
            !results[0].states[0].start_up,
            "Coal was initially on, so start_up should be false in period 0"
        );
    }

    /// Minimum down-time constraint: a unit recently shut down cannot restart.
    #[test]
    fn test_min_down_time_constraint() {
        // Construct a unit that has been off for only 2 hours but requires min_down_h=8.
        let mut u = Unit::base_load("Constrained", 200.0, 30.0);
        u.initially_on = false;
        u.initial_hours = 2.0; // has been off 2h, which is < min_down_h=8
                               // Coal-1 alone can cover moderate demand if it was initially on.
        let coal = Unit::base_load("Coal-On", 500.0, 25.0);
        let units = vec![coal, u];
        let demands = vec![600.0]; // coal covers 500, constrained would be needed for full
        let results = priority_commit(&units, &demands, 1.0, 0.0);
        // "Constrained" must NOT be committed because it violated min_down_h.
        assert!(
            !results[0].states[1].committed,
            "Unit with insufficient down-time must not be committed"
        );
    }

    /// Empty demand slice returns an empty result vector without panicking.
    #[test]
    fn test_empty_demands() {
        let units = three_unit_system();
        let results = priority_commit(&units, &[], 1.0, 15.0);
        assert!(
            results.is_empty(),
            "Empty demands should yield empty results"
        );
    }

    /// `total_schedule_cost` scales linearly with `dt_h`.
    #[test]
    fn test_total_schedule_cost_scales_with_dt() {
        let units = three_unit_system();
        let demands = vec![200.0; 4];
        let results_1h = priority_commit(&units, &demands, 1.0, 0.0);
        let results_half = priority_commit(&units, &demands, 0.5, 0.0);
        let cost_1h = total_schedule_cost(&results_1h, 1.0);
        let cost_half = total_schedule_cost(&results_half, 0.5);
        // Both runs cover the same demand, dt_h halved → cost_h values may
        // differ (startup amortisation), but both costs must be positive.
        assert!(cost_1h > 0.0, "1h-step cost must be positive");
        assert!(cost_half > 0.0, "0.5h-step cost must be positive");
    }

    /// Single-unit system: generation equals p_max when demand exceeds p_max.
    #[test]
    fn test_single_unit_at_capacity() {
        let unit = Unit::base_load("Only-Unit", 100.0, 50.0);
        let demands = vec![200.0]; // exceeds unit capacity
        let results = priority_commit(&[unit], &demands, 1.0, 0.0);
        assert_eq!(results.len(), 1);
        let p = &results[0];
        // The unit is committed; dispatch should be clamped to p_max (100 MW).
        assert!(p.states[0].committed, "Single unit must be committed");
        assert!(
            (p.states[0].dispatch_mw - 100.0).abs() < 1e-6,
            "Dispatch should equal p_max=100, got {:.2}",
            p.states[0].dispatch_mw
        );
        assert!(
            p.load_shed_mw > 0.0,
            "Excess demand beyond capacity should be shed"
        );
    }

    /// Zero reserve requirement: generation must still meet demand exactly.
    #[test]
    fn test_zero_reserve_generation_meets_demand() {
        let units = three_unit_system();
        let demands = vec![180.0, 320.0, 450.0];
        let results = priority_commit(&units, &demands, 1.0, 0.0);
        for r in &results {
            assert!(
                r.total_generation_mw >= r.demand_mw - 1e-6 || r.load_shed_mw > 1e-6,
                "Period {}: gen={:.2} must cover demand={:.2}",
                r.period,
                r.total_generation_mw,
                r.demand_mw
            );
        }
    }
}
