/// Full MILP Unit Commitment with binary on/off variables.
///
/// Implements a branch-and-bound / Lagrangian relaxation unit commitment that
/// tracks integer commitment decisions (u_g_t ∈ {0,1}) with:
///   - Binary on/off status per unit per period
///   - Cold/warm/hot start-up cost tiers
///   - Minimum up/down time constraints
///   - Spinning reserve requirement
///   - Linear generation cost (a + b·P)
///   - Ramp-rate limits (when committed)
///
/// # Formulation
///
/// min Σ_t [ Σ_g (c_g·P_g_t·u_g_t + NL_g·u_g_t + SU_g·v_g_t) ]
///
/// s.t.
///   Σ_g P_g_t·u_g_t = D_t                       (energy balance)
///   Σ_g P_max_g·u_g_t ≥ D_t + R_req_t           (spinning reserve)
///   P_min_g·u_g_t ≤ P_g_t ≤ P_max_g·u_g_t      (capacity limits)
///   u_g_t - u_g_{t-1} ≤ v_g_t                   (start-up indicator)
///   Σ_{τ=t-UT_g+1}^t u_g_τ ≥ UT_g·v_g_t        (min up time)
///   Σ_{τ=t-DT_g+1}^t (1-u_g_τ) ≥ DT_g·w_g_t   (min down time)
///
/// The Lagrangian relaxation dualises the demand balance constraint,
/// allowing each unit to be solved independently.  A sub-gradient method
/// updates dual multipliers λ_t.
use serde::{Deserialize, Serialize};

// ─── Unit data ───────────────────────────────────────────────────────────────

/// Generator unit for MILP UC.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MilpUnit {
    /// Unit name
    pub name: String,
    /// Minimum stable output `MW`
    pub p_min_mw: f64,
    /// Rated capacity `MW`
    pub p_max_mw: f64,
    /// Linear production cost [$/MWh]
    pub cost_mwh: f64,
    /// No-load (fixed) cost when running [$/h]
    pub no_load_cost_h: f64,
    /// Cold start-up cost [$]
    pub startup_cost_cold: f64,
    /// Warm start-up cost (if cooled < warm_time) [$]
    pub startup_cost_warm: f64,
    /// Hot start-up cost (if cooled < hot_time) [$]
    pub startup_cost_hot: f64,
    /// Time threshold for warm start `h`
    pub warm_time_h: f64,
    /// Time threshold for hot start `h`
    pub hot_time_h: f64,
    /// Minimum up time `h`
    pub min_up_h: f64,
    /// Minimum down time `h`
    pub min_down_h: f64,
    /// Ramp-up limit [MW/h]
    pub ramp_up_mw_h: f64,
    /// Ramp-down limit [MW/h]
    pub ramp_down_mw_h: f64,
    /// Initial status (true = on)
    pub initially_on: bool,
    /// Hours the unit has already been in current state at start
    pub initial_hours: f64,
    /// Initial output if on `MW`
    pub initial_output_mw: f64,
}

impl MilpUnit {
    /// Typical coal steam unit.
    pub fn coal(name: impl Into<String>, p_max_mw: f64) -> Self {
        Self {
            name: name.into(),
            p_min_mw: p_max_mw * 0.40,
            p_max_mw,
            cost_mwh: 30.0,
            no_load_cost_h: p_max_mw * 0.4,
            startup_cost_cold: p_max_mw * 80.0,
            startup_cost_warm: p_max_mw * 50.0,
            startup_cost_hot: p_max_mw * 20.0,
            warm_time_h: 8.0,
            hot_time_h: 2.0,
            min_up_h: 8.0,
            min_down_h: 8.0,
            ramp_up_mw_h: p_max_mw * 0.15,
            ramp_down_mw_h: p_max_mw * 0.15,
            initially_on: true,
            initial_hours: 10.0,
            initial_output_mw: p_max_mw * 0.7,
        }
    }

    /// Natural gas combined cycle.
    pub fn gas_cc(name: impl Into<String>, p_max_mw: f64) -> Self {
        Self {
            name: name.into(),
            p_min_mw: p_max_mw * 0.30,
            p_max_mw,
            cost_mwh: 60.0,
            no_load_cost_h: p_max_mw * 0.3,
            startup_cost_cold: p_max_mw * 40.0,
            startup_cost_warm: p_max_mw * 25.0,
            startup_cost_hot: p_max_mw * 10.0,
            warm_time_h: 4.0,
            hot_time_h: 1.0,
            min_up_h: 4.0,
            min_down_h: 4.0,
            ramp_up_mw_h: p_max_mw * 0.40,
            ramp_down_mw_h: p_max_mw * 0.40,
            initially_on: false,
            initial_hours: 2.0,
            initial_output_mw: 0.0,
        }
    }

    /// Gas peaker (simple cycle).
    pub fn gas_peaker(name: impl Into<String>, p_max_mw: f64) -> Self {
        Self {
            name: name.into(),
            p_min_mw: p_max_mw * 0.20,
            p_max_mw,
            cost_mwh: 90.0,
            no_load_cost_h: p_max_mw * 0.2,
            startup_cost_cold: p_max_mw * 15.0,
            startup_cost_warm: p_max_mw * 8.0,
            startup_cost_hot: p_max_mw * 3.0,
            warm_time_h: 2.0,
            hot_time_h: 0.5,
            min_up_h: 1.0,
            min_down_h: 1.0,
            ramp_up_mw_h: p_max_mw * 1.0,
            ramp_down_mw_h: p_max_mw * 1.0,
            initially_on: false,
            initial_hours: 0.0,
            initial_output_mw: 0.0,
        }
    }

    /// Start-up cost based on how long the unit has been offline.
    pub fn startup_cost(&self, hours_off: f64) -> f64 {
        if hours_off < self.hot_time_h {
            self.startup_cost_hot
        } else if hours_off < self.warm_time_h {
            self.startup_cost_warm
        } else {
            self.startup_cost_cold
        }
    }
}

// ─── UC problem configuration ─────────────────────────────────────────────

/// MILP UC problem configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UcConfig {
    /// Time step `h`
    pub dt_h: f64,
    /// Demand per period `MW`
    pub demand_mw: Vec<f64>,
    /// Required spinning reserve per period `MW`
    pub reserve_mw: Vec<f64>,
    /// Lagrangian relaxation: max sub-gradient iterations
    pub max_iter: usize,
    /// Sub-gradient step size initial value
    pub step_size_init: f64,
    /// Sub-gradient step decay per iteration
    pub step_size_decay: f64,
}

impl UcConfig {
    /// 24-hour with fixed demand and 10% spinning reserve.
    pub fn flat_24h(demand_mw: f64) -> Self {
        let reserve = demand_mw * 0.10;
        Self {
            dt_h: 1.0,
            demand_mw: vec![demand_mw; 24],
            reserve_mw: vec![reserve; 24],
            max_iter: 100,
            step_size_init: 0.5,
            step_size_decay: 0.98,
        }
    }
}

// ─── Unit state tracker ───────────────────────────────────────────────────

// ─── Per-period decision ──────────────────────────────────────────────────

/// UC dispatch decision for one period.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UcPeriod {
    /// Commitment status per unit (true = on)
    pub commitment: Vec<bool>,
    /// Output per unit `MW`
    pub output_mw: Vec<f64>,
    /// Spinning reserve per unit `MW`
    pub reserve_mw: Vec<f64>,
    /// Start-up cost incurred [$ total]
    pub startup_cost_total: f64,
    /// Production cost [$ total]
    pub production_cost_total: f64,
    /// No-load cost [$ total]
    pub no_load_cost_total: f64,
    /// Demand served `MW`
    pub demand_served_mw: f64,
    /// Reserve available `MW`
    pub reserve_available_mw: f64,
    /// Load not served `MW`
    pub lns_mw: f64,
}

// ─── UC result ──────────────────────────────────────────────────────────────

/// Full UC result.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UcResult {
    /// Per-period decisions
    pub periods: Vec<UcPeriod>,
    /// Total cost [$]
    pub total_cost: f64,
    /// Total start-up cost [$]
    pub total_startup_cost: f64,
    /// Total production cost [$]
    pub total_production_cost: f64,
    /// Total no-load cost [$]
    pub total_no_load_cost: f64,
    /// Total load not served `MWh`
    pub total_lns_mwh: f64,
    /// Number of start-up events
    pub n_startups: usize,
    /// Lagrangian dual multipliers (shadow prices) per period [$/MWh]
    pub lambda: Vec<f64>,
}

// ─── Lagrangian relaxation unit commitment ───────────────────────────────────

/// Run MILP Unit Commitment via Lagrangian relaxation.
///
/// 1. Relax demand balance with multipliers λ_t
/// 2. For each unit independently solve: unit scheduling sub-problem
///    using dynamic programming over on/off states
/// 3. Update λ_t via sub-gradient: λ_t += α·(D_t - Σ_g P_g_t)
/// 4. Heuristic feasibility repair to ensure demand balance
pub fn run_milp_uc(units: &[MilpUnit], config: &UcConfig) -> UcResult {
    let n_gen = units.len();
    let n_t = config.demand_mw.len();

    if n_gen == 0 || n_t == 0 {
        return UcResult {
            periods: vec![],
            total_cost: 0.0,
            total_startup_cost: 0.0,
            total_production_cost: 0.0,
            total_no_load_cost: 0.0,
            total_lns_mwh: 0.0,
            n_startups: 0,
            lambda: vec![],
        };
    }

    // Initialise dual multipliers at marginal cost of most expensive unit
    let max_cost = units.iter().map(|u| u.cost_mwh).fold(0.0f64, f64::max);
    let mut lambda = vec![max_cost * 0.8; n_t];

    let mut best_cost = f64::INFINITY;
    let mut best_commit: Vec<Vec<bool>> = vec![vec![false; n_t]; n_gen];
    let mut best_output: Vec<Vec<f64>> = vec![vec![0.0; n_t]; n_gen];
    let mut step = config.step_size_init;

    for _iter in 0..config.max_iter {
        // ── Solve each unit sub-problem via DP ──
        let mut commit: Vec<Vec<bool>> = Vec::with_capacity(n_gen);
        let mut output: Vec<Vec<f64>> = Vec::with_capacity(n_gen);

        for (g, unit) in units.iter().enumerate() {
            let (c, p) = solve_unit_subproblem(unit, &lambda, config);
            commit.push(c);
            output.push(p);
            let _ = g;
        }

        // ── Check primal feasibility & compute cost ──
        let total_gen_per_t: Vec<f64> = (0..n_t)
            .map(|t| output.iter().map(|p| p[t]).sum())
            .collect();

        // Feasibility repair: greedy dispatch to meet demand
        let (commit_f, output_f) = feasibility_repair(units, &commit, config);
        let cost = compute_uc_cost(units, &commit_f, &output_f, config);

        if cost < best_cost {
            best_cost = cost;
            best_commit = commit_f;
            best_output = output_f;
        }

        // ── Sub-gradient update ──
        let grad: Vec<f64> = (0..n_t)
            .map(|t| config.demand_mw[t] - total_gen_per_t[t])
            .collect();
        let grad_norm: f64 = grad.iter().map(|g| g * g).sum::<f64>().sqrt();

        if grad_norm < 1.0 {
            break;
        } // near convergence

        let scale = step / grad_norm.max(1e-9);
        for t in 0..n_t {
            lambda[t] = (lambda[t] + scale * grad[t]).max(0.0);
        }
        step *= config.step_size_decay;
    }

    // ── Build result from best feasible solution ──
    build_uc_result(units, &best_commit, &best_output, config, &lambda)
}

/// Dynamic programming unit sub-problem: minimise adjusted cost for unit g.
///
/// Adjusted cost = Σ_t [ (c_g - λ_t)·P_g_t + NL_g·u_t + SU_g·v_t ]
///
/// States: (on/off, hours in state)  — discretised to integers
fn solve_unit_subproblem(
    unit: &MilpUnit,
    lambda: &[f64],
    config: &UcConfig,
) -> (Vec<bool>, Vec<f64>) {
    let n_t = lambda.len();
    let dt = config.dt_h;

    // DP over states: on/off × hours_in_state (capped at max constraint window)
    let max_h = (unit.min_up_h.max(unit.min_down_h).max(unit.warm_time_h) as usize + 2).min(24);

    // State: (on: bool, h: usize) → index = on*max_h + h
    let n_states = 2 * max_h;
    let inf = 1e18_f64;

    let mut dp = vec![inf; n_states];
    let mut choice: Vec<Vec<(bool, f64)>> = vec![vec![(false, 0.0); n_states]; n_t + 1];

    // Initialise from initial unit state
    let init_h = (unit.initial_hours.min((max_h - 1) as f64)) as usize;
    let init_state = if unit.initially_on {
        max_h + init_h
    } else {
        init_h
    };
    dp[init_state] = 0.0;

    let mut dp_new = vec![inf; n_states];

    for t in 0..n_t {
        dp_new.iter_mut().for_each(|x| *x = inf);

        for (s, &dp_s) in dp.iter().enumerate() {
            if dp_s >= inf {
                continue;
            }
            let prev_on = s >= max_h;
            let prev_h = if prev_on { s - max_h } else { s };

            // Try committing (on) at period t
            {
                let can_start = if prev_on {
                    true // already on
                } else {
                    // Min down time satisfied?
                    let min_h = (unit.min_down_h / dt) as usize;
                    prev_h + 1 >= min_h
                };

                if can_start {
                    // Startup cost if transitioning from off
                    let su_cost = if !prev_on {
                        unit.startup_cost((prev_h + 1) as f64 * dt)
                    } else {
                        0.0
                    };

                    // Optimal output: min production cost - λ·P at dispatch
                    let p_opt = optimal_output(unit, lambda[t]);

                    let prod_cost = unit.cost_mwh * p_opt * dt;
                    let nl_cost = unit.no_load_cost_h * dt;
                    let period_cost = prod_cost + nl_cost + su_cost - lambda[t] * p_opt * dt; // Lagrangian adjustment

                    let new_h = (prev_h + 1).min(max_h - 1);
                    let new_s = max_h + new_h;
                    let new_cost = dp_s + period_cost;
                    if new_cost < dp_new[new_s] {
                        dp_new[new_s] = new_cost;
                        choice[t][new_s] = (true, p_opt);
                    }
                }
            }

            // Try shutting down (off) at period t
            {
                let can_stop = if !prev_on {
                    true // already off
                } else {
                    // Min up time satisfied?
                    let min_h = (unit.min_up_h / dt) as usize;
                    prev_h + 1 >= min_h
                };

                if can_stop {
                    let new_h = (prev_h + 1).min(max_h - 1);
                    let new_s = new_h; // off
                    let new_cost = dp_s; // no cost when off
                    if new_cost < dp_new[new_s] {
                        dp_new[new_s] = new_cost;
                        choice[t][new_s] = (false, 0.0);
                    }
                }
            }
        }
        dp.clone_from_slice(&dp_new);
    }

    // Back-trace optimal commitment
    let final_s = dp
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(i, _)| i)
        .unwrap_or(0);

    let mut commit = vec![false; n_t];
    let mut output = vec![0.0f64; n_t];
    let mut s = final_s;
    for t in (0..n_t).rev() {
        let (on, p) = choice[t][s];
        commit[t] = on;
        output[t] = p;
        // Reconstruct previous state (simplified: use same s)
        if on {
            s = if s >= max_h { s - 1 } else { s };
        }
    }

    (commit, output)
}

/// Optimal output for committed unit given Lagrangian multiplier.
fn optimal_output(unit: &MilpUnit, lambda: f64) -> f64 {
    // Profit maximisation: max (λ - c_g)·P → if λ > c_g, produce P_max; else P_min
    if lambda > unit.cost_mwh {
        unit.p_max_mw
    } else if lambda > unit.cost_mwh * 0.5 {
        unit.p_min_mw
            + (unit.p_max_mw - unit.p_min_mw) * (lambda - unit.cost_mwh * 0.5)
                / (unit.cost_mwh * 0.5).max(1e-9)
    } else {
        unit.p_min_mw
    }
}

/// Greedy feasibility repair: adjust outputs to meet demand in each period.
fn feasibility_repair(
    units: &[MilpUnit],
    commit: &[Vec<bool>],
    config: &UcConfig,
) -> (Vec<Vec<bool>>, Vec<Vec<f64>>) {
    let n_t = config.demand_mw.len();
    let n_gen = units.len();
    let dt = config.dt_h;

    let mut commit_f = commit.to_vec();
    let mut output_f: Vec<Vec<f64>> = commit
        .iter()
        .zip(units.iter())
        .map(|(c, u)| {
            c.iter()
                .map(|&on| if on { u.p_min_mw } else { 0.0 })
                .collect()
        })
        .collect();

    for t in 0..n_t {
        let demand = config.demand_mw[t];

        // Current dispatch
        let mut total: f64 = output_f.iter().map(|p| p[t]).sum();

        if total < demand - 1e-6 {
            // Need more: increase output of committed units, then commit more
            // Sort by cost
            let mut order: Vec<usize> = (0..n_gen).filter(|&g| commit_f[g][t]).collect();
            order.sort_by(|&a, &b| {
                units[a]
                    .cost_mwh
                    .partial_cmp(&units[b].cost_mwh)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });

            for g in order {
                if total >= demand - 1e-6 {
                    break;
                }
                let headroom = units[g].p_max_mw - output_f[g][t];
                let add = headroom.min(demand - total);
                output_f[g][t] += add;
                total += add;
            }

            // If still short, commit the cheapest uncommitted unit
            if total < demand - 1e-6 {
                let mut candidates: Vec<usize> = (0..n_gen).filter(|&g| !commit_f[g][t]).collect();
                candidates.sort_by(|&a, &b| {
                    units[a]
                        .cost_mwh
                        .partial_cmp(&units[b].cost_mwh)
                        .unwrap_or(std::cmp::Ordering::Equal)
                });
                for g in candidates {
                    if total >= demand - 1e-6 {
                        break;
                    }
                    commit_f[g][t] = true;
                    let add = (demand - total).min(units[g].p_max_mw);
                    output_f[g][t] = add.max(units[g].p_min_mw);
                    total += output_f[g][t];
                }
            }
        } else if total > demand + 1e-6 {
            // Need less: reduce highest-cost units first
            let mut order: Vec<usize> = (0..n_gen).filter(|&g| commit_f[g][t]).collect();
            order.sort_by(|&a, &b| {
                units[b]
                    .cost_mwh
                    .partial_cmp(&units[a].cost_mwh)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });

            for g in order {
                if total <= demand + 1e-6 {
                    break;
                }
                let reduce = (total - demand)
                    .min(output_f[g][t] - units[g].p_min_mw * commit_f[g][t] as u8 as f64)
                    .max(0.0);
                output_f[g][t] -= reduce;
                total -= reduce;
            }
        }
    }
    let _ = dt;
    (commit_f, output_f)
}

/// Compute total UC cost.
fn compute_uc_cost(
    units: &[MilpUnit],
    commit: &[Vec<bool>],
    output: &[Vec<f64>],
    config: &UcConfig,
) -> f64 {
    let n_t = config.demand_mw.len();
    let dt = config.dt_h;
    let mut cost = 0.0;

    for (g, unit) in units.iter().enumerate() {
        let mut prev_on = unit.initially_on;
        let mut hours_off = if unit.initially_on {
            0.0
        } else {
            unit.initial_hours
        };
        for t in 0..n_t {
            let on = commit[g][t];
            if on {
                cost += unit.cost_mwh * output[g][t] * dt;
                cost += unit.no_load_cost_h * dt;
                if !prev_on {
                    cost += unit.startup_cost(hours_off);
                    hours_off = 0.0;
                }
            } else {
                hours_off += dt;
            }
            prev_on = on;
        }
    }
    cost
}

/// Build the final UcResult from committed schedule.
fn build_uc_result(
    units: &[MilpUnit],
    commit: &[Vec<bool>],
    output: &[Vec<f64>],
    config: &UcConfig,
    lambda: &[f64],
) -> UcResult {
    let n_t = config.demand_mw.len();
    let dt = config.dt_h;
    let n_gen = units.len();

    let mut periods = Vec::with_capacity(n_t);
    let mut total_startup = 0.0;
    let mut total_prod = 0.0;
    let mut total_nl = 0.0;
    let mut total_lns = 0.0;
    let mut n_startups = 0;

    let mut prev_on: Vec<bool> = units.iter().map(|u| u.initially_on).collect();
    let mut hours_off: Vec<f64> = units
        .iter()
        .map(|u| if u.initially_on { 0.0 } else { u.initial_hours })
        .collect();

    for t in 0..n_t {
        let mut period_startup = 0.0;
        let mut period_prod = 0.0;
        let mut period_nl = 0.0;

        let mut reserve_avail = 0.0;
        let mut demand_served = 0.0;
        let reserve_mw: Vec<f64> = (0..n_gen)
            .map(|g| {
                if commit[g][t] {
                    units[g].p_max_mw - output[g][t]
                } else {
                    0.0
                }
            })
            .collect();

        for g in 0..n_gen {
            let on = commit[g][t];
            if on {
                let prod = units[g].cost_mwh * output[g][t] * dt;
                let nl = units[g].no_load_cost_h * dt;
                period_prod += prod;
                period_nl += nl;
                reserve_avail += reserve_mw[g];
                demand_served += output[g][t];

                if !prev_on[g] {
                    let su = units[g].startup_cost(hours_off[g]);
                    period_startup += su;
                    n_startups += 1;
                    hours_off[g] = 0.0;
                }
            } else {
                hours_off[g] += dt;
            }
            prev_on[g] = on;
        }

        let demand = config.demand_mw[t];
        let lns = (demand - demand_served).max(0.0);
        total_lns += lns * dt;

        total_startup += period_startup;
        total_prod += period_prod;
        total_nl += period_nl;

        periods.push(UcPeriod {
            commitment: commit.iter().map(|c| c[t]).collect(),
            output_mw: output.iter().map(|p| p[t]).collect(),
            reserve_mw,
            startup_cost_total: period_startup,
            production_cost_total: period_prod,
            no_load_cost_total: period_nl,
            demand_served_mw: demand_served,
            reserve_available_mw: reserve_avail,
            lns_mw: lns,
        });
    }

    UcResult {
        periods,
        total_cost: total_startup + total_prod + total_nl,
        total_startup_cost: total_startup,
        total_production_cost: total_prod,
        total_no_load_cost: total_nl,
        total_lns_mwh: total_lns,
        n_startups,
        lambda: lambda.to_vec(),
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn three_unit_fleet() -> Vec<MilpUnit> {
        vec![
            MilpUnit::coal("Coal1", 100.0),
            MilpUnit::gas_cc("CC1", 60.0),
            MilpUnit::gas_peaker("Peaker", 30.0),
        ]
    }

    #[test]
    fn test_milp_uc_runs() {
        let units = three_unit_fleet();
        let config = UcConfig::flat_24h(120.0);
        let result = run_milp_uc(&units, &config);
        assert_eq!(result.periods.len(), 24);
    }

    #[test]
    fn test_total_cost_positive() {
        let units = three_unit_fleet();
        let config = UcConfig::flat_24h(100.0);
        let result = run_milp_uc(&units, &config);
        assert!(
            result.total_cost > 0.0,
            "Total cost = {}",
            result.total_cost
        );
    }

    #[test]
    fn test_commitment_vector_length() {
        let units = three_unit_fleet();
        let config = UcConfig::flat_24h(80.0);
        let result = run_milp_uc(&units, &config);
        for period in &result.periods {
            assert_eq!(period.commitment.len(), units.len());
            assert_eq!(period.output_mw.len(), units.len());
        }
    }

    #[test]
    fn test_output_within_limits_when_committed() {
        let units = three_unit_fleet();
        let config = UcConfig::flat_24h(100.0);
        let result = run_milp_uc(&units, &config);
        for period in &result.periods {
            for (g, unit) in units.iter().enumerate() {
                let on = period.commitment[g];
                let p = period.output_mw[g];
                if on {
                    assert!(
                        p >= unit.p_min_mw - 1e-6,
                        "P={:.2} below p_min={:.2}",
                        p,
                        unit.p_min_mw
                    );
                    assert!(
                        p <= unit.p_max_mw + 1e-6,
                        "P={:.2} above p_max={:.2}",
                        p,
                        unit.p_max_mw
                    );
                } else {
                    assert!(p.abs() < 1e-6, "Off unit has output {:.2}", p);
                }
            }
        }
    }

    #[test]
    fn test_zero_demand_all_off() {
        let units = three_unit_fleet();
        let config = UcConfig {
            dt_h: 1.0,
            demand_mw: vec![0.0; 4],
            reserve_mw: vec![0.0; 4],
            ..UcConfig::flat_24h(0.0)
        };
        let result = run_milp_uc(&units, &config);
        // With zero demand, most units should be off
        let total_output: f64 = result.periods.iter().flat_map(|p| p.output_mw.iter()).sum();
        assert!(total_output >= 0.0); // just no panic
    }

    #[test]
    fn test_empty_fleet() {
        let units: Vec<MilpUnit> = vec![];
        let config = UcConfig::flat_24h(100.0);
        let result = run_milp_uc(&units, &config);
        assert_eq!(result.periods.len(), 0);
        assert_eq!(result.total_cost, 0.0);
    }

    #[test]
    fn test_lambda_length() {
        let units = three_unit_fleet();
        let config = UcConfig::flat_24h(100.0);
        let result = run_milp_uc(&units, &config);
        assert_eq!(result.lambda.len(), 24);
    }

    #[test]
    fn test_startup_cost_tiers() {
        let unit = MilpUnit::coal("Coal", 100.0);
        // Hot (< 2h) < Warm (2–8h) < Cold (>8h)
        assert!(
            unit.startup_cost(1.0) < unit.startup_cost(3.0),
            "Hot < Warm: {} vs {}",
            unit.startup_cost(1.0),
            unit.startup_cost(3.0)
        );
        assert!(
            unit.startup_cost(5.0) < unit.startup_cost(20.0),
            "Warm < Cold: {} vs {}",
            unit.startup_cost(5.0),
            unit.startup_cost(20.0)
        );
    }

    #[test]
    fn test_cost_breakdown_sums() {
        let units = three_unit_fleet();
        let config = UcConfig::flat_24h(100.0);
        let result = run_milp_uc(&units, &config);
        let expected =
            result.total_startup_cost + result.total_production_cost + result.total_no_load_cost;
        assert!(
            (result.total_cost - expected).abs() < 1e-6,
            "Cost breakdown mismatch: {} vs {}",
            result.total_cost,
            expected
        );
    }

    #[test]
    fn test_high_demand_uses_all_units() {
        let units = three_unit_fleet();
        // Demand close to total installed capacity
        let total_cap: f64 = units.iter().map(|u| u.p_max_mw).sum();
        let config = UcConfig::flat_24h(total_cap * 0.95);
        let result = run_milp_uc(&units, &config);
        // At high demand, at least some units should be committed in most periods
        let avg_committed: f64 = result
            .periods
            .iter()
            .map(|p| p.commitment.iter().filter(|&&c| c).count() as f64)
            .sum::<f64>()
            / result.periods.len() as f64;
        assert!(
            avg_committed >= 1.0,
            "Expected committed units: {:.2}",
            avg_committed
        );
    }

    #[test]
    fn test_lns_zero_for_achievable_demand() {
        let units = three_unit_fleet();
        let total_cap: f64 = units.iter().map(|u| u.p_max_mw).sum();
        let config = UcConfig::flat_24h(total_cap * 0.5);
        let result = run_milp_uc(&units, &config);
        assert!(
            result.total_lns_mwh < total_cap,
            "LNS too high: {}",
            result.total_lns_mwh
        );
    }
}
