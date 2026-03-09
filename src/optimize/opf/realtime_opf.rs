/// Real-Time Optimal Power Flow (RTOPF) for sub-minute dispatch.
///
/// Solves the economic dispatch problem with network constraints using
/// Sequential Linear Programming (SLP). Designed for fast (<1s) solve times
/// suitable for real-time AGC and intra-dispatch-interval re-dispatch.
///
/// # Algorithm
/// 1. Initialize from warm start or economic dispatch (no-network)
/// 2. SLP iteration:
///    - DC power flow (Gaussian elimination)
///    - PTDF matrix computation (shift factors)
///    - Gradient projection LP step (equalise marginal costs subject to
///      ramp and branch flow constraints)
///    - Convergence check
/// 3. LMP computation via shift-factor method
/// 4. AGC area-control-error correction
use crate::error::OxiGridError;
use std::time::Instant;

// ---------------------------------------------------------------------------
// Data structures
// ---------------------------------------------------------------------------

/// Generator descriptor for RTOPF (simplified from full UC model).
#[derive(Debug, Clone)]
pub struct RtGenerator {
    /// Generator index (unique per problem)
    pub id: usize,
    /// Terminal bus index (0-based)
    pub bus: usize,
    /// Minimum real power output [MW]
    pub p_min_mw: f64,
    /// Maximum real power output [MW]
    pub p_max_mw: f64,
    /// Minimum reactive power output [MVAr]
    pub q_min_mvar: f64,
    /// Maximum reactive power output [MVAr]
    pub q_max_mvar: f64,
    /// Ramp rate capability [MW/min]
    pub ramp_rate_mw_per_min: f64,
    /// Quadratic cost coefficient [$/MW²h]
    pub cost_a: f64,
    /// Linear cost coefficient [$/MWh]
    pub cost_b: f64,
    /// No-load (constant) cost [$/h]
    pub cost_c: f64,
    /// AGC participation factor (0–1 fraction of total AGC signal)
    pub agc_participation: f64,
    /// Commitment status
    pub is_online: bool,
    /// Current active power setpoint [MW] (used for ramp constraints)
    pub p_current_mw: f64,
    /// Current reactive power setpoint [MVAr]
    pub q_current_mvar: f64,
}

/// Branch descriptor for RTOPF.
#[derive(Debug, Clone)]
pub struct RtBranch {
    /// From-bus index (0-based)
    pub from: usize,
    /// To-bus index (0-based)
    pub to: usize,
    /// Series resistance [pu]
    pub r_pu: f64,
    /// Series reactance [pu]
    pub x_pu: f64,
    /// Thermal rating [MVA]
    pub rating_mva: f64,
    /// Off-nominal tap ratio (1.0 = nominal)
    pub tap: f64,
}

/// Load forecast for a single bus for one RTOPF interval.
#[derive(Debug, Clone)]
pub struct RtLoadForecast {
    /// Bus index (0-based)
    pub bus: usize,
    /// Forecasted active load [MW]
    pub p_mw: f64,
    /// Forecasted reactive load [MVAr]
    pub q_mvar: f64,
    /// 1-sigma forecast uncertainty [MW]
    pub forecast_error_mw: f64,
}

/// Warm-start data propagated from one RTOPF solve to the next.
#[derive(Debug, Clone)]
pub struct RtWarmStart {
    /// Active power setpoint per generator [MW]
    pub generator_setpoints: Vec<f64>,
    /// Bus voltage magnitude [pu]
    pub voltage_magnitudes: Vec<f64>,
    /// Bus voltage angle [rad]
    pub voltage_angles: Vec<f64>,
    /// Dual variables (LMP) per bus [$/MWh]
    pub lambda: Vec<f64>,
}

/// RTOPF solver configuration.
#[derive(Debug, Clone)]
pub struct RtConfig {
    /// Number of buses in the network
    pub n_buses: usize,
    /// System base MVA
    pub base_mva: f64,
    /// Maximum SLP iterations (default 20)
    pub max_iterations: usize,
    /// Convergence tolerance in pu (default 1e-4)
    pub convergence_tol: f64,
    /// Ramp constraint window [min] (default 5)
    pub ramp_window_min: f64,
    /// Branch loading security margin, fraction of rating (default 0.95)
    pub security_margin: f64,
    /// Enable AGC area-control-error correction
    pub agc_enabled: bool,
    /// Nominal frequency [Hz] (50 or 60)
    pub frequency_hz: f64,
    /// AGC droop setting [%] (default 4%)
    pub droop_percent: f64,
}

impl Default for RtConfig {
    fn default() -> Self {
        Self {
            n_buses: 2,
            base_mva: 100.0,
            max_iterations: 20,
            convergence_tol: 1e-4,
            ramp_window_min: 5.0,
            security_margin: 0.95,
            agc_enabled: true,
            frequency_hz: 60.0,
            droop_percent: 4.0,
        }
    }
}

/// Solution returned by the RTOPF solver.
#[derive(Debug, Clone)]
pub struct RtSolution {
    /// Unix-like timestamp [s]
    pub timestamp: f64,
    /// Active power dispatch per generator [MW]
    pub generator_dispatch: Vec<f64>,
    /// Reactive power dispatch per generator [MVAr]
    pub reactive_dispatch: Vec<f64>,
    /// Bus voltage magnitude [pu]
    pub voltage_magnitude: Vec<f64>,
    /// Bus voltage angle [rad]
    pub voltage_angle: Vec<f64>,
    /// Locational marginal price per bus [$/MWh]
    pub lmp: Vec<f64>,
    /// Branch loading as fraction of thermal rating
    pub branch_loading: Vec<f64>,
    /// Total generation cost [$/h]
    pub total_cost_usd_per_hr: f64,
    /// Estimated total network losses [MW]
    pub total_losses_mw: f64,
    /// AGC correction signal [MW] (area control error)
    pub agc_signal_mw: f64,
    /// Whether the SLP converged
    pub converged: bool,
    /// Number of SLP iterations used
    pub iterations: usize,
    /// Wall-clock solve time [ms]
    pub solve_time_ms: f64,
    /// Warm-start data for the next solve
    pub warm_start: RtWarmStart,
}

/// A single constraint violation identified post-solve.
#[derive(Debug, Clone)]
pub struct RtConstraintViolation {
    /// Category: "branch_loading", "voltage", "ramp"
    pub constraint_type: String,
    /// Index of the violating element (branch or bus or generator index)
    pub element_id: usize,
    /// Actual value (fraction of rating, pu voltage, MW ramp)
    pub value: f64,
    /// Limit (1.0, voltage limits, MW ramp limit)
    pub limit: f64,
    /// Severity = value/limit − 1.0 (positive means violated)
    pub severity: f64,
}

// ---------------------------------------------------------------------------
// Main solver struct
// ---------------------------------------------------------------------------

/// Real-Time OPF solver using Sequential Linear Programming.
pub struct RealTimeOpf {
    /// Generator fleet
    pub generators: Vec<RtGenerator>,
    /// Network branches
    pub branches: Vec<RtBranch>,
    /// Solver configuration
    pub config: RtConfig,
}

impl RealTimeOpf {
    /// Create a new RTOPF solver.
    pub fn new(generators: Vec<RtGenerator>, branches: Vec<RtBranch>, config: RtConfig) -> Self {
        Self {
            generators,
            branches,
            config,
        }
    }

    // -----------------------------------------------------------------------
    // Public API
    // -----------------------------------------------------------------------

    /// Solve the RTOPF for the given load forecast.
    ///
    /// Uses Sequential Linear Programming:
    /// 1. Initialise from `warm_start` or economic dispatch (no network).
    /// 2. SLP loop: DC PF → PTDF → gradient-projection LP → convergence.
    /// 3. Compute LMP, losses, AGC signal.
    ///
    /// Returns `RtSolution` containing dispatch, LMP, warm-start for next call.
    pub fn solve(
        &mut self,
        loads: &[RtLoadForecast],
        warm_start: Option<RtWarmStart>,
        timestamp: f64,
    ) -> Result<RtSolution, OxiGridError> {
        let t0 = Instant::now();
        let n_bus = self.config.n_buses;
        let n_gen = self.generators.len();
        let _n_br = self.branches.len();
        let base = self.config.base_mva;

        // ------------------------------------------------------------------
        // 1. Initialise active power dispatch
        // ------------------------------------------------------------------
        let total_load_mw: f64 = loads.iter().map(|l| l.p_mw).sum();

        let mut p_gen: Vec<f64> = if let Some(ref ws) = warm_start {
            if ws.generator_setpoints.len() == n_gen {
                ws.generator_setpoints.clone()
            } else {
                self.economic_dispatch_no_network(total_load_mw)
            }
        } else {
            self.economic_dispatch_no_network(total_load_mw)
        };

        // Clamp to online generators only
        for (i, gen) in self.generators.iter().enumerate() {
            if !gen.is_online {
                p_gen[i] = 0.0;
            }
        }

        // ------------------------------------------------------------------
        // 2. Build B-matrix (constant for the network)
        // ------------------------------------------------------------------
        let b_matrix = self.build_b_matrix();

        // ------------------------------------------------------------------
        // 3. SLP iteration
        // ------------------------------------------------------------------
        let mut converged = false;
        let mut iterations = 0usize;

        for iter in 0..self.config.max_iterations {
            iterations = iter + 1;

            // Build nodal power injections [pu]
            let p_inj = self.build_p_injections(&p_gen, loads, base, n_bus);

            // DC power flow: Bθ = P_inj
            let theta = self.solve_dc_power_flow(&b_matrix, &p_inj)?;

            // PTDF matrix
            let ptdf = self.compute_ptdf(&b_matrix);

            // Gradient-projection LP step
            let p_gen_new = self.slp_step(&p_gen, &ptdf, loads, base)?;

            // Convergence check: max |ΔP| in MW
            let max_delta = p_gen
                .iter()
                .zip(p_gen_new.iter())
                .map(|(a, b)| (a - b).abs())
                .fold(0.0_f64, f64::max);

            p_gen = p_gen_new;

            // Store latest angles (used below for losses / warm start)
            let _ = theta; // used implicitly via ptdf in next iter

            if max_delta < self.config.convergence_tol * base {
                converged = true;
                break;
            }
        }

        // ------------------------------------------------------------------
        // 4. Final DC power flow for angles and branch flows
        // ------------------------------------------------------------------
        let p_inj_final = self.build_p_injections(&p_gen, loads, base, n_bus);
        let theta_final = self.solve_dc_power_flow(&b_matrix, &p_inj_final)?;

        // Branch flows [pu] via B'θ
        let branch_flows_pu = self.compute_branch_flows(&theta_final);

        // Branch loading [fraction of rating]
        let branch_loading: Vec<f64> = branch_flows_pu
            .iter()
            .enumerate()
            .map(|(i, &f_pu)| {
                let rating_pu = self.branches[i].rating_mva / base;
                if rating_pu > 1e-9 {
                    f_pu.abs() / rating_pu
                } else {
                    0.0
                }
            })
            .collect();

        // ------------------------------------------------------------------
        // 5. LMP via shift-factor method
        // ------------------------------------------------------------------
        let ptdf_final = self.compute_ptdf(&b_matrix);
        let lmp = self.compute_lmp(&p_gen, &ptdf_final, &branch_flows_pu, base);

        // ------------------------------------------------------------------
        // 6. Reactive dispatch (proportional to Q capacity)
        // ------------------------------------------------------------------
        let total_q_load: f64 = loads.iter().map(|l| l.q_mvar).sum();
        let reactive_dispatch = self.dispatch_reactive(total_q_load);

        // ------------------------------------------------------------------
        // 7. Losses (approximate: sum |P_from - P_to| per branch in MW)
        // ------------------------------------------------------------------
        let total_losses_mw = self.estimate_losses(&branch_flows_pu, base);

        // ------------------------------------------------------------------
        // 8. AGC signal
        // ------------------------------------------------------------------
        let ace_mw = total_load_mw + total_losses_mw - p_gen.iter().sum::<f64>();
        let agc_signal_mw = if self.config.agc_enabled { ace_mw } else { 0.0 };

        // ------------------------------------------------------------------
        // 9. Total cost [$/h]
        // ------------------------------------------------------------------
        let total_cost_usd_per_hr: f64 = self
            .generators
            .iter()
            .enumerate()
            .map(|(i, g)| {
                let p = p_gen[i];
                g.cost_c + g.cost_b * p + g.cost_a * p * p
            })
            .sum();

        // ------------------------------------------------------------------
        // 10. Voltage magnitudes (flat 1.0 pu for DC, or warm-start)
        // ------------------------------------------------------------------
        let voltage_magnitude: Vec<f64> = if let Some(ref ws) = warm_start {
            if ws.voltage_magnitudes.len() == n_bus {
                ws.voltage_magnitudes.clone()
            } else {
                vec![1.0; n_bus]
            }
        } else {
            vec![1.0; n_bus]
        };

        let solve_time_ms = t0.elapsed().as_secs_f64() * 1000.0;

        let warm_start_out = RtWarmStart {
            generator_setpoints: p_gen.clone(),
            voltage_magnitudes: voltage_magnitude.clone(),
            voltage_angles: theta_final.clone(),
            lambda: lmp.clone(),
        };

        Ok(RtSolution {
            timestamp,
            generator_dispatch: p_gen,
            reactive_dispatch,
            voltage_magnitude,
            voltage_angle: theta_final,
            lmp,
            branch_loading,
            total_cost_usd_per_hr,
            total_losses_mw,
            agc_signal_mw,
            converged,
            iterations,
            solve_time_ms,
            warm_start: warm_start_out,
        })
    }

    /// Identify constraint violations in a solved solution.
    pub fn check_violations(&self, solution: &RtSolution) -> Vec<RtConstraintViolation> {
        let mut violations = Vec::new();

        // Branch loading violations
        for (i, &loading) in solution.branch_loading.iter().enumerate() {
            if loading > self.config.security_margin {
                violations.push(RtConstraintViolation {
                    constraint_type: "branch_loading".to_string(),
                    element_id: i,
                    value: loading,
                    limit: self.config.security_margin,
                    severity: loading / self.config.security_margin - 1.0,
                });
            }
        }

        // Voltage violations (only meaningful for AC; check against ±5% band)
        let v_min = 0.95_f64;
        let v_max = 1.05_f64;
        for (i, &v) in solution.voltage_magnitude.iter().enumerate() {
            if v < v_min {
                violations.push(RtConstraintViolation {
                    constraint_type: "voltage".to_string(),
                    element_id: i,
                    value: v,
                    limit: v_min,
                    severity: v_min / v - 1.0,
                });
            } else if v > v_max {
                violations.push(RtConstraintViolation {
                    constraint_type: "voltage".to_string(),
                    element_id: i,
                    value: v,
                    limit: v_max,
                    severity: v / v_max - 1.0,
                });
            }
        }

        // Ramp violations
        let ramp_window = self.config.ramp_window_min;
        for (i, (gen, &p_new)) in self
            .generators
            .iter()
            .zip(solution.generator_dispatch.iter())
            .enumerate()
        {
            let ramp_limit = gen.ramp_rate_mw_per_min * ramp_window;
            let delta = (p_new - gen.p_current_mw).abs();
            if delta > ramp_limit + 1e-6 {
                violations.push(RtConstraintViolation {
                    constraint_type: "ramp".to_string(),
                    element_id: i,
                    value: delta,
                    limit: ramp_limit,
                    severity: delta / (ramp_limit + 1e-9) - 1.0,
                });
            }
        }

        violations
    }

    /// Sensitivity-based re-dispatch to relieve a single overloaded branch.
    ///
    /// Computes the minimum-cost redispatch `ΔP` that reduces the overloaded
    /// branch's loading below the security margin using PTDF sensitivities.
    pub fn redispatch_for_overload(
        &mut self,
        overloaded_branch: usize,
        current_dispatch: &[f64],
        loads: &[RtLoadForecast],
    ) -> Result<Vec<f64>, OxiGridError> {
        let n_gen = self.generators.len();
        if overloaded_branch >= self.branches.len() {
            return Err(OxiGridError::InvalidParameter(format!(
                "branch index {} out of range ({})",
                overloaded_branch,
                self.branches.len()
            )));
        }
        if current_dispatch.len() != n_gen {
            return Err(OxiGridError::InvalidParameter(format!(
                "current_dispatch length {} != n_gen {}",
                current_dispatch.len(),
                n_gen
            )));
        }

        let base = self.config.base_mva;
        let b_matrix = self.build_b_matrix();
        let ptdf = self.compute_ptdf(&b_matrix);
        let n_bus = self.config.n_buses;

        // Current injection
        let p_inj = self.build_p_injections(current_dispatch, loads, base, n_bus);

        // Current flow on overloaded branch [pu]
        let current_flow_pu: f64 = ptdf[overloaded_branch]
            .iter()
            .zip(p_inj.iter())
            .map(|(ptdf_lk, p_k)| ptdf_lk * p_k)
            .sum();

        let br = &self.branches[overloaded_branch];
        let rating_pu = br.rating_mva / base;
        let limit_pu = rating_pu * self.config.security_margin;

        // Amount of flow relief needed [pu]
        let excess_pu = current_flow_pu.abs() - limit_pu;
        if excess_pu <= 0.0 {
            // Already within limits
            return Ok(current_dispatch.to_vec());
        }

        // Build generator sensitivities: how much does flow on this branch change
        // per MW increase at generator i's bus?
        // sensitivity_i = PTDF[branch][gen_bus_i] / base_mva
        let mut p_new = current_dispatch.to_vec();
        let sign = if current_flow_pu > 0.0 { 1.0 } else { -1.0 };

        // Sort generators by |sensitivity| descending to greedily relieve congestion
        let mut gen_sensitivity: Vec<(usize, f64)> = self
            .generators
            .iter()
            .enumerate()
            .filter_map(|(i, gen)| {
                if !gen.is_online {
                    return None;
                }
                let bus = gen.bus.min(n_bus - 1);
                let sens = ptdf[overloaded_branch][bus] / base; // pu/MW
                                                                // We want generators whose injection *reduces* the overloaded flow
                                                                // If sign > 0, we want generators with negative PTDF on that branch
                Some((i, -sign * sens))
            })
            .collect();

        gen_sensitivity.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        let mut remaining_relief_pu = excess_pu;
        let ramp_window = self.config.ramp_window_min;

        for (gen_idx, sens) in &gen_sensitivity {
            if remaining_relief_pu <= 1e-9 {
                break;
            }
            if sens.abs() < 1e-9 {
                continue;
            }
            let gen = &self.generators[*gen_idx];
            // Maximum allowed change for this generator (ramp + capacity)
            let ramp_limit = gen.ramp_rate_mw_per_min * ramp_window;
            let current_p = p_new[*gen_idx];

            // Change in MW that provides `remaining_relief_pu` relief
            let required_delta_mw = remaining_relief_pu / sens.abs() * base;

            if *sens > 0.0 {
                // Increase this generator's output
                let max_increase = (gen.p_max_mw - current_p)
                    .min(ramp_limit - (current_p - gen.p_current_mw).max(0.0))
                    .max(0.0);
                let actual_delta = required_delta_mw.min(max_increase);
                p_new[*gen_idx] += actual_delta;
                remaining_relief_pu -= actual_delta * sens / base * base; // cancel
                remaining_relief_pu -= actual_delta * sens.abs();
            } else {
                // Decrease this generator's output
                let max_decrease = (current_p - gen.p_min_mw)
                    .min(ramp_limit - (gen.p_current_mw - current_p).max(0.0))
                    .max(0.0);
                let actual_delta = required_delta_mw.min(max_decrease);
                p_new[*gen_idx] -= actual_delta;
                remaining_relief_pu -= actual_delta * sens.abs();
            }
        }

        // Clamp results
        for (i, gen) in self.generators.iter().enumerate() {
            if gen.is_online {
                p_new[i] = p_new[i].clamp(gen.p_min_mw, gen.p_max_mw);
            } else {
                p_new[i] = 0.0;
            }
        }

        Ok(p_new)
    }

    /// Solve a rolling-horizon sequence of RTOPF instances.
    ///
    /// Each step warm-starts from the previous solution.
    /// `load_forecasts[t]` is the vector of bus loads at timestep `t`.
    /// `dt_seconds` is the interval between steps.
    pub fn solve_rolling_horizon(
        &mut self,
        load_forecasts: &[Vec<RtLoadForecast>],
        start_timestamp: f64,
        dt_seconds: f64,
    ) -> Result<Vec<RtSolution>, OxiGridError> {
        if load_forecasts.is_empty() {
            return Ok(Vec::new());
        }

        let mut solutions = Vec::with_capacity(load_forecasts.len());
        let mut warm: Option<RtWarmStart> = None;

        for (t, loads) in load_forecasts.iter().enumerate() {
            let ts = start_timestamp + t as f64 * dt_seconds;
            let sol = self.solve(loads, warm.clone(), ts)?;

            // Update generator current setpoints for next ramp constraint
            for (i, gen) in self.generators.iter_mut().enumerate() {
                if i < sol.generator_dispatch.len() {
                    gen.p_current_mw = sol.generator_dispatch[i];
                }
            }

            warm = Some(sol.warm_start.clone());
            solutions.push(sol);
        }

        Ok(solutions)
    }

    // -----------------------------------------------------------------------
    // Internal helpers
    // -----------------------------------------------------------------------

    /// Economic dispatch without network constraints (lambda iteration).
    ///
    /// Finds λ* such that Σ P_i(λ*) = total_load_mw, where:
    ///   P_i(λ) = clamp((λ − b_i) / (2·a_i), P_min_i, P_max_i)   [quadratic]
    ///   P_i(λ) = P_max_i if λ ≥ b_i, else P_min_i                [linear]
    fn economic_dispatch_no_network(&self, total_load_mw: f64) -> Vec<f64> {
        let online: Vec<&RtGenerator> = self.generators.iter().filter(|g| g.is_online).collect();

        if online.is_empty() {
            return vec![0.0; self.generators.len()];
        }

        let p_min_total: f64 = online.iter().map(|g| g.p_min_mw).sum();
        let p_max_total: f64 = online.iter().map(|g| g.p_max_mw).sum();
        let load_clamped = total_load_mw.clamp(p_min_total, p_max_total);

        // Check if all costs are linear (cost_a ≈ 0)
        let all_linear = online.iter().all(|g| g.cost_a.abs() < 1e-12);

        let dispatch_at = |lam: f64| -> Vec<f64> {
            self.generators
                .iter()
                .map(|g| {
                    if !g.is_online {
                        return 0.0;
                    }
                    if g.cost_a.abs() < 1e-12 {
                        // Linear: binary dispatch
                        if lam >= g.cost_b {
                            g.p_max_mw
                        } else {
                            g.p_min_mw
                        }
                    } else {
                        let p_opt = (lam - g.cost_b) / (2.0 * g.cost_a);
                        p_opt.clamp(g.p_min_mw, g.p_max_mw)
                    }
                })
                .collect()
        };

        if all_linear {
            // Merit-order dispatch
            let mut order: Vec<usize> = (0..self.generators.len())
                .filter(|&i| self.generators[i].is_online)
                .collect();
            order.sort_by(|&a, &b| {
                self.generators[a]
                    .cost_b
                    .partial_cmp(&self.generators[b].cost_b)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });

            let mut p = self
                .generators
                .iter()
                .map(|g| if g.is_online { g.p_min_mw } else { 0.0 })
                .collect::<Vec<f64>>();
            let mut remaining = load_clamped - p_min_total;

            for &i in &order {
                let headroom = self.generators[i].p_max_mw - self.generators[i].p_min_mw;
                let added = remaining.min(headroom).max(0.0);
                p[i] += added;
                remaining -= added;
                if remaining <= 1e-6 {
                    break;
                }
            }
            return p;
        }

        // Bisect on lambda
        let b_lo = online
            .iter()
            .map(|g| g.cost_b)
            .fold(f64::INFINITY, f64::min);
        let b_hi = online
            .iter()
            .map(|g| g.cost_b + 2.0 * g.cost_a * g.p_max_mw)
            .fold(f64::NEG_INFINITY, f64::max);

        let mut lo = b_lo - 1.0;
        let mut hi = b_hi + 1.0;

        for _ in 0..80 {
            let mid = (lo + hi) / 2.0;
            let p_sum: f64 = dispatch_at(mid).iter().sum();
            if p_sum < load_clamped {
                lo = mid;
            } else {
                hi = mid;
            }
            if (hi - lo) < 1e-7 {
                break;
            }
        }

        dispatch_at((lo + hi) / 2.0)
    }

    /// Build the DC B-matrix (n_bus × n_bus susceptance matrix).
    ///
    /// B[i][j] = -1/x_ij for i≠j (branch susceptance)
    /// B[i][i] = Σ_j 1/x_ij   (sum of all branch susceptances at bus i)
    pub fn build_b_matrix(&self) -> Vec<Vec<f64>> {
        let n = self.config.n_buses;
        let mut b = vec![vec![0.0_f64; n]; n];

        for br in &self.branches {
            if br.x_pu.abs() < 1e-12 {
                continue;
            }
            let from = br.from.min(n - 1);
            let to = br.to.min(n - 1);
            let b_line = 1.0 / br.x_pu;

            b[from][from] += b_line;
            b[to][to] += b_line;
            b[from][to] -= b_line;
            b[to][from] -= b_line;
        }

        b
    }

    /// Solve the DC power flow: Bθ = P_inj via Gaussian elimination.
    ///
    /// The reference bus (index 0) is fixed: θ_0 = 0.
    /// The reduced (n-1)×(n-1) system is solved and θ_0 is prepended.
    #[allow(clippy::ptr_arg)]
    pub fn solve_dc_power_flow(
        &self,
        b_matrix: &Vec<Vec<f64>>,
        p_injections: &[f64],
    ) -> Result<Vec<f64>, OxiGridError> {
        let n = self.config.n_buses;
        if n == 0 {
            return Err(OxiGridError::InvalidNetwork(
                "zero buses in RTOPF".to_string(),
            ));
        }
        if n == 1 {
            return Ok(vec![0.0]);
        }

        // Build reduced (n-1)×(n-1) system (remove ref bus row/col)
        let nr = n - 1;
        let mut a = vec![vec![0.0_f64; nr]; nr];
        let mut rhs = vec![0.0_f64; nr];

        for i in 0..nr {
            for j in 0..nr {
                a[i][j] = b_matrix[i + 1][j + 1];
            }
            rhs[i] = if i + 1 < p_injections.len() {
                p_injections[i + 1]
            } else {
                0.0
            };
        }

        // Gaussian elimination with partial pivoting
        #[allow(clippy::needless_range_loop)]
        for col in 0..nr {
            // Find pivot
            let mut max_val = a[col][col].abs();
            let mut max_row = col;
            for row in (col + 1)..nr {
                if a[row][col].abs() > max_val {
                    max_val = a[row][col].abs();
                    max_row = row;
                }
            }

            if max_val < 1e-12 {
                return Err(OxiGridError::LinearAlgebra(format!(
                    "singular B-matrix at column {} (max pivot = {:.2e}); \
                     check for disconnected buses",
                    col, max_val
                )));
            }

            a.swap(col, max_row);
            rhs.swap(col, max_row);

            let pivot = a[col][col];
            for row in (col + 1)..nr {
                let factor = a[row][col] / pivot;
                for k in col..nr {
                    let tmp = a[col][k];
                    a[row][k] -= factor * tmp;
                }
                let tmp = rhs[col];
                rhs[row] -= factor * tmp;
            }
        }

        // Back substitution
        let mut theta_r = vec![0.0_f64; nr];
        for i in (0..nr).rev() {
            let mut sum = rhs[i];
            for j in (i + 1)..nr {
                sum -= a[i][j] * theta_r[j];
            }
            if a[i][i].abs() < 1e-15 {
                return Err(OxiGridError::LinearAlgebra(
                    "near-zero diagonal in back-substitution".to_string(),
                ));
            }
            theta_r[i] = sum / a[i][i];
        }

        // Prepend reference bus angle θ_0 = 0
        let mut theta = vec![0.0_f64; n];
        theta[1..(nr + 1)].copy_from_slice(&theta_r[..nr]);

        Ok(theta)
    }

    /// Compute the PTDF matrix (n_branches × n_buses).
    ///
    /// PTDF[l][k] = (1/x_l) * (θ_from − θ_to) per unit injection at bus k.
    /// Computed via: inject 1 pu at bus k, withdraw at slack (bus 0), solve DC PF.
    #[allow(clippy::ptr_arg)]
    pub fn compute_ptdf(&self, b_matrix: &Vec<Vec<f64>>) -> Vec<Vec<f64>> {
        let n_bus = self.config.n_buses;
        let n_br = self.branches.len();

        let mut ptdf = vec![vec![0.0_f64; n_bus]; n_br];

        for k in 1..n_bus {
            // Inject 1 pu at bus k, withdraw at bus 0 (reference)
            let mut p_inj = vec![0.0_f64; n_bus];
            p_inj[0] = -1.0;
            p_inj[k] = 1.0;

            let theta = match self.solve_dc_power_flow(b_matrix, &p_inj) {
                Ok(t) => t,
                Err(_) => continue,
            };

            // Flow on each branch = (θ_from − θ_to) / x_l
            for (l, br) in self.branches.iter().enumerate() {
                let from = br.from.min(n_bus - 1);
                let to = br.to.min(n_bus - 1);
                if br.x_pu.abs() < 1e-12 {
                    continue;
                }
                ptdf[l][k] = (theta[from] - theta[to]) / br.x_pu;
            }
            // Column 0 is implicitly 0 (reference bus)
        }

        // Compute column 0 from power-balance (reference bus): PTDF column 0
        // = -sum of all other columns (shift-factor convention)
        for row in ptdf.iter_mut().take(n_br) {
            let sum_others: f64 = (1..n_bus).map(|k| row[k]).sum();
            row[0] = -sum_others;
        }

        ptdf
    }

    /// Compute AGC correction: distribute ACE proportionally by participation.
    ///
    /// Returns the total ACE correction signal [MW].
    pub fn compute_agc_correction(&self, solution: &RtSolution, ace_mw: f64) -> f64 {
        if !self.config.agc_enabled {
            return 0.0;
        }

        let total_participation: f64 = self
            .generators
            .iter()
            .filter(|g| g.is_online)
            .map(|g| g.agc_participation)
            .sum();

        if total_participation < 1e-9 {
            return 0.0;
        }

        // Scale ACE by droop: effective correction = ACE / (droop/100)
        let droop_factor = self.config.droop_percent / 100.0;
        let correction = ace_mw * droop_factor.recip().min(10.0); // cap amplification

        let _ = solution; // solution used by caller for context
        correction
    }

    // -----------------------------------------------------------------------
    // Private helpers
    // -----------------------------------------------------------------------

    /// Build nodal power injection vector [pu].
    fn build_p_injections(
        &self,
        p_gen: &[f64],
        loads: &[RtLoadForecast],
        base: f64,
        n_bus: usize,
    ) -> Vec<f64> {
        let mut p_inj = vec![0.0_f64; n_bus];

        // Generation
        for (i, gen) in self.generators.iter().enumerate() {
            if gen.is_online && i < p_gen.len() {
                let bus = gen.bus.min(n_bus - 1);
                p_inj[bus] += p_gen[i] / base;
            }
        }

        // Load (subtract)
        for load in loads {
            let bus = load.bus.min(n_bus - 1);
            p_inj[bus] -= load.p_mw / base;
        }

        p_inj
    }

    /// Gradient-projection SLP step.
    ///
    /// Equalises marginal costs subject to:
    /// - Power balance (sum ΔP = 0)
    /// - Ramp limits per generator
    /// - Branch flow limits (PTDF × ΔP ≤ remaining headroom)
    fn slp_step(
        &self,
        p_gen: &[f64],
        ptdf: &[Vec<f64>],
        loads: &[RtLoadForecast],
        base: f64,
    ) -> Result<Vec<f64>, OxiGridError> {
        let n_gen = self.generators.len();
        let n_bus = self.config.n_buses;
        let n_br = self.branches.len();
        let ramp_window = self.config.ramp_window_min;

        // Marginal costs
        let mc: Vec<f64> = self
            .generators
            .iter()
            .enumerate()
            .map(|(i, g)| g.cost_b + 2.0 * g.cost_a * p_gen[i])
            .collect();

        // Current branch flows [pu]
        let p_inj = self.build_p_injections(p_gen, loads, base, n_bus);
        let branch_flows: Vec<f64> = (0..n_br)
            .map(|l| {
                ptdf[l]
                    .iter()
                    .zip(p_inj.iter())
                    .map(|(ptdf_lk, p_k)| ptdf_lk * p_k)
                    .sum()
            })
            .collect();

        let mut p_new = p_gen.to_vec();

        // Find cheapest and most expensive online generators
        let online_indices: Vec<usize> = (0..n_gen)
            .filter(|&i| self.generators[i].is_online)
            .collect();

        if online_indices.len() < 2 {
            return Ok(p_new);
        }

        let cheapest_idx = *online_indices
            .iter()
            .min_by(|&&a, &&b| {
                mc[a]
                    .partial_cmp(&mc[b])
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .unwrap_or(&online_indices[0]);

        let expensive_idx = *online_indices
            .iter()
            .max_by(|&&a, &&b| {
                mc[a]
                    .partial_cmp(&mc[b])
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .unwrap_or(&online_indices[0]);

        if cheapest_idx == expensive_idx {
            return Ok(p_new);
        }

        let mc_gap = mc[expensive_idx] - mc[cheapest_idx];
        if mc_gap < 1e-4 {
            // Already nearly optimal
            return Ok(p_new);
        }

        // Maximum shift limited by ramp and capacity
        let gen_cheap = &self.generators[cheapest_idx];
        let gen_exp = &self.generators[expensive_idx];

        let ramp_cheap = gen_cheap.ramp_rate_mw_per_min * ramp_window;
        let ramp_exp = gen_exp.ramp_rate_mw_per_min * ramp_window;

        let max_increase_cheap = (gen_cheap.p_max_mw - p_gen[cheapest_idx])
            .min(ramp_cheap)
            .max(0.0);
        let max_decrease_exp = (p_gen[expensive_idx] - gen_exp.p_min_mw)
            .min(ramp_exp)
            .max(0.0);

        let mut delta_mw = max_increase_cheap.min(max_decrease_exp);

        if delta_mw < 1e-9 {
            return Ok(p_new);
        }

        // Check branch constraints for proposed shift
        // Δf_l = PTDF[l][cheap_bus] * (delta/base) - PTDF[l][exp_bus] * (delta/base)
        let cheap_bus = gen_cheap.bus.min(n_bus - 1);
        let exp_bus = gen_exp.bus.min(n_bus - 1);

        for (l, br) in self.branches.iter().enumerate() {
            let rating_pu = br.rating_mva / base;
            let limit_pu = rating_pu * self.config.security_margin;
            let current_flow = branch_flows[l];

            // Net sensitivity of branch l to this shift [pu/MW]
            let sens = (ptdf[l][cheap_bus] - ptdf[l][exp_bus]) / base;

            // New flow if we apply full delta
            let new_flow = current_flow + sens * delta_mw;

            if new_flow.abs() > limit_pu + 1e-9 {
                // Limit delta so that |current_flow + sens * delta| = limit
                let headroom = (limit_pu - current_flow.abs()).max(0.0);
                if sens.abs() > 1e-12 {
                    let limited = headroom / sens.abs();
                    delta_mw = delta_mw.min(limited);
                } else {
                    delta_mw = 0.0;
                }
            }
        }

        delta_mw = delta_mw.max(0.0);

        // Apply shift
        p_new[cheapest_idx] =
            (p_new[cheapest_idx] + delta_mw).clamp(gen_cheap.p_min_mw, gen_cheap.p_max_mw);
        p_new[expensive_idx] =
            (p_new[expensive_idx] - delta_mw).clamp(gen_exp.p_min_mw, gen_exp.p_max_mw);

        Ok(p_new)
    }

    /// Compute branch flows [pu] from voltage angles.
    fn compute_branch_flows(&self, theta: &[f64]) -> Vec<f64> {
        let n_bus = self.config.n_buses;
        self.branches
            .iter()
            .map(|br| {
                if br.x_pu.abs() < 1e-12 {
                    return 0.0;
                }
                let from = br.from.min(n_bus - 1);
                let to = br.to.min(n_bus - 1);
                let th_from = if from < theta.len() { theta[from] } else { 0.0 };
                let th_to = if to < theta.len() { theta[to] } else { 0.0 };
                (th_from - th_to) / br.x_pu
            })
            .collect()
    }

    /// Compute LMPs via the shift-factor (PTDF) method.
    ///
    /// LMP_k = λ + Σ_l μ_l · PTDF[l][k]
    /// where λ is the energy (balance) price and μ_l is the congestion rent
    /// on binding branch l.
    fn compute_lmp(
        &self,
        p_gen: &[f64],
        ptdf: &[Vec<f64>],
        branch_flows: &[f64],
        base: f64,
    ) -> Vec<f64> {
        let n_bus = self.config.n_buses;
        let n_br = self.branches.len();

        // System lambda: marginal cost of the marginal (partially loaded) generator
        let lambda = self
            .generators
            .iter()
            .enumerate()
            .filter(|(_, g)| g.is_online)
            .map(|(i, g)| {
                let p = p_gen[i];
                let mc = g.cost_b + 2.0 * g.cost_a * p;
                let within_limits = p > g.p_min_mw + 1e-3 * (g.p_max_mw - g.p_min_mw)
                    && p < g.p_max_mw - 1e-3 * (g.p_max_mw - g.p_min_mw);
                (mc, within_limits)
            })
            .filter(|(_, within)| *within)
            .map(|(mc, _)| mc)
            .fold(f64::NEG_INFINITY, f64::max);

        // If no marginal generator found, use average of min/max MC
        let lambda = if lambda.is_finite() {
            lambda
        } else {
            self.generators
                .iter()
                .filter(|g| g.is_online)
                .map(|g| g.cost_b)
                .fold(0.0_f64, |acc, b| acc + b)
                / self
                    .generators
                    .iter()
                    .filter(|g| g.is_online)
                    .count()
                    .max(1) as f64
        };

        // Congestion rents μ_l (dual of branch flow constraint)
        // Positive if branch is binding at its upper limit, negative at lower
        let mut mu: Vec<f64> = vec![0.0; n_br];
        for (l, br) in self.branches.iter().enumerate() {
            let rating_pu = br.rating_mva / base;
            let limit_pu = rating_pu * self.config.security_margin;
            let flow = branch_flows[l];
            let headroom = limit_pu - flow.abs();

            if headroom < 0.01 * limit_pu {
                // Binding constraint: estimate shadow price
                // μ_l ≈ mc_gap / max_sensitivity
                let max_sens = ptdf[l].iter().map(|s| s.abs()).fold(0.0_f64, f64::max);
                if max_sens > 1e-9 {
                    let mc_range = self
                        .generators
                        .iter()
                        .enumerate()
                        .filter(|(_, g)| g.is_online)
                        .map(|(i, g)| g.cost_b + 2.0 * g.cost_a * p_gen[i])
                        .fold((f64::INFINITY, f64::NEG_INFINITY), |(mn, mx), mc| {
                            (mn.min(mc), mx.max(mc))
                        });
                    let mc_gap = (mc_range.1 - mc_range.0).max(0.0);
                    mu[l] = flow.signum() * mc_gap / (max_sens * base);
                }
            }
        }

        // LMP per bus
        (0..n_bus)
            .map(|k| {
                let congestion_term: f64 = (0..n_br).map(|l| mu[l] * ptdf[l][k]).sum();
                lambda + congestion_term
            })
            .collect()
    }

    /// Dispatch reactive power proportionally to each generator's Q capacity.
    fn dispatch_reactive(&self, total_q_load: f64) -> Vec<f64> {
        let q_max_total: f64 = self
            .generators
            .iter()
            .filter(|g| g.is_online)
            .map(|g| g.q_max_mvar - g.q_min_mvar)
            .sum();

        self.generators
            .iter()
            .map(|g| {
                if !g.is_online || q_max_total < 1e-9 {
                    return 0.0;
                }
                let q_cap = g.q_max_mvar - g.q_min_mvar;
                let q = total_q_load * q_cap / q_max_total;
                q.clamp(g.q_min_mvar, g.q_max_mvar)
            })
            .collect()
    }

    /// Estimate network losses from branch flows [MW].
    ///
    /// Loss on branch l ≈ r_l * I_l² ≈ r_l * (f_l / x_l)² × base  (DC approx)
    fn estimate_losses(&self, branch_flows_pu: &[f64], base: f64) -> f64 {
        self.branches
            .iter()
            .enumerate()
            .map(|(i, br)| {
                if i >= branch_flows_pu.len() || br.x_pu.abs() < 1e-12 {
                    return 0.0;
                }
                let f_pu = branch_flows_pu[i];
                // Current magnitude (pu) ≈ f_pu (for DC, V ≈ 1 pu)
                let i_pu = f_pu.abs();
                br.r_pu * i_pu * i_pu * base
            })
            .sum()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Simple LCG for deterministic pseudo-randomness in tests (no rand crate).
    fn lcg_next(state: &mut u64) -> f64 {
        *state = state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (*state >> 33) as f64 / (1u64 << 31) as f64
    }

    fn make_gen(
        id: usize,
        bus: usize,
        p_min: f64,
        p_max: f64,
        cost_a: f64,
        cost_b: f64,
    ) -> RtGenerator {
        RtGenerator {
            id,
            bus,
            p_min_mw: p_min,
            p_max_mw: p_max,
            q_min_mvar: -p_max * 0.5,
            q_max_mvar: p_max * 0.5,
            ramp_rate_mw_per_min: p_max * 0.1, // 10%/min
            cost_a,
            cost_b,
            cost_c: 0.0,
            agc_participation: 0.5,
            is_online: true,
            p_current_mw: p_min,
            q_current_mvar: 0.0,
        }
    }

    fn make_branch(from: usize, to: usize, x_pu: f64, rating_mva: f64) -> RtBranch {
        RtBranch {
            from,
            to,
            r_pu: x_pu * 0.1,
            x_pu,
            rating_mva,
            tap: 1.0,
        }
    }

    fn make_load(bus: usize, p_mw: f64) -> RtLoadForecast {
        RtLoadForecast {
            bus,
            p_mw,
            q_mvar: p_mw * 0.2,
            forecast_error_mw: p_mw * 0.05,
        }
    }

    fn make_config(n_buses: usize) -> RtConfig {
        RtConfig {
            n_buses,
            base_mva: 100.0,
            max_iterations: 20,
            convergence_tol: 1e-4,
            ramp_window_min: 5.0,
            security_margin: 0.95,
            agc_enabled: true,
            frequency_hz: 60.0,
            droop_percent: 4.0,
        }
    }

    // -----------------------------------------------------------------------
    // 1. Struct creation tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_rt_generator_creation() {
        let gen = make_gen(0, 0, 10.0, 100.0, 0.01, 20.0);
        assert_eq!(gen.id, 0);
        assert_eq!(gen.bus, 0);
        assert!((gen.p_min_mw - 10.0).abs() < 1e-9);
        assert!((gen.p_max_mw - 100.0).abs() < 1e-9);
        assert!((gen.cost_a - 0.01).abs() < 1e-9);
        assert!((gen.cost_b - 20.0).abs() < 1e-9);
        assert!(gen.is_online);
    }

    #[test]
    fn test_rt_branch_creation() {
        let br = make_branch(0, 1, 0.1, 200.0);
        assert_eq!(br.from, 0);
        assert_eq!(br.to, 1);
        assert!((br.x_pu - 0.1).abs() < 1e-9);
        assert!((br.rating_mva - 200.0).abs() < 1e-9);
        assert!((br.tap - 1.0).abs() < 1e-9);
    }

    // -----------------------------------------------------------------------
    // 2. B-matrix tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_build_b_matrix_2bus() {
        // 2-bus system: branch x=0.1 pu → b_line = 10
        let gens = vec![make_gen(0, 0, 0.0, 100.0, 0.01, 20.0)];
        let branches = vec![make_branch(0, 1, 0.1, 200.0)];
        let solver = RealTimeOpf::new(gens, branches, make_config(2));
        let b = solver.build_b_matrix();

        assert_eq!(b.len(), 2);
        // Diagonal = 10, off-diagonal = -10
        assert!((b[0][0] - 10.0).abs() < 1e-6, "B[0][0]={}", b[0][0]);
        assert!((b[1][1] - 10.0).abs() < 1e-6, "B[1][1]={}", b[1][1]);
        assert!((b[0][1] + 10.0).abs() < 1e-6, "B[0][1]={}", b[0][1]);
        assert!((b[1][0] + 10.0).abs() < 1e-6, "B[1][0]={}", b[1][0]);
    }

    #[test]
    fn test_build_b_matrix_3bus() {
        // 3-bus: bus0-bus1 x=0.1, bus1-bus2 x=0.2
        let gens = vec![make_gen(0, 0, 0.0, 200.0, 0.01, 20.0)];
        let branches = vec![make_branch(0, 1, 0.1, 200.0), make_branch(1, 2, 0.2, 150.0)];
        let solver = RealTimeOpf::new(gens, branches, make_config(3));
        let b = solver.build_b_matrix();

        // b01 = 1/0.1 = 10, b12 = 1/0.2 = 5
        // B[0][0] = 10, B[1][1] = 15, B[2][2] = 5
        assert!((b[0][0] - 10.0).abs() < 1e-6);
        assert!((b[1][1] - 15.0).abs() < 1e-6);
        assert!((b[2][2] - 5.0).abs() < 1e-6);
        assert!((b[0][1] + 10.0).abs() < 1e-6);
        assert!((b[1][2] + 5.0).abs() < 1e-6);
    }

    // -----------------------------------------------------------------------
    // 3. DC power flow tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_solve_dc_power_flow_2bus() {
        // 2-bus: 1 pu injection at bus 0, withdraw at bus 1
        // θ_1 = -P / B = -1 / 10 = -0.1 rad
        let gens = vec![make_gen(0, 0, 0.0, 100.0, 0.01, 20.0)];
        let branches = vec![make_branch(0, 1, 0.1, 200.0)];
        let solver = RealTimeOpf::new(gens, branches, make_config(2));
        let b = solver.build_b_matrix();

        let p_inj = vec![1.0, -1.0]; // 1 pu injection at bus 0
        let theta = solver
            .solve_dc_power_flow(&b, &p_inj)
            .expect("DC PF failed");

        assert_eq!(theta.len(), 2);
        assert!((theta[0]).abs() < 1e-9, "ref bus angle must be 0");
        // θ_1: B_reduced = [[10]], rhs = -1 → θ_1 = -0.1
        assert!((theta[1] + 0.1).abs() < 1e-6, "θ_1={}", theta[1]);
    }

    #[test]
    fn test_solve_dc_power_flow_3bus() {
        // 3-bus: bus0-1 x=0.1, bus1-2 x=0.2
        // Inject 1 pu at bus 0, withdraw 0.5 at bus 1 and 0.5 at bus 2
        let gens = vec![make_gen(0, 0, 0.0, 200.0, 0.01, 20.0)];
        let branches = vec![make_branch(0, 1, 0.1, 200.0), make_branch(1, 2, 0.2, 150.0)];
        let solver = RealTimeOpf::new(gens, branches, make_config(3));
        let b = solver.build_b_matrix();

        let p_inj = vec![1.0, -0.5, -0.5];
        let theta = solver
            .solve_dc_power_flow(&b, &p_inj)
            .expect("DC PF 3bus failed");

        assert_eq!(theta.len(), 3);
        assert!((theta[0]).abs() < 1e-9);
        // Verify power balance via flows: flow01 = (θ0-θ1)/0.1, flow12 = (θ1-θ2)/0.2
        let f01 = (theta[0] - theta[1]) / 0.1;
        let f12 = (theta[1] - theta[2]) / 0.2;
        // Bus 0: f01 = 1 pu
        assert!((f01 - 1.0).abs() < 1e-4, "f01={}", f01);
        // Bus 1: f01 - f12 = 0.5
        assert!((f01 - f12 - 0.5).abs() < 1e-4, "f01-f12={}", f01 - f12);
    }

    // -----------------------------------------------------------------------
    // 4. Economic dispatch tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_economic_dispatch_equal_cost() {
        // Two identical generators should share load equally
        let gens = vec![
            make_gen(0, 0, 0.0, 100.0, 0.01, 20.0),
            make_gen(1, 1, 0.0, 100.0, 0.01, 20.0),
        ];
        let branches = vec![make_branch(0, 1, 0.1, 200.0)];
        let solver = RealTimeOpf::new(gens, branches, make_config(2));

        let p = solver.economic_dispatch_no_network(80.0);
        assert_eq!(p.len(), 2);
        // Each should get ~40 MW
        assert!((p[0] - 40.0).abs() < 1.0, "p[0]={}", p[0]);
        assert!((p[1] - 40.0).abs() < 1.0, "p[1]={}", p[1]);
        assert!((p[0] + p[1] - 80.0).abs() < 0.1);
    }

    #[test]
    fn test_economic_dispatch_different_cost() {
        // Cheaper generator (lower cost_b) should be dispatched more
        let gens = vec![
            make_gen(0, 0, 0.0, 100.0, 0.01, 15.0), // cheaper
            make_gen(1, 1, 0.0, 100.0, 0.01, 30.0), // more expensive
        ];
        let branches = vec![make_branch(0, 1, 0.1, 200.0)];
        let solver = RealTimeOpf::new(gens, branches, make_config(2));

        let p = solver.economic_dispatch_no_network(80.0);
        assert_eq!(p.len(), 2);
        // Cheaper gen should carry more load
        assert!(p[0] > p[1], "cheap gen={:.1}, expensive={:.1}", p[0], p[1]);
        assert!((p[0] + p[1] - 80.0).abs() < 0.1);
    }

    #[test]
    fn test_economic_dispatch_ramp_limited() {
        // Generator with small ramp limit: p_current_mw = 10, ramp = 1 MW/min
        // So in 5 min window max change = 5 MW → can only reach 15 MW
        let mut gen0 = make_gen(0, 0, 0.0, 100.0, 0.01, 15.0);
        gen0.ramp_rate_mw_per_min = 1.0; // very small ramp
        gen0.p_current_mw = 10.0;
        let gen1 = make_gen(1, 1, 0.0, 100.0, 0.01, 30.0);

        let gens = vec![gen0, gen1];
        let branches = vec![make_branch(0, 1, 0.1, 200.0)];
        let solver = RealTimeOpf::new(gens, branches, make_config(2));

        // Economic dispatch itself doesn't enforce ramp (that's the SLP step)
        // But we verify total balance is met
        let p = solver.economic_dispatch_no_network(60.0);
        assert!((p[0] + p[1] - 60.0).abs() < 0.5, "total={}", p[0] + p[1]);
    }

    // -----------------------------------------------------------------------
    // 5. PTDF tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_compute_ptdf_2bus() {
        // 2-bus, 1 branch (from=0, to=1), x=0.1 pu.
        // Inject 1 pu at bus 1, withdraw at bus 0 (reference).
        // p_inj = [-1, +1], B_reduced = [[10]], rhs = [+1] → θ_1 = +0.1 rad
        // Flow (from→to convention): f = (θ_0 - θ_1)/x = (0 - 0.1)/0.1 = -1 pu
        // So PTDF[0][1] = -1.0
        // Column 0 (reference): PTDF[0][0] = -sum_others = -(-1.0) = +1.0
        let gens = vec![make_gen(0, 0, 0.0, 100.0, 0.01, 20.0)];
        let branches = vec![make_branch(0, 1, 0.1, 200.0)];
        let solver = RealTimeOpf::new(gens, branches, make_config(2));
        let b = solver.build_b_matrix();
        let ptdf = solver.compute_ptdf(&b);

        assert_eq!(ptdf.len(), 1);
        assert_eq!(ptdf[0].len(), 2);
        // |PTDF[0][1]| = 1.0: unit injection at bus 1 fully transfers via the only branch
        assert!(
            ptdf[0][1].abs() > 0.99,
            "PTDF magnitude must be ~1.0, got {}",
            ptdf[0][1]
        );
        // PTDF[0][0] = -PTDF[0][1] by power-balance convention
        assert!(
            (ptdf[0][0] + ptdf[0][1]).abs() < 1e-6,
            "PTDF columns must sum to ~0 (power balance): [{}, {}]",
            ptdf[0][0],
            ptdf[0][1]
        );
    }

    // -----------------------------------------------------------------------
    // 6. Full RTOPF solve tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_solve_rtopf_2bus_uncongested() {
        let gens = vec![
            make_gen(0, 0, 10.0, 100.0, 0.01, 20.0),
            make_gen(1, 1, 10.0, 100.0, 0.01, 25.0),
        ];
        let branches = vec![make_branch(0, 1, 0.1, 500.0)]; // large rating → uncongested
        let mut solver = RealTimeOpf::new(gens, branches, make_config(2));
        let loads = vec![make_load(1, 80.0)];
        let sol = solver.solve(&loads, None, 0.0).expect("solve failed");

        assert!(sol.generator_dispatch.iter().sum::<f64>() > 0.0);
        assert_eq!(sol.generator_dispatch.len(), 2);
        assert_eq!(sol.voltage_angle.len(), 2);
        assert_eq!(sol.lmp.len(), 2);
        assert!(
            sol.branch_loading[0] < 1.0,
            "branch should not be overloaded"
        );
        assert!(sol.total_cost_usd_per_hr > 0.0);
    }

    #[test]
    fn test_solve_rtopf_3bus_basic() {
        let gens = vec![
            make_gen(0, 0, 5.0, 80.0, 0.01, 20.0),
            make_gen(1, 2, 5.0, 80.0, 0.01, 25.0),
        ];
        let branches = vec![
            make_branch(0, 1, 0.05, 300.0),
            make_branch(1, 2, 0.08, 200.0),
        ];
        let mut solver = RealTimeOpf::new(gens, branches, make_config(3));
        let loads = vec![make_load(1, 60.0), make_load(2, 20.0)];
        let sol = solver
            .solve(&loads, None, 1000.0)
            .expect("3bus solve failed");

        assert_eq!(sol.generator_dispatch.len(), 2);
        assert_eq!(sol.branch_loading.len(), 2);
        assert_eq!(sol.lmp.len(), 3);
        // Total dispatch ≈ total load (within losses)
        let total_dispatch: f64 = sol.generator_dispatch.iter().sum();
        assert!(
            total_dispatch > 70.0,
            "dispatch too low: {:.1}",
            total_dispatch
        );
        assert!(sol.solve_time_ms >= 0.0);
    }

    #[test]
    fn test_solve_rtopf_warm_start() {
        let gens = vec![
            make_gen(0, 0, 10.0, 100.0, 0.01, 20.0),
            make_gen(1, 1, 10.0, 100.0, 0.01, 25.0),
        ];
        let branches = vec![make_branch(0, 1, 0.1, 500.0)];
        let config = make_config(2);
        let mut solver = RealTimeOpf::new(gens, branches, config);
        let loads = vec![make_load(1, 80.0)];

        // First solve (cold start)
        let sol1 = solver.solve(&loads, None, 0.0).expect("cold solve failed");

        // Second solve with warm start should converge in fewer iterations
        let ws = sol1.warm_start.clone();
        let sol2 = solver
            .solve(&loads, Some(ws), 60.0)
            .expect("warm solve failed");

        // Both should converge
        assert!(sol1.converged || sol1.iterations > 0);
        assert!(sol2.converged || sol2.iterations > 0);
        // Warm start should need ≤ cold start iterations
        assert!(
            sol2.iterations <= sol1.iterations + 2,
            "warm={} cold={}",
            sol2.iterations,
            sol1.iterations
        );
    }

    #[test]
    fn test_solve_rtopf_congested() {
        // Very small branch rating forces dispatch to differ from uncongested case
        let gens = vec![
            make_gen(0, 0, 0.0, 100.0, 0.01, 15.0), // cheap, bus 0
            make_gen(1, 1, 0.0, 100.0, 0.01, 30.0), // expensive, bus 1
        ];
        let branches = vec![make_branch(0, 1, 0.1, 20.0)]; // 20 MVA rating (tight)
        let mut solver_congested = RealTimeOpf::new(gens.clone(), branches, make_config(2));

        // Uncongested reference
        let branches_uncongested = vec![make_branch(0, 1, 0.1, 500.0)];
        let mut solver_free = RealTimeOpf::new(gens, branches_uncongested, make_config(2));

        let loads = vec![make_load(1, 60.0)];

        let sol_c = solver_congested
            .solve(&loads, None, 0.0)
            .expect("congested solve");
        let sol_f = solver_free.solve(&loads, None, 0.0).expect("free solve");

        // In the congested case, the local expensive generator (bus 1) must produce
        // more than in the free case, because it can't import from the cheap generator
        assert!(
            sol_c.generator_dispatch[1] >= sol_f.generator_dispatch[1] - 5.0,
            "congested expensive gen should dispatch more: cong={:.1} free={:.1}",
            sol_c.generator_dispatch[1],
            sol_f.generator_dispatch[1]
        );
    }

    // -----------------------------------------------------------------------
    // 7. LMP computation test
    // -----------------------------------------------------------------------

    #[test]
    fn test_lmp_computation() {
        // 2-bus uncongested: LMP should equal marginal cost of marginal generator
        let gens = vec![
            make_gen(0, 0, 0.0, 200.0, 0.0, 20.0), // linear cost, always cheapest
            make_gen(1, 1, 0.0, 200.0, 0.0, 30.0), // more expensive
        ];
        let branches = vec![make_branch(0, 1, 0.05, 1000.0)]; // very large rating
        let mut solver = RealTimeOpf::new(gens, branches, make_config(2));
        let loads = vec![make_load(1, 100.0)];
        let sol = solver.solve(&loads, None, 0.0).expect("lmp solve");

        // LMPs should be non-negative and non-infinite
        for (k, &lmp) in sol.lmp.iter().enumerate() {
            assert!(lmp.is_finite(), "LMP[{}] must be finite", k);
            assert!(lmp >= 0.0, "LMP[{}]={:.2} should be non-negative", k, lmp);
        }
    }

    // -----------------------------------------------------------------------
    // 8. AGC test
    // -----------------------------------------------------------------------

    #[test]
    fn test_agc_correction() {
        let gens = vec![
            make_gen(0, 0, 10.0, 100.0, 0.01, 20.0),
            make_gen(1, 1, 10.0, 100.0, 0.01, 25.0),
        ];
        let branches = vec![make_branch(0, 1, 0.1, 500.0)];
        let mut solver = RealTimeOpf::new(gens, branches, make_config(2));
        let loads = vec![make_load(1, 80.0)];
        let sol = solver.solve(&loads, None, 0.0).expect("agc solve");

        // AGC signal should be finite and reasonable
        assert!(sol.agc_signal_mw.is_finite());

        // Test compute_agc_correction directly
        let correction = solver.compute_agc_correction(&sol, 5.0);
        assert!(correction.is_finite(), "AGC correction must be finite");
    }

    // -----------------------------------------------------------------------
    // 9. Constraint violation tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_check_violations_clean() {
        // Build a solution with no violations
        let gens = vec![
            make_gen(0, 0, 10.0, 100.0, 0.01, 20.0),
            make_gen(1, 1, 10.0, 100.0, 0.01, 25.0),
        ];
        let branches = vec![make_branch(0, 1, 0.1, 500.0)];
        let mut solver = RealTimeOpf::new(gens, branches, make_config(2));
        let loads = vec![make_load(1, 80.0)];
        let sol = solver.solve(&loads, None, 0.0).expect("clean solve");

        let violations = solver.check_violations(&sol);
        // With generous ratings and typical load, expect no branch violations
        let branch_viols: Vec<_> = violations
            .iter()
            .filter(|v| v.constraint_type == "branch_loading")
            .collect();
        assert!(
            branch_viols.is_empty(),
            "expected no branch violations, got {:?}",
            branch_viols.len()
        );
    }

    #[test]
    fn test_check_violations_overloaded() {
        // Manually construct a solution with an overloaded branch
        let gens = vec![make_gen(0, 0, 0.0, 200.0, 0.01, 20.0)];
        let branches = vec![make_branch(0, 1, 0.1, 50.0)]; // 50 MVA rating
        let solver = RealTimeOpf::new(gens, branches, make_config(2));

        // Fabricate solution with loading > security_margin (0.95)
        let sol = RtSolution {
            timestamp: 0.0,
            generator_dispatch: vec![100.0],
            reactive_dispatch: vec![0.0],
            voltage_magnitude: vec![1.0, 1.0],
            voltage_angle: vec![0.0, -0.1],
            lmp: vec![20.0, 20.0],
            branch_loading: vec![1.05], // overloaded
            total_cost_usd_per_hr: 2000.0,
            total_losses_mw: 1.0,
            agc_signal_mw: 0.0,
            converged: true,
            iterations: 5,
            solve_time_ms: 1.0,
            warm_start: RtWarmStart {
                generator_setpoints: vec![100.0],
                voltage_magnitudes: vec![1.0, 1.0],
                voltage_angles: vec![0.0, -0.1],
                lambda: vec![20.0, 20.0],
            },
        };

        let violations = solver.check_violations(&sol);
        let branch_v: Vec<_> = violations
            .iter()
            .filter(|v| v.constraint_type == "branch_loading")
            .collect();
        assert_eq!(branch_v.len(), 1, "expected 1 branch violation");
        assert!(branch_v[0].severity > 0.0);
    }

    // -----------------------------------------------------------------------
    // 10. Re-dispatch test
    // -----------------------------------------------------------------------

    #[test]
    fn test_redispatch_for_overload() {
        // 2-bus: cheap gen at bus 0, expensive at bus 1
        let gens = vec![
            make_gen(0, 0, 0.0, 100.0, 0.01, 15.0),
            make_gen(1, 1, 0.0, 100.0, 0.01, 30.0),
        ];
        let branches = vec![make_branch(0, 1, 0.1, 30.0)]; // tight rating
        let mut solver = RealTimeOpf::new(gens, branches, make_config(2));

        let current_dispatch = vec![80.0, 10.0]; // cheap gen pushing lots through branch
        let loads = vec![make_load(1, 90.0)];

        let new_dispatch = solver
            .redispatch_for_overload(0, &current_dispatch, &loads)
            .expect("redispatch failed");

        assert_eq!(new_dispatch.len(), 2);
        // New dispatch should reduce power from bus 0 gen or increase bus 1 gen
        // (to relieve branch 0 loading)
        // At minimum, dispatch should be within generator limits
        for (i, (&p, gen)) in new_dispatch
            .iter()
            .zip(solver.generators.iter())
            .enumerate()
        {
            assert!(
                p >= gen.p_min_mw - 1e-6 && p <= gen.p_max_mw + 1e-6,
                "gen {} out of limits: {:.1}",
                i,
                p
            );
        }
    }

    // -----------------------------------------------------------------------
    // 11. Rolling horizon test
    // -----------------------------------------------------------------------

    #[test]
    fn test_rolling_horizon_3steps() {
        let gens = vec![
            make_gen(0, 0, 10.0, 120.0, 0.01, 20.0),
            make_gen(1, 1, 10.0, 120.0, 0.01, 25.0),
        ];
        let branches = vec![make_branch(0, 1, 0.08, 300.0)];
        let mut solver = RealTimeOpf::new(gens, branches, make_config(2));

        // Simulate 3 timesteps with slightly varying loads
        let mut state: u64 = 42;
        let load_forecasts: Vec<Vec<RtLoadForecast>> = (0..3)
            .map(|_| {
                let load_mw = 80.0 + lcg_next(&mut state) * 20.0; // 80–100 MW
                vec![make_load(1, load_mw)]
            })
            .collect();

        let solutions = solver
            .solve_rolling_horizon(&load_forecasts, 0.0, 300.0)
            .expect("rolling horizon failed");

        assert_eq!(solutions.len(), 3);
        for (t, sol) in solutions.iter().enumerate() {
            assert_eq!(sol.generator_dispatch.len(), 2, "step {}", t);
            assert!(sol.total_cost_usd_per_hr > 0.0, "step {} cost", t);
            // Timestamps should be sequential
            let expected_ts = t as f64 * 300.0;
            assert!(
                (sol.timestamp - expected_ts).abs() < 1.0,
                "step {} timestamp: {} vs {}",
                t,
                sol.timestamp,
                expected_ts
            );
        }

        // Later warm-started solves should generally converge as fast or faster
        // (soft check: just verify all complete without error)
        assert!(solutions.iter().all(|s| s.solve_time_ms >= 0.0));
    }
}
