//! Economic-Environmental Dispatch (EED) — multi-objective optimization.
//!
//! Minimizes both fuel cost AND emissions simultaneously using a Pareto front
//! approach. Supports:
//! - Valve-point effect in fuel cost
//! - Exponential term in emission model
//! - Ramp-rate constraints
//! - Epsilon-constraint / price-scalarisation method for Pareto front generation
//! - Cap-and-trade market integration
//!
//! # References
//! - Abido (2003) "Environmental/economic power dispatch using
//!   multiobjective evolutionary algorithms", IEEE Trans. Power Syst.
//! - Basu (2011) "Economic environmental dispatch using multi-objective
//!   differential evolution", Applied Soft Computing.

use crate::error::OxiGridError;
use serde::{Deserialize, Serialize};

// ─────────────────────────────────────────────────────────────────────────────
// Generator model
// ─────────────────────────────────────────────────────────────────────────────

/// Generator model for economic-environmental dispatch.
///
/// Fuel cost includes the valve-point ripple effect:
/// `F(P) = a·P² + b·P + c + |e·sin(f·(P_min − P))|`
///
/// Emission model includes an exponential term:
/// `E(P) = α·P² + β·P + γ + ζ·exp(λ·P)`
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EedGenerator {
    pub id: usize,
    pub p_min_mw: f64,
    pub p_max_mw: f64,
    /// Quadratic cost coefficient ($/MW²h)
    pub cost_a: f64,
    /// Linear cost coefficient ($/MWh)
    pub cost_b: f64,
    /// No-load cost ($/h)
    pub cost_c: f64,
    /// Valve-point ripple coefficient
    pub cost_e: f64,
    /// Valve-point ripple frequency
    pub cost_f: f64,
    /// Emission quadratic coefficient (tons/MW²h)
    pub emission_alpha: f64,
    /// Emission linear coefficient (tons/MWh)
    pub emission_beta: f64,
    /// Emission constant term (tons/h)
    pub emission_gamma: f64,
    /// Emission exponential coefficient
    pub emission_zeta: f64,
    /// Emission exponential rate
    pub emission_lambda: f64,
    /// Ramp-up limit (MW/h)
    pub ramp_up_mw_per_h: f64,
    /// Ramp-down limit (MW/h)
    pub ramp_down_mw_per_h: f64,
    /// Current dispatch level (MW)
    pub p_current_mw: f64,
}

impl EedGenerator {
    /// Create a new generator with sensible defaults.
    pub fn new(id: usize, p_min_mw: f64, p_max_mw: f64) -> Self {
        let mid = (p_min_mw + p_max_mw) * 0.5;
        Self {
            id,
            p_min_mw,
            p_max_mw,
            cost_a: 0.01,
            cost_b: 2.0,
            cost_c: 10.0,
            cost_e: 0.0,
            cost_f: 1.0,
            emission_alpha: 0.0001,
            emission_beta: 0.01,
            emission_gamma: 0.1,
            emission_zeta: 0.0,
            emission_lambda: 0.01,
            ramp_up_mw_per_h: 999.0,
            ramp_down_mw_per_h: 999.0,
            p_current_mw: mid,
        }
    }

    /// Fuel cost with valve-point effect ($/h).
    ///
    /// `F(P) = a·P² + b·P + c + |e·sin(f·(P_min − P))|`
    #[inline]
    pub fn fuel_cost(&self, p_mw: f64) -> f64 {
        let base = self.cost_a * p_mw * p_mw + self.cost_b * p_mw + self.cost_c;
        let ripple = (self.cost_e * (self.cost_f * (self.p_min_mw - p_mw)).sin()).abs();
        base + ripple
    }

    /// Emission rate (tons/h).
    ///
    /// `E(P) = α·P² + β·P + γ + ζ·exp(λ·P)`
    #[inline]
    pub fn emission(&self, p_mw: f64) -> f64 {
        self.emission_alpha * p_mw * p_mw
            + self.emission_beta * p_mw
            + self.emission_gamma
            + self.emission_zeta * (self.emission_lambda * p_mw).exp()
    }

    /// Combined cost: `F(P) + emission_price·E(P)`
    #[inline]
    pub fn combined_cost(&self, p_mw: f64, emission_price_per_ton: f64) -> f64 {
        self.fuel_cost(p_mw) + emission_price_per_ton * self.emission(p_mw)
    }

    /// Incremental fuel cost: `dF/dP = 2·a·P + b + e·f·cos(f·(P_min − P))`
    #[inline]
    pub fn incremental_cost(&self, p_mw: f64) -> f64 {
        2.0 * self.cost_a * p_mw
            + self.cost_b
            + self.cost_e * self.cost_f * (self.cost_f * (self.p_min_mw - p_mw)).cos()
    }

    /// Incremental emission: `dE/dP = 2·α·P + β + ζ·λ·exp(λ·P)`
    #[inline]
    pub fn incremental_emission(&self, p_mw: f64) -> f64 {
        2.0 * self.emission_alpha * p_mw
            + self.emission_beta
            + self.emission_zeta * self.emission_lambda * (self.emission_lambda * p_mw).exp()
    }

    /// Combined effective quadratic coefficient for lambda iteration.
    #[inline]
    fn combined_a(&self, emission_price: f64) -> f64 {
        self.cost_a + emission_price * self.emission_alpha
    }

    /// Combined effective linear coefficient for lambda iteration.
    #[inline]
    fn combined_b(&self, emission_price: f64) -> f64 {
        self.cost_b + emission_price * self.emission_beta
    }

    /// Ramp-constrained dispatch bounds given current output.
    #[inline]
    fn ramp_bounds(&self) -> (f64, f64) {
        let lo = self
            .p_min_mw
            .max(self.p_current_mw - self.ramp_down_mw_per_h);
        let hi = self.p_max_mw.min(self.p_current_mw + self.ramp_up_mw_per_h);
        (lo, hi)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Dispatch result
// ─────────────────────────────────────────────────────────────────────────────

/// Result of an EED dispatch solve.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EedDispatch {
    /// Per-generator output (MW)
    pub generation_mw: Vec<f64>,
    /// Sum of all generation (MW)
    pub total_generation_mw: f64,
    /// Total fuel cost ($/h)
    pub total_cost_per_h: f64,
    /// Total emission rate (tons/h)
    pub total_emission_per_h: f64,
    /// Generation − load mismatch (MW); should be ≈ 0
    pub power_balance_mw: f64,
}

impl EedDispatch {
    /// Compute metrics from a raw dispatch vector.
    pub fn compute_metrics(generators: &[EedGenerator], dispatch: &[f64]) -> Self {
        let total_gen: f64 = dispatch.iter().sum();
        let total_cost: f64 = generators
            .iter()
            .zip(dispatch.iter())
            .map(|(g, &p)| g.fuel_cost(p))
            .sum();
        let total_emission: f64 = generators
            .iter()
            .zip(dispatch.iter())
            .map(|(g, &p)| g.emission(p))
            .sum();
        Self {
            generation_mw: dispatch.to_vec(),
            total_generation_mw: total_gen,
            total_cost_per_h: total_cost,
            total_emission_per_h: total_emission,
            power_balance_mw: total_gen, // caller may adjust
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Pareto front structures
// ─────────────────────────────────────────────────────────────────────────────

/// A single Pareto-optimal operating point.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ParetoSolution {
    /// Total fuel cost ($/h)
    pub cost: f64,
    /// Total emission rate (tons/h)
    pub emission: f64,
    /// Per-generator dispatch (MW)
    pub dispatch: Vec<f64>,
    /// Scalarisation weight (emission price) used to obtain this point
    pub weight: f64,
}

/// Full Pareto front result.
#[derive(Debug, Clone)]
pub struct ParetoResult {
    /// Non-dominated solutions sorted by cost (ascending)
    pub solutions: Vec<ParetoSolution>,
    /// Index of the knee-point solution
    pub knee_point_idx: usize,
    /// Index of the minimum-cost solution
    pub min_cost_idx: usize,
    /// Index of the minimum-emission solution
    pub min_emission_idx: usize,
    /// Range of cost values across the front ($/h)
    pub pareto_spread: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Carbon price scenario
// ─────────────────────────────────────────────────────────────────────────────

/// Carbon-price scenario for emission valuation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CarbonPriceScenario {
    pub carbon_price_eur_per_ton: f64,
    /// Hourly emission cap (tons/h); `None` = no cap
    pub cap_tons_per_h: Option<f64>,
}

// ─────────────────────────────────────────────────────────────────────────────
// Epsilon-constraint solver
// ─────────────────────────────────────────────────────────────────────────────

/// Multi-objective EED solver using epsilon-constraint / price-scalarisation.
///
/// The solver finds the Pareto front by solving the combined dispatch with a
/// range of emission prices and then filtering dominated solutions.
#[derive(Debug, Clone)]
pub struct EpsilonConstraintSolver {
    pub generators: Vec<EedGenerator>,
    /// Number of Pareto-front sweep points (default 20)
    pub n_points: usize,
    /// Lambda-iteration convergence tolerance (default 1e-6)
    pub lambda_iter_tol: f64,
    /// Maximum lambda-iteration bisection steps (default 1000)
    pub max_iter: usize,
}

impl EpsilonConstraintSolver {
    /// Create a new solver.
    pub fn new(generators: Vec<EedGenerator>, n_points: usize) -> Self {
        Self {
            generators,
            n_points,
            lambda_iter_tol: 1e-6,
            max_iter: 1000,
        }
    }

    // ── public solve methods ──────────────────────────────────────────────────

    /// Economic dispatch only (minimise fuel cost, ignore emissions).
    pub fn solve_economic(&self, load_mw: f64) -> Result<EedDispatch, OxiGridError> {
        let dispatch = self.lambda_iteration(load_mw, "cost", 0.0)?;
        let mut result = EedDispatch::compute_metrics(&self.generators, &dispatch);
        result.power_balance_mw = result.total_generation_mw - load_mw;
        Ok(result)
    }

    /// Emission-minimising dispatch (ignore fuel cost).
    pub fn solve_emission(&self, load_mw: f64) -> Result<EedDispatch, OxiGridError> {
        let dispatch = self.lambda_iteration(load_mw, "emission", 0.0)?;
        let mut result = EedDispatch::compute_metrics(&self.generators, &dispatch);
        result.power_balance_mw = result.total_generation_mw - load_mw;
        Ok(result)
    }

    /// Combined (scalarised) dispatch with a given emission price ($/ton).
    pub fn solve_combined(
        &self,
        load_mw: f64,
        emission_price: f64,
    ) -> Result<EedDispatch, OxiGridError> {
        let dispatch = self.lambda_iteration(load_mw, "combined", emission_price)?;
        let mut result = EedDispatch::compute_metrics(&self.generators, &dispatch);
        result.power_balance_mw = result.total_generation_mw - load_mw;
        Ok(result)
    }

    /// Generate Pareto front via epsilon-constraint / price-scalarisation sweep.
    ///
    /// # Algorithm
    /// 1. Solve pure economic dispatch (E_max) and pure emission dispatch (E_min).
    /// 2. Sweep `n_points` emission prices on a log scale plus epsilon-penalty prices.
    /// 3. Collect all solutions, filter non-dominated ones, sort by cost.
    /// 4. Locate knee point, min-cost, min-emission indices.
    pub fn generate_pareto_front(&self, load_mw: f64) -> Result<ParetoResult, OxiGridError> {
        let eco_disp = self.solve_economic(load_mw)?;
        let em_disp = self.solve_emission(load_mw)?;

        let e_max = eco_disp.total_emission_per_h;
        let e_min = em_disp.total_emission_per_h;

        let mut all: Vec<ParetoSolution> = Vec::with_capacity(self.n_points * 2 + 2);

        // Include anchor solutions
        all.push(ParetoSolution {
            cost: eco_disp.total_cost_per_h,
            emission: eco_disp.total_emission_per_h,
            dispatch: eco_disp.generation_mw.clone(),
            weight: 0.0,
        });
        all.push(ParetoSolution {
            cost: em_disp.total_cost_per_h,
            emission: em_disp.total_emission_per_h,
            dispatch: em_disp.generation_mw.clone(),
            weight: f64::INFINITY,
        });

        let n = self.n_points.max(2);

        // Sweep emission prices on log scale (1e-2 to 1e6)
        for k in 0..n {
            let t = k as f64 / (n - 1) as f64;
            let emission_price = 0.01_f64 * (1e8_f64.ln() * t).exp();

            if let Ok(sol) = self.solve_combined(load_mw, emission_price) {
                all.push(ParetoSolution {
                    cost: sol.total_cost_per_h,
                    emission: sol.total_emission_per_h,
                    dispatch: sol.generation_mw,
                    weight: emission_price,
                });
            }

            // Epsilon-constraint: penalty aimed at epsilon ∈ [E_min, E_max]
            if e_max > e_min + 1e-9 {
                let epsilon = e_min + (e_max - e_min) * t;
                let penalty = 1_000.0 * (e_max - e_min + 1.0) / (epsilon - e_min + 1e-6).max(1e-9);
                if let Ok(d) = self.solve_combined(load_mw, penalty) {
                    all.push(ParetoSolution {
                        cost: d.total_cost_per_h,
                        emission: d.total_emission_per_h,
                        dispatch: d.generation_mw,
                        weight: penalty,
                    });
                }
            }
        }

        // Filter non-dominated solutions
        let nd_indices = Self::filter_pareto(&all);
        let mut nd_solutions: Vec<ParetoSolution> =
            nd_indices.iter().map(|&i| all[i].clone()).collect();

        // Sort by cost ascending
        nd_solutions.sort_by(|a, b| {
            a.cost
                .partial_cmp(&b.cost)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        if nd_solutions.is_empty() {
            // Fallback: return anchor solutions
            nd_solutions = vec![all[0].clone(), all[1].clone()];
            nd_solutions.sort_by(|a, b| {
                a.cost
                    .partial_cmp(&b.cost)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
        }

        let min_cost_idx = nd_solutions
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| {
                a.cost
                    .partial_cmp(&b.cost)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(i, _)| i)
            .unwrap_or(0);

        let min_emission_idx = nd_solutions
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| {
                a.emission
                    .partial_cmp(&b.emission)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(i, _)| i)
            .unwrap_or(0);

        let knee_point_idx = Self::find_knee_point(&nd_solutions);

        let costs: Vec<f64> = nd_solutions.iter().map(|s| s.cost).collect();
        let c_max = costs.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let c_min_val = costs.iter().cloned().fold(f64::INFINITY, f64::min);
        let pareto_spread = (c_max - c_min_val).max(0.0);

        Ok(ParetoResult {
            solutions: nd_solutions,
            knee_point_idx,
            min_cost_idx,
            min_emission_idx,
            pareto_spread,
        })
    }

    // ── internal helpers ──────────────────────────────────────────────────────

    /// Lambda-iteration (bisection) to find the optimal dispatch vector.
    ///
    /// `cost_type` selects the objective incremental curve:
    /// - `"cost"` — pure economic dispatch
    /// - `"emission"` — pure emission minimisation
    /// - `"combined"` — weighted sum with `emission_price`
    fn lambda_iteration(
        &self,
        load_mw: f64,
        cost_type: &str,
        emission_price: f64,
    ) -> Result<Vec<f64>, OxiGridError> {
        if self.generators.is_empty() {
            return Err(OxiGridError::InvalidParameter(
                "no generators provided".into(),
            ));
        }

        // Feasibility bounds (respecting ramp constraints)
        let p_max_total: f64 = self.generators.iter().map(|g| g.ramp_bounds().1).sum();
        let p_min_total: f64 = self.generators.iter().map(|g| g.ramp_bounds().0).sum();

        if load_mw > p_max_total + 1e-6 {
            return Err(OxiGridError::InvalidParameter(format!(
                "load {:.2} MW exceeds maximum generation capacity {:.2} MW",
                load_mw, p_max_total
            )));
        }
        if load_mw < p_min_total - 1e-6 {
            return Err(OxiGridError::InvalidParameter(format!(
                "load {:.2} MW is below minimum generation {:.2} MW",
                load_mw, p_min_total
            )));
        }

        // Effective quadratic and linear coefficients for lambda sweep
        let coeffs: Vec<(f64, f64)> = self
            .generators
            .iter()
            .map(|g| match cost_type {
                "emission" => (g.emission_alpha, g.emission_beta),
                "combined" => (g.combined_a(emission_price), g.combined_b(emission_price)),
                _ => (g.cost_a, g.cost_b),
            })
            .collect();

        // Bisection bounds for λ
        let lambda_lo_init = coeffs.iter().map(|(_, b)| *b).fold(f64::INFINITY, f64::min);
        let lambda_hi_init = coeffs
            .iter()
            .zip(self.generators.iter())
            .map(|((a, b), g)| {
                let (_, hi) = g.ramp_bounds();
                b + 2.0 * a * hi
            })
            .fold(f64::NEG_INFINITY, f64::max);

        let mut lo = lambda_lo_init - 100.0;
        let mut hi = lambda_hi_init + 100.0;

        let dispatch_at = |lam: f64| -> Vec<f64> {
            coeffs
                .iter()
                .zip(self.generators.iter())
                .map(|((a, b), g)| {
                    let (ramp_lo, ramp_hi) = g.ramp_bounds();
                    if *a < 1e-15 {
                        if lam >= *b {
                            ramp_hi
                        } else {
                            ramp_lo
                        }
                    } else {
                        let p = (lam - b) / (2.0 * a);
                        p.clamp(ramp_lo, ramp_hi)
                    }
                })
                .collect()
        };

        let total_at = |lam: f64| -> f64 { dispatch_at(lam).iter().sum() };

        // Expand range until it brackets load_mw
        let mut expand_iters = 0_usize;
        while total_at(lo) > load_mw && expand_iters < 100 {
            lo -= (hi - lo).abs().max(1.0) * 2.0;
            expand_iters += 1;
        }
        expand_iters = 0;
        while total_at(hi) < load_mw && expand_iters < 100 {
            hi += (hi - lo).abs().max(1.0) * 2.0;
            expand_iters += 1;
        }

        // Bisection
        let mut dispatch = dispatch_at((lo + hi) * 0.5);
        for _ in 0..self.max_iter {
            let mid = (lo + hi) * 0.5;
            let total: f64 = dispatch_at(mid).iter().sum();
            if (total - load_mw).abs() < self.lambda_iter_tol {
                dispatch = dispatch_at(mid);
                break;
            }
            if total < load_mw {
                lo = mid;
            } else {
                hi = mid;
            }
            dispatch = dispatch_at((lo + hi) * 0.5);
        }

        // Proportional balance correction for residual imbalance
        let current_total: f64 = dispatch.iter().sum();
        let residual = load_mw - current_total;
        if residual.abs() > self.lambda_iter_tol {
            let mut headroom_total = 0.0_f64;
            let mut adjustable: Vec<(usize, f64)> = Vec::new();
            for (i, g) in self.generators.iter().enumerate() {
                let (ramp_lo, ramp_hi) = g.ramp_bounds();
                let room = if residual > 0.0 {
                    ramp_hi - dispatch[i]
                } else {
                    dispatch[i] - ramp_lo
                };
                if room > 1e-9 {
                    headroom_total += room;
                    adjustable.push((i, room));
                }
            }
            if headroom_total > 1e-9 {
                for (i, room) in adjustable {
                    let g = &self.generators[i];
                    let (ramp_lo, ramp_hi) = g.ramp_bounds();
                    let share = residual * room / headroom_total;
                    dispatch[i] = (dispatch[i] + share).clamp(ramp_lo, ramp_hi);
                }
            }
        }

        Ok(dispatch)
    }

    /// Check whether the dispatch vector meets the power balance within `tol`.
    pub fn check_balance(dispatch: &[f64], load_mw: f64, tol: f64) -> bool {
        let total: f64 = dispatch.iter().sum();
        (total - load_mw).abs() <= tol
    }

    /// Return indices of Pareto non-dominated solutions.
    ///
    /// Solution `i` is dominated if there exists `j ≠ i` such that
    /// `cost_j ≤ cost_i` AND `emission_j ≤ emission_i` (at least one strict).
    pub fn filter_pareto(solutions: &[ParetoSolution]) -> Vec<usize> {
        let n = solutions.len();
        let mut dominated = vec![false; n];
        for i in 0..n {
            if dominated[i] {
                continue;
            }
            for j in 0..n {
                if i == j || dominated[j] {
                    continue;
                }
                let cost_le = solutions[j].cost <= solutions[i].cost + 1e-9;
                let em_le = solutions[j].emission <= solutions[i].emission + 1e-9;
                let cost_lt = solutions[j].cost < solutions[i].cost - 1e-9;
                let em_lt = solutions[j].emission < solutions[i].emission - 1e-9;
                if cost_le && em_le && (cost_lt || em_lt) {
                    dominated[i] = true;
                    break;
                }
            }
        }
        (0..n).filter(|&i| !dominated[i]).collect()
    }

    /// Find the knee point: maximum normalised perpendicular distance from
    /// the anti-diagonal line in (cost, emission) space.
    ///
    /// Solutions are normalised to \[0,1\] in both objectives.  The knee is the
    /// point furthest from the line connecting (0,1) to (1,0) (i.e. the
    /// trade-off boundary between pure-cost and pure-emission optima).
    pub fn find_knee_point(solutions: &[ParetoSolution]) -> usize {
        if solutions.len() <= 1 {
            return 0;
        }

        let costs: Vec<f64> = solutions.iter().map(|s| s.cost).collect();
        let emissions: Vec<f64> = solutions.iter().map(|s| s.emission).collect();

        let c_min = costs.iter().cloned().fold(f64::INFINITY, f64::min);
        let c_max = costs.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let e_min = emissions.iter().cloned().fold(f64::INFINITY, f64::min);
        let e_max = emissions.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

        let c_range = (c_max - c_min).max(1e-12);
        let e_range = (e_max - e_min).max(1e-12);

        let mut best_idx = 0;
        let mut best_dist = f64::NEG_INFINITY;

        for (i, s) in solutions.iter().enumerate() {
            let x = (s.cost - c_min) / c_range;
            let y = (s.emission - e_min) / e_range;
            // Distance from (x,y) to anti-diagonal x + y = 1:
            // d = |x + y - 1| / sqrt(2)
            let dist = (x + y - 1.0).abs();
            if dist > best_dist {
                best_dist = dist;
                best_idx = i;
            }
        }
        best_idx
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Cap-and-trade dispatch
// ─────────────────────────────────────────────────────────────────────────────

/// Dispatch under a cap-and-trade emission market.
///
/// Excess emissions require purchasing allowances at `allowance_price`;
/// shortfalls generate revenue (negative `allowance_cost`).
#[derive(Debug, Clone)]
pub struct CapAndTradeDispatch {
    pub solver: EpsilonConstraintSolver,
    /// Hourly emission cap (tons/h)
    pub emission_cap: f64,
    /// Allowance price ($/ton)
    pub allowance_price: f64,
}

impl CapAndTradeDispatch {
    /// Create a new cap-and-trade dispatcher.
    pub fn new(generators: Vec<EedGenerator>, emission_cap: f64, allowance_price: f64) -> Self {
        Self {
            solver: EpsilonConstraintSolver::new(generators, 20),
            emission_cap,
            allowance_price,
        }
    }

    /// Dispatch under cap-and-trade rules.
    ///
    /// Returns `(dispatch, allowance_cost_or_revenue)`.
    /// Positive `allowance_cost` means the system must purchase allowances;
    /// negative means surplus allowances are sold as revenue.
    pub fn dispatch(&self, load_mw: f64) -> Result<(EedDispatch, f64), OxiGridError> {
        let result = self.solver.solve_combined(load_mw, self.allowance_price)?;
        let allowance_cost =
            (result.total_emission_per_h - self.emission_cap) * self.allowance_price;
        Ok((result, allowance_cost))
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_gen(id: usize, p_min: f64, p_max: f64, a: f64, b: f64, c: f64) -> EedGenerator {
        let mut g = EedGenerator::new(id, p_min, p_max);
        g.cost_a = a;
        g.cost_b = b;
        g.cost_c = c;
        g
    }

    // ── Generator model tests ─────────────────────────────────────────────────

    #[test]
    fn test_generator_fuel_cost_quadratic() {
        let mut g = EedGenerator::new(0, 10.0, 100.0);
        g.cost_a = 0.01;
        g.cost_b = 2.0;
        g.cost_c = 5.0;
        g.cost_e = 0.0;
        let cost = g.fuel_cost(50.0);
        // 0.01*2500 + 2*50 + 5 = 25 + 100 + 5 = 130
        assert!((cost - 130.0).abs() < 1e-9, "expected 130, got {}", cost);
    }

    #[test]
    fn test_generator_fuel_cost_valve_point() {
        let mut g = EedGenerator::new(0, 10.0, 100.0);
        g.cost_a = 0.01;
        g.cost_b = 2.0;
        g.cost_c = 5.0;
        g.cost_e = 10.0;
        g.cost_f = 0.5;
        let base: f64 = 0.01 * 2500.0 + 2.0 * 50.0 + 5.0;
        let ripple: f64 = (10.0_f64 * (0.5_f64 * (10.0_f64 - 50.0_f64)).sin()).abs();
        let cost = g.fuel_cost(50.0);
        assert!(
            (cost - base - ripple).abs() < 1e-9,
            "expected {:.6}, got {:.6}",
            base + ripple,
            cost
        );
        assert!(cost >= base, "valve-point must add non-negative cost");
    }

    #[test]
    fn test_generator_emission_model() {
        let mut g = EedGenerator::new(0, 10.0, 100.0);
        g.emission_alpha = 0.0001;
        g.emission_beta = 0.01;
        g.emission_gamma = 0.1;
        g.emission_zeta = 0.001;
        g.emission_lambda = 0.02;
        let p = 50.0;
        let expected = 0.0001 * 2500.0 + 0.01 * 50.0 + 0.1 + 0.001 * (0.02 * 50.0_f64).exp();
        let actual = g.emission(p);
        assert!(
            (actual - expected).abs() < 1e-9,
            "expected {:.9}, got {:.9}",
            expected,
            actual
        );
    }

    #[test]
    fn test_generator_combined_cost() {
        let mut g = EedGenerator::new(0, 10.0, 100.0);
        g.cost_a = 0.01;
        g.cost_b = 2.0;
        g.cost_c = 5.0;
        g.cost_e = 0.0;
        g.emission_alpha = 0.0001;
        g.emission_beta = 0.01;
        g.emission_gamma = 0.1;
        g.emission_zeta = 0.0;
        let price = 20.0;
        let p = 60.0;
        let fc = g.fuel_cost(p);
        let em = g.emission(p);
        let combined = g.combined_cost(p, price);
        assert!(
            (combined - fc - price * em).abs() < 1e-9,
            "combined mismatch: {} vs {}",
            combined,
            fc + price * em
        );
    }

    #[test]
    fn test_incremental_cost() {
        let mut g = EedGenerator::new(0, 10.0, 100.0);
        g.cost_a = 0.01;
        g.cost_b = 2.0;
        g.cost_e = 0.0;
        let ic = g.incremental_cost(50.0);
        let expected = 2.0 * 0.01 * 50.0 + 2.0;
        assert!(
            (ic - expected).abs() < 1e-9,
            "expected {}, got {}",
            expected,
            ic
        );
    }

    #[test]
    fn test_incremental_emission() {
        let mut g = EedGenerator::new(0, 10.0, 100.0);
        g.emission_alpha = 0.0001;
        g.emission_beta = 0.01;
        g.emission_zeta = 0.0;
        let ie = g.incremental_emission(50.0);
        let expected = 2.0 * 0.0001 * 50.0 + 0.01;
        assert!(
            (ie - expected).abs() < 1e-9,
            "expected {}, got {}",
            expected,
            ie
        );
    }

    // ── Economic dispatch tests ───────────────────────────────────────────────

    #[test]
    fn test_economic_dispatch_2_generators() {
        let g1 = make_gen(0, 10.0, 100.0, 0.01, 2.0, 5.0);
        let g2 = make_gen(1, 10.0, 100.0, 0.01, 2.0, 5.0);
        let solver = EpsilonConstraintSolver::new(vec![g1, g2], 10);
        let result = solver.solve_economic(100.0).expect("dispatch failed");
        assert!(
            (result.generation_mw[0] - result.generation_mw[1]).abs() < 1.0,
            "identical gens should produce equal output: {:?}",
            result.generation_mw
        );
        assert!(
            (result.total_generation_mw - 100.0).abs() < 0.1,
            "power balance: {}",
            result.total_generation_mw
        );
    }

    #[test]
    fn test_economic_dispatch_different_cost() {
        // g0 has lower b (marginal cost) → gets more load
        let g0 = make_gen(0, 10.0, 150.0, 0.005, 1.5, 5.0);
        let g1 = make_gen(1, 10.0, 150.0, 0.02, 3.0, 5.0);
        let solver = EpsilonConstraintSolver::new(vec![g0, g1], 10);
        let result = solver.solve_economic(150.0).expect("dispatch failed");
        assert!(
            result.generation_mw[0] > result.generation_mw[1],
            "cheaper gen should produce more: {:?}",
            result.generation_mw
        );
    }

    #[test]
    fn test_economic_dispatch_ramp_limited() {
        let mut g0 = make_gen(0, 10.0, 100.0, 0.01, 2.0, 5.0);
        g0.p_current_mw = 50.0;
        g0.ramp_up_mw_per_h = 10.0; // max → 60 MW
        let mut g1 = make_gen(1, 10.0, 100.0, 0.01, 2.0, 5.0);
        g1.p_current_mw = 50.0;
        g1.ramp_up_mw_per_h = 50.0;
        let solver = EpsilonConstraintSolver::new(vec![g0, g1], 10);
        let result = solver.solve_economic(110.0).expect("dispatch failed");
        assert!(
            result.generation_mw[0] <= 60.0 + 1e-6,
            "ramp violated: {}",
            result.generation_mw[0]
        );
    }

    #[test]
    fn test_economic_dispatch_feasible() {
        let g0 = make_gen(0, 20.0, 80.0, 0.01, 2.0, 5.0);
        let g1 = make_gen(1, 20.0, 80.0, 0.01, 2.0, 5.0);
        let solver = EpsilonConstraintSolver::new(vec![g0, g1], 10);
        // Infeasible: load > sum p_max = 160
        let err = solver.solve_economic(200.0);
        assert!(err.is_err(), "should fail for infeasible load");
    }

    // ── Emission dispatch tests ───────────────────────────────────────────────

    #[test]
    fn test_emission_dispatch_2_generators() {
        let mut g0 = make_gen(0, 10.0, 100.0, 0.01, 2.0, 5.0);
        g0.emission_alpha = 0.0001;
        g0.emission_beta = 0.01;
        g0.emission_gamma = 0.1;
        let mut g1 = make_gen(1, 10.0, 100.0, 0.01, 2.0, 5.0);
        g1.emission_alpha = 0.001;
        g1.emission_beta = 0.05;
        g1.emission_gamma = 0.2;
        let solver = EpsilonConstraintSolver::new(vec![g0, g1], 10);
        let result = solver
            .solve_emission(100.0)
            .expect("emission dispatch failed");
        assert!(
            result.generation_mw[0] >= result.generation_mw[1] - 5.0,
            "cleaner gen should produce more: {:?}",
            result.generation_mw
        );
        assert!(
            (result.total_generation_mw - 100.0).abs() < 0.5,
            "power balance: {}",
            result.total_generation_mw
        );
    }

    // ── Combined dispatch tests ───────────────────────────────────────────────

    #[test]
    fn test_combined_dispatch_zero_price() {
        let g0 = make_gen(0, 10.0, 100.0, 0.01, 2.0, 5.0);
        let g1 = make_gen(1, 10.0, 100.0, 0.02, 3.0, 5.0);
        let solver = EpsilonConstraintSolver::new(vec![g0, g1], 10);
        let eco = solver.solve_economic(100.0).expect("eco failed");
        let combined = solver.solve_combined(100.0, 0.0).expect("combined failed");
        for (a, b) in eco.generation_mw.iter().zip(combined.generation_mw.iter()) {
            assert!(
                (*a - *b).abs() < 0.5_f64,
                "zero price should match eco: {} vs {}",
                a,
                b
            );
        }
    }

    #[test]
    fn test_combined_dispatch_high_price() {
        let mut g0 = make_gen(0, 10.0, 100.0, 0.01, 2.0, 5.0);
        g0.emission_alpha = 0.0001;
        g0.emission_beta = 0.01;
        g0.emission_gamma = 0.1;
        let mut g1 = make_gen(1, 10.0, 100.0, 0.01, 2.0, 5.0);
        g1.emission_alpha = 0.001;
        g1.emission_beta = 0.1;
        g1.emission_gamma = 0.2;
        let solver = EpsilonConstraintSolver::new(vec![g0, g1], 10);
        let em = solver.solve_emission(100.0).expect("em failed");
        let combined = solver
            .solve_combined(100.0, 10000.0)
            .expect("combined failed");
        let em_total = em.total_emission_per_h;
        let comb_total = combined.total_emission_per_h;
        assert!(
            comb_total <= em_total + 1.0,
            "high price should approach emission min: em={} combined={}",
            em_total,
            comb_total
        );
    }

    // ── Pareto front tests ────────────────────────────────────────────────────

    #[test]
    fn test_pareto_front_generation_2gen() {
        let mut g0 = make_gen(0, 10.0, 100.0, 0.01, 2.0, 5.0);
        g0.emission_alpha = 0.0001;
        g0.emission_beta = 0.01;
        let mut g1 = make_gen(1, 10.0, 100.0, 0.005, 1.5, 5.0);
        g1.emission_alpha = 0.002;
        g1.emission_beta = 0.05;
        let solver = EpsilonConstraintSolver::new(vec![g0, g1], 5);
        let result = solver.generate_pareto_front(100.0).expect("pareto failed");
        assert!(
            !result.solutions.is_empty(),
            "Pareto front must be non-empty"
        );
        let min_cost = result.solutions[result.min_cost_idx].cost;
        for s in &result.solutions {
            assert!(s.cost >= min_cost - 1e-6, "min_cost violated");
        }
    }

    #[test]
    fn test_pareto_front_size() {
        let g0 = make_gen(0, 10.0, 100.0, 0.01, 2.0, 5.0);
        let g1 = make_gen(1, 10.0, 100.0, 0.02, 3.0, 5.0);
        let n = 8;
        let solver = EpsilonConstraintSolver::new(vec![g0, g1], n);
        let result = solver.generate_pareto_front(100.0).expect("pareto failed");
        assert!(
            !result.solutions.is_empty(),
            "Pareto front must be non-empty"
        );
        // Non-dominated set ≤ n + 2 (sweep + 2 anchors; epsilon sweep doubles sweep)
        assert!(
            result.solutions.len() <= (n * 2) + 2,
            "too many solutions: {}",
            result.solutions.len()
        );
    }

    #[test]
    fn test_pareto_non_dominated_filtering() {
        let solutions = vec![
            ParetoSolution {
                cost: 100.0,
                emission: 5.0,
                dispatch: vec![],
                weight: 0.0,
            },
            ParetoSolution {
                cost: 120.0,
                emission: 4.0,
                dispatch: vec![],
                weight: 0.0,
            },
            ParetoSolution {
                cost: 110.0,
                emission: 6.0,
                dispatch: vec![],
                weight: 0.0,
            }, // dominated by solution 0
        ];
        let indices = EpsilonConstraintSolver::filter_pareto(&solutions);
        assert!(
            !indices.contains(&2),
            "dominated solution should be filtered: {:?}",
            indices
        );
        assert!(indices.contains(&0), "solution 0 should survive");
        assert!(indices.contains(&1), "solution 1 should survive");
    }

    #[test]
    fn test_pareto_knee_point() {
        let solutions = vec![
            ParetoSolution {
                cost: 100.0,
                emission: 1.0,
                dispatch: vec![],
                weight: 0.0,
            },
            ParetoSolution {
                cost: 110.0,
                emission: 0.7,
                dispatch: vec![],
                weight: 0.0,
            },
            ParetoSolution {
                cost: 130.0,
                emission: 0.5,
                dispatch: vec![],
                weight: 0.0,
            },
            ParetoSolution {
                cost: 200.0,
                emission: 0.3,
                dispatch: vec![],
                weight: 0.0,
            },
        ];
        let knee = EpsilonConstraintSolver::find_knee_point(&solutions);
        assert!(knee < solutions.len(), "knee index out of range: {}", knee);
    }

    #[test]
    fn test_min_cost_solution_cost() {
        let mut g0 = make_gen(0, 10.0, 100.0, 0.01, 2.0, 5.0);
        g0.emission_alpha = 0.0002;
        let mut g1 = make_gen(1, 10.0, 100.0, 0.008, 1.8, 5.0);
        g1.emission_alpha = 0.0005;
        let solver = EpsilonConstraintSolver::new(vec![g0, g1], 6);
        let result = solver.generate_pareto_front(100.0).expect("pareto failed");
        let min_cost = result.solutions[result.min_cost_idx].cost;
        for s in &result.solutions {
            assert!(
                s.cost >= min_cost - 1e-3,
                "min_cost violated: {} < {}",
                s.cost,
                min_cost
            );
        }
    }

    #[test]
    fn test_min_emission_solution() {
        let mut g0 = make_gen(0, 10.0, 100.0, 0.01, 2.0, 5.0);
        g0.emission_alpha = 0.0002;
        let mut g1 = make_gen(1, 10.0, 100.0, 0.008, 1.8, 5.0);
        g1.emission_alpha = 0.0005;
        let solver = EpsilonConstraintSolver::new(vec![g0, g1], 6);
        let result = solver.generate_pareto_front(100.0).expect("pareto failed");
        let min_em = result.solutions[result.min_emission_idx].emission;
        for s in &result.solutions {
            assert!(
                s.emission >= min_em - 1e-3,
                "min_emission violated: {} < {}",
                s.emission,
                min_em
            );
        }
    }

    #[test]
    fn test_power_balance_check() {
        let dispatch = vec![50.0, 50.0];
        assert!(
            EpsilonConstraintSolver::check_balance(&dispatch, 100.0, 0.1),
            "should balance"
        );
        assert!(
            !EpsilonConstraintSolver::check_balance(&dispatch, 101.0, 0.1),
            "should not balance"
        );
    }

    // ── Cap-and-trade tests ───────────────────────────────────────────────────

    #[test]
    fn test_cap_and_trade_within_cap() {
        let mut g0 = make_gen(0, 10.0, 100.0, 0.01, 2.0, 5.0);
        g0.emission_alpha = 0.0001;
        g0.emission_beta = 0.01;
        g0.emission_gamma = 0.05;
        let mut g1 = make_gen(1, 10.0, 100.0, 0.01, 2.0, 5.0);
        g1.emission_alpha = 0.0001;
        g1.emission_beta = 0.01;
        g1.emission_gamma = 0.05;
        let cap = CapAndTradeDispatch::new(vec![g0, g1], 100.0, 15.0); // generous cap
        let (dispatch, allowance_cost) = cap.dispatch(100.0).expect("cap dispatch failed");
        assert!(
            (dispatch.total_generation_mw - 100.0).abs() < 0.5,
            "power balance: {}",
            dispatch.total_generation_mw
        );
        assert!(
            allowance_cost < 0.0 || dispatch.total_emission_per_h > 99.0,
            "under cap should yield revenue or be at cap: cost={}",
            allowance_cost
        );
    }

    #[test]
    fn test_cap_and_trade_exceeds_cap() {
        let mut g0 = make_gen(0, 50.0, 200.0, 0.01, 2.0, 5.0);
        g0.emission_alpha = 0.001;
        g0.emission_beta = 0.1;
        g0.emission_gamma = 1.0;
        g0.p_current_mw = 125.0;
        let mut g1 = make_gen(1, 50.0, 200.0, 0.01, 2.0, 5.0);
        g1.emission_alpha = 0.001;
        g1.emission_beta = 0.1;
        g1.emission_gamma = 1.0;
        g1.p_current_mw = 125.0;
        let cap = CapAndTradeDispatch::new(vec![g0, g1], 0.001, 50.0); // tiny cap
        let (dispatch, allowance_cost) = cap.dispatch(300.0).expect("cap dispatch failed");
        assert!(
            (dispatch.total_generation_mw - 300.0).abs() < 1.0,
            "power balance: {}",
            dispatch.total_generation_mw
        );
        assert!(
            allowance_cost > 0.0,
            "exceeds cap → positive allowance_cost: {}",
            allowance_cost
        );
    }
}
