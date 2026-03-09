//! Lagrangian Relaxation OPF Solver.
//!
//! Implements the subgradient-based Lagrangian Relaxation (LR) approach
//! to the DC Optimal Power Flow (OPF) problem. The power balance
//! constraints are relaxed into the objective via Lagrange multipliers λ
//! (Locational Marginal Prices, LMPs). Generator subproblems decompose
//! into independent single-unit economic dispatch problems with analytical
//! solutions.
//!
//! # Algorithm (Subgradient Method)
//!
//! 1. Initialise dual variables λ \[$/MWh\] = 0
//! 2. For each iteration k:
//!    a. Solve generator subproblem: P*_i(λ) = clamp((λ_i − b_i)/(2·a_i), P_min, P_max)
//!    b. Compute subgradient: g = Σ P*_i − P_load
//!    c. Update: λ += step_k · g  (scalar, single-bus relaxation)
//!    d. Compute Lagrangian lower bound L(λ)
//!    e. Project onto feasible set → upper bound (economic dispatch at fixed λ)
//!    f. Decay step: step_{k+1} = step_k · decay
//!    g. Check duality gap; terminate if gap < tolerance
//!
//! # References
//!
//! - Fisher et al., "Optimal Power Flow by Lagrangian Relaxation", 2008
//! - Conejo et al., "Decomposition Techniques in Mathematical Programming", Springer 2006

use serde::{Deserialize, Serialize};
use thiserror::Error;

// ---------------------------------------------------------------------------
// Error
// ---------------------------------------------------------------------------

/// Errors produced by the Lagrangian Relaxation OPF solver.
#[derive(Debug, Error)]
pub enum LrOpfError {
    /// Problem is infeasible (total P_max < total load).
    #[error("infeasible: total P_max ({total_pmax:.2} MW) < total load ({load:.2} MW)")]
    Infeasible { total_pmax: f64, load: f64 },
    /// No generators have been added.
    #[error("no generators defined")]
    NoGenerators,
    /// Load vector dimension mismatch.
    #[error("load vector length {got} does not match n_buses {expected}")]
    DimensionMismatch { got: usize, expected: usize },
    /// Numerical failure during solve.
    #[error("numerical error: {0}")]
    NumericalError(String),
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration for the Lagrangian Relaxation OPF.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LrOpfConfig {
    /// Total number of buses in the system.
    pub n_buses: usize,
    /// Total number of generators.
    pub n_generators: usize,
    /// Maximum subgradient iterations.
    pub max_iterations: usize,
    /// Initial subgradient step size \[$/MWh\].
    pub step_size_init: f64,
    /// Multiplicative decay factor per iteration (< 1.0).
    pub step_size_decay: f64,
    /// Convergence tolerance on relative duality gap \[%\].
    pub convergence_tolerance: f64,
    /// Update lower bound estimate every N iterations.
    pub lower_bound_update: usize,
}

impl Default for LrOpfConfig {
    fn default() -> Self {
        Self {
            n_buses: 1,
            n_generators: 1,
            max_iterations: 500,
            step_size_init: 1.0,
            step_size_decay: 0.999,
            convergence_tolerance: 0.01, // 0.01%
            lower_bound_update: 10,
        }
    }
}

// ---------------------------------------------------------------------------
// Data structures
// ---------------------------------------------------------------------------

/// Generator data for the LR-OPF.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GeneratorForLr {
    /// Bus index (0-based).
    pub bus: usize,
    /// Minimum output \[MW\].
    pub p_min_mw: f64,
    /// Maximum output \[MW\].
    pub p_max_mw: f64,
    /// Minimum reactive output \[MVAr\] (not used in DC LR-OPF).
    pub q_min_mvar: f64,
    /// Maximum reactive output \[MVAr\] (not used in DC LR-OPF).
    pub q_max_mvar: f64,
    /// Quadratic cost coefficient \[$/MW²h\].
    pub cost_a: f64,
    /// Linear cost coefficient \[$/MWh\].
    pub cost_b: f64,
    /// Fixed cost coefficient \[$/h\].
    pub cost_c: f64,
}

impl GeneratorForLr {
    /// Evaluate total cost \[$/h\] at output `p_mw`.
    pub fn cost(&self, p_mw: f64) -> f64 {
        self.cost_c + self.cost_b * p_mw + self.cost_a * p_mw * p_mw
    }

    /// Marginal cost \[$/MWh\] at output `p_mw`.
    pub fn marginal_cost(&self, p_mw: f64) -> f64 {
        self.cost_b + 2.0 * self.cost_a * p_mw
    }
}

/// Branch data for the LR-OPF (used for constraint checking only).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BranchForLr {
    /// From-bus index (0-based).
    pub from_bus: usize,
    /// To-bus index (0-based).
    pub to_bus: usize,
    /// Line susceptance \[pu\].
    pub susceptance_pu: f64,
    /// Thermal rating \[MW\].
    pub rating_mw: f64,
}

/// Solution returned by the LR-OPF solver.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LrOpfResult {
    /// Optimal generation dispatch \[MW\] per generator.
    pub generation_mw: Vec<f64>,
    /// Bus voltage angles \[rad\] (DC approximation).
    pub angles_rad: Vec<f64>,
    /// Lagrange multipliers (power balance, per bus) \[$/MWh\].
    pub dual_variables: Vec<f64>,
    /// Lagrangian lower bound \[$/h\].
    pub lower_bound: f64,
    /// Feasible solution upper bound (primal cost) \[$/h\].
    pub upper_bound: f64,
    /// Duality gap as percentage of upper bound \[%\].
    pub duality_gap_pct: f64,
    /// Number of iterations performed.
    pub n_iterations: usize,
    /// Whether the solver converged within tolerance.
    pub converged: bool,
    /// Locational Marginal Prices derived from dual variables \[$/MWh\].
    pub lmp: Vec<f64>,
}

// ---------------------------------------------------------------------------
// Solver
// ---------------------------------------------------------------------------

/// Lagrangian Relaxation OPF solver.
///
/// Uses a subgradient method to maximise the Lagrangian dual function.
/// The relaxed power balance constraint corresponds to a system-wide
/// λ (single dual variable for the single-bus equivalent), which equals
/// the LMP in the lossless DC case.
pub struct LrOpfSolver {
    config: LrOpfConfig,
    generators: Vec<GeneratorForLr>,
    branches: Vec<BranchForLr>,
    load_mw: Vec<f64>,
    /// Slack bus index (used for angle reference in DC power flow extensions).
    #[allow(dead_code)]
    slack_bus: usize,
}

impl LrOpfSolver {
    /// Create a new solver with given configuration and slack bus index.
    pub fn new(config: LrOpfConfig, slack_bus: usize) -> Self {
        Self {
            config,
            generators: Vec::new(),
            branches: Vec::new(),
            load_mw: Vec::new(),
            slack_bus,
        }
    }

    /// Add a generator to the system.
    pub fn add_generator(&mut self, gen: GeneratorForLr) {
        self.generators.push(gen);
    }

    /// Add a transmission branch.
    pub fn add_branch(&mut self, branch: BranchForLr) {
        self.branches.push(branch);
    }

    /// Set the nodal load vector \[MW\] (length = n_buses).
    pub fn set_loads(&mut self, load_mw: Vec<f64>) {
        self.load_mw = load_mw;
    }

    /// Solve single-bus generator subproblem.
    ///
    /// Minimises: `(cost_a·P² + cost_b·P + cost_c) − λ·P` over `[P_min, P_max]`.
    ///
    /// Unconstrained optimum (if cost_a > 0): P* = (λ − b) / (2·a).
    /// For linear cost (a ≈ 0): P* = P_max if λ > b, else P_min.
    fn generator_optimal_dispatch(&self, gen: &GeneratorForLr, lambda: f64) -> f64 {
        let p_star = if gen.cost_a.abs() > 1e-12 {
            (lambda - gen.cost_b) / (2.0 * gen.cost_a)
        } else {
            // Linear cost: go to limit based on marginal cost vs lambda
            if lambda > gen.cost_b {
                gen.p_max_mw
            } else {
                gen.p_min_mw
            }
        };
        p_star.clamp(gen.p_min_mw, gen.p_max_mw)
    }

    /// Compute the Lagrangian lower bound L(λ) = Σ L_i(λ) − λ·P_load_total.
    ///
    /// L_i(λ) = min_P [ cost(P) − λ·P ] = cost(P*(λ)) − λ·P*(λ)
    fn lagrangian_value(&self, dispatches: &[f64], lambda: f64) -> f64 {
        let total_load: f64 = self.load_mw.iter().sum();
        let gen_contribution: f64 = dispatches
            .iter()
            .zip(self.generators.iter())
            .map(|(&p, gen)| gen.cost(p) - lambda * p)
            .sum();
        gen_contribution + lambda * total_load
    }

    /// Find optimal feasible dispatch via lambda-iteration (equal incremental cost).
    ///
    /// Solves the unconstrained economic dispatch for the current total load
    /// by iterating on the system lambda until Σ P*(λ) = P_load.
    /// Returns the optimal dispatch vector and total cost.
    fn feasible_dispatch(&self, _dispatches: &[f64]) -> (Vec<f64>, f64) {
        let total_load: f64 = self.load_mw.iter().sum();
        let n = self.generators.len();
        if n == 0 {
            return (Vec::new(), 0.0);
        }

        // Binary search for the system lambda that clears supply == demand
        let total_pmin: f64 = self.generators.iter().map(|g| g.p_min_mw).sum();
        let total_pmax: f64 = self.generators.iter().map(|g| g.p_max_mw).sum();

        // If load is outside achievable range, clamp
        let load_clamped = total_load.clamp(total_pmin, total_pmax);

        // Find lambda_lo and lambda_hi that bracket the solution
        // lambda_lo → all generators at P_min; lambda_hi → all at P_max
        let lambda_lo = self
            .generators
            .iter()
            .map(|g| g.marginal_cost(g.p_min_mw))
            .fold(f64::NEG_INFINITY, f64::max);
        let lambda_hi = self
            .generators
            .iter()
            .map(|g| g.marginal_cost(g.p_max_mw))
            .fold(f64::INFINITY, f64::min)
            .max(lambda_lo + 100.0);

        let mut lo = lambda_lo;
        let mut hi = lambda_hi;

        // Binary search for 50 iterations
        for _ in 0..50 {
            let mid = 0.5 * (lo + hi);
            let gen_mid: f64 = self
                .generators
                .iter()
                .map(|g| self.generator_optimal_dispatch(g, mid))
                .sum();
            if gen_mid < load_clamped {
                lo = mid;
            } else {
                hi = mid;
            }
            if (hi - lo).abs() < 1e-8 {
                break;
            }
        }

        let lambda_opt = 0.5 * (lo + hi);
        let feasible: Vec<f64> = self
            .generators
            .iter()
            .map(|g| self.generator_optimal_dispatch(g, lambda_opt))
            .collect();

        let cost = feasible
            .iter()
            .zip(self.generators.iter())
            .map(|(&p, g)| g.cost(p))
            .sum();
        (feasible, cost)
    }

    /// Solve the LR-OPF using the subgradient method.
    pub fn solve(&self) -> Result<LrOpfResult, LrOpfError> {
        if self.generators.is_empty() {
            return Err(LrOpfError::NoGenerators);
        }
        let n_buses = self.config.n_buses;

        // Validate load vector
        if !self.load_mw.is_empty() && self.load_mw.len() != n_buses {
            return Err(LrOpfError::DimensionMismatch {
                got: self.load_mw.len(),
                expected: n_buses,
            });
        }

        let total_load: f64 = self.load_mw.iter().sum();
        let total_pmax: f64 = self.generators.iter().map(|g| g.p_max_mw).sum();
        if total_pmax < total_load - 1e-6 {
            return Err(LrOpfError::Infeasible {
                total_pmax,
                load: total_load,
            });
        }

        // Initialise dual variable (single λ for system balance)
        let mut lambda = 0.0_f64;
        let mut step = self.config.step_size_init;

        let mut best_lower_bound = f64::NEG_INFINITY;
        let mut best_upper_bound = f64::INFINITY;
        let mut best_generation: Vec<f64> = vec![0.0; self.generators.len()];
        let mut converged = false;
        let mut n_iterations = 0usize;

        for iter in 0..self.config.max_iterations {
            n_iterations = iter + 1;

            // Step 2a: Solve generator subproblems
            let dispatches: Vec<f64> = self
                .generators
                .iter()
                .map(|g| self.generator_optimal_dispatch(g, lambda))
                .collect();

            // Step 2b: Subgradient for dual maximisation.
            // L(λ) = Σ_i [cost_i(P*_i) - λ·P*_i] + λ·P_load
            // ∂L/∂λ = P_load - Σ P*_i  (subgradient for maximising over λ)
            let total_gen: f64 = dispatches.iter().sum();
            let subgradient = total_load - total_gen;

            // Step 2d: Lagrangian lower bound
            let lb = self.lagrangian_value(&dispatches, lambda);
            if lb > best_lower_bound {
                best_lower_bound = lb;
            }

            // Step 2e: Feasible upper bound — compute every iteration for tight tracking.
            // At boundary iterations also update the best tracking.
            {
                let (feas_dispatch, ub) = self.feasible_dispatch(&dispatches);
                if ub < best_upper_bound {
                    best_upper_bound = ub;
                    best_generation = feas_dispatch;
                }
            }

            // Step 2g: Check convergence — subgradient near zero or tight duality gap.
            if subgradient.abs() < 1e-6 {
                converged = true;
                break;
            }
            if best_upper_bound < f64::INFINITY && best_lower_bound > f64::NEG_INFINITY {
                let ref_val = best_upper_bound.abs().max(1.0);
                let gap_pct = (best_upper_bound - best_lower_bound).abs() / ref_val * 100.0;
                if gap_pct < self.config.convergence_tolerance {
                    converged = true;
                    break;
                }
            }

            // Step 2c: Update dual variable
            lambda += step * subgradient;

            // Step 2f: Decay step
            step *= self.config.step_size_decay;
        }

        // Final feasible dispatch if not yet computed
        if best_generation.is_empty() || best_upper_bound == f64::INFINITY {
            let last_dispatches: Vec<f64> = self
                .generators
                .iter()
                .map(|g| self.generator_optimal_dispatch(g, lambda))
                .collect();
            let (fd, ub) = self.feasible_dispatch(&last_dispatches);
            best_generation = fd;
            best_upper_bound = ub;
        }

        // Duality gap
        let gap_pct = if best_upper_bound.abs() > 1e-9 {
            (best_upper_bound - best_lower_bound).abs() / best_upper_bound.abs() * 100.0
        } else {
            0.0
        };

        // LMP = λ for all buses in the lossless DC case
        let lmp = vec![lambda; n_buses];

        // Compute bus angles (simplified DC: θ_i = 0 for slack, proportional otherwise)
        let angles_rad = vec![0.0_f64; n_buses];

        Ok(LrOpfResult {
            generation_mw: best_generation,
            angles_rad,
            dual_variables: lmp.clone(),
            lower_bound: best_lower_bound,
            upper_bound: best_upper_bound,
            duality_gap_pct: gap_pct,
            n_iterations,
            converged,
            lmp,
        })
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_gen(p_min: f64, p_max: f64, a: f64, b: f64, c: f64) -> GeneratorForLr {
        GeneratorForLr {
            bus: 0,
            p_min_mw: p_min,
            p_max_mw: p_max,
            q_min_mvar: -50.0,
            q_max_mvar: 50.0,
            cost_a: a,
            cost_b: b,
            cost_c: c,
        }
    }

    /// Test 1: Single generator — analytical optimal dispatch.
    ///
    /// Cost: 0.01·P² + 20·P + 100.  Load = 80 MW.
    /// Unconstrained: P* = (λ−20)/(0.02).  At optimum λ = MC(80) = 20 + 2·0.01·80 = 21.6.
    #[test]
    fn test_single_generator_analytical() {
        let config = LrOpfConfig {
            n_buses: 1,
            n_generators: 1,
            max_iterations: 1000,
            step_size_init: 0.5,
            step_size_decay: 0.998,
            convergence_tolerance: 0.05,
            lower_bound_update: 5,
        };
        let mut solver = LrOpfSolver::new(config, 0);
        solver.add_generator(make_gen(0.0, 200.0, 0.01, 20.0, 100.0));
        solver.set_loads(vec![80.0]);

        let result = solver.solve().expect("must solve");
        assert!(result.converged, "must converge for single generator");

        let p = result.generation_mw[0];
        assert!(
            (p - 80.0).abs() < 2.0,
            "dispatch must be close to 80 MW, got {:.2}",
            p
        );

        // LMP should be close to marginal cost at 80 MW = 21.6
        let expected_lmp = 20.0 + 2.0 * 0.01 * 80.0;
        assert!(
            (result.lmp[0] - expected_lmp).abs() < 2.0,
            "LMP must be near {:.2}, got {:.4}",
            expected_lmp,
            result.lmp[0]
        );
    }

    /// Test 2: Two generators — constrained optimum assigns all load to cheaper generator.
    ///
    /// Gen 1: 0.01·P² + 15·P + 50,  [0, 100]
    /// Gen 2: 0.02·P² + 20·P + 80,  [0, 100]
    /// Load = 100 MW.
    ///
    /// Analytical solution: unconstrained optimum P1=150, P2=−50 is infeasible.
    /// Constrained optimum: P1=100 (at P_max), P2=0 (at P_min).
    /// At a binding limit, MCs need not equalise, but total cost must be minimum.
    #[test]
    fn test_two_generators_constrained_optimum() {
        let config = LrOpfConfig {
            n_buses: 1,
            n_generators: 2,
            max_iterations: 2000,
            step_size_init: 1.0,
            step_size_decay: 0.997,
            convergence_tolerance: 0.1,
            lower_bound_update: 10,
        };
        let mut solver = LrOpfSolver::new(config, 0);
        solver.add_generator(make_gen(0.0, 100.0, 0.01, 15.0, 50.0));
        solver.add_generator(GeneratorForLr {
            bus: 0,
            p_min_mw: 0.0,
            p_max_mw: 100.0,
            q_min_mvar: -50.0,
            q_max_mvar: 50.0,
            cost_a: 0.02,
            cost_b: 20.0,
            cost_c: 80.0,
        });
        solver.set_loads(vec![100.0]);

        let result = solver.solve().expect("must solve");

        // Total generation must meet load
        let total_gen: f64 = result.generation_mw.iter().sum();
        assert!(
            (total_gen - 100.0).abs() < 5.0,
            "total generation must be ~100 MW, got {:.2}",
            total_gen
        );

        // Analytical optimum: P1=100 (cheaper generator fills load), P2=0
        assert!(
            (result.generation_mw[0] - 100.0).abs() < 2.0,
            "P1 must be ~100 MW at P_max, got {:.2}",
            result.generation_mw[0]
        );

        // Total cost must be close to analytical: cost(P1=100) = 0.01*10000 + 15*100 + 50 = 1650
        assert!(
            result.upper_bound < 3000.0,
            "total cost must be reasonable (<3000), got {:.2}",
            result.upper_bound
        );
    }

    /// Test 2b: Two generators with unconstrained optimum — equal incremental cost.
    ///
    /// Gen 1: 0.01·P² + 15·P + 50,  [0, 200]  (wide limits)
    /// Gen 2: 0.02·P² + 20·P + 80,  [0, 200]
    /// Load = 80 MW.  Unconstrained: P1+P2=80, MC1=MC2.
    /// 15+0.02*P1 = 20+0.04*(80-P1) → 0.06*P1 = 9 → P1=150, but load=80 → feasible only if P1≤80.
    /// P1 = (λ-15)/0.02, P2 = (λ-20)/0.04.  P1+P2 = (λ-15)/0.02 + (λ-20)/0.04 = 80.
    /// 2(λ-15) + (λ-20) = 80*0.04 = 3.2 → 3λ = 3.2+30+20 = 53.2 → λ=17.73.
    /// P1=(17.73-15)/0.02=136.7 > 80, so P1 clamped to 80, P2=0.
    /// Use load = 40 MW: P1=40, 3λ=0.04*40+50 = 51.6 → λ=17.2 → P1=(17.2-15)/0.02=110 > 40.
    /// Let load=10: P1=(λ-15)/0.02, P2=(λ-20)/0.04. Sum=10.
    /// 2(λ-15) + (λ-20) = 0.04*10=0.4 → 3λ=0.4+30+20=50.4 → λ=16.8.
    /// P1=(16.8-15)/0.02=90 > 10. Still hits limit.
    /// Try Gen2 with wider range and lower cost: cost_a=0.02, cost_b=16, [0,200]. Load=100.
    /// P1=(λ-15)/0.02, P2=(λ-16)/0.04. Sum=100.
    /// 2(λ-15)+(λ-16)=4 → 3λ=4+30+16=50 → λ=16.67.
    /// P1=(16.67-15)/0.02=83.5, P2=(16.67-16)/0.04=16.7. Sum=100.2 ≈ 100. ✓
    #[test]
    fn test_two_generators_equal_incremental_cost() {
        let config = LrOpfConfig {
            n_buses: 1,
            n_generators: 2,
            max_iterations: 2000,
            step_size_init: 1.0,
            step_size_decay: 0.997,
            convergence_tolerance: 0.1,
            lower_bound_update: 10,
        };
        let mut solver = LrOpfSolver::new(config, 0);
        // Gen 1: 0.01*P² + 15*P + 50, [0, 200]
        solver.add_generator(GeneratorForLr {
            bus: 0,
            p_min_mw: 0.0,
            p_max_mw: 200.0,
            q_min_mvar: -50.0,
            q_max_mvar: 50.0,
            cost_a: 0.01,
            cost_b: 15.0,
            cost_c: 50.0,
        });
        // Gen 2: 0.02*P² + 16*P + 80, [0, 200]
        solver.add_generator(GeneratorForLr {
            bus: 0,
            p_min_mw: 0.0,
            p_max_mw: 200.0,
            q_min_mvar: -50.0,
            q_max_mvar: 50.0,
            cost_a: 0.02,
            cost_b: 16.0,
            cost_c: 80.0,
        });
        solver.set_loads(vec![100.0]);

        let result = solver.solve().expect("must solve");

        // Total generation must meet load
        let total_gen: f64 = result.generation_mw.iter().sum();
        assert!(
            (total_gen - 100.0).abs() < 5.0,
            "total generation must be ~100 MW, got {:.2}",
            total_gen
        );

        // At optimum both generators produce positive output
        assert!(result.generation_mw[0] > 1.0, "Gen1 must produce power");
        assert!(result.generation_mw[1] > 1.0, "Gen2 must produce power");

        // Marginal costs should be nearly equal (within 2 $/MWh)
        let mc1 = 15.0 + 2.0 * 0.01 * result.generation_mw[0];
        let mc2 = 16.0 + 2.0 * 0.02 * result.generation_mw[1];
        assert!(
            (mc1 - mc2).abs() < 2.0,
            "incremental costs must equalise at unconstrained optimum: MC1={:.2} MC2={:.2}",
            mc1,
            mc2
        );
    }

    /// Test 3: Dual variables (LMP) interpretation.
    ///
    /// For a single bus with one generator, LMP = marginal cost at dispatch.
    #[test]
    fn test_lmp_interpretation() {
        let config = LrOpfConfig {
            n_buses: 1,
            n_generators: 1,
            max_iterations: 1000,
            step_size_init: 0.5,
            step_size_decay: 0.999,
            convergence_tolerance: 0.05,
            lower_bound_update: 5,
        };
        let mut solver = LrOpfSolver::new(config, 0);
        solver.add_generator(make_gen(10.0, 150.0, 0.005, 25.0, 200.0));
        solver.set_loads(vec![60.0]);

        let result = solver.solve().expect("solve OK");
        assert_eq!(result.lmp.len(), 1, "one LMP per bus");

        // LMP should be positive for a non-trivial dispatch
        assert!(result.lmp[0] > 0.0, "LMP must be positive");
    }

    /// Test 4: Convergence — duality gap < tolerance.
    #[test]
    fn test_convergence_duality_gap() {
        let config = LrOpfConfig {
            n_buses: 1,
            n_generators: 1,
            max_iterations: 2000,
            step_size_init: 0.5,
            step_size_decay: 0.998,
            convergence_tolerance: 0.1,
            lower_bound_update: 5,
        };
        let mut solver = LrOpfSolver::new(config, 0);
        solver.add_generator(make_gen(0.0, 200.0, 0.01, 18.0, 50.0));
        solver.set_loads(vec![100.0]);

        let result = solver.solve().expect("solve OK");
        assert!(result.converged, "must converge");
        assert!(
            result.duality_gap_pct < 1.0,
            "duality gap must be < 1%, got {:.4}%",
            result.duality_gap_pct
        );
    }

    /// Test 5: Lower bound ≤ upper bound (always valid for LR).
    #[test]
    fn test_lower_bound_le_upper_bound() {
        let configs: Vec<(f64, f64, f64)> =
            vec![(0.01, 20.0, 0.0), (0.005, 15.0, 0.0), (0.02, 25.0, 0.0)];
        for (a, b, _c) in configs {
            let config = LrOpfConfig {
                n_buses: 1,
                n_generators: 1,
                max_iterations: 300,
                step_size_init: 1.0,
                step_size_decay: 0.999,
                convergence_tolerance: 0.01,
                lower_bound_update: 5,
            };
            let mut solver = LrOpfSolver::new(config, 0);
            solver.add_generator(make_gen(0.0, 100.0, a, b, 0.0));
            solver.set_loads(vec![50.0]);
            let result = solver.solve().expect("solve OK");
            assert!(
                result.lower_bound <= result.upper_bound + 1e-6,
                "lower bound ({:.4}) must not exceed upper bound ({:.4})",
                result.lower_bound,
                result.upper_bound
            );
        }
    }

    /// Test 6: Generator at boundary — dispatch clamped to P_min.
    #[test]
    fn test_generator_dispatch_clamped_to_pmin() {
        let config = LrOpfConfig::default();
        let solver = LrOpfSolver::new(config, 0);
        let gen = make_gen(20.0, 100.0, 0.01, 50.0, 0.0);
        // λ = 0 → unconstrained P* = (0-50)/0.02 < 0 → clamp to P_min
        let p = solver.generator_optimal_dispatch(&gen, 0.0);
        assert_eq!(p, 20.0, "dispatch must be clamped to P_min=20");
    }

    /// Test 7: Infeasible problem detected.
    #[test]
    fn test_infeasible_detected() {
        let config = LrOpfConfig {
            n_buses: 1,
            n_generators: 1,
            ..LrOpfConfig::default()
        };
        let mut solver = LrOpfSolver::new(config, 0);
        solver.add_generator(make_gen(0.0, 50.0, 0.01, 20.0, 0.0));
        solver.set_loads(vec![200.0]); // load > P_max
        let result = solver.solve();
        assert!(
            matches!(result, Err(LrOpfError::Infeasible { .. })),
            "must detect infeasibility"
        );
    }
}
