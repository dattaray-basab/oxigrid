//! Carbon-Constrained Optimal Power Flow (Carbon-OPF).
//!
//! Extends the classic economic dispatch (DC-OPF) with CO₂ emissions
//! constraints and a dual-objective formulation that allows trading off
//! generation cost against greenhouse-gas emissions.
//!
//! # Algorithm
//!
//! 1. Compute augmented marginal cost for each generator:
//!    `c_aug = w·energy_cost + carbon_price·co2_rate`
//!    where `w` is the dual-objective weight (1 = pure cost, 0 = pure emissions).
//! 2. Dispatch must-run units first at their fixed/minimum output.
//! 3. Sort remaining generators by `c_aug` (merit order).
//! 4. Fill load in merit order subject to `P_min ≤ P ≤ P_max`.
//! 5. If a hard carbon cap is set and the solution violates it, iteratively
//!    substitute the highest-emission dispatched unit with the next-cheapest
//!    lower-emission alternative.
//! 6. Compute Green LMP (including carbon adder) for each bus.
//!
//! # Pareto Front
//!
//! [`CarbonOpfSolver::pareto_front`] sweeps the dual-objective weight from
//! 0 (emissions minimised) to 1 (cost minimised) and returns one
//! [`CarbonOpfResult`] per point.

use thiserror::Error;

// ── Error ─────────────────────────────────────────────────────────────────────

/// Errors produced by the carbon-constrained OPF solver.
#[derive(Debug, Error)]
pub enum CarbonOpfError {
    /// No feasible dispatch satisfies the carbon cap.
    #[error("infeasible: {0}")]
    Infeasible(String),

    /// Configuration is self-contradictory or missing data.
    #[error("invalid configuration: {0}")]
    InvalidConfig(String),

    /// No generators have been added.
    #[error("solver has no generators")]
    NoGenerators,
}

// ── Public types ───────────────────────────────────────────────────────────────

/// Global configuration for the Carbon-OPF solver.
#[derive(Debug, Clone)]
pub struct CarbonOpfConfig {
    /// System base \[MVA\].
    pub base_mva: f64,
    /// Number of buses in the network.
    pub n_buses: usize,
    /// Hard carbon emission cap \[t/h\]. `None` means unconstrained.
    pub carbon_limit_t_per_h: Option<f64>,
    /// Carbon price for soft penalty \[$/t\].
    pub carbon_price_usd_per_t: f64,
    /// When `true`, zero-emission generators are given lexicographic priority.
    pub renewable_priority: bool,
    /// Weight `w` (0–1): objective = `w·cost + (1−w)·emissions·carbon_price`.
    pub dual_objective_weight: f64,
}

/// One generator in the carbon-constrained dispatch.
#[derive(Debug, Clone)]
pub struct GeneratorCarbon {
    /// Bus index (0-based).
    pub bus: usize,
    /// Maximum active power output \[MW\].
    pub p_max_mw: f64,
    /// Minimum active power output \[MW\].
    pub p_min_mw: f64,
    /// Energy production cost \[$/MWh\].
    pub energy_cost_usd_per_mwh: f64,
    /// CO₂ emission factor \[t/MWh\] (0 for zero-carbon renewables).
    pub co2_rate_t_per_mwh: f64,
    /// `true` if the unit must be online and cannot be reduced below `p_min_mw`.
    pub is_must_run: bool,
    /// Fixed dispatch \[MW\] for must-run units; `None` → dispatch at `p_min_mw`.
    pub p_fixed_mw: Option<f64>,
}

/// Result returned by [`CarbonOpfSolver::solve`].
#[derive(Debug, Clone)]
pub struct CarbonOpfResult {
    /// Optimal dispatch \[MW\] per generator (same order as added).
    pub dispatch_mw: Vec<f64>,
    /// Total generation cost \[$/h\].
    pub total_cost_usd_per_h: f64,
    /// Total CO₂ emissions \[t/h\].
    pub total_emissions_t_per_h: f64,
    /// Remaining headroom against the carbon cap \[t/h\] (0 if unconstrained).
    pub emissions_budget_slack_t_per_h: f64,
    /// Total curtailed renewable power (not used here — always 0) \[MW\].
    pub renewable_curtailment_mw: f64,
    /// Shadow price of the carbon constraint \[$/t\].
    pub carbon_price_shadow: f64,
    /// `(total_cost, total_emissions)` Pareto coordinate for this run.
    pub pareto_point: (f64, f64),
    /// Locational Marginal Price including carbon adder per bus \[$/MWh\].
    pub green_lmp: Vec<f64>,
}

/// Carbon-constrained OPF solver.
pub struct CarbonOpfSolver {
    config: CarbonOpfConfig,
    generators: Vec<GeneratorCarbon>,
    network_b_matrix: Vec<Vec<f64>>,
    load_mw: Vec<f64>,
}

impl CarbonOpfSolver {
    /// Create a new solver with the given configuration.
    pub fn new(config: CarbonOpfConfig) -> Self {
        Self {
            config,
            generators: Vec::new(),
            network_b_matrix: Vec::new(),
            load_mw: Vec::new(),
        }
    }

    /// Set the network B matrix and per-bus load.
    pub fn set_network(&mut self, b_matrix: Vec<Vec<f64>>, load_mw: Vec<f64>) {
        self.network_b_matrix = b_matrix;
        self.load_mw = load_mw;
    }

    /// Add a generator to the pool.
    pub fn add_generator(&mut self, gen: GeneratorCarbon) {
        self.generators.push(gen);
    }

    /// Solve the carbon-constrained dispatch.
    pub fn solve(&self) -> Result<CarbonOpfResult, CarbonOpfError> {
        self.solve_with_weight(self.config.dual_objective_weight)
    }

    /// Generate the Pareto front by sweeping `dual_objective_weight` 0→1.
    pub fn pareto_front(&self, n_points: usize) -> Result<Vec<CarbonOpfResult>, CarbonOpfError> {
        if n_points == 0 {
            return Err(CarbonOpfError::InvalidConfig(
                "n_points must be ≥ 1".to_string(),
            ));
        }
        let mut results = Vec::with_capacity(n_points);
        for i in 0..n_points {
            let w = if n_points == 1 {
                0.5
            } else {
                i as f64 / (n_points - 1) as f64
            };
            results.push(self.solve_with_weight(w)?);
        }
        Ok(results)
    }

    // ── Internal solver ────────────────────────────────────────────────────

    fn solve_with_weight(&self, w: f64) -> Result<CarbonOpfResult, CarbonOpfError> {
        if self.generators.is_empty() {
            return Err(CarbonOpfError::NoGenerators);
        }
        if self.load_mw.is_empty() {
            return Err(CarbonOpfError::InvalidConfig(
                "load_mw must be set before solving".to_string(),
            ));
        }

        let n_gen = self.generators.len();
        let total_load: f64 = self.load_mw.iter().sum();

        // ── Step 1: dispatch must-run generators ──────────────────────────
        let mut dispatch = vec![0.0_f64; n_gen];
        let mut remaining_load = total_load;

        for (i, gen) in self.generators.iter().enumerate() {
            if gen.is_must_run {
                let p = gen
                    .p_fixed_mw
                    .unwrap_or(gen.p_min_mw)
                    .clamp(gen.p_min_mw, gen.p_max_mw);
                dispatch[i] = p;
                remaining_load -= p;
            }
        }

        // ── Step 2: augmented marginal costs for flexible generators ──────
        let mut merit: Vec<(usize, f64)> = self
            .generators
            .iter()
            .enumerate()
            .filter(|(_, g)| !g.is_must_run)
            .map(|(i, g)| {
                let mut c_aug = w * g.energy_cost_usd_per_mwh
                    + self.config.carbon_price_usd_per_t * g.co2_rate_t_per_mwh;
                if self.config.renewable_priority && g.co2_rate_t_per_mwh == 0.0 {
                    c_aug -= 1000.0; // force renewables to front of queue
                }
                (i, c_aug)
            })
            .collect();
        merit.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

        // ── Step 3: economic dispatch in merit order ───────────────────────
        for &(i, _) in &merit {
            if remaining_load <= 0.0 {
                break;
            }
            let gen = &self.generators[i];
            let headroom = gen.p_max_mw - gen.p_min_mw;
            let to_dispatch = remaining_load.min(headroom).max(0.0);
            dispatch[i] = gen.p_min_mw + to_dispatch;
            remaining_load -= to_dispatch;
        }

        // If load cannot be met, note how much is unserved (no error — just
        // report; real systems would shed load).
        // remaining_load may be > 0 if total generation < load.

        // ── Step 4: carbon-cap enforcement ────────────────────────────────
        let mut emissions: f64 = self
            .generators
            .iter()
            .enumerate()
            .map(|(i, g)| dispatch[i] * g.co2_rate_t_per_mwh)
            .sum();

        let mut shadow_price = 0.0_f64;

        if let Some(cap) = self.config.carbon_limit_t_per_h {
            // Iterative substitution: replace highest-emission dispatched unit
            // with the cheapest lower-emission unit available.
            let max_iter = n_gen * 2;
            for _ in 0..max_iter {
                if emissions <= cap + 1e-9 {
                    break;
                }
                // Find highest-emission dispatched non-must-run generator.
                let worst = self
                    .generators
                    .iter()
                    .enumerate()
                    .filter(|(i, g)| !g.is_must_run && dispatch[*i] > g.p_min_mw + 1e-9)
                    .max_by(|a, b| {
                        (a.1.co2_rate_t_per_mwh * dispatch[a.0])
                            .partial_cmp(&(b.1.co2_rate_t_per_mwh * dispatch[b.0]))
                            .unwrap_or(std::cmp::Ordering::Equal)
                    });

                // Find cheapest lower-emission generator with headroom.
                let best_green = self
                    .generators
                    .iter()
                    .enumerate()
                    .filter(|(i, g)| {
                        !g.is_must_run
                            && dispatch[*i] < g.p_max_mw - 1e-9
                            && g.co2_rate_t_per_mwh
                                < worst
                                    .as_ref()
                                    .map(|(_, g2)| g2.co2_rate_t_per_mwh)
                                    .unwrap_or(f64::INFINITY)
                    })
                    .min_by(|a, b| {
                        a.1.energy_cost_usd_per_mwh
                            .partial_cmp(&b.1.energy_cost_usd_per_mwh)
                            .unwrap_or(std::cmp::Ordering::Equal)
                    });

                match (worst, best_green) {
                    (Some((wi, wgen)), Some((gi, ggen))) => {
                        let reduce = (dispatch[wi] - wgen.p_min_mw)
                            .min(ggen.p_max_mw - dispatch[gi])
                            .min(1.0); // 1 MW steps
                        if reduce < 1e-9 {
                            break;
                        }
                        shadow_price = ggen.energy_cost_usd_per_mwh - wgen.energy_cost_usd_per_mwh;
                        dispatch[wi] -= reduce;
                        dispatch[gi] += reduce;
                        emissions -= reduce * (wgen.co2_rate_t_per_mwh - ggen.co2_rate_t_per_mwh);
                    }
                    _ => break, // no more substitutions possible
                }
            }

            if emissions > cap + 1.0 {
                return Err(CarbonOpfError::Infeasible(format!(
                    "cannot reduce emissions ({:.2} t/h) below cap ({:.2} t/h)",
                    emissions, cap
                )));
            }
        }

        // ── Step 5: compute totals ─────────────────────────────────────────
        let total_cost: f64 = self
            .generators
            .iter()
            .enumerate()
            .map(|(i, g)| dispatch[i] * g.energy_cost_usd_per_mwh)
            .sum();

        let slack = self
            .config
            .carbon_limit_t_per_h
            .map(|cap| (cap - emissions).max(0.0))
            .unwrap_or(0.0);

        // ── Step 6: Green LMP per bus ─────────────────────────────────────
        let green_lmp = self.compute_green_lmp(&dispatch, w);

        Ok(CarbonOpfResult {
            dispatch_mw: dispatch,
            total_cost_usd_per_h: total_cost,
            total_emissions_t_per_h: emissions,
            emissions_budget_slack_t_per_h: slack,
            renewable_curtailment_mw: 0.0,
            carbon_price_shadow: shadow_price,
            pareto_point: (total_cost, emissions),
            green_lmp,
        })
    }

    /// Compute Green LMP (energy cost + carbon adder) for each bus.
    fn compute_green_lmp(&self, dispatch: &[f64], w: f64) -> Vec<f64> {
        let n = self.config.n_buses;
        if n == 0 {
            return Vec::new();
        }

        // For each bus, find the marginal (highest augmented cost) generator
        // that is dispatched and connected to that bus.
        let mut lmp = vec![0.0_f64; n];

        // Build per-bus generator map.
        for (i, gen) in self.generators.iter().enumerate() {
            if dispatch[i] <= gen.p_min_mw + 1e-9 {
                continue; // not dispatched above minimum
            }
            let bus = gen.bus.min(n - 1);
            let aug = w * gen.energy_cost_usd_per_mwh
                + self.config.carbon_price_usd_per_t * gen.co2_rate_t_per_mwh;
            if aug > lmp[bus] {
                lmp[bus] = aug;
            }
        }

        // Propagate system marginal to buses with no local generator.
        let system_lmp: f64 = if lmp.iter().any(|&v| v > 0.0) {
            lmp.iter().cloned().fold(f64::NEG_INFINITY, f64::max)
        } else {
            0.0
        };
        for l in lmp.iter_mut() {
            if *l < 1e-12 {
                *l = system_lmp;
            }
        }

        lmp
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn base_config() -> CarbonOpfConfig {
        CarbonOpfConfig {
            base_mva: 100.0,
            n_buses: 3,
            carbon_limit_t_per_h: None,
            carbon_price_usd_per_t: 0.0,
            renewable_priority: false,
            dual_objective_weight: 1.0, // pure cost
        }
    }

    fn coal_gen(bus: usize, p_min: f64, p_max: f64, cost: f64) -> GeneratorCarbon {
        GeneratorCarbon {
            bus,
            p_max_mw: p_max,
            p_min_mw: p_min,
            energy_cost_usd_per_mwh: cost,
            co2_rate_t_per_mwh: 0.9,
            is_must_run: false,
            p_fixed_mw: None,
        }
    }

    fn solar_gen(bus: usize, p_max: f64) -> GeneratorCarbon {
        GeneratorCarbon {
            bus,
            p_max_mw: p_max,
            p_min_mw: 0.0,
            energy_cost_usd_per_mwh: 5.0,
            co2_rate_t_per_mwh: 0.0,
            is_must_run: false,
            p_fixed_mw: None,
        }
    }

    #[test]
    fn test_no_carbon_limit_matches_economic_dispatch() {
        let mut solver = CarbonOpfSolver::new(base_config());
        solver.set_network(vec![], vec![80.0, 0.0, 0.0]);
        // Cheap generator dispatched first.
        solver.add_generator(coal_gen(0, 0.0, 100.0, 30.0)); // cheap
        solver.add_generator(coal_gen(1, 0.0, 100.0, 80.0)); // expensive
        let res = solver.solve().expect("solve");
        // Cheap gen should carry most of the load.
        assert!(
            res.dispatch_mw[0] >= res.dispatch_mw[1],
            "cheap gen should dispatch ≥ expensive gen: {:?}",
            res.dispatch_mw
        );
    }

    #[test]
    fn test_tight_carbon_cap_increases_renewable() {
        let mut cfg = base_config();
        cfg.carbon_limit_t_per_h = Some(20.0); // tight cap
        let mut solver = CarbonOpfSolver::new(cfg);
        solver.set_network(vec![], vec![80.0, 0.0, 0.0]);
        solver.add_generator(coal_gen(0, 0.0, 100.0, 30.0));
        solver.add_generator(solar_gen(1, 100.0));
        let res = solver.solve().expect("solve with cap");
        // Solar (zero-emission) should be dispatched.
        assert!(
            res.dispatch_mw[1] > 0.0,
            "renewable should be dispatched under tight cap"
        );
        assert!(
            res.total_emissions_t_per_h <= 21.0,
            "emissions {} should be near cap",
            res.total_emissions_t_per_h
        );
    }

    #[test]
    fn test_higher_carbon_price_more_renewable() {
        let make_solver = |carbon_price: f64| {
            let mut cfg = base_config();
            cfg.carbon_price_usd_per_t = carbon_price;
            let mut solver = CarbonOpfSolver::new(cfg);
            solver.set_network(vec![], vec![80.0, 0.0, 0.0]);
            solver.add_generator(coal_gen(0, 0.0, 100.0, 30.0));
            solver.add_generator(solar_gen(1, 100.0));
            solver
        };

        let low = make_solver(0.0).solve().expect("low price solve");
        let high = make_solver(500.0).solve().expect("high price solve");

        // At high carbon price, solar dispatch should increase.
        assert!(
            high.dispatch_mw[1] >= low.dispatch_mw[1],
            "higher carbon price should increase solar dispatch"
        );
    }

    #[test]
    fn test_pareto_front_tradeoff() {
        let mut solver = CarbonOpfSolver::new(base_config());
        solver.set_network(vec![], vec![80.0, 0.0, 0.0]);
        solver.add_generator(coal_gen(0, 0.0, 100.0, 30.0));
        solver.add_generator(solar_gen(1, 100.0));
        let front = solver.pareto_front(5).expect("pareto_front");
        assert_eq!(front.len(), 5, "should return 5 Pareto points");
        // w=0 (pure emission min) vs w=1 (pure cost min) should differ.
        let first = &front[0];
        let last = &front[4];
        // At w=0, emissions minimised; at w=1, cost minimised.
        // They should not both be identical.
        let cost_range = (last.total_cost_usd_per_h - first.total_cost_usd_per_h).abs();
        let emit_range = (last.total_emissions_t_per_h - first.total_emissions_t_per_h).abs();
        assert!(
            cost_range + emit_range > 0.0,
            "Pareto front should show cost-emission trade-off"
        );
    }

    #[test]
    fn test_green_lmp_nonempty() {
        let mut solver = CarbonOpfSolver::new(base_config());
        solver.set_network(vec![], vec![50.0, 0.0, 0.0]);
        solver.add_generator(coal_gen(0, 0.0, 100.0, 40.0));
        let res = solver.solve().expect("solve");
        assert_eq!(
            res.green_lmp.len(),
            3,
            "green_lmp length should equal n_buses"
        );
    }
}
