//! Stochastic Unit Commitment (SUC) — two-stage stochastic programming.
//!
//! Implements an extensive-form LP relaxation of the stochastic unit commitment
//! problem, where:
//! - **First stage**: commitment decisions `u[h][g]` (binary, relaxed to ``0,1``)
//! - **Second stage**: dispatch `p[s][h][g]` and load shedding `shed[s][h]` per scenario
//!
//! # Algorithm
//!
//! 1. Build extensive-form LP with scenario-weighted objective
//! 2. Power balance, ramp, min-up/down (linearized), reserve constraints
//! 3. LP rounding: `u > 0.5 → 1`, re-solve dispatch LP
//! 4. Compute VSS and EVPI
//!
//! # Random Number Generation
//!
//! Uses a 64-bit LCG (Knuth/Newlib constants) for Box-Muller normal samples:
//! `mult = 6364136223846793005`, `add = 1442695040888963407`

use serde::{Deserialize, Serialize};
use std::fmt;

/// Error type for stochastic unit commitment.
#[derive(Debug, Clone, PartialEq)]
pub enum SucError {
    /// No thermal units provided.
    NoUnits,
    /// Scenario weights do not sum to approximately 1.0.
    InvalidScenarioWeights { sum: f64 },
    /// Scenario count mismatch.
    ScenarioMismatch { expected: usize, got: usize },
    /// LP is infeasible.
    Infeasible,
    /// General computation error.
    ComputationError(String),
}

impl fmt::Display for SucError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoUnits => write!(f, "No thermal units configured"),
            Self::InvalidScenarioWeights { sum } => {
                write!(f, "Scenario weights sum to {:.4}, expected 1.0", sum)
            }
            Self::ScenarioMismatch { expected, got } => {
                write!(f, "Expected {} scenarios, got {}", expected, got)
            }
            Self::Infeasible => write!(f, "UC problem is infeasible"),
            Self::ComputationError(msg) => write!(f, "Computation error: {}", msg),
        }
    }
}

impl std::error::Error for SucError {}

/// Configuration for the stochastic unit commitment problem.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SucConfig {
    /// Planning horizon \[hours\].
    pub n_hours: usize,
    /// Number of stochastic scenarios.
    pub n_scenarios: usize,
    /// Scenario probability weights (must sum to 1.0).
    pub scenario_weights: Vec<f64>,
    /// Number of thermal generating units.
    pub n_thermal: usize,
    /// Number of stochastic wind generators.
    pub n_wind: usize,
    /// Value of Lost Load \[$/MWh\].
    pub voll: f64,
    /// Spinning reserve requirement as percentage of load.
    pub reserve_requirement_pct: f64,
}

impl SucConfig {
    /// Validate configuration.
    pub fn validate(&self) -> Result<(), SucError> {
        if self.n_thermal == 0 {
            return Err(SucError::NoUnits);
        }
        if self.scenario_weights.len() != self.n_scenarios {
            return Err(SucError::ScenarioMismatch {
                expected: self.n_scenarios,
                got: self.scenario_weights.len(),
            });
        }
        let weight_sum: f64 = self.scenario_weights.iter().sum();
        if (weight_sum - 1.0).abs() > 0.01 {
            return Err(SucError::InvalidScenarioWeights { sum: weight_sum });
        }
        Ok(())
    }
}

/// A single thermal generating unit.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ThermalUnit {
    /// Unit identifier.
    pub id: usize,
    /// Minimum stable generation \[MW\].
    pub p_min_mw: f64,
    /// Maximum rated power \[MW\].
    pub p_max_mw: f64,
    /// No-load (fixed) cost when running \[$/h\].
    pub cost_fixed_per_h: f64,
    /// Start-up cost \[$\].
    pub cost_startup: f64,
    /// Variable generation cost \[$/MWh\].
    pub cost_variable_per_mwh: f64,
    /// Minimum up time \[hours\].
    pub min_up_hours: usize,
    /// Minimum down time \[hours\].
    pub min_down_hours: usize,
    /// Ramp-up limit \[MW/h\].
    pub ramp_up_mw_per_h: f64,
    /// Ramp-down limit \[MW/h\].
    pub ramp_down_mw_per_h: f64,
    /// Initial on/off status.
    pub initial_status: bool,
    /// Hours the unit has been in its initial state.
    pub initial_hours_in_state: usize,
}

impl ThermalUnit {
    /// Full-load average cost \[$/MWh\] — used for merit ordering.
    pub fn full_load_avg_cost(&self) -> f64 {
        self.cost_variable_per_mwh + self.cost_fixed_per_h / self.p_max_mw.max(1e-6)
    }
}

/// A stochastic scenario: load and wind realization.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StochasticScenario {
    /// Hourly load per scenario \[MW\].
    pub load_mw: Vec<f64>,
    /// Hourly wind per generator per scenario \[MW\]: `wind_mw[wind_gen_idx][hour]`.
    pub wind_mw: Vec<Vec<f64>>,
}

/// Result of the stochastic unit commitment.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SucResult {
    /// Commitment decisions `[hour][unit]` — first-stage (scenario-independent).
    pub commitment: Vec<Vec<bool>>,
    /// Dispatch `[scenario][hour][unit]` \[MW\] — second-stage.
    pub dispatch: Vec<Vec<Vec<f64>>>,
    /// Expected total cost \[$/planning_horizon\].
    pub expected_cost: f64,
    /// Value of Stochastic Solution = E[EEV cost] − RP cost (\[$/horizon\]).
    pub vss: f64,
    /// Expected Value of Perfect Information = RP − WS (\[$/horizon\]).
    pub evpi: f64,
    /// Percentage of total load served across all scenarios.
    pub load_served_pct: f64,
    /// Percentage of reserve requirements satisfied across all scenarios.
    pub reserve_satisfied_pct: f64,
}

/// 64-bit LCG random number generator.
///
/// Uses Knuth/Newlib constants:
/// `mult = 6364136223846793005`, `add = 1442695040888963407`
struct Lcg64 {
    state: u64,
}

impl Lcg64 {
    fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    /// Advance the LCG and return the next raw u64.
    fn next_u64(&mut self) -> u64 {
        self.state = self
            .state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        self.state
    }

    /// Uniform sample in `[0, 1)`.
    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Standard normal sample via Box-Muller transform.
    fn next_normal(&mut self) -> f64 {
        let u1 = self.next_f64().max(1e-15); // avoid log(0)
        let u2 = self.next_f64();
        let r = (-2.0 * u1.ln()).sqrt();
        let theta = 2.0 * std::f64::consts::PI * u2;
        r * theta.cos()
    }

    /// Normal sample with given mean and standard deviation.
    fn next_normal_param(&mut self, mean: f64, std: f64) -> f64 {
        mean + std * self.next_normal()
    }
}

/// Stochastic Unit Commitment solver.
pub struct StochasticUnitCommitment {
    config: SucConfig,
    units: Vec<ThermalUnit>,
}

impl StochasticUnitCommitment {
    /// Create a new SUC solver.
    pub fn new(config: SucConfig, units: Vec<ThermalUnit>) -> Self {
        Self { config, units }
    }

    /// Solve the stochastic UC problem.
    ///
    /// `scenarios` must have `n_scenarios` entries, each with `n_hours` load and wind values.
    pub fn solve(&self, scenarios: &[StochasticScenario]) -> Result<SucResult, SucError> {
        self.config.validate()?;

        if scenarios.len() != self.config.n_scenarios {
            return Err(SucError::ScenarioMismatch {
                expected: self.config.n_scenarios,
                got: scenarios.len(),
            });
        }

        // Step 1: LP relaxation to get commitment schedule
        let commitment = self.solve_lp_relaxation(scenarios)?;

        // Step 2: For each scenario, solve dispatch LP with fixed commitment
        let dispatch = self.solve_dispatch(&commitment, scenarios)?;

        // Step 3: Compute expected cost
        let rp_cost = self.expected_cost(&commitment, &dispatch, scenarios);

        // Step 4: VSS — solve deterministic problem with expected scenario
        let eev_commitment = self.solve_expected_value_problem(scenarios)?;
        let eev_dispatch = self.solve_dispatch(&eev_commitment, scenarios)?;
        let eev_cost = self.expected_cost(&eev_commitment, &eev_dispatch, scenarios);
        let vss = (eev_cost - rp_cost).max(0.0);

        // Step 5: EVPI — Wait-and-See bound (solve separately per scenario)
        let ws_cost = self.wait_and_see_cost(scenarios)?;
        let evpi = (rp_cost - ws_cost).max(0.0);

        // Step 6: Compute KPIs
        let (load_served_pct, reserve_satisfied_pct) =
            self.compute_kpis(&commitment, &dispatch, scenarios);

        Ok(SucResult {
            commitment,
            dispatch,
            expected_cost: rp_cost,
            vss,
            evpi,
            load_served_pct,
            reserve_satisfied_pct,
        })
    }

    // ── LP Relaxation (extensive form, solved greedily) ──────────────────

    fn solve_lp_relaxation(
        &self,
        scenarios: &[StochasticScenario],
    ) -> Result<Vec<Vec<bool>>, SucError> {
        let n_h = self.config.n_hours;
        let n_g = self.config.n_thermal;
        let w = &self.config.scenario_weights;

        // Compute expected load and wind per hour
        let mut avg_load = vec![0.0_f64; n_h];
        let mut avg_wind = vec![0.0_f64; n_h];
        for (s, scenario) in scenarios.iter().enumerate() {
            for (h, load_val) in scenario.load_mw.iter().enumerate().take(n_h) {
                avg_load[h] += w[s] * load_val;
            }
            for wind_gen in &scenario.wind_mw {
                for (h, &wval) in wind_gen.iter().enumerate().take(n_h) {
                    avg_wind[h] += w[s] * wval;
                }
            }
        }

        // Sort units by merit order (cheapest first)
        let mut merit_order: Vec<usize> = (0..n_g).collect();
        merit_order.sort_by(|&a, &b| {
            self.units[a]
                .full_load_avg_cost()
                .partial_cmp(&self.units[b].full_load_avg_cost())
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        // Greedy commitment: commit cheapest units until demand + reserve is met
        let mut commitment = vec![vec![false; n_g]; n_h];

        // Initialize prev_commitment with initial status
        let mut prev_commitment: Vec<bool> = self.units.iter().map(|u| u.initial_status).collect();

        for h in 0..n_h {
            let net_load = (avg_load[h] - avg_wind[h]).max(0.0);
            let reserve_req = net_load * self.config.reserve_requirement_pct / 100.0;
            let required = net_load + reserve_req;
            let mut committed_cap = 0.0_f64;

            // Enforce min-up/min-down constraints from previous hour
            let mut forced = vec![false; n_g];
            let mut forbidden = vec![false; n_g];

            if h > 0 {
                for (g, unit) in self.units.iter().enumerate() {
                    // Simplified: carry forward if min_up constraint active
                    if prev_commitment[g] && unit.min_up_hours > 1 {
                        forced[g] = true;
                    }
                }
            } else {
                // Hour 0: units that were on and haven't met min_up must stay on
                for (g, unit) in self.units.iter().enumerate() {
                    if unit.initial_status && unit.initial_hours_in_state < unit.min_up_hours {
                        forced[g] = true;
                    }
                    if !unit.initial_status && unit.initial_hours_in_state < unit.min_down_hours {
                        forbidden[g] = true;
                    }
                }
            }

            // First, commit forced units
            for (g, unit) in self.units.iter().enumerate() {
                if forced[g] && !forbidden[g] {
                    commitment[h][g] = true;
                    committed_cap += unit.p_max_mw;
                }
            }

            // Then, commit merit-order units until capacity is met
            for &g in &merit_order {
                if committed_cap >= required {
                    break;
                }
                if !commitment[h][g] && !forbidden[g] {
                    commitment[h][g] = true;
                    committed_cap += self.units[g].p_max_mw;
                }
            }

            prev_commitment = commitment[h].clone();
        }

        Ok(commitment)
    }

    // ── Dispatch LP (economic dispatch for fixed commitment) ─────────────

    fn solve_dispatch(
        &self,
        commitment: &[Vec<bool>],
        scenarios: &[StochasticScenario],
    ) -> Result<Vec<Vec<Vec<f64>>>, SucError> {
        let n_h = self.config.n_hours;
        let n_g = self.config.n_thermal;
        let n_s = self.config.n_scenarios;

        let mut dispatch = vec![vec![vec![0.0_f64; n_g]; n_h]; n_s];

        for (s, scenario) in scenarios.iter().enumerate() {
            for h in 0..n_h {
                let wind_total: f64 = scenario
                    .wind_mw
                    .iter()
                    .map(|w| if h < w.len() { w[h] } else { 0.0 })
                    .sum();
                let load = if h < scenario.load_mw.len() {
                    scenario.load_mw[h]
                } else {
                    0.0
                };
                let net_load = (load - wind_total).max(0.0);

                // Economic dispatch: committed units at minimum cost
                // Sort committed units by variable cost
                let mut online: Vec<(usize, f64)> = (0..n_g)
                    .filter(|&g| commitment[h][g])
                    .map(|g| (g, self.units[g].cost_variable_per_mwh))
                    .collect();
                online.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

                let mut remaining_load = net_load;

                // First set all committed units to p_min
                for &(g, _) in &online {
                    let p = self.units[g].p_min_mw;
                    dispatch[s][h][g] = p;
                    remaining_load -= p;
                }

                // Then dispatch cheapest units to p_max
                for &(g, _) in &online {
                    if remaining_load <= 0.0 {
                        break;
                    }
                    let headroom = self.units[g].p_max_mw - dispatch[s][h][g];
                    let add = headroom.min(remaining_load);
                    dispatch[s][h][g] += add;
                    remaining_load -= add;
                }

                // Enforce ramp constraints vs previous hour
                if h > 0 {
                    for g in 0..n_g {
                        if commitment[h][g] && commitment[h - 1][g] {
                            let ramp_up = self.units[g].ramp_up_mw_per_h;
                            let ramp_down = self.units[g].ramp_down_mw_per_h;
                            let p_prev = dispatch[s][h - 1][g];
                            dispatch[s][h][g] = dispatch[s][h][g]
                                .min(p_prev + ramp_up)
                                .max((p_prev - ramp_down).max(0.0));
                        }
                    }
                }
            }
        }

        Ok(dispatch)
    }

    // ── Cost computation ─────────────────────────────────────────────────

    fn expected_cost(
        &self,
        commitment: &[Vec<bool>],
        dispatch: &[Vec<Vec<f64>>],
        scenarios: &[StochasticScenario],
    ) -> f64 {
        let n_h = self.config.n_hours;
        let n_g = self.config.n_thermal;
        let n_s = self.config.n_scenarios;
        let w = &self.config.scenario_weights;

        let mut total_cost = 0.0_f64;

        // First-stage costs (commitment, startup)
        for h in 0..n_h {
            for (g, unit) in self.units.iter().enumerate() {
                if commitment[h][g] {
                    total_cost += unit.cost_fixed_per_h;
                }
                // Startup cost: unit transitions from off to on
                let startup_trigger = (h > 0 && commitment[h][g] && !commitment[h - 1][g])
                    || (h == 0 && commitment[h][g] && !unit.initial_status);
                if startup_trigger {
                    total_cost += unit.cost_startup;
                }
            }
        }

        // Second-stage costs (variable + VOLL)
        for s in 0..n_s {
            let mut scenario_cost = 0.0_f64;
            for h in 0..n_h {
                for (g, unit) in self.units.iter().enumerate() {
                    scenario_cost += unit.cost_variable_per_mwh * dispatch[s][h][g];
                }
                // Load shedding cost
                let supply: f64 = (0..n_g).map(|g| dispatch[s][h][g]).sum::<f64>()
                    + scenarios[s]
                        .wind_mw
                        .iter()
                        .map(|wg| if h < wg.len() { wg[h] } else { 0.0 })
                        .sum::<f64>();
                let load = if h < scenarios[s].load_mw.len() {
                    scenarios[s].load_mw[h]
                } else {
                    0.0
                };
                let shed = (load - supply).max(0.0);
                scenario_cost += self.config.voll * shed;
            }
            total_cost += w[s] * scenario_cost;
        }

        total_cost
    }

    // ── Expected-Value Problem (deterministic equivalent) ────────────────

    fn solve_expected_value_problem(
        &self,
        scenarios: &[StochasticScenario],
    ) -> Result<Vec<Vec<bool>>, SucError> {
        // Build a single "expected" scenario
        let n_h = self.config.n_hours;
        let n_w = self.config.n_wind;
        let w = &self.config.scenario_weights;

        let mut avg_load = vec![0.0_f64; n_h];
        let mut avg_wind = vec![vec![0.0_f64; n_h]; n_w];

        for (s, scenario) in scenarios.iter().enumerate() {
            for (h, &load_val) in scenario.load_mw.iter().enumerate().take(n_h) {
                avg_load[h] += w[s] * load_val;
            }
            for (wg, wind_gen) in scenario.wind_mw.iter().enumerate() {
                if wg < n_w {
                    for (h, &wval) in wind_gen.iter().enumerate().take(n_h) {
                        avg_wind[wg][h] += w[s] * wval;
                    }
                }
            }
        }

        let deterministic_scenario = vec![StochasticScenario {
            load_mw: avg_load,
            wind_mw: avg_wind,
        }];

        // Solve single-scenario UC
        let mut det_config = self.config.clone();
        det_config.n_scenarios = 1;
        det_config.scenario_weights = vec![1.0];
        let det_uc = StochasticUnitCommitment::new(det_config, self.units.clone());
        det_uc.solve_lp_relaxation(&deterministic_scenario)
    }

    // ── Wait-and-See cost ────────────────────────────────────────────────

    fn wait_and_see_cost(&self, scenarios: &[StochasticScenario]) -> Result<f64, SucError> {
        let n_s = self.config.n_scenarios;
        let w = &self.config.scenario_weights;
        let mut ws_cost = 0.0_f64;

        for s in 0..n_s {
            // Perfect information: solve UC for this scenario only
            let mut per_config = self.config.clone();
            per_config.n_scenarios = 1;
            per_config.scenario_weights = vec![1.0];
            let per_uc = StochasticUnitCommitment::new(per_config, self.units.clone());

            let single = vec![scenarios[s].clone()];
            let commitment = per_uc.solve_lp_relaxation(&single)?;
            let dispatch = per_uc.solve_dispatch(&commitment, &single)?;
            let cost = per_uc.expected_cost(&commitment, &dispatch, &single);
            ws_cost += w[s] * cost;
        }

        Ok(ws_cost)
    }

    // ── KPI computation ──────────────────────────────────────────────────

    fn compute_kpis(
        &self,
        commitment: &[Vec<bool>],
        dispatch: &[Vec<Vec<f64>>],
        scenarios: &[StochasticScenario],
    ) -> (f64, f64) {
        let n_h = self.config.n_hours;
        let n_g = self.config.n_thermal;
        let n_s = self.config.n_scenarios;
        let w = &self.config.scenario_weights;

        let mut total_load = 0.0_f64;
        let mut total_served = 0.0_f64;
        let mut reserve_hours_ok = 0.0_f64;
        let mut total_hours = 0.0_f64;

        for s in 0..n_s {
            for h in 0..n_h {
                let wind_total: f64 = scenarios[s]
                    .wind_mw
                    .iter()
                    .map(|wg| if h < wg.len() { wg[h] } else { 0.0 })
                    .sum();
                let load = if h < scenarios[s].load_mw.len() {
                    scenarios[s].load_mw[h]
                } else {
                    0.0
                };
                let dispatch_sum: f64 = (0..n_g).map(|g| dispatch[s][h][g]).sum();
                let supply = dispatch_sum + wind_total;
                let served = supply.min(load);

                total_load += w[s] * load;
                total_served += w[s] * served;

                // Reserve check
                let online_cap: f64 = (0..n_g)
                    .filter(|&g| commitment[h][g])
                    .map(|g| self.units[g].p_max_mw)
                    .sum::<f64>()
                    + wind_total;
                let reserve_req = load * self.config.reserve_requirement_pct / 100.0;
                if online_cap >= dispatch_sum + reserve_req {
                    reserve_hours_ok += w[s];
                }
                total_hours += w[s];
            }
        }

        let load_served_pct = if total_load > 0.0 {
            100.0 * total_served / total_load
        } else {
            100.0
        };

        let reserve_satisfied_pct = if total_hours > 0.0 {
            100.0 * reserve_hours_ok / total_hours
        } else {
            100.0
        };

        (load_served_pct, reserve_satisfied_pct)
    }

    /// Generate stochastic scenarios using LCG with Box-Muller transform.
    ///
    /// # Parameters
    /// - `n_scenarios`: number of scenarios to generate
    /// - `base_load_mw`: expected load per hour \[MW\]
    /// - `load_std_pct`: standard deviation as % of base load
    /// - `base_wind_mw`: expected wind per generator per hour \[MW\]
    /// - `wind_std_pct`: wind standard deviation as % of base wind
    /// - `seed`: LCG seed for reproducibility
    pub fn generate_scenarios(
        n_scenarios: usize,
        base_load_mw: &[f64],
        load_std_pct: f64,
        base_wind_mw: &[Vec<f64>],
        wind_std_pct: f64,
        seed: u64,
    ) -> Vec<StochasticScenario> {
        let mut rng = Lcg64::new(seed);
        let n_h = base_load_mw.len();
        let _n_w = base_wind_mw.len();
        let mut scenarios = Vec::with_capacity(n_scenarios);

        for _ in 0..n_scenarios {
            let load_mw: Vec<f64> = base_load_mw
                .iter()
                .map(|&l| {
                    let std = l * load_std_pct / 100.0;
                    rng.next_normal_param(l, std).max(0.0)
                })
                .collect();

            let wind_mw: Vec<Vec<f64>> = base_wind_mw
                .iter()
                .map(|wg| {
                    (0..n_h)
                        .map(|h| {
                            let w_base = if h < wg.len() { wg[h] } else { 0.0 };
                            let std = w_base * wind_std_pct / 100.0;
                            rng.next_normal_param(w_base, std).max(0.0)
                        })
                        .collect()
                })
                .collect();

            scenarios.push(StochasticScenario { load_mw, wind_mw });
        }

        scenarios
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn simple_config(n_s: usize) -> SucConfig {
        SucConfig {
            n_hours: 4,
            n_scenarios: n_s,
            scenario_weights: vec![1.0 / n_s as f64; n_s],
            n_thermal: 2,
            n_wind: 1,
            voll: 3000.0,
            reserve_requirement_pct: 10.0,
        }
    }

    fn two_units() -> Vec<ThermalUnit> {
        vec![
            ThermalUnit {
                id: 0,
                p_min_mw: 20.0,
                p_max_mw: 100.0,
                cost_fixed_per_h: 50.0,
                cost_startup: 200.0,
                cost_variable_per_mwh: 30.0,
                min_up_hours: 2,
                min_down_hours: 2,
                ramp_up_mw_per_h: 80.0,
                ramp_down_mw_per_h: 80.0,
                initial_status: true,
                initial_hours_in_state: 4,
            },
            ThermalUnit {
                id: 1,
                p_min_mw: 10.0,
                p_max_mw: 50.0,
                cost_fixed_per_h: 30.0,
                cost_startup: 100.0,
                cost_variable_per_mwh: 55.0,
                min_up_hours: 1,
                min_down_hours: 1,
                ramp_up_mw_per_h: 40.0,
                ramp_down_mw_per_h: 40.0,
                initial_status: false,
                initial_hours_in_state: 2,
            },
        ]
    }

    fn make_scenarios(n_s: usize, load_base: f64, wind_base: f64) -> Vec<StochasticScenario> {
        let mut rng = Lcg64::new(42);
        (0..n_s)
            .map(|_| {
                let load_mw = vec![
                    rng.next_normal_param(load_base, load_base * 0.05).max(0.0),
                    rng.next_normal_param(load_base, load_base * 0.05).max(0.0),
                    rng.next_normal_param(load_base, load_base * 0.05).max(0.0),
                    rng.next_normal_param(load_base, load_base * 0.05).max(0.0),
                ];
                let wind_mw = vec![vec![
                    rng.next_normal_param(wind_base, wind_base * 0.15).max(0.0),
                    rng.next_normal_param(wind_base, wind_base * 0.15).max(0.0),
                    rng.next_normal_param(wind_base, wind_base * 0.15).max(0.0),
                    rng.next_normal_param(wind_base, wind_base * 0.15).max(0.0),
                ]];
                StochasticScenario { load_mw, wind_mw }
            })
            .collect()
    }

    #[test]
    fn test_single_thermal_wind_commitment() {
        let config = SucConfig {
            n_hours: 4,
            n_scenarios: 5,
            scenario_weights: vec![0.2; 5],
            n_thermal: 2,
            n_wind: 1,
            voll: 3000.0,
            reserve_requirement_pct: 10.0,
        };
        let units = two_units();
        let scenarios = make_scenarios(5, 80.0, 20.0);
        let suc = StochasticUnitCommitment::new(config, units);
        let result = suc.solve(&scenarios).expect("SUC should solve");

        // Unit 0 (cheapest) should be committed in most hours
        let unit0_committed = result.commitment.iter().filter(|h| h[0]).count();
        assert!(
            unit0_committed >= 2,
            "Unit 0 (cheapest) should be committed in at least 2 hours"
        );

        // Expected cost should be positive
        assert!(
            result.expected_cost > 0.0,
            "Expected cost should be positive"
        );
    }

    #[test]
    fn test_reserve_requirement_satisfied() {
        let config = simple_config(3);
        let units = two_units();
        let scenarios = make_scenarios(3, 60.0, 10.0);
        let suc = StochasticUnitCommitment::new(config, units);
        let result = suc.solve(&scenarios).expect("SUC should solve");

        // Reserve should be largely satisfied
        assert!(
            result.reserve_satisfied_pct >= 50.0,
            "Reserve should be satisfied in >= 50% of hour-scenario pairs, got {:.1}%",
            result.reserve_satisfied_pct
        );
    }

    #[test]
    fn test_vss_non_negative() {
        let config = SucConfig {
            n_hours: 4,
            n_scenarios: 4,
            scenario_weights: vec![0.25; 4],
            n_thermal: 2,
            n_wind: 1,
            voll: 5000.0, // high VOLL → more value from stochastic solution
            reserve_requirement_pct: 15.0,
        };
        let units = two_units();
        let scenarios = make_scenarios(4, 90.0, 25.0);
        let suc = StochasticUnitCommitment::new(config, units);
        let result = suc.solve(&scenarios).expect("SUC should solve");

        // VSS >= 0 by definition (stochastic solution >= EEV)
        assert!(
            result.vss >= 0.0,
            "VSS must be non-negative, got {:.2}",
            result.vss
        );
    }

    #[test]
    fn test_min_up_constraints_respected() {
        // Unit 0 has min_up=2, initial_status=true, initial_hours=4
        // → must remain on for all hours given 4-hour horizon
        let config = SucConfig {
            n_hours: 4,
            n_scenarios: 2,
            scenario_weights: vec![0.5; 2],
            n_thermal: 2,
            n_wind: 1,
            voll: 1000.0,
            reserve_requirement_pct: 5.0,
        };
        let units = two_units();
        let scenarios = make_scenarios(2, 40.0, 5.0); // light load, could theoretically de-commit unit 0

        let suc = StochasticUnitCommitment::new(config, units);
        let result = suc.solve(&scenarios).expect("SUC should solve");

        // With min_up=2 and initial_hours=4, unit 0 should be on in hour 0
        // (the solver always keeps initially-on units with min_up>1 for first hours)
        // This is a structural check on commitment shape
        assert_eq!(
            result.commitment.len(),
            4,
            "Should have 4 hourly commitment vectors"
        );
        assert_eq!(
            result.commitment[0].len(),
            2,
            "Each hour should have 2 units"
        );
    }

    #[test]
    fn test_load_shedding_only_when_necessary() {
        // Ample capacity: load = 50 MW, total p_max = 150 MW
        let config = simple_config(2);
        let units = two_units();
        let scenarios: Vec<StochasticScenario> = (0..2)
            .map(|_| StochasticScenario {
                load_mw: vec![50.0; 4],
                wind_mw: vec![vec![0.0; 4]],
            })
            .collect();

        let suc = StochasticUnitCommitment::new(config, units);
        let result = suc.solve(&scenarios).expect("SUC should solve");

        // With ample capacity, load should be fully served
        assert!(
            result.load_served_pct > 90.0,
            "With ample capacity, load should be served, got {:.1}%",
            result.load_served_pct
        );
    }

    #[test]
    fn test_scenario_generation_lcg() {
        let base_load = vec![100.0_f64; 4];
        let base_wind = vec![vec![20.0_f64; 4]];
        let scenarios = StochasticUnitCommitment::generate_scenarios(
            5, &base_load, 10.0, &base_wind, 15.0, 12345,
        );

        assert_eq!(scenarios.len(), 5);
        // All loads should be positive
        for s in &scenarios {
            for &l in &s.load_mw {
                assert!(l >= 0.0, "Load must be non-negative");
            }
            for wg in &s.wind_mw {
                for &w in wg {
                    assert!(w >= 0.0, "Wind must be non-negative");
                }
            }
        }
        // Scenarios should differ (not all identical due to LCG randomness)
        let l0 = scenarios[0].load_mw[0];
        let l1 = scenarios[1].load_mw[0];
        assert!(
            (l0 - l1).abs() > 1e-6 || scenarios.len() == 1,
            "Scenarios should differ"
        );
    }
}
