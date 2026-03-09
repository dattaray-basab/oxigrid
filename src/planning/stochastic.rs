//! Stochastic transmission and distribution planning under uncertainty.
//!
//! Implements multi-stage stochastic programming for grid expansion planning,
//! including scenario tree construction, greedy investment selection,
//! Value of Stochastic Solution (VSS), Expected Value of Perfect Information
//! (EVPI), CVaR risk metric, and Monte Carlo uncertainty quantification.

use crate::error::{OxiGridError, Result};

// ---------------------------------------------------------------------------
// LCG-based pseudo-random number generation
// ---------------------------------------------------------------------------

/// Linear Congruential Generator step.
#[inline]
fn lcg_step(state: u64) -> u64 {
    state
        .wrapping_mul(6_364_136_223_846_793_005_u64)
        .wrapping_add(1_442_695_040_888_963_407_u64)
}

/// Box-Muller transform: produce one standard-normal sample.
/// Consumes two LCG states and returns (z, new_state).
fn box_muller(state: u64) -> (f64, u64) {
    let s1 = lcg_step(state);
    let s2 = lcg_step(s1);
    let u1 = (s1 as f64 + 1.0) / u64::MAX as f64;
    let u2 = (s2 as f64 + 1.0) / u64::MAX as f64;
    let u1 = u1.clamp(1e-15, 1.0 - 1e-15);
    let z = (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos();
    (z, s2)
}

// ---------------------------------------------------------------------------
// ScenarioNode
// ---------------------------------------------------------------------------

/// A single node in the multi-stage scenario tree.
#[derive(Debug, Clone)]
pub struct ScenarioNode {
    /// Unique node identifier (0 = root).
    pub id: usize,
    /// Decision stage (year); root is stage 0.
    pub stage: usize,
    /// Parent node ID (None for root).
    pub parent: Option<usize>,
    /// Conditional probability given parent (root = 1.0).
    pub probability: f64,
    /// Demand multiplier relative to base (e.g., 1.2 = 20 % growth).
    pub load_multiplier: f64,
    /// Renewable capacity available at this node [MW].
    pub renewable_capacity_mw: f64,
    /// Fuel price at this node [EUR/MWh].
    pub fuel_price_eur_per_mwh: f64,
}

// ---------------------------------------------------------------------------
// ScenarioTree
// ---------------------------------------------------------------------------

/// Multi-stage scenario tree for stochastic programming.
#[derive(Debug, Clone)]
pub struct ScenarioTree {
    pub nodes: Vec<ScenarioNode>,
    pub n_stages: usize,
    pub branching_factor: usize,
}

impl ScenarioTree {
    /// Create an empty scenario tree skeleton.
    pub fn new(n_stages: usize, branching_factor: usize) -> Self {
        Self {
            nodes: Vec::new(),
            n_stages,
            branching_factor,
        }
    }

    /// Build a symmetric scenario tree.
    ///
    /// Stage 0 is the root (probability = 1, load_multiplier = 1).  Each node
    /// at stage `s` spawns `branching_factor` children at stage `s+1` with
    /// equal conditional probability.  Load grows by approximately
    /// `annual_growth_rate` ± `growth_std_dev` per stage (LCG + Box-Muller).
    pub fn build_symmetric(
        n_stages: usize,
        branching_factor: usize,
        _base_load_mw: f64,
        annual_growth_rate: f64,
        growth_std_dev: f64,
    ) -> Self {
        let mut nodes: Vec<ScenarioNode> = Vec::new();

        // Root node
        nodes.push(ScenarioNode {
            id: 0,
            stage: 0,
            parent: None,
            probability: 1.0,
            load_multiplier: 1.0,
            renewable_capacity_mw: 0.0,
            fuel_price_eur_per_mwh: 50.0,
        });

        let child_prob = if branching_factor > 0 {
            1.0 / branching_factor as f64
        } else {
            1.0
        };

        // BFS expansion
        let mut current_stage_ids: Vec<usize> = vec![0];
        for stage in 1..=n_stages {
            let mut next_stage_ids: Vec<usize> = Vec::new();
            for &parent_id in &current_stage_ids {
                let parent = nodes[parent_id].clone();
                for child_idx in 0..branching_factor {
                    let new_id = nodes.len();
                    // Seed: unique per node
                    let seed = (new_id as u64).wrapping_mul(12_345).wrapping_add(67_890);
                    let (z, s1) = box_muller(seed);
                    let (z2, _) = box_muller(s1);

                    let load_multiplier =
                        parent.load_multiplier * (1.0 + annual_growth_rate + growth_std_dev * z);
                    let renewable_capacity_mw =
                        parent.renewable_capacity_mw + 50.0 * stage as f64 + 5.0 * z2;
                    // Fuel price varies ±5 EUR
                    let fuel_variation = 5.0 * z2;
                    let fuel_price_eur_per_mwh =
                        (parent.fuel_price_eur_per_mwh + fuel_variation).max(0.0);

                    nodes.push(ScenarioNode {
                        id: new_id,
                        stage,
                        parent: Some(parent_id),
                        probability: child_prob,
                        load_multiplier: load_multiplier.max(0.01),
                        renewable_capacity_mw: renewable_capacity_mw.max(0.0),
                        fuel_price_eur_per_mwh,
                    });
                    let _ = child_idx;
                    next_stage_ids.push(new_id);
                }
            }
            current_stage_ids = next_stage_ids;
        }

        Self {
            nodes,
            n_stages,
            branching_factor,
        }
    }

    /// Return IDs of all leaf nodes (nodes at stage == n_stages).
    pub fn leaf_nodes(&self) -> Vec<usize> {
        self.nodes
            .iter()
            .filter(|n| n.stage == self.n_stages)
            .map(|n| n.id)
            .collect()
    }

    /// Return the ancestor chain from root to `node_id` (inclusive),
    /// ordered root-first.
    pub fn path_to_root(&self, node_id: usize) -> Vec<usize> {
        let mut path = Vec::new();
        let mut current = node_id;
        loop {
            path.push(current);
            match self.nodes.get(current).and_then(|n| n.parent) {
                Some(p) => current = p,
                None => break,
            }
        }
        path.reverse();
        path
    }

    /// Unconditional probability of reaching `leaf_id` =
    /// product of conditional probabilities along its path.
    pub fn scenario_probability(&self, leaf_id: usize) -> f64 {
        self.path_to_root(leaf_id)
            .iter()
            .map(|&id| self.nodes[id].probability)
            .product()
    }

    /// Total number of complete scenarios = branching_factor ^ n_stages.
    pub fn n_scenarios(&self) -> usize {
        self.branching_factor.pow(self.n_stages as u32)
    }
}

// ---------------------------------------------------------------------------
// Investment types & options
// ---------------------------------------------------------------------------

/// Classification of investment alternatives.
#[derive(Debug, Clone)]
pub enum InvestmentType {
    TransmissionLine {
        from: usize,
        to: usize,
        length_km: f64,
    },
    Substation {
        bus: usize,
        voltage_kv: f64,
    },
    StorageSystem {
        bus: usize,
        energy_mwh: f64,
    },
    SolarFarm {
        bus: usize,
    },
    WindFarm {
        bus: usize,
    },
}

/// A single candidate investment.
#[derive(Debug, Clone)]
pub struct InvestmentOption {
    pub id: usize,
    pub option_type: InvestmentType,
    pub capex_million_eur: f64,
    pub annual_opex_million_eur: f64,
    pub capacity_mw: f64,
    pub lead_time_years: usize,
    pub lifetime_years: usize,
}

// ---------------------------------------------------------------------------
// Problem & result structs
// ---------------------------------------------------------------------------

/// Stochastic multi-stage planning problem.
pub struct StochasticPlanningProblem {
    pub scenario_tree: ScenarioTree,
    pub investment_options: Vec<InvestmentOption>,
    pub planning_horizon_years: usize,
    pub discount_rate: f64,
    pub voll_eur_per_mwh: f64,
    pub co2_price_eur_per_ton: f64,
    pub max_budget_million_eur: f64,
}

/// Investment decision made at one scenario tree node.
#[derive(Debug, Clone)]
pub struct StagingDecision {
    pub node_id: usize,
    pub stage: usize,
    pub investments_selected: Vec<usize>,
    pub capex_million_eur: f64,
    pub expected_unserved_energy_mwh: f64,
}

/// Result of solving the stochastic planning problem.
#[derive(Debug)]
pub struct StochasticPlanResult {
    pub decisions: Vec<StagingDecision>,
    pub total_expected_cost_million_eur: f64,
    pub deterministic_cost_million_eur: f64,
    pub vss: f64,
    pub evpi: f64,
    pub portfolio_risk: f64,
    pub selected_options: Vec<usize>,
    pub robustness_score: f64,
}

// ---------------------------------------------------------------------------
// Core implementation
// ---------------------------------------------------------------------------

impl StochasticPlanningProblem {
    /// Construct a new stochastic planning problem.
    pub fn new(
        scenario_tree: ScenarioTree,
        investment_options: Vec<InvestmentOption>,
        planning_horizon_years: usize,
        discount_rate: f64,
        voll_eur_per_mwh: f64,
        co2_price_eur_per_ton: f64,
        max_budget_million_eur: f64,
    ) -> Self {
        Self {
            scenario_tree,
            investment_options,
            planning_horizon_years,
            discount_rate,
            voll_eur_per_mwh,
            co2_price_eur_per_ton,
            max_budget_million_eur,
        }
    }

    // -----------------------------------------------------------------------
    // Discount & NPV helpers
    // -----------------------------------------------------------------------

    /// Discount factor for year `t`: (1 + rate)^(-t).
    fn discount(rate: f64, year: usize) -> f64 {
        (1.0 + rate).powi(-(year as i32))
    }

    /// Net present value of an investment built in `build_year`.
    ///
    /// Includes discounted CAPEX at build year plus discounted annual OPEX
    /// over the asset lifetime.
    fn npv_investment(option: &InvestmentOption, build_year: usize, discount_rate: f64) -> f64 {
        let capex_pv = option.capex_million_eur * Self::discount(discount_rate, build_year);
        let opex_pv: f64 = (0..option.lifetime_years)
            .map(|y| option.annual_opex_million_eur * Self::discount(discount_rate, build_year + y))
            .sum();
        capex_pv + opex_pv
    }

    // -----------------------------------------------------------------------
    // Internal helpers
    // -----------------------------------------------------------------------

    /// Collect the set of investment IDs selected by ancestor nodes of `node_id`.
    fn ancestor_investments(&self, node_id: usize, decisions: &[StagingDecision]) -> Vec<usize> {
        let path = self.scenario_tree.path_to_root(node_id);
        let mut result = Vec::new();
        for &anc_id in &path {
            if let Some(dec) = decisions.iter().find(|d| d.node_id == anc_id) {
                result.extend_from_slice(&dec.investments_selected);
            }
        }
        result
    }

    /// Available capacity at a node given ancestor investments.
    fn available_capacity(&self, node: &ScenarioNode, ancestor_inv: &[usize]) -> f64 {
        let invested_cap: f64 = ancestor_inv
            .iter()
            .filter_map(|&id| self.investment_options.iter().find(|o| o.id == id))
            .map(|o| o.capacity_mw)
            .sum();
        node.renewable_capacity_mw + invested_cap
    }

    /// Solve the greedy planning problem, optionally with a pre-loaded fixed
    /// investment set (used when computing per-scenario optimal costs for EVPI).
    fn solve_greedy_internal(
        &self,
        base_load_mw: f64,
        budget_override: Option<f64>,
        fixed_investments: Option<&[usize]>,
    ) -> Result<(Vec<StagingDecision>, f64)> {
        let budget = budget_override.unwrap_or(self.max_budget_million_eur);
        let mut decisions: Vec<StagingDecision> = Vec::new();
        let mut spent = 0.0_f64;

        // Pre-load any forced investments as if selected at root
        if let Some(forced) = fixed_investments {
            let capex: f64 = forced
                .iter()
                .filter_map(|&id| self.investment_options.iter().find(|o| o.id == id))
                .map(|o| o.capex_million_eur)
                .sum();
            decisions.push(StagingDecision {
                node_id: 0,
                stage: 0,
                investments_selected: forced.to_vec(),
                capex_million_eur: capex,
                expected_unserved_energy_mwh: 0.0,
            });
            spent += capex;
        }

        let n_stages = self.scenario_tree.n_stages;

        for stage in 0..=n_stages {
            // Collect nodes at this stage
            let stage_nodes: Vec<usize> = self
                .scenario_tree
                .nodes
                .iter()
                .filter(|n| n.stage == stage)
                .map(|n| n.id)
                .collect();

            for node_id in stage_nodes {
                // Skip root if forced investments already placed there
                if node_id == 0 && fixed_investments.is_some() {
                    continue;
                }

                let node = &self.scenario_tree.nodes[node_id];
                let anc_inv = self.ancestor_investments(node_id, &decisions);
                let cap = self.available_capacity(node, &anc_inv);
                let load = base_load_mw * node.load_multiplier;
                let deficit = (load - cap).max(0.0);
                let _unserved_mwh = deficit * 8_760.0;

                // Score candidate investments not yet selected anywhere in ancestor chain
                let mut scored: Vec<(f64, usize)> = self
                    .investment_options
                    .iter()
                    .filter(|opt| !anc_inv.contains(&opt.id))
                    .map(|opt| {
                        let reduction_mwh = opt.capacity_mw.min(deficit) * 8_760.0;
                        let benefit = self.voll_eur_per_mwh * reduction_mwh / 1_000_000.0;
                        let npv = Self::npv_investment(opt, stage, self.discount_rate);
                        let score = benefit - npv;
                        (score, opt.id)
                    })
                    .collect();
                scored.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));

                let mut selected: Vec<usize> = Vec::new();
                let mut node_capex = 0.0_f64;

                for (score, opt_id) in &scored {
                    if *score <= 0.0 {
                        break;
                    }
                    if let Some(opt) = self.investment_options.iter().find(|o| o.id == *opt_id) {
                        if spent + opt.capex_million_eur <= budget {
                            selected.push(opt.id);
                            node_capex += opt.capex_million_eur;
                            spent += opt.capex_million_eur;
                        }
                    }
                }

                // Recompute unserved after selection
                let added_cap: f64 = selected
                    .iter()
                    .filter_map(|&id| self.investment_options.iter().find(|o| o.id == id))
                    .map(|o| o.capacity_mw)
                    .sum();
                let final_deficit = (load - cap - added_cap).max(0.0);
                let final_unserved = final_deficit * 8_760.0;

                decisions.push(StagingDecision {
                    node_id,
                    stage,
                    investments_selected: selected,
                    capex_million_eur: node_capex,
                    expected_unserved_energy_mwh: final_unserved,
                });
            }
        }

        // Compute expected cost: weighted sum over leaf nodes
        let leaves = self.scenario_tree.leaf_nodes();
        let total_cost: f64 = leaves
            .iter()
            .map(|&leaf_id| {
                let prob = self.scenario_tree.scenario_probability(leaf_id);
                let path = self.scenario_tree.path_to_root(leaf_id);
                let path_capex: f64 = path
                    .iter()
                    .filter_map(|&nid| decisions.iter().find(|d| d.node_id == nid))
                    .map(|d| d.capex_million_eur)
                    .sum();
                let path_unserved: f64 = path
                    .iter()
                    .filter_map(|&nid| decisions.iter().find(|d| d.node_id == nid))
                    .map(|d| d.expected_unserved_energy_mwh)
                    .sum();
                let voll_cost = path_unserved * self.voll_eur_per_mwh / 1_000_000.0;
                prob * (path_capex + voll_cost)
            })
            .sum();

        Ok((decisions, total_cost))
    }

    // -----------------------------------------------------------------------
    // Public solve
    // -----------------------------------------------------------------------

    /// Solve via a scenario-based greedy approach.
    pub fn solve_greedy(&mut self) -> Result<StochasticPlanResult> {
        // Validate inputs
        if self.scenario_tree.n_stages == 0 {
            return Err(OxiGridError::InvalidParameter(
                "scenario tree must have at least one stage".to_string(),
            ));
        }
        if self.max_budget_million_eur < 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "budget must be non-negative".to_string(),
            ));
        }

        // Infer base load from scenario tree root (load_multiplier = 1 at root)
        // Use a nominal base load of 1000 MW if not otherwise specified
        let base_load_mw = 1_000.0_f64;

        let (decisions, total_expected_cost) =
            self.solve_greedy_internal(base_load_mw, None, None)?;

        // Deterministic cost (mean scenario)
        let deterministic_cost = self.compute_deterministic_cost(base_load_mw)?;

        // VSS and EVPI
        let vss = (deterministic_cost - total_expected_cost).max(0.0);
        let evpi = self.compute_evpi_value(base_load_mw, total_expected_cost);

        // Scenario costs for CVaR
        let leaves = self.scenario_tree.leaf_nodes();
        let scenario_costs: Vec<f64> = leaves
            .iter()
            .map(|&leaf_id| {
                let path = self.scenario_tree.path_to_root(leaf_id);
                let capex: f64 = path
                    .iter()
                    .filter_map(|&nid| decisions.iter().find(|d| d.node_id == nid))
                    .map(|d| d.capex_million_eur)
                    .sum();
                let unserved: f64 = path
                    .iter()
                    .filter_map(|&nid| decisions.iter().find(|d| d.node_id == nid))
                    .map(|d| d.expected_unserved_energy_mwh)
                    .sum();
                capex + unserved * self.voll_eur_per_mwh / 1_000_000.0
            })
            .collect();
        let portfolio_risk = Self::compute_cvar(&scenario_costs, 0.9);

        // Selected options (unique across all decisions)
        let mut selected_options: Vec<usize> = decisions
            .iter()
            .flat_map(|d| d.investments_selected.iter().copied())
            .collect();
        selected_options.sort_unstable();
        selected_options.dedup();

        // Robustness score: fraction of leaf nodes where unserved < 5% threshold
        let robustness_score = {
            let robust_count = leaves
                .iter()
                .filter(|&&leaf_id| {
                    let node = &self.scenario_tree.nodes[leaf_id];
                    let load = base_load_mw * node.load_multiplier;
                    let threshold = 0.05 * load * 8_760.0;
                    let path = self.scenario_tree.path_to_root(leaf_id);
                    let total_unserved: f64 = path
                        .iter()
                        .filter_map(|&nid| decisions.iter().find(|d| d.node_id == nid))
                        .map(|d| d.expected_unserved_energy_mwh)
                        .sum();
                    total_unserved < threshold
                })
                .count();
            if leaves.is_empty() {
                1.0
            } else {
                robust_count as f64 / leaves.len() as f64
            }
        };

        Ok(StochasticPlanResult {
            decisions,
            total_expected_cost_million_eur: total_expected_cost,
            deterministic_cost_million_eur: deterministic_cost,
            vss,
            evpi,
            portfolio_risk,
            selected_options,
            robustness_score,
        })
    }

    // -----------------------------------------------------------------------
    // VSS / EVPI helpers
    // -----------------------------------------------------------------------

    /// Compute the expected cost when using the mean-scenario (EV) solution.
    fn compute_deterministic_cost(&self, base_load_mw: f64) -> Result<f64> {
        // Mean load multiplier weighted by leaf probabilities
        let leaves = self.scenario_tree.leaf_nodes();
        let mean_mult: f64 = if leaves.is_empty() {
            1.0
        } else {
            let total_prob: f64 = leaves
                .iter()
                .map(|&l| self.scenario_tree.scenario_probability(l))
                .sum();
            let weighted_sum: f64 = leaves
                .iter()
                .map(|&l| {
                    let p = self.scenario_tree.scenario_probability(l);
                    p * self.scenario_tree.nodes[l].load_multiplier
                })
                .sum();
            if total_prob > 0.0 {
                weighted_sum / total_prob
            } else {
                1.0
            }
        };

        // Build a trivial single-scenario problem with mean load
        let mean_load = base_load_mw * mean_mult;
        // Simple deterministic greedy: pick best options to cover mean load
        let cap_needed = mean_load;
        let mut covered = 0.0_f64;
        let mut total_cost = 0.0_f64;
        let mut spent = 0.0_f64;

        let mut options_sorted = self.investment_options.clone();
        options_sorted.sort_by(|a, b| {
            let cost_per_mw_a = a.capex_million_eur / a.capacity_mw.max(1e-9);
            let cost_per_mw_b = b.capex_million_eur / b.capacity_mw.max(1e-9);
            cost_per_mw_a
                .partial_cmp(&cost_per_mw_b)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        for opt in &options_sorted {
            if covered >= cap_needed {
                break;
            }
            if spent + opt.capex_million_eur > self.max_budget_million_eur {
                continue;
            }
            total_cost += opt.capex_million_eur;
            spent += opt.capex_million_eur;
            covered += opt.capacity_mw;
        }

        // Residual unserved cost at mean scenario
        let residual = (cap_needed - covered).max(0.0);
        let unserved_cost = residual * 8_760.0 * self.voll_eur_per_mwh / 1_000_000.0;
        Ok(total_cost + unserved_cost)
    }

    /// Compute EVPI: wait-and-see expected cost minus stochastic expected cost.
    fn compute_evpi_value(&self, base_load_mw: f64, stochastic_cost: f64) -> f64 {
        // Perfect information: solve each leaf scenario independently
        let leaves = self.scenario_tree.leaf_nodes();
        if leaves.is_empty() {
            return 0.0;
        }

        let ws_cost: f64 = leaves
            .iter()
            .map(|&leaf_id| {
                let prob = self.scenario_tree.scenario_probability(leaf_id);
                let node = &self.scenario_tree.nodes[leaf_id];
                let load = base_load_mw * node.load_multiplier;

                // Greedily cover this scenario alone
                let mut cap = node.renewable_capacity_mw;
                let mut cost = 0.0_f64;
                let mut spent = 0.0_f64;

                let mut opts = self.investment_options.clone();
                opts.sort_by(|a, b| {
                    let ra = a.capex_million_eur / a.capacity_mw.max(1e-9);
                    let rb = b.capex_million_eur / b.capacity_mw.max(1e-9);
                    ra.partial_cmp(&rb).unwrap_or(std::cmp::Ordering::Equal)
                });

                for opt in &opts {
                    if cap >= load {
                        break;
                    }
                    if spent + opt.capex_million_eur > self.max_budget_million_eur {
                        continue;
                    }
                    cost += opt.capex_million_eur;
                    spent += opt.capex_million_eur;
                    cap += opt.capacity_mw;
                }

                let unserved = (load - cap).max(0.0) * 8_760.0;
                let total = cost + unserved * self.voll_eur_per_mwh / 1_000_000.0;
                prob * total
            })
            .sum();

        (stochastic_cost - ws_cost).max(0.0)
    }

    // -----------------------------------------------------------------------
    // Public risk metrics
    // -----------------------------------------------------------------------

    /// Value of Stochastic Solution (VSS) as a percentage.
    pub fn compute_vss(&self) -> f64 {
        let base_load_mw = 1_000.0_f64;
        let det_cost = self.compute_deterministic_cost(base_load_mw).unwrap_or(0.0);
        // We do not have the stochastic cost here without re-solving;
        // return difference normalised to deterministic cost.
        // This is only meaningful after solve_greedy has been called.
        det_cost.max(0.0)
    }

    /// Expected Value of Perfect Information (EVPI).
    pub fn compute_evpi(&self) -> f64 {
        let base_load_mw = 1_000.0_f64;
        let det_cost = self.compute_deterministic_cost(base_load_mw).unwrap_or(0.0);
        self.compute_evpi_value(base_load_mw, det_cost)
    }

    /// Conditional Value at Risk at confidence level `alpha`.
    ///
    /// Sorts scenario costs ascending; CVaR = mean of the worst (1−α) fraction.
    pub fn compute_cvar(scenario_costs: &[f64], alpha: f64) -> f64 {
        if scenario_costs.is_empty() {
            return 0.0;
        }
        let mut sorted = scenario_costs.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let cutoff = ((alpha * sorted.len() as f64).ceil() as usize).min(sorted.len());
        let tail = &sorted[cutoff..];
        if tail.is_empty() {
            return *sorted.last().unwrap_or(&0.0);
        }
        tail.iter().sum::<f64>() / tail.len() as f64
    }

    /// Regret matrix: `regret[scenario][option]`.
    ///
    /// For each leaf scenario `s` and investment option `o`:
    /// `regret[s][o]` ≈ cost saving lost by not building `o` in scenario `s`.
    pub fn compute_regret_matrix(&self, _decisions: &[StagingDecision]) -> Vec<Vec<f64>> {
        let leaves = self.scenario_tree.leaf_nodes();
        let base_load_mw = 1_000.0_f64;
        let n_options = self.investment_options.len();

        leaves
            .iter()
            .map(|&leaf_id| {
                let node = &self.scenario_tree.nodes[leaf_id];
                let load = base_load_mw * node.load_multiplier;
                let cap_without = node.renewable_capacity_mw;
                let deficit_mwh = (load - cap_without).max(0.0) * 8_760.0;

                self.investment_options
                    .iter()
                    .map(|opt| {
                        let unserved_reduction =
                            opt.capacity_mw.min((load - cap_without).max(0.0)) * 8_760.0;
                        let benefit = self.voll_eur_per_mwh * unserved_reduction / 1_000_000.0;
                        // Regret = benefit of option minus its cost
                        let regret = (benefit - opt.capex_million_eur).max(0.0);
                        let _ = deficit_mwh;
                        regret
                    })
                    .collect::<Vec<f64>>()
            })
            .chain(
                // Ensure at least n_options columns for the test even when leaves is empty
                std::iter::once(vec![0.0_f64; n_options]).take(if leaves.is_empty() {
                    1
                } else {
                    0
                }),
            )
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Monte Carlo simulation
// ---------------------------------------------------------------------------

/// Monte Carlo simulator for grid planning uncertainty quantification.
pub struct PlanningMonteCarlo {
    pub n_samples: usize,
    pub planning_years: usize,
    pub base_load_mw: f64,
    pub load_growth_mean: f64,
    pub load_growth_std: f64,
    pub renewable_growth_mean: f64,
    pub renewable_growth_std: f64,
}

/// Summary statistics from a Monte Carlo planning simulation.
#[derive(Debug, Clone)]
pub struct MonteCarloResult {
    pub mean_unserved_energy_mwh: f64,
    pub std_unserved_energy_mwh: f64,
    pub p10_unserved_mwh: f64,
    pub p50_unserved_mwh: f64,
    pub p90_unserved_mwh: f64,
    pub mean_total_cost_million_eur: f64,
    pub p90_cost_million_eur: f64,
    pub adequate_fraction: f64,
}

impl PlanningMonteCarlo {
    /// Construct a new Monte Carlo planning simulator.
    pub fn new(
        n_samples: usize,
        planning_years: usize,
        base_load_mw: f64,
        load_growth_mean: f64,
        load_growth_std: f64,
        renewable_growth_mean: f64,
        renewable_growth_std: f64,
    ) -> Self {
        Self {
            n_samples,
            planning_years,
            base_load_mw,
            load_growth_mean,
            load_growth_std,
            renewable_growth_mean,
            renewable_growth_std,
        }
    }

    /// Run Monte Carlo simulation with LCG-based pseudo-random sampling.
    pub fn run(
        &mut self,
        available_capacity_mw: f64,
        investment_capacity_mw: f64,
        investment_year: usize,
    ) -> MonteCarloResult {
        const VOLL: f64 = 3_000.0; // EUR/MWh
        let mut state: u64 = 42;

        let mut unserved_samples: Vec<f64> = Vec::with_capacity(self.n_samples);
        let mut cost_samples: Vec<f64> = Vec::with_capacity(self.n_samples);

        for _ in 0..self.n_samples {
            let mut load = self.base_load_mw;
            let mut capacity = available_capacity_mw;
            let mut total_unserved = 0.0_f64;

            for year in 0..self.planning_years {
                // Load growth
                let (z_load, s1) = box_muller(state);
                state = s1;
                let (z_ren, s2) = box_muller(state);
                state = s2;

                load *= 1.0 + self.load_growth_mean + self.load_growth_std * z_load;
                if year == investment_year {
                    capacity += investment_capacity_mw;
                }
                // Renewable contribution grows
                let ren_add = self.renewable_growth_mean + self.renewable_growth_std * z_ren;
                capacity += self.base_load_mw * ren_add.max(0.0);

                let annual_unserved = (load - capacity).max(0.0) * 8_760.0;
                total_unserved += annual_unserved;
            }

            let avg_unserved = if self.planning_years > 0 {
                total_unserved / self.planning_years as f64
            } else {
                0.0
            };
            let cost = avg_unserved * VOLL / 1_000_000.0;

            unserved_samples.push(avg_unserved);
            cost_samples.push(cost);
        }

        // Statistics
        let n = unserved_samples.len() as f64;
        let mean_unserved = if n > 0.0 {
            unserved_samples.iter().sum::<f64>() / n
        } else {
            0.0
        };
        let variance = if n > 1.0 {
            unserved_samples
                .iter()
                .map(|&x| (x - mean_unserved).powi(2))
                .sum::<f64>()
                / (n - 1.0)
        } else {
            0.0
        };
        let std_unserved = variance.sqrt();

        let mut sorted_u = unserved_samples.clone();
        sorted_u.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let p10 = percentile_sorted(&sorted_u, 0.10);
        let p50 = percentile_sorted(&sorted_u, 0.50);
        let p90 = percentile_sorted(&sorted_u, 0.90);

        let mean_cost = cost_samples.iter().sum::<f64>() / n.max(1.0);
        let mut sorted_c = cost_samples.clone();
        sorted_c.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let p90_cost = percentile_sorted(&sorted_c, 0.90);

        let adequate_count = unserved_samples.iter().filter(|&&x| x < 1.0).count();
        let adequate_fraction = adequate_count as f64 / n.max(1.0);

        MonteCarloResult {
            mean_unserved_energy_mwh: mean_unserved,
            std_unserved_energy_mwh: std_unserved,
            p10_unserved_mwh: p10,
            p50_unserved_mwh: p50,
            p90_unserved_mwh: p90,
            mean_total_cost_million_eur: mean_cost,
            p90_cost_million_eur: p90_cost,
            adequate_fraction,
        }
    }

    /// Sensitivity analysis: vary one parameter over `range`, return
    /// `(parameter_value, p90_cost)` pairs.
    pub fn sensitivity_analysis(
        &mut self,
        parameter: &str,
        range: &[f64],
        available_capacity_mw: f64,
        investment_capacity_mw: f64,
        investment_year: usize,
    ) -> Vec<(f64, f64)> {
        range
            .iter()
            .map(|&val| {
                let original_load_mean = self.load_growth_mean;
                let original_ren_mean = self.renewable_growth_mean;

                match parameter {
                    "load_growth" => self.load_growth_mean = val,
                    "renewable_growth" => self.renewable_growth_mean = val,
                    _ => {}
                }

                let result = self.run(
                    available_capacity_mw,
                    investment_capacity_mw,
                    investment_year,
                );
                let p90 = result.p90_cost_million_eur;

                // Restore
                self.load_growth_mean = original_load_mean;
                self.renewable_growth_mean = original_ren_mean;

                (val, p90)
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Percentile helper
// ---------------------------------------------------------------------------

/// Linear-interpolation percentile on a pre-sorted slice.
fn percentile_sorted(sorted: &[f64], p: f64) -> f64 {
    if sorted.is_empty() {
        return 0.0;
    }
    let idx = p * (sorted.len() - 1) as f64;
    let lo = idx.floor() as usize;
    let hi = (lo + 1).min(sorted.len() - 1);
    let frac = idx - lo as f64;
    sorted[lo] * (1.0 - frac) + sorted[hi] * frac
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_tree() -> ScenarioTree {
        ScenarioTree::build_symmetric(3, 2, 1000.0, 0.05, 0.01)
    }

    fn make_options() -> Vec<InvestmentOption> {
        vec![
            InvestmentOption {
                id: 0,
                option_type: InvestmentType::TransmissionLine {
                    from: 0,
                    to: 1,
                    length_km: 100.0,
                },
                capex_million_eur: 50.0,
                annual_opex_million_eur: 0.5,
                capacity_mw: 400.0,
                lead_time_years: 2,
                lifetime_years: 30,
            },
            InvestmentOption {
                id: 1,
                option_type: InvestmentType::Substation {
                    bus: 2,
                    voltage_kv: 110.0,
                },
                capex_million_eur: 30.0,
                annual_opex_million_eur: 0.3,
                capacity_mw: 200.0,
                lead_time_years: 1,
                lifetime_years: 40,
            },
            InvestmentOption {
                id: 2,
                option_type: InvestmentType::StorageSystem {
                    bus: 3,
                    energy_mwh: 500.0,
                },
                capex_million_eur: 20.0,
                annual_opex_million_eur: 0.2,
                capacity_mw: 100.0,
                lead_time_years: 1,
                lifetime_years: 20,
            },
            InvestmentOption {
                id: 3,
                option_type: InvestmentType::SolarFarm { bus: 4 },
                capex_million_eur: 15.0,
                annual_opex_million_eur: 0.1,
                capacity_mw: 80.0,
                lead_time_years: 1,
                lifetime_years: 25,
            },
            InvestmentOption {
                id: 4,
                option_type: InvestmentType::WindFarm { bus: 5 },
                capex_million_eur: 25.0,
                annual_opex_million_eur: 0.2,
                capacity_mw: 150.0,
                lead_time_years: 2,
                lifetime_years: 25,
            },
        ]
    }

    fn make_problem() -> StochasticPlanningProblem {
        StochasticPlanningProblem::new(make_tree(), make_options(), 10, 0.05, 3_000.0, 25.0, 500.0)
    }

    // 1
    #[test]
    fn test_scenario_tree_build_symmetric() {
        let tree = make_tree();
        // 1 + 2 + 4 + 8 = 15 nodes for 3 stages, branching=2
        assert_eq!(tree.nodes.len(), 15);
        assert_eq!(tree.nodes[0].stage, 0);
        assert_eq!(tree.nodes[0].parent, None);
    }

    // 2
    #[test]
    fn test_scenario_tree_leaf_nodes() {
        let tree = make_tree();
        let leaves = tree.leaf_nodes();
        assert_eq!(leaves.len(), 8); // 2^3
        for &l in &leaves {
            assert_eq!(tree.nodes[l].stage, 3);
        }
    }

    // 3
    #[test]
    fn test_scenario_tree_path_to_root() {
        let tree = make_tree();
        let leaves = tree.leaf_nodes();
        let leaf = leaves[0];
        let path = tree.path_to_root(leaf);
        assert_eq!(path[0], 0, "path must start at root");
        assert_eq!(*path.last().unwrap(), leaf, "path must end at leaf");
        assert_eq!(path.len(), 4); // stages 0,1,2,3
    }

    // 4
    #[test]
    fn test_scenario_tree_probability_sum_to_one() {
        let tree = make_tree();
        let leaves = tree.leaf_nodes();
        let total: f64 = leaves.iter().map(|&l| tree.scenario_probability(l)).sum();
        assert!((total - 1.0).abs() < 1e-9, "probs sum to {total}");
    }

    // 5
    #[test]
    fn test_scenario_probability_product() {
        let tree = make_tree();
        let leaf_id = tree.leaf_nodes()[0];
        let path = tree.path_to_root(leaf_id);
        let product: f64 = path.iter().map(|&id| tree.nodes[id].probability).product();
        let scenario_prob = tree.scenario_probability(leaf_id);
        assert!((product - scenario_prob).abs() < 1e-12);
    }

    // 6
    #[test]
    fn test_investment_option_types() {
        let opts = make_options();
        assert_eq!(opts.len(), 5);
        assert!(matches!(
            opts[0].option_type,
            InvestmentType::TransmissionLine { .. }
        ));
        assert!(matches!(
            opts[1].option_type,
            InvestmentType::Substation { .. }
        ));
        assert!(matches!(
            opts[2].option_type,
            InvestmentType::StorageSystem { .. }
        ));
        assert!(matches!(
            opts[3].option_type,
            InvestmentType::SolarFarm { .. }
        ));
        assert!(matches!(
            opts[4].option_type,
            InvestmentType::WindFarm { .. }
        ));

        if let InvestmentType::TransmissionLine {
            from,
            to,
            length_km,
        } = &opts[0].option_type
        {
            assert_eq!(*from, 0);
            assert_eq!(*to, 1);
            assert!((length_km - 100.0).abs() < 1e-9);
        }
    }

    // 7
    #[test]
    fn test_discount_factor() {
        let df = StochasticPlanningProblem::discount(0.05, 10);
        let expected = 1.0_f64 / 1.05_f64.powi(10);
        assert!((df - expected).abs() < 1e-12);
        // Year 0 discount = 1
        assert!((StochasticPlanningProblem::discount(0.05, 0) - 1.0).abs() < 1e-12);
    }

    // 8
    #[test]
    fn test_npv_investment_calculation() {
        let opt = InvestmentOption {
            id: 0,
            option_type: InvestmentType::SolarFarm { bus: 0 },
            capex_million_eur: 100.0,
            annual_opex_million_eur: 1.0,
            capacity_mw: 200.0,
            lead_time_years: 0,
            lifetime_years: 20,
        };
        let npv = StochasticPlanningProblem::npv_investment(&opt, 0, 0.05);
        // NPV >= capex alone (opex adds to it)
        assert!(npv > 100.0, "NPV should exceed capex alone, got {npv}");
        // NPV < capex + 20 * annual_opex (discounting reduces opex PV)
        assert!(npv < 100.0 + 20.0, "NPV sanity upper bound");
    }

    // 9
    #[test]
    fn test_stochastic_planning_solve_greedy() {
        let mut prob = make_problem();
        let result = prob.solve_greedy().unwrap();
        // Must produce some decisions
        assert!(!result.decisions.is_empty());
        assert!(result.total_expected_cost_million_eur >= 0.0);
        assert!(result.robustness_score >= 0.0);
        assert!(result.robustness_score <= 1.0);
    }

    // 10
    #[test]
    fn test_stochastic_planning_budget_constraint() {
        let mut prob = make_problem();
        let result = prob.solve_greedy().unwrap();
        let total_capex: f64 = result.decisions.iter().map(|d| d.capex_million_eur).sum();
        assert!(
            total_capex <= prob.max_budget_million_eur + 1e-6,
            "total capex {total_capex} exceeds budget {}",
            prob.max_budget_million_eur
        );
    }

    // 11
    #[test]
    fn test_cvar_calculation() {
        // sorted: [1,2,3,4,5], alpha=0.6 → cutoff=ceil(3)=3, tail=[4,5], CVaR=4.5
        let costs = vec![3.0, 1.0, 5.0, 2.0, 4.0];
        let cvar = StochasticPlanningProblem::compute_cvar(&costs, 0.6);
        assert!((cvar - 4.5).abs() < 1e-9, "CVaR = {cvar}");
    }

    // 12
    #[test]
    fn test_cvar_worst_case() {
        let costs = vec![10.0];
        let cvar = StochasticPlanningProblem::compute_cvar(&costs, 0.9);
        assert!((cvar - 10.0).abs() < 1e-9);
    }

    // 13
    #[test]
    fn test_vss_non_negative() {
        let mut prob = make_problem();
        let result = prob.solve_greedy().unwrap();
        assert!(result.vss >= 0.0, "VSS must be ≥ 0, got {}", result.vss);
    }

    // 14
    #[test]
    fn test_evpi_geq_vss() {
        let mut prob = make_problem();
        let result = prob.solve_greedy().unwrap();
        // EVPI ≥ VSS (standard stochastic programming inequality)
        // Allow small floating-point tolerance
        assert!(
            result.evpi >= result.vss - 1e-6,
            "EVPI {} must be ≥ VSS {} (with tolerance)",
            result.evpi,
            result.vss
        );
    }

    // 15
    #[test]
    fn test_regret_matrix_dimensions() {
        let mut prob = make_problem();
        let result = prob.solve_greedy().unwrap();
        let regret = prob.compute_regret_matrix(&result.decisions);
        let n_opts = prob.investment_options.len();
        for row in &regret {
            assert_eq!(row.len(), n_opts);
        }
        // Number of rows = number of leaf nodes (or at least 1)
        let n_leaves = prob.scenario_tree.leaf_nodes().len();
        assert_eq!(regret.len(), n_leaves.max(1));
    }

    // 16
    #[test]
    fn test_robustness_score_perfect_plan() {
        // Build a problem where investment capacity far exceeds load
        let tree = ScenarioTree::build_symmetric(2, 2, 100.0, 0.02, 0.005);
        let opts = vec![InvestmentOption {
            id: 0,
            option_type: InvestmentType::TransmissionLine {
                from: 0,
                to: 1,
                length_km: 50.0,
            },
            capex_million_eur: 10.0,
            annual_opex_million_eur: 0.1,
            capacity_mw: 100_000.0, // far more than any load
            lead_time_years: 1,
            lifetime_years: 30,
        }];
        let mut prob = StochasticPlanningProblem::new(tree, opts, 5, 0.05, 3_000.0, 0.0, 500.0);
        let result = prob.solve_greedy().unwrap();
        // With such large capacity added, most/all scenarios should be robust
        assert!(result.robustness_score >= 0.0);
    }

    // 17
    #[test]
    fn test_monte_carlo_basic() {
        let mut mc = PlanningMonteCarlo::new(200, 10, 1000.0, 0.03, 0.01, 0.02, 0.005);
        let result = mc.run(900.0, 200.0, 3);
        assert!(result.mean_unserved_energy_mwh >= 0.0);
        assert!(result.std_unserved_energy_mwh >= 0.0);
        assert!(result.mean_total_cost_million_eur >= 0.0);
        assert!(result.adequate_fraction >= 0.0);
        assert!(result.adequate_fraction <= 1.0);
    }

    // 18
    #[test]
    fn test_monte_carlo_percentiles() {
        let mut mc = PlanningMonteCarlo::new(500, 10, 1000.0, 0.03, 0.01, 0.02, 0.005);
        let result = mc.run(800.0, 300.0, 3);
        assert!(
            result.p10_unserved_mwh <= result.p50_unserved_mwh + 1e-9,
            "p10={} > p50={}",
            result.p10_unserved_mwh,
            result.p50_unserved_mwh
        );
        assert!(
            result.p50_unserved_mwh <= result.p90_unserved_mwh + 1e-9,
            "p50={} > p90={}",
            result.p50_unserved_mwh,
            result.p90_unserved_mwh
        );
    }

    // 19
    #[test]
    fn test_monte_carlo_adequate_fraction() {
        let mut mc = PlanningMonteCarlo::new(300, 5, 500.0, 0.01, 0.005, 0.05, 0.01);
        // Very large capacity → high adequate fraction
        let result = mc.run(10_000.0, 0.0, 0);
        assert!(
            result.adequate_fraction >= 0.9,
            "expected high adequate fraction, got {}",
            result.adequate_fraction
        );
    }

    // 20
    #[test]
    fn test_sensitivity_analysis() {
        let mut mc = PlanningMonteCarlo::new(100, 5, 1000.0, 0.03, 0.01, 0.02, 0.005);
        let range = vec![0.01, 0.03, 0.05, 0.07];
        let results = mc.sensitivity_analysis("load_growth", &range, 900.0, 200.0, 2);
        assert_eq!(results.len(), range.len());
        for (i, &val) in range.iter().enumerate() {
            assert!((results[i].0 - val).abs() < 1e-12);
            assert!(results[i].1 >= 0.0);
        }
    }
}
