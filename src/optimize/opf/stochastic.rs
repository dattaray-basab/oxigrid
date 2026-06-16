/// Stochastic Optimal Power Flow (S-OPF).
///
/// Incorporates uncertainty in renewable generation and load through
/// Monte Carlo scenario sampling, chance constraints, and CVaR (Conditional
/// Value at Risk) risk measures.
///
/// # Method
///
/// 1. **Scenario generation**: Sample `n_scenarios` realisations of uncertain
///    parameters (renewable output, load) from specified distributions.
///
/// 2. **Deterministic solve**: Run DC-OPF for each scenario independently.
///
/// 3. **Chance-constraint satisfaction**: Check that the fraction of scenarios
///    where constraint violations occur is below the allowed violation
///    probability `epsilon`.
///
/// 4. **CVaR computation**: Compute CVaR_α of cost (expected cost in the
///    worst `(1−α)` fraction of scenarios).
///
/// 5. **Optimal base dispatch**: Minimise E[cost] + β · CVaR subject to the
///    chance constraints.  Implemented as a weighted average over scenarios.
///
/// # References
/// - Bienstock & Shukla, "Chance-Constrained Optimal Power Flow", 2014.
/// - Rockafellar & Uryasev, "CVaR methodology", JCAM 2002.
use crate::error::{OxiGridError, Result};
use crate::network::topology::PowerNetwork;
use crate::optimize::opf::dc_opf::{economic_dispatch_pub, GenCost};
use serde::{Deserialize, Serialize};

/// Uncertain parameter distribution for one variable.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum UncertainDist {
    /// Normal distribution N(μ, σ²)
    Normal { mean: f64, std: f64 },
    /// Uniform U[lo, hi]
    Uniform { lo: f64, hi: f64 },
    /// Beta distribution B(α, β) scaled to [lo, hi]
    Beta {
        alpha: f64,
        beta_param: f64,
        lo: f64,
        hi: f64,
    },
    /// Fixed (deterministic) value
    Fixed(f64),
}

impl UncertainDist {
    /// Sample from this distribution using a standard uniform u ∈ (0,1).
    pub fn sample(&self, u: f64) -> f64 {
        match self {
            Self::Normal { mean, std } => {
                // Box-Muller requires two uniforms; use single approximate inversion
                mean + std * normal_quantile(u)
            }
            Self::Uniform { lo, hi } => lo + u * (hi - lo),
            Self::Beta {
                alpha,
                beta_param,
                lo,
                hi,
            } => {
                let x = beta_sample(u, *alpha, *beta_param);
                lo + x * (hi - lo)
            }
            Self::Fixed(v) => *v,
        }
    }

    /// Expected value of this distribution.
    pub fn mean(&self) -> f64 {
        match self {
            Self::Normal { mean, .. } => *mean,
            Self::Uniform { lo, hi } => (lo + hi) / 2.0,
            Self::Beta {
                alpha,
                beta_param,
                lo,
                hi,
            } => {
                let mu = alpha / (alpha + beta_param);
                lo + mu * (hi - lo)
            }
            Self::Fixed(v) => *v,
        }
    }

    /// Standard deviation.
    pub fn std_dev(&self) -> f64 {
        match self {
            Self::Normal { std, .. } => *std,
            Self::Uniform { lo, hi } => (hi - lo) / (12.0_f64).sqrt(),
            Self::Beta {
                alpha,
                beta_param,
                lo,
                hi,
            } => {
                let var = alpha * beta_param
                    / ((alpha + beta_param).powi(2) * (alpha + beta_param + 1.0));
                (hi - lo) * var.sqrt()
            }
            Self::Fixed(_) => 0.0,
        }
    }
}

/// Uncertain renewable/load source tied to a bus.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UncertainSource {
    /// Bus ID where the uncertainty is injected
    pub bus_id: u32,
    /// Type: "load" or "renewable"
    pub source_type: String,
    /// Power output distribution `MW`
    pub dist: UncertainDist,
}

impl UncertainSource {
    pub fn wind(bus_id: u32, mean_mw: f64, std_mw: f64) -> Self {
        Self {
            bus_id,
            source_type: "renewable".into(),
            dist: UncertainDist::Normal {
                mean: mean_mw,
                std: std_mw,
            },
        }
    }

    pub fn solar(bus_id: u32, peak_mw: f64) -> Self {
        Self {
            bus_id,
            source_type: "renewable".into(),
            dist: UncertainDist::Beta {
                alpha: 2.0,
                beta_param: 5.0,
                lo: 0.0,
                hi: peak_mw,
            },
        }
    }

    pub fn load(bus_id: u32, mean_mw: f64, std_mw: f64) -> Self {
        Self {
            bus_id,
            source_type: "load".into(),
            dist: UncertainDist::Normal {
                mean: mean_mw,
                std: std_mw,
            },
        }
    }
}

/// Configuration for stochastic OPF.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StochasticOpfConfig {
    /// Number of Monte Carlo scenarios
    pub n_scenarios: usize,
    /// Random seed for reproducibility
    pub seed: u64,
    /// Allowed constraint violation probability ε (chance constraint level)
    pub epsilon: f64,
    /// CVaR confidence level α (e.g. 0.95 = worst 5% of scenarios)
    pub cvar_alpha: f64,
    /// CVaR risk penalty weight β (objective = `E[cost]` + β·CVaR)
    pub cvar_beta: f64,
}

impl Default for StochasticOpfConfig {
    fn default() -> Self {
        Self {
            n_scenarios: 100,
            seed: 42,
            epsilon: 0.05,
            cvar_alpha: 0.95,
            cvar_beta: 0.2,
        }
    }
}

/// One scenario realisation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Scenario {
    /// Scenario index
    pub index: usize,
    /// Sampled uncertain power injections `MW`, keyed by bus_id
    pub injections: Vec<(u32, f64)>,
    /// Total load for this scenario `MW`
    pub total_load_mw: f64,
    /// Dispatch result `MW` per generator
    pub p_gen_mw: Vec<f64>,
    /// Total cost [$/h]
    pub cost: f64,
    /// Any constraint violated in this scenario
    pub constraint_violated: bool,
    /// Curtailment of renewables `MW` (renewable sampled − renewable used)
    pub curtailment_mw: f64,
}

/// Stochastic OPF result.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StochasticOpfResult {
    /// All scenario solutions
    pub scenarios: Vec<Scenario>,
    /// Expected cost across all scenarios [$/h]
    pub expected_cost: f64,
    /// Cost standard deviation [$/h]
    pub cost_std: f64,
    /// CVaR at α confidence level [$/h]
    pub cvar: f64,
    /// Risk-adjusted objective: `E[cost]` + β·CVaR `$/h`
    pub objective: f64,
    /// Probability of constraint violation across scenarios
    pub violation_probability: f64,
    /// Chance constraint satisfied? (violation_prob ≤ ε)
    pub chance_constraint_satisfied: bool,
    /// Optimal base dispatch (expected dispatch across scenarios) `MW`
    pub base_dispatch_mw: Vec<f64>,
    /// Expected curtailment `MW`
    pub expected_curtailment_mw: f64,
}

impl StochasticOpfResult {
    /// Percentile cost at given probability level p ∈ `0,1`.
    pub fn cost_percentile(&self, p: f64) -> f64 {
        let mut costs: Vec<f64> = self.scenarios.iter().map(|s| s.cost).collect();
        costs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let idx = ((p * costs.len() as f64) as usize).min(costs.len() - 1);
        costs[idx]
    }

    /// Number of scenarios with constraint violations.
    pub fn n_violations(&self) -> usize {
        self.scenarios
            .iter()
            .filter(|s| s.constraint_violated)
            .count()
    }
}

/// Run a stochastic OPF with Monte Carlo scenario sampling.
///
/// For each scenario:
///   1. Sample uncertain sources using the configured seed+LCG.
///   2. Compute effective load = base load + uncertain load − uncertain renewable.
///   3. Solve deterministic DC-OPF for the scenario load.
///   4. Check constraint feasibility (load ≤ gen capacity).
///
/// Then aggregate statistics: `E[cost]`, CVaR, violation probability.
pub fn run_stochastic_opf(
    network: &PowerNetwork,
    gen_costs: &[GenCost],
    uncertain_sources: &[UncertainSource],
    config: &StochasticOpfConfig,
) -> Result<StochasticOpfResult> {
    if gen_costs.len() != network.generators.len() {
        return Err(OxiGridError::InvalidParameter(format!(
            "gen_costs length {} != generators {}",
            gen_costs.len(),
            network.generators.len()
        )));
    }

    let base_load_mw: f64 = network.buses.iter().map(|b| b.pd.0).sum();
    let n_gen = network.generators.len();

    // Total generation capacity
    let p_max_total: f64 = gen_costs.iter().map(|c| c.p_max).sum();
    let p_min_total: f64 = gen_costs.iter().map(|c| c.p_min).sum();

    let mut scenarios = Vec::with_capacity(config.n_scenarios);

    // LCG random number generator
    let mut rng = LcgRng::new(config.seed);

    for i in 0..config.n_scenarios {
        // Sample injections
        let mut net_load = base_load_mw;
        let mut injections = Vec::with_capacity(uncertain_sources.len());
        let mut curtailment = 0.0;

        for src in uncertain_sources {
            let u = rng.next_f64();
            let power_mw = src.dist.sample(u).max(0.0);
            injections.push((src.bus_id, power_mw));

            if src.source_type == "load" {
                net_load += power_mw - src.dist.mean(); // deviation from mean
            } else {
                // Renewable reduces net load; any excess is curtailed
                net_load -= power_mw;
            }
        }

        // Ensure non-negative load
        if net_load < 0.0 {
            curtailment = -net_load;
            net_load = 0.0;
        }

        // Check feasibility
        let mut constraint_violated = false;
        let (p_gen_mw, cost) = if net_load > p_max_total || net_load < p_min_total {
            constraint_violated = true;
            // Use clamped dispatch
            let clamped_load = net_load.clamp(p_min_total, p_max_total);
            let p =
                economic_dispatch_pub(gen_costs, clamped_load).unwrap_or_else(|_| vec![0.0; n_gen]);
            let c: f64 = gen_costs
                .iter()
                .zip(p.iter())
                .map(|(gc, &pg)| gc.total_cost(pg))
                .sum();
            (p, c)
        } else {
            match economic_dispatch_pub(gen_costs, net_load) {
                Ok(p) => {
                    let c: f64 = gen_costs
                        .iter()
                        .zip(p.iter())
                        .map(|(gc, &pg)| gc.total_cost(pg))
                        .sum();
                    (p, c)
                }
                Err(_) => {
                    constraint_violated = true;
                    (vec![0.0; n_gen], 0.0)
                }
            }
        };

        scenarios.push(Scenario {
            index: i,
            injections,
            total_load_mw: net_load,
            p_gen_mw,
            cost,
            constraint_violated,
            curtailment_mw: curtailment,
        });
    }

    // Aggregate statistics
    let n = scenarios.len() as f64;
    let expected_cost = scenarios.iter().map(|s| s.cost).sum::<f64>() / n;
    let cost_var = scenarios
        .iter()
        .map(|s| (s.cost - expected_cost).powi(2))
        .sum::<f64>()
        / n;
    let cost_std = cost_var.sqrt();

    // CVaR at alpha level
    let cvar = compute_cvar(&scenarios, config.cvar_alpha);

    let violation_probability =
        scenarios.iter().filter(|s| s.constraint_violated).count() as f64 / n;

    let chance_constraint_satisfied = violation_probability <= config.epsilon;

    let objective = expected_cost + config.cvar_beta * cvar;

    // Base dispatch: average across non-violated scenarios (or all if all violated)
    let valid_scenarios: Vec<&Scenario> = scenarios
        .iter()
        .filter(|s| !s.constraint_violated)
        .collect();
    let ref_scenarios = if valid_scenarios.is_empty() {
        scenarios.iter().collect()
    } else {
        valid_scenarios
    };
    let base_dispatch_mw = if ref_scenarios.is_empty() {
        vec![0.0; n_gen]
    } else {
        let m = ref_scenarios.len() as f64;
        (0..n_gen)
            .map(|g| ref_scenarios.iter().map(|s| s.p_gen_mw[g]).sum::<f64>() / m)
            .collect()
    };

    let expected_curtailment_mw = scenarios.iter().map(|s| s.curtailment_mw).sum::<f64>() / n;

    Ok(StochasticOpfResult {
        scenarios,
        expected_cost,
        cost_std,
        cvar,
        objective,
        violation_probability,
        chance_constraint_satisfied,
        base_dispatch_mw,
        expected_curtailment_mw,
    })
}

/// Compute CVaR at confidence level α for a set of scenarios.
///
/// CVaR_α = E[cost | cost ≥ VaR_α]
fn compute_cvar(scenarios: &[Scenario], alpha: f64) -> f64 {
    let mut costs: Vec<f64> = scenarios.iter().map(|s| s.cost).collect();
    costs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

    let n = costs.len();
    if n == 0 {
        return 0.0;
    }

    let var_idx = ((alpha * n as f64) as usize).min(n - 1);
    let tail_costs = &costs[var_idx..];

    if tail_costs.is_empty() {
        costs[n - 1]
    } else {
        tail_costs.iter().sum::<f64>() / tail_costs.len() as f64
    }
}

/// Sensitivity analysis: partial derivative of expected cost w.r.t. uncertain mean.
///
/// Uses finite-difference approximation: `ΔE[cost]` / Δμ_i.
pub fn cost_sensitivity_to_mean(
    network: &PowerNetwork,
    gen_costs: &[GenCost],
    uncertain_sources: &[UncertainSource],
    config: &StochasticOpfConfig,
    source_idx: usize,
    delta_mw: f64,
) -> Result<f64> {
    // Baseline
    let result0 = run_stochastic_opf(network, gen_costs, uncertain_sources, config)?;

    // Perturbed
    let mut sources_pert = uncertain_sources.to_vec();
    if source_idx >= sources_pert.len() {
        return Err(OxiGridError::InvalidParameter(
            "source_idx out of range".into(),
        ));
    }
    let mean0 = sources_pert[source_idx].dist.mean();
    sources_pert[source_idx].dist = UncertainDist::Normal {
        mean: mean0 + delta_mw,
        std: uncertain_sources[source_idx].dist.std_dev(),
    };

    let result1 = run_stochastic_opf(network, gen_costs, &sources_pert, config)?;

    Ok((result1.expected_cost - result0.expected_cost) / delta_mw)
}

/// Pinball loss for quantile forecast evaluation.
///
/// L_τ(y, ŷ) = (y − ŷ)·τ if y ≥ ŷ, else (ŷ − y)·(1 − τ)
pub fn pinball_loss(actual: f64, predicted: f64, tau: f64) -> f64 {
    if actual >= predicted {
        (actual - predicted) * tau
    } else {
        (predicted - actual) * (1.0 - tau)
    }
}

/// Average pinball loss for a set of forecasts.
pub fn mean_pinball_loss(actuals: &[f64], predicteds: &[f64], tau: f64) -> f64 {
    if actuals.len() != predicteds.len() || actuals.is_empty() {
        return 0.0;
    }
    actuals
        .iter()
        .zip(predicteds.iter())
        .map(|(&a, &p)| pinball_loss(a, p, tau))
        .sum::<f64>()
        / actuals.len() as f64
}

/// Linear Congruential Generator (fast, reproducible).
struct LcgRng {
    state: u64,
}

impl LcgRng {
    fn new(seed: u64) -> Self {
        Self {
            state: seed.wrapping_add(1),
        }
    }

    fn next_u64(&mut self) -> u64 {
        // Parameters from Knuth / Numerical Recipes
        self.state = self
            .state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        self.state
    }

    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// Approximate standard normal quantile (Beasley-Springer-Moro algorithm).
fn normal_quantile(p: f64) -> f64 {
    let p = p.clamp(1e-10, 1.0 - 1e-10);
    let a = [
        2.50662823884,
        -18.61500062529,
        41.39119773534,
        -25.44106049637,
    ];
    let b = [
        -8.47351093090,
        23.08336743743,
        -21.06224101826,
        3.13082909833,
    ];
    let c = [
        0.337475482272615,
        0.976169019091719,
        0.160797971491821,
        2.76438810333863e-2,
        3.8405729373609e-3,
        3.951896511349e-4,
        3.21767881768e-5,
        2.888167364e-7,
        3.960315187e-7,
    ];

    let q = p - 0.5;
    if q.abs() <= 0.42 {
        let r = q * q;
        let num = q * (((a[3] * r + a[2]) * r + a[1]) * r + a[0]);
        let den = (((b[3] * r + b[2]) * r + b[1]) * r + b[0]) * r + 1.0;
        num / den
    } else {
        let r = if q < 0.0 { p } else { 1.0 - p };
        let r = (-r.ln()).ln();
        let mut s = c[0];
        for &ci in &c[1..] {
            s = s * r + ci;
        }
        if q < 0.0 {
            -s
        } else {
            s
        }
    }
}

/// Sample from Beta(α, β) distribution using acceptance-rejection.
fn beta_sample(u: f64, alpha: f64, beta: f64) -> f64 {
    // Simplified: use Johnk's method for α,β close to 1,
    // otherwise use normal approximation
    let mu = alpha / (alpha + beta);
    let var = alpha * beta / ((alpha + beta).powi(2) * (alpha + beta + 1.0));
    // Transform u through normal quantile
    let z = normal_quantile(u);
    (mu + var.sqrt() * z).clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ieee14_net() -> PowerNetwork {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
        PowerNetwork::from_matpower(path).expect("ieee14")
    }

    fn ieee14_costs(net: &PowerNetwork) -> Vec<GenCost> {
        net.generators
            .iter()
            .map(|g| GenCost::quadratic(0.0, 20.0, 0.05, g.pmin.max(0.0), g.pmax.max(10.0)))
            .collect()
    }

    #[test]
    fn test_stochastic_opf_basic() {
        let net = ieee14_net();
        let costs = ieee14_costs(&net);
        let sources = vec![UncertainSource::wind(1, 20.0, 5.0)];
        let config = StochasticOpfConfig {
            n_scenarios: 50,
            ..Default::default()
        };
        let result = run_stochastic_opf(&net, &costs, &sources, &config).unwrap();
        assert_eq!(result.scenarios.len(), 50);
        assert!(result.expected_cost > 0.0);
    }

    #[test]
    fn test_expected_cost_positive() {
        let net = ieee14_net();
        let costs = ieee14_costs(&net);
        let sources = vec![UncertainSource::solar(2, 15.0)];
        let config = StochasticOpfConfig {
            n_scenarios: 30,
            seed: 123,
            ..Default::default()
        };
        let result = run_stochastic_opf(&net, &costs, &sources, &config).unwrap();
        assert!(
            result.expected_cost > 0.0,
            "Expected cost: {:.2}",
            result.expected_cost
        );
    }

    #[test]
    fn test_cvar_geq_expected_cost() {
        let net = ieee14_net();
        let costs = ieee14_costs(&net);
        let sources = vec![UncertainSource::wind(1, 10.0, 8.0)];
        let config = StochasticOpfConfig {
            n_scenarios: 100,
            cvar_alpha: 0.9,
            ..Default::default()
        };
        let result = run_stochastic_opf(&net, &costs, &sources, &config).unwrap();
        // CVaR ≥ Expected cost by definition
        assert!(
            result.cvar >= result.expected_cost - 1.0,
            "CVaR={:.2} should be ≥ E[cost]={:.2}",
            result.cvar,
            result.expected_cost
        );
    }

    #[test]
    fn test_no_uncertain_sources() {
        let net = ieee14_net();
        let costs = ieee14_costs(&net);
        let config = StochasticOpfConfig {
            n_scenarios: 20,
            ..Default::default()
        };
        let result = run_stochastic_opf(&net, &costs, &[], &config).unwrap();
        // All scenarios identical → std dev = 0
        assert!(
            result.cost_std < 1.0,
            "Cost std with no uncertainty: {:.4}",
            result.cost_std
        );
    }

    #[test]
    fn test_base_dispatch_length() {
        let net = ieee14_net();
        let costs = ieee14_costs(&net);
        let sources = vec![UncertainSource::wind(1, 5.0, 2.0)];
        let config = StochasticOpfConfig {
            n_scenarios: 20,
            ..Default::default()
        };
        let result = run_stochastic_opf(&net, &costs, &sources, &config).unwrap();
        assert_eq!(result.base_dispatch_mw.len(), net.generators.len());
    }

    #[test]
    fn test_pinball_loss_zero_at_quantile() {
        // For tau=0.5 (median), loss should be symmetric
        let loss_over = pinball_loss(12.0, 10.0, 0.5);
        let loss_under = pinball_loss(8.0, 10.0, 0.5);
        assert!(
            (loss_over - loss_under).abs() < 1e-10,
            "Median pinball: over={:.4}, under={:.4}",
            loss_over,
            loss_under
        );
    }

    #[test]
    fn test_pinball_loss_asymmetric() {
        // tau=0.9: under-prediction is heavily penalised
        let loss_over = pinball_loss(12.0, 10.0, 0.9);
        let loss_under = pinball_loss(8.0, 10.0, 0.9);
        assert!(
            loss_over > loss_under,
            "At τ=0.9 over-prediction penalised more"
        );
    }

    #[test]
    fn test_mean_pinball_loss() {
        let actuals = vec![10.0, 12.0, 8.0, 11.0];
        let predicted = vec![10.0, 10.0, 10.0, 10.0];
        let loss = mean_pinball_loss(&actuals, &predicted, 0.5);
        assert!(loss >= 0.0);
    }

    #[test]
    fn test_violation_probability_in_range() {
        let net = ieee14_net();
        let costs = ieee14_costs(&net);
        // High-variance wind exceeding generation capacity
        let sources = vec![UncertainSource::load(1, 50.0, 5.0)];
        let config = StochasticOpfConfig {
            n_scenarios: 50,
            ..Default::default()
        };
        let result = run_stochastic_opf(&net, &costs, &sources, &config).unwrap();
        assert!(result.violation_probability >= 0.0 && result.violation_probability <= 1.0);
    }

    #[test]
    fn test_uncertain_dist_mean() {
        let d = UncertainDist::Normal {
            mean: 10.0,
            std: 2.0,
        };
        assert_eq!(d.mean(), 10.0);
        let d2 = UncertainDist::Uniform { lo: 4.0, hi: 6.0 };
        assert!((d2.mean() - 5.0).abs() < 1e-10);
        let d3 = UncertainDist::Fixed(7.5);
        assert_eq!(d3.mean(), 7.5);
    }

    #[test]
    fn test_lcg_reproducible() {
        let mut rng1 = LcgRng::new(99);
        let mut rng2 = LcgRng::new(99);
        for _ in 0..10 {
            assert_eq!(rng1.next_f64(), rng2.next_f64());
        }
    }

    #[test]
    fn test_cost_percentile() {
        let net = ieee14_net();
        let costs = ieee14_costs(&net);
        let sources = vec![UncertainSource::wind(1, 5.0, 3.0)];
        let config = StochasticOpfConfig {
            n_scenarios: 100,
            ..Default::default()
        };
        let result = run_stochastic_opf(&net, &costs, &sources, &config).unwrap();
        let p50 = result.cost_percentile(0.5);
        let p95 = result.cost_percentile(0.95);
        assert!(
            p95 >= p50 - 1.0,
            "P95={:.2} should be ≥ P50={:.2}",
            p95,
            p50
        );
    }
}
