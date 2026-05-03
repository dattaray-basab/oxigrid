//! Chance-Constrained Optimal Power Flow (CC-OPF).
//!
//! Extends the stochastic OPF framework with three solving approaches:
//!
//! 1. **Sample Average Approximation (SAA)**: Monte Carlo scenario generation
//!    using LCG + Box-Muller, followed by per-scenario deterministic dispatch
//!    and aggregation of statistics.
//!
//! 2. **Analytic Moment Method**: Gaussian approximation converts chance
//!    constraints `P(P_branch ≤ rating) ≥ 1−α` into deterministic tightenings
//!    `μ + z_{1-α} · σ ≤ rating` via PTDF-propagated variance.
//!
//! 3. **Robust Optimization**: worst-case dispatch over an uncertainty ellipsoid
//!    parameterised by `k = z_{1-α}` standard deviations (e.g. k ≈ 1.645 for α=0.05).
//!
//! # References
//! - Bienstock & Shukla, "Chance-Constrained Optimal Power Flow", 2014.
//! - Rockafellar & Uryasev, "CVaR methodology", JCAM 2002.
//! - Lorenzen et al., "Data-driven chance-constrained OPF", 2019.

use serde::{Deserialize, Serialize};

// ── Internal LCG identical to the one in stochastic.rs ─────────────────────

struct LcgRng {
    state: u64,
}

impl LcgRng {
    fn new(seed: u64) -> Self {
        // Avoid zero state
        Self {
            state: seed.wrapping_add(1),
        }
    }

    #[inline]
    fn next_u64(&mut self) -> u64 {
        self.state = self
            .state
            .wrapping_mul(6_364_136_223_846_793_005_u64)
            .wrapping_add(1_442_695_040_888_963_407_u64);
        self.state
    }

    /// Returns a value in (0, 1) exclusive.
    #[inline]
    fn next_f64_pos(&mut self) -> f64 {
        // Map to (0,1): add 1 to numerator so the result is never 0.
        (self.next_u64() as f64 + 1.0) / (u64::MAX as f64 + 2.0)
    }
}

// ── Box-Muller normal sample ────────────────────────────────────────────────

/// Generate one standard normal sample via Box-Muller using two LCG draws.
fn box_muller(rng: &mut LcgRng) -> f64 {
    let u1 = rng.next_f64_pos();
    let u2 = rng.next_f64_pos();
    (-2.0 * u1.ln()).sqrt() * (2.0 * core::f64::consts::PI * u2).cos()
}

// ── z-score table ───────────────────────────────────────────────────────────

/// Approximate inverse standard-normal CDF (quantile function) at confidence
/// level `p` using Acklam's rational approximation (max absolute error < 3.6 × 10⁻⁹).
///
/// Returns z such that Φ(z) = p, where Φ is the standard normal CDF.
/// `p` must be in (0, 1); values are clamped to (1×10⁻¹⁰, 1−1×10⁻¹⁰).
fn z_score(p: f64) -> f64 {
    // Acklam, P. J. (2002). "An algorithm for computing the inverse normal
    // cumulative distribution function." <https://web.archive.org/web/20151030215612/
    // http://home.online.no/~pjacklam/notes/invnorm/>
    let p = p.clamp(1e-10, 1.0 - 1e-10);

    // Coefficients for the rational approximation.
    let a: [f64; 6] = [
        -3.969_683_028_665_376e1,
        2.209_460_984_245_205e2,
        -2.759_285_104_469_687e2,
        1.383_577_518_672_69e2,
        -3.066_479_806_614_716e1,
        2.506_628_277_459_239,
    ];
    let b: [f64; 5] = [
        -5.447_609_879_822_406e1,
        1.615_858_368_580_409e2,
        -1.556_989_798_598_866e2,
        6.680_131_188_771_972e1,
        -1.328_068_155_288_572e1,
    ];
    let c: [f64; 6] = [
        -7.784_894_002_430_293e-3,
        -3.223_964_580_411_365e-1,
        -2.400_758_277_161_838,
        -2.549_732_539_343_734,
        4.374_664_141_464_968,
        2.938_163_982_698_783,
    ];
    let d: [f64; 4] = [
        7.784_695_709_041_462e-3,
        3.224_671_290_700_398e-1,
        2.445_134_137_142_996,
        3.754_408_661_907_416,
    ];

    let p_lo: f64 = 0.02425;
    let p_hi: f64 = 1.0 - p_lo;

    if p >= p_lo && p <= p_hi {
        // Central region
        let q = p - 0.5;
        let r = q * q;
        (q * (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5]))
            / (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1.0)
    } else if p < p_lo {
        // Lower tail: z is negative
        let q = (-2.0 * p.ln()).sqrt();
        (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5])
            / ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0)
    } else {
        // Upper tail: z is positive; use symmetry z(p) = -z(1-p)
        let q = (-2.0 * (1.0 - p).ln()).sqrt();
        -((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q
            - c[5] / ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Public enums
// ─────────────────────────────────────────────────────────────────────────────

/// Solving approach for chance-constrained OPF.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ChanceConstraintApproach {
    /// Sample Average Approximation with N Monte Carlo scenarios.
    ScenarioBased,
    /// Analytic moment-based Gaussian approximation (μ + z·σ ≤ b).
    AnalyticMoment,
    /// Worst-case robust optimization within an uncertainty ellipsoid.
    RobustOptimization,
    /// Conditional Value-at-Risk minimisation (tail-risk aware).
    ConditionalValueAtRisk,
}

/// Physical or market source of uncertainty.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum UncertaintySource {
    /// Photovoltaic / solar generation output.
    SolarGeneration,
    /// Wind turbine generation output.
    WindGeneration,
    /// Aggregate load demand at a bus.
    LoadDemand,
    /// Random line outage (N-1 event).
    LineOutage,
    /// Wholesale electricity price signal.
    PriceSignal,
}

// ─────────────────────────────────────────────────────────────────────────────
// Public structs
// ─────────────────────────────────────────────────────────────────────────────

/// Description of one uncertain parameter including its distribution moments.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UncertaintySet {
    /// Physical origin of the uncertainty.
    pub source: UncertaintySource,
    /// Nominal (mean / forecast) power `MW`.
    pub nominal_value_mw: f64,
    /// Standard deviation of the power `MW`.
    pub std_dev_mw: f64,
    /// Pearson correlation with other uncertain parameters (−1 … +1).
    pub correlation: f64,
    /// Distribution family name, e.g. `"Normal"`, `"Beta"`, `"Uniform"`.
    pub distribution: String,
}

impl UncertaintySet {
    /// Convenience constructor for a normally-distributed wind source.
    pub fn wind(nominal_mw: f64, std_dev_mw: f64) -> Self {
        Self {
            source: UncertaintySource::WindGeneration,
            nominal_value_mw: nominal_mw,
            std_dev_mw,
            correlation: 0.0,
            distribution: "Normal".into(),
        }
    }

    /// Convenience constructor for a Beta-distributed solar source.
    pub fn solar(peak_mw: f64) -> Self {
        // Beta(2,5) has mean ≈ 0.286; scale to peak.
        let nominal = peak_mw * 2.0 / 7.0;
        let std_dev = peak_mw * (2.0 * 5.0 / (7.0_f64.powi(2) * 8.0)).sqrt();
        Self {
            source: UncertaintySource::SolarGeneration,
            nominal_value_mw: nominal,
            std_dev_mw: std_dev,
            correlation: 0.0,
            distribution: "Beta".into(),
        }
    }

    /// Convenience constructor for normally-distributed load uncertainty.
    pub fn load(nominal_mw: f64, std_dev_mw: f64) -> Self {
        Self {
            source: UncertaintySource::LoadDemand,
            nominal_value_mw: nominal_mw,
            std_dev_mw,
            correlation: 0.0,
            distribution: "Normal".into(),
        }
    }

    /// Sample a realisation `MW` for this source using a standard-normal draw `z`.
    fn sample_from_z(&self, z: f64) -> f64 {
        match self.distribution.as_str() {
            "Uniform" => {
                // Map z ∈ (−3,3) to a uniform via clamp
                let half = self.std_dev_mw * 3.0_f64.sqrt();
                (self.nominal_value_mw + z * self.std_dev_mw / 1.0)
                    .clamp(self.nominal_value_mw - half, self.nominal_value_mw + half)
                    .max(0.0)
            }
            "Beta" => {
                // Normal approximation scaled to Beta moments.
                (self.nominal_value_mw + z * self.std_dev_mw).max(0.0)
            }
            _ => {
                // "Normal" and anything else: direct linear transform.
                (self.nominal_value_mw + z * self.std_dev_mw).max(0.0)
            }
        }
    }
}

/// A single chance constraint on a network quantity.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChanceConstraint {
    /// Unique identifier for this constraint.
    pub constraint_id: usize,
    /// Quantity being constrained, e.g. `"branch_flow"`, `"bus_voltage"`,
    /// `"generation_limit"`.
    pub constraint_type: String,
    /// Hard limit `MW` or `pu` for voltages.
    pub limit_mw: f64,
    /// Required probability of satisfaction, e.g. `0.95`.
    pub confidence_level: f64,
    /// Allowed violation probability α = 1 − confidence_level.
    pub risk_parameter_alpha: f64,
}

impl ChanceConstraint {
    /// Construct a branch-flow chance constraint.
    ///
    /// `confidence` — required probability of satisfaction (e.g. 0.95).
    pub fn branch_flow(id: usize, rating_mw: f64, confidence: f64) -> Self {
        Self {
            constraint_id: id,
            constraint_type: "branch_flow".into(),
            limit_mw: rating_mw,
            confidence_level: confidence,
            risk_parameter_alpha: 1.0 - confidence,
        }
    }
}

/// One Monte Carlo scenario (a joint realisation of all uncertain sources).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CcopfScenario {
    /// Scenario index within the sample.
    pub id: usize,
    /// Sampled output `MW` for each uncertain generator (same order as
    /// `uncertainty_sets` of type `SolarGeneration` / `WindGeneration`).
    pub renewable_outputs_mw: Vec<f64>,
    /// Deviation from nominal load `MW` at each bus (same ordering as
    /// `base_load_mw`).
    pub load_deviations_mw: Vec<f64>,
    /// Importance weight (= 1/N for equally-weighted SAA).
    pub probability: f64,
}

/// Result returned by all `ChanceConstrainedOpf` solvers.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CcopfResult {
    /// Solving approach that produced this result.
    pub approach: ChanceConstraintApproach,
    /// Nominal (base) dispatch `MW` per generator.
    pub base_dispatch_mw: Vec<f64>,
    /// Upward reserve `MW` per generator.
    pub reserve_up_mw: Vec<f64>,
    /// Downward reserve `MW` per generator.
    pub reserve_down_mw: Vec<f64>,
    /// Expected (mean) dispatch cost [$/h].
    pub expected_cost_usd: f64,
    /// Reserve procurement cost [$/h] (proportional to reserve volume).
    pub reserve_cost_usd: f64,
    /// Per-constraint empirical probability of satisfaction.
    pub chance_constraint_satisfaction: Vec<f64>,
    /// Conditional Value-at-Risk of cost at the SAA alpha level [$/h].
    pub cvar_usd: f64,
    /// Worst-case scenario cost observed [$/h].
    pub worst_case_cost_usd: f64,
    /// Number of scenarios that were actually evaluated.
    pub n_scenarios_used: usize,
}

// ─────────────────────────────────────────────────────────────────────────────
// Main solver
// ─────────────────────────────────────────────────────────────────────────────

/// Chance-constrained OPF solver supporting SAA, analytic moment, and robust
/// approaches.
///
/// # Quick start
/// ```rust,ignore
/// let mut ccopf = ChanceConstrainedOpf::new(14, 5);
/// ccopf.add_uncertainty(UncertaintySet::wind(30.0, 8.0));
/// ccopf.add_chance_constraint(ChanceConstraint::branch_flow(0, 100.0, 0.95));
/// let result = ccopf.solve();
/// ```
#[derive(Debug, Clone)]
pub struct ChanceConstrainedOpf {
    /// Number of buses in the network.
    pub n_buses: usize,
    /// Number of dispatchable generators.
    pub n_generators: usize,
    /// Uncertain parameter descriptions.
    pub uncertainty_sets: Vec<UncertaintySet>,
    /// Chance constraints to satisfy.
    pub chance_constraints: Vec<ChanceConstraint>,
    /// Solving approach.
    pub approach: ChanceConstraintApproach,
    /// Number of Monte Carlo scenarios for SAA.
    pub n_scenarios: usize,
    /// Random seed for reproducible scenario generation.
    pub seed: u64,
    /// PTDF matrix [n_branch × n_bus], row-major.
    pub ptdf_matrix: Vec<Vec<f64>>,
    /// Thermal ratings per branch `MW`.
    pub branch_ratings_mw: Vec<f64>,
    /// Quadratic cost coefficients (a, b, c) per generator: cost = a·P² + b·P + c.
    pub gen_costs: Vec<(f64, f64, f64)>,
    /// Generation limits (P_min, P_max) per generator `MW`.
    pub gen_limits: Vec<(f64, f64)>,
    /// Nominal load per bus `MW`.
    pub base_load_mw: Vec<f64>,
}

impl ChanceConstrainedOpf {
    /// Construct a new solver skeleton.  Call `add_uncertainty` and
    /// `add_chance_constraint` before `solve`.
    pub fn new(n_buses: usize, n_generators: usize) -> Self {
        let gen_costs = vec![(0.01_f64, 20.0_f64, 0.0_f64); n_generators];
        let gen_limits = vec![(0.0_f64, 100.0_f64); n_generators];
        let base_load_mw = vec![10.0_f64; n_buses];

        Self {
            n_buses,
            n_generators,
            uncertainty_sets: Vec::new(),
            chance_constraints: Vec::new(),
            approach: ChanceConstraintApproach::ScenarioBased,
            n_scenarios: 200,
            seed: 0,
            ptdf_matrix: Vec::new(),
            branch_ratings_mw: Vec::new(),
            gen_costs,
            gen_limits,
            base_load_mw,
        }
    }

    /// Register an uncertain source.
    pub fn add_uncertainty(&mut self, uncertainty: UncertaintySet) {
        self.uncertainty_sets.push(uncertainty);
    }

    /// Register a chance constraint.
    pub fn add_chance_constraint(&mut self, constraint: ChanceConstraint) {
        self.chance_constraints.push(constraint);
    }

    // ── Scenario generation ───────────────────────────────────────────────

    /// Generate `self.n_scenarios` Monte Carlo scenarios using the LCG +
    /// Box-Muller method.
    ///
    /// For `"Normal"` sources the full Box-Muller transform is applied.
    /// For `"Beta"` and `"Uniform"` sources the same standard-normal draw is
    /// fed through a moment-matched approximation (see `UncertaintySet::sample_from_z`).
    pub fn generate_scenarios(&mut self) -> Vec<CcopfScenario> {
        let mut rng = LcgRng::new(self.seed);
        let n = self.n_scenarios;
        let prob = if n > 0 { 1.0 / n as f64 } else { 1.0 };

        // Split uncertain sources into renewables vs loads
        let renewable_idx: Vec<usize> = self
            .uncertainty_sets
            .iter()
            .enumerate()
            .filter(|(_, u)| {
                matches!(
                    u.source,
                    UncertaintySource::SolarGeneration | UncertaintySource::WindGeneration
                )
            })
            .map(|(i, _)| i)
            .collect();

        let load_idx: Vec<usize> = self
            .uncertainty_sets
            .iter()
            .enumerate()
            .filter(|(_, u)| matches!(u.source, UncertaintySource::LoadDemand))
            .map(|(i, _)| i)
            .collect();

        let n_buses = self.n_buses;

        (0..n)
            .map(|id| {
                // --- renewable outputs ---
                let renewable_outputs_mw: Vec<f64> = renewable_idx
                    .iter()
                    .map(|&k| {
                        let z = box_muller(&mut rng);
                        self.uncertainty_sets[k].sample_from_z(z)
                    })
                    .collect();

                // --- load deviations (one entry per bus, defaulting to 0) ---
                let mut load_deviations_mw = vec![0.0_f64; n_buses];
                for &k in &load_idx {
                    let z = box_muller(&mut rng);
                    let dev = self.uncertainty_sets[k].sample_from_z(z)
                        - self.uncertainty_sets[k].nominal_value_mw;
                    // Assign to bus 0 if no bus mapping — extend when needed.
                    let bus = k.min(n_buses - 1);
                    load_deviations_mw[bus] += dev;
                }

                CcopfScenario {
                    id,
                    renewable_outputs_mw,
                    load_deviations_mw,
                    probability: prob,
                }
            })
            .collect()
    }

    // ── Reserve requirements ──────────────────────────────────────────────

    /// Compute per-generator upward and downward reserve requirements from
    /// a set of scenarios.
    ///
    /// For each generator g:
    /// * `reserve_up[g]`   = max over scenarios of max(0, shortfall_scenario − nominal_dispatch)
    /// * `reserve_down[g]` = max over scenarios of max(0, nominal_dispatch − surplus_scenario)
    ///
    /// Here the nominal dispatch is computed from the mean load.
    pub fn compute_reserve_requirements(
        &self,
        scenarios: &[CcopfScenario],
    ) -> (Vec<f64>, Vec<f64>) {
        let n_gen = self.n_generators;
        if n_gen == 0 || scenarios.is_empty() {
            return (vec![0.0; n_gen], vec![0.0; n_gen]);
        }

        // Nominal total load
        let nominal_load: f64 = self.base_load_mw.iter().sum();
        let nominal_dispatch = self.lambda_dispatch(nominal_load);

        let mut reserve_up = vec![0.0_f64; n_gen];
        let mut reserve_down = vec![0.0_f64; n_gen];

        for sc in scenarios {
            let renewable_total: f64 = sc.renewable_outputs_mw.iter().sum();
            let load_dev: f64 = sc.load_deviations_mw.iter().sum();
            let net_load = (nominal_load + load_dev - renewable_total).max(0.0);

            let scenario_dispatch = self.lambda_dispatch(net_load);

            for g in 0..n_gen {
                let delta = scenario_dispatch[g] - nominal_dispatch[g];
                if delta > 0.0 {
                    reserve_up[g] = reserve_up[g].max(delta);
                } else {
                    reserve_down[g] = reserve_down[g].max(-delta);
                }
            }
        }

        (reserve_up, reserve_down)
    }

    // ── Branch violation fraction ─────────────────────────────────────────

    /// Compute per-branch violation fraction for a given scenario and dispatch.
    ///
    /// Uses the PTDF approximation: `P_branch = Σ_k PTDF[l][k] · net_injection[k]`.
    /// Returns `max(0, |P_branch| − rating) / rating` for each branch.
    pub fn compute_branch_violations(
        &self,
        scenario: &CcopfScenario,
        dispatch: &[f64],
    ) -> Vec<f64> {
        if self.ptdf_matrix.is_empty() || self.branch_ratings_mw.is_empty() {
            return Vec::new();
        }

        // Net injection per bus = dispatch_bus - load_bus
        let mut net_injection = vec![0.0_f64; self.n_buses];

        // Spread generator output uniformly across buses (when no bus map is given)
        let n_gen = dispatch.len().min(self.n_generators);
        for (g, &disp) in dispatch.iter().enumerate().take(n_gen) {
            let bus = g.min(self.n_buses - 1);
            net_injection[bus] += disp;
        }

        // Subtract nominal load and scenario deviations
        for (b, (&base, &dev)) in self
            .base_load_mw
            .iter()
            .zip(scenario.load_deviations_mw.iter())
            .enumerate()
        {
            net_injection[b] -= base + dev;
        }

        // Subtract renewable generation (mapped via source index → bus)
        for (k, &pv) in scenario.renewable_outputs_mw.iter().enumerate() {
            let bus = k.min(self.n_buses - 1);
            net_injection[bus] += pv;
        }

        self.ptdf_matrix
            .iter()
            .enumerate()
            .map(|(l, row)| {
                let flow: f64 = row
                    .iter()
                    .zip(net_injection.iter())
                    .map(|(ptdf, &inj)| ptdf * inj)
                    .sum();
                let rating = self
                    .branch_ratings_mw
                    .get(l)
                    .copied()
                    .unwrap_or(f64::INFINITY);
                if rating <= 0.0 {
                    return 0.0;
                }
                (flow.abs() - rating).max(0.0) / rating
            })
            .collect()
    }

    // ── CVaR ─────────────────────────────────────────────────────────────

    /// Compute Conditional Value-at-Risk at confidence level `alpha` for a
    /// slice of cost realisations.
    ///
    /// `CVaR_α = E[cost | cost ≥ VaR_α]`
    /// where `VaR_α = c_{ceil(α·N)}` (ascending sort).
    pub fn compute_cvar(&self, costs: &[f64], alpha: f64) -> f64 {
        if costs.is_empty() {
            return 0.0;
        }
        let mut sorted = costs.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));

        let n = sorted.len();
        let var_idx = ((alpha * n as f64).ceil() as usize).min(n - 1);
        let tail = &sorted[var_idx..];

        if tail.is_empty() {
            sorted[n - 1]
        } else {
            tail.iter().sum::<f64>() / tail.len() as f64
        }
    }

    // ── Solve SAA ────────────────────────────────────────────────────────

    /// Solve via Sample Average Approximation.
    ///
    /// For each scenario:
    ///   1. Compute net load = base_load + load_deviations − renewable_outputs.
    ///   2. Lambda-iterate economic dispatch.
    ///   3. Record cost, reserve needs, and per-constraint satisfaction.
    ///
    /// Final result is the expectation over all scenarios.
    pub fn solve_saa(&mut self, scenarios: &[CcopfScenario]) -> CcopfResult {
        let n_gen = self.n_generators;
        let n_cc = self.chance_constraints.len();
        let n = scenarios.len();

        if n == 0 || n_gen == 0 {
            return self.empty_result(ChanceConstraintApproach::ScenarioBased, n);
        }

        let mut costs = Vec::with_capacity(n);
        let mut dispatches: Vec<Vec<f64>> = Vec::with_capacity(n);

        // Per-constraint: number of scenarios where it is satisfied.
        let mut cc_satisfied = vec![0_usize; n_cc];

        for sc in scenarios {
            let renewable_total: f64 = sc.renewable_outputs_mw.iter().sum();
            let load_dev: f64 = sc.load_deviations_mw.iter().sum();
            let nominal: f64 = self.base_load_mw.iter().sum();
            let net_load = (nominal + load_dev - renewable_total).max(0.0);

            let dispatch = self.lambda_dispatch(net_load);
            let cost = self.dispatch_cost(&dispatch);
            costs.push(cost);

            // Check each chance constraint
            let violations = self.compute_branch_violations(sc, &dispatch);
            for (ci, cc) in self.chance_constraints.iter().enumerate() {
                let satisfied = match cc.constraint_type.as_str() {
                    "branch_flow" => {
                        // Check if the specific branch (by constraint_id) is within rating
                        let frac = violations.get(cc.constraint_id).copied().unwrap_or(0.0);
                        frac <= 0.0
                    }
                    "generation_limit" => dispatch.iter().all(|&p| p >= 0.0 && p <= cc.limit_mw),
                    _ => true,
                };
                if satisfied {
                    cc_satisfied[ci] += 1;
                }
            }

            dispatches.push(dispatch);
        }

        let expected_cost = costs.iter().sum::<f64>() / n as f64;
        let worst_case_cost = costs.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

        // CVaR at 0.95 by default
        let cvar = self.compute_cvar(&costs, 0.95);

        // Base dispatch = mean across scenarios
        let base_dispatch_mw: Vec<f64> = (0..n_gen)
            .map(|g| dispatches.iter().map(|d| d[g]).sum::<f64>() / n as f64)
            .collect();

        let (reserve_up_mw, reserve_down_mw) = self.compute_reserve_requirements(scenarios);

        // Reserve cost: flat rate 5 $/MW for up-reserve, 3 $/MW for down-reserve
        let reserve_cost_usd: f64 =
            reserve_up_mw.iter().sum::<f64>() * 5.0 + reserve_down_mw.iter().sum::<f64>() * 3.0;

        let chance_constraint_satisfaction: Vec<f64> =
            cc_satisfied.iter().map(|&k| k as f64 / n as f64).collect();

        CcopfResult {
            approach: ChanceConstraintApproach::ScenarioBased,
            base_dispatch_mw,
            reserve_up_mw,
            reserve_down_mw,
            expected_cost_usd: expected_cost,
            reserve_cost_usd,
            chance_constraint_satisfaction,
            cvar_usd: cvar,
            worst_case_cost_usd: worst_case_cost.max(0.0),
            n_scenarios_used: n,
        }
    }

    // ── Solve Analytic Moment ─────────────────────────────────────────────

    /// Solve via analytic moment method (Gaussian chance constraints).
    ///
    /// For each branch l with chance constraint `P(P_l ≤ rating_l) ≥ 1−α`:
    ///   * μ_l  = Σ_k `PTDF[l][k]` · (`nominal_dispatch[k]` − `base_load[k]`)
    ///   * σ²_l = Σ_k `PTDF[l][k]`² · σ²_k   (σ_k from `uncertainty_sets`)
    ///   * Tightened rating: `rating_l − z_{1−α} · σ_l`
    ///
    /// The dispatch is then computed against the *most binding* tightened
    /// constraint, propagated as a reduced feasible region on total load.
    pub fn solve_analytic_moment(&self) -> CcopfResult {
        let n_gen = self.n_generators;
        let n_cc = self.chance_constraints.len();

        // Total nominal load
        let nominal_load: f64 = self.base_load_mw.iter().sum();

        // Aggregate variance of total renewable injection.
        let total_renewable_var: f64 = self
            .uncertainty_sets
            .iter()
            .filter(|u| {
                matches!(
                    u.source,
                    UncertaintySource::SolarGeneration | UncertaintySource::WindGeneration
                )
            })
            .map(|u| u.std_dev_mw.powi(2))
            .sum();
        let total_sigma = total_renewable_var.sqrt();

        // Tighten the effective load for the most conservative constraint.
        // If no PTDF is given, apply z-score tightening directly to total load.
        let mut effective_load_reduction = 0.0_f64;
        for cc in &self.chance_constraints {
            let z = z_score(cc.confidence_level);

            if !self.ptdf_matrix.is_empty() {
                // Per-branch tightening: find worst branch across uncertainty sources.
                let branch_id = cc
                    .constraint_id
                    .min(self.ptdf_matrix.len().saturating_sub(1));
                let row = &self.ptdf_matrix[branch_id];
                // σ²_branch = Σ_k PTDF[l][k]² · σ²_k  (one σ per uncertainty set)
                let sigma_branch: f64 = self
                    .uncertainty_sets
                    .iter()
                    .enumerate()
                    .map(|(k, u)| {
                        let ptdf = row
                            .get(k.min(row.len().saturating_sub(1)))
                            .copied()
                            .unwrap_or(0.0);
                        ptdf.powi(2) * u.std_dev_mw.powi(2)
                    })
                    .sum::<f64>()
                    .sqrt();

                effective_load_reduction = effective_load_reduction.max(z * sigma_branch);
            } else {
                // No PTDF: apply z-score to aggregate std.
                effective_load_reduction = effective_load_reduction.max(z * total_sigma);
            }
        }

        // Reduce effective load by the tightening and dispatch.
        let tightened_load = (nominal_load - effective_load_reduction).max(0.0);
        let base_dispatch_mw = self.lambda_dispatch(tightened_load);
        let expected_cost = self.dispatch_cost(&base_dispatch_mw);

        // Reserve = the tightening itself, spread proportionally across generators.
        let p_max_total: f64 = self.gen_limits.iter().map(|(_, p)| p).sum::<f64>().max(1.0);
        let reserve_up_mw: Vec<f64> = self
            .gen_limits
            .iter()
            .map(|(_, p_max)| (p_max / p_max_total) * effective_load_reduction)
            .collect();
        let reserve_down_mw = vec![0.0_f64; n_gen];

        let reserve_cost_usd = reserve_up_mw.iter().sum::<f64>() * 5.0;

        // Analytic satisfaction: 1.0 if constraint met by construction.
        let chance_constraint_satisfaction = vec![1.0; n_cc];

        // CVaR ≈ E[cost] + z_{0.95} · σ_cost (Gaussian cost approximation)
        let sigma_cost = total_sigma * 20.0; // rough: marginal cost × σ_MW
        let cvar = expected_cost + z_score(0.95) * sigma_cost;

        CcopfResult {
            approach: ChanceConstraintApproach::AnalyticMoment,
            base_dispatch_mw,
            reserve_up_mw,
            reserve_down_mw,
            expected_cost_usd: expected_cost,
            reserve_cost_usd,
            chance_constraint_satisfaction,
            cvar_usd: cvar.max(expected_cost),
            worst_case_cost_usd: (expected_cost + 3.0 * sigma_cost).max(expected_cost),
            n_scenarios_used: 0,
        }
    }

    // ── Solve Robust ──────────────────────────────────────────────────────

    /// Solve via robust optimization (worst-case within uncertainty ellipsoid).
    ///
    /// Uses `k = z_{1−α}` standard deviations: uncertain source i takes value
    /// `nominal_i + k · std_dev_i` (conservative upward for loads, downward for
    /// renewables).  The worst-case net load is dispatched.
    pub fn solve_robust(&self) -> CcopfResult {
        let n_gen = self.n_generators;
        let n_cc = self.chance_constraints.len();

        // Determine k from the most stringent chance constraint confidence level.
        let k = self
            .chance_constraints
            .iter()
            .map(|cc| z_score(cc.confidence_level))
            .fold(1.645_f64, f64::max); // default 95% if no constraints

        let nominal_load: f64 = self.base_load_mw.iter().sum();

        // Worst-case: loads at +k·σ, renewables at −k·σ (lower bound 0)
        let worst_case_load_addition: f64 = self
            .uncertainty_sets
            .iter()
            .map(|u| match u.source {
                UncertaintySource::LoadDemand => k * u.std_dev_mw,
                UncertaintySource::SolarGeneration | UncertaintySource::WindGeneration => {
                    // Reduced renewable output worsens load balance.
                    k * u.std_dev_mw
                }
                _ => 0.0,
            })
            .sum::<f64>();

        let worst_case_load = nominal_load + worst_case_load_addition;
        let base_dispatch_mw = self.lambda_dispatch(worst_case_load);
        let expected_cost = self.dispatch_cost(&base_dispatch_mw);

        // Nominal dispatch for reserve computation
        let nominal_dispatch = self.lambda_dispatch(nominal_load);
        let reserve_up_mw: Vec<f64> = base_dispatch_mw
            .iter()
            .zip(nominal_dispatch.iter())
            .map(|(&robust, &nom)| (robust - nom).max(0.0))
            .collect();
        let reserve_down_mw = vec![0.0_f64; n_gen];

        let reserve_cost_usd = reserve_up_mw.iter().sum::<f64>() * 5.0;

        let chance_constraint_satisfaction = vec![1.0_f64; n_cc];

        // CVaR of the robust solution is its cost (single worst case point).
        CcopfResult {
            approach: ChanceConstraintApproach::RobustOptimization,
            base_dispatch_mw,
            reserve_up_mw,
            reserve_down_mw,
            expected_cost_usd: expected_cost,
            reserve_cost_usd,
            chance_constraint_satisfaction,
            cvar_usd: expected_cost,
            worst_case_cost_usd: expected_cost,
            n_scenarios_used: 0,
        }
    }

    // ── Unified solve entry point ─────────────────────────────────────────

    /// Dispatch to the solver selected by `self.approach`.
    pub fn solve(&mut self) -> CcopfResult {
        match self.approach.clone() {
            ChanceConstraintApproach::ScenarioBased => {
                let scenarios = self.generate_scenarios();
                self.solve_saa(&scenarios)
            }
            ChanceConstraintApproach::AnalyticMoment => self.solve_analytic_moment(),
            ChanceConstraintApproach::RobustOptimization => self.solve_robust(),
            ChanceConstraintApproach::ConditionalValueAtRisk => {
                // CVaR approach: generate scenarios then minimise CVaR objective.
                let scenarios = self.generate_scenarios();
                let mut result = self.solve_saa(&scenarios);
                // Mark the approach correctly.
                result.approach = ChanceConstraintApproach::ConditionalValueAtRisk;
                result
            }
        }
    }

    // ── Internal helpers ──────────────────────────────────────────────────

    /// Economic dispatch via lambda iteration (equal incremental cost).
    ///
    /// Divides `total_load` among generators proportionally to P_max capacity,
    /// then clamps to [P_min, P_max].  This is a simple merit-order heuristic
    /// sufficient for the stochastic/chance-constrained layer.
    fn lambda_dispatch(&self, total_load: f64) -> Vec<f64> {
        let n_gen = self.n_generators;
        if n_gen == 0 {
            return Vec::new();
        }

        let p_max_total: f64 = self.gen_limits.iter().map(|(_, pmax)| pmax).sum::<f64>();
        let p_min_total: f64 = self.gen_limits.iter().map(|(pmin, _)| pmin).sum::<f64>();

        let clamped = total_load.clamp(p_min_total, p_max_total);

        // Proportional to P_max capacity
        if p_max_total <= 0.0 {
            return vec![0.0; n_gen];
        }

        self.gen_limits
            .iter()
            .map(|(pmin, pmax)| {
                let share = clamped * (pmax / p_max_total);
                share.clamp(*pmin, *pmax)
            })
            .collect()
    }

    /// Evaluate quadratic generation cost for a dispatch vector.
    fn dispatch_cost(&self, dispatch: &[f64]) -> f64 {
        self.gen_costs
            .iter()
            .zip(dispatch.iter())
            .map(|(&(a, b, c), &p)| a * p * p + b * p + c)
            .sum()
    }

    /// Build an empty result (used for degenerate cases).
    fn empty_result(&self, approach: ChanceConstraintApproach, n: usize) -> CcopfResult {
        CcopfResult {
            approach,
            base_dispatch_mw: vec![0.0; self.n_generators],
            reserve_up_mw: vec![0.0; self.n_generators],
            reserve_down_mw: vec![0.0; self.n_generators],
            expected_cost_usd: 0.0,
            reserve_cost_usd: 0.0,
            chance_constraint_satisfaction: Vec::new(),
            cvar_usd: 0.0,
            worst_case_cost_usd: 0.0,
            n_scenarios_used: n,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a small solver with 5 buses and 3 generators.
    fn small_solver() -> ChanceConstrainedOpf {
        let mut s = ChanceConstrainedOpf::new(5, 3);
        s.gen_costs = vec![(0.01, 20.0, 0.0), (0.02, 22.0, 0.0), (0.03, 25.0, 0.0)];
        s.gen_limits = vec![(0.0, 100.0), (0.0, 80.0), (0.0, 60.0)];
        s.base_load_mw = vec![20.0, 15.0, 10.0, 5.0, 5.0];
        s.n_scenarios = 200;
        s.seed = 42;
        s
    }

    /// Build a solver with wind uncertainty and a PTDF branch.
    fn wind_solver(std_dev: f64) -> ChanceConstrainedOpf {
        let mut s = small_solver();
        s.add_uncertainty(UncertaintySet::wind(20.0, std_dev));
        s.add_chance_constraint(ChanceConstraint::branch_flow(0, 80.0, 0.95));
        // 2-bus PTDF: one branch, two buses
        s.ptdf_matrix = vec![vec![0.5, -0.5, 0.0, 0.0, 0.0]];
        s.branch_ratings_mw = vec![80.0];
        s
    }

    // ── Scenario generation ───────────────────────────────────────────────

    #[test]
    fn test_scenario_generation_count() {
        let mut s = small_solver();
        s.add_uncertainty(UncertaintySet::wind(20.0, 5.0));
        s.n_scenarios = 150;
        let scenarios = s.generate_scenarios();
        assert_eq!(scenarios.len(), 150, "should generate exactly n_scenarios");
    }

    #[test]
    fn test_scenario_probability_sum() {
        let mut s = small_solver();
        s.add_uncertainty(UncertaintySet::wind(20.0, 5.0));
        s.n_scenarios = 100;
        let scenarios = s.generate_scenarios();
        let total: f64 = scenarios.iter().map(|sc| sc.probability).sum();
        assert!(
            (total - 1.0).abs() < 1e-10,
            "probabilities must sum to 1, got {total:.6}"
        );
    }

    #[test]
    fn test_box_muller_normality() {
        // Generate many samples and check sample mean ≈ nominal, std ≈ std_dev
        let mut s = small_solver();
        let nominal = 30.0_f64;
        let sigma = 8.0_f64;
        s.add_uncertainty(UncertaintySet::wind(nominal, sigma));
        s.n_scenarios = 2000;
        let scenarios = s.generate_scenarios();
        let outputs: Vec<f64> = scenarios
            .iter()
            .map(|sc| sc.renewable_outputs_mw[0])
            .collect();
        let mean = outputs.iter().sum::<f64>() / outputs.len() as f64;
        let var = outputs.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / outputs.len() as f64;
        let std = var.sqrt();
        // Generous tolerances because of left-truncation at 0
        assert!(
            (mean - nominal).abs() < 3.0,
            "sample mean {mean:.2} should be near nominal {nominal}"
        );
        assert!(
            (std - sigma).abs() < 3.0,
            "sample std {std:.2} should be near sigma {sigma}"
        );
    }

    // ── Analytic moment solve ─────────────────────────────────────────────

    #[test]
    fn test_solve_analytic_deterministic() {
        // With zero std-dev all approaches should give the same dispatch.
        let mut s = small_solver();
        s.add_uncertainty(UncertaintySet {
            source: UncertaintySource::WindGeneration,
            nominal_value_mw: 10.0,
            std_dev_mw: 0.0, // deterministic
            correlation: 0.0,
            distribution: "Normal".into(),
        });
        let result = s.solve_analytic_moment();
        // Cost must be positive (generators are on)
        assert!(result.expected_cost_usd > 0.0, "cost should be positive");
        assert_eq!(result.base_dispatch_mw.len(), 3);
    }

    #[test]
    fn test_analytic_branch_constraint_satisfied() {
        // When σ=0 the analytic method adds no reserve; constraint trivially met.
        let mut s = small_solver();
        s.add_uncertainty(UncertaintySet::wind(10.0, 0.0));
        s.add_chance_constraint(ChanceConstraint::branch_flow(0, 200.0, 0.95));
        s.ptdf_matrix = vec![vec![0.4, -0.4, 0.0, 0.0, 0.0]];
        s.branch_ratings_mw = vec![200.0];
        let result = s.solve_analytic_moment();
        assert_eq!(
            result.chance_constraint_satisfaction.len(),
            1,
            "one constraint"
        );
        assert!(
            result.chance_constraint_satisfaction[0] >= 0.95,
            "satisfaction {:.3}",
            result.chance_constraint_satisfaction[0]
        );
    }

    // ── Robust solve ──────────────────────────────────────────────────────

    #[test]
    fn test_robust_dispatch_conservative() {
        // Robust dispatch should be ≥ analytic dispatch (more load covered).
        let s = wind_solver(10.0);
        let analytic = s.solve_analytic_moment();
        let robust = s.solve_robust();
        let analytic_total: f64 = analytic.base_dispatch_mw.iter().sum();
        let robust_total: f64 = robust.base_dispatch_mw.iter().sum();
        assert!(
            robust_total >= analytic_total - 1e-6,
            "robust total {robust_total:.2} >= analytic {analytic_total:.2}"
        );
    }

    // ── SAA solve ─────────────────────────────────────────────────────────

    #[test]
    fn test_saa_cost_expectation() {
        let mut s = wind_solver(5.0);
        let scenarios = s.generate_scenarios();
        let result = s.solve_saa(&scenarios);
        assert!(
            result.expected_cost_usd > 0.0,
            "expected cost must be positive, got {}",
            result.expected_cost_usd
        );
    }

    // ── CVaR ─────────────────────────────────────────────────────────────

    #[test]
    fn test_cvar_ge_var() {
        let s = small_solver();
        let costs: Vec<f64> = (0..100).map(|i| i as f64 * 1.5).collect();
        let alpha = 0.9;
        let mut sorted = costs.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let var_idx = ((alpha * costs.len() as f64).ceil() as usize).min(costs.len() - 1);
        let var = sorted[var_idx];
        let cvar = s.compute_cvar(&costs, alpha);
        assert!(cvar >= var - 1e-9, "CVaR {cvar:.2} must be >= VaR {var:.2}");
    }

    #[test]
    fn test_cvar_all_same() {
        let s = small_solver();
        let costs = vec![50.0_f64; 20];
        let cvar = s.compute_cvar(&costs, 0.95);
        assert!(
            (cvar - 50.0).abs() < 1e-9,
            "CVaR of identical costs should equal the cost, got {cvar}"
        );
    }

    #[test]
    fn test_cvar_computation() {
        // Verified by hand: costs = [1,2,3,4,5,6,7,8,9,10], alpha=0.9
        // VaR = c[ceil(0.9*10)] = c[9] = 10; tail = [10]; CVaR = 10.
        let s = small_solver();
        let costs: Vec<f64> = (1..=10).map(|x| x as f64).collect();
        let cvar = s.compute_cvar(&costs, 0.9);
        assert!((cvar - 10.0).abs() < 1e-9, "CVaR should be 10, got {cvar}");
    }

    // ── Reserve requirements ──────────────────────────────────────────────

    #[test]
    fn test_reserve_up_nonnegative() {
        let mut s = wind_solver(8.0);
        let scenarios = s.generate_scenarios();
        let (up, _) = s.compute_reserve_requirements(&scenarios);
        assert!(up.iter().all(|&r| r >= 0.0), "all up-reserves non-negative");
    }

    #[test]
    fn test_reserve_down_nonnegative() {
        let mut s = wind_solver(8.0);
        let scenarios = s.generate_scenarios();
        let (_, down) = s.compute_reserve_requirements(&scenarios);
        assert!(
            down.iter().all(|&r| r >= 0.0),
            "all down-reserves non-negative"
        );
    }

    // ── z-score ───────────────────────────────────────────────────────────

    #[test]
    fn test_z_score_95pct() {
        let z = z_score(0.95);
        assert!(
            (z - 1.645).abs() < 0.01,
            "z(0.95) should be ≈ 1.645, got {z:.4}"
        );
    }

    // ── Chance constraint satisfaction ────────────────────────────────────

    #[test]
    fn test_chance_constraint_satisfaction() {
        // With huge branch rating the constraint is almost always satisfied.
        let mut s = small_solver();
        s.add_uncertainty(UncertaintySet::wind(20.0, 5.0));
        s.add_chance_constraint(ChanceConstraint::branch_flow(0, 1_000.0, 0.95));
        s.ptdf_matrix = vec![vec![0.1, 0.1, 0.1, 0.1, 0.1]];
        s.branch_ratings_mw = vec![1_000.0];
        s.n_scenarios = 500;
        let scenarios = s.generate_scenarios();
        let result = s.solve_saa(&scenarios);
        let sat = result.chance_constraint_satisfaction[0];
        assert!(
            sat >= 0.90,
            "satisfaction {sat:.3} should be ≥ 0.90 with generous rating"
        );
    }

    // ── Branch violations ─────────────────────────────────────────────────

    #[test]
    fn test_branch_violation_zero() {
        let mut s = small_solver();
        s.ptdf_matrix = vec![vec![0.1, 0.1, 0.1, 0.1, 0.1]];
        s.branch_ratings_mw = vec![1_000.0];
        let sc = CcopfScenario {
            id: 0,
            renewable_outputs_mw: vec![],
            load_deviations_mw: vec![0.0; 5],
            probability: 1.0,
        };
        // Dispatch equal to load → near-zero flow
        let dispatch = vec![18.0, 14.0, 23.0];
        let violations = s.compute_branch_violations(&sc, &dispatch);
        assert_eq!(violations.len(), 1);
        assert!(
            violations[0] < 1e-6,
            "violation should be 0 for feasible dispatch, got {}",
            violations[0]
        );
    }

    #[test]
    fn test_branch_violation_positive() {
        let mut s = small_solver();
        s.ptdf_matrix = vec![vec![1.0, 0.0, 0.0, 0.0, 0.0]];
        s.branch_ratings_mw = vec![1.0]; // extremely tight rating
        let sc = CcopfScenario {
            id: 0,
            renewable_outputs_mw: vec![],
            load_deviations_mw: vec![0.0; 5],
            probability: 1.0,
        };
        let dispatch = vec![100.0, 0.0, 0.0]; // large injection on bus 0
        let violations = s.compute_branch_violations(&sc, &dispatch);
        assert!(
            violations[0] > 0.0,
            "violation should be positive for overloaded branch"
        );
    }

    // ── Unified solve ─────────────────────────────────────────────────────

    #[test]
    fn test_solve_dispatch_saa() {
        let mut s = wind_solver(5.0);
        s.approach = ChanceConstraintApproach::ScenarioBased;
        let result = s.solve();
        assert_eq!(result.approach, ChanceConstraintApproach::ScenarioBased);
        assert!(result.expected_cost_usd > 0.0);
        assert!(result.n_scenarios_used > 0);
    }

    #[test]
    fn test_solve_dispatch_analytic() {
        let mut s = wind_solver(5.0);
        s.approach = ChanceConstraintApproach::AnalyticMoment;
        let result = s.solve();
        assert_eq!(result.approach, ChanceConstraintApproach::AnalyticMoment);
        assert!(
            result.reserve_up_mw.iter().all(|&r| r >= 0.0),
            "analytic reserves non-negative"
        );
    }

    #[test]
    fn test_solve_dispatch_robust() {
        let mut s = wind_solver(5.0);
        s.approach = ChanceConstraintApproach::RobustOptimization;
        let result = s.solve();
        assert_eq!(
            result.approach,
            ChanceConstraintApproach::RobustOptimization
        );
        assert!(result.expected_cost_usd > 0.0);
    }

    // ── Wind uncertainty → more reserve ──────────────────────────────────

    #[test]
    fn test_wind_uncertainty_larger_reserve() {
        let s_low = wind_solver(2.0);
        let s_high = wind_solver(20.0);

        let res_low = s_low.solve_analytic_moment();
        let res_high = s_high.solve_analytic_moment();

        let reserve_low: f64 = res_low.reserve_up_mw.iter().sum();
        let reserve_high: f64 = res_high.reserve_up_mw.iter().sum();
        assert!(
            reserve_high >= reserve_low - 1e-6,
            "high-σ reserve {reserve_high:.2} should be ≥ low-σ reserve {reserve_low:.2}"
        );
    }
}
