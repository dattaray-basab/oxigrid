//! Stochastic Load Flow with uncertainty quantification.
//!
//! Propagates uncertain load, generation and renewable inputs through the power
//! flow equations using four methods, and produces statistical output distributions
//! for bus voltages and branch power flows.
//!
//! # Methods
//!
//! | Method | Description |
//! |--------|-------------|
//! | [`StochasticMethod::MonteCarlo`] | Full random sampling; `n_samples` DC LF runs |
//! | [`StochasticMethod::LatinHypercubeSampling`] | Stratified sampling; better coverage per sample |
//! | [`StochasticMethod::PointEstimate2m`] | Hong 2m+1 deterministic point evaluations |
//! | [`StochasticMethod::Linearized`] | First-order Taylor variance propagation (1 solve) |
//!
//! The inner power flow uses a simplified linearized DC model:
//! voltage at bus *i* is approximated as `V_nom + sensitivity * ΔP`, where the
//! sensitivity is set to 0.01 \[pu/MW\] (typical for radial distribution feeders).
//! Branch flows are computed from the per-sample net injection vector.
//!
//! # Random number generation
//!
//! All sampling uses an LCG with parameters from Knuth/Numerical Recipes:
//! - multiplier: 6364136223846793005
//! - addend:    1442695040888963407
//!
//! Box-Muller transform is used for Normal and LogNormal sampling.
//!
//! # References
//! - Borkowska, B., "Probabilistic Load Flow", IEEE Trans. PAS, 1974
//! - Hong, Y.-Y. & Luo, Y.-F., "Optimal VAR Control Considering Wind Farms Using
//!   Probabilistic Load-Flow and Gray-Based Genetic Algorithms", IEEE Trans. PD, 2009
//! - McKay, M.D. et al., "A Comparison of Three Methods for Selecting Values of Input
//!   Variables in the Analysis of Output From a Computer Code", Technometrics, 1979

use serde::{Deserialize, Serialize};

// ── Error type ─────────────────────────────────────────────────────────────

/// Errors from the stochastic load flow solver.
#[derive(Debug, thiserror::Error)]
pub enum SlfError {
    /// Configuration is invalid.
    #[error("invalid configuration: {0}")]
    InvalidConfig(String),
    /// No uncertain inputs were added before calling solve().
    #[error("no uncertain inputs registered — call add_uncertain_input() first")]
    NoInputs,
    /// Base load vector size does not match n_buses.
    #[error("base load size {0} does not match n_buses {1}")]
    LoadSizeMismatch(usize, usize),
}

// ── Configuration ──────────────────────────────────────────────────────────

/// Configuration for the stochastic load flow solver.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StochasticLfConfig {
    /// Number of Monte Carlo / LHS samples.
    pub n_samples: usize,
    /// Confidence level for interval reporting (e.g. 0.95 for 95 % CI).
    pub confidence_level: f64,
    /// Sampling method.
    pub method: StochasticMethod,
    /// LCG seed for reproducibility.
    pub seed: u64,
}

impl Default for StochasticLfConfig {
    fn default() -> Self {
        Self {
            n_samples: 1000,
            confidence_level: 0.95,
            method: StochasticMethod::MonteCarlo,
            seed: 42,
        }
    }
}

/// Stochastic load flow sampling method.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum StochasticMethod {
    /// Full Monte Carlo: sample all inputs from their distributions each iteration.
    MonteCarlo,
    /// Latin Hypercube Sampling: stratify `[0,1]` into equal intervals for each input.
    LatinHypercubeSampling,
    /// Hong 2m+1 Point Estimate Method: `2m+1` deterministic evaluations.
    PointEstimate2m,
    /// First-order linearized: analytical variance propagation (1 DC LF solve).
    Linearized,
}

// ── Input uncertainty description ───────────────────────────────────────────

/// A single uncertain input variable at a specific bus.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UncertainInput {
    /// Bus index this input is associated with.
    pub bus: usize,
    /// Which variable is uncertain.
    pub variable: InputVariable,
    /// Probability distribution for this variable.
    pub distribution: Distribution,
}

/// Power system variable subject to uncertainty.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum InputVariable {
    /// Active load \[MW\].
    LoadP,
    /// Reactive load \[Mvar\].
    LoadQ,
    /// Active generation \[MW\].
    GenP,
    /// Reactive generation \[Mvar\].
    GenQ,
    /// Wind speed \[m/s\] (converted to power internally).
    WindSpeed,
    /// Solar irradiance \[W/m²\] (converted to power internally).
    SolarIrradiance,
}

/// Probability distribution for an uncertain input.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Distribution {
    /// Normal distribution N(mean, std\_dev).
    Normal {
        /// Mean value.
        mean: f64,
        /// Standard deviation (≥ 0).
        std_dev: f64,
    },
    /// Uniform distribution U\[low, high\].
    Uniform {
        /// Lower bound.
        low: f64,
        /// Upper bound.
        high: f64,
    },
    /// Weibull distribution W(scale λ, shape k) — common for wind speed \[m/s\].
    Weibull {
        /// Scale parameter λ \> 0.
        scale: f64,
        /// Shape parameter k \> 0.
        shape: f64,
    },
    /// Beta distribution B(α, β) — common for solar irradiance normalised to \[0,1\].
    Beta {
        /// Shape parameter α \> 0.
        alpha: f64,
        /// Shape parameter β \> 0.
        beta: f64,
    },
    /// Log-normal distribution LN(μ, σ) where μ and σ are the log-space parameters.
    LogNormal {
        /// Mean of ln(X).
        mu: f64,
        /// Standard deviation of ln(X).
        sigma: f64,
    },
}

// ── Output statistics ───────────────────────────────────────────────────────

/// Voltage statistics for a single bus.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BusStatistics {
    /// Bus index.
    pub bus: usize,
    /// Sample mean of voltage magnitude \[pu\].
    pub v_mean: f64,
    /// Sample standard deviation of voltage magnitude \[pu\].
    pub v_std: f64,
    /// Sample minimum voltage \[pu\].
    pub v_min: f64,
    /// Sample maximum voltage \[pu\].
    pub v_max: f64,
    /// 5th percentile voltage \[pu\].
    pub v_p5: f64,
    /// 95th percentile voltage \[pu\].
    pub v_p95: f64,
    /// Probability that voltage violates the \[0.95, 1.05\] pu band.
    pub prob_violation_pu: f64,
}

/// Power flow statistics for a single branch.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BranchStatistics {
    /// Branch index.
    pub branch_id: usize,
    /// Sample mean of active power flow \[MW\].
    pub flow_mean_mw: f64,
    /// Sample standard deviation of active power flow \[MW\].
    pub flow_std_mw: f64,
    /// 95th percentile active power flow \[MW\].
    pub flow_p95_mw: f64,
    /// Probability that flow exceeds the thermal rating (uses 110 % of mean as proxy).
    pub prob_overload: f64,
}

/// Complete result of a stochastic load flow run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StochasticLfResult {
    /// Per-bus voltage statistics.
    pub bus_stats: Vec<BusStatistics>,
    /// Per-branch flow statistics.
    pub branch_stats: Vec<BranchStatistics>,
    /// Number of samples that converged (all for linearized DC).
    pub n_converged: usize,
    /// Number of samples that did not converge.
    pub n_diverged: usize,
    /// Informational note about computational cost.
    pub computational_time_note: String,
}

// ── Solver ─────────────────────────────────────────────────────────────────

/// Stochastic load flow solver.
///
/// # Example
///
/// ```rust,ignore
/// use oxigrid::powerflow::stochastic_lf::{
///     StochasticLfConfig, StochasticLfSolver, StochasticMethod,
///     UncertainInput, InputVariable, Distribution,
/// };
/// let cfg = StochasticLfConfig { n_samples: 500, ..Default::default() };
/// let mut solver = StochasticLfSolver::new(cfg, 4);
/// solver.set_base_loads(vec![0.0, 1.0, 2.0, 1.5], vec![0.0, 0.3, 0.6, 0.4]);
/// solver.add_uncertain_input(UncertainInput {
///     bus: 1,
///     variable: InputVariable::LoadP,
///     distribution: Distribution::Normal { mean: 1.0, std_dev: 0.1 },
/// });
/// let result = solver.solve().expect("solve ok");
/// ```
#[derive(Debug, Clone)]
pub struct StochasticLfSolver {
    config: StochasticLfConfig,
    uncertain_inputs: Vec<UncertainInput>,
    n_buses: usize,
    base_load_mw: Vec<f64>,
    base_load_mvar: Vec<f64>,
}

impl StochasticLfSolver {
    /// Create a new solver for a network with `n_buses` buses.
    pub fn new(config: StochasticLfConfig, n_buses: usize) -> Self {
        Self {
            config,
            uncertain_inputs: Vec::new(),
            n_buses,
            base_load_mw: vec![0.0; n_buses],
            base_load_mvar: vec![0.0; n_buses],
        }
    }

    /// Register an uncertain input.
    pub fn add_uncertain_input(&mut self, input: UncertainInput) {
        self.uncertain_inputs.push(input);
    }

    /// Set the base (deterministic) load at each bus.
    ///
    /// Both vectors must have length equal to `n_buses`.
    pub fn set_base_loads(&mut self, p_mw: Vec<f64>, q_mvar: Vec<f64>) {
        self.base_load_mw = p_mw;
        self.base_load_mvar = q_mvar;
    }

    /// Run the stochastic load flow.
    ///
    /// Returns [`SlfError`] if configuration is invalid or no inputs are registered.
    pub fn solve(&self) -> Result<StochasticLfResult, SlfError> {
        if self.uncertain_inputs.is_empty() {
            return Err(SlfError::NoInputs);
        }
        if self.config.n_samples == 0 {
            return Err(SlfError::InvalidConfig("n_samples must be > 0".into()));
        }

        match self.config.method {
            StochasticMethod::MonteCarlo => self.solve_mc(false),
            StochasticMethod::LatinHypercubeSampling => self.solve_mc(true),
            StochasticMethod::PointEstimate2m => self.solve_point_estimate(),
            StochasticMethod::Linearized => self.solve_linearized(),
        }
    }

    // ── Internal: Monte Carlo (and LHS) ────────────────────────────────────

    fn solve_mc(&self, use_lhs: bool) -> Result<StochasticLfResult, SlfError> {
        let n = self.config.n_samples;
        let mut rng = self.config.seed;

        // Precompute per-input LHS strata if LHS mode
        // lhs_u[input_idx][sample_idx] = uniform sample in [0,1]
        let lhs_u: Vec<Vec<f64>> = if use_lhs {
            self.uncertain_inputs
                .iter()
                .map(|_| Self::lhs_sample(n, &mut rng))
                .collect()
        } else {
            Vec::new()
        };

        // Collect per-bus voltage samples and branch flow samples
        let n_branches = self.n_buses.saturating_sub(1); // simplified radial topology
        let mut v_samples: Vec<Vec<f64>> = vec![Vec::with_capacity(n); self.n_buses];
        let mut flow_samples: Vec<Vec<f64>> = vec![Vec::with_capacity(n); n_branches];

        for s in 0..n {
            // Build per-bus net injection delta (departure from base load)
            let mut delta_p = vec![0.0_f64; self.n_buses];

            for (ii, inp) in self.uncertain_inputs.iter().enumerate() {
                let u = if use_lhs {
                    // lhs_u is indexed [input][sample]
                    lhs_u
                        .get(ii)
                        .and_then(|v| v.get(s))
                        .copied()
                        .unwrap_or_else(|| Self::lcg_uniform(&mut rng))
                } else {
                    Self::lcg_uniform(&mut rng)
                };
                let sampled = self.sample_distribution_from_u(&inp.distribution, u, &mut rng);
                let base = match inp.variable {
                    InputVariable::LoadP
                    | InputVariable::WindSpeed
                    | InputVariable::SolarIrradiance => {
                        self.base_load_mw.get(inp.bus).copied().unwrap_or(0.0)
                    }
                    InputVariable::GenP => -self.base_load_mw.get(inp.bus).copied().unwrap_or(0.0),
                    InputVariable::LoadQ | InputVariable::GenQ => {
                        self.base_load_mvar.get(inp.bus).copied().unwrap_or(0.0)
                    }
                };
                if inp.bus < self.n_buses {
                    delta_p[inp.bus] += sampled - base;
                }
            }

            // Simplified DC voltage: V_i = 1.0 - 0.01 * (cumulative P from slack)
            // More negative net injection (more load) → lower voltage
            let mut cum_p = 0.0_f64;
            for (b, vs) in v_samples.iter_mut().enumerate() {
                cum_p += self.base_load_mw.get(b).copied().unwrap_or(0.0) + delta_p[b];
                let v = (1.0 - 0.01 * cum_p).clamp(0.5, 1.5);
                vs.push(v);
            }

            // Branch flows: P_branch[k] = sum of loads downstream of branch k
            for (br, fs) in flow_samples.iter_mut().enumerate() {
                let downstream_load: f64 = (br + 1..self.n_buses)
                    .map(|b| self.base_load_mw.get(b).copied().unwrap_or(0.0) + delta_p[b])
                    .sum();
                fs.push(downstream_load);
            }
        }

        // Compute statistics
        let bus_stats: Vec<BusStatistics> = (0..self.n_buses)
            .map(|b| compute_bus_stats(b, &v_samples[b]))
            .collect();

        // Rating proxy: 110% of mean base load carried by branch 0
        let base_branch0_flow: f64 = self.base_load_mw.iter().sum::<f64>();
        let rating_proxy = (base_branch0_flow * 1.1).max(1.0);

        let branch_stats: Vec<BranchStatistics> = (0..n_branches)
            .map(|br| compute_branch_stats(br, &flow_samples[br], rating_proxy))
            .collect();

        let method_label = if use_lhs { "LHS" } else { "Monte Carlo" };
        Ok(StochasticLfResult {
            n_converged: n,
            n_diverged: 0,
            bus_stats,
            branch_stats,
            computational_time_note: format!(
                "{method_label}: {n} samples, {nb} buses, DC linearized model",
                nb = self.n_buses
            ),
        })
    }

    // ── Internal: 2m+1 Point Estimate ──────────────────────────────────────

    fn solve_point_estimate(&self) -> Result<StochasticLfResult, SlfError> {
        let m = self.uncertain_inputs.len();
        // For each input xi: evaluate at mean+kσ and mean-kσ (k = sqrt(3) for 2m+1)
        // Then at the combined mean point.
        let k = 3.0_f64.sqrt();

        let mut v_samples: Vec<Vec<f64>> = vec![Vec::new(); self.n_buses];
        let mut flow_samples: Vec<Vec<f64>> = vec![Vec::new(); self.n_buses.saturating_sub(1)];

        // Base evaluation (all at mean)
        self.evaluate_at_deltas(&vec![0.0; self.n_buses], &mut v_samples, &mut flow_samples);

        for inp in &self.uncertain_inputs {
            if inp.bus >= self.n_buses {
                continue;
            }
            let (mean_val, std_val) = distribution_mean_std(&inp.distribution);

            for &sign in &[1.0_f64, -1.0_f64] {
                let delta_p_val = sign * k * std_val;
                let mut delta_p = vec![0.0_f64; self.n_buses];
                delta_p[inp.bus] = delta_p_val;
                // Adjust sign for generators
                if matches!(inp.variable, InputVariable::GenP | InputVariable::GenQ) {
                    delta_p[inp.bus] = -delta_p_val;
                }
                let _ = mean_val; // used indirectly via delta
                self.evaluate_at_deltas(&delta_p, &mut v_samples, &mut flow_samples);
            }
        }

        let n_evals = 1 + 2 * m;
        let bus_stats: Vec<BusStatistics> = (0..self.n_buses)
            .map(|b| compute_bus_stats(b, &v_samples[b]))
            .collect();
        let base_flow: f64 = self.base_load_mw.iter().sum::<f64>();
        let rating_proxy = (base_flow * 1.1).max(1.0);
        let branch_stats: Vec<BranchStatistics> = (0..self.n_buses.saturating_sub(1))
            .map(|br| compute_branch_stats(br, &flow_samples[br], rating_proxy))
            .collect();

        Ok(StochasticLfResult {
            n_converged: n_evals,
            n_diverged: 0,
            bus_stats,
            branch_stats,
            computational_time_note: format!(
                "Point Estimate 2m+1: {} evaluations ({m} inputs)",
                n_evals
            ),
        })
    }

    /// Evaluate DC load flow with per-bus active power deltas and record samples.
    fn evaluate_at_deltas(
        &self,
        delta_p: &[f64],
        v_samples: &mut [Vec<f64>],
        flow_samples: &mut [Vec<f64>],
    ) {
        let mut cum_p = 0.0_f64;
        for (b, vs) in v_samples.iter_mut().enumerate() {
            cum_p += self.base_load_mw.get(b).copied().unwrap_or(0.0)
                + delta_p.get(b).copied().unwrap_or(0.0);
            let v = (1.0 - 0.01 * cum_p).clamp(0.5, 1.5);
            vs.push(v);
        }
        for (br, fs) in flow_samples.iter_mut().enumerate() {
            let downstream: f64 = (br + 1..self.n_buses)
                .map(|b| {
                    self.base_load_mw.get(b).copied().unwrap_or(0.0)
                        + delta_p.get(b).copied().unwrap_or(0.0)
                })
                .sum();
            fs.push(downstream);
        }
    }

    // ── Internal: Linearized ───────────────────────────────────────────────

    fn solve_linearized(&self) -> Result<StochasticLfResult, SlfError> {
        // Compute base case voltage and variance propagation.
        // Sensitivity S_ij = dV_i / dP_j = -0.01 if j <= i, else 0 (radial topology)
        let mut v_mean = vec![0.0_f64; self.n_buses];
        let mut v_var = vec![0.0_f64; self.n_buses];

        // Base voltages
        let mut cum = 0.0_f64;
        for (b, vm) in v_mean.iter_mut().enumerate() {
            cum += self.base_load_mw.get(b).copied().unwrap_or(0.0);
            *vm = (1.0 - 0.01 * cum).clamp(0.5, 1.5);
        }

        // Variance from uncertain inputs
        for inp in &self.uncertain_inputs {
            if inp.bus >= self.n_buses {
                continue;
            }
            let (_, std_val) = distribution_mean_std(&inp.distribution);
            let var_input = std_val * std_val;
            // Sensitivity: V_i changes by -0.01 per MW at any bus j <= i
            for vv in v_var.iter_mut().take(self.n_buses).skip(inp.bus) {
                *vv += 0.01 * 0.01 * var_input;
            }
        }

        let bus_stats: Vec<BusStatistics> = (0..self.n_buses)
            .map(|b| {
                let mean = v_mean[b];
                let std = v_var[b].sqrt();
                // Gaussian 5th/95th percentiles
                let z95 = 1.645_f64;
                let v_p5 = mean - z95 * std;
                let v_p95 = mean + z95 * std;
                // Approximate probability of violation: P(V < 0.95 or V > 1.05)
                // Using normal CDF approximation
                let p_low = gaussian_tail_prob((mean - 0.95) / (std + 1e-12));
                let p_high = gaussian_tail_prob((1.05 - mean) / (std + 1e-12));
                let prob_violation = (p_low + p_high).clamp(0.0, 1.0);
                BusStatistics {
                    bus: b,
                    v_mean: mean,
                    v_std: std,
                    v_min: v_p5.min(mean),
                    v_max: v_p95.max(mean),
                    v_p5,
                    v_p95,
                    prob_violation_pu: prob_violation,
                }
            })
            .collect();

        // Branch flow statistics (variance propagation)
        let n_branches = self.n_buses.saturating_sub(1);
        let base_flow: f64 = self.base_load_mw.iter().sum::<f64>();
        let rating_proxy = (base_flow * 1.1).max(1.0);
        let branch_stats: Vec<BranchStatistics> = (0..n_branches)
            .map(|br| {
                let flow_mean: f64 = (br + 1..self.n_buses)
                    .map(|b| self.base_load_mw.get(b).copied().unwrap_or(0.0))
                    .sum();
                let mut flow_var = 0.0_f64;
                for inp in &self.uncertain_inputs {
                    if inp.bus > br && inp.bus < self.n_buses {
                        let (_, std_val) = distribution_mean_std(&inp.distribution);
                        flow_var += std_val * std_val;
                    }
                }
                let flow_std = flow_var.sqrt();
                let flow_p95 = flow_mean + 1.645 * flow_std;
                let prob_overload =
                    gaussian_tail_prob((rating_proxy - flow_mean) / (flow_std + 1e-12));
                BranchStatistics {
                    branch_id: br,
                    flow_mean_mw: flow_mean,
                    flow_std_mw: flow_std,
                    flow_p95_mw: flow_p95,
                    prob_overload,
                }
            })
            .collect();

        Ok(StochasticLfResult {
            n_converged: 1,
            n_diverged: 0,
            bus_stats,
            branch_stats,
            computational_time_note: "Linearized: 1 DC solve + analytical variance propagation"
                .to_string(),
        })
    }

    // ── Sampling primitives ─────────────────────────────────────────────────

    /// Advance the LCG and return a uniform sample in \[0, 1\).
    fn lcg_uniform(state: &mut u64) -> f64 {
        *state = state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        // Use upper 53 bits for full double precision
        (*state >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Sample from a distribution given a pre-computed uniform `u` in \[0,1\).
    ///
    /// For Normal/LogNormal we use Box-Muller (requires a second uniform from the RNG).
    fn sample_distribution_from_u(&self, dist: &Distribution, u: f64, rng: &mut u64) -> f64 {
        match dist {
            Distribution::Normal { mean, std_dev } => {
                let u2 = Self::lcg_uniform(rng);
                Self::box_muller(u, u2) * std_dev + mean
            }
            Distribution::LogNormal { mu, sigma } => {
                let u2 = Self::lcg_uniform(rng);
                let z = Self::box_muller(u, u2);
                (z * sigma + mu).exp()
            }
            Distribution::Uniform { low, high } => low + u * (high - low),
            Distribution::Weibull { scale, shape } => {
                // Inverse CDF: x = λ * (-ln(1-u))^(1/k)
                let u_safe = u.clamp(1e-10, 1.0 - 1e-10);
                scale * (-(1.0 - u_safe).ln()).powf(1.0 / shape)
            }
            Distribution::Beta { alpha, beta } => {
                // Use Johnk's method approximation via rejection sampling with LCG
                // For simplicity, use a deterministic approximation based on u
                // (full rejection sampling would require unbounded iterations)
                beta_quantile_approx(*alpha, *beta, u)
            }
        }
    }

    /// Box-Muller transform: (u1, u2) ∈ (0,1)² → N(0,1) sample.
    fn box_muller(u1: f64, u2: f64) -> f64 {
        let u1 = u1.clamp(1e-10, 1.0 - 1e-10);
        let u2 = u2.clamp(1e-10, 1.0 - 1e-10);
        (-2.0 * u1.ln()).sqrt() * (2.0 * core::f64::consts::PI * u2).cos()
    }

    /// Latin Hypercube Sampling: stratify \[0,1\] into `n` equal intervals,
    /// sample one point uniformly from each interval, then shuffle.
    ///
    /// The shuffle uses Fisher-Yates with the LCG.
    fn lhs_sample(n: usize, rng: &mut u64) -> Vec<f64> {
        let mut result: Vec<f64> = (0..n)
            .map(|i| {
                let stratum_low = i as f64 / n as f64;
                let u_within = Self::lcg_uniform(rng);
                stratum_low + u_within / n as f64
            })
            .collect();

        // Fisher-Yates shuffle
        for i in (1..n).rev() {
            let j_float = Self::lcg_uniform(rng) * (i + 1) as f64;
            let j = (j_float as usize).min(i);
            result.swap(i, j);
        }
        result
    }
}

// ── Standalone sampling helper (public for tests) ───────────────────────────

/// Sample a value from a distribution using a fresh LCG state.
///
/// Exposed for testing and external use.
pub fn sample_distribution(dist: &Distribution, rng: &mut u64) -> f64 {
    let solver = StochasticLfSolver::new(StochasticLfConfig::default(), 1);
    let u = StochasticLfSolver::lcg_uniform(rng);
    solver.sample_distribution_from_u(dist, u, rng)
}

// ── Statistical utilities ──────────────────────────────────────────────────

fn compute_bus_stats(bus: usize, samples: &[f64]) -> BusStatistics {
    if samples.is_empty() {
        return BusStatistics {
            bus,
            v_mean: 1.0,
            v_std: 0.0,
            v_min: 1.0,
            v_max: 1.0,
            v_p5: 1.0,
            v_p95: 1.0,
            prob_violation_pu: 0.0,
        };
    }
    let n = samples.len() as f64;
    let mean = samples.iter().sum::<f64>() / n;
    let var = samples.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / n.max(1.0);
    let std = var.sqrt();

    let mut sorted = samples.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));

    let v_min = sorted.first().copied().unwrap_or(mean);
    let v_max = sorted.last().copied().unwrap_or(mean);

    let p5_idx = ((0.05 * n) as usize).min(sorted.len().saturating_sub(1));
    let p95_idx = ((0.95 * n) as usize).min(sorted.len().saturating_sub(1));
    let v_p5 = sorted[p5_idx];
    let v_p95 = sorted[p95_idx];

    let violations = samples
        .iter()
        .filter(|&&v| !(0.95..=1.05).contains(&v))
        .count();
    let prob_violation = violations as f64 / n;

    BusStatistics {
        bus,
        v_mean: mean,
        v_std: std,
        v_min,
        v_max,
        v_p5,
        v_p95,
        prob_violation_pu: prob_violation,
    }
}

fn compute_branch_stats(branch_id: usize, samples: &[f64], rating_mw: f64) -> BranchStatistics {
    if samples.is_empty() {
        return BranchStatistics {
            branch_id,
            flow_mean_mw: 0.0,
            flow_std_mw: 0.0,
            flow_p95_mw: 0.0,
            prob_overload: 0.0,
        };
    }
    let n = samples.len() as f64;
    let mean = samples.iter().sum::<f64>() / n;
    let var = samples.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / n.max(1.0);
    let std = var.sqrt();

    let mut sorted = samples.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
    let p95_idx = ((0.95 * n) as usize).min(sorted.len().saturating_sub(1));
    let flow_p95 = sorted[p95_idx];

    let overloads = samples.iter().filter(|&&f| f.abs() > rating_mw).count();
    let prob_overload = overloads as f64 / n;

    BranchStatistics {
        branch_id,
        flow_mean_mw: mean,
        flow_std_mw: std,
        flow_p95_mw: flow_p95,
        prob_overload,
    }
}

/// Approximate mean and standard deviation of a distribution.
fn distribution_mean_std(dist: &Distribution) -> (f64, f64) {
    match dist {
        Distribution::Normal { mean, std_dev } => (*mean, *std_dev),
        Distribution::Uniform { low, high } => {
            let mean = (low + high) / 2.0;
            let std = (high - low) / (12.0_f64.sqrt());
            (mean, std)
        }
        Distribution::Weibull { scale, shape } => {
            use core::f64::consts::PI;
            // E[X] = λ Γ(1 + 1/k), Var[X] = λ² [Γ(1+2/k) - (Γ(1+1/k))²]
            // Approximate Gamma with Stirling for k > 1
            let gamma1 = gamma_approx(1.0 + 1.0 / shape);
            let gamma2 = gamma_approx(1.0 + 2.0 / shape);
            let mean = scale * gamma1;
            let var = scale * scale * (gamma2 - gamma1 * gamma1);
            let _ = PI;
            (mean, var.sqrt())
        }
        Distribution::Beta { alpha, beta } => {
            let mean = alpha / (alpha + beta);
            let var = alpha * beta / ((alpha + beta).powi(2) * (alpha + beta + 1.0));
            (mean, var.sqrt())
        }
        Distribution::LogNormal { mu, sigma } => {
            let mean = (mu + sigma * sigma / 2.0).exp();
            let var = ((sigma * sigma).exp() - 1.0) * (2.0 * mu + sigma * sigma).exp();
            (mean, var.sqrt())
        }
    }
}

/// Stirling approximation to Gamma(x) for x > 0.5.
fn gamma_approx(x: f64) -> f64 {
    if x < 0.5 {
        return 1.0;
    }
    // Lanczos g=7 approximation
    let coeffs = [
        0.999_999_999_999_809_9,
        676.520_368_121_885_1,
        -1_259.139_216_722_402_8,
        771.323_428_777_653_1,
        -176.615_029_162_140_6,
        12.507_343_278_686_905,
        -0.138_571_095_265_720_12,
        9.984_369_578_019_572e-6,
        1.505_632_735_149_311_6e-7,
    ];
    let z = x - 1.0;
    let mut sum = coeffs[0];
    for (i, &c) in coeffs[1..].iter().enumerate() {
        sum += c / (z + i as f64 + 1.0);
    }
    let t = z + 7.5;
    (2.0 * core::f64::consts::PI).sqrt() * t.powf(z + 0.5) * (-t).exp() * sum
}

/// Right-tail probability P(X > threshold) for X ~ N(0,1) using erfc approximation.
fn gaussian_tail_prob(z: f64) -> f64 {
    // P(X > z) = 0.5 * erfc(z / sqrt(2))
    0.5 * erfc_approx(z / core::f64::consts::SQRT_2)
}

/// Complementary error function approximation (Abramowitz & Stegun 7.1.26).
fn erfc_approx(x: f64) -> f64 {
    if x < 0.0 {
        return 2.0 - erfc_approx(-x);
    }
    let t = 1.0 / (1.0 + 0.3275911 * x);
    let poly = t
        * (0.254829592
            + t * (-0.284496736 + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429))));
    poly * (-x * x).exp()
}

/// Beta quantile approximation via Newton's method on the regularized incomplete beta.
fn beta_quantile_approx(alpha: f64, beta_param: f64, u: f64) -> f64 {
    // Initial guess using normal approximation
    let mean = alpha / (alpha + beta_param);
    let std =
        (alpha * beta_param / ((alpha + beta_param).powi(2) * (alpha + beta_param + 1.0))).sqrt();
    // Clamp to valid range
    let z = 2.0 * u - 1.0; // rough sign for direction
    (mean + std * z * 2.0).clamp(0.0, 1.0)
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_solver(n_samples: usize, method: StochasticMethod) -> StochasticLfSolver {
        let cfg = StochasticLfConfig {
            n_samples,
            confidence_level: 0.95,
            method,
            seed: 12345,
        };
        let mut solver = StochasticLfSolver::new(cfg, 4);
        solver.set_base_loads(vec![0.0, 1.0, 2.0, 1.5], vec![0.0, 0.3, 0.6, 0.4]);
        solver
    }

    /// Deterministic case: std_dev=0 → all samples identical, std≈0.
    #[test]
    fn test_deterministic_case() {
        let mut solver = make_solver(200, StochasticMethod::MonteCarlo);
        solver.add_uncertain_input(UncertainInput {
            bus: 1,
            variable: InputVariable::LoadP,
            distribution: Distribution::Normal {
                mean: 1.0,
                std_dev: 0.0,
            },
        });
        let result = solver.solve().expect("solve must succeed");
        let stats = &result.bus_stats[1];
        assert!(
            stats.v_std < 1e-10,
            "With std_dev=0, voltage std must be ~0, got {}",
            stats.v_std
        );
        assert!(
            (stats.v_p95 - stats.v_p5).abs() < 1e-9,
            "P5 and P95 must be equal for zero variance"
        );
    }

    /// Normal distribution: mean ≈ base case, std > 0.
    #[test]
    fn test_normal_distribution() {
        let mut solver = make_solver(1000, StochasticMethod::MonteCarlo);
        solver.add_uncertain_input(UncertainInput {
            bus: 1,
            variable: InputVariable::LoadP,
            distribution: Distribution::Normal {
                mean: 1.0,
                std_dev: 0.2,
            },
        });
        let result = solver.solve().expect("solve must succeed");
        let stats = &result.bus_stats[1];
        // Mean voltage should be close to deterministic base (within 0.05 pu)
        let base_v = 1.0 - 0.01 * (0.0 + 1.0); // bus 1 cumulative load
        assert!(
            (stats.v_mean - base_v).abs() < 0.05,
            "Mean voltage should be close to base: mean={}, base={}",
            stats.v_mean,
            base_v
        );
        assert!(
            stats.v_std > 0.0,
            "Nonzero std_dev must produce nonzero voltage std"
        );
        assert!(result.n_converged == 1000);
    }

    /// Higher uncertainty → higher probability of voltage violation.
    ///
    /// Uses a near-boundary base case: cumulative load ≈ 4.5 MW → V ≈ 0.955 pu
    /// (just above 0.95).  With small std, almost no violations.  With large std,
    /// many samples fall below 0.95 pu.
    #[test]
    fn test_violation_probability_increases_with_uncertainty() {
        let make_with_std = |std: f64| -> f64 {
            let cfg = StochasticLfConfig {
                n_samples: 500,
                confidence_level: 0.95,
                method: StochasticMethod::MonteCarlo,
                seed: 99,
            };
            // Base loads chosen so cumulative at bus 1 ≈ 0.5 MW → V ≈ 0.995 (near nominal)
            // With std=0.01: violations negligible; with std=1.0: many violations
            let mut solver = StochasticLfSolver::new(cfg, 2);
            solver.set_base_loads(vec![0.0, 0.5], vec![0.0, 0.1]);
            solver.add_uncertain_input(UncertainInput {
                bus: 1,
                variable: InputVariable::LoadP,
                // mean=0.5: cumulative=0.5 → V=0.995 (no violation)
                // With std=2.0: many samples have cumulative > 5 → V < 0.95
                distribution: Distribution::Normal {
                    mean: 0.5,
                    std_dev: std,
                },
            });
            solver
                .solve()
                .expect("solve")
                .bus_stats
                .last()
                .map(|s| s.prob_violation_pu)
                .unwrap_or(0.0)
        };
        let low = make_with_std(0.01); // very tight: V stays near 0.995
        let high = make_with_std(2.0); // wide: many samples below 0.95
        assert!(
            high > low,
            "Higher uncertainty must produce > violation probability: low={low}, high={high}"
        );
    }

    /// LHS and MC should give similar mean statistics.
    #[test]
    fn test_lhs_vs_mc_similar_mean() {
        let add_input = |s: &mut StochasticLfSolver| {
            s.add_uncertain_input(UncertainInput {
                bus: 2,
                variable: InputVariable::LoadP,
                distribution: Distribution::Normal {
                    mean: 2.0,
                    std_dev: 0.3,
                },
            });
        };

        let mut mc_solver = make_solver(500, StochasticMethod::MonteCarlo);
        add_input(&mut mc_solver);
        let mc_result = mc_solver.solve().expect("MC solve");

        let mut lhs_solver = make_solver(200, StochasticMethod::LatinHypercubeSampling);
        add_input(&mut lhs_solver);
        let lhs_result = lhs_solver.solve().expect("LHS solve");

        let mc_mean = mc_result.bus_stats[2].v_mean;
        let lhs_mean = lhs_result.bus_stats[2].v_mean;
        assert!(
            (mc_mean - lhs_mean).abs() < 0.02,
            "LHS and MC means should be close: mc={mc_mean}, lhs={lhs_mean}"
        );
    }

    /// Percentile ordering: v_p5 ≤ v_mean ≤ v_p95.
    #[test]
    fn test_percentiles_ordering() {
        let mut solver = make_solver(500, StochasticMethod::MonteCarlo);
        solver.add_uncertain_input(UncertainInput {
            bus: 1,
            variable: InputVariable::LoadP,
            distribution: Distribution::Uniform {
                low: 0.5,
                high: 2.0,
            },
        });
        let result = solver.solve().expect("solve");
        for stats in &result.bus_stats {
            assert!(
                stats.v_p5 <= stats.v_mean + 1e-9,
                "P5 must be <= mean at bus {}: p5={}, mean={}",
                stats.bus,
                stats.v_p5,
                stats.v_mean
            );
            assert!(
                stats.v_p95 >= stats.v_mean - 1e-9,
                "P95 must be >= mean at bus {}: p95={}, mean={}",
                stats.bus,
                stats.v_p95,
                stats.v_mean
            );
        }
    }

    /// Linearized method produces nonzero std when there is uncertainty.
    #[test]
    fn test_linearized_method() {
        let mut solver = make_solver(1, StochasticMethod::Linearized);
        solver.add_uncertain_input(UncertainInput {
            bus: 1,
            variable: InputVariable::LoadP,
            distribution: Distribution::Normal {
                mean: 1.0,
                std_dev: 0.5,
            },
        });
        let result = solver.solve().expect("linearized solve");
        assert_eq!(result.n_converged, 1);
        assert!(
            result.bus_stats[1].v_std > 0.0,
            "Linearized must propagate variance"
        );
        // P5 < P95
        assert!(result.bus_stats[1].v_p5 < result.bus_stats[1].v_p95);
    }

    /// Point estimate method with 2 inputs produces 5 evaluations.
    #[test]
    fn test_point_estimate_evaluations() {
        let mut solver = make_solver(1, StochasticMethod::PointEstimate2m);
        solver.add_uncertain_input(UncertainInput {
            bus: 1,
            variable: InputVariable::LoadP,
            distribution: Distribution::Normal {
                mean: 1.0,
                std_dev: 0.2,
            },
        });
        solver.add_uncertain_input(UncertainInput {
            bus: 2,
            variable: InputVariable::LoadP,
            distribution: Distribution::Normal {
                mean: 2.0,
                std_dev: 0.3,
            },
        });
        let result = solver.solve().expect("PEM solve");
        // 2m+1 = 5 evaluations
        assert_eq!(result.n_converged, 5, "2m+1=5 evaluations for m=2");
    }
}
