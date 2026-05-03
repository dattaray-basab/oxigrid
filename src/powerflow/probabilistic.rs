//! Probabilistic Power Flow (PPF).
//!
//! Propagates uncertain renewable injections through the DC power-flow equations
//! to produce output distributions for bus voltages and branch power flows.
//!
//! # Methods
//!
//! | Method | Struct/fn | Notes |
//! |--------|-----------|-------|
//! | Monte Carlo | [`run_mc_ppf`] | Full AC/DC solve per sample; exact but slow |
//! | Latin Hypercube Sampling | [`run_lhs_ppf`] | Stratified MC; better coverage per sample |
//! | Point Estimate Method (Hong 2m+1) | [`run_pem_ppf`] | 2m+1 deterministic evaluations |
//! | Linear sensitivity | [`run_linear_ppf`] | Analytical variance propagation, 1 solve |

use crate::error::{OxiGridError, Result};
use crate::network::PowerNetwork;
use crate::powerflow::{PowerFlowConfig, PowerFlowMethod};
use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// Input uncertainty description
// ---------------------------------------------------------------------------

/// Distribution type for a single uncertain input.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum UncertaintyDistribution {
    /// Normal(mean, std_dev)
    Normal { mean: f64, std_dev: f64 },
    /// Uniform(low, high)
    Uniform { low: f64, high: f64 },
    /// Beta(alpha, beta) scaled to [low, high]
    Beta {
        alpha: f64,
        beta: f64,
        low: f64,
        high: f64,
    },
    /// Weibull(scale λ, shape k) — common for wind speed
    Weibull { scale: f64, shape: f64 },
    /// Lognormal(mu, sigma) — μ, σ of the underlying normal
    Lognormal { mu: f64, sigma: f64 },
}

impl UncertaintyDistribution {
    /// Mean of the distribution.
    pub fn mean(&self) -> f64 {
        match self {
            Self::Normal { mean, .. } => *mean,
            Self::Uniform { low, high } => (low + high) / 2.0,
            Self::Beta {
                alpha,
                beta,
                low,
                high,
            } => {
                let mu = alpha / (alpha + beta);
                low + mu * (high - low)
            }
            Self::Weibull { scale, shape } => scale * gamma_fn(1.0 + 1.0 / shape),
            Self::Lognormal { mu, sigma } => (mu + sigma * sigma / 2.0).exp(),
        }
    }

    /// Variance of the distribution.
    pub fn variance(&self) -> f64 {
        match self {
            Self::Normal { std_dev, .. } => std_dev * std_dev,
            Self::Uniform { low, high } => {
                let w = high - low;
                w * w / 12.0
            }
            Self::Beta {
                alpha,
                beta,
                low,
                high,
            } => {
                let ab = alpha + beta;
                let v = alpha * beta / (ab * ab * (ab + 1.0));
                let w = high - low;
                v * w * w
            }
            Self::Weibull { scale, shape } => {
                let g1 = gamma_fn(1.0 + 2.0 / shape);
                let g2 = gamma_fn(1.0 + 1.0 / shape);
                scale * scale * (g1 - g2 * g2)
            }
            Self::Lognormal { mu, sigma } => {
                let s2 = sigma * sigma;
                (s2.exp() - 1.0) * (2.0 * mu + s2).exp()
            }
        }
    }

    /// Standard deviation.
    pub fn std_dev(&self) -> f64 {
        self.variance().sqrt()
    }

    /// Inverse CDF (quantile function) at probability p ∈ (0,1).
    pub fn icdf(&self, p: f64) -> f64 {
        let p = p.clamp(1e-9, 1.0 - 1e-9);
        match self {
            Self::Normal { mean, std_dev } => mean + std_dev * normal_icdf(p),
            Self::Uniform { low, high } => low + p * (high - low),
            Self::Beta {
                alpha,
                beta,
                low,
                high,
            } => {
                let u = beta_icdf(p, *alpha, *beta);
                low + u * (high - low)
            }
            Self::Weibull { scale, shape } => scale * (-((1.0 - p).ln())).powf(1.0 / shape),
            Self::Lognormal { mu, sigma } => (mu + sigma * normal_icdf(p)).exp(),
        }
    }

    /// Central moments: (mean, variance, skewness, excess-kurtosis).
    pub fn moments(&self) -> (f64, f64, f64, f64) {
        let mu = self.mean();
        let var = self.variance();
        match self {
            Self::Normal { .. } => (mu, var, 0.0, 0.0),
            Self::Uniform { .. } => (mu, var, 0.0, -6.0 / 5.0),
            Self::Beta { alpha, beta, .. } => {
                let ab = alpha + beta;
                let skew =
                    2.0 * (beta - alpha) * (ab + 1.0).sqrt() / ((ab + 2.0) * (alpha * beta).sqrt());
                let kurt = 6.0
                    * (alpha.powi(3) + beta.powi(3) + alpha * beta * (alpha + beta - 6.0))
                    / (alpha * beta * (ab + 2.0) * (ab + 3.0));
                (mu, var, skew, kurt)
            }
            Self::Weibull { shape, .. } => {
                let k = *shape;
                let g1 = gamma_fn(1.0 + 1.0 / k);
                let g2 = gamma_fn(1.0 + 2.0 / k);
                let g3 = gamma_fn(1.0 + 3.0 / k);
                let g4 = gamma_fn(1.0 + 4.0 / k);
                let v = var.max(1e-30);
                let skew = (g3 - 3.0 * g2 * g1 + 2.0 * g1.powi(3)) / v.powf(1.5);
                let kurt =
                    (g4 - 4.0 * g3 * g1 + 6.0 * g2 * g1.powi(2) - 3.0 * g1.powi(4)) / (v * v) - 3.0;
                (mu, var, skew, kurt)
            }
            Self::Lognormal { sigma, .. } => {
                let s2 = sigma * sigma;
                let skew = (s2.exp() + 2.0) * (s2.exp() - 1.0).sqrt();
                let kurt = s2.exp().powi(4) + 2.0 * s2.exp().powi(3) + 3.0 * s2.exp().powi(2) - 6.0;
                (mu, var, skew, kurt)
            }
        }
    }
}

/// One uncertain bus injection (active power perturbation around a base case).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UncertainInjection {
    /// Bus ID (matches `Bus::id` in `PowerNetwork`).
    pub bus_id: usize,
    /// Distribution of ΔP injection (MW). Positive = more generation (reduces net load).
    pub delta_p: UncertaintyDistribution,
    /// Optional correlated ΔQ (MVAr).
    pub delta_q: Option<UncertaintyDistribution>,
}

// ---------------------------------------------------------------------------
// Output statistics
// ---------------------------------------------------------------------------

/// Summary statistics for a single scalar output across all PPF samples.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OutputStats {
    /// Number of valid samples.
    pub n_samples: usize,
    pub mean: f64,
    pub std_dev: f64,
    /// 5th percentile (P05).
    pub p05: f64,
    /// 25th percentile (P25).
    pub p25: f64,
    /// Median (P50).
    pub p50: f64,
    /// 75th percentile (P75).
    pub p75: f64,
    /// 95th percentile (P95).
    pub p95: f64,
    /// 99th percentile (P99) — value-at-risk proxy.
    pub p99: f64,
    /// Minimum observed sample.
    pub min: f64,
    /// Maximum observed sample.
    pub max: f64,
}

impl OutputStats {
    /// Build from a vector of samples (need not be sorted).
    pub fn from_samples(mut samples: Vec<f64>) -> Self {
        assert!(!samples.is_empty(), "OutputStats: empty sample set");
        let n = samples.len();
        samples.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let mean = samples.iter().sum::<f64>() / n as f64;
        let var = samples.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / n as f64;
        let std_dev = var.sqrt();
        let pct = |p: f64| -> f64 {
            let idx = (p * (n - 1) as f64).round() as usize;
            samples[idx.min(n - 1)]
        };
        Self {
            n_samples: n,
            mean,
            std_dev,
            p05: pct(0.05),
            p25: pct(0.25),
            p50: pct(0.50),
            p75: pct(0.75),
            p95: pct(0.95),
            p99: pct(0.99),
            min: *samples.first().unwrap(),
            max: *samples.last().unwrap(),
        }
    }

    /// Probability that output exceeds a threshold (Gaussian approximation).
    pub fn prob_exceed(&self, threshold: f64) -> f64 {
        if self.std_dev < 1e-12 {
            return if self.mean > threshold { 1.0 } else { 0.0 };
        }
        let z = (threshold - self.mean) / self.std_dev;
        1.0 - normal_cdf(z)
    }

    /// Coefficient of variation (σ/μ).
    pub fn cv(&self) -> f64 {
        self.std_dev / self.mean
    }
}

// ---------------------------------------------------------------------------
// PPF result container
// ---------------------------------------------------------------------------

/// Output of a probabilistic power flow run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PpfResult {
    pub method: PpfMethod,
    pub n_samples: usize,
    /// Per-bus voltage magnitude statistics (indexed by bus position in network).
    pub voltage_stats: Vec<OutputStats>,
    /// Per-bus active injection statistics.
    pub p_injection_stats: Vec<OutputStats>,
    /// Per-branch active power flow statistics.
    pub branch_flow_stats: Vec<OutputStats>,
    /// Number of samples that failed to converge (MC/LHS only).
    pub n_diverged: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum PpfMethod {
    MonteCarlo,
    LatinHypercube,
    PointEstimate,
    LinearSensitivity,
}

// ---------------------------------------------------------------------------
// PPF configuration
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PpfConfig {
    pub method: PpfMethod,
    /// Number of MC/LHS samples (ignored for PEM and linear).
    pub n_samples: usize,
    /// Random seed for reproducibility.
    pub seed: u64,
    /// Underlying power-flow solver config.
    pub pf_config: PowerFlowConfig,
}

impl Default for PpfConfig {
    fn default() -> Self {
        Self {
            method: PpfMethod::MonteCarlo,
            n_samples: 500,
            seed: 42,
            pf_config: PowerFlowConfig {
                method: PowerFlowMethod::DcApproximation,
                max_iter: 50,
                tolerance: 1e-6,
                enforce_q_limits: false,
            },
        }
    }
}

// ---------------------------------------------------------------------------
// Monte Carlo PPF
// ---------------------------------------------------------------------------

/// Run a Monte Carlo probabilistic power flow.
///
/// For each sample: perturb injections → solve power flow → record outputs.
#[allow(clippy::needless_range_loop)]
pub fn run_mc_ppf(
    network: &PowerNetwork,
    injections: &[UncertainInjection],
    config: &PpfConfig,
) -> Result<PpfResult> {
    let n_buses = network.buses.len();
    let n_branches = network.branches.len();
    let n = config.n_samples;

    let mut v_samples: Vec<Vec<f64>> = vec![Vec::with_capacity(n); n_buses];
    let mut p_samples: Vec<Vec<f64>> = vec![Vec::with_capacity(n); n_buses];
    let mut f_samples: Vec<Vec<f64>> = vec![Vec::with_capacity(n); n_branches];
    let mut n_diverged = 0usize;

    let mut rng = Lcg64::new(config.seed);

    for _ in 0..n {
        let mut net_copy = network.clone();
        for inj in injections {
            let delta_p = inj.delta_p.icdf(rng.next_f64());
            if let Some(bus) = net_copy.buses.iter_mut().find(|b| b.id == inj.bus_id) {
                bus.pd = crate::units::Power(bus.pd.0 - delta_p);
                if let Some(dq_dist) = &inj.delta_q {
                    let delta_q = dq_dist.icdf(rng.next_f64());
                    bus.qd = crate::units::ReactivePower(bus.qd.0 - delta_q);
                }
            }
        }
        match net_copy.solve_powerflow(&config.pf_config) {
            Ok(res) if res.converged => {
                for (i, &v) in res.voltage_magnitude.iter().enumerate() {
                    if i < n_buses {
                        v_samples[i].push(v);
                    }
                }
                for (i, &p) in res.p_injected.iter().enumerate() {
                    if i < n_buses {
                        p_samples[i].push(p);
                    }
                }
                for (i, bf) in res.branch_flows.iter().enumerate() {
                    if i < n_branches {
                        f_samples[i].push(bf.p_from_mw);
                    }
                }
            }
            _ => n_diverged += 1,
        }
    }

    let voltage_stats = v_samples
        .into_iter()
        .map(|s| {
            if s.is_empty() {
                dummy_stats()
            } else {
                OutputStats::from_samples(s)
            }
        })
        .collect();
    let p_injection_stats = p_samples
        .into_iter()
        .map(|s| {
            if s.is_empty() {
                dummy_stats()
            } else {
                OutputStats::from_samples(s)
            }
        })
        .collect();
    let branch_flow_stats = f_samples
        .into_iter()
        .map(|s| {
            if s.is_empty() {
                dummy_stats()
            } else {
                OutputStats::from_samples(s)
            }
        })
        .collect();

    Ok(PpfResult {
        method: PpfMethod::MonteCarlo,
        n_samples: n,
        voltage_stats,
        p_injection_stats,
        branch_flow_stats,
        n_diverged,
    })
}

// ---------------------------------------------------------------------------
// Latin Hypercube Sampling PPF
// ---------------------------------------------------------------------------

/// Run a Latin Hypercube Sampling probabilistic power flow.
///
/// Stratifies each input dimension into `n_samples` equal-probability bins,
/// randomly samples one value per stratum, and shuffles across dimensions.
#[allow(clippy::needless_range_loop)]
pub fn run_lhs_ppf(
    network: &PowerNetwork,
    injections: &[UncertainInjection],
    config: &PpfConfig,
) -> Result<PpfResult> {
    let n = config.n_samples;
    let m = injections.len();
    let mut rng = Lcg64::new(config.seed);

    let lhs_matrix = lhs_sample(m, n, &mut rng);

    let samples_per_inj: Vec<Vec<f64>> = injections
        .iter()
        .zip(lhs_matrix.iter())
        .map(|(inj, u_row)| u_row.iter().map(|&u| inj.delta_p.icdf(u)).collect())
        .collect();

    let n_buses = network.buses.len();
    let n_branches = network.branches.len();
    let mut v_samples: Vec<Vec<f64>> = vec![Vec::with_capacity(n); n_buses];
    let mut p_samples: Vec<Vec<f64>> = vec![Vec::with_capacity(n); n_buses];
    let mut f_samples: Vec<Vec<f64>> = vec![Vec::with_capacity(n); n_branches];
    let mut n_diverged = 0usize;

    for s_idx in 0..n {
        let mut net_copy = network.clone();
        for (i, inj) in injections.iter().enumerate() {
            let delta_p = samples_per_inj[i][s_idx];
            if let Some(bus) = net_copy.buses.iter_mut().find(|b| b.id == inj.bus_id) {
                bus.pd = crate::units::Power(bus.pd.0 - delta_p);
            }
        }
        match net_copy.solve_powerflow(&config.pf_config) {
            Ok(res) if res.converged => {
                for (i, &v) in res.voltage_magnitude.iter().enumerate() {
                    if i < n_buses {
                        v_samples[i].push(v);
                    }
                }
                for (i, &p) in res.p_injected.iter().enumerate() {
                    if i < n_buses {
                        p_samples[i].push(p);
                    }
                }
                for (i, bf) in res.branch_flows.iter().enumerate() {
                    if i < n_branches {
                        f_samples[i].push(bf.p_from_mw);
                    }
                }
            }
            _ => n_diverged += 1,
        }
    }

    let voltage_stats = v_samples
        .into_iter()
        .map(|s| {
            if s.is_empty() {
                dummy_stats()
            } else {
                OutputStats::from_samples(s)
            }
        })
        .collect();
    let p_injection_stats = p_samples
        .into_iter()
        .map(|s| {
            if s.is_empty() {
                dummy_stats()
            } else {
                OutputStats::from_samples(s)
            }
        })
        .collect();
    let branch_flow_stats = f_samples
        .into_iter()
        .map(|s| {
            if s.is_empty() {
                dummy_stats()
            } else {
                OutputStats::from_samples(s)
            }
        })
        .collect();

    Ok(PpfResult {
        method: PpfMethod::LatinHypercube,
        n_samples: n,
        voltage_stats,
        p_injection_stats,
        branch_flow_stats,
        n_diverged,
    })
}

// ---------------------------------------------------------------------------
// Point Estimate Method (Hong 2m+1 scheme)
// ---------------------------------------------------------------------------

/// Run a Point Estimate Method (PEM) probabilistic power flow.
///
/// Uses Hong's 2m+1 scheme: for each of m uncertain inputs, evaluate at
/// mean ± k·σ concentration points, plus the all-mean base case.
/// Total = 2m+1 deterministic power flows.
pub fn run_pem_ppf(
    network: &PowerNetwork,
    injections: &[UncertainInjection],
    config: &PpfConfig,
) -> Result<PpfResult> {
    let m = injections.len();
    let n_buses = network.buses.len();
    let n_branches = network.branches.len();
    let n_evaluations = 2 * m + 1;

    let mut v_wsum = vec![0.0f64; n_buses];
    let mut v_wxsum = vec![0.0f64; n_buses];
    let mut v_wx2sum = vec![0.0f64; n_buses];
    let mut p_wsum = vec![0.0f64; n_buses];
    let mut p_wxsum = vec![0.0f64; n_buses];
    let mut p_wx2sum = vec![0.0f64; n_buses];
    let mut f_wsum = vec![0.0f64; n_branches];
    let mut f_wxsum = vec![0.0f64; n_branches];
    let mut f_wx2sum = vec![0.0f64; n_branches];
    let mut n_diverged = 0usize;

    let w_pair = 1.0 / (2.0 * m.max(1) as f64);
    let w_base = if m > 0 {
        (m as f64 - 1.0) / m as f64
    } else {
        1.0
    };

    // Base case (all-mean)
    let base_result = evaluate_pem_point(network, injections, &[], config)?;
    if base_result.converged {
        accumulate_weighted(
            &base_result,
            w_base,
            n_buses,
            n_branches,
            &mut v_wsum,
            &mut v_wxsum,
            &mut v_wx2sum,
            &mut p_wsum,
            &mut p_wxsum,
            &mut p_wx2sum,
            &mut f_wsum,
            &mut f_wxsum,
            &mut f_wx2sum,
        );
    } else {
        n_diverged += 1;
    }

    // 2m concentration points
    for (k, inj) in injections.iter().enumerate() {
        let (mu, var, skew, _kurt) = inj.delta_p.moments();
        let sigma = var.sqrt().max(1e-9);
        let lambda = skew / 2.0;
        let xi_plus = lambda + (m as f64 - lambda * lambda).max(0.0).sqrt();
        let xi_minus = lambda - (m as f64 - lambda * lambda).max(0.0).sqrt();

        for xi in [xi_plus, xi_minus] {
            let delta = mu + xi * sigma;
            let perturbation = vec![(k, delta)];
            match evaluate_pem_point(network, injections, &perturbation, config) {
                Ok(res) if res.converged => {
                    accumulate_weighted(
                        &res,
                        w_pair,
                        n_buses,
                        n_branches,
                        &mut v_wsum,
                        &mut v_wxsum,
                        &mut v_wx2sum,
                        &mut p_wsum,
                        &mut p_wxsum,
                        &mut p_wx2sum,
                        &mut f_wsum,
                        &mut f_wxsum,
                        &mut f_wx2sum,
                    );
                }
                _ => n_diverged += 1,
            }
        }
    }

    let stats_from_moments = |wsum: &[f64], wxsum: &[f64], wx2sum: &[f64]| -> Vec<OutputStats> {
        wsum.iter()
            .zip(wxsum)
            .zip(wx2sum)
            .map(|((&w, &wx), &wx2)| {
                let mean = if w > 1e-12 { wx / w } else { 0.0 };
                let e_x2 = if w > 1e-12 { wx2 / w } else { 0.0 };
                let var = (e_x2 - mean * mean).max(0.0);
                let std_dev = var.sqrt();
                OutputStats {
                    n_samples: n_evaluations,
                    mean,
                    std_dev,
                    p05: mean + std_dev * normal_icdf(0.05),
                    p25: mean + std_dev * normal_icdf(0.25),
                    p50: mean,
                    p75: mean + std_dev * normal_icdf(0.75),
                    p95: mean + std_dev * normal_icdf(0.95),
                    p99: mean + std_dev * normal_icdf(0.99),
                    min: mean - 4.0 * std_dev,
                    max: mean + 4.0 * std_dev,
                }
            })
            .collect()
    };

    Ok(PpfResult {
        method: PpfMethod::PointEstimate,
        n_samples: n_evaluations,
        voltage_stats: stats_from_moments(&v_wsum, &v_wxsum, &v_wx2sum),
        p_injection_stats: stats_from_moments(&p_wsum, &p_wxsum, &p_wx2sum),
        branch_flow_stats: stats_from_moments(&f_wsum, &f_wxsum, &f_wx2sum),
        n_diverged,
    })
}

// ---------------------------------------------------------------------------
// Sensitivity-based linear PPF
// ---------------------------------------------------------------------------

/// Linear (sensitivity-based) PPF.
///
/// Uses numerical sensitivity ∂V/∂P and ∂F/∂P via one perturbed solve per input.
/// Variances propagate analytically: `Var[y]` = Σ_k (∂y/∂P_k)² · `Var[ΔP_k]`.
pub fn run_linear_ppf(
    network: &PowerNetwork,
    injections: &[UncertainInjection],
    config: &PpfConfig,
) -> Result<PpfResult> {
    let n_buses = network.buses.len();
    let n_branches = network.branches.len();
    let eps = 1.0f64; // 1 MW perturbation

    let base_result = network.solve_powerflow(&config.pf_config)?;
    if !base_result.converged {
        return Err(OxiGridError::Convergence {
            iterations: config.pf_config.max_iter,
            residual: f64::INFINITY,
        });
    }

    let mut v_sensitivity: Vec<Vec<f64>> = Vec::new();
    let mut f_sensitivity: Vec<Vec<f64>> = Vec::new();

    for inj in injections {
        let mut net_p = network.clone();
        if let Some(bus) = net_p.buses.iter_mut().find(|b| b.id == inj.bus_id) {
            bus.pd = crate::units::Power(bus.pd.0 - eps);
        }
        let res_p = net_p.solve_powerflow(&config.pf_config)?;

        let dv: Vec<f64> = res_p
            .voltage_magnitude
            .iter()
            .zip(&base_result.voltage_magnitude)
            .map(|(&vp, &vb)| (vp - vb) / eps)
            .collect();
        v_sensitivity.push(dv);

        let df: Vec<f64> = res_p
            .branch_flows
            .iter()
            .zip(&base_result.branch_flows)
            .map(|(fp, fb)| (fp.p_from_mw - fb.p_from_mw) / eps)
            .collect();
        f_sensitivity.push(df);
    }

    let mut v_var = vec![0.0f64; n_buses];
    let mut f_var = vec![0.0f64; n_branches];
    for (k, inj) in injections.iter().enumerate() {
        let var_k = inj.delta_p.variance();
        for (i, vv) in v_var.iter_mut().enumerate() {
            if i < v_sensitivity[k].len() {
                *vv += v_sensitivity[k][i].powi(2) * var_k;
            }
        }
        for (i, fv) in f_var.iter_mut().enumerate() {
            if i < f_sensitivity[k].len() {
                *fv += f_sensitivity[k][i].powi(2) * var_k;
            }
        }
    }

    let make_stats = |means: &[f64], variances: &[f64]| -> Vec<OutputStats> {
        means
            .iter()
            .zip(variances)
            .map(|(&mu, &var)| {
                let std_dev = var.sqrt();
                OutputStats {
                    n_samples: injections.len() + 1,
                    mean: mu,
                    std_dev,
                    p05: mu + std_dev * normal_icdf(0.05),
                    p25: mu + std_dev * normal_icdf(0.25),
                    p50: mu,
                    p75: mu + std_dev * normal_icdf(0.75),
                    p95: mu + std_dev * normal_icdf(0.95),
                    p99: mu + std_dev * normal_icdf(0.99),
                    min: mu - 4.0 * std_dev,
                    max: mu + 4.0 * std_dev,
                }
            })
            .collect()
    };

    let f_means: Vec<f64> = base_result
        .branch_flows
        .iter()
        .map(|b| b.p_from_mw)
        .collect();
    let p_vars = vec![0.0f64; n_buses];

    Ok(PpfResult {
        method: PpfMethod::LinearSensitivity,
        n_samples: injections.len() + 1,
        voltage_stats: make_stats(&base_result.voltage_magnitude, &v_var),
        p_injection_stats: make_stats(&base_result.p_injected, &p_vars),
        branch_flow_stats: make_stats(&f_means, &f_var),
        n_diverged: 0,
    })
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

#[allow(clippy::too_many_arguments)]
fn accumulate_weighted(
    res: &crate::powerflow::PowerFlowResult,
    w: f64,
    n_buses: usize,
    n_branches: usize,
    v_wsum: &mut [f64],
    v_wxsum: &mut [f64],
    v_wx2sum: &mut [f64],
    p_wsum: &mut [f64],
    p_wxsum: &mut [f64],
    p_wx2sum: &mut [f64],
    f_wsum: &mut [f64],
    f_wxsum: &mut [f64],
    f_wx2sum: &mut [f64],
) {
    for (i, &v) in res.voltage_magnitude.iter().enumerate() {
        if i < n_buses {
            v_wsum[i] += w;
            v_wxsum[i] += w * v;
            v_wx2sum[i] += w * v * v;
        }
    }
    for (i, &p) in res.p_injected.iter().enumerate() {
        if i < n_buses {
            p_wsum[i] += w;
            p_wxsum[i] += w * p;
            p_wx2sum[i] += w * p * p;
        }
    }
    for (i, bf) in res.branch_flows.iter().enumerate() {
        if i < n_branches {
            let fp = bf.p_from_mw;
            f_wsum[i] += w;
            f_wxsum[i] += w * fp;
            f_wx2sum[i] += w * fp * fp;
        }
    }
}

fn evaluate_pem_point(
    network: &PowerNetwork,
    injections: &[UncertainInjection],
    perturbations: &[(usize, f64)],
    config: &PpfConfig,
) -> Result<crate::powerflow::PowerFlowResult> {
    let mut net_copy = network.clone();
    for (k, inj) in injections.iter().enumerate() {
        let mu = inj.delta_p.mean();
        let delta = perturbations
            .iter()
            .find(|(idx, _)| *idx == k)
            .map(|(_, v)| *v)
            .unwrap_or(mu);
        if let Some(bus) = net_copy.buses.iter_mut().find(|b| b.id == inj.bus_id) {
            bus.pd = crate::units::Power(bus.pd.0 - delta);
        }
    }
    net_copy.solve_powerflow(&config.pf_config)
}

fn dummy_stats() -> OutputStats {
    OutputStats {
        n_samples: 0,
        mean: 0.0,
        std_dev: 0.0,
        p05: 0.0,
        p25: 0.0,
        p50: 0.0,
        p75: 0.0,
        p95: 0.0,
        p99: 0.0,
        min: 0.0,
        max: 0.0,
    }
}

/// Build a Latin Hypercube sample matrix [m × n] with values in (0,1).
fn lhs_sample(m: usize, n: usize, rng: &mut Lcg64) -> Vec<Vec<f64>> {
    let mut matrix = Vec::with_capacity(m);
    for _ in 0..m {
        let mut row: Vec<f64> = (0..n)
            .map(|j| (j as f64 + rng.next_f64()) / n as f64)
            .collect();
        for i in (1..n).rev() {
            let j = (rng.next_f64() * (i + 1) as f64) as usize;
            row.swap(i, j);
        }
        matrix.push(row);
    }
    matrix
}

// ---------------------------------------------------------------------------
// Math utilities
// ---------------------------------------------------------------------------

/// Linear Congruential Generator (64-bit, Numerical Recipes constants).
struct Lcg64 {
    state: u64,
}

impl Lcg64 {
    fn new(seed: u64) -> Self {
        Self {
            state: seed.wrapping_add(1),
        }
    }
    fn next_u64(&mut self) -> u64 {
        self.state = self
            .state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.state
    }
    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// Rational approximation of normal CDF (Abramowitz & Stegun 26.2.17).
pub fn normal_cdf(x: f64) -> f64 {
    let t = 1.0 / (1.0 + 0.231_641_9 * x.abs());
    let poly = t
        * (0.319_381_530
            + t * (-0.356_563_782
                + t * (1.781_477_937 + t * (-1.821_255_978 + t * 1.330_274_429))));
    let pdf = (-x * x / 2.0).exp() / (2.0 * std::f64::consts::PI).sqrt();
    let cdf = 1.0 - pdf * poly;
    if x >= 0.0 {
        cdf
    } else {
        1.0 - cdf
    }
}

/// Inverse normal CDF via rational approximation (Abramowitz & Stegun / Peter Acklam).
///
/// Accurate to ~1e-9 over (0, 1).
pub fn normal_icdf(p: f64) -> f64 {
    let p = p.clamp(1e-15, 1.0 - 1e-15);

    // Coefficients for the rational approximation (Acklam)
    const A: [f64; 6] = [
        -3.969_683_028_665_376e1,
        2.209_460_984_245_205e2,
        -2.759_285_104_469_687e2,
        1.383_577_518_672_69e2,
        -3.066_479_806_614_716e1,
        2.506_628_277_459_239e0,
    ];
    const B: [f64; 5] = [
        -5.447_609_879_822_406e1,
        1.615_858_368_580_409e2,
        -1.556_989_798_598_866e2,
        6.680_131_188_771_972e1,
        -1.328_068_155_288_572e1,
    ];
    const C: [f64; 6] = [
        -7.784_894_002_430_293e-3,
        -3.223_964_580_411_365e-1,
        -2.400_758_277_161_838e0,
        -2.549_732_539_343_734e0,
        4.374_664_141_464_968e0,
        2.938_163_982_698_783e0,
    ];
    const D: [f64; 4] = [
        7.784_695_709_041_462e-3,
        3.224_671_290_700_398e-1,
        2.445_134_137_142_996e0,
        3.754_408_661_907_416e0,
    ];

    const P_LOW: f64 = 0.02425;
    const P_HIGH: f64 = 1.0 - P_LOW;

    if p < P_LOW {
        // Lower tail
        let q = (-2.0 * p.ln()).sqrt();
        (((((C[0] * q + C[1]) * q + C[2]) * q + C[3]) * q + C[4]) * q + C[5])
            / ((((D[0] * q + D[1]) * q + D[2]) * q + D[3]) * q + 1.0)
    } else if p <= P_HIGH {
        // Central region
        let q = p - 0.5;
        let r = q * q;
        (((((A[0] * r + A[1]) * r + A[2]) * r + A[3]) * r + A[4]) * r + A[5]) * q
            / (((((B[0] * r + B[1]) * r + B[2]) * r + B[3]) * r + B[4]) * r + 1.0)
    } else {
        // Upper tail
        let q = (-2.0 * (1.0 - p).ln()).sqrt();
        -(((((C[0] * q + C[1]) * q + C[2]) * q + C[3]) * q + C[4]) * q + C[5])
            / ((((D[0] * q + D[1]) * q + D[2]) * q + D[3]) * q + 1.0)
    }
}

/// Lanczos approximation for Γ(z), z > 0.
fn gamma_fn(z: f64) -> f64 {
    if z < 0.5 {
        std::f64::consts::PI / ((std::f64::consts::PI * z).sin() * gamma_fn(1.0 - z))
    } else {
        let z = z - 1.0;
        let g = 7.0f64;
        let c: [f64; 9] = [
            0.999_999_999_999_809_9,
            676.520_368_121_885_1,
            -1_259.139_216_722_402_8,
            771.323_428_777_653_1,
            -176.615_029_162_140_6,
            12.507_343_278_686_905,
            -0.138_571_095_265_720_6,
            9.984_369_578_019_572e-6,
            1.505_632_735_149_311_6e-7,
        ];
        let mut x = c[0];
        for (i, &ci) in c[1..].iter().enumerate() {
            x += ci / (z + (i as f64 + 1.0));
        }
        let t = z + g + 0.5;
        (2.0 * std::f64::consts::PI).sqrt() * t.powf(z + 0.5) * (-t).exp() * x
    }
}

/// Regularised incomplete beta function I(x; a, b) via series expansion.
fn regularised_beta(x: f64, a: f64, b: f64) -> f64 {
    if x <= 0.0 {
        return 0.0;
    }
    if x >= 1.0 {
        return 1.0;
    }
    let ln_beta_ab = log_gamma(a) + log_gamma(b) - log_gamma(a + b);
    let front = (a * x.ln() + b * (1.0 - x).ln() - ln_beta_ab).exp() / a;
    front * beta_cf(x, a, b)
}

fn log_gamma(z: f64) -> f64 {
    gamma_fn(z).abs().ln()
}

fn beta_cf(x: f64, a: f64, b: f64) -> f64 {
    let mut c = 1.0f64;
    let mut d = 1.0 - (a + b) * x / (a + 1.0);
    if d.abs() < 1e-30 {
        d = 1e-30;
    }
    d = 1.0 / d;
    let mut f = d;
    for m_int in 1..=100usize {
        let m = m_int as f64;
        let aa = m * (b - m) * x / ((a + 2.0 * m - 1.0) * (a + 2.0 * m));
        d = 1.0 + aa * d;
        if d.abs() < 1e-30 {
            d = 1e-30;
        }
        d = 1.0 / d;
        if c.abs() < 1e-30 {
            c = 1e-30;
        }
        c = 1.0 + aa / c;
        f *= c * d;
        let aa2 = -(a + m) * (a + b + m) * x / ((a + 2.0 * m) * (a + 2.0 * m + 1.0));
        d = 1.0 + aa2 * d;
        if d.abs() < 1e-30 {
            d = 1e-30;
        }
        d = 1.0 / d;
        if c.abs() < 1e-30 {
            c = 1e-30;
        }
        c = 1.0 + aa2 / c;
        let del = c * d;
        f *= del;
        if (del - 1.0).abs() < 1e-9 {
            break;
        }
    }
    f
}

/// Beta ICDF via Newton's method on regularised incomplete beta.
fn beta_icdf(p: f64, alpha: f64, beta: f64) -> f64 {
    let mut x = 0.5f64;
    for _ in 0..30 {
        let fx = regularised_beta(x, alpha, beta) - p;
        let ln_beta = log_gamma(alpha) + log_gamma(beta) - log_gamma(alpha + beta);
        let pdf = (x.powf(alpha - 1.0) * (1.0 - x).powf(beta - 1.0)) / ln_beta.exp();
        let step = fx / pdf.max(1e-30);
        x = (x - step).clamp(1e-9, 1.0 - 1e-9);
        if step.abs() < 1e-9 {
            break;
        }
    }
    x
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::{Branch, Bus, BusType, PowerNetwork};
    use crate::units::{Power, ReactivePower, Voltage};

    // --- Distribution tests ---

    #[test]
    fn test_normal_dist_moments() {
        let d = UncertaintyDistribution::Normal {
            mean: 10.0,
            std_dev: 2.0,
        };
        assert!((d.mean() - 10.0).abs() < 1e-9);
        assert!((d.std_dev() - 2.0).abs() < 1e-9);
        assert!((d.variance() - 4.0).abs() < 1e-9);
    }

    #[test]
    fn test_uniform_dist_moments() {
        let d = UncertaintyDistribution::Uniform {
            low: 0.0,
            high: 10.0,
        };
        assert!((d.mean() - 5.0).abs() < 1e-9);
        assert!((d.variance() - 100.0 / 12.0).abs() < 1e-6);
    }

    #[test]
    fn test_weibull_dist_mean() {
        let d = UncertaintyDistribution::Weibull {
            scale: 10.0,
            shape: 2.0,
        };
        let expected = 10.0 * gamma_fn(1.5);
        assert!((d.mean() - expected).abs() < 1e-4);
    }

    #[test]
    fn test_lognormal_dist_mean() {
        let d = UncertaintyDistribution::Lognormal {
            mu: 0.0,
            sigma: 1.0,
        };
        assert!((d.mean() - (0.5f64).exp()).abs() < 1e-9);
    }

    #[test]
    fn test_normal_icdf_roundtrip() {
        let d = UncertaintyDistribution::Normal {
            mean: 5.0,
            std_dev: 1.5,
        };
        for p in [0.1, 0.5, 0.9] {
            let x = d.icdf(p);
            let p_back = normal_cdf((x - 5.0) / 1.5);
            assert!((p_back - p).abs() < 0.01, "p={p} p_back={p_back}");
        }
    }

    #[test]
    fn test_weibull_icdf_monotone() {
        let d = UncertaintyDistribution::Weibull {
            scale: 5.0,
            shape: 2.0,
        };
        assert!(d.icdf(0.75) > d.icdf(0.25));
    }

    #[test]
    fn test_uniform_icdf() {
        let d = UncertaintyDistribution::Uniform {
            low: 2.0,
            high: 8.0,
        };
        assert!((d.icdf(0.5) - 5.0).abs() < 1e-9);
    }

    #[test]
    fn test_beta_distribution_mean() {
        // Beta(2,2) symmetric: mean = 0.5
        let d = UncertaintyDistribution::Beta {
            alpha: 2.0,
            beta: 2.0,
            low: 0.0,
            high: 1.0,
        };
        assert!((d.mean() - 0.5).abs() < 1e-9);
    }

    // --- OutputStats tests ---

    #[test]
    fn test_output_stats_from_samples() {
        let samples: Vec<f64> = (1..=100).map(|i| i as f64).collect();
        let stats = OutputStats::from_samples(samples);
        assert_eq!(stats.n_samples, 100);
        assert!((stats.mean - 50.5).abs() < 1e-6);
        assert!(stats.p05 < stats.p50);
        assert!(stats.p50 < stats.p95);
    }

    #[test]
    fn test_output_stats_prob_exceed() {
        let samples: Vec<f64> = (0..1000).map(|i| i as f64 / 10.0).collect();
        let stats = OutputStats::from_samples(samples);
        let p = stats.prob_exceed(50.0);
        assert!(p > 0.3 && p < 0.7);
    }

    #[test]
    fn test_output_stats_constant() {
        let stats = OutputStats::from_samples(vec![5.0f64; 100]);
        assert!((stats.mean - 5.0).abs() < 1e-9);
        assert!(stats.std_dev < 1e-9);
        assert_eq!(stats.prob_exceed(4.0), 1.0);
        assert_eq!(stats.prob_exceed(5.0), 0.0);
    }

    // --- Normal CDF / ICDF tests ---

    #[test]
    fn test_normal_cdf_symmetry() {
        assert!((normal_cdf(0.0) - 0.5).abs() < 1e-3);
        assert!((normal_cdf(1.0) + normal_cdf(-1.0) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_normal_icdf_boundary() {
        assert!(normal_icdf(0.5).abs() < 0.01);
        assert!((normal_icdf(0.975) - 1.96).abs() < 0.05);
    }

    // --- LHS stratification test ---

    #[test]
    fn test_lhs_sample_coverage() {
        let mut rng = Lcg64::new(99);
        let matrix = lhs_sample(3, 20, &mut rng);
        assert_eq!(matrix.len(), 3);
        for row in &matrix {
            assert_eq!(row.len(), 20);
            let mut counts = [0usize; 20];
            for &v in row {
                let j = (v * 20.0) as usize;
                counts[j.min(19)] += 1;
            }
            assert!(counts.iter().all(|&c| c == 1));
        }
    }

    // --- Gamma function test ---

    #[test]
    fn test_gamma_function() {
        assert!((gamma_fn(1.0) - 1.0).abs() < 1e-9);
        assert!((gamma_fn(2.0) - 1.0).abs() < 1e-9);
        assert!((gamma_fn(0.5) - std::f64::consts::PI.sqrt()).abs() < 1e-6);
        assert!((gamma_fn(5.0) - 24.0).abs() < 1e-4); // 4! = 24
    }

    // --- PPF integration tests ---

    fn make_3bus_network() -> PowerNetwork {
        PowerNetwork {
            buses: vec![
                Bus {
                    id: 1,
                    name: "B1".into(),
                    bus_type: BusType::Slack,
                    base_kv: Voltage(69.0),
                    vm: 1.0,
                    va: 0.0,
                    pd: Power(0.0),
                    qd: ReactivePower(0.0),
                    gs: 0.0,
                    bs: 0.0,
                    zone: None,
                },
                Bus {
                    id: 2,
                    name: "B2".into(),
                    bus_type: BusType::PQ,
                    base_kv: Voltage(69.0),
                    vm: 1.0,
                    va: 0.0,
                    pd: Power(50.0),
                    qd: ReactivePower(10.0),
                    gs: 0.0,
                    bs: 0.0,
                    zone: None,
                },
                Bus {
                    id: 3,
                    name: "B3".into(),
                    bus_type: BusType::PQ,
                    base_kv: Voltage(69.0),
                    vm: 1.0,
                    va: 0.0,
                    pd: Power(30.0),
                    qd: ReactivePower(5.0),
                    gs: 0.0,
                    bs: 0.0,
                    zone: None,
                },
            ],
            branches: vec![
                Branch {
                    from_bus: 1,
                    to_bus: 2,
                    r: 0.01938,
                    x: 0.05917,
                    b: 0.0528,
                    rate_a: 100.0,
                    rate_b: 100.0,
                    rate_c: 100.0,
                    tap: 1.0,
                    shift: 0.0,
                    status: true,
                },
                Branch {
                    from_bus: 2,
                    to_bus: 3,
                    r: 0.04699,
                    x: 0.19797,
                    b: 0.0438,
                    rate_a: 100.0,
                    rate_b: 100.0,
                    rate_c: 100.0,
                    tap: 1.0,
                    shift: 0.0,
                    status: true,
                },
            ],
            generators: vec![],
            base_mva: 100.0,
        }
    }

    #[test]
    fn test_mc_ppf_runs() {
        let network = make_3bus_network();
        let injections = vec![UncertainInjection {
            bus_id: 2,
            delta_p: UncertaintyDistribution::Normal {
                mean: 0.0,
                std_dev: 5.0,
            },
            delta_q: None,
        }];
        let config = PpfConfig {
            method: PpfMethod::MonteCarlo,
            n_samples: 50,
            seed: 42,
            pf_config: PowerFlowConfig {
                method: PowerFlowMethod::DcApproximation,
                max_iter: 30,
                tolerance: 1e-6,
                enforce_q_limits: false,
            },
        };
        let result = run_mc_ppf(&network, &injections, &config).unwrap();
        assert_eq!(result.method, PpfMethod::MonteCarlo);
        assert!(!result.voltage_stats.is_empty());
    }

    #[test]
    fn test_lhs_ppf_runs() {
        let network = make_3bus_network();
        let injections = vec![UncertainInjection {
            bus_id: 2,
            delta_p: UncertaintyDistribution::Uniform {
                low: -10.0,
                high: 10.0,
            },
            delta_q: None,
        }];
        let config = PpfConfig {
            method: PpfMethod::LatinHypercube,
            n_samples: 30,
            seed: 7,
            pf_config: PowerFlowConfig {
                method: PowerFlowMethod::DcApproximation,
                max_iter: 30,
                tolerance: 1e-6,
                enforce_q_limits: false,
            },
        };
        let result = run_lhs_ppf(&network, &injections, &config).unwrap();
        assert_eq!(result.method, PpfMethod::LatinHypercube);
        assert!(!result.branch_flow_stats.is_empty());
    }

    #[test]
    fn test_pem_ppf_runs() {
        let network = make_3bus_network();
        let injections = vec![UncertainInjection {
            bus_id: 2,
            delta_p: UncertaintyDistribution::Normal {
                mean: 0.0,
                std_dev: 3.0,
            },
            delta_q: None,
        }];
        let config = PpfConfig {
            method: PpfMethod::PointEstimate,
            n_samples: 3,
            seed: 1,
            pf_config: PowerFlowConfig {
                method: PowerFlowMethod::DcApproximation,
                max_iter: 30,
                tolerance: 1e-6,
                enforce_q_limits: false,
            },
        };
        let result = run_pem_ppf(&network, &injections, &config).unwrap();
        assert_eq!(result.method, PpfMethod::PointEstimate);
        assert_eq!(result.n_samples, 3); // 2*1+1
    }

    #[test]
    fn test_linear_ppf_runs() {
        let network = make_3bus_network();
        let injections = vec![UncertainInjection {
            bus_id: 2,
            delta_p: UncertaintyDistribution::Normal {
                mean: 0.0,
                std_dev: 5.0,
            },
            delta_q: None,
        }];
        // Use Newton-Raphson so voltage magnitudes actually change with load perturbation
        // (DC power flow fixes all voltages at 1.0 pu, yielding zero sensitivity).
        let config = PpfConfig {
            method: PpfMethod::LinearSensitivity,
            n_samples: 1,
            seed: 42,
            pf_config: PowerFlowConfig {
                method: PowerFlowMethod::NewtonRaphson,
                max_iter: 50,
                tolerance: 1e-6,
                enforce_q_limits: false,
            },
        };
        let result =
            run_linear_ppf(&network, &injections, &config).expect("linear PPF should converge");
        assert_eq!(result.method, PpfMethod::LinearSensitivity);
        // Voltage uncertainty should propagate
        assert!(result.voltage_stats[1].std_dev > 0.0);
    }

    #[test]
    fn test_pem_2_injections() {
        let network = make_3bus_network();
        let injections = vec![
            UncertainInjection {
                bus_id: 2,
                delta_p: UncertaintyDistribution::Normal {
                    mean: 0.0,
                    std_dev: 3.0,
                },
                delta_q: None,
            },
            UncertainInjection {
                bus_id: 3,
                delta_p: UncertaintyDistribution::Uniform {
                    low: -5.0,
                    high: 5.0,
                },
                delta_q: None,
            },
        ];
        let config = PpfConfig {
            method: PpfMethod::PointEstimate,
            n_samples: 5,
            seed: 1,
            pf_config: PowerFlowConfig {
                method: PowerFlowMethod::DcApproximation,
                max_iter: 30,
                tolerance: 1e-6,
                enforce_q_limits: false,
            },
        };
        let result = run_pem_ppf(&network, &injections, &config).unwrap();
        assert_eq!(result.n_samples, 5); // 2*2+1
    }

    #[test]
    fn test_mc_vs_linear_mean_close() {
        let network = make_3bus_network();
        let injections = vec![UncertainInjection {
            bus_id: 2,
            delta_p: UncertaintyDistribution::Normal {
                mean: 0.0,
                std_dev: 2.0,
            },
            delta_q: None,
        }];
        let pf_config = PowerFlowConfig {
            method: PowerFlowMethod::DcApproximation,
            max_iter: 30,
            tolerance: 1e-6,
            enforce_q_limits: false,
        };
        let mc_config = PpfConfig {
            method: PpfMethod::MonteCarlo,
            n_samples: 200,
            seed: 99,
            pf_config: pf_config.clone(),
        };
        let lin_config = PpfConfig {
            pf_config,
            ..PpfConfig::default()
        };
        let mc = run_mc_ppf(&network, &injections, &mc_config).unwrap();
        let lin = run_linear_ppf(&network, &injections, &lin_config).unwrap();
        // Branch flow means should be close (linear DC case)
        let mc_mean = mc.branch_flow_stats[0].mean;
        let lin_mean = lin.branch_flow_stats[0].mean;
        assert!(
            (mc_mean - lin_mean).abs() < 5.0,
            "MC:{mc_mean} lin:{lin_mean}"
        );
    }
}
