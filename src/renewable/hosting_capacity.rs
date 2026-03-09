//! Probabilistic Hosting Capacity Analysis using Monte Carlo simulation.
//!
//! Extends deterministic hosting capacity assessment with a full Monte Carlo
//! framework that captures uncertainty in DG placement across the feeder.
//! The analysis sweeps DG penetration from zero to a configurable maximum,
//! checking voltage and thermal constraints at each step, and aggregates
//! results into probabilistic percentile statistics.
//!
//! # Approach
//!
//! For each Monte Carlo trial:
//! 1. Select a DG placement bus according to [`PlacementStrategy`].
//! 2. Sweep DG output from 0 to `max_penetration_mw` in `step_mw` increments.
//! 3. At each step, estimate voltage deviation and branch loading using a
//!    linearised sensitivity model \[ΔV ≈ R\_ij · P\_dg / V\_base\].
//! 4. Record the penetration level at the first constraint violation.
//!
//! Results are sorted and percentiles extracted.
//!
//! # References
//! - Ismael et al., "State-of-the-art of hosting capacity in modern power systems
//!   with distributed generation", Renewable Energy, 2019.
//! - Bollen & Hassan, "Integration of Distributed Generation in the Power System",
//!   Wiley-IEEE Press, 2011.

use crate::error::{OxiGridError, Result};
use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// LCG random number generator (Knuth MMIX constants — no `rand` crate)
// ---------------------------------------------------------------------------

/// Linear Congruential Generator using Knuth MMIX constants.
struct Lcg {
    state: u64,
}

impl Lcg {
    fn new(seed: u64) -> Self {
        Lcg {
            state: seed.wrapping_add(1),
        } // avoid zero state
    }

    fn next_u64(&mut self) -> u64 {
        self.state = self
            .state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.state
    }

    fn next_usize(&mut self, n: usize) -> usize {
        if n == 0 {
            return 0;
        }
        (self.next_u64() % n as u64) as usize
    }
}

// ---------------------------------------------------------------------------
// Public data structures
// ---------------------------------------------------------------------------

/// Strategy for placing DG units across Monte Carlo trials.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum PlacementStrategy {
    /// Random bus selection uniformly over all non-substation buses each trial.
    Uniform,
    /// Always place at the electrically weakest bus (highest impedance to source).
    ///
    /// This is equivalent to the deterministic worst-case HC.
    WorstCase,
    /// Always place at the electrically strongest bus (lowest impedance to source).
    BestCase,
    /// Random bus each trial (alias for [`Uniform`](PlacementStrategy::Uniform)).
    Random,
}

/// Configuration for the probabilistic hosting capacity analysis.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProbabilisticHcConfig {
    /// Number of Monte Carlo trials (default 1000).
    pub n_monte_carlo: usize,
    /// DG power factor (default 1.0 = unity, real power only).
    pub dg_power_factor: f64,
    /// Voltage limits \[p.u.\] as (min, max).  Typical: (0.95, 1.05).
    pub voltage_limit_pu: (f64, f64),
    /// Thermal loading limit as a fraction of rated capacity.  1.0 = 100%.
    pub thermal_limit_pct: f64,
    /// DG penetration search step \[MW\].
    pub step_mw: f64,
    /// Maximum DG penetration to search \[MW\].
    pub max_penetration_mw: f64,
    /// Bus selection strategy for each trial.
    pub placement_strategy: PlacementStrategy,
    /// Seed for the LCG pseudo-random number generator.
    pub seed: u64,
}

impl Default for ProbabilisticHcConfig {
    fn default() -> Self {
        Self {
            n_monte_carlo: 1000,
            dg_power_factor: 1.0,
            voltage_limit_pu: (0.95, 1.05),
            thermal_limit_pct: 1.0,
            step_mw: 0.1,
            max_penetration_mw: 10.0,
            placement_strategy: PlacementStrategy::Uniform,
            seed: 42,
        }
    }
}

/// Probabilistic hosting capacity result.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProbabilisticHcResult {
    /// Deterministic (worst-bus) hosting capacity \[MW\].
    pub hc_deterministic_mw: f64,
    /// 50th percentile HC across Monte Carlo trials \[MW\].
    pub hc_p50_mw: f64,
    /// 90th percentile HC (optimistic scenario) \[MW\].
    pub hc_p90_mw: f64,
    /// 10th percentile HC (conservative scenario) \[MW\].
    pub hc_p10_mw: f64,
    /// Probability of voltage violation at the P50 penetration level.
    pub voltage_violation_probability: f64,
    /// Probability of thermal violation at the P50 penetration level.
    pub thermal_violation_probability: f64,
    /// Index of the electrically weakest bus.
    pub weakest_bus: usize,
    /// Per-bus hosting capacity \[MW\]: `(bus_index, hc_mw)`.
    pub hc_by_bus: Vec<(usize, f64)>,
}

// ---------------------------------------------------------------------------
// Network model for sensitivity computation
// ---------------------------------------------------------------------------

/// Simplified distribution feeder model used for HC assessment.
///
/// Represents a radial feeder as a sequence of R+jX segments.
#[derive(Debug, Clone)]
pub struct FeederModel {
    /// Number of load buses (excluding the slack/substation bus 0).
    pub n_load_buses: usize,
    /// Cumulative resistance from substation to each load bus \[Ω or p.u.\].
    /// Index `i` corresponds to load bus `i+1`.
    pub r_cumulative: Vec<f64>,
    /// Cumulative reactance from substation to each load bus \[p.u.\].
    pub x_cumulative: Vec<f64>,
    /// Rated branch capacity \[MW\] for each segment.
    pub branch_rating_mw: Vec<f64>,
    /// Base load at each bus \[MW\] (without DG).
    pub bus_load_mw: Vec<f64>,
    /// System voltage base \[p.u.\] (typically 1.0).
    pub v_base_pu: f64,
}

impl FeederModel {
    /// Estimate voltage at each load bus with DG of `p_dg_mw` at `dg_bus` (0-based load bus).
    ///
    /// Uses a first-order voltage sensitivity: ΔV\_i ≈ R\_i · P\_dg / V\_base.
    /// Positive DG injection raises voltage downstream of injection point.
    fn voltage_with_dg(&self, dg_bus: usize, p_dg_mw: f64) -> Vec<f64> {
        let n = self.n_load_buses;
        // Build voltage vector: V_i = V_base - Σ_{j=0}^{i} R_j * (P_j - P_dg·δ(j=dg_bus)) / V_base
        (0..n)
            .map(|i| {
                let base_drop: f64 = (0..=i)
                    .map(|j| {
                        if j == dg_bus {
                            -self.r_cumulative[j] * p_dg_mw / self.v_base_pu
                        } else {
                            self.r_cumulative[j] * self.bus_load_mw[j] / self.v_base_pu
                        }
                    })
                    .sum();
                self.v_base_pu - base_drop
            })
            .collect()
    }

    /// Estimate branch loading \[MW\] at each segment with DG injection.
    fn branch_loading_with_dg(&self, dg_bus: usize, p_dg_mw: f64) -> Vec<f64> {
        let n = self.n_load_buses;
        (0..n)
            .map(|seg| {
                // Power flow through segment = sum of loads downstream minus DG if downstream
                let total_downstream: f64 = (seg..n).map(|j| self.bus_load_mw[j]).sum::<f64>();
                let dg_downstream = if dg_bus >= seg { p_dg_mw } else { 0.0 };
                (total_downstream - dg_downstream).max(0.0)
            })
            .collect()
    }

    /// Compute hosting capacity at a specific bus \[MW\].
    fn hc_at_bus(&self, bus: usize, config: &ProbabilisticHcConfig) -> f64 {
        let mut p = 0.0_f64;
        loop {
            p += config.step_mw;
            if p > config.max_penetration_mw {
                return config.max_penetration_mw;
            }
            let voltages = self.voltage_with_dg(bus, p);
            let loadings = self.branch_loading_with_dg(bus, p);

            let v_viol = voltages
                .iter()
                .any(|&v| v < config.voltage_limit_pu.0 || v > config.voltage_limit_pu.1);

            let rated = self
                .branch_rating_mw
                .get(bus)
                .copied()
                .unwrap_or(f64::INFINITY);
            let t_viol = loadings
                .iter()
                .any(|&l| l > rated * config.thermal_limit_pct);

            if v_viol || t_viol {
                return (p - config.step_mw).max(0.0);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Main analysis function
// ---------------------------------------------------------------------------

/// Run probabilistic hosting capacity analysis using Monte Carlo simulation.
///
/// # Arguments
/// - `model`  — simplified feeder model with impedance sensitivities
/// - `config` — MC configuration (trials, strategy, limits)
///
/// # Returns
/// A [`ProbabilisticHcResult`] with deterministic and probabilistic HC metrics.
///
/// # Errors
/// Returns [`OxiGridError::InvalidParameter`] if the feeder model is empty or
/// the configuration contains invalid values.
pub fn probabilistic_hosting_capacity(
    model: &FeederModel,
    config: &ProbabilisticHcConfig,
) -> Result<ProbabilisticHcResult> {
    if model.n_load_buses == 0 {
        return Err(OxiGridError::InvalidParameter(
            "FeederModel has no load buses".into(),
        ));
    }
    if config.step_mw <= 0.0 {
        return Err(OxiGridError::InvalidParameter(
            "step_mw must be positive".into(),
        ));
    }
    if config.max_penetration_mw < config.step_mw {
        return Err(OxiGridError::InvalidParameter(
            "max_penetration_mw must be >= step_mw".into(),
        ));
    }

    let n = model.n_load_buses;

    // --- Per-bus deterministic HC ---
    let hc_by_bus: Vec<(usize, f64)> = (0..n)
        .map(|bus| (bus, model.hc_at_bus(bus, config)))
        .collect();

    // Weakest bus = minimum HC
    let (weakest_bus, hc_deterministic_mw) = hc_by_bus
        .iter()
        .copied()
        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
        .unwrap_or((0, 0.0));

    // Best bus = maximum HC
    let best_bus = hc_by_bus
        .iter()
        .copied()
        .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(b, _)| b)
        .unwrap_or(0);

    // --- Monte Carlo ---
    let mut lcg = Lcg::new(config.seed);
    let mut mc_hcs: Vec<f64> = Vec::with_capacity(config.n_monte_carlo);

    for _ in 0..config.n_monte_carlo {
        let bus = match config.placement_strategy {
            PlacementStrategy::Uniform | PlacementStrategy::Random => lcg.next_usize(n),
            PlacementStrategy::WorstCase => weakest_bus,
            PlacementStrategy::BestCase => best_bus,
        };
        mc_hcs.push(model.hc_at_bus(bus, config));
    }

    mc_hcs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

    let percentile = |p: f64| -> f64 {
        let idx = ((p / 100.0) * mc_hcs.len() as f64) as usize;
        mc_hcs
            .get(idx.min(mc_hcs.len().saturating_sub(1)))
            .copied()
            .unwrap_or(0.0)
    };

    let hc_p10_mw = percentile(10.0);
    let hc_p50_mw = percentile(50.0);
    let hc_p90_mw = percentile(90.0);

    // --- Violation probabilities at P50 ---
    let p50 = hc_p50_mw;
    let mut v_viol_count = 0usize;
    let mut t_viol_count = 0usize;
    let n_probe = config.n_monte_carlo.min(200); // limit probe runs
    let mut lcg2 = Lcg::new(config.seed.wrapping_add(999));

    for _ in 0..n_probe {
        let bus = match config.placement_strategy {
            PlacementStrategy::Uniform | PlacementStrategy::Random => lcg2.next_usize(n),
            PlacementStrategy::WorstCase => weakest_bus,
            PlacementStrategy::BestCase => best_bus,
        };
        let voltages = model.voltage_with_dg(bus, p50);
        let loadings = model.branch_loading_with_dg(bus, p50);
        let rated = model
            .branch_rating_mw
            .get(bus)
            .copied()
            .unwrap_or(f64::INFINITY);

        if voltages
            .iter()
            .any(|&v| v < config.voltage_limit_pu.0 || v > config.voltage_limit_pu.1)
        {
            v_viol_count += 1;
        }
        if loadings
            .iter()
            .any(|&l| l > rated * config.thermal_limit_pct)
        {
            t_viol_count += 1;
        }
    }

    let voltage_violation_probability = v_viol_count as f64 / n_probe.max(1) as f64;
    let thermal_violation_probability = t_viol_count as f64 / n_probe.max(1) as f64;

    Ok(ProbabilisticHcResult {
        hc_deterministic_mw,
        hc_p50_mw,
        hc_p90_mw,
        hc_p10_mw,
        voltage_violation_probability,
        thermal_violation_probability,
        weakest_bus,
        hc_by_bus,
    })
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn simple_feeder_model() -> FeederModel {
        // 4-bus radial: sub(0) — bus1 — bus2 — bus3 — bus4
        // R=0.1 pu per segment, X=0.05, rated 5 MW per branch
        FeederModel {
            n_load_buses: 4,
            r_cumulative: vec![0.1, 0.2, 0.3, 0.4],
            x_cumulative: vec![0.05, 0.10, 0.15, 0.20],
            branch_rating_mw: vec![5.0, 5.0, 5.0, 5.0],
            bus_load_mw: vec![0.5, 0.5, 0.5, 0.5],
            v_base_pu: 1.0,
        }
    }

    #[test]
    fn test_uniform_p50_ge_deterministic() {
        let model = simple_feeder_model();
        let config = ProbabilisticHcConfig {
            n_monte_carlo: 500,
            placement_strategy: PlacementStrategy::Uniform,
            step_mw: 0.2,
            max_penetration_mw: 8.0,
            seed: 123,
            ..Default::default()
        };
        let result = probabilistic_hosting_capacity(&model, &config).expect("hc analysis");
        // P50 should be >= deterministic (worst-case) since uniform picks diverse buses
        assert!(
            result.hc_p50_mw >= result.hc_deterministic_mw - 0.2,
            "P50={:.2} should be >= deterministic={:.2} (±step)",
            result.hc_p50_mw,
            result.hc_deterministic_mw
        );
    }

    #[test]
    fn test_worst_case_matches_deterministic() {
        let model = simple_feeder_model();
        let config = ProbabilisticHcConfig {
            n_monte_carlo: 100,
            placement_strategy: PlacementStrategy::WorstCase,
            step_mw: 0.1,
            max_penetration_mw: 10.0,
            seed: 42,
            ..Default::default()
        };
        let result = probabilistic_hosting_capacity(&model, &config).expect("hc analysis");
        // When always placing at worst bus, P50 == deterministic
        assert!(
            (result.hc_p50_mw - result.hc_deterministic_mw).abs() < 0.11,
            "WorstCase: P50={:.3} should match deterministic={:.3}",
            result.hc_p50_mw,
            result.hc_deterministic_mw
        );
    }

    #[test]
    fn test_max_penetration_cap_respected() {
        let model = simple_feeder_model();
        let config = ProbabilisticHcConfig {
            n_monte_carlo: 200,
            placement_strategy: PlacementStrategy::BestCase,
            step_mw: 0.5,
            max_penetration_mw: 2.0, // very low cap
            seed: 7,
            ..Default::default()
        };
        let result = probabilistic_hosting_capacity(&model, &config).expect("hc analysis");
        // All HC values should be <= max_penetration_mw
        assert!(
            result.hc_p90_mw <= config.max_penetration_mw + config.step_mw,
            "P90={:.2} should not exceed max_penetration_mw={:.2}",
            result.hc_p90_mw,
            config.max_penetration_mw
        );
        assert!(
            result.hc_deterministic_mw <= config.max_penetration_mw + config.step_mw,
            "Deterministic HC should not exceed max"
        );
    }

    #[test]
    fn test_hc_by_bus_populated() {
        let model = simple_feeder_model();
        let config = ProbabilisticHcConfig::default();
        let result = probabilistic_hosting_capacity(&model, &config).expect("hc analysis");
        assert_eq!(
            result.hc_by_bus.len(),
            model.n_load_buses,
            "hc_by_bus should have entry per load bus"
        );
        for (bus_idx, hc) in &result.hc_by_bus {
            assert!(*bus_idx < model.n_load_buses);
            assert!(*hc >= 0.0 && *hc <= config.max_penetration_mw + config.step_mw);
        }
    }

    #[test]
    fn test_percentile_ordering() {
        let model = simple_feeder_model();
        let config = ProbabilisticHcConfig {
            n_monte_carlo: 300,
            placement_strategy: PlacementStrategy::Uniform,
            seed: 99,
            ..Default::default()
        };
        let result = probabilistic_hosting_capacity(&model, &config).expect("hc analysis");
        // P10 <= P50 <= P90 always
        assert!(
            result.hc_p10_mw <= result.hc_p50_mw + 1e-9,
            "P10={:.3} should be <= P50={:.3}",
            result.hc_p10_mw,
            result.hc_p50_mw
        );
        assert!(
            result.hc_p50_mw <= result.hc_p90_mw + 1e-9,
            "P50={:.3} should be <= P90={:.3}",
            result.hc_p50_mw,
            result.hc_p90_mw
        );
    }

    #[test]
    fn test_empty_feeder_returns_error() {
        let model = FeederModel {
            n_load_buses: 0,
            r_cumulative: vec![],
            x_cumulative: vec![],
            branch_rating_mw: vec![],
            bus_load_mw: vec![],
            v_base_pu: 1.0,
        };
        let config = ProbabilisticHcConfig::default();
        let result = probabilistic_hosting_capacity(&model, &config);
        assert!(result.is_err(), "Empty feeder should return error");
    }

    #[test]
    fn test_lcg_reproducible() {
        let mut lcg1 = Lcg::new(42);
        let mut lcg2 = Lcg::new(42);
        for _ in 0..100 {
            assert_eq!(lcg1.next_u64(), lcg2.next_u64());
        }
    }
}
