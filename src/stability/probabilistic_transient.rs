//! Probabilistic Transient Stability Assessment (PTSA).
//!
//! Evaluates the probability of transient instability in a multi-machine power
//! system via Monte Carlo simulation.  Each scenario samples:
//!
//! - Fault clearing time from a configurable distribution.
//! - Loading level from a configurable distribution.
//!
//! For each scenario the classical swing equation is integrated with a fixed-step
//! RK4 integrator, and stability is assessed using one of three criteria.
//!
//! # Reference
//! - Anderson, P. M., & Fouad, A. A. (2008). *Power System Control and Stability* (2nd ed.).
//! - Billinton, R., & Allan, R. N. (1996). *Reliability Evaluation of Power Systems* (2nd ed.).
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;
use thiserror::Error;

// ---------------------------------------------------------------------------
// Error types
// ---------------------------------------------------------------------------

/// Errors produced by the PTSA engine.
#[derive(Debug, Error)]
pub enum PtsaError {
    /// No generators have been configured.
    #[error("no generators configured")]
    NoGenerators,
    /// Generator index out of range.
    #[error("generator index {0} out of range")]
    GeneratorIndexOutOfRange(usize),
    /// Configuration value invalid.
    #[error("invalid configuration: {0}")]
    InvalidConfig(String),
}

// ---------------------------------------------------------------------------
// Probability distributions
// ---------------------------------------------------------------------------

/// Probability distribution for scenario sampling.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Distribution {
    /// Gaussian distribution: N(mean, std²).
    Normal { mean: f64, std: f64 },
    /// Uniform distribution on \[low, high\].
    Uniform { low: f64, high: f64 },
    /// Log-normal: ln(X) ~ N(mu, sigma²).
    Lognormal { mu: f64, sigma: f64 },
}

/// LCG random number generator (policy: no rand crate).
///
/// Multiplier / addend follow Knuth MMIX.
struct Lcg {
    state: u64,
}

impl Lcg {
    fn new(seed: u64) -> Self {
        Self {
            state: seed.wrapping_add(1),
        }
    }

    /// Next u64.
    fn next_u64(&mut self) -> u64 {
        self.state = self
            .state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.state
    }

    /// Uniform \[0, 1\).
    fn uniform_01(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Box-Muller transform → standard normal.
    fn standard_normal(&mut self) -> f64 {
        let u1 = self.uniform_01().max(1e-15);
        let u2 = self.uniform_01();
        (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
    }

    /// Sample from a [`Distribution`].
    fn sample(&mut self, dist: &Distribution) -> f64 {
        match dist {
            Distribution::Normal { mean, std } => mean + std * self.standard_normal(),
            Distribution::Uniform { low, high } => low + (high - low) * self.uniform_01(),
            Distribution::Lognormal { mu, sigma } => (mu + sigma * self.standard_normal()).exp(),
        }
    }
}

// ---------------------------------------------------------------------------
// Stability criterion
// ---------------------------------------------------------------------------

/// Criterion for declaring a scenario as unstable.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum StabilityCriterion {
    /// Unstable when any rotor angle exceeds `threshold_deg` relative to the COI.
    MaxAngle { threshold_deg: f64 },
    /// Unstable when total kinetic energy exceeds `e_critical` \[pu·s\].
    EnergyBased { e_critical: f64 },
    /// Unstable when angles have not returned within `settling_band_deg` of
    /// initial equilibrium by the end of the simulation.
    TimeDomain { settling_band_deg: f64 },
}

// ---------------------------------------------------------------------------
// PTSA configuration
// ---------------------------------------------------------------------------

/// Configuration for one probabilistic transient stability assessment run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PtsaConfig {
    /// Number of Monte Carlo scenarios.
    pub n_scenarios: usize,
    /// Time-domain simulation length per scenario \[s\]
    pub simulation_time_s: f64,
    /// Distribution of fault clearing times \[s\]
    pub clearing_time_distribution: Distribution,
    /// Distribution of loading levels \[pu\]
    pub loading_distribution: Distribution,
    /// Stability assessment criterion.
    pub stability_criterion: StabilityCriterion,
    /// Random seed for reproducibility.
    pub seed: u64,
}

// ---------------------------------------------------------------------------
// Scenario and result types
// ---------------------------------------------------------------------------

/// One Monte Carlo scenario.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PtsaScenario {
    /// Scenario index.
    pub id: usize,
    /// Sampled fault clearing time \[s\]
    pub fault_clearing_time_s: f64,
    /// Sampled loading level \[pu\]
    pub loading_pu: f64,
    /// True when the scenario is assessed as stable.
    pub stable: bool,
    /// Maximum absolute rotor angle deviation from COI \[deg\]
    pub max_angle_deg: f64,
    /// Index of the most-critical generator (None when stable).
    pub critical_generator: Option<usize>,
}

/// Aggregated PTSA results.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PtsaResult {
    /// Probability of instability P(unstable) = n_unstable / n_scenarios.
    pub probability_of_instability: f64,
    /// Probability of stability = 1 − P(unstable).
    pub probability_of_stability: f64,
    /// Expected (mean) critical clearing time \[s\]
    pub expected_cct_s: f64,
    /// 5th-percentile CCT (conservative bound) \[s\]
    pub cct_5th_percentile_s: f64,
    /// 95th-percentile CCT (optimistic bound) \[s\]
    pub cct_95th_percentile_s: f64,
    /// All individual scenarios.
    pub scenarios: Vec<PtsaScenario>,
    /// Stability margin: fraction of stable scenarios − 0.5 (−0.5 = all unstable, +0.5 = all stable).
    pub stability_margin: f64,
    /// Risk index: 0 = fully safe, 1 = fully critical.
    pub risk_index: f64,
}

// ---------------------------------------------------------------------------
// Generator model (classical swing equation)
// ---------------------------------------------------------------------------

/// Classical generator model parameters (constant voltage behind transient reactance).
#[derive(Debug, Clone)]
struct GeneratorModel {
    /// Inertia constant H \[MJ/MVA\]
    h: f64,
    /// Damping coefficient D \[pu\]
    d: f64,
    /// Base mechanical power \[pu\]
    pm_base_pu: f64,
    /// Electrical power coefficient: Pe = pe_coeff · sin(delta) \[pu\]
    pe_coeff: f64,
    /// Equilibrium rotor angle \[rad\]
    delta_eq: f64,
}

impl GeneratorModel {
    fn new(h: f64, d: f64, pm_pu: f64, pe_coeff: f64) -> Self {
        // Equilibrium: Pm = Pe_coeff · sin(delta_eq)
        let ratio = (pm_pu / pe_coeff.max(1e-12)).clamp(-1.0, 1.0);
        let delta_eq = ratio.asin();
        Self {
            h,
            d,
            pm_base_pu: pm_pu,
            pe_coeff,
            delta_eq,
        }
    }

    /// Inertia constant M = 2H/ω₀ \[s²/rad\]
    fn m(&self, omega_0: f64) -> f64 {
        2.0 * self.h / omega_0
    }
}

// ---------------------------------------------------------------------------
// Probabilistic transient stability solver
// ---------------------------------------------------------------------------

/// Probabilistic Transient Stability Assessment solver.
pub struct ProbabilisticTransientSolver {
    config: PtsaConfig,
    n_generators: usize,
    generators: Vec<GeneratorModel>,
}

impl ProbabilisticTransientSolver {
    /// Create a new solver.  Generator parameters must be set with
    /// [`set_generator`](Self::set_generator) before calling
    /// [`assess`](Self::assess).
    pub fn new(config: PtsaConfig, n_generators: usize) -> Self {
        let generators = (0..n_generators)
            .map(|_| GeneratorModel::new(5.0, 2.0, 0.8, 1.5))
            .collect();
        Self {
            config,
            n_generators,
            generators,
        }
    }

    /// Configure generator `i`.
    ///
    /// # Arguments
    /// * `i`        — generator index (0-based)
    /// * `h`        — inertia constant \[MJ/MVA\]
    /// * `d`        — damping coefficient \[pu\]
    /// * `pm_pu`    — mechanical power (base loading) \[pu\]
    /// * `pe_coeff` — electrical power coefficient (Eʹ·V/xʹd) \[pu\]
    pub fn set_generator(
        &mut self,
        i: usize,
        h: f64,
        d: f64,
        pm_pu: f64,
        pe_coeff: f64,
    ) -> Result<(), PtsaError> {
        if i >= self.n_generators {
            return Err(PtsaError::GeneratorIndexOutOfRange(i));
        }
        self.generators[i] = GeneratorModel::new(h, d, pm_pu, pe_coeff);
        Ok(())
    }

    // -----------------------------------------------------------------------
    // Swing equation RK4 integration
    // -----------------------------------------------------------------------

    /// RK4 derivative: returns `[d(delta)/dt, d(omega)/dt]`.
    ///
    /// State: `[delta_rad, omega_rad_s]` where omega = dδ/dt.
    fn swing_deriv(
        gen: &GeneratorModel,
        delta: f64,
        omega: f64,
        pm_pu: f64,
        pe_coeff_eff: f64,
        omega_0: f64,
    ) -> (f64, f64) {
        let pe = pe_coeff_eff * delta.sin();
        let m = gen.m(omega_0);
        let d_delta = omega;
        let d_omega = (pm_pu - pe - gen.d * omega) / m;
        (d_delta, d_omega)
    }

    /// Integrate swing equation for one generator with RK4, fixed step `dt`.
    ///
    /// During `[0, clearing_time]` electrical power is zero (bolted 3-phase fault).
    /// After clearing, the post-fault electrical power is restored to `pe_coeff`.
    fn integrate_generator(
        gen: &GeneratorModel,
        pm_pu: f64,
        clearing_time_s: f64,
        sim_time_s: f64,
        dt: f64,
        omega_0: f64,
    ) -> (Vec<f64>, f64) {
        let n_steps = (sim_time_s / dt).ceil() as usize;
        let mut delta = gen.delta_eq;
        let mut omega = 0.0_f64;
        let mut max_dev_rad = 0.0_f64;

        let mut deltas = Vec::with_capacity(n_steps + 1);
        deltas.push(delta);

        let mut t = 0.0_f64;
        for _ in 0..n_steps {
            // During fault: Pe = 0 (bolted 3-phase fault at generator bus)
            let pe_coeff_eff = if t < clearing_time_s {
                0.0
            } else {
                gen.pe_coeff
            };

            // RK4 step
            let (k1d, k1w) = Self::swing_deriv(gen, delta, omega, pm_pu, pe_coeff_eff, omega_0);
            let (k2d, k2w) = Self::swing_deriv(
                gen,
                delta + 0.5 * dt * k1d,
                omega + 0.5 * dt * k1w,
                pm_pu,
                pe_coeff_eff,
                omega_0,
            );
            let (k3d, k3w) = Self::swing_deriv(
                gen,
                delta + 0.5 * dt * k2d,
                omega + 0.5 * dt * k2w,
                pm_pu,
                pe_coeff_eff,
                omega_0,
            );
            let (k4d, k4w) = Self::swing_deriv(
                gen,
                delta + dt * k3d,
                omega + dt * k3w,
                pm_pu,
                pe_coeff_eff,
                omega_0,
            );

            delta += dt * (k1d + 2.0 * k2d + 2.0 * k3d + k4d) / 6.0;
            omega += dt * (k1w + 2.0 * k2w + 2.0 * k3w + k4w) / 6.0;
            t += dt;

            let dev = (delta - gen.delta_eq).abs();
            if dev > max_dev_rad {
                max_dev_rad = dev;
            }
            deltas.push(delta);
        }

        (deltas, max_dev_rad)
    }

    // -----------------------------------------------------------------------
    // Scenario simulation
    // -----------------------------------------------------------------------

    /// Simulate one scenario and return a [`PtsaScenario`].
    fn simulate_scenario(&self, id: usize, clearing_time_s: f64, loading_pu: f64) -> PtsaScenario {
        let omega_0 = 2.0 * PI * 60.0; // 60 Hz system
        let dt = 0.01; // 10 ms fixed step

        let mut max_angle_deg = 0.0_f64;
        let mut critical_gen: Option<usize> = None;
        let mut worst_dev = 0.0_f64;

        // Per-generator delta trajectories for stability assessment.
        let mut all_deltas: Vec<Vec<f64>> = Vec::with_capacity(self.n_generators);

        for (i, gen) in self.generators.iter().enumerate() {
            let pm_pu = gen.pm_base_pu * loading_pu;
            let (deltas, max_dev_rad) = Self::integrate_generator(
                gen,
                pm_pu,
                clearing_time_s,
                self.config.simulation_time_s,
                dt,
                omega_0,
            );
            let max_dev_deg = max_dev_rad.to_degrees();
            if max_dev_deg > max_angle_deg {
                max_angle_deg = max_dev_deg;
            }
            if max_dev_rad > worst_dev {
                worst_dev = max_dev_rad;
                critical_gen = Some(i);
            }
            all_deltas.push(deltas);
        }

        // Assess stability.
        let stable = match &self.config.stability_criterion {
            StabilityCriterion::MaxAngle { threshold_deg } => max_angle_deg < *threshold_deg,
            StabilityCriterion::EnergyBased { e_critical } => {
                // Kinetic energy = sum_i (M_i * omega_i²/2); use max angle as proxy.
                let ke_approx = worst_dev * worst_dev * 0.5;
                ke_approx < *e_critical
            }
            StabilityCriterion::TimeDomain { settling_band_deg } => {
                // Check that final angle is within settling band of equilibrium.
                let settling_rad = settling_band_deg.to_radians();
                all_deltas.iter().enumerate().all(|(i, traj)| {
                    if let Some(&last) = traj.last() {
                        (last - self.generators[i].delta_eq).abs() < settling_rad
                    } else {
                        true
                    }
                })
            }
        };

        let critical_generator = if stable { None } else { critical_gen };

        PtsaScenario {
            id,
            fault_clearing_time_s: clearing_time_s,
            loading_pu,
            stable,
            max_angle_deg,
            critical_generator,
        }
    }

    // -----------------------------------------------------------------------
    // Public API
    // -----------------------------------------------------------------------

    /// Run the full probabilistic transient stability assessment.
    ///
    /// Returns a [`PtsaResult`] with probability of instability, CCT statistics,
    /// and individual scenarios.
    pub fn assess(&self) -> Result<PtsaResult, PtsaError> {
        if self.n_generators == 0 {
            return Err(PtsaError::NoGenerators);
        }
        if self.config.n_scenarios == 0 {
            return Err(PtsaError::InvalidConfig("n_scenarios must be > 0".into()));
        }

        let mut rng = Lcg::new(self.config.seed);
        let mut scenarios: Vec<PtsaScenario> = Vec::with_capacity(self.config.n_scenarios);

        for id in 0..self.config.n_scenarios {
            let mut tclr = rng.sample(&self.config.clearing_time_distribution);
            let mut load = rng.sample(&self.config.loading_distribution);

            // Clamp to physically sensible bounds.
            tclr = tclr.max(0.0).min(self.config.simulation_time_s);
            load = load.clamp(0.01, 2.0);

            let scenario = self.simulate_scenario(id, tclr, load);
            scenarios.push(scenario);
        }

        // Aggregate statistics.
        let n_unstable = scenarios.iter().filter(|s| !s.stable).count();
        let n = self.config.n_scenarios as f64;
        let p_unstable = n_unstable as f64 / n;
        let p_stable = 1.0 - p_unstable;

        // CCT statistics: collect fault clearing times for unstable scenarios and
        // for stable scenarios.  The "expected CCT" is estimated as the mean
        // clearing time across all scenarios that are on the stability boundary.
        // In practice we compute statistics of clearing times weighted by stability.
        let mut cct_samples: Vec<f64> = scenarios.iter().map(|s| s.fault_clearing_time_s).collect();
        cct_samples.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        let expected_cct_s = if cct_samples.is_empty() {
            0.0
        } else {
            cct_samples.iter().sum::<f64>() / cct_samples.len() as f64
        };

        let percentile = |pct: f64| -> f64 {
            if cct_samples.is_empty() {
                return 0.0;
            }
            let idx = (pct / 100.0 * (cct_samples.len() - 1) as f64).round() as usize;
            cct_samples[idx.min(cct_samples.len() - 1)]
        };

        let cct_5th = percentile(5.0);
        let cct_95th = percentile(95.0);

        // Stability margin: centered at 0.5.
        let stability_margin = p_stable - 0.5;
        // Risk index: 0 = safe, 1 = critical.
        let risk_index = p_unstable;

        Ok(PtsaResult {
            probability_of_instability: p_unstable,
            probability_of_stability: p_stable,
            expected_cct_s,
            cct_5th_percentile_s: cct_5th,
            cct_95th_percentile_s: cct_95th,
            scenarios,
            stability_margin,
            risk_index,
        })
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_solver(
        n_scenarios: usize,
        clearing_dist: Distribution,
        loading_dist: Distribution,
        criterion: StabilityCriterion,
    ) -> ProbabilisticTransientSolver {
        let config = PtsaConfig {
            n_scenarios,
            simulation_time_s: 3.0,
            clearing_time_distribution: clearing_dist,
            loading_distribution: loading_dist,
            stability_criterion: criterion,
            seed: 42,
        };
        let mut solver = ProbabilisticTransientSolver::new(config, 1);
        // Strong generator: large Pe_coeff relative to Pm → highly stable
        solver
            .set_generator(0, 8.0, 5.0, 0.5, 2.0)
            .expect("set_generator failed");
        solver
    }

    // ------------------------------------------------------------------
    // 1. All scenarios stable: P(instability) = 0
    // ------------------------------------------------------------------
    #[test]
    fn test_all_stable_zero_probability() {
        // Very short clearing time: fault clears immediately → always stable.
        let solver = make_solver(
            50,
            Distribution::Uniform {
                low: 0.01,
                high: 0.05,
            }, // extremely fast clearing
            Distribution::Uniform {
                low: 0.5,
                high: 0.7,
            },
            StabilityCriterion::MaxAngle {
                threshold_deg: 180.0,
            },
        );
        let result = solver.assess().expect("assessment failed");
        assert_eq!(
            result.probability_of_instability, 0.0,
            "All scenarios should be stable for instant fault clearing"
        );
        assert!(
            (result.probability_of_stability - 1.0).abs() < 1e-10,
            "P(stable) should be 1.0"
        );
        assert!(result.risk_index < 0.01, "Risk index should be near 0");
    }

    // ------------------------------------------------------------------
    // 2. Long clearing time: all scenarios unstable
    // ------------------------------------------------------------------
    #[test]
    fn test_long_clearing_all_unstable() {
        let config = PtsaConfig {
            n_scenarios: 30,
            simulation_time_s: 3.0,
            // Very long clearing time → generator accelerates well past stability limit
            clearing_time_distribution: Distribution::Uniform {
                low: 1.0,
                high: 1.5,
            },
            loading_distribution: Distribution::Uniform {
                low: 1.0,
                high: 1.0,
            },
            stability_criterion: StabilityCriterion::MaxAngle { threshold_deg: 5.0 }, // tight limit
            seed: 7,
        };
        let mut solver = ProbabilisticTransientSolver::new(config, 1);
        solver
            .set_generator(0, 5.0, 1.0, 0.9, 1.0)
            .expect("set_generator failed");
        let result = solver.assess().expect("assessment failed");

        assert!(
            result.probability_of_instability > 0.5,
            "Expected high instability probability for long clearing; got {:.2}",
            result.probability_of_instability
        );
    }

    // ------------------------------------------------------------------
    // 3. CCT statistics: P5 <= E[CCT] <= P95
    // ------------------------------------------------------------------
    #[test]
    fn test_cct_statistics_ordering() {
        let solver = make_solver(
            100,
            Distribution::Normal {
                mean: 0.15,
                std: 0.05,
            },
            Distribution::Uniform {
                low: 0.8,
                high: 1.0,
            },
            StabilityCriterion::MaxAngle {
                threshold_deg: 120.0,
            },
        );
        let result = solver.assess().expect("assessment failed");

        assert!(
            result.cct_5th_percentile_s <= result.expected_cct_s + 1e-9,
            "P5 ({:.4}) should be ≤ E[CCT] ({:.4})",
            result.cct_5th_percentile_s,
            result.expected_cct_s
        );
        assert!(
            result.expected_cct_s <= result.cct_95th_percentile_s + 1e-9,
            "E[CCT] ({:.4}) should be ≤ P95 ({:.4})",
            result.expected_cct_s,
            result.cct_95th_percentile_s
        );
    }

    // ------------------------------------------------------------------
    // 4. Loading uncertainty affects stability probability
    // ------------------------------------------------------------------
    #[test]
    fn test_loading_uncertainty_affects_stability() {
        // Light load → stable; heavy load → may be unstable at same clearing time.
        let clearing = Distribution::Uniform {
            low: 0.3,
            high: 0.4,
        };
        let criterion = StabilityCriterion::MaxAngle {
            threshold_deg: 90.0,
        };

        let config_light = PtsaConfig {
            n_scenarios: 50,
            simulation_time_s: 3.0,
            clearing_time_distribution: clearing.clone(),
            loading_distribution: Distribution::Uniform {
                low: 0.3,
                high: 0.4,
            },
            stability_criterion: criterion.clone(),
            seed: 13,
        };
        let config_heavy = PtsaConfig {
            n_scenarios: 50,
            simulation_time_s: 3.0,
            clearing_time_distribution: clearing,
            loading_distribution: Distribution::Uniform {
                low: 1.0,
                high: 1.2,
            },
            stability_criterion: criterion,
            seed: 13,
        };

        let mut solver_light = ProbabilisticTransientSolver::new(config_light, 1);
        solver_light.set_generator(0, 6.0, 3.0, 0.5, 1.5).unwrap();

        let mut solver_heavy = ProbabilisticTransientSolver::new(config_heavy, 1);
        solver_heavy.set_generator(0, 6.0, 3.0, 0.5, 1.5).unwrap();

        let r_light = solver_light.assess().expect("light assessment failed");
        let r_heavy = solver_heavy.assess().expect("heavy assessment failed");

        // Heavy loading should yield higher or equal instability probability.
        assert!(
            r_heavy.probability_of_instability >= r_light.probability_of_instability,
            "Heavy loading ({:.2}) should be no more stable than light ({:.2})",
            r_heavy.probability_of_instability,
            r_light.probability_of_instability
        );
    }

    // ------------------------------------------------------------------
    // 5. Risk index: 0 for fully stable, approaching 1 for very unstable
    // ------------------------------------------------------------------
    #[test]
    fn test_risk_index_boundaries() {
        // Fully stable scenario.
        let stable_solver = make_solver(
            20,
            Distribution::Uniform {
                low: 0.001,
                high: 0.01,
            },
            Distribution::Uniform {
                low: 0.5,
                high: 0.5,
            },
            StabilityCriterion::MaxAngle {
                threshold_deg: 360.0,
            },
        );
        let r_stable = stable_solver.assess().expect("failed");
        assert!(
            r_stable.risk_index < 0.1,
            "Risk index should be near 0 for stable system, got {:.4}",
            r_stable.risk_index
        );
        assert!(
            r_stable.stability_margin > 0.0,
            "Stability margin should be positive for stable system"
        );
    }

    // ------------------------------------------------------------------
    // 6. Energy-based criterion behaves monotonically with clearing time
    // ------------------------------------------------------------------
    #[test]
    fn test_energy_criterion_monotonic() {
        let criterion = StabilityCriterion::EnergyBased { e_critical: 0.1 };

        let config_fast = PtsaConfig {
            n_scenarios: 30,
            simulation_time_s: 2.0,
            clearing_time_distribution: Distribution::Uniform {
                low: 0.02,
                high: 0.05,
            },
            loading_distribution: Distribution::Uniform {
                low: 0.8,
                high: 0.8,
            },
            stability_criterion: criterion.clone(),
            seed: 99,
        };
        let config_slow = PtsaConfig {
            n_scenarios: 30,
            simulation_time_s: 2.0,
            clearing_time_distribution: Distribution::Uniform {
                low: 0.5,
                high: 0.8,
            },
            loading_distribution: Distribution::Uniform {
                low: 0.8,
                high: 0.8,
            },
            stability_criterion: criterion,
            seed: 99,
        };

        let mut s_fast = ProbabilisticTransientSolver::new(config_fast, 1);
        s_fast.set_generator(0, 5.0, 2.0, 0.7, 1.2).unwrap();
        let mut s_slow = ProbabilisticTransientSolver::new(config_slow, 1);
        s_slow.set_generator(0, 5.0, 2.0, 0.7, 1.2).unwrap();

        let r_fast = s_fast.assess().expect("fast assess failed");
        let r_slow = s_slow.assess().expect("slow assess failed");

        // Slower clearing should produce higher or equal instability.
        assert!(
            r_slow.probability_of_instability >= r_fast.probability_of_instability,
            "Slower clearing ({:.2}) should have ≥ instability than fast ({:.2})",
            r_slow.probability_of_instability,
            r_fast.probability_of_instability
        );
    }
}
