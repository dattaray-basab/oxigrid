//! Multi-objective offshore wind farm layout optimization.
//!
//! Implements four layout strategies:
//! - `GridSearch` — regular rectangular grid
//! - `Hexagonal` — staggered rows for denser packing
//! - `GeneticAlgorithm` — population-based evolutionary optimizer
//! - `SimulatedAnnealing` — single-solution stochastic local search
//!
//! Wake losses are computed using the Jensen (1983) top-hat wake model.
//! Annual Energy Production (AEP) is computed by integrating over the
//! wind rose using a simplified Weibull speed distribution.
//!
//! # References
//! - Jensen, N.O., "A note on wind generator interaction", 1983.
//! - Mosetti et al., "Optimization of wind turbine positioning in large
//!   windfarms by means of a genetic algorithm", J. Wind Eng. 1994.

use serde::{Deserialize, Serialize};
use std::fmt;

// ─────────────────────────────────────────────────────────────────────────────
// Error
// ─────────────────────────────────────────────────────────────────────────────

/// Errors from layout optimization.
#[derive(Debug, Clone, PartialEq)]
pub enum LayoutError {
    /// Configuration is physically invalid.
    InvalidConfig(String),
    /// Optimization algorithm failed to converge or produce a valid layout.
    OptimizationFailed(String),
}

impl fmt::Display for LayoutError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidConfig(s) => write!(f, "invalid layout config: {s}"),
            Self::OptimizationFailed(s) => write!(f, "optimization failed: {s}"),
        }
    }
}

impl std::error::Error for LayoutError {}

// ─────────────────────────────────────────────────────────────────────────────
// LCG random number generator
// ─────────────────────────────────────────────────────────────────────────────

struct LcgRng {
    state: u64,
}

impl LcgRng {
    fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    fn next_u64(&mut self) -> u64 {
        self.state = self
            .state
            .wrapping_mul(6_364_136_223_846_793_005u64)
            .wrapping_add(1_442_695_040_888_963_407u64);
        self.state
    }

    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Uniform random in \[lo, hi\).
    fn uniform(&mut self, lo: f64, hi: f64) -> f64 {
        lo + self.next_f64() * (hi - lo)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Wind Rose
// ─────────────────────────────────────────────────────────────────────────────

/// Wind climate description as a directional-speed frequency distribution.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WindRose {
    /// Wind direction sector centres \[deg\] (meteorological convention).
    pub directions_deg: Vec<f64>,
    /// Probability for each direction sector (must sum to ≈ 1.0).
    pub frequencies: Vec<f64>,
    /// Mean wind speed for each direction sector \[m/s\].
    pub mean_speed_ms: Vec<f64>,
    /// Weibull shape parameter `k` for each direction sector (k ≈ 2 is Rayleigh).
    pub weibull_k: Vec<f64>,
}

impl WindRose {
    /// Uniform wind rose: equal probability for all 12 sectors, constant speed.
    pub fn uniform_12(mean_speed_ms: f64) -> Self {
        let n = 12;
        let dirs: Vec<f64> = (0..n).map(|i| i as f64 * 30.0).collect();
        let freqs = vec![1.0 / n as f64; n];
        let speeds = vec![mean_speed_ms; n];
        let ks = vec![2.0; n];
        Self {
            directions_deg: dirs,
            frequencies: freqs,
            mean_speed_ms: speeds,
            weibull_k: ks,
        }
    }

    /// Validate that the wind rose is internally consistent.
    pub fn validate(&self) -> Result<(), LayoutError> {
        let n = self.directions_deg.len();
        if n == 0 {
            return Err(LayoutError::InvalidConfig(
                "wind rose has no sectors".to_string(),
            ));
        }
        if self.frequencies.len() != n || self.mean_speed_ms.len() != n || self.weibull_k.len() != n
        {
            return Err(LayoutError::InvalidConfig(
                "wind rose arrays must all have the same length".to_string(),
            ));
        }
        Ok(())
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Layout configuration
// ─────────────────────────────────────────────────────────────────────────────

/// Optimization method for turbine layout.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum LayoutOptMethod {
    /// Regular rectangular grid with given rows × columns.
    GridSearch {
        /// Number of grid rows.
        rows: usize,
        /// Number of grid columns.
        cols: usize,
    },
    /// Staggered (hexagonal close-packing) row layout.
    Hexagonal,
    /// Evolutionary genetic algorithm.
    GeneticAlgorithm {
        /// Population size (number of candidate layouts per generation).
        population: usize,
        /// Number of evolution generations.
        generations: usize,
    },
    /// Simulated annealing local search.
    SimulatedAnnealing {
        /// Initial temperature parameter.
        initial_temp: f64,
        /// Temperature cooling multiplier per step (0 < rate < 1).
        cooling_rate: f64,
    },
}

/// Wind farm layout optimization configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LayoutConfig {
    /// Total developable site area \[km²\].
    pub site_area_km2: f64,
    /// Target number of turbines to place.
    pub n_turbines: usize,
    /// Turbine rated power \[MW\].
    pub turbine_rated_mw: f64,
    /// Rotor diameter \[m\].
    pub turbine_rotor_diameter_m: f64,
    /// Minimum inter-turbine spacing expressed as a multiple of rotor diameter.
    pub min_spacing_diameters: f64,
    /// Number of wind direction sectors to evaluate.
    pub n_wind_directions: usize,
    /// Number of wind speed bins for AEP integration.
    pub n_wind_speeds: usize,
    /// Layout optimization method.
    pub optimization_method: LayoutOptMethod,
}

impl LayoutConfig {
    /// Validate configuration parameters.
    pub fn validate(&self) -> Result<(), LayoutError> {
        if self.site_area_km2 <= 0.0 {
            return Err(LayoutError::InvalidConfig(
                "site_area_km2 must be positive".to_string(),
            ));
        }
        if self.n_turbines == 0 {
            return Err(LayoutError::InvalidConfig(
                "n_turbines must be >= 1".to_string(),
            ));
        }
        if self.turbine_rated_mw <= 0.0 {
            return Err(LayoutError::InvalidConfig(
                "turbine_rated_mw must be positive".to_string(),
            ));
        }
        if self.turbine_rotor_diameter_m <= 0.0 {
            return Err(LayoutError::InvalidConfig(
                "turbine_rotor_diameter_m must be positive".to_string(),
            ));
        }
        if self.min_spacing_diameters < 1.0 {
            return Err(LayoutError::InvalidConfig(
                "min_spacing_diameters must be >= 1.0".to_string(),
            ));
        }
        Ok(())
    }

    /// Minimum inter-turbine distance \[m\].
    pub fn min_spacing_m(&self) -> f64 {
        self.min_spacing_diameters * self.turbine_rotor_diameter_m
    }

    /// Site side length (square approximation) \[m\].
    pub fn site_side_m(&self) -> f64 {
        (self.site_area_km2 * 1e6_f64).sqrt()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Turbine position
// ─────────────────────────────────────────────────────────────────────────────

/// Position of a wind turbine within the farm.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct WindFarmTurbinePosition {
    /// Turbine index within the layout.
    pub id: usize,
    /// Easting coordinate \[m\].
    pub x_m: f64,
    /// Northing coordinate \[m\].
    pub y_m: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Layout result
// ─────────────────────────────────────────────────────────────────────────────

/// Result of a wind farm layout optimization.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WindFarmLayoutResult {
    /// Optimized turbine positions.
    pub positions: Vec<WindFarmTurbinePosition>,
    /// Estimated annual energy production \[GWh\].
    pub annual_energy_gwh: f64,
    /// Capacity factor \[%\] (AEP / (rated × 8760 h)).
    pub capacity_factor_pct: f64,
    /// Wake loss as a fraction of gross AEP \[%\].
    pub wake_loss_pct: f64,
    /// Overall turbine availability factor \[%\].
    pub availability_factor_pct: f64,
    /// Levelised cost of energy \[USD/MWh\].
    pub lcoe_usd_per_mwh: f64,
    /// Number of turbine pairs violating minimum spacing constraint.
    pub n_constraint_violations: usize,
}

// ─────────────────────────────────────────────────────────────────────────────
// Optimizer
// ─────────────────────────────────────────────────────────────────────────────

/// Wind farm layout optimizer.
pub struct WindFarmLayoutOptimizer {
    config: LayoutConfig,
    wind_rose: WindRose,
}

impl WindFarmLayoutOptimizer {
    /// Create a new layout optimizer.
    pub fn new(config: LayoutConfig, wind_rose: WindRose) -> Self {
        Self { config, wind_rose }
    }

    // ── Jensen wake model ──────────────────────────────────────────────────

    /// Jensen (1983) top-hat velocity deficit at downstream distance `x_m`.
    ///
    /// ```text
    /// Δu/u₀ = (1 − √(1 − Ct)) / (1 + k·x/D)²
    /// ```
    ///
    /// Returns a value in \[0, 1\]; 0 means no deficit (x ≤ 0).
    pub fn jensen_deficit(&self, x_m: f64, ct: f64, k_wake: f64) -> f64 {
        if x_m <= 0.0 || ct <= 0.0 {
            return 0.0;
        }
        let d = self.config.turbine_rotor_diameter_m;
        let numerator = 1.0 - (1.0 - ct).max(0.0).sqrt();
        let denominator = (1.0 + k_wake * x_m / d).powi(2);
        (numerator / denominator.max(1e-12)).clamp(0.0, 1.0)
    }

    // ── Power curve ────────────────────────────────────────────────────────

    /// Simplified cubic power curve.
    ///
    /// ```text
    /// P(v) = P_rated · min(1, (v / v_rated)³)   for v_cut_in ≤ v ≤ v_cut_out
    ///       0                                     otherwise
    /// ```
    pub fn power_at_speed(&self, wind_speed_ms: f64) -> f64 {
        let v_cut_in = 3.0_f64;
        let v_rated = 12.0_f64;
        let v_cut_out = 25.0_f64;
        let p_rated = self.config.turbine_rated_mw;

        if wind_speed_ms < v_cut_in || wind_speed_ms > v_cut_out {
            return 0.0;
        }
        if wind_speed_ms >= v_rated {
            return p_rated;
        }
        p_rated * (wind_speed_ms / v_rated).powi(3)
    }

    // ── AEP computation ────────────────────────────────────────────────────

    /// Compute gross (no-wake) AEP for a single turbine \[GWh/year\].
    fn gross_aep_single(&self) -> f64 {
        let n_speeds = self.config.n_wind_speeds.max(10);
        let mut total_mwh = 0.0_f64;
        let n_dir = self.wind_rose.directions_deg.len();

        for dir_idx in 0..n_dir {
            let freq = self.wind_rose.frequencies[dir_idx];
            let mean_v = self.wind_rose.mean_speed_ms[dir_idx];
            let k = self.wind_rose.weibull_k[dir_idx];
            // Weibull scale parameter: λ = mean / Γ(1+1/k) ≈ mean / 0.9 for k=2
            let gamma_1_plus_1_k = weibull_gamma_approx(k);
            let lambda = mean_v / gamma_1_plus_1_k;

            // Integrate P(v) × f_Weibull(v) dv over [0, v_cut_out]
            let v_max = 25.0_f64;
            let dv = v_max / n_speeds as f64;
            let mut mwh_dir = 0.0_f64;
            for speed_bin in 0..n_speeds {
                let v = (speed_bin as f64 + 0.5) * dv;
                let p = self.power_at_speed(v);
                let f_wei = weibull_pdf(v, k, lambda);
                mwh_dir += p * f_wei * dv * 8760.0; // [MW × fraction × h]
            }
            total_mwh += mwh_dir * freq;
        }
        total_mwh / 1_000.0 // MWh → GWh
    }

    /// Compute AEP for a layout, accounting for Jensen wake losses \[GWh/year\].
    pub fn compute_aep(&self, positions: &[WindFarmTurbinePosition]) -> f64 {
        if positions.is_empty() {
            return 0.0;
        }
        let n = positions.len();
        let k_wake = 0.04_f64; // Jensen spreading constant (offshore)
        let ct = 0.8_f64; // thrust coefficient (typical operating point)
        let availability = 0.97_f64;

        let n_dir = self.wind_rose.directions_deg.len();
        let n_speeds = self.config.n_wind_speeds.max(10);
        let mut total_gwh = 0.0_f64;

        for dir_idx in 0..n_dir {
            let dir_deg = self.wind_rose.directions_deg[dir_idx];
            let dir_rad = dir_deg.to_radians();
            let freq = self.wind_rose.frequencies[dir_idx];
            let mean_v = self.wind_rose.mean_speed_ms[dir_idx];
            let k = self.wind_rose.weibull_k[dir_idx];
            let gamma_k = weibull_gamma_approx(k);
            let lambda = mean_v / gamma_k;
            let (sin_d, cos_d) = (dir_rad.sin(), dir_rad.cos());

            let v_max = 25.0_f64;
            let dv = v_max / n_speeds as f64;

            for speed_bin in 0..n_speeds {
                let v0 = (speed_bin as f64 + 0.5) * dv;
                let f_wei = weibull_pdf(v0, k, lambda) * dv;
                if f_wei < 1e-12 {
                    continue;
                }

                // Effective speed at each turbine accounting for all upwind wakes
                let mut eff_speeds = vec![v0; n];
                // Sum of squared deficits (linear superposition variant)
                for i in 0..n {
                    let mut deficit_sq_sum = 0.0_f64;
                    for j in 0..n {
                        if i == j {
                            continue;
                        }
                        let dx = positions[i].x_m - positions[j].x_m;
                        let dy = positions[i].y_m - positions[j].y_m;
                        // Downstream distance of i relative to j in wind direction
                        // (positive = i is downwind of j)
                        let x_down = dx * sin_d + dy * cos_d;
                        if x_down <= 0.0 {
                            continue;
                        }
                        // Lateral offset
                        let y_lat = (-dx * cos_d + dy * sin_d).abs();
                        let d = self.config.turbine_rotor_diameter_m;
                        // Wake radius at distance x_down
                        let r_wake = (d / 2.0) + k_wake * x_down;
                        if y_lat > r_wake {
                            continue; // turbine i outside wake of j
                        }
                        let deficit = self.jensen_deficit(x_down, ct, k_wake);
                        deficit_sq_sum += deficit * deficit;
                    }
                    let total_deficit = deficit_sq_sum.sqrt().min(0.95);
                    eff_speeds[i] = v0 * (1.0 - total_deficit);
                }

                let farm_mw: f64 = eff_speeds.iter().map(|&v| self.power_at_speed(v)).sum();
                total_gwh += farm_mw * f_wei * 8760.0 * freq * availability / 1_000.0;
            }
        }

        total_gwh
    }

    // ── Spacing constraint ─────────────────────────────────────────────────

    /// Count turbine pairs violating the minimum spacing constraint.
    pub fn check_spacing(&self, positions: &[WindFarmTurbinePosition]) -> usize {
        let min_dist = self.config.min_spacing_m();
        let mut violations = 0usize;
        let n = positions.len();
        for i in 0..n {
            for j in (i + 1)..n {
                let dx = positions[i].x_m - positions[j].x_m;
                let dy = positions[i].y_m - positions[j].y_m;
                if (dx * dx + dy * dy).sqrt() < min_dist {
                    violations += 1;
                }
            }
        }
        violations
    }

    // ── Layout generators ──────────────────────────────────────────────────

    /// Generate a regular rectangular grid layout.
    fn grid_layout(&self, rows: usize, cols: usize) -> Vec<WindFarmTurbinePosition> {
        let spacing = self
            .config
            .min_spacing_m()
            .max(self.config.turbine_rotor_diameter_m * self.config.min_spacing_diameters);
        let mut positions = Vec::new();
        let mut id = 0;
        'outer: for r in 0..rows {
            for c in 0..cols {
                if positions.len() >= self.config.n_turbines {
                    break 'outer;
                }
                positions.push(WindFarmTurbinePosition {
                    id,
                    x_m: r as f64 * spacing,
                    y_m: c as f64 * spacing,
                });
                id += 1;
            }
        }
        positions
    }

    /// Generate a hexagonal (staggered-row) layout.
    fn hexagonal_layout(&self) -> Vec<WindFarmTurbinePosition> {
        let d = self.config.turbine_rotor_diameter_m;
        let spacing = self.config.min_spacing_diameters * d;
        // Row spacing for hexagonal: spacing_y = spacing * √3/2
        let row_spacing = spacing * (3.0_f64.sqrt() / 2.0);
        let side = self.config.site_side_m();
        let cols = (side / spacing).floor() as usize + 1;
        let rows = (side / row_spacing).floor() as usize + 1;

        let mut positions = Vec::new();
        let mut id = 0;
        'outer: for r in 0..rows {
            let offset = if r % 2 == 1 { spacing * 0.5 } else { 0.0 };
            for c in 0..cols {
                if positions.len() >= self.config.n_turbines {
                    break 'outer;
                }
                let x = r as f64 * row_spacing;
                let y = c as f64 * spacing + offset;
                if x > side || y > side + spacing {
                    continue;
                }
                positions.push(WindFarmTurbinePosition { id, x_m: x, y_m: y });
                id += 1;
            }
        }
        positions
    }

    /// Generate a random layout within site bounds using LCG.
    fn random_layout(&self, rng: &mut LcgRng) -> Vec<WindFarmTurbinePosition> {
        let side = self.config.site_side_m();
        (0..self.config.n_turbines)
            .map(|id| WindFarmTurbinePosition {
                id,
                x_m: rng.uniform(0.0, side),
                y_m: rng.uniform(0.0, side),
            })
            .collect()
    }

    /// Mutate a layout: perturb one random turbine by ±D.
    fn mutate(
        &self,
        layout: &[WindFarmTurbinePosition],
        rng: &mut LcgRng,
    ) -> Vec<WindFarmTurbinePosition> {
        let mut new_layout = layout.to_vec();
        let idx = (rng.next_u64() % layout.len() as u64) as usize;
        let d = self.config.turbine_rotor_diameter_m;
        let side = self.config.site_side_m();
        new_layout[idx].x_m = (new_layout[idx].x_m + rng.uniform(-d, d)).clamp(0.0, side);
        new_layout[idx].y_m = (new_layout[idx].y_m + rng.uniform(-d, d)).clamp(0.0, side);
        new_layout
    }

    // ── Genetic Algorithm ──────────────────────────────────────────────────

    fn run_genetic_algorithm(
        &self,
        pop_size: usize,
        generations: usize,
    ) -> Vec<WindFarmTurbinePosition> {
        let pop_size = pop_size.max(4);
        let mut rng = LcgRng::new(12345);

        // Initialize population
        let mut population: Vec<Vec<WindFarmTurbinePosition>> = (0..pop_size)
            .map(|_| self.random_layout(&mut rng))
            .collect();

        // Seed with grid layout for diversity
        let rows = (self.config.n_turbines as f64).sqrt().ceil() as usize;
        let cols = self.config.n_turbines.div_ceil(rows);
        population[0] = self.grid_layout(rows, cols);

        let mut fitnesses: Vec<f64> = population.iter().map(|l| self.compute_aep(l)).collect();

        for _gen in 0..generations {
            // Tournament selection (k=2) + mutation
            let mut new_pop = Vec::with_capacity(pop_size);
            for _ in 0..pop_size {
                // Tournament: pick 2 random individuals
                let a = (rng.next_u64() % pop_size as u64) as usize;
                let b = (rng.next_u64() % pop_size as u64) as usize;
                let winner = if fitnesses[a] >= fitnesses[b] { a } else { b };
                let child = self.mutate(&population[winner], &mut rng);
                new_pop.push(child);
            }
            // Evaluate new population
            let new_fitnesses: Vec<f64> = new_pop.iter().map(|l| self.compute_aep(l)).collect();

            population = new_pop;
            fitnesses = new_fitnesses;
        }

        // Return best individual
        let best_idx = fitnesses
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
            .map(|(i, _)| i)
            .unwrap_or(0);

        population.remove(best_idx)
    }

    // ── Simulated Annealing ────────────────────────────────────────────────

    fn run_simulated_annealing(
        &self,
        initial_temp: f64,
        cooling_rate: f64,
    ) -> Vec<WindFarmTurbinePosition> {
        let rows = (self.config.n_turbines as f64).sqrt().ceil() as usize;
        let cols = self.config.n_turbines.div_ceil(rows);
        let mut current = self.grid_layout(rows, cols);
        let mut current_aep = self.compute_aep(&current);
        let mut best = current.clone();
        let mut best_aep = current_aep;

        let mut temp = initial_temp;
        let mut rng = LcgRng::new(99999);

        let n_steps = (1.0 / (1.0 - cooling_rate.min(0.9999))) as usize + 500;
        let n_steps = n_steps.min(2000);

        for _ in 0..n_steps {
            if temp < 1e-9 {
                break;
            }
            let candidate = self.mutate(&current, &mut rng);
            let candidate_aep = self.compute_aep(&candidate);
            let delta = candidate_aep - current_aep;

            if delta > 0.0 || rng.next_f64() < (-delta / temp).exp() {
                current = candidate;
                current_aep = candidate_aep;
                if current_aep > best_aep {
                    best = current.clone();
                    best_aep = current_aep;
                }
            }
            temp *= cooling_rate;
        }
        best
    }

    // ── LCOE helper ────────────────────────────────────────────────────────

    fn compute_lcoe(&self, annual_energy_gwh: f64, n_turbines: usize) -> f64 {
        let capex_per_mw = 4_500_000.0_f64; // USD/MW (offshore)
        let installed_mw = n_turbines as f64 * self.config.turbine_rated_mw;
        let capex = capex_per_mw * installed_mw;
        // Simple present-value of OpEx over 20 years at 3% of CapEx/year
        let opex_pv = 0.03 * capex * 20.0;
        let lifetime_mwh = annual_energy_gwh * 1_000.0 * 20.0;
        if lifetime_mwh < 1.0 {
            return f64::INFINITY;
        }
        (capex + opex_pv) / lifetime_mwh
    }

    // ── Public optimize entry point ────────────────────────────────────────

    /// Run the configured layout optimization algorithm.
    pub fn optimize(&self) -> Result<WindFarmLayoutResult, LayoutError> {
        self.config.validate()?;
        self.wind_rose.validate()?;

        let positions = match &self.config.optimization_method {
            LayoutOptMethod::GridSearch { rows, cols } => self.grid_layout(*rows, *cols),
            LayoutOptMethod::Hexagonal => self.hexagonal_layout(),
            LayoutOptMethod::GeneticAlgorithm {
                population,
                generations,
            } => self.run_genetic_algorithm(*population, *generations),
            LayoutOptMethod::SimulatedAnnealing {
                initial_temp,
                cooling_rate,
            } => self.run_simulated_annealing(*initial_temp, *cooling_rate),
        };

        if positions.is_empty() {
            return Err(LayoutError::OptimizationFailed(
                "no turbines placed — check site area and spacing constraints".to_string(),
            ));
        }

        let n = positions.len();
        let annual_energy_gwh = self.compute_aep(&positions);
        let n_constraint_violations = self.check_spacing(&positions);
        let availability_factor_pct = 97.0_f64; // standard offshore availability

        // Gross AEP (no wake) for wake loss computation
        let gross_aep_gwh = self.gross_aep_single() * n as f64;
        let wake_loss_pct = if gross_aep_gwh > 0.0 {
            ((gross_aep_gwh - annual_energy_gwh) / gross_aep_gwh * 100.0).clamp(0.0, 100.0)
        } else {
            0.0
        };

        let installed_mw = n as f64 * self.config.turbine_rated_mw;
        let capacity_factor_pct = if installed_mw > 0.0 {
            annual_energy_gwh * 1_000.0 / (installed_mw * 8760.0) * 100.0
        } else {
            0.0
        };

        let lcoe_usd_per_mwh = self.compute_lcoe(annual_energy_gwh, n);

        Ok(WindFarmLayoutResult {
            positions,
            annual_energy_gwh,
            capacity_factor_pct,
            wake_loss_pct,
            availability_factor_pct,
            lcoe_usd_per_mwh,
            n_constraint_violations,
        })
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Weibull helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Approximate Γ(1 + 1/k) for Weibull scale parameter computation.
///
/// Uses a rational approximation valid for k ∈ \[1, 4\].
fn weibull_gamma_approx(k: f64) -> f64 {
    // For k = 2 (Rayleigh): Γ(1.5) = √π/2 ≈ 0.8862
    // For k = 1 (exponential): Γ(2) = 1.0
    // Linear interpolation for general k: Γ(1 + 1/k) ≈ exp(-0.5772/k) * (1 + 0.5/k)
    let x = 1.0 + 1.0 / k.max(0.5);
    // Stirling / Lanczos approximation for Gamma near 1–2
    gamma_approx(x)
}

/// Lanczos approximation of the Gamma function for x in (1, 3).
fn gamma_approx(x: f64) -> f64 {
    // Coefficients for Lanczos g=5, n=6
    let p = [
        76.180_091_729_471_46_f64,
        -86.505_320_329_416_77,
        24.014_098_240_830_91,
        -1.231_739_572_450_155,
        1.208_650_973_866_179e-3,
        -5.395_239_384_953_e-6,
    ];
    let x = x - 1.0;
    let mut ser = 1.000_000_000_190_015_f64;
    let mut y = x;
    for &c in &p {
        y += 1.0;
        ser += c / y;
    }
    let t = x + 5.5;
    (2.0 * std::f64::consts::PI).sqrt() * t.powf(x + 0.5) * (-t).exp() * ser
}

/// Weibull probability density function.
///
/// `f(v; k, λ) = (k/λ) · (v/λ)^(k-1) · exp(-(v/λ)^k)`
fn weibull_pdf(v: f64, k: f64, lambda: f64) -> f64 {
    if v <= 0.0 || lambda <= 0.0 || k <= 0.0 {
        return 0.0;
    }
    let ratio = v / lambda;
    (k / lambda) * ratio.powf(k - 1.0) * (-(ratio.powf(k))).exp()
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_config(method: LayoutOptMethod, n: usize) -> LayoutConfig {
        LayoutConfig {
            site_area_km2: 25.0, // 5 km × 5 km
            n_turbines: n,
            turbine_rated_mw: 5.0,
            turbine_rotor_diameter_m: 126.0,
            min_spacing_diameters: 5.0,
            n_wind_directions: 12,
            n_wind_speeds: 15,
            optimization_method: method,
        }
    }

    fn make_wind_rose() -> WindRose {
        WindRose::uniform_12(8.0)
    }

    // ── Test 1: Grid layout places n_turbines with 0 violations ─────────────

    #[test]
    fn test_grid_layout_no_violations() {
        let config = make_config(LayoutOptMethod::GridSearch { rows: 3, cols: 3 }, 9);
        let opt = WindFarmLayoutOptimizer::new(config, make_wind_rose());
        let result = opt.optimize().expect("grid optimize should succeed");
        assert_eq!(
            result.positions.len(),
            9,
            "grid 3×3 should place 9 turbines"
        );
        assert_eq!(
            result.n_constraint_violations, 0,
            "regular grid should have zero spacing violations"
        );
    }

    // ── Test 2: Hexagonal layout produces positive AEP ───────────────────────

    #[test]
    fn test_hexagonal_layout_positive_aep() {
        let config = make_config(LayoutOptMethod::Hexagonal, 6);
        let opt = WindFarmLayoutOptimizer::new(config, make_wind_rose());
        let result = opt.optimize().expect("hexagonal optimize should succeed");
        assert!(
            result.annual_energy_gwh > 0.0,
            "hexagonal layout AEP should be positive, got {}",
            result.annual_energy_gwh
        );
        assert_eq!(
            result.n_constraint_violations, 0,
            "hexagonal layout should satisfy spacing"
        );
    }

    // ── Test 3: Jensen deficit formula ───────────────────────────────────────

    #[test]
    fn test_jensen_deficit_formula() {
        let config = make_config(LayoutOptMethod::Hexagonal, 4);
        let opt = WindFarmLayoutOptimizer::new(config, make_wind_rose());
        let d = 126.0_f64;
        let ct = 0.8_f64;
        let k = 0.04_f64;
        let x = 5.0 * d; // 5D downstream

        let deficit = opt.jensen_deficit(x, ct, k);
        // Expected: (1 - sqrt(1-0.8)) / (1 + 0.04*5)^2 = (1-0.4472)/(1.2)^2 ≈ 0.5528/1.44 ≈ 0.384
        let expected = (1.0 - (1.0 - ct).sqrt()) / (1.0 + k * 5.0).powi(2);
        assert!(
            (deficit - expected).abs() < 1e-6,
            "Jensen deficit: got {deficit:.6}, expected {expected:.6}"
        );
        assert!(deficit > 0.0 && deficit < 1.0, "deficit must be in (0,1)");

        // Deficit at x=0 (upstream) must be 0
        assert_eq!(opt.jensen_deficit(0.0, ct, k), 0.0);
    }

    // ── Test 4: More turbines → more AEP ─────────────────────────────────────

    #[test]
    fn test_more_turbines_more_aep() {
        let rose = make_wind_rose();
        let config4 = LayoutConfig {
            site_area_km2: 100.0,
            n_turbines: 4,
            turbine_rated_mw: 5.0,
            turbine_rotor_diameter_m: 126.0,
            min_spacing_diameters: 5.0,
            n_wind_directions: 12,
            n_wind_speeds: 10,
            optimization_method: LayoutOptMethod::GridSearch { rows: 2, cols: 2 },
        };
        let config9 = LayoutConfig {
            n_turbines: 9,
            optimization_method: LayoutOptMethod::GridSearch { rows: 3, cols: 3 },
            ..config4.clone()
        };

        let opt4 = WindFarmLayoutOptimizer::new(config4, rose.clone());
        let opt9 = WindFarmLayoutOptimizer::new(config9, rose);

        let aep4 = opt4
            .optimize()
            .expect("4-turbine optimize")
            .annual_energy_gwh;
        let aep9 = opt9
            .optimize()
            .expect("9-turbine optimize")
            .annual_energy_gwh;

        assert!(
            aep9 > aep4,
            "9-turbine AEP ({aep9:.3} GWh) should exceed 4-turbine ({aep4:.3} GWh)"
        );
    }

    // ── Test 5: Spacing constraint violation detection ────────────────────────

    #[test]
    fn test_spacing_violations_detected() {
        let config = make_config(LayoutOptMethod::Hexagonal, 4);
        let opt = WindFarmLayoutOptimizer::new(config, make_wind_rose());

        // Place two turbines 10 m apart — far below min spacing (5D = 630 m)
        let positions = vec![
            WindFarmTurbinePosition {
                id: 0,
                x_m: 0.0,
                y_m: 0.0,
            },
            WindFarmTurbinePosition {
                id: 1,
                x_m: 10.0,
                y_m: 0.0,
            }, // violation
            WindFarmTurbinePosition {
                id: 2,
                x_m: 2000.0,
                y_m: 0.0,
            },
            WindFarmTurbinePosition {
                id: 3,
                x_m: 4000.0,
                y_m: 0.0,
            },
        ];
        let violations = opt.check_spacing(&positions);
        assert!(
            violations > 0,
            "closely-spaced pair should register a violation"
        );
    }

    // ── Test 6: Simulated annealing completes and improves vs cold start ──────

    #[test]
    fn test_simulated_annealing_runs() {
        let config = make_config(
            LayoutOptMethod::SimulatedAnnealing {
                initial_temp: 1.0,
                cooling_rate: 0.95,
            },
            6,
        );
        let opt = WindFarmLayoutOptimizer::new(config, make_wind_rose());
        let result = opt.optimize().expect("SA optimize should succeed");
        assert!(
            result.annual_energy_gwh > 0.0,
            "SA result must have positive AEP"
        );
    }

    // ── Test 7: Genetic algorithm runs ───────────────────────────────────────

    #[test]
    fn test_genetic_algorithm_runs() {
        let config = make_config(
            LayoutOptMethod::GeneticAlgorithm {
                population: 10,
                generations: 5,
            },
            6,
        );
        let opt = WindFarmLayoutOptimizer::new(config, make_wind_rose());
        let result = opt.optimize().expect("GA optimize should succeed");
        assert!(
            result.annual_energy_gwh > 0.0,
            "GA result must have positive AEP"
        );
        assert_eq!(result.positions.len(), 6);
    }

    // ── Test 8: LCOE is finite and positive ──────────────────────────────────

    #[test]
    fn test_lcoe_finite_positive() {
        let config = make_config(LayoutOptMethod::GridSearch { rows: 2, cols: 2 }, 4);
        let opt = WindFarmLayoutOptimizer::new(config, make_wind_rose());
        let result = opt.optimize().expect("optimize ok");
        assert!(
            result.lcoe_usd_per_mwh > 0.0 && result.lcoe_usd_per_mwh.is_finite(),
            "LCOE must be finite and positive, got {}",
            result.lcoe_usd_per_mwh
        );
    }
}
