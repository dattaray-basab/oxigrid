//! Microgrid Optimal Sizing.
//!
//! Determines the lowest-LCOE (Levelised Cost of Energy) combination of
//! microgrid components (solar PV, wind, diesel, battery storage, fuel cell)
//! that satisfies a user-defined reliability target expressed as Loss of Power
//! Supply Probability (LPSP).
//!
//! # Method
//!
//! 1. **Screening** — eliminate combinations that are obviously under-sized.
//! 2. **LCG Monte Carlo** — for each candidate combination, sample load and
//!    resource variability over `n_monte_carlo` trials and simulate 8760 hours.
//! 3. **LPSP** — compute the fraction of unmet energy demand.
//! 4. **LCOE** — for feasible combinations (LPSP ≤ 1 − reliability_target),
//!    compute the net-present-value-weighted levelised cost.
//! 5. **Selection** — return the minimum-LCOE feasible combination.
//!
//! # Random Number Generation
//!
//! All stochastic draws use a 64-bit Linear Congruential Generator (LCG):
//! - multiplier = `6364136223846793005`
//! - addend = `1442695040888963407`

use thiserror::Error;

// ── Error ─────────────────────────────────────────────────────────────────────

/// Errors produced by the microgrid sizer.
#[derive(Debug, Error)]
pub enum SizingError {
    /// No component options were provided.
    #[error("no component options supplied")]
    NoComponents,

    /// The load profile is invalid.
    #[error("invalid load profile: {0}")]
    InvalidLoad(String),

    /// No combination satisfies the reliability target.
    #[error("no feasible configuration found for the given reliability target")]
    NoFeasibleConfig,

    /// A numerical computation failed.
    #[error("computation error: {0}")]
    ComputationError(String),
}

// ── LCG ───────────────────────────────────────────────────────────────────────

const LCG_MULT: u64 = 6364136223846793005;
const LCG_ADD: u64 = 1442695040888963407;

struct Lcg(u64);

impl Lcg {
    fn new(seed: u64) -> Self {
        Self(seed)
    }

    /// Advance state and return next pseudo-random `u64`.
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_mul(LCG_MULT).wrapping_add(LCG_ADD);
        self.0
    }

    /// Uniform sample in \[0, 1).
    fn next_f64(&mut self) -> f64 {
        self.next() as f64 / u64::MAX as f64
    }
}

// ── Public types ───────────────────────────────────────────────────────────────

/// Global configuration for the microgrid sizer.
#[derive(Debug, Clone)]
pub struct MicrogridSizingConfig {
    /// Project economic lifetime \[years\].
    pub project_lifetime_years: usize,
    /// Discount rate (e.g. 0.08 for 8 %).
    pub discount_rate: f64,
    /// Minimum reliability (e.g. 0.9999 = four nines). LPSP ≤ 1 − target.
    pub reliability_target: f64,
    /// `true` for grid-tied systems (partial reliability from utility).
    pub grid_connected: bool,
    /// Number of Monte Carlo trials (default 500).
    pub n_monte_carlo: usize,
}

impl Default for MicrogridSizingConfig {
    fn default() -> Self {
        Self {
            project_lifetime_years: 20,
            discount_rate: 0.08,
            reliability_target: 0.9999,
            grid_connected: false,
            n_monte_carlo: 500,
        }
    }
}

/// Annual hourly load profile.
#[derive(Debug, Clone)]
pub struct LoadProfile {
    /// Hourly load \[kW\] — typically 8760 values (one per hour of the year).
    pub hourly_load_kw: Vec<f64>,
    /// Peak load \[kW\].
    pub peak_load_kw: f64,
    /// Total annual energy \[kWh\].
    pub annual_energy_kwh: f64,
}

/// A microgrid component type.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ComponentType {
    /// Solar photovoltaic array.
    SolarPv,
    /// Wind turbine.
    WindTurbine,
    /// Diesel generator set.
    DieselGenerator,
    /// Battery energy storage system.
    BatteryStorage,
    /// Hydrogen fuel cell.
    FuelCell,
}

/// Discrete sizing options for one component type.
#[derive(Debug, Clone)]
pub struct ComponentOption {
    /// Component technology.
    pub component_type: ComponentType,
    /// Available sizes \[kW\] (for generators) or \[kWh\] (for storage).
    pub sizes_kw_or_kwh: Vec<f64>,
    /// Capital cost \[USD\] for each size (must parallel `sizes_kw_or_kwh`).
    pub capex_usd_per_unit: Vec<f64>,
    /// Annual O&M cost \[USD/kW/year\] (or per kWh for storage).
    pub opex_usd_per_kw_year: f64,
    /// Component lifetime \[years\].
    pub lifetime_years: usize,
    /// Replacement cost at end of lifetime \[USD\].
    pub replacement_cost_usd: f64,
}

/// Optimal sizing result.
#[derive(Debug, Clone)]
pub struct SizingResult {
    /// Selected component sizes: `(type, size_kw_or_kwh)`.
    pub optimal_sizes: Vec<(ComponentType, f64)>,
    /// Levelised Cost of Energy \[USD/kWh\].
    pub lcoe_usd_per_kwh: f64,
    /// Net present value of the project \[USD\] (negative = cost).
    pub npv_usd: f64,
    /// Achieved reliability (1 − mean LPSP).
    pub reliability_achieved: f64,
    /// Fraction of annual energy supplied by renewables (0–1).
    pub renewable_fraction: f64,
    /// Annual energy curtailed \[kWh\].
    pub annual_curtailment_kwh: f64,
    /// Total capital expenditure \[USD\].
    pub total_capex_usd: f64,
    /// Simple payback period \[years\].
    pub payback_years: f64,
    /// Annual CO₂ reduction compared to full-diesel baseline \[t/year\].
    pub co2_reduction_t_per_year: f64,
}

// ── Sizer ─────────────────────────────────────────────────────────────────────

/// Optimal microgrid sizer using LCG Monte Carlo + LPSP method.
pub struct MicrogridSizer {
    config: MicrogridSizingConfig,
}

impl MicrogridSizer {
    /// Create a new sizer with the given configuration.
    pub fn new(config: MicrogridSizingConfig) -> Self {
        Self { config }
    }

    /// Find the minimum-LCOE configuration that meets the reliability target.
    ///
    /// - `solar_irradiance_kw_per_kw` — hourly capacity factors for solar (0–1).
    /// - `wind_capacity_factor` — hourly capacity factors for wind (0–1).
    pub fn optimize(
        &self,
        load: &LoadProfile,
        solar_irradiance_kw_per_kw: &[f64],
        wind_capacity_factor: &[f64],
        component_options: Vec<ComponentOption>,
    ) -> Result<SizingResult, SizingError> {
        if component_options.is_empty() {
            return Err(SizingError::NoComponents);
        }
        if load.hourly_load_kw.is_empty() {
            return Err(SizingError::InvalidLoad(
                "hourly_load_kw must not be empty".to_string(),
            ));
        }
        if load.annual_energy_kwh <= 0.0 {
            return Err(SizingError::InvalidLoad(
                "annual_energy_kwh must be positive".to_string(),
            ));
        }

        let combinations = self.screen_combinations(&component_options);

        let mut best_lcoe = f64::INFINITY;
        let mut best_result: Option<SizingResult> = None;

        let n_mc = self.config.n_monte_carlo.max(1);
        let irr_len = solar_irradiance_kw_per_kw.len().max(1);
        let wind_len = wind_capacity_factor.len().max(1);
        let load_len = load.hourly_load_kw.len();

        for combo in &combinations {
            // Extract sizes and capex for this combination.
            let sizes: Vec<(ComponentType, f64)> = combo
                .iter()
                .zip(component_options.iter())
                .map(|(&idx, opt)| {
                    let size = opt.sizes_kw_or_kwh.get(idx).copied().unwrap_or(0.0);
                    (opt.component_type.clone(), size)
                })
                .collect();

            let capex_vec: Vec<f64> = combo
                .iter()
                .zip(component_options.iter())
                .map(|(&idx, opt)| opt.capex_usd_per_unit.get(idx).copied().unwrap_or(0.0))
                .collect();

            // Component sizes by type.
            let solar_kw = total_size_of_type(&sizes, &ComponentType::SolarPv);
            let wind_kw = total_size_of_type(&sizes, &ComponentType::WindTurbine);
            let battery_kwh = total_size_of_type(&sizes, &ComponentType::BatteryStorage);
            let diesel_kw = total_size_of_type(&sizes, &ComponentType::DieselGenerator)
                + total_size_of_type(&sizes, &ComponentType::FuelCell);

            // ── Monte Carlo LPSP ──────────────────────────────────────────
            let mut rng = Lcg::new(12345);
            let mut sum_lpsp = 0.0_f64;

            for _ in 0..n_mc {
                let load_scale = 1.0 + (rng.next_f64() - 0.5) * 0.10; // ±5%
                let resource_scale = 0.8 + rng.next_f64() * 0.4; // 0.8–1.2

                let mut unmet_energy = 0.0_f64;
                let mut total_energy = 0.0_f64;

                for h in 0..8760_usize {
                    let load_h = load.hourly_load_kw[h % load_len] * load_scale;
                    total_energy += load_h;

                    let solar_gen =
                        solar_kw * solar_irradiance_kw_per_kw[h % irr_len] * resource_scale;
                    let wind_gen = wind_kw * wind_capacity_factor[h % wind_len] * resource_scale;
                    let renewable_total = solar_gen + wind_gen;

                    // Battery: 50 % DoD, instantaneous dispatch model.
                    let battery_available = battery_kwh * 0.5;
                    let supply = renewable_total + battery_available + diesel_kw;

                    if supply < load_h {
                        unmet_energy += load_h - supply;
                    }
                }

                let lpsp = if total_energy > 1e-9 {
                    (unmet_energy / total_energy).clamp(0.0, 1.0)
                } else {
                    1.0
                };
                sum_lpsp += lpsp;
            }

            let mean_lpsp = sum_lpsp / n_mc as f64;
            let reliability_achieved = 1.0 - mean_lpsp;

            let lpsp_limit = 1.0 - self.config.reliability_target;
            if mean_lpsp > lpsp_limit + 1e-9 {
                continue; // infeasible
            }

            // ── LCOE ─────────────────────────────────────────────────────
            let lcoe = self.compute_lcoe(&sizes, &component_options, load.annual_energy_kwh);

            if lcoe < best_lcoe {
                best_lcoe = lcoe;

                let total_capex: f64 = capex_vec.iter().sum();
                let renewable_fraction = if solar_kw + wind_kw + diesel_kw > 1e-9 {
                    (solar_kw + wind_kw) / (solar_kw + wind_kw + diesel_kw)
                } else {
                    0.0
                };
                let annual_curtailment_kwh =
                    ((solar_kw + wind_kw) * 8760.0 * 0.5 - load.annual_energy_kwh).max(0.0);
                let payback_years = if load.annual_energy_kwh > 1e-9 {
                    total_capex / (load.annual_energy_kwh * 0.15)
                } else {
                    f64::INFINITY
                };
                let co2_reduction_t_per_year = renewable_fraction * load.annual_energy_kwh * 0.0005;

                best_result = Some(SizingResult {
                    optimal_sizes: sizes,
                    lcoe_usd_per_kwh: lcoe,
                    npv_usd: -total_capex,
                    reliability_achieved,
                    renewable_fraction,
                    annual_curtailment_kwh,
                    total_capex_usd: total_capex,
                    payback_years,
                    co2_reduction_t_per_year,
                });
            }
        }

        best_result.ok_or(SizingError::NoFeasibleConfig)
    }

    /// Quick screening: generate all size-index combinations, one per option.
    ///
    /// Options with more than 3 size choices are trimmed to the first 3 to
    /// keep the search tractable without an exhaustive enumeration.
    fn screen_combinations(&self, options: &[ComponentOption]) -> Vec<Vec<usize>> {
        if options.is_empty() {
            return vec![];
        }

        // Cap each option at 3 choices.
        let ranges: Vec<usize> = options
            .iter()
            .map(|o| o.sizes_kw_or_kwh.len().clamp(1, 3))
            .collect();

        // Cartesian product via index counter.
        let total: usize = ranges.iter().product();
        let mut combos = Vec::with_capacity(total);

        for flat in 0..total {
            let mut idx = flat;
            let combo: Vec<usize> = ranges
                .iter()
                .map(|&r| {
                    let i = idx % r;
                    idx /= r;
                    i
                })
                .collect();
            combos.push(combo);
        }

        combos
    }

    /// Compute LCOE \[USD/kWh\] using NPV of all cost streams.
    pub fn compute_lcoe(
        &self,
        sizes: &[(ComponentType, f64)],
        options: &[ComponentOption],
        annual_energy_kwh: f64,
    ) -> f64 {
        if annual_energy_kwh < 1e-9 {
            return f64::INFINITY;
        }

        let lt = self.config.project_lifetime_years;
        let r = self.config.discount_rate;
        let mut total_npv_cost = 0.0_f64;

        for (ctype, size) in sizes {
            // Find matching option by component type.
            let opt = match options.iter().find(|o| &o.component_type == ctype) {
                Some(o) => o,
                None => continue,
            };

            // Find closest available size index.
            let idx = closest_size_idx(opt, *size);
            let capex = opt.capex_usd_per_unit.get(idx).copied().unwrap_or(0.0);
            let annual_opex = opt.opex_usd_per_kw_year * size;

            // NPV of annual O&M.
            let npv_opex = npv_annuity(annual_opex, r, lt);

            // NPV of replacement costs (at end of each component lifetime).
            let mut npv_replacement = 0.0_f64;
            let comp_life = opt.lifetime_years.max(1);
            let mut year = comp_life;
            while year < lt {
                npv_replacement += opt.replacement_cost_usd / (1.0 + r).powi(year as i32);
                year += comp_life;
            }

            total_npv_cost += capex + npv_opex + npv_replacement;
        }

        // LCOE = total NPV cost / (annual energy × project lifetime, discounted)
        let npv_energy = npv_annuity(annual_energy_kwh, r, lt);
        if npv_energy > 1e-9 {
            total_npv_cost / npv_energy
        } else {
            f64::INFINITY
        }
    }
}

// ── Private helpers ────────────────────────────────────────────────────────────

/// Sum the sizes of all components of the given type.
fn total_size_of_type(sizes: &[(ComponentType, f64)], target: &ComponentType) -> f64 {
    sizes
        .iter()
        .filter(|(t, _)| t == target)
        .map(|(_, s)| s)
        .sum()
}

/// Index of the available size closest to `size`.
fn closest_size_idx(opt: &ComponentOption, size: f64) -> usize {
    opt.sizes_kw_or_kwh
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| {
            (*a - size)
                .abs()
                .partial_cmp(&(*b - size).abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|(i, _)| i)
        .unwrap_or(0)
}

/// Net present value of a uniform annuity of `payment` per year.
fn npv_annuity(payment: f64, rate: f64, years: usize) -> f64 {
    if rate.abs() < 1e-12 {
        return payment * years as f64;
    }
    payment * (1.0 - (1.0 + rate).powi(-(years as i32))) / rate
}

// ── Tests ──────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn flat_load(kw: f64) -> LoadProfile {
        let hours = 8760_usize;
        LoadProfile {
            hourly_load_kw: vec![kw; hours],
            peak_load_kw: kw,
            annual_energy_kwh: kw * hours as f64,
        }
    }

    fn solar_option() -> ComponentOption {
        ComponentOption {
            component_type: ComponentType::SolarPv,
            sizes_kw_or_kwh: vec![50.0, 100.0, 200.0],
            capex_usd_per_unit: vec![50_000.0, 90_000.0, 160_000.0],
            opex_usd_per_kw_year: 15.0,
            lifetime_years: 25,
            replacement_cost_usd: 10_000.0,
        }
    }

    fn battery_option() -> ComponentOption {
        ComponentOption {
            component_type: ComponentType::BatteryStorage,
            sizes_kw_or_kwh: vec![50.0, 100.0, 200.0],
            capex_usd_per_unit: vec![25_000.0, 45_000.0, 80_000.0],
            opex_usd_per_kw_year: 5.0,
            lifetime_years: 10,
            replacement_cost_usd: 20_000.0,
        }
    }

    fn diesel_option() -> ComponentOption {
        ComponentOption {
            component_type: ComponentType::DieselGenerator,
            sizes_kw_or_kwh: vec![100.0],
            capex_usd_per_unit: vec![40_000.0],
            opex_usd_per_kw_year: 30.0,
            lifetime_years: 15,
            replacement_cost_usd: 15_000.0,
        }
    }

    fn solar_irr() -> Vec<f64> {
        // Simplified: 12 hours of sun, 12 of night per day.
        let mut v = vec![0.0_f64; 8760];
        for (h, item) in v.iter_mut().enumerate() {
            *item = if h % 24 < 12 { 0.5 } else { 0.0 };
        }
        v
    }

    fn wind_cf() -> Vec<f64> {
        vec![0.3_f64; 8760]
    }

    fn make_config(rel_target: f64) -> MicrogridSizingConfig {
        MicrogridSizingConfig {
            project_lifetime_years: 20,
            discount_rate: 0.08,
            reliability_target: rel_target,
            grid_connected: false,
            n_monte_carlo: 10, // keep tests fast
        }
    }

    #[test]
    fn test_isolated_microgrid_sizing_returns_result() {
        let sizer = MicrogridSizer::new(make_config(0.80));
        let load = flat_load(20.0);
        let irr = solar_irr();
        let wcf = wind_cf();
        let opts = vec![solar_option(), battery_option(), diesel_option()];
        let result = sizer.optimize(&load, &irr, &wcf, opts);
        assert!(
            result.is_ok(),
            "optimize should succeed: {:?}",
            result.err()
        );
    }

    #[test]
    fn test_reliability_constraint_met() {
        let sizer = MicrogridSizer::new(make_config(0.80));
        let load = flat_load(20.0);
        let irr = solar_irr();
        let wcf = wind_cf();
        let opts = vec![solar_option(), battery_option(), diesel_option()];
        let result = sizer.optimize(&load, &irr, &wcf, opts).expect("optimize");
        assert!(
            result.reliability_achieved >= 0.80,
            "reliability {} should be ≥ 0.80",
            result.reliability_achieved
        );
    }

    #[test]
    fn test_lcoe_positive() {
        let sizer = MicrogridSizer::new(make_config(0.80));
        let load = flat_load(20.0);
        let irr = solar_irr();
        let wcf = wind_cf();
        let opts = vec![solar_option(), battery_option(), diesel_option()];
        let result = sizer.optimize(&load, &irr, &wcf, opts).expect("optimize");
        assert!(
            result.lcoe_usd_per_kwh > 0.0,
            "LCOE must be positive, got {}",
            result.lcoe_usd_per_kwh
        );
    }

    #[test]
    fn test_renewable_fraction_with_no_diesel() {
        let sizer = MicrogridSizer::new(make_config(0.50));
        let load = flat_load(10.0);
        let irr = solar_irr();
        let wcf = wind_cf();
        let opts = vec![solar_option(), battery_option()];
        let result = sizer.optimize(&load, &irr, &wcf, opts).expect("optimize");
        assert!(
            result.renewable_fraction >= 0.9,
            "solar+battery system should have renewable_fraction ≈ 1, got {}",
            result.renewable_fraction
        );
    }

    #[test]
    fn test_payback_period_positive() {
        let sizer = MicrogridSizer::new(make_config(0.80));
        let load = flat_load(20.0);
        let irr = solar_irr();
        let wcf = wind_cf();
        let opts = vec![solar_option(), battery_option(), diesel_option()];
        let result = sizer.optimize(&load, &irr, &wcf, opts).expect("optimize");
        assert!(
            result.payback_years > 0.0,
            "payback period must be positive, got {}",
            result.payback_years
        );
    }
}
