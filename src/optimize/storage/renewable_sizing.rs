//! Renewable Energy Storage Sizing Tool.
//!
//! Sizes energy storage systems paired with renewable generation to meet
//! reliability targets at minimum LCOE (Levelised Cost of Energy).
//!
//! The sizing algorithm:
//! 1. Sweeps a grid of (renewable\_capacity, storage\_mwh) pairs.
//! 2. Simulates hourly operation: charge excess, discharge deficit.
//! 3. Computes LOLP (Loss of Load Probability) and curtailment.
//! 4. Computes LCOE including CAPEX, OPEX, and financing.
//! 5. Identifies minimum-LCOE point meeting the reliability target.
//! 6. Returns the Pareto frontier and sensitivity analysis.

// ─── Errors ───────────────────────────────────────────────────────────────────

/// Errors from the renewable storage sizing tool.
#[derive(Debug, thiserror::Error)]
pub enum SizingError {
    /// Load profile not set or empty.
    #[error("load profile not set or has zero length")]
    NoLoad,
    /// Renewable capacity factor vector length mismatch.
    #[error("capacity factor vector length {0} does not match n_hours {1}")]
    LengthMismatch(usize, usize),
    /// No feasible sizing found within the search grid.
    #[error("no feasible sizing found meeting reliability target {0:.4}")]
    NoFeasibleSizing(f64),
    /// Configuration error.
    #[error("configuration error: {0}")]
    Config(String),
}

// ─── Configuration ────────────────────────────────────────────────────────────

/// Configuration for the renewable storage sizing tool.
#[derive(Debug, Clone)]
pub struct RenewableSizingConfig {
    /// Simulation period \[h\] (8760 for one year).
    pub n_hours: usize,
    /// Target Loss of Load Probability (e.g. 0.01 = 1 %).
    pub target_reliability: f64,
    /// Typical battery cycles per day.
    pub battery_cycles_per_day: f64,
    /// Battery calendar life \[years\].
    pub battery_calendar_life_years: f64,
    /// Battery full-cycle life (e.g. 2000 cycles).
    pub battery_cycle_life: f64,
    /// Inverter efficiency \[0, 1\].
    pub inverter_efficiency: f64,
    /// Battery round-trip efficiency \[0, 1\].
    pub battery_efficiency: f64,
    /// Discount rate for NPV calculation.
    pub discount_rate: f64,
    /// Project economic life \[years\].
    pub project_life_years: usize,
}

impl Default for RenewableSizingConfig {
    fn default() -> Self {
        Self {
            n_hours: 8760,
            target_reliability: 0.01,
            battery_cycles_per_day: 1.0,
            battery_calendar_life_years: 15.0,
            battery_cycle_life: 2000.0,
            inverter_efficiency: 0.97,
            battery_efficiency: 0.90,
            discount_rate: 0.07,
            project_life_years: 25,
        }
    }
}

// ─── Input Data ───────────────────────────────────────────────────────────────

/// Renewable generator definition.
#[derive(Debug, Clone)]
pub struct RenewableGenerator {
    /// Technology name (e.g. "Solar PV", "Onshore Wind").
    pub technology: String,
    /// Nameplate capacity \[MW\].
    pub capacity_mw: f64,
    /// Capital cost \[USD/MW\].
    pub capex_usd_per_mw: f64,
    /// Annual operating cost \[USD/MW/year\].
    pub opex_usd_per_mw_year: f64,
    /// Hourly capacity factors (length must equal n\_hours).
    pub capacity_factor_hourly: Vec<f64>,
}

/// Battery storage technology definition.
#[derive(Debug, Clone)]
pub struct StorageOption {
    /// Technology name (e.g. "Li-ion", "Flow Battery", "CAES").
    pub technology: String,
    /// Energy-related capital cost \[USD/MWh\].
    pub capex_usd_per_mwh: f64,
    /// Power-related capital cost \[USD/MW\].
    pub capex_usd_per_mw: f64,
    /// Annual operating cost \[USD/MWh/year\].
    pub opex_usd_per_mwh_year: f64,
    /// Round-trip efficiency \[0, 1\].
    pub efficiency: f64,
    /// Minimum state of charge \[fraction\].
    pub min_soc: f64,
    /// Maximum state of charge \[fraction\].
    pub max_soc: f64,
    /// Daily self-discharge rate \[fraction/day\].
    pub self_discharge_per_day: f64,
}

// ─── Results ─────────────────────────────────────────────────────────────────

/// A single point on the sizing Pareto frontier.
#[derive(Debug, Clone)]
pub struct SizingPoint {
    /// Renewable capacity \[MW\].
    pub renewable_mw: f64,
    /// Storage energy capacity \[MWh\].
    pub storage_mwh: f64,
    /// Storage power capacity \[MW\].
    pub storage_mw: f64,
    /// Loss of Load Probability (fraction of hours unserved).
    pub lolp: f64,
    /// Curtailment percentage of total generation.
    pub curtailment_pct: f64,
    /// Levelised Cost of Energy \[USD/MWh\].
    pub lcoe_usd_per_mwh: f64,
    /// Total CAPEX \[M USD\].
    pub total_capex_m_usd: f64,
}

/// Optimal sizing result.
#[derive(Debug, Clone)]
pub struct OptimalSizing {
    /// Optimal renewable capacity \[MW\].
    pub renewable_mw: f64,
    /// Optimal storage energy capacity \[MWh\].
    pub storage_mwh: f64,
    /// Optimal storage power capacity \[MW\].
    pub storage_mw: f64,
    /// LCOE at optimal point \[USD/MWh\].
    pub lcoe_usd_per_mwh: f64,
    /// Achieved LOLP.
    pub lolp_achieved: f64,
    /// Curtailment at optimal point \[%\].
    pub curtailment_pct: f64,
    /// Sensitivity analysis: (parameter, % LCOE impact).
    pub sensitivity: Vec<(String, f64)>,
    /// Pareto frontier: cost vs reliability trade-off.
    pub sizing_curve: Vec<SizingPoint>,
}

// ─── Sizer ────────────────────────────────────────────────────────────────────

/// Renewable energy storage sizing tool.
pub struct RenewableStorageSizer {
    config: RenewableSizingConfig,
    load_mw: Vec<f64>,
}

impl RenewableStorageSizer {
    /// Create a new sizer with the given configuration.
    pub fn new(config: RenewableSizingConfig) -> Self {
        Self {
            config,
            load_mw: Vec::new(),
        }
    }

    /// Set the hourly load profile \[MW\].
    pub fn set_load(&mut self, load_mw: Vec<f64>) {
        self.load_mw = load_mw;
    }

    /// Find the optimal renewable + storage sizing.
    pub fn size_system(
        &self,
        generator: &RenewableGenerator,
        storage: &StorageOption,
    ) -> Result<OptimalSizing, SizingError> {
        let n = self.config.n_hours;

        if self.load_mw.is_empty() {
            return Err(SizingError::NoLoad);
        }
        if generator.capacity_factor_hourly.len() != n {
            return Err(SizingError::LengthMismatch(
                generator.capacity_factor_hourly.len(),
                n,
            ));
        }

        let avg_load = self.load_mw.iter().sum::<f64>() / self.load_mw.len() as f64;
        if avg_load <= 0.0 {
            return Err(SizingError::Config("Average load must be > 0".to_string()));
        }

        // Build search grid
        // Renewable: 0.5x to 3x average load in 8 steps
        let gen_steps = 8usize;
        let stor_steps = 8usize;

        let gen_sizes: Vec<f64> = (0..gen_steps)
            .map(|i| avg_load * 0.5 + avg_load * 2.5 * i as f64 / (gen_steps - 1).max(1) as f64)
            .collect();
        // Storage: 0 to 2x daily average demand
        let max_storage_mwh = avg_load * 24.0 * 2.0;
        let stor_sizes_mwh: Vec<f64> = (0..stor_steps)
            .map(|i| max_storage_mwh * i as f64 / (stor_steps - 1).max(1) as f64)
            .collect();

        let mut all_points: Vec<SizingPoint> = Vec::new();

        let annual_energy_mwh: f64 = self.load_mw.iter().sum::<f64>();
        let project_n = self.config.project_life_years;
        let r = self.config.discount_rate;
        // Capital recovery factor
        let crf = if r > 0.0 {
            r * (1.0 + r).powi(project_n as i32) / ((1.0 + r).powi(project_n as i32) - 1.0)
        } else {
            1.0 / project_n as f64
        };

        for &gen_mw in &gen_sizes {
            for &stor_mwh in &stor_sizes_mwh {
                // Power capacity = sqrt(MWh), or fixed at 0.5C rate, min 1 MW
                let stor_mw = (stor_mwh / 2.0).max(1.0).min(gen_mw);

                let (lolp, curtailment_pct) = self.simulate_operation(
                    gen_mw,
                    stor_mwh,
                    stor_mw,
                    &generator.capacity_factor_hourly,
                    storage,
                );

                let lcoe = self.compute_lcoe(
                    gen_mw,
                    generator,
                    stor_mwh,
                    stor_mw,
                    storage,
                    annual_energy_mwh * (1.0 - lolp),
                    crf,
                );

                let total_capex = (gen_mw * generator.capex_usd_per_mw
                    + stor_mwh * storage.capex_usd_per_mwh
                    + stor_mw * storage.capex_usd_per_mw)
                    / 1e6;

                all_points.push(SizingPoint {
                    renewable_mw: gen_mw,
                    storage_mwh: stor_mwh,
                    storage_mw: stor_mw,
                    lolp,
                    curtailment_pct,
                    lcoe_usd_per_mwh: lcoe,
                    total_capex_m_usd: total_capex,
                });
            }
        }

        // Filter feasible points (LOLP ≤ target)
        let target = self.config.target_reliability;
        let feasible: Vec<&SizingPoint> = all_points.iter().filter(|p| p.lolp <= target).collect();

        // Find minimum LCOE among feasible points
        let optimal = feasible
            .iter()
            .min_by(|a, b| {
                a.lcoe_usd_per_mwh
                    .partial_cmp(&b.lcoe_usd_per_mwh)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .copied()
            .ok_or(SizingError::NoFeasibleSizing(target))?;

        // Build Pareto frontier: min LCOE for each LOLP level
        let mut sizing_curve = self.build_pareto_curve(&all_points);
        sizing_curve.sort_by(|a, b| {
            a.lolp
                .partial_cmp(&b.lolp)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        // Sensitivity analysis at optimal point
        let sensitivity =
            self.sensitivity_analysis(optimal, generator, storage, crf, annual_energy_mwh);

        Ok(OptimalSizing {
            renewable_mw: optimal.renewable_mw,
            storage_mwh: optimal.storage_mwh,
            storage_mw: optimal.storage_mw,
            lcoe_usd_per_mwh: optimal.lcoe_usd_per_mwh,
            lolp_achieved: optimal.lolp,
            curtailment_pct: optimal.curtailment_pct,
            sensitivity,
            sizing_curve,
        })
    }

    /// Simulate hourly operation and return (lolp, curtailment\_pct).
    pub fn simulate_operation(
        &self,
        gen_capacity_mw: f64,
        storage_mwh: f64,
        storage_mw: f64,
        cf_hourly: &[f64],
        storage: &StorageOption,
    ) -> (f64, f64) {
        let n = cf_hourly.len().min(self.load_mw.len());
        if n == 0 {
            return (1.0, 0.0);
        }

        let mut soc = storage_mwh * 0.5; // start at 50 % SoC
        let soc_min = storage_mwh * storage.min_soc;
        let soc_max = storage_mwh * storage.max_soc;

        let rt_eff = storage.efficiency.clamp(0.01, 1.0).sqrt();
        let charge_eff = rt_eff;
        let discharge_eff = rt_eff;
        let inv_eff = self.config.inverter_efficiency.clamp(0.01, 1.0);

        let self_discharge_hourly =
            1.0 - (1.0 - storage.self_discharge_per_day.clamp(0.0, 0.99)).powf(1.0 / 24.0);

        let mut unserved_hours = 0usize;
        let mut curtailed_energy = 0.0f64;
        let mut total_gen = 0.0f64;

        for (h, &cf_h) in cf_hourly.iter().enumerate().take(n) {
            // Self-discharge
            soc *= 1.0 - self_discharge_hourly;

            let load = self.load_mw[h].max(0.0);
            let gen = gen_capacity_mw * cf_h.clamp(0.0, 1.0) * inv_eff;
            total_gen += gen;

            let net = gen - load; // positive = surplus, negative = deficit

            if net >= 0.0 {
                // Surplus: charge battery
                let charge_avail = net * charge_eff;
                let charge_actual = charge_avail.min(storage_mw).min(soc_max - soc);
                soc += charge_actual;
                // Excess beyond battery: curtailed
                let delivered_to_storage = charge_actual / charge_eff;
                curtailed_energy += (net - delivered_to_storage).max(0.0);
            } else {
                // Deficit: discharge battery
                let deficit = (-net).min(storage_mw);
                let discharge_actual = deficit.min((soc - soc_min) * discharge_eff);
                soc -= discharge_actual / discharge_eff;
                let net_after_storage = -net - discharge_actual;
                if net_after_storage > 1e-6 {
                    unserved_hours += 1;
                }
            }
        }

        let lolp = unserved_hours as f64 / n as f64;
        let curtailment_pct = if total_gen > 0.0 {
            curtailed_energy / total_gen * 100.0
        } else {
            0.0
        };

        (lolp.clamp(0.0, 1.0), curtailment_pct.clamp(0.0, 100.0))
    }

    // ─── Internal helpers ───────────────────────────────────────────────────

    /// Compute LCOE \[USD/MWh\] for a given sizing.
    #[allow(clippy::too_many_arguments)]
    fn compute_lcoe(
        &self,
        gen_mw: f64,
        generator: &RenewableGenerator,
        stor_mwh: f64,
        stor_mw: f64,
        storage: &StorageOption,
        annual_energy_served_mwh: f64,
        crf: f64,
    ) -> f64 {
        if annual_energy_served_mwh < 1.0 {
            return f64::MAX;
        }

        let gen_capex = gen_mw * generator.capex_usd_per_mw;
        let stor_capex = stor_mwh * storage.capex_usd_per_mwh + stor_mw * storage.capex_usd_per_mw;
        let total_capex = gen_capex + stor_capex;

        let annual_capex = total_capex * crf;
        let annual_opex =
            gen_mw * generator.opex_usd_per_mw_year + stor_mwh * storage.opex_usd_per_mwh_year;

        (annual_capex + annual_opex) / annual_energy_served_mwh
    }

    /// Build a Pareto-efficient sizing curve (min LCOE for each LOLP bucket).
    fn build_pareto_curve(&self, points: &[SizingPoint]) -> Vec<SizingPoint> {
        // Bucket by LOLP into 10 groups
        let n_buckets = 10usize;
        let mut buckets: Vec<Option<&SizingPoint>> = vec![None; n_buckets];

        for p in points {
            let bucket = ((p.lolp * n_buckets as f64).floor() as usize).min(n_buckets - 1);
            match buckets[bucket] {
                None => buckets[bucket] = Some(p),
                Some(existing) => {
                    if p.lcoe_usd_per_mwh < existing.lcoe_usd_per_mwh {
                        buckets[bucket] = Some(p);
                    }
                }
            }
        }

        buckets.into_iter().flatten().cloned().collect()
    }

    /// Compute sensitivity of LCOE to key parameters at the optimal point.
    fn sensitivity_analysis(
        &self,
        optimal: &SizingPoint,
        generator: &RenewableGenerator,
        storage: &StorageOption,
        crf: f64,
        annual_energy_mwh: f64,
    ) -> Vec<(String, f64)> {
        let base_lcoe = optimal.lcoe_usd_per_mwh;
        if base_lcoe <= 0.0 || base_lcoe == f64::MAX {
            return Vec::new();
        }

        let perturb = 0.10; // 10 % perturbation
        let mut results = Vec::new();

        // Sensitivity to storage CAPEX
        {
            let mut s = storage.clone();
            s.capex_usd_per_mwh *= 1.0 + perturb;
            let new_lcoe = self.compute_lcoe(
                optimal.renewable_mw,
                generator,
                optimal.storage_mwh,
                optimal.storage_mw,
                &s,
                annual_energy_mwh,
                crf,
            );
            let impact = (new_lcoe - base_lcoe) / base_lcoe * 100.0;
            results.push(("Storage CAPEX ($/MWh)".to_string(), impact));
        }

        // Sensitivity to generator CAPEX
        {
            let mut g = generator.clone();
            g.capex_usd_per_mw *= 1.0 + perturb;
            let new_lcoe = self.compute_lcoe(
                optimal.renewable_mw,
                &g,
                optimal.storage_mwh,
                optimal.storage_mw,
                storage,
                annual_energy_mwh,
                crf,
            );
            let impact = (new_lcoe - base_lcoe) / base_lcoe * 100.0;
            results.push(("Renewable CAPEX ($/MW)".to_string(), impact));
        }

        // Sensitivity to discount rate
        {
            let r = self.config.discount_rate * (1.0 + perturb);
            let n = self.config.project_life_years;
            let new_crf = if r > 0.0 {
                r * (1.0 + r).powi(n as i32) / ((1.0 + r).powi(n as i32) - 1.0)
            } else {
                1.0 / n as f64
            };
            let new_lcoe = self.compute_lcoe(
                optimal.renewable_mw,
                generator,
                optimal.storage_mwh,
                optimal.storage_mw,
                storage,
                annual_energy_mwh,
                new_crf,
            );
            let impact = (new_lcoe - base_lcoe) / base_lcoe * 100.0;
            results.push(("Discount Rate".to_string(), impact));
        }

        // Sensitivity to battery efficiency
        {
            let mut s = storage.clone();
            s.efficiency = (s.efficiency * (1.0 - perturb)).max(0.5);
            let (new_lolp, _) = self.simulate_operation(
                optimal.renewable_mw,
                optimal.storage_mwh,
                optimal.storage_mw,
                &generator.capacity_factor_hourly,
                &s,
            );
            // Lower efficiency → less energy served
            let served = annual_energy_mwh * (1.0 - new_lolp);
            let new_lcoe = self.compute_lcoe(
                optimal.renewable_mw,
                generator,
                optimal.storage_mwh,
                optimal.storage_mw,
                &s,
                served,
                crf,
            );
            let impact = (new_lcoe - base_lcoe) / base_lcoe * 100.0;
            results.push(("Battery Efficiency".to_string(), impact));
        }

        results
    }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> RenewableSizingConfig {
        RenewableSizingConfig {
            n_hours: 24,
            target_reliability: 0.05,
            battery_cycles_per_day: 1.0,
            battery_calendar_life_years: 15.0,
            battery_cycle_life: 2000.0,
            inverter_efficiency: 0.97,
            battery_efficiency: 0.90,
            discount_rate: 0.07,
            project_life_years: 25,
        }
    }

    fn solar_cf(n: usize) -> Vec<f64> {
        // Simple diurnal profile: 0 at night, peaks at midday
        (0..n)
            .map(|h| {
                let hour = h % 24;
                if !(6..20).contains(&hour) {
                    0.0
                } else if hour < 13 {
                    (hour - 6) as f64 / 7.0
                } else {
                    (20 - hour) as f64 / 7.0
                }
            })
            .collect()
    }

    fn flat_load(n: usize, mw: f64) -> Vec<f64> {
        vec![mw; n]
    }

    fn make_generator(n_hours: usize, capacity_mw: f64) -> RenewableGenerator {
        RenewableGenerator {
            technology: "Solar PV".to_string(),
            capacity_mw,
            capex_usd_per_mw: 1_000_000.0,
            opex_usd_per_mw_year: 15_000.0,
            capacity_factor_hourly: solar_cf(n_hours),
        }
    }

    fn make_storage() -> StorageOption {
        StorageOption {
            technology: "Li-ion".to_string(),
            capex_usd_per_mwh: 300_000.0,
            capex_usd_per_mw: 150_000.0,
            opex_usd_per_mwh_year: 5_000.0,
            efficiency: 0.90,
            min_soc: 0.1,
            max_soc: 0.95,
            self_discharge_per_day: 0.001,
        }
    }

    // Test 1: Oversized system → LOLP = 0
    #[test]
    fn test_oversized_system_zero_lolp() {
        let cfg = default_config();
        let n = cfg.n_hours;
        let mut sizer = RenewableStorageSizer::new(cfg);
        sizer.set_load(flat_load(n, 5.0)); // 5 MW constant load

        let storage = make_storage();
        // Very large generator and storage relative to load
        let (lolp, _) = sizer.simulate_operation(
            100.0, // 100 MW generator
            500.0, // 500 MWh storage
            50.0,  // 50 MW power
            &solar_cf(n),
            &storage,
        );
        assert!(
            lolp < 0.01,
            "Oversized system should have LOLP ≈ 0: got {lolp:.4}"
        );
    }

    // Test 2: Undersized system → LOLP > 0
    #[test]
    fn test_undersized_system_positive_lolp() {
        let cfg = default_config();
        let n = cfg.n_hours;
        let mut sizer = RenewableStorageSizer::new(cfg);
        sizer.set_load(flat_load(n, 20.0)); // 20 MW load

        let storage = make_storage();
        // Tiny generator with no storage
        let (lolp, _) = sizer.simulate_operation(
            1.0, // 1 MW generator
            0.0, // no storage
            0.0,
            &solar_cf(n),
            &storage,
        );
        assert!(
            lolp > 0.1,
            "Undersized system should have LOLP > 10 %: got {lolp:.4}"
        );
    }

    // Test 3: LCOE increases monotonically with more storage (at fixed gen)
    #[test]
    fn test_lcoe_increases_with_storage() {
        let cfg = default_config();
        let n = cfg.n_hours;
        let mut sizer = RenewableStorageSizer::new(cfg);
        sizer.set_load(flat_load(n, 5.0));

        let generator = make_generator(n, 10.0);
        let storage = make_storage();

        let annual_mwh: f64 = sizer.load_mw.iter().sum::<f64>();
        let crf = 0.07 * (1.07f64).powi(25) / ((1.07f64).powi(25) - 1.0);

        let lcoe_small = sizer.compute_lcoe(10.0, &generator, 10.0, 5.0, &storage, annual_mwh, crf);
        let lcoe_large =
            sizer.compute_lcoe(10.0, &generator, 500.0, 50.0, &storage, annual_mwh, crf);

        assert!(
            lcoe_large > lcoe_small,
            "LCOE should increase with more storage: small={lcoe_small:.2} large={lcoe_large:.2}"
        );
    }

    // Test 4: Curtailment decreases with more storage (at fixed gen)
    #[test]
    fn test_curtailment_decreases_with_storage() {
        let cfg = default_config();
        let n = cfg.n_hours;
        let mut sizer = RenewableStorageSizer::new(cfg);
        sizer.set_load(flat_load(n, 3.0)); // 3 MW load

        let storage = make_storage();
        let cf = solar_cf(n);

        let (_, curtail_small) = sizer.simulate_operation(10.0, 1.0, 0.5, &cf, &storage);
        let (_, curtail_large) = sizer.simulate_operation(10.0, 100.0, 10.0, &cf, &storage);

        assert!(
            curtail_large <= curtail_small,
            "More storage should reduce curtailment: small={curtail_small:.2}% large={curtail_large:.2}%"
        );
    }

    // Test 5: Sensitivity analysis produces non-empty results
    #[test]
    fn test_sensitivity_analysis_produced() {
        let cfg = default_config();
        let n = cfg.n_hours;
        let mut sizer = RenewableStorageSizer::new(cfg);
        sizer.set_load(flat_load(n, 3.0));

        let generator = make_generator(n, 8.0);
        let storage = make_storage();
        let result = sizer.size_system(&generator, &storage).expect("sizing ok");

        assert!(
            !result.sensitivity.is_empty(),
            "Sensitivity analysis must produce results"
        );
        // Storage cost should have an impact on LCOE
        let stor_impact = result
            .sensitivity
            .iter()
            .find(|(name, _)| name.contains("Storage CAPEX"))
            .map(|(_, v)| *v);
        assert!(
            stor_impact.is_some(),
            "Storage CAPEX sensitivity must be reported"
        );
    }

    // Test 6: No load → error
    #[test]
    fn test_no_load_error() {
        let cfg = default_config();
        let sizer = RenewableStorageSizer::new(cfg.clone());
        let generator = make_generator(cfg.n_hours, 10.0);
        let storage = make_storage();
        assert!(
            sizer.size_system(&generator, &storage).is_err(),
            "No load profile should return error"
        );
    }

    // Test 7: Optimal sizing meets reliability target
    #[test]
    fn test_optimal_meets_reliability() {
        let cfg = default_config();
        let n = cfg.n_hours;
        let target = cfg.target_reliability;
        let mut sizer = RenewableStorageSizer::new(cfg);
        sizer.set_load(flat_load(n, 3.0));

        let generator = make_generator(n, 10.0);
        let storage = make_storage();
        let result = sizer.size_system(&generator, &storage).expect("sizing ok");

        assert!(
            result.lolp_achieved <= target + 1e-9,
            "LOLP {:.4} must meet target {target:.4}",
            result.lolp_achieved
        );
    }

    // Test 8: Sizing curve is non-empty
    #[test]
    fn test_sizing_curve_non_empty() {
        let cfg = default_config();
        let n = cfg.n_hours;
        let mut sizer = RenewableStorageSizer::new(cfg);
        sizer.set_load(flat_load(n, 3.0));

        let generator = make_generator(n, 10.0);
        let storage = make_storage();
        let result = sizer.size_system(&generator, &storage).expect("sizing ok");

        assert!(
            !result.sizing_curve.is_empty(),
            "Sizing curve must be non-empty"
        );
    }
}
