//! Grid Carbon Accounting and Marginal Emissions (QQQ).
//!
//! Provides carbon intensity computation, marginal emission factor (MEF)
//! analysis, and annual carbon reporting for power grids.
//!
//! # Methods
//!
//! - `SimpleAverageFactor` — weighted average of dispatched generators \[t\_CO2/MWh\]
//! - `MarginalGeneratingUnit` — emission factor of the last (most expensive) unit
//! - `LongRunMarginal` — rolling average of marginal factors over time window
//! - `OperatingMarginal` — dispatch-order MEF based on generator merit order
//! - `BuildMarginal` — considers new capacity investments in marginal stack

use thiserror::Error;

// ── Error ────────────────────────────────────────────────────────────────────

/// Errors from the carbon accounting module.
#[derive(Debug, Error)]
pub enum CarbonError {
    /// Empty dispatch sequence provided.
    #[error("dispatch sequence is empty")]
    EmptyDispatch,
    /// Generation vector length does not match generator count.
    #[error("generation vector length {got} does not match generator count {expected}")]
    LengthMismatch { got: usize, expected: usize },
    /// Negative load value encountered.
    #[error("negative total load {0} MW at hour {1}")]
    NegativeLoad(f64, usize),
}

// ── Generator type ────────────────────────────────────────────────────────────

/// Technology type for each generator in the fleet.
#[derive(Debug, Clone, PartialEq)]
pub enum GeneratorType {
    /// High-carbon coal plant.
    Coal,
    /// Natural gas combined-cycle.
    NaturalGasCc,
    /// Natural gas open-cycle peaker.
    NaturalGasPeaker,
    /// Zero-carbon nuclear plant.
    Nuclear,
    /// Hydro (near-zero operational carbon).
    Hydro,
    /// Onshore/offshore wind.
    Wind,
    /// Solar photovoltaic.
    Solar,
    /// Biomass combustion.
    Biomass,
    /// Oil-fired peaker.
    OilPeaker,
    /// Geothermal.
    Geothermal,
}

impl GeneratorType {
    /// Returns `true` when this generator is considered a clean / renewable resource.
    pub fn is_clean(&self) -> bool {
        matches!(
            self,
            GeneratorType::Nuclear
                | GeneratorType::Hydro
                | GeneratorType::Wind
                | GeneratorType::Solar
                | GeneratorType::Geothermal
        )
    }

    /// Default emission factor \[t\_CO2/MWh\] for this technology (operational only).
    pub fn default_factor(&self) -> f64 {
        match self {
            GeneratorType::Coal => 0.95,
            GeneratorType::NaturalGasCc => 0.40,
            GeneratorType::NaturalGasPeaker => 0.65,
            GeneratorType::Nuclear => 0.012,
            GeneratorType::Hydro => 0.024,
            GeneratorType::Wind => 0.011,
            GeneratorType::Solar => 0.041,
            GeneratorType::Biomass => 0.23,
            GeneratorType::OilPeaker => 0.75,
            GeneratorType::Geothermal => 0.038,
        }
    }
}

// ── Marginal emission method ──────────────────────────────────────────────────

/// Algorithm used to compute the marginal emission factor (MEF).
#[derive(Debug, Clone, PartialEq)]
pub enum MarginalEmissionMethod {
    /// Weighted-average emission factor across all dispatched generators.
    SimpleAverageFactor,
    /// Emission factor of the last (highest-cost) dispatched unit.
    MarginalGeneratingUnit,
    /// Rolling long-run average of MEFs over the full dispatch window.
    LongRunMarginal,
    /// Operating marginal: stack-based, sorted by generator cost/type.
    OperatingMarginal,
    /// Build marginal: accounts for new capacity construction decisions.
    BuildMarginal,
}

// ── Dispatch point ────────────────────────────────────────────────────────────

/// Snapshot of the dispatch at one time step (typically one hour).
pub struct DispatchPoint {
    /// Hour index (0-based) in the analysis horizon.
    pub hour: usize,
    /// Generation output per generator \[MW\].
    pub generation_mw: Vec<f64>,
    /// Technology type for each generator entry.
    pub generator_types: Vec<GeneratorType>,
    /// Emission factor per generator \[t\_CO2/MWh\].
    pub emissions_factors: Vec<f64>,
    /// Total system load at this hour \[MW\].
    pub total_load_mw: f64,
}

// ── Carbon metrics per hour ───────────────────────────────────────────────────

/// Carbon emissions metrics computed for a single hour.
pub struct CarbonMetrics {
    /// Hour index.
    pub hour: usize,
    /// Total CO2 emitted \[t\_CO2\].
    pub total_emissions_t: f64,
    /// Load-weighted average emission intensity \[t\_CO2/MWh\].
    pub average_emissions_factor: f64,
    /// Marginal emission factor \[t\_CO2/MWh\] per the selected method.
    pub marginal_emissions_factor: f64,
    /// Fraction of load served by clean/renewable sources \[%\].
    pub clean_energy_pct: f64,
    /// Carbon intensity in display-friendly units \[g\_CO2/kWh\].
    pub carbon_intensity_g_per_kwh: f64,
    /// Emissions avoided versus an all-coal baseline \[t\_CO2\].
    pub avoided_emissions_t: f64,
    /// Total renewable generation dispatched \[MW\].
    pub renewable_generation_mw: f64,
}

// ── Annual analysis result ────────────────────────────────────────────────────

/// Summary of carbon accounting over the full analysis horizon.
pub struct CarbonAnalysisResult {
    /// Per-hour carbon metrics.
    pub hourly: Vec<CarbonMetrics>,
    /// Aggregate annual emissions \[Mt\_CO2\] (millions of tonnes).
    pub annual_emissions_mt: f64,
    /// Average annual emission intensity \[t\_CO2/MWh\].
    pub annual_avg_intensity_t_per_mwh: f64,
    /// Average marginal emission intensity \[t\_CO2/MWh\] over the year.
    pub annual_marginal_intensity_t_per_mwh: f64,
    /// Hour with the highest emissions (0-based).
    pub peak_emissions_hour: usize,
    /// Cleanest hour (lowest carbon intensity, 0-based).
    pub cleanest_hour: usize,
    /// Carbon savings versus a fossil-only (all-coal) baseline \[t\_CO2\].
    pub carbon_savings_t: f64,
    /// Equivalent number of passenger-cars removed from roads for one year.
    /// Conversion: 1 metric tonne CO2 = 1/4.6 cars.
    pub equivalent_cars_removed: f64,
}

// ── Configuration ─────────────────────────────────────────────────────────────

/// Configuration for the `CarbonAccountant`.
pub struct CarbonAccountingConfig {
    /// Override emission factors per technology \[t\_CO2/MWh\].
    /// If a technology is absent the `GeneratorType::default_factor()` is used.
    pub base_emissions_factors: Vec<(GeneratorType, f64)>,
    /// Algorithm used to compute the marginal emission factor.
    pub marginal_emission_method: MarginalEmissionMethod,
    /// Width of each time step \[h\]. Used to convert \[MW\] → \[MWh\].
    pub time_resolution_hours: f64,
    /// ISO/regional identifier for context (informational).
    pub region: String,
}

impl Default for CarbonAccountingConfig {
    fn default() -> Self {
        Self {
            base_emissions_factors: Vec::new(),
            marginal_emission_method: MarginalEmissionMethod::MarginalGeneratingUnit,
            time_resolution_hours: 1.0,
            region: String::from("default"),
        }
    }
}

// ── Carbon accountant ─────────────────────────────────────────────────────────

/// Computes carbon emissions and marginal emission factors from dispatch data.
pub struct CarbonAccountant {
    config: CarbonAccountingConfig,
}

impl CarbonAccountant {
    /// Create a new `CarbonAccountant` with the given configuration.
    pub fn new(config: CarbonAccountingConfig) -> Self {
        Self { config }
    }

    /// Run the full carbon analysis over a sequence of dispatch points.
    ///
    /// Returns a [`CarbonAnalysisResult`] with hourly metrics and annual
    /// aggregates.
    ///
    /// # Errors
    ///
    /// Returns [`CarbonError::EmptyDispatch`] if `dispatch` is empty,
    /// [`CarbonError::LengthMismatch`] if per-generator vectors differ in
    /// length, or [`CarbonError::NegativeLoad`] for invalid load values.
    pub fn analyze(&self, dispatch: &[DispatchPoint]) -> Result<CarbonAnalysisResult, CarbonError> {
        if dispatch.is_empty() {
            return Err(CarbonError::EmptyDispatch);
        }

        // Validate all dispatch points up-front.
        for dp in dispatch {
            let n = dp.generator_types.len();
            if dp.generation_mw.len() != n || dp.emissions_factors.len() != n {
                return Err(CarbonError::LengthMismatch {
                    got: dp.generation_mw.len().min(dp.emissions_factors.len()),
                    expected: n,
                });
            }
            if dp.total_load_mw < 0.0 {
                return Err(CarbonError::NegativeLoad(dp.total_load_mw, dp.hour));
            }
        }

        // Pre-compute time-varying MEFs (needed for some methods).
        let mefs = self.time_varying_mef(dispatch);

        let dt = self.config.time_resolution_hours;
        // Coal emission factor used for avoided-emissions baseline.
        const COAL_FACTOR: f64 = 0.95; // t_CO2/MWh

        let mut hourly = Vec::with_capacity(dispatch.len());
        for (idx, dp) in dispatch.iter().enumerate() {
            let total_gen: f64 = dp.generation_mw.iter().sum();

            // Total emissions this hour [t_CO2].
            let total_emissions_t: f64 = dp
                .generation_mw
                .iter()
                .zip(dp.emissions_factors.iter())
                .map(|(g, ef)| g * ef * dt)
                .sum();

            // Weighted-average emission factor.
            let average_emissions_factor = if total_gen > 0.0 {
                total_emissions_t / (total_gen * dt)
            } else {
                0.0
            };

            let marginal_emissions_factor = mefs[idx];

            // Renewable generation and clean energy percentage.
            let renewable_generation_mw: f64 = dp
                .generation_mw
                .iter()
                .zip(dp.generator_types.iter())
                .filter(|(_, gt)| gt.is_clean())
                .map(|(g, _)| *g)
                .sum();

            let clean_energy_pct = if total_gen > 0.0 {
                100.0 * renewable_generation_mw / total_gen
            } else {
                0.0
            };

            // g/kWh = t/MWh * 1000.
            let carbon_intensity_g_per_kwh = average_emissions_factor * 1000.0;

            // Avoided vs all-coal baseline [t_CO2].
            let baseline_emissions = total_gen * COAL_FACTOR * dt;
            let avoided_emissions_t = baseline_emissions - total_emissions_t;

            hourly.push(CarbonMetrics {
                hour: dp.hour,
                total_emissions_t,
                average_emissions_factor,
                marginal_emissions_factor,
                clean_energy_pct,
                carbon_intensity_g_per_kwh,
                avoided_emissions_t,
                renewable_generation_mw,
            });
        }

        // Annual aggregates.
        let annual_emissions_t: f64 = hourly.iter().map(|m| m.total_emissions_t).sum();
        let annual_emissions_mt = annual_emissions_t / 1_000_000.0;

        let total_gen_all: f64 = dispatch
            .iter()
            .map(|dp| dp.generation_mw.iter().sum::<f64>() * dt)
            .sum();
        let annual_avg_intensity_t_per_mwh = if total_gen_all > 0.0 {
            annual_emissions_t / total_gen_all
        } else {
            0.0
        };

        let annual_marginal_intensity_t_per_mwh = if hourly.is_empty() {
            0.0
        } else {
            hourly
                .iter()
                .map(|m| m.marginal_emissions_factor)
                .sum::<f64>()
                / hourly.len() as f64
        };

        // Peak and cleanest hours.
        let peak_emissions_hour = hourly
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| {
                a.total_emissions_t
                    .partial_cmp(&b.total_emissions_t)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(i, _)| i)
            .unwrap_or(0);

        let cleanest_hour = hourly
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| {
                a.carbon_intensity_g_per_kwh
                    .partial_cmp(&b.carbon_intensity_g_per_kwh)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(i, _)| i)
            .unwrap_or(0);

        let carbon_savings_t: f64 = hourly.iter().map(|m| m.avoided_emissions_t).sum();
        let equivalent_cars_removed = Self::cars_equivalent(carbon_savings_t);

        Ok(CarbonAnalysisResult {
            hourly,
            annual_emissions_mt,
            annual_avg_intensity_t_per_mwh,
            annual_marginal_intensity_t_per_mwh,
            peak_emissions_hour,
            cleanest_hour,
            carbon_savings_t,
            equivalent_cars_removed,
        })
    }

    /// Identify the marginal (last-dispatched) generator index for a dispatch
    /// point. Returns the index of the generator that is partially loaded or
    /// has the highest emission factor among those generating.
    ///
    /// The heuristic used here: among all generators with positive output,
    /// the marginal unit is the one with the highest emission factor
    /// (a proxy for being the most expensive / last in merit order).
    fn marginal_unit(&self, dp: &DispatchPoint) -> Option<usize> {
        dp.generation_mw
            .iter()
            .zip(dp.emissions_factors.iter())
            .enumerate()
            .filter(|(_, (g, _))| **g > 1e-6)
            .max_by(|(_, (_, a)), (_, (_, b))| {
                a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(i, _)| i)
    }

    /// Compute a per-hour marginal emission factor vector using the configured
    /// method.
    fn time_varying_mef(&self, dispatch: &[DispatchPoint]) -> Vec<f64> {
        match self.config.marginal_emission_method {
            MarginalEmissionMethod::SimpleAverageFactor => {
                dispatch.iter().map(|dp| self.average_factor(dp)).collect()
            }

            MarginalEmissionMethod::MarginalGeneratingUnit => dispatch
                .iter()
                .map(|dp| {
                    self.marginal_unit(dp)
                        .map(|i| dp.emissions_factors[i])
                        .unwrap_or(0.0)
                })
                .collect(),

            MarginalEmissionMethod::LongRunMarginal => {
                // Rolling average of instantaneous marginal factors.
                let instant: Vec<f64> = dispatch
                    .iter()
                    .map(|dp| {
                        self.marginal_unit(dp)
                            .map(|i| dp.emissions_factors[i])
                            .unwrap_or(0.0)
                    })
                    .collect();
                let mut mefs = Vec::with_capacity(instant.len());
                let mut running_sum = 0.0;
                for (k, &v) in instant.iter().enumerate() {
                    running_sum += v;
                    mefs.push(running_sum / (k + 1) as f64);
                }
                mefs
            }

            MarginalEmissionMethod::OperatingMarginal => {
                // Sort dispatched generators by emission factor (merit-order
                // proxy). The last unit in merit order is marginal.
                dispatch
                    .iter()
                    .map(|dp| {
                        let mut active: Vec<(f64, f64)> = dp
                            .generation_mw
                            .iter()
                            .zip(dp.emissions_factors.iter())
                            .filter(|(g, _)| **g > 1e-6)
                            .map(|(g, ef)| (*g, *ef))
                            .collect();
                        // Sort ascending by emission factor.
                        active.sort_by(|(_, a), (_, b)| {
                            a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal)
                        });
                        active.last().map(|(_, ef)| *ef).unwrap_or(0.0)
                    })
                    .collect()
            }

            MarginalEmissionMethod::BuildMarginal => {
                // Build marginal: assume new capacity is combined-cycle gas
                // (0.40 t_CO2/MWh) if system is short, else the operating
                // marginal factor.
                const NEW_BUILD_FACTOR: f64 = 0.40;
                dispatch
                    .iter()
                    .map(|dp| {
                        let total_gen: f64 = dp.generation_mw.iter().sum();
                        if total_gen < dp.total_load_mw - 1e-3 {
                            NEW_BUILD_FACTOR
                        } else {
                            self.marginal_unit(dp)
                                .map(|i| dp.emissions_factors[i])
                                .unwrap_or(0.0)
                        }
                    })
                    .collect()
            }
        }
    }

    /// Weighted-average emission factor for a single dispatch point.
    fn average_factor(&self, dp: &DispatchPoint) -> f64 {
        let total_gen: f64 = dp.generation_mw.iter().sum();
        if total_gen < 1e-9 {
            return 0.0;
        }
        dp.generation_mw
            .iter()
            .zip(dp.emissions_factors.iter())
            .map(|(g, ef)| g * ef)
            .sum::<f64>()
            / total_gen
    }

    /// Convert avoided emissions to equivalent passenger-cars removed from
    /// roads for one year.
    ///
    /// EPA conversion: 4.6 metric tonnes CO2 per car per year.
    pub fn cars_equivalent(emissions_t: f64) -> f64 {
        emissions_t / 4.6
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn coal_only_dispatch(hour: usize, mw: f64) -> DispatchPoint {
        DispatchPoint {
            hour,
            generation_mw: vec![mw],
            generator_types: vec![GeneratorType::Coal],
            emissions_factors: vec![0.95],
            total_load_mw: mw,
        }
    }

    fn renewable_dispatch(hour: usize, mw: f64) -> DispatchPoint {
        DispatchPoint {
            hour,
            generation_mw: vec![mw],
            generator_types: vec![GeneratorType::Wind],
            emissions_factors: vec![0.0],
            total_load_mw: mw,
        }
    }

    fn config(method: MarginalEmissionMethod) -> CarbonAccountingConfig {
        CarbonAccountingConfig {
            marginal_emission_method: method,
            ..Default::default()
        }
    }

    /// All-fossil dispatch should produce high carbon intensity.
    #[test]
    fn test_all_fossil_high_intensity() {
        let dispatch: Vec<DispatchPoint> = (0..3).map(|h| coal_only_dispatch(h, 500.0)).collect();
        let accountant = CarbonAccountant::new(config(MarginalEmissionMethod::SimpleAverageFactor));
        let result = accountant
            .analyze(&dispatch)
            .expect("analysis should succeed");

        for m in &result.hourly {
            assert!(
                m.average_emissions_factor > 0.9,
                "Coal intensity must be > 0.9 t/MWh, got {:.4}",
                m.average_emissions_factor
            );
            assert!(
                m.clean_energy_pct < 1.0,
                "No clean generation expected, got {:.2}%",
                m.clean_energy_pct
            );
        }
    }

    /// All-renewable dispatch should yield zero or near-zero emissions.
    #[test]
    fn test_all_renewable_zero_emissions() {
        let dispatch: Vec<DispatchPoint> = (0..4).map(|h| renewable_dispatch(h, 300.0)).collect();
        let accountant =
            CarbonAccountant::new(config(MarginalEmissionMethod::MarginalGeneratingUnit));
        let result = accountant
            .analyze(&dispatch)
            .expect("analysis should succeed");

        for m in &result.hourly {
            assert!(
                m.total_emissions_t.abs() < 1e-9,
                "Renewable dispatch must emit zero, got {:.6}",
                m.total_emissions_t
            );
            assert!(
                (m.clean_energy_pct - 100.0).abs() < 1e-6,
                "Clean % must be 100, got {:.2}",
                m.clean_energy_pct
            );
        }
        assert!(
            result.annual_emissions_mt < 1e-9,
            "Annual emissions must be zero"
        );
    }

    /// Coal peaker is marginal when it is the highest-emission generator running.
    #[test]
    fn test_marginal_coal_peaker() {
        // Mix: base wind + coal peaker running.
        let dp = DispatchPoint {
            hour: 0,
            generation_mw: vec![200.0, 50.0],
            generator_types: vec![GeneratorType::Wind, GeneratorType::Coal],
            emissions_factors: vec![0.011, 0.95],
            total_load_mw: 250.0,
        };
        let accountant =
            CarbonAccountant::new(config(MarginalEmissionMethod::MarginalGeneratingUnit));
        let result = accountant.analyze(&[dp]).expect("analysis should succeed");

        // Marginal unit is coal → MEF ≈ 0.95.
        assert!(
            (result.hourly[0].marginal_emissions_factor - 0.95).abs() < 1e-9,
            "Marginal unit must be coal peaker (0.95 t/MWh), got {:.4}",
            result.hourly[0].marginal_emissions_factor
        );
    }

    /// Annual totals must be consistent with hourly sums.
    #[test]
    fn test_annual_totals_consistent() {
        let dispatch: Vec<DispatchPoint> = (0..24).map(|h| coal_only_dispatch(h, 100.0)).collect();
        let accountant = CarbonAccountant::new(config(MarginalEmissionMethod::SimpleAverageFactor));
        let result = accountant
            .analyze(&dispatch)
            .expect("analysis should succeed");

        let sum_hourly: f64 = result.hourly.iter().map(|m| m.total_emissions_t).sum();
        let from_mt = result.annual_emissions_mt * 1_000_000.0;
        assert!(
            (sum_hourly - from_mt).abs() < 1e-6,
            "Hourly sum {:.4} must match annual Mt {:.6}",
            sum_hourly,
            from_mt
        );

        // 24 h × 100 MW × 0.95 t/MWh = 2280 t
        let expected = 24.0 * 100.0 * 0.95;
        assert!(
            (sum_hourly - expected).abs() < 1e-6,
            "Expected {:.2} t, got {:.4} t",
            expected,
            sum_hourly
        );
    }

    /// Cars-equivalent: 4600 t CO2 saved → 1000 cars.
    #[test]
    fn test_cars_equivalent_formula() {
        let cars = CarbonAccountant::cars_equivalent(4600.0);
        assert!(
            (cars - 1000.0).abs() < 1e-6,
            "4600 t / 4.6 = 1000 cars, got {:.4}",
            cars
        );
    }

    /// Long-run marginal MEF is the running average.
    #[test]
    fn test_long_run_marginal_is_running_average() {
        let dispatch = vec![
            DispatchPoint {
                hour: 0,
                generation_mw: vec![100.0],
                generator_types: vec![GeneratorType::Coal],
                emissions_factors: vec![0.8],
                total_load_mw: 100.0,
            },
            DispatchPoint {
                hour: 1,
                generation_mw: vec![100.0],
                generator_types: vec![GeneratorType::NaturalGasCc],
                emissions_factors: vec![0.4],
                total_load_mw: 100.0,
            },
        ];
        let accountant = CarbonAccountant::new(config(MarginalEmissionMethod::LongRunMarginal));
        let result = accountant
            .analyze(&dispatch)
            .expect("analysis should succeed");

        // Hour 0: running avg = 0.8; Hour 1: running avg = (0.8+0.4)/2 = 0.6.
        assert!(
            (result.hourly[0].marginal_emissions_factor - 0.8).abs() < 1e-9,
            "Hour 0 LRM must be 0.8"
        );
        assert!(
            (result.hourly[1].marginal_emissions_factor - 0.6).abs() < 1e-9,
            "Hour 1 LRM must be 0.6, got {:.4}",
            result.hourly[1].marginal_emissions_factor
        );
    }

    /// Empty dispatch returns an error.
    #[test]
    fn test_empty_dispatch_error() {
        let accountant = CarbonAccountant::new(config(MarginalEmissionMethod::SimpleAverageFactor));
        let result = accountant.analyze(&[]);
        assert!(
            matches!(result, Err(CarbonError::EmptyDispatch)),
            "Expected EmptyDispatch error"
        );
    }

    /// OperatingMarginal selects the highest emission factor unit.
    #[test]
    fn test_operating_marginal_selects_highest_ef_unit() {
        let dp = DispatchPoint {
            hour: 0,
            generation_mw: vec![150.0, 50.0],
            generator_types: vec![GeneratorType::Wind, GeneratorType::Coal],
            emissions_factors: vec![0.0, 0.95],
            total_load_mw: 200.0,
        };
        let accountant = CarbonAccountant::new(config(MarginalEmissionMethod::OperatingMarginal));
        let result = accountant.analyze(&[dp]).expect("analysis should succeed");
        assert!(
            (result.hourly[0].marginal_emissions_factor - 0.95).abs() < 1e-9,
            "OperatingMarginal must pick coal (0.95 t/MWh), got {:.6}",
            result.hourly[0].marginal_emissions_factor
        );
    }

    /// BuildMarginal returns new-build factor when generation is short of load.
    #[test]
    fn test_build_marginal_uses_new_build_factor_when_short() {
        let dp = DispatchPoint {
            hour: 0,
            generation_mw: vec![80.0],
            generator_types: vec![GeneratorType::NaturalGasCc],
            emissions_factors: vec![0.40],
            total_load_mw: 100.0,
        };
        let accountant = CarbonAccountant::new(config(MarginalEmissionMethod::BuildMarginal));
        let result = accountant.analyze(&[dp]).expect("analysis should succeed");
        assert!(
            (result.hourly[0].marginal_emissions_factor - 0.40).abs() < 1e-9,
            "BuildMarginal when short must return 0.40 (new-build CC), got {:.6}",
            result.hourly[0].marginal_emissions_factor
        );
    }

    /// BuildMarginal returns the running unit's factor when generation meets load.
    #[test]
    fn test_build_marginal_uses_marginal_unit_when_sufficient() {
        let dp = DispatchPoint {
            hour: 0,
            generation_mw: vec![100.0],
            generator_types: vec![GeneratorType::Coal],
            emissions_factors: vec![0.95],
            total_load_mw: 100.0,
        };
        let accountant = CarbonAccountant::new(config(MarginalEmissionMethod::BuildMarginal));
        let result = accountant.analyze(&[dp]).expect("analysis should succeed");
        assert!(
            (result.hourly[0].marginal_emissions_factor - 0.95).abs() < 1e-9,
            "BuildMarginal when sufficient must return coal factor (0.95), got {:.6}",
            result.hourly[0].marginal_emissions_factor
        );
    }

    /// Mismatched generation and generator-type vector lengths yield LengthMismatch.
    #[test]
    fn test_length_mismatch_error() {
        let dp = DispatchPoint {
            hour: 0,
            generation_mw: vec![100.0], // length = 1
            generator_types: vec![GeneratorType::Coal, GeneratorType::Wind], // length = 2
            emissions_factors: vec![0.95, 0.0],
            total_load_mw: 100.0,
        };
        let accountant = CarbonAccountant::new(config(MarginalEmissionMethod::SimpleAverageFactor));
        let result = accountant.analyze(&[dp]);
        assert!(
            matches!(result, Err(CarbonError::LengthMismatch { .. })),
            "Expected LengthMismatch error, got {:?}",
            result.map(|_| ())
        );
    }

    /// Negative load value returns NegativeLoad error.
    #[test]
    fn test_negative_load_error() {
        let dp = DispatchPoint {
            hour: 0,
            generation_mw: vec![100.0],
            generator_types: vec![GeneratorType::Coal],
            emissions_factors: vec![0.95],
            total_load_mw: -10.0,
        };
        let accountant = CarbonAccountant::new(config(MarginalEmissionMethod::SimpleAverageFactor));
        let result = accountant.analyze(&[dp]);
        assert!(
            matches!(result, Err(CarbonError::NegativeLoad(..))),
            "Expected NegativeLoad error, got {:?}",
            result.map(|_| ())
        );
    }

    /// GeneratorType::is_clean() and default_factor() return expected values.
    #[test]
    fn test_generator_type_is_clean_and_default_factor() {
        assert!(GeneratorType::Wind.is_clean(), "Wind must be clean");
        assert!(GeneratorType::Solar.is_clean(), "Solar must be clean");
        assert!(!GeneratorType::Coal.is_clean(), "Coal must not be clean");
        assert!(
            !GeneratorType::NaturalGasCc.is_clean(),
            "NaturalGasCc must not be clean"
        );
        assert!(
            (GeneratorType::Coal.default_factor() - 0.95).abs() < 1e-12,
            "Coal default factor must be 0.95, got {:.14}",
            GeneratorType::Coal.default_factor()
        );
        assert!(
            (GeneratorType::Wind.default_factor() - 0.011).abs() < 1e-12,
            "Wind default factor must be 0.011, got {:.14}",
            GeneratorType::Wind.default_factor()
        );
    }

    /// carbon_intensity_g_per_kwh, avoided_emissions_t, and clean_energy_pct
    /// are computed correctly for a pure-coal dispatch.
    #[test]
    fn test_carbon_intensity_g_per_kwh_and_avoided_emissions() {
        let dp = DispatchPoint {
            hour: 0,
            generation_mw: vec![100.0],
            generator_types: vec![GeneratorType::Coal],
            emissions_factors: vec![0.95],
            total_load_mw: 100.0,
        };
        let accountant = CarbonAccountant::new(CarbonAccountingConfig {
            marginal_emission_method: MarginalEmissionMethod::SimpleAverageFactor,
            time_resolution_hours: 1.0,
            ..Default::default()
        });
        let result = accountant.analyze(&[dp]).expect("analysis should succeed");
        let m = &result.hourly[0];
        assert!(
            (m.carbon_intensity_g_per_kwh - 950.0).abs() < 1e-9,
            "Carbon intensity must be 950 g/kWh, got {:.6}",
            m.carbon_intensity_g_per_kwh
        );
        // Coal dispatched == coal baseline → zero avoided emissions.
        assert!(
            m.avoided_emissions_t.abs() < 1e-9,
            "Avoided emissions must be 0.0 for all-coal dispatch, got {:.6}",
            m.avoided_emissions_t
        );
        assert!(
            m.clean_energy_pct.abs() < 1e-9,
            "Clean energy pct must be 0.0, got {:.6}",
            m.clean_energy_pct
        );
    }
}
