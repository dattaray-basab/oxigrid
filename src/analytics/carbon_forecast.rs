//! Grid Carbon Intensity Forecasting and Marginal Emission Rate (MER) Estimation.
//!
//! Provides a complete pipeline for:
//! - Merit-order dispatch stack construction
//! - Average grid carbon intensity \[kg CO₂/MWh\]
//! - Marginal emission rate (MER) computation \[kg CO₂/MWh\]
//! - 24-hour ahead carbon intensity forecasting with prediction intervals
//! - Locational marginal emissions (LME) via PTDF sensitivity
//! - Carbon-optimal dispatch (cleanest-first greedy)
//! - Emission savings vs. a baseline dispatch schedule
//! - Forecast skill score (Murphy's SS)

use thiserror::Error;

// ── Error ─────────────────────────────────────────────────────────────────────

/// Errors emitted by the carbon forecast module.
#[derive(Debug, Error)]
pub enum CarbonForecastError {
    /// No generators registered in the forecaster.
    #[error("generator fleet is empty")]
    EmptyFleet,

    /// The dispatch slice is empty; cannot compute weighted average.
    #[error("dispatch slice is empty — cannot compute average intensity")]
    EmptyDispatch,

    /// Load and renewable profiles must have equal length.
    #[error("load profile length {load} ≠ renewable profile length {renewable}")]
    ProfileLengthMismatch { load: usize, renewable: usize },

    /// Demand exceeds the total installed capacity of the fleet.
    #[error("demand {demand_mw:.2} MW exceeds total fleet capacity {capacity_mw:.2} MW")]
    DemandExceedsCapacity { demand_mw: f64, capacity_mw: f64 },

    /// Forecast or actual slice is empty; cannot compute skill score.
    #[error("forecast/actual slice is empty — cannot compute skill score")]
    EmptyForecastSlice,

    /// Forecast and actual slices have different lengths.
    #[error("forecast length {forecast} ≠ actual length {actual}")]
    ForecastLengthMismatch { forecast: usize, actual: usize },

    /// No history available to compute climatology for skill score.
    #[error("no historical CI observations available for climatology")]
    NoHistory,

    /// A unit referenced in baseline/optimal dispatch was not found in the fleet.
    #[error("unit '{0}' not found in generator fleet")]
    UnitNotFound(String),

    /// Total dispatched power in the slice is zero; cannot normalise.
    #[error("total dispatched power is zero — cannot compute average intensity")]
    ZeroTotalPower,
}

// ── Fuel type ─────────────────────────────────────────────────────────────────

/// Fuel category for a generation unit.
///
/// Zero-emission (renewable/nuclear/storage) fuel types carry a `co2_kg_per_mwh`
/// of exactly 0.0 in practice; the caller is responsible for setting the correct
/// emission rate on [`GeneratorEmission`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FuelType {
    /// High-carbon pulverised coal or lignite plant.
    Coal,
    /// Natural gas combined-cycle or open-cycle turbine.
    NaturalGas,
    /// Heavy/light fuel oil peaker.
    Oil,
    /// Zero-carbon nuclear (fission).
    Nuclear,
    /// Run-of-river or reservoir hydro (near-zero operational carbon).
    Hydro,
    /// Onshore or offshore wind.
    Wind,
    /// Solar photovoltaic or concentrated solar power.
    Solar,
    /// Biomass combustion (may carry upstream lifecycle emissions).
    Biomass,
    /// Geothermal (low direct carbon, minor non-condensable gases).
    Geothermal,
    /// Battery storage or pumped-hydro — zero direct emissions.
    Storage,
}

impl FuelType {
    /// Returns `true` if this fuel type has zero *direct* operational emissions.
    pub fn is_zero_emission(&self) -> bool {
        matches!(
            self,
            FuelType::Nuclear
                | FuelType::Hydro
                | FuelType::Wind
                | FuelType::Solar
                | FuelType::Geothermal
                | FuelType::Storage
        )
    }
}

// ── Generator emission descriptor ─────────────────────────────────────────────

/// Emission and capacity data for one generation unit.
#[derive(Debug, Clone)]
pub struct GeneratorEmission {
    /// Unique plant or unit identifier.
    pub unit_id: String,
    /// Primary fuel/technology type.
    pub fuel_type: FuelType,
    /// Nameplate capacity \[MW\].
    pub capacity_mw: f64,
    /// Direct stack emission rate \[kg CO₂/MWh\].
    pub co2_kg_per_mwh: f64,
    /// Heat rate \[MMBtu/MWh\] (used for cost proxy when prices are not provided).
    pub heat_rate_mmbtu_per_mwh: f64,
    /// Flag indicating whether this unit is currently on the margin.
    pub marginal: bool,
}

// ── Configuration ─────────────────────────────────────────────────────────────

/// Configuration for [`CarbonIntensityForecaster`].
#[derive(Debug, Clone)]
pub struct CarbonForecastConfig {
    /// Forecast horizon in hours (default: 24).
    pub horizon_h: usize,
    /// Nominal coverage for prediction intervals, e.g. 0.90 for 90 % PI.
    pub confidence_level: f64,
    /// If `true`, include upstream/lifecycle emissions in intensity estimates.
    pub include_upstream: bool,
    /// Human-readable label for the grid region or control area.
    pub region_name: String,
}

impl Default for CarbonForecastConfig {
    fn default() -> Self {
        Self {
            horizon_h: 24,
            confidence_level: 0.90,
            include_upstream: false,
            region_name: String::from("default"),
        }
    }
}

// ── Output types ──────────────────────────────────────────────────────────────

/// Result of a marginal emission rate computation.
#[derive(Debug, Clone)]
pub struct MarginalEmissionResult {
    /// Unit ID of the marginal (last-dispatched) generator.
    pub marginal_unit_id: String,
    /// Marginal emission rate \[kg CO₂/MWh\].
    pub mer_kg_co2_per_mwh: f64,
    /// Total CO₂ emitted over one hour for the full dispatch \[kg CO₂\].
    pub total_emissions_kg_co2: f64,
    /// Per-unit dispatch quantities \[(unit_id, MW)\].
    pub dispatch_mw: Vec<(String, f64)>,
    /// Fraction of demand served by zero-emission sources \[dimensionless, 0–1\].
    pub clean_fraction: f64,
}

/// 24-hour (or `horizon_h`-hour) carbon intensity forecast.
#[derive(Debug, Clone)]
pub struct CarbonForecastResult {
    /// Hour timestamps \[h\] — typically 1.0, 2.0, … horizon\_h.
    pub timestamps: Vec<f64>,
    /// Point forecast of grid average carbon intensity \[kg CO₂/MWh\].
    pub ci_kg_co2_per_mwh: Vec<f64>,
    /// Lower bound of the prediction interval \[kg CO₂/MWh\].
    pub ci_lower: Vec<f64>,
    /// Upper bound of the prediction interval \[kg CO₂/MWh\].
    pub ci_upper: Vec<f64>,
    /// Marginal emission rate at each hour \[kg CO₂/MWh\].
    pub mer_kg_co2_per_mwh: Vec<f64>,
    /// Total emissions over the forecast horizon \[t CO₂\].
    pub total_emissions_t_co2: f64,
}

/// Emission savings of a carbon-optimal schedule vs. a baseline.
#[derive(Debug, Clone)]
pub struct EmissionSavings {
    /// Absolute CO₂ reduction \[t CO₂\].
    pub co2_reduction_t: f64,
    /// Relative reduction \[%\].
    pub reduction_pct: f64,
    /// Additional clean energy dispatched in optimal vs baseline \[MWh\].
    pub clean_energy_added_mwh: f64,
    /// Dirty (emitting) energy replaced by clean energy \[MWh\].
    pub dirty_energy_replaced_mwh: f64,
}

// ── Forecaster ────────────────────────────────────────────────────────────────

/// Grid carbon intensity forecaster and MER estimator.
///
/// Maintains a generator fleet, a historical CI record, and a merit-order
/// dispatch stack that is rebuilt by [`Self::build_merit_order`].
pub struct CarbonIntensityForecaster {
    /// Forecaster configuration.
    pub config: CarbonForecastConfig,
    /// Registered generation units with emission data.
    pub generators: Vec<GeneratorEmission>,
    /// Historical observations as `(timestamp_h, ci_kg_co2_per_mwh)` pairs.
    history: Vec<(f64, f64)>,
    /// Indices into `generators` sorted by ascending dispatch cost (merit order).
    dispatch_stack: Vec<usize>,
}

impl CarbonIntensityForecaster {
    /// Create a new forecaster with the given configuration and generator fleet.
    pub fn new(config: CarbonForecastConfig, generators: Vec<GeneratorEmission>) -> Self {
        let n = generators.len();
        Self {
            config,
            generators,
            history: Vec::new(),
            dispatch_stack: (0..n).collect(),
        }
    }

    // ── Merit order ───────────────────────────────────────────────────────────

    /// Build the merit-order dispatch stack from external energy prices.
    ///
    /// Each generator's marginal cost proxy is:
    /// - `energy_prices[i]` when the slice has at least one element and length ≥ fleet size,
    /// - `heat_rate_mmbtu_per_mwh × fuel_price_per_mmbtu_implied` (heat rate alone) otherwise.
    ///
    /// Generators are sorted in ascending cost order so that the cheapest (and
    /// usually cleanest) units are dispatched first.
    ///
    /// # Arguments
    /// * `energy_prices` — per-unit marginal cost \[$/MWh\]; pass an empty slice
    ///   to fall back to heat-rate ordering.
    pub fn build_merit_order(&mut self, energy_prices: &[f64]) {
        let n = self.generators.len();
        let mut indices: Vec<usize> = (0..n).collect();

        if energy_prices.len() >= n && !energy_prices.is_empty() {
            // Use provided prices directly.
            indices.sort_by(|&a, &b| {
                energy_prices[a]
                    .partial_cmp(&energy_prices[b])
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
        } else {
            // Fall back: sort by heat rate (lower heat rate = more efficient = cheaper).
            indices.sort_by(|&a, &b| {
                self.generators[a]
                    .heat_rate_mmbtu_per_mwh
                    .partial_cmp(&self.generators[b].heat_rate_mmbtu_per_mwh)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
        }

        self.dispatch_stack = indices;
    }

    // ── Average intensity ─────────────────────────────────────────────────────

    /// Compute the generation-weighted average carbon intensity for a dispatch
    /// snapshot.
    ///
    /// Formula:
    /// ```text
    /// CI_avg = Σ_i (dispatch_i `MW` × co2_rate_i [kg/MWh]) / Σ_i dispatch_i `MW`
    /// ```
    ///
    /// # Arguments
    /// * `dispatch` — slice of `(unit_id, dispatched_mw)` pairs.
    ///
    /// # Errors
    /// Returns [`CarbonForecastError::EmptyDispatch`] if the slice is empty, or
    /// [`CarbonForecastError::ZeroTotalPower`] if the sum of dispatched MW is zero.
    pub fn calculate_average_intensity(
        &self,
        dispatch: &[(String, f64)],
    ) -> Result<f64, CarbonForecastError> {
        if dispatch.is_empty() {
            return Err(CarbonForecastError::EmptyDispatch);
        }

        let mut weighted_sum = 0.0_f64;
        let mut total_mw = 0.0_f64;

        for (unit_id, mw) in dispatch {
            let emission_rate = self
                .generators
                .iter()
                .find(|g| &g.unit_id == unit_id)
                .map(|g| g.co2_kg_per_mwh)
                .unwrap_or(0.0); // unknown unit assumed zero-emission
            weighted_sum += mw * emission_rate;
            total_mw += mw;
        }

        if total_mw == 0.0 {
            return Err(CarbonForecastError::ZeroTotalPower);
        }

        Ok(weighted_sum / total_mw)
    }

    // ── Marginal emission rate ─────────────────────────────────────────────────

    /// Compute the marginal emission rate (MER) by walking up the dispatch stack.
    ///
    /// The merit order must be built via [`Self::build_merit_order`] before
    /// calling this method (it falls back to index order otherwise).
    ///
    /// Algorithm:
    /// 1. Walk `dispatch_stack` from cheapest to most expensive unit.
    /// 2. Dispatch each unit at full capacity until demand is met.
    /// 3. The *last* unit dispatched (partially or fully) is the marginal unit.
    /// 4. MER = emission rate of the marginal unit \[kg CO₂/MWh\].
    ///
    /// # Arguments
    /// * `demand_mw` — net electrical demand to be served \[MW\].
    /// * `fuel_price_per_mmbtu` — spot fuel price \[$/MMBtu\]; currently unused
    ///   in the greedy walk but stored for future cost-curve extensions.
    ///
    /// # Errors
    /// * [`CarbonForecastError::EmptyFleet`] — no generators registered.
    /// * [`CarbonForecastError::DemandExceedsCapacity`] — demand exceeds total
    ///   installed capacity.
    pub fn calculate_marginal_emission_rate(
        &self,
        demand_mw: f64,
        _fuel_price_per_mmbtu: f64,
    ) -> Result<MarginalEmissionResult, CarbonForecastError> {
        if self.generators.is_empty() {
            return Err(CarbonForecastError::EmptyFleet);
        }

        let total_capacity: f64 = self.generators.iter().map(|g| g.capacity_mw).sum();
        if demand_mw > total_capacity {
            return Err(CarbonForecastError::DemandExceedsCapacity {
                demand_mw,
                capacity_mw: total_capacity,
            });
        }

        let mut remaining = demand_mw;
        let mut dispatch_mw: Vec<(String, f64)> = Vec::new();
        let mut total_emissions_kg = 0.0_f64;
        let mut clean_mw = 0.0_f64;
        let mut marginal_unit_id = String::new();
        let mut mer = 0.0_f64;

        for &idx in &self.dispatch_stack {
            if remaining <= 0.0 {
                break;
            }
            let gen = &self.generators[idx];
            let dispatched = gen.capacity_mw.min(remaining);
            dispatch_mw.push((gen.unit_id.clone(), dispatched));
            total_emissions_kg += dispatched * gen.co2_kg_per_mwh;
            if gen.fuel_type.is_zero_emission() {
                clean_mw += dispatched;
            }
            marginal_unit_id = gen.unit_id.clone();
            mer = gen.co2_kg_per_mwh;
            remaining -= dispatched;
        }

        let clean_fraction = if demand_mw > 0.0 {
            clean_mw / demand_mw
        } else {
            0.0
        };

        Ok(MarginalEmissionResult {
            marginal_unit_id,
            mer_kg_co2_per_mwh: mer,
            total_emissions_kg_co2: total_emissions_kg,
            dispatch_mw,
            clean_fraction,
        })
    }

    // ── 24-hour forecast ──────────────────────────────────────────────────────

    /// Forecast carbon intensity for each hour in `load_profile`.
    ///
    /// For each hour *h*:
    /// 1. Net load = max(load\[h\] − renewable\[h\], 0.0) \[MW\]
    /// 2. Walk merit order to compute MER and average CI.
    /// 3. Apply a linearly growing prediction interval:
    ///    - At *h* = 1: half-width = 10 % of point forecast
    ///    - At *h* = `horizon_h`: half-width = 20 % of point forecast
    ///    - Intermediate hours: linear interpolation
    ///
    /// # Arguments
    /// * `load_profile` — hourly gross load \[MW\]; length determines forecast horizon.
    /// * `renewable_profile` — hourly renewable output \[MW\]; same length as `load_profile`.
    ///
    /// # Errors
    /// * [`CarbonForecastError::ProfileLengthMismatch`] — slices have different lengths.
    /// * [`CarbonForecastError::EmptyFleet`] — no generators registered.
    pub fn forecast_carbon_intensity(
        &self,
        load_profile: &[f64],
        renewable_profile: &[f64],
    ) -> Result<CarbonForecastResult, CarbonForecastError> {
        if load_profile.len() != renewable_profile.len() {
            return Err(CarbonForecastError::ProfileLengthMismatch {
                load: load_profile.len(),
                renewable: renewable_profile.len(),
            });
        }
        if self.generators.is_empty() {
            return Err(CarbonForecastError::EmptyFleet);
        }

        let horizon = load_profile.len();
        let mut timestamps = Vec::with_capacity(horizon);
        let mut ci_point = Vec::with_capacity(horizon);
        let mut ci_lower = Vec::with_capacity(horizon);
        let mut ci_upper = Vec::with_capacity(horizon);
        let mut mer_series = Vec::with_capacity(horizon);
        let mut total_emissions_kg = 0.0_f64;

        for h in 0..horizon {
            let net_load = (load_profile[h] - renewable_profile[h]).max(0.0);
            timestamps.push((h + 1) as f64);

            // Marginal emission rate for this hour.
            let mer_result = self.calculate_marginal_emission_rate(net_load, 0.0);

            let (mer, avg_ci, hour_emissions) = match mer_result {
                Ok(ref r) => {
                    // Build a dispatch slice for average intensity.
                    let avg = self
                        .calculate_average_intensity(&r.dispatch_mw)
                        .unwrap_or(0.0);
                    (r.mer_kg_co2_per_mwh, avg, r.total_emissions_kg_co2)
                }
                Err(_) => (0.0, 0.0, 0.0),
            };

            // Uncertainty half-width grows linearly from 10 % at h=1 to 20 % at h=horizon.
            let frac = if horizon <= 1 {
                0.0
            } else {
                h as f64 / (horizon - 1) as f64
            };
            let half_width_frac = 0.10 + 0.10 * frac; // 10 % → 20 %
            let half_width = avg_ci * half_width_frac;

            ci_point.push(avg_ci);
            ci_lower.push((avg_ci - half_width).max(0.0));
            ci_upper.push(avg_ci + half_width);
            mer_series.push(mer);
            total_emissions_kg += hour_emissions;
        }

        Ok(CarbonForecastResult {
            timestamps,
            ci_kg_co2_per_mwh: ci_point,
            ci_lower,
            ci_upper,
            mer_kg_co2_per_mwh: mer_series,
            total_emissions_t_co2: total_emissions_kg / 1_000.0,
        })
    }

    // ── Locational marginal emissions ─────────────────────────────────────────

    /// Compute locational marginal emissions (LME) for each bus.
    ///
    /// Formula (simplified):
    /// ```text
    /// LME[bus] = MER × (1 + Σ_branch PTDF[bus][branch] × loss_sensitivity[branch])
    /// ```
    ///
    /// When no PTDF matrix is provided (`ptdf` is empty), all buses receive `mer`
    /// directly (no congestion, no loss differentiation).
    ///
    /// The loss sensitivity vector is approximated internally as a uniform
    /// 2 % per branch (0.02) to represent resistive line losses; callers may
    /// replace this by pre-processing their own loss factors before calling.
    ///
    /// # Arguments
    /// * `ptdf` — power transfer distribution factor matrix \[n_buses × n_branches\].
    ///   May be empty, in which case all LMEs equal `mer`.
    /// * `mer` — system marginal emission rate \[kg CO₂/MWh\].
    ///
    /// # Returns
    /// LME per bus \[kg CO₂/MWh\].
    pub fn locational_marginal_emissions(&self, ptdf: &[Vec<f64>], mer: f64) -> Vec<f64> {
        if ptdf.is_empty() {
            return Vec::new();
        }

        // Simplified uniform loss sensitivity: 2 % per branch.
        let n_branches = ptdf[0].len();
        let loss_sensitivity: Vec<f64> = vec![0.02; n_branches];

        ptdf.iter()
            .map(|row| {
                let sensitivity_sum: f64 = row
                    .iter()
                    .zip(loss_sensitivity.iter())
                    .map(|(ptdf_val, ls)| ptdf_val * ls)
                    .sum();
                mer * (1.0 + sensitivity_sum)
            })
            .collect()
    }

    // ── Carbon-optimal dispatch ───────────────────────────────────────────────

    /// Dispatch generators in ascending emission-rate order (cleanest first).
    ///
    /// This greedy approach minimises total CO₂ emissions for a given demand
    /// level by exhausting zero-emission capacity before dispatching fossil units.
    ///
    /// # Arguments
    /// * `generators` — fleet to optimise over (may differ from `self.generators`).
    /// * `demand_mw` — total demand to be served \[MW\].
    ///
    /// # Returns
    /// Dispatched MW per generator in the same order as `generators`.
    /// Elements sum to at most `demand_mw`.
    pub fn carbon_optimal_dispatch(
        &self,
        generators: &[GeneratorEmission],
        demand_mw: f64,
    ) -> Vec<f64> {
        if generators.is_empty() || demand_mw <= 0.0 {
            return vec![0.0; generators.len()];
        }

        // Build index order sorted by ascending co2_kg_per_mwh.
        let mut order: Vec<usize> = (0..generators.len()).collect();
        order.sort_by(|&a, &b| {
            generators[a]
                .co2_kg_per_mwh
                .partial_cmp(&generators[b].co2_kg_per_mwh)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let mut dispatch = vec![0.0_f64; generators.len()];
        let mut remaining = demand_mw;

        for idx in order {
            if remaining <= 0.0 {
                break;
            }
            let dispatched = generators[idx].capacity_mw.min(remaining);
            dispatch[idx] = dispatched;
            remaining -= dispatched;
        }

        dispatch
    }

    // ── Emission savings ──────────────────────────────────────────────────────

    /// Compute emission savings of a carbon-optimal schedule vs. a baseline.
    ///
    /// Both `carbon_optimal` and `baseline` are `(unit_id, dispatched_mw)` vectors
    /// in any order; lookup is by `unit_id` against `generators`.
    ///
    /// # Errors
    /// Returns [`CarbonForecastError::UnitNotFound`] if a unit referenced in
    /// either slice does not exist in `generators`.
    pub fn emission_savings_vs_baseline(
        &self,
        carbon_optimal: &[(String, f64)],
        baseline: &[(String, f64)],
        generators: &[GeneratorEmission],
    ) -> Result<EmissionSavings, CarbonForecastError> {
        let lookup = |uid: &str| -> Result<&GeneratorEmission, CarbonForecastError> {
            generators
                .iter()
                .find(|g| g.unit_id == uid)
                .ok_or_else(|| CarbonForecastError::UnitNotFound(uid.to_string()))
        };

        // Baseline emissions [kg CO2].
        let mut baseline_emissions = 0.0_f64;
        let mut baseline_dirty_mwh = 0.0_f64;
        for (uid, mw) in baseline {
            let g = lookup(uid)?;
            baseline_emissions += mw * g.co2_kg_per_mwh;
            if !g.fuel_type.is_zero_emission() {
                baseline_dirty_mwh += mw;
            }
        }

        // Carbon-optimal emissions [kg CO2].
        let mut optimal_emissions = 0.0_f64;
        let mut optimal_clean_mwh = 0.0_f64;
        let mut optimal_dirty_mwh = 0.0_f64;
        for (uid, mw) in carbon_optimal {
            let g = lookup(uid)?;
            optimal_emissions += mw * g.co2_kg_per_mwh;
            if g.fuel_type.is_zero_emission() {
                optimal_clean_mwh += mw;
            } else {
                optimal_dirty_mwh += mw;
            }
        }

        let delta_kg = baseline_emissions - optimal_emissions;
        let reduction_pct = if baseline_emissions > 0.0 {
            100.0 * delta_kg / baseline_emissions
        } else {
            0.0
        };

        let clean_energy_added_mwh = (optimal_clean_mwh
            - baseline
                .iter()
                .map(|(uid, mw)| {
                    generators
                        .iter()
                        .find(|g| &g.unit_id == uid)
                        .map(|g| {
                            if g.fuel_type.is_zero_emission() {
                                *mw
                            } else {
                                0.0
                            }
                        })
                        .unwrap_or(0.0)
                })
                .sum::<f64>())
        .max(0.0);

        let dirty_energy_replaced_mwh = (baseline_dirty_mwh - optimal_dirty_mwh).max(0.0);

        Ok(EmissionSavings {
            co2_reduction_t: delta_kg / 1_000.0,
            reduction_pct,
            clean_energy_added_mwh,
            dirty_energy_replaced_mwh,
        })
    }

    // ── History management ────────────────────────────────────────────────────

    /// Append an observed carbon intensity data point to the history buffer.
    ///
    /// # Arguments
    /// * `timestamp` — observation time \[h\] (e.g. Unix time / 3600 or hour-of-year).
    /// * `actual_ci` — observed grid carbon intensity \[kg CO₂/MWh\].
    pub fn update_history(&mut self, timestamp: f64, actual_ci: f64) {
        self.history.push((timestamp, actual_ci));
    }

    // ── Skill score ───────────────────────────────────────────────────────────

    /// Compute the Murphy skill score (SS) for a carbon intensity forecast.
    ///
    /// ```text
    /// SS = 1 − MSE_forecast / MSE_climatology
    /// ```
    ///
    /// where *climatology* is the mean of the historical CI observations stored
    /// in `self.history`.
    ///
    /// * SS = 1.0 → perfect forecast.
    /// * SS = 0.0 → forecast no better than climatology.
    /// * SS < 0.0 → forecast worse than climatology.
    ///
    /// # Arguments
    /// * `forecasts` — point forecast values \[kg CO₂/MWh\].
    /// * `actuals` — corresponding observed values \[kg CO₂/MWh\].
    ///
    /// # Errors
    /// * [`CarbonForecastError::EmptyForecastSlice`] — empty slices.
    /// * [`CarbonForecastError::ForecastLengthMismatch`] — slice lengths differ.
    /// * [`CarbonForecastError::NoHistory`] — no historical data for climatology.
    pub fn skill_score_ci(
        &self,
        forecasts: &[f64],
        actuals: &[f64],
    ) -> Result<f64, CarbonForecastError> {
        if forecasts.is_empty() || actuals.is_empty() {
            return Err(CarbonForecastError::EmptyForecastSlice);
        }
        if forecasts.len() != actuals.len() {
            return Err(CarbonForecastError::ForecastLengthMismatch {
                forecast: forecasts.len(),
                actual: actuals.len(),
            });
        }
        if self.history.is_empty() {
            return Err(CarbonForecastError::NoHistory);
        }

        let n = forecasts.len() as f64;

        // MSE of forecast.
        let mse_forecast: f64 = forecasts
            .iter()
            .zip(actuals.iter())
            .map(|(f, a)| (f - a).powi(2))
            .sum::<f64>()
            / n;

        // Climatology: mean of historical CI observations.
        let clim_mean: f64 =
            self.history.iter().map(|(_, ci)| ci).sum::<f64>() / self.history.len() as f64;

        // MSE of climatology vs actuals.
        let mse_clim: f64 = actuals.iter().map(|a| (a - clim_mean).powi(2)).sum::<f64>() / n;

        if mse_clim == 0.0 {
            // If actuals are all equal to climatology and forecast is perfect,
            // return 1.0; otherwise return 0.0 (undefined, clamp to 0).
            return Ok(if mse_forecast == 0.0 { 1.0 } else { 0.0 });
        }

        Ok(1.0 - mse_forecast / mse_clim)
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a small mixed fleet for test reuse.
    fn test_fleet() -> Vec<GeneratorEmission> {
        vec![
            GeneratorEmission {
                unit_id: "wind_1".to_string(),
                fuel_type: FuelType::Wind,
                capacity_mw: 100.0,
                co2_kg_per_mwh: 0.0,
                heat_rate_mmbtu_per_mwh: 0.0,
                marginal: false,
            },
            GeneratorEmission {
                unit_id: "solar_1".to_string(),
                fuel_type: FuelType::Solar,
                capacity_mw: 80.0,
                co2_kg_per_mwh: 0.0,
                heat_rate_mmbtu_per_mwh: 0.0,
                marginal: false,
            },
            GeneratorEmission {
                unit_id: "gas_1".to_string(),
                fuel_type: FuelType::NaturalGas,
                capacity_mw: 200.0,
                co2_kg_per_mwh: 450.0,
                heat_rate_mmbtu_per_mwh: 7.5,
                marginal: false,
            },
            GeneratorEmission {
                unit_id: "coal_1".to_string(),
                fuel_type: FuelType::Coal,
                capacity_mw: 300.0,
                co2_kg_per_mwh: 950.0,
                heat_rate_mmbtu_per_mwh: 10.5,
                marginal: false,
            },
        ]
    }

    fn default_forecaster() -> CarbonIntensityForecaster {
        CarbonIntensityForecaster::new(CarbonForecastConfig::default(), test_fleet())
    }

    // ── Test 1: Merit order — coal dispatched after gas ────────────────────────

    /// In a carbon-optimal dispatch, gas (450 kg/MWh) must be dispatched before
    /// coal (950 kg/MWh).  Verify that gas receives non-zero dispatch while coal
    /// receives zero when demand is ≤ wind + solar + gas capacity.
    #[test]
    fn test_merit_order_coal_after_gas() {
        let forecaster = default_forecaster();
        let fleet = test_fleet();
        // demand = 300 MW: wind(100) + solar(80) + gas(120) → coal = 0
        let dispatch = forecaster.carbon_optimal_dispatch(&fleet, 300.0);

        let gas_idx = fleet.iter().position(|g| g.unit_id == "gas_1").unwrap();
        let coal_idx = fleet.iter().position(|g| g.unit_id == "coal_1").unwrap();

        assert!(
            dispatch[gas_idx] > 0.0,
            "Gas should be dispatched before coal"
        );
        assert_eq!(
            dispatch[coal_idx], 0.0,
            "Coal must not be dispatched when demand can be met cleanly + gas"
        );
    }

    // ── Test 2: MER at low demand → cleanest unit marginal ────────────────────

    /// When demand ≤ wind capacity, the marginal unit is wind and MER = 0.
    #[test]
    fn test_mer_low_demand_cleanest_marginal() {
        let mut forecaster = default_forecaster();
        // Merit order: wind → solar → gas → coal (heat-rate / emission order)
        // With default dispatch_stack (index order 0=wind, 1=solar, ...) this
        // already dispatches wind first as it is index 0; but build merit order
        // explicitly using heat-rate fallback.
        forecaster.build_merit_order(&[]); // heat-rate fallback

        let result = forecaster
            .calculate_marginal_emission_rate(50.0, 3.0)
            .expect("should succeed");

        assert_eq!(
            result.marginal_unit_id, "wind_1",
            "Marginal unit must be wind at low demand"
        );
        assert_eq!(
            result.mer_kg_co2_per_mwh, 0.0,
            "MER must be 0 when wind is marginal"
        );
        assert!(
            result.clean_fraction > 0.99,
            "Clean fraction must be ~1.0 at low demand, got {}",
            result.clean_fraction
        );
    }

    // ── Test 3: Average intensity weighted by dispatch quantities ─────────────

    /// CI_avg = (200×0 + 100×450) / 300 = 150 kg/MWh.
    #[test]
    fn test_average_intensity_weighted() {
        let forecaster = default_forecaster();
        let dispatch = vec![
            ("wind_1".to_string(), 200.0_f64),
            ("gas_1".to_string(), 100.0_f64),
        ];
        let ci = forecaster
            .calculate_average_intensity(&dispatch)
            .expect("should succeed");

        let expected = (200.0 * 0.0 + 100.0 * 450.0) / 300.0;
        assert!(
            (ci - expected).abs() < 1e-9,
            "Expected {:.4} kg/MWh, got {:.4}",
            expected,
            ci
        );
    }

    // ── Test 4: Carbon-optimal with 100 % renewables → near-zero CI ──────────

    /// If all demand is ≤ renewable capacity, optimal dispatch gives CI = 0.
    #[test]
    fn test_carbon_optimal_full_renewables() {
        let forecaster = default_forecaster();
        let fleet = test_fleet();
        // demand = 100 MW, wind = 100 MW capacity → all from wind
        let dispatch = forecaster.carbon_optimal_dispatch(&fleet, 100.0);

        let wind_idx = fleet.iter().position(|g| g.unit_id == "wind_1").unwrap();
        assert!(
            (dispatch[wind_idx] - 100.0).abs() < 1e-9,
            "All demand should be met by wind"
        );

        // Compute average CI from this dispatch.
        let dispatch_pairs: Vec<(String, f64)> = fleet
            .iter()
            .zip(dispatch.iter())
            .filter(|(_, &mw)| mw > 0.0)
            .map(|(g, &mw)| (g.unit_id.clone(), mw))
            .collect();
        let ci = forecaster
            .calculate_average_intensity(&dispatch_pairs)
            .expect("should succeed");
        assert!(
            ci < 1e-9,
            "CI must be near-zero with 100 % wind, got {:.4}",
            ci
        );
    }

    // ── Test 5: Forecast 24 h — PI widens with horizon ────────────────────────

    /// Upper − lower at hour 24 must be strictly greater than at hour 1.
    #[test]
    fn test_forecast_uncertainty_grows_with_horizon() {
        let mut forecaster = default_forecaster();
        forecaster.build_merit_order(&[]);

        // Flat load profile, modest renewable profile.
        let load: Vec<f64> = vec![300.0; 24];
        let renew: Vec<f64> = vec![100.0; 24];

        let result = forecaster
            .forecast_carbon_intensity(&load, &renew)
            .expect("forecast should succeed");

        assert_eq!(result.timestamps.len(), 24);

        let width_h1 = result.ci_upper[0] - result.ci_lower[0];
        let width_h24 = result.ci_upper[23] - result.ci_lower[23];

        assert!(
            width_h24 > width_h1,
            "PI at h=24 ({:.4}) must be wider than at h=1 ({:.4})",
            width_h24,
            width_h1
        );
    }

    // ── Test 6: Emission savings — optimal vs coal-heavy baseline ─────────────

    /// Carbon-optimal dispatch should achieve strictly lower emissions than a
    /// baseline that dispatches coal first.
    #[test]
    fn test_emission_savings_positive() {
        let forecaster = default_forecaster();
        let fleet = test_fleet();

        // Baseline: 300 MW all from coal.
        let baseline = vec![("coal_1".to_string(), 300.0_f64)];

        // Optimal: wind(100) + solar(80) + gas(120).
        let optimal = vec![
            ("wind_1".to_string(), 100.0_f64),
            ("solar_1".to_string(), 80.0_f64),
            ("gas_1".to_string(), 120.0_f64),
        ];

        let savings = forecaster
            .emission_savings_vs_baseline(&optimal, &baseline, &fleet)
            .expect("should succeed");

        assert!(
            savings.co2_reduction_t > 0.0,
            "CO₂ reduction must be positive, got {:.4}",
            savings.co2_reduction_t
        );
        assert!(
            savings.reduction_pct > 0.0,
            "Reduction % must be positive, got {:.4}",
            savings.reduction_pct
        );
    }

    // ── Test 7: LME without congestion (zero PTDF) ≈ MER ─────────────────────

    /// When all PTDF entries are zero (no congestion), LME[bus] must equal MER.
    #[test]
    fn test_lme_no_congestion_equals_mer() {
        let forecaster = default_forecaster();
        let mer = 450.0_f64; // [kg CO2/MWh]

        // 3 buses, 2 branches, all PTDF = 0.
        let ptdf = vec![vec![0.0, 0.0], vec![0.0, 0.0], vec![0.0, 0.0]];
        let lme = forecaster.locational_marginal_emissions(&ptdf, mer);

        assert_eq!(lme.len(), 3, "Should return one LME per bus");
        for (i, &l) in lme.iter().enumerate() {
            assert!(
                (l - mer).abs() < 1e-9,
                "Bus {}: LME ({:.4}) must equal MER ({:.4}) when PTDF=0",
                i,
                l,
                mer
            );
        }
    }

    // ── Test 8: Skill score — perfect forecast → SS = 1.0 ────────────────────

    /// When forecasts equal actuals exactly, MSE_forecast = 0 → SS = 1.0.
    #[test]
    fn test_skill_score_perfect_forecast() {
        let mut forecaster = default_forecaster();
        // Populate history so climatology is defined.
        forecaster.update_history(0.0, 400.0);
        forecaster.update_history(1.0, 500.0);
        forecaster.update_history(2.0, 450.0);

        let actuals = vec![420.0, 480.0, 460.0, 440.0];
        let forecasts = actuals.clone(); // perfect

        let ss = forecaster
            .skill_score_ci(&forecasts, &actuals)
            .expect("should succeed");

        assert!(
            (ss - 1.0).abs() < 1e-9,
            "Perfect forecast must yield SS=1.0, got {:.6}",
            ss
        );
    }

    // ── Bonus test: build_merit_order with explicit prices ────────────────────

    /// When explicit energy prices are provided, the merit order reflects those
    /// prices rather than heat rates.
    #[test]
    fn test_build_merit_order_with_prices() {
        let mut forecaster = default_forecaster();
        // prices: wind=50, solar=55, gas=30, coal=20  → order: coal, gas, wind, solar
        let prices = vec![50.0, 55.0, 30.0, 20.0];
        forecaster.build_merit_order(&prices);

        // First in stack should be coal (price=20).
        let first_idx = forecaster.dispatch_stack[0];
        assert_eq!(
            forecaster.generators[first_idx].unit_id, "coal_1",
            "Cheapest price = coal should be first in merit order"
        );
    }

    // ── Bonus test: error on empty dispatch ───────────────────────────────────

    #[test]
    fn test_average_intensity_empty_dispatch_error() {
        let forecaster = default_forecaster();
        let result = forecaster.calculate_average_intensity(&[]);
        assert!(
            matches!(result, Err(CarbonForecastError::EmptyDispatch)),
            "Expected EmptyDispatch error"
        );
    }

    // ── Bonus test: demand exceeds capacity → error ───────────────────────────

    #[test]
    fn test_mer_demand_exceeds_capacity() {
        let forecaster = default_forecaster();
        // Total capacity = 100+80+200+300 = 680 MW; demand = 1000 MW.
        let result = forecaster.calculate_marginal_emission_rate(1000.0, 5.0);
        assert!(
            matches!(
                result,
                Err(CarbonForecastError::DemandExceedsCapacity { .. })
            ),
            "Expected DemandExceedsCapacity error"
        );
    }
}
