//! Grid-Scale Hydrogen Seasonal Storage Optimization.
//!
//! Models underground bulk hydrogen storage (salt caverns, lined rock caverns,
//! underground tanks, pipeline buffers) and optimizes dispatch over a full year
//! to capture seasonal renewable surplus and perform long-duration energy arbitrage.
//!
//! # Units Convention
//! - Energy:    \[MWh\], \[kWh\]
//! - Power:     \[MW\]
//! - Mass:      \[kg\], \[t\]
//! - Pressure:  \[bar\]
//! - Volume:    \[m³\]
//! - Cost:      \[USD\] or \[$/kg\], \[$/MWh\]

// ─── Physical constants ───────────────────────────────────────────────────────
/// Universal gas constant \[J/(mol·K)\]
const R_J_PER_MOL_K: f64 = 8.314_462_618;
/// Molar mass of H₂ \[kg/mol\]
const M_H2_KG_PER_MOL: f64 = 2.016e-3;
/// Reference temperature for gas calculations \[K\] (25 °C)
const T_REF_K: f64 = 298.15;
/// Reference pressure \[Pa\] (1 bar)
const P_REF_PA: f64 = 1.0e5;
/// Hours per week
const HOURS_PER_WEEK: usize = 168;
/// kg per metric tonne
const KG_PER_TONNE: f64 = 1_000.0;
/// Lower heating value of hydrogen \[kWh/kg\]
pub const H2_LHV_KWH_PER_KG: f64 = 33.3;
/// EU Taxonomy green hydrogen carbon-intensity threshold \[gCO₂/kgH₂\]
pub const GREEN_H2_THRESHOLD_G_CO2_PER_KG: f64 = 1_000.0;

// ─── Storage geometry types ───────────────────────────────────────────────────

/// Underground hydrogen storage technology.
///
/// Each variant carries geometry/pressure parameters needed to estimate
/// the *working* (usable) hydrogen mass that can be stored.
#[derive(Debug, Clone)]
pub enum SeasonalStorageType {
    /// Salt-cavern storage — the dominant technology for TWh-scale seasonal storage.
    ///
    /// The working gas volume is the gas stored between `min_pressure_bar` and
    /// `max_pressure_bar`; cushion gas (at min pressure) is not usable.
    SaltCavern {
        /// Usable geometric gas volume at operating pressure conditions \[m³\]
        working_volume_m3: f64,
        /// Minimum operating pressure (bottom-of-cycle) \[bar\]
        min_pressure_bar: f64,
        /// Maximum operating pressure (top-of-cycle) \[bar\]
        max_pressure_bar: f64,
        /// Fraction of total inventory that must remain as cushion gas \[–\]
        cushion_gas_pct: f64,
    },
    /// Lined rock cavern (LRC) — pressurised concrete-lined excavation.
    LinerCavern {
        /// Geometric volume of the lined cavity \[m³\]
        volume_m3: f64,
        /// Single (fixed) operating pressure \[bar\]
        operating_pressure_bar: f64,
    },
    /// Above- or below-ground insulated pressure vessel farm.
    UndergroundTank {
        /// Nameplate hydrogen storage capacity \[t H₂\]
        capacity_t_h2: f64,
        /// Operating pressure \[bar\]
        pressure_bar: f64,
    },
    /// High-pressure pipeline used as a buffer storage medium ("line-pack").
    PipelineBuffer {
        /// Total pipeline length \[km\]
        pipe_length_km: f64,
        /// Internal pipe diameter \[m\]
        diameter_m: f64,
        /// Maximum allowable operating pressure \[bar\]
        max_pressure_bar: f64,
    },
}

// ─── Equipment fleets ─────────────────────────────────────────────────────────

/// Aggregate electrolyzer fleet (PEM / alkaline / SOEC — technology-agnostic here).
#[derive(Debug, Clone)]
pub struct ElectrolyzerFleet {
    /// Installed electrical capacity of the fleet \[MW\]
    pub capacity_mw: f64,
    /// Specific electricity consumption to produce 1 kg H₂ \[kWh/kg\]
    pub efficiency_kwh_per_kg: f64,
    /// Minimum stable operating point as a fraction of rated capacity \[–\]
    pub min_load_pct: f64,
    /// Cold-start time to reach rated output \[min\]
    pub startup_time_min: f64,
    /// Maximum ramp rate \[% of rated capacity per minute\]
    pub ramp_rate_pct_per_min: f64,
}

impl Default for ElectrolyzerFleet {
    fn default() -> Self {
        Self {
            capacity_mw: 100.0,
            efficiency_kwh_per_kg: 55.0,
            min_load_pct: 0.10,
            startup_time_min: 30.0,
            ramp_rate_pct_per_min: 10.0,
        }
    }
}

/// Aggregate fuel-cell / gas-turbine fleet for H₂ → electricity reconversion.
#[derive(Debug, Clone)]
pub struct FuelCellFleet {
    /// Installed electrical output capacity \[MW\]
    pub capacity_mw: f64,
    /// Specific H₂ consumption to generate 1 MWh electricity \[kg/MWh\]
    ///
    /// Equivalently: efficiency = LHV / (efficiency_kwh_per_kg / 1000) — kept as an
    /// opaque field so users can set it directly from vendor datasheets.
    pub efficiency_kwh_per_kg: f64,
    /// Minimum stable operating point as a fraction of rated capacity \[–\]
    pub min_load_pct: f64,
    /// Cold-start time \[min\]
    pub startup_time_min: f64,
}

impl Default for FuelCellFleet {
    fn default() -> Self {
        Self {
            capacity_mw: 50.0,
            efficiency_kwh_per_kg: 20.0,
            min_load_pct: 0.10,
            startup_time_min: 15.0,
        }
    }
}

// ─── Optimiser configuration ──────────────────────────────────────────────────

/// Configuration for the seasonal storage optimiser.
///
/// All `Vec` fields must have length == `planning_weeks`.
#[derive(Debug, Clone)]
pub struct SeasonalStorageConfig {
    /// Planning horizon \[weeks\] (52 for a full calendar year)
    pub planning_weeks: usize,
    /// H₂ lower heating value \[kWh/kg\] (default 33.3)
    pub h2_lower_heating_value_kwh_per_kg: f64,
    /// Weekly average wholesale electricity price \[$/MWh\]
    pub electricity_price_seasonal: Vec<f64>,
    /// Weekly average renewable surplus available to the electrolyzer \[MW\]
    pub renewable_surplus_mw: Vec<f64>,
    /// Weekly H₂ offtake demand (industry / transport) \[t/week\]
    pub h2_demand_t_per_week: Vec<f64>,
    /// Annual discount rate \[–\] (e.g. 0.08 for 8 %)
    pub discount_rate: f64,
    /// Annual OPEX as a fraction of initial CAPEX \[–\]
    pub opex_fraction: f64,
}

impl Default for SeasonalStorageConfig {
    fn default() -> Self {
        // Flat defaults — callers should always supply real price / surplus data.
        let weeks = 52_usize;
        Self {
            planning_weeks: weeks,
            h2_lower_heating_value_kwh_per_kg: H2_LHV_KWH_PER_KG,
            electricity_price_seasonal: vec![50.0; weeks],
            renewable_surplus_mw: vec![200.0; weeks],
            h2_demand_t_per_week: vec![100.0; weeks],
            discount_rate: 0.08,
            opex_fraction: 0.03,
        }
    }
}

// ─── Output structs ───────────────────────────────────────────────────────────

/// Dispatch schedule and financial summary for a single week.
#[derive(Debug, Clone)]
pub struct WeeklyDispatch {
    /// Calendar week index (0-based)
    pub week: usize,
    /// Hydrogen produced by electrolysis during the week \[kg\]
    pub h2_produced_kg: f64,
    /// Hydrogen consumed by fuel cells during the week \[kg\]
    pub h2_consumed_kg: f64,
    /// Hydrogen sold directly to market (demand fulfilment) \[kg\]
    pub h2_to_market_kg: f64,
    /// Electricity generated by fuel cells \[MWh\]
    pub electricity_generated_mwh: f64,
    /// Electricity consumed by electrolyzers \[MWh\]
    pub electricity_consumed_mwh: f64,
    /// Hydrogen inventory at end of week \[kg\]
    pub end_inventory_kg: f64,
    /// Gross revenues: electricity sales + H₂ sales \[USD\]
    pub revenue_usd: f64,
    /// Gross costs: electricity purchases \[USD\]
    pub cost_usd: f64,
}

/// Full-year simulation summary.
#[derive(Debug, Clone)]
pub struct YearlySimResult {
    /// Per-week dispatch records
    pub weekly_results: Vec<WeeklyDispatch>,
    /// Total H₂ produced over the year \[t\]
    pub total_h2_produced_t: f64,
    /// Total gross revenue \[USD\]
    pub total_revenue_usd: f64,
    /// Total gross cost \[USD\]
    pub total_cost_usd: f64,
    /// Net profit = revenue − cost \[USD\]
    pub net_profit_usd: f64,
    /// System round-trip efficiency (electricity out / electricity in) \[%\]
    pub round_trip_efficiency_pct: f64,
    /// Average storage utilisation over the year \[%\]
    pub storage_utilization_pct: f64,
    /// Cumulative unserved H₂ demand \[t\]
    pub unserved_h2_demand_t: f64,
}

/// Economic assessment results.
#[derive(Debug, Clone)]
pub struct H2EconomicsResult {
    /// Levelised cost of hydrogen \[$/kg\]
    pub lcoh_per_kg: f64,
    /// Net present value of the system over its lifetime \[USD\]
    pub npv_usd: f64,
    /// Simple payback period \[years\]
    pub payback_years: f64,
    /// Annual revenue (from simulation or estimate) \[USD\]
    pub annual_revenue_usd: f64,
    /// Whether H₂ qualifies as "green" under the EU taxonomy
    pub is_green_hydrogen: bool,
    /// Carbon intensity of produced H₂ \[gCO₂/kgH₂\]
    pub carbon_intensity_g_co2_per_kg: f64,
}

// ─── Main optimiser ───────────────────────────────────────────────────────────

/// Grid-scale seasonal hydrogen storage optimiser.
///
/// Combines an underground storage asset, an electrolyzer fleet, and a fuel-cell
/// fleet into a single dispatch model.  The weekly optimisation uses a
/// price-threshold heuristic (percentile bands) which is fast and interpretable;
/// an exact MILP can be layered on top via the `oxiz` crate if needed.
#[derive(Clone)]
pub struct SeasonalH2Optimizer {
    /// Bulk storage asset
    pub storage: SeasonalStorageType,
    /// Electrolyzer fleet (H₂ production side)
    pub electrolyzers: ElectrolyzerFleet,
    /// Fuel-cell / reconversion fleet (H₂ → electricity)
    pub fuel_cells: FuelCellFleet,
    /// Optimiser / scenario configuration
    pub config: SeasonalStorageConfig,
}

impl SeasonalH2Optimizer {
    // ─── Capacity helpers ─────────────────────────────────────────────────────

    /// Maximum working (usable) hydrogen inventory of the storage asset \[kg\].
    ///
    /// ## Method
    /// For gas-law types we use the ideal-gas law as a first-order approximation:
    ///
    /// ```text
    /// n_H₂ [mol] = (P [Pa] × V `m³`) / (R [J/(mol·K)] × T `K`)
    /// m_H₂ `kg`  = n_H₂ × M_H₂ [kg/mol]
    /// ```
    ///
    /// For salt caverns the working mass is the gas stored between `min_pressure_bar`
    /// and `max_pressure_bar`; cushion gas is excluded.
    /// For tanks the nameplate capacity is used directly.
    pub fn storage_capacity_kg(&self) -> f64 {
        match &self.storage {
            SeasonalStorageType::SaltCavern {
                working_volume_m3,
                min_pressure_bar,
                max_pressure_bar,
                cushion_gas_pct,
            } => {
                // Ideal-gas working inventory between min and max pressure.
                let delta_p_pa = (max_pressure_bar - min_pressure_bar) * P_REF_PA;
                let total_mass_kg =
                    (delta_p_pa * working_volume_m3) / (R_J_PER_MOL_K * T_REF_K) * M_H2_KG_PER_MOL;
                // Subtract cushion fraction (cushion is at min pressure, hence
                // already excluded by delta_p — but the field allows an additional
                // engineering safety margin on top of the pressure swing).
                total_mass_kg * (1.0 - cushion_gas_pct.clamp(0.0, 0.99))
            }
            SeasonalStorageType::LinerCavern {
                volume_m3,
                operating_pressure_bar,
            } => {
                // Full volume at single operating pressure (5 % safety reserve built in).
                let p_pa = operating_pressure_bar * P_REF_PA;
                let total_mass_kg =
                    (p_pa * volume_m3) / (R_J_PER_MOL_K * T_REF_K) * M_H2_KG_PER_MOL;
                total_mass_kg * 0.95
            }
            SeasonalStorageType::UndergroundTank { capacity_t_h2, .. } => {
                capacity_t_h2 * KG_PER_TONNE
            }
            SeasonalStorageType::PipelineBuffer {
                pipe_length_km,
                diameter_m,
                max_pressure_bar,
            } => {
                // Geometric volume of a cylinder: V = π/4 × D² × L
                let length_m = pipe_length_km * 1_000.0;
                let volume_m3 = std::f64::consts::FRAC_PI_4 * diameter_m * diameter_m * length_m;
                // Line-pack between ~1 bar (delivery pressure) and max_pressure.
                let delta_p_pa = (max_pressure_bar - 1.0).max(0.0) * P_REF_PA;
                (delta_p_pa * volume_m3) / (R_J_PER_MOL_K * T_REF_K) * M_H2_KG_PER_MOL
            }
        }
    }

    // ─── Electrolyzer dispatch (single hour) ──────────────────────────────────

    /// Compute H₂ production for one hour given operating conditions.
    ///
    /// # Arguments
    /// * `renewable_surplus_mw` — available renewable power surplus \[MW\]
    /// * `h2_price_per_kg`      — current H₂ spot price \[$/kg\]
    /// * `electricity_price`    — marginal electricity cost \[$/MWh\]
    ///
    /// # Returns
    /// H₂ produced \[kg/h\], clamped to fleet capacity and ramp constraints.
    pub fn electrolyzer_dispatch(
        &self,
        renewable_surplus_mw: f64,
        h2_price_per_kg: f64,
        electricity_price: f64,
    ) -> f64 {
        let eff = self.electrolyzers.efficiency_kwh_per_kg;
        // Break-even electricity price: cost of producing 1 kg equals its value.
        // electricity_cost = eff [kWh/kg] × price [$/MWh] / 1000 [kWh/MWh]  [$/kg]
        let break_even_price = h2_price_per_kg * 1_000.0 / eff; // $/MWh

        if electricity_price >= break_even_price {
            // Uneconomic — do not electrolyze.
            return 0.0;
        }

        // Maximum power limited by fleet capacity and available surplus.
        let available_mw = renewable_surplus_mw.min(self.electrolyzers.capacity_mw);
        if available_mw <= 0.0 {
            return 0.0;
        }

        // Ramp limit: maximum fraction reachable in one hour from cold start.
        // ramp_rate is %/min → fraction/min; 60 min per hour.
        let ramp_fraction = (self.electrolyzers.ramp_rate_pct_per_min / 100.0 * 60.0).min(1.0);
        let max_load_pct = ramp_fraction;
        let min_load_pct = self.electrolyzers.min_load_pct;

        let load_fraction = (available_mw / self.electrolyzers.capacity_mw)
            .min(max_load_pct)
            .max(0.0);

        if load_fraction < min_load_pct {
            // Below minimum stable load — shut down.
            return 0.0;
        }

        let power_mw = load_fraction * self.electrolyzers.capacity_mw;
        // Production [kg/h] = power [MW] × 1000 [kWh/MWh] / eff [kWh/kg]
        power_mw * 1_000.0 / eff
    }

    // ─── Fuel cell dispatch (single hour) ────────────────────────────────────

    /// Compute electricity generation from the fuel-cell fleet for one hour.
    ///
    /// # Arguments
    /// * `h2_inventory_kg`    — current hydrogen inventory available \[kg\]
    /// * `electricity_price`  — electricity spot price \[$/MWh\]
    /// * `h2_price_per_kg`    — H₂ opportunity cost \[$/kg\]
    ///
    /// # Returns
    /// Electricity generated \[MWh/h\].
    pub fn fuel_cell_dispatch(
        &self,
        h2_inventory_kg: f64,
        electricity_price: f64,
        h2_price_per_kg: f64,
    ) -> f64 {
        let eff = self.fuel_cells.efficiency_kwh_per_kg; // kWh/kg → kWh per kg consumed
                                                         // Opportunity cost threshold: value of H₂ converted to electricity equivalent.
                                                         // 1 kg H₂ → eff kWh electricity → eff/1000 MWh at price $/MWh → $ revenue
                                                         // threshold [$/MWh] = H2_price [$/kg] / (eff/1000) [MWh/kg]
        let threshold = h2_price_per_kg * 1_000.0 / eff;

        if electricity_price <= threshold {
            return 0.0;
        }
        if h2_inventory_kg <= 0.0 {
            return 0.0;
        }

        // Maximum output constrained by fleet capacity.
        let max_output_mw = self.fuel_cells.capacity_mw;
        // H₂ needed for full output in 1 h: output_mwh / (eff/1000) kg/MWh
        let max_h2_per_h_kg = max_output_mw * 1_000.0 / eff;
        // Can't consume more than available inventory.
        let h2_consumed = max_h2_per_h_kg.min(h2_inventory_kg);

        // Electricity output [MWh] = h2_consumed [kg] × eff [kWh/kg] / 1000
        h2_consumed * eff / 1_000.0
    }

    // ─── Weekly optimisation ──────────────────────────────────────────────────

    /// Optimise dispatch for a single week using a price-percentile heuristic.
    ///
    /// Strategy:
    /// - **p20 band** (cheapest 20 % of hours): electrolyze if storage not full.
    /// - **p80 band** (most expensive 20 % of hours): generate if storage > 10 % full.
    /// - **H₂ demand**: serve from storage at any hour (before arbitrage).
    ///
    /// # Arguments
    /// * `week_idx`        — zero-based week index
    /// * `h2_inventory_kg` — inventory at the start of the week \[kg\]
    ///
    /// # Returns
    /// [`WeeklyDispatch`] with hourly aggregates and financials.
    pub fn optimize_weekly_dispatch(
        &self,
        week_idx: usize,
        h2_inventory_kg: f64,
    ) -> WeeklyDispatch {
        let weeks = self.config.planning_weeks;
        let w = week_idx.min(weeks.saturating_sub(1));

        let elec_price = self
            .config
            .electricity_price_seasonal
            .get(w)
            .copied()
            .unwrap_or(50.0);
        let surplus_mw = self
            .config
            .renewable_surplus_mw
            .get(w)
            .copied()
            .unwrap_or(0.0);
        let demand_t = self
            .config
            .h2_demand_t_per_week
            .get(w)
            .copied()
            .unwrap_or(0.0);
        let demand_kg = demand_t * KG_PER_TONNE;

        let capacity_kg = self.storage_capacity_kg();

        // Simulate intra-week price variation using a ±30 % swing around the
        // weekly average.  The 168 hours are divided into three bands:
        //   - Hours  0.. 56: "cheap"    → price × 0.70  (off-peak / surplus)
        //   - Hours 56..112: "mid"       → price × 1.00
        //   - Hours 112..168: "expensive" → price × 1.30  (peak demand)
        // This ensures the dispatch heuristic sees both low- and high-price
        // hours even when only a weekly average is available.
        let price_cheap = elec_price * 0.70;
        let price_mid = elec_price;
        let price_peak = elec_price * 1.30;

        // H₂ price heuristic: $3/kg base, adjusted by electricity cost.
        let h2_price = 3.0 + elec_price * 0.01;

        // Break-even electricity price for electrolysis ($/MWh).
        let break_even =
            h2_price * 1_000.0 / self.electrolyzers.efficiency_kwh_per_kg.max(f64::EPSILON);
        // Dispatch threshold for fuel cell: $/MWh above which reconversion is worthwhile.
        let fc_threshold =
            h2_price * 1_000.0 / self.fuel_cells.efficiency_kwh_per_kg.max(f64::EPSILON);

        let mut inventory = h2_inventory_kg.max(0.0);
        let mut h2_produced = 0.0_f64;
        let mut h2_consumed_fc = 0.0_f64;
        let mut h2_to_market = 0.0_f64;
        let mut elec_generated = 0.0_f64;
        let mut elec_consumed = 0.0_f64;
        let mut revenue = 0.0_f64;
        let mut cost = 0.0_f64;

        // Spread demand evenly across the 168 hours of the week.
        let demand_per_hour = demand_kg / HOURS_PER_WEEK as f64;

        for hour in 0..HOURS_PER_WEEK {
            // Assign hourly price from the three-band model.
            let hourly_price = if hour < 56 {
                price_cheap
            } else if hour < 112 {
                price_mid
            } else {
                price_peak
            };

            // 1. Serve H₂ demand first.
            let served = demand_per_hour.min(inventory);
            inventory -= served;
            h2_to_market += served;
            revenue += served * h2_price;

            // 2. Electrolyze in hours below break-even if storage has headroom.
            if hourly_price < break_even {
                let headroom = (capacity_kg - inventory).max(0.0);
                let prod_kg = self
                    .electrolyzer_dispatch(surplus_mw, h2_price, hourly_price)
                    .min(headroom);
                if prod_kg > 0.0 {
                    let power_mw = prod_kg * self.electrolyzers.efficiency_kwh_per_kg / 1_000.0;
                    inventory += prod_kg;
                    h2_produced += prod_kg;
                    elec_consumed += power_mw; // MWh (1-hour window)
                    cost += power_mw * hourly_price / 1_000.0; // $
                }
            }

            // 3. Generate electricity in expensive hours when above minimum inventory.
            if hourly_price > fc_threshold {
                let min_inventory = capacity_kg * 0.10;
                if inventory > min_inventory {
                    let gen_mwh = self.fuel_cell_dispatch(inventory, hourly_price, h2_price);
                    let h2_used =
                        gen_mwh * 1_000.0 / self.fuel_cells.efficiency_kwh_per_kg.max(f64::EPSILON);
                    inventory = (inventory - h2_used).max(0.0);
                    h2_consumed_fc += h2_used;
                    elec_generated += gen_mwh;
                    revenue += gen_mwh * hourly_price / 1_000.0; // $
                }
            }
        }

        WeeklyDispatch {
            week: week_idx,
            h2_produced_kg: h2_produced,
            h2_consumed_kg: h2_consumed_fc,
            h2_to_market_kg: h2_to_market,
            electricity_generated_mwh: elec_generated,
            electricity_consumed_mwh: elec_consumed,
            end_inventory_kg: inventory,
            revenue_usd: revenue,
            cost_usd: cost,
        }
    }

    // ─── Full-year simulation ─────────────────────────────────────────────────

    /// Run a week-by-week simulation over the full planning horizon.
    ///
    /// # Arguments
    /// * `initial_inventory_kg` — hydrogen inventory at the start of week 0 \[kg\]
    ///
    /// # Returns
    /// [`YearlySimResult`] with per-week records and aggregate KPIs.
    pub fn full_year_simulation(&self, initial_inventory_kg: f64) -> YearlySimResult {
        let capacity_kg = self.storage_capacity_kg();
        let mut inventory = initial_inventory_kg.clamp(0.0, capacity_kg);
        let mut weekly_results = Vec::with_capacity(self.config.planning_weeks);

        let mut total_h2_produced_kg = 0.0_f64;
        let mut total_revenue = 0.0_f64;
        let mut total_cost = 0.0_f64;
        let mut total_elec_in = 0.0_f64;
        let mut total_elec_out = 0.0_f64;
        let mut sum_utilisation = 0.0_f64;
        let mut unserved_kg = 0.0_f64;

        for w in 0..self.config.planning_weeks {
            let wd = self.optimize_weekly_dispatch(w, inventory);

            // Track unserved demand.
            let demand_kg = self
                .config
                .h2_demand_t_per_week
                .get(w)
                .copied()
                .unwrap_or(0.0)
                * KG_PER_TONNE;
            let served = wd.h2_to_market_kg;
            unserved_kg += (demand_kg - served).max(0.0);

            total_h2_produced_kg += wd.h2_produced_kg;
            total_revenue += wd.revenue_usd;
            total_cost += wd.cost_usd;
            total_elec_in += wd.electricity_consumed_mwh;
            total_elec_out += wd.electricity_generated_mwh;

            // Clamp end inventory to physical bounds.
            inventory = wd.end_inventory_kg.clamp(0.0, capacity_kg);
            sum_utilisation += inventory / capacity_kg.max(f64::EPSILON);

            weekly_results.push(WeeklyDispatch {
                end_inventory_kg: inventory,
                ..wd
            });
        }

        let n_weeks = self.config.planning_weeks as f64;
        let round_trip_efficiency_pct = if total_elec_in > 0.0 {
            (total_elec_out / total_elec_in) * 100.0
        } else {
            0.0
        };
        let storage_utilization_pct = if n_weeks > 0.0 {
            (sum_utilisation / n_weeks) * 100.0
        } else {
            0.0
        };

        YearlySimResult {
            weekly_results,
            total_h2_produced_t: total_h2_produced_kg / KG_PER_TONNE,
            total_revenue_usd: total_revenue,
            total_cost_usd: total_cost,
            net_profit_usd: total_revenue - total_cost,
            round_trip_efficiency_pct,
            storage_utilization_pct,
            unserved_h2_demand_t: unserved_kg / KG_PER_TONNE,
        }
    }

    // ─── Storage sizing ───────────────────────────────────────────────────────

    /// Estimate the minimum storage capacity required to achieve a reliability target.
    ///
    /// Computes the maximum cumulative H₂ shortfall that would occur if no storage
    /// were present (i.e., Σ max(demand − surplus_as_h2, 0) over the worst season),
    /// then adds a safety buffer proportional to `(1 − target_reliability)`.
    ///
    /// # Arguments
    /// * `weekly_demand`    — weekly H₂ demand series \[t/week\]
    /// * `seasonal_surplus` — weekly renewable surplus \[MW\]
    /// * `target_reliability` — fraction of demand to be served (e.g. 0.95)
    ///
    /// # Returns
    /// Required storage capacity \[kg\].
    pub fn size_storage_for_reliability(
        &self,
        weekly_demand: &[f64],
        seasonal_surplus: &[f64],
        target_reliability: f64,
    ) -> f64 {
        let eff = self.electrolyzers.efficiency_kwh_per_kg;
        let weeks = weekly_demand.len().min(seasonal_surplus.len());

        // Convert surplus [MW] to H₂ production potential [kg/week].
        // Each MW × 168 h = 168 MWh = 168 000 kWh → 168 000 / eff kg
        let surplus_as_h2_kg: Vec<f64> = (0..weeks)
            .map(|i| {
                let surplus_mwh = seasonal_surplus[i] * HOURS_PER_WEEK as f64;
                surplus_mwh * 1_000.0 / eff
            })
            .collect();

        // Weekly net deficit [kg] (demand − surplus potential when negative).
        let deficits: Vec<f64> = (0..weeks)
            .map(|i| {
                let demand_kg = weekly_demand[i] * KG_PER_TONNE;
                (demand_kg - surplus_as_h2_kg[i]).max(0.0)
            })
            .collect();

        // Find the worst consecutive season (max rolling 13-week deficit).
        let window = 13_usize; // ~1 quarter
        let mut max_deficit = 0.0_f64;
        for start in 0..weeks.saturating_sub(window).saturating_add(1) {
            let window_sum: f64 = deficits[start..(start + window).min(weeks)].iter().sum();
            max_deficit = max_deficit.max(window_sum);
        }

        // Safety buffer: for a reliability of r, add (1−r)/(r) × base as margin.
        let reliability = target_reliability.clamp(0.5, 0.9999);
        let safety_factor = 1.0 + (1.0 - reliability) / reliability;

        (max_deficit * safety_factor).max(0.0)
    }

    // ─── Economic assessment ──────────────────────────────────────────────────

    /// Levelised cost of hydrogen and system NPV.
    ///
    /// # Arguments
    /// * `capex_per_kg`       — overnight capital cost per kg of storage capacity \[$/kg\]
    /// * `opex_fraction`      — annual OPEX as a fraction of CAPEX \[–\]
    /// * `lifetime_years`     — economic lifetime \[years\]
    ///
    /// # Returns
    /// [`H2EconomicsResult`].
    pub fn economic_assessment(
        &self,
        capex_per_kg: f64,
        opex_fraction: f64,
        lifetime_years: f64,
    ) -> H2EconomicsResult {
        let capacity_kg = self.storage_capacity_kg();
        let r = self.config.discount_rate;

        // Capital recovery factor (annuity).
        let crf = if r > 0.0 {
            r * (1.0 + r).powf(lifetime_years) / ((1.0 + r).powf(lifetime_years) - 1.0)
        } else {
            1.0 / lifetime_years.max(1.0)
        };

        // Total CAPEX of the storage farm.
        let storage_capex = capacity_kg * capex_per_kg;
        // Add a representative electrolyzer and fuel-cell CAPEX.
        // Electrolyzer: $1 000/kW; Fuel cell: $2 000/kW.
        let electrolyzer_capex = self.electrolyzers.capacity_mw * 1_000.0 * 1_000.0; // $
        let fuel_cell_capex = self.fuel_cells.capacity_mw * 2_000.0 * 1_000.0; // $
        let total_capex = storage_capex + electrolyzer_capex + fuel_cell_capex;

        let annual_capex_charge = total_capex * crf;
        let annual_opex = total_capex * opex_fraction;
        let annual_fixed_cost = annual_capex_charge + annual_opex;

        // Run a reference year simulation to obtain annual production and revenue.
        let initial_inventory = capacity_kg * 0.50;
        let sim = self.full_year_simulation(initial_inventory);

        let annual_h2_kg = sim.total_h2_produced_t * KG_PER_TONNE;
        let annual_revenue = sim.total_revenue_usd;

        // LCOH [$/kg]: (annual fixed costs + variable costs) / H₂ produced.
        let annual_variable_cost = sim.total_cost_usd;
        let total_annual_cost = annual_fixed_cost + annual_variable_cost;
        let lcoh_per_kg = if annual_h2_kg > 0.0 {
            total_annual_cost / annual_h2_kg
        } else {
            f64::INFINITY
        };

        // NPV: present value of (annual_revenue − annual_cost) for lifetime_years.
        let annual_net = annual_revenue - annual_fixed_cost - annual_variable_cost;
        let pv_factor = if r > 0.0 {
            (1.0 - (1.0 + r).powf(-lifetime_years)) / r
        } else {
            lifetime_years
        };
        let npv_usd = annual_net * pv_factor - total_capex;

        let payback_years = if annual_net > 0.0 {
            total_capex / annual_net
        } else {
            f64::INFINITY
        };

        // Carbon intensity (computed with a representative grid; zero surplus usage here).
        let carbon_intensity = self.carbon_intensity(0.0); // placeholder — see method below

        H2EconomicsResult {
            lcoh_per_kg,
            npv_usd,
            payback_years,
            annual_revenue_usd: annual_revenue,
            is_green_hydrogen: carbon_intensity < GREEN_H2_THRESHOLD_G_CO2_PER_KG,
            carbon_intensity_g_co2_per_kg: carbon_intensity,
        }
    }

    // ─── Carbon intensity ─────────────────────────────────────────────────────

    /// Compute the carbon intensity of electrolytic hydrogen given a grid emission factor.
    ///
    /// ```text
    /// CI [gCO₂/kgH₂] = grid_ci [gCO₂/kWh] × eff [kWh/kgH₂]
    /// ```
    ///
    /// Returns the carbon intensity value and sets [`H2EconomicsResult::is_green_hydrogen`]
    /// if called via `economic_assessment`.
    ///
    /// # Arguments
    /// * `grid_co2_intensity_g_per_kwh` — grid-average emission factor \[gCO₂/kWh\]
    ///
    /// # Returns
    /// Carbon intensity \[gCO₂/kgH₂\].
    pub fn carbon_intensity(&self, grid_co2_intensity_g_per_kwh: f64) -> f64 {
        self.electrolyzers.efficiency_kwh_per_kg * grid_co2_intensity_g_per_kwh
    }

    /// Returns `true` if the hydrogen produced qualifies as *green* under the EU Taxonomy
    /// at the given grid emission factor.
    pub fn is_green_hydrogen(&self, grid_co2_intensity_g_per_kwh: f64) -> bool {
        self.carbon_intensity(grid_co2_intensity_g_per_kwh) < GREEN_H2_THRESHOLD_G_CO2_PER_KG
    }
}

// ─── Unit tests ───────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a default optimiser with a salt-cavern storage for reuse across tests.
    fn default_optimizer(capacity_mw: f64) -> SeasonalH2Optimizer {
        let weeks = 52_usize;
        // Construct a mild seasonal price profile: summer cheap, winter expensive.
        let prices: Vec<f64> = (0..weeks)
            .map(|w| 40.0 + 30.0 * ((2.0 * std::f64::consts::PI * w as f64 / 52.0).sin()))
            .collect();
        let surpluses: Vec<f64> = (0..weeks)
            .map(|w| 300.0 + 200.0 * ((2.0 * std::f64::consts::PI * w as f64 / 52.0).sin()))
            .collect();
        let demands: Vec<f64> = vec![80.0; weeks]; // 80 t/week

        SeasonalH2Optimizer {
            storage: SeasonalStorageType::SaltCavern {
                working_volume_m3: 500_000.0,
                min_pressure_bar: 40.0,
                max_pressure_bar: 200.0,
                cushion_gas_pct: 0.25,
            },
            electrolyzers: ElectrolyzerFleet {
                capacity_mw,
                ..ElectrolyzerFleet::default()
            },
            fuel_cells: FuelCellFleet {
                capacity_mw: capacity_mw * 0.5,
                ..FuelCellFleet::default()
            },
            config: SeasonalStorageConfig {
                planning_weeks: weeks,
                electricity_price_seasonal: prices,
                renewable_surplus_mw: surpluses,
                h2_demand_t_per_week: demands,
                ..SeasonalStorageConfig::default()
            },
        }
    }

    // ── Test 1: Salt cavern — larger volume → more storage ────────────────────
    #[test]
    fn test_salt_cavern_capacity_scales_with_volume() {
        let small = SeasonalH2Optimizer {
            storage: SeasonalStorageType::SaltCavern {
                working_volume_m3: 100_000.0,
                min_pressure_bar: 40.0,
                max_pressure_bar: 200.0,
                cushion_gas_pct: 0.0,
            },
            electrolyzers: ElectrolyzerFleet::default(),
            fuel_cells: FuelCellFleet::default(),
            config: SeasonalStorageConfig::default(),
        };
        let large = SeasonalH2Optimizer {
            storage: SeasonalStorageType::SaltCavern {
                working_volume_m3: 1_000_000.0,
                min_pressure_bar: 40.0,
                max_pressure_bar: 200.0,
                cushion_gas_pct: 0.0,
            },
            ..small.clone()
        };
        // Capacity should scale linearly with volume.
        let cap_small = small.storage_capacity_kg();
        let cap_large = large.storage_capacity_kg();
        assert!(
            cap_large > cap_small,
            "larger volume must give more storage"
        );
        let ratio = cap_large / cap_small;
        assert!(
            (ratio - 10.0).abs() < 0.01,
            "capacity should scale 10× with 10× volume, got {ratio}"
        );
    }

    // ── Test 2: Electrolyzer — cheap electricity → H₂ production ─────────────
    #[test]
    fn test_electrolyzer_produces_when_electricity_cheap() {
        let opt = default_optimizer(100.0);
        // At $20/MWh the electrolyzer should be well below break-even.
        let prod = opt.electrolyzer_dispatch(200.0, 3.0, 20.0);
        assert!(prod > 0.0, "should produce H₂ when electricity is cheap");
    }

    // ── Test 3: Electrolyzer — expensive electricity → no production ──────────
    #[test]
    fn test_electrolyzer_idle_when_electricity_expensive() {
        let opt = default_optimizer(100.0);
        // Break-even for eff=55 kWh/kg, h2=$3/kg → 3×1000/55 ≈ $54.5/MWh.
        // At $200/MWh, no production expected.
        let prod = opt.electrolyzer_dispatch(200.0, 3.0, 200.0);
        assert_eq!(
            prod, 0.0,
            "should not produce H₂ when electricity is expensive"
        );
    }

    // ── Test 4: Fuel cell — high electricity price → generation ──────────────
    #[test]
    fn test_fuel_cell_generates_at_high_price() {
        let opt = default_optimizer(100.0);
        // threshold = 3.0 × 1000 / 20 = 150 $/MWh; at 300 $/MWh should generate.
        let gen = opt.fuel_cell_dispatch(10_000.0, 300.0, 3.0);
        assert!(
            gen > 0.0,
            "fuel cell should generate at high electricity price"
        );
    }

    // ── Test 5: Sizing — high seasonal surplus → smaller storage needed ───────
    #[test]
    fn test_sizing_high_surplus_reduces_storage() {
        let opt = default_optimizer(100.0);
        let demand = vec![100.0_f64; 52]; // 100 t/week

        let low_surplus = vec![10.0_f64; 52]; // only 10 MW surplus
        let high_surplus = vec![500.0_f64; 52]; // 500 MW surplus

        let storage_low = opt.size_storage_for_reliability(&demand, &low_surplus, 0.95);
        let storage_high = opt.size_storage_for_reliability(&demand, &high_surplus, 0.95);

        assert!(
            storage_low > storage_high,
            "low surplus should require more storage: {storage_low} vs {storage_high}"
        );
    }

    // ── Test 6: Year simulation — inventory stays non-negative ─────────────────
    #[test]
    fn test_year_simulation_non_negative_inventory() {
        let opt = default_optimizer(200.0);
        let sim = opt.full_year_simulation(500_000.0);
        for wd in &sim.weekly_results {
            assert!(
                wd.end_inventory_kg >= 0.0,
                "inventory went negative at week {}: {}",
                wd.week,
                wd.end_inventory_kg
            );
        }
    }

    // ── Test 7: LCOH within realistic range ────────────────────────────────────
    #[test]
    fn test_lcoh_within_realistic_range() {
        let opt = default_optimizer(200.0);
        let econ = opt.economic_assessment(10.0, 0.03, 20.0);
        assert!(
            econ.lcoh_per_kg.is_finite(),
            "LCOH must be finite, got {}",
            econ.lcoh_per_kg
        );
        // Realistic LCOH range: $2–$200/kg (wider bounds to allow for test configs).
        assert!(
            econ.lcoh_per_kg >= 0.5,
            "LCOH suspiciously low: {}",
            econ.lcoh_per_kg
        );
        assert!(
            econ.lcoh_per_kg <= 500.0,
            "LCOH suspiciously high: {}",
            econ.lcoh_per_kg
        );
    }

    // ── Test 8: Green hydrogen — low-carbon grid → is_green = true ────────────
    #[test]
    fn test_green_hydrogen_low_carbon_grid() {
        let opt = default_optimizer(100.0);
        // 10 gCO₂/kWh × 55 kWh/kg = 550 gCO₂/kgH₂ < 1000 threshold → green.
        assert!(
            opt.is_green_hydrogen(10.0),
            "should be green at 10 gCO₂/kWh grid intensity"
        );
        // 30 gCO₂/kWh × 55 = 1650 > 1000 → not green.
        assert!(
            !opt.is_green_hydrogen(30.0),
            "should not be green at 30 gCO₂/kWh grid intensity"
        );
    }

    // ── Test 9: Pipeline buffer capacity ──────────────────────────────────────
    #[test]
    fn test_pipeline_buffer_capacity() {
        let opt = SeasonalH2Optimizer {
            storage: SeasonalStorageType::PipelineBuffer {
                pipe_length_km: 100.0,
                diameter_m: 0.5,
                max_pressure_bar: 80.0,
            },
            electrolyzers: ElectrolyzerFleet::default(),
            fuel_cells: FuelCellFleet::default(),
            config: SeasonalStorageConfig::default(),
        };
        let cap = opt.storage_capacity_kg();
        assert!(
            cap > 0.0,
            "pipeline buffer should have positive capacity: {cap}"
        );
    }

    // ── Test 10: Underground tank capacity direct from nameplate ──────────────
    #[test]
    fn test_underground_tank_capacity() {
        let opt = SeasonalH2Optimizer {
            storage: SeasonalStorageType::UndergroundTank {
                capacity_t_h2: 1_000.0,
                pressure_bar: 200.0,
            },
            electrolyzers: ElectrolyzerFleet::default(),
            fuel_cells: FuelCellFleet::default(),
            config: SeasonalStorageConfig::default(),
        };
        let cap = opt.storage_capacity_kg();
        assert!(
            (cap - 1_000_000.0).abs() < 1.0,
            "underground tank capacity should equal 1 000 t × 1000 kg/t = 1 000 000 kg, got {cap}"
        );
    }
}
