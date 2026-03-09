//! EV Charging Infrastructure Planning.
//!
//! Provides optimal charger placement, capacity sizing, demand forecasting,
//! grid impact assessment, NPV calculation, equity analysis, and demand
//! response potential estimation for EV charging infrastructure.
//!
//! # References
//!
//! - IEA Global EV Outlook (2023)
//! - EPRI "EV Energy Impact Study" (2021)
//! - Erlang B traffic engineering model

use serde::{Deserialize, Serialize};

// ─────────────────────────────────────────────────────────────────────────────
// Charger type
// ─────────────────────────────────────────────────────────────────────────────

/// EV charger category with rated power \[kW\].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum PlanChargerType {
    /// Level 1 AC charger, typically 1.4 \[kW\], residential use.
    Level1 { power_kw: f64 },
    /// Level 2 AC charger, 3.3–19.2 \[kW\], commercial/residential.
    Level2 { power_kw: f64 },
    /// DC fast charger, 50–350 \[kW\], highway and commercial hubs.
    DcFastCharge { power_kw: f64 },
    /// Mega-charger, 1000+ \[kW\], heavy vehicles and fleet depots.
    MegaCharge { power_kw: f64 },
}

impl PlanChargerType {
    /// Rated output power \[kW\].
    pub fn power_kw(&self) -> f64 {
        match self {
            PlanChargerType::Level1 { power_kw } => *power_kw,
            PlanChargerType::Level2 { power_kw } => *power_kw,
            PlanChargerType::DcFastCharge { power_kw } => *power_kw,
            PlanChargerType::MegaCharge { power_kw } => *power_kw,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Location type
// ─────────────────────────────────────────────────────────────────────────────

/// Classification of a charging site by land-use category.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum LocationType {
    /// Single-family residential driveway or garage.
    Residential,
    /// Retail, office, or mixed-use commercial parking.
    Commercial,
    /// Highway rest-stop or travel plaza.
    Highway,
    /// Commercial vehicle depot or logistics hub.
    Fleet,
    /// Multi-unit residential building (apartment/condo).
    MultiFamily,
}

// ─────────────────────────────────────────────────────────────────────────────
// Charging location
// ─────────────────────────────────────────────────────────────────────────────

/// A candidate site for EV charging infrastructure.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChargingLocation {
    /// Unique location identifier.
    pub location_id: String,
    /// Network bus index where this site connects.
    pub bus_id: usize,
    /// Land-use category.
    pub location_type: LocationType,
    /// Available physical area \[m²\].
    pub available_area_sqm: f64,
    /// Estimated vehicles visiting per day.
    pub daily_vehicles: f64,
    /// Fraction of daily vehicles arriving during the peak hour (0–1).
    pub peak_hour_fraction: f64,
    /// Maximum grid connection capacity \[kW\].
    pub grid_capacity_kw: f64,
    /// Latitude (WGS-84).
    pub lat: f64,
    /// Longitude (WGS-84).
    pub lon: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Demand forecast inputs
// ─────────────────────────────────────────────────────────────────────────────

/// Charging behaviour preference for demand modelling.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ChargingPreference {
    /// Charge immediately upon arrival (uncontrolled).
    Uncoordinated,
    /// Time-of-use optimised smart charging.
    Smart {
        /// Sensitivity to electricity price signals (0–1).
        price_sensitivity: f64,
    },
    /// Vehicle-to-Grid with a fraction participating in export.
    V2g {
        /// Fraction of vehicles offering V2G services (0–1).
        v2g_fraction: f64,
    },
}

/// Annual EV demand forecast input parameters.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EvDemandForecast {
    /// Base forecast year.
    pub year: usize,
    /// EV penetration as a percentage of the total vehicle fleet.
    pub ev_penetration_pct: f64,
    /// Average daily travel distance per vehicle \[km\].
    pub avg_daily_km: f64,
    /// Average energy consumption \[kWh/km\] (typical 0.2).
    pub avg_consumption_kwh_per_km: f64,
    /// Fraction of EV charging occurring at home (0–1).
    pub home_charging_pct: f64,
    /// Fraction of EV charging occurring at work (0–1).
    pub work_charging_pct: f64,
    /// Fraction of EV charging occurring at public stations (0–1).
    pub public_charging_pct: f64,
    /// Driver charging behaviour preference.
    pub charging_preference: ChargingPreference,
}

// ─────────────────────────────────────────────────────────────────────────────
// Configuration
// ─────────────────────────────────────────────────────────────────────────────

/// Charger capital and operating cost parameters.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChargerCost {
    /// Installed cost of a Level-2 charger \[USD\].
    pub level2_installed_usd: f64,
    /// Installed cost of a DC fast charger \[USD\].
    pub dcfc_installed_usd: f64,
    /// Annual O&M as a fraction of CAPEX (e.g. 0.05 = 5 %).
    pub annual_opex_pct: f64,
    /// Electricity purchase rate \[USD/kWh\].
    pub electricity_rate_per_kwh: f64,
}

impl Default for ChargerCost {
    fn default() -> Self {
        Self {
            level2_installed_usd: 5_000.0,
            dcfc_installed_usd: 50_000.0,
            annual_opex_pct: 0.05,
            electricity_rate_per_kwh: 0.12,
        }
    }
}

/// Top-level configuration for the infrastructure planning optimiser.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InfrastructurePlanConfig {
    /// Investment planning horizon \[years\].
    pub planning_horizon_years: usize,
    /// Discount rate for NPV calculations (e.g. 0.07 = 7 %).
    pub discount_rate: f64,
    /// Charger capital and operating costs.
    pub charger_cost: ChargerCost,
    /// Minimum fraction of peak demand that must be served (0–1).
    pub reliability_target: f64,
    /// Grid upgrade cost \[USD/kW\] of added capacity.
    pub grid_upgrade_cost_per_kw: f64,
}

impl Default for InfrastructurePlanConfig {
    fn default() -> Self {
        Self {
            planning_horizon_years: 10,
            discount_rate: 0.07,
            charger_cost: ChargerCost::default(),
            reliability_target: 0.95,
            grid_upgrade_cost_per_kw: 500.0,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Main planner
// ─────────────────────────────────────────────────────────────────────────────

/// EV charging infrastructure planner.
pub struct EvInfraPlanner {
    /// Candidate charging locations.
    pub locations: Vec<ChargingLocation>,
    /// Planning configuration and cost parameters.
    pub config: InfrastructurePlanConfig,
}

impl EvInfraPlanner {
    /// Create a new planner with the given locations and configuration.
    pub fn new(locations: Vec<ChargingLocation>, config: InfrastructurePlanConfig) -> Self {
        Self { locations, config }
    }

    // ── 1. Demand forecast ──────────────────────────────────────────────────

    /// Forecast year-by-year EV energy demand.
    ///
    /// # Parameters
    /// - `forecast` – base-year demand parameters
    /// - `total_vehicles` – total vehicle fleet size (not just EVs)
    /// - `horizon_years` – number of years to project
    ///
    /// # Returns
    /// Vector of [`DemandForecastYear`] with `horizon_years` entries.
    pub fn forecast_demand(
        &self,
        forecast: &EvDemandForecast,
        total_vehicles: f64,
        horizon_years: usize,
    ) -> Vec<DemandForecastYear> {
        let mut results = Vec::with_capacity(horizon_years);
        let avg_daily_kwh = forecast.avg_daily_km * forecast.avg_consumption_kwh_per_km;
        // Peak fraction: fraction of daily kWh consumed in the peak 1-hour window.
        // Assume charging spread over ~7 h; peak hour ≈ 1/7 ≈ 0.143 but bounded by
        // location's peak_hour_fraction aggregated across all locations.
        let peak_fraction = 0.15_f64;

        let mut pen = forecast.ev_penetration_pct / 100.0;

        for t in 0..horizon_years {
            let n_ev = total_vehicles * pen;
            let total_ev_kwh_per_day = n_ev * avg_daily_kwh;
            let home_kwh = total_ev_kwh_per_day * forecast.home_charging_pct;
            let work_kwh = total_ev_kwh_per_day * forecast.work_charging_pct;
            let public_kwh = total_ev_kwh_per_day * forecast.public_charging_pct;
            let peak_demand_kw = total_ev_kwh_per_day * peak_fraction;

            results.push(DemandForecastYear {
                year: forecast.year + t,
                total_ev_kwh_per_day,
                home_kwh,
                work_kwh,
                public_kwh,
                peak_demand_kw,
            });

            // 5 % annual growth in EV penetration
            pen *= 1.05;
        }
        results
    }

    // ── 2. Charger sizing ───────────────────────────────────────────────────

    /// Size chargers at a location to meet peak demand with a reliability target.
    ///
    /// Uses the Erlang B formula (iterative) to determine the minimum number of
    /// chargers such that the call-blocking probability ≤ `1 − reliability_target`.
    ///
    /// # Parameters
    /// - `location` – the charging site
    /// - `demand_kw_peak` – peak charging demand \[kW\]
    /// - `reliability_target` – fraction of demand that must be served (0–1)
    pub fn size_chargers(
        &self,
        location: &ChargingLocation,
        demand_kw_peak: f64,
        reliability_target: f64,
    ) -> ChargerSizing {
        let charger_type = charger_type_for_location(&location.location_type);
        let charger_kw = charger_type.power_kw().max(1.0);

        // Traffic intensity (Erlangs): offered load / service capacity
        let rho = demand_kw_peak / charger_kw;
        let max_blocking = (1.0 - reliability_target).max(0.0);

        // Minimum servers needed: start at ceil(rho)
        let mut n = (rho.ceil() as usize).max(1);
        let mut bp = erlang_b(rho, n);

        // Increase until blocking probability is acceptable
        while bp > max_blocking && n < 200 {
            n += 1;
            bp = erlang_b(rho, n);
        }

        // Respect grid capacity
        let max_chargers_by_grid = if charger_kw > 0.0 {
            (location.grid_capacity_kw / charger_kw).floor() as usize
        } else {
            n
        };
        n = n.min(max_chargers_by_grid).max(1);
        // Recompute blocking with capped n
        bp = erlang_b(rho, n);

        let total_capacity_kw = n as f64 * charger_kw;

        ChargerSizing {
            location_id: location.location_id.clone(),
            charger_type,
            num_chargers: n,
            total_capacity_kw,
            blocking_probability: bp,
        }
    }

    // ── 3. Optimal charger placement ────────────────────────────────────────

    /// Greedy benefit-cost charger placement across all candidate locations.
    ///
    /// For each location the peak demand is estimated from `daily_vehicles` and
    /// `peak_hour_fraction`.  Chargers are sized, a benefit/cost ratio computed,
    /// and locations are selected in descending B/C order until `budget_usd` is
    /// exhausted.
    ///
    /// # Parameters
    /// - `demand_forecast` – year-by-year demand (year 0 used for sizing)
    /// - `budget_usd` – total capital budget \[USD\]
    pub fn optimal_charger_placement(
        &self,
        demand_forecast: &[DemandForecastYear],
        budget_usd: f64,
    ) -> PlacementPlan {
        // Average session energy: 20 [kWh]; average revenue per session: $5.00
        const AVG_SESSION_KWH: f64 = 20.0;
        const AVG_REVENUE_PER_SESSION: f64 = 5.0;
        let horizon = self.config.planning_horizon_years as f64;

        // Annualised charger cost helper (simple straight-line)
        let annualized_cost = |capex: f64| -> f64 {
            capex / horizon + capex * self.config.charger_cost.annual_opex_pct
        };

        // Compute B/C for each location
        let mut candidates: Vec<(usize, f64, ChargerSizing, f64)> = self
            .locations
            .iter()
            .enumerate()
            .filter_map(|(idx, loc)| {
                // Peak demand estimate [kW]
                let sessions_peak = loc.daily_vehicles * loc.peak_hour_fraction;
                let demand_kw_peak = sessions_peak * AVG_SESSION_KWH; // sessions × kWh/session → kWh in 1h = kW
                let sizing =
                    self.size_chargers(loc, demand_kw_peak, self.config.reliability_target);

                let capex = charger_capex(
                    &sizing.charger_type,
                    sizing.num_chargers,
                    &self.config.charger_cost,
                );
                if capex <= 0.0 {
                    return None;
                }

                let annual_benefit = loc.daily_vehicles * AVG_REVENUE_PER_SESSION * 365.0;
                let annual_cost = annualized_cost(capex);
                let bc_ratio = if annual_cost > 0.0 {
                    annual_benefit / annual_cost
                } else {
                    f64::INFINITY
                };

                Some((idx, bc_ratio, sizing, capex))
            })
            .collect();

        // Sort by B/C descending
        candidates.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        let mut sites: Vec<SiteAllocation> = Vec::new();
        let mut remaining_budget = budget_usd;
        let mut total_cost = 0.0_f64;
        let mut total_capacity_kw = 0.0_f64;

        for (loc_idx, _bc, sizing, capex) in candidates {
            if remaining_budget < capex {
                continue;
            }
            let loc = &self.locations[loc_idx];
            // Ensure grid capacity is not exceeded
            if sizing.total_capacity_kw > loc.grid_capacity_kw + 1e-6 {
                // Try with fewer chargers
                let capped_n = (loc.grid_capacity_kw / sizing.charger_type.power_kw().max(1.0))
                    .floor() as usize;
                if capped_n == 0 {
                    continue;
                }
                let capped_cap = capped_n as f64 * sizing.charger_type.power_kw();
                let capped_cost =
                    charger_capex(&sizing.charger_type, capped_n, &self.config.charger_cost);
                if remaining_budget < capped_cost {
                    continue;
                }
                sites.push(SiteAllocation {
                    location_id: loc.location_id.clone(),
                    num_chargers: capped_n,
                    charger_type: sizing.charger_type,
                    cost_usd: capped_cost,
                });
                remaining_budget -= capped_cost;
                total_cost += capped_cost;
                total_capacity_kw += capped_cap;
            } else {
                sites.push(SiteAllocation {
                    location_id: loc.location_id.clone(),
                    num_chargers: sizing.num_chargers,
                    charger_type: sizing.charger_type,
                    cost_usd: capex,
                });
                remaining_budget -= capex;
                total_cost += capex;
                total_capacity_kw += sizing.total_capacity_kw;
            }
        }

        // Expected annual revenue: sum over selected sites
        let mut expected_annual_revenue = 0.0_f64;
        for site in &sites {
            // Find location for daily_vehicles
            let daily_v = self
                .locations
                .iter()
                .find(|l| l.location_id == site.location_id)
                .map(|l| l.daily_vehicles)
                .unwrap_or(0.0);
            expected_annual_revenue += daily_v * AVG_REVENUE_PER_SESSION * 365.0;
        }

        // Scale by demand forecast year 0 utilisation if available
        let utilisation_scale = demand_forecast
            .first()
            .map(|y| {
                let total_pub = y.public_kwh;
                let total_cap = total_capacity_kw * 24.0; // kWh/day capacity
                if total_cap > 0.0 {
                    (total_pub / total_cap).min(1.0)
                } else {
                    1.0
                }
            })
            .unwrap_or(1.0);

        expected_annual_revenue *= utilisation_scale.max(0.1);

        PlacementPlan {
            sites,
            total_cost_usd: total_cost,
            total_capacity_kw,
            expected_annual_revenue,
        }
    }

    // ── 4. Grid impact assessment ───────────────────────────────────────────

    /// Assess the grid impact of a placement plan.
    ///
    /// Uses a simplified linear voltage sensitivity: ΔV ≈ 0.1 (pu/MW) × ΔP (MW).
    ///
    /// # Parameters
    /// - `placement_plan` – output from `optimal_charger_placement`
    /// - `bus_voltages_pu` – pre-charging bus voltages in per-unit
    /// - `branch_flows_mw` – pre-charging branch active power flows \[MW\]
    /// - `branch_ratings_mw` – thermal ratings for each branch \[MW\]
    pub fn grid_impact_assessment(
        &self,
        placement_plan: &PlacementPlan,
        bus_voltages_pu: &[f64],
        branch_flows_mw: &[f64],
        branch_ratings_mw: &[f64],
    ) -> GridImpact {
        // Sensitivity: ΔV [pu] = Z_sensitivity × ΔP [MW]
        const Z_SENSITIVITY: f64 = 0.1;

        let mut voltage_violations: Vec<usize> = Vec::new();
        let mut thermal_violations: Vec<usize> = Vec::new();
        let mut required_upgrade_kw = 0.0_f64;
        let mut max_voltage_drop_pu = 0.0_f64;

        for site in &placement_plan.sites {
            // Find the bus_id for this site
            let bus_id = self
                .locations
                .iter()
                .find(|l| l.location_id == site.location_id)
                .map(|l| l.bus_id)
                .unwrap_or(0);

            let load_addition_mw = site.num_chargers as f64 * site.charger_type.power_kw() / 1000.0;

            let delta_v = Z_SENSITIVITY * load_addition_mw;
            if delta_v > max_voltage_drop_pu {
                max_voltage_drop_pu = delta_v;
            }

            // Check voltage
            if let Some(&v_pre) = bus_voltages_pu.get(bus_id) {
                let v_post = v_pre - delta_v;
                if v_post < 0.95 {
                    if !voltage_violations.contains(&bus_id) {
                        voltage_violations.push(bus_id);
                    }
                    required_upgrade_kw += load_addition_mw * 1000.0;
                }
            } else {
                // Bus not in voltage array; assume worst case if large load
                if load_addition_mw > 1.0 {
                    if !voltage_violations.contains(&bus_id) {
                        voltage_violations.push(bus_id);
                    }
                    required_upgrade_kw += load_addition_mw * 1000.0;
                }
            }

            // Check thermal: use branch index = bus_id as heuristic
            let branch_idx = bus_id.min(branch_flows_mw.len().saturating_sub(1));
            if let (Some(&flow), Some(&rating)) = (
                branch_flows_mw.get(branch_idx),
                branch_ratings_mw.get(branch_idx),
            ) {
                if flow + load_addition_mw > rating && !thermal_violations.contains(&branch_idx) {
                    thermal_violations.push(branch_idx);
                }
            }
        }

        let upgrade_cost_usd = required_upgrade_kw * self.config.grid_upgrade_cost_per_kw;

        GridImpact {
            voltage_violations,
            thermal_violations,
            required_upgrade_kw,
            upgrade_cost_usd,
            max_voltage_drop_pu,
        }
    }

    // ── 5. NPV calculation ──────────────────────────────────────────────────

    /// Calculate the net present value (NPV) of a placement plan.
    ///
    /// CAPEX is incurred at year 0; annual revenue and OPEX are discounted.
    ///
    /// # Returns
    /// NPV \[USD\].
    pub fn calculate_npv(
        &self,
        placement_plan: &PlacementPlan,
        demand_forecast: &[DemandForecastYear],
    ) -> f64 {
        let capex = placement_plan.total_cost_usd;
        let annual_opex = capex * self.config.charger_cost.annual_opex_pct;
        let r = self.config.discount_rate;
        let horizon = self.config.planning_horizon_years;

        let mut pv_sum = 0.0_f64;
        for t in 1..=horizon {
            // Revenue: scale by demand growth from forecast if available
            let revenue_scale = demand_forecast
                .get(t.saturating_sub(1))
                .map(|y| {
                    let base = demand_forecast
                        .first()
                        .map(|b| b.total_ev_kwh_per_day)
                        .unwrap_or(1.0);
                    if base > 0.0 {
                        y.total_ev_kwh_per_day / base
                    } else {
                        1.0
                    }
                })
                .unwrap_or(1.0);

            let revenue = placement_plan.expected_annual_revenue * revenue_scale;
            let net_cashflow = revenue - annual_opex;
            pv_sum += net_cashflow / (1.0 + r).powi(t as i32);
        }

        pv_sum - capex
    }

    // ── 6. Equity analysis ──────────────────────────────────────────────────

    /// Analyse the equity of charger distribution across income quintiles.
    ///
    /// # Parameters
    /// - `placement_plan` – selected sites
    /// - `income_levels` – income score per location (same order as `self.locations`)
    /// - `population` – population served per location
    ///
    /// # Returns
    /// [`EquityReport`] with Gini coefficient and coverage by quintile.
    pub fn equity_analysis(
        &self,
        placement_plan: &PlacementPlan,
        income_levels: &[f64],
        population: &[f64],
    ) -> EquityReport {
        const N_QUINTILES: usize = 5;

        // Build (income, chargers, population) per location
        let mut loc_data: Vec<(f64, f64, f64)> = self
            .locations
            .iter()
            .enumerate()
            .map(|(i, loc)| {
                let income = income_levels.get(i).copied().unwrap_or(0.0);
                let pop = population.get(i).copied().unwrap_or(1.0).max(1.0);
                let chargers = placement_plan
                    .sites
                    .iter()
                    .find(|s| s.location_id == loc.location_id)
                    .map(|s| s.num_chargers as f64)
                    .unwrap_or(0.0);
                (income, chargers, pop)
            })
            .collect();

        // Sort by income ascending for quintile assignment
        loc_data.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

        let n = loc_data.len();
        let quintile_size = if n < N_QUINTILES {
            1
        } else {
            n.div_ceil(N_QUINTILES)
        };

        let mut coverage_by_quintile = vec![0.0_f64; N_QUINTILES];
        for (i, &(_, chargers, pop)) in loc_data.iter().enumerate() {
            let q = (i / quintile_size).min(N_QUINTILES - 1);
            coverage_by_quintile[q] += chargers / pop * 1000.0;
        }
        // Average within quintile
        let counts_per_quintile: Vec<usize> = (0..N_QUINTILES)
            .map(|q| {
                loc_data
                    .iter()
                    .enumerate()
                    .filter(|(i, _)| (i / quintile_size).min(N_QUINTILES - 1) == q)
                    .count()
                    .max(1)
            })
            .collect();
        for q in 0..N_QUINTILES {
            coverage_by_quintile[q] /= counts_per_quintile[q] as f64;
        }

        let gini = gini_coefficient(&coverage_by_quintile);

        // Underserved: locations where coverage < half of mean coverage
        let mean_coverage = coverage_by_quintile.iter().copied().sum::<f64>() / N_QUINTILES as f64;
        let mut underserved_areas: Vec<String> = Vec::new();
        for (i, loc) in self.locations.iter().enumerate() {
            let q = (i / quintile_size.max(1)).min(N_QUINTILES - 1);
            let cov = coverage_by_quintile.get(q).copied().unwrap_or(0.0);
            if cov < mean_coverage / 2.0 {
                underserved_areas.push(loc.location_id.clone());
            }
        }
        underserved_areas.dedup();

        EquityReport {
            coverage_by_quintile,
            gini_coefficient: gini,
            underserved_areas,
        }
    }

    // ── 7. Demand response potential ────────────────────────────────────────

    /// Estimate demand response potential from smart EV chargers.
    ///
    /// Smart chargers can shift up to 4 hours of load from peak to off-peak.
    ///
    /// # Parameters
    /// - `placement_plan` – deployed charger fleet
    /// - `smart_charger_fraction` – fraction of chargers with smart capability (0–1)
    /// - `peak_price_per_mwh` – peak electricity price \[USD/MWh\]
    /// - `off_peak_price_per_mwh` – off-peak electricity price \[USD/MWh\]
    pub fn demand_response_potential(
        &self,
        placement_plan: &PlacementPlan,
        smart_charger_fraction: f64,
        peak_price_per_mwh: f64,
        off_peak_price_per_mwh: f64,
    ) -> DrPotential {
        const SHIFTABLE_HOURS: f64 = 4.0;
        const DAYS_PER_YEAR: f64 = 365.0;

        let dr_capacity_mw = smart_charger_fraction * placement_plan.total_capacity_kw / 1000.0;

        let shifted_mwh_per_day = dr_capacity_mw * SHIFTABLE_HOURS;
        let price_spread = (peak_price_per_mwh - off_peak_price_per_mwh).max(0.0);
        let annual_value_usd = shifted_mwh_per_day * DAYS_PER_YEAR * price_spread;

        DrPotential {
            dr_capacity_mw,
            annual_value_usd,
            peak_reduction_mw: dr_capacity_mw,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Output structs
// ─────────────────────────────────────────────────────────────────────────────

/// EV energy demand for one forecast year.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DemandForecastYear {
    /// Calendar year.
    pub year: usize,
    /// Total EV energy demand across all charging locations \[kWh/day\].
    pub total_ev_kwh_per_day: f64,
    /// Home charging portion \[kWh/day\].
    pub home_kwh: f64,
    /// Work / workplace charging portion \[kWh/day\].
    pub work_kwh: f64,
    /// Public charging portion \[kWh/day\].
    pub public_kwh: f64,
    /// Estimated peak-hour charging demand \[kW\].
    pub peak_demand_kw: f64,
}

/// Charger sizing recommendation for one location.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChargerSizing {
    /// Location identifier this sizing applies to.
    pub location_id: String,
    /// Recommended charger type.
    pub charger_type: PlanChargerType,
    /// Number of charger units to install.
    pub num_chargers: usize,
    /// Aggregate installed capacity \[kW\].
    pub total_capacity_kw: f64,
    /// Erlang B call-blocking probability at peak load (0–1).
    pub blocking_probability: f64,
}

/// Allocation of chargers to one site in the placement plan.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SiteAllocation {
    /// Location identifier.
    pub location_id: String,
    /// Number of charger units installed.
    pub num_chargers: usize,
    /// Charger type installed.
    pub charger_type: PlanChargerType,
    /// Total installed cost \[USD\].
    pub cost_usd: f64,
}

/// Optimised charger placement across all candidate sites.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlacementPlan {
    /// Per-site charger allocations.
    pub sites: Vec<SiteAllocation>,
    /// Total capital expenditure \[USD\].
    pub total_cost_usd: f64,
    /// Aggregate installed capacity across all sites \[kW\].
    pub total_capacity_kw: f64,
    /// Estimated annual revenue from charging sessions \[USD/year\].
    pub expected_annual_revenue: f64,
}

/// Grid impact assessment results.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GridImpact {
    /// Bus IDs where voltage drops below 0.95 pu after EV load addition.
    pub voltage_violations: Vec<usize>,
    /// Branch IDs where thermal ratings are exceeded.
    pub thermal_violations: Vec<usize>,
    /// Total grid upgrade capacity needed \[kW\].
    pub required_upgrade_kw: f64,
    /// Estimated grid upgrade cost \[USD\].
    pub upgrade_cost_usd: f64,
    /// Maximum voltage drop caused by EV load addition \[pu\].
    pub max_voltage_drop_pu: f64,
}

/// Equity report on charger distribution by income quintile.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EquityReport {
    /// Chargers per 1 000 residents in each income quintile (Q1=lowest income).
    pub coverage_by_quintile: Vec<f64>,
    /// Gini coefficient of coverage distribution (0 = perfect equity).
    pub gini_coefficient: f64,
    /// Location IDs in areas with below-average coverage.
    pub underserved_areas: Vec<String>,
}

/// Demand response potential from smart EV chargers.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DrPotential {
    /// Dispatchable DR capacity available for grid balancing \[MW\].
    pub dr_capacity_mw: f64,
    /// Annual value of DR from peak-to-off-peak load shifting \[USD/year\].
    pub annual_value_usd: f64,
    /// Peak demand reduction achievable \[MW\].
    pub peak_reduction_mw: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Private helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Choose charger type based on location category.
fn charger_type_for_location(loc_type: &LocationType) -> PlanChargerType {
    match loc_type {
        LocationType::Highway | LocationType::Fleet => {
            PlanChargerType::DcFastCharge { power_kw: 150.0 }
        }
        LocationType::Commercial => PlanChargerType::Level2 { power_kw: 22.0 },
        LocationType::Residential | LocationType::MultiFamily => {
            PlanChargerType::Level2 { power_kw: 7.4 }
        }
    }
}

/// Compute installed capital cost for a set of chargers \[USD\].
fn charger_capex(charger_type: &PlanChargerType, num_chargers: usize, cost: &ChargerCost) -> f64 {
    let unit_cost = match charger_type {
        PlanChargerType::Level1 { .. } => cost.level2_installed_usd * 0.3,
        PlanChargerType::Level2 { .. } => cost.level2_installed_usd,
        PlanChargerType::DcFastCharge { .. } => cost.dcfc_installed_usd,
        PlanChargerType::MegaCharge { .. } => cost.dcfc_installed_usd * 5.0,
    };
    unit_cost * num_chargers as f64
}

/// Erlang B blocking probability using the iterative recursion.
///
/// B(0, ρ) = 1 / (1 + ρ)  … initialised via the recurrence
/// B(n, ρ) = ρ · B(n-1, ρ) / (n + ρ · B(n-1, ρ))
///
/// # Parameters
/// - `rho` – traffic intensity (offered load in Erlangs)
/// - `n`   – number of servers (chargers)
fn erlang_b(rho: f64, n: usize) -> f64 {
    if rho <= 0.0 {
        return 0.0;
    }
    // Iterative form: avoids factorial overflow
    let mut b = 1.0_f64;
    for k in 1..=n {
        b = rho * b / (k as f64 + rho * b);
    }
    b
}

/// Gini coefficient for a distribution vector (0 = perfect equality).
fn gini_coefficient(values: &[f64]) -> f64 {
    let n = values.len();
    if n == 0 {
        return 0.0;
    }
    let mut sorted = values.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

    let sum: f64 = sorted.iter().sum();
    if sum <= 0.0 {
        return 0.0;
    }

    let mut numerator = 0.0_f64;
    for (i, &v) in sorted.iter().enumerate() {
        numerator += (2 * (i + 1)) as f64 * v;
    }
    // Gini = (2 * Σ (i+1)*x_i) / (n * Σ x_i) - (n+1)/n
    numerator / (n as f64 * sum) - (n as f64 + 1.0) / n as f64
}

/// Discrete charger categories with fixed cost models for infrastructure planning.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum ChargerType {
    /// Level 1 AC, 1 kW, typical residential overnight.
    Level1_1kw,
    /// Level 2 AC, 7 kW, residential and light commercial.
    Level2_7kw,
    /// Level 2 AC, 22 kW, commercial parking.
    Level2_22kw,
    /// DC fast charger, 50 kW.
    Dcfc50kw,
    /// DC fast charger, 150 kW, highway corridor.
    Dcfc150kw,
    /// DC ultra-fast charger, 350 kW, flagship hub.
    Dcfc350kw,
}

impl ChargerType {
    /// Rated output power \[kW\].
    pub fn rated_kw(self) -> f64 {
        match self {
            ChargerType::Level1_1kw => 1.0,
            ChargerType::Level2_7kw => 7.0,
            ChargerType::Level2_22kw => 22.0,
            ChargerType::Dcfc50kw => 50.0,
            ChargerType::Dcfc150kw => 150.0,
            ChargerType::Dcfc350kw => 350.0,
        }
    }

    /// Installed capital cost per charger unit \[USD\].
    pub fn capital_cost_usd(self) -> f64 {
        match self {
            ChargerType::Level1_1kw => 500.0,
            ChargerType::Level2_7kw => 3_000.0,
            ChargerType::Level2_22kw => 8_000.0,
            ChargerType::Dcfc50kw => 35_000.0,
            ChargerType::Dcfc150kw => 80_000.0,
            ChargerType::Dcfc350kw => 150_000.0,
        }
    }
}

/// A deployed EV charging station with physical, financial, and grid attributes.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChargingStation {
    /// Unique station identifier.
    pub id: usize,
    /// Geographic location (lat, lon) in WGS-84 decimal degrees.
    pub location: (f64, f64),
    /// Total installed charging capacity \[kW\].
    pub capacity_kw: f64,
    /// Number of charger ports.
    pub n_chargers: usize,
    /// Charger hardware type.
    pub charger_type: ChargerType,
    /// Total capital cost \[USD\].
    pub capital_cost_usd: f64,
    /// Annual O&M cost \[USD/year\].
    pub annual_opex_usd: f64,
    /// Average occupancy fraction (0–1).
    pub utilization_factor: f64,
    /// Power-network bus where this station connects.
    pub grid_connection_bus: usize,
}

/// A demand point representing EV charging need at a geographic location.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DemandNode {
    /// Unique identifier.
    pub id: usize,
    /// Geographic location (lat, lon) in WGS-84 decimal degrees.
    pub location: (f64, f64),
    /// Daily EV energy demand \[kWh/day\].
    pub daily_ev_demand_kwh: f64,
    /// Peak charging power required \[kW\].
    pub peak_power_kw: f64,
    /// Number of EVs generating this demand.
    pub ev_count: usize,
    /// Preferred charger type for this demand cluster.
    pub preferred_charger_type: ChargerType,
}

/// Power-network capacity constraint at a grid bus for EV load additions.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GridConstraint {
    /// Bus identifier.
    pub bus_id: usize,
    /// Maximum additional load that can be connected \[kW\].
    pub max_additional_load_kw: f64,
    /// Available spare capacity \[kW\].
    pub available_capacity_kw: f64,
}

/// A complete EV charging infrastructure deployment plan.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InfrastructurePlan {
    /// Deployed charging stations.
    pub stations: Vec<ChargingStation>,
    /// Total capital investment \[USD\].
    pub total_capital_cost_usd: f64,
    /// Total annual OPEX \[USD/year\].
    pub total_annual_cost_usd: f64,
    /// Fraction of demand nodes served within 10 km (0–1).
    pub coverage_pct: f64,
    /// Mean Haversine distance from demand nodes to nearest station \[km\].
    pub average_distance_km: f64,
    /// Grid loading fraction per constraint bus (0–1+).
    pub grid_loading_pct: Vec<f64>,
    /// Levelised cost of energy \[USD/kWh\].
    pub lcoe_usd_per_kwh: f64,
}

/// Evaluated performance metrics for a deployment plan.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanMetrics {
    /// Fraction of demand nodes within 10 km of a station (0–1).
    pub coverage_pct: f64,
    /// Mean distance from demand node to nearest station \[km\].
    pub average_distance_km: f64,
    /// Maximum distance from any demand node to nearest station \[km\].
    pub max_distance_km: f64,
    /// Total installed capacity \[kW\].
    pub total_capacity_kw: f64,
    /// Fraction of peak demand met (capped at 1.0).
    pub demand_satisfaction_pct: f64,
    /// Whether the plan passes all grid-capacity constraints.
    pub grid_feasible: bool,
    /// Benefit–cost ratio (dimensionless).
    pub benefit_cost_ratio: f64,
}

/// A violation of a grid capacity constraint caused by a deployment plan.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GridViolation {
    /// Bus where the violation occurs.
    pub bus_id: usize,
    /// Load excess beyond the constraint limit \[kW\].
    pub excess_kw: f64,
    /// Human-readable description.
    pub description: String,
}

/// Haversine great-circle distance between two WGS-84 coordinates \[km\].
///
/// # Arguments
/// * `lat1`, `lon1` — first point in decimal degrees
/// * `lat2`, `lon2` — second point in decimal degrees
pub fn haversine_km(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    const R: f64 = 6_371.0;
    let dlat = (lat2 - lat1).to_radians();
    let dlon = (lon2 - lon1).to_radians();
    let a = (dlat / 2.0).sin().powi(2)
        + lat1.to_radians().cos() * lat2.to_radians().cos() * (dlon / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().asin();
    R * c
}

/// Exponential-growth demand forecaster for EV charging sessions.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChargingDemandForecaster {
    /// Base number of daily charging sessions in year 0.
    pub base_daily_sessions: f64,
    /// Ratio of peak to average demand (dimensionless).
    pub base_peak_to_avg_ratio: f64,
}

impl ChargingDemandForecaster {
    /// Create a forecaster. Peak-to-average ratio defaults to 2.5.
    pub fn new(base_daily_sessions: f64) -> Self {
        Self {
            base_daily_sessions,
            base_peak_to_avg_ratio: 2.5,
        }
    }

    /// Project daily sessions: `base * (1 + growth)^t` for each year.
    pub fn forecast_daily_sessions(
        &self,
        ev_penetration_growth: f64,
        horizon_years: usize,
    ) -> Vec<f64> {
        (0..horizon_years)
            .map(|t| self.base_daily_sessions * (1.0 + ev_penetration_growth).powi(t as i32))
            .collect()
    }

    /// Return the peak-to-average demand ratio.
    pub fn compute_peak_to_average_ratio(&self) -> f64 {
        self.base_peak_to_avg_ratio
    }

    /// Seasonal factor: winter (12/1/2) → 1.15, summer (6/7/8) → 0.85, else 1.0.
    pub fn seasonal_adjustment(month: u32) -> f64 {
        match month {
            12 | 1 | 2 => 1.15,
            6..=8 => 0.85,
            _ => 1.0,
        }
    }
}

/// Choose the most common `ChargerType` from a slice, falling back to `Level2_22kw`.
fn most_common_charger(types: &[ChargerType]) -> ChargerType {
    if types.is_empty() {
        return ChargerType::Level2_22kw;
    }
    let variants = [
        ChargerType::Level1_1kw,
        ChargerType::Level2_7kw,
        ChargerType::Level2_22kw,
        ChargerType::Dcfc50kw,
        ChargerType::Dcfc150kw,
        ChargerType::Dcfc350kw,
    ];
    variants
        .iter()
        .max_by_key(|&&v| types.iter().filter(|&&t| t == v).count())
        .copied()
        .unwrap_or(ChargerType::Level2_22kw)
}

/// Build an [`InfrastructurePlan`] from a set of chosen stations and planning parameters.
fn build_plan(
    stations: Vec<ChargingStation>,
    demand_nodes: &[DemandNode],
    grid_constraints: &[GridConstraint],
    planning_horizon_years: f64,
    discount_rate: f64,
) -> InfrastructurePlan {
    let total_capital_cost_usd: f64 = stations.iter().map(|s| s.capital_cost_usd).sum();
    let total_annual_cost_usd: f64 = stations.iter().map(|s| s.annual_opex_usd).sum();
    let coverage_pct = if demand_nodes.is_empty() {
        1.0
    } else {
        let c = demand_nodes
            .iter()
            .filter(|n| {
                stations.iter().any(|s| {
                    haversine_km(n.location.0, n.location.1, s.location.0, s.location.1) <= 10.0
                })
            })
            .count();
        c as f64 / demand_nodes.len() as f64
    };
    let average_distance_km = if demand_nodes.is_empty() || stations.is_empty() {
        0.0
    } else {
        demand_nodes
            .iter()
            .map(|n| {
                stations
                    .iter()
                    .map(|s| haversine_km(n.location.0, n.location.1, s.location.0, s.location.1))
                    .fold(f64::MAX, f64::min)
            })
            .sum::<f64>()
            / demand_nodes.len() as f64
    };
    let grid_loading_pct: Vec<f64> = grid_constraints
        .iter()
        .map(|c| {
            let load: f64 = stations
                .iter()
                .filter(|s| s.grid_connection_bus == c.bus_id)
                .map(|s| {
                    s.capacity_kw
                        * if s.utilization_factor > 0.0 {
                            s.utilization_factor
                        } else {
                            0.5
                        }
                })
                .sum();
            if c.max_additional_load_kw > 0.0 {
                load / c.max_additional_load_kw
            } else {
                0.0
            }
        })
        .collect();
    let t = (planning_horizon_years as usize).max(1);
    let r = discount_rate;
    let e_yr: f64 = stations
        .iter()
        .map(|s| {
            s.capacity_kw
                * 8_760.0
                * if s.utilization_factor > 0.0 {
                    s.utilization_factor
                } else {
                    0.5
                }
        })
        .sum();
    let lcoe_usd_per_kwh = if e_yr > 0.0 {
        let po: f64 = (1..=t)
            .map(|y| total_annual_cost_usd / (1.0 + r).powi(y as i32))
            .sum();
        let pe: f64 = (1..=t).map(|y| e_yr / (1.0 + r).powi(y as i32)).sum();
        if pe > 0.0 {
            (total_capital_cost_usd + po) / pe
        } else {
            0.0
        }
    } else {
        0.0
    };

    InfrastructurePlan {
        stations,
        total_capital_cost_usd,
        total_annual_cost_usd,
        coverage_pct,
        average_distance_km,
        grid_loading_pct,
        lcoe_usd_per_kwh,
    }
}

/// Optimal EV charging infrastructure placement (greedy, LP-relaxation, p-median).
#[derive(Debug, Clone)]
pub struct InfrastructurePlanner {
    /// Demand nodes representing EV charging needs at geographic locations.
    pub demand_nodes: Vec<DemandNode>,
    /// Candidate geographic sites for charging station deployment.
    pub candidate_locations: Vec<(f64, f64)>,
    /// Power-network capacity constraints per bus.
    pub grid_constraints: Vec<GridConstraint>,
    /// Planning horizon for NPV / LCOE calculations \[years\].
    pub planning_horizon_years: f64,
    /// Discount rate for NPV / LCOE calculations (e.g. 0.08 = 8 %).
    pub discount_rate: f64,
    /// Minimum required demand-node coverage fraction (e.g. 0.90).
    pub target_coverage_pct: f64,
    /// Maximum allowable capital expenditure \[USD\].
    pub budget_usd: f64,
}

impl InfrastructurePlanner {
    /// Construct with defaults: 10-yr horizon, 8% discount, 90% coverage, unlimited budget.
    pub fn new(
        demand_nodes: Vec<DemandNode>,
        candidate_locations: Vec<(f64, f64)>,
        grid_constraints: Vec<GridConstraint>,
    ) -> Self {
        Self {
            demand_nodes,
            candidate_locations,
            grid_constraints,
            planning_horizon_years: 10.0,
            discount_rate: 0.08,
            target_coverage_pct: 0.90,
            budget_usd: f64::MAX,
        }
    }

    /// Greedy: add best demand-per-dollar site until coverage target or budget reached.
    pub fn solve_greedy(&self) -> InfrastructurePlan {
        self.solve_greedy_with_budget(self.budget_usd)
    }

    fn solve_greedy_with_budget(&self, budget: f64) -> InfrastructurePlan {
        if self.candidate_locations.is_empty() {
            return build_plan(
                vec![],
                &self.demand_nodes,
                &self.grid_constraints,
                self.planning_horizon_years,
                self.discount_rate,
            );
        }

        let mut scores: Vec<(usize, f64, ChargerType)> = self
            .candidate_locations
            .iter()
            .enumerate()
            .map(|(i, &(lat, lon))| {
                let preferred = self
                    .demand_nodes
                    .iter()
                    .min_by(|a, b| {
                        haversine_km(a.location.0, a.location.1, lat, lon)
                            .partial_cmp(&haversine_km(b.location.0, b.location.1, lat, lon))
                            .unwrap_or(std::cmp::Ordering::Equal)
                    })
                    .map(|n| n.preferred_charger_type)
                    .unwrap_or(ChargerType::Level2_22kw);
                let cost = preferred.capital_cost_usd() * 2.0;
                let dc = self
                    .demand_nodes
                    .iter()
                    .filter(|n| haversine_km(n.location.0, n.location.1, lat, lon) <= 10.0)
                    .count() as f64;
                let score = if cost > 0.0 {
                    dc * preferred.rated_kw() / cost
                } else {
                    0.0
                };
                (i, score, preferred)
            })
            .collect();

        scores.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        let mut stations: Vec<ChargingStation> = Vec::new();
        let mut remaining_budget = budget;
        let mut covered: std::collections::HashSet<usize> = std::collections::HashSet::new();

        for (idx, (cand_idx, _, ctype)) in scores.iter().enumerate() {
            let (lat, lon) = self.candidate_locations[*cand_idx];
            let cap_cost = ctype.capital_cost_usd() * 2.0;
            if cap_cost > remaining_budget {
                continue;
            }
            let bus = self
                .grid_constraints
                .first()
                .map(|c| c.bus_id)
                .unwrap_or(*cand_idx);
            let station = ChargingStation {
                id: idx,
                location: (lat, lon),
                capacity_kw: ctype.rated_kw() * 2.0,
                n_chargers: 2,
                charger_type: *ctype,
                capital_cost_usd: cap_cost,
                annual_opex_usd: cap_cost * 0.05,
                utilization_factor: 0.5,
                grid_connection_bus: bus,
            };
            for (nid, node) in self.demand_nodes.iter().enumerate() {
                if haversine_km(node.location.0, node.location.1, lat, lon) <= 10.0 {
                    covered.insert(nid);
                }
            }
            remaining_budget -= cap_cost;
            stations.push(station);
            let cov = if self.demand_nodes.is_empty() {
                1.0
            } else {
                covered.len() as f64 / self.demand_nodes.len() as f64
            };
            if cov >= self.target_coverage_pct {
                break;
            }
        }

        build_plan(
            stations,
            &self.demand_nodes,
            &self.grid_constraints,
            self.planning_horizon_years,
            self.discount_rate,
        )
    }

    /// LP relaxation: demand-score-weighted fractional allocation, rounded to binary.
    pub fn solve_milp_relaxation(&self) -> InfrastructurePlan {
        if self.candidate_locations.is_empty() {
            return build_plan(
                vec![],
                &self.demand_nodes,
                &self.grid_constraints,
                self.planning_horizon_years,
                self.discount_rate,
            );
        }

        // Compute raw demand scores and station costs.
        let candidates: Vec<(usize, f64, ChargerType, f64)> = self
            .candidate_locations
            .iter()
            .enumerate()
            .map(|(i, &(lat, lon))| {
                let preferred = self
                    .demand_nodes
                    .iter()
                    .min_by(|a, b| {
                        haversine_km(a.location.0, a.location.1, lat, lon)
                            .partial_cmp(&haversine_km(b.location.0, b.location.1, lat, lon))
                            .unwrap_or(std::cmp::Ordering::Equal)
                    })
                    .map(|n| n.preferred_charger_type)
                    .unwrap_or(ChargerType::Level2_22kw);
                let cost = preferred.capital_cost_usd() * 2.0;
                let demand: f64 = self
                    .demand_nodes
                    .iter()
                    .filter(|n| haversine_km(n.location.0, n.location.1, lat, lon) <= 10.0)
                    .map(|n| n.daily_ev_demand_kwh)
                    .sum();
                let score = if cost > 0.0 { demand / cost } else { 0.0 };
                (i, score, preferred, cost)
            })
            .collect();

        let total_score: f64 = candidates.iter().map(|(_, s, _, _)| s).sum();
        let mut remaining = self.budget_usd;
        let mut stations: Vec<ChargingStation> = Vec::new();

        let mut indexed: Vec<_> = candidates.iter().collect();
        indexed.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        for (sid, (i, score, ctype, cost)) in indexed.iter().enumerate() {
            let frac = if total_score > 0.0 {
                score / total_score
            } else {
                0.0
            };
            let x = if *cost > 0.0 {
                (frac * remaining).min(*cost) / cost
            } else {
                0.0
            };
            if x >= 0.5 && *cost <= remaining {
                let (lat, lon) = self.candidate_locations[*i];
                let bus = self
                    .grid_constraints
                    .first()
                    .map(|c| c.bus_id)
                    .unwrap_or(*i);
                stations.push(ChargingStation {
                    id: sid,
                    location: (lat, lon),
                    capacity_kw: ctype.rated_kw() * 2.0,
                    n_chargers: 2,
                    charger_type: *ctype,
                    capital_cost_usd: *cost,
                    annual_opex_usd: cost * 0.05,
                    utilization_factor: 0.5,
                    grid_connection_bus: bus,
                });
                remaining -= cost;
            }
            if remaining <= 0.0 {
                break;
            }
        }

        build_plan(
            stations,
            &self.demand_nodes,
            &self.grid_constraints,
            self.planning_horizon_years,
            self.discount_rate,
        )
    }

    /// p-median: minimise demand-weighted travel distance via Lloyd's algorithm.
    pub fn solve_p_median(&self, n_stations: usize) -> InfrastructurePlan {
        if n_stations == 0 || self.candidate_locations.is_empty() || self.demand_nodes.is_empty() {
            return build_plan(
                vec![],
                &self.demand_nodes,
                &self.grid_constraints,
                self.planning_horizon_years,
                self.discount_rate,
            );
        }

        let k = n_stations.min(self.candidate_locations.len());
        // Initialise centres as first k candidate locations.
        let mut centres: Vec<(f64, f64)> = self.candidate_locations[..k].to_vec();

        for _ in 0..100 {
            let mut clusters: Vec<Vec<usize>> = vec![vec![]; k];
            for (nid, node) in self.demand_nodes.iter().enumerate() {
                let best = centres
                    .iter()
                    .enumerate()
                    .min_by(|(_, a), (_, b)| {
                        haversine_km(node.location.0, node.location.1, a.0, a.1)
                            .partial_cmp(&haversine_km(node.location.0, node.location.1, b.0, b.1))
                            .unwrap_or(std::cmp::Ordering::Equal)
                    })
                    .map(|(i, _)| i)
                    .unwrap_or(0);
                clusters[best].push(nid);
            }
            let mut new_centres = centres.clone();
            for (ci, cluster) in clusters.iter().enumerate() {
                if cluster.is_empty() {
                    continue;
                }
                let tw: f64 = cluster
                    .iter()
                    .map(|&n| self.demand_nodes[n].daily_ev_demand_kwh)
                    .sum();
                if tw <= 0.0 {
                    continue;
                }
                let clat = cluster
                    .iter()
                    .map(|&n| {
                        self.demand_nodes[n].location.0 * self.demand_nodes[n].daily_ev_demand_kwh
                    })
                    .sum::<f64>()
                    / tw;
                let clon = cluster
                    .iter()
                    .map(|&n| {
                        self.demand_nodes[n].location.1 * self.demand_nodes[n].daily_ev_demand_kwh
                    })
                    .sum::<f64>()
                    / tw;
                let snapped = self
                    .candidate_locations
                    .iter()
                    .min_by(|a, b| {
                        haversine_km(clat, clon, a.0, a.1)
                            .partial_cmp(&haversine_km(clat, clon, b.0, b.1))
                            .unwrap_or(std::cmp::Ordering::Equal)
                    })
                    .copied()
                    .unwrap_or((clat, clon));
                new_centres[ci] = snapped;
            }
            if new_centres == centres {
                break;
            }
            centres = new_centres;
        }

        // Build stations from final centres.
        let mut stations: Vec<ChargingStation> = Vec::new();
        // Re-assign nodes to final centres for cluster composition.
        let mut clusters: Vec<Vec<usize>> = vec![vec![]; k];
        for (nid, node) in self.demand_nodes.iter().enumerate() {
            let best = centres
                .iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| {
                    haversine_km(node.location.0, node.location.1, a.0, a.1)
                        .partial_cmp(&haversine_km(node.location.0, node.location.1, b.0, b.1))
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .map(|(i, _)| i)
                .unwrap_or(0);
            clusters[best].push(nid);
        }

        for (sid, (centre, cluster)) in centres.iter().zip(clusters.iter()).enumerate() {
            if cluster.is_empty() {
                continue;
            }
            let types: Vec<ChargerType> = cluster
                .iter()
                .map(|&nid| self.demand_nodes[nid].preferred_charger_type)
                .collect();
            let ctype = most_common_charger(&types);
            let cap_kw: f64 = cluster
                .iter()
                .map(|&nid| self.demand_nodes[nid].peak_power_kw)
                .sum();
            let n_chargers = ((cap_kw / ctype.rated_kw()).ceil() as usize).max(1);
            let cap_cost = ctype.capital_cost_usd() * n_chargers as f64;
            let bus = self
                .grid_constraints
                .first()
                .map(|c| c.bus_id)
                .unwrap_or(sid);
            stations.push(ChargingStation {
                id: sid,
                location: *centre,
                capacity_kw: cap_kw,
                n_chargers,
                charger_type: ctype,
                capital_cost_usd: cap_cost,
                annual_opex_usd: cap_cost * 0.05,
                utilization_factor: 0.5,
                grid_connection_bus: bus,
            });
        }

        build_plan(
            stations,
            &self.demand_nodes,
            &self.grid_constraints,
            self.planning_horizon_years,
            self.discount_rate,
        )
    }

    /// Compute detailed performance metrics for a deployment plan.
    pub fn evaluate_plan(&self, plan: &InfrastructurePlan) -> PlanMetrics {
        let coverage_pct = plan.coverage_pct;

        let (avg_dist, max_dist) = if self.demand_nodes.is_empty() || plan.stations.is_empty() {
            (0.0, 0.0)
        } else {
            let dists: Vec<f64> = self
                .demand_nodes
                .iter()
                .map(|n| {
                    plan.stations
                        .iter()
                        .map(|s| {
                            haversine_km(n.location.0, n.location.1, s.location.0, s.location.1)
                        })
                        .fold(f64::MAX, f64::min)
                })
                .collect();
            let avg = dists.iter().sum::<f64>() / dists.len() as f64;
            let max = dists.iter().cloned().fold(0.0_f64, f64::max);
            (avg, max)
        };

        let total_capacity_kw: f64 = plan.stations.iter().map(|s| s.capacity_kw).sum();
        let total_peak_demand: f64 = self.demand_nodes.iter().map(|n| n.peak_power_kw).sum();
        let demand_satisfaction_pct = if total_peak_demand > 0.0 {
            (total_capacity_kw / total_peak_demand).min(1.0)
        } else {
            1.0
        };
        let grid_feasible = self.check_grid_feasibility(plan).is_empty();
        let benefit_cost_ratio = if plan.total_annual_cost_usd > 0.0 {
            coverage_pct * total_capacity_kw * 365.0 * 0.3 / plan.total_annual_cost_usd
        } else {
            1.0
        };

        PlanMetrics {
            coverage_pct,
            average_distance_km: avg_dist,
            max_distance_km: max_dist,
            total_capacity_kw,
            demand_satisfaction_pct,
            grid_feasible,
            benefit_cost_ratio,
        }
    }

    /// Demand-weighted fraction of daily kWh within 10 km of a station.
    pub fn compute_accessibility_index(&self, plan: &InfrastructurePlan) -> f64 {
        if self.demand_nodes.is_empty() {
            return 1.0;
        }
        let total_w: f64 = self
            .demand_nodes
            .iter()
            .map(|n| n.daily_ev_demand_kwh)
            .sum();
        if total_w <= 0.0 {
            return 1.0;
        }
        let accessible_w: f64 = self
            .demand_nodes
            .iter()
            .filter(|n| {
                plan.stations.iter().any(|s| {
                    haversine_km(n.location.0, n.location.1, s.location.0, s.location.1) <= 10.0
                })
            })
            .map(|n| n.daily_ev_demand_kwh)
            .sum();
        accessible_w / total_w
    }

    /// Identify grid capacity violations caused by a deployment plan.
    pub fn check_grid_feasibility(&self, plan: &InfrastructurePlan) -> Vec<GridViolation> {
        self.grid_constraints
            .iter()
            .filter_map(|c| {
                let load: f64 = plan
                    .stations
                    .iter()
                    .filter(|s| s.grid_connection_bus == c.bus_id)
                    .map(|s| {
                        let uf = if s.utilization_factor > 0.0 {
                            s.utilization_factor
                        } else {
                            0.5
                        };
                        s.capacity_kw * uf
                    })
                    .sum();
                if load > c.max_additional_load_kw {
                    Some(GridViolation {
                        bus_id: c.bus_id,
                        excess_kw: load - c.max_additional_load_kw,
                        description: format!(
                            "Bus {}: EV load {:.1} kW exceeds limit {:.1} kW",
                            c.bus_id, load, c.max_additional_load_kw
                        ),
                    })
                } else {
                    None
                }
            })
            .collect()
    }

    /// Generate `n` alternative plans with LCG-perturbed budgets (±20 %).
    pub fn generate_alternatives(&self, n: usize) -> Vec<InfrastructurePlan> {
        let mut st: u64 = 0xDEAD_BEEF_CAFE_1234;
        (0..n)
            .map(|_| {
                st = st
                    .wrapping_mul(6_364_136_223_846_793_005_u64)
                    .wrapping_add(1_442_695_040_888_963_407_u64);
                let factor = 0.8 + 0.4 * (st % 1_000) as f64 / 1_000.0;
                let budget = if self.budget_usd == f64::MAX {
                    f64::MAX
                } else {
                    self.budget_usd * factor
                };
                self.solve_greedy_with_budget(budget)
            })
            .collect()
    }

    /// NPV = -C₀ + Σ (R_t - O_t) / (1+r)^t over the planning horizon.
    pub fn compute_npv(&self, plan: &InfrastructurePlan, annual_revenue_usd: f64) -> f64 {
        let t = (self.planning_horizon_years as usize).max(1);
        let r = self.discount_rate;
        let pv: f64 = (1..=t)
            .map(|y| (annual_revenue_usd - plan.total_annual_cost_usd) / (1.0 + r).powi(y as i32))
            .sum();
        -plan.total_capital_cost_usd + pv
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_location(
        id: &str,
        bus: usize,
        loc_type: LocationType,
        daily_v: f64,
    ) -> ChargingLocation {
        ChargingLocation {
            location_id: id.to_string(),
            bus_id: bus,
            location_type: loc_type,
            available_area_sqm: 500.0,
            daily_vehicles: daily_v,
            peak_hour_fraction: 0.1,
            grid_capacity_kw: 1_000.0,
            lat: 0.0,
            lon: 0.0,
        }
    }

    fn base_forecast() -> EvDemandForecast {
        EvDemandForecast {
            year: 2025,
            ev_penetration_pct: 10.0,
            avg_daily_km: 50.0,
            avg_consumption_kwh_per_km: 0.2,
            home_charging_pct: 0.8,
            work_charging_pct: 0.1,
            public_charging_pct: 0.1,
            charging_preference: ChargingPreference::Uncoordinated,
        }
    }

    fn default_planner(locations: Vec<ChargingLocation>) -> EvInfraPlanner {
        EvInfraPlanner::new(locations, InfrastructurePlanConfig::default())
    }

    // ── Test 1: demand forecast — EV penetration 10 % ────────────────────

    #[test]
    fn test_demand_forecast_ev_penetration() {
        let planner = default_planner(vec![]);
        let forecast = base_forecast();
        let total_vehicles = 10_000.0;

        let results = planner.forecast_demand(&forecast, total_vehicles, 3);
        assert_eq!(results.len(), 3);

        // Year 0: 10 % of 10 000 = 1 000 EVs × 50 km × 0.2 kWh/km = 10 000 kWh/day
        let y0 = &results[0];
        assert!(
            (y0.total_ev_kwh_per_day - 10_000.0).abs() < 1.0,
            "Expected ~10000 kWh/day, got {}",
            y0.total_ev_kwh_per_day
        );

        // Home split: 80 %
        assert!(
            (y0.home_kwh - 8_000.0).abs() < 1.0,
            "Expected ~8000 kWh home, got {}",
            y0.home_kwh
        );

        // Year 1 penetration grows by 5 %
        let y1 = &results[1];
        assert!(
            y1.total_ev_kwh_per_day > y0.total_ev_kwh_per_day,
            "Year 1 demand should exceed year 0"
        );
    }

    // ── Test 2: charger sizing — higher demand → more chargers ───────────

    #[test]
    fn test_charger_sizing_higher_demand_more_chargers() {
        let loc = make_location("A", 0, LocationType::Commercial, 100.0);
        let planner = default_planner(vec![loc.clone()]);

        let sizing_low = planner.size_chargers(&loc, 22.0, 0.95);
        let sizing_high = planner.size_chargers(&loc, 220.0, 0.95);

        assert!(
            sizing_high.num_chargers >= sizing_low.num_chargers,
            "Higher demand should require >= chargers: low={}, high={}",
            sizing_low.num_chargers,
            sizing_high.num_chargers
        );
    }

    // ── Test 3: placement — highest B/C location selected first ─────────

    #[test]
    fn test_placement_highest_bc_first() {
        // Location A: 1000 vehicles/day (high benefit)
        // Location B: 10 vehicles/day (low benefit)
        let loc_a = make_location("A", 0, LocationType::Commercial, 1000.0);
        let loc_b = make_location("B", 1, LocationType::Commercial, 10.0);

        let planner = default_planner(vec![loc_a, loc_b]);
        let forecast = planner.forecast_demand(&base_forecast(), 10_000.0, 1);
        // Budget only enough for one high-traffic site
        let plan = planner.optimal_charger_placement(&forecast, 500_000.0);

        // At least one site allocated
        assert!(
            !plan.sites.is_empty(),
            "Expected at least one site allocated"
        );

        // If both sites fit, A should appear (higher vehicles)
        let has_a = plan.sites.iter().any(|s| s.location_id == "A");
        assert!(has_a, "High-traffic location A should be selected");
    }

    // ── Test 4: grid impact — large load → voltage violation ─────────────

    #[test]
    fn test_grid_impact_large_load_voltage_violation() {
        let loc = make_location("A", 0, LocationType::Highway, 500.0);
        let planner = default_planner(vec![loc]);

        // Manually build a plan with a large DCFC site
        let plan = PlacementPlan {
            sites: vec![SiteAllocation {
                location_id: "A".to_string(),
                num_chargers: 20,
                charger_type: PlanChargerType::DcFastCharge { power_kw: 150.0 },
                cost_usd: 1_000_000.0,
            }],
            total_cost_usd: 1_000_000.0,
            total_capacity_kw: 3_000.0,
            expected_annual_revenue: 500_000.0,
        };

        // Bus 0 voltage at 0.97 pu; 3 MW addition → ΔV = 0.3 pu → drops to 0.67 pu < 0.95
        let bus_v = vec![0.97_f64];
        let branch_flows = vec![0.5_f64];
        let branch_ratings = vec![5.0_f64];

        let impact = planner.grid_impact_assessment(&plan, &bus_v, &branch_flows, &branch_ratings);

        assert!(
            !impact.voltage_violations.is_empty(),
            "Expected voltage violation from 3 MW DCFC addition"
        );
        assert!(impact.max_voltage_drop_pu > 0.0);
        assert!(impact.upgrade_cost_usd > 0.0);
    }

    // ── Test 5: NPV — positive NPV for high-utilisation site ─────────────

    #[test]
    fn test_npv_positive_high_utilization() {
        let loc = make_location("A", 0, LocationType::Commercial, 500.0);
        let planner = default_planner(vec![loc]);
        let forecast = planner.forecast_demand(&base_forecast(), 50_000.0, 10);

        // Plan with modest cost but high revenue (500 vehicles × $5/session × 365)
        let plan = PlacementPlan {
            sites: vec![SiteAllocation {
                location_id: "A".to_string(),
                num_chargers: 5,
                charger_type: PlanChargerType::Level2 { power_kw: 22.0 },
                cost_usd: 25_000.0,
            }],
            total_cost_usd: 25_000.0,
            total_capacity_kw: 110.0,
            expected_annual_revenue: 500.0 * 5.0 * 365.0, // $912 500/year
        };

        let npv = planner.calculate_npv(&plan, &forecast);
        assert!(
            npv > 0.0,
            "Expected positive NPV for high-utilisation site, got {npv}"
        );
    }

    // ── Test 6: blocking probability — more chargers → lower blocking ─────

    #[test]
    fn test_blocking_probability_more_chargers() {
        // rho = 5 Erlangs
        let rho = 5.0;
        let bp5 = erlang_b(rho, 5);
        let bp10 = erlang_b(rho, 10);
        let bp20 = erlang_b(rho, 20);

        assert!(
            bp5 >= bp10,
            "More chargers should reduce blocking: bp5={bp5:.4} >= bp10={bp10:.4}"
        );
        assert!(
            bp10 >= bp20,
            "More chargers should reduce blocking: bp10={bp10:.4} >= bp20={bp20:.4}"
        );
        assert!(
            bp20 < 0.01,
            "20 chargers for 5 Erlang should have <1% blocking"
        );
    }

    // ── Test 7: DR potential — 50 % smart chargers → significant DR ──────

    #[test]
    fn test_dr_potential_50pct_smart_chargers() {
        let loc = make_location("A", 0, LocationType::Commercial, 200.0);
        let planner = default_planner(vec![loc]);

        let plan = PlacementPlan {
            sites: vec![SiteAllocation {
                location_id: "A".to_string(),
                num_chargers: 10,
                charger_type: PlanChargerType::Level2 { power_kw: 22.0 },
                cost_usd: 50_000.0,
            }],
            total_cost_usd: 50_000.0,
            total_capacity_kw: 220.0,
            expected_annual_revenue: 100_000.0,
        };

        let dr = planner.demand_response_potential(&plan, 0.5, 150.0, 40.0);

        // 50 % of 220 kW = 110 kW = 0.11 MW
        assert!(
            (dr.dr_capacity_mw - 0.11).abs() < 1e-6,
            "Expected 0.11 MW DR capacity, got {}",
            dr.dr_capacity_mw
        );
        assert!(dr.annual_value_usd > 0.0, "DR should have positive value");
        assert_eq!(dr.peak_reduction_mw, dr.dr_capacity_mw);
    }

    // ── Test 8: equity — uniform distribution → Gini ≈ 0 ───────────────

    #[test]
    fn test_equity_uniform_distribution_gini_near_zero() {
        // 5 locations, each with same income level and same population
        let locs: Vec<ChargingLocation> = (0..5)
            .map(|i| {
                let mut loc = make_location(&format!("L{i}"), i, LocationType::Commercial, 100.0);
                loc.peak_hour_fraction = 0.1;
                loc
            })
            .collect();
        let planner = default_planner(locs.clone());

        // Uniform plan: 2 chargers at every location
        let sites: Vec<SiteAllocation> = locs
            .iter()
            .map(|l| SiteAllocation {
                location_id: l.location_id.clone(),
                num_chargers: 2,
                charger_type: PlanChargerType::Level2 { power_kw: 22.0 },
                cost_usd: 10_000.0,
            })
            .collect();

        let plan = PlacementPlan {
            total_cost_usd: 50_000.0,
            total_capacity_kw: 220.0,
            expected_annual_revenue: 100_000.0,
            sites,
        };

        let income_levels = vec![1.0_f64; 5];
        let population = vec![1000.0_f64; 5];

        let report = planner.equity_analysis(&plan, &income_levels, &population);

        assert!(
            report.gini_coefficient.abs() < 0.05,
            "Uniform distribution should have near-zero Gini, got {}",
            report.gini_coefficient
        );
    }

    // ── Test 9: Erlang B — zero rho → zero blocking ──────────────────────

    #[test]
    fn test_erlang_b_zero_rho() {
        assert_eq!(erlang_b(0.0, 5), 0.0);
    }

    // ── Test 10: forecast year count correct ─────────────────────────────

    #[test]
    fn test_forecast_year_count() {
        let planner = default_planner(vec![]);
        let f = base_forecast();
        let results = planner.forecast_demand(&f, 1000.0, 7);
        assert_eq!(results.len(), 7);
        for (i, y) in results.iter().enumerate() {
            assert_eq!(y.year, 2025 + i);
        }
    }

    // ── New tests for InfrastructurePlanner / ChargerType / helpers ───────

    fn make_demand_node(id: usize, lat: f64, lon: f64) -> DemandNode {
        DemandNode {
            id,
            location: (lat, lon),
            daily_ev_demand_kwh: 100.0,
            peak_power_kw: 50.0,
            ev_count: 10,
            preferred_charger_type: ChargerType::Level2_22kw,
        }
    }

    #[test]
    fn test_haversine_known_distance() {
        let d = haversine_km(40.7128, -74.0060, 51.5074, -0.1278);
        assert!((d - 5570.0).abs() < 100.0, "Expected ~5570 km, got {d:.1}");
    }

    #[test]
    fn test_greedy_solver() {
        let nodes = (0..5)
            .map(|i| make_demand_node(i, i as f64 * 0.05, i as f64 * 0.05))
            .collect();
        let candidates = vec![(0.0, 0.0), (0.1, 0.1), (0.2, 0.2)];
        let planner = InfrastructurePlanner::new(nodes, candidates, vec![]);
        let plan = planner.solve_greedy();
        assert!(plan.coverage_pct >= 0.0);
        assert!(plan.total_capital_cost_usd <= planner.budget_usd);
    }

    #[test]
    fn test_p_median_solver() {
        let nodes: Vec<DemandNode> = (0..9)
            .map(|i| make_demand_node(i, (i / 3) as f64, (i % 3) as f64))
            .collect();
        let candidates: Vec<(f64, f64)> =
            (0..9).map(|i| ((i / 3) as f64, (i % 3) as f64)).collect();
        let planner = InfrastructurePlanner::new(nodes, candidates, vec![]);
        let plan = planner.solve_p_median(3);
        assert!(plan.stations.len() <= 3);
    }

    #[test]
    fn test_milp_relaxation() {
        let nodes = vec![make_demand_node(0, 0.0, 0.0)];
        let planner = InfrastructurePlanner::new(nodes, vec![(0.0, 0.0), (1.0, 1.0)], vec![]);
        let plan = planner.solve_milp_relaxation();
        assert!(plan.total_capital_cost_usd <= planner.budget_usd);
    }

    #[test]
    fn test_coverage_calculation() {
        let nodes = vec![
            make_demand_node(0, 0.0, 0.0),
            make_demand_node(1, 45.0, 90.0), // far away
        ];
        let planner = InfrastructurePlanner::new(nodes, vec![(0.0, 0.0)], vec![]);
        let plan = planner.solve_greedy();
        let metrics = planner.evaluate_plan(&plan);
        assert!(metrics.coverage_pct >= 0.0 && metrics.coverage_pct <= 1.0);
    }

    #[test]
    fn test_grid_constraint_check() {
        let station = ChargingStation {
            id: 0,
            location: (0.0, 0.0),
            capacity_kw: 500.0,
            n_chargers: 5,
            charger_type: ChargerType::Dcfc150kw,
            capital_cost_usd: 400_000.0,
            annual_opex_usd: 20_000.0,
            utilization_factor: 0.8,
            grid_connection_bus: 0,
        };
        let plan = InfrastructurePlan {
            stations: vec![station],
            total_capital_cost_usd: 400_000.0,
            total_annual_cost_usd: 20_000.0,
            coverage_pct: 1.0,
            average_distance_km: 0.0,
            grid_loading_pct: vec![0.8],
            lcoe_usd_per_kwh: 0.2,
        };
        let constraint = GridConstraint {
            bus_id: 0,
            max_additional_load_kw: 100.0,
            available_capacity_kw: 100.0,
        };
        let planner = InfrastructurePlanner::new(vec![], vec![], vec![constraint]);
        let violations = planner.check_grid_feasibility(&plan);
        assert!(!violations.is_empty(), "Should detect overload at bus 0");
        assert_eq!(violations[0].bus_id, 0);
    }

    #[test]
    fn test_npv_positive() {
        let plan = InfrastructurePlan {
            stations: vec![],
            total_capital_cost_usd: 10_000.0,
            total_annual_cost_usd: 500.0,
            coverage_pct: 1.0,
            average_distance_km: 0.0,
            grid_loading_pct: vec![],
            lcoe_usd_per_kwh: 0.15,
        };
        let planner = InfrastructurePlanner::new(vec![], vec![], vec![]);
        let npv = planner.compute_npv(&plan, 5_000.0);
        assert!(npv > 0.0, "NPV should be positive with high revenue: {npv}");
    }

    #[test]
    fn test_lcoe_calculation() {
        let nodes = vec![DemandNode {
            id: 0,
            location: (0.0, 0.0),
            daily_ev_demand_kwh: 100.0,
            peak_power_kw: 50.0,
            ev_count: 5,
            preferred_charger_type: ChargerType::Dcfc50kw,
        }];
        let planner = InfrastructurePlanner::new(nodes, vec![(0.0, 0.0)], vec![]);
        let solved = planner.solve_greedy();
        if !solved.stations.is_empty() {
            assert!(solved.lcoe_usd_per_kwh >= 0.0);
        }
    }

    #[test]
    fn test_demand_forecaster() {
        let f = ChargingDemandForecaster::new(100.0);
        let sessions = f.forecast_daily_sessions(0.20, 5);
        assert_eq!(sessions.len(), 5);
        assert!((sessions[0] - 100.0).abs() < 1e-6);
        assert!(sessions[4] > sessions[0]);
    }

    #[test]
    fn test_seasonal_adjustment() {
        let winter = ChargingDemandForecaster::seasonal_adjustment(1);
        let summer = ChargingDemandForecaster::seasonal_adjustment(7);
        let spring = ChargingDemandForecaster::seasonal_adjustment(4);
        assert!(winter > summer, "Winter > summer");
        assert!((spring - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_alternatives_generation() {
        let nodes = vec![make_demand_node(0, 0.0, 0.0)];
        let planner = InfrastructurePlanner::new(nodes, vec![(0.0, 0.0), (1.0, 1.0)], vec![]);
        let alts = planner.generate_alternatives(3);
        assert_eq!(alts.len(), 3);
    }

    #[test]
    fn test_accessibility_index() {
        let nodes = vec![make_demand_node(0, 0.0, 0.0)];
        let planner = InfrastructurePlanner::new(nodes, vec![(0.0, 0.0)], vec![]);
        let plan = planner.solve_greedy();
        let idx = planner.compute_accessibility_index(&plan);
        assert!((0.0..=1.0).contains(&idx));
    }

    #[test]
    fn test_empty_demand() {
        let planner = InfrastructurePlanner::new(vec![], vec![(0.0, 0.0)], vec![]);
        let plan = planner.solve_greedy();
        assert!(plan.coverage_pct >= 0.0);
        let plan2 = planner.solve_p_median(2);
        assert!(plan2.stations.is_empty());
    }

    #[test]
    fn test_budget_constraint() {
        let nodes: Vec<DemandNode> = (0..5)
            .map(|i| make_demand_node(i, i as f64 * 0.01, 0.0))
            .collect();
        let candidates: Vec<(f64, f64)> = (0..5).map(|i| (i as f64 * 0.01, 0.0)).collect();
        let mut planner = InfrastructurePlanner::new(nodes, candidates, vec![]);
        planner.budget_usd = 20_000.0;
        let plan = planner.solve_greedy();
        assert!(plan.total_capital_cost_usd <= 20_001.0);
    }

    #[test]
    fn test_charger_type_costs() {
        assert!(
            ChargerType::Dcfc50kw.capital_cost_usd() > ChargerType::Level2_22kw.capital_cost_usd()
        );
        assert!(
            ChargerType::Dcfc350kw.capital_cost_usd() > ChargerType::Dcfc150kw.capital_cost_usd()
        );
        assert_eq!(ChargerType::Level1_1kw.capital_cost_usd(), 500.0);
    }

    #[test]
    fn test_grid_feasibility_violations() {
        let station = ChargingStation {
            id: 0,
            location: (0.0, 0.0),
            capacity_kw: 1000.0,
            n_chargers: 10,
            charger_type: ChargerType::Dcfc150kw,
            capital_cost_usd: 800_000.0,
            annual_opex_usd: 40_000.0,
            utilization_factor: 1.0,
            grid_connection_bus: 1,
        };
        let plan = InfrastructurePlan {
            stations: vec![station],
            total_capital_cost_usd: 800_000.0,
            total_annual_cost_usd: 40_000.0,
            coverage_pct: 1.0,
            average_distance_km: 0.0,
            grid_loading_pct: vec![],
            lcoe_usd_per_kwh: 0.0,
        };
        let planner = InfrastructurePlanner::new(
            vec![],
            vec![],
            vec![GridConstraint {
                bus_id: 1,
                max_additional_load_kw: 200.0,
                available_capacity_kw: 200.0,
            }],
        );
        let violations = planner.check_grid_feasibility(&plan);
        assert!(!violations.is_empty());
        assert!(violations[0].excess_kw > 0.0);
    }

    #[test]
    fn test_plan_metrics() {
        let nodes = vec![make_demand_node(0, 0.0, 0.0)];
        let planner = InfrastructurePlanner::new(nodes, vec![(0.0, 0.0)], vec![]);
        let plan = planner.solve_greedy();
        let m = planner.evaluate_plan(&plan);
        assert!(m.coverage_pct >= 0.0 && m.coverage_pct <= 1.0);
        assert!(m.total_capacity_kw >= 0.0);
    }

    #[test]
    fn test_p_median_convergence() {
        let nodes: Vec<DemandNode> = (0..6)
            .map(|i| make_demand_node(i, i as f64 * 0.01, 0.0))
            .collect();
        let candidates: Vec<(f64, f64)> = (0..6).map(|i| (i as f64 * 0.01, 0.0)).collect();
        let planner = InfrastructurePlanner::new(nodes, candidates, vec![]);
        let plan1 = planner.solve_p_median(2);
        let plan2 = planner.solve_p_median(2);
        assert_eq!(plan1.stations.len(), plan2.stations.len());
    }

    #[test]
    fn test_coverage_improvement() {
        let nodes: Vec<DemandNode> = (0..4)
            .map(|i| make_demand_node(i, i as f64 * 0.05, 0.0))
            .collect();
        let candidates = vec![(0.0, 0.0), (0.1, 0.0), (0.2, 0.0)];
        let planner = InfrastructurePlanner::new(nodes, candidates, vec![]);
        let m1 = planner.evaluate_plan(&planner.solve_p_median(1));
        let m2 = planner.evaluate_plan(&planner.solve_p_median(2));
        assert!(m2.coverage_pct >= m1.coverage_pct - 0.01);
    }

    #[test]
    fn test_bcr_calculation() {
        let station = ChargingStation {
            id: 0,
            location: (0.0, 0.0),
            capacity_kw: 100.0,
            n_chargers: 2,
            charger_type: ChargerType::Level2_22kw,
            capital_cost_usd: 16_000.0,
            annual_opex_usd: 2_000.0,
            utilization_factor: 0.5,
            grid_connection_bus: 0,
        };
        let plan = InfrastructurePlan {
            stations: vec![station],
            total_capital_cost_usd: 16_000.0,
            total_annual_cost_usd: 2_000.0,
            coverage_pct: 1.0,
            average_distance_km: 0.5,
            grid_loading_pct: vec![],
            lcoe_usd_per_kwh: 0.15,
        };
        let nodes = vec![DemandNode {
            id: 0,
            location: (0.0, 0.0),
            daily_ev_demand_kwh: 100.0,
            peak_power_kw: 100.0,
            ev_count: 10,
            preferred_charger_type: ChargerType::Level2_22kw,
        }];
        let planner = InfrastructurePlanner::new(nodes, vec![], vec![]);
        let metrics = planner.evaluate_plan(&plan);
        assert!(metrics.benefit_cost_ratio >= 0.0);
    }
}
