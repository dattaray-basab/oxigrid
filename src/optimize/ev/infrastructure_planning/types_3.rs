//! Auto-generated module
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

use super::functions::charger_capex;
use super::types::{
    ChargerSizing, ChargingLocation, DemandForecastYear, DrPotential, EquityReport,
    EvDemandForecast, GridImpact, InfrastructurePlanConfig, PlacementPlan, SiteAllocation,
};

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
            pen *= 1.05;
        }
        results
    }
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
        let charger_type = super::functions::charger_type_for_location(&location.location_type);
        let charger_kw = charger_type.power_kw().max(1.0);
        let rho = demand_kw_peak / charger_kw;
        let max_blocking = (1.0 - reliability_target).max(0.0);
        let mut n = (rho.ceil() as usize).max(1);
        let mut bp = super::functions::erlang_b(rho, n);
        while bp > max_blocking && n < 200 {
            n += 1;
            bp = super::functions::erlang_b(rho, n);
        }
        let max_chargers_by_grid = if charger_kw > 0.0 {
            (location.grid_capacity_kw / charger_kw).floor() as usize
        } else {
            n
        };
        n = n.min(max_chargers_by_grid).max(1);
        bp = super::functions::erlang_b(rho, n);
        let total_capacity_kw = n as f64 * charger_kw;
        ChargerSizing {
            location_id: location.location_id.clone(),
            charger_type,
            num_chargers: n,
            total_capacity_kw,
            blocking_probability: bp,
        }
    }
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
        const AVG_SESSION_KWH: f64 = 20.0;
        const AVG_REVENUE_PER_SESSION: f64 = 5.0;
        let horizon = self.config.planning_horizon_years as f64;
        let annualized_cost = |capex: f64| -> f64 {
            capex / horizon + capex * self.config.charger_cost.annual_opex_pct
        };
        let mut candidates: Vec<(usize, f64, ChargerSizing, f64)> = self
            .locations
            .iter()
            .enumerate()
            .filter_map(|(idx, loc)| {
                let sessions_peak = loc.daily_vehicles * loc.peak_hour_fraction;
                let demand_kw_peak = sessions_peak * AVG_SESSION_KWH;
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
            if sizing.total_capacity_kw > loc.grid_capacity_kw + 1e-6 {
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
        let mut expected_annual_revenue = 0.0_f64;
        for site in &sites {
            let daily_v = self
                .locations
                .iter()
                .find(|l| l.location_id == site.location_id)
                .map(|l| l.daily_vehicles)
                .unwrap_or(0.0);
            expected_annual_revenue += daily_v * AVG_REVENUE_PER_SESSION * 365.0;
        }
        let utilisation_scale = demand_forecast
            .first()
            .map(|y| {
                let total_pub = y.public_kwh;
                let total_cap = total_capacity_kw * 24.0;
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
        const Z_SENSITIVITY: f64 = 0.1;
        let mut voltage_violations: Vec<usize> = Vec::new();
        let mut thermal_violations: Vec<usize> = Vec::new();
        let mut required_upgrade_kw = 0.0_f64;
        let mut max_voltage_drop_pu = 0.0_f64;
        for site in &placement_plan.sites {
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
            if let Some(&v_pre) = bus_voltages_pu.get(bus_id) {
                let v_post = v_pre - delta_v;
                if v_post < 0.95 {
                    if !voltage_violations.contains(&bus_id) {
                        voltage_violations.push(bus_id);
                    }
                    required_upgrade_kw += load_addition_mw * 1000.0;
                }
            } else {
                if load_addition_mw > 1.0 {
                    if !voltage_violations.contains(&bus_id) {
                        voltage_violations.push(bus_id);
                    }
                    required_upgrade_kw += load_addition_mw * 1000.0;
                }
            }
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
        let gini = super::functions::gini_coefficient(&coverage_by_quintile);
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
