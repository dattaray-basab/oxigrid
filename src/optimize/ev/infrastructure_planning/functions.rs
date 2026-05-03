//! Auto-generated module
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

use super::types::{
    ChargerCost, ChargerType, ChargingStation, DemandNode, GridConstraint, InfrastructurePlan,
    LocationType, PlanChargerType,
};

/// Choose charger type based on location category.
pub fn charger_type_for_location(loc_type: &LocationType) -> PlanChargerType {
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
pub fn charger_capex(
    charger_type: &PlanChargerType,
    num_chargers: usize,
    cost: &ChargerCost,
) -> f64 {
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
pub fn erlang_b(rho: f64, n: usize) -> f64 {
    if rho <= 0.0 {
        return 0.0;
    }
    let mut b = 1.0_f64;
    for k in 1..=n {
        b = rho * b / (k as f64 + rho * b);
    }
    b
}
/// Gini coefficient for a distribution vector (0 = perfect equality).
pub fn gini_coefficient(values: &[f64]) -> f64 {
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
    numerator / (n as f64 * sum) - (n as f64 + 1.0) / n as f64
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
/// Choose the most common `ChargerType` from a slice, falling back to `Level2_22kw`.
pub fn most_common_charger(types: &[ChargerType]) -> ChargerType {
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
pub fn build_plan(
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
#[cfg(test)]
mod tests {
    use super::super::*;
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
    #[test]
    fn test_demand_forecast_ev_penetration() {
        let planner = default_planner(vec![]);
        let forecast = base_forecast();
        let total_vehicles = 10_000.0;
        let results = planner.forecast_demand(&forecast, total_vehicles, 3);
        assert_eq!(results.len(), 3);
        let y0 = &results[0];
        assert!(
            (y0.total_ev_kwh_per_day - 10_000.0).abs() < 1.0,
            "Expected ~10000 kWh/day, got {}",
            y0.total_ev_kwh_per_day
        );
        assert!(
            (y0.home_kwh - 8_000.0).abs() < 1.0,
            "Expected ~8000 kWh home, got {}",
            y0.home_kwh
        );
        let y1 = &results[1];
        assert!(
            y1.total_ev_kwh_per_day > y0.total_ev_kwh_per_day,
            "Year 1 demand should exceed year 0"
        );
    }
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
    #[test]
    fn test_placement_highest_bc_first() {
        let loc_a = make_location("A", 0, LocationType::Commercial, 1000.0);
        let loc_b = make_location("B", 1, LocationType::Commercial, 10.0);
        let planner = default_planner(vec![loc_a, loc_b]);
        let forecast = planner.forecast_demand(&base_forecast(), 10_000.0, 1);
        let plan = planner.optimal_charger_placement(&forecast, 500_000.0);
        assert!(
            !plan.sites.is_empty(),
            "Expected at least one site allocated"
        );
        let has_a = plan.sites.iter().any(|s| s.location_id == "A");
        assert!(has_a, "High-traffic location A should be selected");
    }
    #[test]
    fn test_grid_impact_large_load_voltage_violation() {
        let loc = make_location("A", 0, LocationType::Highway, 500.0);
        let planner = default_planner(vec![loc]);
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
    #[test]
    fn test_npv_positive_high_utilization() {
        let loc = make_location("A", 0, LocationType::Commercial, 500.0);
        let planner = default_planner(vec![loc]);
        let forecast = planner.forecast_demand(&base_forecast(), 50_000.0, 10);
        let plan = PlacementPlan {
            sites: vec![SiteAllocation {
                location_id: "A".to_string(),
                num_chargers: 5,
                charger_type: PlanChargerType::Level2 { power_kw: 22.0 },
                cost_usd: 25_000.0,
            }],
            total_cost_usd: 25_000.0,
            total_capacity_kw: 110.0,
            expected_annual_revenue: 500.0 * 5.0 * 365.0,
        };
        let npv = planner.calculate_npv(&plan, &forecast);
        assert!(
            npv > 0.0,
            "Expected positive NPV for high-utilisation site, got {npv}"
        );
    }
    #[test]
    fn test_blocking_probability_more_chargers() {
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
        assert!(
            (dr.dr_capacity_mw - 0.11).abs() < 1e-6,
            "Expected 0.11 MW DR capacity, got {}",
            dr.dr_capacity_mw
        );
        assert!(dr.annual_value_usd > 0.0, "DR should have positive value");
        assert_eq!(dr.peak_reduction_mw, dr.dr_capacity_mw);
    }
    #[test]
    fn test_equity_uniform_distribution_gini_near_zero() {
        let locs: Vec<ChargingLocation> = (0..5)
            .map(|i| {
                let mut loc = make_location(&format!("L{i}"), i, LocationType::Commercial, 100.0);
                loc.peak_hour_fraction = 0.1;
                loc
            })
            .collect();
        let planner = default_planner(locs.clone());
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
    #[test]
    fn test_erlang_b_zero_rho() {
        assert_eq!(erlang_b(0.0, 5), 0.0);
    }
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
            make_demand_node(1, 45.0, 90.0),
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
