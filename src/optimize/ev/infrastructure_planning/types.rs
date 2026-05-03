//! Auto-generated module
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

use serde::{Deserialize, Serialize};

use super::functions::{build_plan, haversine_km, most_common_charger};

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
        let mut stations: Vec<ChargingStation> = Vec::new();
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
