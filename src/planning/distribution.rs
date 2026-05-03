//! Distribution system planning module.
//!
//! Provides comprehensive tools for distribution network expansion planning,
//! load forecasting, asset condition assessment, reliability-centred maintenance
//! (RCM), distributed-energy-resource (DER) integration analysis, and long-term
//! investment strategy formulation.
//!
//! # Units
//!
//! Unless otherwise noted:
//! - Power capacity: \[MVA\] or \[MW\]
//! - Energy: \[MWh\]
//! - Cost / NPV: \[$\]
//! - Distance: \[km\]
//! - Time: \[years\]
//! - Percentages: \[%\]

// ────────────────────────────────────────────────────────────────────────────
// 1. Distribution Network Expansion Planner
// ────────────────────────────────────────────────────────────────────────────

/// Classification of distribution-level capital projects.
#[derive(Debug, Clone, PartialEq)]
pub enum ProjectType {
    /// New overhead or underground feeder circuit.
    NewFeeder,
    /// Transformer or switchgear upgrade at an existing substation.
    SubstationUpgrade,
    /// Like-for-like cable replacement (ageing or overloaded asset).
    CableReplacement,
    /// SCADA / remote switching automation addition.
    AutomationAddition,
    /// Step-voltage regulator installation.
    VoltageRegulator,
    /// Fixed or switched capacitor bank.
    CapacitorBank,
    /// Point-of-common-coupling for an islanded microgrid.
    MicrogridConnection,
    /// EV fast-charging stations and associated grid reinforcement.
    EvChargingInfrastructure,
}

/// Monetisable and non-monetisable benefits attributed to a project.
#[derive(Debug, Clone)]
pub struct ProjectBenefits {
    /// Technical losses saved annually \[MWh/year\].
    pub loss_reduction_mwh_per_year: f64,
    /// SAIDI improvement (minutes of interruption avoided per customer-year).
    pub reliability_improvement_minutes: f64,
    /// Additional loadability released on the network \[MVA\].
    pub capacity_released_mva: f64,
    /// Net-present value of capital investment that the project defers \[$\].
    pub deferred_investment: f64,
    /// Annual CO₂ displacement \[tCO₂/year\].
    pub co2_reduction_tco2_per_year: f64,
}

/// A single candidate capital project in the distribution expansion portfolio.
#[derive(Debug, Clone)]
pub struct DistributionProject {
    /// Unique project identifier.
    pub id: usize,
    /// Human-readable project name.
    pub name: String,
    /// Category of the project.
    pub project_type: ProjectType,
    /// Overnight capital cost \[$\].
    pub capital_cost: f64,
    /// Annual operating expenditure once commissioned \[$/year\].
    pub annual_opex: f64,
    /// Rated capacity of the asset being installed \[MVA\].
    pub capacity_mva: f64,
    /// Total length of new or replaced conductors \[km\].
    pub feeder_length_km: f64,
    /// Number of years from decision to in-service date \[years\].
    pub construction_years: u32,
    /// Calendar year the project enters service.
    pub year_commissioned: u32,
    /// Design life from commissioning \[years\].
    pub expected_life_years: u32,
    /// Quantified benefit streams attributable to this project.
    pub benefits: ProjectBenefits,
}

/// Optimal expansion plan produced by [`DistributionExpansionPlanner`].
#[derive(Debug, Clone)]
pub struct ExpansionPlan {
    /// IDs of projects selected for implementation.
    pub selected_projects: Vec<usize>,
    /// Sum of capital costs for all selected projects \[$\].
    pub total_capex: f64,
    /// Aggregate NPV of all selected projects \[$\].
    pub total_npv: f64,
    /// Commissioning schedule: `(year, [project_id, …])`.
    pub annual_schedule: Vec<(u32, Vec<usize>)>,
}

/// Long-term distribution network expansion planner.
///
/// Evaluates candidate projects using NPV / benefit-cost ratio (BCR) and
/// assembles an annual investment programme subject to budget constraints.
#[derive(Debug, Clone)]
pub struct DistributionExpansionPlanner {
    /// First year of the planning study.
    pub base_year: u32,
    /// Length of the planning window \[years\].
    pub planning_horizon_years: u32,
    /// Weighted-average cost of capital / discount rate \[fraction, e.g. 0.07\].
    pub discount_rate: f64,
    /// Annual demand growth assumption \[%\].
    pub load_growth_rate_pct: f64,
    /// Full set of candidate projects under consideration.
    pub candidate_projects: Vec<DistributionProject>,
    /// Available capital budget for each year of the horizon \[$/year\].
    pub budget_per_year: Vec<f64>,
}

impl DistributionExpansionPlanner {
    /// Net-present value of a single project \[$\].
    ///
    /// `NPV = Σ_{t=0}^{life} (annual_benefits − opex) / (1+r)^t − capex`
    ///
    /// Annual benefits = loss reduction value (at $50/MWh proxy) +
    ///                   reliability value (at `voll` $/kWh × interruptions) +
    ///                   deferred-investment annuity.
    /// The VOLL is **not** used here — use [`Self::benefit_cost_ratio`] for
    /// that calculation.  Here a conservative $50/MWh loss value is assumed.
    pub fn npv(&self, project: &DistributionProject) -> f64 {
        let r = self.discount_rate;
        let loss_value_per_mwh = 50.0_f64; // $/MWh proxy
        let annual_benefit = project.benefits.loss_reduction_mwh_per_year * loss_value_per_mwh
            + project.benefits.deferred_investment / project.expected_life_years as f64;

        let net_annual = annual_benefit - project.annual_opex;
        let life = project.expected_life_years as f64;

        // Annuity factor: Σ_{t=1}^{life} 1/(1+r)^t
        let annuity = if r.abs() < 1e-12 {
            life
        } else {
            (1.0 - (1.0 + r).powf(-life)) / r
        };

        net_annual * annuity - project.capital_cost
    }

    /// Benefit-cost ratio of a project given a value-of-lost-load \[$/MWh\].
    ///
    /// Benefits include reliability improvement valued at `voll`, loss
    /// reduction, released-capacity value (at $30k/MVA-year proxy), and
    /// CO₂ reduction (at $30/tCO₂ proxy).
    pub fn benefit_cost_ratio(&self, project: &DistributionProject, voll: f64) -> f64 {
        let r = self.discount_rate;
        let loss_value_per_mwh = 50.0_f64;
        let co2_price_per_tco2 = 30.0_f64;
        let capacity_value_per_mva_year = 30_000.0_f64;

        let annual_benefit = project.benefits.loss_reduction_mwh_per_year * loss_value_per_mwh
            + project.benefits.reliability_improvement_minutes / 60.0 * voll
            + project.benefits.capacity_released_mva * capacity_value_per_mva_year
            + project.benefits.co2_reduction_tco2_per_year * co2_price_per_tco2
            + project.benefits.deferred_investment / project.expected_life_years as f64;

        let life = project.expected_life_years as f64;
        let annuity = if r.abs() < 1e-12 {
            life
        } else {
            (1.0 - (1.0 + r).powf(-life)) / r
        };

        let total_cost = project.capital_cost + project.annual_opex * annuity;
        if total_cost <= 0.0 {
            return 0.0;
        }
        annual_benefit * annuity / total_cost
    }

    /// Greedy expansion plan: rank all projects by BCR (VOLL = $100/MWh),
    /// then select in descending BCR order while respecting the annual budget.
    pub fn plan_greedy(&self) -> ExpansionPlan {
        let voll = 100_000.0_f64; // $100/MWh → $100 000/MWh for consistency
        let mut ranked: Vec<(usize, f64)> = self
            .candidate_projects
            .iter()
            .map(|p| (p.id, self.benefit_cost_ratio(p, voll)))
            .collect();
        ranked.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        // Per-year budget availability
        let horizon = self.planning_horizon_years as usize;
        let budget_available: Vec<f64> = (0..horizon)
            .map(|i| {
                self.budget_per_year
                    .get(i)
                    .copied()
                    .unwrap_or(f64::INFINITY)
            })
            .collect();
        let mut remaining_budget = budget_available.clone();

        let mut selected: Vec<usize> = Vec::new();
        let mut total_capex = 0.0_f64;
        let mut total_npv = 0.0_f64;
        let mut schedule: std::collections::HashMap<u32, Vec<usize>> =
            std::collections::HashMap::new();

        for (pid, _bcr) in &ranked {
            // find matching project
            let proj = match self.candidate_projects.iter().find(|p| p.id == *pid) {
                Some(p) => p,
                None => continue,
            };
            // budget index = commissioned year relative to base year
            let yr_idx = proj.year_commissioned.saturating_sub(self.base_year) as usize;
            if yr_idx >= horizon {
                continue;
            }
            if remaining_budget[yr_idx] >= proj.capital_cost {
                remaining_budget[yr_idx] -= proj.capital_cost;
                selected.push(proj.id);
                total_capex += proj.capital_cost;
                total_npv += self.npv(proj);
                schedule
                    .entry(proj.year_commissioned)
                    .or_default()
                    .push(proj.id);
            }
        }

        let mut annual_schedule: Vec<(u32, Vec<usize>)> = schedule.into_iter().collect();
        annual_schedule.sort_by_key(|(y, _)| *y);

        ExpansionPlan {
            selected_projects: selected,
            total_capex,
            total_npv,
            annual_schedule,
        }
    }

    /// Value of deferring a project by `deferral_years` \[$\].
    ///
    /// `Value = NPV_now − NPV_deferred`
    ///
    /// A positive value means the project should proceed now; negative means
    /// deferral is economically preferable.
    pub fn plan_deferred(&self, project: &DistributionProject, deferral_years: u32) -> f64 {
        let npv_now = self.npv(project);
        let r = self.discount_rate;
        let discount = (1.0 + r).powi(deferral_years as i32);
        let npv_deferred = self.npv(project) / discount;
        npv_now - npv_deferred
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 2. Load Forecast for Planning
// ────────────────────────────────────────────────────────────────────────────

/// A single demand-growth scenario for sensitivity / probabilistic planning.
#[derive(Debug, Clone)]
pub struct GrowthScenario {
    /// Human-readable scenario label (e.g. "Central", "High EV").
    pub name: String,
    /// Annual energy demand growth rate \[%\].
    pub annual_energy_growth_pct: f64,
    /// Annual peak-demand growth rate \[%\] (may exceed energy growth due to EVs).
    pub peak_growth_pct: f64,
    /// EV fleet penetration fraction for each year of the horizon \[0–1\].
    pub ev_penetration_by_year: Vec<f64>,
    /// Behind-the-meter PV penetration fraction for each year \[0–1\].
    pub pv_penetration_by_year: Vec<f64>,
    /// Demand-response participation rate \[%\].
    pub dr_participation_pct: f64,
}

/// Distribution-level load forecast engine.
#[derive(Debug, Clone)]
pub struct DistributionLoadForecast {
    /// Base-year annual energy demand \[MWh\].
    pub base_demand_mw: f64,
    /// Base-year coincident peak demand \[MW\].
    pub base_peak_mw: f64,
    /// Set of growth scenarios used for planning analysis.
    pub growth_scenarios: Vec<GrowthScenario>,
}

impl DistributionLoadForecast {
    /// Forecast coincident peak demand for scenario `scenario_idx` at
    /// `years_ahead` years in the future \[MW\].
    pub fn forecast_peak_mw(&self, scenario_idx: usize, years_ahead: u32) -> Result<f64, String> {
        let scenario = self
            .growth_scenarios
            .get(scenario_idx)
            .ok_or_else(|| format!("scenario index {} out of range", scenario_idx))?;
        let g = scenario.peak_growth_pct / 100.0;
        Ok(self.base_peak_mw * (1.0 + g).powi(years_ahead as i32))
    }

    /// Forecast annual energy throughput for scenario `scenario_idx` \[MWh\].
    pub fn forecast_energy_mwh(
        &self,
        scenario_idx: usize,
        years_ahead: u32,
    ) -> Result<f64, String> {
        let scenario = self
            .growth_scenarios
            .get(scenario_idx)
            .ok_or_else(|| format!("scenario index {} out of range", scenario_idx))?;
        let g = scenario.annual_energy_growth_pct / 100.0;
        // Convert base MW to MWh (8760 h/year) then grow
        Ok(self.base_demand_mw * 8_760.0 * (1.0 + g).powi(years_ahead as i32))
    }

    /// Net peak after crediting behind-the-meter PV and demand-response \[MW\].
    ///
    /// `net_peak = gross_peak − PV_credit − DR_credit`
    ///
    /// PV credit assumes 80 \[%\] coincidence factor at the peak hour.
    /// DR credit is computed from the participation rate applied to gross peak.
    pub fn net_peak_with_der(&self, scenario_idx: usize, years_ahead: u32) -> Result<f64, String> {
        let scenario = self
            .growth_scenarios
            .get(scenario_idx)
            .ok_or_else(|| format!("scenario index {} out of range", scenario_idx))?;

        let gross_peak = self.forecast_peak_mw(scenario_idx, years_ahead)?;

        let pv_penetration = scenario
            .pv_penetration_by_year
            .get(years_ahead as usize)
            .copied()
            .unwrap_or_else(|| {
                scenario
                    .pv_penetration_by_year
                    .last()
                    .copied()
                    .unwrap_or(0.0)
            });

        let ev_penetration = scenario
            .ev_penetration_by_year
            .get(years_ahead as usize)
            .copied()
            .unwrap_or_else(|| {
                scenario
                    .ev_penetration_by_year
                    .last()
                    .copied()
                    .unwrap_or(0.0)
            });

        // PV reduces peak with 0.8 coincidence; EV adds to peak at 0.3 CF
        let pv_credit = gross_peak * pv_penetration * 0.8;
        let ev_addition = gross_peak * ev_penetration * 0.3;
        let dr_credit = gross_peak * scenario.dr_participation_pct / 100.0 * 0.5;

        let net = (gross_peak - pv_credit + ev_addition - dr_credit).max(0.0);
        Ok(net)
    }

    /// Compound load-growth factor for scenario `scenario_idx` over `years` \[dimensionless\].
    pub fn load_growth_factor(&self, scenario_idx: usize, years: u32) -> Result<f64, String> {
        let scenario = self
            .growth_scenarios
            .get(scenario_idx)
            .ok_or_else(|| format!("scenario index {} out of range", scenario_idx))?;
        let g = scenario.peak_growth_pct / 100.0;
        Ok((1.0 + g).powi(years as i32))
    }

    /// Conservative planning peak: maximum forecast peak across all scenarios
    /// at the specified confidence level (equal-weight default) \[MW\].
    ///
    /// With equal weighting, `confidence_pct = 100` returns the maximum
    /// scenario peak; lower values return a percentile across scenarios.
    pub fn planning_peak(&self, confidence_pct: f64) -> f64 {
        if self.growth_scenarios.is_empty() {
            return self.base_peak_mw;
        }
        let horizon = self
            .growth_scenarios
            .iter()
            .map(|s| {
                s.ev_penetration_by_year
                    .len()
                    .max(s.pv_penetration_by_year.len())
            })
            .max()
            .unwrap_or(1) as u32;

        let mut peaks: Vec<f64> = self
            .growth_scenarios
            .iter()
            .enumerate()
            .filter_map(|(i, _)| self.forecast_peak_mw(i, horizon).ok())
            .collect();
        peaks.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        if peaks.is_empty() {
            return self.base_peak_mw;
        }
        // percentile index
        let idx = ((confidence_pct / 100.0) * peaks.len() as f64 - 1.0)
            .max(0.0)
            .min((peaks.len() - 1) as f64) as usize;
        peaks[idx]
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 3. Asset Condition Assessment
// ────────────────────────────────────────────────────────────────────────────

/// Class of distribution asset.
#[derive(Debug, Clone, PartialEq)]
pub enum DistAssetType {
    /// Power transformer (distribution or zone substation).
    Transformer,
    /// Underground cable or overhead conductor.
    Cable,
    /// Circuit-breaker, disconnector, or ring-main unit.
    Switchgear,
    /// Electricity meter (AMI or legacy).
    Meter,
    /// Protection relay (electromechanical, solid-state, or numerical).
    Relay,
    /// Fixed or switched capacitor bank.
    CapBank,
}

/// Record for a single distribution asset.
#[derive(Debug, Clone)]
pub struct DistributionAsset {
    /// Unique asset identifier.
    pub id: usize,
    /// Asset category.
    pub asset_type: DistAssetType,
    /// Calendar year the asset was commissioned.
    pub installation_year: u32,
    /// Original design life from commissioning \[years\].
    pub expected_life_years: u32,
    /// Composite condition score from inspections/OILIM/DGA \[0–100\].
    pub condition_score: f64,
    /// Annualised probability of failure \[failures/year\].
    pub failure_rate_per_year: f64,
    /// Estimated current-day replacement cost \[$\].
    pub replacement_cost: f64,
    /// Fraction of system load served through this asset \[0–1\].
    pub criticality: f64,
}

/// Portfolio-level condition assessment and prioritisation engine.
#[derive(Debug, Clone)]
pub struct AssetConditionAssessor {
    /// All distribution assets under management.
    pub assets: Vec<DistributionAsset>,
}

impl AssetConditionAssessor {
    /// Age of an asset at `current_year` \[years\].
    pub fn age_years(&self, asset: &DistributionAsset, current_year: u32) -> u32 {
        current_year.saturating_sub(asset.installation_year)
    }

    /// Remaining life fraction relative to design life \[0–1\].
    ///
    /// Clipped to `[0, 1]`; assets beyond their design life return 0.
    pub fn remaining_life_pct(&self, asset: &DistributionAsset, current_year: u32) -> f64 {
        let age = self.age_years(asset, current_year) as f64;
        let life = asset.expected_life_years as f64;
        if life <= 0.0 {
            return 0.0;
        }
        ((life - age) / life).clamp(0.0, 1.0)
    }

    /// Composite health index \[0–100\].
    ///
    /// `HI = condition_score × (1 − failure_rate) × remaining_life_pct × 100`
    ///
    /// Clamped to \[0, 100\].
    pub fn health_index(&self, asset: &DistributionAsset, current_year: u32) -> f64 {
        let rl = self.remaining_life_pct(asset, current_year);
        let hi = asset.condition_score * (1.0 - asset.failure_rate_per_year).max(0.0) * rl;
        hi.clamp(0.0, 100.0)
    }

    /// Replacement priority score (higher → more urgent).
    ///
    /// `Priority = criticality × failure_rate / max(HI, ε)`
    pub fn replacement_priority(&self, asset: &DistributionAsset, current_year: u32) -> f64 {
        let hi = self.health_index(asset, current_year).max(1e-6);
        asset.criticality * asset.failure_rate_per_year / hi
    }

    /// Indices into `self.assets` for the `top_n` highest-priority replacements.
    pub fn top_priority_replacements(&self, current_year: u32, top_n: usize) -> Vec<usize> {
        let mut scored: Vec<(usize, f64)> = self
            .assets
            .iter()
            .enumerate()
            .map(|(i, a)| (i, self.replacement_priority(a, current_year)))
            .collect();
        scored.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        scored.truncate(top_n);
        scored.into_iter().map(|(i, _)| i).collect()
    }

    /// Total replacement cost for assets whose remaining life expires within
    /// `years` from `current_year` \[$\].
    pub fn total_replacement_cost_within_horizon(&self, years: u32, current_year: u32) -> f64 {
        self.assets
            .iter()
            .filter(|a| {
                let remaining = self.remaining_life_pct(a, current_year);
                let remaining_years = (remaining * a.expected_life_years as f64).ceil() as u32;
                remaining_years <= years
            })
            .map(|a| a.replacement_cost)
            .sum()
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 4. Reliability-Centred Maintenance
// ────────────────────────────────────────────────────────────────────────────

/// A scheduled maintenance activity for a specific asset.
#[derive(Debug, Clone)]
pub struct MaintenanceActivity {
    /// Asset identifier.
    pub asset_id: usize,
    /// Descriptive maintenance activity label.
    pub activity_type: String,
    /// Optimal inspection / servicing interval \[years\].
    pub interval_years: f64,
    /// Annualised cost of carrying out this activity \[$/year\].
    pub annual_cost: f64,
}

/// Optimised maintenance programme produced by [`RcmAnalyzer`].
#[derive(Debug, Clone)]
pub struct MaintenancePlan {
    /// Asset IDs included in the maintenance programme.
    pub selected_assets: Vec<usize>,
    /// Total annualised maintenance expenditure \[$/year\].
    pub total_cost: f64,
    /// Risk reduction relative to do-nothing baseline \[%\].
    pub risk_reduction_pct: f64,
    /// Individual maintenance tasks scheduled.
    pub activities: Vec<MaintenanceActivity>,
}

/// Reliability-centred maintenance analyser for distribution assets.
#[derive(Debug, Clone)]
pub struct RcmAnalyzer {
    /// Asset fleet under study.
    pub assets: Vec<DistributionAsset>,
    /// Available annual maintenance budget \[$/year\].
    pub maintenance_budget: f64,
    /// Consequence cost per failure event (repair + outage penalty) \[$\].
    pub failure_consequence: f64,
}

impl RcmAnalyzer {
    /// Total expected failures per year across the entire asset fleet.
    pub fn expected_annual_failures(&self) -> f64 {
        self.assets.iter().map(|a| a.failure_rate_per_year).sum()
    }

    /// Total expected failure cost per year \[$/year\].
    pub fn expected_annual_failure_cost(&self) -> f64 {
        self.assets
            .iter()
            .map(|a| a.failure_rate_per_year * self.failure_consequence)
            .sum()
    }

    /// Optimal preventive maintenance interval for an asset \[years\].
    ///
    /// Derived from the classical age-replacement model:
    /// `T* = sqrt(2 × Cm / (λ × Cf))`
    ///
    /// where `Cm` is the maintenance cost (proxy: 1 \[%\] of replacement cost),
    /// `λ` is the failure rate, and `Cf` is the failure consequence.
    pub fn preventive_maintenance_interval_years(&self, asset: &DistributionAsset) -> f64 {
        let maintenance_cost = asset.replacement_cost * 0.01; // 1% of CAPEX
        let denom = asset.failure_rate_per_year * self.failure_consequence;
        if denom <= 0.0 {
            return f64::INFINITY;
        }
        (2.0 * maintenance_cost / denom).sqrt()
    }

    /// Expected risk reduction from performing maintenance on a single asset \[$/year\].
    ///
    /// Preventive maintenance is assumed to halve the failure rate.
    pub fn risk_reduction_from_maintenance(&self, asset: &DistributionAsset) -> f64 {
        asset.failure_rate_per_year * 0.5 * self.failure_consequence
    }

    /// Maintenance plan tuples: `(asset_id, interval_years, annual_cost)`.
    pub fn maintenance_plan(&self) -> Vec<(usize, f64, f64)> {
        self.assets
            .iter()
            .map(|a| {
                let interval = self.preventive_maintenance_interval_years(a);
                let annual_cost = if interval.is_finite() && interval > 0.0 {
                    a.replacement_cost * 0.01 / interval
                } else {
                    0.0
                };
                (a.id, interval, annual_cost)
            })
            .collect()
    }

    /// Budget-constrained maintenance optimisation.
    ///
    /// Assets are ranked by risk-reduction-per-dollar, and selected greedily
    /// until the annual budget is exhausted.
    pub fn optimize_maintenance_budget(&self) -> MaintenancePlan {
        let plan_raw = self.maintenance_plan();
        // (asset_idx, risk_reduction, annual_cost, interval)
        let mut candidates: Vec<(usize, f64, f64, f64)> = self
            .assets
            .iter()
            .enumerate()
            .filter_map(|(i, a)| {
                let (_, interval, annual_cost) = plan_raw.get(i)?;
                let rr = self.risk_reduction_from_maintenance(a);
                if *annual_cost <= 0.0 {
                    return None;
                }
                Some((i, rr, *annual_cost, *interval))
            })
            .collect();

        // rank by risk_reduction / annual_cost (descending)
        candidates.sort_by(|a, b| {
            let ra = a.1 / a.2;
            let rb = b.1 / b.2;
            rb.partial_cmp(&ra).unwrap_or(std::cmp::Ordering::Equal)
        });

        let baseline_risk = self.expected_annual_failure_cost();
        let mut remaining_budget = self.maintenance_budget;
        let mut selected_assets: Vec<usize> = Vec::new();
        let mut total_cost = 0.0_f64;
        let mut total_rr = 0.0_f64;
        let mut activities: Vec<MaintenanceActivity> = Vec::new();

        for (i, rr, annual_cost, interval) in &candidates {
            if remaining_budget < *annual_cost {
                continue;
            }
            remaining_budget -= annual_cost;
            total_cost += annual_cost;
            total_rr += rr;
            selected_assets.push(self.assets[*i].id);
            activities.push(MaintenanceActivity {
                asset_id: self.assets[*i].id,
                activity_type: format!("{:?} preventive maintenance", self.assets[*i].asset_type),
                interval_years: *interval,
                annual_cost: *annual_cost,
            });
        }

        let risk_reduction_pct = if baseline_risk > 0.0 {
            (total_rr / baseline_risk * 100.0).min(100.0)
        } else {
            0.0
        };

        MaintenancePlan {
            selected_assets,
            total_cost,
            risk_reduction_pct,
            activities,
        }
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 5. DER Integration Planning
// ────────────────────────────────────────────────────────────────────────────

/// Technology category for a DER connection candidate.
#[derive(Debug, Clone, PartialEq)]
pub enum DerCandidateType {
    /// Photovoltaic generation.
    Pv,
    /// Wind generation (onshore small/medium).
    Wind,
    /// Battery energy-storage system.
    Battery,
    /// Electric vehicle (aggregated fleet or hub).
    Ev,
    /// Hydrogen fuel cell.
    FuelCell,
    /// Combined heat and power unit.
    CombinedHeatPower,
}

/// A DER project applying for grid connection.
#[derive(Debug, Clone)]
pub struct DerCandidate {
    /// Unique candidate identifier.
    pub id: usize,
    /// DER technology type.
    pub der_type: DerCandidateType,
    /// Installed capacity \[MW\].
    pub capacity_mw: f64,
    /// Network bus at which the DER will be connected.
    pub location_bus: usize,
    /// One-time grid connection cost paid by the developer \[$\].
    pub connection_cost: f64,
    /// Expected annual energy generation (or displacement) \[MWh/year\].
    pub annual_generation_mwh: f64,
    /// Risk that the DER will be curtailed due to network constraints \[%\].
    pub curtailment_risk_pct: f64,
}

/// DER connection and hosting-capacity assessment planner.
#[derive(Debug, Clone)]
pub struct DerIntegrationPlanner {
    /// Thermal rating of the distribution feeder \[MVA\].
    pub feeder_capacity_mva: f64,
    /// Current coincident peak load on the feeder \[MW\].
    pub existing_load_mw: f64,
    /// Total DER already connected and active \[MW\].
    pub existing_der_mw: f64,
    /// Maximum additional DER supportable without network upgrade \[MW\].
    pub hosting_capacity_mw: f64,
    /// Queue of DER connection candidates.
    pub der_candidates: Vec<DerCandidate>,
}

impl DerIntegrationPlanner {
    /// Remaining hosting capacity available for new DER connections \[MW\].
    pub fn available_hosting_mw(&self) -> f64 {
        (self.hosting_capacity_mw - self.existing_der_mw).max(0.0)
    }

    /// Ranked list of `(candidate_id, score)` in descending priority order.
    ///
    /// `score = annual_generation_mwh / connection_cost × (1 − curtailment_risk / 100)`
    pub fn prioritize_candidates(&self) -> Vec<(usize, f64)> {
        let mut scored: Vec<(usize, f64)> = self
            .der_candidates
            .iter()
            .map(|c| {
                let score = if c.connection_cost <= 0.0 {
                    0.0
                } else {
                    c.annual_generation_mwh / c.connection_cost
                        * (1.0 - c.curtailment_risk_pct / 100.0)
                };
                (c.id, score)
            })
            .collect();
        scored.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        scored
    }

    /// Returns `true` if the candidate fits within the available hosting capacity.
    pub fn can_integrate(&self, candidate: &DerCandidate) -> bool {
        candidate.capacity_mw <= self.available_hosting_mw()
    }

    /// Estimated network upgrade cost to accommodate `additional_mw` \[$\].
    ///
    /// Rule-of-thumb: $50 000/MVA for LV/MV distribution reinforcement.
    pub fn network_upgrade_cost(&self, additional_mw: f64) -> f64 {
        let upgrade_needed = (additional_mw - self.available_hosting_mw()).max(0.0);
        upgrade_needed * 50_000.0
    }

    /// Simplified NPV of the benefit stream from a DER candidate \[$\].
    ///
    /// `NPV = annual_gen × price / r − connection_cost − upgrade_cost`
    ///
    /// A 7 \[%\] discount rate is used internally.
    pub fn der_benefit_analysis(
        &self,
        candidate: &DerCandidate,
        electricity_price_mwh: f64,
    ) -> f64 {
        let r = 0.07_f64;
        let annual_revenue = candidate.annual_generation_mwh
            * electricity_price_mwh
            * (1.0 - candidate.curtailment_risk_pct / 100.0);
        let perpetuity_value = annual_revenue / r;
        let upgrade = self.network_upgrade_cost(candidate.capacity_mw);
        perpetuity_value - candidate.connection_cost - upgrade
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 6. Long-Term Investment Strategy
// ────────────────────────────────────────────────────────────────────────────

/// Summary KPIs for the long-term investment strategy.
#[derive(Debug, Clone)]
pub struct StrategyMetrics {
    /// Total capacity added by all selected projects \[MVA\].
    pub total_capacity_added_mva: f64,
    /// Total SAIDI improvement across all selected projects \[min/customer-year\].
    pub total_reliability_improvement_minutes: f64,
    /// Cumulative CO₂ displacement over the planning horizon \[tCO₂\].
    pub total_co2_reduction_tco2: f64,
    /// Aggregate NPV of selected projects \[$\].
    pub overall_npv: f64,
    /// Budget consumed as a fraction of the total available budget \[%\].
    pub budget_utilization_pct: f64,
}

/// Long-term distribution investment strategy.
///
/// Integrates expansion planning, asset management, and strategic objectives
/// into a coherent multi-year capital programme.
#[derive(Debug, Clone)]
pub struct LongTermStrategy {
    /// Planning window length \[years\].
    pub planning_horizon_years: u32,
    /// Total capital budget over the planning horizon \[$\].
    pub total_budget: f64,
    /// High-level strategic objectives (narrative labels).
    pub strategic_objectives: Vec<String>,
    /// All candidate projects (subset of or equal to those in the planner).
    pub projects: Vec<DistributionProject>,
    /// Embedded expansion planner (provides NPV and BCR methods).
    pub expansion_planner: DistributionExpansionPlanner,
}

impl LongTermStrategy {
    /// Construct a new strategy with sensible defaults.
    ///
    /// Sets `discount_rate = 0.07`, `load_growth_rate_pct = 1.5`, and an
    /// equal budget allocation across the horizon.
    pub fn new(horizon: u32, budget: f64) -> Self {
        let budget_per_year = if horizon > 0 {
            vec![budget / horizon as f64; horizon as usize]
        } else {
            vec![]
        };
        LongTermStrategy {
            planning_horizon_years: horizon,
            total_budget: budget,
            strategic_objectives: Vec::new(),
            projects: Vec::new(),
            expansion_planner: DistributionExpansionPlanner {
                base_year: 2025,
                planning_horizon_years: horizon,
                discount_rate: 0.07,
                load_growth_rate_pct: 1.5,
                candidate_projects: Vec::new(),
                budget_per_year,
            },
        }
    }

    /// Total capital cost of all candidate projects in the strategy \[$\].
    pub fn total_capex_requirements(&self) -> f64 {
        self.projects.iter().map(|p| p.capital_cost).sum()
    }

    /// Shortfall between capital requirements and available budget \[$\].
    ///
    /// A positive value indicates unfunded requirements.
    pub fn capex_gap(&self) -> f64 {
        (self.total_capex_requirements() - self.total_budget).max(0.0)
    }

    /// Prioritised investment schedule: `(year, project_name, capital_cost)`.
    ///
    /// Uses the embedded greedy planner; projects not selected are excluded.
    pub fn prioritized_investment_plan(&self) -> Vec<(u32, String, f64)> {
        // Temporarily mirror projects into the planner
        let mut planner = self.expansion_planner.clone();
        planner.candidate_projects = self.projects.clone();
        let plan = planner.plan_greedy();

        let id_to_project: std::collections::HashMap<usize, &DistributionProject> =
            self.projects.iter().map(|p| (p.id, p)).collect();

        let mut result: Vec<(u32, String, f64)> = Vec::new();
        for (year, ids) in &plan.annual_schedule {
            for pid in ids {
                if let Some(proj) = id_to_project.get(pid) {
                    result.push((*year, proj.name.clone(), proj.capital_cost));
                }
            }
        }
        result.sort_by_key(|(y, _, _)| *y);
        result
    }

    /// Aggregate KPIs for the recommended investment strategy.
    pub fn strategy_metrics(&self) -> StrategyMetrics {
        let mut planner = self.expansion_planner.clone();
        planner.candidate_projects = self.projects.clone();
        let plan = planner.plan_greedy();

        let id_to_project: std::collections::HashMap<usize, &DistributionProject> =
            self.projects.iter().map(|p| (p.id, p)).collect();

        let mut total_capacity_added_mva = 0.0_f64;
        let mut total_reliability_improvement_minutes = 0.0_f64;
        let mut total_co2_reduction_tco2 = 0.0_f64;
        let mut total_capex_selected = 0.0_f64;

        for pid in &plan.selected_projects {
            if let Some(proj) = id_to_project.get(pid) {
                total_capacity_added_mva += proj.capacity_mva;
                total_reliability_improvement_minutes +=
                    proj.benefits.reliability_improvement_minutes
                        * self.planning_horizon_years as f64;
                total_co2_reduction_tco2 +=
                    proj.benefits.co2_reduction_tco2_per_year * self.planning_horizon_years as f64;
                total_capex_selected += proj.capital_cost;
            }
        }

        let budget_utilization_pct = if self.total_budget > 0.0 {
            (total_capex_selected / self.total_budget * 100.0).min(100.0)
        } else {
            0.0
        };

        StrategyMetrics {
            total_capacity_added_mva,
            total_reliability_improvement_minutes,
            total_co2_reduction_tco2,
            overall_npv: plan.total_npv,
            budget_utilization_pct,
        }
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Tests
// ────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_project(id: usize, capex: f64, year: u32) -> DistributionProject {
        DistributionProject {
            id,
            name: format!("Project-{}", id),
            project_type: ProjectType::NewFeeder,
            capital_cost: capex,
            annual_opex: 1_000.0,
            capacity_mva: 5.0,
            feeder_length_km: 2.0,
            construction_years: 1,
            year_commissioned: year,
            expected_life_years: 30,
            benefits: ProjectBenefits {
                loss_reduction_mwh_per_year: 500.0,
                reliability_improvement_minutes: 60.0,
                capacity_released_mva: 2.0,
                deferred_investment: 50_000.0,
                co2_reduction_tco2_per_year: 10.0,
            },
        }
    }

    fn sample_planner() -> DistributionExpansionPlanner {
        DistributionExpansionPlanner {
            base_year: 2025,
            planning_horizon_years: 10,
            discount_rate: 0.07,
            load_growth_rate_pct: 1.5,
            candidate_projects: vec![
                sample_project(1, 100_000.0, 2025),
                sample_project(2, 200_000.0, 2026),
                sample_project(3, 50_000.0, 2027),
            ],
            budget_per_year: vec![150_000.0; 10],
        }
    }

    // ── ExpansionPlanner tests ──────────────────────────────────────────────

    #[test]
    fn test_npv_decreases_with_higher_discount_rate() {
        let proj = sample_project(1, 100_000.0, 2025);
        let mut planner = sample_planner();
        planner.candidate_projects = vec![proj.clone()];

        planner.discount_rate = 0.03;
        let npv_low = planner.npv(&proj);
        planner.discount_rate = 0.12;
        let npv_high = planner.npv(&proj);

        assert!(
            npv_low > npv_high,
            "NPV should decrease as discount rate increases: low={npv_low:.2} high={npv_high:.2}"
        );
    }

    #[test]
    fn test_plan_greedy_selects_highest_bcr_within_budget() {
        // Project 3 has lowest capex (50k) → highest BCR → must be selected
        let planner = sample_planner();
        let plan = planner.plan_greedy();
        assert!(
            plan.selected_projects.contains(&3),
            "Greedy planner should select the cheapest high-BCR project (id=3)"
        );
        assert!(
            plan.total_capex <= planner.budget_per_year[0] * planner.planning_horizon_years as f64,
            "Total CAPEX must not exceed total horizon budget"
        );
    }

    #[test]
    fn test_plan_greedy_respects_budget() {
        let mut planner = sample_planner();
        // Very tight budget — only room for project 3 (50k) each year
        planner.budget_per_year = vec![60_000.0; 10];
        let plan = planner.plan_greedy();
        // Projects 1 (100k) and 2 (200k) both exceed single-year budget
        assert!(
            !plan.selected_projects.contains(&1),
            "Project 1 should not be selected with tight budget"
        );
        assert!(
            !plan.selected_projects.contains(&2),
            "Project 2 should not be selected with tight budget"
        );
    }

    #[test]
    fn test_deferred_value_positive_for_positive_npv_project() {
        let planner = sample_planner();
        let proj = sample_project(99, 10_000.0, 2025); // low capex → positive NPV
        let npv = planner.npv(&proj);
        let deferred_val = planner.plan_deferred(&proj, 5);
        if npv > 0.0 {
            assert!(
                deferred_val > 0.0,
                "Deferral value should be positive for a positive-NPV project: npv={npv:.2}"
            );
        }
    }

    #[test]
    fn test_bcr_positive() {
        let planner = sample_planner();
        let proj = sample_project(1, 100_000.0, 2025);
        let bcr = planner.benefit_cost_ratio(&proj, 100_000.0);
        assert!(bcr >= 0.0, "BCR must be non-negative");
    }

    // ── LoadForecast tests ─────────────────────────────────────────────────

    fn sample_forecast() -> DistributionLoadForecast {
        let scenario = GrowthScenario {
            name: "Central".to_string(),
            annual_energy_growth_pct: 2.0,
            peak_growth_pct: 2.5,
            ev_penetration_by_year: vec![0.0, 0.02, 0.05, 0.08, 0.12],
            pv_penetration_by_year: vec![0.05, 0.08, 0.10, 0.12, 0.15],
            dr_participation_pct: 5.0,
        };
        DistributionLoadForecast {
            base_demand_mw: 50.0,
            base_peak_mw: 70.0,
            growth_scenarios: vec![scenario],
        }
    }

    #[test]
    fn test_forecast_peak_increases_with_growth() {
        let fc = sample_forecast();
        let peak_0 = fc.forecast_peak_mw(0, 0).expect("forecast 0");
        let peak_10 = fc.forecast_peak_mw(0, 10).expect("forecast 10");
        assert!(peak_10 > peak_0, "Peak must grow over time");
    }

    #[test]
    fn test_net_peak_less_than_gross_peak() {
        // Scenario with high PV and significant DR so net < gross
        let scenario = GrowthScenario {
            name: "High PV".to_string(),
            annual_energy_growth_pct: 1.0,
            peak_growth_pct: 1.0,
            ev_penetration_by_year: vec![0.0],
            pv_penetration_by_year: vec![0.5],
            dr_participation_pct: 20.0,
        };
        let fc = DistributionLoadForecast {
            base_demand_mw: 50.0,
            base_peak_mw: 70.0,
            growth_scenarios: vec![scenario],
        };
        let gross = fc.forecast_peak_mw(0, 0).expect("gross peak");
        let net = fc.net_peak_with_der(0, 0).expect("net peak");
        assert!(
            net < gross,
            "Net peak [MW] should be less than gross peak due to PV and DR: gross={gross:.2} net={net:.2}"
        );
    }

    #[test]
    fn test_planning_peak_ge_base_demand() {
        let fc = sample_forecast();
        let pp = fc.planning_peak(50.0);
        assert!(
            pp >= fc.base_peak_mw,
            "Planning peak [MW] must be ≥ base peak [MW]"
        );
    }

    #[test]
    fn test_load_growth_factor_gt_one() {
        let fc = sample_forecast();
        let gf = fc.load_growth_factor(0, 5).expect("growth factor");
        assert!(gf > 1.0, "Growth factor over 5 years must exceed 1.0");
    }

    // ── AssetCondition tests ───────────────────────────────────────────────

    fn sample_asset(id: usize, install_year: u32, life: u32) -> DistributionAsset {
        DistributionAsset {
            id,
            asset_type: DistAssetType::Transformer,
            installation_year: install_year,
            expected_life_years: life,
            condition_score: 70.0,
            failure_rate_per_year: 0.05,
            replacement_cost: 200_000.0,
            criticality: 0.8,
        }
    }

    #[test]
    fn test_remaining_life_decreases_with_age() {
        let assessor = AssetConditionAssessor {
            assets: vec![sample_asset(1, 2000, 40)],
        };
        let a = &assessor.assets[0];
        let rl_2010 = assessor.remaining_life_pct(a, 2010);
        let rl_2020 = assessor.remaining_life_pct(a, 2020);
        assert!(
            rl_2010 > rl_2020,
            "Remaining life [%] must decrease over time: 2010={rl_2010:.3} 2020={rl_2020:.3}"
        );
    }

    #[test]
    fn test_health_index_in_range() {
        let assessor = AssetConditionAssessor {
            assets: vec![sample_asset(1, 2010, 40)],
        };
        let a = &assessor.assets[0];
        let hi = assessor.health_index(a, 2025);
        assert!(
            (0.0..=100.0).contains(&hi),
            "Health index must be in [0, 100]: got {hi:.2}"
        );
    }

    #[test]
    fn test_top_priority_selects_high_failure_low_health() {
        let mut a1 = sample_asset(1, 2000, 40); // old, high failure
        a1.failure_rate_per_year = 0.3;
        let mut a2 = sample_asset(2, 2020, 40); // new, low failure
        a2.failure_rate_per_year = 0.01;
        let assessor = AssetConditionAssessor {
            assets: vec![a1, a2],
        };
        let top = assessor.top_priority_replacements(2025, 1);
        assert_eq!(top.len(), 1);
        // asset index 0 (id=1) should be top priority
        assert_eq!(
            top[0], 0,
            "High failure-rate old asset must be top priority"
        );
    }

    // ── RcmAnalyzer tests ──────────────────────────────────────────────────

    #[test]
    fn test_preventive_maintenance_interval_positive() {
        let asset = sample_asset(1, 2000, 40);
        let rcm = RcmAnalyzer {
            assets: vec![asset.clone()],
            maintenance_budget: 20_000.0,
            failure_consequence: 500_000.0,
        };
        let interval = rcm.preventive_maintenance_interval_years(&asset);
        assert!(
            interval > 0.0,
            "Maintenance interval [years] must be positive: {interval:.3}"
        );
    }

    #[test]
    fn test_rcm_expected_failures_positive() {
        let rcm = RcmAnalyzer {
            assets: vec![sample_asset(1, 2000, 40), sample_asset(2, 2010, 40)],
            maintenance_budget: 30_000.0,
            failure_consequence: 200_000.0,
        };
        assert!(rcm.expected_annual_failures() > 0.0);
        assert!(rcm.expected_annual_failure_cost() > 0.0);
    }

    // ── DerIntegrationPlanner tests ────────────────────────────────────────

    fn sample_der_planner() -> DerIntegrationPlanner {
        DerIntegrationPlanner {
            feeder_capacity_mva: 20.0,
            existing_load_mw: 10.0,
            existing_der_mw: 3.0,
            hosting_capacity_mw: 8.0,
            der_candidates: vec![
                DerCandidate {
                    id: 1,
                    der_type: DerCandidateType::Pv,
                    capacity_mw: 4.0,
                    location_bus: 5,
                    connection_cost: 80_000.0,
                    annual_generation_mwh: 5_000.0,
                    curtailment_risk_pct: 5.0,
                },
                DerCandidate {
                    id: 2,
                    der_type: DerCandidateType::Wind,
                    capacity_mw: 10.0,
                    location_bus: 7,
                    connection_cost: 200_000.0,
                    annual_generation_mwh: 20_000.0,
                    curtailment_risk_pct: 15.0,
                },
            ],
        }
    }

    #[test]
    fn test_available_hosting_equals_capacity_minus_existing() {
        let planner = sample_der_planner();
        let expected = planner.hosting_capacity_mw - planner.existing_der_mw;
        let actual = planner.available_hosting_mw();
        assert!(
            (actual - expected).abs() < 1e-9,
            "Available hosting [MW] mismatch: expected {expected:.3} got {actual:.3}"
        );
    }

    #[test]
    fn test_can_integrate_false_when_exceeds_hosting() {
        let planner = sample_der_planner();
        // candidate id=2 capacity=10 MW > available_hosting=5 MW → cannot integrate
        let c2 = planner.der_candidates[1].clone();
        assert!(
            !planner.can_integrate(&c2),
            "DER candidate exceeding hosting capacity must not be integrable"
        );
    }

    #[test]
    fn test_can_integrate_true_when_within_hosting() {
        let planner = sample_der_planner();
        let c1 = planner.der_candidates[0].clone(); // 4 MW ≤ 5 MW available
        assert!(
            planner.can_integrate(&c1),
            "DER candidate within hosting capacity must be integrable"
        );
    }

    // ── LongTermStrategy tests ─────────────────────────────────────────────

    #[test]
    fn test_strategy_budget_utilization_le_100() {
        let mut strategy = LongTermStrategy::new(10, 500_000.0);
        strategy.projects = vec![
            sample_project(1, 100_000.0, 2025),
            sample_project(2, 200_000.0, 2026),
            sample_project(3, 50_000.0, 2027),
        ];
        // Mirror projects into planner
        strategy.expansion_planner.candidate_projects = strategy.projects.clone();
        let metrics = strategy.strategy_metrics();
        assert!(
            metrics.budget_utilization_pct <= 100.0,
            "Budget utilization [%] must not exceed 100: got {:.2}",
            metrics.budget_utilization_pct
        );
    }

    #[test]
    fn test_capex_gap_non_negative() {
        let mut strategy = LongTermStrategy::new(10, 100_000.0);
        strategy.projects = vec![sample_project(1, 500_000.0, 2025)];
        let gap = strategy.capex_gap();
        assert!(gap >= 0.0, "CAPEX gap [$] must be non-negative");
    }

    #[test]
    fn test_strategy_new_defaults() {
        let s = LongTermStrategy::new(5, 1_000_000.0);
        assert_eq!(s.planning_horizon_years, 5);
        assert!((s.total_budget - 1_000_000.0).abs() < 1e-6);
        assert_eq!(s.expansion_planner.budget_per_year.len(), 5);
    }

    // ── 8 new tests ───────────────────────────────────────────────────────────

    #[test]
    fn test_forecast_energy_mwh_grows_with_years() {
        let fc = sample_forecast();
        let e5 = fc.forecast_energy_mwh(0, 5).expect("forecast_energy_mwh");
        let e10 = fc.forecast_energy_mwh(0, 10).expect("forecast_energy_mwh");
        assert!(
            e10 > e5,
            "energy forecast at year 10 ({e10:.1}) should exceed year 5 ({e5:.1})"
        );
    }

    #[test]
    fn test_age_years_reflects_installation_year() {
        let asset = sample_asset(1, 2000, 40);
        let assessor = AssetConditionAssessor {
            assets: vec![asset.clone()],
        };
        assert_eq!(
            assessor.age_years(&asset, 2025),
            25,
            "age should be 25 in 2025"
        );
        assert_eq!(
            assessor.age_years(&asset, 2000),
            0,
            "age should be 0 in install year"
        );
    }

    #[test]
    fn test_total_replacement_cost_within_horizon_includes_near_end_assets() {
        // Asset installed 1985, life 40 years → expired by 2025, remaining_life ≈ 0.
        let old_asset = DistributionAsset {
            id: 1,
            asset_type: DistAssetType::Transformer,
            installation_year: 1985,
            expected_life_years: 40,
            condition_score: 50.0,
            failure_rate_per_year: 0.10,
            replacement_cost: 300_000.0,
            criticality: 0.9,
        };
        let assessor = AssetConditionAssessor {
            assets: vec![old_asset],
        };
        // within 5 years from 2025 → should include the expired asset
        let cost = assessor.total_replacement_cost_within_horizon(5, 2025);
        assert!(
            (cost - 300_000.0).abs() < 1e-6,
            "expired asset should be included, got {cost}"
        );
    }

    #[test]
    fn test_risk_reduction_from_maintenance_half_of_failure_cost() {
        let asset = sample_asset(1, 2000, 40); // failure_rate=0.05
        let rcm = RcmAnalyzer {
            assets: vec![asset.clone()],
            maintenance_budget: 20_000.0,
            failure_consequence: 200_000.0,
        };
        let rr = rcm.risk_reduction_from_maintenance(&asset);
        // 0.05 * 0.5 * 200_000 = 5_000
        assert!(
            (rr - 5_000.0).abs() < 1e-6,
            "risk_reduction should be 5_000, got {rr}"
        );
    }

    #[test]
    fn test_maintenance_plan_length_equals_asset_count() {
        let rcm = RcmAnalyzer {
            assets: vec![sample_asset(1, 2000, 40), sample_asset(2, 2010, 40)],
            maintenance_budget: 50_000.0,
            failure_consequence: 300_000.0,
        };
        let plan = rcm.maintenance_plan();
        assert_eq!(
            plan.len(),
            2,
            "maintenance_plan should have one entry per asset, got {}",
            plan.len()
        );
    }

    #[test]
    fn test_optimize_maintenance_budget_within_budget() {
        let rcm = RcmAnalyzer {
            assets: vec![sample_asset(1, 2000, 40), sample_asset(2, 2010, 40)],
            maintenance_budget: 5_000.0,
            failure_consequence: 300_000.0,
        };
        let plan = rcm.optimize_maintenance_budget();
        assert!(
            plan.total_cost <= rcm.maintenance_budget + 1e-6,
            "total maintenance cost ({:.2}) must not exceed budget ({:.2})",
            plan.total_cost,
            rcm.maintenance_budget
        );
        assert!(
            plan.risk_reduction_pct >= 0.0 && plan.risk_reduction_pct <= 100.0,
            "risk_reduction_pct out of range: {}",
            plan.risk_reduction_pct
        );
    }

    #[test]
    fn test_prioritize_candidates_sorted_descending() {
        let planner = sample_der_planner();
        let ranked = planner.prioritize_candidates();
        assert_eq!(ranked.len(), 2, "should rank all 2 candidates");
        // First ranked candidate should have a score >= second
        if ranked.len() >= 2 {
            assert!(
                ranked[0].1 >= ranked[1].1,
                "candidates should be in descending score order: {:.4} >= {:.4}",
                ranked[0].1,
                ranked[1].1
            );
        }
    }

    #[test]
    fn test_total_capex_requirements_sums_all_project_costs() {
        let mut strategy = LongTermStrategy::new(10, 1_000_000.0);
        strategy.projects = vec![
            sample_project(1, 100_000.0, 2025),
            sample_project(2, 200_000.0, 2026),
            sample_project(3, 50_000.0, 2027),
        ];
        let total = strategy.total_capex_requirements();
        assert!(
            (total - 350_000.0).abs() < 1e-6,
            "total CAPEX should be 350_000, got {total}"
        );
    }
}
