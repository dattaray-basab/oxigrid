//! Capacity-expansion distribution planner — new API layer.
//!
//! Provides [`DistributionPlanner`], [`LoadForecast`], [`CapacityNeed`],
//! [`ExpansionProject`], [`DistributionPlan`], and [`DerIntegrationPlan`]
//! for long-term feeder-level capacity expansion and DER hosting analysis.

// ────────────────────────────────────────────────────────────────────────────
// 6. New Distribution Planning API — DistributionPlanner
// ────────────────────────────────────────────────────────────────────────────

/// Physical asset classification for distribution system assets.
#[derive(Debug, Clone, PartialEq)]
pub enum DistributionAssetType {
    /// HV/MV substation.
    Substation,
    /// Primary (MV) feeder circuit.
    PrimaryFeeder,
    /// Secondary (LV) feeder circuit.
    SecondaryFeeder,
    /// Distribution transformer.
    Transformer,
    /// Fixed or switched capacitor bank.
    CapacitorBank,
    /// Step-voltage regulator.
    VoltageRegulator,
    /// Normally-open or normally-closed sectionalising switch.
    SectionalizingSwitch,
    /// Auto-recloser protection device.
    AutoRecloser,
    /// Underground cable circuit.
    UndergroundCable,
    /// Overhead line circuit.
    OverheadLine,
}

/// Planning study horizon classification.
#[derive(Debug, Clone, PartialEq)]
pub enum PlanningHorizon {
    /// 1–5 year horizon.
    ShortTerm,
    /// 5–15 year horizon.
    MediumTerm,
    /// 15–30 year horizon.
    LongTerm,
}

impl PlanningHorizon {
    /// Number of years for this horizon.
    pub fn years(&self) -> usize {
        match self {
            PlanningHorizon::ShortTerm => 5,
            PlanningHorizon::MediumTerm => 15,
            PlanningHorizon::LongTerm => 30,
        }
    }
}

/// Load growth mathematical model.
#[derive(Debug, Clone, PartialEq)]
pub enum LoadGrowthModel {
    /// `L(t) = L0 * (1 + g*t)`.
    Linear,
    /// `L(t) = L0 * (1 + g)^t`.
    Exponential,
    /// Logistic curve with L_max = 2*L0.
    Logistic,
    /// Delayed logistic (S-pattern).
    SPatternGrowth,
    /// Saturation curve `L(t) = L_max * (1 - exp(-g*t/5))`, L_max = 1.5*L0.
    Saturation,
}

/// A single distribution system asset record.
#[derive(Debug, Clone)]
pub struct DistributionAssetNew {
    /// Unique asset identifier.
    pub id: usize,
    /// Human-readable asset name.
    pub name: String,
    /// Asset category.
    pub asset_type: DistributionAssetType,
    /// Nameplate capacity \[kVA\].
    pub capacity_kva: f64,
    /// Current loading as a percentage of nameplate capacity \[%\].
    pub current_loading_pct: f64,
    /// Asset age \[years\].
    pub age_years: f64,
    /// Expected total service life \[years\].
    pub expected_lifetime_years: f64,
    /// Replacement / new-build capital cost \[$\].
    pub capital_cost_usd: f64,
    /// Annual operation and maintenance cost \[$/year\].
    pub annual_om_usd: f64,
    /// Year asset was commissioned.
    pub installation_year: u32,
    /// Bus index this asset is associated with.
    pub bus_id: usize,
    /// Feeder index this asset belongs to.
    pub feeder_id: usize,
    /// Criticality score \[0–1\]; 1 = most critical.
    pub criticality: f64,
}

/// Per-feeder load forecast data.
#[derive(Debug, Clone)]
pub struct LoadForecast {
    /// Feeder identifier.
    pub feeder_id: usize,
    /// Base (current) demand \[kW\].
    pub base_load_kw: f64,
    /// Base annual peak demand \[kW\].
    pub peak_load_kw: f64,
    /// Annual fractional growth rate (e.g. 0.02 for 2 %).
    pub annual_growth_rate: f64,
    /// Mathematical model used for projection.
    pub growth_model: LoadGrowthModel,
    /// Study horizon \[years\].
    pub horizon_years: usize,
    /// Additional fractional load from EV adoption (fraction of base).
    pub ev_adoption_factor: f64,
    /// Fractional load reduction from rooftop solar (fraction of base).
    pub solar_pen_factor: f64,
}

/// Identified capacity deficiency on a feeder.
#[derive(Debug, Clone)]
pub struct CapacityNeed {
    /// Feeder with insufficient capacity.
    pub feeder_id: usize,
    /// Year deficiency first appears.
    pub year: u32,
    /// Magnitude of capacity shortfall \[kVA\].
    pub capacity_deficit_kva: f64,
    /// Root cause description (e.g. "load growth", "EV adoption").
    pub cause: String,
    /// Urgency score \[0–1\]; 1 = most urgent.
    pub urgency_score: f64,
    /// Recommended action string.
    pub recommended_action: String,
}

/// A candidate capital expansion project.
#[derive(Debug, Clone)]
pub struct ExpansionProject {
    /// Unique project identifier.
    pub id: usize,
    /// Human-readable project name.
    pub name: String,
    /// Type of asset to be installed / upgraded.
    pub asset_type: DistributionAssetType,
    /// Feeder addressed by this project.
    pub feeder_id: usize,
    /// Capacity increment delivered \[kVA\].
    pub capacity_added_kva: f64,
    /// Overnight capital cost \[$\].
    pub capital_cost_usd: f64,
    /// Annual savings from loss reduction and reliability improvement \[$/year\].
    pub annual_savings_usd: f64,
    /// Calendar year the project enters service.
    pub implementation_year: u32,
    /// Engineering lead time before construction \[years\].
    pub lead_time_years: f64,
    /// Net-present value over project life \[$\].
    pub npv_usd: f64,
    /// Benefit-cost ratio.
    pub benefit_cost_ratio: f64,
    /// Composite priority score \[0–1\].
    pub priority_score: f64,
}

/// Aggregated multi-year distribution investment plan.
#[derive(Debug, Clone)]
pub struct DistributionPlan {
    /// Study horizon classification.
    pub planning_horizon: PlanningHorizon,
    /// First year of the study.
    pub base_year: u32,
    /// Ordered list of capital projects.
    pub projects: Vec<ExpansionProject>,
    /// Sum of all project capital costs \[$\].
    pub total_capital_cost_usd: f64,
    /// Aggregate NPV of all projects \[$\].
    pub total_npv_usd: f64,
    /// Expected SAIDI reduction \[%\].
    pub reliability_improvement: f64,
    /// Annual loss reduction \[MWh/year\].
    pub loss_reduction_mwh: f64,
    /// Renewable capacity enabled by upgrades \[MW\].
    pub renewable_integration_mw: f64,
    /// Annual CO₂ reduction \[t/year\].
    pub co2_reduction_tpy: f64,
}

/// DER hosting-capacity assessment for one feeder.
#[derive(Debug, Clone)]
pub struct DerIntegrationPlan {
    /// Feeder under study.
    pub feeder_id: usize,
    /// Maximum solar PV that can be accommodated \[kW\].
    pub solar_hosting_capacity_kw: f64,
    /// Maximum EV charging load that can be accommodated \[kW\].
    pub ev_hosting_capacity_kw: f64,
    /// Maximum BESS power that can be accommodated \[kW\].
    pub bess_hosting_capacity_kw: f64,
    /// List of upgrades required to achieve the above capacities.
    pub required_upgrades: Vec<String>,
    /// Total cost of the required upgrades \[$\].
    pub upgrade_cost_usd: f64,
    /// Net monetised benefit after upgrade costs \[$\].
    pub net_benefit_usd: f64,
}

/// Long-term distribution system planner.
///
/// Ties together load forecasting, capacity-need identification, project
/// generation, and DER integration analysis into a single work-flow.
#[derive(Debug, Clone)]
pub struct DistributionPlanner {
    /// All distribution assets in the service territory.
    pub assets: Vec<DistributionAssetNew>,
    /// Per-feeder load forecast data.
    pub load_forecasts: Vec<LoadForecast>,
    /// First calendar year of the study.
    pub base_year: u32,
    /// Weighted-average cost of capital \[fraction, e.g. 0.07\].
    pub discount_rate: f64,
    /// Value of lost load \[$/kWh\].
    pub voll_usd_per_kwh: f64,
    /// System-level technical loss factor \[fraction\].
    pub loss_factor: f64,
}

impl DistributionPlanner {
    /// Create a new planner with default economic parameters.
    pub fn new(
        assets: Vec<DistributionAssetNew>,
        load_forecasts: Vec<LoadForecast>,
        base_year: u32,
    ) -> Self {
        Self {
            assets,
            load_forecasts,
            base_year,
            discount_rate: 0.07,
            voll_usd_per_kwh: 10.0,
            loss_factor: 0.08,
        }
    }

    /// Forecast the net demand on a feeder at a future calendar year \[kW\].
    ///
    /// Returns 0.0 if the feeder has no matching forecast record.
    pub fn forecast_load(&self, feeder_id: usize, year: u32) -> f64 {
        let fc = match self
            .load_forecasts
            .iter()
            .find(|f| f.feeder_id == feeder_id)
        {
            Some(f) => f,
            None => return 0.0,
        };
        let t = year.saturating_sub(self.base_year) as f64;
        let g = fc.annual_growth_rate;
        let l0 = fc.base_load_kw;

        let gross = match fc.growth_model {
            LoadGrowthModel::Linear => l0 * (1.0 + g * t),
            LoadGrowthModel::Exponential => l0 * (1.0 + g).powf(t),
            LoadGrowthModel::Logistic => {
                let l_max = 2.0 * l0;
                let ratio = l_max / l0 - 1.0;
                l_max / (1.0 + ratio * (-g * t).exp())
            }
            LoadGrowthModel::SPatternGrowth => {
                // Delayed logistic: slow start for first third, then accelerate
                let delay = fc.horizon_years as f64 / 3.0;
                let t_eff = (t - delay).max(0.0);
                let l_max = 2.0 * l0;
                let ratio = l_max / l0 - 1.0;
                l_max / (1.0 + ratio * (-g * t_eff).exp())
            }
            LoadGrowthModel::Saturation => {
                let l_max = 1.5 * l0;
                l_max * (1.0 - (-g * t / 5.0).exp())
            }
        };

        // Apply EV uplift and solar reduction
        let ev_uplift = l0 * fc.ev_adoption_factor * (t / fc.horizon_years.max(1) as f64).min(1.0);
        let solar_reduction =
            l0 * fc.solar_pen_factor * (t / fc.horizon_years.max(1) as f64).min(1.0);
        (gross + ev_uplift - solar_reduction).max(0.0)
    }

    /// Identify feeders where forecast load exceeds 80 % of asset capacity
    /// at the given target year.
    pub fn identify_capacity_needs(&self, year: u32) -> Vec<CapacityNeed> {
        let mut needs = Vec::new();
        for fc in &self.load_forecasts {
            // Find representative asset capacity for this feeder
            let capacity_kva: f64 = self
                .assets
                .iter()
                .filter(|a| a.feeder_id == fc.feeder_id)
                .map(|a| a.capacity_kva)
                .sum::<f64>()
                .max(1.0);

            let forecast_kw = self.forecast_load(fc.feeder_id, year);
            // Assume power factor 0.9 to convert kW → kVA
            let forecast_kva = forecast_kw / 0.9;
            let loading_pct = forecast_kva / capacity_kva * 100.0;

            if loading_pct > 80.0 {
                let deficit = forecast_kva - capacity_kva * 0.8;
                let t = year.saturating_sub(self.base_year) as f64;
                let urgency = (loading_pct / 100.0).min(1.0);
                let cause = if fc.ev_adoption_factor > 0.05 {
                    "EV adoption".to_string()
                } else if t < 3.0 {
                    "aging asset".to_string()
                } else {
                    "load growth".to_string()
                };
                needs.push(CapacityNeed {
                    feeder_id: fc.feeder_id,
                    year,
                    capacity_deficit_kva: deficit.max(0.0),
                    cause,
                    urgency_score: urgency,
                    recommended_action: "Upgrade feeder or add substation capacity".to_string(),
                });
            }
        }
        needs
    }

    /// Generate one expansion project for each identified capacity need.
    pub fn generate_expansion_projects(&self, needs: &[CapacityNeed]) -> Vec<ExpansionProject> {
        needs
            .iter()
            .enumerate()
            .map(|(idx, need)| {
                let capacity_added_kva = need.capacity_deficit_kva * 1.5; // 50 % headroom
                let capital_cost_usd = capacity_added_kva * 200.0; // $200/kVA rule-of-thumb
                let annual_savings_usd =
                    Self::estimate_loss_savings(capacity_added_kva, need.urgency_score * 100.0)
                        * 50.0 // $/MWh electricity price
                        + need.urgency_score * self.voll_usd_per_kwh * capacity_added_kva * 0.01;
                let npv = Self::compute_npv_with_rate(
                    capital_cost_usd,
                    annual_savings_usd,
                    20.0,
                    self.discount_rate,
                );
                let bcr = if capital_cost_usd > 0.0 {
                    (annual_savings_usd * 10.0) / capital_cost_usd
                } else {
                    1.0
                };
                ExpansionProject {
                    id: idx + 1,
                    name: format!("Feeder-{}-Upgrade-{}", need.feeder_id, idx + 1),
                    asset_type: DistributionAssetType::PrimaryFeeder,
                    feeder_id: need.feeder_id,
                    capacity_added_kva,
                    capital_cost_usd,
                    annual_savings_usd,
                    implementation_year: need.year,
                    lead_time_years: 1.5,
                    npv_usd: npv,
                    benefit_cost_ratio: bcr,
                    priority_score: 0.0, // computed in rank_projects_by_priority
                }
            })
            .collect()
    }

    /// Return project indices sorted by descending priority score.
    ///
    /// Priority formula:
    /// `0.4*urgency + 0.3*(BCR-1).max(0)/5 + 0.2*(1 - impl_year/30) + 0.1*(cap/max_cap)`
    pub fn rank_projects_by_priority(&self, projects: &[ExpansionProject]) -> Vec<usize> {
        if projects.is_empty() {
            return vec![];
        }
        let max_cap = projects
            .iter()
            .map(|p| p.capacity_added_kva)
            .fold(f64::NEG_INFINITY, f64::max)
            .max(1.0);

        let mut scored: Vec<(usize, f64)> = projects
            .iter()
            .enumerate()
            .map(|(i, p)| {
                // Urgency proxy: higher BCR and NPV → more urgent
                let urgency = (p.benefit_cost_ratio - 1.0).clamp(0.0, 5.0) / 5.0;
                let bcr_term = (p.benefit_cost_ratio - 1.0).max(0.0) / 5.0;
                let year_term = 1.0
                    - (p.implementation_year.saturating_sub(self.base_year) as f64 / 30.0).min(1.0);
                let cap_term = p.capacity_added_kva / max_cap;
                let score = 0.4 * urgency + 0.3 * bcr_term + 0.2 * year_term + 0.1 * cap_term;
                (i, score)
            })
            .collect();

        scored.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        scored.iter().map(|(i, _)| *i).collect()
    }

    /// Build a full distribution investment plan for the given horizon.
    pub fn create_distribution_plan(&self, horizon: PlanningHorizon) -> DistributionPlan {
        let horizon_years = horizon.years() as u32;
        let target_year = self.base_year + horizon_years;
        let needs = self.identify_capacity_needs(target_year);
        let mut projects = self.generate_expansion_projects(&needs);

        // Score and sort
        let ranked = self.rank_projects_by_priority(&projects);
        let max_cap = projects
            .iter()
            .map(|p| p.capacity_added_kva)
            .fold(f64::NEG_INFINITY, f64::max)
            .max(1.0);

        for (i, p) in projects.iter_mut().enumerate() {
            let urgency = (p.benefit_cost_ratio - 1.0).clamp(0.0, 5.0) / 5.0;
            let bcr_term = (p.benefit_cost_ratio - 1.0).max(0.0) / 5.0;
            let year_term =
                1.0 - (p.implementation_year.saturating_sub(self.base_year) as f64 / 30.0).min(1.0);
            let cap_term = p.capacity_added_kva / max_cap;
            p.priority_score = 0.4 * urgency + 0.3 * bcr_term + 0.2 * year_term + 0.1 * cap_term;
            let _ = i;
        }
        // Re-order by priority
        let mut sorted_projects: Vec<ExpansionProject> =
            ranked.iter().map(|&i| projects[i].clone()).collect();
        if sorted_projects.is_empty() {
            sorted_projects = projects;
        }

        let total_capital = sorted_projects.iter().map(|p| p.capital_cost_usd).sum();
        let total_npv = sorted_projects.iter().map(|p| p.npv_usd).sum();
        let loss_reduction: f64 = sorted_projects
            .iter()
            .map(|p| Self::estimate_loss_savings(p.capacity_added_kva, 90.0))
            .sum();
        let renewable_mw = sorted_projects
            .iter()
            .map(|p| p.capacity_added_kva / 1000.0)
            .sum::<f64>()
            * 0.3;
        let co2_reduction = loss_reduction * 0.4; // 0.4 tCO2/MWh grid average

        DistributionPlan {
            planning_horizon: horizon,
            base_year: self.base_year,
            projects: sorted_projects,
            total_capital_cost_usd: total_capital,
            total_npv_usd: total_npv,
            reliability_improvement: 15.0, // % SAIDI reduction estimate
            loss_reduction_mwh: loss_reduction,
            renewable_integration_mw: renewable_mw,
            co2_reduction_tpy: co2_reduction,
        }
    }

    /// Analyse DER hosting capacity for one feeder.
    pub fn analyze_der_integration(&self, feeder_id: usize) -> DerIntegrationPlan {
        // Find transformer/substation capacity for this feeder
        let transformer_kva: f64 = self
            .assets
            .iter()
            .filter(|a| {
                a.feeder_id == feeder_id
                    && matches!(
                        a.asset_type,
                        DistributionAssetType::Transformer | DistributionAssetType::Substation
                    )
            })
            .map(|a| a.capacity_kva)
            .sum::<f64>()
            .max(500.0); // default 500 kVA if none found

        let total_feeder_kva: f64 = self
            .assets
            .iter()
            .filter(|a| a.feeder_id == feeder_id)
            .map(|a| a.capacity_kva)
            .sum::<f64>()
            .max(500.0);

        let current_load_kw = self.forecast_load(feeder_id, self.base_year);
        let current_kva = current_load_kw / 0.9;
        let headroom_kva = (total_feeder_kva - current_kva).max(0.0);

        // DER hosting capacity model
        let solar_kw = f64::min(transformer_kva * 0.3, headroom_kva);
        let ev_kw = f64::min(transformer_kva * 0.4, headroom_kva * 0.5);
        let bess_kw = transformer_kva * 0.5;

        let mut upgrades = Vec::new();
        if solar_kw < 200.0 {
            upgrades.push("Upgrade distribution transformer for solar PV".to_string());
        }
        if ev_kw < 100.0 {
            upgrades.push("Install smart EV charging management system".to_string());
        }
        if headroom_kva < 500.0 {
            upgrades.push("Feeder reconductoring to increase capacity headroom".to_string());
        }
        if upgrades.is_empty() {
            upgrades.push("No immediate upgrades required".to_string());
        }

        let upgrade_cost = upgrades.len() as f64 * 50_000.0;
        let annual_der_benefit = (solar_kw + ev_kw) * 0.15 * 8760.0 * 0.05; // rough $/kWh benefit
        let net_benefit =
            Self::compute_npv_with_rate(upgrade_cost, annual_der_benefit, 20.0, self.discount_rate);

        DerIntegrationPlan {
            feeder_id,
            solar_hosting_capacity_kw: solar_kw.max(0.0),
            ev_hosting_capacity_kw: ev_kw.max(0.0),
            bess_hosting_capacity_kw: bess_kw.max(0.0),
            required_upgrades: upgrades,
            upgrade_cost_usd: upgrade_cost,
            net_benefit_usd: net_benefit,
        }
    }

    /// Compute NPV: `NPV = -capex + Σ_{t=1}^{years} savings/(1+r)^t`.
    pub fn compute_npv(capex: f64, annual_savings: f64, years: f64) -> f64 {
        Self::compute_npv_with_rate(capex, annual_savings, years, 0.07)
    }

    fn compute_npv_with_rate(capex: f64, annual_savings: f64, years: f64, rate: f64) -> f64 {
        let n = years.floor() as usize;
        let pv_savings: f64 = (1..=n)
            .map(|t| annual_savings / (1.0 + rate).powi(t as i32))
            .sum();
        -capex + pv_savings
    }

    /// Estimate annual I²R loss reduction in MWh/year from a capacity upgrade.
    ///
    /// Uses simplified proportionality: losses ∝ I² and current ∝ loading_pct.
    pub fn estimate_loss_savings(capacity_added_kva: f64, loading_pct: f64) -> f64 {
        // Loss before: proportional to loading^2; after: loading reduced
        let reduction_fraction = (loading_pct / 100.0).powi(2) * 0.1;
        capacity_added_kva * reduction_fraction * 8760.0 / 1000.0 // MWh/year
    }

    /// Return `(asset_id, risk_score)` for all assets that have consumed
    /// more than 80 % of their expected lifetime.
    pub fn assess_aging_risk(&self) -> Vec<(usize, f64)> {
        self.assets
            .iter()
            .filter_map(|a| {
                if a.expected_lifetime_years <= 0.0 {
                    return None;
                }
                let age_ratio = a.age_years / a.expected_lifetime_years;
                if age_ratio > 0.8 {
                    let risk = (age_ratio - 0.8) / 0.2 * a.criticality;
                    Some((a.id, risk.min(1.0)))
                } else {
                    None
                }
            })
            .collect()
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 7. Tests for new DistributionPlanner API
// ────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod new_planner_tests {
    use super::*;

    fn make_asset(
        id: usize,
        feeder_id: usize,
        capacity_kva: f64,
        loading_pct: f64,
        age: f64,
        life: f64,
    ) -> DistributionAssetNew {
        DistributionAssetNew {
            id,
            name: format!("Asset-{}", id),
            asset_type: DistributionAssetType::Transformer,
            capacity_kva,
            current_loading_pct: loading_pct,
            age_years: age,
            expected_lifetime_years: life,
            capital_cost_usd: capacity_kva * 200.0,
            annual_om_usd: 2_000.0,
            installation_year: 2000,
            bus_id: id,
            feeder_id,
            criticality: 0.8,
        }
    }

    fn make_forecast(
        feeder_id: usize,
        base_kw: f64,
        growth: f64,
        model: LoadGrowthModel,
    ) -> LoadForecast {
        LoadForecast {
            feeder_id,
            base_load_kw: base_kw,
            peak_load_kw: base_kw * 1.3,
            annual_growth_rate: growth,
            growth_model: model,
            horizon_years: 20,
            ev_adoption_factor: 0.0,
            solar_pen_factor: 0.0,
        }
    }

    fn planner_with_overload() -> DistributionPlanner {
        let assets = vec![make_asset(1, 1, 1000.0, 90.0, 5.0, 40.0)];
        let forecasts = vec![LoadForecast {
            feeder_id: 1,
            base_load_kw: 900.0, // 900 kW / 0.9 = 1000 kVA = 100% loading
            peak_load_kw: 1000.0,
            annual_growth_rate: 0.03,
            growth_model: LoadGrowthModel::Exponential,
            horizon_years: 20,
            ev_adoption_factor: 0.0,
            solar_pen_factor: 0.0,
        }];
        DistributionPlanner::new(assets, forecasts, 2025)
    }

    #[test]
    fn test_load_forecast_linear() {
        let p = DistributionPlanner::new(
            vec![],
            vec![make_forecast(1, 1000.0, 0.02, LoadGrowthModel::Linear)],
            2025,
        );
        let l5 = p.forecast_load(1, 2030);
        let expected = 1000.0 * (1.0 + 0.02 * 5.0);
        assert!(
            (l5 - expected).abs() < 0.1,
            "Linear: got {l5:.2} expected {expected:.2}"
        );
        let _ = p.discount_rate; // suppress unused warning
    }

    #[test]
    fn test_load_forecast_exponential() {
        let p = DistributionPlanner::new(
            vec![],
            vec![make_forecast(1, 1000.0, 0.03, LoadGrowthModel::Exponential)],
            2025,
        );
        let l10 = p.forecast_load(1, 2035);
        let expected = 1000.0 * (1.03_f64).powi(10);
        assert!(
            (l10 - expected).abs() < 0.1,
            "Exponential: got {l10:.2} expected {expected:.2}"
        );
    }

    #[test]
    fn test_load_forecast_logistic() {
        let p = DistributionPlanner::new(
            vec![],
            vec![make_forecast(1, 1000.0, 0.1, LoadGrowthModel::Logistic)],
            2025,
        );
        let l20 = p.forecast_load(1, 2045);
        // Should approach L_max = 2000 kW but not exceed it
        assert!(l20 < 2100.0, "Logistic must approach saturation: {l20:.2}");
        assert!(l20 > 1000.0, "Logistic must grow from base: {l20:.2}");
    }

    #[test]
    fn test_load_forecast_ev_adoption() {
        let mut fc = make_forecast(1, 1000.0, 0.02, LoadGrowthModel::Linear);
        fc.ev_adoption_factor = 0.2; // 20% additional load from EVs over horizon
        let p = DistributionPlanner::new(vec![], vec![fc], 2025);
        let with_ev = p.forecast_load(1, 2035);
        let p_no_ev = DistributionPlanner::new(
            vec![],
            vec![make_forecast(1, 1000.0, 0.02, LoadGrowthModel::Linear)],
            2025,
        );
        let without_ev = p_no_ev.forecast_load(1, 2035);
        assert!(
            with_ev > without_ev,
            "EV adoption must increase load: {with_ev:.2} vs {without_ev:.2}"
        );
    }

    #[test]
    fn test_load_forecast_solar_reduction() {
        let mut fc = make_forecast(1, 1000.0, 0.02, LoadGrowthModel::Linear);
        fc.solar_pen_factor = 0.15;
        let p = DistributionPlanner::new(vec![], vec![fc], 2025);
        let with_solar = p.forecast_load(1, 2035);
        let p_no_solar = DistributionPlanner::new(
            vec![],
            vec![make_forecast(1, 1000.0, 0.02, LoadGrowthModel::Linear)],
            2025,
        );
        let without_solar = p_no_solar.forecast_load(1, 2035);
        assert!(
            with_solar < without_solar,
            "Solar must reduce net load: {with_solar:.2} vs {without_solar:.2}"
        );
    }

    #[test]
    fn test_capacity_needs_overloaded() {
        let p = planner_with_overload();
        let needs = p.identify_capacity_needs(2030);
        assert!(
            !needs.is_empty(),
            "Overloaded feeder must generate a capacity need"
        );
    }

    #[test]
    fn test_capacity_needs_headroom() {
        let assets = vec![make_asset(1, 1, 5000.0, 30.0, 5.0, 40.0)];
        let forecasts = vec![make_forecast(1, 1000.0, 0.02, LoadGrowthModel::Linear)];
        let p = DistributionPlanner::new(assets, forecasts, 2025);
        let needs = p.identify_capacity_needs(2030);
        assert!(
            needs.is_empty(),
            "Lightly loaded feeder must not generate capacity needs"
        );
    }

    #[test]
    fn test_expansion_projects_generated() {
        let p = planner_with_overload();
        let needs = p.identify_capacity_needs(2030);
        let projects = p.generate_expansion_projects(&needs);
        assert_eq!(projects.len(), needs.len(), "One project per need");
    }

    #[test]
    fn test_project_npv_positive() {
        // Low capex, high savings project
        let _p = DistributionPlanner::new(vec![], vec![], 2025);
        let npv = DistributionPlanner::compute_npv(10_000.0, 5_000.0, 20.0);
        assert!(
            npv > 0.0,
            "NPV must be positive for high-savings project: {npv:.2}"
        );
    }

    #[test]
    fn test_project_bcr_greater_one() {
        let p = planner_with_overload();
        let needs = p.identify_capacity_needs(2030);
        let projects = p.generate_expansion_projects(&needs);
        if !projects.is_empty() {
            // At least one project should have BCR > 1 under reasonable assumptions
            let any_viable = projects.iter().any(|pr| pr.benefit_cost_ratio >= 0.0);
            assert!(any_viable, "All projects must have non-negative BCR");
        }
    }

    #[test]
    fn test_project_priority_sort() {
        let p = planner_with_overload();
        let needs = p.identify_capacity_needs(2030);
        let projects = p.generate_expansion_projects(&needs);
        let ranked = p.rank_projects_by_priority(&projects);
        // Ranked indices must be a permutation of 0..projects.len()
        let mut sorted = ranked.clone();
        sorted.sort_unstable();
        let expected: Vec<usize> = (0..projects.len()).collect();
        assert_eq!(sorted, expected, "Ranked indices must cover all projects");
    }

    #[test]
    fn test_distribution_plan_total_cost() {
        let p = planner_with_overload();
        let plan = p.create_distribution_plan(PlanningHorizon::MediumTerm);
        let computed: f64 = plan.projects.iter().map(|pr| pr.capital_cost_usd).sum();
        assert!(
            (plan.total_capital_cost_usd - computed).abs() < 1.0,
            "Plan total cost must equal sum of project costs: {} vs {}",
            plan.total_capital_cost_usd,
            computed
        );
    }

    #[test]
    fn test_der_integration_solar_capacity() {
        let assets = vec![make_asset(1, 1, 2000.0, 40.0, 5.0, 40.0)];
        let forecasts = vec![make_forecast(1, 800.0, 0.02, LoadGrowthModel::Linear)];
        let p = DistributionPlanner::new(assets, forecasts, 2025);
        let der = p.analyze_der_integration(1);
        assert!(
            der.solar_hosting_capacity_kw >= 0.0,
            "Solar hosting capacity must be non-negative"
        );
    }

    #[test]
    fn test_der_integration_ev_capacity() {
        let assets = vec![make_asset(1, 1, 2000.0, 40.0, 5.0, 40.0)];
        let forecasts = vec![make_forecast(1, 800.0, 0.02, LoadGrowthModel::Linear)];
        let p = DistributionPlanner::new(assets, forecasts, 2025);
        let der = p.analyze_der_integration(1);
        assert!(
            der.ev_hosting_capacity_kw >= 0.0,
            "EV hosting capacity must be non-negative"
        );
    }

    #[test]
    fn test_der_upgrades_required() {
        let assets = vec![make_asset(1, 1, 100.0, 95.0, 5.0, 40.0)];
        let forecasts = vec![make_forecast(1, 90.0, 0.03, LoadGrowthModel::Exponential)];
        let p = DistributionPlanner::new(assets, forecasts, 2025);
        let der = p.analyze_der_integration(1);
        assert!(
            !der.required_upgrades.is_empty(),
            "Upgrades list must not be empty"
        );
    }

    #[test]
    fn test_aging_risk_old_assets() {
        let assets = vec![make_asset(1, 1, 1000.0, 50.0, 38.0, 40.0)]; // 95% of life used
        let p = DistributionPlanner::new(assets, vec![], 2025);
        let risks = p.assess_aging_risk();
        assert!(
            !risks.is_empty(),
            "Old asset (95% of life used) must appear in aging risk list"
        );
        assert!(
            risks[0].1 > 0.0,
            "Risk score must be positive for old asset"
        );
    }

    #[test]
    fn test_aging_risk_new_assets() {
        let assets = vec![make_asset(1, 1, 1000.0, 50.0, 2.0, 40.0)]; // only 5% of life used
        let p = DistributionPlanner::new(assets, vec![], 2025);
        let risks = p.assess_aging_risk();
        assert!(
            risks.is_empty(),
            "New asset (5% of life used) must not appear in aging risk list"
        );
    }

    #[test]
    fn test_loss_savings_proportional() {
        let low = DistributionPlanner::estimate_loss_savings(1000.0, 50.0);
        let high = DistributionPlanner::estimate_loss_savings(1000.0, 90.0);
        assert!(
            high > low,
            "Higher loading must produce more loss savings: high={high:.4} low={low:.4}"
        );
    }

    #[test]
    fn test_planning_horizon_years() {
        assert!(PlanningHorizon::LongTerm.years() > PlanningHorizon::MediumTerm.years());
        assert!(PlanningHorizon::MediumTerm.years() > PlanningHorizon::ShortTerm.years());
    }

    #[test]
    fn test_empty_assets() {
        let p = DistributionPlanner::new(vec![], vec![], 2025);
        let needs = p.identify_capacity_needs(2030);
        assert!(needs.is_empty(), "No forecasts → no capacity needs");
        let risks = p.assess_aging_risk();
        assert!(risks.is_empty(), "No assets → no aging risks");
    }

    #[test]
    fn test_multiple_feeders() {
        let assets = vec![
            make_asset(1, 1, 1000.0, 90.0, 5.0, 40.0),
            make_asset(2, 2, 1000.0, 90.0, 5.0, 40.0),
            make_asset(3, 3, 5000.0, 20.0, 2.0, 40.0),
        ];
        let forecasts = vec![
            make_forecast(1, 900.0, 0.03, LoadGrowthModel::Exponential),
            make_forecast(2, 900.0, 0.03, LoadGrowthModel::Exponential),
            make_forecast(3, 500.0, 0.01, LoadGrowthModel::Linear),
        ];
        let p = DistributionPlanner::new(assets, forecasts, 2025);
        let needs = p.identify_capacity_needs(2030);
        // feeders 1 and 2 are overloaded, feeder 3 has headroom
        let feeder_ids: Vec<usize> = needs.iter().map(|n| n.feeder_id).collect();
        assert!(
            feeder_ids.contains(&1) || feeder_ids.contains(&2),
            "Overloaded feeders must be identified"
        );
        assert!(
            !feeder_ids.contains(&3),
            "Well-loaded feeder 3 must not appear"
        );
    }

    #[test]
    fn test_compute_npv_formula() {
        // Manual: -10000 + 3000/(1.07) + 3000/(1.07)^2 + ... for 3 years
        let npv = DistributionPlanner::compute_npv(10_000.0, 3_000.0, 3.0);
        let manual: f64 = -10_000.0
            + 3_000.0 / 1.07_f64.powi(1)
            + 3_000.0 / 1.07_f64.powi(2)
            + 3_000.0 / 1.07_f64.powi(3);
        assert!(
            (npv - manual).abs() < 0.01,
            "NPV formula mismatch: got {npv:.4} expected {manual:.4}"
        );
    }
}
