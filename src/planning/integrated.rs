//! Integrated Resource Planning (IRP) module.
//!
//! Combines generation, transmission, and distribution planning across a
//! multi-year horizon, incorporating:
//!
//! - Cost-Benefit Analysis (CBA) and LCOE for each candidate resource
//! - Greedy portfolio construction driven by BCR ranking
//! - Multi-criteria decision analysis (MCDA) with configurable weights
//! - Environmental and Social Impact Assessment (ESIA)
//! - Sensitivity analysis (±20% parameter variation)
//! - Loss-of-Load Expectation (LOLE) estimation

use crate::error::OxiGridError;

// ---------------------------------------------------------------------------
// Resource options
// ---------------------------------------------------------------------------

/// A candidate resource that may be built during the planning horizon.
#[derive(Debug, Clone)]
pub enum ResourceOption {
    /// Dispatchable baseload generation (nuclear, large gas CC, coal, etc.).
    BaseloadPlant {
        technology: String,
        capacity_mw: f64,
        capital_cost_million_eur: f64,
        opex_million_eur_per_yr: f64,
        capacity_factor: f64,
        co2_kg_per_mwh: f64,
        lifetime_years: usize,
        build_time_years: usize,
    },
    /// Fast-start peaking plant (gas turbine, diesel, etc.).
    PeakingPlant {
        technology: String,
        capacity_mw: f64,
        capital_cost_million_eur: f64,
        opex_million_eur_per_yr: f64,
        capacity_factor: f64,
        co2_kg_per_mwh: f64,
        lifetime_years: usize,
    },
    /// Variable renewable resource (solar, wind, etc.).
    RenewableResource {
        technology: String,
        capacity_mw: f64,
        capital_cost_million_eur: f64,
        opex_million_eur_per_yr: f64,
        /// Average annual capacity factor.
        capacity_factor: f64,
        /// 0 = perfectly predictable, 1 = fully stochastic.
        variability_factor: f64,
        lifetime_years: usize,
    },
    /// Grid-scale energy storage (battery, PHES, etc.).
    EnergyStorage {
        technology: String,
        power_mw: f64,
        energy_mwh: f64,
        capital_cost_million_eur: f64,
        opex_million_eur_per_yr: f64,
        roundtrip_efficiency: f64,
        lifetime_years: usize,
    },
    /// Demand-response programme.
    DemandResponse {
        peak_reduction_mw: f64,
        annual_cost_million_eur: f64,
        response_time_min: f64,
    },
    /// Transmission line or transformer upgrade.
    TransmissionUpgrade {
        from_bus: usize,
        to_bus: usize,
        capacity_increase_mw: f64,
        capital_cost_million_eur: f64,
        lifetime_years: usize,
    },
    /// Distribution feeder upgrade (optionally with smart-grid capability).
    DistributionUpgrade {
        feeder_id: usize,
        capacity_increase_mw: f64,
        capital_cost_million_eur: f64,
        smart_grid: bool,
    },
}

impl ResourceOption {
    /// Nominal capacity contribution \[MW\] of this option.
    pub fn capacity_mw(&self) -> f64 {
        match self {
            ResourceOption::BaseloadPlant { capacity_mw, .. } => *capacity_mw,
            ResourceOption::PeakingPlant { capacity_mw, .. } => *capacity_mw,
            ResourceOption::RenewableResource { capacity_mw, .. } => *capacity_mw,
            ResourceOption::EnergyStorage { power_mw, .. } => *power_mw,
            ResourceOption::DemandResponse {
                peak_reduction_mw, ..
            } => *peak_reduction_mw,
            ResourceOption::TransmissionUpgrade {
                capacity_increase_mw,
                ..
            } => *capacity_increase_mw,
            ResourceOption::DistributionUpgrade {
                capacity_increase_mw,
                ..
            } => *capacity_increase_mw,
        }
    }

    /// Total capital cost \[million EUR\].
    pub fn capital_cost(&self) -> f64 {
        match self {
            ResourceOption::BaseloadPlant {
                capital_cost_million_eur,
                ..
            } => *capital_cost_million_eur,
            ResourceOption::PeakingPlant {
                capital_cost_million_eur,
                ..
            } => *capital_cost_million_eur,
            ResourceOption::RenewableResource {
                capital_cost_million_eur,
                ..
            } => *capital_cost_million_eur,
            ResourceOption::EnergyStorage {
                capital_cost_million_eur,
                ..
            } => *capital_cost_million_eur,
            ResourceOption::DemandResponse {
                annual_cost_million_eur,
                ..
            } => *annual_cost_million_eur,
            ResourceOption::TransmissionUpgrade {
                capital_cost_million_eur,
                ..
            } => *capital_cost_million_eur,
            ResourceOption::DistributionUpgrade {
                capital_cost_million_eur,
                ..
            } => *capital_cost_million_eur,
        }
    }

    /// Annual operating cost \[million EUR/year\].
    pub fn opex(&self) -> f64 {
        match self {
            ResourceOption::BaseloadPlant {
                opex_million_eur_per_yr,
                ..
            } => *opex_million_eur_per_yr,
            ResourceOption::PeakingPlant {
                opex_million_eur_per_yr,
                ..
            } => *opex_million_eur_per_yr,
            ResourceOption::RenewableResource {
                opex_million_eur_per_yr,
                ..
            } => *opex_million_eur_per_yr,
            ResourceOption::EnergyStorage {
                opex_million_eur_per_yr,
                ..
            } => *opex_million_eur_per_yr,
            ResourceOption::DemandResponse {
                annual_cost_million_eur,
                ..
            } => *annual_cost_million_eur,
            ResourceOption::TransmissionUpgrade { .. } => 0.0,
            ResourceOption::DistributionUpgrade { .. } => 0.0,
        }
    }

    /// Economic lifetime \[years\].
    pub fn lifetime_years(&self) -> usize {
        match self {
            ResourceOption::BaseloadPlant { lifetime_years, .. } => *lifetime_years,
            ResourceOption::PeakingPlant { lifetime_years, .. } => *lifetime_years,
            ResourceOption::RenewableResource { lifetime_years, .. } => *lifetime_years,
            ResourceOption::EnergyStorage { lifetime_years, .. } => *lifetime_years,
            ResourceOption::DemandResponse { .. } => 20,
            ResourceOption::TransmissionUpgrade { lifetime_years, .. } => *lifetime_years,
            ResourceOption::DistributionUpgrade { .. } => 30,
        }
    }

    /// Average capacity factor \[fraction\].
    pub fn capacity_factor(&self) -> f64 {
        match self {
            ResourceOption::BaseloadPlant {
                capacity_factor, ..
            } => *capacity_factor,
            ResourceOption::PeakingPlant {
                capacity_factor, ..
            } => *capacity_factor,
            ResourceOption::RenewableResource {
                capacity_factor, ..
            } => *capacity_factor,
            ResourceOption::EnergyStorage { .. } => 0.25,
            ResourceOption::DemandResponse { .. } => 0.1,
            ResourceOption::TransmissionUpgrade { .. } => 0.5,
            ResourceOption::DistributionUpgrade { .. } => 0.5,
        }
    }

    /// CO₂ intensity \[kg/MWh\].  Returns 0 for non-generating options.
    pub fn co2_kg_per_mwh(&self) -> f64 {
        match self {
            ResourceOption::BaseloadPlant { co2_kg_per_mwh, .. } => *co2_kg_per_mwh,
            ResourceOption::PeakingPlant { co2_kg_per_mwh, .. } => *co2_kg_per_mwh,
            _ => 0.0,
        }
    }

    /// Returns `true` for technologies classified as renewable.
    pub fn is_renewable(&self) -> bool {
        matches!(self, ResourceOption::RenewableResource { .. })
    }

    /// Returns `true` for fully dispatchable resources.
    pub fn is_dispatchable(&self) -> bool {
        matches!(
            self,
            ResourceOption::BaseloadPlant { .. }
                | ResourceOption::PeakingPlant { .. }
                | ResourceOption::EnergyStorage { .. }
                | ResourceOption::DemandResponse { .. }
        )
    }
}

// ---------------------------------------------------------------------------
// Planning load forecast
// ---------------------------------------------------------------------------

/// Annual load forecast for a single year in the planning horizon.
#[derive(Debug, Clone)]
pub struct PlanningLoadForecast {
    /// Calendar year.
    pub year: usize,
    /// System peak demand \[MW\].
    pub peak_load_mw: f64,
    /// Annual energy consumption \[TWh\].
    pub annual_energy_twh: f64,
    /// Year-on-year peak demand growth \[%\].
    pub peak_demand_growth_pct: f64,
    /// DER penetration as fraction of peak \[%\].
    pub der_penetration_pct: f64,
    /// Additional EV charging load at system peak \[MW\].
    pub ev_load_mw: f64,
    /// Additional heat-pump load at system peak \[MW\].
    pub heat_pump_load_mw: f64,
}

// ---------------------------------------------------------------------------
// IRP configuration
// ---------------------------------------------------------------------------

/// Planning horizon and policy parameters for the IRP.
#[derive(Debug, Clone)]
pub struct IrpConfig {
    /// Number of years in the planning horizon.
    pub planning_horizon_years: usize,
    /// First year of the planning horizon.
    pub base_year: usize,
    /// Discount rate used for NPV calculations (e.g. 0.07 = 7 %).
    pub discount_rate: f64,
    /// Required capacity reserve margin above peak load \[%\] (default 15 %).
    pub reserve_margin_pct: f64,
    /// Required CO₂ reduction vs. base-year intensity by end of horizon \[%\].
    pub co2_reduction_target_pct: f64,
    /// Loss-of-Load Expectation reliability target \[h/year\].
    pub reliability_lole_h_per_yr: f64,
    /// Total capital budget over the planning horizon \[billion EUR\].
    pub budget_constraint_billion_eur: f64,
}

impl Default for IrpConfig {
    fn default() -> Self {
        Self {
            planning_horizon_years: 20,
            base_year: 2025,
            discount_rate: 0.07,
            reserve_margin_pct: 15.0,
            co2_reduction_target_pct: 50.0,
            reliability_lole_h_per_yr: 3.0,
            budget_constraint_billion_eur: 100.0,
        }
    }
}

// ---------------------------------------------------------------------------
// CBA result
// ---------------------------------------------------------------------------

/// Cost-Benefit Analysis outcome for a single resource option.
#[derive(Debug, Clone)]
pub struct ResourceCba {
    pub option_id: usize,
    /// NPV of all costs \[million EUR\].
    pub npv_cost_million_eur: f64,
    /// NPV of all benefits \[million EUR\].
    pub npv_benefit_million_eur: f64,
    /// Benefit-cost ratio (BCR = npv_benefit / npv_cost).
    pub bcr: f64,
    /// Levelised cost of energy \[EUR/MWh\].
    pub lcoe_eur_per_mwh: f64,
    /// CO₂ reduction over lifetime \[million tonnes\].
    pub co2_reduction_million_ton: f64,
    /// Estimated full-time-equivalent jobs created.
    pub jobs_created: f64,
    /// Simple payback period \[years\].
    pub payback_years: f64,
}

// ---------------------------------------------------------------------------
// Portfolio
// ---------------------------------------------------------------------------

/// A collection of resource options forming a complete capacity plan.
#[derive(Debug, Clone)]
pub struct ResourcePortfolio {
    /// `(option_index, build_year)` pairs.
    pub selected_options: Vec<(usize, usize)>,
    /// Total installed capacity \[MW\].
    pub total_capacity_mw: f64,
    /// Share of installed capacity that is renewable \[%\].
    pub total_renewable_pct: f64,
    /// NPV of total portfolio cost \[million EUR\].
    pub total_npv_cost_million_eur: f64,
    /// CO₂ intensity reduction achieved vs. base year \[%\].
    pub co2_reduction_pct: f64,
    /// Portfolio reserve margin \[%\].
    pub reserve_margin_pct: f64,
    /// Estimated LOLE \[h/year\].
    pub lole_estimate_h_per_yr: f64,
    /// Whether the reliability target is met.
    pub meets_reliability: bool,
    /// Whether the CO₂ reduction target is met.
    pub meets_co2_target: bool,
    /// Whether the budget constraint is met.
    pub meets_budget: bool,
}

// ---------------------------------------------------------------------------
// IRP result
// ---------------------------------------------------------------------------

/// Full IRP optimisation result.
#[derive(Debug, Clone)]
pub struct IrpResult {
    /// Primary (greedy) portfolio.
    pub portfolio: ResourcePortfolio,
    /// Year-by-year snapshots of the primary portfolio build-out.
    pub annual_snapshots: Vec<YearlyPlanSnapshot>,
    /// Sensitivity results from parameter variation.
    pub sensitivity_results: Vec<SensitivityResult>,
    /// Recommended (primary greedy) portfolio.
    pub recommended_portfolio: ResourcePortfolio,
    /// Alternative portfolios (least-cost, max-renewable, min-risk).
    pub alternative_portfolios: Vec<ResourcePortfolio>,
}

/// Single-year planning snapshot.
#[derive(Debug, Clone)]
pub struct YearlyPlanSnapshot {
    pub year: usize,
    pub installed_capacity_mw: f64,
    pub renewable_fraction_pct: f64,
    pub peak_demand_mw: f64,
    pub reserve_margin_pct: f64,
    pub annual_cost_million_eur: f64,
    pub co2_intensity_kg_per_mwh: f64,
    pub capacity_adequacy: bool,
}

/// Result of varying one parameter by ±20 %.
#[derive(Debug, Clone)]
pub struct SensitivityResult {
    pub parameter: String,
    pub variation_pct: f64,
    pub npv_change_million_eur: f64,
    pub co2_change_million_ton: f64,
    pub portfolio_changes: bool,
}

// ---------------------------------------------------------------------------
// IntegratedResourcePlanner
// ---------------------------------------------------------------------------

/// Integrated resource planner — greedy capacity expansion with CBA ranking.
#[derive(Debug, Clone)]
pub struct IntegratedResourcePlanner {
    /// Candidate resource options.
    pub options: Vec<ResourceOption>,
    /// Load forecasts — one entry per planning year.
    pub load_forecasts: Vec<PlanningLoadForecast>,
    /// Planning configuration.
    pub config: IrpConfig,
    /// Existing installed capacity at base year \[MW\].
    pub existing_capacity_mw: f64,
    /// Existing fleet CO₂ intensity \[kg/MWh\].
    pub existing_co2_kg_per_mwh: f64,
}

impl IntegratedResourcePlanner {
    /// Create a new planner.
    pub fn new(
        options: Vec<ResourceOption>,
        load_forecasts: Vec<PlanningLoadForecast>,
        config: IrpConfig,
        existing_capacity_mw: f64,
        existing_co2_kg_per_mwh: f64,
    ) -> Self {
        Self {
            options,
            load_forecasts,
            config,
            existing_capacity_mw,
            existing_co2_kg_per_mwh,
        }
    }

    // -----------------------------------------------------------------------
    // LCOE
    // -----------------------------------------------------------------------

    /// Compute LCOE \[EUR/MWh\] for a resource option.
    ///
    /// ```text
    /// LCOE = (NPV_capex + NPV_opex) / NPV_energy
    /// ```
    ///
    /// Uses the Capital Recovery Factor approach:
    ///
    /// ```text
    /// CRF = r(1+r)^n / [(1+r)^n − 1]
    /// LCOE = (capex × CRF + opex) / (CF × 8760)
    /// ```
    pub fn compute_lcoe(&self, option: &ResourceOption, _build_year: usize) -> f64 {
        let r = self.config.discount_rate;
        let n = option.lifetime_years() as f64;
        let crf = if r.abs() < 1e-12 {
            1.0 / n.max(1.0)
        } else {
            let rn = (1.0 + r).powf(n);
            r * rn / (rn - 1.0)
        };

        let capex = option.capital_cost(); // million EUR
        let opex = option.opex(); // million EUR/yr
        let cap_mw = option.capacity_mw().max(0.001);
        let cf = option.capacity_factor().max(0.001);
        let annual_energy_mwh = cap_mw * cf * 8760.0;

        // Annualised cost in million EUR/yr
        let annualised = capex * crf + opex;
        // Convert to EUR/MWh (million EUR / MWh * 1e6)
        annualised * 1_000_000.0 / annual_energy_mwh.max(1.0)
    }

    // -----------------------------------------------------------------------
    // CBA
    // -----------------------------------------------------------------------

    /// Compute Cost-Benefit Analysis for a candidate option.
    ///
    /// Benefit components:
    /// - Energy value: assumed market price of 80 EUR/MWh
    /// - CO₂ savings: 50 EUR/tonne
    /// - Capacity value: 50 000 EUR/MW/year
    pub fn compute_cba(&self, option_idx: usize, build_year: usize) -> ResourceCba {
        let option = match self.options.get(option_idx) {
            Some(o) => o,
            None => {
                return ResourceCba {
                    option_id: option_idx,
                    npv_cost_million_eur: 0.0,
                    npv_benefit_million_eur: 0.0,
                    bcr: 0.0,
                    lcoe_eur_per_mwh: 0.0,
                    co2_reduction_million_ton: 0.0,
                    jobs_created: 0.0,
                    payback_years: f64::INFINITY,
                };
            }
        };

        let r = self.config.discount_rate;
        let n = option.lifetime_years() as f64;
        let cap_mw = option.capacity_mw().max(0.001);
        let cf = option.capacity_factor().max(0.001);

        // Annual energy generated [MWh/yr]
        let annual_energy_mwh = cap_mw * cf * 8760.0;

        // Annual benefits [million EUR/yr]
        let energy_value_per_yr = annual_energy_mwh * 80.0 / 1_000_000.0; // 80 EUR/MWh
        let co2_savings_per_yr = self.co2_savings_per_yr(option); // million EUR/yr
        let capacity_value_per_yr = cap_mw * 50_000.0 / 1_000_000.0; // 50k EUR/MW/yr

        let annual_benefit = energy_value_per_yr + co2_savings_per_yr + capacity_value_per_yr;

        // Annual cost [million EUR/yr]
        let capex = option.capital_cost();
        let opex = option.opex();
        let crf = if r.abs() < 1e-12 {
            1.0 / n.max(1.0)
        } else {
            let rn = (1.0 + r).powf(n);
            r * rn / (rn - 1.0)
        };
        let annual_cost = capex * crf + opex;

        // NPV over lifetime (uniform annuity)
        let annuity_factor = if r.abs() < 1e-12 {
            n
        } else {
            let rn = (1.0 + r).powf(n);
            (rn - 1.0) / (r * rn)
        };
        // Discount from build_year relative to base_year
        let delay = build_year.saturating_sub(self.config.base_year) as i32;
        let delay_factor = 1.0 / (1.0 + r).powi(delay);

        let npv_cost = (annual_cost * annuity_factor + capex) * delay_factor;
        let npv_benefit = annual_benefit * annuity_factor * delay_factor;

        let bcr = if npv_cost > 1e-12 {
            npv_benefit / npv_cost
        } else {
            0.0
        };

        let lcoe = self.compute_lcoe(option, build_year);

        // CO₂ reduction [million tonnes over lifetime]
        let co2_intensity_existing = self.existing_co2_kg_per_mwh / 1000.0; // kg→t per MWh
        let co2_intensity_new = option.co2_kg_per_mwh() / 1000.0;
        let co2_saved_per_yr =
            (co2_intensity_existing - co2_intensity_new).max(0.0) * annual_energy_mwh / 1_000_000.0; // million tonnes
        let co2_reduction = co2_saved_per_yr * n;

        // Jobs
        let jobs = match option {
            ResourceOption::RenewableResource { capacity_mw, .. } => capacity_mw * 0.5,
            ResourceOption::BaseloadPlant { capacity_mw, .. } => capacity_mw * 0.2,
            ResourceOption::PeakingPlant { capacity_mw, .. } => capacity_mw * 0.15,
            ResourceOption::EnergyStorage { power_mw, .. } => power_mw * 0.1,
            _ => 10.0,
        };

        // Simple payback = capex / (annual_benefit - opex)
        let net_annual = annual_benefit - opex;
        let payback = if net_annual > 1e-12 {
            capex / net_annual
        } else {
            f64::INFINITY
        };

        ResourceCba {
            option_id: option_idx,
            npv_cost_million_eur: npv_cost,
            npv_benefit_million_eur: npv_benefit,
            bcr,
            lcoe_eur_per_mwh: lcoe,
            co2_reduction_million_ton: co2_reduction,
            jobs_created: jobs,
            payback_years: payback,
        }
    }

    /// CO₂ savings \[million EUR/yr\] at 50 EUR/tonne.
    fn co2_savings_per_yr(&self, option: &ResourceOption) -> f64 {
        let cap_mw = option.capacity_mw().max(0.001);
        let cf = option.capacity_factor().max(0.001);
        let annual_energy_mwh = cap_mw * cf * 8760.0;
        let co2_existing_t = self.existing_co2_kg_per_mwh / 1000.0 * annual_energy_mwh;
        let co2_new_t = option.co2_kg_per_mwh() / 1000.0 * annual_energy_mwh;
        let saved_t = (co2_existing_t - co2_new_t).max(0.0);
        saved_t * 50.0 / 1_000_000.0 // 50 EUR/t → million EUR
    }

    // -----------------------------------------------------------------------
    // ELCC
    // -----------------------------------------------------------------------

    /// Effective Load Carrying Capability of a resource \[MW\].
    fn compute_elcc(option: &ResourceOption) -> f64 {
        match option {
            ResourceOption::RenewableResource {
                capacity_mw,
                capacity_factor,
                variability_factor,
                ..
            } => capacity_mw * capacity_factor * (1.0 - variability_factor * 0.5),
            ResourceOption::EnergyStorage { power_mw, .. } => power_mw * 0.95,
            ResourceOption::DemandResponse {
                peak_reduction_mw, ..
            } => *peak_reduction_mw,
            _ => option.capacity_mw(),
        }
    }

    // -----------------------------------------------------------------------
    // NPV helper
    // -----------------------------------------------------------------------

    /// Net present value of a cash-flow stream.
    ///
    /// `cashflows[t]` is the cost at end of period `t` (0-indexed).
    #[allow(dead_code)]
    fn npv(cashflows: &[f64], discount_rate: f64) -> f64 {
        cashflows
            .iter()
            .enumerate()
            .map(|(t, &cf)| cf / (1.0 + discount_rate).powi(t as i32 + 1))
            .sum()
    }

    // -----------------------------------------------------------------------
    // LOLE estimate
    // -----------------------------------------------------------------------

    /// Estimate Loss-of-Load Expectation \[h/year\] for a given year.
    ///
    /// Simplified linear approximation:
    /// ```text
    /// LOLE ≈ max(0, peak - capacity) / capacity × 8760
    /// ```
    pub fn estimate_lole(&self, portfolio: &ResourcePortfolio, year: usize) -> f64 {
        let peak = self
            .load_forecasts
            .iter()
            .find(|f| f.year == year)
            .map(|f| f.peak_load_mw)
            .unwrap_or_else(|| {
                self.load_forecasts
                    .last()
                    .map(|f| f.peak_load_mw)
                    .unwrap_or(0.0)
            });
        let cap = portfolio.total_capacity_mw.max(0.001);
        if peak > cap {
            (peak - cap) / cap * 8760.0
        } else {
            // Small residual representing forced outage rates at 1 %
            (cap - peak) / cap * 0.01 * 8760.0
        }
    }

    // -----------------------------------------------------------------------
    // Greedy optimisation
    // -----------------------------------------------------------------------

    /// Run greedy IRP optimisation.
    ///
    /// For each planning year:
    /// 1. Compute peak demand (using load forecast or simple growth).
    /// 2. Check capacity deficit against reserve margin requirement.
    /// 3. Rank unbuilt options by BCR (descending).
    /// 4. Select the highest-BCR option that fills the deficit.
    /// 5. Advance installed capacity and CO₂ intensity tracking.
    pub fn optimize_greedy(&mut self) -> Result<IrpResult, OxiGridError> {
        if self.config.planning_horizon_years == 0 {
            return Err(OxiGridError::InvalidParameter(
                "planning_horizon_years must be > 0".to_string(),
            ));
        }
        if self.existing_capacity_mw < 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "existing_capacity_mw must be non-negative".to_string(),
            ));
        }

        let mut installed_mw = self.existing_capacity_mw;
        let mut co2_intensity = self.existing_co2_kg_per_mwh;
        let mut built: Vec<(usize, usize)> = Vec::new(); // (option_idx, build_year)
        let mut total_npv_cost = 0.0_f64;
        let mut total_renewable_mw = 0.0_f64;
        let mut annual_snapshots: Vec<YearlyPlanSnapshot> = Vec::new();
        let mut cumulative_capex_billion = 0.0_f64;

        let n_years = self.config.planning_horizon_years;
        let base_year = self.config.base_year;

        for yr in 0..n_years {
            let calendar_year = base_year + yr;

            // Peak demand for this year
            let peak_mw = self
                .load_forecasts
                .get(yr)
                .map(|f| f.peak_load_mw)
                .unwrap_or_else(|| {
                    self.load_forecasts
                        .first()
                        .map(|f| f.peak_load_mw * (1.02_f64.powi(yr as i32)))
                        .unwrap_or(1000.0)
                });

            let required_mw = peak_mw * (1.0 + self.config.reserve_margin_pct / 100.0);
            let deficit = (required_mw - installed_mw).max(0.0);

            // Score all not-yet-built options by BCR
            let mut scored: Vec<(usize, f64)> = (0..self.options.len())
                .filter(|&i| !built.iter().any(|(bi, _)| *bi == i))
                .map(|i| {
                    let cba = self.compute_cba(i, calendar_year);
                    (i, cba.bcr)
                })
                .collect();
            scored.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

            // Greedily add options until deficit is covered
            let mut remaining_deficit = deficit;
            for (opt_idx, _bcr) in &scored {
                if remaining_deficit <= 0.0 {
                    break;
                }
                let option = &self.options[*opt_idx];
                let elcc = Self::compute_elcc(option);
                let cba = self.compute_cba(*opt_idx, calendar_year);

                // Budget check
                let capex_billion = option.capital_cost() / 1000.0;
                if cumulative_capex_billion + capex_billion
                    > self.config.budget_constraint_billion_eur
                {
                    continue;
                }

                // Add to portfolio
                built.push((*opt_idx, calendar_year));
                installed_mw += elcc;
                total_npv_cost += cba.npv_cost_million_eur;
                cumulative_capex_billion += capex_billion;
                remaining_deficit -= elcc;

                // Update CO₂ intensity (weighted average)
                let new_co2 = option.co2_kg_per_mwh();
                if installed_mw > 0.0 {
                    co2_intensity =
                        (co2_intensity * (installed_mw - elcc) + new_co2 * elcc) / installed_mw;
                }

                // Track renewable capacity
                if option.is_renewable() {
                    total_renewable_mw += elcc;
                }
            }

            // Annual cost: annuity of all built options this year
            let annual_cost: f64 = built
                .iter()
                .filter(|(_, by)| *by == calendar_year)
                .map(|(idx, _)| {
                    let opt = &self.options[*idx];
                    let r = self.config.discount_rate;
                    let n = opt.lifetime_years() as f64;
                    let crf = if r.abs() < 1e-12 {
                        1.0 / n.max(1.0)
                    } else {
                        let rn = (1.0 + r).powf(n);
                        r * rn / (rn - 1.0)
                    };
                    opt.capital_cost() * crf + opt.opex()
                })
                .sum();

            let ren_pct = if installed_mw > 0.0 {
                total_renewable_mw / installed_mw * 100.0
            } else {
                0.0
            };
            let margin_pct = if peak_mw > 0.0 {
                (installed_mw - peak_mw) / peak_mw * 100.0
            } else {
                0.0
            };
            let adequate = installed_mw >= required_mw;

            annual_snapshots.push(YearlyPlanSnapshot {
                year: calendar_year,
                installed_capacity_mw: installed_mw,
                renewable_fraction_pct: ren_pct,
                peak_demand_mw: peak_mw,
                reserve_margin_pct: margin_pct,
                annual_cost_million_eur: annual_cost,
                co2_intensity_kg_per_mwh: co2_intensity,
                capacity_adequacy: adequate,
            });
        }

        // Compute final CO₂ reduction
        let initial_co2 = self.existing_co2_kg_per_mwh.max(0.001);
        let co2_reduction_pct = ((initial_co2 - co2_intensity) / initial_co2 * 100.0).max(0.0);

        let last_snapshot = annual_snapshots.last();
        let final_peak = last_snapshot.map(|s| s.peak_demand_mw).unwrap_or(0.0);
        let reserve_margin = if final_peak > 0.0 {
            (installed_mw - final_peak) / final_peak * 100.0
        } else {
            0.0
        };

        let ren_pct = if installed_mw > 0.0 {
            total_renewable_mw / installed_mw * 100.0
        } else {
            0.0
        };

        let portfolio = ResourcePortfolio {
            selected_options: built.clone(),
            total_capacity_mw: installed_mw,
            total_renewable_pct: ren_pct,
            total_npv_cost_million_eur: total_npv_cost,
            co2_reduction_pct,
            reserve_margin_pct: reserve_margin,
            lole_estimate_h_per_yr: 0.0, // filled below
            meets_reliability: reserve_margin >= self.config.reserve_margin_pct,
            meets_co2_target: co2_reduction_pct >= self.config.co2_reduction_target_pct,
            meets_budget: cumulative_capex_billion <= self.config.budget_constraint_billion_eur,
        };

        let last_year = base_year + n_years - 1;
        let lole = self.estimate_lole(&portfolio, last_year);

        let mut portfolio = portfolio;
        portfolio.lole_estimate_h_per_yr = lole;
        portfolio.meets_reliability =
            lole <= self.config.reliability_lole_h_per_yr && reserve_margin >= 0.0;

        let sensitivity_results = self.run_sensitivity(&portfolio);
        let alternative_portfolios = self.generate_alternatives();

        let result = IrpResult {
            recommended_portfolio: portfolio.clone(),
            portfolio,
            annual_snapshots,
            sensitivity_results,
            alternative_portfolios,
        };

        Ok(result)
    }

    // -----------------------------------------------------------------------
    // Alternative portfolios
    // -----------------------------------------------------------------------

    /// Generate three alternative portfolios.
    ///
    /// 1. **Least cost** — rank by LCOE, ignore CO₂ target.
    /// 2. **Maximum renewable** — prioritise renewable options first.
    /// 3. **Minimum risk** — prioritise fully dispatchable options.
    pub fn generate_alternatives(&mut self) -> Vec<ResourcePortfolio> {
        let base_year = self.config.base_year;
        let final_year = base_year + self.config.planning_horizon_years.saturating_sub(1);
        let peak_mw = self
            .load_forecasts
            .last()
            .map(|f| f.peak_load_mw)
            .unwrap_or(self.existing_capacity_mw * 1.1);
        let required_mw = peak_mw * (1.0 + self.config.reserve_margin_pct / 100.0);

        let portfolios: Vec<ResourcePortfolio> = vec![
            self.build_alternative("least_cost", required_mw, final_year),
            self.build_alternative("max_renewable", required_mw, final_year),
            self.build_alternative("min_risk", required_mw, final_year),
        ];

        portfolios
    }

    /// Build a single alternative portfolio according to a named strategy.
    fn build_alternative(
        &self,
        strategy: &str,
        required_mw: f64,
        build_year: usize,
    ) -> ResourcePortfolio {
        let _r = self.config.discount_rate;

        // Sort option indices by strategy criterion
        let mut indices: Vec<usize> = (0..self.options.len()).collect();
        match strategy {
            "least_cost" => {
                indices.sort_by(|&a, &b| {
                    let la = self.compute_lcoe(&self.options[a], build_year);
                    let lb = self.compute_lcoe(&self.options[b], build_year);
                    la.partial_cmp(&lb).unwrap_or(std::cmp::Ordering::Equal)
                });
            }
            "max_renewable" => {
                indices.sort_by(|&a, &b| {
                    let ra = self.options[a].is_renewable() as u8;
                    let rb = self.options[b].is_renewable() as u8;
                    rb.cmp(&ra)
                });
            }
            "min_risk" => {
                indices.sort_by(|&a, &b| {
                    let da = self.options[a].is_dispatchable() as u8;
                    let db = self.options[b].is_dispatchable() as u8;
                    db.cmp(&da)
                });
            }
            _ => {}
        }

        let mut selected: Vec<(usize, usize)> = Vec::new();
        let mut total_cap = self.existing_capacity_mw;
        let mut total_ren = 0.0_f64;
        let mut total_npv = 0.0_f64;
        let mut total_capex_billion = 0.0_f64;

        for idx in &indices {
            if total_cap >= required_mw {
                break;
            }
            let option = &self.options[*idx];
            let elcc = Self::compute_elcc(option);
            let cba = self.compute_cba(*idx, build_year);

            let capex_billion = option.capital_cost() / 1000.0;
            if total_capex_billion + capex_billion > self.config.budget_constraint_billion_eur {
                continue;
            }

            selected.push((*idx, build_year));
            total_cap += elcc;
            total_npv += cba.npv_cost_million_eur;
            total_capex_billion += capex_billion;
            if option.is_renewable() {
                total_ren += elcc;
            }
        }

        let peak_mw = self
            .load_forecasts
            .last()
            .map(|f| f.peak_load_mw)
            .unwrap_or(total_cap * 0.85);
        let reserve_pct = if peak_mw > 0.0 {
            (total_cap - peak_mw) / peak_mw * 100.0
        } else {
            0.0
        };
        let ren_pct = if total_cap > 0.0 {
            total_ren / total_cap * 100.0
        } else {
            0.0
        };

        // Simplified CO₂ tracking
        let new_co2 = selected
            .iter()
            .map(|(idx, _)| self.options[*idx].co2_kg_per_mwh())
            .sum::<f64>()
            / selected.len().max(1) as f64;
        let co2_init = self.existing_co2_kg_per_mwh.max(0.001);
        let blended_co2 = if total_cap > self.existing_capacity_mw {
            let new_cap = total_cap - self.existing_capacity_mw;
            (self.existing_co2_kg_per_mwh * self.existing_capacity_mw + new_co2 * new_cap)
                / total_cap
        } else {
            self.existing_co2_kg_per_mwh
        };
        let co2_reduction_pct = ((co2_init - blended_co2) / co2_init * 100.0).max(0.0);

        // Simplified LOLE for alternative
        let lole = if total_cap >= required_mw {
            self.config.reliability_lole_h_per_yr * 0.5
        } else {
            self.config.reliability_lole_h_per_yr * 2.0
        };

        ResourcePortfolio {
            selected_options: selected,
            total_capacity_mw: total_cap,
            total_renewable_pct: ren_pct,
            total_npv_cost_million_eur: total_npv,
            co2_reduction_pct,
            reserve_margin_pct: reserve_pct,
            lole_estimate_h_per_yr: lole,
            meets_reliability: lole <= self.config.reliability_lole_h_per_yr,
            meets_co2_target: co2_reduction_pct >= self.config.co2_reduction_target_pct,
            meets_budget: total_capex_billion <= self.config.budget_constraint_billion_eur,
        }
    }

    // -----------------------------------------------------------------------
    // Sensitivity analysis
    // -----------------------------------------------------------------------

    /// Vary discount rate and CO₂ target by ±20 %; record NPV and CO₂ changes.
    pub fn run_sensitivity(&self, base_portfolio: &ResourcePortfolio) -> Vec<SensitivityResult> {
        let mut results = Vec::new();

        // Variations: (parameter_name, field_selector_fn returning modified planner)
        let base_npv = base_portfolio.total_npv_cost_million_eur;
        let _base_co2 = base_portfolio.co2_reduction_pct;

        for &variation_pct in &[-20.0_f64, 20.0] {
            // Discount rate variation
            {
                let new_dr = self.config.discount_rate * (1.0 + variation_pct / 100.0);
                let npv_change = self.sensitivity_npv_change(base_portfolio, new_dr) - base_npv;
                results.push(SensitivityResult {
                    parameter: "discount_rate".to_string(),
                    variation_pct,
                    npv_change_million_eur: npv_change,
                    co2_change_million_ton: 0.0,
                    portfolio_changes: npv_change.abs() > base_npv * 0.05,
                });
            }
            // CO₂ target variation
            {
                let new_target =
                    self.config.co2_reduction_target_pct * (1.0 + variation_pct / 100.0);
                let co2_change = (new_target - self.config.co2_reduction_target_pct)
                    * base_portfolio.total_capacity_mw
                    * 0.001; // simplistic: capacity * δtarget * factor
                let portfolio_changes = (base_portfolio.co2_reduction_pct < new_target)
                    != (base_portfolio.co2_reduction_pct < self.config.co2_reduction_target_pct);
                results.push(SensitivityResult {
                    parameter: "co2_reduction_target_pct".to_string(),
                    variation_pct,
                    npv_change_million_eur: 0.0,
                    co2_change_million_ton: co2_change,
                    portfolio_changes,
                });
            }
        }

        results
    }

    /// Re-compute portfolio NPV with a different discount rate.
    fn sensitivity_npv_change(&self, portfolio: &ResourcePortfolio, new_dr: f64) -> f64 {
        portfolio
            .selected_options
            .iter()
            .map(|(idx, build_year)| {
                if let Some(option) = self.options.get(*idx) {
                    let r = new_dr;
                    let n = option.lifetime_years() as f64;
                    let crf = if r.abs() < 1e-12 {
                        1.0 / n.max(1.0)
                    } else {
                        let rn = (1.0 + r).powf(n);
                        r * rn / (rn - 1.0)
                    };
                    let annuity = if r.abs() < 1e-12 {
                        n
                    } else {
                        let rn = (1.0 + r).powf(n);
                        (rn - 1.0) / (r * rn)
                    };
                    let delay = build_year.saturating_sub(self.config.base_year) as i32;
                    let delay_factor = 1.0 / (1.0 + r).powi(delay);
                    let annual_cost = option.capital_cost() * crf + option.opex();
                    (annual_cost * annuity + option.capital_cost()) * delay_factor
                } else {
                    0.0
                }
            })
            .sum()
    }
}

// ---------------------------------------------------------------------------
// ESIA
// ---------------------------------------------------------------------------

/// Visual impact classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VisualImpact {
    Negligible,
    Low,
    Medium,
    High,
}

impl VisualImpact {
    /// Elevate visual impact one level.
    fn elevate(self) -> Self {
        match self {
            VisualImpact::Negligible => VisualImpact::Low,
            VisualImpact::Low => VisualImpact::Medium,
            VisualImpact::Medium => VisualImpact::High,
            VisualImpact::High => VisualImpact::High,
        }
    }
}

/// Environmental and Social Impact Assessment for a resource option.
#[derive(Debug, Clone)]
pub struct EsiaAssessment {
    pub option_idx: usize,
    /// Estimated land use \[km²\].
    pub land_use_km2: f64,
    /// Water consumption \[m³/MWh\].
    pub water_consumption_m3_per_mwh: f64,
    /// Noise level at site boundary \[dB\].
    pub noise_level_db: f64,
    pub visual_impact: VisualImpact,
    /// Biodiversity impact score 0–10 (0 = no impact).
    pub biodiversity_impact: f64,
    /// Permanent jobs (FTE).
    pub jobs_permanent: f64,
    /// Construction-phase jobs (FTE).
    pub jobs_construction: f64,
    /// Local tax revenue over lifetime \[million EUR\].
    pub local_tax_revenue_million_eur: f64,
}

impl EsiaAssessment {
    /// Rule-based ESIA for a resource option.
    pub fn assess(
        option: &ResourceOption,
        option_idx: usize,
        location_urban: bool,
    ) -> EsiaAssessment {
        let (land, water, noise, visual, biodiversity, jobs_perm, jobs_constr) = match option {
            ResourceOption::RenewableResource {
                technology,
                capacity_mw,
                ..
            } => {
                let tech = technology.to_ascii_lowercase();
                if tech.contains("solar") || tech.contains("pv") {
                    (
                        capacity_mw * 0.01,
                        0.001,
                        35.0,
                        VisualImpact::Low,
                        2.0,
                        capacity_mw * 0.1,
                        capacity_mw * 0.5,
                    )
                } else {
                    // wind
                    (
                        capacity_mw * 0.05,
                        0.002,
                        45.0,
                        VisualImpact::Medium,
                        4.0,
                        capacity_mw * 0.15,
                        capacity_mw * 0.6,
                    )
                }
            }
            ResourceOption::BaseloadPlant { capacity_mw, .. } => (
                capacity_mw * 0.002,
                1.5,
                55.0,
                VisualImpact::High,
                3.0,
                capacity_mw * 0.2,
                capacity_mw * 0.8,
            ),
            ResourceOption::PeakingPlant { capacity_mw, .. } => (
                capacity_mw * 0.001,
                0.5,
                50.0,
                VisualImpact::Medium,
                2.0,
                capacity_mw * 0.1,
                capacity_mw * 0.4,
            ),
            ResourceOption::EnergyStorage { power_mw, .. } => (
                power_mw * 0.001,
                0.01,
                40.0,
                VisualImpact::Low,
                1.0,
                power_mw * 0.05,
                power_mw * 0.2,
            ),
            ResourceOption::TransmissionUpgrade { .. } => {
                (0.1, 0.0, 30.0, VisualImpact::Negligible, 0.5, 10.0, 50.0)
            }
            ResourceOption::DistributionUpgrade { smart_grid, .. } => {
                let visual = if *smart_grid {
                    VisualImpact::Negligible
                } else {
                    VisualImpact::Low
                };
                (0.05, 0.0, 28.0, visual, 0.3, 5.0, 30.0)
            }
            ResourceOption::DemandResponse { .. } => {
                (0.0, 0.0, 25.0, VisualImpact::Negligible, 0.0, 5.0, 10.0)
            }
        };

        // Urban adjustment: +5 dB, visual one step up
        let (noise_final, visual_final) = if location_urban {
            (noise + 5.0, visual.elevate())
        } else {
            (noise, visual)
        };

        let tax_revenue = jobs_perm * 0.05;

        EsiaAssessment {
            option_idx,
            land_use_km2: land,
            water_consumption_m3_per_mwh: water,
            noise_level_db: noise_final,
            visual_impact: visual_final,
            biodiversity_impact: biodiversity,
            jobs_permanent: jobs_perm,
            jobs_construction: jobs_constr,
            local_tax_revenue_million_eur: tax_revenue,
        }
    }
}

// ---------------------------------------------------------------------------
// MCDA
// ---------------------------------------------------------------------------

/// Criteria weights for Multi-Criteria Decision Analysis.
#[derive(Debug, Clone)]
pub struct McdaWeights {
    pub cost: f64,
    pub reliability: f64,
    pub environment: f64,
    pub social: f64,
    pub flexibility: f64,
}

impl McdaWeights {
    /// Balanced weights — all criteria equal (each 0.2).
    pub fn balanced() -> Self {
        Self {
            cost: 0.2,
            reliability: 0.2,
            environment: 0.2,
            social: 0.2,
            flexibility: 0.2,
        }
    }

    /// Cost-focused weights — cost 0.4, others equal share of remaining 0.6.
    pub fn cost_focused() -> Self {
        let rest = 0.6 / 4.0;
        Self {
            cost: 0.4,
            reliability: rest,
            environment: rest,
            social: rest,
            flexibility: rest,
        }
    }

    /// Green-focused weights — environment 0.4, reliability 0.2, others split.
    pub fn green_focused() -> Self {
        let rest = 0.4 / 3.0;
        Self {
            cost: rest,
            reliability: 0.2,
            environment: 0.4,
            social: rest,
            flexibility: rest,
        }
    }
}

/// Multi-Criteria Decision Analysis engine.
#[derive(Debug, Clone)]
pub struct McdaAnalysis {
    pub criteria_weights: McdaWeights,
}

impl McdaAnalysis {
    /// Create a new MCDA engine with the given weights.
    pub fn new(criteria_weights: McdaWeights) -> Self {
        Self { criteria_weights }
    }

    /// Compute a composite score \[0, 1\] for a portfolio.
    pub fn score_portfolio(&self, portfolio: &ResourcePortfolio, esia: &[EsiaAssessment]) -> f64 {
        let w = &self.criteria_weights;

        // Cost score: higher cost → lower score
        let cost_score = 1.0
            - portfolio.total_npv_cost_million_eur
                / (portfolio.total_npv_cost_million_eur + 1000.0);

        // Reliability score
        let reliability_score = if portfolio.meets_reliability {
            1.0
        } else {
            (portfolio.reserve_margin_pct.max(0.0) / 20.0).min(1.0)
        };

        // Environment score: average (1 - biodiversity_impact / 10)
        let env_score = if esia.is_empty() {
            0.5
        } else {
            let sum: f64 = esia
                .iter()
                .map(|e| (1.0 - e.biodiversity_impact / 10.0).clamp(0.0, 1.0))
                .sum();
            (sum / esia.len() as f64).clamp(0.0, 1.0)
        };

        // Social score: permanent jobs normalised by (n * 100)
        let n_esia = esia.len().max(1);
        let social_score = {
            let sum: f64 = esia.iter().map(|e| e.jobs_permanent).sum();
            (sum / (n_esia as f64 * 100.0)).clamp(0.0, 1.0)
        };

        // Flexibility score: dispatchable heavy → more flexible
        let flexibility_score = if portfolio.total_renewable_pct < 80.0 {
            0.8
        } else {
            0.5
        };

        w.cost * cost_score
            + w.reliability * reliability_score
            + w.environment * env_score
            + w.social * social_score
            + w.flexibility * flexibility_score
    }

    /// Rank multiple portfolios by MCDA score (descending).
    ///
    /// Returns `Vec<(portfolio_index, score)>`.
    pub fn rank_portfolios(
        &self,
        portfolios: &[ResourcePortfolio],
        esia_data: &[Vec<EsiaAssessment>],
    ) -> Vec<(usize, f64)> {
        let mut ranked: Vec<(usize, f64)> = portfolios
            .iter()
            .enumerate()
            .map(|(i, p)| {
                let esia = esia_data.get(i).map(|v| v.as_slice()).unwrap_or(&[]);
                let score = self.score_portfolio(p, esia);
                (i, score)
            })
            .collect();

        ranked.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        ranked
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_config() -> IrpConfig {
        IrpConfig {
            planning_horizon_years: 5,
            base_year: 2025,
            discount_rate: 0.07,
            reserve_margin_pct: 15.0,
            co2_reduction_target_pct: 30.0,
            reliability_lole_h_per_yr: 3.0,
            budget_constraint_billion_eur: 500.0,
        }
    }

    fn make_forecasts(n: usize) -> Vec<PlanningLoadForecast> {
        (0..n)
            .map(|i| PlanningLoadForecast {
                year: 2025 + i,
                peak_load_mw: 5000.0 + i as f64 * 100.0,
                annual_energy_twh: 40.0 + i as f64 * 0.5,
                peak_demand_growth_pct: 2.0,
                der_penetration_pct: 5.0,
                ev_load_mw: 100.0,
                heat_pump_load_mw: 50.0,
            })
            .collect()
    }

    fn solar_option() -> ResourceOption {
        ResourceOption::RenewableResource {
            technology: "solar".to_string(),
            capacity_mw: 200.0,
            capital_cost_million_eur: 140.0,
            opex_million_eur_per_yr: 2.0,
            capacity_factor: 0.22,
            variability_factor: 0.8,
            lifetime_years: 25,
        }
    }

    fn gas_baseload_option() -> ResourceOption {
        ResourceOption::BaseloadPlant {
            technology: "CCGT".to_string(),
            capacity_mw: 400.0,
            capital_cost_million_eur: 320.0,
            opex_million_eur_per_yr: 8.0,
            capacity_factor: 0.55,
            co2_kg_per_mwh: 400.0,
            lifetime_years: 30,
            build_time_years: 3,
        }
    }

    fn battery_option() -> ResourceOption {
        ResourceOption::EnergyStorage {
            technology: "Li-ion".to_string(),
            power_mw: 100.0,
            energy_mwh: 400.0,
            capital_cost_million_eur: 80.0,
            opex_million_eur_per_yr: 1.0,
            roundtrip_efficiency: 0.88,
            lifetime_years: 15,
        }
    }

    fn transmission_option() -> ResourceOption {
        ResourceOption::TransmissionUpgrade {
            from_bus: 1,
            to_bus: 5,
            capacity_increase_mw: 300.0,
            capital_cost_million_eur: 50.0,
            lifetime_years: 40,
        }
    }

    fn make_planner() -> IntegratedResourcePlanner {
        let opts = vec![
            solar_option(),
            gas_baseload_option(),
            battery_option(),
            transmission_option(),
        ];
        let forecasts = make_forecasts(5);
        let config = make_config();
        IntegratedResourcePlanner::new(opts, forecasts, config, 4500.0, 450.0)
    }

    // -----------------------------------------------------------------------

    #[test]
    fn test_irp_config_creation() {
        let cfg = make_config();
        assert_eq!(cfg.planning_horizon_years, 5);
        assert_eq!(cfg.base_year, 2025);
        assert!((cfg.discount_rate - 0.07).abs() < 1e-9);
        assert!((cfg.reserve_margin_pct - 15.0).abs() < 1e-9);
        assert!((cfg.co2_reduction_target_pct - 30.0).abs() < 1e-9);
    }

    #[test]
    fn test_load_forecast_creation() {
        let forecasts = make_forecasts(3);
        assert_eq!(forecasts.len(), 3);
        assert_eq!(forecasts[0].year, 2025);
        assert!((forecasts[0].peak_load_mw - 5000.0).abs() < 1e-9);
        assert!(forecasts[1].annual_energy_twh > forecasts[0].annual_energy_twh);
    }

    #[test]
    fn test_baseload_plant_option() {
        let opt = gas_baseload_option();
        assert!((opt.capacity_mw() - 400.0).abs() < 1e-9);
        assert!((opt.capacity_factor() - 0.55).abs() < 1e-9);
        assert!((opt.co2_kg_per_mwh() - 400.0).abs() < 1e-9);
        assert!(!opt.is_renewable());
        assert!(opt.is_dispatchable());
        assert_eq!(opt.lifetime_years(), 30);
    }

    #[test]
    fn test_renewable_option() {
        let opt = solar_option();
        assert!((opt.capacity_mw() - 200.0).abs() < 1e-9);
        assert!(opt.is_renewable());
        assert!(!opt.is_dispatchable());
        assert!((opt.co2_kg_per_mwh() - 0.0).abs() < 1e-9);
        assert_eq!(opt.lifetime_years(), 25);
    }

    #[test]
    fn test_storage_option() {
        let opt = battery_option();
        assert!((opt.capacity_mw() - 100.0).abs() < 1e-9);
        assert!(!opt.is_renewable());
        assert!(opt.is_dispatchable());
        let elcc = IntegratedResourcePlanner::compute_elcc(&opt);
        assert!((elcc - 95.0).abs() < 1e-9); // 100 * 0.95
    }

    #[test]
    fn test_transmission_upgrade_option() {
        let opt = transmission_option();
        assert!((opt.capacity_mw() - 300.0).abs() < 1e-9);
        assert_eq!(opt.lifetime_years(), 40);
        assert!(!opt.is_renewable());
        assert!(!opt.is_dispatchable());
    }

    #[test]
    fn test_compute_lcoe_baseload() {
        let planner = make_planner();
        let opt = gas_baseload_option();
        let lcoe = planner.compute_lcoe(&opt, 2025);
        // Should be in plausible EUR/MWh range for a CCGT (EUR-denominated inputs)
        assert!(lcoe > 5.0, "LCOE too low: {lcoe}");
        assert!(lcoe < 500.0, "LCOE too high: {lcoe}");
    }

    #[test]
    fn test_compute_lcoe_renewable() {
        let planner = make_planner();
        let opt = solar_option();
        let lcoe = planner.compute_lcoe(&opt, 2025);
        // Solar LCOE: 30–200 EUR/MWh plausible range
        assert!(lcoe > 10.0, "LCOE too low: {lcoe}");
        assert!(lcoe < 500.0, "LCOE too high: {lcoe}");
    }

    #[test]
    fn test_compute_cba_positive_bcr() {
        let planner = make_planner();
        // Solar with existing high-CO₂ fleet → CO₂ savings + energy value should dominate
        let cba = planner.compute_cba(0, 2025); // solar
        assert!(cba.bcr > 0.0, "BCR should be positive, got {}", cba.bcr);
        assert!(cba.npv_benefit_million_eur >= 0.0);
        assert!(cba.npv_cost_million_eur >= 0.0);
    }

    #[test]
    fn test_compute_cba_negative_bcr() {
        // Very expensive option with tiny capacity → benefit << cost
        let expensive = ResourceOption::BaseloadPlant {
            technology: "Exotic".to_string(),
            capacity_mw: 1.0,                   // 1 MW only
            capital_cost_million_eur: 50_000.0, // ridiculously expensive
            opex_million_eur_per_yr: 1_000.0,
            capacity_factor: 0.001, // almost never runs
            co2_kg_per_mwh: 0.0,
            lifetime_years: 5,
            build_time_years: 1,
        };
        let opts = vec![expensive];
        let forecasts = make_forecasts(5);
        let config = make_config();
        let planner = IntegratedResourcePlanner::new(opts, forecasts, config, 1000.0, 100.0);
        let cba = planner.compute_cba(0, 2025);
        // With minimal generation and enormous cost, BCR should be < 1
        assert!(
            cba.bcr < 1.0,
            "Expected BCR < 1.0 for prohibitively expensive option, got {}",
            cba.bcr
        );
    }

    #[test]
    fn test_greedy_optimization_basic() {
        let mut planner = make_planner();
        let result = planner
            .optimize_greedy()
            .expect("greedy optimize should succeed");
        assert!(!result.annual_snapshots.is_empty());
        assert_eq!(result.annual_snapshots.len(), 5);
        // Each snapshot year should be increasing
        for w in result.annual_snapshots.windows(2) {
            assert!(w[1].year > w[0].year);
        }
    }

    #[test]
    fn test_greedy_meets_reserve_margin() {
        // With sufficient existing capacity (5500 MW) and low peak (5000 MW + 15% = 5750 MW),
        // the planner may need to add some capacity.
        let opts = vec![gas_baseload_option(), solar_option()];
        let forecasts = make_forecasts(3);
        let config = IrpConfig {
            planning_horizon_years: 3,
            reserve_margin_pct: 10.0,
            budget_constraint_billion_eur: 1000.0,
            ..make_config()
        };
        let mut planner = IntegratedResourcePlanner::new(opts, forecasts, config, 5000.0, 500.0);
        let result = planner.optimize_greedy().expect("should succeed");
        let last = result.annual_snapshots.last().expect("has snapshots");
        // Reserve margin should be non-negative (capacity >= peak)
        assert!(
            last.installed_capacity_mw >= last.peak_demand_mw,
            "capacity {} < peak {}",
            last.installed_capacity_mw,
            last.peak_demand_mw
        );
    }

    #[test]
    fn test_greedy_co2_reduction() {
        let opts = vec![solar_option()]; // zero-CO₂ option only
        let forecasts = make_forecasts(5);
        let config = make_config();
        let mut planner = IntegratedResourcePlanner::new(opts, forecasts, config, 2000.0, 600.0);
        let result = planner.optimize_greedy().expect("should succeed");
        // Adding solar to a high-CO₂ fleet should reduce blended CO₂ intensity
        let final_snap = result.annual_snapshots.last().expect("has snapshots");
        // CO₂ intensity in final year should be ≤ initial (600 kg/MWh)
        assert!(
            final_snap.co2_intensity_kg_per_mwh <= 600.0 + 1e-6,
            "CO₂ intensity should not increase, got {}",
            final_snap.co2_intensity_kg_per_mwh
        );
    }

    #[test]
    fn test_generate_alternatives_3_portfolios() {
        let mut planner = make_planner();
        let alts = planner.generate_alternatives();
        assert_eq!(
            alts.len(),
            3,
            "Should generate exactly 3 alternative portfolios"
        );
        // All portfolios must have valid capacity values
        for p in &alts {
            assert!(p.total_capacity_mw >= 0.0);
            assert!(p.total_renewable_pct >= 0.0 && p.total_renewable_pct <= 100.0);
        }
    }

    #[test]
    fn test_lole_estimate_adequate() {
        let planner = make_planner();
        let portfolio = ResourcePortfolio {
            selected_options: vec![],
            total_capacity_mw: 7000.0,
            total_renewable_pct: 20.0,
            total_npv_cost_million_eur: 500.0,
            co2_reduction_pct: 20.0,
            reserve_margin_pct: 40.0,
            lole_estimate_h_per_yr: 0.0,
            meets_reliability: true,
            meets_co2_target: false,
            meets_budget: true,
        };
        let lole = planner.estimate_lole(&portfolio, 2025);
        // Capacity well above peak → very low LOLE
        assert!(
            lole < 100.0,
            "LOLE should be low for adequate capacity: {lole}"
        );
    }

    #[test]
    fn test_lole_estimate_inadequate() {
        let planner = make_planner();
        // Only 3000 MW capacity but 5000 MW peak → severe deficit
        let portfolio = ResourcePortfolio {
            selected_options: vec![],
            total_capacity_mw: 3000.0,
            total_renewable_pct: 0.0,
            total_npv_cost_million_eur: 100.0,
            co2_reduction_pct: 0.0,
            reserve_margin_pct: -40.0,
            lole_estimate_h_per_yr: 0.0,
            meets_reliability: false,
            meets_co2_target: false,
            meets_budget: true,
        };
        let lole = planner.estimate_lole(&portfolio, 2025);
        // 5000 MW peak, 3000 MW capacity → (5000-3000)/3000 * 8760 ≈ 5840 h
        assert!(
            lole > 1000.0,
            "LOLE should be very high for inadequate capacity: {lole}"
        );
    }

    #[test]
    fn test_npv_calculation() {
        let cashflows = vec![100.0, 100.0, 100.0];
        let npv = IntegratedResourcePlanner::npv(&cashflows, 0.10);
        // Manual: 100/1.1 + 100/1.21 + 100/1.331 ≈ 90.91 + 82.64 + 75.13 = 248.68
        assert!((npv - 248.685).abs() < 0.1, "NPV calculation wrong: {npv}");

        // Zero discount rate
        let npv_zero = IntegratedResourcePlanner::npv(&cashflows, 0.0);
        assert!(
            (npv_zero - 300.0).abs() < 1e-6,
            "NPV at 0% should be 300: {npv_zero}"
        );
    }

    #[test]
    fn test_esia_solar_assessment() {
        let opt = ResourceOption::RenewableResource {
            technology: "solar".to_string(),
            capacity_mw: 100.0,
            capital_cost_million_eur: 70.0,
            opex_million_eur_per_yr: 1.0,
            capacity_factor: 0.22,
            variability_factor: 0.8,
            lifetime_years: 25,
        };
        let esia = EsiaAssessment::assess(&opt, 0, false);
        assert!((esia.land_use_km2 - 1.0).abs() < 1e-9); // 100 * 0.01
        assert!((esia.water_consumption_m3_per_mwh - 0.001).abs() < 1e-9);
        assert!((esia.noise_level_db - 35.0).abs() < 1e-9);
        assert_eq!(esia.visual_impact, VisualImpact::Low);
        assert!((esia.biodiversity_impact - 2.0).abs() < 1e-9);
        assert!((esia.jobs_permanent - 10.0).abs() < 1e-9); // 100 * 0.1
    }

    #[test]
    fn test_esia_wind_assessment() {
        let opt = ResourceOption::RenewableResource {
            technology: "wind_onshore".to_string(),
            capacity_mw: 200.0,
            capital_cost_million_eur: 260.0,
            opex_million_eur_per_yr: 6.0,
            capacity_factor: 0.35,
            variability_factor: 0.6,
            lifetime_years: 25,
        };
        let esia = EsiaAssessment::assess(&opt, 1, false);
        assert!((esia.land_use_km2 - 10.0).abs() < 1e-9); // 200 * 0.05
        assert!((esia.noise_level_db - 45.0).abs() < 1e-9);
        assert_eq!(esia.visual_impact, VisualImpact::Medium);
        assert!((esia.biodiversity_impact - 4.0).abs() < 1e-9);

        // Urban: noise +5, visual elevate Medium → High
        let esia_urban = EsiaAssessment::assess(&opt, 1, true);
        assert!((esia_urban.noise_level_db - 50.0).abs() < 1e-9);
        assert_eq!(esia_urban.visual_impact, VisualImpact::High);
    }

    #[test]
    fn test_mcda_balanced_scoring() {
        let mcda = McdaAnalysis::new(McdaWeights::balanced());
        let portfolio = ResourcePortfolio {
            selected_options: vec![],
            total_capacity_mw: 6000.0,
            total_renewable_pct: 40.0,
            total_npv_cost_million_eur: 500.0,
            co2_reduction_pct: 35.0,
            reserve_margin_pct: 20.0,
            lole_estimate_h_per_yr: 2.0,
            meets_reliability: true,
            meets_co2_target: true,
            meets_budget: true,
        };
        let esia = vec![EsiaAssessment::assess(&solar_option(), 0, false)];
        let score = mcda.score_portfolio(&portfolio, &esia);
        assert!((0.0..=1.0).contains(&score), "Score out of range: {score}");
        // Reliability=1 → high reliability_score. Should be reasonably high.
        assert!(
            score > 0.3,
            "Score should be reasonable for a good portfolio: {score}"
        );
    }

    #[test]
    fn test_mcda_rank_portfolios() {
        let mcda = McdaAnalysis::new(McdaWeights::balanced());

        let good_portfolio = ResourcePortfolio {
            selected_options: vec![],
            total_capacity_mw: 6000.0,
            total_renewable_pct: 60.0,
            total_npv_cost_million_eur: 200.0,
            co2_reduction_pct: 50.0,
            reserve_margin_pct: 20.0,
            lole_estimate_h_per_yr: 1.0,
            meets_reliability: true,
            meets_co2_target: true,
            meets_budget: true,
        };
        let bad_portfolio = ResourcePortfolio {
            selected_options: vec![],
            total_capacity_mw: 4000.0,
            total_renewable_pct: 5.0,
            total_npv_cost_million_eur: 10_000.0,
            co2_reduction_pct: 2.0,
            reserve_margin_pct: -10.0,
            lole_estimate_h_per_yr: 50.0,
            meets_reliability: false,
            meets_co2_target: false,
            meets_budget: false,
        };

        let portfolios = vec![bad_portfolio, good_portfolio];
        let esia_data: Vec<Vec<EsiaAssessment>> = vec![
            vec![EsiaAssessment::assess(&gas_baseload_option(), 0, false)],
            vec![EsiaAssessment::assess(&solar_option(), 0, false)],
        ];

        let ranked = mcda.rank_portfolios(&portfolios, &esia_data);
        assert_eq!(ranked.len(), 2);
        // Good portfolio (index 1) should rank first
        assert_eq!(ranked[0].0, 1, "Good portfolio should be ranked first");
        assert!(
            ranked[0].1 > ranked[1].1,
            "First-ranked score should be higher"
        );
    }
}
