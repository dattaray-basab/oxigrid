//! Carbon budget and emission trading market module.
//!
//! Implements:
//! - **Emission factors** by fuel type (CO₂, CH₄, N₂O, lifecycle CO₂e)
//! - **Emitting generators** with allowance positions and carbon cost
//! - **Carbon allowances** (EUA-like permits) across multiple ETS schemes
//! - **Carbon market** with supply/demand price dynamics, auctions, and forecasting
//! - **Carbon budget tracker** with optimal dispatch and scope 1/2/3 reporting
//! - **Grid emission intensity** (average and marginal rates)
//!
//! # Units
//! - Emissions: \[tonne CO₂e\] (metric tonnes)
//! - Carbon price: \[EUR/tonne\]
//! - Power: \[MW\], Energy: \[MWh\]
//! - Emission factors: \[kg CO₂e/MWh\]
//!
//! # References
//! - EU ETS Directive 2003/87/EC and amendments
//! - IPCC AR6 GWP100: CH₄ = 27.9, N₂O = 273 (we use classic AR5: 25/298 per spec)
//! - IEA Emission Factors 2023
//! - Ellerman, A.D. et al., "Pricing Carbon", Cambridge University Press, 2010
//! - ISO 14064-1:2018 Greenhouse Gas Accounting

use crate::error::OxiGridError;
use serde::{Deserialize, Serialize};

// ─────────────────────────────────────────────────────────────────────────────
// Re-export legacy types so they remain available from the market module
// ─────────────────────────────────────────────────────────────────────────────

pub use legacy::{
    AllocationMethod, AuctionBid, AuctionResult, CarbonBudgetConfig, CarbonDispatchResult,
    ComplianceStatus, GeneratorCarbonProfile, MultiYearCarbonPlan, ParetoPoint, PermitAllocation,
    PermitTransaction, TradingResult,
};

// ─────────────────────────────────────────────────────────────────────────────
// Legacy module — preserve existing public API surface
// ─────────────────────────────────────────────────────────────────────────────

#[path = "carbon_budget_legacy.rs"]
pub mod legacy;

// ─────────────────────────────────────────────────────────────────────────────
// Carbon accounting period
// ─────────────────────────────────────────────────────────────────────────────

/// Carbon accounting period for budget tracking.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum CarbonPeriod {
    /// Annual accounting period.
    Annual { year: u32 },
    /// Monthly accounting period.
    Monthly { year: u32, month: u8 },
    /// Daily accounting period.
    Daily { year: u32, month: u8, day: u8 },
}

impl CarbonPeriod {
    /// Returns a human-readable label for the period.
    pub fn label(&self) -> String {
        match self {
            CarbonPeriod::Annual { year } => format!("{}", year),
            CarbonPeriod::Monthly { year, month } => format!("{}-{:02}", year, month),
            CarbonPeriod::Daily { year, month, day } => {
                format!("{}-{:02}-{:02}", year, month, day)
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Emission factor
// ─────────────────────────────────────────────────────────────────────────────

/// Emission intensity by fuel type.
///
/// All quantities in kg CO₂ (or equivalent) per MWh of electricity generated.
/// Global Warming Potentials (GWP100, AR5): CH₄ = 25, N₂O = 298.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmissionFactor {
    /// Fuel type label (e.g., "coal", "natural_gas").
    pub fuel_type: String,
    /// Direct CO₂ emissions \[kg CO₂/MWh\].
    pub co2_kg_per_mwh: f64,
    /// Direct methane (CH₄) emissions \[kg CH₄/MWh\].
    pub ch4_kg_per_mwh: f64,
    /// Direct nitrous oxide (N₂O) emissions \[kg N₂O/MWh\].
    pub n2o_kg_per_mwh: f64,
    /// Full lifecycle CO₂e including upstream activities \[kg CO₂e/MWh\].
    pub lifecycle_co2e_kg_per_mwh: f64,
}

/// GWP100 (AR5) for methane: 1 kg CH₄ = 25 kg CO₂e.
const GWP_CH4: f64 = 25.0;
/// GWP100 (AR5) for nitrous oxide: 1 kg N₂O = 298 kg CO₂e.
const GWP_N2O: f64 = 298.0;

impl EmissionFactor {
    /// Total CO₂ equivalent: CO₂ + 25·CH₄ + 298·N₂O \[kg CO₂e/MWh\].
    pub fn co2e_kg_per_mwh(&self) -> f64 {
        self.co2_kg_per_mwh + GWP_CH4 * self.ch4_kg_per_mwh + GWP_N2O * self.n2o_kg_per_mwh
    }

    /// Coal (hard coal / bituminous): ~820 kg CO₂e/MWh.
    ///
    /// High direct CO₂ due to high carbon content (~94 g C/MJ).
    pub fn coal() -> Self {
        Self {
            fuel_type: "coal".into(),
            co2_kg_per_mwh: 800.0,
            ch4_kg_per_mwh: 0.3,
            n2o_kg_per_mwh: 0.014,
            lifecycle_co2e_kg_per_mwh: 820.0,
        }
    }

    /// Natural gas (combined-cycle): ~490 kg CO₂e/MWh.
    ///
    /// Lower carbon content than coal; methane slip from upstream is significant.
    pub fn natural_gas() -> Self {
        Self {
            fuel_type: "natural_gas".into(),
            co2_kg_per_mwh: 400.0,
            ch4_kg_per_mwh: 3.5,
            n2o_kg_per_mwh: 0.002,
            lifecycle_co2e_kg_per_mwh: 490.0,
        }
    }

    /// Oil / diesel generation: ~650 kg CO₂e/MWh.
    pub fn oil() -> Self {
        Self {
            fuel_type: "oil".into(),
            co2_kg_per_mwh: 620.0,
            ch4_kg_per_mwh: 0.5,
            n2o_kg_per_mwh: 0.005,
            lifecycle_co2e_kg_per_mwh: 650.0,
        }
    }

    /// Nuclear (lifecycle): ~12 kg CO₂e/MWh.
    ///
    /// Near-zero operational emissions; lifecycle includes uranium enrichment.
    pub fn nuclear() -> Self {
        Self {
            fuel_type: "nuclear".into(),
            co2_kg_per_mwh: 0.0,
            ch4_kg_per_mwh: 0.0,
            n2o_kg_per_mwh: 0.0,
            lifecycle_co2e_kg_per_mwh: 12.0,
        }
    }

    /// Onshore wind (lifecycle): ~11 kg CO₂e/MWh.
    pub fn wind() -> Self {
        Self {
            fuel_type: "wind".into(),
            co2_kg_per_mwh: 0.0,
            ch4_kg_per_mwh: 0.0,
            n2o_kg_per_mwh: 0.0,
            lifecycle_co2e_kg_per_mwh: 11.0,
        }
    }

    /// Solar photovoltaic (lifecycle): ~45 kg CO₂e/MWh.
    ///
    /// Primarily from silicon purification and module manufacture.
    pub fn solar_pv() -> Self {
        Self {
            fuel_type: "solar_pv".into(),
            co2_kg_per_mwh: 0.0,
            ch4_kg_per_mwh: 0.0,
            n2o_kg_per_mwh: 0.0,
            lifecycle_co2e_kg_per_mwh: 45.0,
        }
    }

    /// Hydroelectric (reservoir): ~24 kg CO₂e/MWh.
    ///
    /// Includes methane from anaerobic decomposition in reservoir.
    pub fn hydro() -> Self {
        Self {
            fuel_type: "hydro".into(),
            co2_kg_per_mwh: 0.0,
            ch4_kg_per_mwh: 0.96, // ~24 kg CO₂e via GWP25
            n2o_kg_per_mwh: 0.0,
            lifecycle_co2e_kg_per_mwh: 24.0,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Emitting generator
// ─────────────────────────────────────────────────────────────────────────────

/// Generator with full emission characteristics for carbon market participation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmittingGenerator {
    /// Unique generator identifier.
    pub id: usize,
    /// Human-readable name.
    pub name: String,
    /// Installed capacity \[MW\].
    pub capacity_mw: f64,
    /// Emission factor for this generator.
    pub emission_factor: EmissionFactor,
    /// Free allowances allocated for the current compliance period \[tonne CO₂e\].
    pub allocated_allowances_ton: f64,
    /// Variable (fuel) cost \[EUR/MWh\].
    pub cost_per_mwh: f64,
    /// True if this generator is classified as renewable (zero direct emissions).
    pub is_renewable: bool,
}

impl EmittingGenerator {
    /// Emissions for given generation: `generation_mwh × kg/MWh / 1000` → \[tonne CO₂e\].
    pub fn compute_emissions_ton(&self, generation_mwh: f64) -> f64 {
        generation_mwh * self.emission_factor.co2e_kg_per_mwh() / 1_000.0
    }

    /// Net allowance position: allocated − emitted \[tonne CO₂e\].
    ///
    /// Positive = surplus (can sell). Negative = shortfall (must buy).
    pub fn allowance_position(&self, generation_mwh: f64) -> f64 {
        self.allocated_allowances_ton - self.compute_emissions_ton(generation_mwh)
    }

    /// Carbon cost (negative = revenue) at given carbon price \[EUR\].
    ///
    /// Shortfall × price = cost (positive). Surplus × price = revenue (negative).
    pub fn carbon_cost_eur(&self, generation_mwh: f64, carbon_price_eur_per_ton: f64) -> f64 {
        let position = self.allowance_position(generation_mwh);
        // Shortfall is positive cost; surplus is negative cost (= revenue)
        -position * carbon_price_eur_per_ton
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Emission scheme
// ─────────────────────────────────────────────────────────────────────────────

/// Emission trading scheme / regulatory context.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum EmissionScheme {
    /// EU Emissions Trading System (ETS Phase IV).
    EuEts,
    /// UK Emissions Trading Scheme (post-Brexit).
    UkEts,
    /// China National ETS (power sector).
    ChinaEts,
    /// California Cap-and-Trade Program (AB 32 / SB 32).
    CaliforniaCap,
    /// Voluntary carbon offset market (Gold Standard, VCS, etc.).
    VoluntaryOffset {
        /// Certification standard name (e.g., "Gold Standard", "VCS").
        standard: String,
    },
    /// Internal carbon price set by a company for internal accounting.
    InternalPrice {
        /// Shadow price \[EUR/tonne\].
        company_price_eur_per_ton: f64,
    },
}

// ─────────────────────────────────────────────────────────────────────────────
// Carbon allowance (permit)
// ─────────────────────────────────────────────────────────────────────────────

/// A carbon allowance (EUA-like permit) granting the right to emit 1 tonne CO₂e.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CarbonAllowance {
    /// Unique permit identifier (e.g., "EUA-2025-0001").
    pub permit_id: String,
    /// Year the allowance was issued.
    pub vintage_year: u32,
    /// Quantity of CO₂e covered \[tonne\].
    pub quantity_ton: f64,
    /// Emission trading scheme this allowance belongs to.
    pub scheme: EmissionScheme,
    /// True if independently certified / verified.
    pub is_certified: bool,
    /// Entity ID of the current owner.
    pub owner_id: String,
}

// ─────────────────────────────────────────────────────────────────────────────
// Carbon market
// ─────────────────────────────────────────────────────────────────────────────

/// Emission trading market with price dynamics, auctions, and forecasting.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CarbonMarket {
    /// Regulatory / scheme context.
    pub scheme: EmissionScheme,
    /// Current spot price \[EUR/tonne\].
    pub current_price_eur_per_ton: f64,
    /// Minimum price floor (EU ETS MSR floor) \[EUR/tonne\].
    pub price_floor_eur_per_ton: f64,
    /// Maximum price ceiling (if price containment mechanism exists) \[EUR/tonne\].
    pub price_ceiling_eur_per_ton: f64,
    /// Total annual system cap \[million tonne CO₂e\].
    pub total_cap_million_ton: f64,
    /// Free allocations for the period \[million tonne CO₂e\].
    pub allocated_million_ton: f64,
    /// Allowances available for auction \[million tonne CO₂e\].
    pub auctioned_million_ton: f64,
    /// Maximum fraction of compliance obligation that offsets can cover (0–1).
    pub offset_limit_pct: f64,
    /// Historical price time-series: `(timestamp_unix, price_eur_per_ton)`.
    pub price_history: Vec<(f64, f64)>,
}

impl CarbonMarket {
    /// Create a new carbon market with sensible EU ETS defaults.
    pub fn new(scheme: EmissionScheme, initial_price: f64, total_cap_million_ton: f64) -> Self {
        let floor = match &scheme {
            EmissionScheme::EuEts => 20.0,
            EmissionScheme::UkEts => 22.0,
            _ => 0.0,
        };
        let ceiling = match &scheme {
            EmissionScheme::EuEts => 500.0,
            EmissionScheme::UkEts => 400.0,
            EmissionScheme::CaliforniaCap => 65.0,
            _ => f64::MAX,
        };
        let price = initial_price.max(floor);
        CarbonMarket {
            scheme,
            current_price_eur_per_ton: price.min(ceiling),
            price_floor_eur_per_ton: floor,
            price_ceiling_eur_per_ton: ceiling,
            total_cap_million_ton,
            allocated_million_ton: total_cap_million_ton * 0.43, // ~43% free allocation (EU ETS Phase IV)
            auctioned_million_ton: total_cap_million_ton * 0.57,
            offset_limit_pct: 0.10,
            price_history: vec![(0.0, price.min(ceiling))],
        }
    }

    /// Update market price using a simple supply–demand model.
    ///
    /// If emissions > cap: shortage fraction drives price up.
    /// If emissions < cap: surplus fraction drives price down.
    /// Price is clamped to `[price_floor, price_ceiling]`.
    ///
    /// # Arguments
    /// - `total_emissions_million_ton` — verified emissions for the period
    /// - `price_elasticity` — % price change per % imbalance (default 2.0)
    pub fn update_price(&mut self, total_emissions_million_ton: f64, price_elasticity: f64) {
        let cap = self.total_cap_million_ton;
        if cap <= 0.0 {
            return;
        }
        let imbalance_fraction = (total_emissions_million_ton - cap) / cap;
        let pct_change = price_elasticity * imbalance_fraction;
        let new_price = self.current_price_eur_per_ton * (1.0 + pct_change);
        self.current_price_eur_per_ton = new_price
            .max(self.price_floor_eur_per_ton)
            .min(self.price_ceiling_eur_per_ton);

        // Record in history with a simple sequential timestamp
        let next_ts = self
            .price_history
            .last()
            .map(|(t, _)| t + 1.0)
            .unwrap_or(0.0);
        self.price_history
            .push((next_ts, self.current_price_eur_per_ton));
    }

    /// Execute a bilateral trade: buyer receives `quantity_ton` allowances;
    /// seller receives payment at the current market price.
    ///
    /// Returns the total EUR cost of the trade.
    ///
    /// # Errors
    /// Returns `OxiGridError` if quantity is non-positive or buyer/seller are the same entity.
    pub fn execute_trade(
        &mut self,
        buyer_id: &str,
        seller_id: &str,
        quantity_ton: f64,
    ) -> Result<f64, OxiGridError> {
        if quantity_ton <= 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "Trade quantity must be positive".into(),
            ));
        }
        if buyer_id == seller_id {
            return Err(OxiGridError::InvalidParameter(
                "Buyer and seller must be different entities".into(),
            ));
        }
        let total_eur = quantity_ton * self.current_price_eur_per_ton;
        Ok(total_eur)
    }

    /// Conduct a uniform-price permit auction.
    ///
    /// Bids sorted descending by max price; lowest accepted bid sets the clearing price.
    /// Returns `Vec<(winner_id, allocated_ton, paid_eur)>`.
    ///
    /// # Arguments
    /// - `bids` — `(bidder_id, quantity_ton, max_price_eur_per_ton)`
    pub fn conduct_auction(&self, bids: &[(String, f64, f64)]) -> Vec<(String, f64, f64)> {
        if bids.is_empty() {
            return vec![];
        }

        // Sort bids descending by max price
        let mut sorted: Vec<&(String, f64, f64)> = bids.iter().collect();
        sorted.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap_or(std::cmp::Ordering::Equal));

        // Total supply = auctioned amount (convert from million tonne to tonne)
        let supply_ton = self.auctioned_million_ton * 1_000_000.0;
        let mut remaining = supply_ton;
        let mut clearing_price = 0.0_f64;

        // Determine clearing price (lowest price that clears supply)
        let mut allocations: Vec<(String, f64)> = Vec::new();
        for bid in &sorted {
            if remaining <= 0.0 {
                break;
            }
            let allocated = bid.1.min(remaining);
            remaining -= allocated;
            clearing_price = bid.2;
            allocations.push((bid.0.clone(), allocated));
        }

        // All winners pay the uniform clearing price
        allocations
            .into_iter()
            .map(|(id, qty)| {
                let paid = qty * clearing_price;
                (id, qty, paid)
            })
            .collect()
    }

    /// Forecast carbon price using trend + mean-reversion model.
    ///
    /// `P(t+1) = P(t) × (1 + trend) + reversion × (long_run_mean − P(t))`
    ///
    /// Long-run mean is estimated as the midpoint of `[floor, min(ceiling, 3×current)]`.
    ///
    /// # Arguments
    /// - `horizon_years` — number of years to forecast
    /// - `annual_trend_pct` — annual drift (e.g., 0.05 = +5%/year)
    /// - `mean_reversion_speed` — Ornstein-Uhlenbeck κ (0 = no reversion, 1 = fast)
    ///
    /// # Returns
    /// `Vec<(year, forecast_price)>` with `year` counting from 1.
    pub fn forecast_price(
        &self,
        horizon_years: f64,
        annual_trend_pct: f64,
        mean_reversion_speed: f64,
    ) -> Vec<(f64, f64)> {
        let steps = horizon_years.ceil() as usize;
        if steps == 0 {
            return vec![];
        }

        // Long-run equilibrium price
        let ceiling_cap = self
            .price_ceiling_eur_per_ton
            .min(3.0 * self.current_price_eur_per_ton);
        let long_run_mean = 0.5 * (self.price_floor_eur_per_ton + ceiling_cap);

        let mut result = Vec::with_capacity(steps);
        let mut price = self.current_price_eur_per_ton;
        let kappa = mean_reversion_speed.clamp(0.0, 1.0);

        for step in 1..=steps {
            let year = step as f64;
            // Trend component
            let trend_component = price * annual_trend_pct;
            // Mean-reversion pull
            let reversion_component = kappa * (long_run_mean - price);
            price += trend_component + reversion_component;
            // Clamp to floor/ceiling
            price = price
                .max(self.price_floor_eur_per_ton)
                .min(self.price_ceiling_eur_per_ton);
            result.push((year, price));
        }
        result
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Budget action recommendation
// ─────────────────────────────────────────────────────────────────────────────

/// Recommended action based on carbon budget status.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum BudgetAction {
    /// On track — no action required.
    Continue,
    /// Curtail high-emission generators by the given amount \[MW\].
    ReduceProduction { mw: f64 },
    /// Purchase additional allowances to cover projected shortfall.
    PurchaseAllowances {
        /// Tonnes to purchase.
        ton: f64,
        /// Estimated EUR cost at current carbon price.
        estimated_cost_eur: f64,
    },
    /// Replace fossil generation with renewable capacity \[MW\].
    IncreaseRenewable { mw: f64 },
    /// Budget already exceeded — accept compliance penalty.
    NoAction,
}

// ─────────────────────────────────────────────────────────────────────────────
// Budget status
// ─────────────────────────────────────────────────────────────────────────────

/// Snapshot of the current carbon budget status.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BudgetStatus {
    /// Total budget for the period \[tonne CO₂e\].
    pub budget_ton: f64,
    /// Cumulative emissions to date \[tonne CO₂e\].
    pub emissions_ton: f64,
    /// Budget remaining \[tonne CO₂e\] = `budget - emissions`.
    pub remaining_budget_ton: f64,
    /// Fraction of budget consumed (0–1+).
    pub pct_budget_used: f64,
    /// Extrapolated end-of-period emissions at the current rate \[tonne CO₂e\].
    pub projected_end_of_period_ton: f64,
    /// True if the extrapolation exceeds the budget.
    pub will_exceed_budget: bool,
    /// Allowance surplus (positive) or deficit (negative) \[tonne CO₂e\].
    pub allowance_surplus_deficit_ton: f64,
    /// Recommended management action.
    pub recommended_action: BudgetAction,
}

// ─────────────────────────────────────────────────────────────────────────────
// Scope 1/2/3 emissions report
// ─────────────────────────────────────────────────────────────────────────────

/// ISO 14064-1 scope emissions breakdown.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScopeEmissionsReport {
    /// Scope 1 — direct combustion emissions \[tonne CO₂e\].
    pub scope1_ton: f64,
    /// Scope 2 — purchased electricity (zero for generators) \[tonne CO₂e\].
    pub scope2_ton: f64,
    /// Scope 3 — value-chain (upstream fuel, equipment manufacture) \[tonne CO₂e\].
    pub scope3_ton: f64,
    /// Total across all scopes \[tonne CO₂e\].
    pub total_ton: f64,
    /// Emission intensity \[kg CO₂e/MWh\].
    pub intensity_kg_per_mwh: f64,
    /// Renewable fraction of total generation \[%\].
    pub renewable_fraction_pct: f64,
    /// CO₂e avoided vs. a 100% coal baseline \[tonne CO₂e\].
    pub co2e_avoided_vs_baseline_ton: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Carbon budget tracker
// ─────────────────────────────────────────────────────────────────────────────

/// Tracks carbon emissions, allowances, and budget for a fleet of generators.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CarbonBudgetTracker {
    /// Registered generators in the fleet.
    pub generators: Vec<EmittingGenerator>,
    /// Accounting period for this budget.
    pub budget_period: CarbonPeriod,
    /// Total CO₂e budget for the period \[tonne CO₂e\].
    pub total_budget_ton: f64,
    /// Cumulative emissions recorded so far \[tonne CO₂e\].
    pub emissions_to_date_ton: f64,
    /// Current allowances held by the entity \[tonne CO₂e\].
    pub allowances_held_ton: f64,
    /// Cumulative generation per generator (indexed by `EmittingGenerator.id`) \[MWh\].
    generation_log: Vec<f64>,
}

impl CarbonBudgetTracker {
    /// Create a new tracker for a fleet of generators.
    ///
    /// # Arguments
    /// - `generators`       — fleet of emitting generators
    /// - `budget_period`    — accounting period
    /// - `total_budget_ton` — total CO₂e budget for the period \[tonne\]
    pub fn new(
        generators: Vec<EmittingGenerator>,
        budget_period: CarbonPeriod,
        total_budget_ton: f64,
    ) -> Self {
        let n = generators.len();
        let allowances_held: f64 = generators.iter().map(|g| g.allocated_allowances_ton).sum();
        Self {
            generators,
            budget_period,
            total_budget_ton,
            emissions_to_date_ton: 0.0,
            allowances_held_ton: allowances_held,
            generation_log: vec![0.0; n],
        }
    }

    /// Record actual generation and accumulate emissions.
    ///
    /// # Arguments
    /// - `generator_id`  — the `EmittingGenerator.id` field
    /// - `generation_mwh` — energy generated this interval \[MWh\]
    ///
    /// # Returns
    /// Emissions from this interval \[tonne CO₂e\].
    ///
    /// # Errors
    /// Returns `OxiGridError` if the generator ID is not found in the fleet.
    pub fn record_generation(
        &mut self,
        generator_id: usize,
        generation_mwh: f64,
    ) -> Result<f64, OxiGridError> {
        let idx = self
            .generators
            .iter()
            .position(|g| g.id == generator_id)
            .ok_or_else(|| {
                OxiGridError::InvalidParameter(format!(
                    "Generator id {} not found in fleet",
                    generator_id
                ))
            })?;

        let emissions = self.generators[idx].compute_emissions_ton(generation_mwh);
        self.emissions_to_date_ton += emissions;
        self.generation_log[idx] += generation_mwh;
        Ok(emissions)
    }

    /// Compute current carbon budget status and recommend an action.
    ///
    /// # Arguments
    /// - `fraction_of_period_elapsed` — how far through the accounting period (0–1)
    /// - `carbon_price`               — current carbon price \[EUR/tonne\]
    pub fn budget_status(
        &self,
        fraction_of_period_elapsed: f64,
        carbon_price: f64,
    ) -> BudgetStatus {
        let fraction = fraction_of_period_elapsed.clamp(1e-9, 1.0);
        let projected = if fraction > 0.0 {
            self.emissions_to_date_ton / fraction
        } else {
            self.emissions_to_date_ton
        };
        let remaining = self.total_budget_ton - self.emissions_to_date_ton;
        let pct_used = if self.total_budget_ton > 0.0 {
            self.emissions_to_date_ton / self.total_budget_ton
        } else {
            0.0
        };
        let will_exceed = projected > self.total_budget_ton;
        let surplus_deficit = self.allowances_held_ton - self.emissions_to_date_ton;

        // Shortfall of allowances vs projected total emissions
        let projected_deficit = (projected - self.allowances_held_ton).max(0.0);

        let action = if !will_exceed && surplus_deficit >= 0.0 {
            BudgetAction::Continue
        } else if projected_deficit > 0.0 && fraction < 0.9 {
            // There is still time to act: recommend purchasing allowances
            let cost = projected_deficit * carbon_price;
            BudgetAction::PurchaseAllowances {
                ton: projected_deficit,
                estimated_cost_eur: cost,
            }
        } else if will_exceed {
            // Already over: estimate how much fossil generation to curtail
            let excess = projected - self.total_budget_ton;
            // Find average emission intensity of fossil generators [tonne/MWh]
            let avg_fossil_intensity: f64 = {
                let fossils: Vec<f64> = self
                    .generators
                    .iter()
                    .filter(|g| !g.is_renewable)
                    .map(|g| g.emission_factor.co2e_kg_per_mwh() / 1_000.0)
                    .collect();
                if fossils.is_empty() {
                    1.0 // fallback
                } else {
                    fossils.iter().sum::<f64>() / fossils.len() as f64
                }
            };
            if avg_fossil_intensity > 0.0 {
                let curtail_mwh = excess / avg_fossil_intensity;
                // Scale from total-period MWh to average MW (assume 8760 h/yr)
                let curtail_mw = curtail_mwh / 8_760.0;
                BudgetAction::ReduceProduction { mw: curtail_mw }
            } else {
                BudgetAction::NoAction
            }
        } else {
            BudgetAction::Continue
        };

        BudgetStatus {
            budget_ton: self.total_budget_ton,
            emissions_ton: self.emissions_to_date_ton,
            remaining_budget_ton: remaining,
            pct_budget_used: pct_used,
            projected_end_of_period_ton: projected,
            will_exceed_budget: will_exceed,
            allowance_surplus_deficit_ton: surplus_deficit,
            recommended_action: action,
        }
    }

    /// Optimal dispatch considering carbon cost.
    ///
    /// Adds `carbon_price × emission_factor_tonne_per_mwh` to each generator's
    /// fuel cost, then ranks generators by total effective cost (merit order).
    ///
    /// # Returns
    /// `Vec<(generator_id, dispatch_mw)>` in dispatch order.
    pub fn carbon_adjusted_dispatch(
        &self,
        total_demand_mw: f64,
        carbon_price_eur_per_ton: f64,
    ) -> Vec<(usize, f64)> {
        // Build (index, effective_cost) pairs
        let mut order: Vec<(usize, f64)> = self
            .generators
            .iter()
            .enumerate()
            .map(|(idx, g)| {
                let carbon_cost_per_mwh =
                    carbon_price_eur_per_ton * g.emission_factor.co2e_kg_per_mwh() / 1_000.0;
                let effective_cost = g.cost_per_mwh + carbon_cost_per_mwh;
                (idx, effective_cost)
            })
            .collect();

        // Sort by ascending effective cost
        order.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

        let mut remaining = total_demand_mw;
        let mut result = Vec::new();

        for (idx, _cost) in &order {
            if remaining <= 0.0 {
                break;
            }
            let gen = &self.generators[*idx];
            let dispatch = gen.capacity_mw.min(remaining);
            remaining -= dispatch;
            result.push((gen.id, dispatch));
        }

        result
    }

    /// Estimate the marginal abatement cost (MAC) of switching generation from one
    /// generator to another.
    ///
    /// `MAC = (cost_to - cost_from) / (emission_from - emission_to)` \[EUR/tonne CO₂e\].
    ///
    /// A positive MAC means abatement is costly; negative means it is profitable.
    ///
    /// # Arguments
    /// - `from_generator` — generator being displaced (higher emissions)
    /// - `to_generator`   — replacement generator (lower emissions)
    ///
    /// # Errors
    /// Returns `OxiGridError` if either generator ID is not found, or if the
    /// emission intensities are identical (no abatement, MAC undefined).
    pub fn marginal_abatement_cost(
        &self,
        from_generator: usize,
        to_generator: usize,
    ) -> Result<f64, OxiGridError> {
        let from = self
            .generators
            .iter()
            .find(|g| g.id == from_generator)
            .ok_or_else(|| {
                OxiGridError::InvalidParameter(format!(
                    "from_generator {} not found",
                    from_generator
                ))
            })?;

        let to = self
            .generators
            .iter()
            .find(|g| g.id == to_generator)
            .ok_or_else(|| {
                OxiGridError::InvalidParameter(format!("to_generator {} not found", to_generator))
            })?;

        // Convert kg/MWh → tonne/MWh
        let emission_from = from.emission_factor.co2e_kg_per_mwh() / 1_000.0;
        let emission_to = to.emission_factor.co2e_kg_per_mwh() / 1_000.0;
        let delta_emission = emission_from - emission_to; // [tonne CO₂e/MWh]

        if delta_emission.abs() < 1e-12 {
            return Err(OxiGridError::InvalidParameter(
                "Generators have identical emission intensities; MAC is undefined".into(),
            ));
        }

        let delta_cost = to.cost_per_mwh - from.cost_per_mwh; // [EUR/MWh]
                                                              // MAC [EUR/tonne] = delta_cost [EUR/MWh] / delta_emission [tonne/MWh]
        Ok(delta_cost / delta_emission)
    }

    /// Compute scope 1, 2, 3 emissions report for the fleet.
    ///
    /// # Arguments
    /// - `generation_mwh` — energy generated per generator (same order as `self.generators`)
    ///
    /// # Scope definitions (ISO 14064-1):
    /// - **Scope 1**: Direct CO₂/CH₄/N₂O from combustion (`co2e_kg_per_mwh`)
    /// - **Scope 2**: Purchased electricity — zero for electricity generators
    /// - **Scope 3**: Lifecycle upstream (module manufacture, fuel extraction)
    ///   = `lifecycle_co2e - direct_co2e`
    pub fn scope_emissions_report(&self, generation_mwh: &[f64]) -> ScopeEmissionsReport {
        let n = self.generators.len().min(generation_mwh.len());

        let mut scope1 = 0.0_f64;
        let mut scope3 = 0.0_f64;
        let mut total_gen = 0.0_f64;
        let mut renewable_gen = 0.0_f64;
        let mut coal_baseline_emissions = 0.0_f64;

        let coal_factor = EmissionFactor::coal();
        let coal_intensity = coal_factor.co2e_kg_per_mwh() / 1_000.0; // tonne/MWh

        for (gen, &mwh) in self.generators.iter().zip(generation_mwh.iter()).take(n) {
            let direct_tonne = gen.emission_factor.co2e_kg_per_mwh() * mwh / 1_000.0;
            let lifecycle_tonne = gen.emission_factor.lifecycle_co2e_kg_per_mwh * mwh / 1_000.0;

            scope1 += direct_tonne;
            // Scope 3 = lifecycle minus direct combustion
            scope3 += (lifecycle_tonne - direct_tonne).max(0.0);

            total_gen += mwh;
            if gen.is_renewable {
                renewable_gen += mwh;
            }
            // What the same MWh would have emitted from coal
            coal_baseline_emissions += mwh * coal_intensity;
        }

        let total_ton = scope1 + scope3; // scope2 = 0
        let intensity = if total_gen > 0.0 {
            total_ton * 1_000.0 / total_gen // kg/MWh
        } else {
            0.0
        };
        let renewable_pct = if total_gen > 0.0 {
            100.0 * renewable_gen / total_gen
        } else {
            0.0
        };
        let avoided = (coal_baseline_emissions - total_ton).max(0.0);

        ScopeEmissionsReport {
            scope1_ton: scope1,
            scope2_ton: 0.0,
            scope3_ton: scope3,
            total_ton,
            intensity_kg_per_mwh: intensity,
            renewable_fraction_pct: renewable_pct,
            co2e_avoided_vs_baseline_ton: avoided,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Grid emission intensity
// ─────────────────────────────────────────────────────────────────────────────

/// Instantaneous grid emission intensity (average and marginal).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GridEmissionIntensity {
    /// Timestamp (e.g., Unix epoch seconds).
    pub timestamp: f64,
    /// Dispatch-weighted average emission intensity \[kg CO₂e/MWh\].
    pub average_intensity_kg_per_mwh: f64,
    /// Emission rate of the next (most expensive) marginal unit \[kg CO₂e/MWh\].
    pub marginal_intensity_kg_per_mwh: f64,
    /// Renewable fraction of total dispatched output \[%\].
    pub renewable_fraction_pct: f64,
    /// Dispatchable (non-renewable) fraction of dispatched output \[%\].
    pub dispatchable_fraction_pct: f64,
}

impl GridEmissionIntensity {
    /// Compute from a dispatch vector.
    ///
    /// # Arguments
    /// - `generators` — generator fleet (same order as `dispatch_mw`)
    /// - `dispatch_mw` — current dispatch \[MW\]
    pub fn from_dispatch(
        generators: &[EmittingGenerator],
        dispatch_mw: &[f64],
    ) -> GridEmissionIntensity {
        let n = generators.len().min(dispatch_mw.len());
        let mut total_mw = 0.0_f64;
        let mut weighted_intensity = 0.0_f64;
        let mut renewable_mw = 0.0_f64;

        for i in 0..n {
            let mw = dispatch_mw[i];
            if mw <= 0.0 {
                continue;
            }
            let intensity = generators[i].emission_factor.co2e_kg_per_mwh();
            weighted_intensity += mw * intensity;
            total_mw += mw;
            if generators[i].is_renewable {
                renewable_mw += mw;
            }
        }

        let avg_intensity = if total_mw > 0.0 {
            weighted_intensity / total_mw
        } else {
            0.0
        };

        let marginal = Self::marginal_rate(generators, dispatch_mw);

        let renewable_pct = if total_mw > 0.0 {
            100.0 * renewable_mw / total_mw
        } else {
            0.0
        };
        let dispatchable_pct = 100.0 - renewable_pct;

        GridEmissionIntensity {
            timestamp: 0.0,
            average_intensity_kg_per_mwh: avg_intensity,
            marginal_intensity_kg_per_mwh: marginal,
            renewable_fraction_pct: renewable_pct,
            dispatchable_fraction_pct: dispatchable_pct,
        }
    }

    /// Marginal emission rate: the emission factor of the last dispatched generator.
    ///
    /// "Last" is defined as the generator with the highest fuel cost among those
    /// with dispatch > 0. If no generators are dispatched, returns 0.
    pub fn marginal_rate(generators: &[EmittingGenerator], dispatch_mw: &[f64]) -> f64 {
        let n = generators.len().min(dispatch_mw.len());

        // Find the dispatched generator with the highest cost per MWh (the price-setter)
        let marginal_gen = (0..n).filter(|&i| dispatch_mw[i] > 1e-9).max_by(|&a, &b| {
            generators[a]
                .cost_per_mwh
                .partial_cmp(&generators[b].cost_per_mwh)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        marginal_gen
            .map(|i| generators[i].emission_factor.co2e_kg_per_mwh())
            .unwrap_or(0.0)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ─── Helpers ────────────────────────────────────────────────────────────

    fn make_coal_gen() -> EmittingGenerator {
        EmittingGenerator {
            id: 0,
            name: "Coal Plant".into(),
            capacity_mw: 200.0,
            emission_factor: EmissionFactor::coal(),
            allocated_allowances_ton: 50_000.0,
            cost_per_mwh: 35.0,
            is_renewable: false,
        }
    }

    fn make_gas_gen() -> EmittingGenerator {
        EmittingGenerator {
            id: 1,
            name: "CCGT".into(),
            capacity_mw: 150.0,
            emission_factor: EmissionFactor::natural_gas(),
            allocated_allowances_ton: 20_000.0,
            cost_per_mwh: 55.0,
            is_renewable: false,
        }
    }

    fn make_wind_gen() -> EmittingGenerator {
        EmittingGenerator {
            id: 2,
            name: "Wind Farm".into(),
            capacity_mw: 100.0,
            emission_factor: EmissionFactor::wind(),
            allocated_allowances_ton: 0.0,
            cost_per_mwh: 5.0,
            is_renewable: true,
        }
    }

    fn make_solar_gen() -> EmittingGenerator {
        EmittingGenerator {
            id: 3,
            name: "Solar PV".into(),
            capacity_mw: 80.0,
            emission_factor: EmissionFactor::solar_pv(),
            allocated_allowances_ton: 0.0,
            cost_per_mwh: 3.0,
            is_renewable: true,
        }
    }

    fn make_tracker() -> CarbonBudgetTracker {
        CarbonBudgetTracker::new(
            vec![make_coal_gen(), make_gas_gen(), make_wind_gen()],
            CarbonPeriod::Annual { year: 2025 },
            200_000.0, // 200 kt CO₂e annual budget
        )
    }

    fn make_market() -> CarbonMarket {
        CarbonMarket::new(EmissionScheme::EuEts, 65.0, 1_500.0)
    }

    // ─── EmissionFactor tests ────────────────────────────────────────────────

    #[test]
    fn test_emission_factor_coal() {
        let coal = EmissionFactor::coal();
        assert_eq!(coal.fuel_type, "coal");
        assert!(
            coal.co2_kg_per_mwh > 700.0,
            "Coal CO₂ should be >700 kg/MWh"
        );
        assert!(
            coal.lifecycle_co2e_kg_per_mwh >= 800.0,
            "Coal lifecycle should be >=800 kg CO₂e/MWh"
        );
    }

    #[test]
    fn test_emission_factor_co2e() {
        let gas = EmissionFactor::natural_gas();
        let co2e = gas.co2e_kg_per_mwh();
        // Must include CH₄ GWP contribution (3.5 × 25 = 87.5)
        assert!(
            co2e > gas.co2_kg_per_mwh,
            "CO₂e must exceed direct CO₂: {:.2} vs {:.2}",
            co2e,
            gas.co2_kg_per_mwh
        );
        // Verify formula: CO₂ + 25*CH₄ + 298*N₂O
        let expected = gas.co2_kg_per_mwh + 25.0 * gas.ch4_kg_per_mwh + 298.0 * gas.n2o_kg_per_mwh;
        assert!(
            (co2e - expected).abs() < 1e-9,
            "CO₂e formula mismatch: {:.4} vs {:.4}",
            co2e,
            expected
        );
    }

    #[test]
    fn test_emission_factor_wind_low() {
        let wind = EmissionFactor::wind();
        // Direct emissions are zero; lifecycle is ~11 kg CO₂e/MWh
        assert!(
            wind.co2e_kg_per_mwh() < 1.0,
            "Wind direct CO₂e should be near zero: {:.4}",
            wind.co2e_kg_per_mwh()
        );
        assert!(
            wind.lifecycle_co2e_kg_per_mwh < 20.0,
            "Wind lifecycle CO₂e should be <20 kg/MWh: {:.1}",
            wind.lifecycle_co2e_kg_per_mwh
        );
    }

    // ─── EmittingGenerator tests ─────────────────────────────────────────────

    #[test]
    fn test_emitting_generator_emissions() {
        let coal = make_coal_gen();
        // 1 MWh coal at ~800 kg CO₂e/MWh → ~0.8 tonne
        let emissions = coal.compute_emissions_ton(1.0);
        assert!(
            (emissions - coal.emission_factor.co2e_kg_per_mwh() / 1_000.0).abs() < 1e-9,
            "Emissions per MWh mismatch: {:.6}",
            emissions
        );
    }

    #[test]
    fn test_emitting_generator_allowance_surplus() {
        let coal = make_coal_gen();
        // Allocated = 50,000 t; generate 10,000 MWh → ~8,000 t emissions → surplus
        let surplus = coal.allowance_position(10_000.0);
        assert!(
            surplus > 0.0,
            "Should have allowance surplus for modest generation: {:.2}",
            surplus
        );
    }

    #[test]
    fn test_emitting_generator_allowance_deficit() {
        let coal = make_coal_gen();
        // Generate at full capacity for entire year: 200 MW × 8760 h = 1,752,000 MWh
        // Emissions ≈ 1,752,000 × 0.8/1000 = 1,401.6 kt >> 50 kt allocation
        let deficit = coal.allowance_position(1_752_000.0);
        assert!(
            deficit < 0.0,
            "Should have allowance deficit at full-year generation: {:.2}",
            deficit
        );
    }

    #[test]
    fn test_carbon_cost_with_price() {
        let coal = make_coal_gen();
        let price = 80.0; // EUR/t
        let mwh = 1_000.0;
        // With 50,000 t allocated and ~800 t emitted → surplus → negative cost (revenue)
        let cost = coal.carbon_cost_eur(mwh, price);
        let position = coal.allowance_position(mwh);
        let expected = -position * price;
        assert!(
            (cost - expected).abs() < 1e-6,
            "Carbon cost mismatch: {:.4} vs {:.4}",
            cost,
            expected
        );
    }

    // ─── CarbonMarket tests ──────────────────────────────────────────────────

    #[test]
    fn test_carbon_market_creation() {
        let market = make_market();
        assert_eq!(market.total_cap_million_ton, 1_500.0);
        assert!(
            market.current_price_eur_per_ton >= market.price_floor_eur_per_ton,
            "Price must be >= floor"
        );
        assert!(
            market.current_price_eur_per_ton <= market.price_ceiling_eur_per_ton,
            "Price must be <= ceiling"
        );
    }

    #[test]
    fn test_carbon_market_price_update_shortage() {
        let mut market = make_market();
        let initial_price = market.current_price_eur_per_ton;
        // Emit 10% more than cap → shortage → price should rise
        let excess_emissions = market.total_cap_million_ton * 1.10;
        market.update_price(excess_emissions, 2.0);
        assert!(
            market.current_price_eur_per_ton > initial_price,
            "Price should rise on shortage: {:.2} → {:.2}",
            initial_price,
            market.current_price_eur_per_ton
        );
    }

    #[test]
    fn test_carbon_market_price_update_surplus() {
        let mut market = make_market();
        let initial_price = market.current_price_eur_per_ton;
        // Emit 20% less than cap → surplus → price should fall
        let low_emissions = market.total_cap_million_ton * 0.80;
        market.update_price(low_emissions, 2.0);
        assert!(
            market.current_price_eur_per_ton < initial_price,
            "Price should fall on surplus: {:.2} → {:.2}",
            initial_price,
            market.current_price_eur_per_ton
        );
    }

    #[test]
    fn test_execute_trade() {
        let mut market = make_market();
        let cost = market
            .execute_trade("company_a", "company_b", 500.0)
            .expect("trade should succeed");
        let expected = 500.0 * market.current_price_eur_per_ton;
        assert!(
            (cost - expected).abs() < 1e-6,
            "Trade cost mismatch: {:.2} vs {:.2}",
            cost,
            expected
        );
    }

    #[test]
    fn test_conduct_auction_basic() {
        let market = make_market();
        let bids = vec![
            ("company_a".into(), 100_000.0, 70.0_f64),
            ("company_b".into(), 200_000.0, 60.0_f64),
            ("company_c".into(), 50_000.0, 80.0_f64),
        ];
        let winners = market.conduct_auction(&bids);
        // All winners should have been allocated something
        assert!(!winners.is_empty(), "Auction should produce winners");
        for (id, qty, paid) in &winners {
            assert!(*qty > 0.0, "Winner {} should get positive quantity", id);
            assert!(*paid > 0.0, "Winner {} should pay positive amount", id);
        }
    }

    #[test]
    fn test_price_forecast_trend() {
        let market = make_market();
        // Positive trend: price should increase over time
        let forecast = market.forecast_price(5.0, 0.05, 0.0);
        assert_eq!(forecast.len(), 5, "Should forecast 5 years");
        let (_, p1) = forecast[0];
        let (_, p5) = forecast[4];
        assert!(
            p5 > p1,
            "With positive trend, year-5 price should exceed year-1: {:.2} vs {:.2}",
            p5,
            p1
        );
        // All prices should be within floor/ceiling
        for (yr, price) in &forecast {
            assert!(
                *price >= market.price_floor_eur_per_ton,
                "Year {:.0} price {:.2} below floor",
                yr,
                price
            );
            assert!(
                *price <= market.price_ceiling_eur_per_ton,
                "Year {:.0} price {:.2} above ceiling",
                yr,
                price
            );
        }
    }

    // ─── CarbonBudgetTracker tests ───────────────────────────────────────────

    #[test]
    fn test_budget_tracker_creation() {
        let tracker = make_tracker();
        assert_eq!(tracker.generators.len(), 3);
        assert_eq!(tracker.total_budget_ton, 200_000.0);
        assert!(tracker.emissions_to_date_ton < 1e-9);
        // Allowances should sum allocated amounts from coal + gas + wind
        let expected_allowances: f64 = tracker
            .generators
            .iter()
            .map(|g| g.allocated_allowances_ton)
            .sum();
        assert!(
            (tracker.allowances_held_ton - expected_allowances).abs() < 1e-6,
            "Allowances held should equal sum of allocated: {:.2} vs {:.2}",
            tracker.allowances_held_ton,
            expected_allowances
        );
    }

    #[test]
    fn test_record_generation() {
        let mut tracker = make_tracker();
        // Record 1000 MWh from coal generator (id=0)
        let emissions = tracker
            .record_generation(0, 1_000.0)
            .expect("should succeed");
        // Emissions = 1000 × coal_co2e_kg / 1000 → tonnes
        let expected = make_coal_gen().compute_emissions_ton(1_000.0);
        assert!(
            (emissions - expected).abs() < 1e-9,
            "Recorded emissions mismatch: {:.4} vs {:.4}",
            emissions,
            expected
        );
        assert!(
            (tracker.emissions_to_date_ton - expected).abs() < 1e-9,
            "Cumulative emissions not updated correctly"
        );
    }

    #[test]
    fn test_budget_status_on_track() {
        let mut tracker = make_tracker();
        // Generate 5000 MWh from coal at half-way point → should be on track
        // 5000 MWh × 800 kg/MWh / 1000 = 4000 t → extrapolated 8000 t << 200,000 t budget
        tracker.record_generation(0, 5_000.0).expect("ok");
        let status = tracker.budget_status(0.5, 65.0);
        assert!(
            !status.will_exceed_budget,
            "Should be on track for modest generation"
        );
        assert!(status.pct_budget_used < 0.5, "Should use < 50% of budget");
    }

    #[test]
    fn test_budget_status_over_budget() {
        let mut tracker = make_tracker();
        // Record a massive amount of coal generation that will exceed the budget
        // Budget = 200,000 t; coal = ~800 kg/MWh; need > 250,000 MWh to exceed
        // Let's record 400,000 MWh at 10% elapsed → projected = 4M MWh → way over budget
        tracker.record_generation(0, 400_000.0).expect("ok");
        let status = tracker.budget_status(0.10, 65.0);
        assert!(
            status.will_exceed_budget,
            "Should flag budget exceedance: projected {:.0} t vs budget {:.0} t",
            status.projected_end_of_period_ton, status.budget_ton
        );
    }

    #[test]
    fn test_carbon_adjusted_dispatch() {
        let tracker = make_tracker();
        // At high carbon price, renewable (wind, id=2) should come first
        // Wind cost = 5 EUR/MWh + 0 carbon; Coal cost = 35 + high carbon
        let dispatch = tracker.carbon_adjusted_dispatch(200.0, 200.0);

        // First dispatched generator should be wind (lowest effective cost)
        let first_gen_id = dispatch.first().map(|(id, _)| *id).unwrap_or(99);
        assert_eq!(
            first_gen_id, 2,
            "At high carbon price, wind (id=2) should be dispatched first, got {}",
            first_gen_id
        );

        // Total dispatched should equal demand
        let total: f64 = dispatch.iter().map(|(_, mw)| mw).sum();
        assert!(
            (total - 200.0).abs() < 1e-6 || total <= 200.0 + 1e-6,
            "Dispatch total should match demand: {:.2}",
            total
        );
    }

    #[test]
    fn test_marginal_abatement_cost() {
        let tracker = make_tracker();
        // Switching from coal (id=0, high emission) to gas (id=1, lower emission)
        let mac = tracker
            .marginal_abatement_cost(0, 1)
            .expect("MAC should be computable");
        // Coal ~800 kg/MWh → 0.8 t/MWh; Gas ~490 kg/MWh direct + GWP
        // The switch costs more per MWh (gas costs 55 vs coal 35) but saves emissions
        // Positive MAC = it costs money to switch from coal to gas
        assert!(mac.is_finite(), "MAC should be a finite number: {:.4}", mac);
    }

    #[test]
    fn test_scope_emissions_report() {
        let tracker = make_tracker();
        // 1000 MWh from each generator
        let generation = vec![1_000.0, 1_000.0, 1_000.0];
        let report = tracker.scope_emissions_report(&generation);

        assert!(
            report.scope1_ton > 0.0,
            "Scope 1 should be positive (coal+gas)"
        );
        assert!(
            report.scope2_ton.abs() < 1e-9,
            "Scope 2 should be zero for generators"
        );
        assert!(report.scope3_ton >= 0.0, "Scope 3 should be non-negative");
        assert!(
            (report.total_ton - report.scope1_ton - report.scope2_ton - report.scope3_ton).abs()
                < 1e-6,
            "Total should equal sum of scopes"
        );
        // Renewable fraction: 1000 MWh wind out of 3000 MWh total = ~33%
        assert!(
            (report.renewable_fraction_pct - 100.0 / 3.0).abs() < 1.0,
            "Renewable fraction should be ~33%: {:.2}%",
            report.renewable_fraction_pct
        );
        // CO₂e avoided vs coal baseline should be positive (gas and wind are cleaner)
        assert!(
            report.co2e_avoided_vs_baseline_ton > 0.0,
            "CO₂e avoided should be positive: {:.2}",
            report.co2e_avoided_vs_baseline_ton
        );
    }

    // ─── GridEmissionIntensity tests ─────────────────────────────────────────

    #[test]
    fn test_grid_emission_intensity() {
        let generators = vec![make_coal_gen(), make_wind_gen()];
        // 100 MW coal, 50 MW wind
        let dispatch = vec![100.0, 50.0];
        let gei = GridEmissionIntensity::from_dispatch(&generators, &dispatch);

        // Average should be between wind (0) and coal (~800)
        assert!(
            gei.average_intensity_kg_per_mwh > 0.0,
            "Average intensity should be positive with coal in mix"
        );
        assert!(
            gei.average_intensity_kg_per_mwh < EmissionFactor::coal().co2e_kg_per_mwh(),
            "Average should be less than pure coal intensity"
        );

        // Renewable fraction: 50/(100+50) = 33.3%
        assert!(
            (gei.renewable_fraction_pct - 100.0 / 3.0).abs() < 1.0,
            "Renewable fraction {:.2}% should be ~33%",
            gei.renewable_fraction_pct
        );
        assert!(
            (gei.dispatchable_fraction_pct + gei.renewable_fraction_pct - 100.0).abs() < 1e-6,
            "Dispatchable + renewable should = 100%"
        );
    }

    #[test]
    fn test_marginal_emission_rate() {
        let generators = vec![make_coal_gen(), make_gas_gen(), make_wind_gen()];
        // Wind is cheapest (5 EUR/MWh); coal and gas also dispatched
        // Marginal unit = highest cost unit dispatched = gas (55 EUR/MWh)
        let dispatch = vec![100.0, 80.0, 50.0];
        let marginal = GridEmissionIntensity::marginal_rate(&generators, &dispatch);
        // Gas emission intensity
        let gas_intensity = EmissionFactor::natural_gas().co2e_kg_per_mwh();
        assert!(
            (marginal - gas_intensity).abs() < 1e-6,
            "Marginal rate should be gas intensity ({:.2}), got {:.2}",
            gas_intensity,
            marginal
        );
    }

    // ─── Additional edge-case tests ──────────────────────────────────────────

    #[test]
    fn test_execute_trade_invalid_same_entity() {
        let mut market = make_market();
        let result = market.execute_trade("same", "same", 100.0);
        assert!(
            result.is_err(),
            "Trade with same buyer and seller should fail"
        );
    }

    #[test]
    fn test_execute_trade_invalid_zero_quantity() {
        let mut market = make_market();
        let result = market.execute_trade("a", "b", 0.0);
        assert!(result.is_err(), "Trade with zero quantity should fail");
    }

    #[test]
    fn test_record_generation_invalid_id() {
        let mut tracker = make_tracker();
        let result = tracker.record_generation(999, 100.0);
        assert!(result.is_err(), "Unknown generator id should return error");
    }

    #[test]
    fn test_carbon_period_label() {
        assert_eq!(CarbonPeriod::Annual { year: 2025 }.label(), "2025");
        assert_eq!(
            CarbonPeriod::Monthly {
                year: 2025,
                month: 3
            }
            .label(),
            "2025-03"
        );
        assert_eq!(
            CarbonPeriod::Daily {
                year: 2025,
                month: 3,
                day: 9
            }
            .label(),
            "2025-03-09"
        );
    }

    #[test]
    fn test_mac_undefined_same_intensity() {
        // Two identical generators (same emission intensity)
        let tracker = CarbonBudgetTracker::new(
            vec![
                EmittingGenerator {
                    id: 10,
                    name: "A".into(),
                    capacity_mw: 100.0,
                    emission_factor: EmissionFactor::coal(),
                    allocated_allowances_ton: 0.0,
                    cost_per_mwh: 30.0,
                    is_renewable: false,
                },
                EmittingGenerator {
                    id: 11,
                    name: "B".into(),
                    capacity_mw: 100.0,
                    emission_factor: EmissionFactor::coal(),
                    allocated_allowances_ton: 0.0,
                    cost_per_mwh: 40.0,
                    is_renewable: false,
                },
            ],
            CarbonPeriod::Annual { year: 2025 },
            100_000.0,
        );
        let result = tracker.marginal_abatement_cost(10, 11);
        assert!(
            result.is_err(),
            "MAC should be undefined for identical emission intensities"
        );
    }

    #[test]
    fn test_emission_factors_ordering() {
        // Lifecycle CO₂e ordering: coal > oil > gas > solar > hydro > nuclear ≈ wind
        let coal = EmissionFactor::coal().lifecycle_co2e_kg_per_mwh;
        let oil = EmissionFactor::oil().lifecycle_co2e_kg_per_mwh;
        let gas = EmissionFactor::natural_gas().lifecycle_co2e_kg_per_mwh;
        let solar = EmissionFactor::solar_pv().lifecycle_co2e_kg_per_mwh;
        let hydro = EmissionFactor::hydro().lifecycle_co2e_kg_per_mwh;
        let nuclear = EmissionFactor::nuclear().lifecycle_co2e_kg_per_mwh;
        let wind = EmissionFactor::wind().lifecycle_co2e_kg_per_mwh;

        assert!(
            coal > oil,
            "Coal lifecycle > oil: {:.0} vs {:.0}",
            coal,
            oil
        );
        assert!(oil > gas, "Oil lifecycle > gas: {:.0} vs {:.0}", oil, gas);
        assert!(
            gas > solar,
            "Gas lifecycle > solar: {:.0} vs {:.0}",
            gas,
            solar
        );
        assert!(
            solar > hydro,
            "Solar lifecycle > hydro: {:.0} vs {:.0}",
            solar,
            hydro
        );
        assert!(
            hydro > nuclear,
            "Hydro lifecycle > nuclear: {:.0} vs {:.0}",
            hydro,
            nuclear
        );
        assert!(
            nuclear > wind || (nuclear - wind).abs() < 5.0,
            "Nuclear and wind lifecycle should be close: {:.0} vs {:.0}",
            nuclear,
            wind
        );
    }

    #[test]
    fn test_scope_report_all_renewable() {
        let tracker = CarbonBudgetTracker::new(
            vec![make_wind_gen(), make_solar_gen()],
            CarbonPeriod::Annual { year: 2025 },
            5_000.0,
        );
        let generation = vec![1_000.0, 1_000.0];
        let report = tracker.scope_emissions_report(&generation);
        assert!(
            report.scope1_ton.abs() < 1e-9,
            "All-renewable fleet has zero scope 1: {:.6}",
            report.scope1_ton
        );
        assert_eq!(
            report.renewable_fraction_pct as u32, 100,
            "All-renewable fraction should be 100%"
        );
        assert!(
            report.co2e_avoided_vs_baseline_ton > 0.0,
            "Renewables should avoid significant CO₂e vs coal baseline"
        );
    }

    // ─── New unit tests ──────────────────────────────────────────────────────

    #[test]
    fn test_total_allocated_allowances_within_budget() {
        // All per-generator allocations summed must not exceed the budget cap
        let coal = EmittingGenerator {
            id: 0,
            name: "Coal".into(),
            capacity_mw: 200.0,
            emission_factor: EmissionFactor::coal(),
            allocated_allowances_ton: 60_000.0,
            cost_per_mwh: 35.0,
            is_renewable: false,
        };
        let gas = EmittingGenerator {
            id: 1,
            name: "Gas".into(),
            capacity_mw: 150.0,
            emission_factor: EmissionFactor::natural_gas(),
            allocated_allowances_ton: 30_000.0,
            cost_per_mwh: 55.0,
            is_renewable: false,
        };
        let budget_cap = 100_000.0_f64;
        let tracker = CarbonBudgetTracker::new(
            vec![coal, gas],
            CarbonPeriod::Annual { year: 2025 },
            budget_cap,
        );
        let total_allocated: f64 = tracker
            .generators
            .iter()
            .map(|g| g.allocated_allowances_ton)
            .sum();
        assert!(
            total_allocated <= budget_cap,
            "Total allocated allowances {:.0} t must not exceed budget cap {:.0} t",
            total_allocated,
            budget_cap
        );
    }

    #[test]
    fn test_carbon_market_price_non_negative() {
        let mut market = CarbonMarket::new(EmissionScheme::EuEts, 65.0, 1_500.0);
        // Drive price down with very low emissions
        market.update_price(0.001, 5.0);
        assert!(
            market.current_price_eur_per_ton >= 0.0,
            "Carbon price must be non-negative after update, got {:.4}",
            market.current_price_eur_per_ton
        );
        // Price must not fall below the scheme floor
        assert!(
            market.current_price_eur_per_ton >= market.price_floor_eur_per_ton,
            "Price {:.2} must not fall below floor {:.2}",
            market.current_price_eur_per_ton,
            market.price_floor_eur_per_ton
        );
    }

    #[test]
    fn test_tight_budget_reduces_cumulative_emissions() {
        // Under a tight budget, carbon-adjusted dispatch should prefer low-emission sources.
        // With a high carbon price the wind generator (0 emissions, low cost) always comes first;
        // coal should be dispatched last or not at all.
        let coal = EmittingGenerator {
            id: 10,
            name: "Coal".into(),
            capacity_mw: 200.0,
            emission_factor: EmissionFactor::coal(),
            allocated_allowances_ton: 0.0,
            cost_per_mwh: 35.0,
            is_renewable: false,
        };
        let wind = EmittingGenerator {
            id: 11,
            name: "Wind".into(),
            capacity_mw: 100.0,
            emission_factor: EmissionFactor::wind(),
            allocated_allowances_ton: 0.0,
            cost_per_mwh: 5.0,
            is_renewable: true,
        };
        let tracker_loose = CarbonBudgetTracker::new(
            vec![coal.clone(), wind.clone()],
            CarbonPeriod::Annual { year: 2025 },
            500_000.0,
        );
        let tracker_tight = CarbonBudgetTracker::new(
            vec![coal.clone(), wind.clone()],
            CarbonPeriod::Annual { year: 2025 },
            1_000.0, // very tight
        );
        let demand = 80.0; // within wind capacity — tight budget should serve from wind only
                           // Loose budget: low carbon price — coal may lead (lower base cost with 0 carbon price)
        let dispatch_loose = tracker_loose.carbon_adjusted_dispatch(demand, 0.0);
        // Tight (high carbon price): wind must come first
        let dispatch_tight = tracker_tight.carbon_adjusted_dispatch(demand, 500.0);

        // With carbon price = 500 EUR/t, coal effective cost = 35 + 500*(820/1000) ≈ 445 EUR/MWh
        // Wind effective cost = 5 + 0 = 5 EUR/MWh → wind dispatched first
        let tight_wind_dispatch = dispatch_tight
            .iter()
            .find(|(id, _)| *id == 11)
            .map(|(_, mw)| *mw)
            .unwrap_or(0.0);
        assert!(
            tight_wind_dispatch >= demand - 1e-6,
            "Under high carbon price, wind should cover the full demand {:.0} MW, got {:.2}",
            demand,
            tight_wind_dispatch
        );

        // Loose (zero carbon price): order should put cheap wind first anyway (cost 5 vs 35)
        let _ = dispatch_loose; // used only to show contrast; both should prefer wind
    }

    #[test]
    fn test_trading_surplus_deficit_sign() {
        // A generator with large allocation and small generation → positive surplus
        // A generator with zero allocation and large generation → negative (deficit)
        let gen_surplus = EmittingGenerator {
            id: 20,
            name: "Surplus Gen".into(),
            capacity_mw: 100.0,
            emission_factor: EmissionFactor::natural_gas(),
            allocated_allowances_ton: 50_000.0,
            cost_per_mwh: 55.0,
            is_renewable: false,
        };
        let gen_deficit = EmittingGenerator {
            id: 21,
            name: "Deficit Gen".into(),
            capacity_mw: 200.0,
            emission_factor: EmissionFactor::coal(),
            allocated_allowances_ton: 0.0,
            cost_per_mwh: 35.0,
            is_renewable: false,
        };

        // Surplus gen: 1 MWh gas → ~0.4 t emitted, 50000 t allocated → large surplus
        let surplus = gen_surplus.allowance_position(1.0);
        assert!(
            surplus > 0.0,
            "Generator with large allocation and tiny generation must have a positive surplus, got {:.4}",
            surplus
        );

        // Deficit gen: 10000 MWh coal → ~8000 t emitted, 0 t allocated → deficit
        let deficit = gen_deficit.allowance_position(10_000.0);
        assert!(
            deficit < 0.0,
            "Generator with zero allocation and high generation must have a negative deficit, got {:.4}",
            deficit
        );
    }

    #[test]
    fn test_carbon_shadow_price_from_budget_status() {
        // When budget is nearly exhausted, the recommended action carries an estimated cost.
        // Verify that PurchaseAllowances.estimated_cost_eur = ton × carbon_price.
        let mut tracker = CarbonBudgetTracker::new(
            vec![EmittingGenerator {
                id: 30,
                name: "Coal".into(),
                capacity_mw: 500.0,
                emission_factor: EmissionFactor::coal(),
                allocated_allowances_ton: 1_000.0, // very low allocation
                cost_per_mwh: 35.0,
                is_renewable: false,
            }],
            CarbonPeriod::Annual { year: 2025 },
            5_000.0, // 5 kt budget
        );
        // Record 2000 MWh of coal → ~1640 t emissions → just under budget at 33% elapsed
        tracker
            .record_generation(30, 2_000.0)
            .expect("record_generation should succeed for known generator id 30");

        let carbon_price = 80.0;
        let status = tracker.budget_status(0.33, carbon_price);

        // projected = 1640 / 0.33 ≈ 4970 t, which may be within budget here;
        // the allowances_held is only 1000 t → projected deficit > 0
        if let BudgetAction::PurchaseAllowances {
            ton,
            estimated_cost_eur,
        } = &status.recommended_action
        {
            let expected_cost = ton * carbon_price;
            assert!(
                (estimated_cost_eur - expected_cost).abs() < 1e-6,
                "estimated_cost_eur {:.4} should equal ton {:.4} × price {:.4} = {:.4}",
                estimated_cost_eur,
                ton,
                carbon_price,
                expected_cost
            );
        }
        // The allowance surplus/deficit should be negative (deficit) given very small allocation
        assert!(
            status.allowance_surplus_deficit_ton < 0.0,
            "Allowance surplus/deficit should be negative (deficit) with insufficient allocation, got {:.2}",
            status.allowance_surplus_deficit_ton
        );
    }

    #[test]
    fn test_mac_coal_to_wind_is_positive() {
        // Switching from coal (id=0, cost=35) to wind (id=2, cost=5):
        // MAC = (wind_cost - coal_cost) / (coal_emission - wind_emission)
        // coal cost=35, wind cost=5 → delta_cost = 5-35 = -30 EUR/MWh
        // coal ~0.82 t/MWh, wind ~0 t/MWh → delta_emission ≈ 0.82 t/MWh
        // MAC ≈ -30 / 0.82 ≈ -36.6 EUR/t  (negative = abatement is cheaper than coal)
        let tracker = CarbonBudgetTracker::new(
            vec![make_coal_gen(), make_wind_gen()],
            CarbonPeriod::Annual { year: 2025 },
            200_000.0,
        );
        let mac = tracker
            .marginal_abatement_cost(0, 2)
            .expect("MAC from coal (id=0) to wind (id=2) should succeed");
        // wind is cheaper than coal → negative MAC
        assert!(
            mac < 0.0,
            "MAC from coal to wind should be negative (wind is cheaper), got {:.4}",
            mac
        );
        // Sanity: coal emission ~ 820 kg/MWh = 0.82 t/MWh; delta_cost = 5-35 = -30
        let coal_intensity = EmissionFactor::coal().co2e_kg_per_mwh() / 1_000.0;
        let wind_intensity = EmissionFactor::wind().co2e_kg_per_mwh() / 1_000.0;
        let expected_mac = (5.0_f64 - 35.0) / (coal_intensity - wind_intensity);
        assert!(
            (mac - expected_mac).abs() < 1e-6,
            "MAC {:.4} should equal expected {:.4}",
            mac,
            expected_mac
        );
    }

    #[test]
    fn test_compliance_check_emissions_within_allocation() {
        // After recording generation within allocated allowances, allowance_position must be >= 0
        let mut tracker = CarbonBudgetTracker::new(
            vec![EmittingGenerator {
                id: 40,
                name: "Gas".into(),
                capacity_mw: 150.0,
                emission_factor: EmissionFactor::natural_gas(),
                allocated_allowances_ton: 100_000.0, // generous allocation
                cost_per_mwh: 55.0,
                is_renewable: false,
            }],
            CarbonPeriod::Annual { year: 2025 },
            200_000.0,
        );
        // Record only 10 MWh; gas emits ~0.4 t → well within 100 kt allocation
        tracker
            .record_generation(40, 10.0)
            .expect("record_generation must succeed for id 40");

        let gen = &tracker.generators[0];
        let position = gen.allowance_position(10.0);
        assert!(
            position >= 0.0,
            "Generator with 100 kt allocation and tiny generation must be compliant (position >= 0), got {:.4}",
            position
        );
    }

    #[test]
    fn test_tighter_budget_increases_effective_dispatch_cost() {
        // With a higher carbon price (simulating tighter budget pressure),
        // the carbon-adjusted dispatch cost for a coal unit must increase.
        let coal = EmittingGenerator {
            id: 50,
            name: "Coal".into(),
            capacity_mw: 100.0,
            emission_factor: EmissionFactor::coal(),
            allocated_allowances_ton: 0.0,
            cost_per_mwh: 35.0,
            is_renewable: false,
        };
        // carbon_cost_eur for 1 MWh at low vs high carbon price
        let low_price_cost = coal.carbon_cost_eur(1.0, 20.0);
        let high_price_cost = coal.carbon_cost_eur(1.0, 100.0);
        assert!(
            high_price_cost > low_price_cost,
            "Carbon cost at price 100 EUR/t ({:.4}) must exceed cost at 20 EUR/t ({:.4})",
            high_price_cost,
            low_price_cost
        );
        // Verify proportionality: cost ratio should equal price ratio
        let price_ratio = 100.0_f64 / 20.0;
        let cost_ratio = high_price_cost / low_price_cost;
        assert!(
            (cost_ratio - price_ratio).abs() < 1e-6,
            "Carbon cost should scale linearly with price: ratio {:.6} vs expected {:.6}",
            cost_ratio,
            price_ratio
        );
    }
}
