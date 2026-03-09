//! Electricity market clearing models.
//!
//! Implements:
//! - **Merit order stack**: sort generators by marginal cost, dispatch cheapest first
//! - **Uniform price clearing**: all cleared generators receive the marginal price
//! - **Pay-as-bid clearing**: each generator is paid its own bid price
//! - **Locational Marginal Price (LMP)**: energy + congestion + loss components
//! - **Demand response market**: curtailable load as a virtual resource
//! - **Market statistics**: surplus, uplift, market power index (Lerner index)
//! - **Day-Ahead Market (DAM)**: security-constrained unit commitment + LMP
//! - **Real-Time Market (RTM)**: balancing market with imbalance settlement
//! - **Ancillary Services Market**: spinning/non-spinning reserve, regulation
//! - **DSO Flexibility Market**: local DER flexibility procurement
//!
//! # References
//! - Stoft, S., "Power System Economics", Wiley-IEEE Press, 2002
//! - Kirschen & Strbac, "Fundamentals of Power System Economics", Wiley, 2004
//! - PJM, "PJM Manual 15: Cost Development Guidelines", rev. 2023
//! - Conejo, A.J. et al., "Decomposition Techniques in Mathematical Programming", Springer, 2006
use serde::{Deserialize, Serialize};

pub mod analytics;
pub mod ancillary;
pub mod ancillary_market;
pub mod carbon_budget;
pub mod dam;
pub mod demand_response;
pub mod dso;
pub mod ledger;
pub mod lmp;
pub mod peer_to_peer;
pub mod rec;
pub mod renewable_auction;
pub mod rtm;
pub mod tso_services;

pub use peer_to_peer::{
    BidDirection, ConsumerType, P2pAgent, P2pAnalytics, P2pBid, P2pMarket, P2pMarketClearingResult,
    P2pMechanism, P2pTrade, ProducerType, TradeStatus,
};

pub use renewable_auction::{
    AuctionConfig, AuctionMechanism, AuctionResult, RenewableAuction, RenewableBid, SupportScheme,
};

pub use carbon_budget::{
    // Legacy API (permit allocation, compliance, multi-year planning)
    AllocationMethod,
    AuctionBid,
    AuctionResult as LegacyAuctionResult,
    // New emission-trading market API
    BudgetAction,
    BudgetStatus,
    CarbonAllowance,
    CarbonBudgetConfig,
    CarbonBudgetTracker,
    CarbonDispatchResult,
    CarbonMarket,
    CarbonPeriod,
    ComplianceStatus,
    EmissionFactor,
    EmissionScheme,
    EmittingGenerator,
    GeneratorCarbonProfile,
    GridEmissionIntensity,
    MultiYearCarbonPlan,
    ParetoPoint,
    PermitAllocation,
    PermitTransaction,
    ScopeEmissionsReport,
    TradingResult,
};

pub use ancillary_market::{
    AncillaryBid, AncillaryMarket, AncillaryMarketConfig, AncillaryMarketResult,
    AncillaryRequirements, AncillaryServiceType, ClearingMethod, MarketError,
};

pub use analytics::{
    compute_hhi, herfindahl_hirschman_index, lerner_index, pivotal_supplier_test,
    price_duration_curve, supply_curve_stats, SupplyCurveStats,
};
pub use ancillary::{clear_ancillary_market, AncillaryOffer, AncillaryResult, AncillaryService};
pub use dam::{
    compute_ptdf_matrix, lagrangian_uc, DamBid, DamConfig, DamOffer, DamResult, DayAheadMarket,
};
pub use dso::{
    DsoFlexibilityOffer, DsoFlexibilityRequest, DsoMarket, DsoMarketResult, FlexDirection,
    FlexProvider,
};
pub use lmp::{compute_lmps, LmpComponents};
pub use rtm::{RealTimeMarket, RtmConfig, RtmOffer, RtmResult};
pub use tso_services::{
    AncillaryBid as TsoAncillaryBid, AncillaryProduct, ClearingResult as TsoClearingResult,
    ProductRequirements, TsoMarket,
};

// ─────────────────────────────────────────────────────────────────────────────
// Bid / offer structures
// ─────────────────────────────────────────────────────────────────────────────

/// Generator energy bid (offer to supply).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GeneratorBid {
    /// Unique generator identifier
    pub gen_id: usize,
    /// Bus location (for LMP)
    pub bus_id: usize,
    /// Minimum dispatch (must-run) \[MW\]
    pub p_min_mw: f64,
    /// Maximum capacity offered \[MW\]
    pub p_max_mw: f64,
    /// Marginal cost / offer price \[$/MWh\]
    pub offer_price: f64,
    /// Start-up cost \[$\] (for unit commitment markets)
    pub startup_cost: f64,
    /// No-load (fixed) cost \[$/h\]
    pub no_load_cost: f64,
}

impl GeneratorBid {
    /// Simple bid with zero no-load and startup costs.
    pub fn simple(gen_id: usize, bus_id: usize, p_min: f64, p_max: f64, price: f64) -> Self {
        Self {
            gen_id,
            bus_id,
            p_min_mw: p_min,
            p_max_mw: p_max,
            offer_price: price,
            startup_cost: 0.0,
            no_load_cost: 0.0,
        }
    }
}

/// Demand (load) bid — willingness to pay.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LoadBid {
    /// Load identifier
    pub load_id: usize,
    /// Bus location
    pub bus_id: usize,
    /// Fixed (inelastic) load \[MW\]
    pub p_fixed_mw: f64,
    /// Price-responsive (curtailable) load \[MW\]
    pub p_curtailable_mw: f64,
    /// Willingness-to-pay for the curtailable portion \[$/MWh\]
    pub willingness_to_pay: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Market clearing result
// ─────────────────────────────────────────────────────────────────────────────

/// Dispatch instruction for one generator.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GeneratorDispatch {
    pub gen_id: usize,
    pub bus_id: usize,
    /// Dispatched output \[MW\]
    pub p_dispatched_mw: f64,
    /// Payment received \[$/h\]
    pub payment: f64,
    /// True if the unit is the marginal (price-setting) unit
    pub is_marginal: bool,
}

/// Cleared demand instruction.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LoadDispatch {
    pub load_id: usize,
    pub bus_id: usize,
    /// Served load \[MW\]
    pub p_served_mw: f64,
    /// Curtailed load \[MW\]
    pub p_curtailed_mw: f64,
    /// Payment for served load \[$/h\]
    pub payment: f64,
}

/// Full market clearing result.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MarketClearingResult {
    /// Generator dispatch instructions
    pub gen_dispatches: Vec<GeneratorDispatch>,
    /// Load dispatch instructions
    pub load_dispatches: Vec<LoadDispatch>,
    /// Uniform clearing price \[$/MWh\]
    pub clearing_price: f64,
    /// Total dispatched generation \[MW\]
    pub total_generation_mw: f64,
    /// Total served demand \[MW\]
    pub total_demand_mw: f64,
    /// Total curtailed load \[MW\]
    pub total_curtailed_mw: f64,
    /// Social surplus = consumer + producer surplus \[$/h\]
    pub social_surplus: f64,
    /// Whether the market fully cleared (all demand served)
    pub cleared: bool,
    /// Unserved energy \[MW\] (0 if cleared)
    pub unserved_energy_mw: f64,
}

impl MarketClearingResult {
    /// Market efficiency ratio: social surplus / maximum possible surplus.
    pub fn efficiency_ratio(&self) -> f64 {
        if self.total_demand_mw < 1e-12 {
            return 1.0;
        }
        let max_surplus = self.clearing_price * self.total_demand_mw;
        if max_surplus < 1e-12 {
            return 0.0;
        }
        (self.social_surplus / max_surplus).clamp(0.0, 1.0)
    }

    /// Total payments from demand to generators \[$/h\].
    pub fn total_payments(&self) -> f64 {
        self.gen_dispatches.iter().map(|d| d.payment).sum()
    }

    /// Uplift: total payments minus consumer spend (rent transfer).
    pub fn uplift(&self) -> f64 {
        let consumer_spend: f64 = self.load_dispatches.iter().map(|l| l.payment).sum::<f64>()
            + self.total_demand_mw.min(self.total_generation_mw) * self.clearing_price;
        (self.total_payments() - consumer_spend).abs()
    }

    /// Total system cost [$/h]: sum of payments to all dispatched generators.
    pub fn system_cost(&self) -> f64 {
        self.gen_dispatches.iter().map(|d| d.payment).sum()
    }

    /// Dispatch-weighted average offer price [$/MWh].
    ///
    /// Returns `total_payments / total_dispatched_mw`.
    /// If no generators are dispatched, returns 0.0.
    pub fn weighted_average_offer(&self) -> f64 {
        let total_mw: f64 = self.gen_dispatches.iter().map(|d| d.p_dispatched_mw).sum();
        if total_mw < 1e-12 {
            return 0.0;
        }
        self.system_cost() / total_mw
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Merit order (uniform price) clearing
// ─────────────────────────────────────────────────────────────────────────────

/// Clear the market using uniform (single) pricing with merit order dispatch.
///
/// Algorithm:
/// 1. Sort generators by offer price (ascending)
/// 2. Dispatch in order until demand is met
/// 3. Marginal unit sets the clearing price
/// 4. All dispatched generators paid the clearing price (uniform)
///
/// # Arguments
/// - `bids`         — generator bids (may include must-run with p_min > 0)
/// - `demand_mw`    — total inelastic demand \[MW\]
/// - `load_bids`    — optional price-responsive loads (demand-side bidding)
pub fn uniform_price_clearing(
    bids: &[GeneratorBid],
    demand_mw: f64,
    load_bids: &[LoadBid],
) -> MarketClearingResult {
    let fixed_demand: f64 = demand_mw + load_bids.iter().map(|l| l.p_fixed_mw).sum::<f64>();

    let mut sorted_bids: Vec<&GeneratorBid> = bids.iter().collect();
    sorted_bids.sort_by(|a, b| {
        a.offer_price
            .partial_cmp(&b.offer_price)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let must_run_mw: f64 = sorted_bids.iter().map(|b| b.p_min_mw).sum();
    let mut remaining_demand = (fixed_demand - must_run_mw).max(0.0);

    let mut dispatches: Vec<GeneratorDispatch> = vec![];
    let mut total_gen = must_run_mw;
    let mut clearing_price = 0.0_f64;
    let mut marginal_set = false;

    for bid in &sorted_bids {
        let available = bid.p_max_mw - bid.p_min_mw;
        let dispatched_extra = available.min(remaining_demand);
        let dispatched = bid.p_min_mw + dispatched_extra;

        if dispatched > 0.0 {
            clearing_price = bid.offer_price;
            let is_marginal = dispatched_extra < available - 1e-6 && !marginal_set;
            if is_marginal {
                marginal_set = true;
            }
            dispatches.push(GeneratorDispatch {
                gen_id: bid.gen_id,
                bus_id: bid.bus_id,
                p_dispatched_mw: dispatched,
                payment: 0.0,
                is_marginal,
            });
            total_gen += dispatched_extra;
            remaining_demand -= dispatched_extra;
        }
    }

    for d in dispatches.iter_mut() {
        d.payment = d.p_dispatched_mw * clearing_price;
    }

    let unserved = remaining_demand.max(0.0);
    let cleared = unserved < 1e-6;

    let load_dispatches: Vec<LoadDispatch> = load_bids
        .iter()
        .map(|l| {
            let curtailed = if l.willingness_to_pay < clearing_price {
                l.p_curtailable_mw
            } else {
                0.0
            };
            let served = l.p_fixed_mw + l.p_curtailable_mw - curtailed;
            LoadDispatch {
                load_id: l.load_id,
                bus_id: l.bus_id,
                p_served_mw: served,
                p_curtailed_mw: curtailed,
                payment: served * clearing_price,
            }
        })
        .collect();

    let total_curtailed: f64 = load_dispatches.iter().map(|l| l.p_curtailed_mw).sum();
    let total_demand: f64 = load_dispatches.iter().map(|l| l.p_served_mw).sum::<f64>() + demand_mw;

    let consumer_surplus: f64 = load_bids
        .iter()
        .zip(load_dispatches.iter())
        .map(|(bid, dispatch)| {
            let curtailable_served = dispatch.p_served_mw - bid.p_fixed_mw;
            (bid.willingness_to_pay - clearing_price) * curtailable_served.max(0.0)
        })
        .sum();
    let producer_surplus: f64 = dispatches
        .iter()
        .map(|d| d.p_dispatched_mw * (clearing_price - d.payment / d.p_dispatched_mw.max(1e-12)))
        .sum();
    let social_surplus = consumer_surplus + producer_surplus;

    MarketClearingResult {
        gen_dispatches: dispatches,
        load_dispatches,
        clearing_price,
        total_generation_mw: total_gen,
        total_demand_mw: total_demand,
        total_curtailed_mw: total_curtailed,
        social_surplus,
        cleared,
        unserved_energy_mw: unserved,
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Pay-as-bid clearing
// ─────────────────────────────────────────────────────────────────────────────

/// Clear the market with pay-as-bid (discriminatory) pricing.
///
/// Each dispatched generator receives its own offer price.
/// No single clearing price — prevents "windfall profits" but reduces liquidity.
pub fn pay_as_bid_clearing(bids: &[GeneratorBid], demand_mw: f64) -> MarketClearingResult {
    let mut sorted_bids: Vec<&GeneratorBid> = bids.iter().collect();
    sorted_bids.sort_by(|a, b| {
        a.offer_price
            .partial_cmp(&b.offer_price)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let mut remaining = demand_mw;
    let mut dispatches = vec![];
    let mut total_gen = 0.0;
    let mut clearing_price = 0.0_f64;

    for bid in &sorted_bids {
        if remaining <= 0.0 {
            break;
        }
        let dispatched = bid.p_max_mw.min(remaining + bid.p_min_mw).max(bid.p_min_mw);
        let extra = (dispatched - bid.p_min_mw).max(0.0);
        total_gen += extra;
        remaining -= extra;
        clearing_price = bid.offer_price;
        dispatches.push(GeneratorDispatch {
            gen_id: bid.gen_id,
            bus_id: bid.bus_id,
            p_dispatched_mw: dispatched,
            payment: dispatched * bid.offer_price,
            is_marginal: remaining <= 0.0 && dispatches.is_empty(),
        });
    }

    let unserved = remaining.max(0.0);
    MarketClearingResult {
        clearing_price,
        total_generation_mw: total_gen,
        total_demand_mw: demand_mw - unserved,
        total_curtailed_mw: 0.0,
        load_dispatches: vec![],
        gen_dispatches: dispatches,
        social_surplus: 0.0,
        cleared: unserved < 1e-6,
        unserved_energy_mw: unserved,
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_bids() -> Vec<GeneratorBid> {
        vec![
            GeneratorBid::simple(0, 0, 0.0, 100.0, 20.0),
            GeneratorBid::simple(1, 1, 0.0, 150.0, 35.0),
            GeneratorBid::simple(2, 2, 0.0, 200.0, 50.0),
        ]
    }

    #[test]
    fn test_uniform_price_clears_demand() {
        let bids = sample_bids();
        let result = uniform_price_clearing(&bids, 200.0, &[]);
        assert!(result.cleared, "Market should clear for 200 MW demand");
        assert!(result.unserved_energy_mw < 1e-6);
    }

    #[test]
    fn test_uniform_price_merit_order() {
        let bids = sample_bids();
        let result = uniform_price_clearing(&bids, 80.0, &[]);
        assert!(
            (result.clearing_price - 20.0).abs() < 1e-6,
            "Clearing price should be $20: {}",
            result.clearing_price
        );
    }

    #[test]
    fn test_uniform_price_marginal_setter() {
        let bids = sample_bids();
        let result = uniform_price_clearing(&bids, 200.0, &[]);
        assert!(
            (result.clearing_price - 35.0).abs() < 1e-6,
            "Clearing price should be $35 (G1 sets price): {}",
            result.clearing_price
        );
    }

    #[test]
    fn test_uniform_price_all_units_paid_clearing_price() {
        let bids = sample_bids();
        let result = uniform_price_clearing(&bids, 200.0, &[]);
        for d in &result.gen_dispatches {
            let expected_payment = d.p_dispatched_mw * result.clearing_price;
            assert!(
                (d.payment - expected_payment).abs() < 1e-6,
                "Gen {} payment mismatch",
                d.gen_id
            );
        }
    }

    #[test]
    fn test_uniform_price_excess_demand_not_cleared() {
        let bids = sample_bids();
        let result = uniform_price_clearing(&bids, 500.0, &[]);
        assert!(!result.cleared, "Should not clear with demand > capacity");
        assert!(result.unserved_energy_mw > 0.0);
    }

    #[test]
    fn test_uniform_price_with_load_bids() {
        let bids = sample_bids();
        let load_bids = vec![LoadBid {
            load_id: 0,
            bus_id: 3,
            p_fixed_mw: 0.0,
            p_curtailable_mw: 50.0,
            willingness_to_pay: 100.0,
        }];
        let result = uniform_price_clearing(&bids, 150.0, &load_bids);
        assert!(result.cleared);
    }

    #[test]
    fn test_load_curtailed_below_clearing_price() {
        let bids = vec![GeneratorBid::simple(0, 0, 0.0, 100.0, 40.0)];
        let load_bids = vec![LoadBid {
            load_id: 0,
            bus_id: 1,
            p_fixed_mw: 50.0,
            p_curtailable_mw: 30.0,
            willingness_to_pay: 25.0,
        }];
        let result = uniform_price_clearing(&bids, 50.0, &load_bids);
        assert!(
            result.load_dispatches[0].p_curtailed_mw > 0.0,
            "Load should be curtailed when WTP < clearing price"
        );
    }

    #[test]
    fn test_pay_as_bid_dispatch_order() {
        let bids = sample_bids();
        let result = pay_as_bid_clearing(&bids, 80.0);
        assert!(result.cleared);
        assert_eq!(result.gen_dispatches[0].gen_id, 0);
    }

    #[test]
    fn test_pay_as_bid_each_paid_own_price() {
        let bids = sample_bids();
        let result = pay_as_bid_clearing(&bids, 250.0);
        for d in &result.gen_dispatches {
            let bid_price = bids
                .iter()
                .find(|b| b.gen_id == d.gen_id)
                .map(|b| b.offer_price)
                .unwrap_or(0.0);
            assert!(
                (d.payment - d.p_dispatched_mw * bid_price).abs() < 1e-6,
                "Pay-as-bid: gen {} should be paid own price",
                d.gen_id
            );
        }
    }

    #[test]
    fn test_total_generation_matches_demand() {
        let bids = sample_bids();
        let demand = 200.0;
        let result = uniform_price_clearing(&bids, demand, &[]);
        assert!(
            (result.total_generation_mw - demand).abs() < 1e-6,
            "Gen {:.2} should match demand {:.2}",
            result.total_generation_mw,
            demand
        );
    }
}
