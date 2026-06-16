use serde::{Deserialize, Serialize};

/// Method used to allocate free emission permits to generators.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum AllocationMethod {
    /// Allocate based on historical emissions (grandfathering / free allocation).
    Grandfathering,
    /// Allocate based on emission intensity benchmark × capacity.
    Benchmarking,
    /// No free allocation — all permits sold at auction.
    Auctioning,
    /// Partial free allocation: `pct_free` fraction via grandfathering, rest auctioned.
    HybridAuction {
        /// Fraction of permits allocated for free (0.0–1.0).
        pct_free: f64,
    },
}

/// Carbon emission profile and cost data for a single generator.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GeneratorCarbonProfile {
    pub unit_id: String,
    pub pmax_mw: f64,
    pub pmin_mw: f64,
    pub marginal_cost_per_mwh: f64,
    pub emission_rate_t_co2_per_mwh: f64,
    pub free_permits_t_co2: f64,
}

/// Configuration for the carbon budget / cap-and-trade system.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CarbonBudgetConfig {
    pub total_budget_t_co2: f64,
    pub permit_price_per_t: f64,
    pub banking_allowed: bool,
    pub borrowing_allowed: bool,
    pub price_floor_per_t: f64,
    pub price_ceiling_per_t: f64,
    pub planning_horizon_years: usize,
}

impl Default for CarbonBudgetConfig {
    fn default() -> Self {
        Self {
            total_budget_t_co2: 1_000_000.0,
            permit_price_per_t: 50.0,
            banking_allowed: true,
            borrowing_allowed: false,
            price_floor_per_t: 20.0,
            price_ceiling_per_t: 200.0,
            planning_horizon_years: 10,
        }
    }
}

/// Record of a single permit transfer between entities.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PermitTransaction {
    pub buyer: String,
    pub seller: String,
    pub quantity_t: f64,
    pub price_per_t: f64,
}

/// Permit allocation result for one generator.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PermitAllocation {
    pub unit_id: String,
    pub allocated_t_co2: f64,
    pub method: String,
}

/// A bid submitted in a permit auction.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AuctionBid {
    pub bidder_id: String,
    pub quantity_t: f64,
    pub price_per_t: f64,
}

/// Result of a uniform-price permit auction (legacy).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AuctionResult {
    pub clearing_price_per_t: f64,
    pub total_permits_sold: f64,
    pub revenue_usd: f64,
    pub unsold_permits: f64,
}

/// Result of carbon-constrained economic dispatch.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CarbonDispatchResult {
    pub dispatch_mw: Vec<f64>,
    pub total_cost_usd: f64,
    pub total_emissions_t_co2: f64,
    pub permit_cost_usd: f64,
    pub permit_surplus_t: f64,
}

/// One point on the cost–emissions Pareto front.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ParetoPoint {
    pub carbon_price: f64,
    pub total_cost_usd: f64,
    pub total_emissions_t: f64,
}

/// Multi-year carbon plan covering the planning horizon.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MultiYearCarbonPlan {
    pub annual_dispatch: Vec<Vec<f64>>,
    pub annual_emissions: Vec<f64>,
    pub annual_permit_cost: Vec<f64>,
    pub banked_permits: Vec<f64>,
    pub total_npv_cost: f64,
}

/// Compliance status for a regulated entity.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ComplianceStatus {
    pub net_permit_position_t: f64,
    pub compliant: bool,
    pub penalty_usd: f64,
    pub recommendation: String,
}

/// Result of a bilateral permit trading round.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TradingResult {
    pub transactions: Vec<PermitTransaction>,
    pub total_volume_t: f64,
    pub clearing_price: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_carbon_budget_config_default() {
        let cfg = CarbonBudgetConfig::default();
        assert_eq!(cfg.total_budget_t_co2, 1_000_000.0);
        assert_eq!(cfg.permit_price_per_t, 50.0);
        assert!(cfg.banking_allowed, "banking_allowed should be true");
        assert!(!cfg.borrowing_allowed, "borrowing_allowed should be false");
        assert_eq!(cfg.price_floor_per_t, 20.0);
        assert_eq!(cfg.price_ceiling_per_t, 200.0);
        assert_eq!(cfg.planning_horizon_years, 10);
    }

    #[test]
    fn test_allocation_method_debug() {
        let grandfathering = AllocationMethod::Grandfathering;
        let repr_g = format!("{:?}", grandfathering);
        assert!(
            repr_g.contains("Grandfathering"),
            "expected 'Grandfathering' in debug output, got: {}",
            repr_g
        );

        let hybrid = AllocationMethod::HybridAuction { pct_free: 0.3 };
        let repr_h = format!("{:?}", hybrid);
        assert!(
            repr_h.contains("HybridAuction"),
            "expected 'HybridAuction' in debug output, got: {}",
            repr_h
        );
    }

    #[test]
    fn test_generator_carbon_profile_fields() {
        let profile = GeneratorCarbonProfile {
            unit_id: "GEN_42".to_string(),
            pmax_mw: 500.0,
            pmin_mw: 50.0,
            marginal_cost_per_mwh: 35.0,
            emission_rate_t_co2_per_mwh: 0.45,
            free_permits_t_co2: 1200.0,
        };

        assert_eq!(profile.unit_id, "GEN_42");
        assert_eq!(profile.pmax_mw, 500.0);
        assert_eq!(profile.pmin_mw, 50.0);
        assert_eq!(profile.marginal_cost_per_mwh, 35.0);
        assert_eq!(profile.emission_rate_t_co2_per_mwh, 0.45);
        assert_eq!(profile.free_permits_t_co2, 1200.0);
    }

    #[test]
    fn test_permit_transaction_fields() {
        let txn = PermitTransaction {
            buyer: "BUYER_A".to_string(),
            seller: "SELLER_B".to_string(),
            quantity_t: 500.0,
            price_per_t: 55.0,
        };

        assert_eq!(txn.buyer, "BUYER_A");
        assert_eq!(txn.seller, "SELLER_B");
        assert_eq!(txn.quantity_t, 500.0);
        assert_eq!(txn.price_per_t, 55.0);
    }

    #[test]
    fn test_auction_bid_and_result() {
        let bid = AuctionBid {
            bidder_id: "UTILITY_X".to_string(),
            quantity_t: 1000.0,
            price_per_t: 62.5,
        };

        assert_eq!(bid.bidder_id, "UTILITY_X");
        assert_eq!(bid.quantity_t, 1000.0);
        assert_eq!(bid.price_per_t, 62.5);

        let result = AuctionResult {
            clearing_price_per_t: 60.0,
            total_permits_sold: 9500.0,
            revenue_usd: 570_000.0,
            unsold_permits: 500.0,
        };

        assert_eq!(result.clearing_price_per_t, 60.0);
        assert_eq!(result.total_permits_sold, 9500.0);
        assert_eq!(result.revenue_usd, 570_000.0);
        assert_eq!(result.unsold_permits, 500.0);
    }

    #[test]
    fn test_carbon_dispatch_result_non_trivial() {
        let dispatch = CarbonDispatchResult {
            dispatch_mw: vec![100.0, 200.0, 50.0],
            total_cost_usd: 18_500.0,
            total_emissions_t_co2: 157.5,
            permit_cost_usd: 7_875.0,
            permit_surplus_t: 42.5,
        };

        assert_eq!(dispatch.dispatch_mw.len(), 3);
        assert!(
            dispatch.total_emissions_t_co2 > 0.0,
            "total_emissions_t_co2 must be positive"
        );
        assert_eq!(dispatch.permit_surplus_t, 42.5);
    }

    #[test]
    fn test_multi_year_plan_compliance_trading() {
        let num_years = 5_usize;

        let plan = MultiYearCarbonPlan {
            annual_dispatch: vec![vec![100.0, 200.0]; num_years],
            annual_emissions: vec![150.0, 145.0, 140.0, 135.0, 130.0],
            annual_permit_cost: vec![7_500.0; num_years],
            banked_permits: vec![50.0; num_years],
            total_npv_cost: 35_000.0,
        };

        assert_eq!(
            plan.annual_emissions.len(),
            num_years,
            "annual_emissions length should equal num_years"
        );

        let compliance = ComplianceStatus {
            net_permit_position_t: 200.0,
            compliant: true,
            penalty_usd: 0.0,
            recommendation: "No action required".to_string(),
        };

        assert!(
            compliance.compliant,
            "ComplianceStatus.compliant should be true"
        );

        let num_transactions = 3_usize;
        let transactions: Vec<PermitTransaction> = (0..num_transactions)
            .map(|i| PermitTransaction {
                buyer: format!("BUYER_{}", i),
                seller: format!("SELLER_{}", i),
                quantity_t: 100.0 * (i + 1) as f64,
                price_per_t: 55.0,
            })
            .collect();

        let trading = TradingResult {
            transactions,
            total_volume_t: 600.0,
            clearing_price: 55.0,
        };

        assert_eq!(
            trading.transactions.len(),
            num_transactions,
            "TradingResult.transactions length should equal num_transactions"
        );
    }
}
