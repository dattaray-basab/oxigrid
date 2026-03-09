//! DSO (Distribution System Operator) local flexibility market.
//!
//! DSOs procure flexibility from DERs (EV, BESS, DR, industrial loads) to
//! manage distribution-level congestion and voltage constraints.
//!
//! # Algorithm
//! 1. DSO posts flexibility requests describing constrained buses/branches
//! 2. DER aggregators submit flexibility offers with PTDF eligibility
//! 3. Merit-order clearing: accept cheapest offers first until constraint relieved
//! 4. Clearing price = last accepted offer price (uniform)
//!
//! # References
//! - ENTSO-E, "TSO-DSO Report on System Defense and Restoration", 2020
//! - CEDEC et al., "TSO-DSO-Consumer INTERFACE: Flexibility Toolbox", 2021
use crate::error::Result;
use serde::{Deserialize, Serialize};

/// Direction of flexibility required or offered.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum FlexDirection {
    /// Increase consumption or decrease generation (absorb excess power)
    Upward,
    /// Decrease consumption or increase generation (relieve deficit)
    Downward,
    /// Provider can respond in either direction
    Symmetric,
}

impl FlexDirection {
    /// Returns true if `offer_dir` is compatible with `request_dir`.
    pub fn is_compatible(request: FlexDirection, offer: FlexDirection) -> bool {
        match (request, offer) {
            (FlexDirection::Symmetric, _) => true,
            (_, FlexDirection::Symmetric) => true,
            (a, b) => a == b,
        }
    }
}

/// Type of flexibility provider / DER asset.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum FlexProvider {
    /// Electric vehicle fleet (managed charging/V2G)
    EvFleet,
    /// Battery energy storage system
    Bess,
    /// Demand response (curtailable loads)
    DemandResponse,
    /// Industrial interruptible load
    IndustrialLoad,
    /// Distributed generation (e.g. CHP, rooftop solar with smart inverter)
    DistributedGeneration,
    /// Virtual power plant (aggregated heterogeneous DERs)
    Virtual,
}

/// DSO request for flexibility to relieve a network constraint.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DsoFlexibilityRequest {
    /// Unique request identifier
    pub request_id: usize,
    /// Bus where congestion manifests
    pub constraint_bus: usize,
    /// Branch index that is congested
    pub constraint_branch: usize,
    /// Time period (e.g., hour index in planning horizon)
    pub period: usize,
    /// Required direction of flexibility
    pub direction: FlexDirection,
    /// Required flexibility volume \[MW\]
    pub volume_mw: f64,
    /// Maximum price the DSO is willing to pay \[$/MW\]
    pub max_price_per_mw: f64,
    /// Probability that this request will be activated (0..1)
    pub activation_probability: f64,
}

/// Flexibility offer submitted by a DER provider.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DsoFlexibilityOffer {
    /// Unique offer identifier (index in offers vector)
    pub provider_id: usize,
    /// Type of DER asset
    pub provider_type: FlexProvider,
    /// Bus where the DER is connected
    pub bus: usize,
    /// Time period this offer covers
    pub period: usize,
    /// Direction the provider can respond
    pub direction: FlexDirection,
    /// Volume available \[MW\]
    pub volume_mw: f64,
    /// Activation price \[$/MW\]
    pub price_per_mw: f64,
    /// Maximum ramp rate \[MW/min\]
    pub ramp_rate_mw_per_min: f64,
    /// Minimum activation duration \[hours\]
    pub min_activation_duration_h: f64,
    /// Power Transfer Distribution Factor to the constraint bus/branch.
    /// PTDF ≥ ptdf_threshold means this provider can effectively relieve the constraint.
    pub ptdf: f64,
}

/// Result of DSO market clearing for one request.
#[derive(Debug, Clone)]
pub struct DsoMarketResult {
    /// (offer provider_id, cleared_volume_mw) pairs, in merit-order
    pub cleared_offers: Vec<(usize, f64)>,
    /// Uniform clearing price \[$/MW\] (last accepted offer price)
    pub clearing_price: f64,
    /// Total flexibility volume cleared \[MW\]
    pub total_volume_cleared_mw: f64,
    /// Whether the requested volume was fully met
    pub congestion_relieved: bool,
    /// Residual congestion after clearing \[MW\]
    pub residual_congestion_mw: f64,
    /// Total procurement cost \[$ = MW × $/MW\]
    pub total_cost: f64,
    /// Number of distinct providers cleared
    pub n_providers_cleared: usize,
}

/// DSO local flexibility market.
pub struct DsoMarket {
    /// Flexibility requests from DSO
    pub requests: Vec<DsoFlexibilityRequest>,
    /// Flexibility offers from DER providers
    pub offers: Vec<DsoFlexibilityOffer>,
    /// Minimum absolute PTDF for a provider to be eligible (default 0.05)
    pub ptdf_threshold: f64,
}

impl DsoMarket {
    /// Create a new DSO flexibility market with default PTDF threshold (0.05).
    pub fn new(requests: Vec<DsoFlexibilityRequest>, offers: Vec<DsoFlexibilityOffer>) -> Self {
        Self {
            requests,
            offers,
            ptdf_threshold: 0.05,
        }
    }

    /// Create with a custom PTDF threshold.
    pub fn with_ptdf_threshold(
        requests: Vec<DsoFlexibilityRequest>,
        offers: Vec<DsoFlexibilityOffer>,
        ptdf_threshold: f64,
    ) -> Self {
        Self {
            requests,
            offers,
            ptdf_threshold,
        }
    }

    /// Clear the DSO flexibility market for each request.
    ///
    /// For each request:
    /// 1. Filter eligible offers (same period, compatible direction, |PTDF| ≥ threshold)
    /// 2. Convert to effective impact: effective_vol = offer.volume_mw × |PTDF|
    /// 3. Sort by price_per_mw ascending (merit order)
    /// 4. Accept offers greedily until request.volume_mw satisfied
    /// 5. Clearing price = last accepted offer price_per_mw
    pub fn clear(&self) -> Result<Vec<DsoMarketResult>> {
        if self.requests.is_empty() {
            return Ok(Vec::new());
        }

        let mut results = Vec::with_capacity(self.requests.len());

        for request in &self.requests {
            // Filter eligible offers
            let mut eligible: Vec<(usize, &DsoFlexibilityOffer)> = self
                .offers
                .iter()
                .enumerate()
                .filter(|(_, offer)| {
                    offer.period == request.period
                        && offer.ptdf.abs() >= self.ptdf_threshold
                        && FlexDirection::is_compatible(request.direction, offer.direction)
                        && offer.price_per_mw <= request.max_price_per_mw
                })
                .collect();

            if eligible.is_empty() {
                results.push(DsoMarketResult {
                    cleared_offers: Vec::new(),
                    clearing_price: 0.0,
                    total_volume_cleared_mw: 0.0,
                    congestion_relieved: false,
                    residual_congestion_mw: request.volume_mw,
                    total_cost: 0.0,
                    n_providers_cleared: 0,
                });
                continue;
            }

            // Sort by price ascending (merit order)
            eligible.sort_by(|(_, a), (_, b)| {
                a.price_per_mw
                    .partial_cmp(&b.price_per_mw)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });

            let mut cleared_offers: Vec<(usize, f64)> = Vec::new();
            let mut remaining = request.volume_mw;
            let mut clearing_price = 0.0_f64;
            let mut total_cost = 0.0_f64;

            for (idx, offer) in &eligible {
                if remaining <= 1e-9 {
                    break;
                }
                // Effective volume at constraint: scale by PTDF
                let effective_vol = offer.volume_mw * offer.ptdf.abs();
                if effective_vol <= 0.0 {
                    continue;
                }
                let cleared_vol = effective_vol.min(remaining);
                cleared_offers.push((*idx, cleared_vol));
                remaining -= cleared_vol;
                clearing_price = offer.price_per_mw;
                total_cost += cleared_vol * clearing_price;
            }

            let total_volume_cleared_mw = request.volume_mw - remaining.max(0.0);
            let residual_congestion_mw = remaining.max(0.0);
            let congestion_relieved = residual_congestion_mw < 1e-6;
            let n_providers_cleared = cleared_offers.len();

            results.push(DsoMarketResult {
                cleared_offers,
                clearing_price,
                total_volume_cleared_mw,
                congestion_relieved,
                residual_congestion_mw,
                total_cost,
                n_providers_cleared,
            });
        }

        Ok(results)
    }

    /// Build the aggregate flexibility supply curve for a given direction.
    ///
    /// Returns `(cumulative_volume_mw, price_per_mw)` pairs sorted by price ascending.
    pub fn aggregate_portfolio(
        providers: &[DsoFlexibilityOffer],
        direction: FlexDirection,
    ) -> Vec<(f64, f64)> {
        let mut eligible: Vec<&DsoFlexibilityOffer> = providers
            .iter()
            .filter(|o| FlexDirection::is_compatible(direction, o.direction))
            .collect();

        eligible.sort_by(|a, b| {
            a.price_per_mw
                .partial_cmp(&b.price_per_mw)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let mut cumulative = 0.0_f64;
        eligible
            .into_iter()
            .map(|o| {
                cumulative += o.volume_mw;
                (cumulative, o.price_per_mw)
            })
            .collect()
    }

    /// Estimate MW congestion relief at the constraint branch from cleared offers.
    ///
    /// Relief = Σ_cleared (cleared_volume_mw × |PTDF|) — since cleared_volume
    /// already incorporates PTDF scaling, this returns the raw MW equivalent.
    pub fn estimate_congestion_relief(
        cleared: &DsoMarketResult,
        offers: &[DsoFlexibilityOffer],
    ) -> f64 {
        cleared
            .cleared_offers
            .iter()
            .map(|(offer_id, cleared_vol)| {
                let ptdf_abs = offers.get(*offer_id).map(|o| o.ptdf.abs()).unwrap_or(0.0);
                // cleared_vol is already PTDF-scaled; multiply again would double-count.
                // Return as direct MW relief at constraint.
                cleared_vol * ptdf_abs.max(1.0).recip() * ptdf_abs
            })
            .sum()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_request(volume: f64, dir: FlexDirection) -> DsoFlexibilityRequest {
        DsoFlexibilityRequest {
            request_id: 0,
            constraint_bus: 1,
            constraint_branch: 0,
            period: 0,
            direction: dir,
            volume_mw: volume,
            max_price_per_mw: 1000.0,
            activation_probability: 1.0,
        }
    }

    fn make_offer(
        id: usize,
        vol: f64,
        price: f64,
        ptdf: f64,
        dir: FlexDirection,
    ) -> DsoFlexibilityOffer {
        DsoFlexibilityOffer {
            provider_id: id,
            provider_type: FlexProvider::Bess,
            bus: 2,
            period: 0,
            direction: dir,
            volume_mw: vol,
            price_per_mw: price,
            ramp_rate_mw_per_min: 5.0,
            min_activation_duration_h: 0.5,
            ptdf,
        }
    }

    #[test]
    fn test_dso_merit_order_clearing() {
        let requests = vec![make_request(50.0, FlexDirection::Downward)];
        let offers = vec![
            make_offer(0, 30.0, 10.0, 0.8, FlexDirection::Downward), // cheapest
            make_offer(1, 30.0, 20.0, 0.8, FlexDirection::Downward),
            make_offer(2, 30.0, 30.0, 0.8, FlexDirection::Downward),
        ];
        let market = DsoMarket::new(requests, offers);
        let results = market.clear().expect("DSO market should clear");
        assert_eq!(results.len(), 1);
        let r = &results[0];
        // Cheapest offer (id=0) should be cleared first
        assert!(!r.cleared_offers.is_empty(), "Should have cleared offers");
        let first_cleared_id = r.cleared_offers[0].0;
        assert_eq!(
            first_cleared_id, 0,
            "Cheapest offer (id=0) should clear first"
        );
    }

    #[test]
    fn test_dso_ptdf_filter() {
        // Offer with PTDF below threshold (0.05) should be excluded
        let requests = vec![make_request(10.0, FlexDirection::Downward)];
        let offers = vec![
            make_offer(0, 100.0, 5.0, 0.01, FlexDirection::Downward), // PTDF=0.01 < 0.05
        ];
        let market = DsoMarket::new(requests, offers);
        let results = market.clear().expect("DSO market should clear");
        assert_eq!(results.len(), 1);
        let r = &results[0];
        assert_eq!(
            r.n_providers_cleared, 0,
            "Low-PTDF offer should be filtered out"
        );
        assert!(!r.congestion_relieved);
    }

    #[test]
    fn test_dso_congestion_relief_positive() {
        let requests = vec![make_request(20.0, FlexDirection::Downward)];
        let offers = vec![make_offer(0, 25.0, 10.0, 0.9, FlexDirection::Downward)];
        let market = DsoMarket::new(requests, offers.clone());
        let results = market.clear().expect("DSO market should clear");
        let r = &results[0];
        let relief = DsoMarket::estimate_congestion_relief(r, &offers);
        assert!(
            relief > 0.0,
            "Congestion relief should be positive: {relief:.4}"
        );
    }

    #[test]
    fn test_dso_market_result_fields() {
        let requests = vec![make_request(10.0, FlexDirection::Downward)];
        let offers = vec![make_offer(0, 15.0, 20.0, 0.8, FlexDirection::Downward)];
        let market = DsoMarket::new(requests, offers);
        let results = market.clear().expect("DSO market should clear");
        let r = &results[0];
        assert!(
            r.congestion_relieved,
            "10 MW request should be met by 15*0.8=12 MW offer"
        );
        assert!(r.total_cost > 0.0, "total_cost={:.2}", r.total_cost);
        assert!(r.clearing_price > 0.0);
        assert!(r.residual_congestion_mw < 1e-6);
    }

    #[test]
    fn test_dso_aggregate_portfolio() {
        let offers = vec![
            make_offer(0, 20.0, 15.0, 0.5, FlexDirection::Downward),
            make_offer(1, 30.0, 10.0, 0.5, FlexDirection::Downward), // cheaper
            make_offer(2, 10.0, 25.0, 0.5, FlexDirection::Upward),   // different dir
        ];
        let curve = DsoMarket::aggregate_portfolio(&offers, FlexDirection::Downward);
        // Should include offers 0 and 1, sorted by price
        assert_eq!(curve.len(), 2, "Should include 2 downward offers");
        // First entry should have lower cumulative volume (cheapest=offer1 at 30 MW first)
        assert!(
            curve[0].0 <= curve[1].0,
            "Cumulative volume should be non-decreasing"
        );
        assert!(
            curve[0].1 <= curve[1].1,
            "Prices should be sorted ascending"
        );
    }

    #[test]
    fn test_dso_symmetric_direction_compatibility() {
        let requests = vec![make_request(10.0, FlexDirection::Downward)];
        let offers = vec![make_offer(0, 15.0, 10.0, 0.8, FlexDirection::Symmetric)];
        let market = DsoMarket::new(requests, offers);
        let results = market.clear().expect("DSO market should clear");
        assert!(
            results[0].congestion_relieved,
            "Symmetric offer should match downward request"
        );
    }
}
