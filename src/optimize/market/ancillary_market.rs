//! Comprehensive Ancillary Services Market — procurement and settlement.
//!
//! Implements:
//! - Multi-service ancillary market (primary/secondary/tertiary reserve,
//!   frequency regulation up/down, reactive support, black-start, voltage control)
//! - Three clearing methods: uniform price, pay-as-bid, merit order
//! - Performance-weighted bid evaluation
//! - Herfindahl-Hirschman Index (HHI) market concentration
//! - Shortage detection per service type
//!
//! # References
//! - FERC Order 755, "Frequency Regulation Compensation", 2011
//! - NERC BAL-003-1, "Frequency Response and Frequency Bias Setting", 2022
//! - Ela, E. et al., "Ancillary Services in the United States", NREL/TP-5500-62708, 2014

use thiserror::Error;

// ─────────────────────────────────────────────────────────────────────────────
// Error type
// ─────────────────────────────────────────────────────────────────────────────

/// Errors that can occur during ancillary market clearing.
#[derive(Debug, Error)]
pub enum MarketError {
    /// Invalid interval index.
    #[error("interval {interval} out of range (n_intervals={n_intervals})")]
    InvalidInterval { interval: usize, n_intervals: usize },
    /// Negative capacity in bid.
    #[error("bid {bid_id} has non-positive capacity {capacity_mw:.3} MW")]
    InvalidCapacity { bid_id: usize, capacity_mw: f64 },
    /// Performance score out of \[0, 1\].
    #[error("bid {bid_id} has performance score {score:.3} outside [0, 1]")]
    InvalidPerformanceScore { bid_id: usize, score: f64 },
}

// ─────────────────────────────────────────────────────────────────────────────
// Market configuration
// ─────────────────────────────────────────────────────────────────────────────

/// Ancillary service type.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum AncillaryServiceType {
    /// Fast-response primary reserve (< 10 s).
    PrimaryReserve,
    /// AGC-range secondary reserve (< 30 s).
    SecondaryReserve,
    /// Slow-response tertiary reserve (< 15 min).
    TertiaryReserve,
    /// Regulation-up (frequency regulation fast raise).
    FrequencyRegulationUp,
    /// Regulation-down (frequency regulation fast lower).
    FrequencyRegulationDown,
    /// Reactive power / voltage support \[MVAr\].
    ReactiveSupport,
    /// Black-start restoration capability.
    BlackStart,
    /// Voltage control (AVR support).
    VoltageControl,
}

impl AncillaryServiceType {
    /// All service types in a canonical order.
    pub fn all() -> [AncillaryServiceType; 8] {
        [
            AncillaryServiceType::PrimaryReserve,
            AncillaryServiceType::SecondaryReserve,
            AncillaryServiceType::TertiaryReserve,
            AncillaryServiceType::FrequencyRegulationUp,
            AncillaryServiceType::FrequencyRegulationDown,
            AncillaryServiceType::ReactiveSupport,
            AncillaryServiceType::BlackStart,
            AncillaryServiceType::VoltageControl,
        ]
    }

    /// Human-readable label.
    pub fn label(&self) -> &'static str {
        match self {
            AncillaryServiceType::PrimaryReserve => "PrimaryReserve",
            AncillaryServiceType::SecondaryReserve => "SecondaryReserve",
            AncillaryServiceType::TertiaryReserve => "TertiaryReserve",
            AncillaryServiceType::FrequencyRegulationUp => "FreqRegUp",
            AncillaryServiceType::FrequencyRegulationDown => "FreqRegDown",
            AncillaryServiceType::ReactiveSupport => "ReactiveSupport",
            AncillaryServiceType::BlackStart => "BlackStart",
            AncillaryServiceType::VoltageControl => "VoltageControl",
        }
    }
}

/// System-level requirements for each ancillary service \[MW\] or \[MVAr\].
#[derive(Debug, Clone)]
pub struct AncillaryRequirements {
    /// Fast primary reserve requirement \[MW\].
    pub primary_reserve_mw: f64,
    /// AGC secondary reserve requirement \[MW\].
    pub secondary_reserve_mw: f64,
    /// Slow tertiary reserve requirement \[MW\].
    pub tertiary_reserve_mw: f64,
    /// Reactive reserve requirement \[MVAr\].
    pub reactive_reserve_mvar: f64,
    /// Black-start capability requirement \[MW\].
    pub black_start_mw: f64,
    /// Frequency regulation requirement \[MW\] (shared up/down).
    pub frequency_regulation_mw: f64,
}

impl AncillaryRequirements {
    /// Return the requirement for a given service type.
    pub fn requirement_for(&self, svc: &AncillaryServiceType) -> f64 {
        match svc {
            AncillaryServiceType::PrimaryReserve => self.primary_reserve_mw,
            AncillaryServiceType::SecondaryReserve => self.secondary_reserve_mw,
            AncillaryServiceType::TertiaryReserve => self.tertiary_reserve_mw,
            AncillaryServiceType::FrequencyRegulationUp => self.frequency_regulation_mw,
            AncillaryServiceType::FrequencyRegulationDown => self.frequency_regulation_mw,
            AncillaryServiceType::ReactiveSupport => self.reactive_reserve_mvar,
            AncillaryServiceType::BlackStart => self.black_start_mw,
            AncillaryServiceType::VoltageControl => 0.0,
        }
    }
}

/// Price-determination method used for clearing.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ClearingMethod {
    /// All accepted bids pay the marginal clearing price (uniform).
    UniformPrice,
    /// Each provider is paid their own bid price.
    PayAsBid,
    /// Lowest cost first; clearing price = marginal offer price (merit order).
    MeritOrder,
}

/// Top-level market configuration.
#[derive(Debug, Clone)]
pub struct AncillaryMarketConfig {
    /// Market interval duration \[h\].
    pub market_interval_h: f64,
    /// Number of intervals in the planning window.
    pub n_intervals: usize,
    /// Day-ahead forward horizon \[h\].
    pub forward_horizon_h: usize,
    /// Mandatory service requirements.
    pub requirements: AncillaryRequirements,
    /// Bid clearing methodology.
    pub clearing_method: ClearingMethod,
}

// ─────────────────────────────────────────────────────────────────────────────
// Bid structure
// ─────────────────────────────────────────────────────────────────────────────

/// Ancillary services bid from a capacity provider.
#[derive(Debug, Clone)]
pub struct AncillaryBid {
    /// Unique bid identifier (assigned externally).
    pub bid_id: usize,
    /// Providing entity identifier.
    pub provider_id: usize,
    /// Ancillary service being offered.
    pub service: AncillaryServiceType,
    /// Offered capacity \[MW\] (or \[MVAr\] for reactive services).
    pub capacity_mw: f64,
    /// Capacity payment requested \[USD/MW\] per interval.
    pub price_usd_per_mw: f64,
    /// Historical performance score in \[0, 1\] (1 = perfect delivery).
    pub performance_score: f64,
    /// Per-interval availability flags (length must equal n_intervals).
    pub availability_hours: Vec<bool>,
}

// ─────────────────────────────────────────────────────────────────────────────
// Result structures
// ─────────────────────────────────────────────────────────────────────────────

/// A single awarded ancillary service contract.
#[derive(Debug, Clone)]
pub struct AncillaryAward {
    /// Original bid identifier.
    pub bid_id: usize,
    /// Providing entity identifier.
    pub provider_id: usize,
    /// Service awarded.
    pub service: AncillaryServiceType,
    /// Awarded capacity \[MW\].
    pub awarded_mw: f64,
    /// Clearing price applied to this award \[USD/MW\].
    pub clearing_price_usd_per_mw: f64,
    /// Total payment for this award \[USD\].
    pub payment_usd: f64,
    /// Interval for which this award applies.
    pub interval: usize,
}

/// Full result of one interval's ancillary market clearing.
#[derive(Debug, Clone)]
pub struct AncillaryMarketResult {
    /// All cleared awards for this interval.
    pub awards: Vec<AncillaryAward>,
    /// Marginal clearing price per service type \[(type, USD/MW)\].
    pub clearing_prices: Vec<(AncillaryServiceType, f64)>,
    /// Whether each requirement was fully met \[(type, met)\].
    pub requirements_met: Vec<(AncillaryServiceType, bool)>,
    /// Total procurement cost across all services \[USD\].
    pub total_procurement_cost_usd: f64,
    /// Herfindahl-Hirschman Index (HHI) over total awarded capacity.
    /// Range 0–10000; 10000 = monopoly.
    pub market_concentration: f64,
    /// Capacity shortage per service \[(type, shortage_mw)\].
    pub shortage_mw: Vec<(AncillaryServiceType, f64)>,
}

// ─────────────────────────────────────────────────────────────────────────────
// Market implementation
// ─────────────────────────────────────────────────────────────────────────────

/// Comprehensive ancillary services market.
///
/// # Workflow
/// 1. Create with [`AncillaryMarket::new`].
/// 2. Submit bids via [`AncillaryMarket::submit_bid`].
/// 3. Call [`AncillaryMarket::clear`] for each desired interval.
pub struct AncillaryMarket {
    config: AncillaryMarketConfig,
    bids: Vec<AncillaryBid>,
}

impl AncillaryMarket {
    /// Create a new empty ancillary market.
    pub fn new(config: AncillaryMarketConfig) -> Self {
        Self {
            config,
            bids: Vec::new(),
        }
    }

    /// Submit a bid to the market.
    ///
    /// Returns an error if `capacity_mw <= 0` or `performance_score` is
    /// outside \[0, 1\].
    pub fn submit_bid(&mut self, bid: AncillaryBid) -> Result<(), MarketError> {
        if bid.capacity_mw <= 0.0 {
            return Err(MarketError::InvalidCapacity {
                bid_id: bid.bid_id,
                capacity_mw: bid.capacity_mw,
            });
        }
        if !(0.0..=1.0).contains(&bid.performance_score) {
            return Err(MarketError::InvalidPerformanceScore {
                bid_id: bid.bid_id,
                score: bid.performance_score,
            });
        }
        self.bids.push(bid);
        Ok(())
    }

    /// Performance-weighted effective price for a bid.
    ///
    /// `effective_price = bid_price / performance_score`.
    /// A performance score of 0 is treated as a very large penalty price.
    pub fn performance_weighted_price(&self, bid: &AncillaryBid) -> f64 {
        if bid.performance_score < 1e-12 {
            f64::MAX / 2.0
        } else {
            bid.price_usd_per_mw / bid.performance_score
        }
    }

    /// Clear the ancillary market for a single interval.
    ///
    /// For each service type:
    /// 1. Filter eligible bids (available in `interval`, positive capacity).
    /// 2. Sort by performance-weighted price (merit order).
    /// 3. Select bids until requirement is met or stack exhausted.
    /// 4. Determine payment based on [`ClearingMethod`].
    /// 5. Compute HHI over total awarded capacity.
    pub fn clear(&self, interval: usize) -> Result<AncillaryMarketResult, MarketError> {
        if interval >= self.config.n_intervals {
            return Err(MarketError::InvalidInterval {
                interval,
                n_intervals: self.config.n_intervals,
            });
        }

        let mut all_awards: Vec<AncillaryAward> = Vec::new();
        let mut clearing_prices: Vec<(AncillaryServiceType, f64)> = Vec::new();
        let mut requirements_met: Vec<(AncillaryServiceType, bool)> = Vec::new();
        let mut shortage_mw: Vec<(AncillaryServiceType, f64)> = Vec::new();

        for svc in AncillaryServiceType::all() {
            let requirement = self.config.requirements.requirement_for(&svc);

            // Filter eligible bids for this service and interval
            let mut eligible: Vec<&AncillaryBid> = self
                .bids
                .iter()
                .filter(|b| {
                    b.service == svc
                        && b.capacity_mw > 0.0
                        && b.availability_hours.get(interval).copied().unwrap_or(false)
                })
                .collect();

            // Sort by performance-weighted price (ascending)
            eligible.sort_by(|a, b| {
                let pa = self.performance_weighted_price(a);
                let pb = self.performance_weighted_price(b);
                pa.partial_cmp(&pb).unwrap_or(std::cmp::Ordering::Equal)
            });

            let mut remaining = requirement;
            let mut service_awards: Vec<(&AncillaryBid, f64)> = Vec::new(); // (bid, awarded_mw)
            let mut marginal_price = 0.0_f64;

            for bid in &eligible {
                if remaining <= 1e-9 {
                    break;
                }
                let awarded = bid.capacity_mw.min(remaining);
                if awarded > 0.0 {
                    marginal_price = bid.price_usd_per_mw;
                    service_awards.push((bid, awarded));
                    remaining -= awarded;
                }
            }

            let shortage = remaining.max(0.0);
            let met = shortage < 1e-6;

            // Determine clearing price per ClearingMethod
            let clearing_price = match self.config.clearing_method {
                ClearingMethod::UniformPrice | ClearingMethod::MeritOrder => marginal_price,
                ClearingMethod::PayAsBid => 0.0, // each pays own price (no single price)
            };

            // Build awards
            for (bid, awarded_mw) in &service_awards {
                let pay_price = match self.config.clearing_method {
                    ClearingMethod::UniformPrice | ClearingMethod::MeritOrder => clearing_price,
                    ClearingMethod::PayAsBid => bid.price_usd_per_mw,
                };
                all_awards.push(AncillaryAward {
                    bid_id: bid.bid_id,
                    provider_id: bid.provider_id,
                    service: svc.clone(),
                    awarded_mw: *awarded_mw,
                    clearing_price_usd_per_mw: pay_price,
                    payment_usd: awarded_mw * pay_price * self.config.market_interval_h,
                    interval,
                });
            }

            clearing_prices.push((svc.clone(), clearing_price));
            requirements_met.push((svc.clone(), met));
            shortage_mw.push((svc, shortage));
        }

        let total_procurement_cost_usd: f64 = all_awards.iter().map(|a| a.payment_usd).sum();

        // Compute HHI over all awarded capacity by provider
        let market_concentration = compute_hhi_from_awards(&all_awards);

        Ok(AncillaryMarketResult {
            awards: all_awards,
            clearing_prices,
            requirements_met,
            total_procurement_cost_usd,
            market_concentration,
            shortage_mw,
        })
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// HHI computation
// ─────────────────────────────────────────────────────────────────────────────

/// Compute the Herfindahl-Hirschman Index from a slice of awards.
///
/// `HHI = Σ (s_i)² × 10000`
/// where `s_i` is provider `i`'s share of total awarded capacity.
fn compute_hhi_from_awards(awards: &[AncillaryAward]) -> f64 {
    let total: f64 = awards.iter().map(|a| a.awarded_mw).sum();
    if total < 1e-12 {
        return 0.0;
    }

    // Aggregate by provider_id
    let mut provider_mw: std::collections::HashMap<usize, f64> = std::collections::HashMap::new();
    for award in awards {
        *provider_mw.entry(award.provider_id).or_insert(0.0) += award.awarded_mw;
    }

    provider_mw
        .values()
        .map(|&mw| {
            let share = mw / total;
            share * share * 10_000.0
        })
        .sum()
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_config(method: ClearingMethod) -> AncillaryMarketConfig {
        AncillaryMarketConfig {
            market_interval_h: 1.0,
            n_intervals: 24,
            forward_horizon_h: 24,
            requirements: AncillaryRequirements {
                primary_reserve_mw: 50.0,
                secondary_reserve_mw: 30.0,
                tertiary_reserve_mw: 20.0,
                reactive_reserve_mvar: 40.0,
                black_start_mw: 10.0,
                frequency_regulation_mw: 25.0,
            },
            clearing_method: method,
        }
    }

    fn avail(n: usize) -> Vec<bool> {
        vec![true; n]
    }

    #[test]
    fn test_all_requirements_met_no_shortage() {
        let config = make_config(ClearingMethod::MeritOrder);
        let n = config.n_intervals;
        let mut market = AncillaryMarket::new(config);

        // Submit generous bids for primary reserve
        market
            .submit_bid(AncillaryBid {
                bid_id: 0,
                provider_id: 0,
                service: AncillaryServiceType::PrimaryReserve,
                capacity_mw: 100.0,
                price_usd_per_mw: 10.0,
                performance_score: 1.0,
                availability_hours: avail(n),
            })
            .unwrap();

        let result = market.clear(0).unwrap();

        let primary_shortage = result
            .shortage_mw
            .iter()
            .find(|(s, _)| *s == AncillaryServiceType::PrimaryReserve)
            .map(|(_, mw)| *mw)
            .unwrap_or(0.0);

        assert!(
            primary_shortage < 1e-6,
            "Primary reserve shortage should be zero: {primary_shortage}"
        );

        let primary_met = result
            .requirements_met
            .iter()
            .find(|(s, _)| *s == AncillaryServiceType::PrimaryReserve)
            .map(|(_, m)| *m)
            .unwrap_or(false);

        assert!(primary_met, "Primary reserve requirement should be met");
    }

    #[test]
    fn test_insufficient_bids_shortage_reported() {
        let config = make_config(ClearingMethod::MeritOrder);
        let n = config.n_intervals;
        let mut market = AncillaryMarket::new(config);

        // Only 10 MW available, requirement is 50 MW
        market
            .submit_bid(AncillaryBid {
                bid_id: 0,
                provider_id: 0,
                service: AncillaryServiceType::PrimaryReserve,
                capacity_mw: 10.0,
                price_usd_per_mw: 5.0,
                performance_score: 0.9,
                availability_hours: avail(n),
            })
            .unwrap();

        let result = market.clear(0).unwrap();

        let shortage = result
            .shortage_mw
            .iter()
            .find(|(s, _)| *s == AncillaryServiceType::PrimaryReserve)
            .map(|(_, mw)| *mw)
            .unwrap_or(0.0);

        assert!(
            (shortage - 40.0).abs() < 1e-6,
            "Shortage should be 40 MW: {shortage}"
        );

        let met = result
            .requirements_met
            .iter()
            .find(|(s, _)| *s == AncillaryServiceType::PrimaryReserve)
            .map(|(_, m)| *m)
            .unwrap_or(true);

        assert!(!met, "Requirement should NOT be met when shortage exists");
    }

    #[test]
    fn test_uniform_price_all_pay_same_clearing_price() {
        let config = make_config(ClearingMethod::UniformPrice);
        let n = config.n_intervals;
        let mut market = AncillaryMarket::new(config);

        market
            .submit_bid(AncillaryBid {
                bid_id: 0,
                provider_id: 0,
                service: AncillaryServiceType::SecondaryReserve,
                capacity_mw: 20.0,
                price_usd_per_mw: 8.0,
                performance_score: 1.0,
                availability_hours: avail(n),
            })
            .unwrap();
        market
            .submit_bid(AncillaryBid {
                bid_id: 1,
                provider_id: 1,
                service: AncillaryServiceType::SecondaryReserve,
                capacity_mw: 20.0,
                price_usd_per_mw: 15.0,
                performance_score: 1.0,
                availability_hours: avail(n),
            })
            .unwrap();

        let result = market.clear(0).unwrap();

        let awards: Vec<&AncillaryAward> = result
            .awards
            .iter()
            .filter(|a| a.service == AncillaryServiceType::SecondaryReserve)
            .collect();

        assert!(
            !awards.is_empty(),
            "Should have awards for SecondaryReserve"
        );

        // All awards should have the same clearing price (= marginal bid = 15.0)
        let prices: Vec<f64> = awards.iter().map(|a| a.clearing_price_usd_per_mw).collect();
        let first = prices[0];
        for p in &prices {
            assert!(
                (p - first).abs() < 1e-9,
                "Uniform price: all awards should have same price, got {p} vs {first}"
            );
        }
        // Marginal price should be 15.0 (second cheapest sets the price when 30 MW needed)
        assert!(
            (first - 15.0).abs() < 1e-9,
            "Clearing price should be 15.0 (marginal bid): {first}"
        );
    }

    #[test]
    fn test_pay_as_bid_each_pays_own_price() {
        let config = make_config(ClearingMethod::PayAsBid);
        let n = config.n_intervals;
        let mut market = AncillaryMarket::new(config);

        let bid_prices = [(0usize, 8.0_f64), (1, 15.0)];
        for (id, price) in &bid_prices {
            market
                .submit_bid(AncillaryBid {
                    bid_id: *id,
                    provider_id: *id,
                    service: AncillaryServiceType::SecondaryReserve,
                    capacity_mw: 20.0,
                    price_usd_per_mw: *price,
                    performance_score: 1.0,
                    availability_hours: avail(n),
                })
                .unwrap();
        }

        let result = market.clear(0).unwrap();

        for award in result
            .awards
            .iter()
            .filter(|a| a.service == AncillaryServiceType::SecondaryReserve)
        {
            let expected_price = bid_prices
                .iter()
                .find(|(id, _)| *id == award.bid_id)
                .map(|(_, p)| *p)
                .unwrap_or(0.0);
            assert!(
                (award.clearing_price_usd_per_mw - expected_price).abs() < 1e-9,
                "Pay-as-bid: award {} should have price {}, got {}",
                award.bid_id,
                expected_price,
                award.clearing_price_usd_per_mw
            );
        }
    }

    #[test]
    fn test_hhi_monopoly_equals_10000() {
        let config = make_config(ClearingMethod::MeritOrder);
        let n = config.n_intervals;
        let mut market = AncillaryMarket::new(config);

        // Single provider for all services
        let all_services = [
            (0usize, AncillaryServiceType::PrimaryReserve, 100.0_f64),
            (1, AncillaryServiceType::SecondaryReserve, 100.0),
            (2, AncillaryServiceType::TertiaryReserve, 100.0),
            (3, AncillaryServiceType::FrequencyRegulationUp, 100.0),
            (4, AncillaryServiceType::FrequencyRegulationDown, 100.0),
            (5, AncillaryServiceType::ReactiveSupport, 100.0),
            (6, AncillaryServiceType::BlackStart, 100.0),
            (7, AncillaryServiceType::VoltageControl, 100.0),
        ];
        for (bid_id, svc, cap) in all_services {
            market
                .submit_bid(AncillaryBid {
                    bid_id,
                    provider_id: 0, // single provider
                    service: svc,
                    capacity_mw: cap,
                    price_usd_per_mw: 5.0,
                    performance_score: 1.0,
                    availability_hours: avail(n),
                })
                .unwrap();
        }

        let result = market.clear(0).unwrap();

        assert!(
            (result.market_concentration - 10_000.0).abs() < 1e-3,
            "HHI should be 10000 for monopoly: {}",
            result.market_concentration
        );
    }

    #[test]
    fn test_invalid_interval_returns_error() {
        let config = make_config(ClearingMethod::MeritOrder);
        let market = AncillaryMarket::new(config);
        let result = market.clear(999);
        assert!(result.is_err(), "Should error for out-of-range interval");
    }

    #[test]
    fn test_negative_capacity_bid_rejected() {
        let config = make_config(ClearingMethod::MeritOrder);
        let n = config.n_intervals;
        let mut market = AncillaryMarket::new(config);
        let result = market.submit_bid(AncillaryBid {
            bid_id: 0,
            provider_id: 0,
            service: AncillaryServiceType::PrimaryReserve,
            capacity_mw: -10.0,
            price_usd_per_mw: 5.0,
            performance_score: 1.0,
            availability_hours: avail(n),
        });
        assert!(result.is_err(), "Negative capacity bid should be rejected");
    }

    #[test]
    fn test_performance_weighted_price() {
        let config = make_config(ClearingMethod::MeritOrder);
        let market = AncillaryMarket::new(config.clone());

        let bid = AncillaryBid {
            bid_id: 0,
            provider_id: 0,
            service: AncillaryServiceType::PrimaryReserve,
            capacity_mw: 10.0,
            price_usd_per_mw: 20.0,
            performance_score: 0.5,
            availability_hours: vec![true; config.n_intervals],
        };

        let effective = market.performance_weighted_price(&bid);
        assert!(
            (effective - 40.0).abs() < 1e-9,
            "Effective price should be 20/0.5=40: {effective}"
        );
    }

    #[test]
    fn test_merit_order_favors_better_performance() {
        // Two bids with same nominal price but different performance
        // Higher performance → lower effective price → chosen first
        let config = make_config(ClearingMethod::MeritOrder);
        let n = config.n_intervals;
        let mut market = AncillaryMarket::new(config);

        market
            .submit_bid(AncillaryBid {
                bid_id: 0,
                provider_id: 0,
                service: AncillaryServiceType::PrimaryReserve,
                capacity_mw: 30.0,
                price_usd_per_mw: 10.0,
                performance_score: 0.5, // effective = 20.0
                availability_hours: avail(n),
            })
            .unwrap();
        market
            .submit_bid(AncillaryBid {
                bid_id: 1,
                provider_id: 1,
                service: AncillaryServiceType::PrimaryReserve,
                capacity_mw: 30.0,
                price_usd_per_mw: 10.0,
                performance_score: 1.0, // effective = 10.0 ← cheaper
                availability_hours: avail(n),
            })
            .unwrap();

        let result = market.clear(0).unwrap();

        // First award should be from bid_id=1 (better performance)
        let first_award = result
            .awards
            .iter()
            .find(|a| a.service == AncillaryServiceType::PrimaryReserve);
        assert!(first_award.is_some());
        assert_eq!(
            first_award.unwrap().bid_id,
            1,
            "Better-performing bid should be selected first"
        );
    }
}
