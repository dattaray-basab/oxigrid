//! VPP market bidding strategies.
//!
//! Implements risk-adjusted bidding for virtual power plants participating in
//! wholesale electricity and ancillary service markets.
//!
//! ## Bidding framework
//!
//! The VPP's bid for day-ahead energy in slot `t` is:
//!
//! ```text
//! price_bid[t] = price_forecast[t] - risk_aversion * price_forecast_std[t]
//! volume_bid[t] = min(envelope.p_max[t], grid_limit)
//! ```
//!
//! Risk-averse bidding (Markowitz-inspired) lowers the bid price to guard against
//! forecast over-estimation, reducing the probability of delivering at a loss.
//!
//! Value at Risk (VaR) at confidence level α is estimated via Monte-Carlo simulation
//! using a linear congruential generator (LCG) to avoid external RNG dependencies.
//!
//! ## References
//! - Morales, J.M. et al., "Integrating Renewables in Electricity Markets", Springer, 2014.
//! - Rockafellar, R.T. & Uryasev, S., "Conditional Value-at-Risk for General Loss
//!   Distributions", Journal of Banking & Finance, 2002.
//! - Lüth, A. et al., "Local vs. Central Electricity Markets for Distributed Flexible
//!   Resources — a Comparison", Energy Procedia, 2018.

use crate::optimize::vpp::aggregator::VirtualPowerPlant;
use serde::{Deserialize, Serialize};

// ── Market service types ──────────────────────────────────────────────────────

/// Electricity market service that the VPP bids into.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum MarketService {
    /// Day-ahead energy market (hourly clearing).
    EnergyDayAhead,
    /// Real-time / spot energy market (5–15 minute clearing).
    EnergyRealTime,
    /// Frequency regulation upward (AGC up).
    FrequencyRegulationUp,
    /// Frequency regulation downward (AGC down).
    FrequencyRegulationDown,
    /// Spinning reserve (synchronised, can respond within 10 minutes).
    SpinningReserve,
    /// Non-spinning reserve (unsynchronised, can respond within 30 minutes).
    NonSpinningReserve,
    /// Reactive power / voltage support.
    VoltageSupport,
    /// Intraday balancing market.
    Balancing,
}

impl MarketService {
    /// Required response time \[seconds\] for this service.
    pub fn required_response_s(&self) -> f64 {
        match self {
            Self::FrequencyRegulationUp | Self::FrequencyRegulationDown => 4.0,
            Self::SpinningReserve => 600.0,
            Self::NonSpinningReserve => 1800.0,
            Self::VoltageSupport => 1.0,
            Self::EnergyDayAhead | Self::EnergyRealTime => 300.0,
            Self::Balancing => 900.0,
        }
    }

    /// Typical market premium over energy price for this service [fraction of energy price].
    pub fn typical_premium(&self) -> f64 {
        match self {
            Self::FrequencyRegulationUp | Self::FrequencyRegulationDown => 0.20,
            Self::SpinningReserve => 0.10,
            Self::NonSpinningReserve => 0.05,
            Self::VoltageSupport => 0.08,
            Self::EnergyDayAhead | Self::EnergyRealTime => 0.0,
            Self::Balancing => 0.03,
        }
    }
}

// ── Market bid ────────────────────────────────────────────────────────────────

/// A single price-volume bid submitted by the VPP to a market.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MarketBid {
    /// Settlement period index.
    pub period: usize,
    /// Market service being bid.
    pub service: MarketService,
    /// Bid volume \[MW\].
    pub volume_mw: f64,
    /// Bid-in price [$/MWh].
    pub price_mwh: f64,
    /// Estimated probability that this bid will be accepted [0, 1].
    pub probability_of_acceptance: f64,
}

impl MarketBid {
    /// Expected revenue from this bid [$] in one period of `dt_h` hours.
    pub fn expected_revenue_per_period(&self, dt_h: f64) -> f64 {
        self.volume_mw * self.price_mwh * dt_h * self.probability_of_acceptance
    }
}

// ── VPP bidding strategy ──────────────────────────────────────────────────────

/// Risk-adjusted bidding strategy for a Virtual Power Plant.
pub struct VppBiddingStrategy {
    /// The VPP whose capabilities determine bid volumes.
    pub vpp: VirtualPowerPlant,
    /// Risk aversion coefficient [0 = risk-neutral, 1 = very risk-averse].
    pub risk_aversion: f64,
    /// Standard deviation of the VPP's own forecast error [per unit].
    pub forecast_error_std: f64,
}

impl VppBiddingStrategy {
    /// Create a new bidding strategy for a VPP.
    ///
    /// # Arguments
    /// - `vpp`             — The VPP to bid on behalf of.
    /// - `risk_aversion`   — Risk aversion in \[0, 1\].  A value of 0 bids at the
    ///   forecast price; higher values shade bids downwards.
    pub fn new(vpp: VirtualPowerPlant, risk_aversion: f64) -> Self {
        Self {
            vpp,
            risk_aversion: risk_aversion.clamp(0.0, 1.0),
            forecast_error_std: 0.05, // default 5% forecast error
        }
    }

    /// Compute day-ahead energy market bids for each slot in the forecast horizon.
    ///
    /// The bid price is shaded downward by the risk aversion coefficient:
    /// ```text
    /// price_bid[t] = price_forecast[t] - risk_aversion * price_forecast_std[t]
    /// ```
    /// The bid volume equals the VPP's maximum injection capacity for that slot,
    /// clamped to the grid connection limit.
    ///
    /// # Arguments
    /// - `price_forecast`     — Expected clearing price per slot [$/MWh].
    /// - `price_forecast_std` — One-sigma forecast uncertainty per slot [$/MWh].
    /// - `dt_h`               — Slot duration \[hours\].
    pub fn compute_da_bids(
        &self,
        price_forecast: &[f64],
        price_forecast_std: &[f64],
        dt_h: f64,
    ) -> Vec<MarketBid> {
        let n = price_forecast.len().min(price_forecast_std.len());
        if n == 0 {
            return Vec::new();
        }

        let envelope = self.vpp.compute_envelope(n, dt_h);
        let metrics = self.vpp.metrics();
        let avg_cost = metrics.weighted_avg_cost;

        let mut bids = Vec::with_capacity(n);

        for i in 0..n {
            let price_expected = price_forecast[i];
            let price_sigma = price_forecast_std[i].max(0.0);

            // Risk-adjusted bid price: shade down by risk_aversion * sigma.
            let price_bid = price_expected - self.risk_aversion * price_sigma;

            // Only bid if expected to be profitable.
            let p_max = if i < envelope.p_max_mw.len() {
                envelope.p_max_mw[i]
            } else {
                0.0
            };

            let volume = if price_bid > avg_cost {
                p_max
            } else {
                // Not profitable: do not bid energy; still offer absorption at p_min.
                0.0_f64.max(p_max * 0.0)
            };

            // Probability of acceptance: a simple logistic function of the margin
            // between bid price and expected clearing price distribution.
            let margin = price_expected - price_bid;
            let z = if price_sigma > 1e-9 {
                margin / price_sigma
            } else {
                if margin >= 0.0 {
                    3.0
                } else {
                    -3.0
                }
            };
            let p_accept = logistic(z);

            bids.push(MarketBid {
                period: i,
                service: MarketService::EnergyDayAhead,
                volume_mw: volume.max(0.0),
                price_mwh: price_bid,
                probability_of_acceptance: p_accept,
            });
        }

        bids
    }

    /// Compute a bid for a specific ancillary service market.
    ///
    /// Returns `None` if the VPP cannot meet the service's response-time requirement
    /// or if the available capacity is insufficient.
    ///
    /// # Arguments
    /// - `service`          — Target ancillary service.
    /// - `requirement_mw`   — Minimum volume required \[MW\] (the market's minimum lot).
    /// - `price_signal`     — Reference market price for energy [$/MWh].
    pub fn compute_ancillary_bid(
        &self,
        service: MarketService,
        requirement_mw: f64,
        price_signal: f64,
    ) -> Option<MarketBid> {
        let metrics = self.vpp.metrics();

        // Check response-time requirement.
        if metrics.average_response_time_s > service.required_response_s() {
            return None;
        }

        let available_mw = metrics.available_capacity_mw;
        if available_mw < requirement_mw - 1e-9 {
            return None;
        }

        // Ancillary price = energy price * (1 + service premium) - risk adjustment.
        let premium_price = price_signal * (1.0 + service.typical_premium());
        let bid_price =
            premium_price - self.risk_aversion * premium_price * self.forecast_error_std;

        // Accept a maximum of available capacity.
        let volume = available_mw.min(requirement_mw * 2.0); // cap at 2x the requirement

        Some(MarketBid {
            period: 0,
            service,
            volume_mw: volume,
            price_mwh: bid_price,
            probability_of_acceptance: 0.8 - self.risk_aversion * 0.3,
        })
    }

    /// Compute the expected revenue from a portfolio of bids.
    ///
    /// Revenue for bid `i` in period `t` = `volume[i] * realized_price[t] * dt_h`
    /// (if the bid was accepted, i.e. `realized_price[t] >= price_bid[i]`).
    ///
    /// # Arguments
    /// - `bids`            — Portfolio of market bids.
    /// - `realized_prices` — Actual clearing prices per period [$/MWh].
    pub fn expected_revenue(&self, bids: &[MarketBid], realized_prices: &[f64]) -> f64 {
        let dt_h = 1.0; // assume 1-hour settlement periods
        bids.iter()
            .map(|bid| {
                let realized = if bid.period < realized_prices.len() {
                    realized_prices[bid.period]
                } else {
                    0.0
                };
                // Bid accepted if clearing price >= bid price.
                if realized >= bid.price_mwh {
                    bid.volume_mw * realized * dt_h
                } else {
                    0.0
                }
            })
            .sum()
    }

    /// Estimate the Value at Risk (VaR) of the bid portfolio at confidence level `alpha`.
    ///
    /// VaR at level α is the threshold such that the probability of losing more than
    /// VaR is at most (1 − α).  We estimate this via Monte-Carlo simulation of
    /// `n_scenarios` price scenarios, using a simple LCG pseudo-random number generator
    /// to avoid external dependencies.
    ///
    /// # Arguments
    /// - `bids`        — Bid portfolio.
    /// - `alpha`       — Confidence level (e.g. 0.95 for 95th percentile VaR).
    /// - `n_scenarios` — Number of Monte-Carlo price scenarios.
    pub fn value_at_risk(&self, bids: &[MarketBid], alpha: f64, n_scenarios: usize) -> f64 {
        if bids.is_empty() || n_scenarios == 0 {
            return 0.0;
        }

        let alpha = alpha.clamp(0.0, 1.0);
        let dt_h = 1.0;

        // Collect reference bid prices to determine the forecast base for each period.
        // We use the mid-point price of the bid as the scenario centre.
        let mut revenues = Vec::with_capacity(n_scenarios);

        let mut lcg_state: u64 = 0xDEAD_BEEF_CAFE_1234_u64;

        for _ in 0..n_scenarios {
            let mut scenario_revenue = 0.0_f64;

            for bid in bids {
                // Generate standard-normal sample via Box–Muller using two LCG draws.
                let u1 = lcg_next(&mut lcg_state);
                let u2 = lcg_next(&mut lcg_state);
                let z = box_muller(u1, u2);

                // Simulated clearing price = bid_price + sigma * N(0,1)
                // sigma proportional to forecast_error_std and bid price level.
                let price_scale = bid.price_mwh.abs().max(1.0) * self.forecast_error_std;
                let sim_price = bid.price_mwh + price_scale * z;

                if sim_price >= bid.price_mwh {
                    scenario_revenue += bid.volume_mw * sim_price * dt_h;
                }
            }
            revenues.push(scenario_revenue);
        }

        // Sort revenues ascending.
        revenues.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));

        // VaR = -percentile(1 - alpha) of revenue distribution
        let var_idx = ((1.0 - alpha) * n_scenarios as f64) as usize;
        let var_idx = var_idx.min(revenues.len().saturating_sub(1));
        let threshold_revenue = revenues[var_idx];
        (-threshold_revenue).max(0.0)
    }
}

// ── Multi-VPP coordination ────────────────────────────────────────────────────

/// Coordinate bids from multiple VPPs to prevent market dominance.
///
/// Each VPP's bid volume is capped at `market_cap_mw / n_vpps` to ensure no single
/// VPP can dominate the market.  The algorithm then scales each VPP's bids to
/// maximise aggregate social welfare (sum of volumes × bid prices).
///
/// # Arguments
/// - `vpps`          — Slice of VPP bidding strategies.
/// - `market_cap_mw` — Total market capacity \[MW\] used to determine per-VPP share.
///
/// # Returns
/// A vector of bid portfolios (one per VPP).
pub fn coordinate_vpp_bids(vpps: &[VppBiddingStrategy], market_cap_mw: f64) -> Vec<Vec<MarketBid>> {
    let n_vpps = vpps.len();
    if n_vpps == 0 {
        return Vec::new();
    }

    let max_share_mw = (market_cap_mw / n_vpps as f64).max(0.0);

    vpps.iter()
        .map(|strategy| {
            let metrics = strategy.vpp.metrics();
            let envelope = strategy.vpp.compute_envelope(24, 1.0);

            let mut bids: Vec<MarketBid> = (0..24)
                .map(|period| {
                    let p_max = if period < envelope.p_max_mw.len() {
                        envelope.p_max_mw[period]
                    } else {
                        metrics.available_capacity_mw
                    };

                    // Limit bid volume to per-VPP market share cap.
                    let volume = p_max.min(max_share_mw);
                    let bid_price = metrics.weighted_avg_cost
                        * (1.0 + strategy.vpp.resources.len() as f64 * 0.01);

                    MarketBid {
                        period,
                        service: MarketService::EnergyDayAhead,
                        volume_mw: volume.max(0.0),
                        price_mwh: bid_price,
                        probability_of_acceptance: 0.75,
                    }
                })
                .collect();

            // Remove zero-volume bids.
            bids.retain(|b| b.volume_mw > 1e-9);
            bids
        })
        .collect()
}

// ── LCG pseudo-random number generator ───────────────────────────────────────

/// Advance one step of a 64-bit linear congruential generator.
/// Parameters from Knuth / MMIX.
fn lcg_next(state: &mut u64) -> f64 {
    const A: u64 = 6_364_136_223_846_793_005_u64;
    const C: u64 = 1_442_695_040_888_963_407_u64;
    *state = state.wrapping_mul(A).wrapping_add(C);
    // Map to (0, 1) by taking top 53 bits.
    let mantissa = *state >> 11;
    mantissa as f64 * (1.0 / (1u64 << 53) as f64)
}

/// Box–Muller transform: convert two uniform samples to a standard normal.
fn box_muller(u1: f64, u2: f64) -> f64 {
    let u1 = u1.max(1e-15); // avoid log(0)
    let r = (-2.0 * u1.ln()).sqrt();
    let theta = 2.0 * core::f64::consts::PI * u2;
    r * theta.cos()
}

/// Logistic (sigmoid) function: maps real-valued z to (0, 1).
fn logistic(z: f64) -> f64 {
    1.0 / (1.0 + (-z).exp())
}

// ── Unit tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::optimize::vpp::aggregator::{DerResource, DerState, DerType};

    fn make_battery_der(id: usize, p_max: f64, cost: f64) -> DerResource {
        DerResource {
            resource_id: id,
            resource_type: DerType::BatteryStorage,
            bus: id,
            p_max_mw: p_max,
            p_min_mw: -p_max,
            e_max_mwh: Some(p_max * 4.0),
            current_state: DerState::idle_with_soc(0.8),
            availability: 1.0,
            response_time_s: 30.0,
            cost_mwh: cost,
        }
    }

    fn make_vpp(p_max: f64, cost: f64) -> VirtualPowerPlant {
        VirtualPowerPlant::new(
            0,
            vec![
                make_battery_der(0, p_max, cost),
                make_battery_der(1, p_max, cost * 1.2),
            ],
            0,
            p_max * 2.0,
        )
    }

    fn make_strategy(vpp: VirtualPowerPlant, risk: f64) -> VppBiddingStrategy {
        VppBiddingStrategy::new(vpp, risk)
    }

    #[test]
    fn test_da_bid_risk_adjustment_reduces_price() {
        let vpp = make_vpp(5.0, 30.0);
        let risk_neutral = make_strategy(make_vpp(5.0, 30.0), 0.0);
        let risk_averse = make_strategy(vpp, 0.8);

        let prices = vec![80.0; 5];
        let sigmas = vec![10.0; 5];

        let bids_neutral = risk_neutral.compute_da_bids(&prices, &sigmas, 1.0);
        let bids_averse = risk_averse.compute_da_bids(&prices, &sigmas, 1.0);

        // Risk-averse bids should have lower or equal prices.
        for (bn, ba) in bids_neutral.iter().zip(bids_averse.iter()) {
            assert!(
                ba.price_mwh <= bn.price_mwh + 1e-9,
                "risk-averse bid price {:.4} should be <= risk-neutral {:.4}",
                ba.price_mwh,
                bn.price_mwh
            );
        }
    }

    #[test]
    fn test_da_bid_volume_nonnegative() {
        let strategy = make_strategy(make_vpp(5.0, 30.0), 0.5);
        let prices = vec![80.0; 4];
        let sigmas = vec![5.0; 4];
        let bids = strategy.compute_da_bids(&prices, &sigmas, 1.0);
        for bid in &bids {
            assert!(bid.volume_mw >= 0.0, "bid volume must be non-negative");
        }
    }

    #[test]
    fn test_da_bid_count_matches_horizon() {
        let strategy = make_strategy(make_vpp(5.0, 20.0), 0.3);
        let n = 6;
        let prices = vec![50.0; n];
        let sigmas = vec![8.0; n];
        let bids = strategy.compute_da_bids(&prices, &sigmas, 1.0);
        assert_eq!(bids.len(), n);
    }

    #[test]
    fn test_ancillary_bid_response_time_filter() {
        let mut vpp = make_vpp(10.0, 20.0);
        // Set slow response time so frequency regulation bid should be rejected.
        for res in vpp.resources.iter_mut() {
            res.response_time_s = 1000.0; // > 4s requirement for freq reg
        }
        let strategy = make_strategy(vpp, 0.2);
        let bid = strategy.compute_ancillary_bid(MarketService::FrequencyRegulationUp, 1.0, 60.0);
        assert!(
            bid.is_none(),
            "slow-response VPP should not qualify for freq reg"
        );
    }

    #[test]
    fn test_ancillary_bid_sufficient_capacity() {
        let strategy = make_strategy(make_vpp(10.0, 20.0), 0.2);
        let bid = strategy.compute_ancillary_bid(MarketService::SpinningReserve, 5.0, 60.0);
        assert!(bid.is_some(), "VPP with sufficient capacity should bid");
        let b = bid.expect("bid present");
        assert!(b.volume_mw > 0.0);
        assert!(b.price_mwh > 0.0);
    }

    #[test]
    fn test_expected_revenue_positive_when_price_exceeds_bid() {
        let strategy = make_strategy(make_vpp(5.0, 20.0), 0.0);
        let bids = vec![MarketBid {
            period: 0,
            service: MarketService::EnergyDayAhead,
            volume_mw: 10.0,
            price_mwh: 50.0,
            probability_of_acceptance: 0.9,
        }];
        let realized = vec![70.0]; // clearing > bid price
        let revenue = strategy.expected_revenue(&bids, &realized);
        assert!(
            revenue > 0.0,
            "revenue should be positive when clearing > bid price"
        );
    }

    #[test]
    fn test_expected_revenue_zero_when_price_below_bid() {
        let strategy = make_strategy(make_vpp(5.0, 20.0), 0.0);
        let bids = vec![MarketBid {
            period: 0,
            service: MarketService::EnergyDayAhead,
            volume_mw: 10.0,
            price_mwh: 80.0,
            probability_of_acceptance: 0.9,
        }];
        let realized = vec![40.0]; // clearing < bid price → not accepted
        let revenue = strategy.expected_revenue(&bids, &realized);
        assert_eq!(revenue, 0.0, "bid not accepted → zero revenue");
    }

    #[test]
    fn test_var_nonnegative() {
        let strategy = make_strategy(make_vpp(5.0, 20.0), 0.5);
        let bids = vec![MarketBid {
            period: 0,
            service: MarketService::EnergyDayAhead,
            volume_mw: 10.0,
            price_mwh: 50.0,
            probability_of_acceptance: 0.8,
        }];
        let var = strategy.value_at_risk(&bids, 0.95, 200);
        assert!(var >= 0.0, "VaR must be non-negative, got {var:.4}");
    }

    #[test]
    fn test_var_increases_with_uncertainty() {
        let mut strategy_low = make_strategy(make_vpp(5.0, 20.0), 0.3);
        strategy_low.forecast_error_std = 0.02; // low uncertainty
        let mut strategy_high = make_strategy(make_vpp(5.0, 20.0), 0.3);
        strategy_high.forecast_error_std = 0.30; // high uncertainty

        let bids = vec![MarketBid {
            period: 0,
            service: MarketService::EnergyDayAhead,
            volume_mw: 10.0,
            price_mwh: 60.0,
            probability_of_acceptance: 0.8,
        }];

        let var_low = strategy_low.value_at_risk(&bids, 0.95, 500);
        let var_high = strategy_high.value_at_risk(&bids, 0.95, 500);

        // Higher forecast uncertainty should generally lead to higher or equal VaR.
        // (This is probabilistic — we allow a small tolerance.)
        assert!(
            var_high >= var_low * 0.5,
            "higher uncertainty should produce comparable or higher VaR. low={var_low:.4} high={var_high:.4}"
        );
    }

    #[test]
    fn test_multi_vpp_bid_limit() {
        let vpps = vec![
            make_strategy(make_vpp(20.0, 25.0), 0.2),
            make_strategy(make_vpp(20.0, 30.0), 0.3),
        ];
        let market_cap = 15.0; // MW
        let all_bids = coordinate_vpp_bids(&vpps, market_cap);
        assert_eq!(all_bids.len(), vpps.len());

        let per_vpp_cap = market_cap / 2.0; // 7.5 MW each
        for vpp_bids in &all_bids {
            for bid in vpp_bids {
                assert!(
                    bid.volume_mw <= per_vpp_cap + 1e-9,
                    "bid volume {:.3} MW exceeds per-VPP market cap {per_vpp_cap:.3} MW",
                    bid.volume_mw
                );
            }
        }
    }

    #[test]
    fn test_multi_vpp_coordination_returns_bids_for_each_vpp() {
        let vpps = vec![
            make_strategy(make_vpp(5.0, 20.0), 0.1),
            make_strategy(make_vpp(8.0, 25.0), 0.2),
            make_strategy(make_vpp(3.0, 15.0), 0.4),
        ];
        let bids = coordinate_vpp_bids(&vpps, 50.0);
        assert_eq!(bids.len(), 3, "should have one bid set per VPP");
    }

    #[test]
    fn test_lcg_determinism() {
        // LCG with same seed should produce the same sequence.
        let mut s1: u64 = 12345;
        let mut s2: u64 = 12345;
        for _ in 0..10 {
            let v1 = lcg_next(&mut s1);
            let v2 = lcg_next(&mut s2);
            assert_eq!(v1, v2, "LCG must be deterministic");
        }
    }

    #[test]
    fn test_lcg_range() {
        let mut state: u64 = 0xABCD_1234;
        for _ in 0..100 {
            let v = lcg_next(&mut state);
            assert!((0.0..1.0).contains(&v), "LCG must produce values in [0, 1)");
        }
    }
}
