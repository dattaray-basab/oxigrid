//! Market power analytics: HHI, Lerner index, price-duration curve, supply statistics.
//!
//! # References
//! - U.S. DOJ/FTC, "Horizontal Merger Guidelines", 2010
//! - Stoft, S., "Power System Economics", Wiley-IEEE Press, 2002
use serde::{Deserialize, Serialize};

use super::{GeneratorBid, MarketClearingResult};

/// Supply curve statistics for market analysis.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SupplyCurveStats {
    /// Total installed capacity \[MW\]
    pub total_capacity_mw: f64,
    /// Capacity below clearing price (cleared) \[MW\]
    pub cleared_capacity_mw: f64,
    /// Capacity at clearing price (infra-marginal) \[MW\]
    pub infra_marginal_mw: f64,
    /// Offered range: (min_price, max_price) \[$/MWh\]
    pub price_range: (f64, f64),
    /// Weighted average offer price \[$/MWh\]
    pub weighted_avg_offer: f64,
}

/// Compute supply curve statistics from bids and clearing result.
pub fn supply_curve_stats(
    bids: &[GeneratorBid],
    result: &MarketClearingResult,
) -> SupplyCurveStats {
    let total_cap: f64 = bids.iter().map(|b| b.p_max_mw).sum();
    let cleared_cap: f64 = result
        .gen_dispatches
        .iter()
        .map(|d| d.p_dispatched_mw)
        .sum();

    let dispatched_ids: std::collections::HashSet<usize> =
        result.gen_dispatches.iter().map(|d| d.gen_id).collect();
    let infra_marginal: f64 = result
        .gen_dispatches
        .iter()
        .filter(|d| !d.is_marginal)
        .map(|d| d.p_dispatched_mw)
        .sum();

    let min_p = bids
        .iter()
        .map(|b| b.offer_price)
        .fold(f64::INFINITY, f64::min);
    let max_p = bids
        .iter()
        .map(|b| b.offer_price)
        .fold(f64::NEG_INFINITY, f64::max);

    let total_offer_mw: f64 = bids
        .iter()
        .filter(|b| dispatched_ids.contains(&b.gen_id))
        .map(|b| b.p_max_mw)
        .sum();
    let weighted_avg = if total_offer_mw > 1e-6 {
        bids.iter()
            .filter(|b| dispatched_ids.contains(&b.gen_id))
            .map(|b| b.offer_price * b.p_max_mw)
            .sum::<f64>()
            / total_offer_mw
    } else {
        0.0
    };

    SupplyCurveStats {
        total_capacity_mw: total_cap,
        cleared_capacity_mw: cleared_cap,
        infra_marginal_mw: infra_marginal,
        price_range: (min_p, max_p),
        weighted_avg_offer: weighted_avg,
    }
}

/// Herfindahl-Hirschman Index (HHI) for market concentration.
///
/// HHI = Σ (market_share_i)^2 × 10000
/// HHI < 1500 → competitive; 1500–2500 → moderately concentrated; >2500 → concentrated
pub fn herfindahl_hirschman_index(output_by_firm: &[f64]) -> f64 {
    let total: f64 = output_by_firm.iter().sum();
    if total < 1e-12 {
        return 0.0;
    }
    output_by_firm
        .iter()
        .map(|&q| (q / total * 100.0).powi(2))
        .sum()
}

/// Compute the Herfindahl-Hirschman Index (HHI) for market concentration.
///
/// HHI = Σ (s_i × 100)² where s_i is the market share (fraction) of firm i.
///
/// # Interpretation
/// - HHI < 1500:   Competitive market
/// - 1500–2500:    Moderately concentrated
/// - HHI > 2500:   Highly concentrated (potential regulatory concern)
/// - HHI = 10000:  Pure monopoly
///
/// # Arguments
/// - `market_shares` — vector of market shares as *fractions* (must sum to ≈ 1)
pub fn compute_hhi(market_shares: &[f64]) -> f64 {
    market_shares.iter().map(|&s| (s * 100.0).powi(2)).sum()
}

/// Compute the Lerner index (market power measure).
///
/// L = (P − MC) / P
///
/// - L = 0: perfectly competitive pricing (P = MC)
/// - L = 1: pure monopoly with zero marginal cost
///
/// Returns 0.0 if price is effectively zero (prevents division by zero).
pub fn lerner_index(price: f64, marginal_cost: f64) -> f64 {
    if price < 1e-12 {
        return 0.0;
    }
    ((price - marginal_cost) / price).clamp(0.0, 1.0)
}

/// Pivotal supplier test: determine if a single unit is pivotal.
///
/// A unit is *pivotal* if the residual supply (total capacity minus this
/// unit's capacity) is insufficient to meet demand.  Pivotal suppliers
/// have market power to withhold capacity and raise prices.
///
/// # Arguments
/// - `total_capacity`   — total installed capacity of all suppliers \[MW\]
/// - `residual_demand`  — demand minus capacity of all competitors \[MW\]
/// - `unit_capacity`    — capacity of the unit under test \[MW\]
///
/// # Returns
/// `true` if the unit is pivotal (competitors cannot serve residual demand alone).
pub fn pivotal_supplier_test(
    total_capacity: f64,
    residual_demand: f64,
    unit_capacity: f64,
) -> bool {
    let competitor_capacity = (total_capacity - unit_capacity).max(0.0);
    competitor_capacity < residual_demand
}

/// Compute the price duration curve from an hourly price series.
///
/// Returns sorted (cumulative_probability, price) pairs, ordered by price
/// descending.  The cumulative probability represents the fraction of hours
/// at or above the corresponding price.
///
/// # Example
/// ```rust
/// use oxigrid::optimize::market::price_duration_curve;
/// let prices = vec![50.0, 30.0, 80.0, 20.0];
/// let pdc = price_duration_curve(&prices);
/// // pdc[0] has the highest price and lowest cumulative probability
/// assert!(pdc[0].1 >= pdc[pdc.len()-1].1);
/// ```
pub fn price_duration_curve(prices: &[f64]) -> Vec<(f64, f64)> {
    if prices.is_empty() {
        return Vec::new();
    }
    let n = prices.len();
    let mut sorted = prices.to_vec();
    sorted.sort_by(|a, b| b.partial_cmp(a).unwrap_or(std::cmp::Ordering::Equal));

    sorted
        .into_iter()
        .enumerate()
        .map(|(i, price)| {
            let cumulative_prob = (i + 1) as f64 / n as f64;
            (cumulative_prob, price)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::optimize::market::{uniform_price_clearing, GeneratorBid};

    fn sample_bids() -> Vec<GeneratorBid> {
        vec![
            GeneratorBid::simple(0, 0, 0.0, 100.0, 20.0),
            GeneratorBid::simple(1, 1, 0.0, 150.0, 35.0),
            GeneratorBid::simple(2, 2, 0.0, 200.0, 50.0),
        ]
    }

    #[test]
    fn test_lerner_index_competitive() {
        assert!((lerner_index(40.0, 40.0) - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_lerner_index_markup() {
        assert!((lerner_index(40.0, 20.0) - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_hhi_monopoly() {
        let hhi = herfindahl_hirschman_index(&[500.0]);
        assert!((hhi - 10000.0).abs() < 1e-6);
    }

    #[test]
    fn test_hhi_equal_firms() {
        let hhi = herfindahl_hirschman_index(&[100.0; 10]);
        assert!((hhi - 1000.0).abs() < 1e-4, "HHI={hhi:.2}");
    }

    #[test]
    fn test_supply_curve_stats() {
        let bids = sample_bids();
        let result = uniform_price_clearing(&bids, 200.0, &[]);
        let stats = supply_curve_stats(&bids, &result);
        assert!((stats.total_capacity_mw - 450.0).abs() < 1e-6);
        assert!(stats.cleared_capacity_mw > 0.0);
        assert!(stats.price_range.0 <= stats.price_range.1);
    }

    #[test]
    fn test_price_duration_curve() {
        let prices = vec![50.0, 30.0, 80.0, 20.0, 60.0];
        let pdc = price_duration_curve(&prices);
        assert_eq!(pdc.len(), 5);
        for i in 1..pdc.len() {
            assert!(pdc[i - 1].1 >= pdc[i].1);
        }
        for (prob, _) in &pdc {
            assert!(*prob > 0.0 && *prob <= 1.0);
        }
    }

    #[test]
    fn test_pivotal_supplier_with_full_capacity() {
        assert!(pivotal_supplier_test(100.0, 80.0, 100.0));
    }

    #[test]
    fn test_pivotal_supplier_not_pivotal() {
        assert!(!pivotal_supplier_test(300.0, 80.0, 100.0));
    }

    #[test]
    fn test_hhi_perfect_competition() {
        let shares = vec![1.0 / 100.0; 100];
        let hhi = compute_hhi(&shares);
        assert!(
            (hhi - 100.0).abs() < 1.0,
            "HHI for 100 equal firms should be 100, got {hhi:.2}"
        );
    }
}
