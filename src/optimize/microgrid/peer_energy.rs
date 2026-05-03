/// Peer-to-peer (P2P) energy trading for microgrid prosumers.
///
/// Implements a double-auction market clearing mechanism where prosumers
/// (producers/consumers) post offers and bids for local energy trading.
/// Settled at a clearing price between the best bid and offer.
///
/// # Reference
/// Morstyn et al., "Using peer-to-peer energy-trading platforms to incentivize
/// prosumers to form federated power plants," Nature Energy, 2018.
use serde::{Deserialize, Serialize};

/// An energy offer (from a seller/producer).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnergyOffer {
    /// Prosumer/node identifier
    pub node_id: usize,
    /// Energy quantity offered `kWh`
    pub quantity_kwh: f64,
    /// Minimum acceptable price [$/kWh]
    pub min_price: f64,
}

/// An energy bid (from a buyer/consumer).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnergyBid {
    /// Prosumer/node identifier
    pub node_id: usize,
    /// Energy quantity desired `kWh`
    pub quantity_kwh: f64,
    /// Maximum acceptable price [$/kWh]
    pub max_price: f64,
}

/// A matched trade between one seller and one buyer.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Trade {
    pub seller_id: usize,
    pub buyer_id: usize,
    /// Energy traded `kWh`
    pub quantity_kwh: f64,
    /// Settlement price [$/kWh]
    pub price: f64,
}

impl Trade {
    /// Total value of this trade [$].
    pub fn value(&self) -> f64 {
        self.quantity_kwh * self.price
    }
}

/// Market clearing result.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MarketResult {
    /// Matched trades
    pub trades: Vec<Trade>,
    /// Market clearing price [$/kWh] (volume-weighted average)
    pub clearing_price: f64,
    /// Total energy traded `kWh`
    pub total_traded_kwh: f64,
    /// Unmatched supply `kWh`
    pub unmatched_supply_kwh: f64,
    /// Unmatched demand `kWh`
    pub unmatched_demand_kwh: f64,
}

impl MarketResult {
    /// Total market value [$].
    pub fn total_value(&self) -> f64 {
        self.trades.iter().map(|t| t.value()).sum()
    }

    /// Aggregate seller revenue [$].
    pub fn seller_revenue(&self, seller_id: usize) -> f64 {
        self.trades
            .iter()
            .filter(|t| t.seller_id == seller_id)
            .map(|t| t.value())
            .sum()
    }

    /// Aggregate buyer cost [$].
    pub fn buyer_cost(&self, buyer_id: usize) -> f64 {
        self.trades
            .iter()
            .filter(|t| t.buyer_id == buyer_id)
            .map(|t| t.value())
            .sum()
    }
}

/// Clear the P2P energy market using a uniform-price double auction.
///
/// Algorithm:
/// 1. Sort offers ascending by price (cheapest first).
/// 2. Sort bids descending by price (highest willingness-to-pay first).
/// 3. Match offers to bids sequentially until no profitable match remains.
/// 4. Clearing price = mid-point of last matched offer and bid prices.
pub fn clear_market(offers: &[EnergyOffer], bids: &[EnergyBid]) -> MarketResult {
    // Sort offers by price ascending
    let mut sorted_offers: Vec<(usize, EnergyOffer)> = offers.iter().cloned().enumerate().collect();
    sorted_offers.sort_by(|a, b| a.1.min_price.partial_cmp(&b.1.min_price).unwrap());

    // Sort bids by price descending
    let mut sorted_bids: Vec<(usize, EnergyBid)> = bids.iter().cloned().enumerate().collect();
    sorted_bids.sort_by(|a, b| b.1.max_price.partial_cmp(&a.1.max_price).unwrap());

    let mut trades = Vec::new();
    let mut offer_remaining: Vec<f64> = sorted_offers.iter().map(|o| o.1.quantity_kwh).collect();
    let mut bid_remaining: Vec<f64> = sorted_bids.iter().map(|b| b.1.quantity_kwh).collect();

    let mut oi = 0; // offer pointer
    let mut bi = 0; // bid pointer
    let mut last_price = 0.0_f64;

    while oi < sorted_offers.len() && bi < sorted_bids.len() {
        let offer = &sorted_offers[oi].1;
        let bid = &sorted_bids[bi].1;

        // No match if cheapest offer > highest bid
        if offer.min_price > bid.max_price {
            break;
        }

        // Clearing price: midpoint
        let price = (offer.min_price + bid.max_price) / 2.0;
        last_price = price;

        let qty = offer_remaining[oi].min(bid_remaining[bi]);

        trades.push(Trade {
            seller_id: offer.node_id,
            buyer_id: bid.node_id,
            quantity_kwh: qty,
            price,
        });

        offer_remaining[oi] -= qty;
        bid_remaining[bi] -= qty;

        if offer_remaining[oi] < 1e-9 {
            oi += 1;
        }
        if bid_remaining[bi] < 1e-9 {
            bi += 1;
        }
    }

    let total_traded: f64 = trades.iter().map(|t| t.quantity_kwh).sum();
    let clearing_price = if total_traded > 1e-9 {
        trades.iter().map(|t| t.price * t.quantity_kwh).sum::<f64>() / total_traded
    } else {
        last_price
    };

    let total_supply: f64 = offers.iter().map(|o| o.quantity_kwh).sum();
    let total_demand: f64 = bids.iter().map(|b| b.quantity_kwh).sum();

    MarketResult {
        trades,
        clearing_price,
        total_traded_kwh: total_traded,
        unmatched_supply_kwh: (total_supply - total_traded).max(0.0),
        unmatched_demand_kwh: (total_demand - total_traded).max(0.0),
    }
}

/// Prosumer model for iterative P2P trading.
///
/// Each prosumer has a baseline import/export based on their PV generation
/// and load, then posts offers/bids to the P2P market.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Prosumer {
    pub id: usize,
    /// Current generation `kW`
    pub generation_kw: f64,
    /// Current load `kW`
    pub load_kw: f64,
    /// Minimum sell price (marginal cost of PV ≈ 0, but opportunity cost > 0) [$/kWh]
    pub sell_price: f64,
    /// Maximum buy price (willingness-to-pay before using grid) [$/kWh]
    pub buy_price: f64,
}

impl Prosumer {
    /// Net power: positive = surplus (seller), negative = deficit (buyer).
    pub fn net_kw(&self) -> f64 {
        self.generation_kw - self.load_kw
    }

    /// Create an offer if prosumer has surplus.
    pub fn offer(&self, dt_h: f64) -> Option<EnergyOffer> {
        let net = self.net_kw();
        if net > 0.01 {
            Some(EnergyOffer {
                node_id: self.id,
                quantity_kwh: net * dt_h,
                min_price: self.sell_price,
            })
        } else {
            None
        }
    }

    /// Create a bid if prosumer has deficit.
    pub fn bid(&self, dt_h: f64) -> Option<EnergyBid> {
        let net = self.net_kw();
        if net < -0.01 {
            Some(EnergyBid {
                node_id: self.id,
                quantity_kwh: net.abs() * dt_h,
                max_price: self.buy_price,
            })
        } else {
            None
        }
    }
}

/// Run one P2P trading interval for a set of prosumers.
pub fn run_p2p_interval(prosumers: &[Prosumer], dt_h: f64) -> MarketResult {
    let offers: Vec<EnergyOffer> = prosumers.iter().filter_map(|p| p.offer(dt_h)).collect();
    let bids: Vec<EnergyBid> = prosumers.iter().filter_map(|p| p.bid(dt_h)).collect();
    clear_market(&offers, &bids)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simple_trade_clears() {
        let offers = vec![EnergyOffer {
            node_id: 0,
            quantity_kwh: 5.0,
            min_price: 0.05,
        }];
        let bids = vec![EnergyBid {
            node_id: 1,
            quantity_kwh: 5.0,
            max_price: 0.15,
        }];
        let result = clear_market(&offers, &bids);
        assert_eq!(result.trades.len(), 1);
        assert!((result.total_traded_kwh - 5.0).abs() < 1e-9);
        assert!(result.clearing_price > 0.05 && result.clearing_price < 0.15);
    }

    #[test]
    fn test_no_trade_when_offer_above_bid() {
        let offers = vec![EnergyOffer {
            node_id: 0,
            quantity_kwh: 5.0,
            min_price: 0.20,
        }];
        let bids = vec![EnergyBid {
            node_id: 1,
            quantity_kwh: 5.0,
            max_price: 0.10,
        }];
        let result = clear_market(&offers, &bids);
        assert_eq!(result.trades.len(), 0);
        assert_eq!(result.total_traded_kwh, 0.0);
    }

    #[test]
    fn test_partial_match() {
        let offers = vec![EnergyOffer {
            node_id: 0,
            quantity_kwh: 10.0,
            min_price: 0.05,
        }];
        let bids = vec![EnergyBid {
            node_id: 1,
            quantity_kwh: 4.0,
            max_price: 0.15,
        }];
        let result = clear_market(&offers, &bids);
        assert!((result.total_traded_kwh - 4.0).abs() < 1e-9);
        assert!((result.unmatched_supply_kwh - 6.0).abs() < 1e-9);
    }

    #[test]
    fn test_multi_prosumer_interval() {
        let prosumers = vec![
            Prosumer {
                id: 0,
                generation_kw: 20.0,
                load_kw: 5.0,
                sell_price: 0.05,
                buy_price: 0.20,
            },
            Prosumer {
                id: 1,
                generation_kw: 0.0,
                load_kw: 10.0,
                sell_price: 0.05,
                buy_price: 0.18,
            },
            Prosumer {
                id: 2,
                generation_kw: 0.0,
                load_kw: 8.0,
                sell_price: 0.05,
                buy_price: 0.15,
            },
        ];
        let result = run_p2p_interval(&prosumers, 1.0);
        assert!(result.total_traded_kwh > 0.0, "Should trade energy");
    }

    #[test]
    fn test_seller_revenue_positive() {
        let offers = vec![EnergyOffer {
            node_id: 0,
            quantity_kwh: 5.0,
            min_price: 0.08,
        }];
        let bids = vec![EnergyBid {
            node_id: 1,
            quantity_kwh: 5.0,
            max_price: 0.12,
        }];
        let result = clear_market(&offers, &bids);
        assert!(result.seller_revenue(0) > 0.0);
        assert!(result.buyer_cost(1) > 0.0);
        assert!((result.seller_revenue(0) - result.buyer_cost(1)).abs() < 1e-9);
    }

    #[test]
    fn test_clearing_price_between_offer_and_bid() {
        let offers = vec![EnergyOffer {
            node_id: 0,
            quantity_kwh: 3.0,
            min_price: 0.06,
        }];
        let bids = vec![EnergyBid {
            node_id: 1,
            quantity_kwh: 3.0,
            max_price: 0.14,
        }];
        let result = clear_market(&offers, &bids);
        assert!(
            result.clearing_price >= 0.06 && result.clearing_price <= 0.14,
            "Clearing price={:.4} not in [0.06, 0.14]",
            result.clearing_price
        );
    }

    #[test]
    fn test_prosumer_offer_and_bid() {
        let seller = Prosumer {
            id: 0,
            generation_kw: 10.0,
            load_kw: 3.0,
            sell_price: 0.05,
            buy_price: 0.20,
        };
        let buyer = Prosumer {
            id: 1,
            generation_kw: 0.0,
            load_kw: 5.0,
            sell_price: 0.05,
            buy_price: 0.20,
        };
        assert!(seller.offer(1.0).is_some());
        assert!(seller.bid(1.0).is_none());
        assert!(buyer.bid(1.0).is_some());
        assert!(buyer.offer(1.0).is_none());
    }
}
