//! Peer-to-peer (P2P) energy trading market.
//!
//! Enables prosumers (producers + consumers) to trade energy directly with each
//! other using various market mechanisms without requiring a centralised utility.
//!
//! # Mechanisms supported
//! - **Double-sided auction** — classic merit-order matching with midpoint pricing
//! - **Community micromarket** — local-first matching to minimise grid imports
//! - **Bilateral contract** — direct buyer–seller matching by proximity (location bus)
//! - **Blockchain ledger**, **Virtual net billing**, **Flexibility market** — hooks
//!   for future protocol integration (currently fall back to double-sided auction)
//!
//! # References
//! - Tushar, W. et al., "Peer-to-peer energy systems for connected communities",
//!   Applied Energy 282 (2021) 116131
//! - Long, C. et al., "Peer-to-peer energy trading in a community microgrid",
//!   IEEE Transactions on Smart Grid 9(6) 2018
//! - Zhang, C. et al., "A double-sided auction mechanism for P2P energy trading",
//!   IEEE Transactions on Smart Grid 10(6) 2019

use serde::{Deserialize, Serialize};

// ─────────────────────────────────────────────────────────────────────────────
// Enums
// ─────────────────────────────────────────────────────────────────────────────

/// Matching / clearing mechanism used by the P2P market.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum P2pMechanism {
    /// Buyer and seller negotiate terms directly (OTC contracts).
    BilateralContract,
    /// Classic double-sided auction with overlapping bid matching.
    DoubleSidedAuction,
    /// Community-level micromarket that prioritises local trades.
    CommunityMicromarket,
    /// Immutable ledger-based settlement (blockchain / DLT).
    BlockchainLedger,
    /// Aggregated virtual net-billing across community members.
    VirtualNetBilling,
    /// DSO-adjacent flexibility market for demand-side resources.
    FlexibilityMarket,
}

/// Technology / source type of the energy producer in a bid.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProducerType {
    /// Rooftop or utility-scale photovoltaic solar.
    SolarPv,
    /// Onshore or offshore wind turbine.
    WindTurbine,
    /// Battery energy storage system (discharging).
    BatteryDischarge,
    /// Combined heat-and-power plant.
    Chp,
    /// Run-of-river or reservoir hydro.
    HydroPower,
    /// Grid import (utility supply to community).
    GridImport,
}

impl ProducerType {
    /// Returns `true` for renewable / zero-carbon sources.
    pub fn is_renewable(self) -> bool {
        matches!(
            self,
            ProducerType::SolarPv | ProducerType::WindTurbine | ProducerType::HydroPower
        )
    }

    /// Operational CO₂ intensity of energy from this source \[gCO₂/kWh\].
    ///
    /// Uses *point-of-use* accounting consistent with how P2P communities
    /// report emissions:
    /// - Renewables (solar, wind, hydro) and battery discharge are credited as
    ///   zero — their lifecycle / charging emissions are attributed upstream.
    /// - CHP carries its natural-gas direct-combustion factor (≈443 gCO₂/kWh,
    ///   IPCC AR5 median for gas with a heat credit applied).
    /// - Grid imports inherit the caller-supplied grid-average factor, which
    ///   varies by region and time of day.
    pub fn operational_carbon_g_per_kwh(self, grid_ci_g_per_kwh: f64) -> f64 {
        match self {
            ProducerType::SolarPv
            | ProducerType::WindTurbine
            | ProducerType::HydroPower
            | ProducerType::BatteryDischarge => 0.0,
            ProducerType::Chp => 443.0,
            ProducerType::GridImport => grid_ci_g_per_kwh,
        }
    }
}

/// Category of the energy consumer in a bid.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ConsumerType {
    /// Household / residential load.
    Residential,
    /// Commercial building or SME.
    Commercial,
    /// Large industrial consumer.
    Industrial,
    /// Electric vehicle charging load.
    ElectricVehicle,
    /// Heat pump (thermal flexibility).
    HeatPump,
    /// Grid export (community surplus sold to utility).
    GridExport,
}

/// Lifecycle state of a bid or trade.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TradeStatus {
    /// Bid submitted, awaiting matching.
    Pending,
    /// Counterparty found; awaiting settlement.
    Matched,
    /// Trade settled and energy delivered.
    Executed,
    /// Cancelled by participant or market operator.
    Cancelled,
    /// Past `expiry_timestamp` without being matched.
    Expired,
    /// Bid partially filled; residual volume remains.
    PartiallyFilled,
}

/// Direction of a bid (buy or sell).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BidDirection {
    /// Agent wishes to purchase energy.
    Buy,
    /// Agent wishes to sell / export energy.
    Sell,
}

// ─────────────────────────────────────────────────────────────────────────────
// Structs
// ─────────────────────────────────────────────────────────────────────────────

/// A prosumer (producer + consumer) participating in the P2P market.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct P2pAgent {
    /// Unique agent identifier.
    pub id: usize,
    /// Human-readable name.
    pub name: String,
    /// True if the agent can supply energy (has generation assets).
    pub is_producer: bool,
    /// True if the agent consumes energy.
    pub is_consumer: bool,
    /// Grid bus at which this agent is connected.
    pub location_bus: usize,
    /// Maximum generation capacity \[kW\].
    pub generation_capacity_kw: f64,
    /// Current consumption \[kW\].
    pub demand_kw: f64,
    /// Usable battery storage capacity \[kWh\]; 0 if none.
    pub storage_kwh: f64,
    /// Battery state-of-charge \[%\] (0–100); 0 if no storage.
    pub soc_pct: f64,
    /// Preferred fraction of demand served by local (community) energy (0–1).
    pub preferred_local_fraction: f64,
    /// Maximum willingness-to-pay as a buyer \[USD/kWh\].
    pub max_price_usd_per_kwh: f64,
    /// Minimum acceptable revenue as a seller \[USD/kWh\].
    pub min_price_usd_per_kwh: f64,
}

/// A buy or sell bid submitted to the P2P market.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct P2pBid {
    /// Unique bid identifier (assigned by [`P2pMarket::submit_bid`]).
    pub id: usize,
    /// Owner agent identifier.
    pub agent_id: usize,
    /// Whether this is a buy or sell bid.
    pub bid_type: BidDirection,
    /// Energy volume offered or demanded \[kWh\].
    pub quantity_kwh: f64,
    /// Limit price: seller minimum or buyer maximum \[USD/kWh\].
    pub price_usd_per_kwh: f64,
    /// Delivery window start (hour index).
    pub start_hour: usize,
    /// Delivery window end (hour index, inclusive).
    pub end_hour: usize,
    /// Source technology (relevant for sell bids).
    pub producer_type: Option<ProducerType>,
    /// Consumer category (relevant for buy bids).
    pub consumer_type: Option<ConsumerType>,
    /// True if the buyer is willing to pay the green premium for renewables.
    pub green_premium: bool,
    /// True if the agent will only trade within the local community (same bus cluster).
    pub local_only: bool,
    /// Unix-epoch timestamp after which the bid expires \[s\].
    pub expiry_timestamp: f64,
    /// Current lifecycle status.
    pub status: TradeStatus,
}

impl P2pBid {
    /// Returns `true` if this bid has not yet expired or been filled.
    pub fn is_active(&self, now: f64) -> bool {
        self.expiry_timestamp > now
            && matches!(
                self.status,
                TradeStatus::Pending | TradeStatus::PartiallyFilled
            )
    }

    /// Returns `true` when this sell bid offers renewable energy.
    pub fn is_renewable(&self) -> bool {
        self.producer_type
            .map(|p| p.is_renewable())
            .unwrap_or(false)
    }

    /// Operational CO₂ intensity of the energy in this sell bid \[gCO₂/kWh\].
    ///
    /// Dispatches on [`P2pBid::producer_type`]; an unspecified producer falls
    /// back to the grid-average factor (worst-case attribution).
    pub fn carbon_intensity_g_per_kwh(&self, grid_ci_g_per_kwh: f64) -> f64 {
        self.producer_type
            .map(|p| p.operational_carbon_g_per_kwh(grid_ci_g_per_kwh))
            .unwrap_or(grid_ci_g_per_kwh)
    }
}

/// A matched and (optionally) executed P2P energy trade.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct P2pTrade {
    /// Unique trade identifier.
    pub id: usize,
    /// Bid ID of the matched sell bid.
    pub seller_bid_id: usize,
    /// Bid ID of the matched buy bid.
    pub buyer_bid_id: usize,
    /// Agent ID of the seller.
    pub seller_id: usize,
    /// Agent ID of the buyer.
    pub buyer_id: usize,
    /// Traded energy volume \[kWh\].
    pub quantity_kwh: f64,
    /// Agreed clearing price \[USD/kWh\].
    pub clearing_price_usd_per_kwh: f64,
    /// Distribution grid usage fee \[USD/kWh\].
    pub grid_fee_usd_per_kwh: f64,
    /// Net payment received by the seller \[USD\] (= (price − grid_fee) × qty).
    pub net_payment_usd: f64,
    /// Trade creation timestamp \[s\].
    pub timestamp: f64,
    /// Lifecycle status.
    pub status: TradeStatus,
    /// CO₂ intensity of the traded energy \[g/kWh\].
    pub carbon_g_per_kwh: f64,
}

/// Aggregated result of a single market clearing round.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct P2pMarketClearingResult {
    /// All matched trades in this clearing round.
    pub trades: Vec<P2pTrade>,
    /// Total traded volume \[kWh\].
    pub total_volume_kwh: f64,
    /// Volume-weighted average clearing price \[USD/kWh\].
    pub clearing_price_usd_per_kwh: f64,
    /// Aggregate seller surplus above minimum ask prices \[USD\].
    pub seller_surplus_usd: f64,
    /// Aggregate buyer surplus below maximum willingness-to-pay \[USD\].
    pub buyer_surplus_usd: f64,
    /// Total social welfare (seller + buyer surplus) \[USD\].
    pub social_welfare_usd: f64,
    /// Fraction of total traded volume matched within the local community (0–1).
    pub local_trading_fraction: f64,
    /// Total grid fee revenue collected \[USD\].
    pub grid_revenue_usd: f64,
}

impl P2pMarketClearingResult {
    /// Constructs an empty result representing a round with no trades.
    pub fn empty() -> Self {
        Self {
            trades: vec![],
            total_volume_kwh: 0.0,
            clearing_price_usd_per_kwh: 0.0,
            seller_surplus_usd: 0.0,
            buyer_surplus_usd: 0.0,
            social_welfare_usd: 0.0,
            local_trading_fraction: 0.0,
            grid_revenue_usd: 0.0,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// P2pMarket
// ─────────────────────────────────────────────────────────────────────────────

/// Main P2P energy trading market.
///
/// Aggregates agents, bids, and executed trades.  Call [`P2pMarket::clear_market`]
/// to run the configured mechanism and obtain a [`P2pMarketClearingResult`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct P2pMarket {
    /// Market mechanism used for clearing.
    pub mechanism: P2pMechanism,
    /// Registered prosumer agents.
    pub agents: Vec<P2pAgent>,
    /// All submitted bids (historical + active).
    pub bids: Vec<P2pBid>,
    /// All executed and pending trades.
    pub trades: Vec<P2pTrade>,
    /// Default distribution grid usage fee \[USD/kWh\].
    pub grid_fee_usd_per_kwh: f64,
    /// Additional premium charged/paid for verified green energy \[USD/kWh\].
    pub green_premium_usd_per_kwh: f64,
    /// Grid-average CO₂ intensity applied to imported / unspecified energy
    /// \[gCO₂/kWh\].  Defaults to 200; set per region or per dispatch interval.
    pub grid_carbon_intensity_g_per_kwh: f64,
    /// Platform fee percentage of trade value (e.g. 1.0 = 1 %).
    pub platform_fee_pct: f64,
    /// Counter for assigning unique bid IDs.
    pub next_bid_id: usize,
    /// Counter for assigning unique trade IDs.
    pub next_trade_id: usize,
}

impl P2pMarket {
    /// Create a new P2P market with the given mechanism and sensible defaults.
    ///
    /// - Grid fee: 0.05 USD/kWh
    /// - Green premium: 0.02 USD/kWh
    /// - Grid carbon intensity: 200 gCO₂/kWh
    /// - Platform fee: 1.0 %
    pub fn new(mechanism: P2pMechanism) -> Self {
        Self {
            mechanism,
            agents: vec![],
            bids: vec![],
            trades: vec![],
            grid_fee_usd_per_kwh: 0.05,
            green_premium_usd_per_kwh: 0.02,
            grid_carbon_intensity_g_per_kwh: 200.0,
            platform_fee_pct: 1.0,
            next_bid_id: 0,
            next_trade_id: 0,
        }
    }

    /// Register a prosumer agent with the market.
    pub fn add_agent(&mut self, agent: P2pAgent) {
        self.agents.push(agent);
    }

    /// Submit a bid and return its assigned bid ID.
    ///
    /// The bid ID is set inside the submitted bid record so callers do not need
    /// to carry separate state.
    pub fn submit_bid(&mut self, mut bid: P2pBid) -> usize {
        let id = self.next_bid_id;
        bid.id = id;
        bid.status = TradeStatus::Pending;
        self.next_bid_id += 1;
        self.bids.push(bid);
        id
    }

    /// Return all bids that are currently active (not expired, not fully filled).
    ///
    /// Uses timestamp 0.0 as the reference "now" for expiry checking; callers
    /// with a real clock should filter the returned slice themselves.
    pub fn get_active_bids(&self) -> Vec<&P2pBid> {
        // Use a sentinel "now" of 0; a bid is considered active if
        // expiry > 0 (i.e. expiry has been set to some future time)
        // OR its status is Pending/PartiallyFilled and it has a non-zero expiry.
        self.bids
            .iter()
            .filter(|b| {
                matches!(
                    b.status,
                    TradeStatus::Pending | TradeStatus::PartiallyFilled
                ) && b.expiry_timestamp > 0.0
            })
            .collect()
    }

    // ─── Clearing mechanisms ──────────────────────────────────────────────────

    /// Run a double-sided auction clearing.
    ///
    /// Algorithm:
    /// 1. Collect active sell bids sorted by price ascending (cheapest first).
    /// 2. Collect active buy bids sorted by price descending (highest WTP first).
    /// 3. While the cheapest sell price ≤ the highest buy price: match.
    ///    Clearing price = midpoint `(sell_price + buy_price) / 2`.
    /// 4. Compute per-trade surplus and aggregate statistics.
    pub fn run_double_sided_auction(&mut self) -> P2pMarketClearingResult {
        // Collect indices of active bids
        let mut sell_indices: Vec<usize> = self
            .bids
            .iter()
            .enumerate()
            .filter(|(_, b)| {
                b.bid_type == BidDirection::Sell
                    && matches!(
                        b.status,
                        TradeStatus::Pending | TradeStatus::PartiallyFilled
                    )
                    && b.expiry_timestamp > 0.0
            })
            .map(|(i, _)| i)
            .collect();

        let mut buy_indices: Vec<usize> = self
            .bids
            .iter()
            .enumerate()
            .filter(|(_, b)| {
                b.bid_type == BidDirection::Buy
                    && matches!(
                        b.status,
                        TradeStatus::Pending | TradeStatus::PartiallyFilled
                    )
                    && b.expiry_timestamp > 0.0
            })
            .map(|(i, _)| i)
            .collect();

        // Sort sell bids ascending by price, buy bids descending
        sell_indices.sort_by(|&a, &b| {
            self.bids[a]
                .price_usd_per_kwh
                .partial_cmp(&self.bids[b].price_usd_per_kwh)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        buy_indices.sort_by(|&a, &b| {
            self.bids[b]
                .price_usd_per_kwh
                .partial_cmp(&self.bids[a].price_usd_per_kwh)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        // Working copies of remaining quantities
        let mut sell_qty: Vec<f64> = sell_indices
            .iter()
            .map(|&i| self.bids[i].quantity_kwh)
            .collect();
        let mut buy_qty: Vec<f64> = buy_indices
            .iter()
            .map(|&i| self.bids[i].quantity_kwh)
            .collect();

        let mut new_trades: Vec<P2pTrade> = vec![];
        let mut si = 0;
        let mut bi = 0;

        while si < sell_indices.len() && bi < buy_indices.len() {
            let sell_price = self.bids[sell_indices[si]].price_usd_per_kwh;
            let buy_price = self.bids[buy_indices[bi]].price_usd_per_kwh;

            if sell_price > buy_price {
                break; // No more overlapping bids
            }

            let clearing_price = (sell_price + buy_price) / 2.0;

            // Green premium: add to clearing price if buyer wants green and seller is renewable
            let seller_renewable = self.bids[sell_indices[si]].is_renewable();
            let buyer_wants_green = self.bids[buy_indices[bi]].green_premium;
            let effective_price = if seller_renewable && buyer_wants_green {
                clearing_price + self.green_premium_usd_per_kwh
            } else {
                clearing_price
            };

            let matched_qty = sell_qty[si].min(buy_qty[bi]);

            let seller_id = self.bids[sell_indices[si]].agent_id;
            let buyer_id = self.bids[buy_indices[bi]].agent_id;
            let seller_bid_id = self.bids[sell_indices[si]].id;
            let buyer_bid_id = self.bids[buy_indices[bi]].id;

            let grid_fee = self.grid_fee_usd_per_kwh;
            let net_payment = (effective_price - grid_fee) * matched_qty;

            // Source-aware CO₂ intensity: renewables/storage = 0, CHP = gas
            // factor, grid import / unknown = configured grid average.
            let carbon = self.bids[sell_indices[si]]
                .carbon_intensity_g_per_kwh(self.grid_carbon_intensity_g_per_kwh);

            let trade_id = self.next_trade_id;
            self.next_trade_id += 1;

            new_trades.push(P2pTrade {
                id: trade_id,
                seller_bid_id,
                buyer_bid_id,
                seller_id,
                buyer_id,
                quantity_kwh: matched_qty,
                clearing_price_usd_per_kwh: effective_price,
                grid_fee_usd_per_kwh: grid_fee,
                net_payment_usd: net_payment,
                timestamp: 0.0,
                status: TradeStatus::Executed,
                carbon_g_per_kwh: carbon,
            });

            sell_qty[si] -= matched_qty;
            buy_qty[bi] -= matched_qty;

            // Advance pointers for fully consumed bids
            if sell_qty[si] < 1e-9 {
                si += 1;
            }
            if buy_qty[bi] < 1e-9 {
                bi += 1;
            }
        }

        // Update bid statuses
        self.update_bid_statuses_after_matching(&sell_indices, &sell_qty);
        self.update_bid_statuses_after_matching(&buy_indices, &buy_qty);

        let result = self.build_result(new_trades.clone());
        self.trades.extend(new_trades);
        result
    }

    /// Run community micromarket clearing.
    ///
    /// Priority order:
    /// 1. Match buyers and sellers on the **same bus** (local trades).
    /// 2. Match remaining volume across buses (inter-community).
    ///
    /// Within each group, sell bids are sorted ascending by price and buy bids
    /// descending, same as in the double-sided auction.
    pub fn run_community_micromarket(&mut self) -> P2pMarketClearingResult {
        // Collect active bids
        let mut active_sells: Vec<usize> = self
            .bids
            .iter()
            .enumerate()
            .filter(|(_, b)| {
                b.bid_type == BidDirection::Sell
                    && matches!(
                        b.status,
                        TradeStatus::Pending | TradeStatus::PartiallyFilled
                    )
                    && b.expiry_timestamp > 0.0
            })
            .map(|(i, _)| i)
            .collect();

        let mut active_buys: Vec<usize> = self
            .bids
            .iter()
            .enumerate()
            .filter(|(_, b)| {
                b.bid_type == BidDirection::Buy
                    && matches!(
                        b.status,
                        TradeStatus::Pending | TradeStatus::PartiallyFilled
                    )
                    && b.expiry_timestamp > 0.0
            })
            .map(|(i, _)| i)
            .collect();

        // Sort by price
        active_sells.sort_by(|&a, &b| {
            self.bids[a]
                .price_usd_per_kwh
                .partial_cmp(&self.bids[b].price_usd_per_kwh)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        active_buys.sort_by(|&a, &b| {
            self.bids[b]
                .price_usd_per_kwh
                .partial_cmp(&self.bids[a].price_usd_per_kwh)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let mut sell_qty: Vec<f64> = active_sells
            .iter()
            .map(|&i| self.bids[i].quantity_kwh)
            .collect();
        let mut buy_qty: Vec<f64> = active_buys
            .iter()
            .map(|&i| self.bids[i].quantity_kwh)
            .collect();

        let mut new_trades: Vec<P2pTrade> = vec![];

        // Phase 1: local (same-bus) matches
        self.match_bids_pass(
            &active_sells,
            &active_buys,
            &mut sell_qty,
            &mut buy_qty,
            true, // local_only pass
            &mut new_trades,
        );

        // Phase 2: cross-community matches (skip local_only buy bids if already satisfied)
        self.match_bids_pass(
            &active_sells,
            &active_buys,
            &mut sell_qty,
            &mut buy_qty,
            false, // allow inter-community
            &mut new_trades,
        );

        self.update_bid_statuses_after_matching(&active_sells, &sell_qty);
        self.update_bid_statuses_after_matching(&active_buys, &buy_qty);

        let result = self.build_result(new_trades.clone());
        self.trades.extend(new_trades);
        result
    }

    /// Run bilateral contract matching by proximity (location bus).
    ///
    /// Each buyer is paired with the seller on the nearest bus (smallest
    /// `|buyer_bus − seller_bus|` difference).  Ties are broken by price spread.
    pub fn run_bilateral_matching(&mut self) -> P2pMarketClearingResult {
        let active_sells: Vec<usize> = self
            .bids
            .iter()
            .enumerate()
            .filter(|(_, b)| {
                b.bid_type == BidDirection::Sell
                    && matches!(
                        b.status,
                        TradeStatus::Pending | TradeStatus::PartiallyFilled
                    )
                    && b.expiry_timestamp > 0.0
            })
            .map(|(i, _)| i)
            .collect();

        let active_buys: Vec<usize> = self
            .bids
            .iter()
            .enumerate()
            .filter(|(_, b)| {
                b.bid_type == BidDirection::Buy
                    && matches!(
                        b.status,
                        TradeStatus::Pending | TradeStatus::PartiallyFilled
                    )
                    && b.expiry_timestamp > 0.0
            })
            .map(|(i, _)| i)
            .collect();

        let mut sell_qty: Vec<f64> = active_sells
            .iter()
            .map(|&i| self.bids[i].quantity_kwh)
            .collect();
        let mut buy_qty: Vec<f64> = active_buys
            .iter()
            .map(|&i| self.bids[i].quantity_kwh)
            .collect();

        let mut new_trades: Vec<P2pTrade> = vec![];

        // For each buyer, find the closest seller by bus distance
        for (bi, &buy_idx) in active_buys.iter().enumerate() {
            if buy_qty[bi] < 1e-9 {
                continue;
            }
            let buyer_bus = self.agent_bus(self.bids[buy_idx].agent_id);
            let buyer_max_price = self.bids[buy_idx].price_usd_per_kwh;

            // Sort potential sellers by bus distance then price
            let mut candidates: Vec<(usize, usize)> = active_sells
                .iter()
                .enumerate()
                .filter(|(si, &sell_idx)| {
                    sell_qty[*si] > 1e-9 && self.bids[sell_idx].price_usd_per_kwh <= buyer_max_price
                })
                .map(|(si, &sell_idx)| {
                    let seller_bus = self.agent_bus(self.bids[sell_idx].agent_id);
                    let dist = (buyer_bus as isize - seller_bus as isize).unsigned_abs();
                    (dist, si)
                })
                .collect();

            candidates.sort_by_key(|&(dist, _)| dist);

            for (_, si) in candidates {
                if buy_qty[bi] < 1e-9 {
                    break;
                }
                if sell_qty[si] < 1e-9 {
                    continue;
                }

                let sell_price = self.bids[active_sells[si]].price_usd_per_kwh;
                let clearing_price = (sell_price + buyer_max_price) / 2.0;
                let matched_qty = sell_qty[si].min(buy_qty[bi]);

                let seller_renewable = self.bids[active_sells[si]].is_renewable();
                let effective_price = if seller_renewable && self.bids[buy_idx].green_premium {
                    clearing_price + self.green_premium_usd_per_kwh
                } else {
                    clearing_price
                };

                let grid_fee = self.grid_fee_usd_per_kwh;
                let trade_id = self.next_trade_id;
                self.next_trade_id += 1;

                new_trades.push(P2pTrade {
                    id: trade_id,
                    seller_bid_id: self.bids[active_sells[si]].id,
                    buyer_bid_id: self.bids[buy_idx].id,
                    seller_id: self.bids[active_sells[si]].agent_id,
                    buyer_id: self.bids[buy_idx].agent_id,
                    quantity_kwh: matched_qty,
                    clearing_price_usd_per_kwh: effective_price,
                    grid_fee_usd_per_kwh: grid_fee,
                    net_payment_usd: (effective_price - grid_fee) * matched_qty,
                    timestamp: 0.0,
                    status: TradeStatus::Executed,
                    carbon_g_per_kwh: self.bids[active_sells[si]]
                        .carbon_intensity_g_per_kwh(self.grid_carbon_intensity_g_per_kwh),
                });

                sell_qty[si] -= matched_qty;
                buy_qty[bi] -= matched_qty;
            }
        }

        self.update_bid_statuses_after_matching(&active_sells, &sell_qty);
        self.update_bid_statuses_after_matching(&active_buys, &buy_qty);

        let result = self.build_result(new_trades.clone());
        self.trades.extend(new_trades);
        result
    }

    /// Clear the market using the mechanism configured at construction time.
    pub fn clear_market(&mut self) -> P2pMarketClearingResult {
        match self.mechanism {
            P2pMechanism::DoubleSidedAuction => self.run_double_sided_auction(),
            P2pMechanism::CommunityMicromarket => self.run_community_micromarket(),
            P2pMechanism::BilateralContract => self.run_bilateral_matching(),
            // Blockchain / VNB / Flexibility: fall back to double-sided auction as base
            P2pMechanism::BlockchainLedger
            | P2pMechanism::VirtualNetBilling
            | P2pMechanism::FlexibilityMarket => self.run_double_sided_auction(),
        }
    }

    // ─── Agent-level analytics ────────────────────────────────────────────────

    /// Compute `(net_energy_kwh, net_payment_usd)` for a given agent.
    ///
    /// - Energy: positive = net seller (exported), negative = net buyer (imported).
    /// - Payment: positive = money received, negative = money paid out.
    pub fn compute_agent_balance(&self, agent_id: usize) -> (f64, f64) {
        let mut net_energy = 0.0_f64;
        let mut net_payment = 0.0_f64;

        for trade in &self.trades {
            if trade.status != TradeStatus::Executed {
                continue;
            }
            if trade.seller_id == agent_id {
                net_energy -= trade.quantity_kwh;
                net_payment += trade.net_payment_usd;
            }
            if trade.buyer_id == agent_id {
                net_energy += trade.quantity_kwh;
                net_payment -= trade.clearing_price_usd_per_kwh * trade.quantity_kwh;
            }
        }

        (net_energy, net_payment)
    }

    /// Fraction of total community demand that is served by local generation (0–1).
    ///
    /// `self_sufficiency = min(local_generation, total_demand) / total_demand`
    pub fn compute_community_self_sufficiency(&self) -> f64 {
        let total_demand: f64 = self.agents.iter().map(|a| a.demand_kw).sum();
        if total_demand < 1e-12 {
            return 1.0;
        }
        let total_generation: f64 = self.agents.iter().map(|a| a.generation_capacity_kw).sum();
        (total_generation / total_demand).min(1.0)
    }

    /// Fraction of traded energy sourced from renewable producers (0–1).
    pub fn compute_renewable_fraction(&self) -> f64 {
        let total_vol: f64 = self
            .trades
            .iter()
            .filter(|t| t.status == TradeStatus::Executed)
            .map(|t| t.quantity_kwh)
            .sum();
        if total_vol < 1e-12 {
            return 0.0;
        }
        let renewable_vol: f64 = self
            .trades
            .iter()
            .filter(|t| t.status == TradeStatus::Executed && t.carbon_g_per_kwh < 1e-9)
            .map(|t| t.quantity_kwh)
            .sum();
        renewable_vol / total_vol
    }

    // ─── Private helpers ──────────────────────────────────────────────────────

    /// Look up the grid bus for an agent by ID; returns 0 if not found.
    fn agent_bus(&self, agent_id: usize) -> usize {
        self.agents
            .iter()
            .find(|a| a.id == agent_id)
            .map(|a| a.location_bus)
            .unwrap_or(0)
    }

    /// One matching pass used by community micromarket.
    ///
    /// When `local_only_pass` is `true`, each buyer is matched only with sellers
    /// on the **same bus**; the sell list is scanned per-buyer to find the
    /// cheapest compatible seller.  When `false`, remaining volume is matched
    /// across buses (excluding buyers with `local_only = true`).
    #[allow(clippy::ptr_arg)]
    fn match_bids_pass(
        &mut self,
        sell_indices: &[usize],
        buy_indices: &[usize],
        sell_qty: &mut Vec<f64>,
        buy_qty: &mut Vec<f64>,
        local_only_pass: bool,
        new_trades: &mut Vec<P2pTrade>,
    ) {
        // For each buyer, scan all sellers looking for a compatible match.
        // This O(B·S) approach correctly handles the case where the cheapest
        // global seller is on a different bus from the buyer (local-pass must
        // skip it and find the next cheapest *local* seller).
        for bi in 0..buy_indices.len() {
            if buy_qty[bi] < 1e-9 {
                continue;
            }
            let buy_idx = buy_indices[bi];

            // Respect local_only flag on buy bid in cross-community pass
            if !local_only_pass && self.bids[buy_idx].local_only {
                continue;
            }

            let buy_bus = self.agent_bus(self.bids[buy_idx].agent_id);
            let buy_price = self.bids[buy_idx].price_usd_per_kwh;

            // Collect eligible seller positions sorted by price ascending
            let mut eligible: Vec<usize> = sell_indices
                .iter()
                .enumerate()
                .filter(|(si, &sell_idx)| {
                    if sell_qty[*si] < 1e-9 {
                        return false;
                    }
                    let sell_bus = self.agent_bus(self.bids[sell_idx].agent_id);
                    if local_only_pass && sell_bus != buy_bus {
                        return false;
                    }
                    self.bids[sell_idx].price_usd_per_kwh <= buy_price
                })
                .map(|(si, _)| si)
                .collect();

            // Sort by price ascending (cheapest first)
            eligible.sort_by(|&a, &b| {
                self.bids[sell_indices[a]]
                    .price_usd_per_kwh
                    .partial_cmp(&self.bids[sell_indices[b]].price_usd_per_kwh)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });

            for si in eligible {
                if buy_qty[bi] < 1e-9 {
                    break;
                }
                if sell_qty[si] < 1e-9 {
                    continue;
                }

                let sell_idx = sell_indices[si];
                let sell_price = self.bids[sell_idx].price_usd_per_kwh;
                let clearing_price = (sell_price + buy_price) / 2.0;
                let seller_renewable = self.bids[sell_idx].is_renewable();
                let effective_price = if seller_renewable && self.bids[buy_idx].green_premium {
                    clearing_price + self.green_premium_usd_per_kwh
                } else {
                    clearing_price
                };

                let matched_qty = sell_qty[si].min(buy_qty[bi]);
                let grid_fee = self.grid_fee_usd_per_kwh;
                let trade_id = self.next_trade_id;
                self.next_trade_id += 1;

                new_trades.push(P2pTrade {
                    id: trade_id,
                    seller_bid_id: self.bids[sell_idx].id,
                    buyer_bid_id: self.bids[buy_idx].id,
                    seller_id: self.bids[sell_idx].agent_id,
                    buyer_id: self.bids[buy_idx].agent_id,
                    quantity_kwh: matched_qty,
                    clearing_price_usd_per_kwh: effective_price,
                    grid_fee_usd_per_kwh: grid_fee,
                    net_payment_usd: (effective_price - grid_fee) * matched_qty,
                    timestamp: 0.0,
                    status: TradeStatus::Executed,
                    carbon_g_per_kwh: self.bids[sell_idx]
                        .carbon_intensity_g_per_kwh(self.grid_carbon_intensity_g_per_kwh),
                });

                sell_qty[si] -= matched_qty;
                buy_qty[bi] -= matched_qty;
            }
        }
    }

    /// Stamp matched / partially-filled / pending statuses onto bids after a run.
    fn update_bid_statuses_after_matching(&mut self, indices: &[usize], remaining: &[f64]) {
        for (pos, &bid_idx) in indices.iter().enumerate() {
            let original_qty = self.bids[bid_idx].quantity_kwh;
            let rem = remaining[pos];
            if rem < 1e-9 {
                self.bids[bid_idx].status = TradeStatus::Matched;
            } else if rem < original_qty - 1e-9 {
                self.bids[bid_idx].status = TradeStatus::PartiallyFilled;
            }
            // else: still Pending (unchanged)
        }
    }

    /// Build a [`P2pMarketClearingResult`] from a slice of newly created trades.
    fn build_result(&self, trades: Vec<P2pTrade>) -> P2pMarketClearingResult {
        if trades.is_empty() {
            return P2pMarketClearingResult::empty();
        }

        let total_volume: f64 = trades.iter().map(|t| t.quantity_kwh).sum();

        // Volume-weighted average clearing price
        let clearing_price = if total_volume > 1e-12 {
            trades
                .iter()
                .map(|t| t.clearing_price_usd_per_kwh * t.quantity_kwh)
                .sum::<f64>()
                / total_volume
        } else {
            0.0
        };

        // Seller surplus: (clearing_price − min_ask) × qty
        let seller_surplus: f64 = trades
            .iter()
            .map(|t| {
                let min_ask = self
                    .bids
                    .iter()
                    .find(|b| b.id == t.seller_bid_id)
                    .map(|b| b.price_usd_per_kwh)
                    .unwrap_or(0.0);
                (t.clearing_price_usd_per_kwh - min_ask).max(0.0) * t.quantity_kwh
            })
            .sum();

        // Buyer surplus: (max_wtp − clearing_price) × qty
        let buyer_surplus: f64 = trades
            .iter()
            .map(|t| {
                let max_wtp = self
                    .bids
                    .iter()
                    .find(|b| b.id == t.buyer_bid_id)
                    .map(|b| b.price_usd_per_kwh)
                    .unwrap_or(0.0);
                (max_wtp - t.clearing_price_usd_per_kwh).max(0.0) * t.quantity_kwh
            })
            .sum();

        let grid_revenue: f64 = trades
            .iter()
            .map(|t| t.grid_fee_usd_per_kwh * t.quantity_kwh)
            .sum();

        // Local trading fraction: trades where seller_bus == buyer_bus
        let local_vol: f64 = trades
            .iter()
            .filter(|t| self.agent_bus(t.seller_id) == self.agent_bus(t.buyer_id))
            .map(|t| t.quantity_kwh)
            .sum();
        let local_fraction = if total_volume > 1e-12 {
            local_vol / total_volume
        } else {
            0.0
        };

        P2pMarketClearingResult {
            trades,
            total_volume_kwh: total_volume,
            clearing_price_usd_per_kwh: clearing_price,
            seller_surplus_usd: seller_surplus,
            buyer_surplus_usd: buyer_surplus,
            social_welfare_usd: seller_surplus + buyer_surplus,
            local_trading_fraction: local_fraction,
            grid_revenue_usd: grid_revenue,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// P2pAnalytics
// ─────────────────────────────────────────────────────────────────────────────

/// Post-market analysis utilities for P2P energy trading results.
pub struct P2pAnalytics;

impl P2pAnalytics {
    /// Price discovery efficiency: ratio of achieved social welfare to theoretical
    /// maximum (all surplus captured at perfectly competitive prices).
    ///
    /// Returns a value in \[0, 1\].
    pub fn compute_price_discovery_efficiency(result: &P2pMarketClearingResult) -> f64 {
        let theoretical_max = result.social_welfare_usd + result.grid_revenue_usd;
        if theoretical_max < 1e-12 {
            return 1.0; // Vacuously efficient if no trades
        }
        (result.social_welfare_usd / theoretical_max).clamp(0.0, 1.0)
    }

    /// Gini coefficient for a distribution of payments.
    ///
    /// Returns 0 for perfect equality, 1 for maximum inequality.
    /// Empty or all-zero slices return 0.
    pub fn compute_gini_coefficient(payments: &[f64]) -> f64 {
        if payments.is_empty() {
            return 0.0;
        }
        let mut sorted = payments.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        let n = sorted.len() as f64;
        let sum: f64 = sorted.iter().sum();
        if sum < 1e-12 {
            return 0.0;
        }

        // Gini = (2 * Σ_i (i+1)*x_i) / (n * Σ x_i) - (n+1)/n
        let weighted_sum: f64 = sorted
            .iter()
            .enumerate()
            .map(|(i, &x)| (i + 1) as f64 * x)
            .sum();

        let gini = 2.0 * weighted_sum / (n * sum) - (n + 1.0) / n;
        gini.clamp(0.0, 1.0)
    }

    /// Prosumer benefit: total savings (or earnings) versus the reference grid price.
    ///
    /// - For sellers: earnings above the grid export tariff (0.05 USD/kWh by default).
    /// - For buyers: savings below the grid retail price (0.20 USD/kWh by default).
    ///
    /// Uses `grid_buy = 0.20`, `grid_sell = 0.05` as reference rates.
    pub fn compute_prosumer_benefit(agent: &P2pAgent, trades: &[P2pTrade]) -> f64 {
        let grid_buy_price = 0.20_f64;
        let grid_sell_price = 0.05_f64;
        let mut benefit = 0.0_f64;

        for trade in trades {
            if trade.status != TradeStatus::Executed {
                continue;
            }
            if trade.seller_id == agent.id {
                // Seller earns P2P clearing minus what they'd earn on the grid
                benefit += (trade.net_payment_usd) - grid_sell_price * trade.quantity_kwh;
            }
            if trade.buyer_id == agent.id {
                // Buyer saves by paying less than the retail grid price
                benefit += (grid_buy_price - trade.clearing_price_usd_per_kwh) * trade.quantity_kwh;
            }
        }
        benefit
    }

    /// Identify the best-spread (seller, buyer) bid pairs.
    ///
    /// Returns a list of `(sell_bid_id, buy_bid_id)` tuples sorted by price
    /// spread descending (largest potential surplus first).
    /// Only overlapping pairs (sell_price ≤ buy_price) are included.
    pub fn identify_best_matches(bids: &[P2pBid]) -> Vec<(usize, usize)> {
        let sell_bids: Vec<&P2pBid> = bids
            .iter()
            .filter(|b| b.bid_type == BidDirection::Sell)
            .collect();
        let buy_bids: Vec<&P2pBid> = bids
            .iter()
            .filter(|b| b.bid_type == BidDirection::Buy)
            .collect();

        let mut pairs: Vec<(f64, usize, usize)> = vec![];
        for sell in &sell_bids {
            for buy in &buy_bids {
                let spread = buy.price_usd_per_kwh - sell.price_usd_per_kwh;
                if spread >= 0.0 {
                    pairs.push((spread, sell.id, buy.id));
                }
            }
        }

        // Sort descending by spread
        pairs.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));
        pairs.into_iter().map(|(_, s, b)| (s, b)).collect()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Helpers ───────────────────────────────────────────────────────────────

    fn make_market() -> P2pMarket {
        P2pMarket::new(P2pMechanism::DoubleSidedAuction)
    }

    fn make_agent(id: usize, bus: usize, gen_kw: f64, demand_kw: f64) -> P2pAgent {
        P2pAgent {
            id,
            name: format!("agent_{id}"),
            is_producer: gen_kw > 0.0,
            is_consumer: demand_kw > 0.0,
            location_bus: bus,
            generation_capacity_kw: gen_kw,
            demand_kw,
            storage_kwh: 0.0,
            soc_pct: 0.0,
            preferred_local_fraction: 0.8,
            max_price_usd_per_kwh: 0.25,
            min_price_usd_per_kwh: 0.05,
        }
    }

    fn sell_bid(agent_id: usize, qty: f64, price: f64) -> P2pBid {
        P2pBid {
            id: 0,
            agent_id,
            bid_type: BidDirection::Sell,
            quantity_kwh: qty,
            price_usd_per_kwh: price,
            start_hour: 0,
            end_hour: 1,
            producer_type: Some(ProducerType::SolarPv),
            consumer_type: None,
            green_premium: false,
            local_only: false,
            expiry_timestamp: 9999.0,
            status: TradeStatus::Pending,
        }
    }

    fn buy_bid(agent_id: usize, qty: f64, price: f64) -> P2pBid {
        P2pBid {
            id: 0,
            agent_id,
            bid_type: BidDirection::Buy,
            quantity_kwh: qty,
            price_usd_per_kwh: price,
            start_hour: 0,
            end_hour: 1,
            producer_type: None,
            consumer_type: Some(ConsumerType::Residential),
            green_premium: false,
            local_only: false,
            expiry_timestamp: 9999.0,
            status: TradeStatus::Pending,
        }
    }

    // ── Tests ─────────────────────────────────────────────────────────────────

    #[test]
    fn test_submit_bid() {
        let mut market = make_market();
        let id0 = market.submit_bid(sell_bid(0, 10.0, 0.10));
        let id1 = market.submit_bid(buy_bid(1, 10.0, 0.20));
        assert_eq!(id0, 0);
        assert_eq!(id1, 1);
        assert_eq!(market.bids.len(), 2);
    }

    #[test]
    fn test_double_sided_auction_match() {
        let mut market = make_market();
        market.add_agent(make_agent(0, 1, 20.0, 0.0));
        market.add_agent(make_agent(1, 2, 0.0, 10.0));
        market.submit_bid(sell_bid(0, 10.0, 0.10));
        market.submit_bid(buy_bid(1, 10.0, 0.20));
        let result = market.run_double_sided_auction();
        assert_eq!(result.trades.len(), 1);
        assert!((result.total_volume_kwh - 10.0).abs() < 1e-9);
    }

    #[test]
    fn test_double_sided_auction_no_match() {
        let mut market = make_market();
        market.add_agent(make_agent(0, 1, 20.0, 0.0));
        market.add_agent(make_agent(1, 2, 0.0, 10.0));
        // Sell price > buy price → no overlap
        market.submit_bid(sell_bid(0, 10.0, 0.25));
        market.submit_bid(buy_bid(1, 10.0, 0.10));
        let result = market.run_double_sided_auction();
        assert!(result.trades.is_empty());
        assert!((result.total_volume_kwh).abs() < 1e-9);
    }

    #[test]
    fn test_clearing_price_midpoint() {
        let mut market = make_market();
        market.add_agent(make_agent(0, 1, 20.0, 0.0));
        market.add_agent(make_agent(1, 2, 0.0, 10.0));
        market.submit_bid(sell_bid(0, 10.0, 0.10));
        market.submit_bid(buy_bid(1, 10.0, 0.20));
        let result = market.run_double_sided_auction();
        assert!(!result.trades.is_empty());
        let trade = &result.trades[0];
        // Solar is renewable → no green premium here (green_premium=false)
        let expected = (0.10 + 0.20) / 2.0;
        assert!((trade.clearing_price_usd_per_kwh - expected).abs() < 1e-9);
    }

    #[test]
    fn test_surplus_computation() {
        let mut market = make_market();
        market.add_agent(make_agent(0, 1, 20.0, 0.0));
        market.add_agent(make_agent(1, 2, 0.0, 10.0));
        market.submit_bid(sell_bid(0, 10.0, 0.10)); // min price 0.10
        market.submit_bid(buy_bid(1, 10.0, 0.20)); // max price 0.20
        let result = market.run_double_sided_auction();
        // clearing = 0.15; seller surplus = (0.15-0.10)*10 = 0.50
        assert!((result.seller_surplus_usd - 0.50).abs() < 1e-6);
        // buyer surplus = (0.20-0.15)*10 = 0.50
        assert!((result.buyer_surplus_usd - 0.50).abs() < 1e-6);
    }

    #[test]
    fn test_social_welfare_positive() {
        let mut market = make_market();
        market.add_agent(make_agent(0, 1, 20.0, 0.0));
        market.add_agent(make_agent(1, 2, 0.0, 10.0));
        market.submit_bid(sell_bid(0, 10.0, 0.10));
        market.submit_bid(buy_bid(1, 10.0, 0.20));
        let result = market.run_double_sided_auction();
        assert!(result.social_welfare_usd > 0.0);
    }

    #[test]
    fn test_community_market_local_priority() {
        let mut market = P2pMarket::new(P2pMechanism::CommunityMicromarket);
        // Two agents on same bus (bus 1)
        market.add_agent(make_agent(0, 1, 20.0, 0.0));
        market.add_agent(make_agent(1, 1, 0.0, 10.0));
        // One remote agent on bus 2
        market.add_agent(make_agent(2, 2, 20.0, 0.0));

        // Seller on bus 1 (local)
        market.submit_bid(sell_bid(0, 10.0, 0.10));
        // Seller on bus 2 (remote), cheaper
        market.submit_bid(sell_bid(2, 10.0, 0.08));
        // Buyer on bus 1, local_only = false so both sellers eligible
        let mut b = buy_bid(1, 10.0, 0.25);
        b.local_only = false;
        market.submit_bid(b);

        let result = market.run_community_micromarket();
        assert!(!result.trades.is_empty());
        // Local trading fraction should be > 0 for at least one trade on same bus
        // (community market prioritises bus 1 seller first)
        // Agent 0 (bus 1) should have been matched with agent 1 (bus 1)
        let local_trade = result
            .trades
            .iter()
            .any(|t| t.seller_id == 0 && t.buyer_id == 1);
        assert!(local_trade, "Local seller should have been matched first");
    }

    #[test]
    fn test_bilateral_proximity_match() {
        let mut market = P2pMarket::new(P2pMechanism::BilateralContract);
        market.add_agent(make_agent(0, 5, 20.0, 0.0)); // seller on bus 5
        market.add_agent(make_agent(1, 3, 20.0, 0.0)); // seller on bus 3
        market.add_agent(make_agent(2, 4, 0.0, 10.0)); // buyer on bus 4 (closer to bus 3 & 5 equally, but bus 5 is dist 1, bus 3 is dist 1)

        market.submit_bid(sell_bid(0, 10.0, 0.12)); // seller bus 5
        market.submit_bid(sell_bid(1, 10.0, 0.12)); // seller bus 3
        market.submit_bid(buy_bid(2, 10.0, 0.20)); // buyer bus 4

        let result = market.run_bilateral_matching();
        assert_eq!(result.trades.len(), 1);
        // Buyer on bus 4 should match the nearest seller (bus 3 or 5, both distance 1)
        let trade = &result.trades[0];
        assert_eq!(trade.buyer_id, 2);
    }

    #[test]
    fn test_local_trading_fraction() {
        let mut market = make_market();
        market.add_agent(make_agent(0, 1, 20.0, 0.0));
        market.add_agent(make_agent(1, 1, 0.0, 5.0)); // same bus → local
        market.add_agent(make_agent(2, 2, 0.0, 5.0)); // different bus → remote

        market.submit_bid(sell_bid(0, 10.0, 0.10));
        market.submit_bid(buy_bid(1, 5.0, 0.20));
        market.submit_bid(buy_bid(2, 5.0, 0.20));

        let result = market.run_double_sided_auction();
        // 5 kWh local, 5 kWh remote → fraction = 0.5
        assert!(!result.trades.is_empty());
        assert!((result.local_trading_fraction - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_agent_balance_seller() {
        let mut market = make_market();
        market.add_agent(make_agent(0, 1, 20.0, 0.0));
        market.add_agent(make_agent(1, 2, 0.0, 10.0));
        market.submit_bid(sell_bid(0, 10.0, 0.10));
        market.submit_bid(buy_bid(1, 10.0, 0.20));
        market.run_double_sided_auction();

        let (_, payment) = market.compute_agent_balance(0);
        assert!(payment > 0.0, "Seller should receive positive payment");
    }

    #[test]
    fn test_agent_balance_buyer() {
        let mut market = make_market();
        market.add_agent(make_agent(0, 1, 20.0, 0.0));
        market.add_agent(make_agent(1, 2, 0.0, 10.0));
        market.submit_bid(sell_bid(0, 10.0, 0.10));
        market.submit_bid(buy_bid(1, 10.0, 0.20));
        market.run_double_sided_auction();

        let (_, payment) = market.compute_agent_balance(1);
        assert!(
            payment < 0.0,
            "Buyer should have negative net payment (money out)"
        );
    }

    #[test]
    fn test_grid_fee_deducted() {
        let mut market = make_market();
        market.grid_fee_usd_per_kwh = 0.05;
        market.add_agent(make_agent(0, 1, 20.0, 0.0));
        market.add_agent(make_agent(1, 2, 0.0, 10.0));
        market.submit_bid(sell_bid(0, 10.0, 0.10));
        market.submit_bid(buy_bid(1, 10.0, 0.20));
        let result = market.run_double_sided_auction();
        let trade = &result.trades[0];
        let expected_net = (trade.clearing_price_usd_per_kwh - 0.05) * trade.quantity_kwh;
        assert!((trade.net_payment_usd - expected_net).abs() < 1e-9);
    }

    #[test]
    fn test_renewable_fraction() {
        let mut market = make_market();
        market.add_agent(make_agent(0, 1, 20.0, 0.0));
        market.add_agent(make_agent(1, 2, 0.0, 10.0));

        // Solar PV sell bid → renewable
        let mut s = sell_bid(0, 10.0, 0.10);
        s.producer_type = Some(ProducerType::SolarPv);
        market.submit_bid(s);
        market.submit_bid(buy_bid(1, 10.0, 0.20));
        market.run_double_sided_auction();

        let frac = market.compute_renewable_fraction();
        assert!((frac - 1.0).abs() < 1e-9, "All traded energy is from solar");
    }

    #[test]
    fn test_self_sufficiency() {
        let mut market = make_market();
        market.add_agent(make_agent(0, 1, 50.0, 30.0)); // surplus producer
        market.add_agent(make_agent(1, 2, 0.0, 20.0)); // pure consumer

        let ss = market.compute_community_self_sufficiency();
        // total_gen=50, total_demand=50 → ss=1.0
        assert!((ss - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_gini_equal_payments() {
        let payments = vec![10.0, 10.0, 10.0, 10.0];
        let gini = P2pAnalytics::compute_gini_coefficient(&payments);
        assert!(gini < 0.01, "Gini should be ~0 for equal payments: {gini}");
    }

    #[test]
    fn test_gini_one_dominant() {
        let payments = vec![1.0, 1.0, 1.0, 1000.0];
        let gini = P2pAnalytics::compute_gini_coefficient(&payments);
        assert!(
            gini > 0.7,
            "Gini should be high when one agent dominates: {gini}"
        );
    }

    #[test]
    fn test_active_bids_filter() {
        let mut market = make_market();
        // Active bid (expiry in future)
        market.submit_bid(sell_bid(0, 10.0, 0.10));
        // Expired bid
        let mut expired = sell_bid(1, 10.0, 0.10);
        expired.expiry_timestamp = 0.0; // expiry = 0 → inactive
        market.submit_bid(expired);

        let active = market.get_active_bids();
        assert_eq!(active.len(), 1);
        assert_eq!(active[0].agent_id, 0);
    }

    #[test]
    fn test_partial_fill() {
        let mut market = make_market();
        market.add_agent(make_agent(0, 1, 20.0, 0.0));
        market.add_agent(make_agent(1, 2, 0.0, 20.0));

        market.submit_bid(sell_bid(0, 5.0, 0.10)); // only 5 kWh available
        market.submit_bid(buy_bid(1, 20.0, 0.20)); // wants 20 kWh

        let result = market.run_double_sided_auction();
        assert_eq!(result.trades.len(), 1);
        assert!((result.trades[0].quantity_kwh - 5.0).abs() < 1e-9);

        // Buy bid should be partially filled
        let buy_bid_status = market
            .bids
            .iter()
            .find(|b| b.bid_type == BidDirection::Buy)
            .map(|b| b.status);
        assert_eq!(buy_bid_status, Some(TradeStatus::PartiallyFilled));
    }

    #[test]
    fn test_green_premium_price() {
        let mut market = make_market();
        market.green_premium_usd_per_kwh = 0.02;
        market.add_agent(make_agent(0, 1, 20.0, 0.0));
        market.add_agent(make_agent(1, 2, 0.0, 10.0));

        // Renewable sell bid + buyer requests green premium
        let mut s = sell_bid(0, 10.0, 0.10);
        s.producer_type = Some(ProducerType::SolarPv);
        market.submit_bid(s);

        let mut b = buy_bid(1, 10.0, 0.25);
        b.green_premium = true;
        market.submit_bid(b);

        let result = market.run_double_sided_auction();
        assert!(!result.trades.is_empty());
        let base_clear = (0.10 + 0.25) / 2.0;
        let expected_price = base_clear + 0.02;
        let actual_price = result.trades[0].clearing_price_usd_per_kwh;
        assert!(
            (actual_price - expected_price).abs() < 1e-9,
            "Expected {expected_price}, got {actual_price}"
        );
    }

    #[test]
    fn test_empty_bids() {
        let mut market = make_market();
        let result = market.clear_market();
        assert!(result.trades.is_empty());
        assert!((result.total_volume_kwh).abs() < 1e-9);
        assert!((result.social_welfare_usd).abs() < 1e-9);
    }

    #[test]
    fn test_identify_best_matches() {
        // Assign explicit IDs so the returned (sell_id, buy_id) pairs are meaningful
        let mut b0 = sell_bid(0, 5.0, 0.10);
        b0.id = 10;
        let mut b1 = sell_bid(1, 5.0, 0.15);
        b1.id = 11;
        let mut b2 = buy_bid(2, 5.0, 0.25);
        b2.id = 20;
        let mut b3 = buy_bid(3, 5.0, 0.18);
        b3.id = 21;
        let bids = vec![b0, b1, b2, b3];
        let matches = P2pAnalytics::identify_best_matches(&bids);
        assert!(!matches.is_empty());
        // Best spread: sell@0.10 vs buy@0.25 → spread 0.15
        let (sell_id, buy_id) = matches[0];
        assert_eq!(sell_id, 10); // sell bid id 10
        assert_eq!(buy_id, 20); // buy bid id 20
    }

    #[test]
    fn test_prosumer_benefit_seller() {
        let agent = make_agent(0, 1, 20.0, 0.0);
        let trades = vec![P2pTrade {
            id: 0,
            seller_bid_id: 0,
            buyer_bid_id: 1,
            seller_id: 0,
            buyer_id: 1,
            quantity_kwh: 10.0,
            clearing_price_usd_per_kwh: 0.15,
            grid_fee_usd_per_kwh: 0.05,
            net_payment_usd: 1.00, // (0.15 - 0.05) * 10
            timestamp: 0.0,
            status: TradeStatus::Executed,
            carbon_g_per_kwh: 0.0,
        }];
        let benefit = P2pAnalytics::compute_prosumer_benefit(&agent, &trades);
        // net_payment=1.00, grid_sell=0.05*10=0.50 → benefit=0.50
        assert!((benefit - 0.50).abs() < 1e-9);
    }

    #[test]
    fn test_price_discovery_efficiency() {
        let result = P2pMarketClearingResult {
            trades: vec![],
            total_volume_kwh: 10.0,
            clearing_price_usd_per_kwh: 0.15,
            seller_surplus_usd: 0.5,
            buyer_surplus_usd: 0.5,
            social_welfare_usd: 1.0,
            local_trading_fraction: 1.0,
            grid_revenue_usd: 0.5,
        };
        let eff = P2pAnalytics::compute_price_discovery_efficiency(&result);
        // social_welfare / (social_welfare + grid_revenue) = 1.0 / 1.5 ≈ 0.667
        assert!((eff - 1.0 / 1.5).abs() < 1e-9);
    }

    #[test]
    fn test_clear_market_dispatches_mechanism() {
        // Blockchain ledger should fall back to DSA
        let mut market = P2pMarket::new(P2pMechanism::BlockchainLedger);
        market.add_agent(make_agent(0, 1, 20.0, 0.0));
        market.add_agent(make_agent(1, 2, 0.0, 10.0));
        market.submit_bid(sell_bid(0, 10.0, 0.10));
        market.submit_bid(buy_bid(1, 10.0, 0.20));
        let result = market.clear_market();
        assert_eq!(result.trades.len(), 1);
    }

    #[test]
    fn test_buyer_price_too_low_no_match() {
        // Buyer's max price (0.08) is below seller's ask (0.15) — no trade should occur.
        let mut market = make_market();
        market.add_agent(make_agent(0, 1, 20.0, 0.0));
        market.add_agent(make_agent(1, 2, 0.0, 10.0));
        market.submit_bid(sell_bid(0, 10.0, 0.15));
        market.submit_bid(buy_bid(1, 10.0, 0.08));
        let result = market.run_double_sided_auction();
        assert!(
            result.trades.is_empty(),
            "expected zero trades when buy price < sell price"
        );
        assert!((result.total_volume_kwh).abs() < 1e-9);
    }

    #[test]
    fn test_clearing_price_within_spread() {
        // Sell at 0.10, buy at 0.20 → clearing_price should equal (0.10+0.20)/2 = 0.15.
        let mut market = make_market();
        market.add_agent(make_agent(0, 1, 20.0, 0.0));
        market.add_agent(make_agent(1, 2, 0.0, 10.0));
        market.submit_bid(sell_bid(0, 10.0, 0.10));
        market.submit_bid(buy_bid(1, 10.0, 0.20));
        let result = market.run_double_sided_auction();
        assert!(!result.trades.is_empty(), "expected at least one trade");
        let cp = result.trades[0].clearing_price_usd_per_kwh;
        let expected = (0.10_f64 + 0.20_f64) / 2.0;
        assert!(
            (0.10_f64..=0.20_f64).contains(&cp),
            "clearing price {cp} not within spread [0.10, 0.20]"
        );
        assert!(
            (cp - expected).abs() < 1e-9,
            "clearing price {cp} != expected {expected}"
        );
    }

    #[test]
    fn test_volume_conservation() {
        // Two sell bids (5 kWh each) and two buy bids (5 kWh each): total tradeable = 10 kWh.
        let mut market = make_market();
        market.add_agent(make_agent(0, 1, 20.0, 0.0));
        market.add_agent(make_agent(1, 2, 20.0, 0.0));
        market.add_agent(make_agent(2, 3, 0.0, 10.0));
        market.add_agent(make_agent(3, 4, 0.0, 10.0));
        market.submit_bid(sell_bid(0, 5.0, 0.10));
        market.submit_bid(sell_bid(1, 5.0, 0.10));
        market.submit_bid(buy_bid(2, 5.0, 0.20));
        market.submit_bid(buy_bid(3, 5.0, 0.20));
        let result = market.run_double_sided_auction();
        let sum_trade_qty: f64 = result.trades.iter().map(|t| t.quantity_kwh).sum();
        assert!(
            (sum_trade_qty - result.total_volume_kwh).abs() < 1e-9,
            "sum of trade quantities {sum_trade_qty} != total_volume_kwh {}",
            result.total_volume_kwh
        );
        assert!((result.total_volume_kwh - 10.0).abs() < 1e-9);
    }

    #[test]
    fn test_prosumer_acts_as_seller_and_buyer() {
        // Prosumer (agent 0) with gen_kw=15, demand_kw=5 sells 10 kWh to agent 1.
        // After clearing: agent 0 net_energy < 0 (exported), net_payment > 0 (received money).
        let mut market = make_market();
        market.add_agent(make_agent(0, 1, 15.0, 5.0));
        market.add_agent(make_agent(1, 2, 0.0, 10.0));
        market.submit_bid(sell_bid(0, 10.0, 0.10));
        market.submit_bid(buy_bid(1, 10.0, 0.20));
        let result = market.run_double_sided_auction();
        assert_eq!(result.trades.len(), 1, "expected one trade");
        let (net_energy, net_payment) = market.compute_agent_balance(0);
        assert!(
            net_energy < 0.0,
            "prosumer exported energy, net_energy should be negative"
        );
        assert!(
            net_payment > 0.0,
            "prosumer received payment, net_payment should be positive"
        );
    }

    #[test]
    fn test_bilateral_mechanism_proximity() {
        // Buyer on bus 2 should prefer the closer seller (bus 1) over the distant seller (bus 5).
        let mut market = P2pMarket::new(P2pMechanism::BilateralContract);
        market.add_agent(make_agent(0, 1, 20.0, 0.0)); // seller close to buyer
        market.add_agent(make_agent(1, 5, 20.0, 0.0)); // seller far from buyer
        market.add_agent(make_agent(2, 2, 0.0, 10.0)); // buyer
        market.submit_bid(sell_bid(0, 10.0, 0.10));
        market.submit_bid(sell_bid(1, 10.0, 0.10));
        market.submit_bid(buy_bid(2, 10.0, 0.20));
        let result = market.run_bilateral_matching();
        assert!(!result.trades.is_empty(), "expected at least one trade");
        assert_eq!(
            result.trades[0].seller_id, 0,
            "closest seller (agent 0, bus 1) should be matched, got seller_id={}",
            result.trades[0].seller_id
        );
    }

    #[test]
    fn test_community_micromarket_local_preference() {
        // Agent 0 (bus 1, seller) and agent 1 (bus 1, buyer) are co-located.
        // Agent 2 (bus 2, seller) is on a different bus.
        // The local pair should trade, giving local_trading_fraction == 1.0.
        let mut market = P2pMarket::new(P2pMechanism::CommunityMicromarket);
        market.add_agent(make_agent(0, 1, 20.0, 0.0)); // local seller
        market.add_agent(make_agent(1, 1, 0.0, 10.0)); // local buyer (same bus)
        market.add_agent(make_agent(2, 2, 20.0, 0.0)); // remote seller
        market.submit_bid(sell_bid(0, 10.0, 0.10));
        market.submit_bid(buy_bid(1, 10.0, 0.20));
        market.submit_bid(sell_bid(2, 10.0, 0.10));
        let result = market.run_community_micromarket();
        assert!(!result.trades.is_empty(), "expected at least one trade");
        assert!(
            (result.local_trading_fraction - 1.0).abs() < 1e-9,
            "expected local_trading_fraction=1.0, got {}",
            result.local_trading_fraction
        );
    }

    #[test]
    fn test_renewable_carbon_intensity_zero() {
        // A sell bid from a WindTurbine is renewable; its trade's carbon_g_per_kwh must be 0.0.
        // compute_renewable_fraction should also return 1.0 after clearing.
        let mut market = make_market();
        market.add_agent(make_agent(0, 1, 20.0, 0.0));
        market.add_agent(make_agent(1, 2, 0.0, 10.0));
        let wind_sell = P2pBid {
            id: 0,
            agent_id: 0,
            bid_type: BidDirection::Sell,
            quantity_kwh: 10.0,
            price_usd_per_kwh: 0.10,
            start_hour: 0,
            end_hour: 1,
            producer_type: Some(ProducerType::WindTurbine),
            consumer_type: None,
            green_premium: false,
            local_only: false,
            expiry_timestamp: 9999.0,
            status: TradeStatus::Pending,
        };
        market.submit_bid(wind_sell);
        market.submit_bid(buy_bid(1, 10.0, 0.20));
        let result = market.run_double_sided_auction();
        assert!(!result.trades.is_empty(), "expected a trade");
        assert!(
            (result.trades[0].carbon_g_per_kwh).abs() < 1e-9,
            "wind trade should have zero carbon intensity, got {}",
            result.trades[0].carbon_g_per_kwh
        );
        let renewable_frac = market.compute_renewable_fraction();
        assert!(
            (renewable_frac - 1.0).abs() < 1e-9,
            "expected renewable_fraction=1.0, got {renewable_frac}"
        );
    }

    #[test]
    fn producer_type_operational_carbon_is_source_aware() {
        let grid = 350.0;
        // Renewables and storage are zero at point of use.
        for pt in [
            ProducerType::SolarPv,
            ProducerType::WindTurbine,
            ProducerType::HydroPower,
            ProducerType::BatteryDischarge,
        ] {
            assert_eq!(pt.operational_carbon_g_per_kwh(grid), 0.0, "{pt:?}");
        }
        // CHP carries its gas combustion factor, independent of grid CI.
        assert_eq!(ProducerType::Chp.operational_carbon_g_per_kwh(grid), 443.0);
        assert_eq!(ProducerType::Chp.operational_carbon_g_per_kwh(0.0), 443.0);
        // Grid import inherits the configured grid average.
        assert_eq!(
            ProducerType::GridImport.operational_carbon_g_per_kwh(grid),
            grid
        );
    }

    #[test]
    fn bid_carbon_falls_back_to_grid_for_unknown_source() {
        let mut bid = sell_bid(0, 10.0, 0.10);
        bid.producer_type = None;
        assert_eq!(bid.carbon_intensity_g_per_kwh(275.0), 275.0);
        bid.producer_type = Some(ProducerType::GridImport);
        assert_eq!(bid.carbon_intensity_g_per_kwh(275.0), 275.0);
        bid.producer_type = Some(ProducerType::SolarPv);
        assert_eq!(bid.carbon_intensity_g_per_kwh(275.0), 0.0);
    }

    #[test]
    fn chp_trade_carries_nonzero_carbon_intensity() {
        let mut market = make_market();
        market.add_agent(make_agent(0, 1, 20.0, 0.0));
        market.add_agent(make_agent(1, 2, 0.0, 10.0));
        let mut s = sell_bid(0, 10.0, 0.10);
        s.producer_type = Some(ProducerType::Chp);
        market.submit_bid(s);
        market.submit_bid(buy_bid(1, 10.0, 0.20));
        let result = market.run_double_sided_auction();
        assert!(!result.trades.is_empty(), "expected a CHP trade");
        assert!(
            (result.trades[0].carbon_g_per_kwh - 443.0).abs() < 1e-9,
            "CHP trade carbon should be 443 g/kWh, got {}",
            result.trades[0].carbon_g_per_kwh
        );
    }

    #[test]
    fn grid_import_trade_uses_configured_intensity() {
        let mut market = make_market();
        market.grid_carbon_intensity_g_per_kwh = 410.0;
        market.add_agent(make_agent(0, 1, 20.0, 0.0));
        market.add_agent(make_agent(1, 2, 0.0, 10.0));
        let mut s = sell_bid(0, 10.0, 0.10);
        s.producer_type = Some(ProducerType::GridImport);
        market.submit_bid(s);
        market.submit_bid(buy_bid(1, 10.0, 0.20));
        let result = market.run_double_sided_auction();
        assert!(!result.trades.is_empty(), "expected a grid-import trade");
        assert!(
            (result.trades[0].carbon_g_per_kwh - 410.0).abs() < 1e-9,
            "grid-import trade carbon should follow configured 410 g/kWh, got {}",
            result.trades[0].carbon_g_per_kwh
        );
    }

    #[test]
    fn test_single_participant_no_trade() {
        // Only one agent (a seller) with no buyers: clearing must yield zero trades.
        let mut market = make_market();
        market.add_agent(make_agent(0, 1, 20.0, 0.0));
        market.submit_bid(sell_bid(0, 10.0, 0.10));
        let result = market.clear_market();
        assert!(
            result.trades.is_empty(),
            "no buyers means no trades expected"
        );
        assert!(
            (result.total_volume_kwh).abs() < 1e-9,
            "total_volume_kwh should be 0.0 with no trades, got {}",
            result.total_volume_kwh
        );
    }
}
