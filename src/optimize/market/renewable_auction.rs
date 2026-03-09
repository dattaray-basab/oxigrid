//! Renewable Energy Auction Platform.
//!
//! Implements:
//! - **Contract-for-Difference (CfD)**: two-way payment between generator and
//!   settlement body based on strike vs. reference price.
//! - **Feed-in Tariff (FiT)**: fixed tariff per MWh for a defined duration.
//! - **Competitive sealed-bid auctions**: merit-order selection of lowest bids.
//! - **Pay-as-bid vs. uniform price clearing**: discriminatory vs. single-price
//!   settlement.
//! - **Descending-clock auction**: iterative price reduction until excess supply
//!   clears.
//! - **Auction outcome analysis**: efficiency metrics, technology mix, HHI.
//!
//! # References
//! - IRENA, "Renewable Energy Auctions: A Guide to Design", 2015
//! - EC, "Guidelines on State Aid for Climate, Environmental Protection and
//!   Energy 2022"
//! - Couture & Gagnon, "An analysis of feed-in tariff remuneration models",
//!   Energy Policy, 2010

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

// ─────────────────────────────────────────────────────────────────────────────
// Enums
// ─────────────────────────────────────────────────────────────────────────────

/// Auction price-setting mechanism.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum AuctionMechanism {
    /// Each winner paid their own submitted bid price (discriminatory).
    PayAsBid,
    /// All winners paid the clearing price (last accepted bid).
    UniformPrice,
    /// Synonym for [`AuctionMechanism::UniformPrice`].
    PayAsClear,
    /// Iterative descending-clock: price lowered until excess supply clears.
    DescendingClock,
}

/// Renewable energy support scheme applied to auction winners.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SupportScheme {
    /// Contract for Difference — two-way payment vs. market reference \[$/MWh\].
    Cfd {
        /// Guaranteed strike price agreed at auction \[$/MWh\]
        strike_price_per_mwh: f64,
        /// Market reference (index) price at contract start \[$/MWh\]
        reference_price_per_mwh: f64,
        /// Contract duration \[years\]
        duration_years: usize,
    },
    /// Fixed feed-in tariff, independent of market price \[$/MWh\].
    FeedInTariff {
        /// Fixed tariff paid per MWh generated \[$/MWh\]
        tariff_per_mwh: f64,
        /// Tariff period \[years\]
        duration_years: usize,
    },
    /// Renewable obligation certificate top-up on market revenue.
    RenewableObligation {
        /// ROC/REC value per MWh \[$/MWh\]
        certificate_value_per_mwh: f64,
    },
    /// Premium FiT: market revenue plus a fixed premium top-up \[$/MWh\].
    PremiumFeedIn {
        /// Premium added on top of market price \[$/MWh\]
        premium_per_mwh: f64,
    },
}

/// Renewable generation technology category.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq, Hash)]
pub enum RenewableTech {
    SolarPv,
    OnshoreWind,
    OffshoreWind,
    Hydro,
    Geothermal,
    Biomass,
    Tidal,
}

impl RenewableTech {
    /// Human-readable technology name.
    pub fn name(&self) -> &'static str {
        match self {
            RenewableTech::SolarPv => "Solar PV",
            RenewableTech::OnshoreWind => "Onshore Wind",
            RenewableTech::OffshoreWind => "Offshore Wind",
            RenewableTech::Hydro => "Hydro",
            RenewableTech::Geothermal => "Geothermal",
            RenewableTech::Biomass => "Biomass",
            RenewableTech::Tidal => "Tidal",
        }
    }

    /// CAPEX benchmark \[$/MW\].
    pub fn capex_per_mw(&self) -> f64 {
        match self {
            RenewableTech::SolarPv => 800_000.0,
            RenewableTech::OnshoreWind => 1_300_000.0,
            RenewableTech::OffshoreWind => 3_000_000.0,
            RenewableTech::Hydro => 2_000_000.0,
            RenewableTech::Geothermal => 4_000_000.0,
            RenewableTech::Biomass => 2_500_000.0,
            RenewableTech::Tidal => 5_000_000.0,
        }
    }

    /// Annual OPEX benchmark \[$/MW/year\].
    pub fn opex_per_mw_yr(&self) -> f64 {
        match self {
            RenewableTech::SolarPv => 15_000.0,
            RenewableTech::OnshoreWind => 40_000.0,
            RenewableTech::OffshoreWind => 100_000.0,
            RenewableTech::Hydro => 20_000.0,
            RenewableTech::Geothermal => 60_000.0,
            RenewableTech::Biomass => 80_000.0,
            RenewableTech::Tidal => 150_000.0,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Bid / configuration structures
// ─────────────────────────────────────────────────────────────────────────────

/// A renewable energy project bid submitted to an auction.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RenewableBid {
    /// Unique bidder / project identifier
    pub bidder_id: String,
    /// Technology type
    pub technology: RenewableTech,
    /// Nameplate capacity offered \[MW\]
    pub capacity_mw: f64,
    /// Strike price requested by the bidder \[$/MWh\]
    pub strike_price_per_mwh: f64,
    /// Expected capacity factor \[%\], e.g. 35.0 for 35 %
    pub capacity_factor_pct: f64,
    /// Planned commissioning year
    pub commissioning_year: usize,
    /// Project lifetime \[years\]
    pub lifetime_years: usize,
    /// Grid connection point / region
    pub location: String,
    /// Estimated grid connection capital cost \[M$\]
    pub grid_connection_cost_m_usd: f64,
}

/// Per-technology procurement quota with a separate price cap.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TechBand {
    /// Technology this band applies to
    pub technology: RenewableTech,
    /// Maximum MW procured from this technology in this band \[MW\]
    pub quota_mw: f64,
    /// Technology-specific price cap \[$/MWh\]
    pub max_price_per_mwh: f64,
}

/// Eligibility requirements that bids must pass before entering the auction.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QualificationCriteria {
    /// Minimum acceptable capacity factor \[%\]
    pub min_capacity_factor_pct: f64,
    /// Maximum distance from grid connection point \[km\]
    pub max_distance_to_grid_km: f64,
    /// Whether a formal grid connection study is required
    pub require_grid_connection_study: bool,
    /// Minimum local content threshold \[%\] (0 = no requirement)
    pub local_content_pct: f64,
}

impl Default for QualificationCriteria {
    fn default() -> Self {
        Self {
            min_capacity_factor_pct: 20.0,
            max_distance_to_grid_km: f64::MAX,
            require_grid_connection_study: false,
            local_content_pct: 0.0,
        }
    }
}

/// Full auction configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AuctionConfig {
    /// Price-setting mechanism
    pub mechanism: AuctionMechanism,
    /// Support scheme to be granted to winners
    pub support_scheme: SupportScheme,
    /// Total renewable capacity to procure \[MW\]
    pub procurement_target_mw: f64,
    /// Auction price ceiling — bids above this are disqualified \[$/MWh\]
    pub max_bid_price_per_mwh: f64,
    /// Auction price floor — bids below this are disqualified \[$/MWh\]
    pub min_bid_price_per_mwh: f64,
    /// Optional per-technology procurement bands
    pub technology_bands: Vec<TechBand>,
    /// Bid qualification criteria
    pub qualification_criteria: QualificationCriteria,
}

// ─────────────────────────────────────────────────────────────────────────────
// Output structures
// ─────────────────────────────────────────────────────────────────────────────

/// A winning bid after auction clearing.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AuctionWinner {
    /// Bidder identifier
    pub bidder_id: String,
    /// Technology
    pub technology: RenewableTech,
    /// Awarded capacity \[MW\]
    pub capacity_mw: f64,
    /// Settlement price (may differ from bid under uniform price) \[$/MWh\]
    pub awarded_price_per_mwh: f64,
    /// Short description of the support scheme applied
    pub support_scheme: String,
}

/// Aggregate result of an auction round.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AuctionResult {
    /// List of winning bidders with their settlement terms
    pub winners: Vec<AuctionWinner>,
    /// Clearing price (= last accepted bid for uniform price) \[$/MWh\]
    pub clearing_price_per_mwh: f64,
    /// Sum of awarded capacities \[MW\]
    pub total_mw_awarded: f64,
    /// Total submitted MW / procurement target (> 1 means oversubscribed)
    pub oversubscription_ratio: f64,
    /// Estimated annual support cost \[M$/year\]
    pub auction_cost_m_usd_per_year: f64,
}

/// Annual CfD cash-flow entry.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CfdPayment {
    /// Calendar or contract year index
    pub year: usize,
    /// Generation output in that year \[MWh\]
    pub generation_mwh: f64,
    /// Strike price \[$/MWh\]
    pub strike_price: f64,
    /// Market reference price in that year \[$/MWh\]
    pub reference_price: f64,
    /// Net CfD payment \[$\]: positive = generator receives, negative = pays back
    pub net_payment_usd: f64,
}

/// Welfare and efficiency metrics for a completed auction.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AuctionMetrics {
    /// Present-value consumer cost \[M$/year\]
    pub consumer_cost_m_usd_per_year: f64,
    /// Consumer surplus = (price_cap − clearing) × awarded_mw \[M$\]
    pub consumer_surplus_m_usd: f64,
    /// Producer surplus = (awarded_price − LCOE) × output \[M$\]
    pub producer_surplus_m_usd: f64,
    /// Oversubscription ratio (same as in AuctionResult)
    pub oversubscription_ratio: f64,
}

/// Technology portfolio breakdown report.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TechMixReport {
    /// Total awarded MW per technology name \[(name, MW)\]
    pub mw_by_tech: Vec<(String, f64)>,
    /// Weighted average clearing price per technology \[(name, $/MWh)\]
    pub avg_price_by_tech: Vec<(String, f64)>,
    /// Herfindahl-Hirschman Index of MW concentration (0–1)
    pub hhi_concentration: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Core auction engine
// ─────────────────────────────────────────────────────────────────────────────

/// Renewable energy auction engine.
///
/// # Example
/// ```rust,no_run
/// use oxigrid::optimize::market::renewable_auction::{
///     RenewableAuction, AuctionConfig, AuctionMechanism, SupportScheme,
///     QualificationCriteria, RenewableBid, RenewableTech,
/// };
///
/// let config = AuctionConfig {
///     mechanism: AuctionMechanism::UniformPrice,
///     support_scheme: SupportScheme::Cfd {
///         strike_price_per_mwh: 70.0,
///         reference_price_per_mwh: 55.0,
///         duration_years: 15,
///     },
///     procurement_target_mw: 500.0,
///     max_bid_price_per_mwh: 100.0,
///     min_bid_price_per_mwh: 0.0,
///     technology_bands: vec![],
///     qualification_criteria: QualificationCriteria::default(),
/// };
/// let mut auction = RenewableAuction::new("UK-CfD-2024".into(), config);
/// ```
pub struct RenewableAuction {
    /// Auction identifier
    pub auction_id: String,
    /// All submitted bids
    pub bids: Vec<RenewableBid>,
    /// Auction configuration
    pub config: AuctionConfig,
    /// Internal event log
    auction_log: Vec<String>,
}

impl RenewableAuction {
    /// Construct a new auction with no bids yet.
    pub fn new(auction_id: String, config: AuctionConfig) -> Self {
        Self {
            auction_id,
            bids: Vec::new(),
            config,
            auction_log: Vec::new(),
        }
    }

    /// Submit a bid into the auction.
    pub fn submit_bid(&mut self, bid: RenewableBid) {
        self.auction_log.push(format!(
            "BID RECEIVED: {} | {:.1} MW @ ${:.2}/MWh",
            bid.bidder_id, bid.capacity_mw, bid.strike_price_per_mwh
        ));
        self.bids.push(bid);
    }

    /// Return a read-only reference to the event log.
    pub fn log(&self) -> &[String] {
        &self.auction_log
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 1. Qualification
    // ─────────────────────────────────────────────────────────────────────────

    /// Filter bids that meet the qualification criteria and price bounds.
    ///
    /// A bid passes when:
    /// - `capacity_factor_pct` ≥ `min_capacity_factor_pct`
    /// - `strike_price_per_mwh` ≤ `max_bid_price_per_mwh`
    /// - `strike_price_per_mwh` ≥ `min_bid_price_per_mwh`
    /// - If technology bands are defined for this tech, price ≤ band cap
    pub fn qualify_bids(&self) -> Vec<&RenewableBid> {
        let crit = &self.config.qualification_criteria;

        self.bids
            .iter()
            .filter(|bid| {
                // Capacity factor floor
                if bid.capacity_factor_pct < crit.min_capacity_factor_pct {
                    return false;
                }
                // Global price ceiling
                if bid.strike_price_per_mwh > self.config.max_bid_price_per_mwh {
                    return false;
                }
                // Global price floor
                if bid.strike_price_per_mwh < self.config.min_bid_price_per_mwh {
                    return false;
                }
                // Technology band price cap
                for band in &self.config.technology_bands {
                    if band.technology == bid.technology
                        && bid.strike_price_per_mwh > band.max_price_per_mwh
                    {
                        return false;
                    }
                }
                true
            })
            .collect()
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 2. Auction clearing
    // ─────────────────────────────────────────────────────────────────────────

    /// Run the auction and return clearing results.
    ///
    /// Algorithm (for sealed-bid mechanisms):
    /// 1. Qualify bids.
    /// 2. Sort by `strike_price_per_mwh` ascending (merit order).
    /// 3. Greedily select cheapest bids while honouring per-tech band quotas
    ///    and the global `procurement_target_mw`.
    /// 4. Apply price settlement according to mechanism:
    ///    - `PayAsBid`: each winner paid own bid.
    ///    - `UniformPrice` / `PayAsClear`: all winners paid last accepted price.
    ///    - `DescendingClock`: delegate to `descending_clock_round`.
    pub fn run_auction(&mut self) -> Result<AuctionResult, String> {
        let total_submitted_mw: f64 = self.bids.iter().map(|b| b.capacity_mw).sum();

        // Collect owned copies so we can mutably borrow self later for logging.
        let mut qualified: Vec<RenewableBid> = self.qualify_bids().into_iter().cloned().collect();
        // Merit-order sort: cheapest first
        qualified.sort_by(|a, b| {
            a.strike_price_per_mwh
                .partial_cmp(&b.strike_price_per_mwh)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        self.auction_log.push(format!(
            "AUCTION START: {} qualified bids, target {:.1} MW",
            qualified.len(),
            self.config.procurement_target_mw
        ));

        // Descending-clock: iterative price rounds
        if self.config.mechanism == AuctionMechanism::DescendingClock {
            return self.run_descending_clock_owned(&qualified, total_submitted_mw);
        }

        // Sealed-bid clearing
        let mut remaining_target = self.config.procurement_target_mw;
        // Track MW awarded per technology band
        let mut band_awarded: HashMap<String, f64> = HashMap::new();
        let mut winners: Vec<AuctionWinner> = Vec::new();
        let mut clearing_price = 0.0_f64;

        for bid in &qualified {
            if remaining_target <= 0.0 {
                break;
            }
            // Check technology band quota
            let band_remaining = self.band_remaining_mw(bid, &band_awarded);
            let award_mw = bid.capacity_mw.min(remaining_target).min(band_remaining);
            if award_mw <= 0.0 {
                self.auction_log.push(format!(
                    "SKIP (band full): {} [{:?}]",
                    bid.bidder_id, bid.technology
                ));
                continue;
            }

            clearing_price = bid.strike_price_per_mwh;
            remaining_target -= award_mw;
            *band_awarded
                .entry(bid.technology.name().to_owned())
                .or_insert(0.0) += award_mw;

            let scheme_label = self.support_scheme_label();
            winners.push(AuctionWinner {
                bidder_id: bid.bidder_id.clone(),
                technology: bid.technology.clone(),
                capacity_mw: award_mw,
                awarded_price_per_mwh: bid.strike_price_per_mwh, // tentative; overridden below for uniform
                support_scheme: scheme_label,
            });

            self.auction_log.push(format!(
                "SELECTED: {} | {:.1} MW @ ${:.2}/MWh",
                bid.bidder_id, award_mw, bid.strike_price_per_mwh
            ));
        }

        // For uniform / pay-as-clear: override all awarded prices to clearing
        if matches!(
            self.config.mechanism,
            AuctionMechanism::UniformPrice | AuctionMechanism::PayAsClear
        ) {
            for w in winners.iter_mut() {
                w.awarded_price_per_mwh = clearing_price;
            }
        }

        let total_mw_awarded: f64 = winners.iter().map(|w| w.capacity_mw).sum();
        let oversubscription_ratio = if self.config.procurement_target_mw > 0.0 {
            total_submitted_mw / self.config.procurement_target_mw
        } else {
            0.0
        };

        let auction_cost_m_usd_per_year = self.annual_support_cost(&winners, clearing_price) / 1e6;

        self.auction_log.push(format!(
            "AUCTION CLOSE: {:.1} MW awarded @ ${:.2}/MWh clearing, cost ${:.2}M/yr",
            total_mw_awarded, clearing_price, auction_cost_m_usd_per_year
        ));

        Ok(AuctionResult {
            winners,
            clearing_price_per_mwh: clearing_price,
            total_mw_awarded,
            oversubscription_ratio,
            auction_cost_m_usd_per_year,
        })
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 3. CfD payment stream
    // ─────────────────────────────────────────────────────────────────────────

    /// Compute annual CfD cash-flows for a portfolio of winners.
    ///
    /// For each year `i`:
    /// - `generation_mwh = generation_profiles[i]` (sum across portfolio)
    /// - `net_payment = (strike − reference_prices[i]) × generation_mwh`
    ///   - Positive → generator receives top-up from settlement body.
    ///   - Negative → generator pays back excess to settlement body.
    ///
    /// Only valid for winners under a [`SupportScheme::Cfd`]; other winners
    /// are skipped.  If `reference_prices` or `generation_profiles` are
    /// shorter than `duration_years`, only the available years are returned.
    pub fn calculate_cfd_payments(
        &self,
        winners: &[AuctionWinner],
        reference_prices: &[f64],
        generation_profiles: &[f64],
    ) -> Vec<CfdPayment> {
        // Determine strike and duration from the auction-level support scheme.
        // Only applicable to CfD auctions; return empty for other schemes.
        if winners.is_empty() {
            return vec![];
        }
        let (strike, duration_years) = match &self.config.support_scheme {
            SupportScheme::Cfd {
                strike_price_per_mwh,
                duration_years,
                ..
            } => (*strike_price_per_mwh, *duration_years),
            _ => return vec![],
        };

        let n_years = duration_years
            .min(reference_prices.len())
            .min(generation_profiles.len());

        (0..n_years)
            .map(|i| {
                let ref_price = reference_prices[i];
                let gen_mwh = generation_profiles[i];
                let net_payment_usd = (strike - ref_price) * gen_mwh;
                CfdPayment {
                    year: i + 1,
                    generation_mwh: gen_mwh,
                    strike_price: strike,
                    reference_price: ref_price,
                    net_payment_usd,
                }
            })
            .collect()
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 4. LCOE estimation
    // ─────────────────────────────────────────────────────────────────────────

    /// Estimate the Levelised Cost of Electricity for a bid \[$/MWh\].
    ///
    /// Formula:
    /// ```text
    /// LCOE = (CAPEX × CRF + OPEX) / (CF × 8760 × capacity_mw)
    /// ```
    /// where `CRF = r(1+r)^n / ((1+r)^n − 1)`, `r = 0.07`.
    ///
    /// CAPEX and OPEX are sourced from [`RenewableTech`] benchmarks and
    /// scaled by `capacity_mw`.
    pub fn lcoe_estimation(&self, bid: &RenewableBid) -> f64 {
        let r = 0.07_f64;
        let n = bid.lifetime_years as f64;
        let capex = bid.technology.capex_per_mw() * bid.capacity_mw;
        let opex = bid.technology.opex_per_mw_yr() * bid.capacity_mw; // $/year

        let crf = if n > 0.0 {
            let factor = (1.0 + r).powf(n);
            r * factor / (factor - 1.0)
        } else {
            1.0
        };

        let annual_capex = capex * crf; // $/year
        let cf = bid.capacity_factor_pct / 100.0;
        let annual_output_mwh = cf * 8760.0 * bid.capacity_mw;

        if annual_output_mwh < 1e-6 {
            return f64::INFINITY;
        }

        (annual_capex + opex) / annual_output_mwh
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 5. Efficiency metrics
    // ─────────────────────────────────────────────────────────────────────────

    /// Compute welfare and efficiency metrics for a completed auction.
    ///
    /// - **Consumer cost**: Σ winner_mw × awarded_price × avg_CF × 8760 \[M$/yr\]
    /// - **Consumer surplus**: Σ (max_price_cap − clearing) × winner_mw \[M$\]
    /// - **Producer surplus**: Σ (awarded_price − LCOE) × output \[M$/yr\]
    pub fn auction_efficiency_metrics(&self, auction_result: &AuctionResult) -> AuctionMetrics {
        let clearing = auction_result.clearing_price_per_mwh;
        let max_price = self.config.max_bid_price_per_mwh;

        let mut consumer_cost = 0.0_f64;
        let mut consumer_surplus = 0.0_f64;
        let mut producer_surplus = 0.0_f64;

        for winner in &auction_result.winners {
            // Look up the original bid to get CF and LCOE
            let cf = self
                .bids
                .iter()
                .find(|b| b.bidder_id == winner.bidder_id)
                .map(|b| b.capacity_factor_pct / 100.0)
                .unwrap_or(0.25);

            let annual_output_mwh = winner.capacity_mw * cf * 8760.0;
            consumer_cost += winner.capacity_mw * winner.awarded_price_per_mwh * cf * 8760.0;
            consumer_surplus += (max_price - clearing) * winner.capacity_mw;

            // Producer surplus: use LCOE if bid is available
            if let Some(bid) = self.bids.iter().find(|b| b.bidder_id == winner.bidder_id) {
                let lcoe = self.lcoe_estimation(bid);
                let ps = (winner.awarded_price_per_mwh - lcoe) * annual_output_mwh;
                producer_surplus += ps;
            }
        }

        AuctionMetrics {
            consumer_cost_m_usd_per_year: consumer_cost / 1e6,
            consumer_surplus_m_usd: consumer_surplus / 1e6,
            producer_surplus_m_usd: producer_surplus / 1e6,
            oversubscription_ratio: auction_result.oversubscription_ratio,
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 6. Technology mix analysis
    // ─────────────────────────────────────────────────────────────────────────

    /// Analyse the portfolio of auction winners by technology.
    ///
    /// Returns:
    /// - MW awarded per technology (sorted descending by MW)
    /// - Weighted average clearing price per technology
    /// - HHI concentration index (0 = fully diversified, 1 = monopoly)
    pub fn technology_mix_analysis(&self, winners: &[AuctionWinner]) -> TechMixReport {
        let mut mw_map: HashMap<String, f64> = HashMap::new();
        let mut price_x_mw: HashMap<String, f64> = HashMap::new();

        for w in winners {
            let key = w.technology.name().to_owned();
            *mw_map.entry(key.clone()).or_insert(0.0) += w.capacity_mw;
            *price_x_mw.entry(key).or_insert(0.0) += w.awarded_price_per_mwh * w.capacity_mw;
        }

        let total_mw: f64 = mw_map.values().sum();

        let mut mw_by_tech: Vec<(String, f64)> =
            mw_map.iter().map(|(k, v)| (k.clone(), *v)).collect();
        mw_by_tech.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        let avg_price_by_tech: Vec<(String, f64)> = mw_by_tech
            .iter()
            .map(|(name, mw)| {
                let wavg = if *mw > 0.0 {
                    price_x_mw.get(name).copied().unwrap_or(0.0) / mw
                } else {
                    0.0
                };
                (name.clone(), wavg)
            })
            .collect();

        // HHI: sum of squared market shares
        let hhi = if total_mw > 0.0 {
            mw_by_tech
                .iter()
                .map(|(_, mw)| {
                    let share = mw / total_mw;
                    share * share
                })
                .sum()
        } else {
            0.0
        };

        TechMixReport {
            mw_by_tech,
            avg_price_by_tech,
            hhi_concentration: hhi,
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 7. Descending-clock round
    // ─────────────────────────────────────────────────────────────────────────

    /// Execute one round of a descending-clock auction.
    ///
    /// - If `submitted_mw > procurement_target_mw`: lower price by 5 %, return
    ///   `(new_price, false)`.
    /// - If `submitted_mw ≤ procurement_target_mw`: auction clears, return
    ///   `(current_price, true)`.
    pub fn descending_clock_round(&self, current_price: f64, submitted_mw: f64) -> (f64, bool) {
        if submitted_mw > self.config.procurement_target_mw {
            let new_price = current_price * 0.95;
            (new_price, false)
        } else {
            (current_price, true)
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Private helpers
    // ─────────────────────────────────────────────────────────────────────────

    /// Human-readable label for the configured support scheme.
    fn support_scheme_label(&self) -> String {
        match &self.config.support_scheme {
            SupportScheme::Cfd { duration_years, .. } => {
                format!("CfD ({duration_years}yr)")
            }
            SupportScheme::FeedInTariff { duration_years, .. } => {
                format!("FiT ({duration_years}yr)")
            }
            SupportScheme::RenewableObligation { .. } => "RO".to_owned(),
            SupportScheme::PremiumFeedIn { .. } => "Premium FiT".to_owned(),
        }
    }

    /// Remaining band quota for the technology of `bid` \[MW\].
    /// Returns `f64::MAX` if no band is defined for this technology.
    fn band_remaining_mw(&self, bid: &RenewableBid, awarded: &HashMap<String, f64>) -> f64 {
        let band = self
            .config
            .technology_bands
            .iter()
            .find(|b| b.technology == bid.technology);

        match band {
            None => f64::MAX,
            Some(b) => {
                let already = awarded.get(bid.technology.name()).copied().unwrap_or(0.0);
                (b.quota_mw - already).max(0.0)
            }
        }
    }

    /// Estimated annual support cost summed across all winners \[$/year\].
    fn annual_support_cost(&self, winners: &[AuctionWinner], clearing_price: f64) -> f64 {
        winners
            .iter()
            .map(|w| {
                let cf = self
                    .bids
                    .iter()
                    .find(|b| b.bidder_id == w.bidder_id)
                    .map(|b| b.capacity_factor_pct / 100.0)
                    .unwrap_or(0.30);
                // Support cost = (awarded_price - market_ref) × output, or full tariff for FiT
                let effective_support = match &self.config.support_scheme {
                    SupportScheme::Cfd {
                        reference_price_per_mwh,
                        ..
                    } => (clearing_price - reference_price_per_mwh).max(0.0),
                    SupportScheme::FeedInTariff { tariff_per_mwh, .. } => *tariff_per_mwh,
                    SupportScheme::RenewableObligation {
                        certificate_value_per_mwh,
                    } => *certificate_value_per_mwh,
                    SupportScheme::PremiumFeedIn { premium_per_mwh } => *premium_per_mwh,
                };
                w.capacity_mw * cf * 8760.0 * effective_support
            })
            .sum()
    }

    /// Internal: run descending-clock clearing with owned bid snapshots.
    fn run_descending_clock_owned(
        &mut self,
        qualified: &[RenewableBid],
        total_submitted_mw: f64,
    ) -> Result<AuctionResult, String> {
        // Start at the price ceiling
        let mut current_price = self.config.max_bid_price_per_mwh;
        let max_rounds = 200_usize;

        for round in 0..max_rounds {
            // Compute MW supplied at or below current_price
            let submitted: f64 = qualified
                .iter()
                .filter(|b| b.strike_price_per_mwh <= current_price)
                .map(|b| b.capacity_mw)
                .sum();

            self.auction_log.push(format!(
                "DCR round {round}: price=${current_price:.2}/MWh  submitted={submitted:.1} MW"
            ));

            let (new_price, cleared) = self.descending_clock_round(current_price, submitted);
            if cleared {
                // Build winner list: all qualified at or below clearing price
                let mut remaining = self.config.procurement_target_mw;
                let mut band_awarded: HashMap<String, f64> = HashMap::new();
                let mut winners: Vec<AuctionWinner> = Vec::new();

                for bid in qualified
                    .iter()
                    .filter(|b| b.strike_price_per_mwh <= current_price)
                {
                    if remaining <= 0.0 {
                        break;
                    }
                    let band_rem = self.band_remaining_mw(bid, &band_awarded);
                    let award_mw = bid.capacity_mw.min(remaining).min(band_rem);
                    if award_mw <= 0.0 {
                        continue;
                    }
                    remaining -= award_mw;
                    *band_awarded
                        .entry(bid.technology.name().to_owned())
                        .or_insert(0.0) += award_mw;
                    let scheme_label = self.support_scheme_label();
                    winners.push(AuctionWinner {
                        bidder_id: bid.bidder_id.clone(),
                        technology: bid.technology.clone(),
                        capacity_mw: award_mw,
                        awarded_price_per_mwh: current_price,
                        support_scheme: scheme_label,
                    });
                }

                let total_mw_awarded: f64 = winners.iter().map(|w| w.capacity_mw).sum();
                let oversubscription_ratio = if self.config.procurement_target_mw > 0.0 {
                    total_submitted_mw / self.config.procurement_target_mw
                } else {
                    0.0
                };
                let auction_cost_m_usd_per_year =
                    self.annual_support_cost(&winners, current_price) / 1e6;

                return Ok(AuctionResult {
                    winners,
                    clearing_price_per_mwh: current_price,
                    total_mw_awarded,
                    oversubscription_ratio,
                    auction_cost_m_usd_per_year,
                });
            }
            current_price = new_price;

            // Safety: if price falls below floor, stop iterating
            if current_price < self.config.min_bid_price_per_mwh {
                break;
            }
        }

        Err(format!(
            "Descending-clock auction '{}' did not clear after {max_rounds} rounds",
            self.auction_id
        ))
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_bid(id: &str, tech: RenewableTech, mw: f64, price: f64, cf_pct: f64) -> RenewableBid {
        RenewableBid {
            bidder_id: id.to_owned(),
            technology: tech,
            capacity_mw: mw,
            strike_price_per_mwh: price,
            capacity_factor_pct: cf_pct,
            commissioning_year: 2026,
            lifetime_years: 20,
            location: "TestGrid".to_owned(),
            grid_connection_cost_m_usd: 5.0,
        }
    }

    fn base_config(mechanism: AuctionMechanism) -> AuctionConfig {
        AuctionConfig {
            mechanism,
            support_scheme: SupportScheme::Cfd {
                strike_price_per_mwh: 80.0,
                reference_price_per_mwh: 55.0,
                duration_years: 15,
            },
            procurement_target_mw: 300.0,
            max_bid_price_per_mwh: 120.0,
            min_bid_price_per_mwh: 0.0,
            technology_bands: vec![],
            qualification_criteria: QualificationCriteria::default(),
        }
    }

    // ── Test 1: Pay-as-bid — each winner paid own bid price ──────────────────
    #[test]
    fn test_pay_as_bid_own_price() {
        let mut auction =
            RenewableAuction::new("T1".into(), base_config(AuctionMechanism::PayAsBid));
        auction.submit_bid(make_bid("A", RenewableTech::SolarPv, 150.0, 60.0, 25.0));
        auction.submit_bid(make_bid("B", RenewableTech::OnshoreWind, 200.0, 75.0, 35.0));

        let result = auction.run_auction().expect("auction should succeed");
        for winner in &result.winners {
            let original_bid = auction
                .bids
                .iter()
                .find(|b| b.bidder_id == winner.bidder_id)
                .expect("bid must exist");
            assert!(
                (winner.awarded_price_per_mwh - original_bid.strike_price_per_mwh).abs() < 1e-9,
                "Pay-as-bid: {} should be paid own bid {:.2}, got {:.2}",
                winner.bidder_id,
                original_bid.strike_price_per_mwh,
                winner.awarded_price_per_mwh
            );
        }
    }

    // ── Test 2: Uniform price — all winners paid clearing price ───────────────
    #[test]
    fn test_uniform_price_clearing() {
        let mut auction =
            RenewableAuction::new("T2".into(), base_config(AuctionMechanism::UniformPrice));
        auction.submit_bid(make_bid("A", RenewableTech::SolarPv, 150.0, 60.0, 25.0));
        auction.submit_bid(make_bid("B", RenewableTech::OnshoreWind, 200.0, 75.0, 35.0));

        let result = auction.run_auction().expect("auction should succeed");
        let cp = result.clearing_price_per_mwh;
        for winner in &result.winners {
            assert!(
                (winner.awarded_price_per_mwh - cp).abs() < 1e-9,
                "Uniform price: {} should be paid clearing {:.2}, got {:.2}",
                winner.bidder_id,
                cp,
                winner.awarded_price_per_mwh
            );
        }
    }

    // ── Test 3: Merit order — cheapest bid wins first ─────────────────────────
    #[test]
    fn test_merit_order_cheapest_first() {
        let mut auction =
            RenewableAuction::new("T3".into(), base_config(AuctionMechanism::UniformPrice));
        auction.submit_bid(make_bid(
            "Expensive",
            RenewableTech::Biomass,
            100.0,
            110.0,
            30.0,
        ));
        auction.submit_bid(make_bid("Cheap", RenewableTech::SolarPv, 100.0, 55.0, 25.0));
        auction.submit_bid(make_bid(
            "Mid",
            RenewableTech::OnshoreWind,
            200.0,
            70.0,
            35.0,
        ));

        // Target 150 MW → should pick Cheap (100) + Mid (50), not Expensive
        let mut cfg = base_config(AuctionMechanism::UniformPrice);
        cfg.procurement_target_mw = 150.0;
        let mut auction2 = RenewableAuction::new("T3b".into(), cfg);
        auction2.submit_bid(make_bid(
            "Expensive",
            RenewableTech::Biomass,
            100.0,
            110.0,
            30.0,
        ));
        auction2.submit_bid(make_bid("Cheap", RenewableTech::SolarPv, 100.0, 55.0, 25.0));
        auction2.submit_bid(make_bid(
            "Mid",
            RenewableTech::OnshoreWind,
            200.0,
            70.0,
            35.0,
        ));

        let result = auction2.run_auction().expect("should succeed");
        let winner_ids: Vec<&str> = result
            .winners
            .iter()
            .map(|w| w.bidder_id.as_str())
            .collect();
        assert!(
            winner_ids.contains(&"Cheap"),
            "Cheapest bid should win: {winner_ids:?}"
        );
        assert!(
            !winner_ids.contains(&"Expensive"),
            "Expensive bid should not win within 150 MW: {winner_ids:?}"
        );
    }

    // ── Test 4: Technology band — quota per tech respected ────────────────────
    #[test]
    fn test_technology_band_quota() {
        let mut cfg = base_config(AuctionMechanism::UniformPrice);
        cfg.procurement_target_mw = 400.0;
        cfg.technology_bands = vec![TechBand {
            technology: RenewableTech::SolarPv,
            quota_mw: 100.0, // cap solar at 100 MW
            max_price_per_mwh: 120.0,
        }];

        let mut auction = RenewableAuction::new("T4".into(), cfg);
        // Submit 300 MW of solar (very cheap) + 200 MW wind
        auction.submit_bid(make_bid("Sol1", RenewableTech::SolarPv, 150.0, 50.0, 25.0));
        auction.submit_bid(make_bid("Sol2", RenewableTech::SolarPv, 150.0, 55.0, 25.0));
        auction.submit_bid(make_bid(
            "Wind1",
            RenewableTech::OnshoreWind,
            200.0,
            70.0,
            35.0,
        ));

        let result = auction.run_auction().expect("should succeed");
        let solar_awarded: f64 = result
            .winners
            .iter()
            .filter(|w| w.technology == RenewableTech::SolarPv)
            .map(|w| w.capacity_mw)
            .sum();
        assert!(
            solar_awarded <= 100.0 + 1e-6,
            "Solar band quota 100 MW must be respected; got {solar_awarded:.1} MW"
        );
    }

    // ── Test 5: Qualification — bid above price ceiling excluded ──────────────
    #[test]
    fn test_qualification_price_ceiling() {
        let cfg = base_config(AuctionMechanism::UniformPrice); // ceiling = 120
        let mut auction = RenewableAuction::new("T5".into(), cfg);
        auction.submit_bid(make_bid(
            "TooExpensive",
            RenewableTech::Tidal,
            50.0,
            150.0,
            30.0,
        ));
        auction.submit_bid(make_bid("OK", RenewableTech::SolarPv, 300.0, 70.0, 25.0));

        let qualified = auction.qualify_bids();
        let ids: Vec<&str> = qualified.iter().map(|b| b.bidder_id.as_str()).collect();
        assert!(
            !ids.contains(&"TooExpensive"),
            "Bid above ceiling must be excluded: {ids:?}"
        );
        assert!(
            ids.contains(&"OK"),
            "Bid within ceiling must qualify: {ids:?}"
        );
    }

    // ── Test 6: CfD strike > reference → generator receives payment ──────────
    #[test]
    fn test_cfd_positive_payment() {
        let cfg = base_config(AuctionMechanism::UniformPrice); // strike=80, ref=55
        let mut auction = RenewableAuction::new("T6".into(), cfg);
        auction.submit_bid(make_bid(
            "W1",
            RenewableTech::OnshoreWind,
            200.0,
            70.0,
            35.0,
        ));
        let result = auction.run_auction().expect("should succeed");

        let ref_prices = vec![50.0_f64; 15];
        let gen_profiles = vec![200.0 * 0.35 * 8760.0; 15];
        let payments = auction.calculate_cfd_payments(&result.winners, &ref_prices, &gen_profiles);

        assert!(!payments.is_empty(), "Should produce CfD payments");
        for p in &payments {
            assert!(
                p.net_payment_usd > 0.0,
                "Strike 80 > reference 50: generator should receive; got {:.2}",
                p.net_payment_usd
            );
        }
    }

    // ── Test 7: CfD strike < reference → generator pays back ─────────────────
    #[test]
    fn test_cfd_negative_payment() {
        let mut cfg = base_config(AuctionMechanism::UniformPrice);
        // Set reference price ABOVE strike so generator pays back
        cfg.support_scheme = SupportScheme::Cfd {
            strike_price_per_mwh: 60.0,
            reference_price_per_mwh: 80.0,
            duration_years: 5,
        };
        let mut auction = RenewableAuction::new("T7".into(), cfg);
        auction.submit_bid(make_bid("W1", RenewableTech::SolarPv, 200.0, 60.0, 25.0));
        let result = auction.run_auction().expect("should succeed");

        // reference price 90 > strike 60 → generator pays back
        let ref_prices = vec![90.0_f64; 5];
        let gen_profiles = vec![50_000.0_f64; 5];
        let payments = auction.calculate_cfd_payments(&result.winners, &ref_prices, &gen_profiles);

        assert!(!payments.is_empty(), "Should produce CfD payments");
        for p in &payments {
            assert!(
                p.net_payment_usd < 0.0,
                "Strike 60 < reference 90: generator pays back; got {:.2}",
                p.net_payment_usd
            );
        }
    }

    // ── Test 8: Consumer surplus = (max_price - clearing) × awarded_mw ───────
    #[test]
    fn test_consumer_surplus_formula() {
        let mut cfg = base_config(AuctionMechanism::UniformPrice);
        cfg.max_bid_price_per_mwh = 100.0;
        cfg.procurement_target_mw = 200.0;
        let mut auction = RenewableAuction::new("T8".into(), cfg);
        auction.submit_bid(make_bid("A", RenewableTech::SolarPv, 100.0, 60.0, 25.0));
        auction.submit_bid(make_bid("B", RenewableTech::OnshoreWind, 150.0, 70.0, 35.0));

        let result = auction.run_auction().expect("should succeed");
        let metrics = auction.auction_efficiency_metrics(&result);

        // Clearing = 70 (last accepted), max = 100
        let clearing = result.clearing_price_per_mwh;
        let total_mw: f64 = result.winners.iter().map(|w| w.capacity_mw).sum();
        let expected_cs_musd = (100.0 - clearing) * total_mw / 1e6;

        assert!(
            (metrics.consumer_surplus_m_usd - expected_cs_musd).abs() < 1e-6,
            "Consumer surplus mismatch: expected {expected_cs_musd:.6} got {:.6}",
            metrics.consumer_surplus_m_usd
        );
    }

    // ── Test 9: Oversubscription ratio ────────────────────────────────────────
    #[test]
    fn test_oversubscription_ratio() {
        let mut cfg = base_config(AuctionMechanism::UniformPrice);
        cfg.procurement_target_mw = 100.0;
        let mut auction = RenewableAuction::new("T9".into(), cfg);
        auction.submit_bid(make_bid("A", RenewableTech::SolarPv, 150.0, 60.0, 25.0));
        auction.submit_bid(make_bid("B", RenewableTech::OnshoreWind, 200.0, 70.0, 35.0));

        let result = auction.run_auction().expect("should succeed");
        // 350 MW total submitted / 100 MW target = 3.5
        assert!(
            (result.oversubscription_ratio - 3.5).abs() < 1e-6,
            "Oversubscription ratio should be 3.5, got {:.4}",
            result.oversubscription_ratio
        );
    }

    // ── Test 10: Technology mix HHI ───────────────────────────────────────────
    #[test]
    fn test_tech_mix_hhi_monopoly() {
        let cfg = base_config(AuctionMechanism::UniformPrice);
        let mut auction = RenewableAuction::new("T10".into(), cfg);
        auction.submit_bid(make_bid("S1", RenewableTech::SolarPv, 200.0, 60.0, 25.0));
        auction.submit_bid(make_bid("S2", RenewableTech::SolarPv, 200.0, 65.0, 25.0));

        let result = auction.run_auction().expect("should succeed");
        let report = auction.technology_mix_analysis(&result.winners);
        // All solar → HHI = 1.0
        assert!(
            (report.hhi_concentration - 1.0).abs() < 1e-6,
            "Single-tech HHI should be 1.0, got {:.4}",
            report.hhi_concentration
        );
    }

    // ── Test 11: Descending-clock clears when supply ≤ target ─────────────────
    #[test]
    fn test_descending_clock_clears() {
        let mut cfg = base_config(AuctionMechanism::DescendingClock);
        cfg.procurement_target_mw = 200.0;
        cfg.max_bid_price_per_mwh = 100.0;
        let mut auction = RenewableAuction::new("T11".into(), cfg);
        // At price 80 only 100 MW qualify (bid below 80)
        auction.submit_bid(make_bid("A", RenewableTech::SolarPv, 100.0, 75.0, 25.0));
        // At any price ≥ 100 this second bid adds another 200 MW
        auction.submit_bid(make_bid("B", RenewableTech::OnshoreWind, 200.0, 95.0, 35.0));

        let result = auction
            .run_auction()
            .expect("descending clock should clear");
        assert!(
            result.total_mw_awarded > 0.0,
            "Should award some MW: {result:?}"
        );
    }

    // ── Test 12: LCOE > 0 for all technologies ────────────────────────────────
    #[test]
    fn test_lcoe_positive() {
        let cfg = base_config(AuctionMechanism::UniformPrice);
        let auction = RenewableAuction::new("T12".into(), cfg);
        let techs = [
            RenewableTech::SolarPv,
            RenewableTech::OnshoreWind,
            RenewableTech::OffshoreWind,
        ];
        for tech in &techs {
            let bid = make_bid("X", tech.clone(), 100.0, 80.0, 30.0);
            let lcoe = auction.lcoe_estimation(&bid);
            assert!(lcoe > 0.0, "LCOE must be positive for {tech:?}: {lcoe}");
        }
    }
}
