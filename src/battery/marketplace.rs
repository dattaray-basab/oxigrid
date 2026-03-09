//! Battery lifecycle marketplace: asset registration, listing, trading,
//! and second-life value assessment.
//!
//! Models the emerging battery swap and secondary-use market where
//! used battery packs (e.g. from EVs) are assessed, valued, and
//! traded for second-life applications such as grid storage.
//!
//! # References
//! - Harper et al., "Recycling lithium-ion batteries from electric vehicles",
//!   Nature, 2019.
//! - Canals Casals et al., "Reused second life batteries for aggregated
//!   demand response services", 2017.

use serde::{Deserialize, Serialize};
use std::fmt;

// ─────────────────────────────────────────────────────────────────────────────
// Error
// ─────────────────────────────────────────────────────────────────────────────

/// Errors from marketplace operations.
#[derive(Debug, Clone, PartialEq)]
pub enum MarketError {
    /// Asset with given ID not found in the marketplace.
    AssetNotFound(u64),
    /// Asset is already listed for sale.
    AlreadyListed(u64),
    /// Asset is not currently listed for sale.
    NotForSale(u64),
}

impl fmt::Display for MarketError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::AssetNotFound(id) => write!(f, "asset {id} not found"),
            Self::AlreadyListed(id) => write!(f, "asset {id} is already listed"),
            Self::NotForSale(id) => write!(f, "asset {id} is not for sale"),
        }
    }
}

impl std::error::Error for MarketError {}

// ─────────────────────────────────────────────────────────────────────────────
// Battery Chemistry
// ─────────────────────────────────────────────────────────────────────────────

/// Electrochemical cell chemistry of the battery asset.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BatteryChemistry {
    /// Lithium Nickel Manganese Cobalt Oxide — high energy density EV cells.
    NmcLithiumIon,
    /// Lithium Iron Phosphate — safe, long-cycle-life stationary cells.
    LfpLithiumIron,
    /// Lithium Titanate — extremely fast charge, very long cycle life.
    LtoLithiumTitanate,
    /// Lithium Nickel Cobalt Aluminum Oxide — high energy (Tesla 18650/21700).
    NcaLithiumNickel,
    /// Flooded or sealed lead-acid — lowest cost, lowest energy density.
    LeadAcid,
    /// Sodium-ion — emerging low-cost alternative to lithium-ion.
    SodiumIon,
}

impl BatteryChemistry {
    /// Reference market price for second-life cells \[USD/kWh\].
    ///
    /// Based on 2025 approximate market data; NMC > NCA > LTO > LFP > NaI > PbA.
    pub fn second_life_price_per_kwh(self) -> f64 {
        match self {
            Self::NmcLithiumIon => 150.0,
            Self::LfpLithiumIron => 120.0,
            Self::LtoLithiumTitanate => 200.0,
            Self::NcaLithiumNickel => 160.0,
            Self::LeadAcid => 60.0,
            Self::SodiumIon => 100.0,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Application
// ─────────────────────────────────────────────────────────────────────────────

/// Primary or intended application of the battery asset.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BatteryApplication {
    /// Battery Electric Vehicle or Plug-in Hybrid.
    ElectricVehicle,
    /// Utility-scale or behind-the-meter grid energy storage.
    GridStorage,
    /// Uninterruptible Power Supply / critical backup.
    Ups,
    /// Portable consumer electronics or power tools.
    Portable,
    /// Already operating in a second-life role.
    SecondLife,
}

// ─────────────────────────────────────────────────────────────────────────────
// Lifecycle Events
// ─────────────────────────────────────────────────────────────────────────────

/// Type of lifecycle event recorded for an asset.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum LifecycleEventType {
    /// Battery pack manufactured and assembled.
    Manufactured,
    /// Pack sold to first owner (OEM or end-user).
    Sold,
    /// Pack put into first operational use.
    FirstUseStart,
    /// Physical battery swap (module replaced or exchanged).
    SwapEvent,
    /// State-of-health assessment recorded.
    SohAssessment {
        /// Measured SoH at time of assessment \[%\].
        soh_pct: f64,
    },
    /// Pack repurposed for a second-life application.
    SecondLifeStart,
    /// Pack retired and sent for material recycling.
    Recycled,
}

/// A single lifecycle event attached to a battery asset.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BatteryLifecycleEvent {
    /// Calendar date of the event as `(year, month, day)`.
    pub timestamp: (usize, usize, usize),
    /// The type of lifecycle event.
    pub event_type: LifecycleEventType,
    /// Monetary value associated with the event \[USD\].
    pub value_usd: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Battery Asset
// ─────────────────────────────────────────────────────────────────────────────

/// A battery asset tracked in the marketplace.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BatteryAsset {
    /// Unique marketplace asset identifier.
    pub id: u64,
    /// Electrochemical chemistry of the pack.
    pub chemistry: BatteryChemistry,
    /// Factory-rated usable energy capacity \[kWh\].
    pub original_capacity_kwh: f64,
    /// Current usable energy capacity after degradation \[kWh\].
    pub current_capacity_kwh: f64,
    /// State of health — current / original capacity × 100 \[%\].
    pub current_soh_pct: f64,
    /// Equivalent full charge cycles accumulated.
    pub cycle_count: f64,
    /// Calendar age from manufacture date \[years\].
    pub calendar_age_years: f64,
    /// Application the battery was originally designed/sold for.
    pub original_application: BatteryApplication,
    /// Index of the current owner (marketplace participant ID).
    pub current_owner: usize,
    /// Ordered log of lifecycle events.
    pub history: Vec<BatteryLifecycleEvent>,
}

impl BatteryAsset {
    /// Convenience constructor.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        id: u64,
        chemistry: BatteryChemistry,
        original_capacity_kwh: f64,
        current_soh_pct: f64,
        cycle_count: f64,
        calendar_age_years: f64,
        original_application: BatteryApplication,
        owner: usize,
    ) -> Self {
        let current_capacity_kwh = original_capacity_kwh * current_soh_pct / 100.0;
        Self {
            id,
            chemistry,
            original_capacity_kwh,
            current_capacity_kwh,
            current_soh_pct,
            cycle_count,
            calendar_age_years,
            original_application,
            current_owner: owner,
            history: Vec::new(),
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Marketplace Offer
// ─────────────────────────────────────────────────────────────────────────────

/// A listing posted to the battery marketplace.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MarketplaceOffer {
    /// Asset being offered for sale.
    pub asset_id: u64,
    /// Seller's asking price \[USD\].
    pub asking_price_usd: f64,
    /// Applications this asset is suitable for.
    pub application_fit: Vec<BatteryApplication>,
    /// Estimated remaining useful life in a second-life role \[years\].
    pub remaining_useful_life_years: f64,
    /// Seller's guarantee: minimum energy throughput \[kWh\].
    pub warranty_kwh: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Marketplace
// ─────────────────────────────────────────────────────────────────────────────

/// Battery asset marketplace — registration, listing, trading, and analytics.
#[derive(Debug, Default, Clone)]
pub struct BatteryMarketplace {
    /// All registered battery assets (including sold ones).
    assets: Vec<BatteryAsset>,
    /// Active sale listings.
    offers: Vec<MarketplaceOffer>,
    /// Transaction log: `(asset_id, buyer_id, price_usd, soh_pct_at_sale)`.
    transaction_history: Vec<(u64, usize, f64, f64)>,
}

impl BatteryMarketplace {
    /// Create an empty marketplace.
    pub fn new() -> Self {
        Self::default()
    }

    /// Register a new battery asset and return its assigned ID.
    ///
    /// If the asset already has an ID of 0, a new ID is auto-assigned.
    pub fn register_asset(&mut self, mut asset: BatteryAsset) -> u64 {
        // Auto-assign ID = current length + 1 if asset.id == 0
        if asset.id == 0 {
            asset.id = self.assets.len() as u64 + 1;
        }
        let id = asset.id;
        self.assets.push(asset);
        id
    }

    /// List an asset for sale at the given asking price \[USD\].
    ///
    /// Returns an error if the asset is not found or is already listed.
    pub fn list_for_sale(
        &mut self,
        asset_id: u64,
        asking_price_usd: f64,
    ) -> Result<(), MarketError> {
        // Verify asset exists
        let asset = self
            .assets
            .iter()
            .find(|a| a.id == asset_id)
            .ok_or(MarketError::AssetNotFound(asset_id))?;

        // Check not already listed
        if self.offers.iter().any(|o| o.asset_id == asset_id) {
            return Err(MarketError::AlreadyListed(asset_id));
        }

        // Determine application fit based on SoH
        let soh = asset.current_soh_pct;
        let mut fit = Vec::new();
        if soh >= 80.0 {
            fit.push(BatteryApplication::ElectricVehicle);
            fit.push(BatteryApplication::GridStorage);
            fit.push(BatteryApplication::Ups);
        } else if soh >= 70.0 {
            fit.push(BatteryApplication::GridStorage);
            fit.push(BatteryApplication::Ups);
            fit.push(BatteryApplication::SecondLife);
        } else if soh >= 60.0 {
            fit.push(BatteryApplication::Ups);
            fit.push(BatteryApplication::SecondLife);
        } else {
            // Low SoH: only secondary
            fit.push(BatteryApplication::SecondLife);
        }

        // Estimated remaining life: each 1% SoH above 60% ≈ 0.2 years
        let rul_years = ((soh - 60.0).max(0.0) * 0.2).min(15.0);

        // Warranty throughput: current_capacity × remaining_cycles_estimate
        let remaining_cycles = ((soh - 60.0).max(0.0) / 40.0 * 1000.0).max(0.0);
        let warranty_kwh = asset.current_capacity_kwh * remaining_cycles;

        self.offers.push(MarketplaceOffer {
            asset_id,
            asking_price_usd,
            application_fit: fit,
            remaining_useful_life_years: rul_years,
            warranty_kwh,
        });

        Ok(())
    }

    /// Purchase a listed asset on behalf of `buyer_id`.
    ///
    /// Returns the final sale price \[USD\].
    /// Transfers ownership, removes the listing, and logs the transaction.
    pub fn buy_asset(&mut self, asset_id: u64, buyer_id: usize) -> Result<f64, MarketError> {
        // Find and remove the offer
        let offer_idx = self
            .offers
            .iter()
            .position(|o| o.asset_id == asset_id)
            .ok_or(MarketError::NotForSale(asset_id))?;
        let offer = self.offers.remove(offer_idx);

        // Find the asset and transfer ownership
        let asset = self
            .assets
            .iter_mut()
            .find(|a| a.id == asset_id)
            .ok_or(MarketError::AssetNotFound(asset_id))?;

        let soh_at_sale = asset.current_soh_pct;
        asset.current_owner = buyer_id;

        // Log a sale lifecycle event
        asset.history.push(BatteryLifecycleEvent {
            timestamp: (2026, 1, 1), // placeholder date
            event_type: LifecycleEventType::Sold,
            value_usd: offer.asking_price_usd,
        });

        // Record transaction
        self.transaction_history
            .push((asset_id, buyer_id, offer.asking_price_usd, soh_at_sale));

        Ok(offer.asking_price_usd)
    }

    /// Estimate the second-life market value of a battery asset \[USD\].
    ///
    /// ```text
    /// value = current_capacity_kwh × (soh_pct/100) × price_per_kwh × age_factor
    /// age_factor = max(0.1, 1.0 − calendar_age_years × 0.05)
    /// ```
    pub fn assess_second_life_value(&self, asset: &BatteryAsset) -> f64 {
        let price_per_kwh = asset.chemistry.second_life_price_per_kwh();
        let age_factor = (1.0 - asset.calendar_age_years * 0.05).max(0.1);
        asset.current_capacity_kwh * (asset.current_soh_pct / 100.0) * price_per_kwh * age_factor
    }

    /// Find listings matching an application requirement and minimum SoH.
    pub fn find_matching_offers(
        &self,
        requirement: &BatteryApplication,
        min_soh_pct: f64,
    ) -> Vec<&MarketplaceOffer> {
        self.offers
            .iter()
            .filter(|offer| {
                // Check application fit
                let fits_app = offer.application_fit.contains(requirement);
                // Check asset SoH
                let soh_ok = self
                    .assets
                    .iter()
                    .find(|a| a.id == offer.asset_id)
                    .map(|a| a.current_soh_pct >= min_soh_pct)
                    .unwrap_or(false);
                fits_app && soh_ok
            })
            .collect()
    }

    /// Total USD volume of all completed transactions.
    pub fn total_market_volume_usd(&self) -> f64 {
        self.transaction_history
            .iter()
            .map(|(_, _, price, _)| price)
            .sum()
    }

    /// Average SoH of assets sold in second-life transactions \[%\].
    ///
    /// Returns 0.0 if no transactions have occurred.
    pub fn average_second_life_soh(&self) -> f64 {
        if self.transaction_history.is_empty() {
            return 0.0;
        }
        let total_soh: f64 = self
            .transaction_history
            .iter()
            .map(|(_, _, _, soh)| soh)
            .sum();
        total_soh / self.transaction_history.len() as f64
    }

    /// Number of registered assets.
    pub fn n_assets(&self) -> usize {
        self.assets.len()
    }

    /// Number of active listings.
    pub fn n_listings(&self) -> usize {
        self.offers.len()
    }

    /// Number of completed transactions.
    pub fn n_transactions(&self) -> usize {
        self.transaction_history.len()
    }

    /// Retrieve an asset by ID (read-only).
    pub fn asset(&self, id: u64) -> Option<&BatteryAsset> {
        self.assets.iter().find(|a| a.id == id)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_nmc_asset(id: u64, soh_pct: f64, age_years: f64, owner: usize) -> BatteryAsset {
        BatteryAsset::new(
            id,
            BatteryChemistry::NmcLithiumIon,
            100.0, // 100 kWh original
            soh_pct,
            500.0, // cycle_count
            age_years,
            BatteryApplication::ElectricVehicle,
            owner,
        )
    }

    // ── Test 1: Register and list ────────────────────────────────────────────

    #[test]
    fn test_register_and_list_asset() {
        let mut mp = BatteryMarketplace::new();
        let asset = make_nmc_asset(1, 82.0, 3.0, 0);
        let id = mp.register_asset(asset);
        assert_eq!(id, 1, "returned id should match asset id");
        assert_eq!(mp.n_assets(), 1);

        mp.list_for_sale(id, 5000.0)
            .expect("list_for_sale should succeed");
        assert_eq!(mp.n_listings(), 1, "one active listing after list_for_sale");
    }

    // ── Test 2: Buy asset — ownership transfer and transaction log ───────────

    #[test]
    fn test_buy_asset_transfers_ownership() {
        let mut mp = BatteryMarketplace::new();
        let asset = make_nmc_asset(42, 78.0, 4.0, 0); // owner = 0
        mp.register_asset(asset);
        mp.list_for_sale(42, 4200.0).expect("list ok");

        let price = mp.buy_asset(42, 7).expect("buy should succeed");
        assert!(
            (price - 4200.0).abs() < 1e-9,
            "price should match asking price"
        );

        // Ownership transferred
        let a = mp.asset(42).expect("asset must still exist");
        assert_eq!(a.current_owner, 7, "owner should be buyer 7");

        // Transaction logged
        assert_eq!(mp.n_transactions(), 1);
        let (aid, buyer, tx_price, _soh) = mp.transaction_history[0];
        assert_eq!(aid, 42);
        assert_eq!(buyer, 7);
        assert!((tx_price - 4200.0).abs() < 1e-9);

        // Listing removed
        assert_eq!(mp.n_listings(), 0);
    }

    // ── Test 3: Second-life value — higher SoH = higher value ───────────────

    #[test]
    fn test_second_life_value_higher_soh_higher_value() {
        let mp = BatteryMarketplace::new();
        let asset_high = make_nmc_asset(1, 90.0, 2.0, 0);
        let asset_low = make_nmc_asset(2, 70.0, 2.0, 0);

        let val_high = mp.assess_second_life_value(&asset_high);
        let val_low = mp.assess_second_life_value(&asset_low);

        assert!(
            val_high > val_low,
            "higher SoH asset ({:.2}) should have higher value than lower SoH ({:.2})",
            val_high,
            val_low
        );
    }

    // ── Test 4: Find matching offers ─────────────────────────────────────────

    #[test]
    fn test_find_matching_offers_filters_correctly() {
        let mut mp = BatteryMarketplace::new();
        // High SoH — fits EV, GridStorage, UPS
        let asset_good = make_nmc_asset(1, 85.0, 2.0, 0);
        // Low SoH — only fits SecondLife
        let asset_poor = make_nmc_asset(2, 55.0, 8.0, 0);

        mp.register_asset(asset_good);
        mp.register_asset(asset_poor);

        mp.list_for_sale(1, 6000.0).expect("list good");
        mp.list_for_sale(2, 1000.0).expect("list poor");

        // Looking for GridStorage with SoH >= 80%
        let matches = mp.find_matching_offers(&BatteryApplication::GridStorage, 80.0);
        assert_eq!(
            matches.len(),
            1,
            "only 1 asset meets GridStorage + SoH 80% requirement"
        );
        assert_eq!(matches[0].asset_id, 1);

        // Looking for SecondLife with SoH >= 50%
        let matches2 = mp.find_matching_offers(&BatteryApplication::SecondLife, 50.0);
        // asset 2 (soh=55%) fits SecondLife; asset 1 (soh=85%) does NOT have SecondLife in fit
        assert!(
            !matches2.is_empty(),
            "at least asset_poor should match SecondLife"
        );
    }

    // ── Test 5: Market volume = sum of transactions ──────────────────────────

    #[test]
    fn test_market_volume_equals_transaction_sum() {
        let mut mp = BatteryMarketplace::new();
        // Register and trade 3 assets
        for i in 1u64..=3 {
            let asset = make_nmc_asset(i, 75.0 + i as f64, 3.0, 0);
            mp.register_asset(asset);
            let price = 1000.0 * i as f64;
            mp.list_for_sale(i, price).expect("list ok");
            mp.buy_asset(i, i as usize + 10).expect("buy ok");
        }

        let expected_volume = 1000.0 + 2000.0 + 3000.0;
        let actual_volume = mp.total_market_volume_usd();
        assert!(
            (actual_volume - expected_volume).abs() < 1e-6,
            "market volume {actual_volume:.2} != expected {expected_volume:.2}"
        );
    }

    // ── Test 6: Already listed error ─────────────────────────────────────────

    #[test]
    fn test_double_listing_returns_error() {
        let mut mp = BatteryMarketplace::new();
        let asset = make_nmc_asset(1, 80.0, 2.0, 0);
        mp.register_asset(asset);
        mp.list_for_sale(1, 5000.0).expect("first listing ok");
        let result = mp.list_for_sale(1, 4500.0);
        assert!(
            matches!(result, Err(MarketError::AlreadyListed(1))),
            "double listing should return AlreadyListed"
        );
    }

    // ── Test 7: Buy unlisted asset returns error ──────────────────────────────

    #[test]
    fn test_buy_unlisted_returns_error() {
        let mut mp = BatteryMarketplace::new();
        let asset = make_nmc_asset(5, 80.0, 2.0, 0);
        mp.register_asset(asset);
        // Not listed for sale
        let result = mp.buy_asset(5, 99);
        assert!(
            matches!(result, Err(MarketError::NotForSale(5))),
            "buying unlisted asset should return NotForSale"
        );
    }

    // ── Test 8: Average second-life SoH ──────────────────────────────────────

    #[test]
    fn test_average_second_life_soh() {
        let mut mp = BatteryMarketplace::new();
        // Two transactions: SoH 80% and 70% → average 75%
        for (id, soh) in [(1u64, 80.0_f64), (2, 70.0)] {
            let asset = make_nmc_asset(id, soh, 3.0, 0);
            mp.register_asset(asset);
            mp.list_for_sale(id, 1000.0).expect("list ok");
            mp.buy_asset(id, 10).expect("buy ok");
        }
        let avg = mp.average_second_life_soh();
        assert!(
            (avg - 75.0).abs() < 1e-6,
            "average SoH should be 75%, got {avg:.4}"
        );
    }
}
