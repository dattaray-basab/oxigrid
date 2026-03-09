//! Renewable Energy Certificate (REC) tracking and trading system.
//!
//! Implements:
//! - REC issuance, retirement, and transfer lifecycle management
//! - Compliance checking against renewable energy obligations
//! - Technology and vintage filtering
//! - Market price integration
//!
//! # Standards
//!
//! RECs conform to the 1 MWh = 1 REC standard used in the US (NERC M-RETS),
//! European GOs (Guarantees of Origin, EN 16325), and Australia (LGCs).
//!
//! # References
//! - NERC M-RETS Tracking System Operations Manual, 2023
//! - EU Directive 2018/2001 (RED II), Article 19 — Guarantees of Origin
//! - EPA Green Power Partnership, REC Accounting Guide, 2022

use serde::{Deserialize, Serialize};

// ── Error type ────────────────────────────────────────────────────────────────

/// Errors produced by the REC tracking system.
#[derive(Debug, Clone, thiserror::Error)]
pub enum RecError {
    /// The requested REC ID does not exist in the registry.
    #[error("REC {0} not found in registry")]
    NotFound(u64),

    /// The REC is not in a state that permits the requested operation.
    #[error("REC {0} has invalid status for this operation: {1}")]
    InvalidStatus(u64, String),

    /// A parameter supplied to the call is invalid.
    #[error("invalid parameter: {0}")]
    InvalidParameter(String),
}

// ── Renewable technology classification ───────────────────────────────────────

/// Renewable energy technology type for REC classification.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum RenewableTechnology {
    /// Photovoltaic or concentrating solar.
    Solar,
    /// Onshore or offshore wind.
    Wind,
    /// Run-of-river, reservoir, or tidal hydro.
    Hydro,
    /// Geothermal power.
    Geothermal,
    /// Biomass or biogas combustion.
    Biomass,
    /// Tidal or wave energy.
    TidalWave,
    /// Other or unclassified renewable technology.
    Other(String),
}

impl std::fmt::Display for RenewableTechnology {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Solar => write!(f, "Solar"),
            Self::Wind => write!(f, "Wind"),
            Self::Hydro => write!(f, "Hydro"),
            Self::Geothermal => write!(f, "Geothermal"),
            Self::Biomass => write!(f, "Biomass"),
            Self::TidalWave => write!(f, "TidalWave"),
            Self::Other(s) => write!(f, "Other({s})"),
        }
    }
}

// ── Certificate lifecycle ─────────────────────────────────────────────────────

/// REC certificate lifecycle status.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum RecStatus {
    /// Certificate is valid and can be transferred or retired.
    Active,
    /// Certificate has been surrendered against an obligation (terminal state).
    Retired,
    /// Certificate validity period has lapsed (terminal state).
    Expired,
    /// Certificate has been transferred to another party and is no longer
    /// held by the original issuing account.
    Transferred,
}

// ── Core certificate ──────────────────────────────────────────────────────────

/// A single Renewable Energy Certificate (REC / GO).
///
/// Represents 1 MWh (or `generation_mwh` MWh if fractional) of certified
/// renewable electricity generation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RecCertificate {
    /// Globally unique REC identifier assigned by the registry.
    pub rec_id: u64,

    /// Generator identifier (plant-level facility code).
    pub generator_id: usize,

    /// Renewable technology used to produce the certificate.
    pub technology: RenewableTechnology,

    /// Net generation represented by this certificate \[MWh\].
    ///
    /// Typically 1.0 under US/EU standards; can be fractional for bundles.
    pub generation_mwh: f64,

    /// Generation vintage: `(year, month)`.
    pub vintage_month: (usize, usize),

    /// Date on which the certificate was issued: `(year, month, day)`.
    pub issue_date: (usize, usize, usize),

    /// Date on which the certificate expires: `(year, month, day)`.
    ///
    /// Typically 5 years after issue in Europe, 7 in some US markets.
    pub expiry_date: (usize, usize, usize),

    /// Current lifecycle status.
    pub status: RecStatus,

    /// Grid region or balancing authority of the generating facility.
    pub region: String,
}

impl RecCertificate {
    /// Returns `true` if this certificate is currently active (can be used).
    pub fn is_active(&self) -> bool {
        self.status == RecStatus::Active
    }
}

// ── Portfolio ─────────────────────────────────────────────────────────────────

/// A collection of RECs held by one entity.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RecPortfolio {
    /// Owner identifier.
    pub owner_id: usize,

    /// All certificates in the portfolio (active, retired, or expired).
    pub certificates: Vec<RecCertificate>,
}

impl RecPortfolio {
    /// Create an empty portfolio.
    pub fn new(owner_id: usize) -> Self {
        Self {
            owner_id,
            certificates: Vec::new(),
        }
    }

    /// Total active MWh represented by active certificates.
    pub fn active_mwh(&self) -> f64 {
        self.certificates
            .iter()
            .filter(|c| c.is_active())
            .map(|c| c.generation_mwh)
            .sum()
    }
}

// ── Market data ───────────────────────────────────────────────────────────────

/// Spot and forward market prices for RECs.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RecMarket {
    /// Spot price \[USD/REC\].
    pub spot_price_usd_per_rec: f64,

    /// Forward curve: `(year, price_USD/REC)`.
    pub forward_prices: Vec<(usize, f64)>,

    /// Market liquidity score \[0–1\] (1 = highly liquid).
    pub liquidity_score: f64,
}

// ── Compliance check ──────────────────────────────────────────────────────────

/// Result of a REC compliance check against a renewable energy obligation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RecComplianceCheck {
    /// Required renewable electricity \[MWh\] (the obligation).
    pub obligation_mwh: f64,

    /// Renewable MWh covered by active RECs in the portfolio \[MWh\].
    pub fulfilled_mwh: f64,

    /// Shortfall: max(0, obligation − fulfilled) \[MWh\].
    pub deficit_mwh: f64,

    /// Excess: max(0, fulfilled − obligation) \[MWh\].
    pub surplus_mwh: f64,

    /// Compliance ratio as a percentage: `(fulfilled / obligation) * 100`.
    pub compliance_pct: f64,

    /// Non-compliance penalty \[USD\] = deficit × penalty_rate.
    pub penalty_usd: f64,

    /// Estimated cost of RECs already purchased \[USD\].
    ///
    /// Computed as `fulfilled_mwh * spot_price` if provided, else 0.
    pub rec_cost_usd: f64,
}

// ── Tracker ───────────────────────────────────────────────────────────────────

/// Configuration for the REC registry.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RecConfig {
    /// Registry name (e.g., "M-RETS", "GREXEL", "REGO").
    pub registry_name: String,

    /// Jurisdiction (e.g., "US-MISO", "EU", "AU").
    pub jurisdiction: String,

    /// Default vintage year for issued certificates.
    pub vintage_year: usize,

    /// MWh represented by one REC issuance unit (always 1.0 in US/EU).
    pub rec_megawatt_hours: f64,
}

impl Default for RecConfig {
    fn default() -> Self {
        Self {
            registry_name: "M-RETS".into(),
            jurisdiction: "US".into(),
            vintage_year: 2026,
            rec_megawatt_hours: 1.0,
        }
    }
}

/// REC registry and tracker.
///
/// Manages the full lifecycle of RECs: issuance, retirement, transfer,
/// compliance checking, and portfolio queries.
pub struct RecTracker {
    config: RecConfig,
    registry: Vec<RecCertificate>,
    next_id: u64,
}

impl RecTracker {
    /// Create a new, empty REC tracker.
    pub fn new(config: RecConfig) -> Self {
        Self {
            config,
            registry: Vec::new(),
            next_id: 1,
        }
    }

    /// Issue a new REC for a generator.
    ///
    /// # Arguments
    ///
    /// - `generator_id` — facility identifier.
    /// - `tech`         — renewable technology type.
    /// - `mwh`          — generation MWh this certificate represents.
    /// - `region`       — grid region of the generator.
    ///
    /// Returns the newly-created certificate (also stored in the registry).
    pub fn issue_rec(
        &mut self,
        generator_id: usize,
        tech: RenewableTechnology,
        mwh: f64,
        region: String,
    ) -> RecCertificate {
        let id = self.next_id;
        self.next_id += 1;
        let year = self.config.vintage_year;
        let cert = RecCertificate {
            rec_id: id,
            generator_id,
            technology: tech,
            generation_mwh: mwh,
            vintage_month: (year, 1),
            issue_date: (year, 1, 1),
            expiry_date: (year + 5, 1, 1),
            status: RecStatus::Active,
            region,
        };
        self.registry.push(cert.clone());
        cert
    }

    /// Retire a REC (surrender against an obligation).
    ///
    /// Only `Active` certificates can be retired.
    pub fn retire_rec(&mut self, rec_id: u64) -> Result<(), RecError> {
        let cert = self
            .registry
            .iter_mut()
            .find(|c| c.rec_id == rec_id)
            .ok_or(RecError::NotFound(rec_id))?;

        if cert.status != RecStatus::Active {
            return Err(RecError::InvalidStatus(
                rec_id,
                format!("cannot retire {:?} certificate", cert.status),
            ));
        }
        cert.status = RecStatus::Retired;
        Ok(())
    }

    /// Transfer a REC to a new owner.
    ///
    /// Only `Active` certificates can be transferred. The certificate status
    /// is set to `Transferred` in the registry; the new owner should issue
    /// a corresponding certificate in their own registry.
    pub fn transfer_rec(&mut self, rec_id: u64, _new_owner: usize) -> Result<(), RecError> {
        let cert = self
            .registry
            .iter_mut()
            .find(|c| c.rec_id == rec_id)
            .ok_or(RecError::NotFound(rec_id))?;

        if cert.status != RecStatus::Active {
            return Err(RecError::InvalidStatus(
                rec_id,
                format!("cannot transfer {:?} certificate", cert.status),
            ));
        }
        cert.status = RecStatus::Transferred;
        Ok(())
    }

    /// Check compliance of a portfolio against a renewable energy obligation.
    ///
    /// # Arguments
    ///
    /// - `portfolio`              — the entity's REC portfolio.
    /// - `obligation_mwh`         — required renewable MWh.
    /// - `penalty_rate_usd_per_mwh` — $/MWh penalty for non-compliance.
    pub fn check_compliance(
        &self,
        portfolio: &RecPortfolio,
        obligation_mwh: f64,
        penalty_rate_usd_per_mwh: f64,
    ) -> RecComplianceCheck {
        let fulfilled_mwh: f64 = portfolio
            .certificates
            .iter()
            .filter(|c| c.is_active())
            .map(|c| c.generation_mwh)
            .sum();

        let deficit_mwh = (obligation_mwh - fulfilled_mwh).max(0.0);
        let surplus_mwh = (fulfilled_mwh - obligation_mwh).max(0.0);
        let compliance_pct = if obligation_mwh > 0.0 {
            (fulfilled_mwh / obligation_mwh * 100.0).min(100.0)
        } else {
            100.0
        };
        let penalty_usd = deficit_mwh * penalty_rate_usd_per_mwh;

        RecComplianceCheck {
            obligation_mwh,
            fulfilled_mwh,
            deficit_mwh,
            surplus_mwh,
            compliance_pct,
            penalty_usd,
            rec_cost_usd: 0.0, // populated by caller if spot price known
        }
    }

    /// Return references to all `Active` certificates in the registry.
    pub fn get_active_recs(&self) -> Vec<&RecCertificate> {
        self.registry.iter().filter(|c| c.is_active()).collect()
    }

    /// Total MWh represented by all active certificates in the registry.
    pub fn total_active_mwh(&self) -> f64 {
        self.registry
            .iter()
            .filter(|c| c.is_active())
            .map(|c| c.generation_mwh)
            .sum()
    }

    /// Filter registry certificates by technology type.
    ///
    /// Compares by `PartialEq` — note that `Other(s)` variants match only if
    /// the inner strings are identical.
    pub fn filter_by_technology(&self, tech: &RenewableTechnology) -> Vec<&RecCertificate> {
        self.registry
            .iter()
            .filter(|c| &c.technology == tech)
            .collect()
    }

    /// Look up a certificate by ID (returns `None` if not found).
    pub fn get_by_id(&self, rec_id: u64) -> Option<&RecCertificate> {
        self.registry.iter().find(|c| c.rec_id == rec_id)
    }

    /// Total number of certificates in the registry (all statuses).
    pub fn registry_size(&self) -> usize {
        self.registry.len()
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_tracker() -> RecTracker {
        RecTracker::new(RecConfig::default())
    }

    /// Issue a REC and verify it appears active in the registry.
    #[test]
    fn test_issue_rec_active() {
        let mut tracker = make_tracker();
        let cert = tracker.issue_rec(1, RenewableTechnology::Solar, 1.0, "US-MISO".into());
        assert_eq!(cert.status, RecStatus::Active);
        assert_eq!(cert.generator_id, 1);
        assert_eq!(tracker.get_active_recs().len(), 1);
        assert!((tracker.total_active_mwh() - 1.0).abs() < 1e-12);
    }

    /// Retire a REC: status transitions to Retired; no longer active.
    #[test]
    fn test_retire_rec_status_transitions() {
        let mut tracker = make_tracker();
        let cert = tracker.issue_rec(1, RenewableTechnology::Wind, 1.0, "EU".into());
        tracker.retire_rec(cert.rec_id).unwrap();

        let stored = tracker.get_by_id(cert.rec_id).unwrap();
        assert_eq!(stored.status, RecStatus::Retired);
        assert!(tracker.get_active_recs().is_empty());
    }

    /// Retiring an already-retired REC must return InvalidStatus error.
    #[test]
    fn test_double_retire_returns_error() {
        let mut tracker = make_tracker();
        let cert = tracker.issue_rec(2, RenewableTechnology::Hydro, 5.0, "AU".into());
        tracker.retire_rec(cert.rec_id).unwrap();
        let err = tracker.retire_rec(cert.rec_id);
        assert!(err.is_err(), "Second retire should fail");
    }

    /// Compliance check with deficit: penalty computed correctly.
    #[test]
    fn test_compliance_deficit_penalty() {
        let mut tracker = make_tracker();
        let cert = tracker.issue_rec(1, RenewableTechnology::Solar, 80.0, "US".into());

        let mut portfolio = RecPortfolio::new(100);
        portfolio.certificates.push(cert);

        let check = tracker.check_compliance(&portfolio, 100.0, 50.0);
        assert!((check.deficit_mwh - 20.0).abs() < 1e-9);
        assert!((check.surplus_mwh).abs() < 1e-9);
        assert!((check.penalty_usd - 1000.0).abs() < 1e-6);
        assert!((check.compliance_pct - 80.0).abs() < 1e-6);
    }

    /// Compliance check with surplus: no penalty.
    #[test]
    fn test_compliance_surplus_no_penalty() {
        let mut tracker = make_tracker();
        let cert = tracker.issue_rec(1, RenewableTechnology::Wind, 120.0, "EU".into());

        let mut portfolio = RecPortfolio::new(200);
        portfolio.certificates.push(cert);

        let check = tracker.check_compliance(&portfolio, 100.0, 50.0);
        assert!((check.surplus_mwh - 20.0).abs() < 1e-9);
        assert!((check.deficit_mwh).abs() < 1e-9);
        assert!((check.penalty_usd).abs() < 1e-12);
        assert!((check.compliance_pct - 100.0).abs() < 1e-6);
    }

    /// Technology filtering: only solar RECs returned.
    #[test]
    fn test_filter_by_technology_solar() {
        let mut tracker = make_tracker();
        tracker.issue_rec(1, RenewableTechnology::Solar, 1.0, "US".into());
        tracker.issue_rec(2, RenewableTechnology::Wind, 1.0, "US".into());
        tracker.issue_rec(3, RenewableTechnology::Solar, 2.0, "US".into());

        let solar = tracker.filter_by_technology(&RenewableTechnology::Solar);
        assert_eq!(solar.len(), 2, "Should find 2 solar RECs");
        for c in &solar {
            assert_eq!(c.technology, RenewableTechnology::Solar);
        }
    }

    /// Transfer: certificate status changes to Transferred.
    #[test]
    fn test_transfer_changes_status() {
        let mut tracker = make_tracker();
        let cert = tracker.issue_rec(1, RenewableTechnology::Geothermal, 1.0, "US".into());
        tracker.transfer_rec(cert.rec_id, 999).unwrap();

        let stored = tracker.get_by_id(cert.rec_id).unwrap();
        assert_eq!(stored.status, RecStatus::Transferred);
        // No longer active
        assert!(tracker.get_active_recs().is_empty());
    }

    /// Retiring a Transferred REC should return error.
    #[test]
    fn test_cannot_retire_transferred_rec() {
        let mut tracker = make_tracker();
        let cert = tracker.issue_rec(1, RenewableTechnology::Biomass, 1.0, "US".into());
        tracker.transfer_rec(cert.rec_id, 99).unwrap();
        assert!(tracker.retire_rec(cert.rec_id).is_err());
    }
}
