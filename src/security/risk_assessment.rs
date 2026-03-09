//! Grid Cybersecurity Risk Assessment.
//!
//! This module provides a comprehensive risk assessment framework for power grid
//! cyber assets based on:
//!
//! 1. **Inherent risk** = asset criticality × network exposure × (1 − patch level)
//! 2. **Residual risk** = inherent risk × Π(1 − control effectiveness)
//! 3. **Annual expected loss** = threat likelihood × impact × operational cost
//! 4. **Monte Carlo quantification** — sample attack scenarios (LCG RNG)
//! 5. **Control recommendations** — ranked by cost-effectiveness
//!
//! # References
//!
//! - NERC CIP-002 through CIP-014 standards
//! - IEC 62443 (Industrial Automation and Control Systems Security)
//! - NIST Cybersecurity Framework (CSF) v2.0
//! - DOE Cybersecurity Capability Maturity Model (C2M2)

use thiserror::Error;

/// Errors from the risk assessor.
#[derive(Debug, Error)]
pub enum RiskError {
    #[error("No assets defined")]
    NoAssets,
    #[error("No threats defined")]
    NoThreats,
    #[error("Monte Carlo count must be positive")]
    InvalidMonteCarloN,
    #[error("Invalid risk appetite: {0} (must be 0–1)")]
    InvalidRiskAppetite(f64),
}

// ── Configuration ─────────────────────────────────────────────────────────────

/// Overall threat landscape characterisation.
#[derive(Debug, Clone, PartialEq)]
pub enum ThreatLandscape {
    /// Advanced Persistent Threat — nation-state level.
    NationState,
    /// Financially motivated ransomware / extortion.
    Cybercriminal,
    /// Ideologically motivated disruption.
    Hacktivist,
    /// Trusted insider with malicious intent.
    Insider,
    /// Unintentional misconfiguration or human error.
    AccidentalError,
    /// Combined natural disaster plus cyber exploitation.
    Natural,
}

impl ThreatLandscape {
    /// Scaling factor on likelihood for this threat landscape.
    fn likelihood_multiplier(&self) -> f64 {
        match self {
            ThreatLandscape::NationState => 2.0,
            ThreatLandscape::Cybercriminal => 1.5,
            ThreatLandscape::Hacktivist => 1.0,
            ThreatLandscape::Insider => 1.2,
            ThreatLandscape::AccidentalError => 0.8,
            ThreatLandscape::Natural => 0.5,
        }
    }
}

/// Configuration for a cybersecurity risk assessment.
#[derive(Debug, Clone)]
pub struct CyberRiskConfig {
    /// Characterisation of the threat landscape.
    pub threat_landscape: ThreatLandscape,
    /// Weight on asset criticality vs attack likelihood in risk score \[0–1\].
    pub asset_criticality_weight: f64,
    /// Number of Monte Carlo scenarios for risk quantification.
    pub monte_carlo_n: usize,
    /// Acceptable risk level \[0–1\].
    pub risk_appetite: f64,
}

impl Default for CyberRiskConfig {
    fn default() -> Self {
        Self {
            threat_landscape: ThreatLandscape::Cybercriminal,
            asset_criticality_weight: 0.6,
            monte_carlo_n: 10_000,
            risk_appetite: 0.2,
        }
    }
}

// ── Asset types ───────────────────────────────────────────────────────────────

/// Type of cyber asset.
#[derive(Debug, Clone, PartialEq)]
pub enum CyberAssetType {
    Ems,
    Scada,
    Rtu,
    Ied,
    CorporateIt,
    EngineeringWs,
    Historian,
    Firewall,
    ControlCenter,
}

impl CyberAssetType {
    /// Base vulnerability factor for this asset type.
    fn base_vulnerability(&self) -> f64 {
        match self {
            CyberAssetType::Ems => 0.75,
            CyberAssetType::Scada => 0.70,
            CyberAssetType::Rtu => 0.65,
            CyberAssetType::Ied => 0.60,
            CyberAssetType::CorporateIt => 0.80,
            CyberAssetType::EngineeringWs => 0.70,
            CyberAssetType::Historian => 0.55,
            CyberAssetType::Firewall => 0.30,
            CyberAssetType::ControlCenter => 0.80,
        }
    }
}

/// An individual security control applied to an asset.
#[derive(Debug, Clone)]
pub struct SecurityControl {
    /// Control name (e.g. "Multi-factor Authentication").
    pub name: String,
    /// Risk reduction effectiveness \[0–1\].
    pub effectiveness: f64,
    /// CMMI maturity level \[0–5\].
    pub maturity: f64,
}

/// A critical cyber asset in the grid.
#[derive(Debug, Clone)]
pub struct CriticalAsset {
    /// Unique identifier.
    pub id: usize,
    /// Human-readable name.
    pub name: String,
    /// Asset type.
    pub asset_type: CyberAssetType,
    /// Criticality of the asset \[0–1\].
    pub criticality: f64,
    /// Network exposure (0 = air-gapped, 1 = internet-facing) \[0–1\].
    pub network_exposure: f64,
    /// Patch level (0 = unpatched, 1 = fully patched) \[0–1\].
    pub patch_level: f64,
    /// Security controls currently in place.
    pub security_controls: Vec<SecurityControl>,
    /// Operational impact if this asset is successfully compromised \[MW\].
    pub operational_impact_mw: f64,
}

// ── Threat vectors ────────────────────────────────────────────────────────────

/// Attack category.
#[derive(Debug, Clone, PartialEq)]
pub enum AttackType {
    NetworkIntrusion,
    Phishing,
    InsiderThreat,
    PhysicalAccess,
    SupplyChain,
    ZeroDay,
    DenialOfService,
    ManInTheMiddle,
}

impl AttackType {
    /// Sophistication required for this attack \[0–1\].
    fn sophistication(&self) -> f64 {
        match self {
            AttackType::ZeroDay => 0.95,
            AttackType::SupplyChain => 0.85,
            AttackType::NetworkIntrusion => 0.60,
            AttackType::ManInTheMiddle => 0.55,
            AttackType::PhysicalAccess => 0.40,
            AttackType::InsiderThreat => 0.30,
            AttackType::Phishing => 0.20,
            AttackType::DenialOfService => 0.15,
        }
    }
}

/// A threat vector that can be applied against assets.
#[derive(Debug, Clone)]
pub struct ThreatVector {
    /// Human-readable threat name.
    pub name: String,
    /// Attack category.
    pub attack_type: AttackType,
    /// Annualised attack rate (events per year).
    pub likelihood_per_year: f64,
    /// Required attacker sophistication \[0–1\].
    pub sophistication_required: f64,
}

// ── Output types ──────────────────────────────────────────────────────────────

/// Risk rating classification.
#[derive(Debug, Clone, PartialEq)]
pub enum RiskRating {
    VeryLow,
    Low,
    Medium,
    High,
    Critical,
}

impl RiskRating {
    fn from_score(score: f64) -> Self {
        if score < 0.10 {
            RiskRating::VeryLow
        } else if score < 0.25 {
            RiskRating::Low
        } else if score < 0.50 {
            RiskRating::Medium
        } else if score < 0.75 {
            RiskRating::High
        } else {
            RiskRating::Critical
        }
    }
}

/// Per-asset risk breakdown.
#[derive(Debug, Clone)]
pub struct AssetRisk {
    /// Asset identifier.
    pub asset_id: usize,
    /// Inherent risk before controls \[0–1\].
    pub inherent_risk: f64,
    /// Residual risk after existing controls \[0–1\].
    pub residual_risk: f64,
    /// Annualised attack likelihood \[events/year\].
    pub likelihood: f64,
    /// Impact if compromised \[0–1\].
    pub impact: f64,
    /// Overall risk rating.
    pub risk_rating: RiskRating,
}

/// A recommended additional security control.
#[derive(Debug, Clone)]
pub struct RecommendedControl {
    /// Control name.
    pub name: String,
    /// Asset IDs this control should be applied to.
    pub affected_assets: Vec<usize>,
    /// Expected risk reduction \[0–1\].
    pub risk_reduction: f64,
    /// Estimated implementation cost \[USD\].
    pub implementation_cost_usd: f64,
    /// Cost-effectiveness = risk_reduction / cost.
    pub cost_effectiveness: f64,
    /// Priority: 1 = highest.
    pub priority: u8,
}

/// Full result of a cybersecurity risk assessment.
#[derive(Debug, Clone)]
pub struct CyberRiskResult {
    /// Per-asset risk breakdown.
    pub asset_risks: Vec<AssetRisk>,
    /// Aggregate risk score \[0–1\].
    pub total_risk_score: f64,
    /// Annual expected monetary loss \[USD\].
    pub annual_expected_loss_usd: f64,
    /// Probability of at least one incident per year \[0–1\].
    pub probability_of_incident_per_year: f64,
    /// Top contributing threats `(name, risk contribution)`.
    pub top_threats: Vec<(String, f64)>,
    /// Recommended additional controls.
    pub recommended_controls: Vec<RecommendedControl>,
    /// 5×5 risk map indexed by likelihood band × impact band.
    pub risk_map: Vec<Vec<f64>>,
    /// Residual risk after recommended controls are applied.
    pub residual_risk: f64,
}

// ── Assessor ──────────────────────────────────────────────────────────────────

/// Cyber risk assessor for power grid assets.
pub struct CyberRiskAssessor {
    config: CyberRiskConfig,
    assets: Vec<CriticalAsset>,
    threats: Vec<ThreatVector>,
}

impl CyberRiskAssessor {
    /// Create a new assessor with the given configuration.
    pub fn new(config: CyberRiskConfig) -> Self {
        Self {
            config,
            assets: Vec::new(),
            threats: Vec::new(),
        }
    }

    /// Add a critical asset.  Returns the asset index.
    pub fn add_asset(&mut self, asset: CriticalAsset) -> usize {
        let idx = self.assets.len();
        self.assets.push(asset);
        idx
    }

    /// Add a threat vector.
    pub fn add_threat(&mut self, threat: ThreatVector) {
        self.threats.push(threat);
    }

    /// Run the full risk assessment.
    pub fn assess(&self) -> Result<CyberRiskResult, RiskError> {
        if self.assets.is_empty() {
            return Err(RiskError::NoAssets);
        }
        if self.threats.is_empty() {
            return Err(RiskError::NoThreats);
        }
        if self.config.monte_carlo_n == 0 {
            return Err(RiskError::InvalidMonteCarloN);
        }
        if !(0.0..=1.0).contains(&self.config.risk_appetite) {
            return Err(RiskError::InvalidRiskAppetite(self.config.risk_appetite));
        }

        // Step 1 & 2: Compute inherent and residual risk per asset
        let asset_risks: Vec<AssetRisk> = self
            .assets
            .iter()
            .map(|asset| {
                let inherent = self.inherent_risk(asset);
                let residual = self.apply_controls(inherent, &asset.security_controls);
                let likelihood = self.total_threat_likelihood(asset);
                let impact = asset.criticality.clamp(0.0, 1.0);
                let risk_rating = RiskRating::from_score(residual);
                AssetRisk {
                    asset_id: asset.id,
                    inherent_risk: inherent,
                    residual_risk: residual,
                    likelihood,
                    impact,
                    risk_rating,
                }
            })
            .collect();

        // Step 3: Annual expected loss
        let cost_per_mw_usd = 50_000.0_f64; // heuristic: $50k per MW per incident
        let annual_expected_loss_usd: f64 = self
            .assets
            .iter()
            .zip(asset_risks.iter())
            .map(|(asset, ar)| {
                ar.likelihood * ar.impact * asset.operational_impact_mw * cost_per_mw_usd
            })
            .sum();

        // Step 4: Monte Carlo
        let prob_incident = self.monte_carlo_probability(&asset_risks);

        // Aggregate risk score — weighted average of residual risks
        let total_criticality: f64 = self
            .assets
            .iter()
            .map(|a| a.criticality)
            .sum::<f64>()
            .max(1e-9);
        let total_risk_score = self
            .assets
            .iter()
            .zip(asset_risks.iter())
            .map(|(a, ar)| ar.residual_risk * a.criticality)
            .sum::<f64>()
            / total_criticality;

        // Top threats by contribution
        let mut threat_contributions: Vec<(String, f64)> = self
            .threats
            .iter()
            .map(|t| {
                let contrib = self.threat_contribution(t);
                (t.name.clone(), contrib)
            })
            .collect();
        threat_contributions
            .sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        threat_contributions.truncate(5);

        // Step 5: Recommendations
        let recommendations = self.recommend_controls(&asset_risks);
        let residual_risk = self.residual_after_recommendations(total_risk_score, &recommendations);

        // Risk map (5×5)
        let risk_map = self.build_risk_map(&asset_risks);

        Ok(CyberRiskResult {
            asset_risks,
            total_risk_score,
            annual_expected_loss_usd,
            probability_of_incident_per_year: prob_incident,
            top_threats: threat_contributions,
            recommended_controls: recommendations,
            risk_map,
            residual_risk,
        })
    }

    // ── Risk computation ──────────────────────────────────────────────────────

    fn inherent_risk(&self, asset: &CriticalAsset) -> f64 {
        let base_vuln = asset.asset_type.base_vulnerability();
        let patch_penalty = 1.0 - asset.patch_level;
        let exposure = asset.network_exposure;
        let criticality = asset.criticality;
        // Inherent risk = criticality × exposure × (1 − patch_level) × base_vulnerability
        (criticality * exposure * patch_penalty * base_vuln).clamp(0.0, 1.0)
    }

    /// Apply security controls: residual = inherent × Π(1 − effectiveness_i).
    pub fn apply_controls(&self, inherent_risk: f64, controls: &[SecurityControl]) -> f64 {
        let reduction_factor = controls.iter().fold(1.0_f64, |acc, ctrl| {
            acc * (1.0 - ctrl.effectiveness.clamp(0.0, 1.0))
        });
        (inherent_risk * reduction_factor).clamp(0.0, 1.0)
    }

    fn total_threat_likelihood(&self, asset: &CriticalAsset) -> f64 {
        let landscape_mult = self.config.threat_landscape.likelihood_multiplier();
        self.threats
            .iter()
            .map(|t| self.threat_likelihood_for_asset(asset, t) * landscape_mult)
            .sum::<f64>()
            .clamp(0.0, 100.0)
    }

    /// Compute annualised attack likelihood for a specific asset–threat pair.
    pub fn threat_likelihood_for_asset(&self, asset: &CriticalAsset, threat: &ThreatVector) -> f64 {
        // An attacker can execute the threat if their sophistication ≥ required sophistication
        // We model probability of attacker meeting required sophistication using exposure
        let attack_soph = threat.attack_type.sophistication();
        let attacker_capability = asset.network_exposure; // proxy: internet-facing → broad attacker pool
        let prob_capable = if attacker_capability >= attack_soph {
            1.0
        } else {
            (attacker_capability / attack_soph).powi(2)
        };
        threat.likelihood_per_year * prob_capable * asset.criticality
    }

    fn threat_contribution(&self, threat: &ThreatVector) -> f64 {
        let landscape_mult = self.config.threat_landscape.likelihood_multiplier();
        self.assets
            .iter()
            .map(|a| self.threat_likelihood_for_asset(a, threat) * a.criticality)
            .sum::<f64>()
            * landscape_mult
    }

    // ── Monte Carlo ───────────────────────────────────────────────────────────

    fn monte_carlo_probability(&self, asset_risks: &[AssetRisk]) -> f64 {
        // LCG parameters per project policy
        const MULT: u64 = 6364136223846793005;
        const ADD: u64 = 1442695040888963407;

        let n = self.config.monte_carlo_n;
        let mut state: u64 = 0xDEAD_BEEF_CAFE_BABEu64;
        let mut incident_count = 0u64;

        for _ in 0..n {
            let mut incident = false;
            for ar in asset_risks {
                state = state.wrapping_mul(MULT).wrapping_add(ADD);
                let u = (state >> 11) as f64 / (1u64 << 53) as f64;
                // Per-year incident probability ≈ 1 − exp(−λ) where λ = likelihood × residual_risk
                let prob = 1.0 - (-(ar.likelihood * ar.residual_risk)).exp();
                if u < prob {
                    incident = true;
                    break;
                }
            }
            if incident {
                incident_count += 1;
            }
        }

        incident_count as f64 / n as f64
    }

    // ── Recommendations ───────────────────────────────────────────────────────

    fn recommend_controls(&self, asset_risks: &[AssetRisk]) -> Vec<RecommendedControl> {
        let mut recommendations = Vec::new();

        // Identify high-risk assets
        let high_risk_assets: Vec<usize> = asset_risks
            .iter()
            .filter(|ar| ar.residual_risk > self.config.risk_appetite)
            .map(|ar| ar.asset_id)
            .collect();

        if high_risk_assets.is_empty() {
            return recommendations;
        }

        // Recommended controls library
        let control_templates: &[(&str, f64, f64, u8)] = &[
            ("Network Segmentation / DMZ", 0.40, 50_000.0, 1),
            ("Multi-Factor Authentication", 0.35, 15_000.0, 1),
            ("Endpoint Detection and Response (EDR)", 0.30, 25_000.0, 2),
            ("Patch Management Program", 0.25, 10_000.0, 2),
            ("Privileged Access Management (PAM)", 0.30, 30_000.0, 2),
            (
                "Security Information and Event Management (SIEM)",
                0.20,
                80_000.0,
                3,
            ),
            ("Zero-Trust Architecture", 0.45, 200_000.0, 3),
            ("OT/IT Network Monitoring", 0.35, 60_000.0, 2),
        ];

        for (priority_idx, &(name, effectiveness, cost, priority)) in
            control_templates.iter().enumerate()
        {
            if priority_idx >= 5 {
                break; // Limit to 5 recommendations
            }
            let cost_eff = if cost > 0.0 {
                effectiveness / cost
            } else {
                0.0
            };
            recommendations.push(RecommendedControl {
                name: name.to_string(),
                affected_assets: high_risk_assets.clone(),
                risk_reduction: effectiveness,
                implementation_cost_usd: cost,
                cost_effectiveness: cost_eff,
                priority,
            });
        }

        // Sort by cost-effectiveness (higher = better)
        recommendations.sort_by(|a, b| {
            b.cost_effectiveness
                .partial_cmp(&a.cost_effectiveness)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        // Re-assign priority based on sorted order
        for (i, rec) in recommendations.iter_mut().enumerate() {
            rec.priority = (i + 1) as u8;
        }

        recommendations
    }

    fn residual_after_recommendations(
        &self,
        current_risk: f64,
        recs: &[RecommendedControl],
    ) -> f64 {
        recs.iter().take(3).fold(current_risk, |risk, rec| {
            (risk * (1.0 - rec.risk_reduction)).max(0.0)
        })
    }

    fn build_risk_map(&self, asset_risks: &[AssetRisk]) -> Vec<Vec<f64>> {
        // 5×5 map: rows = likelihood band (0–4, low to high), cols = impact band (0–4)
        let mut map = vec![vec![0.0_f64; 5]; 5];
        for ar in asset_risks {
            let lhood_band = (ar.likelihood.min(4.99) as usize).min(4);
            let impact_band = ((ar.impact * 4.99) as usize).min(4);
            map[lhood_band][impact_band] += ar.residual_risk;
        }
        map
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_config() -> CyberRiskConfig {
        CyberRiskConfig {
            threat_landscape: ThreatLandscape::Cybercriminal,
            asset_criticality_weight: 0.6,
            monte_carlo_n: 1_000, // fast for tests
            risk_appetite: 0.2,
        }
    }

    fn basic_threat() -> ThreatVector {
        ThreatVector {
            name: "Phishing".into(),
            attack_type: AttackType::Phishing,
            likelihood_per_year: 2.0,
            sophistication_required: 0.2,
        }
    }

    fn well_secured_asset() -> CriticalAsset {
        CriticalAsset {
            id: 1,
            name: "Hardened EMS".into(),
            asset_type: CyberAssetType::Ems,
            criticality: 0.8,
            network_exposure: 0.05, // almost air-gapped
            patch_level: 0.99,      // fully patched
            security_controls: vec![
                SecurityControl {
                    name: "MFA".into(),
                    effectiveness: 0.80,
                    maturity: 4.0,
                },
                SecurityControl {
                    name: "SIEM".into(),
                    effectiveness: 0.50,
                    maturity: 3.0,
                },
            ],
            operational_impact_mw: 200.0,
        }
    }

    fn vulnerable_asset() -> CriticalAsset {
        CriticalAsset {
            id: 2,
            name: "Unpatched RTU".into(),
            asset_type: CyberAssetType::Rtu,
            criticality: 0.9,
            network_exposure: 1.0, // internet-facing
            patch_level: 0.0,      // completely unpatched
            security_controls: vec![],
            operational_impact_mw: 150.0,
        }
    }

    // Test 1: Well-secured asset has low residual risk
    #[test]
    fn test_well_secured_asset_low_residual_risk() {
        let mut assessor = CyberRiskAssessor::new(make_config());
        assessor.add_asset(well_secured_asset());
        assessor.add_threat(basic_threat());

        let result = assessor.assess().expect("assessment ok");
        let ar = &result.asset_risks[0];
        assert!(
            ar.residual_risk < 0.30,
            "Well-secured asset should have low residual risk: {:.4}",
            ar.residual_risk
        );
        assert!(
            ar.inherent_risk > ar.residual_risk,
            "Residual risk must be less than inherent risk after controls"
        );
    }

    // Test 2: Unpatched internet-facing asset has high residual risk
    #[test]
    fn test_unpatched_internet_facing_high_risk() {
        let mut assessor = CyberRiskAssessor::new(make_config());
        assessor.add_asset(vulnerable_asset());
        assessor.add_threat(basic_threat());

        let result = assessor.assess().expect("assessment ok");
        let ar = &result.asset_risks[0];
        assert!(
            ar.inherent_risk > 0.40,
            "Unpatched internet-facing asset should have high inherent risk: {:.4}",
            ar.inherent_risk
        );
        // No controls → residual ≈ inherent
        assert!(
            ar.residual_risk > 0.30,
            "Without controls, residual risk should remain high: {:.4}",
            ar.residual_risk
        );
    }

    // Test 3: Annual expected loss computed from likelihood × impact
    #[test]
    fn test_annual_loss_computed() {
        let mut assessor = CyberRiskAssessor::new(make_config());
        assessor.add_asset(CriticalAsset {
            id: 10,
            name: "Control Center".into(),
            asset_type: CyberAssetType::ControlCenter,
            criticality: 0.8,
            network_exposure: 0.5,
            patch_level: 0.5,
            security_controls: vec![],
            operational_impact_mw: 500.0,
        });
        assessor.add_threat(ThreatVector {
            name: "Network Intrusion".into(),
            attack_type: AttackType::NetworkIntrusion,
            likelihood_per_year: 1.0,
            sophistication_required: 0.6,
        });

        let result = assessor.assess().expect("assessment ok");
        assert!(
            result.annual_expected_loss_usd > 0.0,
            "Annual expected loss must be positive: {:.2}",
            result.annual_expected_loss_usd
        );
    }

    // Test 4: Recommendations improve highest-risk assets
    #[test]
    fn test_recommendations_target_high_risk_assets() {
        let config = CyberRiskConfig {
            risk_appetite: 0.05, // very low tolerance → many assets trigger recommendations
            ..make_config()
        };
        let mut assessor = CyberRiskAssessor::new(config);
        assessor.add_asset(vulnerable_asset());
        assessor.add_threat(ThreatVector {
            name: "Zero-Day".into(),
            attack_type: AttackType::ZeroDay,
            likelihood_per_year: 0.5,
            sophistication_required: 0.95,
        });

        let result = assessor.assess().expect("assessment ok");
        assert!(
            !result.recommended_controls.is_empty(),
            "High-risk assets should generate recommendations"
        );
        // First recommendation should have the best cost-effectiveness (sorted)
        if result.recommended_controls.len() >= 2 {
            assert!(
                result.recommended_controls[0].cost_effectiveness
                    >= result.recommended_controls[1].cost_effectiveness - 1e-9,
                "Recommendations should be sorted by cost-effectiveness"
            );
        }
    }

    // Test 5: Monte Carlo computes P(incident) in [0, 1]
    #[test]
    fn test_monte_carlo_probability_in_range() {
        let mut assessor = CyberRiskAssessor::new(make_config());
        assessor.add_asset(vulnerable_asset());
        assessor.add_asset(well_secured_asset());
        assessor.add_threat(basic_threat());
        assessor.add_threat(ThreatVector {
            name: "Insider Threat".into(),
            attack_type: AttackType::InsiderThreat,
            likelihood_per_year: 0.3,
            sophistication_required: 0.3,
        });

        let result = assessor.assess().expect("assessment ok");
        assert!(
            result.probability_of_incident_per_year >= 0.0
                && result.probability_of_incident_per_year <= 1.0,
            "P(incident) must be in [0,1]: {:.4}",
            result.probability_of_incident_per_year
        );
    }

    // Test 6: apply_controls reduces risk monotonically with more controls
    #[test]
    fn test_apply_controls_reduces_risk() {
        let assessor = CyberRiskAssessor::new(make_config());
        let inherent = 0.8;
        let no_controls: Vec<SecurityControl> = vec![];
        let one_control = vec![SecurityControl {
            name: "Firewall".into(),
            effectiveness: 0.50,
            maturity: 3.0,
        }];
        let two_controls = vec![
            SecurityControl {
                name: "Firewall".into(),
                effectiveness: 0.50,
                maturity: 3.0,
            },
            SecurityControl {
                name: "MFA".into(),
                effectiveness: 0.40,
                maturity: 4.0,
            },
        ];

        let r0 = assessor.apply_controls(inherent, &no_controls);
        let r1 = assessor.apply_controls(inherent, &one_control);
        let r2 = assessor.apply_controls(inherent, &two_controls);

        assert!((r0 - inherent).abs() < 1e-9, "No controls: risk unchanged");
        assert!(r1 < r0, "One control reduces risk: r0={r0:.4} r1={r1:.4}");
        assert!(r2 < r1, "Two controls reduce more: r1={r1:.4} r2={r2:.4}");
    }

    // Test 7: Risk map is 5×5 with non-negative entries
    #[test]
    fn test_risk_map_shape() {
        let mut assessor = CyberRiskAssessor::new(make_config());
        assessor.add_asset(well_secured_asset());
        assessor.add_asset(vulnerable_asset());
        assessor.add_threat(basic_threat());

        let result = assessor.assess().expect("assessment ok");
        assert_eq!(result.risk_map.len(), 5, "Risk map should be 5 rows");
        for row in &result.risk_map {
            assert_eq!(row.len(), 5, "Each row should have 5 columns");
            for &val in row {
                assert!(val >= 0.0, "Risk map entries must be non-negative");
            }
        }
    }

    // Test 8: Residual risk after recommendations < current total risk score
    #[test]
    fn test_residual_risk_after_recommendations_decreases() {
        let config = CyberRiskConfig {
            risk_appetite: 0.01,
            ..make_config()
        };
        let mut assessor = CyberRiskAssessor::new(config);
        assessor.add_asset(vulnerable_asset());
        assessor.add_threat(basic_threat());

        let result = assessor.assess().expect("assessment ok");
        if !result.recommended_controls.is_empty() {
            assert!(
                result.residual_risk <= result.total_risk_score + 1e-9,
                "Residual after recommendations must be ≤ current risk: {:.4} vs {:.4}",
                result.residual_risk,
                result.total_risk_score
            );
        }
    }
}
