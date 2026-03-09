//! Cybersecurity modules for power grid protection.
//!
//! This module provides research and engineering tools for securing grid
//! state estimation against sensor manipulation attacks:
//!
//! - **`fdi`** — False Data Injection attack generation (for testing) and detection
//! - **`anomaly`** — Statistical anomaly detection in grid measurement time-series
//! - **`integrity`** — Physical-constraint-based measurement integrity verification
//! - **`ids`** — Multi-layer Intrusion Detection System (IDS) for grid cyber-security

pub mod anomaly;
pub mod cyber_physical;
pub mod fdi;
pub mod ids;
pub mod integrity;
pub mod risk_assessment;
pub mod vulnerability;

pub use anomaly::{AnomalyResult, GridAnomalyDetector, MeasurementCorrelationAnalyzer};
pub use cyber_physical::{
    AnomalyReport, AttackImpactResult, AttackObjective, AttackTiming, AttackVector,
    CpAttackScenario, CyberPhysicalSim, CyberPhysicalSimConfig, DefenseLayer, DefenseRoi,
    ImpactSeverity, PhysicalImpact, ResilienceMetrics, RiskMatrix,
};
pub use fdi::{DetectionMethod, DetectionResult, FdiAttack, FdiAttackGenerator, FdiDetector};
pub use ids::{
    AttackSignature, GridIds, GridIdsConfig, GridMeasurement, IdsAlert, IdsAlertType, IdsError,
    IdsResult,
};
pub use integrity::{IntegrityReport, IntegrityVerifier, MeasurementWatermark, PhysicalConstraint};
pub use risk_assessment::{
    AssetRisk, AttackType, CriticalAsset, CyberAssetType, CyberRiskAssessor, CyberRiskConfig,
    CyberRiskResult, RecommendedControl, RiskError, RiskRating, SecurityControl, ThreatLandscape,
    ThreatVector,
};
pub use vulnerability::{
    AttackScenario, CriticalElement, ElementType, GridVulnerabilityAssessor, ThreatModel,
    VulnError, VulnerabilityConfig, VulnerabilityResult,
};

pub mod anomaly_detection;
pub use anomaly_detection::{
    AnomalyDetector, CorrelationMatrix, DetectionAlert, DetectionSeverity, GridAnomalyType,
    Measurement, MeasurementKind, StatisticalModel,
};

pub mod real_time_sa;
pub use real_time_sa::{
    ConstraintType, Contingency, ContingencyAnalysisResult, ContingencyScreenResult,
    CorrectiveAction, CorrectiveActionType, RealTimeSecurityAssessor, RtsaConfig, RtsaResult,
    SecurityBoundaryTracer, SecurityLevel, SystemOperatingState,
};

pub mod threat_intelligence;
pub use threat_intelligence::{
    AnomalyAlert, AnomalySeverity, AnomalyType, AttackTechnique, ConnectivityLevel,
    CyberIncidentType, IcsAsset, IcsAssetType, IcsThreat, IcsThreatModel, IncidentPhase,
    IncidentResponsePlaybook, KnownVulnerability, MeasurementBaseline, NercCipChecker,
    NetworkTopology, PatchStatus, ResponseStep, ScadaComponent, ScadaControlType, ScadaProtocol,
    ScadaSecurityAssessment, ScadaSecurityControl, ThreatActor, ThreatAnomalyDetector,
    VulnScanReport, VulnerabilityScanner,
};
