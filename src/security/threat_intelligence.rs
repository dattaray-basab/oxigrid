//! Grid Cybersecurity Threat Intelligence.
//!
//! Provides a comprehensive threat intelligence and security assessment
//! framework aligned with MITRE ATT&CK for ICS, IEC 62443, and NERC CIP
//! standards:
//!
//! - [`IcsThreatModel`]            — MITRE ATT&CK for ICS threat modelling
//! - [`ThreatAnomalyDetector`]     — Statistical baseline anomaly detection
//! - [`ScadaSecurityAssessment`]   — SCADA component security scoring
//! - [`IncidentResponsePlaybook`]  — Cyber incident response playbooks
//! - [`VulnerabilityScanner`]      — ICS vulnerability database scanner
//! - [`NercCipChecker`]            — NERC CIP compliance checker
//!
//! # References
//!
//! - MITRE ATT&CK for ICS: <https://attack.mitre.org/matrices/ics/>
//! - IEC 62443 (Industrial Automation and Control Systems Security)
//! - NERC CIP-002 through CIP-014 standards
//! - NIST SP 800-82 Rev 3 (Guide to OT Security)

// ─── ICS Asset & Threat Modelling ────────────────────────────────────────────

/// Type of ICS/OT asset in a power grid environment.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum IcsAssetType {
    /// Programmable Logic Controller.
    Plc,
    /// Remote Terminal Unit.
    Rtu,
    /// Human-Machine Interface.
    Hmi,
    /// SCADA server / master station.
    Scada,
    /// Process historian database.
    Historian,
    /// Engineering workstation.
    EngineeringWorkstation,
    /// Network switch (OT LAN).
    NetworkSwitch,
    /// Router / WAN edge.
    Router,
    /// Firewall / UTM appliance.
    Firewall,
    /// Phasor Measurement Unit.
    Pmu,
    /// Protection relay (IEC 61850).
    ProtectionRelay,
}

/// Network connectivity exposure level of an ICS asset.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConnectivityLevel {
    /// No network connection — manual data transfer only.
    AirGapped,
    /// Isolated OT network, no IT/internet connection.
    IsolatedOt,
    /// Connected through a Demilitarised Zone (DMZ).
    PartialDmz,
    /// Direct internet or corporate IT connection.
    FullyConnected,
}

impl ConnectivityLevel {
    /// Multiplicative factor added to risk (0.0 = no extra risk).
    fn factor(&self) -> f64 {
        match self {
            Self::AirGapped => 0.0,
            Self::IsolatedOt => 0.2,
            Self::PartialDmz => 0.6,
            Self::FullyConnected => 1.0,
        }
    }
}

/// Patch / firmware currency status.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PatchStatus {
    /// Running latest vendor release.
    Current,
    /// One minor version behind.
    OneVersionBehind,
    /// Multiple versions behind.
    MultipleVersionsBehind,
    /// Vendor has dropped support.
    Unsupported,
}

impl PatchStatus {
    /// Divisor for risk (higher = better patch posture → lower risk).
    fn divisor(&self) -> f64 {
        match self {
            Self::Current => 2.0,
            Self::OneVersionBehind => 1.5,
            Self::MultipleVersionsBehind => 1.1,
            Self::Unsupported => 1.0,
        }
    }
}

/// An ICS/OT cyber asset in the grid environment.
#[derive(Debug, Clone)]
pub struct IcsAsset {
    /// Unique asset identifier.
    pub id: usize,
    /// Human-readable asset name.
    pub name: String,
    /// Category of asset.
    pub asset_type: IcsAssetType,
    /// Business / operational criticality (1 = low, 10 = critical).
    pub criticality: u8,
    /// Network connectivity exposure.
    pub connectivity: ConnectivityLevel,
    /// Current patch / firmware status.
    pub patch_status: PatchStatus,
    /// Known CVE identifiers affecting this asset.
    pub vulnerabilities: Vec<String>,
    /// Vendor name (used for vulnerability matching).
    pub vendor: String,
    /// SCADA protocol spoken by this asset.
    pub protocol: Option<ScadaProtocol>,
}

/// Threat actor category (MITRE ATT&CK for ICS).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ThreatActor {
    /// Nation-state sponsored APT group.
    NationState,
    /// Financially motivated criminal group.
    CriminalGroup,
    /// Trusted employee / contractor with malicious intent.
    Insider,
    /// Ideologically motivated hacktivist.
    Hacktivist,
    /// Attribution unknown.
    Unknown,
}

/// Attack technique aligned with MITRE ATT&CK for ICS.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AttackTechnique {
    /// T1566 – Phishing / spear-phishing.
    SpearPhishing,
    /// T1189 – Drive-by compromise via watering hole.
    WateringHole,
    /// T1195 – Supply chain compromise.
    SupplyChain,
    /// T0886 – Remote access via VPN/RDP/ICCP.
    RemoteAccess,
    /// T0812 – Lateral movement through OT network.
    LateralMovement,
    /// T0890 – Privilege escalation.
    PrivilegeEscalation,
    /// T0869 – Command and control over OT protocol.
    CommandAndControl,
    /// T0832 – Manipulate sensor readings (False Data Injection).
    FalseDataInjection,
    /// T0814 – Denial of Service against control plane.
    DenialOfService,
    /// T0857 – Firmware tampering / rootkit.
    FirmwareTampering,
}

/// A specific threat scenario: actor × technique × likelihood × impact.
#[derive(Debug, Clone)]
pub struct IcsThreat {
    /// Who is likely to conduct this attack.
    pub threat_actor: ThreatActor,
    /// How the attack is executed.
    pub technique: AttackTechnique,
    /// Estimated annual probability of occurrence \[0, 1\].
    pub probability: f64,
    /// Operational impact severity (1 = negligible, 10 = catastrophic).
    pub impact_severity: u8,
}

/// MITRE ATT&CK for ICS threat model for a power system or substation.
#[derive(Debug, Clone)]
pub struct IcsThreatModel {
    /// Identifier for the target system (e.g. substation name).
    pub system_id: String,
    /// Inventory of cyber assets within scope.
    pub asset_inventory: Vec<IcsAsset>,
    /// Threat scenarios applicable to this system.
    pub known_threats: Vec<IcsThreat>,
}

impl IcsThreatModel {
    /// Create an empty threat model.
    pub fn new(system_id: impl Into<String>) -> Self {
        Self {
            system_id: system_id.into(),
            asset_inventory: Vec::new(),
            known_threats: Vec::new(),
        }
    }

    /// Compute a composite risk score for one asset under one threat scenario.
    ///
    /// Formula:
    /// ```text
    /// risk = probability × impact × (1 + connectivity_factor) / patch_divisor
    /// ```
    ///
    /// Returned value is unbounded above but typically in \[0, ~20\].
    pub fn risk_score(&self, asset: &IcsAsset, threat: &IcsThreat) -> f64 {
        let connectivity_factor = asset.connectivity.factor();
        let patch_divisor = asset.patch_status.divisor();
        let impact = threat.impact_severity as f64;
        threat.probability * impact * (1.0 + connectivity_factor) / patch_divisor
    }

    /// Return the top-`n` assets by aggregate risk (summed over all threats).
    ///
    /// Each entry is `(asset_id, total_risk_score)`.
    pub fn highest_risk_assets(&self, top_n: usize) -> Vec<(usize, f64)> {
        let mut scores: Vec<(usize, f64)> = self
            .asset_inventory
            .iter()
            .map(|a| {
                let total: f64 = self
                    .known_threats
                    .iter()
                    .map(|t| self.risk_score(a, t))
                    .sum();
                (a.id, total)
            })
            .collect();
        scores.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        scores.truncate(top_n);
        scores
    }

    /// Sum of risk scores across all (asset, threat) pairs — the threat
    /// surface area \[dimensionless\].
    pub fn threat_surface_area(&self) -> f64 {
        self.asset_inventory
            .iter()
            .flat_map(|a| {
                self.known_threats
                    .iter()
                    .map(move |t| self.risk_score(a, t))
            })
            .sum()
    }

    /// Generate a prioritised list of mitigation recommendations.
    pub fn recommended_mitigations(&self) -> Vec<String> {
        let mut recs: Vec<String> = Vec::new();

        let has_unsupported = self
            .asset_inventory
            .iter()
            .any(|a| a.patch_status == PatchStatus::Unsupported);
        if has_unsupported {
            recs.push(
                "Replace or upgrade end-of-life / unsupported assets immediately (CIP-007-6 R2)"
                    .into(),
            );
        }

        let has_fully_connected = self
            .asset_inventory
            .iter()
            .any(|a| a.connectivity == ConnectivityLevel::FullyConnected);
        if has_fully_connected {
            recs.push(
                "Implement Electronic Security Perimeters and DMZ segmentation (CIP-005-7 R1)"
                    .into(),
            );
        }

        let has_fdi = self
            .known_threats
            .iter()
            .any(|t| t.technique == AttackTechnique::FalseDataInjection);
        if has_fdi {
            recs.push(
                "Deploy physics-based anomaly detection for FDI (measurement integrity checking)"
                    .into(),
            );
        }

        let has_nation_state = self
            .known_threats
            .iter()
            .any(|t| t.threat_actor == ThreatActor::NationState);
        if has_nation_state {
            recs.push(
                "Apply Zero Trust architecture and enforce MFA on all remote access paths".into(),
            );
        }

        let has_supply_chain = self
            .known_threats
            .iter()
            .any(|t| t.technique == AttackTechnique::SupplyChain);
        if has_supply_chain {
            recs.push(
                "Establish vendor risk management and firmware verification (NIST SP 800-161)"
                    .into(),
            );
        }

        if recs.is_empty() {
            recs.push(
                "No critical mitigations identified; maintain current security posture".into(),
            );
        }

        recs
    }
}

// ─── Anomaly Detection (Statistical Baseline) ────────────────────────────────

/// Statistical baseline for one measurement channel.
#[derive(Debug, Clone)]
pub struct MeasurementBaseline {
    /// Unique measurement channel identifier.
    pub measurement_id: usize,
    /// Human-readable channel name.
    pub measurement_name: String,
    /// Long-run mean value \[engineering units\].
    pub mean: f64,
    /// Long-run standard deviation \[engineering units\].
    pub std_dev: f64,
    /// Physical minimum valid value (hard bound).
    pub min_valid: f64,
    /// Physical maximum valid value (hard bound).
    pub max_valid: f64,
    /// Hourly seasonal means (24 entries, index = hour-of-day 0-23).
    pub seasonal_means: Vec<f64>,
}

/// Classification of detected anomaly.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AnomalyType {
    /// Value outside ±σ·threshold from baseline mean.
    StatisticalOutlier,
    /// Value violates physical bounds (e.g. negative power).
    PhysicalImpossible,
    /// Temporal pattern deviates from hourly seasonal profile.
    PatternAnomaly,
    /// Loss of expected correlation between measurement channels.
    CorrelationBreak,
}

/// Severity tier for anomaly alerts.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AnomalySeverity {
    /// z ∈ \[threshold, threshold+1\).
    Low,
    /// z ∈ \[threshold+1, threshold+3\).
    Medium,
    /// z ∈ \[threshold+3, threshold+6\).
    High,
    /// z ≥ threshold+6 or physical impossibility.
    Critical,
}

/// A detected anomaly event.
#[derive(Debug, Clone)]
pub struct AnomalyAlert {
    /// UNIX timestamp \[s\] when the anomaly was observed.
    pub timestamp: f64,
    /// Measurement channel that triggered the alert.
    pub measurement_id: usize,
    /// Raw observed value \[engineering units\].
    pub value: f64,
    /// Standardised deviation from baseline mean.
    pub z_score: f64,
    /// Classification of the anomaly.
    pub alert_type: AnomalyType,
    /// Operational severity.
    pub severity: AnomalySeverity,
}

/// Statistical baseline anomaly detector for grid measurement streams.
///
/// Each measurement channel maintains its own baseline. Detection uses:
/// 1. Hard physical bounds check.
/// 2. Hourly seasonal z-score (if seasonal means are populated).
/// 3. Global z-score against `mean` ± `std_dev`.
#[derive(Debug, Clone)]
pub struct ThreatAnomalyDetector {
    /// Per-channel baselines.
    pub measurement_baselines: Vec<MeasurementBaseline>,
    /// Number of standard deviations required to trigger an alert \[σ\].
    pub alert_threshold_sigma: f64,
    /// Historical alert log.
    pub detection_history: Vec<AnomalyAlert>,
}

impl ThreatAnomalyDetector {
    /// Construct a new detector with the given sigma threshold.
    pub fn new(threshold_sigma: f64) -> Self {
        Self {
            measurement_baselines: Vec::new(),
            alert_threshold_sigma: threshold_sigma,
            detection_history: Vec::new(),
        }
    }

    /// Register a new measurement baseline.
    pub fn add_baseline(&mut self, baseline: MeasurementBaseline) {
        self.measurement_baselines.push(baseline);
    }

    /// Update baseline mean with exponential moving average (EMA).
    ///
    /// `new_mean = alpha * value + (1 - alpha) * old_mean`
    ///
    /// `alpha` ∈ \[0, 1\]; typical value 0.01–0.1 for slow adaptation.
    pub fn update_baseline(&mut self, id: usize, new_value: f64, alpha: f64) {
        if let Some(b) = self
            .measurement_baselines
            .iter_mut()
            .find(|b| b.measurement_id == id)
        {
            b.mean = alpha * new_value + (1.0 - alpha) * b.mean;
        }
    }

    fn severity_from_z(z_abs: f64, threshold: f64) -> AnomalySeverity {
        let excess = z_abs - threshold;
        if excess < 1.0 {
            AnomalySeverity::Low
        } else if excess < 3.0 {
            AnomalySeverity::Medium
        } else if excess < 6.0 {
            AnomalySeverity::High
        } else {
            AnomalySeverity::Critical
        }
    }

    /// Analyse a single incoming measurement sample.
    ///
    /// Returns `Some(AnomalyAlert)` if the sample is anomalous, `None` otherwise.
    ///
    /// `hour_of_day` ∈ \[0, 23\] is used for seasonal comparison when the
    /// baseline has 24 hourly means; otherwise the global mean is used.
    pub fn detect(
        &mut self,
        measurement_id: usize,
        value: f64,
        timestamp: f64,
        hour_of_day: usize,
    ) -> Option<AnomalyAlert> {
        let baseline = self
            .measurement_baselines
            .iter()
            .find(|b| b.measurement_id == measurement_id)?;

        // 1. Physical bounds check.
        if value < baseline.min_valid || value > baseline.max_valid {
            let alert = AnomalyAlert {
                timestamp,
                measurement_id,
                value,
                z_score: f64::NAN,
                alert_type: AnomalyType::PhysicalImpossible,
                severity: AnomalySeverity::Critical,
            };
            self.detection_history.push(alert.clone());
            return Some(alert);
        }

        // 2. Choose reference mean: seasonal if available.
        let ref_mean = if baseline.seasonal_means.len() == 24 {
            let h = hour_of_day.min(23);
            baseline.seasonal_means[h]
        } else {
            baseline.mean
        };

        let std = if baseline.std_dev > 1e-12 {
            baseline.std_dev
        } else {
            1e-12
        };
        let z = (value - ref_mean) / std;
        let z_abs = z.abs();

        // 3. Check seasonal pattern anomaly.
        if baseline.seasonal_means.len() == 24 {
            let global_z = (value - baseline.mean) / std;
            if global_z.abs() > self.alert_threshold_sigma && z_abs > self.alert_threshold_sigma {
                let severity = Self::severity_from_z(z_abs, self.alert_threshold_sigma);
                let alert = AnomalyAlert {
                    timestamp,
                    measurement_id,
                    value,
                    z_score: z,
                    alert_type: AnomalyType::PatternAnomaly,
                    severity,
                };
                self.detection_history.push(alert.clone());
                return Some(alert);
            }
        }

        // 4. Global z-score outlier test.
        if z_abs > self.alert_threshold_sigma {
            let severity = Self::severity_from_z(z_abs, self.alert_threshold_sigma);
            let alert = AnomalyAlert {
                timestamp,
                measurement_id,
                value,
                z_score: z,
                alert_type: AnomalyType::StatisticalOutlier,
                severity,
            };
            self.detection_history.push(alert.clone());
            return Some(alert);
        }

        None
    }

    /// Estimate detectability of a hypothetical False Data Injection attack.
    ///
    /// `injected_values` is a slice of `(measurement_id, injected_value)`.
    ///
    /// Returns a score in \[0, 1\]:
    /// - 0 = attack would be undetectable (all z-scores below threshold)
    /// - 1 = attack is obviously anomalous (very high z-scores)
    pub fn false_data_injection_score(&self, injected_values: &[(usize, f64)]) -> f64 {
        if injected_values.is_empty() {
            return 0.0;
        }
        let mut total_detectability = 0.0_f64;
        let mut count = 0usize;

        for (id, val) in injected_values {
            if let Some(b) = self
                .measurement_baselines
                .iter()
                .find(|b| b.measurement_id == *id)
            {
                let std = if b.std_dev > 1e-12 { b.std_dev } else { 1e-12 };
                let z = ((val - b.mean) / std).abs();
                // Sigmoid-like mapping: tanh(z / threshold)
                let detectability = (z / self.alert_threshold_sigma.max(1e-12)).tanh();
                total_detectability += detectability;
                count += 1;
            }
        }

        if count == 0 {
            return 0.0;
        }
        (total_detectability / count as f64).clamp(0.0, 1.0)
    }

    /// Return all alerts that occurred within `window_s` seconds before
    /// `current_time`.
    pub fn recent_alerts(&self, window_s: f64, current_time: f64) -> Vec<&AnomalyAlert> {
        let cutoff = current_time - window_s;
        self.detection_history
            .iter()
            .filter(|a| a.timestamp >= cutoff)
            .collect()
    }
}

// ─── SCADA Security Assessment ───────────────────────────────────────────────

/// Industrial communication protocol used by a SCADA component.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ScadaProtocol {
    /// Modbus TCP/RTU (no native security).
    Modbus,
    /// DNP3 (Secure Authentication v5 optional).
    Dnp3,
    /// IEC 61850 (MMS/GOOSE/SV with TLS optional).
    Iec61850,
    /// PROFIBUS (no native security).
    Profibus,
    /// OPC Unified Architecture (built-in security stack).
    OpcUa,
    /// IEC 60870-5-104 (no native security).
    Iec104,
    /// MQTT (TLS optional).
    Mqtt,
}

impl ScadaProtocol {
    /// Returns `true` if the protocol includes native encryption support.
    pub fn has_native_encryption(&self) -> bool {
        matches!(self, Self::OpcUa)
    }

    /// Returns `true` if the protocol supports native authentication.
    pub fn has_native_authentication(&self) -> bool {
        matches!(self, Self::OpcUa | Self::Dnp3)
    }
}

/// A single SCADA system component (server, HMI, RTU, etc.).
#[derive(Debug, Clone)]
pub struct ScadaComponent {
    /// Unique identifier within the assessment scope.
    pub id: usize,
    /// Human-readable component name.
    pub name: String,
    /// SCADA/OT communication protocol in use.
    pub protocol: ScadaProtocol,
    /// Whether communications are encrypted (transport layer).
    pub encrypted: bool,
    /// Whether the component enforces authentication.
    pub authenticated: bool,
    /// Days since last security audit / penetration test.
    pub last_audit_days: u32,
}

/// OT network topology characteristics.
#[derive(Debug, Clone)]
pub struct NetworkTopology {
    /// Number of distinct security zones (OT, DMZ, IT, …).
    pub zones: usize,
    /// Whether a DMZ separates OT from IT/internet.
    pub dmz_present: bool,
    /// Whether data diodes / unidirectional security gateways are deployed.
    pub unidirectional_gateways: bool,
}

/// Type of security control.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ScadaControlType {
    /// Network firewall / packet filter.
    Firewall,
    /// Intrusion Detection / Prevention System.
    Ids,
    /// Automated asset inventory and discovery.
    AssetInventory,
    /// Patch and vulnerability management process.
    PatchManagement,
    /// Encrypted communications (TLS/IPSec).
    EncryptedComms,
    /// Multi-Factor Authentication.
    Mfa,
    /// Documented and tested incident response plan.
    IncidentResponse,
}

/// A technical or organisational security control.
#[derive(Debug, Clone)]
pub struct ScadaSecurityControl {
    /// Category of security control.
    pub control_type: ScadaControlType,
    /// Whether this control is deployed and operational.
    pub implemented: bool,
    /// Estimated effectiveness \[0, 1\] when implemented.
    pub effectiveness: f64,
}

/// SCADA system security assessment (IEC 62443 / NERC CIP aligned).
#[derive(Debug, Clone)]
pub struct ScadaSecurityAssessment {
    /// Name of the SCADA system being assessed.
    pub system_name: String,
    /// Inventory of assessed SCADA components.
    pub components: Vec<ScadaComponent>,
    /// OT network topology.
    pub network_topology: NetworkTopology,
    /// Security controls catalogue.
    pub security_controls: Vec<ScadaSecurityControl>,
}

impl ScadaSecurityAssessment {
    /// Composite security score \[0, 100\] (higher = more secure).
    ///
    /// Score contributions:
    /// - Encryption \[%\]: 25 points
    /// - Authentication \[%\]: 20 points
    /// - Security controls effectiveness: 30 points
    /// - Network topology: 15 points
    /// - Audit currency: 10 points
    pub fn security_score(&self) -> f64 {
        if self.components.is_empty() {
            return 0.0;
        }

        // Encryption ratio.
        let enc_ratio = self.components.iter().filter(|c| c.encrypted).count() as f64
            / self.components.len() as f64;

        // Authentication ratio.
        let auth_ratio = self.components.iter().filter(|c| c.authenticated).count() as f64
            / self.components.len() as f64;

        // Security controls score.
        let controls_score = if self.security_controls.is_empty() {
            0.0
        } else {
            let implemented_eff: f64 = self
                .security_controls
                .iter()
                .filter(|c| c.implemented)
                .map(|c| c.effectiveness)
                .sum();
            let max_eff: f64 = self.security_controls.iter().map(|c| c.effectiveness).sum();
            if max_eff > 0.0 {
                implemented_eff / max_eff
            } else {
                0.0
            }
        };

        // Network topology bonus.
        let topo_score = {
            let mut t = 0.0_f64;
            if self.network_topology.zones >= 3 {
                t += 0.5;
            } else if self.network_topology.zones >= 2 {
                t += 0.25;
            }
            if self.network_topology.dmz_present {
                t += 0.3;
            }
            if self.network_topology.unidirectional_gateways {
                t += 0.2;
            }
            t.min(1.0)
        };

        // Audit currency: full score if audited within 365 days.
        let audit_score = {
            let max_days = self
                .components
                .iter()
                .map(|c| c.last_audit_days)
                .max()
                .unwrap_or(u32::MAX);
            (1.0 - (max_days as f64 / 730.0).min(1.0)).max(0.0)
        };

        let score = enc_ratio * 25.0
            + auth_ratio * 20.0
            + controls_score * 30.0
            + topo_score * 15.0
            + audit_score * 10.0;

        score.clamp(0.0, 100.0)
    }

    /// List components that do not use encrypted communications.
    pub fn unencrypted_protocols(&self) -> Vec<&ScadaComponent> {
        self.components.iter().filter(|c| !c.encrypted).collect()
    }

    /// Estimate IEC 62443 Security Level (SL 0–4).
    ///
    /// | SL | Meaning                                       |
    /// |----|-----------------------------------------------|
    /// | 0  | No specific requirements                      |
    /// | 1  | Protection against unintentional violation    |
    /// | 2  | Protection against intentional simple means   |
    /// | 3  | Protection against sophisticated means        |
    /// | 4  | Protection against nation-state APT           |
    pub fn iec62443_compliance_level(&self) -> u8 {
        let score = self.security_score();
        match score as u32 {
            0..=24 => 0,
            25..=49 => 1,
            50..=69 => 2,
            70..=89 => 3,
            _ => 4,
        }
    }

    /// Count components with known security weaknesses (unencrypted or
    /// unauthenticated and using legacy protocols).
    pub fn vulnerability_count(&self) -> usize {
        self.components
            .iter()
            .filter(|c| {
                !c.encrypted
                    || !c.authenticated
                    || matches!(c.protocol, ScadaProtocol::Modbus | ScadaProtocol::Profibus)
            })
            .count()
    }

    /// Generate prioritised remediation recommendations.
    pub fn recommended_actions(&self) -> Vec<String> {
        let mut actions: Vec<String> = Vec::new();

        let unenc = self.unencrypted_protocols();
        if !unenc.is_empty() {
            actions.push(format!(
                "Enable TLS/IPSec for {} unencrypted component(s) (IEC 62443-3-3 SR 4.3)",
                unenc.len()
            ));
        }

        let unauth = self.components.iter().filter(|c| !c.authenticated).count();
        if unauth > 0 {
            actions.push(format!(
                "Enforce authentication on {} component(s) — consider MFA (CIP-007-6 R5)",
                unauth
            ));
        }

        let overdue_audit = self
            .components
            .iter()
            .filter(|c| c.last_audit_days > 365)
            .count();
        if overdue_audit > 0 {
            actions.push(format!(
                "Schedule penetration tests for {} component(s) overdue for security audit",
                overdue_audit
            ));
        }

        let missing_controls: Vec<_> = self
            .security_controls
            .iter()
            .filter(|c| !c.implemented)
            .collect();
        if !missing_controls.is_empty() {
            actions.push(format!(
                "Implement {} missing security control(s) to improve IEC 62443 SL",
                missing_controls.len()
            ));
        }

        if !self.network_topology.dmz_present {
            actions.push("Deploy a DMZ between OT and IT networks (IEC 62443-3-2 ZCR 3.1)".into());
        }

        if actions.is_empty() {
            actions
                .push("Security posture meets baseline requirements; continue monitoring".into());
        }

        actions
    }
}

// ─── Incident Response Playbook ───────────────────────────────────────────────

/// Category of cyber incident in a grid environment.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CyberIncidentType {
    /// Ransomware encryption of OT/IT systems.
    RansomwareAttack,
    /// False Data Injection into SCADA measurements.
    FalseDataInjection,
    /// Unauthorised access to OT systems.
    UnauthorizedAccess,
    /// Denial of Service against control communications.
    DenialOfService,
    /// Malicious action by trusted insider.
    InsiderThreat,
    /// Compromise via vendor software / hardware update.
    SupplyChainCompromise,
}

/// Phase of the incident response lifecycle.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum IncidentPhase {
    /// Detection and initial triage.
    Detect,
    /// Isolation and containment to limit spread.
    Contain,
    /// Root-cause removal and eradication.
    Eradicate,
    /// System restoration and return to operations.
    Recover,
    /// Lessons-learned and process improvement.
    PostIncident,
}

/// A single action step in an incident response workflow.
#[derive(Debug, Clone)]
pub struct ResponseStep {
    /// Lifecycle phase this step belongs to.
    pub phase: IncidentPhase,
    /// Specific action to be taken.
    pub action: String,
    /// Team or role responsible for executing this step.
    pub responsible_team: String,
    /// Target completion time from incident declaration \[minutes\].
    pub time_target_minutes: u32,
}

/// Structured incident response playbook for a specific incident type.
#[derive(Debug, Clone)]
pub struct IncidentResponsePlaybook {
    /// Type of cyber incident this playbook addresses.
    pub incident_type: CyberIncidentType,
    /// Ordered list of response actions.
    pub steps: Vec<ResponseStep>,
    /// Names / emails of escalation contacts.
    pub escalation_contacts: Vec<String>,
    /// Regulatory bodies requiring notification (e.g. NERC CIP, NIS2).
    pub regulatory_notifications: Vec<String>,
}

impl IncidentResponsePlaybook {
    /// Pre-built playbook for ransomware incidents.
    pub fn for_ransomware() -> Self {
        Self {
            incident_type: CyberIncidentType::RansomwareAttack,
            steps: vec![
                ResponseStep {
                    phase: IncidentPhase::Detect,
                    action: "Identify encrypted files and affected systems via EDR alerts".into(),
                    responsible_team: "SOC / OT Security Team".into(),
                    time_target_minutes: 15,
                },
                ResponseStep {
                    phase: IncidentPhase::Detect,
                    action: "Activate incident commander and notify CISO".into(),
                    responsible_team: "Incident Commander".into(),
                    time_target_minutes: 30,
                },
                ResponseStep {
                    phase: IncidentPhase::Contain,
                    action: "Isolate affected OT network segments at firewall and switch level"
                        .into(),
                    responsible_team: "Network / OT Team".into(),
                    time_target_minutes: 45,
                },
                ResponseStep {
                    phase: IncidentPhase::Contain,
                    action: "Switch control to manual / backup SCADA if available".into(),
                    responsible_team: "Control Room / Operations".into(),
                    time_target_minutes: 60,
                },
                ResponseStep {
                    phase: IncidentPhase::Eradicate,
                    action: "Preserve forensic images before remediation".into(),
                    responsible_team: "Forensics / IR Team".into(),
                    time_target_minutes: 180,
                },
                ResponseStep {
                    phase: IncidentPhase::Eradicate,
                    action: "Wipe and reinstall affected systems from known-good golden images"
                        .into(),
                    responsible_team: "IT / OT Engineering".into(),
                    time_target_minutes: 720,
                },
                ResponseStep {
                    phase: IncidentPhase::Recover,
                    action: "Restore from verified offline backups; validate data integrity".into(),
                    responsible_team: "IT / OT Engineering".into(),
                    time_target_minutes: 1440,
                },
                ResponseStep {
                    phase: IncidentPhase::Recover,
                    action: "Gradually restore OT systems with enhanced monitoring".into(),
                    responsible_team: "Operations / OT Team".into(),
                    time_target_minutes: 2880,
                },
                ResponseStep {
                    phase: IncidentPhase::PostIncident,
                    action: "Conduct root-cause analysis and update threat model".into(),
                    responsible_team: "Security / Engineering".into(),
                    time_target_minutes: 7200,
                },
            ],
            escalation_contacts: vec![
                "CISO <ciso@utility.example>".into(),
                "NERC E-ISAC <eisac@nerc.net>".into(),
                "CISA ICS-CERT <ics-cert@cisa.dhs.gov>".into(),
            ],
            regulatory_notifications: vec![
                "NERC CIP-008-6 R1.3 (notify within 1 hour)".into(),
                "NIS2 Directive Art. 23 (notify within 24 hours)".into(),
                "DOE OE-417 (notify within 1 hour)".into(),
            ],
        }
    }

    /// Pre-built playbook for False Data Injection incidents.
    pub fn for_fdi() -> Self {
        Self {
            incident_type: CyberIncidentType::FalseDataInjection,
            steps: vec![
                ResponseStep {
                    phase: IncidentPhase::Detect,
                    action: "Correlate anomalous measurement z-scores across measurement channels"
                        .into(),
                    responsible_team: "SOC / SCADA Analyst".into(),
                    time_target_minutes: 10,
                },
                ResponseStep {
                    phase: IncidentPhase::Detect,
                    action: "Cross-validate SCADA readings against PMU synchrophasor data".into(),
                    responsible_team: "Control Room Engineer".into(),
                    time_target_minutes: 20,
                },
                ResponseStep {
                    phase: IncidentPhase::Contain,
                    action: "Flag affected measurements as untrusted in state estimator".into(),
                    responsible_team: "EMS / SCADA Team".into(),
                    time_target_minutes: 30,
                },
                ResponseStep {
                    phase: IncidentPhase::Contain,
                    action: "Revert to manual readings from substation field instrumentation"
                        .into(),
                    responsible_team: "Control Room / Field Team".into(),
                    time_target_minutes: 60,
                },
                ResponseStep {
                    phase: IncidentPhase::Eradicate,
                    action: "Identify compromised RTU/sensor and isolate from network".into(),
                    responsible_team: "OT Security / Field Team".into(),
                    time_target_minutes: 120,
                },
                ResponseStep {
                    phase: IncidentPhase::Recover,
                    action: "Recalibrate / replace tampered sensors and restore trusted data feeds"
                        .into(),
                    responsible_team: "Field Engineering".into(),
                    time_target_minutes: 480,
                },
                ResponseStep {
                    phase: IncidentPhase::PostIncident,
                    action: "Deploy physics-based bad-data detection in state estimator".into(),
                    responsible_team: "EMS / Security Engineering".into(),
                    time_target_minutes: 10080,
                },
            ],
            escalation_contacts: vec![
                "Control Room Manager <cr-manager@utility.example>".into(),
                "NERC E-ISAC <eisac@nerc.net>".into(),
            ],
            regulatory_notifications: vec![
                "NERC CIP-008-6 R1 (if BES cyber system affected)".into(),
                "NIS2 Directive Art. 23".into(),
            ],
        }
    }

    /// Total planned response time in hours (sum of maximum step targets).
    pub fn total_response_time_hours(&self) -> f64 {
        let max_minutes = self
            .steps
            .iter()
            .map(|s| s.time_target_minutes)
            .max()
            .unwrap_or(0);
        max_minutes as f64 / 60.0
    }

    /// Filter steps belonging to the specified incident phase.
    pub fn steps_by_phase(&self, phase: &IncidentPhase) -> Vec<&ResponseStep> {
        self.steps.iter().filter(|s| &s.phase == phase).collect()
    }
}

// ─── Vulnerability Scanner (ICS CVE Database) ────────────────────────────────

/// A known ICS vulnerability from the CVE database.
#[derive(Debug, Clone)]
pub struct KnownVulnerability {
    /// CVE identifier (e.g. "CVE-2024-00001").
    pub cve_id: String,
    /// CVSS v3.1 base score \[0, 10\].
    pub cvss_score: f64,
    /// SCADA/OT protocols affected by this vulnerability.
    pub affected_protocols: Vec<ScadaProtocol>,
    /// Vendor names affected.
    pub affected_vendors: Vec<String>,
    /// Whether an official vendor patch exists.
    pub patch_available: bool,
    /// Whether working exploits are known in the wild.
    pub exploit_in_wild: bool,
    /// Brief technical description.
    pub description: String,
}

/// Simulated ICS vulnerability scanner backed by a pre-seeded CVE database.
#[derive(Debug, Clone)]
pub struct VulnerabilityScanner {
    /// Known ICS vulnerabilities in the scanner database.
    pub known_vulnerabilities: Vec<KnownVulnerability>,
}

/// Aggregated vulnerability scan report.
#[derive(Debug, Clone)]
pub struct VulnScanReport {
    /// Total number of assets scanned.
    pub total_assets: usize,
    /// Number of assets with at least one matched vulnerability.
    pub vulnerable_assets: usize,
    /// Number of critical vulnerabilities (CVSS ≥ 9.0).
    pub critical_vulns: usize,
    /// Number of high vulnerabilities (CVSS 7.0–8.9).
    pub high_vulns: usize,
    /// CVE IDs requiring vendor patches.
    pub patch_required: Vec<String>,
    /// Aggregate risk score across all assets and vulnerabilities.
    pub overall_risk_score: f64,
}

impl VulnerabilityScanner {
    /// Construct a scanner pre-seeded with 5 representative ICS CVEs.
    ///
    /// Note: CVE IDs below are fictional placeholders for demonstration.
    pub fn new_with_ics_database() -> Self {
        Self {
            known_vulnerabilities: vec![
                KnownVulnerability {
                    cve_id: "CVE-2024-10001".into(),
                    cvss_score: 9.8,
                    affected_protocols: vec![ScadaProtocol::Modbus],
                    affected_vendors: vec!["GenericPLC Corp".into(), "IndustrialSoft".into()],
                    patch_available: false,
                    exploit_in_wild: true,
                    description: "Unauthenticated Modbus write allows arbitrary coil manipulation \
                                  on affected PLCs — no authentication required."
                        .into(),
                },
                KnownVulnerability {
                    cve_id: "CVE-2024-10002".into(),
                    cvss_score: 8.1,
                    affected_protocols: vec![ScadaProtocol::Dnp3],
                    affected_vendors: vec!["GridComm Systems".into()],
                    patch_available: true,
                    exploit_in_wild: false,
                    description: "DNP3 Secure Authentication v5 implementation allows replay \
                                  attacks due to weak nonce generation."
                        .into(),
                },
                KnownVulnerability {
                    cve_id: "CVE-2024-10003".into(),
                    cvss_score: 7.5,
                    affected_protocols: vec![ScadaProtocol::Iec104],
                    affected_vendors: vec!["TeleControl AG".into(), "GridComm Systems".into()],
                    patch_available: true,
                    exploit_in_wild: false,
                    description: "IEC 60870-5-104 server crashes on malformed ASDU type 0x7F \
                                  causing denial of service."
                        .into(),
                },
                KnownVulnerability {
                    cve_id: "CVE-2024-10004".into(),
                    cvss_score: 9.0,
                    affected_protocols: vec![ScadaProtocol::Iec61850],
                    affected_vendors: vec!["ProtectRelay GmbH".into()],
                    patch_available: false,
                    exploit_in_wild: true,
                    description: "IEC 61850 MMS server stack overflow via crafted Read request \
                                  allows remote code execution on protection relays."
                        .into(),
                },
                KnownVulnerability {
                    cve_id: "CVE-2024-10005".into(),
                    cvss_score: 6.5,
                    affected_protocols: vec![ScadaProtocol::OpcUa],
                    affected_vendors: vec!["IndustrialSoft".into()],
                    patch_available: true,
                    exploit_in_wild: false,
                    description: "OPC UA server leaks session token in HTTP header allowing \
                                  session hijacking by network-adjacent attacker."
                        .into(),
                },
            ],
        }
    }

    /// Scan a single asset and return matching vulnerabilities from the database.
    ///
    /// Matching criteria (either condition sufficient):
    /// - Asset protocol matches `affected_protocols`, OR
    /// - Asset vendor matches any entry in `affected_vendors`.
    pub fn scan_asset<'a>(&'a self, asset: &IcsAsset) -> Vec<&'a KnownVulnerability> {
        self.known_vulnerabilities
            .iter()
            .filter(|v| {
                let proto_match = asset
                    .protocol
                    .as_ref()
                    .map(|p| v.affected_protocols.contains(p))
                    .unwrap_or(false);
                let vendor_match = v
                    .affected_vendors
                    .iter()
                    .any(|av| av.eq_ignore_ascii_case(&asset.vendor));
                proto_match || vendor_match
            })
            .collect()
    }

    /// Risk Priority Score for a single vulnerability.
    ///
    /// ```text
    /// RPS = CVSS × (1 + exploit_in_wild) × (1 + !patch_available)
    /// ```
    ///
    /// RPS range: \[0, 40\] (when CVSS = 10, exploit active, no patch).
    pub fn risk_priority_score(&self, vuln: &KnownVulnerability) -> f64 {
        let exploit_factor = if vuln.exploit_in_wild { 1.0 } else { 0.0 };
        let patch_factor = if vuln.patch_available { 0.0 } else { 1.0 };
        vuln.cvss_score * (1.0 + exploit_factor) * (1.0 + patch_factor)
    }

    /// Generate a summary scan report across a set of assets.
    pub fn generate_report(&self, assets: &[IcsAsset]) -> VulnScanReport {
        let total_assets = assets.len();
        let mut vulnerable_assets = 0usize;
        let mut critical_vulns = 0usize;
        let mut high_vulns = 0usize;
        let mut patch_required: Vec<String> = Vec::new();
        let mut overall_risk_score = 0.0_f64;

        for asset in assets {
            let matches = self.scan_asset(asset);
            if !matches.is_empty() {
                vulnerable_assets += 1;
            }
            for vuln in &matches {
                if vuln.cvss_score >= 9.0 {
                    critical_vulns += 1;
                } else if vuln.cvss_score >= 7.0 {
                    high_vulns += 1;
                }
                if !vuln.patch_available && !patch_required.contains(&vuln.cve_id) {
                    patch_required.push(vuln.cve_id.clone());
                }
                overall_risk_score += self.risk_priority_score(vuln);
            }
        }

        VulnScanReport {
            total_assets,
            vulnerable_assets,
            critical_vulns,
            high_vulns,
            patch_required,
            overall_risk_score,
        }
    }
}

// ─── NERC CIP Compliance Checker ─────────────────────────────────────────────

/// NERC CIP compliance checker for Bulk Electric System (BES) cyber assets.
#[derive(Debug, Clone)]
pub struct NercCipChecker {
    /// BES cyber assets within scope.
    pub bcs: Vec<IcsAsset>,
    /// Implemented status per NERC CIP standard: `(standard_id, implemented)`.
    pub controls_implemented: Vec<(String, bool)>,
}

impl NercCipChecker {
    /// Construct a checker pre-populated with CIP-002 through CIP-014 standards.
    pub fn new_with_standards() -> Self {
        Self {
            bcs: Vec::new(),
            controls_implemented: vec![
                ("CIP-002-5.1a".into(), false), // BES Cyber System Categorization
                ("CIP-003-8".into(), false),    // Security Management Controls
                ("CIP-004-6".into(), false),    // Personnel & Training
                ("CIP-005-7".into(), false),    // Electronic Security Perimeters
                ("CIP-006-6".into(), false),    // Physical Security of BES Cyber Systems
                ("CIP-007-6".into(), false),    // System Security Management
                ("CIP-008-6".into(), false),    // Incident Reporting and Response Planning
                ("CIP-009-6".into(), false),    // Recovery Plans for BES Cyber Systems
                ("CIP-010-4".into(), false),    // Configuration Change Management
                ("CIP-011-3".into(), false),    // Information Protection
                ("CIP-012-1".into(), false),    // Communications between Control Centers
                ("CIP-013-2".into(), false),    // Supply Chain Risk Management
                ("CIP-014-3".into(), false),    // Physical Security (Transmission Stations)
            ],
        }
    }

    /// Mark a specific standard as implemented.
    pub fn set_implemented(&mut self, standard_prefix: &str, implemented: bool) {
        for (std_id, imp) in &mut self.controls_implemented {
            if std_id.starts_with(standard_prefix) {
                *imp = implemented;
            }
        }
    }

    /// CIP-002: BES Cyber System categorisation is complete.
    ///
    /// Passes if the organisation has assigned a High/Medium/Low impact rating
    /// to each asset (approximated here by asset criticality ≥ 1) AND the
    /// standard is marked implemented.
    pub fn check_cip_002(&self) -> bool {
        let all_categorised = self.bcs.iter().all(|a| a.criticality >= 1);
        let standard_on = self
            .controls_implemented
            .iter()
            .any(|(s, imp)| s.starts_with("CIP-002") && *imp);
        all_categorised && standard_on
    }

    /// CIP-005: Electronic Security Perimeters are defined.
    ///
    /// Passes if no BES asset is `FullyConnected` without going through a
    /// DMZ, AND the standard is marked implemented.
    pub fn check_cip_005(&self) -> bool {
        let no_direct_exposure = !self
            .bcs
            .iter()
            .any(|a| a.connectivity == ConnectivityLevel::FullyConnected);
        let standard_on = self
            .controls_implemented
            .iter()
            .any(|(s, imp)| s.starts_with("CIP-005") && *imp);
        no_direct_exposure && standard_on
    }

    /// CIP-007: System security management controls are in place.
    ///
    /// Passes if all BES assets are patched to at most one version behind AND
    /// the standard is marked implemented.
    pub fn check_cip_007(&self) -> bool {
        let all_patched = self.bcs.iter().all(|a| {
            matches!(
                a.patch_status,
                PatchStatus::Current | PatchStatus::OneVersionBehind
            )
        });
        let standard_on = self
            .controls_implemented
            .iter()
            .any(|(s, imp)| s.starts_with("CIP-007") && *imp);
        all_patched && standard_on
    }

    /// CIP-010: Configuration change management is documented.
    ///
    /// Passes when the standard is marked implemented.
    pub fn check_cip_010(&self) -> bool {
        self.controls_implemented
            .iter()
            .any(|(s, imp)| s.starts_with("CIP-010") && *imp)
    }

    /// Percentage of CIP standards currently implemented \[%\].
    pub fn overall_compliance_pct(&self) -> f64 {
        if self.controls_implemented.is_empty() {
            return 0.0;
        }
        let implemented = self
            .controls_implemented
            .iter()
            .filter(|(_, imp)| *imp)
            .count();
        (implemented as f64 / self.controls_implemented.len() as f64 * 100.0).clamp(0.0, 100.0)
    }

    /// Return standard identifiers that are not yet implemented.
    pub fn non_compliant_standards(&self) -> Vec<String> {
        self.controls_implemented
            .iter()
            .filter(|(_, imp)| !*imp)
            .map(|(s, _)| s.clone())
            .collect()
    }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_asset(id: usize, conn: ConnectivityLevel, patch: PatchStatus) -> IcsAsset {
        IcsAsset {
            id,
            name: format!("Asset-{id}"),
            asset_type: IcsAssetType::Plc,
            criticality: 8,
            connectivity: conn,
            patch_status: patch,
            vulnerabilities: vec![],
            vendor: "GenericPLC Corp".into(),
            protocol: Some(ScadaProtocol::Modbus),
        }
    }

    fn make_threat(prob: f64, impact: u8) -> IcsThreat {
        IcsThreat {
            threat_actor: ThreatActor::NationState,
            technique: AttackTechnique::FalseDataInjection,
            probability: prob,
            impact_severity: impact,
        }
    }

    // ── IcsThreatModel ────────────────────────────────────────────────────────

    #[test]
    fn test_risk_score_positive() {
        let mut model = IcsThreatModel::new("Substation-A");
        let asset = make_asset(1, ConnectivityLevel::PartialDmz, PatchStatus::Current);
        let threat = make_threat(0.3, 8);
        model.asset_inventory.push(asset.clone());
        model.known_threats.push(threat.clone());
        let score = model.risk_score(&asset, &threat);
        assert!(score > 0.0, "risk_score should be positive: {score}");
    }

    #[test]
    fn test_highest_risk_assets_top_n() {
        let mut model = IcsThreatModel::new("Substation-B");
        for i in 1..=5 {
            model.asset_inventory.push(make_asset(
                i,
                ConnectivityLevel::FullyConnected,
                PatchStatus::Unsupported,
            ));
        }
        model.known_threats.push(make_threat(0.5, 9));
        let top2 = model.highest_risk_assets(2);
        assert_eq!(top2.len(), 2, "should return exactly 2 assets");
    }

    #[test]
    fn test_threat_surface_area_sums_all() {
        let mut model = IcsThreatModel::new("Substation-C");
        model.asset_inventory.push(make_asset(
            1,
            ConnectivityLevel::IsolatedOt,
            PatchStatus::Current,
        ));
        model.asset_inventory.push(make_asset(
            2,
            ConnectivityLevel::PartialDmz,
            PatchStatus::OneVersionBehind,
        ));
        model.known_threats.push(make_threat(0.2, 5));
        model.known_threats.push(make_threat(0.1, 7));

        let surface = model.threat_surface_area();
        // Manual sum: avoid borrow conflicts by collecting scores independently.
        let mut manual = 0.0_f64;
        for a in &model.asset_inventory {
            for t in &model.known_threats {
                manual += model.risk_score(a, t);
            }
        }
        assert!((surface - manual).abs() < 1e-10);
        assert!(surface > 0.0);
    }

    #[test]
    fn test_recommended_mitigations_not_empty() {
        let mut model = IcsThreatModel::new("Substation-D");
        model.asset_inventory.push(make_asset(
            1,
            ConnectivityLevel::FullyConnected,
            PatchStatus::Unsupported,
        ));
        model.known_threats.push(make_threat(0.4, 9));
        let mitigations = model.recommended_mitigations();
        assert!(!mitigations.is_empty());
    }

    // ── ThreatAnomalyDetector ─────────────────────────────────────────────────

    fn make_baseline(id: usize, mean: f64, std: f64) -> MeasurementBaseline {
        MeasurementBaseline {
            measurement_id: id,
            measurement_name: format!("Meas-{id}"),
            mean,
            std_dev: std,
            min_valid: mean - 5.0 * std,
            max_valid: mean + 5.0 * std,
            seasonal_means: vec![],
        }
    }

    #[test]
    fn test_no_alert_within_threshold() {
        let mut det = ThreatAnomalyDetector::new(3.0);
        det.add_baseline(make_baseline(1, 100.0, 5.0));
        // value within 1σ — should not trigger
        let alert = det.detect(1, 102.0, 1000.0, 10);
        assert!(alert.is_none(), "expected no alert for normal value");
    }

    #[test]
    fn test_alert_when_z_score_exceeds_threshold() {
        let mut det = ThreatAnomalyDetector::new(3.0);
        det.add_baseline(make_baseline(1, 100.0, 5.0));
        // value 5σ above mean → should trigger
        let alert = det.detect(1, 125.0, 2000.0, 10);
        assert!(alert.is_some(), "expected alert for extreme value");
        let a = alert.unwrap();
        assert!(a.z_score.abs() > 3.0);
    }

    #[test]
    fn test_update_baseline_adjusts_mean() {
        let mut det = ThreatAnomalyDetector::new(3.0);
        det.add_baseline(make_baseline(1, 100.0, 5.0));
        det.update_baseline(1, 200.0, 0.5); // alpha=0.5
        let new_mean = det.measurement_baselines[0].mean;
        // expected: 0.5*200 + 0.5*100 = 150
        assert!(
            (new_mean - 150.0).abs() < 1e-9,
            "EMA update incorrect: {new_mean}"
        );
    }

    #[test]
    fn test_false_data_injection_score_bounded() {
        let mut det = ThreatAnomalyDetector::new(3.0);
        det.add_baseline(make_baseline(1, 100.0, 5.0));
        det.add_baseline(make_baseline(2, 50.0, 2.0));

        // Injected values very far from baseline
        let score = det.false_data_injection_score(&[(1, 999.0), (2, -999.0)]);
        assert!(
            (0.0..=1.0).contains(&score),
            "FDI score out of bounds: {score}"
        );
        assert!(
            score > 0.5,
            "obvious attack should score above 0.5: {score}"
        );

        // Injected values close to baseline — low detectability
        let score2 = det.false_data_injection_score(&[(1, 100.1), (2, 50.1)]);
        assert!(
            (0.0..=1.0).contains(&score2),
            "FDI score out of bounds: {score2}"
        );
        assert!(
            score2 < 0.5,
            "subtle attack should score below 0.5: {score2}"
        );
    }

    // ── ScadaSecurityAssessment ───────────────────────────────────────────────

    fn make_scada_assessment() -> ScadaSecurityAssessment {
        ScadaSecurityAssessment {
            system_name: "TestSCADA".into(),
            components: vec![
                ScadaComponent {
                    id: 1,
                    name: "RTU-1".into(),
                    protocol: ScadaProtocol::Modbus,
                    encrypted: false,
                    authenticated: false,
                    last_audit_days: 200,
                },
                ScadaComponent {
                    id: 2,
                    name: "HMI-1".into(),
                    protocol: ScadaProtocol::OpcUa,
                    encrypted: true,
                    authenticated: true,
                    last_audit_days: 100,
                },
            ],
            network_topology: NetworkTopology {
                zones: 3,
                dmz_present: true,
                unidirectional_gateways: false,
            },
            security_controls: vec![
                ScadaSecurityControl {
                    control_type: ScadaControlType::Firewall,
                    implemented: true,
                    effectiveness: 0.8,
                },
                ScadaSecurityControl {
                    control_type: ScadaControlType::Mfa,
                    implemented: false,
                    effectiveness: 0.7,
                },
            ],
        }
    }

    #[test]
    fn test_security_score_bounded() {
        let assessment = make_scada_assessment();
        let score = assessment.security_score();
        assert!(
            (0.0..=100.0).contains(&score),
            "security score out of range: {score}"
        );
    }

    #[test]
    fn test_unencrypted_protocols_contains_modbus() {
        let assessment = make_scada_assessment();
        let unenc = assessment.unencrypted_protocols();
        assert!(
            !unenc.is_empty(),
            "should find at least one unencrypted component"
        );
        assert!(
            unenc.iter().any(|c| c.protocol == ScadaProtocol::Modbus),
            "Modbus component should appear in unencrypted list"
        );
    }

    // ── IncidentResponsePlaybook ──────────────────────────────────────────────

    #[test]
    fn test_ransomware_playbook_has_detect_phase() {
        let playbook = IncidentResponsePlaybook::for_ransomware();
        let detect_steps = playbook.steps_by_phase(&IncidentPhase::Detect);
        assert!(
            !detect_steps.is_empty(),
            "ransomware playbook must have Detect phase steps"
        );
    }

    #[test]
    fn test_total_response_time_positive() {
        let playbook = IncidentResponsePlaybook::for_ransomware();
        let hours = playbook.total_response_time_hours();
        assert!(
            hours > 0.0,
            "total response time should be positive: {hours}"
        );
        // Ransomware: max step = 7200 min → 120 h
        assert!((hours - 120.0).abs() < 1e-9, "expected 120 h, got {hours}");
    }

    #[test]
    fn test_fdi_playbook_has_contain_and_recover() {
        let playbook = IncidentResponsePlaybook::for_fdi();
        let contain = playbook.steps_by_phase(&IncidentPhase::Contain);
        let recover = playbook.steps_by_phase(&IncidentPhase::Recover);
        assert!(!contain.is_empty(), "FDI playbook must have Contain steps");
        assert!(!recover.is_empty(), "FDI playbook must have Recover steps");
    }

    // ── VulnerabilityScanner ──────────────────────────────────────────────────

    #[test]
    fn test_scan_asset_returns_relevant_vulns() {
        let scanner = VulnerabilityScanner::new_with_ics_database();
        let asset = make_asset(10, ConnectivityLevel::PartialDmz, PatchStatus::Unsupported);
        // asset uses Modbus and vendor "GenericPLC Corp" → should match CVE-2024-10001
        let results = scanner.scan_asset(&asset);
        assert!(
            !results.is_empty(),
            "should find at least one vulnerability for Modbus asset"
        );
        assert!(
            results.iter().any(|v| v.cve_id == "CVE-2024-10001"),
            "CVE-2024-10001 expected for Modbus / GenericPLC Corp"
        );
    }

    #[test]
    fn test_risk_priority_score_formula() {
        let scanner = VulnerabilityScanner::new_with_ics_database();
        // CVE-2024-10001: CVSS=9.8, exploit=true, patch=false → RPS = 9.8 * 2 * 2 = 39.2
        let vuln = &scanner.known_vulnerabilities[0];
        let rps = scanner.risk_priority_score(vuln);
        assert!(
            (rps - 39.2).abs() < 1e-9,
            "RPS should be 39.2 but got {rps}"
        );
    }

    // ── NercCipChecker ────────────────────────────────────────────────────────

    #[test]
    fn test_nerc_cip_overall_compliance_pct_bounded() {
        let checker = NercCipChecker::new_with_standards();
        let pct = checker.overall_compliance_pct();
        assert!(
            (0.0..=100.0).contains(&pct),
            "compliance pct out of range: {pct}"
        );
        // All standards start as not implemented.
        assert!(
            (pct - 0.0).abs() < 1e-9,
            "fresh checker should have 0% compliance"
        );
    }

    #[test]
    fn test_nerc_cip_non_compliant_standards_all_initially() {
        let checker = NercCipChecker::new_with_standards();
        let nc = checker.non_compliant_standards();
        assert_eq!(
            nc.len(),
            13,
            "all 13 standards should be non-compliant initially"
        );
    }

    #[test]
    fn test_nerc_cip_check_cip_005_fails_fully_connected() {
        let mut checker = NercCipChecker::new_with_standards();
        checker.bcs.push(make_asset(
            1,
            ConnectivityLevel::FullyConnected,
            PatchStatus::Current,
        ));
        checker.set_implemented("CIP-005", true);
        // Fully connected asset without DMZ → CIP-005 should fail
        assert!(
            !checker.check_cip_005(),
            "CIP-005 should fail for FullyConnected asset"
        );
    }

    #[test]
    fn test_nerc_cip_compliance_increases_when_implemented() {
        let mut checker = NercCipChecker::new_with_standards();
        checker.set_implemented("CIP-002", true);
        checker.set_implemented("CIP-007", true);
        let pct = checker.overall_compliance_pct();
        assert!(
            pct > 0.0,
            "compliance should increase after implementing standards: {pct}"
        );
        assert!(pct < 100.0, "not all standards are implemented yet: {pct}");
    }

    // ── 8 new tests ───────────────────────────────────────────────────────────

    #[test]
    fn test_iec62443_compliance_level_reflects_score() {
        // All components encrypted, authenticated, all controls implemented,
        // ≥3 zones, DMZ present → high score → SL 4.
        let assessment = ScadaSecurityAssessment {
            system_name: "HighSecurity".into(),
            components: vec![ScadaComponent {
                id: 1,
                name: "RTU-Secure".into(),
                protocol: ScadaProtocol::OpcUa,
                encrypted: true,
                authenticated: true,
                last_audit_days: 10,
            }],
            network_topology: NetworkTopology {
                zones: 4,
                dmz_present: true,
                unidirectional_gateways: true,
            },
            security_controls: vec![ScadaSecurityControl {
                control_type: ScadaControlType::Firewall,
                implemented: true,
                effectiveness: 1.0,
            }],
        };
        let sl = assessment.iec62443_compliance_level();
        assert_eq!(sl, 4, "high-security SCADA should achieve SL-4, got {sl}");
    }

    #[test]
    fn test_iec62443_compliance_level_low_for_bare_system() {
        let assessment = ScadaSecurityAssessment {
            system_name: "LowSecurity".into(),
            components: vec![ScadaComponent {
                id: 1,
                name: "OldRTU".into(),
                protocol: ScadaProtocol::Modbus,
                encrypted: false,
                authenticated: false,
                last_audit_days: 800,
            }],
            network_topology: NetworkTopology {
                zones: 1,
                dmz_present: false,
                unidirectional_gateways: false,
            },
            security_controls: vec![],
        };
        let sl = assessment.iec62443_compliance_level();
        assert!(
            sl <= 1,
            "bare SCADA system should be SL-0 or SL-1, got {sl}"
        );
    }

    #[test]
    fn test_vulnerability_count_includes_modbus_components() {
        let assessment = make_scada_assessment();
        // make_scada_assessment: 1 Modbus unauth+unenc, 1 OpcUa enc+auth
        // vulnerability_count includes: unencrypted OR unauthenticated OR legacy
        let count = assessment.vulnerability_count();
        // RTU-1 is Modbus + unencrypted + unauthenticated → counted
        // HMI-1 is OpcUa + encrypted + authenticated → NOT counted
        assert_eq!(count, 1, "only RTU-1 should be flagged, got {count}");
    }

    #[test]
    fn test_recommended_actions_includes_tls_advice() {
        let assessment = make_scada_assessment();
        let actions = assessment.recommended_actions();
        assert!(!actions.is_empty(), "should produce at least one action");
        let has_tls = actions
            .iter()
            .any(|a| a.contains("TLS") || a.contains("IPSec"));
        assert!(
            has_tls,
            "should recommend enabling TLS/IPSec for unencrypted components"
        );
    }

    #[test]
    fn test_recent_alerts_filters_by_window() {
        let mut det = ThreatAnomalyDetector::new(3.0);
        det.add_baseline(make_baseline(1, 100.0, 5.0));
        // Generate an alert at t = 1000.0
        det.detect(1, 125.0, 1000.0, 10);
        // Generate an alert at t = 2000.0
        det.detect(1, 125.0, 2000.0, 10);

        // Window from t=1500 to t=2500 — only the second alert should appear.
        let recent = det.recent_alerts(1000.0, 2500.0);
        assert_eq!(
            recent.len(),
            1,
            "should have exactly 1 recent alert in window, got {}",
            recent.len()
        );
        assert!(
            (recent[0].timestamp - 2000.0).abs() < 1e-9,
            "recent alert timestamp should be 2000, got {}",
            recent[0].timestamp
        );
    }

    #[test]
    fn test_has_native_encryption_only_opcua() {
        assert!(
            ScadaProtocol::OpcUa.has_native_encryption(),
            "OpcUa should have native encryption"
        );
        assert!(
            !ScadaProtocol::Modbus.has_native_encryption(),
            "Modbus should NOT have native encryption"
        );
        assert!(
            !ScadaProtocol::Dnp3.has_native_encryption(),
            "Dnp3 should NOT have native encryption"
        );
    }

    #[test]
    fn test_has_native_authentication_opcua_and_dnp3() {
        assert!(
            ScadaProtocol::OpcUa.has_native_authentication(),
            "OpcUa should have native authentication"
        );
        assert!(
            ScadaProtocol::Dnp3.has_native_authentication(),
            "Dnp3 should have native authentication (SAv5)"
        );
        assert!(
            !ScadaProtocol::Modbus.has_native_authentication(),
            "Modbus should NOT have native authentication"
        );
    }

    #[test]
    fn test_generate_report_counts_vulnerable_assets_and_critical_vulns() {
        let scanner = VulnerabilityScanner::new_with_ics_database();
        // Asset using Modbus (CVE-2024-10001: CVSS=9.8 critical, no patch)
        let asset_modbus = make_asset(1, ConnectivityLevel::PartialDmz, PatchStatus::Unsupported);
        let report = scanner.generate_report(&[asset_modbus]);
        assert_eq!(
            report.total_assets, 1,
            "total_assets should be 1, got {}",
            report.total_assets
        );
        assert!(
            report.vulnerable_assets >= 1,
            "at least 1 vulnerable asset expected, got {}",
            report.vulnerable_assets
        );
        assert!(
            report.critical_vulns >= 1,
            "at least 1 critical vuln expected (CVE-2024-10001 CVSS=9.8), got {}",
            report.critical_vulns
        );
        assert!(
            report.overall_risk_score > 0.0,
            "overall_risk_score should be > 0.0, got {}",
            report.overall_risk_score
        );
    }
}
