//! Advanced protection scheme for Inverter-Based Resource (IBR) dominated grids.
//!
//! Addresses challenges including:
//! - Reduced fault current magnitude (IBR typically 1.1–2.0× rated vs 5–10× for synchronous)
//! - Variable fault current infeed causing distance relay reach errors
//! - Negative sequence current differences (IBR may suppress I2)
//! - Loss-of-mains (LOM) / islanding detection
//!
//! # References
//! - IEEE 2800-2022 — Interconnection standards for IBR
//! - IEC 60909 — Short-circuit current calculations
//! - NERC PRC-024 — Frequency and voltage ride-through
//! - ENTSO-E RfG (Requirement for Generators) NC

use serde::{Deserialize, Serialize};

// ─── IBR unit types ──────────────────────────────────────────────────────────

/// Classification of inverter-based resource technology.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum IbrType {
    /// Photovoltaic solar plant.
    SolarPv {
        /// Installed AC capacity \[MW\]
        capacity_mw: f64,
        /// Inverter control mode (e.g. "grid-following", "grid-forming")
        control_mode: String,
    },
    /// Type 3 / Type 4 wind turbine generator.
    WindTurbine {
        /// Installed AC capacity \[MW\]
        capacity_mw: f64,
    },
    /// Battery energy storage system.
    BatteryStorage {
        /// Power rating \[MW\]
        capacity_mw: f64,
        /// Energy capacity \[MWh\]
        capacity_mwh: f64,
    },
    /// High-voltage DC link (LCC or VSC).
    Hvdc {
        /// Converter rated power \[MW\]
        capacity_mw: f64,
    },
    /// Co-located hybrid plant (PV + Wind + BESS).
    Hybrid {
        /// PV component \[MW\]
        mw_pv: f64,
        /// Wind component \[MW\]
        mw_wind: f64,
        /// BESS component \[MW\]
        mw_bess: f64,
    },
}

impl IbrType {
    /// Total rated capacity \[MW\].
    pub fn capacity_mw(&self) -> f64 {
        match self {
            IbrType::SolarPv { capacity_mw, .. } => *capacity_mw,
            IbrType::WindTurbine { capacity_mw } => *capacity_mw,
            IbrType::BatteryStorage { capacity_mw, .. } => *capacity_mw,
            IbrType::Hvdc { capacity_mw } => *capacity_mw,
            IbrType::Hybrid {
                mw_pv,
                mw_wind,
                mw_bess,
            } => mw_pv + mw_wind + mw_bess,
        }
    }
}

// ─── IBR unit ────────────────────────────────────────────────────────────────

/// Single inverter-based resource unit connected to the grid.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IbrUnit {
    /// Unique identifier for this IBR unit.
    pub unit_id: String,
    /// Bus index to which the IBR is connected.
    pub bus_id: usize,
    /// Technology type.
    pub ibr_type: IbrType,
    /// Maximum fault current contribution in per-unit on IBR rating base \[pu\].
    ///
    /// Typical: 1.1–2.0 pu (grid-following) or up to 2.0 pu (grid-forming).
    pub fault_current_pu: f64,
    /// Whether the IBR can actively inject negative-sequence current (I2).
    pub negative_seq_capability: bool,
    /// Whether the IBR provides fast frequency response (FFR).
    pub fast_frequency_response: bool,
    /// Whether LVRT (Low-Voltage Ride-Through) is enabled.
    pub lvrt_capability: bool,
}

// ─── Adaptive distance relay ─────────────────────────────────────────────────

/// Mho/quadrilateral distance relay with IBR-aware reach correction.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdaptiveDistanceRelay {
    /// Relay identifier string.
    pub relay_id: String,
    /// Index of the protected branch in the network branch list.
    pub protected_branch_idx: usize,
    /// Zone 1 reach \[pu impedance\] — typically 80 % of line.
    pub zone1_reach_pu: f64,
    /// Zone 2 reach \[pu impedance\] — typically 120 % of next line section.
    pub zone2_reach_pu: f64,
    /// Zone 3 reach \[pu impedance\] — remote backup.
    pub zone3_reach_pu: f64,
    /// Enable IBR infeed correction algorithm.
    pub ibr_infeed_correction: bool,
    /// Enable negative-sequence supervision element.
    pub negative_seq_supervision: bool,
}

// ─── Loss-of-mains detection ─────────────────────────────────────────────────

/// Algorithm used by the LOM detector.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum LomMethod {
    /// Rate-of-change-of-frequency (ROCOF) — df/dt method.
    Rocof,
    /// Vector shift / phase angle jump method.
    VectorShift,
    /// Under/over frequency threshold crossings.
    UnderOverFreq,
    /// Combined — any single method triggering declares LOM.
    Combined,
}

/// Loss-of-mains (islanding) detector configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LomDetector {
    /// Detection algorithm.
    pub method: LomMethod,
    /// ROCOF trip threshold \[Hz/s\].
    pub rate_of_change_hz_per_s: f64,
    /// Vector-shift trip threshold \[deg\].
    pub vector_shift_deg: f64,
    /// Minimum permissible frequency \[Hz\].
    pub freq_min_hz: f64,
    /// Maximum permissible frequency \[Hz\].
    pub freq_max_hz: f64,
}

// ─── Scheme configuration ─────────────────────────────────────────────────────

/// Overall configuration for an IBR-dominated grid protection scheme.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IbrProtectionConfig {
    /// Fraction of total generation supplied by IBR \[%\].
    pub ibr_penetration_pct: f64,
    /// Expected minimum fault current under worst-case IBR scenario \[pu\].
    pub min_fault_current_pu: f64,
    /// Use negative-sequence quantities for fault detection/supervision.
    pub use_negative_sequence: bool,
    /// Use communication-aided (pilot) protection schemes.
    pub communication_aided: bool,
    /// Enable directional element supervision.
    pub directional_element: bool,
}

/// Top-level IBR protection scheme aggregating all components.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IbrProtectionScheme {
    /// All IBR units in the study area.
    pub ibr_units: Vec<IbrUnit>,
    /// Adaptive distance/overcurrent relays.
    pub relays: Vec<AdaptiveDistanceRelay>,
    /// Loss-of-mains detectors (one per point-of-common-coupling).
    pub lom_detectors: Vec<LomDetector>,
    /// Scheme-wide protection configuration.
    pub config: IbrProtectionConfig,
}

// ─── Output types ─────────────────────────────────────────────────────────────

/// Result of fault-current adequacy assessment for an IBR-dominated bus.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FaultCurrentAssessment {
    /// Total fault current (IBR + synchronous) \[pu\].
    pub total_fault_current_pu: f64,
    /// IBR contribution to fault current \[pu\].
    pub ibr_contribution_pu: f64,
    /// Synchronous machine contribution \[pu\].
    pub sync_contribution_pu: f64,
    /// Whether total fault current is sufficient for overcurrent relay operation.
    ///
    /// Conventional OC relays typically require ≥ 1.5× pickup (set at ~1.2 pu load).
    pub adequate_for_overcurrent: bool,
    /// Recommended minimum relay setting \[pu\] considering measurement uncertainty.
    pub min_relay_setting_pu: f64,
}

/// Result of a loss-of-mains detection analysis.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LomResult {
    /// True if loss-of-mains condition is detected.
    pub lom_detected: bool,
    /// Computed ROCOF value \[Hz/s\].
    pub rocof_hz_per_s: f64,
    /// Computed vector shift \[deg\].
    pub vector_shift_deg: f64,
    /// Name of the method that triggered (empty if not detected).
    pub triggering_method: String,
    /// Confidence level \[0–1\] in the LOM declaration.
    pub confidence: f64,
}

/// Directional classification of a detected fault.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum FaultDirection {
    /// Fault is in the forward (protected) direction.
    Forward,
    /// Fault is in the reverse direction.
    Reverse,
    /// Direction cannot be determined (insufficient signal).
    Unknown,
}

/// Reliability metrics for a protection scheme.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProtectionReliability {
    /// Probability of correct operation when a fault occurs \[%\].
    pub dependability_pct: f64,
    /// Probability of not tripping when no fault exists \[%\].
    pub security_pct: f64,
    /// True if both dependability and security exceed 99.9 %.
    pub adequate: bool,
    /// Human-readable recommendation.
    pub recommendation: String,
}

// ─── Core algorithms ──────────────────────────────────────────────────────────

/// Assess fault current adequacy for a specific fault-location bus.
///
/// # Arguments
/// * `ibr_units`            — Slice of IBR units in the network
/// * `fault_location_bus`   — Bus index where fault is applied
/// * `sync_contribution_pu` — Synchronous machine fault current contribution \[pu\]
///
/// # Returns
/// `FaultCurrentAssessment` with adequacy judgement.
pub fn assess_fault_current(
    ibr_units: &[IbrUnit],
    fault_location_bus: usize,
    sync_contribution_pu: f64,
) -> FaultCurrentAssessment {
    // Sum IBR contributions from electrically adjacent / connected units.
    // In a simplified study each IBR unit feeds into the fault at its bus;
    // units at the same bus contribute their full fault_current_pu.
    let ibr_contribution_pu: f64 = ibr_units
        .iter()
        .filter(|u| u.bus_id == fault_location_bus)
        .map(|u| u.fault_current_pu)
        .sum();

    let total_fault_current_pu = ibr_contribution_pu + sync_contribution_pu;

    // Conventional overcurrent relays typically need ≥ 1.5× their pickup.
    // A typical pickup is set at 1.2 × maximum load current.
    // For a 1 pu base, the relay pickup threshold is ~1.2 pu,
    // and we require at least 1.5× that → 1.8 pu.
    const OC_ADEQUACY_THRESHOLD_PU: f64 = 1.8;
    let adequate_for_overcurrent = total_fault_current_pu >= OC_ADEQUACY_THRESHOLD_PU;

    // Recommended relay setting: leave 20 % security margin below minimum fault
    // current to avoid false operation on load swings.
    let min_relay_setting_pu = total_fault_current_pu * 0.80;

    FaultCurrentAssessment {
        total_fault_current_pu,
        ibr_contribution_pu,
        sync_contribution_pu,
        adequate_for_overcurrent,
        min_relay_setting_pu,
    }
}

/// Correct distance relay reach for IBR infeed effect.
///
/// When an IBR feeds into the fault point from behind the relay, the voltage
/// at the relay terminal is elevated by the IBR contribution, making the
/// apparent impedance **larger** than the true fault impedance.
///
/// The corrected (effective) reach is:
/// ```text
/// Z_apparent = Z_actual × (I_relay + I_ibr) / I_relay
/// adjusted_reach = original_reach / correction_factor
/// ```
///
/// # Arguments
/// * `relay`             — Distance relay under assessment
/// * `ibr_infeed_current` — IBR infeed current magnitude \[pu\] at fault point
/// * `relay_current`     — Relay measured current magnitude \[pu\]
///
/// # Returns
/// Adjusted zone-1 reach \[pu\] accounting for IBR infeed dilution.
pub fn correct_distance_reach(
    relay: &AdaptiveDistanceRelay,
    ibr_infeed_current: f64,
    relay_current: f64,
) -> f64 {
    if !relay.ibr_infeed_correction || relay_current <= 0.0 {
        return relay.zone1_reach_pu;
    }

    // Correction factor > 1 means relay under-reaches without correction.
    let correction_factor = (relay_current + ibr_infeed_current) / relay_current;

    // Divide original reach by correction factor to get effective reach.
    // This "pre-compensates" the relay so it correctly trips for the true
    // impedance despite the apparent under-reach.
    let adjusted_reach = relay.zone1_reach_pu / correction_factor;

    // Clamp to a minimum of 50 % of original reach to prevent over-correction.
    adjusted_reach.max(relay.zone1_reach_pu * 0.50)
}

/// Detect loss-of-mains (islanding) condition.
///
/// # Arguments
/// * `freq_hz`        — Current frequency measurement \[Hz\]
/// * `freq_history`   — Slice of recent frequency samples, oldest first \[Hz\]
/// * `angle_deg`      — Current voltage phase angle \[deg\]
/// * `angle_history`  — Slice of recent angle samples, oldest first \[deg\]
/// * `detector`       — LOM detector configuration
/// * `sample_period_s` — Time between consecutive samples \[s\]
///
/// # Returns
/// `LomResult` with detection flag and diagnostic values.
pub fn detect_loss_of_mains(
    freq_hz: f64,
    freq_history: &[f64],
    angle_deg: f64,
    angle_history: &[f64],
    detector: &LomDetector,
    sample_period_s: f64,
) -> LomResult {
    // ── ROCOF computation ─────────────────────────────────────────────────────
    // Use a 100 ms window: compute df/dt over the available history.
    let rocof_hz_per_s = compute_rocof(freq_history, freq_hz, sample_period_s);

    // ── Vector shift computation ──────────────────────────────────────────────
    let vector_shift = compute_vector_shift(angle_history, angle_deg);

    // ── Under/over-frequency check ────────────────────────────────────────────
    let under_over_freq = freq_hz < detector.freq_min_hz || freq_hz > detector.freq_max_hz;

    // ── Method evaluation ─────────────────────────────────────────────────────
    let rocof_triggered = rocof_hz_per_s.abs() > detector.rate_of_change_hz_per_s;
    let vs_triggered = vector_shift.abs() > detector.vector_shift_deg;
    let uof_triggered = under_over_freq;

    let (lom_detected, triggering_method, confidence) = match &detector.method {
        LomMethod::Rocof => {
            let triggered = rocof_triggered;
            let conf = if triggered {
                (rocof_hz_per_s.abs() / detector.rate_of_change_hz_per_s).min(1.0)
            } else {
                0.0
            };
            (
                triggered,
                if triggered {
                    "ROCOF".to_string()
                } else {
                    String::new()
                },
                conf,
            )
        }
        LomMethod::VectorShift => {
            let triggered = vs_triggered;
            let conf = if triggered {
                (vector_shift.abs() / detector.vector_shift_deg).min(1.0)
            } else {
                0.0
            };
            (
                triggered,
                if triggered {
                    "VectorShift".to_string()
                } else {
                    String::new()
                },
                conf,
            )
        }
        LomMethod::UnderOverFreq => {
            let triggered = uof_triggered;
            let conf = if triggered { 0.95 } else { 0.0 };
            (
                triggered,
                if triggered {
                    "UnderOverFreq".to_string()
                } else {
                    String::new()
                },
                conf,
            )
        }
        LomMethod::Combined => {
            // Any method triggers → LOM declared; confidence accumulates.
            let mut methods: Vec<&str> = Vec::new();
            let mut conf = 0.0_f64;

            if rocof_triggered {
                methods.push("ROCOF");
                conf = conf.max((rocof_hz_per_s.abs() / detector.rate_of_change_hz_per_s).min(1.0));
            }
            if vs_triggered {
                methods.push("VectorShift");
                conf = conf.max((vector_shift.abs() / detector.vector_shift_deg).min(1.0));
            }
            if uof_triggered {
                methods.push("UnderOverFreq");
                conf = conf.max(0.95);
            }
            let triggered = !methods.is_empty();
            let trig_str = if triggered {
                methods.join("+")
            } else {
                String::new()
            };
            (triggered, trig_str, conf)
        }
    };

    LomResult {
        lom_detected,
        rocof_hz_per_s,
        vector_shift_deg: vector_shift,
        triggering_method,
        confidence,
    }
}

/// Compute ROCOF (df/dt) over a sliding window of frequency samples \[Hz/s\].
fn compute_rocof(freq_history: &[f64], current_freq: f64, sample_period_s: f64) -> f64 {
    if freq_history.is_empty() || sample_period_s <= 0.0 {
        return 0.0;
    }
    // Use the oldest available sample for the gradient.
    let oldest = *freq_history.first().unwrap_or(&current_freq);
    let n_steps = freq_history.len() as f64; // number of intervals back
    let time_window_s = n_steps * sample_period_s;
    if time_window_s <= 0.0 {
        return 0.0;
    }
    (current_freq - oldest) / time_window_s
}

/// Compute voltage vector shift between consecutive cycles \[deg\].
fn compute_vector_shift(angle_history: &[f64], current_angle: f64) -> f64 {
    if let Some(&prev) = angle_history.last() {
        // Wrap into (−180, 180].
        let diff = current_angle - prev;
        wrap_angle_deg(diff)
    } else {
        0.0
    }
}

/// Wrap an angle difference into the range (−180, 180\] degrees.
fn wrap_angle_deg(diff: f64) -> f64 {
    let mut d = diff % 360.0;
    if d > 180.0 {
        d -= 360.0;
    } else if d <= -180.0 {
        d += 360.0;
    }
    d
}

/// Negative-sequence supervision element for IBR-dominated grids.
///
/// Conventional overcurrent relays lose sensitivity with low IBR fault currents.
/// Negative-sequence quantities remain fault-selective even at low I1 levels.
///
/// Thresholds used:
/// - |I2| > 0.10 pu → fault indicated (absolute threshold)
/// - |I2/I1| > 0.20 → fault indicated (ratio threshold for unbalanced faults)
/// - |V2| > 0.05 pu → voltage-based confirmation
///
/// # Arguments
/// * `i1_pu` — Positive-sequence current magnitude \[pu\]
/// * `i2_pu` — Negative-sequence current magnitude \[pu\]
/// * `v2_pu` — Negative-sequence voltage magnitude \[pu\]
///
/// # Returns
/// `true` if a fault condition is indicated.
pub fn negative_sequence_supervision(i1_pu: f64, i2_pu: f64, v2_pu: f64) -> bool {
    const I2_ABS_THRESHOLD: f64 = 0.10; // 10 % of base current
    const I2_I1_RATIO_THRESHOLD: f64 = 0.20; // 20 % of positive sequence
    const V2_THRESHOLD: f64 = 0.05; // 5 % of base voltage

    let abs_fault = i2_pu > I2_ABS_THRESHOLD;
    let ratio_fault = if i1_pu > 1e-6 {
        (i2_pu / i1_pu) > I2_I1_RATIO_THRESHOLD
    } else {
        false
    };
    let voltage_confirm = v2_pu > V2_THRESHOLD;

    (abs_fault || ratio_fault) && voltage_confirm
}

/// Directional element using symmetrical components.
///
/// For a relay at voltage `v_pu` measuring current `i_pu` on a line with
/// characteristic angle `line_angle_deg` (typically 60–85° for overhead lines),
/// the torque angle is compared to the expected forward/reverse signatures.
///
/// Forward fault: the angle of (V × I*) is within ±90° of the line angle.
///
/// Also uses negative-sequence quantities for enhanced security:
/// The negative-sequence directional element uses the relationship
/// –V2 leads I2 for forward faults.
///
/// # Arguments
/// * `v_pu`            — Voltage phasor magnitude \[pu\] at relay terminal
/// * `i_pu`            — Current phasor magnitude \[pu\]
/// * `line_angle_deg`  — Characteristic (maximum torque) angle of the line \[deg\]
/// * `fault_angle_deg` — Measured impedance angle of fault trajectory \[deg\]
///
/// # Returns
/// `FaultDirection` classification.
pub fn directional_element(
    v_pu: f64,
    i_pu: f64,
    line_angle_deg: f64,
    fault_angle_deg: f64,
) -> FaultDirection {
    const MIN_CURRENT_PU: f64 = 0.05;
    const MIN_VOLTAGE_PU: f64 = 0.02;

    if i_pu < MIN_CURRENT_PU || v_pu < MIN_VOLTAGE_PU {
        return FaultDirection::Unknown;
    }

    // Angle difference between fault trajectory and line characteristic angle.
    let angle_diff = wrap_angle_deg(fault_angle_deg - line_angle_deg);

    // Forward: angle difference within ±90° (positive torque region).
    if angle_diff.abs() <= 90.0 {
        FaultDirection::Forward
    } else if angle_diff.abs() > 90.0 {
        FaultDirection::Reverse
    } else {
        FaultDirection::Unknown
    }
}

/// Rate the reliability of a protection scheme against IEEE/CIGRÉ benchmarks.
///
/// Dependability and security are estimated from the IBR penetration level
/// and the scheme features enabled (negative sequence, pilot, directional).
///
/// Reference: IEEE PSRC WG I26 — Protection challenges for IBR.
///
/// # Arguments
/// * `scheme` — IBR protection scheme to evaluate
///
/// # Returns
/// `ProtectionReliability` with dependability and security estimates.
pub fn rate_protection_reliability(scheme: &IbrProtectionScheme) -> ProtectionReliability {
    let pct = scheme.config.ibr_penetration_pct;

    // Baseline dependability for conventional OC/distance at high IBR penetration.
    // At 0 % IBR: ~99.95 %; at 100 % IBR with only OC: can drop to ~90 %.
    let base_dependability = if pct <= 30.0 {
        99.95
    } else if pct <= 50.0 {
        99.5 - (pct - 30.0) * 0.20 // degrades with penetration
    } else if pct <= 70.0 {
        95.5 - (pct - 50.0) * 0.25
    } else {
        90.5 - (pct - 70.0) * 0.25
    };

    // Feature upgrades improve dependability.
    let mut dependability = base_dependability;
    if scheme.config.use_negative_sequence {
        dependability += 1.5;
    }
    if scheme.config.communication_aided {
        dependability += 2.5;
    }
    if scheme.config.directional_element {
        dependability += 0.5;
    }
    dependability = dependability.min(99.99);

    // Security degrades slightly with aggressive settings needed for IBR.
    let mut security = if pct <= 50.0 {
        99.95
    } else {
        99.95 - (pct - 50.0) * 0.02
    };
    // Communication-aided pilot schemes have excellent security.
    if scheme.config.communication_aided {
        security = security.max(99.9);
    }
    security = security.min(99.99);

    let adequate = dependability >= 99.9 && security >= 99.9;

    let recommendation = build_reliability_recommendation(dependability, security, pct, scheme);

    ProtectionReliability {
        dependability_pct: dependability,
        security_pct: security,
        adequate,
        recommendation,
    }
}

fn build_reliability_recommendation(
    dependability_pct: f64,
    security_pct: f64,
    ibr_pct: f64,
    scheme: &IbrProtectionScheme,
) -> String {
    let mut parts: Vec<String> = Vec::new();

    if dependability_pct < 99.9 {
        parts.push(format!(
            "Dependability {:.2}% below 99.9% target — improve detection sensitivity.",
            dependability_pct
        ));
    }
    if security_pct < 99.9 {
        parts.push(format!(
            "Security {:.2}% below 99.9% target — review relay settings.",
            security_pct
        ));
    }
    if ibr_pct > 30.0 && !scheme.config.use_negative_sequence {
        parts.push("Enable negative-sequence supervision for IBR penetration > 30%.".to_string());
    }
    if ibr_pct > 50.0 && !scheme.config.communication_aided {
        parts.push(
            "Add pilot/communication-aided protection for IBR penetration > 50%.".to_string(),
        );
    }
    if parts.is_empty() {
        "Protection scheme meets reliability targets for current IBR penetration.".to_string()
    } else {
        parts.join(" ")
    }
}

/// Recommend protection upgrades based on IBR penetration level.
///
/// Upgrade thresholds aligned with IEEE 2800 and NERC PRC guidance:
/// - > 30 %: negative-sequence elements required
/// - > 50 %: pilot/communication-aided protection required
/// - > 70 %: grid-forming inverter capabilities recommended for protection support
///
/// # Arguments
/// * `current_scheme`       — Existing scheme configuration
/// * `ibr_penetration_pct`  — Projected or actual IBR penetration \[%\]
///
/// # Returns
/// List of actionable upgrade recommendations.
pub fn recommend_protection_upgrades(
    current_scheme: &IbrProtectionScheme,
    ibr_penetration_pct: f64,
) -> Vec<String> {
    let mut recommendations: Vec<String> = Vec::new();

    // ── Tier 1: > 30 % ────────────────────────────────────────────────────────
    if ibr_penetration_pct > 30.0 {
        if !current_scheme.config.use_negative_sequence {
            recommendations.push(
                "Add negative-sequence (I2/V2) supervision elements: \
                 conventional overcurrent relays lose sensitivity at >30% IBR penetration."
                    .to_string(),
            );
        }
        recommendations.push(
            "Review overcurrent relay pickup settings: minimum fault current may have \
             decreased due to IBR displacement of synchronous machines."
                .to_string(),
        );
        recommendations.push(
            "Deploy adaptive distance relay reach correction for IBR infeed compensation."
                .to_string(),
        );
    }

    // ── Tier 2: > 50 % ────────────────────────────────────────────────────────
    if ibr_penetration_pct > 50.0 {
        if !current_scheme.config.communication_aided {
            recommendations.push(
                "Implement pilot/communication-aided protection (POTT or DUTT scheme): \
                 overcurrent and distance relays alone are insufficient at >50% IBR penetration."
                    .to_string(),
            );
        }
        recommendations.push(
            "Install synchrophasor-based wide-area protection for islanding detection \
             and system integrity protection."
                .to_string(),
        );
        recommendations.push(
            "Configure ROCOF and vector-shift LOM detectors at every IBR point-of-connection: \
             loss-of-mains risk increases with reduced synchronous inertia."
                .to_string(),
        );
    }

    // ── Tier 3: > 70 % ────────────────────────────────────────────────────────
    if ibr_penetration_pct > 70.0 {
        recommendations.push(
            "Mandate grid-forming inverter control for large IBR units (>10 MW): \
             grid-forming converters emulate synchronous inertia and provide higher \
             fault current (up to 2 pu) to support protection operation."
                .to_string(),
        );
        recommendations.push(
            "Consider virtual inertia / synthetic inertia emulation to maintain ROCOF \
             below protection thresholds during grid disturbances."
                .to_string(),
        );
        recommendations.push(
            "Implement centralized protection coordination using real-time IBR \
             dispatch data to adaptively update relay settings."
                .to_string(),
        );
    }

    // ── General ────────────────────────────────────────────────────────────────
    if ibr_penetration_pct > 20.0 && !current_scheme.config.directional_element {
        recommendations.push(
            "Enable directional elements on all distance/overcurrent relays: \
             bidirectional power flow from IBR can cause maloperation without directional supervision."
                .to_string(),
        );
    }

    let lom_count = current_scheme.lom_detectors.len();
    let ibr_count = current_scheme.ibr_units.len();
    if ibr_penetration_pct > 25.0 && lom_count < ibr_count {
        recommendations.push(format!(
            "Deploy LOM detectors at all {} IBR connection points (currently {} configured).",
            ibr_count, lom_count
        ));
    }

    recommendations
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_ibr_unit(bus_id: usize, fault_current_pu: f64) -> IbrUnit {
        IbrUnit {
            unit_id: format!("IBR-{}", bus_id),
            bus_id,
            ibr_type: IbrType::SolarPv {
                capacity_mw: 50.0,
                control_mode: "grid-following".to_string(),
            },
            fault_current_pu,
            negative_seq_capability: false,
            fast_frequency_response: false,
            lvrt_capability: true,
        }
    }

    fn make_relay(zone1: f64) -> AdaptiveDistanceRelay {
        AdaptiveDistanceRelay {
            relay_id: "R1".to_string(),
            protected_branch_idx: 0,
            zone1_reach_pu: zone1,
            zone2_reach_pu: zone1 * 1.5,
            zone3_reach_pu: zone1 * 2.5,
            ibr_infeed_correction: true,
            negative_seq_supervision: true,
        }
    }

    fn make_lom_detector(method: LomMethod) -> LomDetector {
        LomDetector {
            method,
            rate_of_change_hz_per_s: 1.0,
            vector_shift_deg: 12.0,
            freq_min_hz: 49.0,
            freq_max_hz: 51.0,
        }
    }

    fn make_scheme(
        penetration_pct: f64,
        use_neg_seq: bool,
        comm_aided: bool,
    ) -> IbrProtectionScheme {
        IbrProtectionScheme {
            ibr_units: vec![make_ibr_unit(0, 1.5)],
            relays: vec![make_relay(0.8)],
            lom_detectors: vec![make_lom_detector(LomMethod::Combined)],
            config: IbrProtectionConfig {
                ibr_penetration_pct: penetration_pct,
                min_fault_current_pu: 1.5,
                use_negative_sequence: use_neg_seq,
                communication_aided: comm_aided,
                directional_element: true,
            },
        }
    }

    // ── Test 1: 100% IBR → fault current < 2× rated, inadequate for OC ───────
    #[test]
    fn test_fault_current_all_ibr_inadequate() {
        // Two IBR units each contributing 0.8 pu → total 1.6 pu, no sync machines
        let units = vec![make_ibr_unit(0, 0.8), make_ibr_unit(0, 0.8)];
        let assessment = assess_fault_current(&units, 0, 0.0);

        assert!(
            assessment.total_fault_current_pu < 2.0,
            "Expected < 2.0 pu total, got {}",
            assessment.total_fault_current_pu
        );
        assert!(
            !assessment.adequate_for_overcurrent,
            "Should be inadequate for OC at 1.6 pu"
        );
        assert_eq!(assessment.ibr_contribution_pu, 1.6);
        assert_eq!(assessment.sync_contribution_pu, 0.0);
    }

    // ── Test 2: Distance relay IBR infeed correction ───────────────────────────
    #[test]
    fn test_distance_reach_correction_ibr_infeed() {
        let relay = make_relay(0.8);
        // IBR infeed of 0.5 pu with relay current of 1.0 pu
        // correction_factor = (1.0 + 0.5)/1.0 = 1.5
        // adjusted_reach = 0.8 / 1.5 ≈ 0.533 pu
        let adjusted = correct_distance_reach(&relay, 0.5, 1.0);
        assert!(
            adjusted < relay.zone1_reach_pu,
            "Adjusted reach {} should be less than original {}",
            adjusted,
            relay.zone1_reach_pu
        );
        let expected = 0.8 / 1.5;
        assert!(
            (adjusted - expected).abs() < 1e-9,
            "Expected {:.4}, got {:.4}",
            expected,
            adjusted
        );
    }

    // ── Test 3: LOM ROCOF — df/dt = 2 Hz/s > 1 Hz/s threshold ────────────────
    #[test]
    fn test_lom_rocof_detected() {
        let detector = make_lom_detector(LomMethod::Rocof);
        // History: 50.0 Hz 0.5 s ago; current 51.0 Hz → df/dt = 2 Hz/s
        let freq_history = vec![50.0];
        let result = detect_loss_of_mains(51.0, &freq_history, 0.0, &[], &detector, 0.5);

        assert!(result.lom_detected, "ROCOF LOM should be detected");
        assert!(
            result.rocof_hz_per_s.abs() > 1.0,
            "ROCOF should exceed threshold, got {}",
            result.rocof_hz_per_s
        );
        assert_eq!(result.triggering_method, "ROCOF");
    }

    // ── Test 4: LOM vector shift — 15° jump > 12° threshold ──────────────────
    #[test]
    fn test_lom_vector_shift_detected() {
        let detector = make_lom_detector(LomMethod::VectorShift);
        let angle_history = vec![0.0]; // previous angle
        let result = detect_loss_of_mains(
            50.0,
            &[50.0],
            15.0, // current angle: 15° jump
            &angle_history,
            &detector,
            0.02,
        );

        assert!(result.lom_detected, "Vector shift LOM should be detected");
        assert!(
            result.vector_shift_deg.abs() > 12.0,
            "Vector shift {} should exceed 12°",
            result.vector_shift_deg
        );
        assert_eq!(result.triggering_method, "VectorShift");
    }

    // ── Test 5: Negative sequence supervision — I2 > threshold ───────────────
    #[test]
    fn test_negative_sequence_supervision_fault_indicated() {
        // I2 = 0.15 pu (> 0.10 threshold), I1 = 0.8 pu, V2 = 0.08 pu (> 0.05)
        let fault_indicated = negative_sequence_supervision(0.8, 0.15, 0.08);
        assert!(
            fault_indicated,
            "Fault should be indicated with I2=0.15, V2=0.08"
        );
    }

    #[test]
    fn test_negative_sequence_supervision_no_fault() {
        // I2 = 0.02 pu (< threshold), V2 = 0.02 pu
        let fault_indicated = negative_sequence_supervision(1.0, 0.02, 0.02);
        assert!(!fault_indicated, "No fault should be indicated at low I2");
    }

    // ── Test 6: Directional element — forward fault angle ────────────────────
    #[test]
    fn test_directional_forward_fault() {
        // Line angle = 75°, fault angle = 72° → angle_diff = -3° → forward
        let direction = directional_element(0.9, 2.0, 75.0, 72.0);
        assert_eq!(
            direction,
            FaultDirection::Forward,
            "Expected Forward for fault angle near line angle"
        );
    }

    #[test]
    fn test_directional_reverse_fault() {
        // Line angle = 75°, fault angle = -120° → angle_diff = -195° → outside +/-90° → reverse
        let direction = directional_element(0.9, 2.0, 75.0, -120.0);
        assert_eq!(
            direction,
            FaultDirection::Reverse,
            "Expected Reverse for fault angle 180° from line angle"
        );
    }

    // ── Test 7: Reliability — low IBR penetration → high dependability ────────
    #[test]
    fn test_reliability_low_ibr_penetration_adequate() {
        let scheme = make_scheme(20.0, false, false);
        let reliability = rate_protection_reliability(&scheme);

        assert!(
            reliability.dependability_pct > 99.9,
            "Low IBR penetration should give >99.9% dependability, got {}",
            reliability.dependability_pct
        );
        assert!(
            reliability.adequate,
            "Scheme should be adequate at 20% IBR penetration"
        );
    }

    // ── Test 8: Upgrade recommendations — 60% IBR → pilot protection ─────────
    #[test]
    fn test_upgrade_recommendations_60pct_ibr() {
        let scheme = make_scheme(60.0, false, false);
        let recs = recommend_protection_upgrades(&scheme, 60.0);

        let has_pilot = recs.iter().any(|r| {
            r.to_lowercase().contains("pilot") || r.to_lowercase().contains("communication")
        });

        assert!(
            has_pilot,
            "Should recommend pilot protection at 60% IBR penetration. Got: {:?}",
            recs
        );
        assert!(
            !recs.is_empty(),
            "Should have multiple recommendations at 60% penetration"
        );
    }

    // ── Bonus: Combined LOM method picks up multiple triggers ─────────────────
    #[test]
    fn test_lom_combined_multiple_triggers() {
        let mut detector = make_lom_detector(LomMethod::Combined);
        detector.freq_min_hz = 49.5; // tight band

        // Frequency: 48.5 Hz (under-freq) AND ROCOF = 3 Hz/s
        let freq_history = vec![50.0];
        let result = detect_loss_of_mains(48.5, &freq_history, 0.0, &[], &detector, 0.5);

        assert!(result.lom_detected, "Combined LOM should trigger");
        assert!(
            result.triggering_method.contains("ROCOF")
                || result.triggering_method.contains("UnderOverFreq"),
            "Should report triggering methods: {}",
            result.triggering_method
        );
    }

    // ── Bonus: Upgrade at > 70% recommends grid-forming inverters ─────────────
    #[test]
    fn test_upgrade_recommendations_80pct_grid_forming() {
        let scheme = make_scheme(80.0, true, true);
        let recs = recommend_protection_upgrades(&scheme, 80.0);

        let has_grid_forming = recs
            .iter()
            .any(|r| r.to_lowercase().contains("grid-forming"));

        assert!(
            has_grid_forming,
            "Should recommend grid-forming inverters at 80% penetration. Got: {:?}",
            recs
        );
    }

    // ── Bonus: Fault current with mixed sync + IBR ────────────────────────────
    #[test]
    fn test_fault_current_mixed_sync_ibr() {
        let units = vec![make_ibr_unit(0, 1.2)]; // IBR: 1.2 pu
                                                 // sync contribution: 3.5 pu → total 4.7 pu → adequate for OC
        let assessment = assess_fault_current(&units, 0, 3.5);

        assert!(
            assessment.adequate_for_overcurrent,
            "Mixed sync+IBR should be adequate: total={}",
            assessment.total_fault_current_pu
        );
        assert!((assessment.ibr_contribution_pu - 1.2).abs() < 1e-9);
        assert!((assessment.sync_contribution_pu - 3.5).abs() < 1e-9);
    }

    // ── Bonus: No-infeed-correction relay leaves reach unchanged ──────────────
    #[test]
    fn test_distance_reach_no_correction() {
        let mut relay = make_relay(0.8);
        relay.ibr_infeed_correction = false;
        let adjusted = correct_distance_reach(&relay, 0.5, 1.0);
        assert!(
            (adjusted - 0.8).abs() < 1e-9,
            "Without correction, reach should remain 0.8 pu"
        );
    }
}
