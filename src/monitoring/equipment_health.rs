//! Substation Equipment Condition Monitoring (TTT).
//!
//! Implements health scoring and maintenance recommendation for substation
//! assets following IEC 60422 (transformer oil), IEC 60599 (DGA), and
//! industry best practices for SF6 circuit breakers.
//!
//! # Transformer scoring model
//!
//! | Factor | Weight | Notes |
//! |--------|--------|-------|
//! | Age | 0.20 | 40-year nominal life |
//! | Thermal (hotspot) | 0.25 | Rated at 98 °C |
//! | Moisture (oil) | 0.20 | >25 ppm critical |
//! | Acidity | 0.10 | >0.2 mg KOH/g critical |
//! | DGA | 0.15 | Rogers ratio severity |
//! | Power factor | 0.10 | Dielectric loss |
//!
//! # Breaker scoring model
//!
//! | Factor | Weight | Notes |
//! |--------|--------|-------|
//! | SF6 pressure | 0.30 | <80% → critical |
//! | Contact wear | 0.25 | >80% → maintenance |
//! | Timing deviation | 0.20 | >20% from spec |
//! | Operation count | 0.15 | vs rated 10 000 ops |
//! | Age | 0.10 | 30-year nominal life |

use thiserror::Error;

// ── Error ─────────────────────────────────────────────────────────────────────

/// Errors from the equipment health monitoring module.
#[derive(Debug, Error)]
pub enum HealthError {
    /// Equipment ID collision.
    #[error("equipment id {0} already registered")]
    DuplicateId(usize),
    /// Sensor reading is physically impossible.
    #[error("invalid sensor reading for {field}: {value}")]
    InvalidReading { field: &'static str, value: f64 },
}

// ── Risk and maintenance ──────────────────────────────────────────────────────

/// Equipment risk level classification.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum RiskLevel {
    /// Score > 75: normal operation.
    Low,
    /// Score 50–75: increased monitoring recommended.
    Medium,
    /// Score 25–50: planned maintenance required.
    High,
    /// Score < 25: immediate action needed.
    Critical,
}

/// Recommended maintenance action derived from the health score.
#[derive(Debug, Clone, PartialEq)]
pub enum MaintenanceAction {
    /// Equipment is healthy; no action required.
    NoAction,
    /// Increase monitoring frequency.
    IncreasedMonitoring,
    /// Schedule maintenance within the given window.
    ScheduledMaintenance {
        /// Months within which maintenance should be completed.
        within_months: usize,
    },
    /// Urgent but not immediately safety-critical maintenance.
    UrgentMaintenance {
        /// Human-readable reason for urgency.
        reason: String,
    },
    /// Take the equipment out of service immediately.
    ImmediateOutage {
        /// Human-readable reason for immediate outage.
        reason: String,
    },
}

// ── DGA result ────────────────────────────────────────────────────────────────

/// Dissolved Gas Analysis (DGA) measurement from transformer oil \[ppm\].
#[derive(Debug, Clone)]
pub struct DgaResult {
    /// Hydrogen H2 \[ppm\].
    pub hydrogen_ppm: f64,
    /// Methane CH4 \[ppm\].
    pub methane_ppm: f64,
    /// Ethane C2H6 \[ppm\].
    pub ethane_ppm: f64,
    /// Ethylene C2H4 \[ppm\].
    pub ethylene_ppm: f64,
    /// Acetylene C2H2 \[ppm\].
    pub acetylene_ppm: f64,
    /// Carbon monoxide CO \[ppm\].
    pub co_ppm: f64,
    /// Carbon dioxide CO2 \[ppm\].
    pub co2_ppm: f64,
    /// Oxygen O2 \[ppm\].
    pub o2_ppm: f64,
    /// Nitrogen N2 \[ppm\].
    pub n2_ppm: f64,
    /// Total dissolved combustible gas (TDCG) \[ppm\].
    pub total_dissolved_gas_ppm: f64,
}

impl DgaResult {
    /// Compute total combustible gas (H2 + CH4 + C2H2 + C2H4 + C2H6 + CO).
    pub fn tdcg(&self) -> f64 {
        self.hydrogen_ppm
            + self.methane_ppm
            + self.acetylene_ppm
            + self.ethylene_ppm
            + self.ethane_ppm
            + self.co_ppm
    }
}

// ── Transformer health indicators ─────────────────────────────────────────────

/// Condition measurements for a power transformer.
pub struct TransformerHealthIndicators {
    /// Equipment identifier.
    pub id: usize,
    /// Transformer age \[years\].
    pub age_years: f64,
    /// Thermal loading \[% of rated capacity\].
    pub loading_pct: f64,
    /// Winding hotspot temperature \[°C\].
    pub hotspot_temp_c: f64,
    /// Top oil temperature \[°C\].
    pub top_oil_temp_c: f64,
    /// Dissolved water in oil (moisture) \[ppm\].
    pub moisture_ppm: f64,
    /// Oil acidity number \[mg KOH/g\].
    pub acidity_mg_koh_per_g: f64,
    /// Dissolved gas analysis readings.
    pub dissolved_gas_analysis: DgaResult,
    /// Dielectric loss factor \[%\].
    pub power_factor_pct: f64,
    /// Insulation resistance at 60 s \[MΩ\].
    pub insulation_resistance_mohm: f64,
}

// ── Breaker health indicators ─────────────────────────────────────────────────

/// Condition measurements for an SF6 circuit breaker.
pub struct BreakerHealthIndicators {
    /// Equipment identifier.
    pub id: usize,
    /// Breaker age \[years\].
    pub age_years: f64,
    /// Total number of open/close operations performed.
    pub operation_count: usize,
    /// Percentage of contact erosion \[%\] (0 = new, 100 = fully worn).
    pub contact_wear_pct: f64,
    /// SF6 fill pressure as fraction of rated \[%\].
    pub sf6_pressure_pct: f64,
    /// Moisture in SF6 gas \[ppm\].
    pub sf6_moisture_ppm: f64,
    /// Control circuit voltage \[V\].
    pub control_voltage_v: f64,
    /// Spring charging motor current \[A\].
    pub charging_current_a: f64,
    /// Measured opening time \[ms\].
    pub timing_open_ms: f64,
    /// Measured closing time \[ms\].
    pub timing_close_ms: f64,
    /// Cumulative fault current duty \[kA²·cycles\].
    pub arcing_current_ka_sq_cycles: f64,
}

// ── Health score ──────────────────────────────────────────────────────────────

/// Computed health score for one piece of equipment.
pub struct HealthScore {
    /// Equipment identifier.
    pub equipment_id: usize,
    /// Human-readable equipment type label.
    pub equipment_type: String,
    /// Overall health score \[0–100\] (100 = new, 0 = failed).
    pub overall_score: f64,
    /// Individual indicator contributions:
    /// `(name, measured_value, score_contribution_0_to_100)`.
    pub indicators: Vec<(String, f64, f64)>,
    /// Derived risk level.
    pub risk_level: RiskLevel,
    /// Estimated remaining useful life \[years\].
    pub remaining_life_years: f64,
    /// Recommended maintenance action.
    pub maintenance_recommendation: MaintenanceAction,
}

impl HealthScore {
    /// Classify the risk level from the overall score.
    fn classify_risk(score: f64) -> RiskLevel {
        if score >= 75.0 {
            RiskLevel::Low
        } else if score >= 50.0 {
            RiskLevel::Medium
        } else if score >= 25.0 {
            RiskLevel::High
        } else {
            RiskLevel::Critical
        }
    }

    /// Recommend a maintenance action based on risk level and sub-indicators.
    fn recommend_action(
        risk: &RiskLevel,
        critical_reasons: &[&str],
        urgent_reasons: &[&str],
    ) -> MaintenanceAction {
        if !critical_reasons.is_empty() {
            return MaintenanceAction::ImmediateOutage {
                reason: critical_reasons.join("; "),
            };
        }
        if !urgent_reasons.is_empty() {
            return MaintenanceAction::UrgentMaintenance {
                reason: urgent_reasons.join("; "),
            };
        }
        match risk {
            RiskLevel::Low => MaintenanceAction::NoAction,
            RiskLevel::Medium => MaintenanceAction::IncreasedMonitoring,
            RiskLevel::High => MaintenanceAction::ScheduledMaintenance { within_months: 6 },
            RiskLevel::Critical => MaintenanceAction::ImmediateOutage {
                reason: String::from("overall health score critical"),
            },
        }
    }
}

// ── Monitor config ────────────────────────────────────────────────────────────

/// Configuration for the equipment health monitoring system.
pub struct EquipmentHealthConfig {
    /// Health score threshold above which an alarm is raised \[0–100\].
    pub alarm_on_threshold: f64,
    /// Health score below which the equipment is deemed critical \[0–100\].
    pub critical_threshold: f64,
    /// Interval between monitoring cycles \[s\].
    pub monitoring_interval_s: f64,
}

impl Default for EquipmentHealthConfig {
    fn default() -> Self {
        Self {
            alarm_on_threshold: 70.0,
            critical_threshold: 30.0,
            monitoring_interval_s: 300.0,
        }
    }
}

// ── Monitor ───────────────────────────────────────────────────────────────────

/// Scores health of substation equipment using IEC-based models.
pub struct EquipmentHealthMonitor {
    config: EquipmentHealthConfig,
}

impl EquipmentHealthMonitor {
    /// Create a monitor with the given configuration.
    pub fn new(config: EquipmentHealthConfig) -> Self {
        Self { config }
    }

    // ── Transformer scoring ───────────────────────────────────────────────────

    /// Score transformer health following IEC 60422 / IEC 60599.
    ///
    /// Returns a [`HealthScore`] with weighted component contributions.
    pub fn score_transformer(&self, ind: &TransformerHealthIndicators) -> HealthScore {
        const NOMINAL_LIFE: f64 = 40.0; // years
        const RATED_HOTSPOT: f64 = 98.0; // °C

        let mut indicators: Vec<(String, f64, f64)> = Vec::new();

        // ── Age factor (weight 0.20) ──────────────────────────────────────────
        // Score: 100 − (age/40) × 20  → linear deduction, max 20 points off.
        let age_score = (100.0 - (ind.age_years / NOMINAL_LIFE) * 20.0).clamp(0.0, 100.0);
        indicators.push(("age".into(), ind.age_years, age_score));

        // ── Thermal: hotspot (weight 0.25) ────────────────────────────────────
        // Score: 100 − (hotspot − 98) × 2, clamped to [0, 100].
        let thermal_score =
            (100.0 - (ind.hotspot_temp_c - RATED_HOTSPOT).max(0.0) * 2.0).clamp(0.0, 100.0);
        indicators.push(("hotspot_temp_c".into(), ind.hotspot_temp_c, thermal_score));

        // ── Moisture (weight 0.20) ────────────────────────────────────────────
        // Score: 100 − moisture_ppm × 2, clamped.
        let moisture_score = (100.0 - ind.moisture_ppm * 2.0).clamp(0.0, 100.0);
        indicators.push(("moisture_ppm".into(), ind.moisture_ppm, moisture_score));

        // ── Acidity (weight 0.10) ─────────────────────────────────────────────
        // Threshold: >0.2 mg KOH/g → score < 50.
        let acid_score = (100.0 - ind.acidity_mg_koh_per_g * 200.0).clamp(0.0, 100.0);
        indicators.push((
            "acidity_mg_koh_per_g".into(),
            ind.acidity_mg_koh_per_g,
            acid_score,
        ));

        // ── DGA (weight 0.15) ─────────────────────────────────────────────────
        let (_, severity) = self.rogers_ratio(&ind.dissolved_gas_analysis);
        // severity is in [0.0 = normal … 1.0 = severe].
        let dga_score = (100.0 - severity * 100.0).clamp(0.0, 100.0);
        indicators.push(("dga_severity".into(), severity, dga_score));

        // ── Power factor (weight 0.10) ─────────────────────────────────────────
        // Threshold: >0.5% → concern; >1.0% → significant.
        let pf_score = (100.0 - ind.power_factor_pct * 50.0).clamp(0.0, 100.0);
        indicators.push(("power_factor_pct".into(), ind.power_factor_pct, pf_score));

        // ── Weighted overall score ─────────────────────────────────────────────
        let weights = [0.20, 0.25, 0.20, 0.10, 0.15, 0.10];
        let overall_score: f64 = indicators
            .iter()
            .zip(weights.iter())
            .map(|((_n, _v, s), w)| s * w)
            .sum();

        let risk_level = HealthScore::classify_risk(overall_score);

        // ── Remaining life estimate ────────────────────────────────────────────
        // Linear extrapolation from current score.
        let remaining_life_years = if overall_score > 0.0 {
            (NOMINAL_LIFE - ind.age_years) * (overall_score / 100.0)
        } else {
            0.0
        }
        .max(0.0);

        // ── Critical / urgent conditions ──────────────────────────────────────
        let mut critical_reasons: Vec<&str> = Vec::new();
        let mut urgent_reasons: Vec<&str> = Vec::new();

        if ind.moisture_ppm > 35.0 {
            critical_reasons.push("extreme moisture (>35 ppm)");
        }
        if ind.hotspot_temp_c > 140.0 {
            critical_reasons.push("hotspot temperature critical (>140 °C)");
        }
        if ind.dissolved_gas_analysis.acetylene_ppm > 35.0 {
            critical_reasons.push("high acetylene (arcing fault indication)");
        }
        if ind.moisture_ppm > 20.0 && ind.moisture_ppm <= 35.0 {
            urgent_reasons.push("moisture elevated (>20 ppm)");
        }
        if ind.acidity_mg_koh_per_g > 0.3 {
            urgent_reasons.push("oil acidity high (>0.3 mg KOH/g)");
        }

        let recommendation =
            HealthScore::recommend_action(&risk_level, &critical_reasons, &urgent_reasons);

        HealthScore {
            equipment_id: ind.id,
            equipment_type: String::from("Transformer"),
            overall_score,
            indicators,
            risk_level,
            remaining_life_years,
            maintenance_recommendation: recommendation,
        }
    }

    // ── Breaker scoring ───────────────────────────────────────────────────────

    /// Score circuit breaker health based on SF6, wear, timing, and age.
    pub fn score_breaker(&self, ind: &BreakerHealthIndicators) -> HealthScore {
        const NOMINAL_LIFE: f64 = 30.0; // years
        const RATED_OPS: f64 = 10_000.0; // operations
        const SPEC_OPEN_MS: f64 = 50.0; // ms nominal opening time
        const SPEC_CLOSE_MS: f64 = 60.0; // ms nominal closing time

        let mut indicators: Vec<(String, f64, f64)> = Vec::new();

        // ── SF6 pressure (weight 0.30) ────────────────────────────────────────
        // <80% is critical; score: linear from 100% → 80% keeps 100 score,
        // then drops sharply.
        let sf6_score = if ind.sf6_pressure_pct >= 80.0 {
            100.0
        } else {
            ((ind.sf6_pressure_pct - 50.0) / 30.0 * 100.0).clamp(0.0, 100.0)
        };
        indicators.push(("sf6_pressure_pct".into(), ind.sf6_pressure_pct, sf6_score));

        // ── Contact wear (weight 0.25) ────────────────────────────────────────
        // 0% wear = 100 score; 100% wear = 0 score.
        let wear_score = (100.0 - ind.contact_wear_pct).clamp(0.0, 100.0);
        indicators.push(("contact_wear_pct".into(), ind.contact_wear_pct, wear_score));

        // ── Timing deviation (weight 0.20) ────────────────────────────────────
        // Deviation from spec: >20% from either open or close spec → concern.
        let open_dev =
            ((ind.timing_open_ms - SPEC_OPEN_MS).abs() / SPEC_OPEN_MS * 100.0).min(100.0);
        let close_dev =
            ((ind.timing_close_ms - SPEC_CLOSE_MS).abs() / SPEC_CLOSE_MS * 100.0).min(100.0);
        let timing_score = (100.0 - (open_dev + close_dev) / 2.0).clamp(0.0, 100.0);
        indicators.push((
            "timing_deviation_pct".into(),
            (open_dev + close_dev) / 2.0,
            timing_score,
        ));

        // ── Operation count (weight 0.15) ─────────────────────────────────────
        let ops_ratio = ind.operation_count as f64 / RATED_OPS;
        let ops_score = (100.0 - ops_ratio * 100.0).clamp(0.0, 100.0);
        indicators.push((
            "operation_count".into(),
            ind.operation_count as f64,
            ops_score,
        ));

        // ── Age (weight 0.10) ─────────────────────────────────────────────────
        let age_score = (100.0 - (ind.age_years / NOMINAL_LIFE) * 20.0).clamp(0.0, 100.0);
        indicators.push(("age_years".into(), ind.age_years, age_score));

        let weights = [0.30, 0.25, 0.20, 0.15, 0.10];
        let overall_score: f64 = indicators
            .iter()
            .zip(weights.iter())
            .map(|((_n, _v, s), w)| s * w)
            .sum();

        let risk_level = HealthScore::classify_risk(overall_score);

        let remaining_life_years =
            ((NOMINAL_LIFE - ind.age_years) * (overall_score / 100.0)).max(0.0);

        let mut critical_reasons: Vec<&str> = Vec::new();
        let mut urgent_reasons: Vec<&str> = Vec::new();

        if ind.sf6_pressure_pct < 70.0 {
            critical_reasons.push("SF6 pressure critically low (<70%)");
        } else if ind.sf6_pressure_pct < 80.0 {
            urgent_reasons.push("SF6 pressure low (<80%)");
        }
        if ind.contact_wear_pct > 90.0 {
            urgent_reasons.push("contact wear excessive (>90%)");
        }
        if ind.sf6_moisture_ppm > 200.0 {
            urgent_reasons.push("SF6 moisture high (>200 ppm)");
        }

        let recommendation =
            HealthScore::recommend_action(&risk_level, &critical_reasons, &urgent_reasons);

        HealthScore {
            equipment_id: ind.id,
            equipment_type: String::from("CircuitBreaker"),
            overall_score,
            indicators,
            risk_level,
            remaining_life_years,
            maintenance_recommendation: recommendation,
        }
    }

    // ── DGA interpretation ────────────────────────────────────────────────────

    /// Rogers ratio analysis (IEC 60599) for fault type identification.
    ///
    /// Computes CH4/H2, C2H2/C2H4, and C2H4/C2H6 ratios and maps them to
    /// a fault type and severity factor.
    ///
    /// Returns `(fault_type_label, severity_factor)` where
    /// `severity_factor` ∈ \[0.0, 1.0\].
    pub fn rogers_ratio(&self, dga: &DgaResult) -> (String, f64) {
        let r1 = if dga.hydrogen_ppm > 1e-6 {
            dga.methane_ppm / dga.hydrogen_ppm
        } else {
            0.0
        };
        let r2 = if dga.ethylene_ppm > 1e-6 {
            dga.acetylene_ppm / dga.ethylene_ppm
        } else {
            0.0
        };
        let r3 = if dga.ethane_ppm > 1e-6 {
            dga.ethylene_ppm / dga.ethane_ppm
        } else {
            0.0
        };

        // Rogers ratio fault code table (IEC 60599 Annex C).
        let fault_type = match (r1_code(r1), r2_code(r2), r3_code(r3)) {
            (0, 0, 0) => "No fault (normal)",
            (0, 0, 1) => "Low-energy thermal fault (T1 <300°C)",
            (0, 0, 2) => "Low-energy thermal fault (T2 300–700°C)",
            (0, 1, 0) => "No fault or PD",
            (0, 1, 2) => "High-energy thermal fault (T3 >700°C)",
            (1, 0, 0) => "Partial discharge (PD)",
            (1, 0, 1) => "Partial discharge with low energy discharges",
            (2, 0, 0) => "Electrical discharge (D1 low energy)",
            (2, 0, 1) => "Electrical discharge (D2 high energy)",
            (2, 1, 0) => "Electrical discharge with thermal overlay (DT)",
            _ => "Mixed / indeterminate fault",
        };

        // Severity: based on TDCG and acetylene concentration.
        let tdcg = dga.tdcg();
        let severity = if dga.acetylene_ppm > 35.0 || tdcg > 4630.0 {
            1.0
        } else if dga.acetylene_ppm > 9.0 || tdcg > 1920.0 {
            0.75
        } else if dga.acetylene_ppm > 4.0 || tdcg > 720.0 {
            0.50
        } else if tdcg > 300.0 {
            0.25
        } else {
            0.05
        };

        (fault_type.into(), severity)
    }

    /// Duval triangle method (IEC 60599) for fault zone identification.
    ///
    /// Maps CH4, C2H4, C2H2 percentages of their combined total to a
    /// fault zone: T1, T2, T3, D1, D2, or DT.
    pub fn duval_triangle(&self, dga: &DgaResult) -> String {
        let total = dga.methane_ppm + dga.ethylene_ppm + dga.acetylene_ppm;
        if total < 1e-9 {
            return String::from("No combustible hydrocarbons detected");
        }

        let pct_ch4 = dga.methane_ppm / total * 100.0;
        let pct_c2h4 = dga.ethylene_ppm / total * 100.0;
        let pct_c2h2 = dga.acetylene_ppm / total * 100.0;

        // Duval triangle zone boundaries (simplified):
        // D1: C2H2 > 13% (and moderate C2H2)
        // D2: C2H2 > 29%
        // DT: C2H2 > 13% and C2H4 > 20%
        // T3: C2H4 > 50%, C2H2 < 4%
        // T2: C2H4 20–50%, C2H2 < 4%
        // T1: CH4 > 98% (residual zone), or default low-temperature thermal

        if pct_c2h2 > 29.0 {
            format!(
                "D2: High-energy electrical discharge (C2H2={:.1}%, C2H4={:.1}%, CH4={:.1}%)",
                pct_c2h2, pct_c2h4, pct_ch4
            )
        } else if pct_c2h2 > 13.0 && pct_c2h4 > 20.0 {
            format!(
                "DT: Electrical + thermal fault (C2H2={:.1}%, C2H4={:.1}%, CH4={:.1}%)",
                pct_c2h2, pct_c2h4, pct_ch4
            )
        } else if pct_c2h2 > 13.0 {
            format!(
                "D1: Low-energy electrical discharge (C2H2={:.1}%, C2H4={:.1}%, CH4={:.1}%)",
                pct_c2h2, pct_c2h4, pct_ch4
            )
        } else if pct_c2h4 > 50.0 {
            format!(
                "T3: High-temperature thermal fault >700°C (C2H4={:.1}%, CH4={:.1}%)",
                pct_c2h4, pct_ch4
            )
        } else if pct_c2h4 > 20.0 {
            format!(
                "T2: Medium-temperature thermal fault 300–700°C (C2H4={:.1}%, CH4={:.1}%)",
                pct_c2h4, pct_ch4
            )
        } else {
            format!(
                "T1: Low-temperature thermal fault <300°C (CH4={:.1}%, C2H4={:.1}%)",
                pct_ch4, pct_c2h4
            )
        }
    }

    /// Return the configured alarm threshold.
    pub fn alarm_threshold(&self) -> f64 {
        self.config.alarm_on_threshold
    }

    /// Return the configured critical threshold.
    pub fn critical_threshold(&self) -> f64 {
        self.config.critical_threshold
    }
}

// ── Rogers ratio helper functions ─────────────────────────────────────────────

/// Encode CH4/H2 ratio into Rogers code 0/1/2.
fn r1_code(r: f64) -> u8 {
    if (0.1..1.0).contains(&r) {
        0
    } else if r < 0.1 {
        1
    } else {
        2
    }
}

/// Encode C2H2/C2H4 ratio into Rogers code 0/1/2.
fn r2_code(r: f64) -> u8 {
    if r < 0.1 {
        0
    } else if r < 3.0 {
        1
    } else {
        2
    }
}

/// Encode C2H4/C2H6 ratio into Rogers code 0/1/2.
fn r3_code(r: f64) -> u8 {
    if r < 1.0 {
        0
    } else if r < 3.0 {
        1
    } else {
        2
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn monitor() -> EquipmentHealthMonitor {
        EquipmentHealthMonitor::new(EquipmentHealthConfig::default())
    }

    fn clean_dga() -> DgaResult {
        DgaResult {
            hydrogen_ppm: 10.0,
            methane_ppm: 5.0,
            ethane_ppm: 3.0,
            ethylene_ppm: 2.0,
            acetylene_ppm: 0.0,
            co_ppm: 50.0,
            co2_ppm: 400.0,
            o2_ppm: 5000.0,
            n2_ppm: 40000.0,
            total_dissolved_gas_ppm: 70.0,
        }
    }

    fn new_transformer() -> TransformerHealthIndicators {
        TransformerHealthIndicators {
            id: 1,
            age_years: 2.0,
            loading_pct: 60.0,
            hotspot_temp_c: 80.0,
            top_oil_temp_c: 60.0,
            moisture_ppm: 5.0,
            acidity_mg_koh_per_g: 0.02,
            dissolved_gas_analysis: clean_dga(),
            power_factor_pct: 0.1,
            insulation_resistance_mohm: 5000.0,
        }
    }

    fn old_wet_transformer() -> TransformerHealthIndicators {
        TransformerHealthIndicators {
            id: 2,
            age_years: 38.0,
            loading_pct: 95.0,
            hotspot_temp_c: 130.0,
            top_oil_temp_c: 90.0,
            moisture_ppm: 30.0, // > 25 ppm critical
            acidity_mg_koh_per_g: 0.35,
            dissolved_gas_analysis: DgaResult {
                hydrogen_ppm: 300.0,
                methane_ppm: 150.0,
                ethane_ppm: 50.0,
                ethylene_ppm: 100.0,
                acetylene_ppm: 40.0, // arcing indicator
                co_ppm: 500.0,
                co2_ppm: 5000.0,
                o2_ppm: 1000.0,
                n2_ppm: 30000.0,
                total_dissolved_gas_ppm: 1140.0,
            },
            power_factor_pct: 0.8,
            insulation_resistance_mohm: 200.0,
        }
    }

    fn new_breaker() -> BreakerHealthIndicators {
        BreakerHealthIndicators {
            id: 10,
            age_years: 1.0,
            operation_count: 100,
            contact_wear_pct: 2.0,
            sf6_pressure_pct: 100.0,
            sf6_moisture_ppm: 10.0,
            control_voltage_v: 125.0,
            charging_current_a: 2.0,
            timing_open_ms: 50.0,
            timing_close_ms: 60.0,
            arcing_current_ka_sq_cycles: 0.5,
        }
    }

    fn critical_sf6_breaker() -> BreakerHealthIndicators {
        BreakerHealthIndicators {
            id: 11,
            age_years: 15.0,
            operation_count: 5000,
            contact_wear_pct: 40.0,
            sf6_pressure_pct: 55.0, // critically low
            sf6_moisture_ppm: 80.0,
            control_voltage_v: 110.0,
            charging_current_a: 3.5,
            timing_open_ms: 50.0,
            timing_close_ms: 60.0,
            arcing_current_ka_sq_cycles: 20.0,
        }
    }

    /// New transformer should score high (> 80).
    #[test]
    fn test_new_transformer_high_score() {
        let score = monitor().score_transformer(&new_transformer());
        assert!(
            score.overall_score > 80.0,
            "New transformer must score > 80, got {:.2}",
            score.overall_score
        );
        assert!(
            score.risk_level == RiskLevel::Low,
            "Risk should be Low for new transformer"
        );
    }

    /// Old transformer with high moisture should score low and recommend action.
    #[test]
    fn test_old_wet_transformer_low_score() {
        let score = monitor().score_transformer(&old_wet_transformer());
        assert!(
            score.overall_score < 50.0,
            "Old wet transformer must score < 50, got {:.2}",
            score.overall_score
        );
        // Should recommend immediate outage due to high acetylene (>35 ppm).
        assert!(
            matches!(
                score.maintenance_recommendation,
                MaintenanceAction::ImmediateOutage { .. }
            ),
            "Expected ImmediateOutage, got {:?}",
            score.maintenance_recommendation
        );
    }

    /// Critically low SF6 pressure should trigger ImmediateOutage.
    #[test]
    fn test_critical_sf6_immediate_outage() {
        let score = monitor().score_breaker(&critical_sf6_breaker());
        assert!(
            matches!(
                score.maintenance_recommendation,
                MaintenanceAction::ImmediateOutage { .. }
                    | MaintenanceAction::UrgentMaintenance { .. }
            ),
            "SF6 <70% should be ImmediateOutage or UrgentMaintenance, got {:?}",
            score.maintenance_recommendation
        );
    }

    /// New breaker should score near 100 and require no action.
    #[test]
    fn test_new_breaker_no_action() {
        let score = monitor().score_breaker(&new_breaker());
        assert!(
            score.overall_score > 85.0,
            "New breaker must score > 85, got {:.2}",
            score.overall_score
        );
        assert!(
            matches!(
                score.maintenance_recommendation,
                MaintenanceAction::NoAction
            ),
            "New breaker: expected NoAction, got {:?}",
            score.maintenance_recommendation
        );
    }

    /// Rogers ratio should identify thermal fault.
    ///
    /// Target code: (0, 0, 1) → "Low-energy thermal fault (T1 <300°C)"
    /// r1 = CH4/H2 = 50/100 = 0.5   → code 0 (in \[0.1, 1.0\))
    /// r2 = C2H2/C2H4 = 1/300 ≈ 0.003 → code 0 (<0.1)
    /// r3 = C2H4/C2H6 = 300/200 = 1.5  → code 1 (in \[1.0, 3.0\))
    #[test]
    fn test_rogers_ratio_thermal_fault() {
        let dga = DgaResult {
            hydrogen_ppm: 100.0,
            methane_ppm: 50.0, // CH4/H2 = 0.5 → code 0
            ethane_ppm: 200.0, // C2H4/C2H6 = 300/200 = 1.5 → code 1
            ethylene_ppm: 300.0,
            acetylene_ppm: 1.0, // C2H2/C2H4 ≈ 0.003 → code 0
            co_ppm: 200.0,
            co2_ppm: 1500.0,
            o2_ppm: 4000.0,
            n2_ppm: 35000.0,
            total_dissolved_gas_ppm: 851.0,
        };
        let (fault_type, severity) = monitor().rogers_ratio(&dga);
        assert!(
            fault_type.to_lowercase().contains("thermal"),
            "Should identify thermal fault (code 0,0,1 → T1), got: {}",
            fault_type
        );
        assert!(severity > 0.0, "Severity must be positive for active fault");
    }

    /// Duval triangle: high C2H2 percentage → D2 (high-energy discharge).
    #[test]
    fn test_duval_triangle_d2_zone() {
        let dga = DgaResult {
            hydrogen_ppm: 200.0,
            methane_ppm: 10.0,
            ethane_ppm: 5.0,
            ethylene_ppm: 20.0,  // 20/(10+20+70) = 20%
            acetylene_ppm: 70.0, // 70/(10+20+70) = 70% > 29% → D2
            co_ppm: 50.0,
            co2_ppm: 500.0,
            o2_ppm: 3000.0,
            n2_ppm: 30000.0,
            total_dissolved_gas_ppm: 355.0,
        };
        let zone = monitor().duval_triangle(&dga);
        assert!(zone.starts_with("D2"), "Expected D2 zone, got: {}", zone);
    }

    /// Duval triangle: high C2H4, low C2H2 → T3 (high-temperature thermal).
    #[test]
    fn test_duval_triangle_t3_zone() {
        let dga = DgaResult {
            hydrogen_ppm: 100.0,
            methane_ppm: 30.0,
            ethane_ppm: 10.0,
            ethylene_ppm: 65.0, // 65/(30+65+2) = 67% > 50% → T3
            acetylene_ppm: 2.0, // < 13% → not D
            co_ppm: 80.0,
            co2_ppm: 600.0,
            o2_ppm: 2000.0,
            n2_ppm: 25000.0,
            total_dissolved_gas_ppm: 289.0,
        };
        let zone = monitor().duval_triangle(&dga);
        assert!(zone.starts_with("T3"), "Expected T3 zone, got: {}", zone);
    }

    /// Breaker with high contact wear should recommend maintenance.
    #[test]
    fn test_breaker_high_wear_maintenance() {
        let mut br = new_breaker();
        br.contact_wear_pct = 92.0; // > 90% → urgent
        let score = monitor().score_breaker(&br);
        assert!(
            matches!(
                score.maintenance_recommendation,
                MaintenanceAction::UrgentMaintenance { .. }
                    | MaintenanceAction::ScheduledMaintenance { .. }
                    | MaintenanceAction::ImmediateOutage { .. }
            ),
            "High wear should recommend maintenance, got {:?}",
            score.maintenance_recommendation
        );
    }
}
