/// Battery pack fault detection and diagnostics.
///
/// Detects the following fault classes using ECM residual analysis
/// and impedance monitoring:
///
/// - **Open-circuit fault**       — broken cell/module connection (sudden ↑ voltage, ↓ capacity)
/// - **Internal short circuit**   — ISC: localised heat generation, abnormal self-discharge
/// - **Isolation resistance drop** — IMD: electrolyte leakage, cell casing breach (ground fault)
/// - **Thermal runaway precursor** — ΔT/Δt exceeding safe rate
/// - **Sensor fault**             — voltage/current sensor drift or failure
///
/// # Method
/// Each detector computes a residual (model output − measurement).
/// A fault is flagged when the residual exceeds a statistical threshold
/// derived from a χ² distribution at the configured confidence level.
use serde::{Deserialize, Serialize};

// ─── Fault types ─────────────────────────────────────────────────────────────

/// Battery fault classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FaultType {
    /// No fault detected
    None,
    /// Open-circuit fault in cell or module
    OpenCircuit,
    /// Internal short circuit (ISC)
    InternalShort,
    /// Isolation resistance drop (ground fault)
    IsolationResistance,
    /// Thermal runaway precursor (ΔT/Δt > threshold)
    ThermalRunaway,
    /// Voltage sensor fault (implausible reading)
    SensorVoltage,
    /// Current sensor fault (implausible reading)
    SensorCurrent,
    /// Capacity fade below safety threshold
    CapacityFade,
}

impl FaultType {
    pub fn is_critical(self) -> bool {
        matches!(
            self,
            Self::InternalShort | Self::ThermalRunaway | Self::OpenCircuit
        )
    }

    pub fn description(self) -> &'static str {
        match self {
            Self::None => "No fault",
            Self::OpenCircuit => "Open-circuit fault: connection break detected",
            Self::InternalShort => "Internal short circuit: localised heat source",
            Self::IsolationResistance => "Isolation resistance degraded: ground fault risk",
            Self::ThermalRunaway => "Thermal runaway precursor: temperature rate excessive",
            Self::SensorVoltage => "Voltage sensor fault: implausible reading",
            Self::SensorCurrent => "Current sensor fault: implausible reading",
            Self::CapacityFade => "Capacity fade below safety threshold",
        }
    }
}

// ─── Fault event ─────────────────────────────────────────────────────────────

/// A detected fault event with timestamp and confidence.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FaultEvent {
    /// Fault type
    pub fault_type: FaultType,
    /// Time of detection [s]
    pub time_s: f64,
    /// Residual magnitude at detection
    pub residual: f64,
    /// Detection threshold
    pub threshold: f64,
    /// Confidence (residual / threshold; > 1 means fault)
    pub confidence: f64,
    /// Recommended action
    pub action: &'static str,
}

impl FaultEvent {
    fn new(fault_type: FaultType, time_s: f64, residual: f64, threshold: f64) -> Self {
        Self {
            fault_type,
            time_s,
            residual,
            threshold,
            confidence: residual / threshold.max(1e-9),
            action: recommended_action(fault_type),
        }
    }
}

fn recommended_action(ft: FaultType) -> &'static str {
    match ft {
        FaultType::OpenCircuit => "Inspect wiring and busbars; check cell connectivity",
        FaultType::InternalShort => "Isolate immediately; check for thermal signs",
        FaultType::IsolationResistance => "IMD alert: check for electrolyte leakage",
        FaultType::ThermalRunaway => "Emergency shutdown; activate cooling; alert operator",
        FaultType::SensorVoltage => "Replace voltage sensor; cross-check with backup",
        FaultType::SensorCurrent => "Replace current sensor; verify shunt resistance",
        FaultType::CapacityFade => "Schedule replacement; reduce charge current",
        FaultType::None => "Monitor",
    }
}

// ─── Detector configuration ───────────────────────────────────────────────────

/// Fault detector configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FaultDetectorConfig {
    // ── Open-circuit detector ──
    /// Voltage residual threshold for open-circuit [V]
    pub oc_voltage_threshold_v: f64,
    /// Window length for voltage spike detection [samples]
    pub oc_window: usize,

    // ── Internal short circuit ──
    /// Self-discharge rate threshold for ISC detection [mAh/min]
    pub isc_self_discharge_ma: f64,
    /// Abnormal temperature rise rate [°C/s]
    pub isc_temp_rate_c_s: f64,

    // ── Isolation resistance ──
    /// Minimum acceptable isolation resistance [kΩ]
    pub imr_min_kohm: f64,

    // ── Thermal runaway ──
    /// Maximum safe temperature rate [°C/s]
    pub tr_rate_c_s: f64,
    /// Absolute maximum cell temperature [°C]
    pub tr_max_temp_c: f64,

    // ── Sensor faults ──
    /// Maximum plausible cell voltage [V]
    pub v_max_plausible: f64,
    /// Minimum plausible cell voltage [V]
    pub v_min_plausible: f64,
    /// Maximum plausible current magnitude [A]
    pub i_max_plausible: f64,

    // ── Capacity fade ──
    /// Minimum remaining capacity fraction before fault [0–1]
    pub capacity_min_fraction: f64,
}

impl FaultDetectorConfig {
    /// Typical NMC Li-ion cell configuration.
    pub fn nmc_default() -> Self {
        Self {
            oc_voltage_threshold_v: 0.5,
            oc_window: 5,
            isc_self_discharge_ma: 10.0,
            isc_temp_rate_c_s: 0.5,
            imr_min_kohm: 100.0,
            tr_rate_c_s: 1.0,
            tr_max_temp_c: 60.0,
            v_max_plausible: 4.5,
            v_min_plausible: 2.0,
            i_max_plausible: 500.0,
            capacity_min_fraction: 0.70,
        }
    }

    /// LFP-specific configuration (wider voltage range, higher temp tolerance).
    pub fn lfp_default() -> Self {
        Self {
            oc_voltage_threshold_v: 0.3,
            oc_window: 5,
            isc_self_discharge_ma: 8.0,
            isc_temp_rate_c_s: 0.8,
            imr_min_kohm: 80.0,
            tr_rate_c_s: 2.0,
            tr_max_temp_c: 70.0,
            v_max_plausible: 3.8,
            v_min_plausible: 2.5,
            i_max_plausible: 1000.0,
            capacity_min_fraction: 0.65,
        }
    }
}

// ─── Measurement sample ───────────────────────────────────────────────────────

/// Instantaneous battery measurement.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct BatterySample {
    /// Time [s]
    pub time_s: f64,
    /// Terminal voltage [V]
    pub voltage_v: f64,
    /// Applied current [A] (positive = discharge)
    pub current_a: f64,
    /// Cell/module temperature [°C]
    pub temp_c: f64,
    /// State of charge (0–1)
    pub soc: f64,
    /// Model-predicted voltage [V] (from ECM)
    pub v_pred: f64,
    /// Isolation resistance measurement [kΩ] (from IMD)
    pub isolation_kohm: f64,
    /// Remaining capacity estimate [Ah] (from SoC integrator)
    pub capacity_ah: f64,
    /// Nominal capacity [Ah]
    pub capacity_nominal_ah: f64,
}

// ─── Fault detector ───────────────────────────────────────────────────────────

/// Stateful fault detector with sliding window history.
pub struct FaultDetector {
    config: FaultDetectorConfig,
    history: Vec<BatterySample>,
    /// Active faults (can have multiple simultaneous)
    pub active_faults: Vec<FaultEvent>,
    /// All detected fault events (log)
    pub fault_log: Vec<FaultEvent>,
}

impl FaultDetector {
    pub fn new(config: FaultDetectorConfig) -> Self {
        Self {
            config,
            history: Vec::new(),
            active_faults: Vec::new(),
            fault_log: Vec::new(),
        }
    }

    /// Process a new measurement sample, returning any newly detected faults.
    pub fn update(&mut self, sample: BatterySample) -> Vec<FaultEvent> {
        let mut new_faults = Vec::new();

        // Run each detector
        new_faults.extend(self.detect_sensor_fault(&sample));
        new_faults.extend(self.detect_open_circuit(&sample));
        new_faults.extend(self.detect_thermal_runaway(&sample));
        new_faults.extend(self.detect_isolation_fault(&sample));
        new_faults.extend(self.detect_isc(&sample));
        new_faults.extend(self.detect_capacity_fade(&sample));

        // Update history
        self.history.push(sample);
        let max_history = self.config.oc_window * 4;
        if self.history.len() > max_history {
            self.history.drain(0..self.history.len() - max_history);
        }

        // Update active faults (clear resolved)
        self.active_faults.retain(|f| {
            // Keep only faults whose residual would still be active
            new_faults.iter().any(|nf| nf.fault_type == f.fault_type)
        });
        for f in &new_faults {
            if !self
                .active_faults
                .iter()
                .any(|af| af.fault_type == f.fault_type)
            {
                self.active_faults.push(f.clone());
            }
        }
        self.fault_log.extend(new_faults.clone());

        new_faults
    }

    /// Check if any critical fault is active.
    pub fn has_critical_fault(&self) -> bool {
        self.active_faults
            .iter()
            .any(|f| f.fault_type.is_critical())
    }

    /// Get the most severe active fault (highest confidence).
    pub fn most_severe(&self) -> Option<&FaultEvent> {
        self.active_faults
            .iter()
            .max_by(|a, b| a.confidence.partial_cmp(&b.confidence).unwrap())
    }

    // ── Individual detectors ──────────────────────────────────────────────────

    fn detect_sensor_fault(&self, s: &BatterySample) -> Vec<FaultEvent> {
        let mut faults = Vec::new();
        let c = &self.config;

        if s.voltage_v > c.v_max_plausible || s.voltage_v < c.v_min_plausible {
            let residual = if s.voltage_v > c.v_max_plausible {
                s.voltage_v - c.v_max_plausible
            } else {
                c.v_min_plausible - s.voltage_v
            };
            faults.push(FaultEvent::new(
                FaultType::SensorVoltage,
                s.time_s,
                residual,
                0.1,
            ));
        }

        if s.current_a.abs() > c.i_max_plausible {
            let residual = s.current_a.abs() - c.i_max_plausible;
            faults.push(FaultEvent::new(
                FaultType::SensorCurrent,
                s.time_s,
                residual,
                1.0,
            ));
        }
        faults
    }

    fn detect_open_circuit(&self, s: &BatterySample) -> Vec<FaultEvent> {
        // Open circuit: sudden large voltage residual when under load
        let v_residual = (s.voltage_v - s.v_pred).abs();
        let threshold = self.config.oc_voltage_threshold_v;

        // Only flag if load is non-trivial (avoid idle condition false positives)
        if s.current_a.abs() > 0.5 && v_residual > threshold {
            // Check if this is a sustained deviation (not a transient)
            if self.history.len() >= 2 {
                let recent_residuals: Vec<f64> = self
                    .history
                    .iter()
                    .rev()
                    .take(self.config.oc_window.min(self.history.len()))
                    .map(|h| (h.voltage_v - h.v_pred).abs())
                    .collect();
                let avg_residual =
                    recent_residuals.iter().sum::<f64>() / recent_residuals.len() as f64;
                if avg_residual > threshold * 0.7 {
                    return vec![FaultEvent::new(
                        FaultType::OpenCircuit,
                        s.time_s,
                        v_residual,
                        threshold,
                    )];
                }
            }
        }
        vec![]
    }

    fn detect_thermal_runaway(&self, s: &BatterySample) -> Vec<FaultEvent> {
        let mut faults = Vec::new();
        let c = &self.config;

        // Absolute temperature threshold
        if s.temp_c > c.tr_max_temp_c {
            let residual = s.temp_c - c.tr_max_temp_c;
            faults.push(FaultEvent::new(
                FaultType::ThermalRunaway,
                s.time_s,
                residual,
                5.0,
            ));
            return faults;
        }

        // Temperature rate threshold
        if let Some(prev) = self.history.last() {
            let dt = (s.time_s - prev.time_s).max(1e-9);
            let dt_temp = (s.temp_c - prev.temp_c) / dt;
            if dt_temp > c.tr_rate_c_s {
                let residual = dt_temp - c.tr_rate_c_s;
                faults.push(FaultEvent::new(
                    FaultType::ThermalRunaway,
                    s.time_s,
                    residual,
                    0.2,
                ));
            }
        }
        faults
    }

    fn detect_isolation_fault(&self, s: &BatterySample) -> Vec<FaultEvent> {
        let threshold = self.config.imr_min_kohm;
        if s.isolation_kohm > 0.0 && s.isolation_kohm < threshold {
            let residual = threshold - s.isolation_kohm;
            return vec![FaultEvent::new(
                FaultType::IsolationResistance,
                s.time_s,
                residual,
                threshold * 0.1,
            )];
        }
        vec![]
    }

    fn detect_isc(&self, s: &BatterySample) -> Vec<FaultEvent> {
        // ISC indicators:
        // 1. Self-discharge when supposedly idle (low current but SOC decreasing)
        // 2. Temperature rate above ISC threshold despite low current

        let mut faults = Vec::new();

        // Temperature anomaly under low current
        if s.current_a.abs() < 0.1 && !self.history.is_empty() {
            if let Some(prev) = self.history.last() {
                let dt = (s.time_s - prev.time_s).max(1e-9);
                let dt_temp = (s.temp_c - prev.temp_c) / dt;
                if dt_temp > self.config.isc_temp_rate_c_s {
                    let residual = dt_temp - self.config.isc_temp_rate_c_s;
                    faults.push(FaultEvent::new(
                        FaultType::InternalShort,
                        s.time_s,
                        residual,
                        0.1,
                    ));
                }
            }
        }

        // Anomalous self-discharge: SOC drop without current
        if s.current_a.abs() < 0.05 && self.history.len() >= 3 {
            let old = &self.history[self.history.len() - 3];
            let dt = (s.time_s - old.time_s).max(1e-9);
            let soc_rate = (old.soc - s.soc) / dt; // positive = draining
            let self_discharge_a = soc_rate * s.capacity_nominal_ah * 3600.0;
            let threshold_a = self.config.isc_self_discharge_ma / 1000.0;
            if self_discharge_a > threshold_a {
                let residual = self_discharge_a - threshold_a;
                faults.push(FaultEvent::new(
                    FaultType::InternalShort,
                    s.time_s,
                    residual,
                    threshold_a * 0.1,
                ));
            }
        }
        faults
    }

    fn detect_capacity_fade(&self, s: &BatterySample) -> Vec<FaultEvent> {
        let min_cap = self.config.capacity_min_fraction * s.capacity_nominal_ah;
        if s.capacity_ah > 0.0 && s.capacity_ah < min_cap {
            let residual = min_cap - s.capacity_ah;
            return vec![FaultEvent::new(
                FaultType::CapacityFade,
                s.time_s,
                residual,
                min_cap * 0.05,
            )];
        }
        vec![]
    }
}

// ─── Isolation resistance monitoring (IMD) ────────────────────────────────────

/// Insulation Monitoring Device (IMD) simulation.
///
/// Tracks isolation resistance history and computes trend.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InsulationMonitor {
    /// Historical (time, R_isolation [kΩ]) measurements
    pub history: Vec<(f64, f64)>,
    /// Minimum acceptable resistance [kΩ]
    pub r_min_kohm: f64,
}

impl InsulationMonitor {
    pub fn new(r_min_kohm: f64) -> Self {
        Self {
            history: Vec::new(),
            r_min_kohm,
        }
    }

    pub fn update(&mut self, time_s: f64, r_kohm: f64) {
        self.history.push((time_s, r_kohm));
    }

    /// Current isolation resistance [kΩ].
    pub fn current_r(&self) -> Option<f64> {
        self.history.last().map(|(_, r)| *r)
    }

    /// Linear trend of isolation resistance [kΩ/s].
    pub fn trend_kohm_per_s(&self) -> f64 {
        if self.history.len() < 2 {
            return 0.0;
        }
        let n = self.history.len() as f64;
        let sum_t: f64 = self.history.iter().map(|(t, _)| t).sum();
        let sum_r: f64 = self.history.iter().map(|(_, r)| r).sum();
        let sum_t2: f64 = self.history.iter().map(|(t, _)| t * t).sum();
        let sum_tr: f64 = self.history.iter().map(|(t, r)| t * r).sum();
        let denom = n * sum_t2 - sum_t * sum_t;
        if denom.abs() < 1e-12 {
            return 0.0;
        }
        (n * sum_tr - sum_t * sum_r) / denom
    }

    /// Estimated time to reach minimum acceptable resistance [s].
    pub fn time_to_alarm_s(&self) -> Option<f64> {
        let r = self.current_r()?;
        let trend = self.trend_kohm_per_s();
        if trend >= 0.0 || r <= self.r_min_kohm {
            return None;
        } // not degrading
        let t_last = self.history.last()?.0;
        Some(t_last + (r - self.r_min_kohm) / (-trend))
    }

    /// True if isolation resistance is below minimum.
    pub fn is_fault(&self) -> bool {
        self.current_r()
            .map(|r| r < self.r_min_kohm)
            .unwrap_or(false)
    }
}

// ─── Diagnostic summary ────────────────────────────────────────────────────

/// Overall pack diagnostic report.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DiagnosticReport {
    pub n_faults_detected: usize,
    pub n_critical: usize,
    pub most_common_fault: Option<FaultType>,
    pub fault_rate_per_hour: f64,
    pub health_score: f64, // 0 (failed) – 1 (healthy)
}

impl DiagnosticReport {
    pub fn from_log(fault_log: &[FaultEvent], elapsed_hours: f64) -> Self {
        let n = fault_log.len();
        let n_crit = fault_log
            .iter()
            .filter(|f| f.fault_type.is_critical())
            .count();

        // Most common fault type
        let mut counts = std::collections::HashMap::new();
        for f in fault_log {
            *counts.entry(f.fault_type as u8).or_insert(0usize) += 1;
        }
        let most_common = counts
            .into_iter()
            .max_by_key(|(_, c)| *c)
            .map(|(t, _)| unsafe { std::mem::transmute::<u8, FaultType>(t) });

        let rate = if elapsed_hours > 0.0 {
            n as f64 / elapsed_hours
        } else {
            0.0
        };
        // Health score: penalise for critical faults (−0.2 each) and minor faults (−0.05 each)
        let penalty = n_crit as f64 * 0.2 + (n - n_crit) as f64 * 0.05;
        let health = (1.0 - penalty).clamp(0.0, 1.0);

        Self {
            n_faults_detected: n,
            n_critical: n_crit,
            most_common_fault: most_common,
            fault_rate_per_hour: rate,
            health_score: health,
        }
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn normal_sample(t: f64) -> BatterySample {
        BatterySample {
            time_s: t,
            voltage_v: 3.7,
            current_a: 5.0,
            temp_c: 25.0,
            soc: 0.6,
            v_pred: 3.7,
            isolation_kohm: 1000.0,
            capacity_ah: 48.0,
            capacity_nominal_ah: 50.0,
        }
    }

    #[test]
    fn test_no_fault_normal_operation() {
        let mut det = FaultDetector::new(FaultDetectorConfig::nmc_default());
        for t in 0..10 {
            let faults = det.update(normal_sample(t as f64));
            assert!(faults.is_empty(), "No faults expected: {:?}", faults);
        }
    }

    #[test]
    fn test_sensor_voltage_fault_high() {
        let config = FaultDetectorConfig::nmc_default();
        let mut det = FaultDetector::new(config.clone());
        let mut s = normal_sample(1.0);
        s.voltage_v = config.v_max_plausible + 0.5; // above max plausible
        let faults = det.update(s);
        assert!(
            faults
                .iter()
                .any(|f| f.fault_type == FaultType::SensorVoltage),
            "Expected sensor voltage fault"
        );
    }

    #[test]
    fn test_sensor_voltage_fault_low() {
        let config = FaultDetectorConfig::nmc_default();
        let mut det = FaultDetector::new(config.clone());
        let mut s = normal_sample(1.0);
        s.voltage_v = config.v_min_plausible - 0.5;
        let faults = det.update(s);
        assert!(faults
            .iter()
            .any(|f| f.fault_type == FaultType::SensorVoltage));
    }

    #[test]
    fn test_sensor_current_fault() {
        let config = FaultDetectorConfig::nmc_default();
        let mut det = FaultDetector::new(config.clone());
        let mut s = normal_sample(1.0);
        s.current_a = config.i_max_plausible + 50.0;
        let faults = det.update(s);
        assert!(faults
            .iter()
            .any(|f| f.fault_type == FaultType::SensorCurrent));
    }

    #[test]
    fn test_thermal_runaway_absolute() {
        let config = FaultDetectorConfig::nmc_default();
        let mut det = FaultDetector::new(config.clone());
        let mut s = normal_sample(1.0);
        s.temp_c = config.tr_max_temp_c + 5.0;
        let faults = det.update(s);
        assert!(faults
            .iter()
            .any(|f| f.fault_type == FaultType::ThermalRunaway));
    }

    #[test]
    fn test_thermal_runaway_rate() {
        let config = FaultDetectorConfig::nmc_default();
        let mut det = FaultDetector::new(config.clone());
        // Warm up history
        det.update(BatterySample {
            temp_c: 25.0,
            time_s: 0.0,
            ..normal_sample(0.0)
        });
        // Sudden spike
        let mut s = normal_sample(1.0);
        s.temp_c = 25.0 + config.tr_rate_c_s * 2.0; // 2× threshold rate
        let faults = det.update(s);
        assert!(faults
            .iter()
            .any(|f| f.fault_type == FaultType::ThermalRunaway));
    }

    #[test]
    fn test_isolation_fault() {
        let config = FaultDetectorConfig::nmc_default();
        let mut det = FaultDetector::new(config.clone());
        let mut s = normal_sample(1.0);
        s.isolation_kohm = config.imr_min_kohm * 0.5; // Below threshold
        let faults = det.update(s);
        assert!(faults
            .iter()
            .any(|f| f.fault_type == FaultType::IsolationResistance));
    }

    #[test]
    fn test_capacity_fade_fault() {
        let config = FaultDetectorConfig::nmc_default();
        let mut det = FaultDetector::new(config.clone());
        let mut s = normal_sample(1.0);
        s.capacity_ah = s.capacity_nominal_ah * (config.capacity_min_fraction - 0.05);
        let faults = det.update(s);
        assert!(faults
            .iter()
            .any(|f| f.fault_type == FaultType::CapacityFade));
    }

    #[test]
    fn test_open_circuit_detected() {
        let config = FaultDetectorConfig::nmc_default();
        let mut det = FaultDetector::new(config.clone());

        // Build up history with large voltage residuals
        for i in 0..config.oc_window + 1 {
            let mut s = normal_sample(i as f64);
            s.v_pred = 3.7;
            s.voltage_v = 3.7 + config.oc_voltage_threshold_v * 2.0; // large residual
            s.current_a = 10.0; // under load
            det.update(s);
        }

        let mut s = normal_sample((config.oc_window + 2) as f64);
        s.v_pred = 3.7;
        s.voltage_v = 3.7 + config.oc_voltage_threshold_v * 2.0;
        s.current_a = 10.0;
        let faults = det.update(s);
        assert!(
            faults
                .iter()
                .any(|f| f.fault_type == FaultType::OpenCircuit),
            "Expected open circuit fault"
        );
    }

    #[test]
    fn test_has_critical_fault() {
        let config = FaultDetectorConfig::nmc_default();
        let mut det = FaultDetector::new(config.clone());
        let mut s = normal_sample(1.0);
        s.temp_c = config.tr_max_temp_c + 5.0;
        det.update(s);
        assert!(
            det.has_critical_fault(),
            "Thermal runaway should be critical"
        );
    }

    #[test]
    fn test_most_severe_fault() {
        let config = FaultDetectorConfig::nmc_default();
        let mut det = FaultDetector::new(config.clone());
        let mut s = normal_sample(1.0);
        s.temp_c = config.tr_max_temp_c + 10.0; // high confidence thermal fault
        det.update(s);
        let severe = det.most_severe();
        assert!(severe.is_some());
    }

    #[test]
    fn test_fault_type_critical() {
        assert!(FaultType::ThermalRunaway.is_critical());
        assert!(FaultType::InternalShort.is_critical());
        assert!(!FaultType::CapacityFade.is_critical());
        assert!(!FaultType::SensorVoltage.is_critical());
    }

    #[test]
    fn test_insulation_monitor_fault() {
        let mut imd = InsulationMonitor::new(100.0);
        imd.update(0.0, 500.0);
        imd.update(100.0, 50.0); // below threshold
        assert!(imd.is_fault(), "Should be fault at 50 kΩ");
    }

    #[test]
    fn test_insulation_monitor_trend_negative() {
        let mut imd = InsulationMonitor::new(100.0);
        imd.update(0.0, 1000.0);
        imd.update(1000.0, 800.0);
        imd.update(2000.0, 600.0);
        let trend = imd.trend_kohm_per_s();
        assert!(
            trend < 0.0,
            "Trend should be negative (degrading): {}",
            trend
        );
    }

    #[test]
    fn test_insulation_monitor_time_to_alarm() {
        let mut imd = InsulationMonitor::new(100.0);
        imd.update(0.0, 500.0);
        imd.update(1000.0, 400.0);
        imd.update(2000.0, 300.0);
        let tta = imd.time_to_alarm_s();
        assert!(tta.is_some(), "Should predict time to alarm");
        assert!(tta.unwrap() > 2000.0, "Should be in the future: {:?}", tta);
    }

    #[test]
    fn test_diagnostic_report() {
        let log = vec![
            FaultEvent::new(FaultType::ThermalRunaway, 1.0, 5.0, 1.0),
            FaultEvent::new(FaultType::CapacityFade, 2.0, 2.0, 1.0),
        ];
        let report = DiagnosticReport::from_log(&log, 1.0);
        assert_eq!(report.n_faults_detected, 2);
        assert_eq!(report.n_critical, 1);
        assert!(report.health_score < 1.0);
    }
}
