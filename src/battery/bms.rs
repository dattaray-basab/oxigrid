/// Battery Management System (BMS) coordination layer.
///
/// Implements:
/// - **Cell balancing**: passive (shunt resistor) and active (energy redistribution)
/// - **State-of-Health (SoH)** estimation: capacity test + resistance rise method
/// - **Protection logic**: OV/UV/OT/UT/OC/USC trip with hysteresis
/// - **BMS state machine**: idle/charge/discharge/balance/fault/shutdown
/// - **Pre-charge control**: inrush current limiting for capacitive loads
///
/// # References
/// - Andrea, D., "Battery Management Systems for Large Lithium-Ion Battery Packs", Artech 2010
/// - Plett, G.L., "Battery Management Systems Vol. 1 & 2", Artech 2015
/// - IEC 62619:2022 — Safety requirements for secondary lithium cells/batteries
use crate::units::{Current, Temperature, Voltage};
use serde::{Deserialize, Serialize};

// ─────────────────────────────────────────────────────────────────────────────
// Protection thresholds
// ─────────────────────────────────────────────────────────────────────────────

/// BMS protection thresholds for a single cell type.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BmsThresholds {
    /// Over-voltage trip level [V]
    pub ov_trip_v: f64,
    /// Over-voltage release level [V] (hysteresis)
    pub ov_release_v: f64,
    /// Under-voltage trip level [V]
    pub uv_trip_v: f64,
    /// Under-voltage release level [V]
    pub uv_release_v: f64,
    /// Over-temperature trip [°C]
    pub ot_trip_c: f64,
    /// Over-temperature release [°C]
    pub ot_release_c: f64,
    /// Under-temperature (charge inhibit) [°C]
    pub ut_charge_inhibit_c: f64,
    /// Over-current (discharge) trip [A]
    pub oc_discharge_trip_a: f64,
    /// Over-current (charge) trip [A]
    pub oc_charge_trip_a: f64,
    /// Short-circuit detection threshold [A] (instantaneous)
    pub sc_trip_a: f64,
}

impl BmsThresholds {
    /// Typical thresholds for LiFePO4 cells (3.2 V nominal).
    pub fn lfp_default() -> Self {
        Self {
            ov_trip_v: 3.65,
            ov_release_v: 3.55,
            uv_trip_v: 2.80,
            uv_release_v: 2.90,
            ot_trip_c: 60.0,
            ot_release_c: 55.0,
            ut_charge_inhibit_c: 0.0,
            oc_discharge_trip_a: 200.0,
            oc_charge_trip_a: 100.0,
            sc_trip_a: 500.0,
        }
    }

    /// Typical thresholds for NMC cells (3.7 V nominal).
    pub fn nmc_default() -> Self {
        Self {
            ov_trip_v: 4.25,
            ov_release_v: 4.15,
            uv_trip_v: 3.00,
            uv_release_v: 3.10,
            ot_trip_c: 55.0,
            ot_release_c: 50.0,
            ut_charge_inhibit_c: 5.0,
            oc_discharge_trip_a: 150.0,
            oc_charge_trip_a: 80.0,
            sc_trip_a: 400.0,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Fault flags
// ─────────────────────────────────────────────────────────────────────────────

/// Active BMS fault flags (bitmask-style struct).
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize, PartialEq)]
pub struct FaultFlags {
    pub over_voltage: bool,
    pub under_voltage: bool,
    pub over_temperature: bool,
    pub under_temperature_charge: bool,
    pub over_current_discharge: bool,
    pub over_current_charge: bool,
    pub short_circuit: bool,
    pub cell_imbalance: bool,
    pub comm_fault: bool,
}

impl FaultFlags {
    /// Returns `true` if any fault is active.
    pub fn any_fault(&self) -> bool {
        self.over_voltage
            || self.under_voltage
            || self.over_temperature
            || self.under_temperature_charge
            || self.over_current_discharge
            || self.over_current_charge
            || self.short_circuit
            || self.cell_imbalance
            || self.comm_fault
    }

    /// Returns `true` if a hard-fault (trip-critical) is active.
    pub fn hard_fault(&self) -> bool {
        self.over_voltage
            || self.under_voltage
            || self.over_temperature
            || self.short_circuit
            || self.over_current_discharge
            || self.over_current_charge
    }

    /// Count total active faults.
    pub fn count(&self) -> u32 {
        [
            self.over_voltage,
            self.under_voltage,
            self.over_temperature,
            self.under_temperature_charge,
            self.over_current_discharge,
            self.over_current_charge,
            self.short_circuit,
            self.cell_imbalance,
            self.comm_fault,
        ]
        .iter()
        .filter(|&&f| f)
        .count() as u32
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// BMS state machine
// ─────────────────────────────────────────────────────────────────────────────

/// BMS operating mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BmsMode {
    /// Pack is idle (contactors closed, no current)
    Idle,
    /// Pre-charge sequence (soft-start through pre-charge resistor)
    Precharge,
    /// Normal discharge mode
    Discharge,
    /// Normal charge mode
    Charge,
    /// Active cell balancing ongoing
    Balancing,
    /// Fault detected — contactors may be open
    Fault,
    /// Emergency shutdown (thermal runaway or SC)
    Shutdown,
}

/// BMS command output (to power electronics / contactors).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BmsCommand {
    /// Continue current operation
    Continue,
    /// Open main contactor — stop all current flow
    OpenContactor,
    /// Close main contactor — enable current flow
    CloseContactor,
    /// Enable pre-charge path
    EnablePrecharge,
    /// Reduce charge current (thermal derating)
    DerateCharge,
    /// Reduce discharge current (thermal derating)
    DerateDischarge,
    /// Enable passive balancing on given cells
    EnableBalancing,
}

/// Per-cell measurement snapshot.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CellMeasurement {
    pub cell_id: usize,
    pub voltage: Voltage,
    pub temperature: Temperature,
    pub current: Current,
}

/// BMS state evaluation result.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BmsEvaluation {
    pub mode: BmsMode,
    pub faults: FaultFlags,
    pub command: BmsCommand,
    /// Recommended charge current limit [A] (after derating)
    pub charge_current_limit_a: f64,
    /// Recommended discharge current limit [A]
    pub discharge_current_limit_a: f64,
    /// Min cell voltage seen [V]
    pub v_min: f64,
    /// Max cell voltage seen [V]
    pub v_max: f64,
    /// Max cell temperature [°C]
    pub t_max: f64,
    /// Cell imbalance (Vmax - Vmin) [mV]
    pub imbalance_mv: f64,
}

/// Evaluate BMS protection for a set of cell measurements.
///
/// Applies OV/UV/OT/OC checks with hysteresis and returns the recommended
/// operating command and current limits.
pub fn evaluate_protection(
    cells: &[CellMeasurement],
    pack_current: Current,
    thresholds: &BmsThresholds,
    prior_faults: &FaultFlags,
) -> BmsEvaluation {
    if cells.is_empty() {
        return BmsEvaluation {
            mode: BmsMode::Fault,
            faults: FaultFlags {
                comm_fault: true,
                ..Default::default()
            },
            command: BmsCommand::OpenContactor,
            charge_current_limit_a: 0.0,
            discharge_current_limit_a: 0.0,
            v_min: 0.0,
            v_max: 0.0,
            t_max: 0.0,
            imbalance_mv: 0.0,
        };
    }

    let v_min = cells
        .iter()
        .map(|c| c.voltage.0)
        .fold(f64::INFINITY, f64::min);
    let v_max = cells
        .iter()
        .map(|c| c.voltage.0)
        .fold(f64::NEG_INFINITY, f64::max);
    let t_max = cells
        .iter()
        .map(|c| c.temperature.0)
        .fold(f64::NEG_INFINITY, f64::max);
    let imbalance_mv = (v_max - v_min) * 1000.0;
    let i = pack_current.0; // positive = discharge

    let mut faults = *prior_faults;

    // OV with hysteresis
    if v_max >= thresholds.ov_trip_v {
        faults.over_voltage = true;
    } else if v_max <= thresholds.ov_release_v {
        faults.over_voltage = false;
    }

    // UV with hysteresis
    if v_min <= thresholds.uv_trip_v {
        faults.under_voltage = true;
    } else if v_min >= thresholds.uv_release_v {
        faults.under_voltage = false;
    }

    // OT with hysteresis
    if t_max >= thresholds.ot_trip_c {
        faults.over_temperature = true;
    } else if t_max <= thresholds.ot_release_c {
        faults.over_temperature = false;
    }

    // UT charge inhibit
    faults.under_temperature_charge = t_max < thresholds.ut_charge_inhibit_c;

    // OC instantaneous
    faults.over_current_discharge = i >= thresholds.oc_discharge_trip_a;

    faults.over_current_charge = i <= -thresholds.oc_charge_trip_a;

    // Short circuit
    faults.short_circuit = i.abs() >= thresholds.sc_trip_a;

    // Imbalance flag (>50 mV)
    faults.cell_imbalance = imbalance_mv > 50.0;

    // Determine command and limits
    let (mode, command, clim, dlim) = if faults.short_circuit || faults.over_temperature {
        (BmsMode::Shutdown, BmsCommand::OpenContactor, 0.0, 0.0)
    } else if faults.hard_fault() {
        (BmsMode::Fault, BmsCommand::OpenContactor, 0.0, 0.0)
    } else {
        // Thermal derating: linearly reduce limits as temperature rises above 45°C
        let derate = if t_max > 45.0 {
            (1.0 - (t_max - 45.0) / (thresholds.ot_trip_c - 45.0)).max(0.0)
        } else {
            1.0
        };

        let clim = if faults.under_temperature_charge {
            0.0
        } else {
            thresholds.oc_charge_trip_a * derate
        };
        let dlim = thresholds.oc_discharge_trip_a * derate;

        let cmd = if faults.cell_imbalance {
            BmsCommand::EnableBalancing
        } else if derate < 1.0 && i > 0.0 {
            BmsCommand::DerateDischarge
        } else if derate < 1.0 && i < 0.0 {
            BmsCommand::DerateCharge
        } else {
            BmsCommand::Continue
        };

        let mode = if faults.cell_imbalance {
            BmsMode::Balancing
        } else if i > 0.0 {
            BmsMode::Discharge
        } else if i < 0.0 {
            BmsMode::Charge
        } else {
            BmsMode::Idle
        };

        (mode, cmd, clim, dlim)
    };

    BmsEvaluation {
        mode,
        faults,
        command,
        charge_current_limit_a: clim,
        discharge_current_limit_a: dlim,
        v_min,
        v_max,
        t_max,
        imbalance_mv,
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Cell balancing
// ─────────────────────────────────────────────────────────────────────────────

/// Passive balancing decision for one cell.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct BalancingDecision {
    pub cell_id: usize,
    /// Enable shunt resistor bypass for this cell
    pub shunt_enable: bool,
    /// Estimated balancing current [A] (through shunt)
    pub balance_current_a: f64,
    /// Energy dissipated in shunt [W] (heat)
    pub shunt_power_w: f64,
}

/// Passive (dissipative) cell balancing: top-balance strategy.
///
/// Cells above the minimum voltage by more than `threshold_mv` are bled
/// through a bypass resistor. This is the simplest approach used in most
/// commercial BMS packs.
///
/// # Arguments
/// - `cell_voltages` — slice of (cell_id, voltage_v) pairs
/// - `threshold_mv`  — minimum imbalance to trigger balancing [mV]
/// - `shunt_r_ohm`   — bypass shunt resistance [Ω]
pub fn passive_balance(
    cell_voltages: &[(usize, f64)],
    threshold_mv: f64,
    shunt_r_ohm: f64,
) -> Vec<BalancingDecision> {
    if cell_voltages.is_empty() {
        return vec![];
    }
    let v_min = cell_voltages
        .iter()
        .map(|&(_, v)| v)
        .fold(f64::INFINITY, f64::min);
    let thresh = threshold_mv / 1000.0;

    cell_voltages
        .iter()
        .map(|&(id, v)| {
            let excess = v - v_min;
            let shunt_enable = excess > thresh;
            let balance_current_a = if shunt_enable { v / shunt_r_ohm } else { 0.0 };
            let shunt_power_w = balance_current_a * v;
            BalancingDecision {
                cell_id: id,
                shunt_enable,
                balance_current_a,
                shunt_power_w,
            }
        })
        .collect()
}

/// Active (lossless) balancing: charge shuttle from high to low cell.
///
/// Returns pairs of (source_cell_id, target_cell_id) for the switched-capacitor
/// or inductor-based energy transfer. Matches highest-to-lowest cell pairs.
pub fn active_balance_pairs(
    cell_voltages: &[(usize, f64)],
    threshold_mv: f64,
) -> Vec<(usize, usize)> {
    if cell_voltages.len() < 2 {
        return vec![];
    }
    let thresh = threshold_mv / 1000.0;
    let mut sorted = cell_voltages.to_vec();
    sorted.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

    let mut pairs = vec![];
    let n = sorted.len();
    for i in 0..n / 2 {
        let (hi_id, hi_v) = sorted[i];
        let (lo_id, lo_v) = sorted[n - 1 - i];
        if hi_v - lo_v > thresh {
            pairs.push((hi_id, lo_id));
        }
    }
    pairs
}

// ─────────────────────────────────────────────────────────────────────────────
// State-of-Health estimation
// ─────────────────────────────────────────────────────────────────────────────

/// SoH estimate from a full capacity test.
///
/// SoH_capacity = Q_measured / Q_nominal × 100%
///
/// # Arguments
/// - `q_measured_ah` — measured capacity during full charge/discharge [Ah]
/// - `q_nominal_ah`  — original rated capacity [Ah]
pub fn soh_capacity(q_measured_ah: f64, q_nominal_ah: f64) -> f64 {
    if q_nominal_ah <= 0.0 {
        return 0.0;
    }
    (q_measured_ah / q_nominal_ah * 100.0).clamp(0.0, 100.0)
}

/// SoH estimate from internal resistance rise.
///
/// SoH_resistance = (R_EOL - R_now) / (R_EOL - R_BOL) × 100%
///
/// where R_EOL is the resistance at end-of-life (typically 2× BOL).
///
/// # Arguments
/// - `r_now_ohm`  — current DC internal resistance [Ω]
/// - `r_bol_ohm`  — beginning-of-life resistance [Ω]
/// - `r_eol_ohm`  — end-of-life resistance threshold [Ω]
pub fn soh_resistance(r_now_ohm: f64, r_bol_ohm: f64, r_eol_ohm: f64) -> f64 {
    if (r_eol_ohm - r_bol_ohm).abs() < 1e-12 {
        return 100.0;
    }
    let soh = (r_eol_ohm - r_now_ohm) / (r_eol_ohm - r_bol_ohm) * 100.0;
    soh.clamp(0.0, 100.0)
}

/// Blended SoH combining capacity and resistance estimates.
///
/// Weighted average: `alpha` weight on capacity, `(1-alpha)` on resistance.
pub fn soh_blended(soh_cap: f64, soh_res: f64, alpha: f64) -> f64 {
    let alpha = alpha.clamp(0.0, 1.0);
    alpha * soh_cap + (1.0 - alpha) * soh_res
}

/// Remaining useful life estimate (linear extrapolation from fade rate).
///
/// Returns estimated remaining cycles until SoH < `eol_threshold_pct`.
///
/// # Arguments
/// - `current_soh_pct` — current SoH [%]
/// - `fade_rate_per_cycle` — SoH fade per equivalent full cycle [% per cycle]
/// - `eol_threshold_pct` — SoH at end of life (typically 80%)
pub fn remaining_useful_life_cycles(
    current_soh_pct: f64,
    fade_rate_per_cycle: f64,
    eol_threshold_pct: f64,
) -> Option<f64> {
    if fade_rate_per_cycle <= 0.0 || current_soh_pct <= eol_threshold_pct {
        return None;
    }
    Some((current_soh_pct - eol_threshold_pct) / fade_rate_per_cycle)
}

// ─────────────────────────────────────────────────────────────────────────────
// Pre-charge controller
// ─────────────────────────────────────────────────────────────────────────────

/// Pre-charge state for capacitive load soft-start.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PrechargeState {
    /// Time elapsed in pre-charge [s]
    pub elapsed_s: f64,
    /// Target bus voltage [V]
    pub v_target: f64,
    /// Current bus voltage [V]
    pub v_bus: f64,
    /// Pre-charge resistance [Ω]
    pub r_precharge_ohm: f64,
    /// Bus capacitance [F]
    pub c_bus_f: f64,
    /// Completion threshold: fraction of v_target (e.g. 0.95)
    pub completion_threshold: f64,
}

impl PrechargeState {
    /// Advance pre-charge simulation by `dt` seconds.
    /// Returns `true` when pre-charge is complete.
    pub fn step(&mut self, dt: f64) -> bool {
        // RC charging: v(t) = V_target * (1 - exp(-t/RC))
        let tau = self.r_precharge_ohm * self.c_bus_f;
        self.elapsed_s += dt;
        self.v_bus = self.v_target * (1.0 - (-self.elapsed_s / tau).exp());
        self.v_bus >= self.v_target * self.completion_threshold
    }

    /// Pre-charge inrush current [A] at current v_bus.
    pub fn inrush_current_a(&self) -> f64 {
        (self.v_target - self.v_bus) / self.r_precharge_ohm
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_cells(voltages_v: &[f64], temp_c: f64) -> Vec<CellMeasurement> {
        voltages_v
            .iter()
            .enumerate()
            .map(|(i, &v)| CellMeasurement {
                cell_id: i,
                voltage: Voltage(v),
                temperature: Temperature(temp_c),
                current: Current(0.0),
            })
            .collect()
    }

    // ── Protection tests ─────────────────────────────────────────────────────

    #[test]
    fn test_no_fault_normal_conditions() {
        let cells = make_cells(&[3.30, 3.31, 3.29, 3.30], 25.0);
        let thresh = BmsThresholds::lfp_default();
        let prior = FaultFlags::default();
        let eval = evaluate_protection(&cells, Current(50.0), &thresh, &prior);
        assert!(
            !eval.faults.any_fault(),
            "Normal conditions should be fault-free"
        );
        assert_eq!(eval.command, BmsCommand::Continue);
    }

    #[test]
    fn test_over_voltage_trip() {
        let cells = make_cells(&[3.30, 3.30, 3.70, 3.30], 25.0); // cell 2 OV
        let thresh = BmsThresholds::lfp_default();
        let prior = FaultFlags::default();
        let eval = evaluate_protection(&cells, Current(10.0), &thresh, &prior);
        assert!(eval.faults.over_voltage, "Should trip on OV");
        assert_eq!(eval.command, BmsCommand::OpenContactor);
    }

    #[test]
    fn test_under_voltage_trip() {
        let cells = make_cells(&[3.30, 2.75, 3.30, 3.30], 25.0); // cell 1 UV
        let thresh = BmsThresholds::lfp_default();
        let prior = FaultFlags::default();
        let eval = evaluate_protection(&cells, Current(50.0), &thresh, &prior);
        assert!(eval.faults.under_voltage, "Should trip on UV");
    }

    #[test]
    fn test_over_temperature_trip() {
        let cells = make_cells(&[3.30; 4], 65.0); // OT
        let thresh = BmsThresholds::lfp_default();
        let prior = FaultFlags::default();
        let eval = evaluate_protection(&cells, Current(10.0), &thresh, &prior);
        assert!(eval.faults.over_temperature);
        assert_eq!(eval.mode, BmsMode::Shutdown);
    }

    #[test]
    fn test_short_circuit_trip() {
        let cells = make_cells(&[3.30; 4], 25.0);
        let thresh = BmsThresholds::lfp_default();
        let prior = FaultFlags::default();
        let eval = evaluate_protection(&cells, Current(600.0), &thresh, &prior); // > sc_trip_a
        assert!(eval.faults.short_circuit);
        assert_eq!(eval.mode, BmsMode::Shutdown);
    }

    #[test]
    fn test_over_voltage_hysteresis() {
        let thresh = BmsThresholds::lfp_default();
        // Fault was set previously
        let prior = FaultFlags {
            over_voltage: true,
            ..Default::default()
        };
        // Voltage now between release (3.55) and trip (3.65): fault should persist
        let cells = make_cells(&[3.60; 4], 25.0);
        let eval = evaluate_protection(&cells, Current(0.0), &thresh, &prior);
        assert!(
            eval.faults.over_voltage,
            "OV should persist in hysteresis band"
        );

        // Voltage drops below release: fault should clear
        let cells2 = make_cells(&[3.50; 4], 25.0);
        let eval2 = evaluate_protection(&cells2, Current(0.0), &thresh, &eval.faults);
        assert!(
            !eval2.faults.over_voltage,
            "OV should clear below release threshold"
        );
    }

    #[test]
    fn test_thermal_derating() {
        let cells = make_cells(&[3.30; 4], 52.0); // above 45°C, below OT trip (60°C)
        let thresh = BmsThresholds::lfp_default();
        let prior = FaultFlags::default();
        let eval = evaluate_protection(&cells, Current(80.0), &thresh, &prior);
        assert!(
            eval.discharge_current_limit_a < thresh.oc_discharge_trip_a,
            "Current limit should be derated at high temp"
        );
        assert!(
            !eval.faults.over_temperature,
            "Should not be OT fault at 52°C"
        );
    }

    #[test]
    fn test_cell_imbalance_flag() {
        let cells = make_cells(&[3.20, 3.40, 3.20, 3.20], 25.0); // 200 mV spread
        let thresh = BmsThresholds::lfp_default();
        let prior = FaultFlags::default();
        let eval = evaluate_protection(&cells, Current(0.0), &thresh, &prior);
        assert!(eval.faults.cell_imbalance, "Should flag imbalance > 50 mV");
        assert_eq!(eval.mode, BmsMode::Balancing);
    }

    #[test]
    fn test_empty_cells_comm_fault() {
        let thresh = BmsThresholds::lfp_default();
        let prior = FaultFlags::default();
        let eval = evaluate_protection(&[], Current(0.0), &thresh, &prior);
        assert!(eval.faults.comm_fault);
    }

    // ── Balancing tests ──────────────────────────────────────────────────────

    #[test]
    fn test_passive_balance_triggers_high_cell() {
        let cells = vec![(0, 3.30), (1, 3.40), (2, 3.30)]; // cell 1 is 100 mV high
        let decisions = passive_balance(&cells, 20.0, 10.0);
        assert!(decisions[1].shunt_enable, "High cell should be bypassed");
        assert!(!decisions[0].shunt_enable);
        assert!(!decisions[2].shunt_enable);
    }

    #[test]
    fn test_passive_balance_no_action_balanced() {
        let cells = vec![(0, 3.30), (1, 3.31), (2, 3.30)]; // only 10 mV spread
        let decisions = passive_balance(&cells, 20.0, 10.0);
        assert!(
            !decisions.iter().any(|d| d.shunt_enable),
            "No balancing for < threshold"
        );
    }

    #[test]
    fn test_active_balance_pairs_high_to_low() {
        let cells = vec![(0, 3.45), (1, 3.30), (2, 3.42), (3, 3.28)];
        let pairs = active_balance_pairs(&cells, 50.0);
        assert!(!pairs.is_empty(), "Should identify transfer pairs");
        // Source should always be higher voltage than target
        for (src, tgt) in &pairs {
            let v_src = cells[*src].1;
            let v_tgt = cells[*tgt].1;
            assert!(
                v_src > v_tgt,
                "Source ({v_src:.3}) should be higher than target ({v_tgt:.3})"
            );
        }
    }

    #[test]
    fn test_active_balance_no_pairs_balanced() {
        let cells = vec![(0, 3.30), (1, 3.31)]; // < 50 mV threshold
        let pairs = active_balance_pairs(&cells, 50.0);
        assert!(pairs.is_empty());
    }

    // ── SoH tests ────────────────────────────────────────────────────────────

    #[test]
    fn test_soh_capacity_new_cell() {
        assert!((soh_capacity(100.0, 100.0) - 100.0).abs() < 1e-10);
    }

    #[test]
    fn test_soh_capacity_degraded() {
        assert!((soh_capacity(85.0, 100.0) - 85.0).abs() < 1e-10);
    }

    #[test]
    fn test_soh_resistance_new() {
        // R_now = R_BOL → SoH = 100%
        assert!((soh_resistance(0.05, 0.05, 0.10) - 100.0).abs() < 1e-10);
    }

    #[test]
    fn test_soh_resistance_eol() {
        // R_now = R_EOL → SoH = 0%
        assert!((soh_resistance(0.10, 0.05, 0.10) - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_soh_blended() {
        let soh = soh_blended(90.0, 80.0, 0.5);
        assert!((soh - 85.0).abs() < 1e-10);
    }

    #[test]
    fn test_rul_cycles() {
        let rul = remaining_useful_life_cycles(95.0, 0.01, 80.0).unwrap();
        assert!((rul - 1500.0).abs() < 1e-6, "RUL={rul:.1}");
    }

    #[test]
    fn test_rul_already_eol() {
        let rul = remaining_useful_life_cycles(75.0, 0.01, 80.0);
        assert!(rul.is_none(), "Already past EOL should return None");
    }

    // ── Pre-charge tests ─────────────────────────────────────────────────────

    #[test]
    fn test_precharge_completes() {
        let mut pc = PrechargeState {
            elapsed_s: 0.0,
            v_target: 800.0,
            v_bus: 0.0,
            r_precharge_ohm: 100.0,
            c_bus_f: 0.01,
            completion_threshold: 0.95,
        };
        let tau = pc.r_precharge_ohm * pc.c_bus_f; // 1 s
                                                   // After 3τ ≈ 95% of target
        for _ in 0..300 {
            if pc.step(tau / 100.0) {
                break;
            }
        }
        assert!(
            pc.v_bus >= pc.v_target * 0.95,
            "Pre-charge should reach 95%: {:.1}",
            pc.v_bus
        );
    }

    #[test]
    fn test_precharge_inrush_decreases() {
        let mut pc = PrechargeState {
            elapsed_s: 0.0,
            v_target: 400.0,
            v_bus: 0.0,
            r_precharge_ohm: 50.0,
            c_bus_f: 0.005,
            completion_threshold: 0.95,
        };
        let i0 = pc.inrush_current_a();
        pc.step(0.1);
        let i1 = pc.inrush_current_a();
        assert!(
            i1 < i0,
            "Inrush current should decrease as capacitor charges"
        );
    }

    #[test]
    fn test_fault_flags_count() {
        let f = FaultFlags {
            over_voltage: true,
            under_voltage: true,
            ..Default::default()
        };
        assert_eq!(f.count(), 2);
    }

    #[test]
    fn test_fault_flags_hard_fault() {
        let f = FaultFlags {
            over_voltage: true,
            ..Default::default()
        };
        assert!(f.hard_fault());
        let f2 = FaultFlags {
            cell_imbalance: true,
            comm_fault: false,
            ..Default::default()
        };
        assert!(!f2.hard_fault());
    }
}
