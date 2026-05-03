//! Auto-generated module
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

use crate::error::OxiGridError;

use super::types_3::{
    CapacitorStatus, CoordinatedVoltageController, FeederVoltageProfile, TapAction,
};

/// On-Load Tap Changer controller.
///
/// Models an OLTC transformer with discrete tap positions, deadband control,
/// time delay, and daily operation limits.
#[derive(Debug, Clone)]
pub struct OltcController {
    /// Unique device identifier.
    pub id: usize,
    /// Bus ID on the controlled (secondary) side.
    pub bus_id: usize,
    /// Minimum tap position (e.g. -16).
    pub min_tap: i32,
    /// Maximum tap position (e.g. +16).
    pub max_tap: i32,
    /// Current tap position.
    pub current_tap: i32,
    /// Per-unit voltage change per tap step (e.g. 0.00625).
    pub tap_step_pu: f64,
    /// Target voltage setpoint in per-unit.
    pub target_voltage_pu: f64,
    /// Deadband: no action if |V − V_target| < deadband.
    pub deadband_pu: f64,
    /// Operating delay in seconds before tap change executes.
    pub time_delay_s: f64,
    /// Maximum tap operations per day.
    pub max_operations_per_day: u32,
    /// Number of tap operations performed today.
    pub daily_operations: u32,
    /// Control mode.
    pub control_mode: TapControlMode,
    /// Line-drop compensation resistance in per-unit (LDC mode).
    pub line_drop_r_pu: f64,
    /// Line-drop compensation reactance in per-unit (LDC mode).
    pub line_drop_x_pu: f64,
}
impl OltcController {
    /// Create a new OLTC controller with default settings.
    pub fn new(id: usize, bus_id: usize, min_tap: i32, max_tap: i32) -> Self {
        Self {
            id,
            bus_id,
            min_tap,
            max_tap,
            current_tap: 0,
            tap_step_pu: 0.00625,
            target_voltage_pu: 1.0,
            deadband_pu: 0.01,
            time_delay_s: 30.0,
            max_operations_per_day: 20,
            daily_operations: 0,
            control_mode: TapControlMode::Automatic,
            line_drop_r_pu: 0.0,
            line_drop_x_pu: 0.0,
        }
    }
    /// Returns the voltage transformation ratio: `1.0 + current_tap × tap_step_pu`.
    pub fn voltage_ratio(&self) -> f64 {
        1.0 + self.current_tap as f64 * self.tap_step_pu
    }
    /// Compute the required tap change given a measured voltage.
    ///
    /// Returns `Some(+1)` to tap up, `Some(-1)` to tap down, or `None` if
    /// the voltage is within the deadband.
    pub fn compute_action(&self, measured_voltage_pu: f64) -> Option<i32> {
        let error = measured_voltage_pu - self.target_voltage_pu;
        if error.abs() <= self.deadband_pu {
            None
        } else if error < 0.0 {
            Some(1)
        } else {
            Some(-1)
        }
    }
    /// Apply a tap change delta, enforcing tap limits and daily operation limits.
    ///
    /// Returns the new voltage ratio on success, or [`OxiGridError::InvalidParameter`]
    /// when the daily limit is exhausted or the tap is already at the limit.
    pub fn apply_tap(&mut self, delta_tap: i32) -> Result<f64, OxiGridError> {
        if self.daily_operations >= self.max_operations_per_day {
            return Err(OxiGridError::InvalidParameter(format!(
                "OLTC {}: daily operation limit ({}) reached",
                self.id, self.max_operations_per_day
            )));
        }
        let new_tap = (self.current_tap + delta_tap).clamp(self.min_tap, self.max_tap);
        if new_tap == self.current_tap && delta_tap != 0 {
            return Err(OxiGridError::InvalidParameter(format!(
                "OLTC {}: tap already at limit (tap={})",
                self.id, self.current_tap
            )));
        }
        self.current_tap = new_tap;
        self.daily_operations += 1;
        Ok(self.voltage_ratio())
    }
    /// Reset the daily operation counter (call at the start of each new day).
    pub fn reset_daily_counter(&mut self) {
        self.daily_operations = 0;
    }
}
/// Snapshot of bus voltage profile at a given simulation instant.
#[derive(Debug, Clone)]
pub struct VoltageProfile {
    /// Per-unit voltages indexed in the same order as `bus_ids`.
    pub bus_voltages_pu: Vec<f64>,
    /// Ordered list of bus identifiers.
    pub bus_ids: Vec<usize>,
    /// Timestamp in seconds.
    pub timestamp: f64,
    /// Minimum bus voltage across all monitored buses.
    pub min_voltage_pu: f64,
    /// Maximum bus voltage across all monitored buses.
    pub max_voltage_pu: f64,
    /// Number of buses with voltage outside the statutory `[0.95, 1.05]` band.
    pub n_violations: usize,
    /// Voltage unbalance percentage (0.0 for balanced / single-phase data).
    pub voltage_unbalance_pct: f64,
}
/// IEEE C57.15 step-type voltage regulator for the legacy coordinator.
///
/// Models a single-phase or three-phase autotransformer with ±16 × 0.625 %
/// tap range, optional line-drop compensation, and time-delayed tap control.
#[derive(Debug, Clone)]
pub struct StepRegulator {
    /// Unique device identifier.
    pub id: usize,
    /// Source bus index.
    pub bus_from: usize,
    /// Regulated (load-side) bus index.
    pub bus_to: usize,
    /// Rated apparent power `[kVA]`.
    pub rated_kva: f64,
    /// Rated voltage ``kV``.
    pub rated_kv: f64,
    /// Minimum tap position (e.g. -16).
    pub min_tap: i32,
    /// Maximum tap position (e.g. +16).
    pub max_tap: i32,
    /// Voltage change per tap step as a percentage (typically 0.625 %).
    pub step_voltage_pct: f64,
    /// Current tap position.
    pub current_tap: i32,
    /// Voltage setpoint ``pu``.
    pub v_setpoint_pu: f64,
    /// Control deadband (half-width) ``pu``.
    pub bandwidth_pu: f64,
    /// Line drop compensator resistance setting ``pu``.
    pub r_compensator: f64,
    /// Line drop compensator reactance setting ``pu``.
    pub x_compensator: f64,
    /// Time delay before a tap change is executed ``s``.
    pub time_delay_s: f64,
    /// Accumulated time the voltage has been outside the deadband ``s``.
    pub pending_time: f64,
    /// Cumulative number of tap-change operations.
    pub total_operations: usize,
}
impl StepRegulator {
    /// Constructs a regulator with default ±16-step, 0.625 %/step settings.
    pub fn new(id: usize, rated_kva: f64, rated_kv: f64, v_setpoint_pu: f64) -> Self {
        Self {
            id,
            bus_from: 0,
            bus_to: 1,
            rated_kva,
            rated_kv,
            min_tap: -16,
            max_tap: 16,
            step_voltage_pct: 0.625,
            current_tap: 0,
            v_setpoint_pu,
            bandwidth_pu: 0.01,
            r_compensator: 0.0,
            x_compensator: 0.0,
            time_delay_s: 30.0,
            pending_time: 0.0,
            total_operations: 0,
        }
    }
    /// Per-unit tap ratio `a = 1 + tap × step / 100` ``pu``.
    #[inline]
    pub fn tap_ratio(&self) -> f64 {
        1.0 + self.current_tap as f64 * self.step_voltage_pct / 100.0
    }
    /// Voltage at the sensing point after line-drop compensation ``pu``.
    pub fn compensated_voltage(&self, v_meas_pu: f64, i_pu: f64, pf: f64) -> f64 {
        let pf_clamped = pf.clamp(-1.0, 1.0);
        let sin_theta = (1.0 - pf_clamped * pf_clamped).max(0.0).sqrt();
        v_meas_pu - i_pu * (self.r_compensator * pf_clamped + self.x_compensator * sin_theta)
    }
    /// Time-delayed step control — call once per simulation time step `dt_s`.
    pub fn step_control(&mut self, v_controlled_pu: f64, dt_s: f64) -> TapAction {
        let error = v_controlled_pu - self.v_setpoint_pu;
        if error.abs() <= self.bandwidth_pu {
            self.pending_time = 0.0;
            return TapAction::NoChange;
        }
        self.pending_time += dt_s;
        if self.pending_time < self.time_delay_s {
            return TapAction::NoChange;
        }
        self.pending_time = 0.0;
        if error < 0.0 {
            if self.current_tap >= self.max_tap {
                TapAction::AtLimit(format!("regulator {} at max tap {}", self.id, self.max_tap))
            } else {
                self.current_tap += 1;
                self.total_operations += 1;
                TapAction::TapUp
            }
        } else {
            if self.current_tap <= self.min_tap {
                TapAction::AtLimit(format!("regulator {} at min tap {}", self.id, self.min_tap))
            } else {
                self.current_tap -= 1;
                self.total_operations += 1;
                TapAction::TapDown
            }
        }
    }
    /// Achievable regulated-voltage range `(V_min, V_max)` ``pu``.
    pub fn effective_voltage_range(&self) -> (f64, f64) {
        let v_min = 1.0 + self.min_tap as f64 * self.step_voltage_pct / 100.0;
        let v_max = 1.0 + self.max_tap as f64 * self.step_voltage_pct / 100.0;
        (v_min, v_max)
    }
}
/// Simplified greedy Volt-VAR Optimiser for the legacy [`CoordinatedVoltageController`].
#[derive(Debug, Clone)]
pub struct VvoOptimizer {
    /// Target (nominal) voltage ``pu``.
    pub target_voltage_pu: f64,
    /// Weight assigned to voltage deviation squared.
    pub w_voltage: f64,
    /// Weight assigned to estimated feeder losses.
    pub w_losses: f64,
    /// Weight assigned to switching operations.
    pub w_switching: f64,
}
impl VvoOptimizer {
    /// Creates a VVO with default balanced weights.
    pub fn new() -> Self {
        Self {
            target_voltage_pu: 1.0,
            w_voltage: 1.0,
            w_losses: 0.5,
            w_switching: 0.1,
        }
    }
    /// Runs one optimisation pass and returns recommended setpoints.
    pub fn optimize_setpoints(
        &self,
        controller: &mut CoordinatedVoltageController,
        bus_voltages: &[f64],
        load_mw: f64,
    ) -> VvoResult {
        let v_dev_before: f64 = if bus_voltages.is_empty() {
            0.0
        } else {
            bus_voltages
                .iter()
                .map(|&v| (v - self.target_voltage_pu).powi(2))
                .sum::<f64>()
                / bus_voltages.len() as f64
        };
        let mut optimal_v_setpoints: Vec<f64> = Vec::with_capacity(controller.regulators.len());
        for reg in &mut controller.regulators {
            let candidates = [
                reg.v_setpoint_pu - 2.0 * reg.bandwidth_pu,
                reg.v_setpoint_pu - reg.bandwidth_pu,
                reg.v_setpoint_pu,
                reg.v_setpoint_pu + reg.bandwidth_pu,
                reg.v_setpoint_pu + 2.0 * reg.bandwidth_pu,
            ];
            let mut best_sp = reg.v_setpoint_pu;
            let mut best_cost = f64::MAX;
            for &sp in &candidates {
                let sp_clamped = sp.clamp(0.90, 1.10);
                let cost = self.w_voltage * (sp_clamped - self.target_voltage_pu).powi(2)
                    + self.w_switching * 0.1;
                if cost < best_cost {
                    best_cost = cost;
                    best_sp = sp_clamped;
                }
            }
            reg.v_setpoint_pu = best_sp;
            optimal_v_setpoints.push(best_sp);
        }
        let mut optimal_cap_steps: Vec<usize> = Vec::with_capacity(controller.capacitors.len());
        for cap in &mut controller.capacitors {
            let mut best_steps = cap.current_steps;
            let mut best_cost = f64::MAX;
            for s in 0..=cap.steps {
                let q_inj = if cap.steps > 0 {
                    cap.q_rated_mvar * s as f64 / cap.steps as f64
                } else {
                    0.0
                };
                let loss_factor = if controller.base_mva > 0.0 {
                    (load_mw / controller.base_mva - q_inj / controller.base_mva).powi(2)
                } else {
                    0.0
                };
                let switching_cost = (s as i64 - cap.current_steps as i64).unsigned_abs() as f64;
                let cost = self.w_losses * loss_factor + self.w_switching * switching_cost;
                if cost < best_cost {
                    best_cost = cost;
                    best_steps = s;
                }
            }
            cap.current_steps = best_steps;
            optimal_cap_steps.push(best_steps);
        }
        let v_dev_after: f64 = if bus_voltages.is_empty() {
            0.0
        } else {
            bus_voltages
                .iter()
                .map(|&v| (v - self.target_voltage_pu).powi(2))
                .sum::<f64>()
                / bus_voltages.len() as f64
        };
        let voltage_improvement_pu = (v_dev_before - v_dev_after).max(0.0).sqrt();
        let total_q = controller.total_reactive_support_mvar();
        let loss_reduction = if load_mw > 0.0 {
            (total_q / (load_mw + total_q).max(1e-6) * 2.0 * 100.0).min(20.0)
        } else {
            0.0
        };
        VvoResult {
            optimal_v_setpoints,
            optimal_cap_steps,
            estimated_loss_reduction_pct: loss_reduction,
            voltage_improvement_pu,
        }
    }
}
/// Action returned by one control evaluation of a `StepCapacitorBank`.
#[derive(Debug, Clone, PartialEq)]
pub enum CapAction {
    /// No change this step.
    NoChange,
    /// One capacitor step switched in (more capacitive).
    StepIn,
    /// One capacitor step switched out (less capacitive).
    StepOut,
    /// Already at a switching limit.
    AtLimit,
}
/// Static VAR Compensator with first-order dynamic response.
///
/// Implements a proportional droop Q-V characteristic:
/// ```text
/// Q = k_svc × (V_sp − V_meas)   `MVAr`
/// ```
/// clamped to `[q_min_mvar, q_max_mvar]`, with a first-order lag filter
/// characterised by time constant `t2_s` `s`.
#[derive(Debug, Clone)]
pub struct SvcModel {
    /// Unique device identifier.
    pub id: usize,
    /// Bus at which the SVC is connected.
    pub bus: usize,
    /// Inductive (absorbing) reactive-power limit ``MVAr`` (negative).
    pub q_min_mvar: f64,
    /// Capacitive (injecting) reactive-power limit ``MVAr`` (positive).
    pub q_max_mvar: f64,
    /// Voltage setpoint ``pu``.
    pub v_setpoint_pu: f64,
    /// Droop slope as a percentage of the voltage base.
    pub droop_pct: f64,
    /// Proportional gain: `Q_range / droop` `[MVAr/pu]`.
    pub k_svc: f64,
    /// Lead time constant ``s``.
    pub t1_s: f64,
    /// Lag time constant (dominant) ``s``.
    pub t2_s: f64,
    /// Current reactive power output ``MVAr``.
    pub q_output_mvar: f64,
    /// Internal voltage reference ``pu``.
    pub v_ref_pu: f64,
}
impl SvcModel {
    /// Creates a new SVC model with 5 % droop and a 0.1 s lag time constant.
    pub fn new(bus: usize, q_min: f64, q_max: f64, v_setpoint: f64) -> Self {
        let q_range = (q_max - q_min).abs().max(1e-6);
        let droop_pct = 5.0_f64;
        let k_svc = q_range / (droop_pct / 100.0);
        Self {
            id: 0,
            bus,
            q_min_mvar: q_min,
            q_max_mvar: q_max,
            v_setpoint_pu: v_setpoint,
            droop_pct,
            k_svc,
            t1_s: 0.02,
            t2_s: 0.1,
            q_output_mvar: 0.0,
            v_ref_pu: v_setpoint,
        }
    }
    /// Steady-state reactive output from the static Q-V droop curve ``MVAr``.
    pub fn q_from_voltage(&self, v_pu: f64) -> f64 {
        let q_raw = self.k_svc * (self.v_setpoint_pu - v_pu);
        q_raw.clamp(self.q_min_mvar, self.q_max_mvar)
    }
    /// Advances the SVC dynamic state by `dt_s` ``s`` and returns updated Q ``MVAr``.
    pub fn step(&mut self, v_pu: f64, dt_s: f64) -> f64 {
        let q_target = self.q_from_voltage(v_pu);
        let alpha = dt_s / (self.t2_s + dt_s);
        self.q_output_mvar += alpha * (q_target - self.q_output_mvar);
        self.q_output_mvar
    }
    /// Equivalent shunt susceptance ``pu`` at V ≈ 1 pu.
    pub fn susceptance_pu(&self, base_mva: f64) -> f64 {
        if base_mva == 0.0 {
            return 0.0;
        }
        self.q_output_mvar / base_mva
    }
}
/// Result of a Volt-VAR optimisation run.
#[derive(Debug, Clone)]
pub struct VvoResult {
    /// Optimal voltage setpoints for each regulator ``pu``.
    pub optimal_v_setpoints: Vec<f64>,
    /// Optimal number of energised steps for each capacitor bank.
    pub optimal_cap_steps: Vec<usize>,
    /// Estimated feeder loss reduction relative to no reactive support (%).
    pub estimated_loss_reduction_pct: f64,
    /// Mean voltage improvement across all buses ``pu``.
    pub voltage_improvement_pu: f64,
}
/// Computes and analyses voltage profiles along a distribution feeder.
#[derive(Debug, Clone)]
pub struct VoltageProfileAnalyzer {
    /// Distance of each bus from the substation ``km``.
    pub bus_distances_km: Vec<f64>,
    /// Nominal feeder voltage ``kV``.
    pub nominal_kv: f64,
}
impl VoltageProfileAnalyzer {
    /// Creates an analyser for the given nominal voltage ``kV``.
    pub fn new(nominal_kv: f64) -> Self {
        Self {
            bus_distances_km: Vec::new(),
            nominal_kv,
        }
    }
    /// Computes the full voltage profile for the supplied bus voltages ``pu``.
    pub fn compute_profile(&self, bus_voltages_pu: &[f64]) -> FeederVoltageProfile {
        let n = self.bus_distances_km.len().min(bus_voltages_pu.len());
        let distances: Vec<f64> = self.bus_distances_km[..n].to_vec();
        let voltages_pu: Vec<f64> = bus_voltages_pu[..n].to_vec();
        let voltages_kv: Vec<f64> = voltages_pu.iter().map(|&v| v * self.nominal_kv).collect();
        let min_v = voltages_pu.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_v = voltages_pu
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        let mean_v = if n == 0 {
            0.0
        } else {
            voltages_pu.iter().sum::<f64>() / n as f64
        };
        FeederVoltageProfile {
            distances,
            voltages_pu,
            voltages_kv,
            min_voltage_pu: if min_v.is_infinite() { 0.0 } else { min_v },
            max_voltage_pu: if max_v.is_infinite() { 0.0 } else { max_v },
            mean_voltage_pu: mean_v,
        }
    }
    /// Returns the index and voltage ``pu`` of the bus with the lowest voltage.
    pub fn worst_case_bus(&self, bus_voltages_pu: &[f64]) -> (usize, f64) {
        bus_voltages_pu
            .iter()
            .enumerate()
            .min_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(core::cmp::Ordering::Equal))
            .map(|(i, &v)| (i, v))
            .unwrap_or((0, 1.0))
    }
    /// Voltage drop along the feeder as a percentage: `(1 − V_min) × 100`.
    pub fn voltage_drop_pct(&self, bus_voltages_pu: &[f64]) -> f64 {
        let v_min = bus_voltages_pu.iter().cloned().fold(1.0_f64, f64::min);
        (1.0 - v_min) * 100.0
    }
}
/// Type of voltage regulation device.
#[derive(Debug, Clone, PartialEq)]
pub enum RegulatorType {
    /// On-load tap changer transformer.
    Oltc,
    /// Step voltage regulator (line regulator).
    StepVoltageRegulator,
    /// Shunt capacitor bank.
    ShuntCapacitorBank,
    /// Static var compensator.
    StaticVarCompensator,
    /// STATCOM (static synchronous compensator).
    StatCom,
    /// Distributed generator providing reactive support.
    DistributedGenerator,
    /// Battery energy storage system.
    BatteryEss,
}
/// Line-drop compensator for remote voltage regulation via OLTC.
///
/// Computes the apparent voltage at a remote regulation point by subtracting
/// the resistive and reactive voltage drops along the feeder.
pub struct LineDropCompensator;
impl LineDropCompensator {
    /// Compute the LDC-compensated voltage at the remote regulation point.
    ///
    /// Formula: `V_ldc = V_meas − I × (R × cos φ + X × sin φ)`
    ///
    /// # Arguments
    /// * `v_measured`   — measured voltage at the transformer secondary (pu)
    /// * `i_current`    — line current magnitude (pu)
    /// * `r_comp`       — compensation resistance (pu)
    /// * `x_comp`       — compensation reactance (pu)
    /// * `power_factor` — load power factor clamped to `[0, 1]`
    pub fn compute_ldc_voltage(
        v_measured: f64,
        i_current: f64,
        r_comp: f64,
        x_comp: f64,
        power_factor: f64,
    ) -> f64 {
        let cos_phi = power_factor.clamp(0.0, 1.0);
        let sin_phi = (1.0 - cos_phi * cos_phi).max(0.0).sqrt();
        v_measured - i_current * (r_comp * cos_phi + x_comp * sin_phi)
    }
}
/// Switchable shunt capacitor bank with multiple discrete steps.
///
/// Reactive power is specified in kVAR per step.
#[derive(Debug, Clone)]
pub struct CapacitorBank {
    /// Unique device identifier.
    pub id: usize,
    /// Bus ID where this bank is connected.
    pub bus_id: usize,
    /// Total number of switchable steps.
    pub n_steps: usize,
    /// Reactive power per step in kVAR.
    pub kvar_per_step: f64,
    /// Number of currently energized steps.
    pub active_steps: usize,
    /// Current status of the bank.
    pub status: CapacitorStatus,
    /// Control mode.
    pub control_mode: TapControlMode,
    /// Target voltage setpoint in per-unit.
    pub target_voltage_pu: f64,
    /// Deadband in per-unit.
    pub deadband_pu: f64,
    /// Switch in (energize one step) when voltage falls below this level.
    pub on_voltage_pu: f64,
    /// Switch out (de-energize one step) when voltage rises above this level.
    pub off_voltage_pu: f64,
    /// Maximum switching operations per day.
    pub max_switching_per_day: u32,
    /// Number of switching operations performed today.
    pub daily_switching: u32,
}
impl CapacitorBank {
    /// Create a new capacitor bank with default control settings.
    pub fn new(id: usize, bus_id: usize, n_steps: usize, kvar_per_step: f64) -> Self {
        Self {
            id,
            bus_id,
            n_steps,
            kvar_per_step,
            active_steps: 0,
            status: CapacitorStatus::Open,
            control_mode: TapControlMode::Automatic,
            target_voltage_pu: 1.0,
            deadband_pu: 0.01,
            on_voltage_pu: 0.97,
            off_voltage_pu: 1.03,
            max_switching_per_day: 10,
            daily_switching: 0,
        }
    }
    /// Returns total reactive output: `active_steps × kvar_per_step` `[kVAR]`.
    pub fn total_kvar(&self) -> f64 {
        self.active_steps as f64 * self.kvar_per_step
    }
    /// Compute switching action based on the measured bus voltage.
    ///
    /// Returns `+1` to energize one step, `-1` to de-energize one step,
    /// or `0` for no action.
    pub fn compute_switching_action(&self, measured_voltage_pu: f64) -> i32 {
        if self.status == CapacitorStatus::Fault {
            return 0;
        }
        if measured_voltage_pu < self.on_voltage_pu && self.active_steps < self.n_steps {
            1
        } else if measured_voltage_pu > self.off_voltage_pu && self.active_steps > 0 {
            -1
        } else {
            0
        }
    }
    /// Apply a switching delta, clamped to `[0, n_steps]`, enforcing daily limits.
    ///
    /// Returns the new total kVAR on success, or [`OxiGridError::InvalidParameter`]
    /// when the device is faulted or the daily limit is exhausted.
    pub fn apply_switching(&mut self, delta_steps: i32) -> Result<f64, OxiGridError> {
        if self.status == CapacitorStatus::Fault {
            return Err(OxiGridError::InvalidParameter(format!(
                "CapacitorBank {}: device is in fault state",
                self.id
            )));
        }
        if self.daily_switching >= self.max_switching_per_day {
            return Err(OxiGridError::InvalidParameter(format!(
                "CapacitorBank {}: daily switching limit ({}) reached",
                self.id, self.max_switching_per_day
            )));
        }
        let new_steps =
            (self.active_steps as i32 + delta_steps).clamp(0, self.n_steps as i32) as usize;
        self.active_steps = new_steps;
        self.status = if new_steps == 0 {
            CapacitorStatus::Open
        } else {
            CapacitorStatus::Closed
        };
        self.daily_switching += 1;
        Ok(self.total_kvar())
    }
}
/// Step voltage regulator (line regulator) for distribution feeders.
///
/// Implements a bidirectional autotransformer with a continuous boost range
/// and discrete step control.
#[derive(Debug, Clone)]
pub struct VoltageRegulatorUnit {
    /// Unique device identifier.
    pub id: usize,
    /// From-bus (source side).
    pub from_bus: usize,
    /// To-bus (regulated/load side).
    pub to_bus: usize,
    /// Minimum boost in per-unit (e.g. -0.1).
    pub min_boost_pu: f64,
    /// Maximum boost in per-unit (e.g. +0.1).
    pub max_boost_pu: f64,
    /// Current boost applied in per-unit.
    pub current_boost_pu: f64,
    /// Number of discrete boost steps.
    pub n_steps: u32,
    /// Target voltage setpoint in per-unit.
    pub target_voltage_pu: f64,
    /// Deadband in per-unit.
    pub deadband_pu: f64,
    /// Regulation band in per-unit.
    pub bandwidth_pu: f64,
    /// Operating delay in seconds.
    pub time_delay_s: f64,
}
impl VoltageRegulatorUnit {
    /// Create a new step voltage regulator with default ±10 % range, 32 steps.
    pub fn new(id: usize, from_bus: usize, to_bus: usize) -> Self {
        Self {
            id,
            from_bus,
            to_bus,
            min_boost_pu: -0.1,
            max_boost_pu: 0.1,
            current_boost_pu: 0.0,
            n_steps: 32,
            target_voltage_pu: 1.0,
            deadband_pu: 0.01,
            bandwidth_pu: 0.02,
            time_delay_s: 30.0,
        }
    }
    /// Compute the boost step size in per-unit.
    pub fn step_size_pu(&self) -> f64 {
        if self.n_steps == 0 {
            return 0.0;
        }
        (self.max_boost_pu - self.min_boost_pu) / self.n_steps as f64
    }
    /// Apply a boost delta, clamped to `[min_boost_pu, max_boost_pu]`.
    ///
    /// Returns the new boost value.
    pub fn apply_boost(&mut self, delta_pu: f64) -> f64 {
        self.current_boost_pu =
            (self.current_boost_pu + delta_pu).clamp(self.min_boost_pu, self.max_boost_pu);
        self.current_boost_pu
    }
}
/// Direction of a voltage limit violation.
#[derive(Debug, Clone, PartialEq)]
pub enum ViolationType {
    /// Voltage below the minimum limit.
    UnderVoltage,
    /// Voltage above the maximum limit.
    OverVoltage,
}
/// Control mode for tap changers and capacitor banks.
#[derive(Debug, Clone, PartialEq)]
pub enum TapControlMode {
    /// Operator sets tap position manually.
    Manual,
    /// Automatic voltage regulation based on local measurement.
    Automatic,
    /// Remote setpoint control.
    Remote,
    /// Line-drop compensation for remote bus voltage regulation.
    LineDropCompensation,
    /// Reactive power control mode.
    ReactivePowerControl,
    /// Direct voltage control mode.
    VoltageControl,
}
/// Voltage violation on a single bus.
#[derive(Debug, Clone)]
pub struct VoltageViolation {
    /// Bus index.
    pub bus: usize,
    /// Measured voltage ``pu``.
    pub voltage_pu: f64,
    /// Whether the violation is above or below the limits.
    pub violation_type: ViolationType,
}
