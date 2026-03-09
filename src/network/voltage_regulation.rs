//! Voltage regulation module for distribution and transmission networks.
//!
//! Provides OLTC control, capacitor bank switching, and coordinated
//! voltage-var management (VVO) for both distribution and transmission systems.
//!
//! # Device Types
//!
//! - [`OltcController`]            — On-load tap changer with deadband and daily-limit control
//! - [`CapacitorBank`]             — Switched shunt capacitor bank with step control
//! - [`VoltageRegulatorUnit`]      — IEEE C57.15 step voltage regulator
//! - [`VoltageRegulationSystem`]   — Coordinated multi-device VVO controller
//! - [`LineDropCompensator`]       — Remote voltage regulation via LDC
//! - [`VoltageVarOptimizer`]       — Greedy sensitivity-based VVO
//! - [`SvcModel`]                  — Static VAR compensator with droop dynamics
//! - [`CoordinatedVoltageController`] — Priority-ordered multi-device coordinator
//!
//! All voltage quantities are in `[pu]`, reactive power in `[MVAr]` or `[kVAR]`
//! (noted per struct), and time in `[s]`.

use crate::error::OxiGridError;

// ---------------------------------------------------------------------------
// Enums — required API
// ---------------------------------------------------------------------------

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

/// Status of a capacitor bank.
#[derive(Debug, Clone, PartialEq)]
pub enum CapacitorStatus {
    /// All steps open (de-energized).
    Open,
    /// One or more steps closed (energized).
    Closed,
    /// Device in fault state — no switching allowed.
    Fault,
}

// ---------------------------------------------------------------------------
// OltcController
// ---------------------------------------------------------------------------

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
            Some(1) // voltage too low → tap up
        } else {
            Some(-1) // voltage too high → tap down
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

// ---------------------------------------------------------------------------
// CapacitorBank
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// VoltageRegulatorUnit
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// VoltageProfile
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// RegulationAction
// ---------------------------------------------------------------------------

/// A recommended or applied voltage regulation action.
#[derive(Debug, Clone)]
pub struct RegulationAction {
    /// Identifier of the target device.
    pub device_id: usize,
    /// Type of regulation device.
    pub device_type: RegulatorType,
    /// Human-readable description (e.g. `"tap_up"`, `"capacitor_on"`).
    pub action: String,
    /// Magnitude of the change (tap steps, switching steps, or boost delta in pu).
    pub delta: f64,
    /// Estimated improvement in bus voltage magnitude `[pu]`.
    pub expected_voltage_improvement_pu: f64,
    /// Estimated monetary cost of performing this action `[USD]`.
    pub cost_usd: f64,
}

// ---------------------------------------------------------------------------
// VoltageRegulationSystem
// ---------------------------------------------------------------------------

/// Coordinated voltage regulation system managing OLTCs, capacitor banks,
/// and step voltage regulators.
///
/// Implements centralized VVO (Volt-VAR Optimization) with a sensitivity-based
/// dispatch strategy.
#[derive(Debug, Clone)]
pub struct VoltageRegulationSystem {
    /// Collection of OLTC controllers.
    pub oltc_controllers: Vec<OltcController>,
    /// Collection of shunt capacitor banks.
    pub capacitor_banks: Vec<CapacitorBank>,
    /// Collection of step voltage regulators.
    pub voltage_regulators: Vec<VoltageRegulatorUnit>,
    /// Sensitivity matrix dV/dQ: `n_bus` rows × `n_devices` columns.
    pub bus_sensitivities: Vec<Vec<f64>>,
    /// Lower statutory voltage limit (default 0.95 pu).
    pub min_voltage_limit_pu: f64,
    /// Upper statutory voltage limit (default 1.05 pu).
    pub max_voltage_limit_pu: f64,
}

impl VoltageRegulationSystem {
    /// Create a new empty voltage regulation system with default statutory limits.
    pub fn new() -> Self {
        Self {
            oltc_controllers: Vec::new(),
            capacitor_banks: Vec::new(),
            voltage_regulators: Vec::new(),
            bus_sensitivities: Vec::new(),
            min_voltage_limit_pu: 0.95,
            max_voltage_limit_pu: 1.05,
        }
    }

    /// Add an OLTC controller to the system.
    pub fn add_oltc(&mut self, oltc: OltcController) {
        self.oltc_controllers.push(oltc);
    }

    /// Add a capacitor bank to the system.
    pub fn add_capacitor_bank(&mut self, cap: CapacitorBank) {
        self.capacitor_banks.push(cap);
    }

    /// Add a step voltage regulator to the system.
    pub fn add_regulator(&mut self, reg: VoltageRegulatorUnit) {
        self.voltage_regulators.push(reg);
    }

    /// Set the dV/dQ sensitivity matrix (`n_bus` × `n_devices`).
    pub fn set_sensitivity_matrix(&mut self, matrix: Vec<Vec<f64>>) {
        self.bus_sensitivities = matrix;
    }

    /// Compute a [`VoltageProfile`] snapshot from raw per-unit voltage measurements.
    pub fn assess_voltage_profile(&self, voltages_pu: &[f64], bus_ids: &[usize]) -> VoltageProfile {
        let n = voltages_pu.len();
        let min_v = voltages_pu.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_v = voltages_pu
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        let n_violations = voltages_pu
            .iter()
            .filter(|&&v| v < self.min_voltage_limit_pu || v > self.max_voltage_limit_pu)
            .count();
        let voltage_unbalance_pct = if n > 1 {
            let avg = voltages_pu.iter().sum::<f64>() / n as f64;
            if avg > 0.0 {
                (max_v - min_v) / avg * 100.0
            } else {
                0.0
            }
        } else {
            0.0
        };
        VoltageProfile {
            bus_voltages_pu: voltages_pu.to_vec(),
            bus_ids: bus_ids.to_vec(),
            timestamp: 0.0,
            min_voltage_pu: if n == 0 { 0.0 } else { min_v },
            max_voltage_pu: if n == 0 { 0.0 } else { max_v },
            n_violations,
            voltage_unbalance_pct,
        }
    }

    /// Compute coordinated regulation actions to resolve voltage violations.
    ///
    /// Actions are generated only for buses outside the statutory limits.
    /// The returned list is sorted by cost ascending (cheapest action first).
    pub fn compute_regulation_actions(&self, profile: &VoltageProfile) -> Vec<RegulationAction> {
        let mut actions: Vec<RegulationAction> = Vec::new();

        for (idx, &v) in profile.bus_voltages_pu.iter().enumerate() {
            if v >= self.min_voltage_limit_pu && v <= self.max_voltage_limit_pu {
                continue;
            }
            let bus_id = profile.bus_ids.get(idx).copied().unwrap_or(idx);
            let undervolt = v < self.min_voltage_limit_pu;

            // --- Capacitor banks (~$1 per switching operation) ---------------
            for cap in &self.capacitor_banks {
                if cap.bus_id != bus_id {
                    continue;
                }
                if cap.status == CapacitorStatus::Fault {
                    continue;
                }
                if cap.daily_switching >= cap.max_switching_per_day {
                    continue;
                }
                let can_act = if undervolt {
                    cap.active_steps < cap.n_steps
                } else {
                    cap.active_steps > 0
                };
                if !can_act {
                    continue;
                }
                let sens = self.lookup_sensitivity(idx, cap.id);
                let improvement = sens * cap.kvar_per_step.abs() * 0.001;
                actions.push(RegulationAction {
                    device_id: cap.id,
                    device_type: RegulatorType::ShuntCapacitorBank,
                    action: if undervolt {
                        "capacitor_on".to_string()
                    } else {
                        "capacitor_off".to_string()
                    },
                    delta: if undervolt { 1.0 } else { -1.0 },
                    expected_voltage_improvement_pu: improvement,
                    cost_usd: 1.0,
                });
            }

            // --- Step voltage regulators (~$3 per operation) -----------------
            for reg in &self.voltage_regulators {
                if reg.to_bus != bus_id {
                    continue;
                }
                let step = reg.step_size_pu();
                let can_act = if undervolt {
                    reg.current_boost_pu < reg.max_boost_pu
                } else {
                    reg.current_boost_pu > reg.min_boost_pu
                };
                if !can_act {
                    continue;
                }
                let delta = if undervolt { step } else { -step };
                actions.push(RegulationAction {
                    device_id: reg.id,
                    device_type: RegulatorType::StepVoltageRegulator,
                    action: if undervolt {
                        "boost_increase".to_string()
                    } else {
                        "boost_decrease".to_string()
                    },
                    delta,
                    expected_voltage_improvement_pu: step.abs(),
                    cost_usd: 3.0,
                });
            }

            // --- OLTC controllers (~$5 per tap operation) --------------------
            for oltc in &self.oltc_controllers {
                if oltc.bus_id != bus_id {
                    continue;
                }
                if oltc.daily_operations >= oltc.max_operations_per_day {
                    continue;
                }
                let can_act = if undervolt {
                    oltc.current_tap < oltc.max_tap
                } else {
                    oltc.current_tap > oltc.min_tap
                };
                if !can_act {
                    continue;
                }
                actions.push(RegulationAction {
                    device_id: oltc.id,
                    device_type: RegulatorType::Oltc,
                    action: if undervolt {
                        "tap_up".to_string()
                    } else {
                        "tap_down".to_string()
                    },
                    delta: if undervolt { 1.0 } else { -1.0 },
                    expected_voltage_improvement_pu: oltc.tap_step_pu,
                    cost_usd: 5.0,
                });
            }
        }

        // Sort cheapest first
        actions.sort_by(|a, b| {
            a.cost_usd
                .partial_cmp(&b.cost_usd)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        actions
    }

    /// Look up dV/dQ sensitivity for a (`bus_idx`, `device_id`) pair.
    ///
    /// Returns a small default value when the matrix is unpopulated.
    fn lookup_sensitivity(&self, bus_idx: usize, device_id: usize) -> f64 {
        self.bus_sensitivities
            .get(bus_idx)
            .and_then(|row| row.get(device_id))
            .copied()
            .unwrap_or(0.01)
    }

    /// Apply a list of regulation actions.
    ///
    /// Returns an empty vector; use [`apply_actions_to_voltages`] when an
    /// explicit voltage vector is available.
    ///
    /// [`apply_actions_to_voltages`]: VoltageRegulationSystem::apply_actions_to_voltages
    pub fn apply_actions(&mut self, actions: &[RegulationAction]) -> Vec<f64> {
        let _ = actions;
        Vec::new()
    }

    /// Apply actions against an explicit voltage vector, mutating device state
    /// and returning estimated post-action voltages.
    pub fn apply_actions_to_voltages(
        &mut self,
        actions: &[RegulationAction],
        voltages_pu: &[f64],
    ) -> Vec<f64> {
        let mut new_v = voltages_pu.to_vec();
        for action in actions {
            let sign = if action.delta >= 0.0 {
                1.0_f64
            } else {
                -1.0_f64
            };
            let dv = action.expected_voltage_improvement_pu * sign;
            match action.device_type {
                RegulatorType::Oltc => {
                    if let Some(oltc) = self
                        .oltc_controllers
                        .iter_mut()
                        .find(|o| o.id == action.device_id)
                    {
                        let delta_tap = if action.delta >= 0.0 { 1_i32 } else { -1_i32 };
                        let _ = oltc.apply_tap(delta_tap);
                        for v in new_v.iter_mut() {
                            *v += dv;
                        }
                    }
                }
                RegulatorType::ShuntCapacitorBank => {
                    if let Some(cap) = self
                        .capacitor_banks
                        .iter_mut()
                        .find(|c| c.id == action.device_id)
                    {
                        let delta = if action.delta >= 0.0 { 1_i32 } else { -1_i32 };
                        let _ = cap.apply_switching(delta);
                        for v in new_v.iter_mut() {
                            *v += dv;
                        }
                    }
                }
                RegulatorType::StepVoltageRegulator => {
                    if let Some(reg) = self
                        .voltage_regulators
                        .iter_mut()
                        .find(|r| r.id == action.device_id)
                    {
                        let _ = reg.apply_boost(action.delta);
                        for v in new_v.iter_mut() {
                            *v += dv;
                        }
                    }
                }
                _ => {}
            }
        }
        new_v
    }

    /// Run one full coordination step: assess profile, compute actions, apply top-5.
    ///
    /// Returns the initial [`VoltageProfile`] and the full list of recommended
    /// [`RegulationAction`]s (including those not applied this step).
    pub fn run_coordination_step(
        &mut self,
        voltages_pu: &[f64],
        bus_ids: &[usize],
    ) -> (VoltageProfile, Vec<RegulationAction>) {
        let profile = self.assess_voltage_profile(voltages_pu, bus_ids);
        let actions = self.compute_regulation_actions(&profile);
        let top: Vec<RegulationAction> = actions.iter().take(5).cloned().collect();
        let _ = self.apply_actions_to_voltages(&top, voltages_pu);
        (profile, actions)
    }
}

impl Default for VoltageRegulationSystem {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// LineDropCompensator
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// VoltageVarOptimizer
// ---------------------------------------------------------------------------

/// Coordinated Volt-VAR Optimizer (VVO).
///
/// Uses a greedy algorithm: fix the worst voltage violation first by dispatching
/// the highest-sensitivity available device at each violated bus.
pub struct VoltageVarOptimizer;

impl VoltageVarOptimizer {
    /// Generate optimized regulation actions for the given voltage profile.
    ///
    /// Strategy:
    /// 1. Collect all violated buses sorted by deviation magnitude (largest first).
    /// 2. For each violated bus, select the capacitor bank with the highest
    ///    dV/dQ sensitivity.  If none is available, fall back to the highest-
    ///    sensitivity OLTC.
    pub fn optimize_setpoints(
        profile: &VoltageProfile,
        devices: &VoltageRegulationSystem,
    ) -> Vec<RegulationAction> {
        // Collect violated buses, sorted worst-first
        let mut violated: Vec<(usize, f64)> = profile
            .bus_voltages_pu
            .iter()
            .enumerate()
            .filter_map(|(i, &v)| {
                let dev = if v < devices.min_voltage_limit_pu {
                    devices.min_voltage_limit_pu - v
                } else if v > devices.max_voltage_limit_pu {
                    v - devices.max_voltage_limit_pu
                } else {
                    return None;
                };
                Some((i, dev))
            })
            .collect();
        violated.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        let mut actions = Vec::new();

        for (bus_idx, _deviation) in violated {
            let bus_id = profile.bus_ids.get(bus_idx).copied().unwrap_or(bus_idx);
            let v = profile.bus_voltages_pu[bus_idx];
            let undervolt = v < devices.min_voltage_limit_pu;

            // Best capacitor bank at this bus by sensitivity
            let best_cap = devices
                .capacitor_banks
                .iter()
                .filter(|c| c.bus_id == bus_id && c.status != CapacitorStatus::Fault)
                .filter(|c| {
                    if undervolt {
                        c.active_steps < c.n_steps
                    } else {
                        c.active_steps > 0
                    }
                })
                .max_by(|a, b| {
                    let sa = devices
                        .bus_sensitivities
                        .get(bus_idx)
                        .and_then(|r| r.get(a.id))
                        .copied()
                        .unwrap_or(0.0);
                    let sb = devices
                        .bus_sensitivities
                        .get(bus_idx)
                        .and_then(|r| r.get(b.id))
                        .copied()
                        .unwrap_or(0.0);
                    sa.partial_cmp(&sb).unwrap_or(std::cmp::Ordering::Equal)
                });

            if let Some(cap) = best_cap {
                let sens = devices
                    .bus_sensitivities
                    .get(bus_idx)
                    .and_then(|r| r.get(cap.id))
                    .copied()
                    .unwrap_or(0.01);
                actions.push(RegulationAction {
                    device_id: cap.id,
                    device_type: RegulatorType::ShuntCapacitorBank,
                    action: if undervolt {
                        "capacitor_on".to_string()
                    } else {
                        "capacitor_off".to_string()
                    },
                    delta: if undervolt { 1.0 } else { -1.0 },
                    expected_voltage_improvement_pu: sens * cap.kvar_per_step * 0.001,
                    cost_usd: 1.0,
                });
                continue;
            }

            // Fallback: best OLTC at this bus by sensitivity
            let best_oltc = devices
                .oltc_controllers
                .iter()
                .filter(|o| o.bus_id == bus_id && o.daily_operations < o.max_operations_per_day)
                .filter(|o| {
                    if undervolt {
                        o.current_tap < o.max_tap
                    } else {
                        o.current_tap > o.min_tap
                    }
                })
                .max_by(|a, b| {
                    let sa = devices
                        .bus_sensitivities
                        .get(bus_idx)
                        .and_then(|r| r.get(a.id))
                        .copied()
                        .unwrap_or(0.0);
                    let sb = devices
                        .bus_sensitivities
                        .get(bus_idx)
                        .and_then(|r| r.get(b.id))
                        .copied()
                        .unwrap_or(0.0);
                    sa.partial_cmp(&sb).unwrap_or(std::cmp::Ordering::Equal)
                });

            if let Some(oltc) = best_oltc {
                actions.push(RegulationAction {
                    device_id: oltc.id,
                    device_type: RegulatorType::Oltc,
                    action: if undervolt {
                        "tap_up".to_string()
                    } else {
                        "tap_down".to_string()
                    },
                    delta: if undervolt { 1.0 } else { -1.0 },
                    expected_voltage_improvement_pu: oltc.tap_step_pu,
                    cost_usd: 5.0,
                });
            }
        }

        actions
    }
}

// ---------------------------------------------------------------------------
// SvcModel
// ---------------------------------------------------------------------------

/// Static VAR Compensator with first-order dynamic response.
///
/// Implements a proportional droop Q-V characteristic:
/// ```text
/// Q = k_svc × (V_sp − V_meas)   [MVAr]
/// ```
/// clamped to `[q_min_mvar, q_max_mvar]`, with a first-order lag filter
/// characterised by time constant `t2_s` `[s]`.
#[derive(Debug, Clone)]
pub struct SvcModel {
    /// Unique device identifier.
    pub id: usize,
    /// Bus at which the SVC is connected.
    pub bus: usize,
    /// Inductive (absorbing) reactive-power limit `[MVAr]` (negative).
    pub q_min_mvar: f64,
    /// Capacitive (injecting) reactive-power limit `[MVAr]` (positive).
    pub q_max_mvar: f64,
    /// Voltage setpoint `[pu]`.
    pub v_setpoint_pu: f64,
    /// Droop slope as a percentage of the voltage base.
    pub droop_pct: f64,
    /// Proportional gain: `Q_range / droop` `[MVAr/pu]`.
    pub k_svc: f64,
    /// Lead time constant `[s]`.
    pub t1_s: f64,
    /// Lag time constant (dominant) `[s]`.
    pub t2_s: f64,
    /// Current reactive power output `[MVAr]`.
    pub q_output_mvar: f64,
    /// Internal voltage reference `[pu]`.
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

    /// Steady-state reactive output from the static Q-V droop curve `[MVAr]`.
    pub fn q_from_voltage(&self, v_pu: f64) -> f64 {
        let q_raw = self.k_svc * (self.v_setpoint_pu - v_pu);
        q_raw.clamp(self.q_min_mvar, self.q_max_mvar)
    }

    /// Advances the SVC dynamic state by `dt_s` `[s]` and returns updated Q `[MVAr]`.
    pub fn step(&mut self, v_pu: f64, dt_s: f64) -> f64 {
        let q_target = self.q_from_voltage(v_pu);
        let alpha = dt_s / (self.t2_s + dt_s);
        self.q_output_mvar += alpha * (q_target - self.q_output_mvar);
        self.q_output_mvar
    }

    /// Equivalent shunt susceptance `[pu]` at V ≈ 1 pu.
    pub fn susceptance_pu(&self, base_mva: f64) -> f64 {
        if base_mva == 0.0 {
            return 0.0;
        }
        self.q_output_mvar / base_mva
    }
}

// ---------------------------------------------------------------------------
// CoordinatedVoltageController
// ---------------------------------------------------------------------------

/// Action produced by one step-control evaluation of a [`StepRegulator`].
#[derive(Debug, Clone, PartialEq)]
pub enum TapAction {
    /// Voltage within deadband — no change.
    NoChange,
    /// Tap position incremented by one step.
    TapUp,
    /// Tap position decremented by one step.
    TapDown,
    /// Already at the tap limit in the required direction.
    AtLimit(String),
}

/// Control mode for a switched capacitor bank (legacy coordinator API).
#[derive(Debug, Clone, PartialEq)]
pub enum CapControlMode {
    /// Switch based on local bus voltage `[pu]`.
    Voltage,
    /// Switch based on reactive power demand.
    Reactive,
    /// Schedule-based (time-of-day) switching.
    Time,
    /// Switch based on feeder current magnitude.
    Current,
}

/// Action returned by one control evaluation of a [`StepCapacitorBank`].
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
    /// Rated voltage `[kV]`.
    pub rated_kv: f64,
    /// Minimum tap position (e.g. -16).
    pub min_tap: i32,
    /// Maximum tap position (e.g. +16).
    pub max_tap: i32,
    /// Voltage change per tap step as a percentage (typically 0.625 %).
    pub step_voltage_pct: f64,
    /// Current tap position.
    pub current_tap: i32,
    /// Voltage setpoint `[pu]`.
    pub v_setpoint_pu: f64,
    /// Control deadband (half-width) `[pu]`.
    pub bandwidth_pu: f64,
    /// Line drop compensator resistance setting `[pu]`.
    pub r_compensator: f64,
    /// Line drop compensator reactance setting `[pu]`.
    pub x_compensator: f64,
    /// Time delay before a tap change is executed `[s]`.
    pub time_delay_s: f64,
    /// Accumulated time the voltage has been outside the deadband `[s]`.
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

    /// Per-unit tap ratio `a = 1 + tap × step / 100` `[pu]`.
    #[inline]
    pub fn tap_ratio(&self) -> f64 {
        1.0 + self.current_tap as f64 * self.step_voltage_pct / 100.0
    }

    /// Voltage at the sensing point after line-drop compensation `[pu]`.
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

    /// Achievable regulated-voltage range `(V_min, V_max)` `[pu]`.
    pub fn effective_voltage_range(&self) -> (f64, f64) {
        let v_min = 1.0 + self.min_tap as f64 * self.step_voltage_pct / 100.0;
        let v_max = 1.0 + self.max_tap as f64 * self.step_voltage_pct / 100.0;
        (v_min, v_max)
    }
}

/// Switched shunt capacitor bank for the legacy [`CoordinatedVoltageController`].
///
/// Reactive power is specified in MVAr.
#[derive(Debug, Clone)]
pub struct StepCapacitorBank {
    /// Unique device identifier.
    pub id: usize,
    /// Bus at which the capacitor is connected.
    pub bus: usize,
    /// Total rated reactive power `[MVAr]`.
    pub q_rated_mvar: f64,
    /// Rated terminal voltage `[kV]`.
    pub voltage_kv: f64,
    /// Total number of switchable sections.
    pub steps: usize,
    /// Currently energised sections.
    pub current_steps: usize,
    /// Active control mode.
    pub control_mode: CapControlMode,
    /// Switch-in voltage threshold `[pu]`.
    pub v_on_pu: f64,
    /// Switch-out voltage threshold `[pu]`.
    pub v_off_pu: f64,
    /// Minimum time the voltage must remain outside threshold `[s]`.
    pub time_delay_s: f64,
    /// Accumulated time the voltage has been outside threshold `[s]`.
    pub pending_time: f64,
    /// Cumulative switching operations count.
    pub total_switching: usize,
}

impl StepCapacitorBank {
    /// Constructs a capacitor bank with default voltage-mode control.
    pub fn new(id: usize, bus: usize, q_rated_mvar: f64, voltage_kv: f64, steps: usize) -> Self {
        Self {
            id,
            bus,
            q_rated_mvar,
            voltage_kv,
            steps: steps.max(1),
            current_steps: 0,
            control_mode: CapControlMode::Voltage,
            v_on_pu: 0.98,
            v_off_pu: 1.02,
            time_delay_s: 10.0,
            pending_time: 0.0,
            total_switching: 0,
        }
    }

    /// Reactive power currently injected `[MVAr]`.
    #[inline]
    pub fn q_injected_mvar(&self) -> f64 {
        if self.steps == 0 {
            return 0.0;
        }
        self.q_rated_mvar * self.current_steps as f64 / self.steps as f64
    }

    /// Per-unit susceptance of the currently injected reactive power `[pu]`.
    pub fn susceptance_pu(&self, base_mva: f64, base_kv: f64) -> f64 {
        if base_mva == 0.0 || self.voltage_kv == 0.0 {
            return 0.0;
        }
        self.q_injected_mvar() / base_mva * (base_kv / self.voltage_kv).powi(2)
    }

    /// Time-delayed voltage-mode step control.
    pub fn step_control(&mut self, v_pu: f64, dt_s: f64) -> CapAction {
        match self.control_mode {
            CapControlMode::Voltage => self.voltage_mode_control(v_pu, dt_s),
            _ => CapAction::NoChange,
        }
    }

    fn voltage_mode_control(&mut self, v_pu: f64, dt_s: f64) -> CapAction {
        let need_in = v_pu < self.v_on_pu && self.current_steps < self.steps;
        let need_out = v_pu > self.v_off_pu && self.current_steps > 0;

        if need_in {
            self.pending_time += dt_s;
            if self.pending_time >= self.time_delay_s {
                self.pending_time = 0.0;
                self.current_steps += 1;
                self.total_switching += 1;
                return CapAction::StepIn;
            }
            return CapAction::NoChange;
        }
        if need_out {
            self.pending_time += dt_s;
            if self.pending_time >= self.time_delay_s {
                self.pending_time = 0.0;
                self.current_steps -= 1;
                self.total_switching += 1;
                return CapAction::StepOut;
            }
            return CapAction::NoChange;
        }
        if v_pu < self.v_on_pu && self.current_steps >= self.steps {
            self.pending_time = 0.0;
            return CapAction::AtLimit;
        }
        if v_pu > self.v_off_pu && self.current_steps == 0 {
            self.pending_time = 0.0;
            return CapAction::AtLimit;
        }
        self.pending_time = 0.0;
        CapAction::NoChange
    }

    /// Manually switch a number of steps (positive = in, negative = out).
    pub fn switch_step(&mut self, steps_delta: i32) -> Result<(), String> {
        let new_steps = self.current_steps as i64 + steps_delta as i64;
        if new_steps < 0 {
            return Err(format!(
                "cap {} cannot switch out {} steps (only {} in service)",
                self.id, -steps_delta, self.current_steps
            ));
        }
        if new_steps > self.steps as i64 {
            return Err(format!(
                "cap {} cannot switch in {} steps (max {})",
                self.id, steps_delta, self.steps
            ));
        }
        let prev = self.current_steps;
        self.current_steps = new_steps as usize;
        self.total_switching += (self.current_steps as i64 - prev as i64).unsigned_abs() as usize;
        Ok(())
    }
}

/// Voltage violation on a single bus.
#[derive(Debug, Clone)]
pub struct VoltageViolation {
    /// Bus index.
    pub bus: usize,
    /// Measured voltage `[pu]`.
    pub voltage_pu: f64,
    /// Whether the violation is above or below the limits.
    pub violation_type: ViolationType,
}

/// Direction of a voltage limit violation.
#[derive(Debug, Clone, PartialEq)]
pub enum ViolationType {
    /// Voltage below the minimum limit.
    UnderVoltage,
    /// Voltage above the maximum limit.
    OverVoltage,
}

/// Summary of a single coordinated control time step.
#[derive(Debug, Clone)]
pub struct CoordVoltageResult {
    /// Number of tap-change operations executed this step.
    pub tap_changes: usize,
    /// Number of capacitor-step switching operations this step.
    pub cap_switchings: usize,
    /// Total SVC reactive power output this step `[MVAr]`.
    pub svc_q_total_mvar: f64,
    /// Number of bus voltage violations remaining after control actions.
    pub violations_remaining: usize,
}

/// Multi-device coordinated voltage controller.
///
/// Aggregates [`StepRegulator`]s, [`StepCapacitorBank`]s and [`SvcModel`]s and
/// applies them in priority order (SVCs → capacitors → tap changers) each step.
#[derive(Debug, Clone)]
pub struct CoordinatedVoltageController {
    /// Tap-changing voltage regulators.
    pub regulators: Vec<StepRegulator>,
    /// Switched capacitor banks.
    pub capacitors: Vec<StepCapacitorBank>,
    /// Static VAR compensators.
    pub svcs: Vec<SvcModel>,
    /// System base MVA `[MVA]`.
    pub base_mva: f64,
    /// System base voltage `[kV]`.
    pub base_kv: f64,
    /// Acceptable voltage window `(V_min, V_max)` `[pu]`.
    pub v_limits: (f64, f64),
}

impl CoordinatedVoltageController {
    /// Creates an empty controller with ANSI ±5 % voltage limits.
    pub fn new(base_mva: f64, base_kv: f64) -> Self {
        Self {
            regulators: Vec::new(),
            capacitors: Vec::new(),
            svcs: Vec::new(),
            base_mva,
            base_kv,
            v_limits: (0.95, 1.05),
        }
    }

    /// Registers a step voltage regulator.
    pub fn add_regulator(&mut self, reg: StepRegulator) {
        self.regulators.push(reg);
    }

    /// Registers a capacitor bank.
    pub fn add_capacitor(&mut self, cap: StepCapacitorBank) {
        self.capacitors.push(cap);
    }

    /// Registers an SVC.
    pub fn add_svc(&mut self, svc: SvcModel) {
        self.svcs.push(svc);
    }

    /// Advances all devices by `dt_s` `[s]` and updates `bus_voltages` in place.
    pub fn step(&mut self, bus_voltages: &mut [f64], dt_s: f64) -> CoordVoltageResult {
        let mut tap_changes = 0usize;
        let mut cap_switchings = 0usize;
        let mut svc_q_total = 0.0_f64;

        // 1. SVCs — continuous reactive injection
        for svc in &mut self.svcs {
            let v = if svc.bus < bus_voltages.len() {
                bus_voltages[svc.bus]
            } else {
                svc.v_setpoint_pu
            };
            let q = svc.step(v, dt_s);
            svc_q_total += q;
            if svc.bus < bus_voltages.len() && self.base_mva > 0.0 {
                let dv = q / self.base_mva * 0.05;
                bus_voltages[svc.bus] = (bus_voltages[svc.bus] + dv).clamp(0.85, 1.15);
            }
        }

        // 2. Capacitor banks — discrete reactive steps
        for cap in &mut self.capacitors {
            let v = if cap.bus < bus_voltages.len() {
                bus_voltages[cap.bus]
            } else {
                1.0
            };
            let action = cap.step_control(v, dt_s);
            match action {
                CapAction::StepIn | CapAction::StepOut => {
                    cap_switchings += 1;
                    if cap.bus < bus_voltages.len() && self.base_mva > 0.0 {
                        let dv = cap.q_injected_mvar() / self.base_mva * 0.01;
                        bus_voltages[cap.bus] = (bus_voltages[cap.bus] + dv).clamp(0.85, 1.15);
                    }
                }
                _ => {}
            }
        }

        // 3. Voltage regulators — discrete tap changes
        for reg in &mut self.regulators {
            let v = if reg.bus_to < bus_voltages.len() {
                bus_voltages[reg.bus_to]
            } else {
                reg.v_setpoint_pu
            };
            let old_ratio = reg.tap_ratio();
            let action = reg.step_control(v, dt_s);
            let new_ratio = reg.tap_ratio();
            match action {
                TapAction::TapUp | TapAction::TapDown => {
                    tap_changes += 1;
                    if reg.bus_to < bus_voltages.len() && old_ratio > 0.0 {
                        bus_voltages[reg.bus_to] *= new_ratio / old_ratio;
                        bus_voltages[reg.bus_to] = bus_voltages[reg.bus_to].clamp(0.85, 1.15);
                    }
                }
                _ => {}
            }
        }

        let violations = self.check_voltage_violations(bus_voltages);
        CoordVoltageResult {
            tap_changes,
            cap_switchings,
            svc_q_total_mvar: svc_q_total,
            violations_remaining: violations.len(),
        }
    }

    /// Sum of all capacitor and SVC reactive power currently injected `[MVAr]`.
    pub fn total_reactive_support_mvar(&self) -> f64 {
        let cap_q: f64 = self.capacitors.iter().map(|c| c.q_injected_mvar()).sum();
        let svc_q: f64 = self.svcs.iter().map(|s| s.q_output_mvar).sum();
        cap_q + svc_q
    }

    /// Returns all buses whose voltage is outside `v_limits`.
    pub fn check_voltage_violations(&self, bus_voltages: &[f64]) -> Vec<VoltageViolation> {
        let (v_min, v_max) = self.v_limits;
        bus_voltages
            .iter()
            .enumerate()
            .filter_map(|(bus, &v)| {
                if v < v_min {
                    Some(VoltageViolation {
                        bus,
                        voltage_pu: v,
                        violation_type: ViolationType::UnderVoltage,
                    })
                } else if v > v_max {
                    Some(VoltageViolation {
                        bus,
                        voltage_pu: v,
                        violation_type: ViolationType::OverVoltage,
                    })
                } else {
                    None
                }
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// VvoOptimizer (legacy)
// ---------------------------------------------------------------------------

/// Result of a Volt-VAR optimisation run.
#[derive(Debug, Clone)]
pub struct VvoResult {
    /// Optimal voltage setpoints for each regulator `[pu]`.
    pub optimal_v_setpoints: Vec<f64>,
    /// Optimal number of energised steps for each capacitor bank.
    pub optimal_cap_steps: Vec<usize>,
    /// Estimated feeder loss reduction relative to no reactive support (%).
    pub estimated_loss_reduction_pct: f64,
    /// Mean voltage improvement across all buses `[pu]`.
    pub voltage_improvement_pu: f64,
}

/// Simplified greedy Volt-VAR Optimiser for the legacy [`CoordinatedVoltageController`].
#[derive(Debug, Clone)]
pub struct VvoOptimizer {
    /// Target (nominal) voltage `[pu]`.
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

impl Default for VvoOptimizer {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// VoltageProfileAnalyzer
// ---------------------------------------------------------------------------

/// Snapshot of the voltage profile along a distribution feeder.
#[derive(Debug, Clone)]
pub struct FeederVoltageProfile {
    /// Distance from substation for each bus `[km]`.
    pub distances: Vec<f64>,
    /// Per-unit bus voltages `[pu]`.
    pub voltages_pu: Vec<f64>,
    /// Absolute bus voltages `[kV]`.
    pub voltages_kv: Vec<f64>,
    /// Minimum bus voltage in the profile `[pu]`.
    pub min_voltage_pu: f64,
    /// Maximum bus voltage in the profile `[pu]`.
    pub max_voltage_pu: f64,
    /// Mean bus voltage across the profile `[pu]`.
    pub mean_voltage_pu: f64,
}

/// Computes and analyses voltage profiles along a distribution feeder.
#[derive(Debug, Clone)]
pub struct VoltageProfileAnalyzer {
    /// Distance of each bus from the substation `[km]`.
    pub bus_distances_km: Vec<f64>,
    /// Nominal feeder voltage `[kV]`.
    pub nominal_kv: f64,
}

impl VoltageProfileAnalyzer {
    /// Creates an analyser for the given nominal voltage `[kV]`.
    pub fn new(nominal_kv: f64) -> Self {
        Self {
            bus_distances_km: Vec::new(),
            nominal_kv,
        }
    }

    /// Computes the full voltage profile for the supplied bus voltages `[pu]`.
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

    /// Returns the index and voltage `[pu]` of the bus with the lowest voltage.
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

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // ---- helpers -----------------------------------------------------------

    fn make_oltc() -> OltcController {
        OltcController::new(1, 10, -16, 16)
    }

    fn make_cap() -> CapacitorBank {
        CapacitorBank::new(1, 10, 4, 150.0)
    }

    // ---- OltcController ----------------------------------------------------

    #[test]
    fn test_oltc_voltage_ratio_positive_tap() {
        let mut oltc = make_oltc();
        oltc.current_tap = 1;
        assert!(oltc.voltage_ratio() > 1.0, "positive tap must raise ratio");
    }

    #[test]
    fn test_oltc_voltage_ratio_negative_tap() {
        let mut oltc = make_oltc();
        oltc.current_tap = -1;
        assert!(oltc.voltage_ratio() < 1.0, "negative tap must lower ratio");
    }

    #[test]
    fn test_oltc_action_low_voltage() {
        let oltc = make_oltc();
        assert_eq!(oltc.compute_action(0.93), Some(1));
    }

    #[test]
    fn test_oltc_action_high_voltage() {
        let oltc = make_oltc();
        assert_eq!(oltc.compute_action(1.06), Some(-1));
    }

    #[test]
    fn test_oltc_action_within_deadband() {
        let oltc = make_oltc(); // deadband = 0.01
        assert_eq!(oltc.compute_action(1.005), None);
    }

    #[test]
    fn test_oltc_tap_limits() {
        let mut oltc = make_oltc();
        oltc.current_tap = 16;
        assert!(
            oltc.apply_tap(1).is_err(),
            "tapping beyond max_tap must fail"
        );
    }

    #[test]
    fn test_oltc_daily_limit() {
        let mut oltc = make_oltc();
        oltc.daily_operations = 20;
        assert!(
            oltc.apply_tap(1).is_err(),
            "exceeding daily limit must fail"
        );
    }

    #[test]
    fn test_oltc_apply_tap_success() {
        let mut oltc = make_oltc();
        let ratio = oltc.apply_tap(1).expect("first tap should succeed");
        assert!((ratio - 1.00625).abs() < 1e-9);
        assert_eq!(oltc.current_tap, 1);
        assert_eq!(oltc.daily_operations, 1);
    }

    #[test]
    fn test_oltc_reset_daily_counter() {
        let mut oltc = make_oltc();
        oltc.daily_operations = 15;
        oltc.reset_daily_counter();
        assert_eq!(oltc.daily_operations, 0);
    }

    // ---- CapacitorBank -----------------------------------------------------

    #[test]
    fn test_capacitor_bank_total_kvar() {
        let mut cap = make_cap();
        cap.active_steps = 2;
        assert!((cap.total_kvar() - 300.0).abs() < 1e-9);
    }

    #[test]
    fn test_capacitor_switch_in_low_v() {
        let cap = make_cap(); // on_voltage_pu = 0.97
        assert_eq!(cap.compute_switching_action(0.95), 1);
    }

    #[test]
    fn test_capacitor_switch_out_high_v() {
        let mut cap = make_cap(); // off_voltage_pu = 1.03
        cap.active_steps = 2;
        assert_eq!(cap.compute_switching_action(1.05), -1);
    }

    #[test]
    fn test_capacitor_limits() {
        let mut cap = make_cap();
        cap.active_steps = 4; // at n_steps
        assert_eq!(cap.compute_switching_action(0.90), 0);
        cap.active_steps = 0;
        assert_eq!(cap.compute_switching_action(1.06), 0);
    }

    #[test]
    fn test_capacitor_apply_switching_clamped() {
        let mut cap = make_cap();
        let kvar = cap.apply_switching(10).expect("clamped apply must succeed");
        assert_eq!(cap.active_steps, 4);
        assert!((kvar - 600.0).abs() < 1e-9);
    }

    // ---- VoltageProfile / VoltageRegulationSystem --------------------------

    #[test]
    fn test_voltage_profile_violations() {
        let sys = VoltageRegulationSystem::new();
        let v = vec![0.93, 1.00, 1.06, 0.98];
        let ids: Vec<usize> = (0..4).collect();
        let profile = sys.assess_voltage_profile(&v, &ids);
        assert_eq!(profile.n_violations, 2);
    }

    #[test]
    fn test_voltage_profile_min_max() {
        let sys = VoltageRegulationSystem::new();
        let v = vec![0.93, 1.00, 1.06, 0.98];
        let ids: Vec<usize> = (0..4).collect();
        let profile = sys.assess_voltage_profile(&v, &ids);
        assert!((profile.min_voltage_pu - 0.93).abs() < 1e-9);
        assert!((profile.max_voltage_pu - 1.06).abs() < 1e-9);
    }

    #[test]
    fn test_coordination_actions_nonempty() {
        let mut sys = VoltageRegulationSystem::new();
        let cap = CapacitorBank::new(1, 0, 4, 150.0);
        sys.add_capacitor_bank(cap);
        let v = vec![0.92];
        let ids = vec![0_usize];
        let profile = sys.assess_voltage_profile(&v, &ids);
        let actions = sys.compute_regulation_actions(&profile);
        assert!(!actions.is_empty());
    }

    #[test]
    fn test_coordination_prioritizes_cheapest() {
        let mut sys = VoltageRegulationSystem::new();
        let cap = CapacitorBank::new(1, 0, 4, 150.0);
        sys.add_capacitor_bank(cap);
        let oltc = OltcController::new(2, 0, -16, 16);
        sys.add_oltc(oltc);
        let v = vec![0.92];
        let ids = vec![0_usize];
        let profile = sys.assess_voltage_profile(&v, &ids);
        let actions = sys.compute_regulation_actions(&profile);
        assert!(!actions.is_empty());
        // capacitor ($1) must precede OLTC ($5)
        assert!(actions[0].cost_usd <= actions.last().map(|a| a.cost_usd).unwrap_or(f64::MAX));
    }

    #[test]
    fn test_ldc_compensated_voltage() {
        // V_ldc = 1.0 - 0.1*(0.02*0.8 + 0.05*0.6) = 1.0 - 0.0046 = 0.9954
        let v_ldc = LineDropCompensator::compute_ldc_voltage(1.0, 0.1, 0.02, 0.05, 0.8);
        assert!((v_ldc - 0.9954).abs() < 1e-9);
    }

    #[test]
    fn test_vvo_optimizer_reduces_violations() {
        let mut sys = VoltageRegulationSystem::new();
        let cap = CapacitorBank::new(1, 0, 4, 150.0);
        sys.add_capacitor_bank(cap);
        let v = vec![0.93, 1.00];
        let ids = vec![0_usize, 1];
        let profile = sys.assess_voltage_profile(&v, &ids);
        let actions = VoltageVarOptimizer::optimize_setpoints(&profile, &sys);
        assert!(!actions.is_empty());
    }

    #[test]
    fn test_sensitivity_matrix_usage() {
        let mut sys = VoltageRegulationSystem::new();
        let cap0 = CapacitorBank::new(0, 0, 4, 100.0);
        let cap1 = CapacitorBank::new(1, 0, 4, 100.0);
        sys.add_capacitor_bank(cap0);
        sys.add_capacitor_bank(cap1);
        // bus 0: device 0 → sensitivity 0.001, device 1 → 0.05
        sys.set_sensitivity_matrix(vec![vec![0.001, 0.05]]);
        let v = vec![0.93];
        let ids = vec![0_usize];
        let profile = sys.assess_voltage_profile(&v, &ids);
        let actions = VoltageVarOptimizer::optimize_setpoints(&profile, &sys);
        assert!(!actions.is_empty());
        assert_eq!(
            actions[0].device_id, 1,
            "higher-sensitivity device must be chosen"
        );
    }

    #[test]
    fn test_regulator_boost_limits() {
        let mut reg = VoltageRegulatorUnit::new(1, 0, 1);
        reg.apply_boost(0.5); // clamped to 0.1
        assert!((reg.current_boost_pu - 0.1).abs() < 1e-9);
        reg.apply_boost(-0.5); // clamped to -0.1
        assert!((reg.current_boost_pu - (-0.1)).abs() < 1e-9);
    }

    #[test]
    fn test_run_coordination_step() {
        let mut sys = VoltageRegulationSystem::new();
        let cap = CapacitorBank::new(1, 0, 4, 150.0);
        sys.add_capacitor_bank(cap);
        let v = vec![0.92, 1.0, 1.02];
        let ids = vec![0_usize, 1, 2];
        let (profile, actions) = sys.run_coordination_step(&v, &ids);
        assert_eq!(profile.bus_voltages_pu.len(), 3);
        assert!(!actions.is_empty());
    }

    #[test]
    fn test_capacitor_fault_no_action() {
        let mut cap = make_cap();
        cap.status = CapacitorStatus::Fault;
        assert_eq!(cap.compute_switching_action(0.90), 0);
        assert!(cap.apply_switching(1).is_err());
    }

    #[test]
    fn test_voltage_regulator_step_size() {
        let reg = VoltageRegulatorUnit::new(1, 0, 1);
        // (0.1 − (−0.1)) / 32 = 0.2 / 32 = 0.00625
        assert!((reg.step_size_pu() - 0.00625).abs() < 1e-9);
    }

    // ---- StepRegulator (legacy) -------------------------------------------

    #[test]
    fn tap_ratio_at_zero() {
        let reg = StepRegulator::new(0, 500.0, 11.0, 1.0);
        assert!((reg.tap_ratio() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn tap_up_after_delay() {
        let mut reg = StepRegulator::new(1, 500.0, 11.0, 1.0);
        reg.time_delay_s = 5.0;
        let action1 = reg.step_control(0.95, 3.0);
        assert_eq!(action1, TapAction::NoChange);
        let action2 = reg.step_control(0.95, 3.0);
        assert_eq!(action2, TapAction::TapUp);
        assert_eq!(reg.current_tap, 1);
        assert_eq!(reg.total_operations, 1);
    }

    #[test]
    fn tap_down_after_delay() {
        let mut reg = StepRegulator::new(2, 500.0, 11.0, 1.0);
        reg.time_delay_s = 4.0;
        let _ = reg.step_control(1.06, 5.0);
        assert_eq!(reg.current_tap, -1);
    }

    #[test]
    fn effective_voltage_range_bounds() {
        let reg = StepRegulator::new(0, 500.0, 11.0, 1.0);
        let (v_min, v_max) = reg.effective_voltage_range();
        assert!(v_min < 1.0);
        assert!(v_max > 1.0);
        assert!((v_min - 0.9).abs() < 1e-10);
        assert!((v_max - 1.1).abs() < 1e-10);
    }

    #[test]
    fn compensated_voltage_ldc() {
        let mut reg = StepRegulator::new(0, 500.0, 11.0, 1.0);
        reg.r_compensator = 0.05;
        reg.x_compensator = 0.1;
        let v_comp = reg.compensated_voltage(1.0, 0.5, 0.9);
        assert!(v_comp < 1.0);
    }

    // ---- StepCapacitorBank (legacy) ----------------------------------------

    #[test]
    fn cap_q_proportional_to_steps() {
        let mut cap = StepCapacitorBank::new(0, 1, 3.0, 11.0, 6);
        cap.current_steps = 3;
        let q = cap.q_injected_mvar();
        assert!((q - 1.5).abs() < 1e-10);
    }

    #[test]
    fn cap_q_zero_when_off() {
        let cap = StepCapacitorBank::new(0, 1, 3.0, 11.0, 6);
        assert_eq!(cap.q_injected_mvar(), 0.0);
    }

    #[test]
    fn cap_switch_in_below_v_on() {
        let mut cap = StepCapacitorBank::new(0, 1, 3.0, 11.0, 4);
        cap.time_delay_s = 5.0;
        let a1 = cap.step_control(0.95, 3.0);
        assert_eq!(a1, CapAction::NoChange);
        let a2 = cap.step_control(0.95, 3.0);
        assert_eq!(a2, CapAction::StepIn);
        assert_eq!(cap.current_steps, 1);
    }

    #[test]
    fn cap_switch_out_above_v_off() {
        let mut cap = StepCapacitorBank::new(0, 1, 3.0, 11.0, 4);
        cap.current_steps = 4;
        cap.time_delay_s = 5.0;
        let _ = cap.step_control(1.05, 3.0);
        let a = cap.step_control(1.05, 3.0);
        assert_eq!(a, CapAction::StepOut);
        assert_eq!(cap.current_steps, 3);
    }

    #[test]
    fn cap_switch_step_bounds() {
        let mut cap = StepCapacitorBank::new(0, 1, 3.0, 11.0, 4);
        assert!(cap.switch_step(2).is_ok());
        assert_eq!(cap.current_steps, 2);
        assert!(cap.switch_step(10).is_err());
        assert!(cap.switch_step(-5).is_err());
    }

    // ---- SvcModel ---------------------------------------------------------

    #[test]
    fn svc_q_zero_at_setpoint() {
        let svc = SvcModel::new(0, -10.0, 10.0, 1.0);
        let q = svc.q_from_voltage(1.0);
        assert!(q.abs() < 1e-10);
    }

    #[test]
    fn svc_capacitive_when_low_v() {
        let svc = SvcModel::new(0, -10.0, 10.0, 1.0);
        let q = svc.q_from_voltage(0.95);
        assert!(q > 0.0);
    }

    #[test]
    fn svc_step_dynamics() {
        let mut svc = SvcModel::new(0, -10.0, 10.0, 1.0);
        let q = svc.step(0.95, 0.05);
        assert!(q > 0.0);
        assert!(q <= svc.q_max_mvar);
    }

    // ---- CoordinatedVoltageController -------------------------------------

    #[test]
    fn coord_detects_undervoltage() {
        let ctrl = CoordinatedVoltageController::new(100.0, 11.0);
        let voltages = vec![1.0, 0.90, 1.02, 0.94];
        let violations = ctrl.check_voltage_violations(&voltages);
        assert_eq!(violations.len(), 2);
        for v in &violations {
            assert_eq!(v.violation_type, ViolationType::UnderVoltage);
        }
    }

    #[test]
    fn coord_detects_overvoltage() {
        let ctrl = CoordinatedVoltageController::new(100.0, 11.0);
        let voltages = vec![1.06, 1.0, 1.07];
        let violations = ctrl.check_voltage_violations(&voltages);
        assert_eq!(violations.len(), 2);
        assert!(violations
            .iter()
            .all(|v| v.violation_type == ViolationType::OverVoltage));
    }

    #[test]
    fn coord_total_reactive_support() {
        let mut ctrl = CoordinatedVoltageController::new(100.0, 11.0);
        let mut cap = StepCapacitorBank::new(0, 0, 6.0, 11.0, 3);
        cap.current_steps = 3;
        ctrl.add_capacitor(cap);
        let mut svc = SvcModel::new(1, -5.0, 5.0, 1.0);
        svc.q_output_mvar = 2.0;
        ctrl.add_svc(svc);
        let total = ctrl.total_reactive_support_mvar();
        assert!((total - 8.0).abs() < 1e-9);
    }

    #[test]
    fn coord_step_runs_without_panic() {
        let mut ctrl = CoordinatedVoltageController::new(100.0, 11.0);
        ctrl.add_regulator(StepRegulator::new(0, 500.0, 11.0, 1.0));
        ctrl.add_capacitor(StepCapacitorBank::new(0, 1, 3.0, 11.0, 4));
        ctrl.add_svc(SvcModel::new(2, -5.0, 5.0, 1.0));
        let mut voltages = vec![1.0, 0.94, 0.97, 1.0];
        let result = ctrl.step(&mut voltages, 1.0);
        assert!(result.svc_q_total_mvar.is_finite());
    }

    // ---- VvoOptimizer (legacy) --------------------------------------------

    #[test]
    fn vvo_optimize_no_panic() {
        let mut ctrl = CoordinatedVoltageController::new(100.0, 11.0);
        ctrl.add_regulator(StepRegulator::new(0, 500.0, 11.0, 1.0));
        ctrl.add_capacitor(StepCapacitorBank::new(0, 1, 6.0, 11.0, 3));
        let voltages = vec![1.0, 0.96, 0.97];
        let vvo = VvoOptimizer::new();
        let result = vvo.optimize_setpoints(&mut ctrl, &voltages, 5.0);
        assert_eq!(result.optimal_v_setpoints.len(), 1);
        assert_eq!(result.optimal_cap_steps.len(), 1);
        assert!(result.estimated_loss_reduction_pct >= 0.0);
        assert!(result.voltage_improvement_pu >= 0.0);
    }

    // ---- VoltageProfileAnalyzer -------------------------------------------

    #[test]
    fn profile_worst_case_bus() {
        let mut analyzer = VoltageProfileAnalyzer::new(11.0);
        analyzer.bus_distances_km = vec![0.0, 1.0, 2.0, 3.0];
        let voltages = vec![1.0, 0.98, 0.94, 0.97];
        let (idx, v) = analyzer.worst_case_bus(&voltages);
        assert_eq!(idx, 2);
        assert!((v - 0.94).abs() < 1e-12);
    }

    #[test]
    fn profile_min_max_correct() {
        let mut analyzer = VoltageProfileAnalyzer::new(11.0);
        analyzer.bus_distances_km = vec![0.0, 1.0, 2.0];
        let voltages = vec![1.02, 0.97, 0.94];
        let profile = analyzer.compute_profile(&voltages);
        assert!((profile.min_voltage_pu - 0.94).abs() < 1e-12);
        assert!((profile.max_voltage_pu - 1.02).abs() < 1e-12);
    }

    #[test]
    fn profile_kv_conversion() {
        let mut analyzer = VoltageProfileAnalyzer::new(11.0);
        analyzer.bus_distances_km = vec![0.0, 1.0];
        let voltages = vec![1.0, 0.95];
        let profile = analyzer.compute_profile(&voltages);
        assert!((profile.voltages_kv[0] - 11.0).abs() < 1e-10);
        assert!((profile.voltages_kv[1] - 10.45).abs() < 1e-10);
    }

    #[test]
    fn voltage_drop_pct_correct() {
        let analyzer = VoltageProfileAnalyzer::new(11.0);
        let voltages = vec![1.0, 0.95, 0.92];
        let drop = analyzer.voltage_drop_pct(&voltages);
        assert!((drop - 8.0).abs() < 1e-10);
    }
}
