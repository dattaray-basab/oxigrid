//! Auto-generated module
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

use super::types::{
    CapAction, CapacitorBank, OltcController, RegulatorType, StepRegulator, SvcModel,
    ViolationType, VoltageProfile, VoltageRegulatorUnit, VoltageViolation,
};

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
/// Summary of a single coordinated control time step.
#[derive(Debug, Clone)]
pub struct CoordVoltageResult {
    /// Number of tap-change operations executed this step.
    pub tap_changes: usize,
    /// Number of capacitor-step switching operations this step.
    pub cap_switchings: usize,
    /// Total SVC reactive power output this step ``MVAr``.
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
    /// System base MVA ``MVA``.
    pub base_mva: f64,
    /// System base voltage ``kV``.
    pub base_kv: f64,
    /// Acceptable voltage window `(V_min, V_max)` ``pu``.
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
    /// Advances all devices by `dt_s` ``s`` and updates `bus_voltages` in place.
    pub fn step(&mut self, bus_voltages: &mut [f64], dt_s: f64) -> CoordVoltageResult {
        let mut tap_changes = 0usize;
        let mut cap_switchings = 0usize;
        let mut svc_q_total = 0.0_f64;
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
    /// Sum of all capacitor and SVC reactive power currently injected ``MVAr``.
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
    /// Estimated improvement in bus voltage magnitude ``pu``.
    pub expected_voltage_improvement_pu: f64,
    /// Estimated monetary cost of performing this action ``USD``.
    pub cost_usd: f64,
}
/// Control mode for a switched capacitor bank (legacy coordinator API).
#[derive(Debug, Clone, PartialEq)]
pub enum CapControlMode {
    /// Switch based on local bus voltage ``pu``.
    Voltage,
    /// Switch based on reactive power demand.
    Reactive,
    /// Schedule-based (time-of-day) switching.
    Time,
    /// Switch based on feeder current magnitude.
    Current,
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
    /// Total rated reactive power ``MVAr``.
    pub q_rated_mvar: f64,
    /// Rated terminal voltage ``kV``.
    pub voltage_kv: f64,
    /// Total number of switchable sections.
    pub steps: usize,
    /// Currently energised sections.
    pub current_steps: usize,
    /// Active control mode.
    pub control_mode: CapControlMode,
    /// Switch-in voltage threshold ``pu``.
    pub v_on_pu: f64,
    /// Switch-out voltage threshold ``pu``.
    pub v_off_pu: f64,
    /// Minimum time the voltage must remain outside threshold ``s``.
    pub time_delay_s: f64,
    /// Accumulated time the voltage has been outside threshold ``s``.
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
    /// Reactive power currently injected ``MVAr``.
    #[inline]
    pub fn q_injected_mvar(&self) -> f64 {
        if self.steps == 0 {
            return 0.0;
        }
        self.q_rated_mvar * self.current_steps as f64 / self.steps as f64
    }
    /// Per-unit susceptance of the currently injected reactive power ``pu``.
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
/// Snapshot of the voltage profile along a distribution feeder.
#[derive(Debug, Clone)]
pub struct FeederVoltageProfile {
    /// Distance from substation for each bus ``km``.
    pub distances: Vec<f64>,
    /// Per-unit bus voltages ``pu``.
    pub voltages_pu: Vec<f64>,
    /// Absolute bus voltages ``kV``.
    pub voltages_kv: Vec<f64>,
    /// Minimum bus voltage in the profile ``pu``.
    pub min_voltage_pu: f64,
    /// Maximum bus voltage in the profile ``pu``.
    pub max_voltage_pu: f64,
    /// Mean bus voltage across the profile ``pu``.
    pub mean_voltage_pu: f64,
}

#[cfg(test)]
mod tests {
    use super::super::types::ViolationType;
    use super::*;

    #[test]
    fn voltage_regulation_system_new_defaults() {
        let sys = VoltageRegulationSystem::new();
        assert!((sys.min_voltage_limit_pu - 0.95).abs() < 1e-9);
        assert!((sys.max_voltage_limit_pu - 1.05).abs() < 1e-9);
        assert!(sys.oltc_controllers.is_empty());
        assert!(sys.capacitor_banks.is_empty());
    }

    #[test]
    fn assess_voltage_profile_counts_violations() {
        let sys = VoltageRegulationSystem::new();
        let voltages = vec![0.93, 1.0, 1.07];
        let bus_ids = vec![0usize, 1, 2];
        let profile = sys.assess_voltage_profile(&voltages, &bus_ids);
        assert_eq!(profile.n_violations, 2);
    }

    #[test]
    fn assess_voltage_profile_min_max() {
        let sys = VoltageRegulationSystem::new();
        let voltages = vec![0.96, 1.02, 1.04];
        let bus_ids = vec![0usize, 1, 2];
        let profile = sys.assess_voltage_profile(&voltages, &bus_ids);
        assert!((profile.min_voltage_pu - 0.96).abs() < 1e-9);
        assert!((profile.max_voltage_pu - 1.04).abs() < 1e-9);
    }

    #[test]
    fn coordinated_controller_new_defaults() {
        let ctrl = CoordinatedVoltageController::new(100.0, 11.0);
        assert!((ctrl.base_mva - 100.0).abs() < 1e-9);
        assert!((ctrl.base_kv - 11.0).abs() < 1e-9);
        assert!((ctrl.v_limits.0 - 0.95).abs() < 1e-9);
        assert!((ctrl.v_limits.1 - 1.05).abs() < 1e-9);
        assert!(ctrl.regulators.is_empty());
    }

    #[test]
    fn check_voltage_violations_under_and_over() {
        let ctrl = CoordinatedVoltageController::new(100.0, 11.0);
        let voltages = vec![0.9, 1.0, 1.1];
        let violations = ctrl.check_voltage_violations(&voltages);
        assert_eq!(violations.len(), 2);
        let first = violations
            .iter()
            .find(|v| v.bus == 0)
            .expect("should have violation at bus 0");
        assert_eq!(first.violation_type, ViolationType::UnderVoltage);
        let last = violations
            .iter()
            .find(|v| v.bus == 2)
            .expect("should have violation at bus 2");
        assert_eq!(last.violation_type, ViolationType::OverVoltage);
    }

    #[test]
    fn step_capacitor_bank_q_injected() {
        let mut cap = StepCapacitorBank::new(0, 0, 10.0, 11.0, 4);
        cap.switch_step(2)
            .expect("switching 2 steps in should succeed");
        let q = cap.q_injected_mvar();
        assert!((q - 5.0).abs() < 1e-9);
    }

    #[test]
    fn step_capacitor_bank_switch_step_bounds() {
        let mut cap = StepCapacitorBank::new(0, 0, 10.0, 11.0, 4);
        let result_over = cap.switch_step(6);
        assert!(result_over.is_err());
        let result_under = cap.switch_step(-1);
        assert!(result_under.is_err());
    }

    #[test]
    fn tap_action_at_limit_contains_message() {
        let msg = "reg 0 at max tap 16".to_string();
        let action = TapAction::AtLimit(msg);
        match action {
            TapAction::AtLimit(s) => assert!(!s.is_empty()),
            _ => panic!("expected AtLimit variant"),
        }
    }
}
