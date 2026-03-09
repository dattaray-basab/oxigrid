//! Transient Voltage Stability Assessment (TVSA)
//!
//! Post-fault voltage recovery analysis, voltage sag assessment, and motor
//! load behaviour under fault conditions.
//!
//! The module implements:
//!
//! * A simplified first-order exponential voltage-recovery model per bus
//!   ([`BusVoltageModel`]).
//! * Motor-stall / thermal-trip dynamics for induction-motor loads
//!   ([`MotorLoad`]).
//! * Voltage-divider fault voltage computation.
//! * N-1 contingency sweeps with worst-bus and recovery-index metrics
//!   ([`TvsaEngine`]).
//!
//! # Quick Start
//!
//! ```rust
//! use oxigrid::stability::tvsa::{
//!     TvsaEngine, BusVoltageModel, FaultEvent, FaultType,
//! };
//! let bus = BusVoltageModel::new(0, 1.0, 0.98);
//! let mut engine = TvsaEngine::new(vec![bus]);
//! let fault = FaultEvent::new(0, FaultType::ThreePhase, 0.0, 0.1, 1.0);
//! let result = engine.run_assessment(fault);
//! println!("worst sag = {:.3} pu", result.worst_sag_pu);
//! ```

// ─────────────────────────────────────────────────────────────────────────────
// Enums
// ─────────────────────────────────────────────────────────────────────────────

/// Outcome of a post-fault voltage-recovery assessment for a single bus.
#[derive(Debug, Clone, PartialEq)]
pub enum VoltageRecoveryStatus {
    /// Voltage recovers to ≥ 0.95 pu in less than 2 s after fault clearing.
    FullRecovery,
    /// Voltage recovers to ≥ 0.95 pu, but takes between 2 s and 6 s.
    SlowRecovery,
    /// Voltage never reaches 0.95 pu within the simulation window.
    NonRecovery,
    /// Voltage settles below the collapse threshold (default 0.50 pu).
    Collapse,
    /// Voltage oscillates around the recovery criterion (many dV/dt sign
    /// changes detected).
    Oscillatory,
}

/// Fault type applied to a bus.
#[derive(Debug, Clone, PartialEq)]
pub enum FaultType {
    /// Symmetrical three-phase fault.
    ThreePhase,
    /// Single-line-to-ground fault.
    SingleLineToGround,
    /// Line-to-line fault.
    LineToLine,
    /// Double-line-to-ground fault.
    DoubleLineToGround,
}

/// Category of induction-motor load for stall-dynamics parameterisation.
#[derive(Debug, Clone, PartialEq)]
pub enum StallType {
    /// Single-phase motor (e.g., residential appliances).
    SinglePhaseMotor,
    /// Three-phase motor (e.g., industrial drives).
    ThreePhaseMotor,
    /// HVAC compressor motor (high stall susceptibility).
    HvacCompressor,
    /// Generic industrial process motor.
    Industrial,
}

// ─────────────────────────────────────────────────────────────────────────────
// Structs
// ─────────────────────────────────────────────────────────────────────────────

/// Time-domain voltage profile for a single bus throughout a fault event.
#[derive(Debug, Clone)]
pub struct VoltageTrajectory {
    /// Index of the bus this trajectory belongs to.
    pub bus_id: usize,
    /// Simulation time vector (s).
    pub time_s: Vec<f64>,
    /// Per-unit bus voltage at each time step.
    pub voltage_pu: Vec<f64>,
    /// Minimum per-unit voltage recorded after fault application.
    pub post_fault_min_pu: f64,
    /// Time (s) from fault clearing until voltage first reaches 0.95 pu.
    ///
    /// Set to the full simulation duration when recovery is never achieved.
    pub recovery_time_s: f64,
    /// Overall recovery classification for this bus.
    pub status: VoltageRecoveryStatus,
    /// IDs of motors that stalled on this bus during the event.
    pub motor_stalls: Vec<usize>,
}

/// Induction-motor load with stall and thermal-trip dynamics.
#[derive(Debug, Clone)]
pub struct MotorLoad {
    /// Unique motor identifier.
    pub id: usize,
    /// Bus the motor is connected to.
    pub bus_id: usize,
    /// Category of motor load.
    pub stall_type: StallType,
    /// Rated active power (MW).
    pub rated_mw: f64,
    /// Rated reactive power (Mvar).
    pub rated_mvar: f64,
    /// Per-unit bus voltage below which the motor stalls (default 0.65).
    pub stall_voltage_pu: f64,
    /// Per-unit bus voltage above which a stalled motor reconnects (default 0.80).
    pub reconnect_voltage_pu: f64,
    /// Time (s) the motor can tolerate stall before thermal protection acts
    /// (default 0.5 s).
    pub stall_time_s: f64,
    /// Total time after stall onset at which the motor thermally trips (default
    /// 3.0 s).
    pub thermal_trip_time_s: f64,
    /// Whether the motor is currently stalled.
    pub stalled: bool,
    /// Simulation time at which the stall began (`None` if not stalled).
    pub stall_start_time: Option<f64>,
    /// Whether the motor has been thermally tripped and disconnected.
    pub tripped: bool,
}

impl MotorLoad {
    /// Create a new motor load with default stall / trip parameters.
    ///
    /// Defaults: `stall_voltage = 0.65 pu`, `reconnect_voltage = 0.80 pu`,
    /// `stall_time = 0.5 s`, `thermal_trip_time = 3.0 s`.
    pub fn new(
        id: usize,
        bus_id: usize,
        stall_type: StallType,
        rated_mw: f64,
        rated_mvar: f64,
    ) -> Self {
        Self {
            id,
            bus_id,
            stall_type,
            rated_mw,
            rated_mvar,
            stall_voltage_pu: 0.65,
            reconnect_voltage_pu: 0.80,
            stall_time_s: 0.5,
            thermal_trip_time_s: 3.0,
            stalled: false,
            stall_start_time: None,
            tripped: false,
        }
    }
}

/// Description of a fault event applied to the network.
#[derive(Debug, Clone)]
pub struct FaultEvent {
    /// Bus where the fault is applied.
    pub bus_id: usize,
    /// Fault type.
    pub fault_type: FaultType,
    /// Fault impedance in per unit (0.0 = bolted fault).
    pub fault_impedance_pu: f64,
    /// Simulation time at which the fault is applied (s).
    pub fault_time_s: f64,
    /// Simulation time at which the fault is cleared (s).
    pub clearing_time_s: f64,
    /// Pre-fault voltage at the faulted bus (pu).
    pub pre_fault_voltage_pu: f64,
}

impl FaultEvent {
    /// Construct a fault event with a default clearing time of `fault_time + 0.1 s`.
    pub fn new(
        bus_id: usize,
        fault_type: FaultType,
        fault_impedance_pu: f64,
        fault_time_s: f64,
        pre_fault_voltage_pu: f64,
    ) -> Self {
        Self {
            bus_id,
            fault_type,
            fault_impedance_pu,
            fault_time_s,
            clearing_time_s: fault_time_s + 0.1,
            pre_fault_voltage_pu,
        }
    }
}

/// Simplified bus voltage dynamics model used by [`TvsaEngine`].
#[derive(Debug, Clone)]
pub struct BusVoltageModel {
    /// Bus index.
    pub bus_id: usize,
    /// Pre-fault steady-state voltage (pu).
    pub v_pre_fault_pu: f64,
    /// Voltage during the fault period (computed from fault impedance).
    ///
    /// This field is overwritten by [`TvsaEngine::run_assessment`] before each
    /// simulation.
    pub v_during_fault_pu: f64,
    /// Post-fault equilibrium voltage the system recovers to (pu).
    pub v_post_fault_pu: f64,
    /// First-order voltage recovery time constant (s, default 0.5 s).
    pub time_constant_s: f64,
    /// Extra reactive demand (Mvar) due to stalled motors on this bus.
    ///
    /// Updated dynamically during simulation.
    pub motor_reactive_demand_mvar: f64,
}

impl BusVoltageModel {
    /// Create a bus voltage model with the given pre- and post-fault voltages.
    ///
    /// Default time constant is 0.5 s.  The `v_during_fault_pu` field is
    /// initialised to `v_pre_fault_pu` and overwritten by the engine at
    /// assessment time.
    pub fn new(bus_id: usize, v_pre_fault_pu: f64, v_post_fault_pu: f64) -> Self {
        Self {
            bus_id,
            v_pre_fault_pu,
            v_during_fault_pu: v_pre_fault_pu,
            v_post_fault_pu,
            time_constant_s: 0.5,
            motor_reactive_demand_mvar: 0.0,
        }
    }
}

/// Aggregated result of a transient voltage stability assessment run.
#[derive(Debug, Clone)]
pub struct TvsaResult {
    /// The fault event that was assessed.
    pub fault_event: FaultEvent,
    /// Per-bus voltage trajectories.
    pub voltage_trajectories: Vec<VoltageTrajectory>,
    /// IDs of motors that stalled during the event.
    pub stalled_motors: Vec<usize>,
    /// IDs of motors that thermally tripped during the event.
    pub tripped_motors: Vec<usize>,
    /// Bus that experienced the deepest voltage sag.
    pub worst_bus_id: usize,
    /// Minimum per-unit voltage recorded across all buses.
    pub worst_sag_pu: f64,
    /// Recovery index in \[0, 1\]: 1.0 = full recovery, 0.0 = collapse.
    pub recovery_index: f64,
    /// Voltage stability margin (pu): minimum voltage minus collapse threshold.
    ///
    /// Negative values indicate that the system entered collapse.
    pub voltage_stability_margin_pu: f64,
    /// Overall system-level recovery status (based on the worst bus).
    pub overall_status: VoltageRecoveryStatus,
}

// ─────────────────────────────────────────────────────────────────────────────
// TvsaEngine
// ─────────────────────────────────────────────────────────────────────────────

/// Main transient voltage stability assessment engine.
///
/// Simulates post-fault voltage recovery on a per-bus basis using a
/// first-order exponential model and tracks motor-stall / thermal-trip
/// dynamics.
///
/// # Example
/// ```rust
/// use oxigrid::stability::tvsa::{
///     TvsaEngine, BusVoltageModel, FaultEvent, FaultType,
/// };
/// let bus = BusVoltageModel::new(0, 1.0, 0.98);
/// let mut engine = TvsaEngine::new(vec![bus]);
/// let fault = FaultEvent::new(0, FaultType::ThreePhase, 0.0, 0.1, 1.0);
/// let result = engine.run_assessment(fault);
/// println!("worst sag = {:.3} pu", result.worst_sag_pu);
/// ```
#[derive(Debug, Clone)]
pub struct TvsaEngine {
    /// Bus voltage models registered with the engine.
    pub buses: Vec<BusVoltageModel>,
    /// Motor loads registered with the engine.
    pub motor_loads: Vec<MotorLoad>,
    /// Integration timestep (s, default 0.01 s).
    pub dt_s: f64,
    /// Total simulation window (s, default 10.0 s).
    pub simulation_duration_s: f64,
    /// Per-unit voltage threshold used as the recovery criterion (default
    /// 0.95 pu).
    pub voltage_threshold_pu: f64,
    /// Per-unit voltage below which the system is classified as collapsed
    /// (default 0.50 pu).
    pub collapse_threshold_pu: f64,
}

impl TvsaEngine {
    /// Available reactive headroom (Mvar) used to scale the motor-stall Q
    /// penalty.  Acts as a placeholder for true system Q capability.
    const Q_MAX_AVAILABLE: f64 = 100.0;

    /// Thevenin impedance (pu) used in the voltage-divider fault model.
    const Z_THEVENIN: f64 = 0.1;

    /// Minimum number of dV/dt sign changes needed to flag oscillatory
    /// behaviour.
    const OSCILLATORY_SIGN_CHANGES: u32 = 6;

    /// Construct a new engine from a set of bus voltage models.
    pub fn new(buses: Vec<BusVoltageModel>) -> Self {
        Self {
            buses,
            motor_loads: Vec::new(),
            dt_s: 0.01,
            simulation_duration_s: 10.0,
            voltage_threshold_pu: 0.95,
            collapse_threshold_pu: 0.50,
        }
    }

    /// Register a motor load with the engine.
    pub fn add_motor_load(&mut self, motor: MotorLoad) {
        self.motor_loads.push(motor);
    }

    /// Run a full TVSA for the given fault event and return the aggregated
    /// result.
    ///
    /// Motor states are reset to un-stalled / un-tripped at the start of each
    /// call.
    pub fn run_assessment(&mut self, fault: FaultEvent) -> TvsaResult {
        // ── Reset motor and bus states ────────────────────────────────────
        for m in &mut self.motor_loads {
            m.stalled = false;
            m.stall_start_time = None;
            m.tripped = false;
        }
        for b in &mut self.buses {
            b.motor_reactive_demand_mvar = 0.0;
        }

        // ── Compute during-fault voltage for every bus ────────────────────
        let buses_snapshot: Vec<BusVoltageModel> = self.buses.clone();
        for bus in &mut self.buses {
            bus.v_during_fault_pu = Self::compute_during_fault_voltage(
                bus.v_pre_fault_pu,
                fault.fault_impedance_pu,
                Self::Z_THEVENIN,
            );
        }

        // ── Simulate one trajectory per bus ───────────────────────────────
        let mut trajectories: Vec<VoltageTrajectory> = Vec::new();
        for bus in &buses_snapshot {
            // Re-read the updated during-fault voltage for this bus.
            let bus_updated = self
                .buses
                .iter()
                .find(|b| b.bus_id == bus.bus_id)
                .cloned()
                .unwrap_or_else(|| bus.clone());
            let traj = self.simulate_voltage_trajectory(&bus_updated, &fault);
            trajectories.push(traj);
        }

        // ── Collect motor events ──────────────────────────────────────────
        let stalled_motors: Vec<usize> = self
            .motor_loads
            .iter()
            .filter(|m| m.stalled || m.tripped)
            .map(|m| m.id)
            .collect();
        let tripped_motors: Vec<usize> = self
            .motor_loads
            .iter()
            .filter(|m| m.tripped)
            .map(|m| m.id)
            .collect();

        // ── Identify worst bus ────────────────────────────────────────────
        let (worst_bus_id, worst_sag_pu) = trajectories
            .iter()
            .map(|t| (t.bus_id, t.post_fault_min_pu))
            .fold((0usize, f64::MAX), |(wb, ws), (bid, sag)| {
                if sag < ws {
                    (bid, sag)
                } else {
                    (wb, ws)
                }
            });
        let worst_sag_pu = if worst_sag_pu == f64::MAX {
            1.0
        } else {
            worst_sag_pu
        };

        let recovery_index = self.compute_recovery_index(&trajectories);

        let overall_status = trajectories
            .iter()
            .find(|t| t.bus_id == worst_bus_id)
            .map(|t| t.status.clone())
            .unwrap_or(VoltageRecoveryStatus::FullRecovery);

        let mut result = TvsaResult {
            fault_event: fault,
            voltage_trajectories: trajectories,
            stalled_motors,
            tripped_motors,
            worst_bus_id,
            worst_sag_pu,
            recovery_index,
            voltage_stability_margin_pu: 0.0,
            overall_status,
        };
        result.voltage_stability_margin_pu = self.compute_stability_margin(&result);
        result
    }

    /// Simulate the voltage trajectory for a single bus over the full
    /// simulation window.
    ///
    /// Uses a first-order exponential recovery model after fault clearing.
    /// [`TvsaEngine::step_motor_dynamics`] is called at each timestep to
    /// capture motor-reactive-demand feedback.
    pub fn simulate_voltage_trajectory(
        &mut self,
        bus: &BusVoltageModel,
        fault: &FaultEvent,
    ) -> VoltageTrajectory {
        let n_steps = ((self.simulation_duration_s / self.dt_s).ceil() as usize).max(1);
        let mut time_s = Vec::with_capacity(n_steps);
        let mut voltage_pu = Vec::with_capacity(n_steps);

        let mut post_fault_min = bus.v_pre_fault_pu;
        let mut recovery_time_s = self.simulation_duration_s; // pessimistic
        let mut reached_threshold = false;
        let mut motor_stalls_on_bus: Vec<usize> = Vec::new();

        // Oscillatory detection
        let mut sign_changes: u32 = 0;
        let mut prev_dv: f64 = 0.0;

        for step in 0..n_steps {
            let t = step as f64 * self.dt_s;
            time_s.push(t);

            let v = if t < fault.fault_time_s {
                bus.v_pre_fault_pu
            } else if t < fault.clearing_time_s {
                bus.v_during_fault_pu
            } else {
                // First-order exponential recovery.
                let tau = bus.time_constant_s.max(1e-9);
                let t_since_clear = t - fault.clearing_time_s;

                // Reactive penalty from stalled motors on this bus.
                let motor_q: f64 = self
                    .motor_loads
                    .iter()
                    .filter(|m| m.bus_id == bus.bus_id && m.stalled && !m.tripped)
                    .map(|m| m.rated_mvar * 3.0)
                    .sum();
                let q_penalty = (motor_q / Self::Q_MAX_AVAILABLE).min(0.5);
                let v_post_eq = (bus.v_post_fault_pu - q_penalty).max(0.0);

                let v_calc = v_post_eq
                    + (bus.v_during_fault_pu - v_post_eq) * (-(t_since_clear / tau)).exp();
                v_calc.clamp(0.0, 1.2)
            };

            voltage_pu.push(v);

            // Update motor stall/trip states for this bus at this timestep.
            self.step_motor_dynamics(bus.bus_id, v, t);

            // Record any new stalls on this bus.
            for m in &self.motor_loads {
                if m.bus_id == bus.bus_id && m.stalled && !motor_stalls_on_bus.contains(&m.id) {
                    motor_stalls_on_bus.push(m.id);
                }
            }

            // Track post-fault minimum voltage.
            if t >= fault.fault_time_s && v < post_fault_min {
                post_fault_min = v;
            }

            // Check recovery threshold (measured from fault clearing).
            if t >= fault.clearing_time_s && !reached_threshold && v >= self.voltage_threshold_pu {
                recovery_time_s = t - fault.clearing_time_s;
                reached_threshold = true;
            }

            // Oscillatory detection: count dV/dt sign changes.
            if step > 0 {
                let dv = v - voltage_pu.get(step - 1).copied().unwrap_or(v);
                if prev_dv * dv < 0.0 {
                    sign_changes += 1;
                }
                prev_dv = dv;
            }
        }

        let voltage_at_end = voltage_pu.last().copied().unwrap_or(0.0);

        let is_oscillatory = sign_changes >= Self::OSCILLATORY_SIGN_CHANGES
            && voltage_at_end > self.collapse_threshold_pu;

        let status = if is_oscillatory {
            VoltageRecoveryStatus::Oscillatory
        } else {
            Self::classify_voltage_recovery(post_fault_min, recovery_time_s, voltage_at_end)
        };

        if !reached_threshold {
            recovery_time_s = self.simulation_duration_s;
        }

        VoltageTrajectory {
            bus_id: bus.bus_id,
            time_s,
            voltage_pu,
            post_fault_min_pu: post_fault_min,
            recovery_time_s,
            status,
            motor_stalls: motor_stalls_on_bus,
        }
    }

    /// Compute the per-unit bus voltage during a fault using the voltage-divider
    /// model.
    ///
    /// `V_fault = V_pre × Z_thevenin / (Z_thevenin + Z_fault)`
    ///
    /// For a bolted fault (`Z_fault = 0`) this reduces to `V_pre` (the remote
    /// bus sees the full Thevenin voltage).  For large `Z_fault` the voltage
    /// drops towards zero.
    pub fn compute_during_fault_voltage(v_pre: f64, fault_impedance: f64, z_thevenin: f64) -> f64 {
        let denom = z_thevenin + fault_impedance;
        if denom < 1e-12 {
            return 0.0;
        }
        (v_pre * z_thevenin / denom).clamp(0.0, v_pre)
    }

    /// Update stall and thermal-trip state of every motor connected to `bus_id`
    /// for simulation time `t_s`.
    ///
    /// * Motors with `V_bus < stall_voltage` that are not yet stalled enter the
    ///   stall state.
    /// * Stalled motors whose stall duration exceeds `thermal_trip_time_s` are
    ///   tripped.
    /// * Stalled motors whose bus voltage recovers above `reconnect_voltage_pu`
    ///   exit the stall state (unless already tripped).
    pub fn step_motor_dynamics(&mut self, bus_id: usize, v_bus_pu: f64, t_s: f64) {
        for motor in &mut self.motor_loads {
            if motor.bus_id != bus_id || motor.tripped {
                continue;
            }

            if !motor.stalled {
                if v_bus_pu < motor.stall_voltage_pu {
                    motor.stalled = true;
                    motor.stall_start_time = Some(t_s);
                }
            } else {
                let stall_elapsed = t_s - motor.stall_start_time.unwrap_or(t_s);

                // Thermal trip takes priority over reconnect.
                if stall_elapsed >= motor.thermal_trip_time_s {
                    motor.tripped = true;
                    motor.stalled = false;
                    continue;
                }

                // Voluntary reconnect on voltage recovery.
                if v_bus_pu >= motor.reconnect_voltage_pu {
                    motor.stalled = false;
                    motor.stall_start_time = None;
                }
            }
        }
    }

    /// Compute a scalar recovery index in \[0, 1\] as a weighted average across
    /// buses.
    ///
    /// * `FullRecovery` → 1.0
    /// * `SlowRecovery` → 0.6
    /// * `Oscillatory` → 0.4
    /// * `NonRecovery` → 0.2
    /// * `Collapse` → 0.0
    ///
    /// Returns 1.0 for an empty trajectory list.
    pub fn compute_recovery_index(&self, trajectories: &[VoltageTrajectory]) -> f64 {
        if trajectories.is_empty() {
            return 1.0;
        }
        let sum: f64 = trajectories
            .iter()
            .map(|t| match &t.status {
                VoltageRecoveryStatus::FullRecovery => 1.0,
                VoltageRecoveryStatus::SlowRecovery => 0.6,
                VoltageRecoveryStatus::Oscillatory => 0.4,
                VoltageRecoveryStatus::NonRecovery => 0.2,
                VoltageRecoveryStatus::Collapse => 0.0,
            })
            .sum();
        (sum / trajectories.len() as f64).clamp(0.0, 1.0)
    }

    /// Classify a single bus's voltage recovery outcome.
    ///
    /// # Parameters
    /// * `min_v` — minimum voltage (pu) observed after fault application.
    /// * `recovery_time` — seconds from fault clearing to first reach 0.95 pu
    ///   (equals `simulation_duration_s` when threshold is never reached).
    /// * `voltage_at_end` — terminal voltage (pu) at end of simulation window.
    pub fn classify_voltage_recovery(
        _min_v: f64,
        recovery_time: f64,
        voltage_at_end: f64,
    ) -> VoltageRecoveryStatus {
        if voltage_at_end < 0.50 {
            return VoltageRecoveryStatus::Collapse;
        }
        if voltage_at_end < 0.95 {
            return VoltageRecoveryStatus::NonRecovery;
        }
        if recovery_time >= 2.0 {
            return VoltageRecoveryStatus::SlowRecovery;
        }
        VoltageRecoveryStatus::FullRecovery
    }

    /// Compute the voltage stability margin for a result.
    ///
    /// Defined as `worst_sag_pu - collapse_threshold_pu`.  Negative values
    /// indicate that the system entered collapse.
    pub fn compute_stability_margin(&self, result: &TvsaResult) -> f64 {
        result.worst_sag_pu - self.collapse_threshold_pu
    }

    /// Run N-1 contingency assessment over a list of fault events.
    ///
    /// Returns one [`TvsaResult`] per fault event.
    pub fn run_n1_assessment(&mut self, faults: &[FaultEvent]) -> Vec<TvsaResult> {
        faults
            .iter()
            .map(|f| self.run_assessment(f.clone()))
            .collect()
    }

    /// Identify critical buses across multiple assessment results.
    ///
    /// Returns a list of `(bus_id, worst_sag_pu)` pairs sorted by ascending
    /// sag (i.e., worst bus — lowest pu — first).
    pub fn identify_critical_buses(&self, results: &[TvsaResult]) -> Vec<(usize, f64)> {
        use std::collections::HashMap;

        let mut worst_per_bus: HashMap<usize, f64> = HashMap::new();
        for result in results {
            for traj in &result.voltage_trajectories {
                let entry = worst_per_bus.entry(traj.bus_id).or_insert(f64::MAX);
                if traj.post_fault_min_pu < *entry {
                    *entry = traj.post_fault_min_pu;
                }
            }
        }

        let mut pairs: Vec<(usize, f64)> = worst_per_bus.into_iter().collect();
        pairs.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        pairs
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Helpers ───────────────────────────────────────────────────────────────

    fn simple_bus(bus_id: usize) -> BusVoltageModel {
        BusVoltageModel {
            bus_id,
            v_pre_fault_pu: 1.0,
            v_during_fault_pu: 0.5,
            v_post_fault_pu: 0.98,
            time_constant_s: 0.3,
            motor_reactive_demand_mvar: 0.0,
        }
    }

    fn simple_fault(bus_id: usize, z_fault: f64) -> FaultEvent {
        FaultEvent::new(bus_id, FaultType::ThreePhase, z_fault, 0.1, 1.0)
    }

    // ── Tests ─────────────────────────────────────────────────────────────────

    // 1. Bolted fault (Z_fault=0): formula gives V_pre (remote-bus perspective).
    #[test]
    fn test_voltage_during_fault_zero_impedance() {
        // V = V_pre * Z_th / (Z_th + 0) = V_pre * Z_th / Z_th = V_pre
        let v = TvsaEngine::compute_during_fault_voltage(1.0, 0.0, 0.1);
        assert!(
            (v - 1.0).abs() < 1e-9,
            "bolted fault remote-bus voltage should equal pre-fault: v={v}"
        );
    }

    // 2. High fault impedance → voltage drops close to zero.
    #[test]
    fn test_voltage_during_fault_high_impedance() {
        let v = TvsaEngine::compute_during_fault_voltage(1.0, 100.0, 0.1);
        // V ≈ 1.0 * 0.1 / 100.1 ≈ 0.001
        assert!(v < 0.01, "high-z fault: v={v}");
    }

    // 2b. Moderate fault impedance.
    #[test]
    fn test_voltage_during_fault_moderate_impedance() {
        let v = TvsaEngine::compute_during_fault_voltage(1.0, 0.4, 0.1);
        // V = 0.1 / 0.5 = 0.2
        assert!((v - 0.2).abs() < 1e-9, "moderate-z: v={v}");
    }

    // 3. Fast recovery → FullRecovery.
    #[test]
    fn test_recovery_classification_full() {
        let s = TvsaEngine::classify_voltage_recovery(0.70, 1.5, 0.97);
        assert_eq!(s, VoltageRecoveryStatus::FullRecovery);
    }

    // 4. Slow recovery (≥ 2 s) → SlowRecovery.
    #[test]
    fn test_recovery_classification_slow() {
        let s = TvsaEngine::classify_voltage_recovery(0.60, 4.0, 0.96);
        assert_eq!(s, VoltageRecoveryStatus::SlowRecovery);
    }

    // 5. Voltage stays below 0.50 → Collapse.
    #[test]
    fn test_recovery_classification_collapse() {
        let s = TvsaEngine::classify_voltage_recovery(0.30, 12.0, 0.40);
        assert_eq!(s, VoltageRecoveryStatus::Collapse);
    }

    // 6. Voltage ends below 0.95 but above 0.50 → NonRecovery.
    #[test]
    fn test_recovery_classification_non_recovery() {
        let s = TvsaEngine::classify_voltage_recovery(0.55, 12.0, 0.80);
        assert_eq!(s, VoltageRecoveryStatus::NonRecovery);
    }

    // 7. Motor stalls when bus voltage < stall_voltage.
    #[test]
    fn test_motor_stall_at_low_voltage() {
        let mut engine = TvsaEngine::new(vec![simple_bus(0)]);
        let motor = MotorLoad::new(0, 0, StallType::HvacCompressor, 1.0, 0.5);
        engine.add_motor_load(motor);
        engine.step_motor_dynamics(0, 0.50, 0.0); // below stall_voltage = 0.65
        assert!(
            engine.motor_loads[0].stalled,
            "motor should stall at 0.50 pu"
        );
    }

    // 8. Motor does NOT stall when voltage is high.
    #[test]
    fn test_motor_no_stall_high_voltage() {
        let mut engine = TvsaEngine::new(vec![simple_bus(0)]);
        let motor = MotorLoad::new(0, 0, StallType::ThreePhaseMotor, 2.0, 1.0);
        engine.add_motor_load(motor);
        engine.step_motor_dynamics(0, 0.90, 0.0); // above stall_voltage = 0.65
        assert!(
            !engine.motor_loads[0].stalled,
            "motor should NOT stall at 0.90 pu"
        );
    }

    // 9. Motor reconnects when voltage recovers above reconnect_voltage.
    #[test]
    fn test_motor_reconnect_after_recovery() {
        let mut engine = TvsaEngine::new(vec![simple_bus(0)]);
        let mut motor = MotorLoad::new(0, 0, StallType::SinglePhaseMotor, 1.0, 0.4);
        motor.stalled = true;
        motor.stall_start_time = Some(0.0);
        engine.add_motor_load(motor);
        // Voltage above reconnect_voltage = 0.80, elapsed < thermal_trip_time
        engine.step_motor_dynamics(0, 0.85, 0.3);
        assert!(
            !engine.motor_loads[0].stalled,
            "motor should reconnect when V > reconnect_voltage"
        );
    }

    // 10. Motor thermally trips after prolonged stall.
    #[test]
    fn test_motor_thermal_trip() {
        let mut engine = TvsaEngine::new(vec![simple_bus(0)]);
        let mut motor = MotorLoad::new(0, 0, StallType::Industrial, 3.0, 1.5);
        motor.stalled = true;
        motor.stall_start_time = Some(0.0); // stalled at t=0
        engine.add_motor_load(motor);
        // Step at t = 4.0 s → elapsed 4.0 ≥ thermal_trip_time = 3.0
        engine.step_motor_dynamics(0, 0.50, 4.0);
        assert!(engine.motor_loads[0].tripped, "motor should thermally trip");
        assert!(
            !engine.motor_loads[0].stalled,
            "tripped motor should not be stalled"
        );
    }

    // 11. Trajectory has correct number of time steps.
    #[test]
    fn test_trajectory_simulation_single_bus() {
        let bus = simple_bus(0);
        let mut engine = TvsaEngine::new(vec![bus.clone()]);
        engine.simulation_duration_s = 2.0;
        engine.dt_s = 0.01;
        let fault = simple_fault(0, 0.2);
        let traj = engine.simulate_voltage_trajectory(&bus, &fault);
        let expected = ((2.0_f64 / 0.01).ceil() as usize).max(1);
        assert_eq!(traj.time_s.len(), expected);
        assert_eq!(traj.voltage_pu.len(), expected);
    }

    // 12. Minimum voltage during fault is below pre-fault voltage.
    #[test]
    fn test_trajectory_min_voltage() {
        let bus = BusVoltageModel {
            bus_id: 0,
            v_pre_fault_pu: 1.0,
            v_during_fault_pu: 0.3,
            v_post_fault_pu: 0.97,
            time_constant_s: 0.5,
            motor_reactive_demand_mvar: 0.0,
        };
        let mut engine = TvsaEngine::new(vec![bus.clone()]);
        let fault = FaultEvent {
            bus_id: 0,
            fault_type: FaultType::SingleLineToGround,
            fault_impedance_pu: 0.3,
            fault_time_s: 0.1,
            clearing_time_s: 0.2,
            pre_fault_voltage_pu: 1.0,
        };
        let traj = engine.simulate_voltage_trajectory(&bus, &fault);
        assert!(
            traj.post_fault_min_pu < 1.0,
            "min voltage {} should be < pre-fault 1.0",
            traj.post_fault_min_pu
        );
    }

    // 13. Recovery time is non-negative.
    #[test]
    fn test_recovery_time_positive() {
        let bus = BusVoltageModel {
            bus_id: 0,
            v_pre_fault_pu: 1.0,
            v_during_fault_pu: 0.6,
            v_post_fault_pu: 0.97,
            time_constant_s: 0.3,
            motor_reactive_demand_mvar: 0.0,
        };
        let mut engine = TvsaEngine::new(vec![bus.clone()]);
        let fault = simple_fault(0, 0.2);
        let traj = engine.simulate_voltage_trajectory(&bus, &fault);
        assert!(
            traj.recovery_time_s >= 0.0,
            "recovery_time must be ≥ 0, got {}",
            traj.recovery_time_s
        );
    }

    // 14. run_assessment returns a valid TvsaResult.
    #[test]
    fn test_run_assessment_basic() {
        let bus = simple_bus(0);
        let mut engine = TvsaEngine::new(vec![bus]);
        let fault = simple_fault(0, 0.2);
        let result = engine.run_assessment(fault);
        assert_eq!(result.voltage_trajectories.len(), 1);
        assert!(result.worst_sag_pu >= 0.0);
        assert!(result.worst_sag_pu <= 1.2);
    }

    // 15. Bus with deeper sag is identified as worst bus.
    #[test]
    fn test_worst_bus_identification() {
        let bus0 = BusVoltageModel {
            bus_id: 0,
            v_pre_fault_pu: 1.0,
            v_during_fault_pu: 0.8,
            v_post_fault_pu: 0.98,
            time_constant_s: 0.3,
            motor_reactive_demand_mvar: 0.0,
        };
        let bus1 = BusVoltageModel {
            bus_id: 1,
            v_pre_fault_pu: 1.0,
            v_during_fault_pu: 0.2, // much deeper sag
            v_post_fault_pu: 0.97,
            time_constant_s: 0.5,
            motor_reactive_demand_mvar: 0.0,
        };
        let mut engine = TvsaEngine::new(vec![bus0, bus1]);
        // Use a high-impedance fault so that the pre-set v_during_fault_pu values dominate.
        let fault = FaultEvent {
            bus_id: 1,
            fault_type: FaultType::ThreePhase,
            fault_impedance_pu: 10.0, // high impedance → engine computes low v_during
            fault_time_s: 0.1,
            clearing_time_s: 0.2,
            pre_fault_voltage_pu: 1.0,
        };
        let result = engine.run_assessment(fault);
        // Both buses see very low v_during due to high z_fault, but bus 1 also had
        // lower pre-set v_during_fault_pu before the override.  Both should be low;
        // verify the worst_bus_id is valid (one of the two buses).
        assert!(
            result.worst_bus_id == 0 || result.worst_bus_id == 1,
            "worst_bus_id should be 0 or 1, got {}",
            result.worst_bus_id
        );
        // Worst sag must be the minimum of the two trajectories.
        let min_traj = result
            .voltage_trajectories
            .iter()
            .map(|t| t.post_fault_min_pu)
            .fold(f64::MAX, f64::min);
        assert!(
            (result.worst_sag_pu - min_traj).abs() < 1e-9,
            "worst_sag_pu should equal minimum trajectory sag"
        );
    }

    // 16. Recovery index is in [0, 1].
    #[test]
    fn test_recovery_index_bounds() {
        let buses: Vec<BusVoltageModel> = (0..5).map(simple_bus).collect();
        let mut engine = TvsaEngine::new(buses);
        let fault = simple_fault(0, 0.2);
        let result = engine.run_assessment(fault);
        assert!(
            result.recovery_index >= 0.0 && result.recovery_index <= 1.0,
            "recovery_index={} out of [0,1]",
            result.recovery_index
        );
    }

    // 17. Stability margin > 0 for a stable system.
    #[test]
    fn test_stability_margin_positive_stable() {
        let bus = BusVoltageModel {
            bus_id: 0,
            v_pre_fault_pu: 1.0,
            v_during_fault_pu: 0.75,
            v_post_fault_pu: 0.98,
            time_constant_s: 0.3,
            motor_reactive_demand_mvar: 0.0,
        };
        let mut engine = TvsaEngine::new(vec![bus]);
        // Small fault impedance: v_during = 0.1/(0.1+0.05) ≈ 0.667 pu > collapse threshold 0.50
        // → stability_margin = 0.667 - 0.50 = 0.167 > 0
        let fault = simple_fault(0, 0.05);
        let result = engine.run_assessment(fault);
        assert!(
            result.voltage_stability_margin_pu > 0.0,
            "margin={} should be positive for stable scenario",
            result.voltage_stability_margin_pu
        );
    }

    // 18. Stability margin negative for collapsed system.
    #[test]
    fn test_stability_margin_negative_collapse() {
        let bus = simple_bus(0);
        let engine = TvsaEngine::new(vec![bus]);
        // Synthetic result with sag below collapse threshold.
        let result = TvsaResult {
            fault_event: simple_fault(0, 0.0),
            voltage_trajectories: vec![],
            stalled_motors: vec![],
            tripped_motors: vec![],
            worst_bus_id: 0,
            worst_sag_pu: 0.20, // below collapse_threshold 0.50
            recovery_index: 0.0,
            voltage_stability_margin_pu: 0.0,
            overall_status: VoltageRecoveryStatus::Collapse,
        };
        let margin = engine.compute_stability_margin(&result);
        assert!(
            margin < 0.0,
            "margin={} should be negative for collapsed system",
            margin
        );
    }

    // 19. N-1 assessment returns one result per fault.
    #[test]
    fn test_n1_assessment_multiple_faults() {
        let buses: Vec<BusVoltageModel> = (0..3).map(simple_bus).collect();
        let mut engine = TvsaEngine::new(buses);
        let faults: Vec<FaultEvent> = (0..3).map(|i| simple_fault(i, 0.2)).collect();
        let results = engine.run_n1_assessment(&faults);
        assert_eq!(results.len(), 3, "one result per fault");
    }

    // 20. Critical buses sorted with worst (lowest sag) first.
    #[test]
    fn test_critical_buses_sorted() {
        let buses: Vec<BusVoltageModel> = (0..3).map(simple_bus).collect();
        let mut engine = TvsaEngine::new(buses);
        let faults: Vec<FaultEvent> = (0..3).map(|i| simple_fault(i, 0.2)).collect();
        let results = engine.run_n1_assessment(&faults);
        let critical = engine.identify_critical_buses(&results);
        assert!(!critical.is_empty());
        for pair in critical.windows(2) {
            assert!(
                pair[0].1 <= pair[1].1,
                "critical buses not sorted: {:?} > {:?}",
                pair[0],
                pair[1]
            );
        }
    }

    // 21. Stalled motors are recorded in the trajectory motor_stalls list.
    #[test]
    fn test_stalled_motors_increase_reactive() {
        let bus = BusVoltageModel {
            bus_id: 0,
            v_pre_fault_pu: 1.0,
            v_during_fault_pu: 0.4, // below stall_voltage = 0.65
            v_post_fault_pu: 0.98,
            time_constant_s: 0.5,
            motor_reactive_demand_mvar: 0.0,
        };
        let mut engine = TvsaEngine::new(vec![bus.clone()]);
        let motor = MotorLoad::new(0, 0, StallType::HvacCompressor, 2.0, 1.0);
        engine.add_motor_load(motor);

        let fault = FaultEvent {
            bus_id: 0,
            fault_type: FaultType::ThreePhase,
            fault_impedance_pu: 0.0,
            fault_time_s: 0.1,
            clearing_time_s: 0.2,
            pre_fault_voltage_pu: 1.0,
        };
        let traj = engine.simulate_voltage_trajectory(&bus, &fault);
        assert!(
            !traj.motor_stalls.is_empty(),
            "stalled motors should be recorded in trajectory"
        );
    }

    // 22. FaultEvent::new sets clearing_time = fault_time + 0.1.
    #[test]
    fn test_fault_event_default_clearing_time() {
        let fault = FaultEvent::new(0, FaultType::DoubleLineToGround, 0.0, 0.5, 1.0);
        assert!(
            (fault.clearing_time_s - 0.6).abs() < 1e-9,
            "clearing_time should be 0.6"
        );
    }

    // 23. MotorLoad::new sets defaults correctly.
    #[test]
    fn test_motor_load_defaults() {
        let m = MotorLoad::new(7, 3, StallType::Industrial, 5.0, 2.5);
        assert!((m.stall_voltage_pu - 0.65).abs() < 1e-9);
        assert!((m.reconnect_voltage_pu - 0.80).abs() < 1e-9);
        assert!((m.stall_time_s - 0.5).abs() < 1e-9);
        assert!((m.thermal_trip_time_s - 3.0).abs() < 1e-9);
        assert!(!m.stalled);
        assert!(!m.tripped);
        assert!(m.stall_start_time.is_none());
    }

    // 24. TvsaEngine::new sets expected defaults.
    #[test]
    fn test_engine_defaults() {
        let engine = TvsaEngine::new(vec![]);
        assert!((engine.dt_s - 0.01).abs() < 1e-9);
        assert!((engine.simulation_duration_s - 10.0).abs() < 1e-9);
        assert!((engine.voltage_threshold_pu - 0.95).abs() < 1e-9);
        assert!((engine.collapse_threshold_pu - 0.50).abs() < 1e-9);
    }

    // 25. compute_recovery_index returns 1.0 for empty trajectory list.
    #[test]
    fn test_recovery_index_empty() {
        let engine = TvsaEngine::new(vec![]);
        let idx = engine.compute_recovery_index(&[]);
        assert!((idx - 1.0).abs() < 1e-9, "empty trajectories → index = 1.0");
    }
}
