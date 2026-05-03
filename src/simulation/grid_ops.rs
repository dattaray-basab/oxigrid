//! Grid Operations Simulator.
//!
//! This module provides two simulation paradigms for grid operations:
//!
//! 1. **Legacy discrete-event operator-focused simulator** (`GridOpsSimulator`) —
//!    models operator skill, automation level, contingency rates, and workload
//!    metrics over multi-hour/year horizons.
//!
//! 2. **Event-driven quasi-dynamic simulator** (`GridOperationsSimulator`) —
//!    physics-based, timestep-accurate simulation with swing-equation frequency
//!    dynamics, AGC, UFLS, DC branch flows, and storage SoC tracking.
//!
//! # Quick Start (legacy)
//!
//! ```rust
//! use oxigrid::simulation::grid_ops::{GridOpsConfig, GridOpsSimulator};
//!
//! let config = GridOpsConfig {
//!     simulation_hours: 24,
//!     dt_minutes: 5.0,
//!     operator_skill: 0.85,
//!     automation_level: 0.7,
//!     contingency_probability: 0.02,
//!     weather_events: false,
//! };
//! let sim = GridOpsSimulator::new(config, 1000.0, 800.0);
//! let result = sim.simulate().expect("simulation failed");
//! assert!(result.system_reliability_pct > 90.0);
//! ```

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::error::OxiGridError;

// ─── LCG RNG ─────────────────────────────────────────────────────────────────

const LCG_MULT: u64 = 6_364_136_223_846_793_005;
const LCG_ADD: u64 = 1_442_695_040_888_963_407;

fn lcg_next(state: &mut u64) -> f64 {
    *state = state.wrapping_mul(LCG_MULT).wrapping_add(LCG_ADD);
    (*state >> 32) as f64 / u32::MAX as f64
}

// ═══════════════════════════════════════════════════════════════════════════
// LEGACY SIMULATOR
// ═══════════════════════════════════════════════════════════════════════════

/// Errors produced by the legacy grid operations simulator.
#[derive(Debug, Error)]
pub enum GridOpsError {
    #[error("simulation_hours must be > 0")]
    ZeroSimulationHours,
    #[error("gen_capacity_mw must be positive, got {0}")]
    InvalidGenCapacity(f64),
    #[error("peak_load_mw must be positive, got {0}")]
    InvalidPeakLoad(f64),
    #[error("dt_minutes must be positive, got {0}")]
    InvalidTimeStep(f64),
}

/// Configuration for the legacy grid operations simulator.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GridOpsConfig {
    pub simulation_hours: usize,
    pub dt_minutes: f64,
    pub operator_skill: f64,
    pub automation_level: f64,
    pub contingency_probability: f64,
    pub weather_events: bool,
}

impl Default for GridOpsConfig {
    fn default() -> Self {
        Self {
            simulation_hours: 8760,
            dt_minutes: 60.0,
            operator_skill: 0.8,
            automation_level: 0.5,
            contingency_probability: 0.05,
            weather_events: true,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum OperatorActionType {
    GeneratorDispatch,
    LoadShedding,
    TransformerTapChange,
    CapacitorSwitching,
    BreakerOperation,
    EmergencyShutdown,
    RestoreService,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ActionOutcome {
    Success,
    PartialSuccess { achieved_pct: f64 },
    Failed { reason: String },
    Delayed { delay_min: f64 },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum EventType {
    GeneratorTrip,
    LineTrip,
    UnderFrequency,
    OverVoltage,
    ShortCircuit,
    WeatherEvent,
    LoadSurge,
    CybersecurityAlert,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OperatorAction {
    pub time_h: f64,
    pub action_type: OperatorActionType,
    pub target: String,
    pub value: f64,
    pub reason: String,
    pub outcome: ActionOutcome,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OperationalEvent {
    pub time_h: f64,
    pub event_type: EventType,
    pub severity: f64,
    pub duration_h: f64,
    pub affected_element: String,
    pub automatic_response: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GridOpsResult {
    pub total_hours: usize,
    pub n_events: usize,
    pub n_operator_actions: usize,
    pub n_successful_actions: usize,
    pub load_shed_mwh: f64,
    pub unserved_energy_mwh: f64,
    pub frequency_excursion_hours: f64,
    pub voltage_violation_hours: f64,
    pub operator_workload_score: f64,
    pub system_reliability_pct: f64,
    pub event_log: Vec<OperationalEvent>,
    pub action_log: Vec<OperatorAction>,
}

pub struct GridOpsSimulator {
    config: GridOpsConfig,
    gen_capacity_mw: f64,
    peak_load_mw: f64,
    #[allow(dead_code)]
    reserve_margin_pct: f64,
}

impl GridOpsSimulator {
    pub fn new(config: GridOpsConfig, gen_mw: f64, peak_load_mw: f64) -> Self {
        let reserve_margin_pct = if peak_load_mw > 0.0 {
            (gen_mw - peak_load_mw) / peak_load_mw * 100.0
        } else {
            0.0
        };
        Self {
            config,
            gen_capacity_mw: gen_mw,
            peak_load_mw,
            reserve_margin_pct,
        }
    }

    pub fn simulate(&self) -> Result<GridOpsResult, GridOpsError> {
        self.validate()?;
        let hours = self.config.simulation_hours;
        let dt_h = self.config.dt_minutes / 60.0;
        let mut rng: u64 = 0xDEAD_BEEF_CAFE_F00D;
        let load_profile = self.generate_load_profile(hours, &mut rng);
        let mut event_log: Vec<OperationalEvent> = Vec::new();
        let mut action_log: Vec<OperatorAction> = Vec::new();
        let mut load_shed_mwh = 0.0f64;
        let mut unserved_energy_mwh = 0.0f64;
        let mut freq_excursion_hours = 0.0f64;
        let mut voltage_violation_hours = 0.0f64;
        let mut n_successful_actions = 0usize;
        let mut available_capacity_mw = self.gen_capacity_mw;

        for (h, &load_mw) in load_profile.iter().enumerate() {
            let reserve_mw = (available_capacity_mw - load_mw).max(0.0);
            let p_contingency = self.config.contingency_probability;
            if lcg_next(&mut rng) < p_contingency {
                let severity = lcg_next(&mut rng) * 0.8 + 0.1;
                let is_gen_trip = lcg_next(&mut rng) < 0.5;
                let event_type = if is_gen_trip {
                    EventType::GeneratorTrip
                } else {
                    EventType::LineTrip
                };
                let affected = if is_gen_trip {
                    format!("GEN_{}", (lcg_next(&mut rng) * 10.0) as usize + 1)
                } else {
                    format!("LINE_{}", (lcg_next(&mut rng) * 20.0) as usize + 1)
                };
                let duration_h = severity * 4.0 + 0.25;
                if is_gen_trip {
                    let lost_mw = severity * self.gen_capacity_mw * 0.15;
                    available_capacity_mw = (available_capacity_mw - lost_mw).max(0.0);
                }
                let event = OperationalEvent {
                    time_h: h as f64,
                    event_type,
                    severity,
                    duration_h,
                    affected_element: affected,
                    automatic_response: self.config.automation_level > lcg_next(&mut rng),
                };
                let action = self.operator_response(&event, reserve_mw, &mut rng);
                let success = matches!(
                    action.outcome,
                    ActionOutcome::Success | ActionOutcome::PartialSuccess { .. }
                );
                if success {
                    n_successful_actions += 1;
                }
                freq_excursion_hours += severity * duration_h * 0.3;
                voltage_violation_hours += severity * duration_h * 0.2;
                action_log.push(action);
                event_log.push(event);
            }

            if self.config.weather_events {
                let in_storm = h < 720;
                let in_heat = (4320..5040).contains(&h);
                let p_weather = if in_storm || in_heat { 0.02 } else { 0.002 };
                if lcg_next(&mut rng) < p_weather {
                    let severity = lcg_next(&mut rng) * 0.6 + 0.2;
                    let event = OperationalEvent {
                        time_h: h as f64,
                        event_type: EventType::WeatherEvent,
                        severity,
                        duration_h: lcg_next(&mut rng) * 8.0 + 1.0,
                        affected_element: if in_storm {
                            "WINTER_STORM_AREA".to_string()
                        } else {
                            "HEAT_WAVE_AREA".to_string()
                        },
                        automatic_response: false,
                    };
                    let action = self.operator_response(&event, reserve_mw, &mut rng);
                    let success = matches!(
                        action.outcome,
                        ActionOutcome::Success | ActionOutcome::PartialSuccess { .. }
                    );
                    if success {
                        n_successful_actions += 1;
                    }
                    action_log.push(action);
                    event_log.push(event);
                }
            }

            let load_ratio = if self.gen_capacity_mw > 0.0 {
                load_mw / self.gen_capacity_mw
            } else {
                1.0
            };
            if load_ratio > 0.95 {
                freq_excursion_hours += dt_h;
                event_log.push(OperationalEvent {
                    time_h: h as f64,
                    event_type: EventType::UnderFrequency,
                    severity: load_ratio - 0.9,
                    duration_h: dt_h,
                    affected_element: "SYSTEM_WIDE".to_string(),
                    automatic_response: self.config.automation_level > 0.5,
                });
            }
            if load_ratio < 0.30 && self.peak_load_mw > 0.0 {
                voltage_violation_hours += dt_h;
            }
            if available_capacity_mw < load_mw {
                let shortfall_mw = load_mw - available_capacity_mw;
                let shed_mw = shortfall_mw.min(load_mw * 0.2);
                load_shed_mwh += shed_mw * dt_h;
                unserved_energy_mwh += shortfall_mw * dt_h;
                if lcg_next(&mut rng) > self.config.automation_level {
                    action_log.push(OperatorAction {
                        time_h: h as f64,
                        action_type: OperatorActionType::LoadShedding,
                        target: "INTERRUPTIBLE_LOAD".to_string(),
                        value: shed_mw,
                        reason: "Capacity deficit — emergency load shedding".to_string(),
                        outcome: ActionOutcome::Success,
                    });
                    n_successful_actions += 1;
                }
            }
            let repair_rate = self.gen_capacity_mw * 0.01 * dt_h;
            available_capacity_mw = (available_capacity_mw + repair_rate).min(self.gen_capacity_mw);
        }

        let n_events = event_log.len();
        let n_actions = action_log.len();
        let manual_actions = action_log
            .iter()
            .filter(|a| !matches!(a.outcome, ActionOutcome::Delayed { .. }))
            .count();
        let operator_workload_score = {
            let raw =
                manual_actions as f64 / hours.max(1) as f64 * (1.0 - self.config.automation_level);
            raw.clamp(0.0, 1.0)
        };
        let unserved_hours = unserved_energy_mwh / self.peak_load_mw.max(1.0);
        let system_reliability_pct =
            ((hours as f64 - unserved_hours) / hours.max(1) as f64 * 100.0).clamp(0.0, 100.0);

        Ok(GridOpsResult {
            total_hours: hours,
            n_events,
            n_operator_actions: n_actions,
            n_successful_actions,
            load_shed_mwh,
            unserved_energy_mwh,
            frequency_excursion_hours: freq_excursion_hours,
            voltage_violation_hours,
            operator_workload_score,
            system_reliability_pct,
            event_log,
            action_log,
        })
    }

    pub fn generate_load_profile(&self, hours: usize, rng: &mut u64) -> Vec<f64> {
        let mut profile = Vec::with_capacity(hours);
        for h in 0..hours {
            let hour_of_day = (h % 24) as f64;
            let day_of_year = (h / 24) as f64;
            let diurnal = 0.15 * (std::f64::consts::PI * (hour_of_day - 4.0) / 12.0).sin();
            let seasonal = 0.10 * (2.0 * std::f64::consts::PI * day_of_year / 365.0).cos();
            let base = 0.72 + diurnal + seasonal;
            let noise = (lcg_next(rng) - 0.5) * 0.06;
            let p = ((base + noise) * self.peak_load_mw).clamp(0.0, self.gen_capacity_mw);
            profile.push(p);
        }
        profile
    }

    pub fn operator_response(
        &self,
        event: &OperationalEvent,
        reserve_mw: f64,
        rng: &mut u64,
    ) -> OperatorAction {
        let skill = self.config.operator_skill.clamp(0.0, 1.0);
        let auto = self.config.automation_level.clamp(0.0, 1.0);
        let combined_effectiveness = skill * (1.0 - auto * 0.3) + auto * 0.5;
        let roll = lcg_next(rng);
        let outcome = if roll < combined_effectiveness * 0.85 {
            ActionOutcome::Success
        } else if roll < combined_effectiveness * 0.97 {
            ActionOutcome::PartialSuccess {
                achieved_pct: 50.0 + roll * 40.0,
            }
        } else if roll < 0.99 {
            ActionOutcome::Delayed {
                delay_min: (1.0 - combined_effectiveness) * 30.0,
            }
        } else {
            ActionOutcome::Failed {
                reason: "Insufficient reserve or operator unavailable".to_string(),
            }
        };
        let (action_type, target, value, reason) = match &event.event_type {
            EventType::GeneratorTrip => (
                OperatorActionType::GeneratorDispatch,
                event.affected_element.clone(),
                reserve_mw.min(event.severity * self.peak_load_mw * 0.1),
                "Re-dispatch reserves after generator trip".to_string(),
            ),
            EventType::LineTrip => (
                OperatorActionType::BreakerOperation,
                event.affected_element.clone(),
                1.0,
                "Isolate tripped line and close bypass".to_string(),
            ),
            EventType::UnderFrequency => (
                OperatorActionType::LoadShedding,
                "INTERRUPTIBLE_LOAD".to_string(),
                event.severity * self.peak_load_mw * 0.05,
                "Under-frequency load shedding".to_string(),
            ),
            EventType::OverVoltage => (
                OperatorActionType::CapacitorSwitching,
                "CAP_BANK_HV".to_string(),
                0.0,
                "Switch out capacitor to reduce overvoltage".to_string(),
            ),
            EventType::ShortCircuit => (
                OperatorActionType::EmergencyShutdown,
                event.affected_element.clone(),
                0.0,
                "Emergency isolation of faulted element".to_string(),
            ),
            EventType::WeatherEvent => (
                OperatorActionType::TransformerTapChange,
                "TX_MAIN".to_string(),
                event.severity * 2.0,
                "Adjust transformer taps during weather event".to_string(),
            ),
            EventType::LoadSurge => (
                OperatorActionType::GeneratorDispatch,
                "PEAKER_GEN".to_string(),
                event.severity * self.peak_load_mw * 0.08,
                "Commit peaking units for load surge".to_string(),
            ),
            EventType::CybersecurityAlert => (
                OperatorActionType::BreakerOperation,
                "SCADA_GATEWAY".to_string(),
                0.0,
                "Isolate compromised SCADA node".to_string(),
            ),
        };
        OperatorAction {
            time_h: event.time_h,
            action_type,
            target,
            value,
            reason,
            outcome,
        }
    }

    fn validate(&self) -> Result<(), GridOpsError> {
        if self.config.simulation_hours == 0 {
            return Err(GridOpsError::ZeroSimulationHours);
        }
        if self.gen_capacity_mw <= 0.0 {
            return Err(GridOpsError::InvalidGenCapacity(self.gen_capacity_mw));
        }
        if self.peak_load_mw <= 0.0 {
            return Err(GridOpsError::InvalidPeakLoad(self.peak_load_mw));
        }
        if self.config.dt_minutes <= 0.0 {
            return Err(GridOpsError::InvalidTimeStep(self.config.dt_minutes));
        }
        Ok(())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// QUASI-DYNAMIC EVENT-DRIVEN SIMULATOR
// ═══════════════════════════════════════════════════════════════════════════

// ─── Simulation Clock ─────────────────────────────────────────────────────

/// Discrete-event simulation clock.
#[derive(Debug, Clone)]
pub struct SimClock {
    pub current_time_s: f64,
    pub dt_s: f64,
    pub end_time_s: f64,
}

impl SimClock {
    pub fn new(start_s: f64, end_s: f64, dt_s: f64) -> Self {
        Self {
            current_time_s: start_s,
            dt_s,
            end_time_s: end_s,
        }
    }

    /// Advance the clock by one timestep. Returns `false` when simulation is complete.
    pub fn advance(&mut self) -> bool {
        self.current_time_s += self.dt_s;
        self.current_time_s <= self.end_time_s
    }

    /// Hour of day (0–24) corresponding to `current_time_s`.
    pub fn time_of_day_h(&self) -> f64 {
        (self.current_time_s / 3600.0) % 24.0
    }
}

// ─── Grid Events ──────────────────────────────────────────────────────────

/// All events that can occur during simulation.
#[derive(Debug, Clone)]
pub enum GridEvent {
    GeneratorTrip {
        bus: usize,
        capacity_mw: f64,
        reason: String,
    },
    LineTrip {
        branch_id: usize,
        reason: String,
    },
    LoadIncrease {
        bus: usize,
        delta_mw: f64,
    },
    LoadDecrease {
        bus: usize,
        delta_mw: f64,
    },
    GeneratorReconnect {
        bus: usize,
        capacity_mw: f64,
    },
    LineReconnect {
        branch_id: usize,
    },
    StorageCharge {
        bus: usize,
        rate_mw: f64,
    },
    StorageDischarge {
        bus: usize,
        rate_mw: f64,
    },
    AutomaticGenControl {
        area_mw: f64,
    },
    UnderFrequencyLoadShedding {
        buses: Vec<usize>,
        shed_mw: f64,
    },
    VoltageLimitViolation {
        bus: usize,
        voltage_pu: f64,
    },
    OverloadAlarm {
        branch_id: usize,
        loading_pct: f64,
    },
}

/// A grid event scheduled to occur at a specific simulation time.
pub struct ScheduledEvent {
    pub time_s: f64,
    pub event: GridEvent,
    pub description: String,
}

// ─── Component States ─────────────────────────────────────────────────────

/// Generator operating state.
#[derive(Debug, Clone)]
pub struct SimGenerator {
    pub id: usize,
    pub bus: usize,
    pub p_mw: f64,
    pub p_max_mw: f64,
    pub p_min_mw: f64,
    pub ramp_rate_mw_per_min: f64,
    pub agc_participation: f64,
    pub is_online: bool,
    pub startup_time_min: f64,
    pub fuel_type: String,
    pub co2_kg_per_mwh: f64,
}

/// Load bus state.
#[derive(Debug, Clone)]
pub struct SimLoad {
    pub bus: usize,
    pub p_mw: f64,
    pub q_mvar: f64,
    pub is_shedable: bool,
    pub priority: usize,
}

/// Network branch state.
#[derive(Debug, Clone)]
pub struct SimBranch {
    pub id: usize,
    pub from: usize,
    pub to: usize,
    pub is_online: bool,
    pub rating_mva: f64,
    pub current_flow_mw: f64,
    pub current_flow_mvar: f64,
    pub loading_pct: f64,
}

/// Battery storage unit state.
#[derive(Debug, Clone)]
pub struct SimStorage {
    pub bus: usize,
    pub soc: f64,
    pub capacity_mwh: f64,
    pub power_mw: f64,
    pub max_charge_mw: f64,
    pub max_discharge_mw: f64,
    pub efficiency: f64,
}

// ─── System Snapshot ──────────────────────────────────────────────────────

/// Complete power system state at one timestep.
pub struct SystemSnapshot {
    pub time_s: f64,
    pub generators: Vec<SimGenerator>,
    pub loads: Vec<SimLoad>,
    pub branches: Vec<SimBranch>,
    pub storages: Vec<SimStorage>,
    pub frequency_hz: f64,
    pub total_generation_mw: f64,
    pub total_load_mw: f64,
    pub total_losses_mw: f64,
    pub power_imbalance_mw: f64,
    pub events_this_step: Vec<String>,
    pub violations: Vec<String>,
    pub n_generators_online: usize,
}

// ─── Configuration ────────────────────────────────────────────────────────

/// Configuration for the quasi-dynamic grid operations simulator.
#[derive(Debug, Clone)]
pub struct QdGridOpsConfig {
    pub n_buses: usize,
    pub base_mva: f64,
    pub nominal_frequency_hz: f64,
    pub frequency_deadband_hz: f64,
    pub ufls_threshold_hz: f64,
    pub ufls_shed_pct: Vec<f64>,
    pub ovf_threshold_hz: f64,
    pub voltage_min_pu: f64,
    pub voltage_max_pu: f64,
    pub max_branch_loading_pct: f64,
}

impl Default for QdGridOpsConfig {
    fn default() -> Self {
        Self {
            n_buses: 10,
            base_mva: 100.0,
            nominal_frequency_hz: 50.0,
            frequency_deadband_hz: 0.02,
            ufls_threshold_hz: 47.5,
            ufls_shed_pct: vec![0.10, 0.15, 0.20],
            ovf_threshold_hz: 51.5,
            voltage_min_pu: 0.95,
            voltage_max_pu: 1.05,
            max_branch_loading_pct: 90.0,
        }
    }
}

// ─── Results ──────────────────────────────────────────────────────────────

/// Simulation result from the quasi-dynamic simulator.
pub struct QdGridOpsResult {
    pub snapshots: Vec<SystemSnapshot>,
    pub events_log: Vec<(f64, String)>,
    pub frequency_history: Vec<(f64, f64)>,
    pub statistics: GridOpsStatistics,
}

/// Aggregated statistics for a quasi-dynamic simulation run.
#[derive(Debug, Clone)]
pub struct GridOpsStatistics {
    pub duration_s: f64,
    pub total_energy_mwh: f64,
    pub renewable_energy_mwh: f64,
    pub load_served_pct: f64,
    pub shed_energy_mwh: f64,
    pub total_co2_ton: f64,
    pub min_frequency_hz: f64,
    pub max_frequency_hz: f64,
    pub n_frequency_violations: usize,
    pub n_voltage_violations: usize,
    pub n_line_trips: usize,
    pub n_generator_trips: usize,
    pub n_load_shed_events: usize,
    pub system_resilience_index: f64,
}

// ─── Main Simulator ───────────────────────────────────────────────────────

/// Event-driven quasi-dynamic grid operations simulator.
pub struct GridOperationsSimulator {
    pub config: QdGridOpsConfig,
    pub generators: Vec<SimGenerator>,
    pub loads: Vec<SimLoad>,
    pub branches: Vec<SimBranch>,
    pub storages: Vec<SimStorage>,
    pub scheduled_events: Vec<ScheduledEvent>,
    pub clock: SimClock,
    pub branch_susceptances: Vec<f64>,
}

impl GridOperationsSimulator {
    pub fn new(
        config: QdGridOpsConfig,
        generators: Vec<SimGenerator>,
        loads: Vec<SimLoad>,
        branches: Vec<SimBranch>,
        storages: Vec<SimStorage>,
        duration_s: f64,
        dt_s: f64,
    ) -> Self {
        // Build branch susceptances from branch ratings (proxy: 1/rating as susceptance)
        let branch_susceptances: Vec<f64> = branches
            .iter()
            .map(|b| {
                if b.rating_mva > 0.0 {
                    1.0 / b.rating_mva
                } else {
                    1.0
                }
            })
            .collect();
        Self {
            config,
            generators,
            loads,
            branches,
            storages,
            scheduled_events: Vec::new(),
            clock: SimClock::new(0.0, duration_s, dt_s),
            branch_susceptances,
        }
    }

    /// Schedule a grid event to occur at `time_s`.
    pub fn schedule_event(&mut self, time_s: f64, event: GridEvent, description: String) {
        self.scheduled_events.push(ScheduledEvent {
            time_s,
            event,
            description,
        });
        // Keep events sorted by time for efficient processing
        self.scheduled_events.sort_by(|a, b| {
            a.time_s
                .partial_cmp(&b.time_s)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
    }

    /// Run the quasi-dynamic simulation loop.
    pub fn run(&mut self) -> Result<QdGridOpsResult, OxiGridError> {
        if self.clock.end_time_s <= 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "end_time_s must be positive".to_string(),
            ));
        }
        if self.clock.dt_s <= 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "dt_s must be positive".to_string(),
            ));
        }

        let mut snapshots: Vec<SystemSnapshot> = Vec::new();
        let mut events_log: Vec<(f64, String)> = Vec::new();
        let mut frequency_history: Vec<(f64, f64)> = Vec::new();

        let nominal = self.config.nominal_frequency_hz;
        let mut freq_hz = nominal;
        let dt_s = self.clock.dt_s;

        // Main simulation loop
        loop {
            let t = self.clock.current_time_s;
            let mut step_events: Vec<String> = Vec::new();

            // 1. Process scheduled events at or before current time
            let mut i = 0;
            while i < self.scheduled_events.len() {
                if self.scheduled_events[i].time_s <= t {
                    let se = self.scheduled_events.remove(i);
                    let desc = self.process_event(&se.event);
                    step_events.push(desc.clone());
                    events_log.push((t, desc));
                } else {
                    i += 1;
                }
            }

            // 2. Compute power balance
            let delta_p = self.compute_power_balance();

            // 3. Update frequency via swing equation
            freq_hz = self.update_frequency(delta_p, freq_hz, dt_s);

            // 4. Apply AGC if outside deadband
            let delta_f = freq_hz - nominal;
            if delta_f.abs() > self.config.frequency_deadband_hz {
                self.apply_agc(delta_f, dt_s);
                // Recompute frequency after AGC correction
                let delta_p2 = self.compute_power_balance();
                freq_hz = self.update_frequency(delta_p2, freq_hz, dt_s);
            }

            // 5. Apply UFLS if frequency below threshold
            if freq_hz < self.config.ufls_threshold_hz {
                let shed = self.apply_ufls(freq_hz);
                if shed > 0.0 {
                    let msg = format!("UFLS shed {shed:.2} MW at f={freq_hz:.3} Hz");
                    step_events.push(msg.clone());
                    events_log.push((t, msg));
                }
            }

            // 6. Compute branch flows (DC approximation)
            self.compute_branch_flows();

            // 7. Simplified voltages (flat profile: 1.0 per bus)
            let voltages: Vec<f64> = vec![1.0; self.config.n_buses];

            // 8. Check violations
            let violations = self.check_violations(freq_hz, &voltages);
            for v in &violations {
                events_log.push((t, v.clone()));
            }

            // 9. Update storage SoC
            self.update_storage_soc(dt_s);

            // 10. Record snapshot
            let snap = self.take_snapshot(freq_hz, step_events, violations);
            frequency_history.push((t, freq_hz));
            snapshots.push(snap);

            // Advance clock; stop if simulation is complete
            if !self.clock.advance() {
                break;
            }
        }

        let statistics = Self::compute_statistics(&snapshots, dt_s);

        Ok(QdGridOpsResult {
            snapshots,
            events_log,
            frequency_history,
            statistics,
        })
    }

    /// Process a single grid event, mutating system state. Returns a human-readable description.
    fn process_event(&mut self, event: &GridEvent) -> String {
        match event {
            GridEvent::GeneratorTrip {
                bus,
                capacity_mw,
                reason,
            } => {
                for gen in &mut self.generators {
                    if gen.bus == *bus {
                        gen.is_online = false;
                        gen.p_mw = 0.0;
                    }
                }
                format!("GeneratorTrip: bus={bus} cap={capacity_mw:.1} MW reason={reason}")
            }
            GridEvent::LineTrip { branch_id, reason } => {
                for br in &mut self.branches {
                    if br.id == *branch_id {
                        br.is_online = false;
                        br.current_flow_mw = 0.0;
                        br.loading_pct = 0.0;
                    }
                }
                format!("LineTrip: branch={branch_id} reason={reason}")
            }
            GridEvent::LoadIncrease { bus, delta_mw } => {
                for load in &mut self.loads {
                    if load.bus == *bus {
                        load.p_mw += delta_mw;
                    }
                }
                format!("LoadIncrease: bus={bus} delta={delta_mw:.2} MW")
            }
            GridEvent::LoadDecrease { bus, delta_mw } => {
                for load in &mut self.loads {
                    if load.bus == *bus {
                        load.p_mw = (load.p_mw - delta_mw).max(0.0);
                    }
                }
                format!("LoadDecrease: bus={bus} delta={delta_mw:.2} MW")
            }
            GridEvent::GeneratorReconnect { bus, capacity_mw } => {
                for gen in &mut self.generators {
                    if gen.bus == *bus {
                        gen.is_online = true;
                        gen.p_max_mw = *capacity_mw;
                        gen.p_mw = gen.p_min_mw;
                    }
                }
                format!("GeneratorReconnect: bus={bus} cap={capacity_mw:.1} MW")
            }
            GridEvent::LineReconnect { branch_id } => {
                for br in &mut self.branches {
                    if br.id == *branch_id {
                        br.is_online = true;
                    }
                }
                format!("LineReconnect: branch={branch_id}")
            }
            GridEvent::StorageCharge { bus, rate_mw } => {
                for st in &mut self.storages {
                    if st.bus == *bus {
                        st.power_mw = -rate_mw;
                    }
                }
                format!("StorageCharge: bus={bus} rate={rate_mw:.2} MW")
            }
            GridEvent::StorageDischarge { bus, rate_mw } => {
                for st in &mut self.storages {
                    if st.bus == *bus {
                        st.power_mw = *rate_mw;
                    }
                }
                format!("StorageDischarge: bus={bus} rate={rate_mw:.2} MW")
            }
            GridEvent::AutomaticGenControl { area_mw } => {
                format!("AGC signal: area_mw={area_mw:.2} MW")
            }
            GridEvent::UnderFrequencyLoadShedding { buses, shed_mw } => {
                let per_bus = if buses.is_empty() {
                    0.0
                } else {
                    shed_mw / buses.len() as f64
                };
                for b in buses {
                    for load in &mut self.loads {
                        if load.bus == *b {
                            load.p_mw = (load.p_mw - per_bus).max(0.0);
                        }
                    }
                }
                format!("UFLS: buses={buses:?} shed={shed_mw:.2} MW")
            }
            GridEvent::VoltageLimitViolation { bus, voltage_pu } => {
                format!("VoltageLimitViolation: bus={bus} V={voltage_pu:.4} pu")
            }
            GridEvent::OverloadAlarm {
                branch_id,
                loading_pct,
            } => {
                format!("OverloadAlarm: branch={branch_id} loading={loading_pct:.1}%")
            }
        }
    }

    /// Compute net power balance: total online generation minus total load minus storage injection.
    fn compute_power_balance(&self) -> f64 {
        let gen: f64 = self
            .generators
            .iter()
            .filter(|g| g.is_online)
            .map(|g| g.p_mw)
            .sum();
        let load: f64 = self.loads.iter().map(|l| l.p_mw).sum();
        // Storage: positive power_mw = discharging (adds to supply), negative = charging (adds to demand)
        let storage_net: f64 = self.storages.iter().map(|s| s.power_mw).sum();
        gen + storage_net - load
    }

    /// Update system frequency using the swing equation.
    ///
    /// df/dt = ΔP / (2 * H * S_base)
    fn update_frequency(&self, delta_p_mw: f64, freq_hz: f64, dt_s: f64) -> f64 {
        let h_inertia = 5.0_f64;
        let s_base = self.config.base_mva;
        let df_dt = delta_p_mw / (2.0 * h_inertia * s_base);
        let new_freq = freq_hz + df_dt * dt_s;
        // Clamp to physically meaningful range (±5 Hz from nominal)
        let nom = self.config.nominal_frequency_hz;
        new_freq.clamp(nom - 5.0, nom + 5.0)
    }

    /// Apply Automatic Generation Control (AGC) to restore frequency.
    fn apply_agc(&mut self, delta_f_hz: f64, dt_s: f64) {
        let bias = 10.0 * self.config.base_mva; // frequency bias [MW/Hz]
        let ace = delta_f_hz * bias; // Area Control Error [MW]

        let total_participation: f64 = self
            .generators
            .iter()
            .filter(|g| g.is_online)
            .map(|g| g.agc_participation)
            .sum();

        if total_participation <= 0.0 {
            return;
        }

        for gen in &mut self.generators {
            if !gen.is_online || gen.agc_participation <= 0.0 {
                continue;
            }
            let fraction = gen.agc_participation / total_participation;
            let delta_p_gen = -ace * fraction;
            let max_ramp = gen.ramp_rate_mw_per_min * dt_s / 60.0;
            let delta_p_clamped = delta_p_gen.clamp(-max_ramp, max_ramp);
            gen.p_mw = (gen.p_mw + delta_p_clamped).clamp(gen.p_min_mw, gen.p_max_mw);
        }
    }

    /// Apply Under-Frequency Load Shedding in steps.
    ///
    /// Each 0.2 Hz below `ufls_threshold_hz` triggers one shedding step.
    fn apply_ufls(&mut self, frequency_hz: f64) -> f64 {
        let threshold = self.config.ufls_threshold_hz;
        if frequency_hz >= threshold {
            return 0.0;
        }
        let steps_below = ((threshold - frequency_hz) / 0.2).floor() as usize;
        let n_steps = steps_below.min(self.config.ufls_shed_pct.len());
        if n_steps == 0 {
            return 0.0;
        }

        // Gather total shedable load
        let total_shedable: f64 = self
            .loads
            .iter()
            .filter(|l| l.is_shedable)
            .map(|l| l.p_mw)
            .sum();

        let mut total_shed = 0.0_f64;

        for step_idx in 0..n_steps {
            let shed_fraction = self.config.ufls_shed_pct[step_idx];
            let target_shed = total_shedable * shed_fraction;
            let mut remaining = target_shed;

            // Shed from loads by descending priority (higher priority number = less critical)
            let mut indices: Vec<usize> = self
                .loads
                .iter()
                .enumerate()
                .filter(|(_, l)| l.is_shedable && l.p_mw > 0.0)
                .map(|(i, _)| i)
                .collect();
            indices.sort_by(|&a, &b| self.loads[b].priority.cmp(&self.loads[a].priority));

            for idx in indices {
                if remaining <= 0.0 {
                    break;
                }
                let shed = self.loads[idx].p_mw.min(remaining);
                self.loads[idx].p_mw -= shed;
                remaining -= shed;
                total_shed += shed;
            }
        }
        total_shed
    }

    /// DC power flow approximation: distribute flows proportional to branch susceptances.
    fn compute_branch_flows(&mut self) {
        let total_gen: f64 = self
            .generators
            .iter()
            .filter(|g| g.is_online)
            .map(|g| g.p_mw)
            .sum();
        let total_load: f64 = self.loads.iter().map(|l| l.p_mw).sum();
        let net_power = total_gen - total_load;

        let total_susceptance: f64 = self
            .branches
            .iter()
            .enumerate()
            .filter(|(_, b)| b.is_online)
            .map(|(i, _)| self.branch_susceptances.get(i).copied().unwrap_or(1.0))
            .sum();

        for (i, branch) in self.branches.iter_mut().enumerate() {
            if !branch.is_online {
                branch.current_flow_mw = 0.0;
                branch.current_flow_mvar = 0.0;
                branch.loading_pct = 0.0;
                continue;
            }
            let b_i = self.branch_susceptances.get(i).copied().unwrap_or(1.0);
            branch.current_flow_mw = if total_susceptance > 0.0 {
                (b_i / total_susceptance) * net_power * 0.5
            } else {
                0.0
            };
            branch.loading_pct =
                (branch.current_flow_mw.abs() / branch.rating_mva.max(1.0)) * 100.0;
        }
    }

    /// Check and return violation strings for frequency, voltage, and branch overloads.
    fn check_violations(&self, freq_hz: f64, voltages: &[f64]) -> Vec<String> {
        let mut violations = Vec::new();
        let nom = self.config.nominal_frequency_hz;

        if freq_hz < nom - 0.5 {
            violations.push(format!(
                "UnderFrequency: f={freq_hz:.3} Hz (nominal {nom} Hz)"
            ));
        } else if freq_hz > nom + 0.5 {
            violations.push(format!(
                "OverFrequency: f={freq_hz:.3} Hz (nominal {nom} Hz)"
            ));
        }

        for (bus, &v) in voltages.iter().enumerate() {
            if v < self.config.voltage_min_pu {
                violations.push(format!("UnderVoltage: bus={bus} V={v:.4} pu"));
            } else if v > self.config.voltage_max_pu {
                violations.push(format!("OverVoltage: bus={bus} V={v:.4} pu"));
            }
        }

        for branch in &self.branches {
            if branch.is_online && branch.loading_pct > self.config.max_branch_loading_pct {
                violations.push(format!(
                    "BranchOverload: branch={} loading={:.1}%",
                    branch.id, branch.loading_pct
                ));
            }
        }

        violations
    }

    /// Update storage state-of-charge based on current power dispatch.
    fn update_storage_soc(&mut self, dt_s: f64) {
        for st in &mut self.storages {
            let soc_delta = if st.power_mw > 0.0 {
                // Discharging: loses energy
                let energy_out = st.power_mw * dt_s / 3600.0;
                -energy_out / (st.capacity_mwh.max(1e-9) * st.efficiency.max(1e-9))
            } else if st.power_mw < 0.0 {
                // Charging: gains energy
                let energy_in = st.power_mw.abs() * dt_s / 3600.0 * st.efficiency;
                energy_in / st.capacity_mwh.max(1e-9)
            } else {
                0.0
            };
            st.soc = (st.soc + soc_delta).clamp(0.0, 1.0);
        }
    }

    /// Compute summary statistics from the recorded snapshots.
    fn compute_statistics(snapshots: &[SystemSnapshot], dt_s: f64) -> GridOpsStatistics {
        if snapshots.is_empty() {
            return GridOpsStatistics {
                duration_s: 0.0,
                total_energy_mwh: 0.0,
                renewable_energy_mwh: 0.0,
                load_served_pct: 100.0,
                shed_energy_mwh: 0.0,
                total_co2_ton: 0.0,
                min_frequency_hz: 50.0,
                max_frequency_hz: 50.0,
                n_frequency_violations: 0,
                n_voltage_violations: 0,
                n_line_trips: 0,
                n_generator_trips: 0,
                n_load_shed_events: 0,
                system_resilience_index: 1.0,
            };
        }

        let duration_s = snapshots.last().map(|s| s.time_s).unwrap_or(0.0)
            - snapshots.first().map(|s| s.time_s).unwrap_or(0.0);

        let dt_h = dt_s / 3600.0;
        let mut total_energy_mwh = 0.0_f64;
        let mut renewable_energy_mwh = 0.0_f64;
        let mut total_co2_ton = 0.0_f64;
        let mut total_load_mwh = 0.0_f64;
        let mut shed_energy_mwh = 0.0_f64;
        let mut min_freq = f64::INFINITY;
        let mut max_freq = f64::NEG_INFINITY;
        let mut n_freq_violations = 0usize;
        let mut n_volt_violations = 0usize;
        let mut n_line_trips = 0usize;
        let mut n_gen_trips = 0usize;
        let mut n_load_shed_events = 0usize;

        // Infer nominal frequency from first snapshot (50 or 60 Hz)
        let nominal = if !snapshots.is_empty() && snapshots[0].frequency_hz > 55.0 {
            60.0
        } else {
            50.0
        };

        for snap in snapshots {
            let gen_mwh = snap.total_generation_mw * dt_h;
            total_energy_mwh += gen_mwh;
            total_load_mwh += snap.total_load_mw * dt_h;

            for gen in &snap.generators {
                if gen.is_online {
                    let e = gen.p_mw * dt_h;
                    let fuel = gen.fuel_type.to_lowercase();
                    if fuel.contains("wind") || fuel.contains("solar") || fuel.contains("pv") {
                        renewable_energy_mwh += e;
                    }
                    total_co2_ton += gen.p_mw * gen.co2_kg_per_mwh * dt_h / 1000.0;
                }
            }

            if (snap.frequency_hz - nominal).abs() > 0.5 {
                n_freq_violations += 1;
            }
            min_freq = min_freq.min(snap.frequency_hz);
            max_freq = max_freq.max(snap.frequency_hz);

            for v in &snap.violations {
                let vl = v.to_lowercase();
                if vl.contains("voltage") {
                    n_volt_violations += 1;
                }
            }

            for ev in &snap.events_this_step {
                let el = ev.to_lowercase();
                if el.contains("linetrip") || el.contains("line trip") {
                    n_line_trips += 1;
                }
                if el.contains("generatortrip") || el.contains("generator trip") {
                    n_gen_trips += 1;
                }
                if el.contains("ufls") || el.contains("shed") {
                    n_load_shed_events += 1;
                }
            }

            // Estimate shed from imbalance: positive imbalance = surplus, negative = deficit
            if snap.power_imbalance_mw < 0.0 {
                shed_energy_mwh += snap.power_imbalance_mw.abs() * dt_h;
            }
        }

        let load_served_pct = if total_load_mwh > 0.0 {
            ((total_load_mwh - shed_energy_mwh) / total_load_mwh * 100.0).clamp(0.0, 100.0)
        } else {
            100.0
        };

        let system_resilience_index =
            1.0 - shed_energy_mwh / (total_energy_mwh + shed_energy_mwh + 1e-9);

        if min_freq.is_infinite() {
            min_freq = nominal;
        }
        if max_freq.is_infinite() {
            max_freq = nominal;
        }

        GridOpsStatistics {
            duration_s,
            total_energy_mwh,
            renewable_energy_mwh,
            load_served_pct,
            shed_energy_mwh,
            total_co2_ton,
            min_frequency_hz: min_freq,
            max_frequency_hz: max_freq,
            n_frequency_violations: n_freq_violations,
            n_voltage_violations: n_volt_violations,
            n_line_trips,
            n_generator_trips: n_gen_trips,
            n_load_shed_events,
            system_resilience_index: system_resilience_index.clamp(0.0, 1.0),
        }
    }

    /// Take a snapshot of the current system state.
    fn take_snapshot(
        &self,
        freq_hz: f64,
        events: Vec<String>,
        violations: Vec<String>,
    ) -> SystemSnapshot {
        let total_generation_mw: f64 = self
            .generators
            .iter()
            .filter(|g| g.is_online)
            .map(|g| g.p_mw)
            .sum();
        let total_load_mw: f64 = self.loads.iter().map(|l| l.p_mw).sum();
        // Simplified losses: 2% of transmitted power
        let total_losses_mw = total_generation_mw * 0.02;
        let power_imbalance_mw = total_generation_mw - total_load_mw - total_losses_mw;
        let n_generators_online = self.generators.iter().filter(|g| g.is_online).count();

        SystemSnapshot {
            time_s: self.clock.current_time_s,
            generators: self.generators.clone(),
            loads: self.loads.clone(),
            branches: self.branches.clone(),
            storages: self.storages.clone(),
            frequency_hz: freq_hz,
            total_generation_mw,
            total_load_mw,
            total_losses_mw,
            power_imbalance_mw,
            events_this_step: events,
            violations,
            n_generators_online,
        }
    }
}

// ─── Scenario Builder ─────────────────────────────────────────────────────

/// Builder for common simulation scenarios.
pub struct ScenarioBuilder;

impl ScenarioBuilder {
    /// Generate a 24-hour sinusoidal load curve as scheduled events (hourly steps).
    ///
    /// P(h) = P_min + (P_max - P_min) * 0.5 * (1 - cos(2π*h/24))
    pub fn daily_load_curve(bus: usize, p_max_mw: f64, p_min_mw: f64) -> Vec<ScheduledEvent> {
        let mut events = Vec::with_capacity(24);
        let mut prev_p = p_min_mw; // hour 0 starts at minimum

        for h in 0..24usize {
            let t_s = h as f64 * 3600.0;
            let p = p_min_mw
                + (p_max_mw - p_min_mw)
                    * 0.5
                    * (1.0 - (2.0 * std::f64::consts::PI * h as f64 / 24.0).cos());
            let delta = p - prev_p;

            let (event, desc) = if delta >= 0.0 {
                (
                    GridEvent::LoadIncrease {
                        bus,
                        delta_mw: delta,
                    },
                    format!("DailyLoadCurve h={h} +{delta:.2} MW"),
                )
            } else {
                (
                    GridEvent::LoadDecrease {
                        bus,
                        delta_mw: delta.abs(),
                    },
                    format!("DailyLoadCurve h={h} -{:.2} MW", delta.abs()),
                )
            };

            events.push(ScheduledEvent {
                time_s: t_s,
                event,
                description: desc,
            });
            prev_p = p;
        }
        events
    }

    /// N-1 single line trip event at `t_fault_s`.
    pub fn n1_line_trip(branch_id: usize, t_fault_s: f64) -> Vec<ScheduledEvent> {
        vec![ScheduledEvent {
            time_s: t_fault_s,
            event: GridEvent::LineTrip {
                branch_id,
                reason: "N-1 contingency".to_string(),
            },
            description: format!("N-1 LineTrip: branch={branch_id} at t={t_fault_s:.0}s"),
        }]
    }

    /// Major generator loss event at `t_fault_s`.
    pub fn generator_loss(
        generator_bus: usize,
        capacity_mw: f64,
        t_fault_s: f64,
    ) -> Vec<ScheduledEvent> {
        vec![ScheduledEvent {
            time_s: t_fault_s,
            event: GridEvent::GeneratorTrip {
                bus: generator_bus,
                capacity_mw,
                reason: "Unexpected generator trip".to_string(),
            },
            description: format!(
                "GeneratorLoss: bus={generator_bus} cap={capacity_mw:.1} MW at t={t_fault_s:.0}s"
            ),
        }]
    }

    /// Wind ramp event: load events every 300 s from `t_start_s` to `t_start_s + duration_s`.
    pub fn wind_ramp(
        bus: usize,
        ramp_start_mw: f64,
        ramp_end_mw: f64,
        duration_s: f64,
        t_start_s: f64,
    ) -> Vec<ScheduledEvent> {
        let step_s = 300.0_f64;
        let n_steps = (duration_s / step_s).ceil() as usize;
        let mut events = Vec::with_capacity(n_steps);
        let mut prev_power = ramp_start_mw;

        for i in 0..=n_steps {
            let t = t_start_s + i as f64 * step_s;
            let frac = if n_steps > 0 {
                (i as f64 / n_steps as f64).clamp(0.0, 1.0)
            } else {
                1.0
            };
            let power = ramp_start_mw + (ramp_end_mw - ramp_start_mw) * frac;
            let delta = power - prev_power;

            let (event, desc) = if delta >= 0.0 {
                (
                    GridEvent::LoadIncrease {
                        bus,
                        delta_mw: delta,
                    },
                    format!("WindRamp t={t:.0}s +{delta:.2} MW"),
                )
            } else {
                (
                    GridEvent::LoadDecrease {
                        bus,
                        delta_mw: delta.abs(),
                    },
                    format!("WindRamp t={t:.0}s -{:.2} MW", delta.abs()),
                )
            };

            events.push(ScheduledEvent {
                time_s: t,
                event,
                description: desc,
            });
            prev_power = power;
        }
        events
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Helpers ──────────────────────────────────────────────────────────

    fn make_generator(id: usize, bus: usize, p_mw: f64, p_max: f64) -> SimGenerator {
        SimGenerator {
            id,
            bus,
            p_mw,
            p_max_mw: p_max,
            p_min_mw: 0.0,
            ramp_rate_mw_per_min: 5.0,
            agc_participation: 0.5,
            is_online: true,
            startup_time_min: 10.0,
            fuel_type: "gas".to_string(),
            co2_kg_per_mwh: 400.0,
        }
    }

    fn make_load(bus: usize, p_mw: f64) -> SimLoad {
        SimLoad {
            bus,
            p_mw,
            q_mvar: 0.0,
            is_shedable: true,
            priority: 2,
        }
    }

    fn make_branch(id: usize, from: usize, to: usize) -> SimBranch {
        SimBranch {
            id,
            from,
            to,
            is_online: true,
            rating_mva: 100.0,
            current_flow_mw: 0.0,
            current_flow_mvar: 0.0,
            loading_pct: 0.0,
        }
    }

    fn make_storage(bus: usize) -> SimStorage {
        SimStorage {
            bus,
            soc: 0.5,
            capacity_mwh: 10.0,
            power_mw: 0.0,
            max_charge_mw: 5.0,
            max_discharge_mw: 5.0,
            efficiency: 0.95,
        }
    }

    fn make_simulator(duration_s: f64, dt_s: f64) -> GridOperationsSimulator {
        let config = QdGridOpsConfig {
            n_buses: 5,
            base_mva: 100.0,
            nominal_frequency_hz: 50.0,
            frequency_deadband_hz: 0.02,
            ufls_threshold_hz: 47.5,
            ufls_shed_pct: vec![0.10, 0.15, 0.20],
            ovf_threshold_hz: 51.5,
            voltage_min_pu: 0.95,
            voltage_max_pu: 1.05,
            max_branch_loading_pct: 90.0,
        };
        let generators = vec![
            make_generator(0, 1, 60.0, 100.0),
            make_generator(1, 2, 40.0, 80.0),
        ];
        let loads = vec![make_load(3, 50.0), make_load(4, 45.0)];
        let branches = vec![make_branch(0, 1, 2), make_branch(1, 2, 3)];
        let storages = vec![make_storage(5)];
        GridOperationsSimulator::new(
            config, generators, loads, branches, storages, duration_s, dt_s,
        )
    }

    // ── SimClock tests ────────────────────────────────────────────────────

    #[test]
    fn test_sim_clock_advance() {
        let mut clock = SimClock::new(0.0, 100.0, 10.0);
        assert!((clock.current_time_s - 0.0).abs() < 1e-9);
        let cont = clock.advance();
        assert!(cont);
        assert!((clock.current_time_s - 10.0).abs() < 1e-9);
    }

    #[test]
    fn test_sim_clock_time_of_day() {
        let clock = SimClock::new(25.0 * 3600.0, 100.0 * 3600.0, 3600.0);
        let tod = clock.time_of_day_h();
        assert!((tod - 1.0).abs() < 1e-9, "Expected 1.0 h, got {tod}");
    }

    #[test]
    fn test_sim_clock_complete() {
        let mut clock = SimClock::new(0.0, 30.0, 10.0);
        assert!(clock.advance()); // t=10
        assert!(clock.advance()); // t=20
        assert!(clock.advance()); // t=30
        assert!(!clock.advance()); // t=40 > end
    }

    // ── Component creation tests ──────────────────────────────────────────

    #[test]
    fn test_generator_creation() {
        let gen = make_generator(0, 1, 80.0, 100.0);
        assert_eq!(gen.bus, 1);
        assert!((gen.p_mw - 80.0).abs() < 1e-9);
        assert!(gen.is_online);
        assert_eq!(gen.fuel_type, "gas");
    }

    #[test]
    fn test_load_creation() {
        let load = make_load(3, 50.0);
        assert_eq!(load.bus, 3);
        assert!((load.p_mw - 50.0).abs() < 1e-9);
        assert!(load.is_shedable);
        assert_eq!(load.priority, 2);
    }

    // ── Storage SoC test ─────────────────────────────────────────────────

    #[test]
    fn test_storage_soc_update() {
        let mut sim = make_simulator(60.0, 60.0);
        // Set storage to discharge at 5 MW
        sim.storages[0].power_mw = 5.0;
        sim.storages[0].soc = 0.5;
        let initial_soc = sim.storages[0].soc;
        sim.update_storage_soc(3600.0); // 1 hour
                                        // Should lose 5 MWh / (10 MWh * 0.95 eff) = 0.5263
        assert!(
            sim.storages[0].soc < initial_soc,
            "SoC should decrease when discharging"
        );
    }

    // ── Simulator creation ────────────────────────────────────────────────

    #[test]
    fn test_simulator_creation() {
        let sim = make_simulator(3600.0, 60.0);
        assert_eq!(sim.generators.len(), 2);
        assert_eq!(sim.loads.len(), 2);
        assert_eq!(sim.branches.len(), 2);
        assert_eq!(sim.storages.len(), 1);
        assert!((sim.clock.end_time_s - 3600.0).abs() < 1e-9);
    }

    // ── Schedule event test ───────────────────────────────────────────────

    #[test]
    fn test_schedule_event() {
        let mut sim = make_simulator(3600.0, 60.0);
        sim.schedule_event(
            500.0,
            GridEvent::LineTrip {
                branch_id: 0,
                reason: "test".to_string(),
            },
            "test event".to_string(),
        );
        assert_eq!(sim.scheduled_events.len(), 1);
        assert!((sim.scheduled_events[0].time_s - 500.0).abs() < 1e-9);
    }

    // ── Power balance tests ───────────────────────────────────────────────

    #[test]
    fn test_power_balance_balanced() {
        let mut sim = make_simulator(3600.0, 60.0);
        // Set gen = load = 50 MW
        sim.generators[0].p_mw = 50.0;
        sim.generators[1].p_mw = 0.0;
        sim.generators[1].is_online = false;
        sim.loads[0].p_mw = 30.0;
        sim.loads[1].p_mw = 20.0;
        let bal = sim.compute_power_balance();
        assert!((bal).abs() < 1e-6, "Balance should be ~0, got {bal}");
    }

    #[test]
    fn test_power_balance_surplus() {
        let mut sim = make_simulator(3600.0, 60.0);
        sim.generators[0].p_mw = 80.0;
        sim.generators[1].p_mw = 20.0;
        sim.loads[0].p_mw = 40.0;
        sim.loads[1].p_mw = 30.0;
        let bal = sim.compute_power_balance();
        assert!(bal > 0.0, "Surplus: balance should be positive, got {bal}");
        assert!((bal - 30.0).abs() < 1e-6);
    }

    // ── Frequency update tests ────────────────────────────────────────────

    #[test]
    fn test_update_frequency_surplus() {
        let sim = make_simulator(3600.0, 60.0);
        let f = sim.update_frequency(50.0, 50.0, 1.0);
        assert!(f > 50.0, "Surplus should raise frequency: {f}");
    }

    #[test]
    fn test_update_frequency_deficit() {
        let sim = make_simulator(3600.0, 60.0);
        let f = sim.update_frequency(-50.0, 50.0, 1.0);
        assert!(f < 50.0, "Deficit should lower frequency: {f}");
    }

    // ── AGC test ──────────────────────────────────────────────────────────

    #[test]
    fn test_apply_agc_reduces_imbalance() {
        let mut sim = make_simulator(3600.0, 60.0);
        // Under-frequency: generators should ramp up
        let p_before: f64 = sim
            .generators
            .iter()
            .filter(|g| g.is_online)
            .map(|g| g.p_mw)
            .sum();
        sim.apply_agc(-0.5, 60.0); // -0.5 Hz deviation, 1 minute step
        let p_after: f64 = sim
            .generators
            .iter()
            .filter(|g| g.is_online)
            .map(|g| g.p_mw)
            .sum();
        assert!(p_after >= p_before, "AGC should increase generation under under-frequency: before={p_before} after={p_after}");
    }

    // ── UFLS tests ────────────────────────────────────────────────────────

    #[test]
    fn test_apply_ufls_below_threshold() {
        let mut sim = make_simulator(3600.0, 60.0);
        let total_load_before: f64 = sim.loads.iter().map(|l| l.p_mw).sum();
        let shed = sim.apply_ufls(47.0); // 0.5 Hz below threshold → 1 step
        assert!(shed > 0.0, "Should shed load below UFLS threshold: {shed}");
        let total_load_after: f64 = sim.loads.iter().map(|l| l.p_mw).sum();
        assert!(
            total_load_after < total_load_before,
            "Load should decrease after UFLS"
        );
    }

    #[test]
    fn test_apply_ufls_not_triggered() {
        let mut sim = make_simulator(3600.0, 60.0);
        let shed = sim.apply_ufls(49.0); // Above threshold
        assert!(
            (shed).abs() < 1e-9,
            "UFLS should not trigger above threshold: {shed}"
        );
    }

    // ── Event processing tests ────────────────────────────────────────────

    #[test]
    fn test_process_event_generator_trip() {
        let mut sim = make_simulator(3600.0, 60.0);
        assert!(sim.generators[0].is_online);
        let evt = GridEvent::GeneratorTrip {
            bus: 1,
            capacity_mw: 100.0,
            reason: "test".to_string(),
        };
        let desc = sim.process_event(&evt);
        assert!(
            !sim.generators[0].is_online,
            "Generator at bus 1 should be offline"
        );
        assert!((sim.generators[0].p_mw).abs() < 1e-9);
        assert!(desc.contains("GeneratorTrip"));
    }

    #[test]
    fn test_process_event_line_trip() {
        let mut sim = make_simulator(3600.0, 60.0);
        assert!(sim.branches[0].is_online);
        let evt = GridEvent::LineTrip {
            branch_id: 0,
            reason: "fault".to_string(),
        };
        let desc = sim.process_event(&evt);
        assert!(!sim.branches[0].is_online, "Branch 0 should be offline");
        assert!(desc.contains("LineTrip"));
    }

    #[test]
    fn test_process_event_reconnect() {
        let mut sim = make_simulator(3600.0, 60.0);
        // Trip first, then reconnect
        sim.generators[0].is_online = false;
        sim.generators[0].p_mw = 0.0;
        let evt = GridEvent::GeneratorReconnect {
            bus: 1,
            capacity_mw: 100.0,
        };
        let desc = sim.process_event(&evt);
        assert!(
            sim.generators[0].is_online,
            "Generator at bus 1 should be back online"
        );
        assert!(desc.contains("GeneratorReconnect"));
    }

    // ── Full simulation run tests ─────────────────────────────────────────

    #[test]
    fn test_run_24h_no_events() {
        let mut sim = make_simulator(86400.0, 300.0); // 24h, 5-min steps
        let result = sim.run().expect("simulation should succeed");
        assert!(!result.snapshots.is_empty(), "Should have snapshots");
        // 24h / 5min = 288 steps
        assert!(
            result.snapshots.len() >= 280,
            "Expected ~288 snapshots, got {}",
            result.snapshots.len()
        );
        assert!(!result.frequency_history.is_empty());
    }

    #[test]
    fn test_run_with_n1_event() {
        let mut sim = make_simulator(7200.0, 60.0); // 2h, 1-min steps
        let events = ScenarioBuilder::n1_line_trip(0, 1800.0);
        for se in events {
            sim.schedule_event(se.time_s, se.event, se.description);
        }
        let result = sim.run().expect("simulation with N-1 should succeed");
        // Check that line trip event was logged
        let has_line_trip = result
            .events_log
            .iter()
            .any(|(_, e)| e.contains("LineTrip"));
        assert!(has_line_trip, "Should have a LineTrip in events log");
    }

    // ── Statistics test ───────────────────────────────────────────────────

    #[test]
    fn test_statistics_load_served() {
        let mut sim = make_simulator(3600.0, 60.0);
        let result = sim.run().expect("simulation failed");
        let stats = &result.statistics;
        assert!(
            stats.load_served_pct >= 0.0 && stats.load_served_pct <= 100.0,
            "load_served_pct out of range: {}",
            stats.load_served_pct
        );
        assert!(stats.min_frequency_hz <= stats.max_frequency_hz);
        assert!(stats.system_resilience_index >= 0.0 && stats.system_resilience_index <= 1.0);
    }

    // ── ScenarioBuilder tests ─────────────────────────────────────────────

    #[test]
    fn test_scenario_builder_daily_curve() {
        let events = ScenarioBuilder::daily_load_curve(1, 100.0, 40.0);
        assert_eq!(events.len(), 24, "Should produce exactly 24 events");
        // First event should be at t=0
        assert!((events[0].time_s).abs() < 1e-9);
        // Last event at t=23*3600
        assert!((events[23].time_s - 23.0 * 3600.0).abs() < 1e-9);
    }

    // ── Legacy simulator tests ────────────────────────────────────────────

    fn make_config(hours: usize, contingency_prob: f64, weather: bool) -> GridOpsConfig {
        GridOpsConfig {
            simulation_hours: hours,
            dt_minutes: 60.0,
            operator_skill: 0.85,
            automation_level: 0.6,
            contingency_probability: contingency_prob,
            weather_events: weather,
        }
    }

    #[test]
    fn test_no_contingencies_high_reliability() {
        let config = make_config(168, 0.0, false);
        let sim = GridOpsSimulator::new(config, 1500.0, 800.0);
        let result = sim.simulate().expect("simulation failed");
        assert!(
            result.system_reliability_pct > 99.0,
            "Reliability should be >99%: {:.2}%",
            result.system_reliability_pct
        );
        assert_eq!(result.total_hours, 168);
    }

    #[test]
    fn test_high_contingency_reliability_drops() {
        let low_config = make_config(720, 0.0, false);
        let high_config = make_config(720, 0.5, false);
        let res_low = GridOpsSimulator::new(low_config, 1200.0, 800.0)
            .simulate()
            .expect("low sim failed");
        let res_high = GridOpsSimulator::new(high_config, 1200.0, 800.0)
            .simulate()
            .expect("high sim failed");
        assert!(res_low.system_reliability_pct >= res_high.system_reliability_pct);
        assert!(res_high.n_events > 0);
    }

    #[test]
    fn test_error_zero_hours() {
        let config = make_config(0, 0.0, false);
        let sim = GridOpsSimulator::new(config, 1000.0, 800.0);
        assert!(matches!(
            sim.simulate(),
            Err(GridOpsError::ZeroSimulationHours)
        ));
    }

    #[test]
    fn test_load_profile_length() {
        let config = make_config(8760, 0.0, false);
        let sim = GridOpsSimulator::new(config, 1200.0, 1000.0);
        let mut rng: u64 = 42;
        let profile = sim.generate_load_profile(8760, &mut rng);
        assert_eq!(profile.len(), 8760);
    }

    // ── run() error paths ─────────────────────────────────────────────────

    // Reason: run() must return InvalidParameter when end_time_s == 0
    #[test]
    fn test_run_invalid_end_time() {
        let mut sim = make_simulator(0.0, 60.0);
        assert!(sim.run().is_err(), "run() with end_time_s=0 should error");
    }

    // Reason: run() must return InvalidParameter when dt_s <= 0
    #[test]
    fn test_run_invalid_dt() {
        let mut sim = make_simulator(3600.0, 0.0);
        assert!(sim.run().is_err(), "run() with dt_s=0 should error");
    }

    // ── Legacy validate() error paths ─────────────────────────────────────

    // Reason: validate() must catch negative generator capacity
    #[test]
    fn test_validate_invalid_gen_capacity() {
        let config = make_config(24, 0.0, false);
        let sim = GridOpsSimulator::new(config, -100.0, 800.0);
        assert!(matches!(
            sim.simulate(),
            Err(GridOpsError::InvalidGenCapacity(_))
        ));
    }

    // Reason: validate() must catch non-positive dt_minutes
    #[test]
    fn test_validate_invalid_timestep() {
        let config = GridOpsConfig {
            simulation_hours: 24,
            dt_minutes: -5.0,
            ..GridOpsConfig::default()
        };
        let sim = GridOpsSimulator::new(config, 1000.0, 800.0);
        assert!(matches!(
            sim.simulate(),
            Err(GridOpsError::InvalidTimeStep(_))
        ));
    }

    // ── Storage SoC charging branch ───────────────────────────────────────

    // Reason: update_storage_soc must increase SoC when power_mw is negative (charging)
    #[test]
    fn test_storage_soc_charging_increases() {
        let mut sim = make_simulator(60.0, 60.0);
        sim.storages[0].power_mw = -4.0; // charging
        sim.storages[0].soc = 0.3;
        let initial_soc = sim.storages[0].soc;
        sim.update_storage_soc(3600.0); // 1 hour
        assert!(
            sim.storages[0].soc > initial_soc,
            "SoC should increase when charging: before={initial_soc} after={}",
            sim.storages[0].soc
        );
    }

    // ── compute_branch_flows distributes net power ─────────────────────────

    // Reason: online branches must carry nonzero flow; offline branches must have zero flow
    #[test]
    fn test_compute_branch_flows_online_offline() {
        let mut sim = make_simulator(3600.0, 60.0);
        // Gen surplus: 100 MW gen, 40 MW load → +60 MW net
        sim.generators[0].p_mw = 100.0;
        sim.generators[1].is_online = false;
        sim.generators[1].p_mw = 0.0;
        sim.loads[0].p_mw = 20.0;
        sim.loads[1].p_mw = 20.0;
        // Trip branch 1; branch 0 stays online
        sim.branches[1].is_online = false;
        sim.compute_branch_flows();
        assert!(
            sim.branches[0].current_flow_mw.abs() > 0.0,
            "Online branch 0 should carry nonzero flow"
        );
        assert!(
            (sim.branches[1].current_flow_mw).abs() < 1e-9,
            "Offline branch 1 should have zero flow"
        );
        assert!(
            (sim.branches[1].loading_pct).abs() < 1e-9,
            "Offline branch 1 loading_pct should be zero"
        );
    }

    // ── ScenarioBuilder::wind_ramp event count ────────────────────────────

    // Reason: wind_ramp must produce n_steps+1 events spanning the full duration
    #[test]
    fn test_scenario_wind_ramp_event_count() {
        let events = ScenarioBuilder::wind_ramp(1, 10.0, 50.0, 900.0, 0.0);
        // 900s / 300s = 3 steps → 4 events (indices 0..=3)
        assert_eq!(
            events.len(),
            4,
            "wind_ramp should produce n_steps+1 events, got {}",
            events.len()
        );
        // First event at t_start=0
        assert!((events[0].time_s).abs() < 1e-9);
        // Last event at t_start + n_steps*step_s = 0 + 3*300 = 900
        assert!(
            (events.last().map(|e| e.time_s).unwrap_or(-1.0) - 900.0).abs() < 1e-9,
            "Last wind ramp event should be at t=900 s"
        );
    }
}
