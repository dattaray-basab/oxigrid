//! Auto-generated module
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

use serde::{Deserialize, Serialize};
use thiserror::Error;

use super::functions::lcg_next;
use super::types_4::{GridOpsConfig, SimStorage};

pub struct GridOpsSimulator {
    pub(super) config: GridOpsConfig,
    pub(super) gen_capacity_mw: f64,
    pub(super) peak_load_mw: f64,
    #[allow(dead_code)]
    pub(super) reserve_margin_pct: f64,
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
    pub(super) fn validate(&self) -> Result<(), GridOpsError> {
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
/// Builder for common simulation scenarios.
pub struct ScenarioBuilder;
impl ScenarioBuilder {
    /// Generate a 24-hour sinusoidal load curve as scheduled events (hourly steps).
    ///
    /// P(h) = P_min + (P_max - P_min) * 0.5 * (1 - cos(2π*h/24))
    pub fn daily_load_curve(bus: usize, p_max_mw: f64, p_min_mw: f64) -> Vec<ScheduledEvent> {
        let mut events = Vec::with_capacity(24);
        let mut prev_p = p_min_mw;
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
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OperationalEvent {
    pub time_h: f64,
    pub event_type: EventType,
    pub severity: f64,
    pub duration_h: f64,
    pub affected_element: String,
    pub automatic_response: bool,
}
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
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ActionOutcome {
    Success,
    PartialSuccess { achieved_pct: f64 },
    Failed { reason: String },
    Delayed { delay_min: f64 },
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
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OperatorAction {
    pub time_h: f64,
    pub action_type: OperatorActionType,
    pub target: String,
    pub value: f64,
    pub reason: String,
    pub outcome: ActionOutcome,
}
/// A grid event scheduled to occur at a specific simulation time.
pub struct ScheduledEvent {
    pub time_s: f64,
    pub event: GridEvent,
    pub description: String,
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
