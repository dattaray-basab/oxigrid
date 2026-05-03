//! New-generation grid operator training simulator.
//!
//! Provides [`TrainingSimSession`] for scenario injection, trainee performance
//! tracking, debrief analysis, and a [`ScenarioLibrary`] of pre-built exercises.
//!
//! # Quick Start
//!
//! ```rust,ignore
//! use oxigrid::simulation::operator_training_v2::{ScenarioLibrary, TrainingSimSession};
//!
//! let scenario = ScenarioLibrary::create_n1_scenario();
//! let mut session = TrainingSimSession::new(scenario, "trainee_42".to_string(), 5, 6, 3);
//! let events = session.advance_time(35.0);
//! assert!(!events.is_empty());
//! ```

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

// ─── Event types ──────────────────────────────────────────────────────────────

/// Type of training event injected into the simulation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum TrainingEventType {
    /// A generator trips offline at the given bus.
    GeneratorTrip {
        /// Bus index (0-based).
        bus: usize,
        /// Lost capacity \[MW\].
        mw: f64,
    },
    /// A transmission line trips between two buses.
    LineTrip {
        /// Sending-end bus index.
        from: usize,
        /// Receiving-end bus index.
        to: usize,
    },
    /// Load increases at a bus.
    LoadIncrease {
        /// Bus index.
        bus: usize,
        /// Load increase \[MW\].
        delta_mw: f64,
    },
    /// Bus voltage deviates from nominal.
    VoltageDeviation {
        /// Bus index.
        bus: usize,
        /// Voltage deviation \[pu\].
        delta_pu: f64,
    },
    /// System frequency deviates from 50 Hz.
    FrequencyDeviation {
        /// Frequency deviation \[Hz\] (negative = under-frequency).
        delta_hz: f64,
    },
    /// SCADA/communication link fails for a duration.
    CommunicationFailure {
        /// Duration of failure \[s\].
        duration_s: f64,
    },
    /// Instructor-triggered narrative event.
    ManualTrigger {
        /// Narrative description shown to the trainee.
        description: String,
    },
}

/// A single event injected during a training scenario.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TrainingEvent {
    /// Simulation time at which the event fires \[s\].
    pub trigger_time: f64,
    /// What kind of stimulus occurs.
    pub event_type: TrainingEventType,
}

// ─── KPI ─────────────────────────────────────────────────────────────────────

/// A key-performance indicator target for a training scenario.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct KpiTarget {
    /// Human-readable name, e.g. "Frequency recovery".
    pub name: String,
    /// Desired value (e.g. 50.0 Hz, 1.0 pu).
    pub target_value: f64,
    /// Acceptable deviation from target.
    pub tolerance: f64,
    /// Whether the KPI was achieved at the end of the session.
    pub achieved: bool,
}

// ─── TrainingScenario ─────────────────────────────────────────────────────────

/// Defines a complete training exercise.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrainingScenario {
    /// Short name of the scenario.
    pub name: String,
    /// Detailed description.
    pub description: String,
    /// Total scenario duration \[s\].
    pub duration_seconds: f64,
    /// Events sorted by `trigger_time`.
    pub events: Vec<TrainingEvent>,
    /// KPIs the trainee must satisfy.
    pub target_kpis: Vec<KpiTarget>,
}

// ─── Trainee action ───────────────────────────────────────────────────────────

/// Action type available to a trainee.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum TraineeActionType {
    /// Re-dispatch a generator at the given bus.
    DispatchGenerator {
        /// Bus index.
        bus: usize,
        /// New MW setpoint.
        mw: f64,
    },
    /// Open a circuit breaker on a branch.
    TripBreaker {
        /// Sending-end bus.
        from: usize,
        /// Receiving-end bus.
        to: usize,
    },
    /// Close a circuit breaker on a branch.
    CloseBreaker {
        /// Sending-end bus.
        from: usize,
        /// Receiving-end bus.
        to: usize,
    },
    /// Change transformer tap position.
    TapChanger {
        /// Transformer identifier.
        transformer_id: usize,
        /// New tap position (integer step).
        tap_position: i32,
    },
    /// Activate spinning/non-spinning reserve.
    ActivateReserve {
        /// Reserve amount \[MW\].
        reserve_mw: f64,
    },
    /// Island a set of buses from the rest of the grid.
    IslandSection {
        /// Bus indices to isolate.
        buses: Vec<usize>,
    },
    /// Request assistance from a supervisor.
    RequestHelp,
    /// Acknowledge a SCADA alarm.
    AcknowledgeAlarm {
        /// Alarm identifier.
        alarm_id: usize,
    },
}

/// Recorded action by the trainee during a session.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TraineeResponse {
    /// Simulation time when the action was taken \[s\].
    pub timestamp: f64,
    /// The action performed.
    pub action_type: TraineeActionType,
    /// Time from the triggering event to this action \[s\].
    pub response_latency_s: f64,
    /// Whether the action was evaluated as correct.
    pub is_correct: bool,
}

// ─── Alarm ────────────────────────────────────────────────────────────────────

/// Alarm severity level.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AlarmSeverity {
    /// Life-safety or equipment-damaging condition.
    Critical,
    /// Significant operational impact.
    Major,
    /// Minor deviation from normal.
    Minor,
    /// Informational notice only.
    Info,
}

impl AlarmSeverity {
    fn rank(self) -> u8 {
        match self {
            Self::Critical => 3,
            Self::Major => 2,
            Self::Minor => 1,
            Self::Info => 0,
        }
    }
}

impl PartialOrd for AlarmSeverity {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for AlarmSeverity {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.rank().cmp(&other.rank())
    }
}

/// A SCADA alarm generated during the training simulation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Alarm {
    /// Unique alarm identifier.
    pub id: usize,
    /// Severity level.
    pub severity: AlarmSeverity,
    /// Human-readable message.
    pub message: String,
    /// Simulation time when the alarm was raised \[s\].
    pub time: f64,
    /// Whether the trainee has acknowledged this alarm.
    pub acknowledged: bool,
}

// ─── SystemState ──────────────────────────────────────────────────────────────

/// Simplified power system state at a simulation time step.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SystemState {
    /// System frequency \[Hz\] (nominal 50.0).
    pub frequency_hz: f64,
    /// Bus voltage magnitudes \[pu\], one per bus.
    pub voltage_pu: Vec<f64>,
    /// Branch loading as percentage of thermal rating, one per branch.
    pub branch_loading_pct: Vec<f64>,
    /// Generator MW output, one per generator.
    pub generation_mw: Vec<f64>,
    /// Active SCADA alarms.
    pub alarms: Vec<Alarm>,
}

// ─── DebriefReport ────────────────────────────────────────────────────────────

/// End-of-session debrief analysis report.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DebriefReport {
    /// Overall session score in \[0, 100\].
    pub overall_score: f64,
    /// Average response latency across all recorded actions \[s\].
    pub response_time_avg_s: f64,
    /// Fraction of correct actions \[0–1\].
    pub correct_actions_pct: f64,
    /// Number of KPIs achieved.
    pub kpis_met: usize,
    /// Total number of KPIs in the scenario.
    pub kpis_total: usize,
    /// Descriptions of critical mistakes made.
    pub critical_errors: Vec<String>,
    /// Training recommendations for skill gaps.
    pub recommendations: Vec<String>,
    /// Competency domain scores: SystemAwareness, DecisionMaking,
    /// ProcedureAdherence, CommunicationSkills — each in \[0, 100\].
    pub competency_scores: HashMap<String, f64>,
}

// ─── TrainingSimSession internals ─────────────────────────────────────────────

/// Tracks frequency deviation state for first-order decay.
#[derive(Debug, Clone, Serialize, Deserialize)]
struct FreqState {
    delta: f64,
    start_time: f64,
}

/// Tracks per-bus voltage deviation for first-order decay.
#[derive(Debug, Clone, Serialize, Deserialize)]
struct VoltState {
    bus: usize,
    delta: f64,
    start_time: f64,
}

// ─── TrainingSimSession ───────────────────────────────────────────────────────

/// Main operator training simulator session.
///
/// Drives scenario playback, applies simplified system dynamics, records trainee
/// actions, and produces a [`DebriefReport`] at the end.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrainingSimSession {
    /// The training scenario being executed.
    pub scenario: TrainingScenario,
    /// Identifier of the trainee under evaluation.
    pub trainee_id: String,
    /// Chronological list of trainee responses.
    pub actions: Vec<TraineeResponse>,
    /// Current simplified system state.
    pub system_state: SystemState,
    /// Simulation time elapsed \[s\].
    pub elapsed_time: f64,
    /// Running session score.
    pub score: f64,
    /// Accumulated feedback strings.
    pub feedback: Vec<String>,

    // Private simulation state
    freq_state: Option<FreqState>,
    volt_states: Vec<VoltState>,
    events_triggered: Vec<usize>,
    next_alarm_id: usize,
    lcg_state: u64,
}

impl TrainingSimSession {
    /// Create a new training session.
    ///
    /// # Arguments
    /// * `scenario` — The scenario to execute.
    /// * `trainee_id` — Trainee identifier string.
    /// * `n_buses` — Number of buses for initialising voltage vector.
    /// * `n_branches` — Number of branches for branch loading vector.
    /// * `n_gens` — Number of generators for generation vector.
    pub fn new(
        scenario: TrainingScenario,
        trainee_id: String,
        n_buses: usize,
        n_branches: usize,
        n_gens: usize,
    ) -> Self {
        let system_state = SystemState {
            frequency_hz: 50.0,
            voltage_pu: vec![1.0; n_buses],
            branch_loading_pct: vec![60.0; n_branches],
            generation_mw: vec![100.0; n_gens],
            alarms: Vec::new(),
        };
        Self {
            scenario,
            trainee_id,
            actions: Vec::new(),
            system_state,
            elapsed_time: 0.0,
            score: 100.0,
            feedback: Vec::new(),
            freq_state: None,
            volt_states: Vec::new(),
            events_triggered: Vec::new(),
            next_alarm_id: 1,
            lcg_state: 12345678901234567u64,
        }
    }

    /// Advance simulation time by `dt` seconds.
    ///
    /// Returns all [`TrainingEvent`]s whose `trigger_time` falls in
    /// `[elapsed_time, elapsed_time + dt)`.  System dynamics are updated and
    /// alarms are raised for each triggered event.
    pub fn advance_time(&mut self, dt: f64) -> Vec<TrainingEvent> {
        let t0 = self.elapsed_time;
        let t1 = t0 + dt;

        // Collect indices and clones of events to trigger (avoids borrow conflict).
        let to_trigger: Vec<(usize, TrainingEvent)> = self
            .scenario
            .events
            .iter()
            .enumerate()
            .filter(|(idx, ev)| {
                ev.trigger_time >= t0
                    && ev.trigger_time < t1
                    && !self.events_triggered.contains(idx)
            })
            .map(|(idx, ev)| (idx, ev.clone()))
            .collect();

        let mut triggered = Vec::new();
        for (idx, event) in to_trigger {
            self.events_triggered.push(idx);
            self.apply_event_dynamics(&event, t1);
            triggered.push(event);
        }

        // Update continuous dynamics at new elapsed time.
        self.elapsed_time = t1;
        self.update_dynamics(t1);

        triggered
    }

    /// Apply system dynamics when an event fires.
    fn apply_event_dynamics(&mut self, event: &TrainingEvent, t: f64) {
        match &event.event_type {
            TrainingEventType::GeneratorTrip { mw, bus } => {
                let delta = -(*mw) / 1000.0 * 2.0; // simplified: 2 Hz per 1000 MW loss
                self.freq_state = Some(FreqState {
                    delta,
                    start_time: t,
                });
                self.raise_alarm(
                    AlarmSeverity::Critical,
                    format!("Generator trip at bus {} ({} MW)", bus, mw),
                    t,
                );
            }
            TrainingEventType::FrequencyDeviation { delta_hz } => {
                self.freq_state = Some(FreqState {
                    delta: *delta_hz,
                    start_time: t,
                });
                let sev = if delta_hz.abs() > 0.5 {
                    AlarmSeverity::Critical
                } else {
                    AlarmSeverity::Major
                };
                self.raise_alarm(sev, format!("Frequency deviation: {:+.3} Hz", delta_hz), t);
            }
            TrainingEventType::LineTrip { from, to } => {
                self.raise_alarm(
                    AlarmSeverity::Major,
                    format!("Line trip: bus {} — bus {}", from, to),
                    t,
                );
            }
            TrainingEventType::VoltageDeviation { bus, delta_pu } => {
                self.volt_states.push(VoltState {
                    bus: *bus,
                    delta: *delta_pu,
                    start_time: t,
                });
                self.raise_alarm(
                    AlarmSeverity::Major,
                    format!("Voltage deviation at bus {}: {:+.3} pu", bus, delta_pu),
                    t,
                );
            }
            TrainingEventType::LoadIncrease { bus, delta_mw } => {
                self.raise_alarm(
                    AlarmSeverity::Minor,
                    format!("Load increase at bus {}: +{:.1} MW", bus, delta_mw),
                    t,
                );
            }
            TrainingEventType::CommunicationFailure { duration_s } => {
                self.raise_alarm(
                    AlarmSeverity::Info,
                    format!("Communication failure for {:.1} s", duration_s),
                    t,
                );
            }
            TrainingEventType::ManualTrigger { description } => {
                self.raise_alarm(
                    AlarmSeverity::Info,
                    format!("Instructor event: {}", description),
                    t,
                );
            }
        }
    }

    /// Update decaying state variables.
    fn update_dynamics(&mut self, t: f64) {
        // Frequency decay.
        let freq = if let Some(ref fs) = self.freq_state {
            let elapsed = (t - fs.start_time).max(0.0);
            50.0 + fs.delta * (-elapsed / 10.0_f64).exp()
        } else {
            50.0
        };
        self.system_state.frequency_hz = freq;

        // Voltage decay per bus.
        for vs in &self.volt_states {
            let elapsed = (t - vs.start_time).max(0.0);
            let dv = vs.delta * (-elapsed / 5.0_f64).exp();
            if let Some(v) = self.system_state.voltage_pu.get_mut(vs.bus) {
                *v = 1.0 + dv;
            }
        }
    }

    /// Raise a new alarm.
    fn raise_alarm(&mut self, severity: AlarmSeverity, message: String, time: f64) {
        let id = self.next_alarm_id;
        self.next_alarm_id += 1;
        self.system_state.alarms.push(Alarm {
            id,
            severity,
            message,
            time,
            acknowledged: false,
        });
    }

    /// LCG step — advances internal RNG state.
    #[allow(dead_code)]
    fn lcg_next(&mut self) -> u64 {
        self.lcg_state = self
            .lcg_state
            .wrapping_mul(6364136223846793005u64)
            .wrapping_add(1442695040888963407u64);
        self.lcg_state
    }

    /// Record and evaluate a trainee action.
    ///
    /// Alarm acknowledgements are processed immediately; scoring and feedback are
    /// updated.
    pub fn record_action(&mut self, action: TraineeResponse) {
        // Handle alarm acknowledgement.
        if let TraineeActionType::AcknowledgeAlarm { alarm_id } = action.action_type {
            for alarm in &mut self.system_state.alarms {
                if alarm.id == alarm_id {
                    alarm.acknowledged = true;
                }
            }
        }
        // Penalise incorrect actions.
        if !action.is_correct {
            self.score = (self.score - 10.0).max(0.0);
            self.feedback.push("Incorrect action recorded.".to_string());
        }
        self.actions.push(action);
    }

    /// Return unacknowledged alarms sorted by descending severity (Critical first).
    pub fn get_active_alarms(&self) -> Vec<&Alarm> {
        let mut active: Vec<&Alarm> = self
            .system_state
            .alarms
            .iter()
            .filter(|a| !a.acknowledged)
            .collect();
        active.sort_by_key(|b| std::cmp::Reverse(b.severity));
        active
    }

    /// Evaluate a candidate action given an optional triggering event.
    ///
    /// Returns `(correct: bool, partial_score: f64, feedback: String)`.
    pub fn evaluate_action(
        &self,
        action: &TraineeResponse,
        triggering_event: Option<&TrainingEvent>,
    ) -> (bool, f64, String) {
        let Some(event) = triggering_event else {
            // No event context — give partial credit for proactive actions.
            let score = match &action.action_type {
                TraineeActionType::AcknowledgeAlarm { .. } => 30.0,
                TraineeActionType::RequestHelp => 20.0,
                _ => 10.0,
            };
            return (false, score, "No triggering event context.".to_string());
        };

        let (correct, score, msg) = match (&event.event_type, &action.action_type) {
            // Generator trip → activate reserve or dispatch generator
            (
                TrainingEventType::GeneratorTrip { mw, .. },
                TraineeActionType::ActivateReserve { reserve_mw },
            ) => {
                let needed = mw;
                let pct = (reserve_mw / needed).min(1.0);
                let s = 60.0 + pct * 40.0;
                (
                    pct >= 0.8,
                    s,
                    format!("Reserve {:.1} MW vs needed {:.1} MW.", reserve_mw, needed),
                )
            }
            (
                TrainingEventType::GeneratorTrip { mw, bus },
                TraineeActionType::DispatchGenerator { bus: dbus, mw: dmw },
            ) => {
                let correct = *dbus != *bus && *dmw >= mw * 0.5;
                let s = if correct { 80.0 } else { 40.0 };
                (
                    correct,
                    s,
                    "Generator re-dispatch for trip compensation.".to_string(),
                )
            }
            // Line trip → close breaker or island section
            (
                TrainingEventType::LineTrip { from, to },
                TraineeActionType::CloseBreaker { from: cf, to: ct },
            ) => {
                let correct = cf == from && ct == to;
                (
                    correct,
                    if correct { 90.0 } else { 30.0 },
                    "Breaker re-close attempt.".to_string(),
                )
            }
            (TrainingEventType::LineTrip { .. }, TraineeActionType::IslandSection { .. }) => {
                (true, 70.0, "Islanding applied after line trip.".to_string())
            }
            // Frequency deviation → activate reserve
            (
                TrainingEventType::FrequencyDeviation { delta_hz },
                TraineeActionType::ActivateReserve { reserve_mw },
            ) => {
                let needed = delta_hz.abs() * 200.0; // simplified
                let pct = (reserve_mw / needed.max(1.0)).min(1.0);
                let s = 50.0 + pct * 50.0;
                (
                    pct >= 0.5,
                    s,
                    format!("Reserve {:.1} MW for freq deviation.", reserve_mw),
                )
            }
            // Voltage deviation → tap changer
            (TrainingEventType::VoltageDeviation { .. }, TraineeActionType::TapChanger { .. }) => (
                true,
                80.0,
                "Tap changer applied for voltage control.".to_string(),
            ),
            // Alarm acknowledgement is always correct
            (_, TraineeActionType::AcknowledgeAlarm { .. }) => {
                (true, 40.0, "Alarm acknowledged.".to_string())
            }
            // Request help — partial credit
            (_, TraineeActionType::RequestHelp) => (
                false,
                20.0,
                "Help requested; independent action preferred.".to_string(),
            ),
            _ => (
                false,
                10.0,
                "Action does not match triggering event.".to_string(),
            ),
        };

        (correct, score, msg)
    }

    /// Evaluate KPIs against current system state and return met/total counts.
    fn evaluate_kpis(&self) -> (usize, usize) {
        let mut met = 0usize;
        let total = self.scenario.target_kpis.len();
        for kpi in &self.scenario.target_kpis {
            let achieved = match kpi.name.to_lowercase().as_str() {
                s if s.contains("frequency") => {
                    (self.system_state.frequency_hz - kpi.target_value).abs() <= kpi.tolerance
                }
                s if s.contains("voltage") => self
                    .system_state
                    .voltage_pu
                    .iter()
                    .all(|&v| (v - kpi.target_value).abs() <= kpi.tolerance),
                _ => {
                    // Generic: check frequency or voltage depending on target
                    (self.system_state.frequency_hz - kpi.target_value).abs() <= kpi.tolerance
                }
            };
            if achieved {
                met += 1;
            }
        }
        (met, total)
    }

    /// Generate the end-of-session debrief report.
    pub fn generate_debrief(&self) -> DebriefReport {
        let n_actions = self.actions.len();
        let correct_count = self.actions.iter().filter(|a| a.is_correct).count();
        let correct_pct = if n_actions > 0 {
            correct_count as f64 / n_actions as f64
        } else {
            0.0
        };
        let avg_latency = if n_actions > 0 {
            self.actions
                .iter()
                .map(|a| a.response_latency_s)
                .sum::<f64>()
                / n_actions as f64
        } else {
            0.0
        };

        let (kpis_met, kpis_total) = self.evaluate_kpis();

        // Compute base score applying rules.
        let mut base = 100.0_f64;
        // Deduct for incorrect actions.
        let incorrect = n_actions - correct_count;
        base -= incorrect as f64 * 10.0;
        // Deduct for unacknowledged alarms.
        let unack = self
            .system_state
            .alarms
            .iter()
            .filter(|a| !a.acknowledged)
            .count();
        base -= unack as f64 * 5.0;
        // Bonus for fast critical responses.
        let fast_critical = self
            .actions
            .iter()
            .filter(|a| a.is_correct && a.response_latency_s < 30.0)
            .count();
        base += fast_critical as f64 * 5.0;
        // KPI contribution.
        if kpis_total > 0 {
            base += (kpis_met as f64 / kpis_total as f64) * 30.0;
        }
        let overall_score = base.clamp(0.0, 100.0);

        // Identify critical errors.
        let mut critical_errors = Vec::new();
        for action in &self.actions {
            if !action.is_correct {
                if let TraineeActionType::TripBreaker { from, to } = &action.action_type {
                    critical_errors.push(format!(
                        "Unnecessary breaker trip: bus {}–{} at t={:.1}s",
                        from, to, action.timestamp
                    ));
                }
            }
        }
        if unack > 0 {
            critical_errors.push(format!("{} alarms left unacknowledged.", unack));
        }

        // Build recommendations.
        let mut recommendations = Vec::new();
        if correct_pct < 0.5 {
            recommendations.push("Review emergency response procedures.".to_string());
        }
        if avg_latency > 60.0 {
            recommendations.push("Practice faster alarm-to-action response.".to_string());
        }
        if unack > 2 {
            recommendations.push("Improve alarm management and prioritisation.".to_string());
        }
        if kpis_total > 0 && kpis_met < kpis_total / 2 {
            recommendations.push("Review KPI targets for this scenario type.".to_string());
        }
        if recommendations.is_empty() {
            recommendations
                .push("Performance meets requirements. Continue advanced scenarios.".to_string());
        }

        DebriefReport {
            overall_score,
            response_time_avg_s: avg_latency,
            correct_actions_pct: correct_pct,
            kpis_met,
            kpis_total,
            critical_errors,
            recommendations,
            competency_scores: self.compute_competency_scores(),
        }
    }

    /// Compute competency domain scores.
    ///
    /// Returns a map with keys: `"SystemAwareness"`, `"DecisionMaking"`,
    /// `"ProcedureAdherence"`, `"CommunicationSkills"` — each in \[0, 100\].
    pub fn compute_competency_scores(&self) -> HashMap<String, f64> {
        let n_actions = self.actions.len();
        let n_events = self.events_triggered.len().max(1);

        // SystemAwareness: proportion of triggered events where an action was taken.
        let responded = self
            .actions
            .iter()
            .filter(|a| a.response_latency_s < 120.0)
            .count();
        let system_awareness = ((responded as f64 / n_events as f64) * 100.0).min(100.0);

        // DecisionMaking: correct action rate.
        let correct_count = self.actions.iter().filter(|a| a.is_correct).count();
        let decision_making = if n_actions > 0 {
            (correct_count as f64 / n_actions as f64 * 100.0).min(100.0)
        } else {
            50.0 // neutral when no actions taken
        };

        // ProcedureAdherence: penalised for TripBreaker errors and RequestHelp.
        let violations = self
            .actions
            .iter()
            .filter(|a| {
                !a.is_correct && matches!(a.action_type, TraineeActionType::TripBreaker { .. })
            })
            .count();
        let help_requests = self
            .actions
            .iter()
            .filter(|a| matches!(a.action_type, TraineeActionType::RequestHelp))
            .count();
        let procedure_adherence =
            (100.0 - violations as f64 * 20.0 - help_requests as f64 * 5.0).clamp(0.0, 100.0);

        // CommunicationSkills: alarm acknowledgement rate.
        let total_alarms = self.system_state.alarms.len();
        let acked = self
            .system_state
            .alarms
            .iter()
            .filter(|a| a.acknowledged)
            .count();
        let communication = if total_alarms > 0 {
            (acked as f64 / total_alarms as f64 * 100.0).min(100.0)
        } else {
            80.0 // no alarms → neutral good score
        };

        let mut map = HashMap::new();
        map.insert("SystemAwareness".to_string(), system_awareness);
        map.insert("DecisionMaking".to_string(), decision_making);
        map.insert("ProcedureAdherence".to_string(), procedure_adherence);
        map.insert("CommunicationSkills".to_string(), communication);
        map
    }
}

// ─── ScenarioLibrary ──────────────────────────────────────────────────────────

/// Pre-built training scenario library.
pub struct ScenarioLibrary;

impl ScenarioLibrary {
    /// N-1 scenario: single generator trip, trainee must activate reserve.
    pub fn create_n1_scenario() -> TrainingScenario {
        TrainingScenario {
            name: "N-1 Generator Trip".to_string(),
            description: "A 200 MW generator trips at t=30s. The trainee must activate \
                          spinning reserve and restore frequency within limits."
                .to_string(),
            duration_seconds: 300.0,
            events: vec![TrainingEvent {
                trigger_time: 30.0,
                event_type: TrainingEventType::GeneratorTrip { bus: 2, mw: 200.0 },
            }],
            target_kpis: vec![KpiTarget {
                name: "Frequency recovery".to_string(),
                target_value: 50.0,
                tolerance: 0.2,
                achieved: false,
            }],
        }
    }

    /// Cascading failure scenario: multiple sequential events.
    pub fn create_cascading_failure_scenario() -> TrainingScenario {
        TrainingScenario {
            name: "Cascading Failure".to_string(),
            description: "A generator trip triggers line overloads leading to cascade outage. \
                          Trainee must prevent propagation."
                .to_string(),
            duration_seconds: 600.0,
            events: vec![
                TrainingEvent {
                    trigger_time: 20.0,
                    event_type: TrainingEventType::GeneratorTrip { bus: 1, mw: 300.0 },
                },
                TrainingEvent {
                    trigger_time: 35.0,
                    event_type: TrainingEventType::LineTrip { from: 1, to: 3 },
                },
                TrainingEvent {
                    trigger_time: 50.0,
                    event_type: TrainingEventType::FrequencyDeviation { delta_hz: -0.8 },
                },
                TrainingEvent {
                    trigger_time: 70.0,
                    event_type: TrainingEventType::VoltageDeviation {
                        bus: 3,
                        delta_pu: -0.12,
                    },
                },
            ],
            target_kpis: vec![
                KpiTarget {
                    name: "Frequency recovery".to_string(),
                    target_value: 50.0,
                    tolerance: 0.3,
                    achieved: false,
                },
                KpiTarget {
                    name: "Voltage restoration".to_string(),
                    target_value: 1.0,
                    tolerance: 0.05,
                    achieved: false,
                },
            ],
        }
    }

    /// Voltage collapse scenario: progressive voltage decay.
    pub fn create_voltage_collapse_scenario() -> TrainingScenario {
        TrainingScenario {
            name: "Voltage Collapse Prevention".to_string(),
            description: "Reactive power deficit causes progressive voltage decline. \
                          Trainee must apply voltage support before collapse."
                .to_string(),
            duration_seconds: 480.0,
            events: vec![
                TrainingEvent {
                    trigger_time: 15.0,
                    event_type: TrainingEventType::VoltageDeviation {
                        bus: 0,
                        delta_pu: -0.05,
                    },
                },
                TrainingEvent {
                    trigger_time: 60.0,
                    event_type: TrainingEventType::VoltageDeviation {
                        bus: 1,
                        delta_pu: -0.08,
                    },
                },
                TrainingEvent {
                    trigger_time: 120.0,
                    event_type: TrainingEventType::VoltageDeviation {
                        bus: 2,
                        delta_pu: -0.15,
                    },
                },
                TrainingEvent {
                    trigger_time: 200.0,
                    event_type: TrainingEventType::LoadIncrease {
                        bus: 1,
                        delta_mw: 50.0,
                    },
                },
            ],
            target_kpis: vec![KpiTarget {
                name: "Voltage restoration".to_string(),
                target_value: 1.0,
                tolerance: 0.1,
                achieved: false,
            }],
        }
    }

    /// Islanding scenario: line trips isolate a section.
    pub fn create_islanding_scenario() -> TrainingScenario {
        TrainingScenario {
            name: "Controlled Islanding".to_string(),
            description: "Multiple line trips risk uncontrolled islanding. \
                          Trainee must execute controlled islanding to preserve the island."
                .to_string(),
            duration_seconds: 400.0,
            events: vec![
                TrainingEvent {
                    trigger_time: 10.0,
                    event_type: TrainingEventType::LineTrip { from: 0, to: 2 },
                },
                TrainingEvent {
                    trigger_time: 25.0,
                    event_type: TrainingEventType::LineTrip { from: 2, to: 4 },
                },
                TrainingEvent {
                    trigger_time: 40.0,
                    event_type: TrainingEventType::FrequencyDeviation { delta_hz: -0.3 },
                },
            ],
            target_kpis: vec![KpiTarget {
                name: "Frequency recovery".to_string(),
                target_value: 50.0,
                tolerance: 0.5,
                achieved: false,
            }],
        }
    }

    /// Black-start restoration scenario.
    pub fn create_restoration_scenario() -> TrainingScenario {
        TrainingScenario {
            name: "Black Start Restoration".to_string(),
            description: "Full blackout — trainee executes cranking path, picks up loads \
                          in sequence, and synchronises sections."
                .to_string(),
            duration_seconds: 900.0,
            events: vec![
                TrainingEvent {
                    trigger_time: 0.0,
                    event_type: TrainingEventType::ManualTrigger {
                        description: "System blackout. Initiate black start from unit BS-1."
                            .to_string(),
                    },
                },
                TrainingEvent {
                    trigger_time: 120.0,
                    event_type: TrainingEventType::ManualTrigger {
                        description: "BS-1 online. Energise cranking path to G2.".to_string(),
                    },
                },
                TrainingEvent {
                    trigger_time: 240.0,
                    event_type: TrainingEventType::LoadIncrease {
                        bus: 0,
                        delta_mw: 50.0,
                    },
                },
                TrainingEvent {
                    trigger_time: 420.0,
                    event_type: TrainingEventType::LoadIncrease {
                        bus: 1,
                        delta_mw: 80.0,
                    },
                },
                TrainingEvent {
                    trigger_time: 600.0,
                    event_type: TrainingEventType::ManualTrigger {
                        description: "Synchronise island A to island B.".to_string(),
                    },
                },
            ],
            target_kpis: vec![
                KpiTarget {
                    name: "Frequency recovery".to_string(),
                    target_value: 50.0,
                    tolerance: 0.5,
                    achieved: false,
                },
                KpiTarget {
                    name: "Voltage restoration".to_string(),
                    target_value: 1.0,
                    tolerance: 0.1,
                    achieved: false,
                },
            ],
        }
    }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_session(scenario: TrainingScenario) -> TrainingSimSession {
        TrainingSimSession::new(scenario, "tester".to_string(), 5, 6, 3)
    }

    // 1. ScenarioLibrary creates valid scenarios
    #[test]
    fn test_scenario_creation() {
        let s = ScenarioLibrary::create_n1_scenario();
        assert!(!s.events.is_empty());
        assert!(!s.target_kpis.is_empty());
        assert!(s.duration_seconds > 0.0);
    }

    // 2. Session initialisation
    #[test]
    fn test_session_initialization() {
        let s = ScenarioLibrary::create_n1_scenario();
        let session = make_session(s);
        assert!((session.system_state.frequency_hz - 50.0).abs() < 1e-9);
        assert!(session
            .system_state
            .voltage_pu
            .iter()
            .all(|&v| (v - 1.0).abs() < 1e-9));
        assert_eq!(session.elapsed_time, 0.0);
    }

    // 3. Events fire at correct times
    #[test]
    fn test_advance_time_triggers_events() {
        let s = ScenarioLibrary::create_n1_scenario();
        let mut session = make_session(s);
        let before = session.advance_time(25.0);
        assert!(before.is_empty(), "no event before t=30");
        let at = session.advance_time(10.0); // t=35
        assert_eq!(at.len(), 1);
        // Should not fire again
        let after = session.advance_time(100.0);
        assert!(after.is_empty());
    }

    // 4. Correct action keeps score high
    #[test]
    fn test_record_correct_action() {
        let s = ScenarioLibrary::create_n1_scenario();
        let mut session = make_session(s);
        let before = session.score;
        session.record_action(TraineeResponse {
            timestamp: 31.0,
            action_type: TraineeActionType::ActivateReserve { reserve_mw: 200.0 },
            response_latency_s: 5.0,
            is_correct: true,
        });
        assert_eq!(
            session.score, before,
            "correct action should not reduce score"
        );
    }

    // 5. Incorrect action reduces score by 10
    #[test]
    fn test_record_incorrect_action() {
        let s = ScenarioLibrary::create_n1_scenario();
        let mut session = make_session(s);
        let before = session.score;
        session.record_action(TraineeResponse {
            timestamp: 31.0,
            action_type: TraineeActionType::TripBreaker { from: 0, to: 1 },
            response_latency_s: 5.0,
            is_correct: false,
        });
        assert!((session.score - (before - 10.0)).abs() < 1e-9);
    }

    // 6. Alarms generated and acknowledged
    #[test]
    fn test_alarm_management() {
        let s = ScenarioLibrary::create_n1_scenario();
        let mut session = make_session(s);
        session.advance_time(35.0); // triggers GeneratorTrip → Critical alarm
        assert!(!session.system_state.alarms.is_empty());
        let alarm_id = session.system_state.alarms[0].id;
        session.record_action(TraineeResponse {
            timestamp: 36.0,
            action_type: TraineeActionType::AcknowledgeAlarm { alarm_id },
            response_latency_s: 1.0,
            is_correct: true,
        });
        assert!(session.system_state.alarms[0].acknowledged);
    }

    // 7. Frequency dynamics after deviation event
    #[test]
    fn test_frequency_dynamics() {
        let scenario = TrainingScenario {
            name: "FreqTest".to_string(),
            description: "freq test".to_string(),
            duration_seconds: 120.0,
            events: vec![TrainingEvent {
                trigger_time: 5.0,
                event_type: TrainingEventType::FrequencyDeviation { delta_hz: -1.0 },
            }],
            target_kpis: vec![],
        };
        let mut session = make_session(scenario);
        session.advance_time(10.0);
        assert!(
            (session.system_state.frequency_hz - 50.0).abs() > 0.01,
            "frequency should deviate from 50 Hz after event"
        );
    }

    // 8. Debrief report fields are populated
    #[test]
    fn test_debrief_report() {
        let s = ScenarioLibrary::create_n1_scenario();
        let mut session = make_session(s);
        session.advance_time(35.0);
        session.record_action(TraineeResponse {
            timestamp: 36.0,
            action_type: TraineeActionType::ActivateReserve { reserve_mw: 200.0 },
            response_latency_s: 6.0,
            is_correct: true,
        });
        let report = session.generate_debrief();
        assert!(report.overall_score >= 0.0 && report.overall_score <= 100.0);
        assert_eq!(report.kpis_total, 1);
        assert!(!report.recommendations.is_empty());
        assert_eq!(report.competency_scores.len(), 4);
    }

    // 9. All 4 competencies computed
    #[test]
    fn test_competency_scores() {
        let s = ScenarioLibrary::create_n1_scenario();
        let session = make_session(s);
        let scores = session.compute_competency_scores();
        assert!(scores.contains_key("SystemAwareness"));
        assert!(scores.contains_key("DecisionMaking"));
        assert!(scores.contains_key("ProcedureAdherence"));
        assert!(scores.contains_key("CommunicationSkills"));
        for v in scores.values() {
            assert!(*v >= 0.0 && *v <= 100.0);
        }
    }

    // 10. N-1 scenario has exactly 1 trip event
    #[test]
    fn test_n1_scenario() {
        let s = ScenarioLibrary::create_n1_scenario();
        let trips = s
            .events
            .iter()
            .filter(|e| matches!(e.event_type, TrainingEventType::GeneratorTrip { .. }))
            .count();
        assert_eq!(trips, 1);
    }

    // 11. Cascading scenario has >= 3 events
    #[test]
    fn test_cascading_scenario() {
        let s = ScenarioLibrary::create_cascading_failure_scenario();
        assert!(s.events.len() >= 3);
    }

    // 12. Fast response for critical event gives bonus
    #[test]
    fn test_response_latency_scoring() {
        let s = ScenarioLibrary::create_n1_scenario();
        let mut fast_session = make_session(s.clone());
        fast_session.advance_time(35.0);
        fast_session.record_action(TraineeResponse {
            timestamp: 31.0,
            action_type: TraineeActionType::ActivateReserve { reserve_mw: 200.0 },
            response_latency_s: 5.0, // < 30s
            is_correct: true,
        });
        let fast_report = fast_session.generate_debrief();

        let mut slow_session = make_session(s);
        slow_session.advance_time(35.0);
        slow_session.record_action(TraineeResponse {
            timestamp: 90.0,
            action_type: TraineeActionType::ActivateReserve { reserve_mw: 200.0 },
            response_latency_s: 60.0, // >= 30s
            is_correct: true,
        });
        let slow_report = slow_session.generate_debrief();

        assert!(
            fast_report.overall_score >= slow_report.overall_score,
            "fast response should score >= slow: {} vs {}",
            fast_report.overall_score,
            slow_report.overall_score
        );
    }

    // 13. KPI evaluation
    #[test]
    fn test_kpi_evaluation() {
        let s = ScenarioLibrary::create_n1_scenario();
        let session = make_session(s);
        // No deviation → frequency KPI met
        let report = session.generate_debrief();
        assert_eq!(report.kpis_total, 1);
        // frequency is 50.0, target 50.0, tol 0.2 → met
        assert_eq!(report.kpis_met, 1);
    }

    // 14. evaluate_action returns correct bool
    #[test]
    fn test_action_evaluation() {
        let s = ScenarioLibrary::create_n1_scenario();
        let session = make_session(s);
        let event = TrainingEvent {
            trigger_time: 30.0,
            event_type: TrainingEventType::GeneratorTrip { bus: 2, mw: 200.0 },
        };
        let action = TraineeResponse {
            timestamp: 35.0,
            action_type: TraineeActionType::ActivateReserve { reserve_mw: 200.0 },
            response_latency_s: 5.0,
            is_correct: true,
        };
        let (correct, score, msg) = session.evaluate_action(&action, Some(&event));
        assert!(correct);
        assert!(score > 0.0);
        assert!(!msg.is_empty());
    }

    // 15. Active alarms sorted: Critical first
    #[test]
    fn test_active_alarms_sorted() {
        let s = ScenarioLibrary::create_n1_scenario();
        let mut session = make_session(s);
        // Manually add alarms in wrong order.
        session.system_state.alarms.push(Alarm {
            id: 100,
            severity: AlarmSeverity::Minor,
            message: "minor".to_string(),
            time: 1.0,
            acknowledged: false,
        });
        session.system_state.alarms.push(Alarm {
            id: 101,
            severity: AlarmSeverity::Critical,
            message: "critical".to_string(),
            time: 2.0,
            acknowledged: false,
        });
        let active = session.get_active_alarms();
        assert_eq!(active[0].severity, AlarmSeverity::Critical);
    }

    // 16. Score stays in [0, 100]
    #[test]
    fn test_score_bounds() {
        let s = ScenarioLibrary::create_n1_scenario();
        let mut session = make_session(s);
        // Record many incorrect actions.
        for i in 0..20 {
            session.record_action(TraineeResponse {
                timestamp: i as f64,
                action_type: TraineeActionType::RequestHelp,
                response_latency_s: 99.0,
                is_correct: false,
            });
        }
        assert!(session.score >= 0.0);
        let report = session.generate_debrief();
        assert!(report.overall_score >= 0.0 && report.overall_score <= 100.0);
    }

    // 17. All 5 ScenarioLibrary scenarios have events
    #[test]
    fn test_scenario_library_all() {
        let scenarios = [
            ScenarioLibrary::create_n1_scenario(),
            ScenarioLibrary::create_cascading_failure_scenario(),
            ScenarioLibrary::create_voltage_collapse_scenario(),
            ScenarioLibrary::create_islanding_scenario(),
            ScenarioLibrary::create_restoration_scenario(),
        ];
        for s in &scenarios {
            assert!(!s.events.is_empty(), "scenario '{}' has no events", s.name);
        }
    }

    // 18. Debrief with no actions returns valid report
    #[test]
    fn test_empty_session() {
        let s = ScenarioLibrary::create_n1_scenario();
        let session = make_session(s);
        let report = session.generate_debrief();
        assert!(report.overall_score >= 0.0 && report.overall_score <= 100.0);
        assert_eq!(report.correct_actions_pct, 0.0);
        assert_eq!(report.response_time_avg_s, 0.0);
    }

    // 19. Multiple events at same trigger_time all returned
    #[test]
    fn test_multiple_events_same_time() {
        let scenario = TrainingScenario {
            name: "SimultaneousEvents".to_string(),
            description: "Two events at t=10".to_string(),
            duration_seconds: 100.0,
            events: vec![
                TrainingEvent {
                    trigger_time: 10.0,
                    event_type: TrainingEventType::FrequencyDeviation { delta_hz: -0.5 },
                },
                TrainingEvent {
                    trigger_time: 10.0,
                    event_type: TrainingEventType::VoltageDeviation {
                        bus: 0,
                        delta_pu: -0.05,
                    },
                },
            ],
            target_kpis: vec![],
        };
        let mut session = make_session(scenario);
        let fired = session.advance_time(15.0);
        assert_eq!(fired.len(), 2, "both simultaneous events should fire");
    }

    // 20. Restoration scenario has ManualTrigger or LoadIncrease events
    #[test]
    fn test_restoration_scenario() {
        let s = ScenarioLibrary::create_restoration_scenario();
        let has_restore_events = s.events.iter().any(|e| {
            matches!(
                &e.event_type,
                TrainingEventType::ManualTrigger { .. } | TrainingEventType::LoadIncrease { .. }
            )
        });
        assert!(
            has_restore_events,
            "restoration scenario must have restore-type events"
        );
    }
}
