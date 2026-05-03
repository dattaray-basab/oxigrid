//! Power System Operator Training Simulator (OTS).
//!
//! Provides scenario playback, decision grading, emergency response training,
//! and competency assessment for power grid operators.
//!
//! # Quick Start
//!
//! ```rust
//! use oxigrid::simulation::operator_training::{TrainingSession, ScenarioType};
//!
//! let mut session = TrainingSession::generator_trip_scenario("trainee_001".to_string());
//! let events = session.advance_time(35.0);
//! assert!(!events.is_empty(), "event should fire within 35 s");
//! ```

use serde::{Deserialize, Serialize};
use std::fmt;

// ─── Scenario type ────────────────────────────────────────────────────────────

/// Category of training scenario.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ScenarioType {
    /// Routine operations with no disturbance.
    Normal,
    /// Sudden generation loss → frequency drop.
    EmergencyFrequency,
    /// Reactive power deficit → voltage collapse risk.
    EmergencyVoltage,
    /// N-1 contingency management.
    NMinusOneClearance,
    /// System restoration from blackout (black-start).
    BlackStart,
    /// Prevent cascading outage.
    CascadePrevent,
    /// Post-fault transient stability management.
    TransientStability,
    /// Anomalous SCADA readings (cyber incident).
    CyberIncident,
}

impl fmt::Display for ScenarioType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            Self::Normal => "Normal Operations",
            Self::EmergencyFrequency => "Emergency – Frequency",
            Self::EmergencyVoltage => "Emergency – Voltage",
            Self::NMinusOneClearance => "N-1 Contingency Clearance",
            Self::BlackStart => "Black Start",
            Self::CascadePrevent => "Cascade Prevention",
            Self::TransientStability => "Transient Stability",
            Self::CyberIncident => "Cyber Incident",
        };
        f.write_str(s)
    }
}

// ─── Event type ───────────────────────────────────────────────────────────────

/// Stimulus injected into a training scenario.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum OtsEventType {
    /// A generator has tripped offline.
    GeneratorTrip {
        /// Identifier of the lost unit.
        unit_id: String,
        /// Lost capacity \[MW\].
        capacity_mw: f64,
    },
    /// A transmission or distribution line has tripped.
    LineTrip {
        /// Identifier of the disconnected branch.
        branch_id: String,
    },
    /// A sudden load change at a bus.
    LoadStep {
        /// Bus index (0-based).
        bus_id: usize,
        /// Load increase (positive) or decrease (negative) \[MW\].
        delta_mw: f64,
    },
    /// Frequency has left the normal band.
    FrequencyAlert {
        /// Measured system frequency \[Hz\].
        frequency_hz: f64,
    },
    /// Bus voltage has left limits.
    VoltageAlert {
        /// Bus index (0-based).
        bus_id: usize,
        /// Measured bus voltage \[pu\].
        voltage_pu: f64,
    },
    /// A protection relay has operated.
    ProtectionOperation {
        /// Relay identifier.
        relay_id: String,
    },
    /// A fault has been cleared by protection.
    FaultCleared {
        /// Location description.
        location: String,
    },
    /// Instructor-issued narrative trigger.
    ManualTrigger {
        /// Instruction text shown to trainee.
        instruction: String,
    },
}

// ─── Operator action ──────────────────────────────────────────────────────────

/// Action a trainee (or the correct answer key) can submit.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum OtsOperatorAction {
    /// Bring an offline unit online.
    CommitUnit {
        /// Unit identifier.
        unit_id: String,
    },
    /// Take a unit offline.
    DecommitUnit {
        /// Unit identifier.
        unit_id: String,
    },
    /// Adjust MW output of a running unit.
    AdjustGeneration {
        /// Unit identifier.
        unit_id: String,
        /// New or incremental MW setpoint \[MW\].
        mw: f64,
    },
    /// Open a network switch or breaker.
    OpenSwitch {
        /// Switch identifier.
        switch_id: String,
    },
    /// Close a network switch or breaker.
    CloseSwitch {
        /// Switch identifier.
        switch_id: String,
    },
    /// Activate a spinning or non-spinning reserve product.
    ActivateReserve {
        /// Reserve product name.
        reserve_type: String,
    },
    /// Broadcast an alert to relevant parties.
    IssueAlert {
        /// Alert category.
        alert_type: String,
    },
    /// Cross-check readings before acting (correct answer for CyberIncident).
    VerifyData,
    /// Deliberately take no action (sometimes the correct response).
    DoNothing,
}

impl OtsOperatorAction {
    /// Returns `true` if the action is potentially dangerous when applied
    /// to a frequency-emergency scenario.
    fn is_dangerous_for_frequency_emergency(&self) -> bool {
        matches!(self, Self::DecommitUnit { .. } | Self::OpenSwitch { .. })
    }

    /// Human-readable label.
    fn label(&self) -> String {
        match self {
            Self::CommitUnit { unit_id } => format!("CommitUnit({})", unit_id),
            Self::DecommitUnit { unit_id } => format!("DecommitUnit({})", unit_id),
            Self::AdjustGeneration { unit_id, mw } => {
                format!("AdjustGeneration({}, {:.1} MW)", unit_id, mw)
            }
            Self::OpenSwitch { switch_id } => format!("OpenSwitch({})", switch_id),
            Self::CloseSwitch { switch_id } => format!("CloseSwitch({})", switch_id),
            Self::ActivateReserve { reserve_type } => {
                format!("ActivateReserve({})", reserve_type)
            }
            Self::IssueAlert { alert_type } => format!("IssueAlert({})", alert_type),
            Self::VerifyData => "VerifyData".to_string(),
            Self::DoNothing => "DoNothing".to_string(),
        }
    }
}

// ─── Correct action key ───────────────────────────────────────────────────────

/// Model answer for a scenario event.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CorrectAction {
    /// The action the trainee should take.
    pub action_type: OtsOperatorAction,
    /// Human-readable target description.
    pub target: String,
    /// Optional numerical value (setpoint, MW, etc.) \[unit depends on action\].
    pub value: Option<f64>,
    /// Why this is the correct action.
    pub rationale: String,
}

// ─── Scenario event ───────────────────────────────────────────────────────────

/// A discrete event injected into a training scenario.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScenarioEvent {
    /// Simulation time at which the event fires \[s\].
    pub time_s: f64,
    /// What kind of stimulus is injected.
    pub event_type: OtsEventType,
    /// Narrative description shown to the trainee.
    pub description: String,
    /// Whether the trainee must submit an action for this event.
    pub requires_action: bool,
    /// Model answer (used for scoring).
    pub correct_action: CorrectAction,
    /// Maximum time to respond for full credit \[s\].
    pub time_limit_s: f64,
}

// ─── System snapshot ──────────────────────────────────────────────────────────

/// First-order approximation of power system state at a moment in time.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SystemSnapshot {
    /// Bus voltage magnitudes \[pu\].
    pub bus_voltages_pu: Vec<f64>,
    /// Generator MW dispatch \[MW\].
    pub generation_mw: Vec<f64>,
    /// Load MW at each bus \[MW\].
    pub load_mw: Vec<f64>,
    /// System frequency \[Hz\].
    pub frequency_hz: f64,
    /// Branch loading as percentage of thermal rating \[%\].
    pub branch_loading_pct: Vec<f64>,
}

impl SystemSnapshot {
    /// Construct a nominal-operating snapshot for `n_bus` buses.
    pub fn nominal(n_bus: usize) -> Self {
        Self {
            bus_voltages_pu: vec![1.0; n_bus],
            generation_mw: vec![100.0; n_bus],
            load_mw: vec![80.0; n_bus],
            frequency_hz: 50.0,
            branch_loading_pct: vec![60.0; n_bus.saturating_sub(1)],
        }
    }
}

// ─── Trainee action ───────────────────────────────────────────────────────────

/// An action recorded from the trainee during a session.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TraineeAction {
    /// Simulation time when action was submitted \[s\].
    pub timestamp_s: f64,
    /// The action taken.
    pub action: OtsOperatorAction,
    /// Target element identifier or name.
    pub target: String,
    /// Optional numerical setpoint \[unit depends on action\].
    pub value: Option<f64>,
    /// Index of the scenario event this action addresses (if any).
    pub event_idx: Option<usize>,
    /// Whether a hint was used before this action.
    pub hint_used: bool,
}

// ─── Scoring method ───────────────────────────────────────────────────────────

/// How the OTS should compute event scores.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ScoringMethod {
    /// Faster correct responses earn more points.
    TimeBased,
    /// Only correctness matters; timing is ignored.
    AccuracyOnly,
    /// Weighted combination of time and accuracy sub-scores.
    Weighted {
        /// Weight given to response speed (0–1).
        time_weight: f64,
        /// Weight given to action correctness (0–1).
        accuracy_weight: f64,
    },
}

// ─── OTS configuration ────────────────────────────────────────────────────────

/// Configuration knobs for a training session.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OtsConfig {
    /// Ratio of simulated time to wall-clock time (1.0 = real-time, >1 = accelerated).
    pub real_time_factor: f64,
    /// Whether the trainee may request hints.
    pub hints_enabled: bool,
    /// Algorithm used to compute event scores.
    pub scoring_method: ScoringMethod,
    /// Minimum overall score to pass the session (0–100).
    pub minimum_pass_score: f64,
}

impl Default for OtsConfig {
    fn default() -> Self {
        Self {
            real_time_factor: 1.0,
            hints_enabled: true,
            scoring_method: ScoringMethod::Weighted {
                time_weight: 0.3,
                accuracy_weight: 0.7,
            },
            minimum_pass_score: 70.0,
        }
    }
}

// ─── Feedback types ───────────────────────────────────────────────────────────

/// Immediate feedback returned after [`TrainingSession::submit_action`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActionFeedback {
    /// Whether the action exactly matches the model answer.
    pub correct: bool,
    /// Partial credit in \[0, 100\].
    pub partial_credit: f64,
    /// Explanation of the grading decision.
    pub explanation: String,
    /// Wall-clock seconds elapsed since the event fired \[s\].
    pub time_taken_s: f64,
}

/// Detailed score for a single event response.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResponseScore {
    /// Points earned for this event.
    pub points: f64,
    /// Maximum possible points for this event (before bonuses).
    pub max_points: f64,
    /// Additional points awarded for responding within the time limit.
    pub time_bonus: f64,
    /// Human-readable grading explanation.
    pub feedback: String,
}

// ─── Session report ───────────────────────────────────────────────────────────

/// End-of-session assessment report.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SessionReport {
    /// Trainee identifier.
    pub trainee_id: String,
    /// Total points earned across all events.
    pub total_score: f64,
    /// Maximum achievable points (without bonus).
    pub max_score: f64,
    /// Whether the trainee met the passing threshold.
    pub pass_fail: bool,
    /// Per-event results: `(event_description, points_earned)`.
    pub event_results: Vec<(String, f64)>,
    /// List of identified competency weaknesses.
    pub competency_gaps: Vec<String>,
    /// Recommended follow-up training modules.
    pub recommended_training: Vec<String>,
}

// ─── Internal grading state ────────────────────────────────────────────────────

/// Grading record for one scenario event.
#[derive(Debug, Clone, Serialize, Deserialize)]
struct EventGrade {
    event_idx: usize,
    score: ResponseScore,
    hint_used: bool,
}

// ─── Training session ─────────────────────────────────────────────────────────

/// A single operator training session.
///
/// # Usage
///
/// 1. Construct via one of the predefined scenario constructors or manually.
/// 2. Call [`advance_time`](TrainingSession::advance_time) in a simulation loop.
/// 3. For each [`ScenarioEvent`] returned, call [`submit_action`](TrainingSession::submit_action).
/// 4. At the end, call [`generate_session_report`](TrainingSession::generate_session_report).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrainingSession {
    /// Unique session identifier.
    pub session_id: String,
    /// Category of training scenario.
    pub scenario_type: ScenarioType,
    /// Identifier of the trainee under evaluation.
    pub trainee_id: String,
    /// Ordered list of events that make up the scenario.
    pub events: Vec<ScenarioEvent>,
    /// Approximate system state at the current simulation time.
    pub system_state: SystemSnapshot,
    /// Chronological list of actions submitted by the trainee.
    pub actions_taken: Vec<TraineeAction>,
    /// Simulation time at which the session started \[s\].
    pub start_time_s: f64,
    /// Current simulation time \[s\].
    pub current_time_s: f64,
    /// Session configuration.
    pub config: OtsConfig,

    // Internal (not exposed via public API but serialised for replay).
    grades: Vec<EventGrade>,
    /// Hint penalty accumulator \[points\].
    hint_penalty: f64,
    /// Set of event indices that have been "delivered" to the trainee.
    delivered_events: Vec<usize>,
}

// ─── Grading helpers ──────────────────────────────────────────────────────────

/// Compare two actions and return a correctness fraction in \[0, 1\].
fn action_match_score(submitted: &OtsOperatorAction, correct: &OtsOperatorAction) -> f64 {
    if submitted == correct {
        return 1.0;
    }
    // Same variant, different parameters → partial credit.
    match (submitted, correct) {
        (
            OtsOperatorAction::AdjustGeneration {
                unit_id: su,
                mw: sm,
            },
            OtsOperatorAction::AdjustGeneration {
                unit_id: cu,
                mw: cm,
            },
        ) if su == cu => {
            // Credit proportional to MW closeness, capped at 0.7.
            let rel_err = ((sm - cm).abs() / cm.abs().max(1.0)).min(1.0);
            (0.7 * (1.0 - rel_err)).max(0.0)
        }
        // CommitUnit vs ActivateReserve in frequency emergency → partial.
        (OtsOperatorAction::CommitUnit { .. }, OtsOperatorAction::ActivateReserve { .. })
        | (OtsOperatorAction::ActivateReserve { .. }, OtsOperatorAction::CommitUnit { .. }) => 0.5,
        // IssueAlert for any type → half credit if correct variant.
        (OtsOperatorAction::IssueAlert { .. }, OtsOperatorAction::IssueAlert { .. }) => 0.6,
        _ => 0.0,
    }
}

/// Determine whether a submitted action is dangerous in the scenario context.
fn is_dangerous(submitted: &OtsOperatorAction, event: &ScenarioEvent) -> bool {
    match &event.event_type {
        OtsEventType::GeneratorTrip { .. } | OtsEventType::FrequencyAlert { .. } => {
            submitted.is_dangerous_for_frequency_emergency()
        }
        _ => false,
    }
}

impl TrainingSession {
    // ─── Constructors ──────────────────────────────────────────────────────

    /// Create a new training session from scratch.
    pub fn new(
        session_id: String,
        scenario_type: ScenarioType,
        trainee_id: String,
        events: Vec<ScenarioEvent>,
        system_state: SystemSnapshot,
        config: OtsConfig,
    ) -> Self {
        Self {
            session_id,
            scenario_type,
            trainee_id,
            events,
            system_state,
            actions_taken: Vec::new(),
            start_time_s: 0.0,
            current_time_s: 0.0,
            config,
            grades: Vec::new(),
            hint_penalty: 0.0,
            delivered_events: Vec::new(),
        }
    }

    // ─── Predefined scenario builders ─────────────────────────────────────

    /// Scenario: a 300 MW unit trips at t = 30 s, trainee must activate reserves.
    pub fn generator_trip_scenario(trainee_id: String) -> Self {
        let events = vec![
            ScenarioEvent {
                time_s: 30.0,
                event_type: OtsEventType::GeneratorTrip {
                    unit_id: "G3".to_string(),
                    capacity_mw: 300.0,
                },
                description: "Unit G3 (300 MW) has tripped offline. \
                              System frequency is falling."
                    .to_string(),
                requires_action: true,
                correct_action: CorrectAction {
                    action_type: OtsOperatorAction::ActivateReserve {
                        reserve_type: "spinning".to_string(),
                    },
                    target: "spinning_reserve".to_string(),
                    value: None,
                    rationale: "Spinning reserve must be activated immediately to arrest \
                                the frequency decline before AGC takes over."
                        .to_string(),
                },
                time_limit_s: 60.0,
            },
            ScenarioEvent {
                time_s: 90.0,
                event_type: OtsEventType::FrequencyAlert { frequency_hz: 49.3 },
                description: "Frequency has dropped to 49.3 Hz. \
                              Commit peaking unit G5 to restore balance."
                    .to_string(),
                requires_action: true,
                correct_action: CorrectAction {
                    action_type: OtsOperatorAction::CommitUnit {
                        unit_id: "G5".to_string(),
                    },
                    target: "G5".to_string(),
                    value: None,
                    rationale: "With spinning reserve exhausted, the next correct step is \
                                to commit fast-start peaking generation."
                        .to_string(),
                },
                time_limit_s: 120.0,
            },
        ];

        Self::new(
            "GEN-TRIP-001".to_string(),
            ScenarioType::EmergencyFrequency,
            trainee_id,
            events,
            SystemSnapshot {
                bus_voltages_pu: vec![1.0, 0.98, 0.97, 0.99],
                generation_mw: vec![500.0, 400.0, 300.0, 200.0],
                load_mw: vec![350.0, 300.0, 250.0, 200.0],
                frequency_hz: 50.0,
                branch_loading_pct: vec![55.0, 62.0, 48.0],
            },
            OtsConfig::default(),
        )
    }

    /// Scenario: line trip creating an N-1 overload; trainee must re-dispatch.
    pub fn n1_contingency_scenario(trainee_id: String) -> Self {
        let events = vec![ScenarioEvent {
            time_s: 20.0,
            event_type: OtsEventType::LineTrip {
                branch_id: "L12".to_string(),
            },
            description: "Line L12 has tripped. Branch L13 is now loaded at 118 %. \
                          Re-dispatch generation to relieve the overload."
                .to_string(),
            requires_action: true,
            correct_action: CorrectAction {
                action_type: OtsOperatorAction::AdjustGeneration {
                    unit_id: "G1".to_string(),
                    mw: 150.0,
                },
                target: "G1".to_string(),
                value: Some(150.0),
                rationale: "Reducing G1 output redirects power flow away from L13 \
                            and returns it within thermal limits."
                    .to_string(),
            },
            time_limit_s: 90.0,
        }];

        Self::new(
            "N1-CONT-001".to_string(),
            ScenarioType::NMinusOneClearance,
            trainee_id,
            events,
            SystemSnapshot {
                bus_voltages_pu: vec![1.02, 1.0, 0.98],
                generation_mw: vec![400.0, 300.0],
                load_mw: vec![320.0, 280.0],
                frequency_hz: 50.0,
                branch_loading_pct: vec![118.0, 55.0],
            },
            OtsConfig::default(),
        )
    }

    /// Scenario: severe frequency emergency requiring sequential actions.
    pub fn frequency_emergency_scenario(trainee_id: String) -> Self {
        let events = vec![
            ScenarioEvent {
                time_s: 10.0,
                event_type: OtsEventType::FrequencyAlert { frequency_hz: 48.8 },
                description: "Frequency has collapsed to 48.8 Hz following a large generation \
                              loss. Immediate under-frequency load shedding required."
                    .to_string(),
                requires_action: true,
                correct_action: CorrectAction {
                    action_type: OtsOperatorAction::ActivateReserve {
                        reserve_type: "ufls".to_string(),
                    },
                    target: "ufls_scheme".to_string(),
                    value: None,
                    rationale: "Under-frequency load shedding (UFLS) is the fastest way to \
                                arrest frequency collapse below 49 Hz."
                        .to_string(),
                },
                time_limit_s: 30.0,
            },
            ScenarioEvent {
                time_s: 60.0,
                event_type: OtsEventType::ManualTrigger {
                    instruction: "Frequency has stabilised at 49.1 Hz. \
                                 Restore shed load in controlled steps."
                        .to_string(),
                },
                description: "UFLS has arrested the decline. Restore load gradually.".to_string(),
                requires_action: true,
                correct_action: CorrectAction {
                    action_type: OtsOperatorAction::CloseSwitch {
                        switch_id: "LS_ZONE_A".to_string(),
                    },
                    target: "LS_ZONE_A".to_string(),
                    value: None,
                    rationale: "Restoring Zone A load first is correct because it carries \
                                the smallest load and allows frequency monitoring between steps."
                        .to_string(),
                },
                time_limit_s: 120.0,
            },
        ];

        Self::new(
            "FREQ-EMRG-001".to_string(),
            ScenarioType::EmergencyFrequency,
            trainee_id,
            events,
            SystemSnapshot {
                bus_voltages_pu: vec![1.0, 0.99, 0.97, 0.95],
                generation_mw: vec![600.0, 400.0, 200.0, 100.0],
                load_mw: vec![450.0, 380.0, 210.0, 120.0],
                frequency_hz: 48.8,
                branch_loading_pct: vec![72.0, 81.0, 44.0],
            },
            OtsConfig {
                minimum_pass_score: 75.0,
                ..OtsConfig::default()
            },
        )
    }

    // ─── Time advancement ──────────────────────────────────────────────────

    /// Advance simulation time by `dt_s` seconds (scaled by `real_time_factor`).
    ///
    /// Returns all [`ScenarioEvent`]s whose `time_s` falls inside the new
    /// time window and that have not previously been delivered.
    pub fn advance_time(&mut self, dt_s: f64) -> Vec<ScenarioEvent> {
        let sim_dt = dt_s * self.config.real_time_factor;
        let prev_time = self.current_time_s;
        self.current_time_s += sim_dt;

        // Collect indices and clones first to avoid simultaneous borrow.
        let to_fire: Vec<(usize, ScenarioEvent)> = self
            .events
            .iter()
            .enumerate()
            .filter(|(idx, event)| {
                event.time_s > prev_time
                    && event.time_s <= self.current_time_s
                    && !self.delivered_events.contains(idx)
            })
            .map(|(idx, event)| (idx, event.clone()))
            .collect();

        let mut fired = Vec::new();
        for (idx, event) in to_fire {
            self.delivered_events.push(idx);
            self.apply_event_to_state(&event);
            fired.push(event);
        }
        fired
    }

    /// Apply a simple first-order state perturbation when an event fires.
    fn apply_event_to_state(&mut self, event: &ScenarioEvent) {
        match &event.event_type {
            OtsEventType::GeneratorTrip { capacity_mw, .. } => {
                // Frequency droop approximation: Δf ≈ -ΔP / (2H × S_base).
                // Using lumped H = 5 s, S_base = total_gen.
                let total_gen: f64 = self.system_state.generation_mw.iter().sum();
                let s_base = total_gen.max(1.0);
                let delta_f = -capacity_mw / (2.0 * 5.0 * s_base) * 50.0;
                self.system_state.frequency_hz =
                    (self.system_state.frequency_hz + delta_f).clamp(47.0, 52.0);
                // Remove the tripped unit's output from the first matching gen.
                if let Some(g) = self.system_state.generation_mw.first_mut() {
                    *g = (*g - capacity_mw).max(0.0);
                }
            }
            OtsEventType::FrequencyAlert { frequency_hz } => {
                self.system_state.frequency_hz = *frequency_hz;
            }
            OtsEventType::LoadStep { bus_id, delta_mw } => {
                let n = self.system_state.load_mw.len();
                let idx = *bus_id % n.max(1);
                if let Some(l) = self.system_state.load_mw.get_mut(idx) {
                    *l = (*l + delta_mw).max(0.0);
                }
            }
            OtsEventType::VoltageAlert { bus_id, voltage_pu } => {
                let n = self.system_state.bus_voltages_pu.len();
                let idx = *bus_id % n.max(1);
                if let Some(v) = self.system_state.bus_voltages_pu.get_mut(idx) {
                    *v = *voltage_pu;
                }
            }
            OtsEventType::LineTrip { .. } => {
                // Overload a branch if any branches are tracked.
                if let Some(b) = self.system_state.branch_loading_pct.first_mut() {
                    *b = (*b * 1.3).min(150.0);
                }
            }
            // Narrative-only events do not perturb state.
            OtsEventType::ProtectionOperation { .. }
            | OtsEventType::FaultCleared { .. }
            | OtsEventType::ManualTrigger { .. } => {}
        }
    }

    // ─── Action submission ─────────────────────────────────────────────────

    /// Submit a trainee action and receive immediate grading feedback.
    ///
    /// If `event_idx` is `None`, the action is matched against the next
    /// unresolved event returned by [`get_next_event`](TrainingSession::get_next_event).
    pub fn submit_action(
        &mut self,
        action: OtsOperatorAction,
        target: String,
        value: Option<f64>,
        event_idx: Option<usize>,
        hint_used: bool,
    ) -> ActionFeedback {
        let resolved_idx = event_idx.or_else(|| self.next_unresolved_event_idx());

        let trainee_action = TraineeAction {
            timestamp_s: self.current_time_s,
            action: action.clone(),
            target: target.clone(),
            value,
            event_idx: resolved_idx,
            hint_used,
        };
        self.actions_taken.push(trainee_action);

        if let Some(idx) = resolved_idx {
            let score = self.evaluate_response(idx, &action);
            let feedback_text = score.feedback.clone();
            // partial_credit is in [0, 100]; time bonus can push points above max_points
            // but we normalise against max_points and cap at 100.
            let partial = (score.points / score.max_points.max(1.0) * 100.0).min(100.0);
            // Correct if the base action was an exact match (points >= max before bonus).
            let correct = score.points >= score.max_points;

            // Record grade.
            self.grades.push(EventGrade {
                event_idx: idx,
                score,
                hint_used,
            });

            let time_taken = self
                .events
                .get(idx)
                .map(|e| (self.current_time_s - e.time_s).max(0.0))
                .unwrap_or(0.0);

            ActionFeedback {
                correct,
                partial_credit: partial,
                explanation: feedback_text,
                time_taken_s: time_taken,
            }
        } else {
            ActionFeedback {
                correct: action == OtsOperatorAction::DoNothing,
                partial_credit: if action == OtsOperatorAction::DoNothing {
                    100.0
                } else {
                    0.0
                },
                explanation: "No pending event requires action at this time.".to_string(),
                time_taken_s: 0.0,
            }
        }
    }

    /// Index of the next unresolved required-action event.
    fn next_unresolved_event_idx(&self) -> Option<usize> {
        let graded: Vec<usize> = self.grades.iter().map(|g| g.event_idx).collect();
        self.events
            .iter()
            .enumerate()
            .filter(|(i, e)| {
                e.requires_action && e.time_s <= self.current_time_s && !graded.contains(i)
            })
            .map(|(i, _)| i)
            .next()
    }

    // ─── Response evaluation ───────────────────────────────────────────────

    /// Score a trainee's response to a specific event.
    ///
    /// # Scoring
    ///
    /// | Scenario                  | Base points |
    /// |---------------------------|-------------|
    /// | Exact match               | 100         |
    /// | Partial match             | 0–70        |
    /// | Wrong but safe            | 50          |
    /// | Dangerous action          | 0           |
    ///
    /// A time bonus of up to 20 additional points is added when the response
    /// arrives within the event's `time_limit_s`.
    pub fn evaluate_response(
        &self,
        event_idx: usize,
        trainee_action: &OtsOperatorAction,
    ) -> ResponseScore {
        let event = match self.events.get(event_idx) {
            Some(e) => e,
            None => {
                return ResponseScore {
                    points: 0.0,
                    max_points: 100.0,
                    time_bonus: 0.0,
                    feedback: format!("Event index {} does not exist.", event_idx),
                }
            }
        };

        let correct = &event.correct_action.action_type;
        let max_points = 100.0_f64;

        // ── Danger check first ──
        if is_dangerous(trainee_action, event) {
            return ResponseScore {
                points: 0.0,
                max_points,
                time_bonus: 0.0,
                feedback: format!(
                    "DANGEROUS ACTION: {} is contra-indicated during a frequency emergency. \
                     Rationale: {}",
                    trainee_action.label(),
                    event.correct_action.rationale
                ),
            };
        }

        // ── Match score ──
        let match_frac = action_match_score(trainee_action, correct);

        let (base_points, feedback_str) = if (match_frac - 1.0).abs() < 1e-9 {
            (
                max_points,
                format!("Correct! {}", event.correct_action.rationale),
            )
        } else if match_frac > 0.0 {
            let pts = max_points * match_frac;
            (
                pts,
                format!(
                    "Partial credit ({:.0}/100). You chose '{}' but the model answer is '{}'. {}",
                    pts,
                    trainee_action.label(),
                    correct.label(),
                    event.correct_action.rationale
                ),
            )
        } else {
            // Wrong but safe → 50 points as a floor.
            (
                50.0,
                format!(
                    "Incorrect (50/100 – safe action but not optimal). \
                     You chose '{}'; model answer: '{}'. {}",
                    trainee_action.label(),
                    correct.label(),
                    event.correct_action.rationale
                ),
            )
        };

        // ── Time bonus ──
        let elapsed = (self.current_time_s - event.time_s).max(0.0);
        let time_bonus = self.compute_time_bonus(elapsed, event.time_limit_s, match_frac);

        let total = (base_points + time_bonus).min(120.0); // cap with bonus

        ResponseScore {
            points: total,
            max_points,
            time_bonus,
            feedback: feedback_str,
        }
    }

    /// Compute the time bonus (up to 20 points) based on how quickly
    /// the trainee responded relative to the event's time limit.
    fn compute_time_bonus(&self, elapsed_s: f64, time_limit_s: f64, match_frac: f64) -> f64 {
        // Only award time bonus on correctly-matched actions.
        if match_frac < 1.0 {
            return 0.0;
        }
        match &self.config.scoring_method {
            ScoringMethod::AccuracyOnly => 0.0,
            ScoringMethod::TimeBased | ScoringMethod::Weighted { .. } => {
                if elapsed_s <= 0.0 || time_limit_s <= 0.0 {
                    return 20.0;
                }
                let ratio = elapsed_s / time_limit_s;
                if ratio <= 1.0 {
                    20.0 * (1.0 - ratio * 0.5) // 10–20 bonus within limit
                } else {
                    // Decays to zero by 3× the time limit.
                    (20.0 * (1.0 - (ratio - 1.0) / 2.0)).max(0.0)
                }
            }
        }
    }

    // ─── Hint ──────────────────────────────────────────────────────────────

    /// Request a hint for event `event_idx`.
    ///
    /// Returns `None` if hints are disabled or the event index is invalid.
    /// Each hint call deducts 10 points from the session score.
    pub fn hint(&mut self, event_idx: usize) -> Option<String> {
        if !self.config.hints_enabled {
            return None;
        }
        let event = self.events.get(event_idx)?;
        self.hint_penalty += 10.0;
        let hint = format!(
            "[HINT –10 pts] Consider action type: {}. Rationale: {}",
            event.correct_action.action_type.label(),
            event.correct_action.rationale
        );
        Some(hint)
    }

    // ─── Next event query ──────────────────────────────────────────────────

    /// Return the next scenario event that requires an action and has not yet
    /// been graded, or `None` if all required events have been addressed.
    pub fn get_next_event(&self) -> Option<&ScenarioEvent> {
        let graded: Vec<usize> = self.grades.iter().map(|g| g.event_idx).collect();
        self.events
            .iter()
            .enumerate()
            .filter(|(i, e)| {
                e.requires_action && e.time_s <= self.current_time_s && !graded.contains(i)
            })
            .map(|(_, e)| e)
            .next()
    }

    // ─── System response simulation ────────────────────────────────────────

    /// Apply an operator action and return the updated [`SystemSnapshot`].
    ///
    /// Uses a simple first-order model:
    /// - `CommitUnit` → generation +capacity, frequency recovers towards 50 Hz.
    /// - `ActivateReserve` → frequency partial recovery (+0.3 Hz).
    /// - `AdjustGeneration` → generation MW updated, branch loadings recalculated.
    /// - `OpenSwitch` / `CloseSwitch` → branch loading changes.
    pub fn simulate_system_response(&mut self, action: &OtsOperatorAction) -> SystemSnapshot {
        match action {
            OtsOperatorAction::CommitUnit { .. } => {
                // Add nominal 100 MW to first generator slot.
                if let Some(g) = self.system_state.generation_mw.first_mut() {
                    *g += 100.0;
                }
                // Frequency recovery: τ = 10 s; step-response approx.
                let deficit = 50.0 - self.system_state.frequency_hz;
                self.system_state.frequency_hz += deficit * 0.6;
            }
            OtsOperatorAction::ActivateReserve { .. } => {
                let deficit = 50.0 - self.system_state.frequency_hz;
                self.system_state.frequency_hz += deficit * 0.3;
            }
            OtsOperatorAction::AdjustGeneration { mw, .. } => {
                if let Some(g) = self.system_state.generation_mw.first_mut() {
                    *g = (*g + mw).max(0.0);
                }
                // Rebalance: update branch loadings proportionally.
                let total_gen: f64 = self.system_state.generation_mw.iter().sum();
                let total_load: f64 = self.system_state.load_mw.iter().sum();
                let mismatch_frac = (total_gen - total_load) / total_load.max(1.0);
                for b in self.system_state.branch_loading_pct.iter_mut() {
                    *b = (*b * (1.0 - 0.1 * mismatch_frac)).clamp(0.0, 200.0);
                }
            }
            OtsOperatorAction::CloseSwitch { .. } => {
                // Closing restores a load block → slight frequency dip.
                self.system_state.frequency_hz = (self.system_state.frequency_hz - 0.05).max(47.0);
            }
            OtsOperatorAction::OpenSwitch { .. } => {
                // Opening sheds load → frequency relief.
                self.system_state.frequency_hz = (self.system_state.frequency_hz + 0.1).min(52.0);
            }
            OtsOperatorAction::IssueAlert { .. }
            | OtsOperatorAction::VerifyData
            | OtsOperatorAction::DoNothing
            | OtsOperatorAction::DecommitUnit { .. } => {
                // No immediate physical effect in the simplified model.
            }
        }
        self.system_state.clone()
    }

    // ─── Session report ────────────────────────────────────────────────────

    /// Generate an end-of-session competency report.
    pub fn generate_session_report(&self) -> SessionReport {
        let max_score: f64 = self
            .events
            .iter()
            .filter(|e| e.requires_action)
            .map(|_| 100.0)
            .sum();

        // Collect scored events; unscored required events get 0.
        let mut event_results: Vec<(String, f64)> = Vec::new();
        let mut total_earned = 0.0_f64;

        for (idx, event) in self.events.iter().enumerate() {
            if !event.requires_action {
                continue;
            }
            let earned = self
                .grades
                .iter()
                .find(|g| g.event_idx == idx)
                .map(|g| g.score.points.min(100.0))
                .unwrap_or(0.0);
            total_earned += earned;
            event_results.push((event.description.clone(), earned));
        }

        // Apply hint penalty.
        let total_score = (total_earned - self.hint_penalty).max(0.0);

        let pass_fail = total_score >= self.config.minimum_pass_score / 100.0 * max_score.max(1.0);

        // Identify gaps.
        let mut competency_gaps = Vec::new();
        let mut recommended_training = Vec::new();

        for (idx, event) in self.events.iter().enumerate() {
            if !event.requires_action {
                continue;
            }
            let earned = self
                .grades
                .iter()
                .find(|g| g.event_idx == idx)
                .map(|g| g.score.points)
                .unwrap_or(0.0);
            if earned < 70.0 {
                let gap = match &event.event_type {
                    OtsEventType::GeneratorTrip { .. } => "Generation trip response",
                    OtsEventType::FrequencyAlert { .. } => "Frequency emergency management",
                    OtsEventType::VoltageAlert { .. } => "Voltage emergency response",
                    OtsEventType::LineTrip { .. } => "N-1 contingency clearance",
                    OtsEventType::ProtectionOperation { .. } => "Protection coordination",
                    OtsEventType::FaultCleared { .. } => "Post-fault restoration",
                    OtsEventType::ManualTrigger { .. } => "Procedure adherence",
                    OtsEventType::LoadStep { .. } => "Load management",
                };
                if !competency_gaps.contains(&gap.to_string()) {
                    competency_gaps.push(gap.to_string());
                }
            }
        }

        // Map gaps to recommended modules.
        for gap in &competency_gaps {
            let module = match gap.as_str() {
                "Generation trip response" => "Module 3: Generator Outage Management",
                "Frequency emergency management" => "Module 4: Frequency Control & UFLS",
                "Voltage emergency response" => "Module 5: Reactive Power & Voltage Control",
                "N-1 contingency clearance" => "Module 6: Contingency Analysis & Re-dispatch",
                "Protection coordination" => "Module 7: Protection Systems",
                "Post-fault restoration" => "Module 8: System Restoration",
                "Procedure adherence" => "Module 2: Standard Operating Procedures",
                "Load management" => "Module 9: Demand-Side Management",
                _ => "Module 1: Grid Fundamentals Review",
            };
            if !recommended_training.contains(&module.to_string()) {
                recommended_training.push(module.to_string());
            }
        }

        if !pass_fail && recommended_training.is_empty() {
            recommended_training.push("Module 1: Grid Fundamentals Review".to_string());
        }

        SessionReport {
            trainee_id: self.trainee_id.clone(),
            total_score,
            max_score,
            pass_fail,
            event_results,
            competency_gaps,
            recommended_training,
        }
    }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Helper ────────────────────────────────────────────────────────────────

    fn trip_session() -> TrainingSession {
        TrainingSession::generator_trip_scenario("tst_trainee".to_string())
    }

    // ── 1. Generator trip scenario: event fires at correct time ───────────────

    #[test]
    fn test_generator_trip_event_at_correct_time() {
        let mut session = trip_session();
        // Before t = 30 s, no events.
        let early = session.advance_time(25.0);
        assert!(
            early.is_empty(),
            "no event expected before t=30 s, got {:?}",
            early
        );
        // Advance to t = 35 s; t=30 event should fire.
        let fired = session.advance_time(10.0); // total 35 s
        assert_eq!(fired.len(), 1, "expected exactly 1 event");
        assert!(
            matches!(&fired[0].event_type, OtsEventType::GeneratorTrip { unit_id, .. } if unit_id == "G3")
        );
    }

    // ── 2. Correct action → 100 pts; wrong action → lower score ──────────────

    #[test]
    fn test_correct_action_scores_100() {
        let mut session = trip_session();
        let _fired = session.advance_time(35.0); // deliver t=30 event

        let feedback = session.submit_action(
            OtsOperatorAction::ActivateReserve {
                reserve_type: "spinning".to_string(),
            },
            "spinning_reserve".to_string(),
            None,
            Some(0), // event index 0
            false,
        );
        assert!(feedback.correct, "exact match should be correct");
        assert!(
            (feedback.partial_credit - 100.0).abs() < 0.1,
            "expected 100 credit, got {}",
            feedback.partial_credit
        );
    }

    #[test]
    fn test_wrong_action_scores_lower() {
        let mut session = trip_session();
        let _fired = session.advance_time(35.0);

        let feedback = session.submit_action(
            OtsOperatorAction::DecommitUnit {
                unit_id: "G1".to_string(),
            },
            "G1".to_string(),
            None,
            Some(0),
            false,
        );
        assert!(!feedback.correct, "dangerous action should not be correct");
        assert!(
            feedback.partial_credit < 50.0,
            "dangerous action should score 0, got {}",
            feedback.partial_credit
        );
    }

    // ── 3. Hint reduces available score ───────────────────────────────────────

    #[test]
    fn test_hint_reduces_score() {
        let mut session = trip_session();
        let _fired = session.advance_time(35.0);

        let hint_text = session.hint(0);
        assert!(hint_text.is_some(), "hints should be available by default");

        // Answer correctly after hint.
        session.submit_action(
            OtsOperatorAction::ActivateReserve {
                reserve_type: "spinning".to_string(),
            },
            "spinning_reserve".to_string(),
            None,
            Some(0),
            true,
        );

        let report = session.generate_session_report();
        // Two required events; first gets ~100-120 pts (capped at 100 in report), hint −10.
        // max_score = 200 (2 events × 100).
        assert!(
            report.total_score < 200.0,
            "hint penalty should reduce total score below max"
        );
        assert!(
            session.hint_penalty >= 10.0,
            "hint penalty should be at least 10"
        );
    }

    // ── 4. Time-based scoring: late action gets lower bonus ───────────────────

    #[test]
    fn test_time_based_scoring_late_action() {
        let mut session = trip_session();
        session.config.scoring_method = ScoringMethod::TimeBased;

        let _fired = session.advance_time(35.0);
        // Simulate a very late response (200 s after event at t=30 s).
        session.current_time_s = 230.0;

        let score_late = session.evaluate_response(
            0,
            &OtsOperatorAction::ActivateReserve {
                reserve_type: "spinning".to_string(),
            },
        );

        // Now test a fast response.
        let mut session2 = trip_session();
        session2.config.scoring_method = ScoringMethod::TimeBased;
        let _fired2 = session2.advance_time(35.0);
        // Response 5 s after event.
        session2.current_time_s = 35.0;
        let score_fast = session2.evaluate_response(
            0,
            &OtsOperatorAction::ActivateReserve {
                reserve_type: "spinning".to_string(),
            },
        );

        assert!(
            score_fast.points >= score_late.points,
            "fast response ({}) should score >= late response ({})",
            score_fast.points,
            score_late.points
        );
    }

    // ── 5. Session report: score computed correctly ────────────────────────────

    #[test]
    fn test_session_report_score() {
        let mut session = trip_session();
        let _e1 = session.advance_time(35.0);
        // Answer event 0 correctly.
        session.submit_action(
            OtsOperatorAction::ActivateReserve {
                reserve_type: "spinning".to_string(),
            },
            "spinning_reserve".to_string(),
            None,
            Some(0),
            false,
        );
        // Advance to t=100 s so event 1 (t=90 s) fires.
        let _e2 = session.advance_time(65.0);
        // Answer event 1 correctly.
        session.submit_action(
            OtsOperatorAction::CommitUnit {
                unit_id: "G5".to_string(),
            },
            "G5".to_string(),
            None,
            Some(1),
            false,
        );

        let report = session.generate_session_report();
        assert_eq!(report.max_score, 200.0, "two required events × 100 pts");
        assert!(report.total_score > 0.0, "earned score should be positive");
    }

    // ── 6. Advance time: events returned when time passes ─────────────────────

    #[test]
    fn test_advance_time_events_returned() {
        let mut session = trip_session();
        // Single large step covering both events (t=30, t=90).
        let fired = session.advance_time(100.0);
        assert_eq!(fired.len(), 2, "both events should fire in a 100 s window");
        // Second advance should deliver no more events.
        let fired2 = session.advance_time(100.0);
        assert!(fired2.is_empty(), "events should not be delivered twice");
    }

    // ── 7. System response: CommitUnit → frequency increases ──────────────────

    #[test]
    fn test_system_response_commit_unit_frequency_rises() {
        let mut session = trip_session();
        // Depress frequency artificially.
        session.system_state.frequency_hz = 49.0;

        let action = OtsOperatorAction::CommitUnit {
            unit_id: "G5".to_string(),
        };
        let snapshot = session.simulate_system_response(&action);

        assert!(
            snapshot.frequency_hz > 49.0,
            "CommitUnit should raise frequency, got {:.3} Hz",
            snapshot.frequency_hz
        );
        assert!(
            snapshot.frequency_hz <= 50.0,
            "frequency should not overshoot nominal"
        );
    }

    // ── 8. Pass/fail threshold ────────────────────────────────────────────────

    #[test]
    fn test_pass_fail_threshold() {
        let mut session = trip_session();
        let _e1 = session.advance_time(100.0); // deliver both events
                                               // Provide correct answers for both events.
        session.submit_action(
            OtsOperatorAction::ActivateReserve {
                reserve_type: "spinning".to_string(),
            },
            "spinning_reserve".to_string(),
            None,
            Some(0),
            false,
        );
        session.submit_action(
            OtsOperatorAction::CommitUnit {
                unit_id: "G5".to_string(),
            },
            "G5".to_string(),
            None,
            Some(1),
            false,
        );

        let report = session.generate_session_report();
        assert!(
            report.pass_fail,
            "perfect score should pass (threshold 70 %)"
        );

        // Now build a session where we score 0.
        let mut fail_session = trip_session();
        let _e2 = fail_session.advance_time(100.0);
        // No actions submitted → all events score 0.
        let fail_report = fail_session.generate_session_report();
        assert!(!fail_report.pass_fail, "zero score should fail");
    }

    // ── 9. N-1 contingency scenario sanity check ──────────────────────────────

    #[test]
    fn test_n1_contingency_scenario_basic() {
        let mut session = TrainingSession::n1_contingency_scenario("trainee_n1".to_string());
        let fired = session.advance_time(25.0);
        assert_eq!(fired.len(), 1);
        assert!(matches!(
            &fired[0].event_type,
            OtsEventType::LineTrip { branch_id } if branch_id == "L12"
        ));
    }

    // ── 10. Hint disabled returns None ────────────────────────────────────────

    #[test]
    fn test_hint_disabled_returns_none() {
        let mut session = trip_session();
        session.config.hints_enabled = false;
        let hint = session.hint(0);
        assert!(hint.is_none(), "hints should be disabled");
        assert_eq!(session.hint_penalty, 0.0, "no penalty when hints disabled");
    }

    // ── 11. get_next_event returns correct event ───────────────────────────────

    #[test]
    fn test_get_next_event() {
        let mut session = trip_session();
        // Before any event fires, get_next_event should return None.
        assert!(
            session.get_next_event().is_none(),
            "no event due before time advance"
        );
        let _fired = session.advance_time(35.0);
        let next = session.get_next_event();
        assert!(next.is_some(), "after advance event should be available");
    }

    // ── 12. ActivateReserve response raises frequency ─────────────────────────

    #[test]
    fn test_activate_reserve_raises_frequency() {
        let mut session = frequency_emergency_session();
        let action = OtsOperatorAction::ActivateReserve {
            reserve_type: "ufls".to_string(),
        };
        let before = session.system_state.frequency_hz;
        let after = session.simulate_system_response(&action);
        assert!(
            after.frequency_hz > before,
            "ActivateReserve should raise frequency from {:.3} Hz",
            before
        );
    }

    fn frequency_emergency_session() -> TrainingSession {
        TrainingSession::frequency_emergency_scenario("trainee_freq".to_string())
    }
}
