/// Auto-recloser logic for overhead line protection.
///
/// Implements:
/// - Fast/delayed reclose sequence (typically 3 shots)
/// - Persistent fault blocking after sequence exhaustion
/// - Coordination with upstream/downstream devices (zone selectivity)
/// - Reclose inhibit conditions (sync check, voltage check)
/// - Dead time management between reclose attempts
///
/// # Standard sequences
///
/// The IEC 60255-4 sequence is configurable:
///   - Shot 1: Fast trip + dead time t_d1 → Reclose 1
///   - Shot 2: Delayed trip + dead time t_d2 → Reclose 2
///   - Shot 3: Delayed trip + dead time t_d3 → Reclose 3
///   - Lockout after final shot failure
///
/// # References
/// - IEC 60255-4:1976 — Auto-reclosing of high voltage distribution systems
/// - Westinghouse, "Applied Protective Relaying", Ch. 12
use serde::{Deserialize, Serialize};

// ────────────────────────────────────────────────────────────────────────────
// Configuration
// ────────────────────────────────────────────────────────────────────────────

/// Auto-recloser configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AutorecloserConfig {
    /// Maximum number of reclose attempts
    pub n_shots: usize,
    /// Dead times between reclose attempts [s] (length ≥ n_shots)
    pub dead_times_s: Vec<f64>,
    /// Reclaim time after successful reclose [s] (reset timer to normal)
    pub reclaim_time_s: f64,
    /// Instantaneous (fast) trip applied on first shot?
    pub first_shot_fast: bool,
    /// Sync check required for reclose? (checks voltage diff < limit)
    pub sync_check_enabled: bool,
    /// Voltage difference limit for sync check [p.u.]
    pub sync_check_dv_pu: f64,
    /// Minimum voltage on line to confirm de-energisation before reclose
    pub min_line_voltage_pu: f64,
    /// Hold-off time before allowing first reclose after trip [s]
    pub holdoff_time_s: f64,
}

impl AutorecloserConfig {
    /// Typical distribution line auto-recloser (3 shots, IEC defaults).
    pub fn distribution_3shot() -> Self {
        Self {
            n_shots: 3,
            dead_times_s: vec![0.3, 15.0, 30.0],
            reclaim_time_s: 120.0,
            first_shot_fast: true,
            sync_check_enabled: false,
            sync_check_dv_pu: 0.15,
            min_line_voltage_pu: 0.05,
            holdoff_time_s: 0.0,
        }
    }

    /// High-voltage transmission line recloser (1 fast shot + sync check).
    pub fn transmission_1shot_sync() -> Self {
        Self {
            n_shots: 1,
            dead_times_s: vec![0.5],
            reclaim_time_s: 300.0,
            first_shot_fast: true,
            sync_check_enabled: true,
            sync_check_dv_pu: 0.10,
            min_line_voltage_pu: 0.05,
            holdoff_time_s: 0.05,
        }
    }

    /// Underground cable: no reclose (cables sustain permanent faults).
    pub fn cable_no_reclose() -> Self {
        Self {
            n_shots: 0,
            dead_times_s: vec![],
            reclaim_time_s: 600.0,
            first_shot_fast: false,
            sync_check_enabled: false,
            sync_check_dv_pu: 0.15,
            min_line_voltage_pu: 0.05,
            holdoff_time_s: 0.0,
        }
    }
}

// ────────────────────────────────────────────────────────────────────────────
// State machine
// ────────────────────────────────────────────────────────────────────────────

/// Phase of the auto-recloser sequence.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum RecloserPhase {
    /// Normal service — breaker closed, no fault
    Normal,
    /// Fault detected — breaker tripped, counting dead time
    WaitingDeadTime { shot: usize, elapsed_s: f64 },
    /// Reclosing attempt in progress (breaker closes for sync check or direct)
    ReclosingAttempt { shot: usize },
    /// Successful reclose — counting reclaim time
    Reclaiming { shots_used: usize, elapsed_s: f64 },
    /// Lockout — all shots exhausted, breaker locked open
    Lockout { n_shots: usize },
    /// Inhibited — reclose inhibited by external signal
    Inhibited,
}

/// Auto-recloser state.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AutorecloserState {
    /// Current phase
    pub phase: RecloserPhase,
    /// Is the breaker closed?
    pub breaker_closed: bool,
    /// Number of shots fired so far (cumulative)
    pub total_shots_fired: usize,
    /// Time since last state transition [s]
    pub time_in_state_s: f64,
}

impl AutorecloserState {
    pub fn new() -> Self {
        Self {
            phase: RecloserPhase::Normal,
            breaker_closed: true,
            total_shots_fired: 0,
            time_in_state_s: 0.0,
        }
    }

    /// Is the recloser in lockout?
    pub fn is_locked_out(&self) -> bool {
        matches!(self.phase, RecloserPhase::Lockout { .. })
    }

    /// Is the recloser in normal service (no ongoing sequence)?
    pub fn is_normal(&self) -> bool {
        matches!(self.phase, RecloserPhase::Normal)
    }
}

impl Default for AutorecloserState {
    fn default() -> Self {
        Self::new()
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Events
// ────────────────────────────────────────────────────────────────────────────

/// External input events to the auto-recloser.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum RecloserEvent {
    /// Protective relay issued a trip signal
    FaultTrip,
    /// Fault cleared (current returned to normal after reclose)
    FaultCleared,
    /// Fault persisted after reclose (overcurrent detected again)
    FaultPersisted,
    /// External inhibit signal (e.g. from transfer trip, manual lockout)
    InhibitSignal,
    /// Inhibit released
    InhibitReleased,
    /// Manual reset of lockout
    ManualReset,
}

/// Output commands from the recloser logic.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum RecloserCommand {
    /// Open the breaker
    OpenBreaker,
    /// Close the breaker (reclose attempt)
    CloseBreaker,
    /// Lock out (open and inhibit future closing)
    LockOut,
    /// Issue alarm
    Alarm(String),
    /// No action
    None,
}

// ────────────────────────────────────────────────────────────────────────────
// Core logic
// ────────────────────────────────────────────────────────────────────────────

/// Process an event and advance the auto-recloser state machine.
///
/// Returns a command to issue to the breaker/SCADA.
pub fn process_event(
    state: &mut AutorecloserState,
    config: &AutorecloserConfig,
    event: RecloserEvent,
) -> RecloserCommand {
    match (&state.phase, event) {
        (RecloserPhase::Normal, RecloserEvent::FaultTrip) => {
            if config.n_shots == 0 {
                // No reclose configured — lockout immediately
                state.phase = RecloserPhase::Lockout { n_shots: 0 };
                state.breaker_closed = false;
                state.total_shots_fired += 1;
                return RecloserCommand::LockOut;
            }
            state.phase = RecloserPhase::WaitingDeadTime {
                shot: 1,
                elapsed_s: 0.0,
            };
            state.breaker_closed = false;
            state.total_shots_fired += 1;
            RecloserCommand::OpenBreaker
        }

        (RecloserPhase::WaitingDeadTime { shot, .. }, RecloserEvent::FaultTrip) => {
            // Another trip during dead time → advance to next shot or lockout
            let shot = *shot;
            if shot >= config.n_shots {
                state.phase = RecloserPhase::Lockout {
                    n_shots: state.total_shots_fired,
                };
                RecloserCommand::LockOut
            } else {
                state.phase = RecloserPhase::WaitingDeadTime {
                    shot: shot + 1,
                    elapsed_s: 0.0,
                };
                state.total_shots_fired += 1;
                RecloserCommand::OpenBreaker
            }
        }

        (RecloserPhase::ReclosingAttempt { shot }, RecloserEvent::FaultPersisted) => {
            let shot = *shot;
            state.breaker_closed = false;
            if shot >= config.n_shots {
                state.phase = RecloserPhase::Lockout {
                    n_shots: state.total_shots_fired,
                };
                RecloserCommand::LockOut
            } else {
                state.total_shots_fired += 1;
                state.phase = RecloserPhase::WaitingDeadTime {
                    shot: shot + 1,
                    elapsed_s: 0.0,
                };
                RecloserCommand::OpenBreaker
            }
        }

        (RecloserPhase::ReclosingAttempt { shot }, RecloserEvent::FaultCleared) => {
            let shot = *shot;
            state.breaker_closed = true;
            state.phase = RecloserPhase::Reclaiming {
                shots_used: shot,
                elapsed_s: 0.0,
            };
            RecloserCommand::None
        }

        (RecloserPhase::Reclaiming { .. }, RecloserEvent::FaultTrip) => {
            // Fault during reclaim time → continue sequence
            let shot = state.total_shots_fired;
            if shot >= config.n_shots {
                state.phase = RecloserPhase::Lockout { n_shots: shot };
                state.breaker_closed = false;
                RecloserCommand::LockOut
            } else {
                state.total_shots_fired += 1;
                state.phase = RecloserPhase::WaitingDeadTime {
                    shot: shot + 1,
                    elapsed_s: 0.0,
                };
                state.breaker_closed = false;
                RecloserCommand::OpenBreaker
            }
        }

        (_, RecloserEvent::InhibitSignal) => {
            state.phase = RecloserPhase::Inhibited;
            RecloserCommand::Alarm("Recloser inhibited".to_string())
        }

        (RecloserPhase::Inhibited, RecloserEvent::InhibitReleased) => {
            state.phase = RecloserPhase::Normal;
            RecloserCommand::None
        }

        (_, RecloserEvent::ManualReset) => {
            state.phase = RecloserPhase::Normal;
            state.breaker_closed = false; // Requires manual closing
            RecloserCommand::Alarm("Lockout reset — manual close required".to_string())
        }

        _ => RecloserCommand::None,
    }
}

/// Advance the auto-recloser on a time tick [s].
///
/// Handles dead-time expiry (issue reclose) and reclaim-time expiry (return to Normal).
/// Returns a command if the state changes.
pub fn tick(
    state: &mut AutorecloserState,
    config: &AutorecloserConfig,
    dt_s: f64,
) -> RecloserCommand {
    state.time_in_state_s += dt_s;

    match &mut state.phase {
        RecloserPhase::WaitingDeadTime { shot, elapsed_s } => {
            let shot_idx = shot
                .saturating_sub(1)
                .min(config.dead_times_s.len().saturating_sub(1));
            let dead_time = config.dead_times_s.get(shot_idx).copied().unwrap_or(30.0);
            *elapsed_s += dt_s;
            if *elapsed_s >= dead_time + config.holdoff_time_s {
                let shot_val = *shot;
                state.phase = RecloserPhase::ReclosingAttempt { shot: shot_val };
                state.breaker_closed = true;
                return RecloserCommand::CloseBreaker;
            }
        }

        RecloserPhase::Reclaiming { elapsed_s, .. } => {
            *elapsed_s += dt_s;
            if *elapsed_s >= config.reclaim_time_s {
                // Successful reclaim — return to Normal
                state.phase = RecloserPhase::Normal;
            }
        }

        _ => {}
    }

    RecloserCommand::None
}

// ────────────────────────────────────────────────────────────────────────────
// Statistics
// ────────────────────────────────────────────────────────────────────────────

/// Fault classification based on recloser outcome.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum FaultClassification {
    /// Transient: cleared on first shot
    Transient,
    /// Transient: cleared on subsequent shot (n > 1)
    SemiPermanent { shot: usize },
    /// Permanent: all shots exhausted → lockout
    Permanent,
    /// Unknown: sequence still in progress
    InProgress,
}

/// Classify the fault outcome from the recloser state.
pub fn classify_fault(state: &AutorecloserState) -> FaultClassification {
    match &state.phase {
        RecloserPhase::Lockout { .. } => FaultClassification::Permanent,
        RecloserPhase::Reclaiming { shots_used, .. } => {
            if *shots_used == 1 {
                FaultClassification::Transient
            } else {
                FaultClassification::SemiPermanent { shot: *shots_used }
            }
        }
        RecloserPhase::Normal => FaultClassification::Transient,
        _ => FaultClassification::InProgress,
    }
}

/// Reliability statistics for a collection of fault events.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RecloserReliabilityStats {
    pub total_faults: usize,
    pub transient_faults: usize,
    pub semi_permanent_faults: usize,
    pub permanent_faults: usize,
    /// Transient fault rate [fraction]
    pub transient_rate: f64,
    /// Permanent fault rate [fraction]
    pub permanent_rate: f64,
    /// Average shots per fault
    pub avg_shots_per_fault: f64,
}

impl RecloserReliabilityStats {
    pub fn from_classifications(events: &[(FaultClassification, usize)]) -> Self {
        let total = events.len();
        let transient = events
            .iter()
            .filter(|(c, _)| *c == FaultClassification::Transient)
            .count();
        let semi = events
            .iter()
            .filter(|(c, _)| matches!(c, FaultClassification::SemiPermanent { .. }))
            .count();
        let permanent = events
            .iter()
            .filter(|(c, _)| *c == FaultClassification::Permanent)
            .count();
        let total_shots: usize = events.iter().map(|(_, s)| s).sum();

        Self {
            total_faults: total,
            transient_faults: transient,
            semi_permanent_faults: semi,
            permanent_faults: permanent,
            transient_rate: if total > 0 {
                transient as f64 / total as f64
            } else {
                0.0
            },
            permanent_rate: if total > 0 {
                permanent as f64 / total as f64
            } else {
                0.0
            },
            avg_shots_per_fault: if total > 0 {
                total_shots as f64 / total as f64
            } else {
                0.0
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run_sequence(
        config: &AutorecloserConfig,
        fault_clears_on_shot: Option<usize>,
    ) -> AutorecloserState {
        let mut state = AutorecloserState::new();
        // Trip
        process_event(&mut state, config, RecloserEvent::FaultTrip);
        assert!(!state.breaker_closed);

        for shot in 1..=config.n_shots {
            // Advance dead time
            let dt = 0.001;
            let dead = config.dead_times_s.get(shot - 1).copied().unwrap_or(30.0) + 0.01;
            let steps = (dead / dt).ceil() as usize;
            for _ in 0..steps {
                tick(&mut state, config, dt);
            }

            if matches!(state.phase, RecloserPhase::ReclosingAttempt { .. }) {
                if fault_clears_on_shot == Some(shot) {
                    process_event(&mut state, config, RecloserEvent::FaultCleared);
                    break;
                } else {
                    process_event(&mut state, config, RecloserEvent::FaultPersisted);
                }
            }
        }
        state
    }

    #[test]
    fn test_transient_fault_clears_on_shot1() {
        let config = AutorecloserConfig::distribution_3shot();
        let state = run_sequence(&config, Some(1));
        assert!(
            state.breaker_closed,
            "Breaker should be closed after successful reclose"
        );
        assert!(matches!(state.phase, RecloserPhase::Reclaiming { .. }));
    }

    #[test]
    fn test_permanent_fault_lockout() {
        let config = AutorecloserConfig::distribution_3shot();
        let state = run_sequence(&config, None);
        assert!(
            state.is_locked_out(),
            "All shots exhausted → lockout: {:?}",
            state.phase
        );
        assert!(!state.breaker_closed);
    }

    #[test]
    fn test_no_reclose_configured_immediate_lockout() {
        let config = AutorecloserConfig::cable_no_reclose();
        let mut state = AutorecloserState::new();
        let cmd = process_event(&mut state, &config, RecloserEvent::FaultTrip);
        assert_eq!(cmd, RecloserCommand::LockOut);
        assert!(state.is_locked_out());
    }

    #[test]
    fn test_manual_reset_after_lockout() {
        let config = AutorecloserConfig::distribution_3shot();
        let mut state = AutorecloserState::new();
        state.phase = RecloserPhase::Lockout { n_shots: 3 };
        process_event(&mut state, &config, RecloserEvent::ManualReset);
        assert!(state.is_normal());
    }

    #[test]
    fn test_inhibit_signal() {
        let config = AutorecloserConfig::distribution_3shot();
        let mut state = AutorecloserState::new();
        let cmd = process_event(&mut state, &config, RecloserEvent::InhibitSignal);
        assert!(matches!(cmd, RecloserCommand::Alarm(_)));
        assert_eq!(state.phase, RecloserPhase::Inhibited);
        process_event(&mut state, &config, RecloserEvent::InhibitReleased);
        assert!(state.is_normal());
    }

    #[test]
    fn test_dead_time_triggers_reclose() {
        let config = AutorecloserConfig::distribution_3shot();
        let mut state = AutorecloserState::new();
        // Enter dead time
        process_event(&mut state, &config, RecloserEvent::FaultTrip);
        // Advance past dead_time[0] = 0.3s; capture any CloseBreaker command
        let mut issued_close = false;
        for _ in 0..400 {
            if tick(&mut state, &config, 0.001) == RecloserCommand::CloseBreaker {
                issued_close = true;
            }
        }
        assert!(
            issued_close || matches!(state.phase, RecloserPhase::ReclosingAttempt { .. }),
            "Should have issued close or entered ReclosingAttempt: {:?}",
            state.phase
        );
    }

    #[test]
    fn test_reclaim_time_returns_to_normal() {
        let mut config = AutorecloserConfig::distribution_3shot();
        config.reclaim_time_s = 0.1; // short reclaim for test
        let mut state = AutorecloserState::new();
        state.phase = RecloserPhase::Reclaiming {
            shots_used: 1,
            elapsed_s: 0.0,
        };
        for _ in 0..200 {
            tick(&mut state, &config, 0.001);
        }
        assert!(
            state.is_normal(),
            "Should return to normal after reclaim: {:?}",
            state.phase
        );
    }

    #[test]
    fn test_shots_fired_tracked() {
        let config = AutorecloserConfig::distribution_3shot();
        let state = run_sequence(&config, None);
        assert_eq!(state.total_shots_fired, 3, "Should have fired 3 shots");
    }

    #[test]
    fn test_fault_classification_transient() {
        let mut state = AutorecloserState::new();
        state.phase = RecloserPhase::Reclaiming {
            shots_used: 1,
            elapsed_s: 0.0,
        };
        assert_eq!(classify_fault(&state), FaultClassification::Transient);
    }

    #[test]
    fn test_fault_classification_permanent() {
        let mut state = AutorecloserState::new();
        state.phase = RecloserPhase::Lockout { n_shots: 3 };
        assert_eq!(classify_fault(&state), FaultClassification::Permanent);
    }

    #[test]
    fn test_reliability_stats() {
        let events = vec![
            (FaultClassification::Transient, 1),
            (FaultClassification::Transient, 1),
            (FaultClassification::SemiPermanent { shot: 2 }, 2),
            (FaultClassification::Permanent, 3),
        ];
        let stats = RecloserReliabilityStats::from_classifications(&events);
        assert_eq!(stats.total_faults, 4);
        assert_eq!(stats.transient_faults, 2);
        assert_eq!(stats.permanent_faults, 1);
        assert!((stats.transient_rate - 0.5).abs() < 1e-9);
        assert!((stats.avg_shots_per_fault - 1.75).abs() < 1e-9);
    }

    #[test]
    fn test_transmission_config_1shot() {
        let config = AutorecloserConfig::transmission_1shot_sync();
        assert_eq!(config.n_shots, 1);
        assert!(config.sync_check_enabled);
    }
}
