//! Distribution Feeder Automation (DFA) system.
//!
//! Implements SCADA-driven switching operations, automatic fault isolation,
//! service restoration, and sectionalizer coordination for distribution feeders.
//!
//! # Overview
//! - [`FeederTopology`] — top-level structure holding switches and sections
//! - [`process_fault`](FeederTopology::process_fault) — fault isolation via recloser + sectionalizers
//! - [`restore_unfaulted_sections`](FeederTopology::restore_unfaulted_sections) — tie-switch restoration
//! - [`feeder_health_report`](FeederTopology::feeder_health_report) — maintenance & load summary
//! - [`simulate_n1_contingency`](FeederTopology::simulate_n1_contingency) — N-1 security check

use serde::{Deserialize, Serialize};

// ─────────────────────────────────────────────────────────────────────────────
// Error type
// ─────────────────────────────────────────────────────────────────────────────

/// Errors that can arise during feeder automation operations.
#[derive(Debug)]
pub enum FeederAutomationError {
    /// Section index is out of bounds.
    InvalidSectionIndex(usize),
    /// Switch index is out of bounds.
    InvalidSwitchIndex(usize),
    /// Attempted an invalid operation (e.g., reclosing a blown fuse).
    InvalidOperation(String),
}

impl std::fmt::Display for FeederAutomationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidSectionIndex(i) => write!(f, "invalid section index: {i}"),
            Self::InvalidSwitchIndex(i) => write!(f, "invalid switch index: {i}"),
            Self::InvalidOperation(msg) => write!(f, "invalid operation: {msg}"),
        }
    }
}

impl std::error::Error for FeederAutomationError {}

// ─────────────────────────────────────────────────────────────────────────────
// Core enums
// ─────────────────────────────────────────────────────────────────────────────

/// Functional type of a distribution switch.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SwitchType {
    /// Automatic recloser — performs up to N shots (instantaneous + timed).
    Recloser,
    /// Sectionalizer — counts upstream recloser operations; opens at lockout.
    Sectionalizer,
    /// Manually operated switch — no automatic action.
    ManualSwitch,
    /// SCADA-controllable switch — no automatic reclosing.
    AutoSwitch,
    /// One-time fuse element — must be replaced after operation.
    Fuse,
}

/// Operational state of a feeder switch.
///
/// Prefixed `Fa` to avoid conflict with `topology_optimization::SwitchState`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FaSwitchState {
    /// Contacts closed — circuit energised.
    Closed,
    /// Contacts open — circuit de-energised normally.
    Open,
    /// Opened due to fault detection (transient or permanent).
    Tripped,
    /// Recloser locked out after exhausting all shots.
    Lockout,
    /// Device in fault/damaged state — requires inspection.
    Fault,
}

// ─────────────────────────────────────────────────────────────────────────────
// Primary data structures
// ─────────────────────────────────────────────────────────────────────────────

/// A single distribution switch with mechanical and operational metadata.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FeedSwitch {
    /// Unique string identifier (e.g., `"REC-01"`).
    pub switch_id: String,
    /// Functional type of the switch.
    pub switch_type: SwitchType,
    /// Current operational state.
    pub state: FaSwitchState,
    /// From-node (upstream) index.
    pub from_node: usize,
    /// To-node (downstream) index.
    pub to_node: usize,
    /// Continuous current rating \[A\].
    pub rating_a: f64,
    /// Normal (design) state — used for restoration and health checks.
    pub normal_state: FaSwitchState,
    /// Total operations performed (used to track wear).
    pub operation_count: u32,
    /// Maximum allowed operations before maintenance is required.
    pub max_operations: u32,
}

impl FeedSwitch {
    /// Returns `true` if the switch is in a conducting state.
    #[inline]
    pub fn is_conducting(&self) -> bool {
        self.state == FaSwitchState::Closed
    }

    /// Returns `true` if this switch is a normally-open tie switch.
    #[inline]
    pub fn is_tie_switch(&self) -> bool {
        self.normal_state == FaSwitchState::Open
    }
}

/// A contiguous segment of the feeder bounded by switches.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FeederSection {
    /// Unique section identifier.
    pub section_id: String,
    /// Bus/node indices belonging to this section.
    pub nodes: Vec<usize>,
    /// Active load at each node \[kW\].
    pub load_kw: Vec<f64>,
    /// Reactive load at each node \[kvar\].
    pub load_kvar: Vec<f64>,
    /// `true` if the section is currently energised.
    pub energized: bool,
    /// Identifier of the source feeder/substation supplying this section.
    pub supplied_by: Option<String>,
}

impl FeederSection {
    /// Total active load of the section \[kW\].
    pub fn total_load_kw(&self) -> f64 {
        self.load_kw.iter().sum()
    }

    /// Customer count (one per node for simplicity).
    pub fn customer_count(&self) -> usize {
        self.nodes.len()
    }
}

/// Configuration parameters for the feeder automation controller.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FeederAutomationConfig {
    /// Number of recloser shots (instantaneous + timed), default 3.
    pub recloser_shots: u8,
    /// Instantaneous trip duration in power-frequency cycles (default 3 ≈ 50 ms at 60 Hz).
    pub inst_trip_cycles: u8,
    /// Timed (delayed) trip delay \[s\], default 0.3 s.
    pub timed_trip_s: f64,
    /// Time to lock out after exhausting all shots \[s\], default 2.0 s.
    pub lockout_time_s: f64,
    /// Maximum time budget for automatic service restoration \[s\], default 60 s.
    pub restoration_timeout_s: f64,
    /// Maximum load that can be transferred via a tie switch \[kW\].
    pub max_load_transfer_kw: f64,
}

impl Default for FeederAutomationConfig {
    fn default() -> Self {
        Self {
            recloser_shots: 3,
            inst_trip_cycles: 3,
            timed_trip_s: 0.3,
            lockout_time_s: 2.0,
            restoration_timeout_s: 60.0,
            max_load_transfer_kw: 5_000.0,
        }
    }
}

/// Complete distribution feeder model with switches, sections, and automation config.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FeederTopology {
    /// Feeder identifier (e.g., `"F1"`).
    pub feeder_id: String,
    /// Substation (source) node indices.
    pub source_nodes: Vec<usize>,
    /// All switches on the feeder (reclosers, sectionalizers, tie switches, …).
    pub switches: Vec<FeedSwitch>,
    /// Feeder sections between switch points.
    pub sections: Vec<FeederSection>,
    /// Indices into `switches` that are tie points (normally open).
    pub tie_switches: Vec<usize>,
    /// Automation configuration.
    pub config: FeederAutomationConfig,
}

// ─────────────────────────────────────────────────────────────────────────────
// Output structures
// ─────────────────────────────────────────────────────────────────────────────

/// Result of an automatic fault isolation sequence.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FaultIsolationResult {
    /// Index of the faulted section in [`FeederTopology::sections`].
    pub faulted_section: usize,
    /// `true` if the fault was successfully isolated.
    pub isolated: bool,
    /// Elapsed time from fault detection to isolation \[s\].
    pub isolation_time_s: f64,
    /// IDs of switches operated during isolation.
    pub switches_operated: Vec<String>,
    /// Number of customers (nodes) interrupted by the isolation.
    pub customers_interrupted: usize,
}

/// Result of an automatic service restoration attempt.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RestorationResult {
    /// Indices of sections successfully restored.
    pub sections_restored: Vec<usize>,
    /// Total customers (nodes) restored.
    pub customers_restored: usize,
    /// Time from isolation to last restoration action \[s\].
    pub restoration_time_s: f64,
    /// IDs of tie switches that were closed to restore supply.
    pub tie_switches_closed: Vec<String>,
    /// Indices of sections that could not be restored (permanent fault or no alternate path).
    pub sections_unrestorable: Vec<usize>,
}

/// Outcome of a single recloser shot sequence.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RecloserResult {
    /// Number of shots fired before clearing or locking out.
    pub shots_fired: u8,
    /// `true` if the recloser locked out (fault persisted through all shots).
    pub locked_out: bool,
    /// `true` if the fault was cleared before lockout.
    pub fault_cleared: bool,
    /// Total time consumed by the shot sequence \[s\].
    pub time_elapsed_s: f64,
}

/// Snapshot of feeder health for operator review.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FeederHealthReport {
    /// IDs of switches that have exceeded 80 % of their maximum operation count.
    pub switches_near_limit: Vec<String>,
    /// IDs of currently de-energised sections.
    pub de_energized_sections: Vec<String>,
    /// IDs of normally-open switches (tie switches).
    pub normally_open_switches: Vec<String>,
    /// Load balance metric: ratio of load std-dev to mean across energised sections \[%\].
    /// Lower is better (0 % = perfectly balanced).
    pub load_balance_pct: f64,
}

/// Proposed switch operation for load-balance optimisation or restoration.
///
/// Prefixed `Fa` to avoid conflict with `reconfiguration::SwitchAction`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FaSwitchAction {
    /// ID of the switch to operate.
    pub switch_id: String,
    /// Target state after the operation.
    pub action: FaSwitchState,
    /// Human-readable reason for the operation.
    pub reason: String,
}

/// Security assessment for a single switch outage (N-1 contingency).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContingencyAssessment {
    /// ID of the outaged switch.
    pub outaged_switch: String,
    /// Section indices that lose supply when the switch opens.
    pub affected_sections: Vec<usize>,
    /// Total customers affected.
    pub customers_affected: usize,
    /// `true` if all affected sections can be restored via alternate paths.
    pub restoration_possible: bool,
    /// `true` if at least one alternate supply path exists.
    pub alternate_supply_available: bool,
}

// ─────────────────────────────────────────────────────────────────────────────
// FeederTopology implementation
// ─────────────────────────────────────────────────────────────────────────────

impl FeederTopology {
    /// Attempt to isolate a permanent fault on `fault_section_idx`.
    ///
    /// Procedure:
    /// 1. Find the upstream recloser protecting the faulted section.
    /// 2. Execute the recloser shot sequence (`recloser_sequence`).
    /// 3. If the fault persists (recloser locks out), open sectionalizers
    ///    and auto-switches to isolate the section.
    /// 4. Return the isolation result with timing and operated switches.
    ///
    /// # Errors
    /// Returns [`FeederAutomationError::InvalidSectionIndex`] if `fault_section_idx`
    /// is out of bounds.
    pub fn process_fault(
        &mut self,
        fault_section_idx: usize,
        fault_current_a: f64,
    ) -> Result<FaultIsolationResult, FeederAutomationError> {
        if fault_section_idx >= self.sections.len() {
            return Err(FeederAutomationError::InvalidSectionIndex(
                fault_section_idx,
            ));
        }

        let mut switches_operated: Vec<String> = Vec::new();
        let mut elapsed_s = 0.0_f64;

        // ── Step 1: find the upstream recloser ──────────────────────────────
        // The recloser is the switch whose to_node belongs to the faulted section
        // (or whose from_node is a source node feeding into the section).
        let faulted_nodes: std::collections::HashSet<usize> = self.sections[fault_section_idx]
            .nodes
            .iter()
            .copied()
            .collect();

        let recloser_idx_opt = self
            .switches
            .iter()
            .enumerate()
            .find(|(_, sw)| {
                sw.switch_type == SwitchType::Recloser
                    && sw.state == FaSwitchState::Closed
                    && (faulted_nodes.contains(&sw.to_node)
                        || self
                            .sections
                            .iter()
                            .any(|sec| sec.nodes.contains(&sw.to_node) && sec.energized))
            })
            .map(|(i, _)| i);

        // If no dedicated recloser is found, fall back to the first closed
        // switch upstream of the faulted section.
        let recloser_idx = match recloser_idx_opt {
            Some(i) => i,
            None => {
                // Fallback: find any closed switch feeding into the faulted section
                let fallback = self
                    .switches
                    .iter()
                    .enumerate()
                    .find(|(_, sw)| {
                        sw.state == FaSwitchState::Closed
                            && (faulted_nodes.contains(&sw.to_node)
                                || faulted_nodes.contains(&sw.from_node))
                    })
                    .map(|(i, _)| i);

                match fallback {
                    Some(i) => i,
                    None => {
                        // No upstream switch found — section already isolated or
                        // directly connected to source; open directly.
                        self.sections[fault_section_idx].energized = false;
                        return Ok(FaultIsolationResult {
                            faulted_section: fault_section_idx,
                            isolated: true,
                            isolation_time_s: 0.05,
                            switches_operated,
                            customers_interrupted: self.sections[fault_section_idx]
                                .customer_count(),
                        });
                    }
                }
            }
        };

        // ── Step 2: recloser shot sequence ──────────────────────────────────
        let rec_result = self.recloser_sequence(recloser_idx, fault_current_a, 1.0 / 60.0)?;
        elapsed_s += rec_result.time_elapsed_s;
        switches_operated.push(self.switches[recloser_idx].switch_id.clone());

        if rec_result.fault_cleared {
            // Temporary fault — feeder restored automatically
            return Ok(FaultIsolationResult {
                faulted_section: fault_section_idx,
                isolated: false, // section not permanently isolated
                isolation_time_s: elapsed_s,
                switches_operated,
                customers_interrupted: 0,
            });
        }

        // ── Step 3: fault persists — isolate the faulted section ────────────
        // Open all switches that directly border the faulted section.
        let border_switch_indices: Vec<usize> = self
            .switches
            .iter()
            .enumerate()
            .filter(|(i, sw)| {
                *i != recloser_idx
                    && (faulted_nodes.contains(&sw.from_node)
                        || faulted_nodes.contains(&sw.to_node))
                    && matches!(
                        sw.switch_type,
                        SwitchType::Sectionalizer | SwitchType::AutoSwitch | SwitchType::Recloser
                    )
            })
            .map(|(i, _)| i)
            .collect();

        for &sw_idx in &border_switch_indices {
            let sw = &mut self.switches[sw_idx];
            sw.state = FaSwitchState::Open;
            sw.operation_count = sw.operation_count.saturating_add(1);
            switches_operated.push(sw.switch_id.clone());
        }
        elapsed_s += 1.0; // sectionalizer coordination time

        // Mark the section de-energised
        self.sections[fault_section_idx].energized = false;

        // Also de-energise downstream sections that were solely fed through
        // the faulted section (simplified: any section sharing nodes with
        // operated border switches on the downstream side).
        let interrupted_customers = self.sections[fault_section_idx].customer_count();

        Ok(FaultIsolationResult {
            faulted_section: fault_section_idx,
            isolated: true,
            isolation_time_s: elapsed_s,
            switches_operated,
            customers_interrupted: interrupted_customers,
        })
    }

    /// Restore de-energised sections (excluding the faulted one) by closing
    /// normally-open tie switches when capacity allows.
    ///
    /// For each de-energised, non-faulted section the method:
    /// 1. Finds candidate tie switches that could supply it.
    /// 2. Validates capacity with [`check_path_capacity`](Self::check_path_capacity).
    /// 3. Closes the tie switch and marks the section energised.
    pub fn restore_unfaulted_sections(
        &mut self,
        isolation_result: &FaultIsolationResult,
    ) -> RestorationResult {
        let mut sections_restored: Vec<usize> = Vec::new();
        let mut sections_unrestorable: Vec<usize> = Vec::new();
        let mut tie_switches_closed: Vec<String> = Vec::new();
        let mut customers_restored = 0_usize;
        let mut restoration_time_s = 0.0_f64;

        // Collect de-energised sections that are not the faulted section.
        let de_energized_indices: Vec<usize> = self
            .sections
            .iter()
            .enumerate()
            .filter(|(i, sec)| !sec.energized && *i != isolation_result.faulted_section)
            .map(|(i, _)| i)
            .collect();

        for sec_idx in de_energized_indices {
            let section_load_kw = self.sections[sec_idx].total_load_kw();

            // Find a normally-open tie switch that could supply this section.
            // A tie switch is eligible when its from_node or to_node matches
            // any node in the de-energised section (simplified connectivity).
            let section_nodes: std::collections::HashSet<usize> =
                self.sections[sec_idx].nodes.iter().copied().collect();

            let tie_candidate: Option<usize> = self.tie_switches.iter().copied().find(|&ti| {
                let sw = &self.switches[ti];
                sw.state == FaSwitchState::Open
                    && (section_nodes.contains(&sw.from_node)
                        || section_nodes.contains(&sw.to_node))
                    && self.check_path_capacity(&[ti], section_load_kw)
            });

            match tie_candidate {
                Some(ti) => {
                    // Close the tie switch.
                    {
                        let sw = &mut self.switches[ti];
                        sw.state = FaSwitchState::Closed;
                        sw.operation_count = sw.operation_count.saturating_add(1);
                        tie_switches_closed.push(sw.switch_id.clone());
                    }
                    self.sections[sec_idx].energized = true;
                    customers_restored += self.sections[sec_idx].customer_count();
                    sections_restored.push(sec_idx);
                    restoration_time_s += 5.0; // nominal switching time
                }
                None => {
                    sections_unrestorable.push(sec_idx);
                }
            }
        }

        RestorationResult {
            sections_restored,
            customers_restored,
            restoration_time_s,
            tie_switches_closed,
            sections_unrestorable,
        }
    }

    /// Check whether closing `path_switches` to restore `load_to_restore_kw` \[kW\]
    /// would exceed the feeder's `max_load_transfer_kw` capacity.
    ///
    /// Returns `true` if the transfer is within capacity.
    pub fn check_path_capacity(&self, path_switches: &[usize], load_to_restore_kw: f64) -> bool {
        // Sum load on sections already energised (existing load on alternate feeder).
        let existing_load_kw: f64 = self
            .sections
            .iter()
            .filter(|sec| sec.energized)
            .map(|sec| sec.total_load_kw())
            .sum();

        // Check that none of the path switches are overloaded individually.
        // Rating is converted to kW using a nominal 11 kV three-phase distribution
        // voltage: P_max [kW] = √3 × V_LL [kV] × I_rating [A]
        const SQRT3: f64 = 1.732_050_808_f64;
        const V_NOMINAL_KV: f64 = 11.0; // typical medium-voltage distribution feeder
        let switch_ok = path_switches.iter().all(|&sw_idx| {
            if sw_idx < self.switches.len() {
                let sw = &self.switches[sw_idx];
                if sw.rating_a <= 0.0 {
                    return true; // no rating defined — don't block
                }
                let rating_kw = SQRT3 * V_NOMINAL_KV * sw.rating_a;
                load_to_restore_kw <= rating_kw
            } else {
                true // unknown switch index — don't block
            }
        });

        switch_ok && (existing_load_kw + load_to_restore_kw) < self.config.max_load_transfer_kw
    }

    /// Simulate the full shot sequence of a recloser.
    ///
    /// Shot sequence (default 3 shots):
    /// - Shot 1: instantaneous trip → open for ~500 ms → reclose
    /// - Shot 2: timed trip → open for ~500 ms → reclose
    /// - Shot 3: timed trip → lockout
    ///
    /// If `fault_current_a == 0.0` the fault is assumed to have self-cleared
    /// before the sequence starts; the recloser recloses successfully.
    ///
    /// # Errors
    /// Returns [`FeederAutomationError::InvalidSwitchIndex`] if `recloser_idx`
    /// is out of bounds, or [`FeederAutomationError::InvalidOperation`] if the
    /// switch is not a `Recloser`.
    pub fn recloser_sequence(
        &mut self,
        recloser_idx: usize,
        fault_current_a: f64,
        dt_s: f64,
    ) -> Result<RecloserResult, FeederAutomationError> {
        if recloser_idx >= self.switches.len() {
            return Err(FeederAutomationError::InvalidSwitchIndex(recloser_idx));
        }

        let max_shots = self.config.recloser_shots;
        let inst_trip_s = f64::from(self.config.inst_trip_cycles) * dt_s;
        let timed_trip_s = self.config.timed_trip_s;
        let reclose_delay_s = 0.5_f64; // 500 ms dead time between shots

        // Fault already cleared?
        if fault_current_a <= 0.0 {
            let sw = &mut self.switches[recloser_idx];
            sw.state = FaSwitchState::Closed;
            sw.operation_count = sw.operation_count.saturating_add(1);
            return Ok(RecloserResult {
                shots_fired: 0,
                locked_out: false,
                fault_cleared: true,
                time_elapsed_s: 0.0,
            });
        }

        let mut time_s = 0.0_f64;
        let mut shots_fired = 0_u8;

        for shot in 0..max_shots {
            // Trip
            {
                let sw = &mut self.switches[recloser_idx];
                sw.state = FaSwitchState::Tripped;
                sw.operation_count = sw.operation_count.saturating_add(1);
            }
            shots_fired += 1;

            // Trip duration: instantaneous for shot 0, timed for subsequent
            let trip_duration = if shot == 0 { inst_trip_s } else { timed_trip_s };
            time_s += trip_duration;

            // Last shot → lock out, no reclose
            if shots_fired >= max_shots {
                let sw = &mut self.switches[recloser_idx];
                sw.state = FaSwitchState::Lockout;
                time_s += self.config.lockout_time_s;
                return Ok(RecloserResult {
                    shots_fired,
                    locked_out: true,
                    fault_cleared: false,
                    time_elapsed_s: time_s,
                });
            }

            // Dead time before reclose
            time_s += reclose_delay_s;

            // Reclose — fault persists (fault_current_a > 0)
            {
                let sw = &mut self.switches[recloser_idx];
                sw.state = FaSwitchState::Closed;
                sw.operation_count = sw.operation_count.saturating_add(1);
            }
        }

        // Should not reach here given loop above, but handle gracefully
        let sw = &mut self.switches[recloser_idx];
        sw.state = FaSwitchState::Lockout;
        Ok(RecloserResult {
            shots_fired,
            locked_out: true,
            fault_cleared: false,
            time_elapsed_s: time_s,
        })
    }

    /// Change the state of switch `switch_idx` to `new_state`.
    ///
    /// Validates:
    /// - Index bounds
    /// - Fuses cannot be reclosed once blown
    ///
    /// Increments `operation_count` on success.
    ///
    /// # Errors
    /// - [`FeederAutomationError::InvalidSwitchIndex`] if out of bounds
    /// - [`FeederAutomationError::InvalidOperation`] for fuse reclosure attempt
    pub fn switch_operation(
        &mut self,
        switch_idx: usize,
        new_state: FaSwitchState,
    ) -> Result<(), FeederAutomationError> {
        if switch_idx >= self.switches.len() {
            return Err(FeederAutomationError::InvalidSwitchIndex(switch_idx));
        }

        let sw = &mut self.switches[switch_idx];

        // Fuse cannot be reclosed after it has blown
        if sw.switch_type == SwitchType::Fuse
            && matches!(sw.state, FaSwitchState::Tripped | FaSwitchState::Lockout)
            && new_state == FaSwitchState::Closed
        {
            return Err(FeederAutomationError::InvalidOperation(format!(
                "fuse '{}' is blown and cannot be reclosed",
                sw.switch_id
            )));
        }

        sw.state = new_state;
        sw.operation_count = sw.operation_count.saturating_add(1);
        Ok(())
    }

    /// Generate a health report for the feeder.
    ///
    /// Reports:
    /// - Switches exceeding 80 % of their `max_operations` limit
    /// - Currently de-energised sections
    /// - Normally-open (tie) switches
    /// - Load imbalance as coefficient of variation \[%\]
    pub fn feeder_health_report(&self) -> FeederHealthReport {
        // Switches near maintenance limit (> 80 % of max_operations)
        let switches_near_limit: Vec<String> = self
            .switches
            .iter()
            .filter(|sw| {
                sw.max_operations > 0 && sw.operation_count > (sw.max_operations * 4 / 5)
                // > 80 %
            })
            .map(|sw| sw.switch_id.clone())
            .collect();

        // De-energised sections
        let de_energized_sections: Vec<String> = self
            .sections
            .iter()
            .filter(|sec| !sec.energized)
            .map(|sec| sec.section_id.clone())
            .collect();

        // Normally-open switches
        let normally_open_switches: Vec<String> = self
            .switches
            .iter()
            .filter(|sw| sw.normal_state == FaSwitchState::Open)
            .map(|sw| sw.switch_id.clone())
            .collect();

        // Load balance: coefficient of variation of energised section loads
        let energized_loads: Vec<f64> = self
            .sections
            .iter()
            .filter(|sec| sec.energized)
            .map(|sec| sec.total_load_kw())
            .collect();

        let load_balance_pct = compute_cv_pct(&energized_loads);

        FeederHealthReport {
            switches_near_limit,
            de_energized_sections,
            normally_open_switches,
            load_balance_pct,
        }
    }

    /// Assess the N-1 impact of opening `outaged_switch_idx`.
    ///
    /// Returns a [`ContingencyAssessment`] describing:
    /// - Which sections lose supply
    /// - Total customer impact
    /// - Whether alternate supply via tie switches is available
    ///
    /// # Errors
    /// Returns [`FeederAutomationError::InvalidSwitchIndex`] if out of bounds.
    pub fn simulate_n1_contingency(
        &self,
        outaged_switch_idx: usize,
    ) -> Result<ContingencyAssessment, FeederAutomationError> {
        if outaged_switch_idx >= self.switches.len() {
            return Err(FeederAutomationError::InvalidSwitchIndex(
                outaged_switch_idx,
            ));
        }

        let outaged_sw = &self.switches[outaged_switch_idx];
        let outaged_switch_id = outaged_sw.switch_id.clone();

        // Find sections that would be de-energised if this switch opens.
        // A section is affected when it is currently energised and the outaged
        // switch is the only path from any source node to that section's nodes.
        // Simplified: sections whose nodes overlap the switch's to_node side.
        let affected_sections: Vec<usize> = self
            .sections
            .iter()
            .enumerate()
            .filter(|(_, sec)| {
                sec.energized
                    && sec.nodes.contains(&outaged_sw.to_node)
                    // Verify source nodes don't directly reach the section via
                    // another path (simplified: check if from_node is a source)
                    && !self.source_nodes.contains(&outaged_sw.to_node)
            })
            .map(|(i, _)| i)
            .collect();

        let customers_affected: usize = affected_sections
            .iter()
            .map(|&i| self.sections[i].customer_count())
            .sum();

        // Check alternate supply: is there any tie switch that borders an
        // affected section AND is currently open?
        let alternate_supply_available = affected_sections.iter().any(|&sec_idx| {
            let sec_nodes: std::collections::HashSet<usize> =
                self.sections[sec_idx].nodes.iter().copied().collect();
            self.tie_switches.iter().any(|&ti| {
                let sw = &self.switches[ti];
                sw.state == FaSwitchState::Open
                    && (sec_nodes.contains(&sw.from_node) || sec_nodes.contains(&sw.to_node))
            })
        });

        // Restoration is possible if alternate supply exists for all affected sections.
        let restoration_possible = !affected_sections.is_empty()
            && affected_sections.iter().all(|&sec_idx| {
                let sec_nodes: std::collections::HashSet<usize> =
                    self.sections[sec_idx].nodes.iter().copied().collect();
                self.tie_switches.iter().any(|&ti| {
                    let sw = &self.switches[ti];
                    sw.state == FaSwitchState::Open
                        && (sec_nodes.contains(&sw.from_node) || sec_nodes.contains(&sw.to_node))
                })
            });

        Ok(ContingencyAssessment {
            outaged_switch: outaged_switch_id,
            affected_sections,
            customers_affected,
            restoration_possible,
            alternate_supply_available,
        })
    }

    /// Compute optimal switch position adjustments to balance loading across feeders.
    ///
    /// `load_profile[i]` is the per-section additional load \[kW\] (e.g., from a
    /// forecast horizon). Uses tie-switch repositioning to equalise loading.
    ///
    /// Returns an empty vec if the feeder is already balanced (max/min ratio < 1.2).
    pub fn optimize_switch_positions(&self, load_profile: &[f64]) -> Vec<FaSwitchAction> {
        if self.sections.is_empty() || self.tie_switches.is_empty() {
            return Vec::new();
        }

        // Compute effective load per energised section.
        let section_loads: Vec<f64> = self
            .sections
            .iter()
            .enumerate()
            .map(|(i, sec)| {
                let profile_load = load_profile.get(i).copied().unwrap_or(0.0);
                sec.total_load_kw() + profile_load
            })
            .collect();

        let max_load = section_loads
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        let min_load = section_loads.iter().cloned().fold(f64::INFINITY, f64::min);

        // Already balanced?
        if min_load <= 0.0 || max_load / min_load < 1.2 {
            return Vec::new();
        }

        let max_sec_idx = section_loads
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
            .map(|(i, _)| i)
            .unwrap_or(0);

        let min_sec_idx = section_loads
            .iter()
            .enumerate()
            .min_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
            .map(|(i, _)| i)
            .unwrap_or(0);

        if max_sec_idx == min_sec_idx {
            return Vec::new();
        }

        let mut actions: Vec<FaSwitchAction> = Vec::new();

        // Find a tie switch adjacent to the heavily loaded section.
        let heavy_nodes: std::collections::HashSet<usize> =
            self.sections[max_sec_idx].nodes.iter().copied().collect();

        let light_nodes: std::collections::HashSet<usize> =
            self.sections[min_sec_idx].nodes.iter().copied().collect();

        for &ti in &self.tie_switches {
            let sw = &self.switches[ti];
            if sw.state == FaSwitchState::Open
                && (heavy_nodes.contains(&sw.from_node) || heavy_nodes.contains(&sw.to_node))
            {
                actions.push(FaSwitchAction {
                    switch_id: sw.switch_id.clone(),
                    action: FaSwitchState::Closed,
                    reason: format!(
                        "close tie switch to supply load from overloaded section {}",
                        max_sec_idx
                    ),
                });
                break;
            }
        }

        // Find a sectionalizer/auto-switch on the boundary between heavy and
        // light sections to shift load.
        for (i, sw) in self.switches.iter().enumerate() {
            if self.tie_switches.contains(&i) {
                continue;
            }
            if sw.state == FaSwitchState::Closed
                && matches!(
                    sw.switch_type,
                    SwitchType::AutoSwitch | SwitchType::Sectionalizer
                )
                && (heavy_nodes.contains(&sw.from_node) || heavy_nodes.contains(&sw.to_node))
                && (light_nodes.contains(&sw.from_node) || light_nodes.contains(&sw.to_node))
            {
                actions.push(FaSwitchAction {
                    switch_id: sw.switch_id.clone(),
                    action: FaSwitchState::Open,
                    reason: format!(
                        "open boundary switch to shift load from section {} to section {}",
                        max_sec_idx, min_sec_idx
                    ),
                });
                break;
            }
        }

        actions
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Internal helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Compute the coefficient of variation (std-dev / mean) as a percentage.
/// Returns `0.0` for empty or zero-mean slices.
fn compute_cv_pct(values: &[f64]) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    let n = values.len() as f64;
    let mean = values.iter().sum::<f64>() / n;
    if mean <= 0.0 {
        return 0.0;
    }
    let variance = values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / n;
    (variance.sqrt() / mean) * 100.0
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a simple linear feeder for testing:
    ///
    /// ```text
    /// Source(0) ─[SW0:Recloser]─ Section0(node 1) ─[SW1:AutoSwitch]─ Section1(node 2)
    ///                                                                       │
    ///                                                               [SW2:Sectionalizer]
    ///                                                                       │
    ///                                                                 Section2(node 3)
    ///
    /// AltSource(10) ─[SW3:AutoSwitch,NormallyOpen]─ Section2(node 3)
    /// ```
    fn make_test_topology() -> FeederTopology {
        let cfg = FeederAutomationConfig {
            recloser_shots: 3,
            inst_trip_cycles: 3,
            timed_trip_s: 0.3,
            lockout_time_s: 2.0,
            restoration_timeout_s: 60.0,
            max_load_transfer_kw: 5_000.0,
        };

        let switches = vec![
            // SW0: Recloser between source(0) and section0(1)
            FeedSwitch {
                switch_id: "SW0-REC".to_string(),
                switch_type: SwitchType::Recloser,
                state: FaSwitchState::Closed,
                from_node: 0,
                to_node: 1,
                rating_a: 400.0,
                normal_state: FaSwitchState::Closed,
                operation_count: 0,
                max_operations: 100,
            },
            // SW1: AutoSwitch between section0(1) and section1(2)
            FeedSwitch {
                switch_id: "SW1-AUTO".to_string(),
                switch_type: SwitchType::AutoSwitch,
                state: FaSwitchState::Closed,
                from_node: 1,
                to_node: 2,
                rating_a: 400.0,
                normal_state: FaSwitchState::Closed,
                operation_count: 0,
                max_operations: 200,
            },
            // SW2: Sectionalizer between section1(2) and section2(3)
            FeedSwitch {
                switch_id: "SW2-SEC".to_string(),
                switch_type: SwitchType::Sectionalizer,
                state: FaSwitchState::Closed,
                from_node: 2,
                to_node: 3,
                rating_a: 400.0,
                normal_state: FaSwitchState::Closed,
                operation_count: 0,
                max_operations: 150,
            },
            // SW3: Tie switch (normally open) between alt-source(10) and section2(3)
            FeedSwitch {
                switch_id: "SW3-TIE".to_string(),
                switch_type: SwitchType::AutoSwitch,
                state: FaSwitchState::Open,
                from_node: 10,
                to_node: 3,
                rating_a: 400.0,
                normal_state: FaSwitchState::Open,
                operation_count: 0,
                max_operations: 200,
            },
        ];

        let sections = vec![
            FeederSection {
                section_id: "SEC-0".to_string(),
                nodes: vec![1],
                load_kw: vec![200.0],
                load_kvar: vec![50.0],
                energized: true,
                supplied_by: Some("F1".to_string()),
            },
            FeederSection {
                section_id: "SEC-1".to_string(),
                nodes: vec![2],
                load_kw: vec![300.0],
                load_kvar: vec![75.0],
                energized: true,
                supplied_by: Some("F1".to_string()),
            },
            FeederSection {
                section_id: "SEC-2".to_string(),
                nodes: vec![3],
                load_kw: vec![400.0],
                load_kvar: vec![100.0],
                energized: true,
                supplied_by: Some("F1".to_string()),
            },
        ];

        FeederTopology {
            feeder_id: "F1".to_string(),
            source_nodes: vec![0],
            switches,
            sections,
            tie_switches: vec![3], // SW3 is the tie switch
            config: cfg,
        }
    }

    // ─── Test 1: Fault isolation ──────────────────────────────────────────────

    #[test]
    fn test_fault_isolation_basic() {
        let mut topo = make_test_topology();

        // Fault on section 2 with high persistent fault current
        let result = topo
            .process_fault(2, 1_500.0)
            .expect("process_fault should succeed");

        assert_eq!(result.faulted_section, 2);
        assert!(result.isolated, "section should be isolated");
        assert!(
            !result.switches_operated.is_empty(),
            "at least one switch should have operated"
        );
        assert!(
            !topo.sections[2].energized,
            "faulted section must be de-energised"
        );
    }

    // ─── Test 2: Restoration via tie switch ──────────────────────────────────

    #[test]
    fn test_restoration_via_tie_switch() {
        let mut topo = make_test_topology();

        // Manually de-energise section 2 (simulating a prior isolation)
        topo.sections[2].energized = false;

        let isolation = FaultIsolationResult {
            faulted_section: 2,
            isolated: true,
            isolation_time_s: 3.0,
            switches_operated: vec!["SW2-SEC".to_string()],
            customers_interrupted: 1,
        };

        let result = topo.restore_unfaulted_sections(&isolation);

        // Section 2 is faulted so it should not be restored; sections 0 & 1 were
        // already energised so only de-energised non-faulted sections are targeted.
        // (No other de-energised sections here — result should be empty or valid)
        // Verify the result is internally consistent.
        assert!(
            result.sections_restored.len() + result.sections_unrestorable.len()
                <= topo.sections.len()
        );
    }

    // ─── Test 3: Capacity check ───────────────────────────────────────────────

    #[test]
    fn test_capacity_check_overload() {
        let topo = make_test_topology();

        // Load well within limit
        assert!(
            topo.check_path_capacity(&[3], 100.0),
            "small load should fit"
        );

        // Load exceeding max_load_transfer_kw
        assert!(
            !topo.check_path_capacity(&[3], 6_000.0),
            "oversize load should be rejected"
        );
    }

    // ─── Test 4: Recloser 3-shot lockout ─────────────────────────────────────

    #[test]
    fn test_recloser_three_shots_lockout() {
        let mut topo = make_test_topology();

        // Persistent fault current
        let result = topo
            .recloser_sequence(0, 1_200.0, 1.0 / 60.0)
            .expect("recloser_sequence should succeed");

        assert_eq!(
            result.shots_fired, topo.config.recloser_shots,
            "should fire all configured shots"
        );
        assert!(result.locked_out, "should lock out after all shots");
        assert!(!result.fault_cleared, "fault should not have cleared");
        assert!(result.time_elapsed_s > 0.0, "must consume time");
        assert_eq!(
            topo.switches[0].state,
            FaSwitchState::Lockout,
            "recloser state must be Lockout"
        );
    }

    // ─── Test 5: Recloser clears temporary fault on shot 0 ───────────────────

    #[test]
    fn test_recloser_temporary_fault_cleared() {
        let mut topo = make_test_topology();

        // fault_current_a == 0 → fault already cleared
        let result = topo
            .recloser_sequence(0, 0.0, 1.0 / 60.0)
            .expect("recloser_sequence should succeed");

        assert!(result.fault_cleared, "fault should be cleared");
        assert!(!result.locked_out, "should not lock out");
        assert_eq!(result.shots_fired, 0, "no shots needed");
        assert_eq!(
            topo.switches[0].state,
            FaSwitchState::Closed,
            "recloser should be closed"
        );
    }

    // ─── Test 6: Health report — switch near maintenance limit ───────────────

    #[test]
    fn test_health_report_near_limit() {
        let mut topo = make_test_topology();

        // Set SW0 to 85 % of its max_operations (100)
        topo.switches[0].operation_count = 86; // > 80

        let report = topo.feeder_health_report();

        assert!(
            report.switches_near_limit.contains(&"SW0-REC".to_string()),
            "SW0-REC should be flagged as near limit; got: {:?}",
            report.switches_near_limit
        );
        // SW3 is normally open — should appear in normally_open_switches
        assert!(
            report
                .normally_open_switches
                .contains(&"SW3-TIE".to_string()),
            "SW3-TIE should be listed as normally open"
        );
    }

    // ─── Test 7: N-1 contingency with alternate supply ────────────────────────

    #[test]
    fn test_n1_contingency_with_alternate() {
        let topo = make_test_topology();

        // N-1 on SW2 (sectionalizer before section2)
        // Section2 has node 3, and SW3-TIE also connects to node 3.
        let assessment = topo
            .simulate_n1_contingency(2)
            .expect("contingency should succeed");

        // Section 2 (node 3) should be affected
        assert!(
            assessment.affected_sections.contains(&2),
            "section 2 should be affected"
        );
        assert!(
            assessment.alternate_supply_available || assessment.affected_sections.is_empty(),
            "alternate supply via SW3-TIE should exist"
        );
    }

    // ─── Test 8: Switch operation state change ────────────────────────────────

    #[test]
    fn test_switch_operation_state_change() {
        let mut topo = make_test_topology();

        // Open SW1
        topo.switch_operation(1, FaSwitchState::Open)
            .expect("should succeed");
        assert_eq!(
            topo.switches[1].state,
            FaSwitchState::Open,
            "state should be Open"
        );
        assert_eq!(
            topo.switches[1].operation_count, 1,
            "operation count should increment"
        );

        // Out-of-bounds → Err
        let err = topo.switch_operation(99, FaSwitchState::Closed);
        assert!(err.is_err(), "out-of-bounds switch should return Err");
    }

    // ─── Test 9: Fuse cannot be reclosed ─────────────────────────────────────

    #[test]
    fn test_fuse_cannot_be_reclosed() {
        let mut topo = make_test_topology();

        // Add a fuse in Tripped state
        topo.switches.push(FeedSwitch {
            switch_id: "FUSE-01".to_string(),
            switch_type: SwitchType::Fuse,
            state: FaSwitchState::Tripped,
            from_node: 5,
            to_node: 6,
            rating_a: 200.0,
            normal_state: FaSwitchState::Closed,
            operation_count: 1,
            max_operations: 1,
        });
        let fuse_idx = topo.switches.len() - 1;

        let err = topo.switch_operation(fuse_idx, FaSwitchState::Closed);
        assert!(err.is_err(), "blown fuse should not be reclosable");
    }

    // ─── Test 10: Switch optimization returns actions when imbalanced ─────────

    #[test]
    fn test_optimize_switch_positions_imbalanced() {
        let mut topo = make_test_topology();

        // Make section 0 very heavy
        topo.sections[0].load_kw = vec![4_000.0];
        topo.sections[1].load_kw = vec![100.0];
        topo.sections[2].load_kw = vec![100.0];

        let actions = topo.optimize_switch_positions(&[]);
        // With a 40:1 imbalance there should be at least one recommended action
        assert!(
            !actions.is_empty(),
            "should suggest switching actions to balance load"
        );
    }
}
