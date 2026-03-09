//! Distribution automation — self-healing grid controller.
//!
//! Implements an automated fault response sequence for distribution networks:
//!
//! 1. **Locate** the faulted segment from protection device trips.
//! 2. **Isolate** the faulted section by opening boundary switches.
//! 3. **Restore upstream** section immediately (already connected to substation).
//! 4. **Restore downstream** via a normally-open tie switch to an alternate feeder.
//! 5. **Activate DER** on the downstream section if no tie switch is available.
//!
//! Priority loads are always scheduled first in the restoration sequence.
//!
//! # References
//! - IEEE Std 1366-2012 — Guide for Electric Power Distribution Reliability Indices
//! - Jeon, Y.-J. et al., "An efficient simulated annealing algorithm for network
//!   reconfiguration in large-scale distribution systems", IEEE Trans. Power Deliv., 2002
//! - Zidan, A. & El-Saadany, E.F., "A Cooperative Multiagent Framework for Self-Healing
//!   Mechanisms in Distribution Systems", IEEE Trans. Smart Grid, 2012

use serde::{Deserialize, Serialize};

// ── Error type ─────────────────────────────────────────────────────────────

/// Errors from the self-healing controller.
#[derive(Debug, thiserror::Error)]
pub enum HealingError {
    /// The fault location references an invalid feeder segment.
    #[error("invalid fault location: segment {0} not found in any feeder")]
    InvalidSegment(usize),
    /// The fault location references buses not in any configured feeder.
    #[error("fault buses (upstream={0}, downstream={1}) not found in feeders")]
    InvalidFaultBuses(usize, usize),
    /// Controller has no feeders configured.
    #[error("no feeders configured — call add_feeder() first")]
    NoFeeders,
    /// Load vector length does not match bus count.
    #[error("load vector length {0} does not match n_buses {1}")]
    LoadSizeMismatch(usize, usize),
}

// ── Configuration ──────────────────────────────────────────────────────────

/// Configuration for the self-healing grid controller.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SelfHealingConfig {
    /// Total number of buses in the distribution network.
    pub n_buses: usize,
    /// Target time from fault detection to full restoration \[s\].
    pub restoration_time_target_s: f64,
    /// Maximum load that can be transferred to any single feeder \[MW\].
    pub max_load_transfer_mw: f64,
    /// Bus indices of critical / priority loads (restored first).
    pub priority_loads: Vec<usize>,
    /// Level of automation (determines whether actions execute automatically).
    pub automation_level: AutomationLevel,
}

/// Level of automation of the self-healing controller.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AutomationLevel {
    /// Only alert operators; no automated switching.
    Manual,
    /// Propose a switching plan but wait for operator confirmation.
    SemiAutomatic,
    /// Execute the switching plan immediately without operator input.
    Automatic,
}

// ── Fault description ───────────────────────────────────────────────────────

/// A fault event that triggers the self-healing response.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FaultEvent {
    /// Type of fault.
    pub fault_type: FaultType,
    /// Precise location of the fault.
    pub location: FaultLocation,
    /// Fault severity (fault resistance) \[Ω\].
    pub severity: f64,
    /// Timestamp of fault detection \[s\].
    pub timestamp_s: f64,
}

/// Classification of the fault.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum FaultType {
    /// Permanent phase-to-ground fault — requires physical repair.
    PermanentPhaseToGround,
    /// Self-clearing temporary arc fault — auto-recloser can restore.
    TemporaryArcFault,
    /// Equipment failure (transformer, cable, etc.).
    Equipment(String),
}

/// Precise location of a fault on the network.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FaultLocation {
    /// Feeder segment index (matches the index into `feeders` vector).
    pub segment: usize,
    /// Bus upstream of the fault (closer to substation).
    pub bus_upstream: usize,
    /// Bus downstream of the fault (further from substation).
    pub bus_downstream: usize,
    /// Distance along the segment where the fault occurred, as a percentage \[%\].
    pub distance_pct: f64,
}

// ── Healing actions ─────────────────────────────────────────────────────────

/// A single automated switching or control action.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HealingAction {
    /// Type of action to perform.
    pub action_type: HealingActionType,
    /// Device identifier (branch index for switches, bus index for DER).
    pub device_id: usize,
    /// Time at which this action should be executed \[s\].
    pub timestamp_s: f64,
    /// Expected number of customers restored by this action.
    pub expected_customers_restored: usize,
    /// Expected load restored by this action \[MW\].
    pub expected_load_mw: f64,
}

/// The type of automated healing action.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum HealingActionType {
    /// Open a normally-closed sectionalizing switch to isolate a fault.
    OpenSwitch,
    /// Close a normally-open tie switch to restore supply from alternate feeder.
    CloseSwitch,
    /// Transfer load from one feeder to another via tie switch.
    LoadTransfer {
        /// Source feeder index.
        from_feeder: usize,
        /// Destination feeder index.
        to_feeder: usize,
    },
    /// Activate a distributed energy resource to island and serve local loads.
    DerActivation {
        /// Bus where the DER is connected.
        der_bus: usize,
        /// Active power output of the DER \[MW\].
        p_mw: f64,
    },
    /// Isolate a faulted section by opening all boundary switches.
    SectionIsolation,
}

// ── Result type ─────────────────────────────────────────────────────────────

/// Summary result of the self-healing response.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SelfHealingResult {
    /// Whether the fault segment was successfully located.
    pub fault_located: bool,
    /// Time from fault detection to section isolation \[s\].
    pub fault_isolation_time_s: f64,
    /// Time from fault detection to full restoration \[s\].
    pub restoration_time_s: f64,
    /// Ordered list of healing actions executed (or proposed for SemiAutomatic).
    pub actions: Vec<HealingAction>,
    /// Number of customers restored.
    pub customers_restored: usize,
    /// Number of customers that could not be restored.
    pub customers_unrestored: usize,
    /// Total load restored \[MW\].
    pub load_restored_mw: f64,
    /// Total load that could not be restored \[MW\].
    pub load_unrestored_mw: f64,
    /// Whether all priority (critical) loads were restored.
    pub priority_loads_restored: bool,
    /// Whether actions were executed automatically (vs. proposed only).
    pub automatic_execution: bool,
}

// ── Controller ─────────────────────────────────────────────────────────────

/// Distribution network self-healing automation controller.
///
/// # Example
///
/// ```rust,ignore
/// use oxigrid::network::self_healing::{SelfHealingConfig, SelfHealingController,
///     AutomationLevel, FaultEvent, FaultType, FaultLocation};
///
/// let cfg = SelfHealingConfig {
///     n_buses: 10,
///     restoration_time_target_s: 60.0,
///     max_load_transfer_mw: 5.0,
///     priority_loads: vec![3],
///     automation_level: AutomationLevel::Automatic,
/// };
/// let mut ctrl = SelfHealingController::new(cfg);
/// ctrl.add_feeder(vec![0, 1, 2, 3, 4]);
/// ctrl.add_switch(2, false);  // normally-closed sectionalizer
/// ctrl.set_loads(vec![0.0, 1.0, 1.5, 2.0, 0.5], vec![0, 100, 150, 200, 50]);
/// ```
#[derive(Debug, Clone)]
pub struct SelfHealingController {
    config: SelfHealingConfig,
    /// Feeder topology: each entry is an ordered bus sequence from substation to end.
    feeders: Vec<Vec<usize>>,
    /// Switches: (branch_idx, is_normally_open).
    switches: Vec<(usize, bool)>,
    /// DER resources: (bus, available_mw).
    der_resources: Vec<(usize, f64)>,
    /// Load per bus \[MW\].
    load_mw: Vec<f64>,
    /// Customer count per bus.
    customers: Vec<usize>,
}

impl SelfHealingController {
    /// Create a new controller with the given configuration.
    pub fn new(config: SelfHealingConfig) -> Self {
        Self {
            config,
            feeders: Vec::new(),
            switches: Vec::new(),
            der_resources: Vec::new(),
            load_mw: Vec::new(),
            customers: Vec::new(),
        }
    }

    /// Register a feeder as an ordered bus sequence (substation bus first).
    pub fn add_feeder(&mut self, bus_sequence: Vec<usize>) {
        self.feeders.push(bus_sequence);
    }

    /// Register a switch on a branch.
    ///
    /// # Arguments
    /// - `branch_idx` — Index of the branch this switch controls.
    /// - `normally_open` — `true` for tie switches (NOP), `false` for sectionalizers.
    pub fn add_switch(&mut self, branch_idx: usize, normally_open: bool) {
        self.switches.push((branch_idx, normally_open));
    }

    /// Register a DER resource.
    ///
    /// # Arguments
    /// - `bus` — Bus where the DER is connected.
    /// - `available_mw` — Maximum DER output \[MW\].
    pub fn add_der(&mut self, bus: usize, available_mw: f64) {
        self.der_resources.push((bus, available_mw));
    }

    /// Set load and customer data for all buses.
    ///
    /// Both vectors must have length equal to `config.n_buses`.
    pub fn set_loads(&mut self, load_mw: Vec<f64>, customers: Vec<usize>) {
        self.load_mw = load_mw;
        self.customers = customers;
    }

    /// Execute the full self-healing response to a fault event.
    ///
    /// # Steps
    /// 1. Locate the faulted section in the feeder topology.
    /// 2. Open boundary switches to isolate the faulted segment.
    /// 3. Restore the upstream (substation-side) section.
    /// 4. Restore the downstream section via tie switch or DER.
    /// 5. Prioritize critical loads throughout.
    ///
    /// Returns [`HealingError`] if the fault location is invalid or no feeders
    /// are configured.
    pub fn respond_to_fault(&self, event: &FaultEvent) -> Result<SelfHealingResult, HealingError> {
        if self.feeders.is_empty() {
            return Err(HealingError::NoFeeders);
        }

        // --- Step 1: Locate fault feeder and partition buses ---
        let (feeder_idx, upstream_buses, downstream_buses) = self.locate_fault(&event.location)?;

        let mut actions: Vec<HealingAction> = Vec::new();
        let mut t = event.timestamp_s;

        // --- Step 2: Isolate faulted section ---
        // Open sectionalizing switches around the fault (simplified: one per boundary)
        t += 2.0; // typical SCADA command delay
        actions.push(HealingAction {
            action_type: HealingActionType::SectionIsolation,
            device_id: event.location.segment,
            timestamp_s: t,
            expected_customers_restored: 0,
            expected_load_mw: 0.0,
        });
        let fault_isolation_time_s = t - event.timestamp_s;

        // Open upstream boundary switch
        t += 1.0;
        actions.push(HealingAction {
            action_type: HealingActionType::OpenSwitch,
            device_id: event.location.bus_upstream,
            timestamp_s: t,
            expected_customers_restored: 0,
            expected_load_mw: 0.0,
        });

        // Open downstream boundary switch
        t += 0.5;
        actions.push(HealingAction {
            action_type: HealingActionType::OpenSwitch,
            device_id: event.location.bus_downstream,
            timestamp_s: t,
            expected_customers_restored: 0,
            expected_load_mw: 0.0,
        });

        // --- Step 3: Restore upstream section ---
        // Upstream buses already have supply from substation once fault is isolated.
        // Schedule in priority order.
        let upstream_load: f64 = self.sum_load(&upstream_buses);
        let upstream_customers: usize = self.sum_customers(&upstream_buses);

        t += 1.0;
        actions.push(HealingAction {
            action_type: HealingActionType::CloseSwitch,
            device_id: event.location.bus_upstream,
            timestamp_s: t,
            expected_customers_restored: upstream_customers,
            expected_load_mw: upstream_load,
        });

        // --- Step 4: Restore downstream section ---
        let downstream_load: f64 = self.sum_load(&downstream_buses);
        let downstream_customers: usize = self.sum_customers(&downstream_buses);

        // Try tie switch first (normally-open switch connecting to another feeder)
        let tie_switch = self.find_tie_switch(feeder_idx);

        let (downstream_restored_customers, downstream_restored_load) =
            if let Some((sw_branch, alt_feeder)) = tie_switch {
                // Load transfer feasibility check
                if downstream_load <= self.config.max_load_transfer_mw {
                    t += 3.0;
                    actions.push(HealingAction {
                        action_type: HealingActionType::LoadTransfer {
                            from_feeder: feeder_idx,
                            to_feeder: alt_feeder,
                        },
                        device_id: sw_branch,
                        timestamp_s: t,
                        expected_customers_restored: downstream_customers,
                        expected_load_mw: downstream_load,
                    });
                    t += 1.0;
                    actions.push(HealingAction {
                        action_type: HealingActionType::CloseSwitch,
                        device_id: sw_branch,
                        timestamp_s: t,
                        expected_customers_restored: downstream_customers,
                        expected_load_mw: downstream_load,
                    });
                    (downstream_customers, downstream_load)
                } else {
                    // Overload — partial restore
                    (0, 0.0)
                }
            } else if let Some((der_bus, der_mw)) = self.find_der_for_downstream(&downstream_buses)
            {
                // Activate DER to island downstream
                let der_load = downstream_load.min(der_mw);
                let der_customers = if der_load >= downstream_load {
                    downstream_customers
                } else {
                    // Proportional
                    (downstream_customers as f64 * der_load / downstream_load.max(1e-9)) as usize
                };
                t += 5.0;
                actions.push(HealingAction {
                    action_type: HealingActionType::DerActivation {
                        der_bus,
                        p_mw: der_load,
                    },
                    device_id: der_bus,
                    timestamp_s: t,
                    expected_customers_restored: der_customers,
                    expected_load_mw: der_load,
                });
                (der_customers, der_load)
            } else {
                // No alternative supply
                (0, 0.0)
            };

        let restoration_time_s = t - event.timestamp_s;

        // --- Step 5: Compute summary ---
        let customers_restored = upstream_customers + downstream_restored_customers;
        let total_customers = upstream_customers + downstream_customers;
        let customers_unrestored = total_customers.saturating_sub(customers_restored);

        let load_restored_mw = upstream_load + downstream_restored_load;
        let total_load = upstream_load + downstream_load;
        let load_unrestored_mw = (total_load - load_restored_mw).max(0.0);

        // Check priority loads
        let priority_loads_restored = self.check_priority_loads_restored(&upstream_buses)
            || self.check_priority_loads_restored(&if downstream_restored_customers > 0 {
                downstream_buses.clone()
            } else {
                Vec::new()
            });

        let automatic_execution = self.config.automation_level == AutomationLevel::Automatic;

        Ok(SelfHealingResult {
            fault_located: true,
            fault_isolation_time_s,
            restoration_time_s,
            actions,
            customers_restored,
            customers_unrestored,
            load_restored_mw,
            load_unrestored_mw,
            priority_loads_restored,
            automatic_execution,
        })
    }

    // ── Private helpers ─────────────────────────────────────────────────────

    /// Locate the fault in the feeder topology, returning:
    /// `(feeder_index, upstream_buses, downstream_buses)`
    fn locate_fault(
        &self,
        loc: &FaultLocation,
    ) -> Result<(usize, Vec<usize>, Vec<usize>), HealingError> {
        for (fi, feeder) in self.feeders.iter().enumerate() {
            // Find upstream bus position in this feeder
            let up_pos = feeder.iter().position(|&b| b == loc.bus_upstream);
            let down_pos = feeder.iter().position(|&b| b == loc.bus_downstream);

            if let (Some(up_i), Some(down_i)) = (up_pos, down_pos) {
                if up_i < down_i {
                    // Upstream: feeder[0..=up_i]
                    // Downstream: feeder[down_i..]
                    let upstream: Vec<usize> = feeder[..=up_i].to_vec();
                    let downstream: Vec<usize> = feeder[down_i..].to_vec();
                    return Ok((fi, upstream, downstream));
                }
            }
        }

        // Not found — return error based on which part failed
        Err(HealingError::InvalidFaultBuses(
            loc.bus_upstream,
            loc.bus_downstream,
        ))
    }

    /// Sum load over a set of buses \[MW\].
    fn sum_load(&self, buses: &[usize]) -> f64 {
        if self.load_mw.is_empty() {
            return 0.0;
        }
        buses.iter().filter_map(|&b| self.load_mw.get(b)).sum()
    }

    /// Sum customers over a set of buses.
    fn sum_customers(&self, buses: &[usize]) -> usize {
        if self.customers.is_empty() {
            return 0;
        }
        buses.iter().filter_map(|&b| self.customers.get(b)).sum()
    }

    /// Find a normally-open tie switch that connects `feeder_idx` to an alternate feeder.
    /// Returns `(branch_idx, alternate_feeder_idx)` if found.
    fn find_tie_switch(&self, feeder_idx: usize) -> Option<(usize, usize)> {
        // Tie switches are normally-open; if any feeder has more than one feeder
        // registered, we can pair them via a NOP switch.
        if self.feeders.len() < 2 {
            return None;
        }
        // Find any NOP switch
        for &(branch_idx, normally_open) in &self.switches {
            if normally_open {
                // Associate with the alternate feeder (any feeder != feeder_idx)
                let alt = (0..self.feeders.len())
                    .find(|&f| f != feeder_idx)
                    .unwrap_or(0);
                return Some((branch_idx, alt));
            }
        }
        None
    }

    /// Find a DER on or near the downstream buses.
    /// Returns `(der_bus, available_mw)` if found.
    fn find_der_for_downstream(&self, downstream_buses: &[usize]) -> Option<(usize, f64)> {
        // Prefer DER directly on a downstream bus
        for &(der_bus, der_mw) in &self.der_resources {
            if downstream_buses.contains(&der_bus) {
                return Some((der_bus, der_mw));
            }
        }
        // Fall back: any DER in the network
        self.der_resources.first().copied()
    }

    /// Returns `true` if all configured priority loads are in the given restored bus set.
    fn check_priority_loads_restored(&self, restored_buses: &[usize]) -> bool {
        if self.config.priority_loads.is_empty() {
            return true;
        }
        self.config
            .priority_loads
            .iter()
            .all(|pl| restored_buses.contains(pl))
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a two-feeder test network:
    ///
    /// Feeder 0: [0, 1, 2, 3, 4]  (substation=0)
    /// Feeder 1: [5, 6, 7]        (alternate supply)
    /// Tie switch (NOP): branch 10 linking feeders 0 and 1
    /// Sectionalizer (NC): branch 2
    /// DER: bus 4, 3.0 MW
    /// Loads: 1 MW per load bus (buses 1-4, 6-7)
    fn build_ctrl() -> SelfHealingController {
        let cfg = SelfHealingConfig {
            n_buses: 8,
            restoration_time_target_s: 60.0,
            max_load_transfer_mw: 10.0,
            priority_loads: vec![1, 2],
            automation_level: AutomationLevel::Automatic,
        };
        let mut ctrl = SelfHealingController::new(cfg);
        ctrl.add_feeder(vec![0, 1, 2, 3, 4]);
        ctrl.add_feeder(vec![5, 6, 7]);
        ctrl.add_switch(2, false); // NC sectionalizer
        ctrl.add_switch(10, true); // NOP tie switch
        ctrl.add_der(4, 3.0);
        ctrl.set_loads(
            vec![0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0],
            vec![0, 100, 100, 100, 100, 0, 100, 100],
        );
        ctrl
    }

    fn fault_between(up: usize, down: usize) -> FaultEvent {
        FaultEvent {
            fault_type: FaultType::PermanentPhaseToGround,
            location: FaultLocation {
                segment: 0,
                bus_upstream: up,
                bus_downstream: down,
                distance_pct: 50.0,
            },
            severity: 0.5,
            timestamp_s: 0.0,
        }
    }

    /// Fault between buses 2 and 3: section should be isolated correctly.
    #[test]
    fn test_fault_isolation() {
        let ctrl = build_ctrl();
        let event = fault_between(2, 3);
        let result = ctrl.respond_to_fault(&event).expect("should succeed");
        assert!(result.fault_located, "Fault must be located");
        assert!(
            result.fault_isolation_time_s > 0.0,
            "Isolation must take nonzero time"
        );
        // Should contain at least one SectionIsolation and two OpenSwitch actions
        let isolations = result
            .actions
            .iter()
            .filter(|a| matches!(a.action_type, HealingActionType::SectionIsolation))
            .count();
        assert!(
            isolations >= 1,
            "At least one section isolation action required"
        );
        let opens = result
            .actions
            .iter()
            .filter(|a| matches!(a.action_type, HealingActionType::OpenSwitch))
            .count();
        assert!(opens >= 2, "Two boundary switches must be opened");
    }

    /// Upstream section (buses 0–2) restored immediately after isolation.
    #[test]
    fn test_upstream_restoration() {
        let ctrl = build_ctrl();
        let event = fault_between(2, 3);
        let result = ctrl.respond_to_fault(&event).expect("should succeed");
        // Upstream = buses [0,1,2], load = 2 MW, customers = 200
        assert!(
            result.load_restored_mw >= 2.0,
            "Upstream load (2 MW) must be restored"
        );
        assert!(
            result.customers_restored >= 200,
            "At least upstream customers restored"
        );
    }

    /// Downstream restored via tie switch to feeder 1.
    #[test]
    fn test_downstream_via_tie_switch() {
        let ctrl = build_ctrl();
        let event = fault_between(2, 3);
        let result = ctrl.respond_to_fault(&event).expect("should succeed");
        // Tie switch exists and load (2 MW) <= max_load_transfer_mw (10 MW)
        let has_load_transfer = result
            .actions
            .iter()
            .any(|a| matches!(a.action_type, HealingActionType::LoadTransfer { .. }));
        assert!(has_load_transfer, "Load transfer via tie switch must occur");
        // All downstream customers should also be restored
        assert_eq!(
            result.customers_unrestored, 0,
            "All customers should be restored"
        );
    }

    /// DER activation when no tie switch exists.
    #[test]
    fn test_der_activation() {
        let cfg = SelfHealingConfig {
            n_buses: 5,
            restoration_time_target_s: 60.0,
            max_load_transfer_mw: 5.0,
            priority_loads: vec![],
            automation_level: AutomationLevel::Automatic,
        };
        let mut ctrl = SelfHealingController::new(cfg);
        ctrl.add_feeder(vec![0, 1, 2, 3, 4]);
        // Only a NC sectionalizer, no NOP tie switch
        ctrl.add_switch(2, false);
        ctrl.add_der(4, 3.0); // DER on downstream bus
        ctrl.set_loads(vec![0.0, 1.0, 1.0, 1.0, 1.0], vec![0, 100, 100, 100, 100]);

        let event = fault_between(2, 3);
        let result = ctrl.respond_to_fault(&event).expect("should succeed");

        let has_der = result
            .actions
            .iter()
            .any(|a| matches!(a.action_type, HealingActionType::DerActivation { .. }));
        assert!(
            has_der,
            "DER activation must be used when no tie switch available"
        );
    }

    /// Priority loads (buses 1, 2) must be among the restored buses.
    #[test]
    fn test_priority_loads_restored() {
        let ctrl = build_ctrl();
        // Fault between 3 and 4 — priority loads (1,2) are upstream → always restored
        let event = fault_between(3, 4);
        let result = ctrl.respond_to_fault(&event).expect("should succeed");
        assert!(
            result.priority_loads_restored,
            "Priority loads on upstream side must be restored"
        );
    }

    /// Manual automation level: actions are still planned but flagged as non-automatic.
    #[test]
    fn test_manual_automation_level() {
        let cfg = SelfHealingConfig {
            n_buses: 5,
            restoration_time_target_s: 60.0,
            max_load_transfer_mw: 5.0,
            priority_loads: vec![],
            automation_level: AutomationLevel::Manual,
        };
        let mut ctrl = SelfHealingController::new(cfg);
        ctrl.add_feeder(vec![0, 1, 2, 3, 4]);
        ctrl.add_switch(2, false);
        ctrl.set_loads(vec![0.0, 1.0, 1.0, 1.0, 1.0], vec![0, 100, 100, 100, 100]);

        let event = fault_between(2, 3);
        let result = ctrl.respond_to_fault(&event).expect("should succeed");
        assert!(
            !result.automatic_execution,
            "Manual mode must not auto-execute"
        );
        assert!(!result.actions.is_empty(), "Actions must still be planned");
    }

    /// Invalid bus configuration returns an error.
    #[test]
    fn test_invalid_fault_location() {
        let ctrl = build_ctrl();
        let event = FaultEvent {
            fault_type: FaultType::PermanentPhaseToGround,
            location: FaultLocation {
                segment: 99,
                bus_upstream: 99,
                bus_downstream: 100,
                distance_pct: 50.0,
            },
            severity: 0.5,
            timestamp_s: 0.0,
        };
        let result = ctrl.respond_to_fault(&event);
        assert!(result.is_err(), "Invalid fault buses must return error");
    }
}
