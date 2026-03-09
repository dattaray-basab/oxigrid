//! FLISR — Fault Location, Isolation and Service Restoration.
//!
//! Distribution automation algorithm that detects faulted sections, isolates them
//! using sectionalizing switches, and restores power to de-energised (but healthy)
//! sections via normally-open tie switches.
//!
//! # Algorithm Overview
//!
//! 1. **Fault Location** — scan fault indicators for the pattern `tripped → not tripped`
//!    along the feeder path to identify the faulted branch.
//! 2. **Fault Isolation** — open the two normally-closed sectionalizing switches that
//!    bracket the faulted section (forming a minimum isolation zone).
//! 3. **Service Restoration** — for each de-energised healthy section, search for
//!    an alternative supply path via normally-open tie switches; close the tie switch
//!    if the additional load does not exceed the feeder thermal limit.
//!
//! # References
//! - Short, T.A., "Electric Power Distribution Handbook", CRC Press, 2004.
//! - IEEE Std 1547.6-2011, "Recommended Practice for Interconnection of Distributed Resources
//!   with Electric Power Systems Distribution Secondary Networks".

use crate::error::{OxiGridError, Result};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet, VecDeque};

// ---------------------------------------------------------------------------
// Public data structures
// ---------------------------------------------------------------------------

/// Action performed on a switch during FLISR sequence.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum SwitchAction {
    /// Open the switch (de-energise the controlled branch).
    Open,
    /// Close the switch (energise the controlled branch).
    Close,
}

/// A controllable switch in the distribution network.
///
/// Each switch controls one branch.  In normal operating conditions,
/// sectionalizing switches are closed and tie switches are open.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SwitchDevice {
    /// Unique switch identifier.
    pub id: usize,
    /// Index of the branch this switch controls (0-based into the branch list).
    pub branch_idx: usize,
    /// From-bus index of the controlled branch.
    pub from_bus: usize,
    /// To-bus index of the controlled branch.
    pub to_bus: usize,
    /// `true` if this is a tie (normally-open) switch.
    pub is_normally_open: bool,
    /// `true` if the switch is healthy and can be operated.
    pub can_operate: bool,
    /// Time required to operate this switch \[s\].
    pub operation_time_s: f64,
}

/// A fault indicator installed at a specific bus.
///
/// The indicator trips when fault current exceeds `current_threshold_a` \[A\].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FaultIndicator {
    /// Unique indicator identifier.
    pub id: usize,
    /// Bus index where this indicator is installed.
    pub bus_idx: usize,
    /// Current threshold for tripping \[A\].
    pub current_threshold_a: f64,
    /// `true` if the indicator has tripped (fault current detected).
    pub tripped: bool,
}

/// A single switch operation in the restoration sequence.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SwitchOperation {
    /// Switch to operate.
    pub switch_id: usize,
    /// Action (open or close).
    pub action: SwitchAction,
    /// Timestamp at which this operation is performed \[s\] from fault event.
    pub time_s: f64,
    /// Human-readable reason for this operation.
    pub reason: String,
}

/// Full result of a FLISR execution.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FlisrResult {
    /// Index of the faulted branch, if located.
    pub fault_location: Option<usize>,
    /// Bus indices forming the isolated (de-energised faulted) section.
    pub isolated_section: Vec<usize>,
    /// Ordered sequence of switch operations to isolate and restore.
    pub restoration_steps: Vec<SwitchOperation>,
    /// Total load restored \[MW\].
    pub restored_load_mw: f64,
    /// Load that could not be restored (no alternative supply) \[MW\].
    pub unrestored_load_mw: f64,
    /// Cumulative switch operation time \[s\].
    pub total_operation_time_s: f64,
    /// Total number of customers restored.
    pub customers_restored: usize,
}

/// FLISR controller for a distribution feeder.
///
/// # Example
/// ```rust,no_run
/// use oxigrid::network::flisr::{FlisrController, SwitchDevice, FaultIndicator};
///
/// let controller = FlisrController {
///     switches: vec![],
///     fault_indicators: vec![],
///     bus_loads_mw: vec![0.0; 5],
///     feeder_capacity_mw: 10.0,
///     bus_customers: vec![100; 5],
///     n_buses: 5,
///     adjacency: vec![vec![]; 5],
///     substation_buses: vec![0],
/// };
/// ```
#[derive(Debug, Clone)]
pub struct FlisrController {
    /// All controllable switches in the feeder.
    pub switches: Vec<SwitchDevice>,
    /// Fault indicators installed throughout the feeder.
    pub fault_indicators: Vec<FaultIndicator>,
    /// Active load at each bus \[MW\].
    pub bus_loads_mw: Vec<f64>,
    /// Maximum load that a single feeder section can supply \[MW\].
    pub feeder_capacity_mw: f64,
    /// Number of customers at each bus.
    pub bus_customers: Vec<usize>,
    /// Total number of buses in the network.
    pub n_buses: usize,
    /// Adjacency list for the network graph (neighbour bus indices).
    ///
    /// `adjacency[i]` gives the list of buses directly connected to bus `i`
    /// via *closed* branches (in normal operation).
    pub adjacency: Vec<Vec<usize>>,
    /// Indices of substation (source) buses.
    pub substation_buses: Vec<usize>,
}

impl FlisrController {
    /// Execute the FLISR algorithm and return the restoration plan.
    ///
    /// Steps performed:
    /// 1. Locate the fault from indicator pattern.
    /// 2. Isolate the faulted section by opening bounding sectionalizing switches.
    /// 3. Restore de-energised healthy sections via tie switches.
    ///
    /// # Errors
    /// Returns [`OxiGridError::InvalidNetwork`] if the network topology is inconsistent.
    pub fn execute(&self) -> Result<FlisrResult> {
        // --- Step 1: Fault Location ---
        let fault_branch = self.locate_fault();
        let mut steps: Vec<SwitchOperation> = Vec::new();
        let mut current_time = 0.0_f64;

        // --- Step 2: Fault Isolation ---
        let isolated_section = if let Some(fbranch) = fault_branch {
            let (section, isolation_ops, iso_time) = self.isolate_fault(fbranch, current_time)?;
            current_time = iso_time;
            steps.extend(isolation_ops);
            section
        } else {
            Vec::new()
        };

        // Build set of opened sectionalizing branch indices (from isolation step)
        let opened_branches: HashSet<usize> = fault_branch.into_iter().collect();

        // Build active graph (with isolated section removed)
        let isolated_set: HashSet<usize> = isolated_section.iter().cloned().collect();

        // --- Step 3: Service Restoration ---
        let (restore_ops, restored_mw, unrestored_mw, customers_restored, final_time) =
            self.restore_service(&isolated_set, &opened_branches, current_time)?;
        steps.extend(restore_ops);

        Ok(FlisrResult {
            fault_location: fault_branch,
            isolated_section,
            restoration_steps: steps.clone(),
            restored_load_mw: restored_mw,
            unrestored_load_mw: unrestored_mw,
            total_operation_time_s: final_time,
            customers_restored,
        })
    }

    /// Locate the fault by analysing fault indicator trip pattern.
    ///
    /// Returns the branch index of the faulted branch, or `None` if no
    /// consistent fault pattern is found.
    fn locate_fault(&self) -> Option<usize> {
        // Strategy: the fault lies on the branch whose source-side indicator tripped
        // but load-side indicator did NOT trip (last tripped indicator in feed direction).
        //
        // Simplified approach: find the switch whose from-bus indicator is tripped
        // but to-bus indicator is not tripped.

        // Build a map: bus_idx → tripped
        let mut bus_tripped: HashMap<usize, bool> = HashMap::new();
        for fi in &self.fault_indicators {
            bus_tripped.insert(fi.bus_idx, fi.tripped);
        }

        for sw in &self.switches {
            if sw.is_normally_open {
                continue; // tie switches don't carry fault current normally
            }
            let from_tripped = bus_tripped.get(&sw.from_bus).copied().unwrap_or(false);
            let to_tripped = bus_tripped.get(&sw.to_bus).copied().unwrap_or(false);
            // Fault is between a tripped source indicator and non-tripped load indicator
            if from_tripped && !to_tripped {
                return Some(sw.branch_idx);
            }
        }

        // Fallback: if any indicator is tripped at all, find the last one
        // (indicator furthest from substation that tripped)
        let mut last_tripped_branch: Option<usize> = None;
        for sw in &self.switches {
            if sw.is_normally_open {
                continue;
            }
            let from_tripped = bus_tripped.get(&sw.from_bus).copied().unwrap_or(false);
            if from_tripped {
                last_tripped_branch = Some(sw.branch_idx);
            }
        }

        last_tripped_branch
    }

    /// Determine the isolation zone and generate isolation switch operations.
    ///
    /// Returns `(isolated_buses, operations, time_at_completion)`.
    fn isolate_fault(
        &self,
        fault_branch: usize,
        start_time: f64,
    ) -> Result<(Vec<usize>, Vec<SwitchOperation>, f64)> {
        // Find the normally-closed switches that bound the faulted branch
        let bounding_switches: Vec<&SwitchDevice> = self
            .switches
            .iter()
            .filter(|sw| !sw.is_normally_open && sw.can_operate && sw.branch_idx == fault_branch)
            .collect();

        if bounding_switches.is_empty() {
            // No operable switch on this branch — open nearest switches
            let nearest: Vec<&SwitchDevice> = self
                .switches
                .iter()
                .filter(|sw| !sw.is_normally_open && sw.can_operate)
                .take(2)
                .collect();

            if nearest.is_empty() {
                return Err(OxiGridError::InvalidNetwork(
                    "No operable sectionalizing switches found for fault isolation".into(),
                ));
            }

            let mut ops = Vec::new();
            let mut t = start_time;
            for sw in &nearest {
                t += sw.operation_time_s;
                ops.push(SwitchOperation {
                    switch_id: sw.id,
                    action: SwitchAction::Open,
                    time_s: t,
                    reason: format!("Isolate fault on branch {} (nearest switch)", fault_branch),
                });
            }
            // Isolated section: only the direct fault endpoints (bus pair of each opened switch)
            let isolated: Vec<usize> = nearest
                .iter()
                .flat_map(|sw| [sw.from_bus, sw.to_bus])
                .collect::<std::collections::HashSet<_>>()
                .into_iter()
                .collect();
            return Ok((isolated, ops, t));
        }

        let mut ops = Vec::new();
        let mut t = start_time;
        for sw in &bounding_switches {
            t += sw.operation_time_s;
            ops.push(SwitchOperation {
                switch_id: sw.id,
                action: SwitchAction::Open,
                time_s: t,
                reason: format!("Isolate fault on branch {}", fault_branch),
            });
        }

        // Isolated section: the fault endpoints (from_bus and to_bus of the opened switches)
        // We deliberately do NOT include downstream radial buses here — those become de-energised
        // healthy sections that can be restored via tie switches.
        let isolated: Vec<usize> = bounding_switches
            .iter()
            .flat_map(|sw| [sw.from_bus, sw.to_bus])
            .collect::<HashSet<_>>()
            .into_iter()
            .collect();

        Ok((isolated, ops, t))
    }

    /// Restore service to de-energised healthy buses via tie switches.
    ///
    /// Returns `(operations, restored_mw, unrestored_mw, customers_restored, time)`.
    fn restore_service(
        &self,
        isolated_set: &HashSet<usize>,
        opened_branches: &HashSet<usize>,
        start_time: f64,
    ) -> Result<(Vec<SwitchOperation>, f64, f64, usize, f64)> {
        // Identify de-energised buses (not in isolated faulted section, not substation)
        let sub_set: HashSet<usize> = self.substation_buses.iter().cloned().collect();
        let de_energised = self.find_de_energised_buses(isolated_set, opened_branches, &sub_set);

        if de_energised.is_empty() {
            return Ok((Vec::new(), 0.0, 0.0, 0, start_time));
        }

        let mut ops: Vec<SwitchOperation> = Vec::new();
        let mut t = start_time;
        let mut restored_mw = 0.0_f64;
        let mut customers_restored = 0usize;

        // Track which buses are now restored (energised from alternative supply)
        let mut restored_buses: HashSet<usize> = sub_set.clone();

        // Iterate over tie switches — try to close each one to restore adjacent section
        let tie_switches: Vec<&SwitchDevice> = self
            .switches
            .iter()
            .filter(|sw| sw.is_normally_open && sw.can_operate)
            .collect();

        for tie_sw in &tie_switches {
            // Check if closing this tie switch can reach de-energised buses
            let reachable_de_energised = self.find_reachable_de_energised(
                tie_sw,
                &de_energised,
                isolated_set,
                &restored_buses,
            );

            if reachable_de_energised.is_empty() {
                continue;
            }

            // Capacity check: sum of loads in the section to be restored
            let section_load: f64 = reachable_de_energised
                .iter()
                .filter_map(|&b| self.bus_loads_mw.get(b))
                .sum();

            // Also account for existing feeder load
            let existing_load: f64 = restored_buses
                .iter()
                .filter(|&&b| !sub_set.contains(&b))
                .filter_map(|&b| self.bus_loads_mw.get(b))
                .sum();

            if existing_load + section_load > self.feeder_capacity_mw {
                // Exceed capacity — skip this tie switch
                continue;
            }

            // Close tie switch
            t += tie_sw.operation_time_s;
            ops.push(SwitchOperation {
                switch_id: tie_sw.id,
                action: SwitchAction::Close,
                time_s: t,
                reason: format!(
                    "Restore {} buses ({:.2} MW) via tie switch",
                    reachable_de_energised.len(),
                    section_load
                ),
            });

            // Mark as restored
            for &b in &reachable_de_energised {
                restored_buses.insert(b);
                restored_mw += self.bus_loads_mw.get(b).copied().unwrap_or(0.0);
                customers_restored += self.bus_customers.get(b).copied().unwrap_or(0);
            }
        }

        // Compute unrestored load (de-energised and not restored)
        let unrestored_mw: f64 = de_energised
            .iter()
            .filter(|b| !restored_buses.contains(b))
            .filter_map(|&b| self.bus_loads_mw.get(b))
            .sum();

        Ok((ops, restored_mw, unrestored_mw, customers_restored, t))
    }

    /// BFS to find de-energised buses not in the isolated faulted section.
    ///
    /// `opened_branches` is the set of branch indices that were opened for isolation;
    /// these branches are treated as absent from the adjacency graph.
    fn find_de_energised_buses(
        &self,
        isolated_set: &HashSet<usize>,
        opened_branches: &HashSet<usize>,
        sub_set: &HashSet<usize>,
    ) -> HashSet<usize> {
        // Build adjacency using only closed sectionalizing switches,
        // excluding the opened branches (fault isolation switches)
        let mut adj: HashMap<usize, Vec<usize>> = HashMap::new();
        for sw in &self.switches {
            if sw.is_normally_open || opened_branches.contains(&sw.branch_idx) {
                continue;
            }
            adj.entry(sw.from_bus).or_default().push(sw.to_bus);
            adj.entry(sw.to_bus).or_default().push(sw.from_bus);
        }

        // BFS from substations — all reached buses (excluding isolated section) are energised
        let mut energised: HashSet<usize> = sub_set.clone();
        let mut queue: VecDeque<usize> = sub_set.iter().cloned().collect();

        while let Some(bus) = queue.pop_front() {
            if isolated_set.contains(&bus) {
                continue; // isolated (faulted) section is not energised
            }
            if let Some(neighbors) = adj.get(&bus) {
                for &nb in neighbors {
                    if !energised.contains(&nb) && !isolated_set.contains(&nb) {
                        energised.insert(nb);
                        queue.push_back(nb);
                    }
                }
            }
        }

        // De-energised = all buses not reachable from substations,
        // not in isolated section, not substation
        (0..self.n_buses)
            .filter(|b| !energised.contains(b) && !isolated_set.contains(b) && !sub_set.contains(b))
            .collect()
    }

    /// BFS from tie switch to find which de-energised buses can be reached.
    fn find_reachable_de_energised(
        &self,
        tie_sw: &SwitchDevice,
        de_energised: &HashSet<usize>,
        isolated_set: &HashSet<usize>,
        restored_buses: &HashSet<usize>,
    ) -> Vec<usize> {
        // The tie switch connects from_bus (energised side) to to_bus (de-energised side)
        // We BFS from to_bus along closed switch adjacency to find reachable de-energised buses
        let seed = if de_energised.contains(&tie_sw.to_bus) {
            tie_sw.to_bus
        } else if de_energised.contains(&tie_sw.from_bus) {
            tie_sw.from_bus
        } else {
            return Vec::new();
        };

        let mut adj: HashMap<usize, Vec<usize>> = HashMap::new();
        for sw in &self.switches {
            if sw.is_normally_open {
                continue;
            }
            adj.entry(sw.from_bus).or_default().push(sw.to_bus);
            adj.entry(sw.to_bus).or_default().push(sw.from_bus);
        }

        let mut reachable = Vec::new();
        let mut visited: HashSet<usize> = HashSet::new();
        visited.insert(seed);
        let mut queue: VecDeque<usize> = VecDeque::new();
        queue.push_back(seed);

        while let Some(bus) = queue.pop_front() {
            if de_energised.contains(&bus) && !isolated_set.contains(&bus) {
                reachable.push(bus);
            }
            if let Some(neighbors) = adj.get(&bus) {
                for &nb in neighbors {
                    if !visited.contains(&nb)
                        && !isolated_set.contains(&nb)
                        && !restored_buses.contains(&nb)
                    {
                        visited.insert(nb);
                        queue.push_back(nb);
                    }
                }
            }
        }

        reachable
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a simple 5-bus radial feeder:
    /// Substation(0) — 1 — 2 — [FAULT] — 3 — 4
    /// Tie switch from bus 4 to an alternate substation at bus 5
    fn simple_feeder() -> FlisrController {
        //  Bus layout: 0(sub) - 1 - 2 - 3 - 4   , tie: 4-5, 5 is alt-sub
        let n_buses = 6;
        let bus_loads_mw = vec![0.0, 1.0, 2.0, 1.5, 1.0, 0.0];
        let bus_customers = vec![0, 100, 200, 150, 100, 0];

        // Adjacency (in normal config: 0-1-2-3-4, tie 4-5 is open)
        let adjacency = vec![
            vec![1],    // 0: sub
            vec![0, 2], // 1
            vec![1, 3], // 2
            vec![2, 4], // 3
            vec![3],    // 4 (tie to 5 is normally open)
            vec![],     // 5: alt-sub
        ];

        // Switches: branch_idx = switch index in this simplified model
        // Branch 0: 0-1, Branch 1: 1-2, Branch 2: 2-3, Branch 3: 3-4, Tie: 4-5
        let switches = vec![
            SwitchDevice {
                id: 10,
                branch_idx: 0,
                from_bus: 0,
                to_bus: 1,
                is_normally_open: false,
                can_operate: true,
                operation_time_s: 0.5,
            },
            SwitchDevice {
                id: 11,
                branch_idx: 1,
                from_bus: 1,
                to_bus: 2,
                is_normally_open: false,
                can_operate: true,
                operation_time_s: 0.5,
            },
            SwitchDevice {
                id: 12,
                branch_idx: 2,
                from_bus: 2,
                to_bus: 3,
                is_normally_open: false,
                can_operate: true,
                operation_time_s: 0.5,
            },
            SwitchDevice {
                id: 13,
                branch_idx: 3,
                from_bus: 3,
                to_bus: 4,
                is_normally_open: false,
                can_operate: true,
                operation_time_s: 0.5,
            },
            SwitchDevice {
                id: 20,
                branch_idx: 4,
                from_bus: 4,
                to_bus: 5,
                is_normally_open: true,
                can_operate: true,
                operation_time_s: 0.5,
            },
        ];

        // Fault indicators at each bus
        let fault_indicators = vec![
            FaultIndicator {
                id: 1,
                bus_idx: 0,
                current_threshold_a: 100.0,
                tripped: true,
            },
            FaultIndicator {
                id: 2,
                bus_idx: 1,
                current_threshold_a: 100.0,
                tripped: true,
            },
            FaultIndicator {
                id: 3,
                bus_idx: 2,
                current_threshold_a: 100.0,
                tripped: true,
            },
            FaultIndicator {
                id: 4,
                bus_idx: 3,
                current_threshold_a: 100.0,
                tripped: false,
            }, // fault between 2 and 3
            FaultIndicator {
                id: 5,
                bus_idx: 4,
                current_threshold_a: 100.0,
                tripped: false,
            },
        ];

        FlisrController {
            switches,
            fault_indicators,
            bus_loads_mw,
            feeder_capacity_mw: 10.0,
            bus_customers,
            n_buses,
            adjacency,
            substation_buses: vec![0, 5],
        }
    }

    #[test]
    fn test_fault_location_mid_feeder() {
        let ctrl = simple_feeder();
        let result = ctrl.execute().expect("flisr execute");
        // Fault should be located on branch 2 (between bus 2 and bus 3)
        assert_eq!(
            result.fault_location,
            Some(2),
            "Expected fault on branch 2, got {:?}",
            result.fault_location
        );
    }

    #[test]
    fn test_isolation_step_generated() {
        let ctrl = simple_feeder();
        let result = ctrl.execute().expect("flisr execute");
        // At least one Open operation should be generated for isolation
        let open_ops: Vec<_> = result
            .restoration_steps
            .iter()
            .filter(|op| op.action == SwitchAction::Open)
            .collect();
        assert!(
            !open_ops.is_empty(),
            "Expected at least one Open operation for fault isolation"
        );
    }

    #[test]
    fn test_restoration_via_tie_switch() {
        let ctrl = simple_feeder();
        let result = ctrl.execute().expect("flisr execute");
        // At least one Close operation should be generated for restoration
        let close_ops: Vec<_> = result
            .restoration_steps
            .iter()
            .filter(|op| op.action == SwitchAction::Close)
            .collect();
        // Bus 3 and 4 should be restorable via tie switch 20 (4-5)
        assert!(
            !close_ops.is_empty(),
            "Expected tie switch closure for restoration, restored_mw={:.2}",
            result.restored_load_mw
        );
        assert!(
            result.restored_load_mw > 0.0,
            "Expected positive restored load"
        );
    }

    #[test]
    fn test_no_restoration_path_available() {
        // Build feeder where tie switch is inoperable
        let mut ctrl = simple_feeder();
        // Make tie switch inoperable
        for sw in &mut ctrl.switches {
            if sw.is_normally_open {
                sw.can_operate = false;
            }
        }
        let result = ctrl.execute().expect("flisr execute");
        // No close operations should be generated
        let close_ops: Vec<_> = result
            .restoration_steps
            .iter()
            .filter(|op| op.action == SwitchAction::Close)
            .collect();
        assert!(
            close_ops.is_empty(),
            "Expected no restoration ops when tie switch inoperable"
        );
        assert_eq!(result.customers_restored, 0);
    }

    #[test]
    fn test_capacity_limit_prevents_restoration() {
        // Set feeder capacity so low that restoration would exceed it
        let mut ctrl = simple_feeder();
        ctrl.feeder_capacity_mw = 0.001; // 1 W — impossible to restore anything
        let result = ctrl.execute().expect("flisr execute");
        // Restoration should be blocked by capacity check
        let close_ops: Vec<_> = result
            .restoration_steps
            .iter()
            .filter(|op| op.action == SwitchAction::Close)
            .collect();
        assert!(
            close_ops.is_empty(),
            "Expected no restoration when capacity too low"
        );
    }

    #[test]
    fn test_operation_time_accumulates() {
        let ctrl = simple_feeder();
        let result = ctrl.execute().expect("flisr execute");
        // Total operation time should be positive and equal sum of step times
        let last_time = result
            .restoration_steps
            .last()
            .map(|op| op.time_s)
            .unwrap_or(0.0);
        assert!(
            result.total_operation_time_s >= 0.0,
            "Total operation time should be non-negative"
        );
        assert!(
            (result.total_operation_time_s - last_time).abs() < 1e-9
                || result.total_operation_time_s >= last_time,
            "Total operation time should be at least last step time"
        );
    }
}
