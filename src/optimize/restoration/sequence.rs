//! Restoration sequencing: parallel path discovery, load-block ordering,
//! and SAIDI/SAIFI/ENS reliability metrics.

use crate::network::topology::PowerNetwork;
use crate::optimize::restoration::black_start::{EnergizationPath, LoadBlock, RestorationPlan};
use std::collections::{HashSet, VecDeque};

// ─────────────────────────────────────────────────────────────────────────────
// RestorationSequencer
// ─────────────────────────────────────────────────────────────────────────────

/// Computes parallel energisation opportunities and orders load blocks for
/// optimal restoration priority.
pub struct RestorationSequencer {
    /// Maximum number of simultaneously active energisation paths.
    pub max_parallel_paths: usize,
}

impl RestorationSequencer {
    /// Construct a sequencer allowing up to `max_parallel_paths` concurrent paths.
    pub fn new(max_parallel_paths: usize) -> Self {
        Self {
            max_parallel_paths: max_parallel_paths.max(1),
        }
    }

    // ── Public API ──────────────────────────────────────────────────────────

    /// Discover groups of energisation paths that can be executed in parallel.
    ///
    /// Two paths are *electrically independent* when they share no common
    /// intermediate bus — i.e., they form disjoint sub-trees rooted at
    /// distinct black-start buses.
    ///
    /// Returns a `Vec` of groups; each group is a `Vec<EnergizationPath>` that
    /// can be energised simultaneously.
    pub fn compute_parallel_paths(
        &self,
        network: &PowerNetwork,
        black_start_buses: &[usize],
    ) -> Vec<Vec<EnergizationPath>> {
        // One parallel group per black-start bus (radial sub-tree BFS)
        let mut groups: Vec<Vec<EnergizationPath>> = Vec::new();

        for &bs_bus in black_start_buses {
            let subtree_paths = self.bfs_subtree(network, bs_bus);
            if !subtree_paths.is_empty() {
                groups.push(subtree_paths);
            }
        }

        // Limit group count to max_parallel_paths
        groups.truncate(self.max_parallel_paths);
        groups
    }

    /// Order load blocks for sequential pickup given the available net headroom.
    ///
    /// Strategy:
    /// 1. Non-deferrable blocks (priority 1) first.
    /// 2. Among blocks at the same priority level, choose those whose
    ///    cold-load pickup demand fits within the headroom first.
    /// 3. Ties broken by block_id for determinism.
    ///
    /// Returns a `Vec` of block IDs in the recommended restoration order.
    pub fn order_load_blocks(
        &self,
        blocks: &[LoadBlock],
        available_mw: f64,
        reserve_mw: f64,
    ) -> Vec<usize> {
        let headroom = (available_mw - reserve_mw).max(0.0);
        let mut remaining_headroom = headroom;
        let mut ordered: Vec<usize> = Vec::new();

        // Group by priority, then within each group pick greedily by demand fit
        let mut priorities: Vec<usize> = blocks.iter().map(|b| b.priority).collect();
        priorities.sort_unstable();
        priorities.dedup();

        for prio in priorities {
            let mut tier: Vec<&LoadBlock> = blocks.iter().filter(|b| b.priority == prio).collect();
            // Sort within tier: non-deferrable first, then by ascending demand
            tier.sort_by(|a, b| {
                let a_nd = !a.can_defer as u8;
                let b_nd = !b.can_defer as u8;
                b_nd.cmp(&a_nd)
                    .then_with(|| {
                        a.base_demand_mw
                            .partial_cmp(&b.base_demand_mw)
                            .unwrap_or(std::cmp::Ordering::Equal)
                    })
                    .then(a.block_id.cmp(&b.block_id))
            });

            for block in tier {
                // Cold-load pickup demand at t=0 after restore
                let clp = block.base_demand_mw * block.cold_load_pickup_factor;
                if clp <= remaining_headroom || !block.can_defer {
                    ordered.push(block.block_id);
                    remaining_headroom = (remaining_headroom - clp).max(0.0);
                }
            }
        }

        ordered
    }

    // ── Private helpers ─────────────────────────────────────────────────────

    /// BFS from `root_bus` to enumerate all reachable energisation paths in the
    /// network sub-tree (one path per reachable bus).
    fn bfs_subtree(&self, network: &PowerNetwork, root_bus: usize) -> Vec<EnergizationPath> {
        let mut paths: Vec<EnergizationPath> = Vec::new();
        let mut visited: HashSet<usize> = HashSet::new();
        // (current_bus, path_so_far, accumulated_km)
        let mut queue: VecDeque<(usize, Vec<usize>, f64)> = VecDeque::new();

        visited.insert(root_bus);
        queue.push_back((root_bus, Vec::new(), 0.0));

        while let Some((current, branch_path, dist)) = queue.pop_front() {
            for (bi, branch) in network.branches.iter().enumerate() {
                if !branch.status {
                    continue;
                }
                let neighbor = if branch.from_bus == current {
                    branch.to_bus
                } else if branch.to_bus == current {
                    branch.from_bus
                } else {
                    continue;
                };

                if visited.contains(&neighbor) {
                    continue;
                }
                visited.insert(neighbor);

                let z_pu = (branch.r * branch.r + branch.x * branch.x).sqrt();
                let length_km = (z_pu / 0.3).max(0.5);
                let new_dist = dist + length_km;
                let mut new_path = branch_path.clone();
                new_path.push(bi);

                let charging: f64 = new_path
                    .iter()
                    .map(|&idx| network.branches[idx].b * 0.5 * 100.0)
                    .sum();

                paths.push(EnergizationPath {
                    from_bus: root_bus,
                    to_bus: neighbor,
                    branch_sequence: new_path.clone(),
                    total_length_km: new_dist,
                    charging_current_mvar: charging,
                    can_energize_at_t: 0.0,
                });

                queue.push_back((neighbor, new_path, new_dist));
            }
        }
        paths
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// RestorationMetrics: SAIDI / SAIFI / ENS
// ─────────────────────────────────────────────────────────────────────────────

/// Tracks reliability impact of a restoration event.
pub struct RestorationMetrics {
    /// Total number of customers affected by the blackout.
    pub total_customers: usize,
    /// Clock time at which the blackout began \[min\].
    pub outage_start_min: f64,
}

impl RestorationMetrics {
    /// Construct metrics for an event that started at `outage_start_min` and
    /// affects `total_customers` customers.
    pub fn new(total_customers: usize, outage_start_min: f64) -> Self {
        Self {
            total_customers,
            outage_start_min,
        }
    }

    /// System Average Interruption Duration Index \[customer-minutes / customer\].
    ///
    /// ```text
    /// SAIDI = Σ_i (customers_i × interruption_duration_i) / total_customers
    /// ```
    ///
    /// `customers_per_block[i]` is the customer count for block `i` (0-based,
    /// matching `RestorationPlan::steps` load-block ordering).
    pub fn compute_saidi(&self, plan: &RestorationPlan, customers_per_block: &[usize]) -> f64 {
        use crate::optimize::restoration::black_start::RestorationAction;

        if self.total_customers == 0 {
            return 0.0;
        }

        let mut sum = 0.0_f64;
        for step in &plan.steps {
            if let RestorationAction::PickupLoadBlock { block_id, .. } = &step.action {
                let customers = customers_per_block.get(*block_id).copied().unwrap_or(0);
                let duration_min = step.time_min - self.outage_start_min;
                sum += (customers as f64) * duration_min.max(0.0);
            }
        }
        sum / (self.total_customers as f64)
    }

    /// System Average Interruption Frequency Index \[interruptions / customer\].
    ///
    /// For a single black-start event every customer experiences exactly one
    /// interruption, so SAIFI = total interrupted customers / total customers.
    ///
    /// `customers_per_block[i]` is the customer count for block `i`.
    pub fn compute_saifi(&self, plan: &RestorationPlan, customers_per_block: &[usize]) -> f64 {
        use crate::optimize::restoration::black_start::RestorationAction;

        if self.total_customers == 0 {
            return 0.0;
        }

        let mut interrupted = 0usize;
        for step in &plan.steps {
            if let RestorationAction::PickupLoadBlock { block_id, .. } = &step.action {
                interrupted += customers_per_block.get(*block_id).copied().unwrap_or(0);
            }
        }
        interrupted as f64 / self.total_customers as f64
    }

    /// Energy Not Supplied \[MWh\].
    ///
    /// ```text
    /// ENS = Σ_i (P_base_i × outage_duration_i)
    /// ```
    ///
    /// where `outage_duration_i` is measured from `outage_start_min` to the
    /// step time at which block `i` is restored.
    pub fn compute_ens(&self, plan: &RestorationPlan, blocks: &[LoadBlock]) -> f64 {
        use crate::optimize::restoration::black_start::RestorationAction;

        let mut ens = 0.0_f64;
        for step in &plan.steps {
            if let RestorationAction::PickupLoadBlock { block_id, .. } = &step.action {
                if let Some(block) = blocks.iter().find(|b| b.block_id == *block_id) {
                    let duration_h = (step.time_min - self.outage_start_min).max(0.0) / 60.0;
                    ens += block.base_demand_mw * duration_h;
                }
            }
        }
        // Blocks not restored: add full plan duration
        for block in blocks {
            let restored = plan.steps.iter().any(|s| {
                matches!(&s.action, RestorationAction::PickupLoadBlock { block_id, .. }
                    if *block_id == block.block_id)
            });
            if !restored {
                let duration_h = (plan.total_time_min - self.outage_start_min).max(0.0) / 60.0;
                ens += block.base_demand_mw * duration_h;
            }
        }
        ens
    }
}

#[cfg(test)]
mod tests {
    use super::{RestorationMetrics, RestorationSequencer};
    use crate::network::bus::{Bus, BusType};
    use crate::network::topology::PowerNetwork;
    use crate::optimize::restoration::black_start::{
        LoadBlock, RestorationAction, RestorationPlan, RestorationStep,
    };

    fn make_load_block(
        block_id: usize,
        demand_mw: f64,
        priority: usize,
        can_defer: bool,
    ) -> LoadBlock {
        LoadBlock {
            block_id,
            buses: vec![block_id + 10],
            base_demand_mw: demand_mw,
            cold_load_pickup_factor: 1.5,
            cold_load_decay_min: 30.0,
            priority,
            can_defer,
        }
    }

    #[test]
    fn test_order_load_blocks_returns_nonempty() {
        let sequencer = RestorationSequencer::new(3);
        let blocks = vec![
            make_load_block(0, 20.0, 1, false),
            make_load_block(1, 10.0, 2, true),
            make_load_block(2, 15.0, 2, true),
        ];
        let result = sequencer.order_load_blocks(&blocks, 100.0, 10.0);
        assert!(!result.is_empty(), "result should be non-empty");
        assert!(
            result.contains(&0),
            "non-deferrable block 0 must be included"
        );
    }

    #[test]
    fn test_order_load_blocks_priority_ordering() {
        let sequencer = RestorationSequencer::new(3);
        let blocks = vec![
            make_load_block(0, 20.0, 1, false),
            make_load_block(1, 10.0, 2, true),
            make_load_block(2, 15.0, 2, true),
        ];
        let result = sequencer.order_load_blocks(&blocks, 100.0, 0.0);
        let pos0 = result
            .iter()
            .position(|&id| id == 0)
            .expect("block 0 must be in result");
        let pos1 = result.iter().position(|&id| id == 1);
        let pos2 = result.iter().position(|&id| id == 2);
        if let Some(p1) = pos1 {
            assert!(pos0 < p1, "priority-1 block 0 must come before block 1");
        }
        if let Some(p2) = pos2 {
            assert!(pos0 < p2, "priority-1 block 0 must come before block 2");
        }
    }

    #[test]
    fn test_order_load_blocks_respects_headroom() {
        let sequencer = RestorationSequencer::new(3);
        // block0: 80MW * 1.5 CLP = 120MW required — exceeds headroom of 20MW
        // block1: 5MW  * 1.5 CLP =  7.5MW required — fits within 20MW
        // both are deferrable, so block0 should be excluded
        let blocks = vec![
            make_load_block(0, 80.0, 1, true),
            make_load_block(1, 5.0, 1, true),
        ];
        let result = sequencer.order_load_blocks(&blocks, 20.0, 0.0);
        assert!(
            result.contains(&1),
            "block 1 (7.5 MW CLP) must fit within 20 MW headroom"
        );
        assert!(
            !result.contains(&0),
            "block 0 (120 MW CLP) must not fit within 20 MW headroom"
        );
    }

    #[test]
    fn test_compute_parallel_paths_empty_network() {
        let sequencer = RestorationSequencer::new(4);
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        let groups = sequencer.compute_parallel_paths(&net, &[1]);
        assert!(groups.is_empty(), "no branches means no energization paths");
    }

    #[test]
    fn test_metrics_saidi_saifi_ens() {
        let metrics = RestorationMetrics::new(100, 0.0);
        let blocks = vec![
            make_load_block(0, 20.0, 1, false),
            make_load_block(1, 10.0, 2, true),
        ];
        let plan = RestorationPlan {
            steps: vec![
                RestorationStep {
                    step_id: 1,
                    time_min: 30.0,
                    action: RestorationAction::PickupLoadBlock {
                        block_id: 0,
                        actual_mw: 20.0,
                    },
                    available_generation_mw: 50.0,
                    connected_load_mw: 20.0,
                    frequency_hz: 50.0,
                    notes: String::new(),
                },
                RestorationStep {
                    step_id: 2,
                    time_min: 60.0,
                    action: RestorationAction::PickupLoadBlock {
                        block_id: 1,
                        actual_mw: 10.0,
                    },
                    available_generation_mw: 50.0,
                    connected_load_mw: 30.0,
                    frequency_hz: 50.0,
                    notes: String::new(),
                },
            ],
            total_time_min: 120.0,
            restored_load_pct: 1.0,
            n_black_start_units_used: 1,
            critical_loads_restored_min: 30.0,
            feasible: true,
            bottlenecks: vec![],
        };
        let customers_per_block = [50usize, 50usize];
        let saidi = metrics.compute_saidi(&plan, &customers_per_block);
        let saifi = metrics.compute_saifi(&plan, &customers_per_block);
        let ens = metrics.compute_ens(&plan, &blocks);
        assert!(saidi > 0.0, "SAIDI must be positive");
        assert!(
            (saifi - 1.0).abs() < 1e-9,
            "SAIFI must equal 1.0 (all 100 customers interrupted, 100/100)"
        );
        assert!(ens > 0.0, "ENS must be positive");
    }

    #[test]
    fn test_sequencer_new_clamps_to_one() {
        let sequencer = RestorationSequencer::new(0);
        assert_eq!(
            sequencer.max_parallel_paths, 1,
            "new(0) must clamp max_parallel_paths to 1"
        );
    }

    #[test]
    fn test_order_load_blocks_empty_input() {
        let sequencer = RestorationSequencer::new(3);
        let result = sequencer.order_load_blocks(&[], 100.0, 10.0);
        assert!(
            result.is_empty(),
            "empty blocks slice must produce empty result"
        );
    }

    #[test]
    fn test_order_load_blocks_nondeferrable_always_included() {
        let sequencer = RestorationSequencer::new(3);
        // CLP = 200.0 * 1.5 = 300 MW, far exceeds headroom of 10 MW
        let blocks = vec![make_load_block(0, 200.0, 1, false)];
        let result = sequencer.order_load_blocks(&blocks, 15.0, 5.0);
        assert!(
            result.contains(&0),
            "non-deferrable block must be included regardless of CLP vs headroom"
        );
    }

    #[test]
    fn test_order_load_blocks_reserve_reduces_headroom() {
        let sequencer = RestorationSequencer::new(3);
        // headroom = 50 - 40 = 10 MW
        // block 0: deferrable, CLP = 10.0 * 1.5 = 15 MW → excluded (15 > 10)
        // block 1: deferrable, CLP =  3.0 * 1.5 =  4.5 MW → included (4.5 ≤ 10)
        let blocks = vec![
            make_load_block(0, 10.0, 1, true),
            make_load_block(1, 3.0, 1, true),
        ];
        let result = sequencer.order_load_blocks(&blocks, 50.0, 40.0);
        assert!(
            result.contains(&1),
            "block 1 (4.5 MW CLP) must fit within 10 MW headroom"
        );
        assert!(
            !result.contains(&0),
            "block 0 (15 MW CLP) must be excluded when headroom is only 10 MW"
        );
    }

    #[test]
    fn test_compute_parallel_paths_limits_groups() {
        use crate::network::branch::Branch;

        let sequencer = RestorationSequencer::new(2);
        let mut net = PowerNetwork::new(100.0);
        // Three black-start buses (1, 3, 5) each with one neighbour (2, 4, 6)
        for id in [1, 2, 3, 4, 5, 6] {
            net.buses.push(Bus::new(id, BusType::Slack));
        }
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.1,
            x: 0.2,
            b: 0.01,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 1.0,
            shift: 0.0,
            status: true,
        });
        net.branches.push(Branch {
            from_bus: 3,
            to_bus: 4,
            r: 0.1,
            x: 0.2,
            b: 0.01,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 1.0,
            shift: 0.0,
            status: true,
        });
        net.branches.push(Branch {
            from_bus: 5,
            to_bus: 6,
            r: 0.1,
            x: 0.2,
            b: 0.01,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 1.0,
            shift: 0.0,
            status: true,
        });

        let groups = sequencer.compute_parallel_paths(&net, &[1, 3, 5]);
        assert!(
            groups.len() <= 2,
            "result must be clamped to max_parallel_paths=2, got {}",
            groups.len()
        );
    }

    #[test]
    fn test_metrics_zero_customers_returns_zero() {
        let metrics = RestorationMetrics::new(0, 0.0);
        let blocks = vec![make_load_block(0, 20.0, 1, false)];
        let plan = RestorationPlan {
            steps: vec![RestorationStep {
                step_id: 1,
                time_min: 30.0,
                action: RestorationAction::PickupLoadBlock {
                    block_id: 0,
                    actual_mw: 20.0,
                },
                available_generation_mw: 50.0,
                connected_load_mw: 20.0,
                frequency_hz: 50.0,
                notes: String::new(),
            }],
            total_time_min: 60.0,
            restored_load_pct: 1.0,
            n_black_start_units_used: 1,
            critical_loads_restored_min: 30.0,
            feasible: true,
            bottlenecks: vec![],
        };
        let customers_per_block = [100usize];
        assert!(
            (metrics.compute_saidi(&plan, &customers_per_block)).abs() < 1e-9,
            "SAIDI must be 0.0 when total_customers == 0"
        );
        assert!(
            (metrics.compute_saifi(&plan, &customers_per_block)).abs() < 1e-9,
            "SAIFI must be 0.0 when total_customers == 0"
        );
        // ENS is independent of customer count; block 0 restored at t=30 min
        // → duration_h = 0.5, ENS = 20 MW * 0.5 h = 10 MWh
        assert!(
            (metrics.compute_ens(&plan, &blocks) - 10.0_f64).abs() < 1e-9,
            "ENS must equal 10 MWh for the one restored block"
        );
    }

    #[test]
    fn test_metrics_ens_unrestored_block() {
        // outage starts at t=0, plan runs 120 min
        // block 0 restored at t=30, block 1 never restored
        let metrics = RestorationMetrics::new(100, 0.0);
        let blocks = vec![
            make_load_block(0, 20.0, 1, false), // 20 MW
            make_load_block(1, 10.0, 2, true),  // 10 MW — unrestored
        ];
        let plan = RestorationPlan {
            steps: vec![RestorationStep {
                step_id: 1,
                time_min: 30.0,
                action: RestorationAction::PickupLoadBlock {
                    block_id: 0,
                    actual_mw: 20.0,
                },
                available_generation_mw: 50.0,
                connected_load_mw: 20.0,
                frequency_hz: 50.0,
                notes: String::new(),
            }],
            total_time_min: 120.0,
            restored_load_pct: 0.5,
            n_black_start_units_used: 1,
            critical_loads_restored_min: 30.0,
            feasible: true,
            bottlenecks: vec![],
        };
        let ens = metrics.compute_ens(&plan, &blocks);
        // block 0 restored at t=30: ENS contribution = 20 MW * (30/60) h = 10 MWh
        // block 1 unrestored:       ENS contribution = 10 MW * (120/60) h = 20 MWh
        // total = 30 MWh
        let expected = 10.0_f64 + 20.0_f64;
        assert!(
            (ens - expected).abs() < 1e-9,
            "ENS must include unrestored block contribution: expected {expected}, got {ens}"
        );
    }
}
