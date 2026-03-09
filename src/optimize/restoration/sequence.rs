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
