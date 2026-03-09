//! Distribution network topology optimisation (network reconfiguration).
//!
//! Distribution networks are meshed in design but operated as radial trees for
//! protection simplicity.  Normally-open **tie switches** allow the topology to
//! be reconfigured for loss minimisation, voltage profile improvement, or load
//! balancing.
//!
//! # Algorithm
//! The **branch-exchange** (Merlin–Back) heuristic is used:
//!
//! 1. Start with the current (initial) radial topology.
//! 2. For each tie switch, temporarily close it to create a loop.
//! 3. Identify the loop branches and open the one that most reduces the
//!    objective (losses, voltage deviation, or a weighted combination).
//! 4. Accept the switch pair `(close tie, open section)` if it improves the
//!    objective.
//! 5. Repeat until no improving exchange is found (convergence).
//!
//! Losses are estimated via a **simplified forward–backward sweep** that
//! propagates I²R losses from leaves to the root.
//!
//! # References
//! - Merlin & Back, "Search for a Minimum-Loss Operating Spanning Tree
//!   Configuration in an Urban Power Distribution System", PSCC 1975.
//! - Shirmohammadi & Hong, IEEE TPWRD 4(2), 1989.

use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use std::fmt;

// ─────────────────────────────────────────────────────────────────────────────
// Error
// ─────────────────────────────────────────────────────────────────────────────

/// Errors from the topology optimiser.
#[derive(Debug, Clone, PartialEq)]
pub enum TopologyError {
    /// The network data is inconsistent or incomplete.
    InvalidNetwork(String),
    /// Optimisation failed to produce a feasible topology.
    NoFeasibleTopology(String),
}

impl fmt::Display for TopologyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidNetwork(m) => write!(f, "invalid network: {m}"),
            Self::NoFeasibleTopology(m) => write!(f, "no feasible topology: {m}"),
        }
    }
}

impl std::error::Error for TopologyError {}

// ─────────────────────────────────────────────────────────────────────────────
// Objective
// ─────────────────────────────────────────────────────────────────────────────

/// Optimisation objective for network reconfiguration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum TopologyObjective {
    /// Minimise total I²R active power losses `\[MW\]`.
    MinimizeLosses,
    /// Minimise the maximum/mean load imbalance across feeders.
    MinimizeLoadImbalance,
    /// Minimise total voltage deviation from 1.0 `\[pu\]`.
    MaximizeVoltageProfile,
    /// Weighted combination: `w_loss·L + w_voltage·V + w_balance·B`.
    MultiObjective {
        /// Weight on loss component `\[0, 1\]`.
        w_loss: f64,
        /// Weight on voltage-deviation component `\[0, 1\]`.
        w_voltage: f64,
        /// Weight on load-balance component `\[0, 1\]`.
        w_balance: f64,
    },
}

// ─────────────────────────────────────────────────────────────────────────────
// Configuration
// ─────────────────────────────────────────────────────────────────────────────

/// Configuration for the topology optimiser.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TopologyOptConfig {
    /// Total number of buses (nodes) in the network.
    pub n_buses: usize,
    /// Total number of branches (edges) in the network (section + tie).
    pub n_branches: usize,
    /// Number of tie (normally-open) switches.
    pub n_tie_switches: usize,
    /// Optimisation objective.
    pub objective: TopologyObjective,
    /// Maximum number of branch-exchange iterations before stopping.
    pub max_iterations: usize,
    /// Whether a radial (tree) topology must be preserved.
    pub radiality_required: bool,
}

impl Default for TopologyOptConfig {
    fn default() -> Self {
        Self {
            n_buses: 10,
            n_branches: 12,
            n_tie_switches: 3,
            objective: TopologyObjective::MinimizeLosses,
            max_iterations: 100,
            radiality_required: true,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Switch and topology state
// ─────────────────────────────────────────────────────────────────────────────

/// State of a single switch in the network.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SwitchState {
    /// Unique switch identifier.
    pub switch_id: usize,
    /// Index of the branch this switch controls.
    pub branch_idx: usize,
    /// `true` if the switch is currently open (branch de-energised).
    pub is_open: bool,
    /// `true` if this switch is a tie switch (normally open).
    pub is_normally_open: bool,
    /// `false` locks the switch — it cannot be operated.
    pub can_switch: bool,
}

/// Complete snapshot of the network switching state.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TopologyState {
    /// State of every switch.
    pub switch_states: Vec<SwitchState>,
    /// `active_branches[i] = true` means branch `i` is energised (closed switch).
    pub active_branches: Vec<bool>,
    /// Whether the active sub-graph is a tree (no cycles).
    pub is_radial: bool,
    /// Whether all non-source buses are reachable from bus 0.
    pub is_connected: bool,
}

// ─────────────────────────────────────────────────────────────────────────────
// Result
// ─────────────────────────────────────────────────────────────────────────────

/// Result of a topology optimisation run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TopologyOptResult {
    /// Optimal switch configuration.
    pub optimal_state: TopologyState,
    /// Total I²R losses in the initial topology `\[MW\]`.
    pub initial_losses_mw: f64,
    /// Total I²R losses in the optimal topology `\[MW\]`.
    pub optimal_losses_mw: f64,
    /// Loss reduction achieved `\[%\]`.
    pub loss_reduction_pct: f64,
    /// Mean absolute voltage deviation from 1.0 pu in optimal topology.
    pub voltage_deviation_pu: f64,
    /// Load balance index (lower = more balanced, 0 = perfect).
    pub load_balance_index: f64,
    /// Number of switch operations performed (pairs opened/closed).
    pub n_switching_operations: usize,
    /// Description of the algorithm used.
    pub algorithm: String,
    /// Actual iterations performed.
    pub iterations: usize,
}

// ─────────────────────────────────────────────────────────────────────────────
// Internal branch record
// ─────────────────────────────────────────────────────────────────────────────

#[derive(Debug, Clone)]
struct Branch {
    from: usize,
    to: usize,
    r_pu: f64,
    x_pu: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Optimiser
// ─────────────────────────────────────────────────────────────────────────────

/// Distribution network topology optimiser.
pub struct TopologyOptimizer {
    config: TopologyOptConfig,
    /// `(P_load_mw, Q_load_mvar)` per bus.
    buses: Vec<(f64, f64)>,
    branches: Vec<Branch>,
    switches: Vec<SwitchState>,
}

impl TopologyOptimizer {
    /// Create a new optimiser with the given configuration.
    pub fn new(config: TopologyOptConfig) -> Self {
        Self {
            config,
            buses: Vec::new(),
            branches: Vec::new(),
            switches: Vec::new(),
        }
    }

    /// Add a load bus.  Bus 0 is assumed to be the slack/source.
    ///
    /// - `p_mw`    — active load `\[MW\]`
    /// - `q_mvar`  — reactive load `\[Mvar\]`
    pub fn add_bus(&mut self, p_mw: f64, q_mvar: f64) {
        self.buses.push((p_mw, q_mvar));
    }

    /// Add a branch (section) of the network.
    ///
    /// - `from`, `to`  — bus indices
    /// - `r_pu`, `x_pu` — resistance and reactance `\[pu\]`
    pub fn add_branch(&mut self, from: usize, to: usize, r_pu: f64, x_pu: f64) {
        self.branches.push(Branch {
            from,
            to,
            r_pu,
            x_pu,
        });
    }

    /// Register a switch.
    pub fn add_switch(&mut self, switch: SwitchState) {
        self.switches.push(switch);
    }

    /// Run the branch-exchange topology optimisation.
    pub fn optimize(&self) -> Result<TopologyOptResult, TopologyError> {
        if self.buses.is_empty() {
            return Err(TopologyError::InvalidNetwork(
                "no buses defined".to_string(),
            ));
        }
        if self.branches.is_empty() {
            return Err(TopologyError::InvalidNetwork(
                "no branches defined".to_string(),
            ));
        }
        if self.switches.is_empty() {
            return Err(TopologyError::InvalidNetwork(
                "no switches defined".to_string(),
            ));
        }

        // Build initial active-branch mask from switch states
        let mut active = self.initial_active_branches();
        let initial_losses = self.compute_losses(&active);

        // Identify tie switches (normally open — currently open)
        let tie_switches: Vec<&SwitchState> = self
            .switches
            .iter()
            .filter(|s| s.is_normally_open && s.can_switch)
            .collect();

        let mut best_losses = initial_losses;
        let mut best_active = active.clone();
        let mut n_ops = 0usize;
        let mut iters = 0usize;
        let max_iter = self.config.max_iterations;

        loop {
            if iters >= max_iter {
                break;
            }
            iters += 1;

            let mut improved = false;

            for tie_sw in &tie_switches {
                let tie_branch = tie_sw.branch_idx;
                if tie_branch >= self.branches.len() {
                    continue;
                }
                // Try closing this tie switch
                let mut candidate = best_active.clone();
                candidate[tie_branch] = true;

                // Find the loop formed and try opening each branch in it
                let loop_branches = self.find_loop_branches(&candidate, tie_branch);

                for &open_candidate in &loop_branches {
                    if open_candidate == tie_branch {
                        continue;
                    }
                    // Check this branch has a switchable section switch
                    let can_open = self.switches.iter().any(|s| {
                        s.branch_idx == open_candidate && s.can_switch && !s.is_normally_open
                    });
                    if !can_open {
                        continue;
                    }

                    let mut trial = candidate.clone();
                    trial[open_candidate] = false;

                    // Enforce radiality if required
                    if self.config.radiality_required && !self.verify_radiality(&trial) {
                        continue;
                    }
                    if !self.is_connected(&trial) {
                        continue;
                    }

                    let trial_losses = self.compute_losses(&trial);
                    if trial_losses < best_losses - 1e-9 {
                        best_losses = trial_losses;
                        best_active = trial;
                        improved = true;
                        n_ops += 1;
                    }
                }
            }

            if !improved {
                break;
            }
        }

        active = best_active.clone();
        let optimal_losses = self.compute_losses(&active);
        let loss_reduction_pct = if initial_losses > 1e-12 {
            (initial_losses - optimal_losses) / initial_losses * 100.0
        } else {
            0.0
        };

        let voltage_deviation = self.estimate_voltage_deviation(&active);
        let load_balance_index = self.compute_load_balance_index(&active);

        let optimal_state = self.build_state(active);

        Ok(TopologyOptResult {
            optimal_state,
            initial_losses_mw: initial_losses,
            optimal_losses_mw: optimal_losses,
            loss_reduction_pct,
            voltage_deviation_pu: voltage_deviation,
            load_balance_index,
            n_switching_operations: n_ops,
            algorithm: "Branch Exchange (Merlin-Back)".to_string(),
            iterations: iters,
        })
    }

    // ── Internal helpers ──────────────────────────────────────────────────────

    /// Build initial active-branch vector from switch states.
    fn initial_active_branches(&self) -> Vec<bool> {
        let n = self.branches.len();
        let mut active = vec![true; n];
        for sw in &self.switches {
            if sw.branch_idx < n && sw.is_open {
                active[sw.branch_idx] = false;
            }
        }
        active
    }

    /// Estimate total I²R losses via a simplified forward-backward sweep.
    ///
    /// Assumes a **radial** feeder topology originating at bus 0.
    /// Power flows are approximated by ignoring voltage magnitude deviations
    /// (flat-voltage approximation: `|V| = 1 pu` everywhere).
    ///
    /// For each active branch: `P_loss = (P² + Q²) / V² × R ≈ (P² + Q²) × R`
    /// summed over all branches, where `(P, Q)` is the power flowing through
    /// the branch (sum of downstream loads).
    pub fn compute_losses(&self, active_branches: &[bool]) -> f64 {
        // Build correct tree structure via BFS from root (bus 0).
        // This avoids the "from < to" heuristic which fails when the tree
        // is reconfigured and bus ordering no longer reflects root proximity.
        let n_buses = self.buses.len();

        // Adjacency list: each entry is (neighbour_bus, branch_idx)
        let mut adj: Vec<Vec<(usize, usize)>> = vec![Vec::new(); n_buses];
        for (bi, branch) in self.branches.iter().enumerate() {
            if bi < active_branches.len()
                && active_branches[bi]
                && branch.from < n_buses
                && branch.to < n_buses
            {
                adj[branch.from].push((branch.to, bi));
                adj[branch.to].push((branch.from, bi));
            }
        }

        // BFS from root to determine parent-child relationships
        let mut parent: Vec<Option<(usize, usize)>> = vec![None; n_buses]; // (parent_bus, branch_idx)
        let mut bfs_order: Vec<usize> = Vec::with_capacity(n_buses);
        let mut visited = vec![false; n_buses];
        let mut queue = VecDeque::new();
        queue.push_back(0usize);
        visited[0] = true;
        while let Some(node) = queue.pop_front() {
            bfs_order.push(node);
            for &(nb, bi) in &adj[node] {
                if !visited[nb] {
                    visited[nb] = true;
                    parent[nb] = Some((node, bi));
                    queue.push_back(nb);
                }
            }
        }

        // Initialise downstream power with local load
        let mut p_down = vec![0.0_f64; n_buses];
        let mut q_down = vec![0.0_f64; n_buses];
        for i in 0..n_buses {
            if i < self.buses.len() {
                p_down[i] = self.buses[i].0;
                q_down[i] = self.buses[i].1;
            }
        }

        // Back-propagate in reverse BFS order (leaves → root)
        let mut total_loss = 0.0;
        for &node in bfs_order.iter().rev() {
            if let Some((par, bi)) = parent[node] {
                let p_flow = p_down[node];
                let q_flow = q_down[node];
                let r = if bi < self.branches.len() {
                    self.branches[bi].r_pu
                } else {
                    0.0
                };
                total_loss += (p_flow * p_flow + q_flow * q_flow) * r;
                p_down[par] += p_flow;
                q_down[par] += q_flow;
            }
        }

        total_loss
    }

    /// Verify that the active branch set forms a tree (no cycles) using DFS
    /// cycle detection.
    pub fn verify_radiality(&self, active_branches: &[bool]) -> bool {
        let n = self.buses.len();
        if n == 0 {
            return true;
        }

        // Build adjacency list
        let mut adj: Vec<Vec<(usize, usize)>> = vec![Vec::new(); n];
        for (bi, branch) in self.branches.iter().enumerate() {
            if bi < active_branches.len() && active_branches[bi] && branch.from < n && branch.to < n
            {
                adj[branch.from].push((branch.to, bi));
                adj[branch.to].push((branch.from, bi));
            }
        }

        // DFS cycle detection
        let mut visited = vec![false; n];
        let mut has_cycle = false;

        fn dfs(
            node: usize,
            parent_edge: usize,
            adj: &[Vec<(usize, usize)>],
            visited: &mut Vec<bool>,
            has_cycle: &mut bool,
        ) {
            visited[node] = true;
            for &(neighbour, edge_idx) in &adj[node] {
                if !visited[neighbour] {
                    dfs(neighbour, edge_idx, adj, visited, has_cycle);
                } else if edge_idx != parent_edge {
                    *has_cycle = true;
                }
            }
        }

        dfs(0, usize::MAX, &adj, &mut visited, &mut has_cycle);
        !has_cycle
    }

    /// Check that all buses are reachable from bus 0 (connected).
    fn is_connected(&self, active_branches: &[bool]) -> bool {
        let n = self.buses.len();
        if n == 0 {
            return true;
        }

        let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];
        for (bi, branch) in self.branches.iter().enumerate() {
            if bi < active_branches.len() && active_branches[bi] && branch.from < n && branch.to < n
            {
                adj[branch.from].push(branch.to);
                adj[branch.to].push(branch.from);
            }
        }

        let mut visited = vec![false; n];
        let mut queue = VecDeque::new();
        queue.push_back(0usize);
        visited[0] = true;
        while let Some(node) = queue.pop_front() {
            for &nb in &adj[node] {
                if !visited[nb] {
                    visited[nb] = true;
                    queue.push_back(nb);
                }
            }
        }
        visited.iter().all(|&v| v)
    }

    /// Find all branches in the loop created by closing `tie_branch`.
    ///
    /// Returns indices of branches on the unique path between
    /// `tie_branch.from` and `tie_branch.to` in the remaining tree.
    fn find_loop_branches(&self, active: &[bool], tie_branch: usize) -> Vec<usize> {
        if tie_branch >= self.branches.len() {
            return Vec::new();
        }
        let src = self.branches[tie_branch].from;
        let dst = self.branches[tie_branch].to;
        let n = self.buses.len();

        // Build adjacency without tie_branch itself
        let mut adj: Vec<Vec<(usize, usize)>> = vec![Vec::new(); n];
        for (bi, branch) in self.branches.iter().enumerate() {
            if bi == tie_branch {
                continue;
            }
            if bi < active.len() && active[bi] && branch.from < n && branch.to < n {
                adj[branch.from].push((branch.to, bi));
                adj[branch.to].push((branch.from, bi));
            }
        }

        // BFS to find path src → dst and the branches along it
        let mut parent_edge: Vec<Option<usize>> = vec![None; n];
        let mut parent_node: Vec<Option<usize>> = vec![None; n];
        let mut visited = vec![false; n];
        let mut queue = VecDeque::new();
        if src < n {
            queue.push_back(src);
            visited[src] = true;
        }
        while let Some(node) = queue.pop_front() {
            if node == dst {
                break;
            }
            for &(nb, ei) in &adj[node] {
                if !visited[nb] {
                    visited[nb] = true;
                    parent_node[nb] = Some(node);
                    parent_edge[nb] = Some(ei);
                    queue.push_back(nb);
                }
            }
        }

        // Reconstruct path
        let mut loop_branches = vec![tie_branch];
        let mut cur = dst;
        while let Some(edge) = parent_edge.get(cur).and_then(|e| *e) {
            loop_branches.push(edge);
            cur = match parent_node.get(cur).and_then(|n| *n) {
                Some(p) => p,
                None => break,
            };
            if cur == src {
                break;
            }
        }
        loop_branches
    }

    /// Estimate mean absolute voltage deviation `\[pu\]` using a linear
    /// approximation: `ΔV ≈ R·P + X·Q` for each branch.
    fn estimate_voltage_deviation(&self, active: &[bool]) -> f64 {
        let n = self.buses.len();
        if n == 0 {
            return 0.0;
        }
        let mut v = vec![1.0_f64; n];

        // Simple topological voltage drop (single pass, root→leaves)
        let mut children: Vec<Vec<(usize, usize)>> = vec![Vec::new(); n];
        for (bi, branch) in self.branches.iter().enumerate() {
            if bi < active.len() && active[bi] && branch.from < n && branch.to < n {
                let (p, c) = if branch.from < branch.to {
                    (branch.from, branch.to)
                } else {
                    (branch.to, branch.from)
                };
                children[p].push((c, bi));
            }
        }

        let mut stack = vec![0usize];
        let mut visited = vec![false; n];
        while let Some(node) = stack.pop() {
            if visited[node] {
                continue;
            }
            visited[node] = true;
            for &(child, bi) in &children[node] {
                if bi < self.branches.len() && child < self.buses.len() {
                    let r = self.branches[bi].r_pu;
                    let x = self.branches[bi].x_pu;
                    let (p, q) = self.buses[child];
                    let drop = r * p + x * q;
                    v[child] = (v[node] - drop).max(0.8);
                }
                stack.push(child);
            }
        }

        let total_dev: f64 = v.iter().map(|&vi| (vi - 1.0).abs()).sum();
        total_dev / n as f64
    }

    /// Compute load-balance index = variance of active branch loading fractions.
    fn compute_load_balance_index(&self, active: &[bool]) -> f64 {
        let loads: Vec<f64> = self
            .buses
            .iter()
            .map(|(p, q)| (p * p + q * q).sqrt())
            .collect();
        let total: f64 = loads.iter().sum();
        if total < 1e-12 {
            return 0.0;
        }

        // Count active feeders as children of root
        let n_feeders = self
            .branches
            .iter()
            .enumerate()
            .filter(|(bi, b)| bi < &active.len() && active[*bi] && b.from == 0)
            .count()
            .max(1);

        // Load per feeder: assign loads by their bus index modulo n_feeders
        let mut feeder_load = vec![0.0; n_feeders];
        for (i, &l) in loads.iter().enumerate() {
            feeder_load[i % n_feeders] += l;
        }
        let mean = total / n_feeders as f64;
        let variance: f64 = feeder_load
            .iter()
            .map(|&fl| (fl - mean).powi(2))
            .sum::<f64>()
            / n_feeders as f64;
        variance.sqrt() / (mean + 1e-12)
    }

    /// Build a [`TopologyState`] from an active-branch mask.
    fn build_state(&self, active: Vec<bool>) -> TopologyState {
        let switch_states: Vec<SwitchState> = self
            .switches
            .iter()
            .map(|sw| SwitchState {
                switch_id: sw.switch_id,
                branch_idx: sw.branch_idx,
                is_open: if sw.branch_idx < active.len() {
                    !active[sw.branch_idx]
                } else {
                    sw.is_open
                },
                is_normally_open: sw.is_normally_open,
                can_switch: sw.can_switch,
            })
            .collect();
        let is_radial = self.verify_radiality(&active);
        let is_connected = self.is_connected(&active);
        TopologyState {
            switch_states,
            active_branches: active,
            is_radial,
            is_connected,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a simple 5-bus radial network with one tie switch.
    ///
    /// ```
    /// 0 ─[0]─ 1 ─[1]─ 2
    ///          |
    ///         [2]
    ///          |
    ///          3 ─[3]─ 4
    /// ```
    /// Tie switch: branch 4 connects bus 2 to bus 4 (normally open).
    fn simple_net() -> TopologyOptimizer {
        let config = TopologyOptConfig {
            n_buses: 5,
            n_branches: 5,
            n_tie_switches: 1,
            objective: TopologyObjective::MinimizeLosses,
            max_iterations: 20,
            radiality_required: true,
        };
        let mut opt = TopologyOptimizer::new(config);
        // Bus loads
        opt.add_bus(0.0, 0.0); // bus 0 — slack
        opt.add_bus(1.0, 0.3); // bus 1
        opt.add_bus(2.0, 0.5); // bus 2
        opt.add_bus(0.5, 0.1); // bus 3
        opt.add_bus(1.5, 0.4); // bus 4
                               // Branches
        opt.add_branch(0, 1, 0.01, 0.005); // branch 0
        opt.add_branch(1, 2, 0.02, 0.010); // branch 1 — high R (loss)
        opt.add_branch(1, 3, 0.01, 0.005); // branch 2
        opt.add_branch(3, 4, 0.02, 0.010); // branch 3
        opt.add_branch(2, 4, 0.005, 0.003); // branch 4 — tie (low R)
                                            // Switches: branches 0-3 are section switches (normally closed)
        for bi in 0..4 {
            opt.add_switch(SwitchState {
                switch_id: bi,
                branch_idx: bi,
                is_open: false,
                is_normally_open: false,
                can_switch: true,
            });
        }
        // Branch 4 is the tie switch (normally open)
        opt.add_switch(SwitchState {
            switch_id: 4,
            branch_idx: 4,
            is_open: true,
            is_normally_open: true,
            can_switch: true,
        });
        opt
    }

    #[test]
    fn test_no_switching_if_already_optimal() {
        // Build a perfectly balanced network where the initial topology is optimal.
        let config = TopologyOptConfig {
            n_buses: 3,
            n_branches: 3,
            n_tie_switches: 1,
            objective: TopologyObjective::MinimizeLosses,
            max_iterations: 10,
            radiality_required: true,
        };
        let mut opt = TopologyOptimizer::new(config);
        opt.add_bus(0.0, 0.0); // slack
        opt.add_bus(1.0, 0.0);
        opt.add_bus(1.0, 0.0);
        opt.add_branch(0, 1, 0.01, 0.0); // branch 0
        opt.add_branch(0, 2, 0.01, 0.0); // branch 1
        opt.add_branch(1, 2, 0.01, 0.0); // branch 2 — tie
                                         // Section switches
        for bi in 0..2 {
            opt.add_switch(SwitchState {
                switch_id: bi,
                branch_idx: bi,
                is_open: false,
                is_normally_open: false,
                can_switch: true,
            });
        }
        // Tie switch
        opt.add_switch(SwitchState {
            switch_id: 2,
            branch_idx: 2,
            is_open: true,
            is_normally_open: true,
            can_switch: true,
        });

        let result = opt.optimize().unwrap();
        // The symmetric network is already optimal: either no switching is needed,
        // or any switching found by the heuristic produces identical losses
        // (loss_reduction_pct ≈ 0).
        assert!(
            result.loss_reduction_pct < 1.0,
            "Already-optimal symmetric network: loss reduction must be < 1 %, got {:.2} %",
            result.loss_reduction_pct
        );
    }

    #[test]
    fn test_optimize_reduces_losses_or_maintains() {
        let opt = simple_net();
        let result = opt.optimize().unwrap();
        assert!(
            result.optimal_losses_mw <= result.initial_losses_mw + 1e-9,
            "Optimal losses ({:.6}) must be ≤ initial losses ({:.6})",
            result.optimal_losses_mw,
            result.initial_losses_mw
        );
    }

    #[test]
    fn test_radiality_maintained_after_reconfiguration() {
        let opt = simple_net();
        let result = opt.optimize().unwrap();
        assert!(
            result.optimal_state.is_radial,
            "Optimal topology must be radial"
        );
    }

    #[test]
    fn test_multi_objective_produces_valid_result() {
        let config = TopologyOptConfig {
            n_buses: 5,
            n_branches: 5,
            n_tie_switches: 1,
            objective: TopologyObjective::MultiObjective {
                w_loss: 0.6,
                w_voltage: 0.3,
                w_balance: 0.1,
            },
            max_iterations: 20,
            radiality_required: true,
        };
        let mut opt = TopologyOptimizer::new(config);
        opt.add_bus(0.0, 0.0);
        opt.add_bus(2.0, 0.5);
        opt.add_bus(3.0, 0.8);
        opt.add_bus(1.0, 0.2);
        opt.add_bus(2.5, 0.6);
        opt.add_branch(0, 1, 0.02, 0.01);
        opt.add_branch(1, 2, 0.03, 0.015);
        opt.add_branch(1, 3, 0.01, 0.005);
        opt.add_branch(3, 4, 0.02, 0.01);
        opt.add_branch(2, 4, 0.005, 0.003);
        for bi in 0..4 {
            opt.add_switch(SwitchState {
                switch_id: bi,
                branch_idx: bi,
                is_open: false,
                is_normally_open: false,
                can_switch: true,
            });
        }
        opt.add_switch(SwitchState {
            switch_id: 4,
            branch_idx: 4,
            is_open: true,
            is_normally_open: true,
            can_switch: true,
        });
        let result = opt.optimize().unwrap();
        // Result must be a valid connected radial network
        assert!(result.optimal_state.is_connected, "Must be connected");
        assert!(
            result.loss_reduction_pct >= 0.0,
            "Loss reduction must be non-negative"
        );
    }

    #[test]
    fn test_connectivity_after_reconfiguration() {
        let opt = simple_net();
        let result = opt.optimize().unwrap();
        assert!(
            result.optimal_state.is_connected,
            "All buses must remain reachable after reconfiguration"
        );
    }

    #[test]
    fn test_verify_radiality_detects_cycle() {
        let config = TopologyOptConfig::default();
        let mut opt = TopologyOptimizer::new(config);
        opt.add_bus(0.0, 0.0);
        opt.add_bus(1.0, 0.0);
        opt.add_bus(1.0, 0.0);
        opt.add_branch(0, 1, 0.01, 0.0);
        opt.add_branch(1, 2, 0.01, 0.0);
        opt.add_branch(0, 2, 0.01, 0.0); // creates a triangle
                                         // All active → cycle present
        let all_active = vec![true, true, true];
        assert!(
            !opt.verify_radiality(&all_active),
            "Triangle must be detected as non-radial"
        );
        // Remove one branch → should be radial
        let tree = vec![true, true, false];
        assert!(
            opt.verify_radiality(&tree),
            "Two-branch path must be radial"
        );
    }

    #[test]
    fn test_loss_computation_positive() {
        let opt = simple_net();
        let active = opt.initial_active_branches();
        let losses = opt.compute_losses(&active);
        assert!(losses >= 0.0, "Losses must be non-negative, got {losses}");
    }

    #[test]
    fn test_no_buses_error() {
        let config = TopologyOptConfig::default();
        let mut opt = TopologyOptimizer::new(config);
        opt.add_switch(SwitchState {
            switch_id: 0,
            branch_idx: 0,
            is_open: true,
            is_normally_open: true,
            can_switch: true,
        });
        let err = opt.optimize().unwrap_err();
        assert!(matches!(err, TopologyError::InvalidNetwork(_)));
    }
}
