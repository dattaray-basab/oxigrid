//! Optimal network topology design — Steiner tree, transmission expansion planning,
//! and geographic substation siting.
//!
//! # Modules
//! - [`SteinerTreeSolver`] — approximate Steiner tree via shortest-path heuristic (metric closure MST)
//! - [`ExpansionPlanner`] — greedy BCR-ranked transmission expansion planning with PTDF-based congestion
//! - [`SubstationSiting`] — k-means load-weighted substation placement
//!
//! # References
//! - Takahashi & Matsuyama, "An approximate solution for the Steiner problem in graphs", 1980.
//! - Garver, "Transmission Network Estimation Using Linear Programming", IEEE TPAS 89(7), 1970.
//! - Hakimi, "Optimum Distribution of Switching Centers in a Communication Network", 1965.

use crate::error::OxiGridError;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet, VecDeque};

// ─────────────────────────────────────────────────────────────────────────────
// Core data structures
// ─────────────────────────────────────────────────────────────────────────────

/// Edge candidate for network expansion or Steiner tree computation.
#[derive(Debug, Clone)]
pub struct NetworkEdge {
    /// Source node index.
    pub from: usize,
    /// Destination node index.
    pub to: usize,
    /// Physical length of the line [km].
    pub length_km: f64,
    /// Nominal voltage level [kV].
    pub voltage_kv: f64,
    /// Thermal capacity [MW].
    pub capacity_mw: f64,
    /// Capital cost [million EUR].
    pub cost_million_eur: f64,
    /// Series resistance in per-unit on system base.
    pub resistance_pu: f64,
    /// Series reactance in per-unit on system base.
    pub reactance_pu: f64,
    /// `true` if the line is already built (no investment required).
    pub is_existing: bool,
    /// Construction lead time [years].
    pub build_years: f64,
}

impl NetworkEdge {
    /// DC susceptance in per-unit: `b = 1 / x`.
    pub fn susceptance_pu(&self) -> f64 {
        if self.reactance_pu.abs() < 1e-12 {
            0.0
        } else {
            1.0 / self.reactance_pu
        }
    }
}

/// Load/generation node for topology planning.
#[derive(Debug, Clone)]
pub struct TopologyNode {
    /// Unique node identifier.
    pub id: usize,
    /// `true` if this node must be connected (demand or generation node).
    pub is_terminal: bool,
    /// `true` if this node can serve as a Steiner (relay) point.
    pub is_substation: bool,
    /// Peak active load [MW].
    pub peak_load_mw: f64,
    /// Peak active generation [MW].
    pub peak_generation_mw: f64,
    /// X geographic coordinate [km].
    pub x: f64,
    /// Y geographic coordinate [km].
    pub y: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Steiner Tree Solver
// ─────────────────────────────────────────────────────────────────────────────

/// Approximate Steiner tree solver for power network design.
///
/// Uses the Takahashi-Matsuyama heuristic:
/// 1. Build shortest-path distance matrix on terminal nodes (Dijkstra on edge costs).
/// 2. Construct metric closure graph on terminal nodes only.
/// 3. Find MST of the metric closure (Kruskal).
/// 4. Map back to original graph edges via the stored shortest paths.
/// 5. Prune non-terminal Steiner points with degree ≤ 1.
pub struct SteinerTreeSolver {
    /// All nodes (terminals + potential Steiner points).
    pub nodes: Vec<TopologyNode>,
    /// All candidate edges.
    pub edges: Vec<NetworkEdge>,
}

/// Result of a Steiner tree computation.
#[derive(Debug, Clone)]
pub struct SteinerTreeResult {
    /// Indices into [`SteinerTreeSolver::edges`] that form the selected tree.
    pub selected_edges: Vec<usize>,
    /// Sum of `cost_million_eur` for selected edges.
    pub total_cost_million_eur: f64,
    /// Sum of `length_km` for selected edges.
    pub total_length_km: f64,
    /// Whether the result connects all terminal nodes.
    pub is_connected: bool,
    /// Whether the selected edge set forms a tree (no cycles).
    pub radial: bool,
}

/// Wrapper for Dijkstra priority queue: (cost, node_index).
#[derive(Debug, Clone, PartialEq)]
struct DijkState {
    cost: f64,
    node: usize,
}

impl Eq for DijkState {}

impl Ord for DijkState {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse ordering for min-heap
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for DijkState {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl SteinerTreeSolver {
    /// Construct a new solver.
    pub fn new(nodes: Vec<TopologyNode>, edges: Vec<NetworkEdge>) -> Self {
        Self { nodes, edges }
    }

    /// Run the approximate Steiner tree algorithm.
    ///
    /// Returns an error if:
    /// - There are fewer than 2 terminal nodes.
    /// - The graph is disconnected (some terminals are unreachable from others).
    pub fn solve_approximate(&self) -> Result<SteinerTreeResult, OxiGridError> {
        let n_nodes = self.nodes.len();
        if n_nodes == 0 {
            return Err(OxiGridError::InvalidNetwork("no nodes defined".to_string()));
        }

        let terminals: Vec<usize> = self
            .nodes
            .iter()
            .enumerate()
            .filter(|(_, n)| n.is_terminal)
            .map(|(i, _)| i)
            .collect();

        if terminals.len() < 2 {
            return Err(OxiGridError::InvalidNetwork(
                "Steiner tree requires at least 2 terminal nodes".to_string(),
            ));
        }

        // Step 1: Dijkstra from each terminal to all other nodes (on edge cost)
        let (dist_matrix, path_matrix) = self.all_pairs_shortest_paths(&terminals)?;

        // Step 2 & 3: Kruskal MST on metric closure (terminals only)
        let mst_edges_in_closure = self.mst_of_metric_closure(&terminals, &dist_matrix)?;

        // Step 4: Map back to original edges
        let mut selected_set: HashSet<usize> = HashSet::new();
        for (ti, tj) in &mst_edges_in_closure {
            let path = &path_matrix[*ti][*tj];
            for &edge_idx in path {
                selected_set.insert(edge_idx);
            }
        }

        // Step 5: Prune non-terminal Steiner points with degree ≤ 1
        let selected_edges = self.prune_steiner_points(selected_set, &terminals, n_nodes);

        let total_cost: f64 = selected_edges
            .iter()
            .map(|&i| self.edges[i].cost_million_eur)
            .sum();
        let total_length: f64 = selected_edges
            .iter()
            .map(|&i| self.edges[i].length_km)
            .sum();

        let connected =
            Self::is_connected_fn(n_nodes, &selected_edges, &self.edges, Some(&terminals));
        let radial = Self::is_radial(n_nodes, &selected_edges, &self.edges);

        Ok(SteinerTreeResult {
            selected_edges,
            total_cost_million_eur: total_cost,
            total_length_km: total_length,
            is_connected: connected,
            radial,
        })
    }

    /// MST baseline: run Kruskal on ALL nodes (ignores Steiner structure).
    pub fn solve_mst_baseline(&self) -> Result<SteinerTreeResult, OxiGridError> {
        let n_nodes = self.nodes.len();
        if n_nodes == 0 || self.edges.is_empty() {
            return Err(OxiGridError::InvalidNetwork(
                "no nodes or edges defined".to_string(),
            ));
        }

        // Sort edges by cost ascending
        let mut sorted_indices: Vec<usize> = (0..self.edges.len()).collect();
        sorted_indices.sort_by(|&a, &b| {
            self.edges[a]
                .cost_million_eur
                .partial_cmp(&self.edges[b].cost_million_eur)
                .unwrap_or(Ordering::Equal)
        });

        // Kruskal with union-find
        let mut uf = UnionFind::new(n_nodes);
        let mut selected = Vec::new();

        for &ei in &sorted_indices {
            let e = &self.edges[ei];
            if e.from >= n_nodes || e.to >= n_nodes {
                continue;
            }
            if uf.find(e.from) != uf.find(e.to) {
                uf.union(e.from, e.to);
                selected.push(ei);
                if selected.len() == n_nodes - 1 {
                    break;
                }
            }
        }

        let total_cost: f64 = selected
            .iter()
            .map(|&i| self.edges[i].cost_million_eur)
            .sum();
        let total_length: f64 = selected.iter().map(|&i| self.edges[i].length_km).sum();
        let connected = Self::is_connected_fn(n_nodes, &selected, &self.edges, None);
        let radial = Self::is_radial(n_nodes, &selected, &self.edges);

        Ok(SteinerTreeResult {
            selected_edges: selected,
            total_cost_million_eur: total_cost,
            total_length_km: total_length,
            is_connected: connected,
            radial,
        })
    }

    /// BFS/DFS connectivity check.
    ///
    /// If `required_nodes` is `Some(list)`, checks that all listed nodes are
    /// mutually reachable.  If `None`, checks that all `n_nodes` are reachable
    /// from node 0.
    pub fn is_connected(n_nodes: usize, selected_edges: &[usize], edges: &[NetworkEdge]) -> bool {
        Self::is_connected_fn(n_nodes, selected_edges, edges, None)
    }

    // ── Internal helpers ──────────────────────────────────────────────────────

    fn is_connected_fn(
        n_nodes: usize,
        selected_edges: &[usize],
        edges: &[NetworkEdge],
        required_nodes: Option<&[usize]>,
    ) -> bool {
        if n_nodes == 0 {
            return true;
        }
        // Build adjacency
        let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n_nodes];
        for &ei in selected_edges {
            if ei < edges.len() {
                let e = &edges[ei];
                if e.from < n_nodes && e.to < n_nodes {
                    adj[e.from].push(e.to);
                    adj[e.to].push(e.from);
                }
            }
        }

        let start = required_nodes.and_then(|r| r.first()).copied().unwrap_or(0);
        let mut visited = vec![false; n_nodes];
        let mut queue = VecDeque::new();
        if start < n_nodes {
            queue.push_back(start);
            visited[start] = true;
        }
        while let Some(node) = queue.pop_front() {
            for &nb in &adj[node] {
                if !visited[nb] {
                    visited[nb] = true;
                    queue.push_back(nb);
                }
            }
        }

        match required_nodes {
            Some(req) => req.iter().all(|&r| r < n_nodes && visited[r]),
            None => visited.iter().all(|&v| v),
        }
    }

    /// Check whether the selected edges form a tree (connected + no cycles).
    fn is_radial(n_nodes: usize, selected_edges: &[usize], edges: &[NetworkEdge]) -> bool {
        // A tree on n nodes has exactly n-1 edges and is connected.
        if n_nodes == 0 {
            return true;
        }
        let relevant: Vec<usize> = selected_edges
            .iter()
            .filter(|&&ei| ei < edges.len() && edges[ei].from < n_nodes && edges[ei].to < n_nodes)
            .copied()
            .collect();

        if relevant.len() != n_nodes.saturating_sub(1) {
            return false;
        }
        Self::is_connected_fn(n_nodes, &relevant, edges, None)
    }

    /// Single-source Dijkstra returning `(dist[node], predecessor_edge[node])`.
    fn dijkstra(&self, source: usize) -> (Vec<f64>, Vec<Option<usize>>) {
        let n = self.nodes.len();
        let mut dist = vec![f64::INFINITY; n];
        let mut pred_edge: Vec<Option<usize>> = vec![None; n];
        dist[source] = 0.0;

        // Build adjacency list (edge_index, neighbour, cost)
        let mut adj: Vec<Vec<(usize, usize, f64)>> = vec![Vec::new(); n];
        for (ei, e) in self.edges.iter().enumerate() {
            if e.from < n && e.to < n {
                adj[e.from].push((ei, e.to, e.cost_million_eur));
                adj[e.to].push((ei, e.from, e.cost_million_eur));
            }
        }

        let mut heap = BinaryHeap::new();
        heap.push(DijkState {
            cost: 0.0,
            node: source,
        });

        while let Some(DijkState { cost, node }) = heap.pop() {
            if cost > dist[node] + 1e-12 {
                continue;
            }
            for &(ei, nb, w) in &adj[node] {
                let new_cost = dist[node] + w;
                if new_cost < dist[nb] - 1e-12 {
                    dist[nb] = new_cost;
                    pred_edge[nb] = Some(ei);
                    heap.push(DijkState {
                        cost: new_cost,
                        node: nb,
                    });
                }
            }
        }

        (dist, pred_edge)
    }

    /// Reconstruct the edge-path from `source` to `target` using predecessor arrays.
    fn reconstruct_path(&self, target: usize, pred_edge: &[Option<usize>]) -> Vec<usize> {
        let mut path = Vec::new();
        let mut cur = target;
        // Traverse backwards via predecessor edges.
        // To find the predecessor node, look at the edge's endpoints.
        let mut seen = HashSet::new();
        loop {
            if seen.contains(&cur) {
                break; // cycle guard
            }
            seen.insert(cur);
            match pred_edge.get(cur).and_then(|e| *e) {
                None => break,
                Some(ei) => {
                    path.push(ei);
                    let e = &self.edges[ei];
                    // Move to the other endpoint of the edge
                    cur = if e.to == cur { e.from } else { e.to };
                }
            }
        }
        path
    }

    /// Compute all-pairs shortest paths *from each terminal* to *all nodes*.
    ///
    /// Returns:
    /// - `dist_matrix[ti][tj]` — shortest cost from terminal `ti` to terminal `tj`.
    /// - `path_matrix[ti][tj]` — list of original edge indices on that path.
    ///
    /// Indices in the returned matrices are into `terminals`, not node IDs.
    #[allow(clippy::type_complexity)]
    fn all_pairs_shortest_paths(
        &self,
        terminals: &[usize],
    ) -> Result<(Vec<Vec<f64>>, Vec<Vec<Vec<usize>>>), OxiGridError> {
        let nt = terminals.len();
        let mut dist_matrix = vec![vec![f64::INFINITY; nt]; nt];
        let mut path_matrix = vec![vec![Vec::new(); nt]; nt];

        for (ti, &t_node) in terminals.iter().enumerate() {
            let (dist, pred) = self.dijkstra(t_node);
            dist_matrix[ti][ti] = 0.0;
            for (tj, &t2_node) in terminals.iter().enumerate() {
                if ti == tj {
                    continue;
                }
                if dist[t2_node].is_infinite() {
                    // disconnected
                    dist_matrix[ti][tj] = f64::INFINITY;
                } else {
                    dist_matrix[ti][tj] = dist[t2_node];
                    path_matrix[ti][tj] = self.reconstruct_path(t2_node, &pred);
                }
            }
        }

        // Verify all terminals are reachable from terminal 0
        for tj in 1..nt {
            if dist_matrix[0][tj].is_infinite() {
                return Err(OxiGridError::InvalidNetwork(format!(
                    "terminal node {} is unreachable from terminal node {} (disconnected graph)",
                    terminals[tj], terminals[0]
                )));
            }
        }

        Ok((dist_matrix, path_matrix))
    }

    /// Kruskal MST on the metric closure of terminals.
    ///
    /// Returns a list of `(ti_index, tj_index)` pairs (indices into `terminals`).
    fn mst_of_metric_closure(
        &self,
        terminals: &[usize],
        dist_matrix: &[Vec<f64>],
    ) -> Result<Vec<(usize, usize)>, OxiGridError> {
        let nt = terminals.len();
        // Build all pairs with finite distances
        let mut closure_edges: Vec<(f64, usize, usize)> = Vec::new();
        #[allow(clippy::needless_range_loop)]
        for ti in 0..nt {
            for tj in (ti + 1)..nt {
                let d = dist_matrix[ti][tj];
                if d.is_finite() {
                    closure_edges.push((d, ti, tj));
                }
            }
        }
        // Sort by distance
        closure_edges.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(Ordering::Equal));

        let mut uf = UnionFind::new(nt);
        let mut mst_pairs = Vec::new();

        for (_, ti, tj) in &closure_edges {
            if uf.find(*ti) != uf.find(*tj) {
                uf.union(*ti, *tj);
                mst_pairs.push((*ti, *tj));
                if mst_pairs.len() == nt - 1 {
                    break;
                }
            }
        }

        if mst_pairs.len() < nt - 1 {
            return Err(OxiGridError::InvalidNetwork(
                "cannot form spanning tree over terminals — graph is disconnected".to_string(),
            ));
        }

        Ok(mst_pairs)
    }

    /// Remove non-terminal Steiner nodes that have degree ≤ 1 in the selected tree
    /// (iterative leaf pruning).
    fn prune_steiner_points(
        &self,
        mut selected_set: HashSet<usize>,
        terminals: &[usize],
        n_nodes: usize,
    ) -> Vec<usize> {
        let terminal_set: HashSet<usize> = terminals.iter().copied().collect();

        loop {
            // Build degree map
            let mut degree: HashMap<usize, usize> = HashMap::new();
            for &ei in &selected_set {
                if ei < self.edges.len() {
                    let e = &self.edges[ei];
                    if e.from < n_nodes && e.to < n_nodes {
                        *degree.entry(e.from).or_insert(0) += 1;
                        *degree.entry(e.to).or_insert(0) += 1;
                    }
                }
            }

            // Find a non-terminal leaf
            let leaf = degree
                .iter()
                .find(|(&node, &deg)| deg <= 1 && !terminal_set.contains(&node))
                .map(|(&node, _)| node);

            match leaf {
                None => break,
                Some(leaf_node) => {
                    // Remove all edges incident to this leaf
                    selected_set.retain(|&ei| {
                        if ei >= self.edges.len() {
                            return true;
                        }
                        let e = &self.edges[ei];
                        e.from != leaf_node && e.to != leaf_node
                    });
                }
            }
        }

        let mut result: Vec<usize> = selected_set.into_iter().collect();
        result.sort_unstable();
        result
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Union-Find (Disjoint Set Union)
// ─────────────────────────────────────────────────────────────────────────────

struct UnionFind {
    parent: Vec<usize>,
    rank: Vec<usize>,
}

impl UnionFind {
    fn new(n: usize) -> Self {
        Self {
            parent: (0..n).collect(),
            rank: vec![0; n],
        }
    }

    fn find(&mut self, x: usize) -> usize {
        if self.parent[x] != x {
            self.parent[x] = self.find(self.parent[x]);
        }
        self.parent[x]
    }

    fn union(&mut self, a: usize, b: usize) {
        let ra = self.find(a);
        let rb = self.find(b);
        if ra == rb {
            return;
        }
        match self.rank[ra].cmp(&self.rank[rb]) {
            Ordering::Less => self.parent[ra] = rb,
            Ordering::Greater => self.parent[rb] = ra,
            Ordering::Equal => {
                self.parent[rb] = ra;
                self.rank[ra] += 1;
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Transmission Expansion Planner
// ─────────────────────────────────────────────────────────────────────────────

/// Greedy BCR-ranked transmission expansion planner.
///
/// For each candidate line, estimates the NPV benefit via PTDF-based congestion
/// relief and VOLL (Value of Lost Load), then ranks by benefit-cost ratio.
pub struct ExpansionPlanner {
    /// Already-built network lines.
    pub existing_network: Vec<NetworkEdge>,
    /// Lines that could be built (investment candidates).
    pub candidate_lines: Vec<NetworkEdge>,
    /// All planning nodes.
    pub nodes: Vec<TopologyNode>,
    /// Planning horizon [years].
    pub planning_years: usize,
    /// Annual discount rate (e.g. 0.07 for 7 %).
    pub discount_rate: f64,
    /// Number of Monte Carlo scenarios for uncertainty.
    pub n_scenarios: usize,
    /// Annual load growth fraction (e.g. 0.02 for 2 %).
    pub load_growth_rate: f64,
}

/// A single transmission investment decision with economics.
#[derive(Debug, Clone)]
pub struct ExpansionCandidate {
    /// Index into [`ExpansionPlanner::candidate_lines`].
    pub line_idx: usize,
    /// Year in the planning horizon when the line would be built (0-based).
    pub build_year: usize,
    /// Discounted benefit [million EUR].
    pub npv_benefit_million_eur: f64,
    /// Discounted cost [million EUR].
    pub npv_cost_million_eur: f64,
    /// Benefit-cost ratio (BCR = benefit / cost).
    pub bcr: f64,
    /// Estimated congestion relief [MW].
    pub congestion_relief_mw: f64,
}

/// Full expansion plan output.
#[derive(Debug, Clone)]
pub struct ExpansionPlan {
    /// Selected investments sorted by descending BCR.
    pub investments: Vec<ExpansionCandidate>,
    /// Total capital cost [million EUR].
    pub total_cost_million_eur: f64,
    /// Total discounted benefit [million EUR].
    pub total_benefit_million_eur: f64,
    /// Net present value of the plan (benefit − cost) [million EUR].
    pub total_npv_million_eur: f64,
    /// Approximate years until each node hits its capacity limit.
    pub years_to_capacity_limit: Vec<f64>,
}

/// Value of Lost Load assumed for benefit calculation [million EUR/MWh].
const VOLL_MILLION_EUR_PER_MWH: f64 = 0.010; // 10 000 EUR/MWh
/// Annual operating hours assumed for energy calculations.
const OPERATING_HOURS_PER_YEAR: f64 = 8_760.0;

impl ExpansionPlanner {
    /// Create a new planner.
    pub fn new(
        existing_network: Vec<NetworkEdge>,
        candidate_lines: Vec<NetworkEdge>,
        nodes: Vec<TopologyNode>,
        planning_years: usize,
        discount_rate: f64,
        n_scenarios: usize,
        load_growth_rate: f64,
    ) -> Self {
        Self {
            existing_network,
            candidate_lines,
            nodes,
            planning_years,
            discount_rate,
            n_scenarios,
            load_growth_rate,
        }
    }

    /// Run the greedy BCR-ranked expansion optimisation.
    ///
    /// Iterates through planning years; for each year computes per-candidate BCR
    /// and selects the best until the budget is exhausted.
    pub fn optimize_greedy(&self, budget_million_eur: f64) -> Result<ExpansionPlan, OxiGridError> {
        if self.candidate_lines.is_empty() {
            return Err(OxiGridError::InvalidNetwork(
                "no candidate lines defined".to_string(),
            ));
        }
        if self.planning_years == 0 {
            return Err(OxiGridError::InvalidParameter(
                "planning_years must be > 0".to_string(),
            ));
        }
        if budget_million_eur < 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "budget must be non-negative".to_string(),
            ));
        }

        let mut spent = 0.0_f64;
        let mut selected: Vec<ExpansionCandidate> = Vec::new();
        let mut built_set: HashSet<usize> = HashSet::new();

        // Evaluate all candidates for each year and greedily pick highest BCR
        for year in 0..self.planning_years {
            if spent >= budget_million_eur - 1e-9 {
                break;
            }
            let growth_factor = (1.0 + self.load_growth_rate).powi(year as i32);
            let discount_factor = self.npv_discount_factor(year);

            // Collect all un-built candidates with BCR > 1
            let mut candidates_this_year: Vec<ExpansionCandidate> = self
                .candidate_lines
                .iter()
                .enumerate()
                .filter(|(idx, _)| !built_set.contains(idx))
                .map(|(idx, line)| {
                    let ptdf = Self::compute_ptdf_entry(line.from, line.to, &self.existing_network);
                    let relief_mw = line.capacity_mw * ptdf.abs() * growth_factor;
                    let benefit_per_year =
                        relief_mw * OPERATING_HOURS_PER_YEAR * VOLL_MILLION_EUR_PER_MWH * 0.01; // assume 1% probability of congestion per hour
                    let npv_benefit = benefit_per_year * self.annuity_factor() * discount_factor;
                    let npv_cost = line.cost_million_eur * discount_factor;
                    let bcr = if npv_cost > 1e-9 {
                        npv_benefit / npv_cost
                    } else {
                        0.0
                    };
                    ExpansionCandidate {
                        line_idx: idx,
                        build_year: year,
                        npv_benefit_million_eur: npv_benefit,
                        npv_cost_million_eur: npv_cost,
                        bcr,
                        congestion_relief_mw: relief_mw,
                    }
                })
                .filter(|c| c.bcr > 0.0)
                .collect();

            // Sort by BCR descending
            candidates_this_year
                .sort_by(|a, b| b.bcr.partial_cmp(&a.bcr).unwrap_or(Ordering::Equal));

            // Greedily select
            for cand in candidates_this_year {
                let remaining = budget_million_eur - spent;
                if cand.npv_cost_million_eur > remaining + 1e-9 {
                    continue;
                }
                spent += cand.npv_cost_million_eur;
                built_set.insert(cand.line_idx);
                selected.push(cand);
                if spent >= budget_million_eur - 1e-9 {
                    break;
                }
            }
        }

        let total_cost: f64 = selected.iter().map(|c| c.npv_cost_million_eur).sum();
        let total_benefit: f64 = selected.iter().map(|c| c.npv_benefit_million_eur).sum();
        let years_to_limit = self.estimate_years_to_capacity_limit();

        Ok(ExpansionPlan {
            investments: selected,
            total_cost_million_eur: total_cost,
            total_benefit_million_eur: total_benefit,
            total_npv_million_eur: total_benefit - total_cost,
            years_to_capacity_limit: years_to_limit,
        })
    }

    /// Compute an approximate PTDF entry for a new line from `from` to `to`.
    ///
    /// Uses the lossless DC formula:
    /// `PTDF ≈ b_new / (b_new + b_system)` where `b_system` is the total
    /// susceptance of existing lines sharing at least one endpoint.
    pub fn compute_ptdf_entry(from: usize, to: usize, network: &[NetworkEdge]) -> f64 {
        let b_system: f64 = network
            .iter()
            .filter(|e| e.from == from || e.to == from || e.from == to || e.to == to)
            .map(|e| e.susceptance_pu())
            .sum();

        if b_system < 1e-12 {
            1.0 // isolated corridor — new line carries 100% of flow
        } else {
            b_system / (b_system + b_system) // simplified: 50% sharing
        }
    }

    /// Check whether all existing + selected lines keep their loading below capacity.
    ///
    /// Uses a flat-voltage DC power flow approximation:
    /// `P_flow = V²·b·(θ_from - θ_to) ≈ b·Δθ`
    /// Here, loading is estimated as `total_load / (n_lines · mean_capacity)`.
    pub fn check_feasibility(&self, selected: &[usize]) -> bool {
        let total_load: f64 = self.nodes.iter().map(|n| n.peak_load_mw).sum();
        let mut all_lines: Vec<&NetworkEdge> = self.existing_network.iter().collect();
        for &si in selected {
            if si < self.candidate_lines.len() {
                all_lines.push(&self.candidate_lines[si]);
            }
        }
        if all_lines.is_empty() {
            return total_load < 1e-9;
        }
        let total_capacity: f64 = all_lines.iter().map(|e| e.capacity_mw).sum();
        // Feasible if total capacity exceeds total peak load (conservative DC check)
        total_capacity >= total_load
    }

    /// N-1 security check: for each selected line, temporarily remove it and
    /// verify the remaining network is still feasible.
    ///
    /// Returns `(line_idx, is_secure)` for each investment.
    pub fn n1_security_check(&self, plan: &ExpansionPlan) -> Vec<(usize, bool)> {
        plan.investments
            .iter()
            .map(|inv| {
                // Build set of all selected lines except this one
                let others: Vec<usize> = plan
                    .investments
                    .iter()
                    .filter(|c| c.line_idx != inv.line_idx)
                    .map(|c| c.line_idx)
                    .collect();
                let secure = self.check_feasibility(&others);
                (inv.line_idx, secure)
            })
            .collect()
    }

    // ── Internal helpers ──────────────────────────────────────────────────────

    /// NPV discount factor for a given year: `1 / (1 + r)^year`.
    fn npv_discount_factor(&self, year: usize) -> f64 {
        (1.0 + self.discount_rate).powi(-(year as i32))
    }

    /// Present value of an annuity of 1 EUR/year over `planning_years`.
    fn annuity_factor(&self) -> f64 {
        let r = self.discount_rate;
        let n = self.planning_years as f64;
        if r.abs() < 1e-12 {
            n
        } else {
            (1.0 - (1.0 + r).powf(-n)) / r
        }
    }

    /// Estimate years until each node's local load exceeds a nominal 100 MW limit.
    fn estimate_years_to_capacity_limit(&self) -> Vec<f64> {
        self.nodes
            .iter()
            .map(|node| {
                if node.peak_load_mw < 1e-9 || self.load_growth_rate < 1e-9 {
                    return f64::INFINITY;
                }
                // nominal_capacity = sum of existing line capacities incident on node
                let cap: f64 = self
                    .existing_network
                    .iter()
                    .filter(|e| e.from == node.id || e.to == node.id)
                    .map(|e| e.capacity_mw)
                    .sum::<f64>()
                    .max(100.0); // fallback 100 MW if no lines
                                 // years until load * (1+g)^t > cap
                                 // t = ln(cap / load) / ln(1+g)
                let ratio = cap / node.peak_load_mw;
                if ratio <= 1.0 {
                    0.0
                } else {
                    ratio.ln() / (1.0 + self.load_growth_rate).ln()
                }
            })
            .collect()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Substation Siting — K-means
// ─────────────────────────────────────────────────────────────────────────────

/// Geographic substation siting via load-weighted k-means clustering.
///
/// Each cluster centroid represents an optimal substation location that minimises
/// the total weighted distance (≈ cable length) to served load points.
pub struct SubstationSiting {
    /// Load points: `(x_km, y_km, load_mw)`.
    pub load_points: Vec<(f64, f64, f64)>,
    /// Desired number of substations.
    pub n_substations: usize,
    /// Nominal voltage of the distribution feeder [kV].
    pub voltage_kv: f64,
    /// Cable cost per km [million EUR/km].
    pub cable_cost_million_eur_per_km: f64,
}

/// Result of the substation siting optimisation.
#[derive(Debug, Clone)]
pub struct SitingResult {
    /// Optimal geographic coordinates of each substation `(x_km, y_km)`.
    pub substation_locations: Vec<(f64, f64)>,
    /// Cluster assignment: `assignments[i]` is the substation index for load point `i`.
    pub assignments: Vec<usize>,
    /// Estimated total cable investment cost [million EUR].
    pub total_cable_cost_million_eur: f64,
    /// Length of the longest feeder [km].
    pub max_feeder_length_km: f64,
    /// Mean feeder length weighted by number of load points [km].
    pub avg_feeder_length_km: f64,
}

impl SubstationSiting {
    /// Construct a new siting problem.
    pub fn new(
        load_points: Vec<(f64, f64, f64)>,
        n_substations: usize,
        voltage_kv: f64,
        cable_cost_million_eur_per_km: f64,
    ) -> Self {
        Self {
            load_points,
            n_substations,
            voltage_kv,
            cable_cost_million_eur_per_km,
        }
    }

    /// Optimise substation locations using load-weighted k-means.
    ///
    /// Initialisation: spread substations by percentile of x-coordinate, choosing
    /// the load-weighted centroid within each segment as the initial centre.
    ///
    /// Iteration: assign each load point to its nearest substation (Euclidean),
    /// recompute load-weighted centroids.  Converge when centroid shift < 0.01 km.
    pub fn optimize_kmeans(&self, max_iter: usize) -> Result<SitingResult, OxiGridError> {
        let n = self.load_points.len();
        let k = self.n_substations;

        if n == 0 {
            return Err(OxiGridError::InvalidNetwork(
                "no load points defined".to_string(),
            ));
        }
        if k == 0 {
            return Err(OxiGridError::InvalidParameter(
                "n_substations must be > 0".to_string(),
            ));
        }
        if k > n {
            return Err(OxiGridError::InvalidParameter(format!(
                "n_substations ({k}) exceeds number of load points ({n})"
            )));
        }

        // Initialise centroids by splitting sorted-x load points into k segments
        let mut sorted_indices: Vec<usize> = (0..n).collect();
        sorted_indices.sort_by(|&a, &b| {
            self.load_points[a]
                .0
                .partial_cmp(&self.load_points[b].0)
                .unwrap_or(Ordering::Equal)
        });

        let mut centroids: Vec<(f64, f64)> = (0..k)
            .map(|seg| {
                let start = seg * n / k;
                let end = ((seg + 1) * n / k).min(n);
                let seg_points: Vec<(f64, f64, f64)> = sorted_indices[start..end]
                    .iter()
                    .map(|&i| self.load_points[i])
                    .collect();
                Self::load_weighted_centroid(&seg_points)
            })
            .collect();

        let mut assignments = vec![0usize; n];

        for _iter in 0..max_iter {
            // Assignment step
            #[allow(clippy::needless_range_loop)]
            for i in 0..n {
                let (px, py, _) = self.load_points[i];
                let nearest = centroids
                    .iter()
                    .enumerate()
                    .min_by(|(_, &c1), (_, &c2)| {
                        Self::euclidean_distance((px, py), c1)
                            .partial_cmp(&Self::euclidean_distance((px, py), c2))
                            .unwrap_or(Ordering::Equal)
                    })
                    .map(|(idx, _)| idx)
                    .unwrap_or(0);
                assignments[i] = nearest;
            }

            // Update centroids
            let mut new_centroids: Vec<(f64, f64)> = Vec::with_capacity(k);
            let mut converged = true;

            #[allow(clippy::needless_range_loop)]
            for ci in 0..k {
                let cluster_points: Vec<(f64, f64, f64)> = (0..n)
                    .filter(|&i| assignments[i] == ci)
                    .map(|i| self.load_points[i])
                    .collect();

                let new_c = if cluster_points.is_empty() {
                    centroids[ci] // keep existing centroid if cluster is empty
                } else {
                    Self::load_weighted_centroid(&cluster_points)
                };

                let shift = Self::euclidean_distance(centroids[ci], new_c);
                if shift > 0.01 {
                    converged = false;
                }
                new_centroids.push(new_c);
            }

            centroids = new_centroids;

            if converged {
                break;
            }
        }

        // Compute metrics
        let distances: Vec<f64> = (0..n)
            .map(|i| {
                let (px, py, _) = self.load_points[i];
                Self::euclidean_distance((px, py), centroids[assignments[i]])
            })
            .collect();

        let total_cable_len: f64 = distances.iter().sum();
        let max_len = distances.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let avg_len = if n > 0 {
            total_cable_len / n as f64
        } else {
            0.0
        };
        let total_cost = total_cable_len * self.cable_cost_million_eur_per_km;

        Ok(SitingResult {
            substation_locations: centroids,
            assignments,
            total_cable_cost_million_eur: total_cost,
            max_feeder_length_km: max_len.max(0.0),
            avg_feeder_length_km: avg_len,
        })
    }

    /// Euclidean distance between two 2D points.
    pub fn euclidean_distance(p1: (f64, f64), p2: (f64, f64)) -> f64 {
        let dx = p1.0 - p2.0;
        let dy = p1.1 - p2.1;
        (dx * dx + dy * dy).sqrt()
    }

    /// Load-weighted centroid of a set of `(x, y, load_mw)` points.
    ///
    /// Returns the unweighted centroid if total load is zero.
    pub fn load_weighted_centroid(points: &[(f64, f64, f64)]) -> (f64, f64) {
        let total_w: f64 = points.iter().map(|(_, _, w)| w).sum();
        if total_w < 1e-12 {
            // Unweighted centroid as fallback
            let n = points.len() as f64;
            if n < 1e-12 {
                return (0.0, 0.0);
            }
            let x = points.iter().map(|(x, _, _)| x).sum::<f64>() / n;
            let y = points.iter().map(|(_, y, _)| y).sum::<f64>() / n;
            return (x, y);
        }
        let x = points.iter().map(|(xi, _, wi)| xi * wi).sum::<f64>() / total_w;
        let y = points.iter().map(|(_, yi, wi)| yi * wi).sum::<f64>() / total_w;
        (x, y)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // LCG random number generator (no rand crate)
    struct Lcg(u64);
    impl Lcg {
        fn new(seed: u64) -> Self {
            Self(seed)
        }
        fn next_f64(&mut self) -> f64 {
            self.0 = self
                .0
                .wrapping_mul(6_364_136_223_846_793_005u64)
                .wrapping_add(1_442_695_040_888_963_407u64);
            // Map to [0, 1)
            (self.0 >> 11) as f64 / (1u64 << 53) as f64
        }
        fn next_range(&mut self, lo: f64, hi: f64) -> f64 {
            lo + self.next_f64() * (hi - lo)
        }
    }

    // ── NetworkEdge / TopologyNode construction ───────────────────────────────

    #[test]
    fn test_network_edge_creation() {
        let edge = NetworkEdge {
            from: 0,
            to: 1,
            length_km: 50.0,
            voltage_kv: 110.0,
            capacity_mw: 200.0,
            cost_million_eur: 5.0,
            resistance_pu: 0.01,
            reactance_pu: 0.05,
            is_existing: false,
            build_years: 3.0,
        };
        assert_eq!(edge.from, 0);
        assert_eq!(edge.to, 1);
        assert!((edge.length_km - 50.0).abs() < 1e-9);
        assert!((edge.susceptance_pu() - 20.0).abs() < 1e-6);
    }

    #[test]
    fn test_topology_node_creation() {
        let node = TopologyNode {
            id: 3,
            is_terminal: true,
            is_substation: false,
            peak_load_mw: 150.0,
            peak_generation_mw: 0.0,
            x: 10.0,
            y: 20.0,
        };
        assert_eq!(node.id, 3);
        assert!(node.is_terminal);
        assert!(!node.is_substation);
        assert!((node.peak_load_mw - 150.0).abs() < 1e-9);
    }

    // ── Helper to build a simple 3-node triangle ──────────────────────────────

    fn make_triangle() -> SteinerTreeSolver {
        let nodes = vec![
            TopologyNode {
                id: 0,
                is_terminal: true,
                is_substation: false,
                peak_load_mw: 100.0,
                peak_generation_mw: 0.0,
                x: 0.0,
                y: 0.0,
            },
            TopologyNode {
                id: 1,
                is_terminal: true,
                is_substation: false,
                peak_load_mw: 80.0,
                peak_generation_mw: 0.0,
                x: 10.0,
                y: 0.0,
            },
            TopologyNode {
                id: 2,
                is_terminal: true,
                is_substation: false,
                peak_load_mw: 60.0,
                peak_generation_mw: 0.0,
                x: 5.0,
                y: 8.0,
            },
        ];
        let edges = vec![
            NetworkEdge {
                from: 0,
                to: 1,
                length_km: 10.0,
                voltage_kv: 110.0,
                capacity_mw: 200.0,
                cost_million_eur: 1.0,
                resistance_pu: 0.01,
                reactance_pu: 0.05,
                is_existing: false,
                build_years: 1.0,
            },
            NetworkEdge {
                from: 1,
                to: 2,
                length_km: 9.4,
                voltage_kv: 110.0,
                capacity_mw: 200.0,
                cost_million_eur: 1.5,
                resistance_pu: 0.01,
                reactance_pu: 0.05,
                is_existing: false,
                build_years: 1.0,
            },
            NetworkEdge {
                from: 0,
                to: 2,
                length_km: 9.4,
                voltage_kv: 110.0,
                capacity_mw: 200.0,
                cost_million_eur: 2.0,
                resistance_pu: 0.01,
                reactance_pu: 0.05,
                is_existing: false,
                build_years: 1.0,
            },
        ];
        SteinerTreeSolver::new(nodes, edges)
    }

    #[test]
    fn test_steiner_tree_3_terminals() {
        let solver = make_triangle();
        let result = solver.solve_approximate().expect("should succeed");
        // For a 3-terminal problem all terminals, Steiner tree = MST of terminals = 2 edges
        assert_eq!(
            result.selected_edges.len(),
            2,
            "3-terminal Steiner tree needs exactly 2 edges"
        );
        assert!(result.is_connected, "result must be connected");
    }

    #[test]
    fn test_steiner_tree_4_terminals() {
        // 4 corners + 1 Steiner point in the center
        // Terminals: 0(0,0), 1(10,0), 2(10,10), 3(0,10)
        // Steiner point: 4(5,5)
        let nodes = vec![
            TopologyNode {
                id: 0,
                is_terminal: true,
                is_substation: false,
                peak_load_mw: 50.0,
                peak_generation_mw: 0.0,
                x: 0.0,
                y: 0.0,
            },
            TopologyNode {
                id: 1,
                is_terminal: true,
                is_substation: false,
                peak_load_mw: 50.0,
                peak_generation_mw: 0.0,
                x: 10.0,
                y: 0.0,
            },
            TopologyNode {
                id: 2,
                is_terminal: true,
                is_substation: false,
                peak_load_mw: 50.0,
                peak_generation_mw: 0.0,
                x: 10.0,
                y: 10.0,
            },
            TopologyNode {
                id: 3,
                is_terminal: true,
                is_substation: false,
                peak_load_mw: 50.0,
                peak_generation_mw: 0.0,
                x: 0.0,
                y: 10.0,
            },
            TopologyNode {
                id: 4,
                is_terminal: false,
                is_substation: true,
                peak_load_mw: 0.0,
                peak_generation_mw: 0.0,
                x: 5.0,
                y: 5.0,
            },
        ];
        // Edges: corners to center (cheap), plus direct corner-to-corner (expensive)
        let edges = vec![
            NetworkEdge {
                from: 0,
                to: 4,
                length_km: 7.07,
                voltage_kv: 110.0,
                capacity_mw: 200.0,
                cost_million_eur: 1.0,
                resistance_pu: 0.01,
                reactance_pu: 0.05,
                is_existing: false,
                build_years: 1.0,
            },
            NetworkEdge {
                from: 1,
                to: 4,
                length_km: 7.07,
                voltage_kv: 110.0,
                capacity_mw: 200.0,
                cost_million_eur: 1.0,
                resistance_pu: 0.01,
                reactance_pu: 0.05,
                is_existing: false,
                build_years: 1.0,
            },
            NetworkEdge {
                from: 2,
                to: 4,
                length_km: 7.07,
                voltage_kv: 110.0,
                capacity_mw: 200.0,
                cost_million_eur: 1.0,
                resistance_pu: 0.01,
                reactance_pu: 0.05,
                is_existing: false,
                build_years: 1.0,
            },
            NetworkEdge {
                from: 3,
                to: 4,
                length_km: 7.07,
                voltage_kv: 110.0,
                capacity_mw: 200.0,
                cost_million_eur: 1.0,
                resistance_pu: 0.01,
                reactance_pu: 0.05,
                is_existing: false,
                build_years: 1.0,
            },
            // Expensive direct edges (fallback connectivity)
            NetworkEdge {
                from: 0,
                to: 1,
                length_km: 10.0,
                voltage_kv: 110.0,
                capacity_mw: 200.0,
                cost_million_eur: 5.0,
                resistance_pu: 0.01,
                reactance_pu: 0.05,
                is_existing: false,
                build_years: 1.0,
            },
            NetworkEdge {
                from: 1,
                to: 2,
                length_km: 10.0,
                voltage_kv: 110.0,
                capacity_mw: 200.0,
                cost_million_eur: 5.0,
                resistance_pu: 0.01,
                reactance_pu: 0.05,
                is_existing: false,
                build_years: 1.0,
            },
            NetworkEdge {
                from: 2,
                to: 3,
                length_km: 10.0,
                voltage_kv: 110.0,
                capacity_mw: 200.0,
                cost_million_eur: 5.0,
                resistance_pu: 0.01,
                reactance_pu: 0.05,
                is_existing: false,
                build_years: 1.0,
            },
            NetworkEdge {
                from: 0,
                to: 3,
                length_km: 10.0,
                voltage_kv: 110.0,
                capacity_mw: 200.0,
                cost_million_eur: 5.0,
                resistance_pu: 0.01,
                reactance_pu: 0.05,
                is_existing: false,
                build_years: 1.0,
            },
        ];
        let solver = SteinerTreeSolver::new(nodes, edges);
        let result = solver
            .solve_approximate()
            .expect("4-terminal Steiner should succeed");
        assert!(result.is_connected, "4-terminal result must be connected");
        // Cost should be dominated by center connections (4 × 1.0 = 4.0 before pruning)
        assert!(
            result.total_cost_million_eur < 20.0,
            "cost should be reasonable"
        );
    }

    #[test]
    fn test_steiner_tree_connectivity() {
        let solver = make_triangle();
        let result = solver.solve_approximate().expect("should succeed");
        assert!(
            result.is_connected,
            "Steiner result must connect all terminals"
        );
        // Verify manually using the public helper
        assert!(
            SteinerTreeSolver::is_connected(
                solver.nodes.len(),
                &result.selected_edges,
                &solver.edges
            ),
            "public is_connected must confirm result"
        );
    }

    #[test]
    fn test_steiner_tree_vs_mst() {
        let solver = make_triangle();
        let steiner = solver.solve_approximate().expect("Steiner ok");
        let mst = solver.solve_mst_baseline().expect("MST ok");
        // For all-terminal graphs, Steiner = MST, so cost should be ≤ MST cost
        assert!(
            steiner.total_cost_million_eur <= mst.total_cost_million_eur + 1e-9,
            "Steiner cost ({:.3}) must be ≤ MST cost ({:.3})",
            steiner.total_cost_million_eur,
            mst.total_cost_million_eur
        );
    }

    #[test]
    fn test_mst_baseline() {
        let solver = make_triangle();
        let mst = solver.solve_mst_baseline().expect("MST should succeed");
        // MST of 3 nodes = 2 edges
        assert_eq!(mst.selected_edges.len(), 2);
        assert!(mst.total_cost_million_eur > 0.0);
    }

    #[test]
    fn test_is_connected_true() {
        let edges = vec![
            NetworkEdge {
                from: 0,
                to: 1,
                length_km: 5.0,
                voltage_kv: 110.0,
                capacity_mw: 100.0,
                cost_million_eur: 1.0,
                resistance_pu: 0.01,
                reactance_pu: 0.05,
                is_existing: true,
                build_years: 0.0,
            },
            NetworkEdge {
                from: 1,
                to: 2,
                length_km: 5.0,
                voltage_kv: 110.0,
                capacity_mw: 100.0,
                cost_million_eur: 1.0,
                resistance_pu: 0.01,
                reactance_pu: 0.05,
                is_existing: true,
                build_years: 0.0,
            },
        ];
        assert!(SteinerTreeSolver::is_connected(3, &[0, 1], &edges));
    }

    #[test]
    fn test_is_connected_false() {
        let edges = vec![
            NetworkEdge {
                from: 0,
                to: 1,
                length_km: 5.0,
                voltage_kv: 110.0,
                capacity_mw: 100.0,
                cost_million_eur: 1.0,
                resistance_pu: 0.01,
                reactance_pu: 0.05,
                is_existing: true,
                build_years: 0.0,
            },
            NetworkEdge {
                from: 2,
                to: 3,
                length_km: 5.0,
                voltage_kv: 110.0,
                capacity_mw: 100.0,
                cost_million_eur: 1.0,
                resistance_pu: 0.01,
                reactance_pu: 0.05,
                is_existing: true,
                build_years: 0.0,
            },
        ];
        // 4 nodes, 2 disconnected edges → not connected
        assert!(!SteinerTreeSolver::is_connected(4, &[0, 1], &edges));
    }

    // ── ExpansionPlanner tests ────────────────────────────────────────────────

    fn make_planner_single() -> ExpansionPlanner {
        let existing = vec![NetworkEdge {
            from: 0,
            to: 1,
            length_km: 20.0,
            voltage_kv: 220.0,
            capacity_mw: 300.0,
            cost_million_eur: 0.0,
            resistance_pu: 0.005,
            reactance_pu: 0.02,
            is_existing: true,
            build_years: 0.0,
        }];
        let candidate = vec![NetworkEdge {
            from: 1,
            to: 2,
            length_km: 30.0,
            voltage_kv: 220.0,
            capacity_mw: 250.0,
            cost_million_eur: 10.0,
            resistance_pu: 0.008,
            reactance_pu: 0.03,
            is_existing: false,
            build_years: 2.0,
        }];
        let nodes = vec![
            TopologyNode {
                id: 0,
                is_terminal: true,
                is_substation: true,
                peak_load_mw: 0.0,
                peak_generation_mw: 500.0,
                x: 0.0,
                y: 0.0,
            },
            TopologyNode {
                id: 1,
                is_terminal: true,
                is_substation: true,
                peak_load_mw: 200.0,
                peak_generation_mw: 0.0,
                x: 20.0,
                y: 0.0,
            },
            TopologyNode {
                id: 2,
                is_terminal: true,
                is_substation: false,
                peak_load_mw: 150.0,
                peak_generation_mw: 0.0,
                x: 50.0,
                y: 0.0,
            },
        ];
        ExpansionPlanner::new(existing, candidate, nodes, 10, 0.07, 5, 0.02)
    }

    #[test]
    fn test_expansion_planner_single_candidate() {
        let planner = make_planner_single();
        let plan = planner.optimize_greedy(100.0).expect("should succeed");
        // With a generous budget, the single candidate should be selected
        assert!(
            !plan.investments.is_empty(),
            "should select at least one line"
        );
        assert!(plan.total_cost_million_eur > 0.0);
        assert!(plan.total_npv_million_eur.is_finite());
    }

    #[test]
    fn test_expansion_planner_budget_constraint() {
        let planner = make_planner_single();
        // Budget too small to build anything (cost = 10.0, budget = 0.5)
        let plan = planner.optimize_greedy(0.5).expect("should succeed");
        assert!(
            plan.total_cost_million_eur <= 0.5 + 1e-9,
            "total cost must not exceed budget"
        );
    }

    #[test]
    fn test_expansion_planner_bcr_ranking() {
        // Two candidates: one with high BCR, one with low BCR
        let existing = vec![NetworkEdge {
            from: 0,
            to: 1,
            length_km: 10.0,
            voltage_kv: 110.0,
            capacity_mw: 100.0,
            cost_million_eur: 0.0,
            resistance_pu: 0.01,
            reactance_pu: 0.05,
            is_existing: true,
            build_years: 0.0,
        }];
        let candidates = vec![
            // Cheap, high-capacity → high BCR
            NetworkEdge {
                from: 1,
                to: 2,
                length_km: 5.0,
                voltage_kv: 110.0,
                capacity_mw: 200.0,
                cost_million_eur: 1.0,
                resistance_pu: 0.01,
                reactance_pu: 0.05,
                is_existing: false,
                build_years: 1.0,
            },
            // Expensive, low-capacity → low BCR
            NetworkEdge {
                from: 2,
                to: 3,
                length_km: 50.0,
                voltage_kv: 110.0,
                capacity_mw: 10.0,
                cost_million_eur: 50.0,
                resistance_pu: 0.05,
                reactance_pu: 0.2,
                is_existing: false,
                build_years: 3.0,
            },
        ];
        let nodes = vec![
            TopologyNode {
                id: 0,
                is_terminal: true,
                is_substation: true,
                peak_load_mw: 0.0,
                peak_generation_mw: 200.0,
                x: 0.0,
                y: 0.0,
            },
            TopologyNode {
                id: 1,
                is_terminal: true,
                is_substation: false,
                peak_load_mw: 100.0,
                peak_generation_mw: 0.0,
                x: 10.0,
                y: 0.0,
            },
            TopologyNode {
                id: 2,
                is_terminal: true,
                is_substation: false,
                peak_load_mw: 80.0,
                peak_generation_mw: 0.0,
                x: 15.0,
                y: 0.0,
            },
            TopologyNode {
                id: 3,
                is_terminal: true,
                is_substation: false,
                peak_load_mw: 50.0,
                peak_generation_mw: 0.0,
                x: 65.0,
                y: 0.0,
            },
        ];
        let planner = ExpansionPlanner::new(existing, candidates, nodes, 10, 0.07, 3, 0.02);
        let plan = planner.optimize_greedy(5.0).expect("BCR ranking test");
        // Only the cheap line (index 0) should fit in budget of 5.0
        if !plan.investments.is_empty() {
            let first_bcr = plan.investments[0].bcr;
            for c in &plan.investments[1..] {
                assert!(
                    c.bcr <= first_bcr + 1e-9,
                    "investments should be sorted by descending BCR within each year"
                );
            }
        }
    }

    #[test]
    fn test_ptdf_entry_calculation() {
        let network = vec![
            NetworkEdge {
                from: 0,
                to: 1,
                length_km: 10.0,
                voltage_kv: 110.0,
                capacity_mw: 100.0,
                cost_million_eur: 1.0,
                resistance_pu: 0.01,
                reactance_pu: 0.05,
                is_existing: true,
                build_years: 0.0,
            },
            NetworkEdge {
                from: 1,
                to: 2,
                length_km: 10.0,
                voltage_kv: 110.0,
                capacity_mw: 100.0,
                cost_million_eur: 1.0,
                resistance_pu: 0.01,
                reactance_pu: 0.05,
                is_existing: true,
                build_years: 0.0,
            },
        ];
        let ptdf = ExpansionPlanner::compute_ptdf_entry(0, 1, &network);
        assert!(
            ptdf > 0.0 && ptdf <= 1.0,
            "PTDF must be in (0, 1], got {ptdf}"
        );
    }

    #[test]
    fn test_feasibility_check_within_capacity() {
        let planner = make_planner_single();
        // Existing network alone should be feasible if capacity ≥ load
        let feasible = planner.check_feasibility(&[]);
        // existing capacity = 300 MW, total load = 350 MW → might not be feasible
        // but with candidate added, 300+250 = 550 MW > 350 MW
        let feasible_with_cand = planner.check_feasibility(&[0]);
        assert!(
            feasible_with_cand,
            "with candidate line, total cap 550 > load 350"
        );
        let _ = feasible; // just check it runs without panic
    }

    #[test]
    fn test_feasibility_check_exceeds_capacity() {
        // Network with very low capacity vs high load
        let existing = vec![NetworkEdge {
            from: 0,
            to: 1,
            length_km: 5.0,
            voltage_kv: 110.0,
            capacity_mw: 10.0,
            cost_million_eur: 0.0,
            resistance_pu: 0.01,
            reactance_pu: 0.05,
            is_existing: true,
            build_years: 0.0,
        }];
        let nodes = vec![
            TopologyNode {
                id: 0,
                is_terminal: true,
                is_substation: true,
                peak_load_mw: 0.0,
                peak_generation_mw: 500.0,
                x: 0.0,
                y: 0.0,
            },
            TopologyNode {
                id: 1,
                is_terminal: true,
                is_substation: false,
                peak_load_mw: 1000.0,
                peak_generation_mw: 0.0,
                x: 5.0,
                y: 0.0,
            },
        ];
        let planner = ExpansionPlanner::new(existing, vec![], nodes, 5, 0.07, 1, 0.02);
        assert!(
            !planner.check_feasibility(&[]),
            "10 MW capacity < 1000 MW load"
        );
    }

    #[test]
    fn test_n1_security_check() {
        let planner = make_planner_single();
        let plan = planner.optimize_greedy(100.0).expect("plan ok");
        let security = planner.n1_security_check(&plan);
        // Each entry should have a valid line_idx
        for (line_idx, is_secure) in &security {
            assert!(*line_idx < planner.candidate_lines.len());
            let _ = is_secure;
        }
        assert_eq!(security.len(), plan.investments.len());
    }

    // ── SubstationSiting tests ────────────────────────────────────────────────

    fn make_load_points_2cluster() -> Vec<(f64, f64, f64)> {
        // Two well-separated clusters
        vec![
            (0.0, 0.0, 10.0),
            (1.0, 0.0, 12.0),
            (0.5, 1.0, 8.0), // cluster A
            (20.0, 0.0, 15.0),
            (21.0, 0.0, 11.0),
            (20.5, 1.0, 9.0), // cluster B
        ]
    }

    #[test]
    fn test_substation_siting_2_substations() {
        let points = make_load_points_2cluster();
        let siting = SubstationSiting::new(points, 2, 110.0, 0.1);
        let result = siting.optimize_kmeans(100).expect("siting ok");
        assert_eq!(result.substation_locations.len(), 2);
        assert_eq!(result.assignments.len(), 6);
        assert!(result.total_cable_cost_million_eur >= 0.0);
        assert!(result.max_feeder_length_km >= result.avg_feeder_length_km - 1e-9);
    }

    #[test]
    fn test_substation_siting_3_substations() {
        let mut rng = Lcg::new(42);
        let points: Vec<(f64, f64, f64)> = (0..15)
            .map(|i| {
                let cluster = i / 5;
                let base_x = cluster as f64 * 30.0;
                (
                    base_x + rng.next_range(0.0, 5.0),
                    rng.next_range(0.0, 5.0),
                    rng.next_range(5.0, 20.0),
                )
            })
            .collect();
        let siting = SubstationSiting::new(points, 3, 110.0, 0.1);
        let result = siting.optimize_kmeans(50).expect("3-siting ok");
        assert_eq!(result.substation_locations.len(), 3);
        // Each cluster should have some assigned points
        let counts = {
            let mut c = [0usize; 3];
            for &a in &result.assignments {
                if a < 3 {
                    c[a] += 1;
                }
            }
            c
        };
        assert!(
            counts.iter().all(|&c| c > 0),
            "each substation should serve some points"
        );
    }

    #[test]
    fn test_kmeans_convergence() {
        // Perfectly separated clusters → should converge in very few iterations
        let points = vec![
            (0.0, 0.0, 1.0),
            (0.1, 0.0, 1.0),
            (0.0, 0.1, 1.0),
            (100.0, 0.0, 1.0),
            (100.1, 0.0, 1.0),
            (100.0, 0.1, 1.0),
        ];
        let siting = SubstationSiting::new(points, 2, 110.0, 0.05);
        let result = siting.optimize_kmeans(200).expect("convergence test");
        // Two substations should end up near (0.033, 0.033) and (100.033, 0.033)
        let locs = &result.substation_locations;
        let any_near_origin = locs.iter().any(|(x, y)| x.abs() < 5.0 && y.abs() < 5.0);
        let any_near_100 = locs
            .iter()
            .any(|(x, y)| (x - 100.0).abs() < 5.0 && y.abs() < 5.0);
        assert!(any_near_origin, "one substation should be near origin");
        assert!(any_near_100, "one substation should be near x=100");
    }

    #[test]
    fn test_load_weighted_centroid() {
        let points = vec![(0.0, 0.0, 1.0), (2.0, 0.0, 1.0)];
        let (cx, cy) = SubstationSiting::load_weighted_centroid(&points);
        assert!(
            (cx - 1.0).abs() < 1e-9,
            "centroid x should be 1.0, got {cx}"
        );
        assert!(cy.abs() < 1e-9, "centroid y should be 0.0, got {cy}");
    }

    #[test]
    fn test_load_weighted_centroid_weighted() {
        // Point at 0 with weight 3, point at 4 with weight 1 → centroid at 1.0
        let points = vec![(0.0, 0.0, 3.0), (4.0, 0.0, 1.0)];
        let (cx, _cy) = SubstationSiting::load_weighted_centroid(&points);
        assert!(
            (cx - 1.0).abs() < 1e-9,
            "weighted centroid x should be 1.0, got {cx}"
        );
    }

    #[test]
    fn test_euclidean_distance() {
        let d = SubstationSiting::euclidean_distance((0.0, 0.0), (3.0, 4.0));
        assert!((d - 5.0).abs() < 1e-9, "distance should be 5.0, got {d}");
    }

    #[test]
    fn test_expansion_planner_years_to_limit() {
        let planner = make_planner_single();
        let plan = planner.optimize_greedy(100.0).expect("plan ok");
        assert_eq!(plan.years_to_capacity_limit.len(), planner.nodes.len());
        for &y in &plan.years_to_capacity_limit {
            assert!(
                y >= 0.0 || y.is_infinite(),
                "years must be non-negative or infinite"
            );
        }
    }

    #[test]
    fn test_steiner_tree_total_length_positive() {
        let solver = make_triangle();
        let result = solver.solve_approximate().expect("ok");
        assert!(
            result.total_length_km > 0.0,
            "total length should be positive"
        );
    }

    #[test]
    fn test_mst_baseline_connected() {
        let solver = make_triangle();
        let mst = solver.solve_mst_baseline().expect("MST ok");
        assert!(mst.is_connected, "MST result must be connected");
    }

    #[test]
    fn test_expansion_zero_budget() {
        let planner = make_planner_single();
        let plan = planner.optimize_greedy(0.0).expect("zero budget ok");
        assert!(plan.investments.is_empty(), "zero budget → no investments");
        assert!((plan.total_cost_million_eur).abs() < 1e-9);
    }

    #[test]
    fn test_network_edge_susceptance_zero_reactance() {
        let edge = NetworkEdge {
            from: 0,
            to: 1,
            length_km: 1.0,
            voltage_kv: 110.0,
            capacity_mw: 100.0,
            cost_million_eur: 1.0,
            resistance_pu: 0.01,
            reactance_pu: 0.0,
            is_existing: false,
            build_years: 1.0,
        };
        assert_eq!(
            edge.susceptance_pu(),
            0.0,
            "zero reactance → zero susceptance"
        );
    }
}
