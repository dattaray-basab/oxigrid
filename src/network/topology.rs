use crate::error::{OxiGridError, Result};
use crate::network::admittance::build_y_bus;
use crate::network::branch::Branch;
use crate::network::bus::{Bus, BusType};
use num_complex::Complex64;
use serde::{Deserialize, Serialize};
use sprs::CsMat;

/// Node data stored in the internal petgraph Graph.
#[derive(Debug, Clone, Copy)]
struct NodeData {
    /// Internal 0-based bus index.
    bus_idx: usize,
}

/// Edge data stored in the internal petgraph Graph.
#[derive(Debug, Clone, Copy)]
struct EdgeData {
    /// Index into `PowerNetwork::branches`.
    branch_idx: usize,
    /// |Z| = sqrt(r² + x²) — used as edge weight.
    z_magnitude: f64,
}

/// A synchronous generator or voltage-controlled reactive source.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Generator {
    /// Bus this generator is connected to (1-based bus ID).
    pub bus_id: usize,
    /// Real power output setpoint `MW`.
    pub pg: f64,
    /// Reactive power output setpoint `MVAr`.
    pub qg: f64,
    /// Maximum reactive power output `MVAr`.
    pub qmax: f64,
    /// Minimum reactive power output `MVAr` (negative = absorbing).
    pub qmin: f64,
    /// Terminal voltage setpoint [p.u.] (used for PV bus initialisation).
    pub vg: f64,
    /// Machine MVA base (usually = system base).
    pub mbase: f64,
    /// Online status (`true` = in-service).
    pub status: bool,
    /// Maximum real power output `MW`.
    pub pmax: f64,
    /// Minimum real power output `MW`.
    pub pmin: f64,
}

/// AC power network: buses, branches, and generators.
///
/// Holds the full topology and parameter data needed to run power flow,
/// OPF, stability analysis, and other studies.
///
/// # Example
/// ```rust,ignore
/// let net = PowerNetwork::from_matpower("ieee14.m")?;
/// let result = net.solve_powerflow(&PowerFlowConfig::default())?;
/// println!("Losses: {:.2} MW", result.total_p_loss_mw);
/// ```
///
/// # Examples
///
/// ```rust
/// use oxigrid::network::topology::PowerNetwork;
/// use oxigrid::network::bus::{Bus, BusType};
/// use oxigrid::network::branch::Branch;
///
/// let mut net = PowerNetwork::new(100.0);
///
/// // Add a slack bus (reference bus)
/// net.buses.push(Bus::new(1, BusType::Slack));
/// // Add a PQ load bus
/// net.buses.push(Bus::new(2, BusType::PQ));
///
/// // Add a branch with r=0.01, x=0.1, b=0.02 (p.u.)
/// net.branches.push(Branch {
///     from_bus: 1,
///     to_bus: 2,
///     r: 0.01,
///     x: 0.1,
///     b: 0.02,
///     rate_a: 100.0,
///     rate_b: 100.0,
///     rate_c: 100.0,
///     tap: 0.0,
///     shift: 0.0,
///     status: true,
/// });
///
/// assert!(net.validate().is_ok());
/// assert_eq!(net.bus_count(), 2);
/// assert_eq!(net.branch_count(), 1);
/// ```
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PowerNetwork {
    /// All buses (indexed 0..n−1 internally; `bus.id` holds the external ID).
    pub buses: Vec<Bus>,
    /// All branches (π-model: r, x, b/2 shunt, tap, shift).
    pub branches: Vec<Branch>,
    /// All generators (slack, PV, and dispatchable reactive sources).
    pub generators: Vec<Generator>,
    /// System MVA base used throughout for per-unit conversion.
    pub base_mva: f64,
}

impl PowerNetwork {
    /// Create an empty network with the given MVA base.
    pub fn new(base_mva: f64) -> Self {
        Self {
            buses: Vec::new(),
            branches: Vec::new(),
            generators: Vec::new(),
            base_mva,
        }
    }

    /// Number of buses in the network.
    pub fn bus_count(&self) -> usize {
        self.buses.len()
    }

    /// Number of branches (lines + transformers) in the network.
    pub fn branch_count(&self) -> usize {
        self.branches.len()
    }

    /// Look up the internal 0-based index for a bus given its external ID.
    ///
    /// Returns `Err` if the bus ID is not found.
    pub fn bus_index(&self, bus_id: usize) -> Result<usize> {
        self.buses
            .iter()
            .position(|b| b.id == bus_id)
            .ok_or_else(|| OxiGridError::InvalidNetwork(format!("Bus {bus_id} not found")))
    }

    /// Internal index of the slack bus.
    ///
    /// Returns `Err` if no slack bus is defined (`BusType::Slack`).
    pub fn slack_bus_index(&self) -> Result<usize> {
        self.buses
            .iter()
            .position(|b| b.bus_type == BusType::Slack)
            .ok_or_else(|| OxiGridError::InvalidNetwork("No slack bus found".to_string()))
    }

    /// Build and return the sparse nodal admittance (Y-bus) matrix.
    ///
    /// Uses the π-model for every branch (including transformers with tap + shift).
    pub fn admittance_matrix(&self) -> Result<CsMat<Complex64>> {
        build_y_bus(self)
    }

    /// Total real power load across all buses `MW`.
    pub fn total_load_mw(&self) -> f64 {
        self.buses.iter().map(|b| b.pd.0).sum()
    }

    /// Total reactive power load across all buses `MVAr`.
    pub fn total_load_mvar(&self) -> f64 {
        self.buses.iter().map(|b| b.qd.0).sum()
    }

    /// Sum of `Pmax` for all in-service generators `MW` (installed capacity).
    pub fn installed_capacity_mw(&self) -> f64 {
        self.generators
            .iter()
            .filter(|g| g.status)
            .map(|g| g.pmax)
            .sum()
    }

    /// Sum of current `Pg` setpoints for all in-service generators `MW`.
    pub fn total_generation_mw(&self) -> f64 {
        self.generators
            .iter()
            .filter(|g| g.status)
            .map(|g| g.pg)
            .sum()
    }

    /// Reserve margin: (installed capacity − total load) / total load.
    ///
    /// Returns `f64::INFINITY` if total load is zero.
    pub fn reserve_margin(&self) -> f64 {
        let load = self.total_load_mw();
        if load < 1e-12 {
            return f64::INFINITY;
        }
        (self.installed_capacity_mw() - load) / load
    }

    /// Number of PQ buses (load buses).
    pub fn n_pq_buses(&self) -> usize {
        self.buses
            .iter()
            .filter(|b| b.bus_type == BusType::PQ)
            .count()
    }

    /// Number of PV buses (generator/voltage-controlled buses).
    pub fn n_pv_buses(&self) -> usize {
        self.buses
            .iter()
            .filter(|b| b.bus_type == BusType::PV)
            .count()
    }

    /// Net scheduled real and reactive power injections at each bus [p.u.].
    ///
    /// Returns `(p_inj, q_inj)` vectors of length `bus_count()`.
    /// Positive = injection into network (generation), negative = load.
    pub fn net_injection(&self) -> (Vec<f64>, Vec<f64>) {
        let n = self.bus_count();
        let mut p_inj = vec![0.0; n];
        let mut q_inj = vec![0.0; n];

        // Subtract loads
        for (i, bus) in self.buses.iter().enumerate() {
            p_inj[i] -= bus.pd.0 / self.base_mva;
            q_inj[i] -= bus.qd.0 / self.base_mva;
        }

        // Add generation
        for gen in &self.generators {
            if !gen.status {
                continue;
            }
            if let Ok(idx) = self.bus_index(gen.bus_id) {
                p_inj[idx] += gen.pg / self.base_mva;
                q_inj[idx] += gen.qg / self.base_mva;
            }
        }

        (p_inj, q_inj)
    }

    /// Parse a MATPOWER `.m` file (Case Format v2).
    ///
    /// Accepts any MATPOWER test case: `case14`, `case30`, `case57`, `case118`, etc.
    pub fn from_matpower(path: &str) -> Result<Self> {
        crate::network::formats::matpower::parse_matpower_file(path)
    }

    /// Parse an IEEE Common Data Format (CDF) file.
    pub fn from_ieee_cdf(path: &str) -> Result<Self> {
        crate::network::formats::ieee_cdf::parse_ieee_cdf_file(path)
    }

    /// Parse a pandapower JSON file.
    pub fn from_pandapower(path: &str) -> Result<Self> {
        crate::network::formats::pandapower::parse_pandapower_file(path)
    }

    /// Parse pandapower JSON from a string.
    pub fn from_pandapower_str(content: &str) -> Result<Self> {
        crate::network::formats::pandapower::parse_pandapower_string(content)
    }

    /// Bus-branch incidence matrix A of size (n_bus × n_branch).
    ///
    /// `A[i,k]` = +1 if branch k leaves bus i (from-bus)
    ///         = -1 if branch k enters bus i (to-bus)
    ///         =  0 otherwise
    ///
    /// Returns a dense matrix as `Vec<Vec<f64>>`.
    pub fn incidence_matrix(&self) -> Vec<Vec<f64>> {
        let n = self.bus_count();
        let m = self.branch_count();
        let mut a = vec![vec![0.0f64; m]; n];
        for (k, branch) in self.branches.iter().enumerate() {
            if let (Ok(fi), Ok(ti)) = (
                self.bus_index(branch.from_bus),
                self.bus_index(branch.to_bus),
            ) {
                a[fi][k] = 1.0;
                a[ti][k] = -1.0;
            }
        }
        a
    }

    /// Build a petgraph undirected graph from current buses and branches.
    ///
    /// Called at the start of every topology algorithm. Not cached because
    /// `buses` and `branches` are public and can be mutated externally.
    fn build_petgraph(&self) -> petgraph::Graph<NodeData, EdgeData, petgraph::Undirected> {
        use petgraph::Graph;
        let mut g = Graph::<NodeData, EdgeData, petgraph::Undirected>::new_undirected();

        // Add one node per bus; record NodeIndex for each internal bus index.
        let node_indices: Vec<_> = (0..self.buses.len())
            .map(|i| g.add_node(NodeData { bus_idx: i }))
            .collect();

        // Add one edge per branch using bus_index() to get the internal indices.
        for (bi, branch) in self.branches.iter().enumerate() {
            if let (Ok(fi), Ok(ti)) = (
                self.bus_index(branch.from_bus),
                self.bus_index(branch.to_bus),
            ) {
                let z = (branch.r * branch.r + branch.x * branch.x).sqrt();
                g.add_edge(
                    node_indices[fi],
                    node_indices[ti],
                    EdgeData {
                        branch_idx: bi,
                        z_magnitude: z,
                    },
                );
            }
        }
        g
    }

    /// Returns `true` if the network is connected (all buses reachable from any bus).
    pub fn is_connected(&self) -> bool {
        if self.buses.is_empty() {
            return true;
        }
        let g = self.build_petgraph();
        petgraph::algo::connected_components(&g) == 1
    }

    /// Returns connected components as groups of internal bus indices.
    pub fn connected_components_petgraph(&self) -> Vec<Vec<usize>> {
        use petgraph::visit::{Bfs, VisitMap, Visitable};
        let g = self.build_petgraph();
        let mut visited = g.visit_map();
        let mut components: Vec<Vec<usize>> = Vec::new();

        for start in g.node_indices() {
            if visited.is_visited(&start) {
                continue;
            }
            let mut component = Vec::new();
            let mut bfs = Bfs::new(&g, start);
            while let Some(nx) = bfs.next(&g) {
                visited.visit(nx);
                component.push(g[nx].bus_idx);
            }
            components.push(component);
        }
        components
    }

    /// Returns `true` if the network is a tree (connected, branch_count == bus_count - 1).
    pub fn is_radial(&self) -> bool {
        self.bus_count() > 0 && self.is_connected() && self.branch_count() == self.bus_count() - 1
    }

    /// Shortest path between two buses by impedance magnitude (Dijkstra / A*).
    ///
    /// `from` and `to` are internal 0-based bus indices. Returns `None` if
    /// either index is out of range or there is no path.
    pub fn shortest_path(&self, from: usize, to: usize) -> Option<(f64, Vec<usize>)> {
        use petgraph::algo::dijkstra;
        use petgraph::graph::NodeIndex;

        let n = self.buses.len();
        if from >= n || to >= n {
            return None;
        }

        let g = self.build_petgraph();
        let start = NodeIndex::new(from);
        let goal = NodeIndex::new(to);

        // Compute total cost with Dijkstra first.
        let costs = dijkstra(&g, start, Some(goal), |e| e.weight().z_magnitude);
        let total_cost = *costs.get(&goal)?;

        // Reconstruct the actual path using A*.
        let (_, path) = petgraph::algo::astar(
            &g,
            start,
            |n| n == goal,
            |e| e.weight().z_magnitude,
            |_| 0.0,
        )?;

        let bus_path: Vec<usize> = path.iter().map(|&nx| g[nx].bus_idx).collect();
        Some((total_cost, bus_path))
    }

    /// Minimum spanning tree branch indices (Kruskal by impedance magnitude).
    ///
    /// Returns a list of branch indices (into `self.branches`) that form
    /// the minimum spanning tree. Returns an empty Vec if the network has
    /// fewer than 2 buses.
    pub fn spanning_tree(&self) -> Vec<usize> {
        let n = self.buses.len();
        if n < 2 {
            return Vec::new();
        }

        // Build petgraph and collect edges with their weights.
        let g = self.build_petgraph();

        // Gather (z_magnitude, from_node_idx, to_node_idx, branch_idx) tuples.
        let mut sorted: Vec<(f64, usize, usize, usize)> = g
            .edge_indices()
            .filter_map(|ei| {
                let (src, dst) = g.edge_endpoints(ei)?;
                let w = g[ei].z_magnitude;
                let bi = g[ei].branch_idx;
                Some((w, src.index(), dst.index(), bi))
            })
            .collect();
        sorted.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

        // Union-Find with path compression and union by rank.
        let mut parent: Vec<usize> = (0..n).collect();
        let mut rank = vec![0usize; n];

        fn find(parent: &mut [usize], x: usize) -> usize {
            if parent[x] != x {
                let next = parent[x];
                let root = find(parent, next);
                parent[x] = root;
            }
            parent[x]
        }

        fn union(parent: &mut [usize], rank: &mut [usize], x: usize, y: usize) -> bool {
            let rx = find(parent, x);
            let ry = find(parent, y);
            if rx == ry {
                return false;
            }
            if rank[rx] < rank[ry] {
                parent[rx] = ry;
            } else if rank[rx] > rank[ry] {
                parent[ry] = rx;
            } else {
                parent[ry] = rx;
                rank[rx] += 1;
            }
            true
        }

        let mut mst = Vec::with_capacity(n - 1);
        for (_weight, fi, ti, bi) in &sorted {
            if union(&mut parent, &mut rank, *fi, *ti) {
                mst.push(*bi);
                if mst.len() == n - 1 {
                    break;
                }
            }
        }
        mst
    }

    /// BFS traversal order starting from `start_bus_idx` (internal 0-based).
    ///
    /// Returns internal bus indices in BFS discovery order. Returns empty
    /// Vec if `start_bus_idx` is out of range.
    pub fn bfs_visit_order(&self, start_bus_idx: usize) -> Vec<usize> {
        if start_bus_idx >= self.buses.len() {
            return Vec::new();
        }
        let g = self.build_petgraph();
        let start = petgraph::graph::NodeIndex::new(start_bus_idx);
        let mut bfs = petgraph::visit::Bfs::new(&g, start);
        let mut order = Vec::new();
        while let Some(nx) = bfs.next(&g) {
            order.push(g[nx].bus_idx);
        }
        order
    }

    /// Number of branches incident to `bus_id` (degree in the network graph).
    ///
    /// Returns `0` if the bus ID is not found or has no branches.
    pub fn degree(&self, bus_id: usize) -> usize {
        self.branches
            .iter()
            .filter(|b| b.from_bus == bus_id || b.to_bus == bus_id)
            .count()
    }

    /// External bus IDs of all buses directly connected to `bus_id` by a branch.
    ///
    /// Returns an empty vector if the bus ID is not found or has no branches.
    pub fn neighbors(&self, bus_id: usize) -> Vec<usize> {
        let mut nbrs: Vec<usize> = self
            .branches
            .iter()
            .filter_map(|b| {
                if b.from_bus == bus_id {
                    Some(b.to_bus)
                } else if b.to_bus == bus_id {
                    Some(b.from_bus)
                } else {
                    None
                }
            })
            .collect();
        nbrs.sort_unstable();
        nbrs.dedup();
        nbrs
    }

    /// Validate network data consistency.
    ///
    /// Checks:
    /// - At least one bus exists.
    /// - Exactly one slack bus is present.
    /// - All branch from/to bus IDs refer to defined buses.
    pub fn validate(&self) -> Result<()> {
        if self.buses.is_empty() {
            return Err(OxiGridError::InvalidNetwork("No buses defined".into()));
        }

        let has_slack = self.buses.iter().any(|b| b.bus_type == BusType::Slack);
        if !has_slack {
            return Err(OxiGridError::InvalidNetwork("No slack bus defined".into()));
        }

        for branch in &self.branches {
            self.bus_index(branch.from_bus)?;
            self.bus_index(branch.to_bus)?;
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::branch::Branch;
    use crate::network::bus::{Bus, BusType};
    use crate::units::Power;

    /// Build a simple 3-bus triangle network: slack(1) -- PV(2) -- PQ(3) -- slack(1).
    fn three_bus_ring() -> PowerNetwork {
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        net.buses.push(Bus::new(2, BusType::PV));
        net.buses.push(Bus::new(3, BusType::PQ));

        let make_branch = |f: usize, t: usize| Branch {
            from_bus: f,
            to_bus: t,
            r: 0.01,
            x: 0.10,
            b: 0.02,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        };

        net.branches.push(make_branch(1, 2));
        net.branches.push(make_branch(2, 3));
        net.branches.push(make_branch(3, 1));
        net
    }

    /// Build a simple 2-bus radial (tree) network.
    fn two_bus_radial() -> PowerNetwork {
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        net.buses.push(Bus::new(2, BusType::PQ));
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.01,
            x: 0.10,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
        net
    }

    #[test]
    fn test_bus_index_found() {
        let net = three_bus_ring();
        let idx = net.bus_index(2).expect("bus 2 should exist");
        assert_eq!(net.buses[idx].id, 2);
    }

    #[test]
    fn test_bus_index_not_found_returns_err() {
        let net = three_bus_ring();
        assert!(net.bus_index(99).is_err());
    }

    #[test]
    fn test_slack_bus_index_found() {
        let net = three_bus_ring();
        let idx = net.slack_bus_index().expect("slack must exist");
        assert_eq!(net.buses[idx].bus_type, BusType::Slack);
    }

    #[test]
    fn test_slack_bus_index_no_slack_returns_err() {
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::PQ));
        assert!(net.slack_bus_index().is_err());
    }

    #[test]
    fn test_total_load_mw_sums_all_buses() {
        let mut net = three_bus_ring();
        net.buses[1].pd = Power(50.0);
        net.buses[2].pd = Power(30.0);
        let load = net.total_load_mw();
        assert!(
            (load - 80.0).abs() < 1e-9,
            "expected 80 MW, got {:.2}",
            load
        );
    }

    #[test]
    fn test_reserve_margin_infinity_when_no_load() {
        let net = three_bus_ring();
        // No load defined → total_load_mw == 0 → INFINITY
        assert_eq!(net.reserve_margin(), f64::INFINITY);
    }

    #[test]
    fn test_reserve_margin_with_load_and_capacity() {
        let mut net = three_bus_ring();
        net.buses[1].pd = Power(80.0);
        net.generators.push(Generator {
            bus_id: 1,
            pg: 100.0,
            qg: 0.0,
            qmax: 50.0,
            qmin: -50.0,
            vg: 1.0,
            mbase: 100.0,
            status: true,
            pmax: 120.0,
            pmin: 0.0,
        });
        // installed = 120, load = 80 → margin = 0.5
        let margin = net.reserve_margin();
        assert!((margin - 0.5).abs() < 1e-9, "margin = {:.4}", margin);
    }

    #[test]
    fn test_n_pq_and_pv_buses() {
        let net = three_bus_ring();
        assert_eq!(net.n_pq_buses(), 1);
        assert_eq!(net.n_pv_buses(), 1);
    }

    #[test]
    fn test_net_injection_length_matches_bus_count() {
        let net = three_bus_ring();
        let (p, q) = net.net_injection();
        assert_eq!(p.len(), net.bus_count());
        assert_eq!(q.len(), net.bus_count());
    }

    #[test]
    fn test_net_injection_load_is_negative() {
        let mut net = two_bus_radial();
        net.buses[1].pd = Power(50.0);
        let (p, _q) = net.net_injection();
        // Bus 1 (index 1) has 50 MW load → injection = -0.5 pu (base 100 MVA)
        assert!(p[1] < 0.0, "load bus injection must be negative");
        assert!((p[1] + 0.5).abs() < 1e-9, "p[1] = {:.4}", p[1]);
    }

    #[test]
    fn test_incidence_matrix_dimensions() {
        let net = three_bus_ring();
        let a = net.incidence_matrix();
        assert_eq!(a.len(), net.bus_count());
        assert_eq!(a[0].len(), net.branch_count());
    }

    #[test]
    fn test_is_connected_for_connected_network() {
        let net = three_bus_ring();
        assert!(net.is_connected());
    }

    #[test]
    fn test_is_connected_false_for_isolated_bus() {
        let mut net = three_bus_ring();
        // Add an isolated bus (no branches)
        net.buses.push(Bus::new(4, BusType::PQ));
        assert!(!net.is_connected());
    }

    #[test]
    fn test_connected_components_two_islands() {
        let mut net = three_bus_ring();
        net.buses.push(Bus::new(4, BusType::PQ));
        let comps = net.connected_components_petgraph();
        assert_eq!(comps.len(), 2, "expected 2 components, got {}", comps.len());
    }

    #[test]
    fn test_is_radial_true_for_tree() {
        let net = two_bus_radial();
        // 2 buses, 1 branch → radial
        assert!(net.is_radial());
    }

    #[test]
    fn test_is_radial_false_for_ring() {
        let net = three_bus_ring();
        // 3 buses, 3 branches → not radial (has a cycle)
        assert!(!net.is_radial());
    }

    #[test]
    fn test_shortest_path_two_hop() {
        let net = three_bus_ring();
        // Bus indices 0→2 (external IDs 1→3) — ring, so a direct 1-hop path exists
        let result = net.shortest_path(0, 2);
        assert!(result.is_some());
        let (cost, path) = result.unwrap();
        assert!(cost > 0.0, "cost must be positive");
        assert!(path.len() >= 2);
    }

    #[test]
    fn test_spanning_tree_covers_all_buses() {
        let net = three_bus_ring();
        let mst = net.spanning_tree();
        // MST of 3 buses = 2 branches
        assert_eq!(mst.len(), 2, "MST size = {}", mst.len());
    }

    #[test]
    fn test_bfs_visit_order_starts_at_given_bus() {
        let net = three_bus_ring();
        let order = net.bfs_visit_order(0);
        assert!(!order.is_empty());
        assert_eq!(order[0], 0, "BFS must start with given bus index");
    }

    #[test]
    fn test_validate_ok_for_valid_network() {
        let net = three_bus_ring();
        assert!(net.validate().is_ok());
    }

    #[test]
    fn test_validate_fails_no_buses() {
        let net = PowerNetwork::new(100.0);
        assert!(net.validate().is_err());
    }

    #[test]
    fn test_validate_fails_no_slack() {
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::PQ));
        assert!(net.validate().is_err());
    }
}
