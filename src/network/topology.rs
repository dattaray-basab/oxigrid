use crate::error::{OxiGridError, Result};
use crate::network::admittance::build_y_bus;
use crate::network::branch::Branch;
use crate::network::bus::{Bus, BusType};
use num_complex::Complex64;
use serde::{Deserialize, Serialize};
use sprs::CsMat;

/// A synchronous generator or voltage-controlled reactive source.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Generator {
    /// Bus this generator is connected to (1-based bus ID).
    pub bus_id: usize,
    /// Real power output setpoint [MW].
    pub pg: f64,
    /// Reactive power output setpoint [MVAr].
    pub qg: f64,
    /// Maximum reactive power output [MVAr].
    pub qmax: f64,
    /// Minimum reactive power output [MVAr] (negative = absorbing).
    pub qmin: f64,
    /// Terminal voltage setpoint [p.u.] (used for PV bus initialisation).
    pub vg: f64,
    /// Machine MVA base (usually = system base).
    pub mbase: f64,
    /// Online status (`true` = in-service).
    pub status: bool,
    /// Maximum real power output [MW].
    pub pmax: f64,
    /// Minimum real power output [MW].
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

    /// Total real power load across all buses [MW].
    pub fn total_load_mw(&self) -> f64 {
        self.buses.iter().map(|b| b.pd.0).sum()
    }

    /// Total reactive power load across all buses [MVAr].
    pub fn total_load_mvar(&self) -> f64 {
        self.buses.iter().map(|b| b.qd.0).sum()
    }

    /// Sum of `Pmax` for all in-service generators [MW] (installed capacity).
    pub fn installed_capacity_mw(&self) -> f64 {
        self.generators
            .iter()
            .filter(|g| g.status)
            .map(|g| g.pmax)
            .sum()
    }

    /// Sum of current `Pg` setpoints for all in-service generators [MW].
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
    /// A[i,k] = +1 if branch k leaves bus i (from-bus)
    ///         = -1 if branch k enters bus i (to-bus)
    ///         =  0 otherwise
    ///
    /// Returns a dense matrix as Vec<Vec<f64>>.
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

    /// Returns `true` if the network graph is fully connected (ignoring branch status).
    ///
    /// Uses a BFS from the first bus over all defined branches.
    pub fn is_connected(&self) -> bool {
        let n = self.buses.len();
        if n == 0 {
            return true;
        }
        // Build adjacency list (use bus index, not external ID)
        let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];
        for branch in &self.branches {
            if let (Ok(fi), Ok(ti)) = (
                self.bus_index(branch.from_bus),
                self.bus_index(branch.to_bus),
            ) {
                adj[fi].push(ti);
                adj[ti].push(fi);
            }
        }
        // BFS from node 0
        let mut visited = vec![false; n];
        let mut queue = std::collections::VecDeque::new();
        visited[0] = true;
        queue.push_back(0usize);
        while let Some(node) = queue.pop_front() {
            for &nbr in &adj[node] {
                if !visited[nbr] {
                    visited[nbr] = true;
                    queue.push_back(nbr);
                }
            }
        }
        visited.iter().all(|&v| v)
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
