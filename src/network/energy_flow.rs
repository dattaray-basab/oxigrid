//! Multi-commodity energy network flow model.
//!
//! Solves a minimum-cost flow problem across multiple energy carriers
//! (electricity, natural gas, heat, hydrogen, cooling water) with
//! coupling constraints between energy forms (e.g., power-to-gas,
//! combined heat and power, electrolysers).
//!
//! # Algorithm
//! For each commodity the solver runs a greedy shortest-path routing:
//! 1. Build supply/demand imbalances per node.
//! 2. Find least-cost path from supply nodes to demand nodes (Dijkstra).
//! 3. Route flow along the path up to `min(supply, demand, arc capacity)`.
//! 4. Apply coupling constraints: excess electricity may feed an
//!    electrolyser and generate hydrogen (or heat).
//! 5. Accumulate losses and unmet demand.
//!
//! # References
//! - Geidl, M. & Andersson, G., "Optimal Power Flow of Multiple Energy Carriers",
//!   IEEE Trans. Power Systems, vol. 22, no. 1, 2007.
//! - Papaefthymiou, G. & Kurowicka, D., "Using Copulas for Modeling Stochastic
//!   Dependence in Power System Uncertainty Analysis", IEEE TPWRS, 2009.

use thiserror::Error;

// ─────────────────────────────────────────────────────────────────────────────
// Error
// ─────────────────────────────────────────────────────────────────────────────

/// Errors from the multi-commodity flow solver.
#[derive(Debug, Error)]
pub enum FlowError {
    /// Configuration is inconsistent.
    #[error("invalid configuration: {0}")]
    InvalidConfig(String),
    /// A referenced node or carrier index is out of range.
    #[error("index out of range: {0}")]
    IndexOutOfRange(String),
    /// Numerical issue during solving.
    #[error("numerical error: {0}")]
    NumericalError(String),
}

// ─────────────────────────────────────────────────────────────────────────────
// Energy carrier types
// ─────────────────────────────────────────────────────────────────────────────

/// An energy carrier with its physical parameters.
#[derive(Debug, Clone)]
pub enum EnergyCarrierFlow {
    /// AC/DC electricity with a system base MVA.
    Electricity {
        /// System base \[MVA\].
        base_mva: f64,
    },
    /// Natural gas at a given operating pressure.
    NaturalGas {
        /// Operating pressure \[bar\].
        pressure_bar: f64,
    },
    /// Thermal energy (district heating/cooling) at a given temperature.
    Heat {
        /// Supply temperature \[°C\].
        temperature_c: f64,
    },
    /// Hydrogen (compressed or liquid) at a given pressure.
    Hydrogen {
        /// Operating pressure \[bar\].
        pressure_bar: f64,
    },
    /// Cooling water at a volumetric flow rate.
    CoolingWater {
        /// Nominal flow rate \[m³/h\].
        flow_rate_m3_per_h: f64,
    },
}

impl EnergyCarrierFlow {
    /// Human-readable name for this carrier.
    pub fn name(&self) -> &'static str {
        match self {
            EnergyCarrierFlow::Electricity { .. } => "Electricity",
            EnergyCarrierFlow::NaturalGas { .. } => "NaturalGas",
            EnergyCarrierFlow::Heat { .. } => "Heat",
            EnergyCarrierFlow::Hydrogen { .. } => "Hydrogen",
            EnergyCarrierFlow::CoolingWater { .. } => "CoolingWater",
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Coupling constraint
// ─────────────────────────────────────────────────────────────────────────────

/// Conversion link between two energy carriers at a hub node.
///
/// Examples: CHP (gas → electricity + heat), electrolyser (electricity → hydrogen).
#[derive(Debug, Clone)]
pub struct CouplingConstraint {
    /// Index of the input (source) carrier in [`EnergyFlowConfig::commodities`].
    pub source_carrier: usize,
    /// Index of the output (destination) carrier.
    pub dest_carrier: usize,
    /// Node where the conversion device is installed.
    pub node: usize,
    /// Conversion efficiency: output / input (dimensionless).
    pub conversion_efficiency: f64,
    /// Maximum conversion rate in source carrier units per time step.
    pub max_conversion_rate: f64,
    /// If `true`, energy can also flow from dest back to source.
    pub bidirectional: bool,
}

// ─────────────────────────────────────────────────────────────────────────────
// Objective
// ─────────────────────────────────────────────────────────────────────────────

/// Optimisation objective for the flow solver.
#[derive(Debug, Clone)]
pub enum FlowObjective {
    /// Minimise total arc cost.
    MinimizeCost,
    /// Minimise total energy losses.
    MinimizeLosses,
    /// Maximise renewable utilisation (minimise curtailment).
    MaximizeRenewable,
    /// Weighted combination of cost, losses, and renewable.
    MultiObjective {
        /// Weights for \[cost, losses, renewable\].
        weights: Vec<f64>,
    },
}

// ─────────────────────────────────────────────────────────────────────────────
// Configuration
// ─────────────────────────────────────────────────────────────────────────────

/// Configuration for the multi-commodity flow model.
#[derive(Debug, Clone)]
pub struct EnergyFlowConfig {
    /// Number of nodes in the energy network.
    pub n_nodes: usize,
    /// Energy carriers handled by this model.
    pub commodities: Vec<EnergyCarrierFlow>,
    /// Coupling constraints between carriers.
    pub coupling_constraints: Vec<CouplingConstraint>,
    /// Optimisation objective.
    pub optimization_objective: FlowObjective,
}

// ─────────────────────────────────────────────────────────────────────────────
// Network elements
// ─────────────────────────────────────────────────────────────────────────────

/// Node classification.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum NodeType {
    /// Pure supply node.
    Source,
    /// Pure demand node.
    Sink,
    /// Conversion hub (both supply and demand possible).
    Hub,
    /// Pass-through / intermediate node.
    Transit,
}

/// A node in the multi-energy network.
#[derive(Debug, Clone)]
pub struct NetworkNode {
    /// Unique node identifier (0-indexed).
    pub id: usize,
    /// Human-readable label.
    pub name: String,
    /// Node type.
    pub node_type: NodeType,
    /// Available supply per commodity \[energy units\].
    pub supply: Vec<f64>,
    /// Required demand per commodity \[energy units\].
    pub demand: Vec<f64>,
    /// Storage capacity per commodity \[energy units\].
    pub storage: Vec<f64>,
}

/// A directed arc in the multi-energy network.
#[derive(Debug, Clone)]
pub struct NetworkArc {
    /// Tail node.
    pub from_node: usize,
    /// Head node.
    pub to_node: usize,
    /// Carrier index.
    pub carrier: usize,
    /// Maximum flow capacity \[energy units\].
    pub capacity: f64,
    /// Transportation cost per unit flow \[cost/unit\].
    pub cost_per_unit: f64,
    /// Fraction of flow lost in transit (0 = lossless).
    pub loss_factor: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Result
// ─────────────────────────────────────────────────────────────────────────────

/// Flow solution.
#[derive(Debug, Clone)]
pub struct FlowResult {
    /// Flows on each arc: `(from_node, to_node, carrier, flow)`.
    pub arc_flows: Vec<(usize, usize, usize, f64)>,
    /// Net balance per `[node][carrier]` (positive = net supply, negative = net demand).
    pub node_balance: Vec<Vec<f64>>,
    /// Total transportation cost.
    pub total_cost: f64,
    /// Total energy losses across all arcs.
    pub total_losses: f64,
    /// Unmet demand per commodity.
    pub unmet_demand: Vec<f64>,
    /// Utilisation fraction of each coupling constraint (0–1).
    pub coupling_utilization: Vec<f64>,
}

// ─────────────────────────────────────────────────────────────────────────────
// Solver
// ─────────────────────────────────────────────────────────────────────────────

/// Multi-commodity energy network flow solver.
///
/// # Usage
/// ```rust,ignore
/// use oxigrid::network::energy_flow::{
///     MultiCommodityFlowSolver, EnergyFlowConfig, EnergyCarrierFlow,
///     FlowObjective, NetworkNode, NetworkArc, NodeType,
/// };
/// let config = EnergyFlowConfig {
///     n_nodes: 3,
///     commodities: vec![EnergyCarrierFlow::Electricity { base_mva: 100.0 }],
///     coupling_constraints: vec![],
///     optimization_objective: FlowObjective::MinimizeCost,
/// };
/// let mut solver = MultiCommodityFlowSolver::new(config);
/// ```
pub struct MultiCommodityFlowSolver {
    config: EnergyFlowConfig,
    nodes: Vec<NetworkNode>,
    arcs: Vec<NetworkArc>,
}

impl MultiCommodityFlowSolver {
    /// Create a new solver with the given configuration.
    pub fn new(config: EnergyFlowConfig) -> Self {
        Self {
            config,
            nodes: Vec::new(),
            arcs: Vec::new(),
        }
    }

    /// Add a node to the network.
    pub fn add_node(&mut self, node: NetworkNode) {
        self.nodes.push(node);
    }

    /// Add a directed arc to the network.
    pub fn add_arc(&mut self, arc: NetworkArc) {
        self.arcs.push(arc);
    }

    /// Solve the multi-commodity flow problem.
    ///
    /// Returns a [`FlowResult`] containing arc flows, node balances, cost,
    /// losses, unmet demand, and coupling utilisation.
    pub fn solve(&self) -> Result<FlowResult, FlowError> {
        let n_nodes = self.nodes.len();
        let n_commodities = self.config.commodities.len();

        if n_nodes == 0 {
            return Err(FlowError::InvalidConfig("No nodes in network".to_string()));
        }
        if n_commodities == 0 {
            return Err(FlowError::InvalidConfig(
                "No commodities defined".to_string(),
            ));
        }

        // Validate node indices
        for (i, node) in self.nodes.iter().enumerate() {
            if node.id != i {
                // Allow any id — we use positional index internally
            }
            if node.supply.len() != n_commodities {
                return Err(FlowError::IndexOutOfRange(format!(
                    "Node {} supply vector has {} entries, expected {}",
                    node.id,
                    node.supply.len(),
                    n_commodities
                )));
            }
        }

        // Mutable arc capacities for flow routing
        let mut arc_remaining: Vec<f64> = self.arcs.iter().map(|a| a.capacity).collect();

        // Track flows per arc
        let mut arc_flow_values: Vec<f64> = vec![0.0; self.arcs.len()];

        // Node balance [node][carrier] — starts at supply - demand
        let mut node_balance: Vec<Vec<f64>> = self
            .nodes
            .iter()
            .map(|n| {
                (0..n_commodities)
                    .map(|c| {
                        n.supply.get(c).copied().unwrap_or(0.0)
                            - n.demand.get(c).copied().unwrap_or(0.0)
                    })
                    .collect()
            })
            .collect();

        // For each commodity, route supply to demand greedily
        for carrier in 0..n_commodities {
            self.route_commodity(
                carrier,
                &mut node_balance,
                &mut arc_remaining,
                &mut arc_flow_values,
            )?;
        }

        // Apply coupling constraints
        let mut coupling_utilization = vec![0.0_f64; self.config.coupling_constraints.len()];
        for (ci, cc) in self.config.coupling_constraints.iter().enumerate() {
            let src = cc.source_carrier;
            let dst = cc.dest_carrier;
            if src >= n_commodities || dst >= n_commodities {
                continue;
            }
            let node_pos = self
                .nodes
                .iter()
                .position(|n| n.id == cc.node)
                .unwrap_or(cc.node);
            if node_pos >= n_nodes {
                continue;
            }
            // Available surplus in source carrier at the coupling node
            let surplus = node_balance[node_pos][src].max(0.0);
            // Deficit in destination carrier at the coupling node
            let deficit = (-node_balance[node_pos][dst]).max(0.0);
            // Convert up to max_conversion_rate
            let convertible = surplus.min(cc.max_conversion_rate);
            let converted_out = convertible * cc.conversion_efficiency;
            let actual_convert = convertible.min(deficit / cc.conversion_efficiency.max(1e-12));
            let actual_out = actual_convert * cc.conversion_efficiency;
            node_balance[node_pos][src] -= actual_convert;
            node_balance[node_pos][dst] += actual_out;
            coupling_utilization[ci] = if cc.max_conversion_rate > 1e-12 {
                actual_convert / cc.max_conversion_rate
            } else {
                0.0
            };
            let _ = converted_out; // suppress warning
        }

        // Compute unmet demand per commodity
        let unmet_demand: Vec<f64> = (0..n_commodities)
            .map(|c| {
                self.nodes
                    .iter()
                    .enumerate()
                    .map(|(ni, node)| {
                        let demand = node.demand.get(c).copied().unwrap_or(0.0);
                        let supply_net = node_balance[ni][c];
                        // Unmet = demand still unsatisfied
                        if supply_net < 0.0 {
                            supply_net.abs().min(demand)
                        } else {
                            0.0
                        }
                    })
                    .sum()
            })
            .collect();

        // Build arc flow records and compute totals
        let mut arc_flows: Vec<(usize, usize, usize, f64)> = Vec::new();
        let mut total_cost = 0.0_f64;
        let mut total_losses = 0.0_f64;

        for (i, arc) in self.arcs.iter().enumerate() {
            let flow = arc_flow_values[i];
            if flow > 1e-12 {
                arc_flows.push((arc.from_node, arc.to_node, arc.carrier, flow));
                total_cost += flow * arc.cost_per_unit;
                total_losses += flow * arc.loss_factor;
            }
        }

        Ok(FlowResult {
            arc_flows,
            node_balance,
            total_cost,
            total_losses,
            unmet_demand,
            coupling_utilization,
        })
    }

    /// Route a single commodity greedily using Dijkstra-based shortest paths.
    fn route_commodity(
        &self,
        carrier: usize,
        node_balance: &mut [Vec<f64>],
        arc_remaining: &mut [f64],
        arc_flow_values: &mut [f64],
    ) -> Result<(), FlowError> {
        let n_nodes = self.nodes.len();

        // Collect supply and demand nodes for this carrier
        let supply_nodes: Vec<usize> = (0..n_nodes)
            .filter(|&i| node_balance[i][carrier] > 1e-9)
            .collect();

        let demand_nodes: Vec<usize> = (0..n_nodes)
            .filter(|&i| node_balance[i][carrier] < -1e-9)
            .collect();

        if supply_nodes.is_empty() || demand_nodes.is_empty() {
            return Ok(()); // Nothing to route
        }

        // For each supply node, route to demand nodes via shortest path
        for &src in &supply_nodes {
            if node_balance[src][carrier] < 1e-9 {
                continue;
            }
            // Dijkstra to find cheapest path to all demand nodes
            let predecessors = self.dijkstra(src, carrier, arc_remaining);

            for &dst in &demand_nodes {
                if node_balance[dst][carrier] > -1e-9 {
                    continue; // Demand already satisfied
                }
                if node_balance[src][carrier] < 1e-9 {
                    break; // Supply exhausted
                }

                // Reconstruct path from dst back to src
                let path = reconstruct_path(src, dst, &predecessors);
                if path.is_empty() {
                    continue; // No path found
                }

                // Determine max flow along the path
                let avail_supply = node_balance[src][carrier];
                let demand_deficit = (-node_balance[dst][carrier]).max(0.0);
                let bottleneck = self.path_bottleneck(&path, carrier, arc_remaining);

                let flow = avail_supply.min(demand_deficit).min(bottleneck);
                if flow < 1e-12 {
                    continue;
                }

                // Route flow along path
                let mut remaining_flow = flow;
                for (&u, &v) in path.iter().zip(path.iter().skip(1)) {
                    if let Some(arc_idx) = self.find_arc(u, v, carrier) {
                        let arc = &self.arcs[arc_idx];
                        let actual = remaining_flow.min(arc_remaining[arc_idx]);
                        arc_remaining[arc_idx] -= actual;
                        arc_flow_values[arc_idx] += actual;
                        remaining_flow = actual * (1.0 - arc.loss_factor);
                    }
                }

                // Update balances
                node_balance[src][carrier] -= flow;
                node_balance[dst][carrier] += flow; // reduce deficit (note: deficit is negative)
            }
        }
        Ok(())
    }

    /// Dijkstra shortest-path tree from `source` for `carrier`.
    ///
    /// Returns a predecessor map: `predecessors[v]` = the node before `v` on
    /// the shortest path from `source`.
    pub fn shortest_path(&self, source: usize, carrier: usize) -> Vec<usize> {
        let arc_remaining = vec![f64::MAX; self.arcs.len()]; // ignore capacity
        self.dijkstra(source, carrier, &arc_remaining)
    }

    /// Internal Dijkstra with capacity-aware arc filtering.
    fn dijkstra(&self, source: usize, carrier: usize, arc_remaining: &[f64]) -> Vec<usize> {
        let n = self.nodes.len();
        let mut dist = vec![f64::MAX; n];
        let mut pred = vec![usize::MAX; n];
        let mut visited = vec![false; n];

        dist[source] = 0.0;

        for _ in 0..n {
            // Select unvisited node with minimum distance
            let u = match (0..n)
                .filter(|&i| !visited[i] && dist[i] < f64::MAX)
                .min_by(|&a, &b| {
                    dist[a]
                        .partial_cmp(&dist[b])
                        .unwrap_or(std::cmp::Ordering::Equal)
                }) {
                Some(v) => v,
                None => break,
            };
            visited[u] = true;

            // Relax outgoing arcs from u for this carrier
            for (arc_idx, arc) in self.arcs.iter().enumerate() {
                if arc.carrier != carrier || arc.from_node != u {
                    continue;
                }
                if arc_remaining[arc_idx] < 1e-12 {
                    continue; // arc saturated
                }
                let v = arc.to_node;
                if v >= n || visited[v] {
                    continue;
                }
                let new_dist = dist[u] + arc.cost_per_unit;
                if new_dist < dist[v] {
                    dist[v] = new_dist;
                    pred[v] = u;
                }
            }
        }
        pred
    }

    /// Find the arc index connecting `u → v` for `carrier` (first match).
    fn find_arc(&self, u: usize, v: usize, carrier: usize) -> Option<usize> {
        self.arcs
            .iter()
            .position(|a| a.from_node == u && a.to_node == v && a.carrier == carrier)
    }

    /// Minimum remaining capacity along a path (bottleneck).
    fn path_bottleneck(&self, path: &[usize], carrier: usize, arc_remaining: &[f64]) -> f64 {
        let mut min_cap = f64::MAX;
        for (&u, &v) in path.iter().zip(path.iter().skip(1)) {
            if let Some(arc_idx) = self.find_arc(u, v, carrier) {
                min_cap = min_cap.min(arc_remaining[arc_idx]);
            } else {
                return 0.0; // broken path
            }
        }
        if min_cap == f64::MAX {
            0.0
        } else {
            min_cap
        }
    }
}

/// Reconstruct a path from `src` to `dst` given a predecessor array.
fn reconstruct_path(src: usize, dst: usize, pred: &[usize]) -> Vec<usize> {
    if pred[dst] == usize::MAX && dst != src {
        return Vec::new(); // unreachable
    }
    let mut path = Vec::new();
    let mut cur = dst;
    let mut steps = 0usize;
    let max_steps = pred.len() + 1;
    while cur != src && steps < max_steps {
        path.push(cur);
        let p = pred[cur];
        if p == usize::MAX {
            return Vec::new(); // broken predecessor chain
        }
        cur = p;
        steps += 1;
    }
    path.push(src);
    path.reverse();
    path
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn elec_config() -> EnergyFlowConfig {
        EnergyFlowConfig {
            n_nodes: 3,
            commodities: vec![EnergyCarrierFlow::Electricity { base_mva: 100.0 }],
            coupling_constraints: vec![],
            optimization_objective: FlowObjective::MinimizeCost,
        }
    }

    fn make_node(id: usize, supply: f64, demand: f64, n_carriers: usize) -> NetworkNode {
        NetworkNode {
            id,
            name: format!("N{id}"),
            node_type: if supply > 0.0 {
                NodeType::Source
            } else {
                NodeType::Sink
            },
            supply: (0..n_carriers)
                .map(|c| if c == 0 { supply } else { 0.0 })
                .collect(),
            demand: (0..n_carriers)
                .map(|c| if c == 0 { demand } else { 0.0 })
                .collect(),
            storage: vec![0.0; n_carriers],
        }
    }

    #[test]
    fn test_single_commodity_balanced_flow() {
        let config = elec_config();
        let mut solver = MultiCommodityFlowSolver::new(config);

        // Node 0: 100 MW supply; Node 1: 100 MW demand; arc 0→1
        solver.add_node(make_node(0, 100.0, 0.0, 1));
        solver.add_node(make_node(1, 0.0, 100.0, 1));
        solver.add_arc(NetworkArc {
            from_node: 0,
            to_node: 1,
            carrier: 0,
            capacity: 150.0,
            cost_per_unit: 1.0,
            loss_factor: 0.0,
        });

        let result = solver.solve().unwrap();

        assert!(
            result.unmet_demand[0] < 1e-6,
            "Demand should be fully met: {:.4}",
            result.unmet_demand[0]
        );
        let flow: f64 = result.arc_flows.iter().map(|&(_, _, _, f)| f).sum();
        assert!(
            (flow - 100.0).abs() < 1e-6,
            "Total flow should be 100: {flow:.4}"
        );
    }

    #[test]
    fn test_multi_commodity_separate_carriers() {
        // Two carriers (electricity + gas) routed independently
        let config = EnergyFlowConfig {
            n_nodes: 4,
            commodities: vec![
                EnergyCarrierFlow::Electricity { base_mva: 100.0 },
                EnergyCarrierFlow::NaturalGas { pressure_bar: 50.0 },
            ],
            coupling_constraints: vec![],
            optimization_objective: FlowObjective::MinimizeCost,
        };

        let mut solver = MultiCommodityFlowSolver::new(config);

        // Electricity: node 0 → node 1
        solver.add_node(NetworkNode {
            id: 0,
            name: "ElecSource".to_string(),
            node_type: NodeType::Source,
            supply: vec![50.0, 0.0],
            demand: vec![0.0, 0.0],
            storage: vec![0.0, 0.0],
        });
        solver.add_node(NetworkNode {
            id: 1,
            name: "ElecSink".to_string(),
            node_type: NodeType::Sink,
            supply: vec![0.0, 0.0],
            demand: vec![50.0, 0.0],
            storage: vec![0.0, 0.0],
        });
        // Gas: node 2 → node 3
        solver.add_node(NetworkNode {
            id: 2,
            name: "GasSource".to_string(),
            node_type: NodeType::Source,
            supply: vec![0.0, 80.0],
            demand: vec![0.0, 0.0],
            storage: vec![0.0, 0.0],
        });
        solver.add_node(NetworkNode {
            id: 3,
            name: "GasSink".to_string(),
            node_type: NodeType::Sink,
            supply: vec![0.0, 0.0],
            demand: vec![0.0, 80.0],
            storage: vec![0.0, 0.0],
        });

        solver.add_arc(NetworkArc {
            from_node: 0,
            to_node: 1,
            carrier: 0,
            capacity: 100.0,
            cost_per_unit: 1.0,
            loss_factor: 0.0,
        });
        solver.add_arc(NetworkArc {
            from_node: 2,
            to_node: 3,
            carrier: 1,
            capacity: 200.0,
            cost_per_unit: 0.5,
            loss_factor: 0.0,
        });

        let result = solver.solve().unwrap();
        assert!(
            result.unmet_demand[0] < 1e-6,
            "Electricity unmet: {:.4}",
            result.unmet_demand[0]
        );
        assert!(
            result.unmet_demand[1] < 1e-6,
            "Gas unmet: {:.4}",
            result.unmet_demand[1]
        );
        assert_eq!(result.arc_flows.len(), 2, "Should have 2 arc flows");
    }

    #[test]
    fn test_coupling_electricity_to_heat() {
        // Electricity surplus → heat via coupling (CHP or heat pump)
        let config = EnergyFlowConfig {
            n_nodes: 2,
            commodities: vec![
                EnergyCarrierFlow::Electricity { base_mva: 100.0 },
                EnergyCarrierFlow::Heat {
                    temperature_c: 90.0,
                },
            ],
            coupling_constraints: vec![CouplingConstraint {
                source_carrier: 0, // electricity
                dest_carrier: 1,   // heat
                node: 0,
                conversion_efficiency: 0.9,
                max_conversion_rate: 50.0,
                bidirectional: false,
            }],
            optimization_objective: FlowObjective::MinimizeCost,
        };

        let mut solver = MultiCommodityFlowSolver::new(config);

        // Node 0: 100 MW electricity supply; 40 MW heat demand
        solver.add_node(NetworkNode {
            id: 0,
            name: "Hub".to_string(),
            node_type: NodeType::Hub,
            supply: vec![100.0, 0.0],
            demand: vec![0.0, 40.0],
            storage: vec![0.0, 0.0],
        });
        // Node 1: just a transit (no supply/demand)
        solver.add_node(NetworkNode {
            id: 1,
            name: "Transit".to_string(),
            node_type: NodeType::Transit,
            supply: vec![0.0, 0.0],
            demand: vec![0.0, 0.0],
            storage: vec![0.0, 0.0],
        });

        let result = solver.solve().unwrap();

        // Heat unmet demand should be zero (coupling supplies heat from electricity)
        assert!(
            result.unmet_demand[1] < 1e-6,
            "Heat demand should be met via coupling: {:.4}",
            result.unmet_demand[1]
        );
        // Coupling utilisation should be non-zero
        assert!(
            result.coupling_utilization[0] > 0.0,
            "Coupling should be utilised"
        );
    }

    #[test]
    fn test_capacity_limit_bounds_flow() {
        let config = elec_config();
        let mut solver = MultiCommodityFlowSolver::new(config);

        solver.add_node(make_node(0, 200.0, 0.0, 1));
        solver.add_node(make_node(1, 0.0, 200.0, 1));
        // Arc limited to 80 MW
        solver.add_arc(NetworkArc {
            from_node: 0,
            to_node: 1,
            carrier: 0,
            capacity: 80.0,
            cost_per_unit: 1.0,
            loss_factor: 0.0,
        });

        let result = solver.solve().unwrap();

        let total_flow: f64 = result.arc_flows.iter().map(|&(_, _, _, f)| f).sum();
        assert!(
            total_flow <= 80.0 + 1e-6,
            "Flow should be bounded by arc capacity: {total_flow:.4}"
        );
        assert!(
            result.unmet_demand[0] > 1e-6,
            "There should be unmet demand when capacity is insufficient"
        );
    }

    #[test]
    fn test_unmet_demand_when_no_supply() {
        let config = elec_config();
        let mut solver = MultiCommodityFlowSolver::new(config);

        // No supply, only demand
        solver.add_node(make_node(0, 0.0, 100.0, 1));
        solver.add_node(make_node(1, 0.0, 50.0, 1));

        let result = solver.solve().unwrap();

        assert!(
            result.unmet_demand[0] > 1.0,
            "All demand should be unmet when there is no supply"
        );
    }

    #[test]
    fn test_no_nodes_returns_error() {
        let config = elec_config();
        let solver = MultiCommodityFlowSolver::new(config);
        let result = solver.solve();
        assert!(result.is_err(), "Empty node set should return error");
    }

    #[test]
    fn test_losses_accumulated() {
        let config = elec_config();
        let mut solver = MultiCommodityFlowSolver::new(config);

        solver.add_node(make_node(0, 100.0, 0.0, 1));
        solver.add_node(make_node(1, 0.0, 100.0, 1));
        solver.add_arc(NetworkArc {
            from_node: 0,
            to_node: 1,
            carrier: 0,
            capacity: 200.0,
            cost_per_unit: 1.0,
            loss_factor: 0.05, // 5% losses
        });

        let result = solver.solve().unwrap();
        assert!(
            result.total_losses > 0.0,
            "Losses should be positive with loss_factor > 0"
        );
    }

    #[test]
    fn test_multi_hop_routing() {
        // Source → hub → sink via two arcs
        let config = elec_config();
        let mut solver = MultiCommodityFlowSolver::new(config);

        solver.add_node(make_node(0, 60.0, 0.0, 1));
        solver.add_node(NetworkNode {
            id: 1,
            name: "Hub".to_string(),
            node_type: NodeType::Transit,
            supply: vec![0.0],
            demand: vec![0.0],
            storage: vec![0.0],
        });
        solver.add_node(make_node(2, 0.0, 60.0, 1));

        solver.add_arc(NetworkArc {
            from_node: 0,
            to_node: 1,
            carrier: 0,
            capacity: 100.0,
            cost_per_unit: 1.0,
            loss_factor: 0.0,
        });
        solver.add_arc(NetworkArc {
            from_node: 1,
            to_node: 2,
            carrier: 0,
            capacity: 100.0,
            cost_per_unit: 1.0,
            loss_factor: 0.0,
        });

        let result = solver.solve().unwrap();
        assert!(
            result.unmet_demand[0] < 1e-6,
            "Multi-hop: demand should be met: {:.4}",
            result.unmet_demand[0]
        );
    }
}
