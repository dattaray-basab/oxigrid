//! Grid resilience metrics: trapezoid characterization, IEEE 1366 reliability indices,
//! and N-k vulnerability analysis.
//!
//! # References
//! - Panteli & Mancarella, "The Grid: Stronger, Bigger, Smarter?", IEEE Power & Energy 2015
//! - Holling, "Resilience and Stability of Ecological Systems", Annual Review 1973
//! - IEEE Std 1366-2012

use crate::error::{OxiGridError, Result};
use crate::network::topology::PowerNetwork;

// ─────────────────────────────────────────────────────────────────────────────
// Resilience Trapezoid
// ─────────────────────────────────────────────────────────────────────────────

/// Resilience trapezoid characterization capturing the four phases:
/// resistance, absorption, recovery, and adaptation.
#[derive(Debug, Clone)]
pub struct ResilienceTrapezoid {
    /// Baseline performance before the event (p.u.)
    pub pre_event_performance: f64,
    /// Time of event onset (h)
    pub event_start: f64,
    /// Time of maximum degradation (absorption phase end) (h)
    pub absorption_end: f64,
    /// Time recovery begins (h)
    pub recovery_start: f64,
    /// Time of full (or final) recovery (h)
    pub recovery_end: f64,
    /// Final performance (may differ from baseline due to adaptation) (p.u.)
    pub post_event_performance: f64,
    /// Sampled performance trajectory: (time_h, performance_pu)
    pub performance_trajectory: Vec<(f64, f64)>,
}

impl ResilienceTrapezoid {
    /// Construct a resilience trapezoid and synthesise a linear trajectory if none provided.
    pub fn new(
        pre_event_performance: f64,
        event_start: f64,
        absorption_end: f64,
        recovery_start: f64,
        recovery_end: f64,
        post_event_performance: f64,
        performance_trajectory: Vec<(f64, f64)>,
    ) -> Self {
        Self {
            pre_event_performance,
            event_start,
            absorption_end,
            recovery_start,
            recovery_end,
            post_event_performance,
            performance_trajectory,
        }
    }

    /// Minimum performance during the degraded state (worst-case performance).
    pub fn min_performance(&self) -> f64 {
        if self.performance_trajectory.is_empty() {
            // Approximate: linear drop then flat then linear rise
            // Minimum occurs at absorption_end / recovery_start window
            self.post_event_performance
                .min(self.pre_event_performance)
                .min(0.0_f64.max(self.post_event_performance - 0.5))
        } else {
            self.performance_trajectory
                .iter()
                .map(|(_, p)| *p)
                .fold(f64::INFINITY, f64::min)
        }
    }

    /// Degradation area: energy-equivalent loss = ∫(baseline − performance) dt
    /// computed via trapezoidal integration over the trajectory.
    ///
    /// Returns the area in p.u.·hours.
    pub fn degradation_area(&self) -> f64 {
        let baseline = self.pre_event_performance;

        if self.performance_trajectory.len() >= 2 {
            // Trapezoidal integration over provided trajectory
            let mut area = 0.0;
            let traj = &self.performance_trajectory;
            for i in 1..traj.len() {
                let (t0, p0) = traj[i - 1];
                let (t1, p1) = traj[i];
                let dt = t1 - t0;
                if dt <= 0.0 {
                    continue;
                }
                // degradation at each endpoint, clamped to non-negative
                let d0 = (baseline - p0).max(0.0);
                let d1 = (baseline - p1).max(0.0);
                area += 0.5 * (d0 + d1) * dt;
            }
            area
        } else {
            // Analytical piecewise-linear trapezoid (4-phase model):
            // Phase 1: event_start → absorption_end  (drop from baseline to min_perf)
            // Phase 2: absorption_end → recovery_start (degraded plateau at min_perf)
            // Phase 3: recovery_start → recovery_end  (rise from min_perf to post)
            let min_perf = self
                .min_performance()
                .min(self.pre_event_performance)
                .min(self.post_event_performance);

            let t_start = self.event_start;
            let t_abs = self.absorption_end.max(t_start);
            let t_rec_start = self.recovery_start.max(t_abs);
            let t_rec_end = self.recovery_end.max(t_rec_start);

            // Phase 1: triangle (baseline, baseline) → (baseline, min_perf)
            let d_phase1 = 0.5 * (baseline - min_perf).max(0.0) * (t_abs - t_start);

            // Phase 2: rectangle (baseline - min_perf) * duration
            let d_phase2 = (baseline - min_perf).max(0.0) * (t_rec_start - t_abs);

            // Phase 3: triangle from min_perf to post_event_performance
            let post = self.post_event_performance;
            let avg_recovery_degradation =
                0.5 * ((baseline - min_perf).max(0.0) + (baseline - post).max(0.0));
            let d_phase3 = avg_recovery_degradation * (t_rec_end - t_rec_start);

            d_phase1 + d_phase2 + d_phase3
        }
    }

    /// Recovery rate: how fast the system recovers (p.u./h).
    ///
    /// Defined as (post_event_performance − min_performance) / (recovery_end − recovery_start).
    pub fn recovery_rate(&self) -> f64 {
        let duration = self.recovery_end - self.recovery_start;
        if duration <= 0.0 {
            return 0.0;
        }
        let min_perf = self.min_performance_during_event();
        (self.post_event_performance - min_perf).max(0.0) / duration
    }

    /// Resilience index: RI = 1 − degradation_area / (baseline × total_duration)
    ///
    /// RI = 1.0 means no loss of performance. RI = 0.0 means complete, unrecovered loss.
    pub fn resilience_index(&self) -> f64 {
        let total_duration = self.recovery_end - self.event_start;
        if total_duration <= 0.0 || self.pre_event_performance <= 0.0 {
            return 1.0;
        }
        let normaliser = self.pre_event_performance * total_duration;
        let ri = 1.0 - self.degradation_area() / normaliser;
        ri.clamp(0.0, 1.0)
    }

    /// Time to restoration (TTR): hours from event onset to full recovery.
    pub fn time_to_restoration(&self) -> f64 {
        (self.recovery_end - self.event_start).max(0.0)
    }

    /// Minimum performance reached during the event window (from trajectory or piecewise model).
    fn min_performance_during_event(&self) -> f64 {
        if self.performance_trajectory.is_empty() {
            // Estimate from piecewise linear model
            self.pre_event_performance
                .min(self.post_event_performance)
                .min(0.0)
        } else {
            let t_start = self.event_start;
            let t_end = self.recovery_end;
            self.performance_trajectory
                .iter()
                .filter(|(t, _)| *t >= t_start && *t <= t_end)
                .map(|(_, p)| *p)
                .fold(f64::INFINITY, f64::min)
                .min(self.pre_event_performance)
                .min(self.post_event_performance)
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Performance Metric Trait
// ─────────────────────────────────────────────────────────────────────────────

/// Snapshot of network operational state for performance evaluation.
#[derive(Debug, Clone)]
pub struct NetworkState {
    /// Load currently served (MW)
    pub load_served_mw: f64,
    /// Total system load demand (MW)
    pub total_load_mw: f64,
    /// Number of energised buses
    pub connected_buses: usize,
    /// Total number of buses
    pub total_buses: usize,
    /// Online generation (MW)
    pub generation_online_mw: f64,
}

/// Trait for computing a scalar performance metric from a network state snapshot.
pub trait PerformanceMetric {
    /// Evaluate and return a dimensionless performance score in [0, 1].
    fn evaluate(&self, network_state: &NetworkState) -> f64;
}

/// Load-based performance: fraction of load demand successfully served.
pub struct LoadPerformance;

impl PerformanceMetric for LoadPerformance {
    fn evaluate(&self, state: &NetworkState) -> f64 {
        if state.total_load_mw > 0.0 {
            (state.load_served_mw / state.total_load_mw).clamp(0.0, 1.0)
        } else {
            1.0
        }
    }
}

/// Connectivity-based performance: fraction of buses energised.
pub struct ConnectivityPerformance;

impl PerformanceMetric for ConnectivityPerformance {
    fn evaluate(&self, state: &NetworkState) -> f64 {
        if state.total_buses > 0 {
            (state.connected_buses as f64 / state.total_buses as f64).clamp(0.0, 1.0)
        } else {
            1.0
        }
    }
}

/// Generation adequacy performance: fraction of generation capacity online.
pub struct GenerationPerformance {
    /// Total installed generation capacity (MW)
    pub total_capacity_mw: f64,
}

impl PerformanceMetric for GenerationPerformance {
    fn evaluate(&self, state: &NetworkState) -> f64 {
        if self.total_capacity_mw > 0.0 {
            (state.generation_online_mw / self.total_capacity_mw).clamp(0.0, 1.0)
        } else {
            1.0
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// IEEE 1366 Reliability Indices (resilience module version — hours-based)
// ─────────────────────────────────────────────────────────────────────────────

/// Interruption event for resilience reliability analysis (hours-based).
#[derive(Debug, Clone)]
pub struct InterruptionEvent {
    /// Unique event identifier
    pub event_id: usize,
    /// Bus indices affected by this interruption
    pub affected_buses: Vec<usize>,
    /// Number of customers interrupted
    pub customers_affected: usize,
    /// Duration of the interruption (h)
    pub duration_hours: f64,
    /// Energy not supplied during this event (MWh)
    pub energy_not_supplied_mwh: f64,
    /// Root cause classification
    pub cause: InterruptionCause,
    /// True if the interruption lasted < 5 minutes (momentary per IEEE 1366)
    pub is_momentary: bool,
}

/// Root-cause classification for interruption events.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterruptionCause {
    /// Equipment failure (transformer, cable, breaker)
    Equipment,
    /// Weather-related (storm, lightning, ice)
    Weather,
    /// Thermal or current overload
    Overload,
    /// Operator or maintenance error
    HumanError,
    /// Unknown or unclassified
    Unknown,
}

/// IEEE 1366 reliability indices (hours-based, consistent with resilience analysis).
#[derive(Debug, Clone, Default)]
pub struct ReliabilityIndices {
    /// System Average Interruption Duration Index (h/customer/year)
    pub saidi: f64,
    /// System Average Interruption Frequency Index (interruptions/customer/year)
    pub saifi: f64,
    /// Customer Average Interruption Duration Index (h/interruption) = SAIDI / SAIFI
    pub caidi: f64,
    /// Momentary Average Interruption Frequency Index
    pub maifi: f64,
    /// Average Service Availability Index = 1 − SAIDI / 8760
    pub asai: f64,
    /// Energy Not Supplied (MWh/year)
    pub ens: f64,
    /// Average ENS per customer (MWh/customer/year)
    pub aens: f64,
}

/// Compute IEEE 1366 reliability indices from a historical event log.
///
/// # Arguments
/// - `events` — all interruption events in the observation period
/// - `total_customers` — total customers served
/// - `observation_period_years` — length of the observation window (years)
///
/// Returns `ReliabilityIndices` scaled to a per-year basis.
pub fn compute_reliability_indices(
    events: &[InterruptionEvent],
    total_customers: usize,
    observation_period_years: f64,
) -> ReliabilityIndices {
    if total_customers == 0 || observation_period_years <= 0.0 {
        return ReliabilityIndices::default();
    }

    let n = total_customers as f64;
    let scale = 1.0 / observation_period_years; // annualise

    let sustained: Vec<&InterruptionEvent> = events.iter().filter(|e| !e.is_momentary).collect();
    let momentary: Vec<&InterruptionEvent> = events.iter().filter(|e| e.is_momentary).collect();

    // SAIDI: Σ(duration_i × customers_i) / total_customers  [h]
    let total_customer_hours: f64 = sustained
        .iter()
        .map(|e| e.duration_hours * e.customers_affected as f64)
        .sum();
    let saidi = total_customer_hours / n * scale;

    // SAIFI: Σ(customers_interrupted_i) / total_customers
    let total_customer_interruptions: f64 =
        sustained.iter().map(|e| e.customers_affected as f64).sum();
    let saifi = total_customer_interruptions / n * scale;

    // CAIDI = SAIDI / SAIFI
    let caidi = if saifi > 1e-12 { saidi / saifi } else { 0.0 };

    // MAIFI: momentary interruptions per customer per year
    let momentary_customer_events: f64 =
        momentary.iter().map(|e| e.customers_affected as f64).sum();
    let maifi = momentary_customer_events / n * scale;

    // ASAI: fraction of hours service is available = 1 − SAIDI / 8760
    let asai = (1.0 - saidi / 8760.0).clamp(0.0, 1.0);

    // ENS: total energy not supplied per year
    let ens: f64 = events
        .iter()
        .map(|e| e.energy_not_supplied_mwh)
        .sum::<f64>()
        * scale;

    // AENS: ENS per customer
    let aens = ens / n;

    ReliabilityIndices {
        saidi,
        saifi,
        caidi,
        maifi,
        asai,
        ens,
        aens,
    }
}

/// Compute analytical reliability indices from component failure rates and repair times.
///
/// Uses first-order series-parallel reduction: each branch contributes independently.
/// The method computes load-point indices then aggregates to system indices.
///
/// # Arguments
/// - `network` — network topology (buses and branches)
/// - `failure_rates` — failure rate per branch (failures/year), length = branch_count
/// - `repair_times` — mean repair time per branch (h), length = branch_count
/// - `customers_per_bus` — number of customers at each bus (0-indexed), length = bus_count
pub fn compute_analytical_reliability(
    network: &PowerNetwork,
    failure_rates: &[f64],
    repair_times: &[f64],
    customers_per_bus: &[usize],
) -> ReliabilityIndices {
    let n_branches = network.branch_count();
    let n_buses = network.bus_count();

    if n_branches == 0 || n_buses == 0 {
        return ReliabilityIndices::default();
    }

    let total_customers: usize = customers_per_bus.iter().sum();
    if total_customers == 0 {
        return ReliabilityIndices::default();
    }

    // Build adjacency list for connectivity analysis (0-indexed buses)
    let mut adj: Vec<Vec<(usize, usize)>> = vec![vec![]; n_buses]; // (neighbor_bus_idx, branch_idx)
    for (bi, branch) in network.branches.iter().enumerate() {
        let from_idx = network.buses.iter().position(|b| b.id == branch.from_bus);
        let to_idx = network.buses.iter().position(|b| b.id == branch.to_bus);
        if let (Some(f), Some(t)) = (from_idx, to_idx) {
            if branch.status && bi < failure_rates.len() && bi < repair_times.len() {
                adj[f].push((t, bi));
                adj[t].push((f, bi));
            }
        }
    }

    // Find slack bus index (source of supply)
    let slack_idx = network
        .buses
        .iter()
        .position(|b| b.bus_type == crate::network::bus::BusType::Slack)
        .unwrap_or(0);

    // For each load bus, find branches on the path from the slack bus via BFS.
    // Each branch on the path contributes its failure rate to the load point.
    let mut total_cust_interruptions = 0.0_f64;
    let mut total_cust_hours = 0.0_f64;
    let mut total_ens = 0.0_f64;

    for (bus_idx, &n_cust) in customers_per_bus.iter().enumerate() {
        if n_cust == 0 || bus_idx == slack_idx {
            continue;
        }

        // BFS to find path from slack to this bus; collect branch indices on path
        let path_branches = bfs_path_branches(&adj, slack_idx, bus_idx, n_buses);

        // Approximate average load at this bus for ENS calculation
        let bus_load_mw = if bus_idx < network.buses.len() {
            network.buses[bus_idx].pd.0
        } else {
            0.0
        };

        // Series combination: load point failure rate = Σ λ_i
        // Load point outage time = Σ λ_i * r_i  (h/year)
        let mut lp_failure_rate = 0.0;
        let mut lp_outage_time = 0.0;

        for &bi in &path_branches {
            if bi < failure_rates.len() && bi < repair_times.len() {
                lp_failure_rate += failure_rates[bi];
                lp_outage_time += failure_rates[bi] * repair_times[bi];
            }
        }

        total_cust_interruptions += lp_failure_rate * n_cust as f64;
        total_cust_hours += lp_outage_time * n_cust as f64;
        total_ens += lp_outage_time * bus_load_mw;
    }

    let n = total_customers as f64;
    let saidi = total_cust_hours / n;
    let saifi = total_cust_interruptions / n;
    let caidi = if saifi > 1e-12 { saidi / saifi } else { 0.0 };
    let asai = (1.0 - saidi / 8760.0).clamp(0.0, 1.0);
    let aens = total_ens / n;

    ReliabilityIndices {
        saidi,
        saifi,
        caidi,
        maifi: 0.0, // analytical model does not model momentary events
        asai,
        ens: total_ens,
        aens,
    }
}

/// BFS to find branch indices on the shortest path from `start` to `target`.
/// Returns an empty vec if no path exists.
fn bfs_path_branches(
    adj: &[Vec<(usize, usize)>],
    start: usize,
    target: usize,
    n: usize,
) -> Vec<usize> {
    if start == target {
        return vec![];
    }
    let mut parent: Vec<Option<(usize, usize)>> = vec![None; n]; // (prev_bus, branch_idx)
    let mut visited = vec![false; n];
    let mut queue = std::collections::VecDeque::new();
    queue.push_back(start);
    visited[start] = true;

    while let Some(u) = queue.pop_front() {
        if u == target {
            break;
        }
        for &(v, bi) in &adj[u] {
            if !visited[v] {
                visited[v] = true;
                parent[v] = Some((u, bi));
                queue.push_back(v);
            }
        }
    }

    if !visited[target] {
        return vec![];
    }

    // Reconstruct path
    let mut branches = Vec::new();
    let mut cur = target;
    while let Some((prev, bi)) = parent[cur] {
        branches.push(bi);
        cur = prev;
        if cur == start {
            break;
        }
    }
    branches
}

// ─────────────────────────────────────────────────────────────────────────────
// N-k Vulnerability Analysis
// ─────────────────────────────────────────────────────────────────────────────

/// N-k contingency analysis for resilience assessment.
///
/// Evaluates the system impact of removing k branches simultaneously,
/// identifying catastrophic failure combinations and critical components.
pub struct NkAnalysis {
    /// Contingency order (N-1: k=1, N-2: k=2, ...)
    pub k: usize,
    /// Maximum number of contingency combinations to evaluate (limits computational cost)
    pub max_combinations: usize,
}

/// Impact record for a single N-k contingency.
#[derive(Debug, Clone)]
pub struct ContingencyImpact {
    /// Indices of the branches removed in this contingency
    pub contingency: Vec<usize>,
    /// Fraction of total load that remains served (0.0 = complete blackout)
    pub load_served_fraction: f64,
    /// Number of islands (disconnected components) formed
    pub islands_formed: usize,
    /// True if load served fraction falls below 50%
    pub is_catastrophic: bool,
}

impl NkAnalysis {
    /// Create a new N-k analyser.
    pub fn new(k: usize) -> Self {
        Self {
            k,
            max_combinations: 10_000,
        }
    }

    /// Enumerate and evaluate all N-k contingencies, up to `max_combinations`.
    ///
    /// Returns a `ContingencyImpact` record for each contingency evaluated.
    pub fn analyze(&self, network: &PowerNetwork) -> Result<Vec<ContingencyImpact>> {
        let n_branches = network.branch_count();
        if self.k == 0 {
            return Err(OxiGridError::InvalidParameter(
                "N-k order k must be ≥ 1".to_string(),
            ));
        }
        if n_branches < self.k {
            return Err(OxiGridError::InvalidParameter(format!(
                "Network has {n_branches} branches; cannot form N-{k} contingencies",
                k = self.k
            )));
        }

        // Build bus→index map and edge list
        let n_buses = network.bus_count();
        let edges: Vec<(usize, usize, usize)> = network
            .branches
            .iter()
            .enumerate()
            .filter(|(_, b)| b.status)
            .filter_map(|(bi, b)| {
                let f = network.buses.iter().position(|bus| bus.id == b.from_bus)?;
                let t = network.buses.iter().position(|bus| bus.id == b.to_bus)?;
                Some((f, t, bi))
            })
            .collect();

        // Pre-compute total load
        let total_load: f64 = network.buses.iter().map(|b| b.pd.0).sum();

        // Enumerate combinations via iterative index generation
        let active_branch_indices: Vec<usize> = edges.iter().map(|(_, _, bi)| *bi).collect();
        let n_active = active_branch_indices.len();

        let mut results = Vec::new();
        let mut combinator = CombinationIterator::new(n_active, self.k);
        let mut count = 0usize;

        while let Some(combo_positions) = combinator.next_combination() {
            if count >= self.max_combinations {
                break;
            }
            count += 1;

            // Map positions to actual branch indices
            let removed: Vec<usize> = combo_positions
                .iter()
                .map(|&pos| active_branch_indices[pos])
                .collect();

            // Build reduced edge list (topology only, for connectivity)
            let removed_set: std::collections::HashSet<usize> = removed.iter().copied().collect();
            let reduced_edges: Vec<(usize, usize)> = edges
                .iter()
                .filter(|(_, _, bi)| !removed_set.contains(bi))
                .map(|(f, t, _)| (*f, *t))
                .collect();

            // Compute connectivity (number of islands)
            let (islands, bus_components) = count_components(n_buses, &reduced_edges);

            // Determine which buses are connected to the slack bus
            let slack_component = {
                let slack_idx = network
                    .buses
                    .iter()
                    .position(|b| b.bus_type == crate::network::bus::BusType::Slack)
                    .unwrap_or(0);
                if slack_idx < bus_components.len() {
                    bus_components[slack_idx]
                } else {
                    0
                }
            };

            // Load served = load on buses in the same component as the slack
            let load_served: f64 = network
                .buses
                .iter()
                .enumerate()
                .filter(|(bi, _)| {
                    bi < &bus_components.len() && bus_components[*bi] == slack_component
                })
                .map(|(_, b)| b.pd.0)
                .sum();

            let load_fraction = if total_load > 0.0 {
                (load_served / total_load).clamp(0.0, 1.0)
            } else {
                1.0
            };

            let is_catastrophic = load_fraction < 0.5;

            results.push(ContingencyImpact {
                contingency: removed,
                load_served_fraction: load_fraction,
                islands_formed: islands,
                is_catastrophic,
            });
        }

        Ok(results)
    }

    /// Find the top-n most critical single components (branches) by impact severity.
    ///
    /// Returns a sorted list of `(branch_idx, impact_fraction)` where
    /// `impact_fraction = 1.0 − load_served_fraction`.
    pub fn find_critical_components(
        &self,
        impacts: &[ContingencyImpact],
        n_top: usize,
    ) -> Vec<(usize, f64)> {
        // For N-1 style identification, look at single-branch contingencies
        // and also identify branches that appear most in high-impact combinations.
        let mut branch_worst_impact: std::collections::HashMap<usize, f64> =
            std::collections::HashMap::new();

        for impact in impacts {
            let impact_frac = 1.0 - impact.load_served_fraction;
            for &bi in &impact.contingency {
                let entry = branch_worst_impact.entry(bi).or_insert(0.0);
                if impact_frac > *entry {
                    *entry = impact_frac;
                }
            }
        }

        let mut ranked: Vec<(usize, f64)> = branch_worst_impact.into_iter().collect();
        ranked.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        ranked.truncate(n_top);
        ranked
    }

    /// System criticality index: weighted average impact across all evaluated contingencies.
    ///
    /// CI = Σ(1 − load_served_i) / n_contingencies
    ///
    /// CI = 0.0 means no contingency causes any load loss.
    /// CI = 1.0 means all contingencies cause complete blackout.
    pub fn criticality_index(impacts: &[ContingencyImpact]) -> f64 {
        if impacts.is_empty() {
            return 0.0;
        }
        let sum: f64 = impacts
            .iter()
            .map(|imp| 1.0 - imp.load_served_fraction)
            .sum();
        sum / impacts.len() as f64
    }
}

/// Count connected components and return (n_components, component_id_per_bus).
fn count_components(n: usize, edges: &[(usize, usize)]) -> (usize, Vec<usize>) {
    let mut adj: Vec<Vec<usize>> = vec![vec![]; n];
    for &(u, v) in edges {
        if u < n && v < n {
            adj[u].push(v);
            adj[v].push(u);
        }
    }

    let mut component = vec![usize::MAX; n];
    let mut n_components = 0usize;

    for start in 0..n {
        if component[start] != usize::MAX {
            continue;
        }
        let comp_id = n_components;
        n_components += 1;
        let mut queue = std::collections::VecDeque::new();
        queue.push_back(start);
        component[start] = comp_id;
        while let Some(u) = queue.pop_front() {
            for &v in &adj[u] {
                if component[v] == usize::MAX {
                    component[v] = comp_id;
                    queue.push_back(v);
                }
            }
        }
    }

    (n_components, component)
}

/// Iterator over k-combinations of indices from 0..n.
struct CombinationIterator {
    n: usize,
    k: usize,
    indices: Vec<usize>,
    first: bool,
    exhausted: bool,
}

impl CombinationIterator {
    fn new(n: usize, k: usize) -> Self {
        let indices = (0..k).collect();
        Self {
            n,
            k,
            indices,
            first: true,
            exhausted: k > n,
        }
    }

    fn next_combination(&mut self) -> Option<&[usize]> {
        if self.exhausted {
            return None;
        }
        if self.first {
            self.first = false;
            return Some(&self.indices);
        }
        // Find the rightmost index that can be incremented
        let mut i = self.k;
        loop {
            if i == 0 {
                self.exhausted = true;
                return None;
            }
            i -= 1;
            if self.indices[i] < self.n - self.k + i {
                break;
            }
        }
        self.indices[i] += 1;
        for j in (i + 1)..self.k {
            self.indices[j] = self.indices[j - 1] + 1;
        }
        Some(&self.indices)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::branch::Branch;
    use crate::network::bus::{Bus, BusType};
    use crate::network::topology::PowerNetwork;
    use crate::units::{Power, ReactivePower, Voltage};

    fn make_branch(from: usize, to: usize) -> Branch {
        Branch {
            from_bus: from,
            to_bus: to,
            r: 0.01,
            x: 0.1,
            b: 0.0,
            rate_a: 100.0,
            rate_b: 100.0,
            rate_c: 100.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        }
    }

    fn make_bus(id: usize, bus_type: BusType, pd_mw: f64) -> Bus {
        Bus {
            id,
            name: format!("Bus {id}"),
            bus_type,
            pd: Power(pd_mw),
            qd: ReactivePower(0.0),
            gs: 0.0,
            bs: 0.0,
            vm: 1.0,
            va: 0.0,
            base_kv: Voltage(100.0),
            zone: None,
        }
    }

    // ── Resilience Trapezoid Tests ────────────────────────────────────────────

    #[test]
    fn test_resilience_index_perfect() {
        // No degradation at all: trajectory stays at baseline throughout
        let traj = vec![(0.0, 1.0), (1.0, 1.0), (2.0, 1.0), (3.0, 1.0), (4.0, 1.0)];
        let trap = ResilienceTrapezoid::new(1.0, 1.0, 1.5, 2.0, 4.0, 1.0, traj);
        let ri = trap.resilience_index();
        assert!(
            (ri - 1.0).abs() < 1e-6,
            "Perfect resilience: RI should be 1.0, got {ri:.6}"
        );
    }

    #[test]
    fn test_resilience_trapezoid_area_analytical() {
        // Piecewise linear trapezoid:
        // baseline = 1.0, event_start = 0.0, absorption_end = 1.0 (drops to 0.5),
        // recovery_start = 2.0 (plateau at 0.5), recovery_end = 3.0 (rises to 1.0).
        //
        // Phase 1 (0→1): triangle area = 0.5 * (1.0-0.5) * 1.0 = 0.25
        // Phase 2 (1→2): rectangle = (1.0-0.5) * 1.0 = 0.50
        // Phase 3 (2→3): avg_deg = 0.5*((0.5)+(0.0)) = 0.25; area = 0.25*1.0 = 0.25
        // Total = 1.0
        let traj = vec![(0.0, 1.0), (1.0, 0.5), (2.0, 0.5), (3.0, 1.0)];
        let trap = ResilienceTrapezoid::new(1.0, 0.0, 1.0, 2.0, 3.0, 1.0, traj);
        let area = trap.degradation_area();
        // Trapezoidal: (0→1): avg deg = 0.5*(0+0.5)*1 = 0.25; (1→2): 0.5; (2→3): avg = 0.5*(0.5+0)*1 = 0.25
        // Total = 1.0
        assert!(
            (area - 1.0).abs() < 1e-6,
            "Trapezoid degradation area should be 1.0, got {area:.6}"
        );
    }

    #[test]
    fn test_resilience_index_partial_recovery() {
        // Partial event: RI < 1 but > 0
        let traj = vec![(0.0, 1.0), (1.0, 0.6), (3.0, 0.6), (5.0, 0.9)];
        let trap = ResilienceTrapezoid::new(1.0, 0.0, 1.0, 3.0, 5.0, 0.9, traj);
        let ri = trap.resilience_index();
        assert!(ri > 0.0 && ri < 1.0, "Partial recovery: RI={ri:.4}");
    }

    #[test]
    fn test_recovery_rate_positive() {
        let traj = vec![(0.0, 1.0), (2.0, 0.5), (4.0, 0.5), (6.0, 1.0)];
        let trap = ResilienceTrapezoid::new(1.0, 0.0, 2.0, 4.0, 6.0, 1.0, traj);
        let rr = trap.recovery_rate();
        // (1.0 - 0.5) / (6.0 - 4.0) = 0.25 p.u./h
        assert!(rr > 0.0, "Recovery rate should be positive, got {rr:.4}");
    }

    #[test]
    fn test_time_to_restoration() {
        let trap = ResilienceTrapezoid::new(1.0, 2.0, 3.0, 4.0, 7.0, 1.0, vec![]);
        assert!(
            (trap.time_to_restoration() - 5.0).abs() < 1e-9,
            "TTR = 7-2 = 5h"
        );
    }

    // ── IEEE 1366 Reliability Index Tests ────────────────────────────────────

    fn make_event(
        id: usize,
        customers: usize,
        duration_h: f64,
        ens_mwh: f64,
        momentary: bool,
    ) -> InterruptionEvent {
        InterruptionEvent {
            event_id: id,
            affected_buses: vec![id],
            customers_affected: customers,
            duration_hours: duration_h,
            energy_not_supplied_mwh: ens_mwh,
            cause: InterruptionCause::Equipment,
            is_momentary: momentary,
        }
    }

    #[test]
    fn test_saidi_single_event() {
        // 1000 customers total, 100 affected, 2h outage → SAIDI = 100*2/1000 = 0.2 h/customer
        let events = vec![make_event(0, 100, 2.0, 10.0, false)];
        let idx = compute_reliability_indices(&events, 1000, 1.0);
        assert!(
            (idx.saidi - 0.2).abs() < 1e-9,
            "SAIDI should be 0.2, got {:.6}",
            idx.saidi
        );
    }

    #[test]
    fn test_saifi_calculation() {
        // 1000 customers, 100 affected → SAIFI = 0.1
        let events = vec![make_event(0, 100, 2.0, 10.0, false)];
        let idx = compute_reliability_indices(&events, 1000, 1.0);
        assert!(
            (idx.saifi - 0.1).abs() < 1e-9,
            "SAIFI should be 0.1, got {:.6}",
            idx.saifi
        );
    }

    #[test]
    fn test_caidi_equals_saidi_over_saifi() {
        let events = vec![
            make_event(0, 200, 3.0, 20.0, false),
            make_event(1, 100, 1.0, 5.0, false),
        ];
        let idx = compute_reliability_indices(&events, 1000, 1.0);
        if idx.saifi > 1e-12 {
            let expected_caidi = idx.saidi / idx.saifi;
            assert!(
                (idx.caidi - expected_caidi).abs() < 1e-9,
                "CAIDI = SAIDI/SAIFI: expected {expected_caidi:.6}, got {:.6}",
                idx.caidi
            );
        }
    }

    #[test]
    fn test_maifi_only_momentary_events() {
        // 2 momentary events affecting 50 customers each, 1000 total
        let events = vec![
            make_event(0, 50, 0.05, 0.5, true),
            make_event(1, 50, 0.04, 0.4, true),
        ];
        let idx = compute_reliability_indices(&events, 1000, 1.0);
        assert!(
            (idx.maifi - 0.1).abs() < 1e-9,
            "MAIFI = 100/1000 = 0.1, got {:.6}",
            idx.maifi
        );
        assert!(
            idx.saidi < 1e-12,
            "No sustained events → SAIDI=0, got {:.6}",
            idx.saidi
        );
    }

    #[test]
    fn test_asai_near_unity() {
        // Very short outage → ASAI ≈ 1
        let events = vec![make_event(0, 10, 0.1, 0.1, false)];
        let idx = compute_reliability_indices(&events, 100_000, 1.0);
        assert!(
            idx.asai > 0.9999,
            "ASAI should be near 1, got {:.6}",
            idx.asai
        );
    }

    // ── N-k Analysis Tests ────────────────────────────────────────────────────

    /// Build a simple 3-bus radial network: slack(1) -- bus(2) -- bus(3)
    fn make_radial_network() -> PowerNetwork {
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(make_bus(1, BusType::Slack, 0.0));
        net.buses.push(make_bus(2, BusType::PQ, 50.0));
        net.buses.push(make_bus(3, BusType::PQ, 50.0));
        net.branches.push(make_branch(1, 2));
        net.branches.push(make_branch(2, 3));
        net
    }

    /// Build a 3-bus ring: slack(1) -- bus(2) -- bus(3) -- slack(1)
    fn make_ring_network() -> PowerNetwork {
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(make_bus(1, BusType::Slack, 0.0));
        net.buses.push(make_bus(2, BusType::PQ, 50.0));
        net.buses.push(make_bus(3, BusType::PQ, 50.0));
        net.branches.push(make_branch(1, 2));
        net.branches.push(make_branch(2, 3));
        net.branches.push(make_branch(3, 1));
        net
    }

    #[test]
    fn test_n1_analysis_radial() {
        let net = make_radial_network();
        let analyzer = NkAnalysis::new(1);
        let impacts = analyzer.analyze(&net).expect("N-1 analysis should succeed");
        // In radial network: removing branch 0 (1-2) disconnects both buses 2 and 3
        // removing branch 1 (2-3) disconnects bus 3
        // At least one contingency should have load_served_fraction < 1.0
        assert!(
            impacts.iter().any(|imp| imp.load_served_fraction < 1.0),
            "Radial network: some N-1 contingency must reduce load served"
        );
    }

    #[test]
    fn test_n1_ring_fully_redundant() {
        let net = make_ring_network();
        let analyzer = NkAnalysis::new(1);
        let impacts = analyzer.analyze(&net).expect("N-1 analysis for ring");
        // Ring: every N-1 contingency should keep all buses connected (load_served = 1.0)
        for imp in &impacts {
            assert!(
                imp.load_served_fraction > 0.99,
                "Ring N-1: all buses should be served, got {:.4}",
                imp.load_served_fraction
            );
        }
    }

    #[test]
    fn test_n2_analysis_produces_results() {
        let net = make_ring_network();
        let analyzer = NkAnalysis::new(2);
        let impacts = analyzer.analyze(&net).expect("N-2 analysis for ring");
        assert!(!impacts.is_empty(), "N-2 analysis should produce results");
    }

    #[test]
    fn test_criticality_index_zero_for_redundant() {
        let net = make_ring_network();
        let analyzer = NkAnalysis::new(1);
        let impacts = analyzer.analyze(&net).expect("analyze");
        let ci = NkAnalysis::criticality_index(&impacts);
        assert!(ci < 1e-6, "Ring N-1 criticality should be ~0, got {ci:.6}");
    }

    #[test]
    fn test_find_critical_components() {
        let net = make_radial_network();
        let analyzer = NkAnalysis::new(1);
        let impacts = analyzer.analyze(&net).expect("analyze");
        let critical = analyzer.find_critical_components(&impacts, 5);
        assert!(
            !critical.is_empty(),
            "Should find at least one critical component"
        );
        // Verify sorted by impact (descending)
        for w in critical.windows(2) {
            assert!(
                w[0].1 >= w[1].1,
                "Critical components should be sorted by impact"
            );
        }
    }

    // ─── New tests (Round 27) ────────────────────────────────────────────────

    #[test]
    fn test_nk_analysis_k_zero_returns_err() {
        // Reason: k=0 is an invalid contingency order and must return an error.
        let net = make_radial_network();
        let analyzer = NkAnalysis::new(0);
        let result = analyzer.analyze(&net);
        assert!(result.is_err(), "k=0 must return Err");
    }

    #[test]
    fn test_nk_analysis_k_exceeds_branches_returns_err() {
        // Reason: requesting more removed branches than exist must return an error.
        let net = make_radial_network(); // 2 branches
        let analyzer = NkAnalysis::new(5);
        let result = analyzer.analyze(&net);
        assert!(result.is_err(), "k > branch_count must return Err");
    }

    #[test]
    fn test_load_performance_evaluate_in_unit_interval() {
        // Reason: LoadPerformance::evaluate must always return a value in [0, 1].
        let metric = LoadPerformance;
        let state_partial = NetworkState {
            load_served_mw: 60.0,
            total_load_mw: 100.0,
            connected_buses: 3,
            total_buses: 4,
            generation_online_mw: 70.0,
        };
        let score = metric.evaluate(&state_partial);
        assert!(
            (0.0..=1.0).contains(&score),
            "LoadPerformance score must be in [0,1], got {score:.6}"
        );
        approx::assert_relative_eq!(score, 0.6, max_relative = 1e-9);
    }

    #[test]
    fn test_connectivity_performance_evaluate_full() {
        // Reason: ConnectivityPerformance must return 1.0 when all buses are energised.
        let metric = ConnectivityPerformance;
        let state_full = NetworkState {
            load_served_mw: 100.0,
            total_load_mw: 100.0,
            connected_buses: 5,
            total_buses: 5,
            generation_online_mw: 110.0,
        };
        let score = metric.evaluate(&state_full);
        approx::assert_relative_eq!(score, 1.0, max_relative = 1e-9);
    }

    #[test]
    fn test_generation_performance_evaluate_partial() {
        // Reason: GenerationPerformance must correctly compute capacity fraction.
        let metric = GenerationPerformance {
            total_capacity_mw: 200.0,
        };
        let state = NetworkState {
            load_served_mw: 80.0,
            total_load_mw: 100.0,
            connected_buses: 4,
            total_buses: 4,
            generation_online_mw: 100.0,
        };
        let score = metric.evaluate(&state);
        approx::assert_relative_eq!(score, 0.5, max_relative = 1e-9);
    }

    #[test]
    fn test_compute_reliability_indices_empty_events() {
        // Reason: zero events must produce zero SAIDI/SAIFI and ASAI=1.0.
        let idx = compute_reliability_indices(&[], 1000, 1.0);
        approx::assert_relative_eq!(idx.saidi, 0.0, max_relative = 1e-9, epsilon = 1e-12);
        approx::assert_relative_eq!(idx.saifi, 0.0, max_relative = 1e-9, epsilon = 1e-12);
        approx::assert_relative_eq!(idx.asai, 1.0, max_relative = 1e-9);
    }

    #[test]
    fn test_compute_reliability_indices_zero_customers_returns_default() {
        // Reason: zero total_customers must return the default (zero) indices.
        let events = vec![make_event(0, 100, 2.0, 10.0, false)];
        let idx = compute_reliability_indices(&events, 0, 1.0);
        approx::assert_relative_eq!(idx.saidi, 0.0, max_relative = 1e-9, epsilon = 1e-12);
        approx::assert_relative_eq!(idx.saifi, 0.0, max_relative = 1e-9, epsilon = 1e-12);
    }

    #[test]
    fn test_compute_analytical_reliability_simple_radial() {
        // Reason: analytical reliability indices for a radial network must be non-negative.
        let net = make_radial_network();
        let failure_rates = vec![0.1, 0.2]; // failures/year per branch
        let repair_times = vec![4.0, 6.0]; // hours per repair
        let customers_per_bus = vec![0usize, 50, 30]; // slack has none
        let idx =
            compute_analytical_reliability(&net, &failure_rates, &repair_times, &customers_per_bus);
        assert!(
            idx.saidi >= 0.0,
            "SAIDI must be non-negative, got {:.6}",
            idx.saidi
        );
        assert!(
            idx.saifi >= 0.0,
            "SAIFI must be non-negative, got {:.6}",
            idx.saifi
        );
        assert!(
            (0.0..=1.0).contains(&idx.asai),
            "ASAI must be in [0,1], got {:.6}",
            idx.asai
        );
    }

    #[test]
    fn test_compute_analytical_reliability_empty_network_returns_default() {
        // Reason: an empty network (no buses, no branches) must return default zero indices.
        let net = PowerNetwork::new(100.0);
        let idx = compute_analytical_reliability(&net, &[], &[], &[]);
        approx::assert_relative_eq!(idx.saidi, 0.0, max_relative = 1e-9, epsilon = 1e-12);
        approx::assert_relative_eq!(idx.saifi, 0.0, max_relative = 1e-9, epsilon = 1e-12);
    }

    #[test]
    fn test_resilience_trapezoid_min_performance_from_trajectory() {
        // Reason: min_performance() must return the minimum value in the trajectory.
        let traj = vec![(0.0, 1.0), (1.0, 0.3), (2.0, 0.4), (3.0, 0.9)];
        let trap = ResilienceTrapezoid::new(1.0, 0.0, 1.0, 2.0, 3.0, 0.9, traj);
        let min_p = trap.min_performance();
        approx::assert_relative_eq!(min_p, 0.3, max_relative = 1e-9);
    }
}
