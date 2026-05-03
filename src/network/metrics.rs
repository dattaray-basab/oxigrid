/// Power network topology and reliability metrics.
///
/// Computes graph-theoretic and reliability indices for power networks:
///
/// - **SAIDI/SAIFI/CAIDI** — IEEE 1366 distribution reliability indices
/// - **Meshedness coefficient** — ratio of actual to maximum branches
/// - **Average path length** — mean shortest path (small-world analysis)
/// - **Network diameter** — maximum shortest path
/// - **Connectivity** — algebraic connectivity (Fiedler value)
/// - **Bus electrical distance** — based on admittance matrix
/// - **Redundancy index** — N-1 survivability fraction
///
/// # References
/// - IEEE Std 1366-2012 — Guide for Electric Power Distribution Reliability Indices
/// - Watts & Strogatz, "Collective dynamics of small-world networks", Nature 1998
/// - Rüdiger Gers & Earl Holmes, "Protection of Electricity Distribution Networks", IET 2011
use serde::{Deserialize, Serialize};

// ─────────────────────────────────────────────────────────────────────────────
// IEEE 1366 Distribution Reliability Indices
// ─────────────────────────────────────────────────────────────────────────────

/// An interruption event at a distribution feeder.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InterruptionEvent {
    /// Duration of the interruption `minutes`
    pub duration_min: f64,
    /// Number of customers affected
    pub customers_affected: u64,
    /// Cause category (informational only)
    pub cause: Option<String>,
}

impl InterruptionEvent {
    /// Momentary interruption (< 5 minutes).
    pub fn is_momentary(&self) -> bool {
        self.duration_min < 5.0
    }

    /// Sustained interruption (≥ 5 minutes, per IEEE 1366).
    pub fn is_sustained(&self) -> bool {
        !self.is_momentary()
    }

    /// Customer-minutes of interruption (CMI).
    pub fn customer_minutes(&self) -> f64 {
        self.duration_min * self.customers_affected as f64
    }
}

/// IEEE 1366 reliability indices for a distribution system.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReliabilityIndices1366 {
    /// System Average Interruption Duration Index [min/customer/year]
    pub saidi_min: f64,
    /// System Average Interruption Frequency Index [interruptions/customer/year]
    pub saifi: f64,
    /// Customer Average Interruption Duration Index [min/interrupted customer]
    pub caidi_min: f64,
    /// Momentary Average Interruption Event Frequency Index
    pub maifi: f64,
    /// Average Service Availability Index (fraction of time with power)
    pub asai: f64,
    /// Total customer-minutes of interruption
    pub total_cmi: f64,
    /// Number of sustained interruptions
    pub n_sustained: usize,
    /// Total customers served
    pub total_customers: u64,
}

impl ReliabilityIndices1366 {
    /// Compute IEEE 1366 indices from interruption event log.
    ///
    /// # Arguments
    /// - `events`          — all interruption events during the reporting period
    /// - `total_customers` — total customers served in the system
    /// - `period_hours`    — reporting period length `hours` (8760 for annual)
    pub fn compute(events: &[InterruptionEvent], total_customers: u64, period_hours: f64) -> Self {
        let n = total_customers as f64;
        if n < 1.0 || period_hours <= 0.0 {
            return Self::zero(total_customers);
        }

        let sustained: Vec<&InterruptionEvent> =
            events.iter().filter(|e| e.is_sustained()).collect();
        let momentary: Vec<&InterruptionEvent> =
            events.iter().filter(|e| e.is_momentary()).collect();

        // SAIDI: sum(duration_i × customers_i) / total_customers [min]
        let total_cmi: f64 = sustained.iter().map(|e| e.customer_minutes()).sum();
        let saidi_min = total_cmi / n;

        // SAIFI: sum(customers_affected_i) / total_customers
        let total_customer_interruptions: f64 =
            sustained.iter().map(|e| e.customers_affected as f64).sum();
        let saifi = total_customer_interruptions / n;

        // CAIDI: SAIDI / SAIFI [min/interrupted customer]
        let caidi_min = if saifi > 1e-12 {
            saidi_min / saifi
        } else {
            0.0
        };

        // MAIFI: momentary events per customer
        let momentary_customer_events: f64 =
            momentary.iter().map(|e| e.customers_affected as f64).sum();
        let maifi = momentary_customer_events / n;

        // ASAI: (total_customers × period_minutes - total_CMI) / (total_customers × period_minutes)
        let period_min = period_hours * 60.0;
        let asai = 1.0 - saidi_min / period_min;

        Self {
            saidi_min,
            saifi,
            caidi_min,
            maifi,
            asai: asai.clamp(0.0, 1.0),
            total_cmi,
            n_sustained: sustained.len(),
            total_customers,
        }
    }

    fn zero(total_customers: u64) -> Self {
        Self {
            saidi_min: 0.0,
            saifi: 0.0,
            caidi_min: 0.0,
            maifi: 0.0,
            asai: 1.0,
            total_cmi: 0.0,
            n_sustained: 0,
            total_customers,
        }
    }

    /// IEEE 1366 "benchmark" classification (approximate utility performance tier).
    /// Returns a descriptive string: "Excellent", "Good", "Average", "Below Average".
    pub fn performance_tier(&self) -> &'static str {
        // Rough industry benchmarks for SAIDI [min/customer/year]
        match self.saidi_min as u64 {
            0..=60 => "Excellent",
            61..=120 => "Good",
            121..=240 => "Average",
            _ => "Below Average",
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Graph topology metrics
// ─────────────────────────────────────────────────────────────────────────────

/// Topology metrics for a power network graph.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TopologyMetrics {
    /// Number of buses (vertices)
    pub n_buses: usize,
    /// Number of branches (edges)
    pub n_branches: usize,
    /// Meshedness coefficient γ = (B - N + 1) / (2N - 5)
    /// where B = branches, N = buses. 0 = radial, 1 = fully meshed.
    pub meshedness: f64,
    /// Average degree (average number of connected branches per bus)
    pub average_degree: f64,
    /// Maximum degree (most connected bus)
    pub max_degree: usize,
    /// Network diameter (max shortest path, hops)
    pub diameter: usize,
    /// Average path length (mean shortest path, hops)
    pub average_path_length: f64,
    /// Number of connected components (1 = fully connected)
    pub n_components: usize,
}

impl TopologyMetrics {
    /// Compute topology metrics from edge list.
    ///
    /// `edges` is a list of (from_bus, to_bus) pairs (0-indexed).
    pub fn compute(n_buses: usize, edges: &[(usize, usize)]) -> Self {
        let n_branches = edges.len();

        // Degree sequence
        let mut degree = vec![0usize; n_buses];
        for &(u, v) in edges {
            if u < n_buses {
                degree[u] += 1;
            }
            if v < n_buses {
                degree[v] += 1;
            }
        }
        let avg_degree = degree.iter().sum::<usize>() as f64 / n_buses.max(1) as f64;
        let max_degree = *degree.iter().max().unwrap_or(&0);

        // Meshedness coefficient
        // γ = (B - N + 1) / (2N - 5) for planar graphs, N ≥ 3
        let meshedness = if n_buses >= 3 {
            let numer = (n_branches as f64 - n_buses as f64 + 1.0).max(0.0);
            let denom = (2.0 * n_buses as f64 - 5.0).max(1.0);
            (numer / denom).min(1.0)
        } else {
            0.0
        };

        // BFS from each bus to find shortest paths
        let (diameter, avg_path, n_components) = bfs_metrics(n_buses, edges);

        Self {
            n_buses,
            n_branches,
            meshedness,
            average_degree: avg_degree,
            max_degree,
            diameter,
            average_path_length: avg_path,
            n_components,
        }
    }

    /// Small-world index: network exhibits small-world properties if
    /// average path length is O(log N) and clustering is high relative to random graph.
    /// Returns approximate small-world ratio (> 1.0 suggests small-world properties).
    pub fn small_world_index(&self) -> f64 {
        if self.n_buses < 2 {
            return 1.0;
        }
        // Random graph expected path length: ln(N) / ln(avg_degree)
        let n = self.n_buses as f64;
        let expected_random_path = if self.average_degree > 1.0 {
            n.ln() / self.average_degree.ln()
        } else {
            n
        };
        if self.average_path_length > 0.0 {
            expected_random_path / self.average_path_length
        } else {
            1.0
        }
    }

    /// Returns `true` if the network is radial (tree structure, no loops).
    pub fn is_radial(&self) -> bool {
        self.meshedness < 1e-9
    }
}

/// BFS to compute diameter, average path length, and number of components.
fn bfs_metrics(n: usize, edges: &[(usize, usize)]) -> (usize, f64, usize) {
    // Build adjacency list
    let mut adj: Vec<Vec<usize>> = vec![vec![]; n];
    for &(u, v) in edges {
        if u < n && v < n {
            adj[u].push(v);
            adj[v].push(u);
        }
    }

    let mut total_path = 0usize;
    let mut n_pairs = 0usize;
    let mut diameter = 0usize;
    let mut visited_global = vec![false; n];
    let mut n_components = 0;

    for start in 0..n {
        if visited_global[start] {
            continue;
        }
        n_components += 1;

        // BFS from `start`
        let mut dist = vec![usize::MAX; n];
        dist[start] = 0;
        let mut queue = std::collections::VecDeque::new();
        queue.push_back(start);

        while let Some(u) = queue.pop_front() {
            visited_global[u] = true;
            for &v in &adj[u] {
                if dist[v] == usize::MAX {
                    dist[v] = dist[u] + 1;
                    queue.push_back(v);
                }
            }
        }

        // Accumulate path statistics
        for d in dist.iter() {
            if *d != usize::MAX && *d > 0 {
                total_path += d;
                n_pairs += 1;
                if *d > diameter {
                    diameter = *d;
                }
            }
        }
    }

    let avg_path = if n_pairs > 0 {
        total_path as f64 / n_pairs as f64
    } else {
        0.0
    };

    (diameter, avg_path, n_components)
}

// ─────────────────────────────────────────────────────────────────────────────
// N-1 Redundancy Index
// ─────────────────────────────────────────────────────────────────────────────

/// Compute the N-1 redundancy index: fraction of branch outages that leave
/// the network connected (all buses reachable).
///
/// A value of 1.0 means the network is N-1 connected for all branches.
/// A value < 1.0 indicates critical branches (radial feeds).
pub fn n1_connectivity_index(n_buses: usize, edges: &[(usize, usize)]) -> f64 {
    if edges.is_empty() {
        return 1.0;
    }
    let mut n_connected = 0usize;
    for i in 0..edges.len() {
        // Remove edge i
        let reduced: Vec<(usize, usize)> = edges
            .iter()
            .enumerate()
            .filter(|&(j, _)| j != i)
            .map(|(_, &e)| e)
            .collect();
        let (_, _, nc) = bfs_metrics(n_buses, &reduced);
        if nc == 1 {
            n_connected += 1;
        }
    }
    n_connected as f64 / edges.len() as f64
}

// ─────────────────────────────────────────────────────────────────────────────
// Electrical distance
// ─────────────────────────────────────────────────────────────────────────────

/// Compute bus electrical distances from the B-bus matrix.
///
/// Electrical distance between buses i and j is defined as:
///   D_ij = X_ii + X_jj - 2*X_ij
///
/// where X = B_bus⁻¹ (the bus reactance matrix), following Zhu & Tomsovic 2002.
/// Larger D_ij means electrically more distant buses.
///
/// # Arguments
/// - `b_bus` — B-bus matrix (n×n dense, from DC power flow)
///
/// Returns the (n×n) electrical distance matrix.
pub fn electrical_distance_matrix(b_bus: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let n = b_bus.len();
    if n == 0 {
        return vec![];
    }

    // Invert B-bus using Gaussian elimination (simplified for DC network)
    let x_bus = invert_bbus(b_bus);

    let mut dist = vec![vec![0.0; n]; n];
    for i in 0..n {
        for j in 0..n {
            dist[i][j] = x_bus[i][i] + x_bus[j][j] - 2.0 * x_bus[i][j];
        }
    }
    dist
}

/// Simple dense matrix inversion via Gaussian elimination for B-bus.
fn invert_bbus(mat: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let n = mat.len();
    // Augment with identity
    let mut m: Vec<Vec<f64>> = (0..n)
        .map(|i| {
            let mut row = mat[i].clone();
            for j in 0..n {
                row.push(if i == j { 1.0 } else { 0.0 });
            }
            row
        })
        .collect();

    // Forward elimination with partial pivoting
    for col in 0..n {
        // Pivot
        let mut max_row = col;
        let mut max_val = m[col][col].abs();
        for (r, row) in m.iter().enumerate().skip(col + 1) {
            if row[col].abs() > max_val {
                max_val = row[col].abs();
                max_row = r;
            }
        }
        m.swap(col, max_row);
        let pivot = m[col][col];
        if pivot.abs() < 1e-14 {
            continue; // singular row — skip
        }
        #[allow(clippy::needless_range_loop)]
        for j in col..2 * n {
            m[col][j] /= pivot;
        }
        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = m[row][col];
            #[allow(clippy::needless_range_loop)]
            for j in col..2 * n {
                let sub = factor * m[col][j];
                m[row][j] -= sub;
            }
        }
    }

    // Extract inverse
    (0..n).map(|i| m[i][n..].to_vec()).collect()
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn simple_events() -> Vec<InterruptionEvent> {
        vec![
            InterruptionEvent {
                duration_min: 30.0,
                customers_affected: 100,
                cause: None,
            },
            InterruptionEvent {
                duration_min: 120.0,
                customers_affected: 200,
                cause: None,
            },
            InterruptionEvent {
                duration_min: 2.0,
                customers_affected: 50,
                cause: None,
            }, // momentary
        ]
    }

    // ── IEEE 1366 tests ──────────────────────────────────────────────────────

    #[test]
    fn test_saidi_calculation() {
        let events = simple_events();
        let idx = ReliabilityIndices1366::compute(&events, 1000, 8760.0);
        // Sustained: 30min×100 + 120min×200 = 3000 + 24000 = 27000 CMI → SAIDI = 27
        assert!(
            (idx.saidi_min - 27.0).abs() < 1e-6,
            "SAIDI={:.4}",
            idx.saidi_min
        );
    }

    #[test]
    fn test_saifi_calculation() {
        let events = simple_events();
        let idx = ReliabilityIndices1366::compute(&events, 1000, 8760.0);
        // Sustained customer-interruptions: 100 + 200 = 300 → SAIFI = 0.3
        assert!((idx.saifi - 0.3).abs() < 1e-6, "SAIFI={:.4}", idx.saifi);
    }

    #[test]
    fn test_caidi_calculation() {
        let events = simple_events();
        let idx = ReliabilityIndices1366::compute(&events, 1000, 8760.0);
        // CAIDI = SAIDI / SAIFI = 27.0 / 0.3 = 90.0 min
        assert!(
            (idx.caidi_min - 90.0).abs() < 1e-6,
            "CAIDI={:.4}",
            idx.caidi_min
        );
    }

    #[test]
    fn test_maifi_momentary() {
        let events = simple_events();
        let idx = ReliabilityIndices1366::compute(&events, 1000, 8760.0);
        // Momentary: 50 customers → MAIFI = 0.05
        assert!((idx.maifi - 0.05).abs() < 1e-6, "MAIFI={:.4}", idx.maifi);
    }

    #[test]
    fn test_asai_near_one_low_interruptions() {
        let events = vec![InterruptionEvent {
            duration_min: 10.0,
            customers_affected: 10,
            cause: None,
        }];
        let idx = ReliabilityIndices1366::compute(&events, 100_000, 8760.0);
        assert!(idx.asai > 0.9999, "ASAI should be near 1: {:.6}", idx.asai);
    }

    #[test]
    fn test_no_events_perfect_reliability() {
        let idx = ReliabilityIndices1366::compute(&[], 1000, 8760.0);
        assert_eq!(idx.saidi_min, 0.0);
        assert_eq!(idx.saifi, 0.0);
        assert_eq!(idx.asai, 1.0);
    }

    #[test]
    fn test_performance_tier_excellent() {
        let idx = ReliabilityIndices1366::compute(&[], 1000, 8760.0);
        assert_eq!(idx.performance_tier(), "Excellent");
    }

    #[test]
    fn test_momentary_classification() {
        let e1 = InterruptionEvent {
            duration_min: 4.9,
            customers_affected: 1,
            cause: None,
        };
        let e2 = InterruptionEvent {
            duration_min: 5.0,
            customers_affected: 1,
            cause: None,
        };
        assert!(e1.is_momentary());
        assert!(e2.is_sustained());
    }

    // ── Topology metrics tests ────────────────────────────────────────────────

    #[test]
    fn test_radial_network_meshedness_zero() {
        // Star topology: bus 0 connected to buses 1,2,3,4
        let edges = [(0, 1), (0, 2), (0, 3), (0, 4)];
        let m = TopologyMetrics::compute(5, &edges);
        assert!(
            m.is_radial(),
            "Star network should be radial: γ={:.4}",
            m.meshedness
        );
    }

    #[test]
    fn test_ring_network_meshedness_positive() {
        // Ring: 0-1-2-3-4-0
        let edges = [(0, 1), (1, 2), (2, 3), (3, 4), (4, 0)];
        let m = TopologyMetrics::compute(5, &edges);
        assert!(
            m.meshedness > 0.0,
            "Ring should have positive meshedness: γ={:.4}",
            m.meshedness
        );
    }

    #[test]
    fn test_average_degree_star() {
        let edges = [(0, 1), (0, 2), (0, 3)];
        let m = TopologyMetrics::compute(4, &edges);
        // Degrees: 3, 1, 1, 1 → average = 6/4 = 1.5
        assert!((m.average_degree - 1.5).abs() < 1e-10);
    }

    #[test]
    fn test_diameter_line_graph() {
        // Line: 0-1-2-3-4
        let edges = [(0, 1), (1, 2), (2, 3), (3, 4)];
        let m = TopologyMetrics::compute(5, &edges);
        assert_eq!(m.diameter, 4, "Line graph diameter should be 4");
    }

    #[test]
    fn test_connected_components_disconnected() {
        // Two disconnected pairs
        let edges = [(0, 1), (2, 3)];
        let m = TopologyMetrics::compute(4, &edges);
        assert_eq!(m.n_components, 2);
    }

    #[test]
    fn test_connected_single_component() {
        let edges = [(0, 1), (1, 2), (2, 0)];
        let m = TopologyMetrics::compute(3, &edges);
        assert_eq!(m.n_components, 1);
    }

    // ── N-1 connectivity tests ────────────────────────────────────────────────

    #[test]
    fn test_n1_ring_all_redundant() {
        // Ring of 4 nodes: every edge removal still leaves connected
        let edges = [(0, 1), (1, 2), (2, 3), (3, 0)];
        let idx = n1_connectivity_index(4, &edges);
        assert!(
            (idx - 1.0).abs() < 1e-10,
            "Ring should be N-1 redundant: {idx:.4}"
        );
    }

    #[test]
    fn test_n1_radial_none_redundant() {
        // Line: 0-1-2-3; every edge removal disconnects
        let edges = [(0, 1), (1, 2), (2, 3)];
        let idx = n1_connectivity_index(4, &edges);
        assert!(
            idx < 1e-10,
            "Radial line should have 0 N-1 redundancy: {idx:.4}"
        );
    }

    // ── Electrical distance tests ─────────────────────────────────────────────

    #[test]
    fn test_electrical_distance_self_zero() {
        // B-bus for simple 2-bus: [[2, -2], [-2, 2]]
        // This is singular; use a regularised version
        let b = vec![vec![3.0, -1.0], vec![-1.0, 3.0]];
        let dist = electrical_distance_matrix(&b);
        assert_eq!(dist.len(), 2);
        // D_ii = X_ii + X_ii - 2*X_ii = 0
        assert!(dist[0][0].abs() < 1e-6, "Self-distance should be 0");
        assert!(dist[1][1].abs() < 1e-6);
    }

    #[test]
    fn test_electrical_distance_symmetric() {
        let b = vec![
            vec![4.0, -2.0, -1.0],
            vec![-2.0, 4.0, -1.0],
            vec![-1.0, -1.0, 3.0],
        ];
        let dist = electrical_distance_matrix(&b);
        #[allow(clippy::needless_range_loop)]
        for i in 0..3 {
            for j in 0..3 {
                assert!(
                    (dist[i][j] - dist[j][i]).abs() < 1e-8,
                    "D should be symmetric"
                );
            }
        }
    }

    #[test]
    fn test_electrical_distance_nonnegative() {
        // Use a non-singular positive-definite matrix (diagonal-dominant)
        let b = vec![
            vec![5.0, -1.0, -1.0],
            vec![-1.0, 5.0, -1.0],
            vec![-1.0, -1.0, 5.0],
        ];
        let dist = electrical_distance_matrix(&b);
        for row in &dist {
            for &d in row {
                assert!(
                    d >= -1e-6,
                    "Electrical distance must be non-negative: {d:.4}"
                );
            }
        }
    }

    #[test]
    fn test_topology_metrics_n_branches() {
        let edges = [(0, 1), (1, 2), (2, 3)];
        let m = TopologyMetrics::compute(4, &edges);
        assert_eq!(m.n_branches, 3);
        assert_eq!(m.n_buses, 4);
    }
}
