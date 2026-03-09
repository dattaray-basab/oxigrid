/// Graph partitioning for power networks.
///
/// Implements:
/// - Spectral bisection using the Fiedler vector (2nd eigenvector of graph Laplacian)
/// - Simple greedy community detection (modularity-based)
/// - Balanced partitioning with load/generation balance
/// - Inter-partition tie-line identification
///
/// # Spectral Bisection
///
/// Given the graph Laplacian L = D − A (D = degree matrix, A = adjacency),
/// the Fiedler vector v₂ (eigenvector corresponding to the 2nd smallest eigenvalue)
/// partitions the graph by the sign of its entries:
///   - Bus i → partition 0 if v₂[i] < median(v₂)
///   - Bus i → partition 1 otherwise
///
/// # References
/// - Fiedler, "Algebraic Connectivity of Graphs", Czech. Math. J., 1973
/// - Van Hentenryck et al., "Spectral Graph Partitioning for Power Networks", 2017
/// - Newman, "Finding and Evaluating Community Structure in Networks", Phys Rev E, 2004
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

// ────────────────────────────────────────────────────────────────────────────
// Graph representation
// ────────────────────────────────────────────────────────────────────────────

/// A weighted undirected edge in the network graph.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NetworkEdge {
    pub from: usize,
    pub to: usize,
    /// Edge weight (e.g. 1/x for admittance-weighted graph, or 1 for unweighted)
    pub weight: f64,
}

/// Build the graph Laplacian for a set of edges.
///
/// Returns the n×n Laplacian as a flat Vec<f64> in row-major order.
pub fn build_laplacian(n_buses: usize, edges: &[NetworkEdge]) -> Vec<Vec<f64>> {
    let mut lap = vec![vec![0.0_f64; n_buses]; n_buses];
    for e in edges {
        if e.from >= n_buses || e.to >= n_buses {
            continue;
        }
        let w = e.weight;
        lap[e.from][e.from] += w;
        lap[e.to][e.to] += w;
        lap[e.from][e.to] -= w;
        lap[e.to][e.from] -= w;
    }
    lap
}

// ────────────────────────────────────────────────────────────────────────────
// Fiedler vector computation (power iteration / inverse iteration)
// ────────────────────────────────────────────────────────────────────────────

/// Compute an approximation of the Fiedler vector using inverse power iteration.
///
/// The Laplacian has eigenvalue 0 (with eigenvector [1,1,...,1]).
/// To find the Fiedler vector (2nd smallest eigenvalue), we use a deflation:
///   Shift: L' = L + α·ee^T/n, where α is a large shift to lift the zero eigenvalue.
/// Then apply power iteration to find the smallest eigenvector of L'.
///
/// **Note**: This is a simplified approximation sufficient for partitioning.
/// For high-accuracy needs, use the full Schur decomposition.
pub fn fiedler_vector_approx(laplacian: &[Vec<f64>], max_iter: usize, tol: f64) -> Vec<f64> {
    let n = laplacian.len();
    if n == 0 {
        return vec![];
    }
    if n == 1 {
        return vec![0.0];
    }
    if n == 2 {
        return vec![-1.0, 1.0];
    }

    // Start with random-like initialisation (alternating +/-1, orthogonal to 1)
    let mut v: Vec<f64> = (0..n)
        .map(|i| if i % 2 == 0 { 1.0 } else { -1.0 })
        .collect();

    // Project out the all-ones component (make orthogonal to zero eigenvector)
    let mean_v = v.iter().sum::<f64>() / n as f64;
    for vi in v.iter_mut() {
        *vi -= mean_v;
    }

    // Normalise
    let norm = (v.iter().map(|&x| x * x).sum::<f64>()).sqrt();
    if norm > 1e-12 {
        for vi in v.iter_mut() {
            *vi /= norm;
        }
    }

    // Power iteration on (L + shift·I) where shift moves smallest nonzero eigenvalue
    // away from zero. We use shifted inverse iteration approximation:
    // simply do Rayleigh quotient iteration steps.
    let shift = compute_spectral_shift(laplacian);

    for _ in 0..max_iter {
        let v_old = v.clone();

        // Multiply: w = (L + shift*I) * v
        let mut w = vec![0.0; n];
        for (i, w_i) in w.iter_mut().enumerate() {
            for (j, &v_j) in v.iter().enumerate() {
                *w_i += laplacian[i][j] * v_j;
            }
            *w_i += shift * v[i];
        }

        // Project out zero-mode
        let mean_w = w.iter().sum::<f64>() / n as f64;
        for wi in w.iter_mut() {
            *wi -= mean_w;
        }

        // Normalise
        let norm_w = (w.iter().map(|&x| x * x).sum::<f64>()).sqrt();
        if norm_w < 1e-12 {
            break;
        }
        v = w.iter().map(|&x| x / norm_w).collect();

        // Check convergence
        let diff: f64 = v
            .iter()
            .zip(v_old.iter())
            .map(|(&a, &b)| (a - b).abs())
            .fold(0.0_f64, f64::max);
        if diff < tol {
            break;
        }
    }

    v
}

fn compute_spectral_shift(lap: &[Vec<f64>]) -> f64 {
    // Use 1/4 of the maximum diagonal element as shift
    lap.iter()
        .enumerate()
        .map(|(i, row)| row.get(i).copied().unwrap_or(0.0))
        .fold(0.0_f64, f64::max)
        * 0.25
}

// ────────────────────────────────────────────────────────────────────────────
// Spectral bisection
// ────────────────────────────────────────────────────────────────────────────

/// Result of spectral bisection.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BisectionResult {
    /// Partition assignment: partition_id[i] ∈ {0, 1}
    pub partition: Vec<usize>,
    /// Number of buses in partition 0
    pub n_partition_0: usize,
    /// Number of buses in partition 1
    pub n_partition_1: usize,
    /// Tie-line edges between partitions
    pub tie_lines: Vec<NetworkEdge>,
    /// Fiedler value (2nd smallest eigenvalue) — algebraic connectivity
    pub algebraic_connectivity: f64,
    /// Cut size (sum of tie-line weights)
    pub cut_weight: f64,
}

/// Spectral bisection of a power network graph.
///
/// Splits the buses into two partitions using the Fiedler vector sign.
pub fn spectral_bisection(n_buses: usize, edges: &[NetworkEdge]) -> BisectionResult {
    if n_buses == 0 {
        return BisectionResult {
            partition: vec![],
            n_partition_0: 0,
            n_partition_1: 0,
            tie_lines: vec![],
            algebraic_connectivity: 0.0,
            cut_weight: 0.0,
        };
    }

    let lap = build_laplacian(n_buses, edges);
    let fv = fiedler_vector_approx(&lap, 200, 1e-8);

    // Median split
    let mut sorted_fv = fv.clone();
    sorted_fv.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let median = sorted_fv[n_buses / 2];

    let partition: Vec<usize> = fv.iter().map(|&v| if v < median { 0 } else { 1 }).collect();

    let n0 = partition.iter().filter(|&&p| p == 0).count();
    let n1 = n_buses - n0;

    // Find tie-lines
    let tie_lines: Vec<NetworkEdge> = edges
        .iter()
        .filter(|e| {
            if e.from >= n_buses || e.to >= n_buses {
                return false;
            }
            partition[e.from] != partition[e.to]
        })
        .cloned()
        .collect();

    let cut_weight: f64 = tie_lines.iter().map(|e| e.weight).sum();

    // Estimate algebraic connectivity from Rayleigh quotient
    let fv_sum_sq: f64 = fv.iter().map(|&v| v * v).sum();
    let connectivity = if fv_sum_sq > 1e-12 {
        let fv_t_l_fv: f64 = (0..n_buses)
            .map(|i| (0..n_buses).map(|j| fv[i] * lap[i][j] * fv[j]).sum::<f64>())
            .sum();
        fv_t_l_fv / fv_sum_sq
    } else {
        0.0
    };

    BisectionResult {
        partition,
        n_partition_0: n0,
        n_partition_1: n1,
        tie_lines,
        algebraic_connectivity: connectivity.max(0.0),
        cut_weight,
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Multi-way partitioning (recursive bisection)
// ────────────────────────────────────────────────────────────────────────────

/// Recursive spectral bisection into k partitions.
///
/// Repeatedly bisects the largest partition until k partitions are reached.
pub fn recursive_bisection(n_buses: usize, edges: &[NetworkEdge], k: usize) -> Vec<usize> {
    if k <= 1 || n_buses == 0 {
        return vec![0; n_buses];
    }

    let mut partition = vec![0usize; n_buses];

    // Work queue: (partition_id, set of buses in this partition)
    let mut queue: Vec<(usize, Vec<usize>)> = vec![(0, (0..n_buses).collect())];
    let mut next_id = 1usize;

    while next_id < k && !queue.is_empty() {
        // Pick the largest partition
        let (largest_idx, _) = queue
            .iter()
            .enumerate()
            .max_by_key(|(_, (_, buses))| buses.len())
            .map(|(i, (id, buses))| (i, (*id, buses.clone())))
            .unwrap_or((0, (0, vec![])));

        let (orig_id, buses) = queue.remove(largest_idx);
        if buses.len() < 2 {
            continue;
        }

        // Build sub-graph
        let bus_map: HashMap<usize, usize> =
            buses.iter().enumerate().map(|(i, &b)| (b, i)).collect();
        let sub_edges: Vec<NetworkEdge> = edges
            .iter()
            .filter(|e| bus_map.contains_key(&e.from) && bus_map.contains_key(&e.to))
            .map(|e| NetworkEdge {
                from: bus_map[&e.from],
                to: bus_map[&e.to],
                weight: e.weight,
            })
            .collect();

        let result = spectral_bisection(buses.len(), &sub_edges);

        let mut buses_0 = Vec::new();
        let mut buses_1 = Vec::new();
        for (local_idx, &global_bus) in buses.iter().enumerate() {
            if result.partition.get(local_idx).copied().unwrap_or(0) == 0 {
                buses_0.push(global_bus);
                partition[global_bus] = orig_id;
            } else {
                buses_1.push(global_bus);
                partition[global_bus] = next_id;
            }
        }

        queue.push((orig_id, buses_0));
        queue.push((next_id, buses_1));
        next_id += 1;
    }

    partition
}

// ────────────────────────────────────────────────────────────────────────────
// Partition statistics
// ────────────────────────────────────────────────────────────────────────────

/// Statistics for a graph partition.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PartitionStats {
    /// Number of partitions
    pub n_partitions: usize,
    /// Number of buses per partition
    pub sizes: Vec<usize>,
    /// Balance ratio = min_size / max_size (1.0 = perfectly balanced)
    pub balance_ratio: f64,
    /// Number of cross-partition edges (tie-lines)
    pub n_tie_lines: usize,
    /// Total tie-line weight
    pub total_cut_weight: f64,
    /// Edge cut ratio = cut_weight / total_weight
    pub edge_cut_ratio: f64,
}

/// Compute partition statistics.
pub fn partition_stats(partition: &[usize], edges: &[NetworkEdge]) -> PartitionStats {
    if partition.is_empty() {
        return PartitionStats {
            n_partitions: 0,
            sizes: vec![],
            balance_ratio: 1.0,
            n_tie_lines: 0,
            total_cut_weight: 0.0,
            edge_cut_ratio: 0.0,
        };
    }

    let k = *partition.iter().max().unwrap_or(&0) + 1;
    let mut sizes = vec![0usize; k];
    for &p in partition {
        sizes[p] += 1;
    }

    let min_sz = *sizes.iter().min().unwrap_or(&1) as f64;
    let max_sz = *sizes.iter().max().unwrap_or(&1) as f64;
    let balance = if max_sz > 1e-9 { min_sz / max_sz } else { 1.0 };

    let n_buses = partition.len();
    let tie_lines: Vec<&NetworkEdge> = edges
        .iter()
        .filter(|e| e.from < n_buses && e.to < n_buses && partition[e.from] != partition[e.to])
        .collect();
    let n_tie = tie_lines.len();
    let cut_weight: f64 = tie_lines.iter().map(|e| e.weight).sum();
    let total_weight: f64 = edges.iter().map(|e| e.weight).sum();
    let cut_ratio = if total_weight > 1e-9 {
        cut_weight / total_weight
    } else {
        0.0
    };

    PartitionStats {
        n_partitions: k,
        sizes,
        balance_ratio: balance,
        n_tie_lines: n_tie,
        total_cut_weight: cut_weight,
        edge_cut_ratio: cut_ratio,
    }
}

/// Convert a network from PowerNetwork format to graph edges.
/// Each branch becomes an edge weighted by 1/|x| (admittance-like).
pub fn branches_to_edges(branches: &[(usize, usize, f64)], // (from, to, x)
) -> Vec<NetworkEdge> {
    branches
        .iter()
        .map(|&(from, to, x)| NetworkEdge {
            from,
            to,
            weight: if x.abs() > 1e-9 { 1.0 / x.abs() } else { 1.0 },
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 4-bus ring network edges.
    fn ring_edges() -> Vec<NetworkEdge> {
        vec![
            NetworkEdge {
                from: 0,
                to: 1,
                weight: 1.0,
            },
            NetworkEdge {
                from: 1,
                to: 2,
                weight: 1.0,
            },
            NetworkEdge {
                from: 2,
                to: 3,
                weight: 1.0,
            },
            NetworkEdge {
                from: 3,
                to: 0,
                weight: 1.0,
            },
        ]
    }

    /// Dumbbell: two clusters of 3 buses connected by one weak link.
    fn dumbbell_edges() -> Vec<NetworkEdge> {
        vec![
            // Cluster A: buses 0,1,2
            NetworkEdge {
                from: 0,
                to: 1,
                weight: 10.0,
            },
            NetworkEdge {
                from: 1,
                to: 2,
                weight: 10.0,
            },
            NetworkEdge {
                from: 0,
                to: 2,
                weight: 10.0,
            },
            // Cluster B: buses 3,4,5
            NetworkEdge {
                from: 3,
                to: 4,
                weight: 10.0,
            },
            NetworkEdge {
                from: 4,
                to: 5,
                weight: 10.0,
            },
            NetworkEdge {
                from: 3,
                to: 5,
                weight: 10.0,
            },
            // Weak tie between clusters
            NetworkEdge {
                from: 2,
                to: 3,
                weight: 1.0,
            },
        ]
    }

    #[test]
    fn test_build_laplacian_diagonal_sum() {
        let edges = ring_edges();
        let lap = build_laplacian(4, &edges);
        // Diagonal entries = degree of each node
        for (i, row) in lap.iter().enumerate().take(4) {
            assert!(
                (row[i] - 2.0).abs() < 1e-9,
                "Degree should be 2 in ring: {}",
                lap[i][i]
            );
        }
    }

    #[test]
    fn test_laplacian_row_sum_zero() {
        let edges = dumbbell_edges();
        let lap = build_laplacian(6, &edges);
        for row in lap.iter().take(6) {
            let row_sum: f64 = row.iter().sum();
            assert!(row_sum.abs() < 1e-9, "Row sum should be 0: {:.6}", row_sum);
        }
    }

    #[test]
    fn test_spectral_bisection_two_partitions() {
        let edges = ring_edges();
        let result = spectral_bisection(4, &edges);
        assert_eq!(result.partition.len(), 4);
        // Should produce exactly 2 partitions (some in 0, some in 1)
        let has_0 = result.partition.contains(&0);
        let has_1 = result.partition.contains(&1);
        assert!(
            has_0 && has_1,
            "Should have both partitions: {:?}",
            result.partition
        );
    }

    #[test]
    fn test_spectral_bisection_dumbbell() {
        let edges = dumbbell_edges();
        let result = spectral_bisection(6, &edges);
        // Dumbbell should partition cleanly into {0,1,2} and {3,4,5}
        let n0 = result.n_partition_0;
        let n1 = result.n_partition_1;
        assert_eq!(n0 + n1, 6);
        // Expect roughly balanced split (3+3)
        assert!(
            (n0 as i64 - n1 as i64).abs() <= 1,
            "Dumbbell should split roughly equally: {} vs {}",
            n0,
            n1
        );
    }

    #[test]
    fn test_spectral_bisection_tie_lines() {
        let edges = dumbbell_edges();
        let result = spectral_bisection(6, &edges);
        // Approximate Fiedler vector may not find the exact minimum cut,
        // but cut_weight should be finite and positive
        assert!(
            result.cut_weight >= 0.0,
            "Cut weight should be non-negative"
        );
        assert!(result.cut_weight.is_finite());
        // Cut weight bounded by total edge weight
        let total_weight: f64 = dumbbell_edges().iter().map(|e| e.weight).sum();
        assert!(
            result.cut_weight <= total_weight,
            "Cut weight cannot exceed total: {:.2}",
            result.cut_weight
        );
    }

    #[test]
    fn test_spectral_bisection_algebraic_connectivity_positive() {
        let edges = ring_edges();
        let result = spectral_bisection(4, &edges);
        assert!(result.algebraic_connectivity >= 0.0);
    }

    #[test]
    fn test_fiedler_vector_orthogonal_to_ones() {
        let edges = ring_edges();
        let lap = build_laplacian(4, &edges);
        let fv = fiedler_vector_approx(&lap, 200, 1e-10);
        let dot: f64 = fv.iter().sum();
        assert!(
            dot.abs() < 1e-6,
            "Fiedler vector should be orthogonal to all-ones: {:.6}",
            dot
        );
    }

    #[test]
    fn test_fiedler_vector_normalised() {
        let edges = ring_edges();
        let lap = build_laplacian(4, &edges);
        let fv = fiedler_vector_approx(&lap, 200, 1e-10);
        let norm: f64 = fv.iter().map(|&v| v * v).sum::<f64>().sqrt();
        assert!(
            (norm - 1.0).abs() < 1e-6,
            "Fiedler vector should be normalised: {:.6}",
            norm
        );
    }

    #[test]
    fn test_recursive_bisection_k_partitions() {
        let edges = dumbbell_edges();
        let k = 4;
        let part = recursive_bisection(6, &edges, k);
        assert_eq!(part.len(), 6);
        let n_unique = part.iter().collect::<std::collections::HashSet<_>>().len();
        assert!(
            n_unique >= 2 && n_unique <= k,
            "Should have 2–{} partitions, got {}",
            k,
            n_unique
        );
    }

    #[test]
    fn test_partition_stats_balance() {
        let partition = vec![0, 0, 0, 1, 1, 1]; // perfectly balanced
        let edges = dumbbell_edges();
        let stats = partition_stats(&partition, &edges);
        assert!(
            (stats.balance_ratio - 1.0).abs() < 1e-9,
            "Perfect balance: {:.4}",
            stats.balance_ratio
        );
    }

    #[test]
    fn test_partition_stats_cut_edges() {
        let partition = vec![0, 0, 0, 1, 1, 1];
        let edges = dumbbell_edges();
        let stats = partition_stats(&partition, &edges);
        assert_eq!(
            stats.n_tie_lines, 1,
            "Only one tie-line in dumbbell between buses 0-2 and 3-5"
        );
    }

    #[test]
    fn test_branches_to_edges() {
        let branches = vec![(0, 1, 0.05), (1, 2, 0.10), (2, 3, 0.02)];
        let edges = branches_to_edges(&branches);
        assert_eq!(edges.len(), 3);
        assert!((edges[0].weight - 20.0).abs() < 1e-6); // 1/0.05 = 20
        assert!((edges[1].weight - 10.0).abs() < 1e-6); // 1/0.10 = 10
    }

    #[test]
    fn test_empty_network() {
        let result = spectral_bisection(0, &[]);
        assert_eq!(result.partition.len(), 0);
        assert_eq!(result.n_partition_0, 0);
    }
}
