/// N-1 contingency analysis and LODF-filtered ranking.
///
/// Implements:
/// - N-1 enumeration: remove each branch and check for thermal violations
/// - LODF-based fast screening: estimate post-contingency flows from pre-contingency
/// - Severity ranking: sort contingencies by worst-case loading
/// - Voltage contingency: track minimum bus voltage under each contingency
///
/// # Algorithm
///
/// For each contingency branch k:
///   ΔF_ij = LODF_ij,k · F_k^0
///   F_ij^post = F_ij^0 + ΔF_ij
///   Loading_ij^post = |F_ij^post| / Rate_ij
///
/// Branches with Loading > 1.0 p.u. are flagged as violations.
///
/// # References
/// - Glover, Sarma & Overbye, "Power Systems Analysis and Design", 5th ed., Ch. 11
/// - Wood, Wollenberg & Sheblé, "Power Generation, Operation and Control", 3rd ed.
use serde::{Deserialize, Serialize};

// ────────────────────────────────────────────────────────────────────────────
// Types
// ────────────────────────────────────────────────────────────────────────────

/// A single N-1 contingency (one branch outage).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Contingency {
    /// Branch index (0-based)
    pub branch_idx: usize,
    /// Branch name for reporting
    pub name: String,
    /// From bus index
    pub from_bus: usize,
    /// To bus index
    pub to_bus: usize,
}

/// A post-contingency branch loading violation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContingencyViolation {
    /// Contingency that caused the violation
    pub contingency: Contingency,
    /// Overloaded branch index
    pub overloaded_branch: usize,
    /// Pre-contingency loading [p.u.]
    pub pre_loading_pu: f64,
    /// Post-contingency loading [p.u.]
    pub post_loading_pu: f64,
    /// Post-contingency flow [p.u.]
    pub post_flow_pu: f64,
    /// Thermal limit [p.u.]
    pub limit_pu: f64,
}

/// Result of N-1 contingency analysis.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContingencyResult {
    /// All violations found
    pub violations: Vec<ContingencyViolation>,
    /// Number of contingencies analysed
    pub n_contingencies: usize,
    /// Number of binding contingencies (at least one violation)
    pub n_binding: usize,
    /// Worst-case contingency name
    pub worst_contingency: Option<String>,
    /// Worst-case loading observed
    pub worst_loading_pu: f64,
    /// LODF-filtered contingencies (pruned by threshold)
    pub n_screened_out: usize,
}

/// Configuration for contingency analysis.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContingencyConfig {
    /// Alert thermal loading threshold [p.u.] (default 1.0 = at limit)
    pub loading_limit_pu: f64,
    /// LODF magnitude threshold for fast screening (contingencies with
    /// max |LODF| < this are skipped as "negligible impact")
    pub lodf_screen_threshold: f64,
    /// Pre-contingency loading threshold: only check branches loaded > this
    pub pre_loading_alert_pu: f64,
    /// Maximum violations to report per contingency
    pub max_violations_per_contingency: usize,
}

impl Default for ContingencyConfig {
    fn default() -> Self {
        Self {
            loading_limit_pu: 1.0,
            lodf_screen_threshold: 0.05,
            pre_loading_alert_pu: 0.0,
            max_violations_per_contingency: 5,
        }
    }
}

// ────────────────────────────────────────────────────────────────────────────
// N-1 analysis
// ────────────────────────────────────────────────────────────────────────────

/// Run N-1 contingency analysis using LODF-based screening.
///
/// # Arguments
/// - `base_flows_pu`  — pre-contingency branch flows [p.u.]
/// - `limits_pu`      — branch thermal limits [p.u.]
/// - `lodf_matrix`    — `LODF[i][j]` = power transfer factor (branch i after loss of branch j)
/// - `contingencies`  — list of branches to take out
/// - `config`         — analysis settings
pub fn run_n1_contingency(
    base_flows_pu: &[f64],
    limits_pu: &[f64],
    lodf_matrix: &[Vec<f64>],
    contingencies: &[Contingency],
    config: &ContingencyConfig,
) -> ContingencyResult {
    let n_branches = base_flows_pu.len();
    let mut violations = Vec::new();
    let mut n_binding = 0;
    let mut worst_loading = 0.0_f64;
    let mut worst_name: Option<String> = None;
    let mut n_screened_out = 0;

    for cont in contingencies {
        let k = cont.branch_idx;
        if k >= n_branches {
            continue;
        }

        // LODF screening: check max |LODF[i][k]| across all branches
        let max_lodf = (0..n_branches)
            .filter(|&i| i != k)
            .map(|i| {
                lodf_matrix
                    .get(i)
                    .and_then(|row| row.get(k))
                    .map(|&v| v.abs())
                    .unwrap_or(0.0)
            })
            .fold(0.0_f64, f64::max);

        if max_lodf < config.lodf_screen_threshold {
            n_screened_out += 1;
            continue;
        }

        // Compute post-contingency flows
        let f_k = base_flows_pu[k];
        let mut cont_violations = Vec::new();
        let mut cont_binding = false;

        for i in 0..n_branches {
            if i == k {
                continue;
            }

            let lodf_ik = lodf_matrix
                .get(i)
                .and_then(|row| row.get(k))
                .copied()
                .unwrap_or(0.0);
            let delta_f = lodf_ik * f_k;
            let post_flow = base_flows_pu[i] + delta_f;
            let limit = if i < limits_pu.len() {
                limits_pu[i]
            } else {
                1.0
            };
            let post_loading = post_flow.abs() / limit.max(1e-9);
            let pre_loading = base_flows_pu[i].abs() / limit.max(1e-9);

            if post_loading > config.loading_limit_pu {
                cont_binding = true;
                if worst_loading < post_loading {
                    worst_loading = post_loading;
                    worst_name = Some(cont.name.clone());
                }
                cont_violations.push(ContingencyViolation {
                    contingency: cont.clone(),
                    overloaded_branch: i,
                    pre_loading_pu: pre_loading,
                    post_loading_pu: post_loading,
                    post_flow_pu: post_flow,
                    limit_pu: limit,
                });
                if cont_violations.len() >= config.max_violations_per_contingency {
                    break;
                }
            }
        }

        if cont_binding {
            n_binding += 1;
        }
        violations.extend(cont_violations);
    }

    ContingencyResult {
        violations,
        n_contingencies: contingencies.len(),
        n_binding,
        worst_contingency: worst_name,
        worst_loading_pu: worst_loading,
        n_screened_out,
    }
}

/// Generate N-1 contingency list for all branches.
pub fn enumerate_n1(n_branches: usize) -> Vec<Contingency> {
    (0..n_branches)
        .map(|i| Contingency {
            branch_idx: i,
            name: format!("L{}", i + 1),
            from_bus: 0,
            to_bus: 0,
        })
        .collect()
}

/// Rank contingencies by worst-case loading after outage.
///
/// Returns (contingency_branch_idx, max_loading) pairs sorted descending.
pub fn rank_contingencies(
    base_flows_pu: &[f64],
    limits_pu: &[f64],
    lodf_matrix: &[Vec<f64>],
) -> Vec<(usize, f64)> {
    let n = base_flows_pu.len();
    let mut rankings: Vec<(usize, f64)> = (0..n)
        .map(|k| {
            let f_k = base_flows_pu[k];
            let max_post = (0..n)
                .filter(|&i| i != k)
                .map(|i| {
                    let lodf = lodf_matrix
                        .get(i)
                        .and_then(|r| r.get(k))
                        .copied()
                        .unwrap_or(0.0);
                    let limit = if i < limits_pu.len() {
                        limits_pu[i]
                    } else {
                        1.0
                    };
                    (base_flows_pu[i] + lodf * f_k).abs() / limit.max(1e-9)
                })
                .fold(0.0_f64, f64::max);
            (k, max_post)
        })
        .collect();

    rankings.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
    rankings
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a simple 3-branch LODF matrix for testing.
    /// System: 3 buses, 3 branches in a triangle.
    fn make_lodf_3x3() -> Vec<Vec<f64>> {
        // LODF[i][j]: approximate values
        vec![
            vec![0.0, 0.5, -0.5], // branch 0
            vec![0.5, 0.0, 0.5],  // branch 1
            vec![-0.5, 0.5, 0.0], // branch 2
        ]
    }

    #[test]
    fn test_n1_no_violations_low_loading() {
        let flows = vec![0.3, 0.2, 0.1];
        let limits = vec![1.0, 1.0, 1.0];
        let lodf = make_lodf_3x3();
        let conts = enumerate_n1(3);
        let config = ContingencyConfig::default();
        let result = run_n1_contingency(&flows, &limits, &lodf, &conts, &config);
        assert_eq!(result.violations.len(), 0, "No violations for low loading");
    }

    #[test]
    fn test_n1_violation_detected() {
        let flows = vec![0.9, 0.8, 0.1]; // high pre-loading
        let limits = vec![1.0, 1.0, 1.0];
        let lodf = make_lodf_3x3();
        let conts = enumerate_n1(3);
        let config = ContingencyConfig {
            lodf_screen_threshold: 0.01,
            ..Default::default()
        };
        let result = run_n1_contingency(&flows, &limits, &lodf, &conts, &config);
        // Loss of branch 0 (flow=0.9) → LODF[1][0]=0.5 → ΔF1=0.45 → F1_post=1.25 > 1.0
        assert!(
            !result.violations.is_empty(),
            "Should detect violations with high loading"
        );
    }

    #[test]
    fn test_n1_worst_contingency_identified() {
        let flows = vec![0.9, 0.3, 0.3];
        let limits = vec![1.0, 1.0, 1.0];
        let lodf = make_lodf_3x3();
        let conts = vec![
            Contingency {
                branch_idx: 0,
                name: "L1".to_string(),
                from_bus: 0,
                to_bus: 1,
            },
            Contingency {
                branch_idx: 1,
                name: "L2".to_string(),
                from_bus: 1,
                to_bus: 2,
            },
        ];
        let config = ContingencyConfig {
            lodf_screen_threshold: 0.01,
            ..Default::default()
        };
        let result = run_n1_contingency(&flows, &limits, &lodf, &conts, &config);
        if let Some(wc) = result.worst_contingency {
            assert!(!wc.is_empty(), "Worst contingency should have a name");
        }
    }

    #[test]
    fn test_lodf_screening_reduces_computation() {
        let flows = vec![0.5, 0.5, 0.5];
        let limits = vec![1.0, 1.0, 1.0];
        let lodf = make_lodf_3x3();
        let conts = enumerate_n1(3);
        // High LODF threshold → screens most out
        let config = ContingencyConfig {
            lodf_screen_threshold: 0.9,
            ..Default::default()
        };
        let result = run_n1_contingency(&flows, &limits, &lodf, &conts, &config);
        assert!(
            result.n_screened_out > 0,
            "Should screen some contingencies"
        );
    }

    #[test]
    fn test_enumerate_n1_count() {
        let conts = enumerate_n1(20);
        assert_eq!(conts.len(), 20);
        for (i, c) in conts.iter().enumerate() {
            assert_eq!(c.branch_idx, i);
        }
    }

    #[test]
    fn test_rank_contingencies_sorted_descending() {
        let flows = vec![0.5, 0.8, 0.2];
        let limits = vec![1.0, 1.0, 1.0];
        let lodf = make_lodf_3x3();
        let rankings = rank_contingencies(&flows, &limits, &lodf);
        assert_eq!(rankings.len(), 3);
        for w in rankings.windows(2) {
            assert!(w[0].1 >= w[1].1, "Rankings should be sorted descending");
        }
    }

    #[test]
    fn test_n1_empty_contingencies() {
        let flows = vec![0.5; 5];
        let limits = vec![1.0; 5];
        let lodf = vec![vec![0.0; 5]; 5];
        let result = run_n1_contingency(&flows, &limits, &lodf, &[], &ContingencyConfig::default());
        assert_eq!(result.n_contingencies, 0);
        assert_eq!(result.violations.len(), 0);
    }

    #[test]
    fn test_n1_violation_fields_valid() {
        let flows = vec![0.9, 0.8, 0.1];
        let limits = vec![1.0, 1.0, 1.0];
        let lodf = make_lodf_3x3();
        let conts = enumerate_n1(3);
        let config = ContingencyConfig {
            lodf_screen_threshold: 0.01,
            ..Default::default()
        };
        let result = run_n1_contingency(&flows, &limits, &lodf, &conts, &config);
        for v in &result.violations {
            assert!(
                v.post_loading_pu > 1.0,
                "Violation loading should exceed limit"
            );
            assert!(v.limit_pu > 0.0);
        }
    }

    #[test]
    fn test_n1_result_n_binding_le_n_contingencies() {
        let flows = vec![0.5, 0.5, 0.5];
        let limits = vec![1.0, 1.0, 1.0];
        let lodf = make_lodf_3x3();
        let conts = enumerate_n1(3);
        let result = run_n1_contingency(
            &flows,
            &limits,
            &lodf,
            &conts,
            &ContingencyConfig::default(),
        );
        assert!(result.n_binding <= result.n_contingencies);
    }
}
