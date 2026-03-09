//! N-k contingency analysis for power system security assessment.
//!
//! Provides fast DC-based screening (PTDF/LODF) and full DC power flow
//! evaluation of contingencies with performance index computation.
//!
//! # Overview
//!
//! 1. Build a [`ContingencyAnalyzer`] with a [`NetworkData`] description.
//! 2. Call [`ContingencyAnalyzer::generate_n1_contingencies`] for automatic N-1.
//! 3. Screen with [`ContingencyAnalyzer::screen_with_ptdf`] for fast triage.
//! 4. Evaluate flagged contingencies with [`ContingencyAnalyzer::analyze_all_dc`].
//!
//! # References
//! - Glover, Sarma, Overbye, "Power Systems Analysis and Design", 6th ed.
//! - Wood, Wollenberg, "Power Generation, Operation, and Control", 2nd ed.

use serde::{Deserialize, Serialize};
use std::time::Instant;

// ─── Enums ───────────────────────────────────────────────────────────────────

/// Classification of a contingency event by element type and cardinality.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ContingencyType {
    /// Single transmission line or cable outage (N-1).
    SingleLineOutage,
    /// Double transmission line outage (N-2).
    DoubleLineOutage,
    /// Generator (dispatchable unit) outage.
    GeneratorOutage,
    /// Transformer outage.
    TransformerOutage,
    /// Bus outage (loss of substation bus section).
    BusOutage,
    /// Load outage.
    LoadOutage,
    /// General multi-element outage.
    MultiElement,
}

/// Security status returned after evaluating a contingency.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ContingencyStatus {
    /// All constraints satisfied with adequate margin.
    Secure,
    /// Within 5 % of a thermal or voltage limit.
    Warning,
    /// At least one constraint exceeded.
    Violation,
    /// Network islanding detected (disconnected bus or sub-network).
    Islanding,
    /// Branch thermal overload (flow > 100 % of rating).
    Overload,
    /// Bus voltage outside [V_min, V_max].
    VoltageViolation,
    /// Power-flow did not converge.
    ConvergenceFailed,
}

/// Method used for contingency screening / evaluation.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ScreeningMethod {
    /// Fast linear PTDF-based screening.
    Ptdf,
    /// Simplified DC power flow (Bθ = P).
    DcPowerFlow,
    /// Full AC Newton-Raphson power flow.
    AcPowerFlow,
    /// Hybrid: PTDF pre-screen, then DC/AC for flagged cases.
    Hybrid,
}

// ─── Structs ─────────────────────────────────────────────────────────────────

/// Description of a single contingency event.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Contingency {
    /// Unique identifier (matches index in [`ContingencyAnalyzer::contingencies`] after generation).
    pub id: usize,
    /// Human-readable label (e.g. "Line 3-7 outage").
    pub name: String,
    /// Category of the outage.
    pub contingency_type: ContingencyType,
    /// Indices of the outaged elements (branch IDs, generator IDs, or bus IDs).
    pub outaged_elements: Vec<usize>,
    /// Element category string: `"branch"`, `"generator"`, or `"bus"`.
    pub element_type: String,
    /// Estimated annual probability of occurrence (default 0.001).
    pub probability: f64,
    /// Severity weight used in risk-weighted ranking (default 1.0).
    pub severity_weight: f64,
}

impl Contingency {
    /// Create a new contingency with default probability and severity weight.
    pub fn new(
        id: usize,
        name: impl Into<String>,
        contingency_type: ContingencyType,
        outaged_elements: Vec<usize>,
        element_type: impl Into<String>,
    ) -> Self {
        Self {
            id,
            name: name.into(),
            contingency_type,
            outaged_elements,
            element_type: element_type.into(),
            probability: 0.001,
            severity_weight: 1.0,
        }
    }
}

/// Results of evaluating a single contingency.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContingencyResult {
    /// ID of the [`Contingency`] that produced this result.
    pub contingency_id: usize,
    /// Overall security status.
    pub status: ContingencyStatus,
    /// Maximum branch loading expressed as percentage of its thermal rating.
    pub max_overload_pct: f64,
    /// Branches with loading above their rating: `(branch_index, loading_pct)`.
    pub overloaded_branches: Vec<(usize, f64)>,
    /// Buses with voltage outside limits: `(bus_index, voltage_pu)`.
    pub voltage_violations: Vec<(usize, f64)>,
    /// Estimated load shed in MW (0 for DC approximation).
    pub total_load_shed_mw: f64,
    /// Post-contingency bus voltage angles in radians.
    pub bus_angles: Vec<f64>,
    /// Post-contingency branch active power flows in MW (sign: from→to positive).
    pub branch_flows_mw: Vec<f64>,
    /// Wall-clock time taken to evaluate this contingency in milliseconds.
    pub computation_time_ms: f64,
}

/// Fast linear screening result for a single contingency.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContingencyScreenResult {
    /// ID of the screened [`Contingency`].
    pub contingency_id: usize,
    /// Performance index: `PI = Σ_l w_l · (P_l / P_l_max)^(2n)`.
    pub performance_index: f64,
    /// `true` if `performance_index > pi_threshold`, meaning full analysis is recommended.
    pub needs_full_analysis: bool,
}

/// Simplified network representation used by [`ContingencyAnalyzer`].
///
/// All quantities are in per-unit (for x/r) or MW (for injections / ratings).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NetworkData {
    /// Number of buses.
    pub n_buses: usize,
    /// Number of branches.
    pub n_branches: usize,
    /// Net active power injection at each bus in MW (generation − load).
    pub bus_p_inj: Vec<f64>,
    /// From-bus index for each branch (0-based).
    pub branch_from: Vec<usize>,
    /// To-bus index for each branch (0-based).
    pub branch_to: Vec<usize>,
    /// Series reactance in per-unit for each branch.
    pub branch_x_pu: Vec<f64>,
    /// Series resistance in per-unit for each branch (used for losses, not DC PF).
    pub branch_r_pu: Vec<f64>,
    /// Thermal rating in MW for each branch.
    pub branch_rating_mw: Vec<f64>,
    /// Base-case bus voltage magnitudes in per-unit.
    pub bus_voltage_pu: Vec<f64>,
    /// Index of the slack (reference) bus.
    pub slack_bus: usize,
    /// Pre-computed PTDF matrix `[n_branch][n_bus]`, if available.
    pub ptdf_matrix: Option<Vec<Vec<f64>>>,
}

impl NetworkData {
    /// Construct a [`NetworkData`] with all required fields.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        n_buses: usize,
        n_branches: usize,
        bus_p_inj: Vec<f64>,
        branch_from: Vec<usize>,
        branch_to: Vec<usize>,
        branch_x_pu: Vec<f64>,
        branch_r_pu: Vec<f64>,
        branch_rating_mw: Vec<f64>,
        bus_voltage_pu: Vec<f64>,
        slack_bus: usize,
    ) -> Self {
        Self {
            n_buses,
            n_branches,
            bus_p_inj,
            branch_from,
            branch_to,
            branch_x_pu,
            branch_r_pu,
            branch_rating_mw,
            bus_voltage_pu,
            slack_bus,
            ptdf_matrix: None,
        }
    }
}

// ─── ContingencyAnalyzer ─────────────────────────────────────────────────────

/// Main engine for N-k contingency analysis.
///
/// # Example
/// ```rust,ignore
/// let mut analyzer = ContingencyAnalyzer::new(network);
/// analyzer.generate_n1_contingencies();
/// let screen = analyzer.screen_with_ptdf();
/// let results = analyzer.analyze_all_dc();
/// let worst = analyzer.find_worst_case(&results);
/// ```
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContingencyAnalyzer {
    /// Network topology and parameters.
    pub network: NetworkData,
    /// List of contingencies to be evaluated.
    pub contingencies: Vec<Contingency>,
    /// Screening / evaluation method.
    pub screening_method: ScreeningMethod,
    /// Minimum permissible bus voltage in pu (default 0.95).
    pub voltage_min_pu: f64,
    /// Maximum permissible bus voltage in pu (default 1.05).
    pub voltage_max_pu: f64,
    /// Loading percentage threshold for Warning status (default 95.0 %).
    pub overload_warning_pct: f64,
    /// Performance-index threshold above which full analysis is triggered (default 2.0).
    pub pi_threshold: f64,
    /// Exponent `n` in `PI = Σ (P/P_max)^(2n)` (default 1, range 1–4).
    pub n_exponent: u32,
}

impl ContingencyAnalyzer {
    /// Create a new analyzer with sensible defaults.
    pub fn new(network: NetworkData) -> Self {
        Self {
            network,
            contingencies: Vec::new(),
            screening_method: ScreeningMethod::Ptdf,
            voltage_min_pu: 0.95,
            voltage_max_pu: 1.05,
            overload_warning_pct: 95.0,
            pi_threshold: 2.0,
            n_exponent: 1,
        }
    }

    /// Append a contingency to the analysis list.
    pub fn add_contingency(&mut self, contingency: Contingency) {
        self.contingencies.push(contingency);
    }

    /// Auto-generate one [`ContingencyType::SingleLineOutage`] per branch.
    ///
    /// Clears any previously generated N-1 line contingencies before appending.
    pub fn generate_n1_contingencies(&mut self) {
        let n = self.network.n_branches;
        // Remove any existing auto-generated single line outages first
        self.contingencies.retain(|c| {
            c.element_type != "branch" || c.contingency_type != ContingencyType::SingleLineOutage
        });
        for k in 0..n {
            let c = Contingency {
                id: k,
                name: format!("N-1 Branch {k}"),
                contingency_type: ContingencyType::SingleLineOutage,
                outaged_elements: vec![k],
                element_type: "branch".to_string(),
                probability: 0.001,
                severity_weight: 1.0,
            };
            self.contingencies.push(c);
        }
    }

    // ─── PTDF / LODF ─────────────────────────────────────────────────────────

    /// Compute the full Power Transfer Distribution Factor matrix.
    ///
    /// Returns an `[n_branch × n_bus]` matrix where `PTDF[l][k]` gives the
    /// fraction of a 1 MW injection at bus `k` (withdrawn at slack) that flows
    /// on branch `l`.
    ///
    /// Algorithm: finite-difference perturbation — inject +1 MW at each
    /// non-slack bus, solve DC PF, record branch flows as PTDF column.
    pub fn compute_ptdf_matrix(&self) -> Vec<Vec<f64>> {
        let nb = self.network.n_buses;
        let nl = self.network.n_branches;
        let slack = self.network.slack_bus;

        // Base case with zero injections
        let zero_inj = vec![0.0_f64; nb];
        let base_flows = self.dc_solve_with_inj(&zero_inj, &[]);

        let mut ptdf = vec![vec![0.0_f64; nb]; nl];

        for k in 0..nb {
            if k == slack {
                // Slack column stays zero
                continue;
            }
            let mut p = vec![0.0_f64; nb];
            p[k] = 1.0;
            p[slack] = -1.0;
            let flows = self.dc_solve_with_inj(&p, &[]);
            for l in 0..nl {
                ptdf[l][k] = flows[l] - base_flows[l];
            }
        }
        ptdf
    }

    /// Screen all contingencies using the PTDF / LODF linear approximation.
    ///
    /// Returns one [`ContingencyScreenResult`] per contingency.
    pub fn screen_with_ptdf(&self) -> Vec<ContingencyScreenResult> {
        let ptdf = self
            .network
            .ptdf_matrix
            .clone()
            .unwrap_or_else(|| self.compute_ptdf_matrix());
        let base_flows = self.dc_solve_with_inj(&self.network.bus_p_inj, &[]);

        self.contingencies
            .iter()
            .map(|c| {
                let post_flows = self.linear_post_contingency_flows(c, &base_flows, &ptdf);
                let pi = self.compute_performance_index(&post_flows);
                ContingencyScreenResult {
                    contingency_id: c.id,
                    performance_index: pi,
                    needs_full_analysis: pi > self.pi_threshold,
                }
            })
            .collect()
    }

    /// Compute the Performance Index for a given set of branch flows.
    ///
    /// `PI = Σ_l (|P_l| / P_l_max)^(2n)`
    ///
    /// Branches with zero rating are treated as having a rating of 1 × 10⁶ MW.
    pub fn compute_performance_index(&self, flows: &[f64]) -> f64 {
        flows
            .iter()
            .enumerate()
            .map(|(l, &p)| {
                let rating = if l < self.network.branch_rating_mw.len() {
                    let r = self.network.branch_rating_mw[l];
                    if r < 1e-9 {
                        1e6
                    } else {
                        r
                    }
                } else {
                    1e6
                };
                let ratio = p.abs() / rating;
                ratio.powi(2 * self.n_exponent as i32)
            })
            .sum()
    }

    // ─── DC contingency evaluation ────────────────────────────────────────────

    /// Evaluate a single contingency using DC power flow.
    ///
    /// Removes the outaged branches from the B-matrix, re-solves, and checks
    /// thermal ratings. Voltage magnitudes are assumed 1.0 pu (DC approximation).
    pub fn analyze_contingency_dc(&self, contingency: &Contingency) -> ContingencyResult {
        let t0 = Instant::now();

        let outaged = self.resolve_outaged_branches(contingency);
        let flows = self.dc_solve_with_inj(&self.network.bus_p_inj, &outaged);
        let angles = self.dc_angles_with_inj(&self.network.bus_p_inj, &outaged);

        // Zero out flows on outaged branches
        let mut final_flows = flows;
        for &k in &outaged {
            if k < final_flows.len() {
                final_flows[k] = 0.0;
            }
        }

        // Check for islanding: angle is NaN or large (> 1e9) means disconnected
        let islanding = angles.iter().any(|&a| a.is_nan() || a.abs() > 1e9);

        let mut overloaded_branches: Vec<(usize, f64)> = Vec::new();
        let mut max_pct: f64 = 0.0;

        for (l, &pf) in final_flows.iter().enumerate() {
            let rating = if l < self.network.branch_rating_mw.len() {
                let r = self.network.branch_rating_mw[l];
                if r < 1e-9 {
                    1e6
                } else {
                    r
                }
            } else {
                1e6
            };
            let pct = pf.abs() / rating * 100.0;
            if pct > max_pct {
                max_pct = pct;
            }
            if pct > 100.0 {
                overloaded_branches.push((l, pct));
            }
        }

        // DC: voltages assumed 1.0 pu — no voltage violations
        let voltage_violations: Vec<(usize, f64)> = Vec::new();

        let elapsed_ms = t0.elapsed().as_secs_f64() * 1000.0;

        let mut result = ContingencyResult {
            contingency_id: contingency.id,
            status: ContingencyStatus::Secure,
            max_overload_pct: max_pct,
            overloaded_branches,
            voltage_violations,
            total_load_shed_mw: 0.0,
            bus_angles: angles,
            branch_flows_mw: final_flows,
            computation_time_ms: elapsed_ms,
        };

        result.status = if islanding {
            ContingencyStatus::Islanding
        } else {
            self.classify_contingency(&result)
        };

        result
    }

    /// Evaluate all contingencies using DC power flow.
    ///
    /// Returns one [`ContingencyResult`] per contingency in the same order as
    /// [`ContingencyAnalyzer::contingencies`].
    pub fn analyze_all_dc(&self) -> Vec<ContingencyResult> {
        self.contingencies
            .iter()
            .map(|c| self.analyze_contingency_dc(c))
            .collect()
    }

    /// Classify the security status of a contingency result.
    ///
    /// Priority (highest first): `Overload` → `VoltageViolation` → `Warning` → `Secure`.
    pub fn classify_contingency(&self, result: &ContingencyResult) -> ContingencyStatus {
        if !result.overloaded_branches.is_empty() || result.max_overload_pct > 100.0 {
            return ContingencyStatus::Overload;
        }
        if !result.voltage_violations.is_empty() {
            return ContingencyStatus::VoltageViolation;
        }
        if result.max_overload_pct >= self.overload_warning_pct {
            return ContingencyStatus::Warning;
        }
        ContingencyStatus::Secure
    }

    /// Find the contingency result with the highest thermal loading.
    ///
    /// Returns `None` if the slice is empty.
    pub fn find_worst_case<'a>(
        &self,
        results: &'a [ContingencyResult],
    ) -> Option<&'a ContingencyResult> {
        results.iter().max_by(|a, b| {
            a.max_overload_pct
                .partial_cmp(&b.max_overload_pct)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
    }

    /// Compute the Line Outage Distribution Factor for a monitored branch given an outaged branch.
    ///
    /// Uses the power-transfer PTDF convention:
    /// `PTDF_transfer[l][k] = PTDF[l][from_k] - PTDF[l][to_k]`
    ///
    /// `LODF_{l,k} = PTDF_transfer_{l,k} / (1 − PTDF_transfer_{k,k})`
    ///
    /// By definition, `LODF_{k,k} = −1` (a line always fully loses its own flow when outaged).
    /// This function returns −1.0 when `outaged_branch == monitored_branch` or when the
    /// denominator is near zero.
    pub fn compute_lodf(&self, outaged_branch: usize, monitored_branch: usize) -> f64 {
        // Self-LODF is always -1 by definition.
        if outaged_branch == monitored_branch {
            return -1.0;
        }

        let ptdf = self
            .network
            .ptdf_matrix
            .clone()
            .unwrap_or_else(|| self.compute_ptdf_matrix());

        let nb = self.network.n_buses;
        let from_k = self
            .network
            .branch_from
            .get(outaged_branch)
            .copied()
            .unwrap_or(0);
        let to_k = self
            .network
            .branch_to
            .get(outaged_branch)
            .copied()
            .unwrap_or(0);

        // Power-transfer PTDF: effect of transferring 1 MW from from_k to to_k
        // on the flow of branch l.
        let ptdf_lk = if monitored_branch < ptdf.len() && from_k < nb && to_k < nb {
            ptdf[monitored_branch][from_k] - ptdf[monitored_branch][to_k]
        } else {
            0.0
        };

        // Self power-transfer PTDF for the outaged branch (denominator term).
        let ptdf_kk = if outaged_branch < ptdf.len() && from_k < nb && to_k < nb {
            ptdf[outaged_branch][from_k] - ptdf[outaged_branch][to_k]
        } else {
            0.0
        };

        let denom = 1.0 - ptdf_kk;
        if denom.abs() < 1e-10 {
            return -1.0;
        }
        ptdf_lk / denom
    }

    // ─── Private helpers ──────────────────────────────────────────────────────

    /// Resolve which branch indices are outaged for a given contingency.
    fn resolve_outaged_branches(&self, contingency: &Contingency) -> Vec<usize> {
        match contingency.element_type.as_str() {
            "branch" | "transformer" => contingency.outaged_elements.clone(),
            "bus" => {
                // All branches incident to the outaged bus
                let bus = contingency
                    .outaged_elements
                    .first()
                    .copied()
                    .unwrap_or(usize::MAX);
                (0..self.network.n_branches)
                    .filter(|&l| {
                        self.network.branch_from.get(l).copied() == Some(bus)
                            || self.network.branch_to.get(l).copied() == Some(bus)
                    })
                    .collect()
            }
            _ => Vec::new(),
        }
    }

    /// DC power flow → branch flows in MW.
    ///
    /// `outaged_branches`: branches whose susceptance is zeroed out.
    fn dc_solve_with_inj(&self, p_inj: &[f64], outaged_branches: &[usize]) -> Vec<f64> {
        let angles = self.dc_angles_with_inj(p_inj, outaged_branches);
        self.flows_from_angles(&angles, outaged_branches)
    }

    /// DC power flow → bus voltage angles in radians.
    ///
    /// Returns NaN for disconnected buses.
    fn dc_angles_with_inj(&self, p_inj: &[f64], outaged_branches: &[usize]) -> Vec<f64> {
        let n = self.network.n_buses;
        let slack = self.network.slack_bus;

        if n == 0 {
            return Vec::new();
        }

        // Build B matrix (n × n)
        let mut b = vec![vec![0.0_f64; n]; n];
        for l in 0..self.network.n_branches {
            if outaged_branches.contains(&l) {
                continue;
            }
            let fi = self
                .network
                .branch_from
                .get(l)
                .copied()
                .unwrap_or(usize::MAX);
            let ti = self.network.branch_to.get(l).copied().unwrap_or(usize::MAX);
            let x = self.network.branch_x_pu.get(l).copied().unwrap_or(0.0);
            if fi >= n || ti >= n || x.abs() < 1e-12 {
                continue;
            }
            let bij = 1.0 / x;
            b[fi][fi] += bij;
            b[ti][ti] += bij;
            b[fi][ti] -= bij;
            b[ti][fi] -= bij;
        }

        // Reduce: remove slack row/col
        let non_slack: Vec<usize> = (0..n).filter(|&i| i != slack).collect();
        let m = non_slack.len();
        if m == 0 {
            return vec![0.0; n];
        }

        let mut b_red = vec![vec![0.0_f64; m]; m];
        for (ri, &i) in non_slack.iter().enumerate() {
            for (rj, &j) in non_slack.iter().enumerate() {
                b_red[ri][rj] = b[i][j];
            }
        }

        let mut p_red: Vec<f64> = non_slack
            .iter()
            .map(|&i| p_inj.get(i).copied().unwrap_or(0.0))
            .collect();

        let theta_red = match gaussian_solve(&mut b_red, &mut p_red) {
            Some(t) => t,
            None => {
                // Singular: mark all non-slack angles as NaN (islanding)
                return {
                    let mut angles = vec![f64::NAN; n];
                    angles[slack] = 0.0;
                    angles
                };
            }
        };

        let mut angles = vec![0.0_f64; n];
        for (ri, &i) in non_slack.iter().enumerate() {
            angles[i] = theta_red[ri];
        }
        angles
    }

    /// Convert bus angles to branch flows in MW.
    #[allow(clippy::needless_range_loop)]
    fn flows_from_angles(&self, angles: &[f64], outaged_branches: &[usize]) -> Vec<f64> {
        let n = self.network.n_branches;
        let mut flows = vec![0.0_f64; n];
        for l in 0..n {
            if outaged_branches.contains(&l) {
                flows[l] = 0.0;
                continue;
            }
            let fi = self
                .network
                .branch_from
                .get(l)
                .copied()
                .unwrap_or(usize::MAX);
            let ti = self.network.branch_to.get(l).copied().unwrap_or(usize::MAX);
            let x = self.network.branch_x_pu.get(l).copied().unwrap_or(0.0);
            if fi >= angles.len() || ti >= angles.len() || x.abs() < 1e-12 {
                continue;
            }
            let ai = angles[fi];
            let aj = angles[ti];
            if ai.is_nan() || aj.is_nan() {
                flows[l] = f64::NAN;
            } else {
                flows[l] = (ai - aj) / x;
            }
        }
        flows
    }

    /// Use LODF linear approximation to estimate post-contingency branch flows.
    fn linear_post_contingency_flows(
        &self,
        contingency: &Contingency,
        base_flows: &[f64],
        ptdf: &[Vec<f64>],
    ) -> Vec<f64> {
        let nl = self.network.n_branches;
        let mut post = base_flows.to_vec();
        if post.len() < nl {
            post.resize(nl, 0.0);
        }

        let outaged = self.resolve_outaged_branches(contingency);
        for &k in &outaged {
            if k >= nl {
                continue;
            }
            let p_k = base_flows.get(k).copied().unwrap_or(0.0);
            let ptdf_kk = {
                let from_k = self.network.branch_from.get(k).copied().unwrap_or(0);
                let to_k = self.network.branch_to.get(k).copied().unwrap_or(0);
                if k < ptdf.len() {
                    ptdf[k].get(from_k).copied().unwrap_or(0.0)
                        - ptdf[k].get(to_k).copied().unwrap_or(0.0)
                } else {
                    0.0
                }
            };
            let denom = 1.0 - ptdf_kk;

            for l in 0..nl {
                if l == k {
                    post[l] = 0.0;
                    continue;
                }
                let lodf = if denom.abs() < 1e-10 {
                    -1.0
                } else {
                    let from_k = self.network.branch_from.get(k).copied().unwrap_or(0);
                    let to_k = self.network.branch_to.get(k).copied().unwrap_or(0);
                    let ptdf_lk = if l < ptdf.len() {
                        ptdf[l].get(from_k).copied().unwrap_or(0.0)
                            - ptdf[l].get(to_k).copied().unwrap_or(0.0)
                    } else {
                        0.0
                    };
                    ptdf_lk / denom
                };
                post[l] += lodf * p_k;
            }
        }
        post
    }
}

// ─── Gaussian elimination ─────────────────────────────────────────────────────

/// Solve `A · x = b` by Gaussian elimination with partial pivoting.
///
/// Modifies `a` and `b` in place (augmented-matrix method).
/// Returns `None` if the system is singular (pivot < 1e-12).
#[allow(clippy::ptr_arg, clippy::needless_range_loop)]
fn gaussian_solve(a: &mut Vec<Vec<f64>>, b: &mut Vec<f64>) -> Option<Vec<f64>> {
    let n = b.len();
    if a.len() != n {
        return None;
    }

    // Forward elimination with partial pivoting
    for col in 0..n {
        // Find pivot
        let mut max_row = col;
        let mut max_val = a[col][col].abs();
        for row in (col + 1)..n {
            let v = a[row][col].abs();
            if v > max_val {
                max_val = v;
                max_row = row;
            }
        }
        if max_val < 1e-12 {
            return None; // Singular
        }
        a.swap(col, max_row);
        b.swap(col, max_row);

        let pivot = a[col][col];
        for row in (col + 1)..n {
            let factor = a[row][col] / pivot;
            for c in col..n {
                let sub = factor * a[col][c];
                a[row][c] -= sub;
            }
            let sub_b = factor * b[col];
            b[row] -= sub_b;
        }
    }

    // Back substitution
    let mut x = vec![0.0_f64; n];
    for i in (0..n).rev() {
        let mut sum = b[i];
        for j in (i + 1)..n {
            sum -= a[i][j] * x[j];
        }
        let diag = a[i][i];
        if diag.abs() < 1e-12 {
            return None;
        }
        x[i] = sum / diag;
    }
    Some(x)
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// 3-bus test network:
    /// Bus 0 (slack), Bus 1 (P_inj=+1.0), Bus 2 (P_inj=-1.0)
    /// Branch 0: 0→1, x=0.1, rating=20.0
    /// Branch 1: 1→2, x=0.1, rating=20.0
    /// Branch 2: 0→2, x=0.2, rating=15.0
    fn make_3bus() -> NetworkData {
        NetworkData::new(
            3,
            3,
            vec![0.0, 1.0, -1.0],   // bus injections
            vec![0, 1, 0],          // from
            vec![1, 2, 2],          // to
            vec![0.1, 0.1, 0.2],    // x_pu
            vec![0.0, 0.0, 0.0],    // r_pu
            vec![20.0, 20.0, 15.0], // rating_mw
            vec![1.0, 1.0, 1.0],    // V_pu
            0,                      // slack
        )
    }

    fn make_analyzer() -> ContingencyAnalyzer {
        ContingencyAnalyzer::new(make_3bus())
    }

    // ── Test 1 ────────────────────────────────────────────────────────────────
    #[test]
    fn test_generate_n1_contingencies() {
        let mut a = make_analyzer();
        a.generate_n1_contingencies();
        assert_eq!(
            a.contingencies.len(),
            3,
            "should generate one contingency per branch"
        );
        for (k, c) in a.contingencies.iter().enumerate() {
            assert_eq!(c.outaged_elements, vec![k]);
            assert_eq!(c.element_type, "branch");
        }
    }

    // ── Test 2 ────────────────────────────────────────────────────────────────
    #[test]
    fn test_dc_pf_base_case() {
        let a = make_analyzer();
        let flows = a.dc_solve_with_inj(&a.network.bus_p_inj, &[]);
        // KCL at bus 1: net power leaving bus 1 = injection at bus 1
        // Branch 0 (0→1): positive p01 means flow into bus 1 (from 0).
        // Convention: p[l] = (θ_from - θ_to)/x, so p01 = (θ0 - θ1)/0.1.
        // With θ0=0, θ1>0: p01 < 0 (flow actually goes 1→0).
        // Flow leaving bus 1: via branch 0 toward bus 0 = -p01
        //                     via branch 1 toward bus 2 = +p12
        let p01 = flows[0]; // branch 0: 0→1
        let p12 = flows[1]; // branch 1: 1→2
        let kcl_bus1 = -p01 + p12; // net power leaving bus 1
        assert!(
            (kcl_bus1 - 1.0).abs() < 1e-6,
            "KCL at bus 1 violated: {kcl_bus1}"
        );
    }

    // ── Test 3 ────────────────────────────────────────────────────────────────
    #[test]
    fn test_contingency_removes_branch() {
        let a = make_analyzer();
        let c = Contingency::new(
            0,
            "test",
            ContingencyType::SingleLineOutage,
            vec![0],
            "branch",
        );
        let result = a.analyze_contingency_dc(&c);
        assert_eq!(
            result.branch_flows_mw[0], 0.0,
            "outaged branch flow should be zero"
        );
    }

    // ── Test 4 ────────────────────────────────────────────────────────────────
    #[test]
    fn test_flow_redistribution() {
        let a = make_analyzer();
        let base_flows = a.dc_solve_with_inj(&a.network.bus_p_inj, &[]);
        let c = Contingency::new(
            0,
            "test",
            ContingencyType::SingleLineOutage,
            vec![0],
            "branch",
        );
        let result = a.analyze_contingency_dc(&c);
        // Branch 1 (1→2) should carry more load after branch 0 outage
        assert!(
            result.branch_flows_mw[1].abs() > base_flows[1].abs() - 1e-6,
            "flow on branch 1 should increase or stay after branch 0 outage"
        );
    }

    // ── Test 5 ────────────────────────────────────────────────────────────────
    #[test]
    fn test_overload_detection() {
        // Make a network where outaging branch 2 (the bypass) causes overload on branch 1
        let mut net = NetworkData::new(
            3,
            3,
            vec![0.0, 5.0, -5.0],
            vec![0, 1, 0],
            vec![1, 2, 2],
            vec![0.1, 0.1, 0.2],
            vec![0.0, 0.0, 0.0],
            vec![100.0, 4.0, 100.0], // branch 1 has rating 4 MW
            vec![1.0; 3],
            0,
        );
        net.ptdf_matrix = None;
        let a = ContingencyAnalyzer::new(net);
        let c = Contingency::new(
            0,
            "test",
            ContingencyType::SingleLineOutage,
            vec![2],
            "branch",
        );
        let result = a.analyze_contingency_dc(&c);
        assert!(
            matches!(
                result.status,
                ContingencyStatus::Overload | ContingencyStatus::Violation
            ),
            "should detect overload, got {:?}",
            result.status
        );
    }

    // ── Test 6 ────────────────────────────────────────────────────────────────
    #[test]
    fn test_warning_detection() {
        // P_inj = 1 MW total; base flow on branch 0 ≈ 0.667 MW
        // Set rating on branch 1 such that post-contingency it's at ~96% of limit
        let mut a = make_analyzer();
        a.overload_warning_pct = 90.0; // lower threshold for reliable test

        // After outaging branch 2 (bypass), all 1 MW flows through branch 0→branch 1
        // branch 1 carries 1.0 MW; set rating just above that
        a.network.branch_rating_mw[1] = 1.05; // 1.0 MW / 1.05 MW = ~95.2%
        let c = Contingency::new(
            0,
            "test",
            ContingencyType::SingleLineOutage,
            vec![2],
            "branch",
        );
        let result = a.analyze_contingency_dc(&c);
        let pct = result.branch_flows_mw[1].abs() / 1.05 * 100.0;
        assert!(pct >= 90.0, "expected warning-level loading, got {pct:.1}%");
    }

    // ── Test 7 ────────────────────────────────────────────────────────────────
    #[test]
    fn test_secure_case() {
        let mut a = make_analyzer();
        // Generous ratings: no branch can overload
        a.network.branch_rating_mw = vec![1000.0, 1000.0, 1000.0];
        let c = Contingency::new(
            0,
            "test",
            ContingencyType::SingleLineOutage,
            vec![2],
            "branch",
        );
        let result = a.analyze_contingency_dc(&c);
        assert_eq!(result.status, ContingencyStatus::Secure);
    }

    // ── Test 8 ────────────────────────────────────────────────────────────────
    #[test]
    fn test_performance_index_zero_flows() {
        let a = make_analyzer();
        let flows = vec![0.0; 3];
        let pi = a.compute_performance_index(&flows);
        assert!(
            (pi - 0.0).abs() < 1e-12,
            "PI of zero flows should be 0, got {pi}"
        );
    }

    // ── Test 9 ────────────────────────────────────────────────────────────────
    #[test]
    fn test_performance_index_at_limit() {
        let a = make_analyzer();
        // flows equal to ratings
        let flows: Vec<f64> = a.network.branch_rating_mw.clone();
        let pi = a.compute_performance_index(&flows);
        let expected = a.network.n_branches as f64;
        assert!(
            (pi - expected).abs() < 1e-9,
            "PI at 100% should equal n_branches={expected}, got {pi}"
        );
    }

    // ── Test 10 ───────────────────────────────────────────────────────────────
    #[test]
    fn test_ptdf_matrix_shape() {
        let a = make_analyzer();
        let ptdf = a.compute_ptdf_matrix();
        assert_eq!(
            ptdf.len(),
            a.network.n_branches,
            "PTDF rows should equal n_branches"
        );
        for row in &ptdf {
            assert_eq!(
                row.len(),
                a.network.n_buses,
                "each PTDF row should have n_buses columns"
            );
        }
    }

    // ── Test 11 ───────────────────────────────────────────────────────────────
    #[test]
    fn test_ptdf_slack_column_zero() {
        // PTDF column for the slack bus must be all zeros (no sensitivity
        // to injection at the slack, since the slack is the reference).
        let a = make_analyzer();
        let ptdf = a.compute_ptdf_matrix();
        let slack = a.network.slack_bus;
        for (l, row) in ptdf.iter().enumerate() {
            let v = row.get(slack).copied().unwrap_or(0.0);
            assert!(
                v.abs() < 1e-9,
                "PTDF[{l}][slack={slack}] should be 0, got {v}"
            );
        }
    }

    // ── Test 12 ───────────────────────────────────────────────────────────────
    #[test]
    fn test_lodf_formula() {
        let a = make_analyzer();
        // Self-LODF: LODF(k, k) should be -1 (line always takes its own flow)
        let lodf_self = a.compute_lodf(0, 0);
        assert!(
            (lodf_self - (-1.0)).abs() < 1e-6,
            "self-LODF should be -1, got {lodf_self}"
        );
    }

    // ── Test 13 ───────────────────────────────────────────────────────────────
    #[test]
    fn test_screen_with_ptdf() {
        let mut a = make_analyzer();
        a.generate_n1_contingencies();
        let screen = a.screen_with_ptdf();
        assert_eq!(
            screen.len(),
            a.contingencies.len(),
            "one screen result per contingency"
        );
    }

    // ── Test 14 ───────────────────────────────────────────────────────────────
    #[test]
    fn test_pi_threshold_flagging() {
        let mut a = make_analyzer();
        a.pi_threshold = 0.0; // everything flagged
        a.generate_n1_contingencies();
        let screen = a.screen_with_ptdf();
        // At least one non-trivial contingency should be flagged
        let flagged = screen.iter().filter(|s| s.needs_full_analysis).count();
        // With pi_threshold=0 everything with any flow should be flagged
        assert!(flagged > 0, "expected at least one flagged contingency");
    }

    // ── Test 15 ───────────────────────────────────────────────────────────────
    #[test]
    fn test_find_worst_case() {
        let a = make_analyzer();
        let r1 = ContingencyResult {
            contingency_id: 0,
            status: ContingencyStatus::Warning,
            max_overload_pct: 80.0,
            overloaded_branches: vec![],
            voltage_violations: vec![],
            total_load_shed_mw: 0.0,
            bus_angles: vec![],
            branch_flows_mw: vec![],
            computation_time_ms: 0.0,
        };
        let r2 = ContingencyResult {
            contingency_id: 1,
            status: ContingencyStatus::Overload,
            max_overload_pct: 150.0,
            overloaded_branches: vec![(0, 150.0)],
            voltage_violations: vec![],
            total_load_shed_mw: 0.0,
            bus_angles: vec![],
            branch_flows_mw: vec![],
            computation_time_ms: 0.0,
        };
        let results = vec![r1, r2];
        let worst = a.find_worst_case(&results).expect("should find worst case");
        assert_eq!(worst.contingency_id, 1);
        assert!((worst.max_overload_pct - 150.0).abs() < 1e-9);
    }

    // ── Test 16 ───────────────────────────────────────────────────────────────
    #[test]
    fn test_analyze_all_dc() {
        let mut a = make_analyzer();
        a.generate_n1_contingencies();
        let results = a.analyze_all_dc();
        assert_eq!(results.len(), a.contingencies.len());
    }

    // ── Test 17 ───────────────────────────────────────────────────────────────
    #[test]
    fn test_islanding_detection() {
        // 2-bus radial: Bus 0 (slack) -- Branch 0 --> Bus 1 (load)
        // Outage branch 0 disconnects Bus 1
        let net = NetworkData::new(
            2,
            1,
            vec![0.0, -1.0],
            vec![0],
            vec![1],
            vec![0.1],
            vec![0.0],
            vec![10.0],
            vec![1.0, 1.0],
            0,
        );
        let a = ContingencyAnalyzer::new(net);
        let c = Contingency::new(
            0,
            "radial outage",
            ContingencyType::SingleLineOutage,
            vec![0],
            "branch",
        );
        let result = a.analyze_contingency_dc(&c);
        assert_eq!(
            result.status,
            ContingencyStatus::Islanding,
            "disconnected bus should cause islanding status"
        );
    }

    // ── Test 18 ───────────────────────────────────────────────────────────────
    #[test]
    fn test_3bus_system_n1() {
        // 3-bus: Bus1(+1MW gen) -- Branch1(1→2,x=0.1) -- Bus2(-1MW load)
        //        Bus0(slack) -- Branch2(0→2,x=0.2) -- Bus2
        //        Bus0(slack) -- Branch0(0→1,x=0.1) -- Bus1
        // After outaging branch 0 (0→1):
        //   Bus1 is connected to Bus2 via Branch1
        //   Bus0 is connected to Bus2 via Branch2
        //   The 1MW gen at Bus1 flows directly to Bus2 via Branch1.
        //   Slack bus 0 has zero net injection, Branch2 carries 0.
        let a = make_analyzer();
        let base = a.dc_solve_with_inj(&a.network.bus_p_inj, &[]);
        let c = Contingency::new(
            0,
            "n1-b0",
            ContingencyType::SingleLineOutage,
            vec![0],
            "branch",
        );
        let result = a.analyze_contingency_dc(&c);

        assert_eq!(
            result.branch_flows_mw[0], 0.0,
            "outaged branch must be zero"
        );
        // Branch 1 (1→2) should carry all 1 MW after branch 0 is removed
        assert!(
            result.branch_flows_mw[1].abs() > base[1].abs() - 1e-6,
            "branch 1 (1→2) should carry at least as much after branch 0 outage"
        );
        // Total power balance: KCL satisfied (no NaN flows on active branches)
        assert!(
            result.branch_flows_mw[1].is_finite() && result.branch_flows_mw[2].is_finite(),
            "flows must be finite (no islanding)"
        );
        // Verify the result is not islanding
        assert_ne!(result.status, ContingencyStatus::Islanding);
    }

    // ── Test 19 ───────────────────────────────────────────────────────────────
    #[test]
    fn test_single_element_outage() {
        // Bus outage: outage bus 1 removes branches 0 (0→1) and 1 (1→2)
        let a = make_analyzer();
        let c = Contingency::new(0, "bus1-outage", ContingencyType::BusOutage, vec![1], "bus");
        let result = a.analyze_contingency_dc(&c);
        // Both branches incident to bus 1 should be zero
        assert_eq!(
            result.branch_flows_mw[0], 0.0,
            "branch 0 should be zero (incident to bus 1)"
        );
        assert_eq!(
            result.branch_flows_mw[1], 0.0,
            "branch 1 should be zero (incident to bus 1)"
        );
    }

    // ── Test 20 ───────────────────────────────────────────────────────────────
    #[test]
    fn test_classify_status_ordering() {
        let a = make_analyzer();

        let secure_result = ContingencyResult {
            contingency_id: 0,
            status: ContingencyStatus::Secure,
            max_overload_pct: 50.0,
            overloaded_branches: vec![],
            voltage_violations: vec![],
            total_load_shed_mw: 0.0,
            bus_angles: vec![],
            branch_flows_mw: vec![],
            computation_time_ms: 0.0,
        };
        let warning_result = ContingencyResult {
            max_overload_pct: 96.0,
            ..secure_result.clone()
        };
        let overload_result = ContingencyResult {
            max_overload_pct: 110.0,
            overloaded_branches: vec![(0, 110.0)],
            ..secure_result.clone()
        };

        assert_eq!(
            a.classify_contingency(&secure_result),
            ContingencyStatus::Secure
        );
        assert_eq!(
            a.classify_contingency(&warning_result),
            ContingencyStatus::Warning
        );
        assert_eq!(
            a.classify_contingency(&overload_result),
            ContingencyStatus::Overload
        );
    }

    // ── Test 21 ───────────────────────────────────────────────────────────────
    #[test]
    fn test_add_contingency() {
        let mut a = make_analyzer();
        assert_eq!(a.contingencies.len(), 0);
        a.add_contingency(Contingency::new(
            42,
            "manual",
            ContingencyType::GeneratorOutage,
            vec![0],
            "generator",
        ));
        assert_eq!(a.contingencies.len(), 1);
        assert_eq!(a.contingencies[0].id, 42);
    }

    // ── Test 22 ───────────────────────────────────────────────────────────────
    #[test]
    fn test_contingency_result_fields() {
        let a = make_analyzer();
        let c = Contingency::new(
            7,
            "field test",
            ContingencyType::SingleLineOutage,
            vec![1],
            "branch",
        );
        let result = a.analyze_contingency_dc(&c);
        assert_eq!(result.contingency_id, 7);
        assert_eq!(
            result.branch_flows_mw[1], 0.0,
            "outaged branch flow must be 0"
        );
        assert_eq!(result.bus_angles.len(), a.network.n_buses);
        assert_eq!(result.branch_flows_mw.len(), a.network.n_branches);
        assert!(result.computation_time_ms >= 0.0);
    }
}
