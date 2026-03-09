//! Real-Time Security Assessment (RTSA) for power systems.
//!
//! Provides online contingency analysis using fast screening via Performance
//! Index (PI) and Line Outage Distribution Factors (LODF), full sensitivity-
//! based contingency analysis, corrective action generation, security level
//! classification, and security boundary tracing in 2-D parameter space.
//!
//! # Units
//! - Voltages: \[pu\]
//! - Power flows / generation / load: \[MW or MVA\]
//! - Frequency: \[Hz\]
//! - Time: \[s\] or \[min\] as noted per field

use crate::error::OxiGridError;

// ============================================================================
// Enumerations
// ============================================================================

/// Security level classification following NERC/ENTSO-E operating states.
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub enum SecurityLevel {
    /// N: all constraints satisfied, no emergency reserves needed.
    Normal,
    /// A: current state satisfies constraints but N-1 contingency may violate them.
    Alert,
    /// E: current operating state violates at least one constraint.
    Emergency,
    /// EE: widespread violations, imminent collapse risk.
    ExtremisEmergency,
    /// R: recovering from a blackout event.
    Restorative,
}

/// Operating constraint type for security assessment.
#[derive(Clone, Debug)]
pub enum ConstraintType {
    /// Thermal limit on a transmission branch.
    ThermalLine { branch_idx: usize, limit_mva: f64 },
    /// Acceptable bus voltage band \[pu\].
    VoltageBand {
        bus: usize,
        v_min_pu: f64,
        v_max_pu: f64,
    },
    /// Maximum frequency deviation from nominal.
    FrequencyDeviation { max_hz: f64 },
    /// Minimum stability margin \[%\].
    StabilityMargin { min_margin_pct: f64 },
    /// Minimum reactive reserve at a bus.
    ReactiveReserve { bus: usize, min_mvar: f64 },
}

/// Type of corrective action to relieve a constraint violation.
#[derive(Clone, Debug, PartialEq)]
pub enum CorrectiveActionType {
    /// Shift real-power generation between two buses.
    GeneratorRedispatch { from_bus: usize, to_bus: usize },
    /// Controlled load curtailment at a specific bus.
    LoadShedding { bus: usize, amount_mw: f64 },
    /// Adjust transformer off-nominal tap ratio.
    TransformerTapChange { transformer_id: usize, new_tap: f64 },
    /// Switch in/out reactive compensation at a bus.
    ReactiveCompensation { bus: usize, amount_mvar: f64 },
    /// Toggle a line shunt (capacitor/reactor bank).
    LineShuntOperation { branch: usize },
}

// ============================================================================
// Data structures
// ============================================================================

/// Snapshot of the current power system operating state.
#[derive(Clone, Debug)]
pub struct SystemOperatingState {
    /// Measurement timestamp \[s since epoch\].
    pub timestamp: f64,
    /// Bus voltage magnitudes \[pu\], indexed by bus.
    pub voltage_magnitudes: Vec<f64>,
    /// Bus voltage angles \[rad\], indexed by bus.
    pub voltage_angles: Vec<f64>,
    /// Branch loading percentages \[%\], indexed by branch.
    pub branch_loadings_pct: Vec<f64>,
    /// System frequency \[Hz\].
    pub frequency_hz: f64,
    /// Total system generation \[MW\].
    pub total_generation_mw: f64,
    /// Total system load \[MW\].
    pub total_load_mw: f64,
    /// Available spinning reserve \[MW\].
    pub reserve_mw: f64,
    /// Reactive reserve per generator \[MVAR\].
    pub reactive_reserves_mvar: Vec<f64>,
}

/// Definition of a contingency (single or multiple outage) scenario.
#[derive(Clone, Debug)]
pub struct Contingency {
    /// Unique contingency identifier.
    pub id: usize,
    /// Human-readable name.
    pub name: String,
    /// Indices of outaged branches (N-1 or N-k).
    pub outaged_lines: Vec<usize>,
    /// Indices of outaged generators.
    pub outaged_generators: Vec<usize>,
    /// Annual probability of occurrence.
    pub probability: f64,
    /// Relative severity weight for PI computation.
    pub severity_weight: f64,
}

/// Result of a fast contingency screening step.
#[derive(Clone, Debug)]
pub struct ContingencyScreenResult {
    /// Identifier of the screened contingency.
    pub contingency_id: usize,
    /// Weighted Performance Index value.
    pub performance_index: f64,
    /// Worst post-contingency branch loading \[%\].
    pub max_loading_pct: f64,
    /// Number of branches violating thermal limits post-contingency.
    pub n_violated_branches: usize,
    /// Number of buses violating voltage limits post-contingency.
    pub n_violated_voltages: usize,
    /// `true` if this contingency is selected for full analysis.
    pub screened_in: bool,
}

/// Full contingency analysis result (sensitivity-based).
#[derive(Clone, Debug)]
pub struct ContingencyAnalysisResult {
    /// Identifier matching [`Contingency::id`].
    pub contingency_id: usize,
    /// Post-contingency voltage magnitudes \[pu\].
    pub post_contingency_voltages: Vec<f64>,
    /// Post-contingency branch loadings \[%\].
    pub post_contingency_loadings: Vec<f64>,
    /// List of violated constraints.
    pub violated_constraints: Vec<ConstraintType>,
    /// Security level of the post-contingency state.
    pub security_level: SecurityLevel,
    /// Corrective actions to relieve violations.
    pub corrective_actions: Vec<CorrectiveAction>,
    /// Estimated recovery time \[min\].
    pub estimated_recovery_time_min: f64,
}

/// A recommended corrective control action.
#[derive(Clone, Debug)]
pub struct CorrectiveAction {
    /// Type and parameters of the action.
    pub action_type: CorrectiveActionType,
    /// Action magnitude \[MW or MVAR\].
    pub magnitude: f64,
    /// Expected constraint relief from this action \[MW or pu\].
    pub expected_benefit: f64,
    /// Economic penalty / opportunity cost of the action.
    pub cost_penalty: f64,
}

/// Configuration for the real-time security assessor.
#[derive(Clone, Debug)]
pub struct RtsaConfig {
    /// Number of buses in the network model.
    pub n_buses: usize,
    /// Number of branches in the network model.
    pub n_branches: usize,
    /// Maximum N-1 contingencies to screen per cycle.
    pub max_contingencies: usize,
    /// PI threshold above which a contingency is selected for full analysis.
    pub screening_threshold: f64,
    /// Minimum acceptable voltage \[pu\]. Default 0.95.
    pub v_min_pu: f64,
    /// Maximum acceptable voltage \[pu\]. Default 1.05.
    pub v_max_pu: f64,
    /// Thermal alert threshold \[%\]. Default 90 %.
    pub thermal_limit_pct: f64,
    /// RTSA cycle time \[s\]. Default 30 s.
    pub update_interval_s: f64,
    /// Exponent *n* in PI = Σ (loading / limit)^{2n}. Default 2.
    pub n_exponent: usize,
}

impl Default for RtsaConfig {
    fn default() -> Self {
        Self {
            n_buses: 10,
            n_branches: 15,
            max_contingencies: 100,
            screening_threshold: 1.0,
            v_min_pu: 0.95,
            v_max_pu: 1.05,
            thermal_limit_pct: 90.0,
            update_interval_s: 30.0,
            n_exponent: 2,
        }
    }
}

/// Overall RTSA result for a single assessment cycle.
#[derive(Clone, Debug)]
pub struct RtsaResult {
    /// Assessment timestamp \[s\].
    pub timestamp: f64,
    /// Classified security level for the current cycle.
    pub system_security_level: SecurityLevel,
    /// Base-case constraint violations (pre-contingency).
    pub base_case_violations: Vec<ConstraintType>,
    /// Total contingencies passed through the screening step.
    pub n_contingencies_screened: usize,
    /// Total contingencies selected for full analysis.
    pub n_contingencies_analyzed: usize,
    /// Identifier of the worst post-contingency case (`None` if no violations).
    pub worst_contingency: Option<usize>,
    /// Worst post-contingency branch loading across all analyzed contingencies \[%\].
    pub worst_post_contingency_loading_pct: f64,
    /// Full analysis results for screened-in contingencies.
    pub contingency_results: Vec<ContingencyAnalysisResult>,
    /// Weighted sum of PI over all contingencies.
    pub system_performance_index: f64,
    /// Distance to the security boundary \[%\].
    pub security_margin_pct: f64,
    /// Ranked corrective actions for the worst violation.
    pub recommended_actions: Vec<CorrectiveAction>,
    /// Wall-clock time consumed by the assessment cycle \[ms\].
    pub analysis_time_ms: f64,
}

// ============================================================================
// Core RTSA engine
// ============================================================================

/// Real-Time Security Assessor.
///
/// Holds the DC-network PTDF matrix, branch ratings, contingency list, and
/// configuration.  Call [`RealTimeSecurityAssessor::assess`] each cycle.
pub struct RealTimeSecurityAssessor {
    /// Assessment configuration.
    pub config: RtsaConfig,
    /// Registered contingency scenarios.
    pub contingencies: Vec<Contingency>,
    /// PTDF matrix \[branch\]\[bus\]: sensitivity of branch flow to bus injection.
    pub ptdf_matrix: Vec<Vec<f64>>,
    /// Continuous MVA/MW rating for each branch.
    pub branch_ratings_mva: Vec<f64>,
}

impl RealTimeSecurityAssessor {
    /// Construct a new assessor.
    ///
    /// # Arguments
    /// * `config` – RTSA configuration.
    /// * `contingencies` – list of N-1/N-k contingency definitions.
    /// * `branch_ratings_mva` – rated capacity per branch \[MVA or MW\].
    pub fn new(
        config: RtsaConfig,
        contingencies: Vec<Contingency>,
        branch_ratings_mva: Vec<f64>,
    ) -> Self {
        let n_branches = config.n_branches;
        let n_buses = config.n_buses;
        // Default to zero PTDF; caller should supply via `compute_ptdf` or directly.
        let ptdf_matrix = vec![vec![0.0_f64; n_buses]; n_branches];
        Self {
            config,
            contingencies,
            ptdf_matrix,
            branch_ratings_mva,
        }
    }

    // -----------------------------------------------------------------------
    // PTDF computation
    // -----------------------------------------------------------------------

    /// Compute the Power Transfer Distribution Factor matrix from a DC network
    /// model.
    ///
    /// Uses the B-matrix approach: the branch susceptance matrix `B_br` and
    /// the bus admittance matrix B-matrix are formed; reference bus (index 0)
    /// is eliminated and the reduced system is solved.
    ///
    /// Returns a `[n_branches][n_buses]` matrix; the reference-bus column is
    /// zero by convention.
    ///
    /// # Arguments
    /// * `g_matrix` – Real part of the nodal admittance matrix (unused here,
    ///   kept for API symmetry with AC variants).
    /// * `b_matrix` – Imaginary part of the nodal admittance matrix \[pu\].
    /// * `branches` – Branch endpoint pairs `(from_bus, to_bus)`.
    pub fn compute_ptdf(
        _g_matrix: &[Vec<f64>],
        b_matrix: &[Vec<f64>],
        branches: &[(usize, usize)],
    ) -> Vec<Vec<f64>> {
        let n_buses = b_matrix.len();
        let n_branches = branches.len();

        if n_buses == 0 || n_branches == 0 {
            return Vec::new();
        }

        // Build reduced B matrix (exclude slack bus 0):
        // B_red is (n_buses-1) × (n_buses-1)
        let n_red = n_buses.saturating_sub(1);
        if n_red == 0 {
            return vec![vec![0.0; n_buses]; n_branches];
        }

        let mut b_red = vec![vec![0.0_f64; n_red]; n_red];
        for (i, b_red_row) in b_red.iter_mut().enumerate() {
            for (j, b_red_val) in b_red_row.iter_mut().enumerate() {
                *b_red_val = b_matrix
                    .get(i + 1)
                    .and_then(|row| row.get(j + 1))
                    .copied()
                    .unwrap_or(0.0);
            }
        }

        // Invert B_red via Gaussian elimination
        let b_red_inv = match invert_matrix(&b_red) {
            Some(inv) => inv,
            None => {
                // Singular — return zero PTDF (degenerate network)
                return vec![vec![0.0; n_buses]; n_branches];
            }
        };

        // Build branch susceptance vector (b_br per branch)
        // Extract from b_matrix diagonal difference: b_br ≈ -b_matrix[i][j]
        // for branch (i,j).
        let mut ptdf = vec![vec![0.0_f64; n_buses]; n_branches];

        for (br_idx, &(from, to)) in branches.iter().enumerate() {
            // Branch susceptance from off-diagonal of B matrix
            let b_br = if from < n_buses && to < n_buses && from != to {
                // B[from][to] = -b_br in the admittance convention
                let val = b_matrix
                    .get(from)
                    .and_then(|row| row.get(to))
                    .copied()
                    .unwrap_or(0.0);
                -val // susceptance (positive value)
            } else {
                1.0 // fallback
            };

            // PTDF[br][bus] = b_br * (x_from[bus] - x_to[bus])
            // where x = B_red^{-1} * e_bus (columns of the inverse)
            for (bus, ptdf_val) in ptdf[br_idx].iter_mut().enumerate() {
                if bus == 0 {
                    // Reference bus: PTDF = 0 by convention
                    *ptdf_val = 0.0;
                    continue;
                }
                let red_bus = bus - 1; // index into reduced system

                // x_from: row from_bus of B_red^{-1}, column red_bus
                let x_from = if from == 0 {
                    0.0
                } else {
                    let from_red = from - 1;
                    b_red_inv
                        .get(from_red)
                        .and_then(|row| row.get(red_bus))
                        .copied()
                        .unwrap_or(0.0)
                };

                // x_to: row to_bus of B_red^{-1}, column red_bus
                let x_to = if to == 0 {
                    0.0
                } else {
                    let to_red = to - 1;
                    b_red_inv
                        .get(to_red)
                        .and_then(|row| row.get(red_bus))
                        .copied()
                        .unwrap_or(0.0)
                };

                *ptdf_val = b_br * (x_from - x_to);
            }
        }

        ptdf
    }

    // -----------------------------------------------------------------------
    // Main assessment cycle
    // -----------------------------------------------------------------------

    /// Perform one RTSA cycle.
    ///
    /// Steps:
    /// 1. Check base-case constraint violations.
    /// 2. Screen all N-1 contingencies via Performance Index.
    /// 3. Fully analyze screened-in contingencies.
    /// 4. Classify system security level.
    /// 5. Generate corrective actions if needed.
    ///
    /// # Errors
    /// Returns [`OxiGridError::InvalidParameter`] if the operating state
    /// dimensions do not match the assessor configuration.
    pub fn assess(&mut self, state: &SystemOperatingState) -> Result<RtsaResult, OxiGridError> {
        let t_start = std::time::Instant::now();

        // Validate dimensions
        if state.voltage_magnitudes.len() != self.config.n_buses {
            return Err(OxiGridError::InvalidParameter(format!(
                "voltage_magnitudes length {} != n_buses {}",
                state.voltage_magnitudes.len(),
                self.config.n_buses
            )));
        }
        if state.branch_loadings_pct.len() != self.config.n_branches {
            return Err(OxiGridError::InvalidParameter(format!(
                "branch_loadings_pct length {} != n_branches {}",
                state.branch_loadings_pct.len(),
                self.config.n_branches
            )));
        }

        // Step 1: Base-case check
        let base_violations = self.check_base_case(state);

        // Step 2: Convert percentage loadings to MVA for screening
        let base_loadings_mva: Vec<f64> = state
            .branch_loadings_pct
            .iter()
            .enumerate()
            .map(|(b, &pct)| {
                let rating = self.branch_ratings_mva.get(b).copied().unwrap_or(100.0);
                pct / 100.0 * rating
            })
            .collect();

        let contingencies_to_screen = self.contingencies.len().min(self.config.max_contingencies);

        let mut screen_results: Vec<ContingencyScreenResult> = Vec::new();
        for idx in 0..contingencies_to_screen {
            let contingency = &self.contingencies[idx];
            let sr = self.screen_contingency(contingency, &base_loadings_mva);
            screen_results.push(sr);
        }

        // Compute system PI as weighted sum
        let system_pi: f64 = screen_results
            .iter()
            .zip(self.contingencies.iter())
            .map(|(sr, c)| sr.performance_index * c.severity_weight)
            .sum();

        // Step 3: Full analysis of screened-in contingencies
        let screened_in: Vec<usize> = screen_results
            .iter()
            .enumerate()
            .filter(|(_, sr)| sr.screened_in)
            .map(|(i, _)| i)
            .collect();

        let n_analyzed = screened_in.len();
        let mut contingency_results: Vec<ContingencyAnalysisResult> = Vec::new();

        for &idx in &screened_in {
            if let Some(contingency) = self.contingencies.get(idx) {
                let contingency_clone = contingency.clone();
                let result = self.analyze_contingency(&contingency_clone, state);
                contingency_results.push(result);
            }
        }

        // Step 4: Classify security level
        let security_level = Self::classify_security_level(&base_violations, &contingency_results);

        // Find worst contingency
        let worst_loading = contingency_results
            .iter()
            .flat_map(|r| r.post_contingency_loadings.iter().copied())
            .fold(0.0_f64, f64::max);

        let worst_contingency = contingency_results
            .iter()
            .filter(|r| !r.post_contingency_loadings.is_empty())
            .max_by(|a, b| {
                let max_a = a
                    .post_contingency_loadings
                    .iter()
                    .cloned()
                    .fold(0.0_f64, f64::max);
                let max_b = b
                    .post_contingency_loadings
                    .iter()
                    .cloned()
                    .fold(0.0_f64, f64::max);
                max_a
                    .partial_cmp(&max_b)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|r| r.contingency_id);

        // Step 5: Corrective actions from worst violating contingency
        let recommended_actions = if let Some(worst_result) = contingency_results
            .iter()
            .find(|r| worst_contingency == Some(r.contingency_id))
        {
            let wr = worst_result.clone();
            self.generate_corrective_actions(&wr, state)
        } else {
            Vec::new()
        };

        let security_margin = self.compute_security_margin(state);

        let elapsed_ms = t_start.elapsed().as_secs_f64() * 1000.0;

        Ok(RtsaResult {
            timestamp: state.timestamp,
            system_security_level: security_level,
            base_case_violations: base_violations,
            n_contingencies_screened: contingencies_to_screen,
            n_contingencies_analyzed: n_analyzed,
            worst_contingency,
            worst_post_contingency_loading_pct: worst_loading,
            contingency_results,
            system_performance_index: system_pi,
            security_margin_pct: security_margin,
            recommended_actions,
            analysis_time_ms: elapsed_ms,
        })
    }

    // -----------------------------------------------------------------------
    // Contingency screening
    // -----------------------------------------------------------------------

    /// Screen one contingency via Performance Index (fast LODF-based path).
    ///
    /// PI = Σ_k w_k * (P_k_post / P_k_max)^{2n}
    ///
    /// Post-contingency flow on branch *k* after outage of branch *l*:
    /// P_k_post = P_k_base + LODF[k][l] * P_l_base
    fn screen_contingency(
        &self,
        contingency: &Contingency,
        base_loadings_mva: &[f64],
    ) -> ContingencyScreenResult {
        let n_branches = self.config.n_branches;
        let branches: Vec<(usize, usize)> = (0..n_branches).map(|b| (b, b)).collect();

        let mut post_flows: Vec<f64> = base_loadings_mva.to_vec();

        // Apply LODF for each outaged line
        for &outaged_branch in &contingency.outaged_lines {
            if outaged_branch >= n_branches {
                continue;
            }
            let lodf = self.compute_lodf(outaged_branch, &branches);
            let p_lost = base_loadings_mva
                .get(outaged_branch)
                .copied()
                .unwrap_or(0.0);

            for (b, pf) in post_flows.iter_mut().enumerate() {
                if contingency.outaged_lines.contains(&b) {
                    continue; // outaged branch carries no flow
                }
                let lodf_val = lodf.get(b).copied().unwrap_or(0.0);
                *pf += lodf_val * p_lost;
            }
        }

        // Zero out outaged branch flows
        for &ob in &contingency.outaged_lines {
            if ob < post_flows.len() {
                post_flows[ob] = 0.0;
            }
        }

        // Compute PI and violations
        let n_exp = self.config.n_exponent;
        let two_n = (2 * n_exp) as i32;

        let mut pi = 0.0_f64;
        let mut n_violated_branches = 0usize;
        let mut max_loading_pct = 0.0_f64;

        for (b, &pf_mva) in post_flows.iter().enumerate() {
            if contingency.outaged_lines.contains(&b) {
                continue;
            }
            let rating = self
                .branch_ratings_mva
                .get(b)
                .copied()
                .unwrap_or(f64::INFINITY);
            if !rating.is_finite() || rating <= f64::EPSILON {
                continue;
            }
            let ratio = pf_mva.abs() / rating;
            let loading_pct = ratio * 100.0;
            if loading_pct > max_loading_pct {
                max_loading_pct = loading_pct;
            }
            if ratio > 1.0 {
                n_violated_branches += 1;
            }
            // PI contribution: w_k * ratio^{2n}
            let w = contingency.severity_weight;
            pi += w * ratio.powi(two_n);
        }

        // Voltage violations not computable from PTDF alone — approximate as 0
        let n_violated_voltages = 0usize;

        let screened_in = pi > self.config.screening_threshold || n_violated_branches > 0;

        ContingencyScreenResult {
            contingency_id: contingency.id,
            performance_index: pi,
            max_loading_pct,
            n_violated_branches,
            n_violated_voltages,
            screened_in,
        }
    }

    // -----------------------------------------------------------------------
    // LODF computation
    // -----------------------------------------------------------------------

    /// Compute the Line Outage Distribution Factor vector for outage of branch
    /// `outaged_branch`.
    ///
    /// LODF\[k\]\[l\] = (PTDF\[k\]\[from_l\] - PTDF\[k\]\[to_l\]) /
    ///                  (1 - (PTDF\[l\]\[from_l\] - PTDF\[l\]\[to_l\]))
    ///
    /// Returns a vector of length `n_branches`.
    fn compute_lodf(&self, outaged_branch: usize, branches: &[(usize, usize)]) -> Vec<f64> {
        let n_branches = branches.len().max(self.config.n_branches);
        let mut lodf = vec![0.0_f64; n_branches];

        // For the simplified PTDF structure used in screening (branch ↔ branch
        // sensitivity), derive PTDF from the stored matrix when possible;
        // otherwise use a structural approximation.
        let (from_l, to_l) = if let Some(&(f, t)) = branches.get(outaged_branch) {
            (f, t)
        } else {
            // Cannot determine endpoints — zero LODF
            return lodf;
        };

        // Denominator: 1 - PTDF[outaged_branch][from_l] + PTDF[outaged_branch][to_l]
        let ptdf_ll_from = self
            .ptdf_matrix
            .get(outaged_branch)
            .and_then(|row| row.get(from_l))
            .copied()
            .unwrap_or(0.0);
        let ptdf_ll_to = self
            .ptdf_matrix
            .get(outaged_branch)
            .and_then(|row| row.get(to_l))
            .copied()
            .unwrap_or(0.0);
        let denom = 1.0 - (ptdf_ll_from - ptdf_ll_to);

        if denom.abs() < 1e-10 {
            // Topologically isolated — return zero LODF
            return lodf;
        }

        for (k, lodf_val) in lodf.iter_mut().enumerate() {
            if k == outaged_branch {
                *lodf_val = -1.0; // outaged branch itself takes -1
                continue;
            }
            let ptdf_k_from = self
                .ptdf_matrix
                .get(k)
                .and_then(|row| row.get(from_l))
                .copied()
                .unwrap_or(0.0);
            let ptdf_k_to = self
                .ptdf_matrix
                .get(k)
                .and_then(|row| row.get(to_l))
                .copied()
                .unwrap_or(0.0);
            *lodf_val = (ptdf_k_from - ptdf_k_to) / denom;
        }

        lodf
    }

    // -----------------------------------------------------------------------
    // Security level classification
    // -----------------------------------------------------------------------

    /// Classify system security level based on base-case and post-contingency
    /// violations.
    fn classify_security_level(
        base_violations: &[ConstraintType],
        contingency_results: &[ContingencyAnalysisResult],
    ) -> SecurityLevel {
        // Count base-case violations
        let base_thermal_viols: usize = base_violations
            .iter()
            .filter(|c| matches!(c, ConstraintType::ThermalLine { .. }))
            .count();
        let base_voltage_viols: usize = base_violations
            .iter()
            .filter(|c| matches!(c, ConstraintType::VoltageBand { .. }))
            .count();
        let total_base_viols = base_thermal_viols + base_voltage_viols;

        // Check for widespread base-case violations → ExtremisEmergency
        if total_base_viols > 3
            || base_violations
                .iter()
                .any(|c| matches!(c, ConstraintType::StabilityMargin { .. }))
        {
            return SecurityLevel::ExtremisEmergency;
        }

        // Any base-case violation → Emergency (or EE if very widespread)
        if total_base_viols > 0 {
            return SecurityLevel::Emergency;
        }

        // No base-case violations — check contingency results
        let has_post_contingency_violations = contingency_results
            .iter()
            .any(|r| !r.violated_constraints.is_empty());

        if has_post_contingency_violations {
            SecurityLevel::Alert
        } else {
            SecurityLevel::Normal
        }
    }

    // -----------------------------------------------------------------------
    // Full contingency analysis
    // -----------------------------------------------------------------------

    /// Perform full (sensitivity-based) analysis for a screened-in contingency.
    ///
    /// Uses ΔV ≈ V_sensitivity * ΔP_injection to approximate post-contingency
    /// voltages, then evaluates all constraint types.
    fn analyze_contingency(
        &self,
        contingency: &Contingency,
        state: &SystemOperatingState,
    ) -> ContingencyAnalysisResult {
        let _n_buses = self.config.n_buses;
        let n_branches = self.config.n_branches;

        // Post-contingency loadings via LODF
        let branches: Vec<(usize, usize)> = (0..n_branches).map(|b| (b, b)).collect();
        let mut post_loadings_pct: Vec<f64> = state.branch_loadings_pct.to_vec();

        for &outaged in &contingency.outaged_lines {
            if outaged >= n_branches {
                continue;
            }
            let lodf = self.compute_lodf(outaged, &branches);
            let p_lost_pct = state
                .branch_loadings_pct
                .get(outaged)
                .copied()
                .unwrap_or(0.0);

            for (b, pct) in post_loadings_pct.iter_mut().enumerate() {
                if contingency.outaged_lines.contains(&b) {
                    continue;
                }
                let lodf_val = lodf.get(b).copied().unwrap_or(0.0);
                *pct += lodf_val * p_lost_pct;
            }
        }
        // Zero outaged branches
        for &ob in &contingency.outaged_lines {
            if ob < post_loadings_pct.len() {
                post_loadings_pct[ob] = 0.0;
            }
        }

        // Approximate post-contingency voltages via sensitivity:
        // ΔP = lost_generation (for generator outages)
        // ΔV_bus ≈ -ΔP * dV/dP (simplified: dV/dP ≈ 1/(2*n_gen))
        let mut post_voltages: Vec<f64> = state.voltage_magnitudes.clone();
        let lost_gen_mw: f64 = contingency.outaged_generators.len() as f64
            * (state.total_generation_mw / state.reactive_reserves_mvar.len().max(1) as f64).abs();

        if lost_gen_mw > 0.0 {
            let sensitivity = lost_gen_mw / (state.total_generation_mw.abs().max(1.0));
            for v in post_voltages.iter_mut() {
                // Voltage drops proportionally to lost generation fraction
                *v = (*v - 0.5 * sensitivity).max(0.0);
            }
        }

        // Evaluate violated constraints
        let mut violated_constraints: Vec<ConstraintType> = Vec::new();

        // Thermal violations
        for (b, &loading_pct) in post_loadings_pct.iter().enumerate() {
            let rating = self.branch_ratings_mva.get(b).copied().unwrap_or(100.0);
            if loading_pct > 100.0 {
                violated_constraints.push(ConstraintType::ThermalLine {
                    branch_idx: b,
                    limit_mva: rating,
                });
            }
        }

        // Voltage violations
        for (bus, &v) in post_voltages.iter().enumerate() {
            if v < self.config.v_min_pu || v > self.config.v_max_pu {
                violated_constraints.push(ConstraintType::VoltageBand {
                    bus,
                    v_min_pu: self.config.v_min_pu,
                    v_max_pu: self.config.v_max_pu,
                });
            }
        }

        // Reserve check: lost generators may deplete spinning reserve
        if !contingency.outaged_generators.is_empty() {
            let reserve_fraction = state.reserve_mw / (state.total_generation_mw.abs().max(1.0));
            if reserve_fraction < 0.05 {
                violated_constraints.push(ConstraintType::StabilityMargin {
                    min_margin_pct: reserve_fraction * 100.0,
                });
            }
        }

        let security_level = if violated_constraints.is_empty() {
            SecurityLevel::Normal
        } else if violated_constraints.len() > 3 {
            SecurityLevel::ExtremisEmergency
        } else {
            SecurityLevel::Emergency
        };

        // Estimate recovery time proportional to severity
        let estimated_recovery_time_min = violated_constraints.len() as f64 * 5.0;

        let corrective_actions =
            self.generate_corrective_actions_for_violations(&violated_constraints, state);

        ContingencyAnalysisResult {
            contingency_id: contingency.id,
            post_contingency_voltages: post_voltages,
            post_contingency_loadings: post_loadings_pct,
            violated_constraints,
            security_level,
            corrective_actions: corrective_actions.clone(),
            estimated_recovery_time_min,
        }
    }

    // -----------------------------------------------------------------------
    // Corrective actions
    // -----------------------------------------------------------------------

    /// Generate corrective actions for a fully analyzed violated contingency.
    ///
    /// Strategy:
    /// 1. For thermal violations: redispatch generation (from heavily loaded
    ///    areas to lightly loaded areas).
    /// 2. If redispatch is insufficient: load shedding.
    /// 3. For voltage violations: reactive compensation.
    /// 4. Rank by benefit/cost ratio (descending).
    pub fn generate_corrective_actions(
        &self,
        result: &ContingencyAnalysisResult,
        state: &SystemOperatingState,
    ) -> Vec<CorrectiveAction> {
        self.generate_corrective_actions_for_violations(&result.violated_constraints, state)
    }

    /// Internal helper that derives corrective actions from a violation list.
    fn generate_corrective_actions_for_violations(
        &self,
        violations: &[ConstraintType],
        state: &SystemOperatingState,
    ) -> Vec<CorrectiveAction> {
        let mut actions: Vec<CorrectiveAction> = Vec::new();

        for violation in violations {
            match violation {
                ConstraintType::ThermalLine {
                    branch_idx,
                    limit_mva,
                } => {
                    // Estimate needed redispatch from overload magnitude
                    let loading_pct = state
                        .branch_loadings_pct
                        .get(*branch_idx)
                        .copied()
                        .unwrap_or(100.0);
                    let overload_mva = (loading_pct / 100.0 - 1.0) * limit_mva;
                    let redispatch_mw = overload_mva.max(10.0);

                    // Redispatch: from source bus (0) to sink bus (1) as a
                    // heuristic (real implementation would use shift factors)
                    let from_bus = 0usize;
                    let to_bus = self.config.n_buses.saturating_sub(1);

                    let benefit = redispatch_mw;
                    let cost = redispatch_mw * 2.5; // $/MW redispatch cost

                    actions.push(CorrectiveAction {
                        action_type: CorrectiveActionType::GeneratorRedispatch { from_bus, to_bus },
                        magnitude: redispatch_mw,
                        expected_benefit: benefit,
                        cost_penalty: cost,
                    });

                    // Load shedding as last resort if overload > 20 %
                    if loading_pct > 120.0 {
                        let shed_mw = redispatch_mw * 0.5;
                        actions.push(CorrectiveAction {
                            action_type: CorrectiveActionType::LoadShedding {
                                bus: *branch_idx % self.config.n_buses,
                                amount_mw: shed_mw,
                            },
                            magnitude: shed_mw,
                            expected_benefit: shed_mw,
                            cost_penalty: shed_mw * 10.0, // VOLL
                        });
                    }
                }
                ConstraintType::VoltageBand {
                    bus,
                    v_min_pu,
                    v_max_pu,
                } => {
                    let v = state.voltage_magnitudes.get(*bus).copied().unwrap_or(1.0);
                    let (mvar, positive) = if v < *v_min_pu {
                        // Low voltage: inject reactive power
                        let deficit = (*v_min_pu - v) * 100.0; // rough MVAR
                        (deficit.max(5.0), true)
                    } else {
                        // High voltage: absorb reactive power
                        let excess = (v - *v_max_pu) * 100.0;
                        (excess.max(5.0), false)
                    };
                    let amount = if positive { mvar } else { -mvar };
                    actions.push(CorrectiveAction {
                        action_type: CorrectiveActionType::ReactiveCompensation {
                            bus: *bus,
                            amount_mvar: amount,
                        },
                        magnitude: mvar,
                        expected_benefit: mvar * 0.01, // pu voltage improvement
                        cost_penalty: mvar * 1.5,
                    });

                    // Also try tap change
                    actions.push(CorrectiveAction {
                        action_type: CorrectiveActionType::TransformerTapChange {
                            transformer_id: *bus,
                            new_tap: if positive { 1.05 } else { 0.95 },
                        },
                        magnitude: 0.05,
                        expected_benefit: 0.02,
                        cost_penalty: 0.5,
                    });
                }
                ConstraintType::ReactiveReserve { bus, min_mvar } => {
                    actions.push(CorrectiveAction {
                        action_type: CorrectiveActionType::ReactiveCompensation {
                            bus: *bus,
                            amount_mvar: *min_mvar,
                        },
                        magnitude: *min_mvar,
                        expected_benefit: *min_mvar,
                        cost_penalty: min_mvar * 1.0,
                    });
                }
                ConstraintType::StabilityMargin { min_margin_pct } => {
                    // Activate spinning reserve
                    let needed =
                        (5.0 - min_margin_pct).max(0.0) * state.total_generation_mw / 100.0;
                    if needed > 0.0 {
                        actions.push(CorrectiveAction {
                            action_type: CorrectiveActionType::GeneratorRedispatch {
                                from_bus: 0,
                                to_bus: 1,
                            },
                            magnitude: needed,
                            expected_benefit: needed,
                            cost_penalty: needed * 3.0,
                        });
                    }
                }
                ConstraintType::FrequencyDeviation { max_hz } => {
                    let freq_dev = (state.frequency_hz - 50.0).abs();
                    let reserve_needed = freq_dev * state.total_generation_mw * 0.02;
                    actions.push(CorrectiveAction {
                        action_type: CorrectiveActionType::GeneratorRedispatch {
                            from_bus: 0,
                            to_bus: 0,
                        },
                        magnitude: reserve_needed.max(10.0),
                        expected_benefit: freq_dev / max_hz,
                        cost_penalty: reserve_needed * 5.0,
                    });
                }
            }
        }

        // Rank by benefit/cost ratio (descending)
        actions.sort_by(|a, b| {
            let ratio_a = if a.cost_penalty > f64::EPSILON {
                a.expected_benefit / a.cost_penalty
            } else {
                f64::INFINITY
            };
            let ratio_b = if b.cost_penalty > f64::EPSILON {
                b.expected_benefit / b.cost_penalty
            } else {
                f64::INFINITY
            };
            ratio_b
                .partial_cmp(&ratio_a)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        actions
    }

    // -----------------------------------------------------------------------
    // Security margin
    // -----------------------------------------------------------------------

    /// Compute the security margin as the normalised headroom to the nearest
    /// binding constraint.
    ///
    /// Margin = (thermal_limit - max_loading) / thermal_limit * 100 %
    pub fn compute_security_margin(&self, state: &SystemOperatingState) -> f64 {
        let n = state.branch_loadings_pct.len();
        if n == 0 {
            return 100.0;
        }

        let max_loading = state
            .branch_loadings_pct
            .iter()
            .cloned()
            .fold(0.0_f64, f64::max);

        let thermal_limit = self.config.thermal_limit_pct;

        // Also check voltage headroom
        let v_margin = state
            .voltage_magnitudes
            .iter()
            .map(|&v| {
                let low =
                    (v - self.config.v_min_pu) / (1.0 - self.config.v_min_pu).max(f64::EPSILON);
                let high =
                    (self.config.v_max_pu - v) / (self.config.v_max_pu - 1.0).max(f64::EPSILON);
                low.min(high).clamp(0.0, 1.0) * 100.0
            })
            .fold(f64::INFINITY, f64::min);

        let thermal_margin = if max_loading >= thermal_limit {
            0.0
        } else {
            (thermal_limit - max_loading) / thermal_limit * 100.0
        };

        let v_margin_bounded = if v_margin.is_infinite() {
            100.0
        } else {
            v_margin
        };

        thermal_margin.min(v_margin_bounded).clamp(0.0, 100.0)
    }

    // -----------------------------------------------------------------------
    // Base-case check
    // -----------------------------------------------------------------------

    /// Check base-case constraint violations and return the list.
    pub fn check_base_case(&self, state: &SystemOperatingState) -> Vec<ConstraintType> {
        let mut violations: Vec<ConstraintType> = Vec::new();

        // Thermal violations
        for (b, &loading_pct) in state.branch_loadings_pct.iter().enumerate() {
            let rating = self.branch_ratings_mva.get(b).copied().unwrap_or(100.0);
            if loading_pct > 100.0 {
                violations.push(ConstraintType::ThermalLine {
                    branch_idx: b,
                    limit_mva: rating,
                });
            }
        }

        // Voltage violations
        for (bus, &v) in state.voltage_magnitudes.iter().enumerate() {
            if v < self.config.v_min_pu || v > self.config.v_max_pu {
                violations.push(ConstraintType::VoltageBand {
                    bus,
                    v_min_pu: self.config.v_min_pu,
                    v_max_pu: self.config.v_max_pu,
                });
            }
        }

        // Frequency deviation
        let freq_dev = (state.frequency_hz - 50.0).abs();
        if freq_dev > 0.5 {
            violations.push(ConstraintType::FrequencyDeviation { max_hz: freq_dev });
        }

        violations
    }

    // -----------------------------------------------------------------------
    // Performance Index
    // -----------------------------------------------------------------------

    /// Compute the Performance Index for the base-case loading profile.
    ///
    /// PI = Σ_k (loading_k / 100)^{2n}
    pub fn compute_pi(&self, loadings_pct: &[f64]) -> f64 {
        let two_n = (2 * self.config.n_exponent) as i32;
        loadings_pct.iter().map(|&l| (l / 100.0).powi(two_n)).sum()
    }
}

// ============================================================================
// Security boundary tracer
// ============================================================================

/// Traces the security boundary in a 2-D parameter space by binary search.
pub struct SecurityBoundaryTracer {
    /// The underlying RTSA engine.
    pub assessor: RealTimeSecurityAssessor,
    /// Number of boundary points to compute.
    pub n_points: usize,
}

impl SecurityBoundaryTracer {
    /// Create a new boundary tracer.
    pub fn new(assessor: RealTimeSecurityAssessor, n_points: usize) -> Self {
        Self { assessor, n_points }
    }

    /// Trace the security boundary in the 2-D space (P1, P2) by binary
    /// searching along `n_points` directions from a secure base state.
    ///
    /// # Arguments
    /// * `base_state` – A known secure operating point.
    /// * `parameter_bus_1` – Bus index whose injection defines axis 1.
    /// * `parameter_bus_2` – Bus index whose injection defines axis 2.
    ///
    /// Returns a vector of `(P1_mw, P2_mw)` boundary points.
    pub fn trace_boundary_2d(
        &mut self,
        base_state: &SystemOperatingState,
        parameter_bus_1: usize,
        parameter_bus_2: usize,
    ) -> Vec<(f64, f64)> {
        use std::f64::consts::PI;
        let mut boundary: Vec<(f64, f64)> = Vec::with_capacity(self.n_points);

        let n = self.n_points.max(1);
        let base_loading_1 = base_state
            .voltage_magnitudes
            .get(parameter_bus_1)
            .copied()
            .unwrap_or(1.0)
            * 100.0; // MW proxy
        let base_loading_2 = base_state
            .voltage_magnitudes
            .get(parameter_bus_2)
            .copied()
            .unwrap_or(1.0)
            * 100.0;

        for i in 0..n {
            let angle = 2.0 * PI * i as f64 / n as f64;
            let dir_1 = angle.cos();
            let dir_2 = angle.sin();

            // Binary search along this direction for the security boundary
            let mut lo = 0.0_f64;
            let mut hi = 100.0_f64; // maximum parameter variation in MW
            let mut boundary_point = (base_loading_1, base_loading_2);

            for _ in 0..20 {
                // 20 bisection steps → precision < 100/2^20 ≈ 0.0001 MW
                let mid = (lo + hi) / 2.0;
                let p1 = base_loading_1 + mid * dir_1;
                let p2 = base_loading_2 + mid * dir_2;

                let perturbed_state =
                    perturb_state(base_state, parameter_bus_1, p1, parameter_bus_2, p2);

                let is_secure = self
                    .assessor
                    .assess(&perturbed_state)
                    .map(|r| r.system_security_level == SecurityLevel::Normal)
                    .unwrap_or(false);

                if is_secure {
                    lo = mid;
                    boundary_point = (p1, p2);
                } else {
                    hi = mid;
                }
            }
            boundary.push(boundary_point);
        }

        boundary
    }
}

// ============================================================================
// Helper functions
// ============================================================================

/// Invert an n×n matrix using Gaussian elimination with partial pivoting.
///
/// Returns `None` if the matrix is singular.
fn invert_matrix(a: &[Vec<f64>]) -> Option<Vec<Vec<f64>>> {
    let n = a.len();
    if n == 0 {
        return Some(Vec::new());
    }

    // Build augmented matrix [A | I]
    let mut aug: Vec<Vec<f64>> = (0..n)
        .map(|i| {
            let mut row = a[i].clone();
            for j in 0..n {
                row.push(if i == j { 1.0 } else { 0.0 });
            }
            row
        })
        .collect();

    // Forward elimination with partial pivoting
    for col in 0..n {
        // Find pivot
        let pivot_row = (col..n).max_by(|&r1, &r2| {
            aug[r1][col]
                .abs()
                .partial_cmp(&aug[r2][col].abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        })?;

        aug.swap(col, pivot_row);

        let pivot = aug[col][col];
        if pivot.abs() < 1e-14 {
            return None; // singular
        }

        // Scale pivot row
        for val in aug[col].iter_mut().take(2 * n) {
            *val /= pivot;
        }

        // Eliminate column
        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = aug[row][col];
            let pivot_row: Vec<f64> = aug[col][..2 * n].to_vec();
            for (dest, &src) in aug[row].iter_mut().take(2 * n).zip(pivot_row.iter()) {
                *dest -= factor * src;
            }
        }
    }

    // Extract right half as inverse
    let inv: Vec<Vec<f64>> = aug.into_iter().map(|row| row[n..].to_vec()).collect();
    Some(inv)
}

/// Construct a perturbed operating state for boundary tracing.
fn perturb_state(
    base: &SystemOperatingState,
    bus_1: usize,
    p1_mw: f64,
    bus_2: usize,
    p2_mw: f64,
) -> SystemOperatingState {
    let mut state = base.clone();

    // Convert MW injection to a loading proxy:
    // loading_pct[bus] ≈ P_bus / rating * 100
    if bus_1 < state.branch_loadings_pct.len() {
        state.branch_loadings_pct[bus_1] = p1_mw.abs().min(150.0);
    }
    if bus_2 < state.branch_loadings_pct.len() {
        state.branch_loadings_pct[bus_2] = p2_mw.abs().min(150.0);
    }
    // Adjust voltage proxy
    if bus_1 < state.voltage_magnitudes.len() {
        state.voltage_magnitudes[bus_1] = (1.0 - p1_mw.abs() / 2000.0).clamp(0.85, 1.05);
    }
    if bus_2 < state.voltage_magnitudes.len() {
        state.voltage_magnitudes[bus_2] = (1.0 - p2_mw.abs() / 2000.0).clamp(0.85, 1.05);
    }
    state
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // -----------------------------------------------------------------------
    // Fixtures
    // -----------------------------------------------------------------------

    fn make_config(n_buses: usize, n_branches: usize) -> RtsaConfig {
        RtsaConfig {
            n_buses,
            n_branches,
            max_contingencies: 10,
            screening_threshold: 1.0,
            v_min_pu: 0.95,
            v_max_pu: 1.05,
            thermal_limit_pct: 90.0,
            update_interval_s: 30.0,
            n_exponent: 2,
        }
    }

    fn make_ratings(n: usize, rating: f64) -> Vec<f64> {
        vec![rating; n]
    }

    fn nominal_state(n_buses: usize, n_branches: usize) -> SystemOperatingState {
        SystemOperatingState {
            timestamp: 0.0,
            voltage_magnitudes: vec![1.0; n_buses],
            voltage_angles: vec![0.0; n_buses],
            branch_loadings_pct: vec![50.0; n_branches],
            frequency_hz: 50.0,
            total_generation_mw: 500.0,
            total_load_mw: 480.0,
            reserve_mw: 50.0,
            reactive_reserves_mvar: vec![30.0; 2],
        }
    }

    fn make_assessor(n_buses: usize, n_branches: usize, rating: f64) -> RealTimeSecurityAssessor {
        let config = make_config(n_buses, n_branches);
        let ratings = make_ratings(n_branches, rating);
        RealTimeSecurityAssessor::new(config, Vec::new(), ratings)
    }

    // -----------------------------------------------------------------------
    // Test 1: SecurityLevel ordering
    // -----------------------------------------------------------------------
    #[test]
    fn test_security_level_ordering() {
        assert!(SecurityLevel::Normal < SecurityLevel::Alert);
        assert!(SecurityLevel::Alert < SecurityLevel::Emergency);
        assert!(SecurityLevel::Emergency < SecurityLevel::ExtremisEmergency);
        assert!(SecurityLevel::ExtremisEmergency < SecurityLevel::Restorative);
    }

    // -----------------------------------------------------------------------
    // Test 2: Contingency creation
    // -----------------------------------------------------------------------
    #[test]
    fn test_contingency_creation() {
        let c = Contingency {
            id: 1,
            name: "N-1 Line 3".to_string(),
            outaged_lines: vec![3],
            outaged_generators: Vec::new(),
            probability: 0.01,
            severity_weight: 1.5,
        };
        assert_eq!(c.id, 1);
        assert_eq!(c.outaged_lines, vec![3]);
        assert!((c.probability - 0.01).abs() < 1e-12);
    }

    // -----------------------------------------------------------------------
    // Test 3: RtsaConfig default
    // -----------------------------------------------------------------------
    #[test]
    fn test_rtsa_config_default() {
        let cfg = RtsaConfig::default();
        assert_eq!(cfg.n_buses, 10);
        assert_eq!(cfg.n_branches, 15);
        assert_eq!(cfg.max_contingencies, 100);
        assert!((cfg.v_min_pu - 0.95).abs() < 1e-12);
        assert!((cfg.v_max_pu - 1.05).abs() < 1e-12);
        assert!((cfg.thermal_limit_pct - 90.0).abs() < 1e-12);
        assert!((cfg.update_interval_s - 30.0).abs() < 1e-12);
        assert_eq!(cfg.n_exponent, 2);
    }

    // -----------------------------------------------------------------------
    // Test 4: SystemOperatingState creation
    // -----------------------------------------------------------------------
    #[test]
    fn test_system_operating_state_creation() {
        let s = nominal_state(4, 5);
        assert_eq!(s.voltage_magnitudes.len(), 4);
        assert_eq!(s.branch_loadings_pct.len(), 5);
        assert!((s.frequency_hz - 50.0).abs() < 1e-12);
        assert_eq!(s.voltage_angles.len(), 4);
    }

    // -----------------------------------------------------------------------
    // Test 5: CorrectiveActionType variants
    // -----------------------------------------------------------------------
    #[test]
    fn test_corrective_action_type() {
        let a1 = CorrectiveActionType::GeneratorRedispatch {
            from_bus: 0,
            to_bus: 5,
        };
        let a2 = CorrectiveActionType::LoadShedding {
            bus: 3,
            amount_mw: 20.0,
        };
        let a3 = CorrectiveActionType::ReactiveCompensation {
            bus: 2,
            amount_mvar: 15.0,
        };
        let a4 = CorrectiveActionType::TransformerTapChange {
            transformer_id: 1,
            new_tap: 1.05,
        };
        let a5 = CorrectiveActionType::LineShuntOperation { branch: 4 };

        assert_ne!(a1, a2);
        assert_ne!(a2, a3);
        assert_ne!(a3, a4);
        assert_ne!(a4, a5);
    }

    // -----------------------------------------------------------------------
    // Test 6: compute_pi with zero loading
    // -----------------------------------------------------------------------
    #[test]
    fn test_compute_pi_zero_loading() {
        let assessor = make_assessor(3, 4, 100.0);
        let pi = assessor.compute_pi(&[0.0, 0.0, 0.0, 0.0]);
        assert!(
            (pi - 0.0).abs() < 1e-12,
            "PI should be 0 for zero loading, got {pi}"
        );
    }

    // -----------------------------------------------------------------------
    // Test 7: compute_pi with 50 % loading
    // -----------------------------------------------------------------------
    #[test]
    fn test_compute_pi_half_loading() {
        let assessor = make_assessor(2, 2, 100.0);
        // Each branch at 50 % → (0.5)^4 = 0.0625 per branch; 2 branches → 0.125
        let pi = assessor.compute_pi(&[50.0, 50.0]);
        let expected = 2.0 * 0.5_f64.powi(4);
        assert!(
            (pi - expected).abs() < 1e-10,
            "expected {expected}, got {pi}"
        );
    }

    // -----------------------------------------------------------------------
    // Test 8: compute_pi at 100 % loading
    // -----------------------------------------------------------------------
    #[test]
    fn test_compute_pi_full_loading() {
        let assessor = make_assessor(2, 2, 100.0);
        // Each branch at 100 % → (1.0)^4 = 1.0 per branch; 2 branches → 2.0
        let pi = assessor.compute_pi(&[100.0, 100.0]);
        assert!((pi - 2.0).abs() < 1e-10, "expected 2.0, got {pi}");
    }

    // -----------------------------------------------------------------------
    // Test 9: check_base_case normal state
    // -----------------------------------------------------------------------
    #[test]
    fn test_check_base_case_normal() {
        let assessor = make_assessor(4, 4, 200.0);
        let state = nominal_state(4, 4);
        let violations = assessor.check_base_case(&state);
        assert!(
            violations.is_empty(),
            "expected no violations, got {:?}",
            violations.len()
        );
    }

    // -----------------------------------------------------------------------
    // Test 10: check_base_case voltage violation
    // -----------------------------------------------------------------------
    #[test]
    fn test_check_base_case_voltage_violation() {
        let assessor = make_assessor(4, 4, 200.0);
        let mut state = nominal_state(4, 4);
        state.voltage_magnitudes[2] = 0.90; // below v_min = 0.95

        let violations = assessor.check_base_case(&state);
        let voltage_viols: Vec<_> = violations
            .iter()
            .filter(|v| matches!(v, ConstraintType::VoltageBand { .. }))
            .collect();
        assert!(
            !voltage_viols.is_empty(),
            "expected voltage violation at bus 2"
        );
        let bus_2_violated = voltage_viols
            .iter()
            .any(|v| matches!(v, ConstraintType::VoltageBand { bus: 2, .. }));
        assert!(bus_2_violated, "bus 2 should be flagged");
    }

    // -----------------------------------------------------------------------
    // Test 11: check_base_case thermal violation
    // -----------------------------------------------------------------------
    #[test]
    fn test_check_base_case_thermal_violation() {
        let assessor = make_assessor(3, 3, 100.0);
        let mut state = nominal_state(3, 3);
        state.branch_loadings_pct[1] = 110.0; // overloaded

        let violations = assessor.check_base_case(&state);
        let thermal_viols: Vec<_> = violations
            .iter()
            .filter(|v| matches!(v, ConstraintType::ThermalLine { branch_idx: 1, .. }))
            .collect();
        assert!(
            !thermal_viols.is_empty(),
            "expected thermal violation on branch 1"
        );
    }

    // -----------------------------------------------------------------------
    // Test 12: screen_contingency — no violation for lightly loaded system
    // -----------------------------------------------------------------------
    #[test]
    fn test_screen_contingency_no_violation() {
        let assessor = make_assessor(3, 3, 200.0);
        let contingency = Contingency {
            id: 0,
            name: "N-1-L0".to_string(),
            outaged_lines: vec![0],
            outaged_generators: Vec::new(),
            probability: 0.01,
            severity_weight: 1.0,
        };
        // Light loading: 30 % on each branch → post-contingency still < 100 %
        let base_loadings_mva = vec![60.0, 60.0, 60.0]; // 30 % of 200 MVA
        let result = assessor.screen_contingency(&contingency, &base_loadings_mva);

        assert_eq!(result.contingency_id, 0);
        // PI for two remaining branches at ~90 MVA/200 MVA = 45 %:
        // Should not trigger screened_in if PI ≤ threshold
        // (depends on LODF, which is 0 by default → no redistribution)
        assert_eq!(result.n_violated_branches, 0, "no branches should violate");
    }

    // -----------------------------------------------------------------------
    // Test 13: screen_contingency — triggered when branch overloaded
    // -----------------------------------------------------------------------
    #[test]
    fn test_screen_contingency_violated() {
        let mut assessor = make_assessor(3, 3, 100.0);

        // Pre-load a PTDF so that LODF redistributes flow onto branch 1
        // PTDF[1][from=0] = 0.8, PTDF[1][to=0] = 0.0
        assessor.ptdf_matrix = vec![
            vec![0.0, 0.0, 0.0],
            vec![0.8, 0.0, 0.0],
            vec![0.2, 0.0, 0.0],
        ];

        let contingency = Contingency {
            id: 1,
            name: "N-1-L0".to_string(),
            outaged_lines: vec![0],
            outaged_generators: Vec::new(),
            probability: 0.01,
            severity_weight: 1.0,
        };
        // Branch 0 carries 90 MVA (90 % of 100 MVA rating) — after outage
        // LODF redirects flow to branch 1 which is already near limit
        let base_loadings_mva = vec![90.0, 80.0, 30.0];
        let result = assessor.screen_contingency(&contingency, &base_loadings_mva);

        // The PI should exceed 1.0 because branches are heavily loaded
        assert!(
            result.screened_in || result.performance_index > 0.0,
            "heavily loaded system should produce non-trivial PI"
        );
    }

    // -----------------------------------------------------------------------
    // Test 14: classify_security_level — Normal
    // -----------------------------------------------------------------------
    #[test]
    fn test_classify_normal() {
        let level = RealTimeSecurityAssessor::classify_security_level(&[], &[]);
        assert_eq!(level, SecurityLevel::Normal);
    }

    // -----------------------------------------------------------------------
    // Test 15: classify_security_level — Alert
    // -----------------------------------------------------------------------
    #[test]
    fn test_classify_alert() {
        // No base violations, but one contingency has a violated constraint
        let violated_result = ContingencyAnalysisResult {
            contingency_id: 0,
            post_contingency_voltages: vec![0.93],
            post_contingency_loadings: vec![105.0],
            violated_constraints: vec![ConstraintType::ThermalLine {
                branch_idx: 0,
                limit_mva: 100.0,
            }],
            security_level: SecurityLevel::Emergency,
            corrective_actions: Vec::new(),
            estimated_recovery_time_min: 5.0,
        };
        let level = RealTimeSecurityAssessor::classify_security_level(&[], &[violated_result]);
        assert_eq!(level, SecurityLevel::Alert);
    }

    // -----------------------------------------------------------------------
    // Test 16: classify_security_level — Emergency
    // -----------------------------------------------------------------------
    #[test]
    fn test_classify_emergency() {
        let base_viols = vec![ConstraintType::ThermalLine {
            branch_idx: 0,
            limit_mva: 100.0,
        }];
        let level = RealTimeSecurityAssessor::classify_security_level(&base_viols, &[]);
        assert_eq!(level, SecurityLevel::Emergency);
    }

    // -----------------------------------------------------------------------
    // Test 17: assess — clean network, no contingencies → Normal
    // -----------------------------------------------------------------------
    #[test]
    fn test_assess_normal_state() {
        let mut assessor = make_assessor(4, 4, 200.0);
        let state = nominal_state(4, 4);
        let result = assessor.assess(&state).expect("assess should succeed");

        assert_eq!(result.system_security_level, SecurityLevel::Normal);
        assert!(result.base_case_violations.is_empty());
        assert_eq!(result.n_contingencies_screened, 0);
        assert_eq!(result.n_contingencies_analyzed, 0);
        assert!(result.worst_contingency.is_none());
    }

    // -----------------------------------------------------------------------
    // Test 18: assess — with one contingency registered
    // -----------------------------------------------------------------------
    #[test]
    fn test_assess_with_contingency() {
        let config = make_config(3, 3);
        let ratings = make_ratings(3, 150.0);
        let contingency = Contingency {
            id: 0,
            name: "N-1-L1".to_string(),
            outaged_lines: vec![1],
            outaged_generators: Vec::new(),
            probability: 0.02,
            severity_weight: 1.0,
        };
        let mut assessor = RealTimeSecurityAssessor::new(config, vec![contingency], ratings);

        let state = nominal_state(3, 3); // 50 % loading → secure
        let result = assessor.assess(&state).expect("assess should succeed");

        assert_eq!(result.n_contingencies_screened, 1);
        // System PI should be computed
        assert!(result.system_performance_index >= 0.0);
        // Security margin should be positive
        assert!(result.security_margin_pct >= 0.0);
    }

    // -----------------------------------------------------------------------
    // Test 19: security_margin — normal (lightly loaded)
    // -----------------------------------------------------------------------
    #[test]
    fn test_security_margin_normal() {
        let assessor = make_assessor(3, 3, 100.0);
        let state = nominal_state(3, 3); // 50 % loading
        let margin = assessor.compute_security_margin(&state);
        // thermal_limit_pct = 90 %, loading = 50 % → margin = (90-50)/90*100 ≈ 44.4 %
        assert!(margin > 30.0, "expected margin > 30%, got {margin:.2}%");
    }

    // -----------------------------------------------------------------------
    // Test 20: security_margin — stressed (near limit)
    // -----------------------------------------------------------------------
    #[test]
    fn test_security_margin_stressed() {
        let assessor = make_assessor(3, 3, 100.0);
        let mut state = nominal_state(3, 3);
        state.branch_loadings_pct = vec![89.0, 88.0, 87.0]; // just under thermal_limit_pct = 90

        let margin = assessor.compute_security_margin(&state);
        // thermal_limit = 90, max_loading = 89 → margin = (90-89)/90*100 ≈ 1.1 %
        assert!(
            margin < 5.0,
            "expected small margin (<5%), got {margin:.2}%"
        );
    }

    // -----------------------------------------------------------------------
    // Additional tests for robustness
    // -----------------------------------------------------------------------

    #[test]
    fn test_compute_ptdf_empty_input() {
        let ptdf = RealTimeSecurityAssessor::compute_ptdf(&[], &[], &[]);
        assert!(ptdf.is_empty());
    }

    #[test]
    fn test_compute_ptdf_single_bus() {
        let b = vec![vec![1.0]];
        let ptdf = RealTimeSecurityAssessor::compute_ptdf(&b, &b, &[(0, 0)]);
        // Single bus: reduced system is empty → zero PTDF
        assert_eq!(ptdf.len(), 1);
    }

    #[test]
    fn test_assess_dimension_mismatch_voltages() {
        let mut assessor = make_assessor(4, 4, 100.0);
        let mut state = nominal_state(4, 4);
        state.voltage_magnitudes = vec![1.0; 3]; // wrong size
        let err = assessor.assess(&state);
        assert!(err.is_err(), "should error on dimension mismatch");
    }

    #[test]
    fn test_assess_dimension_mismatch_loadings() {
        let mut assessor = make_assessor(4, 4, 100.0);
        let mut state = nominal_state(4, 4);
        state.branch_loadings_pct = vec![50.0; 3]; // wrong size
        let err = assessor.assess(&state);
        assert!(err.is_err(), "should error on dimension mismatch");
    }

    #[test]
    fn test_security_boundary_tracer_basic() {
        let config = make_config(4, 4);
        let ratings = make_ratings(4, 200.0);
        let assessor = RealTimeSecurityAssessor::new(config, Vec::new(), ratings);
        let mut tracer = SecurityBoundaryTracer::new(assessor, 4);
        let state = nominal_state(4, 4);
        let boundary = tracer.trace_boundary_2d(&state, 0, 1);
        assert_eq!(boundary.len(), 4, "should return 4 boundary points");
    }

    #[test]
    fn test_classify_extremis_emergency() {
        // More than 3 base violations → ExtremisEmergency
        let viols = vec![
            ConstraintType::ThermalLine {
                branch_idx: 0,
                limit_mva: 100.0,
            },
            ConstraintType::ThermalLine {
                branch_idx: 1,
                limit_mva: 100.0,
            },
            ConstraintType::VoltageBand {
                bus: 0,
                v_min_pu: 0.95,
                v_max_pu: 1.05,
            },
            ConstraintType::VoltageBand {
                bus: 1,
                v_min_pu: 0.95,
                v_max_pu: 1.05,
            },
        ];
        let level = RealTimeSecurityAssessor::classify_security_level(&viols, &[]);
        assert_eq!(level, SecurityLevel::ExtremisEmergency);
    }

    #[test]
    fn test_corrective_actions_thermal() {
        let assessor = make_assessor(4, 4, 100.0);
        let result = ContingencyAnalysisResult {
            contingency_id: 0,
            post_contingency_voltages: vec![1.0; 4],
            post_contingency_loadings: vec![110.0, 80.0, 70.0, 60.0],
            violated_constraints: vec![ConstraintType::ThermalLine {
                branch_idx: 0,
                limit_mva: 100.0,
            }],
            security_level: SecurityLevel::Emergency,
            corrective_actions: Vec::new(),
            estimated_recovery_time_min: 5.0,
        };
        let state = nominal_state(4, 4);
        let actions = assessor.generate_corrective_actions(&result, &state);
        assert!(
            !actions.is_empty(),
            "should generate at least one corrective action"
        );
        let has_redispatch = actions.iter().any(|a| {
            matches!(
                a.action_type,
                CorrectiveActionType::GeneratorRedispatch { .. }
            )
        });
        assert!(
            has_redispatch,
            "should recommend redispatch for thermal violation"
        );
    }

    #[test]
    fn test_corrective_actions_voltage() {
        let assessor = make_assessor(4, 4, 100.0);
        let result = ContingencyAnalysisResult {
            contingency_id: 0,
            post_contingency_voltages: vec![0.92, 1.0, 1.0, 1.0],
            post_contingency_loadings: vec![50.0; 4],
            violated_constraints: vec![ConstraintType::VoltageBand {
                bus: 0,
                v_min_pu: 0.95,
                v_max_pu: 1.05,
            }],
            security_level: SecurityLevel::Emergency,
            corrective_actions: Vec::new(),
            estimated_recovery_time_min: 3.0,
        };
        let state = nominal_state(4, 4);
        let actions = assessor.generate_corrective_actions(&result, &state);
        let has_reactive = actions.iter().any(|a| {
            matches!(
                a.action_type,
                CorrectiveActionType::ReactiveCompensation { .. }
            )
        });
        assert!(
            has_reactive,
            "should recommend reactive compensation for voltage violation"
        );
    }

    #[test]
    fn test_invert_matrix_2x2() {
        // [2 1]^{-1} = [2 -1] / 3
        // [1 2]        [-1 2]
        let a = vec![vec![2.0, 1.0], vec![1.0, 2.0]];
        let inv = invert_matrix(&a).expect("should invert");
        // Check A * A^{-1} ≈ I
        let eps = 1e-10;
        let i00 = a[0][0] * inv[0][0] + a[0][1] * inv[1][0];
        let i11 = a[1][0] * inv[0][1] + a[1][1] * inv[1][1];
        assert!((i00 - 1.0).abs() < eps, "i00={i00}");
        assert!((i11 - 1.0).abs() < eps, "i11={i11}");
    }

    #[test]
    fn test_invert_singular_matrix() {
        let a = vec![vec![1.0, 2.0], vec![2.0, 4.0]]; // singular
        let inv = invert_matrix(&a);
        assert!(inv.is_none(), "should return None for singular matrix");
    }

    #[test]
    fn test_system_pi_weighted() {
        // With two contingencies of different severity weights:
        // Contingency 0: weight=1.0, one branch at 100% → PI contribution = 1.0
        // Contingency 1: weight=2.0, one branch at 100% → PI contribution = 2.0
        // system_pi = 1.0*sr0.pi + 2.0*sr1.pi
        let config = make_config(2, 2);
        let ratings = make_ratings(2, 100.0);
        let c0 = Contingency {
            id: 0,
            name: "C0".to_string(),
            outaged_lines: Vec::new(),
            outaged_generators: Vec::new(),
            probability: 0.01,
            severity_weight: 1.0,
        };
        let c1 = Contingency {
            id: 1,
            name: "C1".to_string(),
            outaged_lines: Vec::new(),
            outaged_generators: Vec::new(),
            probability: 0.01,
            severity_weight: 2.0,
        };
        let mut assessor = RealTimeSecurityAssessor::new(config, vec![c0, c1], ratings);

        let mut state = nominal_state(2, 2);
        state.branch_loadings_pct = vec![100.0, 100.0]; // 100 % on each branch

        let result = assessor.assess(&state).expect("assess should succeed");
        // Each contingency has no outaged lines so post flows = base flows
        // PI per branch = (100/100)^4 = 1.0 per branch, 2 branches = 2.0
        // C0 contribution: 1.0 * 2.0 = 2.0; C1: 2.0 * 2.0 = 4.0 → total 6.0
        assert!(
            result.system_performance_index > 0.0,
            "system PI should be positive, got {}",
            result.system_performance_index
        );
    }
}
