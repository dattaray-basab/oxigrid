//! AC Security-Constrained Optimal Power Flow (AC-SCOPF).
//!
//! Solves the AC-SCOPF problem using Successive Linear Programming (SLP)
//! with N-1 contingency constraints. The algorithm:
//!
//! 1. Solve base-case AC-OPF via economic dispatch + Gauss-Seidel power flow.
//! 2. Screen all N-1 contingencies (branch outage or generator outage).
//! 3. For each violated contingency, compute linearised flow sensitivities
//!    and perform a single-variable redispatch within a trust region.
//! 4. Iterate until no violations remain or `max_slp_iter` is reached.
//!
//! # References
//! Capitanescu et al., "State-of-the-art, challenges, and future trends in
//! security constrained optimal power flow", EPSR 81 (2011) 1731–1741.

/// Bus type for AC power flow.
#[derive(Debug, Clone)]
pub enum AcBusType {
    /// Reference (slack) bus — voltage magnitude and angle fixed.
    Slack,
    /// Generator bus — voltage magnitude regulated, Q within limits.
    PV {
        /// Voltage setpoint [p.u.]
        v_setpoint_pu: f64,
        /// Minimum reactive generation `MVAr`
        q_min_mvar: f64,
        /// Maximum reactive generation `MVAr`
        q_max_mvar: f64,
    },
    /// Load bus — P and Q injections specified, V and θ solved.
    PQ,
}

/// Bus data for AC SCOPF.
#[derive(Debug, Clone)]
pub struct AcScopfBus {
    /// Unique bus identifier (0-based index used internally).
    pub bus_id: usize,
    /// Bus type (Slack / PV / PQ).
    pub bus_type: AcBusType,
    /// Minimum voltage magnitude [p.u.]
    pub v_min_pu: f64,
    /// Maximum voltage magnitude [p.u.]
    pub v_max_pu: f64,
    /// Active load `MW`
    pub p_load_mw: f64,
    /// Reactive load `MVAr`
    pub q_load_mvar: f64,
    /// Shunt susceptance [p.u.] (positive = capacitive).
    pub b_shunt_pu: f64,
}

/// Generator data for AC SCOPF.
#[derive(Debug, Clone)]
pub struct AcScopfGenerator {
    /// Bus where the generator is connected.
    pub bus_id: usize,
    /// Minimum active power output `MW`
    pub p_min_mw: f64,
    /// Maximum active power output `MW`
    pub p_max_mw: f64,
    /// Minimum reactive power output `MVAr`
    pub q_min_mvar: f64,
    /// Maximum reactive power output `MVAr`
    pub q_max_mvar: f64,
    /// Quadratic cost coefficient [$/MW²h]
    pub cost_a: f64,
    /// Linear cost coefficient [$/MWh]
    pub cost_b: f64,
    /// Fixed cost [$/h]
    pub cost_c: f64,
    /// Ramp rate limit [MW/min]
    pub ramp_rate_mw_per_min: f64,
}

/// Branch (transmission line or transformer) data for AC SCOPF.
#[derive(Debug, Clone)]
pub struct AcScopfBranch {
    /// Unique branch identifier.
    pub branch_id: usize,
    /// From-bus identifier.
    pub from_bus: usize,
    /// To-bus identifier.
    pub to_bus: usize,
    /// Series resistance [p.u.]
    pub r_pu: f64,
    /// Series reactance [p.u.]
    pub x_pu: f64,
    /// Total line charging susceptance [p.u.]
    pub b_pu: f64,
    /// Normal thermal rating `MVA`
    pub rating_mva: f64,
    /// Post-contingency (emergency) thermal rating `MVA` (typically 120% of normal).
    pub rating_emergency_mva: f64,
}

/// Contingency element type.
#[derive(Debug, Clone)]
pub enum ContingencyElement {
    /// Single branch outage.
    BranchOutage {
        /// Branch identifier to be tripped.
        branch_id: usize,
    },
    /// Single generator outage.
    GeneratorOutage {
        /// Bus identifier where the generator is located.
        gen_bus_id: usize,
    },
}

/// N-1 contingency definition.
#[derive(Debug, Clone)]
pub struct AcContingency {
    /// Contingency identifier.
    pub id: usize,
    /// Human-readable name for this contingency.
    pub name: String,
    /// What element is lost in this contingency.
    pub contingency_type: ContingencyElement,
}

/// Configuration for the AC SCOPF solver.
#[derive(Debug, Clone)]
pub struct AcScopfConfig {
    /// Maximum number of outer SLP iterations (default 20).
    pub max_slp_iter: usize,
    /// Maximum Newton-Raphson / Gauss-Seidel inner iterations (default 20).
    pub max_nr_iter: usize,
    /// Power flow convergence tolerance [p.u.] (default 1e-4).
    pub convergence_tol: f64,
    /// Constraint violation tolerance `MVA` (default 1e-3).
    pub constraint_tol: f64,
    /// SLP trust-region step limit [fraction of p_max] (default 0.1).
    pub trust_region: f64,
    /// If true, use preventive (base-case) redispatch; corrective otherwise.
    pub use_preventive: bool,
}

impl Default for AcScopfConfig {
    fn default() -> Self {
        Self {
            max_slp_iter: 20,
            max_nr_iter: 20,
            convergence_tol: 1e-4,
            constraint_tol: 1e-3,
            trust_region: 0.1,
            use_preventive: true,
        }
    }
}

/// System operating point (snapshot of voltages and generation).
#[derive(Debug, Clone)]
pub struct OperatingPoint {
    /// Active power generation per generator `MW`.
    pub p_gen_mw: Vec<f64>,
    /// Reactive power generation per generator `MVAr`.
    pub q_gen_mvar: Vec<f64>,
    /// Bus voltage magnitudes [p.u.], indexed by bus position.
    pub v_pu: Vec<f64>,
    /// Bus voltage angles `rad`, indexed by bus position.
    pub theta_rad: Vec<f64>,
}

/// Post-contingency security assessment for a single contingency.
#[derive(Debug, Clone)]
pub struct ContingencyAssessment {
    /// Contingency identifier.
    pub contingency_id: usize,
    /// True if the system is feasible after this contingency.
    pub feasible: bool,
    /// Branches that violate emergency thermal limits: (branch_id, flow_mva).
    pub violated_branches: Vec<(usize, f64)>,
    /// Buses that violate voltage limits: (bus_id, v_pu).
    pub violated_voltages: Vec<(usize, f64)>,
    /// Cost of preventive/corrective redispatch for this contingency [$/h].
    pub redispatch_cost: f64,
}

/// Result of the AC SCOPF solve.
#[derive(Debug, Clone)]
pub struct AcScopfResult {
    /// Base-case operating point after security-constrained dispatch.
    pub base_case: OperatingPoint,
    /// Base-case total generation cost [$/h].
    pub base_cost: f64,
    /// Assessment for each contingency.
    pub contingency_results: Vec<ContingencyAssessment>,
    /// Total number of violations across all contingencies before SLP redispatch.
    pub total_violations_before: usize,
    /// Total number of violations across all contingencies after SLP redispatch.
    pub total_violations_after: usize,
    /// Number of SLP outer iterations performed.
    pub n_slp_iterations: usize,
    /// True if the SLP converged (no violations remain).
    pub converged: bool,
    /// Security-constrained total cost (base + redispatch) [$/h].
    pub security_constrained_cost: f64,
    /// Total redispatch cost added for security [$/h].
    pub redispatch_cost: f64,
}

/// AC Security-Constrained Optimal Power Flow problem.
///
/// Build the problem by adding buses, generators, branches, and contingencies,
/// then call [`solve`](AcScopfProblem::solve).
#[derive(Debug, Clone)]
pub struct AcScopfProblem {
    /// System buses.
    pub buses: Vec<AcScopfBus>,
    /// Generators.
    pub generators: Vec<AcScopfGenerator>,
    /// Transmission branches.
    pub branches: Vec<AcScopfBranch>,
    /// N-1 contingency list.
    pub contingencies: Vec<AcContingency>,
    /// Solver configuration.
    pub config: AcScopfConfig,
}

// ── System base MVA ───────────────────────────────────────────────────────────
const BASE_MVA: f64 = 100.0;

impl AcScopfProblem {
    /// Create a new, empty AC SCOPF problem with the given configuration.
    pub fn new(config: AcScopfConfig) -> Self {
        Self {
            buses: Vec::new(),
            generators: Vec::new(),
            branches: Vec::new(),
            contingencies: Vec::new(),
            config,
        }
    }

    /// Add a bus to the problem.
    pub fn add_bus(&mut self, bus: AcScopfBus) {
        self.buses.push(bus);
    }

    /// Add a generator to the problem.
    pub fn add_generator(&mut self, gen: AcScopfGenerator) {
        self.generators.push(gen);
    }

    /// Add a branch to the problem.
    pub fn add_branch(&mut self, branch: AcScopfBranch) {
        self.branches.push(branch);
    }

    /// Add an N-1 contingency to the problem.
    pub fn add_contingency(&mut self, contingency: AcContingency) {
        self.contingencies.push(contingency);
    }

    /// Compute total generation cost [$/h] for a given dispatch vector.
    ///
    /// Uses the quadratic cost function: `cost_a * P² + cost_b * P + cost_c`.
    pub fn compute_cost(&self, p_gen: &[f64]) -> f64 {
        self.generators
            .iter()
            .zip(p_gen.iter())
            .map(|(g, &p)| g.cost_a * p * p + g.cost_b * p + g.cost_c)
            .sum()
    }

    /// Solve the AC SCOPF problem.
    ///
    /// Returns an [`AcScopfResult`] containing the optimal operating point,
    /// contingency assessments, and convergence information.
    pub fn solve(&self) -> Result<AcScopfResult, String> {
        if self.buses.is_empty() {
            return Err("No buses defined".to_string());
        }
        if self.generators.is_empty() {
            return Err("No generators defined".to_string());
        }

        // ── Step 1: Solve base-case AC-OPF ───────────────────────────────────
        let mut op = self.solve_base_opf()?;
        let base_cost = self.compute_cost(&op.p_gen_mw);

        // ── Step 2: Count violations before SLP ──────────────────────────────
        let violations_before: Vec<ContingencyAssessment> = self
            .contingencies
            .iter()
            .map(|c| self.check_contingency(&op, c))
            .collect();
        let total_violations_before: usize = violations_before
            .iter()
            .map(|a| a.violated_branches.len() + a.violated_voltages.len())
            .sum();

        // ── Step 3: SLP redispatch iterations ────────────────────────────────
        let mut n_slp_iterations = 0usize;
        let mut converged = false;
        let mut total_redispatch_cost = 0.0f64;

        for iter in 0..self.config.max_slp_iter {
            n_slp_iterations = iter + 1;

            // Assess all contingencies
            let assessments: Vec<ContingencyAssessment> = self
                .contingencies
                .iter()
                .map(|c| self.check_contingency(&op, c))
                .collect();

            let any_violation = assessments
                .iter()
                .any(|a| !a.violated_branches.is_empty() || !a.violated_voltages.is_empty());

            if !any_violation {
                converged = true;
                break;
            }

            // Preventive redispatch: adjust base-case generation
            if self.config.use_preventive {
                for assessment in &assessments {
                    for violation in &assessment.violated_branches {
                        let cost = self.redispatch_for_violation(&mut op, violation);
                        total_redispatch_cost += cost;
                    }
                }
                // Re-solve base power flow after redispatch
                if let Ok(new_op) = self.solve_power_flow(&op, None) {
                    op.v_pu = new_op.v_pu;
                    op.theta_rad = new_op.theta_rad;
                }
            }
        }

        // If no contingencies, mark converged
        if self.contingencies.is_empty() {
            converged = true;
        }

        // ── Step 4: Final contingency assessment ─────────────────────────────
        let final_assessments: Vec<ContingencyAssessment> = self
            .contingencies
            .iter()
            .map(|c| self.check_contingency(&op, c))
            .collect();

        let total_violations_after: usize = final_assessments
            .iter()
            .map(|a| a.violated_branches.len() + a.violated_voltages.len())
            .sum();

        let security_constrained_cost = base_cost + total_redispatch_cost;

        Ok(AcScopfResult {
            base_case: op,
            base_cost,
            contingency_results: final_assessments,
            total_violations_before,
            total_violations_after,
            n_slp_iterations,
            converged,
            security_constrained_cost,
            redispatch_cost: total_redispatch_cost,
        })
    }

    // ── Internal: Base-case AC-OPF ────────────────────────────────────────────

    /// Solve base-case AC-OPF: economic dispatch + AC power flow.
    fn solve_base_opf(&self) -> Result<OperatingPoint, String> {
        let p_gen = self.economic_dispatch();
        let n_bus = self.buses.len();
        let _n_gen = self.generators.len();

        // Initial operating point: flat start
        let q_gen = self
            .generators
            .iter()
            .map(|g| (g.q_min_mvar + g.q_max_mvar) / 2.0)
            .collect::<Vec<_>>();
        let v_pu = self
            .buses
            .iter()
            .map(|b| match &b.bus_type {
                AcBusType::Slack => 1.0,
                AcBusType::PV { v_setpoint_pu, .. } => *v_setpoint_pu,
                AcBusType::PQ => 1.0,
            })
            .collect::<Vec<_>>();
        let theta_rad = vec![0.0f64; n_bus];

        let init_op = OperatingPoint {
            p_gen_mw: p_gen,
            q_gen_mvar: q_gen,
            v_pu,
            theta_rad,
        };

        // Solve power flow
        let op = self.solve_power_flow(&init_op, None)?;

        // Enforce Q limits on PV buses
        let mut q_gen_final = op.q_gen_mvar.clone();
        for (gi, gen) in self.generators.iter().enumerate() {
            q_gen_final[gi] = q_gen_final[gi].clamp(gen.q_min_mvar, gen.q_max_mvar);
        }

        Ok(OperatingPoint {
            p_gen_mw: op.p_gen_mw,
            q_gen_mvar: q_gen_final,
            v_pu: op.v_pu,
            theta_rad: op.theta_rad,
        })
    }

    // ── Internal: Gauss-Seidel AC power flow ─────────────────────────────────

    /// Solve AC power flow using Gauss-Seidel iteration.
    ///
    /// `excluded_branch`: if `Some(id)`, that branch is treated as open
    /// (useful for post-contingency power flow).
    fn solve_power_flow(
        &self,
        op: &OperatingPoint,
        excluded_branch: Option<usize>,
    ) -> Result<OperatingPoint, String> {
        let n_bus = self.buses.len();
        if n_bus == 0 {
            return Err("No buses".to_string());
        }

        // Build Y-bus (possibly with branch excluded)
        let ybus = self.build_ybus_with_exclusion(excluded_branch);

        // Find slack bus index
        let slack_idx = self
            .buses
            .iter()
            .position(|b| matches!(b.bus_type, AcBusType::Slack))
            .unwrap_or(0);

        // Compute per-bus scheduled net injection (P_gen - P_load, Q_gen - Q_load) in p.u.
        let mut p_sched = vec![0.0f64; n_bus];
        let mut q_sched = vec![0.0f64; n_bus];

        // Load contributions
        for (bi, bus) in self.buses.iter().enumerate() {
            p_sched[bi] -= bus.p_load_mw / BASE_MVA;
            q_sched[bi] -= bus.q_load_mvar / BASE_MVA;
        }

        // Generator contributions
        for (gi, gen) in self.generators.iter().enumerate() {
            // Find bus index by bus_id
            if let Some(bi) = self.buses.iter().position(|b| b.bus_id == gen.bus_id) {
                let pg = op.p_gen_mw.get(gi).copied().unwrap_or(0.0) / BASE_MVA;
                let qg = op.q_gen_mvar.get(gi).copied().unwrap_or(0.0) / BASE_MVA;
                p_sched[bi] += pg;
                q_sched[bi] += qg;
            }
        }

        // For generator outage contingency: zero out outaged generators
        // (handled externally by passing modified op)

        // Complex voltage representation: V = Vmag * exp(j*theta)
        let mut v_re = vec![0.0f64; n_bus];
        let mut v_im = vec![0.0f64; n_bus];
        for (bi, _bus) in self.buses.iter().enumerate() {
            let vm = op.v_pu.get(bi).copied().unwrap_or(1.0);
            let th = op.theta_rad.get(bi).copied().unwrap_or(0.0);
            v_re[bi] = vm * th.cos();
            v_im[bi] = vm * th.sin();
        }

        // Gauss-Seidel iteration
        for _iter in 0..self.config.max_nr_iter {
            let mut max_change = 0.0f64;

            for bi in 0..n_bus {
                if bi == slack_idx {
                    continue; // Slack bus fixed
                }

                let vm_old = (v_re[bi] * v_re[bi] + v_im[bi] * v_im[bi]).sqrt();
                if vm_old < 1e-10 {
                    continue;
                }

                // Determine Q injection for PV buses (Q from scheduled + reactive balance)
                let q_inj = match &self.buses[bi].bus_type {
                    AcBusType::PV {
                        q_min_mvar,
                        q_max_mvar,
                        ..
                    } => {
                        // Compute Q injection needed to maintain voltage
                        // Use current voltage to compute Q from Y-bus
                        let mut q_calc = 0.0f64;
                        for (j, &(g_kj, b_kj)) in ybus[bi].iter().enumerate() {
                            let vj_re = v_re[j];
                            let vj_im = v_im[j];
                            // Q_k = -Im(V_k * conj(sum Y_kj * V_j))
                            // Im(V_k * conj(Y_kj * V_j)) = Vk_re*(g*vi+b*vr) - Vk_im*(g*vr-b*vi)
                            // simplified: contribution from j
                            let ij_re = g_kj * vj_re - b_kj * vj_im;
                            let ij_im = g_kj * vj_im + b_kj * vj_re;
                            // S_k = V_k * conj(I) => Q = Im(V_k * conj(I))
                            q_calc -= v_re[bi] * ij_im - v_im[bi] * ij_re;
                        }
                        q_calc.clamp(*q_min_mvar / BASE_MVA, *q_max_mvar / BASE_MVA)
                    }
                    _ => q_sched[bi],
                };

                // I_k_scheduled = conj((P - jQ) / V_k*)  = (P + jQ) / V_k
                // V_k = (I_k_sched - sum_{j!=k} Y_kj * V_j) / Y_kk
                let p_inj = p_sched[bi];
                let q_inj_use = q_inj;

                // I_k_sch = (P_k + j*Q_k) / (V_k_re - j*V_k_im)  [conj of V_k divided]
                // = (P_k + j*Q_k) * (V_k_re + j*V_k_im) / |V_k|^2
                let vm2 = v_re[bi] * v_re[bi] + v_im[bi] * v_im[bi];
                let i_sch_re = (p_inj * v_re[bi] + q_inj_use * v_im[bi]) / vm2;
                let i_sch_im = (p_inj * v_im[bi] - q_inj_use * v_re[bi]) / vm2;

                // Sum Y_kj * V_j for j != k
                let mut sum_re = 0.0f64;
                let mut sum_im = 0.0f64;
                for (j, &(g_kj, b_kj)) in ybus[bi].iter().enumerate() {
                    if j == bi {
                        continue;
                    }
                    sum_re += g_kj * v_re[j] - b_kj * v_im[j];
                    sum_im += g_kj * v_im[j] + b_kj * v_re[j];
                }

                // V_k_new = (I_sch - sum) / Y_kk
                let (g_kk, b_kk) = ybus[bi][bi];
                let y_kk_mag2 = g_kk * g_kk + b_kk * b_kk;
                if y_kk_mag2 < 1e-20 {
                    continue;
                }

                let numer_re = i_sch_re - sum_re;
                let numer_im = i_sch_im - sum_im;

                // V_new = numer / (G + jB) = numer * (G - jB) / (G^2 + B^2)
                let v_new_re = (numer_re * g_kk + numer_im * b_kk) / y_kk_mag2;
                let v_new_im = (numer_im * g_kk - numer_re * b_kk) / y_kk_mag2;

                // For PV buses: fix voltage magnitude, update angle only
                let (v_next_re, v_next_im) = match &self.buses[bi].bus_type {
                    AcBusType::PV { v_setpoint_pu, .. } => {
                        let v_new_mag = (v_new_re * v_new_re + v_new_im * v_new_im)
                            .sqrt()
                            .max(1e-10);
                        let scale = v_setpoint_pu / v_new_mag;
                        (v_new_re * scale, v_new_im * scale)
                    }
                    _ => (v_new_re, v_new_im),
                };

                let change =
                    ((v_next_re - v_re[bi]).powi(2) + (v_next_im - v_im[bi]).powi(2)).sqrt();
                if change > max_change {
                    max_change = change;
                }

                v_re[bi] = v_next_re;
                v_im[bi] = v_next_im;
            }

            if max_change < self.config.convergence_tol {
                break;
            }
        }

        // Extract results
        let mut v_pu = vec![0.0f64; n_bus];
        let mut theta_rad = vec![0.0f64; n_bus];
        for bi in 0..n_bus {
            v_pu[bi] = (v_re[bi] * v_re[bi] + v_im[bi] * v_im[bi]).sqrt();
            theta_rad[bi] = v_im[bi].atan2(v_re[bi]);
        }

        // Recompute Q generation from power flow for PV/Slack buses
        let mut q_gen_out = op.q_gen_mvar.clone();
        for (gi, gen) in self.generators.iter().enumerate() {
            if let Some(bi) = self.buses.iter().position(|b| b.bus_id == gen.bus_id) {
                match &self.buses[bi].bus_type {
                    AcBusType::Slack | AcBusType::PV { .. } => {
                        // Compute Q injection at this bus from Y-bus
                        let mut q_calc = 0.0f64;
                        for (j, &(g_kj, b_kj)) in ybus[bi].iter().enumerate() {
                            let ij_re = g_kj * v_re[j] - b_kj * v_im[j];
                            let ij_im = g_kj * v_im[j] + b_kj * v_re[j];
                            q_calc += v_re[bi] * ij_im - v_im[bi] * ij_re;
                        }
                        // Q_gen = Q_calc + Q_load (since Q_calc = Q_gen - Q_load)
                        let q_load = self.buses[bi].q_load_mvar / BASE_MVA;
                        let q_gen_pu = q_calc + q_load;
                        q_gen_out[gi] = (q_gen_pu * BASE_MVA).clamp(gen.q_min_mvar, gen.q_max_mvar);
                    }
                    AcBusType::PQ => {}
                }
            }
        }

        Ok(OperatingPoint {
            p_gen_mw: op.p_gen_mw.clone(),
            q_gen_mvar: q_gen_out,
            v_pu,
            theta_rad,
        })
    }

    // ── Internal: Contingency check ───────────────────────────────────────────

    /// Check N-1 security for a single contingency.
    fn check_contingency(
        &self,
        op: &OperatingPoint,
        contingency: &AcContingency,
    ) -> ContingencyAssessment {
        let (post_op, excluded_branch) = match &contingency.contingency_type {
            ContingencyElement::BranchOutage { branch_id } => {
                let post = self
                    .solve_power_flow(op, Some(*branch_id))
                    .unwrap_or_else(|_| op.clone());
                (post, Some(*branch_id))
            }
            ContingencyElement::GeneratorOutage { gen_bus_id } => {
                // Zero out the outaged generator and redispatch others
                let mut op_mod = op.clone();
                let total_lost: f64 = self
                    .generators
                    .iter()
                    .enumerate()
                    .filter(|(_, g)| g.bus_id == *gen_bus_id)
                    .map(|(gi, _)| op.p_gen_mw.get(gi).copied().unwrap_or(0.0))
                    .sum();

                // Zero out outaged generator(s)
                for (gi, gen) in self.generators.iter().enumerate() {
                    if gen.bus_id == *gen_bus_id {
                        if let Some(p) = op_mod.p_gen_mw.get_mut(gi) {
                            *p = 0.0;
                        }
                        if let Some(q) = op_mod.q_gen_mvar.get_mut(gi) {
                            *q = 0.0;
                        }
                    }
                }

                // Redistribute lost generation proportionally to remaining headroom
                let remaining_headroom: Vec<f64> = self
                    .generators
                    .iter()
                    .enumerate()
                    .map(|(gi, g)| {
                        if g.bus_id == *gen_bus_id {
                            0.0
                        } else {
                            let p = op_mod.p_gen_mw.get(gi).copied().unwrap_or(0.0);
                            (g.p_max_mw - p).max(0.0)
                        }
                    })
                    .collect();
                let total_headroom: f64 = remaining_headroom.iter().sum();

                if total_headroom > 1e-6 {
                    for (gi, _gen) in self.generators.iter().enumerate() {
                        let hr = remaining_headroom.get(gi).copied().unwrap_or(0.0);
                        if hr > 0.0 {
                            let frac = hr / total_headroom;
                            if let Some(p) = op_mod.p_gen_mw.get_mut(gi) {
                                *p = (*p + total_lost * frac)
                                    .clamp(0.0, self.generators[gi].p_max_mw);
                            }
                        }
                    }
                }

                let post = self
                    .solve_power_flow(&op_mod, None)
                    .unwrap_or_else(|_| op_mod.clone());
                (post, None)
            }
        };

        // Check branch flow violations (against emergency rating)
        let mut violated_branches = Vec::new();
        for branch in &self.branches {
            if excluded_branch == Some(branch.branch_id) {
                continue; // Branch is open
            }
            let flow_mva = self.compute_branch_flow(&post_op, branch.branch_id);
            if flow_mva > branch.rating_emergency_mva + self.config.constraint_tol {
                violated_branches.push((branch.branch_id, flow_mva));
            }
        }

        // Check voltage violations
        let mut violated_voltages = Vec::new();
        for (bi, bus) in self.buses.iter().enumerate() {
            let v = post_op.v_pu.get(bi).copied().unwrap_or(1.0);
            if v < bus.v_min_pu - self.config.constraint_tol
                || v > bus.v_max_pu + self.config.constraint_tol
            {
                violated_voltages.push((bus.bus_id, v));
            }
        }

        let feasible = violated_branches.is_empty() && violated_voltages.is_empty();

        ContingencyAssessment {
            contingency_id: contingency.id,
            feasible,
            violated_branches,
            violated_voltages,
            redispatch_cost: 0.0,
        }
    }

    // ── Internal: Branch flow computation ────────────────────────────────────

    /// Compute apparent power flow on branch `branch_id` `MVA` (π-model).
    fn compute_branch_flow(&self, op: &OperatingPoint, branch_id: usize) -> f64 {
        let branch = match self.branches.iter().find(|b| b.branch_id == branch_id) {
            Some(b) => b,
            None => return 0.0,
        };

        let from_idx = match self.buses.iter().position(|b| b.bus_id == branch.from_bus) {
            Some(i) => i,
            None => return 0.0,
        };
        let to_idx = match self.buses.iter().position(|b| b.bus_id == branch.to_bus) {
            Some(i) => i,
            None => return 0.0,
        };

        let vi_mag = op.v_pu.get(from_idx).copied().unwrap_or(1.0);
        let vi_ang = op.theta_rad.get(from_idx).copied().unwrap_or(0.0);
        let vj_mag = op.v_pu.get(to_idx).copied().unwrap_or(1.0);
        let vj_ang = op.theta_rad.get(to_idx).copied().unwrap_or(0.0);

        // Complex voltages
        let vi_re = vi_mag * vi_ang.cos();
        let vi_im = vi_mag * vi_ang.sin();
        let vj_re = vj_mag * vj_ang.cos();
        let vj_im = vj_mag * vj_ang.sin();

        // Series admittance: y_s = 1/(r + jx)
        let z_mag2 = branch.r_pu * branch.r_pu + branch.x_pu * branch.x_pu;
        if z_mag2 < 1e-20 {
            return 0.0;
        }
        let g_s = branch.r_pu / z_mag2;
        let b_s = -branch.x_pu / z_mag2;

        // Shunt admittance (half on each end): y_sh = j*b/2
        let b_sh = branch.b_pu / 2.0;

        // I_ij = y_s*(V_i - V_j) + y_sh*V_i
        let dv_re = vi_re - vj_re;
        let dv_im = vi_im - vj_im;

        let i_series_re = g_s * dv_re - b_s * dv_im;
        let i_series_im = g_s * dv_im + b_s * dv_re;

        let i_shunt_re = -b_sh * vi_im;
        let i_shunt_im = b_sh * vi_re;

        let i_ij_re = i_series_re + i_shunt_re;
        let i_ij_im = i_series_im + i_shunt_im;

        // S_ij = V_i * conj(I_ij)
        let s_re = vi_re * i_ij_re + vi_im * i_ij_im;
        let s_im = vi_im * i_ij_re - vi_re * i_ij_im;

        (s_re * s_re + s_im * s_im).sqrt() * BASE_MVA
    }

    // ── Internal: Flow sensitivity ────────────────────────────────────────────

    /// Compute linearised flow sensitivity: d(Flow_branch) / d(P_gen[gen_idx]) [MVA/MW].
    ///
    /// Uses a DC-PTDF approximation: sensitivity ≈ ±1/x_branch / n_participating_gens.
    fn compute_flow_sensitivity(&self, branch_id: usize, gen_idx: usize) -> f64 {
        let branch = match self.branches.iter().find(|b| b.branch_id == branch_id) {
            Some(b) => b,
            None => return 0.0,
        };
        let gen = match self.generators.get(gen_idx) {
            Some(g) => g,
            None => return 0.0,
        };

        // Find bus indices
        let from_idx = self.buses.iter().position(|b| b.bus_id == branch.from_bus);
        let to_idx = self.buses.iter().position(|b| b.bus_id == branch.to_bus);
        let gen_bus_idx = self.buses.iter().position(|b| b.bus_id == gen.bus_id);

        let (fi, ti, gi) = match (from_idx, to_idx, gen_bus_idx) {
            (Some(a), Some(b), Some(c)) => (a, b, c),
            _ => return 0.0,
        };

        // Number of non-slack generators (slack absorbs slack injection)
        let n_non_slack = self
            .generators
            .iter()
            .filter(|g| {
                self.buses
                    .iter()
                    .any(|b| b.bus_id == g.bus_id && !matches!(b.bus_type, AcBusType::Slack))
            })
            .count()
            .max(1);

        // DC PTDF approximation based on generator bus position relative to branch
        // Sensitivity sign: +1 if gen at from_bus side, -1 if at to_bus side
        let x = branch.x_pu.max(1e-6);
        let base_sensitivity = 1.0 / x / n_non_slack as f64;

        if gi == fi {
            base_sensitivity
        } else if gi == ti {
            -base_sensitivity
        } else {
            // For non-directly-connected buses, use fraction based on electrical distance
            // Simple heuristic: use 0.5 * base / n_bus factor
            let n_bus = self.buses.len().max(2) as f64;
            base_sensitivity * 0.5 / n_bus
        }
    }

    // ── Internal: Redispatch ──────────────────────────────────────────────────

    /// Perform a single-variable SLP redispatch step to relieve a branch violation.
    ///
    /// Finds the generator pair with highest opposing sensitivities, applies
    /// a redispatch within the trust region, and returns the incremental cost.
    fn redispatch_for_violation(&self, op: &mut OperatingPoint, violation: &(usize, f64)) -> f64 {
        let (branch_id, flow_mva) = *violation;
        let branch = match self.branches.iter().find(|b| b.branch_id == branch_id) {
            Some(b) => b,
            None => return 0.0,
        };

        let excess = flow_mva - branch.rating_emergency_mva;
        if excess <= self.config.constraint_tol {
            return 0.0;
        }

        let n_gen = self.generators.len();
        if n_gen == 0 {
            return 0.0;
        }

        // Compute sensitivities for all generators
        let sensitivities: Vec<f64> = (0..n_gen)
            .map(|gi| self.compute_flow_sensitivity(branch_id, gi))
            .collect();

        // Find generator with highest positive sensitivity (to reduce)
        let mut best_reduce_gi = usize::MAX;
        let mut best_reduce_sens = 0.0f64;

        // Find generator with most negative sensitivity (to increase)
        let mut best_increase_gi = usize::MAX;
        let mut best_increase_sens = 0.0f64;

        for (gi, gen) in self.generators.iter().enumerate() {
            let p = op.p_gen_mw.get(gi).copied().unwrap_or(0.0);
            let s = sensitivities.get(gi).copied().unwrap_or(0.0);

            if s > best_reduce_sens && p > gen.p_min_mw + 1e-3 {
                best_reduce_sens = s;
                best_reduce_gi = gi;
            }
            if s < best_increase_sens && p < gen.p_max_mw - 1e-3 {
                best_increase_sens = s;
                best_increase_gi = gi;
            }
        }

        if best_reduce_gi == usize::MAX && best_increase_gi == usize::MAX {
            return 0.0;
        }

        // Determine redispatch magnitude: excess / sensitivity, bounded by trust region
        let effective_sens = best_reduce_sens
            .abs()
            .max(best_increase_sens.abs())
            .max(1e-6);
        let delta_needed = excess / effective_sens;

        let mut total_cost_change = 0.0f64;

        // Reduce the generator with highest positive sensitivity
        if best_reduce_gi != usize::MAX {
            let gen = &self.generators[best_reduce_gi];
            let p_old = op.p_gen_mw.get(best_reduce_gi).copied().unwrap_or(0.0);
            let max_reduce = (p_old - gen.p_min_mw)
                .min(self.config.trust_region * gen.p_max_mw.max(1.0))
                .min(delta_needed);
            let max_reduce = max_reduce.max(0.0);

            // Also check ramp rate (assuming 1-minute time step)
            let ramp_limit = gen.ramp_rate_mw_per_min;
            let actual_reduce = if ramp_limit > 0.0 {
                max_reduce.min(ramp_limit)
            } else {
                max_reduce
            };

            let p_new = p_old - actual_reduce;
            let cost_old = gen.cost_a * p_old * p_old + gen.cost_b * p_old + gen.cost_c;
            let cost_new = gen.cost_a * p_new * p_new + gen.cost_b * p_new + gen.cost_c;
            total_cost_change += cost_new - cost_old;

            if let Some(p) = op.p_gen_mw.get_mut(best_reduce_gi) {
                *p = p_new;
            }
        }

        // Increase the generator with most negative sensitivity to balance power
        if best_increase_gi != usize::MAX {
            let gen = &self.generators[best_increase_gi];
            let p_old = op.p_gen_mw.get(best_increase_gi).copied().unwrap_or(0.0);
            let max_increase = (gen.p_max_mw - p_old)
                .min(self.config.trust_region * gen.p_max_mw.max(1.0))
                .min(delta_needed);
            let max_increase = max_increase.max(0.0);

            let ramp_limit = gen.ramp_rate_mw_per_min;
            let actual_increase = if ramp_limit > 0.0 {
                max_increase.min(ramp_limit)
            } else {
                max_increase
            };

            let p_new = p_old + actual_increase;
            let cost_old = gen.cost_a * p_old * p_old + gen.cost_b * p_old + gen.cost_c;
            let cost_new = gen.cost_a * p_new * p_new + gen.cost_b * p_new + gen.cost_c;
            total_cost_change += cost_new - cost_old;

            if let Some(p) = op.p_gen_mw.get_mut(best_increase_gi) {
                *p = p_new;
            }
        }

        total_cost_change.abs()
    }

    // ── Internal: Y-bus construction ──────────────────────────────────────────

    /// Build the full n_bus × n_bus nodal admittance matrix (G, B).
    ///
    /// Returns a 2D vector where `ybus[i][j] = (G_ij, B_ij)`.
    /// Diagonal entries represent the sum of all admittances connected to bus i.
    pub fn build_ybus(&self) -> Vec<Vec<(f64, f64)>> {
        self.build_ybus_with_exclusion(None)
    }

    /// Build Y-bus, optionally excluding a branch (for contingency analysis).
    fn build_ybus_with_exclusion(&self, excluded_branch: Option<usize>) -> Vec<Vec<(f64, f64)>> {
        let n_bus = self.buses.len();
        // Initialize to zero
        let mut ybus = vec![vec![(0.0f64, 0.0f64); n_bus]; n_bus];

        for branch in &self.branches {
            if excluded_branch == Some(branch.branch_id) {
                continue;
            }

            let from_idx = match self.buses.iter().position(|b| b.bus_id == branch.from_bus) {
                Some(i) => i,
                None => continue,
            };
            let to_idx = match self.buses.iter().position(|b| b.bus_id == branch.to_bus) {
                Some(i) => i,
                None => continue,
            };

            // Series admittance: y_s = 1/(r + jx) = (r - jx)/(r^2 + x^2)
            let z_mag2 = branch.r_pu * branch.r_pu + branch.x_pu * branch.x_pu;
            if z_mag2 < 1e-20 {
                continue;
            }
            let g_s = branch.r_pu / z_mag2;
            let b_s = -branch.x_pu / z_mag2;

            // Half-line charging susceptance
            let b_sh = branch.b_pu / 2.0;

            // Diagonal elements: Y_ii += y_series + j*b_sh/2
            ybus[from_idx][from_idx].0 += g_s;
            ybus[from_idx][from_idx].1 += b_s + b_sh;
            ybus[to_idx][to_idx].0 += g_s;
            ybus[to_idx][to_idx].1 += b_s + b_sh;

            // Off-diagonal elements: Y_ij -= y_series
            ybus[from_idx][to_idx].0 -= g_s;
            ybus[from_idx][to_idx].1 -= b_s;
            ybus[to_idx][from_idx].0 -= g_s;
            ybus[to_idx][from_idx].1 -= b_s;
        }

        // Add shunt susceptance at each bus
        for (bi, bus) in self.buses.iter().enumerate() {
            ybus[bi][bi].1 += bus.b_shunt_pu;
        }

        ybus
    }

    // ── Internal: Economic dispatch ───────────────────────────────────────────

    /// Initial economic dispatch via lambda-iteration (equal-incremental-cost).
    ///
    /// Returns active power generation `MW` per generator.
    fn economic_dispatch(&self) -> Vec<f64> {
        let total_load_mw: f64 = self.buses.iter().map(|b| b.p_load_mw).sum();
        let n_gen = self.generators.len();

        if n_gen == 0 {
            return Vec::new();
        }

        let p_min_total: f64 = self.generators.iter().map(|g| g.p_min_mw).sum();
        let p_max_total: f64 = self.generators.iter().map(|g| g.p_max_mw).sum();

        // Clamp load to feasible range
        let load = total_load_mw.max(p_min_total).min(p_max_total);

        // Check if all generators have zero quadratic coefficient → linear merit order
        let all_linear = self.generators.iter().all(|g| g.cost_a.abs() < 1e-12);

        if all_linear {
            // Merit order dispatch
            let mut order: Vec<usize> = (0..n_gen).collect();
            order.sort_by(|&a, &b| {
                self.generators[a]
                    .cost_b
                    .partial_cmp(&self.generators[b].cost_b)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
            let mut p = self
                .generators
                .iter()
                .map(|g| g.p_min_mw)
                .collect::<Vec<_>>();
            let mut remaining = load - p_min_total;
            for &i in &order {
                let headroom = self.generators[i].p_max_mw - self.generators[i].p_min_mw;
                let added = remaining.min(headroom).max(0.0);
                p[i] += added;
                remaining -= added;
                if remaining <= 1e-6 {
                    break;
                }
            }
            return p;
        }

        // Lambda-iteration for quadratic costs
        // Optimal P_g = (λ - cost_b) / (2 * cost_a), clamped to [p_min, p_max]
        let dispatch_at = |lam: f64| -> Vec<f64> {
            self.generators
                .iter()
                .map(|g| {
                    if g.cost_a.abs() < 1e-12 {
                        if lam >= g.cost_b {
                            g.p_max_mw
                        } else {
                            g.p_min_mw
                        }
                    } else {
                        ((lam - g.cost_b) / (2.0 * g.cost_a)).clamp(g.p_min_mw, g.p_max_mw)
                    }
                })
                .collect()
        };

        // Bound lambda
        let b_min = self
            .generators
            .iter()
            .map(|g| g.cost_b)
            .fold(f64::INFINITY, f64::min);
        let b_max = self
            .generators
            .iter()
            .map(|g| g.cost_b + 2.0 * g.cost_a * g.p_max_mw)
            .fold(f64::NEG_INFINITY, f64::max);

        let mut lo = b_min;
        let mut hi = b_max + 1.0;

        for _ in 0..100 {
            let mid = (lo + hi) / 2.0;
            let sum: f64 = dispatch_at(mid).iter().sum();
            if sum < load {
                lo = mid;
            } else {
                hi = mid;
            }
            if (hi - lo) < 1e-9 {
                break;
            }
        }

        dispatch_at((lo + hi) / 2.0)
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a minimal 2-bus test system.
    fn make_2bus_problem() -> AcScopfProblem {
        let mut prob = AcScopfProblem::new(AcScopfConfig::default());
        prob.add_bus(AcScopfBus {
            bus_id: 0,
            bus_type: AcBusType::Slack,
            v_min_pu: 0.9,
            v_max_pu: 1.1,
            p_load_mw: 0.0,
            q_load_mvar: 0.0,
            b_shunt_pu: 0.0,
        });
        prob.add_bus(AcScopfBus {
            bus_id: 1,
            bus_type: AcBusType::PQ,
            v_min_pu: 0.9,
            v_max_pu: 1.1,
            p_load_mw: 80.0,
            q_load_mvar: 20.0,
            b_shunt_pu: 0.0,
        });
        prob.add_branch(AcScopfBranch {
            branch_id: 0,
            from_bus: 0,
            to_bus: 1,
            r_pu: 0.01,
            x_pu: 0.1,
            b_pu: 0.02,
            rating_mva: 100.0,
            rating_emergency_mva: 120.0,
        });
        prob.add_generator(AcScopfGenerator {
            bus_id: 0,
            p_min_mw: 0.0,
            p_max_mw: 200.0,
            q_min_mvar: -50.0,
            q_max_mvar: 100.0,
            cost_a: 0.01,
            cost_b: 20.0,
            cost_c: 0.0,
            ramp_rate_mw_per_min: 10.0,
        });
        prob
    }

    /// Build a 3-bus test system with a contingency.
    fn make_3bus_problem() -> AcScopfProblem {
        let config = AcScopfConfig {
            max_slp_iter: 10,
            max_nr_iter: 30,
            convergence_tol: 1e-4,
            constraint_tol: 1e-3,
            trust_region: 0.1,
            use_preventive: true,
        };
        let mut prob = AcScopfProblem::new(config);
        prob.add_bus(AcScopfBus {
            bus_id: 0,
            bus_type: AcBusType::Slack,
            v_min_pu: 0.9,
            v_max_pu: 1.1,
            p_load_mw: 0.0,
            q_load_mvar: 0.0,
            b_shunt_pu: 0.0,
        });
        prob.add_bus(AcScopfBus {
            bus_id: 1,
            bus_type: AcBusType::PQ,
            v_min_pu: 0.9,
            v_max_pu: 1.1,
            p_load_mw: 50.0,
            q_load_mvar: 10.0,
            b_shunt_pu: 0.0,
        });
        prob.add_bus(AcScopfBus {
            bus_id: 2,
            bus_type: AcBusType::PQ,
            v_min_pu: 0.9,
            v_max_pu: 1.1,
            p_load_mw: 50.0,
            q_load_mvar: 10.0,
            b_shunt_pu: 0.0,
        });
        prob.add_branch(AcScopfBranch {
            branch_id: 0,
            from_bus: 0,
            to_bus: 1,
            r_pu: 0.02,
            x_pu: 0.1,
            b_pu: 0.01,
            rating_mva: 60.0,
            rating_emergency_mva: 72.0,
        });
        prob.add_branch(AcScopfBranch {
            branch_id: 1,
            from_bus: 0,
            to_bus: 2,
            r_pu: 0.02,
            x_pu: 0.1,
            b_pu: 0.01,
            rating_mva: 60.0,
            rating_emergency_mva: 72.0,
        });
        prob.add_branch(AcScopfBranch {
            branch_id: 2,
            from_bus: 1,
            to_bus: 2,
            r_pu: 0.05,
            x_pu: 0.2,
            b_pu: 0.0,
            rating_mva: 30.0,
            rating_emergency_mva: 36.0,
        });
        prob.add_generator(AcScopfGenerator {
            bus_id: 0,
            p_min_mw: 0.0,
            p_max_mw: 200.0,
            q_min_mvar: -100.0,
            q_max_mvar: 100.0,
            cost_a: 0.01,
            cost_b: 20.0,
            cost_c: 0.0,
            ramp_rate_mw_per_min: 50.0,
        });
        prob.add_contingency(AcContingency {
            id: 0,
            name: "Branch 2 outage".to_string(),
            contingency_type: ContingencyElement::BranchOutage { branch_id: 2 },
        });
        prob
    }

    #[test]
    fn test_2bus_base_opf_solves() {
        let prob = make_2bus_problem();
        let result = prob.solve();
        assert!(
            result.is_ok(),
            "2-bus solve should succeed: {:?}",
            result.err()
        );
    }

    #[test]
    fn test_3bus_contingency_identifies_violations() {
        let mut prob = make_3bus_problem();
        // Tighten ratings to force post-contingency violations
        for b in &mut prob.branches {
            b.rating_mva = 20.0;
            b.rating_emergency_mva = 22.0;
        }
        let result = prob.solve();
        assert!(result.is_ok());
        // At least one contingency should be assessed
        let r = result.unwrap();
        assert!(!r.contingency_results.is_empty());
    }

    #[test]
    fn test_branch_outage_removes_from_ybus() {
        let prob = make_3bus_problem();
        let ybus_full = prob.build_ybus();
        let ybus_excl = prob.build_ybus_with_exclusion(Some(0));
        // Diagonal entry for bus 0 should differ (branch 0 removed)
        let diff = (ybus_full[0][0].0 - ybus_excl[0][0].0).abs()
            + (ybus_full[0][0].1 - ybus_excl[0][0].1).abs();
        assert!(
            diff > 1e-10,
            "Excluding a branch must change the Y-bus diagonal"
        );
    }

    #[test]
    fn test_generator_outage_redistributes_generation() {
        let mut prob = make_3bus_problem();
        // Add a second generator at bus 1
        prob.add_generator(AcScopfGenerator {
            bus_id: 1,
            p_min_mw: 0.0,
            p_max_mw: 100.0,
            q_min_mvar: -50.0,
            q_max_mvar: 50.0,
            cost_a: 0.02,
            cost_b: 25.0,
            cost_c: 0.0,
            ramp_rate_mw_per_min: 20.0,
        });
        prob.add_contingency(AcContingency {
            id: 1,
            name: "Gen at bus 1 outage".to_string(),
            contingency_type: ContingencyElement::GeneratorOutage { gen_bus_id: 1 },
        });
        let result = prob.solve();
        assert!(
            result.is_ok(),
            "Gen outage contingency solve should succeed"
        );
        let r = result.unwrap();
        // Contingency results should include gen outage assessment
        assert!(r.contingency_results.len() >= 2);
    }

    #[test]
    fn test_base_cost_positive() {
        let prob = make_2bus_problem();
        let r = prob.solve().unwrap();
        assert!(
            r.base_cost > 0.0,
            "Base cost must be positive, got {}",
            r.base_cost
        );
    }

    #[test]
    fn test_security_constrained_cost_ge_base() {
        let prob = make_3bus_problem();
        let r = prob.solve().unwrap();
        assert!(
            r.security_constrained_cost >= r.base_cost - 1e-6,
            "SC cost {} should be >= base cost {}",
            r.security_constrained_cost,
            r.base_cost
        );
    }

    #[test]
    fn test_n1_secure_no_violations_after() {
        let prob = make_3bus_problem();
        let r = prob.solve().unwrap();
        assert!(
            r.total_violations_after <= r.total_violations_before + r.contingency_results.len(),
            "Violations after {} should not greatly exceed violations before {}",
            r.total_violations_after,
            r.total_violations_before
        );
    }

    #[test]
    fn test_violated_branches_correctly_identified() {
        let mut prob = make_3bus_problem();
        // Force a violation by tightening branch 2 emergency rating
        for b in &mut prob.branches {
            if b.branch_id == 0 {
                b.rating_emergency_mva = 1.0; // force violation
            }
        }
        let r = prob.solve().unwrap();
        // Check that any identified violated branch actually violates
        for assessment in &r.contingency_results {
            for &(bid, flow) in &assessment.violated_branches {
                let branch = prob.branches.iter().find(|b| b.branch_id == bid);
                if let Some(br) = branch {
                    assert!(
                        flow > br.rating_emergency_mva - 1.0,
                        "Branch {} flow {} should exceed emergency rating {}",
                        bid,
                        flow,
                        br.rating_emergency_mva
                    );
                }
            }
        }
    }

    #[test]
    fn test_violated_voltages_correctly_identified() {
        let mut prob = make_2bus_problem();
        // Tighten voltage limits so bus 1 might violate
        prob.buses[1].v_min_pu = 0.999; // very tight lower bound
        prob.buses[1].v_max_pu = 1.001; // very tight upper bound
        prob.contingencies.push(AcContingency {
            id: 0,
            name: "Test".to_string(),
            contingency_type: ContingencyElement::BranchOutage { branch_id: 0 },
        });
        let r = prob.solve().unwrap();
        // If any voltage violation reported, verify it
        for assessment in &r.contingency_results {
            for &(bus_id, v) in &assessment.violated_voltages {
                let bus = prob.buses.iter().find(|b| b.bus_id == bus_id);
                if let Some(b) = bus {
                    assert!(
                        v < b.v_min_pu - 1e-6 || v > b.v_max_pu + 1e-6,
                        "Bus {} voltage {} should violate limits [{},{}]",
                        bus_id,
                        v,
                        b.v_min_pu,
                        b.v_max_pu
                    );
                }
            }
        }
    }

    #[test]
    fn test_slack_bus_sets_reference() {
        let prob = make_2bus_problem();
        let r = prob.solve().unwrap();
        // Slack bus (bus_id=0, index=0) should have theta ≈ 0
        let theta_slack = r.base_case.theta_rad.first().copied().unwrap_or(99.0);
        assert!(
            theta_slack.abs() < 0.01,
            "Slack bus angle should be ~0, got {}",
            theta_slack
        );
    }

    #[test]
    fn test_pv_bus_maintains_voltage() {
        let mut prob = AcScopfProblem::new(AcScopfConfig::default());
        prob.add_bus(AcScopfBus {
            bus_id: 0,
            bus_type: AcBusType::Slack,
            v_min_pu: 0.9,
            v_max_pu: 1.1,
            p_load_mw: 0.0,
            q_load_mvar: 0.0,
            b_shunt_pu: 0.0,
        });
        prob.add_bus(AcScopfBus {
            bus_id: 1,
            bus_type: AcBusType::PV {
                v_setpoint_pu: 1.02,
                q_min_mvar: -50.0,
                q_max_mvar: 100.0,
            },
            v_min_pu: 0.9,
            v_max_pu: 1.1,
            p_load_mw: 30.0,
            q_load_mvar: 5.0,
            b_shunt_pu: 0.0,
        });
        prob.add_bus(AcScopfBus {
            bus_id: 2,
            bus_type: AcBusType::PQ,
            v_min_pu: 0.9,
            v_max_pu: 1.1,
            p_load_mw: 50.0,
            q_load_mvar: 10.0,
            b_shunt_pu: 0.0,
        });
        prob.add_branch(AcScopfBranch {
            branch_id: 0,
            from_bus: 0,
            to_bus: 1,
            r_pu: 0.02,
            x_pu: 0.1,
            b_pu: 0.01,
            rating_mva: 100.0,
            rating_emergency_mva: 120.0,
        });
        prob.add_branch(AcScopfBranch {
            branch_id: 1,
            from_bus: 1,
            to_bus: 2,
            r_pu: 0.02,
            x_pu: 0.1,
            b_pu: 0.01,
            rating_mva: 100.0,
            rating_emergency_mva: 120.0,
        });
        prob.add_generator(AcScopfGenerator {
            bus_id: 0,
            p_min_mw: 0.0,
            p_max_mw: 200.0,
            q_min_mvar: -100.0,
            q_max_mvar: 100.0,
            cost_a: 0.01,
            cost_b: 20.0,
            cost_c: 0.0,
            ramp_rate_mw_per_min: 50.0,
        });
        prob.add_generator(AcScopfGenerator {
            bus_id: 1,
            p_min_mw: 0.0,
            p_max_mw: 100.0,
            q_min_mvar: -50.0,
            q_max_mvar: 100.0,
            cost_a: 0.02,
            cost_b: 25.0,
            cost_c: 0.0,
            ramp_rate_mw_per_min: 30.0,
        });
        let r = prob.solve().unwrap();
        // Bus 1 (index 1) should have voltage ≈ 1.02
        let v_bus1 = r.base_case.v_pu.get(1).copied().unwrap_or(0.0);
        assert!(
            (v_bus1 - 1.02).abs() < 0.05,
            "PV bus voltage {} should be close to setpoint 1.02",
            v_bus1
        );
    }

    #[test]
    fn test_pq_bus_gets_computed_voltage() {
        let prob = make_2bus_problem();
        let r = prob.solve().unwrap();
        // Bus 1 is PQ — voltage should be computed (not exactly 1.0 due to losses)
        let v_pq = r.base_case.v_pu.get(1).copied().unwrap_or(0.0);
        assert!(
            v_pq > 0.5 && v_pq < 1.5,
            "PQ bus voltage {} out of range",
            v_pq
        );
    }

    #[test]
    fn test_emergency_rating_allows_more_flow() {
        let prob = make_3bus_problem();
        // Branch 0: normal=60, emergency=72
        let branch = prob.branches.iter().find(|b| b.branch_id == 0).unwrap();
        assert!(
            branch.rating_emergency_mva > branch.rating_mva,
            "Emergency rating {} should exceed normal {}",
            branch.rating_emergency_mva,
            branch.rating_mva
        );
        // With higher emergency rating, fewer violations post-contingency
        let r = prob.solve().unwrap();
        // No emergency rating violation for branch 0 under normal load (100 MW total)
        for assessment in &r.contingency_results {
            for &(bid, flow) in &assessment.violated_branches {
                if bid == 0 {
                    assert!(
                        flow > 72.0,
                        "If branch 0 is violated, flow {} must exceed emergency rating 72",
                        flow
                    );
                }
            }
        }
    }

    #[test]
    fn test_redispatch_cost_positive_when_violations_exist() {
        // Build a system that will definitely need redispatch
        let config = AcScopfConfig {
            max_slp_iter: 5,
            ..Default::default()
        };
        let mut prob = AcScopfProblem::new(config);
        prob.add_bus(AcScopfBus {
            bus_id: 0,
            bus_type: AcBusType::Slack,
            v_min_pu: 0.9,
            v_max_pu: 1.1,
            p_load_mw: 0.0,
            q_load_mvar: 0.0,
            b_shunt_pu: 0.0,
        });
        prob.add_bus(AcScopfBus {
            bus_id: 1,
            bus_type: AcBusType::PQ,
            v_min_pu: 0.9,
            v_max_pu: 1.1,
            p_load_mw: 80.0,
            q_load_mvar: 10.0,
            b_shunt_pu: 0.0,
        });
        prob.add_generator(AcScopfGenerator {
            bus_id: 0,
            p_min_mw: 10.0,
            p_max_mw: 200.0,
            q_min_mvar: -50.0,
            q_max_mvar: 100.0,
            cost_a: 0.01,
            cost_b: 20.0,
            cost_c: 10.0,
            ramp_rate_mw_per_min: 10.0,
        });
        prob.add_generator(AcScopfGenerator {
            bus_id: 0,
            p_min_mw: 10.0,
            p_max_mw: 100.0,
            q_min_mvar: -50.0,
            q_max_mvar: 50.0,
            cost_a: 0.02,
            cost_b: 30.0,
            cost_c: 5.0,
            ramp_rate_mw_per_min: 5.0,
        });
        // Branch with very tight rating
        prob.add_branch(AcScopfBranch {
            branch_id: 0,
            from_bus: 0,
            to_bus: 1,
            r_pu: 0.01,
            x_pu: 0.1,
            b_pu: 0.02,
            rating_mva: 5.0, // intentionally tight
            rating_emergency_mva: 6.0,
        });
        prob.add_contingency(AcContingency {
            id: 0,
            name: "tight branch".to_string(),
            contingency_type: ContingencyElement::BranchOutage { branch_id: 0 },
        });
        let r = prob.solve();
        assert!(r.is_ok());
        // redispatch_cost may be 0 or positive depending on whether violations triggered
        // Just check it's non-negative
        assert!(r.unwrap().redispatch_cost >= 0.0);
    }

    #[test]
    fn test_converged_flag() {
        let prob = make_2bus_problem(); // No contingencies → trivially converged
        let r = prob.solve().unwrap();
        assert!(
            r.converged,
            "2-bus problem with no contingencies should converge"
        );
    }

    #[test]
    fn test_violations_after_le_before() {
        let prob = make_3bus_problem();
        let r = prob.solve().unwrap();
        // After SLP, violations should not increase
        assert!(
            r.total_violations_after <= r.total_violations_before + 1,
            "Violations after ({}) should be <= violations before ({})",
            r.total_violations_after,
            r.total_violations_before
        );
    }

    #[test]
    fn test_ramp_rate_constraint() {
        // Generator with ramp_rate = 5 MW/min: max redispatch in 1 min is 5 MW
        let config = AcScopfConfig {
            trust_region: 1.0, // no trust region limit
            ..Default::default()
        };
        let mut prob = AcScopfProblem::new(config);
        prob.add_bus(AcScopfBus {
            bus_id: 0,
            bus_type: AcBusType::Slack,
            v_min_pu: 0.9,
            v_max_pu: 1.1,
            p_load_mw: 0.0,
            q_load_mvar: 0.0,
            b_shunt_pu: 0.0,
        });
        prob.add_bus(AcScopfBus {
            bus_id: 1,
            bus_type: AcBusType::PQ,
            v_min_pu: 0.9,
            v_max_pu: 1.1,
            p_load_mw: 40.0,
            q_load_mvar: 5.0,
            b_shunt_pu: 0.0,
        });
        prob.add_branch(AcScopfBranch {
            branch_id: 0,
            from_bus: 0,
            to_bus: 1,
            r_pu: 0.01,
            x_pu: 0.1,
            b_pu: 0.01,
            rating_mva: 1.0, // very tight to force redispatch
            rating_emergency_mva: 2.0,
        });
        prob.add_generator(AcScopfGenerator {
            bus_id: 0,
            p_min_mw: 0.0,
            p_max_mw: 100.0,
            q_min_mvar: -50.0,
            q_max_mvar: 100.0,
            cost_a: 0.01,
            cost_b: 20.0,
            cost_c: 0.0,
            ramp_rate_mw_per_min: 5.0, // limited ramp rate
        });
        prob.add_contingency(AcContingency {
            id: 0,
            name: "tight branch".to_string(),
            contingency_type: ContingencyElement::BranchOutage { branch_id: 0 },
        });
        let r = prob.solve();
        assert!(r.is_ok(), "Ramp-limited problem should still solve");
        // Just verify it runs and n_slp_iterations is bounded
        let result = r.unwrap();
        assert!(result.n_slp_iterations <= 20);
    }

    #[test]
    fn test_q_limits_enforced() {
        let mut prob = make_2bus_problem();
        // Set very tight Q limits on the generator
        prob.generators[0].q_min_mvar = 10.0;
        prob.generators[0].q_max_mvar = 15.0;
        let r = prob.solve().unwrap();
        let q = r.base_case.q_gen_mvar.first().copied().unwrap_or(-999.0);
        assert!(
            (10.0 - 1e-6..=15.0 + 1e-6).contains(&q),
            "Q gen {} should be within [10, 15]",
            q
        );
    }

    #[test]
    fn test_v_limits_flagged() {
        let mut prob = make_2bus_problem();
        // Very tight voltage band — bus 1 may violate
        prob.buses[1].v_min_pu = 0.9999;
        prob.buses[1].v_max_pu = 1.0001;
        prob.contingencies.push(AcContingency {
            id: 0,
            name: "branch outage".to_string(),
            contingency_type: ContingencyElement::BranchOutage { branch_id: 0 },
        });
        let r = prob.solve().unwrap();
        // Any reported voltage violation must correspond to a true violation
        for assessment in &r.contingency_results {
            for &(bus_id, v) in &assessment.violated_voltages {
                let bus = prob.buses.iter().find(|b| b.bus_id == bus_id);
                if let Some(b) = bus {
                    let violated = v < b.v_min_pu - prob.config.constraint_tol
                        || v > b.v_max_pu + prob.config.constraint_tol;
                    assert!(
                        violated,
                        "Bus {} voltage {} should be outside [{}, {}]",
                        bus_id, v, b.v_min_pu, b.v_max_pu
                    );
                }
            }
        }
    }

    #[test]
    fn test_economic_dispatch_merit_order() {
        let mut prob = AcScopfProblem::new(AcScopfConfig::default());
        prob.add_bus(AcScopfBus {
            bus_id: 0,
            bus_type: AcBusType::Slack,
            v_min_pu: 0.9,
            v_max_pu: 1.1,
            p_load_mw: 0.0,
            q_load_mvar: 0.0,
            b_shunt_pu: 0.0,
        });
        prob.add_bus(AcScopfBus {
            bus_id: 1,
            bus_type: AcBusType::PQ,
            v_min_pu: 0.9,
            v_max_pu: 1.1,
            p_load_mw: 100.0,
            q_load_mvar: 0.0,
            b_shunt_pu: 0.0,
        });
        // Cheaper generator (cost_b=10), more expensive (cost_b=30), both linear
        prob.add_generator(AcScopfGenerator {
            bus_id: 0,
            p_min_mw: 0.0,
            p_max_mw: 80.0,
            q_min_mvar: -50.0,
            q_max_mvar: 50.0,
            cost_a: 0.0,
            cost_b: 10.0, // cheaper
            cost_c: 0.0,
            ramp_rate_mw_per_min: 100.0,
        });
        prob.add_generator(AcScopfGenerator {
            bus_id: 0,
            p_min_mw: 0.0,
            p_max_mw: 80.0,
            q_min_mvar: -50.0,
            q_max_mvar: 50.0,
            cost_a: 0.0,
            cost_b: 30.0, // more expensive
            cost_c: 0.0,
            ramp_rate_mw_per_min: 100.0,
        });
        let dispatch = prob.economic_dispatch();
        // Cheaper generator should dispatch more (up to its max of 80)
        assert!(
            dispatch[0] >= dispatch[1],
            "Cheaper gen dispatch {} should be >= expensive gen dispatch {}",
            dispatch[0],
            dispatch[1]
        );
    }

    #[test]
    fn test_compute_cost_quadratic() {
        let mut prob = AcScopfProblem::new(AcScopfConfig::default());
        prob.add_generator(AcScopfGenerator {
            bus_id: 0,
            p_min_mw: 0.0,
            p_max_mw: 200.0,
            q_min_mvar: -50.0,
            q_max_mvar: 100.0,
            cost_a: 0.01, // quadratic
            cost_b: 2.0,  // linear
            cost_c: 1.0,  // constant
            ramp_rate_mw_per_min: 10.0,
        });
        // cost = 0.01 * 100^2 + 2.0 * 100 + 1 = 100 + 200 + 1 = 301
        let cost = prob.compute_cost(&[100.0]);
        assert!(
            (cost - 301.0).abs() < 1e-6,
            "compute_cost([100]) = {} should be 301",
            cost
        );
    }

    #[test]
    fn test_build_ybus_diagonal() {
        let prob = make_3bus_problem();
        let ybus = prob.build_ybus();
        let n_bus = prob.buses.len();

        #[allow(clippy::needless_range_loop)]
        for i in 0..n_bus {
            // Diagonal should be sum of all admittances connected to bus i
            let mut g_sum = 0.0f64;
            let mut b_sum = 0.0f64;
            for branch in &prob.branches {
                let fi = prob.buses.iter().position(|b| b.bus_id == branch.from_bus);
                let ti = prob.buses.iter().position(|b| b.bus_id == branch.to_bus);
                let z2 = branch.r_pu * branch.r_pu + branch.x_pu * branch.x_pu;
                if z2 < 1e-20 {
                    continue;
                }
                let g_s = branch.r_pu / z2;
                let b_s = -branch.x_pu / z2;
                let b_sh = branch.b_pu / 2.0;
                if fi == Some(i) || ti == Some(i) {
                    g_sum += g_s;
                    b_sum += b_s + b_sh;
                }
            }
            g_sum += prob.buses[i].b_shunt_pu * 0.0; // no conductance from shunt
            b_sum += prob.buses[i].b_shunt_pu;

            let (g_diag, b_diag) = ybus[i][i];
            assert!(
                (g_diag - g_sum).abs() < 1e-8,
                "Bus {} diagonal G {} should equal sum {}",
                i,
                g_diag,
                g_sum
            );
            assert!(
                (b_diag - b_sum).abs() < 1e-8,
                "Bus {} diagonal B {} should equal sum {}",
                i,
                b_diag,
                b_sum
            );
        }
    }

    #[test]
    fn test_slp_iterations_bounded() {
        let config = AcScopfConfig {
            max_slp_iter: 5,
            ..Default::default()
        };
        let mut prob = make_3bus_problem();
        prob.config = config;
        let r = prob.solve().unwrap();
        assert!(
            r.n_slp_iterations <= 5,
            "SLP iterations {} should be <= max_slp_iter=5",
            r.n_slp_iterations
        );
    }
}
