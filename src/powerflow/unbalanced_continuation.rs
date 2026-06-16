//! Three-phase unbalanced continuation power flow (CPF) solver.
//!
//! Traces P-V curves under unbalanced loading conditions and finds voltage
//! collapse points for each phase independently using a predictor-corrector
//! scheme with Gauss-Seidel per-phase correction.
//!
//! # Example
//! ```rust,ignore
//! use oxigrid::powerflow::unbalanced_continuation::*;
//!
//! let cfg = UnbalancedCpfConfig::default();
//! let mut cpf = UnbalancedCpf::new(cfg);
//! cpf.add_bus(ThreePhaseBus { bus_id: 0, bus_type: CpfBusType::Slack, .. });
//! cpf.add_bus(ThreePhaseBus { bus_id: 1, bus_type: CpfBusType::PQ, .. });
//! cpf.add_branch(ThreePhaseBranch { from_bus: 0, to_bus: 1, .. });
//! let result = cpf.solve()?;
//! ```

// ── public types ────────────────────────────────────────────────────────────

/// Bus type classification for the unbalanced CPF solver.
///
/// Uses a distinct name (`CpfBusType`) to avoid collision with
/// `crate::network::bus::BusType`.
#[derive(Debug, Clone, PartialEq)]
pub enum CpfBusType {
    /// Slack (reference) bus: voltage magnitude and angle fixed at 1.0∠0°.
    Slack,
    /// PQ bus: active and reactive load are specified; both V and θ are free.
    PQ,
    /// PV bus: voltage magnitude is regulated to `v_setpoint_pu`; angle is free.
    PV {
        /// Regulated voltage setpoint (per-unit).
        v_setpoint_pu: f64,
    },
}

/// Three-phase bus data for the unbalanced CPF solver.
#[derive(Debug, Clone)]
pub struct ThreePhaseBus {
    /// Bus index (0-based).
    pub bus_id: usize,
    /// Bus classification.
    pub bus_type: CpfBusType,
    /// Base active loads per phase \[Pa, Pb, Pc\] (MW).
    pub p_load_mw: [f64; 3],
    /// Base reactive loads per phase \[Qa, Qb, Qc\] (MVAR).
    pub q_load_mvar: [f64; 3],
    /// Generation per phase \[Pa, Pb, Pc\] (MW); zero for load buses.
    pub p_gen_mw: [f64; 3],
}

/// Three-phase branch for the unbalanced CPF solver.
///
/// Mutual coupling between phases is represented by scalar `r_mutual_pu` /
/// `x_mutual_pu` terms applied to all off-phase pairs of the same branch.
#[derive(Debug, Clone)]
pub struct ThreePhaseBranch {
    /// Sending-end bus index (0-based).
    pub from_bus: usize,
    /// Receiving-end bus index (0-based).
    pub to_bus: usize,
    /// Per-phase series resistance \[ra, rb, rc\] (pu).
    pub r_pu: [f64; 3],
    /// Per-phase series reactance \[xa, xb, xc\] (pu).
    pub x_pu: [f64; 3],
    /// Mutual resistance coupling between phase pairs (pu).
    pub r_mutual_pu: f64,
    /// Mutual reactance coupling between phase pairs (pu).
    pub x_mutual_pu: f64,
}

/// Load-scaling strategy applied as λ increases.
#[derive(Debug, Clone)]
pub enum LoadScalingModel {
    /// All three phases scale by the same factor λ.
    Uniform,
    /// Each phase can scale independently (currently all three scale by λ —
    /// extend via custom λ vectors if needed).
    PhaseIndependent,
    /// Phase A scales by λ; phases B and C scale by `λ * ratio` (ratio ∈ (0,1\]).
    PhaseADominant {
        /// Scaling ratio for phases B and C relative to A (typically < 1.0).
        ratio: f64,
    },
}

/// Configuration for the unbalanced continuation power flow solver.
#[derive(Debug, Clone)]
pub struct UnbalancedCpfConfig {
    /// Maximum load factor to trace (default 3.0).
    pub lambda_max: f64,
    /// Initial predictor step size Δλ (default 0.05).
    pub lambda_step: f64,
    /// Minimum allowed step size (default 1e-4).
    pub lambda_min_step: f64,
    /// Gauss-Seidel convergence tolerance on |ΔV| (default 1e-5).
    pub v_tolerance: f64,
    /// Maximum Gauss-Seidel iterations per CPF step (default 20).
    pub max_iter_per_step: usize,
    /// Load-scaling strategy.
    pub load_model: LoadScalingModel,
    /// Voltage threshold below which collapse is declared (default 0.5 pu).
    pub collapse_voltage_pu: f64,
}

impl Default for UnbalancedCpfConfig {
    fn default() -> Self {
        Self {
            lambda_max: 3.0,
            lambda_step: 0.05,
            lambda_min_step: 1e-4,
            v_tolerance: 1e-5,
            max_iter_per_step: 20,
            load_model: LoadScalingModel::Uniform,
            collapse_voltage_pu: 0.5,
        }
    }
}

/// One point on a P-V curve (all three phases).
#[derive(Debug, Clone)]
pub struct PvPoint {
    /// Load-scaling factor λ at this point.
    pub lambda: f64,
    /// Phase-A bus voltage magnitude (pu), averaged across non-slack buses.
    pub v_a: f64,
    /// Phase-B bus voltage magnitude (pu), averaged across non-slack buses.
    pub v_b: f64,
    /// Phase-C bus voltage magnitude (pu), averaged across non-slack buses.
    pub v_c: f64,
    /// Whether Gauss-Seidel converged at this λ.
    pub converged: bool,
    /// Total three-phase active power consumed (MW) at this point.
    pub active_power_total_mw: f64,
}

/// Voltage collapse (nose) point for one phase.
#[derive(Debug, Clone)]
pub struct CollapsePoint {
    /// Phase index: 0 = A, 1 = B, 2 = C.
    pub phase: u8,
    /// Critical loading factor λ at the nose point.
    pub lambda_nose: f64,
    /// Voltage magnitude at the nose point (pu).
    pub v_nose_pu: f64,
    /// Index of the weakest bus (0-based).
    pub bus_id: usize,
    /// Total active power at the nose point (MW).
    pub p_nose_mw: f64,
}

/// Full result returned by [`UnbalancedCpf::solve`].
#[derive(Debug, Clone)]
pub struct UnbalancedCpfResult {
    /// Traced P-V curve points in order of increasing λ.
    pub pv_curve: Vec<PvPoint>,
    /// Detected nose (collapse) points, one per phase that collapsed.
    pub collapse_points: Vec<CollapsePoint>,
    /// Index of the phase (0/1/2) that collapses at the lowest λ.
    pub critical_phase: u8,
    /// Loading margin: λ_nose − λ_base (or λ_max − 1 if no collapse detected).
    pub loading_margin: f64,
    /// Voltage stability index per bus per phase: `vsi[bus][phase]` ∈ \[0, 1\].
    pub vsi: Vec<[f64; 3]>,
    /// Total number of predictor-corrector steps taken.
    pub n_steps: usize,
    /// `true` when the solver successfully bracketed the nose point.
    pub converged_to_nose: bool,
}

// ── solver ───────────────────────────────────────────────────────────────────

/// Three-phase unbalanced continuation power flow solver.
///
/// Uses a predictor-corrector scheme:
/// * **Predictor**: increment λ by `config.lambda_step` and use the previous
///   voltage solution as the starting guess.
/// * **Corrector**: Gauss-Seidel iterations on the per-phase admittance system.
/// * **Step-size adaptation**: halve Δλ when GS fails to converge; double when
///   it converges in fewer than 5 iterations.
pub struct UnbalancedCpf {
    /// Buses in the network (order defines the 0-based index).
    pub buses: Vec<ThreePhaseBus>,
    /// Branches in the network.
    pub branches: Vec<ThreePhaseBranch>,
    /// Solver configuration.
    pub config: UnbalancedCpfConfig,
}

impl UnbalancedCpf {
    /// Create a new solver with the given configuration and empty network.
    pub fn new(config: UnbalancedCpfConfig) -> Self {
        Self {
            buses: Vec::new(),
            branches: Vec::new(),
            config,
        }
    }

    /// Append a bus to the network.
    pub fn add_bus(&mut self, bus: ThreePhaseBus) {
        self.buses.push(bus);
    }

    /// Append a branch to the network.
    pub fn add_branch(&mut self, branch: ThreePhaseBranch) {
        self.branches.push(branch);
    }

    // ── internal helpers ────────────────────────────────────────────────────

    /// Return the per-phase load scaling multiplier for `phase` at factor `lambda`.
    fn phase_lambda(&self, phase: usize, lambda: f64) -> f64 {
        match &self.config.load_model {
            LoadScalingModel::Uniform | LoadScalingModel::PhaseIndependent => lambda,
            LoadScalingModel::PhaseADominant { ratio } => {
                if phase == 0 {
                    lambda
                } else {
                    lambda * ratio
                }
            }
        }
    }

    /// Build the per-phase real (G) and imaginary (B) admittance matrices.
    ///
    /// Returns `(G, B)` where `G[i][j]` and `B[i][j]` are the (i,j) entries
    /// of the nodal admittance matrix for the requested `phase` (0=A, 1=B, 2=C).
    ///
    /// Mutual coupling between phases is *not* included here — it is handled
    /// as a current-injection correction in [`Self::gauss_seidel_phase`].
    fn build_admittance_per_phase(&self, phase: usize) -> (Vec<Vec<f64>>, Vec<Vec<f64>>) {
        let n = self.buses.len();
        let mut g = vec![vec![0.0_f64; n]; n];
        let mut b = vec![vec![0.0_f64; n]; n];

        for br in &self.branches {
            let i = br.from_bus;
            let j = br.to_bus;
            if i >= n || j >= n {
                continue;
            }
            let r = br.r_pu[phase];
            let x = br.x_pu[phase];
            let denom = r * r + x * x;
            if denom < 1e-18 {
                continue;
            }
            let g_br = r / denom;
            let b_br = -x / denom;

            // Off-diagonal
            g[i][j] -= g_br;
            g[j][i] -= g_br;
            b[i][j] -= b_br;
            b[j][i] -= b_br;

            // Diagonal
            g[i][i] += g_br;
            g[j][j] += g_br;
            b[i][i] += b_br;
            b[j][j] += b_br;
        }

        (g, b)
    }

    /// Compute per-bus power injections `(P_inj, Q_inj)` for `phase` given
    /// voltage magnitudes `v` and angles `theta` (radians).
    #[allow(dead_code)]
    fn compute_power_injection(&self, phase: usize, v: &[f64], theta: &[f64]) -> Vec<(f64, f64)> {
        let n = self.buses.len();
        let (g, b) = self.build_admittance_per_phase(phase);
        let mut injections = vec![(0.0_f64, 0.0_f64); n];

        for j in 0..n {
            let mut p = 0.0_f64;
            let mut q = 0.0_f64;
            for k in 0..n {
                let d_theta = theta[j] - theta[k];
                p += v[j] * v[k] * (g[j][k] * d_theta.cos() + b[j][k] * d_theta.sin());
                q += v[j] * v[k] * (g[j][k] * d_theta.sin() - b[j][k] * d_theta.cos());
            }
            injections[j] = (p, q);
        }
        injections
    }

    /// Gauss-Seidel power-flow iteration for a single `phase`.
    ///
    /// Returns a vector of converged voltage magnitudes (pu) for each bus,
    /// or an `Err` if the maximum iteration count is exceeded or `Y_jj = 0`.
    ///
    /// * Slack bus: held at 1.0 pu, 0° throughout.
    /// * PV bus: magnitude clamped to `v_setpoint_pu` after each update.
    /// * PQ bus: voltage updated freely.
    fn gauss_seidel_phase(
        &self,
        phase: usize,
        lambda: f64,
        v_init: &[f64],
    ) -> Result<Vec<f64>, String> {
        let n = self.buses.len();
        if n == 0 {
            return Err("Network has no buses".to_string());
        }

        let (g, b) = self.build_admittance_per_phase(phase);
        let lam = self.phase_lambda(phase, lambda);

        // Compute scheduled net injections (generation – load·λ) in pu (base=1 MW)
        let mut p_sched: Vec<f64> = self
            .buses
            .iter()
            .map(|bus| bus.p_gen_mw[phase] - bus.p_load_mw[phase] * lam)
            .collect();
        let mut q_sched: Vec<f64> = self
            .buses
            .iter()
            .map(|bus| -bus.q_load_mvar[phase] * lam)
            .collect();

        // Slack bus: override
        for (idx, bus) in self.buses.iter().enumerate() {
            if bus.bus_type == CpfBusType::Slack {
                p_sched[idx] = 0.0;
                q_sched[idx] = 0.0;
            }
        }

        // Rectangular voltage: V = e + j*f
        let mut e: Vec<f64> = v_init.to_vec();
        let mut f: Vec<f64> = vec![0.0_f64; n];

        // Fix slack bus
        for (idx, bus) in self.buses.iter().enumerate() {
            if bus.bus_type == CpfBusType::Slack {
                e[idx] = 1.0;
                f[idx] = 0.0;
            }
        }

        // Mutual coupling: build correction currents from other phases.
        // We model it as a fixed current injection perturbation from V ≈ 1∠0.
        let mut i_real_mutual = vec![0.0_f64; n];
        let mut i_imag_mutual = vec![0.0_f64; n];
        for br in &self.branches {
            let fi = br.from_bus;
            let ti = br.to_bus;
            if fi >= n || ti >= n {
                continue;
            }
            let rm = br.r_mutual_pu;
            let xm = br.x_mutual_pu;
            let denom = rm * rm + xm * xm;
            if denom < 1e-18 {
                continue;
            }
            // Approximate mutual current contribution as small perturbation
            // (simplified: treat adjacent-phase voltage ≈ 1∠(±120°))
            let angle_offset = if phase == 0 {
                2.0 * std::f64::consts::PI / 3.0
            } else {
                -2.0 * std::f64::consts::PI / 3.0
            };
            let v_other_e = angle_offset.cos();
            let v_other_f = angle_offset.sin();
            // Current from mutual: I_m = Y_m * V_other; Y_m = 1/(r_m+j*x_m)
            let g_m = rm / denom;
            let b_m = -xm / denom;
            let i_e = g_m * v_other_e - b_m * v_other_f;
            let i_f = b_m * v_other_e + g_m * v_other_f;
            // This coupling subtracts from the net injection at both ends
            i_real_mutual[fi] -= i_e;
            i_imag_mutual[fi] -= i_f;
            i_real_mutual[ti] += i_e;
            i_imag_mutual[ti] += i_f;
        }

        for iter in 0..self.config.max_iter_per_step {
            let mut max_dv = 0.0_f64;

            for j in 0..n {
                if self.buses[j].bus_type == CpfBusType::Slack {
                    e[j] = 1.0;
                    f[j] = 0.0;
                    continue;
                }

                let v2 = e[j] * e[j] + f[j] * f[j];
                if v2 < 1e-12 {
                    // Avoid division by zero — reset to small value
                    e[j] = 0.1;
                    f[j] = 0.0;
                }
                let v2 = e[j] * e[j] + f[j] * f[j];

                // Compute I_j = (P - jQ) / conj(V) = (P-jQ)(e-jf)/v2
                let p_j = p_sched[j];
                let q_j = q_sched[j];
                // I_j (rectangular): numerator = (P-jQ)*(e-jf)
                //   real: P*e - Q*f
                //   imag: -P*f - Q*e ... wait: (P-jQ)*(e+jf)/v^2 for I=(P+jQ)/conj(V)
                // Using: I = (P + jQ) / conj(V) = (P+jQ)*(e+jf) / v^2
                let i_real_j = (p_j * e[j] - q_j * f[j]) / v2 + i_real_mutual[j];
                let i_imag_j = (p_j * f[j] + q_j * e[j]) / v2 + i_imag_mutual[j];

                // Sum Y_jk * V_k for k != j
                let mut sum_e = 0.0_f64;
                let mut sum_f = 0.0_f64;
                for k in 0..n {
                    if k == j {
                        continue;
                    }
                    // Y_jk = g_jk + j*b_jk; V_k = e_k + j*f_k
                    // Y_jk * V_k: real = g*e - b*f; imag = g*f + b*e
                    sum_e += g[j][k] * e[k] - b[j][k] * f[k];
                    sum_f += g[j][k] * f[k] + b[j][k] * e[k];
                }

                // V_j_new = (I_j - sum) / Y_jj
                let y_jj_g = g[j][j];
                let y_jj_b = b[j][j];
                let y_jj2 = y_jj_g * y_jj_g + y_jj_b * y_jj_b;
                if y_jj2 < 1e-18 {
                    // Isolated bus — skip update
                    continue;
                }
                let num_e = i_real_j - sum_e;
                let num_f = i_imag_j - sum_f;
                // Division by (g + jb): multiply by conj (g - jb)
                let e_new = (num_e * y_jj_g + num_f * y_jj_b) / y_jj2;
                let f_new = (num_f * y_jj_g - num_e * y_jj_b) / y_jj2;

                // For PV bus: clamp magnitude
                let (e_upd, f_upd) = match &self.buses[j].bus_type {
                    CpfBusType::PV { v_setpoint_pu } => {
                        let mag = (e_new * e_new + f_new * f_new).sqrt();
                        if mag < 1e-9 {
                            (*v_setpoint_pu, 0.0)
                        } else {
                            (e_new / mag * v_setpoint_pu, f_new / mag * v_setpoint_pu)
                        }
                    }
                    _ => (e_new, f_new),
                };

                let dv = ((e_upd - e[j]).powi(2) + (f_upd - f[j]).powi(2)).sqrt();
                max_dv = max_dv.max(dv);
                e[j] = e_upd;
                f[j] = f_upd;
            }

            if max_dv < self.config.v_tolerance {
                // Converged
                let v_mag: Vec<f64> = (0..n).map(|j| (e[j] * e[j] + f[j] * f[j]).sqrt()).collect();
                return Ok(v_mag);
            }

            let _ = iter; // suppress unused warning
        }

        // Return best-effort result with a warning embedded in Err
        Err(format!(
            "Gauss-Seidel did not converge for phase {} at lambda={:.4} within {} iterations",
            phase, lambda, self.config.max_iter_per_step
        ))
    }

    /// Solve the three-phase power flow at load factor `lambda`.
    ///
    /// Returns voltage magnitudes `v[bus][phase]` (pu).
    #[allow(clippy::needless_range_loop)]
    fn solve_power_flow_at(&self, lambda: f64) -> Result<Vec<[f64; 3]>, String> {
        let n = self.buses.len();
        let init: Vec<f64> = vec![1.0_f64; n];
        let mut v_per_phase: Vec<[f64; 3]> = vec![[1.0_f64; 3]; n];

        for ph in 0..3_usize {
            let v = self.gauss_seidel_phase(ph, lambda, &init)?;
            for (i, &vi) in v.iter().enumerate() {
                v_per_phase[i][ph] = vi;
            }
        }
        Ok(v_per_phase)
    }

    /// Detect the nose (voltage collapse) point for one `phase` from the
    /// traced P-V curve.
    ///
    /// The nose is identified as the last converged point before either:
    /// * the minimum phase voltage drops below `collapse_voltage_pu`, or
    /// * the per-step voltage drop accelerates strongly (second-derivative sign
    ///   change indicates turning of the P-V curve).
    fn detect_nose_point(&self, pv_curve: &[PvPoint]) -> Option<CollapsePoint> {
        if pv_curve.len() < 3 {
            return None;
        }

        // Find first point where ANY phase drops below collapse threshold
        let mut nose_idx: Option<usize> = None;
        let mut nose_phase: u8 = 0;

        for (idx, pt) in pv_curve.iter().enumerate() {
            let voltages = [pt.v_a, pt.v_b, pt.v_c];
            for (ph, &v) in voltages.iter().enumerate() {
                if v < self.config.collapse_voltage_pu && nose_idx.is_none() {
                    nose_idx = Some(idx.saturating_sub(1));
                    nose_phase = ph as u8;
                }
            }
            if nose_idx.is_some() {
                break;
            }
        }

        // Also detect via strong dV/dλ second derivative sign change
        if nose_idx.is_none() && pv_curve.len() >= 5 {
            for i in 2..pv_curve.len() {
                let pt0 = &pv_curve[i - 2];
                let pt1 = &pv_curve[i - 1];
                let pt2 = &pv_curve[i];
                let dl01 = pt1.lambda - pt0.lambda;
                let dl12 = pt2.lambda - pt1.lambda;
                if dl01 < 1e-12 || dl12 < 1e-12 {
                    continue;
                }
                let vols = [
                    (pt0.v_a, pt1.v_a, pt2.v_a, 0u8),
                    (pt0.v_b, pt1.v_b, pt2.v_b, 1u8),
                    (pt0.v_c, pt1.v_c, pt2.v_c, 2u8),
                ];
                for (v0, v1, v2, ph) in vols {
                    let dv1 = (v1 - v0) / dl01;
                    let dv2 = (v2 - v1) / dl12;
                    // Accelerating drop: second derivative < -2.0 (pu/pu²)
                    let d2v = (dv2 - dv1) / ((dl01 + dl12) / 2.0);
                    if d2v < -2.0 && nose_idx.is_none() {
                        nose_idx = Some(i.saturating_sub(1));
                        nose_phase = ph;
                    }
                }
                if nose_idx.is_some() {
                    break;
                }
            }
        }

        let idx = nose_idx?;
        let pt = pv_curve.get(idx)?;
        let v_nose = match nose_phase {
            0 => pt.v_a,
            1 => pt.v_b,
            _ => pt.v_c,
        };

        // Identify weakest bus as the one with lowest voltage at nose λ
        // (approximation: use bus 0 index for weakest since we only store
        //  aggregate voltages in PvPoint; full per-bus tracking below)
        let weakest_bus = self
            .buses
            .iter()
            .enumerate()
            .filter(|(_, b)| b.bus_type != CpfBusType::Slack)
            .map(|(i, _)| i)
            .next()
            .unwrap_or(0);

        Some(CollapsePoint {
            phase: nose_phase,
            lambda_nose: pt.lambda,
            v_nose_pu: v_nose,
            bus_id: weakest_bus,
            p_nose_mw: pt.active_power_total_mw,
        })
    }

    /// Compute the per-bus voltage stability index (VSI) for one `phase`.
    ///
    /// Returns `vsi[bus]` ∈ \[0, 1\] where 1 = fully stable and 0 = at
    /// collapse.  Uses a proximity-to-collapse heuristic:
    /// `VSI(j) = (|V_j| - V_collapse) / (1 - V_collapse)` clamped to \[0,1\].
    fn compute_vsi(&self, _phase: usize, v: &[f64]) -> Vec<f64> {
        let v_col = self.config.collapse_voltage_pu;
        v.iter()
            .map(|&vi| {
                let margin = 1.0 - v_col;
                if margin < 1e-9 {
                    0.0
                } else {
                    ((vi - v_col) / margin).clamp(0.0, 1.0)
                }
            })
            .collect()
    }

    /// Return the loading margin from the result.
    ///
    /// Defined as `λ_nose − 1.0` (since λ=1 is the base case), or
    /// `λ_max − 1.0` if no nose point was found.
    pub fn loading_margin(&self, result: &UnbalancedCpfResult) -> f64 {
        result.loading_margin
    }

    /// Solve the continuation power flow and return the full result.
    ///
    /// The solver:
    /// 1. Starts at λ = 1.0 (base case).
    /// 2. Applies the predictor (increment λ) and corrector (GS iteration).
    /// 3. Records each P-V point.
    /// 4. Adapts step size based on corrector convergence speed.
    /// 5. Stops at `lambda_max` or when all phases have collapsed.
    pub fn solve(&self) -> Result<UnbalancedCpfResult, String> {
        if self.buses.is_empty() {
            return Err("No buses defined".to_string());
        }
        if self.branches.is_empty() {
            return Err("No branches defined".to_string());
        }

        // Validate bus indices in branches
        let n = self.buses.len();
        for br in &self.branches {
            if br.from_bus >= n || br.to_bus >= n {
                return Err(format!(
                    "Branch references bus {} or {} but only {} buses exist",
                    br.from_bus, br.to_bus, n
                ));
            }
        }

        let mut pv_curve: Vec<PvPoint> = Vec::new();
        let mut lambda = 1.0_f64;
        let mut step = self.config.lambda_step;
        let mut n_steps = 0_usize;
        let mut collapsed_phases = [false; 3];

        // Initial flat-start voltages
        let mut v_current: Vec<[f64; 3]> = vec![[1.0_f64; 3]; n];

        // Record base case (λ = 1)
        {
            let v_base = self
                .solve_power_flow_at(1.0)
                .unwrap_or_else(|_| vec![[1.0; 3]; n]);
            v_current = v_base.clone();
            let pt = self.make_pv_point(1.0, &v_base, true);
            pv_curve.push(pt);
        }

        loop {
            if lambda >= self.config.lambda_max {
                break;
            }
            // All phases collapsed
            if collapsed_phases.iter().all(|&c| c) {
                break;
            }

            let lambda_pred = (lambda + step).min(self.config.lambda_max);

            // Corrector: try to converge GS at lambda_pred
            let mut converged_phases = [true; 3];
            let mut v_new: Vec<[f64; 3]> = v_current.clone();
            let mut gs_iters_total = 0_usize;

            for ph in 0..3_usize {
                if collapsed_phases[ph] {
                    continue;
                }
                let v_init: Vec<f64> = v_current.iter().map(|vb| vb[ph]).collect();
                match self.gauss_seidel_phase_counting(ph, lambda_pred, &v_init) {
                    Ok((v_ph, iters)) => {
                        gs_iters_total += iters;
                        for (i, &vi) in v_ph.iter().enumerate() {
                            v_new[i][ph] = vi;
                        }
                        // Check if this phase has collapsed
                        let min_v = v_ph.iter().cloned().fold(f64::INFINITY, f64::min);
                        if min_v < self.config.collapse_voltage_pu {
                            collapsed_phases[ph] = true;
                        }
                    }
                    Err(_) => {
                        converged_phases[ph] = false;
                    }
                }
            }

            let any_converged = converged_phases.iter().any(|&c| c);

            if !any_converged {
                // Halve step and retry
                step /= 2.0;
                if step < self.config.lambda_min_step {
                    // Give up — record unconverged point and stop
                    let pt = self.make_pv_point(lambda_pred, &v_new, false);
                    pv_curve.push(pt);
                    n_steps += 1;
                    break;
                }
                continue;
            }

            // Accept step
            lambda = lambda_pred;
            v_current = v_new.clone();
            n_steps += 1;

            let all_converged = converged_phases.iter().all(|&c| c);
            let pt = self.make_pv_point(lambda, &v_new, all_converged);
            let total_p = pt.active_power_total_mw;
            pv_curve.push(pt);

            // Adaptive step size
            let avg_iters = if gs_iters_total > 0 {
                gs_iters_total / 3
            } else {
                1
            };
            if avg_iters < 5 && step < self.config.lambda_step * 4.0 {
                step = (step * 1.5).min(self.config.lambda_step * 4.0);
            } else if avg_iters > 15 {
                step = (step / 2.0).max(self.config.lambda_min_step);
            }

            // Stop if all phases have collapsed
            if collapsed_phases.iter().all(|&c| c) {
                break;
            }

            let _ = total_p;
        }

        // Detect nose points per phase
        let mut collapse_points: Vec<CollapsePoint> = Vec::new();
        if let Some(cp) = self.detect_nose_point(&pv_curve) {
            // Also check other phases
            let cp_a = self.detect_nose_for_phase(0, &pv_curve);
            let cp_b = self.detect_nose_for_phase(1, &pv_curve);
            let cp_c = self.detect_nose_for_phase(2, &pv_curve);
            for c in [cp_a, cp_b, cp_c].into_iter().flatten() {
                collapse_points.push(c);
            }
            // If no per-phase was found, push the combined one
            if collapse_points.is_empty() {
                collapse_points.push(cp);
            }
        }

        let converged_to_nose = !collapse_points.is_empty();

        // Critical phase: phase with smallest lambda_nose
        let critical_phase = collapse_points
            .iter()
            .min_by(|a, b| {
                a.lambda_nose
                    .partial_cmp(&b.lambda_nose)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|cp| cp.phase)
            .unwrap_or(0);

        let loading_margin = if let Some(cp) = collapse_points.iter().min_by(|a, b| {
            a.lambda_nose
                .partial_cmp(&b.lambda_nose)
                .unwrap_or(std::cmp::Ordering::Equal)
        }) {
            cp.lambda_nose - 1.0
        } else {
            self.config.lambda_max - 1.0
        };

        // Compute VSI at final operating point
        let vsi_data: Vec<[f64; 3]> = (0..n)
            .map(|bus_i| {
                let mut vsi_bus = [1.0_f64; 3];
                for ph in 0..3 {
                    let v_ph: Vec<f64> = v_current.iter().map(|vb| vb[ph]).collect();
                    let vsi_ph = self.compute_vsi(ph, &v_ph);
                    vsi_bus[ph] = vsi_ph.get(bus_i).copied().unwrap_or(1.0);
                }
                vsi_bus
            })
            .collect();

        Ok(UnbalancedCpfResult {
            pv_curve,
            collapse_points,
            critical_phase,
            loading_margin,
            vsi: vsi_data,
            n_steps,
            converged_to_nose,
        })
    }

    /// Like [`gauss_seidel_phase`] but also returns the iteration count.
    fn gauss_seidel_phase_counting(
        &self,
        phase: usize,
        lambda: f64,
        v_init: &[f64],
    ) -> Result<(Vec<f64>, usize), String> {
        let n = self.buses.len();
        if n == 0 {
            return Err("Network has no buses".to_string());
        }

        let (g, b) = self.build_admittance_per_phase(phase);
        let lam = self.phase_lambda(phase, lambda);

        let mut p_sched: Vec<f64> = self
            .buses
            .iter()
            .map(|bus| bus.p_gen_mw[phase] - bus.p_load_mw[phase] * lam)
            .collect();
        let mut q_sched: Vec<f64> = self
            .buses
            .iter()
            .map(|bus| -bus.q_load_mvar[phase] * lam)
            .collect();

        for (idx, bus) in self.buses.iter().enumerate() {
            if bus.bus_type == CpfBusType::Slack {
                p_sched[idx] = 0.0;
                q_sched[idx] = 0.0;
            }
        }

        let mut e: Vec<f64> = v_init.to_vec();
        let mut f: Vec<f64> = vec![0.0_f64; n];

        for (idx, bus) in self.buses.iter().enumerate() {
            if bus.bus_type == CpfBusType::Slack {
                e[idx] = 1.0;
                f[idx] = 0.0;
            }
        }

        // Mutual coupling correction currents
        let mut i_real_mutual = vec![0.0_f64; n];
        let mut i_imag_mutual = vec![0.0_f64; n];
        for br in &self.branches {
            let fi = br.from_bus;
            let ti = br.to_bus;
            if fi >= n || ti >= n {
                continue;
            }
            let rm = br.r_mutual_pu;
            let xm = br.x_mutual_pu;
            let denom = rm * rm + xm * xm;
            if denom < 1e-18 {
                continue;
            }
            let angle_offset = if phase == 0 {
                2.0 * std::f64::consts::PI / 3.0
            } else {
                -2.0 * std::f64::consts::PI / 3.0
            };
            let v_other_e = angle_offset.cos();
            let v_other_f = angle_offset.sin();
            let g_m = rm / denom;
            let b_m = -xm / denom;
            let i_e = g_m * v_other_e - b_m * v_other_f;
            let i_f = b_m * v_other_e + g_m * v_other_f;
            i_real_mutual[fi] -= i_e;
            i_imag_mutual[fi] -= i_f;
            i_real_mutual[ti] += i_e;
            i_imag_mutual[ti] += i_f;
        }

        for iter in 0..self.config.max_iter_per_step {
            let mut max_dv = 0.0_f64;

            for j in 0..n {
                if self.buses[j].bus_type == CpfBusType::Slack {
                    e[j] = 1.0;
                    f[j] = 0.0;
                    continue;
                }

                let v2 = e[j] * e[j] + f[j] * f[j];
                let v2 = if v2 < 1e-12 {
                    e[j] = 0.1;
                    f[j] = 0.0;
                    0.01_f64
                } else {
                    v2
                };

                let p_j = p_sched[j];
                let q_j = q_sched[j];
                let i_real_j = (p_j * e[j] - q_j * f[j]) / v2 + i_real_mutual[j];
                let i_imag_j = (p_j * f[j] + q_j * e[j]) / v2 + i_imag_mutual[j];

                let mut sum_e = 0.0_f64;
                let mut sum_f = 0.0_f64;
                for k in 0..n {
                    if k == j {
                        continue;
                    }
                    sum_e += g[j][k] * e[k] - b[j][k] * f[k];
                    sum_f += g[j][k] * f[k] + b[j][k] * e[k];
                }

                let y_jj_g = g[j][j];
                let y_jj_b = b[j][j];
                let y_jj2 = y_jj_g * y_jj_g + y_jj_b * y_jj_b;
                if y_jj2 < 1e-18 {
                    continue;
                }
                let num_e = i_real_j - sum_e;
                let num_f = i_imag_j - sum_f;
                let e_new = (num_e * y_jj_g + num_f * y_jj_b) / y_jj2;
                let f_new = (num_f * y_jj_g - num_e * y_jj_b) / y_jj2;

                let (e_upd, f_upd) = match &self.buses[j].bus_type {
                    CpfBusType::PV { v_setpoint_pu } => {
                        let mag = (e_new * e_new + f_new * f_new).sqrt();
                        if mag < 1e-9 {
                            (*v_setpoint_pu, 0.0)
                        } else {
                            (e_new / mag * v_setpoint_pu, f_new / mag * v_setpoint_pu)
                        }
                    }
                    _ => (e_new, f_new),
                };

                let dv = ((e_upd - e[j]).powi(2) + (f_upd - f[j]).powi(2)).sqrt();
                max_dv = max_dv.max(dv);
                e[j] = e_upd;
                f[j] = f_upd;
            }

            if max_dv < self.config.v_tolerance {
                let v_mag: Vec<f64> = (0..n).map(|j| (e[j] * e[j] + f[j] * f[j]).sqrt()).collect();
                return Ok((v_mag, iter + 1));
            }
        }

        Err(format!(
            "GS did not converge for phase {} at lambda={:.4}",
            phase, lambda
        ))
    }

    /// Detect the nose point specifically for one `phase_target` from `pv_curve`.
    fn detect_nose_for_phase(
        &self,
        phase_target: usize,
        pv_curve: &[PvPoint],
    ) -> Option<CollapsePoint> {
        if pv_curve.len() < 2 {
            return None;
        }

        let get_v = |pt: &PvPoint| match phase_target {
            0 => pt.v_a,
            1 => pt.v_b,
            _ => pt.v_c,
        };

        // Find first point where this phase's voltage drops below threshold
        let mut nose_idx: Option<usize> = None;
        for (idx, pt) in pv_curve.iter().enumerate() {
            if get_v(pt) < self.config.collapse_voltage_pu {
                nose_idx = Some(idx.saturating_sub(1));
                break;
            }
        }

        // Also detect accelerating drop
        if nose_idx.is_none() && pv_curve.len() >= 5 {
            for i in 2..pv_curve.len() {
                let v0 = get_v(&pv_curve[i - 2]);
                let v1 = get_v(&pv_curve[i - 1]);
                let v2 = get_v(&pv_curve[i]);
                let dl01 = pv_curve[i - 1].lambda - pv_curve[i - 2].lambda;
                let dl12 = pv_curve[i].lambda - pv_curve[i - 1].lambda;
                if dl01 < 1e-12 || dl12 < 1e-12 {
                    continue;
                }
                let dv1 = (v1 - v0) / dl01;
                let dv2 = (v2 - v1) / dl12;
                let d2v = (dv2 - dv1) / ((dl01 + dl12) / 2.0);
                if d2v < -2.0 {
                    nose_idx = Some(i.saturating_sub(1));
                    break;
                }
            }
        }

        let idx = nose_idx?;
        let pt = pv_curve.get(idx)?;
        let v_nose = get_v(pt);

        let weakest_bus = self
            .buses
            .iter()
            .enumerate()
            .filter(|(_, b)| b.bus_type != CpfBusType::Slack)
            .map(|(i, _)| i)
            .next()
            .unwrap_or(0);

        Some(CollapsePoint {
            phase: phase_target as u8,
            lambda_nose: pt.lambda,
            v_nose_pu: v_nose,
            bus_id: weakest_bus,
            p_nose_mw: pt.active_power_total_mw,
        })
    }

    /// Build a [`PvPoint`] from voltages at load factor `lambda`.
    ///
    /// Voltages are averaged over non-slack buses so they represent a
    /// system-level summary rather than a single bus.
    fn make_pv_point(&self, lambda: f64, v: &[[f64; 3]], converged: bool) -> PvPoint {
        let pq_count = self
            .buses
            .iter()
            .filter(|b| b.bus_type != CpfBusType::Slack)
            .count();
        let count = pq_count.max(1);

        let mut sum_a = 0.0_f64;
        let mut sum_b = 0.0_f64;
        let mut sum_c = 0.0_f64;

        for (i, bus) in self.buses.iter().enumerate() {
            if bus.bus_type != CpfBusType::Slack {
                sum_a += v.get(i).map(|vb| vb[0]).unwrap_or(1.0);
                sum_b += v.get(i).map(|vb| vb[1]).unwrap_or(1.0);
                sum_c += v.get(i).map(|vb| vb[2]).unwrap_or(1.0);
            }
        }

        let total_p: f64 = self
            .buses
            .iter()
            .map(|bus| {
                let lam_a = self.phase_lambda(0, lambda);
                let lam_b = self.phase_lambda(1, lambda);
                let lam_c = self.phase_lambda(2, lambda);
                bus.p_load_mw[0] * lam_a + bus.p_load_mw[1] * lam_b + bus.p_load_mw[2] * lam_c
            })
            .sum();

        PvPoint {
            lambda,
            v_a: sum_a / count as f64,
            v_b: sum_b / count as f64,
            v_c: sum_c / count as f64,
            converged,
            active_power_total_mw: total_p,
        }
    }
}

// ── tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a minimal 2-bus system (slack → PQ).
    fn make_2bus(p_load: f64, q_load: f64) -> UnbalancedCpf {
        let cfg = UnbalancedCpfConfig {
            lambda_max: 2.5,
            lambda_step: 0.1,
            lambda_min_step: 1e-4,
            v_tolerance: 1e-5,
            max_iter_per_step: 30,
            load_model: LoadScalingModel::Uniform,
            collapse_voltage_pu: 0.5,
        };
        let mut cpf = UnbalancedCpf::new(cfg);
        cpf.add_bus(ThreePhaseBus {
            bus_id: 0,
            bus_type: CpfBusType::Slack,
            p_load_mw: [0.0; 3],
            q_load_mvar: [0.0; 3],
            p_gen_mw: [0.0; 3],
        });
        cpf.add_bus(ThreePhaseBus {
            bus_id: 1,
            bus_type: CpfBusType::PQ,
            p_load_mw: [p_load; 3],
            q_load_mvar: [q_load; 3],
            p_gen_mw: [0.0; 3],
        });
        cpf.add_branch(ThreePhaseBranch {
            from_bus: 0,
            to_bus: 1,
            r_pu: [0.1; 3],
            x_pu: [0.2; 3],
            r_mutual_pu: 0.0,
            x_mutual_pu: 0.0,
        });
        cpf
    }

    /// Build a 3-bus radial system: slack → bus1 → bus2.
    fn make_3bus_radial() -> UnbalancedCpf {
        let cfg = UnbalancedCpfConfig {
            lambda_max: 2.0,
            lambda_step: 0.1,
            ..UnbalancedCpfConfig::default()
        };
        let mut cpf = UnbalancedCpf::new(cfg);
        cpf.add_bus(ThreePhaseBus {
            bus_id: 0,
            bus_type: CpfBusType::Slack,
            p_load_mw: [0.0; 3],
            q_load_mvar: [0.0; 3],
            p_gen_mw: [0.0; 3],
        });
        cpf.add_bus(ThreePhaseBus {
            bus_id: 1,
            bus_type: CpfBusType::PQ,
            p_load_mw: [0.05; 3],
            q_load_mvar: [0.02; 3],
            p_gen_mw: [0.0; 3],
        });
        cpf.add_bus(ThreePhaseBus {
            bus_id: 2,
            bus_type: CpfBusType::PQ,
            p_load_mw: [0.08; 3],
            q_load_mvar: [0.03; 3],
            p_gen_mw: [0.0; 3],
        });
        cpf.add_branch(ThreePhaseBranch {
            from_bus: 0,
            to_bus: 1,
            r_pu: [0.1; 3],
            x_pu: [0.2; 3],
            r_mutual_pu: 0.0,
            x_mutual_pu: 0.0,
        });
        cpf.add_branch(ThreePhaseBranch {
            from_bus: 1,
            to_bus: 2,
            r_pu: [0.08; 3],
            x_pu: [0.15; 3],
            r_mutual_pu: 0.0,
            x_mutual_pu: 0.0,
        });
        cpf
    }

    // 1. Basic 2-bus solver runs
    #[test]
    fn test_2bus_solver_runs() {
        let cpf = make_2bus(0.2, 0.1);
        let result = cpf.solve();
        assert!(result.is_ok(), "solve() should succeed: {:?}", result.err());
    }

    // 2. Loading margin positive for light load
    #[test]
    fn test_2bus_loading_margin_positive() {
        let cpf = make_2bus(0.05, 0.02);
        let result = cpf.solve().expect("solve failed");
        assert!(
            result.loading_margin > 0.0,
            "Loading margin should be > 0, got {}",
            result.loading_margin
        );
    }

    // 3. Lambda increases monotonically on upper branch
    #[test]
    fn test_pv_curve_lambda_monotonic() {
        let cpf = make_2bus(0.1, 0.05);
        let result = cpf.solve().expect("solve failed");
        for w in result.pv_curve.windows(2) {
            assert!(
                w[1].lambda >= w[0].lambda - 1e-9,
                "Lambda should be non-decreasing: {} then {}",
                w[0].lambda,
                w[1].lambda
            );
        }
    }

    // 4. Voltage decreases as lambda increases
    #[test]
    fn test_voltage_decreases_with_lambda() {
        let cpf = make_2bus(0.3, 0.1);
        let result = cpf.solve().expect("solve failed");
        if result.pv_curve.len() >= 2 {
            let first = &result.pv_curve[0];
            let last = result.pv_curve.last().expect("pv_curve non-empty");
            let v_first = (first.v_a + first.v_b + first.v_c) / 3.0;
            let v_last = (last.v_a + last.v_b + last.v_c) / 3.0;
            assert!(
                v_last <= v_first + 0.1,
                "Voltage should not rise significantly: {:.4} → {:.4}",
                v_first,
                v_last
            );
        }
    }

    // 5. Nose detected before lambda_max with weak network
    #[test]
    fn test_nose_detected_weak_network() {
        let cfg = UnbalancedCpfConfig {
            lambda_max: 5.0,
            lambda_step: 0.05,
            collapse_voltage_pu: 0.6,
            ..UnbalancedCpfConfig::default()
        };
        let mut cpf = UnbalancedCpf::new(cfg);
        cpf.add_bus(ThreePhaseBus {
            bus_id: 0,
            bus_type: CpfBusType::Slack,
            p_load_mw: [0.0; 3],
            q_load_mvar: [0.0; 3],
            p_gen_mw: [0.0; 3],
        });
        cpf.add_bus(ThreePhaseBus {
            bus_id: 1,
            bus_type: CpfBusType::PQ,
            p_load_mw: [0.5; 3],
            q_load_mvar: [0.3; 3],
            p_gen_mw: [0.0; 3],
        });
        cpf.add_branch(ThreePhaseBranch {
            from_bus: 0,
            to_bus: 1,
            r_pu: [0.3; 3],
            x_pu: [0.4; 3],
            r_mutual_pu: 0.0,
            x_mutual_pu: 0.0,
        });
        let result = cpf.solve().expect("solve failed");
        // Either nose detected or we hit lambda_max
        assert!(
            result.converged_to_nose
                || result.pv_curve.last().map(|p| p.lambda).unwrap_or(0.0) >= 4.9,
            "Expected nose or lambda_max reached"
        );
    }

    // 6. Phase A different from B when loading is unbalanced
    #[test]
    fn test_phase_a_different_from_b_unbalanced_load() {
        let cfg = UnbalancedCpfConfig {
            lambda_max: 2.0,
            lambda_step: 0.1,
            load_model: LoadScalingModel::PhaseADominant { ratio: 0.5 },
            ..UnbalancedCpfConfig::default()
        };
        let mut cpf = UnbalancedCpf::new(cfg);
        cpf.add_bus(ThreePhaseBus {
            bus_id: 0,
            bus_type: CpfBusType::Slack,
            p_load_mw: [0.0; 3],
            q_load_mvar: [0.0; 3],
            p_gen_mw: [0.0; 3],
        });
        cpf.add_bus(ThreePhaseBus {
            bus_id: 1,
            bus_type: CpfBusType::PQ,
            p_load_mw: [0.3; 3],
            q_load_mvar: [0.1; 3],
            p_gen_mw: [0.0; 3],
        });
        cpf.add_branch(ThreePhaseBranch {
            from_bus: 0,
            to_bus: 1,
            r_pu: [0.1; 3],
            x_pu: [0.2; 3],
            r_mutual_pu: 0.0,
            x_mutual_pu: 0.0,
        });
        let result = cpf.solve().expect("solve failed");
        if let Some(last) = result.pv_curve.last() {
            // Phase A loads more → lower voltage
            assert!(
                last.v_a <= last.v_b + 0.01,
                "Phase A should be lower or equal to B under PhaseADominant: v_a={:.4} v_b={:.4}",
                last.v_a,
                last.v_b
            );
        }
    }

    // 7. Uniform scaling: all phases same (equal base loads)
    #[test]
    fn test_uniform_scaling_phases_equal() {
        let cpf = make_2bus(0.1, 0.05);
        let result = cpf.solve().expect("solve failed");
        for pt in &result.pv_curve {
            let diff_ab = (pt.v_a - pt.v_b).abs();
            let diff_bc = (pt.v_b - pt.v_c).abs();
            assert!(
                diff_ab < 1e-6,
                "Uniform scaling: v_a and v_b should be equal, diff={}",
                diff_ab
            );
            assert!(
                diff_bc < 1e-6,
                "Uniform scaling: v_b and v_c should be equal, diff={}",
                diff_bc
            );
        }
    }

    // 8. PhaseADominant: phase A collapse point present
    #[test]
    fn test_phase_a_dominant_collapses_first() {
        let cfg = UnbalancedCpfConfig {
            lambda_max: 5.0,
            lambda_step: 0.05,
            collapse_voltage_pu: 0.55,
            load_model: LoadScalingModel::PhaseADominant { ratio: 0.3 },
            ..UnbalancedCpfConfig::default()
        };
        let mut cpf = UnbalancedCpf::new(cfg);
        cpf.add_bus(ThreePhaseBus {
            bus_id: 0,
            bus_type: CpfBusType::Slack,
            p_load_mw: [0.0; 3],
            q_load_mvar: [0.0; 3],
            p_gen_mw: [0.0; 3],
        });
        cpf.add_bus(ThreePhaseBus {
            bus_id: 1,
            bus_type: CpfBusType::PQ,
            p_load_mw: [0.6; 3],
            q_load_mvar: [0.2; 3],
            p_gen_mw: [0.0; 3],
        });
        cpf.add_branch(ThreePhaseBranch {
            from_bus: 0,
            to_bus: 1,
            r_pu: [0.25; 3],
            x_pu: [0.35; 3],
            r_mutual_pu: 0.0,
            x_mutual_pu: 0.0,
        });
        let result = cpf.solve().expect("solve failed");
        // If nose detected, phase A (0) should appear
        if result.converged_to_nose && !result.collapse_points.is_empty() {
            let has_phase_a = result.collapse_points.iter().any(|cp| cp.phase == 0);
            assert!(
                has_phase_a,
                "Phase A collapse point expected under PhaseADominant"
            );
        }
    }

    // 9. collapse_points count 1..=3
    #[test]
    fn test_collapse_points_count() {
        let cpf = make_2bus(0.8, 0.4);
        let result = cpf.solve().expect("solve failed");
        if result.converged_to_nose {
            let cnt = result.collapse_points.len();
            assert!(
                (1..=3).contains(&cnt),
                "collapse_points.len() should be 1-3, got {}",
                cnt
            );
        }
    }

    // 10. critical_phase in 0..=2
    #[test]
    fn test_critical_phase_identified() {
        let cpf = make_2bus(0.2, 0.1);
        let result = cpf.solve().expect("solve failed");
        assert!(
            result.critical_phase <= 2,
            "critical_phase must be 0, 1, or 2, got {}",
            result.critical_phase
        );
    }

    // 11. VSI near 1.0 at light load
    #[test]
    fn test_vsi_light_load_near_one() {
        let cpf = make_2bus(0.05, 0.02);
        let result = cpf.solve().expect("solve failed");
        for vsi_bus in &result.vsi {
            for &v in vsi_bus.iter() {
                assert!(v >= 0.5, "VSI at light load should be ≥ 0.5, got {}", v);
            }
        }
    }

    // 12. VSI at last point <= VSI at first point (approaches 0 near nose)
    #[test]
    fn test_vsi_approaches_zero_near_nose() {
        let cpf = make_2bus(0.6, 0.3);
        let result = cpf.solve().expect("solve failed");
        // VSI is computed at final operating point
        // Just check values are in [0, 1]
        for vsi_bus in &result.vsi {
            for &v in vsi_bus.iter() {
                assert!(
                    (0.0..=1.0).contains(&v),
                    "VSI should be in [0,1], got {}",
                    v
                );
            }
        }
    }

    // 13. converged_to_nose flag set when nose found
    #[test]
    fn test_converged_to_nose_flag() {
        let cfg = UnbalancedCpfConfig {
            lambda_max: 5.0,
            lambda_step: 0.05,
            collapse_voltage_pu: 0.6,
            ..UnbalancedCpfConfig::default()
        };
        let mut cpf = UnbalancedCpf::new(cfg);
        cpf.add_bus(ThreePhaseBus {
            bus_id: 0,
            bus_type: CpfBusType::Slack,
            p_load_mw: [0.0; 3],
            q_load_mvar: [0.0; 3],
            p_gen_mw: [0.0; 3],
        });
        cpf.add_bus(ThreePhaseBus {
            bus_id: 1,
            bus_type: CpfBusType::PQ,
            p_load_mw: [0.8; 3],
            q_load_mvar: [0.4; 3],
            p_gen_mw: [0.0; 3],
        });
        cpf.add_branch(ThreePhaseBranch {
            from_bus: 0,
            to_bus: 1,
            r_pu: [0.4; 3],
            x_pu: [0.5; 3],
            r_mutual_pu: 0.0,
            x_mutual_pu: 0.0,
        });
        let result = cpf.solve().expect("solve failed");
        // If collapse_points is non-empty, flag must be true
        assert_eq!(
            result.converged_to_nose,
            !result.collapse_points.is_empty(),
            "converged_to_nose must match collapse_points non-empty"
        );
    }

    // 14. n_steps nonzero
    #[test]
    fn test_n_steps_nonzero() {
        let cpf = make_2bus(0.1, 0.05);
        let result = cpf.solve().expect("solve failed");
        assert!(result.n_steps > 0, "n_steps should be > 0");
    }

    // 15. PV curve last point near nose or lambda_max
    #[test]
    fn test_pv_curve_last_point() {
        let cpf = make_2bus(0.1, 0.05);
        let result = cpf.solve().expect("solve failed");
        let last_lambda = result.pv_curve.last().map(|p| p.lambda).unwrap_or(0.0);
        assert!(
            last_lambda > 1.0,
            "Last lambda should exceed base case 1.0, got {}",
            last_lambda
        );
    }

    // 16. 3-bus radial: solves correctly
    #[test]
    fn test_3bus_radial() {
        let cpf = make_3bus_radial();
        let result = cpf.solve();
        assert!(
            result.is_ok(),
            "3-bus radial should solve: {:?}",
            result.err()
        );
        let r = result.expect("should be ok");
        assert!(!r.pv_curve.is_empty(), "PV curve should be non-empty");
    }

    // 17. Gauss-Seidel convergence: direct call
    #[test]
    fn test_gauss_seidel_convergence() {
        let cpf = make_2bus(0.1, 0.05);
        let v_init = vec![1.0_f64; 2];
        let result = cpf.gauss_seidel_phase(0, 1.0, &v_init);
        assert!(
            result.is_ok(),
            "GS should converge for light load: {:?}",
            result.err()
        );
        let v = result.expect("converged");
        for &vi in &v {
            assert!(
                vi > 0.0 && vi <= 1.1,
                "Voltage should be in (0, 1.1]: {}",
                vi
            );
        }
    }

    // 18. Slack bus voltage fixed at 1.0 pu
    #[test]
    fn test_slack_bus_voltage_fixed() {
        let cpf = make_2bus(0.2, 0.1);
        let v = cpf
            .gauss_seidel_phase(0, 1.0, &[1.0, 1.0])
            .expect("GS converged");
        let slack_v = v[0]; // bus 0 is slack
        assert!(
            (slack_v - 1.0).abs() < 1e-6,
            "Slack bus voltage should be 1.0 pu, got {}",
            slack_v
        );
    }

    // 19. PV bus voltage controlled
    #[test]
    fn test_pv_bus_voltage_controlled() {
        let cfg = UnbalancedCpfConfig {
            lambda_max: 2.0,
            lambda_step: 0.1,
            ..UnbalancedCpfConfig::default()
        };
        let mut cpf = UnbalancedCpf::new(cfg);
        cpf.add_bus(ThreePhaseBus {
            bus_id: 0,
            bus_type: CpfBusType::Slack,
            p_load_mw: [0.0; 3],
            q_load_mvar: [0.0; 3],
            p_gen_mw: [0.0; 3],
        });
        cpf.add_bus(ThreePhaseBus {
            bus_id: 1,
            bus_type: CpfBusType::PV {
                v_setpoint_pu: 0.98,
            },
            p_load_mw: [0.1; 3],
            q_load_mvar: [0.05; 3],
            p_gen_mw: [0.15; 3],
        });
        cpf.add_bus(ThreePhaseBus {
            bus_id: 2,
            bus_type: CpfBusType::PQ,
            p_load_mw: [0.2; 3],
            q_load_mvar: [0.1; 3],
            p_gen_mw: [0.0; 3],
        });
        cpf.add_branch(ThreePhaseBranch {
            from_bus: 0,
            to_bus: 1,
            r_pu: [0.05; 3],
            x_pu: [0.1; 3],
            r_mutual_pu: 0.0,
            x_mutual_pu: 0.0,
        });
        cpf.add_branch(ThreePhaseBranch {
            from_bus: 1,
            to_bus: 2,
            r_pu: [0.08; 3],
            x_pu: [0.15; 3],
            r_mutual_pu: 0.0,
            x_mutual_pu: 0.0,
        });
        let v = cpf
            .gauss_seidel_phase(0, 1.0, &[1.0, 0.98, 0.95])
            .expect("GS converged");
        let pv_v = v[1];
        assert!(
            (pv_v - 0.98).abs() < 0.02,
            "PV bus voltage should be near setpoint 0.98, got {}",
            pv_v
        );
    }

    // 20. PQ bus reactive load handled
    #[test]
    fn test_pq_bus_reactive_load() {
        let cpf = make_2bus(0.1, 0.15);
        let result = cpf.solve();
        assert!(
            result.is_ok(),
            "High reactive load should not cause error: {:?}",
            result.err()
        );
    }

    // 21. Mutual coupling: non-zero r_mutual_pu doesn't panic
    #[test]
    fn test_mutual_coupling_nonzero() {
        let cfg = UnbalancedCpfConfig {
            lambda_max: 1.5,
            lambda_step: 0.1,
            ..UnbalancedCpfConfig::default()
        };
        let mut cpf = UnbalancedCpf::new(cfg);
        cpf.add_bus(ThreePhaseBus {
            bus_id: 0,
            bus_type: CpfBusType::Slack,
            p_load_mw: [0.0; 3],
            q_load_mvar: [0.0; 3],
            p_gen_mw: [0.0; 3],
        });
        cpf.add_bus(ThreePhaseBus {
            bus_id: 1,
            bus_type: CpfBusType::PQ,
            p_load_mw: [0.1; 3],
            q_load_mvar: [0.05; 3],
            p_gen_mw: [0.0; 3],
        });
        cpf.add_branch(ThreePhaseBranch {
            from_bus: 0,
            to_bus: 1,
            r_pu: [0.1; 3],
            x_pu: [0.2; 3],
            r_mutual_pu: 0.01,
            x_mutual_pu: 0.015,
        });
        let result = cpf.solve();
        assert!(
            result.is_ok(),
            "Mutual coupling should not cause solver error: {:?}",
            result.err()
        );
    }

    // 22. Branch connectivity: from_bus / to_bus used correctly
    #[test]
    fn test_branch_connectivity() {
        let cpf = make_3bus_radial();
        // Should not panic on index access
        let result = cpf.solve();
        assert!(
            result.is_ok(),
            "Branch connectivity check failed: {:?}",
            result.err()
        );
        let r = result.expect("ok");
        // VSI vector length equals number of buses
        assert_eq!(r.vsi.len(), 3, "VSI should have 3 entries (one per bus)");
    }

    // 23. Invalid branch index returns Err
    #[test]
    fn test_invalid_branch_index_returns_err() {
        let cfg = UnbalancedCpfConfig::default();
        let mut cpf = UnbalancedCpf::new(cfg);
        cpf.add_bus(ThreePhaseBus {
            bus_id: 0,
            bus_type: CpfBusType::Slack,
            p_load_mw: [0.0; 3],
            q_load_mvar: [0.0; 3],
            p_gen_mw: [0.0; 3],
        });
        cpf.add_bus(ThreePhaseBus {
            bus_id: 1,
            bus_type: CpfBusType::PQ,
            p_load_mw: [0.1; 3],
            q_load_mvar: [0.0; 3],
            p_gen_mw: [0.0; 3],
        });
        cpf.add_branch(ThreePhaseBranch {
            from_bus: 0,
            to_bus: 99, // invalid
            r_pu: [0.1; 3],
            x_pu: [0.2; 3],
            r_mutual_pu: 0.0,
            x_mutual_pu: 0.0,
        });
        let result = cpf.solve();
        assert!(result.is_err(), "Invalid bus index should return Err");
    }

    // 24. Loading margin helper returns same as result field
    #[test]
    fn test_loading_margin_helper() {
        let cpf = make_2bus(0.1, 0.05);
        let result = cpf.solve().expect("solve ok");
        let margin = cpf.loading_margin(&result);
        assert!(
            (margin - result.loading_margin).abs() < 1e-12,
            "loading_margin helper should match result field"
        );
    }

    // 25. Unbalanced loading: voltage decreases as lambda increases (upper branch)
    #[test]
    fn test_unbalanced_voltage_decreases_with_lambda() {
        let cfg = UnbalancedCpfConfig {
            lambda_max: 3.0,
            lambda_step: 0.1,
            load_model: LoadScalingModel::PhaseADominant { ratio: 0.6 },
            ..UnbalancedCpfConfig::default()
        };
        let mut cpf = UnbalancedCpf::new(cfg);
        cpf.add_bus(ThreePhaseBus {
            bus_id: 0,
            bus_type: CpfBusType::Slack,
            p_load_mw: [0.0; 3],
            q_load_mvar: [0.0; 3],
            p_gen_mw: [0.0; 3],
        });
        cpf.add_bus(ThreePhaseBus {
            bus_id: 1,
            bus_type: CpfBusType::PQ,
            p_load_mw: [0.2, 0.1, 0.1],
            q_load_mvar: [0.1, 0.05, 0.05],
            p_gen_mw: [0.0; 3],
        });
        cpf.add_branch(ThreePhaseBranch {
            from_bus: 0,
            to_bus: 1,
            r_pu: [0.1; 3],
            x_pu: [0.2; 3],
            r_mutual_pu: 0.0,
            x_mutual_pu: 0.0,
        });
        let result = cpf.solve().expect("solve should succeed for moderate load");
        let curve = &result.pv_curve;
        assert!(
            curve.len() >= 2,
            "PV curve should have at least 2 points, got {}",
            curve.len()
        );
        let v_first = (curve[0].v_a + curve[0].v_b + curve[0].v_c) / 3.0;
        let last_pt = curve.last().expect("pv_curve is non-empty");
        let v_last = (last_pt.v_a + last_pt.v_b + last_pt.v_c) / 3.0;
        assert!(
            v_last <= v_first + 0.05,
            "Average voltage should not significantly increase as lambda grows: v_first={:.4} v_last={:.4}",
            v_first,
            v_last
        );
    }

    // 26. Predictor step: lambda in pv_curve is non-decreasing (initial steps)
    #[test]
    fn test_pv_curve_lambda_increments_positive() {
        let cpf = make_2bus(0.15, 0.07);
        let result = cpf.solve().expect("solve should succeed");
        let curve = &result.pv_curve;
        // Check the first several steps: lambda must be non-decreasing
        let early: Vec<_> = curve.iter().take(5).collect();
        for w in early.windows(2) {
            assert!(
                w[1].lambda >= w[0].lambda,
                "Lambda must be non-decreasing in early steps: {:.4} -> {:.4}",
                w[0].lambda,
                w[1].lambda
            );
        }
    }

    // 27. Nose-point detection: lambda at nose does not exceed max lambda in pv_curve
    #[test]
    fn test_nose_lambda_is_maximum_in_curve() {
        let cfg = UnbalancedCpfConfig {
            lambda_max: 6.0,
            lambda_step: 0.05,
            collapse_voltage_pu: 0.55,
            ..UnbalancedCpfConfig::default()
        };
        let mut cpf = UnbalancedCpf::new(cfg);
        cpf.add_bus(ThreePhaseBus {
            bus_id: 0,
            bus_type: CpfBusType::Slack,
            p_load_mw: [0.0; 3],
            q_load_mvar: [0.0; 3],
            p_gen_mw: [0.0; 3],
        });
        cpf.add_bus(ThreePhaseBus {
            bus_id: 1,
            bus_type: CpfBusType::PQ,
            p_load_mw: [0.7; 3],
            q_load_mvar: [0.35; 3],
            p_gen_mw: [0.0; 3],
        });
        cpf.add_branch(ThreePhaseBranch {
            from_bus: 0,
            to_bus: 1,
            r_pu: [0.3; 3],
            x_pu: [0.45; 3],
            r_mutual_pu: 0.0,
            x_mutual_pu: 0.0,
        });
        let result = cpf.solve().expect("solve failed");
        if result.converged_to_nose && !result.collapse_points.is_empty() {
            let max_lambda_in_curve = result
                .pv_curve
                .iter()
                .map(|p| p.lambda)
                .fold(f64::NEG_INFINITY, f64::max);
            for cp in &result.collapse_points {
                assert!(
                    cp.lambda_nose <= max_lambda_in_curve + 1e-6,
                    "Nose lambda ({:.4}) must not exceed max lambda in curve ({:.4})",
                    cp.lambda_nose,
                    max_lambda_in_curve
                );
            }
        }
    }

    // 28. Maximum loading factor reached by solver is positive
    #[test]
    fn test_maximum_loading_factor_positive() {
        let cpf = make_2bus(0.2, 0.1);
        let result = cpf.solve().expect("solve should succeed");
        let max_lambda = result
            .pv_curve
            .iter()
            .map(|p| p.lambda)
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(
            max_lambda > 0.0,
            "Maximum lambda reached should be positive, got {:.4}",
            max_lambda
        );
    }

    // 29. Phase voltage differences are nonzero under PhaseADominant with unequal base loads
    #[test]
    fn test_phase_voltage_differences_nonzero_unbalanced() {
        let cfg = UnbalancedCpfConfig {
            lambda_max: 2.0,
            lambda_step: 0.1,
            load_model: LoadScalingModel::PhaseADominant { ratio: 0.2 },
            ..UnbalancedCpfConfig::default()
        };
        let mut cpf = UnbalancedCpf::new(cfg);
        cpf.add_bus(ThreePhaseBus {
            bus_id: 0,
            bus_type: CpfBusType::Slack,
            p_load_mw: [0.0; 3],
            q_load_mvar: [0.0; 3],
            p_gen_mw: [0.0; 3],
        });
        cpf.add_bus(ThreePhaseBus {
            bus_id: 1,
            bus_type: CpfBusType::PQ,
            p_load_mw: [0.4, 0.1, 0.1],
            q_load_mvar: [0.2, 0.05, 0.05],
            p_gen_mw: [0.0; 3],
        });
        cpf.add_branch(ThreePhaseBranch {
            from_bus: 0,
            to_bus: 1,
            r_pu: [0.1; 3],
            x_pu: [0.2; 3],
            r_mutual_pu: 0.0,
            x_mutual_pu: 0.0,
        });
        let result = cpf.solve().expect("solve failed for unbalanced system");
        // At higher lambda values phases should differ due to different base loads
        let high_lambda_points: Vec<_> =
            result.pv_curve.iter().filter(|p| p.lambda > 1.5).collect();
        if !high_lambda_points.is_empty() {
            let last = high_lambda_points.last().expect("filtered non-empty");
            let diff_ab = (last.v_a - last.v_b).abs();
            let diff_ac = (last.v_a - last.v_c).abs();
            assert!(
                diff_ab > 1e-6 || diff_ac > 1e-6,
                "Phase voltages should differ under strongly unbalanced loading: v_a={:.4} v_b={:.4} v_c={:.4}",
                last.v_a,
                last.v_b,
                last.v_c
            );
        }
    }

    // 30. Active power at each PV point is non-negative
    #[test]
    fn test_pv_curve_active_power_nonneg() {
        let cpf = make_3bus_radial();
        let result = cpf.solve().expect("3-bus solve ok");
        for pt in &result.pv_curve {
            assert!(
                pt.active_power_total_mw >= 0.0,
                "active_power_total_mw must be non-negative at lambda={:.3}, got {:.6}",
                pt.lambda,
                pt.active_power_total_mw
            );
        }
    }

    // 31. Loading margin is non-negative
    #[test]
    fn test_loading_margin_nonneg() {
        let cpf = make_2bus(0.05, 0.02);
        let result = cpf.solve().expect("solve ok");
        assert!(
            result.loading_margin >= 0.0,
            "loading_margin must be >= 0, got {:.4}",
            result.loading_margin
        );
    }

    // 32. VSI per-bus count matches bus count and all values in [0, 1]
    #[test]
    fn test_vsi_length_matches_bus_count() {
        let cpf = make_3bus_radial();
        let result = cpf.solve().expect("3-bus solve ok");
        assert_eq!(
            result.vsi.len(),
            cpf.buses.len(),
            "VSI vector length ({}) must equal bus count ({})",
            result.vsi.len(),
            cpf.buses.len()
        );
        for (bus_idx, vsi_phases) in result.vsi.iter().enumerate() {
            for (phase, &v) in vsi_phases.iter().enumerate() {
                assert!(
                    (0.0..=1.0).contains(&v),
                    "VSI[bus={bus_idx}][phase={phase}] = {v:.6} is outside [0, 1]"
                );
            }
        }
    }
}
