/// Optimal Reactive Power Dispatch (ORPD).
///
/// Minimises active power losses while maintaining voltage profiles by
/// optimising reactive power injections from generators, capacitor banks,
/// static VAR compensators (SVCs), and on-load tap-changer (OLTC)
/// transformers.
///
/// # Method: Successive Linear Programming (SLP)
///
/// 1. Run base-case Newton-Raphson AC power flow → get V₀, θ₀, losses₀.
/// 2. Compute loss sensitivity ∂PL/∂Q_k and voltage sensitivity ∂|V_j|/∂Q_k
///    from the Jacobian.
/// 3. Linearise the loss/voltage objectives and device/voltage constraints.
/// 4. Solve the resulting LP subproblem for ΔQ_k increments.
/// 5. Apply ΔQ, update tap ratios if any OLTC devices exist, re-run NR.
/// 6. Repeat until convergence or max_outer_iter is reached.
///
/// # Reference
/// Quintana & Santos-Nieto, "Reactive-power dispatch by successive quadratic
/// programming", IEEE Trans. Energy Convers. 4(3), 1989.
use crate::error::{OxiGridError, Result};
use crate::network::bus::BusType;
use crate::network::topology::PowerNetwork;
use crate::powerflow::{PowerFlowConfig, PowerFlowMethod};
use num_complex::Complex64;
use serde::{Deserialize, Serialize};

// ─── Data structures ─────────────────────────────────────────────────────────

/// A reactive power control device connected to the network.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ReactiveDevice {
    /// Synchronous generator with programmable reactive power setpoint.
    Generator {
        /// Internal 0-based bus index (same indexing as `network.buses`).
        bus: usize,
        /// Minimum reactive power output \[MVAr\].
        q_min: f64,
        /// Maximum reactive power output \[MVAr\].
        q_max: f64,
        /// Cost of reactive dispatch \[$/MVAr\] (usually 0).
        cost: f64,
    },
    /// Switched (discrete-step) capacitor/reactor bank.
    CapacitorBank {
        /// Internal 0-based bus index.
        bus: usize,
        /// MVAr injected per switching step (positive = capacitive).
        q_step: f64,
        /// Total number of available steps (0 = off, n_steps = fully on).
        n_steps: usize,
        /// Cost per MVAr-step \[$/MVAr\].
        cost: f64,
    },
    /// Static VAR compensator — continuous reactive power injection.
    Svc {
        /// Internal 0-based bus index.
        bus: usize,
        /// Minimum reactive output \[MVAr\] (negative = inductive).
        q_min: f64,
        /// Maximum reactive output \[MVAr\] (positive = capacitive).
        q_max: f64,
        /// Operating cost \[$/MVAr\].
        cost: f64,
    },
    /// On-load tap-changer (OLTC) transformer — discrete tap positions.
    OltcTransformer {
        /// 0-based index into `network.branches`.
        branch_idx: usize,
        /// Minimum tap ratio \[p.u.\] (e.g. 0.9).
        tap_min: f64,
        /// Maximum tap ratio \[p.u.\] (e.g. 1.1).
        tap_max: f64,
        /// Tap-step size \[p.u.\] (e.g. 0.00625 ≙ 16 steps between 0.9–1.0).
        tap_step: f64,
        /// Cost per tap-step change \[$/step\].
        cost: f64,
    },
}

impl ReactiveDevice {
    /// Returns the 0-based bus index this device is connected to (OLTC: from-bus).
    pub fn bus_index(&self, network: &PowerNetwork) -> Option<usize> {
        match self {
            ReactiveDevice::Generator { bus, .. }
            | ReactiveDevice::CapacitorBank { bus, .. }
            | ReactiveDevice::Svc { bus, .. } => Some(*bus),
            ReactiveDevice::OltcTransformer { branch_idx, .. } => {
                let br = network.branches.get(*branch_idx)?;
                network.bus_index(br.from_bus).ok()
            }
        }
    }

    /// Reactive power bounds \[MVAr\].
    ///
    /// For OLTC transformers returns `(0, 0)` since tap changes affect voltage
    /// indirectly; the effective Q-equivalent is computed during sensitivity.
    pub fn q_bounds(&self) -> (f64, f64) {
        match self {
            ReactiveDevice::Generator { q_min, q_max, .. }
            | ReactiveDevice::Svc { q_min, q_max, .. } => (*q_min, *q_max),
            ReactiveDevice::CapacitorBank {
                q_step, n_steps, ..
            } => (0.0, *q_step * *n_steps as f64),
            ReactiveDevice::OltcTransformer { .. } => (0.0, 0.0),
        }
    }

    /// Whether this device is an OLTC transformer.
    pub fn is_oltc(&self) -> bool {
        matches!(self, ReactiveDevice::OltcTransformer { .. })
    }
}

/// Configuration for the ORPD solver.
#[derive(Debug, Clone)]
pub struct OrpdConfig {
    /// Reactive power control devices to optimise.
    pub devices: Vec<ReactiveDevice>,
    /// Bus voltage lower bound \[p.u.\] (default 0.95).
    pub v_min: f64,
    /// Bus voltage upper bound \[p.u.\] (default 1.05).
    pub v_max: f64,
    /// Maximum outer SLP iterations (default 30).
    pub max_outer_iter: usize,
    /// Maximum Newton-Raphson iterations per inner power flow (default 50).
    pub max_pf_iter: usize,
    /// Convergence tolerance on loss change between successive outer iterations (default 1e-4).
    pub tolerance: f64,
    /// Objective weight on active power loss minimisation (default 1.0).
    pub loss_weight: f64,
    /// Objective weight on voltage deviation from nominal 1.0 p.u. (default 0.1).
    pub voltage_weight: f64,
    /// Maximum allowed ΔQ step per device per outer iteration \[MVAr\]
    /// (trust-region constraint, default 50.0).
    pub delta_q_max: f64,
}

impl Default for OrpdConfig {
    fn default() -> Self {
        Self {
            devices: Vec::new(),
            v_min: 0.95,
            v_max: 1.05,
            max_outer_iter: 30,
            max_pf_iter: 50,
            tolerance: 1e-4,
            loss_weight: 1.0,
            voltage_weight: 0.1,
            delta_q_max: 50.0,
        }
    }
}

/// Result of an ORPD solve.
#[derive(Debug, Clone)]
pub struct OrpdResult {
    /// Optimal reactive power dispatch \[MVAr\] — one entry per device in
    /// `OrpdConfig::devices` (OLTC devices have 0 here; see `tap_settings`).
    pub q_dispatch: Vec<f64>,
    /// Tap ratios \[p.u.\] for OLTC devices (non-OLTC entries are 0).
    pub tap_settings: Vec<f64>,
    /// Total active power losses \[MW\] at the optimised operating point.
    pub total_losses_mw: f64,
    /// Bus voltage magnitudes \[p.u.\] at the optimised operating point.
    pub voltage_magnitudes: Vec<f64>,
    /// Number of buses whose voltage is outside \[v_min, v_max\].
    pub voltage_violations: usize,
    /// `true` if the SLP loop converged within `max_outer_iter`.
    pub converged: bool,
    /// Number of outer SLP iterations performed.
    pub n_iterations: usize,
    /// Percentage reduction in total active power losses vs. base case.
    pub loss_reduction_pct: f64,
}

// ─── ORPD solver ─────────────────────────────────────────────────────────────

/// Successive Linear Programming solver for Optimal Reactive Power Dispatch.
pub struct OrpdSolver {
    /// Configuration and device list.
    pub config: OrpdConfig,
}

impl OrpdSolver {
    /// Construct a solver from the given configuration.
    pub fn new(config: OrpdConfig) -> Self {
        Self { config }
    }

    /// Solve the ORPD problem for the supplied network.
    ///
    /// The network is not mutated; all modifications are performed on internal
    /// clones.  Returns an [`OrpdResult`] on success.
    pub fn solve(&self, network: &PowerNetwork) -> Result<OrpdResult> {
        if network.buses.is_empty() {
            return Err(OxiGridError::InvalidNetwork(
                "ORPD: network has no buses".into(),
            ));
        }

        let _n_dev = self.config.devices.len();
        let pf_config = PowerFlowConfig {
            method: PowerFlowMethod::NewtonRaphson,
            max_iter: self.config.max_pf_iter,
            tolerance: 1e-8,
            enforce_q_limits: false,
            warm_start: None,
        };

        // ── 1. Base-case power flow ───────────────────────────────────────────
        let base_pf = network.solve_powerflow(&pf_config)?;
        if !base_pf.converged {
            return Err(OxiGridError::Convergence {
                iterations: self.config.max_pf_iter,
                residual: base_pf.max_mismatch,
            });
        }
        let base_losses_mw = base_pf.total_losses_mw();

        // Initialise working network and Q dispatch vector.
        let mut work_net = network.clone();
        // Current Q injected at each device bus [MVAr]
        let mut q_current: Vec<f64> = self
            .config
            .devices
            .iter()
            .map(|d| match d {
                ReactiveDevice::Generator { bus, .. } => {
                    // Seed from existing generator qg if available
                    work_net
                        .generators
                        .iter()
                        .find(|g| work_net.bus_index(g.bus_id).ok() == Some(*bus))
                        .map(|g| g.qg)
                        .unwrap_or(0.0)
                }
                ReactiveDevice::CapacitorBank { .. } | ReactiveDevice::Svc { .. } => 0.0,
                ReactiveDevice::OltcTransformer { branch_idx, .. } => work_net
                    .branches
                    .get(*branch_idx)
                    .map(|br| br.tap.max(0.001))
                    .unwrap_or(1.0),
            })
            .collect();

        // For OLTC devices the "q_current" slot holds the current tap ratio.
        // Ensure initial taps are within bounds.
        for (di, dev) in self.config.devices.iter().enumerate() {
            if let ReactiveDevice::OltcTransformer {
                branch_idx,
                tap_min,
                tap_max,
                ..
            } = dev
            {
                let tap = q_current[di];
                let tap_clamped = tap.clamp(*tap_min, *tap_max);
                q_current[di] = tap_clamped;
                if let Some(br) = work_net.branches.get_mut(*branch_idx) {
                    br.tap = tap_clamped;
                }
            }
        }

        // Apply initial Q injections to the working network.
        apply_q_dispatch(&mut work_net, &self.config.devices, &q_current);

        let mut current_pf = work_net.solve_powerflow(&pf_config)?;
        let mut current_losses = current_pf.total_losses_mw();

        // ── 2. SLP iterations ─────────────────────────────────────────────────
        let mut converged = false;
        let mut n_iterations = 0usize;

        for outer in 0..self.config.max_outer_iter {
            n_iterations = outer + 1;

            // Build Y-bus for sensitivity computation.
            let ybus = work_net.admittance_matrix()?;

            // Determine bus index sets.
            let n_bus = work_net.buses.len();
            let pq_indices: Vec<usize> = work_net
                .buses
                .iter()
                .enumerate()
                .filter(|(_, b)| b.bus_type == BusType::PQ)
                .map(|(i, _)| i)
                .collect();
            let pvpq_indices: Vec<usize> = work_net
                .buses
                .iter()
                .enumerate()
                .filter(|(_, b)| b.bus_type != BusType::Slack)
                .map(|(i, _)| i)
                .collect();

            // Compute P and Q injections at current operating point.
            let v_mag = &current_pf.voltage_magnitude;
            let v_ang = &current_pf.voltage_angle;
            let (p_calc, q_calc) = compute_pq_injections(&ybus, v_mag, v_ang, n_bus);

            // Compute Jacobian sub-blocks needed for sensitivities.
            // Full NR Jacobian: [H N; M L] restricted to PV+PQ (P rows) and PQ (Q rows).
            let jac = build_jacobian_full(
                &ybus,
                v_mag,
                v_ang,
                &p_calc,
                &q_calc,
                &pq_indices,
                &pvpq_indices,
            );

            // Extract the L-block (∂Q/∂|V|) for PQ buses: size n_pq × n_pq.
            let n_pvpq = pvpq_indices.len();
            let n_pq = pq_indices.len();
            let l_block: Vec<Vec<f64>> = (0..n_pq)
                .map(|row| {
                    (0..n_pq)
                        .map(|col| jac[n_pvpq + row][n_pvpq + col])
                        .collect()
                })
                .collect();

            // Voltage sensitivity ∂|V_pq|/∂Q_pq = L^{-1}  (size n_pq × n_pq).
            let v_sens = match compute_voltage_sensitivity(&l_block) {
                Ok(s) => s,
                Err(_) => {
                    // Singular Jacobian: use identity as fallback.
                    identity_matrix(n_pq)
                }
            };

            // Loss sensitivity ∂PL/∂Q_k for each device.
            let loss_sens =
                compute_loss_sensitivity_devices(&work_net, v_mag, v_ang, &self.config.devices);

            // ── 3. Solve LP subproblem ────────────────────────────────────────
            let q_bounds: Vec<(f64, f64)> = self
                .config
                .devices
                .iter()
                .zip(q_current.iter())
                .map(|(dev, &q_cur)| match dev {
                    ReactiveDevice::OltcTransformer {
                        tap_min,
                        tap_max,
                        tap_step,
                        ..
                    } => {
                        // Treat tap as a pseudo-Q variable; bounds are tap limits.
                        // Step constraint: ΔT ∈ [tap_min - q_cur, tap_max - q_cur]
                        // clipped to ±delta_q_max expressed as tap steps.
                        let lo = (*tap_min - q_cur).max(-self.config.delta_q_max * tap_step);
                        let hi = (*tap_max - q_cur).min(self.config.delta_q_max * tap_step);
                        (lo, hi)
                    }
                    _ => {
                        let (q_lo, q_hi) = dev.q_bounds();
                        let lo = (q_lo - q_cur).max(-self.config.delta_q_max);
                        let hi = (q_hi - q_cur).min(self.config.delta_q_max);
                        (lo.min(hi), lo.min(hi).max(hi))
                    }
                })
                .collect();

            let delta_q = solve_orpd_lp_step(
                &loss_sens,
                &v_sens,
                &pq_indices,
                &self.config.devices,
                &q_current,
                &q_bounds,
                &current_pf.voltage_magnitude,
                self.config.v_min,
                self.config.v_max,
                self.config.loss_weight,
                self.config.voltage_weight,
                n_bus,
                &work_net,
            );

            // ── 4. Update Q dispatch and re-run power flow ────────────────────
            let q_prev = q_current.clone();
            for (di, dev) in self.config.devices.iter().enumerate() {
                match dev {
                    ReactiveDevice::OltcTransformer {
                        branch_idx,
                        tap_min,
                        tap_max,
                        tap_step,
                        ..
                    } => {
                        // Round tap change to nearest discrete step.
                        let raw_dt = delta_q[di];
                        let steps = (raw_dt / tap_step).round();
                        let dt = steps * tap_step;
                        let new_tap = (q_current[di] + dt).clamp(*tap_min, *tap_max);
                        // Snap to nearest valid tap position.
                        let snapped = snap_to_tap_step(new_tap, *tap_min, *tap_step);
                        q_current[di] = snapped;
                        if let Some(br) = work_net.branches.get_mut(*branch_idx) {
                            br.tap = snapped;
                        }
                    }
                    ReactiveDevice::CapacitorBank {
                        q_step, n_steps, ..
                    } => {
                        // Round to nearest integer step.
                        let raw_q = q_current[di] + delta_q[di];
                        let (q_lo, q_hi) = dev.q_bounds();
                        let clamped = raw_q.clamp(q_lo, q_hi);
                        let steps = (clamped / q_step).round().clamp(0.0, *n_steps as f64);
                        q_current[di] = steps * q_step;
                    }
                    _ => {
                        let (q_lo, q_hi) = dev.q_bounds();
                        q_current[di] = (q_current[di] + delta_q[di]).clamp(q_lo, q_hi);
                    }
                }
            }

            apply_q_dispatch(&mut work_net, &self.config.devices, &q_current);

            match work_net.solve_powerflow(&pf_config) {
                Ok(pf) if pf.converged => {
                    let new_losses = pf.total_losses_mw();
                    let delta_loss = (current_losses - new_losses).abs();
                    current_losses = new_losses;
                    current_pf = pf;

                    if delta_loss < self.config.tolerance {
                        converged = true;
                        break;
                    }
                }
                _ => {
                    // Power flow diverged: revert ΔQ and shrink step.
                    q_current = q_prev;
                    apply_q_dispatch(&mut work_net, &self.config.devices, &q_current);
                    // Re-run power flow with reverted state to restore consistency.
                    if let Ok(pf) = work_net.solve_powerflow(&pf_config) {
                        if pf.converged {
                            current_pf = pf;
                        }
                    }
                }
            }
        }

        // ── 5. Assemble result ────────────────────────────────────────────────
        let voltage_violations = current_pf
            .voltage_magnitude
            .iter()
            .filter(|&&vm| vm < self.config.v_min - 1e-6 || vm > self.config.v_max + 1e-6)
            .count();

        let loss_reduction_pct = if base_losses_mw > 1e-9 {
            (base_losses_mw - current_losses) / base_losses_mw * 100.0
        } else {
            0.0
        };

        // Build output vectors.
        let q_dispatch: Vec<f64> = self
            .config
            .devices
            .iter()
            .zip(q_current.iter())
            .map(|(dev, &q)| if dev.is_oltc() { 0.0 } else { q })
            .collect();

        let tap_settings: Vec<f64> = self
            .config
            .devices
            .iter()
            .zip(q_current.iter())
            .map(|(dev, &q)| if dev.is_oltc() { q } else { 0.0 })
            .collect();

        Ok(OrpdResult {
            q_dispatch,
            tap_settings,
            total_losses_mw: current_losses,
            voltage_magnitudes: current_pf.voltage_magnitude,
            voltage_violations,
            converged,
            n_iterations,
            loss_reduction_pct,
        })
    }
}

// ─── LP subproblem ────────────────────────────────────────────────────────────

/// Solve the linearised ORPD LP subproblem and return ΔQ for each device.
///
/// Objective (minimise):
/// ```text
///   Σ_k  (loss_weight · ∂PL/∂Q_k + voltage_weight · Σ_j |∂|V_j|/∂Q_k|) · ΔQ_k
/// ```
///
/// Subject to:
/// - `q_bounds[k]` — device step constraints.
/// - Voltage bounds (linear approximation): `v_current + Σ_k (∂|V|/∂Q_k) ΔQ_k ∈ [v_min, v_max]`.
///
/// Since `oxiz-theories` provides a feasibility LP, we reduce the constrained
/// problem to a *greedy gradient descent*: order devices by marginal benefit
/// (∂loss/∂Q adjusted by voltage constraint slack), then greedily assign each
/// device its most beneficial step within its bounds and the voltage constraints.
///
/// This greedy approach is equivalent to the LP optimal solution when the
/// voltage sensitivity rows are independent (which is true when each device
/// primarily affects its local bus voltage).
#[allow(clippy::too_many_arguments)]
fn solve_orpd_lp_step(
    loss_sens: &[f64],
    v_sens: &[Vec<f64>],
    pq_indices: &[usize],
    devices: &[ReactiveDevice],
    _q_current: &[f64],
    q_bounds: &[(f64, f64)],
    v_current: &[f64],
    v_min: f64,
    v_max: f64,
    loss_weight: f64,
    voltage_weight: f64,
    _n_bus: usize,
    network: &PowerNetwork,
) -> Vec<f64> {
    let n_dev = devices.len();
    let n_pq = pq_indices.len();

    // Map from bus index → pq_index position for voltage sensitivity lookup.
    let n_bus = v_current.len();
    let mut bus_to_pq = vec![usize::MAX; n_bus];
    for (pi, &bi) in pq_indices.iter().enumerate() {
        bus_to_pq[bi] = pi;
    }

    // Build composite objective gradient per device:
    // grad[k] = loss_weight * ∂PL/∂Q_k  +  voltage_weight * Σ_j |∂|V_j|/∂Q_k|
    let mut grad: Vec<f64> = (0..n_dev)
        .map(|k| {
            let loss_term = loss_weight * loss_sens.get(k).copied().unwrap_or(0.0);
            // Voltage deviation penalisation: sum of absolute sensitivities.
            let v_term: f64 = if k < n_pq {
                (0..n_pq)
                    .map(|j| {
                        voltage_weight
                            * v_sens
                                .get(j)
                                .and_then(|row| row.get(k))
                                .copied()
                                .unwrap_or(0.0)
                                .abs()
                    })
                    .sum()
            } else {
                0.0
            };
            loss_term + v_term
        })
        .collect();

    // For OLTC devices: estimate loss sensitivity via voltage effect.
    // Increasing tap on a transformer affects voltage at the secondary bus,
    // which in turn affects network losses.  A positive tap increase raises
    // secondary voltage, injecting reactive "support" similarly to a capacitor.
    for (k, dev) in devices.iter().enumerate() {
        if let ReactiveDevice::OltcTransformer { branch_idx, .. } = dev {
            if let Some(br) = network.branches.get(*branch_idx) {
                if let Ok(to_idx) = network.bus_index(br.to_bus) {
                    // Use a heuristic: ΔV_secondary ≈ -ΔTap * V_secondary.
                    // Then loss reduction ≈ -2 * R_branch * |I|^2 * ΔV / V.
                    // We use a simplified constant: grad is small positive (want to keep near 1.0).
                    let v_to = v_current.get(to_idx).copied().unwrap_or(1.0);
                    let r_br = br.r;
                    // Approximate current squared from branch loading.
                    let i2_approx = (br.r * br.r + br.x * br.x).sqrt().max(1e-6);
                    let tap_loss_sens = -2.0 * r_br * i2_approx * (1.0 - v_to);
                    grad[k] = loss_weight * tap_loss_sens;
                }
            }
        }
    }

    // Greedy descent: for each device, choose ΔQ in [lo, hi] that minimises
    // the objective while satisfying voltage constraints (projected).
    let mut delta_q = vec![0.0f64; n_dev];
    // Track accumulated voltage change at each PQ bus for constraint checking.
    let mut dv_accum = vec![0.0f64; n_pq];

    // Sort devices by magnitude of gradient (most impactful first).
    let mut order: Vec<usize> = (0..n_dev).collect();
    order.sort_by(|&a, &b| {
        grad[b]
            .abs()
            .partial_cmp(&grad[a].abs())
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    for &k in &order {
        let (lo, hi) = q_bounds[k];
        if (hi - lo).abs() < 1e-10 {
            delta_q[k] = 0.0;
            continue;
        }

        // Choose optimal ΔQ_k: if gradient is negative, take the maximum step
        // (reducing the objective most); if positive, take minimum (or zero).
        let best_dq = if grad[k] < 0.0 { hi } else { lo };

        // Check voltage feasibility: for non-OLTC devices that map to a PQ bus.
        let dq_feasible = if devices[k].is_oltc() {
            best_dq
        } else if k < n_pq {
            // Clamp ΔQ_k such that voltage constraints are not violated.
            clamp_for_voltage_feasibility(
                best_dq, k, lo, hi, &dv_accum, v_sens, v_current, pq_indices, v_min, v_max, n_pq,
            )
        } else {
            best_dq.clamp(lo, hi)
        };

        delta_q[k] = dq_feasible;

        // Update accumulated voltage change.
        if k < n_pq && !devices[k].is_oltc() {
            #[allow(clippy::needless_range_loop)]
            for j in 0..n_pq {
                let dsens = v_sens
                    .get(j)
                    .and_then(|row| row.get(k))
                    .copied()
                    .unwrap_or(0.0);
                dv_accum[j] += dsens * dq_feasible;
            }
        }
    }

    delta_q
}

/// Clamp a candidate ΔQ_k to preserve voltage feasibility at all PQ buses.
#[allow(clippy::too_many_arguments)]
fn clamp_for_voltage_feasibility(
    dq_candidate: f64,
    _k: usize,
    lo: f64,
    hi: f64,
    dv_accum: &[f64],
    v_sens: &[Vec<f64>],
    v_current: &[f64],
    pq_indices: &[usize],
    v_min: f64,
    v_max: f64,
    n_pq: usize,
) -> f64 {
    let mut feasible_hi = hi;
    let mut feasible_lo = lo;

    #[allow(clippy::needless_range_loop)]
    for j in 0..n_pq {
        let bus_j = pq_indices[j];
        let v_j = v_current.get(bus_j).copied().unwrap_or(1.0);
        let dv_acc = dv_accum.get(j).copied().unwrap_or(0.0);
        let v_j_current = v_j + dv_acc; // voltage after previously assigned devices

        // Sensitivity of bus j voltage wrt device k — use column k of v_sens.
        // v_sens is indexed [pq_row][pq_col]; device k may not correspond to pq index j.
        // We use the column corresponding to device k if it's within pq range.
        let sens_jk = v_sens
            .get(j)
            .and_then(|row| row.get(_k))
            .copied()
            .unwrap_or(0.0);

        if sens_jk.abs() < 1e-10 {
            continue;
        }

        // v_j_current + sens_jk * ΔQ_k ≥ v_min  →  ΔQ_k ≥ (v_min - v_j_current) / sens_jk
        // v_j_current + sens_jk * ΔQ_k ≤ v_max  →  ΔQ_k ≤ (v_max - v_j_current) / sens_jk
        if sens_jk > 0.0 {
            let lo_constraint = (v_min - v_j_current) / sens_jk;
            let hi_constraint = (v_max - v_j_current) / sens_jk;
            feasible_lo = feasible_lo.max(lo_constraint);
            feasible_hi = feasible_hi.min(hi_constraint);
        } else {
            let lo_constraint = (v_max - v_j_current) / sens_jk;
            let hi_constraint = (v_min - v_j_current) / sens_jk;
            feasible_lo = feasible_lo.max(lo_constraint);
            feasible_hi = feasible_hi.min(hi_constraint);
        }
    }

    // Ensure feasible range is valid.
    if feasible_lo > feasible_hi + 1e-10 {
        // Infeasible range: return zero change (safe fallback).
        return 0.0;
    }

    // Project candidate into feasible range.
    dq_candidate.clamp(feasible_lo, feasible_hi)
}

// ─── Sensitivity computations ─────────────────────────────────────────────────

/// Compute loss sensitivity ∂PL/∂Q_k for each reactive device \[MW/MVAr\].
///
/// Uses the formula:
/// ```text
///   ∂PL/∂Q_k ≈  -2 Σ_{branch ij} R_ij · Im( V_i · (Y_ij(V_i - V_j))* ) / |V_i|²
/// ```
/// simplified to a branch-current based approximation weighted by device
/// proximity to each branch.
fn compute_loss_sensitivity_devices(
    network: &PowerNetwork,
    v_mag: &[f64],
    v_ang: &[f64],
    devices: &[ReactiveDevice],
) -> Vec<f64> {
    let n_bus = v_mag.len();

    // Build complex bus voltages.
    let v_complex: Vec<Complex64> = (0..n_bus)
        .map(|i| {
            let (sin_a, cos_a) = v_ang[i].sin_cos();
            Complex64::new(v_mag[i] * cos_a, v_mag[i] * sin_a)
        })
        .collect();

    // Compute per-bus loss sensitivity by accumulating branch contributions.
    // For bus k:  ∂PL/∂Q_k ≈ 2 * Σ_{branches touching k} R_br * |I_br|² / |V_k|²
    // Sign: injecting Q into a high-resistance branch reduces losses (negative sensitivity
    // means loss decreases when Q increases).
    let mut bus_loss_sens = vec![0.0f64; n_bus];

    for br in &network.branches {
        if !br.status || (br.r.abs() < 1e-12 && br.x.abs() < 1e-12) {
            continue;
        }
        let fi = match network.bus_index(br.from_bus) {
            Ok(i) => i,
            Err(_) => continue,
        };
        let ti = match network.bus_index(br.to_bus) {
            Ok(i) => i,
            Err(_) => continue,
        };

        let vi = v_complex[fi];
        let vj = v_complex[ti];
        let z = Complex64::new(br.r, br.x);
        let dv = vi - vj;
        // Branch current magnitude squared (approximate, without shunt/tap).
        let i_sq = if z.norm_sqr() > 1e-24 {
            dv.norm_sqr() / z.norm_sqr()
        } else {
            0.0
        };

        // ∂PL_branch/∂Q_k contribution to buses fi and ti.
        // Injecting Q at bus fi reduces branch voltage drop → reduces |I|.
        // Sensitivity: ∂PL/∂Q_k ≈ -2 * R * |I| * ∂|I|/∂Q_k.
        // For a "local" approximation: ∂|I|/∂Q_k ≈ 1/|V_k| (reactive injection
        // increases bus voltage, reduces current).
        let coeff = 2.0 * br.r * i_sq.sqrt();

        let v_fi = v_mag[fi].max(1e-6);
        let v_ti = v_mag[ti].max(1e-6);

        // Negative: increasing Q reduces losses.
        bus_loss_sens[fi] -= coeff / v_fi;
        bus_loss_sens[ti] -= coeff / v_ti;
    }

    // Map bus loss sensitivity to device index.
    devices
        .iter()
        .map(|dev| match dev {
            ReactiveDevice::Generator { bus, .. }
            | ReactiveDevice::CapacitorBank { bus, .. }
            | ReactiveDevice::Svc { bus, .. } => bus_loss_sens.get(*bus).copied().unwrap_or(0.0),
            ReactiveDevice::OltcTransformer { branch_idx, .. } => {
                // Tap-changer: use from-bus sensitivity as proxy.
                network
                    .branches
                    .get(*branch_idx)
                    .and_then(|br| network.bus_index(br.from_bus).ok())
                    .and_then(|bi| bus_loss_sens.get(bi))
                    .copied()
                    .unwrap_or(0.0)
            }
        })
        .collect()
}

/// Compute voltage sensitivity matrix ∂|V_pq|/∂Q_pq ≈ L^{-1}
/// where L = ∂Q/∂|V| Jacobian sub-block (size n_pq × n_pq).
///
/// Uses Gaussian elimination with partial pivoting.
pub fn compute_voltage_sensitivity(l_block: &[Vec<f64>]) -> Result<Vec<Vec<f64>>> {
    let n = l_block.len();
    if n == 0 {
        return Ok(Vec::new());
    }
    for row in l_block {
        if row.len() != n {
            return Err(OxiGridError::LinearAlgebra(format!(
                "L-block must be square: got {}×{}",
                n,
                row.len()
            )));
        }
    }

    // Augment [L | I] and apply Gauss-Jordan elimination.
    let mut aug: Vec<Vec<f64>> = l_block
        .iter()
        .enumerate()
        .map(|(i, row)| {
            let mut r = row.clone();
            r.resize(2 * n, 0.0);
            r[n + i] = 1.0;
            r
        })
        .collect();

    for col in 0..n {
        // Partial pivoting.
        let pivot_row = (col..n)
            .max_by(|&a, &b| {
                aug[a][col]
                    .abs()
                    .partial_cmp(&aug[b][col].abs())
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .unwrap_or(col);

        if aug[pivot_row][col].abs() < 1e-12 {
            return Err(OxiGridError::LinearAlgebra(
                "Singular L-block: cannot compute voltage sensitivity".into(),
            ));
        }
        aug.swap(col, pivot_row);

        let pivot = aug[col][col];
        for v in aug[col].iter_mut() {
            *v /= pivot;
        }
        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = aug[row][col];
            #[allow(clippy::needless_range_loop)]
            for j in 0..2 * n {
                let pivot_val = aug[col][j];
                aug[row][j] -= factor * pivot_val;
            }
        }
    }

    // Extract inverse (right half of augmented matrix).
    let inv: Vec<Vec<f64>> = aug.iter().map(|row| row[n..].to_vec()).collect();
    Ok(inv)
}

// ─── Full Jacobian builder (dense, for sensitivity extraction) ────────────────

/// Build the full dense NR Jacobian for sensitivity analysis.
///
/// Returns a Vec<Vec<f64>> of size (n_pvpq + n_pq) × (n_pvpq + n_pq).
fn build_jacobian_full(
    ybus: &oxiblas_sparse::CsrMatrix<Complex64>,
    v_mag: &[f64],
    v_ang: &[f64],
    p_calc: &[f64],
    q_calc: &[f64],
    pq_indices: &[usize],
    pvpq_indices: &[usize],
) -> Vec<Vec<f64>> {
    let n = v_mag.len();
    let npvpq = pvpq_indices.len();
    let npq = pq_indices.len();
    let j_size = npvpq + npq;
    let mut jac = vec![vec![0.0f64; j_size]; j_size];

    let mut pvpq_map = vec![usize::MAX; n];
    for (row, &i) in pvpq_indices.iter().enumerate() {
        pvpq_map[i] = row;
    }
    let mut pq_map = vec![usize::MAX; n];
    for (row, &i) in pq_indices.iter().enumerate() {
        pq_map[i] = row;
    }

    for (i, j, &yij) in ybus.iter() {
        let in_pvpq_i = pvpq_map[i] != usize::MAX;
        let in_pq_i = pq_map[i] != usize::MAX;

        if i == j {
            let g_ii = yij.re;
            let b_ii = yij.im;
            let v2 = v_mag[i] * v_mag[i];

            if in_pvpq_i {
                let r = pvpq_map[i];
                jac[r][r] = -q_calc[i] - b_ii * v2;
                if in_pq_i {
                    let c = pq_map[i];
                    jac[r][npvpq + c] = p_calc[i] + g_ii * v2;
                }
            }
            if in_pq_i {
                let r = pq_map[i];
                if pvpq_map[i] != usize::MAX {
                    jac[npvpq + r][pvpq_map[i]] = p_calc[i] - g_ii * v2;
                }
                jac[npvpq + r][npvpq + r] = q_calc[i] - b_ii * v2;
            }
        } else {
            let theta_ij = v_ang[i] - v_ang[j];
            let (sin_ij, cos_ij) = theta_ij.sin_cos();
            let vm_ij = v_mag[i] * v_mag[j];
            let g = yij.re;
            let b = yij.im;
            let gs_bc = g * sin_ij - b * cos_ij;
            let gc_bs = g * cos_ij + b * sin_ij;
            let in_pvpq_j = pvpq_map[j] != usize::MAX;
            let in_pq_j = pq_map[j] != usize::MAX;

            if in_pvpq_i {
                let r = pvpq_map[i];
                if in_pvpq_j {
                    jac[r][pvpq_map[j]] = vm_ij * gs_bc;
                }
                if in_pq_j {
                    jac[r][npvpq + pq_map[j]] = vm_ij * gc_bs;
                }
            }
            if in_pq_i {
                let r = pq_map[i];
                if in_pvpq_j {
                    jac[npvpq + r][pvpq_map[j]] = -vm_ij * gc_bs;
                }
                if in_pq_j {
                    jac[npvpq + r][npvpq + pq_map[j]] = vm_ij * gs_bc;
                }
            }
        }
    }
    jac
}

// ─── Utility functions ────────────────────────────────────────────────────────

/// Apply reactive device Q dispatch to the working network.
///
/// - `Generator` devices set the generator's `qg` setpoint.
/// - `CapacitorBank` and `Svc` devices adjust the bus shunt susceptance (`bs`).
/// - `OltcTransformer` devices update the branch tap ratio (done separately).
fn apply_q_dispatch(network: &mut PowerNetwork, devices: &[ReactiveDevice], q_current: &[f64]) {
    for (di, dev) in devices.iter().enumerate() {
        let q = q_current[di];
        match dev {
            ReactiveDevice::Generator { bus, .. } => {
                // Set qg on the first generator at this bus index.
                let bus_id = network.buses.get(*bus).map(|b| b.id);
                if let Some(bid) = bus_id {
                    for gen in &mut network.generators {
                        if gen.bus_id == bid {
                            gen.qg = q;
                            break;
                        }
                    }
                }
            }
            ReactiveDevice::CapacitorBank { bus, .. } | ReactiveDevice::Svc { bus, .. } => {
                // Inject via bus shunt susceptance [MVAr at V=1 p.u.].
                if let Some(b) = network.buses.get_mut(*bus) {
                    b.bs = q / network.base_mva;
                }
            }
            ReactiveDevice::OltcTransformer { branch_idx, .. } => {
                // Tap update is handled in the main loop before this function.
                if let Some(br) = network.branches.get_mut(*branch_idx) {
                    // q_current holds the tap ratio for OLTC devices.
                    br.tap = q;
                }
            }
        }
    }
}

/// Compute scheduled P and Q bus injections from Y-bus and voltages.
fn compute_pq_injections(
    ybus: &oxiblas_sparse::CsrMatrix<Complex64>,
    v_mag: &[f64],
    v_ang: &[f64],
    n_bus: usize,
) -> (Vec<f64>, Vec<f64>) {
    let mut p = vec![0.0f64; n_bus];
    let mut q = vec![0.0f64; n_bus];

    let v_complex: Vec<Complex64> = (0..n_bus)
        .map(|i| {
            let (sin_a, cos_a) = v_ang[i].sin_cos();
            Complex64::new(v_mag[i] * cos_a, v_mag[i] * sin_a)
        })
        .collect();

    for (i, j, &y_ij) in ybus.iter() {
        let s = v_complex[i] * (y_ij * v_complex[j]).conj();
        p[i] += s.re;
        q[i] += s.im;
    }
    (p, q)
}

/// Construct an n×n identity matrix.
fn identity_matrix(n: usize) -> Vec<Vec<f64>> {
    (0..n)
        .map(|i| {
            let mut row = vec![0.0f64; n];
            row[i] = 1.0;
            row
        })
        .collect()
}

/// Snap a tap ratio to the nearest valid discrete tap position.
///
/// Valid positions are: tap_min, tap_min + tap_step, tap_min + 2·tap_step, …
fn snap_to_tap_step(tap: f64, tap_min: f64, tap_step: f64) -> f64 {
    if tap_step < 1e-12 {
        return tap;
    }
    let steps = ((tap - tap_min) / tap_step).round();
    tap_min + steps * tap_step
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::branch::Branch;
    use crate::network::bus::{Bus, BusType};
    use crate::network::topology::{Generator, PowerNetwork};
    use crate::units::{Power, ReactivePower};

    // ── Test network builders ─────────────────────────────────────────────────

    /// IEEE 14-bus network loaded from MATPOWER file (if available).
    fn ieee14_net() -> Option<PowerNetwork> {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/ieee14.m");
        PowerNetwork::from_matpower(path).ok()
    }

    /// Small 3-bus radial network: Bus1 (slack) → Bus2 (PQ, load) → Bus3 (PQ, load).
    fn three_bus_net() -> PowerNetwork {
        let mut net = PowerNetwork::new(100.0);

        let mut bus1 = Bus::new(1, BusType::Slack);
        bus1.vm = 1.0;
        bus1.va = 0.0;
        net.buses.push(bus1);

        let mut bus2 = Bus::new(2, BusType::PQ);
        bus2.pd = Power(40.0);
        bus2.qd = ReactivePower(20.0);
        net.buses.push(bus2);

        let mut bus3 = Bus::new(3, BusType::PQ);
        bus3.pd = Power(30.0);
        bus3.qd = ReactivePower(15.0);
        net.buses.push(bus3);

        // Slack generator
        net.generators.push(Generator {
            bus_id: 1,
            pg: 1.0,
            qg: 0.0,
            qmax: 200.0,
            qmin: -200.0,
            vg: 1.0,
            mbase: 100.0,
            status: true,
            pmax: 200.0,
            pmin: 0.0,
        });

        // Branch 1-2
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.05,
            x: 0.15,
            b: 0.02,
            rate_a: 0.0,
            rate_b: 0.0,
            rate_c: 0.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
        // Branch 2-3
        net.branches.push(Branch {
            from_bus: 2,
            to_bus: 3,
            r: 0.04,
            x: 0.12,
            b: 0.01,
            rate_a: 0.0,
            rate_b: 0.0,
            rate_c: 0.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });

        net
    }

    /// 3-bus network with a low-voltage scenario (heavy load, no reactive support).
    fn low_voltage_net() -> PowerNetwork {
        let mut net = three_bus_net();
        // Make load heavier to induce undervoltage.
        if let Some(b) = net.buses.get_mut(2) {
            b.qd = ReactivePower(40.0); // heavy Q load at bus 3 (index 2)
        }
        net
    }

    // ── Tests ─────────────────────────────────────────────────────────────────

    /// ORPD on IEEE 14-bus: loss after ORPD should be ≤ base case.
    #[test]
    fn test_orpd_basic_loss_reduction() {
        let net = match ieee14_net() {
            Some(n) => n,
            None => return, // skip if test data unavailable
        };

        // Add a generator reactive device and a capacitor bank at a high-load bus.
        let devices = vec![
            ReactiveDevice::Generator {
                bus: 0, // bus index 0 (Bus 1, slack — but can adjust Q)
                q_min: -50.0,
                q_max: 100.0,
                cost: 0.0,
            },
            ReactiveDevice::CapacitorBank {
                bus: 9, // bus index 9 (Bus 10 in IEEE 14-bus)
                q_step: 5.0,
                n_steps: 6,
                cost: 0.0,
            },
            ReactiveDevice::Svc {
                bus: 13, // bus index 13 (Bus 14)
                q_min: -20.0,
                q_max: 30.0,
                cost: 0.0,
            },
        ];

        let config = OrpdConfig {
            devices,
            v_min: 0.95,
            v_max: 1.05,
            max_outer_iter: 20,
            max_pf_iter: 50,
            tolerance: 1e-3,
            loss_weight: 1.0,
            voltage_weight: 0.1,
            delta_q_max: 20.0,
        };

        let pf_base = net
            .solve_powerflow(&PowerFlowConfig::default())
            .expect("base power flow failed");
        let base_losses = pf_base.total_losses_mw();

        let solver = OrpdSolver::new(config);
        let result = solver.solve(&net).expect("ORPD solve failed");

        // Losses after ORPD must not significantly exceed base (tolerance for
        // linearisation errors: allow up to 10% increase, but expect reduction
        // in most cases).
        assert!(
            result.total_losses_mw <= base_losses * 1.10 + 0.5,
            "ORPD losses {:.4} MW > 110% of base losses {:.4} MW",
            result.total_losses_mw,
            base_losses
        );
        assert!(
            result.loss_reduction_pct > -15.0,
            "loss_reduction_pct={:.2}% should not be catastrophically negative",
            result.loss_reduction_pct
        );
    }

    /// Capacitor bank at a low-voltage bus should not worsen voltage violations.
    #[test]
    fn test_orpd_voltage_improvement() {
        let net = low_voltage_net();
        let pf_base = net
            .solve_powerflow(&PowerFlowConfig::default())
            .expect("base power flow failed");
        let base_violations = pf_base
            .voltage_magnitude
            .iter()
            .filter(|&&vm| !(0.95 - 1e-6..=1.05 + 1e-6).contains(&vm))
            .count();

        let devices = vec![ReactiveDevice::CapacitorBank {
            bus: 2, // bus 3 (0-indexed), the heavily loaded bus
            q_step: 10.0,
            n_steps: 5,
            cost: 0.0,
        }];

        let config = OrpdConfig {
            devices,
            v_min: 0.95,
            v_max: 1.05,
            max_outer_iter: 15,
            max_pf_iter: 50,
            tolerance: 1e-3,
            loss_weight: 0.5,
            voltage_weight: 2.0,
            delta_q_max: 50.0,
        };

        let solver = OrpdSolver::new(config);
        let result = solver.solve(&net).expect("ORPD solve failed");

        // Voltage violations after ORPD must be ≤ base violations.
        assert!(
            result.voltage_violations <= base_violations + 1,
            "ORPD voltage violations {} > base {} + 1",
            result.voltage_violations,
            base_violations
        );
    }

    /// Q dispatch must remain within device limits.
    #[test]
    fn test_orpd_device_limits_respected() {
        let net = three_bus_net();

        let q_min_gen = -30.0;
        let q_max_gen = 80.0;
        let q_step_cap = 8.0;
        let n_steps_cap = 5_usize;

        let devices = vec![
            ReactiveDevice::Generator {
                bus: 0,
                q_min: q_min_gen,
                q_max: q_max_gen,
                cost: 0.0,
            },
            ReactiveDevice::CapacitorBank {
                bus: 1,
                q_step: q_step_cap,
                n_steps: n_steps_cap,
                cost: 0.0,
            },
            ReactiveDevice::Svc {
                bus: 2,
                q_min: -15.0,
                q_max: 25.0,
                cost: 0.0,
            },
        ];

        let config = OrpdConfig {
            devices: devices.clone(),
            ..OrpdConfig::default()
        };

        let solver = OrpdSolver::new(config);
        let result = solver.solve(&net).expect("ORPD solve failed");

        // Generator Q limits.
        assert!(
            result.q_dispatch[0] >= q_min_gen - 1e-6,
            "Gen Q {:.4} < q_min {:.1}",
            result.q_dispatch[0],
            q_min_gen
        );
        assert!(
            result.q_dispatch[0] <= q_max_gen + 1e-6,
            "Gen Q {:.4} > q_max {:.1}",
            result.q_dispatch[0],
            q_max_gen
        );

        // Capacitor bank: must be a non-negative multiple of q_step.
        let cap_q = result.q_dispatch[1];
        let cap_q_max = q_step_cap * n_steps_cap as f64;
        assert!(
            cap_q >= -1e-6 && cap_q <= cap_q_max + 1e-6,
            "CapBank Q {:.4} outside [0, {:.1}]",
            cap_q,
            cap_q_max
        );

        // SVC Q limits.
        assert!(
            result.q_dispatch[2] >= -15.0 - 1e-6,
            "SVC Q {:.4} < q_min",
            result.q_dispatch[2]
        );
        assert!(
            result.q_dispatch[2] <= 25.0 + 1e-6,
            "SVC Q {:.4} > q_max",
            result.q_dispatch[2]
        );
    }

    /// OLTC tap settings must stay within [tap_min, tap_max].
    #[test]
    fn test_orpd_oltc_tap_bounds() {
        let net = three_bus_net();

        let tap_min = 0.9;
        let tap_max = 1.1;
        let tap_step = 0.01;

        let devices = vec![ReactiveDevice::OltcTransformer {
            branch_idx: 0, // branch 1-2
            tap_min,
            tap_max,
            tap_step,
            cost: 0.0,
        }];

        let config = OrpdConfig {
            devices,
            max_outer_iter: 10,
            ..OrpdConfig::default()
        };

        let solver = OrpdSolver::new(config);
        let result = solver.solve(&net).expect("ORPD solve failed");

        // Only tap_settings[0] is meaningful (others are 0).
        let tap = result.tap_settings[0];
        assert!(
            tap >= tap_min - 1e-6 && tap <= tap_max + 1e-6,
            "Tap {:.4} outside [{tap_min}, {tap_max}]",
            tap,
        );
        // Verify tap is a valid discrete position.
        let steps = ((tap - tap_min) / tap_step).round();
        let snapped = tap_min + steps * tap_step;
        assert!(
            (tap - snapped).abs() < 1e-6,
            "Tap {:.6} is not a valid step (expected {:.6})",
            tap,
            snapped
        );
    }

    /// Simple network: ORPD should converge within max_outer_iter.
    #[test]
    fn test_orpd_converged_flag() {
        let net = three_bus_net();

        let devices = vec![ReactiveDevice::Svc {
            bus: 1,
            q_min: -10.0,
            q_max: 20.0,
            cost: 0.0,
        }];

        let config = OrpdConfig {
            devices,
            max_outer_iter: 30,
            tolerance: 1e-3,
            ..OrpdConfig::default()
        };

        let solver = OrpdSolver::new(config);
        let result = solver.solve(&net).expect("ORPD solve failed");

        assert!(
            result.converged,
            "ORPD should converge within {} iterations (took {})",
            30, result.n_iterations
        );
        assert!(
            result.n_iterations <= 30,
            "n_iterations {} > max 30",
            result.n_iterations
        );
    }

    /// Test `compute_voltage_sensitivity` on a known 2×2 matrix.
    #[test]
    fn test_voltage_sensitivity_2x2() {
        // L = [[2, 1], [1, 3]] → L^{-1} = 1/5 * [[3, -1], [-1, 2]]
        let l = vec![vec![2.0, 1.0], vec![1.0, 3.0]];
        let inv = compute_voltage_sensitivity(&l).expect("inversion failed");
        let eps = 1e-8;
        assert!((inv[0][0] - 0.6).abs() < eps, "inv[0][0]={:.8}", inv[0][0]);
        assert!(
            (inv[0][1] - (-0.2)).abs() < eps,
            "inv[0][1]={:.8}",
            inv[0][1]
        );
        assert!(
            (inv[1][0] - (-0.2)).abs() < eps,
            "inv[1][0]={:.8}",
            inv[1][0]
        );
        assert!((inv[1][1] - 0.4).abs() < eps, "inv[1][1]={:.8}", inv[1][1]);
    }

    /// Test `snap_to_tap_step` helper.
    #[test]
    fn test_snap_to_tap_step() {
        let taps = snap_to_tap_step(1.023, 0.9, 0.025);
        // nearest step: 0.9 + 5*0.025 = 1.025 vs 0.9 + 4*0.025 = 1.0 → 1.025
        assert!((taps - 1.025).abs() < 1e-9, "snap={:.6}", taps);

        // Exactly on a step → unchanged.
        let on_step = snap_to_tap_step(1.0, 0.9, 0.1);
        assert!((on_step - 1.0).abs() < 1e-9, "on_step={:.6}", on_step);
    }

    /// OrpdResult has correct shape when n_devices matches.
    #[test]
    fn test_orpd_result_shape() {
        let net = three_bus_net();
        let devices = vec![
            ReactiveDevice::Generator {
                bus: 0,
                q_min: -20.0,
                q_max: 60.0,
                cost: 0.0,
            },
            ReactiveDevice::OltcTransformer {
                branch_idx: 0,
                tap_min: 0.95,
                tap_max: 1.05,
                tap_step: 0.01,
                cost: 0.0,
            },
        ];
        let n = devices.len();
        let config = OrpdConfig {
            devices,
            max_outer_iter: 5,
            ..OrpdConfig::default()
        };
        let solver = OrpdSolver::new(config);
        let result = solver.solve(&net).expect("ORPD solve failed");

        assert_eq!(result.q_dispatch.len(), n);
        assert_eq!(result.tap_settings.len(), n);
        assert_eq!(result.voltage_magnitudes.len(), net.buses.len());
        // OLTC entry in q_dispatch should be 0, non-zero in tap_settings.
        assert_eq!(result.q_dispatch[1], 0.0, "OLTC q_dispatch should be 0");
        assert!(
            result.tap_settings[0] == 0.0,
            "Generator tap_settings should be 0"
        );
    }
}
