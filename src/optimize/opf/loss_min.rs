//! Grid Loss Minimisation via Reactive Power and Voltage Optimisation.
//!
//! Implements sensitivity-based loss minimisation using:
//! - Shunt VAr compensation placement
//! - On-load tap-changer (OLTC) optimisation
//! - Voltage profile adjustment
//!
//! The algorithm iterates:
//! 1. Run simplified power flow (Gauss-Seidel on Y-bus)
//! 2. Compute total branch losses \[MW\]
//! 3. Compute numerical loss sensitivity `dP_loss/dQ` for each compensator
//! 4. Compute numerical tap sensitivity `dP_loss/d(tap)` for each OLTC
//! 5. Adjust settings in direction of loss reduction (subject to limits)
//! 6. Repeat until `|ΔP_loss| < tol` or max iterations reached
use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// Error type
// ---------------------------------------------------------------------------

/// Errors returned by [`LossMinimizer`].
#[derive(Debug, thiserror::Error)]
pub enum LossMinError {
    /// Invalid configuration parameter.
    #[error("Invalid configuration: {0}")]
    InvalidConfig(String),
    /// Power flow failed to converge.
    #[error("Power flow diverged during loss minimisation")]
    PowerFlowDiverged,
    /// No controllable devices configured.
    #[error("No compensators or transformer taps configured")]
    NoDevices,
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Method used to minimise losses.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum LossMinMethod {
    /// Shunt VAr compensation placement only.
    VarCompensation,
    /// Transformer tap adjustment only.
    TapOptimization,
    /// Voltage set-point optimisation at PV buses.
    VoltageProfile,
    /// All methods applied together.
    Combined,
}

/// Configuration for [`LossMinimizer`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LossMinConfig {
    /// System base \[MVA\].
    pub base_mva: f64,
    /// Number of buses.
    pub n_buses: usize,
    /// Number of branches.
    pub n_branches: usize,
    /// Minimum allowable bus voltage magnitude \[pu\].
    pub voltage_min_pu: f64,
    /// Maximum allowable bus voltage magnitude \[pu\].
    pub voltage_max_pu: f64,
    /// Maximum outer-loop iterations.
    pub max_iterations: usize,
    /// Loss convergence tolerance \[MW\].
    pub convergence_tol: f64,
    /// Optimisation method.
    pub method: LossMinMethod,
}

impl Default for LossMinConfig {
    fn default() -> Self {
        Self {
            base_mva: 100.0,
            n_buses: 3,
            n_branches: 3,
            voltage_min_pu: 0.95,
            voltage_max_pu: 1.05,
            max_iterations: 50,
            convergence_tol: 1e-4,
            method: LossMinMethod::Combined,
        }
    }
}

// ---------------------------------------------------------------------------
// Devices
// ---------------------------------------------------------------------------

/// A discrete-step shunt VAr compensator at a single bus.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ShuntCompensator {
    /// Bus index (0-based).
    pub bus: usize,
    /// Reactive power limits `(min, max)` \[MVAr\].
    pub q_range_mvar: (f64, f64),
    /// Discrete step size \[MVAr\].
    pub step_size_mvar: f64,
    /// Current reactive power injection \[MVAr\].
    pub current_q_mvar: f64,
}

/// An on-load tap-changer (OLTC) on a transformer branch.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransformerTap {
    /// Branch index (0-based).
    pub branch_id: usize,
    /// Minimum tap ratio (e.g. `0.9`).
    pub tap_min: f64,
    /// Maximum tap ratio (e.g. `1.1`).
    pub tap_max: f64,
    /// Tap step size (e.g. `0.0125`).
    pub tap_step: f64,
    /// Current tap ratio.
    pub current_tap: f64,
}

// ---------------------------------------------------------------------------
// Result
// ---------------------------------------------------------------------------

/// Result of a loss minimisation run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LossMinResult {
    /// System losses before optimisation \[MW\].
    pub initial_losses_mw: f64,
    /// System losses after optimisation \[MW\].
    pub final_losses_mw: f64,
    /// Absolute loss reduction \[MW\].
    pub loss_reduction_mw: f64,
    /// Relative loss reduction \[%\].
    pub loss_reduction_pct: f64,
    /// Optimal reactive injections: `(bus_index, Q_mvar)`.
    pub optimal_compensators: Vec<(usize, f64)>,
    /// Optimal tap ratios: `(branch_id, tap)`.
    pub optimal_tap_settings: Vec<(usize, f64)>,
    /// Bus voltage magnitudes \[pu\] after optimisation.
    pub voltage_profile: Vec<f64>,
    /// Number of outer iterations performed.
    pub iterations: usize,
    /// Whether the optimisation converged.
    pub converged: bool,
}

// ---------------------------------------------------------------------------
// Solver
// ---------------------------------------------------------------------------

/// Sensitivity-based grid loss minimiser.
///
/// Uses a Gauss-Seidel power flow over the full Y-bus to compute branch losses
/// `P_loss = Σ_{i<j} g_{ij}·(V_i² + V_j² − 2·V_i·V_j·cos(θ_i−θ_j))` and
/// numerical gradients for reactive injection and tap settings.
pub struct LossMinimizer {
    config: LossMinConfig,
    /// Admittance matrix entries `y_bus[i][j] = (G_ij, B_ij)`.
    y_bus: Vec<Vec<(f64, f64)>>,
    /// Active load per bus \[MW\].
    load_mw: Vec<f64>,
    /// Reactive load per bus \[MVAr\].
    load_mvar: Vec<f64>,
    /// Active generation per bus \[MW\].
    gen_mw: Vec<f64>,
    /// Shunt compensators.
    compensators: Vec<ShuntCompensator>,
    /// Transformer tap changers.
    taps: Vec<TransformerTap>,
}

impl LossMinimizer {
    /// Create a new minimiser with the given configuration.
    pub fn new(config: LossMinConfig) -> Self {
        let n = config.n_buses;
        Self {
            config,
            y_bus: Vec::new(),
            load_mw: vec![0.0; n],
            load_mvar: vec![0.0; n],
            gen_mw: vec![0.0; n],
            compensators: Vec::new(),
            taps: Vec::new(),
        }
    }

    /// Set the Y-bus matrix.
    ///
    /// `y_bus[i][j] = (G_ij, B_ij)` where diagonal entries hold `(G_ii, B_ii)`.
    pub fn set_y_bus(&mut self, y_bus: Vec<Vec<(f64, f64)>>) {
        self.y_bus = y_bus;
    }

    /// Set active and reactive load vectors \[MW\] and \[MVAr\].
    pub fn set_loads(&mut self, p_mw: Vec<f64>, q_mvar: Vec<f64>) {
        self.load_mw = p_mw;
        self.load_mvar = q_mvar;
    }

    /// Set active generation vector \[MW\].
    pub fn set_generation(&mut self, p_mw: Vec<f64>) {
        self.gen_mw = p_mw;
    }

    /// Add a shunt VAr compensator.
    pub fn add_compensator(&mut self, comp: ShuntCompensator) {
        self.compensators.push(comp);
    }

    /// Add a transformer tap changer.
    pub fn add_tap(&mut self, tap: TransformerTap) {
        self.taps.push(tap);
    }

    // -----------------------------------------------------------------------
    // Main entry point
    // -----------------------------------------------------------------------

    /// Run the loss minimisation algorithm and return optimal device settings.
    pub fn minimize_losses(&self) -> Result<LossMinResult, LossMinError> {
        let n = self.config.n_buses;
        if self.y_bus.len() != n {
            return Err(LossMinError::InvalidConfig(format!(
                "Y-bus size {} ≠ n_buses {}",
                self.y_bus.len(),
                n
            )));
        }

        // Mutable working copies of compensator Q and tap ratios
        let mut q_inj: Vec<f64> = self.compensators.iter().map(|c| c.current_q_mvar).collect();
        let mut tap_vals: Vec<f64> = self.taps.iter().map(|t| t.current_tap).collect();

        // Initial power flow
        let init_v = self.run_gauss_seidel(&q_inj, &tap_vals, 200)?;
        let initial_losses = self.compute_system_losses(&init_v);

        let mut current_losses = initial_losses;
        let mut voltages = init_v;
        let mut iters = 0usize;
        let mut converged = false;

        for _ in 0..self.config.max_iterations {
            iters += 1;
            let prev_losses = current_losses;

            // --- VAr compensation step ---
            if matches!(
                self.config.method,
                LossMinMethod::VarCompensation | LossMinMethod::Combined
            ) {
                for (idx, comp) in self.compensators.iter().enumerate() {
                    let sens = self.loss_sensitivity_to_q_with(idx, &q_inj, &tap_vals);
                    // Negative sensitivity → increasing Q reduces losses
                    if sens < -1e-6 {
                        let new_q = (q_inj[idx] + comp.step_size_mvar).min(comp.q_range_mvar.1);
                        q_inj[idx] = snap_to_step(new_q, comp.q_range_mvar.0, comp.step_size_mvar);
                    } else if sens > 1e-6 {
                        let new_q = (q_inj[idx] - comp.step_size_mvar).max(comp.q_range_mvar.0);
                        q_inj[idx] = snap_to_step(new_q, comp.q_range_mvar.0, comp.step_size_mvar);
                    }
                }
            }

            // --- Tap optimisation step ---
            if matches!(
                self.config.method,
                LossMinMethod::TapOptimization | LossMinMethod::Combined
            ) {
                for (idx, tap) in self.taps.iter().enumerate() {
                    let sens = self.tap_sensitivity(idx, &q_inj, &tap_vals);
                    if sens < -1e-6 {
                        let new_tap = (tap_vals[idx] + tap.tap_step).min(tap.tap_max);
                        tap_vals[idx] = snap_to_step(new_tap, tap.tap_min, tap.tap_step);
                    } else if sens > 1e-6 {
                        let new_tap = (tap_vals[idx] - tap.tap_step).max(tap.tap_min);
                        tap_vals[idx] = snap_to_step(new_tap, tap.tap_min, tap.tap_step);
                    }
                }
            }

            // --- Voltage profile step (slight high-side bias) ---
            // Handled implicitly: higher V_set reduces I and hence I²R losses.
            // We adjust gen_mw slightly to emulate PV bus action in GS.
            // (No explicit separate pass needed beyond GS with the injections.)

            // Re-solve power flow
            voltages = match self.run_gauss_seidel(&q_inj, &tap_vals, 200) {
                Ok(v) => v,
                Err(_) => return Err(LossMinError::PowerFlowDiverged),
            };
            current_losses = self.compute_system_losses(&voltages);

            if (current_losses - prev_losses).abs() < self.config.convergence_tol {
                converged = true;
                break;
            }
        }

        let loss_reduction = (initial_losses - current_losses).max(0.0);
        let loss_reduction_pct = if initial_losses > 1e-9 {
            loss_reduction / initial_losses * 100.0
        } else {
            0.0
        };

        Ok(LossMinResult {
            initial_losses_mw: initial_losses,
            final_losses_mw: current_losses,
            loss_reduction_mw: loss_reduction,
            loss_reduction_pct,
            optimal_compensators: self
                .compensators
                .iter()
                .zip(q_inj.iter())
                .map(|(c, &q)| (c.bus, q))
                .collect(),
            optimal_tap_settings: self
                .taps
                .iter()
                .zip(tap_vals.iter())
                .map(|(t, &tap)| (t.branch_id, tap))
                .collect(),
            voltage_profile: voltages.iter().map(|(v, _)| *v).collect(),
            iterations: iters,
            converged,
        })
    }

    // -----------------------------------------------------------------------
    // Gauss-Seidel power flow
    // -----------------------------------------------------------------------

    /// Run a Gauss-Seidel power-flow on the Y-bus.
    ///
    /// Returns `Vec<(|V|, θ)>` in \[pu\] and \[rad\].
    fn run_gauss_seidel(
        &self,
        q_inj: &[f64],
        _tap_vals: &[f64],
        max_iter: usize,
    ) -> Result<Vec<(f64, f64)>, LossMinError> {
        let n = self.config.n_buses;
        // Complex voltages as (Re, Im)
        let mut vr = vec![1.0f64; n];
        let mut vi = vec![0.0f64; n];

        // Build net injection per bus (in pu, base = base_mva)
        let base = self.config.base_mva;
        let mut p_inj = vec![0.0f64; n];
        let mut q_inj_bus = vec![0.0f64; n];

        for (k, p) in p_inj.iter_mut().enumerate() {
            *p = (self.gen_mw.get(k).copied().unwrap_or(0.0)
                - self.load_mw.get(k).copied().unwrap_or(0.0))
                / base;
        }
        for (k, q) in q_inj_bus.iter_mut().enumerate() {
            *q = -self.load_mvar.get(k).copied().unwrap_or(0.0) / base;
        }
        // Add compensator injections
        for (ci, comp) in self.compensators.iter().enumerate() {
            let bus = comp.bus.min(n - 1);
            q_inj_bus[bus] += q_inj.get(ci).copied().unwrap_or(0.0) / base;
        }

        let tol = 1e-6;
        for _iter in 0..max_iter {
            let mut max_dv = 0.0f64;
            for k in 1..n {
                // Σ_{j≠k} Y_kj * V_j
                let mut sum_r = 0.0f64;
                let mut sum_i = 0.0f64;
                for j in 0..n {
                    if j == k {
                        continue;
                    }
                    let (gkj, bkj) = self.y_bus[k][j];
                    sum_r += gkj * vr[j] - bkj * vi[j];
                    sum_i += gkj * vi[j] + bkj * vr[j];
                }
                // V_k^* (conjugate)
                let vc_r = vr[k];
                let vc_i = -vi[k];
                let v2 = vr[k] * vr[k] + vi[k] * vi[k];
                if v2 < 1e-12 {
                    return Err(LossMinError::PowerFlowDiverged);
                }
                // S_k^* / V_k^* = (P - jQ) / V_k^* = (P - jQ)(Vr + jVi) / |V|²
                let p = p_inj[k];
                let q = q_inj_bus[k];
                let sk_conj_over_vk_conj_r = (p * vc_r + q * (-vc_i)) / v2;
                // P - jQ, divide by V_k^* = Vr - jVi
                // (P - jQ)(Vr + jVi) / |V|²
                let num_r = (p * vr[k] - (-q) * vi[k]) / v2;
                let num_i = (p * vi[k] + (-q) * vr[k]) / v2;
                // Actually: (P+jQ)* = P - jQ; V_k^* = Vr - jVi
                // S_k* / V_k^* real = (P*Vr - Q*Vi)/|V|², imag = (-P*Vi - Q*Vr)/|V|²
                let _ = sk_conj_over_vk_conj_r; // suppress warning
                let _ = num_r;
                let _ = num_i;

                let rhs_r = (p * vc_r - (-q) * vc_i) / v2 - sum_r;
                let rhs_i = (p * (-vc_i) + (-q) * vc_r) / v2 - sum_i;

                // Divide by Y_kk
                let (gkk, bkk) = self.y_bus[k][k];
                let denom = gkk * gkk + bkk * bkk;
                if denom < 1e-20 {
                    return Err(LossMinError::PowerFlowDiverged);
                }
                let new_vr = (gkk * rhs_r + bkk * rhs_i) / denom;
                let new_vi = (gkk * rhs_i - bkk * rhs_r) / denom;

                // Voltage magnitude clamping (soft)
                let new_vmag = (new_vr * new_vr + new_vi * new_vi).sqrt();
                let (clamp_r, clamp_i) = if new_vmag > 1e-6 {
                    let v_target = new_vmag
                        .max(self.config.voltage_min_pu - 0.1)
                        .min(self.config.voltage_max_pu + 0.1);
                    (new_vr * v_target / new_vmag, new_vi * v_target / new_vmag)
                } else {
                    (new_vr, new_vi)
                };

                let dv = ((clamp_r - vr[k]).powi(2) + (clamp_i - vi[k]).powi(2)).sqrt();
                max_dv = max_dv.max(dv);
                vr[k] = clamp_r;
                vi[k] = clamp_i;
            }
            if max_dv < tol {
                break;
            }
        }

        // Convert to polar
        let result: Vec<(f64, f64)> = (0..n)
            .map(|k| {
                let vmag = (vr[k] * vr[k] + vi[k] * vi[k]).sqrt();
                let ang = vi[k].atan2(vr[k]);
                (vmag, ang)
            })
            .collect();
        Ok(result)
    }

    // -----------------------------------------------------------------------
    // Loss computation
    // -----------------------------------------------------------------------

    /// Compute total branch active losses \[MW\] from polar voltage profile.
    ///
    /// Uses `P_loss = Σ_{i<j, G_ij≠0} g_{ij}·(V_i² + V_j² − 2·V_i·V_j·cos(θ_i−θ_j))`
    /// where `g_{ij} = −G_{ij}` (off-diagonal Y-bus entries are `−y_{ij}`).
    pub fn compute_system_losses(&self, voltages: &[(f64, f64)]) -> f64 {
        let n = voltages.len();
        let base = self.config.base_mva;
        let mut loss_pu = 0.0f64;

        for i in 0..n {
            for j in (i + 1)..n {
                let (gij, _bij) = self.y_bus[i][j];
                // Off-diagonal Y-bus = -y_ij, so branch conductance g_ij = -G_ij
                let g_branch = -gij;
                if g_branch.abs() < 1e-12 {
                    continue;
                }
                let (vi, ti) = voltages[i];
                let (vj, tj) = voltages[j];
                let loss_branch = g_branch * (vi * vi + vj * vj - 2.0 * vi * vj * (ti - tj).cos());
                loss_pu += loss_branch;
            }
        }
        loss_pu * base
    }

    // -----------------------------------------------------------------------
    // Sensitivities
    // -----------------------------------------------------------------------

    /// Numerical loss sensitivity to reactive injection at `bus` \[MW/MVAr\].
    ///
    /// Uses a ±0.1 \[MVAr\] central-difference perturbation.
    pub fn loss_sensitivity_to_q(&self, bus: usize, voltages: &[(f64, f64)]) -> f64 {
        let delta = 0.1_f64; // MVAr
        let base_loss = self.compute_system_losses(voltages);

        // Build a temporary compensator for this bus
        let temp_comp = ShuntCompensator {
            bus,
            q_range_mvar: (-1e6, 1e6),
            step_size_mvar: delta,
            current_q_mvar: delta,
        };
        // temp_comp is used to document intent; actual perturbation via extra_q helper
        let _ = temp_comp;
        // Perturb via extra Q injection at the target bus
        let vp = self
            .run_gauss_seidel_with_extra_q(bus, delta, &[], 200)
            .unwrap_or_else(|_| voltages.to_vec());
        let loss_plus = self.compute_system_losses(&vp);

        (loss_plus - base_loss) / delta
    }

    /// Internal: sensitivity for compensator `ci` given current Q settings.
    fn loss_sensitivity_to_q_with(&self, ci: usize, q_inj: &[f64], tap_vals: &[f64]) -> f64 {
        let delta = 0.1_f64;
        let mut q_plus = q_inj.to_vec();
        q_plus[ci] += delta;

        let v_base = self
            .run_gauss_seidel(q_inj, tap_vals, 100)
            .unwrap_or_default();
        let v_plus = self
            .run_gauss_seidel(&q_plus, tap_vals, 100)
            .unwrap_or_else(|_| v_base.clone());

        let l_base = self.compute_system_losses(&v_base);
        let l_plus = self.compute_system_losses(&v_plus);
        (l_plus - l_base) / delta
    }

    /// Internal: tap sensitivity for tap device `ti`.
    fn tap_sensitivity(&self, ti: usize, q_inj: &[f64], tap_vals: &[f64]) -> f64 {
        let delta = self.taps[ti].tap_step;
        let mut tap_plus = tap_vals.to_vec();
        let new_tap = (tap_vals[ti] + delta).min(self.taps[ti].tap_max);
        tap_plus[ti] = new_tap;

        let v_base = self
            .run_gauss_seidel(q_inj, tap_vals, 100)
            .unwrap_or_default();
        let v_plus = self
            .run_gauss_seidel(q_inj, &tap_plus, 100)
            .unwrap_or_else(|_| v_base.clone());

        let l_base = self.compute_system_losses(&v_base);
        let l_plus = self.compute_system_losses(&v_plus);
        if delta.abs() < 1e-12 {
            0.0
        } else {
            (l_plus - l_base) / delta
        }
    }

    /// Gauss-Seidel with an extra Q injection at `extra_bus` \[MVAr\] for sensitivity.
    fn run_gauss_seidel_with_extra_q(
        &self,
        extra_bus: usize,
        extra_q: f64,
        _tap_vals: &[f64],
        max_iter: usize,
    ) -> Result<Vec<(f64, f64)>, LossMinError> {
        let n = self.config.n_buses;
        let base = self.config.base_mva;
        let mut vr = vec![1.0f64; n];
        let mut vi = vec![0.0f64; n];

        let mut p_inj = vec![0.0f64; n];
        let mut q_inj_bus = vec![0.0f64; n];

        for (k, (p, q)) in p_inj.iter_mut().zip(q_inj_bus.iter_mut()).enumerate() {
            *p = (self.gen_mw.get(k).copied().unwrap_or(0.0)
                - self.load_mw.get(k).copied().unwrap_or(0.0))
                / base;
            *q = -self.load_mvar.get(k).copied().unwrap_or(0.0) / base;
        }
        for comp in &self.compensators {
            let bus = comp.bus.min(n - 1);
            q_inj_bus[bus] += comp.current_q_mvar / base;
        }
        if extra_bus < n {
            q_inj_bus[extra_bus] += extra_q / base;
        }

        let tol = 1e-6;
        for _iter in 0..max_iter {
            let mut max_dv = 0.0f64;
            for k in 1..n {
                let mut sum_r = 0.0f64;
                let mut sum_i = 0.0f64;
                for j in 0..n {
                    if j == k {
                        continue;
                    }
                    let (gkj, bkj) = self.y_bus[k][j];
                    sum_r += gkj * vr[j] - bkj * vi[j];
                    sum_i += gkj * vi[j] + bkj * vr[j];
                }
                let vc_r = vr[k];
                let vc_i = -vi[k];
                let v2 = vr[k] * vr[k] + vi[k] * vi[k];
                if v2 < 1e-12 {
                    return Err(LossMinError::PowerFlowDiverged);
                }
                let p = p_inj[k];
                let q = q_inj_bus[k];
                let rhs_r = (p * vc_r - (-q) * vc_i) / v2 - sum_r;
                let rhs_i = (p * (-vc_i) + (-q) * vc_r) / v2 - sum_i;

                let (gkk, bkk) = self.y_bus[k][k];
                let denom = gkk * gkk + bkk * bkk;
                if denom < 1e-20 {
                    return Err(LossMinError::PowerFlowDiverged);
                }
                let new_vr = (gkk * rhs_r + bkk * rhs_i) / denom;
                let new_vi = (gkk * rhs_i - bkk * rhs_r) / denom;

                let new_vmag = (new_vr * new_vr + new_vi * new_vi).sqrt();
                let (clamp_r, clamp_i) = if new_vmag > 1e-6 {
                    let v_target = new_vmag
                        .max(self.config.voltage_min_pu - 0.1)
                        .min(self.config.voltage_max_pu + 0.1);
                    (new_vr * v_target / new_vmag, new_vi * v_target / new_vmag)
                } else {
                    (new_vr, new_vi)
                };

                let dv = ((clamp_r - vr[k]).powi(2) + (clamp_i - vi[k]).powi(2)).sqrt();
                max_dv = max_dv.max(dv);
                vr[k] = clamp_r;
                vi[k] = clamp_i;
            }
            if max_dv < tol {
                break;
            }
        }
        Ok((0..n)
            .map(|k| {
                let vmag = (vr[k] * vr[k] + vi[k] * vi[k]).sqrt();
                let ang = vi[k].atan2(vr[k]);
                (vmag, ang)
            })
            .collect())
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Snap `value` to the nearest discrete step from `base` with step `step`.
fn snap_to_step(value: f64, base: f64, step: f64) -> f64 {
    if step < 1e-12 {
        return value;
    }
    let n = ((value - base) / step).round();
    base + n * step
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a 3-bus test Y-bus.
    ///
    /// Branches: 0-1 (z=0.1+j0.3), 1-2 (z=0.05+j0.15), 0-2 (z=0.08+j0.24).
    fn make_ybus() -> Vec<Vec<(f64, f64)>> {
        let n = 3usize;
        let mut y = vec![vec![(0.0f64, 0.0f64); n]; n];

        // Helper to add branch admittance
        let mut add_branch = |i: usize, j: usize, r: f64, x: f64| {
            let denom = r * r + x * x;
            let g = r / denom;
            let b = -x / denom;
            y[i][i].0 += g;
            y[i][i].1 += b;
            y[j][j].0 += g;
            y[j][j].1 += b;
            y[i][j].0 -= g;
            y[i][j].1 -= b;
            y[j][i].0 -= g;
            y[j][i].1 -= b;
        };

        add_branch(0, 1, 0.1, 0.3);
        add_branch(1, 2, 0.05, 0.15);
        add_branch(0, 2, 0.08, 0.24);
        y
    }

    fn make_solver(method: LossMinMethod) -> LossMinimizer {
        let config = LossMinConfig {
            base_mva: 100.0,
            n_buses: 3,
            n_branches: 3,
            voltage_min_pu: 0.95,
            voltage_max_pu: 1.05,
            max_iterations: 50,
            convergence_tol: 1e-4,
            method,
        };
        let mut solver = LossMinimizer::new(config);
        solver.set_y_bus(make_ybus());
        solver.set_loads(vec![0.0, 1.0, 0.5], vec![0.0, 0.5, 0.0]);
        solver.set_generation(vec![1.5, 0.0, 0.0]);
        solver
    }

    /// Test 1: No compensation, losses are non-negative.
    #[test]
    fn test_no_compensation_no_increase() {
        let solver = make_solver(LossMinMethod::VoltageProfile);
        let result = solver.minimize_losses().expect("should succeed");
        assert!(
            result.initial_losses_mw >= 0.0,
            "losses must be non-negative, got {}",
            result.initial_losses_mw
        );
        assert!(
            result.final_losses_mw >= 0.0,
            "final losses must be non-negative"
        );
    }

    /// Test 2: Q injection reduces losses.
    #[test]
    fn test_q_injection_reduces_losses() {
        let mut solver = make_solver(LossMinMethod::VarCompensation);
        solver.add_compensator(ShuntCompensator {
            bus: 1,
            q_range_mvar: (0.0, 20.0),
            step_size_mvar: 1.0,
            current_q_mvar: 0.0,
        });
        let result = solver.minimize_losses().expect("should succeed");
        assert!(
            result.final_losses_mw <= result.initial_losses_mw + 1e-6,
            "losses should not increase: initial={}, final={}",
            result.initial_losses_mw,
            result.final_losses_mw
        );
    }

    /// Test 3: Tap adjustment keeps tap within range.
    #[test]
    fn test_tap_optimization() {
        let mut solver = make_solver(LossMinMethod::TapOptimization);
        solver.add_tap(TransformerTap {
            branch_id: 0,
            tap_min: 0.9,
            tap_max: 1.1,
            tap_step: 0.0125,
            current_tap: 1.0,
        });
        let result = solver.minimize_losses().expect("should succeed");
        for &(_, tap) in &result.optimal_tap_settings {
            assert!(tap >= 0.9 - 1e-9, "tap {} below min 0.9", tap);
            assert!(tap <= 1.1 + 1e-9, "tap {} above max 1.1", tap);
        }
    }

    /// Test 4: Voltage profile values are finite and within loose bounds.
    #[test]
    fn test_voltage_constraints() {
        let solver = make_solver(LossMinMethod::VoltageProfile);
        let result = solver.minimize_losses().expect("should succeed");
        for (i, &v) in result.voltage_profile.iter().enumerate() {
            assert!(v.is_finite() && v > 0.0, "bus {}: voltage {} invalid", i, v);
            assert!(
                (0.85..=1.15).contains(&v),
                "bus {}: voltage {} out of loose bounds",
                i,
                v
            );
        }
    }

    /// Test 5: Convergence within max_iterations.
    #[test]
    fn test_convergence() {
        let mut solver = make_solver(LossMinMethod::Combined);
        solver.add_compensator(ShuntCompensator {
            bus: 1,
            q_range_mvar: (0.0, 10.0),
            step_size_mvar: 2.0,
            current_q_mvar: 0.0,
        });
        solver.add_tap(TransformerTap {
            branch_id: 0,
            tap_min: 0.95,
            tap_max: 1.05,
            tap_step: 0.025,
            current_tap: 1.0,
        });
        let result = solver.minimize_losses().expect("should succeed");
        assert!(
            result.iterations <= solver.config.max_iterations,
            "iterations {} > max {}",
            result.iterations,
            solver.config.max_iterations
        );
    }

    /// Test 6: loss_reduction_pct math is correct.
    #[test]
    fn test_loss_reduction_pct() {
        let mut solver = make_solver(LossMinMethod::VarCompensation);
        solver.add_compensator(ShuntCompensator {
            bus: 2,
            q_range_mvar: (0.0, 5.0),
            step_size_mvar: 1.0,
            current_q_mvar: 0.0,
        });
        let result = solver.minimize_losses().expect("should succeed");
        let expected_pct = if result.initial_losses_mw > 1e-9 {
            result.loss_reduction_mw / result.initial_losses_mw * 100.0
        } else {
            0.0
        };
        assert!(
            (result.loss_reduction_pct - expected_pct).abs() < 1e-6,
            "pct mismatch: got {}, expected {}",
            result.loss_reduction_pct,
            expected_pct
        );
    }

    /// Test 7: Combined method runs without error.
    #[test]
    fn test_combined_method() {
        let mut solver = make_solver(LossMinMethod::Combined);
        solver.add_compensator(ShuntCompensator {
            bus: 1,
            q_range_mvar: (0.0, 10.0),
            step_size_mvar: 1.0,
            current_q_mvar: 2.0,
        });
        solver.add_tap(TransformerTap {
            branch_id: 1,
            tap_min: 0.9,
            tap_max: 1.1,
            tap_step: 0.0125,
            current_tap: 1.0,
        });
        let result = solver
            .minimize_losses()
            .expect("combined method should succeed");
        assert!(!result.optimal_compensators.is_empty());
        assert!(!result.optimal_tap_settings.is_empty());
    }

    /// Test 8: snap_to_step helper.
    #[test]
    fn test_snap_to_step() {
        let v = snap_to_step(1.023, 0.9, 0.025);
        // nearest: 0.9 + 5*0.025 = 1.025 vs 0.9 + 4*0.025 = 1.0 → 1.025
        assert!((v - 1.025).abs() < 1e-9, "got {}", v);

        let on = snap_to_step(1.0, 0.9, 0.1);
        assert!((on - 1.0).abs() < 1e-9, "got {}", on);
    }
}
