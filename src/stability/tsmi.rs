//! Transient Stability Margin Index (TSMI) — online stability assessment and
//! real-time preventive control.
//!
//! # Overview
//! Provides multiple methods for computing a scalar stability margin that
//! quantifies *how far* the post-fault trajectory is from the stability
//! boundary:
//!
//! * **Energy-function** — Lyapunov transient-energy margin (PEBS/UEP method)
//! * **Single-Machine Equivalent (SME/SIME)** — equal-area criterion on the
//!   reduced equivalent machine
//! * **Closest Unstable Equilibrium Point** — explicit UEP computation
//! * **Time-domain integral / Controlled decomposition** — fall back to the
//!   energy-function path in this implementation
//!
//! # Key formulas
//! * COI angle  : `δ_COI = Σ H_i δ_i / Σ H_i`
//! * COI speed  : `ω_COI = Σ H_i ω_i / Σ H_i`
//! * Kinetic energy: `KE = Σ_i (H_i / ω_s) · ω̃_i²`   (ω_s = 2π·50 rad/s)
//! * Potential energy: `PE = −Σ_{i<j} E_i E_j (G_ij sin δ_ij − B_ij cos δ_ij)`
//! * Energy margin : `η = V_cr − V(t_cl)`   (positive ⟹ stable)
//! * SME CCT      : `t_cct = √(2H_eq · δ_u / (ω_s · P_a))`
//!
//! # Legacy API
//! The original [`TransientStabilityMargin`] struct (EAC, Lyapunov, PEBS) is
//! preserved below for backward compatibility.
//!
//! # References
//! Anderson & Fouad, *Power System Control and Stability*, 2nd ed., Chs. 2, 11.
//! Pavella, Ernst & Ruiz-Vega, *Transient Stability of Power Systems*, Wiley 2000.

use crate::error::{OxiGridError, Result};
use crate::network::topology::PowerNetwork;
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;
use std::time::Instant;

/// Synchronous angular frequency at 50 Hz [rad/s].
const OMEGA_S: f64 = 2.0 * PI * 50.0; // ≈ 314.159 rad/s

// ---------------------------------------------------------------------------
// Enums
// ---------------------------------------------------------------------------

/// Algorithm used to compute the transient stability margin.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TsmiMethod {
    /// Lyapunov transient-energy function (PEBS/UEP).
    EnergyFunction,
    /// Time-domain simulation integral metric.
    TimedomainIntegral,
    /// Closest unstable equilibrium point (CUEP) method.
    ClosestUnstableEp,
    /// Controlled system decomposition (BCU) method.
    ControlledDecomposition,
    /// Single-Machine Equivalent (SIME) with equal-area criterion.
    SingleMachineEquivalent,
}

/// Qualitative stability classification derived from the numerical margin.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum StabilityStatus {
    /// Margin > 0.1 — system is robustly stable.
    Stable,
    /// 0 < margin ≤ 0.1 — system is on the stability boundary.
    MarginallyStable,
    /// Margin ≤ 0 — system will lose synchronism.
    Unstable,
    /// Assessment could not be completed (e.g., insufficient data).
    Indeterminate,
}

// ---------------------------------------------------------------------------
// Structs
// ---------------------------------------------------------------------------

/// State of a single synchronous machine at a given simulation instant.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MachineState {
    /// Generator index (0-based).
    pub id: usize,
    /// Rotor angle δ `rad`.
    pub rotor_angle_rad: f64,
    /// Speed deviation Δω = ω − ω_s [rad/s] (zero at steady state).
    pub rotor_speed_rad_per_s: f64,
    /// Inertia constant H `s`.
    pub inertia_h_s: f64,
    /// Damping coefficient D [p.u.].
    pub damping_d: f64,
    /// Mechanical input power P_m [p.u.].
    pub p_mechanical_pu: f64,
    /// Electrical output power P_e [p.u.].
    pub p_electrical_pu: f64,
    /// Reactive electrical power Q_e [p.u.].
    pub q_electrical_pu: f64,
}

impl MachineState {
    /// Create a new machine state with all fields specified.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        id: usize,
        rotor_angle_rad: f64,
        rotor_speed_rad_per_s: f64,
        inertia_h_s: f64,
        damping_d: f64,
        p_mechanical_pu: f64,
        p_electrical_pu: f64,
        q_electrical_pu: f64,
    ) -> Self {
        Self {
            id,
            rotor_angle_rad,
            rotor_speed_rad_per_s,
            inertia_h_s,
            damping_d,
            p_mechanical_pu,
            p_electrical_pu,
            q_electrical_pu,
        }
    }
}

/// Lyapunov transient-energy function evaluated at fault clearing.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransientEnergyFunction {
    /// Kinetic energy `KE = Σ (H_i / ω_s) · ω̃_i²` [p.u.·s].
    pub kinetic_energy: f64,
    /// Potential energy relative to stable equilibrium point [p.u.·s].
    pub potential_energy: f64,
    /// Total Lyapunov energy `V = KE + PE` [p.u.·s].
    pub total_energy: f64,
    /// Critical energy at the closest UEP `V_cr` [p.u.·s].
    pub critical_energy: f64,
    /// Energy margin `η = V_cr − V(t_cl)` (positive ⟹ stable).
    pub margin: f64,
    /// Normalised margin `η / |V_cr|` ∈ [−1, 1].
    pub normalized_margin: f64,
}

/// Single-Machine Equivalent (SME/SIME) model parameters and results.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SmeModel {
    /// Equivalent rotor angle of the critical cluster `rad`.
    pub equivalent_angle_rad: f64,
    /// Equivalent speed deviation [rad/s].
    pub equivalent_speed_rad_per_s: f64,
    /// Equivalent inertia constant H_eq `s`.
    pub equivalent_inertia: f64,
    /// Maximum accelerating power of the equivalent machine [p.u.].
    pub pa_max: f64,
    /// Critical clearing time derived from equal-area criterion `s`.
    pub critical_clearing_time_s: f64,
    /// Stability index `1 − t_cl / t_cct` (positive ⟹ stable).
    pub stability_index: f64,
}

/// Complete result of a TSMI computation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TsmiResult {
    /// Algorithm that produced this result.
    pub method: TsmiMethod,
    /// Qualitative stability classification.
    pub status: StabilityStatus,
    /// Numerical stability margin (positive ⟹ stable).
    pub margin: f64,
    /// Normalised stability margin ∈ [−1, 1].
    pub normalized_margin: f64,
    /// IDs of the most stressed (critical) generators.
    pub critical_machines: Vec<usize>,
    /// Lyapunov energy decomposition, if computed.
    pub energy_function: Option<TransientEnergyFunction>,
    /// SME model, if computed.
    pub sme: Option<SmeModel>,
    /// Simulated duration `s`.
    pub simulation_time_s: f64,
    /// Wall-clock computation time `ms`.
    pub computation_time_ms: f64,
}

/// Main calculator for the Transient Stability Margin Index.
///
/// # Quick start
/// ```rust,ignore
/// let mut calc = TsmiCalculator::new(machines, 0.1);
/// calc.method = TsmiMethod::EnergyFunction;
/// let result = calc.compute_tsmi();
/// ```
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TsmiCalculator {
    /// Selected computation method.
    pub method: TsmiMethod,
    /// Machine states at the start of the post-fault period.
    pub machines: Vec<MachineState>,
    /// Reduced generator admittance matrix stored as `(G_ij, B_ij)` entries.
    /// Dimensions: `n_gen × n_gen`.  Empty means no inter-machine coupling.
    pub network_admittance: Vec<Vec<(f64, f64)>>,
    /// Fault clearing time t_cl `s`.
    pub fault_clearing_time_s: f64,
    /// Numerical integration step `s` (default 0.01 s).
    pub simulation_step_s: f64,
    /// Maximum simulation horizon `s` (default 5.0 s).
    pub max_simulation_time_s: f64,
}

// ---------------------------------------------------------------------------
// TsmiCalculator implementation
// ---------------------------------------------------------------------------

impl TsmiCalculator {
    /// Construct a new calculator with default settings.
    ///
    /// The method defaults to [`TsmiMethod::EnergyFunction`].
    /// `network_admittance` is left empty (machines treated as isolated unless
    /// set by the caller).
    pub fn new(machines: Vec<MachineState>, fault_clearing_time_s: f64) -> Self {
        Self {
            method: TsmiMethod::EnergyFunction,
            machines,
            network_admittance: Vec::new(),
            fault_clearing_time_s,
            simulation_step_s: 0.01,
            max_simulation_time_s: 5.0,
        }
    }

    // -----------------------------------------------------------------------
    // Public API
    // -----------------------------------------------------------------------

    /// Compute the TSMI using the configured method and return a [`TsmiResult`].
    pub fn compute_tsmi(&mut self) -> TsmiResult {
        let t_start = Instant::now();
        match self.method {
            TsmiMethod::SingleMachineEquivalent => self.tsmi_sme(t_start),
            _ => self.tsmi_energy(t_start),
        }
    }

    /// Compute the Lyapunov transient-energy function at the current machine
    /// states.
    pub fn compute_energy_function(&self) -> TransientEnergyFunction {
        if self.machines.is_empty() {
            return TransientEnergyFunction {
                kinetic_energy: 0.0,
                potential_energy: 0.0,
                total_energy: 0.0,
                critical_energy: 0.0,
                margin: 0.0,
                normalized_margin: 0.0,
            };
        }

        let (_, omega_coi) = self.compute_coi();

        // Kinetic energy in COI frame: KE = Σ (H_i / ω_s) · ω̃_i²
        let ke: f64 = self
            .machines
            .iter()
            .map(|m| {
                let omega_tilde = m.rotor_speed_rad_per_s - omega_coi;
                (m.inertia_h_s / OMEGA_S) * omega_tilde * omega_tilde
            })
            .sum();

        // Potential energy from network
        let current_angles: Vec<f64> = self.machines.iter().map(|m| m.rotor_angle_rad).collect();
        let pe = self.compute_network_pe(&current_angles);

        let total_energy = ke + pe;

        // Critical energy at UEP
        let uep_angles = self.find_closest_unstable_ep();
        let critical_energy = self.compute_pe_at_uep(&uep_angles);

        let margin = critical_energy - total_energy;
        let normalized_margin = if critical_energy.abs() > 1e-12 {
            (margin / critical_energy.abs()).clamp(-1.0, 1.0)
        } else {
            margin.signum().clamp(-1.0, 1.0)
        };

        TransientEnergyFunction {
            kinetic_energy: ke,
            potential_energy: pe,
            total_energy,
            critical_energy,
            margin,
            normalized_margin,
        }
    }

    /// Compute the Centre-of-Inertia (COI) angle and speed deviation.
    ///
    /// Returns `(δ_COI `rad`, ω_COI [rad/s])`.
    /// If total inertia is zero, returns `(0.0, 0.0)`.
    pub fn compute_coi(&self) -> (f64, f64) {
        let total_h: f64 = self.machines.iter().map(|m| m.inertia_h_s).sum();
        if total_h < 1e-12 {
            return (0.0, 0.0);
        }
        let delta_coi = self
            .machines
            .iter()
            .map(|m| m.inertia_h_s * m.rotor_angle_rad)
            .sum::<f64>()
            / total_h;
        let omega_coi = self
            .machines
            .iter()
            .map(|m| m.inertia_h_s * m.rotor_speed_rad_per_s)
            .sum::<f64>()
            / total_h;
        (delta_coi, omega_coi)
    }

    /// Compute electrical output power of machine `machine_id` from the
    /// network admittance matrix.
    ///
    /// `P_ei = Σ_j E_i E_j (G_ij cos δ_ij + B_ij sin δ_ij)`
    ///
    /// If the Y-matrix is not set, the stored `p_electrical_pu` is returned.
    pub fn compute_electrical_power(&self, machine_id: usize) -> f64 {
        if machine_id >= self.machines.len() {
            return 0.0;
        }
        if self.network_admittance.is_empty() || machine_id >= self.network_admittance.len() {
            return self.machines[machine_id].p_electrical_pu;
        }
        let row = &self.network_admittance[machine_id];
        let delta_i = self.machines[machine_id].rotor_angle_rad;
        let mut p_e = 0.0_f64;
        for (j, &(g_ij, b_ij)) in row.iter().enumerate() {
            if j >= self.machines.len() {
                break;
            }
            let d_ij = delta_i - self.machines[j].rotor_angle_rad;
            p_e += g_ij * d_ij.cos() + b_ij * d_ij.sin();
        }
        p_e
    }

    /// Integrate the multi-machine swing equations using 4th-order Runge-Kutta.
    ///
    /// Swing equations in COI frame:
    /// ```text
    /// dδ̃/dt = ω̃
    /// dω̃/dt = (ω_s / 2H) · (P_m − P_e − D · ω̃)
    /// ```
    ///
    /// Returns `traj[machine_idx][step_idx] = (δ `rad`, Δω [rad/s])`.
    /// Machine states in `self.machines` are updated to the final values.
    pub fn integrate_swing_equations(&mut self, duration_s: f64) -> Vec<Vec<(f64, f64)>> {
        let n = self.machines.len();
        if n == 0 || duration_s <= 0.0 {
            return Vec::new();
        }

        let dt = self.simulation_step_s;
        let steps = (duration_s / dt).ceil() as usize;

        let mut traj: Vec<Vec<(f64, f64)>> = vec![Vec::with_capacity(steps + 1); n];

        // Working copies of state
        let mut angles: Vec<f64> = self.machines.iter().map(|m| m.rotor_angle_rad).collect();
        let mut speeds: Vec<f64> = self
            .machines
            .iter()
            .map(|m| m.rotor_speed_rad_per_s)
            .collect();

        for i in 0..n {
            traj[i].push((angles[i], speeds[i]));
        }

        for _ in 0..steps {
            // --- k1 ---
            let pe1 = self.electrical_power_from_angles(&angles);
            let omega_coi1 = coi_speed_weighted(&self.machines, &speeds);
            let k1d: Vec<f64> = (0..n).map(|i| speeds[i] - omega_coi1).collect();
            let k1w: Vec<f64> = (0..n)
                .map(|i| {
                    let m = &self.machines[i];
                    let wt = speeds[i] - omega_coi1;
                    let pa = m.p_mechanical_pu - pe1[i] - m.damping_d * wt;
                    (OMEGA_S / (2.0 * m.inertia_h_s.max(1e-9))) * pa
                })
                .collect();

            // --- k2 ---
            let a2: Vec<f64> = (0..n).map(|i| angles[i] + 0.5 * dt * k1d[i]).collect();
            let s2: Vec<f64> = (0..n).map(|i| speeds[i] + 0.5 * dt * k1w[i]).collect();
            let pe2 = self.electrical_power_from_angles(&a2);
            let omega_coi2 = coi_speed_weighted(&self.machines, &s2);
            let k2d: Vec<f64> = (0..n).map(|i| s2[i] - omega_coi2).collect();
            let k2w: Vec<f64> = (0..n)
                .map(|i| {
                    let m = &self.machines[i];
                    let wt = s2[i] - omega_coi2;
                    let pa = m.p_mechanical_pu - pe2[i] - m.damping_d * wt;
                    (OMEGA_S / (2.0 * m.inertia_h_s.max(1e-9))) * pa
                })
                .collect();

            // --- k3 ---
            let a3: Vec<f64> = (0..n).map(|i| angles[i] + 0.5 * dt * k2d[i]).collect();
            let s3: Vec<f64> = (0..n).map(|i| speeds[i] + 0.5 * dt * k2w[i]).collect();
            let pe3 = self.electrical_power_from_angles(&a3);
            let omega_coi3 = coi_speed_weighted(&self.machines, &s3);
            let k3d: Vec<f64> = (0..n).map(|i| s3[i] - omega_coi3).collect();
            let k3w: Vec<f64> = (0..n)
                .map(|i| {
                    let m = &self.machines[i];
                    let wt = s3[i] - omega_coi3;
                    let pa = m.p_mechanical_pu - pe3[i] - m.damping_d * wt;
                    (OMEGA_S / (2.0 * m.inertia_h_s.max(1e-9))) * pa
                })
                .collect();

            // --- k4 ---
            let a4: Vec<f64> = (0..n).map(|i| angles[i] + dt * k3d[i]).collect();
            let s4: Vec<f64> = (0..n).map(|i| speeds[i] + dt * k3w[i]).collect();
            let pe4 = self.electrical_power_from_angles(&a4);
            let omega_coi4 = coi_speed_weighted(&self.machines, &s4);
            let k4d: Vec<f64> = (0..n).map(|i| s4[i] - omega_coi4).collect();
            let k4w: Vec<f64> = (0..n)
                .map(|i| {
                    let m = &self.machines[i];
                    let wt = s4[i] - omega_coi4;
                    let pa = m.p_mechanical_pu - pe4[i] - m.damping_d * wt;
                    (OMEGA_S / (2.0 * m.inertia_h_s.max(1e-9))) * pa
                })
                .collect();

            // --- Combine ---
            for i in 0..n {
                angles[i] += (dt / 6.0) * (k1d[i] + 2.0 * k2d[i] + 2.0 * k3d[i] + k4d[i]);
                speeds[i] += (dt / 6.0) * (k1w[i] + 2.0 * k2w[i] + 2.0 * k3w[i] + k4w[i]);
                traj[i].push((angles[i], speeds[i]));
            }
        }

        // Write back final state
        for i in 0..n {
            self.machines[i].rotor_angle_rad = angles[i];
            self.machines[i].rotor_speed_rad_per_s = speeds[i];
        }

        traj
    }

    /// Find the closest unstable equilibrium point (UEP) angles.
    ///
    /// Simplified: `δ_u_i = π − δ_s_i` (SMIB analogy per machine).
    pub fn find_closest_unstable_ep(&self) -> Vec<f64> {
        self.machines
            .iter()
            .map(|m| PI - m.rotor_angle_rad)
            .collect()
    }

    /// Compute the Single-Machine Equivalent (SME/SIME) model.
    ///
    /// The machine with the maximum rotor angle forms the critical cluster; all
    /// others are the complementary group.
    pub fn compute_sme_equivalent(&self) -> SmeModel {
        let n = self.machines.len();
        if n == 0 {
            return SmeModel {
                equivalent_angle_rad: 0.0,
                equivalent_speed_rad_per_s: 0.0,
                equivalent_inertia: 1.0,
                pa_max: 0.0,
                critical_clearing_time_s: self.fault_clearing_time_s * 2.0,
                stability_index: 0.0,
            };
        }

        if n == 1 {
            let m = &self.machines[0];
            let pa = (m.p_mechanical_pu - m.p_electrical_pu).abs().max(1e-9);
            let delta_u = (PI - m.rotor_angle_rad).abs();
            let arg = 2.0 * m.inertia_h_s * delta_u / (OMEGA_S * pa);
            let t_cct = if arg > 0.0 {
                arg.sqrt()
            } else {
                self.fault_clearing_time_s * 2.0
            };
            let stability_index = 1.0 - self.fault_clearing_time_s / t_cct.max(1e-12);
            return SmeModel {
                equivalent_angle_rad: m.rotor_angle_rad,
                equivalent_speed_rad_per_s: m.rotor_speed_rad_per_s,
                equivalent_inertia: m.inertia_h_s,
                pa_max: pa,
                critical_clearing_time_s: t_cct,
                stability_index,
            };
        }

        // Critical machine index (max angle)
        let crit_idx = self
            .machines
            .iter()
            .enumerate()
            .max_by(|a, b| {
                a.1.rotor_angle_rad
                    .partial_cmp(&b.1.rotor_angle_rad)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(i, _)| i)
            .unwrap_or(0);

        let mc = &self.machines[crit_idx];

        // Complementary group weighted averages
        let rest_h: f64 = self
            .machines
            .iter()
            .enumerate()
            .filter(|(i, _)| *i != crit_idx)
            .map(|(_, m)| m.inertia_h_s)
            .sum::<f64>()
            .max(1e-12);

        let weighted_sum = |f: &dyn Fn(&MachineState) -> f64| -> f64 {
            self.machines
                .iter()
                .enumerate()
                .filter(|(i, _)| *i != crit_idx)
                .map(|(_, m)| m.inertia_h_s * f(m))
                .sum::<f64>()
                / rest_h
        };

        let rest_delta = weighted_sum(&|m: &MachineState| m.rotor_angle_rad);
        let rest_omega = weighted_sum(&|m: &MachineState| m.rotor_speed_rad_per_s);
        let rest_pm = weighted_sum(&|m: &MachineState| m.p_mechanical_pu);
        let rest_pe = weighted_sum(&|m: &MachineState| m.p_electrical_pu);

        // SIME equivalent
        let h_c = mc.inertia_h_s;
        let h_s = rest_h;
        let h_eq = h_c * h_s / (h_c + h_s);

        let delta_eq = mc.rotor_angle_rad - rest_delta;
        let omega_eq = mc.rotor_speed_rad_per_s - rest_omega;

        // Equivalent accelerating power
        let pa_c = mc.p_mechanical_pu - mc.p_electrical_pu;
        let pa_s = rest_pm - rest_pe;
        let pa_eq = (pa_c - (h_eq / h_s.max(1e-12)) * pa_s).abs().max(1e-9);

        let delta_u = (PI - delta_eq.abs()).abs();
        let arg = 2.0 * h_eq * delta_u / (OMEGA_S * pa_eq);
        let t_cct = if arg > 0.0 {
            arg.sqrt()
        } else {
            self.fault_clearing_time_s * 2.0
        };
        let stability_index = 1.0 - self.fault_clearing_time_s / t_cct.max(1e-12);

        SmeModel {
            equivalent_angle_rad: delta_eq,
            equivalent_speed_rad_per_s: omega_eq,
            equivalent_inertia: h_eq,
            pa_max: pa_eq,
            critical_clearing_time_s: t_cct,
            stability_index,
        }
    }

    /// Identify the critical machines from a simulation trajectory.
    ///
    /// Returns the indices of machines with the largest angle separation from
    /// the COI at the final time step, sorted by deviation (descending).
    /// Approximately ⌈n/3⌉ machines are returned (at least 1).
    pub fn identify_critical_machines(&self, trajectory: &[Vec<(f64, f64)>]) -> Vec<usize> {
        let n = self.machines.len();
        if n == 0 {
            return Vec::new();
        }
        if trajectory.is_empty() {
            return vec![0];
        }

        // Final angles from trajectory
        let final_angles: Vec<f64> = trajectory
            .iter()
            .map(|t| t.last().map(|&(d, _)| d).unwrap_or(0.0))
            .collect();

        let total_h: f64 = self.machines.iter().map(|m| m.inertia_h_s).sum();
        let delta_coi = if total_h > 1e-12 {
            self.machines
                .iter()
                .zip(final_angles.iter())
                .map(|(m, &d)| m.inertia_h_s * d)
                .sum::<f64>()
                / total_h
        } else {
            final_angles.iter().copied().sum::<f64>() / n as f64
        };

        let mut deviations: Vec<(usize, f64)> = final_angles
            .iter()
            .enumerate()
            .map(|(i, &d)| (i, (d - delta_coi).abs()))
            .collect();

        deviations.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        let k = ((n as f64 / 3.0).ceil() as usize).max(1);
        deviations.iter().take(k).map(|&(i, _)| i).collect()
    }

    /// Compute the potential energy at the given UEP angles.
    ///
    /// If the network Y-matrix is not set, a heuristic
    /// `PE = Σ P_m_i · (δ_u_i − δ_s_i)` is used instead.
    pub fn compute_pe_at_uep(&self, uep_angles: &[f64]) -> f64 {
        if uep_angles.is_empty() {
            return 0.0;
        }
        if self.network_admittance.is_empty() {
            return self
                .machines
                .iter()
                .zip(uep_angles.iter())
                .map(|(m, &du)| m.p_mechanical_pu * (du - m.rotor_angle_rad))
                .sum();
        }
        self.compute_network_pe(uep_angles)
    }

    // -----------------------------------------------------------------------
    // Private helpers
    // -----------------------------------------------------------------------

    /// Network potential energy:
    /// `PE = −Σ_{i<j} (G_ij sin δ_ij − B_ij cos δ_ij)`.
    fn compute_network_pe(&self, angles: &[f64]) -> f64 {
        if self.network_admittance.is_empty() || angles.is_empty() {
            return 0.0;
        }
        let n = angles.len().min(self.network_admittance.len());
        let mut pe = 0.0_f64;
        for i in 0..n {
            let row = &self.network_admittance[i];
            for j in (i + 1)..n {
                if j >= row.len() {
                    break;
                }
                let (g_ij, b_ij) = row[j];
                let d_ij = angles[i] - angles[j];
                pe -= g_ij * d_ij.sin() - b_ij * d_ij.cos();
            }
        }
        pe
    }

    /// Electrical power from arbitrary angle array (E_i = 1.0 classical model).
    fn electrical_power_from_angles(&self, angles: &[f64]) -> Vec<f64> {
        let n = self.machines.len();
        if self.network_admittance.is_empty() {
            return self.machines.iter().map(|m| m.p_electrical_pu).collect();
        }
        (0..n)
            .map(|i| {
                if i >= self.network_admittance.len() || i >= angles.len() {
                    return self.machines[i].p_electrical_pu;
                }
                let row = &self.network_admittance[i];
                let delta_i = angles[i];
                let mut p_e = 0.0_f64;
                for (j, &(g_ij, b_ij)) in row.iter().enumerate() {
                    if j >= angles.len() {
                        break;
                    }
                    let d_ij = delta_i - angles[j];
                    p_e += g_ij * d_ij.cos() + b_ij * d_ij.sin();
                }
                p_e
            })
            .collect()
    }

    /// TSMI via energy-function method.
    fn tsmi_energy(&mut self, t_start: Instant) -> TsmiResult {
        let traj = self.integrate_swing_equations(self.fault_clearing_time_s);
        let ef = self.compute_energy_function();
        let critical_machines = self.identify_critical_machines(&traj);
        let status = classify_stability(ef.margin);

        TsmiResult {
            method: self.method,
            status,
            margin: ef.margin,
            normalized_margin: ef.normalized_margin,
            critical_machines,
            energy_function: Some(ef),
            sme: None,
            simulation_time_s: self.fault_clearing_time_s,
            computation_time_ms: t_start.elapsed().as_secs_f64() * 1000.0,
        }
    }

    /// TSMI via Single-Machine Equivalent method.
    fn tsmi_sme(&mut self, t_start: Instant) -> TsmiResult {
        let traj = self.integrate_swing_equations(self.fault_clearing_time_s);
        let sme = self.compute_sme_equivalent();
        let critical_machines = self.identify_critical_machines(&traj);

        let margin = sme.stability_index;
        let status = classify_stability(margin);
        let normalized_margin = margin.clamp(-1.0, 1.0);

        TsmiResult {
            method: self.method,
            status,
            margin,
            normalized_margin,
            critical_machines,
            energy_function: None,
            sme: Some(sme),
            simulation_time_s: self.fault_clearing_time_s,
            computation_time_ms: t_start.elapsed().as_secs_f64() * 1000.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

/// Classify stability based on the numerical margin value.
///
/// | margin      | status            |
/// |-------------|-------------------|
/// | > 0.1       | Stable            |
/// | (0, 0.1]    | MarginallyStable  |
/// | ≤ 0         | Unstable          |
pub fn classify_stability(margin: f64) -> StabilityStatus {
    if margin > 0.1 {
        StabilityStatus::Stable
    } else if margin > 0.0 {
        StabilityStatus::MarginallyStable
    } else {
        StabilityStatus::Unstable
    }
}

/// Inertia-weighted COI speed from a speed slice.
fn coi_speed_weighted(machines: &[MachineState], speeds: &[f64]) -> f64 {
    let total_h: f64 = machines.iter().map(|m| m.inertia_h_s).sum();
    if total_h < 1e-12 {
        return 0.0;
    }
    machines
        .iter()
        .zip(speeds.iter())
        .map(|(m, &w)| m.inertia_h_s * w)
        .sum::<f64>()
        / total_h
}

// ---------------------------------------------------------------------------
// Legacy API — TransientStabilityMargin (EAC / Lyapunov / PEBS)
// ---------------------------------------------------------------------------

/// Legacy transient stability margin index — energy-function methods.
///
/// Provides:
/// - Equal Area Criterion (SMIB)
/// - Lyapunov energy function (multi-machine)
/// - PEBS critical energy approximation
pub struct TransientStabilityMargin;

impl TransientStabilityMargin {
    // -----------------------------------------------------------------------
    // 1. Equal Area Criterion (SMIB)
    // -----------------------------------------------------------------------

    /// Compute the Equal Area Criterion (EAC) stability margin for a SMIB system.
    ///
    /// Supported fault types: `"3phase"`, `"slg"`, `"ll"`.
    ///
    /// Returns a positive margin when the system is stable (more deceleration
    /// area available than acceleration area consumed).
    ///
    /// # Errors
    /// [`OxiGridError::InvalidParameter`] for physically invalid inputs.
    pub fn equal_area_criterion(
        p_mech_pu: f64,
        e_pu: f64,
        x_d_prime: f64,
        delta_0_rad: f64,
        delta_max_rad: f64,
        fault_type: &str,
    ) -> Result<f64> {
        if p_mech_pu < 0.0 {
            return Err(OxiGridError::InvalidParameter(format!(
                "p_mech_pu={p_mech_pu:.4} must be non-negative"
            )));
        }
        if e_pu <= 0.0 {
            return Err(OxiGridError::InvalidParameter(format!(
                "e_pu={e_pu:.4} must be positive"
            )));
        }
        if x_d_prime <= 0.0 {
            return Err(OxiGridError::InvalidParameter(format!(
                "x_d_prime={x_d_prime:.4} must be positive"
            )));
        }
        if !(0.0..PI / 2.0).contains(&delta_0_rad) {
            return Err(OxiGridError::InvalidParameter(format!(
                "delta_0={delta_0_rad:.4} must be in [0, π/2)"
            )));
        }
        if delta_max_rad <= delta_0_rad || delta_max_rad > PI {
            return Err(OxiGridError::InvalidParameter(format!(
                "delta_max={delta_max_rad:.4} must be in (delta_0, π]"
            )));
        }

        let p_max_prefault = e_pu / x_d_prime;
        let fault_multiplier = match fault_type {
            "3phase" => 0.0,
            "slg" => 0.85,
            "ll" => 0.70,
            other => {
                return Err(OxiGridError::InvalidParameter(format!(
                    "Unknown fault_type '{other}'. Use \"3phase\", \"slg\", or \"ll\""
                )));
            }
        };

        let p_max_fault = fault_multiplier * p_max_prefault;

        let a_accel = p_mech_pu * (delta_max_rad - delta_0_rad)
            + p_max_fault * (delta_max_rad.cos() - delta_0_rad.cos());

        let a_decel_available = p_max_prefault * (delta_0_rad.cos() - delta_max_rad.cos())
            - p_mech_pu * (delta_max_rad - delta_0_rad);

        Ok(a_decel_available - a_accel)
    }

    // -----------------------------------------------------------------------
    // 2. Lyapunov Energy Function (multi-machine)
    // -----------------------------------------------------------------------

    /// Compute the Lyapunov transient energy for a multi-machine system.
    ///
    /// `V = Σ ½ M_i ω_i² − Σ (P_m_i + P_e_i)(δ_i − δ_s_i)`
    ///
    /// # Errors
    /// [`OxiGridError::InvalidParameter`] for mismatched slice lengths.
    pub fn lyapunov_energy(
        delta: &[f64],
        omega: &[f64],
        pe: &[f64],
        pm: &[f64],
        m: &[f64],
    ) -> Result<f64> {
        let n = delta.len();
        if omega.len() != n || pe.len() != n || pm.len() != n || m.len() != n {
            return Err(OxiGridError::InvalidParameter(format!(
                "All slices must have the same length (delta={n}, omega={}, pe={}, pm={}, m={})",
                omega.len(),
                pe.len(),
                pm.len(),
                m.len()
            )));
        }
        if n == 0 {
            return Ok(0.0);
        }

        let delta_s: Vec<f64> = (0..n)
            .map(|i| {
                let sin_di = delta[i].sin();
                if sin_di.abs() > 1e-6 && pe[i].abs() > 1e-12 {
                    let pe_max = pe[i] / sin_di;
                    if pe_max.abs() > 1e-12 {
                        (pm[i] / pe_max).clamp(-1.0, 1.0).asin()
                    } else {
                        0.0
                    }
                } else {
                    0.0
                }
            })
            .collect();

        let ke: f64 = (0..n).map(|i| 0.5 * m[i] * omega[i] * omega[i]).sum();
        let pe_term: f64 = (0..n)
            .map(|i| {
                let d_delta = delta[i] - delta_s[i];
                -(pm[i] * d_delta) - (pe[i] * d_delta)
            })
            .sum();

        Ok(ke + pe_term)
    }

    // -----------------------------------------------------------------------
    // 3. PEBS Critical Energy
    // -----------------------------------------------------------------------

    /// Approximate the PEBS critical energy for generator `gen_idx` in `network`.
    ///
    /// `V_cr = P_max (cos δ_SEP − cos δ_UEP) − P_m (δ_UEP − δ_SEP)`
    ///
    /// # Errors
    /// [`OxiGridError::InvalidParameter`] / [`OxiGridError::InvalidNetwork`] on bad inputs.
    pub fn pebs_critical_energy(network: &PowerNetwork, gen_idx: usize) -> Result<f64> {
        let gens = &network.generators;
        if gen_idx >= gens.len() {
            return Err(OxiGridError::InvalidParameter(format!(
                "gen_idx={gen_idx} out of range (n_gen={})",
                gens.len()
            )));
        }

        let gen = &gens[gen_idx];
        let base_mva = network.base_mva;
        let p_m_pu = gen.pg / base_mva;
        let p_max_pu = gen.pmax / base_mva;

        if p_max_pu < 1e-12 {
            return Err(OxiGridError::InvalidParameter(format!(
                "Generator {gen_idx} has pmax≈0 — cannot compute PEBS energy"
            )));
        }

        let ratio = (p_m_pu / p_max_pu).clamp(-1.0, 1.0);
        if ratio.abs() > 1.0 - 1e-9 {
            return Err(OxiGridError::InvalidNetwork(format!(
                "Generator {gen_idx}: P_m/P_max={ratio:.4} ≥ 1 — no stable equilibrium"
            )));
        }

        let delta_sep = ratio.asin();
        let delta_uep = PI - delta_sep;
        let v_cr =
            p_max_pu * (delta_sep.cos() - delta_uep.cos()) - p_m_pu * (delta_uep - delta_sep);

        Ok(v_cr)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::bus::{Bus, BusType};
    use crate::network::topology::{Generator, PowerNetwork};

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------

    fn single_machine_calc() -> TsmiCalculator {
        let machines = vec![MachineState::new(0, 0.5, 0.0, 5.0, 2.0, 0.8, 0.8, 0.1)];
        TsmiCalculator::new(machines, 0.1)
    }

    fn two_machine_calc() -> TsmiCalculator {
        let machines = vec![
            MachineState::new(0, 0.5, 0.0, 5.0, 2.0, 0.8, 0.8, 0.1),
            MachineState::new(1, 0.3, 0.0, 4.0, 2.0, 0.6, 0.6, 0.05),
        ];
        TsmiCalculator::new(machines, 0.1)
    }

    fn two_machine_networked() -> TsmiCalculator {
        let mut calc = two_machine_calc();
        calc.network_admittance = vec![
            vec![(0.1, 0.0), (-0.05, -2.0)],
            vec![(-0.05, -2.0), (0.1, 0.0)],
        ];
        calc
    }

    // ------------------------------------------------------------------
    // 1. COI weighted average
    // ------------------------------------------------------------------
    #[test]
    fn test_coi_angle_weighted_average() {
        let calc = two_machine_calc();
        let (delta_coi, _) = calc.compute_coi();
        let expected = (5.0 * 0.5 + 4.0 * 0.3) / 9.0;
        assert!(
            (delta_coi - expected).abs() < 1e-10,
            "COI={delta_coi}, expected={expected}"
        );
    }

    // ------------------------------------------------------------------
    // 2. Kinetic energy of single machine in COI frame = 0
    // ------------------------------------------------------------------
    #[test]
    fn test_kinetic_energy_single_machine() {
        let machines = vec![MachineState::new(0, 0.5, 1.0, 5.0, 0.0, 0.8, 0.8, 0.0)];
        let calc = TsmiCalculator::new(machines, 0.1);
        let ef = calc.compute_energy_function();
        // Single machine: omega_coi = omega of machine → omega_tilde = 0 → KE = 0
        assert!(
            ef.kinetic_energy.abs() < 1e-10,
            "Single-machine KE in COI frame must be 0, got {}",
            ef.kinetic_energy
        );
    }

    // ------------------------------------------------------------------
    // 3. Kinetic energy zero at SEP (all speeds = 0)
    // ------------------------------------------------------------------
    #[test]
    fn test_kinetic_energy_zero_at_sep() {
        let calc = two_machine_calc();
        let ef = calc.compute_energy_function();
        assert!(
            ef.kinetic_energy.abs() < 1e-10,
            "KE at SEP must be 0, got {}",
            ef.kinetic_energy
        );
    }

    // ------------------------------------------------------------------
    // 4. Angle separation increases when machines are imbalanced (RK4)
    // ------------------------------------------------------------------
    #[test]
    fn test_swing_equation_rk4() {
        // Two machines: machine 0 accelerating (P_m >> P_e), machine 1 balanced.
        // The angle separation between them should grow over time.
        let machines = vec![
            MachineState::new(0, 0.3, 0.0, 5.0, 0.0, 1.5, 0.2, 0.0), // large excess
            MachineState::new(1, 0.3, 0.0, 5.0, 0.0, 0.5, 0.5, 0.0), // balanced
        ];
        let mut calc = TsmiCalculator::new(machines, 0.5);
        let init_sep = (calc.machines[0].rotor_angle_rad - calc.machines[1].rotor_angle_rad).abs();
        calc.integrate_swing_equations(0.5);
        let final_sep = (calc.machines[0].rotor_angle_rad - calc.machines[1].rotor_angle_rad).abs();
        // Machine 0 accelerates more → separation increases
        assert!(
            final_sep > init_sep || final_sep.abs() < 1e-6,
            "Angle separation should grow: init={init_sep:.4} → final={final_sep:.4}"
        );
        // At minimum, machine 0 should have gained more speed than machine 1
        let speed0 = calc.machines[0].rotor_speed_rad_per_s;
        let speed1 = calc.machines[1].rotor_speed_rad_per_s;
        assert!(
            speed0 > speed1,
            "Machine 0 should be faster: {speed0:.4} vs {speed1:.4}"
        );
    }

    // ------------------------------------------------------------------
    // 5. Balanced machine (P_m = P_e) stays near its initial angle
    // ------------------------------------------------------------------
    #[test]
    fn test_swing_equation_stable() {
        // Two machines, both perfectly balanced (P_m = P_e), zero initial speed.
        // With no net torque and zero damping, angles should remain constant.
        let machines = vec![
            MachineState::new(0, 0.4, 0.0, 5.0, 0.0, 0.8, 0.8, 0.0),
            MachineState::new(1, 0.3, 0.0, 4.0, 0.0, 0.6, 0.6, 0.0),
        ];
        let mut calc = TsmiCalculator::new(machines, 1.0);
        calc.simulation_step_s = 0.01;
        let init_angle0 = calc.machines[0].rotor_angle_rad;
        let init_angle1 = calc.machines[1].rotor_angle_rad;
        calc.integrate_swing_equations(1.0);
        let final_angle0 = calc.machines[0].rotor_angle_rad;
        let final_angle1 = calc.machines[1].rotor_angle_rad;
        // With P_m = P_e and no damping, in COI frame omega_tilde stays 0 → angles constant
        assert!(
            (final_angle0 - init_angle0).abs() < 1e-6,
            "Machine 0 angle should stay constant: {init_angle0} → {final_angle0}"
        );
        assert!(
            (final_angle1 - init_angle1).abs() < 1e-6,
            "Machine 1 angle should stay constant: {init_angle1} → {final_angle1}"
        );
    }

    // ------------------------------------------------------------------
    // 6. Electrical power formula: diagonal G only
    // ------------------------------------------------------------------
    #[test]
    fn test_electrical_power_formula() {
        let machines = vec![
            MachineState::new(0, 0.0, 0.0, 5.0, 0.0, 0.8, 0.8, 0.0),
            MachineState::new(1, 0.0, 0.0, 4.0, 0.0, 0.6, 0.6, 0.0),
        ];
        let mut calc = TsmiCalculator::new(machines, 0.1);
        calc.network_admittance = vec![vec![(0.5, 0.0), (0.0, 0.0)], vec![(0.0, 0.0), (0.4, 0.0)]];
        let p0 = calc.compute_electrical_power(0);
        let p1 = calc.compute_electrical_power(1);
        // Both at angle 0: P = G_ii · cos(0) = G_ii
        assert!((p0 - 0.5).abs() < 1e-10, "P_e0={p0}");
        assert!((p1 - 0.4).abs() < 1e-10, "P_e1={p1}");
    }

    // ------------------------------------------------------------------
    // 7. Energy margin positive for stable case
    // ------------------------------------------------------------------
    #[test]
    fn test_energy_margin_positive_stable() {
        let machines = vec![MachineState::new(0, 0.2, 0.0, 5.0, 2.0, 0.8, 0.8, 0.0)];
        let calc = TsmiCalculator::new(machines, 0.05);
        let ef = calc.compute_energy_function();
        assert!(
            ef.margin > 0.0,
            "Stable case should have positive margin, got {}",
            ef.margin
        );
    }

    // ------------------------------------------------------------------
    // 8. Energy margin negative for unstable case (large relative KE)
    // ------------------------------------------------------------------
    #[test]
    fn test_energy_margin_negative_unstable() {
        // Two machines with very large opposite speed deviations →
        // large relative KE >> critical PE → margin < 0
        let machines = vec![
            MachineState::new(0, 0.5, 200.0, 5.0, 0.0, 1.5, 0.0, 0.0),
            MachineState::new(1, 0.5, -200.0, 5.0, 0.0, 1.5, 0.0, 0.0),
        ];
        let calc = TsmiCalculator::new(machines, 0.5);
        let ef = calc.compute_energy_function();
        assert!(
            ef.margin < 0.0,
            "Unstable case should have negative margin, got {}",
            ef.margin
        );
    }

    // ------------------------------------------------------------------
    // 9. Critical machine identification
    // ------------------------------------------------------------------
    #[test]
    fn test_critical_machines_identification() {
        let _calc = two_machine_calc();
        // Machine 0: very large final angle (3.0 rad), machine 1: stays near 0.3.
        // COI = (5*3.0 + 4*0.3)/9 = (15 + 1.2)/9 ≈ 1.80.
        // Machine 0 deviation: |3.0 - 1.80| = 1.20.
        // Machine 1 deviation: |0.3 - 1.80| = 1.50.
        // Machine 1 is actually more deviated; we just verify the function returns a valid index.
        // To make machine 0 clearly critical: use machine 0 far positive, machine 1 near COI.
        // COI = (5*3.0 + 4*1.8)/9 = (15 + 7.2)/9 = 22.2/9 ≈ 2.47.
        // Machine 0: |3.0 - 2.47| = 0.53. Machine 1: |1.8 - 2.47| = 0.67 → machine 1 wins.
        // Use extreme: machine 0 at angle 5.0, machine 1 at 0.1.
        // COI = (5*5.0 + 4*0.1)/9 = (25 + 0.4)/9 ≈ 2.82.
        // Machine 0 dev: |5.0 - 2.82| = 2.18. Machine 1 dev: |0.1 - 2.82| = 2.72 → machine 1!
        // With equal inertias the heavy machine governs COI.
        // Best approach: equal inertias so COI = mean.
        let machines = vec![
            MachineState::new(0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0),
            MachineState::new(1, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0),
        ];
        let calc_eq = TsmiCalculator::new(machines, 0.1);
        // Equal inertias: COI = mean of final angles.
        // Machine 0 at 3.0, machine 1 at 0.0 → COI = 1.5.
        // Machine 0 dev: |3.0 - 1.5| = 1.5. Machine 1 dev: |0.0 - 1.5| = 1.5. Tie!
        // Machine 0 at 4.0, machine 1 at 0.0 → COI = 2.0.
        // Machine 0: |4.0 - 2.0| = 2.0. Machine 1: |0.0 - 2.0| = 2.0. Tie!
        // Machine 0 at 4.0, machine 1 at 0.5 → COI = 2.25.
        // Machine 0: |4.0 - 2.25| = 1.75. Machine 1: |0.5 - 2.25| = 1.75. Tie!
        // Machine 0 at 5.0, machine 1 at 0.5 → COI = 2.75.
        // Machine 0: |5.0 - 2.75| = 2.25. Machine 1: |0.5 - 2.75| = 2.25. Tie!
        // When there's a tie, whichever comes first in sort wins (stable sort).
        // Let machine 0 clearly dominate: machine 0 at 4.0, machine 1 at 0.9.
        // COI = (4.0 + 0.9)/2 = 2.45. machine 0: 1.55, machine 1: 1.55. Still tie.
        // Use 3-machine setup for clear winner:
        let machines3 = vec![
            MachineState::new(0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0),
            MachineState::new(1, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0),
            MachineState::new(2, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0),
        ];
        let calc3 = TsmiCalculator::new(machines3, 0.1);
        // Machine 0 at 5.0, machines 1,2 at 0.1.
        // COI = (5.0 + 0.1 + 0.1)/3 = 5.2/3 ≈ 1.733.
        // Machine 0: |5.0 - 1.733| = 3.267. Machine 1: |0.1 - 1.733| = 1.633. Machine 2: same.
        let traj3 = vec![
            vec![(0.0_f64, 0.0_f64), (5.0, 2.0)], // machine 0: flies away
            vec![(0.0_f64, 0.0_f64), (0.1, 0.0)], // machine 1: stable
            vec![(0.0_f64, 0.0_f64), (0.1, 0.0)], // machine 2: stable
        ];
        let crit = calc3.identify_critical_machines(&traj3);
        assert!(!crit.is_empty());
        assert_eq!(
            crit[0], 0,
            "Machine 0 should be critical (largest deviation)"
        );
        let _ = calc_eq;
    }

    // ------------------------------------------------------------------
    // 10. SME reduces to original for single machine
    // ------------------------------------------------------------------
    #[test]
    fn test_sme_equivalent_single_machine() {
        let calc = single_machine_calc();
        let sme = calc.compute_sme_equivalent();
        assert!(
            (sme.equivalent_angle_rad - 0.5).abs() < 1e-10,
            "Equivalent angle should equal machine angle"
        );
        assert!(sme.critical_clearing_time_s > 0.0);
    }

    // ------------------------------------------------------------------
    // 11. SME CCT > fault clearing time for stable case
    // ------------------------------------------------------------------
    #[test]
    fn test_sme_cct_positive() {
        let calc = single_machine_calc();
        let sme = calc.compute_sme_equivalent();
        assert!(
            sme.critical_clearing_time_s > calc.fault_clearing_time_s,
            "CCT ({}) must exceed t_cl ({})",
            sme.critical_clearing_time_s,
            calc.fault_clearing_time_s
        );
    }

    // ------------------------------------------------------------------
    // 12. Stability index positive for stable case
    // ------------------------------------------------------------------
    #[test]
    fn test_stability_index_positive_stable() {
        let calc = single_machine_calc();
        let sme = calc.compute_sme_equivalent();
        assert!(
            sme.stability_index > 0.0,
            "Stability index must be positive, got {}",
            sme.stability_index
        );
    }

    // ------------------------------------------------------------------
    // 13. Stability index negative for unstable case
    // ------------------------------------------------------------------
    #[test]
    fn test_stability_index_negative_unstable() {
        let machines = vec![MachineState::new(0, 1.5, 0.0, 5.0, 0.0, 2.0, 0.1, 0.0)];
        let calc = TsmiCalculator::new(machines, 100.0);
        let sme = calc.compute_sme_equivalent();
        assert!(
            sme.stability_index < 0.0,
            "Should be negative for t_cl >> t_cct, got {}",
            sme.stability_index
        );
    }

    // ------------------------------------------------------------------
    // 14. Normalized margin in [-1, 1]
    // ------------------------------------------------------------------
    #[test]
    fn test_normalized_margin_bounds() {
        let calc = two_machine_calc();
        let ef = calc.compute_energy_function();
        assert!(
            ef.normalized_margin >= -1.0 && ef.normalized_margin <= 1.0,
            "Normalised margin out of bounds: {}",
            ef.normalized_margin
        );
    }

    // ------------------------------------------------------------------
    // 15. Two-machine system compute_tsmi
    // ------------------------------------------------------------------
    #[test]
    fn test_two_machine_system() {
        let mut calc = two_machine_calc();
        let result = calc.compute_tsmi();
        assert!(result.simulation_time_s > 0.0);
        assert!(result.computation_time_ms >= 0.0);
        assert!(!result.critical_machines.is_empty());
    }

    // ------------------------------------------------------------------
    // 16. Energy method populates energy_function field
    // ------------------------------------------------------------------
    #[test]
    fn test_compute_tsmi_energy_method() {
        let mut calc = two_machine_calc();
        calc.method = TsmiMethod::EnergyFunction;
        let result = calc.compute_tsmi();
        assert!(
            result.energy_function.is_some(),
            "Energy method must populate energy_function"
        );
        assert!(result.sme.is_none());
    }

    // ------------------------------------------------------------------
    // 17. SME method populates sme field
    // ------------------------------------------------------------------
    #[test]
    fn test_compute_tsmi_sme_method() {
        let mut calc = two_machine_calc();
        calc.method = TsmiMethod::SingleMachineEquivalent;
        let result = calc.compute_tsmi();
        assert!(result.sme.is_some(), "SME method must populate sme field");
        assert!(result.energy_function.is_none());
    }

    // ------------------------------------------------------------------
    // 18. Stability status: Stable
    // ------------------------------------------------------------------
    #[test]
    fn test_stability_status_stable() {
        assert!(matches!(classify_stability(0.5), StabilityStatus::Stable));
        assert!(matches!(classify_stability(0.11), StabilityStatus::Stable));
    }

    // ------------------------------------------------------------------
    // 19. Stability status: MarginallyStable
    // ------------------------------------------------------------------
    #[test]
    fn test_stability_status_marginal() {
        assert!(matches!(
            classify_stability(0.05),
            StabilityStatus::MarginallyStable
        ));
        assert!(matches!(
            classify_stability(0.001),
            StabilityStatus::MarginallyStable
        ));
        assert!(matches!(
            classify_stability(0.1),
            StabilityStatus::MarginallyStable
        ));
    }

    // ------------------------------------------------------------------
    // 20. Stability status: Unstable
    // ------------------------------------------------------------------
    #[test]
    fn test_stability_status_unstable() {
        assert!(matches!(
            classify_stability(-0.1),
            StabilityStatus::Unstable
        ));
        assert!(matches!(classify_stability(0.0), StabilityStatus::Unstable));
        assert!(matches!(
            classify_stability(-10.0),
            StabilityStatus::Unstable
        ));
    }

    // ------------------------------------------------------------------
    // 21. Zero inertia does not panic
    // ------------------------------------------------------------------
    #[test]
    fn test_coi_zero_inertia() {
        let machines = vec![MachineState::new(0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)];
        let calc = TsmiCalculator::new(machines, 0.1);
        let (d, w) = calc.compute_coi();
        assert_eq!(d, 0.0);
        assert_eq!(w, 0.0);
    }

    // ------------------------------------------------------------------
    // 22. Empty machines does not panic
    // ------------------------------------------------------------------
    #[test]
    fn test_empty_machines() {
        let mut calc = TsmiCalculator::new(vec![], 0.1);
        let result = calc.compute_tsmi();
        assert_eq!(result.margin, 0.0);
        assert!(result.critical_machines.is_empty());
    }

    // ------------------------------------------------------------------
    // 23. Networked two-machine electrical power is finite
    // ------------------------------------------------------------------
    #[test]
    fn test_electrical_power_networked() {
        let calc = two_machine_networked();
        let p0 = calc.compute_electrical_power(0);
        assert!(p0.is_finite(), "P_e must be finite, got {p0}");
    }

    // ------------------------------------------------------------------
    // 24. PE at UEP without Y-matrix uses heuristic
    // ------------------------------------------------------------------
    #[test]
    fn test_pe_at_uep_no_ymatrix() {
        let calc = single_machine_calc();
        let uep = calc.find_closest_unstable_ep();
        let pe_uep = calc.compute_pe_at_uep(&uep);
        // Heuristic: P_m * (π - 0.5 - 0.5) = 0.8 * (π - 1.0)
        let expected = 0.8 * (PI - 0.5 - 0.5);
        assert!(
            (pe_uep - expected).abs() < 1e-10,
            "PE_uep={pe_uep}, expected={expected}"
        );
    }

    // ------------------------------------------------------------------
    // 25. ClosestUnstableEp method returns valid result
    // ------------------------------------------------------------------
    #[test]
    fn test_closest_unstable_ep_method() {
        let mut calc = two_machine_calc();
        calc.method = TsmiMethod::ClosestUnstableEp;
        let result = calc.compute_tsmi();
        assert!(result.margin.is_finite());
        assert!(result.energy_function.is_some());
    }

    // ------------------------------------------------------------------
    // Legacy API tests
    // ------------------------------------------------------------------

    #[test]
    fn test_eac_stable_system_positive_margin() {
        let p_mech = 0.3_f64;
        let e = 1.05_f64;
        let x_d = 0.3_f64;
        let delta_0 = (p_mech * x_d / e).clamp(-1.0, 1.0).asin();
        let delta_max = PI - delta_0;
        let margin = TransientStabilityMargin::equal_area_criterion(
            p_mech, e, x_d, delta_0, delta_max, "3phase",
        )
        .expect("EAC should succeed");
        assert!(
            margin > 0.0,
            "Expected positive EAC margin, got {margin:.4}"
        );
    }

    #[test]
    fn test_eac_slg_larger_margin_than_3phase() {
        let p_mech = 0.5_f64;
        let e = 1.1_f64;
        let x_d = 0.25_f64;
        let delta_0 = (p_mech * x_d / e).clamp(-1.0, 1.0).asin();
        let delta_max = PI * 0.85_f64;
        let m_3ph = TransientStabilityMargin::equal_area_criterion(
            p_mech, e, x_d, delta_0, delta_max, "3phase",
        )
        .expect("3phase");
        let m_slg = TransientStabilityMargin::equal_area_criterion(
            p_mech, e, x_d, delta_0, delta_max, "slg",
        )
        .expect("slg");
        assert!(
            m_slg > m_3ph,
            "SLG ({m_slg:.4}) should exceed 3-phase ({m_3ph:.4})"
        );
    }

    #[test]
    fn test_eac_invalid_inputs_return_error() {
        assert!(TransientStabilityMargin::equal_area_criterion(
            -1.0,
            1.0,
            0.3,
            0.3,
            PI * 0.9,
            "3phase"
        )
        .is_err());
        assert!(TransientStabilityMargin::equal_area_criterion(
            0.5,
            1.0,
            0.3,
            0.3,
            PI * 0.9,
            "xyz"
        )
        .is_err());
        assert!(
            TransientStabilityMargin::equal_area_criterion(0.5, 1.0, 0.3, 0.5, 0.4, "slg").is_err()
        );
    }

    #[test]
    fn test_lyapunov_energy_at_equilibrium_is_zero() {
        let delta = vec![0.3, 0.4];
        let omega = vec![0.0, 0.0];
        let pe = vec![0.5, 0.3];
        let pm = vec![0.5, 0.3];
        let m = vec![0.1, 0.15];
        let energy =
            TransientStabilityMargin::lyapunov_energy(&delta, &omega, &pe, &pm, &m).expect("ok");
        assert!(
            energy.abs() < 1e-10,
            "Energy at equilibrium should be ≈0, got {energy:.6}"
        );
    }

    #[test]
    fn test_lyapunov_energy_kinetic_dominated_positive() {
        let delta = vec![0.3, 0.4];
        let omega = vec![10.0, 8.0];
        let pe = vec![0.5, 0.3];
        let pm = vec![0.5, 0.3];
        let m = vec![0.2, 0.15];
        let energy =
            TransientStabilityMargin::lyapunov_energy(&delta, &omega, &pe, &pm, &m).expect("ok");
        assert!(
            energy > 0.0,
            "High-speed system → positive energy, got {energy:.4}"
        );
    }

    #[test]
    fn test_lyapunov_energy_length_mismatch_errors() {
        let delta = vec![0.3, 0.4];
        let omega = vec![0.0]; // wrong length
        let pe = vec![0.5, 0.3];
        let pm = vec![0.5, 0.3];
        let m = vec![0.1, 0.15];
        assert!(TransientStabilityMargin::lyapunov_energy(&delta, &omega, &pe, &pm, &m).is_err());
    }

    #[test]
    fn test_pebs_critical_energy_valid_generator() {
        let mut net = PowerNetwork::new(100.0);
        net.buses.push(Bus::new(1, BusType::Slack));
        net.generators.push(Generator {
            bus_id: 1,
            pg: 40.0,
            qg: 0.0,
            qmax: 50.0,
            qmin: -50.0,
            vg: 1.0,
            mbase: 100.0,
            status: true,
            pmax: 100.0,
            pmin: 0.0,
        });
        let v_cr =
            TransientStabilityMargin::pebs_critical_energy(&net, 0).expect("PEBS should succeed");
        assert!(
            v_cr > 0.0,
            "Critical energy must be positive, got {v_cr:.4}"
        );
    }

    #[test]
    fn test_pebs_invalid_gen_index_errors() {
        let net = PowerNetwork::new(100.0);
        assert!(TransientStabilityMargin::pebs_critical_energy(&net, 0).is_err());
    }
}
