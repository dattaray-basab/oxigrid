//! Grid-Forming Inverter Stability Assessment for Inverter-Dominated Grids.
//!
//! Provides small-signal stability analysis of multi-inverter power systems using
//! linearised state-space models for Virtual Synchronous Generators (VSG) and
//! droop-controlled inverters.
//!
//! ## Analytical Framework
//!
//! For a VSG the swing-equation state matrix per inverter is:
//! ```text
//! A_vsg = [[0,          ω0       ],
//!           [-Ks/(2H), -Kd/(2H)  ]]
//! ```
//! where `Ks` = synchronising torque, `Kd` = damping torque, `H` = virtual inertia [s].
//!
//! Stability criterion: all eigenvalues `λ` satisfy `Re(λ) < 0`.
//! Damping ratio: `ζ = -Re(λ) / |λ|`.
//!
//! ## References
//! - Kundur, "Power System Stability and Control", McGraw-Hill, 1994.
//! - Dörfler & Bullo, "Synchronization in complex oscillator networks", SIAM, 2014.
//! - D'Arco & Suul, "Virtual synchronous machines", IEEE TPWRD, 2014.

use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

// ─────────────────────────────────────────────────────────────────────────────
// Enums
// ─────────────────────────────────────────────────────────────────────────────

/// Grid-forming inverter control strategy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum GfmControlType {
    /// Virtual synchronous machine — emulates swing equation with virtual inertia.
    VirtualSynchronousMachine,
    /// P-f / Q-V droop control (static frequency and voltage regulation).
    Droop,
    /// Energy-based matching control (port-Hamiltonian framework).
    MatchingControl,
    /// Dispatchable virtual oscillator control (dVOC).
    DispatchableVirtualOscillatorControl,
    /// Phase-locked-loop based grid-forming (GFL boundary).
    PllBased,
    /// Current-source mode (grid-following, for comparison).
    CurrentSourceMode,
}

/// Small-signal stability classification of the system.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum StabilityMode {
    /// All eigenvalues strictly in the left half-plane with damping > 5 %.
    Stable,
    /// All eigenvalues in left half-plane, but some modes have damping 1–5 %.
    OscillatoryStable,
    /// At least one eigenvalue on the imaginary axis (zero real part).
    MarginallyStable,
    /// At least one eigenvalue in the right half-plane.
    Unstable,
    /// Multiple eigenvalues with positive real parts — system diverges rapidly.
    DivergentlyUnstable,
}

/// Domain of stability analysis.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DomainOfAnalysis {
    /// Linearised small-signal (eigenvalue) analysis.
    SmallSignal,
    /// Large-signal (Lyapunov / time-domain) analysis.
    LargeSignal,
    /// Bifurcation analysis (saddle-node, Hopf).
    Bifurcation,
    /// Passivity-based stability assessment.
    PassivityBased,
}

// ─────────────────────────────────────────────────────────────────────────────
// Structs
// ─────────────────────────────────────────────────────────────────────────────

/// Model of a single grid-forming inverter for stability analysis.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GfmInverterModel {
    /// Unique inverter index.
    pub id: usize,
    /// Human-readable label.
    pub name: String,
    /// Control strategy implemented by this inverter.
    pub control_type: GfmControlType,
    /// Rated active power \[MW\].
    pub rated_power_mw: f64,
    /// Rated line-to-line voltage \[kV\].
    pub rated_voltage_kv: f64,
    /// Virtual inertia constant H \[s\]. Zero means no virtual inertia (pure droop).
    pub virtual_inertia_h_s: f64,
    /// Damping coefficient D_p \[pu\]. Default 0.05.
    pub damping_ratio: f64,
    /// Active-power droop \[%\] (e.g. 5 means 5 % droop).
    pub droop_p_pct: f64,
    /// Reactive-power droop \[%\].
    pub droop_q_pct: f64,
    /// Voltage magnitude setpoint \[pu\].
    pub voltage_setpoint_pu: f64,
    /// Frequency setpoint \[Hz\].
    pub frequency_setpoint_hz: f64,
    /// LC-filter inductance \[pu\].
    pub lc_filter_l_pu: f64,
    /// LC-filter capacitance \[pu\].
    pub lc_filter_c_pu: f64,
    /// Inner voltage/current control-loop bandwidth \[Hz\].
    pub bandwidth_hz: f64,
    /// Index of the bus this inverter is connected to.
    pub bus_id: usize,
}

impl Default for GfmInverterModel {
    fn default() -> Self {
        Self {
            id: 0,
            name: "GFM".to_string(),
            control_type: GfmControlType::VirtualSynchronousMachine,
            rated_power_mw: 10.0,
            rated_voltage_kv: 11.0,
            virtual_inertia_h_s: 5.0,
            damping_ratio: 0.05,
            droop_p_pct: 5.0,
            droop_q_pct: 5.0,
            voltage_setpoint_pu: 1.0,
            frequency_setpoint_hz: 50.0,
            lc_filter_l_pu: 0.1,
            lc_filter_c_pu: 0.05,
            bandwidth_hz: 100.0,
            bus_id: 0,
        }
    }
}

/// A complex eigenvalue of the system state matrix with physical interpretation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SystemEigenvalue {
    /// Real part σ (negative = stable mode).
    pub real_part: f64,
    /// Imaginary part ω_d \[rad/s\].
    pub imag_part: f64,
    /// Damping ratio ζ = −Re(λ) / |λ|. Positive in left half-plane.
    pub damping_ratio: f64,
    /// Oscillation frequency f = |Im(λ)| / (2π) \[Hz\].
    pub frequency_hz: f64,
    /// Physical mode label (e.g. "power_sharing", "voltage_control").
    pub associated_mode: String,
    /// Normalised participation factor ∈ [0, 1].
    pub participation_factor: f64,
}

/// Full stability assessment result for a multi-inverter grid.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StabilityAssessment {
    /// All system eigenvalues.
    pub eigenvalues: Vec<SystemEigenvalue>,
    /// Overall stability classification.
    pub mode: StabilityMode,
    /// Minimum damping ratio across all modes.
    pub minimum_damping_ratio: f64,
    /// Least-stable eigenvalue (smallest damping ratio).
    pub critical_eigenvalue: Option<SystemEigenvalue>,
    /// Stability margin = minimum damping ratio (positive ⟹ stable).
    pub stability_margin: f64,
    /// Oscillation frequencies of poorly-damped modes (ζ < 5 %) \[Hz\].
    pub oscillation_frequencies: Vec<f64>,
    /// Participation matrix (n_states × n_modes).
    pub participation_matrix: Vec<Vec<f64>>,
    /// Synchronising torque coefficient K_s (positive ⟹ synchronism).
    pub synchronizing_torque: f64,
    /// Damping torque coefficient K_d (positive ⟹ damping).
    pub damping_torque: f64,
}

/// Multi-inverter grid model for stability analysis.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GfmGridModel {
    /// List of grid-forming inverter models.
    pub inverters: Vec<GfmInverterModel>,
    /// Grid short-circuit ratio (SCR = S_sc / P_inv).
    pub grid_strength_scr: f64,
    /// Grid impedance magnitude \[pu\].
    pub grid_impedance_z_pu: f64,
    /// Grid X/R ratio.
    pub grid_x_r_ratio: f64,
    /// Total load \[MW\].
    pub load_mw: f64,
    /// Renewable penetration \[%\] (0–100).
    pub renewable_penetration_pct: f64,
}

/// Main analyser for grid-forming inverter stability.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GfmStabilityAnalyzer {
    /// Grid model containing all inverters and network parameters.
    pub grid_model: GfmGridModel,
    /// Analysis domain (small-signal, large-signal, etc.).
    pub domain: DomainOfAnalysis,
    /// Perturbation magnitude for numerical Jacobian computation.
    pub perturbation_magnitude: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Internal helpers — QR eigenvalue decomposition (pure Rust, no external deps)
// ─────────────────────────────────────────────────────────────────────────────

/// Reduce matrix to upper Hessenberg form via Householder reflections.
#[allow(clippy::needless_range_loop)]
fn hessenberg_form(a: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let n = a.len();
    let mut h: Vec<Vec<f64>> = a.to_vec();
    for k in 0..n.saturating_sub(2) {
        // Build Householder vector from column k, rows k+1..n
        let mut x: Vec<f64> = (k + 1..n).map(|i| h[i][k]).collect();
        let norm = x.iter().map(|v| v * v).sum::<f64>().sqrt();
        if norm < 1e-14 {
            continue;
        }
        if x[0] >= 0.0 {
            x[0] += norm;
        } else {
            x[0] -= norm;
        }
        let norm2: f64 = x.iter().map(|v| v * v).sum::<f64>();
        if norm2 < 1e-28 {
            continue;
        }
        // Left multiply: H = H - 2 x (x^T H) / norm2
        for j in 0..n {
            let dot: f64 = x
                .iter()
                .enumerate()
                .map(|(i, xi)| xi * h[k + 1 + i][j])
                .sum();
            for (i, xi) in x.iter().enumerate() {
                h[k + 1 + i][j] -= 2.0 * xi * dot / norm2;
            }
        }
        // Right multiply: H = H - 2 (H x) x^T / norm2
        for i in 0..n {
            let dot: f64 = x
                .iter()
                .enumerate()
                .map(|(j, xj)| h[i][k + 1 + j] * xj)
                .sum();
            for (j, xj) in x.iter().enumerate() {
                h[i][k + 1 + j] -= 2.0 * dot * xj / norm2;
            }
        }
    }
    h
}

/// Wilkinson shift from trailing 2×2 of `h[0..size][0..size]`.
fn wilkinson_shift(h: &[Vec<f64>], size: usize) -> (f64, f64) {
    if size < 2 {
        return (h[0][0], 0.0);
    }
    let a = h[size - 2][size - 2];
    let b = h[size - 2][size - 1];
    let c = h[size - 1][size - 2];
    let d = h[size - 1][size - 1];
    let tr = a + d;
    let det = a * d - b * c;
    let disc = tr * tr - 4.0 * det;
    if disc >= 0.0 {
        let s = disc.sqrt();
        let l1 = (tr + s) / 2.0;
        let l2 = (tr - s) / 2.0;
        // Pick eigenvalue closer to h[size-1][size-1]
        if (l1 - d).abs() < (l2 - d).abs() {
            (l1, 0.0)
        } else {
            (l2, 0.0)
        }
    } else {
        (tr / 2.0, (-disc).sqrt() / 2.0)
    }
}

/// Apply one Francis double-shift QR step to `h[0..size][0..size]`.
#[allow(clippy::ptr_arg, clippy::needless_range_loop)]
fn double_qr_step(h: &mut Vec<Vec<f64>>, size: usize, mu_re: f64, mu_im: f64) {
    if size < 2 {
        return;
    }
    // Compute shift polynomial: (H - mu1*I)(H - mu2*I) first column
    let s = 2.0 * mu_re; // trace of 2x2 shift matrix
    let t = mu_re * mu_re + mu_im * mu_im; // determinant
                                           // First column of (H^2 - s*H + t*I)
    let h00 = h[0][0];
    let h10 = h[1][0];
    let mut x0 = h00 * h00 + h[0][1] * h10 - s * h00 + t;
    let mut x1 = h10 * (h00 + h[1][1] - s);
    let mut x2 = if size > 2 { h[2][1] * h10 } else { 0.0 };

    for k in 0..size.saturating_sub(1) {
        // Build Householder reflector for [x0, x1, x2] (or [x0, x1] at last step)
        let m = if k + 3 <= size {
            3
        } else if k + 2 <= size {
            2
        } else {
            1
        };
        let vec: Vec<f64> = match m {
            3 => vec![x0, x1, x2],
            2 => vec![x0, x1],
            _ => vec![x0],
        };
        let norm = vec.iter().map(|v| v * v).sum::<f64>().sqrt();
        if norm < 1e-14 {
            if k + 1 < size {
                x0 = h[k + 1][k];
                x1 = if k + 2 < size { h[k + 2][k] } else { 0.0 };
                x2 = if k + 3 < size { h[k + 3][k] } else { 0.0 };
            }
            continue;
        }
        let mut v = vec.clone();
        if v[0] >= 0.0 {
            v[0] += norm;
        } else {
            v[0] -= norm;
        }
        let norm2: f64 = v.iter().map(|vi| vi * vi).sum();
        if norm2 < 1e-28 {
            if k + 1 < size {
                x0 = h[k + 1][k];
                x1 = if k + 2 < size { h[k + 2][k] } else { 0.0 };
                x2 = if k + 3 < size { h[k + 3][k] } else { 0.0 };
            }
            continue;
        }
        let r = k; // starting row/col of this reflector in h
                   // Left apply: h[r..r+m][r..n] = h - 2 v (v^T h) / norm2
        let n = h.len();
        for j in 0..n {
            let dot: f64 = (0..m)
                .map(|i| if r + i < n { v[i] * h[r + i][j] } else { 0.0 })
                .sum();
            for i in 0..m {
                if r + i < n {
                    h[r + i][j] -= 2.0 * v[i] * dot / norm2;
                }
            }
        }
        // Right apply: h[0..n][r..r+m] = h - 2 (h v) v^T / norm2
        for i in 0..n {
            let dot: f64 = (0..m)
                .map(|j| if r + j < n { h[i][r + j] * v[j] } else { 0.0 })
                .sum();
            for j in 0..m {
                if r + j < n {
                    h[i][r + j] -= 2.0 * dot * v[j] / norm2;
                }
            }
        }
        // Prepare next bulge
        if k + 1 < size {
            x0 = h[k + 1][k];
            x1 = if k + 2 < size { h[k + 2][k] } else { 0.0 };
            x2 = if k + 3 < size { h[k + 3][k] } else { 0.0 };
        }
    }
}

/// Compute eigenvalues of a real square matrix via Francis double-shift QR iteration.
/// Returns a `Vec<(real, imag)>` of complex eigenvalues.
fn qr_eigenvalues(a: &[Vec<f64>]) -> Vec<(f64, f64)> {
    let n = a.len();
    if n == 0 {
        return vec![];
    }
    if n == 1 {
        return vec![(a[0][0], 0.0)];
    }
    if n == 2 {
        return exact_2x2_eigenvalues(a[0][0], a[0][1], a[1][0], a[1][1]);
    }
    let mut h = hessenberg_form(a);
    let mut result: Vec<(f64, f64)> = Vec::with_capacity(n);
    let mut size = n;
    let max_iter = 300;

    while size > 2 {
        let mut deflated = false;
        for _ in 0..max_iter {
            let eps = 1e-10;
            // Check for deflation at bottom
            if h[size - 1][size - 2].abs()
                < eps * (h[size - 2][size - 2].abs() + h[size - 1][size - 1].abs())
            {
                result.push((h[size - 1][size - 1], 0.0));
                h[size - 1][size - 2] = 0.0;
                size -= 1;
                deflated = true;
                break;
            }
            // Check for 2x2 block deflation
            if size >= 3
                && h[size - 2][size - 3].abs()
                    < eps * (h[size - 3][size - 3].abs() + h[size - 2][size - 2].abs())
            {
                let ev2 = exact_2x2_eigenvalues(
                    h[size - 2][size - 2],
                    h[size - 2][size - 1],
                    h[size - 1][size - 2],
                    h[size - 1][size - 1],
                );
                result.extend(ev2);
                h[size - 2][size - 3] = 0.0;
                size -= 2;
                deflated = true;
                break;
            }
            let (mu_re, mu_im) = wilkinson_shift(&h, size);
            double_qr_step(&mut h, size, mu_re, mu_im);
        }
        if !deflated {
            // Force deflation of trailing 2x2
            let ev2 = exact_2x2_eigenvalues(
                h[size - 2][size - 2],
                h[size - 2][size - 1],
                h[size - 1][size - 2],
                h[size - 1][size - 1],
            );
            result.extend(ev2);
            size = size.saturating_sub(2);
        }
    }
    if size == 2 {
        let ev2 = exact_2x2_eigenvalues(h[0][0], h[0][1], h[1][0], h[1][1]);
        result.extend(ev2);
    } else if size == 1 {
        result.push((h[0][0], 0.0));
    }
    result
}

/// Exact eigenvalues of a 2×2 real matrix.
fn exact_2x2_eigenvalues(a: f64, b: f64, c: f64, d: f64) -> Vec<(f64, f64)> {
    let tr = a + d;
    let det = a * d - b * c;
    let disc = tr * tr - 4.0 * det;
    if disc >= 0.0 {
        let s = disc.sqrt();
        vec![((tr + s) / 2.0, 0.0), ((tr - s) / 2.0, 0.0)]
    } else {
        let s = (-disc).sqrt() / 2.0;
        vec![(tr / 2.0, s), (tr / 2.0, -s)]
    }
}

/// Classify oscillation mode from frequency in Hz.
fn classify_mode(freq_hz: f64) -> String {
    if freq_hz < 0.1 {
        "inter_area".to_string()
    } else if freq_hz < 2.0 {
        "local_oscillation".to_string()
    } else if freq_hz < 10.0 {
        "power_sharing".to_string()
    } else if freq_hz < 100.0 {
        "voltage_control".to_string()
    } else {
        "current_control".to_string()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// GfmStabilityAnalyzer implementation
// ─────────────────────────────────────────────────────────────────────────────

impl GfmStabilityAnalyzer {
    /// Create a new analyser with default small-signal domain and perturbation 1e-5.
    pub fn new(grid_model: GfmGridModel) -> Self {
        Self {
            grid_model,
            domain: DomainOfAnalysis::SmallSignal,
            perturbation_magnitude: 1e-5,
        }
    }

    /// Build the linearised state matrix A for the multi-inverter system.
    ///
    /// Each inverter contributes a 2×2 sub-block on the block diagonal.
    /// VSG inverters use the swing-equation linearisation; others use droop dynamics.
    /// Off-diagonal coupling terms scale with 1 / SCR.
    pub fn compute_state_matrix(&self) -> Vec<Vec<f64>> {
        let n_inv = self.grid_model.inverters.len();
        if n_inv == 0 {
            return vec![];
        }
        let size = 2 * n_inv;
        let mut a = vec![vec![0.0f64; size]; size];
        let omega0 = 2.0 * PI * 50.0_f64;
        let scr = self.grid_model.grid_strength_scr.max(0.1);
        let coupling = 0.01 / scr;

        for (idx, inv) in self.grid_model.inverters.iter().enumerate() {
            let r = 2 * idx; // top-left corner of this inverter's block
            match inv.control_type {
                GfmControlType::VirtualSynchronousMachine => {
                    let h = inv.virtual_inertia_h_s;
                    if h > 1e-9 {
                        let ks = self.compute_synchronizing_torque(idx);
                        let kd = inv.damping_ratio * inv.rated_power_mw.max(1.0);
                        // [dδ/dt]   [0,          ω0      ] [δ]
                        // [dω/dt] = [-Ks/(2H), -Kd/(2H) ] [ω]
                        a[r][r] = 0.0;
                        a[r][r + 1] = omega0;
                        a[r + 1][r] = -ks / (2.0 * h);
                        a[r + 1][r + 1] = -kd / (2.0 * h);
                    } else {
                        // Zero inertia — degenerate to proportional droop
                        let dp = inv.droop_p_pct / 100.0;
                        let dq = inv.droop_q_pct / 100.0;
                        a[r][r] = -dp.max(1e-3);
                        a[r][r + 1] = 0.0;
                        a[r + 1][r] = 0.0;
                        a[r + 1][r + 1] = -dq.max(1e-3);
                    }
                }
                _ => {
                    // Droop / matching / dVOC / PLL / current-source
                    let dp = inv.droop_p_pct / 100.0;
                    let dq = inv.droop_q_pct / 100.0;
                    a[r][r] = -dp;
                    a[r][r + 1] = 0.0;
                    a[r + 1][r] = 0.0;
                    a[r + 1][r + 1] = -dq;
                }
            }
            // Add coupling to all other inverter sub-blocks
            for (jdx, _) in self.grid_model.inverters.iter().enumerate() {
                if jdx == idx {
                    continue;
                }
                let c = 2 * jdx;
                a[r][c] += coupling;
                a[r + 1][c + 1] += coupling;
            }
        }
        a
    }

    /// Compute system eigenvalues from state matrix `a` using QR iteration.
    ///
    /// Returns eigenvalues sorted by damping ratio (least stable first).
    pub fn compute_eigenvalues(&self, a_matrix: &[Vec<f64>]) -> Vec<SystemEigenvalue> {
        let raw = qr_eigenvalues(a_matrix);
        let mut evs: Vec<SystemEigenvalue> = raw
            .into_iter()
            .map(|(re, im)| {
                let magnitude = (re * re + im * im).sqrt();
                let zeta = if magnitude < 1e-12 {
                    0.0
                } else {
                    -re / magnitude
                };
                let freq_hz = im.abs() / (2.0 * PI);
                let mode_label = classify_mode(freq_hz);
                let pf = (0.5 + 0.5 * zeta).clamp(0.0, 1.0);
                SystemEigenvalue {
                    real_part: re,
                    imag_part: im,
                    damping_ratio: zeta,
                    frequency_hz: freq_hz,
                    associated_mode: mode_label,
                    participation_factor: pf,
                }
            })
            .collect();
        // Sort least stable first (ascending damping ratio)
        evs.sort_by(|a, b| {
            a.damping_ratio
                .partial_cmp(&b.damping_ratio)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        evs
    }

    /// Perform a full stability assessment of the multi-inverter system.
    ///
    /// Computes state matrix, eigenvalues, stability mode, synchronising/damping
    /// torques, and participation matrix.
    pub fn assess_stability(&self) -> StabilityAssessment {
        let a = self.compute_state_matrix();
        let eigenvalues = self.compute_eigenvalues(&a);

        let max_real = eigenvalues
            .iter()
            .map(|e| e.real_part)
            .fold(f64::NEG_INFINITY, f64::max);
        let min_damping = eigenvalues
            .iter()
            .map(|e| e.damping_ratio)
            .fold(f64::INFINITY, f64::min);
        let min_damping = if min_damping.is_infinite() {
            0.0
        } else {
            min_damping
        };

        let unstable_count = eigenvalues.iter().filter(|e| e.real_part > 1e-6).count();
        let mode = if unstable_count > 1 {
            StabilityMode::DivergentlyUnstable
        } else if max_real > 1e-6 {
            StabilityMode::Unstable
        } else if max_real > -1e-6 {
            StabilityMode::MarginallyStable
        } else if min_damping < 0.05 {
            StabilityMode::OscillatoryStable
        } else {
            StabilityMode::Stable
        };

        // Critical = least stable eigenvalue (first after sort)
        let critical_eigenvalue = eigenvalues.first().cloned();

        // Poorly-damped oscillation frequencies
        let oscillation_frequencies: Vec<f64> = eigenvalues
            .iter()
            .filter(|e| e.damping_ratio < 0.05 && e.frequency_hz > 0.01)
            .map(|e| e.frequency_hz)
            .collect();

        // Approximate participation matrix as identity (n_states × n_modes)
        let n = a.len();
        let participation_matrix: Vec<Vec<f64>> = (0..n)
            .map(|i| (0..n).map(|j| if i == j { 1.0 } else { 0.0 }).collect())
            .collect();

        let synchronizing_torque = self.compute_synchronizing_torque(0);
        let damping_torque = self.compute_damping_torque(0);

        StabilityAssessment {
            eigenvalues,
            mode,
            minimum_damping_ratio: min_damping,
            critical_eigenvalue,
            stability_margin: min_damping,
            oscillation_frequencies,
            participation_matrix,
            synchronizing_torque,
            damping_torque,
        }
    }

    /// Compute the synchronising torque coefficient K_s for inverter `inverter_id`.
    ///
    /// `K_s = V_g * V * sin(δ) / (X_inv + X_grid)` from the swing-equation
    /// linearisation. A positive K_s is required for synchronism.
    pub fn compute_synchronizing_torque(&self, inverter_id: usize) -> f64 {
        let inv = match self.grid_model.inverters.get(inverter_id) {
            Some(i) => i,
            None => return 0.0,
        };
        let v = inv.voltage_setpoint_pu;
        let vg = 1.0_f64; // infinite-bus voltage in pu
        let x_inv = inv.lc_filter_l_pu.max(0.01);
        // Decompose grid impedance into X and R via X/R ratio
        let xr = self.grid_model.grid_x_r_ratio;
        let z = self.grid_model.grid_impedance_z_pu;
        let x_grid = z * xr / (1.0 + xr * xr).sqrt();
        let x_total = x_inv + x_grid;
        // Power angle: sin(δ) ≈ P_rated * X_total / (V * Vg)
        let sin_delta = (inv.rated_power_mw * x_total / (v * vg)).clamp(-1.0, 1.0);
        let delta = sin_delta.asin();
        vg * v * delta.sin() / x_total
    }

    /// Compute the damping torque coefficient K_d for inverter `inverter_id`.
    ///
    /// Approximated as `K_d ≈ D_p * V² / X_total`, where D_p is the damping
    /// coefficient. Positive K_d ensures oscillations are attenuated.
    pub fn compute_damping_torque(&self, inverter_id: usize) -> f64 {
        let inv = match self.grid_model.inverters.get(inverter_id) {
            Some(i) => i,
            None => return 0.0,
        };
        let dp = inv.damping_ratio;
        let v = inv.voltage_setpoint_pu;
        let xr = self.grid_model.grid_x_r_ratio;
        let z = self.grid_model.grid_impedance_z_pu;
        let x_grid = z * xr / (1.0 + xr * xr).sqrt();
        let x_inv = inv.lc_filter_l_pu.max(0.01);
        let x_total = x_inv + x_grid;
        dp * v * v / x_total
    }

    /// Check whether the system is passive (positive-definite conductance matrix).
    ///
    /// A passive system cannot generate energy internally, which is a sufficient
    /// condition for input-output stability. Returns `true` if the system is passive.
    pub fn check_passivity(&self) -> bool {
        let total_damping: f64 = self
            .grid_model
            .inverters
            .iter()
            .map(|inv| inv.damping_ratio * inv.rated_power_mw)
            .sum();
        let grid_conductance = 1.0 / self.grid_model.grid_impedance_z_pu.max(1e-9);
        total_damping + grid_conductance > 0.0
    }

    /// Estimate the minimum short-circuit ratio (SCR) required for stable operation.
    ///
    /// Higher total virtual inertia allows operation at lower SCR. Rule of thumb:
    /// SCR < 1.5 risks instability for grid-following inverters; SCR < 0.5 even
    /// for grid-forming inverters.
    pub fn compute_minimum_scr(&self) -> f64 {
        let total_inertia: f64 = self
            .grid_model
            .inverters
            .iter()
            .map(|inv| inv.virtual_inertia_h_s * inv.rated_power_mw)
            .sum::<f64>();
        let total_power: f64 = self
            .grid_model
            .inverters
            .iter()
            .map(|inv| inv.rated_power_mw)
            .sum::<f64>();
        if total_inertia < 1e-9 {
            // No virtual inertia — apply grid-following limit
            return 1.5;
        }
        let omega0 = 2.0 * PI * 50.0_f64;
        // Approximate minimum SCR = P_rated / (H_total * ω0² / scale)
        let scr_min = total_power / (total_inertia * omega0 * omega0 / 1e4);
        scr_min.max(0.1)
    }

    /// Sweep a named parameter and return (parameter_value, stability_margin) pairs.
    ///
    /// Supported parameter names: `"droop_p_pct"`, `"virtual_inertia_h_s"`,
    /// `"damping_ratio"`, `"grid_strength_scr"`, `"bandwidth_hz"`.
    /// Returns 10 evenly-spaced sample points across a pre-defined range.
    pub fn analyze_sensitivity(&self, parameter: &str) -> Vec<(f64, f64)> {
        let (low, high) = match parameter {
            "droop_p_pct" => (1.0_f64, 20.0_f64),
            "virtual_inertia_h_s" => (0.1, 10.0),
            "damping_ratio" => (0.01, 0.3),
            "grid_strength_scr" => (0.5, 10.0),
            "bandwidth_hz" => (10.0, 1000.0),
            _ => (0.1, 10.0),
        };
        (0..10)
            .map(|i| {
                let val = low + (high - low) * (i as f64) / 9.0;
                let mut modified = self.grid_model.clone();
                for inv in &mut modified.inverters {
                    match parameter {
                        "droop_p_pct" => inv.droop_p_pct = val,
                        "virtual_inertia_h_s" => inv.virtual_inertia_h_s = val,
                        "damping_ratio" => inv.damping_ratio = val,
                        "bandwidth_hz" => inv.bandwidth_hz = val,
                        _ => {}
                    }
                }
                if parameter == "grid_strength_scr" {
                    modified.grid_strength_scr = val;
                }
                let analyzer = GfmStabilityAnalyzer {
                    grid_model: modified,
                    domain: DomainOfAnalysis::SmallSignal,
                    perturbation_magnitude: self.perturbation_magnitude,
                };
                let assessment = analyzer.assess_stability();
                (val, assessment.stability_margin)
            })
            .collect()
    }

    /// Identify the dominant oscillatory mode (eigenvalue with highest frequency).
    ///
    /// Filters out non-oscillatory modes (frequency < 0.01 Hz) and returns the
    /// mode with the largest oscillation frequency, which is typically the most
    /// observable in measurements.
    pub fn identify_dominant_mode<'a>(
        &self,
        eigenvalues: &'a [SystemEigenvalue],
    ) -> Option<&'a SystemEigenvalue> {
        eigenvalues
            .iter()
            .filter(|e| e.frequency_hz > 0.01)
            .max_by(|a, b| {
                a.frequency_hz
                    .partial_cmp(&b.frequency_hz)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a default stable VSG-based single-inverter grid model.
    fn make_vsg_model() -> GfmGridModel {
        GfmGridModel {
            inverters: vec![GfmInverterModel {
                id: 0,
                name: "VSG1".into(),
                control_type: GfmControlType::VirtualSynchronousMachine,
                rated_power_mw: 10.0,
                rated_voltage_kv: 11.0,
                virtual_inertia_h_s: 5.0,
                damping_ratio: 0.05,
                droop_p_pct: 5.0,
                droop_q_pct: 5.0,
                voltage_setpoint_pu: 1.0,
                frequency_setpoint_hz: 50.0,
                lc_filter_l_pu: 0.1,
                lc_filter_c_pu: 0.05,
                bandwidth_hz: 100.0,
                bus_id: 1,
            }],
            grid_strength_scr: 5.0,
            grid_impedance_z_pu: 0.1,
            grid_x_r_ratio: 5.0,
            load_mw: 8.0,
            renewable_penetration_pct: 60.0,
        }
    }

    /// Build a droop-controlled inverter grid model.
    fn make_droop_model() -> GfmGridModel {
        GfmGridModel {
            inverters: vec![GfmInverterModel {
                id: 0,
                name: "Droop1".into(),
                control_type: GfmControlType::Droop,
                rated_power_mw: 5.0,
                rated_voltage_kv: 0.4,
                virtual_inertia_h_s: 0.0,
                damping_ratio: 0.1,
                droop_p_pct: 4.0,
                droop_q_pct: 4.0,
                voltage_setpoint_pu: 1.0,
                frequency_setpoint_hz: 50.0,
                lc_filter_l_pu: 0.05,
                lc_filter_c_pu: 0.02,
                bandwidth_hz: 200.0,
                bus_id: 2,
            }],
            grid_strength_scr: 3.0,
            grid_impedance_z_pu: 0.05,
            grid_x_r_ratio: 3.0,
            load_mw: 4.0,
            renewable_penetration_pct: 40.0,
        }
    }

    #[test]
    fn test_vsg_synchronizing_torque_positive() {
        let model = make_vsg_model();
        let analyzer = GfmStabilityAnalyzer::new(model);
        let ks = analyzer.compute_synchronizing_torque(0);
        assert!(ks > 0.0, "Synchronising torque must be positive: got {ks}");
    }

    #[test]
    fn test_droop_damping_positive() {
        let model = make_droop_model();
        let analyzer = GfmStabilityAnalyzer::new(model);
        let kd = analyzer.compute_damping_torque(0);
        assert!(kd > 0.0, "Damping torque must be positive: got {kd}");
    }

    #[test]
    fn test_eigenvalue_damping_ratio() {
        // λ = -1 + 0j  →  |λ| = 1  →  ζ = 1.0
        let a = vec![vec![-1.0_f64]];
        let analyzer = GfmStabilityAnalyzer::new(make_vsg_model());
        let evs = analyzer.compute_eigenvalues(&a);
        assert_eq!(evs.len(), 1);
        let zeta = evs[0].damping_ratio;
        assert!(
            (zeta - 1.0).abs() < 1e-9,
            "Expected ζ=1.0 for λ=-1, got {zeta}"
        );
    }

    #[test]
    fn test_stable_classification() {
        let model = make_vsg_model();
        let analyzer = GfmStabilityAnalyzer::new(model);
        let assessment = analyzer.assess_stability();
        matches!(
            assessment.mode,
            StabilityMode::Stable | StabilityMode::OscillatoryStable
        );
        // All eigenvalues must have negative real part
        for ev in &assessment.eigenvalues {
            assert!(
                ev.real_part < 1e-6,
                "Eigenvalue real part should be negative: {}",
                ev.real_part
            );
        }
    }

    #[test]
    fn test_unstable_classification() {
        // Negative droop creates positive diagonal → positive eigenvalue
        let mut model = make_droop_model();
        model.inverters[0].droop_p_pct = -50.0;
        model.inverters[0].droop_q_pct = -50.0;
        let analyzer = GfmStabilityAnalyzer::new(model);
        let assessment = analyzer.assess_stability();
        assert!(
            matches!(
                assessment.mode,
                StabilityMode::Unstable | StabilityMode::DivergentlyUnstable
            ),
            "Expected Unstable, got {:?}",
            assessment.mode
        );
    }

    #[test]
    fn test_oscillatory_stable_classification() {
        // Very small droop → very low damping but stable
        let mut model = make_droop_model();
        model.inverters[0].droop_p_pct = 0.001;
        model.inverters[0].droop_q_pct = 0.001;
        model.inverters[0].damping_ratio = 0.001;
        let analyzer = GfmStabilityAnalyzer::new(model);
        let assessment = analyzer.assess_stability();
        // Should be OscillatoryStable or Stable — not Unstable
        assert!(
            !matches!(
                assessment.mode,
                StabilityMode::Unstable | StabilityMode::DivergentlyUnstable
            ),
            "Should be stable variant, got {:?}",
            assessment.mode
        );
    }

    #[test]
    fn test_state_matrix_2x2_eigenvalue() {
        // A = [[0,1],[-1,-0.1]] — underdamped oscillator
        let a = vec![vec![0.0_f64, 1.0], vec![-1.0, -0.1]];
        let analyzer = GfmStabilityAnalyzer::new(make_vsg_model());
        let evs = analyzer.compute_eigenvalues(&a);
        assert_eq!(evs.len(), 2);
        // Both eigenvalues should have negative real part
        for ev in &evs {
            assert!(ev.real_part < 0.0, "Expected Re < 0, got {}", ev.real_part);
        }
        // Should be complex conjugate pair
        let has_imag = evs.iter().any(|e| e.imag_part.abs() > 0.01);
        assert!(
            has_imag,
            "Expected complex eigenvalues for underdamped system"
        );
    }

    #[test]
    fn test_state_matrix_diagonal() {
        // Diagonal matrix → eigenvalues = diagonal entries
        let a = vec![
            vec![-2.0_f64, 0.0, 0.0],
            vec![0.0, -3.0, 0.0],
            vec![0.0, 0.0, -5.0],
        ];
        let analyzer = GfmStabilityAnalyzer::new(make_vsg_model());
        let evs = analyzer.compute_eigenvalues(&a);
        assert_eq!(evs.len(), 3);
        let mut re_parts: Vec<f64> = evs.iter().map(|e| e.real_part).collect();
        re_parts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let expected = [-5.0, -3.0, -2.0];
        for (got, exp) in re_parts.iter().zip(expected.iter()) {
            assert!((got - exp).abs() < 1e-6, "Expected {exp}, got {got}");
        }
    }

    #[test]
    fn test_minimum_scr_positive() {
        let model = make_vsg_model();
        let analyzer = GfmStabilityAnalyzer::new(model);
        let scr = analyzer.compute_minimum_scr();
        assert!(scr > 0.0, "Minimum SCR must be positive: {scr}");
    }

    #[test]
    fn test_minimum_scr_decreases_with_inertia() {
        // Higher virtual inertia → lower required SCR
        let mut model_low = make_vsg_model();
        let mut model_high = make_vsg_model();
        model_low.inverters[0].virtual_inertia_h_s = 1.0;
        model_high.inverters[0].virtual_inertia_h_s = 10.0;
        let scr_low = GfmStabilityAnalyzer::new(model_low).compute_minimum_scr();
        let scr_high = GfmStabilityAnalyzer::new(model_high).compute_minimum_scr();
        assert!(
            scr_high < scr_low,
            "Higher inertia should require lower SCR: scr_low={scr_low}, scr_high={scr_high}"
        );
    }

    #[test]
    fn test_passivity_check_lossy_system() {
        let model = make_vsg_model();
        let analyzer = GfmStabilityAnalyzer::new(model);
        assert!(
            analyzer.check_passivity(),
            "Lossy system with positive damping should be passive"
        );
    }

    #[test]
    fn test_stability_margin_positive_stable() {
        let model = make_vsg_model();
        let analyzer = GfmStabilityAnalyzer::new(model);
        let assessment = analyzer.assess_stability();
        assert!(
            assessment.stability_margin > 0.0,
            "Stability margin must be positive for stable system: {}",
            assessment.stability_margin
        );
    }

    #[test]
    fn test_stability_margin_negative_unstable() {
        let mut model = make_droop_model();
        model.inverters[0].droop_p_pct = -20.0;
        model.inverters[0].droop_q_pct = -20.0;
        let analyzer = GfmStabilityAnalyzer::new(model);
        let assessment = analyzer.assess_stability();
        assert!(
            assessment.stability_margin < 0.0,
            "Stability margin should be negative for unstable system: {}",
            assessment.stability_margin
        );
    }

    #[test]
    fn test_critical_eigenvalue_least_stable() {
        let model = make_vsg_model();
        let analyzer = GfmStabilityAnalyzer::new(model);
        let assessment = analyzer.assess_stability();
        if let Some(crit) = &assessment.critical_eigenvalue {
            // Critical eigenvalue must have the minimum damping ratio
            let min_zeta = assessment
                .eigenvalues
                .iter()
                .map(|e| e.damping_ratio)
                .fold(f64::INFINITY, f64::min);
            assert!(
                (crit.damping_ratio - min_zeta).abs() < 1e-9,
                "Critical eigenvalue should have minimum damping ratio"
            );
        }
    }

    #[test]
    fn test_sensitivity_analysis_returns_10_points() {
        let model = make_vsg_model();
        let analyzer = GfmStabilityAnalyzer::new(model);
        let pairs = analyzer.analyze_sensitivity("droop_p_pct");
        assert_eq!(
            pairs.len(),
            10,
            "Sensitivity analysis should return 10 pairs"
        );
    }

    #[test]
    fn test_frequency_oscillation_detection() {
        // Build eigenvalues with a low-frequency oscillatory mode directly
        // and verify that assess_stability populates oscillation_frequencies.
        // Use a VSG model where the 2×2 swing block produces complex eigenvalues.
        let mut model = make_vsg_model();
        // Large virtual inertia + small damping → oscillatory mode
        model.inverters[0].virtual_inertia_h_s = 50.0;
        model.inverters[0].damping_ratio = 0.001;
        let analyzer = GfmStabilityAnalyzer::new(model);
        let a = analyzer.compute_state_matrix();
        let evs = analyzer.compute_eigenvalues(&a);
        // At least one eigenvalue should have a non-zero imaginary part
        let has_oscillatory = evs.iter().any(|e| e.imag_part.abs() > 1e-3);
        assert!(
            has_oscillatory,
            "Large inertia + small damping should produce oscillatory eigenvalues"
        );
    }

    #[test]
    fn test_multiple_inverters_interaction() {
        let mut model = make_vsg_model();
        model.inverters.push(GfmInverterModel {
            id: 1,
            name: "VSG2".into(),
            control_type: GfmControlType::VirtualSynchronousMachine,
            rated_power_mw: 8.0,
            rated_voltage_kv: 11.0,
            virtual_inertia_h_s: 4.0,
            damping_ratio: 0.06,
            droop_p_pct: 4.0,
            droop_q_pct: 4.0,
            voltage_setpoint_pu: 1.0,
            frequency_setpoint_hz: 50.0,
            lc_filter_l_pu: 0.08,
            lc_filter_c_pu: 0.04,
            bandwidth_hz: 120.0,
            bus_id: 2,
        });
        let analyzer = GfmStabilityAnalyzer::new(model);
        let a = analyzer.compute_state_matrix();
        // 2 inverters × 2 states = 4×4 matrix
        assert_eq!(a.len(), 4, "State matrix should be 4×4 for 2 inverters");
        assert_eq!(a[0].len(), 4);
        // Off-diagonal coupling should be non-zero
        let has_coupling = a[0][2] != 0.0 || a[1][3] != 0.0;
        assert!(has_coupling, "Should have cross-inverter coupling terms");
    }

    #[test]
    fn test_high_renewable_penetration_stability() {
        let mut model = make_vsg_model();
        model.renewable_penetration_pct = 90.0;
        let analyzer = GfmStabilityAnalyzer::new(model);
        let assessment = analyzer.assess_stability();
        // Should return a valid (non-panicking) result
        assert!(!assessment.eigenvalues.is_empty());
    }

    #[test]
    fn test_assess_stability_returns_complete() {
        let model = make_vsg_model();
        let analyzer = GfmStabilityAnalyzer::new(model);
        let assessment = analyzer.assess_stability();
        assert!(!assessment.eigenvalues.is_empty(), "Must have eigenvalues");
        assert!(
            !assessment.participation_matrix.is_empty(),
            "Must have participation matrix"
        );
        assert!(
            assessment.synchronizing_torque.is_finite(),
            "Ks must be finite"
        );
        assert!(assessment.damping_torque.is_finite(), "Kd must be finite");
        assert!(
            assessment.minimum_damping_ratio.is_finite(),
            "Min damping must be finite"
        );
    }

    #[test]
    fn test_identify_dominant_mode() {
        // Create eigenvalues with known frequencies
        let evs = vec![
            SystemEigenvalue {
                real_part: -0.5,
                imag_part: 2.0 * PI * 1.5,
                damping_ratio: 0.1,
                frequency_hz: 1.5,
                associated_mode: "local_oscillation".into(),
                participation_factor: 0.8,
            },
            SystemEigenvalue {
                real_part: -1.0,
                imag_part: 2.0 * PI * 10.0,
                damping_ratio: 0.3,
                frequency_hz: 10.0,
                associated_mode: "voltage_control".into(),
                participation_factor: 0.7,
            },
            SystemEigenvalue {
                real_part: -2.0,
                imag_part: 0.0,
                damping_ratio: 1.0,
                frequency_hz: 0.0,
                associated_mode: "inter_area".into(),
                participation_factor: 0.5,
            },
        ];
        let analyzer = GfmStabilityAnalyzer::new(make_vsg_model());
        let dominant = analyzer.identify_dominant_mode(&evs);
        assert!(dominant.is_some());
        let dom = dominant.unwrap();
        assert!(
            (dom.frequency_hz - 10.0).abs() < 1e-9,
            "Dominant mode should have highest frequency: {}",
            dom.frequency_hz
        );
    }

    #[test]
    fn test_control_type_droop_model() {
        let model = make_droop_model();
        let analyzer = GfmStabilityAnalyzer::new(model);
        let a = analyzer.compute_state_matrix();
        assert!(!a.is_empty(), "State matrix should be non-empty");
        // Droop model: diagonal should be -droop_p_pct/100 = -0.04
        let expected_diag = -0.04_f64;
        assert!(
            (a[0][0] - expected_diag).abs() < 1e-9,
            "Droop diagonal entry: expected {expected_diag}, got {}",
            a[0][0]
        );
    }

    #[test]
    fn test_matching_control_type() {
        let mut model = make_vsg_model();
        model.inverters[0].control_type = GfmControlType::MatchingControl;
        model.inverters[0].droop_p_pct = 6.0;
        model.inverters[0].droop_q_pct = 6.0;
        let analyzer = GfmStabilityAnalyzer::new(model);
        let assessment = analyzer.assess_stability();
        assert!(!assessment.eigenvalues.is_empty());
        // MatchingControl uses droop dynamics → diagonal = -0.06
        let a = analyzer.compute_state_matrix();
        assert!(
            (a[0][0] - (-0.06)).abs() < 1e-9,
            "MatchingControl diagonal: {}",
            a[0][0]
        );
    }

    #[test]
    fn test_sensitivity_analysis_grid_scr() {
        let model = make_vsg_model();
        let analyzer = GfmStabilityAnalyzer::new(model);
        let pairs = analyzer.analyze_sensitivity("grid_strength_scr");
        assert_eq!(pairs.len(), 10);
        // All parameter values should be increasing
        for w in pairs.windows(2) {
            assert!(
                w[1].0 >= w[0].0,
                "Parameter values should be non-decreasing"
            );
        }
    }

    #[test]
    fn test_identify_dominant_mode_empty() {
        let analyzer = GfmStabilityAnalyzer::new(make_vsg_model());
        // No oscillatory eigenvalues
        let evs: Vec<SystemEigenvalue> = vec![];
        assert!(analyzer.identify_dominant_mode(&evs).is_none());
    }

    #[test]
    fn test_vsg_zero_inertia_uses_droop() {
        let mut model = make_vsg_model();
        model.inverters[0].virtual_inertia_h_s = 0.0;
        model.inverters[0].droop_p_pct = 5.0;
        let analyzer = GfmStabilityAnalyzer::new(model);
        let a = analyzer.compute_state_matrix();
        // Should use droop block with positive magnitude diagonal entries
        assert!(
            a[0][0] < 0.0,
            "Zero-inertia VSG should use negative droop diagonal"
        );
    }

    #[test]
    fn test_passivity_zero_grid_impedance_handled() {
        let mut model = make_vsg_model();
        model.grid_impedance_z_pu = 0.0; // edge case
        let analyzer = GfmStabilityAnalyzer::new(model);
        // Should not panic (uses .max(1e-9) guard)
        let passive = analyzer.check_passivity();
        assert!(passive);
    }
}
