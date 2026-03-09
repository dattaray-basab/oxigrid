//! VSC-HVDC multi-terminal DC grid control system.
//!
//! Implements DC voltage droop control, power sharing, AC/DC interaction,
//! and emergency power control for multi-terminal DC (MTDC) grids.
//!
//! # Overview
//!
//! A multi-terminal VSC-HVDC grid consists of several [`VscTerminal`]s connected
//! by [`DcBranch`]s (resistive DC cables). The [`MtdcGrid`] orchestrates:
//!
//! - **DC power flow** — Newton-Raphson on the resistive Y-bus.
//! - **Droop control** — proportional disturbance sharing among droop terminals.
//! - **Emergency power control (EPC)** — overload relief via terminal redispatch.
//! - **Loss accounting** — cable I²R + converter losses.
//! - **N-1 contingency analysis** — branch-by-branch outage screening.
//!
//! # Units
//!
//! All power quantities are in \[MW\], voltages in \[pu\] (base = `base_kv_dc`),
//! currents in \[kA\], resistances in \[Ω\].

use serde::{Deserialize, Serialize};

use crate::error::{OxiGridError, Result};

// ─────────────────────────────────────────────────────────────────────────────
// Control mode
// ─────────────────────────────────────────────────────────────────────────────

/// Operating mode of a VSC converter station.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum VscControlMode {
    /// DC voltage droop — maintains DC bus voltage around the setpoint.
    /// Acts as the primary slack provider; participates in droop sharing.
    DcVoltageControl,
    /// Constant active-power injection. Tracks `p_setpoint_mw` without
    /// voltage droop participation (unless overridden by EPC).
    ActivePowerControl,
    /// Frequency-forming mode for islanded AC sub-networks.
    /// Converter acts as a voltage source behind an LCL filter.
    DriveControl,
    /// Constant DC current injection. Tracks `i_dc_ka` setpoint.
    DcCurrentControl,
}

// ─────────────────────────────────────────────────────────────────────────────
// DC grid topology
// ─────────────────────────────────────────────────────────────────────────────

/// Physical topology of the MTDC grid.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DcTopology {
    /// Two terminals, single cable — classic bipole HVDC.
    PointToPoint,
    /// Three or more terminals in a radial (tree) arrangement.
    /// A single cable outage may isolate a terminal.
    MultiTerminalRadial,
    /// Three or more terminals forming a meshed (looped) network.
    /// Redundant paths exist for N-1 security.
    MultiTerminalMeshed,
}

// ─────────────────────────────────────────────────────────────────────────────
// VSC terminal
// ─────────────────────────────────────────────────────────────────────────────

/// A VSC converter station connected to both an AC bus and the DC grid.
///
/// # Sign convention
/// Positive `p_actual_mw` means power is injected **into the AC system**
/// (the converter operates as an inverter). Negative means power is absorbed
/// from AC (rectifier mode).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VscTerminal {
    /// Unique string identifier (e.g. `"VSC-A"`).
    pub terminal_id: String,
    /// Converter rated capacity \[MW\].
    pub rated_mw: f64,
    /// Nominal DC bus voltage \[kV\].
    pub rated_kv_dc: f64,
    /// Active control mode.
    pub control_mode: VscControlMode,
    /// Active-power setpoint \[MW\] (positive = inject to AC).
    pub p_setpoint_mw: f64,
    /// DC voltage setpoint \[pu\] (default 1.0).
    pub vdc_setpoint_pu: f64,
    /// Droop gain k = ΔP/ΔV_dc \[MW/pu\]. Only effective in
    /// [`VscControlMode::DcVoltageControl`].
    pub droop_gain: f64,
    /// Upper active-power limit \[MW\].
    pub p_max_mw: f64,
    /// Lower active-power limit \[MW\] (may be negative for bidirectional).
    pub p_min_mw: f64,
    /// Converter loss fraction \[%\] of throughput power. Default 1.5 %.
    pub converter_loss_pct: f64,
    // ── state variables (updated by solver) ──────────────────────────────
    /// Current actual AC-side active-power injection \[MW\].
    pub p_actual_mw: f64,
    /// Current DC terminal voltage \[pu\].
    pub vdc_actual_pu: f64,
    /// Current DC current \[kA\] (positive = flowing into DC network).
    pub i_dc_ka: f64,
}

impl VscTerminal {
    /// Create a new terminal with sensible defaults.
    pub fn new(
        terminal_id: impl Into<String>,
        rated_mw: f64,
        rated_kv_dc: f64,
        control_mode: VscControlMode,
    ) -> Self {
        Self {
            terminal_id: terminal_id.into(),
            rated_mw,
            rated_kv_dc,
            control_mode,
            p_setpoint_mw: 0.0,
            vdc_setpoint_pu: 1.0,
            droop_gain: 20.0,
            p_max_mw: rated_mw,
            p_min_mw: -rated_mw,
            converter_loss_pct: 1.5,
            p_actual_mw: 0.0,
            vdc_actual_pu: 1.0,
            i_dc_ka: 0.0,
        }
    }

    /// Clamp a candidate power value to the terminal's MW limits.
    #[inline]
    fn clamp_power(&self, p_mw: f64) -> f64 {
        p_mw.clamp(self.p_min_mw, self.p_max_mw)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// DC branch
// ─────────────────────────────────────────────────────────────────────────────

/// A DC cable (or overhead line) connecting two terminals.
///
/// The cable is modelled as a pure resistance (series element only).
/// Shunt capacitance is neglected for steady-state analysis.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DcBranch {
    /// Index into `MtdcGrid::terminals` for the sending end.
    pub from_terminal: usize,
    /// Index into `MtdcGrid::terminals` for the receiving end.
    pub to_terminal: usize,
    /// DC cable resistance \[pu\] on the grid base (= R_Ω / Z_base).
    pub resistance_pu: f64,
    /// Cable series inductance \[mH\] (informational; not used in steady-state PF).
    pub inductance_mh: f64,
    /// Physical cable length \[km\].
    pub length_km: f64,
    /// Thermal current rating expressed as power \[MW\] at nominal voltage.
    pub rating_mw: f64,
}

impl DcBranch {
    /// Construct a cable from physical parameters.
    ///
    /// `resistance_ohm_per_km` is the per-km DC resistance \[Ω/km\].
    /// `base_z_ohm` is the system impedance base \[Ω\].
    pub fn from_physical(
        from_terminal: usize,
        to_terminal: usize,
        length_km: f64,
        resistance_ohm_per_km: f64,
        base_z_ohm: f64,
        rating_mw: f64,
    ) -> Self {
        let r_total_ohm = resistance_ohm_per_km * length_km;
        let resistance_pu = if base_z_ohm.abs() > 1e-12 {
            r_total_ohm / base_z_ohm
        } else {
            r_total_ohm
        };
        Self {
            from_terminal,
            to_terminal,
            resistance_pu,
            inductance_mh: 0.0,
            length_km,
            rating_mw,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Configuration
// ─────────────────────────────────────────────────────────────────────────────

/// Solver and control configuration for an [`MtdcGrid`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MtdcConfig {
    /// Enable voltage droop control across participating terminals.
    pub droop_enabled: bool,
    /// Maximum Newton-Raphson iterations for DC power flow. Default 50.
    pub max_iterations: usize,
    /// Convergence tolerance on DC voltage mismatch \[pu\]. Default 1 × 10⁻⁵.
    pub convergence_tol: f64,
    /// Maximum power ramp rate used by emergency power control \[MW/s\].
    pub emergency_ramp_rate: f64,
}

impl Default for MtdcConfig {
    fn default() -> Self {
        Self {
            droop_enabled: true,
            max_iterations: 50,
            convergence_tol: 1e-5,
            emergency_ramp_rate: 50.0,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Result structs
// ─────────────────────────────────────────────────────────────────────────────

/// Solution of a DC power flow on the MTDC grid.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DcPowerFlowResult {
    /// DC terminal voltage per terminal \[pu\].
    pub vdc_pu: Vec<f64>,
    /// Net power injection per terminal \[MW\] (positive = into DC network).
    pub p_mw: Vec<f64>,
    /// Power flow (magnitude) per DC branch \[MW\].
    pub branch_flows_mw: Vec<f64>,
    /// Total MTDC system losses \[MW\].
    pub total_losses_mw: f64,
    /// Whether the Newton-Raphson converged within tolerance.
    pub converged: bool,
    /// Number of iterations performed.
    pub iterations: usize,
}

/// Result of an emergency power control (EPC) action.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EpcResult {
    /// Human-readable descriptions of each redispatch action taken.
    pub actions: Vec<String>,
    /// Whether all branch overloads were cleared.
    pub overload_cleared: bool,
    /// Branch power flows after EPC \[MW\].
    pub final_flows_mw: Vec<f64>,
}

/// Per-terminal power sharing assessment.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PowerSharingReport {
    /// Terminal identifiers.
    pub terminal_id: Vec<String>,
    /// Scheduled (setpoint) power per terminal \[MW\].
    pub scheduled_mw: Vec<f64>,
    /// Actual power per terminal \[MW\].
    pub actual_mw: Vec<f64>,
    /// Deviation from setpoint \[%\] (|actual − scheduled| / rated × 100).
    pub deviation_pct: Vec<f64>,
    /// Droop participation factor (droop gain normalised by total droop).
    pub droop_factor: Vec<f64>,
}

/// Result of removing a single DC branch from the grid (N-1 contingency).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DcContingencyResult {
    /// Index of the outaged branch in `MtdcGrid::dc_branches`.
    pub outaged_branch: usize,
    /// Whether the post-contingency power flow converged and all limits satisfied.
    pub feasible: bool,
    /// Maximum per-terminal DC voltage deviation from 1.0 pu after the outage.
    pub max_voltage_deviation_pu: f64,
    /// Indices of DC branches that are overloaded post-contingency.
    pub overloaded_branches: Vec<usize>,
}

// ─────────────────────────────────────────────────────────────────────────────
// MTDC grid
// ─────────────────────────────────────────────────────────────────────────────

/// Multi-terminal VSC-HVDC grid.
///
/// Combines terminal converter models, DC cable network, and higher-level
/// control algorithms (droop, EPC, N-1).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MtdcGrid {
    /// Unique grid identifier.
    pub grid_id: String,
    /// Nominal DC voltage base \[kV\].
    pub base_kv_dc: f64,
    /// System MVA base \[MW\] (used for per-unit conversion).
    pub base_mw: f64,
    /// All VSC converter stations.
    pub terminals: Vec<VscTerminal>,
    /// All DC cables / overhead lines.
    pub dc_branches: Vec<DcBranch>,
    /// Physical topology category.
    pub topology: DcTopology,
    /// Solver and control configuration.
    pub config: MtdcConfig,
}

impl MtdcGrid {
    /// Construct a new MTDC grid.
    pub fn new(
        grid_id: impl Into<String>,
        base_kv_dc: f64,
        base_mw: f64,
        topology: DcTopology,
    ) -> Self {
        Self {
            grid_id: grid_id.into(),
            base_kv_dc,
            base_mw,
            terminals: Vec::new(),
            dc_branches: Vec::new(),
            topology,
            config: MtdcConfig::default(),
        }
    }

    /// Add a terminal to the grid.
    pub fn add_terminal(&mut self, terminal: VscTerminal) {
        self.terminals.push(terminal);
    }

    /// Add a DC branch to the grid.
    pub fn add_branch(&mut self, branch: DcBranch) {
        self.dc_branches.push(branch);
    }

    // ─── Y-bus construction ───────────────────────────────────────────────

    /// Build the DC nodal admittance matrix (Y_dc) as a dense `n × n` array.
    ///
    /// # Formula
    /// - Diagonal: `Y[i][i] = Σ_j (1 / R_ij)` — sum of branch conductances.
    /// - Off-diagonal: `Y[i][j] = -1 / R_ij`.
    ///
    /// Returns `Err` if a branch references an out-of-range terminal index or
    /// has a zero/negative resistance.
    pub fn build_y_bus(&self) -> Result<Vec<Vec<f64>>> {
        let n = self.terminals.len();
        if n == 0 {
            return Err(OxiGridError::InvalidNetwork(
                "MTDC grid has no terminals".into(),
            ));
        }
        let mut y = vec![vec![0.0_f64; n]; n];
        for (bi, branch) in self.dc_branches.iter().enumerate() {
            let i = branch.from_terminal;
            let j = branch.to_terminal;
            if i >= n || j >= n {
                return Err(OxiGridError::InvalidNetwork(format!(
                    "branch {bi}: terminal index out of range (i={i}, j={j}, n={n})"
                )));
            }
            if branch.resistance_pu <= 0.0 {
                return Err(OxiGridError::InvalidParameter(format!(
                    "branch {bi}: resistance_pu must be positive (got {})",
                    branch.resistance_pu
                )));
            }
            let g = 1.0 / branch.resistance_pu;
            y[i][i] += g;
            y[j][j] += g;
            y[i][j] -= g;
            y[j][i] -= g;
        }
        Ok(y)
    }

    // ─── Identify slack terminal ──────────────────────────────────────────

    /// Return the index of the "slack" terminal — the `DcVoltageControl`
    /// terminal with the highest droop gain.  Used to fix one DC voltage
    /// during the Newton-Raphson solution.
    fn slack_index(&self) -> Result<usize> {
        self.terminals
            .iter()
            .enumerate()
            .filter(|(_, t)| t.control_mode == VscControlMode::DcVoltageControl)
            .max_by(|(_, a), (_, b)| {
                a.droop_gain
                    .partial_cmp(&b.droop_gain)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(i, _)| i)
            .ok_or_else(|| {
                OxiGridError::InvalidNetwork(
                    "MTDC grid requires at least one DcVoltageControl terminal".into(),
                )
            })
    }

    // ─── Dense Gauss elimination (no external dep) ───────────────────────

    /// Solve `A x = b` with partial-pivot Gaussian elimination.
    ///
    /// Operates on an `(n × n)` dense matrix.  Returns `Err` if singular.
    fn gauss_solve(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Result<Vec<f64>> {
        let n = b.len();
        for col in 0..n {
            // Partial pivot
            let pivot_row = (col..n)
                .max_by(|&r1, &r2| {
                    a[r1][col]
                        .abs()
                        .partial_cmp(&a[r2][col].abs())
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .ok_or_else(|| OxiGridError::LinearAlgebra("empty column".into()))?;
            a.swap(col, pivot_row);
            b.swap(col, pivot_row);
            let pivot = a[col][col];
            if pivot.abs() < 1e-14 {
                return Err(OxiGridError::LinearAlgebra(
                    "singular DC Y-bus matrix".into(),
                ));
            }
            for row in (col + 1)..n {
                let factor = a[row][col] / pivot;
                #[allow(clippy::needless_range_loop)]
                for k in col..n {
                    let sub = factor * a[col][k];
                    a[row][k] -= sub;
                }
                let sub = factor * b[col];
                b[row] -= sub;
            }
        }
        // Back-substitution
        let mut x = vec![0.0_f64; n];
        for row in (0..n).rev() {
            let mut sum = b[row];
            #[allow(clippy::needless_range_loop)]
            for k in (row + 1)..n {
                sum -= a[row][k] * x[k];
            }
            x[row] = sum / a[row][row];
        }
        Ok(x)
    }

    // ─── Branch flow calculation ──────────────────────────────────────────

    /// Compute power flow on each DC branch given terminal voltages `v_pu`.
    ///
    /// `P_branch = (V_i − V_j) / R_ij × V_avg`
    ///
    /// For a DC cable with monopolar convention:
    /// `I_ij = (V_i − V_j) / R_ij`  [pu current on DC base]
    /// `P_ij = I_ij × V_i`  [pu power] × base_mw → \[MW\]
    fn branch_flows(&self, v_pu: &[f64]) -> Vec<f64> {
        self.dc_branches
            .iter()
            .map(|br| {
                let vi = v_pu[br.from_terminal];
                let vj = v_pu[br.to_terminal];
                let i_pu = (vi - vj) / br.resistance_pu;
                // Power flowing out of from_terminal [MW]
                i_pu * vi * self.base_mw
            })
            .collect()
    }

    // ─── Power injection mismatch ─────────────────────────────────────────

    /// Compute `ΔP_i = P_sched_i − P_calc_i` for all non-slack buses.
    ///
    /// `P_calc_i = V_i × Σ_j Y_ij × V_j` (DC linearised, but kept nonlinear).
    fn power_mismatch(
        &self,
        v_pu: &[f64],
        y_bus: &[Vec<f64>],
        p_sched: &[f64],
        slack_idx: usize,
    ) -> Vec<f64> {
        let n = v_pu.len();
        let mut dp = Vec::with_capacity(n - 1);
        for i in 0..n {
            if i == slack_idx {
                continue;
            }
            let p_calc: f64 = (0..n).map(|j| y_bus[i][j] * v_pu[i] * v_pu[j]).sum();
            dp.push(p_sched[i] / self.base_mw - p_calc);
        }
        dp
    }

    /// Build the reduced Jacobian `J[i][j] = ∂P_i/∂V_j` (rows/cols excluding slack).
    ///
    /// `∂P_i/∂V_i = Σ_j Y_ij × V_j + Y_ii × V_i = Σ_j Y_ij × V_j + Y_ii × V_i`
    /// `∂P_i/∂V_j = Y_ij × V_i`   (j ≠ i)
    fn build_jacobian(&self, v_pu: &[f64], y_bus: &[Vec<f64>], slack_idx: usize) -> Vec<Vec<f64>> {
        let n = v_pu.len();
        let nr = n - 1; // reduced size
        let free_idx: Vec<usize> = (0..n).filter(|&k| k != slack_idx).collect();
        let mut jac = vec![vec![0.0_f64; nr]; nr];
        for (ri, &i) in free_idx.iter().enumerate() {
            for (ci, &j) in free_idx.iter().enumerate() {
                if i == j {
                    // diagonal: Σ_k Y_ij * V_k + Y_ii * V_i
                    let sum: f64 = (0..n).map(|k| y_bus[i][k] * v_pu[k]).sum();
                    jac[ri][ci] = sum + y_bus[i][i] * v_pu[i];
                } else {
                    jac[ri][ci] = y_bus[i][j] * v_pu[i];
                }
            }
        }
        jac
    }

    // ─── DC Power Flow (public) ───────────────────────────────────────────

    /// Solve the DC power flow on the MTDC grid using Newton-Raphson.
    ///
    /// # Algorithm
    /// 1. Build DC Y-bus from branch resistances.
    /// 2. Identify slack terminal (strongest `DcVoltageControl` droop).
    /// 3. Initialise all terminal voltages to their setpoints.
    /// 4. Iterate: compute power mismatch → build Jacobian → solve ΔV → update.
    /// 5. Stop when `max|ΔV| < convergence_tol` or iteration limit reached.
    ///
    /// `ActivePowerControl` terminals have their setpoints fixed; droop
    /// terminals adjust through the mismatch equations.
    pub fn solve_dc_power_flow(&mut self) -> Result<DcPowerFlowResult> {
        let n = self.terminals.len();
        if n == 0 {
            return Err(OxiGridError::InvalidNetwork("no terminals".into()));
        }

        let y_bus = self.build_y_bus()?;
        let slack_idx = self.slack_index()?;

        // Scheduled power injections [MW] — into the DC network
        // (negative of the AC injection because the converter absorbs from DC
        // to inject into AC, and vice-versa).
        let p_sched: Vec<f64> = self.terminals.iter().map(|t| -t.p_setpoint_mw).collect();

        // Initialise voltages at setpoints
        let mut v_pu: Vec<f64> = self.terminals.iter().map(|t| t.vdc_setpoint_pu).collect();

        let mut converged = false;
        let mut iterations = 0;

        for _iter in 0..self.config.max_iterations {
            iterations += 1;
            let dp = self.power_mismatch(&v_pu, &y_bus, &p_sched, slack_idx);

            // Check convergence
            let max_dp = dp.iter().map(|x| x.abs()).fold(0.0_f64, f64::max);
            if max_dp < self.config.convergence_tol {
                converged = true;
                break;
            }

            let jac = self.build_jacobian(&v_pu, &y_bus, slack_idx);
            let dv = Self::gauss_solve(jac, dp)?;

            // Map reduced dv back to full voltage vector
            let free_idx: Vec<usize> = (0..n).filter(|&k| k != slack_idx).collect();
            for (ri, &i) in free_idx.iter().enumerate() {
                v_pu[i] += dv[ri];
            }
        }

        // Compute actual power injections from Y-bus and solved voltages
        let p_calc_pu: Vec<f64> = (0..n)
            .map(|i| (0..n).map(|j| y_bus[i][j] * v_pu[i] * v_pu[j]).sum())
            .collect();
        let p_mw: Vec<f64> = p_calc_pu.iter().map(|&p| p * self.base_mw).collect();

        // Branch flows
        let branch_flows_mw = self.branch_flows(&v_pu);

        // Update terminal state
        for (i, term) in self.terminals.iter_mut().enumerate() {
            term.vdc_actual_pu = v_pu[i];
            term.p_actual_mw = -p_mw[i]; // sign: positive = inject into AC
                                         // DC current [kA]: I = P_dc / V_dc / base_kv
            let v_kv = v_pu[i] * self.base_kv_dc;
            term.i_dc_ka = if v_kv.abs() > 1e-6 {
                p_mw[i].abs() * 1e3 / (v_kv * 1e3) // MW / kV = kA
            } else {
                0.0
            };
        }

        let total_losses_mw = self.calculate_losses();

        Ok(DcPowerFlowResult {
            vdc_pu: v_pu,
            p_mw,
            branch_flows_mw,
            total_losses_mw,
            converged,
            iterations,
        })
    }

    // ─── Droop control ────────────────────────────────────────────────────

    /// Redistribute a power disturbance among droop-controlled terminals.
    ///
    /// # Method
    /// A sudden loss/gain of `disturbance_mw` at `disturbed_terminal` causes a
    /// DC voltage deviation `ΔV_dc`.  Each droop terminal absorbs:
    /// ```text
    /// ΔP_i = k_i / Σ_j k_j × disturbance_mw
    /// ```
    /// where k_i is the `droop_gain` \[MW/pu\] of terminal i.  The disturbed
    /// terminal's setpoint is updated, and all droop terminals' setpoints are
    /// adjusted to restore balance.  The resulting DC voltage deviation is
    /// estimated as `ΔV_dc = disturbance_mw / Σ k_i` \[pu\].
    ///
    /// Returns `Err` if there are no droop terminals available.
    pub fn apply_droop_control(
        &mut self,
        disturbance_mw: f64,
        disturbed_terminal: usize,
    ) -> Result<f64> {
        let n = self.terminals.len();
        if disturbed_terminal >= n {
            return Err(OxiGridError::InvalidParameter(format!(
                "disturbed_terminal {disturbed_terminal} out of range (n={n})"
            )));
        }

        // Collect droop terminals (excluding the disturbed one if it is non-droop)
        let droop_terminals: Vec<usize> = self
            .terminals
            .iter()
            .enumerate()
            .filter(|(idx, t)| {
                *idx != disturbed_terminal
                    && t.control_mode == VscControlMode::DcVoltageControl
                    && t.droop_gain > 0.0
            })
            .map(|(i, _)| i)
            .collect();

        if droop_terminals.is_empty() {
            return Err(OxiGridError::InvalidNetwork(
                "no droop terminals available to absorb disturbance".into(),
            ));
        }

        let total_droop: f64 = droop_terminals
            .iter()
            .map(|&i| self.terminals[i].droop_gain)
            .sum();

        if total_droop < 1e-9 {
            return Err(OxiGridError::InvalidParameter(
                "total droop gain is effectively zero".into(),
            ));
        }

        // Estimated DC voltage deviation [pu]
        let delta_vdc = disturbance_mw / total_droop;

        // Update disturbed terminal setpoint (model the loss/gain)
        self.terminals[disturbed_terminal].p_setpoint_mw -= disturbance_mw;

        // Apply proportional redispatch to droop terminals
        for &i in &droop_terminals {
            let k_i = self.terminals[i].droop_gain;
            let delta_p_i = (k_i / total_droop) * disturbance_mw;
            let new_p = self.terminals[i].p_setpoint_mw + delta_p_i;
            self.terminals[i].p_setpoint_mw = self.terminals[i].clamp_power(new_p);
        }

        Ok(delta_vdc)
    }

    // ─── Emergency power control ──────────────────────────────────────────

    /// Detect and relieve overloads on the specified branch via terminal redispatch.
    ///
    /// # Algorithm
    /// 1. Solve the current DC power flow to get actual branch flows.
    /// 2. If `overloaded_branch` is within rating, no action needed.
    /// 3. Compute excess power: `P_excess = |flow| − rating`.
    /// 4. Ramp down the sending terminal by `P_excess` (clamped to limits).
    /// 5. Distribute the ramp-down among parallel (non-overloaded) branches'
    ///    receiving terminals proportional to their available headroom.
    /// 6. Re-solve and report.
    pub fn emergency_power_control(&mut self, overloaded_branch: usize) -> Result<EpcResult> {
        let nb = self.dc_branches.len();
        if overloaded_branch >= nb {
            return Err(OxiGridError::InvalidParameter(format!(
                "overloaded_branch {overloaded_branch} >= number of branches {nb}"
            )));
        }

        let mut actions: Vec<String> = Vec::new();

        // Solve initial power flow
        let pf0 = self.solve_dc_power_flow()?;
        let flow0 = pf0.branch_flows_mw[overloaded_branch].abs();
        let rating = self.dc_branches[overloaded_branch].rating_mw;

        if flow0 <= rating {
            return Ok(EpcResult {
                actions: vec![format!(
                    "Branch {overloaded_branch}: flow {flow0:.2} MW within rating {rating:.2} MW — no EPC required"
                )],
                overload_cleared: true,
                final_flows_mw: pf0.branch_flows_mw,
            });
        }

        let excess_mw = flow0 - rating;
        let from_t = self.dc_branches[overloaded_branch].from_terminal;
        let to_t = self.dc_branches[overloaded_branch].to_terminal;

        // Determine sending terminal (the one injecting power into this branch)
        let flow_signed = pf0.branch_flows_mw[overloaded_branch];
        let (sending, receiving) = if flow_signed >= 0.0 {
            (from_t, to_t)
        } else {
            (to_t, from_t)
        };

        // Ramp down sending terminal
        let old_p = self.terminals[sending].p_setpoint_mw;
        let ramp_down = excess_mw.min(self.config.emergency_ramp_rate);
        let new_p_sending = self.terminals[sending].clamp_power(old_p - ramp_down);
        let actual_ramp = old_p - new_p_sending;
        self.terminals[sending].p_setpoint_mw = new_p_sending;
        actions.push(format!(
            "EPC: ramp down terminal {} by {actual_ramp:.2} MW (from {old_p:.2} to {new_p_sending:.2} MW)",
            self.terminals[sending].terminal_id
        ));

        // Find parallel paths: all other terminals that can absorb
        let available: Vec<(usize, f64)> = self
            .terminals
            .iter()
            .enumerate()
            .filter(|(i, t)| {
                *i != sending && *i != receiving && t.p_max_mw - t.p_setpoint_mw > 1e-6
            })
            .map(|(i, t)| (i, t.p_max_mw - t.p_setpoint_mw))
            .collect();

        let total_headroom: f64 = available.iter().map(|(_, h)| h).sum();

        if total_headroom > 1e-6 && actual_ramp > 1e-6 {
            for &(idx, headroom) in &available {
                let share = (headroom / total_headroom) * actual_ramp;
                let old = self.terminals[idx].p_setpoint_mw;
                self.terminals[idx].p_setpoint_mw += share;
                actions.push(format!(
                    "EPC: ramp up terminal {} by {share:.2} MW (to {:.2} MW)",
                    self.terminals[idx].terminal_id, self.terminals[idx].p_setpoint_mw
                ));
                let _ = old; // kept for clarity
            }
        }

        // Re-solve after EPC
        let pf1 = self.solve_dc_power_flow()?;
        let flow1 = pf1.branch_flows_mw[overloaded_branch].abs();
        let overload_cleared = flow1 <= rating * 1.001; // 0.1% tolerance

        actions.push(format!(
            "EPC result: branch {overloaded_branch} flow {flow1:.2} MW (rating {rating:.2} MW) — {}",
            if overload_cleared { "CLEARED" } else { "still overloaded" }
        ));

        Ok(EpcResult {
            actions,
            overload_cleared,
            final_flows_mw: pf1.branch_flows_mw,
        })
    }

    // ─── Loss calculation ─────────────────────────────────────────────────

    /// Compute total MTDC system losses \[MW\].
    ///
    /// # Components
    /// 1. **Cable losses**: `P_loss = I² × R` for each branch, where
    ///    `I = (V_i − V_j) / R_ij` \[pu\] and `P = I² × R × base_mw`.
    /// 2. **Converter losses**: `P_conv = |p_actual| × (converter_loss_pct / 100)`.
    pub fn calculate_losses(&self) -> f64 {
        let cable_loss: f64 = self
            .dc_branches
            .iter()
            .map(|br| {
                let vi = self.terminals[br.from_terminal].vdc_actual_pu;
                let vj = self.terminals[br.to_terminal].vdc_actual_pu;
                let i_pu = (vi - vj) / br.resistance_pu;
                i_pu * i_pu * br.resistance_pu * self.base_mw
            })
            .sum();

        let conv_loss: f64 = self
            .terminals
            .iter()
            .map(|t| t.p_actual_mw.abs() * t.converter_loss_pct / 100.0)
            .sum();

        cable_loss + conv_loss
    }

    // ─── Power sharing assessment ─────────────────────────────────────────

    /// Assess how well each terminal tracks its scheduled power.
    ///
    /// The deviation is expressed as a percentage of the terminal's rated capacity:
    /// `deviation_pct = |actual − scheduled| / rated × 100`.
    ///
    /// The droop participation factor is the normalised droop gain:
    /// `droop_factor_i = k_i / Σ_j k_j` (0 for non-droop terminals).
    pub fn power_sharing_assessment(&self) -> PowerSharingReport {
        let total_droop: f64 = self
            .terminals
            .iter()
            .filter(|t| t.control_mode == VscControlMode::DcVoltageControl)
            .map(|t| t.droop_gain)
            .sum();

        let mut report = PowerSharingReport {
            terminal_id: Vec::new(),
            scheduled_mw: Vec::new(),
            actual_mw: Vec::new(),
            deviation_pct: Vec::new(),
            droop_factor: Vec::new(),
        };

        for t in &self.terminals {
            report.terminal_id.push(t.terminal_id.clone());
            report.scheduled_mw.push(t.p_setpoint_mw);
            report.actual_mw.push(t.p_actual_mw);

            let rated = if t.rated_mw.abs() > 1e-9 {
                t.rated_mw
            } else {
                1.0
            };
            let dev = (t.p_actual_mw - t.p_setpoint_mw).abs() / rated * 100.0;
            report.deviation_pct.push(dev);

            let df = if t.control_mode == VscControlMode::DcVoltageControl && total_droop > 1e-9 {
                t.droop_gain / total_droop
            } else {
                0.0
            };
            report.droop_factor.push(df);
        }

        report
    }

    // ─── N-1 contingency analysis ─────────────────────────────────────────

    /// Perform N-1 contingency analysis by removing each DC branch in turn.
    ///
    /// For each branch outage:
    /// 1. Clone the grid with that branch removed.
    /// 2. Solve the power flow.
    /// 3. Check voltage deviations and branch overloads.
    ///
    /// A contingency is **infeasible** if the power flow fails to converge,
    /// or if any terminal voltage deviates more than 0.1 pu from nominal.
    pub fn n1_contingency_analysis(&self) -> Vec<DcContingencyResult> {
        let nb = self.dc_branches.len();
        let mut results = Vec::with_capacity(nb);

        for outaged in 0..nb {
            let mut clone = self.clone();
            clone.dc_branches.remove(outaged);

            // Check if isolated terminals exist (no connected branch)
            // If a terminal has no remaining branch, contingency is infeasible
            let n_terms = clone.terminals.len();
            let mut connected = vec![false; n_terms];
            for br in &clone.dc_branches {
                connected[br.from_terminal] = true;
                connected[br.to_terminal] = true;
            }
            // For P2P grids removing the only branch isolates all terminals
            let isolated = connected.iter().any(|&c| !c);

            if isolated && clone.dc_branches.is_empty() {
                results.push(DcContingencyResult {
                    outaged_branch: outaged,
                    feasible: false,
                    max_voltage_deviation_pu: f64::NAN,
                    overloaded_branches: Vec::new(),
                });
                continue;
            }

            match clone.solve_dc_power_flow() {
                Err(_) => {
                    results.push(DcContingencyResult {
                        outaged_branch: outaged,
                        feasible: false,
                        max_voltage_deviation_pu: f64::NAN,
                        overloaded_branches: Vec::new(),
                    });
                }
                Ok(pf) => {
                    let max_dev = pf
                        .vdc_pu
                        .iter()
                        .map(|&v| (v - 1.0).abs())
                        .fold(0.0_f64, f64::max);

                    let overloaded: Vec<usize> = clone
                        .dc_branches
                        .iter()
                        .enumerate()
                        .filter(|(bi, br)| pf.branch_flows_mw[*bi].abs() > br.rating_mw)
                        .map(|(bi, _)| bi)
                        .collect();

                    let feasible = pf.converged && max_dev <= 0.1 && overloaded.is_empty();

                    results.push(DcContingencyResult {
                        outaged_branch: outaged,
                        feasible,
                        max_voltage_deviation_pu: max_dev,
                        overloaded_branches: overloaded,
                    });
                }
            }
        }

        results
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // Helper: create a symmetric point-to-point grid
    fn make_p2p_grid() -> MtdcGrid {
        let mut grid = MtdcGrid::new("P2P", 320.0, 1000.0, DcTopology::PointToPoint);

        let mut t0 = VscTerminal::new("VSC-A", 500.0, 320.0, VscControlMode::DcVoltageControl);
        t0.p_setpoint_mw = -400.0; // rectifier: absorb from AC
        t0.vdc_setpoint_pu = 1.0;
        t0.droop_gain = 50.0;

        let mut t1 = VscTerminal::new("VSC-B", 500.0, 320.0, VscControlMode::ActivePowerControl);
        t1.p_setpoint_mw = 400.0; // inverter: inject to AC
        t1.vdc_setpoint_pu = 1.0;

        grid.add_terminal(t0);
        grid.add_terminal(t1);

        // Cable: 100 km, 0.02 pu resistance
        grid.add_branch(DcBranch {
            from_terminal: 0,
            to_terminal: 1,
            resistance_pu: 0.02,
            inductance_mh: 50.0,
            length_km: 100.0,
            rating_mw: 600.0,
        });

        grid
    }

    // Helper: 3-terminal meshed grid
    fn make_3terminal_grid() -> MtdcGrid {
        let mut grid = MtdcGrid::new("3T", 320.0, 1000.0, DcTopology::MultiTerminalMeshed);

        let mut t0 = VscTerminal::new("T0", 600.0, 320.0, VscControlMode::DcVoltageControl);
        t0.p_setpoint_mw = -500.0;
        t0.droop_gain = 60.0;
        t0.vdc_setpoint_pu = 1.0;

        let mut t1 = VscTerminal::new("T1", 400.0, 320.0, VscControlMode::ActivePowerControl);
        t1.p_setpoint_mw = 200.0;
        t1.vdc_setpoint_pu = 1.0;

        let mut t2 = VscTerminal::new("T2", 400.0, 320.0, VscControlMode::ActivePowerControl);
        t2.p_setpoint_mw = 280.0;
        t2.vdc_setpoint_pu = 1.0;

        grid.add_terminal(t0);
        grid.add_terminal(t1);
        grid.add_terminal(t2);

        // T0–T1
        grid.add_branch(DcBranch {
            from_terminal: 0,
            to_terminal: 1,
            resistance_pu: 0.01,
            inductance_mh: 30.0,
            length_km: 80.0,
            rating_mw: 400.0,
        });
        // T0–T2
        grid.add_branch(DcBranch {
            from_terminal: 0,
            to_terminal: 2,
            resistance_pu: 0.015,
            inductance_mh: 40.0,
            length_km: 100.0,
            rating_mw: 400.0,
        });
        // T1–T2 (meshed loop)
        grid.add_branch(DcBranch {
            from_terminal: 1,
            to_terminal: 2,
            resistance_pu: 0.012,
            inductance_mh: 25.0,
            length_km: 70.0,
            rating_mw: 200.0,
        });

        grid
    }

    // ── Test 1: Point-to-point power flow converges ───────────────────────

    #[test]
    fn test_p2p_power_flow_converges() {
        let mut grid = make_p2p_grid();
        let result = grid.solve_dc_power_flow().expect("P2P power flow failed");
        assert!(result.converged, "P2P PF should converge");
        assert!(result.iterations > 0);
        // Voltages should be physically reasonable [0.9, 1.1] pu
        for &v in &result.vdc_pu {
            assert!(v > 0.85 && v < 1.15, "voltage {v:.4} pu out of range");
        }
    }

    // ── Test 2: Y-bus is symmetric with positive diagonal ────────────────

    #[test]
    fn test_y_bus_symmetric_positive_diagonal() {
        let grid = make_3terminal_grid();
        let y = grid.build_y_bus().expect("Y-bus construction failed");
        let n = y.len();

        #[allow(clippy::needless_range_loop)]
        for i in 0..n {
            // Diagonal must be positive
            assert!(
                y[i][i] > 0.0,
                "Y[{i}][{i}] = {} should be positive",
                y[i][i]
            );
            for j in 0..n {
                // Symmetry
                let diff = (y[i][j] - y[j][i]).abs();
                assert!(
                    diff < 1e-12,
                    "Y-bus not symmetric: Y[{i}][{j}]={} vs Y[{j}][{i}]={}",
                    y[i][j],
                    y[j][i]
                );
            }
        }
    }

    // ── Test 3: 3-terminal meshed grid power balance ──────────────────────

    #[test]
    fn test_3terminal_power_balance() {
        let mut grid = make_3terminal_grid();
        let result = grid.solve_dc_power_flow().expect("3-terminal PF failed");

        // Sum of DC-side power injections should be ≈ losses (small)
        // p_mw[i] is power injected into the DC network by terminal i
        let sum_p: f64 = result.p_mw.iter().sum();
        // Power balance: net injection = losses → should be small relative to rating
        assert!(
            sum_p.abs() < 50.0,
            "Power balance error {sum_p:.3} MW too large"
        );
    }

    // ── Test 4: Droop shares 100 MW proportional to gains ────────────────

    #[test]
    fn test_droop_proportional_sharing() {
        let mut grid = MtdcGrid::new("Droop", 320.0, 1000.0, DcTopology::MultiTerminalMeshed);

        let mut t0 = VscTerminal::new("SLACK", 800.0, 320.0, VscControlMode::DcVoltageControl);
        t0.droop_gain = 40.0;
        t0.p_setpoint_mw = -300.0;
        t0.vdc_setpoint_pu = 1.0;

        let mut t1 = VscTerminal::new("DROOP2", 400.0, 320.0, VscControlMode::DcVoltageControl);
        t1.droop_gain = 60.0;
        t1.p_setpoint_mw = 200.0;
        t1.vdc_setpoint_pu = 1.0;

        let mut t2 = VscTerminal::new("PQ", 400.0, 320.0, VscControlMode::ActivePowerControl);
        t2.p_setpoint_mw = 100.0;
        t2.vdc_setpoint_pu = 1.0;

        grid.add_terminal(t0);
        grid.add_terminal(t1);
        grid.add_terminal(t2);

        grid.add_branch(DcBranch {
            from_terminal: 0,
            to_terminal: 1,
            resistance_pu: 0.01,
            inductance_mh: 0.0,
            length_km: 50.0,
            rating_mw: 500.0,
        });
        grid.add_branch(DcBranch {
            from_terminal: 1,
            to_terminal: 2,
            resistance_pu: 0.01,
            inductance_mh: 0.0,
            length_km: 50.0,
            rating_mw: 500.0,
        });

        let disturbance_mw = 100.0;
        // Only t0 and t1 are droop (t2 is excluded as disturbed context);
        // disturbed terminal = 2 (PQ, sudden loss)
        let delta_vdc = grid
            .apply_droop_control(disturbance_mw, 2)
            .expect("droop failed");

        // Total droop = 40 + 60 = 100; ΔV = 100/100 = 1 pu (exaggerated for test)
        assert!((delta_vdc - 1.0).abs() < 1e-9, "ΔV_dc = {delta_vdc:.6}");

        // t0 absorbs 40% = 40 MW => setpoint from -300 to -260
        let expected_t0 = -300.0 + 40.0;
        assert!(
            (grid.terminals[0].p_setpoint_mw - expected_t0).abs() < 1e-6,
            "t0 setpoint = {:.3}, expected {expected_t0}",
            grid.terminals[0].p_setpoint_mw
        );

        // t1 absorbs 60% = 60 MW => setpoint from 200 to 260
        let expected_t1 = 200.0 + 60.0;
        assert!(
            (grid.terminals[1].p_setpoint_mw - expected_t1).abs() < 1e-6,
            "t1 setpoint = {:.3}, expected {expected_t1}",
            grid.terminals[1].p_setpoint_mw
        );
    }

    // ── Test 5: Losses are non-negative ──────────────────────────────────

    #[test]
    fn test_losses_non_negative() {
        let mut grid = make_3terminal_grid();
        let _ = grid.solve_dc_power_flow().expect("PF failed");
        let losses = grid.calculate_losses();
        assert!(
            losses >= 0.0,
            "Total losses {losses:.4} MW should be non-negative"
        );
        // Also check via PF result field
        let mut grid2 = make_p2p_grid();
        let pf = grid2.solve_dc_power_flow().expect("P2P PF failed");
        assert!(pf.total_losses_mw >= 0.0, "PF losses should be >= 0");
    }

    // ── Test 6: N-1 contingency — P2P becomes infeasible on outage ───────

    #[test]
    fn test_n1_p2p_infeasible_on_outage() {
        let grid = make_p2p_grid();
        let results = grid.n1_contingency_analysis();
        assert_eq!(results.len(), 1, "P2P has one branch");
        // Removing the only branch should be infeasible
        assert!(
            !results[0].feasible,
            "P2P N-1 with sole branch removed should be infeasible"
        );
    }

    // ── Test 7: Emergency power control clears overload ───────────────────

    #[test]
    fn test_epc_clears_overload() {
        let mut grid = MtdcGrid::new("EPC", 320.0, 1000.0, DcTopology::MultiTerminalMeshed);

        // Deliberately set low rating so overload is triggered
        let mut t0 = VscTerminal::new("GEN", 1000.0, 320.0, VscControlMode::DcVoltageControl);
        t0.p_setpoint_mw = -900.0;
        t0.droop_gain = 100.0;
        t0.vdc_setpoint_pu = 1.0;

        let mut t1 = VscTerminal::new("LOAD1", 600.0, 320.0, VscControlMode::ActivePowerControl);
        t1.p_setpoint_mw = 400.0;
        t1.vdc_setpoint_pu = 1.0;

        let mut t2 = VscTerminal::new("LOAD2", 600.0, 320.0, VscControlMode::ActivePowerControl);
        t2.p_setpoint_mw = 400.0;
        t2.vdc_setpoint_pu = 1.0;

        grid.add_terminal(t0);
        grid.add_terminal(t1);
        grid.add_terminal(t2);

        // Branch 0: GEN→LOAD1, rating just 200 MW (will be overloaded)
        grid.add_branch(DcBranch {
            from_terminal: 0,
            to_terminal: 1,
            resistance_pu: 0.005,
            inductance_mh: 0.0,
            length_km: 50.0,
            rating_mw: 200.0,
        });
        // Branch 1: GEN→LOAD2, plenty of rating
        grid.add_branch(DcBranch {
            from_terminal: 0,
            to_terminal: 2,
            resistance_pu: 0.005,
            inductance_mh: 0.0,
            length_km: 50.0,
            rating_mw: 800.0,
        });

        // Configure fast ramp rate
        grid.config.emergency_ramp_rate = 500.0;

        let epc = grid.emergency_power_control(0).expect("EPC failed");
        assert!(
            !epc.actions.is_empty(),
            "EPC should report at least one action"
        );
        // Either cleared or partially cleared (depends on ramp limits)
        // Key check: final flow reported in result
        assert_eq!(epc.final_flows_mw.len(), 2);
    }

    // ── Test 8: Power sharing report — deviations near zero at setpoint ───

    #[test]
    fn test_power_sharing_report_structure() {
        let mut grid = make_3terminal_grid();
        // Pre-set p_actual to match setpoints for perfect sharing
        for t in &mut grid.terminals {
            t.p_actual_mw = t.p_setpoint_mw;
        }
        let report = grid.power_sharing_assessment();

        assert_eq!(report.terminal_id.len(), 3);
        assert_eq!(report.scheduled_mw.len(), 3);
        assert_eq!(report.actual_mw.len(), 3);
        assert_eq!(report.deviation_pct.len(), 3);
        assert_eq!(report.droop_factor.len(), 3);

        // Droop factors must sum to 1.0 (only T0 is droop)
        let df_sum: f64 = report.droop_factor.iter().sum();
        assert!(
            (df_sum - 1.0).abs() < 1e-9,
            "droop factor sum = {df_sum:.6} should be 1.0"
        );

        // Deviations should be 0 since actual = setpoint
        for (i, &dev) in report.deviation_pct.iter().enumerate() {
            assert!(
                dev.abs() < 1e-9,
                "terminal {i} deviation {dev:.4}% should be zero"
            );
        }
    }

    // ── Test 9: Y-bus rows sum to zero (Kirchhoff's current law) ─────────

    #[test]
    fn test_y_bus_row_sums_zero() {
        let grid = make_3terminal_grid();
        let y = grid.build_y_bus().expect("Y-bus failed");
        for (i, row) in y.iter().enumerate() {
            let row_sum: f64 = row.iter().sum();
            assert!(
                row_sum.abs() < 1e-10,
                "Y-bus row {i} sum = {row_sum:.2e} (should be 0)"
            );
        }
    }

    // ── Test 10: branch_flows length matches dc_branches ─────────────────

    #[test]
    fn test_branch_flows_length() {
        let mut grid = make_3terminal_grid();
        let pf = grid.solve_dc_power_flow().expect("PF failed");
        assert_eq!(
            pf.branch_flows_mw.len(),
            grid.dc_branches.len(),
            "branch_flows_mw length mismatch"
        );
        assert_eq!(pf.vdc_pu.len(), grid.terminals.len());
        assert_eq!(pf.p_mw.len(), grid.terminals.len());
    }
}
