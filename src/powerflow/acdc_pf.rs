//! AC/DC Hybrid Grid Power Flow Solver with VSC-HVDC links.
//!
//! Implements a sequential AC-DC iteration scheme for power systems containing
//! Voltage Source Converter (VSC) HVDC links. The AC and DC sub-networks are
//! solved alternately until the overall mismatch converges.
//!
//! # Algorithm
//!
//! Each outer iteration performs:
//! 1. Solve AC power flow with VSCs modelled as PQ/PV bus injections.
//! 2. Update VSC DC-side power from the AC solution (accounting for losses).
//! 3. Solve DC power flow: purely resistive `G_dc · V_dc = I_dc`.
//! 4. Update VSC AC injections from new DC voltages.
//! 5. Check overall convergence (max mismatch < tolerance \[pu\]).
//!
//! # References
//!
//! Beerten, J., Cole, S., Belmans, R. (2012). "Generalised Steady-State VSC
//! MTDC Model for RMS Simulations". *IEEE Trans. Power Syst.* 27(2).

use crate::error::OxiGridError;
use thiserror::Error;

// ---------------------------------------------------------------------------
// Error type
// ---------------------------------------------------------------------------

/// Errors from the AC/DC hybrid power flow solver.
#[derive(Debug, Error)]
pub enum AcDcError {
    /// Outer iteration did not reach the convergence criterion.
    #[error("AC/DC power flow did not converge after {iterations} iterations")]
    NotConverged { iterations: usize },
    /// A configuration parameter is out of range or inconsistent.
    #[error("invalid AC/DC configuration: {0}")]
    InvalidConfig(String),
    /// The DC conductance matrix is singular (disconnected DC network or missing slack).
    #[error("DC conductance matrix is singular")]
    SingularMatrix,
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Solver configuration for the sequential AC/DC hybrid power flow solver (`AcDcPfSolver`).
#[derive(Debug, Clone)]
pub struct AcDcSequentialConfig {
    /// Number of AC buses in the network.
    pub n_ac_buses: usize,
    /// Number of DC buses in the network.
    pub n_dc_buses: usize,
    /// Convergence tolerance on maximum power mismatch \[pu\].
    pub tolerance: f64,
    /// Maximum number of outer AC-DC iterations.
    pub max_iterations: usize,
    /// System base apparent power \[MVA\].
    pub base_mva: f64,
}

impl Default for AcDcSequentialConfig {
    fn default() -> Self {
        Self {
            n_ac_buses: 0,
            n_dc_buses: 0,
            tolerance: 1e-6,
            max_iterations: 50,
            base_mva: 100.0,
        }
    }
}

// ---------------------------------------------------------------------------
// VSC operating mode
// ---------------------------------------------------------------------------

/// Operating mode of a Voltage Source Converter.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VscMode {
    /// Controls DC active power and DC voltage magnitude.
    PdcVdc,
    /// Controls AC active power and AC voltage magnitude (grid-forming).
    PacVac,
    /// Acts as the DC slack bus — holds DC voltage at setpoint.
    SlackDc,
    /// P-V droop: DC power varies linearly with DC voltage deviation.
    Droop,
}

// ---------------------------------------------------------------------------
// Data structures
// ---------------------------------------------------------------------------

/// A Voltage Source Converter connecting one AC bus to one DC bus.
#[derive(Debug, Clone)]
pub struct VscConverter {
    /// Converter index (0-based).
    pub id: usize,
    /// Index of the connected AC bus (0-based).
    pub ac_bus: usize,
    /// Index of the connected DC bus (0-based).
    pub dc_bus: usize,
    /// Operating mode.
    pub mode: VscMode,
    /// Active power setpoint on the AC side \[MW\] (positive = injecting into AC grid).
    pub p_set_mw: f64,
    /// AC voltage setpoint \[pu\] (used in `PacVac` mode).
    pub v_ac_set_pu: f64,
    /// DC voltage setpoint \[pu\] (used in `SlackDc`/`PdcVdc` mode).
    pub v_dc_set_pu: f64,
    /// Fractional converter losses (e.g. 0.01 = 1 %).
    pub p_loss_fraction: f64,
    /// Minimum reactive power output \[MVAr\].
    pub q_min_mvar: f64,
    /// Maximum reactive power output \[MVAr\].
    pub q_max_mvar: f64,
    /// Converter MVA rating \[MVA\].
    pub rated_mva: f64,
}

/// A DC bus in the HVDC network (used by `AcDcPfSolver` sequential solver).
#[derive(Debug, Clone)]
pub struct VscDcBus {
    /// DC bus index (0-based).
    pub id: usize,
    /// DC voltage \[pu\] (used as initial guess and updated by solver).
    pub v_dc_pu: f64,
    /// DC load (non-VSC) \[MW\].
    pub p_load_mw: f64,
}

/// A resistive DC transmission branch (used by `AcDcPfSolver` sequential solver).
#[derive(Debug, Clone)]
pub struct VscDcBranch {
    /// From-bus index (0-based).
    pub from_bus: usize,
    /// To-bus index (0-based).
    pub to_bus: usize,
    /// Branch resistance \[pu\] (no reactance on DC).
    pub resistance_pu: f64,
    /// Thermal rating \[MW\].
    pub rating_mw: f64,
}

// ---------------------------------------------------------------------------
// Result
// ---------------------------------------------------------------------------

/// Converged solution from the sequential AC/DC hybrid power flow (`AcDcPfSolver`).
#[derive(Debug, Clone)]
pub struct AcDcSequentialResult {
    /// AC bus voltages: `(magnitude [pu], angle [rad])` for each bus.
    pub ac_voltages: Vec<(f64, f64)>,
    /// DC bus voltages \[pu\].
    pub dc_voltages: Vec<f64>,
    /// AC-side active power injection of each VSC \[MW\].
    pub vsc_p_ac_mw: Vec<f64>,
    /// AC-side reactive power injection of each VSC \[MVAr\].
    pub vsc_q_ac_mvar: Vec<f64>,
    /// DC-side active power injection of each VSC \[MW\].
    pub vsc_p_dc_mw: Vec<f64>,
    /// Active power flow in each DC branch \[MW\] (from → to convention).
    pub dc_line_flows_mw: Vec<f64>,
    /// Total VSC converter losses \[MW\].
    pub total_converter_losses_mw: f64,
    /// Whether the outer iteration converged.
    pub converged: bool,
    /// Number of outer iterations performed.
    pub iterations: usize,
}

// ---------------------------------------------------------------------------
// Solver
// ---------------------------------------------------------------------------

/// AC/DC hybrid power flow solver using sequential AC-DC iteration.
#[derive(Debug, Clone)]
pub struct AcDcPfSolver {
    config: AcDcSequentialConfig,
    ac_buses: Vec<AcBusData>,
    ac_branches: Vec<AcBranchData>,
    dc_buses: Vec<VscDcBus>,
    dc_branches: Vec<VscDcBranch>,
    vsc_converters: Vec<VscConverter>,
}

/// Lightweight AC bus data for the internal Gauss-Seidel solver.
#[derive(Debug, Clone)]
struct AcBusData {
    idx: usize,
    bus_type: AcBusType,
    /// Net scheduled power injection \[pu\] (generation − load).
    p_sch_pu: f64,
    /// Net scheduled reactive power \[pu\].
    q_sch_pu: f64,
    /// Voltage setpoint magnitude \[pu\] (for PV/Slack buses).
    v_set_pu: f64,
    /// Current voltage magnitude \[pu\].
    v_mag: f64,
    /// Current voltage angle \[rad\].
    v_ang: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum AcBusType {
    Slack,
    Pv,
    Pq,
}

/// Lightweight AC branch for internal Y-bus construction.
#[derive(Debug, Clone)]
struct AcBranchData {
    from: usize,
    to: usize,
    /// Series conductance \[pu\].
    g: f64,
    /// Series susceptance \[pu\].
    b: f64,
    /// Half-line charging susceptance \[pu\].
    b_half: f64,
}

impl AcDcPfSolver {
    /// Create a new solver with the given configuration.
    pub fn new(config: AcDcSequentialConfig) -> Self {
        Self {
            config,
            ac_buses: Vec::new(),
            ac_branches: Vec::new(),
            dc_buses: Vec::new(),
            dc_branches: Vec::new(),
            vsc_converters: Vec::new(),
        }
    }

    /// Add an AC bus using `crate::network::bus::Bus` data.
    pub fn add_ac_bus(&mut self, bus: crate::network::bus::Bus) {
        use crate::network::bus::BusType;
        let base_mva = self.config.base_mva.max(1.0);
        let bus_type = match bus.bus_type {
            BusType::Slack => AcBusType::Slack,
            BusType::PV => AcBusType::Pv,
            BusType::PQ => AcBusType::Pq,
        };
        // Bus index = position in vec (0-based)
        let idx = self.ac_buses.len();
        self.ac_buses.push(AcBusData {
            idx,
            bus_type,
            p_sch_pu: (-bus.pd.0) / base_mva,
            q_sch_pu: (-bus.qd.0) / base_mva,
            v_set_pu: bus.vm,
            v_mag: bus.vm,
            v_ang: bus.va,
        });
    }

    /// Add an AC transmission branch.
    pub fn add_ac_branch(&mut self, branch: crate::network::branch::Branch) {
        let r = branch.r;
        let x = branch.x;
        let denom = r * r + x * x;
        let (g, b) = if denom > 1e-12 {
            (r / denom, -x / denom)
        } else {
            (0.0, 0.0)
        };
        // from/to are 1-based external IDs; store as 0-based indices
        let from = branch.from_bus.saturating_sub(1);
        let to = branch.to_bus.saturating_sub(1);
        self.ac_branches.push(AcBranchData {
            from,
            to,
            g,
            b,
            b_half: branch.b / 2.0,
        });
    }

    /// Add a DC bus.
    pub fn add_dc_bus(&mut self, bus: VscDcBus) {
        self.dc_buses.push(bus);
    }

    /// Add a DC resistive branch.
    pub fn add_dc_branch(&mut self, branch: VscDcBranch) {
        self.dc_branches.push(branch);
    }

    /// Add a VSC converter.
    pub fn add_vsc(&mut self, vsc: VscConverter) {
        self.vsc_converters.push(vsc);
    }

    /// Run the sequential AC/DC power flow iteration.
    ///
    /// Returns [`AcDcSequentialResult`] on convergence, or [`AcDcError`] on failure.
    pub fn solve(&self) -> Result<AcDcSequentialResult, AcDcError> {
        let n_dc = self.dc_buses.len();
        let n_vsc = self.vsc_converters.len();
        let base_mva = self.config.base_mva.max(1.0);

        // Validate
        if n_dc > 0 && self.dc_branches.is_empty() && n_dc > 1 {
            return Err(AcDcError::InvalidConfig(
                "multi-bus DC network requires DC branches".to_string(),
            ));
        }
        if !self
            .vsc_converters
            .iter()
            .any(|v| v.mode == VscMode::SlackDc)
            && n_dc > 0
        {
            // Warn but continue — first VSC acts as voltage reference
        }

        // State vectors
        let mut v_ac_mag: Vec<f64> = self.ac_buses.iter().map(|b| b.v_mag).collect();
        let mut v_ac_ang: Vec<f64> = self.ac_buses.iter().map(|b| b.v_ang).collect();
        let mut v_dc: Vec<f64> = self.dc_buses.iter().map(|b| b.v_dc_pu).collect();

        // VSC power vectors (pu)
        let mut vsc_p_ac_pu: Vec<f64> = vec![0.0; n_vsc];
        let mut vsc_q_ac_pu: Vec<f64> = vec![0.0; n_vsc];
        let mut vsc_p_dc_pu: Vec<f64> = vec![0.0; n_vsc];

        // Initialize VSC AC injections from setpoints
        for (k, vsc) in self.vsc_converters.iter().enumerate() {
            vsc_p_ac_pu[k] = vsc.p_set_mw / base_mva;
            vsc_q_ac_pu[k] = 0.0;
        }

        let mut converged = false;
        let mut iterations = 0usize;

        for _iter in 0..self.config.max_iterations {
            iterations += 1;

            // -----------------------------------------------------------
            // Step 1: Solve AC sub-problem (Gauss-Seidel with VSC injections)
            // -----------------------------------------------------------
            self.solve_ac_gauss_seidel(
                &mut v_ac_mag,
                &mut v_ac_ang,
                &vsc_p_ac_pu,
                &vsc_q_ac_pu,
                30,
            )?;

            // -----------------------------------------------------------
            // Step 2: Update DC-side VSC power from AC solution
            // -----------------------------------------------------------
            for (k, vsc) in self.vsc_converters.iter().enumerate() {
                let p_ac = vsc_p_ac_pu[k]; // pu
                                           // Rectifier (AC→DC): p_dc = p_ac * (1 - loss)
                                           // Inverter  (DC→AC): p_dc = p_ac / (1 - loss)
                                           // Convention: positive p_ac = power from DC into AC (inverter)
                vsc_p_dc_pu[k] = if p_ac >= 0.0 {
                    // inverter: DC supplies AC
                    p_ac / (1.0 - vsc.p_loss_fraction.clamp(0.0, 0.5))
                } else {
                    // rectifier: AC supplies DC
                    p_ac * (1.0 - vsc.p_loss_fraction.clamp(0.0, 0.5))
                };
            }

            // -----------------------------------------------------------
            // Step 3: Solve DC sub-problem
            // -----------------------------------------------------------
            if n_dc > 0 {
                self.solve_dc_network(&mut v_dc, &vsc_p_dc_pu, &v_ac_mag)?;
            }

            // -----------------------------------------------------------
            // Step 4: Update AC injections from DC solution
            // -----------------------------------------------------------
            for (k, vsc) in self.vsc_converters.iter().enumerate() {
                let vsc_dc = if vsc.dc_bus < v_dc.len() {
                    v_dc[vsc.dc_bus]
                } else {
                    1.0
                };
                match vsc.mode {
                    VscMode::SlackDc | VscMode::PacVac => {
                        // AC injection stays at setpoint
                        vsc_p_ac_pu[k] = vsc.p_set_mw / base_mva;
                    }
                    VscMode::PdcVdc => {
                        // AC power = DC power accounting for losses
                        let p_dc = vsc_p_dc_pu[k];
                        vsc_p_ac_pu[k] = if p_dc >= 0.0 {
                            p_dc * (1.0 - vsc.p_loss_fraction.clamp(0.0, 0.5))
                        } else {
                            p_dc / (1.0 - vsc.p_loss_fraction.clamp(0.0, 0.5))
                        };
                    }
                    VscMode::Droop => {
                        // P-V droop: ΔP = -K * ΔV_dc
                        let droop_gain = 10.0; // pu/pu (fixed, could be configurable)
                        let dv = vsc_dc - vsc.v_dc_set_pu;
                        vsc_p_ac_pu[k] = vsc.p_set_mw / base_mva - droop_gain * dv;
                    }
                }
                // Q injection: clamp to VSC reactive limits
                let q_raw = vsc_q_ac_pu[k];
                let q_min = vsc.q_min_mvar / base_mva;
                let q_max = vsc.q_max_mvar / base_mva;
                vsc_q_ac_pu[k] = q_raw.max(q_min).min(q_max);
            }

            // -----------------------------------------------------------
            // Step 5: Check convergence
            // -----------------------------------------------------------
            let mismatch =
                self.compute_ac_mismatch(&v_ac_mag, &v_ac_ang, &vsc_p_ac_pu, &vsc_q_ac_pu);
            if mismatch < self.config.tolerance {
                converged = true;
                break;
            }
        }

        if !converged {
            return Err(AcDcError::NotConverged { iterations });
        }

        // Compute DC line flows
        let dc_line_flows_mw: Vec<f64> = self
            .dc_branches
            .iter()
            .map(|br| {
                let v_from = if br.from_bus < v_dc.len() {
                    v_dc[br.from_bus]
                } else {
                    1.0
                };
                let v_to = if br.to_bus < v_dc.len() {
                    v_dc[br.to_bus]
                } else {
                    1.0
                };
                // Flow = (V_from - V_to) / R  [pu] → [MW] = * base_mva
                if br.resistance_pu > 1e-12 {
                    (v_from - v_to) / br.resistance_pu * base_mva
                } else {
                    0.0
                }
            })
            .collect();

        // Total losses
        let total_losses_mw: f64 = self
            .vsc_converters
            .iter()
            .enumerate()
            .map(|(k, vsc)| {
                let p_ac = vsc_p_ac_pu[k].abs() * base_mva;
                p_ac * vsc.p_loss_fraction.clamp(0.0, 0.5)
            })
            .sum();

        Ok(AcDcSequentialResult {
            ac_voltages: v_ac_mag
                .iter()
                .zip(v_ac_ang.iter())
                .map(|(&m, &a)| (m, a))
                .collect(),
            dc_voltages: v_dc,
            vsc_p_ac_mw: vsc_p_ac_pu.iter().map(|&p| p * base_mva).collect(),
            vsc_q_ac_mvar: vsc_q_ac_pu.iter().map(|&q| q * base_mva).collect(),
            vsc_p_dc_mw: vsc_p_dc_pu.iter().map(|&p| p * base_mva).collect(),
            dc_line_flows_mw,
            total_converter_losses_mw: total_losses_mw,
            converged,
            iterations,
        })
    }

    // -----------------------------------------------------------------------
    // Internal: Gauss-Seidel AC power flow
    // -----------------------------------------------------------------------

    fn solve_ac_gauss_seidel(
        &self,
        v_mag: &mut [f64],
        v_ang: &mut [f64],
        vsc_p: &[f64],
        vsc_q: &[f64],
        max_inner: usize,
    ) -> Result<(), AcDcError> {
        let n = self.ac_buses.len();
        if n == 0 {
            return Ok(());
        }
        let base_mva = self.config.base_mva.max(1.0);

        // Build Y-bus (conductance + susceptance)
        let mut g_bus = vec![vec![0.0f64; n]; n];
        let mut b_bus = vec![vec![0.0f64; n]; n];
        for br in &self.ac_branches {
            if br.from >= n || br.to >= n {
                continue;
            }
            let i = br.from;
            let j = br.to;
            // Off-diagonal
            g_bus[i][j] -= br.g;
            g_bus[j][i] -= br.g;
            b_bus[i][j] -= br.b;
            b_bus[j][i] -= br.b;
            // Diagonal
            g_bus[i][i] += br.g;
            g_bus[j][j] += br.g;
            b_bus[i][i] += br.b + br.b_half;
            b_bus[j][j] += br.b + br.b_half;
        }

        // Add shunt from AC buses (none stored internally, skip)
        // VSC injections per bus
        let mut p_inj = vec![0.0f64; n];
        let mut q_inj = vec![0.0f64; n];
        for bus in &self.ac_buses {
            if bus.idx < n {
                p_inj[bus.idx] = bus.p_sch_pu;
                q_inj[bus.idx] = bus.q_sch_pu;
            }
        }
        for (k, vsc) in self.vsc_converters.iter().enumerate() {
            if vsc.ac_bus < n {
                p_inj[vsc.ac_bus] += vsc_p[k];
                q_inj[vsc.ac_bus] += vsc_q[k];
            }
        }

        // Gauss-Seidel iterations
        for _inner in 0..max_inner {
            for i in 0..n {
                let bus = &self.ac_buses[i];
                if bus.bus_type == AcBusType::Slack {
                    // Slack: fix V and angle
                    v_mag[i] = bus.v_set_pu;
                    v_ang[i] = 0.0;
                    continue;
                }

                // Compute sum Σ_{j≠i} Y_ij * V_j
                let mut sum_g = 0.0f64;
                let mut sum_b = 0.0f64;
                for j in 0..n {
                    if j == i {
                        continue;
                    }
                    let vj_re = v_mag[j] * v_ang[j].cos();
                    let vj_im = v_mag[j] * v_ang[j].sin();
                    sum_g += g_bus[i][j] * vj_re - b_bus[i][j] * vj_im;
                    sum_b += g_bus[i][j] * vj_im + b_bus[i][j] * vj_re;
                }

                let p_i = p_inj[i];
                let q_i = q_inj[i];
                let vi_mag = v_mag[i].max(0.01);
                let vi_ang = v_ang[i];

                // Gauss-Seidel update for complex voltage
                // V_i^new = (1/Y_ii) * [(P_i - jQ_i)/V_i* - Σ_{j≠i} Y_ij*V_j]
                let g_ii = g_bus[i][i];
                let b_ii = b_bus[i][i];
                let denom = g_ii * g_ii + b_ii * b_ii;
                if denom < 1e-20 {
                    continue;
                }

                let vi_re = vi_mag * vi_ang.cos();
                let vi_im = vi_mag * vi_ang.sin();
                // (P - jQ) / V* = (P - jQ) / (Vre - jVim)
                let pq_re = (p_i * vi_re + q_i * vi_im) / (vi_mag * vi_mag);
                let pq_im = (p_i * vi_im - q_i * vi_re) / (vi_mag * vi_mag);

                let rhs_re = pq_re - sum_g;
                let rhs_im = pq_im - sum_b;

                // new V_i = (rhs_re + j*rhs_im) / (g_ii + j*b_ii)
                let new_re = (rhs_re * g_ii + rhs_im * b_ii) / denom;
                let new_im = (rhs_im * g_ii - rhs_re * b_ii) / denom;

                let new_mag = (new_re * new_re + new_im * new_im).sqrt();
                let new_ang = new_im.atan2(new_re);

                if bus.bus_type == AcBusType::Pv {
                    // Fix magnitude to setpoint
                    v_mag[i] = bus.v_set_pu;
                    v_ang[i] = new_ang;
                } else {
                    v_mag[i] = new_mag.clamp(0.5, 1.5);
                    v_ang[i] = new_ang;
                }
            }
        }

        // Suppress unused warning
        let _ = base_mva;
        Ok(())
    }

    // -----------------------------------------------------------------------
    // Internal: DC resistive power flow
    // -----------------------------------------------------------------------

    fn solve_dc_network(
        &self,
        v_dc: &mut [f64],
        vsc_p_dc_pu: &[f64],
        _v_ac_mag: &[f64],
    ) -> Result<(), AcDcError> {
        let n = self.dc_buses.len();
        if n == 0 {
            return Ok(());
        }
        let base_mva = self.config.base_mva.max(1.0);

        // Find the DC slack bus (first SlackDc VSC)
        let slack_dc_bus = self
            .vsc_converters
            .iter()
            .find(|v| v.mode == VscMode::SlackDc)
            .map(|v| v.dc_bus);

        // If no explicit slack, use first DC bus
        let slack_idx = slack_dc_bus.unwrap_or(0);

        // Build G_dc conductance matrix (n×n)
        let mut g_dc = vec![vec![0.0f64; n]; n];
        for br in &self.dc_branches {
            if br.from_bus >= n || br.to_bus >= n {
                continue;
            }
            if br.resistance_pu < 1e-12 {
                continue;
            }
            let g = 1.0 / br.resistance_pu;
            let i = br.from_bus;
            let j = br.to_bus;
            g_dc[i][j] -= g;
            g_dc[j][i] -= g;
            g_dc[i][i] += g;
            g_dc[j][j] += g;
        }

        // DC current injections [pu power / pu voltage = pu current]
        let mut i_dc = vec![0.0f64; n];
        // From VSCs
        for (k, vsc) in self.vsc_converters.iter().enumerate() {
            if vsc.dc_bus < n {
                // Negative because p_dc_pu is power INTO DC network
                // I_dc = P_dc / V_dc (iterative update)
                let v_k = v_dc[vsc.dc_bus].max(0.5);
                i_dc[vsc.dc_bus] += -vsc_p_dc_pu[k] / v_k;
            }
        }
        // From DC loads
        for bus in &self.dc_buses {
            if bus.id < n && bus.p_load_mw.abs() > 1e-12 {
                let v_k = v_dc[bus.id].max(0.5);
                i_dc[bus.id] -= bus.p_load_mw / base_mva / v_k;
            }
        }

        // Apply slack: remove slack row/col, solve reduced system
        let non_slack: Vec<usize> = (0..n).filter(|&i| i != slack_idx).collect();
        let m = non_slack.len();

        if m == 0 {
            // Single DC bus — just fix voltage
            v_dc[slack_idx] = self
                .vsc_converters
                .iter()
                .find(|v| v.dc_bus == slack_idx && v.mode == VscMode::SlackDc)
                .map(|v| v.v_dc_set_pu)
                .unwrap_or(v_dc[slack_idx]);
            return Ok(());
        }

        // Reduced G matrix (m×m)
        let mut g_red = vec![vec![0.0f64; m]; m];
        let mut rhs = vec![0.0f64; m];
        let v_slack = self
            .vsc_converters
            .iter()
            .find(|v| v.dc_bus == slack_idx && v.mode == VscMode::SlackDc)
            .map(|v| v.v_dc_set_pu)
            .unwrap_or(v_dc[slack_idx]);

        for (ri, &i) in non_slack.iter().enumerate() {
            rhs[ri] = i_dc[i];
            // Subtract slack column contribution
            rhs[ri] -= g_dc[i][slack_idx] * v_slack;
            for (rj, &j) in non_slack.iter().enumerate() {
                g_red[ri][rj] = g_dc[i][j];
            }
        }

        // Solve using Gaussian elimination
        let v_sol = gaussian_elimination(&g_red, &rhs).ok_or(AcDcError::SingularMatrix)?;

        // Update voltages
        v_dc[slack_idx] = v_slack;
        for (ri, &i) in non_slack.iter().enumerate() {
            v_dc[i] = v_sol[ri].clamp(0.5, 1.5);
        }

        Ok(())
    }

    // -----------------------------------------------------------------------
    // Internal: compute AC power mismatch norm
    // -----------------------------------------------------------------------

    fn compute_ac_mismatch(
        &self,
        v_mag: &[f64],
        v_ang: &[f64],
        vsc_p: &[f64],
        vsc_q: &[f64],
    ) -> f64 {
        let n = self.ac_buses.len();
        if n == 0 {
            return 0.0;
        }

        let mut g_bus = vec![vec![0.0f64; n]; n];
        let mut b_bus = vec![vec![0.0f64; n]; n];
        for br in &self.ac_branches {
            if br.from >= n || br.to >= n {
                continue;
            }
            let i = br.from;
            let j = br.to;
            g_bus[i][j] -= br.g;
            g_bus[j][i] -= br.g;
            b_bus[i][j] -= br.b;
            b_bus[j][i] -= br.b;
            g_bus[i][i] += br.g;
            g_bus[j][j] += br.g;
            b_bus[i][i] += br.b + br.b_half;
            b_bus[j][j] += br.b + br.b_half;
        }

        let mut p_inj = vec![0.0f64; n];
        let mut q_inj = vec![0.0f64; n];
        for bus in &self.ac_buses {
            if bus.idx < n {
                p_inj[bus.idx] = bus.p_sch_pu;
                q_inj[bus.idx] = bus.q_sch_pu;
            }
        }
        for (k, vsc) in self.vsc_converters.iter().enumerate() {
            if vsc.ac_bus < n {
                p_inj[vsc.ac_bus] += vsc_p[k];
                q_inj[vsc.ac_bus] += vsc_q[k];
            }
        }

        let mut max_mm = 0.0f64;
        for i in 0..n {
            if self.ac_buses[i].bus_type == AcBusType::Slack {
                continue;
            }
            let vi = v_mag[i];
            let ti = v_ang[i];
            let mut p_calc = 0.0;
            let mut q_calc = 0.0;
            for j in 0..n {
                let vj = v_mag[j];
                let tj = v_ang[j];
                let dth = ti - tj;
                p_calc += vi * vj * (g_bus[i][j] * dth.cos() + b_bus[i][j] * dth.sin());
                q_calc += vi * vj * (g_bus[i][j] * dth.sin() - b_bus[i][j] * dth.cos());
            }
            let dp = (p_inj[i] - p_calc).abs();
            let dq = if self.ac_buses[i].bus_type == AcBusType::Pq {
                (q_inj[i] - q_calc).abs()
            } else {
                0.0
            };
            max_mm = max_mm.max(dp).max(dq);
        }
        max_mm
    }
}

// ---------------------------------------------------------------------------
// Gaussian elimination (dense, small systems)
// ---------------------------------------------------------------------------

/// Solve A·x = b via Gaussian elimination with partial pivoting.
/// Returns `None` if the matrix is singular.
#[allow(clippy::needless_range_loop)]
fn gaussian_elimination(a: &[Vec<f64>], b: &[f64]) -> Option<Vec<f64>> {
    let n = b.len();
    if n == 0 {
        return Some(vec![]);
    }
    // Build augmented matrix
    let mut m: Vec<Vec<f64>> = a
        .iter()
        .zip(b.iter())
        .map(|(row, &bi)| {
            let mut r = row.clone();
            r.push(bi);
            r
        })
        .collect();

    for col in 0..n {
        // Partial pivot
        let mut max_row = col;
        let mut max_val = m[col][col].abs();
        for row in (col + 1)..n {
            let abs_val = m[row][col].abs();
            if abs_val > max_val {
                max_val = abs_val;
                max_row = row;
            }
        }
        if max_val < 1e-14 {
            return None;
        }
        m.swap(col, max_row);
        let pivot = m[col][col];
        for row in (col + 1)..n {
            let factor = m[row][col] / pivot;
            for k in col..=n {
                let piv_k = m[col][k];
                m[row][k] -= factor * piv_k;
            }
        }
    }

    // Back-substitution
    let mut x = vec![0.0f64; n];
    for i in (0..n).rev() {
        let mut s = m[i][n];
        for j in (i + 1)..n {
            s -= m[i][j] * x[j];
        }
        x[i] = s / m[i][i];
    }
    Some(x)
}

// ===========================================================================
// Unified Newton-Raphson AC/DC API (specified interface)
// ===========================================================================

/// DC bus type used in the unified NR formulation.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum DcBusType {
    /// Active power and reactive power specified (load bus).
    PQ,
    /// Active power and DC voltage magnitude specified.
    PV,
    /// DC slack bus — voltage reference.
    Slack,
}

/// AC-side control mode of a VSC-HVDC converter.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum ConverterType {
    /// Controls AC-side P and Q.
    PQ,
    /// Controls AC-side P and DC voltage.
    PV,
    /// Controls DC voltage and AC-side Q (slack converter).
    VdcQ,
}

/// AC-DC Voltage Source Converter model.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct AcDcConverter {
    pub id: usize,
    pub ac_bus: usize,
    pub dc_bus: usize,
    pub converter_type: ConverterType,
    /// AC-side active power injection in MW (positive = injecting into AC).
    pub p_ac_mw: f64,
    /// AC-side reactive power injection in MVAr.
    pub q_ac_mvar: f64,
    /// DC-side power in MW (positive = injecting into DC grid).
    pub p_dc_mw: f64,
    /// DC voltage setpoint in kV.
    pub v_dc_kv: f64,
    pub p_ref_mw: f64,
    pub q_ref_mvar: f64,
    pub v_dc_ref_kv: f64,
    /// Converter losses as a fraction of throughput |P_ac|.
    pub losses_fraction: f64,
    pub q_min_mvar: f64,
    pub q_max_mvar: f64,
    /// True when operating in rectifier mode (AC → DC).
    pub is_rectifier: bool,
}

impl AcDcConverter {
    /// Create a new converter with defaults.
    pub fn new(
        id: usize,
        ac_bus: usize,
        dc_bus: usize,
        converter_type: ConverterType,
        p_ref_mw: f64,
        q_ref_mvar: f64,
        v_dc_ref_kv: f64,
    ) -> Self {
        Self {
            id,
            ac_bus,
            dc_bus,
            converter_type,
            p_ac_mw: p_ref_mw,
            q_ac_mvar: q_ref_mvar,
            p_dc_mw: 0.0,
            v_dc_kv: v_dc_ref_kv,
            p_ref_mw,
            q_ref_mvar,
            v_dc_ref_kv,
            losses_fraction: 0.02,
            q_min_mvar: -200.0,
            q_max_mvar: 200.0,
            is_rectifier: p_ref_mw > 0.0,
        }
    }
}

/// A DC bus in the multi-terminal DC grid.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct DcBus {
    pub id: usize,
    pub bus_type: DcBusType,
    /// DC voltage in kV (state variable; updated during iteration).
    pub v_dc_kv: f64,
    /// Nominal DC voltage in kV.
    pub v_dc_nom_kv: f64,
    /// DC load power in MW.
    pub p_load_mw: f64,
}

impl DcBus {
    pub fn new(id: usize, bus_type: DcBusType, v_dc_nom_kv: f64) -> Self {
        Self {
            id,
            bus_type,
            v_dc_kv: v_dc_nom_kv,
            v_dc_nom_kv,
            p_load_mw: 0.0,
        }
    }
}

/// A DC cable or overhead line connecting two DC buses.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct DcBranch {
    pub from: usize,
    pub to: usize,
    pub resistance_ohm: f64,
    /// Series inductance in mH (for dynamic studies; not used in steady-state PF).
    pub inductance_mh: f64,
    pub current_rating_ka: f64,
    pub length_km: f64,
}

impl DcBranch {
    pub fn new(from: usize, to: usize, resistance_ohm: f64, length_km: f64) -> Self {
        Self {
            from,
            to,
            resistance_ohm,
            inductance_mh: 0.0,
            current_rating_ka: f64::INFINITY,
            length_km,
        }
    }

    pub fn conductance(&self) -> Result<f64, OxiGridError> {
        if self.resistance_ohm <= 0.0 {
            return Err(OxiGridError::InvalidParameter(format!(
                "DC branch ({}->{}) resistance must be positive, got {}",
                self.from, self.to, self.resistance_ohm
            )));
        }
        Ok(1.0 / self.resistance_ohm)
    }
}

/// Full AC/DC hybrid network data container.
#[derive(Debug, Clone)]
pub struct AcDcNetwork {
    pub n_ac_buses: usize,
    pub n_dc_buses: usize,
    /// AC conductance matrix G (n_ac × n_ac).
    pub ac_g: Vec<Vec<f64>>,
    /// AC susceptance matrix B (n_ac × n_ac).
    pub ac_b: Vec<Vec<f64>>,
    /// DC conductance matrix G_dc (n_dc × n_dc), built from DC branches.
    pub dc_g: Vec<Vec<f64>>,
    pub converters: Vec<AcDcConverter>,
    pub dc_buses: Vec<DcBus>,
    pub dc_branches: Vec<DcBranch>,
}

impl AcDcNetwork {
    pub fn new(
        n_ac_buses: usize,
        n_dc_buses: usize,
        ac_g: Vec<Vec<f64>>,
        ac_b: Vec<Vec<f64>>,
        converters: Vec<AcDcConverter>,
        dc_buses: Vec<DcBus>,
        dc_branches: Vec<DcBranch>,
    ) -> Result<Self, OxiGridError> {
        if ac_g.len() != n_ac_buses || ac_b.len() != n_ac_buses {
            return Err(OxiGridError::InvalidNetwork(
                "AC admittance matrix size mismatch".to_string(),
            ));
        }
        if dc_buses.len() != n_dc_buses {
            return Err(OxiGridError::InvalidNetwork(
                "DC bus list length does not match n_dc_buses".to_string(),
            ));
        }
        let dc_g = Self::build_dc_conductance_matrix(&dc_buses, &dc_branches)?;
        Ok(Self {
            n_ac_buses,
            n_dc_buses,
            ac_g,
            ac_b,
            dc_g,
            converters,
            dc_buses,
            dc_branches,
        })
    }

    /// Build the n_dc × n_dc nodal conductance matrix from DC branch data.
    pub fn build_dc_conductance_matrix(
        dc_buses: &[DcBus],
        dc_branches: &[DcBranch],
    ) -> Result<Vec<Vec<f64>>, OxiGridError> {
        let n = dc_buses.len();
        let mut g = vec![vec![0.0_f64; n]; n];
        for br in dc_branches {
            if br.from >= n || br.to >= n {
                return Err(OxiGridError::InvalidNetwork(format!(
                    "DC branch ({}->{}) references bus outside [0, {})",
                    br.from, br.to, n
                )));
            }
            let g_br = br.conductance()?;
            g[br.from][br.from] += g_br;
            g[br.to][br.to] += g_br;
            g[br.from][br.to] -= g_br;
            g[br.to][br.from] -= g_br;
        }
        Ok(g)
    }
}

/// Configuration for the unified NR AC/DC power flow solver.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct AcDcPfConfig {
    pub max_iterations: usize,
    pub tolerance_pu: f64,
    pub base_mva: f64,
    pub base_kv_ac: f64,
    pub base_kv_dc: f64,
}

impl Default for AcDcPfConfig {
    fn default() -> Self {
        Self {
            max_iterations: 50,
            tolerance_pu: 1e-6,
            base_mva: 100.0,
            base_kv_ac: 110.0,
            base_kv_dc: 320.0,
        }
    }
}

/// Result of the AC/DC unified NR power flow.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct AcDcPfResult {
    pub converged: bool,
    pub iterations: usize,
    /// AC bus voltage magnitudes in per-unit.
    pub ac_voltage_magnitude: Vec<f64>,
    /// AC bus voltage angles in radians.
    pub ac_voltage_angle: Vec<f64>,
    /// DC bus voltages in per-unit.
    pub dc_voltage: Vec<f64>,
    /// Converter AC-side active power in MW.
    pub converter_p_ac: Vec<f64>,
    /// Converter AC-side reactive power in MVAr.
    pub converter_q_ac: Vec<f64>,
    /// Converter DC-side power in MW.
    pub converter_p_dc: Vec<f64>,
    /// DC branch power flows in MW (positive = from→to).
    pub dc_branch_flows: Vec<f64>,
    pub total_ac_losses_mw: f64,
    pub total_dc_losses_mw: f64,
    pub total_converter_losses_mw: f64,
    pub max_mismatch: f64,
}

/// Unified Newton-Raphson AC/DC power flow solver.
pub struct AcDcPowerFlow {
    pub network: AcDcNetwork,
    pub config: AcDcPfConfig,
    pub v_ac: Vec<f64>,
    pub theta_ac: Vec<f64>,
    pub v_dc: Vec<f64>,
}

impl AcDcPowerFlow {
    pub fn new(network: AcDcNetwork, config: AcDcPfConfig) -> Self {
        let n_ac = network.n_ac_buses;
        let n_dc = network.n_dc_buses;
        Self {
            v_ac: vec![1.0; n_ac],
            theta_ac: vec![0.0; n_ac],
            v_dc: vec![1.0; n_dc],
            network,
            config,
        }
    }

    /// Run the unified NR AC/DC power flow.
    ///
    /// `p_ac_injections` / `q_ac_injections`: net P/Q at each AC bus in pu.
    /// `ac_bus_types`: 1=PQ, 2=PV, 3=Slack.
    pub fn solve(
        &mut self,
        p_ac_injections: &[f64],
        q_ac_injections: &[f64],
        ac_bus_types: &[u8],
    ) -> Result<AcDcPfResult, OxiGridError> {
        let n_ac = self.network.n_ac_buses;
        if p_ac_injections.len() != n_ac
            || q_ac_injections.len() != n_ac
            || ac_bus_types.len() != n_ac
        {
            return Err(OxiGridError::InvalidParameter(
                "injection / bus-type array length must equal n_ac_buses".to_string(),
            ));
        }

        let base_mva = self.config.base_mva;
        let mut converged = false;
        let mut iterations = 0;
        let mut max_mismatch = f64::MAX;

        for _iter in 0..self.config.max_iterations {
            iterations += 1;

            // Sync DC voltages kV ↔ pu
            self.sync_dc_voltages_to_buses();

            // Update converter operating points
            self.update_converter_operating_points();

            // Build per-iteration AC injection arrays including converter contributions
            let mut p_spec = p_ac_injections.to_vec();
            let mut q_spec = q_ac_injections.to_vec();
            for conv in &self.network.converters {
                if conv.ac_bus < n_ac {
                    p_spec[conv.ac_bus] += conv.p_ac_mw / base_mva;
                    q_spec[conv.ac_bus] += conv.q_ac_mvar / base_mva;
                }
            }

            let (dp_ac, dq_ac) = self.compute_ac_mismatches(&p_spec, &q_spec, ac_bus_types);
            let dp_dc = self.compute_dc_mismatches();

            // Assemble mismatch vector
            // Layout: [ΔP_ac(non-slack), ΔQ_ac(PQ only), ΔP_dc(non-slack DC)]
            let mut mismatch = Vec::new();
            for (i, &bt) in ac_bus_types.iter().enumerate() {
                if bt != 3 {
                    mismatch.push(dp_ac[i]);
                }
            }
            for (i, &bt) in ac_bus_types.iter().enumerate() {
                if bt == 1 {
                    mismatch.push(dq_ac[i]);
                }
            }
            for (k, bus) in self.network.dc_buses.iter().enumerate() {
                if bus.bus_type != DcBusType::Slack {
                    mismatch.push(dp_dc[k]);
                }
            }

            max_mismatch = mismatch.iter().map(|x| x.abs()).fold(0.0_f64, f64::max);

            if max_mismatch < self.config.tolerance_pu {
                converged = true;
                break;
            }

            let n_eq = mismatch.len();
            if n_eq == 0 {
                converged = true;
                break;
            }

            let mut jac = self.build_jacobian(ac_bus_types);
            if jac.len() != n_eq {
                return Err(OxiGridError::LinearAlgebra(format!(
                    "Jacobian row count {} != mismatch length {}",
                    jac.len(),
                    n_eq
                )));
            }

            let mut rhs = mismatch.clone();
            let dx = Self::solve_linear_system(&mut jac, &mut rhs)?;
            self.apply_update(&dx, ac_bus_types);
        }

        // Post-process
        let dc_branch_flows = self.compute_dc_flows();
        let (total_ac_losses_mw, total_dc_losses_mw, total_converter_losses_mw) =
            self.compute_losses(&dc_branch_flows);

        let base_kv_dc = self.config.base_kv_dc;
        let dc_voltage_pu: Vec<f64> = self
            .network
            .dc_buses
            .iter()
            .map(|b| {
                if base_kv_dc > 0.0 {
                    b.v_dc_kv / base_kv_dc
                } else {
                    b.v_dc_kv
                }
            })
            .collect();

        Ok(AcDcPfResult {
            converged,
            iterations,
            ac_voltage_magnitude: self.v_ac.clone(),
            ac_voltage_angle: self.theta_ac.clone(),
            dc_voltage: dc_voltage_pu,
            converter_p_ac: self.network.converters.iter().map(|c| c.p_ac_mw).collect(),
            converter_q_ac: self
                .network
                .converters
                .iter()
                .map(|c| c.q_ac_mvar)
                .collect(),
            converter_p_dc: self.network.converters.iter().map(|c| c.p_dc_mw).collect(),
            dc_branch_flows,
            total_ac_losses_mw,
            total_dc_losses_mw,
            total_converter_losses_mw,
            max_mismatch,
        })
    }

    /// Compute AC bus power mismatches.
    ///
    /// ΔP_i = P_spec_i − Σ_j V_i V_j (G_ij cos(θ_ij) + B_ij sin(θ_ij))
    /// ΔQ_i = Q_spec_i − Σ_j V_i V_j (G_ij sin(θ_ij) − B_ij cos(θ_ij))
    pub fn compute_ac_mismatches(
        &self,
        p_spec: &[f64],
        q_spec: &[f64],
        ac_bus_types: &[u8],
    ) -> (Vec<f64>, Vec<f64>) {
        let n = self.network.n_ac_buses;
        let mut dp = vec![0.0_f64; n];
        let mut dq = vec![0.0_f64; n];

        for i in 0..n {
            if ac_bus_types[i] == 3 {
                continue;
            }
            let vi = self.v_ac[i];
            let ti = self.theta_ac[i];
            let mut p_calc = 0.0_f64;
            let mut q_calc = 0.0_f64;
            for j in 0..n {
                let vj = self.v_ac[j];
                let dtheta = ti - self.theta_ac[j];
                let cos_ij = dtheta.cos();
                let sin_ij = dtheta.sin();
                let g_ij = self.network.ac_g[i][j];
                let b_ij = self.network.ac_b[i][j];
                p_calc += vi * vj * (g_ij * cos_ij + b_ij * sin_ij);
                q_calc += vi * vj * (g_ij * sin_ij - b_ij * cos_ij);
            }
            dp[i] = p_spec[i] - p_calc;
            if ac_bus_types[i] == 1 {
                dq[i] = q_spec[i] - q_calc;
            }
        }
        (dp, dq)
    }

    /// Compute DC bus power mismatches.
    ///
    /// ΔP_dc_k = P_dc_inj_k − Σ_j G_dc_kj · V_dc_k · V_dc_j   (all in pu)
    pub fn compute_dc_mismatches(&self) -> Vec<f64> {
        let n_dc = self.network.n_dc_buses;
        let base_mva = self.config.base_mva;
        let base_kv_dc = self.config.base_kv_dc;

        let v_dc_pu: Vec<f64> = self
            .network
            .dc_buses
            .iter()
            .map(|b| {
                if base_kv_dc > 0.0 {
                    b.v_dc_kv / base_kv_dc
                } else {
                    b.v_dc_kv
                }
            })
            .collect();

        // Z_base = V_base_dc^2 / S_base  [Ω]
        let z_base = if base_mva > 0.0 && base_kv_dc > 0.0 {
            base_kv_dc * base_kv_dc / base_mva
        } else {
            1.0
        };

        let mut dp_dc = vec![0.0_f64; n_dc];
        for k in 0..n_dc {
            let p_conv_pu: f64 = self
                .network
                .converters
                .iter()
                .filter(|c| c.dc_bus == k)
                .map(|c| c.p_dc_mw / base_mva)
                .sum();
            let p_load_pu = self.network.dc_buses[k].p_load_mw / base_mva;
            let p_inj_pu = p_conv_pu - p_load_pu;

            let mut p_calc_pu = 0.0_f64;
            for j in 0..n_dc {
                let g_pu = self.network.dc_g[k][j] * z_base;
                p_calc_pu += g_pu * v_dc_pu[k] * v_dc_pu[j];
            }
            dp_dc[k] = p_inj_pu - p_calc_pu;
        }
        dp_dc
    }

    /// Update converter DC-side power from AC-side power and losses.
    pub fn update_converter_operating_points(&mut self) {
        for conv in self.network.converters.iter_mut() {
            let p_loss = conv.losses_fraction * conv.p_ac_mw.abs();
            if conv.is_rectifier {
                conv.p_dc_mw = conv.p_ac_mw - p_loss;
            } else {
                conv.p_dc_mw = -(conv.p_ac_mw.abs()) - p_loss;
            }
            conv.q_ac_mvar = conv.q_ac_mvar.clamp(conv.q_min_mvar, conv.q_max_mvar);
        }
    }

    /// Build the full Jacobian for the unified NR system.
    pub fn build_jacobian(&self, ac_bus_types: &[u8]) -> Vec<Vec<f64>> {
        let n_ac = self.network.n_ac_buses;
        let pvpq: Vec<usize> = (0..n_ac).filter(|&i| ac_bus_types[i] != 3).collect();
        let pq: Vec<usize> = (0..n_ac).filter(|&i| ac_bus_types[i] == 1).collect();
        let dc_free: Vec<usize> = (0..self.network.n_dc_buses)
            .filter(|&k| self.network.dc_buses[k].bus_type != DcBusType::Slack)
            .collect();

        let n_pvpq = pvpq.len();
        let n_pq = pq.len();
        let n_dc_free = dc_free.len();
        let n_eq = n_pvpq + n_pq + n_dc_free;
        let n_vars = n_pvpq + n_pq + n_dc_free;

        let mut jac = vec![vec![0.0_f64; n_vars]; n_eq];

        // J11: ∂ΔP_i/∂θ_j
        for (ri, &i) in pvpq.iter().enumerate() {
            let vi = self.v_ac[i];
            let ti = self.theta_ac[i];
            for (ci, &j) in pvpq.iter().enumerate() {
                let val = if i == j {
                    let q_i = self.calc_q_bus(i);
                    -q_i - self.network.ac_b[i][i] * vi * vi
                } else {
                    let vj = self.v_ac[j];
                    let dth = ti - self.theta_ac[j];
                    vi * vj
                        * (self.network.ac_g[i][j] * dth.sin()
                            - self.network.ac_b[i][j] * dth.cos())
                };
                jac[ri][ci] = val;
            }
            // J12: ∂ΔP_i/∂V_j
            for (ci, &j) in pq.iter().enumerate() {
                let val = if i == j {
                    let p_i = self.calc_p_bus(i);
                    p_i / vi + self.network.ac_g[i][i] * vi
                } else {
                    let vj = self.v_ac[j];
                    let dth = ti - self.theta_ac[j];
                    vj * (self.network.ac_g[i][j] * dth.cos() + self.network.ac_b[i][j] * dth.sin())
                };
                jac[ri][n_pvpq + ci] = val;
            }
        }

        // J21: ∂ΔQ_i/∂θ_j,  J22: ∂ΔQ_i/∂V_j
        for (ri, &i) in pq.iter().enumerate() {
            let row = n_pvpq + ri;
            let vi = self.v_ac[i];
            let ti = self.theta_ac[i];
            for (ci, &j) in pvpq.iter().enumerate() {
                let val = if i == j {
                    let p_i = self.calc_p_bus(i);
                    p_i - self.network.ac_g[i][i] * vi * vi
                } else {
                    let vj = self.v_ac[j];
                    let dth = ti - self.theta_ac[j];
                    -vi * vj
                        * (self.network.ac_g[i][j] * dth.cos()
                            + self.network.ac_b[i][j] * dth.sin())
                };
                jac[row][ci] = val;
            }
            for (ci, &j) in pq.iter().enumerate() {
                let val = if i == j {
                    let q_i = self.calc_q_bus(i);
                    q_i / vi - self.network.ac_b[i][i] * vi
                } else {
                    let _vj = self.v_ac[j];
                    let dth = ti - self.theta_ac[j];
                    vi * (self.network.ac_g[i][j] * dth.sin() - self.network.ac_b[i][j] * dth.cos())
                };
                jac[row][n_pvpq + ci] = val;
            }
        }

        // DC block
        let base_kv_dc = self.config.base_kv_dc;
        let base_mva = self.config.base_mva;
        let z_base = if base_mva > 0.0 && base_kv_dc > 0.0 {
            base_kv_dc * base_kv_dc / base_mva
        } else {
            1.0
        };

        for (ri, &k) in dc_free.iter().enumerate() {
            let row = n_pvpq + n_pq + ri;
            let v_k_pu =
                self.network.dc_buses[k].v_dc_kv / if base_kv_dc > 0.0 { base_kv_dc } else { 1.0 };
            for (ci, &m) in dc_free.iter().enumerate() {
                let col = n_pvpq + n_pq + ci;
                let val = if k == m {
                    let mut diag = 0.0_f64;
                    for j in 0..self.network.n_dc_buses {
                        let v_j_pu = self.network.dc_buses[j].v_dc_kv
                            / if base_kv_dc > 0.0 { base_kv_dc } else { 1.0 };
                        diag += self.network.dc_g[k][j] * z_base * v_j_pu;
                    }
                    diag += self.network.dc_g[k][k] * z_base * v_k_pu;
                    -diag
                } else {
                    -self.network.dc_g[k][m] * z_base * v_k_pu
                };
                jac[row][col] = val;
            }
        }

        jac
    }

    /// Solve Ax = b using Gaussian elimination with partial pivoting.
    #[allow(clippy::ptr_arg, clippy::needless_range_loop)]
    pub fn solve_linear_system(
        a: &mut Vec<Vec<f64>>,
        b: &mut Vec<f64>,
    ) -> Result<Vec<f64>, OxiGridError> {
        let n = b.len();
        if a.len() != n {
            return Err(OxiGridError::LinearAlgebra(format!(
                "Matrix row count {} != RHS length {}",
                a.len(),
                n
            )));
        }
        for row in a.iter() {
            if row.len() != n {
                return Err(OxiGridError::LinearAlgebra(
                    "Matrix is not square".to_string(),
                ));
            }
        }

        for col in 0..n {
            let mut max_val = a[col][col].abs();
            let mut max_row = col;
            for row in (col + 1)..n {
                if a[row][col].abs() > max_val {
                    max_val = a[row][col].abs();
                    max_row = row;
                }
            }
            if max_val < 1e-14 {
                return Err(OxiGridError::LinearAlgebra(
                    "Singular or near-singular matrix in AC/DC power flow".to_string(),
                ));
            }
            if max_row != col {
                a.swap(col, max_row);
                b.swap(col, max_row);
            }
            let pivot = a[col][col];
            for row in (col + 1)..n {
                let factor = a[row][col] / pivot;
                a[row][col] = 0.0;
                for c in (col + 1)..n {
                    let sub = factor * a[col][c];
                    a[row][c] -= sub;
                }
                b[row] -= factor * b[col];
            }
        }

        let mut x = vec![0.0_f64; n];
        for row in (0..n).rev() {
            let mut s = b[row];
            for c in (row + 1)..n {
                s -= a[row][c] * x[c];
            }
            if a[row][row].abs() < 1e-14 {
                return Err(OxiGridError::LinearAlgebra(
                    "Zero diagonal in back-substitution".to_string(),
                ));
            }
            x[row] = s / a[row][row];
        }
        Ok(x)
    }

    /// Compute DC branch power flows in MW.
    /// P_branch = (V_dc_from − V_dc_to) / R_branch  [kV/Ω * 1e3 = MW]
    pub fn compute_dc_flows(&self) -> Vec<f64> {
        self.network
            .dc_branches
            .iter()
            .map(|br| {
                let v_from = self.network.dc_buses[br.from].v_dc_kv;
                let v_to = self.network.dc_buses[br.to].v_dc_kv;
                if br.resistance_ohm > 0.0 {
                    (v_from - v_to) / br.resistance_ohm * 1e3
                } else {
                    0.0
                }
            })
            .collect()
    }

    /// Compute (total_ac_losses_mw, total_dc_losses_mw, total_converter_losses_mw).
    pub fn compute_losses(&self, dc_branch_flows: &[f64]) -> (f64, f64, f64) {
        // DC line losses: P_loss = I^2 R,  I = P / V_avg
        let total_dc_losses_mw: f64 = self
            .network
            .dc_branches
            .iter()
            .zip(dc_branch_flows.iter())
            .map(|(br, &p_mw)| {
                let v_from = self.network.dc_buses[br.from].v_dc_kv;
                let v_to = self.network.dc_buses[br.to].v_dc_kv;
                let v_avg = 0.5 * (v_from + v_to).max(1.0);
                let i_ka = p_mw / v_avg;
                i_ka * i_ka * br.resistance_ohm * 1e-3
            })
            .sum();

        let total_converter_losses_mw: f64 = self
            .network
            .converters
            .iter()
            .map(|c| c.losses_fraction * c.p_ac_mw.abs())
            .sum();

        (0.0, total_dc_losses_mw, total_converter_losses_mw)
    }

    // -----------------------------------------------------------------------
    // Private helpers
    // -----------------------------------------------------------------------

    fn sync_dc_voltages_to_buses(&mut self) {
        let base_kv_dc = self.config.base_kv_dc;
        for (k, bus) in self.network.dc_buses.iter_mut().enumerate() {
            if bus.bus_type != DcBusType::Slack {
                bus.v_dc_kv = self.v_dc[k] * base_kv_dc;
            }
        }
    }

    fn calc_p_bus(&self, i: usize) -> f64 {
        let n = self.network.n_ac_buses;
        let vi = self.v_ac[i];
        let ti = self.theta_ac[i];
        let mut p = 0.0_f64;
        for j in 0..n {
            let dth = ti - self.theta_ac[j];
            p += vi
                * self.v_ac[j]
                * (self.network.ac_g[i][j] * dth.cos() + self.network.ac_b[i][j] * dth.sin());
        }
        p
    }

    fn calc_q_bus(&self, i: usize) -> f64 {
        let n = self.network.n_ac_buses;
        let vi = self.v_ac[i];
        let ti = self.theta_ac[i];
        let mut q = 0.0_f64;
        for j in 0..n {
            let dth = ti - self.theta_ac[j];
            q += vi
                * self.v_ac[j]
                * (self.network.ac_g[i][j] * dth.sin() - self.network.ac_b[i][j] * dth.cos());
        }
        q
    }

    fn apply_update(&mut self, dx: &[f64], ac_bus_types: &[u8]) {
        let n_ac = self.network.n_ac_buses;
        let pvpq: Vec<usize> = (0..n_ac).filter(|&i| ac_bus_types[i] != 3).collect();
        let pq: Vec<usize> = (0..n_ac).filter(|&i| ac_bus_types[i] == 1).collect();
        let dc_free: Vec<usize> = (0..self.network.n_dc_buses)
            .filter(|&k| self.network.dc_buses[k].bus_type != DcBusType::Slack)
            .collect();

        const MAX_DTHETA: f64 = 0.5;
        const MAX_DV: f64 = 0.1;
        const MAX_DV_DC: f64 = 0.1;

        for (idx, &i) in pvpq.iter().enumerate() {
            if idx < dx.len() {
                self.theta_ac[i] += dx[idx].clamp(-MAX_DTHETA, MAX_DTHETA);
            }
        }
        let pq_off = pvpq.len();
        for (idx, &i) in pq.iter().enumerate() {
            if pq_off + idx < dx.len() {
                let dv = dx[pq_off + idx].clamp(-MAX_DV, MAX_DV);
                self.v_ac[i] = (self.v_ac[i] * (1.0 + dv)).clamp(0.5, 1.5);
            }
        }
        let dc_off = pvpq.len() + pq.len();
        for (idx, &k) in dc_free.iter().enumerate() {
            if dc_off + idx < dx.len() {
                let dv = dx[dc_off + idx].clamp(-MAX_DV_DC, MAX_DV_DC);
                self.v_dc[k] = (self.v_dc[k] + dv).clamp(0.5, 1.5);
            }
        }
    }
}

// ===========================================================================
// Tests — original sequential solver tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::branch::Branch;
    use crate::network::bus::{Bus, BusType};

    fn make_branch(from: usize, to: usize, r: f64, x: f64) -> Branch {
        Branch {
            from_bus: from,
            to_bus: to,
            r,
            x,
            b: 0.0,
            rate_a: 0.0,
            rate_b: 0.0,
            rate_c: 0.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        }
    }

    fn make_bus(id: usize, bus_type: BusType, pd: f64, vm: f64) -> Bus {
        use crate::units::{Power, ReactivePower, Voltage};
        Bus {
            id,
            name: format!("Bus{id}"),
            bus_type,
            base_kv: Voltage(110.0),
            vm,
            va: 0.0,
            pd: Power(pd),
            qd: ReactivePower(0.0),
            gs: 0.0,
            bs: 0.0,
            zone: None,
        }
    }

    #[test]
    fn test_single_vsc_hvdc() {
        // Simple 2-AC-bus + 1-DC-bus + 1 SlackDc VSC
        let cfg = AcDcSequentialConfig {
            n_ac_buses: 2,
            n_dc_buses: 1,
            tolerance: 1e-4,
            max_iterations: 50,
            base_mva: 100.0,
        };
        let mut solver = AcDcPfSolver::new(cfg);
        solver.add_ac_bus(make_bus(1, BusType::Slack, 0.0, 1.0));
        solver.add_ac_bus(make_bus(2, BusType::PQ, 50.0, 1.0));
        solver.add_ac_branch(make_branch(1, 2, 0.01, 0.1));
        solver.add_dc_bus(VscDcBus {
            id: 0,
            v_dc_pu: 1.0,
            p_load_mw: 0.0,
        });
        solver.add_vsc(VscConverter {
            id: 0,
            ac_bus: 0,
            dc_bus: 0,
            mode: VscMode::SlackDc,
            p_set_mw: 30.0,
            v_ac_set_pu: 1.0,
            v_dc_set_pu: 1.0,
            p_loss_fraction: 0.01,
            q_min_mvar: -50.0,
            q_max_mvar: 50.0,
            rated_mva: 100.0,
        });
        let result = solver.solve();
        assert!(result.is_ok(), "single VSC HVDC should converge");
        let r = result.unwrap();
        assert!(r.converged);
        assert_eq!(r.dc_voltages.len(), 1);
        // DC slack voltage should be at setpoint
        assert!((r.dc_voltages[0] - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_back_to_back_converter() {
        // Two separate AC buses connected through a back-to-back VSC
        // (DC bus acts as intermediate; both VSCs connect to same DC bus)
        let cfg = AcDcSequentialConfig {
            n_ac_buses: 2,
            n_dc_buses: 1,
            tolerance: 1e-4,
            max_iterations: 50,
            base_mva: 100.0,
        };
        let mut solver = AcDcPfSolver::new(cfg);
        solver.add_ac_bus(make_bus(1, BusType::Slack, 0.0, 1.0));
        solver.add_ac_bus(make_bus(2, BusType::PQ, 20.0, 1.0));
        // No AC branch — buses are only coupled via VSCs
        solver.add_dc_bus(VscDcBus {
            id: 0,
            v_dc_pu: 1.0,
            p_load_mw: 0.0,
        });
        // Rectifier on bus 0 (absorbs from AC1)
        solver.add_vsc(VscConverter {
            id: 0,
            ac_bus: 0,
            dc_bus: 0,
            mode: VscMode::SlackDc,
            p_set_mw: -20.0,
            v_ac_set_pu: 1.0,
            v_dc_set_pu: 1.0,
            p_loss_fraction: 0.02,
            q_min_mvar: -30.0,
            q_max_mvar: 30.0,
            rated_mva: 50.0,
        });
        // Inverter on bus 1 (supplies AC2)
        solver.add_vsc(VscConverter {
            id: 1,
            ac_bus: 1,
            dc_bus: 0,
            mode: VscMode::PacVac,
            p_set_mw: 20.0,
            v_ac_set_pu: 1.0,
            v_dc_set_pu: 1.0,
            p_loss_fraction: 0.02,
            q_min_mvar: -30.0,
            q_max_mvar: 30.0,
            rated_mva: 50.0,
        });
        let result = solver.solve();
        assert!(
            result.is_ok(),
            "back-to-back should converge: {:?}",
            result.err()
        );
        let r = result.unwrap();
        assert!(r.converged);
        // AC voltages should exist for 2 buses
        assert_eq!(r.ac_voltages.len(), 2);
    }

    #[test]
    fn test_dc_slack_maintains_voltage() {
        // SlackDc VSC should hold DC voltage at its setpoint
        let cfg = AcDcSequentialConfig {
            n_ac_buses: 1,
            n_dc_buses: 2,
            tolerance: 1e-5,
            max_iterations: 50,
            base_mva: 100.0,
        };
        let mut solver = AcDcPfSolver::new(cfg);
        solver.add_ac_bus(make_bus(1, BusType::Slack, 0.0, 1.0));
        solver.add_dc_bus(VscDcBus {
            id: 0,
            v_dc_pu: 1.0,
            p_load_mw: 0.0,
        });
        solver.add_dc_bus(VscDcBus {
            id: 1,
            v_dc_pu: 0.98,
            p_load_mw: 50.0,
        });
        solver.add_dc_branch(VscDcBranch {
            from_bus: 0,
            to_bus: 1,
            resistance_pu: 0.02,
            rating_mw: 200.0,
        });
        solver.add_vsc(VscConverter {
            id: 0,
            ac_bus: 0,
            dc_bus: 0,
            mode: VscMode::SlackDc,
            p_set_mw: 50.0,
            v_ac_set_pu: 1.0,
            v_dc_set_pu: 1.02,
            p_loss_fraction: 0.01,
            q_min_mvar: -50.0,
            q_max_mvar: 50.0,
            rated_mva: 100.0,
        });
        let result = solver.solve().expect("DC slack test should converge");
        // DC slack bus voltage must equal setpoint
        assert!(
            (result.dc_voltages[0] - 1.02).abs() < 1e-9,
            "DC slack voltage should be 1.02 pu, got {}",
            result.dc_voltages[0]
        );
    }

    #[test]
    fn test_converter_losses_reduce_dc_power() {
        // With 5% losses, DC power should be less than AC power for rectifier
        let cfg = AcDcSequentialConfig {
            n_ac_buses: 2,
            n_dc_buses: 1,
            tolerance: 1e-4,
            max_iterations: 50,
            base_mva: 100.0,
        };
        let mut solver = AcDcPfSolver::new(cfg);
        solver.add_ac_bus(make_bus(1, BusType::Slack, 0.0, 1.0));
        solver.add_ac_bus(make_bus(2, BusType::PQ, 40.0, 1.0));
        solver.add_ac_branch(make_branch(1, 2, 0.01, 0.05));
        solver.add_dc_bus(VscDcBus {
            id: 0,
            v_dc_pu: 1.0,
            p_load_mw: 0.0,
        });
        let loss_frac = 0.05;
        solver.add_vsc(VscConverter {
            id: 0,
            ac_bus: 0,
            dc_bus: 0,
            mode: VscMode::SlackDc,
            p_set_mw: -40.0, // rectifier: absorbing from AC
            v_ac_set_pu: 1.0,
            v_dc_set_pu: 1.0,
            p_loss_fraction: loss_frac,
            q_min_mvar: -50.0,
            q_max_mvar: 50.0,
            rated_mva: 100.0,
        });
        let result = solver.solve().expect("loss test should converge");
        // |P_dc| < |P_ac| because of losses
        let p_ac = result.vsc_p_ac_mw[0].abs();
        let p_dc = result.vsc_p_dc_mw[0].abs();
        assert!(
            p_dc <= p_ac + 1e-6,
            "DC power ({p_dc}) should not exceed AC power ({p_ac}) for rectifier with losses"
        );
        assert!(result.total_converter_losses_mw >= 0.0);
    }

    #[test]
    fn test_multi_terminal_dc() {
        // 3-bus DC network with 3 VSCs and 3 DC branches forming a ring
        let cfg = AcDcSequentialConfig {
            n_ac_buses: 3,
            n_dc_buses: 3,
            tolerance: 1e-4,
            max_iterations: 50,
            base_mva: 100.0,
        };
        let mut solver = AcDcPfSolver::new(cfg);
        solver.add_ac_bus(make_bus(1, BusType::Slack, 0.0, 1.0));
        solver.add_ac_bus(make_bus(2, BusType::PQ, 30.0, 1.0));
        solver.add_ac_bus(make_bus(3, BusType::PQ, 20.0, 1.0));
        solver.add_ac_branch(make_branch(1, 2, 0.01, 0.05));
        solver.add_ac_branch(make_branch(2, 3, 0.01, 0.05));
        solver.add_dc_bus(VscDcBus {
            id: 0,
            v_dc_pu: 1.0,
            p_load_mw: 0.0,
        });
        solver.add_dc_bus(VscDcBus {
            id: 1,
            v_dc_pu: 1.0,
            p_load_mw: 0.0,
        });
        solver.add_dc_bus(VscDcBus {
            id: 2,
            v_dc_pu: 1.0,
            p_load_mw: 0.0,
        });
        solver.add_dc_branch(VscDcBranch {
            from_bus: 0,
            to_bus: 1,
            resistance_pu: 0.01,
            rating_mw: 200.0,
        });
        solver.add_dc_branch(VscDcBranch {
            from_bus: 1,
            to_bus: 2,
            resistance_pu: 0.01,
            rating_mw: 200.0,
        });
        solver.add_dc_branch(VscDcBranch {
            from_bus: 0,
            to_bus: 2,
            resistance_pu: 0.02,
            rating_mw: 200.0,
        });
        solver.add_vsc(VscConverter {
            id: 0,
            ac_bus: 0,
            dc_bus: 0,
            mode: VscMode::SlackDc,
            p_set_mw: 50.0,
            v_ac_set_pu: 1.0,
            v_dc_set_pu: 1.0,
            p_loss_fraction: 0.01,
            q_min_mvar: -100.0,
            q_max_mvar: 100.0,
            rated_mva: 200.0,
        });
        solver.add_vsc(VscConverter {
            id: 1,
            ac_bus: 1,
            dc_bus: 1,
            mode: VscMode::PdcVdc,
            p_set_mw: -30.0,
            v_ac_set_pu: 1.0,
            v_dc_set_pu: 1.0,
            p_loss_fraction: 0.01,
            q_min_mvar: -50.0,
            q_max_mvar: 50.0,
            rated_mva: 100.0,
        });
        solver.add_vsc(VscConverter {
            id: 2,
            ac_bus: 2,
            dc_bus: 2,
            mode: VscMode::PdcVdc,
            p_set_mw: -20.0,
            v_ac_set_pu: 1.0,
            v_dc_set_pu: 1.0,
            p_loss_fraction: 0.01,
            q_min_mvar: -50.0,
            q_max_mvar: 50.0,
            rated_mva: 100.0,
        });
        let result = solver.solve();
        assert!(
            result.is_ok(),
            "multi-terminal DC should converge: {:?}",
            result.err()
        );
        let r = result.unwrap();
        assert!(r.converged);
        assert_eq!(r.dc_voltages.len(), 3);
        assert_eq!(r.dc_line_flows_mw.len(), 3);
    }

    // =======================================================================
    // Required 20 tests for AcDcPowerFlow / unified NR API
    // =======================================================================

    fn two_bus_ac_ybus(x: f64) -> (Vec<Vec<f64>>, Vec<Vec<f64>>) {
        let b = 1.0 / x;
        let g = vec![vec![0.0_f64; 2]; 2];
        let bm = vec![vec![b, -b], vec![-b, b]];
        (g, bm)
    }

    fn three_bus_ac_ybus() -> (Vec<Vec<f64>>, Vec<Vec<f64>>) {
        let b = 10.0_f64;
        let g = vec![vec![0.0_f64; 3]; 3];
        let mut bm = vec![vec![0.0_f64; 3]; 3];
        for (i, j) in [(0usize, 1usize), (1, 2), (0, 2)] {
            bm[i][i] += b;
            bm[j][j] += b;
            bm[i][j] -= b;
            bm[j][i] -= b;
        }
        (g, bm)
    }

    #[test]
    fn test_dc_bus_creation() {
        let bus = DcBus::new(3, DcBusType::PQ, 320.0);
        assert_eq!(bus.id, 3);
        assert_eq!(bus.bus_type, DcBusType::PQ);
        assert!((bus.v_dc_kv - 320.0).abs() < 1e-9);
        assert!((bus.v_dc_nom_kv - 320.0).abs() < 1e-9);
        assert_eq!(bus.p_load_mw, 0.0);
    }

    #[test]
    fn test_dc_branch_creation() {
        let br = DcBranch::new(0, 1, 8.0, 150.0);
        assert_eq!(br.from, 0);
        assert_eq!(br.to, 1);
        assert!((br.resistance_ohm - 8.0).abs() < 1e-9);
        assert!((br.length_km - 150.0).abs() < 1e-9);
        let g = br.conductance().expect("conductance ok");
        assert!((g - 0.125).abs() < 1e-9);
    }

    #[test]
    fn test_converter_creation() {
        let conv = AcDcConverter::new(1, 2, 0, ConverterType::PV, 120.0, -10.0, 320.0);
        assert_eq!(conv.id, 1);
        assert_eq!(conv.ac_bus, 2);
        assert_eq!(conv.dc_bus, 0);
        assert!((conv.p_ref_mw - 120.0).abs() < 1e-9);
        assert!((conv.losses_fraction - 0.02).abs() < 1e-9);
        assert!(conv.is_rectifier);
    }

    #[test]
    fn test_ac_dc_network_build() {
        let (g, b) = two_bus_ac_ybus(0.1);
        let dc_buses = vec![DcBus::new(0, DcBusType::Slack, 320.0)];
        let conv = AcDcConverter::new(0, 0, 0, ConverterType::VdcQ, 0.0, 0.0, 320.0);
        let net = AcDcNetwork::new(2, 1, g, b, vec![conv], dc_buses, vec![]).expect("build ok");
        assert_eq!(net.n_ac_buses, 2);
        assert_eq!(net.n_dc_buses, 1);
        assert_eq!(net.converters.len(), 1);
        assert_eq!(net.dc_g.len(), 1);
    }

    #[test]
    fn test_build_dc_conductance_2bus() {
        let dc_buses = vec![
            DcBus::new(0, DcBusType::Slack, 320.0),
            DcBus::new(1, DcBusType::PQ, 320.0),
        ];
        let branches = vec![DcBranch::new(0, 1, 10.0, 200.0)];
        let g = AcDcNetwork::build_dc_conductance_matrix(&dc_buses, &branches).expect("ok");
        let g_br = 0.1_f64;
        assert!((g[0][0] - g_br).abs() < 1e-12);
        assert!((g[1][1] - g_br).abs() < 1e-12);
        assert!((g[0][1] + g_br).abs() < 1e-12);
        assert!((g[1][0] + g_br).abs() < 1e-12);
    }

    #[test]
    fn test_build_dc_conductance_3bus() {
        let dc_buses: Vec<DcBus> = (0..3)
            .map(|i| DcBus::new(i, DcBusType::PQ, 320.0))
            .collect();
        let branches = vec![
            DcBranch::new(0, 1, 10.0, 100.0),
            DcBranch::new(1, 2, 5.0, 100.0),
            DcBranch::new(0, 2, 20.0, 100.0),
        ];
        let g = AcDcNetwork::build_dc_conductance_matrix(&dc_buses, &branches).expect("ok");
        // Row sums must be zero (Kirchhoff)
        for (row_i, row) in g.iter().enumerate() {
            let s: f64 = row.iter().sum();
            assert!(s.abs() < 1e-11, "Row {row_i} sum = {s}, expected ~0");
        }
    }

    #[test]
    fn test_dc_mismatch_balanced() {
        let dc_buses = vec![DcBus::new(0, DcBusType::Slack, 320.0)];
        let (g, b) = two_bus_ac_ybus(0.1);
        let net = AcDcNetwork::new(2, 1, g, b, vec![], dc_buses, vec![]).expect("ok");
        let cfg = AcDcPfConfig::default();
        let pf = AcDcPowerFlow::new(net, cfg);
        let dp = pf.compute_dc_mismatches();
        assert!((dp[0]).abs() < 1e-9, "Slack bus mismatch should be zero");
    }

    #[test]
    fn test_dc_mismatch_imbalanced() {
        let mut dc_buses = vec![
            DcBus::new(0, DcBusType::Slack, 320.0),
            DcBus::new(1, DcBusType::PQ, 320.0),
        ];
        dc_buses[1].p_load_mw = 100.0;
        let branches = vec![DcBranch::new(0, 1, 10.0, 200.0)];
        let (g, b) = two_bus_ac_ybus(0.1);
        let net = AcDcNetwork::new(2, 2, g, b, vec![], dc_buses, branches).expect("ok");
        let cfg = AcDcPfConfig::default();
        let pf = AcDcPowerFlow::new(net, cfg);
        let dp = pf.compute_dc_mismatches();
        assert!(
            dp[1].abs() > 1e-9,
            "Loaded bus should have non-zero mismatch"
        );
    }

    #[test]
    fn test_linear_system_solver_2x2() {
        // 2x + 3y = 8,  4x + y = 6  → x=1, y=2
        let mut a = vec![vec![2.0, 3.0], vec![4.0, 1.0]];
        let mut b = vec![8.0, 6.0];
        let x = AcDcPowerFlow::solve_linear_system(&mut a, &mut b).expect("solved");
        assert!((x[0] - 1.0).abs() < 1e-9, "x[0]={}", x[0]);
        assert!((x[1] - 2.0).abs() < 1e-9, "x[1]={}", x[1]);
    }

    #[test]
    fn test_linear_system_solver_3x3() {
        let mut a = vec![
            vec![1.0, 2.0, -1.0],
            vec![2.0, 1.0, 3.0],
            vec![-1.0, 3.0, 2.0],
        ];
        let mut b = vec![1.0, 13.0, 4.0];
        let x = AcDcPowerFlow::solve_linear_system(&mut a, &mut b).expect("solved");
        let res = [
            (1.0 * x[0] + 2.0 * x[1] - 1.0 * x[2] - 1.0).abs(),
            (2.0 * x[0] + 1.0 * x[1] + 3.0 * x[2] - 13.0).abs(),
            (-x[0] + 3.0 * x[1] + 2.0 * x[2] - 4.0).abs(),
        ];
        for r in res {
            assert!(r < 1e-9, "residual={r}");
        }
    }

    #[test]
    fn test_solve_pure_dc_2bus() {
        let mut dc_buses = vec![
            DcBus::new(0, DcBusType::Slack, 320.0),
            DcBus::new(1, DcBusType::PQ, 320.0),
        ];
        dc_buses[1].p_load_mw = 100.0;
        let branches = vec![DcBranch::new(0, 1, 5.0, 100.0)];
        let g_ac = vec![vec![0.0_f64]];
        let b_ac = vec![vec![0.0_f64]];
        let conv = AcDcConverter::new(0, 0, 0, ConverterType::VdcQ, 0.0, 0.0, 320.0);
        let net = AcDcNetwork::new(1, 2, g_ac, b_ac, vec![conv], dc_buses, branches).expect("ok");
        let cfg = AcDcPfConfig {
            max_iterations: 50,
            tolerance_pu: 1e-6,
            ..Default::default()
        };
        let mut pf = AcDcPowerFlow::new(net, cfg);
        let result = pf.solve(&[0.0], &[0.0], &[3]).expect("solve ok");
        assert!(result.iterations > 0);
        assert_eq!(result.dc_branch_flows.len(), 1);
        assert_eq!(result.dc_voltage.len(), 2);
    }

    #[test]
    fn test_solve_pure_dc_3bus() {
        let mut dc_buses: Vec<DcBus> = (0..3)
            .map(|i| {
                DcBus::new(
                    i,
                    if i == 0 {
                        DcBusType::Slack
                    } else {
                        DcBusType::PQ
                    },
                    320.0,
                )
            })
            .collect();
        dc_buses[1].p_load_mw = 50.0;
        dc_buses[2].p_load_mw = 80.0;
        let branches = vec![
            DcBranch::new(0, 1, 5.0, 100.0),
            DcBranch::new(1, 2, 5.0, 100.0),
            DcBranch::new(0, 2, 10.0, 200.0),
        ];
        let g_ac = vec![vec![0.0_f64]];
        let b_ac = vec![vec![0.0_f64]];
        let conv = AcDcConverter::new(0, 0, 0, ConverterType::VdcQ, 0.0, 0.0, 320.0);
        let net = AcDcNetwork::new(1, 3, g_ac, b_ac, vec![conv], dc_buses, branches).expect("ok");
        let cfg = AcDcPfConfig {
            max_iterations: 50,
            tolerance_pu: 1e-6,
            ..Default::default()
        };
        let mut pf = AcDcPowerFlow::new(net, cfg);
        let result = pf.solve(&[0.0], &[0.0], &[3]).expect("solve ok");
        assert_eq!(result.dc_branch_flows.len(), 3);
        assert_eq!(result.dc_voltage.len(), 3);
    }

    #[test]
    fn test_solve_2bus_ac_with_converter() {
        let (g_ac, b_ac) = two_bus_ac_ybus(0.1);
        let dc_buses = vec![DcBus::new(0, DcBusType::Slack, 320.0)];
        let conv = AcDcConverter::new(0, 1, 0, ConverterType::PQ, 50.0, 0.0, 320.0);
        let net = AcDcNetwork::new(2, 1, g_ac, b_ac, vec![conv], dc_buses, vec![]).expect("ok");
        let cfg = AcDcPfConfig::default();
        let mut pf = AcDcPowerFlow::new(net, cfg);
        let result = pf.solve(&[0.0, -0.5], &[0.0, -0.2], &[3, 1]).expect("ok");
        assert_eq!(result.ac_voltage_magnitude.len(), 2);
        assert_eq!(result.ac_voltage_angle.len(), 2);
        assert_eq!(result.converter_p_ac.len(), 1);
        assert_eq!(result.converter_p_dc.len(), 1);
    }

    #[test]
    fn test_solve_3bus_hybrid() {
        let (g_ac, b_ac) = three_bus_ac_ybus();
        let mut dc_buses = vec![
            DcBus::new(0, DcBusType::Slack, 320.0),
            DcBus::new(1, DcBusType::PQ, 320.0),
        ];
        dc_buses[1].p_load_mw = 80.0;
        let dc_branches = vec![DcBranch::new(0, 1, 8.0, 150.0)];
        let c0 = AcDcConverter::new(0, 0, 0, ConverterType::VdcQ, 0.0, 0.0, 320.0);
        let c1 = AcDcConverter::new(1, 2, 1, ConverterType::PQ, -80.0, 0.0, 320.0);
        let net =
            AcDcNetwork::new(3, 2, g_ac, b_ac, vec![c0, c1], dc_buses, dc_branches).expect("ok");
        let cfg = AcDcPfConfig::default();
        let mut pf = AcDcPowerFlow::new(net, cfg);
        let result = pf
            .solve(&[0.0, -0.3, 0.8], &[0.0, -0.1, 0.0], &[3, 1, 2])
            .expect("ok");
        assert_eq!(result.converter_p_ac.len(), 2);
        assert_eq!(result.converter_q_ac.len(), 2);
        assert_eq!(result.dc_branch_flows.len(), 1);
    }

    #[test]
    fn test_converter_loss_computation() {
        let (g_ac, b_ac) = two_bus_ac_ybus(0.1);
        let mut conv = AcDcConverter::new(0, 0, 0, ConverterType::PQ, 100.0, 0.0, 320.0);
        conv.is_rectifier = true;
        let dc_buses = vec![DcBus::new(0, DcBusType::Slack, 320.0)];
        let net = AcDcNetwork::new(2, 1, g_ac, b_ac, vec![conv], dc_buses, vec![]).expect("ok");
        let cfg = AcDcPfConfig::default();
        let mut pf = AcDcPowerFlow::new(net, cfg);
        pf.update_converter_operating_points();
        let c = &pf.network.converters[0];
        // p_dc = 100 − 2% * 100 = 98 MW
        assert!((c.p_dc_mw - 98.0).abs() < 1e-9, "p_dc={}", c.p_dc_mw);
    }

    #[test]
    fn test_dc_branch_flows_direction() {
        let mut dc_buses = vec![
            DcBus::new(0, DcBusType::Slack, 330.0),
            DcBus::new(1, DcBusType::PQ, 310.0),
        ];
        dc_buses[0].v_dc_kv = 330.0;
        dc_buses[1].v_dc_kv = 310.0;
        let branches = vec![DcBranch::new(0, 1, 10.0, 200.0)];
        let g_ac = vec![vec![0.0_f64]];
        let b_ac = vec![vec![0.0_f64]];
        let net = AcDcNetwork::new(1, 2, g_ac, b_ac, vec![], dc_buses, branches).expect("ok");
        let cfg = AcDcPfConfig::default();
        let pf = AcDcPowerFlow::new(net, cfg);
        let flows = pf.compute_dc_flows();
        assert_eq!(flows.len(), 1);
        assert!(
            flows[0] > 0.0,
            "Expected positive flow from higher to lower voltage"
        );
    }

    #[test]
    fn test_total_dc_losses() {
        let mut dc_buses = vec![
            DcBus::new(0, DcBusType::Slack, 320.0),
            DcBus::new(1, DcBusType::PQ, 310.0),
        ];
        dc_buses[0].v_dc_kv = 320.0;
        dc_buses[1].v_dc_kv = 310.0;
        let branches = vec![DcBranch::new(0, 1, 10.0, 200.0)];
        let g_ac = vec![vec![0.0_f64]];
        let b_ac = vec![vec![0.0_f64]];
        let net = AcDcNetwork::new(1, 2, g_ac, b_ac, vec![], dc_buses, branches).expect("ok");
        let cfg = AcDcPfConfig::default();
        let pf = AcDcPowerFlow::new(net, cfg);
        let flows = pf.compute_dc_flows();
        let (_, dc_losses, _) = pf.compute_losses(&flows);
        assert!(dc_losses >= 0.0, "DC losses must be non-negative");
    }

    #[test]
    fn test_total_converter_losses() {
        let (g_ac, b_ac) = two_bus_ac_ybus(0.1);
        let conv = AcDcConverter::new(0, 0, 0, ConverterType::PQ, 200.0, 0.0, 320.0);
        let dc_buses = vec![DcBus::new(0, DcBusType::Slack, 320.0)];
        let net = AcDcNetwork::new(2, 1, g_ac, b_ac, vec![conv], dc_buses, vec![]).expect("ok");
        let cfg = AcDcPfConfig::default();
        let pf = AcDcPowerFlow::new(net, cfg);
        let (_, _, conv_losses) = pf.compute_losses(&[]);
        // 2% of 200 MW = 4 MW
        assert!(
            (conv_losses - 4.0).abs() < 1e-9,
            "conv_losses={conv_losses}"
        );
    }

    #[test]
    fn test_convergence_tolerance() {
        let (g_ac, b_ac) = two_bus_ac_ybus(0.05);
        let dc_buses = vec![DcBus::new(0, DcBusType::Slack, 320.0)];
        let conv = AcDcConverter::new(0, 1, 0, ConverterType::PQ, 0.0, 0.0, 320.0);
        let net = AcDcNetwork::new(2, 1, g_ac, b_ac, vec![conv], dc_buses, vec![]).expect("ok");
        let cfg = AcDcPfConfig {
            max_iterations: 100,
            tolerance_pu: 1e-10,
            ..Default::default()
        };
        let mut pf = AcDcPowerFlow::new(net, cfg);
        let result = pf.solve(&[0.0, 0.0], &[0.0, 0.0], &[3, 1]);
        assert!(result.is_ok(), "Must not error even with tight tolerance");
        let r = result.unwrap();
        assert!(r.iterations <= 100);
    }

    #[test]
    fn test_acdc_result_struct() {
        let (g_ac, b_ac) = two_bus_ac_ybus(0.1);
        let dc_buses = vec![DcBus::new(0, DcBusType::Slack, 320.0)];
        let conv = AcDcConverter::new(0, 0, 0, ConverterType::VdcQ, 0.0, 0.0, 320.0);
        let net = AcDcNetwork::new(2, 1, g_ac, b_ac, vec![conv], dc_buses, vec![]).expect("ok");
        let cfg = AcDcPfConfig::default();
        let mut pf = AcDcPowerFlow::new(net, cfg);
        let result = pf.solve(&[0.0, 0.0], &[0.0, 0.0], &[3, 1]).expect("ok");
        assert_eq!(result.ac_voltage_magnitude.len(), 2);
        assert_eq!(result.ac_voltage_angle.len(), 2);
        assert_eq!(result.dc_voltage.len(), 1);
        assert_eq!(result.converter_p_ac.len(), 1);
        assert_eq!(result.converter_q_ac.len(), 1);
        assert_eq!(result.converter_p_dc.len(), 1);
        assert_eq!(result.dc_branch_flows.len(), 0);
        assert!(result.total_converter_losses_mw >= 0.0);
        assert!(result.max_mismatch >= 0.0);
    }
}
