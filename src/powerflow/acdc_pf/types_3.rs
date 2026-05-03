//! Auto-generated module
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

use crate::error::OxiGridError;
use thiserror::Error;

use super::functions::gaussian_elimination;
use super::types::{
    AcBranchData, AcDcConverter, AcDcSequentialResult, DcBranch, DcBus, VscConverter, VscDcBranch,
    VscDcBus, VscMode,
};

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
        if n_dc > 0 && self.dc_branches.is_empty() && n_dc > 1 {
            return Err(AcDcError::InvalidConfig(
                "multi-bus DC network requires DC branches".to_string(),
            ));
        }
        let mut v_ac_mag: Vec<f64> = self.ac_buses.iter().map(|b| b.v_mag).collect();
        let mut v_ac_ang: Vec<f64> = self.ac_buses.iter().map(|b| b.v_ang).collect();
        let mut v_dc: Vec<f64> = self.dc_buses.iter().map(|b| b.v_dc_pu).collect();
        let mut vsc_p_ac_pu: Vec<f64> = vec![0.0; n_vsc];
        let mut vsc_q_ac_pu: Vec<f64> = vec![0.0; n_vsc];
        let mut vsc_p_dc_pu: Vec<f64> = vec![0.0; n_vsc];
        for (k, vsc) in self.vsc_converters.iter().enumerate() {
            vsc_p_ac_pu[k] = vsc.p_set_mw / base_mva;
            vsc_q_ac_pu[k] = 0.0;
        }
        let mut converged = false;
        let mut iterations = 0usize;
        for _iter in 0..self.config.max_iterations {
            iterations += 1;
            self.solve_ac_gauss_seidel(
                &mut v_ac_mag,
                &mut v_ac_ang,
                &vsc_p_ac_pu,
                &vsc_q_ac_pu,
                30,
            )?;
            for (k, vsc) in self.vsc_converters.iter().enumerate() {
                let p_ac = vsc_p_ac_pu[k];
                vsc_p_dc_pu[k] = if p_ac >= 0.0 {
                    p_ac / (1.0 - vsc.p_loss_fraction.clamp(0.0, 0.5))
                } else {
                    p_ac * (1.0 - vsc.p_loss_fraction.clamp(0.0, 0.5))
                };
            }
            if n_dc > 0 {
                self.solve_dc_network(&mut v_dc, &vsc_p_dc_pu, &v_ac_mag)?;
            }
            for (k, vsc) in self.vsc_converters.iter().enumerate() {
                let vsc_dc = if vsc.dc_bus < v_dc.len() {
                    v_dc[vsc.dc_bus]
                } else {
                    1.0
                };
                match vsc.mode {
                    VscMode::SlackDc | VscMode::PacVac => {
                        vsc_p_ac_pu[k] = vsc.p_set_mw / base_mva;
                    }
                    VscMode::PdcVdc => {
                        let p_dc = vsc_p_dc_pu[k];
                        vsc_p_ac_pu[k] = if p_dc >= 0.0 {
                            p_dc * (1.0 - vsc.p_loss_fraction.clamp(0.0, 0.5))
                        } else {
                            p_dc / (1.0 - vsc.p_loss_fraction.clamp(0.0, 0.5))
                        };
                    }
                    VscMode::Droop => {
                        let droop_gain = 10.0;
                        let dv = vsc_dc - vsc.v_dc_set_pu;
                        vsc_p_ac_pu[k] = vsc.p_set_mw / base_mva - droop_gain * dv;
                    }
                }
                let q_raw = vsc_q_ac_pu[k];
                let q_min = vsc.q_min_mvar / base_mva;
                let q_max = vsc.q_max_mvar / base_mva;
                vsc_q_ac_pu[k] = q_raw.max(q_min).min(q_max);
            }
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
                if br.resistance_pu > 1e-12 {
                    (v_from - v_to) / br.resistance_pu * base_mva
                } else {
                    0.0
                }
            })
            .collect();
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
        for _inner in 0..max_inner {
            for i in 0..n {
                let bus = &self.ac_buses[i];
                if bus.bus_type == AcBusType::Slack {
                    v_mag[i] = bus.v_set_pu;
                    v_ang[i] = 0.0;
                    continue;
                }
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
                let g_ii = g_bus[i][i];
                let b_ii = b_bus[i][i];
                let denom = g_ii * g_ii + b_ii * b_ii;
                if denom < 1e-20 {
                    continue;
                }
                let vi_re = vi_mag * vi_ang.cos();
                let vi_im = vi_mag * vi_ang.sin();
                let pq_re = (p_i * vi_re + q_i * vi_im) / (vi_mag * vi_mag);
                let pq_im = (p_i * vi_im - q_i * vi_re) / (vi_mag * vi_mag);
                let rhs_re = pq_re - sum_g;
                let rhs_im = pq_im - sum_b;
                let new_re = (rhs_re * g_ii + rhs_im * b_ii) / denom;
                let new_im = (rhs_im * g_ii - rhs_re * b_ii) / denom;
                let new_mag = (new_re * new_re + new_im * new_im).sqrt();
                let new_ang = new_im.atan2(new_re);
                if bus.bus_type == AcBusType::Pv {
                    v_mag[i] = bus.v_set_pu;
                    v_ang[i] = new_ang;
                } else {
                    v_mag[i] = new_mag.clamp(0.5, 1.5);
                    v_ang[i] = new_ang;
                }
            }
        }
        let _ = base_mva;
        Ok(())
    }
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
        let slack_dc_bus = self
            .vsc_converters
            .iter()
            .find(|v| v.mode == VscMode::SlackDc)
            .map(|v| v.dc_bus);
        let slack_idx = slack_dc_bus.unwrap_or(0);
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
        let mut i_dc = vec![0.0f64; n];
        for (k, vsc) in self.vsc_converters.iter().enumerate() {
            if vsc.dc_bus < n {
                let v_k = v_dc[vsc.dc_bus].max(0.5);
                i_dc[vsc.dc_bus] += -vsc_p_dc_pu[k] / v_k;
            }
        }
        for bus in &self.dc_buses {
            if bus.id < n && bus.p_load_mw.abs() > 1e-12 {
                let v_k = v_dc[bus.id].max(0.5);
                i_dc[bus.id] -= bus.p_load_mw / base_mva / v_k;
            }
        }
        let non_slack: Vec<usize> = (0..n).filter(|&i| i != slack_idx).collect();
        let m = non_slack.len();
        if m == 0 {
            v_dc[slack_idx] = self
                .vsc_converters
                .iter()
                .find(|v| v.dc_bus == slack_idx && v.mode == VscMode::SlackDc)
                .map(|v| v.v_dc_set_pu)
                .unwrap_or(v_dc[slack_idx]);
            return Ok(());
        }
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
            rhs[ri] -= g_dc[i][slack_idx] * v_slack;
            for (rj, &j) in non_slack.iter().enumerate() {
                g_red[ri][rj] = g_dc[i][j];
            }
        }
        let v_sol = gaussian_elimination(&g_red, &rhs).ok_or(AcDcError::SingularMatrix)?;
        v_dc[slack_idx] = v_slack;
        for (ri, &i) in non_slack.iter().enumerate() {
            v_dc[i] = v_sol[ri].clamp(0.5, 1.5);
        }
        Ok(())
    }
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
#[derive(Debug, Clone, Copy, PartialEq)]
enum AcBusType {
    Slack,
    Pv,
    Pq,
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
