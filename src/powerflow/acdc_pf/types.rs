//! Auto-generated module
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

use crate::error::OxiGridError;

use super::types_3::{AcDcNetwork, AcDcPfResult};

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
/// Configuration for the unified NR AC/DC power flow solver.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct AcDcPfConfig {
    pub max_iterations: usize,
    pub tolerance_pu: f64,
    pub base_mva: f64,
    pub base_kv_ac: f64,
    pub base_kv_dc: f64,
}
/// Converged solution from the sequential AC/DC hybrid power flow (`AcDcPfSolver`).
#[derive(Debug, Clone)]
pub struct AcDcSequentialResult {
    /// AC bus voltages: `(magnitude `pu`, angle `rad`)` for each bus.
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
/// Lightweight AC branch for internal Y-bus construction.
#[derive(Debug, Clone)]
pub(crate) struct AcBranchData {
    pub(crate) from: usize,
    pub(crate) to: usize,
    /// Series conductance \[pu\].
    pub(crate) g: f64,
    /// Series susceptance \[pu\].
    pub(crate) b: f64,
    /// Half-line charging susceptance \[pu\].
    pub(crate) b_half: f64,
}
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
            self.sync_dc_voltages_to_buses();
            self.update_converter_operating_points();
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dc_branch_new_defaults() {
        let br = DcBranch::new(0, 1, 1.0, 10.0);
        assert!((br.inductance_mh - 0.0).abs() < 1e-9);
        assert!(br.current_rating_ka.is_infinite());
        assert!(br.current_rating_ka > 0.0);
    }

    #[test]
    fn dc_branch_conductance_ok() {
        let br = DcBranch::new(2, 3, 2.0, 5.0);
        let g = br
            .conductance()
            .expect("conductance should be Ok for positive resistance");
        assert!((g - 0.5).abs() < 1e-9);
    }

    #[test]
    fn dc_branch_conductance_zero_err() {
        let br = DcBranch::new(0, 1, 0.0, 1.0);
        let result = br.conductance();
        assert!(result.is_err(), "zero resistance should return Err");
    }

    #[test]
    fn dc_bus_new_voltage_and_load() {
        let bus = DcBus::new(0, DcBusType::Slack, 320.0);
        assert!((bus.v_dc_kv - 320.0).abs() < 1e-9);
        assert!((bus.v_dc_nom_kv - 320.0).abs() < 1e-9);
        assert!((bus.p_load_mw - 0.0).abs() < 1e-9);
    }

    #[test]
    fn converter_new_is_rectifier() {
        let c_rect = AcDcConverter::new(0, 0, 0, ConverterType::PQ, 100.0, 0.0, 320.0);
        assert!(
            c_rect.is_rectifier,
            "positive p_ref_mw should set is_rectifier=true"
        );

        let c_inv = AcDcConverter::new(1, 0, 0, ConverterType::PQ, -100.0, 0.0, 320.0);
        assert!(
            !c_inv.is_rectifier,
            "negative p_ref_mw should set is_rectifier=false"
        );
    }

    #[test]
    fn converter_new_defaults() {
        let c = AcDcConverter::new(0, 0, 0, ConverterType::VdcQ, 50.0, 10.0, 320.0);
        assert!((c.losses_fraction - 0.02).abs() < 1e-9);
        assert!((c.q_min_mvar - (-200.0)).abs() < 1e-9);
        assert!((c.q_max_mvar - 200.0).abs() < 1e-9);
    }

    #[test]
    fn acdc_pf_new_init() {
        let n_ac = 1usize;
        let n_dc = 1usize;
        let ac_g = vec![vec![0.0f64; n_ac]; n_ac];
        let ac_b = vec![vec![0.0f64; n_ac]; n_ac];
        let dc_buses = vec![DcBus::new(0, DcBusType::Slack, 320.0)];
        let dc_branches: Vec<DcBranch> = vec![];
        let converters: Vec<AcDcConverter> = vec![];
        let net = AcDcNetwork::new(n_ac, n_dc, ac_g, ac_b, converters, dc_buses, dc_branches)
            .expect("should build AcDcNetwork");
        let config = AcDcPfConfig {
            max_iterations: 50,
            tolerance_pu: 1e-6,
            base_mva: 100.0,
            base_kv_ac: 110.0,
            base_kv_dc: 320.0,
        };
        let solver = AcDcPowerFlow::new(net, config);
        assert_eq!(solver.v_ac.len(), 1);
        assert!((solver.v_ac[0] - 1.0).abs() < 1e-12);
        assert_eq!(solver.theta_ac.len(), 1);
        assert!((solver.theta_ac[0] - 0.0).abs() < 1e-12);
    }
}
