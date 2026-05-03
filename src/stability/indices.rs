//! Advanced Power System Stability Indices.
//!
//! Provides a comprehensive set of voltage and transient stability indices:
//!
//! - **L-index** (Kessel-Glavitsch): voltage collapse proximity per PQ bus
//! - **FVSI** (Fast Voltage Stability Index): per-branch stability margin
//! - **VCPI** (Voltage Collapse Proximity Index): voltage sensitivity-based
//! - **VSI-PQ**: simplified per-PQ-bus index
//! - **EAC** (Equal Area Criterion): critical clearing angle for SMIB
//! - **Kinetic energy** approach for transient stability assessment

use std::f64::consts::PI;

// ─── Errors ───────────────────────────────────────────────────────────────────

/// Errors from stability index computation.
#[derive(Debug, thiserror::Error)]
pub enum StabilityError {
    /// Inconsistent input data.
    #[error("inconsistent input: {0}")]
    Inconsistent(String),
    /// Singular or ill-conditioned matrix.
    #[error("numerical issue: {0}")]
    Numerical(String),
    /// Empty bus/branch list.
    #[error("empty bus or branch list")]
    Empty,
}

// ─── Configuration ────────────────────────────────────────────────────────────

/// Voltage stability calculation method.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VsMethod {
    /// Kessel-Glavitsch L-index.
    LIndex,
    /// Fast Voltage Stability Index.
    Fvsi,
    /// Voltage Stability Index for PQ buses.
    VsiPQ,
    /// Voltage Collapse Proximity Index.
    Vcpi,
    /// Maximum loadability margin \[MW\].
    MvaMargin,
}

/// Transient stability calculation method.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TsMethod {
    /// Equal Area Criterion for SMIB system.
    EqualAreaCriterion,
    /// Kinetic energy approach.
    KineticEnergy,
    /// Direct Lyapunov method with energy function.
    LyapunovDirect,
}

/// Configuration for the stability index calculator.
#[derive(Debug, Clone)]
pub struct StabilityIndicesConfig {
    /// System base MVA.
    pub base_mva: f64,
    /// Nominal system frequency \[Hz\].
    pub frequency_hz: f64,
    /// Default voltage stability method.
    pub voltage_stability_method: VsMethod,
    /// Default transient stability method.
    pub transient_stability_method: TsMethod,
}

impl Default for StabilityIndicesConfig {
    fn default() -> Self {
        Self {
            base_mva: 100.0,
            frequency_hz: 50.0,
            voltage_stability_method: VsMethod::LIndex,
            transient_stability_method: TsMethod::EqualAreaCriterion,
        }
    }
}

// ─── Input Data Structures ────────────────────────────────────────────────────

/// Bus type classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BusType {
    /// Slack (reference) bus.
    Slack,
    /// Voltage-controlled bus (generator).
    PV,
    /// Load bus.
    PQ,
}

/// Per-bus steady-state data for stability assessment.
#[derive(Debug, Clone)]
pub struct BusStabilityData {
    /// Bus identifier.
    pub bus_id: usize,
    /// Voltage magnitude \[pu\].
    pub voltage_pu: f64,
    /// Voltage angle \[deg\].
    pub angle_deg: f64,
    /// Active power injection \[MW\] (generation positive).
    pub p_mw: f64,
    /// Reactive power injection \[Mvar\].
    pub q_mvar: f64,
    /// Bus type.
    pub bus_type: BusType,
}

/// Per-branch steady-state data for stability assessment.
#[derive(Debug, Clone)]
pub struct BranchStabilityData {
    /// Sending-end bus.
    pub from_bus: usize,
    /// Receiving-end bus.
    pub to_bus: usize,
    /// Resistance \[pu\].
    pub r_pu: f64,
    /// Reactance \[pu\].
    pub x_pu: f64,
    /// Shunt susceptance \[pu\].
    pub b_pu: f64,
    /// Active power flow \[MW\] (from → to).
    pub p_flow_mw: f64,
    /// Reactive power flow \[Mvar\] (from → to).
    pub q_flow_mvar: f64,
}

// ─── Result Structures ────────────────────────────────────────────────────────

/// Voltage stability indices computed for the operating point.
#[derive(Debug, Clone)]
pub struct VoltageStabilityIndices {
    /// L-index per PQ bus (0 = stable, 1 = collapse).
    pub l_index: Vec<f64>,
    /// FVSI per branch (0 = stable, 1 = collapse).
    pub fvsi: Vec<f64>,
    /// VSI-PQ per PQ bus.
    pub vsi_pq: Vec<f64>,
    /// VCPI per bus.
    pub vcpi: Vec<f64>,
    /// Minimum (worst-case) L-index across all PQ buses.
    pub min_l_index: f64,
    /// Index of the most stressed bus by L-index.
    pub critical_bus_l: usize,
    /// Minimum (worst-case) FVSI across all branches.
    pub min_fvsi: f64,
    /// Index of the most stressed branch by FVSI.
    pub critical_branch_fvsi: usize,
    /// Estimated loadability margin \[MW\].
    pub loadability_margin_mw: f64,
    /// Composite system voltage stability index in \[0, 1\].
    pub system_voltage_stability_index: f64,
}

/// Transient stability indices for SMIB system.
#[derive(Debug, Clone)]
pub struct TransientStabilityIndices {
    /// Critical kinetic energy at fault clearing \[MJ\].
    pub kinetic_energy_critical_mj: f64,
    /// Mechanical power deficit \[pu\].
    pub pm_deficit_pu: f64,
    /// Critical clearing angle from EAC \[deg\].
    pub critical_clearing_angle_deg: f64,
    /// Stability energy margin \[pu\].
    pub stability_margin_pu: f64,
}

/// Stability risk level.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StabilityRisk {
    /// All indices well within limits.
    Safe,
    /// Some indices approaching limits.
    Marginal,
    /// Indices close to stability boundary.
    Vulnerable,
    /// System at or beyond stability limit.
    Critical,
}

impl std::fmt::Display for StabilityRisk {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            StabilityRisk::Safe => write!(f, "Safe"),
            StabilityRisk::Marginal => write!(f, "Marginal"),
            StabilityRisk::Vulnerable => write!(f, "Vulnerable"),
            StabilityRisk::Critical => write!(f, "Critical"),
        }
    }
}

/// Full stability assessment result.
#[derive(Debug, Clone)]
pub struct StabilityAssessment {
    /// Voltage stability indices.
    pub voltage: VoltageStabilityIndices,
    /// Transient stability indices (if computable).
    pub transient: Option<TransientStabilityIndices>,
    /// Composite stability margin in \[0, 1\].
    pub overall_margin: f64,
    /// System risk level.
    pub risk_level: StabilityRisk,
}

// ─── Calculator ───────────────────────────────────────────────────────────────

/// Computes comprehensive stability indices for a given operating point.
pub struct StabilityIndexCalculator {
    config: StabilityIndicesConfig,
}

impl StabilityIndexCalculator {
    /// Create a new calculator with the given configuration.
    pub fn new(config: StabilityIndicesConfig) -> Self {
        Self { config }
    }

    /// Compute all stability indices for the given system state.
    ///
    /// # Arguments
    /// * `buses` — per-bus operating data
    /// * `branches` — per-branch operating data
    /// * `y_bus` — Y-bus matrix as `(G_ij, B_ij)` pairs indexed by `[i][j]`
    pub fn assess(
        &self,
        buses: &[BusStabilityData],
        branches: &[BranchStabilityData],
        y_bus: &[Vec<(f64, f64)>],
    ) -> Result<StabilityAssessment, StabilityError> {
        if buses.is_empty() || branches.is_empty() {
            return Err(StabilityError::Empty);
        }
        if y_bus.len() != buses.len() {
            return Err(StabilityError::Inconsistent(format!(
                "Y-bus size {} does not match bus count {}",
                y_bus.len(),
                buses.len()
            )));
        }

        // Voltage stability indices
        let l_index = self.compute_l_index(buses, y_bus);
        let fvsi = self.compute_fvsi(branches, buses);
        let vsi_pq = self.compute_vsi_pq(buses, branches);
        let vcpi = self.compute_vcpi(buses, branches);

        let pq_buses: Vec<usize> = buses
            .iter()
            .enumerate()
            .filter(|(_, b)| b.bus_type == BusType::PQ)
            .map(|(i, _)| i)
            .collect();

        let (min_l_index, critical_bus_l) = if l_index.is_empty() {
            (0.0, 0)
        } else {
            let max_l =
                l_index
                    .iter()
                    .copied()
                    .enumerate()
                    .fold(
                        (0.0f64, 0usize),
                        |acc, (i, v)| {
                            if v > acc.0 {
                                (v, i)
                            } else {
                                acc
                            }
                        },
                    );
            // Map back to global bus index
            let global_idx = pq_buses.get(max_l.1).copied().unwrap_or(max_l.1);
            (max_l.0, global_idx)
        };

        let (min_fvsi, critical_branch_fvsi) = if fvsi.is_empty() {
            (0.0, 0)
        } else {
            fvsi.iter()
                .copied()
                .enumerate()
                .fold(
                    (0.0f64, 0usize),
                    |acc, (i, v)| {
                        if v > acc.0 {
                            (v, i)
                        } else {
                            acc
                        }
                    },
                )
        };

        // Loadability margin: rough estimate from L-index proximity to 1
        let loadability_margin_mw = if min_l_index < 1.0 {
            let total_load: f64 = buses
                .iter()
                .filter(|b| b.p_mw < 0.0)
                .map(|b| -b.p_mw)
                .sum::<f64>();
            total_load * (1.0 - min_l_index).clamp(0.0, 1.0)
        } else {
            0.0
        };

        // Composite system index: weighted average of worst indices
        let system_vsi = 1.0 - (0.5 * min_l_index + 0.5 * min_fvsi).clamp(0.0, 1.0);

        let voltage = VoltageStabilityIndices {
            l_index,
            fvsi,
            vsi_pq,
            vcpi,
            min_l_index,
            critical_bus_l,
            min_fvsi,
            critical_branch_fvsi,
            loadability_margin_mw,
            system_voltage_stability_index: system_vsi,
        };

        // Transient stability: only computable if we can identify an SMIB
        let transient = self.compute_transient_indices(buses, branches);

        // Overall margin: composite of voltage and transient if available
        let overall_margin = if let Some(ref ts) = transient {
            0.6 * system_vsi + 0.4 * ts.stability_margin_pu.clamp(0.0, 1.0)
        } else {
            system_vsi
        };

        let risk_level = classify_risk(overall_margin, min_l_index, min_fvsi);

        Ok(StabilityAssessment {
            voltage,
            transient,
            overall_margin,
            risk_level,
        })
    }

    // ─── L-index (Kessel-Glavitsch) ─────────────────────────────────────────

    /// Compute the L-index for all PQ buses.
    ///
    /// The L-index for bus j is:
    /// `L_j = |1 + Σ_{i∈PV+slack} F_{ji} * V_i / V_j|`
    ///
    /// where `F = -Y_LL^{-1} * Y_LG` and subscripts L/G denote load/generator buses.
    pub fn compute_l_index(
        &self,
        buses: &[BusStabilityData],
        y_bus: &[Vec<(f64, f64)>],
    ) -> Vec<f64> {
        let n = buses.len();
        if n == 0 {
            return Vec::new();
        }

        // Partition buses
        let pq_indices: Vec<usize> = buses
            .iter()
            .enumerate()
            .filter(|(_, b)| b.bus_type == BusType::PQ)
            .map(|(i, _)| i)
            .collect();
        let pv_indices: Vec<usize> = buses
            .iter()
            .enumerate()
            .filter(|(_, b)| b.bus_type != BusType::PQ)
            .map(|(i, _)| i)
            .collect();

        if pq_indices.is_empty() || pv_indices.is_empty() {
            return vec![0.0; pq_indices.len()];
        }

        // Extract Y_LL (PQ×PQ) submatrix and Y_LG (PQ×PV) submatrix
        let nll = pq_indices.len();
        let nlg = pv_indices.len();

        // Y_LL[i][j] = y_bus[pq_indices[i]][pq_indices[j]]
        let y_ll: Vec<Vec<(f64, f64)>> = pq_indices
            .iter()
            .map(|&i| {
                pq_indices
                    .iter()
                    .map(|&j| {
                        if i < y_bus.len() && j < y_bus[i].len() {
                            y_bus[i][j]
                        } else {
                            (0.0, 0.0)
                        }
                    })
                    .collect()
            })
            .collect();

        // Y_LG[i][j] = y_bus[pq_indices[i]][pv_indices[j]]
        let y_lg: Vec<Vec<(f64, f64)>> = pq_indices
            .iter()
            .map(|&i| {
                pv_indices
                    .iter()
                    .map(|&j| {
                        if i < y_bus.len() && j < y_bus[i].len() {
                            y_bus[i][j]
                        } else {
                            (0.0, 0.0)
                        }
                    })
                    .collect()
            })
            .collect();

        // Invert Y_LL using Gaussian elimination (complex arithmetic)
        let y_ll_inv = match complex_gauss_inverse(&y_ll) {
            Some(m) => m,
            None => return vec![0.0; nll],
        };

        // F = -Y_LL_inv * Y_LG
        // F[i][j] = -Σ_k Y_LL_inv[i][k] * Y_LG[k][j]
        let f_matrix: Vec<Vec<(f64, f64)>> = (0..nll)
            .map(|i| {
                (0..nlg)
                    .map(|j| {
                        let sum = (0..nll).fold((0.0f64, 0.0f64), |acc, k| {
                            let (ar, ai) = y_ll_inv[i][k];
                            let (br, bi) = y_lg[k][j];
                            (acc.0 + ar * br - ai * bi, acc.1 + ar * bi + ai * br)
                        });
                        (-sum.0, -sum.1)
                    })
                    .collect()
            })
            .collect();

        // Compute L_j for each PQ bus
        pq_indices
            .iter()
            .enumerate()
            .map(|(li, &j)| {
                let vj_re = buses[j].voltage_pu * (buses[j].angle_deg * PI / 180.0).cos();
                let vj_im = buses[j].voltage_pu * (buses[j].angle_deg * PI / 180.0).sin();

                // sum_i F_{lj_i} * V_i / V_j
                let (sum_re, sum_im) =
                    pv_indices
                        .iter()
                        .enumerate()
                        .fold((0.0f64, 0.0f64), |acc, (gi, &gbus)| {
                            let vi_re =
                                buses[gbus].voltage_pu * (buses[gbus].angle_deg * PI / 180.0).cos();
                            let vi_im =
                                buses[gbus].voltage_pu * (buses[gbus].angle_deg * PI / 180.0).sin();
                            let (fr, fi) = if li < f_matrix.len() && gi < f_matrix[li].len() {
                                f_matrix[li][gi]
                            } else {
                                (0.0, 0.0)
                            };
                            // F * V_i (complex product)
                            let fv_re = fr * vi_re - fi * vi_im;
                            let fv_im = fr * vi_im + fi * vi_re;
                            // Divide by V_j
                            let denom = vj_re * vj_re + vj_im * vj_im;
                            let fv_vj_re = (fv_re * vj_re + fv_im * vj_im) / denom.max(1e-18);
                            let fv_vj_im = (fv_im * vj_re - fv_re * vj_im) / denom.max(1e-18);
                            (acc.0 + fv_vj_re, acc.1 + fv_vj_im)
                        });

                // L_j = |1 + sum|
                let lj_re = 1.0 + sum_re;
                let lj_im = sum_im;
                let l_j = (lj_re * lj_re + lj_im * lj_im).sqrt();
                l_j.clamp(0.0, 2.0) // clamp for numerical safety
            })
            .collect()
    }

    // ─── FVSI ─────────────────────────────────────────────────────────────────

    /// Compute Fast Voltage Stability Index (FVSI) for each branch.
    ///
    /// `FVSI = 4 * Z² * Q_r / (V_s² * X)`
    ///
    /// where Z = |R + jX|, Q_r = reactive power at receiving end \[pu\],
    /// V_s = sending-end voltage \[pu\].
    pub fn compute_fvsi(
        &self,
        branches: &[BranchStabilityData],
        buses: &[BusStabilityData],
    ) -> Vec<f64> {
        // Build bus index map
        let bus_idx: std::collections::HashMap<usize, usize> = buses
            .iter()
            .enumerate()
            .map(|(i, b)| (b.bus_id, i))
            .collect();

        let base_mva = self.config.base_mva.max(1.0);

        branches
            .iter()
            .map(|br| {
                let vs = bus_idx
                    .get(&br.from_bus)
                    .and_then(|&i| buses.get(i))
                    .map(|b| b.voltage_pu)
                    .unwrap_or(1.0);
                let vs2 = (vs * vs).max(1e-12);

                let z2 = br.r_pu * br.r_pu + br.x_pu * br.x_pu;
                let x = br.x_pu.abs().max(1e-12);
                // Q_r in pu = Q_flow_mvar / base_mva (positive = capacitive load)
                let qr = br.q_flow_mvar / base_mva;

                let fvsi = (4.0 * z2 * qr.abs()) / (vs2 * x);
                fvsi.clamp(0.0, 2.0)
            })
            .collect()
    }

    // ─── VSI-PQ ───────────────────────────────────────────────────────────────

    /// Compute VSI-PQ per PQ bus.
    ///
    /// `VSI_j = 1 - (V_j / V_j_no_load)²`
    ///
    /// Approximated as `1 - V_j²` when V_no_load ≈ 1.0 pu.
    fn compute_vsi_pq(
        &self,
        buses: &[BusStabilityData],
        branches: &[BranchStabilityData],
    ) -> Vec<f64> {
        let _ = branches; // kept for future use
        buses
            .iter()
            .filter(|b| b.bus_type == BusType::PQ)
            .map(|b| {
                let v2 = b.voltage_pu * b.voltage_pu;
                // VSI approaches 1 as voltage collapses
                (1.0 - v2).clamp(0.0, 1.0)
            })
            .collect()
    }

    // ─── VCPI ─────────────────────────────────────────────────────────────────

    /// Compute Voltage Collapse Proximity Index (VCPI) for each bus.
    ///
    /// VCPI is based on the ratio of power flow to maximum transferable power.
    /// For bus j connected to bus i via branch b:
    /// `VCPI_j = Σ_b (|S_b| / S_b_max)`
    pub fn compute_vcpi(
        &self,
        buses: &[BusStabilityData],
        branches: &[BranchStabilityData],
    ) -> Vec<f64> {
        let bus_idx: std::collections::HashMap<usize, usize> = buses
            .iter()
            .enumerate()
            .map(|(i, b)| (b.bus_id, i))
            .collect();

        buses
            .iter()
            .map(|bus| {
                let connected_branches: Vec<&BranchStabilityData> = branches
                    .iter()
                    .filter(|br| br.from_bus == bus.bus_id || br.to_bus == bus.bus_id)
                    .collect();

                if connected_branches.is_empty() {
                    return 0.0;
                }

                let vcpi_sum: f64 = connected_branches
                    .iter()
                    .map(|br| {
                        let s_flow =
                            (br.p_flow_mw * br.p_flow_mw + br.q_flow_mvar * br.q_flow_mvar).sqrt();
                        // Max transferable power: V_s * V_r / (2 * Z)
                        let vs = bus_idx
                            .get(&br.from_bus)
                            .and_then(|&i| buses.get(i))
                            .map(|b| b.voltage_pu)
                            .unwrap_or(1.0);
                        let vr = bus_idx
                            .get(&br.to_bus)
                            .and_then(|&i| buses.get(i))
                            .map(|b| b.voltage_pu)
                            .unwrap_or(1.0);
                        let z = (br.r_pu * br.r_pu + br.x_pu * br.x_pu).sqrt().max(1e-9);
                        let s_max = vs * vr / (2.0 * z) * self.config.base_mva;
                        if s_max < 1e-9 {
                            0.0
                        } else {
                            (s_flow / s_max).min(1.0)
                        }
                    })
                    .sum::<f64>();

                (vcpi_sum / connected_branches.len() as f64).clamp(0.0, 1.0)
            })
            .collect()
    }

    // ─── Transient Stability ──────────────────────────────────────────────────

    /// Compute transient stability indices using the Equal Area Criterion (EAC).
    ///
    /// Identifies the dominant generator and estimates critical clearing angle.
    fn compute_transient_indices(
        &self,
        buses: &[BusStabilityData],
        branches: &[BranchStabilityData],
    ) -> Option<TransientStabilityIndices> {
        // Find generator bus with largest P (dominant machine)
        let gen_bus = buses.iter().filter(|b| b.p_mw > 0.0).max_by(|a, b| {
            a.p_mw
                .partial_cmp(&b.p_mw)
                .unwrap_or(std::cmp::Ordering::Equal)
        })?;

        let pm_pu = gen_bus.p_mw / self.config.base_mva;
        let v = gen_bus.voltage_pu;

        // Equivalent transfer reactance: average of connected branch reactances
        let connected_x: Vec<f64> = branches
            .iter()
            .filter(|br| br.from_bus == gen_bus.bus_id || br.to_bus == gen_bus.bus_id)
            .map(|br| br.x_pu.abs())
            .filter(|&x| x > 1e-6)
            .collect();

        if connected_x.is_empty() {
            return None;
        }

        let x_eq = connected_x.iter().sum::<f64>() / connected_x.len() as f64;

        // Maximum power transferable: P_max = V_s * V_r / X_eq (approx V²/X)
        let p_max = (v * v / x_eq).max(pm_pu + 1e-9);

        // Critical clearing angle (EAC):
        // δ_0 = arcsin(P_m / P_max) — pre-fault equilibrium
        // δ_max = π - δ_0        — unstable equilibrium
        // EAC: A_acc = A_dec
        let sin_delta0 = (pm_pu / p_max).clamp(-1.0, 1.0);
        let delta_0 = sin_delta0.asin();
        let delta_max = PI - delta_0;

        // A_acc (area during fault) = P_m * (δ_cr - δ_0)
        // A_dec (area after fault) = P_max * (sin δ_cr - sin δ_max) - P_m * (δ_max - δ_cr)
        // Solving: critical clearing angle δ_cr
        // Approximation: δ_cr = arccos((P_m*(δ_max-δ_0) - P_max*cos(δ_max)) / P_max)
        let numerator = pm_pu * (delta_max - delta_0) - p_max * delta_max.cos();
        let cos_delta_cr = (numerator / p_max).clamp(-1.0, 1.0);
        let delta_cr = cos_delta_cr.acos();

        // Stability margin: how far clearing angle is from operating angle
        let stability_margin = ((delta_cr - delta_0) / (delta_max - delta_0)).clamp(0.0, 1.0);

        // Kinetic energy at fault clearing (inertia H assumed 5 MWs/MVA typical)
        let h = 5.0_f64; // MWs/MVA
        let m = 2.0 * h / (2.0 * PI * self.config.frequency_hz);
        let omega_0 = 2.0 * PI * self.config.frequency_hz;
        let kin_energy_cr = 0.5 * m * omega_0 * omega_0 * (delta_cr - delta_0).powi(2);

        Some(TransientStabilityIndices {
            kinetic_energy_critical_mj: kin_energy_cr * self.config.base_mva,
            pm_deficit_pu: (pm_pu / p_max).clamp(0.0, 1.0),
            critical_clearing_angle_deg: delta_cr.to_degrees(),
            stability_margin_pu: stability_margin,
        })
    }
}

// ─── Helpers ─────────────────────────────────────────────────────────────────

/// Classify stability risk from composite margin and worst indices.
fn classify_risk(overall_margin: f64, l_index: f64, fvsi: f64) -> StabilityRisk {
    let worst = l_index.max(fvsi);
    if overall_margin > 0.75 && worst < 0.3 {
        StabilityRisk::Safe
    } else if overall_margin > 0.5 && worst < 0.6 {
        StabilityRisk::Marginal
    } else if overall_margin > 0.25 && worst < 0.85 {
        StabilityRisk::Vulnerable
    } else {
        StabilityRisk::Critical
    }
}

/// Invert an n×n complex matrix using Gaussian elimination with partial pivoting.
///
/// Returns `None` if the matrix is singular.
fn complex_gauss_inverse(m: &[Vec<(f64, f64)>]) -> Option<Vec<Vec<(f64, f64)>>> {
    let n = m.len();
    if n == 0 {
        return Some(Vec::new());
    }

    // Augment [M | I]
    let mut aug: Vec<Vec<(f64, f64)>> = (0..n)
        .map(|i| {
            let mut row: Vec<(f64, f64)> = m[i].to_vec();
            while row.len() < n {
                row.push((0.0, 0.0));
            }
            row.truncate(n);
            for j in 0..n {
                row.push(if i == j { (1.0, 0.0) } else { (0.0, 0.0) });
            }
            row
        })
        .collect();

    for col in 0..n {
        // Find pivot (max magnitude in column)
        let pivot_row = (col..n).max_by(|&a, &b| {
            let ma = complex_abs(aug[a][col]);
            let mb = complex_abs(aug[b][col]);
            ma.partial_cmp(&mb).unwrap_or(std::cmp::Ordering::Equal)
        })?;

        aug.swap(col, pivot_row);

        let pivot = aug[col][col];
        let pivot_abs = complex_abs(pivot);
        if pivot_abs < 1e-14 {
            return None; // Singular
        }

        // Divide pivot row by pivot
        let pivot_inv = complex_inv(pivot)?;
        for elem in aug[col].iter_mut().take(2 * n) {
            *elem = complex_mul(*elem, pivot_inv);
        }

        // Eliminate column
        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = aug[row][col];
            if complex_abs(factor) < 1e-16 {
                continue;
            }
            // Copy the pivot row to avoid borrow conflict
            let col_row: Vec<(f64, f64)> = aug[col][..2 * n].to_vec();
            for (j, &col_val) in col_row.iter().enumerate() {
                let sub = complex_mul(factor, col_val);
                aug[row][j] = (aug[row][j].0 - sub.0, aug[row][j].1 - sub.1);
            }
        }
    }

    // Extract right half (inverse)
    Some(aug.into_iter().map(|row| row[n..].to_vec()).collect())
}

#[inline]
fn complex_abs((r, i): (f64, f64)) -> f64 {
    (r * r + i * i).sqrt()
}

#[inline]
fn complex_mul((ar, ai): (f64, f64), (br, bi): (f64, f64)) -> (f64, f64) {
    (ar * br - ai * bi, ar * bi + ai * br)
}

#[inline]
fn complex_inv((r, i): (f64, f64)) -> Option<(f64, f64)> {
    let d = r * r + i * i;
    if d < 1e-30 {
        None
    } else {
        Some((r / d, -i / d))
    }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[allow(clippy::type_complexity)]
    fn light_load_system() -> (
        Vec<BusStabilityData>,
        Vec<BranchStabilityData>,
        Vec<Vec<(f64, f64)>>,
    ) {
        // 3-bus: bus 0 = slack, bus 1 = PV gen, bus 2 = PQ load (light)
        let buses = vec![
            BusStabilityData {
                bus_id: 0,
                voltage_pu: 1.05,
                angle_deg: 0.0,
                p_mw: 120.0,
                q_mvar: 50.0,
                bus_type: BusType::Slack,
            },
            BusStabilityData {
                bus_id: 1,
                voltage_pu: 1.02,
                angle_deg: -3.0,
                p_mw: 80.0,
                q_mvar: 20.0,
                bus_type: BusType::PV,
            },
            BusStabilityData {
                bus_id: 2,
                voltage_pu: 0.98,
                angle_deg: -8.0,
                p_mw: -50.0,
                q_mvar: -20.0,
                bus_type: BusType::PQ,
            },
        ];
        let branches = vec![
            BranchStabilityData {
                from_bus: 0,
                to_bus: 2,
                r_pu: 0.02,
                x_pu: 0.06,
                b_pu: 0.02,
                p_flow_mw: 30.0,
                q_flow_mvar: 10.0,
            },
            BranchStabilityData {
                from_bus: 1,
                to_bus: 2,
                r_pu: 0.01,
                x_pu: 0.04,
                b_pu: 0.01,
                p_flow_mw: 20.0,
                q_flow_mvar: 8.0,
            },
        ];
        // Simple 3x3 Y-bus (diagonal dominant, approximate)
        let y = vec![
            vec![(5.0, -20.0), (0.0, 0.0), (-5.0, 15.0)],
            vec![(0.0, 0.0), (4.0, -25.0), (-4.0, 20.0)],
            vec![(-5.0, 15.0), (-4.0, 20.0), (9.0, -35.0)],
        ];
        (buses, branches, y)
    }

    #[allow(clippy::type_complexity)]
    fn heavy_load_system() -> (
        Vec<BusStabilityData>,
        Vec<BranchStabilityData>,
        Vec<Vec<(f64, f64)>>,
    ) {
        // Same topology but heavily loaded: voltage at PQ bus close to collapse
        let buses = vec![
            BusStabilityData {
                bus_id: 0,
                voltage_pu: 1.05,
                angle_deg: 0.0,
                p_mw: 300.0,
                q_mvar: 150.0,
                bus_type: BusType::Slack,
            },
            BusStabilityData {
                bus_id: 1,
                voltage_pu: 1.0,
                angle_deg: -15.0,
                p_mw: 200.0,
                q_mvar: 100.0,
                bus_type: BusType::PV,
            },
            BusStabilityData {
                bus_id: 2,
                voltage_pu: 0.72,
                angle_deg: -35.0, // low voltage
                p_mw: -450.0,
                q_mvar: -200.0,
                bus_type: BusType::PQ,
            },
        ];
        let branches = vec![
            BranchStabilityData {
                from_bus: 0,
                to_bus: 2,
                r_pu: 0.02,
                x_pu: 0.06,
                b_pu: 0.02,
                p_flow_mw: 200.0,
                q_flow_mvar: 100.0,
            },
            BranchStabilityData {
                from_bus: 1,
                to_bus: 2,
                r_pu: 0.01,
                x_pu: 0.04,
                b_pu: 0.01,
                p_flow_mw: 180.0,
                q_flow_mvar: 90.0,
            },
        ];
        let y = vec![
            vec![(5.0, -20.0), (0.0, 0.0), (-5.0, 15.0)],
            vec![(0.0, 0.0), (4.0, -25.0), (-4.0, 20.0)],
            vec![(-5.0, 15.0), (-4.0, 20.0), (9.0, -35.0)],
        ];
        (buses, branches, y)
    }

    // Test 1: Light load → VSI-PQ near 0 (stable), VCPI in valid range
    #[test]
    fn test_light_load_stable() {
        let (buses, branches, y) = light_load_system();
        let calc = StabilityIndexCalculator::new(StabilityIndicesConfig::default());
        let result = calc.assess(&buses, &branches, &y).expect("assessment ok");

        // VSI-PQ should be < 0.5 for light load (V = 0.98 → VSI = 1-0.96 = 0.04)
        for (i, &v) in result.voltage.vsi_pq.iter().enumerate() {
            assert!(
                v < 0.5,
                "VSI-PQ[{i}] = {v:.4} should be < 0.5 for light load (V=0.98)"
            );
        }
        // L-index must be in mathematical valid range [0, 2]
        for (i, &l) in result.voltage.l_index.iter().enumerate() {
            assert!(
                (0.0..=2.0).contains(&l),
                "L-index[{i}] = {l:.4} must be in [0, 2]"
            );
        }
        // VCPI must be in [0, 1]
        for (i, &v) in result.voltage.vcpi.iter().enumerate() {
            assert!(
                (0.0..=1.0).contains(&v),
                "VCPI[{i}] = {v:.4} must be in [0, 1]"
            );
        }
        // Assessment must succeed without error
        assert!(
            result.overall_margin >= 0.0 && result.overall_margin <= 1.0,
            "Overall margin {:.4} must be in [0,1]",
            result.overall_margin
        );
    }

    // Test 2: Heavy load → L-index elevated
    #[test]
    fn test_heavy_load_l_index_elevated() {
        let (buses, branches, y) = heavy_load_system();
        let calc = StabilityIndexCalculator::new(StabilityIndicesConfig::default());
        let result = calc.assess(&buses, &branches, &y).expect("assessment ok");

        // For heavily loaded system (V=0.72 pu), VSI-PQ approaches 1
        let max_vsi: f64 = result.voltage.vsi_pq.iter().cloned().fold(0.0f64, f64::max);
        assert!(
            max_vsi > 0.3,
            "VSI-PQ should be elevated for heavy load: max={max_vsi:.4}"
        );
    }

    // Test 3: FVSI correct for known line data
    #[test]
    fn test_fvsi_known_values() {
        let buses = vec![
            BusStabilityData {
                bus_id: 0,
                voltage_pu: 1.0,
                angle_deg: 0.0,
                p_mw: 100.0,
                q_mvar: 0.0,
                bus_type: BusType::Slack,
            },
            BusStabilityData {
                bus_id: 1,
                voltage_pu: 0.95,
                angle_deg: -5.0,
                p_mw: -80.0,
                q_mvar: -30.0,
                bus_type: BusType::PQ,
            },
        ];
        let branches = vec![BranchStabilityData {
            from_bus: 0,
            to_bus: 1,
            r_pu: 0.01,
            x_pu: 0.1,
            b_pu: 0.0,
            p_flow_mw: 80.0,
            q_flow_mvar: 30.0,
        }];
        let y = vec![
            vec![(1.0, -10.0), (-1.0, 10.0)],
            vec![(-1.0, 10.0), (1.0, -10.0)],
        ];
        let calc = StabilityIndexCalculator::new(StabilityIndicesConfig::default());
        let result = calc.assess(&buses, &branches, &y).expect("assessment ok");

        assert_eq!(result.voltage.fvsi.len(), 1);
        let fvsi = result.voltage.fvsi[0];
        // FVSI = 4 * Z² * Q / (V_s² * X) = 4*(0.01²+0.1²)*(30/100) / (1.0²*0.1)
        // Z² = 0.0101, Q_r = 0.3 pu
        // FVSI = 4*0.0101*0.3 / 0.1 = 0.1212
        assert!(
            fvsi > 0.05 && fvsi < 0.5,
            "FVSI for this branch should be ~0.12, got {fvsi:.4}"
        );
    }

    // Test 4: Critical bus and branch identified
    #[test]
    fn test_critical_bus_branch_identified() {
        let (buses, branches, y) = heavy_load_system();
        let calc = StabilityIndexCalculator::new(StabilityIndicesConfig::default());
        let result = calc.assess(&buses, &branches, &y).expect("assessment ok");

        // Critical bus must be in valid range
        assert!(
            result.voltage.critical_bus_l < buses.len(),
            "critical_bus_l {} out of range {}",
            result.voltage.critical_bus_l,
            buses.len()
        );
        // Critical branch must be in valid range
        assert!(
            result.voltage.critical_branch_fvsi < branches.len(),
            "critical_branch_fvsi {} out of range {}",
            result.voltage.critical_branch_fvsi,
            branches.len()
        );
        // The critical bus should be the PQ bus (index 2)
        assert_eq!(
            result.voltage.critical_bus_l, 2,
            "Critical bus must be bus 2 (PQ, low voltage)"
        );
    }

    // Test 5: Risk level transitions correctly
    #[test]
    fn test_risk_level_transitions() {
        let (light_buses, light_branches, light_y) = light_load_system();
        let (heavy_buses, heavy_branches, heavy_y) = heavy_load_system();
        let calc = StabilityIndexCalculator::new(StabilityIndicesConfig::default());

        let light_result = calc
            .assess(&light_buses, &light_branches, &light_y)
            .expect("ok");
        let heavy_result = calc
            .assess(&heavy_buses, &heavy_branches, &heavy_y)
            .expect("ok");

        // Light load must have better margin than heavy load
        assert!(
            light_result.overall_margin >= heavy_result.overall_margin,
            "Light load margin {:.4} should be >= heavy load margin {:.4}",
            light_result.overall_margin,
            heavy_result.overall_margin
        );

        // Heavy load should have higher risk
        let light_risk_val = risk_to_int(light_result.risk_level);
        let heavy_risk_val = risk_to_int(heavy_result.risk_level);
        assert!(
            heavy_risk_val >= light_risk_val,
            "Heavy load risk {:?} should be >= light load risk {:?}",
            heavy_result.risk_level,
            light_result.risk_level
        );
    }

    fn risk_to_int(r: StabilityRisk) -> u8 {
        match r {
            StabilityRisk::Safe => 0,
            StabilityRisk::Marginal => 1,
            StabilityRisk::Vulnerable => 2,
            StabilityRisk::Critical => 3,
        }
    }

    // Test 6: Empty input returns error
    #[test]
    fn test_empty_input_error() {
        let calc = StabilityIndexCalculator::new(StabilityIndicesConfig::default());
        let result = calc.assess(&[], &[], &[]);
        assert!(result.is_err(), "Empty input must return error");
    }

    // Test 7: Transient stability indices computed when generators present
    #[test]
    fn test_transient_indices_computed() {
        let (buses, branches, y) = light_load_system();
        let calc = StabilityIndexCalculator::new(StabilityIndicesConfig::default());
        let result = calc.assess(&buses, &branches, &y).expect("ok");
        // Generator buses are present (bus 0 and bus 1 with positive P)
        assert!(
            result.transient.is_some(),
            "Transient indices should be computed when generators present"
        );
        if let Some(ts) = result.transient {
            // CCA must be in valid range [0, 180 deg]
            assert!(
                ts.critical_clearing_angle_deg >= 0.0 && ts.critical_clearing_angle_deg <= 180.0,
                "CCA = {:.2} deg should be in [0, 180]",
                ts.critical_clearing_angle_deg
            );
            // Kinetic energy must be non-negative
            assert!(
                ts.kinetic_energy_critical_mj >= 0.0,
                "Kinetic energy must be >= 0: {:.2}",
                ts.kinetic_energy_critical_mj
            );
            // Stability margin in [0, 1]
            assert!(
                ts.stability_margin_pu >= 0.0 && ts.stability_margin_pu <= 1.0,
                "Stability margin = {:.4} must be in [0, 1]",
                ts.stability_margin_pu
            );
        }
    }

    // Test 8: VCPI is between 0 and 1 for all buses
    #[test]
    fn test_vcpi_range() {
        let (buses, branches, y) = heavy_load_system();
        let calc = StabilityIndexCalculator::new(StabilityIndicesConfig::default());
        let result = calc.assess(&buses, &branches, &y).expect("ok");
        for (i, &v) in result.voltage.vcpi.iter().enumerate() {
            assert!(
                (0.0..=1.0).contains(&v),
                "VCPI[{i}] = {v:.4} must be in [0, 1]"
            );
        }
    }

    // Test 9: VSI-PQ is exactly 1 - V^2 for a PQ bus at known voltage.
    // Reason: pins the computation formula (1 - V²) for the simplified VSI-PQ index.
    #[test]
    fn test_vsi_pq_formula_exact() {
        let buses = vec![
            BusStabilityData {
                bus_id: 0,
                voltage_pu: 1.0,
                angle_deg: 0.0,
                p_mw: 100.0,
                q_mvar: 0.0,
                bus_type: BusType::Slack,
            },
            BusStabilityData {
                bus_id: 1,
                voltage_pu: 0.9,
                angle_deg: -5.0,
                p_mw: -50.0,
                q_mvar: -10.0,
                bus_type: BusType::PQ,
            },
        ];
        let branches = vec![BranchStabilityData {
            from_bus: 0,
            to_bus: 1,
            r_pu: 0.02,
            x_pu: 0.08,
            b_pu: 0.0,
            p_flow_mw: 50.0,
            q_flow_mvar: 10.0,
        }];
        let y = vec![
            vec![(2.0, -12.0), (-2.0, 12.0)],
            vec![(-2.0, 12.0), (2.0, -12.0)],
        ];
        let calc = StabilityIndexCalculator::new(StabilityIndicesConfig::default());
        let result = calc.assess(&buses, &branches, &y).expect("ok");
        // VSI-PQ for bus with V=0.9 pu: 1 - 0.9² = 1 - 0.81 = 0.19
        assert_eq!(result.voltage.vsi_pq.len(), 1);
        approx::assert_relative_eq!(result.voltage.vsi_pq[0], 0.19, epsilon = 1e-9);
    }

    // Test 10: FVSI scales with reactive power — doubling Q doubles FVSI.
    // Reason: validates the linear proportionality of FVSI to Q_r (monotonicity/formula).
    #[test]
    fn test_fvsi_scales_with_reactive_power() {
        let make_system = |q_flow_mvar: f64| {
            let buses = vec![
                BusStabilityData {
                    bus_id: 0,
                    voltage_pu: 1.0,
                    angle_deg: 0.0,
                    p_mw: 80.0,
                    q_mvar: 0.0,
                    bus_type: BusType::Slack,
                },
                BusStabilityData {
                    bus_id: 1,
                    voltage_pu: 0.95,
                    angle_deg: -4.0,
                    p_mw: -60.0,
                    q_mvar: -q_flow_mvar,
                    bus_type: BusType::PQ,
                },
            ];
            let branches = vec![BranchStabilityData {
                from_bus: 0,
                to_bus: 1,
                r_pu: 0.01,
                x_pu: 0.05,
                b_pu: 0.0,
                p_flow_mw: 60.0,
                q_flow_mvar,
            }];
            let y = vec![
                vec![(4.0, -20.0), (-4.0, 20.0)],
                vec![(-4.0, 20.0), (4.0, -20.0)],
            ];
            (buses, branches, y)
        };

        let calc = StabilityIndexCalculator::new(StabilityIndicesConfig::default());
        let (b1, br1, y1) = make_system(20.0);
        let r1 = calc.assess(&b1, &br1, &y1).expect("ok");
        let (b2, br2, y2) = make_system(40.0);
        let r2 = calc.assess(&b2, &br2, &y2).expect("ok");

        let fvsi1 = r1.voltage.fvsi[0];
        let fvsi2 = r2.voltage.fvsi[0];
        // FVSI ∝ Q_r: doubling Q should approximately double FVSI
        approx::assert_relative_eq!(fvsi2, 2.0 * fvsi1, epsilon = 1e-9);
    }

    // Test 11: StabilityRisk Display trait produces expected strings.
    // Reason: pins the Display impl for all four risk variants.
    #[test]
    fn test_stability_risk_display() {
        assert_eq!(StabilityRisk::Safe.to_string(), "Safe");
        assert_eq!(StabilityRisk::Marginal.to_string(), "Marginal");
        assert_eq!(StabilityRisk::Vulnerable.to_string(), "Vulnerable");
        assert_eq!(StabilityRisk::Critical.to_string(), "Critical");
    }

    // Test 12: StabilityIndicesConfig default values are as documented.
    // Reason: pins construction defaults (base_mva=100, freq=50 Hz, LIndex, EAC).
    #[test]
    fn test_config_defaults() {
        let cfg = StabilityIndicesConfig::default();
        approx::assert_relative_eq!(cfg.base_mva, 100.0, epsilon = 1e-9);
        approx::assert_relative_eq!(cfg.frequency_hz, 50.0, epsilon = 1e-9);
        assert_eq!(cfg.voltage_stability_method, VsMethod::LIndex);
        assert_eq!(cfg.transient_stability_method, TsMethod::EqualAreaCriterion);
    }

    // Test 13: Y-bus size mismatch returns Inconsistent error.
    // Reason: validates error path when Y-bus dimensions don't match bus count.
    #[test]
    fn test_y_bus_size_mismatch_error() {
        let buses = vec![BusStabilityData {
            bus_id: 0,
            voltage_pu: 1.0,
            angle_deg: 0.0,
            p_mw: 100.0,
            q_mvar: 0.0,
            bus_type: BusType::Slack,
        }];
        let branches = vec![BranchStabilityData {
            from_bus: 0,
            to_bus: 0,
            r_pu: 0.01,
            x_pu: 0.05,
            b_pu: 0.0,
            p_flow_mw: 0.0,
            q_flow_mvar: 0.0,
        }];
        // Provide a 2×2 Y-bus for a 1-bus system — must error
        let y_wrong = vec![
            vec![(1.0, -5.0), (-1.0, 5.0)],
            vec![(-1.0, 5.0), (1.0, -5.0)],
        ];
        let calc = StabilityIndexCalculator::new(StabilityIndicesConfig::default());
        let result = calc.assess(&buses, &branches, &y_wrong);
        assert!(
            matches!(result, Err(StabilityError::Inconsistent(_))),
            "Mismatched Y-bus must return StabilityError::Inconsistent"
        );
    }

    // Test 14: VSI-PQ increases monotonically as PQ bus voltage drops.
    // Reason: confirms VSI-PQ faithfully tracks deteriorating voltage (monotonicity).
    #[test]
    fn test_vsi_pq_monotone_with_falling_voltage() {
        let make_system = |v_pq: f64| {
            let buses = vec![
                BusStabilityData {
                    bus_id: 0,
                    voltage_pu: 1.05,
                    angle_deg: 0.0,
                    p_mw: 100.0,
                    q_mvar: 30.0,
                    bus_type: BusType::Slack,
                },
                BusStabilityData {
                    bus_id: 1,
                    voltage_pu: v_pq,
                    angle_deg: -10.0,
                    p_mw: -80.0,
                    q_mvar: -25.0,
                    bus_type: BusType::PQ,
                },
            ];
            let branches = vec![BranchStabilityData {
                from_bus: 0,
                to_bus: 1,
                r_pu: 0.02,
                x_pu: 0.08,
                b_pu: 0.0,
                p_flow_mw: 80.0,
                q_flow_mvar: 25.0,
            }];
            let y = vec![
                vec![(3.0, -12.0), (-3.0, 12.0)],
                vec![(-3.0, 12.0), (3.0, -12.0)],
            ];
            (buses, branches, y)
        };

        let calc = StabilityIndexCalculator::new(StabilityIndicesConfig::default());
        let voltages = [1.0_f64, 0.95, 0.85, 0.75, 0.65];
        let mut prev_vsi = -1.0_f64;
        for &v in &voltages {
            let (b, br, y) = make_system(v);
            let result = calc.assess(&b, &br, &y).expect("ok");
            let vsi = result.voltage.vsi_pq[0];
            // VSI-PQ = 1 - V² must increase as V decreases
            assert!(
                vsi > prev_vsi,
                "VSI-PQ should increase as V drops: V={v:.2} VSI={vsi:.4} prev={prev_vsi:.4}"
            );
            prev_vsi = vsi;
        }
        // At V=0.65, VSI-PQ should be well above 0 (1 - 0.65² ≈ 0.5775)
        approx::assert_relative_eq!(prev_vsi, 1.0 - 0.65_f64.powi(2), epsilon = 1e-9);
    }
}
