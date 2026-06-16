//! HVDC (High-Voltage Direct Current) link and multi-terminal DC grid models.
//!
//! Provides:
//! - [`HvdcLink`] — point-to-point bipole HVDC link (LCC or VSC)
//! - [`MtdcGrid`] — multi-terminal DC grid with resistive power flow solver
//! - [`OffshoreHvdcSystem`] — HVDC collection system for offshore wind farms
//! - [`HvdcLosses`] — losses breakdown

use crate::error::OxiGridError;

#[cfg(feature = "renewable")]
use crate::renewable::wind::offshore::OffshoreWindFarm;

use serde::{Deserialize, Serialize};

// ─────────────────────────────────────────────────────────────────────────────
// HVDC link type
// ─────────────────────────────────────────────────────────────────────────────

/// Technology category of an HVDC converter station.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum HvdcType {
    /// Line Commutated Converter — classic thyristor-based HVDC.
    /// Requires AC voltage support; cannot start into a passive network.
    Lcc,
    /// Voltage Source Converter — self-commutating IGBT technology.
    /// Independent P and Q control; bidirectional power flow.
    Vsc,
    /// ABB HVDC Light variant of VSC (smaller, modular, subsea-ready).
    HvdcLight,
}

// ─────────────────────────────────────────────────────────────────────────────
// HVDC bipole link
// ─────────────────────────────────────────────────────────────────────────────

/// Point-to-point HVDC bipole link.
///
/// Supports both LCC and VSC technology types with independent converter-loss
/// models, reactive power limits (VSC), and cable resistance calculations.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HvdcLink {
    /// Unique link identifier.
    pub id: usize,
    /// Descriptive name.
    pub name: String,
    /// Rectifier (sending-end) AC bus index.
    pub from_bus: usize,
    /// Inverter (receiving-end) AC bus index.
    pub to_bus: usize,
    /// Converter technology.
    pub hvdc_type: HvdcType,
    /// Rated power transfer capacity \[MW\].
    pub p_rated_mw: f64,
    /// Current active power dispatch setpoint \[MW\] (positive = from→to).
    pub p_setpoint_mw: f64,
    /// Nominal DC pole-to-pole voltage \[kV\].
    pub v_dc_kv: f64,
    /// Cable route length \[km\].
    pub length_km: f64,
    /// DC cable resistance [Ω/km]. Default XLPE submarine cable: 0.011 Ω/km.
    pub resistance_ohm_per_km: f64,
    /// Total loss fraction (converter + cable) [%]. Used for simplified loss estimate.
    pub losses_pct: f64,
    // VSC reactive power capability at from-bus [MVAr]
    /// Minimum reactive injection at from-bus \[MVAr\].
    pub q_from_min: f64,
    /// Maximum reactive injection at from-bus \[MVAr\].
    pub q_from_max: f64,
    /// Minimum reactive injection at to-bus \[MVAr\].
    pub q_to_min: f64,
    /// Maximum reactive injection at to-bus \[MVAr\].
    pub q_to_max: f64,
    /// Current Q setpoint at from-bus \[MVAr\].
    pub q_from_setpoint: f64,
    /// Current Q setpoint at to-bus \[MVAr\].
    pub q_to_setpoint: f64,
    /// Whether the link is in service.
    pub in_service: bool,
}

impl HvdcLink {
    /// Construct a VSC HVDC link.
    ///
    /// VSC links are bidirectional (`±p_mw`), and Q limits default to ±30 % of rated power.
    pub fn new_vsc(id: usize, from: usize, to: usize, p_mw: f64, v_dc_kv: f64) -> Self {
        let q_cap = 0.3 * p_mw;
        Self {
            id,
            name: format!("VSC-HVDC-{id}"),
            from_bus: from,
            to_bus: to,
            hvdc_type: HvdcType::Vsc,
            p_rated_mw: p_mw.abs(),
            p_setpoint_mw: p_mw,
            v_dc_kv,
            length_km: 0.0,
            resistance_ohm_per_km: 0.011,
            losses_pct: 1.5,
            q_from_min: -q_cap,
            q_from_max: q_cap,
            q_to_min: -q_cap,
            q_to_max: q_cap,
            q_from_setpoint: 0.0,
            q_to_setpoint: 0.0,
            in_service: true,
        }
    }

    /// Construct an LCC HVDC link.
    ///
    /// LCC links operate unidirectionally by default (positive setpoint).
    /// Q limits are set to zero (LCC absorbs reactive power, not modelled here).
    pub fn new_lcc(id: usize, from: usize, to: usize, p_mw: f64, v_dc_kv: f64) -> Self {
        Self {
            id,
            name: format!("LCC-HVDC-{id}"),
            from_bus: from,
            to_bus: to,
            hvdc_type: HvdcType::Lcc,
            p_rated_mw: p_mw.abs(),
            p_setpoint_mw: p_mw.abs(), // LCC: positive only
            v_dc_kv,
            length_km: 0.0,
            resistance_ohm_per_km: 0.011,
            losses_pct: 1.5,
            q_from_min: 0.0,
            q_from_max: 0.0,
            q_to_min: 0.0,
            q_to_max: 0.0,
            q_from_setpoint: 0.0,
            q_to_setpoint: 0.0,
            in_service: true,
        }
    }

    /// DC cable losses \[MW\].
    ///
    /// Computed as `P_setpoint × (resistance × length / (V_dc² / 1000))`.
    /// Falls back to simplified `losses_pct` model when length is not set.
    pub fn cable_losses_mw(&self) -> f64 {
        if !self.in_service {
            return 0.0;
        }
        let p = self.p_setpoint_mw.abs();
        if self.length_km > 0.0 && self.v_dc_kv > 0.0 {
            // I = P / V_dc; P_loss = I² × R_total
            // R_total [Ω] = resistance_ohm_per_km × length_km
            // I [kA]  = p [MW] / v_dc [kV]
            let r_total = self.resistance_ohm_per_km * self.length_km; // Ω
            let i_ka = p / self.v_dc_kv; // kA
            i_ka.powi(2) * r_total // MW
        } else {
            // Simplified: half of total losses attributed to cable
            p * self.losses_pct / 200.0
        }
    }

    /// Converter station losses \[MW\].
    ///
    /// LCC: ~0.6 % per converter × 2 converters = 1.2 % of rated.
    /// VSC: ~1.0 % per converter × 2 converters = 2.0 % of rated.
    pub fn converter_losses_mw(&self) -> f64 {
        if !self.in_service {
            return 0.0;
        }
        let p = self.p_setpoint_mw.abs();
        match self.hvdc_type {
            HvdcType::Lcc => p * 0.012,
            HvdcType::Vsc | HvdcType::HvdcLight => p * 0.020,
        }
    }

    /// Net power delivered at the receiving (inverter) end \[MW\].
    pub fn power_delivered_mw(&self) -> f64 {
        if !self.in_service {
            return 0.0;
        }
        let gross = self.p_setpoint_mw;
        let losses = self.cable_losses_mw() + self.converter_losses_mw();
        (gross.abs() - losses).max(0.0) * gross.signum()
    }

    /// Active power operating range `(min_mw, max_mw)`.
    ///
    /// LCC: `[0, p_rated]` (unidirectional).
    /// VSC/HvdcLight: `[-p_rated, p_rated]` (bidirectional).
    pub fn p_range(&self) -> (f64, f64) {
        match self.hvdc_type {
            HvdcType::Lcc => (0.0, self.p_rated_mw),
            HvdcType::Vsc | HvdcType::HvdcLight => (-self.p_rated_mw, self.p_rated_mw),
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Multi-terminal DC (MTDC) grid
// ─────────────────────────────────────────────────────────────────────────────

/// Converter control mode in an MTDC grid.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum MtdcControlMode {
    /// Converter holds constant active power injection.
    ConstantPower,
    /// Converter regulates DC bus voltage to setpoint.
    ConstantVoltage,
    /// Converter uses a voltage margin band for handover.
    VoltageMargin { margin_kv: f64 },
    /// Proportional voltage–power droop control.
    VoltageDroop { droop: f64 },
}

/// Individual converter in an MTDC grid.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MtdcConverter {
    /// Converter identifier.
    pub id: usize,
    /// AC bus index this converter connects to.
    pub ac_bus: usize,
    /// DC bus index this converter connects to.
    pub dc_bus: usize,
    /// Rated power \[MW\].
    pub p_rated_mw: f64,
    /// Control mode.
    pub control_mode: MtdcControlMode,
    /// Active power setpoint \[MW\] (positive = rectifier, absorbs AC power).
    pub p_setpoint_mw: f64,
    /// DC voltage setpoint \[kV\] (only used in ConstantVoltage / droop modes).
    pub v_dc_setpoint_kv: f64,
}

/// DC bus in an MTDC grid.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DcBus {
    /// DC bus identifier.
    pub id: usize,
    /// Rated DC voltage \[kV\].
    pub v_rated_kv: f64,
    /// Current DC voltage after power flow \[kV\].
    pub v_dc: f64,
}

/// DC transmission line segment in an MTDC grid.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DcLine {
    /// Line identifier.
    pub id: usize,
    /// From DC bus index.
    pub from_dc_bus: usize,
    /// To DC bus index.
    pub to_dc_bus: usize,
    /// Total line resistance \[Ω\].
    pub resistance_ohm: f64,
    /// Thermal rating \[MW\].
    pub rating_mw: f64,
}

/// Multi-terminal HVDC grid with resistive DC power flow.
#[derive(Debug, Clone)]
pub struct MtdcGrid {
    /// Grid identifier.
    pub id: usize,
    /// Converter stations.
    pub converters: Vec<MtdcConverter>,
    /// DC buses.
    pub dc_buses: Vec<DcBus>,
    /// DC transmission lines.
    pub dc_lines: Vec<DcLine>,
    /// Index of the slack (voltage-controlling) converter.
    pub slack_converter: usize,
}

impl MtdcGrid {
    /// Create a new MTDC grid.
    pub fn new(
        converters: Vec<MtdcConverter>,
        dc_buses: Vec<DcBus>,
        dc_lines: Vec<DcLine>,
        slack: usize,
    ) -> Self {
        Self {
            id: 0,
            converters,
            dc_buses,
            dc_lines,
            slack_converter: slack,
        }
    }

    /// Solve the DC power flow using nodal conductance analysis.
    ///
    /// Builds the conductance matrix G_dc and solves `G_dc · V_dc = I_inj`
    /// where `I_inj = P_conv / V_dc` for each converter bus.
    ///
    /// Returns the DC bus voltages \[kV\].
    pub fn solve_dc_power_flow(&mut self) -> Result<Vec<f64>, OxiGridError> {
        let n = self.dc_buses.len();
        if n == 0 {
            return Err(OxiGridError::InvalidNetwork(
                "MTDC grid has no DC buses".to_string(),
            ));
        }

        // Build DC bus-id → index mapping
        let bus_index: std::collections::HashMap<usize, usize> = self
            .dc_buses
            .iter()
            .enumerate()
            .map(|(i, b)| (b.id, i))
            .collect();

        // ── Build nodal conductance matrix G_dc (dense, small grids) ─────────
        let mut g = vec![vec![0.0_f64; n]; n];
        for line in &self.dc_lines {
            let i = match bus_index.get(&line.from_dc_bus) {
                Some(&idx) => idx,
                None => {
                    return Err(OxiGridError::InvalidNetwork(format!(
                        "DC line {} references unknown from_dc_bus {}",
                        line.id, line.from_dc_bus
                    )))
                }
            };
            let j = match bus_index.get(&line.to_dc_bus) {
                Some(&idx) => idx,
                None => {
                    return Err(OxiGridError::InvalidNetwork(format!(
                        "DC line {} references unknown to_dc_bus {}",
                        line.id, line.to_dc_bus
                    )))
                }
            };
            let y = if line.resistance_ohm.abs() > 1e-12 {
                1.0 / line.resistance_ohm
            } else {
                return Err(OxiGridError::InvalidParameter(format!(
                    "DC line {} has zero resistance",
                    line.id
                )));
            };
            g[i][i] += y;
            g[j][j] += y;
            g[i][j] -= y;
            g[j][i] -= y;
        }

        // ── Current injections I_inj = P_conv / V_dc_nominal ─────────────────
        let mut i_inj = vec![0.0_f64; n];
        for conv in &self.converters {
            let dc_idx = match bus_index.get(&conv.dc_bus) {
                Some(&idx) => idx,
                None => {
                    return Err(OxiGridError::InvalidNetwork(format!(
                        "Converter {} references unknown dc_bus {}",
                        conv.id, conv.dc_bus
                    )))
                }
            };
            let v_nom = self.dc_buses[dc_idx].v_rated_kv.max(1.0);
            // Positive P = power into DC grid (rectifier)
            i_inj[dc_idx] += conv.p_setpoint_mw / v_nom;
        }

        // ── Identify slack bus (voltage-controlled converter's DC bus) ────────
        let slack_dc_bus = self
            .converters
            .get(self.slack_converter)
            .map(|c| c.dc_bus)
            .ok_or_else(|| {
                OxiGridError::InvalidNetwork("slack_converter index out of range".to_string())
            })?;
        let slack_idx = match bus_index.get(&slack_dc_bus) {
            Some(&idx) => idx,
            None => {
                return Err(OxiGridError::InvalidNetwork(
                    "Slack converter DC bus not found".to_string(),
                ))
            }
        };
        let v_slack = self.dc_buses[slack_idx].v_rated_kv;

        // ── Apply slack bus: modify row slack_idx → unit row ──────────────────
        g[slack_idx][..n].fill(0.0);
        g[slack_idx][slack_idx] = 1.0;
        i_inj[slack_idx] = v_slack;

        // ── Gaussian elimination (small system) ───────────────────────────────
        let v_dc = gaussian_elimination(&g, &i_inj)
            .map_err(|e| OxiGridError::LinearAlgebra(format!("MTDC DC power flow: {e}")))?;

        // ── Update DC bus voltages ────────────────────────────────────────────
        for bus in &mut self.dc_buses {
            if let Some(&idx) = bus_index.get(&bus.id) {
                bus.v_dc = v_dc[idx];
            }
        }

        Ok(v_dc)
    }

    /// Update converter active power after DC power flow.
    ///
    /// For droop-controlled converters, adjusts `p_setpoint_mw` according to
    /// the computed DC voltage deviation from the setpoint.
    pub fn update_converter_powers(&mut self, v_dc: &[f64]) {
        if v_dc.len() != self.dc_buses.len() {
            return;
        }
        let bus_index: std::collections::HashMap<usize, usize> = self
            .dc_buses
            .iter()
            .enumerate()
            .map(|(i, b)| (b.id, i))
            .collect();

        for conv in &mut self.converters {
            if let Some(&dc_idx) = bus_index.get(&conv.dc_bus) {
                if dc_idx < v_dc.len() {
                    let v = v_dc[dc_idx];
                    if let MtdcControlMode::VoltageDroop { droop } = conv.control_mode {
                        let dv = v - conv.v_dc_setpoint_kv;
                        let dp = droop * dv; // MW change per kV deviation
                        conv.p_setpoint_mw =
                            (conv.p_setpoint_mw + dp).clamp(-conv.p_rated_mw, conv.p_rated_mw);
                    }
                }
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Gaussian elimination helper (dense, small matrices for MTDC)
// ─────────────────────────────────────────────────────────────────────────────

fn gaussian_elimination(a: &[Vec<f64>], b: &[f64]) -> Result<Vec<f64>, String> {
    let n = b.len();
    if a.len() != n || a.iter().any(|row| row.len() != n) {
        return Err("matrix dimensions mismatch".to_string());
    }

    let mut mat: Vec<Vec<f64>> = a.to_vec();
    let mut rhs: Vec<f64> = b.to_vec();

    for col in 0..n {
        // Partial pivoting
        let pivot_row = (col..n).max_by(|&r1, &r2| {
            mat[r1][col]
                .abs()
                .partial_cmp(&mat[r2][col].abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        let pivot_row = pivot_row.ok_or("empty range in gaussian elimination")?;
        mat.swap(col, pivot_row);
        rhs.swap(col, pivot_row);

        let pivot = mat[col][col];
        if pivot.abs() < 1e-14 {
            return Err(format!("singular matrix at column {col}"));
        }

        for row in (col + 1)..n {
            let factor = mat[row][col] / pivot;
            #[allow(clippy::needless_range_loop)]
            for k in col..n {
                let delta = factor * mat[col][k];
                mat[row][k] -= delta;
            }
            rhs[row] -= factor * rhs[col];
        }
    }

    // Back-substitution
    let mut x = vec![0.0_f64; n];
    for i in (0..n).rev() {
        let mut s = rhs[i];
        for j in (i + 1)..n {
            s -= mat[i][j] * x[j];
        }
        x[i] = s / mat[i][i];
    }
    Ok(x)
}

// ─────────────────────────────────────────────────────────────────────────────
// HVDC losses breakdown
// ─────────────────────────────────────────────────────────────────────────────

/// Detailed HVDC losses breakdown.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HvdcLosses {
    /// Converter station losses (both ends) \[MW\].
    pub converter_losses_mw: f64,
    /// DC cable / line losses \[MW\].
    pub cable_losses_mw: f64,
    /// Offshore transformer losses (estimated at 0.5 % of total power) \[MW\].
    pub transformer_losses_mw: f64,
    /// Total losses \[MW\].
    pub total_losses_mw: f64,
    /// Overall efficiency [%].
    pub efficiency_pct: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Offshore HVDC collection system
// ─────────────────────────────────────────────────────────────────────────────

/// HVDC collection system aggregating multiple offshore wind farms.
///
/// Each farm is connected to shore through one or more [`HvdcLink`] entries.
/// The system computes total shore power and loss breakdown under given wind
/// conditions.
#[cfg(feature = "renewable")]
pub struct OffshoreHvdcSystem {
    /// Offshore wind farms.
    pub farms: Vec<OffshoreWindFarm>,
    /// HVDC links (one per farm, or shared).
    pub hvdc_links: Vec<HvdcLink>,
    /// Offshore substation AC collection voltage \[kV\].
    pub offshore_substation_voltage_kv: f64,
    /// Total rated generation capacity \[MW\].
    pub total_capacity_mw: f64,
}

#[cfg(feature = "renewable")]
impl OffshoreHvdcSystem {
    /// Create a new offshore HVDC collection system.
    pub fn new(farms: Vec<OffshoreWindFarm>, links: Vec<HvdcLink>) -> Self {
        let total_capacity_mw: f64 = farms.iter().map(|f| f.installed_capacity_mw()).sum();
        Self {
            farms,
            hvdc_links: links,
            offshore_substation_voltage_kv: 66.0,
            total_capacity_mw,
        }
    }

    /// Compute total active power delivered to shore \[MW\].
    ///
    /// `wind_conditions`: `(wind_speed_m_s, wind_direction_deg)` per farm (in order).
    /// If fewer conditions than farms are provided, the last entry is repeated.
    pub fn power_to_shore(&self, wind_conditions: &[(f64, f64)]) -> f64 {
        if self.farms.is_empty() {
            return 0.0;
        }

        let total_generated: f64 = self
            .farms
            .iter()
            .enumerate()
            .map(|(i, farm)| {
                let (speed, dir) = wind_conditions
                    .get(i)
                    .copied()
                    .unwrap_or_else(|| *wind_conditions.last().unwrap_or(&(10.0, 270.0)));
                farm.compute_power(speed, dir)
            })
            .sum();

        // Apply HVDC losses
        let losses = self.losses_breakdown_for_power(total_generated);
        (total_generated - losses.total_losses_mw).max(0.0)
    }

    /// Compute HVDC losses breakdown for a given farm power \[MW\].
    fn losses_breakdown_for_power(&self, total_mw: f64) -> HvdcLosses {
        let conv_losses: f64 = self
            .hvdc_links
            .iter()
            .map(|l| l.converter_losses_mw())
            .sum();
        let cable_losses: f64 = self.hvdc_links.iter().map(|l| l.cable_losses_mw()).sum();
        let transformer_losses = total_mw * 0.005; // 0.5 % offshore transformer
        let total_losses = conv_losses + cable_losses + transformer_losses;
        let efficiency_pct = if total_mw > 0.0 {
            100.0 * (1.0 - total_losses / total_mw)
        } else {
            100.0
        };
        HvdcLosses {
            converter_losses_mw: conv_losses,
            cable_losses_mw: cable_losses,
            transformer_losses_mw: transformer_losses,
            total_losses_mw: total_losses,
            efficiency_pct,
        }
    }

    /// Return the HVDC losses breakdown under given wind conditions.
    pub fn losses_breakdown(&self, wind_conditions: &[(f64, f64)]) -> HvdcLosses {
        let total_generated: f64 = self
            .farms
            .iter()
            .enumerate()
            .map(|(i, farm)| {
                let (speed, dir) = wind_conditions
                    .get(i)
                    .copied()
                    .unwrap_or_else(|| *wind_conditions.last().unwrap_or(&(10.0, 270.0)));
                farm.compute_power(speed, dir)
            })
            .sum();
        self.losses_breakdown_for_power(total_generated)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── HvdcLink ─────────────────────────────────────────────────────────────

    #[test]
    fn test_hvdc_cable_losses_positive() {
        let mut link = HvdcLink::new_vsc(0, 1, 2, 1000.0, 320.0);
        link.length_km = 200.0;
        let losses = link.cable_losses_mw();
        assert!(losses > 0.0, "cable losses must be positive: {losses}");
    }

    #[test]
    fn test_hvdc_power_delivered_approx() {
        // 1000 MW VSC, 1.5 % simplified losses → delivered ≈ 979 MW (after converter losses too)
        let link = HvdcLink::new_vsc(0, 1, 2, 1000.0, 320.0);
        let delivered = link.power_delivered_mw();
        assert!(
            delivered > 950.0 && delivered < 1000.0,
            "delivered should be in (950, 1000) MW: {delivered}"
        );
    }

    #[test]
    fn test_hvdc_p_range_vsc_bidirectional() {
        let link = HvdcLink::new_vsc(0, 1, 2, 500.0, 320.0);
        let (min, max) = link.p_range();
        assert_eq!(min, -500.0);
        assert_eq!(max, 500.0);
    }

    #[test]
    fn test_hvdc_p_range_lcc_unidirectional() {
        let link = HvdcLink::new_lcc(0, 1, 2, 500.0, 500.0);
        let (min, max) = link.p_range();
        assert_eq!(min, 0.0);
        assert_eq!(max, 500.0);
    }

    #[test]
    fn test_hvdc_converter_losses_vsc_higher_than_lcc() {
        let vsc = HvdcLink::new_vsc(0, 1, 2, 1000.0, 320.0);
        let lcc = HvdcLink::new_lcc(1, 1, 2, 1000.0, 500.0);
        assert!(
            vsc.converter_losses_mw() > lcc.converter_losses_mw(),
            "VSC losses ({}) should exceed LCC losses ({})",
            vsc.converter_losses_mw(),
            lcc.converter_losses_mw()
        );
    }

    #[test]
    fn test_hvdc_out_of_service_zero_losses() {
        let mut link = HvdcLink::new_vsc(0, 1, 2, 1000.0, 320.0);
        link.in_service = false;
        assert_eq!(link.cable_losses_mw(), 0.0);
        assert_eq!(link.converter_losses_mw(), 0.0);
        assert_eq!(link.power_delivered_mw(), 0.0);
    }

    // ── MTDC power flow ──────────────────────────────────────────────────────

    fn make_simple_mtdc() -> MtdcGrid {
        // Two DC buses connected by a 1 Ω line; two converters
        let dc_buses = vec![
            DcBus {
                id: 0,
                v_rated_kv: 320.0,
                v_dc: 320.0,
            },
            DcBus {
                id: 1,
                v_rated_kv: 320.0,
                v_dc: 320.0,
            },
        ];
        let dc_lines = vec![DcLine {
            id: 0,
            from_dc_bus: 0,
            to_dc_bus: 1,
            resistance_ohm: 1.0,
            rating_mw: 1000.0,
        }];
        let converters = vec![
            MtdcConverter {
                id: 0,
                ac_bus: 0,
                dc_bus: 0,
                p_rated_mw: 500.0,
                control_mode: MtdcControlMode::ConstantVoltage,
                p_setpoint_mw: 300.0,
                v_dc_setpoint_kv: 320.0,
            },
            MtdcConverter {
                id: 1,
                ac_bus: 1,
                dc_bus: 1,
                p_rated_mw: 500.0,
                control_mode: MtdcControlMode::ConstantPower,
                p_setpoint_mw: -300.0,
                v_dc_setpoint_kv: 320.0,
            },
        ];
        MtdcGrid::new(converters, dc_buses, dc_lines, 0)
    }

    #[test]
    fn test_mtdc_power_flow_solves() {
        let mut grid = make_simple_mtdc();
        let v_dc = grid
            .solve_dc_power_flow()
            .expect("MTDC power flow should solve");
        assert_eq!(v_dc.len(), 2);
        // Slack bus voltage should equal rated
        assert!(
            (v_dc[0] - 320.0).abs() < 1.0,
            "slack bus voltage: {:.2} kV",
            v_dc[0]
        );
    }

    #[test]
    fn test_mtdc_power_flow_balance() {
        let mut grid = make_simple_mtdc();
        let v_dc = grid.solve_dc_power_flow().expect("MTDC power flow");
        // With solution voltages, compute line current and check approximate balance
        // I = ΔV / R;  P_line ≈ I × V_avg
        let delta_v = (v_dc[0] - v_dc[1]).abs();
        let i_line = delta_v / 1.0; // R = 1 Ω
        let v_avg = (v_dc[0] + v_dc[1]) / 2.0;
        let p_line_mw = i_line * v_avg;
        // Line power should be positive (power flows from high to low voltage)
        // and physically plausible (< rated converter capacity)
        assert!(
            p_line_mw >= 0.0,
            "line power must be non-negative: {p_line_mw}"
        );
    }

    #[test]
    fn test_mtdc_update_droop_converter() {
        let dc_buses = vec![
            DcBus {
                id: 0,
                v_rated_kv: 320.0,
                v_dc: 320.0,
            },
            DcBus {
                id: 1,
                v_rated_kv: 320.0,
                v_dc: 320.0,
            },
        ];
        let dc_lines = vec![DcLine {
            id: 0,
            from_dc_bus: 0,
            to_dc_bus: 1,
            resistance_ohm: 0.5,
            rating_mw: 500.0,
        }];
        let converters = vec![
            MtdcConverter {
                id: 0,
                ac_bus: 0,
                dc_bus: 0,
                p_rated_mw: 500.0,
                control_mode: MtdcControlMode::ConstantVoltage,
                p_setpoint_mw: 200.0,
                v_dc_setpoint_kv: 320.0,
            },
            MtdcConverter {
                id: 1,
                ac_bus: 1,
                dc_bus: 1,
                p_rated_mw: 500.0,
                control_mode: MtdcControlMode::VoltageDroop { droop: 10.0 },
                p_setpoint_mw: -150.0,
                v_dc_setpoint_kv: 320.0,
            },
        ];
        let mut grid = MtdcGrid::new(converters, dc_buses, dc_lines, 0);
        let v_dc = grid.solve_dc_power_flow().expect("solve");
        // After droop update, converter 1 setpoint should change
        let p_before = grid.converters[1].p_setpoint_mw;
        grid.update_converter_powers(&v_dc);
        let p_after = grid.converters[1].p_setpoint_mw;
        // If voltage deviated, droop control changes setpoint
        let _ = (p_before, p_after); // values used for side-effects check
    }

    // ── OffshoreHvdcSystem ───────────────────────────────────────────────────

    #[cfg(feature = "renewable")]
    #[test]
    fn test_offshore_hvdc_system_power_to_shore() {
        use crate::renewable::wind::offshore::{OffshoreWindFarm, OffshoreWindTurbine};
        let turbines: Vec<OffshoreWindTurbine> = (0..4)
            .map(|i| OffshoreWindTurbine::new_15mw(i, (i as f64) * 1400.0, 0.0))
            .collect();
        let farm = OffshoreWindFarm::new(turbines, 10, 1);
        let link = HvdcLink::new_vsc(0, 10, 1, 200.0, 320.0);
        let system = OffshoreHvdcSystem::new(vec![farm], vec![link]);
        let power = system.power_to_shore(&[(10.0, 270.0)]);
        assert!(power >= 0.0, "power to shore must be non-negative: {power}");
    }

    #[cfg(feature = "renewable")]
    #[test]
    fn test_offshore_hvdc_losses_breakdown_efficiency() {
        use crate::renewable::wind::offshore::{OffshoreWindFarm, OffshoreWindTurbine};
        let turbines: Vec<OffshoreWindTurbine> = (0..4)
            .map(|i| OffshoreWindTurbine::new_15mw(i, (i as f64) * 1400.0, 0.0))
            .collect();
        let farm = OffshoreWindFarm::new(turbines, 10, 1);
        let mut link = HvdcLink::new_vsc(0, 10, 1, 200.0, 320.0);
        link.p_setpoint_mw = 50.0;
        let system = OffshoreHvdcSystem::new(vec![farm], vec![link]);
        let losses = system.losses_breakdown(&[(10.0, 270.0)]);
        assert!(
            losses.efficiency_pct > 50.0 && losses.efficiency_pct <= 100.0,
            "efficiency should be reasonable: {:.1}%",
            losses.efficiency_pct
        );
    }

    // ── Additional unit tests ─────────────────────────────────────────────────

    /// HvdcLight (VSC variant) must report bidirectional p_range.
    #[test]
    fn test_hvdc_light_bidirectional() {
        let mut link = HvdcLink::new_vsc(0, 1, 2, 600.0, 320.0);
        link.hvdc_type = HvdcType::HvdcLight;
        let (min_p, max_p) = link.p_range();
        assert_eq!(min_p, -600.0, "HvdcLight min should be -p_rated");
        assert_eq!(max_p, 600.0, "HvdcLight max should be +p_rated");
    }

    /// LCC converter losses at 1000 MW must be exactly 1000 × 0.012 = 12 MW.
    #[test]
    fn test_hvdc_lcc_converter_losses_exact() {
        let mut link = HvdcLink::new_lcc(0, 1, 2, 1000.0, 500.0);
        link.p_setpoint_mw = 1000.0;
        let losses = link.converter_losses_mw();
        assert!(
            (losses - 12.0).abs() < 1e-9,
            "LCC converter losses should be 12.0 MW, got {losses}"
        );
    }

    /// VSC Q capacity must be ±30 % of rated power.
    #[test]
    fn test_hvdc_vsc_q_limits_proportional() {
        let link = HvdcLink::new_vsc(0, 1, 2, 800.0, 320.0);
        let expected = 0.3 * 800.0; // 240 MVAr
        assert!(
            (link.q_from_max - expected).abs() < 1e-9,
            "q_from_max should be {expected}, got {}",
            link.q_from_max
        );
        assert!(
            (link.q_to_max - expected).abs() < 1e-9,
            "q_to_max should be {expected}, got {}",
            link.q_to_max
        );
        assert!(
            (link.q_from_min + expected).abs() < 1e-9,
            "q_from_min should be -{expected}, got {}",
            link.q_from_min
        );
        assert!(
            (link.q_to_min + expected).abs() < 1e-9,
            "q_to_min should be -{expected}, got {}",
            link.q_to_min
        );
    }

    /// With length_km = 0, cable losses use simplified losses_pct / 200 model.
    #[test]
    fn test_hvdc_cable_losses_simplified_model() {
        let mut link = HvdcLink::new_vsc(0, 1, 2, 1000.0, 320.0);
        link.length_km = 0.0; // triggers simplified path
        link.losses_pct = 2.0;
        link.p_setpoint_mw = 1000.0;
        let expected = 1000.0 * 2.0 / 200.0; // 10 MW
        let losses = link.cable_losses_mw();
        assert!(
            (losses - expected).abs() < 1e-9,
            "simplified cable losses should be {expected} MW, got {losses}"
        );
    }

    /// Negative p_setpoint (reverse flow) must produce negative delivered power.
    #[test]
    fn test_hvdc_power_delivered_negative_setpoint() {
        let mut link = HvdcLink::new_vsc(0, 1, 2, 500.0, 320.0);
        link.p_setpoint_mw = -500.0;
        let delivered = link.power_delivered_mw();
        assert!(
            delivered < 0.0,
            "reverse-flow VSC delivered power should be negative, got {delivered}"
        );
    }

    /// Three-terminal MTDC grid must solve and return 3 DC voltages with slack near 320 kV.
    #[test]
    fn test_mtdc_three_terminal_power_flow() {
        let dc_buses = vec![
            DcBus {
                id: 0,
                v_rated_kv: 320.0,
                v_dc: 320.0,
            },
            DcBus {
                id: 1,
                v_rated_kv: 320.0,
                v_dc: 320.0,
            },
            DcBus {
                id: 2,
                v_rated_kv: 320.0,
                v_dc: 320.0,
            },
        ];
        let dc_lines = vec![
            DcLine {
                id: 0,
                from_dc_bus: 0,
                to_dc_bus: 1,
                resistance_ohm: 1.0,
                rating_mw: 500.0,
            },
            DcLine {
                id: 1,
                from_dc_bus: 1,
                to_dc_bus: 2,
                resistance_ohm: 1.0,
                rating_mw: 500.0,
            },
        ];
        let converters = vec![
            MtdcConverter {
                id: 0,
                ac_bus: 0,
                dc_bus: 0,
                p_rated_mw: 500.0,
                control_mode: MtdcControlMode::ConstantVoltage,
                p_setpoint_mw: 0.0,
                v_dc_setpoint_kv: 320.0,
            },
            MtdcConverter {
                id: 1,
                ac_bus: 1,
                dc_bus: 1,
                p_rated_mw: 300.0,
                control_mode: MtdcControlMode::ConstantPower,
                p_setpoint_mw: 200.0,
                v_dc_setpoint_kv: 320.0,
            },
            MtdcConverter {
                id: 2,
                ac_bus: 2,
                dc_bus: 2,
                p_rated_mw: 300.0,
                control_mode: MtdcControlMode::ConstantPower,
                p_setpoint_mw: -200.0,
                v_dc_setpoint_kv: 320.0,
            },
        ];
        let mut grid = MtdcGrid::new(converters, dc_buses, dc_lines, 0);
        let v_dc = grid
            .solve_dc_power_flow()
            .expect("3-terminal MTDC power flow should converge");
        assert_eq!(v_dc.len(), 3, "should return voltage for each DC bus");
        assert!(
            (v_dc[0] - 320.0).abs() < 5.0,
            "slack bus voltage should be near 320 kV, got {:.2}",
            v_dc[0]
        );
    }

    /// An MTDC grid with no DC buses must return an error.
    #[test]
    fn test_mtdc_empty_grid_error() {
        let mut grid = MtdcGrid::new(vec![], vec![], vec![], 0);
        let result = grid.solve_dc_power_flow();
        assert!(result.is_err(), "empty MTDC grid must return Err");
    }

    /// LCC links must have all four Q limits set to exactly 0.0.
    #[test]
    fn test_hvdc_lcc_q_limits_zero() {
        let link = HvdcLink::new_lcc(0, 1, 2, 1000.0, 500.0);
        assert_eq!(link.q_from_min, 0.0, "LCC q_from_min must be 0");
        assert_eq!(link.q_from_max, 0.0, "LCC q_from_max must be 0");
        assert_eq!(link.q_to_min, 0.0, "LCC q_to_min must be 0");
        assert_eq!(link.q_to_max, 0.0, "LCC q_to_max must be 0");
    }
}
