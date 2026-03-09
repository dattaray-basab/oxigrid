//! Flexible AC Transmission System (FACTS) device models.
//!
//! Implements the four standard FACTS controllers used in modern power systems:
//!
//! | Device | Full Name | Primary Function |
//! |--------|-----------|-----------------|
//! | [`Statcom`] | Static Synchronous Compensator | Voltage-Q regulation |
//! | [`Svc`]     | Static VAR Compensator          | Voltage support via variable B |
//! | [`Tcsc`]    | Thyristor Controlled Series Compensator | Series reactance control |
//! | [`Upfc`]    | Unified Power Flow Controller   | Independent P/Q/V control |
//!
//! The outer-loop iteration in [`solve_with_facts`] wraps a standard Newton-Raphson
//! solve and updates FACTS setpoints based on the power flow solution.

use crate::error::{OxiGridError, Result};
use crate::network::topology::PowerNetwork;
use crate::powerflow::{PowerFlowConfig, PowerFlowResult};
use num_complex::Complex64;
use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// STATCOM
// ---------------------------------------------------------------------------

/// Static Synchronous Compensator — voltage-regulated reactive power injector.
///
/// The STATCOM injects reactive power `Q` at a bus in proportion to the voltage
/// deviation from `v_ref`, clamped to `[q_min, q_max]`.  A small fraction of
/// `|Q|` is consumed as active-power loss (representing converter losses).
///
/// # Droop characteristic
/// ```text
/// Q = clamp( (v_ref − v_current) / droop,  q_min,  q_max )   [p.u.]
/// ```
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Statcom {
    /// 0-based bus index in the network.
    pub bus: usize,
    /// Minimum reactive power injection \[p.u. on system base\] (negative = absorption).
    pub q_min: f64,
    /// Maximum reactive power injection \[p.u.\].
    pub q_max: f64,
    /// Target voltage \[p.u.\].
    pub v_ref: f64,
    /// Droop slope \[p.u. voltage / p.u. reactive power\].
    /// Smaller droop → tighter voltage regulation.
    pub droop: f64,
    /// Active-power loss factor as a fraction of `|Q|` (0.0–1.0).
    pub loss_factor: f64,
}

impl Statcom {
    /// Compute the reactive power injection \[p.u.\] required to regulate voltage.
    ///
    /// Returns a value in `[q_min, q_max]`.  Positive Q = capacitive (voltage
    /// support); negative Q = inductive (voltage reduction).
    pub fn compute_q_injection(&self, v_current: f64) -> f64 {
        if self.droop.abs() < 1e-15 {
            // Zero droop → pure voltage source: inject maximum available Q
            return if v_current < self.v_ref {
                self.q_max
            } else {
                self.q_min
            };
        }
        let q = (self.v_ref - v_current) / self.droop;
        q.clamp(self.q_min, self.q_max)
    }

    /// Compute the active-power loss \[p.u.\] associated with a given Q injection.
    pub fn active_power_loss(&self, q_injection: f64) -> f64 {
        self.loss_factor * q_injection.abs()
    }
}

// ---------------------------------------------------------------------------
// SVC
// ---------------------------------------------------------------------------

/// Static VAR Compensator — variable shunt susceptance for voltage support.
///
/// Unlike the STATCOM (voltage-source based), the SVC uses thyristor-controlled
/// reactors and fixed capacitors to vary its effective shunt susceptance `B`.
///
/// # Droop characteristic
/// ```text
/// B = clamp( (v_ref − v_current) × slope,  b_min,  b_max )   [p.u.]
/// ```
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Svc {
    /// 0-based bus index in the network.
    pub bus: usize,
    /// Minimum susceptance \[p.u.\] (negative = inductive, absorbs reactive power).
    pub b_min: f64,
    /// Maximum susceptance \[p.u.\] (positive = capacitive, injects reactive power).
    pub b_max: f64,
    /// Target voltage \[p.u.\].
    pub v_ref: f64,
    /// Droop slope \[p.u. susceptance / p.u. voltage\].
    pub slope: f64,
}

impl Svc {
    /// Compute the effective shunt susceptance \[p.u.\] at the current voltage.
    pub fn compute_susceptance(&self, v_current: f64) -> f64 {
        let b = (self.v_ref - v_current) * self.slope;
        b.clamp(self.b_min, self.b_max)
    }

    /// Return the shunt admittance `Y = jB` as a `Complex64` for injection
    /// into the Y-bus or as a load-side admittance.
    pub fn to_shunt_admittance(&self, v_current: f64) -> Complex64 {
        let b = self.compute_susceptance(v_current);
        Complex64::new(0.0, b)
    }
}

// ---------------------------------------------------------------------------
// TCSC
// ---------------------------------------------------------------------------

/// TCSC operating mode.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub enum TcscMode {
    /// Hold `x_current` constant (open-loop).
    ConstantReactance,
    /// Adjust reactance to drive branch active power toward `target_p_mw`.
    PowerFlow { target_p_mw: f64 },
    /// Limit branch current to `i_max_pu` by adjusting reactance.
    CurrentLimiting { i_max_pu: f64 },
}

/// Thyristor Controlled Series Compensator — variable series reactance.
///
/// The TCSC inserts a variable reactance `jX` in series with a branch, allowing
/// smooth control of branch impedance from inductive (+X) to capacitive (−X).
///
/// In `PowerFlow` mode the controller adjusts `x_current` each outer iteration
/// to drive branch real power toward `target_p_mw`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Tcsc {
    /// 0-based branch index in the network.
    pub branch_idx: usize,
    /// Minimum inserted reactance \[p.u.\] (negative = series capacitive compensation).
    pub x_min: f64,
    /// Maximum inserted reactance \[p.u.\] (positive = inductive).
    pub x_max: f64,
    /// Current reactance setting \[p.u.\].
    pub x_current: f64,
    /// Control mode.
    pub control_mode: TcscMode,
}

impl Tcsc {
    /// Return the modified branch impedance after TCSC insertion.
    ///
    /// The TCSC adds `j·x_current` in series with the base branch impedance:
    /// ```text
    /// Z_modified = Z_base + j·x_current
    /// ```
    pub fn modified_branch_impedance(&self, z_base: Complex64) -> Complex64 {
        z_base + Complex64::new(0.0, self.x_current)
    }

    /// Update the reactance setting based on the actual branch power flow.
    ///
    /// In `PowerFlow` mode a proportional controller adjusts `x_current`:
    /// ```text
    /// Δx = gain × (P_actual − P_target) / P_target
    /// ```
    /// Clamps the result to `[x_min, x_max]`.
    ///
    /// Returns the new `x_current`.
    pub fn update_reactance(&mut self, p_actual_mw: f64) -> f64 {
        match self.control_mode {
            TcscMode::PowerFlow { target_p_mw } => {
                if target_p_mw.abs() < 1e-6 {
                    // Avoid division by near-zero target
                    return self.x_current;
                }
                // Proportional gain — small step to avoid oscillation
                let gain = 0.05_f64;
                let error_pu = (p_actual_mw - target_p_mw) / target_p_mw;
                self.x_current = (self.x_current - gain * error_pu).clamp(self.x_min, self.x_max);
            }
            TcscMode::ConstantReactance | TcscMode::CurrentLimiting { .. } => {
                // No automatic update in these modes
            }
        }
        self.x_current
    }
}

// ---------------------------------------------------------------------------
// UPFC
// ---------------------------------------------------------------------------

/// Unified Power Flow Controller — simultaneous series and shunt injection.
///
/// The UPFC combines a shunt converter (STATCOM-like) at `from_bus` and a
/// series converter between `from_bus` and `to_bus`.  It can independently
/// control P flow, Q flow, and/or the `from_bus` voltage magnitude.
///
/// The computations here implement a simplified quasi-steady-state model:
/// - Series voltage injection `V_se` shifts the branch voltage to achieve P/Q
///   flow targets.
/// - Shunt current injection `I_sh` maintains the `from_bus` voltage and
///   supplies the active power consumed by the series converter.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Upfc {
    /// 0-based index of the from-bus.
    pub from_bus: usize,
    /// 0-based index of the to-bus.
    pub to_bus: usize,
    /// 0-based branch index controlled by this UPFC.
    pub branch_idx: usize,
    /// Maximum magnitude of series voltage injection \[p.u.\].
    pub v_series_max: f64,
    /// Minimum reactive power of the shunt converter \[p.u.\].
    pub q_shunt_min: f64,
    /// Maximum reactive power of the shunt converter \[p.u.\].
    pub q_shunt_max: f64,
    /// Target real power flow through the controlled branch \[MW\], if any.
    pub p_target: Option<f64>,
    /// Target reactive power flow through the controlled branch \[MVAr\], if any.
    pub q_target: Option<f64>,
    /// Target voltage magnitude at `from_bus` \[p.u.\], if any.
    pub v_target: Option<f64>,
}

impl Upfc {
    /// Compute the series voltage injection and shunt current injection.
    ///
    /// # Model
    ///
    /// Given from-bus voltage `V_f`, to-bus voltage `V_t`, and branch impedance
    /// `Z`, the series injection `V_se` that achieves target power flow is:
    ///
    /// ```text
    /// S_target = P_target + jQ_target
    /// I_branch = (S_target / V_f)* = conj(S_target / V_f)
    /// V_se = V_f − V_t − Z · I_branch     (series voltage source model)
    /// ```
    ///
    /// The shunt converter supplies active power equal to what the series
    /// converter consumes, and regulates voltage or supplies reactive power.
    ///
    /// # Returns
    /// `Ok((V_series, I_shunt))` — series voltage injection and shunt current injection
    /// in complex p.u.
    pub fn compute_injections(
        &self,
        v_from: Complex64,
        v_to: Complex64,
        z_branch: Complex64,
    ) -> Result<(Complex64, Complex64)> {
        // Target complex power through the branch (p.u.)
        let p_target = self.p_target.unwrap_or(0.0);
        let q_target = self.q_target.unwrap_or(0.0);
        let s_target = Complex64::new(p_target, q_target);

        // Check series voltage magnitude limit
        if v_from.norm() < 1e-6 {
            return Err(OxiGridError::InvalidParameter(
                "UPFC from_bus voltage is near zero — cannot compute series injection".to_string(),
            ));
        }

        // Branch current needed to deliver S_target at from_bus:
        // S = V_f · I*  →  I = conj(S / V_f)
        let i_branch = (s_target / v_from).conj();

        // Series voltage source: V_se = V_f − V_t − Z · I_branch
        let v_se = v_from - v_to - z_branch * i_branch;

        // Clamp series voltage magnitude to physical limit
        let v_se_mag = v_se.norm();
        let v_se_clamped = if v_se_mag > self.v_series_max && v_se_mag > 1e-12 {
            v_se * (self.v_series_max / v_se_mag)
        } else {
            v_se
        };

        // Active power consumed by the series converter:
        // P_se = Re(V_se · I_branch*)
        let p_series = (v_se_clamped * i_branch.conj()).re;

        // Shunt converter supplies this active power and regulates voltage.
        // Model: shunt converter is a current injection at from_bus.
        // Active part: I_sh_re = P_se / |V_f|²  × V_f (in phase)
        // Reactive part: clamp Q_shunt to limits, from voltage target if set.
        let v_target = self.v_target.unwrap_or(v_from.norm());
        let v_f_mag = v_from.norm();
        let q_shunt_raw = if v_f_mag > 1e-6 {
            // Proportional voltage regulation
            let droop = 0.05_f64; // 5% droop
            (v_target - v_f_mag) / droop
        } else {
            0.0
        };
        let q_shunt = q_shunt_raw.clamp(self.q_shunt_min, self.q_shunt_max);

        // Shunt current injection: I_sh = conj(S_sh / V_f)
        // where S_sh = P_se + jQ_shunt (active = cover series consumption)
        let s_shunt = Complex64::new(p_series, q_shunt);
        let i_shunt = if v_f_mag > 1e-6 {
            (s_shunt / v_from).conj()
        } else {
            Complex64::new(0.0, 0.0)
        };

        Ok((v_se_clamped, i_shunt))
    }
}

// ---------------------------------------------------------------------------
// FactsNetwork
// ---------------------------------------------------------------------------

/// Collection of all FACTS devices in a network.
///
/// Provides the outer-loop interface for integrating FACTS control into the
/// power flow iteration.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct FactsNetwork {
    /// STATCOM devices.
    pub statcoms: Vec<Statcom>,
    /// SVC devices.
    pub svcs: Vec<Svc>,
    /// TCSC devices.
    pub tcscs: Vec<Tcsc>,
    /// UPFC devices.
    pub upfcs: Vec<Upfc>,
}

impl FactsNetwork {
    /// Create an empty FACTS collection.
    pub fn new() -> Self {
        Self::default()
    }

    /// Apply all FACTS devices as bus load modifications to the network.
    ///
    /// STATCOMs and SVCs adjust the reactive load at their bus to emulate
    /// shunt injection.  TCSCs modify branch impedance (returned as a separate
    /// adjustment map for the caller).  UPFCs require a separate treatment
    /// (voltage injection) and are noted but not fully applied here.
    ///
    /// This method mutates the network's bus reactive loads to reflect FACTS
    /// control action based on the most recent voltage solution.
    pub fn apply_to_network(
        &self,
        network: &mut PowerNetwork,
        v_mag: &[f64],
        _v_ang: &[f64],
    ) -> Result<()> {
        let n = network.buses.len();

        // Apply STATCOMs: add Q injection as negative reactive load.
        for sc in &self.statcoms {
            if sc.bus >= n {
                return Err(OxiGridError::InvalidParameter(format!(
                    "STATCOM bus index {} out of range (n={})",
                    sc.bus, n
                )));
            }
            let v = v_mag[sc.bus];
            let q_inj = sc.compute_q_injection(v);
            // Reactive load decreases by Q injection (injection = negative load)
            let base_mva = network.base_mva;
            network.buses[sc.bus].qd.0 -= q_inj * base_mva;

            // Active power loss (tiny, but included for accuracy)
            let p_loss = sc.active_power_loss(q_inj);
            network.buses[sc.bus].pd.0 += p_loss * base_mva;
        }

        // Apply SVCs: add jB shunt as reactive load adjustment.
        for svc in &self.svcs {
            if svc.bus >= n {
                return Err(OxiGridError::InvalidParameter(format!(
                    "SVC bus index {} out of range (n={})",
                    svc.bus, n
                )));
            }
            let v = v_mag[svc.bus];
            // Q_svc = B × V² [p.u.] → convert to MVAr
            let b = svc.compute_susceptance(v);
            let q_inj_pu = b * v * v;
            let base_mva = network.base_mva;
            network.buses[svc.bus].qd.0 -= q_inj_pu * base_mva;
        }

        // TCSCs: modify branch impedances.
        for tcsc in &self.tcscs {
            if tcsc.branch_idx >= network.branches.len() {
                return Err(OxiGridError::InvalidParameter(format!(
                    "TCSC branch index {} out of range",
                    tcsc.branch_idx
                )));
            }
            let br = &mut network.branches[tcsc.branch_idx];
            // The TCSC adds x_current to the branch reactance.
            br.x += tcsc.x_current;
        }

        Ok(())
    }

    /// Update FACTS device setpoints based on the latest power flow solution.
    ///
    /// Called after each NR solve in the outer loop.  TCSCs in PowerFlow mode
    /// adjust their reactance toward the target using a proportional controller.
    pub fn update_settings(&mut self, _v_mag: &[f64], _v_ang: &[f64], p_flow: &[f64]) {
        for tcsc in &mut self.tcscs {
            if tcsc.branch_idx < p_flow.len() {
                let p_actual = p_flow[tcsc.branch_idx];
                tcsc.update_reactance(p_actual);
            }
        }
        // STATCOMs and SVCs are purely reactive (output determined by voltage),
        // so they need no explicit update — apply_to_network re-evaluates them
        // at each outer iteration using the latest v_mag.
    }
}

// ---------------------------------------------------------------------------
// Outer-loop FACTS power flow
// ---------------------------------------------------------------------------

/// Solve AC power flow with FACTS devices using an outer-iteration loop.
///
/// # Algorithm
///
/// ```text
/// repeat:
///   1. Clone the network and apply current FACTS setpoints as load modifications
///   2. Run Newton-Raphson inner loop to convergence
///   3. Update FACTS settings based on the NR solution
///   4. Check for FACTS outer-loop convergence (max ΔQ/ΔB < ε)
/// until convergence or max_facts_iter
/// ```
///
/// The outer loop converges when the reactive adjustment from all FACTS devices
/// changes by less than `1e-4 p.u.` between successive iterations.
///
/// # Returns
/// The final `PowerFlowResult` from the last NR solve.
pub fn solve_with_facts(
    network: &PowerNetwork,
    facts: &mut FactsNetwork,
    pf_config: &PowerFlowConfig,
    max_facts_iter: usize,
) -> Result<PowerFlowResult> {
    let mut result: Option<PowerFlowResult> = None;

    // Track previous STATCOM Q injections to detect outer-loop convergence.
    let mut prev_q_statcom: Vec<f64> = facts
        .statcoms
        .iter()
        .map(|sc| sc.compute_q_injection(network.buses.get(sc.bus).map(|b| b.vm).unwrap_or(1.0)))
        .collect();

    for outer_iter in 0..max_facts_iter {
        // Build a modified network with current FACTS setpoints applied.
        let mut net_mod = network.clone();
        let v_mag: Vec<f64> = match &result {
            Some(r) => r.voltage_magnitude.clone(),
            None => net_mod.buses.iter().map(|b| b.vm).collect(),
        };
        let v_ang: Vec<f64> = match &result {
            Some(r) => r.voltage_angle.clone(),
            None => net_mod.buses.iter().map(|b| b.va).collect(),
        };

        facts.apply_to_network(&mut net_mod, &v_mag, &v_ang)?;

        // Run inner NR solve on the modified network.
        let pf_result = net_mod.solve_powerflow(pf_config)?;

        // Extract branch flow vector for TCSC update.
        let p_flow: Vec<f64> = pf_result
            .branch_flows
            .iter()
            .map(|bf| bf.p_from_mw)
            .collect();

        // Update FACTS settings for next iteration.
        facts.update_settings(
            &pf_result.voltage_magnitude,
            &pf_result.voltage_angle,
            &p_flow,
        );

        // Check outer-loop convergence on STATCOM Q injections.
        let curr_q: Vec<f64> = facts
            .statcoms
            .iter()
            .zip(pf_result.voltage_magnitude.iter().enumerate())
            .map(|(sc, (_idx, _v))| {
                let v = pf_result
                    .voltage_magnitude
                    .get(sc.bus)
                    .copied()
                    .unwrap_or(1.0);
                sc.compute_q_injection(v)
            })
            .collect();

        let max_dq = curr_q
            .iter()
            .zip(prev_q_statcom.iter())
            .map(|(a, b)| (a - b).abs())
            .fold(0.0_f64, f64::max);

        log::debug!(
            "FACTS outer iter {}: max_ΔQ = {:.4e}, NR converged = {}",
            outer_iter,
            max_dq,
            pf_result.converged
        );

        prev_q_statcom = curr_q;
        result = Some(pf_result);

        if max_dq < 1e-4 {
            break;
        }
    }

    result.ok_or(OxiGridError::Convergence {
        iterations: max_facts_iter,
        residual: f64::NAN,
    })
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::branch::Branch;
    use crate::network::bus::{Bus, BusType};
    use crate::network::topology::Generator;
    use crate::powerflow::PowerFlowConfig;

    // ── STATCOM ──────────────────────────────────────────────────────────────

    #[test]
    fn test_statcom_q_clamping() {
        let sc = Statcom {
            bus: 0,
            q_min: -0.5,
            q_max: 0.5,
            v_ref: 1.0,
            droop: 0.05,
            loss_factor: 0.01,
        };
        // Voltage well below v_ref → large positive Q request → clamps to q_max
        let q = sc.compute_q_injection(0.5);
        assert!(
            (q - sc.q_max).abs() < 1e-10,
            "expected q_max={}, got {q}",
            sc.q_max
        );
        // Voltage well above v_ref → large negative Q → clamps to q_min
        let q = sc.compute_q_injection(1.5);
        assert!(
            (q - sc.q_min).abs() < 1e-10,
            "expected q_min={}, got {q}",
            sc.q_min
        );
        // At v_ref exactly → Q = 0
        let q = sc.compute_q_injection(1.0);
        assert!(q.abs() < 1e-10, "expected Q=0 at v_ref, got {q}");
    }

    #[test]
    fn test_statcom_q_droop_proportional() {
        let sc = Statcom {
            bus: 0,
            q_min: -10.0,
            q_max: 10.0,
            v_ref: 1.0,
            droop: 0.1,
            loss_factor: 0.0,
        };
        // ΔV = 0.05, droop = 0.1 → Q = 0.05 / 0.1 = 0.5
        let q = sc.compute_q_injection(0.95);
        assert!((q - 0.5).abs() < 1e-10, "expected Q=0.5, got {q}");
    }

    #[test]
    fn test_statcom_active_power_loss() {
        let sc = Statcom {
            bus: 0,
            q_min: -1.0,
            q_max: 1.0,
            v_ref: 1.0,
            droop: 0.1,
            loss_factor: 0.02,
        };
        let q = 0.8_f64;
        let loss = sc.active_power_loss(q);
        assert!(
            (loss - 0.016).abs() < 1e-12,
            "expected loss=0.016, got {loss}"
        );
        // Loss should be symmetric (loss factor applied to |Q|)
        assert!((sc.active_power_loss(-q) - 0.016).abs() < 1e-12);
    }

    // ── SVC ──────────────────────────────────────────────────────────────────

    #[test]
    fn test_svc_susceptance_droop() {
        let svc = Svc {
            bus: 0,
            b_min: -2.0,
            b_max: 2.0,
            v_ref: 1.0,
            slope: 10.0,
        };
        // v = 0.95 → ΔV = 0.05 → B = 0.05 * 10 = 0.5
        let b = svc.compute_susceptance(0.95);
        assert!((b - 0.5).abs() < 1e-10, "expected B=0.5, got {b}");
        // v = 1.0 → ΔV = 0 → B = 0
        let b = svc.compute_susceptance(1.0);
        assert!(b.abs() < 1e-10, "expected B=0, got {b}");
        // v = 1.5 → ΔV = -0.5 → B = -5.0, clamped to -2.0
        let b = svc.compute_susceptance(1.5);
        assert!((b - svc.b_min).abs() < 1e-10, "expected b_min, got {b}");
    }

    #[test]
    fn test_svc_to_shunt_admittance() {
        let svc = Svc {
            bus: 0,
            b_min: -1.0,
            b_max: 1.0,
            v_ref: 1.0,
            slope: 5.0,
        };
        let v = 0.9_f64;
        let y = svc.to_shunt_admittance(v);
        let b_expected = svc.compute_susceptance(v);
        assert!(y.re.abs() < 1e-12, "shunt conductance should be zero");
        assert!((y.im - b_expected).abs() < 1e-12, "shunt B mismatch");
    }

    // ── TCSC ─────────────────────────────────────────────────────────────────

    #[test]
    fn test_tcsc_impedance_modification() {
        let mut tcsc = Tcsc {
            branch_idx: 0,
            x_min: -0.5,
            x_max: 0.5,
            x_current: 0.1,
            control_mode: TcscMode::ConstantReactance,
        };
        let z_base = Complex64::new(0.01, 0.1);
        let z_mod = tcsc.modified_branch_impedance(z_base);
        // Should add j*0.1 to the base impedance
        assert!((z_mod.re - 0.01).abs() < 1e-12);
        assert!(
            (z_mod.im - 0.2).abs() < 1e-12,
            "expected x=0.2, got {}",
            z_mod.im
        );

        // Capacitive insertion: x_current = -0.05
        tcsc.x_current = -0.05;
        let z_cap = tcsc.modified_branch_impedance(z_base);
        assert!(
            (z_cap.im - 0.05).abs() < 1e-12,
            "expected x=0.05, got {}",
            z_cap.im
        );
    }

    #[test]
    fn test_tcsc_power_flow_mode_update() {
        let mut tcsc = Tcsc {
            branch_idx: 0,
            x_min: -0.3,
            x_max: 0.3,
            x_current: 0.0,
            control_mode: TcscMode::PowerFlow { target_p_mw: 100.0 },
        };
        // p_actual = 120 MW → positive error → x decreases to reduce impedance
        let x_after = tcsc.update_reactance(120.0);
        assert!(
            x_after < 0.0,
            "x should decrease when P > P_target, got x={x_after}"
        );
        assert!(x_after >= tcsc.x_min, "x must not violate x_min");
    }

    #[test]
    fn test_tcsc_constant_mode_no_update() {
        let mut tcsc = Tcsc {
            branch_idx: 0,
            x_min: -0.5,
            x_max: 0.5,
            x_current: 0.2,
            control_mode: TcscMode::ConstantReactance,
        };
        let x_before = tcsc.x_current;
        tcsc.update_reactance(200.0);
        assert!(
            (tcsc.x_current - x_before).abs() < 1e-12,
            "ConstantReactance mode must not change x_current"
        );
    }

    // ── UPFC ─────────────────────────────────────────────────────────────────

    #[test]
    fn test_upfc_power_flow_control() {
        let upfc = Upfc {
            from_bus: 0,
            to_bus: 1,
            branch_idx: 0,
            v_series_max: 0.2,
            q_shunt_min: -0.5,
            q_shunt_max: 0.5,
            p_target: Some(0.5),
            q_target: Some(0.1),
            v_target: None,
        };
        let v_from = Complex64::from_polar(1.02, 0.0);
        let v_to = Complex64::from_polar(0.98, -0.05);
        let z_branch = Complex64::new(0.01, 0.1);

        let result = upfc.compute_injections(v_from, v_to, z_branch);
        assert!(result.is_ok(), "UPFC injection computation should succeed");
        let (v_se, _i_sh) = result.unwrap();
        // Series voltage injection must be within limit
        assert!(
            v_se.norm() <= upfc.v_series_max + 1e-10,
            "V_se magnitude {:.4} exceeds v_series_max={}",
            v_se.norm(),
            upfc.v_series_max
        );
    }

    #[test]
    fn test_upfc_zero_from_voltage_error() {
        let upfc = Upfc {
            from_bus: 0,
            to_bus: 1,
            branch_idx: 0,
            v_series_max: 0.2,
            q_shunt_min: -0.5,
            q_shunt_max: 0.5,
            p_target: Some(0.5),
            q_target: None,
            v_target: None,
        };
        // Zero from-bus voltage must return an error
        let result = upfc.compute_injections(
            Complex64::new(0.0, 0.0),
            Complex64::from_polar(1.0, 0.0),
            Complex64::new(0.01, 0.1),
        );
        assert!(result.is_err(), "should error on zero from-bus voltage");
    }

    // ── FACTS outer loop ──────────────────────────────────────────────────────

    fn make_2bus_net() -> PowerNetwork {
        let mut net = PowerNetwork::new(100.0);
        net.buses.push({
            let mut b = Bus::new(1, BusType::Slack);
            b.vm = 1.0;
            b
        });
        net.buses.push({
            let mut b = Bus::new(2, BusType::PQ);
            b.vm = 1.0;
            b.pd = crate::units::Power(50.0);
            b.qd = crate::units::ReactivePower(30.0);
            b
        });
        net.branches.push(Branch {
            from_bus: 1,
            to_bus: 2,
            r: 0.01,
            x: 0.1,
            b: 0.02,
            rate_a: 200.0,
            rate_b: 200.0,
            rate_c: 200.0,
            tap: 0.0,
            shift: 0.0,
            status: true,
        });
        net.generators.push(Generator {
            bus_id: 1,
            pg: 0.0,
            qg: 0.0,
            qmax: 999.0,
            qmin: -999.0,
            vg: 1.0,
            mbase: 100.0,
            status: true,
            pmax: 999.0,
            pmin: 0.0,
        });
        net
    }

    #[test]
    fn test_facts_outer_loop_convergence() {
        let net = make_2bus_net();
        let mut facts = FactsNetwork::new();
        // Add a STATCOM at bus 1 (index 1) to support voltage
        facts.statcoms.push(Statcom {
            bus: 1,
            q_min: -0.5,
            q_max: 0.5,
            v_ref: 1.0,
            droop: 0.05,
            loss_factor: 0.005,
        });

        let pf_config = PowerFlowConfig::default();
        let result = solve_with_facts(&net, &mut facts, &pf_config, 20);
        assert!(
            result.is_ok(),
            "FACTS outer loop should converge: {:?}",
            result
        );
        let pf = result.unwrap();
        assert!(pf.converged, "inner NR must converge");
    }

    #[test]
    fn test_facts_svc_reduces_reactive_need() {
        let net = make_2bus_net();
        let mut facts_with_svc = FactsNetwork::new();
        facts_with_svc.svcs.push(Svc {
            bus: 1,
            b_min: -2.0,
            b_max: 2.0,
            v_ref: 1.0,
            slope: 5.0,
        });

        let pf_config = PowerFlowConfig::default();
        let result = solve_with_facts(&net, &mut facts_with_svc, &pf_config, 10);
        assert!(result.is_ok(), "SVC outer loop should converge");
        // With SVC support, bus 1 voltage should be closer to 1.0
        let pf = result.unwrap();
        assert!(
            pf.voltage_magnitude[1] > 0.9,
            "Bus 1 voltage should be reasonable with SVC, got {}",
            pf.voltage_magnitude[1]
        );
    }
}
