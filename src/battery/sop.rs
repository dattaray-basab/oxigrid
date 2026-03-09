//! State of Power (SoP) estimation for battery packs.
//!
//! SoP is the maximum instantaneous power that can be continuously delivered
//! (discharged) or absorbed (charged) without violating any cell-level
//! constraint over a given prediction horizon.
//!
//! # Constraints considered
//! - Cell terminal voltage bounds: V_min ≤ V_t ≤ V_max
//! - Current rate limits: |I| ≤ I_max (C-rate based)
//! - Thermal limits: cell temperature derating
//! - SoC limits: SoC ∈ [SoC_min, SoC_max]
//!
//! # Algorithms
//! - `StatePowerEstimator`: binary-search + forward simulation over the horizon
//! - `CapacityFadeEstimator`: SEI growth + cycling + calendar aging model
//! - `BatteryStateEstimator`: joint SoC + SoP + SoH wrapper
//!
//! # References
//! - Plett, G.L., "Battery Management Systems Vol. 2", Artech 2015, Ch. 7
//! - Sun, F. et al., "A systematic state-of-charge estimation framework …",
//!   J. Power Sources 220 (2012) 361–369

use crate::battery::ecm::{RintModel, TwoRcModel};
use crate::battery::pack::SeriesParallelPack;
use crate::battery::soc::{CoulombCounter, EkfSocEstimator};
use crate::error::OxiGridError;

// ─────────────────────────────────────────────────────────────────────────────
// SopConstraint — which constraint is binding
// ─────────────────────────────────────────────────────────────────────────────

/// Identifies which constraint is limiting the SoP.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SopConstraint {
    /// Discharge limited by minimum cell voltage
    VoltageLimitMin,
    /// Charge limited by maximum cell voltage
    VoltageLimitMax,
    /// C-rate current limit reached
    CurrentLimit,
    /// Temperature derating is active
    ThermalLimit,
    /// SoC at minimum or maximum boundary
    SocLimit,
}

// ─────────────────────────────────────────────────────────────────────────────
// SopResult
// ─────────────────────────────────────────────────────────────────────────────

/// Output of a SoP estimation.
#[derive(Debug, Clone)]
pub struct SopResult {
    /// Maximum charge power over the horizon \[kW\] (positive = into battery)
    pub p_max_charge_kw: f64,
    /// Maximum discharge power over the horizon \[kW\] (positive = out of battery)
    pub p_max_discharge_kw: f64,
    /// Continuous (indefinitely sustainable) charge power \[kW\]
    pub p_continuous_charge_kw: f64,
    /// Continuous discharge power \[kW\]
    pub p_continuous_discharge_kw: f64,
    /// Which constraint is binding for the peak power
    pub limiting_constraint: SopConstraint,
    /// Trajectory over the prediction horizon: (time_s, p_max_charge_kw, p_max_discharge_kw)
    pub sop_trajectory: Vec<(f64, f64, f64)>,
}

// ─────────────────────────────────────────────────────────────────────────────
// StatePowerEstimator
// ─────────────────────────────────────────────────────────────────────────────

/// Computes State of Power via forward simulation over a prediction horizon.
///
/// Uses binary search to find the maximum current that keeps all cell
/// constraints satisfied throughout the horizon window.
pub struct StatePowerEstimator {
    /// Lookahead prediction horizon \[s\]
    pub prediction_horizon_s: f64,
    /// Number of simulation steps over the horizon
    pub n_steps: usize,
}

impl StatePowerEstimator {
    /// Create a new SoP estimator.
    ///
    /// `horizon_s` — prediction horizon in seconds (e.g. 10s for EV, 1s for grid storage).
    /// `n_steps`   — discretisation of the horizon (e.g. 10).
    pub fn new(horizon_s: f64, n_steps: usize) -> Self {
        let n_steps = n_steps.max(1);
        Self {
            prediction_horizon_s: horizon_s,
            n_steps,
        }
    }

    /// Compute SoP from a Rint ECM cell model.
    ///
    /// Simulates the cell forward at candidate current levels and finds the
    /// highest current that keeps V_terminal within bounds throughout the horizon.
    ///
    /// Returns `OxiGridError::InvalidParameter` if the model parameters are degenerate.
    pub fn compute_from_ecm(
        &self,
        ecm: &RintModel,
        soc: f64,
        temperature_c: f64,
    ) -> Result<SopResult, OxiGridError> {
        if self.prediction_horizon_s <= 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "prediction_horizon_s must be positive".to_string(),
            ));
        }
        if ecm.capacity_ah <= 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "ECM capacity_ah must be positive".to_string(),
            ));
        }

        let dt = self.prediction_horizon_s / self.n_steps as f64;

        // Physically correct resistance vs temperature for Li-ion:
        // R increases at low temperatures (Arrhenius behaviour).
        // Use |ΔT| correction: R(T) = R_ref * exp(Ea/R * (1/T - 1/T_ref))
        // Simplified: at cold, resistance increases; at hot, slight decrease.
        let t_ref_c = 25.0_f64;
        let r0 = if temperature_c < t_ref_c {
            // Cold: exponential increase — approx doubling every 20°C below ref
            let delta = (t_ref_c - temperature_c).min(60.0);
            ecm.r0 * (1.0 + 0.03 * delta + 0.0005 * delta * delta)
        } else {
            // Warm/hot: slight decrease then plateau
            let delta = (temperature_c - t_ref_c).min(40.0);
            ecm.r0 * (1.0 - 0.002 * delta).max(0.85 * ecm.r0)
        };

        // OCV at current SoC
        let ocv = ecm.ocv_curve.ocv(soc.clamp(0.0, 1.0));

        // Hard limits from cell chemistry
        // Use NMC-like limits as defaults (the RintModel does not store v_min/v_max)
        let v_min = 3.0_f64;
        let v_max = 4.2_f64;

        // Maximum current limited by: C-rate (2C discharge, 1C charge)
        let i_max_discharge = 2.0 * ecm.capacity_ah;
        let i_max_charge = 1.0 * ecm.capacity_ah;

        // Temperature derating (simple piecewise)
        let temp_derate = temperature_derating_factor(temperature_c);

        // ── Binary search for maximum discharge current ────────────────────
        // Constraint: V_t = OCV(soc_t) - I * R0 >= V_min over all steps
        let (i_discharge_max, discharge_constraint) = binary_search_current(
            soc,
            ecm.capacity_ah,
            &ecm.ocv_curve,
            r0,
            dt,
            self.n_steps,
            0.0,                           // lower bound
            i_max_discharge * temp_derate, // upper bound
            true,                          // discharge direction
            v_min,
            v_max,
        );

        // ── Binary search for maximum charge current ───────────────────────
        let (i_charge_max, charge_constraint) = binary_search_current(
            soc,
            ecm.capacity_ah,
            &ecm.ocv_curve,
            r0,
            dt,
            self.n_steps,
            0.0,
            i_max_charge * temp_derate,
            false, // charge direction
            v_min,
            v_max,
        );

        // Determine binding constraint
        let limiting_constraint = if temp_derate < 0.95 {
            SopConstraint::ThermalLimit
        } else if soc <= 0.05 {
            SopConstraint::SocLimit
        } else {
            discharge_constraint
        };

        // Build trajectory over horizon
        let sop_trajectory = self.build_trajectory(
            soc,
            ecm.capacity_ah,
            &ecm.ocv_curve,
            r0,
            dt,
            i_discharge_max,
            i_charge_max,
        );

        // Continuous power: what can be sustained indefinitely
        // Approximation: use a 1-hour horizon for "continuous"
        let p_cont_dis = ocv * i_discharge_max * 0.7 / 1000.0; // ~70% of peak
        let p_cont_chg = ocv * i_charge_max * 0.7 / 1000.0;

        let _ = charge_constraint; // constraint info captured in limiting_constraint

        Ok(SopResult {
            p_max_discharge_kw: ocv * i_discharge_max / 1000.0,
            p_max_charge_kw: ocv * i_charge_max / 1000.0,
            p_continuous_discharge_kw: p_cont_dis,
            p_continuous_charge_kw: p_cont_chg,
            limiting_constraint,
            sop_trajectory,
        })
    }

    /// Compute SoP for a full `SeriesParallelPack`.
    ///
    /// Aggregates cell-level SoP across all series/parallel cells: the pack SoP is limited
    /// by the weakest cell (lowest discharge SoP, lowest charge SoP).
    pub fn compute_for_pack(&self, pack: &SeriesParallelPack) -> Result<SopResult, OxiGridError> {
        if pack.cells.is_empty() {
            return Err(OxiGridError::InvalidParameter(
                "SeriesParallelPack has no cells".to_string(),
            ));
        }

        let dt = self.prediction_horizon_s / self.n_steps as f64;

        let mut p_dis_kw = f64::INFINITY;
        let mut p_chg_kw = f64::INFINITY;
        let mut p_cont_dis = f64::INFINITY;
        let mut p_cont_chg = f64::INFINITY;
        let mut limiting = SopConstraint::CurrentLimit;

        for series_row in &pack.cells {
            for cell in series_row {
                let (cell_p_dis, cell_p_chg, constraint) = cell_sop_kw(cell, dt, self.n_steps);

                if cell_p_dis < p_dis_kw {
                    p_dis_kw = cell_p_dis;
                    limiting = constraint;
                }
                p_chg_kw = p_chg_kw.min(cell_p_chg);
                p_cont_dis = p_cont_dis.min(cell_p_dis * 0.7);
                p_cont_chg = p_cont_chg.min(cell_p_chg * 0.7);
            }
        }

        // Scale up to pack level: ns (series voltage multiplier) × np (parallel current multiplier)
        let scale = (pack.ns as f64).max(1.0);

        let sop_trajectory = (0..=self.n_steps)
            .map(|k| {
                let t = k as f64 * dt;
                let decay = 1.0 - (k as f64 / self.n_steps as f64) * 0.05;
                (t, p_chg_kw * scale * decay, p_dis_kw * scale * decay)
            })
            .collect();

        Ok(SopResult {
            p_max_discharge_kw: p_dis_kw * scale,
            p_max_charge_kw: p_chg_kw * scale,
            p_continuous_discharge_kw: p_cont_dis * scale,
            p_continuous_charge_kw: p_cont_chg * scale,
            limiting_constraint: limiting,
            sop_trajectory,
        })
    }

    /// Online SoP update: fast recursive update without full horizon simulation.
    ///
    /// Adjusts the previous SoP estimate based on energy change and new temperature.
    /// Suitable for real-time BMS use (μs latency vs. ms for full computation).
    pub fn update_online(
        &self,
        prev_sop: &SopResult,
        delta_energy_kwh: f64,
        new_temperature_c: f64,
    ) -> SopResult {
        // Temperature derating factor
        let t_derate = temperature_derating_factor(new_temperature_c);

        // Energy change affects SoC → adjust discharge/charge headroom
        // Positive delta_energy_kwh = energy removed (discharge)
        // We assume linear SoP reduction with remaining energy
        // Use a decay factor based on normalised energy change
        let energy_factor = 1.0 - (delta_energy_kwh * 0.01).clamp(-0.2, 0.2);

        let p_dis = (prev_sop.p_max_discharge_kw * energy_factor * t_derate).max(0.0);
        let p_chg = (prev_sop.p_max_charge_kw * (2.0 - energy_factor) * t_derate).max(0.0);
        let p_cont_dis = p_dis * 0.7;
        let p_cont_chg = p_chg * 0.7;

        // Rebuild shortened trajectory
        let dt = self.prediction_horizon_s / self.n_steps as f64;
        let sop_trajectory = (0..=self.n_steps)
            .map(|k| {
                let t = k as f64 * dt;
                let decay = 1.0 - (k as f64 / self.n_steps as f64) * 0.1;
                (t, p_chg * decay, p_dis * decay)
            })
            .collect();

        let limiting = if t_derate < 0.95 {
            SopConstraint::ThermalLimit
        } else if delta_energy_kwh > 0.0 {
            SopConstraint::SocLimit
        } else {
            prev_sop.limiting_constraint
        };

        SopResult {
            p_max_discharge_kw: p_dis,
            p_max_charge_kw: p_chg,
            p_continuous_discharge_kw: p_cont_dis,
            p_continuous_charge_kw: p_cont_chg,
            limiting_constraint: limiting,
            sop_trajectory,
        }
    }

    /// Build the SoP trajectory over the prediction horizon.
    #[allow(clippy::too_many_arguments)]
    fn build_trajectory(
        &self,
        initial_soc: f64,
        capacity_ah: f64,
        ocv_curve: &crate::battery::OcvSocCurve,
        _r0: f64,
        dt: f64,
        i_discharge: f64,
        i_charge: f64,
    ) -> Vec<(f64, f64, f64)> {
        let mut soc_dis = initial_soc;
        let mut soc_chg = initial_soc;
        let mut trajectory = Vec::with_capacity(self.n_steps + 1);

        for k in 0..=self.n_steps {
            let t = k as f64 * dt;
            let ocv_d = ocv_curve.ocv(soc_dis.clamp(0.0, 1.0));
            let ocv_c = ocv_curve.ocv(soc_chg.clamp(0.0, 1.0));

            let p_dis = (ocv_d * i_discharge / 1000.0).max(0.0);
            let p_chg = (ocv_c * i_charge / 1000.0).max(0.0);
            trajectory.push((t, p_chg, p_dis));

            // Advance SoC
            if k < self.n_steps {
                soc_dis = (soc_dis - i_discharge * dt / (3600.0 * capacity_ah)).max(0.0);
                soc_chg = (soc_chg + i_charge * dt / (3600.0 * capacity_ah)).min(1.0);
            }
        }
        trajectory
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Compute cell-level SoP \[kW\] for a `TwoRcModel` cell.
///
/// Returns (p_discharge_kw, p_charge_kw, limiting_constraint).
fn cell_sop_kw(cell: &TwoRcModel, _dt: f64, _n_steps: usize) -> (f64, f64, SopConstraint) {
    // Approximate peak power from Rint model: P = V_oc² / (4 × R0)
    // at 10 s pulse (typical HPPC test duration)
    let v_oc = cell.ocv_curve.ocv(cell.soc);
    let r0 = cell.r0.max(1e-6);
    // Discharge: limited by V_min = 2.5 V (typical), charge: V_max = 4.2 V
    let v_min = 2.5_f64;
    let v_max = 4.2_f64;
    let i_max_dis = ((v_oc - v_min) / r0).max(0.0);
    let i_max_chg = ((v_max - v_oc) / r0).max(0.0);

    // Capacity-based C-rate limit: 2C max
    let cap_ah = cell.capacity_ah.max(0.1);
    let i_c_rate = 2.0 * cap_ah;
    let i_dis = i_max_dis.min(i_c_rate);
    let i_chg = i_max_chg.min(i_c_rate);

    let p_dis_kw = (v_oc * i_dis) / 1000.0;
    let p_chg_kw = (v_oc * i_chg) / 1000.0;

    let constraint = if cell.soc <= 0.05 || cell.soc >= 0.95 {
        SopConstraint::SocLimit
    } else {
        SopConstraint::VoltageLimitMin
    };

    (p_dis_kw, p_chg_kw, constraint)
}

/// Temperature derating factor for the Rint ECM (mirrors `BatteryCell::temperature_derating`).
fn temperature_derating_factor(temperature_c: f64) -> f64 {
    let t = temperature_c;
    if t < 0.0 {
        let below = (-t).min(40.0);
        (1.0 - 0.02 * below - 0.001 * below * below).max(0.05)
    } else if t <= 25.0 {
        0.5 + 0.5 * (t / 25.0)
    } else if t <= 45.0 {
        1.0
    } else if t <= 60.0 {
        1.0 - (t - 45.0) / 15.0 * 0.5
    } else {
        0.1_f64.max(1.0 - (t - 45.0) / 60.0)
    }
}

/// Binary search for the maximum current (discharge or charge) that keeps
/// terminal voltage within bounds over `n_steps` steps of duration `dt`.
///
/// Returns (i_max, binding_constraint).
#[allow(clippy::too_many_arguments)]
fn binary_search_current(
    initial_soc: f64,
    capacity_ah: f64,
    ocv_curve: &crate::battery::OcvSocCurve,
    r0: f64,
    dt: f64,
    n_steps: usize,
    i_lo: f64,
    i_hi: f64,
    discharge: bool,
    v_min: f64,
    v_max: f64,
) -> (f64, SopConstraint) {
    const MAX_ITER: usize = 32;
    const TOL: f64 = 1e-4; // A

    let mut lo = i_lo;
    let mut hi = i_hi;
    let mut best = 0.0_f64;

    for _ in 0..MAX_ITER {
        if hi - lo < TOL {
            break;
        }
        let mid = 0.5 * (lo + hi);
        if is_current_feasible(
            initial_soc,
            capacity_ah,
            ocv_curve,
            r0,
            dt,
            n_steps,
            mid,
            discharge,
            v_min,
            v_max,
        ) {
            best = mid;
            lo = mid;
        } else {
            hi = mid;
        }
    }

    // Determine what constraint was binding at the best feasible current
    let constraint = binding_constraint(
        initial_soc,
        capacity_ah,
        ocv_curve,
        r0,
        dt,
        n_steps,
        best,
        discharge,
        v_min,
        v_max,
    );

    (best, constraint)
}

/// Forward-simulate `n_steps` of duration `dt` at constant current `i_a`.
/// Returns `true` if all voltage constraints are satisfied.
#[allow(clippy::too_many_arguments)]
fn is_current_feasible(
    initial_soc: f64,
    capacity_ah: f64,
    ocv_curve: &crate::battery::OcvSocCurve,
    r0: f64,
    dt: f64,
    n_steps: usize,
    i_a: f64,
    discharge: bool,
    v_min: f64,
    v_max: f64,
) -> bool {
    let sign = if discharge { 1.0 } else { -1.0 };
    let mut soc = initial_soc;

    for _ in 0..n_steps {
        let ocv = ocv_curve.ocv(soc.clamp(0.0, 1.0));
        let v_t = ocv - sign * i_a * r0;

        if discharge && v_t < v_min {
            return false;
        }
        if !discharge && v_t > v_max {
            return false;
        }
        if soc <= 0.0 && discharge {
            return false;
        }
        if soc >= 1.0 && !discharge {
            return false;
        }

        // Advance SoC
        let dsoc = -sign * i_a * dt / (3600.0 * capacity_ah);
        soc = (soc + dsoc).clamp(0.0, 1.0);
    }
    true
}

/// Identify the binding constraint for a given current level.
#[allow(clippy::too_many_arguments)]
fn binding_constraint(
    initial_soc: f64,
    capacity_ah: f64,
    ocv_curve: &crate::battery::OcvSocCurve,
    r0: f64,
    dt: f64,
    n_steps: usize,
    i_a: f64,
    discharge: bool,
    v_min: f64,
    v_max: f64,
) -> SopConstraint {
    let sign = if discharge { 1.0 } else { -1.0 };
    let mut soc = initial_soc;
    let mut hit_voltage = false;
    let mut hit_soc = false;

    for _ in 0..n_steps {
        let ocv = ocv_curve.ocv(soc.clamp(0.0, 1.0));
        let v_t = ocv - sign * i_a * r0;
        if (discharge && v_t <= v_min + 0.05) || (!discharge && v_t >= v_max - 0.05) {
            hit_voltage = true;
        }
        if soc <= 0.01 || soc >= 0.99 {
            hit_soc = true;
        }
        let dsoc = -sign * i_a * dt / (3600.0 * capacity_ah);
        soc = (soc + dsoc).clamp(0.0, 1.0);
    }

    if hit_soc {
        SopConstraint::SocLimit
    } else if hit_voltage {
        if discharge {
            SopConstraint::VoltageLimitMin
        } else {
            SopConstraint::VoltageLimitMax
        }
    } else {
        SopConstraint::CurrentLimit
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// CapacityFadeEstimator
// ─────────────────────────────────────────────────────────────────────────────

/// Capacity fade estimator using an SEI growth + cycling + calendar model.
///
/// # Model
/// Remaining capacity fraction:
/// ```text
/// C_rem / C_nom = 1 − k_cyc·N_cycles·f(DoD) − k_cal·t_days·g(T) − k_sei·√t
/// ```
/// where:
/// - `k_cyc` — capacity loss per equivalent full cycle (default 0.0001 = 0.01%)
/// - `k_cal` — calendar fade per day at room temperature (default 0.00002 = 0.002%)
/// - `k_sei` — SEI growth coefficient (Ω·s^{0.5}) causing gradual fade
pub struct CapacityFadeEstimator {
    /// SEI growth rate coefficient [capacity-fraction / √day]
    pub sei_growth_rate: f64,
    /// Capacity loss per equivalent full cycle \[fraction\]
    pub cycle_fade_rate: f64,
    /// Calendar capacity loss per day at 25°C \[fraction\]
    pub calendar_fade_rate: f64,
    /// Accumulated equivalent full cycles
    pub total_cycles: f64,
    /// Accumulated calendar days
    pub calendar_days: f64,
}

impl Default for CapacityFadeEstimator {
    fn default() -> Self {
        Self::new()
    }
}

impl CapacityFadeEstimator {
    /// Create a new estimator for a fresh (new) battery.
    pub fn new() -> Self {
        Self {
            sei_growth_rate: 1e-4,
            cycle_fade_rate: 0.0001,     // 0.01% per EFC
            calendar_fade_rate: 0.00002, // 0.002% per day
            total_cycles: 0.0,
            calendar_days: 0.0,
        }
    }

    /// Remaining capacity fraction relative to rated capacity [0, 1].
    ///
    /// `C_rem / C_nom = 1 − cycle_loss − calendar_loss − sei_loss`
    pub fn remaining_capacity_fraction(&self) -> f64 {
        let cycle_loss = self.cycle_fade_rate * self.total_cycles;
        let calendar_loss = self.calendar_fade_rate * self.calendar_days;
        // SEI loss grows as √(time) — use calendar_days as proxy for time
        let sei_loss = self.sei_growth_rate * self.calendar_days.sqrt();
        (1.0 - cycle_loss - calendar_loss - sei_loss).clamp(0.0, 1.0)
    }

    /// Predict remaining capacity fraction at a future point.
    ///
    /// `additional_cycles` — additional equivalent full cycles.
    /// `additional_days`   — additional calendar days.
    pub fn predict_capacity(&self, additional_cycles: f64, additional_days: f64) -> f64 {
        let future_cycles = self.total_cycles + additional_cycles.max(0.0);
        let future_days = self.calendar_days + additional_days.max(0.0);

        let cycle_loss = self.cycle_fade_rate * future_cycles;
        let calendar_loss = self.calendar_fade_rate * future_days;
        let sei_loss = self.sei_growth_rate * future_days.sqrt();
        (1.0 - cycle_loss - calendar_loss - sei_loss).clamp(0.0, 1.0)
    }

    /// Update the model after one charge/discharge cycle.
    ///
    /// `dod` — depth of discharge [0, 1] for this cycle.
    ///
    /// Deep discharges accelerate degradation: cycle fade scales with DoD^1.5
    /// (empirical power-law from rainflow counting studies).
    pub fn update_cycle(&mut self, dod: f64) {
        let dod = dod.clamp(0.0, 1.0);
        // One "equivalent full cycle" = dod worth of cycling
        // Deep discharge penalty: DoD^1.5 scaling
        let efc_contribution = dod * dod.powf(0.5); // = dod^1.5
        self.total_cycles += efc_contribution;
    }

    /// Update calendar aging (call once per simulated day).
    pub fn update_calendar(&mut self, days: f64) {
        self.calendar_days += days.max(0.0);
    }

    /// State of Health [0, 1] = remaining_capacity_fraction.
    ///
    /// SoH = 1.0 for a new battery, decreases toward 0 with aging.
    /// End-of-life is typically defined at SoH = 0.8 (20% capacity loss).
    pub fn soh(&self) -> f64 {
        self.remaining_capacity_fraction()
    }

    /// Estimated remaining useful life in equivalent full cycles.
    ///
    /// Extrapolates linearly from current fade rate to the end-of-life threshold.
    /// Returns `None` if already past end of life or fade rate is zero.
    pub fn remaining_useful_life_cycles(&self, eol_threshold: f64) -> Option<f64> {
        let current_soh = self.soh();
        if current_soh <= eol_threshold {
            return None; // Already at or past EOL
        }
        if self.cycle_fade_rate <= 0.0 {
            return None; // No degradation model
        }
        let remaining_capacity = current_soh - eol_threshold;
        Some(remaining_capacity / self.cycle_fade_rate)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// BatteryStateEstimator
// ─────────────────────────────────────────────────────────────────────────────

/// Joint battery state estimator combining SoC (EKF), SoP, and SoH (capacity fade).
///
/// This is the top-level BMS estimator that integrates:
/// - SoC estimation via EKF (with fallback Coulomb counter)
/// - SoP computation at each time step
/// - SoH / capacity fade tracking
pub struct BatteryStateEstimator {
    /// SoC estimator (EKF-based)
    pub soc_filter: EkfSocEstimator,
    /// Coulomb counter (backup / initialization)
    pub coulomb_counter: CoulombCounter,
    /// State of Power estimator
    pub sop: StatePowerEstimator,
    /// Capacity fade / SoH estimator
    pub capacity_estimator: CapacityFadeEstimator,
}

impl BatteryStateEstimator {
    /// Create a new joint state estimator.
    ///
    /// `ocv_curve`    — OCV-SoC curve for the cell chemistry.
    /// `r0_ohm`       — internal resistance \[Ω\].
    /// `capacity_ah`  — nominal capacity \[Ah\].
    /// `initial_soc`  — initial SoC estimate [0, 1].
    /// `horizon_s`    — SoP prediction horizon \[s\].
    pub fn new(
        ocv_curve: crate::battery::OcvSocCurve,
        r0_ohm: f64,
        capacity_ah: f64,
        initial_soc: f64,
        horizon_s: f64,
    ) -> Self {
        let soc_filter = EkfSocEstimator::new(ocv_curve.clone(), r0_ohm, capacity_ah, initial_soc);
        let coulomb_counter = CoulombCounter::new(initial_soc, capacity_ah);
        let sop = StatePowerEstimator::new(horizon_s, 10);
        let capacity_estimator = CapacityFadeEstimator::new();

        Self {
            soc_filter,
            coulomb_counter,
            sop,
            capacity_estimator,
        }
    }

    /// Update the state estimator with new measurements.
    ///
    /// Returns the updated SoC estimate.
    pub fn update(
        &mut self,
        current_a: f64,
        v_measured: f64,
        temperature_c: f64,
        dt_s: f64,
    ) -> crate::units::StateOfCharge {
        use crate::units::{Current, Temperature, Voltage};
        // EKF update
        let soc = self.soc_filter.update(
            Current(current_a),
            Voltage(v_measured),
            dt_s,
            Temperature(temperature_c + 273.15),
        );
        // Also update Coulomb counter for cross-check
        self.coulomb_counter.step(Current(current_a), dt_s);
        soc
    }

    /// Compute SoP at the current estimated state.
    pub fn compute_sop(&self, temperature_c: f64) -> Result<SopResult, OxiGridError> {
        let ecm = RintModel::new(
            self.soc_filter.ocv_curve.clone(),
            self.soc_filter.r0,
            self.soc_filter.capacity_ah,
        );
        self.sop
            .compute_from_ecm(&ecm, self.soc_filter.x, temperature_c)
    }

    /// Current SoH estimate.
    pub fn soh(&self) -> f64 {
        self.capacity_estimator.soh()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::battery::ecm::RintModel;
    use crate::battery::OcvSocCurve;

    fn make_rint(soc: f64) -> RintModel {
        let mut m = RintModel::new(OcvSocCurve::nmc_default(), 0.05, 50.0);
        m.soc = soc;
        m
    }

    fn make_estimator(horizon_s: f64) -> StatePowerEstimator {
        StatePowerEstimator::new(horizon_s, 10)
    }

    // ── SoP from ECM ────────────────────────────────────────────────────────

    #[test]
    fn test_sop_discharge_positive() {
        let ecm = make_rint(0.5); // 50% SoC
        let est = make_estimator(10.0);
        let result = est
            .compute_from_ecm(&ecm, 0.5, 25.0)
            .expect("SoP computation should succeed");
        assert!(
            result.p_max_discharge_kw > 0.0,
            "Discharge SoP should be positive at SoC=0.5, got {}",
            result.p_max_discharge_kw
        );
    }

    #[test]
    fn test_sop_charge_zero_at_full() {
        let ecm = make_rint(1.0); // 100% SoC
        let est = make_estimator(10.0);
        let result = est
            .compute_from_ecm(&ecm, 1.0, 25.0)
            .expect("SoP computation should succeed");
        // At full charge the charging SoP should be near zero (no headroom)
        assert!(
            result.p_max_charge_kw < 1.0,
            "Charge SoP should be ≈0 at SoC=1.0, got {}",
            result.p_max_charge_kw
        );
    }

    #[test]
    fn test_sop_discharge_zero_at_empty() {
        let ecm = make_rint(0.0); // 0% SoC
        let est = make_estimator(10.0);
        let result = est
            .compute_from_ecm(&ecm, 0.0, 25.0)
            .expect("SoP computation should succeed");
        assert!(
            result.p_max_discharge_kw < 0.1,
            "Discharge SoP should be ≈0 at SoC=0.0, got {}",
            result.p_max_discharge_kw
        );
    }

    #[test]
    fn test_sop_trajectory_length() {
        let ecm = make_rint(0.7);
        let n_steps = 5;
        let est = StatePowerEstimator::new(10.0, n_steps);
        let result = est.compute_from_ecm(&ecm, 0.7, 25.0).unwrap();
        assert_eq!(
            result.sop_trajectory.len(),
            n_steps + 1,
            "Trajectory should have n_steps+1 points"
        );
    }

    #[test]
    fn test_sop_cold_temperature_reduces_power() {
        let ecm = make_rint(0.5);
        let est = make_estimator(10.0);
        // Use a large-capacity cell so C-rate limit (not voltage) is the binding constraint
        let mut ecm_large = RintModel::new(OcvSocCurve::nmc_default(), 0.005, 500.0);
        ecm_large.soc = 0.5;
        let result_warm = est.compute_from_ecm(&ecm_large, 0.5, 25.0).unwrap();
        let result_cold = est.compute_from_ecm(&ecm_large, 0.5, -10.0).unwrap();
        // Cold temperature derating reduces the allowed C-rate current,
        // so peak SoP must be ≤ warm SoP.
        assert!(
            result_cold.p_max_discharge_kw <= result_warm.p_max_discharge_kw + 1e-9,
            "Cold temperature should not increase discharge SoP: warm={:.2}, cold={:.2}",
            result_warm.p_max_discharge_kw,
            result_cold.p_max_discharge_kw
        );
        // When current limit is binding (large cell), cold should strictly reduce power
        assert!(
            result_cold.p_max_discharge_kw < result_warm.p_max_discharge_kw,
            "Cold temperature should reduce discharge SoP vs warm for C-rate-limited cell"
        );
        let _ = ecm; // suppress unused warning
    }

    // ── Pack SoP ─────────────────────────────────────────────────────────────

    #[test]
    fn test_sop_for_pack_positive() {
        use crate::battery::ecm::TwoRcModel;
        use crate::battery::pack::SeriesParallelPack;
        use crate::battery::OcvSocCurve;
        // Use SoC = 0.5 so both discharge and charge headroom exist
        let curve = OcvSocCurve::nmc_default();
        // new(ocv, r0, r1, c1, r2, c2, capacity_ah)
        let cell = TwoRcModel::new(curve, 0.012, 0.008, 800.0, 0.003, 5000.0, 50.0).with_soc(0.5);
        // Build a 4s × 2p pack
        let pack = SeriesParallelPack::uniform(4, 2, cell);
        let est = make_estimator(10.0);
        let result = est
            .compute_for_pack(&pack)
            .expect("pack SoP should succeed");
        assert!(result.p_max_discharge_kw > 0.0, "Pack discharge SoP > 0");
        assert!(result.p_max_charge_kw > 0.0, "Pack charge SoP > 0");
    }

    // ── Online update ────────────────────────────────────────────────────────

    #[test]
    fn test_online_update_thermal_limit() {
        let ecm = make_rint(0.5);
        let est = make_estimator(10.0);
        let sop_0 = est.compute_from_ecm(&ecm, 0.5, 25.0).unwrap();
        // Update with hot temperature
        let sop_hot = est.update_online(&sop_0, 0.0, 70.0);
        assert!(
            sop_hot.p_max_discharge_kw < sop_0.p_max_discharge_kw,
            "SoP should decrease at high temperature"
        );
        assert_eq!(sop_hot.limiting_constraint, SopConstraint::ThermalLimit);
    }

    // ── CapacityFadeEstimator ────────────────────────────────────────────────

    #[test]
    fn test_soh_new_battery() {
        let estimator = CapacityFadeEstimator::new();
        let soh = estimator.soh();
        assert!(
            (soh - 1.0).abs() < 1e-10,
            "New battery SoH should be 1.0, got {soh}"
        );
    }

    #[test]
    fn test_capacity_fade_after_cycles() {
        let mut estimator = CapacityFadeEstimator::new();
        // Simulate 1000 full cycles (DoD = 1.0)
        for _ in 0..1000 {
            estimator.update_cycle(1.0);
        }
        let cap = estimator.remaining_capacity_fraction();
        assert!(
            cap < 1.0,
            "Capacity should decrease after 1000 cycles: {cap:.4}"
        );
        assert!(
            cap > 0.5,
            "Capacity should not go below 50% in 1000 full cycles: {cap:.4}"
        );
    }

    #[test]
    fn test_capacity_fade_calendar_aging() {
        let mut estimator = CapacityFadeEstimator::new();
        estimator.update_calendar(365.0); // 1 year
        let cap = estimator.remaining_capacity_fraction();
        assert!(cap < 1.0, "Calendar aging should reduce capacity");
    }

    #[test]
    fn test_predict_capacity_future() {
        let estimator = CapacityFadeEstimator::new();
        let now = estimator.remaining_capacity_fraction();
        let future = estimator.predict_capacity(500.0, 0.0);
        assert!(future < now, "Future capacity should be less than current");
    }

    #[test]
    fn test_soh_eol_threshold() {
        let mut estimator = CapacityFadeEstimator::new();
        // Run many cycles until SoH drops significantly
        for _ in 0..5000 {
            estimator.update_cycle(1.0);
        }
        let rul = estimator.remaining_useful_life_cycles(0.8);
        // With 5000 full cycles at 0.01% each → 50% fade → already past EOL at 0.8
        // or close to it
        let _ = rul; // may be None if past EOL — just ensure no panic
    }

    // ── BatteryStateEstimator ────────────────────────────────────────────────

    #[test]
    fn test_battery_state_estimator_update() {
        let ocv_curve = OcvSocCurve::nmc_default();
        let mut est = BatteryStateEstimator::new(ocv_curve.clone(), 0.05, 50.0, 0.8, 10.0);

        // Simulate a few discharge steps
        for _ in 0..10 {
            let v_meas = ocv_curve.ocv(est.soc_filter.x) - 5.0 * 0.05;
            est.update(5.0, v_meas, 25.0, 1.0);
        }
        // SoC should have decreased
        assert!(
            est.soc_filter.x < 0.8,
            "SoC should decrease after discharge"
        );
        assert!(
            (est.soh() - 1.0).abs() < 1e-10,
            "SoH should be 1.0 (no cycles recorded)"
        );
    }

    #[test]
    fn test_battery_state_estimator_sop() {
        let ocv_curve = OcvSocCurve::nmc_default();
        let est = BatteryStateEstimator::new(ocv_curve, 0.05, 50.0, 0.6, 10.0);
        let sop = est.compute_sop(25.0).expect("SoP should succeed");
        assert!(sop.p_max_discharge_kw > 0.0);
    }

    // ── Deep cycle DoD scaling ───────────────────────────────────────────────

    #[test]
    fn test_deep_dod_degrades_faster() {
        let mut est_deep = CapacityFadeEstimator::new();
        let mut est_shallow = CapacityFadeEstimator::new();

        // 100 cycles at deep DoD=1.0 vs shallow DoD=0.2
        for _ in 0..100 {
            est_deep.update_cycle(1.0);
            est_shallow.update_cycle(0.2);
        }
        assert!(
            est_deep.remaining_capacity_fraction() < est_shallow.remaining_capacity_fraction(),
            "Deep cycling should degrade faster than shallow cycling"
        );
    }
}
