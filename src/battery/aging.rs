/// Battery aging and degradation models.
///
/// Implements two complementary degradation mechanisms:
///
/// **Calendar aging** (storage): capacity fade driven by time and temperature.
///   Based on the SEI (Solid Electrolyte Interphase) growth model:
///     Q_loss_cal(t) = k_cal · exp(−E_a / (R·T)) · √t
///
/// **Cycle aging**: capacity fade driven by charge throughput and depth-of-discharge.
///   Based on the Wöhler curve / rain-flow model:
///     Q_loss_cyc = k_cyc · Σ (DoD_i / DoD_ref)^β
///
/// Both contributions are additive; resistance grows proportionally to capacity loss.
use serde::{Deserialize, Serialize};

const GAS_CONSTANT: f64 = 8.314; // J/(mol·K)

/// Parameters for the combined aging model.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AgingParams {
    /// Pre-exponential factor for calendar aging [%/√s]
    pub k_cal: f64,
    /// Activation energy for SEI growth [J/mol]
    pub e_a: f64,
    /// Cycle aging factor [% per cycle]
    pub k_cyc: f64,
    /// DoD exponent (typically 0.5–1.5)
    pub dod_exponent: f64,
    /// Reference DoD for cycle aging coefficient (0–1)
    pub dod_ref: f64,
    /// Initial capacity `Ah`
    pub q_nom: f64,
    /// Initial internal resistance `Ω`
    pub r0_nom: f64,
    /// Resistance growth factor relative to capacity fade
    pub r_growth_factor: f64,
}

impl AgingParams {
    /// Typical LFP cell (long-life chemistry).
    pub fn lfp_default() -> Self {
        Self {
            k_cal: 0.0003, // %/√s at 25°C
            e_a: 24_500.0, // J/mol
            k_cyc: 0.0025, // % per cycle at DoD=1
            dod_exponent: 0.8,
            dod_ref: 1.0,
            q_nom: 75.0,
            r0_nom: 0.0015,
            r_growth_factor: 2.0, // R doubles for 50% capacity loss
        }
    }

    /// Typical NMC cell.
    pub fn nmc_default() -> Self {
        Self {
            k_cal: 0.00045,
            e_a: 27_000.0,
            k_cyc: 0.005,
            dod_exponent: 1.0,
            dod_ref: 1.0,
            q_nom: 3.0,
            r0_nom: 0.020,
            r_growth_factor: 2.5,
        }
    }
}

/// Accumulated aging state.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AgingState {
    /// Calendar age `s`
    pub time_s: f64,
    /// Equivalent full cycles (charge throughput / 2·Q_nom)
    pub equiv_full_cycles: f64,
    /// Capacity loss due to calendar aging [%]
    pub q_loss_cal_pct: f64,
    /// Capacity loss due to cycle aging [%]
    pub q_loss_cyc_pct: f64,
    /// Remaining capacity `Ah`
    pub q_remaining: f64,
    /// Current internal resistance `Ω`
    pub r0_current: f64,
    /// State of health (SoH) — 0 = dead, 1 = new
    pub soh: f64,
}

impl AgingState {
    pub fn new(params: &AgingParams) -> Self {
        Self {
            time_s: 0.0,
            equiv_full_cycles: 0.0,
            q_loss_cal_pct: 0.0,
            q_loss_cyc_pct: 0.0,
            q_remaining: params.q_nom,
            r0_current: params.r0_nom,
            soh: 1.0,
        }
    }
}

/// Battery aging model combining calendar and cycle degradation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AgingModel {
    pub params: AgingParams,
    pub state: AgingState,
    /// Accumulated charge throughput in current half-cycle `Ah`
    charge_throughput: f64,
    /// Previous SoC for DoD tracking
    prev_soc: f64,
    /// Accumulated Ah throughput for this step (for cycle counting)
    ah_total: f64,
}

impl AgingModel {
    pub fn new(params: AgingParams) -> Self {
        let state = AgingState::new(&params);
        Self {
            state,
            charge_throughput: 0.0,
            prev_soc: 1.0,
            ah_total: 0.0,
            params,
        }
    }

    /// Calendar aging increment: Δt seconds at temperature T `K`.
    pub fn step_calendar(&mut self, dt_s: f64, temp_k: f64) {
        let t0 = self.state.time_s;
        let t1 = t0 + dt_s;
        let k_eff = self.params.k_cal * (-self.params.e_a / (GAS_CONSTANT * temp_k)).exp();
        // Incremental SEI growth: d(Q_cal)/dt = k_eff / (2√t)
        // Integrated: Q_cal(t1) - Q_cal(t0) = k_eff * (√t1 - √t0)
        let delta_q = k_eff * (t1.sqrt() - t0.max(1.0).sqrt());
        self.state.q_loss_cal_pct += delta_q * 100.0;
        self.state.time_s = t1;
        self.update_capacity();
    }

    /// Cycle aging increment: register a partial or full cycle with given DoD (0–1).
    pub fn register_cycle(&mut self, dod: f64) {
        let dod_clamped = dod.clamp(0.0, 1.0);
        if dod_clamped < 1e-4 {
            return;
        }
        let delta_q =
            self.params.k_cyc * (dod_clamped / self.params.dod_ref).powf(self.params.dod_exponent);
        self.state.q_loss_cyc_pct += delta_q * 100.0;
        self.state.equiv_full_cycles += dod_clamped;
        self.update_capacity();
    }

    /// Update state when current flows (for automatic cycle counting).
    ///
    /// `current_a` — positive = discharge; `dt_s` — time step `s`.
    pub fn step_current(&mut self, current_a: f64, dt_s: f64, soc: f64) {
        let dah = current_a.abs() * dt_s / 3600.0;
        self.ah_total += dah;
        // Simple half-cycle counting: when SoC reversal detected, register a cycle
        let soc_change = soc - self.prev_soc;
        let prev_dir = (self.prev_soc - soc).signum();
        let cur_dir = soc_change.signum();
        if cur_dir.abs() > 0.5 && prev_dir * cur_dir < 0.0 {
            // Direction reversal: a half-cycle ended
            let dod = self.charge_throughput / self.params.q_nom;
            self.register_cycle(dod);
            self.charge_throughput = 0.0;
        }
        self.charge_throughput += dah;
        self.prev_soc = soc;
    }

    fn update_capacity(&mut self) {
        let total_loss_pct = (self.state.q_loss_cal_pct + self.state.q_loss_cyc_pct).min(100.0);
        self.state.q_remaining = self.params.q_nom * (1.0 - total_loss_pct / 100.0);
        self.state.soh = (self.state.q_remaining / self.params.q_nom).clamp(0.0, 1.0);
        // Resistance grows inversely with SoH
        let capacity_loss_frac = total_loss_pct / 100.0;
        self.state.r0_current =
            self.params.r0_nom * (1.0 + self.params.r_growth_factor * capacity_loss_frac);
    }

    /// Time to 80% SoH at constant temperature `s` (calendar aging only).
    pub fn time_to_80pct_soh(&self, temp_k: f64) -> f64 {
        let k_eff = self.params.k_cal * (-self.params.e_a / (GAS_CONSTANT * temp_k)).exp();
        // Q_loss_cal = 20% → k_eff * √t = 0.20 → t = (0.20/k_eff)²
        if k_eff < 1e-20 {
            return f64::INFINITY;
        }
        (0.20 / k_eff).powi(2)
    }
}

// ── Lithium Plating Model ─────────────────────────────────────────────────────

/// Lithium plating degradation model.
///
/// Lithium plating occurs during fast charging when the anode potential falls
/// below 0 V vs. Li/Li⁺. Plated lithium may be irreversibly lost (dead lithium)
/// or strip back into the electrolyte (partially reversible).
///
/// Capacity loss model:
///   dQ_plating/dt = k_plating · max(0, I_charge - I_threshold)^β
///
/// where I_threshold is the current density above which plating begins.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LithiumPlatingModel {
    /// Nominal cell capacity `Ah`
    pub q_nom: f64,
    /// Threshold C-rate for plating onset (typical: 0.5–1.0 C)
    pub c_rate_threshold: f64,
    /// Plating rate coefficient [Ah per (A·s)]
    pub k_plating: f64,
    /// Current exponent (typical: 1.0–2.0)
    pub beta: f64,
    /// Fraction of plated Li that is irreversible (dead Li): 0–1
    pub irreversible_fraction: f64,
    /// Accumulated plated lithium `Ah` (total, including reversible)
    pub plated_ah: f64,
    /// Accumulated dead lithium capacity loss `Ah`
    pub dead_li_ah: f64,
    /// Low temperature threshold below which plating rate is amplified `K`
    pub t_threshold_k: f64,
    /// Temperature amplification factor at cold temperatures
    pub cold_amplification: f64,
}

impl LithiumPlatingModel {
    /// Typical NMC 21700 cell, susceptible to plating above 1C.
    pub fn nmc_default(q_nom: f64) -> Self {
        Self {
            q_nom,
            c_rate_threshold: 0.5, // 0.5 C onset
            k_plating: 2e-7,       // Ah per A·s
            beta: 1.5,
            irreversible_fraction: 0.20,
            plated_ah: 0.0,
            dead_li_ah: 0.0,
            t_threshold_k: 278.15, // 5°C
            cold_amplification: 3.0,
        }
    }

    /// LFP chemistry — more resistant to plating due to lower anode potential.
    pub fn lfp_default(q_nom: f64) -> Self {
        Self {
            q_nom,
            c_rate_threshold: 1.0,
            k_plating: 5e-8,
            beta: 1.2,
            irreversible_fraction: 0.15,
            plated_ah: 0.0,
            dead_li_ah: 0.0,
            t_threshold_k: 268.15, // -5°C
            cold_amplification: 4.0,
        }
    }

    /// Advance one time step with charging current `i_charge_a` (positive = charging)
    /// and cell temperature `temp_k`.
    ///
    /// Returns the incremental dead-Li capacity loss this step `Ah`.
    pub fn step(&mut self, i_charge_a: f64, dt_s: f64, temp_k: f64) -> f64 {
        if i_charge_a <= 0.0 {
            // Discharging or idle — no new plating; partial strip of reversible Li
            let strip = self.plated_ah * 0.05 * (dt_s / 3600.0); // 5%/h strip rate
            let strip = strip.min(self.plated_ah - self.dead_li_ah);
            if strip > 0.0 {
                self.plated_ah -= strip;
            }
            return 0.0;
        }

        let i_threshold = self.c_rate_threshold * self.q_nom;
        let excess = (i_charge_a - i_threshold).max(0.0);
        if excess < 1e-9 {
            return 0.0;
        }

        // Temperature amplification: plating accelerates at low temperatures
        let temp_factor = if temp_k < self.t_threshold_k {
            self.cold_amplification
        } else {
            1.0
        };

        let dq_plated = self.k_plating * excess.powf(self.beta) * dt_s * temp_factor;
        self.plated_ah += dq_plated;

        let dead = dq_plated * self.irreversible_fraction;
        self.dead_li_ah += dead;
        dead
    }

    /// State of health reduction due to lithium plating (0 = no loss, 1 = complete loss).
    pub fn capacity_loss_fraction(&self) -> f64 {
        (self.dead_li_ah / self.q_nom).clamp(0.0, 1.0)
    }

    /// Remaining capacity accounting for dead lithium `Ah`.
    pub fn remaining_capacity_ah(&self) -> f64 {
        (self.q_nom - self.dead_li_ah).max(0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_cell_full_capacity() {
        let model = AgingModel::new(AgingParams::lfp_default());
        assert!((model.state.soh - 1.0).abs() < 1e-9);
        assert!((model.state.q_remaining - 75.0).abs() < 1e-9);
    }

    #[test]
    fn test_calendar_aging_reduces_capacity() {
        let mut model = AgingModel::new(AgingParams::lfp_default());
        // 1 year at 25°C = 31,536,000 s
        let one_year = 31_536_000.0;
        model.step_calendar(one_year, 298.15);
        assert!(
            model.state.soh < 1.0,
            "SoH should decrease: {}",
            model.state.soh
        );
        assert!(
            model.state.soh > 0.5,
            "SoH should not collapse in 1 year: {}",
            model.state.soh
        );
    }

    #[test]
    fn test_high_temp_accelerates_aging() {
        let p = AgingParams::lfp_default();
        let mut cool = AgingModel::new(p.clone());
        let mut hot = AgingModel::new(p);
        let one_year = 31_536_000.0;
        cool.step_calendar(one_year, 298.15); // 25°C
        hot.step_calendar(one_year, 333.15); // 60°C
        assert!(
            hot.state.q_loss_cal_pct > cool.state.q_loss_cal_pct,
            "hot={:.4} cool={:.4}",
            hot.state.q_loss_cal_pct,
            cool.state.q_loss_cal_pct
        );
    }

    #[test]
    fn test_cycle_aging_reduces_capacity() {
        let mut model = AgingModel::new(AgingParams::lfp_default());
        for _ in 0..100 {
            model.register_cycle(1.0); // 100 full cycles
        }
        assert!(model.state.q_loss_cyc_pct > 0.0);
        assert!(model.state.soh < 1.0);
    }

    #[test]
    fn test_resistance_grows_with_aging() {
        let mut model = AgingModel::new(AgingParams::lfp_default());
        let r_initial = model.state.r0_current;
        for _ in 0..500 {
            model.register_cycle(1.0);
        }
        assert!(
            model.state.r0_current > r_initial,
            "R should grow: {} > {}",
            model.state.r0_current,
            r_initial
        );
    }

    #[test]
    fn test_soh_bounded_between_0_and_1() {
        let mut model = AgingModel::new(AgingParams::lfp_default());
        // Very long calendar age
        model.step_calendar(1e12, 333.15);
        model.register_cycle(1.0);
        assert!(
            model.state.soh >= 0.0 && model.state.soh <= 1.0,
            "SoH={}",
            model.state.soh
        );
    }

    #[test]
    fn test_time_to_80pct_positive() {
        let p = AgingParams::lfp_default();
        let model = AgingModel::new(p);
        let t80 = model.time_to_80pct_soh(298.15);
        assert!(t80 > 0.0 && t80 < 1e15, "t80={:.2e}", t80);
    }

    // ── Lithium Plating Tests ──────────────────────────────────────────────────

    #[test]
    fn test_plating_no_damage_below_threshold() {
        let mut m = LithiumPlatingModel::nmc_default(3.0);
        // Charging at 0.4 C (below 0.5 C threshold): no plating
        let dead = m.step(0.4 * 3.0, 3600.0, 298.15);
        assert_eq!(dead, 0.0);
        assert_eq!(m.plated_ah, 0.0);
    }

    #[test]
    fn test_plating_occurs_above_threshold() {
        let mut m = LithiumPlatingModel::nmc_default(3.0);
        // Charging at 2 C for 1 hour
        let dead = m.step(2.0 * 3.0, 3600.0, 298.15);
        assert!(dead > 0.0, "Dead Li should accumulate at 2C");
        assert!(m.plated_ah > 0.0);
        assert!(m.dead_li_ah > 0.0);
    }

    #[test]
    fn test_plating_amplified_at_low_temperature() {
        let mut warm = LithiumPlatingModel::nmc_default(3.0);
        let mut cold = LithiumPlatingModel::nmc_default(3.0);
        let i = 2.0 * 3.0;
        let warm_dead = warm.step(i, 3600.0, 298.15); // 25°C
        let cold_dead = cold.step(i, 3600.0, 268.15); // -5°C (below threshold)
        assert!(
            cold_dead > warm_dead,
            "Cold plating {cold_dead:.4e} should exceed warm {warm_dead:.4e}"
        );
    }

    #[test]
    fn test_plating_capacity_loss_fraction_bounded() {
        let mut m = LithiumPlatingModel::nmc_default(3.0);
        // Extreme fast charging: 10 C for 10 hours
        for _ in 0..36000 {
            m.step(10.0 * 3.0, 1.0, 253.15);
        }
        let loss = m.capacity_loss_fraction();
        assert!(
            (0.0..=1.0).contains(&loss),
            "Loss fraction {loss:.4} out of [0,1]"
        );
    }

    #[test]
    fn test_remaining_capacity_non_negative() {
        let mut m = LithiumPlatingModel::lfp_default(75.0);
        for _ in 0..100_000 {
            m.step(150.0, 1.0, 258.15); // 2C at low temp
        }
        assert!(m.remaining_capacity_ah() >= 0.0);
    }

    #[test]
    fn test_no_plating_during_discharge() {
        let mut m = LithiumPlatingModel::nmc_default(3.0);
        let dead = m.step(-3.0, 3600.0, 298.15); // discharge
        assert_eq!(dead, 0.0);
        assert_eq!(m.plated_ah, 0.0);
    }

    #[test]
    fn test_plating_irreversible_fraction_correct() {
        let mut m = LithiumPlatingModel::nmc_default(3.0);
        let dead = m.step(6.0, 1.0, 298.15); // 2C for 1 second
                                             // dead_li_ah = plated_ah * irreversible_fraction
        assert!((dead - m.dead_li_ah).abs() < 1e-12);
        assert!((m.dead_li_ah - m.plated_ah * m.irreversible_fraction).abs() < 1e-12);
    }
}
