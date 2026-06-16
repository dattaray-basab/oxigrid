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

    #[test]
    fn test_calendar_aging_rate_increases_with_temperature() {
        let dt = 31_536_000.0_f64; // 1 year in seconds
        let mut m_cold = AgingModel::new(AgingParams::lfp_default());
        let mut m_hot = AgingModel::new(AgingParams::lfp_default());
        m_cold.step_calendar(dt, 288.15); // 15°C
        m_hot.step_calendar(dt, 318.15); // 45°C
        assert!(m_hot.state.q_loss_cal_pct > m_cold.state.q_loss_cal_pct);
    }

    #[test]
    fn test_cycle_aging_increases_with_dod() {
        let mut low_dod = AgingModel::new(AgingParams::lfp_default());
        let mut high_dod = AgingModel::new(AgingParams::lfp_default());
        for _ in 0..100 {
            low_dod.register_cycle(0.2);
        }
        for _ in 0..100 {
            high_dod.register_cycle(0.8);
        }
        assert!(high_dod.state.q_loss_cyc_pct > low_dod.state.q_loss_cyc_pct);
    }

    #[test]
    fn test_sei_layer_growth_sqrt_time() {
        let mut m1 = AgingModel::new(AgingParams::nmc_default());
        let mut m2 = AgingModel::new(AgingParams::nmc_default());
        m1.step_calendar(3_600.0, 298.15);
        m2.step_calendar(14_400.0, 298.15);
        let ratio = m2.state.q_loss_cal_pct / m1.state.q_loss_cal_pct;
        // √(14400)/√(3600) = 120/60 = 2.0; the model starts at t=max(t0,1) so ratio
        // converges to 2.0 at large times but differs slightly at small t0 — allow 5%
        assert!((ratio - 2.0).abs() < 0.05, "ratio={:.4}", ratio);
    }

    #[test]
    fn test_capacity_fade_at_1000_cycles() {
        let mut m = AgingModel::new(AgingParams::lfp_default());
        for _ in 0..1000 {
            m.register_cycle(1.0);
        }
        let expected = 1000.0 * 0.0025 * 100.0;
        assert!((m.state.q_loss_cyc_pct - expected).abs() < 1e-6);
        assert!(m.state.q_remaining < 75.0);
    }

    #[test]
    fn test_resistance_increase_over_lifetime() {
        let params = AgingParams::nmc_default();
        let r0_nom = params.r0_nom;
        let r_growth_factor = params.r_growth_factor;
        let mut m = AgingModel::new(params);
        for _ in 0..200 {
            m.register_cycle(1.0);
        }
        assert!(m.state.r0_current > r0_nom);
        let total_loss_frac = m.state.q_loss_cyc_pct / 100.0;
        let expected_r0 = r0_nom * (1.0 + r_growth_factor * total_loss_frac);
        assert!((m.state.r0_current - expected_r0).abs() < 1e-9);
    }

    #[test]
    fn test_rul_at_80pct_soh() {
        let m = AgingModel::new(AgingParams::lfp_default());
        let t80 = m.time_to_80pct_soh(298.15);
        // Verify the formula directly: at t=t80, calendar loss should equal ~20%
        // k_eff * sqrt(t80) = 0.20  →  q_loss_cal = k_eff * sqrt(t80) = 0.20
        // Use a 45°C model to get a short enough t80 to cross-check numerically
        let m_hot = AgingModel::new(AgingParams::nmc_default()); // NMC: higher k_cal
        let t80_hot = m_hot.time_to_80pct_soh(333.15); // 60°C
        assert!(t80_hot > 0.0, "t80 should be positive: {:.3e}", t80_hot);
        assert!(
            t80 > t80_hot,
            "LFP@25°C should last longer than NMC@60°C: {:.3e} vs {:.3e}",
            t80,
            t80_hot
        );
        // Simulate to cross t80_hot and confirm SoH ≤ 0.80 at that point
        let dt = t80_hot / 1000.0;
        let mut m2 = AgingModel::new(AgingParams::nmc_default());
        for _ in 0..1100 {
            m2.step_calendar(dt, 333.15);
        }
        assert!(
            m2.state.soh <= 0.80,
            "After simulating past t80 SoH should be ≤ 0.80: {:.6}",
            m2.state.soh
        );
    }

    #[test]
    fn test_combined_calendar_and_cycle_aging() {
        let dt_year = 31_536_000.0_f64;
        let mut m_cal = AgingModel::new(AgingParams::lfp_default());
        let mut m_cyc = AgingModel::new(AgingParams::lfp_default());
        let mut m_both = AgingModel::new(AgingParams::lfp_default());
        m_cal.step_calendar(dt_year, 298.15);
        for _ in 0..100 {
            m_cyc.register_cycle(1.0);
        }
        m_both.step_calendar(dt_year, 298.15);
        for _ in 0..100 {
            m_both.register_cycle(1.0);
        }
        let q_loss_cal_alone = m_cal.state.q_loss_cal_pct;
        let q_loss_cyc_alone = m_cyc.state.q_loss_cyc_pct;
        assert!(m_both.state.soh < 1.0 - q_loss_cal_alone / 100.0);
        assert!(m_both.state.soh < 1.0 - q_loss_cyc_alone / 100.0);
    }

    #[test]
    fn test_aging_acceleration_factor() {
        let m25 = AgingModel::new(AgingParams::lfp_default());
        let m35 = AgingModel::new(AgingParams::lfp_default());
        let t80_25 = m25.time_to_80pct_soh(298.15);
        let t80_35 = m35.time_to_80pct_soh(308.15);
        let ratio = t80_25 / t80_35;
        assert!(ratio > 1.0);
        assert!(ratio > 1.5, "ratio={:.4}", ratio);
    }

    #[test]
    fn test_step_current_cycle_detection() {
        let mut model = AgingModel::new(AgingParams::lfp_default());
        // Discharge: soc drops 1.0 → 0.9, builds charge_throughput
        // First call triggers reversal at t=0 but charge_throughput=0 → no cycle counted
        model.step_current(10.0, 3600.0, 0.9);
        let cycles_before = model.state.equiv_full_cycles;
        // Direction reversal: now charging (soc increases)
        model.step_current(-5.0, 3600.0, 0.95);
        assert!(
            model.state.equiv_full_cycles > cycles_before,
            "equiv_full_cycles should increase on reversal: before={:.4} after={:.4}",
            cycles_before,
            model.state.equiv_full_cycles
        );
    }

    #[test]
    fn test_step_current_no_reversal_no_cycle() {
        let mut model = AgingModel::new(AgingParams::lfp_default());
        // First call: soc=0.5 (from prev_soc=1.0, this causes a direction change at start)
        model.step_current(5.0, 360.0, 0.5);
        // Subsequent calls with same soc: soc_change=0, cur_dir=0 → no reversal
        let cycles_after_first = model.state.equiv_full_cycles;
        model.step_current(5.0, 360.0, 0.5);
        model.step_current(5.0, 360.0, 0.5);
        model.step_current(5.0, 360.0, 0.5);
        assert_eq!(
            model.state.equiv_full_cycles, cycles_after_first,
            "No reversal means no new cycles: equiv_full_cycles={}",
            model.state.equiv_full_cycles
        );
    }

    #[test]
    fn test_aging_state_new_matches_params() {
        let params = AgingParams::lfp_default();
        let q_nom = params.q_nom;
        let r0_nom = params.r0_nom;
        let state = AgingState::new(&params);
        assert_eq!(state.time_s, 0.0, "time_s should be zero");
        assert_eq!(
            state.equiv_full_cycles, 0.0,
            "equiv_full_cycles should be zero"
        );
        assert_eq!(state.q_loss_cal_pct, 0.0, "q_loss_cal_pct should be zero");
        assert_eq!(state.q_loss_cyc_pct, 0.0, "q_loss_cyc_pct should be zero");
        assert!(
            (state.q_remaining - q_nom).abs() < 1e-12,
            "q_remaining={} q_nom={}",
            state.q_remaining,
            q_nom
        );
        assert!(
            (state.r0_current - r0_nom).abs() < 1e-12,
            "r0_current={} r0_nom={}",
            state.r0_current,
            r0_nom
        );
        assert!(
            (state.soh - 1.0).abs() < 1e-12,
            "soh should be 1.0, got {}",
            state.soh
        );
    }

    #[test]
    fn test_nmc_default_params_reasonable() {
        let p = AgingParams::nmc_default();
        assert!(p.k_cal > 0.0, "k_cal should be positive: {}", p.k_cal);
        assert!(p.e_a > 20_000.0, "e_a should be > 20000 J/mol: {}", p.e_a);
        assert!(p.k_cyc > 0.0, "k_cyc should be positive: {}", p.k_cyc);
        assert!(
            p.dod_exponent > 0.0,
            "dod_exponent should be positive: {}",
            p.dod_exponent
        );
        assert!(p.q_nom > 0.0, "q_nom should be positive: {}", p.q_nom);
        assert!(p.r0_nom > 0.0, "r0_nom should be positive: {}", p.r0_nom);
    }

    #[test]
    fn test_small_dod_ignored_in_register_cycle() {
        let mut model = AgingModel::new(AgingParams::lfp_default());
        let cycles_before = model.state.equiv_full_cycles;
        let q_remaining_before = model.state.q_remaining;
        // DoD below 1e-4 threshold should be ignored
        model.register_cycle(1e-6);
        model.register_cycle(1e-5);
        assert_eq!(
            model.state.equiv_full_cycles, cycles_before,
            "equiv_full_cycles should not change for sub-threshold DoD"
        );
        assert!(
            (model.state.q_remaining - q_remaining_before).abs() < 1e-12,
            "q_remaining should not change for sub-threshold DoD"
        );
    }

    #[test]
    fn test_plating_strip_during_discharge() {
        let mut m = LithiumPlatingModel::nmc_default(3.0);
        // Build up plated_ah by fast charging
        m.step(6.0, 3600.0, 298.15); // 2C for 1 hour
        let plated_before = m.plated_ah;
        assert!(
            plated_before > 0.0,
            "Should have plated Li: plated_ah={:.6e}",
            plated_before
        );
        // Now discharge to strip reversible Li
        m.step(-3.0, 3600.0, 298.15); // 1C discharge for 1 hour
        assert!(
            m.plated_ah < plated_before,
            "Stripping should reduce plated_ah: before={:.6e} after={:.6e}",
            plated_before,
            m.plated_ah
        );
    }

    #[test]
    fn test_lfp_plating_model_defaults_reasonable() {
        let m = LithiumPlatingModel::lfp_default(75.0);
        assert!(
            (m.c_rate_threshold - 1.0).abs() < 1e-9,
            "LFP c_rate_threshold should be 1.0: {}",
            m.c_rate_threshold
        );
        assert!(
            (m.irreversible_fraction - 0.15).abs() < 1e-9,
            "LFP irreversible_fraction should be 0.15: {}",
            m.irreversible_fraction
        );
        assert!(
            (m.cold_amplification - 4.0).abs() < 1e-9,
            "LFP cold_amplification should be 4.0: {}",
            m.cold_amplification
        );
    }

    #[test]
    fn test_time_to_80pct_very_cold() {
        let model = AgingModel::new(AgingParams::lfp_default());
        // At -20°C (253.15 K), k_eff is extremely small → t80 should be astronomically large
        let t80 = model.time_to_80pct_soh(253.15);
        assert!(
            t80 > 1e14,
            "t80 at -20°C should be astronomically large: {:.4e}",
            t80
        );
    }
}
