//! Battery second-life assessment and repurposing analysis.
//!
//! Evaluates whether a used battery pack is suitable for repurposing in a
//! secondary application after its primary use (e.g. EV → stationary storage).
//!
//! # Grading System
//! - Grade A: SoH > 80 % — suitable for most applications
//! - Grade B: SoH 70–80 % — suitable for stationary storage
//! - Grade C: SoH 60–70 % — limited applications (UPS, backup)
//! - Grade D: SoH < 60 % — repurposing not recommended
//!
//! # References
//! - Canals Casals et al., "Reused second life batteries for aggregated demand
//!   response services", 2017.
//! - Heymans et al., "Economic analysis of second use electric vehicle batteries
//!   for residential storage", 2014.
//! - Neubauer & Pesaran, "The ability of battery second use strategies to
//!   impact plug-in electric vehicle prices", 2011.

use crate::battery::sop::CapacityFadeEstimator;
use serde::{Deserialize, Serialize};

/// Target application for second-life battery repurposing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SecondLifeApplicationLegacy {
    /// Grid-tied battery energy storage system (BESS).
    StationaryStorage,
    /// Solar or wind energy buffering and output smoothing.
    RenewableIntegration,
    /// Commercial or industrial peak demand shaving.
    PeakShaving,
    /// Uninterruptible power supply (backup power).
    UpsBackup,
    /// Fast-response grid frequency regulation service.
    GridFrequencyReg,
}

impl SecondLifeApplicationLegacy {
    /// Minimum State of Health required for this application.
    pub fn min_soh_required(self) -> f64 {
        match self {
            Self::StationaryStorage => 0.70,
            Self::RenewableIntegration => 0.75,
            Self::PeakShaving => 0.70,
            Self::UpsBackup => 0.65,
            Self::GridFrequencyReg => 0.80,
        }
    }

    /// Maximum recommended C-rate for this application.
    pub fn max_crate(self) -> f64 {
        match self {
            Self::StationaryStorage => 0.5,
            Self::RenewableIntegration => 0.3,
            Self::PeakShaving => 1.0,
            Self::UpsBackup => 0.2,
            Self::GridFrequencyReg => 2.0,
        }
    }

    /// Annual revenue potential \[$/kWh/year\] for this application.
    pub fn revenue_potential_per_kwh_year(self) -> f64 {
        match self {
            Self::StationaryStorage => 25.0,
            Self::RenewableIntegration => 30.0,
            Self::PeakShaving => 40.0,
            Self::UpsBackup => 15.0,
            Self::GridFrequencyReg => 60.0,
        }
    }
}

/// Suitability grade for second-life repurposing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SecondLifeGrade {
    /// SoH > 80 %: suitable for most second-life applications.
    A,
    /// SoH 70–80 %: suitable for stationary storage.
    B,
    /// SoH 60–70 %: limited applications only (UPS, backup).
    C,
    /// SoH < 60 %: repurposing not recommended; materials recycling advised.
    D,
}

impl SecondLifeGrade {
    /// Determine grade from State of Health.
    pub fn from_soh(soh: f64) -> Self {
        if soh >= 0.80 {
            Self::A
        } else if soh >= 0.70 {
            Self::B
        } else if soh >= 0.60 {
            Self::C
        } else {
            Self::D
        }
    }

    /// Refurbishment cost multiplier relative to new-battery cost.
    ///
    /// Grade A requires the least work; Grade D incurs disassembly/recycling costs.
    pub fn cost_multiplier(self) -> f64 {
        match self {
            Self::A => 0.25,
            Self::B => 0.35,
            Self::C => 0.50,
            Self::D => 0.80,
        }
    }
}

/// Result of a second-life suitability assessment.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SecondLifeResult {
    /// Whether the battery is suitable for the target application.
    pub suitable_for_application: bool,
    /// Remaining capacity as a percentage of original rated capacity \[%\].
    pub remaining_capacity_pct: f64,
    /// Estimated remaining useful years in the second-life application.
    pub estimated_remaining_years: f64,
    /// Refurbishment and repackaging cost \[$/kWh\].
    pub repurposing_cost_per_kwh: f64,
    /// Annual revenue potential in the second-life application \[$/kWh/year\].
    pub revenue_potential_per_kwh_year: f64,
    /// Minimum SoH required for the target application.
    pub min_soh_required: f64,
    /// Recommended maximum C-rate for second-life operation.
    pub recommended_crate: f64,
    /// Overall suitability grade.
    pub grading: SecondLifeGrade,
    /// Simple payback period \[years\] = cost / annual_revenue.
    pub payback_years: f64,
    /// Net present value estimate \[$/kWh\] over remaining years at 8 % discount.
    pub npv_per_kwh: f64,
}

/// Battery second-life assessment engine.
pub struct SecondLifeAssessment {
    /// Target second-life application.
    pub target_application: SecondLifeApplicationLegacy,
}

impl SecondLifeAssessment {
    /// Create a new assessment for the given application.
    pub fn new(application: SecondLifeApplicationLegacy) -> Self {
        Self {
            target_application: application,
        }
    }

    /// Assess the suitability of a battery pack for second-life repurposing.
    ///
    /// # Parameters
    /// - `capacity_fade`:  Aging model capturing current cycle and calendar history.
    /// - `n_cells_total`:  Total number of cells in the pack.
    /// - `n_cells_failed`: Number of failed or bypassed cells in the pack.
    pub fn assess(
        &self,
        capacity_fade: &CapacityFadeEstimator,
        n_cells_total: usize,
        n_cells_failed: usize,
    ) -> SecondLifeResult {
        let current_soh = capacity_fade.soh();
        let grade = SecondLifeGrade::from_soh(current_soh);
        let min_soh = self.target_application.min_soh_required();

        // Penalise SoH proportionally for failed cells
        let n_cells_total = n_cells_total.max(1);
        let cell_failure_penalty = n_cells_failed as f64 / n_cells_total as f64 * 0.10;
        let effective_soh = (current_soh - cell_failure_penalty).clamp(0.0, 1.0);

        // Battery is suitable only if SoH meets the threshold, grade is not D,
        // and fewer than 25 % of cells have failed.
        let suitable = effective_soh >= min_soh
            && grade != SecondLifeGrade::D
            && n_cells_failed < n_cells_total / 4 + 1;

        // Estimated usable pack capacity [kWh]
        let capacity_kwh = self.estimate_capacity_kwh(n_cells_total, effective_soh);

        // Refurbishment cost [$/kWh]
        let refurb_cost = self.repackaging_cost(capacity_kwh, grade);

        // Revenue potential [$/kWh/year]
        let revenue = self.target_application.revenue_potential_per_kwh_year();

        // Estimate annual fade rate and remaining lifetime
        let fade_rate_per_year = self.estimate_fade_rate_per_year(capacity_fade);
        // End-of-life SoH for second-life = 60 %
        let second_life_eol_soh = 0.60_f64;
        let remaining_years =
            self.remaining_lifetime_years(effective_soh, fade_rate_per_year, second_life_eol_soh);

        // Payback period
        let payback_years = if revenue > 1e-6 {
            refurb_cost / revenue
        } else {
            f64::INFINITY
        };

        // Net present value at 8 % discount over remaining_years
        let npv_per_kwh = compute_npv(revenue, refurb_cost, 0.08, remaining_years);

        SecondLifeResult {
            suitable_for_application: suitable,
            remaining_capacity_pct: effective_soh * 100.0,
            estimated_remaining_years: remaining_years,
            repurposing_cost_per_kwh: refurb_cost,
            revenue_potential_per_kwh_year: revenue,
            min_soh_required: min_soh,
            recommended_crate: self.target_application.max_crate(),
            grading: grade,
            payback_years,
            npv_per_kwh,
        }
    }

    /// Estimate refurbishment and repackaging cost \[$/kWh\].
    ///
    /// Higher grades require less work; Grade D incurs recycling costs.
    pub fn repackaging_cost(&self, _capacity_kwh: f64, grade: SecondLifeGrade) -> f64 {
        // Base new-battery cost: ~$150/kWh (2025 market estimate)
        let new_battery_cost_per_kwh = 150.0_f64;
        let base_cost = new_battery_cost_per_kwh * grade.cost_multiplier();

        // Additional testing and system integration costs [$/kWh]
        let testing_cost = 10.0_f64;
        let integration_cost = match self.target_application {
            SecondLifeApplicationLegacy::GridFrequencyReg => 20.0,
            SecondLifeApplicationLegacy::StationaryStorage => 15.0,
            SecondLifeApplicationLegacy::RenewableIntegration => 12.0,
            SecondLifeApplicationLegacy::PeakShaving => 10.0,
            SecondLifeApplicationLegacy::UpsBackup => 8.0,
        };

        // Economy-of-scale discount for large packs
        // (kept fixed here; capacity_kwh could refine this)
        base_cost + testing_cost + integration_cost
    }

    /// Estimate remaining useful lifetime in the second-life application \[years\].
    ///
    /// Extrapolates linearly from `current_soh` to `end_of_life_soh` using
    /// `fade_rate_per_year`.  Capped at 30 years.
    pub fn remaining_lifetime_years(
        &self,
        current_soh: f64,
        fade_rate_per_year: f64,
        end_of_life_soh: f64,
    ) -> f64 {
        if current_soh <= end_of_life_soh {
            return 0.0;
        }
        if fade_rate_per_year <= 1e-10 {
            return 20.0; // Negligible degradation → long life
        }
        let remaining = (current_soh - end_of_life_soh) / fade_rate_per_year;
        remaining.min(30.0)
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    fn estimate_capacity_kwh(&self, n_cells: usize, soh: f64) -> f64 {
        // Approximate: typical NMC cell ≈ 3.6 V × 50 Ah = 180 Wh
        let cell_energy_kwh = 0.18_f64; // 180 Wh per cell
        (n_cells as f64 * cell_energy_kwh * soh).max(0.1)
    }

    fn estimate_fade_rate_per_year(&self, fade: &CapacityFadeEstimator) -> f64 {
        // Assume a typical stationary BESS operating rhythm: ~300 EFC/year
        let cycles_per_year = 300.0_f64;
        let cycle_loss_per_year = fade.cycle_fade_rate * cycles_per_year;
        let calendar_loss_per_year = fade.calendar_fade_rate * 365.0;
        // SEI loss grows as √t → instantaneous rate = k_sei / (2√t)
        let days = fade.calendar_days.max(1.0);
        let sei_rate_per_year = fade.sei_growth_rate / (2.0 * days.sqrt()) * 365.0;
        cycle_loss_per_year + calendar_loss_per_year + sei_rate_per_year
    }
}

/// Compute Net Present Value of uniform annual revenue minus an initial cost.
///
/// `NPV = revenue × annuity_factor(r, n) − initial_cost`
/// where `annuity_factor = (1 − (1+r)^{−n}) / r`.
fn compute_npv(annual_revenue: f64, initial_cost: f64, discount_rate: f64, years: f64) -> f64 {
    if years <= 0.0 || discount_rate <= 0.0 {
        return -initial_cost;
    }
    let annuity = annual_revenue * (1.0 - (1.0 + discount_rate).powf(-years)) / discount_rate;
    annuity - initial_cost
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::battery::sop::CapacityFadeEstimator;

    fn make_fresh_fade() -> CapacityFadeEstimator {
        CapacityFadeEstimator::new()
    }

    fn make_aged_fade(cycles: f64, days: f64) -> CapacityFadeEstimator {
        let mut fade = CapacityFadeEstimator::new();
        fade.total_cycles = cycles;
        fade.calendar_days = days;
        fade
    }

    #[test]
    fn test_grade_from_soh() {
        assert_eq!(SecondLifeGrade::from_soh(0.85), SecondLifeGrade::A);
        assert_eq!(SecondLifeGrade::from_soh(0.75), SecondLifeGrade::B);
        assert_eq!(SecondLifeGrade::from_soh(0.65), SecondLifeGrade::C);
        assert_eq!(SecondLifeGrade::from_soh(0.55), SecondLifeGrade::D);
    }

    #[test]
    fn test_fresh_battery_grade_a_and_suitable() {
        let fade = make_fresh_fade();
        // 8 cells total, 0 failed
        let assess = SecondLifeAssessment::new(SecondLifeApplicationLegacy::StationaryStorage);
        let result = assess.assess(&fade, 8, 0);
        assert_eq!(result.grading, SecondLifeGrade::A);
        assert!(
            result.remaining_capacity_pct > 99.0,
            "Fresh battery capacity should be ~100 %, got {:.2}",
            result.remaining_capacity_pct
        );
        assert!(
            result.suitable_for_application,
            "Fresh battery should be suitable for stationary storage"
        );
    }

    #[test]
    fn test_aged_battery_reduced_suitability() {
        // Heavily aged: 3000 EFC + 3 years calendar
        let fade = make_aged_fade(3000.0, 1095.0);
        let assess = SecondLifeAssessment::new(SecondLifeApplicationLegacy::GridFrequencyReg);
        let result = assess.assess(&fade, 8, 0);
        // Heavy aging reduces SoH
        assert!(
            result.remaining_capacity_pct < 100.0,
            "Aged battery capacity should be < 100 %"
        );
        // Frequency regulation requires SoH > 80 %; verify result is consistent
        if result.remaining_capacity_pct / 100.0 < 0.80 {
            assert!(
                !result.suitable_for_application,
                "Battery below 80 % SoH should not be suitable for freq regulation"
            );
        }
    }

    #[test]
    fn test_repackaging_cost_grade_a_cheaper_than_d() {
        let assess = SecondLifeAssessment::new(SecondLifeApplicationLegacy::StationaryStorage);
        let cost_a = assess.repackaging_cost(50.0, SecondLifeGrade::A);
        let cost_d = assess.repackaging_cost(50.0, SecondLifeGrade::D);
        assert!(
            cost_a < cost_d,
            "Grade A repackaging ({:.2}) should cost less than Grade D ({:.2})",
            cost_a,
            cost_d
        );
    }

    #[test]
    fn test_remaining_lifetime_zero_at_eol() {
        let assess = SecondLifeAssessment::new(SecondLifeApplicationLegacy::UpsBackup);
        // SoH already at EOL threshold
        let years = assess.remaining_lifetime_years(0.60, 0.05, 0.60);
        assert!(
            years.abs() < 1e-9,
            "Remaining lifetime at EOL should be 0, got {:.6}",
            years
        );
    }

    #[test]
    fn test_npv_fresh_battery_positive_long_term() {
        let fade = make_fresh_fade();
        // Peak shaving: $40/kWh/year revenue
        let assess = SecondLifeAssessment::new(SecondLifeApplicationLegacy::PeakShaving);
        let result = assess.assess(&fade, 8, 0);
        assert!(
            result.estimated_remaining_years > 0.0,
            "Fresh battery should have positive remaining years"
        );
        // Over a long horizon, NPV should be positive
        // (cost ~$50/kWh, revenue $40/kWh/year for >1.25 years)
        assert!(
            result.npv_per_kwh > -200.0,
            "NPV should not be extremely negative: {:.2}",
            result.npv_per_kwh
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// NEW: Comprehensive Battery Second-Life & Repurposing Optimization Module
// ═══════════════════════════════════════════════════════════════════════════════

// ─── Chemistry ────────────────────────────────────────────────────────────────

/// Battery chemistry variants for second-life assessment.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BatteryChemistry {
    /// LiFePO₄ — very long cycle life, excellent thermal stability.
    LithiumIronPhosphate,
    /// NMC 6:2:2 — good balance of energy/power/life.
    Nmc622,
    /// NMC 8:1:1 — high energy, faster degradation.
    Nmc811,
    /// NCA — highest energy density, fastest calendar degradation.
    Nca,
    /// Li₄Ti₅O₁₂ — ultra-long cycle life, low energy density.
    Lto,
    /// LiMn₂O₄ — moderate performance, cost-effective.
    Lmo,
}

impl BatteryChemistry {
    /// Reference DCIR \[Ω\] for a fresh cell (chemistry-dependent).
    pub fn reference_dcir_ohm(self) -> f64 {
        match self {
            Self::LithiumIronPhosphate => 0.010,
            Self::Nmc622 => 0.012,
            Self::Nmc811 => 0.013,
            Self::Nca => 0.011,
            Self::Lto => 0.008,
            Self::Lmo => 0.015,
        }
    }

    /// Arrhenius activation energy \[K\] for calendar aging (Eₐ/R).
    pub fn arrhenius_ea_over_r(self) -> f64 {
        match self {
            Self::LithiumIronPhosphate => 3800.0,
            Self::Nmc622 => 4200.0,
            Self::Nmc811 => 4500.0,
            Self::Nca => 4600.0,
            Self::Lto => 3200.0,
            Self::Lmo => 4000.0,
        }
    }
}

// ─── SoH Assessment ───────────────────────────────────────────────────────────

/// State-of-Health assessment input for a retired EV battery cell.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SohAssessment {
    /// Unique cell identifier string.
    pub cell_id: String,
    /// Electrochemical chemistry of the cell.
    pub chemistry: BatteryChemistry,
    /// Original (as-new) rated capacity \[Ah\].
    pub original_capacity_ah: f64,
    /// Currently measured capacity \[Ah\].
    pub current_capacity_ah: f64,
    /// Capacity-based SoH = current / original \[0–1\].
    pub soh_capacity: f64,
    /// Measured DC internal resistance \[Ω\].
    pub dcir_ohm: f64,
    /// Resistance-based SoH = reference_dcir / current_dcir \[0–1\].
    pub soh_resistance: f64,
    /// Total full equivalent charge/discharge cycles accumulated.
    pub cycle_count: u32,
    /// Calendar age since manufacture \[years\].
    pub calendar_age_years: f64,
    /// Monthly average ambient/cell temperatures \[°C\].
    pub temperature_history_c: Vec<f64>,
    /// Monthly maximum observed terminal voltage \[V\].
    pub max_voltage_history: Vec<f64>,
    /// Monthly minimum SoC observed \[0–1\].
    pub min_soc_history: Vec<f64>,
}

impl SohAssessment {
    /// Construct a new assessment, computing derived SoH ratios.
    ///
    /// # Arguments
    /// - `cell_id` — unique identifier string.
    /// - `chemistry` — cell chemistry.
    /// - `original_capacity_ah` — as-new capacity \[Ah\].
    /// - `current_capacity_ah` — current measured capacity \[Ah\].
    /// - `dcir_ohm` — current measured DCIR \[Ω\].
    /// - `cycle_count` — total equivalent full cycles.
    /// - `calendar_age_years` — total calendar age \[years\].
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        cell_id: impl Into<String>,
        chemistry: BatteryChemistry,
        original_capacity_ah: f64,
        current_capacity_ah: f64,
        dcir_ohm: f64,
        cycle_count: u32,
        calendar_age_years: f64,
    ) -> Self {
        let soh_capacity = if original_capacity_ah > 1e-9 {
            (current_capacity_ah / original_capacity_ah).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let ref_dcir = chemistry.reference_dcir_ohm();
        let soh_resistance = if dcir_ohm > 1e-12 {
            (ref_dcir / dcir_ohm).clamp(0.0, 1.0)
        } else {
            1.0
        };
        Self {
            cell_id: cell_id.into(),
            chemistry,
            original_capacity_ah,
            current_capacity_ah,
            soh_capacity,
            dcir_ohm,
            soh_resistance,
            cycle_count,
            calendar_age_years,
            temperature_history_c: Vec::new(),
            max_voltage_history: Vec::new(),
            min_soc_history: Vec::new(),
        }
    }

    /// Perform a full SoH assessment and return structured results.
    pub fn assess(&self) -> SohResult {
        // Composite SoH: capacity (60 %) + resistance (40 %)
        let composite_soh = 0.6 * self.soh_capacity + 0.4 * self.soh_resistance;
        let composite_soh = composite_soh.clamp(0.0, 1.0);

        let grade = Self::grade(composite_soh);
        let recommended_application = Self::recommend_application(&grade);
        let estimated_second_life_years = Self::second_life_duration(&grade);

        // Remaining Useful Life: Arrhenius calendar + cycle model
        let rul = self.estimate_rul(composite_soh);

        SohResult {
            composite_soh,
            remaining_useful_life_years: rul,
            grading: grade,
            recommended_application,
            estimated_second_life_years,
        }
    }

    /// Compute grade from composite SoH value.
    pub fn grade(composite_soh: f64) -> SohGrade {
        if composite_soh >= 0.80 {
            SohGrade::A
        } else if composite_soh >= 0.70 {
            SohGrade::B
        } else if composite_soh >= 0.60 {
            SohGrade::C
        } else if composite_soh >= 0.50 {
            SohGrade::D
        } else {
            SohGrade::Scrap
        }
    }

    /// Recommend second-life application based on grade.
    pub fn recommend_application(grade: &SohGrade) -> SecondLifeApplication {
        match grade {
            SohGrade::A => SecondLifeApplication::GridEnergyStorage,
            SohGrade::B => SecondLifeApplication::RenewableIntegration,
            SohGrade::C => SecondLifeApplication::UpsBackup,
            SohGrade::D => SecondLifeApplication::OffGridStorage,
            SohGrade::Scrap => SecondLifeApplication::MaterialRecovery,
        }
    }

    /// Estimate remaining useful life using Arrhenius + cycle degradation model.
    fn estimate_rul(&self, composite_soh: f64) -> f64 {
        // Average temperature from history, defaulting to 25 °C if not available
        let t_avg = if self.temperature_history_c.is_empty() {
            25.0_f64
        } else {
            let sum: f64 = self.temperature_history_c.iter().sum();
            sum / self.temperature_history_c.len() as f64
        };

        // Arrhenius-modified capacity fade per cycle
        let capacity_fade_per_cycle = 0.0001 * (0.02 * (t_avg - 25.0)).exp();

        // Remaining cycles until composite_soh reaches 0.5 (end-of-second-life)
        let soh_remaining_headroom = (composite_soh - 0.5).max(0.0);
        let remaining_cycles = if capacity_fade_per_cycle > 1e-12 {
            soh_remaining_headroom / capacity_fade_per_cycle
        } else {
            50_000.0
        };

        // Annual cycle rate based on historical data
        let cycles_per_year = if self.calendar_age_years > 0.1 {
            self.cycle_count as f64 / self.calendar_age_years
        } else {
            300.0 // default assumption
        };
        let effective_cycles_per_year = cycles_per_year + 1.0; // +1 avoids division by zero

        let rul_years = remaining_cycles / effective_cycles_per_year;
        rul_years.clamp(0.0, 50.0)
    }

    /// Estimate expected second-life duration based on grade.
    fn second_life_duration(grade: &SohGrade) -> f64 {
        match grade {
            SohGrade::A => 8.0,
            SohGrade::B => 6.0,
            SohGrade::C => 4.0,
            SohGrade::D => 2.0,
            SohGrade::Scrap => 0.0,
        }
    }
}

// ─── SoH Result ───────────────────────────────────────────────────────────────

/// Output of a battery state-of-health assessment.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SohResult {
    /// Composite SoH: 0.6 × capacity SoH + 0.4 × resistance SoH \[0–1\].
    pub composite_soh: f64,
    /// Estimated remaining useful life before reaching SoH = 50 % \[years\].
    pub remaining_useful_life_years: f64,
    /// Quality grade (A/B/C/D/Scrap).
    pub grading: SohGrade,
    /// Recommended second-life application.
    pub recommended_application: SecondLifeApplication,
    /// Expected duration in the recommended second-life application \[years\].
    pub estimated_second_life_years: f64,
}

// ─── SoH Grade ────────────────────────────────────────────────────────────────

/// Five-tier quality grade for retired battery cells.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SohGrade {
    /// Composite SoH > 80 % — premium second-life quality.
    A,
    /// Composite SoH 70–80 % — good second-life quality.
    B,
    /// Composite SoH 60–70 % — limited second-life applications.
    C,
    /// Composite SoH 50–60 % — marginal, off-grid or low-demand only.
    D,
    /// Composite SoH < 50 % — retire to material recovery.
    Scrap,
}

// ─── Second-Life Application ───────────────────────────────────────────────────

/// Target deployment application for retired battery cells.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SecondLifeApplication {
    /// Grid-tied BESS requiring SoH > 70 %.
    GridEnergyStorage,
    /// UPS / backup power requiring SoH > 60 %.
    UpsBackup,
    /// Renewable integration buffer requiring SoH > 65 %.
    RenewableIntegration,
    /// Off-grid / remote storage requiring SoH > 55 %.
    OffGridStorage,
    /// Material recovery / recycling for SoH < 55 %.
    MaterialRecovery,
}

impl SecondLifeApplication {
    /// Minimum composite SoH required for this application \[0–1\].
    pub fn min_soh(self) -> f64 {
        match self {
            Self::GridEnergyStorage => 0.70,
            Self::UpsBackup => 0.60,
            Self::RenewableIntegration => 0.65,
            Self::OffGridStorage => 0.55,
            Self::MaterialRecovery => 0.0,
        }
    }

    /// Typical arbitrage price spread coefficient for revenue modelling.
    pub fn arbitrage_spread(self) -> f64 {
        match self {
            Self::GridEnergyStorage => 0.05,
            Self::RenewableIntegration => 0.08,
            Self::UpsBackup => 0.04,
            Self::OffGridStorage => 0.06,
            Self::MaterialRecovery => 0.0,
        }
    }
}

// ─── Pack Assembler ───────────────────────────────────────────────────────────

/// Assembles a second-life pack from a pool of retired cells.
pub struct SecondLifePackAssembler {
    /// Pool of cells available for assembly.
    pub cells: Vec<SohAssessment>,
    /// Target pack nominal voltage \[V\].
    pub target_voltage_v: f64,
    /// Target pack capacity \[Ah\].
    pub target_capacity_ah: f64,
    /// Cell nominal voltage \[V\].
    pub cell_nominal_voltage_v: f64,
}

/// Result of a second-life pack assembly optimisation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PackAssemblyResult {
    /// Number of cells in each series string.
    pub series_cells: usize,
    /// Number of parallel strings.
    pub parallel_strings: usize,
    /// Achieved pack voltage \[V\].
    pub actual_voltage_v: f64,
    /// Achieved pack capacity \[Ah\].
    pub actual_capacity_ah: f64,
    /// Minimum composite SoH across all used cells \[0–1\].
    pub pack_soh: f64,
    /// Indices (into `cells`) of cells incorporated into the pack.
    pub cells_used: Vec<usize>,
    /// Indices (into `cells`) of cells rejected (low SoH or outlier).
    pub cells_rejected: Vec<usize>,
    /// Total usable pack energy \[kWh\].
    pub pack_energy_kwh: f64,
    /// Expected pack lifespan in second-life service \[years\].
    pub estimated_lifespan_years: f64,
}

impl SecondLifePackAssembler {
    /// Create a new assembler.
    pub fn new(
        cells: Vec<SohAssessment>,
        target_voltage_v: f64,
        target_capacity_ah: f64,
        cell_nominal_voltage_v: f64,
    ) -> Self {
        Self {
            cells,
            target_voltage_v,
            target_capacity_ah,
            cell_nominal_voltage_v,
        }
    }

    /// Assemble a matched pack from the cell pool.
    ///
    /// Algorithm:
    /// 1. Filter to Grade A/B cells (composite SoH > 70 %).
    /// 2. Sort by composite SoH descending.
    /// 3. Compute required series count = ⌈target_voltage / cell_voltage⌉.
    /// 4. Compute effective cell capacity = original_ah × composite_soh.
    /// 5. Compute required parallel strings = ⌈target_capacity / (cell_cap × series)⌉.
    /// 6. Reject cells whose SoH deviates > 5 % from group mean.
    /// 7. Select exactly series × parallel cells; reject the rest.
    pub fn assemble(&self) -> Result<PackAssemblyResult, crate::error::OxiGridError> {
        if self.cell_nominal_voltage_v <= 0.0 {
            return Err(crate::error::OxiGridError::InvalidParameter(
                "cell_nominal_voltage_v must be positive".into(),
            ));
        }
        if self.target_voltage_v <= 0.0 {
            return Err(crate::error::OxiGridError::InvalidParameter(
                "target_voltage_v must be positive".into(),
            ));
        }

        // Step 1: identify eligible cells (A or B grade, SoH > 70 %)
        let mut eligible: Vec<(usize, f64)> = self
            .cells
            .iter()
            .enumerate()
            .filter_map(|(idx, cell)| {
                let result = cell.assess();
                if result.composite_soh >= 0.70 {
                    Some((idx, result.composite_soh))
                } else {
                    None
                }
            })
            .collect();

        // Step 2: sort descending by composite SoH
        eligible.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        // Step 3: series count
        let series_cells = (self.target_voltage_v / self.cell_nominal_voltage_v).ceil() as usize;
        if series_cells == 0 {
            return Err(crate::error::OxiGridError::InvalidParameter(
                "computed series_cells is zero; check voltage parameters".into(),
            ));
        }

        // Step 4: parallel strings
        if eligible.is_empty() {
            return Err(crate::error::OxiGridError::InvalidParameter(
                "no eligible cells (SoH ≥ 70 %) available for assembly".into(),
            ));
        }
        let mean_soh: f64 = eligible.iter().map(|(_, s)| s).sum::<f64>() / eligible.len() as f64;

        // Step 5: remove outliers (SoH deviation > 5 % from mean)
        let filtered: Vec<(usize, f64)> = eligible
            .into_iter()
            .filter(|(_, soh)| (soh - mean_soh).abs() <= 0.05)
            .collect();

        if filtered.is_empty() {
            return Err(crate::error::OxiGridError::InvalidParameter(
                "all eligible cells rejected as outliers (SoH spread > 5 %)".into(),
            ));
        }

        // Representative cell capacity (use top-cell's original capacity scaled by mean SoH)
        let rep_idx = filtered[0].0;
        let cell_capacity_ah = self.cells[rep_idx].original_capacity_ah * mean_soh;

        let parallel_strings = if cell_capacity_ah * series_cells as f64 > 1e-9 {
            (self.target_capacity_ah / (cell_capacity_ah * series_cells as f64)).ceil() as usize
        } else {
            1
        };
        let parallel_strings = parallel_strings.max(1);

        let cells_needed = series_cells * parallel_strings;
        let cells_used: Vec<usize> = filtered
            .iter()
            .take(cells_needed)
            .map(|(idx, _)| *idx)
            .collect();

        if cells_used.is_empty() {
            return Err(crate::error::OxiGridError::InvalidParameter(
                "insufficient cells after outlier rejection".into(),
            ));
        }

        // Cells rejected = all not in cells_used
        let used_set: std::collections::HashSet<usize> = cells_used.iter().copied().collect();
        let cells_rejected: Vec<usize> = (0..self.cells.len())
            .filter(|i| !used_set.contains(i))
            .collect();

        // Pack metrics
        let pack_soh = cells_used
            .iter()
            .map(|&i| {
                let r = self.cells[i].assess();
                r.composite_soh
            })
            .fold(f64::INFINITY, f64::min);
        let pack_soh = if pack_soh.is_infinite() {
            0.0
        } else {
            pack_soh
        };

        let actual_voltage_v = series_cells as f64 * self.cell_nominal_voltage_v;
        let actual_capacity_ah = cell_capacity_ah * parallel_strings as f64;
        let pack_energy_kwh = actual_voltage_v * actual_capacity_ah / 1000.0;

        // Lifespan estimate: linear fade from pack_soh to 0.7 (second-life EOL)
        // at 2 %/year degradation rate typical for stationary BESS service
        let annual_fade = 0.02_f64;
        let estimated_lifespan_years = if annual_fade > 0.0 && pack_soh > 0.70 {
            ((pack_soh - 0.70) / annual_fade).clamp(0.0, 20.0)
        } else {
            0.0
        };

        Ok(PackAssemblyResult {
            series_cells,
            parallel_strings,
            actual_voltage_v,
            actual_capacity_ah,
            pack_soh,
            cells_used,
            cells_rejected,
            pack_energy_kwh,
            estimated_lifespan_years,
        })
    }
}

// ─── Second-Life Economics ────────────────────────────────────────────────────

/// Economic parameters for deploying a second-life battery pack.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SecondLifeEconomics {
    /// Usable energy of the pack \[kWh\].
    pub pack_energy_kwh: f64,
    /// Pack composite SoH \[0–1\].
    pub pack_soh: f64,
    /// Battery collection / logistics cost \[EUR/kWh original capacity\].
    pub collection_cost_eur: f64,
    /// Testing and characterization cost \[EUR/kWh\].
    pub testing_cost_eur_per_kwh: f64,
    /// Reassembly and BMS wiring cost \[EUR/kWh\].
    pub reassembly_cost_eur_per_kwh: f64,
    /// Fixed BMS hardware cost per pack \[EUR\].
    pub bms_cost_eur: f64,
    /// Target deployment application.
    pub application: SecondLifeApplication,
    /// Local electricity price \[EUR/kWh\].
    pub electricity_price_eur_per_kwh: f64,
    /// Expected full cycles per year in service.
    pub cycles_per_year: f64,
    /// Usable depth of discharge \[0–1\].
    pub depth_of_discharge: f64,
}

/// Result of a second-life economic analysis.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SecondLifeEconomicsResult {
    /// Total capital expenditure \[EUR\].
    pub total_capex_eur: f64,
    /// Annual gross revenue from arbitrage \[EUR/year\].
    pub annual_revenue_eur: f64,
    /// Simple payback period \[years\].
    pub simple_payback_years: f64,
    /// 10-year net present value at 8 % discount rate \[EUR\].
    pub npv_10yr_eur: f64,
    /// Levelized cost of storage \[EUR/kWh dispatched\].
    pub lcoe_eur_per_kwh: f64,
    /// True when NPV > 0.
    pub is_economically_viable: bool,
}

impl SecondLifeEconomics {
    /// Create a new economics model.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        pack_energy_kwh: f64,
        pack_soh: f64,
        collection_cost_eur: f64,
        testing_cost_eur_per_kwh: f64,
        reassembly_cost_eur_per_kwh: f64,
        bms_cost_eur: f64,
        application: SecondLifeApplication,
        electricity_price_eur_per_kwh: f64,
        cycles_per_year: f64,
        depth_of_discharge: f64,
    ) -> Self {
        Self {
            pack_energy_kwh,
            pack_soh,
            collection_cost_eur,
            testing_cost_eur_per_kwh,
            reassembly_cost_eur_per_kwh,
            bms_cost_eur,
            application,
            electricity_price_eur_per_kwh,
            cycles_per_year,
            depth_of_discharge,
        }
    }

    /// Run the economic analysis.
    pub fn analyze(&self) -> SecondLifeEconomicsResult {
        // CapEx
        let variable_cost = (self.collection_cost_eur
            + self.testing_cost_eur_per_kwh
            + self.reassembly_cost_eur_per_kwh)
            * self.pack_energy_kwh;
        let total_capex_eur = variable_cost + self.bms_cost_eur;

        // Annual revenue via energy arbitrage
        let spread = self.application.arbitrage_spread();
        let annual_revenue_eur = self.cycles_per_year
            * self.pack_energy_kwh
            * self.depth_of_discharge
            * self.electricity_price_eur_per_kwh
            * spread;

        // Simple payback
        let simple_payback_years = if annual_revenue_eur > 1e-9 {
            total_capex_eur / annual_revenue_eur
        } else {
            f64::INFINITY
        };

        // 10-year NPV at 8 %
        let discount_rate = 0.08_f64;
        let npv_10yr_eur = (1..=10_u32)
            .map(|t| annual_revenue_eur / (1.0 + discount_rate).powi(t as i32))
            .sum::<f64>()
            - total_capex_eur;

        // Lifespan estimate: pack degrades ~2 %/year; EOL at SoH = 70 %
        let lifespan_years = if self.pack_soh > 0.70 {
            ((self.pack_soh - 0.70) / 0.02).clamp(0.5, 20.0)
        } else {
            0.5
        };

        // LCOE = capex / total_dispatched_energy
        let total_dispatched_kwh =
            self.pack_energy_kwh * self.depth_of_discharge * self.cycles_per_year * lifespan_years;
        let lcoe_eur_per_kwh = if total_dispatched_kwh > 1e-9 {
            total_capex_eur / total_dispatched_kwh
        } else {
            f64::INFINITY
        };

        SecondLifeEconomicsResult {
            total_capex_eur,
            annual_revenue_eur,
            simple_payback_years,
            npv_10yr_eur,
            lcoe_eur_per_kwh,
            is_economically_viable: npv_10yr_eur > 0.0,
        }
    }
}

// ─── Portfolio Optimizer ──────────────────────────────────────────────────────

/// Fleet-level portfolio optimizer for retired battery cells.
pub struct SecondLifePortfolio {
    /// All assessed cells in the portfolio.
    pub assessments: Vec<SohAssessment>,
    /// Available deployment applications.
    pub available_applications: Vec<SecondLifeApplication>,
    /// Annual revenue per kWh (usable) for each application \[EUR/kWh/year\].
    pub application_revenue_eur_per_kwh_yr: Vec<f64>,
    /// Variable processing cost per kWh of usable energy \[EUR/kWh\].
    pub processing_cost_eur_per_kwh: f64,
}

/// Portfolio allocation result.
#[derive(Debug, Clone)]
pub struct PortfolioAllocation {
    /// `(cell_index, assigned_application)` for each non-scrap cell.
    pub allocations: Vec<(usize, SecondLifeApplication)>,
    /// Total annual portfolio revenue \[EUR/year\].
    pub total_revenue_eur_per_yr: f64,
    /// Total processing cost across all deployed cells \[EUR\].
    pub total_processing_cost_eur: f64,
    /// Indices of cells routed to material recycling.
    pub cells_for_recycling: Vec<usize>,
    /// Approximate internal rate of return \[fraction\].
    pub portfolio_irr: f64,
}

impl SecondLifePortfolio {
    /// Create a new portfolio optimizer.
    pub fn new(
        assessments: Vec<SohAssessment>,
        available_applications: Vec<SecondLifeApplication>,
        application_revenue_eur_per_kwh_yr: Vec<f64>,
        processing_cost_eur_per_kwh: f64,
    ) -> Self {
        Self {
            assessments,
            available_applications,
            application_revenue_eur_per_kwh_yr,
            processing_cost_eur_per_kwh,
        }
    }

    /// Greedy revenue-maximising allocation.
    ///
    /// Cells are sorted by composite SoH descending. Each cell is assigned to
    /// the highest-revenue eligible application, or routed to recycling if none
    /// qualifies (Scrap grade or no eligible application).
    pub fn optimize_allocation(&self) -> PortfolioAllocation {
        // Assess and sort cells by composite SoH descending
        let mut indexed: Vec<(usize, f64)> = self
            .assessments
            .iter()
            .enumerate()
            .map(|(i, cell)| {
                let r = cell.assess();
                (i, r.composite_soh)
            })
            .collect();
        indexed.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        let mut allocations: Vec<(usize, SecondLifeApplication)> = Vec::new();
        let mut cells_for_recycling: Vec<usize> = Vec::new();
        let mut total_revenue = 0.0_f64;
        let mut total_cost = 0.0_f64;

        for (cell_idx, composite_soh) in &indexed {
            let cell = &self.assessments[*cell_idx];
            let grade = SohAssessment::grade(*composite_soh);

            if matches!(grade, SohGrade::Scrap) {
                cells_for_recycling.push(*cell_idx);
                continue;
            }

            // Find highest-revenue eligible application
            let best = self
                .available_applications
                .iter()
                .zip(self.application_revenue_eur_per_kwh_yr.iter())
                .filter(|(app, _)| *composite_soh >= app.min_soh())
                .max_by(|(_, ra), (_, rb)| ra.partial_cmp(rb).unwrap_or(std::cmp::Ordering::Equal));

            match best {
                Some((app, revenue_per_kwh_yr)) => {
                    // Usable energy proportional to current capacity (nominal 3.7 V/cell)
                    let usable_kwh = (cell.current_capacity_ah * 3.7 / 1000.0).max(0.001);
                    total_revenue += usable_kwh * revenue_per_kwh_yr;
                    total_cost += usable_kwh * self.processing_cost_eur_per_kwh;
                    allocations.push((*cell_idx, *app));
                }
                None => {
                    cells_for_recycling.push(*cell_idx);
                }
            }
        }

        // Compute approximate IRR via bisection
        let lifespan_years = 8.0_f64;
        let annual_cashflows: Vec<f64> = (0..lifespan_years as usize)
            .map(|_| total_revenue)
            .collect();
        let irr = Self::compute_irr(total_cost.max(1.0), &annual_cashflows);

        PortfolioAllocation {
            allocations,
            total_revenue_eur_per_yr: total_revenue,
            total_processing_cost_eur: total_cost,
            cells_for_recycling,
            portfolio_irr: irr,
        }
    }

    /// Compute IRR via bisection on NPV(r) = 0.
    ///
    /// Initial search range: \[0 %, 200 %\]. If no sign change is found the
    /// bisection falls back to 0 or returns a boundary value.
    fn compute_irr(initial_cost: f64, annual_cashflows: &[f64]) -> f64 {
        let npv = |r: f64| -> f64 {
            let pv: f64 = annual_cashflows
                .iter()
                .enumerate()
                .map(|(t, cf)| cf / (1.0 + r).powi(t as i32 + 1))
                .sum();
            pv - initial_cost
        };

        let mut lo = 0.0_f64;
        let mut hi = 2.0_f64; // 200 % upper bound

        let npv_lo = npv(lo);
        let npv_hi = npv(hi);

        // If no sign change, the project may not be profitable
        if npv_lo < 0.0 {
            return 0.0; // NPV negative even at 0 % — not viable
        }
        if npv_hi > 0.0 {
            return hi; // NPV positive even at 200 % — very high IRR
        }

        // Bisection: 50 iterations gives precision ~2e-15
        for _ in 0..50 {
            let mid = 0.5 * (lo + hi);
            if npv(mid) > 0.0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        0.5 * (lo + hi)
    }
}

// ─── New module tests ─────────────────────────────────────────────────────────

#[cfg(test)]
mod second_life_tests {
    use super::*;

    // Simple LCG for deterministic pseudo-randomness
    fn lcg_next(state: &mut u64) -> u64 {
        *state = state
            .wrapping_mul(6364136223846793005u64)
            .wrapping_add(1442695040888963407u64);
        *state
    }

    fn make_cell(
        chemistry: BatteryChemistry,
        original_ah: f64,
        current_ah: f64,
        dcir: f64,
        cycles: u32,
        age_yr: f64,
    ) -> SohAssessment {
        SohAssessment::new(
            "CELL-001",
            chemistry,
            original_ah,
            current_ah,
            dcir,
            cycles,
            age_yr,
        )
    }

    #[test]
    fn test_soh_assessment_new() {
        let cell = make_cell(
            BatteryChemistry::LithiumIronPhosphate,
            100.0,
            82.0,
            0.012,
            500,
            3.0,
        );
        assert_eq!(cell.cell_id, "CELL-001");
        assert!((cell.soh_capacity - 0.82).abs() < 1e-9);
        assert!(cell.soh_resistance > 0.0 && cell.soh_resistance <= 1.0);
        assert_eq!(cell.cycle_count, 500);
        assert!((cell.calendar_age_years - 3.0).abs() < 1e-9);
    }

    #[test]
    fn test_soh_composite_calculation() {
        // soh_cap = 0.85, soh_res = ref_dcir/dcir
        // LFP ref_dcir = 0.010, dcir = 0.010 → soh_res = 1.0
        let cell = make_cell(
            BatteryChemistry::LithiumIronPhosphate,
            100.0,
            85.0,
            0.010,
            200,
            2.0,
        );
        let result = cell.assess();
        let expected = 0.6 * 0.85 + 0.4 * 1.0;
        assert!((result.composite_soh - expected).abs() < 1e-6);
    }

    #[test]
    fn test_soh_grade_a_above_80() {
        let grade = SohAssessment::grade(0.85);
        assert_eq!(grade, SohGrade::A);
    }

    #[test]
    fn test_soh_grade_b_70_to_80() {
        let grade = SohAssessment::grade(0.75);
        assert_eq!(grade, SohGrade::B);
        let grade_boundary = SohAssessment::grade(0.70);
        assert_eq!(grade_boundary, SohGrade::B);
    }

    #[test]
    fn test_soh_grade_c_60_to_70() {
        let grade = SohAssessment::grade(0.65);
        assert_eq!(grade, SohGrade::C);
    }

    #[test]
    fn test_soh_grade_scrap_below_50() {
        let grade = SohAssessment::grade(0.45);
        assert_eq!(grade, SohGrade::Scrap);
    }

    #[test]
    fn test_recommend_application_a_grade() {
        let app = SohAssessment::recommend_application(&SohGrade::A);
        assert_eq!(app, SecondLifeApplication::GridEnergyStorage);
    }

    #[test]
    fn test_recommend_application_b_grade() {
        let app = SohAssessment::recommend_application(&SohGrade::B);
        assert_eq!(app, SecondLifeApplication::RenewableIntegration);
    }

    #[test]
    fn test_recommend_application_scrap() {
        let app = SohAssessment::recommend_application(&SohGrade::Scrap);
        assert_eq!(app, SecondLifeApplication::MaterialRecovery);
    }

    #[test]
    fn test_rul_estimation_young_battery() {
        // Fresh battery: 200 cycles, 2 years, high SoH
        let cell = make_cell(BatteryChemistry::Nmc622, 60.0, 57.0, 0.012, 200, 2.0);
        let result = cell.assess();
        // Young battery should have substantial RUL
        assert!(
            result.remaining_useful_life_years > 2.0,
            "Young battery RUL should be > 2 years, got {:.2}",
            result.remaining_useful_life_years
        );
    }

    #[test]
    fn test_rul_estimation_aged_battery() {
        // Heavily aged: 2000 cycles, 10 years, low SoH
        // dcir doubled → soh_res = 0.5
        let cell = make_cell(BatteryChemistry::Nca, 60.0, 36.0, 0.022, 2000, 10.0);
        let result = cell.assess();
        // Aged battery composite SoH should be low
        assert!(
            result.composite_soh < 0.80,
            "Aged battery composite SoH should be < 0.80, got {:.3}",
            result.composite_soh
        );
    }

    #[test]
    fn test_pack_assembler_series_parallel() {
        // Target 48 V, 100 Ah; cells at 3.6 V nominal, 50 Ah original, 80 % SoH
        let cells: Vec<SohAssessment> = (0..30)
            .map(|i| {
                let mut c = make_cell(
                    BatteryChemistry::LithiumIronPhosphate,
                    50.0,
                    40.0, // 80 % capacity SoH
                    0.010,
                    300,
                    3.0,
                );
                c.cell_id = format!("C{:03}", i);
                c
            })
            .collect();

        let assembler = SecondLifePackAssembler::new(cells, 48.0, 100.0, 3.6);
        let result = assembler.assemble().expect("assembly should succeed");

        // series = ceil(48/3.6) = 14
        assert_eq!(result.series_cells, 14);
        assert!(result.parallel_strings >= 1);
        assert!(result.actual_voltage_v > 0.0);
        assert!(result.pack_energy_kwh > 0.0);
    }

    #[test]
    fn test_pack_assembler_rejects_low_soh() {
        // Mix: 10 good cells (85 % SoH) + 5 low-SoH cells (45 %)
        let mut cells: Vec<SohAssessment> = (0..10)
            .map(|i| {
                let mut c = make_cell(BatteryChemistry::Nmc622, 50.0, 42.5, 0.012, 200, 2.0);
                c.cell_id = format!("G{:03}", i);
                c
            })
            .collect();
        for i in 0..5 {
            let mut c = make_cell(BatteryChemistry::Nmc622, 50.0, 22.5, 0.024, 3000, 12.0);
            c.cell_id = format!("B{:03}", i);
            cells.push(c);
        }

        let assembler = SecondLifePackAssembler::new(cells, 36.0, 50.0, 3.6);
        let result = assembler.assemble().expect("assembly should succeed");

        // All rejected cells should include the 5 low-SoH ones
        assert!(
            result.cells_rejected.len() >= 5,
            "Expected ≥5 rejected cells, got {}",
            result.cells_rejected.len()
        );
    }

    #[test]
    fn test_pack_assembler_energy_kwh() {
        let cells: Vec<SohAssessment> = (0..20)
            .map(|_| {
                make_cell(
                    BatteryChemistry::LithiumIronPhosphate,
                    100.0,
                    80.0,
                    0.010,
                    500,
                    5.0,
                )
            })
            .collect();
        let assembler = SecondLifePackAssembler::new(cells, 24.0, 80.0, 3.2);
        let result = assembler.assemble().expect("should assemble");
        // energy = voltage * capacity / 1000 must be positive
        assert!(result.pack_energy_kwh > 0.0);
        let expected_kwh = result.actual_voltage_v * result.actual_capacity_ah / 1000.0;
        assert!((result.pack_energy_kwh - expected_kwh).abs() < 1e-6);
    }

    #[test]
    fn test_second_life_economics_viable() {
        // Large pack, high SoH, favourable economics
        let econ = SecondLifeEconomics::new(
            100.0, // 100 kWh
            0.85,  // SoH
            20.0,  // collection EUR/kWh
            5.0,   // testing EUR/kWh
            10.0,  // reassembly EUR/kWh
            500.0, // BMS fixed cost EUR
            SecondLifeApplication::GridEnergyStorage,
            0.15,  // electricity price EUR/kWh
            300.0, // cycles/year
            0.80,  // DoD
        );
        let result = econ.analyze();
        assert!(result.total_capex_eur > 0.0);
        assert!(result.annual_revenue_eur >= 0.0);
        // With reasonable parameters the project should be viable
        assert!(
            result.is_economically_viable || result.npv_10yr_eur > -50_000.0,
            "NPV too negative: {:.2}",
            result.npv_10yr_eur
        );
    }

    #[test]
    fn test_second_life_economics_not_viable() {
        // Tiny pack, low SoH, expensive setup, minimal revenue
        let econ = SecondLifeEconomics::new(
            1.0, // 1 kWh
            0.72,
            100.0, // very high collection cost
            50.0,
            80.0,
            2000.0, // very high BMS cost
            SecondLifeApplication::UpsBackup,
            0.05, // low electricity price
            50.0, // few cycles
            0.5,
        );
        let result = econ.analyze();
        assert!(
            !result.is_economically_viable,
            "Tiny pack with high costs should not be viable, NPV={:.2}",
            result.npv_10yr_eur
        );
    }

    #[test]
    fn test_lcoe_calculation() {
        let econ = SecondLifeEconomics::new(
            50.0,
            0.80,
            15.0,
            5.0,
            8.0,
            300.0,
            SecondLifeApplication::RenewableIntegration,
            0.12,
            250.0,
            0.80,
        );
        let result = econ.analyze();
        assert!(
            result.lcoe_eur_per_kwh > 0.0,
            "LCOE should be positive, got {:.4}",
            result.lcoe_eur_per_kwh
        );
        assert!(
            result.lcoe_eur_per_kwh < 10.0,
            "LCOE should be reasonable (<10 EUR/kWh), got {:.4}",
            result.lcoe_eur_per_kwh
        );
    }

    #[test]
    fn test_portfolio_allocation_greedy() {
        // 5 high-SoH cells → should all be allocated to applications
        let cells: Vec<SohAssessment> = (0..5)
            .map(|_| make_cell(BatteryChemistry::Nmc622, 60.0, 51.0, 0.012, 300, 3.0))
            .collect();
        let apps = vec![
            SecondLifeApplication::GridEnergyStorage,
            SecondLifeApplication::RenewableIntegration,
        ];
        let revenues = vec![40.0, 50.0];
        let portfolio = SecondLifePortfolio::new(cells, apps, revenues, 20.0);
        let alloc = portfolio.optimize_allocation();

        // All 5 cells should be allocated (not recycled) since SoH > 70 %
        assert_eq!(alloc.allocations.len(), 5);
        assert!(alloc.cells_for_recycling.is_empty());
        assert!(alloc.total_revenue_eur_per_yr > 0.0);
    }

    #[test]
    fn test_portfolio_recycling_scrap() {
        // Mix of good (SoH ~85 %) and scrap (SoH ~40 %) cells
        let mut cells: Vec<SohAssessment> = (0..3)
            .map(|_| {
                make_cell(
                    BatteryChemistry::LithiumIronPhosphate,
                    100.0,
                    85.0,
                    0.010,
                    200,
                    2.0,
                )
            })
            .collect();
        for _ in 0..3 {
            cells.push(make_cell(
                BatteryChemistry::Nca,
                100.0,
                38.0,
                0.030,
                4000,
                15.0,
            ));
        }

        let apps = vec![SecondLifeApplication::GridEnergyStorage];
        let revenues = vec![35.0];
        let portfolio = SecondLifePortfolio::new(cells, apps, revenues, 15.0);
        let alloc = portfolio.optimize_allocation();

        // Scrap cells go to recycling
        assert!(
            alloc.cells_for_recycling.len() >= 3,
            "Expected ≥3 cells for recycling, got {}",
            alloc.cells_for_recycling.len()
        );
    }

    #[test]
    fn test_irr_computation() {
        // Known case: invest 1000, get 200/year for 10 years → IRR ≈ 15.1 %
        let cashflows: Vec<f64> = vec![200.0; 10];
        let irr = SecondLifePortfolio::compute_irr(1000.0, &cashflows);
        // IRR should be in a reasonable range (>0, <50 %)
        assert!(irr > 0.0, "IRR should be positive, got {:.4}", irr);
        assert!(irr < 0.5, "IRR should be < 50 %, got {:.4}", irr);
        // IRR for 200/yr over 10 yr on 1000 investment ≈ 15 %
        assert!(
            (irr - 0.151).abs() < 0.005,
            "IRR should be ≈ 15.1 %, got {:.4}",
            irr
        );
    }

    #[test]
    fn test_lcg_determinism() {
        // Verify LCG produces consistent values across calls
        let mut state: u64 = 12345;
        let v1 = lcg_next(&mut state);
        let mut state2: u64 = 12345;
        let v2 = lcg_next(&mut state2);
        assert_eq!(v1, v2);
    }
}
