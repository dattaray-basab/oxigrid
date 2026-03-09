//! Advanced battery degradation model for lithium-ion chemistries.
//!
//! Combines three major degradation pathways:
//! - **Calendar aging**: SEI layer growth driven by time, temperature, and SoC.
//! - **Cycle aging**: Capacity fade per charge/discharge cycle driven by DoD, C-rate, and temperature.
//! - **Resistance growth**: Impedance rise from SEI accumulation and contact degradation.
//!
//! # Physics Background
//!
//! The SEI (Solid Electrolyte Interphase) grows on the graphite anode during storage and cycling.
//! Its growth follows an Arrhenius-type temperature dependence and is diffusion-limited
//! (square-root time dependence in calendar mode). Cycle aging adds mechanical stress from
//! lithiation/delithiation volume changes proportional to DoD^β.
//!
//! # Example
//!
//! ```rust
//! use oxigrid::battery::degradation_model::{DegradationModel, DegradationChemistry, AgingConditions};
//!
//! let mut model = DegradationModel::new(DegradationChemistry::NmcGraphite, 50.0, 3.7);
//! assert!((model.compute_soh() - 1.0).abs() < 1e-9);
//!
//! let conditions = AgingConditions {
//!     temperature_c: 25.0,
//!     soc_pct: 50.0,
//!     c_rate: 1.0,
//!     depth_of_discharge_pct: 80.0,
//!     cycle_count: 0.0,
//!     calendar_days: 0.0,
//! };
//! model.step_calendar(365.0, &conditions);
//! assert!(model.compute_soh() < 1.0);
//! ```

use serde::{Deserialize, Serialize};

/// Universal gas constant [J/(mol·K)].
const R_GAS: f64 = 8.314;

/// Pre-exponential SEI growth factor [1/day].
const K_SEI: f64 = 1e-4;

/// Default initial internal resistance [Ω].
const DEFAULT_R0: f64 = 0.02;

/// Maximum temperature history window size.
const TEMP_HISTORY_SIZE: usize = 1000;

// ─────────────────────────────────────────────────────────────────────────────
// Enums
// ─────────────────────────────────────────────────────────────────────────────

/// Battery chemistry type, determining degradation kinetic parameters.
///
/// Each variant maps to a unique set of Arrhenius activation energy, calendar
/// aging coefficient, cycle aging coefficient, and DoD exponent.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DegradationChemistry {
    /// NMC cathode with graphite anode — most common EV chemistry.
    NmcGraphite,
    /// LFP cathode with graphite anode — long-cycle-life, thermally stable.
    LfpGraphite,
    /// NCA cathode with graphite anode — high energy density.
    NcaGraphite,
    /// NMC cathode with LTO (Li4Ti5O12) anode — ultra-long life, low energy density.
    NmcLto,
    /// Solid-state electrolyte — minimal SEI growth, lowest degradation.
    SolidState,
    /// Sodium-ion chemistry — emerging technology, moderate degradation.
    SodiumIon,
}

impl DegradationChemistry {
    /// SEI activation energy [J/mol] (converted from kJ/mol table).
    pub fn ea_sei_j_mol(self) -> f64 {
        match self {
            Self::NmcGraphite => 60_000.0,
            Self::LfpGraphite => 55_000.0,
            Self::NcaGraphite => 65_000.0,
            Self::NmcLto => 50_000.0,
            Self::SolidState => 45_000.0,
            Self::SodiumIon => 52_000.0,
        }
    }

    /// Calendar aging pre-exponential coefficient α_cal [fraction/√day].
    pub fn alpha_cal(self) -> f64 {
        match self {
            Self::NmcGraphite => 0.020,
            Self::LfpGraphite => 0.012,
            Self::NcaGraphite => 0.025,
            Self::NmcLto => 0.005,
            Self::SolidState => 0.003,
            Self::SodiumIon => 0.010,
        }
    }

    /// Cycle aging coefficient α_cyc [fraction per cycle at DoD=100%].
    pub fn alpha_cyc(self) -> f64 {
        match self {
            Self::NmcGraphite => 0.015,
            Self::LfpGraphite => 0.008,
            Self::NcaGraphite => 0.020,
            Self::NmcLto => 0.004,
            Self::SolidState => 0.002,
            Self::SodiumIon => 0.007,
        }
    }

    /// DoD exponent β (Wöhler curve exponent).
    pub fn beta_dod(self) -> f64 {
        match self {
            Self::NmcGraphite => 0.80,
            Self::LfpGraphite => 0.65,
            Self::NcaGraphite => 0.85,
            Self::NmcLto => 0.50,
            Self::SolidState => 0.40,
            Self::SodiumIon => 0.60,
        }
    }
}

/// Individual degradation mechanism that contributes to capacity/resistance loss.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DegradationMechanism {
    /// SEI layer growth on the anode (calendar + cycle).
    SeiGrowth,
    /// Lithium plating at the anode during fast charging or low temperatures.
    LithiumPlating,
    /// Cathode active material dissolution into the electrolyte.
    CathodeDissolution,
    /// Particle cracking and loss of electrical contact from volume changes.
    MechanicalStress,
    /// Electrolyte oxidation at high voltages or temperatures.
    ElectrolyteDecomposition,
    /// Pure calendar aging (storage at elevated temperature/SoC).
    CalendarAging,
    /// Pure cycle aging (charge/discharge throughput).
    CycleAging,
}

/// Operating mode for the degradation model integration step.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AgingMode {
    /// Only calendar aging is applied (no cycling).
    Calendar,
    /// Only cycle aging is applied.
    Cycle,
    /// Both calendar and cycle aging are combined.
    Combined,
}

// ─────────────────────────────────────────────────────────────────────────────
// Structs
// ─────────────────────────────────────────────────────────────────────────────

/// Operating conditions during an aging time step.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AgingConditions {
    /// Ambient (or cell surface) temperature [°C].
    pub temperature_c: f64,
    /// State of charge [0–100 %].
    pub soc_pct: f64,
    /// Charge/discharge C-rate (0 = idle storage).
    pub c_rate: f64,
    /// Depth of discharge per cycle [0–100 %].
    pub depth_of_discharge_pct: f64,
    /// Accumulated full equivalent cycles to date.
    pub cycle_count: f64,
    /// Calendar age in days to date.
    pub calendar_days: f64,
}

impl Default for AgingConditions {
    fn default() -> Self {
        Self {
            temperature_c: 25.0,
            soc_pct: 50.0,
            c_rate: 0.0,
            depth_of_discharge_pct: 80.0,
            cycle_count: 0.0,
            calendar_days: 0.0,
        }
    }
}

/// Breakdown of capacity fade by degradation mechanism.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CapacityFade {
    /// Remaining relative capacity Q/Q₀ (1.0 = new, 0.8 = end-of-life).
    pub relative_capacity: f64,
    /// Capacity loss attributed to calendar aging [%].
    pub calendar_loss_pct: f64,
    /// Capacity loss attributed to cycle aging [%].
    pub cycle_loss_pct: f64,
    /// SEI layer contribution to capacity loss [%].
    pub sei_loss_pct: f64,
    /// Lithium plating contribution to capacity loss [%].
    pub plating_loss_pct: f64,
    /// Mechanical stress contribution to capacity loss [%].
    pub mechanical_loss_pct: f64,
    /// Total capacity loss (sum of all mechanisms) [%].
    pub total_loss_pct: f64,
    /// Remaining useful life in full equivalent cycles (until 80 % SoH).
    pub rul_cycles: f64,
    /// Remaining useful life in calendar days (until 80 % SoH).
    pub rul_days: f64,
}

/// Impedance growth decomposition.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResistanceGrowth {
    /// Relative total resistance R/R₀ (1.0 = new).
    pub relative_resistance: f64,
    /// SEI film resistance contribution [Ω].
    pub sei_resistance_ohm: f64,
    /// Contact resistance from particle disconnection [Ω].
    pub contact_resistance_ohm: f64,
    /// Total resistance including all contributions [Ω].
    pub total_resistance_ohm: f64,
}

/// Rainflow-counting cycle counter for depth-of-discharge extraction.
///
/// Implements a simplified 4-point rainflow algorithm to count
/// charge/discharge cycles from a SoC time-series, producing
/// (DoD, mean_SoC) pairs for each completed cycle.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CycleCounterDoD {
    /// Stored half-cycle amplitude stack for rainflow counting.
    pub half_cycles: Vec<f64>,
    /// Completed (DoD_pct, mean_soc_pct) cycle pairs.
    pub completed_cycles: Vec<(f64, f64)>,
    /// SoC at the start of each candidate half-cycle [%].
    soc_at_half_start: Vec<f64>,
}

impl CycleCounterDoD {
    /// Create a new empty cycle counter.
    pub fn new() -> Self {
        Self {
            half_cycles: Vec::new(),
            completed_cycles: Vec::new(),
            soc_at_half_start: Vec::new(),
        }
    }

    /// Add a new SoC measurement point and perform rainflow extraction.
    ///
    /// Call this after every SoC update. Completed cycles are accumulated
    /// into `completed_cycles`.
    pub fn add_soc_point(&mut self, soc: f64) {
        let soc = soc.clamp(0.0, 100.0);
        self.half_cycles.push(soc);
        self.soc_at_half_start.push(soc);

        // Rainflow: need at least 3 points to form a cycle candidate
        loop {
            let n = self.half_cycles.len();
            if n < 3 {
                break;
            }
            let x0 = self.half_cycles[n - 3];
            let x1 = self.half_cycles[n - 2];
            let x2 = self.half_cycles[n - 1];

            let range1 = (x1 - x0).abs();
            let range2 = (x2 - x1).abs();

            // A cycle is counted when the inner range ≤ outer range
            if range1 <= range2 {
                let dod = range1;
                let mean_soc = (x0 + x1) / 2.0;
                self.completed_cycles.push((dod, mean_soc));
                // Remove the two points that formed this cycle
                self.half_cycles.remove(n - 3);
                self.half_cycles.remove(n - 3); // now at n-2 after first removal
                if self.soc_at_half_start.len() >= 2 {
                    self.soc_at_half_start.remove(n - 3);
                    let idx = self.soc_at_half_start.len().saturating_sub(1);
                    if idx < self.soc_at_half_start.len() {
                        self.soc_at_half_start.remove(idx);
                    }
                }
            } else {
                break;
            }
        }
    }

    /// Total count of full equivalent cycles extracted so far.
    ///
    /// Returns the number of completed rainflow cycles (each counted as one
    /// full cycle regardless of DoD for simplicity).
    pub fn count_cycles(&self) -> f64 {
        self.completed_cycles.len() as f64
    }
}

impl Default for CycleCounterDoD {
    fn default() -> Self {
        Self::new()
    }
}

/// Physics-based battery degradation model.
///
/// Tracks capacity fade and resistance growth for a single lithium-ion cell
/// using chemistry-specific Arrhenius calendar aging, empirical cycle aging,
/// and SEI diffusion-limited growth.
///
/// # Usage
///
/// 1. Create with [`DegradationModel::new`].
/// 2. Call [`step_calendar`](DegradationModel::step_calendar) for storage periods.
/// 3. Call [`step_cycle`](DegradationModel::step_cycle) after each charge/discharge cycle.
/// 4. Query [`compute_soh`](DegradationModel::compute_soh), [`compute_capacity_fade`](DegradationModel::compute_capacity_fade),
///    and [`compute_resistance_growth`](DegradationModel::compute_resistance_growth).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DegradationModel {
    /// Battery chemistry (determines kinetic parameters).
    pub chemistry: DegradationChemistry,
    /// Nominal (nameplate) capacity [Ah].
    pub nominal_capacity_ah: f64,
    /// Nominal voltage [V].
    pub nominal_voltage_v: f64,
    /// Fresh internal resistance [Ω].
    pub initial_resistance_ohm: f64,
    /// Current (degraded) capacity [Ah].
    pub current_capacity_ah: f64,
    /// Current (degraded) internal resistance [Ω].
    pub current_resistance_ohm: f64,
    /// Accumulated full equivalent cycles.
    pub total_cycles: f64,
    /// Calendar age [days].
    pub calendar_age_days: f64,
    /// Cumulative capacity lost to SEI growth [Ah].
    pub cumulative_sei_loss: f64,
    /// Cumulative capacity lost to lithium plating [Ah].
    pub cumulative_plating_loss: f64,
    /// Cumulative capacity lost to mechanical stress [Ah].
    pub cumulative_mechanical_loss: f64,
    /// Sliding window of recent cell temperatures [°C] (last 1000 samples).
    pub temperature_history: Vec<f64>,
    /// Cumulative calendar capacity loss [Ah] (for fade breakdown).
    pub cumulative_calendar_loss: f64,
    /// Cumulative cycle capacity loss [Ah] (for fade breakdown).
    pub cumulative_cycle_loss: f64,
}

impl DegradationModel {
    /// Create a new, fresh battery degradation model.
    ///
    /// # Arguments
    /// - `chemistry`: The cell chemistry determining aging kinetics.
    /// - `capacity_ah`: Nominal capacity in Ampere-hours.
    /// - `voltage_v`: Nominal cell voltage in Volts.
    pub fn new(chemistry: DegradationChemistry, capacity_ah: f64, voltage_v: f64) -> Self {
        Self {
            chemistry,
            nominal_capacity_ah: capacity_ah,
            nominal_voltage_v: voltage_v,
            initial_resistance_ohm: DEFAULT_R0,
            current_capacity_ah: capacity_ah,
            current_resistance_ohm: DEFAULT_R0,
            total_cycles: 0.0,
            calendar_age_days: 0.0,
            cumulative_sei_loss: 0.0,
            cumulative_plating_loss: 0.0,
            cumulative_mechanical_loss: 0.0,
            temperature_history: Vec::new(),
            cumulative_calendar_loss: 0.0,
            cumulative_cycle_loss: 0.0,
        }
    }

    // ── Internal helpers ──────────────────────────────────────────────────────

    /// SoC stress factor: storage at high SoC accelerates SEI growth.
    ///
    /// `f(SoC) = 1 + 0.5 * ((SoC - 50) / 50)²`
    fn soc_stress_factor(soc_pct: f64) -> f64 {
        let s = (soc_pct.clamp(0.0, 100.0) - 50.0) / 50.0;
        1.0 + 0.5 * s * s
    }

    /// Normalized Arrhenius factor referenced to 25 °C:
    /// `exp(-Ea/R * (1/T - 1/T_ref))` where T_ref = 298.15 K.
    ///
    /// This equals 1.0 at 25 °C and increases/decreases with temperature,
    /// making the pre-exponential coefficients (α_cal, α_cyc) physically
    /// meaningful at the reference condition.
    fn arrhenius(ea_j_mol: f64, temp_c: f64) -> f64 {
        const T_REF: f64 = 298.15; // 25 °C reference
        let t_k = temp_c + 273.15;
        ((-ea_j_mol / R_GAS) * (1.0 / t_k - 1.0 / T_REF)).exp()
    }

    /// Temperature cycle-aging factor: `exp(0.02 * (T_c - 25))`.
    fn temperature_cycle_factor(temp_c: f64) -> f64 {
        (0.02 * (temp_c - 25.0)).exp()
    }

    // ── Public computation methods ────────────────────────────────────────────

    /// Compute the instantaneous SEI growth rate [Ah/day].
    ///
    /// Based on Arrhenius kinetics with diffusion limitation:
    /// `dSEI/dt = k_sei * exp(-Ea / (R*T)) * f(SoC) / sqrt(t + 1)`
    ///
    /// The `+1` prevents division by zero at `t = 0`.
    pub fn compute_sei_growth_rate(&self, temp_c: f64, soc_pct: f64) -> f64 {
        let ea = self.chemistry.ea_sei_j_mol();
        let arr = Self::arrhenius(ea, temp_c);
        let soc_f = Self::soc_stress_factor(soc_pct);
        let t_ref = (self.calendar_age_days + 1.0).sqrt();
        K_SEI * arr * soc_f / t_ref * self.nominal_capacity_ah
    }

    /// Compute incremental calendar capacity loss for a time step [Ah].
    ///
    /// Uses the square-root (parabolic) SEI growth model:
    /// `ΔQ_cal = (α_cal/100) * Arr(T) * f(SoC) * [sqrt(t + dt) - sqrt(t)]`
    ///
    /// `α_cal` from the chemistry table is in units of percent-capacity per √day
    /// at the reference temperature (25 °C), so it is divided by 100 to convert
    /// to a fractional loss.
    pub fn compute_calendar_loss(&self, dt_days: f64, temp_c: f64, soc_pct: f64) -> f64 {
        let ea = self.chemistry.ea_sei_j_mol();
        // alpha_cal from the table is in units of % per √day — convert to fraction
        let alpha = self.chemistry.alpha_cal() / 100.0;
        let arr = Self::arrhenius(ea, temp_c);
        let soc_f = Self::soc_stress_factor(soc_pct);
        let t0 = self.calendar_age_days;
        let t1 = t0 + dt_days.max(0.0);
        let delta_sqrt = t1.sqrt() - t0.sqrt();
        // Fraction of nominal capacity lost
        let loss_frac = alpha * arr * soc_f * delta_sqrt;
        (loss_frac * self.nominal_capacity_ah).max(0.0)
    }

    /// Compute capacity loss for a single charge/discharge cycle [Ah].
    ///
    /// Empirical Wöhler-type model:
    /// `Q_cyc = (α_cyc/100) * (DoD/100)^β * (1 + 0.15*(C_eff - 1)) * T_factor`
    ///
    /// `α_cyc` is in units of percent-capacity per cycle (at 100 % DoD, 1 C, 25 °C),
    /// so it is divided by 100 here to convert to a fractional loss.
    pub fn compute_cycle_loss(&self, dod_pct: f64, c_rate: f64, temp_c: f64) -> f64 {
        // alpha_cyc from the table is in units of % per cycle — convert to fraction
        let alpha = self.chemistry.alpha_cyc() / 100.0;
        let beta = self.chemistry.beta_dod();
        let dod_norm = dod_pct.clamp(0.0, 100.0) / 100.0;
        let c_eff = c_rate.clamp(0.0, 3.0);
        let c_factor = 1.0 + 0.15 * (c_eff - 1.0);
        let t_factor = Self::temperature_cycle_factor(temp_c);
        let loss_frac = alpha * dod_norm.powf(beta) * c_factor * t_factor;
        (loss_frac * self.nominal_capacity_ah).max(0.0)
    }

    /// Advance the model by a calendar (storage) time step.
    ///
    /// Updates `current_capacity_ah`, `current_resistance_ohm`,
    /// `calendar_age_days`, and appends the temperature to the history window.
    pub fn step_calendar(&mut self, dt_days: f64, conditions: &AgingConditions) {
        if dt_days <= 0.0 {
            return;
        }
        let loss_ah =
            self.compute_calendar_loss(dt_days, conditions.temperature_c, conditions.soc_pct);

        // Clamp actual loss to remaining capacity so cumulative trackers stay consistent
        let actual_loss = loss_ah.min(self.current_capacity_ah);

        // Attribute SEI portion (dominant mechanism in calendar aging)
        let sei_portion = actual_loss * 0.7;
        self.cumulative_sei_loss += sei_portion;
        self.cumulative_calendar_loss += actual_loss;

        // Apply capacity loss (floor at 0)
        self.current_capacity_ah = (self.current_capacity_ah - actual_loss).max(0.0);

        // Advance calendar age
        self.calendar_age_days += dt_days;

        // Update resistance
        self.current_resistance_ohm = self.compute_resistance_ohm_internal();

        // Update temperature history (sliding window)
        self.temperature_history.push(conditions.temperature_c);
        if self.temperature_history.len() > TEMP_HISTORY_SIZE {
            self.temperature_history.remove(0);
        }
    }

    /// Advance the model by one charge/discharge cycle.
    ///
    /// Updates `current_capacity_ah`, `total_cycles`, `current_resistance_ohm`,
    /// and accumulates mechanical stress loss.
    pub fn step_cycle(&mut self, dod_pct: f64, conditions: &AgingConditions) {
        let loss_ah = self.compute_cycle_loss(dod_pct, conditions.c_rate, conditions.temperature_c);

        // Clamp actual loss to remaining capacity so cumulative trackers stay consistent
        let actual_loss = loss_ah.min(self.current_capacity_ah);

        // Distribute cycle loss: mechanical stress ~ DoD fraction
        let mech_portion = actual_loss * 0.2;
        self.cumulative_mechanical_loss += mech_portion;
        self.cumulative_cycle_loss += actual_loss;

        self.current_capacity_ah = (self.current_capacity_ah - actual_loss).max(0.0);
        self.total_cycles += 1.0;

        self.current_resistance_ohm = self.compute_resistance_ohm_internal();
    }

    /// Compute current State of Health (SoH = Q_current / Q_nominal).
    pub fn compute_soh(&self) -> f64 {
        if self.nominal_capacity_ah <= 0.0 {
            return 0.0;
        }
        (self.current_capacity_ah / self.nominal_capacity_ah).clamp(0.0, 1.0)
    }

    /// Compute remaining useful life in full equivalent cycles until 80 % SoH.
    ///
    /// Uses the reference cycle loss rate at 50 % DoD, 1 C, 25 °C.
    pub fn compute_rul_cycles(&self) -> f64 {
        let soh = self.compute_soh();
        if soh <= 0.80 {
            return 0.0;
        }
        let fade_per_cycle = self.compute_cycle_loss(50.0, 1.0, 25.0);
        if fade_per_cycle <= 0.0 || self.nominal_capacity_ah <= 0.0 {
            return f64::INFINITY;
        }
        let fade_per_cycle_frac = fade_per_cycle / self.nominal_capacity_ah;
        ((soh - 0.80) / fade_per_cycle_frac).max(0.0)
    }

    /// Estimate remaining calendar life in days given a daily cycle frequency.
    ///
    /// Combines both calendar and cycle aging to project time until 80 % SoH.
    pub fn estimate_remaining_life_days(&self, daily_cycles: f64) -> f64 {
        let soh = self.compute_soh();
        if soh <= 0.80 {
            return 0.0;
        }
        let rul_cyc = self.compute_rul_cycles();
        let daily = daily_cycles.max(1e-9);
        rul_cyc / daily
    }

    /// Compute full capacity fade breakdown.
    pub fn compute_capacity_fade(&self) -> CapacityFade {
        let nom = self.nominal_capacity_ah.max(1e-12);
        let total_loss_pct = 100.0 * (1.0 - self.current_capacity_ah / nom).max(0.0);
        let calendar_loss_pct = 100.0 * self.cumulative_calendar_loss / nom;
        let cycle_loss_pct = 100.0 * self.cumulative_cycle_loss / nom;
        let sei_loss_pct = 100.0 * self.cumulative_sei_loss / nom;
        let plating_loss_pct = 100.0 * self.cumulative_plating_loss / nom;
        let mechanical_loss_pct = 100.0 * self.cumulative_mechanical_loss / nom;

        let rul_cycles = self.compute_rul_cycles();
        let daily_ref = 0.5_f64; // reference daily cycles
        let rul_days = self.estimate_remaining_life_days(daily_ref);

        CapacityFade {
            relative_capacity: self.compute_soh(),
            calendar_loss_pct,
            cycle_loss_pct,
            sei_loss_pct,
            plating_loss_pct,
            mechanical_loss_pct,
            total_loss_pct,
            rul_cycles,
            rul_days,
        }
    }

    /// Compute resistance growth from cycle and calendar aging.
    pub fn compute_resistance_growth(&self) -> ResistanceGrowth {
        let r0 = self.initial_resistance_ohm;
        let r_total = self.compute_resistance_ohm_internal();
        let relative = if r0 > 0.0 { r_total / r0 } else { 1.0 };

        // SEI contributes most resistance growth (fraction of cycle component)
        let r_sei = r0 * 0.001 * self.total_cycles.sqrt();
        // Contact resistance from mechanical degradation
        let r_contact = r0 * 0.0005 * self.calendar_age_days.sqrt();

        ResistanceGrowth {
            relative_resistance: relative,
            sei_resistance_ohm: r_sei,
            contact_resistance_ohm: r_contact,
            total_resistance_ohm: r_total,
        }
    }

    /// Reset the model to factory-fresh condition.
    pub fn reset(&mut self) {
        self.current_capacity_ah = self.nominal_capacity_ah;
        self.current_resistance_ohm = self.initial_resistance_ohm;
        self.total_cycles = 0.0;
        self.calendar_age_days = 0.0;
        self.cumulative_sei_loss = 0.0;
        self.cumulative_plating_loss = 0.0;
        self.cumulative_mechanical_loss = 0.0;
        self.cumulative_calendar_loss = 0.0;
        self.cumulative_cycle_loss = 0.0;
        self.temperature_history.clear();
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    /// Internal resistance computation from cycle count and calendar age.
    fn compute_resistance_ohm_internal(&self) -> f64 {
        let r0 = self.initial_resistance_ohm;
        r0 * (1.0 + 0.002 * self.total_cycles + 0.001 * self.calendar_age_days.sqrt())
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn default_conditions() -> AgingConditions {
        AgingConditions {
            temperature_c: 25.0,
            soc_pct: 50.0,
            c_rate: 1.0,
            depth_of_discharge_pct: 80.0,
            cycle_count: 0.0,
            calendar_days: 0.0,
        }
    }

    #[test]
    fn test_new_battery_soh_unity() {
        let model = DegradationModel::new(DegradationChemistry::NmcGraphite, 50.0, 3.7);
        let soh = model.compute_soh();
        assert!(
            (soh - 1.0).abs() < 1e-9,
            "New battery SoH should be 1.0, got {soh}"
        );
    }

    #[test]
    fn test_calendar_aging_positive() {
        let mut model = DegradationModel::new(DegradationChemistry::NmcGraphite, 50.0, 3.7);
        let initial_cap = model.current_capacity_ah;
        let cond = default_conditions();
        model.step_calendar(365.0, &cond);
        assert!(
            model.current_capacity_ah < initial_cap,
            "Capacity should decrease after 1 year of calendar aging"
        );
    }

    #[test]
    fn test_calendar_aging_monotone() {
        let mut model = DegradationModel::new(DegradationChemistry::NmcGraphite, 50.0, 3.7);
        let cond = default_conditions();
        model.step_calendar(365.0, &cond);
        let cap_1yr = model.current_capacity_ah;
        model.step_calendar(365.0, &cond);
        let cap_2yr = model.current_capacity_ah;
        assert!(
            cap_2yr < cap_1yr,
            "More days should give more capacity loss: {cap_1yr} vs {cap_2yr}"
        );
    }

    #[test]
    fn test_temperature_effect_calendar() {
        let mut hot = DegradationModel::new(DegradationChemistry::NmcGraphite, 50.0, 3.7);
        let mut cold = DegradationModel::new(DegradationChemistry::NmcGraphite, 50.0, 3.7);
        let mut hot_cond = default_conditions();
        hot_cond.temperature_c = 45.0;
        let mut cold_cond = default_conditions();
        cold_cond.temperature_c = 10.0;

        hot.step_calendar(365.0, &hot_cond);
        cold.step_calendar(365.0, &cold_cond);

        assert!(
            hot.current_capacity_ah < cold.current_capacity_ah,
            "Higher temperature should cause more capacity loss"
        );
    }

    #[test]
    fn test_soc_effect_calendar() {
        let mut high_soc = DegradationModel::new(DegradationChemistry::NmcGraphite, 50.0, 3.7);
        let mut low_soc = DegradationModel::new(DegradationChemistry::NmcGraphite, 50.0, 3.7);
        let mut high_cond = default_conditions();
        high_cond.soc_pct = 95.0;
        let mut low_cond = default_conditions();
        low_cond.soc_pct = 20.0;

        high_soc.step_calendar(365.0, &high_cond);
        low_soc.step_calendar(365.0, &low_cond);

        assert!(
            high_soc.current_capacity_ah < low_soc.current_capacity_ah,
            "High SoC should cause faster calendar aging"
        );
    }

    #[test]
    fn test_cycle_aging_positive() {
        let mut model = DegradationModel::new(DegradationChemistry::NmcGraphite, 50.0, 3.7);
        let initial_cap = model.current_capacity_ah;
        let cond = default_conditions();
        model.step_cycle(80.0, &cond);
        assert!(
            model.current_capacity_ah < initial_cap,
            "Capacity should decrease after cycling"
        );
    }

    #[test]
    fn test_cycle_aging_dod_effect() {
        let mut deep = DegradationModel::new(DegradationChemistry::NmcGraphite, 50.0, 3.7);
        let mut shallow = DegradationModel::new(DegradationChemistry::NmcGraphite, 50.0, 3.7);
        let cond = default_conditions();

        for _ in 0..100 {
            deep.step_cycle(90.0, &cond);
            shallow.step_cycle(20.0, &cond);
        }

        assert!(
            deep.current_capacity_ah < shallow.current_capacity_ah,
            "Deeper DoD should cause more degradation"
        );
    }

    #[test]
    fn test_cycle_aging_crate_effect() {
        let mut fast = DegradationModel::new(DegradationChemistry::NmcGraphite, 50.0, 3.7);
        let mut slow = DegradationModel::new(DegradationChemistry::NmcGraphite, 50.0, 3.7);

        let mut fast_cond = default_conditions();
        fast_cond.c_rate = 3.0;
        let mut slow_cond = default_conditions();
        slow_cond.c_rate = 0.2;

        for _ in 0..100 {
            fast.step_cycle(80.0, &fast_cond);
            slow.step_cycle(80.0, &slow_cond);
        }

        assert!(
            fast.current_capacity_ah < slow.current_capacity_ah,
            "Higher C-rate should cause more degradation"
        );
    }

    #[test]
    fn test_sei_growth_rate_arrhenius() {
        let hot = DegradationModel::new(DegradationChemistry::NmcGraphite, 50.0, 3.7);
        let cold = DegradationModel::new(DegradationChemistry::NmcGraphite, 50.0, 3.7);

        let rate_hot = hot.compute_sei_growth_rate(45.0, 50.0);
        let rate_cold = cold.compute_sei_growth_rate(10.0, 50.0);

        assert!(
            rate_hot > rate_cold,
            "Higher temperature should yield higher SEI growth rate: hot={rate_hot:.4e}, cold={rate_cold:.4e}"
        );
    }

    #[test]
    fn test_resistance_growth_with_cycles() {
        let mut model = DegradationModel::new(DegradationChemistry::NmcGraphite, 50.0, 3.7);
        let initial_r = model.current_resistance_ohm;
        let cond = default_conditions();
        for _ in 0..500 {
            model.step_cycle(80.0, &cond);
        }
        assert!(
            model.current_resistance_ohm > initial_r,
            "Resistance should grow with cycling: initial={initial_r:.4}, current={:.4}",
            model.current_resistance_ohm
        );
    }

    #[test]
    fn test_resistance_growth_with_calendar() {
        let mut model = DegradationModel::new(DegradationChemistry::NmcGraphite, 50.0, 3.7);
        let initial_r = model.current_resistance_ohm;
        let cond = default_conditions();
        model.step_calendar(365.0 * 3.0, &cond); // 3 years
        assert!(
            model.current_resistance_ohm > initial_r,
            "Resistance should grow with calendar age"
        );
    }

    #[test]
    fn test_soh_after_1000_cycles() {
        let mut model = DegradationModel::new(DegradationChemistry::NmcGraphite, 50.0, 3.7);
        let cond = default_conditions();
        for _ in 0..1000 {
            model.step_cycle(80.0, &cond);
        }
        let soh = model.compute_soh();
        assert!(
            soh > 0.5 && soh <= 1.0,
            "SoH after 1000 cycles at 80% DoD should be in (0.5, 1.0], got {soh:.4}"
        );
    }

    #[test]
    fn test_soh_after_calendar_5years() {
        let mut model = DegradationModel::new(DegradationChemistry::NmcGraphite, 50.0, 3.7);
        let cond = AgingConditions {
            temperature_c: 25.0,
            soc_pct: 50.0,
            ..Default::default()
        };
        model.step_calendar(365.0 * 5.0, &cond);
        let soh = model.compute_soh();
        assert!(
            soh > 0.5 && soh <= 1.0,
            "SoH after 5 years calendar should be in (0.5, 1.0], got {soh:.4}"
        );
    }

    #[test]
    fn test_rul_decreases_with_aging() {
        let mut model = DegradationModel::new(DegradationChemistry::NmcGraphite, 50.0, 3.7);
        let rul_initial = model.compute_rul_cycles();
        let cond = default_conditions();
        for _ in 0..200 {
            model.step_cycle(80.0, &cond);
        }
        let rul_after = model.compute_rul_cycles();
        assert!(
            rul_after < rul_initial,
            "RUL should decrease with aging: initial={rul_initial:.1}, after={rul_after:.1}"
        );
    }

    #[test]
    fn test_rul_positive_for_new_battery() {
        let model = DegradationModel::new(DegradationChemistry::NmcGraphite, 50.0, 3.7);
        let rul = model.compute_rul_cycles();
        assert!(rul > 0.0, "New battery should have positive RUL, got {rul}");
    }

    #[test]
    fn test_capacity_fade_breakdown_sums() {
        let mut model = DegradationModel::new(DegradationChemistry::NmcGraphite, 50.0, 3.7);
        let cond = default_conditions();
        model.step_calendar(180.0, &cond);
        for _ in 0..200 {
            model.step_cycle(80.0, &cond);
        }
        let fade = model.compute_capacity_fade();
        // calendar + cycle should be within 10% of total
        let sum = fade.calendar_loss_pct + fade.cycle_loss_pct;
        let diff = (sum - fade.total_loss_pct).abs();
        assert!(
            diff < fade.total_loss_pct * 0.15 + 0.01,
            "calendar({:.3}%) + cycle({:.3}%) ≈ total({:.3}%), diff={diff:.3}",
            fade.calendar_loss_pct,
            fade.cycle_loss_pct,
            fade.total_loss_pct
        );
    }

    #[test]
    fn test_remaining_life_days_reasonable() {
        let model = DegradationModel::new(DegradationChemistry::NmcGraphite, 50.0, 3.7);
        let days = model.estimate_remaining_life_days(1.0);
        assert!(
            days > 0.0,
            "Remaining life should be positive for new battery, got {days}"
        );
    }

    #[test]
    fn test_lto_less_degradation_than_nmc() {
        let mut nmc = DegradationModel::new(DegradationChemistry::NmcGraphite, 50.0, 3.7);
        let mut lto = DegradationModel::new(DegradationChemistry::NmcLto, 50.0, 2.3);
        let cond = default_conditions();

        for _ in 0..500 {
            nmc.step_cycle(80.0, &cond);
            lto.step_cycle(80.0, &cond);
        }
        nmc.step_calendar(365.0, &cond);
        lto.step_calendar(365.0, &cond);

        assert!(
            lto.compute_soh() > nmc.compute_soh(),
            "LTO should have higher SoH than NMC after same cycling: LTO={:.4}, NMC={:.4}",
            lto.compute_soh(),
            nmc.compute_soh()
        );
    }

    #[test]
    fn test_solid_state_minimal_degradation() {
        let mut ss = DegradationModel::new(DegradationChemistry::SolidState, 50.0, 3.8);
        let mut nmc = DegradationModel::new(DegradationChemistry::NmcGraphite, 50.0, 3.7);
        let cond = default_conditions();

        for _ in 0..1000 {
            ss.step_cycle(80.0, &cond);
            nmc.step_cycle(80.0, &cond);
        }
        ss.step_calendar(365.0, &cond);
        nmc.step_calendar(365.0, &cond);

        assert!(
            ss.compute_soh() > nmc.compute_soh(),
            "SolidState should age slower than NMC: SS={:.4}, NMC={:.4}",
            ss.compute_soh(),
            nmc.compute_soh()
        );
    }

    #[test]
    fn test_reset_restores_new_condition() {
        let mut model = DegradationModel::new(DegradationChemistry::NmcGraphite, 50.0, 3.7);
        let cond = default_conditions();
        for _ in 0..300 {
            model.step_cycle(80.0, &cond);
        }
        model.step_calendar(200.0, &cond);
        assert!(
            model.compute_soh() < 1.0,
            "Model should be degraded before reset"
        );

        model.reset();

        let soh = model.compute_soh();
        assert!(
            (soh - 1.0).abs() < 1e-9,
            "SoH should be 1.0 after reset, got {soh}"
        );
        assert_eq!(model.total_cycles, 0.0);
        assert_eq!(model.calendar_age_days, 0.0);
        assert!(model.temperature_history.is_empty());
    }

    // ── CycleCounterDoD tests ─────────────────────────────────────────────────

    #[test]
    fn test_cycle_counter_empty_initially() {
        let counter = CycleCounterDoD::new();
        assert_eq!(counter.count_cycles(), 0.0);
    }

    #[test]
    fn test_cycle_counter_detects_full_cycle() {
        let mut counter = CycleCounterDoD::new();
        // SoC goes 100 → 0 → 100 (one full cycle)
        counter.add_soc_point(100.0);
        counter.add_soc_point(50.0);
        counter.add_soc_point(0.0);
        counter.add_soc_point(50.0);
        counter.add_soc_point(100.0);
        // Should detect at least one cycle
        assert!(
            counter.count_cycles() >= 1.0,
            "Should detect at least one cycle, got {}",
            counter.count_cycles()
        );
    }

    #[test]
    fn test_resistance_growth_struct_relative() {
        let mut model = DegradationModel::new(DegradationChemistry::LfpGraphite, 75.0, 3.2);
        let cond = AgingConditions {
            temperature_c: 25.0,
            c_rate: 0.5,
            ..Default::default()
        };
        for _ in 0..200 {
            model.step_cycle(60.0, &cond);
        }
        let rg = model.compute_resistance_growth();
        assert!(
            rg.relative_resistance > 1.0,
            "Relative resistance should exceed 1.0 after cycling, got {:.4}",
            rg.relative_resistance
        );
        assert!(rg.total_resistance_ohm > model.initial_resistance_ohm);
    }
}
