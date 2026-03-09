//! V2G (Vehicle-to-Grid) battery degradation and second-life economic assessment.
//!
//! Combines V2G cycling degradation models with second-life stationary storage
//! economics to help fleet operators decide when to retire batteries from EV
//! service and repurpose them for grid storage.
//!
//! # Units
//! - Energy: kWh
//! - Power: kW
//! - Cost/Revenue: USD
//! - CO₂: kg
//! - Time: years (unless noted as hours `h`)
//!
//! # References
//! - Neubauer & Pesaran (2011) "The ability of battery second use strategies to
//!   impact plug-in electric vehicle prices", J. Power Sources.
//! - Heymans et al. (2014) "Economic analysis of second use EV batteries for
//!   residential energy storage", Energy Policy.
//! - Wang et al. (2016) "Cycle-life model for graphite-LiFePO4 cells", J. Power Sources.

use serde::{Deserialize, Serialize};

// ─── Chemistry ───────────────────────────────────────────────────────────────

/// Battery chemistry variants with differing degradation characteristics.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BatteryChemistry2 {
    /// NMC — higher energy density, moderate degradation rate.
    Nmc,
    /// LFP — lower energy density, lowest degradation rate, longest calendar life.
    Lfp,
    /// NCA — highest energy density, fastest degradation.
    Nca,
}

impl BatteryChemistry2 {
    /// CO₂ emitted during manufacturing \[kg CO₂/kWh\].
    pub fn manufacturing_co2_per_kwh(self) -> f64 {
        match self {
            Self::Lfp => 85.0,
            Self::Nmc => 100.0,
            Self::Nca => 110.0,
        }
    }

    /// Typical new-battery replacement cost \[$/kWh\].
    pub fn replacement_cost_per_kwh(self) -> f64 {
        match self {
            Self::Lfp => 120.0,
            Self::Nmc => 140.0,
            Self::Nca => 155.0,
        }
    }
}

// ─── EV Battery Profile ───────────────────────────────────────────────────────

/// Current state of an EV battery used in V2G service.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EvBatteryProfile {
    /// Unique vehicle identifier.
    pub vehicle_id: String,
    /// Cell chemistry.
    pub chemistry: BatteryChemistry2,
    /// Nameplate energy capacity when new \[kWh\].
    pub original_capacity_kwh: f64,
    /// State-of-Health: current capacity / original capacity \[dimensionless, 0–1\].
    pub current_soh: f64,
    /// State-of-Charge at this instant \[dimensionless, 0–1\].
    pub current_soc: f64,
    /// Total distance driven \[km\].
    pub odometer_km: f64,
    /// Time since manufacture \[years\].
    pub calendar_age_years: f64,
    /// Number of completed V2G discharge sessions.
    pub v2g_sessions: u64,
    /// Cumulative energy discharged in V2G service \[kWh\].
    pub total_v2g_energy_kwh: f64,
}

impl EvBatteryProfile {
    /// Usable capacity right now \[kWh\].
    pub fn usable_capacity_kwh(&self) -> f64 {
        self.original_capacity_kwh * self.current_soh
    }
}

// ─── Degradation Config ───────────────────────────────────────────────────────

/// Degradation model parameters for a given chemistry and operating profile.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct V2gDegradationConfig {
    /// Battery chemistry (used for defaults and CO₂ factors).
    pub chemistry: BatteryChemistry2,
    /// SoH at which the battery is considered end-of-life for EV traction \[dimensionless\].
    pub soh_eol_threshold: f64,
    /// Fractional SoH loss per equivalent full cycle \[1/cycle\].
    pub cycle_degradation_per_cycle: f64,
    /// Fractional SoH loss per calendar year \[1/year\].
    pub calendar_degradation_per_year: f64,
    /// Extra cycle-degradation multiplier when operating in V2G mode.
    /// Accounts for additional shallow cycling and thermal stress (typical 1.3×).
    pub v2g_penalty_factor: f64,
    /// Degradation multiplier at 100 % SoC; scales exponentially with SoC above 50 %.
    pub soc_stress_factor: f64,
}

impl V2gDegradationConfig {
    /// Sensible defaults for NMC chemistry.
    pub fn default_nmc() -> Self {
        Self {
            chemistry: BatteryChemistry2::Nmc,
            soh_eol_threshold: 0.80,
            cycle_degradation_per_cycle: 0.000_2,
            calendar_degradation_per_year: 0.020,
            v2g_penalty_factor: 1.30,
            soc_stress_factor: 1.50,
        }
    }

    /// Sensible defaults for LFP chemistry.
    pub fn default_lfp() -> Self {
        Self {
            chemistry: BatteryChemistry2::Lfp,
            soh_eol_threshold: 0.80,
            cycle_degradation_per_cycle: 0.000_08,
            calendar_degradation_per_year: 0.010,
            v2g_penalty_factor: 1.20,
            soc_stress_factor: 1.20,
        }
    }

    /// Sensible defaults for NCA chemistry.
    pub fn default_nca() -> Self {
        Self {
            chemistry: BatteryChemistry2::Nca,
            soh_eol_threshold: 0.80,
            cycle_degradation_per_cycle: 0.000_25,
            calendar_degradation_per_year: 0.025,
            v2g_penalty_factor: 1.35,
            soc_stress_factor: 1.60,
        }
    }

    /// SoC stress multiplier at a given average SoC percentage \[0–100\].
    ///
    /// Returns 1.0 for avg_soc_pct ≤ 50; scales up to `soc_stress_factor` at 100 %.
    fn soc_stress_multiplier(&self, avg_soc_pct: f64) -> f64 {
        if avg_soc_pct <= 50.0 {
            1.0
        } else {
            let normalised = (avg_soc_pct - 50.0) / 50.0; // 0..1
            self.soc_stress_factor.powf(normalised)
        }
    }
}

// ─── Second-Life Economics ────────────────────────────────────────────────────

/// Economic parameters for repurposing a used EV battery in stationary storage.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct V2gSecondLifeEconomics {
    /// Minimum SoH required for the battery to enter second-life service \[dimensionless\].
    pub second_life_capacity_threshold: f64,
    /// Remanufacturing / refurbishment cost \[$/kWh of remaining capacity\].
    pub remanufacturing_cost_per_kwh: f64,
    /// One-time installation cost \[USD\].
    pub installation_cost_usd: f64,
    /// Usable energy capacity in second-life service \[kWh\].
    pub second_life_capacity_kwh: f64,
    /// Annual grid revenue per kWh of installed capacity \[$/kWh/year\].
    pub grid_revenue_per_kwh_per_year: f64,
    /// Annual operating and maintenance cost \[$/year\].
    pub operating_cost_per_year: f64,
    /// Discount rate for NPV calculation \[dimensionless, e.g. 0.08 for 8 %\].
    pub discount_rate: f64,
}

impl V2gSecondLifeEconomics {
    /// Upfront capital cost for second-life deployment \[USD\].
    pub fn capex(&self) -> f64 {
        self.remanufacturing_cost_per_kwh * self.second_life_capacity_kwh
            + self.installation_cost_usd
    }

    /// Net annual cash flow from second-life service \[USD/year\].
    pub fn annual_net_revenue(&self) -> f64 {
        self.grid_revenue_per_kwh_per_year * self.second_life_capacity_kwh
            - self.operating_cost_per_year
    }
}

// ─── V2G Session Data ─────────────────────────────────────────────────────────

/// Recorded data for a single V2G discharge / charge session.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct V2gSessionData {
    /// Unique session identifier.
    pub session_id: String,
    /// Session duration \[h\].
    pub duration_h: f64,
    /// Energy delivered to the grid (discharge) \[kWh\].
    pub energy_discharged_kwh: f64,
    /// Energy drawn from the grid (recharge) \[kWh\].
    pub energy_charged_kwh: f64,
    /// Average State-of-Charge during the session \[%, 0–100\].
    pub avg_soc_pct: f64,
    /// Peak power during the session \[kW\].
    pub max_power_kw: f64,
    /// Grid revenue earned in this session \[USD\].
    pub revenue_usd: f64,
}

impl V2gSessionData {
    /// Equivalent full cycles represented by this session.
    ///
    /// Uses discharged energy divided by the notional full-cycle energy (rounded
    /// through the battery twice — once discharge, once recharge).
    pub fn equivalent_cycles(&self, original_capacity_kwh: f64) -> f64 {
        if original_capacity_kwh <= 0.0 {
            return 0.0;
        }
        // A full cycle = 1 full discharge + 1 full recharge = 2 × capacity
        (self.energy_discharged_kwh + self.energy_charged_kwh) / (2.0 * original_capacity_kwh)
    }

    /// Duration in years (for calendar degradation contribution).
    pub fn duration_years(&self) -> f64 {
        self.duration_h / 8_760.0
    }
}

// ─── Output Types ─────────────────────────────────────────────────────────────

/// Remaining Useful Life prediction for an EV battery under a given V2G schedule.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RulPrediction {
    /// Years until SoH drops below the EV end-of-life threshold.
    pub years_to_eol: f64,
    /// Years until SoH drops below the second-life entry threshold (SoH → stationary).
    pub years_to_second_life: f64,
    /// Projected SoH after 1 year at the given session rate.
    pub soh_at_1_year: f64,
    /// Projected SoH after 5 years at the given session rate.
    pub soh_at_5_years: f64,
    /// Estimated V2G sessions remaining before EV EOL.
    pub total_v2g_sessions_remaining: u64,
}

/// Comparison between operating a vehicle with and without V2G service.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct V2gComparison {
    /// Battery life (years) with no V2G participation.
    pub without_v2g_life_years: f64,
    /// Battery life (years) with V2G participation.
    pub with_v2g_life_years: f64,
    /// Total V2G revenue over battery life \[USD\].
    pub v2g_revenue_total_usd: f64,
    /// Battery life reduction attributable to V2G \[years\].
    pub life_reduction_years: f64,
    /// Net economic benefit of V2G (revenue − cost of accelerated replacement) \[USD\].
    pub net_v2g_benefit_usd: f64,
    /// Human-readable recommendation.
    pub recommendation: String,
}

/// Result of screening one battery in a fleet for second-life eligibility.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SecondLifeAssessment2 {
    /// Vehicle / battery identifier.
    pub vehicle_id: String,
    /// Current State-of-Health \[dimensionless\].
    pub current_soh: f64,
    /// Whether the battery meets the SoH threshold for second-life entry.
    pub second_life_eligible: bool,
    /// Estimated Net Present Value of the second-life project \[USD\].
    pub estimated_npv: f64,
    /// Expected years of viable second-life service.
    pub years_in_second_life: f64,
    /// CO₂ emissions avoided by reuse instead of new manufacturing \[kg CO₂\].
    pub co2_avoided_kg: f64,
}

/// Environmental impact metrics for second-life battery reuse.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnvironmentalMetrics {
    /// Greenhouse gas emissions avoided by reuse \[kg CO₂\].
    pub co2_avoided_kg: f64,
    /// Equivalent number of trees planted for one year of CO₂ sequestration.
    /// (1 tree ≈ 22 kg CO₂/year)
    pub equivalent_trees: f64,
    /// Energy that would have been consumed manufacturing a replacement battery \[kWh\].
    pub energy_saved_manufacturing_kwh: f64,
}

// ─── Main Analyzer ────────────────────────────────────────────────────────────

/// Combined V2G degradation tracker and second-life economic analyser.
///
/// # Example
/// ```
/// use oxigrid::battery::v2g_second_life::{
///     V2gSecondLifeAnalyzer, EvBatteryProfile, BatteryChemistry2,
///     V2gDegradationConfig, V2gSecondLifeEconomics, V2gSessionData,
/// };
///
/// let battery = EvBatteryProfile {
///     vehicle_id: "EV-001".to_string(),
///     chemistry: BatteryChemistry2::Lfp,
///     original_capacity_kwh: 60.0,
///     current_soh: 0.92,
///     current_soc: 0.70,
///     odometer_km: 45_000.0,
///     calendar_age_years: 3.0,
///     v2g_sessions: 120,
///     total_v2g_energy_kwh: 1_800.0,
/// };
/// let cfg = V2gDegradationConfig::default_lfp();
/// let econ = V2gSecondLifeEconomics {
///     second_life_capacity_threshold: 0.75,
///     remanufacturing_cost_per_kwh: 30.0,
///     installation_cost_usd: 2_000.0,
///     second_life_capacity_kwh: 45.0,
///     grid_revenue_per_kwh_per_year: 50.0,
///     operating_cost_per_year: 500.0,
///     discount_rate: 0.08,
/// };
/// let mut analyser = V2gSecondLifeAnalyzer::new(battery, cfg, econ);
/// let npv = analyser.second_life_npv();
/// assert!(npv.is_finite());
/// ```
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct V2gSecondLifeAnalyzer {
    /// Current EV battery state.
    pub battery: EvBatteryProfile,
    /// Degradation model parameters.
    pub degradation_config: V2gDegradationConfig,
    /// Second-life economic parameters.
    pub economics: V2gSecondLifeEconomics,
    /// Historical log of completed V2G sessions.
    session_log: Vec<V2gSessionData>,
}

impl V2gSecondLifeAnalyzer {
    /// Create a new analyser with the given battery profile, degradation model, and economics.
    pub fn new(
        battery: EvBatteryProfile,
        degradation_config: V2gDegradationConfig,
        economics: V2gSecondLifeEconomics,
    ) -> Self {
        Self {
            battery,
            degradation_config,
            economics,
            session_log: Vec::new(),
        }
    }

    // ── Session logging ───────────────────────────────────────────────────────

    /// Record a completed V2G session and update the battery's SoH accordingly.
    ///
    /// Degradation contributions:
    /// - **Cycle**: `δ_cycle = cycles × cycle_deg_per_cycle × v2g_penalty × soc_stress`
    /// - **Calendar**: `δ_cal = dt_years × cal_deg_per_year`
    pub fn update_degradation(&mut self, session: &V2gSessionData) {
        let cfg = &self.degradation_config;
        let capacity = self.battery.original_capacity_kwh;

        let cycles = session.equivalent_cycles(capacity);
        let soc_stress = cfg.soc_stress_multiplier(session.avg_soc_pct);

        let delta_cycle =
            cycles * cfg.cycle_degradation_per_cycle * cfg.v2g_penalty_factor * soc_stress;
        let delta_cal = session.duration_years() * cfg.calendar_degradation_per_year;

        self.battery.current_soh -= delta_cycle + delta_cal;
        self.battery.current_soh = self.battery.current_soh.max(0.0);

        self.battery.v2g_sessions += 1;
        self.battery.total_v2g_energy_kwh += session.energy_discharged_kwh;
        self.battery.calendar_age_years += session.duration_years();

        self.session_log.push(session.clone());
    }

    /// Read-only access to the session history.
    pub fn session_log(&self) -> &[V2gSessionData] {
        &self.session_log
    }

    // ── Remaining Useful Life ─────────────────────────────────────────────────

    /// Project the battery's State-of-Health trajectory and estimate remaining life.
    ///
    /// Uses a **linear + calendar + V2G cycle** degradation model anchored at the
    /// current SoH.  The annual degradation rate is computed from the configured
    /// parameters and the anticipated session frequency.
    ///
    /// # Arguments
    /// * `annual_v2g_sessions` — expected V2G sessions per year going forward.
    pub fn predict_remaining_useful_life(&self, annual_v2g_sessions: usize) -> RulPrediction {
        let cfg = &self.degradation_config;
        let capacity = self.battery.original_capacity_kwh;

        // Estimate average cycle depth per session from historical data.
        let avg_cycles_per_session = if self.session_log.is_empty() {
            // Default: each session is ~0.25 equivalent full cycles.
            0.25_f64
        } else {
            self.session_log
                .iter()
                .map(|s| s.equivalent_cycles(capacity))
                .sum::<f64>()
                / self.session_log.len() as f64
        };

        // Average SoC stress from history (or assume 60 % if no history).
        let avg_soc_pct = if self.session_log.is_empty() {
            60.0_f64
        } else {
            self.session_log.iter().map(|s| s.avg_soc_pct).sum::<f64>()
                / self.session_log.len() as f64
        };

        let soc_stress = cfg.soc_stress_multiplier(avg_soc_pct);
        let cycle_deg_annual = annual_v2g_sessions as f64
            * avg_cycles_per_session
            * cfg.cycle_degradation_per_cycle
            * cfg.v2g_penalty_factor
            * soc_stress;
        let annual_deg = cfg.calendar_degradation_per_year + cycle_deg_annual;

        // Avoid division by zero / infinite life predictions.
        let annual_deg = annual_deg.max(1e-9);

        let soh_now = self.battery.current_soh;

        // SoH at t years: soh(t) = soh_now - annual_deg × t  (linear approx)
        // Accelerating component: small quadratic term (5 % per year of extra stress)
        let accel = 0.005 * annual_deg;

        // soh(t) = soh_now - annual_deg*t - accel*t^2  → solve for threshold crossings.
        let solve_time_to_threshold = |threshold: f64| -> f64 {
            let diff = soh_now - threshold;
            if diff <= 0.0 {
                return 0.0; // already below threshold
            }
            if accel.abs() < 1e-12 {
                return diff / annual_deg;
            }
            // Quadratic: accel·t² + annual_deg·t - diff = 0
            // t = (-annual_deg + sqrt(annual_deg² + 4·accel·diff)) / (2·accel)
            let discriminant = annual_deg * annual_deg + 4.0 * accel * diff;
            if discriminant < 0.0 {
                return f64::INFINITY;
            }
            (-annual_deg + discriminant.sqrt()) / (2.0 * accel)
        };

        let years_to_eol = solve_time_to_threshold(cfg.soh_eol_threshold);
        let years_to_second_life =
            solve_time_to_threshold(self.economics.second_life_capacity_threshold);

        let soh_at = |t: f64| -> f64 { (soh_now - annual_deg * t - accel * t * t).max(0.0) };

        let total_v2g_sessions_remaining =
            (years_to_eol * annual_v2g_sessions as f64).round() as u64;

        RulPrediction {
            years_to_eol,
            years_to_second_life,
            soh_at_1_year: soh_at(1.0),
            soh_at_5_years: soh_at(5.0),
            total_v2g_sessions_remaining,
        }
    }

    // ── Second-Life NPV ───────────────────────────────────────────────────────

    /// Net Present Value of the second-life stationary storage project \[USD\].
    ///
    /// Cash flows:
    /// - Year 0: −CAPEX (remanufacturing + installation)
    /// - Years 1..n: +(annual_revenue − opex), discounted at `discount_rate`
    ///
    /// Second-life operation ends when the battery's SoH is projected to fall
    /// below **0.60** (stationary EOL), or after a maximum of 20 years.
    ///
    /// Returns the NPV; a positive value indicates economic viability.
    pub fn second_life_npv(&self) -> f64 {
        let econ = &self.economics;
        let capex = econ.capex();
        let annual_cf = econ.annual_net_revenue();
        let r = econ.discount_rate;

        // Estimate second-life duration from SoH degradation in stationary use.
        // In stationary service (no V2G penalty), only calendar + moderate cycling.
        let stationary_annual_deg = self.degradation_config.calendar_degradation_per_year
            + 52.0  // ~1 cycle/week
                * 0.10  // shallow cycles → ~0.1 EFC each
                * self.degradation_config.cycle_degradation_per_cycle;

        let stationary_annual_deg = stationary_annual_deg.max(1e-9);
        let soh_entry = econ.second_life_capacity_threshold;
        let soh_stationary_eol = 0.60_f64;

        let max_years = ((soh_entry - soh_stationary_eol) / stationary_annual_deg).clamp(0.0, 20.0);
        let n = max_years.ceil() as usize;

        let pv_revenues: f64 = (1..=n)
            .map(|t| annual_cf / (1.0_f64 + r).powi(t as i32))
            .sum();

        pv_revenues - capex
    }

    // ── V2G vs No-V2G Comparison ──────────────────────────────────────────────

    /// Compare battery economics with and without V2G participation.
    ///
    /// # Arguments
    /// * `years` — planning horizon \[years\].
    /// * `annual_sessions` — V2G sessions per year if participating.
    pub fn v2g_vs_no_v2g_comparison(&self, _years: f64, annual_sessions: usize) -> V2gComparison {
        let cfg = &self.degradation_config;
        let soh_now = self.battery.current_soh;
        let eol = cfg.soh_eol_threshold;

        // ── Scenario A: no V2G ────────────────────────────────────────────────
        // Only calendar degradation + normal driving cycles.
        // Assume 15,000 km/year, 6 km/kWh → ~2500 kWh/year discharged.
        let annual_drive_cycles = 15_000.0 / (6.0 * self.battery.original_capacity_kwh).max(1.0);
        let annual_deg_no_v2g = cfg.calendar_degradation_per_year
            + annual_drive_cycles * cfg.cycle_degradation_per_cycle;
        let annual_deg_no_v2g = annual_deg_no_v2g.max(1e-9);
        let life_no_v2g = ((soh_now - eol) / annual_deg_no_v2g).max(0.0);

        // ── Scenario B: with V2G ──────────────────────────────────────────────
        let rul = self.predict_remaining_useful_life(annual_sessions);
        let life_v2g = rul.years_to_eol;

        // Total V2G revenue over the battery life.
        let avg_revenue_per_session = if self.session_log.is_empty() {
            10.0_f64 // default $10/session
        } else {
            self.session_log.iter().map(|s| s.revenue_usd).sum::<f64>()
                / self.session_log.len() as f64
        };
        let total_revenue = life_v2g * annual_sessions as f64 * avg_revenue_per_session;

        // Cost of accelerated battery replacement.
        let replacement_cost_per_kwh = self.battery.chemistry.replacement_cost_per_kwh();
        let early_replacement_cost = (life_no_v2g - life_v2g).max(0.0) / life_no_v2g.max(1e-9)
            * replacement_cost_per_kwh
            * self.battery.original_capacity_kwh;

        // Second-life NPV benefit (both scenarios may qualify; we count the delta).
        let second_life_benefit =
            if self.battery.current_soh >= self.economics.second_life_capacity_threshold {
                self.second_life_npv().max(0.0)
            } else {
                0.0
            };

        let net_v2g_benefit = total_revenue - early_replacement_cost + second_life_benefit;
        let life_reduction = (life_no_v2g - life_v2g).max(0.0);

        let recommendation = if net_v2g_benefit > 0.0 {
            "V2G beneficial".to_string()
        } else {
            "V2G not recommended".to_string()
        };

        V2gComparison {
            without_v2g_life_years: life_no_v2g,
            with_v2g_life_years: life_v2g,
            v2g_revenue_total_usd: total_revenue,
            life_reduction_years: life_reduction,
            net_v2g_benefit_usd: net_v2g_benefit,
            recommendation,
        }
    }

    // ── Fleet Second-Life Screening ───────────────────────────────────────────

    /// Evaluate a fleet of batteries for second-life eligibility and sort by NPV.
    ///
    /// Each entry in `fleet_batteries` is a tuple of `(EvBatteryProfile, V2gDegradationConfig)`.
    /// The caller's `V2gSecondLifeEconomics` template is applied to each battery,
    /// adjusting `second_life_capacity_kwh` to the actual remaining capacity.
    pub fn fleet_second_life_screening(
        &self,
        fleet_batteries: &[(EvBatteryProfile, V2gDegradationConfig)],
    ) -> Vec<SecondLifeAssessment2> {
        let threshold = self.economics.second_life_capacity_threshold;

        let mut assessments: Vec<SecondLifeAssessment2> = fleet_batteries
            .iter()
            .map(|(batt, cfg)| {
                let eligible = batt.current_soh >= threshold;
                let second_life_cap = batt.usable_capacity_kwh();

                // Build a tailored economics object for this battery.
                let mut econ = self.economics.clone();
                econ.second_life_capacity_kwh = second_life_cap;

                let analyser = V2gSecondLifeAnalyzer::new(batt.clone(), cfg.clone(), econ);
                let npv = if eligible {
                    analyser.second_life_npv()
                } else {
                    // Estimate a negative NPV proportional to SoH shortfall.
                    let shortfall = threshold - batt.current_soh;
                    -shortfall * 10_000.0
                };

                // Second-life duration estimate.
                let stationary_annual_deg = cfg.calendar_degradation_per_year
                    + 52.0 * 0.10 * cfg.cycle_degradation_per_cycle;
                let stationary_annual_deg = stationary_annual_deg.max(1e-9);
                let years_in_second_life = if eligible {
                    ((batt.current_soh - 0.60) / stationary_annual_deg).clamp(0.0, 20.0)
                } else {
                    0.0
                };

                let co2_avoided_kg = if eligible {
                    second_life_cap * batt.chemistry.manufacturing_co2_per_kwh()
                } else {
                    0.0
                };

                SecondLifeAssessment2 {
                    vehicle_id: batt.vehicle_id.clone(),
                    current_soh: batt.current_soh,
                    second_life_eligible: eligible,
                    estimated_npv: npv,
                    years_in_second_life,
                    co2_avoided_kg,
                }
            })
            .collect();

        // Sort descending by estimated NPV.
        assessments.sort_by(|a, b| {
            b.estimated_npv
                .partial_cmp(&a.estimated_npv)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        assessments
    }

    // ── Optimal V2G Dispatch ──────────────────────────────────────────────────

    /// Compute an optimal 24-hour V2G dispatch schedule \[kW per hour\].
    ///
    /// Positive values = discharge to grid; zero = idle / charging.
    ///
    /// # Arguments
    /// * `grid_prices` — 24 electricity prices \[$/kWh\].
    /// * `soc_min` — minimum allowable SoC \[0–1\].
    /// * `soc_max` — maximum allowable SoC \[0–1\].
    ///
    /// # Returns
    /// 24-element vector of dispatch powers \[kW\].  Length is clamped to 24 regardless
    /// of input length.
    pub fn optimal_v2g_dispatch(
        &self,
        grid_prices: &[f64],
        soc_min: f64,
        soc_max: f64,
    ) -> Vec<f64> {
        let cfg = &self.degradation_config;
        let capacity = self.battery.original_capacity_kwh;

        // Degradation cost [$/kWh discharged]
        let replacement_cost = self.battery.chemistry.replacement_cost_per_kwh();
        let deg_cost_per_kwh =
            cfg.cycle_degradation_per_cycle * cfg.v2g_penalty_factor * replacement_cost;

        // Maximum continuous discharge power: C/2 rate.
        let max_power_kw = capacity / 2.0;
        // Maximum energy dispatchable above soc_min [kWh].
        let usable_energy = capacity * self.battery.current_soh * (soc_max - soc_min).max(0.0);

        let hours = grid_prices.len().min(24);
        let mut dispatch = vec![0.0_f64; 24];

        // Simple merit-order dispatch: rank hours by (price - deg_cost) descending.
        let mut ranked: Vec<(usize, f64)> = grid_prices[..hours]
            .iter()
            .enumerate()
            .map(|(i, &p)| (i, p - deg_cost_per_kwh))
            .collect();
        ranked.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        let mut energy_remaining = usable_energy;

        for (hour, net_revenue_rate) in &ranked {
            if *net_revenue_rate <= 0.0 {
                break; // no remaining profitable hours
            }
            if energy_remaining <= 0.0 {
                break;
            }
            let dispatchable = energy_remaining.min(max_power_kw); // 1 h slot → kWh = kW
            dispatch[*hour] = dispatchable;
            energy_remaining -= dispatchable;
        }

        dispatch
    }

    // ── Environmental Benefit ─────────────────────────────────────────────────

    /// Estimate the environmental benefit of second-life battery reuse.
    ///
    /// CO₂ is avoided because a new battery does not need to be manufactured to
    /// replace the capacity provided by the repurposed second-life battery.
    ///
    /// Embodied energy for new manufacturing: ≈ 100 kWh/kWh capacity (IPCC estimate).
    pub fn environmental_benefit(&self) -> EnvironmentalMetrics {
        let capacity = self.economics.second_life_capacity_kwh;
        let co2_per_kwh = self.battery.chemistry.manufacturing_co2_per_kwh();

        let co2_avoided_kg = capacity * co2_per_kwh;
        // 1 mature tree sequesters ≈ 22 kg CO₂/year.
        let equivalent_trees = co2_avoided_kg / 22.0;
        // Embodied energy for Li-ion manufacturing ≈ 100 kWh_primary/kWh_capacity.
        let energy_saved_manufacturing_kwh = capacity * 100.0;

        EnvironmentalMetrics {
            co2_avoided_kg,
            equivalent_trees,
            energy_saved_manufacturing_kwh,
        }
    }
}

// ─── Convenience accessor ─────────────────────────────────────────────────────

impl EvBatteryProfile {
    /// Chemistry reference (same as field, but useful in generic contexts).
    pub fn chemistry(&self) -> BatteryChemistry2 {
        self.chemistry
    }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_nmc_battery(soh: f64) -> EvBatteryProfile {
        EvBatteryProfile {
            vehicle_id: "TEST-NMC".to_string(),
            chemistry: BatteryChemistry2::Nmc,
            original_capacity_kwh: 75.0,
            current_soh: soh,
            current_soc: 0.60,
            odometer_km: 50_000.0,
            calendar_age_years: 3.0,
            v2g_sessions: 50,
            total_v2g_energy_kwh: 750.0,
        }
    }

    fn make_lfp_battery(soh: f64) -> EvBatteryProfile {
        EvBatteryProfile {
            vehicle_id: "TEST-LFP".to_string(),
            chemistry: BatteryChemistry2::Lfp,
            original_capacity_kwh: 60.0,
            current_soh: soh,
            current_soc: 0.70,
            odometer_km: 40_000.0,
            calendar_age_years: 2.5,
            v2g_sessions: 80,
            total_v2g_energy_kwh: 960.0,
        }
    }

    fn default_economics(capacity_kwh: f64) -> V2gSecondLifeEconomics {
        V2gSecondLifeEconomics {
            second_life_capacity_threshold: 0.75,
            remanufacturing_cost_per_kwh: 30.0,
            installation_cost_usd: 2_000.0,
            second_life_capacity_kwh: capacity_kwh * 0.80, // ~SoH at entry
            grid_revenue_per_kwh_per_year: 55.0,
            operating_cost_per_year: 500.0,
            discount_rate: 0.08,
        }
    }

    fn make_session(
        avg_soc_pct: f64,
        discharged: f64,
        charged: f64,
        revenue: f64,
    ) -> V2gSessionData {
        V2gSessionData {
            session_id: "S1".to_string(),
            duration_h: 4.0,
            energy_discharged_kwh: discharged,
            energy_charged_kwh: charged,
            avg_soc_pct,
            max_power_kw: 11.0,
            revenue_usd: revenue,
        }
    }

    // ── Test 1: SoH decreases after a V2G session ────────────────────────────
    #[test]
    fn test_soh_decreases_after_session() {
        let batt = make_nmc_battery(0.95);
        let cfg = V2gDegradationConfig::default_nmc();
        let econ = default_economics(75.0);
        let mut analyser = V2gSecondLifeAnalyzer::new(batt, cfg, econ);

        let soh_before = analyser.battery.current_soh;
        let session = make_session(60.0, 15.0, 16.0, 12.0);
        analyser.update_degradation(&session);
        let soh_after = analyser.battery.current_soh;

        assert!(
            soh_after < soh_before,
            "SoH should decrease after a V2G session"
        );
    }

    // ── Test 2: High-SoC session degrades more than low-SoC session ─────────
    #[test]
    fn test_high_soc_degrades_more() {
        let cfg = V2gDegradationConfig::default_nmc();

        let run_session = |avg_soc: f64| {
            let batt = make_nmc_battery(0.95);
            let econ = default_economics(75.0);
            let mut analyser = V2gSecondLifeAnalyzer::new(batt, cfg.clone(), econ);
            let session = make_session(avg_soc, 15.0, 16.0, 12.0);
            let before = analyser.battery.current_soh;
            analyser.update_degradation(&session);
            before - analyser.battery.current_soh
        };

        let deg_low_soc = run_session(30.0); // below 50 % → no stress multiplier
        let deg_high_soc = run_session(90.0); // above 50 % → stress multiplier > 1

        assert!(
            deg_high_soc > deg_low_soc,
            "High-SoC session should cause more degradation: high={deg_high_soc:.6} low={deg_low_soc:.6}"
        );
    }

    // ── Test 3: V2G cycles degrade more than equivalent driving cycles ───────
    #[test]
    fn test_v2g_penalty_exceeds_driving() {
        let mut cfg_v2g = V2gDegradationConfig::default_nmc();
        let mut cfg_drive = cfg_v2g.clone();
        cfg_drive.v2g_penalty_factor = 1.0; // no V2G penalty (driving only)

        let run = |cfg: V2gDegradationConfig| {
            let batt = make_nmc_battery(0.95);
            let econ = default_economics(75.0);
            let mut analyser = V2gSecondLifeAnalyzer::new(batt, cfg, econ);
            let session = make_session(60.0, 20.0, 21.0, 15.0);
            let before = analyser.battery.current_soh;
            analyser.update_degradation(&session);
            before - analyser.battery.current_soh
        };

        cfg_v2g.v2g_penalty_factor = 1.3;
        let deg_v2g = run(cfg_v2g);
        let deg_drive = run(cfg_drive);

        assert!(
            deg_v2g > deg_drive,
            "V2G sessions should cause more degradation than equivalent driving: v2g={deg_v2g:.6} drive={deg_drive:.6}"
        );
    }

    // ── Test 4: RUL prediction yields positive years at SoH=0.85 ────────────
    #[test]
    fn test_rul_moderate_v2g() {
        let batt = make_nmc_battery(0.85);
        let cfg = V2gDegradationConfig::default_nmc();
        let econ = default_economics(75.0);
        let analyser = V2gSecondLifeAnalyzer::new(batt, cfg, econ);

        let rul = analyser.predict_remaining_useful_life(100); // 100 sessions/year

        assert!(
            rul.years_to_eol > 0.0,
            "Should have positive remaining life: {:.2} years",
            rul.years_to_eol
        );
        assert!(
            rul.soh_at_1_year < 0.85,
            "SoH should decrease over 1 year: {:.3}",
            rul.soh_at_1_year
        );
        assert!(
            rul.soh_at_5_years < rul.soh_at_1_year,
            "SoH should be lower at 5 years than at 1 year"
        );
    }

    // ── Test 5: Second-life NPV is positive for LFP with remaining life ─────
    #[test]
    fn test_second_life_npv_lfp_positive() {
        let batt = make_lfp_battery(0.82);
        let cfg = V2gDegradationConfig::default_lfp();
        // Generous economics: high revenue, low cost.
        let econ = V2gSecondLifeEconomics {
            second_life_capacity_threshold: 0.75,
            remanufacturing_cost_per_kwh: 20.0,
            installation_cost_usd: 1_000.0,
            second_life_capacity_kwh: 49.2, // 60 * 0.82
            grid_revenue_per_kwh_per_year: 60.0,
            operating_cost_per_year: 300.0,
            discount_rate: 0.06,
        };
        let analyser = V2gSecondLifeAnalyzer::new(batt, cfg, econ);
        let npv = analyser.second_life_npv();

        assert!(
            npv > 0.0,
            "LFP second-life NPV should be positive with favourable economics: {npv:.2}"
        );
    }

    // ── Test 6: V2G comparison — high revenue → beneficial recommendation ────
    #[test]
    fn test_v2g_comparison_high_revenue_beneficial() {
        let batt = make_nmc_battery(0.90);
        let cfg = V2gDegradationConfig::default_nmc();
        let mut econ = default_economics(75.0);
        // High grid revenue per session → V2G should be beneficial.
        econ.grid_revenue_per_kwh_per_year = 80.0;
        let mut analyser = V2gSecondLifeAnalyzer::new(batt, cfg, econ);

        // Seed session log with high-revenue sessions.
        for i in 0..10 {
            let session = V2gSessionData {
                session_id: format!("S{i}"),
                duration_h: 2.0,
                energy_discharged_kwh: 10.0,
                energy_charged_kwh: 11.0,
                avg_soc_pct: 55.0,
                max_power_kw: 11.0,
                revenue_usd: 25.0, // high revenue
            };
            analyser.update_degradation(&session);
        }

        let comparison = analyser.v2g_vs_no_v2g_comparison(10.0, 200);
        assert_eq!(
            comparison.recommendation, "V2G beneficial",
            "High-revenue V2G should be recommended. net_benefit={:.2}",
            comparison.net_v2g_benefit_usd
        );
    }

    // ── Test 7: Fleet screening sorted by NPV descending ────────────────────
    #[test]
    fn test_fleet_screening_sorted_by_npv() {
        let batt_good = EvBatteryProfile {
            vehicle_id: "FLEET-A".to_string(),
            chemistry: BatteryChemistry2::Lfp,
            original_capacity_kwh: 60.0,
            current_soh: 0.88, // good SoH → higher NPV
            current_soc: 0.70,
            odometer_km: 30_000.0,
            calendar_age_years: 2.0,
            v2g_sessions: 50,
            total_v2g_energy_kwh: 600.0,
        };
        let batt_poor = EvBatteryProfile {
            vehicle_id: "FLEET-B".to_string(),
            chemistry: BatteryChemistry2::Nmc,
            original_capacity_kwh: 75.0,
            current_soh: 0.70, // below threshold → negative NPV
            current_soc: 0.50,
            odometer_km: 120_000.0,
            calendar_age_years: 7.0,
            v2g_sessions: 300,
            total_v2g_energy_kwh: 4_500.0,
        };

        let cfg_lfp = V2gDegradationConfig::default_lfp();
        let cfg_nmc = V2gDegradationConfig::default_nmc();

        let reference_batt = make_lfp_battery(0.85);
        let econ = default_economics(60.0);
        let analyser = V2gSecondLifeAnalyzer::new(reference_batt, cfg_lfp.clone(), econ);

        let fleet = vec![
            (batt_poor.clone(), cfg_nmc.clone()),
            (batt_good.clone(), cfg_lfp.clone()),
        ];
        let results = analyser.fleet_second_life_screening(&fleet);

        assert_eq!(
            results.len(),
            2,
            "Should return assessment for each battery"
        );
        assert!(
            results[0].estimated_npv >= results[1].estimated_npv,
            "Results should be sorted by NPV descending: [{:.2}, {:.2}]",
            results[0].estimated_npv,
            results[1].estimated_npv
        );
        // The good battery (FLEET-A, LFP, SoH 0.88) should rank first.
        assert_eq!(
            results[0].vehicle_id, "FLEET-A",
            "Better battery should rank first"
        );
    }

    // ── Test 8: CO₂ avoided proportional to capacity ────────────────────────
    #[test]
    fn test_environmental_co2_proportional_to_capacity() {
        let make_analyser = |capacity: f64| {
            let mut batt = make_lfp_battery(0.82);
            batt.original_capacity_kwh = capacity;
            let cfg = V2gDegradationConfig::default_lfp();
            let econ = V2gSecondLifeEconomics {
                second_life_capacity_threshold: 0.75,
                remanufacturing_cost_per_kwh: 30.0,
                installation_cost_usd: 2_000.0,
                second_life_capacity_kwh: capacity * 0.82,
                grid_revenue_per_kwh_per_year: 50.0,
                operating_cost_per_year: 500.0,
                discount_rate: 0.08,
            };
            V2gSecondLifeAnalyzer::new(batt, cfg, econ)
        };

        let a = make_analyser(60.0);
        let b = make_analyser(120.0); // double capacity

        let metrics_a = a.environmental_benefit();
        let metrics_b = b.environmental_benefit();

        let ratio = metrics_b.co2_avoided_kg / metrics_a.co2_avoided_kg;
        assert!(
            (ratio - 2.0).abs() < 1e-9,
            "CO₂ avoided should double when capacity doubles: ratio={ratio:.6}"
        );
        assert!(
            metrics_a.equivalent_trees > 0.0,
            "Should report positive equivalent trees"
        );
    }

    // ── Test 9: Optimal dispatch only fires on profitable hours ─────────────
    #[test]
    fn test_optimal_dispatch_respects_degradation_threshold() {
        let batt = make_nmc_battery(0.90);
        let cfg = V2gDegradationConfig::default_nmc();
        let econ = default_economics(75.0);
        let analyser = V2gSecondLifeAnalyzer::new(batt, cfg, econ);

        // All prices below degradation cost → no dispatch.
        let low_prices = vec![0.01_f64; 24];
        let dispatch_low = analyser.optimal_v2g_dispatch(&low_prices, 0.20, 0.90);
        let total_low: f64 = dispatch_low.iter().sum();
        assert!(
            total_low < 1e-9,
            "No dispatch when prices are below degradation cost: total={total_low:.4}"
        );

        // High prices → dispatch in profitable hours.
        let high_prices = vec![0.50_f64; 24];
        let dispatch_high = analyser.optimal_v2g_dispatch(&high_prices, 0.20, 0.90);
        let total_high: f64 = dispatch_high.iter().sum();
        assert!(
            total_high > 0.0,
            "Dispatch expected when prices are high: total={total_high:.4}"
        );
    }

    // ── Test 10: Session log accumulates correctly ───────────────────────────
    #[test]
    fn test_session_log_accumulates() {
        let batt = make_nmc_battery(0.95);
        let cfg = V2gDegradationConfig::default_nmc();
        let econ = default_economics(75.0);
        let mut analyser = V2gSecondLifeAnalyzer::new(batt, cfg, econ);

        for i in 0..5 {
            let s = V2gSessionData {
                session_id: format!("S{i}"),
                duration_h: 1.0,
                energy_discharged_kwh: 5.0,
                energy_charged_kwh: 5.5,
                avg_soc_pct: 60.0,
                max_power_kw: 11.0,
                revenue_usd: 8.0,
            };
            analyser.update_degradation(&s);
        }

        assert_eq!(
            analyser.session_log().len(),
            5,
            "All 5 sessions should be logged"
        );
        assert_eq!(
            analyser.battery.v2g_sessions, 55,
            "v2g_sessions should increment"
        );
    }
}
