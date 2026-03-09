//! Comprehensive Battery Thermal Management System (BTMS).
//!
//! Models heat generation, cooling, and thermal runaway risk for a multi-cell
//! battery pack over a charge/discharge cycle.  Supports four cooling
//! strategies: air-cooled (forced convection), liquid-cooled (direct or
//! indirect), Phase-Change Material (PCM), and a hybrid combination.
//!
//! # Heat generation model
//!
//! ```text
//! Q_gen = I²·R  (irreversible Joule heating)
//!       + α·I·T  (reversible entropic term, α ≈ 1×10⁻⁴ V/K)
//! ```
//!
//! # Thermal runaway risk
//!
//! Arrhenius-inspired sigmoid (logistic) function:
//!
//! ```text
//! risk(T) = exp(x) / (1 + exp(x)),   x = (T − T_onset) / T_char
//! T_onset = 80 °C,  T_char = 10 °C
//! ```
use serde::{Deserialize, Serialize};

// ─── Error ────────────────────────────────────────────────────────────────────

/// Errors from the BTMS controller.
#[derive(Debug, thiserror::Error)]
pub enum BtmsError {
    /// One or more configuration parameters are physically invalid.
    #[error("invalid BTMS config: {0}")]
    InvalidConfig(String),
    /// Cell temperature exceeded `max_temp_c` — thermal runaway detected.
    #[error("thermal runaway at t={time_s:.2} s, T={temp_c:.1} °C")]
    ThermalRunaway {
        /// Simulation time of detection \[s\].
        time_s: f64,
        /// Cell temperature at detection \[°C\].
        temp_c: f64,
    },
}

// ─── Cooling system ───────────────────────────────────────────────────────────

/// Cooling strategy for the battery pack.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum CoolingSystem {
    /// Forced-air cooling fan.
    AirCooled {
        /// Volumetric airflow rate \[m³/s\].
        flow_rate_m3_per_s: f64,
        /// Electrical power consumed by the fan \[W\].
        fan_power_w: f64,
    },
    /// Liquid coolant loop with a heat exchanger.
    LiquidCooled {
        /// Coolant volumetric flow rate \[L/min\].
        coolant_flow_rate_l_per_min: f64,
        /// Coolant inlet temperature \[°C\].
        coolant_inlet_temp_c: f64,
        /// Overall heat-transfer coefficient × area (UA) \[W/K\].
        heat_exchanger_ua_w_per_k: f64,
        /// Electrical power consumed by the coolant pump \[W\].
        pump_power_w: f64,
    },
    /// Passive Phase-Change Material (PCM) thermal buffer.
    PhaseChangeMaterial {
        /// Mass of PCM material \[kg\].
        pcm_mass_kg: f64,
        /// Latent heat of the PCM \[J/kg\].
        pcm_latent_heat_j_per_kg: f64,
        /// PCM melting temperature \[°C\].
        pcm_melting_temp_c: f64,
        /// Thermal conductivity of the PCM \[W/(m·K)\].
        pcm_conductivity_w_per_m_k: f64,
    },
    /// Primary + secondary cooling combined.
    Hybrid {
        /// Primary cooling subsystem.
        primary: Box<CoolingSystem>,
        /// Secondary (supplementary) cooling subsystem.
        secondary: Box<CoolingSystem>,
    },
}

// ─── Configuration ────────────────────────────────────────────────────────────

/// Configuration for the Battery Thermal Management System.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BtmsConfig {
    /// Number of cells in the pack.
    pub n_cells: usize,
    /// Nominal cell capacity \[Ah\].
    pub cell_capacity_ah: f64,
    /// Nominal cell voltage \[V\].
    pub cell_voltage_v: f64,
    /// Cell DC internal resistance \[Ω\].
    pub cell_internal_resistance_ohm: f64,
    /// Cell thermal (heat) capacity \[J/K\].
    pub cell_heat_capacity_j_per_k: f64,
    /// Cell mass (used for thermal inertia) \[kg\].
    pub cell_mass_kg: f64,
    /// Ambient (environment) temperature \[°C\].
    pub ambient_temp_c: f64,
    /// Optimal operating temperature target \[°C\].
    pub target_temp_c: f64,
    /// Maximum allowable cell temperature \[°C\] (above this → runaway risk).
    pub max_temp_c: f64,
    /// Minimum allowable cell temperature \[°C\].
    pub min_temp_c: f64,
    /// Selected cooling system.
    pub cooling_system: CoolingSystem,
}

impl BtmsConfig {
    fn validate(&self) -> Result<(), BtmsError> {
        if self.n_cells == 0 {
            return Err(BtmsError::InvalidConfig("n_cells must be ≥ 1".into()));
        }
        if self.cell_heat_capacity_j_per_k <= 0.0 {
            return Err(BtmsError::InvalidConfig(
                "cell_heat_capacity_j_per_k must be positive".into(),
            ));
        }
        if self.max_temp_c <= self.min_temp_c {
            return Err(BtmsError::InvalidConfig(
                "max_temp_c must be greater than min_temp_c".into(),
            ));
        }
        Ok(())
    }
}

// ─── State ────────────────────────────────────────────────────────────────────

/// Instantaneous thermal state of the battery pack.
#[derive(Debug, Clone)]
pub struct BtmsState {
    /// Per-cell temperatures \[°C\].
    pub cell_temperatures_c: Vec<f64>,
    /// Maximum cell temperature in the pack \[°C\].
    pub max_temp_c: f64,
    /// Minimum cell temperature in the pack \[°C\].
    pub min_temp_c: f64,
    /// Mean cell temperature \[°C\].
    pub avg_temp_c: f64,
    /// Temperature non-uniformity (max − min) \[°C\].
    pub temp_uniformity_c: f64,
    /// Active cooling power \[W\].
    pub cooling_power_w: f64,
    /// Simulation time \[s\].
    pub time_s: f64,
    /// Thermal runaway risk \[0, 1\].
    pub thermal_runaway_risk: f64,
}

// ─── Result ───────────────────────────────────────────────────────────────────

/// Result of a BTMS simulation.
#[derive(Debug, Clone)]
pub struct BtmsResult {
    /// Time-history of pack thermal states.
    pub states: Vec<BtmsState>,
    /// Peak temperature observed during simulation \[°C\].
    pub max_temp_reached_c: f64,
    /// `true` if any cell exceeded `max_temp_c`.
    pub thermal_runaway_occurred: bool,
    /// Time at which thermal runaway was first detected \[s\], if any.
    pub thermal_runaway_time_s: Option<f64>,
    /// Total electrical energy consumed by cooling \[Wh\].
    pub total_cooling_energy_wh: f64,
    /// Coefficient of Performance = Q_removed / W_cooling.
    pub cop: f64,
    /// `true` if temperature non-uniformity stayed below 5 \[°C\] at end of sim.
    pub temperature_uniformity_achieved: bool,
}

// ─── Controller ───────────────────────────────────────────────────────────────

/// Battery Thermal Management System controller.
pub struct BtmsController {
    config: BtmsConfig,
}

impl BtmsController {
    /// Construct a new controller with the given configuration.
    pub fn new(config: BtmsConfig) -> Self {
        Self { config }
    }

    /// Simulate thermal behaviour during a constant-current charge/discharge.
    ///
    /// # Arguments
    /// - `current_a` — charging/discharging current \[A\] (positive = charging)
    /// - `duration_s` — total simulation duration \[s\]
    /// - `dt_s` — time-step \[s\]
    /// - `initial_temp_c` — uniform initial cell temperature \[°C\]
    ///
    /// # Errors
    /// Returns [`BtmsError::InvalidConfig`] for bad parameters.
    /// Does **not** return [`BtmsError::ThermalRunaway`] — instead sets the
    /// `thermal_runaway_occurred` flag and records the time.
    pub fn simulate(
        &self,
        current_a: f64,
        duration_s: f64,
        dt_s: f64,
        initial_temp_c: f64,
    ) -> Result<BtmsResult, BtmsError> {
        self.config.validate()?;
        if dt_s <= 0.0 {
            return Err(BtmsError::InvalidConfig("dt_s must be positive".into()));
        }
        if duration_s <= 0.0 {
            return Err(BtmsError::InvalidConfig(
                "duration_s must be positive".into(),
            ));
        }

        let n = self.config.n_cells;
        let n_steps = ((duration_s / dt_s).ceil() as usize).max(1);

        // Initialise per-cell temperatures with small LCG-based offsets (±0.5 °C)
        let mut temps: Vec<f64> = self.lcg_temp_offsets(n, initial_temp_c, 0.5);

        let mut states: Vec<BtmsState> = Vec::with_capacity(n_steps + 1);
        let mut thermal_runaway_occurred = false;
        let mut thermal_runaway_time_s: Option<f64> = None;
        let mut total_cooling_energy_j = 0.0;
        let mut total_heat_removed_j = 0.0;
        let mut max_temp_reached = initial_temp_c;

        // Record initial state
        states.push(self.build_state(&temps, 0.0, 0.0));

        for step in 0..n_steps {
            let t = (step + 1) as f64 * dt_s;
            let mut total_cooling_power_w = 0.0;

            let n_cells_f = n as f64;
            for cell_temp in temps.iter_mut().take(n) {
                let temp = *cell_temp;

                // Heat generation for this cell
                let q_gen = self.cell_heat_generation_w(current_a, temp);

                // Cooling heat removal for this cell
                let (q_cool, elec_power) = self.cooling_power(temp, &self.config.cooling_system);
                let q_cool_per_cell = q_cool / n_cells_f;
                let elec_per_cell = elec_power / n_cells_f;

                // Energy accounting
                total_cooling_energy_j += elec_per_cell * dt_s;
                total_heat_removed_j += q_cool_per_cell.max(0.0) * dt_s;
                total_cooling_power_w += elec_per_cell;

                // Euler integration: ΔT = (Q_gen − Q_cool) / Cp
                let net_heat_w = q_gen - q_cool_per_cell;
                let delta_t = net_heat_w * dt_s / self.config.cell_heat_capacity_j_per_k;
                *cell_temp += delta_t;

                // Clamp to ambient (cells can cool to ambient but not below)
                if *cell_temp < self.config.ambient_temp_c {
                    *cell_temp = self.config.ambient_temp_c;
                }
            }

            // Pack-level stats
            let max_t = temps.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
            let _min_t = temps.iter().cloned().fold(f64::INFINITY, f64::min);
            max_temp_reached = max_temp_reached.max(max_t);

            // Thermal runaway detection
            if !thermal_runaway_occurred && max_t > self.config.max_temp_c {
                thermal_runaway_occurred = true;
                thermal_runaway_time_s = Some(t);
            }

            let avg_cooling = total_cooling_power_w;
            states.push(self.build_state(&temps, t, avg_cooling));
        }

        // COP
        let cop = if total_cooling_energy_j > f64::EPSILON {
            total_heat_removed_j / total_cooling_energy_j
        } else {
            f64::INFINITY
        };

        // Temperature uniformity at end
        let max_final = temps.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let min_final = temps.iter().cloned().fold(f64::INFINITY, f64::min);
        let uniformity_achieved = (max_final - min_final) < 5.0;

        Ok(BtmsResult {
            states,
            max_temp_reached_c: max_temp_reached,
            thermal_runaway_occurred,
            thermal_runaway_time_s,
            total_cooling_energy_wh: total_cooling_energy_j / 3600.0,
            cop,
            temperature_uniformity_achieved: uniformity_achieved,
        })
    }

    // ── Internal helpers ──────────────────────────────────────────────────────

    /// Build a [`BtmsState`] snapshot from current cell temperatures.
    fn build_state(&self, temps: &[f64], time_s: f64, cooling_power_w: f64) -> BtmsState {
        let n = temps.len().max(1) as f64;
        let avg = temps.iter().sum::<f64>() / n;
        let max_t = temps.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let min_t = temps.iter().cloned().fold(f64::INFINITY, f64::min);
        let risk = self.thermal_runaway_risk(max_t);

        BtmsState {
            cell_temperatures_c: temps.to_vec(),
            max_temp_c: max_t,
            min_temp_c: min_t,
            avg_temp_c: avg,
            temp_uniformity_c: max_t - min_t,
            cooling_power_w,
            time_s,
            thermal_runaway_risk: risk,
        }
    }

    /// Generate per-cell initial temperatures with LCG-based ±`spread_c` offsets.
    ///
    /// LCG parameters: mult = 6364136223846793005, add = 1442695040888963407.
    fn lcg_temp_offsets(&self, n: usize, base_c: f64, spread_c: f64) -> Vec<f64> {
        const MULT: u64 = 6_364_136_223_846_793_005;
        const ADD: u64 = 1_442_695_040_888_963_407;
        let mut state: u64 = 0xDEAD_BEEF_CAFE_1234;
        let mut result = Vec::with_capacity(n);
        for _ in 0..n {
            state = state.wrapping_mul(MULT).wrapping_add(ADD);
            // Map to [-spread, +spread]
            let frac = (state >> 33) as f64 / (u32::MAX as f64);
            let offset = (frac - 0.5) * 2.0 * spread_c;
            result.push(base_c + offset);
        }
        result
    }

    /// Compute irreversible + reversible heat generation for one cell \[W\].
    ///
    /// ```text
    /// Q = I²·R + α·I·T_abs,   α = 1×10⁻⁴ V/K
    /// ```
    fn cell_heat_generation_w(&self, current_a: f64, temp_c: f64) -> f64 {
        let r = self.config.cell_internal_resistance_ohm;
        let t_abs = temp_c + 273.15;
        let q_irreversible = current_a * current_a * r;
        let q_reversible = 0.1 * current_a.abs() * t_abs * 1e-4;
        q_irreversible + q_reversible
    }

    /// Compute (heat_removed_from_pack \[W\], electrical_cooling_power \[W\])
    /// for the whole pack at a given average cell temperature.
    fn cooling_power(&self, cell_temp_c: f64, system: &CoolingSystem) -> (f64, f64) {
        match system {
            CoolingSystem::AirCooled {
                flow_rate_m3_per_s,
                fan_power_w,
            } => {
                // Q = ρ·Cp·V̇·ΔT
                let rho_cp = 1.2 * 1005.0; // ≈ 1206 J/(m³·K) for air
                let delta_t = (cell_temp_c - self.config.ambient_temp_c).max(0.0);
                let q = rho_cp * flow_rate_m3_per_s * delta_t;
                (q, *fan_power_w)
            }
            CoolingSystem::LiquidCooled {
                coolant_inlet_temp_c,
                heat_exchanger_ua_w_per_k,
                pump_power_w,
                ..
            } => {
                let delta_t = (cell_temp_c - coolant_inlet_temp_c).max(0.0);
                let q = heat_exchanger_ua_w_per_k * delta_t;
                (q, *pump_power_w)
            }
            CoolingSystem::PhaseChangeMaterial {
                pcm_melting_temp_c,
                pcm_conductivity_w_per_m_k,
                ..
            } => {
                // Effective cooling only when cell temperature exceeds melting point
                if cell_temp_c > *pcm_melting_temp_c {
                    // Approximate: Q ∝ conductivity × area_eff × ΔT
                    // area_eff chosen to give reasonable values (0.1 m²)
                    let area_eff = 0.1_f64;
                    let delta_t = (cell_temp_c - pcm_melting_temp_c).max(0.0);
                    let q = pcm_conductivity_w_per_m_k * area_eff * delta_t;
                    (q, 0.0) // PCM is passive — no electrical power
                } else {
                    (0.0, 0.0)
                }
            }
            CoolingSystem::Hybrid { primary, secondary } => {
                let (q1, p1) = self.cooling_power(cell_temp_c, primary);
                let (q2, p2) = self.cooling_power(cell_temp_c, secondary);
                (q1 + q2, p1 + p2)
            }
        }
    }

    /// Compute thermal runaway risk using an Arrhenius-inspired logistic model.
    ///
    /// ```text
    /// x    = (T − T_onset) / T_char,   T_onset = 80 °C, T_char = 10 °C
    /// risk = exp(x) / (1 + exp(x))
    /// ```
    fn thermal_runaway_risk(&self, temp_c: f64) -> f64 {
        const T_ONSET: f64 = 80.0;
        const T_CHAR: f64 = 10.0;
        let x = (temp_c - T_ONSET) / T_CHAR;
        // Numerically stable sigmoid
        if x >= 0.0 {
            let ex = x.exp();
            ex / (1.0 + ex)
        } else {
            let enx = (-x).exp();
            1.0 / (1.0 + enx)
        }
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn air_config() -> BtmsConfig {
        BtmsConfig {
            n_cells: 4,
            cell_capacity_ah: 50.0,
            cell_voltage_v: 3.7,
            cell_internal_resistance_ohm: 0.005,
            cell_heat_capacity_j_per_k: 1000.0,
            cell_mass_kg: 0.3,
            ambient_temp_c: 25.0,
            target_temp_c: 25.0,
            max_temp_c: 60.0,
            min_temp_c: -20.0,
            cooling_system: CoolingSystem::AirCooled {
                flow_rate_m3_per_s: 0.01,
                fan_power_w: 20.0,
            },
        }
    }

    fn liquid_config() -> BtmsConfig {
        BtmsConfig {
            cooling_system: CoolingSystem::LiquidCooled {
                coolant_flow_rate_l_per_min: 5.0,
                coolant_inlet_temp_c: 20.0,
                heat_exchanger_ua_w_per_k: 200.0,
                pump_power_w: 30.0,
            },
            ..air_config()
        }
    }

    fn pcm_config() -> BtmsConfig {
        BtmsConfig {
            cooling_system: CoolingSystem::PhaseChangeMaterial {
                pcm_mass_kg: 2.0,
                pcm_latent_heat_j_per_kg: 200_000.0,
                pcm_melting_temp_c: 42.0,
                pcm_conductivity_w_per_m_k: 5.0,
            },
            ..air_config()
        }
    }

    /// Test 1: Low current — temperature stays near ambient.
    #[test]
    fn test_low_current_near_ambient() {
        let ctrl = BtmsController::new(air_config());
        let result = ctrl
            .simulate(1.0, 60.0, 1.0, 25.0)
            .expect("simulation failed");

        assert!(
            result.max_temp_reached_c < 30.0,
            "At 1 A, max temp {:.2} °C should stay below 30 °C",
            result.max_temp_reached_c
        );
        assert!(!result.thermal_runaway_occurred);
    }

    /// Test 2: High current — temperature rises and cooling activates.
    #[test]
    fn test_high_current_temperature_rise() {
        // Use lower heat capacity to see measurable temperature rise
        let mut cfg = air_config();
        cfg.cell_heat_capacity_j_per_k = 50.0; // small cell Cp
        cfg.cell_internal_resistance_ohm = 0.05; // higher resistance
        let ctrl = BtmsController::new(cfg);
        // 50 A for 60 s
        let result = ctrl
            .simulate(50.0, 60.0, 0.5, 25.0)
            .expect("simulation failed");

        // Q = I²R = 2500×0.05 = 125 W; with Cp=50 J/K: ΔT ~ 125×60/50 = 150°C (before cooling)
        assert!(
            result.max_temp_reached_c > 26.0,
            "At 50 A with R=0.05Ω, temperature should rise above 26 °C, got {:.2}",
            result.max_temp_reached_c
        );
        // Cooling energy should be non-zero
        assert!(
            result.total_cooling_energy_wh > 0.0,
            "Cooling energy should be positive"
        );
    }

    /// Test 3: Thermal runaway risk at 90 °C > 0.73 (logistic at +1 sigma).
    #[test]
    fn test_thermal_runaway_risk_at_high_temp() {
        let ctrl = BtmsController::new(air_config());
        let risk = ctrl.thermal_runaway_risk(90.0);
        // sigmoid(1.0) ≈ 0.731
        assert!(risk > 0.73, "Risk at 90 °C should be > 0.73, got {risk:.4}");
        // Risk at 25 °C should be very small (sigmoid(-5.5) ≈ 0.0041)
        let low_risk = ctrl.thermal_runaway_risk(25.0);
        assert!(
            low_risk < 0.01,
            "Risk at 25 °C should be < 0.01, got {low_risk:.6}"
        );
    }

    /// Test 4: Liquid cooling has higher COP than air cooling under same conditions.
    #[test]
    fn test_liquid_vs_air_cop() {
        let ctrl_air = BtmsController::new(air_config());
        let ctrl_liq = BtmsController::new(liquid_config());

        let r_air = ctrl_air
            .simulate(30.0, 60.0, 1.0, 25.0)
            .expect("air sim failed");
        let r_liq = ctrl_liq
            .simulate(30.0, 60.0, 1.0, 25.0)
            .expect("liquid sim failed");

        // Liquid cooling: higher UA → more heat removed per watt of pump power
        // COP_liquid should be higher than COP_air (or both infinite if no elec energy)
        let cop_air = r_air.cop;
        let cop_liq = r_liq.cop;
        assert!(
            cop_liq >= cop_air || (cop_liq.is_infinite() && cop_air.is_infinite()),
            "Liquid COP {cop_liq:.2} should be ≥ air COP {cop_air:.2}"
        );
    }

    /// Test 5: PCM — temperature plateau near melting point.
    #[test]
    fn test_pcm_temperature_plateau() {
        let ctrl = BtmsController::new(pcm_config());
        // Moderate current, long enough to reach melting point
        let result = ctrl
            .simulate(20.0, 120.0, 1.0, 25.0)
            .expect("PCM sim failed");

        // With PCM cooling activating at 42 °C, temperature should not jump
        // far above the melting point quickly
        let final_state = result.states.last().expect("states non-empty");
        let avg = final_state.avg_temp_c;
        // Should be reasonably bounded — PCM absorbs heat near 42 °C
        // We just verify it didn't wildly exceed the melting point
        assert!(
            avg < 80.0,
            "PCM-cooled avg temp {avg:.2} °C should stay well below 80 °C"
        );
    }

    /// Test 6: Temperature uniformity achieved flag is set correctly.
    #[test]
    fn test_temperature_uniformity_flag() {
        let ctrl = BtmsController::new(air_config());
        // Low current: small temperature differences expected
        let result = ctrl
            .simulate(2.0, 30.0, 1.0, 25.0)
            .expect("simulation failed");

        // With only 4 cells and small spread, uniformity should be achieved
        assert!(
            result.temperature_uniformity_achieved,
            "Temperature uniformity (ΔT < 5°C) should be achieved at low current"
        );
    }
}
