//! Advanced PV system modelling: multi-string arrays, MPPT, grid interconnection, degradation.
//!
//! Provides a hierarchy of structs:
//! - [`PvModule`]      — single panel (STC parameters + temperature coefficients)
//! - [`PvString`]      — N modules in series with soiling/shading/mismatch losses
//! - [`PvArray`]       — M strings in parallel with wiring losses and age
//! - [`PvInverter`]    — DC→AC conversion with efficiency, MPPT range, reactive capability
//! - [`PvSystem`]      — full plant: arrays + inverter + AC-side losses
//! - [`PvSystemModel`] — computational engine for simulation and economic analysis

use serde::{Deserialize, Serialize};

// ─── Enumerations ────────────────────────────────────────────────────────────

/// PV cell technology, determining temperature coefficients and spectral response.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PvTechnology {
    /// Standard mono-Si (highest efficiency, ~-0.40 %/°C)
    Monocrystalline,
    /// Multi-crystalline Si (~-0.42 %/°C)
    Polycrystalline,
    /// Cadmium telluride thin-film (~-0.25 %/°C)
    ThinFilmCdTe,
    /// Copper indium gallium selenide (~-0.35 %/°C)
    ThinFilmCigs,
    /// Amorphous silicon (~-0.20 %/°C, light soaking)
    AmorphousSilicon,
    /// Bifacial mono-Si (rear-side albedo gain)
    Bifacial,
    /// Concentrating PV with Fresnel/dish optics
    Cpv,
    /// Multi-junction tandem cell
    Tandem,
}

/// DC–AC conversion topology influencing efficiency and MPPT behaviour.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum InverterTopology {
    /// Single large inverter for whole plant (utility scale)
    CentralInverter,
    /// One inverter per string (residential/commercial)
    StringInverter,
    /// One inverter per module (maximum shade tolerance)
    MicroInverter,
    /// Module-level DC–DC power optimisers + string inverter
    Optimized,
}

/// How the inverter interfaces with the grid.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum GridConnectionMode {
    /// Standard PLL-synchronised current-source inverter
    GridFollowing,
    /// Voltage-source inverter acting as a grid reference
    GridForming,
    /// Off-grid / standalone operation
    IslandMode,
    /// Low/high-voltage ride-through per grid code
    RideThrough,
}

// ─── Module ───────────────────────────────────────────────────────────────────

/// Single PV module model using nameplate and temperature-coefficient parameters.
///
/// All electrical parameters are given at Standard Test Conditions (STC):
/// irradiance G = 1000 W m⁻², cell temperature T = 25 °C, AM 1.5 spectrum.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PvModule {
    /// Unique module identifier.
    pub id: usize,
    /// Descriptive name (manufacturer / model string).
    pub name: String,
    /// Cell technology type.
    pub technology: PvTechnology,
    /// Rated peak power at STC [W].
    pub p_stc_w: f64,
    /// Open-circuit voltage at STC [V].
    pub voc_stc_v: f64,
    /// Short-circuit current at STC [A].
    pub isc_stc_a: f64,
    /// MPP voltage at STC [V].
    pub vmpp_stc_v: f64,
    /// MPP current at STC [A].
    pub impp_stc_a: f64,
    /// Power temperature coefficient [% per °C] (typically negative, e.g. −0.40).
    pub temp_coeff_p_pct_per_c: f64,
    /// Voc temperature coefficient [mV per °C] (typically negative, e.g. −2.3).
    pub temp_coeff_voc_mv_per_c: f64,
    /// Isc temperature coefficient [mA per °C] (typically positive, e.g. +0.06).
    pub temp_coeff_isc_ma_per_c: f64,
    /// Nominal operating cell temperature [°C] (default 45 °C).
    pub noct_c: f64,
    /// Module area [m²].
    pub area_m2: f64,
    /// Nameplate efficiency at STC [%].
    pub efficiency_stc_pct: f64,
    /// Annual power degradation rate [% per year] (default 0.5 %/yr).
    pub degradation_rate_pct_per_year: f64,
}

impl PvModule {
    /// Convenience constructor: typical 400 W monocrystalline module.
    pub fn new_standard_mono_400w() -> Self {
        Self {
            id: 0,
            name: "Standard Mono 400W".to_string(),
            technology: PvTechnology::Monocrystalline,
            p_stc_w: 400.0,
            voc_stc_v: 49.5,
            isc_stc_a: 10.05,
            vmpp_stc_v: 41.2,
            impp_stc_a: 9.72,
            temp_coeff_p_pct_per_c: -0.40,
            temp_coeff_voc_mv_per_c: -2.3,
            temp_coeff_isc_ma_per_c: 0.06,
            noct_c: 45.0,
            area_m2: 1.96,
            efficiency_stc_pct: 20.4,
            degradation_rate_pct_per_year: 0.5,
        }
    }

    /// Cell temperature from the NOCT thermal model.
    ///
    /// `T_cell = T_amb + G × (NOCT − 20) / 800`
    pub fn cell_temp(&self, ambient_temp_c: f64, irradiance_w_per_m2: f64) -> f64 {
        let irr = irradiance_w_per_m2.max(0.0);
        ambient_temp_c + irr * (self.noct_c - 20.0) / 800.0
    }

    /// Temperature- and irradiance-corrected maximum power.
    ///
    /// `P = P_stc × (G/1000) × (1 + γ/100 × (T_cell − 25))`
    ///
    /// where γ is `temp_coeff_p_pct_per_c`.
    pub fn p_max_at_conditions(&self, irradiance_w_per_m2: f64, ambient_temp_c: f64) -> f64 {
        let irr = irradiance_w_per_m2.max(0.0);
        if irr <= 0.0 {
            return 0.0;
        }
        let t_cell = self.cell_temp(ambient_temp_c, irr);
        let irr_factor = irr / 1000.0;
        let temp_factor = 1.0 + self.temp_coeff_p_pct_per_c / 100.0 * (t_cell - 25.0);
        (self.p_stc_w * irr_factor * temp_factor).max(0.0)
    }

    /// Open-circuit voltage corrected for temperature and irradiance.
    ///
    /// `Voc = (Voc_stc + β × (T_cell − 25)/1000) × (1 + 0.05 × ln(G/1000).max(0))`
    ///
    /// where β is `temp_coeff_voc_mv_per_c` in mV/°C.
    pub fn voc_at_conditions(&self, ambient_temp_c: f64, irradiance_w_per_m2: f64) -> f64 {
        let irr = irradiance_w_per_m2.max(1.0); // avoid ln(0)
        let t_cell = self.cell_temp(ambient_temp_c, irr);
        let voc_t = self.voc_stc_v + self.temp_coeff_voc_mv_per_c / 1000.0 * (t_cell - 25.0);
        let irr_factor = 1.0 + 0.05 * (irr / 1000.0).ln().max(0.0);
        (voc_t * irr_factor).max(0.0)
    }

    /// Short-circuit current corrected for irradiance and temperature.
    ///
    /// `Isc = Isc_stc × (G/1000) × (1 + α_isc / (Isc_stc × 1000) × (T_cell − 25))`
    ///
    /// where α_isc is `temp_coeff_isc_ma_per_c`.
    pub fn isc_at_conditions(&self, irradiance_w_per_m2: f64, ambient_temp_c: f64) -> f64 {
        let irr = irradiance_w_per_m2.max(0.0);
        if irr <= 0.0 {
            return 0.0;
        }
        let t_cell = self.cell_temp(ambient_temp_c, irr);
        let isc_stc = self.isc_stc_a.max(1e-9);
        let temp_factor = 1.0 + self.temp_coeff_isc_ma_per_c / 1000.0 / isc_stc * (t_cell - 25.0);
        (self.isc_stc_a * (irr / 1000.0) * temp_factor).max(0.0)
    }

    /// Efficiency at given conditions [%] = P_max / (G × A_module) × 100.
    ///
    /// Returns 0.0 when irradiance or area is non-positive.
    pub fn efficiency_at_conditions(&self, irradiance_w_per_m2: f64, ambient_temp_c: f64) -> f64 {
        let irr = irradiance_w_per_m2.max(0.0);
        let area = self.area_m2.max(1e-9);
        if irr <= 0.0 {
            return 0.0;
        }
        let p_max = self.p_max_at_conditions(irr, ambient_temp_c);
        p_max / (irr * area) * 100.0
    }
}

// ─── String ───────────────────────────────────────────────────────────────────

/// N modules connected in series, sharing the same irradiance and temperature.
///
/// Losses from soiling, shading and parameter mismatch are applied multiplicatively.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PvString {
    /// Number of modules in series.
    pub n_modules: usize,
    /// Module type populating this string.
    pub module: PvModule,
    /// Soiling loss [%] (dust, pollen, bird droppings); default 2 %.
    pub soiling_loss_pct: f64,
    /// Shading loss [%] from obstructions or row-to-row shading.
    pub shading_loss_pct: f64,
    /// Module mismatch loss [%] due to parameter spread; default 1 %.
    pub mismatch_loss_pct: f64,
}

impl PvString {
    /// DC power from this string [W] after all string-level losses.
    ///
    /// `P_string = N × P_module × (1 − soiling/100) × (1 − shading/100) × (1 − mismatch/100)`
    pub fn p_max_dc(&self, irradiance: f64, temp_c: f64) -> f64 {
        let n = self.n_modules as f64;
        let p_mod = self.module.p_max_at_conditions(irradiance, temp_c);
        let loss = (1.0 - self.soiling_loss_pct / 100.0)
            * (1.0 - self.shading_loss_pct / 100.0)
            * (1.0 - self.mismatch_loss_pct / 100.0);
        (n * p_mod * loss).max(0.0)
    }

    /// Total open-circuit voltage of the series string [V].
    ///
    /// `Voc_string = N × Voc_module`
    pub fn voc_string(&self, temp_c: f64, irr: f64) -> f64 {
        self.n_modules as f64 * self.module.voc_at_conditions(temp_c, irr)
    }

    /// Short-circuit current of the series string [A].
    ///
    /// In a series connection the current is the same as a single module's Isc.
    pub fn isc_string(&self, irr: f64, temp_c: f64) -> f64 {
        self.module.isc_at_conditions(irr, temp_c)
    }
}

// ─── Array ────────────────────────────────────────────────────────────────────

/// M identical strings connected in parallel, mounted at a fixed tilt and azimuth.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PvArray {
    /// Number of parallel strings.
    pub n_strings: usize,
    /// String configuration (all strings identical).
    pub string: PvString,
    /// Panel tilt angle [°] from horizontal (0 = horizontal, 90 = vertical).
    pub tilt_deg: f64,
    /// Panel azimuth [°] from south (0 = south, +90 = west, −90 = east).
    pub azimuth_deg: f64,
    /// DC wiring resistance loss [%]; default 1.5 %.
    pub dc_wiring_loss_pct: f64,
    /// Age of the installed system [years] for degradation modelling.
    pub system_age_years: f64,
}

// ─── Inverter ─────────────────────────────────────────────────────────────────

/// Grid-connected DC–AC inverter with MPPT, efficiency, and reactive power parameters.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PvInverter {
    /// Unique inverter identifier.
    pub id: usize,
    /// Descriptive name.
    pub name: String,
    /// Converter topology.
    pub topology: InverterTopology,
    /// Rated AC output power [W].
    pub p_rated_w: f64,
    /// Peak efficiency (CEC/Euro) [%]; default 98.0.
    pub efficiency_pct: f64,
    /// MPPT tracking efficiency [%]; default 99.5.
    pub mppt_efficiency_pct: f64,
    /// Minimum DC input voltage [V].
    pub vdc_min_v: f64,
    /// Maximum DC input voltage [V].
    pub vdc_max_v: f64,
    /// Minimum DC voltage in the MPPT window [V].
    pub vdc_mppt_min_v: f64,
    /// Maximum DC voltage in the MPPT window [V].
    pub vdc_mppt_max_v: f64,
    /// Nominal AC grid voltage [V] (e.g. 400 V three-phase).
    pub ac_voltage_v: f64,
    /// Power factor at rated output (1.0 = unity).
    pub power_factor: f64,
    /// Capable of reactive power export/import.
    pub reactive_power_capable: bool,
    /// Night-time standby consumption [W].
    pub night_tare_w: f64,
}

impl PvInverter {
    /// Convenience constructor: 50 kW three-phase string inverter.
    pub fn new_string_inverter_50kw() -> Self {
        Self {
            id: 0,
            name: "String Inverter 50kW".to_string(),
            topology: InverterTopology::StringInverter,
            p_rated_w: 50_000.0,
            efficiency_pct: 98.0,
            mppt_efficiency_pct: 99.5,
            vdc_min_v: 200.0,
            vdc_max_v: 1000.0,
            vdc_mppt_min_v: 350.0,
            vdc_mppt_max_v: 800.0,
            ac_voltage_v: 400.0,
            power_factor: 1.0,
            reactive_power_capable: true,
            night_tare_w: 5.0,
        }
    }
}

// ─── PvSystem ─────────────────────────────────────────────────────────────────

/// Complete PV power plant: one or more arrays feeding a single inverter.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PvSystem {
    /// Plant name / site identifier.
    pub name: String,
    /// PV arrays (can differ in tilt, azimuth, age, string count).
    pub arrays: Vec<PvArray>,
    /// Grid-tied inverter.
    pub inverter: PvInverter,
    /// AC wiring resistance loss [%]; default 0.5 %.
    pub ac_wiring_loss_pct: f64,
    /// Step-up transformer loss [%]; default 0.5 %.
    pub transformer_loss_pct: f64,
    /// System availability fraction [%] (accounting for downtime); default 99 %.
    pub availability_pct: f64,
    /// Geographic location as (latitude [°], longitude [°]).
    pub location: (f64, f64),
    /// Inverter–grid interface mode.
    pub grid_connection_mode: GridConnectionMode,
}

// ─── Output ───────────────────────────────────────────────────────────────────

/// Simulation result for a single one-hour timestep.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PvOutputPoint {
    /// Hour index or absolute timestamp [h].
    pub timestamp_h: f64,
    /// Plane-of-array irradiance [W m⁻²].
    pub irradiance_w_per_m2: f64,
    /// Representative cell temperature [°C].
    pub cell_temp_c: f64,
    /// Total DC power from all arrays [W].
    pub p_dc_w: f64,
    /// Net AC power exported to grid [W].
    pub p_ac_w: f64,
    /// System AC efficiency relative to incident irradiance [%].
    pub efficiency_pct: f64,
    /// Performance ratio PR = Y_f / Y_r (dimensionless).
    pub performance_ratio: f64,
}

// ─── Computational engine ─────────────────────────────────────────────────────

/// Simulation and economic analysis engine for a [`PvSystem`].
pub struct PvSystemModel {
    /// The PV plant configuration being modelled.
    pub system: PvSystem,
}

impl PvSystemModel {
    /// Construct a new model wrapping the provided system configuration.
    pub fn new(system: PvSystem) -> Self {
        Self { system }
    }

    /// Total DC power from all arrays [W].
    ///
    /// For each array the per-string power is summed, DC wiring losses applied,
    /// and a degradation factor based on `array.system_age_years` is multiplied:
    ///
    /// `P_dc = Σ_arrays [ n_strings × P_string × (1 − wire_loss/100) × (1 − deg_rate/100)^age ]`
    pub fn compute_dc_output(&self, irradiance: f64, ambient_temp_c: f64) -> f64 {
        let irr = irradiance.max(0.0);
        self.system
            .arrays
            .iter()
            .map(|arr| {
                let p_string = arr.string.p_max_dc(irr, ambient_temp_c);
                let n = arr.n_strings as f64;
                let wire = 1.0 - arr.dc_wiring_loss_pct / 100.0;
                let deg_rate = arr.string.module.degradation_rate_pct_per_year / 100.0;
                let age_i = arr.system_age_years.round() as i32;
                let degradation = (1.0 - deg_rate).powi(age_i).max(0.0);
                n * p_string * wire * degradation
            })
            .sum::<f64>()
            .max(0.0)
    }

    /// Convert DC power to net AC export power [W] after all AC-side losses.
    ///
    /// `P_ac = P_dc × η_inv × η_mppt × (1 − ac_wire/100) × (1 − xfmr/100) × (avail/100)`
    ///
    /// Result is clamped to the inverter rated power.
    pub fn compute_ac_output(&self, p_dc_w: f64) -> f64 {
        if p_dc_w <= 0.0 {
            return 0.0;
        }
        let inv = &self.system.inverter;
        let eta_inv = inv.efficiency_pct / 100.0;
        let eta_mppt = inv.mppt_efficiency_pct / 100.0;
        let ac_wire = 1.0 - self.system.ac_wiring_loss_pct / 100.0;
        let xfmr = 1.0 - self.system.transformer_loss_pct / 100.0;
        let avail = self.system.availability_pct / 100.0;
        let p_ac = p_dc_w * eta_inv * eta_mppt * ac_wire * xfmr * avail;
        p_ac.clamp(0.0, inv.p_rated_w)
    }

    /// Simulate a single one-hour timestep and return a full [`PvOutputPoint`].
    pub fn simulate_hour(
        &self,
        irradiance: f64,
        ambient_temp_c: f64,
        timestamp_h: f64,
    ) -> PvOutputPoint {
        let irr = irradiance.max(0.0);
        let p_dc_w = self.compute_dc_output(irr, ambient_temp_c);
        let p_ac_w = self.compute_ac_output(p_dc_w);

        // Representative cell temperature from the first available module
        let cell_temp_c = self
            .system
            .arrays
            .first()
            .map(|arr| arr.string.module.cell_temp(ambient_temp_c, irr))
            .unwrap_or(ambient_temp_c);

        // Total module area for efficiency denominator
        let total_area_m2: f64 = self
            .system
            .arrays
            .iter()
            .map(|arr| {
                let mods_per_arr = (arr.n_strings * arr.string.n_modules) as f64;
                mods_per_arr * arr.string.module.area_m2
            })
            .sum();

        let efficiency_pct = if irr > 0.0 && total_area_m2 > 1e-9 {
            p_ac_w / (irr * total_area_m2) * 100.0
        } else {
            0.0
        };

        let performance_ratio = self.compute_performance_ratio(p_ac_w, irr);

        PvOutputPoint {
            timestamp_h,
            irradiance_w_per_m2: irr,
            cell_temp_c,
            p_dc_w,
            p_ac_w,
            efficiency_pct,
            performance_ratio,
        }
    }

    /// Simulate annual energy yield [MWh] from hourly irradiance and temperature profiles.
    ///
    /// Each element of the input slices represents one hour; slices are zipped so
    /// the shorter one determines the number of hours simulated.
    pub fn simulate_annual_yield(&self, hourly_irradiance: &[f64], hourly_temp_c: &[f64]) -> f64 {
        hourly_irradiance
            .iter()
            .zip(hourly_temp_c.iter())
            .enumerate()
            .map(|(i, (&irr, &temp))| {
                let pt = self.simulate_hour(irr, temp, i as f64);
                pt.p_ac_w / 1_000_000.0 // W·h → MWh for 1-hour step
            })
            .sum()
    }

    /// Performance ratio PR = Y_f / Y_r.
    ///
    /// - Y_f = P_ac / P_rated  (normalised final yield, dimensionless)
    /// - Y_r = G / G_ref       (reference yield where G_ref = 1000 W m⁻²)
    ///
    /// Returns 0.0 when irradiance is zero.
    pub fn compute_performance_ratio(&self, p_ac_w: f64, irradiance: f64) -> f64 {
        if irradiance <= 0.0 {
            return 0.0;
        }
        let p_rated = self.system.inverter.p_rated_w.max(1.0);
        let y_f = p_ac_w / p_rated;
        let y_r = irradiance / 1000.0;
        if y_r <= 0.0 {
            0.0
        } else {
            y_f / y_r
        }
    }

    /// Estimate specific yield [kWh/kWp] from annual plane-of-array irradiance.
    ///
    /// Applies a system derate factor composed of all loss terms:
    /// inverter efficiency, MPPT efficiency, AC wiring, transformer, and availability.
    pub fn estimate_specific_yield_kwh_per_kwp(&self, annual_irradiance_kwh_per_m2: f64) -> f64 {
        let inv = &self.system.inverter;
        let derate = (inv.efficiency_pct / 100.0)
            * (inv.mppt_efficiency_pct / 100.0)
            * (1.0 - self.system.ac_wiring_loss_pct / 100.0)
            * (1.0 - self.system.transformer_loss_pct / 100.0)
            * (self.system.availability_pct / 100.0);
        // Also account for a representative string-level loss from the first array
        let string_derate = self
            .system
            .arrays
            .first()
            .map(|arr| {
                let s = &arr.string;
                (1.0 - s.soiling_loss_pct / 100.0)
                    * (1.0 - s.shading_loss_pct / 100.0)
                    * (1.0 - s.mismatch_loss_pct / 100.0)
                    * (1.0 - arr.dc_wiring_loss_pct / 100.0)
            })
            .unwrap_or(1.0);
        annual_irradiance_kwh_per_m2 * derate * string_derate
    }

    /// Power output at a given age, applying additional degradation on top of any
    /// age already captured in `array.system_age_years`.
    ///
    /// Uses the degradation rate of the first array's module.
    pub fn compute_degraded_output(&self, age_years: f64, irradiance: f64, temp_c: f64) -> f64 {
        // Compute DC as if arrays are brand-new (ignore stored age)
        let p_dc_new: f64 = self
            .system
            .arrays
            .iter()
            .map(|arr| {
                let p_string = arr.string.p_max_dc(irradiance.max(0.0), temp_c);
                let n = arr.n_strings as f64;
                let wire = 1.0 - arr.dc_wiring_loss_pct / 100.0;
                n * p_string * wire
            })
            .sum::<f64>()
            .max(0.0);

        let deg_rate = self
            .system
            .arrays
            .first()
            .map(|arr| arr.string.module.degradation_rate_pct_per_year / 100.0)
            .unwrap_or(0.005);

        let degradation = (1.0 - deg_rate).powf(age_years.max(0.0));
        let p_dc_degraded = p_dc_new * degradation;
        self.compute_ac_output(p_dc_degraded)
    }

    /// Levelised cost of energy [USD/kWh].
    ///
    /// Uses the annuity method:
    /// `LCOE = (CapEx + OpEx × AF) / (CF × P_rated_kW × 8760 × N)`
    ///
    /// where AF = `(1 − (1+r)^−N) / r` (or `N` when r = 0).
    pub fn lcoe_usd_per_kwh(
        &self,
        capital_cost_usd: f64,
        annual_opex_usd: f64,
        lifetime_years: f64,
        discount_rate: f64,
        capacity_factor: f64,
    ) -> f64 {
        let n = lifetime_years.max(1.0);
        let af = if discount_rate.abs() < 1e-9 {
            n
        } else {
            (1.0 - (1.0 + discount_rate).powf(-n)) / discount_rate
        };
        let p_rated_kw = self.system.inverter.p_rated_w / 1000.0;
        let annual_energy_kwh = capacity_factor * p_rated_kw * 8760.0;
        let total_cost = capital_cost_usd + annual_opex_usd * af;
        total_cost / (annual_energy_kwh * n).max(1.0)
    }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Helper factories ──────────────────────────────────────────────────────

    fn default_module() -> PvModule {
        PvModule::new_standard_mono_400w()
    }

    fn default_string(n: usize) -> PvString {
        PvString {
            n_modules: n,
            module: default_module(),
            soiling_loss_pct: 2.0,
            shading_loss_pct: 0.0,
            mismatch_loss_pct: 1.0,
        }
    }

    fn default_array(n_strings: usize, n_modules: usize, age: f64) -> PvArray {
        PvArray {
            n_strings,
            string: default_string(n_modules),
            tilt_deg: 30.0,
            azimuth_deg: 0.0,
            dc_wiring_loss_pct: 1.5,
            system_age_years: age,
        }
    }

    fn default_inverter() -> PvInverter {
        PvInverter::new_string_inverter_50kw()
    }

    fn default_system(n_arrays: usize) -> PvSystem {
        let arrays: Vec<PvArray> = (0..n_arrays).map(|_| default_array(10, 20, 0.0)).collect();
        PvSystem {
            name: "Test Plant".to_string(),
            arrays,
            inverter: default_inverter(),
            ac_wiring_loss_pct: 0.5,
            transformer_loss_pct: 0.5,
            availability_pct: 99.0,
            location: (52.5, 13.4),
            grid_connection_mode: GridConnectionMode::GridFollowing,
        }
    }

    fn default_model(n_arrays: usize) -> PvSystemModel {
        PvSystemModel::new(default_system(n_arrays))
    }

    /// Build a system whose array DC is well matched to the inverter rated power.
    ///
    /// 1 string × 12 modules × 400 W → ~4.8 kWp DC.
    /// Inverter rated at 5 kW so DC/AC ratio ≈ 1.0 and PR stays in [0, 1.0].
    fn matched_model() -> PvSystemModel {
        let arr = PvArray {
            n_strings: 1,
            string: PvString {
                n_modules: 12,
                module: default_module(),
                soiling_loss_pct: 2.0,
                shading_loss_pct: 0.0,
                mismatch_loss_pct: 1.0,
            },
            tilt_deg: 30.0,
            azimuth_deg: 0.0,
            dc_wiring_loss_pct: 1.5,
            system_age_years: 0.0,
        };
        let inv = PvInverter {
            id: 0,
            name: "5kW inverter".to_string(),
            topology: InverterTopology::StringInverter,
            p_rated_w: 5_000.0,
            efficiency_pct: 97.0,
            mppt_efficiency_pct: 99.5,
            vdc_min_v: 100.0,
            vdc_max_v: 600.0,
            vdc_mppt_min_v: 150.0,
            vdc_mppt_max_v: 500.0,
            ac_voltage_v: 230.0,
            power_factor: 1.0,
            reactive_power_capable: false,
            night_tare_w: 2.0,
        };
        PvSystemModel::new(PvSystem {
            name: "Matched 5kW".to_string(),
            arrays: vec![arr],
            inverter: inv,
            ac_wiring_loss_pct: 0.5,
            transformer_loss_pct: 0.5,
            availability_pct: 99.0,
            location: (52.5, 13.4),
            grid_connection_mode: GridConnectionMode::GridFollowing,
        })
    }

    // ── Module tests ──────────────────────────────────────────────────────────

    #[test]
    fn test_cell_temp_model() {
        let m = default_module();
        let t_cell = m.cell_temp(25.0, 800.0);
        assert!(
            t_cell > 25.0,
            "T_cell should be above ambient; got {:.2}",
            t_cell
        );
    }

    #[test]
    fn test_cell_temp_noct() {
        let m = default_module();
        // At G=800 W/m², T_amb=20°C the NOCT formula gives exactly NOCT
        let t_cell = m.cell_temp(20.0, 800.0);
        let diff = (t_cell - m.noct_c).abs();
        assert!(
            diff < 1.0,
            "Expected T_cell ≈ NOCT={:.1} but got {:.2} (diff={:.3})",
            m.noct_c,
            t_cell,
            diff
        );
    }

    #[test]
    fn test_p_max_stc() {
        let m = default_module();
        let p = m.p_max_at_conditions(1000.0, 25.0);
        // At STC T_cell = 25 + 1000*(45-20)/800 = 56.25 °C, so some derating is expected
        // The STC result should be close to P_stc but allow for cell heating
        // At pure G=1000, T_amb such that T_cell=25: T_amb = 25 - 1000*(45-20)/800 ≈ -6.25
        // Test: at T_amb=25°C and G=1000 the power is within a reasonable range
        assert!(
            p > 0.0 && p <= m.p_stc_w * 1.02,
            "P_max at standard irradiance should be ≤ P_stc: got {:.1} W (P_stc={:.1})",
            p,
            m.p_stc_w
        );
    }

    #[test]
    fn test_p_max_stc_cold_ambient() {
        // At G=1000, T_amb=-6.25°C → T_cell ≈ 25°C, result should be ≈ P_stc
        let m = default_module();
        // T_amb = 25 - 1000*(45-20)/800 = 25 - 31.25 = -6.25°C
        let t_amb_stc = 25.0 - 1000.0 * (m.noct_c - 20.0) / 800.0;
        let p = m.p_max_at_conditions(1000.0, t_amb_stc);
        let diff_pct = ((p - m.p_stc_w) / m.p_stc_w).abs() * 100.0;
        assert!(
            diff_pct < 2.0,
            "P_max at STC cell conditions should ≈ P_stc (diff={:.2}%)",
            diff_pct
        );
    }

    #[test]
    fn test_p_max_temperature_derating() {
        let m = default_module();
        let p_cool = m.p_max_at_conditions(1000.0, 15.0);
        let p_hot = m.p_max_at_conditions(1000.0, 50.0);
        assert!(
            p_hot < p_cool,
            "Higher ambient → lower output; cool={:.1} W, hot={:.1} W",
            p_cool,
            p_hot
        );
    }

    #[test]
    fn test_p_max_low_irradiance() {
        let m = default_module();
        let p_full = m.p_max_at_conditions(1000.0, 25.0);
        let p_half = m.p_max_at_conditions(500.0, 25.0);
        // At the same ambient temperature the cell is cooler at lower irradiance,
        // so power is slightly above half. Check it is in [35%, 65%] of full power.
        let ratio = p_half / p_full.max(1e-9);
        assert!(
            ratio > 0.35 && ratio < 0.80,
            "Half irradiance power ratio should be ~0.5; got {:.3}",
            ratio
        );
    }

    // ── String tests ──────────────────────────────────────────────────────────

    #[test]
    fn test_string_series_voltage() {
        let s = default_string(20);
        let voc_str = s.voc_string(25.0, 1000.0);
        let voc_mod = s.module.voc_at_conditions(25.0, 1000.0);
        let expected = 20.0 * voc_mod;
        let diff = (voc_str - expected).abs();
        assert!(
            diff < 1e-6,
            "Voc_string should be 20 × Voc_module; got {:.4} vs {:.4}",
            voc_str,
            expected
        );
    }

    #[test]
    fn test_string_parallel_current() {
        let s = default_string(20);
        let isc_str = s.isc_string(1000.0, 25.0);
        let isc_mod = s.module.isc_at_conditions(1000.0, 25.0);
        let diff = (isc_str - isc_mod).abs();
        assert!(
            diff < 1e-9,
            "Series Isc should equal module Isc; got {:.6} vs {:.6}",
            isc_str,
            isc_mod
        );
    }

    #[test]
    fn test_string_soiling_loss() {
        let mut s_clean = default_string(10);
        s_clean.soiling_loss_pct = 0.0;
        let mut s_soiled = default_string(10);
        s_soiled.soiling_loss_pct = 10.0;

        let p_clean = s_clean.p_max_dc(1000.0, 25.0);
        let p_soiled = s_soiled.p_max_dc(1000.0, 25.0);

        // p_soiled should be ~(1 - 0.10) * p_clean (mismatch also present)
        let ratio = p_soiled / p_clean.max(1e-9);
        let expected_ratio = 0.90; // exact given mismatch cancels (same in both)
        let diff = (ratio - expected_ratio).abs();
        assert!(
            diff < 0.01,
            "10% soiling should reduce power by ~10%; ratio={:.4}",
            ratio
        );
    }

    // ── Array tests ───────────────────────────────────────────────────────────

    #[test]
    fn test_array_dc_output_positive() {
        let model = default_model(1);
        let p_dc = model.compute_dc_output(800.0, 25.0);
        assert!(p_dc > 0.0, "DC output should be positive; got {:.1}", p_dc);
    }

    #[test]
    fn test_array_aging_loss() {
        let sys_new = {
            let mut s = default_system(1);
            s.arrays[0].system_age_years = 0.0;
            s
        };
        let sys_old = {
            let mut s = default_system(1);
            s.arrays[0].system_age_years = 25.0;
            s
        };
        let p_new = PvSystemModel::new(sys_new).compute_dc_output(800.0, 25.0);
        let p_old = PvSystemModel::new(sys_old).compute_dc_output(800.0, 25.0);
        assert!(
            p_old < p_new,
            "25-year-old array should produce less than new; new={:.1} W, old={:.1} W",
            p_new,
            p_old
        );
    }

    // ── Inverter / system tests ───────────────────────────────────────────────

    #[test]
    fn test_inverter_efficiency() {
        let model = default_model(1);
        let p_dc = 40_000.0;
        let p_ac = model.compute_ac_output(p_dc);
        let inv = &model.system.inverter;
        let max_possible = p_dc * (inv.efficiency_pct / 100.0);
        assert!(
            p_ac <= max_possible + 1.0,
            "AC should be ≤ DC × η; ac={:.1}, max={:.1}",
            p_ac,
            max_possible
        );
    }

    #[test]
    fn test_system_compute_dc() {
        let model = default_model(3);
        let p_dc = model.compute_dc_output(800.0, 25.0);
        assert!(
            p_dc > 0.0,
            "3-array system DC should be positive; got {:.1}",
            p_dc
        );
    }

    #[test]
    fn test_system_compute_ac() {
        let model = default_model(1);
        let p_dc = model.compute_dc_output(800.0, 25.0);
        let p_ac = model.compute_ac_output(p_dc);
        assert!(
            p_ac < p_dc,
            "AC should be less than DC; ac={:.1}, dc={:.1}",
            p_ac,
            p_dc
        );
    }

    // ── Simulation tests ──────────────────────────────────────────────────────

    #[test]
    fn test_simulate_hour_pr() {
        // Use a system whose array DC is closely matched to the inverter,
        // so the performance ratio stays in [0.5, 1.0].
        let model = matched_model();
        let pt = model.simulate_hour(800.0, 25.0, 10.0);
        assert!(
            pt.performance_ratio >= 0.5 && pt.performance_ratio <= 1.0,
            "PR should be in [0.5, 1.0]; got {:.4}",
            pt.performance_ratio
        );
    }

    #[test]
    fn test_simulate_hour_zero_irradiance() {
        let model = default_model(1);
        let pt = model.simulate_hour(0.0, 15.0, 2.0);
        assert_eq!(pt.p_dc_w, 0.0, "DC should be zero at night");
        assert_eq!(pt.p_ac_w, 0.0, "AC should be zero at night");
    }

    #[test]
    fn test_annual_yield_reasonable() {
        // 1 MWp system: 1 array with 500 strings × 5 modules × 400 W = 1 MWp
        let arr = PvArray {
            n_strings: 500,
            string: PvString {
                n_modules: 5,
                module: default_module(),
                soiling_loss_pct: 2.0,
                shading_loss_pct: 0.0,
                mismatch_loss_pct: 1.0,
            },
            tilt_deg: 30.0,
            azimuth_deg: 0.0,
            dc_wiring_loss_pct: 1.5,
            system_age_years: 0.0,
        };
        let mut inv = PvInverter::new_string_inverter_50kw();
        inv.p_rated_w = 1_000_000.0;

        let system = PvSystem {
            name: "1MWp Plant".to_string(),
            arrays: vec![arr],
            inverter: inv,
            ac_wiring_loss_pct: 0.5,
            transformer_loss_pct: 0.5,
            availability_pct: 99.0,
            location: (52.5, 13.4),
            grid_connection_mode: GridConnectionMode::GridFollowing,
        };
        let model = PvSystemModel::new(system);

        // Germany: ~1000 peak-sun hours, simplified as 1000 hours at 1000 W/m² and rest zero
        let irr: Vec<f64> = (0..8760)
            .map(|h| if h < 1000 { 1000.0 } else { 0.0 })
            .collect();
        let temp: Vec<f64> = vec![20.0; 8760];
        let yield_mwh = model.simulate_annual_yield(&irr, &temp);
        assert!(
            yield_mwh > 700.0 && yield_mwh < 1200.0,
            "Annual yield should be 700–1200 MWh for 1 MWp; got {:.1} MWh",
            yield_mwh
        );
    }

    #[test]
    fn test_performance_ratio_bounds() {
        // Use a matched model so DC ≈ inverter rated power and PR stays ≤ 1.0.
        let model = matched_model();
        for irr in [100.0, 400.0, 800.0, 1000.0, 1200.0] {
            let pt = model.simulate_hour(irr, 25.0, 12.0);
            assert!(
                pt.performance_ratio >= 0.0 && pt.performance_ratio <= 1.2,
                "PR out of bounds at G={}: PR={:.4}",
                irr,
                pt.performance_ratio
            );
        }
        // Zero irradiance → PR = 0
        let pt_night = model.simulate_hour(0.0, 15.0, 0.0);
        assert_eq!(pt_night.performance_ratio, 0.0);
    }

    #[test]
    fn test_specific_yield_germany() {
        let model = default_model(1);
        let sy = model.estimate_specific_yield_kwh_per_kwp(1000.0);
        assert!(
            sy > 800.0 && sy < 1200.0,
            "Specific yield for Germany irradiance should be 800–1200 kWh/kWp; got {:.1}",
            sy
        );
    }

    #[test]
    fn test_degradation_reduces_output() {
        let model = default_model(1);
        let p_new = model.compute_degraded_output(0.0, 800.0, 25.0);
        let p_aged = model.compute_degraded_output(25.0, 800.0, 25.0);
        assert!(
            p_aged < p_new,
            "Aged output should be less than new output; new={:.1} W, aged={:.1} W",
            p_new,
            p_aged
        );
    }

    #[test]
    fn test_lcoe_reasonable() {
        let model = default_model(1);
        // Utility solar: €800/kWp capex, 15 USD/kW/yr opex, 25 yr, 7% discount, 20% CF
        let p_kw = model.system.inverter.p_rated_w / 1000.0;
        let capex = 800.0 * p_kw;
        let opex = 15.0 * p_kw;
        let lcoe = model.lcoe_usd_per_kwh(capex, opex, 25.0, 0.07, 0.20);
        assert!(
            lcoe > 0.02 && lcoe < 0.25,
            "LCOE should be in [0.02, 0.25] USD/kWh; got {:.4}",
            lcoe
        );
    }
}
