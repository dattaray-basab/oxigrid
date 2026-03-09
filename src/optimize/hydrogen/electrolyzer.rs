//! Electrolyzer models for hydrogen production via water electrolysis.
//!
//! Supports PEM, Alkaline, and SOEC electrolyzer types with:
//! - Efficiency curves (power fraction → efficiency, with linear interpolation)
//! - Stack degradation tracking over operating hours
//! - IV-curve based electrochemical stack model (Butler-Volmer activation)
//! - Water and heat output computation
//! - Operating cost with electricity price, O&M, and degradation

/// Higher Heating Value of hydrogen [kWh/kg].
pub const H2_HHV_KWH_PER_KG: f64 = 39.4;

/// Electrolyzer technology type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ElectrolyzerType {
    /// Proton Exchange Membrane electrolyzer (fast response, low min load)
    Pem,
    /// Alkaline Water Electrolysis (mature technology, higher min load)
    Alkaline,
    /// Solid Oxide Electrolysis Cell (high temperature, high efficiency)
    Soec,
}

/// Electrolyzer model with efficiency curve and degradation tracking.
#[derive(Debug, Clone)]
pub struct Electrolyzer {
    /// Rated electrical power \[MW\]
    pub rated_power_mw: f64,
    /// Technology type
    pub electrolyzer_type: ElectrolyzerType,
    /// Minimum load fraction (0–1)
    pub min_power_fraction: f64,
    /// Efficiency curve: list of (power_fraction, efficiency) pairs, sorted ascending by power_fraction
    pub efficiency_curve: Vec<(f64, f64)>,
    /// Cold-start time \[minutes\]
    pub startup_time_min: f64,
    /// Shutdown time \[minutes\]
    pub shutdown_time_min: f64,
    /// Stack degradation rate [% per 1000 operating hours]
    pub stack_degradation_pct_per_kh: f64,
    /// Cumulative operating hours
    pub operating_hours: f64,
    /// Water consumption [L per kg H2 produced] (default 9.0)
    pub water_consumption_l_per_kg: f64,
    /// Fraction of input power dissipated as recoverable heat
    pub heat_output_fraction: f64,
    /// Fixed O&M cost [$/kg H2]
    pub fixed_om_per_kg: f64,
    /// Stack replacement cost per % degradation [$/kg H2 per %]
    pub degradation_cost_per_pct: f64,
}

impl Electrolyzer {
    /// Construct an electrolyzer with technology-appropriate defaults.
    pub fn new(rated_mw: f64, electrolyzer_type: ElectrolyzerType) -> Self {
        let (min_frac, curve, startup, shutdown, deg_rate, heat_frac) = match electrolyzer_type {
            ElectrolyzerType::Pem => (
                0.05,
                vec![
                    (0.05, 0.55),
                    (0.20, 0.62),
                    (0.50, 0.67),
                    (0.75, 0.65),
                    (1.00, 0.62),
                ],
                15.0,
                5.0,
                0.5,
                0.15,
            ),
            ElectrolyzerType::Alkaline => (
                0.20,
                vec![
                    (0.20, 0.60),
                    (0.40, 0.65),
                    (0.60, 0.68),
                    (0.80, 0.67),
                    (1.00, 0.65),
                ],
                30.0,
                10.0,
                0.3,
                0.18,
            ),
            ElectrolyzerType::Soec => (
                0.10,
                vec![
                    (0.10, 0.70),
                    (0.30, 0.78),
                    (0.60, 0.82),
                    (0.80, 0.80),
                    (1.00, 0.77),
                ],
                60.0,
                30.0,
                1.2,
                0.10,
            ),
        };

        Self {
            rated_power_mw: rated_mw,
            electrolyzer_type,
            min_power_fraction: min_frac,
            efficiency_curve: curve,
            startup_time_min: startup,
            shutdown_time_min: shutdown,
            stack_degradation_pct_per_kh: deg_rate,
            operating_hours: 0.0,
            water_consumption_l_per_kg: 9.0,
            heat_output_fraction: heat_frac,
            fixed_om_per_kg: 0.5,
            degradation_cost_per_pct: 0.02,
        }
    }

    /// Compute hydrogen production rate at given power input.
    ///
    /// Returns 0.0 if power is below the minimum operating threshold.
    /// Production [kg/h] = power_mw * 1000 \[kW\] * efficiency / H2_HHV [kWh/kg]
    pub fn hydrogen_production_kg_per_h(&self, power_mw: f64) -> f64 {
        if self.rated_power_mw < 1e-12 {
            return 0.0;
        }
        let power_fraction = (power_mw / self.rated_power_mw).clamp(0.0, 1.0);
        if power_fraction < self.min_power_fraction - 1e-9 {
            return 0.0;
        }
        let eff = self.efficiency_at(power_fraction);
        // power_mw * 1000 converts to kW; divide by HHV to get kg/h
        let power_kw = power_mw * 1000.0;
        power_kw * eff / H2_HHV_KWH_PER_KG
    }

    /// Current electrical efficiency at a given power fraction (0–1).
    ///
    /// Linearly interpolates the efficiency curve, then applies the degradation factor.
    pub fn efficiency_at(&self, power_fraction: f64) -> f64 {
        let pf = power_fraction.clamp(0.0, 1.0);
        let nominal_eff = interpolate_curve(&self.efficiency_curve, pf);
        nominal_eff * (1.0 - self.degradation_factor())
    }

    /// Fractional performance loss due to stack degradation (0.0 = new, approaching 1.0 = failed).
    pub fn degradation_factor(&self) -> f64 {
        // degradation_fraction = (pct_per_kh * hours / 1000) / 100
        let deg_frac = self.stack_degradation_pct_per_kh * self.operating_hours / 1000.0 / 100.0;
        deg_frac.clamp(0.0, 0.5) // cap at 50% degradation for physical plausibility
    }

    /// Minimum and maximum hydrogen production rates [kg/h] at rated efficiency.
    ///
    /// Returns `(min_kg_per_h, max_kg_per_h)`.
    pub fn h2_range_kg_per_h(&self) -> (f64, f64) {
        let min_prod =
            self.hydrogen_production_kg_per_h(self.min_power_fraction * self.rated_power_mw);
        let max_prod = self.hydrogen_production_kg_per_h(self.rated_power_mw);
        (min_prod, max_prod)
    }

    /// Operating cost per kg of hydrogen produced [$/kg].
    ///
    /// Includes electricity cost, fixed O&M, and degradation cost component.
    /// Returns `f64::INFINITY` if production rate is zero.
    pub fn operating_cost_per_kg(&self, electricity_price_per_mwh: f64, power_mw: f64) -> f64 {
        let production_kg_h = self.hydrogen_production_kg_per_h(power_mw);
        if production_kg_h < 1e-12 {
            return f64::INFINITY;
        }
        // Electricity cost: price [$/MWh] * power [MW] = $/h; divided by production [kg/h] → $/kg
        let electricity_cost_per_kg = electricity_price_per_mwh * power_mw / production_kg_h;
        // Degradation cost proportional to current degradation rate
        let deg_cost = self.degradation_cost_per_pct * self.stack_degradation_pct_per_kh / 1000.0; // $/kg from degradation
        electricity_cost_per_kg + self.fixed_om_per_kg + deg_cost
    }

    /// Heat output from the electrolyzer at given power input [MW thermal].
    pub fn heat_output_mw(&self, power_mw: f64) -> f64 {
        power_mw * self.heat_output_fraction
    }

    /// Water consumed per hour at given power input [L/h].
    pub fn water_consumption_l_per_h(&self, power_mw: f64) -> f64 {
        self.hydrogen_production_kg_per_h(power_mw) * self.water_consumption_l_per_kg
    }
}

/// Linear interpolation of a sorted (x, y) curve.
///
/// Clamps to the boundary values outside the curve range.
fn interpolate_curve(curve: &[(f64, f64)], x: f64) -> f64 {
    if curve.is_empty() {
        return 0.0;
    }
    if curve.len() == 1 {
        return curve[0].1;
    }
    // Clamp to bounds
    if x <= curve[0].0 {
        return curve[0].1;
    }
    if x >= curve[curve.len() - 1].0 {
        return curve[curve.len() - 1].1;
    }
    // Find segment
    for i in 1..curve.len() {
        let (x0, y0) = curve[i - 1];
        let (x1, y1) = curve[i];
        if x <= x1 {
            let t = if (x1 - x0).abs() < 1e-15 {
                0.0
            } else {
                (x - x0) / (x1 - x0)
            };
            return y0 + t * (y1 - y0);
        }
    }
    curve[curve.len() - 1].1
}

// ── Electrochemical stack model ───────────────────────────────────────────────

/// Faraday constant [C/mol]
const FARADAY: f64 = 96485.0;
/// Universal gas constant [J/(mol·K)]
const R_GAS: f64 = 8.314;
/// Number of electrons per H2 molecule
const N_ELECTRONS: f64 = 2.0;
/// Molar mass of H2 [g/mol]
const H2_MOLAR_MASS_G: f64 = 2.016;
/// Standard reversible cell voltage at 25°C, 1 bar [V]
const V_REV_STD: f64 = 1.229;

/// Electrolyzer stack model based on IV (current-voltage) curves.
///
/// Models the electrochemical cell behaviour using Butler-Volmer kinetics
/// for activation overpotential, linear ohmic resistance for membrane loss,
/// and a temperature-corrected reversible voltage.
pub struct ElectrolyzerStack {
    /// Number of cells in series
    pub n_cells: usize,
    /// Active cell area \[cm²\]
    pub cell_area_cm2: f64,
    /// Operating temperature [°C]
    pub temperature_c: f64,
    /// Operating pressure \[bar\]
    pub pressure_bar: f64,
    /// Exchange current density [A/cm²] (Butler-Volmer parameter)
    pub exchange_current_density: f64,
    /// Charge transfer coefficient α (Butler-Volmer)
    pub alpha: f64,
    /// Membrane area-specific resistance [Ω·cm²]
    pub membrane_resistance_ohm_cm2: f64,
}

impl ElectrolyzerStack {
    /// Create a new electrolyzer stack with default parameters for a PEM cell.
    pub fn new(n_cells: usize, cell_area_cm2: f64) -> Self {
        Self {
            n_cells,
            cell_area_cm2,
            temperature_c: 80.0,            // typical PEM operating temperature
            pressure_bar: 30.0,             // typical cathode pressure
            exchange_current_density: 1e-3, // [A/cm²]
            alpha: 0.5,
            membrane_resistance_ohm_cm2: 0.18, // Nafion 117 typical value
        }
    }

    /// Compute single cell voltage at a given current density [A/cm²].
    ///
    /// V_cell = V_rev + V_act + V_ohm
    ///
    /// - V_rev: temperature- and pressure-corrected reversible voltage
    /// - V_act: Butler-Volmer activation overpotential (anodic dominant)
    /// - V_ohm: Ohmic overpotential from membrane resistance
    pub fn cell_voltage(&self, current_density_a_cm2: f64) -> f64 {
        let t_k = self.temperature_c + 273.15;

        // Reversible voltage: temperature correction (ΔS/nF ≈ −8.5e-4 V/K for water splitting)
        // Pressure correction: (RT/nF) * ln(P) added for compressed product
        let delta_s_over_nf = -8.5e-4; // V/K
        let t_ref = 298.15; // K
        let v_rev_t = V_REV_STD + delta_s_over_nf * (t_k - t_ref);
        // Pressure correction: Nernst term (RT/nF)*ln(P/P_ref)
        let p_correction =
            (R_GAS * t_k / (N_ELECTRONS * FARADAY)) * self.pressure_bar.max(1.0).ln();
        let v_rev = v_rev_t + p_correction;

        // Activation overpotential (Butler-Volmer, high-field approximation: Tafel equation)
        // V_act = (RT / α·n·F) * ln(j / j0)
        let j = current_density_a_cm2.max(1e-12); // avoid log(0)
        let j0 = self.exchange_current_density.max(1e-20);
        let v_act = (R_GAS * t_k / (self.alpha * N_ELECTRONS * FARADAY)) * (j / j0).ln();
        let v_act = v_act.max(0.0); // activation always positive (energy barrier)

        // Ohmic overpotential: V_ohm = j * R_mem [A/cm² * Ω·cm² = V]
        let v_ohm = current_density_a_cm2 * self.membrane_resistance_ohm_cm2;

        v_rev + v_act + v_ohm
    }

    /// Compute stack operating point at a given current density.
    ///
    /// Returns `(power_kw, h2_kg_per_h)`.
    ///
    /// H2 production follows Faraday's law:
    /// ṁ_H2 = (n_cells * I * M_H2) / (n_e * F) [g/s] → [kg/h]
    pub fn operating_point(&self, current_density_a_cm2: f64) -> (f64, f64) {
        let v_cell = self.cell_voltage(current_density_a_cm2);
        let current_a = current_density_a_cm2 * self.cell_area_cm2;
        let stack_power_w = v_cell * current_a * self.n_cells as f64;
        let power_kw = stack_power_w / 1000.0;

        // Faraday's law: mol/s of H2 = I / (n_e * F) per cell, times n_cells
        let h2_mol_per_s = (self.n_cells as f64 * current_a) / (N_ELECTRONS * FARADAY);
        // Convert to kg/h: mol/s * (g/mol) / 1000 * 3600
        let h2_kg_per_h = h2_mol_per_s * H2_MOLAR_MASS_G / 1000.0 * 3600.0;

        (power_kw, h2_kg_per_h)
    }

    /// Stack efficiency at given current density (LHV basis, using HHV here for consistency).
    pub fn efficiency(&self, current_density_a_cm2: f64) -> f64 {
        let (power_kw, h2_kg_per_h) = self.operating_point(current_density_a_cm2);
        if power_kw < 1e-12 {
            return 0.0;
        }
        // H2 energy = H2 mass flow [kg/h] * HHV [kWh/kg]
        let h2_energy_kw = h2_kg_per_h * H2_HHV_KWH_PER_KG;
        (h2_energy_kw / power_kw).clamp(0.0, 1.0)
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_electrolyzer_production_rate() {
        // At 1 MW, PEM efficiency ~0.65 area → H2 ≈ 1000 * 0.65 / 39.4 ≈ 16.5 kg/h
        let elz = Electrolyzer::new(1.0, ElectrolyzerType::Pem);
        let h2 = elz.hydrogen_production_kg_per_h(1.0);
        // efficiency at fraction=1.0 is 0.62 (from curve) * (1-0) = 0.62
        // h2 = 1000 * 0.62 / 39.4 ≈ 15.74
        assert!(h2 > 10.0, "Expected H2 > 10 kg/h at 1 MW, got {h2:.3}");
        assert!(h2 < 25.0, "Expected H2 < 25 kg/h at 1 MW, got {h2:.3}");
    }

    #[test]
    fn test_electrolyzer_efficiency_interpolation() {
        let elz = Electrolyzer::new(2.0, ElectrolyzerType::Pem);
        // At 50% power fraction, efficiency should be between min and max
        let eff_50 = elz.efficiency_at(0.50);
        let eff_20 = elz.efficiency_at(0.20);
        let eff_100 = elz.efficiency_at(1.00);
        assert!(eff_50 > 0.0 && eff_50 <= 1.0);
        assert!(eff_20 > 0.0 && eff_20 <= 1.0);
        assert!(eff_100 > 0.0 && eff_100 <= 1.0);
        // PEM curve peaks around 0.5–0.75, so eff_50 should be near the peak
        assert!(
            eff_50 > eff_100,
            "Efficiency at 50% should exceed efficiency at 100% for PEM"
        );
    }

    #[test]
    fn test_electrolyzer_below_min_load_returns_zero() {
        let elz = Electrolyzer::new(1.0, ElectrolyzerType::Alkaline);
        // Below min_power_fraction of 0.20, no production
        let h2 = elz.hydrogen_production_kg_per_h(0.10); // 10% of 1MW
        assert_eq!(h2, 0.0);
    }

    #[test]
    fn test_electrolyzer_degradation_reduces_efficiency() {
        let mut elz = Electrolyzer::new(1.0, ElectrolyzerType::Pem);
        let eff_new = elz.efficiency_at(0.5);
        elz.operating_hours = 10_000.0; // 10,000 hours
        let eff_aged = elz.efficiency_at(0.5);
        assert!(
            eff_aged < eff_new,
            "Degraded electrolyzer should be less efficient"
        );
    }

    #[test]
    fn test_h2_range() {
        let elz = Electrolyzer::new(5.0, ElectrolyzerType::Soec);
        let (min_h2, max_h2) = elz.h2_range_kg_per_h();
        assert!(min_h2 >= 0.0);
        assert!(max_h2 > min_h2, "Max production must exceed min production");
    }

    #[test]
    fn test_operating_cost_finite_at_rated_power() {
        let elz = Electrolyzer::new(2.0, ElectrolyzerType::Pem);
        let cost = elz.operating_cost_per_kg(50.0, 2.0);
        assert!(cost.is_finite() && cost > 0.0);
    }

    #[test]
    fn test_electrolyzer_stack_iv_curve() {
        let stack = ElectrolyzerStack::new(100, 300.0);
        let v_low = stack.cell_voltage(0.1);
        let v_high = stack.cell_voltage(1.0);
        // Higher current density → higher cell voltage (polarization losses)
        assert!(
            v_high > v_low,
            "Cell voltage at j=1.0 A/cm² ({v_high:.4} V) should exceed j=0.1 A/cm² ({v_low:.4} V)"
        );
    }

    #[test]
    fn test_electrolyzer_stack_operating_point() {
        let stack = ElectrolyzerStack::new(200, 400.0);
        let (power_kw, h2_kg_per_h) = stack.operating_point(0.5);
        assert!(power_kw > 0.0, "Stack power must be positive");
        assert!(h2_kg_per_h > 0.0, "H2 production must be positive");
    }
}
