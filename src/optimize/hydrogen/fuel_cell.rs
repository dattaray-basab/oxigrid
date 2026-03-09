//! Fuel cell models for hydrogen-to-electricity conversion.
//!
//! Supports PEMFC, SOFC, and MCFC fuel cell types with:
//! - Efficiency curves (power fraction → electrical efficiency)
//! - CHP heat recovery
//! - H2 consumption calculation (inverse of electrolyzer)
//! - Stoichiometric water production tracking
//! - Stack degradation modelling

use super::electrolyzer::H2_HHV_KWH_PER_KG;

/// Fuel cell technology type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FuelCellType {
    /// Proton Exchange Membrane Fuel Cell (low temperature, fast response)
    Pemfc,
    /// Solid Oxide Fuel Cell (high temperature, high efficiency, good for CHP)
    Sofc,
    /// Molten Carbonate Fuel Cell (high temperature, suitable for large-scale CHP)
    Mcfc,
}

/// Stoichiometric water production ratio [kg H2O per kg H2].
///
/// From: H2 + 1/2 O2 → H2O, molar masses 2 + 16 = 18 g/mol H2O per 2 g/mol H2
const H2O_PER_H2_KG: f64 = 9.0;

/// Fuel cell model with efficiency curve, CHP capability, and degradation.
#[derive(Debug, Clone)]
pub struct FuelCell {
    /// Rated electrical output power \[MW\]
    pub rated_power_mw: f64,
    /// Technology type
    pub fuel_cell_type: FuelCellType,
    /// Minimum load fraction (0–1)
    pub min_power_fraction: f64,
    /// Efficiency curve: (power_fraction, electrical_efficiency) pairs, ascending by power_fraction
    pub efficiency_curve: Vec<(f64, f64)>,
    /// Heat recovery fraction for CHP applications (0–1)
    pub heat_recovery_fraction: f64,
    /// Cold-start time \[minutes\]
    pub startup_time_min: f64,
    /// Stack degradation rate [% per 1000 operating hours]
    pub degradation_pct_per_kh: f64,
    /// Cumulative operating hours
    pub operating_hours: f64,
    /// Parasitic load fraction (auxiliary power for pumps, fans, etc.)
    pub parasitic_fraction: f64,
}

impl FuelCell {
    /// Create a fuel cell with technology-appropriate defaults.
    pub fn new(rated_mw: f64, fc_type: FuelCellType) -> Self {
        let (min_frac, curve, heat_rec, startup, deg_rate, parasitic) = match fc_type {
            FuelCellType::Pemfc => (
                0.10,
                vec![
                    (0.10, 0.52),
                    (0.25, 0.58),
                    (0.50, 0.60),
                    (0.75, 0.57),
                    (1.00, 0.53),
                ],
                0.35, // 35% heat recovery
                5.0,  // 5 min startup
                1.0,  // 1% per 1000 h
                0.03, // 3% parasitic
            ),
            FuelCellType::Sofc => (
                0.15,
                vec![
                    (0.15, 0.55),
                    (0.30, 0.62),
                    (0.60, 0.65),
                    (0.80, 0.63),
                    (1.00, 0.60),
                ],
                0.50, // 50% heat recovery for high-temp
                60.0, // 60 min startup (high temp)
                0.8,
                0.04,
            ),
            FuelCellType::Mcfc => (
                0.20,
                vec![
                    (0.20, 0.47),
                    (0.40, 0.52),
                    (0.60, 0.55),
                    (0.80, 0.54),
                    (1.00, 0.51),
                ],
                0.45,  // 45% heat recovery
                120.0, // 2 h startup
                0.6,
                0.05,
            ),
        };

        Self {
            rated_power_mw: rated_mw,
            fuel_cell_type: fc_type,
            min_power_fraction: min_frac,
            efficiency_curve: curve,
            heat_recovery_fraction: heat_rec,
            startup_time_min: startup,
            degradation_pct_per_kh: deg_rate,
            operating_hours: 0.0,
            parasitic_fraction: parasitic,
        }
    }

    /// Electrical efficiency at a given power fraction (0–1), accounting for degradation.
    pub fn efficiency_at(&self, power_fraction: f64) -> f64 {
        let pf = power_fraction.clamp(0.0, 1.0);
        let nominal_eff = interpolate_curve(&self.efficiency_curve, pf);
        // Apply degradation (same formula as electrolyzer)
        let deg_frac =
            (self.degradation_pct_per_kh * self.operating_hours / 1000.0 / 100.0).clamp(0.0, 0.4);
        nominal_eff * (1.0 - deg_frac)
    }

    /// Hydrogen consumption rate at given power output [kg H2 / h].
    ///
    /// H2 consumed = power_output / (efficiency * HHV_H2) [kW / (kWh/kg)] = [kg/h]
    pub fn hydrogen_consumption_kg_per_h(&self, power_mw: f64) -> f64 {
        if self.rated_power_mw < 1e-12 || power_mw <= 0.0 {
            return 0.0;
        }
        let power_fraction = (power_mw / self.rated_power_mw).clamp(0.0, 1.0);
        if power_fraction < self.min_power_fraction - 1e-9 {
            return 0.0;
        }
        let eff = self.efficiency_at(power_fraction);
        if eff < 1e-12 {
            return 0.0;
        }
        let power_kw = power_mw * 1000.0;
        // H2 consumed [kg/h] = electrical_power [kW] / (efficiency * HHV [kWh/kg])
        power_kw / (eff * H2_HHV_KWH_PER_KG)
    }

    /// Thermal (heat) output for CHP applications at given power output [MW thermal].
    ///
    /// Heat = electrical_power * heat_recovery_fraction * (1 - η_e) / η_e
    /// Derived from: total_fuel_power = P_e / η_e;  P_heat = (fuel - electrical) * heat_rec
    pub fn heat_output_mw(&self, power_mw: f64) -> f64 {
        if power_mw <= 0.0 || self.rated_power_mw < 1e-12 {
            return 0.0;
        }
        let power_fraction = (power_mw / self.rated_power_mw).clamp(0.0, 1.0);
        let eff = self.efficiency_at(power_fraction);
        if !(1e-12..1.0).contains(&eff) {
            return 0.0;
        }
        // Total heat rejected = P_e * (1 - η_e) / η_e (from fuel energy balance)
        let heat_rejected_mw = power_mw * (1.0 - eff) / eff;
        heat_rejected_mw * self.heat_recovery_fraction
    }

    /// Net electrical output after subtracting parasitic loads \[MW\].
    pub fn net_power_mw(&self, power_mw: f64) -> f64 {
        power_mw * (1.0 - self.parasitic_fraction)
    }

    /// Water production rate (stoichiometric, product water only) [kg H2O / h].
    pub fn water_production_kg_per_h(&self, power_mw: f64) -> f64 {
        let h2_consumed = self.hydrogen_consumption_kg_per_h(power_mw);
        h2_consumed * H2O_PER_H2_KG
    }

    /// Maximum H2 consumption at rated power [kg/h].
    pub fn max_h2_consumption_kg_per_h(&self) -> f64 {
        self.hydrogen_consumption_kg_per_h(self.rated_power_mw)
    }

    /// Combined Heat and Power (CHP) system efficiency (electrical + heat recovered).
    pub fn total_chp_efficiency(&self, power_fraction: f64) -> f64 {
        let eff_e = self.efficiency_at(power_fraction);
        if eff_e < 1e-12 {
            return 0.0;
        }
        let heat_fraction = (1.0 - eff_e) * self.heat_recovery_fraction;
        (eff_e + heat_fraction).clamp(0.0, 1.0)
    }
}

/// Linear interpolation of a sorted (x, y) efficiency curve.
fn interpolate_curve(curve: &[(f64, f64)], x: f64) -> f64 {
    if curve.is_empty() {
        return 0.0;
    }
    if curve.len() == 1 {
        return curve[0].1;
    }
    if x <= curve[0].0 {
        return curve[0].1;
    }
    if x >= curve[curve.len() - 1].0 {
        return curve[curve.len() - 1].1;
    }
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

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fuel_cell_h2_consumption() {
        // PEMFC at rated power: efficiency ~0.53, HHV = 39.4 kWh/kg
        // H2 consumed = 1000 kW / (0.53 * 39.4) ≈ 47.9 kg/h
        let fc = FuelCell::new(1.0, FuelCellType::Pemfc);
        let h2 = fc.hydrogen_consumption_kg_per_h(1.0);
        assert!(h2 > 30.0, "Expected H2 consumption > 30 kg/h, got {h2:.3}");
        assert!(h2 < 80.0, "Expected H2 consumption < 80 kg/h, got {h2:.3}");
    }

    #[test]
    fn test_fuel_cell_heat_output() {
        // At rated power, heat output must be positive
        let fc = FuelCell::new(2.0, FuelCellType::Sofc);
        let heat = fc.heat_output_mw(2.0);
        assert!(
            heat > 0.0,
            "SOFC heat output should be positive, got {heat:.4} MW"
        );
    }

    #[test]
    fn test_fuel_cell_heat_scales_with_power() {
        let fc = FuelCell::new(5.0, FuelCellType::Pemfc);
        let heat_full = fc.heat_output_mw(5.0);
        let heat_half = fc.heat_output_mw(2.5);
        // More power → more heat (proportional relationship)
        assert!(
            heat_full > heat_half,
            "Full-power heat should exceed half-power heat"
        );
    }

    #[test]
    fn test_fuel_cell_zero_output_at_zero_power() {
        let fc = FuelCell::new(1.0, FuelCellType::Pemfc);
        assert_eq!(fc.hydrogen_consumption_kg_per_h(0.0), 0.0);
        assert_eq!(fc.heat_output_mw(0.0), 0.0);
        assert_eq!(fc.water_production_kg_per_h(0.0), 0.0);
    }

    #[test]
    fn test_fuel_cell_water_production_stoichiometry() {
        let fc = FuelCell::new(1.0, FuelCellType::Pemfc);
        let h2 = fc.hydrogen_consumption_kg_per_h(1.0);
        let water = fc.water_production_kg_per_h(1.0);
        // Stoichiometric ratio: 9 kg H2O per kg H2
        let ratio = water / h2;
        assert!(
            (ratio - 9.0).abs() < 0.01,
            "Water/H2 ratio should be 9.0, got {ratio:.4}"
        );
    }

    #[test]
    fn test_fuel_cell_chp_efficiency_exceeds_electrical() {
        let fc = FuelCell::new(1.0, FuelCellType::Sofc);
        let eff_e = fc.efficiency_at(0.6);
        let eff_chp = fc.total_chp_efficiency(0.6);
        assert!(
            eff_chp > eff_e,
            "CHP efficiency ({eff_chp:.4}) should exceed electrical ({eff_e:.4})"
        );
    }

    #[test]
    fn test_fuel_cell_below_min_load_returns_zero() {
        let fc = FuelCell::new(1.0, FuelCellType::Mcfc);
        // Below 20% min load
        let h2 = fc.hydrogen_consumption_kg_per_h(0.05);
        assert_eq!(h2, 0.0, "Should not consume H2 below min load");
    }

    #[test]
    fn test_fuel_cell_degradation() {
        let mut fc = FuelCell::new(1.0, FuelCellType::Pemfc);
        let eff_new = fc.efficiency_at(0.5);
        fc.operating_hours = 20_000.0;
        let eff_aged = fc.efficiency_at(0.5);
        assert!(eff_aged < eff_new, "Aged FC should have lower efficiency");
    }
}
