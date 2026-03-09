/// Battery thermal models.
///
/// # Lumped Thermal Model
///
/// A single thermal node (battery cell / pack) described by:
///
///   m·Cp·dT/dt = Q_gen − Q_cool
///
/// Heat generation:
///   Q_gen = I²·R_eff + I·T·ΔS/nF   (Joule + entropic)
///
/// Convective cooling:
///   Q_cool = h·A·(T − T_amb)
///
/// Discrete update (forward Euler):
///   T(k+1) = T(k) + Δt/(m·Cp) · (Q_gen − Q_cool)
use crate::units::Temperature;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LumpedThermalModel {
    /// Cell / pack mass [kg]
    pub mass_kg: f64,
    /// Specific heat capacity [J/(kg·K)]
    pub heat_capacity: f64,
    /// Convective heat transfer coefficient [W/(m²·K)]
    pub h_conv: f64,
    /// Heat transfer area [m²]
    pub area_m2: f64,
    /// Ambient temperature [K]
    pub t_ambient: f64,
    /// Entropic heat coefficient [J/(K·A·s)] (dU/dT * I — usually small)
    pub entropic_coeff: f64,

    // State
    pub temperature: f64,
}

impl LumpedThermalModel {
    pub fn new(mass_kg: f64, heat_capacity: f64, h_conv: f64, area_m2: f64) -> Self {
        Self {
            mass_kg,
            heat_capacity,
            h_conv,
            area_m2,
            t_ambient: 298.15,
            entropic_coeff: 0.0,
            temperature: 298.15,
        }
    }

    /// Simple cylindrical 18650 cell defaults.
    pub fn cell_18650() -> Self {
        Self::new(
            0.045,   // 45 g
            1000.0,  // J/(kg·K)  (approx LFP/NMC)
            10.0,    // natural convection
            0.003_5, // ≈ surface area of 18650
        )
    }

    /// Prismatic pouch cell (75 Ah) defaults.
    pub fn pouch_75ah() -> Self {
        Self::new(
            1.5,    // 1.5 kg
            1000.0, // J/(kg·K)
            15.0,   // forced convection
            0.08,   // m² (approximate)
        )
    }

    /// Advance thermal model by one time step.
    ///
    /// `current_a` — battery current [A] (positive = discharge)
    /// `resistance` — effective internal resistance [Ω]
    /// `dt` — time step [s]
    pub fn step(&mut self, current_a: f64, resistance: f64, dt: f64) -> Temperature {
        // Joule heating
        let q_joule = current_a * current_a * resistance;
        // Entropic heating (positive for exothermic discharge in many Li-ion chemistries)
        let q_entropic = self.entropic_coeff * current_a * self.temperature;
        let q_gen = q_joule + q_entropic;
        // Convective cooling
        let q_cool = self.h_conv * self.area_m2 * (self.temperature - self.t_ambient);

        let d_t = dt / (self.mass_kg * self.heat_capacity) * (q_gen - q_cool);
        self.temperature += d_t;

        Temperature(self.temperature)
    }

    pub fn temperature(&self) -> Temperature {
        Temperature(self.temperature)
    }

    /// Steady-state temperature at constant current.
    pub fn steady_state_temp(&self, current_a: f64, resistance: f64) -> Temperature {
        let q_gen = current_a * current_a * resistance;
        let h_eff = self.h_conv * self.area_m2;
        Temperature(self.t_ambient + q_gen / h_eff)
    }
}

// ── 1D Thermal Model (cylindrical cell) ─────────────────────────────────────

/// Simple 1D radial thermal model for a cylindrical cell.
///
/// Divides the cell into N radial shells.  Heat is generated uniformly
/// in the active material (inner node) and transferred outward.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RadialThermalModel {
    pub n_nodes: usize,
    pub radius_m: f64,
    pub length_m: f64,
    pub conductivity: f64,  // W/(m·K)
    pub density: f64,       // kg/m³
    pub specific_heat: f64, // J/(kg·K)
    pub h_conv: f64,
    pub t_ambient: f64,
    pub temperatures: Vec<f64>, // K, inner to outer
}

impl RadialThermalModel {
    pub fn new(n_nodes: usize, radius_m: f64, length_m: f64) -> Self {
        Self {
            n_nodes,
            radius_m,
            length_m,
            conductivity: 1.0, // W/(m·K) typical Li-ion
            density: 2500.0,   // kg/m³
            specific_heat: 1000.0,
            h_conv: 10.0,
            t_ambient: 298.15,
            temperatures: vec![298.15; n_nodes],
        }
    }

    /// Advance one time step using explicit Euler finite differences.
    pub fn step(&mut self, heat_gen_w_per_m3: f64, dt: f64) {
        let dr = self.radius_m / self.n_nodes as f64;
        let rho_cp = self.density * self.specific_heat;
        let mut new_temps = self.temperatures.clone();

        #[allow(clippy::needless_range_loop)]
        for k in 0..self.n_nodes {
            let r = (k as f64 + 0.5) * dr;
            let flux_in = if k > 0 {
                self.conductivity / dr
                    * (self.temperatures[k - 1] - self.temperatures[k])
                    * (r - dr / 2.0)
                    / r
            } else {
                0.0 // symmetry at centre
            };
            let flux_out = if k < self.n_nodes - 1 {
                self.conductivity / dr
                    * (self.temperatures[k] - self.temperatures[k + 1])
                    * (r + dr / 2.0)
                    / r
            } else {
                // Outer surface: convection
                self.h_conv * (self.temperatures[k] - self.t_ambient) * (r + dr / 2.0) / r
            };
            new_temps[k] +=
                dt / (rho_cp * dr) * (flux_in - flux_out) + dt / rho_cp * heat_gen_w_per_m3;
        }
        self.temperatures = new_temps;
    }

    pub fn max_temperature(&self) -> Temperature {
        Temperature(
            self.temperatures
                .iter()
                .cloned()
                .fold(f64::NEG_INFINITY, f64::max),
        )
    }

    pub fn surface_temperature(&self) -> Temperature {
        Temperature(*self.temperatures.last().unwrap_or(&298.15))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lumped_heats_up_under_load() {
        let mut model = LumpedThermalModel::cell_18650();
        let init_temp = model.temperature;
        // 5A discharge (moderate load)
        for _ in 0..600 {
            model.step(5.0, 0.05, 1.0);
        }
        assert!(model.temperature > init_temp);
    }

    #[test]
    fn test_steady_state() {
        let model = LumpedThermalModel::cell_18650();
        let ss = model.steady_state_temp(5.0, 0.05);
        // With 5A and 0.05Ω: Q = 1.25W, h*A = 10*0.0035 = 0.035
        // ΔT = 1.25/0.035 ≈ 35.7K
        assert!(ss.0 > model.t_ambient);
        assert!(ss.0 < model.t_ambient + 100.0);
    }

    #[test]
    fn test_radial_heats_up() {
        let mut model = RadialThermalModel::new(5, 0.009, 0.065);
        let init_temp = model.temperatures[0];
        // 10 kW/m³ internal heat generation
        for _ in 0..100 {
            model.step(10_000.0, 1.0);
        }
        assert!(model.temperatures[0] > init_temp);
    }
}
