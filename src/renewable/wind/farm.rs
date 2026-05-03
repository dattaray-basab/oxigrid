/// Wind farm aggregate model.
///
/// Combines individual turbine models with wake interaction to compute
/// total farm power output and capacity factor.
use crate::renewable::wind::turbine::{air_density, WindTurbine};
use crate::renewable::wind::wake::{farm_wake_speeds, TurbinePosition, WakeSource};
use serde::{Deserialize, Serialize};

/// Wind farm layout and aggregate power model.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WindFarm {
    /// Turbines in this farm (all assumed identical)
    pub turbine: WindTurbine,
    /// Hub positions in Cartesian coordinates `m`
    pub positions: Vec<TurbinePosition>,
    /// Thrust coefficient at rated operating point
    pub ct_rated: f64,
    /// Jensen wake decay constant
    pub wake_k: f64,
}

/// Farm operating state at a given wind condition.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FarmState {
    /// Wind speed at each turbine after wake interactions [m/s]
    pub hub_speeds: Vec<f64>,
    /// Power at each turbine `kW`
    pub turbine_powers_kw: Vec<f64>,
    /// Total farm power `kW`
    pub total_power_kw: f64,
    /// Wake loss fraction (0 = no losses)
    pub wake_loss_fraction: f64,
    /// Farm capacity factor (−)
    pub capacity_factor: f64,
}

impl WindFarm {
    /// Create a regular row-aligned wind farm.
    ///
    /// `n_rows` × `n_cols` grid; spacing = `spacing_d` × rotor diameter.
    pub fn regular_grid(
        turbine: WindTurbine,
        n_rows: usize,
        n_cols: usize,
        spacing_d: f64,
        ct_rated: f64,
        wake_k: f64,
    ) -> Self {
        let d = turbine.rotor_diameter_m;
        let mut positions = Vec::with_capacity(n_rows * n_cols);
        for r in 0..n_rows {
            for c in 0..n_cols {
                positions.push(TurbinePosition {
                    x: r as f64 * spacing_d * d,
                    y: c as f64 * spacing_d * d,
                });
            }
        }
        Self {
            turbine,
            positions,
            ct_rated,
            wake_k,
        }
    }

    /// Number of turbines.
    pub fn n_turbines(&self) -> usize {
        self.positions.len()
    }

    /// Total installed capacity `kW`.
    pub fn rated_power_kw(&self) -> f64 {
        self.turbine.rated_power_kw * self.n_turbines() as f64
    }

    /// Compute farm state for free-stream wind speed `u_inf` [m/s],
    /// direction `wind_dir_deg`, altitude `altitude_m`, temperature `temp_c`.
    pub fn compute(
        &self,
        u_inf: f64,
        wind_dir_deg: f64,
        altitude_m: f64,
        temp_c: f64,
    ) -> FarmState {
        let rho = air_density(altitude_m, temp_c);
        let n = self.n_turbines();

        let sources: Vec<WakeSource> = (0..n)
            .map(|_| WakeSource {
                u_inf,
                d: self.turbine.rotor_diameter_m,
                ct: self.ct_rated,
                k: self.wake_k,
            })
            .collect();

        let hub_speeds = farm_wake_speeds(&self.positions, wind_dir_deg, &sources);

        let turbine_powers_kw: Vec<f64> = hub_speeds
            .iter()
            .map(|&v| self.turbine.compute(v, rho).power_kw)
            .collect();

        let total_power_kw = turbine_powers_kw.iter().sum();
        let no_wake_power = self.turbine.compute(u_inf, rho).power_kw * n as f64;
        let wake_loss_fraction = if no_wake_power > 0.0 {
            1.0 - total_power_kw / no_wake_power
        } else {
            0.0
        };
        let capacity_factor = total_power_kw / self.rated_power_kw().max(1.0);

        FarmState {
            hub_speeds,
            turbine_powers_kw,
            total_power_kw,
            wake_loss_fraction,
            capacity_factor,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::renewable::wind::turbine::WindTurbine;

    fn small_farm() -> WindFarm {
        WindFarm::regular_grid(WindTurbine::iec_2mw(), 2, 2, 7.0, 0.8, 0.04)
    }

    #[test]
    fn test_farm_n_turbines() {
        assert_eq!(small_farm().n_turbines(), 4);
    }

    #[test]
    fn test_farm_rated_power() {
        let farm = small_farm();
        assert_eq!(farm.rated_power_kw(), 4.0 * 2000.0);
    }

    #[test]
    fn test_farm_power_below_cutin_is_zero() {
        let farm = small_farm();
        let state = farm.compute(2.0, 270.0, 0.0, 15.0);
        assert_eq!(state.total_power_kw, 0.0);
    }

    #[test]
    fn test_wake_losses_positive() {
        let farm = small_farm();
        // Wind aligned with row direction → wake losses
        let state = farm.compute(10.0, 270.0, 0.0, 15.0);
        assert!(state.total_power_kw > 0.0);
        assert!(
            state.wake_loss_fraction >= 0.0,
            "loss={:.4}",
            state.wake_loss_fraction
        );
    }

    #[test]
    fn test_capacity_factor_range() {
        let farm = small_farm();
        let state = farm.compute(10.0, 270.0, 0.0, 15.0);
        assert!(
            state.capacity_factor >= 0.0 && state.capacity_factor <= 1.0,
            "CF={:.3}",
            state.capacity_factor
        );
    }
}
