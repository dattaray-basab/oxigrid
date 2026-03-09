//! Offshore wind farm modelling with large-scale wake effects and HVDC collection.
//!
//! Provides:
//! - [`OffshoreWindTurbine`] — 15 MW-class offshore turbine model with power curve
//! - [`FoundationType`] — monopile / jacket / floating foundation selection
//! - [`LarsenWakeModel`] — Larsen (1988) far-wake model for large arrays
//! - [`superpose_wakes`] — Lissaman linear wake superposition
//! - [`array_efficiency`] — aggregate wake loss factor
//! - [`LayoutOptimizer`] — grid vs staggered layout comparison (AEP-based)
//! - [`OffshoreWindFarm`] — complete farm model with HVDC coupling metadata

use crate::error::OxiGridError;
use serde::{Deserialize, Serialize};

// ─────────────────────────────────────────────────────────────────────────────
// Foundation type
// ─────────────────────────────────────────────────────────────────────────────

/// Support-structure category for an offshore wind turbine.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FoundationType {
    /// Monopile — suitable for water depths < 40 m.
    Monopile,
    /// Jacket lattice — 40–80 m water depth.
    Jacket,
    /// Tripod-jacket hybrid — 50–100 m.
    TripodJacket,
    /// Tension-leg platform — > 100 m.
    FloatingTension,
    /// Spar-buoy — > 100 m.
    FloatingSpar,
}

impl FoundationType {
    /// Recommend a foundation type based on water depth \[m\].
    pub fn for_depth(depth_m: f64) -> Self {
        match depth_m as u64 {
            0..=39 => FoundationType::Monopile,
            40..=49 => FoundationType::Jacket,
            50..=99 => FoundationType::TripodJacket,
            100..=149 => FoundationType::FloatingTension,
            _ => FoundationType::FloatingSpar,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Offshore wind turbine
// ─────────────────────────────────────────────────────────────────────────────

/// Offshore wind turbine descriptor (15 MW-class by default).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OffshoreWindTurbine {
    /// Unique turbine identifier within the farm.
    pub id: usize,
    /// Easting position \[m\].
    pub x_m: f64,
    /// Northing position \[m\].
    pub y_m: f64,
    /// Hub height above sea level \[m\]. Default: 120 m.
    pub hub_height_m: f64,
    /// Rotor diameter \[m\]. Default: 200 m (15 MW class).
    pub rotor_diameter_m: f64,
    /// Rated electrical power \[MW\]. Default: 15 MW.
    pub rated_power_mw: f64,
    /// Rated wind speed [m/s]. Default: 13 m/s.
    pub rated_wind_speed: f64,
    /// Cut-in wind speed [m/s]. Default: 3.5 m/s.
    pub cut_in_wind: f64,
    /// Cut-out wind speed [m/s]. Default: 25 m/s.
    pub cut_out_wind: f64,
    /// Operational availability fraction (0–1). Default: 0.97.
    pub availability: f64,
    /// Foundation / support structure type.
    pub foundation_type: FoundationType,
}

impl OffshoreWindTurbine {
    /// Default 15 MW-class offshore turbine at a given position.
    pub fn new_15mw(id: usize, x_m: f64, y_m: f64) -> Self {
        Self {
            id,
            x_m,
            y_m,
            hub_height_m: 120.0,
            rotor_diameter_m: 200.0,
            rated_power_mw: 15.0,
            rated_wind_speed: 13.0,
            cut_in_wind: 3.5,
            cut_out_wind: 25.0,
            availability: 0.97,
            foundation_type: FoundationType::Monopile,
        }
    }

    /// Power output \[MW\] for given wind speed, accounting for availability.
    pub fn power_mw(&self, wind_speed: f64) -> f64 {
        offshore_power_curve(
            wind_speed,
            self.cut_in_wind,
            self.rated_wind_speed,
            self.cut_out_wind,
            self.rated_power_mw,
        ) * self.availability
    }

    /// Thrust coefficient at a given wind speed (simplified).
    ///
    /// Uses a piecewise model: high Ct below rated, tapering above.
    pub fn thrust_coefficient(&self, wind_speed: f64) -> f64 {
        if wind_speed < self.cut_in_wind || wind_speed > self.cut_out_wind {
            return 0.0;
        }
        if wind_speed <= self.rated_wind_speed {
            // Below rated: Ct ≈ 0.85 at cut-in, drops smoothly toward 0.75 at rated
            let frac = (wind_speed - self.cut_in_wind)
                / (self.rated_wind_speed - self.cut_in_wind).max(1e-9);
            0.85 - 0.1 * frac
        } else {
            // Above rated (pitch control): Ct declines with 1/v²
            let ct_rated = 0.75;
            ct_rated * (self.rated_wind_speed / wind_speed).powi(2)
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Power curve
// ─────────────────────────────────────────────────────────────────────────────

/// Offshore turbine power curve \[MW\].
///
/// | Region          | Formula                                                    |
/// |-----------------|------------------------------------------------------------|
/// | v < cut_in      | 0                                                          |
/// | cut_in ≤ v ≤ rated | P = rated × ((v − cut_in) / (rated − cut_in))³        |
/// | rated < v ≤ cut_out | rated_power_mw                                        |
/// | v > cut_out     | 0 (storm shutdown)                                         |
pub fn offshore_power_curve(
    wind_speed: f64,
    cut_in: f64,
    rated: f64,
    cut_out: f64,
    rated_power_mw: f64,
) -> f64 {
    if wind_speed < cut_in || wind_speed > cut_out {
        return 0.0;
    }
    if wind_speed >= rated {
        return rated_power_mw;
    }
    let frac = (wind_speed - cut_in) / (rated - cut_in).max(1e-9);
    rated_power_mw * frac.powi(3)
}

// ─────────────────────────────────────────────────────────────────────────────
// Larsen wake model
// ─────────────────────────────────────────────────────────────────────────────

/// Larsen (1988) near/far-wake model — well-suited for large offshore arrays.
///
/// The velocity deficit in the wake is modelled as a Gaussian-like profile
/// whose width grows with downstream distance and turbulence intensity.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LarsenWakeModel {
    /// Empirical constant controlling wake width (default: 0.04).
    pub c1: f64,
    /// Empirical scaling constant (default: 1.0).
    pub c2: f64,
    /// Ambient turbulence intensity (offshore default: 0.05).
    pub turbulence_intensity: f64,
}

impl LarsenWakeModel {
    /// Create a Larsen model calibrated for the given turbulence intensity.
    pub fn new(turbulence_intensity: f64) -> Self {
        Self {
            c1: 0.04,
            c2: 1.0,
            turbulence_intensity,
        }
    }

    /// Default offshore Larsen model (TI = 0.05).
    pub fn offshore_default() -> Self {
        Self::new(0.05)
    }

    /// Wake velocity deficit as a fraction of freestream speed.
    ///
    /// Based on the Larsen model:
    ///   Δu/u₀ = (Ct / (8 · (σ/D)²)) · exp(−r² / (2 σ²))
    /// where σ = wake width growing linearly with distance.
    ///
    /// Returns 0.0 if `x_m` ≤ 0 (no upstream wake influence).
    pub fn velocity_deficit(
        &self,
        x_m: f64, // downstream distance [m]
        r_m: f64, // lateral offset from wake centreline [m]
        d_m: f64, // rotor diameter [m]
        ct: f64,  // thrust coefficient [-]
    ) -> f64 {
        if x_m <= 0.0 || d_m <= 0.0 || ct <= 0.0 {
            return 0.0;
        }

        // Wake width parameter σ grows linearly with downstream distance.
        // σ = c1 · (x/D) · D  +  TI · D   (simplified Larsen/Ainslie blend)
        let sigma = (self.c1 * x_m + self.turbulence_intensity * d_m).max(1e-6);

        // Gaussian deficit profile (Ainslie 1988 combination)
        let deficit_centre = ct / (8.0 * (sigma / d_m).powi(2)).max(1e-9);
        let gaussian = (-r_m.powi(2) / (2.0 * sigma.powi(2))).exp();
        let raw = self.c2 * deficit_centre * gaussian;

        // Physical bound: deficit cannot exceed 1.0
        raw.clamp(0.0, 1.0)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Wake superposition
// ─────────────────────────────────────────────────────────────────────────────

/// Rotate Cartesian (x, y) into wind-aligned frame.
///
/// Returns `(downstream_distance, lateral_offset)`.
/// `wind_direction_deg` follows meteorological convention:
/// the direction the wind *comes from*, clockwise from north.
fn to_wind_frame(dx: f64, dy: f64, wind_direction_deg: f64) -> (f64, f64) {
    let dir_rad = wind_direction_deg.to_radians();
    let (sin_d, cos_d) = (dir_rad.sin(), dir_rad.cos());
    // Downwind axis: opposite to "from" direction
    let x_w = -(dx * sin_d + dy * cos_d);
    let y_w = -dx * cos_d + dy * sin_d;
    (x_w, y_w)
}

/// Compute effective wind speed at each turbine using Lissaman linear
/// wake superposition on top of the Larsen deficit model.
///
/// Returns a `Vec<f64>` of effective wind speeds [m/s] for each turbine.
pub fn superpose_wakes(
    turbines: &[OffshoreWindTurbine],
    wind_speed: f64,
    wind_direction_deg: f64,
    wake_model: &LarsenWakeModel,
) -> Vec<f64> {
    let n = turbines.len();
    let mut speeds = vec![wind_speed; n];

    for i in 0..n {
        // Linear superposition: Δu_total = Σ Δu_j
        let mut total_deficit = 0.0_f64;

        for j in 0..n {
            if i == j {
                continue;
            }
            let dx = turbines[i].x_m - turbines[j].x_m;
            let dy = turbines[i].y_m - turbines[j].y_m;
            let (x_w, y_w) = to_wind_frame(dx, dy, wind_direction_deg);

            // j is only upstream of i if x_w > 0
            if x_w <= 0.0 {
                continue;
            }

            let ct_j = turbines[j].thrust_coefficient(wind_speed);
            let d_j = turbines[j].rotor_diameter_m;
            let deficit = wake_model.velocity_deficit(x_w, y_w, d_j, ct_j);
            total_deficit += deficit;
        }

        let effective = wind_speed * (1.0 - total_deficit.min(0.95));
        speeds[i] = effective.max(0.0);
    }

    speeds
}

/// Array efficiency: ratio of actual total power to isolated-turbine total power.
///
/// Returns a value in (0, 1].  For a single turbine, returns 1.0.
pub fn array_efficiency(
    turbines: &[OffshoreWindTurbine],
    wind_speed: f64,
    wind_direction_deg: f64,
    wake_model: &LarsenWakeModel,
) -> f64 {
    if turbines.is_empty() {
        return 1.0;
    }

    let effective_speeds = superpose_wakes(turbines, wind_speed, wind_direction_deg, wake_model);

    let actual_power: f64 = turbines
        .iter()
        .zip(effective_speeds.iter())
        .map(|(t, &v)| t.power_mw(v))
        .sum();

    let isolated_power: f64 = turbines.iter().map(|t| t.power_mw(wind_speed)).sum();

    if isolated_power < 1e-9 {
        return 1.0;
    }
    (actual_power / isolated_power).clamp(0.0, 1.0)
}

// ─────────────────────────────────────────────────────────────────────────────
// Layout optimiser
// ─────────────────────────────────────────────────────────────────────────────

/// Offshore wind farm layout optimiser.
///
/// Compares regular grid vs staggered layout and picks the one with higher AEP.
#[derive(Debug, Clone)]
pub struct LayoutOptimizer {
    /// Total leased farm area \[m²\].
    pub farm_area_m2: f64,
    /// Minimum inter-turbine spacing in rotor diameters. Default: 7D.
    pub spacing_d_min: f64,
    /// Wind rose: `(direction_deg, frequency)` pairs summing to 1.0.
    pub wind_rose: Vec<(f64, f64)>,
    /// Reference turbine model (all turbines identical).
    pub turbine_model: OffshoreWindTurbine,
    /// Maximum number of turbines allowed.
    pub max_turbines: usize,
}

/// Result of a layout optimisation run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LayoutResult {
    /// Turbine positions `(x_m, y_m)` in the chosen layout.
    pub turbine_positions: Vec<(f64, f64)>,
    /// Number of turbines.
    pub n_turbines: usize,
    /// Annual energy production [GWh/year].
    pub annual_energy_production_gwh: f64,
    /// Array efficiency [%].
    pub array_efficiency_pct: f64,
    /// Wake loss [%].
    pub wake_loss_pct: f64,
    /// Capacity factor [-].
    pub capacity_factor: f64,
}

impl LayoutOptimizer {
    /// Create a layout optimiser.
    ///
    /// A default wind rose (uniform distribution over 8 directions at 10 m/s)
    /// is used if `wind_rose` is not specified separately (call [`Self::with_wind_rose`]).
    pub fn new(farm_area_m2: f64, turbine_model: OffshoreWindTurbine, max_turbines: usize) -> Self {
        // Default uniform wind rose: 8 cardinal/intercardinal directions
        let dirs = [0.0f64, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0];
        let wind_rose: Vec<(f64, f64)> = dirs.iter().map(|&d| (d, 1.0 / 8.0)).collect();
        Self {
            farm_area_m2,
            spacing_d_min: 7.0,
            wind_rose,
            turbine_model,
            max_turbines,
        }
    }

    /// Override the wind rose.
    pub fn with_wind_rose(mut self, wind_rose: Vec<(f64, f64)>) -> Self {
        self.wind_rose = wind_rose;
        self
    }

    /// Side length of the (square) farm bounding box \[m\].
    fn farm_side_m(&self) -> f64 {
        self.farm_area_m2.sqrt()
    }

    /// Generate regular (rows × cols) grid layout.
    ///
    /// Spacing = max(spacing_d_min, 1.0) × rotor_diameter in both axes.
    pub fn grid_layout(&self, rows: usize, cols: usize) -> Vec<(f64, f64)> {
        let d = self.turbine_model.rotor_diameter_m;
        let spacing = self.spacing_d_min.max(1.0) * d;
        let n = (rows * cols).min(self.max_turbines);
        let mut positions = Vec::with_capacity(n);
        'outer: for r in 0..rows {
            for c in 0..cols {
                if positions.len() >= self.max_turbines {
                    break 'outer;
                }
                positions.push((r as f64 * spacing, c as f64 * spacing));
            }
        }
        positions
    }

    /// Generate staggered layout (alternating rows offset by 0.5 × spacing).
    pub fn staggered_layout(&self, rows: usize, cols: usize) -> Vec<(f64, f64)> {
        let d = self.turbine_model.rotor_diameter_m;
        let spacing_x = self.spacing_d_min.max(1.0) * d;
        let spacing_y = self.spacing_d_min.max(1.0) * d;
        let mut positions = Vec::with_capacity(rows * cols);
        'outer: for r in 0..rows {
            let x_offset = if r % 2 == 1 { spacing_y * 0.5 } else { 0.0 };
            for c in 0..cols {
                if positions.len() >= self.max_turbines {
                    break 'outer;
                }
                positions.push((r as f64 * spacing_x, c as f64 * spacing_y + x_offset));
            }
        }
        positions
    }

    /// Compute annual energy production [GWh/year] for a given layout.
    ///
    /// Uses the wind rose to weight power over directions.
    /// Assumes a representative wind speed of 10 m/s for each direction bin.
    pub fn compute_aep(&self, positions: &[(f64, f64)], wake_model: &LarsenWakeModel) -> f64 {
        if positions.is_empty() {
            return 0.0;
        }

        // Build turbine list from positions
        let turbines: Vec<OffshoreWindTurbine> = positions
            .iter()
            .enumerate()
            .map(|(id, &(x, y))| {
                let mut t = self.turbine_model.clone();
                t.id = id;
                t.x_m = x;
                t.y_m = y;
                t
            })
            .collect();

        // Representative wind speed bin: use 10 m/s for AEP estimate
        // (a real implementation would integrate over a Weibull distribution)
        let wind_speed_ref = 10.0_f64;

        // Annual hours * weighted-average power [MWh] → [GWh]
        let hours_per_year = 8760.0_f64;

        let mut weighted_power_mw = 0.0_f64;
        for &(direction_deg, frequency) in &self.wind_rose {
            let eff_speeds = superpose_wakes(&turbines, wind_speed_ref, direction_deg, wake_model);
            let total_mw: f64 = turbines
                .iter()
                .zip(eff_speeds.iter())
                .map(|(t, &v)| t.power_mw(v))
                .sum();
            weighted_power_mw += total_mw * frequency;
        }

        weighted_power_mw * hours_per_year / 1_000.0
    }

    /// Compare grid vs staggered layouts and return the better one.
    ///
    /// Searches over square-ish (rows, cols) pairs constrained by farm area
    /// and `max_turbines`.  Returns the layout with the higher AEP.
    pub fn optimize(&self, wake_model: &LarsenWakeModel) -> Result<LayoutResult, OxiGridError> {
        let d = self.turbine_model.rotor_diameter_m;
        let spacing = self.spacing_d_min.max(1.0) * d;
        let side = self.farm_side_m();
        let max_rows = ((side / spacing).floor() as usize).max(1);
        let max_cols = max_rows;

        if max_rows == 0 || max_cols == 0 {
            return Err(OxiGridError::InvalidParameter(
                "Farm area too small for minimum turbine spacing".to_string(),
            ));
        }

        // Target turbine count: fit as many as possible up to max_turbines
        let n_target = (max_rows * max_cols).min(self.max_turbines);
        let rows = (n_target as f64).sqrt().ceil() as usize;
        let cols = (n_target as f64 / rows as f64).ceil() as usize;

        let grid_pos = self.grid_layout(rows, cols);
        let stag_pos = self.staggered_layout(rows, cols);

        let grid_aep = self.compute_aep(&grid_pos, wake_model);
        let stag_aep = self.compute_aep(&stag_pos, wake_model);

        let (best_pos, best_aep) = if stag_aep >= grid_aep {
            (stag_pos, stag_aep)
        } else {
            (grid_pos, grid_aep)
        };

        // Compute array efficiency and wake loss for the winning layout
        let turbines: Vec<OffshoreWindTurbine> = best_pos
            .iter()
            .enumerate()
            .map(|(id, &(x, y))| {
                let mut t = self.turbine_model.clone();
                t.id = id;
                t.x_m = x;
                t.y_m = y;
                t
            })
            .collect();

        let wind_speed_ref = 10.0_f64;
        // Average array efficiency over wind rose directions
        let arr_eff: f64 = if self.wind_rose.is_empty() {
            1.0
        } else {
            self.wind_rose
                .iter()
                .map(|&(dir, freq)| {
                    array_efficiency(&turbines, wind_speed_ref, dir, wake_model) * freq
                })
                .sum()
        };

        let n = best_pos.len();
        let installed_mw = n as f64 * self.turbine_model.rated_power_mw;
        // Capacity factor from AEP
        let capacity_factor = if installed_mw > 0.0 {
            best_aep * 1_000.0 / (installed_mw * 8760.0)
        } else {
            0.0
        };

        Ok(LayoutResult {
            turbine_positions: best_pos,
            n_turbines: n,
            annual_energy_production_gwh: best_aep,
            array_efficiency_pct: arr_eff * 100.0,
            wake_loss_pct: (1.0 - arr_eff) * 100.0,
            capacity_factor,
        })
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Offshore wind farm
// ─────────────────────────────────────────────────────────────────────────────

/// Complete offshore wind farm model with HVDC coupling metadata.
#[derive(Debug, Clone)]
pub struct OffshoreWindFarm {
    /// Farm identifier.
    pub farm_id: usize,
    /// Turbines in this farm.
    pub turbines: Vec<OffshoreWindTurbine>,
    /// Turbine layout positions `(x_m, y_m)` (mirrors turbines\[i\].{x_m,y_m}).
    pub layout: Vec<(f64, f64)>,
    /// AC network bus index of the offshore collector substation.
    pub offshore_substation_bus: usize,
    /// AC network bus index of the onshore grid connection.
    pub onshore_connection_bus: usize,
    /// AC collection network voltage \[kV\]. Default: 66 kV.
    pub collection_voltage_kv: f64,
    /// Identifier of the HVDC link connecting farm to shore (if any).
    pub hvdc_link_id: Option<usize>,
    /// Wake model used for this farm.
    pub wake_model: LarsenWakeModel,
}

impl OffshoreWindFarm {
    /// Create an offshore wind farm from a list of turbines.
    pub fn new(
        turbines: Vec<OffshoreWindTurbine>,
        offshore_sub_bus: usize,
        onshore_bus: usize,
    ) -> Self {
        let layout: Vec<(f64, f64)> = turbines.iter().map(|t| (t.x_m, t.y_m)).collect();
        let farm_id = offshore_sub_bus; // convenience default
        Self {
            farm_id,
            turbines,
            layout,
            offshore_substation_bus: offshore_sub_bus,
            onshore_connection_bus: onshore_bus,
            collection_voltage_kv: 66.0,
            hvdc_link_id: None,
            wake_model: LarsenWakeModel::offshore_default(),
        }
    }

    /// Number of turbines.
    pub fn n_turbines(&self) -> usize {
        self.turbines.len()
    }

    /// Total installed capacity \[MW\].
    pub fn installed_capacity_mw(&self) -> f64 {
        self.turbines.iter().map(|t| t.rated_power_mw).sum()
    }

    /// Compute total farm power output \[MW\] at the given wind conditions.
    ///
    /// Applies Larsen wake superposition across all turbines.
    pub fn compute_power(&self, wind_speed: f64, wind_direction_deg: f64) -> f64 {
        if self.turbines.is_empty() {
            return 0.0;
        }
        let eff_speeds = superpose_wakes(
            &self.turbines,
            wind_speed,
            wind_direction_deg,
            &self.wake_model,
        );
        self.turbines
            .iter()
            .zip(eff_speeds.iter())
            .map(|(t, &v)| t.power_mw(v))
            .sum()
    }

    /// Estimate annual capacity factor from a synthetic wind rose.
    ///
    /// `wind_rose`: `(speed_m_s, direction_deg, probability)` tuples.
    /// Probabilities should sum to 1.0.
    pub fn capacity_factor(&self, wind_rose: &[(f64, f64, f64)]) -> f64 {
        if self.turbines.is_empty() || wind_rose.is_empty() {
            return 0.0;
        }
        let rated_mw = self.installed_capacity_mw();
        if rated_mw < 1e-9 {
            return 0.0;
        }

        let mut weighted_power = 0.0_f64;
        for &(speed, direction, prob) in wind_rose {
            let farm_mw = self.compute_power(speed, direction);
            weighted_power += farm_mw * prob;
        }

        (weighted_power / rated_mw).clamp(0.0, 1.0)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Power curve ──────────────────────────────────────────────────────────

    #[test]
    fn test_offshore_power_curve_below_cut_in() {
        let p = offshore_power_curve(3.0, 3.5, 13.0, 25.0, 15.0);
        assert_eq!(p, 0.0, "below cut-in should be zero");
    }

    #[test]
    fn test_offshore_power_curve_at_rated() {
        let p = offshore_power_curve(13.0, 3.5, 13.0, 25.0, 15.0);
        assert!((p - 15.0).abs() < 0.01, "at rated wind speed: {p}");
    }

    #[test]
    fn test_offshore_power_curve_above_cut_out() {
        let p = offshore_power_curve(26.0, 3.5, 13.0, 25.0, 15.0);
        assert_eq!(p, 0.0, "above cut-out should be zero");
    }

    #[test]
    fn test_offshore_power_curve_cubic_region() {
        // Midway between cut-in and rated: expect ≈ 15 * 0.5^3 = 1.875 MW
        let v_mid = 3.5 + (13.0 - 3.5) * 0.5; // 8.25 m/s
        let p = offshore_power_curve(v_mid, 3.5, 13.0, 25.0, 15.0);
        let expected = 15.0 * 0.5_f64.powi(3);
        assert!(
            (p - expected).abs() < 0.01,
            "cubic region p={p:.3} expected≈{expected:.3}"
        );
    }

    // ── Larsen wake model ────────────────────────────────────────────────────

    #[test]
    fn test_larsen_wake_deficit_zero_at_upstream() {
        let model = LarsenWakeModel::offshore_default();
        let deficit = model.velocity_deficit(-100.0, 0.0, 200.0, 0.8);
        assert_eq!(deficit, 0.0, "upstream: no deficit");
    }

    #[test]
    fn test_larsen_wake_decays_with_distance() {
        let model = LarsenWakeModel::offshore_default();
        let d1 = model.velocity_deficit(500.0, 0.0, 200.0, 0.8);
        let d2 = model.velocity_deficit(2000.0, 0.0, 200.0, 0.8);
        assert!(
            d2 < d1,
            "wake deficit should decay downstream: d1={d1:.4} d2={d2:.4}"
        );
    }

    #[test]
    fn test_larsen_wake_deficit_bounded() {
        let model = LarsenWakeModel::offshore_default();
        let deficit = model.velocity_deficit(200.0, 0.0, 200.0, 0.95);
        assert!(
            (0.0..=1.0).contains(&deficit),
            "deficit must be in [0,1]: {deficit}"
        );
    }

    // ── Array efficiency ─────────────────────────────────────────────────────

    #[test]
    fn test_array_efficiency_single_turbine() {
        let turbine = OffshoreWindTurbine::new_15mw(0, 0.0, 0.0);
        let model = LarsenWakeModel::offshore_default();
        let eff = array_efficiency(&[turbine], 10.0, 270.0, &model);
        assert!(
            (eff - 1.0).abs() < 1e-9,
            "single turbine efficiency must be 1.0: {eff}"
        );
    }

    #[test]
    fn test_array_efficiency_multiple_turbines_in_wake() {
        let turbines = vec![
            OffshoreWindTurbine::new_15mw(0, 0.0, 0.0),
            OffshoreWindTurbine::new_15mw(1, 0.0, 1400.0), // 7D downstream (D=200m)
            OffshoreWindTurbine::new_15mw(2, 0.0, 2800.0),
        ];
        let model = LarsenWakeModel::offshore_default();
        let eff = array_efficiency(&turbines, 10.0, 0.0, &model);
        assert!(eff > 0.0 && eff <= 1.0, "efficiency in (0,1]: {eff}");
    }

    // ── Layout ───────────────────────────────────────────────────────────────

    #[test]
    fn test_grid_layout_correct_count() {
        let turbine = OffshoreWindTurbine::new_15mw(0, 0.0, 0.0);
        let opt = LayoutOptimizer::new(100_000_000.0, turbine, 50);
        let pos = opt.grid_layout(3, 4);
        assert_eq!(pos.len(), 12, "3×4 grid should have 12 positions");
    }

    #[test]
    fn test_staggered_layout_correct_count() {
        let turbine = OffshoreWindTurbine::new_15mw(0, 0.0, 0.0);
        let opt = LayoutOptimizer::new(100_000_000.0, turbine, 50);
        let pos = opt.staggered_layout(3, 4);
        assert_eq!(pos.len(), 12, "3×4 staggered should have 12 positions");
    }

    #[test]
    fn test_layout_optimizer_runs() {
        let turbine = OffshoreWindTurbine::new_15mw(0, 0.0, 0.0);
        let opt = LayoutOptimizer::new(400_000_000.0, turbine, 20); // 20km × 20km area
        let model = LarsenWakeModel::offshore_default();
        let result = opt
            .optimize(&model)
            .expect("layout optimization should succeed");
        assert!(result.n_turbines > 0);
        assert!(result.annual_energy_production_gwh > 0.0);
        assert!(result.capacity_factor >= 0.0 && result.capacity_factor <= 1.0);
    }

    // ── Foundation type ──────────────────────────────────────────────────────

    #[test]
    fn test_foundation_type_shallow_water() {
        assert_eq!(FoundationType::for_depth(25.0), FoundationType::Monopile);
    }

    #[test]
    fn test_foundation_type_deep_water() {
        assert_eq!(
            FoundationType::for_depth(200.0),
            FoundationType::FloatingSpar
        );
    }

    #[test]
    fn test_foundation_type_intermediate() {
        assert_eq!(
            FoundationType::for_depth(60.0),
            FoundationType::TripodJacket
        );
    }

    // ── Offshore wind farm ───────────────────────────────────────────────────

    #[test]
    fn test_offshore_farm_compute_power_at_zero_wind() {
        let turbines = vec![
            OffshoreWindTurbine::new_15mw(0, 0.0, 0.0),
            OffshoreWindTurbine::new_15mw(1, 1400.0, 0.0),
        ];
        let farm = OffshoreWindFarm::new(turbines, 1, 2);
        let power = farm.compute_power(0.0, 270.0);
        assert_eq!(power, 0.0);
    }

    #[test]
    fn test_offshore_farm_capacity_factor_in_range() {
        let turbines: Vec<OffshoreWindTurbine> = (0..4)
            .map(|i| OffshoreWindTurbine::new_15mw(i, (i as f64) * 1400.0, 0.0))
            .collect();
        let farm = OffshoreWindFarm::new(turbines, 1, 2);
        // Uniform wind rose: 10 m/s from all 8 directions equally
        let wind_rose: Vec<(f64, f64, f64)> =
            [0.0f64, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0]
                .iter()
                .map(|&d| (10.0, d, 1.0 / 8.0))
                .collect();
        let cf = farm.capacity_factor(&wind_rose);
        assert!(
            (0.0..=1.0).contains(&cf),
            "capacity factor must be in [0,1]: {cf}"
        );
    }

    #[test]
    fn test_offshore_farm_installed_capacity() {
        let turbines: Vec<OffshoreWindTurbine> = (0..5)
            .map(|i| OffshoreWindTurbine::new_15mw(i, (i as f64) * 1400.0, 0.0))
            .collect();
        let farm = OffshoreWindFarm::new(turbines, 1, 2);
        assert!((farm.installed_capacity_mw() - 75.0).abs() < 0.01);
    }
}
