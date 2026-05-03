/// Wind farm wake models.
///
/// Provides:
/// - **Jensen (1983) top-hat wake model** — linear cone expansion, widely
///   used for layout optimization and energy yield assessment.
/// - **Frandsen (2006) model** — improved wake width model.
use serde::{Deserialize, Serialize};

/// Position of a turbine in a wind farm layout.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct TurbinePosition {
    /// Downstream distance from reference origin `m`
    pub x: f64,
    /// Crosswind distance `m`
    pub y: f64,
}

/// Wake deficit parameters for a single upstream turbine.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct WakeSource {
    /// Hub-height free-stream wind speed [m/s]
    pub u_inf: f64,
    /// Rotor diameter `m`
    pub d: f64,
    /// Thrust coefficient (−)
    pub ct: f64,
    /// Wake decay constant (Jensen: 0.04 offshore, 0.075 onshore)
    pub k: f64,
}

impl WakeSource {
    /// Compute wake wind speed at point (dx, dy) relative to source.
    ///
    /// Uses the Jensen top-hat model.
    /// Returns `u_inf` if outside the wake cone.
    pub fn jensen_wake_speed(&self, dx: f64, dy: f64) -> f64 {
        if dx <= 0.0 {
            return self.u_inf;
        }

        let r0 = self.d / 2.0;
        let r_wake = r0 + self.k * dx;

        // Wake deficit factor for a point fully inside the wake
        let deficit = (1.0 - (1.0 - self.ct).sqrt()) * (r0 / r_wake).powi(2);

        // Check overlap: is the evaluation point inside the wake cone?
        if dy.abs() <= r_wake {
            // Partial overlap area fraction (top-hat: binary inside/outside)
            let overlap = overlap_fraction(dy.abs(), r_wake, r0);
            self.u_inf * (1.0 - deficit * overlap)
        } else {
            self.u_inf
        }
    }

    /// Compute wake wind speed using the Frandsen (2006) improved model.
    pub fn frandsen_wake_speed(&self, dx: f64, dy: f64) -> f64 {
        if dx <= 0.0 {
            return self.u_inf;
        }

        let r0 = self.d / 2.0;
        // Frandsen wake expansion coefficient
        let beta = 0.5 * (1.0 + (1.0 - self.ct).sqrt()) / (1.0 - self.ct).sqrt();
        let r_wake = r0 * (beta + self.k * dx / r0).sqrt();

        if dy.abs() <= r_wake {
            let deficit = 0.5 * (1.0 - (1.0 - self.ct * (r0 / r_wake).powi(2)).sqrt());
            self.u_inf * (1.0 - 2.0 * deficit)
        } else {
            self.u_inf
        }
    }
}

/// Geometric overlap fraction (simplified linear approximation).
fn overlap_fraction(dy: f64, r_wake: f64, _r_rotor: f64) -> f64 {
    if dy > r_wake {
        0.0
    } else {
        1.0
    }
}

/// Compute effective wind speed at each turbine in a farm using the
/// square-sum wake superposition principle (Lissaman 1979).
///
/// `positions` — turbine positions
/// `wind_dir_deg` — wind direction (270° = westerly, blowing east)
/// `sources` — WakeSource parameters for each turbine
pub fn farm_wake_speeds(
    positions: &[TurbinePosition],
    wind_dir_deg: f64,
    sources: &[WakeSource],
) -> Vec<f64> {
    assert_eq!(positions.len(), sources.len());
    let n = positions.len();
    let u_inf = sources[0].u_inf;

    // Rotate coordinates to wind-aligned frame.
    // Meteorological convention: wind_dir_deg is the direction the wind comes FROM,
    // clockwise from north.  Downwind unit vector (x=east, y=north):
    //   u_dw = (-sin(θ), -cos(θ))
    // x_w = downstream distance (positive = further downwind)
    // y_w = crosswind distance
    let dir_rad = wind_dir_deg.to_radians();
    let (sin_d, cos_d) = (dir_rad.sin(), dir_rad.cos());

    let rotate = |p: &TurbinePosition| -> (f64, f64) {
        let x_w = -p.x * sin_d - p.y * cos_d;
        let y_w = -p.x * cos_d + p.y * sin_d;
        (x_w, y_w)
    };

    let rotated: Vec<(f64, f64)> = positions.iter().map(rotate).collect();

    let mut speeds = vec![u_inf; n];

    for i in 0..n {
        let mut deficit_sq_sum = 0.0_f64;
        for j in 0..n {
            if i == j {
                continue;
            }
            let dx = rotated[i].0 - rotated[j].0;
            let dy = rotated[i].1 - rotated[j].1;
            if dx <= 0.0 {
                continue;
            } // j is not upstream of i
            let u_wake = sources[j].jensen_wake_speed(dx, dy);
            let d_rel = (u_inf - u_wake) / u_inf;
            deficit_sq_sum += d_rel * d_rel;
        }
        speeds[i] = u_inf * (1.0 - deficit_sq_sum.sqrt());
    }

    speeds
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_jensen_no_deficit_upstream() {
        let src = WakeSource {
            u_inf: 10.0,
            d: 90.0,
            ct: 0.8,
            k: 0.04,
        };
        let u = src.jensen_wake_speed(-100.0, 0.0);
        assert_eq!(u, 10.0);
    }

    #[test]
    fn test_jensen_deficit_on_centreline() {
        let src = WakeSource {
            u_inf: 10.0,
            d: 90.0,
            ct: 0.8,
            k: 0.04,
        };
        let u = src.jensen_wake_speed(500.0, 0.0);
        assert!(u < 10.0 && u > 0.0, "u={:.2}", u);
    }

    #[test]
    fn test_jensen_deficit_outside_cone() {
        let src = WakeSource {
            u_inf: 10.0,
            d: 90.0,
            ct: 0.8,
            k: 0.04,
        };
        // Very far crosswind — outside wake cone
        let u = src.jensen_wake_speed(500.0, 1000.0);
        assert_eq!(u, 10.0);
    }

    #[test]
    fn test_deficit_decreases_with_distance() {
        let src = WakeSource {
            u_inf: 10.0,
            d: 90.0,
            ct: 0.8,
            k: 0.04,
        };
        let u1 = src.jensen_wake_speed(300.0, 0.0);
        let u2 = src.jensen_wake_speed(600.0, 0.0);
        // Deficit should recover with distance
        assert!(u2 > u1, "u1={:.2} u2={:.2}", u1, u2);
    }

    #[test]
    fn test_farm_two_turbines_in_line() {
        let positions = vec![
            TurbinePosition { x: 0.0, y: 0.0 },
            TurbinePosition { x: 500.0, y: 0.0 },
        ];
        let src = WakeSource {
            u_inf: 10.0,
            d: 90.0,
            ct: 0.8,
            k: 0.04,
        };
        let sources = vec![src; 2];
        let speeds = farm_wake_speeds(&positions, 270.0, &sources); // westerly wind
                                                                    // Second turbine should be in wake of first
        assert!(speeds[1] < speeds[0], "speeds={:?}", speeds);
    }
}
