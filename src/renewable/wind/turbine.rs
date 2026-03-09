/// Wind turbine power models.
///
/// Provides:
/// - Piecewise-linear power curve lookup
/// - Analytical power curve (cubic ramp)
/// - Air density correction
/// - Betz-limit Cp model (optional)
use serde::{Deserialize, Serialize};

const RHO_STD: f64 = 1.225; // Standard air density [kg/m³] at 15°C, sea level

/// Turbine power output for a given wind speed.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct TurbineState {
    /// Hub-height wind speed [m/s]
    pub wind_speed: f64,
    /// Electrical power output [kW]
    pub power_kw: f64,
    /// Power coefficient Cp (≤ 0.593 Betz limit)
    pub cp: f64,
    /// Air density [kg/m³]
    pub air_density: f64,
}

/// Wind turbine specification and power model.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WindTurbine {
    /// Rated power [kW]
    pub rated_power_kw: f64,
    /// Rotor diameter [m]
    pub rotor_diameter_m: f64,
    /// Hub height [m]
    pub hub_height_m: f64,
    /// Cut-in wind speed [m/s]
    pub v_cut_in: f64,
    /// Rated wind speed [m/s]
    pub v_rated: f64,
    /// Cut-out wind speed [m/s]
    pub v_cut_out: f64,
    /// Optional manufacturer power curve as (wind_speed, power_kw) pairs
    pub power_curve: Option<Vec<(f64, f64)>>,
}

impl WindTurbine {
    /// IEC Class II 2 MW reference turbine (analytical cubic model).
    pub fn iec_2mw() -> Self {
        Self {
            rated_power_kw: 2000.0,
            rotor_diameter_m: 90.0,
            hub_height_m: 80.0,
            v_cut_in: 3.5,
            v_rated: 13.0,
            v_cut_out: 25.0,
            power_curve: None,
        }
    }

    /// Large offshore turbine — 5 MW.
    pub fn offshore_5mw() -> Self {
        Self {
            rated_power_kw: 5000.0,
            rotor_diameter_m: 126.0,
            hub_height_m: 90.0,
            v_cut_in: 3.0,
            v_rated: 11.4,
            v_cut_out: 25.0,
            power_curve: None,
        }
    }

    /// Rotor swept area [m²].
    pub fn rotor_area(&self) -> f64 {
        std::f64::consts::PI * (self.rotor_diameter_m / 2.0).powi(2)
    }

    /// Compute turbine state at hub-height wind speed `v` [m/s] and
    /// air density `rho` [kg/m³].
    pub fn compute(&self, v: f64, rho: f64) -> TurbineState {
        let density_ratio = rho / RHO_STD;

        let power_kw = if v < self.v_cut_in || v >= self.v_cut_out {
            0.0
        } else if let Some(ref curve) = self.power_curve {
            interpolate_power_curve(curve, v) * density_ratio
        } else {
            self.analytical_power(v, rho)
        };

        // Cp from P = 0.5 * rho * A * v³ * Cp
        let p_avail = 0.5 * rho * self.rotor_area() * v.powi(3);
        let cp = if p_avail > 1.0 {
            (power_kw * 1000.0 / p_avail).clamp(0.0, 0.593)
        } else {
            0.0
        };

        TurbineState {
            wind_speed: v,
            power_kw,
            cp,
            air_density: rho,
        }
    }

    /// Compute using standard air density.
    pub fn compute_at_stp(&self, v: f64) -> TurbineState {
        self.compute(v, RHO_STD)
    }

    /// Analytical cubic power curve (IEC 61400-12 simplified model).
    fn analytical_power(&self, v: f64, rho: f64) -> f64 {
        let density_ratio = rho / RHO_STD;
        if v < self.v_cut_in || v >= self.v_cut_out {
            return 0.0;
        }
        if v >= self.v_rated {
            return self.rated_power_kw * density_ratio.min(1.0);
        }
        // Cubic interpolation in ramp region
        let frac = (v - self.v_cut_in) / (self.v_rated - self.v_cut_in);
        self.rated_power_kw * frac.powi(3) * density_ratio
    }

    /// Annual Energy Production estimate [MWh/year] using a Weibull wind
    /// speed distribution with scale `c` [m/s] and shape `k`.
    pub fn annual_energy_mwh(&self, weibull_c: f64, weibull_k: f64) -> f64 {
        // Numerical integration over wind speeds 0–40 m/s
        let n = 400usize;
        let dv = 40.0 / n as f64;
        let hours_per_year = 8760.0;

        (0..n)
            .map(|i| {
                let v = (i as f64 + 0.5) * dv;
                let prob = weibull_pdf(v, weibull_c, weibull_k) * dv;
                self.compute_at_stp(v).power_kw * prob
            })
            .sum::<f64>()
            * hours_per_year
            / 1000.0 // kWh → MWh
    }
}

/// Air density correction for altitude and temperature.
///
/// - `altitude_m` — metres above sea level
/// - `temp_celsius` — ambient temperature [°C]
pub fn air_density(altitude_m: f64, temp_celsius: f64) -> f64 {
    let temp_k = temp_celsius + 273.15;
    let pressure = 101325.0 * (1.0 - 2.2557e-5 * altitude_m).powf(5.2559);
    pressure / (287.058 * temp_k) // ideal gas law [kg/m³]
}

/// Wind speed at hub height using logarithmic wind profile.
///
/// - `v_ref`      — reference wind speed [m/s] at `z_ref` [m]
/// - `z_hub`      — hub height [m]
/// - `z_ref`      — reference height [m] (typically 10 m met mast)
/// - `z_roughness`— surface roughness length [m] (0.03 = open farmland)
pub fn log_wind_profile(v_ref: f64, z_ref: f64, z_hub: f64, z_roughness: f64) -> f64 {
    let z0 = z_roughness.max(1e-3);
    v_ref * (z_hub / z0).ln() / (z_ref / z0).ln()
}

fn interpolate_power_curve(curve: &[(f64, f64)], v: f64) -> f64 {
    if curve.is_empty() {
        return 0.0;
    }
    if v <= curve[0].0 {
        return curve[0].1;
    }
    if v >= curve[curve.len() - 1].0 {
        return curve[curve.len() - 1].1;
    }

    let pos = curve.partition_point(|&(s, _)| s <= v);
    let (v0, p0) = curve[pos - 1];
    let (v1, p1) = curve[pos];
    let alpha = (v - v0) / (v1 - v0);
    p0 + alpha * (p1 - p0)
}

fn weibull_pdf(v: f64, c: f64, k: f64) -> f64 {
    if v < 0.0 {
        return 0.0;
    }
    (k / c) * (v / c).powf(k - 1.0) * (-(v / c).powf(k)).exp()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_power_below_cut_in_is_zero() {
        let t = WindTurbine::iec_2mw();
        assert_eq!(t.compute_at_stp(2.0).power_kw, 0.0);
    }

    #[test]
    fn test_power_at_rated_speed() {
        let t = WindTurbine::iec_2mw();
        let state = t.compute_at_stp(t.v_rated);
        assert!(
            (state.power_kw - t.rated_power_kw).abs() < 1.0,
            "power={:.1}",
            state.power_kw
        );
    }

    #[test]
    fn test_power_above_cut_out_is_zero() {
        let t = WindTurbine::iec_2mw();
        assert_eq!(t.compute_at_stp(30.0).power_kw, 0.0);
    }

    #[test]
    fn test_cp_below_betz_limit() {
        let t = WindTurbine::iec_2mw();
        for v in [4.0, 7.0, 10.0, 12.0] {
            let s = t.compute_at_stp(v);
            assert!(s.cp <= 0.593 + 1e-6, "v={} Cp={:.4}", v, s.cp);
        }
    }

    #[test]
    fn test_power_monotone_in_ramp_region() {
        let t = WindTurbine::iec_2mw();
        let mut prev = 0.0;
        let mut v = t.v_cut_in;
        while v < t.v_rated {
            let p = t.compute_at_stp(v).power_kw;
            assert!(
                p >= prev - 1e-6,
                "not monotone at v={:.1}: {:.2} < {:.2}",
                v,
                p,
                prev
            );
            prev = p;
            v += 0.5;
        }
    }

    #[test]
    fn test_log_wind_profile() {
        // Higher hub → higher wind speed
        let v10 = 7.0;
        let v80 = log_wind_profile(v10, 10.0, 80.0, 0.03);
        assert!(v80 > v10, "v80={:.2} <= v10={:.2}", v80, v10);
        assert!(v80 < v10 * 2.0, "unreasonably large extrapolation");
    }

    #[test]
    fn test_air_density_decreases_with_altitude() {
        let rho0 = air_density(0.0, 15.0);
        let rho2000 = air_density(2000.0, 15.0);
        assert!(rho2000 < rho0);
        assert!((rho0 - RHO_STD).abs() < 0.002, "rho0={:.4}", rho0);
    }

    #[test]
    fn test_annual_energy_positive() {
        let t = WindTurbine::iec_2mw();
        let aep = t.annual_energy_mwh(8.0, 2.0); // Weibull c=8 m/s, k=2
        assert!(aep > 1000.0 && aep < 10_000.0, "AEP={:.0} MWh", aep);
    }
}
