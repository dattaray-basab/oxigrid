/// Solar irradiance models.
///
/// Computes solar position angles and plane-of-array irradiance from
/// site location, time, and atmospheric conditions.
///
/// # Coordinate conventions
/// - Latitude / longitude in decimal degrees (positive N / positive E)
/// - Tilt angle: 0° = horizontal, 90° = vertical
/// - Azimuth: 0° = south, positive east (meteorological convention)
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

/// Solar position at a given moment and location.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct SolarPosition {
    /// Solar zenith angle `rad`  (0 = directly overhead)
    pub zenith: f64,
    /// Solar elevation angle `rad` = π/2 − zenith
    pub elevation: f64,
    /// Solar azimuth angle `rad` (0 = south, + eastward)
    pub azimuth: f64,
    /// Air mass (Kasten–Young formula)
    pub air_mass: f64,
}

/// Decomposed irradiance components at a tilted surface [W/m²].
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PlaneOfArrayIrradiance {
    /// Total plane-of-array irradiance [W/m²]
    pub total: f64,
    /// Direct (beam) component [W/m²]
    pub direct: f64,
    /// Sky-diffuse component [W/m²]  (isotropic sky model)
    pub diffuse: f64,
    /// Ground-reflected component [W/m²]
    pub ground: f64,
    /// Global horizontal irradiance (input) [W/m²]
    pub ghi: f64,
}

impl SolarPosition {
    /// Compute solar position from site and time parameters.
    ///
    /// # Arguments
    /// - `latitude_deg` — site latitude [°]
    /// - `day_of_year`  — Julian day number (1–365)
    /// - `hour`         — local solar time `h` (0–24)
    pub fn compute(latitude_deg: f64, day_of_year: u32, hour: f64) -> Self {
        let lat = latitude_deg.to_radians();
        let n = day_of_year as f64;

        // Solar declination (Spencer 1971)
        let b = 2.0 * PI * (n - 1.0) / 365.0;
        let delta = (0.006918 - 0.399912 * b.cos() + 0.070257 * b.sin()
            - 0.006758 * (2.0 * b).cos()
            + 0.000907 * (2.0 * b).sin()
            - 0.002697 * (3.0 * b).cos()
            + 0.00148 * (3.0 * b).sin())
        .asin()
        .clamp(-0.4093, 0.4093); // ±23.45°

        // Hour angle (solar noon = 0)
        let omega = (hour - 12.0) * 15.0_f64.to_radians();

        // Solar elevation
        let sin_elev =
            (lat.sin() * delta.sin() + lat.cos() * delta.cos() * omega.cos()).clamp(-1.0, 1.0);
        let elevation = sin_elev.asin();
        let zenith = PI / 2.0 - elevation;

        // Solar azimuth (south = 0, east positive)
        let cos_az = if zenith.cos().abs() > 1e-6 {
            (sin_elev * lat.sin() - delta.sin()) / (zenith.sin() * lat.cos())
        } else {
            0.0
        };
        let az_abs = cos_az.clamp(-1.0, 1.0).acos();
        let azimuth = if omega > 0.0 { az_abs } else { -az_abs };

        // Air mass (Kasten & Young 1989 formula)
        let air_mass = if elevation > 0.0 {
            1.0 / (elevation.sin() + 0.50572 * (elevation.to_degrees() + 6.07995_f64).powf(-1.6364))
        } else {
            f64::INFINITY
        };

        Self {
            zenith,
            elevation,
            azimuth,
            air_mass,
        }
    }

    /// Whether the sun is above the horizon.
    pub fn is_daytime(&self) -> bool {
        self.elevation > 0.0
    }
}

/// Compute extraterrestrial irradiance [W/m²] on a horizontal surface.
///
/// Uses the ASHRAE correction for Earth–Sun distance variation.
pub fn extraterrestrial_irradiance(day_of_year: u32) -> f64 {
    let gsc = 1361.0; // Solar constant [W/m²]
    let b = 2.0 * PI * day_of_year as f64 / 365.0;
    let et_factor = 1.000110
        + 0.034221 * b.cos()
        + 0.001280 * b.sin()
        + 0.000719 * (2.0 * b).cos()
        + 0.000077 * (2.0 * b).sin();
    gsc * et_factor
}

/// Erbs (1982) diffuse fraction correlation.
///
/// Splits GHI into direct normal (DNI) and diffuse horizontal (DHI)
/// from the clearness index kt = GHI / (I_0 * cos(θz)).
pub fn erbs_decomposition(ghi: f64, clearness_index: f64) -> (f64, f64) {
    let kt = clearness_index.clamp(0.0, 1.0);
    let df = if kt <= 0.22 {
        1.0 - 0.09 * kt
    } else if kt <= 0.80 {
        0.9511 - 0.1604 * kt + 4.388 * kt * kt - 16.638 * kt.powi(3) + 12.336 * kt.powi(4)
    } else {
        0.165
    };
    let dhi = (df * ghi).max(0.0);
    let dni_horiz = (ghi - dhi).max(0.0);
    (dhi, dni_horiz)
}

/// Compute plane-of-array irradiance using the isotropic sky model
/// (Liu & Jordan 1963).
///
/// # Arguments
/// - `ghi`         — global horizontal irradiance [W/m²]
/// - `pos`         — solar position
/// - `tilt_deg`    — panel tilt from horizontal [°]
/// - `panel_az_deg`— panel azimuth (south=0, east positive) [°]
/// - `albedo`      — ground reflectance (typical: 0.2)
pub fn poa_isotropic(
    ghi: f64,
    pos: &SolarPosition,
    tilt_deg: f64,
    panel_az_deg: f64,
    albedo: f64,
    day_of_year: u32,
) -> PlaneOfArrayIrradiance {
    if !pos.is_daytime() || ghi <= 0.0 {
        return PlaneOfArrayIrradiance {
            total: 0.0,
            direct: 0.0,
            diffuse: 0.0,
            ground: 0.0,
            ghi,
        };
    }

    let i0 = extraterrestrial_irradiance(day_of_year);
    let cos_z = pos.zenith.cos().max(1e-6);
    let kt = (ghi / (i0 * cos_z)).min(1.0);
    let (dhi, dni_horiz) = erbs_decomposition(ghi, kt);
    let dni = if cos_z > 1e-3 { dni_horiz / cos_z } else { 0.0 };

    let tilt = tilt_deg.to_radians();
    let panel_az = panel_az_deg.to_radians();

    // Angle of incidence on tilted panel
    let cos_aoi = (pos.elevation.sin() * tilt.cos()
        + pos.elevation.cos() * (pos.azimuth - panel_az).cos() * tilt.sin())
    .max(0.0);

    let direct = dni * cos_aoi;
    let diffuse = dhi * (1.0 + tilt.cos()) / 2.0; // isotropic sky
    let ground = ghi * albedo * (1.0 - tilt.cos()) / 2.0; // ground reflection

    let total = direct + diffuse + ground;
    PlaneOfArrayIrradiance {
        total,
        direct,
        diffuse,
        ground,
        ghi,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_solar_position_noon() {
        // At solar noon, zenith should equal |latitude - declination|
        let pos = SolarPosition::compute(35.0, 172, 12.0); // ~summer solstice, mid-day
        assert!(pos.is_daytime());
        assert!(
            pos.elevation.to_degrees() > 50.0,
            "elevation={}",
            pos.elevation.to_degrees()
        );
    }

    #[test]
    fn test_solar_below_horizon_night() {
        let pos = SolarPosition::compute(35.0, 1, 2.0); // 2 AM
        assert!(!pos.is_daytime());
    }

    #[test]
    fn test_extraterrestrial_near_solar_constant() {
        let i0 = extraterrestrial_irradiance(1);
        assert!(i0 > 1300.0 && i0 < 1420.0, "I0={}", i0);
    }

    #[test]
    fn test_poa_tilted_vs_horizontal() {
        // Tilted south-facing panel should capture more irradiance in winter
        let pos = SolarPosition::compute(35.0, 355, 12.0); // winter solstice noon
        let poa_h = poa_isotropic(700.0, &pos, 0.0, 0.0, 0.2, 355); // horizontal
        let poa_t = poa_isotropic(700.0, &pos, 35.0, 0.0, 0.2, 355); // tilted 35°
        assert!(
            poa_t.total >= poa_h.total,
            "tilted={:.1} horizontal={:.1}",
            poa_t.total,
            poa_h.total
        );
    }

    #[test]
    fn test_erbs_decomposition_overcast() {
        // Low clearness index → high diffuse fraction
        let (dhi, _dni_h) = erbs_decomposition(100.0, 0.1);
        assert!(dhi > 80.0, "dhi={}", dhi);
    }
}
