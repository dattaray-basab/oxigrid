//! Bifacial photovoltaic module model.
//!
//! Computes rear-side irradiance using view factor analysis, albedo effects,
//! and module height above ground. Calculates bifacial energy yield gains
//! versus monofacial modules.

use core::f64::consts::PI;

// ─── Surface types & albedo ──────────────────────────────────────────────────

/// Classification of ground surface material for albedo assignment.
#[derive(Debug, Clone, PartialEq)]
pub enum SurfaceType {
    /// Portland cement concrete (albedo ≈ 0.25).
    Concrete,
    /// Asphalt road surface (albedo ≈ 0.12).
    Asphalt,
    /// Short grass lawn (albedo ≈ 0.20).
    Grass,
    /// Fresh/compacted snow (albedo ≈ 0.70).
    Snow,
    /// Dry sand (albedo ≈ 0.35).
    Sand,
    /// Open water surface (albedo ≈ 0.07).
    Water,
    /// User-specified albedo value.
    Custom {
        /// Albedo coefficient in [0, 1].
        albedo: f64,
    },
}

impl SurfaceType {
    /// Returns the broadband albedo coefficient in [0, 1] for this surface.
    pub fn albedo_value(&self) -> f64 {
        match self {
            SurfaceType::Concrete => 0.25,
            SurfaceType::Asphalt => 0.12,
            SurfaceType::Grass => 0.20,
            SurfaceType::Snow => 0.70,
            SurfaceType::Sand => 0.35,
            SurfaceType::Water => 0.07,
            SurfaceType::Custom { albedo } => *albedo,
        }
    }
}

/// Ground surface albedo used by the bifacial rear-irradiance model.
#[derive(Debug, Clone)]
pub struct GroundAlbedo {
    /// Albedo coefficient [0, 1].
    pub albedo: f64,
    /// Surface classification.
    pub surface_type: SurfaceType,
}

impl GroundAlbedo {
    /// Creates a [`GroundAlbedo`] from a [`SurfaceType`], deriving the
    /// albedo coefficient automatically.
    pub fn new(surface: SurfaceType) -> Self {
        let albedo = surface.albedo_value();
        GroundAlbedo {
            albedo,
            surface_type: surface,
        }
    }
}

// ─── Module geometry ─────────────────────────────────────────────────────────

/// Physical layout of a bifacial PV array row.
#[derive(Debug, Clone)]
pub struct BifacialGeometry {
    /// Height of the module's lowest edge above ground (m).
    pub height_m: f64,
    /// Module tilt angle from horizontal (degrees).
    pub tilt_deg: f64,
    /// Module dimension in the tilt direction (m).
    pub module_length_m: f64,
    /// Module dimension perpendicular to the tilt direction (m).
    pub module_width_m: f64,
    /// Distance between centres of adjacent rows (m).
    pub row_pitch_m: f64,
    /// Number of rows used for shading analysis.
    pub n_rows: u32,
    /// Ground coverage ratio (GCR = module_length * cos(tilt) / row_pitch).
    pub gcr: f64,
}

impl BifacialGeometry {
    /// Convenience constructor for a standard fixed-tilt single-axis mount.
    ///
    /// Defaults: height = 0.5 m, module 2.0 × 1.0 m, row pitch = 4.0 m,
    /// 10 rows, GCR = 0.4.
    pub fn default_fixed_tilt(tilt_deg: f64) -> Self {
        BifacialGeometry {
            height_m: 0.5,
            tilt_deg,
            module_length_m: 2.0,
            module_width_m: 1.0,
            row_pitch_m: 4.0,
            n_rows: 10,
            gcr: 0.4,
        }
    }
}

// ─── Irradiance inputs ────────────────────────────────────────────────────────

/// Instantaneous solar irradiance components for one timestep.
#[derive(Debug, Clone)]
pub struct IrradianceComponents {
    /// Direct Normal Irradiance (W/m²).
    pub dni: f64,
    /// Diffuse Horizontal Irradiance (W/m²).
    pub dhi: f64,
    /// Global Horizontal Irradiance (W/m²).
    pub ghi: f64,
    /// Solar zenith angle (degrees, 0 = overhead).
    pub zenith_deg: f64,
    /// Solar azimuth angle (degrees, 180 = south in northern hemisphere).
    pub azimuth_deg: f64,
}

// ─── Irradiance result ────────────────────────────────────────────────────────

/// Bifacial irradiance decomposition for one timestep.
#[derive(Debug, Clone)]
pub struct BifacialIrradiance {
    /// Front-side plane-of-array irradiance (W/m²).
    pub front_poa: f64,
    /// Rear-side irradiance originating from ground reflection (W/m²).
    pub rear_ground: f64,
    /// Rear-side diffuse sky irradiance (W/m²).
    pub rear_sky: f64,
    /// Total rear-side irradiance (W/m²).
    pub rear_total: f64,
    /// Effective irradiance combining front and bifaciality-weighted rear (W/m²).
    pub effective_irradiance: f64,
    /// Bifacial gain as a fraction (e.g. 0.05 = 5 %).
    pub bifacial_gain: f64,
    /// Fraction of ground area in shadow beneath the array [0, 1].
    pub shadow_fraction: f64,
}

// ─── Module electrical parameters ────────────────────────────────────────────

/// Electrical parameters of a bifacial PV module.
#[derive(Debug, Clone)]
pub struct BifacialModuleParams {
    /// Front-face maximum power at STC (W).
    pub p_max_stc_w: f64,
    /// Bifaciality factor — ratio of rear to front efficiency (typically 0.65–0.85).
    pub bifaciality: f64,
    /// Temperature coefficient of P_max (%/°C, typically −0.35 to −0.45).
    pub temp_coeff_pmax: f64,
    /// Nominal Operating Cell Temperature (°C).
    pub noct_c: f64,
    /// Module active area (m²).
    pub module_area_m2: f64,
    /// Number of modules in the system.
    pub n_modules: u32,
}

// ─── Annual yield inputs / outputs ───────────────────────────────────────────

/// Hourly time-series inputs for an annual yield simulation.
///
/// All vectors must have the same length (typically 8 760).
#[derive(Debug, Clone)]
pub struct AnnualYieldInput {
    /// Hourly GHI (W/m²).
    pub hourly_ghi: Vec<f64>,
    /// Hourly DHI (W/m²).
    pub hourly_dhi: Vec<f64>,
    /// Hourly DNI (W/m²).
    pub hourly_dni: Vec<f64>,
    /// Hourly solar zenith (degrees).
    pub hourly_zenith_deg: Vec<f64>,
    /// Hourly solar azimuth (degrees).
    pub hourly_azimuth_deg: Vec<f64>,
    /// Hourly ambient temperature (°C).
    pub hourly_tamb_c: Vec<f64>,
}

/// Annual energy yield comparison between monofacial and bifacial configurations.
#[derive(Debug, Clone)]
pub struct BifacialYieldResult {
    /// Equivalent monofacial annual energy (kWh/year).
    pub monofacial_kwh_yr: f64,
    /// Bifacial annual energy (kWh/year).
    pub bifacial_kwh_yr: f64,
    /// Bifacial gain relative to monofacial (%).
    pub bifacial_gain_pct: f64,
    /// Monthly bifacial energy yields (kWh/month), 12 entries.
    pub monthly_kwh: Vec<f64>,
    /// Optimal module tilt angle from geometry optimisation (degrees).
    pub optimal_tilt_deg: f64,
    /// Optimal module height from geometry optimisation (m).
    pub optimal_height_m: f64,
    /// Optimal ground coverage ratio from geometry optimisation.
    pub optimal_gcr: f64,
    /// Estimated LCOE improvement (%) due to bifacial gain.
    pub lcoe_improvement_pct: f64,
}

/// Row-to-row shading analysis results.
#[derive(Debug, Clone)]
pub struct ShadingAnalysis {
    /// Hourly shadow fraction on the ground beneath the module [0, 1].
    pub hourly_shade_fraction: Vec<f64>,
    /// GHI-weighted annual shading loss (%).
    pub annual_shading_loss_pct: f64,
    /// Minimum row pitch that keeps annual shading loss below 5 % (m).
    pub min_pitch_for_5pct_loss_m: f64,
}

// ─── Main system struct ───────────────────────────────────────────────────────

/// Bifacial PV system calculator.
///
/// Combines geometry, module parameters, and ground albedo to compute
/// irradiance, power output, annual yields, shading analysis, and geometry
/// optimisation.
#[derive(Debug, Clone)]
pub struct BifacialPvSystem {
    /// Array layout geometry.
    pub geometry: BifacialGeometry,
    /// Module electrical parameters.
    pub module: BifacialModuleParams,
    /// Ground surface albedo.
    pub ground: GroundAlbedo,
}

impl BifacialPvSystem {
    /// Creates a new [`BifacialPvSystem`].
    pub fn new(
        geometry: BifacialGeometry,
        module: BifacialModuleParams,
        ground: GroundAlbedo,
    ) -> Self {
        BifacialPvSystem {
            geometry,
            module,
            ground,
        }
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    /// Computes the angle of incidence (AOI) on the front face of the module.
    ///
    /// Uses the isotropic-sky geometric formula:
    /// `AOI = arccos(cos(z)*cos(tilt) + sin(z)*cos(az_mod - az_sun)*sin(tilt))`
    ///
    /// The module is assumed south-facing (az_mod = 180°).
    ///
    /// Returns AOI in degrees, clamped to [0°, 90°].
    fn angle_of_incidence(&self, zenith_deg: f64, azimuth_deg: f64) -> f64 {
        let tilt_rad = self.geometry.tilt_deg.to_radians();
        let zenith_rad = zenith_deg.to_radians();
        // Module faces south (180°) in the northern hemisphere convention.
        let az_mod_rad = PI; // 180°
        let az_sun_rad = azimuth_deg.to_radians();

        let cos_aoi = zenith_rad.cos() * tilt_rad.cos()
            + zenith_rad.sin() * (az_mod_rad - az_sun_rad).cos() * tilt_rad.sin();

        // Clamp to [-1, 1] to guard against floating-point overshoot.
        let cos_aoi = cos_aoi.clamp(-1.0, 1.0);
        cos_aoi.acos().to_degrees().clamp(0.0, 90.0)
    }

    /// Computes the shadow fraction of the ground beneath the module row.
    ///
    /// Based on a simplified geometric projection:
    /// 1. Compute solar altitude = 90° − zenith.
    /// 2. Project the shadow of the module onto the ground.
    /// 3. Express the shadow as a fraction of the row-pitch-normalised area.
    ///
    /// Returns a value in [0, 1].
    fn shadow_fraction(&self, zenith_deg: f64) -> f64 {
        let altitude_deg = 90.0 - zenith_deg;
        if altitude_deg <= 0.0 {
            return 1.0;
        }
        let altitude_rad = altitude_deg.to_radians();
        let tilt_rad = self.geometry.tilt_deg.to_radians();

        // Length of the shadow cast from the module's lowest edge.
        // shadow_length = height_m / tan(altitude)
        let shadow_length = self.geometry.height_m / altitude_rad.tan().max(1e-9);

        // Projected length of the module on the ground.
        let proj_length = self.geometry.module_length_m * tilt_rad.cos();

        // Ground clearance between front of this row and rear of previous row.
        let clearance = self.geometry.row_pitch_m - proj_length;

        // How much of the shadow extends into the row's footprint.
        let shade = (shadow_length - clearance.max(0.0)) / proj_length.max(1e-9);
        shade.clamp(0.0, 1.0)
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /// Computes bifacial irradiance components for a single timestep.
    ///
    /// # Algorithm
    ///
    /// **Front POA** (Liu & Jordan isotropic-sky model):
    /// - Direct: `DNI * max(0, cos(AOI))`
    /// - Diffuse: `DHI * (1 + cos(tilt)) / 2`
    /// - Ground reflected: `GHI * albedo * (1 − cos(tilt)) / 2`
    ///
    /// **Rear irradiance** (view-factor method):
    /// - Sky: `DHI * (1 − cos(tilt)) / 2`  (rear sees sky portion)
    /// - Ground: `GHI * albedo * (1 − cos(tilt)) / 2 * (1 − shadow_fraction)`
    ///
    /// **Effective irradiance**: `front_poa + bifaciality * rear_total`
    pub fn compute_irradiance(&self, irr: &IrradianceComponents) -> BifacialIrradiance {
        let tilt_rad = self.geometry.tilt_deg.to_radians();
        let cos_tilt = tilt_rad.cos();

        // ── Front POA ────────────────────────────────────────────────────────
        let aoi_deg = self.angle_of_incidence(irr.zenith_deg, irr.azimuth_deg);
        let aoi_rad = aoi_deg.to_radians();
        let g_direct = irr.dni * aoi_rad.cos().max(0.0);
        let g_diff_front = irr.dhi * (1.0 + cos_tilt) / 2.0;
        let g_refl_front = irr.ghi * self.ground.albedo * (1.0 - cos_tilt) / 2.0;
        let front_poa = (g_direct + g_diff_front + g_refl_front).max(0.0);

        // ── Rear irradiance ──────────────────────────────────────────────────
        // View factor from rear face to sky: (1 - cos(tilt)) / 2
        let vf_rear_sky = (1.0 - cos_tilt) / 2.0;
        let rear_sky = irr.dhi * vf_rear_sky;

        let sf = self.shadow_fraction(irr.zenith_deg);
        // Rear view factor to ground: same as front view factor to sky by reciprocity.
        let vf_rear_ground = vf_rear_sky;
        let rear_ground = irr.ghi * self.ground.albedo * vf_rear_ground * (1.0 - sf);

        let rear_total = (rear_sky + rear_ground).max(0.0);

        // ── Effective & gain ─────────────────────────────────────────────────
        let effective_irradiance = front_poa + self.module.bifaciality * rear_total;
        let bifacial_gain = if front_poa > 0.0 {
            (effective_irradiance - front_poa) / front_poa
        } else {
            0.0
        };

        BifacialIrradiance {
            front_poa,
            rear_ground,
            rear_sky,
            rear_total,
            effective_irradiance,
            bifacial_gain,
            shadow_fraction: sf,
        }
    }

    /// Computes instantaneous AC power output (W) for all modules.
    ///
    /// # Temperature model
    /// `T_cell = T_amb + G_front * (NOCT − 20) / 800`
    ///
    /// # Power model
    /// `P = P_STC * (G_eff / 1000) * (1 + temp_coeff / 100 * (T_cell − 25)) * N`
    ///
    /// Output is clamped to ≥ 0 W.
    pub fn compute_power_output(&self, irr: &BifacialIrradiance, t_amb_c: f64) -> f64 {
        let t_cell = t_amb_c + irr.front_poa * (self.module.noct_c - 20.0) / 800.0;
        let temp_factor = 1.0 + self.module.temp_coeff_pmax / 100.0 * (t_cell - 25.0);
        let power = self.module.p_max_stc_w
            * (irr.effective_irradiance / 1000.0)
            * temp_factor
            * self.module.n_modules as f64;
        power.max(0.0)
    }

    /// Runs an annual (or multi-year) yield simulation over the provided hourly data.
    ///
    /// Returns both monofacial and bifacial annual energies, monthly breakdowns,
    /// geometry optimisation results, and a simplified LCOE improvement estimate.
    pub fn analyze_annual_yield(&self, inputs: &AnnualYieldInput) -> BifacialYieldResult {
        let n = inputs.hourly_ghi.len();
        let mut mono_kwh = 0.0_f64;
        let mut bif_kwh = 0.0_f64;
        // 12 months; distribute hours uniformly (730 h/month).
        let mut monthly_kwh = vec![0.0_f64; 12];

        for i in 0..n {
            let irr = IrradianceComponents {
                ghi: inputs.hourly_ghi[i],
                dhi: inputs.hourly_dhi[i],
                dni: inputs.hourly_dni[i],
                zenith_deg: inputs.hourly_zenith_deg[i],
                azimuth_deg: inputs.hourly_azimuth_deg[i],
            };
            let t_amb = inputs.hourly_tamb_c.get(i).copied().unwrap_or(20.0);

            let bifacial_irr = self.compute_irradiance(&irr);

            // Monofacial: same module but rear contribution is zero.
            let mono_irr = BifacialIrradiance {
                effective_irradiance: bifacial_irr.front_poa,
                ..bifacial_irr.clone()
            };
            let p_mono = self.compute_power_output(&mono_irr, t_amb);
            let p_bif = self.compute_power_output(&bifacial_irr, t_amb);

            // Convert W → kWh (1 h timestep assumed).
            mono_kwh += p_mono / 1000.0;
            bif_kwh += p_bif / 1000.0;

            // Assign to month: uniform 730-h/month distribution.
            let month_idx = ((i * 12) / n.max(1)).min(11);
            monthly_kwh[month_idx] += p_bif / 1000.0;
        }

        let bifacial_gain_pct = if mono_kwh > 0.0 {
            (bif_kwh - mono_kwh) / mono_kwh * 100.0
        } else {
            0.0
        };

        // Simplified LCOE improvement: bifacial gain reduces $/kWh proportionally.
        let lcoe_improvement_pct = bifacial_gain_pct * 0.8;

        // Geometry optimisation.
        let opt_geom = self.optimize_geometry(inputs);

        BifacialYieldResult {
            monofacial_kwh_yr: mono_kwh,
            bifacial_kwh_yr: bif_kwh,
            bifacial_gain_pct,
            monthly_kwh,
            optimal_tilt_deg: opt_geom.tilt_deg,
            optimal_height_m: opt_geom.height_m,
            optimal_gcr: opt_geom.gcr,
            lcoe_improvement_pct,
        }
    }

    /// Computes hourly shading fractions and derived annual shading metrics.
    ///
    /// Also finds the minimum row pitch that keeps GHI-weighted annual shading
    /// loss below 5 %.
    pub fn analyze_shading(&self, inputs: &AnnualYieldInput) -> ShadingAnalysis {
        let n = inputs.hourly_ghi.len();
        let mut hourly_shade_fraction = Vec::with_capacity(n);
        let mut ghi_sum = 0.0_f64;
        let mut shaded_ghi_sum = 0.0_f64;

        for i in 0..n {
            let sf = self.shadow_fraction(inputs.hourly_zenith_deg[i]);
            hourly_shade_fraction.push(sf.clamp(0.0, 1.0));
            let g = inputs.hourly_ghi[i].max(0.0);
            ghi_sum += g;
            shaded_ghi_sum += g * sf;
        }

        let annual_shading_loss_pct = if ghi_sum > 0.0 {
            shaded_ghi_sum / ghi_sum * 100.0
        } else {
            0.0
        };

        // Find minimum pitch for < 5 % loss by scanning from current pitch upward.
        let min_pitch_for_5pct_loss_m = self.find_min_pitch_for_loss_target(inputs, 5.0);

        ShadingAnalysis {
            hourly_shade_fraction,
            annual_shading_loss_pct,
            min_pitch_for_5pct_loss_m,
        }
    }

    /// Grid-searches over tilt, height, and GCR to maximise annual bifacial yield.
    ///
    /// Search ranges:
    /// - Tilt: 20°, 25°, 30°, 35°, 40°
    /// - Height: 0.50, 0.75, 1.00, 1.25, 1.50, 1.75, 2.00 m
    /// - GCR: 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60
    ///
    /// Returns the [`BifacialGeometry`] that maximises yield.
    pub fn optimize_geometry(&self, inputs: &AnnualYieldInput) -> BifacialGeometry {
        let tilts = [20.0_f64, 25.0, 30.0, 35.0, 40.0];
        let heights = [0.50_f64, 0.75, 1.00, 1.25, 1.50, 1.75, 2.00];
        let gcrs = [0.30_f64, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60];

        let mut best_kwh = f64::NEG_INFINITY;
        let mut best_geom = self.geometry.clone();

        for &tilt in &tilts {
            for &height in &heights {
                for &gcr in &gcrs {
                    // Row pitch derived from GCR: pitch = module_length * cos(tilt) / gcr.
                    let pitch = (self.geometry.module_length_m * tilt.to_radians().cos()
                        / gcr.max(0.01))
                    .max(self.geometry.module_length_m);

                    let candidate_geom = BifacialGeometry {
                        height_m: height,
                        tilt_deg: tilt,
                        module_length_m: self.geometry.module_length_m,
                        module_width_m: self.geometry.module_width_m,
                        row_pitch_m: pitch,
                        n_rows: self.geometry.n_rows,
                        gcr,
                    };
                    let candidate = BifacialPvSystem {
                        geometry: candidate_geom.clone(),
                        module: self.module.clone(),
                        ground: self.ground.clone(),
                    };
                    let result = candidate.analyze_annual_yield_fast(inputs);
                    if result > best_kwh {
                        best_kwh = result;
                        best_geom = candidate_geom;
                    }
                }
            }
        }
        best_geom
    }

    // ── Internal fast yield (no re-optimisation) ──────────────────────────────

    /// Computes annual bifacial energy (kWh) without geometry optimisation.
    ///
    /// Used internally by [`optimize_geometry`] to avoid infinite recursion.
    fn analyze_annual_yield_fast(&self, inputs: &AnnualYieldInput) -> f64 {
        let n = inputs.hourly_ghi.len();
        let mut bif_kwh = 0.0_f64;
        for i in 0..n {
            let irr = IrradianceComponents {
                ghi: inputs.hourly_ghi[i],
                dhi: inputs.hourly_dhi[i],
                dni: inputs.hourly_dni[i],
                zenith_deg: inputs.hourly_zenith_deg[i],
                azimuth_deg: inputs.hourly_azimuth_deg[i],
            };
            let t_amb = inputs.hourly_tamb_c.get(i).copied().unwrap_or(20.0);
            let bifacial_irr = self.compute_irradiance(&irr);
            let p_bif = self.compute_power_output(&bifacial_irr, t_amb);
            bif_kwh += p_bif / 1000.0;
        }
        bif_kwh
    }

    /// Finds the minimum row pitch (m) that achieves a GHI-weighted annual
    /// shading loss strictly below `target_pct` percent.
    ///
    /// Scans pitch from 1 m to 20 m in 0.5 m steps and returns the first
    /// pitch that satisfies the constraint (or 20 m if none found).
    fn find_min_pitch_for_loss_target(&self, inputs: &AnnualYieldInput, target_pct: f64) -> f64 {
        let n = inputs.hourly_ghi.len();
        let mut pitch = 1.0_f64;
        while pitch <= 20.0 {
            let candidate = BifacialPvSystem {
                geometry: BifacialGeometry {
                    row_pitch_m: pitch,
                    ..self.geometry.clone()
                },
                module: self.module.clone(),
                ground: self.ground.clone(),
            };
            let mut ghi_sum = 0.0_f64;
            let mut shaded_sum = 0.0_f64;
            for i in 0..n {
                let sf = candidate.shadow_fraction(inputs.hourly_zenith_deg[i]);
                let g = inputs.hourly_ghi[i].max(0.0);
                ghi_sum += g;
                shaded_sum += g * sf;
            }
            let loss_pct = if ghi_sum > 0.0 {
                shaded_sum / ghi_sum * 100.0
            } else {
                0.0
            };
            if loss_pct < target_pct {
                return pitch;
            }
            pitch += 0.5;
        }
        20.0
    }
}

// ─── Extended bifacial model (BifacialPvModel API) ───────────────────────────

/// Ground albedo type with standard surface presets.
#[derive(Debug, Clone, PartialEq)]
pub enum AlbedoType {
    /// Short grass (~0.20)
    Grass,
    /// Portland cement concrete (~0.35)
    Concrete,
    /// Asphalt surface (~0.15)
    Asphalt,
    /// Dry sand (~0.30)
    Sand,
    /// Fresh snow (~0.80)
    Snow,
    /// White painted surface (~0.65)
    White,
    /// Custom albedo value in [0, 1]
    Custom(f64),
}

impl AlbedoType {
    /// Returns the broadband albedo coefficient [0, 1] for this surface.
    pub fn albedo_value(&self) -> f64 {
        match self {
            AlbedoType::Grass => 0.20,
            AlbedoType::Concrete => 0.35,
            AlbedoType::Asphalt => 0.15,
            AlbedoType::Sand => 0.30,
            AlbedoType::Snow => 0.80,
            AlbedoType::White => 0.65,
            AlbedoType::Custom(v) => *v,
        }
    }
}

/// Bifacial module electrical and physical configuration.
#[derive(Debug, Clone)]
pub struct BifacialModuleConfig {
    /// Front-side STC power (W).
    pub p_stc_front_w: f64,
    /// Bifaciality factor φ (typically 0.65–0.85).
    pub bifaciality_factor: f64,
    /// Module width in the direction perpendicular to tilt (m).
    pub module_width_m: f64,
    /// Module height in the tilt direction (m).
    pub module_height_m: f64,
    /// Temperature coefficient of P_max (%/°C, negative for Si).
    pub temp_coeff_pct_per_c: f64,
    /// Nominal Operating Cell Temperature (°C).
    pub noct_c: f64,
    /// Frame absorption fraction [0, 1] (typical 0.03).
    pub frame_absorption: f64,
}

/// Rack / mounting configuration for a bifacial array row.
#[derive(Debug, Clone)]
pub struct RackConfig {
    /// Hub height — centre of module above ground (m).
    pub hub_height_m: f64,
    /// Ground clearance — bottom of module above ground (m).
    pub ground_clearance_m: f64,
    /// Module tilt angle from horizontal (degrees).
    pub tilt_deg: f64,
    /// Module azimuth (degrees; 0 = south, 90 = west).
    pub azimuth_deg: f64,
    /// Distance between row centres (m).
    pub row_pitch_m: f64,
    /// Number of rows used for shading / view-factor calculation.
    pub n_rows: usize,
    /// `true` if the array uses single-axis tracking.
    pub is_tracking: bool,
}

/// Instantaneous solar irradiance components and meteorological inputs.
#[derive(Debug, Clone)]
pub struct IrradianceInputs {
    /// Global horizontal irradiance (W/m²).
    pub ghi: f64,
    /// Direct normal irradiance (W/m²).
    pub dni: f64,
    /// Diffuse horizontal irradiance (W/m²).
    pub dhi: f64,
    /// Solar zenith angle (degrees).
    pub solar_zenith_deg: f64,
    /// Solar azimuth angle (degrees).
    pub solar_azimuth_deg: f64,
    /// Ambient temperature (°C).
    pub t_amb_c: f64,
    /// Wind speed (m/s).
    pub wind_speed_ms: f64,
}

/// Near-field and far-field albedo configuration for a bifacial array.
#[derive(Debug, Clone)]
pub struct AlbedoConfig {
    /// Surface type determining the nominal albedo.
    pub albedo_type: AlbedoType,
    /// Near-field albedo (ground directly under the array).
    pub near_field_albedo: f64,
    /// Far-field albedo (surrounding ground beyond the array).
    pub far_field_albedo: f64,
}

/// View factors between module surfaces, sky, and ground.
#[derive(Debug, Clone)]
pub struct ViewFactors {
    /// View factor from rear surface to sky [0, 1].
    pub rear_sky_vf: f64,
    /// View factor from rear surface to ground [0, 1].
    pub rear_ground_vf: f64,
    /// View factor from front surface to sky [0, 1].
    pub front_sky_vf: f64,
    /// Effective ground view factor accounting for row shading.
    pub effective_ground_vf: f64,
}

/// Bifacial irradiance decomposition for a single timestep.
#[derive(Debug, Clone)]
pub struct BifacialIrradianceExt {
    /// Front plane-of-array irradiance (W/m²).
    pub front_poa_w_m2: f64,
    /// Total rear irradiance (W/m²).
    pub rear_irradiance_w_m2: f64,
    /// Effective bifacial irradiance (W/m²).
    pub effective_irradiance_w_m2: f64,
    /// Front beam (direct) component (W/m²).
    pub front_beam: f64,
    /// Front sky-diffuse component (W/m²).
    pub front_diffuse: f64,
    /// Front ground-reflected component (W/m²).
    pub front_ground_reflected: f64,
    /// Rear sky-diffuse component (W/m²).
    pub rear_sky_diffuse: f64,
    /// Rear ground-reflected contribution (W/m²).
    pub rear_ground_contribution: f64,
}

/// Single-timestep energy output from the bifacial model.
#[derive(Debug, Clone)]
pub struct BifacialEnergyResult {
    /// Irradiance decomposition.
    pub irradiance: BifacialIrradianceExt,
    /// View factors used in the calculation.
    pub view_factors: ViewFactors,
    /// Module (cell) temperature (°C).
    pub module_temp_c: f64,
    /// DC power output per module (W).
    pub p_dc_w: f64,
    /// Bifacial gain — extra power vs monofacial (%).
    pub bifacial_gain_pct: f64,
    /// Temperature-correction multiplier applied to STC power.
    pub temp_correction: f64,
    /// Effective combined albedo used in the calculation.
    pub effective_albedo: f64,
}

/// Annual yield simulation result.
#[derive(Debug, Clone)]
pub struct BifacialAnnualResult {
    /// Specific annual yield (kWh per kWp installed).
    pub annual_yield_kwh_per_kw: f64,
    /// Annual bifacial gain vs equivalent monofacial (%).
    pub bifacial_gain_annual_pct: f64,
    /// Performance ratio [0, 1].
    pub performance_ratio: f64,
    /// Monthly energy yield (kWh), 12 entries Jan–Dec.
    pub monthly_energy_kwh: [f64; 12],
    /// Monthly bifacial gain (%), 12 entries.
    pub monthly_bifacial_gain_pct: [f64; 12],
}

/// Comprehensive bifacial PV model combining module, rack, and albedo.
#[derive(Debug, Clone)]
pub struct BifacialPvModel {
    /// Module electrical and physical parameters.
    pub module: BifacialModuleConfig,
    /// Rack / mounting geometry.
    pub rack: RackConfig,
    /// Ground albedo configuration.
    pub albedo: AlbedoConfig,
}

impl BifacialPvModel {
    /// Creates a new [`BifacialPvModel`].
    pub fn new(module: BifacialModuleConfig, rack: RackConfig, albedo: AlbedoConfig) -> Self {
        BifacialPvModel {
            module,
            rack,
            albedo,
        }
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    /// Cosine of the angle of incidence on the front surface.
    ///
    /// `cos(AOI) = sin(z)*cos(az_sun - az_mod)*sin(tilt) + cos(z)*cos(tilt)`
    fn angle_of_incidence_cos(&self, inputs: &IrradianceInputs) -> f64 {
        let tilt = self.rack.tilt_deg.to_radians();
        let zenith = inputs.solar_zenith_deg.to_radians();
        let az_diff = (inputs.solar_azimuth_deg - self.rack.azimuth_deg).to_radians();

        let cos_aoi = zenith.sin() * az_diff.cos() * tilt.sin() + zenith.cos() * tilt.cos();
        cos_aoi.clamp(-1.0, 1.0)
    }

    /// Ground coverage ratio: `GCR = module_height * cos(tilt) / row_pitch`.
    fn ground_coverage_ratio(&self) -> f64 {
        let tilt_cos = self.rack.tilt_deg.to_radians().cos();
        (self.module.module_height_m * tilt_cos / self.rack.row_pitch_m.max(1e-9)).clamp(0.0, 1.0)
    }

    /// Module temperature using the Sandia NOCT model with wind correction.
    ///
    /// `T_cell = T_amb + (NOCT - 20) / 800 * I_eff * (1 - eta) / (1 + u_c * wind)`
    ///
    /// A wind correction factor `u_c = 0.05` provides a ~5 % cooling per m/s.
    fn module_temperature(&self, irradiance: f64, t_amb: f64, wind: f64) -> f64 {
        let eta = 0.20_f64; // approximate efficiency for thermal calculation
        let u_c = 0.05_f64; // wind correction factor
        let wind_factor = 1.0 + u_c * wind.max(0.0);
        t_amb
            + (self.module.noct_c - 20.0) / 800.0 * irradiance.max(0.0) * (1.0 - eta)
                / wind_factor.max(0.1)
    }

    /// Approximate solar zenith for a given month, hour, and latitude.
    ///
    /// Uses a simple declination-based formula (Spencer 1971 approximation).
    fn compute_monthly_zenith(month: usize, hour: f64, lat_deg: f64) -> f64 {
        // Day-of-year for mid-month (approximate).
        let doy = (month as f64 * 30.44 - 15.0).clamp(1.0, 365.0);
        let b = 2.0 * PI * (doy - 1.0) / 365.0;
        // Solar declination (radians).
        let decl = (0.006918 - 0.399912 * b.cos() + 0.070257 * b.sin()
            - 0.006758 * (2.0 * b).cos()
            + 0.000907 * (2.0 * b).sin())
        .clamp(-0.4093, 0.4093); // ±23.45°

        let lat = lat_deg.to_radians();
        // Hour angle (solar noon = 0, ±15° per hour).
        let ha = ((hour - 12.0) * 15.0).to_radians();
        let cos_z = lat.sin() * decl.sin() + lat.cos() * decl.cos() * ha.cos();
        cos_z.clamp(-1.0, 1.0).acos().to_degrees()
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /// Computes view factors for the rear and front surfaces.
    ///
    /// Uses the infinite parallel row approximation:
    /// - Rear-sky VF: `(1 + cos(π − tilt)) / 2`
    /// - Rear-ground VF: `1 − rear_sky_vf`
    /// - Front-sky VF: `(1 + cos(tilt)) / 2`
    pub fn compute_view_factors(&self) -> ViewFactors {
        let tilt_rad = self.rack.tilt_deg.to_radians();
        let rear_sky_vf = (1.0 + (PI - tilt_rad).cos()) / 2.0;
        let rear_ground_vf = 1.0 - rear_sky_vf;
        let front_sky_vf = (1.0 + tilt_rad.cos()) / 2.0;

        // Effective ground VF reduced by shading from adjacent rows.
        let gcr = self.ground_coverage_ratio();
        let shade_fraction = (gcr * tilt_rad.cos()).clamp(0.0, 1.0);
        let effective_ground_vf = rear_ground_vf * (1.0 - shade_fraction * 0.9);

        ViewFactors {
            rear_sky_vf,
            rear_ground_vf,
            front_sky_vf,
            effective_ground_vf,
        }
    }

    /// Computes the bifacial irradiance decomposition for one timestep.
    ///
    /// Uses the Liu & Jordan isotropic-sky model for the front POA and the
    /// view-factor method for the rear irradiance.
    pub fn compute_irradiance(
        &self,
        inputs: &IrradianceInputs,
    ) -> Result<BifacialIrradianceExt, String> {
        if inputs.solar_zenith_deg < 0.0 || inputs.solar_zenith_deg > 180.0 {
            return Err(format!(
                "solar_zenith_deg {} out of [0, 180]",
                inputs.solar_zenith_deg
            ));
        }

        let tilt_rad = self.rack.tilt_deg.to_radians();
        let cos_tilt = tilt_rad.cos();

        // Effective albedo: weighted average of near and far field.
        let base_albedo = self.albedo.albedo_type.albedo_value();
        let effective_albedo = 0.5
            * (self.albedo.near_field_albedo + self.albedo.far_field_albedo).clamp(0.0, 1.0)
            * (base_albedo / base_albedo.max(1e-9));
        let effective_albedo = effective_albedo.clamp(0.0, 1.0);

        // ── Front POA ────────────────────────────────────────────────────────
        let cos_aoi = self.angle_of_incidence_cos(inputs);
        let front_beam = (inputs.dni * cos_aoi.max(0.0)).max(0.0);
        let front_diffuse = inputs.dhi * (1.0 + cos_tilt) / 2.0;
        let front_ground_reflected = inputs.ghi * effective_albedo * (1.0 - cos_tilt) / 2.0;
        let front_poa = (front_beam + front_diffuse + front_ground_reflected).max(0.0);

        // ── Rear irradiance ──────────────────────────────────────────────────
        let vf = self.compute_view_factors();
        // Rear-sky: DHI attenuated by frame absorption.
        let dhi_rear = inputs.dhi * (1.0 - self.module.frame_absorption);
        let rear_sky_diffuse = dhi_rear * vf.rear_sky_vf;

        // Rear-ground: irradiance reflected from ground under/around array.
        let i_ground = inputs.ghi * effective_albedo;
        let rear_ground_contribution = i_ground * vf.effective_ground_vf;

        let rear_irradiance = (rear_sky_diffuse + rear_ground_contribution).max(0.0);

        // ── Effective bifacial irradiance ─────────────────────────────────────
        let effective_irradiance = front_poa + self.module.bifaciality_factor * rear_irradiance;

        Ok(BifacialIrradianceExt {
            front_poa_w_m2: front_poa,
            rear_irradiance_w_m2: rear_irradiance,
            effective_irradiance_w_m2: effective_irradiance,
            front_beam,
            front_diffuse,
            front_ground_reflected,
            rear_sky_diffuse,
            rear_ground_contribution,
        })
    }

    /// Computes single-timestep DC energy output.
    pub fn compute_energy(
        &self,
        inputs: &IrradianceInputs,
    ) -> Result<BifacialEnergyResult, String> {
        let irradiance = self.compute_irradiance(inputs)?;
        let vf = self.compute_view_factors();

        let i_eff = irradiance.effective_irradiance_w_m2;
        let module_temp_c = self.module_temperature(i_eff, inputs.t_amb_c, inputs.wind_speed_ms);

        let temp_correction =
            1.0 + self.module.temp_coeff_pct_per_c / 100.0 * (module_temp_c - 25.0);

        let p_dc_w = (self.module.p_stc_front_w * (i_eff / 1000.0) * temp_correction).max(0.0);

        let bifacial_gain_pct = if irradiance.front_poa_w_m2 > 1e-9 {
            self.module.bifaciality_factor * irradiance.rear_irradiance_w_m2
                / irradiance.front_poa_w_m2
                * 100.0
        } else {
            0.0
        };

        let effective_albedo = self.albedo.albedo_type.albedo_value();

        Ok(BifacialEnergyResult {
            irradiance,
            view_factors: vf,
            module_temp_c,
            p_dc_w,
            bifacial_gain_pct,
            temp_correction,
            effective_albedo,
        })
    }

    /// Simulates annual energy using 12 representative monthly conditions.
    ///
    /// For each month, samples 24 hourly zenith angles and integrates energy
    /// over daylight hours.  The monthly GHI/DNI/DHI values are used as the
    /// peak (noon) irradiance and scaled by a sinusoidal diurnal envelope.
    pub fn simulate_annual(
        &self,
        monthly_ghi: &[f64; 12],
        monthly_dni: &[f64; 12],
        monthly_dhi: &[f64; 12],
        monthly_t_amb: &[f64; 12],
        lat_deg: f64,
    ) -> Result<BifacialAnnualResult, String> {
        let mut monthly_energy_kwh = [0.0_f64; 12];
        let mut monthly_bifacial_gain_pct = [0.0_f64; 12];
        let mut annual_yield_kwh = 0.0_f64;
        let mut annual_mono_kwh = 0.0_f64;

        // Number of daylight hours sampled per month (24 h/day × ~30 days).
        let hours_per_month = 720_usize;

        for month in 0..12 {
            let peak_ghi = monthly_ghi[month];
            let peak_dni = monthly_dni[month];
            let peak_dhi = monthly_dhi[month];
            let t_amb = monthly_t_amb[month];

            let mut month_kwh = 0.0_f64;
            let mut month_mono_kwh = 0.0_f64;
            let mut gain_sum = 0.0_f64;
            let mut gain_count = 0_usize;

            for h in 0..hours_per_month {
                // Hour of day in [0, 24).
                let hour = (h % 24) as f64 + 0.5;
                // Diurnal scale factor: cosine bell centred on noon.
                let scale = ((PI * (hour - 6.0) / 12.0).sin()).max(0.0);

                let zenith = Self::compute_monthly_zenith(month + 1, hour, lat_deg);
                if zenith >= 90.0 {
                    continue; // below horizon
                }

                let ghi = peak_ghi * scale;
                let dni = peak_dni * scale;
                let dhi = peak_dhi * scale;

                let inp = IrradianceInputs {
                    ghi,
                    dni,
                    dhi,
                    solar_zenith_deg: zenith,
                    solar_azimuth_deg: 180.0, // south-facing
                    t_amb_c: t_amb,
                    wind_speed_ms: 2.0,
                };

                let result = self.compute_energy(&inp)?;
                // 1 hour timestep → W → kWh / 1000
                let kwh = result.p_dc_w / 1000.0;
                month_kwh += kwh;

                // Monofacial equivalent (bifaciality = 0).
                let mono_irr = result.irradiance.front_poa_w_m2;
                let mono_t = self.module_temperature(mono_irr, t_amb, 2.0);
                let mono_tc = 1.0 + self.module.temp_coeff_pct_per_c / 100.0 * (mono_t - 25.0);
                let mono_p = (self.module.p_stc_front_w * (mono_irr / 1000.0) * mono_tc).max(0.0);
                month_mono_kwh += mono_p / 1000.0;

                if result.bifacial_gain_pct.is_finite() {
                    gain_sum += result.bifacial_gain_pct;
                    gain_count += 1;
                }
            }

            monthly_energy_kwh[month] = month_kwh;
            monthly_bifacial_gain_pct[month] = if gain_count > 0 {
                gain_sum / gain_count as f64
            } else {
                0.0
            };
            annual_yield_kwh += month_kwh;
            annual_mono_kwh += month_mono_kwh;
        }

        let p_stc_kw = self.module.p_stc_front_w / 1000.0;
        let annual_yield_kwh_per_kw = if p_stc_kw > 1e-9 {
            annual_yield_kwh / p_stc_kw
        } else {
            0.0
        };

        let bifacial_gain_annual_pct = if annual_mono_kwh > 1e-9 {
            (annual_yield_kwh - annual_mono_kwh) / annual_mono_kwh * 100.0
        } else {
            0.0
        };

        // Performance ratio: actual yield vs ideal (GHI-based) yield.
        // PR = actual_kWh / (sum_GHI_kWh/m² * capacity_kW)
        let total_ghi_kwh: f64 = monthly_ghi.iter().map(|&g| g * 730.0 / 1000.0).sum();
        let performance_ratio = if total_ghi_kwh > 1e-9 && p_stc_kw > 1e-9 {
            (annual_yield_kwh / (total_ghi_kwh * p_stc_kw)).clamp(0.0, 1.0)
        } else {
            0.0
        };

        Ok(BifacialAnnualResult {
            annual_yield_kwh_per_kw,
            bifacial_gain_annual_pct,
            performance_ratio,
            monthly_energy_kwh,
            monthly_bifacial_gain_pct,
        })
    }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Helper builders ───────────────────────────────────────────────────────

    fn default_module() -> BifacialModuleParams {
        BifacialModuleParams {
            p_max_stc_w: 400.0,
            bifaciality: 0.75,
            temp_coeff_pmax: -0.40,
            noct_c: 45.0,
            module_area_m2: 2.0,
            n_modules: 10,
        }
    }

    fn default_system() -> BifacialPvSystem {
        BifacialPvSystem::new(
            BifacialGeometry::default_fixed_tilt(30.0),
            default_module(),
            GroundAlbedo::new(SurfaceType::Concrete),
        )
    }

    /// Builds a simplified 8 760-hour dataset with a diurnal sinusoidal pattern.
    fn synthetic_8760() -> AnnualYieldInput {
        let n = 8760_usize;
        let mut ghi = Vec::with_capacity(n);
        let mut dhi = Vec::with_capacity(n);
        let mut dni = Vec::with_capacity(n);
        let mut zenith = Vec::with_capacity(n);
        let mut azimuth = Vec::with_capacity(n);
        let mut tamb = Vec::with_capacity(n);

        for h in 0..n {
            // Hour-of-day 0..23
            let hod = (h % 24) as f64;
            // Simple diurnal model: sun rises at 6, sets at 18.
            let sun_fraction = ((hod - 12.0).abs() / 6.0).min(1.0);
            let daylight = 1.0 - sun_fraction;
            let g = (800.0 * daylight).max(0.0);
            let z = if daylight > 0.0 {
                (90.0 - 80.0 * daylight).clamp(10.0, 90.0)
            } else {
                90.0
            };
            ghi.push(g);
            dhi.push(g * 0.15);
            // DNI from GHI and zenith
            let z_rad = z.to_radians();
            let d = if z_rad.cos() > 0.01 {
                ((g - g * 0.15) / z_rad.cos()).clamp(0.0, 1200.0)
            } else {
                0.0
            };
            dni.push(d);
            zenith.push(z);
            azimuth.push(180.0); // south-facing
            tamb.push(15.0 + 10.0 * (hod / 24.0));
        }

        AnnualYieldInput {
            hourly_ghi: ghi,
            hourly_dhi: dhi,
            hourly_dni: dni,
            hourly_zenith_deg: zenith,
            hourly_azimuth_deg: azimuth,
            hourly_tamb_c: tamb,
        }
    }

    // ── Albedo tests ──────────────────────────────────────────────────────────

    #[test]
    fn test_concrete_albedo() {
        assert_eq!(SurfaceType::Concrete.albedo_value(), 0.25);
    }

    #[test]
    fn test_snow_albedo() {
        assert_eq!(SurfaceType::Snow.albedo_value(), 0.70);
    }

    #[test]
    fn test_custom_albedo() {
        assert_eq!(SurfaceType::Custom { albedo: 0.5 }.albedo_value(), 0.5);
    }

    #[test]
    fn test_ground_albedo_new() {
        let ga = GroundAlbedo::new(SurfaceType::Grass);
        assert!((ga.albedo - 0.20).abs() < 1e-9);
    }

    // ── Geometry tests ────────────────────────────────────────────────────────

    #[test]
    fn test_default_fixed_tilt() {
        let g = BifacialGeometry::default_fixed_tilt(25.0);
        assert_eq!(g.tilt_deg, 25.0);
        assert_eq!(g.gcr, 0.4);
        assert!(g.height_m > 0.0);
    }

    // ── AOI test ──────────────────────────────────────────────────────────────

    #[test]
    fn test_aoi_at_zero_zenith_zero_tilt() {
        let sys = BifacialPvSystem::new(
            BifacialGeometry::default_fixed_tilt(0.0),
            default_module(),
            GroundAlbedo::new(SurfaceType::Concrete),
        );
        // Sun overhead, flat module → AOI should be ≈ 0°.
        let aoi = sys.angle_of_incidence(0.0, 180.0);
        assert!(aoi < 1.0, "AOI={aoi}, expected < 1°");
    }

    // ── Irradiance component tests ────────────────────────────────────────────

    #[test]
    fn test_front_poa_positive() {
        let sys = default_system();
        let irr = IrradianceComponents {
            dni: 700.0,
            dhi: 100.0,
            ghi: 600.0,
            zenith_deg: 30.0,
            azimuth_deg: 180.0,
        };
        let result = sys.compute_irradiance(&irr);
        assert!(result.front_poa > 0.0);
    }

    #[test]
    fn test_rear_sky_formula() {
        let sys = default_system();
        let dhi = 120.0_f64;
        let tilt_rad = sys.geometry.tilt_deg.to_radians();
        let expected = dhi * (1.0 - tilt_rad.cos()) / 2.0;

        let irr = IrradianceComponents {
            dni: 0.0,
            dhi,
            ghi: 0.0,
            zenith_deg: 45.0,
            azimuth_deg: 180.0,
        };
        let result = sys.compute_irradiance(&irr);
        assert!(
            (result.rear_sky - expected).abs() < 1e-6,
            "rear_sky={}, expected={}",
            result.rear_sky,
            expected
        );
    }

    #[test]
    fn test_rear_ground_increases_with_albedo() {
        let irr_comp = IrradianceComponents {
            dni: 500.0,
            dhi: 80.0,
            ghi: 400.0,
            zenith_deg: 35.0,
            azimuth_deg: 180.0,
        };
        let geom = BifacialGeometry::default_fixed_tilt(30.0);
        let module = default_module();

        let sys_low = BifacialPvSystem::new(
            geom.clone(),
            module.clone(),
            GroundAlbedo::new(SurfaceType::Asphalt), // 0.12
        );
        let sys_high = BifacialPvSystem::new(
            geom,
            module,
            GroundAlbedo::new(SurfaceType::Snow), // 0.70
        );
        assert!(
            sys_high.compute_irradiance(&irr_comp).rear_ground
                > sys_low.compute_irradiance(&irr_comp).rear_ground
        );
    }

    #[test]
    fn test_bifacial_gain_positive() {
        let sys = default_system();
        let irr = IrradianceComponents {
            dni: 600.0,
            dhi: 100.0,
            ghi: 500.0,
            zenith_deg: 35.0,
            azimuth_deg: 180.0,
        };
        let result = sys.compute_irradiance(&irr);
        assert!(result.bifacial_gain > 0.0);
    }

    #[test]
    fn test_effective_gt_front() {
        let sys = default_system();
        let irr = IrradianceComponents {
            dni: 600.0,
            dhi: 100.0,
            ghi: 500.0,
            zenith_deg: 35.0,
            azimuth_deg: 180.0,
        };
        let result = sys.compute_irradiance(&irr);
        assert!(result.effective_irradiance > result.front_poa);
    }

    // ── Shadow fraction tests ─────────────────────────────────────────────────

    #[test]
    fn test_shadow_fraction_zero_high_sun() {
        let sys = default_system();
        // Very high sun (zenith = 10°) should cast a very short shadow.
        let sf = sys.shadow_fraction(10.0);
        // With height=0.5 m and row_pitch=4.0 m the shadow is negligible at 80° altitude.
        assert!(
            sf < 0.1,
            "shadow_fraction={sf} should be near 0 at high sun"
        );
    }

    #[test]
    fn test_shadow_fraction_increases_low_sun() {
        // Use a tight pitch (2.5 m) so that a low-sun shadow actually falls inside the row.
        // At zenith=75° (altitude=15°): shadow_length = 0.5/tan(15°) ≈ 1.87 m
        // proj_length = 2.0*cos(30°) ≈ 1.73 m, clearance = 2.5-1.73 = 0.77 m
        // shade = (1.87-0.77)/1.73 ≈ 0.64 > 0
        let sys_tight = BifacialPvSystem::new(
            BifacialGeometry {
                row_pitch_m: 2.5,
                ..BifacialGeometry::default_fixed_tilt(30.0)
            },
            default_module(),
            GroundAlbedo::new(SurfaceType::Concrete),
        );
        let sf_high = sys_tight.shadow_fraction(20.0);
        let sf_low = sys_tight.shadow_fraction(75.0);
        assert!(
            sf_low > sf_high,
            "shadow at zenith=75 ({sf_low}) should be > shadow at zenith=20 ({sf_high})"
        );
    }

    #[test]
    fn test_shadow_fraction_bounded() {
        let sys = default_system();
        for z in [0.0_f64, 30.0, 60.0, 89.0, 90.0, 100.0] {
            let sf = sys.shadow_fraction(z);
            assert!(
                (0.0..=1.0).contains(&sf),
                "shadow_fraction={sf} out of [0,1] for zenith={z}"
            );
        }
    }

    // ── Power output tests ────────────────────────────────────────────────────

    #[test]
    fn test_power_output_positive() {
        let sys = default_system();
        let irr = IrradianceComponents {
            dni: 700.0,
            dhi: 100.0,
            ghi: 600.0,
            zenith_deg: 30.0,
            azimuth_deg: 180.0,
        };
        let bifacial_irr = sys.compute_irradiance(&irr);
        let power = sys.compute_power_output(&bifacial_irr, 25.0);
        assert!(power > 0.0);
    }

    #[test]
    fn test_temperature_correction() {
        let sys = default_system();
        let irr = IrradianceComponents {
            dni: 700.0,
            dhi: 100.0,
            ghi: 600.0,
            zenith_deg: 30.0,
            azimuth_deg: 180.0,
        };
        let bifacial_irr = sys.compute_irradiance(&irr);

        // At cold ambient temperature, temperature de-rating is less severe →
        // power should be higher than at hot temperature (negative temp_coeff).
        let p_cold = sys.compute_power_output(&bifacial_irr, 0.0);
        let p_hot = sys.compute_power_output(&bifacial_irr, 40.0);
        assert!(
            p_cold > p_hot,
            "cold power ({p_cold:.1} W) should exceed hot power ({p_hot:.1} W)"
        );
    }

    // ── Annual yield tests ────────────────────────────────────────────────────

    #[test]
    fn test_annual_yield_bifacial_gt_mono() {
        let sys = default_system();
        let inputs = synthetic_8760();
        let result = sys.analyze_annual_yield(&inputs);
        assert!(
            result.bifacial_kwh_yr > result.monofacial_kwh_yr,
            "bifacial={:.1} kWh, mono={:.1} kWh",
            result.bifacial_kwh_yr,
            result.monofacial_kwh_yr
        );
    }

    #[test]
    fn test_bifacial_gain_pct_positive() {
        let sys = default_system();
        let inputs = synthetic_8760();
        let result = sys.analyze_annual_yield(&inputs);
        assert!(result.bifacial_gain_pct > 0.0);
    }

    #[test]
    fn test_monthly_yields_sum() {
        let sys = default_system();
        let inputs = synthetic_8760();
        let result = sys.analyze_annual_yield(&inputs);
        let monthly_sum: f64 = result.monthly_kwh.iter().sum();
        let ratio = (monthly_sum - result.bifacial_kwh_yr).abs() / result.bifacial_kwh_yr.max(1.0);
        assert!(
            ratio < 0.01,
            "monthly sum {monthly_sum:.1} differs from annual {:.1} by {:.2}%",
            result.bifacial_kwh_yr,
            ratio * 100.0
        );
    }

    #[test]
    fn test_8760_hours_coverage() {
        let sys = default_system();
        let inputs = synthetic_8760();
        let result = sys.analyze_annual_yield(&inputs);
        assert_eq!(result.monthly_kwh.len(), 12);
    }

    // ── Shading tests ─────────────────────────────────────────────────────────

    #[test]
    fn test_shading_analysis_bounded() {
        let sys = default_system();
        let inputs = synthetic_8760();
        let analysis = sys.analyze_shading(&inputs);
        for (i, &sf) in analysis.hourly_shade_fraction.iter().enumerate() {
            assert!(
                (0.0..=1.0).contains(&sf),
                "hourly_shade_fraction[{i}] = {sf} out of [0,1]"
            );
        }
    }

    #[test]
    fn test_larger_pitch_less_shading() {
        let inputs = synthetic_8760();

        let make_sys = |pitch: f64| {
            BifacialPvSystem::new(
                BifacialGeometry {
                    row_pitch_m: pitch,
                    ..BifacialGeometry::default_fixed_tilt(30.0)
                },
                default_module(),
                GroundAlbedo::new(SurfaceType::Concrete),
            )
        };

        let loss_small = make_sys(3.0)
            .analyze_shading(&inputs)
            .annual_shading_loss_pct;
        let loss_large = make_sys(8.0)
            .analyze_shading(&inputs)
            .annual_shading_loss_pct;
        assert!(
            loss_large <= loss_small,
            "larger pitch should reduce shading: small={loss_small:.2}% large={loss_large:.2}%"
        );
    }

    // ── Geometry optimisation tests ───────────────────────────────────────────

    #[test]
    fn test_geometry_optimization() {
        let sys = default_system();
        let inputs = synthetic_8760();
        let opt = sys.optimize_geometry(&inputs);
        assert!(
            (20.0..=40.0).contains(&opt.tilt_deg),
            "optimal_tilt_deg={} not in [20,40]",
            opt.tilt_deg
        );
    }

    #[test]
    fn test_optimal_height_positive() {
        let sys = default_system();
        let inputs = synthetic_8760();
        let result = sys.analyze_annual_yield(&inputs);
        assert!(result.optimal_height_m > 0.0);
    }

    #[test]
    fn test_lcoe_improvement_positive() {
        let sys = default_system();
        let inputs = synthetic_8760();
        let result = sys.analyze_annual_yield(&inputs);
        assert!(result.lcoe_improvement_pct > 0.0);
    }

    // ── BifacialPvModel tests ─────────────────────────────────────────────────

    fn make_model(tilt: f64, bifaciality: f64, albedo: AlbedoType) -> BifacialPvModel {
        let albedo_val = albedo.albedo_value();
        BifacialPvModel::new(
            BifacialModuleConfig {
                p_stc_front_w: 400.0,
                bifaciality_factor: bifaciality,
                module_width_m: 1.0,
                module_height_m: 2.0,
                temp_coeff_pct_per_c: -0.40,
                noct_c: 45.0,
                frame_absorption: 0.03,
            },
            RackConfig {
                hub_height_m: 1.5,
                ground_clearance_m: 0.5,
                tilt_deg: tilt,
                azimuth_deg: 0.0, // south in this convention
                row_pitch_m: 5.0,
                n_rows: 10,
                is_tracking: false,
            },
            AlbedoConfig {
                albedo_type: albedo,
                near_field_albedo: albedo_val,
                far_field_albedo: albedo_val,
            },
        )
    }

    fn standard_inputs(ghi: f64) -> IrradianceInputs {
        IrradianceInputs {
            ghi,
            dni: ghi * 0.85,
            dhi: ghi * 0.15,
            solar_zenith_deg: 30.0,
            solar_azimuth_deg: 180.0,
            t_amb_c: 25.0,
            wind_speed_ms: 1.0,
        }
    }

    #[test]
    fn test_flat_tilt_front_poa_approx_ghi() {
        // At 0° tilt and overhead sun (zenith≈0) front POA ≈ GHI.
        let model = make_model(0.0, 0.75, AlbedoType::Grass);
        let inp = IrradianceInputs {
            ghi: 800.0,
            dni: 700.0,
            dhi: 100.0,
            solar_zenith_deg: 0.001, // near overhead
            solar_azimuth_deg: 180.0,
            t_amb_c: 25.0,
            wind_speed_ms: 1.0,
        };
        let irr = model.compute_irradiance(&inp).expect("irradiance ok");
        // At flat tilt sky-diffuse ≈ DHI, beam ≈ DNI, ground reflected ≈ 0.
        // front_poa ≈ DNI + DHI = 800 → within 20% of GHI=800.
        assert!(
            irr.front_poa_w_m2 > 600.0,
            "front_poa={}",
            irr.front_poa_w_m2
        );
    }

    #[test]
    fn test_vertical_tilt_no_ground_reflected() {
        // At 0° tilt (flat): (1 - cos(0°)) / 2 = 0 → ground-reflected = 0.
        let model = make_model(0.0, 0.75, AlbedoType::Concrete);
        let inp = standard_inputs(600.0);
        let irr = model.compute_irradiance(&inp).expect("irradiance ok");
        assert!(
            irr.front_ground_reflected < 1e-9,
            "ground_reflected={} should be 0 at 0° tilt (flat panel sees no ground)",
            irr.front_ground_reflected
        );
    }

    #[test]
    fn test_normal_incidence_cos_aoi_unity() {
        // When tilt = zenith and azimuth matches → AOI ≈ 0 → cos(AOI) ≈ 1.
        let model = make_model(30.0, 0.75, AlbedoType::Grass);
        // Sun at zenith=30°, module tilted 30° south (azimuth_deg=0 in model).
        // With solar_azimuth = 0 (south=0 in our model convention) cos_aoi ≈ 1.
        let inp = IrradianceInputs {
            ghi: 800.0,
            dni: 700.0,
            dhi: 100.0,
            solar_zenith_deg: 30.0,
            solar_azimuth_deg: 0.0, // same as rack azimuth
            t_amb_c: 25.0,
            wind_speed_ms: 1.0,
        };
        let irr = model.compute_irradiance(&inp).expect("irradiance ok");
        // beam = DNI * cos(AOI); at near-normal AOI beam ≈ DNI = 700
        assert!(irr.front_beam > 600.0, "front_beam={}", irr.front_beam);
    }

    #[test]
    fn test_bifaciality_zero_effective_equals_front() {
        let model = make_model(30.0, 0.0, AlbedoType::Concrete);
        let inp = standard_inputs(600.0);
        let irr = model.compute_irradiance(&inp).expect("irradiance ok");
        assert!(
            (irr.effective_irradiance_w_m2 - irr.front_poa_w_m2).abs() < 1e-9,
            "with bifaciality=0 effective={} front={}",
            irr.effective_irradiance_w_m2,
            irr.front_poa_w_m2
        );
    }

    #[test]
    fn test_bifaciality_one_effective_equals_front_plus_rear() {
        let model = make_model(30.0, 1.0, AlbedoType::Concrete);
        let inp = standard_inputs(600.0);
        let irr = model.compute_irradiance(&inp).expect("irradiance ok");
        let expected = irr.front_poa_w_m2 + irr.rear_irradiance_w_m2;
        assert!(
            (irr.effective_irradiance_w_m2 - expected).abs() < 1e-9,
            "bifaciality=1: effective={} front+rear={}",
            irr.effective_irradiance_w_m2,
            expected
        );
    }

    #[test]
    fn test_snow_albedo_higher_rear_than_grass() {
        let model_snow = make_model(30.0, 0.75, AlbedoType::Snow);
        let model_grass = make_model(30.0, 0.75, AlbedoType::Grass);
        let inp = standard_inputs(600.0);
        let rear_snow = model_snow
            .compute_irradiance(&inp)
            .expect("ok")
            .rear_irradiance_w_m2;
        let rear_grass = model_grass
            .compute_irradiance(&inp)
            .expect("ok")
            .rear_irradiance_w_m2;
        assert!(
            rear_snow > rear_grass,
            "snow rear={rear_snow} grass rear={rear_grass}"
        );
    }

    #[test]
    fn test_module_temp_higher_irradiance_higher_temp() {
        let model = make_model(30.0, 0.75, AlbedoType::Grass);
        let t_low = model.module_temperature(200.0, 20.0, 1.0);
        let t_high = model.module_temperature(900.0, 20.0, 1.0);
        assert!(t_high > t_low, "t_high={t_high} t_low={t_low}");
    }

    #[test]
    fn test_module_temp_higher_wind_lower_temp() {
        let model = make_model(30.0, 0.75, AlbedoType::Grass);
        let t_calm = model.module_temperature(800.0, 25.0, 0.0);
        let t_windy = model.module_temperature(800.0, 25.0, 10.0);
        assert!(t_windy < t_calm, "t_windy={t_windy} t_calm={t_calm}");
    }

    #[test]
    fn test_p_dc_zero_when_ghi_zero() {
        let model = make_model(30.0, 0.75, AlbedoType::Concrete);
        let inp = standard_inputs(0.0);
        let res = model.compute_energy(&inp).expect("ok");
        assert!(res.p_dc_w < 1e-9, "p_dc={}", res.p_dc_w);
    }

    #[test]
    fn test_p_dc_positive_when_ghi_positive() {
        let model = make_model(30.0, 0.75, AlbedoType::Concrete);
        let inp = standard_inputs(600.0);
        let res = model.compute_energy(&inp).expect("ok");
        assert!(res.p_dc_w > 0.0, "p_dc={}", res.p_dc_w);
    }

    #[test]
    fn test_bifacial_gain_positive_when_rear_nonzero() {
        let model = make_model(30.0, 0.75, AlbedoType::Concrete);
        let inp = standard_inputs(600.0);
        let res = model.compute_energy(&inp).expect("ok");
        assert!(
            res.bifacial_gain_pct > 0.0,
            "gain={}",
            res.bifacial_gain_pct
        );
    }

    #[test]
    fn test_vf_rear_sky_plus_ground_equals_one() {
        let model = make_model(30.0, 0.75, AlbedoType::Grass);
        let vf = model.compute_view_factors();
        let sum = vf.rear_sky_vf + vf.rear_ground_vf;
        assert!((sum - 1.0).abs() < 1e-9, "VF sum={sum}");
    }

    #[test]
    fn test_gcr_formula() {
        let model = make_model(30.0, 0.75, AlbedoType::Grass);
        let expected = 2.0 * 30_f64.to_radians().cos() / 5.0;
        let gcr = model.ground_coverage_ratio();
        assert!(
            (gcr - expected).abs() < 1e-9,
            "gcr={gcr} expected={expected}"
        );
    }

    #[test]
    fn test_albedo_type_values() {
        assert!((AlbedoType::Grass.albedo_value() - 0.20).abs() < 1e-9);
        assert!((AlbedoType::Snow.albedo_value() - 0.80).abs() < 1e-9);
        assert!((AlbedoType::Concrete.albedo_value() - 0.35).abs() < 1e-9);
        assert!((AlbedoType::Asphalt.albedo_value() - 0.15).abs() < 1e-9);
        assert!((AlbedoType::Sand.albedo_value() - 0.30).abs() < 1e-9);
        assert!((AlbedoType::White.albedo_value() - 0.65).abs() < 1e-9);
    }

    #[test]
    fn test_custom_albedo_type() {
        assert!((AlbedoType::Custom(0.42).albedo_value() - 0.42).abs() < 1e-9);
    }

    #[test]
    fn test_annual_yield_12_monthly_values() {
        let model = make_model(30.0, 0.75, AlbedoType::Concrete);
        let ghi = [150.0_f64; 12];
        let dni = [120.0_f64; 12];
        let dhi = [30.0_f64; 12];
        let t = [15.0_f64; 12];
        let res = model
            .simulate_annual(&ghi, &dni, &dhi, &t, 45.0)
            .expect("ok");
        assert_eq!(res.monthly_energy_kwh.len(), 12);
    }

    #[test]
    fn test_monthly_energy_positive() {
        let model = make_model(30.0, 0.75, AlbedoType::Concrete);
        let ghi = [200.0_f64; 12];
        let dni = [170.0_f64; 12];
        let dhi = [50.0_f64; 12];
        let t = [15.0_f64; 12];
        let res = model
            .simulate_annual(&ghi, &dni, &dhi, &t, 45.0)
            .expect("ok");
        for (m, &e) in res.monthly_energy_kwh.iter().enumerate() {
            assert!(e >= 0.0, "month {m} energy={e}");
        }
    }

    #[test]
    fn test_bifacial_gain_annual_positive() {
        let model = make_model(30.0, 0.75, AlbedoType::Concrete);
        let ghi = [200.0_f64; 12];
        let dni = [170.0_f64; 12];
        let dhi = [50.0_f64; 12];
        let t = [15.0_f64; 12];
        let res = model
            .simulate_annual(&ghi, &dni, &dhi, &t, 45.0)
            .expect("ok");
        assert!(
            res.bifacial_gain_annual_pct >= 0.0,
            "gain={}",
            res.bifacial_gain_annual_pct
        );
    }

    #[test]
    fn test_performance_ratio_between_zero_and_one() {
        let model = make_model(30.0, 0.75, AlbedoType::Concrete);
        let ghi = [200.0_f64; 12];
        let dni = [170.0_f64; 12];
        let dhi = [50.0_f64; 12];
        let t = [15.0_f64; 12];
        let res = model
            .simulate_annual(&ghi, &dni, &dhi, &t, 45.0)
            .expect("ok");
        assert!(
            res.performance_ratio >= 0.0 && res.performance_ratio <= 1.0,
            "PR={}",
            res.performance_ratio
        );
    }

    #[test]
    fn test_temp_correction_below_one_at_high_temp() {
        let model = make_model(30.0, 0.75, AlbedoType::Grass);
        let inp = IrradianceInputs {
            ghi: 800.0,
            dni: 700.0,
            dhi: 100.0,
            solar_zenith_deg: 30.0,
            solar_azimuth_deg: 180.0,
            t_amb_c: 45.0, // high ambient → T_cell > 25°C
            wind_speed_ms: 0.0,
        };
        let res = model.compute_energy(&inp).expect("ok");
        assert!(
            res.temp_correction < 1.0,
            "temp_correction={}",
            res.temp_correction
        );
    }

    #[test]
    fn test_front_poa_components_sum() {
        let model = make_model(30.0, 0.75, AlbedoType::Concrete);
        let inp = standard_inputs(600.0);
        let irr = model.compute_irradiance(&inp).expect("ok");
        let component_sum = irr.front_beam + irr.front_diffuse + irr.front_ground_reflected;
        assert!(
            (component_sum - irr.front_poa_w_m2).abs() < 1e-6,
            "sum={component_sum} front_poa={}",
            irr.front_poa_w_m2
        );
    }

    #[test]
    fn test_simulation_30deg_tilt_custom_albedo() {
        let albedo_val = 0.25;
        let model = make_model(30.0, 0.75, AlbedoType::Custom(albedo_val));
        let ghi = [250.0_f64; 12];
        let dni = [200.0_f64; 12];
        let dhi = [50.0_f64; 12];
        let t = [15.0_f64; 12];
        let res = model
            .simulate_annual(&ghi, &dni, &dhi, &t, 40.0)
            .expect("ok");
        assert!(res.annual_yield_kwh_per_kw > 0.0);
        assert!(res.monthly_energy_kwh.iter().all(|&e| e >= 0.0));
    }
}
