//! Weather-coupled renewable energy forecasting.
//!
//! Integrates Numerical Weather Prediction (NWP) outputs with bias correction,
//! temporal downscaling, and uncertainty quantification for solar PV and wind power.
//!
//! # Units
//!
//! - Wind speed in \[m/s\]
//! - Irradiance in \[W/m²\]
//! - Power in \[MW\]
//! - Temperature in \[°C\]
//! - Forecast horizon in \[h\]
//!
//! # References
//!
//! - Lorenz et al., "Irradiance Forecasting for the Power Prediction of Grid-Connected PV Systems",
//!   IEEE J. Sel. Top. Appl. Earth Obs. Remote Sens. 2009
//! - Giebel & Kariniotakis, "Wind Power Forecasting — A Review of the State of the Art", 2017

use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// NWP input
// ---------------------------------------------------------------------------

/// Numerical Weather Prediction forecast for a single site.
///
/// All `Vec` fields have length `horizon_h`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NwpForecast {
    /// Forecast horizon \[h\].
    pub horizon_h: usize,
    /// Unix timestamps \[s\] for each hourly step.
    pub timestamps: Vec<f64>,
    /// Wind speed at 10 m height \[m/s\].
    pub wind_speed_10m: Vec<f64>,
    /// Wind speed at hub height \[m/s\].
    pub wind_speed_100m: Vec<f64>,
    /// Wind direction \[°\], meteorological convention.
    pub wind_direction_deg: Vec<f64>,
    /// Air temperature \[°C\].
    pub temperature_c: Vec<f64>,
    /// Global Horizontal Irradiance \[W/m²\].
    pub ghi_w_m2: Vec<f64>,
    /// Direct Normal Irradiance \[W/m²\].
    pub dni_w_m2: Vec<f64>,
    /// Cloud cover \[%\].
    pub cloud_cover_pct: Vec<f64>,
    /// Atmospheric pressure \[hPa\].
    pub pressure_hpa: Vec<f64>,
    /// Relative humidity \[%\].
    pub humidity_pct: Vec<f64>,
}

impl NwpForecast {
    /// Construct an NWP forecast with all zero fields.
    pub fn zeros(horizon_h: usize) -> Self {
        Self {
            horizon_h,
            timestamps: vec![0.0; horizon_h],
            wind_speed_10m: vec![0.0; horizon_h],
            wind_speed_100m: vec![0.0; horizon_h],
            wind_direction_deg: vec![0.0; horizon_h],
            temperature_c: vec![20.0; horizon_h],
            ghi_w_m2: vec![0.0; horizon_h],
            dni_w_m2: vec![0.0; horizon_h],
            cloud_cover_pct: vec![0.0; horizon_h],
            pressure_hpa: vec![1013.25; horizon_h],
            humidity_pct: vec![50.0; horizon_h],
        }
    }
}

// ---------------------------------------------------------------------------
// Asset type
// ---------------------------------------------------------------------------

/// Type of renewable energy asset for weather-coupled forecasting.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum WeatherAssetType {
    /// Solar PV plant.
    SolarPv {
        /// Rated DC capacity \[MWp\].
        capacity_mwp: f64,
        /// Panel tilt from horizontal \[°\].
        tilt_deg: f64,
        /// Panel azimuth from north \[°\].
        azimuth_deg: f64,
    },
    /// Single wind turbine.
    WindTurbine {
        /// Rated electrical output \[MW\].
        rated_mw: f64,
        /// Hub height above ground \[m\].
        hub_height_m: f64,
        /// Cut-in wind speed \[m/s\].
        cut_in_ms: f64,
        /// Cut-out wind speed \[m/s\].
        cut_out_ms: f64,
    },
    /// Wind farm aggregate.
    WindFarm {
        /// Total installed capacity \[MW\].
        total_mw: f64,
        /// Number of turbines.
        num_turbines: usize,
        /// Hub height above ground \[m\].
        hub_height_m: f64,
    },
}

// ---------------------------------------------------------------------------
// Bias correction
// ---------------------------------------------------------------------------

/// Bias correction method applied to raw NWP-derived power forecasts.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum BiasCorrectionMethod {
    /// No correction applied.
    None,
    /// Subtract the mean forecast error from the training history.
    MeanBias,
    /// Multiply by the ratio observed_mean / nwp_mean.
    LinearScaling,
    /// Map NWP quantiles to observed quantiles via linear interpolation.
    QuantileMapping,
    /// Monotone regression correction via Pool Adjacent Violators.
    IsotonicRegression,
}

/// Internal parameters learned from training data for bias correction.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BiasCorrectionModel {
    /// Mean forecast error (forecast − actual), used by `MeanBias`.
    pub mean_error: f64,
    /// Ratio observed_mean / nwp_mean, used by `LinearScaling`.
    pub scale_factor: f64,
    /// 100 quantiles of observed power values.
    pub obs_quantiles: Vec<f64>,
    /// 100 quantiles of NWP-derived power values.
    pub nwp_quantiles: Vec<f64>,
}

impl Default for BiasCorrectionModel {
    fn default() -> Self {
        Self {
            mean_error: 0.0,
            scale_factor: 1.0,
            obs_quantiles: Vec::new(),
            nwp_quantiles: Vec::new(),
        }
    }
}

impl BiasCorrectionModel {
    /// Refit the model from `(predicted, actual)` pairs.
    fn refit(&mut self, history: &[(f64, f64)]) {
        if history.is_empty() {
            return;
        }
        let n = history.len() as f64;
        let sum_err: f64 = history.iter().map(|(p, a)| p - a).sum();
        self.mean_error = sum_err / n;

        let sum_pred: f64 = history.iter().map(|(p, _)| p).sum();
        let sum_act: f64 = history.iter().map(|(_, a)| a).sum();
        let mean_pred = sum_pred / n;
        let mean_act = sum_act / n;
        self.scale_factor = if mean_pred.abs() > 1e-9 {
            mean_act / mean_pred
        } else {
            1.0
        };

        // Compute 100 empirical quantiles from sorted predicted and actual values.
        let mut preds: Vec<f64> = history.iter().map(|(p, _)| *p).collect();
        let mut acts: Vec<f64> = history.iter().map(|(_, a)| *a).collect();
        preds.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        acts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        self.nwp_quantiles = compute_quantiles_100(&preds);
        self.obs_quantiles = compute_quantiles_100(&acts);
    }
}

/// Compute 100 evenly spaced quantiles from a sorted slice.
fn compute_quantiles_100(sorted: &[f64]) -> Vec<f64> {
    let n = sorted.len();
    if n == 0 {
        return vec![0.0; 100];
    }
    (0..100)
        .map(|i| {
            let frac = i as f64 / 99.0;
            let idx = frac * (n - 1) as f64;
            let lo = idx.floor() as usize;
            let hi = (lo + 1).min(n - 1);
            let t = idx - lo as f64;
            sorted[lo] * (1.0 - t) + sorted[hi] * t
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------

/// Configuration for the weather-coupled forecaster.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WeatherForecastConfig {
    /// Bias correction method to apply after power conversion.
    pub bias_correction_method: BiasCorrectionMethod,
    /// Temporal downscaling factor (e.g. 4 = 1 h → 15 min).
    pub downscaling_factor: usize,
    /// Number of ensemble members for uncertainty quantification.
    pub ensemble_members: usize,
    /// Prediction interval coverage (e.g. 0.9 = 90 %).
    pub pi_coverage: f64,
    /// Weight for blending NWP forecast with persistence \[0, 1\].
    /// 0.0 = pure NWP, 1.0 = pure persistence.
    pub persistence_blend_weight: f64,
}

impl Default for WeatherForecastConfig {
    fn default() -> Self {
        Self {
            bias_correction_method: BiasCorrectionMethod::None,
            downscaling_factor: 1,
            ensemble_members: 20,
            pi_coverage: 0.9,
            persistence_blend_weight: 0.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Result
// ---------------------------------------------------------------------------

/// Output of a weather-coupled power forecast.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WeatherForecastResult {
    /// Timestamps corresponding to each forecast step.
    pub timestamps: Vec<f64>,
    /// Point forecast \[MW\].
    pub power_mw: Vec<f64>,
    /// Lower prediction interval bound \[MW\].
    pub power_lower_mw: Vec<f64>,
    /// Upper prediction interval bound \[MW\].
    pub power_upper_mw: Vec<f64>,
    /// All ensemble member power series \[MW\].
    pub ensemble_mw: Vec<Vec<f64>>,
    /// Forecast skill score (None if no reference available).
    pub skill_score: Option<f64>,
    /// Whether bias correction was applied.
    pub bias_corrected: bool,
    /// Continuous Ranked Probability Score per time step (None if not computed).
    pub crps: Option<Vec<f64>>,
}

// ---------------------------------------------------------------------------
// Forecaster
// ---------------------------------------------------------------------------

/// Weather-coupled renewable energy forecaster.
///
/// Converts NWP fields (wind speed, irradiance, temperature) to power forecasts
/// using physical models, with optional bias correction, ensemble generation, and
/// temporal downscaling.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WeatherCoupledForecaster {
    /// Site identifier.
    pub site_name: String,
    /// Site latitude \[°\].
    pub latitude_deg: f64,
    /// Site longitude \[°\].
    pub longitude_deg: f64,
    /// Site altitude above sea level \[m\].
    pub altitude_m: f64,
    /// Type of renewable asset at this site.
    pub asset_type: WeatherAssetType,
    /// Forecasting configuration.
    pub config: WeatherForecastConfig,
    /// Fitted bias correction model.
    bias_correction: BiasCorrectionModel,
    /// Historical (predicted, actual) pairs used to fit bias correction.
    training_history: Vec<(f64, f64)>,
}

impl WeatherCoupledForecaster {
    /// Create a new forecaster for `site_name`.
    pub fn new(
        site_name: impl Into<String>,
        latitude_deg: f64,
        longitude_deg: f64,
        altitude_m: f64,
        asset_type: WeatherAssetType,
        config: WeatherForecastConfig,
    ) -> Self {
        Self {
            site_name: site_name.into(),
            latitude_deg,
            longitude_deg,
            altitude_m,
            asset_type,
            config,
            bias_correction: BiasCorrectionModel::default(),
            training_history: Vec::new(),
        }
    }

    // -----------------------------------------------------------------------
    // Power conversion
    // -----------------------------------------------------------------------

    /// Convert NWP fields to an hourly power forecast \[MW\].
    pub fn forecast_power(&self, nwp: &NwpForecast) -> WeatherForecastResult {
        let n = nwp.horizon_h;
        let power_mw: Vec<f64> = (0..n).map(|i| self.convert_step(nwp, i)).collect();

        let power_mw = if self.config.bias_correction_method != BiasCorrectionMethod::None {
            self.apply_bias_correction(&power_mw, &self.config.bias_correction_method)
        } else {
            power_mw
        };

        WeatherForecastResult {
            timestamps: nwp.timestamps.clone(),
            power_lower_mw: power_mw.clone(),
            power_upper_mw: power_mw.clone(),
            ensemble_mw: Vec::new(),
            skill_score: None,
            bias_corrected: self.config.bias_correction_method != BiasCorrectionMethod::None,
            crps: None,
            power_mw,
        }
    }

    /// Convert a single time step index to power \[MW\].
    fn convert_step(&self, nwp: &NwpForecast, i: usize) -> f64 {
        match &self.asset_type {
            WeatherAssetType::SolarPv {
                capacity_mwp,
                tilt_deg,
                azimuth_deg,
            } => self.solar_power_step(
                nwp.ghi_w_m2[i],
                nwp.temperature_c[i],
                *capacity_mwp,
                *tilt_deg,
                *azimuth_deg,
            ),
            WeatherAssetType::WindTurbine {
                rated_mw,
                hub_height_m,
                cut_in_ms,
                cut_out_ms,
            } => {
                let ws_hub = wind_speed_hub(nwp.wind_speed_10m[i], *hub_height_m);
                wind_power_step(ws_hub, *rated_mw, *cut_in_ms, *cut_out_ms)
            }
            WeatherAssetType::WindFarm {
                total_mw,
                num_turbines: _,
                hub_height_m,
            } => {
                let ws_hub = wind_speed_hub(nwp.wind_speed_10m[i], *hub_height_m);
                // Default cut-in 3 m/s, cut-out 25 m/s for generic farm
                wind_power_step(ws_hub, *total_mw, 3.0, 25.0)
            }
        }
    }

    /// Solar PV power for a single time step using Sandia/King approximation \[MW\].
    ///
    /// - NOCT (Nominal Operating Cell Temperature) assumed 45 °C
    /// - Temperature coefficient γ = 0.0045 /°C
    fn solar_power_step(
        &self,
        ghi: f64,
        t_air: f64,
        capacity_mwp: f64,
        tilt_deg: f64,
        azimuth_deg: f64,
    ) -> f64 {
        // Simplified POA irradiance from GHI using tilt geometry.
        // POA ≈ GHI × cos(tilt) + GHI × (1 - cos(tilt))/2 (diffuse on tilted surface)
        // + ground-reflected component GHI × albedo × (1 - cos(tilt))/2
        let tilt_rad = tilt_deg.to_radians();
        // Azimuth correction: simple cosine projection (south = 0°)
        let azimuth_rad = azimuth_deg.to_radians();
        // For a simplified flat-panel model, approximate incidence factor
        let incidence_factor =
            (tilt_rad.cos() + azimuth_rad.cos().abs() * tilt_rad.sin() * 0.5).clamp(0.0, 1.0);
        let diffuse_factor = (1.0 - tilt_rad.cos()) / 2.0;
        let albedo = 0.2_f64;
        let poa = ghi * incidence_factor + ghi * diffuse_factor + ghi * albedo * diffuse_factor;
        let poa = poa.max(0.0);

        // Cell temperature model
        let noct = 45.0_f64;
        let t_cell = t_air + ghi * (noct - 20.0) / 800.0;

        // Power output
        let gamma = 0.0045_f64;
        let p = capacity_mwp * (poa / 1000.0) * (1.0 - gamma * (t_cell - 25.0));
        p.clamp(0.0, capacity_mwp)
    }

    // -----------------------------------------------------------------------
    // Bias correction
    // -----------------------------------------------------------------------

    /// Apply bias correction to a raw hourly power forecast \[MW\].
    pub fn apply_bias_correction(
        &self,
        raw_forecast: &[f64],
        method: &BiasCorrectionMethod,
    ) -> Vec<f64> {
        match method {
            BiasCorrectionMethod::None => raw_forecast.to_vec(),
            BiasCorrectionMethod::MeanBias => raw_forecast
                .iter()
                .map(|v| (v - self.bias_correction.mean_error).max(0.0))
                .collect(),
            BiasCorrectionMethod::LinearScaling => raw_forecast
                .iter()
                .map(|v| (v * self.bias_correction.scale_factor).max(0.0))
                .collect(),
            BiasCorrectionMethod::QuantileMapping => self.apply_quantile_mapping(raw_forecast),
            BiasCorrectionMethod::IsotonicRegression => isotonic_regression(raw_forecast),
        }
    }

    /// Quantile mapping: maps each raw value via empirical CDF of NWP → observed.
    fn apply_quantile_mapping(&self, raw: &[f64]) -> Vec<f64> {
        let nwp_q = &self.bias_correction.nwp_quantiles;
        let obs_q = &self.bias_correction.obs_quantiles;
        if nwp_q.is_empty() || obs_q.is_empty() {
            return raw.to_vec();
        }
        raw.iter()
            .map(|&v| interp_quantile(v, nwp_q, obs_q))
            .collect()
    }

    // -----------------------------------------------------------------------
    // Ensemble generation
    // -----------------------------------------------------------------------

    /// Generate an ensemble of power forecasts by perturbing NWP inputs.
    ///
    /// Wind speed uncertainty grows linearly to ±20 % at 48 \[h\].
    /// GHI uncertainty grows linearly to ±15 % at 24 \[h\].
    /// Uses a 64-bit LCG (Knuth MMIX) for reproducible perturbations.
    pub fn generate_ensemble(&self, nwp: &NwpForecast, n_members: usize) -> Vec<Vec<f64>> {
        let n = nwp.horizon_h;
        let mut members = Vec::with_capacity(n_members);
        // Seed per member to get distinct sequences
        let base_seed: u64 = 0xDEAD_BEEF_1234_5678;

        for m in 0..n_members {
            let mut state: u64 =
                base_seed.wrapping_add((m as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15));
            let mut perturbed = nwp.clone();

            for i in 0..n {
                // Horizon fraction (0..1 mapped to 48h max)
                let h_frac = (i as f64 / 48.0_f64).min(1.0);

                // Wind speed perturbation ±20% at 48h
                let wind_sigma = 0.20 * h_frac;
                let wind_noise = lcg_normal(&mut state) * wind_sigma;
                perturbed.wind_speed_10m[i] = (nwp.wind_speed_10m[i] * (1.0 + wind_noise)).max(0.0);

                // GHI perturbation ±15% at 24h
                let ghi_frac = (i as f64 / 24.0_f64).min(1.0);
                let ghi_sigma = 0.15 * ghi_frac;
                let ghi_noise = lcg_normal(&mut state) * ghi_sigma;
                perturbed.ghi_w_m2[i] = (nwp.ghi_w_m2[i] * (1.0 + ghi_noise)).max(0.0);
                perturbed.dni_w_m2[i] = (nwp.dni_w_m2[i] * (1.0 + ghi_noise)).max(0.0);
            }

            let result = self.forecast_power(&perturbed);
            members.push(result.power_mw);
        }
        members
    }

    // -----------------------------------------------------------------------
    // Temporal downscaling
    // -----------------------------------------------------------------------

    /// Downscale an hourly power forecast to sub-hourly resolution using cubic spline.
    ///
    /// `factor` = number of sub-intervals per hour (e.g. 4 for 15-min).
    /// Hourly energy is preserved by renormalising after interpolation.
    /// A small mean-reverting stochastic variation (σ = 2 %) is added.
    pub fn temporal_downscaling(&self, hourly_forecast: &[f64], factor: usize) -> Vec<f64> {
        if factor <= 1 || hourly_forecast.is_empty() {
            return hourly_forecast.to_vec();
        }
        let n_hours = hourly_forecast.len();
        let n_out = n_hours * factor;
        let mut out = vec![0.0_f64; n_out];

        // Cubic spline via Catmull-Rom interpolation between hourly points.
        for h in 0..n_hours {
            let p0 = if h > 0 {
                hourly_forecast[h - 1]
            } else {
                hourly_forecast[h]
            };
            let p1 = hourly_forecast[h];
            let p2 = if h + 1 < n_hours {
                hourly_forecast[h + 1]
            } else {
                hourly_forecast[h]
            };
            let p3 = if h + 2 < n_hours {
                hourly_forecast[h + 2]
            } else {
                p2
            };

            for s in 0..factor {
                let t = s as f64 / factor as f64;
                out[h * factor + s] = catmull_rom(p0, p1, p2, p3, t).max(0.0);
            }
        }

        // Add small stochastic variation (σ=2%) while preserving hourly energy.
        let mut state: u64 = 0xCAFE_BABE_DEAD_C0DE;
        for h in 0..n_hours {
            let slice = &mut out[h * factor..(h + 1) * factor];
            // Target sum = hourly_forecast[h] * factor (integral = hourly value)
            let target = hourly_forecast[h] * factor as f64;

            // Add stochastic variation
            let sigma = 0.02;
            for v in slice.iter_mut() {
                let noise = lcg_normal(&mut state) * sigma;
                *v = (*v * (1.0 + noise)).max(0.0);
            }

            // Renormalise to preserve energy
            let current_sum: f64 = slice.iter().sum();
            if current_sum > 1e-12 {
                let scale = target / current_sum;
                for v in slice.iter_mut() {
                    *v *= scale;
                }
            }
        }

        out
    }

    // -----------------------------------------------------------------------
    // Uncertainty quantification
    // -----------------------------------------------------------------------

    /// Generate a full probabilistic forecast with prediction intervals and CRPS.
    pub fn forecast_with_uncertainty(&self, nwp: &NwpForecast) -> WeatherForecastResult {
        let n_members = self.config.ensemble_members;
        let ensemble_mw = self.generate_ensemble(nwp, n_members);
        let n = nwp.horizon_h;

        // Point forecast = ensemble mean
        let power_mw: Vec<f64> = (0..n)
            .map(|i| {
                let sum: f64 = ensemble_mw.iter().map(|m| m[i]).sum();
                sum / n_members as f64
            })
            .collect();

        // Prediction intervals from ensemble quantiles
        let alpha = (1.0 - self.config.pi_coverage) / 2.0;
        let lo_idx = (alpha * n_members as f64).floor() as usize;
        let hi_idx = ((1.0 - alpha) * n_members as f64)
            .ceil()
            .min(n_members as f64 - 1.0) as usize;

        let mut power_lower_mw = vec![0.0; n];
        let mut power_upper_mw = vec![0.0; n];
        for i in 0..n {
            let mut col: Vec<f64> = ensemble_mw.iter().map(|m| m[i]).collect();
            col.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            power_lower_mw[i] = col[lo_idx.min(col.len() - 1)];
            power_upper_mw[i] = col[hi_idx.min(col.len() - 1)];
        }

        // CRPS per time step (using ensemble members as empirical distribution)
        let crps: Vec<f64> = (0..n)
            .map(|i| {
                let mut col: Vec<f64> = ensemble_mw.iter().map(|m| m[i]).collect();
                col.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
                // Pinball / CRPS approximation from sorted ensemble
                compute_crps_ensemble(&col, power_mw[i])
            })
            .collect();

        WeatherForecastResult {
            timestamps: nwp.timestamps.clone(),
            power_mw,
            power_lower_mw,
            power_upper_mw,
            ensemble_mw,
            skill_score: None,
            bias_corrected: false,
            crps: Some(crps),
        }
    }

    // -----------------------------------------------------------------------
    // Training & skill
    // -----------------------------------------------------------------------

    /// Record a (predicted, actual) pair for bias correction fitting.
    pub fn update_training(&mut self, predicted: f64, actual: f64) {
        self.training_history.push((predicted, actual));
        self.bias_correction.refit(&self.training_history);
    }

    /// Forecast skill score relative to climatology.
    ///
    /// `SS = 1 - MSE_forecast / MSE_climatology`
    ///
    /// Returns 1.0 for a perfect forecast, 0.0 for climatology, negative if worse.
    pub fn skill_score(forecasts: &[f64], actuals: &[f64], climatology_mean: f64) -> f64 {
        let n = forecasts.len().min(actuals.len());
        if n == 0 {
            return 0.0;
        }
        let mse_fc: f64 = (0..n)
            .map(|i| (forecasts[i] - actuals[i]).powi(2))
            .sum::<f64>()
            / n as f64;
        let mse_cl: f64 = actuals
            .iter()
            .map(|a| (a - climatology_mean).powi(2))
            .sum::<f64>()
            / n as f64;
        if mse_cl.abs() < 1e-12 {
            return if mse_fc < 1e-12 {
                1.0
            } else {
                f64::NEG_INFINITY
            };
        }
        1.0 - mse_fc / mse_cl
    }
}

// ---------------------------------------------------------------------------
// Physical helpers
// ---------------------------------------------------------------------------

/// Wind speed at hub height via 1/7 power law: `ws_hub = ws_10m × (hub_height/10)^(1/7)`.
pub fn wind_speed_hub(ws_10m: f64, hub_height_m: f64) -> f64 {
    ws_10m * (hub_height_m / 10.0_f64).powf(1.0 / 7.0)
}

/// Cubic wind turbine power curve \[MW\].
///
/// - Below cut-in: 0
/// - Cut-in to rated: cubic ramp
/// - Rated to cut-out: rated power
/// - Above cut-out: 0
///
/// `ws_rated` is set to 12 m/s (generic IEC class II approximation).
pub fn wind_power_step(ws: f64, rated_mw: f64, cut_in_ms: f64, cut_out_ms: f64) -> f64 {
    let ws_rated = 12.0_f64; // generic rated wind speed
    if ws < cut_in_ms || ws > cut_out_ms {
        return 0.0;
    }
    if ws >= ws_rated {
        return rated_mw;
    }
    let ramp = (ws - cut_in_ms) / (ws_rated - cut_in_ms);
    (rated_mw * ramp.powi(3)).clamp(0.0, rated_mw)
}

// ---------------------------------------------------------------------------
// Numerical helpers
// ---------------------------------------------------------------------------

/// Catmull-Rom cubic spline interpolation between `p1` and `p2` at parameter `t ∈ [0, 1]`.
fn catmull_rom(p0: f64, p1: f64, p2: f64, p3: f64, t: f64) -> f64 {
    let t2 = t * t;
    let t3 = t2 * t;
    0.5 * ((2.0 * p1)
        + (-p0 + p2) * t
        + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2
        + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3)
}

/// Advance the LCG state (Knuth MMIX) and return a uniform sample in \[-1, 1\].
fn lcg_step(state: &mut u64) -> f64 {
    *state = state
        .wrapping_mul(6_364_136_223_846_793_005_u64)
        .wrapping_add(1_442_695_040_888_963_407_u64);
    // Map high 32 bits to [-1, 1]
    let high = (*state >> 32) as u32;
    (high as f64 / u32::MAX as f64) * 2.0 - 1.0
}

/// Box-Muller transform approximation using two LCG samples → approximately N(0,1).
fn lcg_normal(state: &mut u64) -> f64 {
    // Two uniform samples
    let u1 = (lcg_step(state) + 1.0) / 2.0; // [0, 1]
    let u2 = (lcg_step(state) + 1.0) / 2.0;
    let u1 = u1.max(1e-12);
    (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
}

/// Pool Adjacent Violators (PAV) isotonic regression (non-decreasing).
fn isotonic_regression(values: &[f64]) -> Vec<f64> {
    let n = values.len();
    if n == 0 {
        return Vec::new();
    }
    // Each block = (sum, count)
    let mut blocks: Vec<(f64, usize)> = values.iter().map(|&v| (v, 1)).collect();
    let mut i = 1;
    while i < blocks.len() {
        if blocks[i].0 / (blocks[i].1 as f64) < blocks[i - 1].0 / (blocks[i - 1].1 as f64) {
            // Merge blocks[i] into blocks[i-1]
            blocks[i - 1].0 += blocks[i].0;
            blocks[i - 1].1 += blocks[i].1;
            blocks.remove(i);
            if i > 1 {
                i -= 1;
            }
        } else {
            i += 1;
        }
    }
    // Expand back to original length
    let mut result = Vec::with_capacity(n);
    for (sum, count) in blocks {
        let mean = sum / count as f64;
        for _ in 0..count {
            result.push(mean);
        }
    }
    result
}

/// Linear interpolation through an empirical quantile mapping.
fn interp_quantile(value: f64, from_q: &[f64], to_q: &[f64]) -> f64 {
    let n = from_q.len().min(to_q.len());
    if n == 0 {
        return value;
    }
    // Find position in from_q
    if value <= from_q[0] {
        return to_q[0];
    }
    if value >= from_q[n - 1] {
        return to_q[n - 1];
    }
    // Binary search
    let mut lo = 0usize;
    let mut hi = n - 1;
    while hi - lo > 1 {
        let mid = (lo + hi) / 2;
        if from_q[mid] <= value {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let t = if (from_q[hi] - from_q[lo]).abs() < 1e-12 {
        0.5
    } else {
        (value - from_q[lo]) / (from_q[hi] - from_q[lo])
    };
    to_q[lo] + t * (to_q[hi] - to_q[lo])
}

/// Energy-score / CRPS approximation from a sorted ensemble for observation `y`.
///
/// Uses the formula:
/// `CRPS = E|X - y| - 0.5 E|X - X'|`
/// where expectation is over ensemble members.
fn compute_crps_ensemble(sorted_ensemble: &[f64], y: f64) -> f64 {
    let m = sorted_ensemble.len() as f64;
    if m < 1.0 {
        return 0.0;
    }
    // E|X - y|
    let e_xy: f64 = sorted_ensemble.iter().map(|&x| (x - y).abs()).sum::<f64>() / m;
    // E|X - X'| via sorted ensemble trick: sum_i (2i - m - 1) * x_i / m^2
    let e_xx: f64 = sorted_ensemble
        .iter()
        .enumerate()
        .map(|(i, &x)| (2.0 * i as f64 - m + 1.0) * x)
        .sum::<f64>()
        / (m * m);
    (e_xy - 0.5 * e_xx).max(0.0)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_solar_forecaster(capacity_mwp: f64, tilt_deg: f64) -> WeatherCoupledForecaster {
        WeatherCoupledForecaster::new(
            "test_solar",
            51.5,
            -0.1,
            10.0,
            WeatherAssetType::SolarPv {
                capacity_mwp,
                tilt_deg,
                azimuth_deg: 0.0,
            },
            WeatherForecastConfig::default(),
        )
    }

    fn make_wind_forecaster(rated_mw: f64, hub_height_m: f64) -> WeatherCoupledForecaster {
        WeatherCoupledForecaster::new(
            "test_wind",
            55.0,
            8.0,
            0.0,
            WeatherAssetType::WindTurbine {
                rated_mw,
                hub_height_m,
                cut_in_ms: 3.0,
                cut_out_ms: 25.0,
            },
            WeatherForecastConfig::default(),
        )
    }

    /// Test 1: Solar power from GHI=800 W/m² gives a reasonable output.
    #[test]
    fn test_solar_power_reasonable() {
        let fc = make_solar_forecaster(1.0, 30.0);
        let mut nwp = NwpForecast::zeros(1);
        nwp.ghi_w_m2[0] = 800.0;
        nwp.temperature_c[0] = 25.0;

        let result = fc.forecast_power(&nwp);
        let p = result.power_mw[0];
        // Expect between 20% and 100% of capacity at 800 W/m²
        assert!(p > 0.2, "solar power too low: {p}");
        assert!(p <= 1.0, "solar power exceeds rated: {p}");
    }

    /// Test 2a: Wind cubic curve at rated speed gives ~P_rated.
    #[test]
    fn test_wind_cubic_at_rated() {
        let p = wind_power_step(12.0, 5.0, 3.0, 25.0);
        assert!(
            (p - 5.0).abs() < 1e-6,
            "expected P_rated=5.0 at rated speed, got {p}"
        );
    }

    /// Test 2b: Wind power above cut-out is zero.
    #[test]
    fn test_wind_cut_out() {
        let p = wind_power_step(26.0, 5.0, 3.0, 25.0);
        assert_eq!(p, 0.0, "expected 0 above cut-out, got {p}");
    }

    /// Test 2c: Wind power below cut-in is zero.
    #[test]
    fn test_wind_cut_in() {
        let p = wind_power_step(2.0, 5.0, 3.0, 25.0);
        assert_eq!(p, 0.0, "expected 0 below cut-in, got {p}");
    }

    /// Test 3: Hub height correction gives higher wind speed at 100m vs 10m.
    #[test]
    fn test_hub_height_correction() {
        let ws_10m = 8.0;
        let ws_100m = wind_speed_hub(ws_10m, 100.0);
        assert!(
            ws_100m > ws_10m,
            "hub height correction should give ws_100m > ws_10m: {ws_100m} vs {ws_10m}"
        );
        // Power law: (100/10)^(1/7) ≈ 1.389
        let ratio = ws_100m / ws_10m;
        assert!(
            (ratio - 10.0_f64.powf(1.0 / 7.0)).abs() < 1e-9,
            "power law ratio incorrect: {ratio}"
        );
    }

    /// Test 4: Mean bias correction reduces mean error on a test set.
    #[test]
    fn test_mean_bias_correction() {
        let mut fc = make_solar_forecaster(1.0, 30.0);
        // Training: forecasts are 0.1 higher than actuals on average
        for i in 0..20 {
            let pred = 0.5 + i as f64 * 0.01;
            let actual = pred - 0.1;
            fc.update_training(pred, actual);
        }
        let raw = vec![0.6_f64; 5];
        let corrected = fc.apply_bias_correction(&raw, &BiasCorrectionMethod::MeanBias);
        // Mean bias ≈ 0.1, corrected should be ~0.5
        let mean_raw = raw.iter().sum::<f64>() / raw.len() as f64;
        let mean_corr = corrected.iter().sum::<f64>() / corrected.len() as f64;
        // Error should be reduced (corrected closer to actual ≈ 0.5)
        assert!(
            (mean_corr - 0.5).abs() < (mean_raw - 0.5).abs(),
            "bias correction should reduce mean error: raw_err={}, corr_err={}",
            (mean_raw - 0.5).abs(),
            (mean_corr - 0.5).abs()
        );
    }

    /// Test 5: Ensemble spread grows with forecast horizon.
    #[test]
    fn test_ensemble_spread_grows_with_horizon() {
        let fc = make_wind_forecaster(2.0, 100.0);
        let mut nwp = NwpForecast::zeros(48);
        for i in 0..48 {
            nwp.wind_speed_10m[i] = 8.0;
        }
        nwp.horizon_h = 48;

        let ensemble = fc.generate_ensemble(&nwp, 20);

        // Compute spread (std dev) at hour 1 vs hour 47
        let spread_early: f64 = {
            let vals: Vec<f64> = ensemble.iter().map(|m| m[1]).collect();
            std_dev(&vals)
        };
        let spread_late: f64 = {
            let vals: Vec<f64> = ensemble.iter().map(|m| m[47]).collect();
            std_dev(&vals)
        };
        assert!(
            spread_late >= spread_early,
            "ensemble spread should grow with horizon: early={spread_early:.4}, late={spread_late:.4}"
        );
    }

    fn std_dev(vals: &[f64]) -> f64 {
        let n = vals.len() as f64;
        if n < 2.0 {
            return 0.0;
        }
        let mean = vals.iter().sum::<f64>() / n;
        let var = vals.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / n;
        var.sqrt()
    }

    /// Test 6: Temporal downscaling produces 4 values per hour and conserves energy.
    #[test]
    fn test_temporal_downscaling_energy_conservation() {
        let fc = make_solar_forecaster(1.0, 30.0);
        let hourly = vec![1.0_f64, 2.0, 3.0, 4.0];
        let factor = 4;
        let sub = fc.temporal_downscaling(&hourly, factor);

        // Check length
        assert_eq!(
            sub.len(),
            hourly.len() * factor,
            "expected {} values, got {}",
            hourly.len() * factor,
            sub.len()
        );

        // Check energy conservation per hour (sum of sub-hourly = hourly * factor)
        for h in 0..hourly.len() {
            let sub_sum: f64 = sub[h * factor..(h + 1) * factor].iter().sum();
            let expected = hourly[h] * factor as f64;
            assert!(
                (sub_sum - expected).abs() < 1e-6,
                "hour {h}: energy not conserved: sub_sum={sub_sum:.6} vs expected={expected:.6}"
            );
        }
    }

    /// Test 7a: Perfect forecast gives skill score = 1.0.
    #[test]
    fn test_skill_score_perfect() {
        let actuals = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let forecasts = actuals.clone();
        let clim_mean = 3.0;
        let ss = WeatherCoupledForecaster::skill_score(&forecasts, &actuals, clim_mean);
        assert!(
            (ss - 1.0).abs() < 1e-9,
            "perfect forecast SS should be 1.0, got {ss}"
        );
    }

    /// Test 7b: Climatology forecast gives skill score = 0.0.
    #[test]
    fn test_skill_score_climatology() {
        let actuals = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let clim_mean = actuals.iter().sum::<f64>() / actuals.len() as f64;
        let forecasts = vec![clim_mean; actuals.len()];
        let ss = WeatherCoupledForecaster::skill_score(&forecasts, &actuals, clim_mean);
        assert!(
            ss.abs() < 1e-9,
            "climatology forecast SS should be 0.0, got {ss}"
        );
    }

    /// Test 8: Quantile mapping output is monotone in the mapped values.
    #[test]
    fn test_quantile_mapping_monotonicity() {
        let mut fc = make_solar_forecaster(1.0, 30.0);
        // Training with a clear upward shift: obs = 0.8 * pred
        for i in 0..100 {
            let pred = i as f64 * 0.01;
            let actual = pred * 0.8;
            fc.update_training(pred, actual);
        }
        // Map an ascending sequence through quantile mapping
        let raw: Vec<f64> = (0..20).map(|i| i as f64 * 0.05).collect();
        let mapped = fc.apply_bias_correction(&raw, &BiasCorrectionMethod::QuantileMapping);

        // Check monotonicity: mapped values should be non-decreasing
        for i in 1..mapped.len() {
            assert!(
                mapped[i] >= mapped[i - 1] - 1e-9,
                "quantile mapping not monotone at index {i}: {} < {}",
                mapped[i],
                mapped[i - 1]
            );
        }
    }
}
