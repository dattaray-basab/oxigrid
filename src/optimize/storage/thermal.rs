//! Grid-scale thermal energy storage (TES) optimisation.
//!
//! Models molten-salt CSP stores, pumped-heat electrical stores (PTES),
//! aquifer thermal energy storage (ATES), ice storage and steam accumulators.
//!
//! Dispatch is optimised over a price forecast horizon via a greedy
//! price-threshold dynamic programming approach.
//!
//! # Physical background
//! - Carnot COP (heat-pump mode):  `COP = T_hot / (T_hot - T_ambient)` \[K\]
//! - Self-discharge is modelled as proportional to the temperature difference
//!   between the hot reservoir and ambient, scaled to the configured daily loss rate.
//! - Round-trip efficiency: `η_rt = η_charge × η_discharge`
//!
//! # References
//! - Steinmann, W.D., "Thermal Energy Storage for CSP Plants", Springer, 2019
//! - White, A.J. et al., "Thermodynamic Analysis of PTES", Joule, 2018

use thiserror::Error;

// ── Error type ────────────────────────────────────────────────────────────────

/// Errors produced by the thermal storage optimiser.
#[derive(Debug, Error)]
pub enum ThermalError {
    /// Optimiser cannot run because price forecast is missing or wrong length.
    #[error("Price forecast length {got} does not match n_hours {expected}")]
    PriceLengthMismatch { got: usize, expected: usize },
    /// Configuration parameter is physically invalid.
    #[error("Invalid configuration: {0}")]
    InvalidConfig(String),
    /// No feasible dispatch found.
    #[error("No feasible dispatch: {0}")]
    Infeasible(String),
}

// ── Enumerations ──────────────────────────────────────────────────────────────

/// Technology type for the thermal storage system.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThermalStorageType {
    /// Molten-salt two-tank CSP storage (hot ~565 °C / cold ~290 °C).
    MoltenSalt,
    /// Pumped-heat electrical storage (Joule heating + heat pump, Brayton power cycle).
    PumpedheatElectrical,
    /// Aquifer thermal energy storage (10–100 °C range, seasonal).
    AquiferThermal,
    /// Ice / chilled-water storage for cooling applications.
    IceStorage,
    /// Steam accumulator (industrial short-duration buffer, ~200 °C).
    SteamAccumulator,
}

// ── Configuration ─────────────────────────────────────────────────────────────

/// Configuration parameters for a grid-scale thermal storage system.
#[derive(Debug, Clone)]
pub struct ThermalStorageConfig {
    /// Technology variant.
    pub storage_type: ThermalStorageType,
    /// Usable thermal energy capacity \[GWh_th\].
    pub capacity_gwh_thermal: f64,
    /// Maximum thermal charging rate (heater or heat-pump) \[MW_th\].
    pub rated_charge_mw_thermal: f64,
    /// Maximum thermal discharge rate (turbine or HEX) \[MW_th\].
    pub rated_discharge_mw_thermal: f64,
    /// Electrical-to-thermal charging efficiency (0–1).
    pub charge_efficiency: f64,
    /// Thermal-to-electrical discharge efficiency (0–1).
    pub discharge_efficiency: f64,
    /// Standing thermal losses as a fraction of stored energy per day (0–1).
    pub self_discharge_pct_per_day: f64,
    /// Minimum state-of-charge \[%\] (0–100).
    pub min_soc_pct: f64,
    /// Maximum state-of-charge \[%\] (0–100).
    pub max_soc_pct: f64,
    /// Hot reservoir temperature \[°C\].
    pub operating_temp_hot_c: f64,
    /// Cold reservoir / ambient sink temperature \[°C\].
    pub operating_temp_cold_c: f64,
}

impl ThermalStorageConfig {
    /// Validate configuration parameters.
    pub fn validate(&self) -> Result<(), ThermalError> {
        if self.capacity_gwh_thermal <= 0.0 {
            return Err(ThermalError::InvalidConfig(
                "capacity_gwh_thermal must be positive".to_string(),
            ));
        }
        if !(0.0..=1.0).contains(&self.charge_efficiency) {
            return Err(ThermalError::InvalidConfig(
                "charge_efficiency must be in [0,1]".to_string(),
            ));
        }
        if !(0.0..=1.0).contains(&self.discharge_efficiency) {
            return Err(ThermalError::InvalidConfig(
                "discharge_efficiency must be in [0,1]".to_string(),
            ));
        }
        if self.min_soc_pct >= self.max_soc_pct {
            return Err(ThermalError::InvalidConfig(
                "min_soc_pct must be less than max_soc_pct".to_string(),
            ));
        }
        if self.operating_temp_hot_c <= self.operating_temp_cold_c {
            return Err(ThermalError::InvalidConfig(
                "operating_temp_hot_c must exceed operating_temp_cold_c".to_string(),
            ));
        }
        Ok(())
    }
}

// ── Dispatch record ───────────────────────────────────────────────────────────

/// Hourly dispatch state of the thermal storage system.
#[derive(Debug, Clone)]
pub struct ThermalDispatch {
    /// Hour index (0-based).
    pub hour: usize,
    /// Electrical power consumed during charging \[MW_e\].
    pub charge_mw_electric: f64,
    /// Electrical power produced during discharge \[MW_e\].
    pub discharge_mw_electric: f64,
    /// Stored thermal energy at the end of this hour \[GWh_th\].
    pub soc_gwh: f64,
    /// Useful heat output (CHP / industrial heat application) \[MW_th\].
    pub heat_output_mw: f64,
    /// Ambient temperature at this hour \[°C\].
    pub ambient_temp_c: f64,
}

// ── Result structure ──────────────────────────────────────────────────────────

/// Full optimisation result for a thermal storage dispatch schedule.
#[derive(Debug, Clone)]
pub struct ThermalStorageResult {
    /// Hourly dispatch profile.
    pub dispatch: Vec<ThermalDispatch>,
    /// Total thermal energy charged over the horizon \[GWh_th\].
    pub total_charged_gwh: f64,
    /// Total electrical energy produced during discharge \[GWh_e\].
    pub total_discharged_gwh: f64,
    /// Achieved round-trip efficiency \[%\].
    pub roundtrip_efficiency_pct: f64,
    /// Storage capacity factor \[%\] = average throughput / rated capacity.
    pub capacity_factor_pct: f64,
    /// Thermal energy lost to self-discharge over the horizon \[GWh_th\].
    pub self_discharge_loss_gwh: f64,
    /// Net revenue (discharge revenue minus charge cost) \[USD\].
    pub net_revenue_usd: f64,
    /// Theoretical maximum (Carnot) COP averaged over the horizon.
    pub carnot_cop: f64,
}

// ── Optimiser ─────────────────────────────────────────────────────────────────

/// Price-dispatch optimiser for grid-scale thermal energy storage.
///
/// Implements a greedy threshold strategy:
/// - Charge when the electricity price is below the low-price percentile.
/// - Discharge when the electricity price is above the high-price percentile.
/// - SoC is kept within `[min_soc_pct, max_soc_pct]`.
pub struct ThermalStorageOptimizer {
    config: ThermalStorageConfig,
    /// Electricity price forecast \[USD/MWh\], length = n_hours.
    price_forecast: Vec<f64>,
    /// Hourly heat demand for CHP mode \[MW_th\], length = n_hours.
    heat_demand: Vec<f64>,
    /// Hourly ambient temperature \[°C\], length = n_hours.
    ambient_temp: Vec<f64>,
}

impl ThermalStorageOptimizer {
    /// Create a new optimiser.
    ///
    /// `prices` must not be empty.
    pub fn new(config: ThermalStorageConfig, prices: Vec<f64>) -> Self {
        let n = prices.len();
        Self {
            config,
            price_forecast: prices,
            heat_demand: vec![0.0; n],
            ambient_temp: vec![15.0; n], // default 15 °C
        }
    }

    /// Set per-hour heat demand \[MW_th\] for CHP applications.
    pub fn set_heat_demand(&mut self, demand: Vec<f64>) {
        self.heat_demand = demand;
    }

    /// Set per-hour ambient temperature \[°C\].
    pub fn set_ambient_temperature(&mut self, temp: Vec<f64>) {
        self.ambient_temp = temp;
    }

    /// Optimise dispatch via a price-threshold greedy heuristic.
    ///
    /// The horizon is the length of the price forecast. SoC starts at 50 % of
    /// usable capacity.  Prices are rank-sorted: the cheapest 33 % trigger
    /// charging; the most expensive 33 % trigger discharging.
    ///
    /// # Errors
    /// - [`ThermalError::InvalidConfig`] — configuration parameters are invalid.
    /// - [`ThermalError::PriceLengthMismatch`] — price forecast is empty.
    pub fn optimize(&self) -> Result<ThermalStorageResult, ThermalError> {
        self.config.validate()?;
        let n = self.price_forecast.len();
        if n == 0 {
            return Err(ThermalError::PriceLengthMismatch {
                got: 0,
                expected: 1,
            });
        }

        // Compute price thresholds (low: 33rd percentile, high: 67th percentile).
        let mut sorted_prices = self.price_forecast.clone();
        sorted_prices.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
        let low_idx = (n as f64 * 0.33) as usize;
        let high_idx = (n as f64 * 0.67).min(n as f64 - 1.0) as usize;
        let price_low = sorted_prices[low_idx];
        let price_high = sorted_prices[high_idx];

        let cap_gwh = self.config.capacity_gwh_thermal;
        let min_soc = cap_gwh * self.config.min_soc_pct / 100.0;
        let max_soc = cap_gwh * self.config.max_soc_pct / 100.0;
        let self_disc_per_hour = self.config.self_discharge_pct_per_day / 100.0 / 24.0; // fraction per hour

        // Max thermal charge/discharge per hour (GWh).
        let max_charge_gwh =
            self.config.rated_charge_mw_thermal * self.config.charge_efficiency / 1000.0;
        let max_discharge_gwh_th = self.config.rated_discharge_mw_thermal / 1000.0;

        let mut soc = (min_soc + max_soc) / 2.0; // start at midpoint
        let mut dispatch = Vec::with_capacity(n);
        let mut total_charged = 0.0f64;
        let mut total_discharged_e = 0.0f64;
        let mut self_disc_loss = 0.0f64;
        let mut net_revenue = 0.0f64;
        let mut cum_cop = 0.0f64;

        for h in 0..n {
            let price = self.price_forecast[h];
            let ambient = *self.ambient_temp.get(h).unwrap_or(&15.0);
            let heat_dem = *self.heat_demand.get(h).unwrap_or(&0.0);
            let cop = self.carnot_cop(ambient);
            cum_cop += cop;

            // Compute self-discharge loss before dispatch.
            let sd_loss = soc * self_disc_per_hour;
            soc = (soc - sd_loss).max(0.0);
            self_disc_loss += sd_loss;

            let mut charge_e = 0.0f64;
            let mut discharge_e = 0.0f64;
            let mut heat_out = 0.0f64;

            if price <= price_low && soc < max_soc {
                // Charge: convert electrical to thermal.
                let available_capacity = max_soc - soc;
                let charge_th = max_charge_gwh.min(available_capacity);
                soc += charge_th;
                // Electrical power consumed = thermal / efficiency.
                charge_e = charge_th * 1000.0 / self.config.charge_efficiency; // MW_e
                total_charged += charge_th;
                net_revenue -= charge_e * price; // cost
            } else if price >= price_high && soc > min_soc {
                // Discharge: convert thermal to electrical.
                let available_th = soc - min_soc;
                let discharge_th = max_discharge_gwh_th.min(available_th);
                soc -= discharge_th;
                discharge_e = discharge_th * 1000.0 * self.config.discharge_efficiency; // MW_e
                total_discharged_e += discharge_th * self.config.discharge_efficiency;
                net_revenue += discharge_e * price; // revenue
            }

            // Heat demand satisfied from storage (CHP mode) if available.
            if heat_dem > 0.0 && soc > min_soc {
                let heat_th_gwh = (heat_dem / 1000.0).min(soc - min_soc);
                soc -= heat_th_gwh;
                heat_out = heat_th_gwh * 1000.0;
            }

            soc = soc.clamp(min_soc, max_soc);

            dispatch.push(ThermalDispatch {
                hour: h,
                charge_mw_electric: charge_e,
                discharge_mw_electric: discharge_e,
                soc_gwh: soc,
                heat_output_mw: heat_out,
                ambient_temp_c: ambient,
            });
        }

        // Compute summary statistics.
        let rt_eff = if total_charged > 1e-9 {
            (total_discharged_e / total_charged) * 100.0
        } else {
            0.0
        };
        let avg_throughput_gwh = (total_charged + total_discharged_e) / 2.0;
        let cap_factor = if cap_gwh > 1e-9 {
            (avg_throughput_gwh / (cap_gwh * n as f64)) * 100.0
        } else {
            0.0
        };
        let avg_cop = cum_cop / n as f64;

        Ok(ThermalStorageResult {
            dispatch,
            total_charged_gwh: total_charged,
            total_discharged_gwh: total_discharged_e,
            roundtrip_efficiency_pct: rt_eff,
            capacity_factor_pct: cap_factor,
            self_discharge_loss_gwh: self_disc_loss,
            net_revenue_usd: net_revenue,
            carnot_cop: avg_cop,
        })
    }

    /// Compute the theoretical Carnot COP for heat-pump charging mode.
    ///
    /// `COP = T_hot / (T_hot - T_ambient)` where temperatures are in Kelvin.
    ///
    /// If the ambient temperature equals or exceeds the hot temperature, returns 1.0
    /// (minimum physical COP).
    pub fn carnot_cop(&self, ambient_c: f64) -> f64 {
        let t_hot_k = self.config.operating_temp_hot_c + 273.15;
        let t_amb_k = ambient_c + 273.15;
        let delta = t_hot_k - t_amb_k;
        if delta <= 0.0 {
            return 1.0;
        }
        (t_hot_k / delta).max(1.0)
    }

    /// Temperature-dependent hourly self-discharge rate \[GWh_th/h\].
    ///
    /// Scales the configured daily loss by the ratio of the actual temperature
    /// difference to the nominal operating temperature difference, weighted by SoC.
    pub fn self_discharge_rate(&self, soc_gwh: f64, ambient_c: f64) -> f64 {
        let nominal_delta = self.config.operating_temp_hot_c - self.config.operating_temp_cold_c;
        let actual_delta = (self.config.operating_temp_hot_c - ambient_c).max(0.0);
        let temp_factor = if nominal_delta > 0.0 {
            actual_delta / nominal_delta
        } else {
            1.0
        };
        soc_gwh * (self.config.self_discharge_pct_per_day / 100.0 / 24.0) * temp_factor
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn molten_salt_config() -> ThermalStorageConfig {
        ThermalStorageConfig {
            storage_type: ThermalStorageType::MoltenSalt,
            capacity_gwh_thermal: 1.0, // 1 GWh_th
            rated_charge_mw_thermal: 200.0,
            rated_discharge_mw_thermal: 150.0,
            charge_efficiency: 0.95,
            discharge_efficiency: 0.40, // thermal-to-electric
            self_discharge_pct_per_day: 0.5,
            min_soc_pct: 5.0,
            max_soc_pct: 95.0,
            operating_temp_hot_c: 565.0,
            operating_temp_cold_c: 290.0,
        }
    }

    fn price_signal_low_then_high(n: usize) -> Vec<f64> {
        // First half: low price (charge); second half: high price (discharge).
        let mut prices = Vec::with_capacity(n);
        for i in 0..n {
            if i < n / 2 {
                prices.push(20.0); // cheap
            } else {
                prices.push(120.0); // expensive
            }
        }
        prices
    }

    #[test]
    fn test_charge_low_discharge_high() {
        let prices = price_signal_low_then_high(24);
        let optimizer = ThermalStorageOptimizer::new(molten_salt_config(), prices);
        let result = optimizer.optimize().expect("optimize ok");

        // Should charge in first half, discharge in second half.
        let charge_sum: f64 = result.dispatch[0..12]
            .iter()
            .map(|d| d.charge_mw_electric)
            .sum();
        let discharge_sum: f64 = result.dispatch[12..24]
            .iter()
            .map(|d| d.discharge_mw_electric)
            .sum();
        assert!(charge_sum > 0.0, "Should charge in low-price hours");
        assert!(discharge_sum > 0.0, "Should discharge in high-price hours");
    }

    #[test]
    fn test_soc_bounds_respected() {
        let prices = price_signal_low_then_high(48);
        let config = molten_salt_config();
        let optimizer = ThermalStorageOptimizer::new(config.clone(), prices);
        let result = optimizer.optimize().expect("ok");

        let cap = config.capacity_gwh_thermal;
        let min_gwh = cap * config.min_soc_pct / 100.0;
        let max_gwh = cap * config.max_soc_pct / 100.0;

        for d in &result.dispatch {
            assert!(
                d.soc_gwh >= min_gwh - 1e-9,
                "SoC {:.4} below min {:.4} at h={}",
                d.soc_gwh,
                min_gwh,
                d.hour
            );
            assert!(
                d.soc_gwh <= max_gwh + 1e-9,
                "SoC {:.4} above max {:.4} at h={}",
                d.soc_gwh,
                max_gwh,
                d.hour
            );
        }
    }

    #[test]
    fn test_roundtrip_efficiency_below_100pct() {
        let prices = price_signal_low_then_high(24);
        let optimizer = ThermalStorageOptimizer::new(molten_salt_config(), prices);
        let result = optimizer.optimize().expect("ok");

        if result.total_charged_gwh > 1e-6 {
            assert!(
                result.roundtrip_efficiency_pct < 100.0,
                "Round-trip efficiency must be < 100 %, got {:.2}",
                result.roundtrip_efficiency_pct
            );
        }
    }

    #[test]
    fn test_self_discharge_accumulates() {
        // Flat high price so no charging/discharging — only self-discharge.
        let prices = vec![200.0; 24]; // always expensive (discharge triggers, but starts mid-SoC)
        let config = molten_salt_config();
        let optimizer = ThermalStorageOptimizer::new(config, prices);
        let result = optimizer.optimize().expect("ok");
        assert!(
            result.self_discharge_loss_gwh > 0.0,
            "Self-discharge must accumulate: {:.6}",
            result.self_discharge_loss_gwh
        );
    }

    #[test]
    fn test_carnot_cop_physically_correct() {
        let config = molten_salt_config(); // hot=565°C, cold=290°C
        let optimizer = ThermalStorageOptimizer::new(config, vec![50.0; 1]);

        // At ambient = 20 °C: T_hot = 838.15 K, T_delta = 818.15 K → COP ≈ 1.024
        let cop = optimizer.carnot_cop(20.0);
        assert!(
            cop > 1.0,
            "Carnot COP must be > 1.0 for heat pump, got {cop}"
        );

        // At ambient = cold_c (290°C): T_hot = 838.15 K, delta = 275 K → COP ≈ 3.05
        let cop_warm = optimizer.carnot_cop(290.0);
        assert!(
            cop_warm > cop,
            "Warmer ambient → higher COP ({cop_warm:.3} vs {cop:.3})"
        );
    }

    #[test]
    fn test_net_revenue_positive_for_arbitrage() {
        // Charge at $10, discharge at $200 → net revenue > 0.
        let mut prices = vec![10.0; 12];
        prices.extend(vec![200.0; 12]);
        let optimizer = ThermalStorageOptimizer::new(molten_salt_config(), prices);
        let result = optimizer.optimize().expect("ok");
        assert!(
            result.net_revenue_usd > 0.0,
            "Arbitrage between $10 and $200 should be profitable, got ${:.2}",
            result.net_revenue_usd
        );
    }

    #[test]
    fn test_ptes_config_valid() {
        let config = ThermalStorageConfig {
            storage_type: ThermalStorageType::PumpedheatElectrical,
            capacity_gwh_thermal: 0.5,
            rated_charge_mw_thermal: 100.0,
            rated_discharge_mw_thermal: 80.0,
            charge_efficiency: 0.90,
            discharge_efficiency: 0.55,
            self_discharge_pct_per_day: 0.2,
            min_soc_pct: 10.0,
            max_soc_pct: 90.0,
            operating_temp_hot_c: 300.0,
            operating_temp_cold_c: -50.0,
        };
        assert!(config.validate().is_ok());
        let optimizer = ThermalStorageOptimizer::new(config, vec![50.0; 24]);
        let result = optimizer.optimize().expect("ok");
        assert_eq!(result.dispatch.len(), 24);
    }
}
