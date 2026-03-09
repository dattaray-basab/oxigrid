//! Advanced renewable energy curtailment analysis.
//!
//! Provides detailed hourly curtailment decomposition by cause:
//! - **Technical** curtailment: line thermal ratings exceeded
//! - **Economic** curtailment: spot price below zero (negative pricing)
//! - **Frequency** curtailment: generation > load (frequency limit)
//! - **Voltage** curtailment: voltage constraint violation
//! - **Congestion** curtailment: transmission congestion on specific lines
//!
//! Curtailment is allocated proportionally across all online renewable units.
//!
//! # Economic metric
//! Cost = curtailed\_MWh × `compensation_rate_usd_per_mwh` \[USD\]
//!
//! # CO2 metric
//! CO2 missed = curtailed\_MWh × emission_factor (default 0.45 t/MWh)
//!
//! # References
//! - Fink, S. et al., "Wind Energy Curtailment Case Studies", NREL/TP-550-46716, 2009
//! - Bird, L. et al., "Integrating Variable Renewable Energy", NREL, 2013

use thiserror::Error;

// ── Error type ────────────────────────────────────────────────────────────────

/// Errors produced by the curtailment analyser.
#[derive(Debug, Error)]
pub enum CurtailmentError {
    /// Mismatch between number of hours and data vector length.
    #[error("Data length {got} does not match n_hours {expected}")]
    LengthMismatch { got: usize, expected: usize },
    /// No renewable units have been added.
    #[error("No renewable units registered")]
    NoUnits,
    /// Configuration is invalid.
    #[error("Invalid configuration: {0}")]
    InvalidConfig(String),
}

// ── Enumerations ──────────────────────────────────────────────────────────────

/// Renewable generation technology.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RenewableTech {
    /// Solar photovoltaic.
    Solar,
    /// Wind power.
    Wind,
    /// Run-of-river hydroelectric.
    RunOfRiver,
    /// Tidal power.
    Tidal,
    /// Geothermal.
    Geothermal,
}

// ── Configuration ─────────────────────────────────────────────────────────────

/// Configuration for the curtailment analysis.
#[derive(Debug, Clone)]
pub struct CurtailmentConfig {
    /// Number of hours in the analysis horizon.
    pub n_hours: usize,
    /// System nominal voltage \[kV\].
    pub system_voltage_kv: f64,
    /// MVA base for per-unit calculations \[MVA\].
    pub base_mva: f64,
    /// Opportunity cost of curtailment (foregone revenue) \[USD/MWh\].
    pub curtailment_cost_usd_per_mwh: f64,
    /// Compensation paid to curtailed generator \[USD/MWh\].
    pub compensation_rate_usd_per_mwh: f64,
}

// ── Renewable unit ────────────────────────────────────────────────────────────

/// A single renewable generation unit.
#[derive(Debug, Clone)]
pub struct RenewableUnit {
    /// Unique identifier.
    pub id: usize,
    /// Bus connection.
    pub bus: usize,
    /// Generation technology.
    pub technology: RenewableTech,
    /// Nameplate capacity \[MW\].
    pub rated_mw: f64,
    /// Hourly available generation \[MW\] (length = n_hours).
    pub available_mw: Vec<f64>,
    /// Marginal cost \[USD/MWh\] (typically ≈ 0 for renewables).
    pub marginal_cost: f64,
}

// ── Curtailment reason flags ──────────────────────────────────────────────────

/// Flags indicating which physical or economic reasons drove curtailment.
#[derive(Debug, Clone, Default)]
pub struct CurtailmentReason {
    /// Grid constraint (line rating, voltage) caused curtailment.
    pub technical_curtailment: bool,
    /// Negative spot price triggered economic curtailment.
    pub economic_curtailment: bool,
    /// Excess generation relative to load triggered frequency limit.
    pub frequency_curtailment: bool,
    /// Voltage limit binding at some bus.
    pub voltage_curtailment: bool,
    /// Transmission congestion caused curtailment.
    pub congestion_curtailment: bool,
}

// ── Hourly curtailment record ─────────────────────────────────────────────────

/// Curtailment details for a single unit in a single hour.
#[derive(Debug, Clone)]
pub struct HourlyCurtailment {
    /// Hour index (0-based).
    pub hour: usize,
    /// Unit identifier.
    pub unit_id: usize,
    /// Available generation \[MW\].
    pub available_mw: f64,
    /// Actually dispatched (uncurtailed) generation \[MW\].
    pub dispatched_mw: f64,
    /// Curtailed volume \[MW\].
    pub curtailed_mw: f64,
    /// Curtailment fraction \[%\].
    pub curtailment_pct: f64,
    /// Reasons for curtailment.
    pub reason: CurtailmentReason,
    /// Cost of curtailment \[USD\].
    pub curtailment_cost_usd: f64,
}

// ── Aggregated reasons ────────────────────────────────────────────────────────

/// Total curtailment volume broken down by cause \[GWh\].
#[derive(Debug, Clone, Default)]
pub struct CurtailmentByReason {
    /// Technical grid constraint curtailment \[GWh\].
    pub technical_gwh: f64,
    /// Economic (negative price) curtailment \[GWh\].
    pub economic_gwh: f64,
    /// Frequency-driven curtailment \[GWh\].
    pub frequency_gwh: f64,
    /// Voltage-driven curtailment \[GWh\].
    pub voltage_gwh: f64,
    /// Congestion curtailment \[GWh\].
    pub congestion_gwh: f64,
}

// ── Analysis result ───────────────────────────────────────────────────────────

/// Full curtailment analysis result.
#[derive(Debug, Clone)]
pub struct CurtailmentAnalysisResult {
    /// Per-unit, per-hour curtailment records.
    pub hourly: Vec<HourlyCurtailment>,
    /// Total renewable energy available over the horizon \[GWh\].
    pub total_available_gwh: f64,
    /// Total energy actually dispatched \[GWh\].
    pub total_dispatched_gwh: f64,
    /// Total energy curtailed \[GWh\].
    pub total_curtailed_gwh: f64,
    /// Overall curtailment rate \[%\].
    pub curtailment_rate_pct: f64,
    /// Peak curtailment across all hours and units \[MW\].
    pub peak_curtailment_mw: f64,
    /// Hour at which peak curtailment occurred.
    pub peak_curtailment_hour: usize,
    /// Curtailment disaggregated by cause.
    pub by_reason: CurtailmentByReason,
    /// Total financial cost of curtailment \[USD\].
    pub total_cost_usd: f64,
    /// CO2 emission reduction foregone due to curtailment \[tonnes\].
    pub co2_avoided_less_t: f64,
    /// Potential MW reduction in curtailment from a hypothetical grid upgrade.
    pub grid_upgrade_benefit_mw: f64,
}

// ── Analyser ─────────────────────────────────────────────────────────────────

/// Analyser for renewable curtailment causes and magnitudes.
pub struct CurtailmentAnalyzer {
    config: CurtailmentConfig,
    units: Vec<RenewableUnit>,
    /// Hourly system load \[MW\].
    load_mw: Vec<f64>,
    /// Thermal ratings per branch \[MW\].
    line_ratings: Vec<f64>,
    /// Acceptable frequency range (min, max) \[Hz\].
    freq_limits: (f64, f64),
    /// Acceptable voltage range (min, max) \[pu\].
    voltage_limits: (f64, f64),
    /// Hourly spot prices \[USD/MWh\].
    spot_prices: Vec<f64>,
}

impl CurtailmentAnalyzer {
    /// Create a new analyser with default limits.
    pub fn new(config: CurtailmentConfig) -> Self {
        let n = config.n_hours;
        Self {
            config,
            units: Vec::new(),
            load_mw: vec![0.0; n],
            line_ratings: Vec::new(),
            freq_limits: (49.0, 51.0),
            voltage_limits: (0.9, 1.1),
            spot_prices: vec![0.0; n],
        }
    }

    /// Register a renewable unit with the analyser.
    pub fn add_unit(&mut self, unit: RenewableUnit) {
        self.units.push(unit);
    }

    /// Set hourly load profile \[MW\].
    pub fn set_load(&mut self, load_mw: Vec<f64>) {
        self.load_mw = load_mw;
    }

    /// Set hourly spot prices \[USD/MWh\].
    pub fn set_spot_prices(&mut self, prices: Vec<f64>) {
        self.spot_prices = prices;
    }

    /// Set line thermal ratings \[MW\].
    pub fn set_line_ratings(&mut self, ratings: Vec<f64>) {
        self.line_ratings = ratings;
    }

    /// Set frequency operating limits \[Hz\].
    pub fn set_freq_limits(&mut self, min_hz: f64, max_hz: f64) {
        self.freq_limits = (min_hz, max_hz);
    }

    /// Set voltage operating limits \[pu\].
    pub fn set_voltage_limits(&mut self, min_pu: f64, max_pu: f64) {
        self.voltage_limits = (min_pu, max_pu);
    }

    /// Run the curtailment analysis.
    ///
    /// # Algorithm
    /// For each hour:
    /// 1. Sum total available renewable generation.
    /// 2. Determine maximum dispatch from system constraints.
    /// 3. If constrained, compute curtailment per unit (proportional).
    /// 4. Classify the binding constraint as the curtailment reason.
    /// 5. Accumulate statistics.
    ///
    /// # Errors
    /// - [`CurtailmentError::NoUnits`] — no units registered.
    /// - [`CurtailmentError::LengthMismatch`] — data vector length mismatch.
    pub fn analyze(&self) -> Result<CurtailmentAnalysisResult, CurtailmentError> {
        if self.units.is_empty() {
            return Err(CurtailmentError::NoUnits);
        }
        let n = self.config.n_hours;

        // Minimum line rating (most constraining single line).
        let min_line_rating = self
            .line_ratings
            .iter()
            .cloned()
            .fold(f64::INFINITY, f64::min);
        let effective_line_cap = if self.line_ratings.is_empty() {
            f64::INFINITY
        } else {
            min_line_rating
        };

        let mut hourly: Vec<HourlyCurtailment> = Vec::new();
        let mut total_available = 0.0f64;
        let mut total_dispatched = 0.0f64;
        let mut total_curtailed = 0.0f64;
        let mut peak_curt_mw = 0.0f64;
        let mut peak_curt_hour = 0usize;
        let mut total_cost = 0.0f64;
        let mut by_reason = CurtailmentByReason::default();

        // CO2 intensity of displaced conventional generation \[t/MWh\].
        const CO2_INTENSITY: f64 = 0.45;

        for h in 0..n {
            let load = *self.load_mw.get(h).unwrap_or(&0.0);
            let price = *self.spot_prices.get(h).unwrap_or(&50.0);

            // Total available renewable at this hour.
            let total_avail_h: f64 = self
                .units
                .iter()
                .map(|u| *u.available_mw.get(h).unwrap_or(&0.0))
                .sum();

            // Determine constraints and maximum allowable dispatch.
            let mut max_dispatch = total_avail_h;
            let mut reason = CurtailmentReason::default();

            // 1. Economic curtailment: negative spot price.
            if price < 0.0 {
                max_dispatch = 0.0;
                reason.economic_curtailment = true;
            }

            // 2. Frequency curtailment: generation >> load (excess).
            if total_avail_h > load * 1.05 && load > 0.0 {
                let freq_limited = load * 1.05;
                if freq_limited < max_dispatch {
                    max_dispatch = freq_limited;
                    reason.frequency_curtailment = true;
                }
            }

            // 3. Congestion / line rating curtailment.
            if total_avail_h > effective_line_cap && effective_line_cap < max_dispatch {
                max_dispatch = effective_line_cap;
                reason.congestion_curtailment = true;
                reason.technical_curtailment = true;
            }

            // 4. Voltage curtailment (simplified: curtail if reactive not supported).
            // Proxy: if renewable > 90 % of load and voltage_limits.max < 1.05.
            let vmax = self.voltage_limits.1;
            if total_avail_h > 0.9 * load && vmax < 1.05 && load > 0.0 {
                let volt_limited = 0.9 * load;
                if volt_limited < max_dispatch {
                    max_dispatch = volt_limited;
                    reason.voltage_curtailment = true;
                    reason.technical_curtailment = true;
                }
            }

            max_dispatch = max_dispatch.max(0.0);
            let total_curtailed_h = (total_avail_h - max_dispatch).max(0.0);

            // Proportional allocation across units.
            for unit in &self.units {
                let avail = *unit.available_mw.get(h).unwrap_or(&0.0);
                let unit_curt = if total_avail_h > 1e-9 {
                    total_curtailed_h * (avail / total_avail_h)
                } else {
                    0.0
                };
                let dispatched = (avail - unit_curt).max(0.0);
                let curt_pct = if avail > 1e-9 {
                    unit_curt / avail * 100.0
                } else {
                    0.0
                };
                let cost = unit_curt * self.config.compensation_rate_usd_per_mwh;

                total_available += avail;
                total_dispatched += dispatched;
                total_curtailed += unit_curt;
                total_cost += cost;

                if unit_curt > peak_curt_mw {
                    peak_curt_mw = unit_curt;
                    peak_curt_hour = h;
                }

                hourly.push(HourlyCurtailment {
                    hour: h,
                    unit_id: unit.id,
                    available_mw: avail,
                    dispatched_mw: dispatched,
                    curtailed_mw: unit_curt,
                    curtailment_pct: curt_pct,
                    reason: reason.clone(),
                    curtailment_cost_usd: cost,
                });
            }

            // Aggregate by reason (1 h → GWh).
            let curt_gwh = total_curtailed_h / 1000.0;
            if reason.technical_curtailment {
                by_reason.technical_gwh += curt_gwh;
            }
            if reason.economic_curtailment {
                by_reason.economic_gwh += curt_gwh;
            }
            if reason.frequency_curtailment {
                by_reason.frequency_gwh += curt_gwh;
            }
            if reason.voltage_curtailment {
                by_reason.voltage_gwh += curt_gwh;
            }
            if reason.congestion_curtailment {
                by_reason.congestion_gwh += curt_gwh;
            }
        }

        let avail_gwh = total_available / 1000.0;
        let disp_gwh = total_dispatched / 1000.0;
        let curt_gwh = total_curtailed / 1000.0;
        let curt_rate = if avail_gwh > 1e-9 {
            curt_gwh / avail_gwh * 100.0
        } else {
            0.0
        };

        let grid_upgrade_benefit = self.estimate_upgrade_benefit(&hourly);

        Ok(CurtailmentAnalysisResult {
            hourly,
            total_available_gwh: avail_gwh,
            total_dispatched_gwh: disp_gwh,
            total_curtailed_gwh: curt_gwh,
            curtailment_rate_pct: curt_rate,
            peak_curtailment_mw: peak_curt_mw,
            peak_curtailment_hour: peak_curt_hour,
            by_reason,
            total_cost_usd: total_cost,
            co2_avoided_less_t: curt_gwh * 1000.0 * CO2_INTENSITY, // GWh → MWh × t/MWh
            grid_upgrade_benefit_mw: grid_upgrade_benefit,
        })
    }

    /// Estimate the grid capacity upgrade \[MW\] that would maximally reduce congestion curtailment.
    ///
    /// Returns the 95th percentile of congestion-curtailed MW across all hours,
    /// as a proxy for the minimum line-rating upgrade required to eliminate
    /// most congestion events.
    fn estimate_upgrade_benefit(&self, results: &[HourlyCurtailment]) -> f64 {
        let congestion_only: Vec<f64> = results
            .iter()
            .filter(|r| r.reason.congestion_curtailment && r.curtailed_mw > 0.0)
            .map(|r| r.curtailed_mw)
            .collect();

        if congestion_only.is_empty() {
            return 0.0;
        }

        let mut sorted = congestion_only;
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
        let idx = ((sorted.len() as f64 * 0.95) as usize).min(sorted.len() - 1);
        sorted[idx]
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_unit(id: usize, available: Vec<f64>) -> RenewableUnit {
        RenewableUnit {
            id,
            bus: id,
            technology: RenewableTech::Wind,
            rated_mw: 100.0,
            available_mw: available,
            marginal_cost: 0.0,
        }
    }

    fn make_config(n: usize) -> CurtailmentConfig {
        CurtailmentConfig {
            n_hours: n,
            system_voltage_kv: 110.0,
            base_mva: 100.0,
            curtailment_cost_usd_per_mwh: 60.0,
            compensation_rate_usd_per_mwh: 50.0,
        }
    }

    #[test]
    fn test_no_curtailment_when_renewable_lt_load() {
        let n = 24;
        let mut analyzer = CurtailmentAnalyzer::new(make_config(n));
        // Available: 50 MW every hour, load: 200 MW.
        analyzer.add_unit(make_unit(0, vec![50.0; n]));
        analyzer.set_load(vec![200.0; n]);
        analyzer.set_spot_prices(vec![50.0; n]);

        let result = analyzer.analyze().expect("ok");
        assert!(
            result.total_curtailed_gwh < 1e-9,
            "No curtailment when renewable < load: {}",
            result.total_curtailed_gwh
        );
        assert!(result.curtailment_rate_pct < 1e-9);
    }

    #[test]
    fn test_economic_curtailment_negative_prices() {
        let n = 24;
        let mut analyzer = CurtailmentAnalyzer::new(make_config(n));
        analyzer.add_unit(make_unit(0, vec![100.0; n]));
        analyzer.set_load(vec![200.0; n]);

        // Set negative prices for first 12 hours.
        let mut prices = vec![-10.0; 12];
        prices.extend(vec![50.0; 12]);
        analyzer.set_spot_prices(prices);

        let result = analyzer.analyze().expect("ok");
        // Should curtail all 100 MW for 12 h → 1200 MWh = 1.2 GWh
        assert!(
            result.total_curtailed_gwh > 0.0,
            "Economic curtailment expected"
        );
        let econ_hours: Vec<_> = result
            .hourly
            .iter()
            .filter(|r| r.reason.economic_curtailment && r.curtailed_mw > 0.0)
            .collect();
        assert!(
            !econ_hours.is_empty(),
            "Some hours should show economic curtailment"
        );
    }

    #[test]
    fn test_congestion_curtailment_line_limit_hit() {
        let n = 4;
        let mut analyzer = CurtailmentAnalyzer::new(make_config(n));
        // 200 MW available, but line rating is only 80 MW.
        analyzer.add_unit(make_unit(0, vec![200.0; n]));
        analyzer.set_load(vec![300.0; n]);
        analyzer.set_spot_prices(vec![50.0; n]);
        analyzer.set_line_ratings(vec![80.0]); // 80 MW thermal limit

        let result = analyzer.analyze().expect("ok");
        // 200 - 80 = 120 MW curtailed per hour × 4 h = 480 MWh = 0.48 GWh
        assert!(
            result.total_curtailed_gwh > 0.0,
            "Congestion curtailment expected: {:?}",
            result.total_curtailed_gwh
        );
        let congestion_hours: Vec<_> = result
            .hourly
            .iter()
            .filter(|r| r.reason.congestion_curtailment)
            .collect();
        assert!(
            !congestion_hours.is_empty(),
            "Congestion reason should be set"
        );
    }

    #[test]
    fn test_cost_calculated_correctly() {
        let n = 1;
        let mut analyzer = CurtailmentAnalyzer::new(CurtailmentConfig {
            n_hours: n,
            system_voltage_kv: 110.0,
            base_mva: 100.0,
            curtailment_cost_usd_per_mwh: 60.0,
            compensation_rate_usd_per_mwh: 50.0,
        });
        // 100 MW available, negative price → all curtailed.
        analyzer.add_unit(make_unit(0, vec![100.0]));
        analyzer.set_load(vec![200.0]);
        analyzer.set_spot_prices(vec![-5.0]);

        let result = analyzer.analyze().expect("ok");
        // 100 MWh curtailed × $50/MWh = $5000
        assert!(
            (result.total_cost_usd - 5000.0).abs() < 1e-6,
            "Cost should be $5000, got ${:.2}",
            result.total_cost_usd
        );
    }

    #[test]
    fn test_co2_impact_from_curtailment() {
        let n = 1;
        let mut analyzer = CurtailmentAnalyzer::new(make_config(n));
        // 100 MW curtailed for 1 h → 100 MWh × 0.45 t/MWh = 45 t CO2.
        analyzer.add_unit(make_unit(0, vec![100.0]));
        analyzer.set_load(vec![200.0]);
        analyzer.set_spot_prices(vec![-1.0]); // negative → full curtailment

        let result = analyzer.analyze().expect("ok");
        assert!(result.total_curtailed_gwh > 0.0, "Should have curtailment");
        // CO2 = curtailed_GWh × 1000 × 0.45 t/MWh
        let expected_co2 = result.total_curtailed_gwh * 1000.0 * 0.45;
        assert!(
            (result.co2_avoided_less_t - expected_co2).abs() < 1e-6,
            "CO2 {:.2} vs expected {:.2}",
            result.co2_avoided_less_t,
            expected_co2
        );
    }

    #[test]
    fn test_peak_curtailment_tracked_correctly() {
        let n = 4;
        let mut analyzer = CurtailmentAnalyzer::new(make_config(n));
        // Hours 0,1: 50 MW available; hours 2,3: 200 MW available.
        // Load: 100 MW. No congestion. Prices: $50 positive.
        // So only frequency curtailment possible at h=2,3 where avail > 1.05×load.
        let available = vec![50.0, 50.0, 200.0, 200.0];
        analyzer.add_unit(make_unit(0, available));
        analyzer.set_load(vec![100.0; n]);
        analyzer.set_spot_prices(vec![50.0; n]);

        let result = analyzer.analyze().expect("ok");
        if result.peak_curtailment_mw > 0.0 {
            assert!(
                result.peak_curtailment_hour >= 2,
                "Peak should be at h=2 or h=3, got {}",
                result.peak_curtailment_hour
            );
        }
    }

    #[test]
    fn test_multiple_units_proportional_curtailment() {
        let n = 2;
        let mut analyzer = CurtailmentAnalyzer::new(make_config(n));
        // Unit 0: 60 MW, Unit 1: 40 MW; total = 100 MW; line limit = 50 MW.
        analyzer.add_unit(make_unit(0, vec![60.0; n]));
        analyzer.add_unit(make_unit(1, vec![40.0; n]));
        analyzer.set_load(vec![200.0; n]);
        analyzer.set_spot_prices(vec![50.0; n]);
        analyzer.set_line_ratings(vec![50.0]);

        let result = analyzer.analyze().expect("ok");
        // Total curtailed: 100 - 50 = 50 MW per hour
        // Unit 0 share: 60/100 × 50 = 30 MW; unit 1 share: 40/100 × 50 = 20 MW
        let unit0_h0 = result
            .hourly
            .iter()
            .find(|r| r.hour == 0 && r.unit_id == 0)
            .map(|r| r.curtailed_mw)
            .unwrap_or(0.0);
        let unit1_h0 = result
            .hourly
            .iter()
            .find(|r| r.hour == 0 && r.unit_id == 1)
            .map(|r| r.curtailed_mw)
            .unwrap_or(0.0);
        assert!(
            (unit0_h0 - 30.0).abs() < 1e-6,
            "Unit 0 curtailment should be 30 MW, got {unit0_h0:.4}"
        );
        assert!(
            (unit1_h0 - 20.0).abs() < 1e-6,
            "Unit 1 curtailment should be 20 MW, got {unit1_h0:.4}"
        );
    }
}
