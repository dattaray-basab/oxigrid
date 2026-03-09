//! Operational analytics for power system KPI computation, trend analysis,
//! anomaly scoring, and operational efficiency assessment.
//!
//! Provides a full pipeline from raw KPI readings to dashboards with
//! health scores, trend detection, and statistical anomaly detection.

use std::collections::HashMap;

// ---------------------------------------------------------------------------
// Enums
// ---------------------------------------------------------------------------

/// Category of a power-system operational KPI.
#[derive(Debug, Clone, PartialEq)]
pub enum OperationalKpiCategory {
    /// Reliability indices (SAIDI, SAIFI, CAIDI, ASAI, ENS …)
    Reliability,
    /// Operational efficiency (loss factor, capacity factor, load factor …)
    Efficiency,
    /// Power quality (THD, flicker, voltage deviations …)
    Quality,
    /// Environmental metrics (CO₂ intensity, renewable fraction …)
    Environmental,
    /// Economic metrics (LCOE, cost per GWh …)
    Economic,
    /// Safety KPIs (fault rate, clearance times …)
    Safety,
    /// Regulatory compliance indicators
    Compliance,
}

/// Direction of a KPI trend over the observed history.
#[derive(Debug, Clone, PartialEq)]
pub enum TrendDirection {
    /// KPI is moving in a favourable direction.
    Improving,
    /// KPI is essentially flat (|slope| ≤ 0.01 per period).
    Stable,
    /// KPI is moving in an unfavourable direction.
    Degrading,
    /// KPI exhibits high variability relative to its trend.
    Volatile,
}

/// Time interval at which KPI readings are collected.
#[derive(Debug, Clone, PartialEq)]
pub enum AnalyticsInterval {
    /// One reading per hour.
    Hourly,
    /// One reading per day.
    Daily,
    /// One reading per week.
    Weekly,
    /// One reading per month.
    Monthly,
    /// One reading per quarter.
    Quarterly,
    /// One reading per year.
    Annual,
}

// ---------------------------------------------------------------------------
// Structs
// ---------------------------------------------------------------------------

/// A single instantaneous reading of a power-system operational KPI.
#[derive(Debug, Clone)]
pub struct OperationalKpi {
    /// Unique identifier.
    pub id: usize,
    /// Human-readable name (used as the history key).
    pub name: String,
    /// Category used for health-score weighting.
    pub category: OperationalKpiCategory,
    /// Measured value.
    pub value: f64,
    /// Engineering unit string (e.g. `"h"`, `"%"`, `"MWh"`).
    pub unit: String,
    /// Optional performance target.
    pub target: Option<f64>,
    /// Optional warning threshold.
    pub threshold_warning: Option<f64>,
    /// Optional critical threshold.
    pub threshold_critical: Option<f64>,
    /// `true` when a larger value is desirable (e.g. ASAI),
    /// `false` when a smaller value is desirable (e.g. SAIDI).
    pub higher_is_better: bool,
    /// Unix-epoch timestamp (seconds) of this reading.
    pub timestamp: f64,
}

/// Historical time-series for a single KPI including pre-computed statistics.
#[derive(Debug, Clone)]
pub struct TimeSeriesKpi {
    /// KPI name matching the key in `OperationalAnalytics::kpi_history`.
    pub kpi_name: String,
    /// Ordered timestamps of each reading.
    pub timestamps: Vec<f64>,
    /// Ordered values corresponding to `timestamps`.
    pub values: Vec<f64>,
    /// Nominal collection interval.
    pub interval: AnalyticsInterval,
    /// Linear regression slope (units per period).
    pub trend_slope: f64,
    /// Qualitative trend direction derived from the slope.
    pub trend_direction: TrendDirection,
    /// Arithmetic mean of `values`.
    pub mean: f64,
    /// Population standard deviation of `values`.
    pub std_dev: f64,
    /// Minimum observed value.
    pub min: f64,
    /// Maximum observed value.
    pub max: f64,
    /// 95th percentile of `values`.
    pub percentile_95: f64,
}

/// Aggregated efficiency metrics for a reporting period.
#[derive(Debug, Clone)]
pub struct EfficiencyReport {
    /// Start of the reporting period (Unix epoch, seconds).
    pub period_start: f64,
    /// End of the reporting period (Unix epoch, seconds).
    pub period_end: f64,
    /// Total generated energy \[GWh\].
    pub total_generation_gwh: f64,
    /// Total served demand \[GWh\].
    pub total_demand_gwh: f64,
    /// Total network losses \[GWh\].
    pub total_losses_gwh: f64,
    /// Loss factor = losses / generation × 100 \[%\].
    pub loss_factor_pct: f64,
    /// Renewable energy fraction = renewable / generation × 100 \[%\].
    pub renewable_fraction_pct: f64,
    /// Capacity factor = average generation / peak generation × 100 \[%\].
    pub capacity_factor_pct: f64,
    /// Load factor = average demand / peak demand × 100 \[%\].
    pub load_factor_pct: f64,
    /// Carbon intensity \[g CO₂/kWh\]; caller-supplied or zero.
    pub co2_intensity_g_per_kwh: f64,
    /// Weighted average cost \[M USD / GWh\]; caller-supplied or zero.
    pub cost_per_gwh_musd: f64,
}

/// Statistical anomaly score for a single KPI observation.
#[derive(Debug, Clone)]
pub struct AnomalyScore {
    /// Timestamp of the observation being scored.
    pub timestamp: f64,
    /// Name of the KPI being scored.
    pub kpi_name: String,
    /// Raw z-score: `(value − mean) / std_dev`.
    pub z_score: f64,
    /// IQR-normalised deviation score in \[0, 1\].
    pub iqr_score: f64,
    /// Combined anomaly score in \[0, 1\]: `0.6 * z_norm + 0.4 * iqr_score`.
    pub combined_score: f64,
    /// `true` when `combined_score` exceeds the configured threshold (default 0.7).
    pub is_anomaly: bool,
    /// Human-readable explanation.
    pub description: String,
}

/// Aggregated operational dashboard snapshot.
#[derive(Debug, Clone)]
pub struct OperationalDashboard {
    /// Name of the power system or area.
    pub system_name: String,
    /// Current KPI readings included in the snapshot.
    pub kpis: Vec<OperationalKpi>,
    /// Alert strings for KPIs that breach warning or critical thresholds.
    pub alerts: Vec<String>,
    /// Period-level efficiency summary.
    pub efficiency_summary: EfficiencyReport,
    /// Detected anomalies sorted descending by `combined_score`.
    pub top_anomalies: Vec<AnomalyScore>,
    /// Weighted overall health score in \[0, 100\].
    pub overall_health_score: f64,
}

// ---------------------------------------------------------------------------
// Main analytics engine
// ---------------------------------------------------------------------------

/// Main operational analytics engine.
///
/// Accumulates KPI readings, computes statistics, detects anomalies,
/// and generates operational dashboards.
#[derive(Debug, Clone)]
pub struct OperationalAnalytics {
    /// KPI history keyed by KPI name.
    pub kpi_history: HashMap<String, TimeSeriesKpi>,
    /// Combined anomaly score threshold above which a reading is flagged
    /// as an anomaly (default 0.7).
    pub anomaly_threshold: f64,
    /// Number of historical periods used for rolling statistics (default 30).
    pub lookback_periods: usize,
}

impl Default for OperationalAnalytics {
    fn default() -> Self {
        Self::new()
    }
}

impl OperationalAnalytics {
    /// Create a new analytics engine with default parameters.
    pub fn new() -> Self {
        Self {
            kpi_history: HashMap::new(),
            anomaly_threshold: 0.7,
            lookback_periods: 30,
        }
    }

    /// Append a KPI reading to the internal history.
    ///
    /// If no history entry exists for `kpi.name`, one is created with
    /// `AnalyticsInterval::Daily` as the default interval.
    pub fn add_kpi_reading(&mut self, kpi: OperationalKpi) {
        let entry = self
            .kpi_history
            .entry(kpi.name.clone())
            .or_insert_with(|| TimeSeriesKpi {
                kpi_name: kpi.name.clone(),
                timestamps: Vec::new(),
                values: Vec::new(),
                interval: AnalyticsInterval::Daily,
                trend_slope: 0.0,
                trend_direction: TrendDirection::Stable,
                mean: 0.0,
                std_dev: 0.0,
                min: f64::MAX,
                max: f64::MIN,
                percentile_95: 0.0,
            });
        entry.timestamps.push(kpi.timestamp);
        entry.values.push(kpi.value);
    }

    /// Compute full descriptive statistics for a named KPI.
    ///
    /// Returns `None` if the KPI has no history or an empty value list.
    pub fn compute_time_series_stats(&self, kpi_name: &str) -> Option<TimeSeriesKpi> {
        let ts = self.kpi_history.get(kpi_name)?;
        let values = &ts.values;
        if values.is_empty() {
            return None;
        }
        let n = values.len() as f64;
        let mean = values.iter().sum::<f64>() / n;
        let variance = values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / n;
        let std_dev = variance.sqrt();
        let min = values.iter().cloned().fold(f64::MAX, f64::min);
        let max = values.iter().cloned().fold(f64::MIN, f64::max);
        let percentile_95 = Self::compute_percentile(values, 0.95);
        let (slope, direction) = Self::compute_trend(values);

        Some(TimeSeriesKpi {
            kpi_name: kpi_name.to_string(),
            timestamps: ts.timestamps.clone(),
            values: values.clone(),
            interval: ts.interval.clone(),
            trend_slope: slope,
            trend_direction: direction,
            mean,
            std_dev,
            min,
            max,
            percentile_95,
        })
    }

    /// Score a new value for anomalousness relative to the historical
    /// distribution of `kpi_name`.
    ///
    /// Returns a default (non-anomaly) score when the KPI has fewer than
    /// two historical readings.
    pub fn detect_anomalies(&self, kpi_name: &str, new_value: f64) -> AnomalyScore {
        let default_score = AnomalyScore {
            timestamp: 0.0,
            kpi_name: kpi_name.to_string(),
            z_score: 0.0,
            iqr_score: 0.0,
            combined_score: 0.0,
            is_anomaly: false,
            description: "Insufficient history".to_string(),
        };

        let ts = match self.kpi_history.get(kpi_name) {
            Some(t) => t,
            None => return default_score,
        };

        if ts.values.len() < 2 {
            return default_score;
        }

        let values = &ts.values;
        let n = values.len() as f64;
        let mean = values.iter().sum::<f64>() / n;
        let variance = values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / n;
        let std_dev = variance.sqrt();

        let z = if std_dev > 1e-12 {
            (new_value - mean) / std_dev
        } else {
            0.0
        };
        let z_normalized = (z.abs() / 5.0).min(1.0);

        let iqr_score = Self::compute_iqr_score(new_value, values);
        let combined = 0.6 * z_normalized + 0.4 * iqr_score;
        let is_anomaly = combined > self.anomaly_threshold;

        let description = if is_anomaly {
            format!(
                "Anomaly detected: value={:.3}, z={:.2}, iqr_score={:.2}",
                new_value, z, iqr_score
            )
        } else {
            format!("Normal: value={:.3}, z={:.2}", new_value, z)
        };

        AnomalyScore {
            timestamp: 0.0,
            kpi_name: kpi_name.to_string(),
            z_score: z,
            iqr_score,
            combined_score: combined,
            is_anomaly,
            description,
        }
    }

    /// Compute the linear regression slope of a value series and classify
    /// the trend direction.
    ///
    /// # Algorithm
    /// `slope = (n·Σxy − Σx·Σy) / (n·Σx² − (Σx)²)`
    ///
    /// The x-axis is the sample index (0, 1, 2 …).
    ///
    /// `Volatile` is declared when `std_dev > 2·|slope|·n`.
    /// Otherwise `slope > 0.01` → `Improving`, `slope < −0.01` → `Degrading`,
    /// else `Stable`.
    pub fn compute_trend(values: &[f64]) -> (f64, TrendDirection) {
        let n = values.len();
        if n < 2 {
            return (0.0, TrendDirection::Stable);
        }
        let nf = n as f64;
        let sum_x: f64 = (0..n).map(|i| i as f64).sum();
        let sum_y: f64 = values.iter().sum();
        let sum_xy: f64 = values.iter().enumerate().map(|(i, y)| i as f64 * y).sum();
        let sum_x2: f64 = (0..n).map(|i| (i as f64).powi(2)).sum();

        let denom = nf * sum_x2 - sum_x * sum_x;
        let slope = if denom.abs() > 1e-12 {
            (nf * sum_xy - sum_x * sum_y) / denom
        } else {
            0.0
        };

        let mean = sum_y / nf;
        let variance = values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / nf;
        let std_dev = variance.sqrt();

        let direction = if std_dev > 2.0 * slope.abs() * nf {
            TrendDirection::Volatile
        } else if slope > 0.01 {
            TrendDirection::Improving
        } else if slope < -0.01 {
            TrendDirection::Degrading
        } else {
            TrendDirection::Stable
        };

        (slope, direction)
    }

    /// Compute an [`EfficiencyReport`] from parallel arrays of energy values.
    ///
    /// All energy arrays (`generation`, `demand`, `losses`, `renewable`) are
    /// expected in **MWh**; results are stored in **GWh**.
    ///
    /// `timestamps` provides the period boundaries; `period_start` and
    /// `period_end` are the first and last values respectively.
    pub fn compute_efficiency_report(
        &self,
        generation: &[f64],
        demand: &[f64],
        losses: &[f64],
        renewable: &[f64],
        timestamps: &[f64],
    ) -> EfficiencyReport {
        let total_gen = generation.iter().sum::<f64>() / 1_000.0;
        let total_dem = demand.iter().sum::<f64>() / 1_000.0;
        let total_loss = losses.iter().sum::<f64>() / 1_000.0;
        let total_ren = renewable.iter().sum::<f64>() / 1_000.0;

        let loss_factor_pct = if total_gen > 1e-12 {
            total_loss / total_gen * 100.0
        } else {
            0.0
        };
        let renewable_fraction_pct = if total_gen > 1e-12 {
            total_ren / total_gen * 100.0
        } else {
            0.0
        };

        let avg_gen = if generation.is_empty() {
            0.0
        } else {
            generation.iter().sum::<f64>() / generation.len() as f64
        };
        let peak_gen = generation.iter().cloned().fold(0.0_f64, f64::max);
        let capacity_factor_pct = if peak_gen > 1e-12 {
            avg_gen / peak_gen * 100.0
        } else {
            0.0
        };

        let avg_dem = if demand.is_empty() {
            0.0
        } else {
            demand.iter().sum::<f64>() / demand.len() as f64
        };
        let peak_dem = demand.iter().cloned().fold(0.0_f64, f64::max);
        let load_factor_pct = if peak_dem > 1e-12 {
            avg_dem / peak_dem * 100.0
        } else {
            0.0
        };

        let period_start = timestamps.first().cloned().unwrap_or(0.0);
        let period_end = timestamps.last().cloned().unwrap_or(0.0);

        EfficiencyReport {
            period_start,
            period_end,
            total_generation_gwh: total_gen,
            total_demand_gwh: total_dem,
            total_losses_gwh: total_loss,
            loss_factor_pct,
            renewable_fraction_pct,
            capacity_factor_pct,
            load_factor_pct,
            co2_intensity_g_per_kwh: 0.0,
            cost_per_gwh_musd: 0.0,
        }
    }

    /// Generate a full [`OperationalDashboard`] for the given system.
    ///
    /// Threshold breaches produce alert strings. Anomaly scores are computed
    /// for every current KPI against its history and sorted descending.
    pub fn generate_dashboard(
        &self,
        system_name: &str,
        current_kpis: Vec<OperationalKpi>,
        efficiency: EfficiencyReport,
    ) -> OperationalDashboard {
        let mut alerts = Vec::new();

        for kpi in &current_kpis {
            // Critical check first.
            if let Some(crit) = kpi.threshold_critical {
                let breached = if kpi.higher_is_better {
                    kpi.value < crit
                } else {
                    kpi.value > crit
                };
                if breached {
                    alerts.push(format!(
                        "CRITICAL: {} = {:.3} {} (critical threshold: {:.3})",
                        kpi.name, kpi.value, kpi.unit, crit
                    ));
                    continue; // skip warning check if already critical
                }
            }
            if let Some(warn) = kpi.threshold_warning {
                let breached = if kpi.higher_is_better {
                    kpi.value < warn
                } else {
                    kpi.value > warn
                };
                if breached {
                    alerts.push(format!(
                        "WARNING: {} = {:.3} {} (warning threshold: {:.3})",
                        kpi.name, kpi.value, kpi.unit, warn
                    ));
                }
            }
        }

        let mut anomalies: Vec<AnomalyScore> = current_kpis
            .iter()
            .map(|kpi| self.detect_anomalies(&kpi.name, kpi.value))
            .filter(|s| s.is_anomaly)
            .collect();

        anomalies.sort_by(|a, b| {
            b.combined_score
                .partial_cmp(&a.combined_score)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let health = self.compute_overall_health(&current_kpis);

        OperationalDashboard {
            system_name: system_name.to_string(),
            kpis: current_kpis,
            alerts,
            efficiency_summary: efficiency,
            top_anomalies: anomalies,
            overall_health_score: health,
        }
    }

    /// Compute a weighted health score in \[0, 100\] across all provided KPIs.
    ///
    /// | Category    | Weight |
    /// |-------------|--------|
    /// | Reliability | 0.30   |
    /// | Safety      | 0.25   |
    /// | Efficiency  | 0.20   |
    /// | Quality     | 0.15   |
    /// | Others      | 0.10   |
    ///
    /// Each KPI contributes a score of 100 when at or better than its target,
    /// 0 when at or worse than its critical threshold, and a linearly
    /// interpolated value in between.  KPIs without both `target` and
    /// `threshold_critical` contribute 100.
    pub fn compute_overall_health(&self, kpis: &[OperationalKpi]) -> f64 {
        if kpis.is_empty() {
            return 100.0;
        }

        let category_weight = |cat: &OperationalKpiCategory| -> f64 {
            match cat {
                OperationalKpiCategory::Reliability => 0.30,
                OperationalKpiCategory::Safety => 0.25,
                OperationalKpiCategory::Efficiency => 0.20,
                OperationalKpiCategory::Quality => 0.15,
                _ => 0.10,
            }
        };

        let mut weighted_sum = 0.0_f64;
        let mut weight_total = 0.0_f64;

        for kpi in kpis {
            let w = category_weight(&kpi.category);
            let score: f64 = match (kpi.target, kpi.threshold_critical) {
                (Some(target), Some(crit)) => {
                    if kpi.higher_is_better {
                        if kpi.value >= target {
                            100.0
                        } else if kpi.value <= crit {
                            0.0
                        } else {
                            let span = target - crit;
                            if span.abs() > 1e-12 {
                                (kpi.value - crit) / span * 100.0
                            } else {
                                0.0
                            }
                        }
                    } else {
                        // lower is better
                        if kpi.value <= target {
                            100.0
                        } else if kpi.value >= crit {
                            0.0
                        } else {
                            let span = crit - target;
                            if span.abs() > 1e-12 {
                                (crit - kpi.value) / span * 100.0
                            } else {
                                0.0
                            }
                        }
                    }
                }
                (Some(target), None) => {
                    if kpi.higher_is_better {
                        if kpi.value >= target {
                            100.0
                        } else {
                            (kpi.value / target * 100.0).max(0.0)
                        }
                    } else {
                        if kpi.value <= target {
                            100.0
                        } else {
                            (target / kpi.value * 100.0).max(0.0)
                        }
                    }
                }
                _ => 100.0,
            };

            weighted_sum += w * score.clamp(0.0, 100.0);
            weight_total += w;
        }

        if weight_total > 1e-12 {
            weighted_sum / weight_total
        } else {
            100.0
        }
    }

    /// Compute the `p`-th percentile of `values` using linear interpolation.
    ///
    /// `p` must be in \[0, 1\]; e.g. `p = 0.95` for the 95th percentile.
    /// Returns 0.0 for an empty slice.
    pub fn compute_percentile(values: &[f64], p: f64) -> f64 {
        if values.is_empty() {
            return 0.0;
        }
        let mut sorted = values.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let n = sorted.len();
        if n == 1 {
            return sorted[0];
        }
        let idx = p * (n - 1) as f64;
        let lo = idx.floor() as usize;
        let hi = (idx.ceil() as usize).min(n - 1);
        let frac = idx - lo as f64;
        sorted[lo] + frac * (sorted[hi] - sorted[lo])
    }

    /// Compute an IQR-based anomaly score in \[0, 1\].
    ///
    /// # Formula
    /// ```text
    /// score = min(1.0, |value − median| / (1.5 · IQR))
    /// ```
    ///
    /// Returns 0.0 when the IQR is negligibly small (all values identical).
    pub fn compute_iqr_score(value: f64, values: &[f64]) -> f64 {
        if values.is_empty() {
            return 0.0;
        }
        let median = Self::compute_percentile(values, 0.5);
        let q1 = Self::compute_percentile(values, 0.25);
        let q3 = Self::compute_percentile(values, 0.75);
        let iqr = q3 - q1;
        if iqr < 1e-12 {
            return 0.0;
        }
        ((value - median).abs() / (1.5 * iqr)).min(1.0)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_kpi(
        name: &str,
        value: f64,
        cat: OperationalKpiCategory,
        target: Option<f64>,
        crit: Option<f64>,
        higher: bool,
    ) -> OperationalKpi {
        OperationalKpi {
            id: 0,
            name: name.to_string(),
            category: cat,
            value,
            unit: String::new(),
            target,
            threshold_warning: None,
            threshold_critical: crit,
            higher_is_better: higher,
            timestamp: 0.0,
        }
    }

    #[test]
    fn test_add_kpi_reading() {
        let mut analytics = OperationalAnalytics::new();
        let kpi = make_kpi(
            "SAIDI",
            10.0,
            OperationalKpiCategory::Reliability,
            None,
            None,
            false,
        );
        analytics.add_kpi_reading(kpi);
        assert_eq!(analytics.kpi_history["SAIDI"].values.len(), 1);
    }

    #[test]
    fn test_compute_stats_mean() {
        let mut analytics = OperationalAnalytics::new();
        for v in [1.0_f64, 2.0, 3.0, 4.0, 5.0] {
            let kpi = make_kpi("X", v, OperationalKpiCategory::Efficiency, None, None, true);
            analytics.add_kpi_reading(kpi);
        }
        let stats = analytics.compute_time_series_stats("X").unwrap();
        assert!((stats.mean - 3.0).abs() < 1e-9);
    }

    #[test]
    fn test_compute_stats_std() {
        let mut analytics = OperationalAnalytics::new();
        for v in [2.0_f64, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0] {
            let kpi = make_kpi("Y", v, OperationalKpiCategory::Efficiency, None, None, true);
            analytics.add_kpi_reading(kpi);
        }
        let stats = analytics.compute_time_series_stats("Y").unwrap();
        assert!((stats.std_dev - 2.0).abs() < 1e-6);
    }

    #[test]
    fn test_compute_stats_percentile95() {
        let mut analytics = OperationalAnalytics::new();
        for v in 1..=100_u32 {
            let kpi = make_kpi(
                "P",
                v as f64,
                OperationalKpiCategory::Quality,
                None,
                None,
                true,
            );
            analytics.add_kpi_reading(kpi);
        }
        let stats = analytics.compute_time_series_stats("P").unwrap();
        assert!(
            stats.percentile_95 >= 94.0 && stats.percentile_95 <= 96.0,
            "p95 = {}",
            stats.percentile_95
        );
    }

    #[test]
    fn test_trend_improving() {
        let values: Vec<f64> = (0..20).map(|i| i as f64).collect();
        let (_, dir) = OperationalAnalytics::compute_trend(&values);
        assert_eq!(dir, TrendDirection::Improving);
    }

    #[test]
    fn test_trend_degrading() {
        let values: Vec<f64> = (0..20).map(|i| -(i as f64)).collect();
        let (_, dir) = OperationalAnalytics::compute_trend(&values);
        assert_eq!(dir, TrendDirection::Degrading);
    }

    #[test]
    fn test_trend_stable() {
        let values: Vec<f64> = vec![5.0; 20];
        let (_, dir) = OperationalAnalytics::compute_trend(&values);
        assert_eq!(dir, TrendDirection::Stable);
    }

    #[test]
    fn test_trend_slope_positive() {
        let values: Vec<f64> = (0..10).map(|i| i as f64 * 2.0).collect();
        let (slope, _) = OperationalAnalytics::compute_trend(&values);
        assert!(slope > 0.0, "slope should be positive, got {slope}");
    }

    #[test]
    fn test_anomaly_detection_normal() {
        let mut analytics = OperationalAnalytics::new();
        for v in 0..30_u32 {
            let kpi = make_kpi(
                "V",
                v as f64,
                OperationalKpiCategory::Reliability,
                None,
                None,
                true,
            );
            analytics.add_kpi_reading(kpi);
        }
        // mean ≈ 14.5, std ≈ 8.8 — value 15 is well within normal range.
        let score = analytics.detect_anomalies("V", 15.0);
        assert!(!score.is_anomaly);
        assert!(score.combined_score < 0.7);
    }

    #[test]
    fn test_anomaly_detection_outlier() {
        let mut analytics = OperationalAnalytics::new();
        for v in 0..30_u32 {
            let kpi = make_kpi(
                "W",
                v as f64,
                OperationalKpiCategory::Reliability,
                None,
                None,
                true,
            );
            analytics.add_kpi_reading(kpi);
        }
        // 5-sigma outlier relative to mean≈14.5, std≈8.8.
        let score = analytics.detect_anomalies("W", 200.0);
        assert!(score.is_anomaly, "200 should be flagged as anomaly");
    }

    #[test]
    fn test_z_score_zero_mean() {
        let mut analytics = OperationalAnalytics::new();
        for _ in 0..10 {
            let kpi = make_kpi(
                "Z",
                5.0,
                OperationalKpiCategory::Efficiency,
                None,
                None,
                true,
            );
            analytics.add_kpi_reading(kpi);
        }
        // std_dev = 0 → z forced to 0.
        let score = analytics.detect_anomalies("Z", 5.0);
        assert!(
            score.z_score.abs() < 1e-9,
            "z should be 0, got {}",
            score.z_score
        );
    }

    #[test]
    fn test_iqr_score_median_value() {
        let values: Vec<f64> = (1..=10).map(|i| i as f64).collect();
        // median = 5.5, Q1 = 3.25, Q3 = 7.75, IQR = 4.5
        // |5.5 − 5.5| / (1.5 × 4.5) = 0
        let score = OperationalAnalytics::compute_iqr_score(5.5, &values);
        assert!(
            score.abs() < 1e-9,
            "iqr_score for median should be 0, got {score}"
        );
    }

    #[test]
    fn test_efficiency_report_loss_factor() {
        let analytics = OperationalAnalytics::new();
        let gen = vec![1_000.0_f64; 10]; // 10 000 MWh = 10 GWh
        let dem = vec![900.0_f64; 10];
        let loss = vec![100.0_f64; 10]; // 1 000 MWh = 1 GWh → 10 %
        let ren = vec![0.0_f64; 10];
        let ts = vec![0.0_f64; 10];
        let report = analytics.compute_efficiency_report(&gen, &dem, &loss, &ren, &ts);
        assert!(
            (report.loss_factor_pct - 10.0).abs() < 1e-6,
            "loss_factor_pct = {}",
            report.loss_factor_pct
        );
    }

    #[test]
    fn test_efficiency_report_renewable_fraction() {
        let analytics = OperationalAnalytics::new();
        let gen = vec![1_000.0_f64; 10];
        let dem = vec![800.0_f64; 10];
        let loss = vec![0.0_f64; 10];
        let ren = vec![400.0_f64; 10]; // 40 %
        let ts = vec![0.0_f64; 10];
        let report = analytics.compute_efficiency_report(&gen, &dem, &loss, &ren, &ts);
        assert!(
            (report.renewable_fraction_pct - 40.0).abs() < 1e-6,
            "renewable_fraction_pct = {}",
            report.renewable_fraction_pct
        );
    }

    #[test]
    fn test_efficiency_report_load_factor() {
        let analytics = OperationalAnalytics::new();
        let gen = vec![100.0_f64, 200.0, 300.0];
        let dem = vec![100.0_f64, 200.0, 300.0]; // avg=200, peak=300 → 66.67 %
        let loss = vec![0.0_f64; 3];
        let ren = vec![0.0_f64; 3];
        let ts = vec![0.0_f64; 3];
        let report = analytics.compute_efficiency_report(&gen, &dem, &loss, &ren, &ts);
        let expected = 200.0 / 300.0 * 100.0;
        assert!(
            (report.load_factor_pct - expected).abs() < 1e-4,
            "load_factor_pct = {}",
            report.load_factor_pct
        );
    }

    #[test]
    fn test_dashboard_generation() {
        let analytics = OperationalAnalytics::new();
        let kpis = vec![
            make_kpi(
                "A",
                10.0,
                OperationalKpiCategory::Reliability,
                None,
                None,
                true,
            ),
            make_kpi(
                "B",
                20.0,
                OperationalKpiCategory::Efficiency,
                None,
                None,
                true,
            ),
        ];
        let eff = analytics.compute_efficiency_report(&[], &[], &[], &[], &[]);
        let dash = analytics.generate_dashboard("TestGrid", kpis, eff);
        assert_eq!(dash.kpis.len(), 2);
        assert_eq!(dash.system_name, "TestGrid");
    }

    #[test]
    fn test_dashboard_alerts() {
        let analytics = OperationalAnalytics::new();
        // higher_is_better=false → critical breach when value > critical threshold
        let kpi = OperationalKpi {
            id: 0,
            name: "SAIDI".to_string(),
            category: OperationalKpiCategory::Reliability,
            value: 10.0,
            unit: "h".to_string(),
            target: None,
            threshold_warning: None,
            threshold_critical: Some(5.0),
            higher_is_better: false,
            timestamp: 0.0,
        };
        let eff = analytics.compute_efficiency_report(&[], &[], &[], &[], &[]);
        let dash = analytics.generate_dashboard("Grid", vec![kpi], eff);
        assert!(!dash.alerts.is_empty(), "should have at least one alert");
    }

    #[test]
    fn test_overall_health_all_good() {
        let analytics = OperationalAnalytics::new();
        let kpis = vec![OperationalKpi {
            id: 0,
            name: "A".to_string(),
            category: OperationalKpiCategory::Reliability,
            value: 100.0,
            unit: String::new(),
            target: Some(90.0),
            threshold_warning: None,
            threshold_critical: Some(0.0),
            higher_is_better: true,
            timestamp: 0.0,
        }];
        let health = analytics.compute_overall_health(&kpis);
        assert!(
            (health - 100.0).abs() < 1e-6,
            "health should be 100, got {health}"
        );
    }

    #[test]
    fn test_overall_health_all_critical() {
        let analytics = OperationalAnalytics::new();
        let kpis = vec![OperationalKpi {
            id: 0,
            name: "A".to_string(),
            category: OperationalKpiCategory::Reliability,
            value: 0.0,
            unit: String::new(),
            target: Some(90.0),
            threshold_warning: None,
            threshold_critical: Some(0.0),
            higher_is_better: true,
            timestamp: 0.0,
        }];
        let health = analytics.compute_overall_health(&kpis);
        assert!(health < 1e-6, "health should be 0, got {health}");
    }

    #[test]
    fn test_percentile_known_values() {
        let values: Vec<f64> = (0..=10).map(|i| i as f64).collect();
        let p50 = OperationalAnalytics::compute_percentile(&values, 0.5);
        assert!((p50 - 5.0).abs() < 1e-9, "p50 should be 5, got {p50}");
    }

    #[test]
    fn test_time_series_min_max() {
        let mut analytics = OperationalAnalytics::new();
        for v in [3.0_f64, 7.0, 1.0, 9.0, 5.0] {
            let kpi = make_kpi("M", v, OperationalKpiCategory::Quality, None, None, true);
            analytics.add_kpi_reading(kpi);
        }
        let stats = analytics.compute_time_series_stats("M").unwrap();
        assert!((stats.min - 1.0).abs() < 1e-9);
        assert!((stats.max - 9.0).abs() < 1e-9);
    }

    #[test]
    fn test_combined_score_bounds() {
        let mut analytics = OperationalAnalytics::new();
        for v in 0..50_u32 {
            let kpi = make_kpi(
                "C",
                v as f64,
                OperationalKpiCategory::Efficiency,
                None,
                None,
                true,
            );
            analytics.add_kpi_reading(kpi);
        }
        let score = analytics.detect_anomalies("C", 1_000.0);
        assert!(score.combined_score <= 1.0, "combined_score <= 1 violated");
        assert!(score.combined_score >= 0.0, "combined_score >= 0 violated");
    }
}
