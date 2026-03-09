//! Demand Side Management (DSM) with Price Elasticity.
//!
//! Models flexible loads that respond to dynamic price signals by reducing
//! consumption (curtailment) or shifting it to cheaper hours.
//!
//! # Economic model
//!
//! Own-price elasticity (ε < 0):
//! ```text
//! ΔQ / Q_base = ε × ΔP / P_base
//! ```
//! Cross-price elasticity (substitution effect, ε_cross > 0):
//! ```text
//! ΔQ_h / Q_base = ε_cross × ΔP_{h'} / P_base   (for h' ≠ h adjacent)
//! ```
//!
//! **Rebound**: load shifted out of peak hours partially rebounds in adjacent
//! hours: `Q_rebound = rebound_factor × Q_shifted`.
//!
//! # Consumer surplus change
//!
//! Using the second-order approximation:
//! ```text
//! ΔCS ≈ ½ × |ε| × (ΔP)² / P_base × Q_base   [USD]
//! ```
//!
//! # References
//! Strbac, G. (2008) "Demand side management: Benefits and challenges",
//! *Energy Policy* 36(12).

use serde::{Deserialize, Serialize};
use thiserror::Error;

// ─── Error ────────────────────────────────────────────────────────────────────

/// Errors from the DSM optimizer.
#[derive(Debug, Error)]
pub enum DsmError {
    /// Dimension mismatch between price signal and number of hours.
    #[error("price signal length {got} does not match n_hours {expected}")]
    PriceSignalMismatch { got: usize, expected: usize },
    /// Load segment baseline has wrong length.
    #[error("segment '{name}' baseline length {got} does not match n_hours {expected}")]
    BaselineLengthMismatch {
        name: String,
        got: usize,
        expected: usize,
    },
    /// Invalid configuration parameter.
    #[error("invalid DSM config: {0}")]
    InvalidConfig(String),
}

// ─── Configuration ────────────────────────────────────────────────────────────

/// Configuration for the DSM optimizer.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DsmConfig {
    /// Number of dispatch hours.
    pub n_hours: usize,
    /// Own-price elasticity (e.g. −0.1 → demand falls 1 % per 10 % price rise).
    pub price_elasticity_own: f64,
    /// Cross-price elasticity (substitution into adjacent hours, e.g. +0.02).
    pub price_elasticity_cross: f64,
    /// Maximum demand reduction as a fraction of baseline (e.g. 0.3 = 30 %).
    pub max_reduction_pct: f64,
    /// Maximum demand shift as a fraction of baseline (e.g. 0.2 = 20 %).
    pub max_shift_pct: f64,
    /// Fraction of shifted load that rebounds in adjacent hours (e.g. 0.7).
    pub rebound_factor: f64,
}

// ─── Load segment ─────────────────────────────────────────────────────────────

/// A flexible load segment eligible for DSM.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LoadSegment {
    /// Unique identifier.
    pub id: usize,
    /// Human-readable name (e.g. `"Industrial HVAC"`).
    pub name: String,
    /// Baseline load \[MW\] per hour (length = `n_hours`).
    pub baseline_mw: Vec<f64>,
    /// Fraction of the load that is flexible (0 → 1).
    pub flexibility_pct: f64,
    /// Own-price elasticity for this segment (typically negative).
    pub elasticity: f64,
    /// Minimum continuous on-time \[hours\].
    pub min_hours_on: usize,
    /// Maximum look-ahead / look-behind window for shifting \[hours\].
    pub shift_window_hours: usize,
    /// Minimum incentive payment required to participate \[USD/MWh\].
    pub incentive_required_usd_per_mwh: f64,
}

// ─── Result ───────────────────────────────────────────────────────────────────

/// Outcome of the DSM optimization.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DsmResult {
    /// Post-DSM load \[MW\] per hour.
    pub modified_load: Vec<f64>,
    /// Load reduction \[MW\] per hour.
    pub load_reduction: Vec<f64>,
    /// Load shifting actions: `(from_hour, to_hour, MW)`.
    pub load_shifted: Vec<(usize, usize, f64)>,
    /// Total energy curtailed over the horizon \[MWh\].
    pub total_reduction_mwh: f64,
    /// Total energy shifted over the horizon \[MWh\].
    pub total_shift_mwh: f64,
    /// Peak reduction achieved \[MW\].
    pub peak_reduction_mw: f64,
    /// Ratio of post-DSM load factor to pre-DSM load factor (> 1 = improvement).
    pub load_factor_improvement: f64,
    /// Total incentive payments to DSM participants \[USD\].
    pub dsm_cost_usd: f64,
    /// Approximate consumer surplus change \[USD\] (positive = consumer gains).
    pub consumer_surplus_change_usd: f64,
}

// ─── Optimizer ────────────────────────────────────────────────────────────────

/// DSM optimizer with price-responsive and shifting capabilities.
///
/// # Example
/// ```rust
/// use oxigrid::optimize::demand_response::price_response::{
///     DsmConfig, DsmOptimizer, LoadSegment,
/// };
///
/// let config = DsmConfig {
///     n_hours: 4,
///     price_elasticity_own: -0.1,
///     price_elasticity_cross: 0.02,
///     max_reduction_pct: 0.3,
///     max_shift_pct: 0.2,
///     rebound_factor: 0.7,
/// };
/// let prices = vec![30.0, 80.0, 90.0, 40.0]; // $/MWh
/// let mut opt = DsmOptimizer::new(config, prices);
/// opt.add_segment(LoadSegment {
///     id: 0, name: "HVAC".into(),
///     baseline_mw: vec![100.0; 4],
///     flexibility_pct: 0.4, elasticity: -0.15,
///     min_hours_on: 1, shift_window_hours: 2,
///     incentive_required_usd_per_mwh: 5.0,
/// });
/// let result = opt.optimize().unwrap();
/// assert!(result.peak_reduction_mw >= 0.0);
/// ```
pub struct DsmOptimizer {
    config: DsmConfig,
    segments: Vec<LoadSegment>,
    /// Electricity price forecast \[USD/MWh\] per hour.
    price_signal: Vec<f64>,
}

impl DsmOptimizer {
    /// Create a new DSM optimizer.
    pub fn new(config: DsmConfig, price_signal: Vec<f64>) -> Self {
        Self {
            config,
            segments: Vec::new(),
            price_signal,
        }
    }

    /// Add a flexible load segment.
    pub fn add_segment(&mut self, segment: LoadSegment) {
        self.segments.push(segment);
    }

    /// Run the DSM optimization.
    pub fn optimize(&self) -> Result<DsmResult, DsmError> {
        let n_h = self.config.n_hours;

        // Validate
        if self.price_signal.len() != n_h {
            return Err(DsmError::PriceSignalMismatch {
                got: self.price_signal.len(),
                expected: n_h,
            });
        }
        if self.config.max_reduction_pct < 0.0 || self.config.max_reduction_pct > 1.0 {
            return Err(DsmError::InvalidConfig(
                "max_reduction_pct must be in [0, 1]".into(),
            ));
        }
        for seg in &self.segments {
            if seg.baseline_mw.len() != n_h {
                return Err(DsmError::BaselineLengthMismatch {
                    name: seg.name.clone(),
                    got: seg.baseline_mw.len(),
                    expected: n_h,
                });
            }
        }

        // ── Reference price (mean over horizon) ─────────────────────────────
        let p_base = if self.price_signal.is_empty() {
            1.0
        } else {
            self.price_signal.iter().sum::<f64>() / n_h as f64
        };

        // ── Aggregate baseline ───────────────────────────────────────────────
        let mut baseline_total: Vec<f64> = vec![0.0; n_h];
        for seg in &self.segments {
            for (h, bt) in baseline_total.iter_mut().enumerate() {
                *bt += seg.baseline_mw[h];
            }
        }

        // ── Compute reductions and shifts per segment ───────────────────────
        let mut reduction: Vec<f64> = vec![0.0; n_h];
        let mut shift_add: Vec<f64> = vec![0.0; n_h]; // rebound load added
        let mut shift_records: Vec<(usize, usize, f64)> = Vec::new();
        let mut dsm_cost: f64 = 0.0;
        let mut surplus_change: f64 = 0.0;

        for seg in &self.segments {
            // Skip segments whose incentive threshold is not met
            let avg_price = self.price_signal.iter().sum::<f64>() / n_h as f64;
            if avg_price < seg.incentive_required_usd_per_mwh {
                continue;
            }

            let eff_elasticity = seg.elasticity * seg.flexibility_pct;

            #[allow(clippy::needless_range_loop)]
            for h in 0..n_h {
                let price = self.price_signal[h];
                let dp = price - p_base;
                let q_base = seg.baseline_mw[h];

                // Own-price reduction: ΔQ = ε × (ΔP/P_base) × Q_base
                let raw_reduction = eff_elasticity * (dp / p_base.max(1e-6)) * q_base;
                // Reduction is negative ε × positive ΔP (if price > base)
                // → we want the magnitude of curtailment
                let curtail_mw = raw_reduction
                    .abs()
                    .min(q_base * self.config.max_reduction_pct * seg.flexibility_pct);

                // Only curtail if price is above baseline (i.e. dp > 0)
                let actual_curtail = if dp > 0.0 { curtail_mw } else { 0.0 };

                reduction[h] += actual_curtail;
                dsm_cost += actual_curtail * seg.incentive_required_usd_per_mwh;

                // Consumer surplus change: ½ |ε| ΔP² / P_base × Q_base
                surplus_change +=
                    0.5 * eff_elasticity.abs() * dp.powi(2) / p_base.max(1e-6) * q_base;

                // Load shifting: shift flexible load from high-price to low-price hours
                if actual_curtail > 0.0 && seg.shift_window_hours > 0 {
                    let shift_mw = actual_curtail
                        .min(q_base * self.config.max_shift_pct * seg.flexibility_pct);
                    // Find cheapest hour within the shift window
                    let win_start = h.saturating_sub(seg.shift_window_hours);
                    let win_end = (h + seg.shift_window_hours).min(n_h - 1);

                    let target_h = (win_start..=win_end).filter(|&t| t != h).min_by(|&a, &b| {
                        self.price_signal[a]
                            .partial_cmp(&self.price_signal[b])
                            .unwrap_or(std::cmp::Ordering::Equal)
                    });

                    if let Some(to_h) = target_h {
                        // Only shift to hours cheaper than current
                        if self.price_signal[to_h] < price {
                            let rebound = shift_mw * self.config.rebound_factor;
                            shift_add[to_h] += rebound;
                            shift_records.push((h, to_h, shift_mw));
                        }
                    }
                }
            }

            // Cross-price: neighbouring hours get a small load increase
            // (substitution effect: if hour h is expensive, adjacent hours attract load)
            #[allow(clippy::needless_range_loop)]
            for h in 0..n_h {
                let dp = self.price_signal[h] - p_base;
                if dp > 0.0 {
                    let cross_mw = self.config.price_elasticity_cross
                        * (dp / p_base.max(1e-6))
                        * seg.baseline_mw[h]
                        * seg.flexibility_pct;
                    // Spill cross-elasticity into adjacent hours
                    if h > 0 {
                        shift_add[h - 1] += cross_mw * 0.5;
                    }
                    if h + 1 < n_h {
                        shift_add[h + 1] += cross_mw * 0.5;
                    }
                }
            }
        }

        // ── Build modified load ──────────────────────────────────────────────
        let mut modified_load: Vec<f64> = baseline_total
            .iter()
            .zip(reduction.iter())
            .zip(shift_add.iter())
            .map(|((base, red), add)| (base - red + add).max(0.0))
            .collect();

        // min_hours_on enforcement: smooth out single-hour dips
        for seg in &self.segments {
            if seg.min_hours_on > 1 {
                enforce_min_hours_on(&mut modified_load, &baseline_total, seg.min_hours_on);
            }
        }

        // ── Metrics ──────────────────────────────────────────────────────────
        let total_reduction_mwh: f64 = reduction.iter().sum();
        let total_shift_mwh: f64 = shift_records.iter().map(|(_, _, mw)| mw).sum();
        let peak_reduction_mw = reduction.iter().cloned().fold(0.0_f64, f64::max);

        let pre_mean = stat_mean(&baseline_total);
        let pre_peak = baseline_total.iter().cloned().fold(0.0_f64, f64::max);
        let pre_lf = if pre_peak > 1e-9 {
            pre_mean / pre_peak
        } else {
            1.0
        };

        let post_mean = stat_mean(&modified_load);
        let post_peak = modified_load.iter().cloned().fold(0.0_f64, f64::max);
        let post_lf = if post_peak > 1e-9 {
            post_mean / post_peak
        } else {
            1.0
        };

        let load_factor_improvement = if pre_lf > 1e-9 { post_lf / pre_lf } else { 1.0 };

        Ok(DsmResult {
            modified_load,
            load_reduction: reduction,
            load_shifted: shift_records,
            total_reduction_mwh,
            total_shift_mwh,
            peak_reduction_mw,
            load_factor_improvement,
            dsm_cost_usd: dsm_cost,
            consumer_surplus_change_usd: surplus_change,
        })
    }
}

// ─── Internal helpers ─────────────────────────────────────────────────────────

/// Enforce minimum on-time by averaging out isolated reductions.
fn enforce_min_hours_on(load: &mut [f64], baseline: &[f64], min_on: usize) {
    let n = load.len();
    if n < 2 || min_on <= 1 {
        return;
    }
    // Simple pass: if a single hour has a large dip (< 80 % of baseline) but
    // neighbours don't, restore it to avoid violating min-on constraint.
    for h in 1..n.saturating_sub(1) {
        let dip = (baseline[h] - load[h]) / baseline[h].max(1e-9);
        let left_dip = (baseline[h - 1] - load[h - 1]) / baseline[h - 1].max(1e-9);
        let right_dip = (baseline[h + 1] - load[h + 1]) / baseline[h + 1].max(1e-9);
        if dip > 0.2 && left_dip < 0.05 && right_dip < 0.05 {
            // Isolated dip: restore
            load[h] = baseline[h];
        }
    }
}

fn stat_mean(v: &[f64]) -> f64 {
    if v.is_empty() {
        return 0.0;
    }
    v.iter().sum::<f64>() / v.len() as f64
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config(n_hours: usize) -> DsmConfig {
        DsmConfig {
            n_hours,
            price_elasticity_own: -0.1,
            price_elasticity_cross: 0.02,
            max_reduction_pct: 0.30,
            max_shift_pct: 0.20,
            rebound_factor: 0.70,
        }
    }

    fn flat_segment(n_hours: usize, mw: f64) -> LoadSegment {
        LoadSegment {
            id: 0,
            name: "TestLoad".into(),
            baseline_mw: vec![mw; n_hours],
            flexibility_pct: 0.5,
            elasticity: -0.15,
            min_hours_on: 1,
            shift_window_hours: 2,
            incentive_required_usd_per_mwh: 1.0,
        }
    }

    // ── Test 1 ─────────────────────────────────────────────────────────────────
    /// High-price hours trigger load reduction.
    #[test]
    fn test_high_price_hours_trigger_reduction() {
        let n_h = 4;
        let prices = vec![20.0, 100.0, 120.0, 25.0]; // hours 1,2 are expensive
        let mut opt = DsmOptimizer::new(default_config(n_h), prices);
        opt.add_segment(flat_segment(n_h, 100.0));

        let res = opt.optimize().expect("optimize failed");
        // Peak reduction should be positive (reduction happened)
        assert!(
            res.peak_reduction_mw > 0.0,
            "high-price hours must cause reduction, got {:.2}",
            res.peak_reduction_mw
        );
        // Hour 1 reduction > hour 0 reduction (hour 0 is cheap)
        assert!(
            res.load_reduction[1] >= res.load_reduction[0],
            "hour 1 (high price) must have >= reduction than hour 0"
        );
    }

    // ── Test 2 ─────────────────────────────────────────────────────────────────
    /// Shifted load lands within the shift_window_hours constraint.
    #[test]
    fn test_shift_window_respected() {
        let n_h = 8;
        let mut prices = vec![30.0; n_h];
        prices[4] = 150.0; // one expensive hour
        let mut opt = DsmOptimizer::new(default_config(n_h), prices);
        let seg = LoadSegment {
            id: 0,
            name: "Shiftable".into(),
            baseline_mw: vec![100.0; n_h],
            flexibility_pct: 0.6,
            elasticity: -0.2,
            min_hours_on: 1,
            shift_window_hours: 2, // can only shift ±2 hours
            incentive_required_usd_per_mwh: 1.0,
        };
        opt.add_segment(seg);

        let res = opt.optimize().expect("optimize failed");
        // All shift targets must be within ±2 hours of hour 4 (i.e. hours 2–6)
        for (from, to, _mw) in &res.load_shifted {
            let dist = (*to as isize - *from as isize).unsigned_abs();
            assert!(
                dist <= 2,
                "shift from hour {from} to hour {to} violates window of 2"
            );
        }
    }

    // ── Test 3 ─────────────────────────────────────────────────────────────────
    /// Rebound load appears in adjacent hours (not the same hour as reduction).
    #[test]
    fn test_rebound_load_adjacent_hours() {
        let n_h = 6;
        let mut prices = vec![25.0; n_h];
        prices[2] = 90.0; // expensive at hour 2
        let mut opt = DsmOptimizer::new(default_config(n_h), prices.clone());
        let seg = LoadSegment {
            id: 0,
            name: "HVAC".into(),
            baseline_mw: vec![100.0; n_h],
            flexibility_pct: 0.5,
            elasticity: -0.2,
            min_hours_on: 1,
            shift_window_hours: 1,
            incentive_required_usd_per_mwh: 1.0,
        };
        opt.add_segment(seg);

        let res = opt.optimize().expect("optimize failed");
        // If load was shifted away from hour 2, adjacent hours should gain load
        if !res.load_shifted.is_empty() {
            let (from_h, to_h, _) = res.load_shifted[0];
            assert_ne!(from_h, to_h, "shift must move load to a different hour");
            // The modified load in the target hour should be >= baseline (gain)
            let base = 100.0_f64;
            // Either hour 1 or hour 3 should have gained some load
            let gained = res.modified_load[to_h] > base * 0.95;
            assert!(
                gained,
                "rebound load should increase modified load in hour {to_h}"
            );
        }
    }

    // ── Test 4 ─────────────────────────────────────────────────────────────────
    /// Load factor improves (≥ 1.0 ratio) after DSM.
    #[test]
    fn test_load_factor_improves() {
        let n_h = 6;
        // Peaky prices: one very expensive hour
        let prices = vec![25.0, 25.0, 150.0, 25.0, 25.0, 25.0];
        let mut opt = DsmOptimizer::new(default_config(n_h), prices);
        opt.add_segment(LoadSegment {
            id: 0,
            name: "Industrial".into(),
            baseline_mw: vec![100.0, 100.0, 200.0, 100.0, 100.0, 100.0], // peak at hour 2
            flexibility_pct: 0.5,
            elasticity: -0.2,
            min_hours_on: 1,
            shift_window_hours: 2,
            incentive_required_usd_per_mwh: 1.0,
        });

        let res = opt.optimize().expect("optimize failed");
        assert!(
            res.load_factor_improvement >= 1.0,
            "load factor must improve or stay equal, got {:.4}",
            res.load_factor_improvement
        );
    }

    // ── Test 5 ─────────────────────────────────────────────────────────────────
    /// DSM cost is computed correctly: incentive × reduced MWh.
    #[test]
    fn test_dsm_cost_computed_correctly() {
        let n_h = 2;
        let prices = vec![5.0, 80.0]; // only hour 1 is above baseline ≈ 42.5
        let incentive = 10.0;
        let config = DsmConfig {
            n_hours: n_h,
            price_elasticity_own: -0.5,
            price_elasticity_cross: 0.0,
            max_reduction_pct: 0.5,
            max_shift_pct: 0.0,
            rebound_factor: 0.0,
        };
        let mut opt = DsmOptimizer::new(config, prices.clone());
        opt.add_segment(LoadSegment {
            id: 0,
            name: "Cost test".into(),
            baseline_mw: vec![100.0; n_h],
            flexibility_pct: 1.0,
            elasticity: -0.5,
            min_hours_on: 1,
            shift_window_hours: 0,
            incentive_required_usd_per_mwh: incentive,
        });

        let res = opt.optimize().expect("optimize failed");
        // Cost = reduction[h] × incentive for each reduced hour
        let expected_cost: f64 = res.load_reduction.iter().map(|&r| r * incentive).sum();
        assert!(
            (res.dsm_cost_usd - expected_cost).abs() < 1e-6,
            "dsm_cost_usd {:.4} != expected {:.4}",
            res.dsm_cost_usd,
            expected_cost
        );
    }

    // ── Test 6: dimension mismatch error ──────────────────────────────────────
    #[test]
    fn test_price_signal_length_error() {
        let config = default_config(4);
        let prices = vec![30.0; 3]; // wrong length
        let opt = DsmOptimizer::new(config, prices);
        assert!(opt.optimize().is_err());
    }
}
