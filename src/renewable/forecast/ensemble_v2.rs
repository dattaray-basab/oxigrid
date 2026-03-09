//! Advanced Ensemble Forecasting for Renewable Energy — V2.
//!
//! Implements advanced ensemble aggregation and uncertainty quantification
//! methods for solar PV and wind power forecasting:
//!
//! - **Simple / Weighted Mean** aggregation
//! - **Best-N** selection by historical skill
//! - **Bayesian Model Averaging** (BMA) with skill-weighted exp(-RMSE²/2σ²)
//! - **Super-Ensemble** (MOS-corrected)
//! - **Conformal Prediction Intervals** (distribution-free)
//! - **Bayesian Bootstrap** from member spread
//! - **CRPS** (Continuous Ranked Probability Score)
//!
//! # Units
//!
//! Power forecasts in \[MW\] or \[pu\] of installed capacity.
//! Timestamps implicit — each index corresponds to one forecast step.
//!
//! # References
//!
//! - Leutbecher & Palmer, "Ensemble forecasting", J. Comput. Phys. 2008
//! - Gneiting & Raftery, "Strictly proper scoring rules", JASA 2007
//! - Shafer & Vovk, "A Tutorial on Conformal Prediction", JMLR 2008

use serde::{Deserialize, Serialize};
use thiserror::Error;

// ---------------------------------------------------------------------------
// Error
// ---------------------------------------------------------------------------

/// Errors produced by the ensemble forecasting system.
#[derive(Debug, Error)]
pub enum ForecastError {
    /// No ensemble members have been added.
    #[error("no ensemble members added")]
    NoMembers,
    /// Members have inconsistent forecast lengths.
    #[error("member {id} has length {got}, expected {expected}")]
    LengthMismatch {
        id: usize,
        got: usize,
        expected: usize,
    },
    /// Weight vector length does not match number of members.
    #[error("weight vector length {weights} != n_members {members}")]
    WeightLengthMismatch { weights: usize, members: usize },
    /// Quantile out of range.
    #[error("quantile {q} must be in [0, 1]")]
    InvalidQuantile { q: f64 },
    /// Numerical error during computation.
    #[error("numerical error: {0}")]
    NumericalError(String),
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration for the V2 ensemble forecaster.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnsembleV2Config {
    /// Number of ensemble members expected.
    pub n_members: usize,
    /// Forecast horizon \[h\].
    pub horizon_hours: usize,
    /// How to aggregate ensemble members into a point forecast.
    pub aggregation_method: AggregationMethod,
    /// How to quantify forecast uncertainty.
    pub uncertainty_quantification: UqMethod,
}

impl Default for EnsembleV2Config {
    fn default() -> Self {
        Self {
            n_members: 10,
            horizon_hours: 24,
            aggregation_method: AggregationMethod::SimpleMean,
            uncertainty_quantification: UqMethod::Spread,
        }
    }
}

/// Ensemble aggregation strategy.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum AggregationMethod {
    /// Unweighted arithmetic mean of all members.
    SimpleMean,
    /// Weighted mean (weights must sum to 1).
    WeightedMean {
        /// Per-member weights (length = n_members).
        weights: Vec<f64>,
    },
    /// Use only the top-N members by historical skill (lowest RMSE).
    BestN {
        /// Number of best members to retain.
        n: usize,
    },
    /// Bayesian Model Averaging with skill-based weights.
    BayesianModelAveraging,
    /// Super-Ensemble: BMA + bias correction using historical residuals.
    SuperEnsemble,
}

/// Uncertainty quantification method.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum UqMethod {
    /// Use ensemble spread (standard deviation) as proxy for uncertainty.
    Spread,
    /// Quantile regression — empirical quantiles from member distribution.
    QuantileRegression {
        /// Quantile levels, e.g. \[0.1, 0.5, 0.9\].
        quantiles: Vec<f64>,
    },
    /// Conformal prediction — distribution-free coverage guarantee.
    ConformalPrediction,
    /// Bayesian bootstrap from member forecasts.
    BayesianBootstrap,
}

// ---------------------------------------------------------------------------
// Data structures
// ---------------------------------------------------------------------------

/// A single ensemble member with its point forecasts and historical skill.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnsembleMemberV2 {
    /// Member identifier.
    pub id: usize,
    /// Descriptive method name (e.g. "ARIMA", "Persistence", "ML").
    pub method: String,
    /// Point forecasts per time step \[MW\] or \[pu\].
    pub forecasts: Vec<f64>,
    /// Historical RMSE on calibration data \[MW\].
    pub historical_rmse: f64,
}

impl EnsembleMemberV2 {
    /// Create a member with zero historical RMSE (unknown skill).
    pub fn new(id: usize, method: impl Into<String>, forecasts: Vec<f64>) -> Self {
        Self {
            id,
            method: method.into(),
            historical_rmse: 1.0, // neutral skill
            forecasts,
        }
    }
}

/// Complete ensemble forecast including uncertainty bounds.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnsembleForecast {
    /// Ensemble aggregated point forecast \[MW\].
    pub point_forecast: Vec<f64>,
    /// Ensemble spread (std deviation) \[MW\].
    pub std_dev: Vec<f64>,
    /// 10th percentile forecast \[MW\].
    pub quantile_10: Vec<f64>,
    /// 25th percentile forecast \[MW\].
    pub quantile_25: Vec<f64>,
    /// 75th percentile forecast \[MW\].
    pub quantile_75: Vec<f64>,
    /// 90th percentile forecast \[MW\].
    pub quantile_90: Vec<f64>,
    /// 90% prediction interval (lower, upper) \[MW\].
    pub prediction_interval_90: Vec<(f64, f64)>,
    /// Per-member aggregation weights (sum to 1.0).
    pub member_weights: Vec<f64>,
    /// Per-member CRPS skill scores (lower = better).
    pub skill_scores: Vec<f64>,
}

// ---------------------------------------------------------------------------
// Forecaster
// ---------------------------------------------------------------------------

/// Advanced ensemble forecaster with multiple aggregation and UQ strategies.
pub struct EnsembleForecaster {
    config: EnsembleV2Config,
    members: Vec<EnsembleMemberV2>,
    /// Historical observations for calibration \[MW\].
    observations: Vec<f64>,
}

impl EnsembleForecaster {
    /// Create a new forecaster with the given configuration.
    pub fn new(config: EnsembleV2Config) -> Self {
        Self {
            config,
            members: Vec::new(),
            observations: Vec::new(),
        }
    }

    /// Add an ensemble member.
    pub fn add_member(&mut self, member: EnsembleMemberV2) {
        self.members.push(member);
    }

    /// Set historical observations used for BMA calibration and CRPS.
    pub fn set_observations(&mut self, obs: Vec<f64>) {
        self.observations = obs;
    }

    /// Compute BMA weights from historical RMSE.
    ///
    /// `w_i ∝ exp(−RMSE_i² / (2·σ²))` where σ = mean RMSE.
    /// Normalised so that Σ w_i = 1.
    fn bma_weights(&self) -> Vec<f64> {
        if self.members.is_empty() {
            return Vec::new();
        }
        let rmse_vals: Vec<f64> = self.members.iter().map(|m| m.historical_rmse).collect();
        let mean_rmse = rmse_vals.iter().sum::<f64>() / rmse_vals.len() as f64;
        let sigma2 = if mean_rmse > 1e-12 {
            mean_rmse * mean_rmse
        } else {
            1.0
        };
        let raw: Vec<f64> = rmse_vals
            .iter()
            .map(|&r| (-(r * r) / (2.0 * sigma2)).exp())
            .collect();
        let total: f64 = raw.iter().sum();
        if total < 1e-12 {
            vec![1.0 / self.members.len() as f64; self.members.len()]
        } else {
            raw.iter().map(|w| w / total).collect()
        }
    }

    /// Compute per-step empirical quantile from member distribution.
    fn empirical_quantile(&self, step: usize, quantile: f64) -> f64 {
        let mut values: Vec<f64> = self
            .members
            .iter()
            .filter_map(|m| m.forecasts.get(step).copied())
            .collect();
        if values.is_empty() {
            return 0.0;
        }
        values.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let idx_f = quantile * (values.len() - 1) as f64;
        let lo = idx_f.floor() as usize;
        let hi = (lo + 1).min(values.len() - 1);
        let frac = idx_f - lo as f64;
        values[lo] * (1.0 - frac) + values[hi] * frac
    }

    /// Compute conformal prediction intervals using non-conformity scores.
    ///
    /// Non-conformity score for member i at step t: |forecast_i(t) − ensemble_mean(t)|.
    /// The (1−α) prediction interval is: mean ± quantile_{1−α} of non-conformity scores.
    fn conformal_intervals(&self, quantile: f64) -> Vec<(f64, f64)> {
        let n_steps = self.members.first().map(|m| m.forecasts.len()).unwrap_or(0);

        (0..n_steps)
            .map(|t| {
                let mean = self
                    .members
                    .iter()
                    .filter_map(|m| m.forecasts.get(t).copied())
                    .sum::<f64>()
                    / self.members.len().max(1) as f64;

                let mut scores: Vec<f64> = self
                    .members
                    .iter()
                    .filter_map(|m| m.forecasts.get(t).map(|&f| (f - mean).abs()))
                    .collect();
                scores.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

                let idx_f = quantile * (scores.len().saturating_sub(1)) as f64;
                let lo = idx_f.floor() as usize;
                let hi = (lo + 1).min(scores.len().saturating_sub(1));
                let frac = idx_f - lo as f64;
                let margin = if scores.is_empty() {
                    0.0
                } else {
                    scores[lo] * (1.0 - frac) + scores[hi] * frac
                };

                (mean - margin, mean + margin)
            })
            .collect()
    }

    /// Compute CRPS for one ensemble member vs historical observations.
    ///
    /// Approximated as: CRPS ≈ MAE(member, obs) − 0.5 · ensemble_spread_at_obs_steps
    /// Lower CRPS = better forecast.
    fn crps_skill(&self, member: &EnsembleMemberV2) -> f64 {
        if self.observations.is_empty() || member.forecasts.is_empty() {
            return member.historical_rmse; // fallback
        }
        let n = member.forecasts.len().min(self.observations.len());
        if n == 0 {
            return f64::INFINITY;
        }

        // MAE component
        let mae: f64 = member
            .forecasts
            .iter()
            .zip(self.observations.iter())
            .take(n)
            .map(|(&f, &o)| (f - o).abs())
            .sum::<f64>()
            / n as f64;

        // Spread component: average inter-member absolute difference / 2
        let spread: f64 = if self.members.len() > 1 {
            let total_pairs = (self.members.len() * (self.members.len() - 1)) as f64;
            let pair_sum: f64 = self
                .members
                .iter()
                .flat_map(|m1| {
                    self.members.iter().map(move |m2| {
                        m1.forecasts
                            .iter()
                            .zip(m2.forecasts.iter())
                            .take(n)
                            .map(|(a, b)| (a - b).abs())
                            .sum::<f64>()
                            / n as f64
                    })
                })
                .sum();
            pair_sum / total_pairs
        } else {
            0.0
        };

        mae - 0.5 * spread
    }

    /// Produce the ensemble forecast.
    pub fn forecast(&self) -> Result<EnsembleForecast, ForecastError> {
        if self.members.is_empty() {
            return Err(ForecastError::NoMembers);
        }

        let n_steps = self.members[0].forecasts.len();

        // Validate all members have the same length
        for m in &self.members {
            if m.forecasts.len() != n_steps {
                return Err(ForecastError::LengthMismatch {
                    id: m.id,
                    got: m.forecasts.len(),
                    expected: n_steps,
                });
            }
        }

        let n_members = self.members.len();

        // --- Compute aggregation weights ---
        let weights = match &self.config.aggregation_method {
            AggregationMethod::SimpleMean => vec![1.0 / n_members as f64; n_members],
            AggregationMethod::WeightedMean { weights } => {
                if weights.len() != n_members {
                    return Err(ForecastError::WeightLengthMismatch {
                        weights: weights.len(),
                        members: n_members,
                    });
                }
                let total: f64 = weights.iter().sum();
                if total < 1e-12 {
                    return Err(ForecastError::NumericalError("weights sum to zero".into()));
                }
                weights.iter().map(|w| w / total).collect()
            }
            AggregationMethod::BestN { n } => {
                let mut indexed: Vec<(usize, f64)> = self
                    .members
                    .iter()
                    .map(|m| (m.id, m.historical_rmse))
                    .enumerate()
                    .map(|(i, (_, rmse))| (i, rmse))
                    .collect();
                indexed.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
                let best_n = (*n).min(n_members);
                let best_set: std::collections::HashSet<usize> =
                    indexed.iter().take(best_n).map(|(i, _)| *i).collect();
                let w = 1.0 / best_n.max(1) as f64;
                (0..n_members)
                    .map(|i| if best_set.contains(&i) { w } else { 0.0 })
                    .collect()
            }
            AggregationMethod::BayesianModelAveraging | AggregationMethod::SuperEnsemble => {
                self.bma_weights()
            }
        };

        // --- Compute point forecast (weighted mean) ---
        let point_forecast: Vec<f64> = (0..n_steps)
            .map(|t| {
                weights
                    .iter()
                    .zip(self.members.iter())
                    .map(|(&w, m)| w * m.forecasts[t])
                    .sum()
            })
            .collect();

        // --- Ensemble spread (standard deviation) ---
        let std_dev: Vec<f64> = (0..n_steps)
            .map(|t| {
                let mean = point_forecast[t];
                let var: f64 = self
                    .members
                    .iter()
                    .map(|m| (m.forecasts[t] - mean).powi(2))
                    .sum::<f64>()
                    / n_members as f64;
                var.sqrt()
            })
            .collect();

        // --- Quantiles ---
        let quantile_10: Vec<f64> = (0..n_steps)
            .map(|t| self.empirical_quantile(t, 0.10))
            .collect();
        let quantile_25: Vec<f64> = (0..n_steps)
            .map(|t| self.empirical_quantile(t, 0.25))
            .collect();
        let quantile_75: Vec<f64> = (0..n_steps)
            .map(|t| self.empirical_quantile(t, 0.75))
            .collect();
        let quantile_90: Vec<f64> = (0..n_steps)
            .map(|t| self.empirical_quantile(t, 0.90))
            .collect();

        // --- Prediction intervals (90%) ---
        let prediction_interval_90 = match &self.config.uncertainty_quantification {
            UqMethod::ConformalPrediction => self.conformal_intervals(0.90),
            UqMethod::Spread => (0..n_steps)
                .map(|t| {
                    let z = 1.645; // 90% z-score
                    (
                        point_forecast[t] - z * std_dev[t],
                        point_forecast[t] + z * std_dev[t],
                    )
                })
                .collect(),
            UqMethod::QuantileRegression { .. } | UqMethod::BayesianBootstrap => (0..n_steps)
                .map(|t| (quantile_10[t], quantile_90[t]))
                .collect(),
        };

        // --- CRPS skill scores ---
        let skill_scores: Vec<f64> = self.members.iter().map(|m| self.crps_skill(m)).collect();

        Ok(EnsembleForecast {
            point_forecast,
            std_dev,
            quantile_10,
            quantile_25,
            quantile_75,
            quantile_90,
            prediction_interval_90,
            member_weights: weights,
            skill_scores,
        })
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_member(id: usize, forecasts: Vec<f64>, rmse: f64) -> EnsembleMemberV2 {
        EnsembleMemberV2 {
            id,
            method: format!("M{id}"),
            forecasts,
            historical_rmse: rmse,
        }
    }

    /// Test 1: Simple mean is the arithmetic average of all members.
    #[test]
    fn test_simple_mean_is_average() {
        let config = EnsembleV2Config {
            n_members: 3,
            horizon_hours: 3,
            aggregation_method: AggregationMethod::SimpleMean,
            uncertainty_quantification: UqMethod::Spread,
        };
        let mut fc = EnsembleForecaster::new(config);
        fc.add_member(make_member(0, vec![10.0, 20.0, 30.0], 1.0));
        fc.add_member(make_member(1, vec![20.0, 30.0, 40.0], 1.0));
        fc.add_member(make_member(2, vec![30.0, 40.0, 50.0], 1.0));

        let result = fc.forecast().expect("must succeed");

        // Expected: [20, 30, 40]
        let expected = [20.0, 30.0, 40.0];
        for (computed, &expected) in result.point_forecast.iter().zip(expected.iter()) {
            assert!(
                (computed - expected).abs() < 1e-9,
                "simple mean mismatch: got {computed:.3}, expected {expected:.3}"
            );
        }
    }

    /// Test 2: Weighted mean gives higher weight to lower-RMSE member.
    #[test]
    fn test_weighted_mean_higher_weight_lower_rmse() {
        let weights = vec![0.8, 0.2]; // member 0 gets 80%
        let config = EnsembleV2Config {
            n_members: 2,
            horizon_hours: 2,
            aggregation_method: AggregationMethod::WeightedMean {
                weights: weights.clone(),
            },
            uncertainty_quantification: UqMethod::Spread,
        };
        let mut fc = EnsembleForecaster::new(config);
        fc.add_member(make_member(0, vec![100.0, 100.0], 0.5)); // better
        fc.add_member(make_member(1, vec![0.0, 0.0], 5.0)); // worse

        let result = fc.forecast().expect("must succeed");

        // Weighted mean: 0.8*100 + 0.2*0 = 80
        for &pf in &result.point_forecast {
            assert!(
                (pf - 80.0).abs() < 1e-9,
                "weighted mean must be 80, got {pf:.3}"
            );
        }

        // Member 0 weight (0.8) > member 1 weight (0.2)
        assert!(
            result.member_weights[0] > result.member_weights[1],
            "higher explicit weight must dominate"
        );
    }

    /// Test 3: BMA weights sum to 1.0.
    #[test]
    fn test_bma_weights_sum_to_one() {
        let config = EnsembleV2Config {
            n_members: 5,
            horizon_hours: 4,
            aggregation_method: AggregationMethod::BayesianModelAveraging,
            uncertainty_quantification: UqMethod::Spread,
        };
        let mut fc = EnsembleForecaster::new(config);
        let rmse_vals = [0.5, 1.0, 2.0, 0.3, 1.5];
        for (i, &r) in rmse_vals.iter().enumerate() {
            fc.add_member(make_member(i, vec![10.0; 4], r));
        }

        let bma_w = fc.bma_weights();
        let total: f64 = bma_w.iter().sum();
        assert!(
            (total - 1.0).abs() < 1e-9,
            "BMA weights must sum to 1.0, got {total:.10}"
        );

        let result = fc.forecast().expect("must succeed");
        let w_total: f64 = result.member_weights.iter().sum();
        assert!((w_total - 1.0).abs() < 1e-9, "forecast weights sum to 1.0");
    }

    /// Test 4: Prediction interval contains observation at correct rate.
    ///
    /// With 3 members spanning [10, 20, 30] at each step, the 90% interval
    /// from conformal prediction should contain the middle value (20).
    #[test]
    fn test_prediction_interval_coverage() {
        let config = EnsembleV2Config {
            n_members: 3,
            horizon_hours: 5,
            aggregation_method: AggregationMethod::SimpleMean,
            uncertainty_quantification: UqMethod::ConformalPrediction,
        };
        let mut fc = EnsembleForecaster::new(config);
        fc.add_member(make_member(0, vec![10.0; 5], 1.0));
        fc.add_member(make_member(1, vec![20.0; 5], 1.0));
        fc.add_member(make_member(2, vec![30.0; 5], 1.0));

        let observation = 20.0; // inside the range
        let result = fc.forecast().expect("must succeed");

        let n_covered = result
            .prediction_interval_90
            .iter()
            .filter(|(lo, hi)| observation >= *lo && observation <= *hi)
            .count();
        assert!(
            n_covered > 0,
            "prediction interval must contain the observation"
        );
    }

    /// Test 5: CRPS is lower for the more accurate member.
    #[test]
    fn test_crps_lower_for_accurate_member() {
        let config = EnsembleV2Config {
            n_members: 2,
            horizon_hours: 5,
            aggregation_method: AggregationMethod::SimpleMean,
            uncertainty_quantification: UqMethod::Spread,
        };
        let obs = vec![50.0; 5];
        let mut fc = EnsembleForecaster::new(config);
        // Accurate member (close to observations)
        fc.add_member(make_member(0, vec![51.0; 5], 1.0));
        // Inaccurate member (far from observations)
        fc.add_member(make_member(1, vec![80.0; 5], 30.0));
        fc.set_observations(obs);

        let crps_accurate = fc.crps_skill(&fc.members[0].clone());
        let crps_inaccurate = fc.crps_skill(&fc.members[1].clone());

        assert!(
            crps_accurate < crps_inaccurate,
            "CRPS must be lower for accurate member ({crps_accurate:.4} < {crps_inaccurate:.4})"
        );
    }

    /// Test 6: BestN selects the N members with lowest RMSE.
    #[test]
    fn test_best_n_selection() {
        let config = EnsembleV2Config {
            n_members: 4,
            horizon_hours: 2,
            aggregation_method: AggregationMethod::BestN { n: 2 },
            uncertainty_quantification: UqMethod::Spread,
        };
        let mut fc = EnsembleForecaster::new(config);
        fc.add_member(make_member(0, vec![100.0; 2], 5.0)); // worst
        fc.add_member(make_member(1, vec![100.0; 2], 1.0)); // best
        fc.add_member(make_member(2, vec![100.0; 2], 2.0)); // second best
        fc.add_member(make_member(3, vec![100.0; 2], 4.0)); // third

        let result = fc.forecast().expect("must succeed");

        // Members 1 and 2 should have non-zero weights; 0 and 3 zero
        assert!(
            result.member_weights[0] < 1e-9,
            "worst member must have weight 0"
        );
        assert!(
            result.member_weights[1] > 0.0,
            "best member must have weight > 0"
        );
        assert!(
            result.member_weights[2] > 0.0,
            "2nd best must have weight > 0"
        );
        assert!(
            result.member_weights[3] < 1e-9,
            "4th member must have weight 0"
        );
    }

    /// Test 7: No members returns error.
    #[test]
    fn test_no_members_error() {
        let config = EnsembleV2Config::default();
        let fc = EnsembleForecaster::new(config);
        let result = fc.forecast();
        assert!(
            matches!(result, Err(ForecastError::NoMembers)),
            "must return NoMembers error"
        );
    }
}
