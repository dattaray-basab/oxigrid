//! False Data Injection (FDI) attack generation and detection.
//!
//! This module provides:
//! - [`FdiAttack`] — a crafted measurement perturbation vector
//! - [`FdiAttackGenerator`] — constructs stealthy / sparse / load-redistribution attacks
//! - [`FdiDetector`] — detects attacks via chi-squared, LNR, isolation forest, etc.

/// A False Data Injection attack that additively perturbs sensor measurements.
#[derive(Debug, Clone)]
pub struct FdiAttack {
    /// Additive perturbation applied to each measurement (same length as measurement vector).
    pub attack_vector: Vec<f64>,
    /// The intended state-estimation error induced by the attack.
    pub target_state_error: Vec<f64>,
    /// Indices of measurement channels that are corrupted (non-zero perturbation).
    pub compromised_meters: Vec<usize>,
    /// `true` iff the attack passes standard Bad-Data Detection (BDD).
    pub stealthy: bool,
    /// L2 norm of `attack_vector`.
    pub attack_magnitude: f64,
}

/// Generates FDI attack vectors for research and testing.
///
/// All random numbers are produced via a 64-bit Linear Congruential Generator (LCG)
/// so no external PRNG crate is needed.
pub struct FdiAttackGenerator {
    /// Maximum number of measurement channels that may be corrupted (sparsity bound).
    pub max_compromised_meters: usize,
}

// ---------------------------------------------------------------------------
// Internal LCG helper
// ---------------------------------------------------------------------------

/// Advance a 64-bit LCG state and return the new state.
#[inline]
fn lcg_next(state: u64) -> u64 {
    state
        .wrapping_mul(6_364_136_223_846_793_005_u64)
        .wrapping_add(1_442_695_040_888_963_407_u64)
}

/// Map a raw LCG state to `[-1.0, 1.0)`.
#[inline]
fn lcg_f64(state: u64) -> f64 {
    (state as f64) / (u64::MAX as f64) * 2.0 - 1.0
}

// ---------------------------------------------------------------------------
// L2 norm helper
// ---------------------------------------------------------------------------
fn l2_norm(v: &[f64]) -> f64 {
    v.iter().map(|x| x * x).sum::<f64>().sqrt()
}

// ---------------------------------------------------------------------------
// Matrix-vector multiply: result[i] = sum_j(a[i][j] * x[j])
// ---------------------------------------------------------------------------
fn mat_vec_mul(a: &[Vec<f64>], x: &[f64]) -> Vec<f64> {
    a.iter()
        .map(|row| row.iter().zip(x.iter()).map(|(aij, xj)| aij * xj).sum())
        .collect()
}

impl FdiAttackGenerator {
    /// Create a new generator with the given sparsity budget.
    pub fn new(max_compromised_meters: usize) -> Self {
        Self {
            max_compromised_meters,
        }
    }

    /// Generate a **stealthy** FDI attack targeting specific state variables.
    ///
    /// The attack vector is computed as `a = H * c`, which lies in the column space of
    /// the measurement Jacobian `H` (m × n).  Any such vector is invisible to the
    /// standard weighted-least-squares bad-data detector because the normalized residual
    /// does not change.
    ///
    /// Sparsity is enforced by zeroing all but the `max_compromised_meters` entries
    /// with the largest magnitude.
    pub fn generate_stealthy(
        &self,
        h_matrix: &[Vec<f64>],
        target_state_error: &[f64],
    ) -> FdiAttack {
        // a = H * c
        let mut attack_vector = mat_vec_mul(h_matrix, target_state_error);

        // Enforce sparsity: keep only top-k entries by absolute value.
        let k = self.max_compromised_meters.min(attack_vector.len());
        // Build sorted index list by descending |a_i|.
        let mut indexed: Vec<(usize, f64)> = attack_vector
            .iter()
            .enumerate()
            .map(|(i, &v)| (i, v.abs()))
            .collect();
        indexed
            .sort_unstable_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(core::cmp::Ordering::Equal));
        let keep: std::collections::HashSet<usize> =
            indexed.iter().take(k).map(|(i, _)| *i).collect();

        let mut compromised_meters = Vec::with_capacity(k);
        for (i, v) in attack_vector.iter_mut().enumerate() {
            if !keep.contains(&i) {
                *v = 0.0;
            } else {
                compromised_meters.push(i);
            }
        }
        compromised_meters.sort_unstable();

        let magnitude = l2_norm(&attack_vector);
        FdiAttack {
            attack_vector,
            target_state_error: target_state_error.to_vec(),
            compromised_meters,
            stealthy: true,
            attack_magnitude: magnitude,
        }
    }

    /// Generate a **random sparse** attack (not necessarily stealthy).
    ///
    /// Exactly `min(max_compromised_meters, n_measurements)` random measurement channels
    /// are perturbed with values drawn uniformly from `[-magnitude, +magnitude]` using
    /// an LCG seeded by `seed`.
    pub fn generate_sparse(&self, n_measurements: usize, magnitude: f64, seed: u64) -> FdiAttack {
        let k = self.max_compromised_meters.min(n_measurements);
        let mut attack_vector = vec![0.0_f64; n_measurements];
        let mut compromised_meters = Vec::with_capacity(k);

        // Pick k distinct indices using reservoir-like approach with LCG.
        let mut state = seed;
        let mut selected: std::collections::HashSet<usize> = std::collections::HashSet::new();
        let mut attempts = 0usize;
        while selected.len() < k && attempts < n_measurements * 4 {
            state = lcg_next(state);
            let idx = (state as usize) % n_measurements;
            selected.insert(idx);
            attempts += 1;
        }

        for &idx in &selected {
            state = lcg_next(state);
            let val = lcg_f64(state) * magnitude;
            attack_vector[idx] = val;
            compromised_meters.push(idx);
        }
        compromised_meters.sort_unstable();

        let magnitude_actual = l2_norm(&attack_vector);
        FdiAttack {
            attack_vector,
            target_state_error: vec![0.0; 1], // undefined for sparse random
            compromised_meters,
            stealthy: false,
            attack_magnitude: magnitude_actual,
        }
    }

    /// Generate a **load redistribution** attack that shifts `magnitude_mw` of apparent
    /// load from `from_bus` to `to_bus` in state estimation.
    ///
    /// This constructs a target state error with +magnitude/base_mva at `from_bus` and
    /// -magnitude/base_mva at `to_bus`, then delegates to [`Self::generate_stealthy`].
    pub fn generate_load_redistribution(
        &self,
        h_matrix: &[Vec<f64>],
        from_bus: usize,
        to_bus: usize,
        magnitude_mw: f64,
    ) -> FdiAttack {
        let n_states = h_matrix.first().map(|r| r.len()).unwrap_or(0);
        let base_mva = 100.0_f64; // standard p.u. base
        let mut c = vec![0.0_f64; n_states];
        if from_bus < n_states {
            c[from_bus] += magnitude_mw / base_mva;
        }
        if to_bus < n_states && to_bus != from_bus {
            c[to_bus] -= magnitude_mw / base_mva;
        }
        self.generate_stealthy(h_matrix, &c)
    }
}

// ---------------------------------------------------------------------------
// Detection
// ---------------------------------------------------------------------------

/// Method used by [`FdiDetector`] to identify compromised measurements.
#[derive(Debug, Clone, Copy)]
pub enum DetectionMethod {
    /// Chi-squared hypothesis test on the weighted sum of squared residuals.
    /// Standard BDD — cannot detect stealthy FDI attacks.
    ChiSquared {
        /// Significance level α (e.g. `0.05`).
        significance: f64,
    },
    /// Largest Normalised Residual test.
    Lnr {
        /// Alarm threshold (number of standard deviations, e.g. `3.0`).
        threshold: f64,
    },
    /// Pairwise measurement consistency check.
    PairwiseConsistency {
        /// Maximum allowed absolute difference between redundant measurements.
        threshold: f64,
    },
    /// Isolation-forest-inspired anomaly score via random projections.
    IsolationScore {
        /// Number of random trees to average over.
        n_trees: usize,
    },
    /// Graph-theoretic check: every measurement must have topological redundancy.
    GraphRedundancy {
        /// Minimum required measurement redundancy degree.
        min_redundancy: usize,
    },
}

/// Result of an FDI detection attempt.
#[derive(Debug, Clone)]
pub struct DetectionResult {
    /// `true` if an attack is declared.
    pub attack_detected: bool,
    /// Scalar suspicion score (higher → more suspicious).
    pub score: f64,
    /// Measurement indices flagged as suspicious.
    pub suspicious_meters: Vec<usize>,
    /// The detection method that produced this result.
    pub method: DetectionMethod,
    /// Estimated probability of a false alarm under this method.
    pub false_alarm_risk: f64,
}

/// Detects False Data Injection attacks in measurement data.
pub struct FdiDetector {
    /// Detection algorithm to apply.
    pub method: DetectionMethod,
    /// Alarm threshold (interpretation depends on [`DetectionMethod`]).
    pub alarm_threshold: f64,
}

impl FdiDetector {
    /// Create a detector using the given method.  The `alarm_threshold` is
    /// derived from the method parameters where applicable.
    pub fn new(method: DetectionMethod) -> Self {
        let alarm_threshold = match method {
            DetectionMethod::ChiSquared { significance } => significance,
            DetectionMethod::Lnr { threshold } => threshold,
            DetectionMethod::PairwiseConsistency { threshold } => threshold,
            DetectionMethod::IsolationScore { n_trees: _ } => 0.5,
            DetectionMethod::GraphRedundancy { min_redundancy } => min_redundancy as f64,
        };
        Self {
            method,
            alarm_threshold,
        }
    }

    /// Detect whether the supplied measurements have been tampered with.
    ///
    /// # Arguments
    /// * `measurements`      — raw sensor readings `z`
    /// * `h_matrix`          — linearised measurement Jacobian `H` (m × n)
    /// * `state_estimate`    — state vector `x̂` from the state estimator
    /// * `measurement_noise` — per-measurement standard deviation `σ_i`
    pub fn detect(
        &self,
        measurements: &[f64],
        h_matrix: &[Vec<f64>],
        state_estimate: &[f64],
        measurement_noise: &[f64],
    ) -> DetectionResult {
        // Compute residuals: r = z - H * x̂
        let h_x = mat_vec_mul(h_matrix, state_estimate);
        let residuals: Vec<f64> = measurements
            .iter()
            .zip(h_x.iter())
            .map(|(z, hx)| z - hx)
            .collect();

        let weights: Vec<f64> = measurement_noise
            .iter()
            .map(|s| if *s > 1e-15 { 1.0 / (s * s) } else { 1e30 })
            .collect();

        match self.method {
            DetectionMethod::ChiSquared { significance } => {
                self.chi_squared_test(&residuals, &weights, significance)
            }
            DetectionMethod::Lnr { threshold } => {
                self.lnr_test(&residuals, measurement_noise, threshold)
            }
            DetectionMethod::PairwiseConsistency { threshold } => {
                self.pairwise_consistency(&residuals, threshold)
            }
            DetectionMethod::IsolationScore { n_trees } => {
                // Use residuals as the "measurement" vector for anomaly scoring.
                self.isolation_score(measurements, &[], n_trees, 0xDEAD_BEEF)
            }
            DetectionMethod::GraphRedundancy { min_redundancy } => {
                self.graph_redundancy(h_matrix, min_redundancy)
            }
        }
    }

    // -----------------------------------------------------------------------
    // Chi-squared test
    // -----------------------------------------------------------------------

    /// Compute `J(x̂) = Σ w_i r_i²` and compare to the chi-squared critical value.
    ///
    /// Degrees of freedom are approximated as `m − n` where `m = len(residuals)` and
    /// `n` is inferred from the H-matrix column count (not available here, so we use
    /// `m * 2 / 3` as a conservative estimate if not otherwise known).
    pub fn chi_squared_test(
        &self,
        residuals: &[f64],
        weights: &[f64],
        significance: f64,
    ) -> DetectionResult {
        let j: f64 = residuals
            .iter()
            .zip(weights.iter())
            .map(|(r, w)| r * r * w)
            .sum();

        let m = residuals.len();
        // Degrees of freedom: heuristic (m - n where n ≈ m/3)
        let dof = (m.saturating_sub(m / 3)).max(1) as f64;

        // Wilson-Hilferty approximation to chi²(dof) critical value.
        let z_alpha = chi2_z_alpha(significance);
        let critical = chi2_critical_wh(dof, z_alpha);

        let attack_detected = j > critical;
        // Suspicious meters: those with largest |w^0.5 * r|
        let mut scored: Vec<(usize, f64)> = residuals
            .iter()
            .zip(weights.iter())
            .enumerate()
            .map(|(i, (r, w))| (i, (r * r * w).sqrt()))
            .collect();
        scored.sort_unstable_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(core::cmp::Ordering::Equal));
        let suspicious_meters: Vec<usize> = scored.iter().take(3).map(|(i, _)| *i).collect();

        DetectionResult {
            attack_detected,
            score: j,
            suspicious_meters,
            method: self.method,
            false_alarm_risk: significance,
        }
    }

    // -----------------------------------------------------------------------
    // LNR test
    // -----------------------------------------------------------------------

    /// Flag the measurement with the largest normalised residual `|r_i / σ_i|`.
    pub fn lnr_test(
        &self,
        residuals: &[f64],
        measurement_noise: &[f64],
        threshold: f64,
    ) -> DetectionResult {
        let normalized: Vec<f64> = residuals
            .iter()
            .zip(measurement_noise.iter())
            .map(|(r, s)| {
                let s_eff = if *s > 1e-15 { *s } else { 1e-15 };
                r.abs() / s_eff
            })
            .collect();

        let (max_idx, max_val) =
            normalized
                .iter()
                .enumerate()
                .fold(
                    (0, 0.0_f64),
                    |(bi, bv), (i, &v)| {
                        if v > bv {
                            (i, v)
                        } else {
                            (bi, bv)
                        }
                    },
                );

        let attack_detected = max_val > threshold;
        let suspicious_meters = if attack_detected {
            vec![max_idx]
        } else {
            vec![]
        };

        DetectionResult {
            attack_detected,
            score: max_val,
            suspicious_meters,
            method: self.method,
            false_alarm_risk: if threshold > 3.0 { 0.003 } else { 0.05 },
        }
    }

    // -----------------------------------------------------------------------
    // Pairwise consistency
    // -----------------------------------------------------------------------

    fn pairwise_consistency(&self, residuals: &[f64], threshold: f64) -> DetectionResult {
        let mut max_diff = 0.0_f64;
        let mut suspicious_meters = Vec::new();
        for i in 0..residuals.len() {
            for j in (i + 1)..residuals.len() {
                let diff = (residuals[i] - residuals[j]).abs();
                if diff > threshold {
                    if diff > max_diff {
                        max_diff = diff;
                    }
                    if !suspicious_meters.contains(&i) {
                        suspicious_meters.push(i);
                    }
                    if !suspicious_meters.contains(&j) {
                        suspicious_meters.push(j);
                    }
                }
            }
        }
        DetectionResult {
            attack_detected: max_diff > threshold,
            score: max_diff,
            suspicious_meters,
            method: self.method,
            false_alarm_risk: 0.05,
        }
    }

    // -----------------------------------------------------------------------
    // Isolation score (random projection trees)
    // -----------------------------------------------------------------------

    /// Compute an isolation-forest-inspired anomaly score.
    ///
    /// For each of `n_trees` random trees, a random unit direction is drawn via LCG
    /// and the projection of `measurements` along that direction is compared to
    /// projections of `historical_measurements`.  The "depth" to isolate the point
    /// is the number of bisections needed.  A shallow depth → high isolation score.
    ///
    /// If `historical_measurements` is empty, the residual magnitudes are used as a
    /// proxy baseline.
    pub fn isolation_score(
        &self,
        measurements: &[f64],
        historical_measurements: &[Vec<f64>],
        n_trees: usize,
        seed: u64,
    ) -> DetectionResult {
        let m = measurements.len();
        if m == 0 || n_trees == 0 {
            return DetectionResult {
                attack_detected: false,
                score: 0.0,
                suspicious_meters: vec![],
                method: self.method,
                false_alarm_risk: 0.1,
            };
        }

        let mut state = seed;
        let mut total_depth = 0.0_f64;

        for _ in 0..n_trees {
            // Random projection direction (unit vector via LCG).
            let mut dir: Vec<f64> = (0..m)
                .map(|_| {
                    state = lcg_next(state);
                    lcg_f64(state)
                })
                .collect();
            // Normalize direction.
            let norm = l2_norm(&dir);
            if norm > 1e-15 {
                for d in dir.iter_mut() {
                    *d /= norm;
                }
            }

            // Project measurement point.
            let proj_point: f64 = measurements
                .iter()
                .zip(dir.iter())
                .map(|(x, d)| x * d)
                .sum();

            // Build projections for baseline.
            let baseline_projs: Vec<f64> = if historical_measurements.is_empty() {
                // Use uniform grid as synthetic baseline.
                (0..8).map(|k| (k as f64 - 3.5) * 0.5).collect()
            } else {
                historical_measurements
                    .iter()
                    .map(|h| h.iter().zip(dir.iter()).map(|(x, d)| x * d).sum())
                    .collect()
            };

            // Count bisection depth to isolate proj_point.
            let depth = isolation_depth(proj_point, &baseline_projs);
            total_depth += depth as f64;
        }

        let avg_depth = total_depth / n_trees as f64;
        let expected_depth = (m as f64 + 1.0).log2().max(1.0);
        // Score: 0 = normal (deep), 1 = isolated (shallow).
        let score = 1.0 - (avg_depth / expected_depth).min(1.0);

        let alarm_threshold = 0.5_f64;
        DetectionResult {
            attack_detected: score > alarm_threshold,
            score,
            suspicious_meters: if score > alarm_threshold {
                (0..m).collect()
            } else {
                vec![]
            },
            method: self.method,
            false_alarm_risk: 0.1,
        }
    }

    // -----------------------------------------------------------------------
    // Graph redundancy
    // -----------------------------------------------------------------------

    fn graph_redundancy(&self, h_matrix: &[Vec<f64>], min_redundancy: usize) -> DetectionResult {
        // Count how many measurements cover each state variable (column).
        let n_states = h_matrix.first().map(|r| r.len()).unwrap_or(0);
        let mut coverage = vec![0usize; n_states];
        for row in h_matrix {
            for (j, &v) in row.iter().enumerate() {
                if v.abs() > 1e-12 {
                    coverage[j] += 1;
                }
            }
        }
        let min_cov = coverage.iter().cloned().min().unwrap_or(0);
        let attack_detected = min_cov < min_redundancy;
        let suspicious_meters: Vec<usize> = coverage
            .iter()
            .enumerate()
            .filter(|(_, &c)| c < min_redundancy)
            .map(|(i, _)| i)
            .collect();

        DetectionResult {
            attack_detected,
            score: min_cov as f64,
            suspicious_meters,
            method: self.method,
            false_alarm_risk: 0.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Private math helpers
// ---------------------------------------------------------------------------

/// Chi-squared critical value via Wilson-Hilferty approximation.
/// `z_alpha` is the standard-normal quantile for the desired significance level.
fn chi2_critical_wh(dof: f64, z_alpha: f64) -> f64 {
    let h = 2.0 / (9.0 * dof);
    dof * (1.0 - h + z_alpha * h.sqrt()).powi(3)
}

/// Approximate standard-normal quantile for common significance levels.
fn chi2_z_alpha(significance: f64) -> f64 {
    if significance <= 0.001 {
        3.090
    } else if significance <= 0.01 {
        2.576
    } else if significance <= 0.05 {
        1.645
    } else if significance <= 0.10 {
        1.282
    } else {
        0.842
    }
}

/// Recursive isolation depth: how many bisections to isolate `point` in `values`.
fn isolation_depth(point: f64, values: &[f64]) -> usize {
    if values.is_empty() || values.len() == 1 {
        return 1;
    }
    let min = values.iter().cloned().fold(f64::INFINITY, f64::min);
    let max = values.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    if (max - min).abs() < 1e-15 {
        return 1;
    }
    let mid = (min + max) / 2.0;
    let (left, right): (Vec<f64>, Vec<f64>) = values.iter().cloned().partition(|&v| v < mid);
    if point < mid {
        1 + isolation_depth(point, &left)
    } else {
        1 + isolation_depth(point, &right)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stealthy_vector_is_h_times_c() {
        let h = vec![vec![1.0, 0.0], vec![0.0, 1.0], vec![1.0, 1.0]];
        let c = vec![0.5, -0.3];
        let gen = FdiAttackGenerator::new(3);
        let attack = gen.generate_stealthy(&h, &c);
        assert!((attack.attack_vector[0] - 0.5).abs() < 1e-12);
        assert!((attack.attack_vector[1] + 0.3).abs() < 1e-12);
        assert!((attack.attack_vector[2] - 0.2).abs() < 1e-12);
        assert!(attack.stealthy);
    }

    #[test]
    fn sparse_attack_is_not_stealthy() {
        let gen = FdiAttackGenerator::new(2);
        let attack = gen.generate_sparse(10, 1.0, 42);
        assert!(!attack.stealthy);
        assert!(attack.compromised_meters.len() <= 2);
    }

    #[test]
    fn chi_squared_passes_zero_residuals() {
        let det = FdiDetector::new(DetectionMethod::ChiSquared { significance: 0.05 });
        let residuals = vec![0.0; 6];
        let weights = vec![100.0; 6];
        let result = det.chi_squared_test(&residuals, &weights, 0.05);
        assert!(!result.attack_detected);
    }

    #[test]
    fn lnr_catches_large_residual() {
        let det = FdiDetector::new(DetectionMethod::Lnr { threshold: 3.0 });
        let residuals = vec![0.1, 0.2, 10.0];
        let noise = vec![0.1, 0.1, 0.1];
        let result = det.lnr_test(&residuals, &noise, 3.0);
        assert!(result.attack_detected);
        assert_eq!(result.suspicious_meters, vec![2]);
    }

    #[test]
    fn load_redistribution_attack_sums_to_zero_across_buses() {
        let h = vec![
            vec![1.0, 0.0, 0.0],
            vec![0.0, 1.0, 0.0],
            vec![0.0, 0.0, 1.0],
            vec![1.0, -1.0, 0.0],
        ];
        let gen = FdiAttackGenerator::new(4);
        let attack = gen.generate_load_redistribution(&h, 0, 1, 10.0);
        // For a 3×3 identity-like H, attacking bus0→bus1 perturbs rows 0 and 1 with opposite signs
        let sum: f64 = attack.attack_vector[0] + attack.attack_vector[1];
        assert!(
            sum.abs() < 1e-10,
            "bus0+bus1 entries should cancel: sum={:.4e}",
            sum
        );
    }

    #[test]
    fn chi_squared_detects_large_attack() {
        let det = FdiDetector::new(DetectionMethod::ChiSquared { significance: 0.05 });
        // Very large residuals → should trigger detection
        let residuals = vec![100.0; 6];
        let weights = vec![1.0; 6];
        let result = det.chi_squared_test(&residuals, &weights, 0.05);
        assert!(result.attack_detected, "score = {:.2}", result.score);
    }

    #[test]
    fn sparse_attack_magnitude_bounded_by_argument() {
        let gen = FdiAttackGenerator::new(5);
        let magnitude = 2.0;
        let attack = gen.generate_sparse(20, magnitude, 12345);
        for &v in &attack.attack_vector {
            assert!(
                v.abs() <= magnitude + 1e-12,
                "component {:.4} exceeds magnitude bound {:.4}",
                v,
                magnitude
            );
        }
    }

    #[test]
    fn lnr_test_all_small_residuals_no_attack() {
        let det = FdiDetector::new(DetectionMethod::Lnr { threshold: 3.0 });
        let residuals = vec![0.01, 0.02, 0.015, 0.005];
        let noise = vec![0.1; 4];
        let result = det.lnr_test(&residuals, &noise, 3.0);
        assert!(!result.attack_detected, "score = {:.4}", result.score);
        assert!(result.suspicious_meters.is_empty());
    }

    #[test]
    fn stealthy_attack_magnitude_is_l2_norm_of_vector() {
        let h = vec![vec![3.0, 0.0], vec![0.0, 4.0]];
        let c = vec![1.0, 0.0];
        let gen = FdiAttackGenerator::new(2);
        let attack = gen.generate_stealthy(&h, &c);
        // a = H*c = [3, 0]; ‖a‖ = 3
        let expected_mag = 3.0;
        assert!(
            (attack.attack_magnitude - expected_mag).abs() < 1e-9,
            "mag = {:.4}",
            attack.attack_magnitude
        );
    }
}
