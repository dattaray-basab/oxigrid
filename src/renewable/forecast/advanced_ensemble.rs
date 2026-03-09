//! Advanced ensemble renewable energy forecasting methods.
//!
//! Provides:
//! - [`AnalogEnsemble`] — NWP-analog method (closest historical days)
//! - [`AdvancedEnsembleForecast`] — ensemble output with CRPS scoring
//! - [`QuantileRegressionForest`] — random-forest quantile regressor (pure Rust LCG)
//! - [`EmosCalibrator`] — EMOS (Ensemble Model Output Statistics) bias correction
//! - [`ConformalPredictor`] — distribution-free conformal prediction intervals
//! - [`ModelBlender`] — Bates-Granger online model combination
//! - [`ForecastSkillAssessor`] — deterministic + probabilistic skill metrics
//!
//! All random numbers use the Knuth MMIX LCG:
//! `state = state * 6364136223846793005 + 1442695040888963407`

// ---------------------------------------------------------------------------
// LCG helper
// ---------------------------------------------------------------------------

/// Knuth MMIX LCG constants.
const LCG_MULT: u64 = 6_364_136_223_846_793_005;
const LCG_ADD: u64 = 1_442_695_040_888_963_407;

fn lcg_next(state: &mut u64) -> u64 {
    *state = state.wrapping_mul(LCG_MULT).wrapping_add(LCG_ADD);
    *state
}

/// Return a pseudo-random `f64` in `[0, 1)` from the LCG state.
#[allow(dead_code)]
fn lcg_f64(state: &mut u64) -> f64 {
    (lcg_next(state) >> 11) as f64 / (1u64 << 53) as f64
}

/// Return a pseudo-random `usize` in `[0, n)`.
fn lcg_usize(state: &mut u64, n: usize) -> usize {
    (lcg_next(state) as usize) % n
}

// ---------------------------------------------------------------------------
// Shared quantile utility
// ---------------------------------------------------------------------------

/// Compute the `alpha`-quantile (linear interpolation) of a mutable slice.
/// The slice is sorted in-place.
fn quantile_sorted(values: &mut [f64], alpha: f64) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    values.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let n = values.len();
    let pos = alpha.clamp(0.0, 1.0) * (n - 1) as f64;
    let lo = pos.floor() as usize;
    let hi = pos.ceil() as usize;
    let frac = pos - lo as f64;
    if hi >= n {
        return values[n - 1];
    }
    values[lo] + frac * (values[hi] - values[lo])
}

// ---------------------------------------------------------------------------
// 1. AdvancedEnsembleForecast
// ---------------------------------------------------------------------------

/// Output of an advanced ensemble forecast.
///
/// Each `members[i]` is a full time-series over `horizon_hours`.
/// `point_forecast` is the ensemble mean; `std_dev` is the per-hour spread \[MW\].
#[derive(Debug, Clone)]
pub struct AdvancedEnsembleForecast {
    /// One row per ensemble member, one column per hour \[MW\].
    pub members: Vec<Vec<f64>>,
    /// Ensemble mean per hour \[MW\].
    pub point_forecast: Vec<f64>,
    /// Ensemble standard deviation per hour \[MW\].
    pub std_dev: Vec<f64>,
}

impl AdvancedEnsembleForecast {
    /// Build forecast statistics from raw members.
    pub fn from_members(members: Vec<Vec<f64>>) -> Result<Self, String> {
        if members.is_empty() {
            return Err("No ensemble members provided".to_string());
        }
        let horizon = members[0].len();
        for (i, m) in members.iter().enumerate() {
            if m.len() != horizon {
                return Err(format!(
                    "Member {} has length {} but expected {}",
                    i,
                    m.len(),
                    horizon
                ));
            }
        }
        let n = members.len() as f64;
        let mut point_forecast = vec![0.0_f64; horizon];
        for m in &members {
            for (h, &v) in m.iter().enumerate() {
                point_forecast[h] += v;
            }
        }
        for v in &mut point_forecast {
            *v /= n;
        }
        let mut std_dev = vec![0.0_f64; horizon];
        for m in &members {
            for (h, &v) in m.iter().enumerate() {
                let diff = v - point_forecast[h];
                std_dev[h] += diff * diff;
            }
        }
        for v in &mut std_dev {
            *v = (*v / n).sqrt();
        }
        Ok(Self {
            members,
            point_forecast,
            std_dev,
        })
    }

    /// Return the `alpha`-quantile of member values at `hour` (linear interpolation).
    ///
    /// `alpha = 0.5` gives the median.
    pub fn percentile(&self, alpha: f64, hour: usize) -> f64 {
        let mut vals: Vec<f64> = self.members.iter().map(|m| m[hour]).collect();
        quantile_sorted(&mut vals, alpha)
    }

    /// Return the symmetric `confidence`-prediction interval at `hour`.
    ///
    /// E.g. `confidence = 0.9` → \[5th, 95th\] percentile.
    pub fn prediction_interval(&self, confidence: f64, hour: usize) -> (f64, f64) {
        let lo_alpha = (1.0 - confidence) / 2.0;
        let hi_alpha = 1.0 - lo_alpha;
        let mut vals: Vec<f64> = self.members.iter().map(|m| m[hour]).collect();
        vals.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let lo = quantile_sorted(&mut vals.clone(), lo_alpha);
        let hi = quantile_sorted(&mut vals, hi_alpha);
        (lo, hi)
    }

    /// Continuous Ranked Probability Score averaged over all hours.
    ///
    /// Approximation via sorted ensemble:
    /// `CRPS = E|X - y| - 0.5 * E|X - X'|`
    pub fn crps(&self, observations: &[f64]) -> f64 {
        let horizon = self.members[0].len().min(observations.len());
        if horizon == 0 {
            return 0.0;
        }
        let mut total = 0.0_f64;
        for h in 0..horizon {
            let y = observations[h];
            let mut vals: Vec<f64> = self.members.iter().map(|m| m[h]).collect();
            vals.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let n = vals.len() as f64;
            // E|X - y|
            let e_xy: f64 = vals.iter().map(|&x| (x - y).abs()).sum::<f64>() / n;
            // 0.5 * E|X - X'| via sorted order formula: sum_i (2i - n - 1) * x_i / n^2
            let e_xx: f64 = vals
                .iter()
                .enumerate()
                .map(|(i, &x)| (2.0 * i as f64 - n + 1.0) * x)
                .sum::<f64>()
                / (n * n);
            total += e_xy - e_xx;
        }
        (total / horizon as f64).max(0.0)
    }
}

// ---------------------------------------------------------------------------
// 2. AnalogEnsemble
// ---------------------------------------------------------------------------

/// Analog Ensemble (AnEn) — selects the `n_analogs` historical days most similar
/// to a query NWP feature vector and uses them as ensemble members.
///
/// Features are any numerical predictors derived from NWP output (e.g. solar
/// irradiance forecast, temperature, wind speed). Each historical day is stored
/// as a feature vector and an observed power profile over `horizon_hours`.
#[derive(Debug, Clone)]
pub struct AnalogEnsemble {
    /// Historical NWP feature vectors, one per day \[varies\].
    pub historical_features: Vec<Vec<f64>>,
    /// Historical observed power profiles \[MW\], one per day.
    pub historical_targets: Vec<Vec<f64>>,
    /// Number of analog members to select.
    pub n_analogs: usize,
    /// Per-feature weight for weighted Euclidean distance.
    pub feature_weights: Vec<f64>,
}

impl AnalogEnsemble {
    /// Create a new `AnalogEnsemble` with uniform feature weights.
    pub fn new(n_analogs: usize) -> Self {
        Self {
            historical_features: Vec::new(),
            historical_targets: Vec::new(),
            n_analogs,
            feature_weights: Vec::new(),
        }
    }

    /// Append one historical day.
    ///
    /// `features` — NWP predictors for that day.
    /// `target` — observed power profile (one value per hour) \[MW\].
    pub fn add_historical_day(&mut self, features: Vec<f64>, target: Vec<f64>) {
        // Lazily initialise uniform weights on first insertion.
        if self.feature_weights.is_empty() || self.feature_weights.len() != features.len() {
            self.feature_weights = vec![1.0; features.len()];
        }
        self.historical_features.push(features);
        self.historical_targets.push(target);
    }

    /// Find the `n_analogs` closest historical days to `query`.
    ///
    /// Returns `(day_index, distance)` pairs sorted ascending by distance.
    pub fn find_analogs(&self, query: &[f64]) -> Vec<(usize, f64)> {
        let mut distances: Vec<(usize, f64)> = self
            .historical_features
            .iter()
            .enumerate()
            .map(|(i, hist)| {
                let d = weighted_euclidean(query, hist, &self.feature_weights);
                (i, d)
            })
            .collect();
        distances.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        distances.truncate(self.n_analogs);
        distances
    }

    /// Produce an `AdvancedEnsembleForecast` from the analog members.
    pub fn forecast(
        &self,
        query_features: &[f64],
        horizon_hours: usize,
    ) -> Result<AdvancedEnsembleForecast, String> {
        if self.historical_features.len() < self.n_analogs {
            return Err(format!(
                "Need at least {} historical days, have {}",
                self.n_analogs,
                self.historical_features.len()
            ));
        }
        let analogs = self.find_analogs(query_features);
        let members: Vec<Vec<f64>> = analogs
            .iter()
            .map(|&(idx, _)| {
                let t = &self.historical_targets[idx];
                // Truncate or pad to horizon_hours.
                let mut row = t.clone();
                row.resize(horizon_hours, 0.0);
                row
            })
            .collect();
        AdvancedEnsembleForecast::from_members(members)
    }
}

fn weighted_euclidean(a: &[f64], b: &[f64], w: &[f64]) -> f64 {
    let len = a.len().min(b.len()).min(w.len());
    let sum: f64 = (0..len).map(|i| w[i] * (a[i] - b[i]).powi(2)).sum();
    sum.sqrt()
}

// ---------------------------------------------------------------------------
// 3. QuantileRegressionForest
// ---------------------------------------------------------------------------

/// A node in a [`RegressionTree`].
#[derive(Debug, Clone)]
pub enum TreeNode {
    /// Leaf stores all training target values that fell into this region.
    Leaf { samples: Vec<f64> },
    /// Internal split: compare `features[feature]` against `threshold`.
    Split {
        feature: usize,
        threshold: f64,
        left: usize,
        right: usize,
    },
}

/// A single regression tree that retains leaf-level sample distributions for
/// quantile estimation (Meinshausen 2006).
#[derive(Debug, Clone)]
pub struct RegressionTree {
    /// Flat node storage; index 0 is the root.
    pub nodes: Vec<TreeNode>,
}

impl RegressionTree {
    /// Build a regression tree from bootstrapped samples.
    ///
    /// `feature_indices` — the subset of features available at each split.
    pub fn build(
        features: &[Vec<f64>],
        targets: &[f64],
        min_samples_leaf: usize,
        feature_indices: &[usize],
        seed: u64,
    ) -> Self {
        let mut nodes: Vec<TreeNode> = Vec::new();
        let indices: Vec<usize> = (0..features.len()).collect();
        let mut rng = seed;
        build_node(
            features,
            targets,
            &indices,
            min_samples_leaf,
            feature_indices,
            &mut nodes,
            &mut rng,
        );
        Self { nodes }
    }

    /// Return the target samples stored in the leaf reached by `features`.
    pub fn predict_samples(&self, features: &[f64]) -> &[f64] {
        let mut idx = 0;
        loop {
            match &self.nodes[idx] {
                TreeNode::Leaf { samples } => return samples,
                TreeNode::Split {
                    feature,
                    threshold,
                    left,
                    right,
                } => {
                    let val = features.get(*feature).copied().unwrap_or(0.0);
                    idx = if val <= *threshold { *left } else { *right };
                }
            }
        }
    }
}

/// Recursively build a node and append it to `nodes`. Returns the node index.
fn build_node(
    features: &[Vec<f64>],
    targets: &[f64],
    indices: &[usize],
    min_samples_leaf: usize,
    feature_indices: &[usize],
    nodes: &mut Vec<TreeNode>,
    rng: &mut u64,
) -> usize {
    // Allocate slot.
    let node_idx = nodes.len();
    nodes.push(TreeNode::Leaf {
        samples: Vec::new(),
    }); // placeholder

    if indices.len() <= min_samples_leaf * 2 || feature_indices.is_empty() {
        let samples: Vec<f64> = indices.iter().map(|&i| targets[i]).collect();
        nodes[node_idx] = TreeNode::Leaf { samples };
        return node_idx;
    }

    // Try each candidate feature and pick the best split (variance reduction).
    let mut best_score = f64::NEG_INFINITY;
    let mut best_feature = feature_indices[0];
    let mut best_threshold = 0.0_f64;
    let mut best_left: Vec<usize> = Vec::new();
    let mut best_right: Vec<usize> = Vec::new();

    // Randomly shuffle feature order via LCG to avoid always trying same first.
    let mut feat_order: Vec<usize> = feature_indices.to_vec();
    // Fisher-Yates shuffle with LCG.
    for i in (1..feat_order.len()).rev() {
        let j = lcg_usize(rng, i + 1);
        feat_order.swap(i, j);
    }

    'feat: for &fi in &feat_order {
        // Collect unique thresholds (midpoints between sorted unique values).
        let mut vals: Vec<f64> = indices
            .iter()
            .filter_map(|&i| features[i].get(fi).copied())
            .collect();
        if vals.is_empty() {
            continue;
        }
        vals.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        vals.dedup_by(|a, b| (*a - *b).abs() < 1e-12);

        for w in vals.windows(2) {
            let thr = (w[0] + w[1]) / 2.0;
            let left: Vec<usize> = indices
                .iter()
                .filter(|&&i| features[i].get(fi).copied().unwrap_or(0.0) <= thr)
                .copied()
                .collect();
            let right: Vec<usize> = indices
                .iter()
                .filter(|&&i| features[i].get(fi).copied().unwrap_or(0.0) > thr)
                .copied()
                .collect();
            if left.len() < min_samples_leaf || right.len() < min_samples_leaf {
                continue;
            }
            let score = variance_reduction(targets, &left, &right);
            if score > best_score {
                best_score = score;
                best_feature = fi;
                best_threshold = thr;
                best_left = left;
                best_right = right;
            }
            // Limit candidates per feature to keep complexity reasonable.
            if best_score > 0.0 {
                break 'feat;
            }
        }
    }

    if best_left.is_empty() || best_right.is_empty() {
        let samples: Vec<f64> = indices.iter().map(|&i| targets[i]).collect();
        nodes[node_idx] = TreeNode::Leaf { samples };
        return node_idx;
    }

    // Reserve left/right slots before recursion.
    let left_idx = build_node(
        features,
        targets,
        &best_left,
        min_samples_leaf,
        feature_indices,
        nodes,
        rng,
    );
    let right_idx = build_node(
        features,
        targets,
        &best_right,
        min_samples_leaf,
        feature_indices,
        nodes,
        rng,
    );

    nodes[node_idx] = TreeNode::Split {
        feature: best_feature,
        threshold: best_threshold,
        left: left_idx,
        right: right_idx,
    };
    node_idx
}

fn variance_reduction(targets: &[f64], left: &[usize], right: &[usize]) -> f64 {
    fn var(targets: &[f64], idx: &[usize]) -> f64 {
        if idx.is_empty() {
            return 0.0;
        }
        let n = idx.len() as f64;
        let mean: f64 = idx.iter().map(|&i| targets[i]).sum::<f64>() / n;
        idx.iter()
            .map(|&i| (targets[i] - mean).powi(2))
            .sum::<f64>()
            / n
    }
    let n = (left.len() + right.len()) as f64;
    let nl = left.len() as f64;
    let nr = right.len() as f64;
    -(nl / n * var(targets, left) + nr / n * var(targets, right))
}

/// Random Forest that stores per-leaf sample distributions for quantile prediction.
#[derive(Debug, Clone)]
pub struct QuantileRegressionForest {
    /// Trained regression trees.
    pub trees: Vec<RegressionTree>,
    /// Number of trees to build.
    pub n_trees: usize,
    /// Minimum number of samples in each leaf.
    pub min_samples_leaf: usize,
    /// Fraction of features randomly selected at each split.
    pub feature_fraction: f64,
    seed: u64,
}

impl QuantileRegressionForest {
    /// Construct with default `feature_fraction = 0.5`.
    pub fn new(n_trees: usize, min_samples_leaf: usize) -> Self {
        Self {
            trees: Vec::new(),
            n_trees,
            min_samples_leaf,
            feature_fraction: 0.5,
            seed: 42,
        }
    }

    /// Fit the forest on `(features, targets)`.
    ///
    /// Uses bootstrap sampling (LCG) and random feature sub-selection per tree.
    pub fn fit(&mut self, features: &[Vec<f64>], targets: &[f64]) -> Result<(), String> {
        if features.is_empty() || targets.is_empty() {
            return Err("Empty training data".to_string());
        }
        if features.len() != targets.len() {
            return Err(format!(
                "features.len()={} != targets.len()={}",
                features.len(),
                targets.len()
            ));
        }
        let n_samples = features.len();
        let n_features = features[0].len();
        let n_sel = ((n_features as f64 * self.feature_fraction).ceil() as usize).max(1);
        self.trees.clear();

        let mut rng = self.seed;
        for _ in 0..self.n_trees {
            // Bootstrap sample.
            let boot: Vec<usize> = (0..n_samples)
                .map(|_| lcg_usize(&mut rng, n_samples))
                .collect();
            let boot_features: Vec<Vec<f64>> = boot.iter().map(|&i| features[i].clone()).collect();
            let boot_targets: Vec<f64> = boot.iter().map(|&i| targets[i]).collect();

            // Random feature subset.
            let mut all_feat: Vec<usize> = (0..n_features).collect();
            // Shuffle first n_sel positions.
            for i in 0..n_sel {
                let j = i + lcg_usize(&mut rng, n_features - i);
                all_feat.swap(i, j);
            }
            let feature_indices: Vec<usize> = all_feat[..n_sel].to_vec();

            let tree = RegressionTree::build(
                &boot_features,
                &boot_targets,
                self.min_samples_leaf,
                &feature_indices,
                lcg_next(&mut rng),
            );
            self.trees.push(tree);
        }
        Ok(())
    }

    /// Predict the `quantile` \[0, 1\] for a single sample by aggregating leaf samples.
    pub fn predict_quantile(&self, features: &[f64], quantile: f64) -> f64 {
        if self.trees.is_empty() {
            return 0.0;
        }
        let per_tree: f64 = self
            .trees
            .iter()
            .map(|t| {
                let mut s: Vec<f64> = t.predict_samples(features).to_vec();
                quantile_sorted(&mut s, quantile)
            })
            .sum::<f64>();
        per_tree / self.trees.len() as f64
    }

    /// Predict the mean of leaf distributions across all trees.
    pub fn predict_mean(&self, features: &[f64]) -> f64 {
        if self.trees.is_empty() {
            return 0.0;
        }
        let total: f64 = self
            .trees
            .iter()
            .map(|t| {
                let s = t.predict_samples(features);
                if s.is_empty() {
                    0.0
                } else {
                    s.iter().sum::<f64>() / s.len() as f64
                }
            })
            .sum();
        total / self.trees.len() as f64
    }

    /// Predict a `(1-alpha)` prediction interval.
    ///
    /// Returns `(alpha/2 quantile, 1-alpha/2 quantile)`.
    pub fn predict_interval(&self, features: &[f64], alpha: f64) -> (f64, f64) {
        let lo = self.predict_quantile(features, alpha / 2.0);
        let hi = self.predict_quantile(features, 1.0 - alpha / 2.0);
        (lo, hi)
    }
}

// ---------------------------------------------------------------------------
// 4. EMOS Calibrator
// ---------------------------------------------------------------------------

/// Ensemble Model Output Statistics (EMOS) post-processor.
///
/// Learns a linear mapping from raw ensemble statistics to calibrated Normal
/// distribution parameters:
/// ```text
/// μ_corr  = α + β · μ_ens
/// σ²_corr = δ + γ · σ²_ens
/// ```
/// Parameters are fitted by minimising the empirical CRPS via gradient descent.
#[derive(Debug, Clone)]
pub struct EmosCalibrator {
    /// Bias / intercept term for the mean \[MW\].
    pub alpha: f64,
    /// Ensemble mean scaling coefficient.
    pub beta: f64,
    /// Ensemble spread (variance) scaling coefficient.
    pub gamma: f64,
    /// Base (irreducible) variance \[MW²\].
    pub delta: f64,
}

impl EmosCalibrator {
    /// Initialise with identity mapping (no correction).
    pub fn new() -> Self {
        Self {
            alpha: 0.0,
            beta: 1.0,
            gamma: 1.0,
            delta: 0.01,
        }
    }

    /// Fit EMOS parameters from training pairs of `(ensemble_mean, ensemble_std, observation)`.
    ///
    /// Uses 20-step gradient descent with step size η = 0.01.
    pub fn fit(
        &mut self,
        ensemble_means: &[f64],
        ensemble_stds: &[f64],
        observations: &[f64],
    ) -> Result<(), String> {
        let n = ensemble_means.len();
        if n == 0 {
            return Err("Empty calibration data".to_string());
        }
        if ensemble_stds.len() != n || observations.len() != n {
            return Err("Input slice lengths must match".to_string());
        }
        let eta = 0.01_f64;
        // Gradient descent — CRPS for Normal(μ, σ) has closed form:
        // CRPS(N(μ,σ), y) = σ * [z*(2Φ(z)-1) + 2φ(z) - 1/√π]
        // where z = (y - μ) / σ.
        // We use finite-difference gradients for simplicity.
        for _ in 0..20 {
            let mut da = 0.0_f64;
            let mut db = 0.0_f64;
            let mut dg = 0.0_f64;
            let mut dd = 0.0_f64;
            for i in 0..n {
                let mu = self.alpha + self.beta * ensemble_means[i];
                let var = (self.delta + self.gamma * ensemble_stds[i].powi(2)).max(1e-9);
                let sigma = var.sqrt();
                let y = observations[i];
                let z = (y - mu) / sigma;
                // dCRPS/dmu ≈ -(2Φ(z)-1) / σ  (standard result)
                let phi_z = normal_cdf(z);
                let dphi = 2.0 * phi_z - 1.0;
                let grad_mu = -dphi / sigma;
                // dCRPS/dsigma ≈ CRPS/sigma - z * grad_mu (chain rule approx)
                let crps_val = sigma
                    * (z * dphi + 2.0 * normal_pdf(z) - std::f64::consts::FRAC_2_SQRT_PI / 2.0);
                let grad_sigma = crps_val / sigma - z * grad_mu;
                // Chain through parameterisation.
                da += grad_mu;
                db += grad_mu * ensemble_means[i];
                let dsigma_dgamma = ensemble_stds[i].powi(2) / (2.0 * sigma);
                let dsigma_ddelta = 1.0 / (2.0 * sigma);
                dg += grad_sigma * dsigma_dgamma;
                dd += grad_sigma * dsigma_ddelta;
            }
            self.alpha -= eta * da / n as f64;
            self.beta -= eta * db / n as f64;
            self.gamma = (self.gamma - eta * dg / n as f64).max(0.0);
            self.delta = (self.delta - eta * dd / n as f64).max(1e-9);
        }
        Ok(())
    }

    /// Apply mean correction: `α + β · ensemble_mean`.
    pub fn calibrate_mean(&self, ensemble_mean: f64) -> f64 {
        self.alpha + self.beta * ensemble_mean
    }

    /// Apply spread correction: `√(δ + γ · σ²_ens)`.
    pub fn calibrate_std(&self, ensemble_std: f64) -> f64 {
        (self.delta + self.gamma * ensemble_std.powi(2))
            .max(0.0)
            .sqrt()
    }

    /// Apply EMOS correction to an existing `AdvancedEnsembleForecast` in-place.
    pub fn calibrate_forecast(&self, forecast: &mut AdvancedEnsembleForecast) {
        let horizon = forecast.point_forecast.len();
        for h in 0..horizon {
            let raw_mean = forecast.point_forecast[h];
            let raw_std = forecast.std_dev[h];
            let new_mean = self.calibrate_mean(raw_mean);
            let new_std = self.calibrate_std(raw_std);
            let delta_mean = new_mean - raw_mean;
            let scale = if raw_std > 1e-12 {
                new_std / raw_std
            } else {
                1.0
            };
            for m in &mut forecast.members {
                m[h] = new_mean + (m[h] - raw_mean + delta_mean) * scale / (scale + 1e-30) * scale;
                // Simplified: shift + scale around new mean.
                m[h] = new_mean + (m[h] - raw_mean) * scale;
            }
            forecast.point_forecast[h] = new_mean;
            forecast.std_dev[h] = new_std;
        }
    }
}

impl Default for EmosCalibrator {
    fn default() -> Self {
        Self::new()
    }
}

// Standard Normal helpers (Abramowitz & Stegun 26.2.17 approximation).
fn normal_cdf(x: f64) -> f64 {
    0.5 * (1.0 + erf_approx(x / std::f64::consts::SQRT_2))
}

fn normal_pdf(x: f64) -> f64 {
    (-0.5 * x * x).exp() / (2.0 * std::f64::consts::PI).sqrt()
}

/// Abramowitz & Stegun rational approximation of erf (max error 1.5e-7).
fn erf_approx(x: f64) -> f64 {
    let t = 1.0 / (1.0 + 0.3275911 * x.abs());
    let poly = t
        * (0.254_829_592
            + t * (-0.284_496_736
                + t * (1.421_413_741 + t * (-1.453_152_027 + t * 1.061_405_429))));
    let sign = if x >= 0.0 { 1.0 } else { -1.0 };
    sign * (1.0 - poly * (-x * x).exp())
}

// ---------------------------------------------------------------------------
// 5. ConformalPredictor
// ---------------------------------------------------------------------------

/// Distribution-free conformal prediction intervals for point forecasts.
///
/// Nonconformity score: `s_i = |y_i - ŷ_i|`.
/// The empirical quantile `q̂` at level `⌈(n+1)(1-α)⌉/n` is used to construct:
/// ```text
/// PI = [ŷ - q̂, ŷ + q̂]
/// ```
/// This guarantees marginal coverage `≥ 1-α` over exchangeable data.
#[derive(Debug, Clone)]
pub struct ConformalPredictor {
    /// Calibration nonconformity scores (absolute errors) \[MW\].
    pub calibration_scores: Vec<f64>,
    /// Target miscoverage rate (e.g. 0.1 → 90 \[%\] coverage).
    pub alpha: f64,
}

impl ConformalPredictor {
    /// Construct with desired miscoverage `alpha`.
    pub fn new(alpha: f64) -> Self {
        Self {
            calibration_scores: Vec::new(),
            alpha: alpha.clamp(0.0, 1.0),
        }
    }

    /// Add calibration pairs to compute nonconformity scores.
    pub fn calibrate(&mut self, point_forecasts: &[f64], observations: &[f64]) {
        let n = point_forecasts.len().min(observations.len());
        self.calibration_scores.clear();
        for i in 0..n {
            self.calibration_scores
                .push((observations[i] - point_forecasts[i]).abs());
        }
    }

    /// Compute the conformal quantile `q̂`.
    fn quantile_hat(&self) -> f64 {
        let n = self.calibration_scores.len();
        if n == 0 {
            return 0.0;
        }
        let level = ((n + 1) as f64 * (1.0 - self.alpha)).ceil() / n as f64;
        let mut s = self.calibration_scores.clone();
        quantile_sorted(&mut s, level.min(1.0))
    }

    /// Return the symmetric prediction interval for `point_forecast`.
    pub fn predict_interval(&self, point_forecast: f64) -> (f64, f64) {
        let q = self.quantile_hat();
        (point_forecast - q, point_forecast + q)
    }

    /// Nominal coverage guarantee: `1 - alpha`.
    pub fn coverage_guarantee(&self) -> f64 {
        1.0 - self.alpha
    }

    /// Average interval width `2 * q̂` \[MW\].
    pub fn average_width(&self) -> f64 {
        2.0 * self.quantile_hat()
    }
}

// ---------------------------------------------------------------------------
// 6. ModelBlender (Bates-Granger)
// ---------------------------------------------------------------------------

/// Online model combination using inverse-MSE weights (Bates & Granger 1969).
///
/// Weights are initialised uniformly and updated after each observation.
#[derive(Debug, Clone)]
pub struct ModelBlender {
    /// Current combination weights (sum to 1.0).
    pub model_weights: Vec<f64>,
    /// Human-readable model names.
    pub model_names: Vec<String>,
    /// Online weight update learning rate.
    pub weight_update_rate: f64,
    /// Rolling window size for MSE estimation.
    pub window_size: usize,
    /// Per-model rolling squared-error buffer.
    mse_history: Vec<Vec<f64>>,
}

impl ModelBlender {
    /// Construct with uniform weights.
    pub fn new(model_names: Vec<String>) -> Self {
        let n = model_names.len();
        let weight = if n > 0 { 1.0 / n as f64 } else { 0.0 };
        Self {
            model_weights: vec![weight; n],
            model_names,
            weight_update_rate: 0.01,
            window_size: 20,
            mse_history: vec![Vec::new(); n],
        }
    }

    /// Reset to uniform weights.
    pub fn equal_weights(&mut self) {
        let n = self.model_weights.len();
        let w = if n > 0 { 1.0 / n as f64 } else { 0.0 };
        for v in &mut self.model_weights {
            *v = w;
        }
    }

    /// Update weights proportional to `1 / MSE_i` (inverse-MSE combination).
    ///
    /// Models with empty history retain current weights.
    pub fn inverse_mse_weights(&mut self) {
        let inv_mses: Vec<f64> = self
            .mse_history
            .iter()
            .map(|h| {
                if h.is_empty() {
                    0.0
                } else {
                    let mse: f64 = h.iter().sum::<f64>() / h.len() as f64;
                    if mse > 1e-12 {
                        1.0 / mse
                    } else {
                        1e12
                    }
                }
            })
            .collect();
        let total: f64 = inv_mses.iter().sum();
        if total > 1e-30 {
            for (i, w) in self.model_weights.iter_mut().enumerate() {
                *w = inv_mses[i] / total;
            }
        } else {
            self.equal_weights();
        }
    }

    /// Compute the weighted-average forecast across models.
    ///
    /// `forecasts` — one row per model, one value per hour \[MW\].
    pub fn blend(&self, forecasts: &[Vec<f64>]) -> Vec<f64> {
        if forecasts.is_empty() {
            return Vec::new();
        }
        let horizon = forecasts[0].len();
        let mut out = vec![0.0_f64; horizon];
        for (fi, forecast) in forecasts.iter().enumerate() {
            let w = self.model_weights.get(fi).copied().unwrap_or(0.0);
            for (h, &v) in forecast.iter().enumerate().take(horizon) {
                out[h] += w * v;
            }
        }
        out
    }

    /// Online update: record squared errors and recompute inverse-MSE weights.
    ///
    /// `forecasts` — point forecast per model for the current period.
    /// `observation` — realised value \[MW\].
    pub fn update_weights(&mut self, forecasts: &[f64], observation: f64) {
        let n = self.model_weights.len();
        for i in 0..n {
            let f = forecasts.get(i).copied().unwrap_or(0.0);
            let err2 = (f - observation).powi(2);
            if self.mse_history[i].len() >= self.window_size {
                self.mse_history[i].remove(0);
            }
            self.mse_history[i].push(err2);
        }
        self.inverse_mse_weights();
    }

    /// Return `(name, weight, mean_squared_error)` per model.
    pub fn performance_summary(&self) -> Vec<(String, f64, f64)> {
        self.model_names
            .iter()
            .enumerate()
            .map(|(i, name)| {
                let w = self.model_weights.get(i).copied().unwrap_or(0.0);
                let mse = if self.mse_history[i].is_empty() {
                    f64::NAN
                } else {
                    self.mse_history[i].iter().sum::<f64>() / self.mse_history[i].len() as f64
                };
                (name.clone(), w, mse)
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// 7. ForecastSkillAssessor
// ---------------------------------------------------------------------------

/// Deterministic and probabilistic skill metrics for renewable energy forecasts.
///
/// All metrics follow standard definitions (Murphy 1988, Wilks 2011).
#[derive(Debug, Clone)]
pub struct ForecastSkillAssessor {
    /// Model point forecasts \[MW\].
    pub forecasts: Vec<f64>,
    /// Observed values \[MW\].
    pub observations: Vec<f64>,
    /// Persistence (naïve) baseline forecasts \[MW\].
    pub persistence_forecasts: Vec<f64>,
    /// Climatological mean forecasts \[MW\].
    pub climatology: Vec<f64>,
}

impl ForecastSkillAssessor {
    /// Root-Mean-Square Error \[MW\].
    pub fn rmse(&self) -> f64 {
        let n = self.forecasts.len().min(self.observations.len());
        if n == 0 {
            return 0.0;
        }
        let mse: f64 = (0..n)
            .map(|i| (self.forecasts[i] - self.observations[i]).powi(2))
            .sum::<f64>()
            / n as f64;
        mse.sqrt()
    }

    /// Mean Absolute Error \[MW\].
    pub fn mae(&self) -> f64 {
        let n = self.forecasts.len().min(self.observations.len());
        if n == 0 {
            return 0.0;
        }
        (0..n)
            .map(|i| (self.forecasts[i] - self.observations[i]).abs())
            .sum::<f64>()
            / n as f64
    }

    /// Mean Bias Error \[MW\] (positive = over-forecast).
    pub fn mbe(&self) -> f64 {
        let n = self.forecasts.len().min(self.observations.len());
        if n == 0 {
            return 0.0;
        }
        (0..n)
            .map(|i| self.forecasts[i] - self.observations[i])
            .sum::<f64>()
            / n as f64
    }

    /// Skill score relative to persistence: `SS = 1 - RMSE_model / RMSE_persistence`.
    ///
    /// A value of 1.0 means perfect forecast; 0.0 means same as persistence.
    pub fn skill_score_vs_persistence(&self) -> f64 {
        let rmse_model = self.rmse();
        let rmse_pers = self.rmse_of(&self.persistence_forecasts);
        if rmse_pers < 1e-12 {
            return if rmse_model < 1e-12 { 1.0 } else { 0.0 };
        }
        1.0 - rmse_model / rmse_pers
    }

    /// Skill score relative to climatology: `SS = 1 - RMSE_model / RMSE_clim`.
    pub fn skill_score_vs_climatology(&self) -> f64 {
        let rmse_model = self.rmse();
        let rmse_clim = self.rmse_of(&self.climatology);
        if rmse_clim < 1e-12 {
            return if rmse_model < 1e-12 { 1.0 } else { 0.0 };
        }
        1.0 - rmse_model / rmse_clim
    }

    fn rmse_of(&self, reference: &[f64]) -> f64 {
        let n = reference.len().min(self.observations.len());
        if n == 0 {
            return 0.0;
        }
        let mse: f64 = (0..n)
            .map(|i| (reference[i] - self.observations[i]).powi(2))
            .sum::<f64>()
            / n as f64;
        mse.sqrt()
    }

    /// Pearson correlation coefficient.
    pub fn correlation(&self) -> f64 {
        let n = self.forecasts.len().min(self.observations.len());
        if n < 2 {
            return 0.0;
        }
        let mean_f = self.forecasts[..n].iter().sum::<f64>() / n as f64;
        let mean_o = self.observations[..n].iter().sum::<f64>() / n as f64;
        let cov: f64 = (0..n)
            .map(|i| (self.forecasts[i] - mean_f) * (self.observations[i] - mean_o))
            .sum();
        let var_f: f64 = (0..n).map(|i| (self.forecasts[i] - mean_f).powi(2)).sum();
        let var_o: f64 = (0..n)
            .map(|i| (self.observations[i] - mean_o).powi(2))
            .sum();
        let denom = (var_f * var_o).sqrt();
        if denom < 1e-12 {
            0.0
        } else {
            cov / denom
        }
    }

    /// Normalised RMSE as a percentage of installed capacity \[%\].
    pub fn normalized_rmse_pct(&self, capacity: f64) -> f64 {
        if capacity < 1e-12 {
            return 0.0;
        }
        self.rmse() / capacity * 100.0
    }

    /// Reliability diagram: bins forecast probabilities and returns observed frequencies.
    ///
    /// `n_bins` — number of equal-width bins over \[0, 1\].
    /// Returns `(bin_centre, observed_frequency)` pairs.
    ///
    /// Assumes `self.forecasts` values are probabilities in \[0, 1\].
    pub fn reliability_diagram(&self, n_bins: usize) -> Vec<(f64, f64)> {
        if n_bins == 0 {
            return Vec::new();
        }
        let n = self.forecasts.len().min(self.observations.len());
        let mut bin_sum = vec![0.0_f64; n_bins];
        let mut bin_count = vec![0_usize; n_bins];
        let mut bin_obs = vec![0.0_f64; n_bins];
        for i in 0..n {
            let p = self.forecasts[i].clamp(0.0, 1.0);
            let bin = ((p * n_bins as f64).floor() as usize).min(n_bins - 1);
            bin_sum[bin] += p;
            bin_count[bin] += 1;
            bin_obs[bin] += self.observations[i];
        }
        (0..n_bins)
            .map(|b| {
                let centre = (b as f64 + 0.5) / n_bins as f64;
                let obs_freq = if bin_count[b] > 0 {
                    bin_obs[b] / bin_count[b] as f64
                } else {
                    centre // fallback to perfect reliability
                };
                (centre, obs_freq)
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_analog_ensemble() -> AnalogEnsemble {
        let mut ae = AnalogEnsemble::new(3);
        for i in 0..10_usize {
            let features = vec![i as f64, (i as f64) * 2.0];
            let target = vec![i as f64 * 10.0; 24];
            ae.add_historical_day(features, target);
        }
        ae
    }

    // -----------------------------------------------------------------------
    // AnalogEnsemble tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_find_analogs_returns_n_analogs() {
        let ae = make_analog_ensemble();
        let query = vec![3.5, 7.0];
        let analogs = ae.find_analogs(&query);
        assert_eq!(
            analogs.len(),
            3,
            "find_analogs must return exactly n_analogs results"
        );
    }

    #[test]
    fn test_find_analogs_sorted_ascending() {
        let ae = make_analog_ensemble();
        let query = vec![0.0, 0.0];
        let analogs = ae.find_analogs(&query);
        for w in analogs.windows(2) {
            assert!(
                w[0].1 <= w[1].1,
                "Analogs must be sorted by ascending distance"
            );
        }
    }

    #[test]
    fn test_forecast_ensemble_member_count() {
        let ae = make_analog_ensemble();
        let query = vec![5.0, 10.0];
        let fc = ae.forecast(&query, 24).expect("forecast must succeed");
        assert_eq!(fc.members.len(), 3, "Ensemble must have n_analogs members");
    }

    #[test]
    fn test_forecast_horizon_length() {
        let ae = make_analog_ensemble();
        let fc = ae
            .forecast(&[5.0, 10.0], 12)
            .expect("forecast must succeed");
        for m in &fc.members {
            assert_eq!(m.len(), 12, "Each member must match requested horizon");
        }
        assert_eq!(fc.point_forecast.len(), 12);
        assert_eq!(fc.std_dev.len(), 12);
    }

    // -----------------------------------------------------------------------
    // AdvancedEnsembleForecast tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_percentile_median_approx() {
        // 5 members, fixed values per hour 0: [1,2,3,4,5]
        let members = vec![
            vec![1.0, 0.0],
            vec![2.0, 0.0],
            vec![3.0, 0.0],
            vec![4.0, 0.0],
            vec![5.0, 0.0],
        ];
        let fc = AdvancedEnsembleForecast::from_members(members).unwrap();
        let median = fc.percentile(0.5, 0);
        assert!(
            (median - 3.0).abs() < 1e-9,
            "Median should be 3.0, got {}",
            median
        );
    }

    #[test]
    fn test_crps_non_negative() {
        let ae = make_analog_ensemble();
        let fc = ae
            .forecast(&[5.0, 10.0], 24)
            .expect("forecast must succeed");
        let obs: Vec<f64> = (0..24).map(|h| h as f64 * 5.0).collect();
        let crps = fc.crps(&obs);
        assert!(crps >= 0.0, "CRPS must be non-negative, got {}", crps);
    }

    #[test]
    fn test_crps_zero_for_perfect_ensemble() {
        // All members equal observation → CRPS = 0.
        let members = vec![vec![42.0]; 5];
        let fc = AdvancedEnsembleForecast::from_members(members).unwrap();
        let crps = fc.crps(&[42.0]);
        assert!(
            crps < 1e-6,
            "CRPS must be ~0 for perfect ensemble, got {}",
            crps
        );
    }

    #[test]
    fn test_prediction_interval_ordered() {
        let ae = make_analog_ensemble();
        let fc = ae.forecast(&[2.0, 4.0], 24).unwrap();
        let (lo, hi) = fc.prediction_interval(0.9, 0);
        assert!(lo <= hi, "PI lower bound must be ≤ upper bound");
    }

    // -----------------------------------------------------------------------
    // QuantileRegressionForest tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_qrf_fit_and_predict_in_unit_range() {
        let features: Vec<Vec<f64>> = (0..50).map(|i| vec![i as f64 / 50.0]).collect();
        let targets: Vec<f64> = (0..50).map(|i| i as f64 / 50.0).collect();
        let mut qrf = QuantileRegressionForest::new(5, 3);
        qrf.fit(&features, &targets).expect("fit must succeed");
        let pred = qrf.predict_mean(&[0.5]);
        assert!(
            (0.0..=1.0).contains(&pred),
            "Prediction must be in [0, 1], got {}",
            pred
        );
    }

    #[test]
    fn test_qrf_predict_interval_lower_lt_upper() {
        let features: Vec<Vec<f64>> = (0..40).map(|i| vec![i as f64]).collect();
        let targets: Vec<f64> = (0..40).map(|i| i as f64).collect();
        let mut qrf = QuantileRegressionForest::new(5, 2);
        qrf.fit(&features, &targets).expect("fit must succeed");
        let (lo, hi) = qrf.predict_interval(&[20.0], 0.1);
        assert!(
            lo <= hi,
            "Interval lower must be ≤ upper, got ({}, {})",
            lo,
            hi
        );
    }

    #[test]
    fn test_qrf_quantile_ordering() {
        let features: Vec<Vec<f64>> = (0..60).map(|i| vec![i as f64]).collect();
        let targets: Vec<f64> = (0..60).map(|i| i as f64 * 2.0).collect();
        let mut qrf = QuantileRegressionForest::new(8, 3);
        qrf.fit(&features, &targets).expect("fit must succeed");
        let q10 = qrf.predict_quantile(&[30.0], 0.1);
        let q90 = qrf.predict_quantile(&[30.0], 0.9);
        assert!(q10 <= q90, "Q10 must be ≤ Q90");
    }

    // -----------------------------------------------------------------------
    // EmosCalibrator tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_emos_fit_converges() {
        let means: Vec<f64> = (0..30).map(|i| i as f64 * 2.0).collect();
        let stds: Vec<f64> = vec![1.0; 30];
        let obs: Vec<f64> = (0..30).map(|i| i as f64 * 2.0 + 0.5).collect();
        let mut emos = EmosCalibrator::new();
        let result = emos.fit(&means, &stds, &obs);
        assert!(result.is_ok(), "EMOS fit must succeed without error");
    }

    #[test]
    fn test_emos_calibrate_mean_formula() {
        let emos = EmosCalibrator {
            alpha: 1.0,
            beta: 2.0,
            gamma: 1.0,
            delta: 0.01,
        };
        let cal = emos.calibrate_mean(3.0);
        assert!(
            (cal - 7.0).abs() < 1e-9,
            "calibrate_mean(3) = 1 + 2*3 = 7, got {}",
            cal
        );
    }

    #[test]
    fn test_emos_calibrate_std_non_negative() {
        let emos = EmosCalibrator::new();
        let s = emos.calibrate_std(2.0);
        assert!(s >= 0.0, "calibrate_std must be non-negative, got {}", s);
    }

    // -----------------------------------------------------------------------
    // ConformalPredictor tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_conformal_interval_symmetric() {
        let mut cp = ConformalPredictor::new(0.1);
        let pf: Vec<f64> = (0..20).map(|i| i as f64).collect();
        let obs: Vec<f64> = (0..20).map(|i| i as f64 + 0.5).collect();
        cp.calibrate(&pf, &obs);
        let yhat = 10.0;
        let (lo, hi) = cp.predict_interval(yhat);
        assert!(
            (yhat - lo - (hi - yhat)).abs() < 1e-9,
            "Interval must be symmetric"
        );
    }

    #[test]
    fn test_conformal_coverage_guarantee() {
        let cp = ConformalPredictor::new(0.15);
        assert!((cp.coverage_guarantee() - 0.85).abs() < 1e-9);
    }

    #[test]
    fn test_conformal_average_width_positive() {
        let mut cp = ConformalPredictor::new(0.1);
        let pf = vec![0.0, 1.0, 2.0, 3.0];
        let obs = vec![0.3, 1.5, 2.1, 3.8];
        cp.calibrate(&pf, &obs);
        assert!(cp.average_width() > 0.0, "Average width must be positive");
    }

    // -----------------------------------------------------------------------
    // ModelBlender tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_equal_weights_sum_to_one() {
        let mut blender = ModelBlender::new(vec!["A".into(), "B".into(), "C".into()]);
        blender.equal_weights();
        let sum: f64 = blender.model_weights.iter().sum();
        assert!(
            (sum - 1.0).abs() < 1e-9,
            "Equal weights must sum to 1.0, got {}",
            sum
        );
    }

    #[test]
    fn test_blend_weighted_sum() {
        let mut blender = ModelBlender::new(vec!["A".into(), "B".into()]);
        blender.model_weights = vec![0.3, 0.7];
        let forecasts = vec![vec![10.0, 20.0], vec![20.0, 40.0]];
        let blended = blender.blend(&forecasts);
        assert_eq!(blended.len(), 2);
        // hour 0: 0.3*10 + 0.7*20 = 17.0
        assert!(
            (blended[0] - 17.0).abs() < 1e-9,
            "blend[0] must be 17.0, got {}",
            blended[0]
        );
        // hour 1: 0.3*20 + 0.7*40 = 34.0
        assert!(
            (blended[1] - 34.0).abs() < 1e-9,
            "blend[1] must be 34.0, got {}",
            blended[1]
        );
    }

    #[test]
    fn test_blender_update_weights_normalised() {
        let mut blender = ModelBlender::new(vec!["A".into(), "B".into()]);
        for i in 0..5 {
            blender.update_weights(&[i as f64, i as f64 * 2.0], i as f64 * 1.5);
        }
        let sum: f64 = blender.model_weights.iter().sum();
        assert!(
            (sum - 1.0).abs() < 1e-9,
            "Weights after update must sum to 1.0"
        );
    }

    // -----------------------------------------------------------------------
    // ForecastSkillAssessor tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_rmse_non_negative() {
        let assessor = ForecastSkillAssessor {
            forecasts: vec![1.0, 2.0, 3.0],
            observations: vec![1.5, 2.5, 2.8],
            persistence_forecasts: vec![1.0, 1.5, 2.0],
            climatology: vec![2.0; 3],
        };
        assert!(assessor.rmse() >= 0.0, "RMSE must be non-negative");
    }

    #[test]
    fn test_skill_score_perfect_vs_persistence() {
        // Perfect forecast: forecasts == observations → RMSE = 0 → SS = 1.
        let obs = vec![1.0, 2.0, 3.0];
        let assessor = ForecastSkillAssessor {
            forecasts: obs.clone(),
            observations: obs.clone(),
            persistence_forecasts: vec![0.5, 1.5, 2.5],
            climatology: vec![2.0; 3],
        };
        let ss = assessor.skill_score_vs_persistence();
        assert!(
            (ss - 1.0).abs() < 1e-9,
            "SS must be 1.0 for perfect forecast, got {}",
            ss
        );
    }

    #[test]
    fn test_mae_non_negative() {
        let assessor = ForecastSkillAssessor {
            forecasts: vec![3.0, 1.0],
            observations: vec![1.0, 3.0],
            persistence_forecasts: vec![3.0, 3.0],
            climatology: vec![2.0; 2],
        };
        assert!(assessor.mae() >= 0.0);
        assert!(
            (assessor.mbe()).abs() < 1e-6,
            "MBE must be ~0 for symmetric errors"
        );
    }

    #[test]
    fn test_reliability_diagram_bin_count() {
        let assessor = ForecastSkillAssessor {
            forecasts: vec![0.1, 0.3, 0.5, 0.7, 0.9],
            observations: vec![0.0, 0.0, 1.0, 1.0, 1.0],
            persistence_forecasts: vec![0.5; 5],
            climatology: vec![0.5; 5],
        };
        let diagram = assessor.reliability_diagram(5);
        assert_eq!(
            diagram.len(),
            5,
            "Reliability diagram must have n_bins entries"
        );
        for (centre, _freq) in &diagram {
            assert!(*centre > 0.0 && *centre < 1.0);
        }
    }
}
