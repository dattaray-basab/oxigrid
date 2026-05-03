/// Spatial wind generation modelling.
///
/// Provides:
/// - `SemiVariogram`         — empirical and theoretical semi-variogram models
/// - `GaussianCopula`        — correlated multi-site scenario generation
/// - `SpatialWindScenario`   — joint wind speed realisations across sites
/// - `SpatialCorrelation`    — correlation matrix estimation from historical data
///
/// # Background
/// Wind speed at spatially separated sites is correlated: nearby sites tend
/// to produce similar wind speeds simultaneously.  The spatial correlation
/// structure is characterised by a semi-variogram γ(h), where h is the
/// separation distance.
///
/// # Scenario Generation (Gaussian Copula)
/// 1. Fit marginal CDF F_i(u) at each site (Weibull)
/// 2. Map to standard normal: z_i = Φ⁻¹(F_i(u_i))
/// 3. Correlate using Cholesky decomposition: x = L·z  where R = L·Lᵀ
/// 4. Back-transform: u_i = F_i⁻¹(Φ(x_i))
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

// ─── Site data ────────────────────────────────────────────────────────────────

/// Geographic location of a wind site.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct SiteLocation {
    /// Easting `km` or longitude (arbitrary consistent units)
    pub x_km: f64,
    /// Northing `km` or latitude
    pub y_km: f64,
    /// Hub height `m`
    pub hub_height_m: f64,
    /// Weibull shape parameter k (≈ 2 for typical wind)
    pub weibull_k: f64,
    /// Weibull scale parameter c [m/s]
    pub weibull_c: f64,
    /// Rated capacity `MW`
    pub capacity_mw: f64,
}

impl SiteLocation {
    /// Distance to another site `km`.
    pub fn distance_km(&self, other: &SiteLocation) -> f64 {
        let dx = self.x_km - other.x_km;
        let dy = self.y_km - other.y_km;
        (dx * dx + dy * dy).sqrt()
    }
}

// ─── Weibull distribution ────────────────────────────────────────────────────

/// Weibull CDF: F(u) = 1 − exp(−(u/c)^k)
pub fn weibull_cdf(u: f64, k: f64, c: f64) -> f64 {
    if u <= 0.0 {
        return 0.0;
    }
    1.0 - (-(u / c).powf(k)).exp()
}

/// Weibull inverse CDF: F⁻¹(p) = c·(−ln(1−p))^{1/k}
pub fn weibull_icdf(p: f64, k: f64, c: f64) -> f64 {
    let p = p.clamp(1e-9, 1.0 - 1e-9);
    c * (-(1.0 - p).ln()).powf(1.0 / k)
}

/// Weibull mean wind speed: `E[U]` = c·Γ(1 + 1/k)
pub fn weibull_mean(k: f64, c: f64) -> f64 {
    c * gamma_func(1.0 + 1.0 / k)
}

/// Lanczos approximation to Gamma function.
fn gamma_func(z: f64) -> f64 {
    if z < 0.5 {
        return PI / ((PI * z).sin() * gamma_func(1.0 - z));
    }
    let g = 7.0;
    let c = [
        0.999_999_999_999_809_9,
        676.520_368_121_885_1,
        -1_259.139_216_722_403,
        771.323_428_777_653_1,
        -176.615_029_162_140_6,
        12.507_343_278_686_9,
        -0.138_571_095_265_72,
        9.984_369_578_019_57e-6,
        1.505_632_735_149_31e-7,
    ];
    let z = z - 1.0;
    let x = c[0]
        + c[1..]
            .iter()
            .enumerate()
            .map(|(i, &ci)| ci / (z + (i + 1) as f64))
            .sum::<f64>();
    let t = z + g + 0.5;
    (2.0 * PI).sqrt() * t.powf(z + 0.5) * (-t).exp() * x
}

// ─── Semi-variogram ──────────────────────────────────────────────────────────

/// Variogram model type.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum VariogramModel {
    /// γ(h) = c₀ + c·(1 − exp(−h/a))
    Exponential,
    /// γ(h) = c₀ + c·(1 − exp(−(h/a)²))
    Gaussian,
    /// γ(h) = c₀ + c·(3h/2a − (h/a)³/2) for h≤a; c₀+c for h>a
    Spherical,
}

/// Theoretical semi-variogram.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SemiVariogram {
    /// Model type
    pub model: VariogramModel,
    /// Nugget variance c₀ (discontinuity at h=0)
    pub nugget: f64,
    /// Sill c (total variance for large h)
    pub sill: f64,
    /// Range a (correlation length `km`)
    pub range: f64,
}

impl SemiVariogram {
    /// Exponential model: typical for wind fields.
    pub fn exponential(nugget: f64, sill: f64, range_km: f64) -> Self {
        Self {
            model: VariogramModel::Exponential,
            nugget,
            sill,
            range: range_km,
        }
    }

    /// Gaussian model.
    pub fn gaussian(nugget: f64, sill: f64, range_km: f64) -> Self {
        Self {
            model: VariogramModel::Gaussian,
            nugget,
            sill,
            range: range_km,
        }
    }

    /// Spherical model.
    pub fn spherical(nugget: f64, sill: f64, range_km: f64) -> Self {
        Self {
            model: VariogramModel::Spherical,
            nugget,
            sill,
            range: range_km,
        }
    }

    /// Evaluate γ(h) at separation distance h `km`.
    pub fn eval(&self, h: f64) -> f64 {
        if h < 1e-9 {
            return 0.0;
        }
        let c = self.sill;
        let a = self.range.max(1e-9);
        let c0 = self.nugget;
        match self.model {
            VariogramModel::Exponential => c0 + c * (1.0 - (-(h / a)).exp()),
            VariogramModel::Gaussian => c0 + c * (1.0 - (-(h / a).powi(2)).exp()),
            VariogramModel::Spherical => {
                if h >= a {
                    c0 + c
                } else {
                    c0 + c * (1.5 * h / a - 0.5 * (h / a).powi(3))
                }
            }
        }
    }

    /// Covariance C(h) = C(0) − γ(h).
    pub fn covariance(&self, h: f64) -> f64 {
        let c0 = self.nugget + self.sill;
        c0 - self.eval(h)
    }

    /// Build the spatial correlation matrix from site locations.
    pub fn correlation_matrix(&self, sites: &[SiteLocation]) -> Vec<Vec<f64>> {
        let n = sites.len();
        let c0 = self.nugget + self.sill;
        (0..n)
            .map(|i| {
                (0..n)
                    .map(|j| {
                        if i == j {
                            1.0
                        } else {
                            let h = sites[i].distance_km(&sites[j]);
                            (self.covariance(h) / c0.max(1e-9)).clamp(0.0, 1.0)
                        }
                    })
                    .collect()
            })
            .collect()
    }

    /// Fit an exponential variogram to empirical (h, γ) pairs via least squares.
    pub fn fit_exponential(pairs: &[(f64, f64)]) -> Self {
        // Simple 2-parameter fit: nugget=0, optimize (sill, range)
        // Using grid search over sill ∈ [0.1·σ², 2·σ²] and range ∈ [1, 500 km]
        let gamma_max = pairs.iter().map(|(_, g)| *g).fold(0.0f64, f64::max);

        let mut best_err = f64::INFINITY;
        let mut best_sill = gamma_max;
        let mut best_range = 50.0;

        let sill_vals = [gamma_max * 0.5, gamma_max * 0.8, gamma_max, gamma_max * 1.2];
        let range_vals = [10.0, 30.0, 50.0, 100.0, 200.0, 300.0];

        for &sill in &sill_vals {
            for &range in &range_vals {
                let vgram = Self::exponential(0.0, sill, range);
                let err: f64 = pairs
                    .iter()
                    .map(|(h, g)| (vgram.eval(*h) - g).powi(2))
                    .sum();
                if err < best_err {
                    best_err = err;
                    best_sill = sill;
                    best_range = range;
                }
            }
        }
        Self::exponential(0.0, best_sill, best_range)
    }
}

/// Compute empirical semi-variogram from historical wind data.
///
/// # Arguments
/// - `locations` — site coordinates
/// - `data`      — wind speeds: `data[site][time]`
/// - `n_lags`    — number of distance lag bins
pub fn empirical_variogram(
    locations: &[SiteLocation],
    data: &[Vec<f64>],
    n_lags: usize,
) -> Vec<(f64, f64)> {
    let n_sites = locations.len();
    if n_sites < 2 || data.is_empty() {
        return vec![];
    }

    let n_times = data[0].len();
    let max_dist = locations
        .iter()
        .flat_map(|a| locations.iter().map(move |b| a.distance_km(b)))
        .fold(0.0f64, f64::max);

    let lag_width = max_dist / n_lags as f64;
    let mut lag_sum = vec![0.0f64; n_lags];
    let mut lag_count = vec![0usize; n_lags];
    let mut lag_dist = vec![0.0f64; n_lags];

    for i in 0..n_sites {
        for j in (i + 1)..n_sites {
            let h = locations[i].distance_km(&locations[j]);
            let lag = ((h / lag_width) as usize).min(n_lags - 1);

            let gamma_ij: f64 = (0..n_times)
                .map(|t| (data[i][t] - data[j][t]).powi(2))
                .sum::<f64>()
                / (2.0 * n_times as f64);

            lag_sum[lag] += gamma_ij;
            lag_count[lag] += 1;
            lag_dist[lag] += h;
        }
    }

    (0..n_lags)
        .filter(|&k| lag_count[k] > 0)
        .map(|k| {
            let h_avg = lag_dist[k] / lag_count[k] as f64;
            let gamma_avg = lag_sum[k] / lag_count[k] as f64;
            (h_avg, gamma_avg)
        })
        .collect()
}

// ─── Gaussian Copula ──────────────────────────────────────────────────────────

/// Gaussian copula for multi-site correlated wind scenario generation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GaussianCopula {
    /// Number of sites
    pub n_sites: usize,
    /// Correlation matrix (n × n)
    pub corr: Vec<Vec<f64>>,
    /// Lower Cholesky factor L (n × n) such that R = L·Lᵀ
    cholesky: Vec<Vec<f64>>,
}

impl GaussianCopula {
    /// Create from a correlation matrix.
    pub fn new(corr: Vec<Vec<f64>>) -> Option<Self> {
        let n = corr.len();
        let chol = cholesky_decompose(&corr)?;
        Some(Self {
            n_sites: n,
            corr,
            cholesky: chol,
        })
    }

    /// Build from a semi-variogram and site locations.
    pub fn from_variogram(vgram: &SemiVariogram, sites: &[SiteLocation]) -> Option<Self> {
        let corr = vgram.correlation_matrix(sites);
        Self::new(corr)
    }

    /// Generate `n_scenarios` correlated uniform samples using a deterministic
    /// quasi-random sequence (Halton).
    ///
    /// Returns a `[n_scenarios × n_sites]` matrix of uniform `0,1` samples.
    pub fn generate_uniform_scenarios(&self, n_scenarios: usize) -> Vec<Vec<f64>> {
        let n = self.n_sites;

        // Generate independent standard normal samples via Halton sequence
        let mut scenarios = Vec::with_capacity(n_scenarios);

        for s in 0..n_scenarios {
            // Independent standard normals via Box-Muller from Halton
            let z_ind: Vec<f64> = (0..n)
                .map(|i| {
                    let u1 = halton(s + 1, PRIMES[i % PRIMES.len()]);
                    let u2 = halton(s + 1, PRIMES[(i + 1) % PRIMES.len()]);
                    let u1 = u1.clamp(1e-10, 1.0 - 1e-10);
                    (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
                })
                .collect();

            // Apply Cholesky: x = L·z
            let x_corr: Vec<f64> = (0..n)
                .map(|i| (0..=i).map(|j| self.cholesky[i][j] * z_ind[j]).sum())
                .collect();

            // Transform to uniform via standard normal CDF
            let u: Vec<f64> = x_corr.iter().map(|&x| norm_cdf(x)).collect();
            scenarios.push(u);
        }
        scenarios
    }

    /// Generate correlated wind speed scenarios at each site.
    ///
    /// Returns `[n_scenarios × n_sites]` wind speeds [m/s].
    pub fn generate_wind_scenarios(
        &self,
        sites: &[SiteLocation],
        n_scenarios: usize,
    ) -> Vec<Vec<f64>> {
        let u_scenarios = self.generate_uniform_scenarios(n_scenarios);
        u_scenarios
            .iter()
            .map(|u_row| {
                u_row
                    .iter()
                    .zip(sites.iter())
                    .map(|(&u, site)| weibull_icdf(u, site.weibull_k, site.weibull_c))
                    .collect()
            })
            .collect()
    }

    /// Generate correlated wind power scenarios `MW` at each site.
    pub fn generate_power_scenarios(
        &self,
        sites: &[SiteLocation],
        n_scenarios: usize,
    ) -> Vec<Vec<f64>> {
        let wind_scenarios = self.generate_wind_scenarios(sites, n_scenarios);
        wind_scenarios
            .iter()
            .map(|winds| {
                winds
                    .iter()
                    .zip(sites.iter())
                    .map(|(&u_ms, site)| wind_to_power(u_ms, site.capacity_mw))
                    .collect()
            })
            .collect()
    }
}

/// Simple cubic wind power curve: P = P_rated · (u³/u_r³) for 3 ≤ u ≤ 12 m/s.
pub fn wind_to_power(u_ms: f64, p_rated_mw: f64) -> f64 {
    let u_cut_in = 3.0;
    let u_rated = 12.0;
    let u_cut_out = 25.0;
    if u_ms < u_cut_in || u_ms > u_cut_out {
        0.0
    } else if u_ms >= u_rated {
        p_rated_mw
    } else {
        p_rated_mw * (u_ms / u_rated).powi(3)
    }
}

/// Cholesky decomposition for positive definite matrix.
/// Returns lower triangular L such that A = L·Lᵀ.
fn cholesky_decompose(a: &[Vec<f64>]) -> Option<Vec<Vec<f64>>> {
    let n = a.len();
    let mut l = vec![vec![0.0f64; n]; n];
    for i in 0..n {
        for j in 0..=i {
            let sum: f64 = (0..j).map(|k| l[i][k] * l[j][k]).sum();
            if i == j {
                let diag = a[i][i] - sum;
                if diag < 1e-14 {
                    return None;
                }
                l[i][j] = diag.sqrt();
            } else {
                if l[j][j].abs() < 1e-14 {
                    return None;
                }
                l[i][j] = (a[i][j] - sum) / l[j][j];
            }
        }
    }
    Some(l)
}

/// Standard normal CDF via error function approximation.
pub fn norm_cdf(x: f64) -> f64 {
    0.5 * (1.0 + erf(x / 2.0_f64.sqrt()))
}

/// Standard normal inverse CDF (rational approximation by Peter Acklam).
pub fn norm_icdf(p: f64) -> f64 {
    let p = p.clamp(1e-9, 1.0 - 1e-9);
    if p < 0.5 {
        -rational_approx((-(2.0 * p).ln()).sqrt())
    } else {
        rational_approx((-(2.0 * (1.0 - p)).ln()).sqrt())
    }
}

fn rational_approx(t: f64) -> f64 {
    let c = [2.515517, 0.802853, 0.010328];
    let d = [1.432788, 0.189269, 0.001308];
    let num = c[0] + c[1] * t + c[2] * t * t;
    let den = 1.0 + d[0] * t + d[1] * t * t + d[2] * t * t * t;
    t - num / den
}

/// Error function via Chebyshev approximation.
fn erf(x: f64) -> f64 {
    let sign = if x < 0.0 { -1.0 } else { 1.0 };
    let x = x.abs();
    let t = 1.0 / (1.0 + 0.3275911 * x);
    let poly = t
        * (0.254829592
            + t * (-0.284496736 + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429))));
    sign * (1.0 - poly * (-x * x).exp())
}

/// Halton low-discrepancy sequence.
fn halton(index: usize, base: usize) -> f64 {
    let mut result = 0.0;
    let mut f = 1.0;
    let mut i = index;
    while i > 0 {
        f /= base as f64;
        result += f * (i % base) as f64;
        i /= base;
    }
    result
}

const PRIMES: [usize; 10] = [2, 3, 5, 7, 11, 13, 17, 19, 23, 29];

// ─── Spatial correlation statistics ─────────────────────────────────────────

/// Estimate cross-correlation matrix from historical data.
///
/// `data[site][time]` → n_sites × n_sites correlation matrix
pub fn estimate_correlation_matrix(data: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let n = data.len();
    if n == 0 {
        return vec![];
    }
    let n_t = data[0].len();
    if n_t < 2 {
        return vec![vec![1.0; n]; n];
    }

    let means: Vec<f64> = data
        .iter()
        .map(|d| d.iter().sum::<f64>() / n_t as f64)
        .collect();
    let stds: Vec<f64> = data
        .iter()
        .zip(means.iter())
        .map(|(d, &mu)| {
            let var = d.iter().map(|&x| (x - mu).powi(2)).sum::<f64>() / n_t as f64;
            var.sqrt().max(1e-9)
        })
        .collect();

    (0..n)
        .map(|i| {
            (0..n)
                .map(|j| {
                    if i == j {
                        return 1.0;
                    }
                    let cov: f64 = (0..n_t)
                        .map(|t| (data[i][t] - means[i]) * (data[j][t] - means[j]))
                        .sum::<f64>()
                        / n_t as f64;
                    (cov / (stds[i] * stds[j])).clamp(-1.0, 1.0)
                })
                .collect()
        })
        .collect()
}

/// Summary statistics for a multi-site wind scenario set.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScenarioStats {
    /// Mean wind speed per site [m/s]
    pub mean_speed: Vec<f64>,
    /// Standard deviation per site [m/s]
    pub std_speed: Vec<f64>,
    /// Empirical correlation between site pairs
    pub empirical_corr: Vec<Vec<f64>>,
    /// Mean total power `MW`
    pub mean_total_power_mw: f64,
}

impl ScenarioStats {
    pub fn from_wind_scenarios(scenarios: &[Vec<f64>], sites: &[SiteLocation]) -> Self {
        let n_sites = sites.len();
        let n_sc = scenarios.len();
        if n_sc == 0 {
            return Self {
                mean_speed: vec![0.0; n_sites],
                std_speed: vec![0.0; n_sites],
                empirical_corr: vec![vec![1.0; n_sites]; n_sites],
                mean_total_power_mw: 0.0,
            };
        }

        // Transpose: per_site[s][sc]
        let per_site: Vec<Vec<f64>> = (0..n_sites)
            .map(|s| scenarios.iter().map(|row| row[s]).collect())
            .collect();

        let mean_speed: Vec<f64> = per_site
            .iter()
            .map(|d| d.iter().sum::<f64>() / n_sc as f64)
            .collect();
        let std_speed: Vec<f64> = per_site
            .iter()
            .zip(mean_speed.iter())
            .map(|(d, &mu)| {
                let var = d.iter().map(|&x| (x - mu).powi(2)).sum::<f64>() / n_sc as f64;
                var.sqrt()
            })
            .collect();

        let empirical_corr = estimate_correlation_matrix(&per_site);

        let mean_total_power_mw = scenarios
            .iter()
            .map(|row| {
                row.iter()
                    .zip(sites.iter())
                    .map(|(&u, site)| wind_to_power(u, site.capacity_mw))
                    .sum::<f64>()
            })
            .sum::<f64>()
            / n_sc as f64;

        Self {
            mean_speed,
            std_speed,
            empirical_corr,
            mean_total_power_mw,
        }
    }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn three_sites() -> Vec<SiteLocation> {
        vec![
            SiteLocation {
                x_km: 0.0,
                y_km: 0.0,
                hub_height_m: 80.0,
                weibull_k: 2.0,
                weibull_c: 8.0,
                capacity_mw: 10.0,
            },
            SiteLocation {
                x_km: 10.0,
                y_km: 0.0,
                hub_height_m: 80.0,
                weibull_k: 2.1,
                weibull_c: 8.5,
                capacity_mw: 12.0,
            },
            SiteLocation {
                x_km: 0.0,
                y_km: 15.0,
                hub_height_m: 80.0,
                weibull_k: 1.9,
                weibull_c: 7.5,
                capacity_mw: 8.0,
            },
        ]
    }

    // ── Weibull ──
    #[test]
    fn test_weibull_cdf_at_zero() {
        assert_eq!(weibull_cdf(0.0, 2.0, 8.0), 0.0);
    }

    #[test]
    fn test_weibull_cdf_monotone() {
        let k = 2.0;
        let c = 8.0;
        let p1 = weibull_cdf(5.0, k, c);
        let p2 = weibull_cdf(10.0, k, c);
        assert!(p1 < p2, "CDF should be monotone: {:.4} < {:.4}", p1, p2);
    }

    #[test]
    fn test_weibull_icdf_roundtrip() {
        let k = 2.0;
        let c = 8.0;
        let u = 7.5;
        let p = weibull_cdf(u, k, c);
        let u_back = weibull_icdf(p, k, c);
        assert!(
            (u_back - u).abs() < 1e-6,
            "ICDF round trip: {:.6} ≠ {:.6}",
            u_back,
            u
        );
    }

    #[test]
    fn test_weibull_mean_positive() {
        assert!(weibull_mean(2.0, 8.0) > 0.0);
    }

    // ── Semi-variogram ──
    #[test]
    fn test_variogram_zero_at_origin() {
        let v = SemiVariogram::exponential(0.0, 1.0, 50.0);
        assert!(v.eval(0.0).abs() < 1e-9, "γ(0) should be 0");
    }

    #[test]
    fn test_variogram_increasing() {
        let v = SemiVariogram::exponential(0.0, 2.0, 50.0);
        assert!(
            v.eval(10.0) < v.eval(50.0),
            "Semi-variogram should increase"
        );
    }

    #[test]
    fn test_variogram_approaches_sill() {
        let v = SemiVariogram::exponential(0.0, 2.0, 50.0);
        assert!(
            v.eval(500.0) > 1.8,
            "Should approach sill at large distances"
        );
    }

    #[test]
    fn test_spherical_variogram_plateau() {
        let v = SemiVariogram::spherical(0.0, 1.0, 50.0);
        // Beyond range, should equal sill
        assert!(
            (v.eval(100.0) - 1.0).abs() < 1e-9,
            "Spherical should equal sill beyond range: {:.4}",
            v.eval(100.0)
        );
    }

    #[test]
    fn test_correlation_matrix_diagonal_one() {
        let sites = three_sites();
        let v = SemiVariogram::exponential(0.0, 1.0, 50.0);
        let corr = v.correlation_matrix(&sites);
        for (i, corr_row) in corr.iter().enumerate() {
            assert!((corr_row[i] - 1.0).abs() < 1e-9);
        }
    }

    #[test]
    fn test_correlation_matrix_symmetric() {
        let sites = three_sites();
        let v = SemiVariogram::exponential(0.0, 1.0, 50.0);
        let corr = v.correlation_matrix(&sites);
        for (i, corr_row) in corr.iter().enumerate() {
            for (j, corr_val) in corr_row.iter().enumerate() {
                assert!((corr_val - corr[j][i]).abs() < 1e-9);
            }
        }
    }

    #[test]
    fn test_variogram_fit() {
        let v_true = SemiVariogram::exponential(0.0, 2.0, 80.0);
        let pairs: Vec<(f64, f64)> = [10.0, 30.0, 50.0, 100.0, 200.0]
            .iter()
            .map(|&h| (h, v_true.eval(h)))
            .collect();
        let v_fit = SemiVariogram::fit_exponential(&pairs);
        // Fitted sill should be close to true sill
        assert!(
            (v_fit.sill - 2.0).abs() < 1.5,
            "Fitted sill: {:.4}",
            v_fit.sill
        );
    }

    // ── Gaussian Copula ──
    #[test]
    fn test_copula_creates() {
        let sites = three_sites();
        let v = SemiVariogram::exponential(0.0, 1.0, 50.0);
        let copula = GaussianCopula::from_variogram(&v, &sites);
        assert!(copula.is_some(), "Copula should be created");
    }

    #[test]
    fn test_scenarios_correct_shape() {
        let sites = three_sites();
        let v = SemiVariogram::exponential(0.0, 1.0, 50.0);
        let copula = GaussianCopula::from_variogram(&v, &sites).unwrap();
        let scenarios = copula.generate_wind_scenarios(&sites, 20);
        assert_eq!(scenarios.len(), 20);
        assert_eq!(scenarios[0].len(), sites.len());
    }

    #[test]
    fn test_wind_speeds_physically_plausible() {
        let sites = three_sites();
        let v = SemiVariogram::exponential(0.0, 1.0, 50.0);
        let copula = GaussianCopula::from_variogram(&v, &sites).unwrap();
        let scenarios = copula.generate_wind_scenarios(&sites, 50);
        for row in &scenarios {
            for &u in row {
                assert!(u > 0.0 && u < 30.0, "Wind speed = {:.2} m/s", u);
            }
        }
    }

    #[test]
    fn test_power_scenarios_non_negative() {
        let sites = three_sites();
        let v = SemiVariogram::exponential(0.0, 1.0, 50.0);
        let copula = GaussianCopula::from_variogram(&v, &sites).unwrap();
        let pow_sc = copula.generate_power_scenarios(&sites, 20);
        for row in &pow_sc {
            for &p in row {
                assert!(p >= 0.0, "Power should be non-negative: {}", p);
            }
        }
    }

    #[test]
    fn test_nearby_sites_correlated() {
        let close_sites = vec![
            SiteLocation {
                x_km: 0.0,
                y_km: 0.0,
                hub_height_m: 80.0,
                weibull_k: 2.0,
                weibull_c: 8.0,
                capacity_mw: 10.0,
            },
            SiteLocation {
                x_km: 2.0,
                y_km: 0.0,
                hub_height_m: 80.0,
                weibull_k: 2.0,
                weibull_c: 8.0,
                capacity_mw: 10.0,
            },
        ];
        let v = SemiVariogram::exponential(0.0, 1.0, 50.0); // long range
        let copula = GaussianCopula::from_variogram(&v, &close_sites).unwrap();
        let scenarios = copula.generate_wind_scenarios(&close_sites, 100);

        // With high correlation (close sites, long range), std of difference should be small
        let diffs: Vec<f64> = scenarios
            .iter()
            .map(|row| (row[0] - row[1]).abs())
            .collect();
        let mean_diff = diffs.iter().sum::<f64>() / diffs.len() as f64;
        // Sites 2km apart with range=50km should be very correlated
        assert!(
            mean_diff < 5.0,
            "Mean |u1-u2| = {:.3} m/s (should be small)",
            mean_diff
        );
    }

    #[test]
    fn test_scenario_stats() {
        let sites = three_sites();
        let v = SemiVariogram::exponential(0.0, 1.0, 50.0);
        let copula = GaussianCopula::from_variogram(&v, &sites).unwrap();
        let scenarios = copula.generate_wind_scenarios(&sites, 50);
        let stats = ScenarioStats::from_wind_scenarios(&scenarios, &sites);
        assert_eq!(stats.mean_speed.len(), sites.len());
        for &m in &stats.mean_speed {
            assert!(m > 0.0, "Mean speed should be positive: {}", m);
        }
        assert!(stats.mean_total_power_mw >= 0.0);
    }

    #[test]
    fn test_norm_cdf_symmetry() {
        assert!((norm_cdf(0.0) - 0.5).abs() < 1e-6);
        assert!((norm_cdf(1.0) + norm_cdf(-1.0) - 1.0).abs() < 1e-6);
    }
}
