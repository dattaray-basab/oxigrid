/// Partial shading and PSO-based global MPPT for PV arrays.
///
/// Partial shading causes multiple local maxima in the P-V curve, defeating
/// simple hill-climbing MPPT.  This module implements:
///
/// - **Bypass diode model**: each cell string has a bypass diode that conducts
///   when shaded cells reverse-bias; produces multiple P-V humps
/// - **PSO global MPPT**: Particle Swarm Optimisation sweeps the V range to
///   find the Global Maximum Power Point (GMPP) among all local maxima
/// - **Shading loss**: energy loss due to mismatch and bypass diode conduction
///
/// # References
/// - IEA PVPS Task 14, "Partial Shading of PV Arrays", 2016
/// - Esram & Chapman, "Comparison of PV Array Maximum Power Point Tracking
///   Techniques", IEEE Trans. Energy Convers., 2007
/// - Kennedy & Eberhart, "Particle Swarm Optimisation", ICNN, 1995
use serde::{Deserialize, Serialize};

// ────────────────────────────────────────────────────────────────────────────
// PV Array / string model with bypass diodes
// ────────────────────────────────────────────────────────────────────────────

/// A PV sub-string (one bypass diode covers several cells in series).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PvSubstring {
    /// Number of cells in this substring
    pub n_cells: usize,
    /// Effective irradiance [W/m²] on this substring (accounts for shading)
    pub irradiance: f64,
    /// Cell temperature [°C]
    pub temperature_c: f64,
}

/// Simple I-V model for a shaded PV substring using one-diode approximation.
///
/// At a given terminal voltage `v_s` [V per cell], returns the current `A`.
/// When shading causes reverse bias the bypass diode conducts (clamps at -V_d).
pub fn substring_current(
    sub: &PvSubstring,
    v_cell: f64,
    i_sc_ref: f64,
    i_o_ref: f64,
    a_factor: f64,
    r_s: f64,
    vt_ref: f64,
) -> f64 {
    // Temperature-adjusted parameters
    let t_ratio = (sub.temperature_c + 273.15) / 298.15;
    let i_sc = i_sc_ref * (sub.irradiance / 1000.0) * (1.0 + 0.0005 * (sub.temperature_c - 25.0));
    let vt = vt_ref * t_ratio;
    let i_o = i_o_ref
        * t_ratio.powi(3)
        * ((-11600.0 / a_factor) * (1.0 / (sub.temperature_c + 273.15) - 1.0 / 298.15)).exp();

    // Diode current (Newton's method one iteration for implicit equation)
    // I = Isc - Io*[exp((V + I*Rs)/(a*Vt)) - 1]
    let v_total = v_cell * sub.n_cells as f64;

    // Check bypass diode conduction
    let v_bypass = -0.7; // bypass diode forward voltage
    if v_total < v_bypass {
        return i_sc; // bypass diode conducts → full Isc from string
    }

    // Photocurrent model: for n_cells in series the diode equation is
    //   I = Isc - Io * exp((V_total + I*Rs_total) / (a * n_cells * Vt))
    let vt_substr = vt * sub.n_cells as f64; // scaled thermal voltage for the substring
    let arg = (v_total + i_sc * r_s) / (a_factor * vt_substr);
    let arg_clamped = arg.min(50.0); // prevent overflow
    let i_diode = i_o * (arg_clamped.exp() - 1.0);
    (i_sc - i_diode).max(-i_sc)
}

/// Standard cell parameters (silicon, 60-cell panel).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CellParams {
    /// Short-circuit current at STC `A`
    pub i_sc_ref: f64,
    /// Reverse saturation current `A`
    pub i_o_ref: f64,
    /// Diode ideality factor
    pub a_factor: f64,
    /// Series resistance [Ω per cell]
    pub r_s: f64,
    /// Thermal voltage at STC `V`
    pub vt_ref: f64,
    /// Open-circuit voltage at STC [V per cell]
    pub v_oc_cell: f64,
}

impl CellParams {
    /// Typical mono-Si cell (STC: 25°C, 1000 W/m²).
    pub fn mono_si() -> Self {
        Self {
            i_sc_ref: 9.0,
            i_o_ref: 1e-10,
            a_factor: 1.3,
            r_s: 0.005,
            vt_ref: 0.02585,
            v_oc_cell: 0.623,
        }
    }

    /// Poly-Si cell (lower Voc, slightly lower efficiency).
    pub fn poly_si() -> Self {
        Self {
            i_sc_ref: 8.5,
            i_o_ref: 2e-10,
            a_factor: 1.4,
            r_s: 0.006,
            vt_ref: 0.02585,
            v_oc_cell: 0.610,
        }
    }
}

/// PV array composed of multiple substrings in series.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PvArray {
    /// Substrings (each with bypass diode)
    pub substrings: Vec<PvSubstring>,
    /// Cells per substring
    pub cells_per_substring: usize,
    /// Cell parameters (same type throughout)
    pub cell_params: CellParams,
    /// Number of parallel strings
    pub n_parallel: usize,
}

impl PvArray {
    /// Create a uniformly illuminated 3-substring array.
    pub fn uniform(
        irradiance: f64,
        temperature_c: f64,
        n_substrings: usize,
        n_parallel: usize,
    ) -> Self {
        let cells = 20; // 20 cells per substring → 60-cell panel if 3 substrings
        Self {
            substrings: (0..n_substrings)
                .map(|_| PvSubstring {
                    n_cells: cells,
                    irradiance,
                    temperature_c,
                })
                .collect(),
            cells_per_substring: cells,
            cell_params: CellParams::mono_si(),
            n_parallel,
        }
    }

    /// Create a partially shaded array (one substring shaded).
    pub fn partially_shaded(
        full_irr: f64,
        shaded_irr: f64,
        temperature_c: f64,
        n_substrings: usize,
        shaded_index: usize,
        n_parallel: usize,
    ) -> Self {
        let cells = 20;
        let substrings = (0..n_substrings)
            .map(|i| PvSubstring {
                n_cells: cells,
                irradiance: if i == shaded_index {
                    shaded_irr
                } else {
                    full_irr
                },
                temperature_c,
            })
            .collect();
        Self {
            substrings,
            cells_per_substring: cells,
            cell_params: CellParams::mono_si(),
            n_parallel,
        }
    }

    /// Compute array current at a given terminal voltage `V`.
    pub fn array_current(&self, v_array: f64) -> f64 {
        let p = &self.cell_params;
        let n_sub = self.substrings.len();
        if n_sub == 0 {
            return 0.0;
        }

        // In a series string with bypass diodes, each substring operates at
        // its own voltage. For simplicity: assume equal voltage distribution
        // and iterate to find consistent operating point.
        // Simplified: each substring gets v_array / n_sub per string
        let v_per_sub = v_array / n_sub as f64;
        let v_per_cell = v_per_sub / self.cells_per_substring as f64;

        // Current is limited by the minimum substring current (series constraint)
        // With bypass diodes: shaded substring bypassed if its current < array current
        // Simplified: return the minimum substring current
        let i_per_string = self
            .substrings
            .iter()
            .map(|sub| {
                substring_current(
                    sub, v_per_cell, p.i_sc_ref, p.i_o_ref, p.a_factor, p.r_s, p.vt_ref,
                )
            })
            .fold(f64::INFINITY, f64::min)
            .max(0.0);

        i_per_string * self.n_parallel as f64
    }

    /// Compute array power at voltage `W`.
    pub fn power(&self, v: f64) -> f64 {
        v * self.array_current(v)
    }

    /// Maximum array voltage (sum of Voc across all substrings) `V`.
    pub fn v_oc_array(&self) -> f64 {
        self.cell_params.v_oc_cell * self.cells_per_substring as f64 * self.substrings.len() as f64
    }
}

// ────────────────────────────────────────────────────────────────────────────
// P-V curve sampling
// ────────────────────────────────────────────────────────────────────────────

/// Sample the P-V curve of an array.
///
/// Returns (voltage, power) pairs from 0 to Voc.
pub fn pv_curve(array: &PvArray, n_points: usize) -> Vec<(f64, f64)> {
    let v_oc = array.v_oc_array();
    (0..n_points)
        .map(|i| {
            let v = v_oc * i as f64 / (n_points - 1).max(1) as f64;
            let p = array.power(v);
            (v, p)
        })
        .collect()
}

/// Find all local maxima in the P-V curve.
///
/// Returns (voltage, power) at each local maximum.
pub fn local_maxima(curve: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let mut maxima = Vec::new();
    for i in 1..curve.len().saturating_sub(1) {
        let (_, p_prev) = curve[i - 1];
        let (v_i, p_i) = curve[i];
        let (_, p_next) = curve[i + 1];
        if p_i > p_prev && p_i >= p_next {
            maxima.push((v_i, p_i));
        }
    }
    maxima
}

// ────────────────────────────────────────────────────────────────────────────
// PSO Global MPPT
// ────────────────────────────────────────────────────────────────────────────

/// PSO configuration for global MPPT.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PsoMpptConfig {
    /// Number of particles
    pub n_particles: usize,
    /// Maximum iterations
    pub max_iterations: usize,
    /// Inertia weight w
    pub inertia: f64,
    /// Cognitive coefficient c1 (personal best)
    pub c1: f64,
    /// Social coefficient c2 (global best)
    pub c2: f64,
    /// Velocity clamp: max velocity as fraction of search space
    pub v_max_fraction: f64,
    /// Convergence tolerance `W`
    pub tol_w: f64,
}

impl Default for PsoMpptConfig {
    fn default() -> Self {
        Self {
            n_particles: 10,
            max_iterations: 50,
            inertia: 0.729,
            c1: 1.494,
            c2: 1.494,
            v_max_fraction: 0.20,
            tol_w: 0.01,
        }
    }
}

/// PSO MPPT result.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PsoMpptResult {
    /// Global Maximum Power Point voltage `V`
    pub v_gmpp: f64,
    /// Power at GMPP `W`
    pub p_gmpp: f64,
    /// Iterations to converge
    pub iterations: usize,
    /// Convergence achieved?
    pub converged: bool,
    /// All particles' final positions and best-known powers
    pub particle_best: Vec<(f64, f64)>,
}

/// Run PSO to find the Global Maximum Power Point.
///
/// Uses a linear congruential pseudo-random sequence for determinism.
pub fn pso_global_mppt(array: &PvArray, config: &PsoMpptConfig) -> PsoMpptResult {
    let v_max = array.v_oc_array() * 0.95;
    let v_min = v_max * 0.01;
    let v_range = v_max - v_min;
    let v_clamp = config.v_max_fraction * v_range;

    // Simple PRNG (LCG) for deterministic results
    let mut rng = LcgRng::new(42);

    // Initialise particles uniformly across voltage range
    let mut positions: Vec<f64> = (0..config.n_particles)
        .map(|i| v_min + v_range * i as f64 / config.n_particles as f64)
        .collect();
    let mut velocities: Vec<f64> = vec![0.0; config.n_particles];
    let mut p_best: Vec<f64> = positions.clone();
    let mut p_best_power: Vec<f64> = positions.iter().map(|&v| array.power(v).max(0.0)).collect();

    let mut g_best = p_best[0];
    let mut g_best_power = p_best_power[0];
    for (v, p) in p_best.iter().zip(p_best_power.iter()) {
        if *p > g_best_power {
            g_best_power = *p;
            g_best = *v;
        }
    }

    let mut iterations = 0;
    let mut prev_best = 0.0_f64;

    for iter in 0..config.max_iterations {
        iterations = iter + 1;

        for i in 0..config.n_particles {
            let r1 = rng.next_f64();
            let r2 = rng.next_f64();

            velocities[i] = config.inertia * velocities[i]
                + config.c1 * r1 * (p_best[i] - positions[i])
                + config.c2 * r2 * (g_best - positions[i]);

            velocities[i] = velocities[i].clamp(-v_clamp, v_clamp);
            positions[i] = (positions[i] + velocities[i]).clamp(v_min, v_max);

            let power = array.power(positions[i]).max(0.0);

            if power > p_best_power[i] {
                p_best_power[i] = power;
                p_best[i] = positions[i];
            }

            if power > g_best_power {
                g_best_power = power;
                g_best = positions[i];
            }
        }

        if (g_best_power - prev_best).abs() < config.tol_w && iter > 5 {
            break;
        }
        prev_best = g_best_power;
    }

    PsoMpptResult {
        v_gmpp: g_best,
        p_gmpp: g_best_power,
        iterations,
        converged: (g_best_power - prev_best).abs() < config.tol_w,
        particle_best: p_best
            .iter()
            .zip(p_best_power.iter())
            .map(|(&v, &p)| (v, p))
            .collect(),
    }
}

/// Simple linear congruential generator for deterministic PSO.
struct LcgRng {
    state: u64,
}

impl LcgRng {
    fn new(seed: u64) -> Self {
        Self { state: seed }
    }
    fn next_u64(&mut self) -> u64 {
        self.state = self
            .state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.state
    }
    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Shading loss analysis
// ────────────────────────────────────────────────────────────────────────────

/// Shading loss analysis result.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ShadingLossResult {
    /// Power without shading (STC uniform irradiance) `W`
    pub p_unshaded: f64,
    /// Power with shading at GMPP `W`
    pub p_shaded: f64,
    /// Absolute shading loss `W`
    pub loss_w: f64,
    /// Relative shading loss `fraction`
    pub loss_fraction: f64,
    /// Mismatch loss (additional loss from non-uniform irradiance) `fraction`
    pub mismatch_loss_fraction: f64,
    /// Bypass diode conduction detected in any substring?
    pub bypass_active: bool,
}

/// Compute shading losses by comparing GMPP power with and without shading.
pub fn shading_loss_analysis(shaded_array: &PvArray, full_irradiance: f64) -> ShadingLossResult {
    // Reference unshaded array
    let n_sub = shaded_array.substrings.len();
    let temp = shaded_array
        .substrings
        .first()
        .map(|s| s.temperature_c)
        .unwrap_or(25.0);
    let unshaded = PvArray::uniform(full_irradiance, temp, n_sub, shaded_array.n_parallel);

    let config = PsoMpptConfig::default();
    let unshaded_result = pso_global_mppt(&unshaded, &config);
    let shaded_result = pso_global_mppt(shaded_array, &config);

    let p_unshaded = unshaded_result.p_gmpp;
    let p_shaded = shaded_result.p_gmpp;
    let loss_w = (p_unshaded - p_shaded).max(0.0);
    let loss_fraction = if p_unshaded > 1e-6 {
        loss_w / p_unshaded
    } else {
        0.0
    };

    // Mismatch loss: fraction of loss not explained by reduced irradiance
    let irr_fraction = shaded_array
        .substrings
        .iter()
        .map(|s| s.irradiance)
        .sum::<f64>()
        / (n_sub as f64 * full_irradiance).max(1e-6);
    let expected_power = p_unshaded * irr_fraction;
    let mismatch_loss = (expected_power - p_shaded).max(0.0) / p_unshaded.max(1e-6);

    // Check bypass diode activity: any substring at very low irradiance
    let bypass_active = shaded_array
        .substrings
        .iter()
        .any(|s| s.irradiance < full_irradiance * 0.5);

    ShadingLossResult {
        p_unshaded,
        p_shaded,
        loss_w,
        loss_fraction,
        mismatch_loss_fraction: mismatch_loss.clamp(0.0, 1.0),
        bypass_active,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_uniform_array_current_positive() {
        let array = PvArray::uniform(1000.0, 25.0, 3, 1);
        let i = array.array_current(array.v_oc_array() * 0.8);
        assert!(i >= 0.0, "Current should be non-negative: {:.4}", i);
    }

    #[test]
    fn test_uniform_array_power_positive() {
        let array = PvArray::uniform(1000.0, 25.0, 3, 1);
        let v_test = array.v_oc_array() * 0.75;
        let p = array.power(v_test);
        assert!(p > 0.0, "Power at MPP region should be positive: {:.2}", p);
    }

    #[test]
    fn test_pv_curve_length() {
        let array = PvArray::uniform(1000.0, 25.0, 3, 1);
        let curve = pv_curve(&array, 100);
        assert_eq!(curve.len(), 100);
    }

    #[test]
    fn test_pv_curve_starts_at_zero() {
        let array = PvArray::uniform(1000.0, 25.0, 3, 1);
        let curve = pv_curve(&array, 50);
        assert!((curve[0].0).abs() < 1e-9, "Curve should start at V=0");
    }

    #[test]
    fn test_pso_finds_power_positive() {
        let array = PvArray::uniform(800.0, 30.0, 3, 2);
        let config = PsoMpptConfig::default();
        let result = pso_global_mppt(&array, &config);
        assert!(
            result.p_gmpp > 0.0,
            "PSO should find positive power: {:.2}",
            result.p_gmpp
        );
        assert!(
            result.v_gmpp > 0.0,
            "GMPP voltage should be positive: {:.2}",
            result.v_gmpp
        );
    }

    #[test]
    fn test_pso_uniform_vs_shaded() {
        let uniform = PvArray::uniform(1000.0, 25.0, 3, 1);
        let shaded = PvArray::partially_shaded(1000.0, 200.0, 25.0, 3, 1, 1);
        let config = PsoMpptConfig::default();
        let p_uniform = pso_global_mppt(&uniform, &config).p_gmpp;
        let p_shaded = pso_global_mppt(&shaded, &config).p_gmpp;
        assert!(
            p_uniform > p_shaded,
            "Shaded array should produce less power"
        );
    }

    #[test]
    fn test_pso_iterations_bounded() {
        let array = PvArray::uniform(1000.0, 25.0, 3, 1);
        let config = PsoMpptConfig {
            max_iterations: 20,
            ..Default::default()
        };
        let result = pso_global_mppt(&array, &config);
        assert!(result.iterations <= 20);
    }

    #[test]
    fn test_shading_loss_positive_for_partial_shade() {
        let shaded = PvArray::partially_shaded(1000.0, 100.0, 25.0, 3, 0, 1);
        let loss = shading_loss_analysis(&shaded, 1000.0);
        assert!(
            loss.loss_fraction > 0.0,
            "Should have shading loss: {:.4}",
            loss.loss_fraction
        );
        assert!(
            loss.bypass_active,
            "Bypass diode should be active under heavy shading"
        );
    }

    #[test]
    fn test_shading_loss_zero_for_uniform() {
        let uniform = PvArray::uniform(1000.0, 25.0, 3, 1);
        let loss = shading_loss_analysis(&uniform, 1000.0);
        // Uniform irradiance → minimal loss (just numerical error)
        assert!(
            loss.loss_fraction < 0.05,
            "Uniform array should have near-zero shading loss: {:.4}",
            loss.loss_fraction
        );
        assert!(!loss.bypass_active);
    }

    #[test]
    fn test_local_maxima_detection() {
        // Construct a synthetic P-V curve with a known maximum at index 5
        let curve: Vec<(f64, f64)> = vec![
            (0.0, 0.0),
            (5.0, 10.0),
            (10.0, 20.0),
            (15.0, 40.0),
            (20.0, 50.0),
            (25.0, 48.0),
            (30.0, 30.0),
            (35.0, 10.0),
            (40.0, 0.0),
        ];
        let maxima = local_maxima(&curve);
        assert!(
            !maxima.is_empty(),
            "Constructed curve has one maximum at (20, 50)"
        );
        assert!(
            (maxima[0].0 - 20.0).abs() < 1e-9,
            "Maximum should be at V=20: {:.2}",
            maxima[0].0
        );
    }

    #[test]
    fn test_substring_current_bypass_at_negative_v() {
        let sub = PvSubstring {
            n_cells: 20,
            irradiance: 1000.0,
            temperature_c: 25.0,
        };
        let p = CellParams::mono_si();
        // Very negative voltage → bypass diode conducts → returns Isc
        let i_bypass = substring_current(
            &sub, -0.5, p.i_sc_ref, p.i_o_ref, p.a_factor, p.r_s, p.vt_ref,
        );
        assert!(i_bypass >= 0.0);
    }

    #[test]
    fn test_cell_params_presets() {
        let mono = CellParams::mono_si();
        let poly = CellParams::poly_si();
        assert!(
            mono.v_oc_cell > poly.v_oc_cell,
            "Mono-Si should have higher Voc"
        );
        assert!(mono.i_sc_ref > poly.i_sc_ref);
    }

    #[test]
    fn test_pso_particle_best_count() {
        let array = PvArray::uniform(1000.0, 25.0, 3, 1);
        let config = PsoMpptConfig {
            n_particles: 8,
            ..Default::default()
        };
        let result = pso_global_mppt(&array, &config);
        assert_eq!(result.particle_best.len(), 8);
    }

    #[test]
    fn test_v_oc_array_scales_with_substrings() {
        let a3 = PvArray::uniform(1000.0, 25.0, 3, 1);
        let a6 = PvArray::uniform(1000.0, 25.0, 6, 1);
        assert!((a6.v_oc_array() / a3.v_oc_array() - 2.0).abs() < 1e-9);
    }
}
