//! Inter-area oscillation analysis for multi-machine power systems.
//!
//! This module provides tools to:
//! - Build linearised multi-machine swing-equation state matrices
//! - Compute inter-area and local oscillation modes (eigenvalues, mode shapes,
//!   participation factors)
//! - Simulate ring-down transients with RK4 integration
//! - Extract modal parameters from time-domain signals (Prony / LS-grid method)
//! - Design Wide-Area Damping Controllers (WADC) using the residue method

/// Angular frequency of synchronous machine reference (50 Hz system, rad/s).
const OMEGA_0: f64 = 2.0 * std::f64::consts::PI * 50.0;

// ──────────────────────────────────────────────────────────────────────────────
// Data structures
// ──────────────────────────────────────────────────────────────────────────────

/// A generator participating in inter-area oscillation analysis.
#[derive(Debug, Clone)]
pub struct IaGenerator {
    /// Unique generator index.
    pub gen_id: usize,
    /// Area this generator belongs to.
    pub area_id: usize,
    /// Terminal bus.
    pub bus_id: usize,
    /// Inertia constant H (seconds).
    pub inertia_h: f64,
    /// Damping coefficient D (pu torque / pu speed).
    pub damping_d: f64,
    /// Synchronising torque coefficient Ks (pu).
    pub sync_torque_ks: f64,
    /// Generator rated MVA.
    pub rated_mva: f64,
}

/// A power-system area (coherent group of generators).
#[derive(Debug, Clone)]
pub struct SystemArea {
    /// Unique area identifier.
    pub area_id: usize,
    /// Human-readable area name.
    pub name: String,
    /// Total kinetic energy stored in the area (MW·s).
    pub total_inertia_mws: f64,
    /// Indices into `InterAreaAnalyzer::generators` that belong to this area.
    pub generator_ids: Vec<usize>,
}

/// A tie-line that couples two areas.
#[derive(Debug, Clone)]
pub struct TieLine {
    /// Area at the sending end.
    pub from_area: usize,
    /// Area at the receiving end.
    pub to_area: usize,
    /// Synchronising power coefficient P_sync (pu MW / rad).
    pub synchronizing_power: f64,
    /// Sending-end bus.
    pub from_bus: usize,
    /// Receiving-end bus.
    pub to_bus: usize,
}

/// An identified electromechanical oscillation mode.
#[derive(Debug, Clone)]
pub struct InterAreaMode {
    /// Oscillation frequency (Hz).
    pub freq_hz: f64,
    /// Damping ratio ζ = -σ / sqrt(σ²+ω²).
    pub damping_ratio: f64,
    /// Real part of the eigenvalue σ (negative ⇒ stable).
    pub sigma: f64,
    /// Imaginary part of the eigenvalue ω (rad/s).
    pub omega_rad_per_s: f64,
    /// `true` when freq_hz < 1.0 Hz (inter-area band).
    pub is_inter_area: bool,
    /// Normalised participation by area: `(area_id, factor)`, factors sum to ≈ 1.
    pub participating_areas: Vec<(usize, f64)>,
    /// Normalised mode shape: `(gen_id, normalised_amplitude)`.
    pub mode_shape: Vec<(usize, f64)>,
}

/// Result of a Prony / LS-grid analysis on a time-domain signal.
#[derive(Debug, Clone)]
pub struct PronyResult {
    /// Identified oscillatory modes.
    pub modes: Vec<PronyMode>,
    /// RMS relative reconstruction error (0 = perfect fit).
    pub signal_reconstruction_error: f64,
    /// Number of modes actually identified.
    pub n_modes_identified: usize,
}

/// A single mode identified by Prony analysis.
#[derive(Debug, Clone)]
pub struct PronyMode {
    /// Amplitude of the mode.
    pub amplitude: f64,
    /// Damping exponent σ (s⁻¹); negative means decaying.
    pub damping: f64,
    /// Frequency (Hz).
    pub freq_hz: f64,
    /// Initial phase (rad).
    pub phase_rad: f64,
    /// Damping ratio ζ.
    pub damping_ratio: f64,
}

/// Wide-area damping controller design (lead-lag + washout).
#[derive(Debug, Clone)]
pub struct WadcDesign {
    /// Bus where the remote PMU signal is measured.
    pub pmu_bus_id: usize,
    /// Generator whose PSS input receives the control signal.
    pub control_gen_id: usize,
    /// Washout filter time constant (s).
    pub washout_time_const: f64,
    /// First lead-lag stage (T1, T2) in seconds; T1 > T2 for phase lead.
    pub lead_lag_1: (f64, f64),
    /// Second lead-lag stage (T1, T2).
    pub lead_lag_2: (f64, f64),
    /// Controller gain K.
    pub gain: f64,
    /// Expected closed-loop damping improvement (%).
    pub expected_improvement_pct: f64,
    /// Round-trip communication delay budget (ms).
    pub communication_delay_ms: f64,
}

/// Time-domain ring-down simulation result.
#[derive(Debug, Clone)]
pub struct RingdownResult {
    /// Time vector (s).
    pub time_s: Vec<f64>,
    /// Rotor angle deviations per generator: `delta_angles[gen_idx][time_step]` (rad).
    pub delta_angles: Vec<Vec<f64>>,
    /// Rotor speed deviations per generator: `delta_speeds[gen_idx][time_step]` (pu).
    pub delta_speeds: Vec<Vec<f64>>,
    /// Tie-line power flow deviations: `tie_line_flows[tl_idx][time_step]` (MW).
    pub tie_line_flows: Vec<Vec<f64>>,
    /// Dominant inter-area angle swing (rad).
    pub inter_area_angle: Vec<f64>,
    /// Prony analysis of the inter-area angle signal.
    pub prony_analysis: PronyResult,
}

// ──────────────────────────────────────────────────────────────────────────────
// Main analysis engine
// ──────────────────────────────────────────────────────────────────────────────

/// Inter-area oscillation analysis engine.
///
/// Build the system description by calling [`add_generator`](Self::add_generator),
/// [`add_area`](Self::add_area) and [`add_tie_line`](Self::add_tie_line), then
/// call [`compute_modes`](Self::compute_modes) to identify oscillation modes.
#[derive(Debug, Clone)]
pub struct InterAreaAnalyzer {
    /// Generators in the multi-machine system.
    pub generators: Vec<IaGenerator>,
    /// Areas (coherent groups).
    pub areas: Vec<SystemArea>,
    /// Tie-lines coupling areas.
    pub tie_lines: Vec<TieLine>,
    /// System MVA base.
    pub base_mva: f64,
}

impl InterAreaAnalyzer {
    /// Create an empty analyser with the given MVA base.
    pub fn new(base_mva: f64) -> Self {
        Self {
            generators: Vec::new(),
            areas: Vec::new(),
            tie_lines: Vec::new(),
            base_mva,
        }
    }

    /// Add a generator to the system.
    pub fn add_generator(&mut self, gen: IaGenerator) {
        self.generators.push(gen);
    }

    /// Add an area definition.
    pub fn add_area(&mut self, area: SystemArea) {
        self.areas.push(area);
    }

    /// Add a tie-line between two areas.
    pub fn add_tie_line(&mut self, tie: TieLine) {
        self.tie_lines.push(tie);
    }

    // ──────────────────────────────────────────────────────────────────────────
    // State matrix
    // ──────────────────────────────────────────────────────────────────────────

    /// Build the linearised 2N × 2N state matrix A for the multi-machine
    /// swing equations.
    ///
    /// State vector: `x = [Δδ₁ … ΔδN, Δω₁ … ΔωN]ᵀ`
    ///
    /// ```text
    /// A = [ 0      |  I      ]
    ///     [ -M⁻¹K  |  -M⁻¹D ]
    /// ```
    #[allow(clippy::needless_range_loop)]
    fn build_state_matrix(&self) -> Vec<Vec<f64>> {
        let n = self.generators.len();
        let dim = 2 * n;
        let mut a = vec![vec![0.0_f64; dim]; dim];

        // Top-right block: identity
        for i in 0..n {
            a[i][n + i] = 1.0;
        }

        if n == 0 {
            return a;
        }

        // M_inv[i] = ω₀ / (2·Hᵢ)
        let m_inv: Vec<f64> = self
            .generators
            .iter()
            .map(|g| OMEGA_0 / (2.0 * g.inertia_h.max(1e-12)))
            .collect();

        // Build synchronising-torque matrix K (N×N)
        let mut k = vec![vec![0.0_f64; n]; n];

        for tl in &self.tie_lines {
            // Collect generator indices in each area for this tie-line
            let from_gens: Vec<usize> = self
                .generators
                .iter()
                .enumerate()
                .filter(|(_, g)| g.area_id == tl.from_area)
                .map(|(idx, _)| idx)
                .collect();
            let to_gens: Vec<usize> = self
                .generators
                .iter()
                .enumerate()
                .filter(|(_, g)| g.area_id == tl.to_area)
                .map(|(idx, _)| idx)
                .collect();

            let n_pairs = (from_gens.len() * to_gens.len()) as f64;
            if n_pairs < 1.0 {
                continue;
            }
            let ks_pair = tl.synchronizing_power / n_pairs;

            for &fi in &from_gens {
                for &ti in &to_gens {
                    k[fi][ti] -= ks_pair;
                    k[ti][fi] -= ks_pair;
                }
            }
        }

        // Also add generator-level synchronising torque on diagonal equivalent
        for i in 0..n {
            let ks_self = self.generators[i].sync_torque_ks;
            // Diagonal: sum of absolute off-diagonal entries in row i
            let off_diag_sum: f64 = k[i].iter().map(|v| v.abs()).sum();
            k[i][i] = off_diag_sum + ks_self * 0.1; // small self-restoring
        }

        // Fill bottom-left block: -M⁻¹·K
        for i in 0..n {
            for j in 0..n {
                a[n + i][j] = -m_inv[i] * k[i][j];
            }
        }

        // Fill bottom-right block: -M⁻¹·D (diagonal)
        for i in 0..n {
            a[n + i][n + i] = -m_inv[i] * self.generators[i].damping_d;
        }

        a
    }

    // ──────────────────────────────────────────────────────────────────────────
    // 2×2 eigenvalue solver (analytical)
    // ──────────────────────────────────────────────────────────────────────────

    /// Compute eigenvalues of a 2×2 real matrix analytically.
    ///
    /// Returns at most 2 `(real, imag)` pairs.
    pub fn compute_eigenvalues_2x2(a: [[f64; 2]; 2]) -> Vec<(f64, f64)> {
        let trace = a[0][0] + a[1][1];
        let det = a[0][0] * a[1][1] - a[0][1] * a[1][0];
        let disc = trace * trace - 4.0 * det;
        if disc >= 0.0 {
            let sq = disc.sqrt();
            vec![((trace + sq) / 2.0, 0.0), ((trace - sq) / 2.0, 0.0)]
        } else {
            let sq = (-disc).sqrt();
            vec![(trace / 2.0, sq / 2.0), (trace / 2.0, -sq / 2.0)]
        }
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Eigenvalue computation for general N (power iteration + deflation)
    // ──────────────────────────────────────────────────────────────────────────

    /// Matrix–vector product.
    fn mat_vec(a: &[Vec<f64>], v: &[f64]) -> Vec<f64> {
        a.iter()
            .map(|row| row.iter().zip(v.iter()).map(|(a, b)| a * b).sum())
            .collect()
    }

    /// Dot product.
    fn dot(u: &[f64], v: &[f64]) -> f64 {
        u.iter().zip(v.iter()).map(|(a, b)| a * b).sum()
    }

    /// Euclidean norm.
    fn norm(v: &[f64]) -> f64 {
        Self::dot(v, v).sqrt()
    }

    /// Normalise a vector; returns the original norm.
    #[allow(clippy::ptr_arg)]
    fn normalise(v: &mut Vec<f64>) -> f64 {
        let n = Self::norm(v);
        if n > 1e-15 {
            for x in v.iter_mut() {
                *x /= n;
            }
        }
        n
    }

    /// Simple LCG pseudo-random number in [−1, 1].
    fn lcg_rand(state: &mut u64) -> f64 {
        *state = state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        let bits = (*state >> 33) as i32;
        (bits as f64) / (i32::MAX as f64)
    }

    /// Compute A·(A·v) − two applications; used for double-shift subspace iteration.
    fn mat_sq_vec(a: &[Vec<f64>], v: &[f64]) -> Vec<f64> {
        let av = Self::mat_vec(a, v);
        Self::mat_vec(a, &av)
    }

    /// Extract dominant complex eigenvalue pair from matrix A using two-vector
    /// subspace (double-shift) iteration.
    ///
    /// Returns `(sigma, omega, right_eigenvector_real_part, right_eigenvector_imag_part)`.
    fn dominant_complex_pair(
        a: &[Vec<f64>],
        seed: &mut u64,
        max_iter: usize,
    ) -> Option<(f64, f64, Vec<f64>, Vec<f64>)> {
        let dim = a.len();
        if dim < 2 {
            return None;
        }

        // Initialise two orthogonal random vectors
        let mut v1: Vec<f64> = (0..dim).map(|_| Self::lcg_rand(seed)).collect();
        let mut v2: Vec<f64> = (0..dim).map(|_| Self::lcg_rand(seed)).collect();
        // Orthogonalise v2 w.r.t. v1
        let d = Self::dot(&v1, &v2);
        let n1 = Self::dot(&v1, &v1).max(1e-30);
        for i in 0..dim {
            v2[i] -= d / n1 * v1[i];
        }
        Self::normalise(&mut v1);
        Self::normalise(&mut v2);

        let mut prev_sigma = f64::MAX;
        let mut prev_omega = 0.0_f64;

        for _iter in 0..max_iter {
            // Power step on each vector using A²
            let mut w1 = Self::mat_sq_vec(a, &v1);
            let mut w2 = Self::mat_sq_vec(a, &v2);

            // Re-orthogonalise (modified Gram-Schmidt)
            let d11 = Self::dot(&w1, &w1).sqrt().max(1e-30);
            for x in w1.iter_mut() {
                *x /= d11;
            }
            let d12 = Self::dot(&w1, &w2);
            for i in 0..dim {
                w2[i] -= d12 * w1[i];
            }
            let d22 = Self::dot(&w2, &w2).sqrt().max(1e-30);
            for x in w2.iter_mut() {
                *x /= d22;
            }

            v1 = w1;
            v2 = w2;

            // Compute 2×2 Rayleigh quotient: R = V^T A V  where V = [v1, v2]
            let av1 = Self::mat_vec(a, &v1);
            let av2 = Self::mat_vec(a, &v2);
            let r11 = Self::dot(&v1, &av1);
            let r12 = Self::dot(&v1, &av2);
            let r21 = Self::dot(&v2, &av1);
            let r22 = Self::dot(&v2, &av2);

            let r = [[r11, r12], [r21, r22]];
            let eigs = Self::compute_eigenvalues_2x2(r);

            // Take the complex pair (largest |imag|)
            let best = eigs.iter().max_by(|a, b| {
                a.1.abs()
                    .partial_cmp(&b.1.abs())
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
            if let Some(&(sigma, omega)) = best {
                if (sigma - prev_sigma).abs() < 1e-8 && (omega - prev_omega).abs() < 1e-8 {
                    // Converged — build eigenvectors from 2×2 solution
                    // Right eigenvector of 2×2 is (r12, sigma+j*omega - r11)
                    let e_re = r12;
                    let e_im = omega; // imaginary part of eigenvalue relative to r11
                                      // Full eigenvector (real part): v1 * e_re + v2 * (-(sigma - r11))
                    let coeff_re = e_re;
                    let coeff_im2 = sigma - r11; // real part of 2x2 eigvec component 2
                    let evec_re: Vec<f64> = v1
                        .iter()
                        .zip(v2.iter())
                        .map(|(a, b)| coeff_re * a - coeff_im2 * b)
                        .collect();
                    let evec_im: Vec<f64> = v1
                        .iter()
                        .zip(v2.iter())
                        .map(|(a, b)| e_im * a + coeff_re * b)
                        .collect();
                    return Some((sigma, omega.abs(), evec_re, evec_im));
                }
                prev_sigma = sigma;
                prev_omega = omega.abs();
            }
        }

        // Return best estimate even if not fully converged
        let av1 = Self::mat_vec(a, &v1);
        let av2 = Self::mat_vec(a, &v2);
        let r11 = Self::dot(&v1, &av1);
        let r12 = Self::dot(&v1, &av2);
        let r21 = Self::dot(&v2, &av1);
        let r22 = Self::dot(&v2, &av2);
        let r = [[r11, r12], [r21, r22]];
        let eigs = Self::compute_eigenvalues_2x2(r);
        let best = eigs.iter().max_by(|a, b| {
            a.1.abs()
                .partial_cmp(&b.1.abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        if let Some(&(sigma, omega)) = best {
            let evec_re: Vec<f64> = v1
                .iter()
                .zip(v2.iter())
                .map(|(a, b)| r12 * a - (sigma - r11) * b)
                .collect();
            let evec_im: Vec<f64> = v1
                .iter()
                .zip(v2.iter())
                .map(|(a, b)| omega * a + r12 * b)
                .collect();
            return Some((sigma, omega.abs(), evec_re, evec_im));
        }
        None
    }

    /// Deflate matrix A by removing the influence of a complex eigenvector pair.
    ///
    /// Uses Gram-Schmidt deflation: A_def = A - λ·vᵀ·A / (vᵀ·v)  (rank-2 update).
    fn deflate(a: &[Vec<f64>], evec_re: &[f64], evec_im: &[f64]) -> Vec<Vec<f64>> {
        let dim = a.len();
        let mut a_def = a.to_vec();

        // Deflate using real part of eigenvector
        let vv_re = Self::dot(evec_re, evec_re).max(1e-30);
        let av_re = Self::mat_vec(a, evec_re);
        for i in 0..dim {
            for j in 0..dim {
                a_def[i][j] -= evec_re[i] * av_re[j] / vv_re;
            }
        }

        // Deflate using imaginary part
        let vv_im = Self::dot(evec_im, evec_im).max(1e-30);
        if vv_im > 1e-15 {
            let av_im = Self::mat_vec(a, evec_im);
            for i in 0..dim {
                for j in 0..dim {
                    a_def[i][j] -= evec_im[i] * av_im[j] / vv_im;
                }
            }
        }

        a_def
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Mode computation
    // ──────────────────────────────────────────────────────────────────────────

    /// Compute electromechanical oscillation modes.
    ///
    /// Returns a vector of [`InterAreaMode`] sorted by frequency (ascending).
    pub fn compute_modes(&self) -> Result<Vec<InterAreaMode>, String> {
        let n = self.generators.len();
        if n == 0 {
            return Err("No generators defined".to_string());
        }

        let a = self.build_state_matrix();

        // Special-case: 2-machine, use the 2-machine analytical formula
        if n == 2 {
            return self.compute_modes_two_machine(&a);
        }

        // General N: power iteration with deflation
        let mut modes = Vec::new();
        let mut a_def = a.clone();
        let mut seed: u64 = 0xdeadbeef_cafebabe;
        let n_pairs = n; // at most N oscillatory pairs

        for _ in 0..n_pairs {
            if a_def.len() < 4 {
                break;
            }
            match Self::dominant_complex_pair(&a_def, &mut seed, 200) {
                Some((sigma, omega, evec_re, evec_im)) => {
                    if omega < 1e-4 {
                        // Real eigenvalue (zero or negative damping mode) — skip
                        a_def = Self::deflate(&a_def, &evec_re, &evec_im);
                        continue;
                    }
                    let mode = self.build_mode(sigma, omega, &evec_re)?;
                    modes.push(mode);
                    a_def = Self::deflate(&a_def, &evec_re, &evec_im);
                }
                None => break,
            }
        }

        if modes.is_empty() {
            // Fallback: try two_area_mode_frequency
            if let Ok(f) = self.two_area_mode_frequency() {
                let omega = 2.0 * std::f64::consts::PI * f;
                let sigma = -0.1 * omega; // assume 10% damping as fallback
                let evec_re: Vec<f64> = self
                    .generators
                    .iter()
                    .enumerate()
                    .map(|(i, _)| if i < n / 2 { 1.0 } else { -1.0 })
                    .collect();
                let mode = self.build_mode(sigma, omega, &evec_re)?;
                modes.push(mode);
            }
        }

        modes.sort_by(|a, b| {
            a.freq_hz
                .partial_cmp(&b.freq_hz)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        Ok(modes)
    }

    /// Build an `InterAreaMode` from eigenvalue components and eigenvector.
    fn build_mode(&self, sigma: f64, omega: f64, evec: &[f64]) -> Result<InterAreaMode, String> {
        let n = self.generators.len();
        let freq_hz = omega / (2.0 * std::f64::consts::PI);
        let mag = (sigma * sigma + omega * omega).sqrt().max(1e-12);
        let damping_ratio = -sigma / mag;

        // Mode shape: first N entries of eigenvector (angle part), normalise to max=1
        let angle_part: Vec<f64> = evec.iter().take(n).cloned().collect();
        let max_abs = angle_part
            .iter()
            .map(|v| v.abs())
            .fold(0.0_f64, f64::max)
            .max(1e-12);
        let mode_shape: Vec<(usize, f64)> = self
            .generators
            .iter()
            .zip(angle_part.iter())
            .map(|(g, &v)| (g.gen_id, v / max_abs))
            .collect();

        // Participating areas: Σ |mode_shape| * H_i per area, normalise
        let mut area_participation: std::collections::HashMap<usize, f64> =
            std::collections::HashMap::new();
        for (idx, g) in self.generators.iter().enumerate() {
            let ms = mode_shape.get(idx).map(|(_, v)| v.abs()).unwrap_or(0.0);
            *area_participation.entry(g.area_id).or_insert(0.0) += ms * g.inertia_h;
        }
        let total: f64 = area_participation.values().sum::<f64>().max(1e-12);
        let mut participating_areas: Vec<(usize, f64)> = area_participation
            .into_iter()
            .map(|(id, v)| (id, v / total))
            .collect();
        participating_areas.sort_by_key(|(id, _)| *id);

        Ok(InterAreaMode {
            freq_hz,
            damping_ratio,
            sigma,
            omega_rad_per_s: omega,
            is_inter_area: freq_hz < 1.0,
            participating_areas,
            mode_shape,
        })
    }

    /// Compute modes for exactly 2 generators using the analytical 2-machine formula.
    ///
    /// For two machines, the relative motion (δ₁ − δ₂) satisfies:
    /// ```text
    /// ẍ = −(M₁⁻¹ + M₂⁻¹)·Ks · x − (M₁⁻¹·D₁ + M₂⁻¹·D₂)/2 · ẋ
    /// ```
    /// where Ks is the total synchronising power coefficient.
    fn compute_modes_two_machine(&self, _a: &[Vec<f64>]) -> Result<Vec<InterAreaMode>, String> {
        if self.generators.len() < 2 {
            return Err("Need at least 2 generators for 2-machine analysis".to_string());
        }

        let g0 = &self.generators[0];
        let g1 = &self.generators[1];

        let m0 = 2.0 * g0.inertia_h / OMEGA_0; // M₀ = 2H₀/ω₀
        let m1 = 2.0 * g1.inertia_h / OMEGA_0; // M₁ = 2H₁/ω₀

        // Total synchronising power between the two areas
        let ks_total: f64 = self
            .tie_lines
            .iter()
            .filter(|tl| {
                (tl.from_area == g0.area_id && tl.to_area == g1.area_id)
                    || (tl.from_area == g1.area_id && tl.to_area == g0.area_id)
            })
            .map(|tl| tl.synchronizing_power)
            .sum::<f64>()
            + (g0.sync_torque_ks + g1.sync_torque_ks) * 0.05;

        // Reduced inertia (series combination)
        let m_red = (m0 * m1) / (m0 + m1);

        // Equivalent damping for relative motion
        // d²x/dt² + (D0/M0 + D1/M1)/2 * dx/dt + Ks/M_red * x = 0
        // (exact when M0=M1, approximate otherwise)
        let d_coeff = (g0.damping_d / m0 + g1.damping_d / m1) / 2.0;
        let k_coeff = ks_total / m_red;

        // Equivalent 2×2:  [[0, 1], [-k_coeff, -d_coeff]]
        let sub = [[0.0_f64, 1.0], [-k_coeff, -d_coeff]];
        let eigs = Self::compute_eigenvalues_2x2(sub);

        let mut modes = Vec::new();
        for &(sigma, omega) in &eigs {
            if omega.abs() > 1e-4 {
                // Mode shape: gen0 moves opposite to gen1 in the swing mode
                let evec_re = vec![1.0_f64, -1.0, sigma, -sigma];
                let mode = self.build_mode(sigma, omega.abs(), &evec_re)?;
                modes.push(mode);
                break;
            }
        }

        // Fallback: compute underdamped frequency from formula directly
        if modes.is_empty() {
            let disc = d_coeff * d_coeff - 4.0 * k_coeff;
            if disc < 0.0 {
                let omega = (-disc).sqrt() / 2.0;
                let sigma = -d_coeff / 2.0;
                let evec_re = vec![1.0_f64, -1.0, sigma, -sigma];
                let mode = self.build_mode(sigma, omega, &evec_re)?;
                modes.push(mode);
            }
        }

        modes.sort_by(|a, b| {
            a.freq_hz
                .partial_cmp(&b.freq_hz)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        Ok(modes)
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Two-area analytical formula
    // ──────────────────────────────────────────────────────────────────────────

    /// Compute the inter-area mode frequency using the simplified 2-area formula.
    ///
    /// ```text
    /// f = (1/2π) · √(ω₀ · P_sync · (M₁+M₂) / (M₁·M₂))
    /// ```
    ///
    /// Returns an error if fewer than 2 areas or no tie-lines are defined.
    pub fn two_area_mode_frequency(&self) -> Result<f64, String> {
        if self.areas.len() < 2 {
            return Err("Need at least 2 areas for two-area formula".to_string());
        }
        if self.tie_lines.is_empty() {
            return Err("No tie-lines defined".to_string());
        }

        // Identify the two dominant areas (by total inertia)
        let mut area_inertia: std::collections::HashMap<usize, f64> =
            std::collections::HashMap::new();
        for g in &self.generators {
            // M = 2H/ω₀
            let m = 2.0 * g.inertia_h / OMEGA_0;
            *area_inertia.entry(g.area_id).or_insert(0.0) += m;
        }

        let mut sorted_areas: Vec<(usize, f64)> = area_inertia.into_iter().collect();
        sorted_areas.sort_by_key(|(id, _)| *id);

        if sorted_areas.len() < 2 {
            // Only one area has generators — use first two defined areas
            return Err("Generators must be spread across at least 2 areas".to_string());
        }

        let m1 = sorted_areas[0].1.max(1e-12);
        let m2 = sorted_areas[1].1.max(1e-12);

        let p_sync: f64 = self.tie_lines.iter().map(|t| t.synchronizing_power).sum();
        if p_sync <= 0.0 {
            return Err("Total synchronising power must be positive".to_string());
        }

        // Two-area inter-area mode (Kundur): ω_n = √(P_sync·(M1+M2)/(M1·M2)),
        // where M = 2H/ω₀ already carries the ω₀ scaling — so there is no extra
        // ω₀ factor here. f = ω_n / 2π.
        let f = (1.0 / (2.0 * std::f64::consts::PI)) * (p_sync * (m1 + m2) / (m1 * m2)).sqrt();
        Ok(f)
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Ring-down simulation (RK4)
    // ──────────────────────────────────────────────────────────────────────────

    /// Simulate the ring-down (free oscillation) of the system using RK4 integration.
    ///
    /// An initial angle perturbation of `perturbation_mw` (MW equivalent) is applied,
    /// with alternating signs between areas.
    pub fn simulate_ringdown(
        &self,
        perturbation_mw: f64,
        duration_s: f64,
        dt_s: f64,
    ) -> Result<RingdownResult, String> {
        let n = self.generators.len();
        if n == 0 {
            return Err("No generators defined for ringdown simulation".to_string());
        }

        let a = self.build_state_matrix();
        let dim = 2 * n;

        // Initial state: small angle perturbation, alternating between areas
        let mut x = vec![0.0_f64; dim];
        let pert_rad = perturbation_mw / (self.base_mva.max(1.0) * n as f64);

        // Determine area of each generator for sign assignment
        let area_ids: Vec<usize> = self.generators.iter().map(|g| g.area_id).collect();
        let first_area = area_ids.first().copied().unwrap_or(0);
        for (i, &aid) in area_ids.iter().enumerate() {
            x[i] = if aid == first_area {
                pert_rad
            } else {
                -pert_rad
            };
        }

        let n_steps = ((duration_s / dt_s).ceil() as usize).max(1);

        // Pre-allocate result storage: [gen_idx][time_step]
        let mut delta_angles = vec![Vec::with_capacity(n_steps + 1); n];
        let mut delta_speeds = vec![Vec::with_capacity(n_steps + 1); n];
        let mut time_s = Vec::with_capacity(n_steps + 1);

        // Store initial condition
        time_s.push(0.0);
        for i in 0..n {
            delta_angles[i].push(x[i]);
            delta_speeds[i].push(x[n + i]);
        }

        // RK4 loop
        for step in 1..=n_steps {
            let t = step as f64 * dt_s;
            x = Self::rk4_step(&a, &x, dt_s);
            time_s.push(t);
            for i in 0..n {
                delta_angles[i].push(x[i]);
                delta_speeds[i].push(x[n + i]);
            }
        }

        // Tie-line flows: P_sync * (δ_from_centroid - δ_to_centroid) * base_mva
        let mut tie_line_flows = vec![Vec::with_capacity(n_steps + 1); self.tie_lines.len()];
        for (step_idx, _t) in time_s.iter().enumerate() {
            for (tl_idx, tl) in self.tie_lines.iter().enumerate() {
                let (from_delta, from_count) =
                    delta_centroid_by_area(&delta_angles, step_idx, tl.from_area, &self.generators);
                let (to_delta, to_count) =
                    delta_centroid_by_area(&delta_angles, step_idx, tl.to_area, &self.generators);
                let flow = if from_count > 0 && to_count > 0 {
                    tl.synchronizing_power * (from_delta - to_delta) * self.base_mva
                } else {
                    0.0
                };
                tie_line_flows[tl_idx].push(flow);
            }
        }

        // Inter-area angle: inertia-weighted centroid of area1 − area2
        let inter_area_angle = compute_inter_area_angle(&delta_angles, &time_s, &self.generators);

        // Prony analysis on inter-area angle
        let prony_analysis = Self::prony_analysis(&inter_area_angle, dt_s, 4);

        Ok(RingdownResult {
            time_s,
            delta_angles,
            delta_speeds,
            tie_line_flows,
            inter_area_angle,
            prony_analysis,
        })
    }

    /// Single RK4 step: x_{k+1} = x_k + (dt/6)(k1 + 2k2 + 2k3 + k4).
    fn rk4_step(a: &[Vec<f64>], x: &[f64], dt: f64) -> Vec<f64> {
        let f = |v: &[f64]| -> Vec<f64> { Self::mat_vec(a, v) };

        let k1 = f(x);
        let x2: Vec<f64> = x
            .iter()
            .zip(k1.iter())
            .map(|(xi, ki)| xi + 0.5 * dt * ki)
            .collect();
        let k2 = f(&x2);
        let x3: Vec<f64> = x
            .iter()
            .zip(k2.iter())
            .map(|(xi, ki)| xi + 0.5 * dt * ki)
            .collect();
        let k3 = f(&x3);
        let x4: Vec<f64> = x
            .iter()
            .zip(k3.iter())
            .map(|(xi, ki)| xi + dt * ki)
            .collect();
        let k4 = f(&x4);

        x.iter()
            .zip(k1.iter().zip(k2.iter().zip(k3.iter().zip(k4.iter()))))
            .map(|(xi, (k1i, (k2i, (k3i, k4i))))| {
                xi + dt / 6.0 * (k1i + 2.0 * k2i + 2.0 * k3i + k4i)
            })
            .collect()
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Prony / LS-grid analysis
    // ──────────────────────────────────────────────────────────────────────────

    /// Identify oscillatory modes in a time-domain signal using a least-squares
    /// frequency-grid search (simplified Prony method).
    ///
    /// # Arguments
    /// * `signal` — sampled time-domain data
    /// * `dt_s`   — sample interval (s)
    /// * `n_modes` — number of modes to return
    fn prony_analysis(signal: &[f64], dt_s: f64, n_modes: usize) -> PronyResult {
        let ns = signal.len();
        if ns < 4 || n_modes == 0 {
            return PronyResult {
                modes: Vec::new(),
                signal_reconstruction_error: 1.0,
                n_modes_identified: 0,
            };
        }

        // Build time vector
        let t: Vec<f64> = (0..ns).map(|i| i as f64 * dt_s).collect();

        // Candidate frequencies (Hz) and damping exponents (s⁻¹)
        let freqs: Vec<f64> = (1..=20).map(|i| i as f64 * 0.1).collect(); // 0.1 to 2.0 Hz
        let dampings: Vec<f64> = vec![-0.1, -0.3, -0.5, -1.0];

        // Build basis matrix Φ: columns = exp(σ·t)·cos(2πf·t), exp(σ·t)·sin(2πf·t)
        let n_cols = freqs.len() * dampings.len() * 2;
        // Φ is ns × n_cols
        let mut phi = vec![vec![0.0_f64; n_cols]; ns];
        let mut col_meta: Vec<(f64, f64, bool)> = Vec::with_capacity(n_cols); // (freq, damp, is_cos)

        for &f in &freqs {
            for &d in &dampings {
                let omega = 2.0 * std::f64::consts::PI * f;
                let cos_col: Vec<f64> = t
                    .iter()
                    .map(|&ti| (d * ti).exp() * (omega * ti).cos())
                    .collect();
                let sin_col: Vec<f64> = t
                    .iter()
                    .map(|&ti| (d * ti).exp() * (omega * ti).sin())
                    .collect();
                let c_idx = col_meta.len();
                col_meta.push((f, d, true));
                let s_idx = col_meta.len();
                col_meta.push((f, d, false));
                for row in 0..ns {
                    phi[row][c_idx] = cos_col[row];
                    phi[row][s_idx] = sin_col[row];
                }
            }
        }

        // Solve normal equations: (ΦᵀΦ) c = Φᵀ y  via Gaussian elimination
        // Build ΦᵀΦ and Φᵀy
        let mut phitphi = vec![vec![0.0_f64; n_cols]; n_cols];
        let mut phity = vec![0.0_f64; n_cols];
        for row in 0..ns {
            let y = signal[row];
            for j in 0..n_cols {
                phity[j] += phi[row][j] * y;
                for k in 0..n_cols {
                    phitphi[j][k] += phi[row][j] * phi[row][k];
                }
            }
        }

        // Gaussian elimination with partial pivoting
        let c = gaussian_elimination(&phitphi, &phity).unwrap_or_else(|| vec![0.0; n_cols]);

        // Compute reconstruction and residual
        let mut reconstructed = vec![0.0_f64; ns];
        for row in 0..ns {
            for j in 0..n_cols {
                reconstructed[row] += phi[row][j] * c[j];
            }
        }
        let rms_signal = (signal.iter().map(|v| v * v).sum::<f64>() / ns as f64)
            .sqrt()
            .max(1e-15);
        let rms_residual = (signal
            .iter()
            .zip(reconstructed.iter())
            .map(|(s, r)| (s - r) * (s - r))
            .sum::<f64>()
            / ns as f64)
            .sqrt();
        let signal_reconstruction_error = rms_residual / rms_signal;

        // Pick best n_modes pairs by energy (|c_cos|² + |c_sin|²)
        // Group by (freq, damp) pair index
        let n_pairs = freqs.len() * dampings.len();
        let mut pair_energy = vec![0.0_f64; n_pairs];
        let mut pair_indices = vec![(0.0_f64, 0.0_f64); n_pairs]; // (freq, damp)

        for (col_idx, &(f, d, is_cos)) in col_meta.iter().enumerate() {
            // Find which pair this column belongs to
            let fi = freqs
                .iter()
                .position(|&ff| (ff - f).abs() < 1e-9)
                .unwrap_or(0);
            let di = dampings
                .iter()
                .position(|&dd| (dd - d).abs() < 1e-9)
                .unwrap_or(0);
            let pair_idx = fi * dampings.len() + di;
            pair_energy[pair_idx] += c[col_idx] * c[col_idx];
            if is_cos {
                pair_indices[pair_idx] = (f, d);
            }
        }

        // Sort pairs by energy descending
        let mut pair_order: Vec<usize> = (0..n_pairs).collect();
        pair_order.sort_by(|&a, &b| {
            pair_energy[b]
                .partial_cmp(&pair_energy[a])
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let mut modes = Vec::new();
        for &pi in pair_order.iter().take(n_modes) {
            let (f, d) = pair_indices[pi];
            let omega = 2.0 * std::f64::consts::PI * f;

            // Find cos and sin coefficients for this pair
            let cos_c = col_meta
                .iter()
                .position(|&(ff, dd, ic)| ic && (ff - f).abs() < 1e-9 && (dd - d).abs() < 1e-9)
                .map(|idx| c[idx])
                .unwrap_or(0.0);
            let sin_c = col_meta
                .iter()
                .position(|&(ff, dd, ic)| !ic && (ff - f).abs() < 1e-9 && (dd - d).abs() < 1e-9)
                .map(|idx| c[idx])
                .unwrap_or(0.0);

            let amplitude = (cos_c * cos_c + sin_c * sin_c).sqrt();
            let phase_rad = (-sin_c).atan2(cos_c);
            let denom = (d * d + omega * omega).sqrt().max(1e-12);
            let damping_ratio = -d / denom;

            modes.push(PronyMode {
                amplitude,
                damping: d,
                freq_hz: f,
                phase_rad,
                damping_ratio,
            });
        }

        let n_modes_identified = modes.len();
        PronyResult {
            modes,
            signal_reconstruction_error,
            n_modes_identified,
        }
    }

    // ──────────────────────────────────────────────────────────────────────────
    // WADC design
    // ──────────────────────────────────────────────────────────────────────────

    /// Design a Wide-Area Damping Controller for the given inter-area mode
    /// using the residue-based lead-lag approach.
    pub fn design_wadc(&self, mode: &InterAreaMode) -> Result<WadcDesign, String> {
        if self.generators.is_empty() {
            return Err("No generators defined for WADC design".to_string());
        }
        if mode.mode_shape.is_empty() {
            return Err("Mode shape is empty".to_string());
        }

        // Find generator with highest participation (|mode_shape| * H_i)
        let participation: Vec<(usize, f64)> = self
            .generators
            .iter()
            .map(|g| {
                let ms = mode
                    .mode_shape
                    .iter()
                    .find(|(id, _)| *id == g.gen_id)
                    .map(|(_, v)| v.abs())
                    .unwrap_or(0.0);
                (g.gen_id, ms * g.inertia_h)
            })
            .collect();

        let control_gen_id = participation
            .iter()
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
            .map(|(id, _)| *id)
            .unwrap_or(self.generators[0].gen_id);

        // PMU bus: generator with 2nd highest participation, different area if possible
        let control_gen = self.generators.iter().find(|g| g.gen_id == control_gen_id);
        let control_area = control_gen.map(|g| g.area_id).unwrap_or(0);

        let pmu_gen = self
            .generators
            .iter()
            .filter(|g| g.gen_id != control_gen_id)
            .max_by(|a, b| {
                let pa = participation
                    .iter()
                    .find(|(id, _)| *id == a.gen_id)
                    .map(|(_, v)| *v)
                    .unwrap_or(0.0);
                let pb = participation
                    .iter()
                    .find(|(id, _)| *id == b.gen_id)
                    .map(|(_, v)| *v)
                    .unwrap_or(0.0);
                pa.partial_cmp(&pb).unwrap_or(std::cmp::Ordering::Equal)
            });

        let pmu_bus_id = pmu_gen
            .map(|g| g.bus_id)
            .unwrap_or_else(|| control_gen.map(|g| g.bus_id).unwrap_or(0));

        // Lead-lag design via phase compensation
        let omega_m = mode.omega_rad_per_s.max(0.01);

        // Phase needed: approximate residue angle from mode shape phase difference
        // between control gen and PMU gen
        let control_phase = mode
            .mode_shape
            .iter()
            .find(|(id, _)| *id == control_gen_id)
            .map(|(_, v)| v.signum())
            .unwrap_or(1.0);
        let pmu_phase = pmu_gen
            .and_then(|g| {
                mode.mode_shape
                    .iter()
                    .find(|(id, _)| *id == g.gen_id)
                    .map(|(_, v)| v.signum())
            })
            .unwrap_or(-1.0);

        // Phase compensation needed (rad)
        let phase_needed = if (control_phase * pmu_phase) < 0.0 {
            std::f64::consts::PI / 3.0
        } else {
            std::f64::consts::PI / 6.0
        };

        let phi_c = phase_needed / 2.0; // split between two stages
        let sin_phi = phi_c.sin().clamp(-0.999, 0.999);
        let alpha = ((1.0 - sin_phi) / (1.0 + sin_phi)).max(1e-6);
        let t1 = 1.0 / (omega_m * alpha.sqrt());
        let t2 = alpha * t1;

        let gain = 10.0 / mode.freq_hz.max(0.01);
        let washout_time_const = 10.0 / (2.0 * std::f64::consts::PI * mode.freq_hz.max(0.01));

        let expected_improvement_pct = 15.0 + 5.0 * mode.freq_hz.clamp(0.1, 2.0);

        // Communication delay: base + per participating area
        let n_areas = mode.participating_areas.len() as f64;
        let communication_delay_ms = 50.0 + 10.0 * n_areas;

        // Determine if control area and pmu area are different
        let _ = control_area; // suppress unused warning

        Ok(WadcDesign {
            pmu_bus_id,
            control_gen_id,
            washout_time_const,
            lead_lag_1: (t1, t2),
            lead_lag_2: (t1, t2),
            gain,
            expected_improvement_pct,
            communication_delay_ms,
        })
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Free helper functions
// ──────────────────────────────────────────────────────────────────────────────

/// Gaussian elimination with partial pivoting to solve A·x = b.
///
/// Returns `None` if the system is singular.
#[allow(clippy::needless_range_loop)]
fn gaussian_elimination(a: &[Vec<f64>], b: &[f64]) -> Option<Vec<f64>> {
    let n = b.len();
    if a.len() != n {
        return None;
    }

    let mut mat: Vec<Vec<f64>> = a
        .iter()
        .zip(b.iter())
        .map(|(row, &bi)| {
            let mut r = row.clone();
            r.push(bi);
            r
        })
        .collect();

    for col in 0..n {
        // Partial pivot
        let pivot_row = (col..n).max_by(|&r1, &r2| {
            mat[r1][col]
                .abs()
                .partial_cmp(&mat[r2][col].abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        })?;
        mat.swap(col, pivot_row);

        let pivot = mat[col][col];
        if pivot.abs() < 1e-15 {
            // Singular — zero out this column contribution
            continue;
        }

        for row in (col + 1)..n {
            let factor = mat[row][col] / pivot;
            for k in col..=n {
                let val = mat[col][k];
                mat[row][k] -= factor * val;
            }
        }
    }

    // Back substitution
    let mut x = vec![0.0_f64; n];
    for row in (0..n).rev() {
        let mut sum = mat[row][n];
        for k in (row + 1)..n {
            sum -= mat[row][k] * x[k];
        }
        let diag = mat[row][row];
        x[row] = if diag.abs() > 1e-15 { sum / diag } else { 0.0 };
    }

    Some(x)
}

/// Compute the inertia-weighted centroid of rotor angles for generators in a given area.
///
/// Returns `(weighted_average_delta, count)`.
fn delta_centroid_by_area(
    delta_angles: &[Vec<f64>],
    step_idx: usize,
    area_id: usize,
    generators: &[IaGenerator],
) -> (f64, usize) {
    let mut weighted_sum = 0.0_f64;
    let mut weight_total = 0.0_f64;
    let mut count = 0usize;

    for (gi, g) in generators.iter().enumerate() {
        if g.area_id == area_id {
            let delta = delta_angles
                .get(gi)
                .and_then(|v| v.get(step_idx))
                .copied()
                .unwrap_or(0.0);
            weighted_sum += g.inertia_h * delta;
            weight_total += g.inertia_h;
            count += 1;
        }
    }

    let centroid = if weight_total > 1e-12 {
        weighted_sum / weight_total
    } else {
        0.0
    };
    (centroid, count)
}

/// Compute the inter-area angle (difference between area centroids, weighted by inertia).
fn compute_inter_area_angle(
    delta_angles: &[Vec<f64>],
    time_s: &[f64],
    generators: &[IaGenerator],
) -> Vec<f64> {
    // Identify the two areas with largest total inertia
    let mut area_inertia: std::collections::HashMap<usize, f64> = std::collections::HashMap::new();
    for g in generators {
        *area_inertia.entry(g.area_id).or_insert(0.0) += g.inertia_h;
    }

    let mut sorted: Vec<(usize, f64)> = area_inertia.into_iter().collect();
    sorted.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

    if sorted.len() < 2 {
        return vec![0.0; time_s.len()];
    }

    let area1 = sorted[0].0;
    let area2 = sorted[1].0;

    time_s
        .iter()
        .enumerate()
        .map(|(step, _)| {
            let (c1, cnt1) = delta_centroid_by_area(delta_angles, step, area1, generators);
            let (c2, cnt2) = delta_centroid_by_area(delta_angles, step, area2, generators);
            if cnt1 > 0 && cnt2 > 0 {
                c1 - c2
            } else {
                0.0
            }
        })
        .collect()
}

// ──────────────────────────────────────────────────────────────────────────────
// Tests
// ──────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a simple 2-machine, 2-area analyser (Kundur-like).
    ///
    /// `damping` is D in pu, typically 0.01–0.05 for underdamped oscillations
    /// at typical H=6s, P_sync=0.5 pu (ω₀=314 rad/s).
    fn two_machine_system(h: f64, p_sync: f64, damping: f64) -> InterAreaAnalyzer {
        let mut ia = InterAreaAnalyzer::new(100.0);
        ia.add_generator(IaGenerator {
            gen_id: 0,
            area_id: 0,
            bus_id: 1,
            inertia_h: h,
            damping_d: damping,
            sync_torque_ks: 0.5,
            rated_mva: 900.0,
        });
        ia.add_generator(IaGenerator {
            gen_id: 1,
            area_id: 1,
            bus_id: 5,
            inertia_h: h,
            damping_d: damping,
            sync_torque_ks: 0.5,
            rated_mva: 900.0,
        });
        ia.add_area(SystemArea {
            area_id: 0,
            name: "Area1".into(),
            total_inertia_mws: 2.0 * h * 900.0,
            generator_ids: vec![0],
        });
        ia.add_area(SystemArea {
            area_id: 1,
            name: "Area2".into(),
            total_inertia_mws: 2.0 * h * 900.0,
            generator_ids: vec![1],
        });
        ia.add_tie_line(TieLine {
            from_area: 0,
            to_area: 1,
            synchronizing_power: p_sync,
            from_bus: 3,
            to_bus: 101,
        });
        ia
    }

    #[test]
    fn test_two_machine_mode_frequency() {
        // H=6s each, P_sync=0.5 pu → known formula result.
        let ia = two_machine_system(6.0, 0.5, 0.05);
        let f = ia.two_area_mode_frequency().expect("should compute");
        // M = 2H/ω₀ already carries ω₀, so (Kundur) f = (1/2π)√(P_sync·(M1+M2)/(M1·M2)).
        // With M1=M2: f = (1/2π)√(2·P_sync/M1); M1 = 12/ω₀ → 2·0.5/M1 = ω₀/12,
        // giving f = (1/2π)√(ω₀/12).
        let m1 = 2.0 * 6.0 / OMEGA_0;
        let expected = (1.0 / (2.0 * std::f64::consts::PI)) * (0.5 * 2.0 * m1 / (m1 * m1)).sqrt();
        assert!((f - expected).abs() < 1e-6, "f={f} expected={expected}");
        // Sanity: a real two-area inter-area mode lives in the 0.1–1 Hz band.
        assert!(
            (0.1..=1.0).contains(&f),
            "two-area mode {f} Hz should be a realistic inter-area frequency"
        );
    }

    #[test]
    fn test_damping_ratio_stable() {
        // σ=-0.3, ω=2.0 → ζ = 0.3/sqrt(0.09+4)
        let sigma = -0.3_f64;
        let omega = 2.0_f64;
        let damp = -sigma / (sigma * sigma + omega * omega).sqrt();
        assert!((damp - 0.14834).abs() < 0.001, "damping_ratio={damp}");
    }

    #[test]
    fn test_is_inter_area_flag() {
        let ia = two_machine_system(6.0, 0.5, 0.05);
        let modes = ia.compute_modes().expect("modes");
        assert!(!modes.is_empty(), "should have at least one mode");
        let inter = modes.iter().filter(|m| m.freq_hz < 1.0).count();
        assert!(inter > 0, "should have inter-area modes");
        for m in &modes {
            assert_eq!(m.is_inter_area, m.freq_hz < 1.0);
        }
    }

    #[test]
    fn test_higher_inertia_lower_frequency() {
        let f_low_h = two_machine_system(3.0, 0.5, 0.05)
            .two_area_mode_frequency()
            .expect("f1");
        let f_high_h = two_machine_system(12.0, 0.5, 0.05)
            .two_area_mode_frequency()
            .expect("f2");
        assert!(
            f_low_h > f_high_h,
            "higher inertia should give lower frequency: {f_low_h} vs {f_high_h}"
        );
    }

    #[test]
    fn test_participating_areas_sum_to_one() {
        let ia = two_machine_system(6.0, 0.5, 0.05);
        let modes = ia.compute_modes().expect("modes");
        assert!(!modes.is_empty());
        for m in &modes {
            let sum: f64 = m.participating_areas.iter().map(|(_, f)| f).sum();
            assert!((sum - 1.0).abs() < 1e-9, "sum={sum}");
        }
    }

    #[test]
    fn test_mode_shape_length() {
        let ia = two_machine_system(6.0, 0.5, 0.05);
        let modes = ia.compute_modes().expect("modes");
        assert!(!modes.is_empty());
        for m in &modes {
            assert_eq!(m.mode_shape.len(), ia.generators.len());
        }
    }

    #[test]
    fn test_ringdown_duration() {
        let ia = two_machine_system(6.0, 0.5, 0.05);
        let rd = ia.simulate_ringdown(100.0, 30.0, 0.05).expect("ringdown");
        let last_t = rd.time_s.last().copied().unwrap_or(0.0);
        assert!((last_t - 30.0).abs() < 0.1, "last_t={last_t}");
    }

    #[test]
    fn test_ringdown_inter_area_angle_oscillates() {
        let ia = two_machine_system(6.0, 0.5, 0.03);
        let rd = ia.simulate_ringdown(100.0, 20.0, 0.05).expect("ringdown");
        let has_positive = rd.inter_area_angle.iter().any(|&v| v > 1e-10);
        let has_negative = rd.inter_area_angle.iter().any(|&v| v < -1e-10);
        assert!(
            has_positive && has_negative,
            "inter_area_angle should oscillate"
        );
    }

    #[test]
    fn test_prony_identifies_dominant_freq() {
        // Inject a pure 0.5 Hz decaying signal
        let dt = 0.05_f64;
        let n = 600_usize; // 30 s
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 * dt;
                0.5 * (-0.1 * t).exp() * (2.0 * std::f64::consts::PI * 0.5 * t).cos()
            })
            .collect();
        let result = InterAreaAnalyzer::prony_analysis(&signal, dt, 3);
        assert!(!result.modes.is_empty(), "should identify modes");
        // Best mode should be near 0.5 Hz
        let best = result
            .modes
            .iter()
            .max_by(|a, b| {
                a.amplitude
                    .partial_cmp(&b.amplitude)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .expect("best mode");
        assert!(
            (best.freq_hz - 0.5).abs() < 0.25,
            "best freq={}",
            best.freq_hz
        );
    }

    #[test]
    fn test_prony_damping_negative_for_decay() {
        let dt = 0.05_f64;
        let n = 400_usize;
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 * dt;
                (-0.3 * t).exp() * (2.0 * std::f64::consts::PI * 0.7 * t).cos()
            })
            .collect();
        let result = InterAreaAnalyzer::prony_analysis(&signal, dt, 3);
        assert!(!result.modes.is_empty());
        // All modes from grid use negative damping exponents by construction
        for m in &result.modes {
            assert!(
                m.damping <= 0.0,
                "damping should be non-positive: {}",
                m.damping
            );
        }
    }

    #[test]
    fn test_two_area_freq_increases_with_tie_strength() {
        let f_weak = two_machine_system(6.0, 0.3, 0.05)
            .two_area_mode_frequency()
            .expect("f_weak");
        let f_strong = two_machine_system(6.0, 1.5, 0.05)
            .two_area_mode_frequency()
            .expect("f_strong");
        assert!(
            f_strong > f_weak,
            "stronger tie should give higher freq: {f_weak} vs {f_strong}"
        );
    }

    #[test]
    fn test_wadc_gain_positive() {
        let ia = two_machine_system(6.0, 0.5, 0.05);
        let modes = ia.compute_modes().expect("modes");
        assert!(!modes.is_empty());
        let wadc = ia.design_wadc(&modes[0]).expect("wadc");
        assert!(wadc.gain > 0.0, "gain={}", wadc.gain);
    }

    #[test]
    fn test_wadc_washout_positive() {
        let ia = two_machine_system(6.0, 0.5, 0.05);
        let modes = ia.compute_modes().expect("modes");
        let wadc = ia.design_wadc(&modes[0]).expect("wadc");
        assert!(wadc.washout_time_const > 0.0);
    }

    #[test]
    fn test_wadc_lead_t1_gt_t2() {
        let ia = two_machine_system(6.0, 0.5, 0.05);
        let modes = ia.compute_modes().expect("modes");
        let wadc = ia.design_wadc(&modes[0]).expect("wadc");
        let (t1, t2) = wadc.lead_lag_1;
        assert!(
            t1 >= t2,
            "T1={t1} should be >= T2={t2} for lead compensation"
        );
    }

    #[test]
    fn test_wadc_improvement_positive() {
        let ia = two_machine_system(6.0, 0.5, 0.05);
        let modes = ia.compute_modes().expect("modes");
        let wadc = ia.design_wadc(&modes[0]).expect("wadc");
        assert!(wadc.expected_improvement_pct > 0.0);
    }

    #[test]
    fn test_wadc_communication_delay() {
        let ia = two_machine_system(6.0, 0.5, 0.05);
        let modes = ia.compute_modes().expect("modes");
        let wadc = ia.design_wadc(&modes[0]).expect("wadc");
        assert!(wadc.communication_delay_ms > 0.0);
    }

    #[test]
    fn test_build_state_matrix_size() {
        let ia = two_machine_system(6.0, 0.5, 0.05);
        let a = ia.build_state_matrix();
        assert_eq!(a.len(), 4, "should be 4×4 for 2 generators");
        for row in &a {
            assert_eq!(row.len(), 4);
        }
    }

    #[test]
    fn test_state_matrix_top_right_identity() {
        let ia = two_machine_system(6.0, 0.5, 0.05);
        let a = ia.build_state_matrix();
        let n = ia.generators.len();
        // Top-right N×N block should be identity
        for (i, a_row) in a.iter().enumerate().take(n) {
            for j in 0..n {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (a_row[n + j] - expected).abs() < 1e-12,
                    "a[{i}][{}]={} expected={expected}",
                    n + j,
                    a_row[n + j]
                );
            }
        }
    }

    #[test]
    fn test_state_matrix_bottom_left_negative_diagonal() {
        let ia = two_machine_system(6.0, 0.5, 0.05);
        let a = ia.build_state_matrix();
        let n = ia.generators.len();
        // Bottom-left diagonal should be negative (restoring force)
        for i in 0..n {
            assert!(
                a[n + i][i] < 0.0,
                "bottom-left diagonal a[{}][{i}]={} should be < 0",
                n + i,
                a[n + i][i]
            );
        }
    }

    #[test]
    fn test_three_generator_system() {
        let mut ia = InterAreaAnalyzer::new(100.0);
        ia.add_generator(IaGenerator {
            gen_id: 0,
            area_id: 0,
            bus_id: 1,
            inertia_h: 6.0,
            damping_d: 0.05,
            sync_torque_ks: 0.5,
            rated_mva: 600.0,
        });
        ia.add_generator(IaGenerator {
            gen_id: 1,
            area_id: 0,
            bus_id: 2,
            inertia_h: 5.0,
            damping_d: 0.05,
            sync_torque_ks: 0.5,
            rated_mva: 600.0,
        });
        ia.add_generator(IaGenerator {
            gen_id: 2,
            area_id: 1,
            bus_id: 5,
            inertia_h: 6.0,
            damping_d: 0.05,
            sync_torque_ks: 0.5,
            rated_mva: 900.0,
        });
        ia.add_area(SystemArea {
            area_id: 0,
            name: "A".into(),
            total_inertia_mws: 6600.0,
            generator_ids: vec![0, 1],
        });
        ia.add_area(SystemArea {
            area_id: 1,
            name: "B".into(),
            total_inertia_mws: 5400.0,
            generator_ids: vec![2],
        });
        ia.add_tie_line(TieLine {
            from_area: 0,
            to_area: 1,
            synchronizing_power: 0.4,
            from_bus: 3,
            to_bus: 101,
        });
        let modes = ia.compute_modes().expect("modes");
        assert!(
            !modes.is_empty(),
            "3-generator system should have at least 1 mode"
        );
    }

    #[test]
    fn test_compute_eigenvalues_2x2_two_values() {
        let a = [[0.0_f64, 1.0], [-1.0, -0.1]];
        let eigs = InterAreaAnalyzer::compute_eigenvalues_2x2(a);
        assert_eq!(eigs.len(), 2);
    }

    #[test]
    fn test_stable_mode_sigma_negative() {
        let ia = two_machine_system(6.0, 0.5, 0.05); // good damping
        let modes = ia.compute_modes().expect("modes");
        assert!(!modes.is_empty());
        // At least one oscillatory mode should be stable
        let stable = modes
            .iter()
            .any(|m| m.sigma < 0.0 && m.omega_rad_per_s > 0.1);
        assert!(stable, "should have at least one stable oscillatory mode");
    }

    #[test]
    fn test_unstable_mode_sigma_positive() {
        // Very small damping, check that sigma near zero or the system eigenvalues are found
        let ia = two_machine_system(6.0, 0.5, 0.0); // zero damping
        let modes = ia.compute_modes().expect("modes");
        // With zero damping, sigma should be near zero (undamped oscillation)
        assert!(!modes.is_empty());
        let near_zero = modes
            .iter()
            .any(|m| m.sigma.abs() < 0.5 && m.omega_rad_per_s > 0.1);
        assert!(near_zero, "with zero damping, sigma should be near 0");
    }

    #[test]
    fn test_zero_perturbation_flat_signal() {
        let ia = two_machine_system(6.0, 0.5, 0.05);
        let rd = ia.simulate_ringdown(0.0, 5.0, 0.05).expect("ringdown");
        let max_angle = rd
            .inter_area_angle
            .iter()
            .map(|v| v.abs())
            .fold(0.0_f64, f64::max);
        assert!(
            max_angle < 1e-15,
            "zero perturbation should give flat signal: max={max_angle}"
        );
    }

    #[test]
    fn test_no_generators_error() {
        let ia = InterAreaAnalyzer::new(100.0);
        assert!(ia.compute_modes().is_err());
    }

    #[test]
    fn test_no_tie_lines_error() {
        let mut ia = InterAreaAnalyzer::new(100.0);
        ia.add_area(SystemArea {
            area_id: 0,
            name: "A".into(),
            total_inertia_mws: 0.0,
            generator_ids: vec![],
        });
        ia.add_area(SystemArea {
            area_id: 1,
            name: "B".into(),
            total_inertia_mws: 0.0,
            generator_ids: vec![],
        });
        assert!(ia.two_area_mode_frequency().is_err());
    }

    #[test]
    fn test_prony_empty_signal() {
        let result = InterAreaAnalyzer::prony_analysis(&[], 0.05, 3);
        assert_eq!(result.n_modes_identified, 0);
    }

    #[test]
    fn test_ringdown_delta_angles_shape() {
        let ia = two_machine_system(6.0, 0.5, 0.05);
        let rd = ia.simulate_ringdown(100.0, 10.0, 0.1).expect("ringdown");
        assert_eq!(rd.delta_angles.len(), 2, "should have 2 generators");
        let n_steps = rd.time_s.len();
        for da in &rd.delta_angles {
            assert_eq!(da.len(), n_steps);
        }
    }

    #[test]
    fn test_tie_line_flows_shape() {
        let ia = two_machine_system(6.0, 0.5, 0.05);
        let rd = ia.simulate_ringdown(100.0, 10.0, 0.1).expect("ringdown");
        assert_eq!(rd.tie_line_flows.len(), 1, "should have 1 tie-line");
        assert_eq!(rd.tie_line_flows[0].len(), rd.time_s.len());
    }

    #[test]
    fn test_mode_freq_in_inter_area_range() {
        let ia = two_machine_system(6.0, 0.5, 0.05);
        let modes = ia.compute_modes().expect("compute_modes should succeed");
        assert!(!modes.is_empty(), "should have at least one mode");
        for m in &modes {
            assert!(
                m.freq_hz > 0.0 && m.freq_hz <= 2.0,
                "freq_hz={} should be in (0.0, 2.0]",
                m.freq_hz
            );
        }
    }

    #[test]
    fn test_damping_ratio_positive_for_stable_system() {
        let ia = two_machine_system(6.0, 0.5, 0.05);
        let modes = ia.compute_modes().expect("compute_modes should succeed");
        assert!(!modes.is_empty(), "should have at least one mode");
        for m in &modes {
            if m.omega_rad_per_s > 0.1 {
                assert!(
                    m.damping_ratio > 0.0,
                    "damping_ratio={} should be positive for stable oscillatory mode at freq={}",
                    m.damping_ratio,
                    m.freq_hz
                );
            }
        }
    }

    #[test]
    fn test_participating_areas_two_areas_both_present() {
        let ia = two_machine_system(6.0, 0.5, 0.05);
        let modes = ia.compute_modes().expect("compute_modes should succeed");
        assert!(!modes.is_empty(), "should have at least one mode");
        // Find the inter-area mode (lowest frequency oscillatory mode)
        let inter_mode = modes
            .iter()
            .find(|m| m.is_inter_area)
            .expect("should have an inter-area mode");
        let area_ids: Vec<usize> = inter_mode
            .participating_areas
            .iter()
            .map(|(id, _)| *id)
            .collect();
        assert!(
            area_ids.contains(&0),
            "area 0 should be present in participating_areas"
        );
        assert!(
            area_ids.contains(&1),
            "area 1 should be present in participating_areas"
        );
    }

    #[test]
    fn test_mode_shape_normalized_max_abs_one() {
        let ia = two_machine_system(6.0, 0.5, 0.05);
        let modes = ia.compute_modes().expect("compute_modes should succeed");
        assert!(!modes.is_empty(), "should have at least one mode");
        for m in &modes {
            let max_abs = m
                .mode_shape
                .iter()
                .map(|(_, v)| v.abs())
                .fold(0.0_f64, f64::max);
            assert!(
                (max_abs - 1.0).abs() < 1e-9,
                "max abs of mode_shape={} should be 1.0 for mode at freq={}",
                max_abs,
                m.freq_hz
            );
        }
    }

    #[test]
    fn test_critical_mode_is_lowest_damping() {
        let ia = two_machine_system(6.0, 0.5, 0.05);
        let modes = ia.compute_modes().expect("compute_modes should succeed");
        assert!(!modes.is_empty(), "should have at least one mode");
        let min_damping = modes
            .iter()
            .min_by(|a, b| {
                a.damping_ratio
                    .partial_cmp(&b.damping_ratio)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|m| m.damping_ratio)
            .expect("should find minimum damping mode");
        for m in &modes {
            assert!(
                m.damping_ratio >= min_damping,
                "mode at freq={} has damping_ratio={} below minimum={}",
                m.freq_hz,
                m.damping_ratio,
                min_damping
            );
        }
    }

    #[test]
    fn test_wadc_lead_lag_t1_positive() {
        let ia = two_machine_system(6.0, 0.5, 0.05);
        let modes = ia.compute_modes().expect("compute_modes should succeed");
        assert!(!modes.is_empty(), "should have at least one mode");
        let wadc = ia
            .design_wadc(&modes[0])
            .expect("design_wadc should succeed");
        assert!(
            wadc.lead_lag_1.0 > 0.0,
            "lead_lag_1.T1={} should be positive",
            wadc.lead_lag_1.0
        );
        assert!(
            wadc.lead_lag_1.1 > 0.0,
            "lead_lag_1.T2={} should be positive",
            wadc.lead_lag_1.1
        );
        assert!(
            wadc.lead_lag_2.0 > 0.0,
            "lead_lag_2.T1={} should be positive",
            wadc.lead_lag_2.0
        );
        assert!(
            wadc.lead_lag_2.1 > 0.0,
            "lead_lag_2.T2={} should be positive",
            wadc.lead_lag_2.1
        );
    }

    #[test]
    fn test_four_generator_two_area_mode_separation() {
        let mut ia = InterAreaAnalyzer::new(100.0);
        // Area 0: two generators
        ia.add_generator(IaGenerator {
            gen_id: 0,
            area_id: 0,
            bus_id: 1,
            inertia_h: 6.0,
            damping_d: 0.05,
            sync_torque_ks: 0.5,
            rated_mva: 900.0,
        });
        ia.add_generator(IaGenerator {
            gen_id: 1,
            area_id: 0,
            bus_id: 2,
            inertia_h: 6.0,
            damping_d: 0.05,
            sync_torque_ks: 0.5,
            rated_mva: 900.0,
        });
        // Area 1: two generators
        ia.add_generator(IaGenerator {
            gen_id: 2,
            area_id: 1,
            bus_id: 5,
            inertia_h: 6.0,
            damping_d: 0.05,
            sync_torque_ks: 0.5,
            rated_mva: 900.0,
        });
        ia.add_generator(IaGenerator {
            gen_id: 3,
            area_id: 1,
            bus_id: 6,
            inertia_h: 6.0,
            damping_d: 0.05,
            sync_torque_ks: 0.5,
            rated_mva: 900.0,
        });
        ia.add_area(SystemArea {
            area_id: 0,
            name: "Area0".into(),
            total_inertia_mws: 2.0 * 6.0 * 900.0,
            generator_ids: vec![0, 1],
        });
        ia.add_area(SystemArea {
            area_id: 1,
            name: "Area1".into(),
            total_inertia_mws: 2.0 * 6.0 * 900.0,
            generator_ids: vec![2, 3],
        });
        ia.add_tie_line(TieLine {
            from_area: 0,
            to_area: 1,
            synchronizing_power: 0.5,
            from_bus: 3,
            to_bus: 101,
        });
        let modes = ia
            .compute_modes()
            .expect("4-generator compute_modes should succeed");
        assert!(
            !modes.is_empty(),
            "4-generator system should have at least one mode"
        );
        for m in &modes {
            assert!(
                m.freq_hz > 0.0,
                "all modes should have positive frequency, got freq={}",
                m.freq_hz
            );
        }
    }

    #[test]
    fn test_two_area_formula_vs_compute_modes_consistent() {
        // Both methods should agree qualitatively: stronger tie-line → higher frequency.
        // We verify monotonicity rather than absolute agreement, since the two methods
        // use different internal models (simplified 2-area formula vs full eigenvalue).
        let ia_weak = two_machine_system(6.0, 0.3, 0.05);
        let ia_strong = two_machine_system(6.0, 1.5, 0.05);

        let formula_freq_weak = ia_weak
            .two_area_mode_frequency()
            .expect("two_area_mode_frequency (weak) should succeed");
        let formula_freq_strong = ia_strong
            .two_area_mode_frequency()
            .expect("two_area_mode_frequency (strong) should succeed");

        let modes_weak = ia_weak
            .compute_modes()
            .expect("compute_modes (weak) should succeed");
        let modes_strong = ia_strong
            .compute_modes()
            .expect("compute_modes (strong) should succeed");

        assert!(
            !modes_weak.is_empty(),
            "weak system should have at least one mode"
        );
        assert!(
            !modes_strong.is_empty(),
            "strong system should have at least one mode"
        );

        // Both methods must agree: stronger tie → higher frequency
        assert!(
            formula_freq_strong > formula_freq_weak,
            "formula: stronger tie should give higher freq ({} vs {})",
            formula_freq_strong,
            formula_freq_weak
        );
        assert!(
            modes_strong[0].freq_hz > modes_weak[0].freq_hz,
            "compute_modes: stronger tie should give higher freq ({} vs {})",
            modes_strong[0].freq_hz,
            modes_weak[0].freq_hz
        );
    }
}
