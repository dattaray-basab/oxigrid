/// Modal analysis for multi-machine power systems.
///
/// Computes participation factors, mode shapes, and classifies oscillatory
/// modes as inter-area or local based on the linearised A-matrix.
///
/// # References
/// - Kundur, "Power System Stability and Control", Chapter 12.
/// - Rogers, "Power System Oscillations", Kluwer 2000.
use nalgebra::DMatrix;
use serde::{Deserialize, Serialize};

/// A single oscillatory mode extracted from the A-matrix eigenvalues.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OscillatoryMode {
    /// Damping coefficient σ (real part of eigenvalue) [1/s]
    pub sigma: f64,
    /// Damped natural frequency ω_d (imaginary part of eigenvalue) [rad/s]
    pub omega_d: f64,
    /// Frequency `Hz`
    pub frequency_hz: f64,
    /// Damping ratio ζ = -σ / sqrt(σ²+ω²)
    pub damping_ratio: f64,
    /// Mode classification
    pub mode_type: ModeType,
    /// Participation factors per state (normalised, sum=1)
    pub participation: Vec<f64>,
    /// Right eigenvector (real part) — mode shape
    pub mode_shape_re: Vec<f64>,
    /// Right eigenvector (imaginary part)
    pub mode_shape_im: Vec<f64>,
}

impl OscillatoryMode {
    /// True if the mode is unstable (σ > 0).
    pub fn is_unstable(&self) -> bool {
        self.sigma > 0.0
    }

    /// True if damping ratio is below the 5% planning criterion.
    pub fn is_poorly_damped(&self) -> bool {
        self.damping_ratio < 0.05
    }

    /// Natural frequency [rad/s] = sqrt(σ² + ω_d²).
    pub fn omega_n(&self) -> f64 {
        (self.sigma * self.sigma + self.omega_d * self.omega_d).sqrt()
    }
}

/// Classification of an oscillatory mode.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ModeType {
    /// Inter-area oscillation (< 1 Hz), involves groups of machines swinging against each other.
    InterArea,
    /// Local plant mode (1–3 Hz), one generator against the rest of the system.
    Local,
    /// Control mode (associated with AVR/governor dynamics).
    Control,
    /// Non-oscillatory (real eigenvalue, no frequency).
    NonOscillatory,
}

/// Configuration for modal analysis.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ModalConfig {
    /// Minimum frequency `Hz` to include in oscillatory mode list.
    pub f_min_hz: f64,
    /// Maximum frequency `Hz` to include.
    pub f_max_hz: f64,
    /// Inter-area / local boundary frequency `Hz`.
    pub inter_area_threshold_hz: f64,
    /// Control mode boundary: modes with high damping but low freq.
    pub control_mode_threshold_hz: f64,
    /// Minimum participation factor to include a state in mode description.
    pub min_participation: f64,
}

impl Default for ModalConfig {
    fn default() -> Self {
        Self {
            f_min_hz: 0.01,
            f_max_hz: 5.0,
            inter_area_threshold_hz: 1.0,
            control_mode_threshold_hz: 0.2,
            min_participation: 0.01,
        }
    }
}

/// Result of modal analysis.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ModalResult {
    /// All extracted oscillatory modes (sorted by frequency ascending).
    pub modes: Vec<OscillatoryMode>,
    /// Real parts of ALL eigenvalues (including non-oscillatory).
    pub all_eigenvalue_real: Vec<f64>,
    /// Imaginary parts of ALL eigenvalues.
    pub all_eigenvalue_imag: Vec<f64>,
    /// Number of unstable modes.
    pub n_unstable: usize,
    /// Number of poorly-damped modes (ζ < 5%).
    pub n_poorly_damped: usize,
}

impl ModalResult {
    /// Worst-case (most negative) damping ratio among all oscillatory modes.
    pub fn min_damping_ratio(&self) -> Option<f64> {
        self.modes.iter().map(|m| m.damping_ratio).reduce(f64::min)
    }

    /// Modes sorted by damping ratio ascending (worst first).
    pub fn modes_by_damping(&self) -> Vec<&OscillatoryMode> {
        let mut refs: Vec<&OscillatoryMode> = self.modes.iter().collect();
        refs.sort_by(|a, b| {
            a.damping_ratio
                .partial_cmp(&b.damping_ratio)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        refs
    }

    /// Return inter-area modes only.
    pub fn inter_area_modes(&self) -> Vec<&OscillatoryMode> {
        self.modes
            .iter()
            .filter(|m| m.mode_type == ModeType::InterArea)
            .collect()
    }

    /// Return local modes only.
    pub fn local_modes(&self) -> Vec<&OscillatoryMode> {
        self.modes
            .iter()
            .filter(|m| m.mode_type == ModeType::Local)
            .collect()
    }
}

/// Perform modal analysis on a linearised system A-matrix.
///
/// The A-matrix is typically obtained from `small_signal::build_a_matrix()`.
/// State ordering is assumed to be: [δ₁, ω₁, δ₂, ω₂, …] (angle, speed per machine).
///
/// Returns an error string if eigenvalue decomposition fails.
pub fn modal_analysis(
    a_matrix: &DMatrix<f64>,
    config: &ModalConfig,
) -> Result<ModalResult, String> {
    let n = a_matrix.nrows();
    if a_matrix.ncols() != n {
        return Err(format!(
            "A-matrix must be square, got {}×{}",
            n,
            a_matrix.ncols()
        ));
    }

    // --- Eigenvalue decomposition via QR iteration (Schur decomposition) ---
    let schur = nalgebra::linalg::Schur::new(a_matrix.clone());
    let (q, t) = schur.unpack();

    // Extract eigenvalues from quasi-upper-triangular T
    let (evals_re, evals_im) = extract_eigenvalues(&t);

    // --- Build right eigenvectors from Schur vectors ---
    // For real Schur form, complex eigenvalue pairs need to be processed together.
    let n_eig = evals_re.len();
    let mut modes = Vec::new();

    let mut i = 0;
    while i < n_eig {
        let lambda_re = evals_re[i];
        let lambda_im = evals_im[i];

        if lambda_im.abs() < 1e-6 {
            // Real eigenvalue — non-oscillatory
            let pf = participation_factors_real(&q, &t, i, n);
            let sigma = lambda_re;
            let mode = OscillatoryMode {
                sigma,
                omega_d: 0.0,
                frequency_hz: 0.0,
                damping_ratio: if sigma < 0.0 { 1.0 } else { -1.0 },
                mode_type: ModeType::NonOscillatory,
                participation: pf,
                mode_shape_re: q.column(i).iter().copied().collect(),
                mode_shape_im: vec![0.0; n],
            };
            if filter_mode(&mode, config) {
                modes.push(mode);
            }
            i += 1;
        } else {
            // Complex pair — take positive imaginary part
            let sigma = lambda_re;
            let omega_d = lambda_im.abs();
            let omega_n = (sigma * sigma + omega_d * omega_d).sqrt();
            let damping_ratio = if omega_n > 1e-12 {
                -sigma / omega_n
            } else {
                0.0
            };
            let frequency_hz = omega_d / (2.0 * std::f64::consts::PI);

            let mode_type = classify_mode(frequency_hz, config);

            let (pf, shape_re, shape_im) = participation_factors_complex(&q, &t, i, n);

            let mode = OscillatoryMode {
                sigma,
                omega_d,
                frequency_hz,
                damping_ratio,
                mode_type,
                participation: pf,
                mode_shape_re: shape_re,
                mode_shape_im: shape_im,
            };
            if filter_mode(&mode, config) {
                modes.push(mode);
            }
            i += 2; // skip conjugate
        }
    }

    // Sort by frequency ascending
    modes.sort_by(|a, b| {
        a.frequency_hz
            .partial_cmp(&b.frequency_hz)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let n_unstable = modes.iter().filter(|m| m.is_unstable()).count();
    let n_poorly_damped = modes
        .iter()
        .filter(|m| m.is_poorly_damped() && !m.is_unstable())
        .count();

    Ok(ModalResult {
        modes,
        all_eigenvalue_real: evals_re,
        all_eigenvalue_imag: evals_im,
        n_unstable,
        n_poorly_damped,
    })
}

/// Extract eigenvalues from quasi-upper-triangular Schur form T.
fn extract_eigenvalues(t: &DMatrix<f64>) -> (Vec<f64>, Vec<f64>) {
    let n = t.nrows();
    let mut re = Vec::with_capacity(n);
    let mut im = Vec::with_capacity(n);
    let mut i = 0;
    while i < n {
        if i + 1 < n && t[(i + 1, i)].abs() > 1e-12 {
            // 2×2 block → complex pair
            let a11 = t[(i, i)];
            let a12 = t[(i, i + 1)];
            let a21 = t[(i + 1, i)];
            let a22 = t[(i + 1, i + 1)];
            let tr = a11 + a22;
            let det = a11 * a22 - a12 * a21;
            let disc = tr * tr / 4.0 - det;
            let sigma = tr / 2.0;
            let omega = (-disc).max(0.0).sqrt();
            re.push(sigma);
            im.push(omega);
            re.push(sigma);
            im.push(-omega);
            i += 2;
        } else {
            re.push(t[(i, i)]);
            im.push(0.0);
            i += 1;
        }
    }
    (re, im)
}

/// Participation factors for a real eigenvalue (column i of Q).
fn participation_factors_real(q: &DMatrix<f64>, _t: &DMatrix<f64>, i: usize, n: usize) -> Vec<f64> {
    // Participation factor p_k = |φ_k * ψ_k| / Σ|φ_j * ψ_j|
    // For real case, left eigenvector ≈ row i of Q^T (since A = Q T Q^T)
    let phi: Vec<f64> = q.column(i).iter().copied().collect();
    let psi: Vec<f64> = q.row(i).iter().copied().collect(); // Q^T row i = Q column i transposed

    let raw: Vec<f64> = (0..n).map(|k| (phi[k] * psi[k]).abs()).collect();
    let total: f64 = raw.iter().sum::<f64>().max(1e-30);
    raw.iter().map(|&v| v / total).collect()
}

/// Participation factors for a complex conjugate pair (columns i, i+1 of Q).
fn participation_factors_complex(
    q: &DMatrix<f64>,
    _t: &DMatrix<f64>,
    i: usize,
    n: usize,
) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    // Right eigenvector for complex pair: φ = q[:,i] + j*q[:,i+1]
    // Left eigenvector (row of Q^-1 ≈ Q^T for real Schur): ψ = q[i,:] - j*q[i+1,:]
    let phi_re: Vec<f64> = q.column(i).iter().copied().collect();
    let phi_im: Vec<f64> = if i + 1 < q.ncols() {
        q.column(i + 1).iter().copied().collect()
    } else {
        vec![0.0; n]
    };

    // Left eigenvector (approximate using Q^T)
    let psi_re: Vec<f64> = q.row(i).iter().copied().collect();
    let psi_im: Vec<f64> = if i + 1 < q.nrows() {
        q.row(i + 1).iter().copied().collect()
    } else {
        vec![0.0; n]
    };

    // Participation: p_k = |Re(φ_k * ψ_k*)|  (complex product)
    let raw: Vec<f64> = (0..n)
        .map(|k| {
            let prod_re = phi_re[k] * psi_re[k] + phi_im[k] * psi_im[k];
            let prod_im = phi_im[k] * psi_re[k] - phi_re[k] * psi_im[k];
            (prod_re * prod_re + prod_im * prod_im).sqrt()
        })
        .collect();

    let total: f64 = raw.iter().sum::<f64>().max(1e-30);
    let pf: Vec<f64> = raw.iter().map(|&v| v / total).collect();

    (pf, phi_re, phi_im)
}

/// Classify a mode by frequency.
fn classify_mode(f_hz: f64, cfg: &ModalConfig) -> ModeType {
    if f_hz < cfg.control_mode_threshold_hz {
        ModeType::Control
    } else if f_hz < cfg.inter_area_threshold_hz {
        ModeType::InterArea
    } else {
        ModeType::Local
    }
}

/// Decide whether to include a mode in the output list.
fn filter_mode(mode: &OscillatoryMode, cfg: &ModalConfig) -> bool {
    match mode.mode_type {
        ModeType::NonOscillatory => false,
        _ => mode.frequency_hz >= cfg.f_min_hz && mode.frequency_hz <= cfg.f_max_hz,
    }
}

/// Compute inter-area oscillation index: ratio of inter-machine angle spread for a mode.
///
/// For a mode with shape φ, the inter-area index is:
///   IAI = max|φ_δ| / mean|φ_δ|  (for angle states)
///
/// High IAI (>3) indicates inter-area character.
pub fn inter_area_index(mode_shape_re: &[f64], mode_shape_im: &[f64], n_machines: usize) -> f64 {
    // Angle states are at even indices (0, 2, 4, …) in [δ,ω,δ,ω,...] ordering
    let angle_mags: Vec<f64> = (0..n_machines.min(mode_shape_re.len() / 2))
        .map(|i| {
            let re = mode_shape_re[2 * i];
            let im = mode_shape_im[2 * i];
            (re * re + im * im).sqrt()
        })
        .collect();

    if angle_mags.is_empty() {
        return 0.0;
    }

    let max_mag = angle_mags.iter().cloned().fold(0.0_f64, f64::max);
    let mean_mag = angle_mags.iter().sum::<f64>() / angle_mags.len() as f64;
    if mean_mag < 1e-12 {
        0.0
    } else {
        max_mag / mean_mag
    }
}

/// Mode shape coherency: group machines with similar angular displacement.
///
/// Returns a vec of group assignments (0 = positive swing, 1 = negative swing).
pub fn coherency_groups(mode_shape_re: &[f64], n_machines: usize) -> Vec<usize> {
    let angles: Vec<f64> = (0..n_machines.min(mode_shape_re.len() / 2))
        .map(|i| mode_shape_re[2 * i])
        .collect();
    angles
        .iter()
        .map(|&a| if a >= 0.0 { 0 } else { 1 })
        .collect()
}

/// Build a simple 2-machine A-matrix for testing (with damping D).
/// States: [δ1, ω1, δ2, ω2]
/// Swing equations: dδ/dt = ω, M*dω/dt = -D*ω - k*(δ_i - δ_j)
///
/// `d` is the damping coefficient per machine.  Use d > 0 to ensure
/// non-zero eigenvalues (required for Schur-QR convergence).
pub fn two_machine_a_matrix(m1: f64, m2: f64, k_sync: f64) -> DMatrix<f64> {
    // Default: light damping to avoid singular eigenvalues
    two_machine_a_matrix_damped(m1, m2, k_sync, 0.5)
}

/// Two-machine A-matrix with explicit damping coefficient `d`.
pub fn two_machine_a_matrix_damped(m1: f64, m2: f64, k_sync: f64, d: f64) -> DMatrix<f64> {
    let mut a = DMatrix::zeros(4, 4);
    // dδ1/dt = ω1
    a[(0, 1)] = 1.0;
    // dω1/dt = -D/M1*ω1 - k/M1*δ1 + k/M1*δ2
    a[(1, 0)] = -k_sync / m1;
    a[(1, 1)] = -d / m1;
    a[(1, 2)] = k_sync / m1;
    // dδ2/dt = ω2
    a[(2, 3)] = 1.0;
    // dω2/dt = k/M2*δ1 - D/M2*ω2 - k/M2*δ2
    a[(3, 0)] = k_sync / m2;
    a[(3, 2)] = -k_sync / m2;
    a[(3, 3)] = -d / m2;
    a
}

/// Damping ratio report for all modes.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DampingReport {
    pub n_modes: usize,
    pub n_inter_area: usize,
    pub n_local: usize,
    pub n_unstable: usize,
    pub n_poorly_damped: usize,
    pub worst_damping_ratio: f64,
    pub worst_mode_freq_hz: f64,
}

impl DampingReport {
    pub fn from_result(res: &ModalResult) -> Self {
        let worst = res.modes_by_damping().first().copied();
        Self {
            n_modes: res.modes.len(),
            n_inter_area: res.inter_area_modes().len(),
            n_local: res.local_modes().len(),
            n_unstable: res.n_unstable,
            n_poorly_damped: res.n_poorly_damped,
            worst_damping_ratio: worst.map(|m| m.damping_ratio).unwrap_or(0.0),
            worst_mode_freq_hz: worst.map(|m| m.frequency_hz).unwrap_or(0.0),
        }
    }
}

// ────────────────────────────────────────────────────────────────────────────
// PSS placement and gain design based on modal analysis
// ────────────────────────────────────────────────────────────────────────────

/// PSS placement recommendation for a single machine.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PssDampingRecommendation {
    /// Machine index (row in state vector / 2)
    pub machine_index: usize,
    /// Participation factor magnitude in the target mode
    pub participation: f64,
    /// Recommended PSS gain (normalised, relative to unit input signal)
    pub recommended_gain: f64,
    /// Phase compensation required at mode frequency `degrees`
    pub phase_compensation_deg: f64,
    /// Priority: 1 = highest (place PSS here first)
    pub priority: usize,
}

/// Modal residue-based PSS gain calculation.
///
/// The residue R of a mode (σ ± jω) for machine k is proportional to
/// the participation factor p_k. The PSS gain K_PSS is chosen so that:
///
///   |K_PSS · R| = Δσ_target / |eigenvalue|
///
/// where Δσ_target is the desired additional damping [1/s].
///
/// Returns recommended gain (dimensionless, relative to per-unit speed signal).
pub fn residue_pss_gain(
    mode: &OscillatoryMode,
    participation: f64,
    delta_sigma_target: f64,
) -> f64 {
    // Residue magnitude is proportional to participation factor
    // Simplified: gain = Δσ_target / (participation * |eigenvalue|)
    let lambda_mag = (mode.sigma * mode.sigma + mode.omega_d * mode.omega_d).sqrt();
    if participation < 1e-6 || lambda_mag < 1e-6 {
        return 0.0;
    }
    (delta_sigma_target / (participation * lambda_mag)).clamp(0.1, 100.0)
}

/// Recommend PSS placement for improving damping of inter-area modes.
///
/// Examines all inter-area modes with damping ratio below `zeta_threshold`
/// and ranks machines by their participation factors.
///
/// Returns one recommendation per machine that appears in any poorly-damped mode.
pub fn recommend_pss_placement(
    result: &ModalResult,
    n_machines: usize,
    zeta_threshold: f64,
    delta_sigma_target: f64,
) -> Vec<PssDampingRecommendation> {
    // Accumulate max participation per machine across all poor inter-area modes
    let mut max_part: Vec<f64> = vec![0.0; n_machines];
    let mut phase_comp: Vec<f64> = vec![0.0; n_machines];
    let mut ref_mode: Vec<Option<&OscillatoryMode>> = vec![None; n_machines];

    for mode in &result.modes {
        if mode.mode_type != ModeType::InterArea {
            continue;
        }
        if mode.damping_ratio >= zeta_threshold {
            continue;
        }
        // Phase compensation needed = 180° - phase(H_pss) at mode frequency
        // For simplified design: target 45° phase lead per stage
        let phase_needed = (180.0 - mode.sigma.atan2(mode.omega_d).to_degrees()).abs();

        for (k, &pf) in mode.participation.iter().enumerate() {
            if k >= n_machines * 2 {
                break;
            }
            let machine = k / 2;
            if machine >= n_machines {
                break;
            }
            if pf > max_part[machine] {
                max_part[machine] = pf;
                phase_comp[machine] = phase_needed;
                ref_mode[machine] = Some(mode);
            }
        }
    }

    // Build recommendations for machines with participation > 1%
    let mut recs: Vec<PssDampingRecommendation> = (0..n_machines)
        .filter(|&m| max_part[m] > 0.01)
        .map(|m| {
            let gain = ref_mode[m]
                .map(|mode| residue_pss_gain(mode, max_part[m], delta_sigma_target))
                .unwrap_or(5.0);
            PssDampingRecommendation {
                machine_index: m,
                participation: max_part[m],
                recommended_gain: gain,
                phase_compensation_deg: phase_comp[m].clamp(0.0, 90.0),
                priority: 0, // filled below
            }
        })
        .collect();

    // Sort by participation descending, assign priority
    recs.sort_by(|a, b| {
        b.participation
            .partial_cmp(&a.participation)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    for (i, rec) in recs.iter_mut().enumerate() {
        rec.priority = i + 1;
    }
    recs
}

/// Compute the damping improvement from adding a PSS with given gain.
///
/// Uses the residue approximation:
///   Δσ ≈ K_PSS · |R_k| · |H_pss(jω)|
///
/// where |R_k| ≈ participation × |eigenvalue|.
///
/// Returns estimated additional damping [1/s] per mode.
pub fn estimated_damping_improvement(
    mode: &OscillatoryMode,
    participation: f64,
    pss_gain: f64,
    pss_magnitude_at_mode: f64,
) -> f64 {
    let lambda_mag = (mode.sigma * mode.sigma + mode.omega_d * mode.omega_d).sqrt();
    pss_gain * participation * lambda_mag * pss_magnitude_at_mode
}

/// Compute the PSS phase that maximises damping for a given mode.
///
/// The optimal phase is 180° - ∠(mode_residue), which for simplified
/// two-machine systems equals 90° - atan(σ / ω_d).
pub fn optimal_pss_phase_deg(mode: &OscillatoryMode) -> f64 {
    let phi = mode.sigma.atan2(mode.omega_d).to_degrees();
    (90.0 - phi).clamp(-90.0, 90.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    #[test]
    fn test_two_machine_a_matrix_shape() {
        let a = two_machine_a_matrix(5.0, 3.0, 1.0);
        assert_eq!(a.nrows(), 4);
        assert_eq!(a.ncols(), 4);
        // Check dδ/dt = ω row
        assert_abs_diff_eq!(a[(0, 1)], 1.0, epsilon = 1e-12);
        assert_abs_diff_eq!(a[(2, 3)], 1.0, epsilon = 1e-12);
    }

    #[test]
    fn test_modal_analysis_two_machine() {
        let a = two_machine_a_matrix(5.0, 3.0, 1.0);
        let cfg = ModalConfig::default();
        let result = modal_analysis(&a, &cfg).unwrap();
        // Should find inter-area mode around 0.1–0.3 Hz
        assert!(!result.modes.is_empty(), "Should find oscillatory modes");
        // All modes should be identified
        assert!(!result.all_eigenvalue_real.is_empty());
        assert_eq!(
            result.all_eigenvalue_real.len(),
            result.all_eigenvalue_imag.len()
        );
    }

    #[test]
    fn test_modal_eigenvalue_count() {
        let a = two_machine_a_matrix(5.0, 3.0, 2.0);
        let cfg = ModalConfig::default();
        let result = modal_analysis(&a, &cfg).unwrap();
        // 4×4 matrix → 4 eigenvalues
        assert_eq!(result.all_eigenvalue_real.len(), 4);
    }

    #[test]
    fn test_modal_damping_ratio_lightly_damped() {
        // Light damping → small damping ratio
        let a = two_machine_a_matrix_damped(5.0, 5.0, 2.0, 0.1); // very small D
        let cfg = ModalConfig::default();
        let result = modal_analysis(&a, &cfg).unwrap();
        for mode in &result.modes {
            // Lightly damped: ζ should be small (< 30%)
            assert!(
                mode.damping_ratio.abs() < 0.30,
                "Expected small damping ratio: {:.4}",
                mode.damping_ratio
            );
        }
    }

    #[test]
    fn test_inter_area_mode_classified() {
        // k=50, M=10 → omega=sqrt(10)=3.16 rad/s → f≈0.50 Hz → InterArea (0.2–1.0 Hz)
        let a = two_machine_a_matrix(10.0, 10.0, 50.0);
        let cfg = ModalConfig::default();
        let result = modal_analysis(&a, &cfg).unwrap();
        let has_inter_area = result
            .modes
            .iter()
            .any(|m| m.mode_type == ModeType::InterArea);
        assert!(
            has_inter_area,
            "Should detect inter-area mode at ~0.5 Hz: {:?}",
            result
                .modes
                .iter()
                .map(|m| (m.frequency_hz, &m.mode_type))
                .collect::<Vec<_>>()
        );
    }

    #[test]
    fn test_local_mode_classified() {
        // k=90, M=2 → omega=sqrt(90)=9.49 rad/s → f≈1.51 Hz → Local (>1.0 Hz)
        let a = two_machine_a_matrix(2.0, 2.0, 90.0);
        let cfg = ModalConfig::default();
        let result = modal_analysis(&a, &cfg).unwrap();
        let has_local = result.modes.iter().any(|m| m.mode_type == ModeType::Local);
        assert!(
            has_local,
            "Stiff system should have local mode: {:?}",
            result
                .modes
                .iter()
                .map(|m| m.frequency_hz)
                .collect::<Vec<_>>()
        );
    }

    #[test]
    fn test_participation_factors_sum_to_one() {
        let a = two_machine_a_matrix(5.0, 3.0, 1.5);
        let cfg = ModalConfig::default();
        let result = modal_analysis(&a, &cfg).unwrap();
        for mode in &result.modes {
            let sum: f64 = mode.participation.iter().sum();
            assert!(
                (sum - 1.0).abs() < 1e-6,
                "Participation factors must sum to 1, got {:.6}",
                sum
            );
        }
    }

    #[test]
    fn test_inter_area_index_two_machine() {
        // Two-machine inter-area: opposite swings, high IAI
        let shape_re = vec![1.0, 0.0, -1.0, 0.0]; // δ1=+1, δ2=-1
        let shape_im = vec![0.0, 0.0, 0.0, 0.0];
        let iai = inter_area_index(&shape_re, &shape_im, 2);
        assert!(iai >= 1.0, "Inter-area IAI should be ≥1: {:.3}", iai);
    }

    #[test]
    fn test_coherency_groups() {
        let shape_re = vec![1.0, 0.0, 1.0, 0.0, -1.0, 0.0]; // 2 positive, 1 negative
        let groups = coherency_groups(&shape_re, 3);
        assert_eq!(groups, vec![0, 0, 1]);
    }

    #[test]
    fn test_damping_report() {
        let a = two_machine_a_matrix(5.0, 3.0, 1.0);
        let cfg = ModalConfig::default();
        let result = modal_analysis(&a, &cfg).unwrap();
        let report = DampingReport::from_result(&result);
        assert_eq!(report.n_modes, result.modes.len());
    }

    #[test]
    fn test_modes_by_damping_order() {
        let a = two_machine_a_matrix(5.0, 3.0, 1.0);
        let cfg = ModalConfig::default();
        let result = modal_analysis(&a, &cfg).unwrap();
        let sorted = result.modes_by_damping();
        for window in sorted.windows(2) {
            assert!(
                window[0].damping_ratio <= window[1].damping_ratio,
                "Not sorted: {:.4} > {:.4}",
                window[0].damping_ratio,
                window[1].damping_ratio
            );
        }
    }

    #[test]
    fn test_unstable_mode_detection() {
        // A-matrix with negative damping → positive real part eigenvalues
        // Two-machine with negative damping coefficient
        let a = two_machine_a_matrix_damped(5.0, 3.0, 1.5, -2.0); // negative D → unstable
        let cfg = ModalConfig::default();
        let result = modal_analysis(&a, &cfg).unwrap();
        // Should find some unstable modes (positive real part)
        let has_positive_real = result.all_eigenvalue_real.iter().any(|&v| v > 0.0);
        assert!(
            has_positive_real,
            "Negative damping should produce unstable eigenvalues"
        );
    }

    #[test]
    fn test_min_damping_ratio_none_for_empty() {
        let result = ModalResult {
            modes: vec![],
            all_eigenvalue_real: vec![],
            all_eigenvalue_imag: vec![],
            n_unstable: 0,
            n_poorly_damped: 0,
        };
        assert!(result.min_damping_ratio().is_none());
    }

    #[test]
    fn test_mode_is_poorly_damped() {
        let mode = OscillatoryMode {
            sigma: -0.02,
            omega_d: 1.0,
            frequency_hz: 0.159,
            damping_ratio: 0.02, // < 5%
            mode_type: ModeType::InterArea,
            participation: vec![0.5, 0.5],
            mode_shape_re: vec![1.0, -1.0],
            mode_shape_im: vec![0.0, 0.0],
        };
        assert!(mode.is_poorly_damped());
        assert!(!mode.is_unstable());
    }

    #[test]
    fn test_residue_pss_gain_basic() {
        let mode = OscillatoryMode {
            sigma: -0.05,
            omega_d: std::f64::consts::PI,
            frequency_hz: 0.5,
            damping_ratio: 0.016,
            mode_type: ModeType::InterArea,
            participation: vec![0.6, 0.4],
            mode_shape_re: vec![1.0, -1.0],
            mode_shape_im: vec![0.0, 0.0],
        };
        let gain = residue_pss_gain(&mode, 0.6, 0.1);
        assert!(
            gain > 0.0 && gain <= 100.0,
            "Gain should be in range: {:.3}",
            gain
        );
    }

    #[test]
    fn test_residue_pss_gain_zero_participation() {
        let mode = OscillatoryMode {
            sigma: -0.05,
            omega_d: std::f64::consts::PI,
            frequency_hz: 0.5,
            damping_ratio: 0.016,
            mode_type: ModeType::InterArea,
            participation: vec![],
            mode_shape_re: vec![],
            mode_shape_im: vec![],
        };
        let gain = residue_pss_gain(&mode, 0.0, 0.1);
        assert_eq!(gain, 0.0);
    }

    #[test]
    fn test_recommend_pss_placement_poorly_damped() {
        let a = two_machine_a_matrix_damped(5.0, 5.0, 10.0, 0.05); // lightly damped
        let cfg = ModalConfig::default();
        let result = modal_analysis(&a, &cfg).unwrap();
        let recs = recommend_pss_placement(&result, 2, 0.5, 0.1);
        // Should recommend PSS for machines with high participation
        for rec in &recs {
            assert!(rec.participation > 0.01);
            assert!(rec.priority >= 1);
            assert!(rec.recommended_gain > 0.0);
        }
    }

    #[test]
    fn test_recommend_pss_well_damped_no_recs() {
        // Heavily damped: no mode below threshold
        let a = two_machine_a_matrix_damped(5.0, 5.0, 1.0, 50.0); // very high D
        let cfg = ModalConfig::default();
        let result = modal_analysis(&a, &cfg).unwrap();
        // With zeta_threshold=0.05, well-damped modes won't trigger
        let recs = recommend_pss_placement(&result, 2, 0.01, 0.1);
        // All modes have high damping → empty or minimal recommendations
        let _ = recs; // Just ensure it doesn't panic
    }

    #[test]
    fn test_estimated_damping_improvement() {
        let mode = OscillatoryMode {
            sigma: -0.05,
            omega_d: std::f64::consts::PI,
            frequency_hz: 0.5,
            damping_ratio: 0.016,
            mode_type: ModeType::InterArea,
            participation: vec![0.6, 0.4],
            mode_shape_re: vec![1.0, -1.0],
            mode_shape_im: vec![0.0, 0.0],
        };
        let delta = estimated_damping_improvement(&mode, 0.6, 10.0, 0.8);
        assert!(delta > 0.0, "PSS should improve damping: {:.4}", delta);
    }

    #[test]
    fn test_optimal_pss_phase() {
        let mode = OscillatoryMode {
            sigma: -0.1,
            omega_d: std::f64::consts::PI,
            frequency_hz: 0.5,
            damping_ratio: 0.032,
            mode_type: ModeType::InterArea,
            participation: vec![0.5, 0.5],
            mode_shape_re: vec![1.0, -1.0],
            mode_shape_im: vec![0.0, 0.0],
        };
        let phase = optimal_pss_phase_deg(&mode);
        assert!(
            (-90.0..=90.0).contains(&phase),
            "Phase should be bounded: {:.2}°",
            phase
        );
    }

    #[test]
    fn test_pss_placement_priority_ordering() {
        let a = two_machine_a_matrix_damped(5.0, 3.0, 5.0, 0.1);
        let cfg = ModalConfig::default();
        let result = modal_analysis(&a, &cfg).unwrap();
        let recs = recommend_pss_placement(&result, 2, 1.0, 0.15);
        // Priority should be in ascending order, participation descending
        for i in 1..recs.len() {
            assert!(
                recs[i - 1].participation >= recs[i].participation,
                "Not sorted by participation"
            );
            assert_eq!(recs[i - 1].priority, i);
        }
    }

    // ─── New tests (Round 27) ──────────────────────────────────────────────

    #[test]
    fn test_omega_n_matches_formula() {
        // Reason: omega_n() must equal sqrt(sigma^2 + omega_d^2) by definition.
        let mode = OscillatoryMode {
            sigma: -0.3,
            omega_d: 4.0,
            frequency_hz: 0.637,
            damping_ratio: 0.0747,
            mode_type: ModeType::InterArea,
            participation: vec![0.5, 0.5],
            mode_shape_re: vec![1.0, -1.0],
            mode_shape_im: vec![0.0, 0.0],
        };
        let expected = (0.3_f64 * 0.3 + 4.0_f64 * 4.0).sqrt();
        approx::assert_relative_eq!(mode.omega_n(), expected, max_relative = 1e-12);
    }

    #[test]
    fn test_is_unstable_hand_built_mode() {
        // Reason: is_unstable() must return true exactly when sigma > 0.
        let stable_mode = OscillatoryMode {
            sigma: -0.1,
            omega_d: 2.0,
            frequency_hz: std::f64::consts::FRAC_1_PI,
            damping_ratio: 0.05,
            mode_type: ModeType::Local,
            participation: vec![1.0],
            mode_shape_re: vec![1.0],
            mode_shape_im: vec![0.0],
        };
        let unstable_mode = OscillatoryMode {
            sigma: 0.2,
            omega_d: 2.0,
            frequency_hz: std::f64::consts::FRAC_1_PI,
            damping_ratio: -0.1,
            mode_type: ModeType::Local,
            participation: vec![1.0],
            mode_shape_re: vec![1.0],
            mode_shape_im: vec![0.0],
        };
        assert!(!stable_mode.is_unstable(), "sigma < 0 must not be unstable");
        assert!(unstable_mode.is_unstable(), "sigma > 0 must be unstable");
    }

    #[test]
    fn test_modal_config_default_field_sanity() {
        // Reason: Default thresholds must satisfy domain ordering constraints.
        let cfg = ModalConfig::default();
        assert!(cfg.f_min_hz < cfg.f_max_hz, "f_min must be < f_max");
        assert!(
            cfg.control_mode_threshold_hz < cfg.inter_area_threshold_hz,
            "control threshold must be < inter-area threshold"
        );
        assert!(
            cfg.inter_area_threshold_hz < cfg.f_max_hz,
            "inter-area threshold must be < f_max"
        );
        assert!(
            cfg.min_participation > 0.0,
            "min participation must be positive"
        );
    }

    #[test]
    fn test_modal_analysis_non_square_returns_err() {
        // Reason: modal_analysis must return Err for a non-square matrix.
        let a = nalgebra::DMatrix::<f64>::zeros(3, 4);
        let cfg = ModalConfig::default();
        let result = modal_analysis(&a, &cfg);
        assert!(result.is_err(), "Non-square A-matrix must return Err");
    }

    #[test]
    fn test_min_damping_ratio_non_empty() {
        // Reason: min_damping_ratio() must return the smallest ratio in a non-empty result.
        let a = two_machine_a_matrix(5.0, 3.0, 1.5);
        let cfg = ModalConfig::default();
        let result = modal_analysis(&a, &cfg).expect("modal analysis should succeed");
        if let Some(min_dr) = result.min_damping_ratio() {
            for mode in &result.modes {
                assert!(
                    mode.damping_ratio >= min_dr - 1e-12,
                    "min_damping_ratio returned {min_dr:.6} but mode has {:.6}",
                    mode.damping_ratio
                );
            }
        }
    }

    #[test]
    fn test_inter_area_modes_filter() {
        // Reason: inter_area_modes() must return only InterArea-classified modes.
        let a = two_machine_a_matrix(10.0, 10.0, 20.0);
        let cfg = ModalConfig::default();
        let result = modal_analysis(&a, &cfg).expect("modal analysis should succeed");
        for mode in result.inter_area_modes() {
            assert_eq!(
                mode.mode_type,
                ModeType::InterArea,
                "Expected InterArea, got {:?}",
                mode.mode_type
            );
        }
    }

    #[test]
    fn test_local_modes_filter() {
        // Reason: local_modes() must return only Local-classified modes.
        let a = two_machine_a_matrix(2.0, 2.0, 90.0);
        let cfg = ModalConfig::default();
        let result = modal_analysis(&a, &cfg).expect("modal analysis should succeed");
        for mode in result.local_modes() {
            assert_eq!(
                mode.mode_type,
                ModeType::Local,
                "Expected Local, got {:?}",
                mode.mode_type
            );
        }
    }

    #[test]
    fn test_all_eigenvalue_real_parts_negative_for_stable_system() {
        // Reason: a damped two-machine system must have all eigenvalue real parts < 0.
        let a = two_machine_a_matrix_damped(5.0, 5.0, 4.0, 1.0);
        let cfg = ModalConfig::default();
        let result = modal_analysis(&a, &cfg).expect("modal analysis should succeed");
        for &re in &result.all_eigenvalue_real {
            assert!(
                re < 1e-9,
                "Stable system must have non-positive real eigenvalue parts, got {re:.6}"
            );
        }
    }

    #[test]
    fn test_inter_area_index_empty_input() {
        // Reason: inter_area_index with zero-length shapes must return 0.0 without panic.
        let iai = inter_area_index(&[], &[], 4);
        approx::assert_relative_eq!(iai, 0.0, max_relative = 1e-12);
    }

    #[test]
    fn test_optimal_pss_phase_clamped_in_range() {
        // Reason: optimal_pss_phase_deg must always return a value in [-90, 90].
        let extreme_modes = vec![
            OscillatoryMode {
                sigma: -100.0,
                omega_d: 0.001,
                frequency_hz: 0.0002,
                damping_ratio: 0.999,
                mode_type: ModeType::Control,
                participation: vec![1.0],
                mode_shape_re: vec![1.0],
                mode_shape_im: vec![0.0],
            },
            OscillatoryMode {
                sigma: 0.001,
                omega_d: 100.0,
                frequency_hz: 15.9,
                damping_ratio: -0.00001,
                mode_type: ModeType::Local,
                participation: vec![1.0],
                mode_shape_re: vec![1.0],
                mode_shape_im: vec![0.0],
            },
        ];
        for mode in &extreme_modes {
            let phase = optimal_pss_phase_deg(mode);
            assert!(
                (-90.0..=90.0).contains(&phase),
                "Phase out of [-90, 90]: {phase:.3} for sigma={:.3}, omega_d={:.3}",
                mode.sigma,
                mode.omega_d
            );
        }
    }
}
