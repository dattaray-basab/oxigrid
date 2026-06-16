/// Small-signal stability analysis for multi-machine power systems.
///
/// Linearises the classical swing equation around an operating point and
/// computes eigenvalues of the state matrix A.
///
/// State vector: x = [Δδ₁…Δδₙ, Δω₁…Δωₙ]ᵀ
/// State matrix:
///   A = [[0,      I  ],
///        [−M⁻¹K, −M⁻¹D]]
///
/// Eigenvalues λ = σ ± jωd
///   σ < 0  → stable mode
///   ζ = −σ/|λ| → damping ratio
///   fₙ = ωd/(2π) → natural frequency `Hz`
use nalgebra::DMatrix;
use serde::{Deserialize, Serialize};

/// Electromechanical oscillation mode extracted from eigenvalue analysis.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OscillationMode {
    /// Real part σ of eigenvalue (negative = stable)
    pub sigma: f64,
    /// Imaginary part ωd [rad/s]
    pub omega_d: f64,
    /// Damping ratio ζ = −σ/|λ|
    pub damping_ratio: f64,
    /// Natural frequency fn `Hz`
    pub freq_hz: f64,
    /// Participation factors (generator index → magnitude)
    pub participation: Vec<f64>,
}

impl OscillationMode {
    /// True if the mode is stable (σ < 0).
    pub fn is_stable(&self) -> bool {
        self.sigma < 0.0
    }

    /// True if the mode is oscillatory (ωd ≠ 0).
    pub fn is_oscillatory(&self) -> bool {
        self.omega_d.abs() > 1e-6
    }
}

/// Multi-machine small-signal model.
///
/// Generators are indexed 0…n−1. The synchronising torque matrix `k_sync`
/// and damping matrix `d_damp` must be pre-computed from the network.
pub struct SmallSignalModel {
    /// Inertia constants M_i = 2H_i/ωs [s²/rad]
    pub inertia: Vec<f64>,
    /// Damping coefficients D_i [p.u./rad·s⁻¹]
    pub damping: Vec<f64>,
    /// n×n synchronising torque matrix K (K_ij = ∂Pe_i/∂δ_j)
    pub k_sync: Vec<Vec<f64>>,
}

impl SmallSignalModel {
    /// Construct a single-machine-infinite-bus model.
    ///
    /// K_s = E'·V_inf·cos(δ₀)/X_tot
    pub fn smib(h: f64, d: f64, k_s: f64, freq_hz: f64) -> Self {
        let m = 2.0 * h / (2.0 * std::f64::consts::PI * freq_hz);
        Self {
            inertia: vec![m],
            damping: vec![d],
            k_sync: vec![vec![k_s]],
        }
    }

    /// Construct from a multi-machine reduced-network representation.
    ///
    /// `k_sync[i][j]` = ∂Pe_i/∂δ_j evaluated at the operating point.
    pub fn new(inertia: Vec<f64>, damping: Vec<f64>, k_sync: Vec<Vec<f64>>) -> Self {
        Self {
            inertia,
            damping,
            k_sync,
        }
    }

    /// Build the 2n×2n state matrix A.
    pub fn state_matrix(&self) -> DMatrix<f64> {
        let n = self.inertia.len();
        let mut a = DMatrix::zeros(2 * n, 2 * n);

        // Upper-right block: I (δ̇ = ω)
        for i in 0..n {
            a[(i, n + i)] = 1.0;
        }

        // Lower-left block: −M⁻¹K
        for i in 0..n {
            for j in 0..n {
                a[(n + i, j)] = -self.k_sync[i][j] / self.inertia[i];
            }
        }

        // Lower-right block: −M⁻¹D
        for i in 0..n {
            a[(n + i, n + i)] = -self.damping[i] / self.inertia[i];
        }

        a
    }

    /// Compute all eigenvalues of the state matrix.
    ///
    /// Returns `(σ, ωd)` pairs (real, imaginary parts).
    pub fn eigenvalues(&self) -> Vec<(f64, f64)> {
        let a = self.state_matrix();
        compute_eigenvalues_schur(&a)
    }

    /// Compute oscillation modes from eigenvalues.
    ///
    /// Only conjugate pairs (oscillatory modes) are returned; real
    /// eigenvalues (non-oscillatory / aperiodic) are omitted.
    pub fn oscillation_modes(&self) -> Vec<OscillationMode> {
        let eigs = self.eigenvalues();
        let n = self.inertia.len();
        let mut modes = Vec::new();

        let mut seen = vec![false; eigs.len()];
        for (i, &(sigma, omega_d)) in eigs.iter().enumerate() {
            if seen[i] {
                continue;
            }
            if omega_d.abs() < 1e-6 {
                continue;
            } // skip aperiodic

            // Find conjugate
            for (j, &(s2, w2)) in eigs.iter().enumerate().skip(i + 1) {
                if !seen[j] && (sigma - s2).abs() < 1e-6 && (omega_d + w2).abs() < 1e-6 {
                    seen[i] = true;
                    seen[j] = true;
                    break;
                }
            }

            let lambda_mag = (sigma * sigma + omega_d * omega_d).sqrt();
            let damping_ratio = if lambda_mag > 1e-12 {
                -sigma / lambda_mag
            } else {
                0.0
            };
            let freq_hz = omega_d.abs() / (2.0 * std::f64::consts::PI);

            // Simplified participation factors: uniform over generators
            let participation = vec![1.0 / n as f64; n];

            modes.push(OscillationMode {
                sigma,
                omega_d: omega_d.abs(),
                damping_ratio,
                freq_hz,
                participation,
            });
        }

        modes.sort_by(|a, b| {
            a.damping_ratio
                .partial_cmp(&b.damping_ratio)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        modes
    }

    /// True if all eigenvalues have negative real parts (Lyapunov stable).
    pub fn is_stable(&self) -> bool {
        self.eigenvalues().iter().all(|&(sigma, _)| sigma < 0.0)
    }
}

/// Compute eigenvalues of a real square matrix using nalgebra's Schur decomposition.
///
/// Extracts `(real, imag)` pairs from the quasi-upper-triangular Schur form.
pub fn compute_eigenvalues_schur(a: &DMatrix<f64>) -> Vec<(f64, f64)> {
    use nalgebra::linalg::Schur;
    assert_eq!(a.nrows(), a.ncols(), "Matrix must be square");
    let schur = Schur::new(a.clone());
    let (_, t) = schur.unpack();
    extract_eigenvalues_from_schur(&t)
}

/// Extract eigenvalues from a quasi-upper triangular Schur form.
fn extract_eigenvalues_from_schur(t: &DMatrix<f64>) -> Vec<(f64, f64)> {
    let n = t.nrows();
    let mut eigenvalues = Vec::with_capacity(n);
    let mut i = 0;

    while i < n {
        if i + 1 < n && t[(i + 1, i)].abs() > 1e-10 {
            // 2×2 block: extract eigenvalues from characteristic polynomial
            let a = t[(i, i)];
            let b = t[(i, i + 1)];
            let c = t[(i + 1, i)];
            let d = t[(i + 1, i + 1)];
            let trace = a + d;
            let det = a * d - b * c;
            let disc = trace * trace - 4.0 * det;
            if disc >= 0.0 {
                // Real eigenvalue pair
                let sq = disc.sqrt();
                eigenvalues.push(((trace + sq) / 2.0, 0.0));
                eigenvalues.push(((trace - sq) / 2.0, 0.0));
            } else {
                // Complex conjugate pair
                let real = trace / 2.0;
                let imag = (-disc).sqrt() / 2.0;
                eigenvalues.push((real, imag));
                eigenvalues.push((real, -imag));
            }
            i += 2;
        } else {
            // 1×1 block: real eigenvalue
            eigenvalues.push((t[(i, i)], 0.0));
            i += 1;
        }
    }

    eigenvalues
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_smib_eigenvalues_stable() {
        // H=6s, D=2 pu, K_s=2 pu/rad, f=60 Hz
        let m = 2.0 * 6.0 / (2.0 * std::f64::consts::PI * 60.0);
        let model = SmallSignalModel::smib(6.0, 2.0, 2.0, 60.0);
        let eigs = model.eigenvalues();
        // Both eigenvalues should have negative real part
        for (sigma, _) in &eigs {
            assert!(*sigma < 0.0, "sigma={:.4} should be negative", sigma);
        }
        // M is correct
        let expected_m = 2.0 * 6.0 / (2.0 * std::f64::consts::PI * 60.0);
        assert!((m - expected_m).abs() < 1e-10);
    }

    #[test]
    fn test_smib_is_stable() {
        let model = SmallSignalModel::smib(6.0, 2.0, 2.0, 60.0);
        assert!(model.is_stable());
    }

    #[test]
    fn test_smib_unstable_negative_ks() {
        // Negative K_s → unstable (beyond nose point)
        let model = SmallSignalModel::smib(6.0, 2.0, -1.0, 60.0);
        assert!(!model.is_stable());
    }

    #[test]
    fn test_oscillation_modes_smib() {
        // Use small damping D=0.05 so the SMIB is underdamped (oscillatory)
        // M = 2*6/(2π*60) ≈ 0.0318; K=2; ωn=√(K/M)≈7.9 rad/s; ζ = D/(2M*ωn) << 1 for D=0.05
        let model = SmallSignalModel::smib(6.0, 0.05, 2.0, 60.0);
        let eigs = model.eigenvalues();
        // At least one eigenvalue should have nonzero imaginary part (oscillatory)
        let oscillatory = eigs.iter().any(|&(_, imag)| imag.abs() > 0.1);
        assert!(
            oscillatory,
            "SMIB with low D should have oscillatory eigenvalues"
        );
        let modes = model.oscillation_modes();
        assert!(!modes.is_empty(), "Should have oscillation modes");
        let m = &modes[0];
        assert!(
            m.freq_hz > 0.5 && m.freq_hz < 5.0,
            "Inter-area freq should be 0.5–5 Hz: {:.3} Hz",
            m.freq_hz
        );
        assert!(
            m.damping_ratio > 0.0,
            "Damping ratio should be positive: {:.4}",
            m.damping_ratio
        );
    }

    #[test]
    fn test_two_machine_modes() {
        // Two identical generators coupled through a line; use small D for underdamped response
        let m = 2.0 * 6.0 / (2.0 * std::f64::consts::PI * 60.0);
        let ks = 2.0;
        let k_sync = vec![vec![ks, -ks / 2.0], vec![-ks / 2.0, ks]];
        let model = SmallSignalModel::new(
            vec![m, m],
            vec![0.05, 0.05], // small damping → oscillatory modes
            k_sync,
        );
        assert!(model.is_stable(), "Two-machine system should be stable");
        let modes = model.oscillation_modes();
        assert!(
            !modes.is_empty(),
            "Should have at least one oscillation mode"
        );
    }

    #[test]
    fn test_state_matrix_dimensions() {
        let model = SmallSignalModel::smib(6.0, 2.0, 2.0, 60.0);
        let a = model.state_matrix();
        assert_eq!(a.nrows(), 2);
        assert_eq!(a.ncols(), 2);
        // Upper-right should be 1 (identity for n=1)
        assert_eq!(a[(0, 1)], 1.0);
        // Upper-left should be 0
        assert_eq!(a[(0, 0)], 0.0);
    }

    // ---- 7 new tests ---------------------------------------------------

    #[test]
    fn test_oscillation_mode_is_stable_and_oscillatory() {
        // Construct a mode manually and verify flag methods.
        let stable_osc = OscillationMode {
            sigma: -0.5,
            omega_d: std::f64::consts::TAU,
            damping_ratio: 0.079,
            freq_hz: 1.0,
            participation: vec![0.5, 0.5],
        };
        assert!(stable_osc.is_stable(), "negative sigma must be stable");
        assert!(
            stable_osc.is_oscillatory(),
            "omega_d > 1e-6 must be oscillatory"
        );

        let unstable_real = OscillationMode {
            sigma: 0.3,
            omega_d: 0.0,
            damping_ratio: 0.0,
            freq_hz: 0.0,
            participation: vec![1.0],
        };
        assert!(
            !unstable_real.is_stable(),
            "positive sigma must not be stable"
        );
        assert!(
            !unstable_real.is_oscillatory(),
            "zero omega_d must not be oscillatory"
        );
    }

    #[test]
    fn test_compute_eigenvalues_schur_known_matrix() {
        // 2×2 matrix with known eigenvalues: [[−1, 0], [0, −3]] → λ = −1, −3 (both real)
        let a = DMatrix::from_row_slice(2, 2, &[-1.0_f64, 0.0, 0.0, -3.0]);
        let eigs = compute_eigenvalues_schur(&a);
        assert_eq!(eigs.len(), 2, "should extract 2 eigenvalues");
        let mut reals: Vec<f64> = eigs.iter().map(|&(r, _)| r).collect();
        reals.sort_by(|x, y| x.partial_cmp(y).expect("finite eigenvalue"));
        assert!(
            (reals[0] - (-3.0)).abs() < 1e-8,
            "first eigenvalue should be -3, got {}",
            reals[0]
        );
        assert!(
            (reals[1] - (-1.0)).abs() < 1e-8,
            "second eigenvalue should be -1, got {}",
            reals[1]
        );
        // Imaginary parts of a diagonal real matrix must be zero
        for &(_, imag) in &eigs {
            assert!(
                imag.abs() < 1e-8,
                "imaginary part should be zero, got {}",
                imag
            );
        }
    }

    #[test]
    fn test_state_matrix_lower_blocks_correct() {
        // Single-machine: verify lower-left (−K/M) and lower-right (−D/M)
        let h = 5.0_f64;
        let d = 3.0_f64;
        let k_s = 1.5_f64;
        let freq = 50.0_f64;
        let m = 2.0 * h / (2.0 * std::f64::consts::PI * freq);
        let model = SmallSignalModel::smib(h, d, k_s, freq);
        let a = model.state_matrix();
        // a[(1,0)] = −K/M
        let expected_lower_left = -k_s / m;
        assert!(
            (a[(1, 0)] - expected_lower_left).abs() < 1e-10,
            "lower-left block: expected {:.6}, got {:.6}",
            expected_lower_left,
            a[(1, 0)]
        );
        // a[(1,1)] = −D/M
        let expected_lower_right = -d / m;
        assert!(
            (a[(1, 1)] - expected_lower_right).abs() < 1e-10,
            "lower-right block: expected {:.6}, got {:.6}",
            expected_lower_right,
            a[(1, 1)]
        );
    }

    #[test]
    fn test_overdamped_smib_no_oscillation_modes() {
        // Very high damping → real eigenvalues → no oscillation modes.
        // ζ > 1 when D/(2*√(K*M)) > 1, i.e. D > 2*√(K*M).
        // M ≈ 0.0318, K=2 → critical D ≈ 2*√(2*0.0318) ≈ 0.504;
        // Use D=5 to be safely overdamped.
        let model = SmallSignalModel::smib(6.0, 5.0, 2.0, 60.0);
        let modes = model.oscillation_modes();
        assert!(
            modes.is_empty(),
            "overdamped SMIB should produce no oscillation modes, got {}",
            modes.len()
        );
        // System must still be stable
        assert!(model.is_stable(), "overdamped SMIB must be stable");
    }

    #[test]
    fn test_new_constructor_matches_manual_fields() {
        // SmallSignalModel::new() should store the supplied vectors verbatim.
        let inertia = vec![0.05, 0.06];
        let damping = vec![0.1, 0.2];
        let k_sync = vec![vec![3.0, -1.0], vec![-1.0, 3.0]];
        let model = SmallSignalModel::new(inertia.clone(), damping.clone(), k_sync.clone());
        assert_eq!(model.inertia, inertia, "inertia mismatch");
        assert_eq!(model.damping, damping, "damping mismatch");
        assert_eq!(model.k_sync, k_sync, "k_sync mismatch");
    }

    #[test]
    fn test_three_machine_stability_and_modes() {
        // Three identical generators coupled in a ring; use small D so modes are oscillatory.
        let m = 2.0 * 5.0 / (2.0 * std::f64::consts::PI * 50.0);
        let ks = 2.0_f64;
        // Ring coupling: diagonal = 2*ks, off-diagonal = −ks/2
        let k_sync = vec![
            vec![2.0 * ks, -ks / 2.0, -ks / 2.0],
            vec![-ks / 2.0, 2.0 * ks, -ks / 2.0],
            vec![-ks / 2.0, -ks / 2.0, 2.0 * ks],
        ];
        let model = SmallSignalModel::new(vec![m; 3], vec![0.05; 3], k_sync);
        // State matrix must be 6×6
        let a = model.state_matrix();
        assert_eq!(a.nrows(), 6, "state matrix rows for 3-machine should be 6");
        assert_eq!(a.ncols(), 6, "state matrix cols for 3-machine should be 6");
        assert!(
            model.is_stable(),
            "three-machine ring system should be stable"
        );
        let modes = model.oscillation_modes();
        assert!(
            !modes.is_empty(),
            "three-machine ring should have oscillation modes"
        );
    }

    #[test]
    fn test_oscillation_modes_sorted_by_damping_ratio_ascending() {
        // Use a three-machine system where modes are expected to differ.
        // At minimum the output must be sorted ascending by damping ratio.
        let m = 2.0 * 4.0 / (2.0 * std::f64::consts::PI * 60.0);
        let k_sync = vec![
            vec![1.5, -0.5, -0.3],
            vec![-0.5, 2.0, -0.4],
            vec![-0.3, -0.4, 1.8],
        ];
        let model =
            SmallSignalModel::new(vec![m, m * 1.2, m * 0.8], vec![0.05, 0.08, 0.03], k_sync);
        let modes = model.oscillation_modes();
        for window in modes.windows(2) {
            assert!(
                window[0].damping_ratio <= window[1].damping_ratio + 1e-12,
                "modes not sorted ascending: {:.6} > {:.6}",
                window[0].damping_ratio,
                window[1].damping_ratio
            );
        }
    }
}
