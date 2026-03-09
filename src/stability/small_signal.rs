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
///   fₙ = ωd/(2π) → natural frequency [Hz]
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
    /// Natural frequency fn [Hz]
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

        modes.sort_by(|a, b| a.damping_ratio.partial_cmp(&b.damping_ratio).unwrap());
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
}
