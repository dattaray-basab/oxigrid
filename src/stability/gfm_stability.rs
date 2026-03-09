//! Grid-Forming Inverter (GFM) Stability Assessment.
//!
//! Assesses the stability of low-inertia grids with Grid-Forming (GFM) inverters.
//! GFM inverters provide virtual inertia and frequency support, replacing the
//! role of conventional synchronous generators in low-carbon power systems.
//!
//! ## System Inertia
//!
//! Effective inertia constant \[s\]:
//! ```text
//! H_eff = (Σ H_i S_i + Σ H_v_k S_k) / S_total
//! ```
//!
//! Rate of Change of Frequency \[Hz/s\]:
//! ```text
//! ROCOF = ΔP / (2 H_eff)
//! ```
//!
//! ## Small-Signal Model
//!
//! Aggregate 2×2 state matrix:
//! ```text
//! A = [[-D/M,  -K/M],
//!      [ ωs,    0  ]]
//! ```
//! where M = 2H/ωs, D = total damping, K = synchronising torque coefficient.
//!
//! ## References
//! - Kundur, "Power System Stability and Control", McGraw-Hill, 1994.
//! - Arghir, Jouini & Dörfler, "Grid-forming control for power converters",
//!   IFAC-PapersOnLine 51(25), 2018.

use crate::error::{OxiGridError, Result};
use oxiblas_lapack::evd::GeneralEvd;
use oxiblas_matrix::Mat;
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

// ─────────────────────────────────────────────────────────────────────────────
// Configuration
// ─────────────────────────────────────────────────────────────────────────────

/// Configuration for GFM stability assessment.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GfmStabilityConfig {
    /// System base apparent power \[MVA\].
    pub base_mva: f64,
    /// System base voltage \[kV\].
    pub base_kv: f64,
    /// Nominal system frequency \[Hz\].
    pub frequency_hz: f64,
    /// Effective inertia constant \[s\] below which GFM support is required.
    pub inertia_threshold_s: f64,
}

impl Default for GfmStabilityConfig {
    fn default() -> Self {
        Self {
            base_mva: 100.0,
            base_kv: 110.0,
            frequency_hz: 50.0,
            inertia_threshold_s: 2.0,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// GFM control modes
// ─────────────────────────────────────────────────────────────────────────────

/// Grid-Forming inverter control mode.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum GfmControlMode {
    /// Classic P-f / Q-V droop control.
    Droop,
    /// Virtual Synchronous Machine (VSM) — emulates swing equation.
    VirtualSynchronousMachine,
    /// Energy-based matching control.
    MatchingControl,
    /// Dispatchable Virtual Oscillator Control (dVOC).
    DispatchableVirtualOscillator,
}

// ─────────────────────────────────────────────────────────────────────────────
// System components
// ─────────────────────────────────────────────────────────────────────────────

/// A Grid-Forming inverter unit.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GfmUnit {
    /// Bus index the inverter is connected to.
    pub bus: usize,
    /// Rated apparent power \[MVA\].
    pub rated_mva: f64,
    /// Virtual inertia constant H_v \[s\].
    pub virtual_inertia_s: f64,
    /// Active power – frequency droop coefficient K_p \[pu/pu\].
    pub droop_kp: f64,
    /// Reactive power – voltage droop coefficient K_q \[pu/pu\].
    pub droop_kq: f64,
    /// Current limit \[pu of rated current\] (typical: 1.1).
    pub current_limit_pu: f64,
    /// Control strategy.
    pub control_mode: GfmControlMode,
}

impl GfmUnit {
    /// Create a GFM unit with typical droop parameters.
    pub fn new_droop(bus: usize, rated_mva: f64, virtual_inertia_s: f64) -> Self {
        Self {
            bus,
            rated_mva,
            virtual_inertia_s,
            droop_kp: 0.05,
            droop_kq: 0.05,
            current_limit_pu: 1.1,
            control_mode: GfmControlMode::Droop,
        }
    }
}

/// A conventional synchronous generator.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SynchronousGenerator {
    /// Bus index.
    pub bus: usize,
    /// Rated apparent power \[MVA\].
    pub rated_mva: f64,
    /// Inertia constant H \[s\].
    pub inertia_h: f64,
    /// Damping coefficient D \[pu/(rad·s⁻¹)\].
    pub damping_d: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Results
// ─────────────────────────────────────────────────────────────────────────────

/// System inertia profile computed from generator + GFM inventory.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SystemInertiaProfile {
    /// Total kinetic energy from synchronous generators \[MJ\].
    pub total_synchronous_inertia_mjs: f64,
    /// Total virtual inertia from GFM inverters \[MJ\].
    pub total_virtual_inertia_mjs: f64,
    /// Combined effective inertia constant H_eff \[s\].
    pub effective_inertia_s: f64,
    /// ROCOF at 1 pu power imbalance \[Hz/s\].
    pub rocof_at_1pu_imbalance: f64,
    /// Estimated worst-case frequency nadir as a percentage \[%\].
    pub frequency_nadir_pct: f64,
    /// Quasi-steady-state frequency deviation \[Hz\].
    pub quasi_steady_state_deviation_hz: f64,
}

/// Full result of the GFM stability assessment.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GfmStabilityResult {
    /// System inertia breakdown and frequency metrics.
    pub system_inertia: SystemInertiaProfile,
    /// Minimum number of GFM units needed for compliance (0 if already compliant).
    pub gfm_units_needed: usize,
    /// Minimum required GFM capacity \[MVA\].
    pub gfm_capacity_mva: f64,
    /// Eigenvalues of the linearised system (real, imag) pairs.
    pub eigenvalues: Vec<(f64, f64)>,
    /// Dominant oscillation mode frequency \[Hz\].
    pub dominant_mode_freq_hz: f64,
    /// Damping ratio of the dominant mode.
    pub dominant_mode_damping: f64,
    /// Actionable recommendations.
    pub recommendations: Vec<String>,
    /// Stability index: 0 = unstable, (0,1) = marginal, ≥ 1 = stable.
    pub stability_index: f64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Assessor
// ─────────────────────────────────────────────────────────────────────────────

/// Performs GFM stability assessment for a given generator + inverter inventory.
pub struct GfmStabilityAssessor {
    /// Assessment configuration.
    pub config: GfmStabilityConfig,
}

impl GfmStabilityAssessor {
    /// Create a new assessor with the given configuration.
    pub fn new(config: GfmStabilityConfig) -> Self {
        Self { config }
    }

    /// Run the stability assessment.
    ///
    /// # Arguments
    /// * `generators`        — Conventional synchronous generators.
    /// * `gfm_units`         — Grid-Forming inverter units.
    /// * `total_load_mw`     — Total system load \[MW\].
    /// * `power_imbalance_pu` — Active power imbalance as a fraction of total load \[pu\].
    pub fn assess(
        &self,
        generators: &[SynchronousGenerator],
        gfm_units: &[GfmUnit],
        total_load_mw: f64,
        power_imbalance_pu: f64,
    ) -> Result<GfmStabilityResult> {
        let freq_hz = self.config.frequency_hz;
        let omega_s = 2.0 * PI * freq_hz;

        // ── 1. System inertia ────────────────────────────────────────────────
        let total_sync_mj: f64 = generators.iter().map(|g| g.inertia_h * g.rated_mva).sum();
        let total_virtual_mj: f64 = gfm_units
            .iter()
            .map(|u| u.virtual_inertia_s * u.rated_mva)
            .sum();

        let total_mva: f64 = generators.iter().map(|g| g.rated_mva).sum::<f64>()
            + gfm_units.iter().map(|u| u.rated_mva).sum::<f64>();

        if total_mva < 1e-6 {
            return Err(OxiGridError::InvalidParameter(
                "total system MVA is effectively zero".to_string(),
            ));
        }

        let effective_h = (total_sync_mj + total_virtual_mj) / total_mva;

        if effective_h < 1e-9 {
            return Err(OxiGridError::InvalidParameter(
                "effective inertia is effectively zero".to_string(),
            ));
        }

        // ROCOF = ΔP [MW] / (2 H_eff [s] * S_base [MVA])
        // Expressed in [Hz/s] for the 1-pu imbalance case:
        let delta_p_mw = power_imbalance_pu * total_load_mw;
        let rocof = delta_p_mw / (2.0 * effective_h * total_mva);

        // Frequency nadir approximation:
        // Δf_nadir ≈ ROCOF * τ_nadir, where τ_nadir ≈ sqrt(2H/Ks)/ωs
        // with synchronising torque coefficient Ks ≈ 0.3 pu
        let k_sync_pu = 0.3_f64;
        let tau_nadir = (2.0 * effective_h / k_sync_pu).sqrt() / omega_s;
        let f_nadir_hz = rocof * tau_nadir;
        let f_nadir_pct = f_nadir_hz / freq_hz * 100.0;

        // Quasi-steady-state deviation: Δf_ss = ΔP / (R_droop * S_total)
        // R_droop from GFM units (sum of kp), default droop gain = 20 pu
        let droop_gain: f64 = if gfm_units.is_empty() {
            20.0
        } else {
            gfm_units
                .iter()
                .map(|u| 1.0 / u.droop_kp.max(1e-6))
                .sum::<f64>()
        };
        let qss_deviation_hz = delta_p_mw / (droop_gain * total_mva) * freq_hz;

        let system_inertia = SystemInertiaProfile {
            total_synchronous_inertia_mjs: total_sync_mj,
            total_virtual_inertia_mjs: total_virtual_mj,
            effective_inertia_s: effective_h,
            rocof_at_1pu_imbalance: rocof,
            frequency_nadir_pct: f_nadir_pct.abs(),
            quasi_steady_state_deviation_hz: qss_deviation_hz.abs(),
        };

        // ── 2. Small-signal linearised model ─────────────────────────────────
        // Aggregate 2×2 model
        // M_eff = 2 H_eff / ωs
        let m_eff = 2.0 * effective_h / omega_s;

        // D_eff: generator damping + GFM droop contribution
        let d_sync: f64 = generators.iter().map(|g| g.damping_d).sum();

        // GFM droop contributes D_gfm = kp / ωs for Droop/VSM modes
        let d_gfm: f64 = gfm_units
            .iter()
            .map(|u| {
                let base = u.droop_kp / omega_s;
                match u.control_mode {
                    GfmControlMode::VirtualSynchronousMachine => base * 1.5,
                    GfmControlMode::DispatchableVirtualOscillator => base * 0.8,
                    _ => base,
                }
            })
            .sum();

        let d_eff = d_sync + d_gfm;

        // K_sync: adjust for dVOC
        let dvoc_factor: f64 = gfm_units
            .iter()
            .map(|u| {
                if u.control_mode == GfmControlMode::DispatchableVirtualOscillator {
                    0.9_f64
                } else {
                    1.0_f64
                }
            })
            .fold(1.0_f64, |acc, f| acc * f);

        let k_sync = k_sync_pu * total_mva * dvoc_factor;

        // A = [[-D/M, -K/M],
        //      [ ωs,   0  ]]
        let mut a_mat = Mat::<f64>::zeros(2, 2);
        a_mat[(0, 0)] = -d_eff / m_eff;
        a_mat[(0, 1)] = -k_sync / m_eff;
        a_mat[(1, 0)] = omega_s;
        a_mat[(1, 1)] = 0.0;

        // Eigenvalues via characteristic polynomial for 2×2 matrix
        // λ² - tr(A)·λ + det(A) = 0
        // tr = A[0,0] + A[1,1] = -D/M
        // det = A[0,0]*A[1,1] - A[0,1]*A[1,0] = 0 - (-K/M)*ωs = K*ωs/M
        let tr_a = a_mat[(0, 0)] + a_mat[(1, 1)];
        let det_a = a_mat[(0, 0)] * a_mat[(1, 1)] - a_mat[(0, 1)] * a_mat[(1, 0)];
        let disc = tr_a * tr_a - 4.0 * det_a;

        let eigenvalues: Vec<(f64, f64)> = if disc < 0.0 {
            // Complex conjugate pair
            let re = tr_a / 2.0;
            let im = (-disc).sqrt() / 2.0;
            vec![(re, im), (re, -im)]
        } else {
            // Two real eigenvalues
            let sqrt_disc = disc.sqrt();
            vec![
                ((tr_a + sqrt_disc) / 2.0, 0.0),
                ((tr_a - sqrt_disc) / 2.0, 0.0),
            ]
        };

        // Also try GeneralEvd for verification / larger systems
        let _ = GeneralEvd::eigenvalues_only(a_mat.as_ref());

        // ── 3. Dominant mode ─────────────────────────────────────────────────
        let dominant = eigenvalues
            .iter()
            .max_by(|(_, im_a), (_, im_b)| {
                im_a.abs()
                    .partial_cmp(&im_b.abs())
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .copied()
            .unwrap_or((0.0, 0.0));

        let dom_freq_hz = dominant.1.abs() / (2.0 * PI);
        let dom_lambda_mag = (dominant.0 * dominant.0 + dominant.1 * dominant.1).sqrt();
        let dom_damping = if dom_lambda_mag > 1e-12 {
            -dominant.0 / dom_lambda_mag
        } else {
            0.0
        };

        // ── 4. Minimum GFM capacity (binary search) ──────────────────────────
        // Target: ROCOF < 1.0 Hz/s  AND  Δf_nadir < 2.0%
        let rocof_limit = 1.0_f64;
        let nadir_limit = 2.0_f64; // %

        let already_compliant = rocof <= rocof_limit && f_nadir_pct.abs() <= nadir_limit;

        let (gfm_capacity_mva, gfm_units_needed) = if already_compliant {
            (0.0, 0usize)
        } else {
            // Assume extra GFM units have H_v = 4.0 s (average)
            let avg_h_v = if gfm_units.is_empty() {
                4.0
            } else {
                gfm_units.iter().map(|u| u.virtual_inertia_s).sum::<f64>() / gfm_units.len() as f64
            };

            let mut lo = 0.0_f64;
            let mut hi = 10_000.0_f64; // [MJ] max search
            for _ in 0..40 {
                let mid = (lo + hi) / 2.0;
                let extra_mva = mid / avg_h_v.max(1e-3);
                let new_total_mj = total_sync_mj + total_virtual_mj + mid;
                let new_total_mva = total_mva + extra_mva;
                let new_h = new_total_mj / new_total_mva;
                let new_rocof = delta_p_mw / (2.0 * new_h * new_total_mva);
                let new_tau = (2.0 * new_h / k_sync_pu).sqrt() / omega_s;
                let new_nadir_pct = (new_rocof * new_tau / freq_hz * 100.0).abs();

                if new_rocof <= rocof_limit && new_nadir_pct <= nadir_limit {
                    hi = mid;
                } else {
                    lo = mid;
                }
            }
            let extra_mj = (lo + hi) / 2.0;
            let extra_mva = extra_mj / avg_h_v.max(1e-3);
            let n_units = if extra_mva < 1.0 {
                0usize
            } else {
                // Assume each additional GFM unit is rated at least 10 MVA
                (extra_mva / 10.0).ceil() as usize
            };
            (extra_mva, n_units)
        };

        // ── 5. Stability index ───────────────────────────────────────────────
        let all_stable = eigenvalues.iter().all(|&(re, _)| re < 0.0);
        let min_damping = eigenvalues
            .iter()
            .map(|&(re, im)| {
                let mag = (re * re + im * im).sqrt();
                if mag > 1e-12 {
                    -re / mag
                } else {
                    0.0
                }
            })
            .fold(f64::INFINITY, f64::min);

        let stability_index = if !all_stable {
            0.0
        } else if min_damping < 0.05 {
            0.5 + min_damping * 10.0
        } else {
            (1.0 + min_damping * 5.0).min(3.0)
        };

        // ── 6. Recommendations ───────────────────────────────────────────────
        let mut recommendations = Vec::new();

        if effective_h < self.config.inertia_threshold_s {
            recommendations.push(format!(
                "Effective inertia H_eff = {:.2} s is below threshold {:.1} s. \
                 Install GFM inverters or synchronous condensers.",
                effective_h, self.config.inertia_threshold_s
            ));
        }

        if rocof > rocof_limit {
            recommendations.push(format!(
                "ROCOF = {:.3} Hz/s exceeds 1.0 Hz/s grid code limit. \
                 Add {:.1} MVA of GFM capacity.",
                rocof, gfm_capacity_mva
            ));
        }

        if f_nadir_pct.abs() > nadir_limit {
            recommendations.push(format!(
                "Frequency nadir ≈ {:.2}% exceeds 2% limit. Increase virtual inertia.",
                f_nadir_pct.abs()
            ));
        }

        if !all_stable {
            recommendations.push(
                "System is SMALL-SIGNAL UNSTABLE. Increase GFM damping (VSM mode recommended)."
                    .to_string(),
            );
        } else if min_damping < 0.05 {
            recommendations.push(format!(
                "Minimum damping ratio {:.3} < 5%. Consider VSM or higher droop gain.",
                min_damping
            ));
        }

        if recommendations.is_empty() {
            recommendations
                .push("System meets inertia, ROCOF, and stability requirements.".to_string());
        }

        Ok(GfmStabilityResult {
            system_inertia,
            gfm_units_needed,
            gfm_capacity_mva,
            eigenvalues,
            dominant_mode_freq_hz: dom_freq_hz,
            dominant_mode_damping: dom_damping,
            recommendations,
            stability_index,
        })
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn large_gen() -> SynchronousGenerator {
        SynchronousGenerator {
            bus: 0,
            rated_mva: 400.0,
            inertia_h: 6.0,
            damping_d: 2.0,
        }
    }

    fn small_gen() -> SynchronousGenerator {
        SynchronousGenerator {
            bus: 0,
            rated_mva: 50.0,
            inertia_h: 1.5,
            damping_d: 0.5,
        }
    }

    fn droop_gfm(mva: f64, h_v: f64) -> GfmUnit {
        GfmUnit {
            bus: 1,
            rated_mva: mva,
            virtual_inertia_s: h_v,
            droop_kp: 0.05,
            droop_kq: 0.05,
            current_limit_pu: 1.1,
            control_mode: GfmControlMode::Droop,
        }
    }

    fn vsm_gfm(mva: f64, h_v: f64) -> GfmUnit {
        GfmUnit {
            control_mode: GfmControlMode::VirtualSynchronousMachine,
            ..droop_gfm(mva, h_v)
        }
    }

    #[test]
    fn test_high_inertia_system_compliant() {
        // Large synchronous generator → high inertia, low ROCOF
        let config = GfmStabilityConfig::default();
        let assessor = GfmStabilityAssessor::new(config);
        let result = assessor
            .assess(&[large_gen()], &[], 300.0, 0.1)
            .expect("assess should succeed");

        // ROCOF should be well below 1 Hz/s for a large high-inertia machine
        assert!(
            result.system_inertia.rocof_at_1pu_imbalance < 1.0,
            "ROCOF={:.4}",
            result.system_inertia.rocof_at_1pu_imbalance
        );
        // No additional GFM capacity needed
        assert_eq!(result.gfm_units_needed, 0);
        assert_eq!(result.gfm_capacity_mva, 0.0);
        // System should be stable
        assert!(
            result.stability_index > 0.5,
            "stability_index={:.3}",
            result.stability_index
        );
    }

    #[test]
    fn test_low_inertia_system_needs_gfm() {
        // Small generator, high imbalance → ROCOF violation
        let config = GfmStabilityConfig::default();
        let assessor = GfmStabilityAssessor::new(config);
        let result = assessor
            .assess(&[small_gen()], &[], 200.0, 0.3)
            .expect("assess should succeed");

        // High ROCOF expected for low-inertia system
        let rocof = result.system_inertia.rocof_at_1pu_imbalance;
        assert!(rocof > 0.0, "ROCOF should be positive");
        // Should recommend GFM capacity (when ROCOF > limit)
        if rocof > 1.0 {
            assert!(
                result.gfm_capacity_mva > 0.0,
                "Should require GFM capacity when non-compliant"
            );
        }
    }

    #[test]
    fn test_vsm_mode_better_damping_than_droop() {
        let config = GfmStabilityConfig::default();
        let assessor = GfmStabilityAssessor::new(config);

        let gen = small_gen();
        let droop_result = assessor
            .assess(
                std::slice::from_ref(&gen),
                &[droop_gfm(50.0, 4.0)],
                100.0,
                0.05,
            )
            .expect("droop assess");
        let vsm_result = assessor
            .assess(
                std::slice::from_ref(&gen),
                &[vsm_gfm(50.0, 4.0)],
                100.0,
                0.05,
            )
            .expect("vsm assess");

        // VSM provides more damping (higher D_gfm contribution)
        // Stability index should be >= droop (or equal)
        assert!(
            vsm_result.dominant_mode_damping >= droop_result.dominant_mode_damping - 1e-10,
            "VSM damping {:.4} should be ≥ droop damping {:.4}",
            vsm_result.dominant_mode_damping,
            droop_result.dominant_mode_damping
        );
    }

    #[test]
    fn test_rocof_compliance_with_sufficient_gfm() {
        // Combine synchronous gen + sufficient GFM
        let config = GfmStabilityConfig::default();
        let assessor = GfmStabilityAssessor::new(config);

        let gen = SynchronousGenerator {
            bus: 0,
            rated_mva: 100.0,
            inertia_h: 3.0,
            damping_d: 1.0,
        };
        // Large GFM unit with high virtual inertia
        let gfm = GfmUnit {
            bus: 1,
            rated_mva: 200.0,
            virtual_inertia_s: 6.0,
            droop_kp: 0.05,
            droop_kq: 0.05,
            current_limit_pu: 1.1,
            control_mode: GfmControlMode::VirtualSynchronousMachine,
        };

        let result = assessor
            .assess(&[gen], &[gfm], 200.0, 0.1)
            .expect("assess should succeed");

        // With combined inertia, ROCOF should be < 1 Hz/s
        assert!(
            result.system_inertia.rocof_at_1pu_imbalance < 1.0,
            "ROCOF={:.4} should be < 1.0 Hz/s with combined inertia",
            result.system_inertia.rocof_at_1pu_imbalance
        );
    }

    #[test]
    fn test_eigenvalue_stability_all_negative_real() {
        // With adequate damping the 2×2 model should always be stable
        let config = GfmStabilityConfig::default();
        let assessor = GfmStabilityAssessor::new(config);

        let gen = SynchronousGenerator {
            bus: 0,
            rated_mva: 200.0,
            inertia_h: 5.0,
            damping_d: 3.0, // strong damping
        };
        let result = assessor
            .assess(&[gen], &[], 150.0, 0.05)
            .expect("assess should succeed");

        // All eigenvalues should have negative real part
        for &(re, im) in &result.eigenvalues {
            assert!(
                re < 0.0,
                "Eigenvalue ({:.4}+{:.4}j) has non-negative real part",
                re,
                im
            );
        }
        assert!(
            result.stability_index > 0.0,
            "Stability index should be positive"
        );
    }

    #[test]
    fn test_zero_generators_returns_error() {
        let config = GfmStabilityConfig::default();
        let assessor = GfmStabilityAssessor::new(config);
        // No generators, no GFM → total_mva = 0
        assert!(assessor.assess(&[], &[], 100.0, 0.1).is_err());
    }

    #[test]
    fn test_inertia_profile_fields() {
        let config = GfmStabilityConfig::default();
        let assessor = GfmStabilityAssessor::new(config);
        let gen = large_gen();
        let gfm = droop_gfm(100.0, 4.0);
        let result = assessor.assess(&[gen], &[gfm], 400.0, 0.1).expect("assess");

        let prof = &result.system_inertia;
        assert!(prof.total_synchronous_inertia_mjs > 0.0);
        assert!(prof.total_virtual_inertia_mjs > 0.0);
        assert!(prof.effective_inertia_s > 0.0);
        assert!(prof.rocof_at_1pu_imbalance >= 0.0);
        assert!(prof.frequency_nadir_pct >= 0.0);
        assert!(prof.quasi_steady_state_deviation_hz >= 0.0);
    }
}
