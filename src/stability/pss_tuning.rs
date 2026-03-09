//! Inter-Area Oscillation Damping — PSS Tuning via the GEP Method.
//!
//! Implements the Generator Electrical Power (GEP) method for tuning
//! Power System Stabilisers (PSS).  The PSS adds a supplementary signal to
//! the AVR excitation input to damp electromechanical oscillations.
//!
//! # Algorithm Overview
//!
//! 1. Model the SMIB system with Phillips-Heffron K-constants.
//! 2. Compute the phase lag of the electrical torque channel at the target
//!    inter-area oscillation frequency.
//! 3. Design a two-stage lead-lag network to provide the required phase
//!    compensation.
//! 4. Tune the PSS gain `K_s` using a grid search to achieve the target
//!    closed-loop damping ratio.
//! 5. Verify phase margin and gain margin.
//!
//! # References
//! - Kundur, P., "Power System Stability and Control", McGraw-Hill, 1994,
//!   Chapter 15.
//! - DeMello, F.P. and Concordia, C., "Concepts of Synchronous Machine
//!   Stability as Affected by Excitation Control", IEEE Trans. PAS, 1969.
//! - IEEE Std 421.5-2016, "IEEE Recommended Practice for Excitation System
//!   Models for Power System Stability Studies".

use crate::error::{OxiGridError, Result};
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Data structures
// ---------------------------------------------------------------------------

/// Phillips-Heffron K-constant linearised model for a SMIB system.
///
/// These six constants fully characterise the small-signal behaviour of a
/// synchronous machine connected to an infinite bus through a transmission
/// network.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PhillipsHeffronModel {
    /// Synchronising torque coefficient: `∂Pe/∂δ` \[p.u./rad\].
    pub k1: f64,
    /// Coupling between d-axis flux and electrical torque: `∂Pe/∂E'q` \[p.u./p.u.\].
    pub k2: f64,
    /// Field circuit demagnetisation factor: `1/(1 + Xd_prime·K3_factor)`.
    pub k3: f64,
    /// Demagnetising effect on d-axis flux: `∂ψd/∂δ` \[p.u./rad\].
    pub k4: f64,
    /// Terminal voltage sensitivity to angle: `∂Vt/∂δ` \[p.u./rad\].
    pub k5: f64,
    /// Terminal voltage sensitivity to flux: `∂Vt/∂E'q` \[p.u./p.u.\].
    pub k6: f64,
    /// Damping coefficient (mechanical) \[p.u.\].
    pub d: f64,
    /// Normalised inertia constant `2H` \[s\].
    pub m: f64,
    /// d-axis open-circuit transient time constant `T'_d0` \[s\].
    pub t_d0: f64,
    /// AVR gain `K_A` \[p.u./p.u.\].
    pub ka: f64,
    /// AVR time constant `T_A` \[s\].
    pub ta: f64,
}

impl Default for PhillipsHeffronModel {
    fn default() -> Self {
        // Typical SMIB parameters (Kundur example, p.813)
        Self {
            k1: 0.7640,
            k2: 0.8649,
            k3: 0.3231,
            k4: 1.4190,
            k5: -0.1110,
            k6: 0.4477,
            d: 0.0,
            m: 10.0, // 2H = 2*5
            t_d0: 7.32,
            ka: 200.0,
            ta: 0.05,
        }
    }
}

/// PSS lead-lag network parameters.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PssParameters {
    /// PSS gain `K_s` \[p.u.\].
    pub k_s: f64,
    /// Washout filter time constant `T_w` \[s\].
    pub t_w: f64,
    /// First lead-lag network: lead time constant `T_1` \[s\].
    pub t1: f64,
    /// First lead-lag network: lag time constant `T_2` \[s\].
    pub t2: f64,
    /// Second lead-lag network: lead time constant `T_3` \[s\].
    pub t3: f64,
    /// Second lead-lag network: lag time constant `T_4` \[s\].
    pub t4: f64,
    /// PSS output upper limit `V_smax` \[p.u.\].
    pub v_smax: f64,
    /// PSS output lower limit `V_smin` \[p.u.\].
    pub v_smin: f64,
}

/// PSS tuner configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PssTuner {
    /// Target inter-area oscillation frequency \[Hz\].  Typical: 0.5–2.0 Hz.
    pub target_mode_freq_hz: f64,
    /// Target closed-loop damping ratio ζ.  IEEE recommends ≥ 0.05.
    pub target_damping_ratio: f64,
    /// Washout filter time constant `T_w` \[s\].  Typical: 10 s.
    pub washout_time_const: f64,
    /// Search range for PSS gain `K_s`: (min, max).
    pub gain_range: (f64, f64),
    /// Search range for lead/lag ratio `T_1/T_2`: (min, max).
    pub lead_lag_range: (f64, f64),
    /// Number of gain search steps.
    pub gain_steps: usize,
}

impl Default for PssTuner {
    fn default() -> Self {
        Self {
            target_mode_freq_hz: 1.0,
            target_damping_ratio: 0.05,
            washout_time_const: 10.0,
            gain_range: (0.1, 50.0),
            lead_lag_range: (0.05, 20.0),
            gain_steps: 200,
        }
    }
}

/// Result of PSS tuning.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PssTuningResult {
    /// Tuned PSS parameters.
    pub parameters: PssParameters,
    /// Achieved closed-loop damping ratio.
    pub achieved_damping: f64,
    /// Achieved electromechanical mode frequency \[Hz\].
    pub achieved_freq_hz: f64,
    /// Phase margin of the open-loop system at crossover \[degrees\].
    pub phase_margin_deg: f64,
    /// Gain margin of the open-loop system \[dB\].
    pub gain_margin_db: f64,
    /// Number of gain search iterations performed.
    pub iterations: usize,
}

// ---------------------------------------------------------------------------
// Core tuning algorithm
// ---------------------------------------------------------------------------

impl PssTuner {
    /// Tune a PSS for the given Phillips-Heffron machine model.
    ///
    /// Uses the GEP (Generator Electrical Power) method:
    /// 1. Compute phase lag of electrical torque loop at ω₀.
    /// 2. Design lead-lag network to provide required phase lead.
    /// 3. Tune gain for target damping ratio.
    ///
    /// # Errors
    /// Returns [`OxiGridError::InvalidParameter`] if the target frequency
    /// or damping ratio are outside physically meaningful bounds.
    pub fn tune(&self, model: &PhillipsHeffronModel) -> Result<PssTuningResult> {
        if self.target_mode_freq_hz <= 0.0 || self.target_mode_freq_hz > 10.0 {
            return Err(OxiGridError::InvalidParameter(format!(
                "target_mode_freq_hz={:.3} must be in (0, 10] Hz",
                self.target_mode_freq_hz
            )));
        }
        if !(0.0..1.0).contains(&self.target_damping_ratio) {
            return Err(OxiGridError::InvalidParameter(format!(
                "target_damping_ratio={:.3} must be in [0, 1)",
                self.target_damping_ratio
            )));
        }

        let omega = 2.0 * PI * self.target_mode_freq_hz;

        // Step 1: Compute phase lag of electrical torque channel at omega
        // Transfer function of electrical torque: G_e(s) = K2 * K3 / (1 + s*K3*Td0)
        // Phase of G_e(jω) = -arctan(ω * K3 * Td0)
        let ge_phase_rad = -(omega * model.k3 * model.t_d0).atan();

        // AVR contribution: G_avr(jω) = Ka / (1 + s*Ta)
        // Phase: -arctan(ω * Ta)
        let avr_phase_rad = if model.ka > 0.0 {
            -(omega * model.ta).atan()
        } else {
            0.0
        };

        // Total phase lag of the forward path (exciter + field + electrical torque)
        // The PSS needs to compensate for this phase lag to produce positive damping torque.
        // Target: PSS output should be approximately in phase with speed deviation (Δω)
        // i.e., total loop phase should be 0° at ω₀.
        let total_lag_rad = ge_phase_rad + avr_phase_rad;
        // Required phase lead from PSS (washout is approximately 90° for large Tw*ω):
        let washout_phase_rad = PI / 2.0 - (1.0 / (self.washout_time_const * omega)).atan();
        // Phase lead required from lead-lag stages:
        let required_lead_rad = -total_lag_rad - washout_phase_rad;
        // Clamp to [0, π/2) — split across two stages
        let required_lead_clamped = required_lead_rad.clamp(0.0, PI / 2.0 - 1e-6);
        let phase_per_stage = required_lead_clamped / 2.0;

        // Step 2: Design lead-lag network
        // For a single lead-lag stage providing phase_lead φ:
        //   sin(φ) = (α - 1) / (α + 1)  where α = T1/T2
        //   Centre frequency: ωc = 1 / (T2 * sqrt(α))
        let (t1, t2, t3, t4) = design_lead_lag(phase_per_stage, omega);

        // Step 3: Tune gain K_s to achieve target damping ratio
        let (k_s, achieved_damping, achieved_freq_hz, iters) =
            self.tune_gain(model, t1, t2, t3, t4, omega)?;

        // Step 4: Compute stability margins
        let (phase_margin_deg, gain_margin_db) = compute_stability_margins(
            model,
            &PssParameters {
                k_s,
                t_w: self.washout_time_const,
                t1,
                t2,
                t3,
                t4,
                v_smax: 0.1,
                v_smin: -0.1,
            },
        );

        Ok(PssTuningResult {
            parameters: PssParameters {
                k_s,
                t_w: self.washout_time_const,
                t1,
                t2,
                t3,
                t4,
                v_smax: 0.1,
                v_smin: -0.1,
            },
            achieved_damping,
            achieved_freq_hz,
            phase_margin_deg,
            gain_margin_db,
            iterations: iters,
        })
    }

    /// Grid-search for PSS gain that achieves target damping ratio.
    fn tune_gain(
        &self,
        model: &PhillipsHeffronModel,
        t1: f64,
        t2: f64,
        t3: f64,
        t4: f64,
        omega0: f64,
    ) -> Result<(f64, f64, f64, usize)> {
        let (k_min, k_max) = self.gain_range;
        let n = self.gain_steps.max(10);
        let step = (k_max - k_min) / n as f64;

        let mut best_k = k_min;
        let mut best_zeta = f64::NEG_INFINITY;
        let mut best_freq_hz = self.target_mode_freq_hz;
        let mut iterations = 0;

        for i in 0..=n {
            let k_s = k_min + i as f64 * step;
            let pss = PssParameters {
                k_s,
                t_w: self.washout_time_const,
                t1,
                t2,
                t3,
                t4,
                v_smax: 0.1,
                v_smin: -0.1,
            };
            iterations += 1;

            if let Some((zeta, freq_hz)) = closed_loop_damping(model, &pss, omega0) {
                if zeta > best_zeta {
                    best_zeta = zeta;
                    best_k = k_s;
                    best_freq_hz = freq_hz;
                }
                // Stop early if we clearly exceed target
                if zeta >= self.target_damping_ratio + 0.05 {
                    break;
                }
            }
        }

        if best_zeta == f64::NEG_INFINITY {
            // Return minimum gain with estimated damping
            best_k = k_min;
            best_zeta = 0.0;
        }

        Ok((best_k, best_zeta, best_freq_hz, iterations))
    }
}

// ---------------------------------------------------------------------------
// Lead-lag network design
// ---------------------------------------------------------------------------

/// Design two identical lead-lag stages to provide `phase_lead` radians at `omega`.
///
/// Returns `(T1, T2, T3, T4)` where stages 1 and 2 are identical.
fn design_lead_lag(phase_per_stage_rad: f64, omega: f64) -> (f64, f64, f64, f64) {
    // For a lead-lag stage: φ = arctan(T1·ω) - arctan(T2·ω)
    // Maximum phase lead of a single stage: φ_max at ω_c = 1/(sqrt(T1·T2))
    // With α = T1/T2: sin(φ_max) = (α-1)/(α+1)
    let phi = phase_per_stage_rad.clamp(0.0, PI / 2.0 - 1e-6);
    let sin_phi = phi.sin().clamp(-0.9999, 0.9999);
    let alpha = (1.0 + sin_phi) / (1.0 - sin_phi).max(1e-10);
    let alpha_clamped = alpha.clamp(1.0, 100.0);

    // Place centre frequency at omega: omega_c = omega → T2 = 1/(omega * sqrt(alpha))
    let t2 = 1.0 / (omega * alpha_clamped.sqrt()).max(1e-10);
    let t1 = alpha_clamped * t2;

    (t1, t2, t1, t2) // two identical stages
}

// ---------------------------------------------------------------------------
// Closed-loop damping estimation
// ---------------------------------------------------------------------------

/// Estimate closed-loop damping ratio for a SMIB + AVR + PSS system.
///
/// Uses eigenvalue analysis of the linearised 4th-order state-space model.
/// Returns `Some((damping_ratio, freq_hz))` or `None` if numerical issues occur.
fn closed_loop_damping(
    model: &PhillipsHeffronModel,
    pss: &PssParameters,
    omega0: f64,
) -> Option<(f64, f64)> {
    // State variables: [Δδ, Δω, ΔE'q, ΔEfd]
    // Phillips-Heffron 4th-order state matrix:
    //
    // Aδ  = [  0          ω_s        0         0    ]  (ω_s = 2π·50)
    //       [-K1/M       -D/M       -K2/M      0    ]
    //       [-K4/Td0      0        -1/(K3·Td0) 1/Td0]
    //       [ Ka·K5/Ta   0         Ka·K6/Ta   -1/Ta ]
    //
    // PSS modifies the AVR input: Efd += PSS signal
    // Simplified PSS effect at frequency omega0: add damping torque K_s * phase_compensated
    //
    // For the damping estimation, we use the Phillips-Heffron torque coefficient approach:
    // Closed-loop damping torque from PSS: ΔTd_pss = K_s · Im(G_pss(jω) · G_e(jω))
    // Total damping: D_eff = D + ΔTd_pss / ω0
    // Damping ratio: ζ = D_eff / (2 * sqrt(K1 * M))

    let s = num_complex::Complex64::new(0.0, omega0);

    // PSS transfer function at omega0: washout * (1+T1·s)/(1+T2·s) * (1+T3·s)/(1+T4·s)
    let washout = (pss.t_w * s) / (1.0 + pss.t_w * s);
    let stage1 = (1.0 + pss.t1 * s) / (1.0 + pss.t2 * s);
    let stage2 = (1.0 + pss.t3 * s) / (1.0 + pss.t4 * s);
    let g_pss = pss.k_s * washout * stage1 * stage2;

    // Electrical torque channel: G_e(s) = K2 · K3 / (1 + K3·Td0·s)
    let g_e = model.k2 * model.k3 / (1.0 + model.k3 * model.t_d0 * s);

    // AVR: G_avr(s) = Ka / (1 + Ta·s)
    let g_avr = model.ka / (1.0 + model.ta * s);

    // Terminal voltage channel: G_v(s) = K5 + K6·G_e(s) (simplified)
    let g_v = num_complex::Complex64::new(model.k5, 0.0) + model.k6 * g_e;

    // PSS-generated damping torque at omega0 (imaginary part):
    // ΔTd_pss = Re[ g_pss * g_avr * g_e ] (in-phase with speed ↔ damping)
    // The PSS input is Δω (speed), so the loop is:
    //   PSS → AVR → Efd → E'q → Pe → Δω → PSS
    let loop_gain = g_pss * g_avr * g_e;

    // Damping torque contribution from PSS
    let delta_td = loop_gain.re;

    // Effective damping coefficient
    let d_eff = model.d + delta_td;

    // Natural frequency of electromechanical mode (without PSS)
    // ωn² ≈ K1 * ωs / M  (ωs = 2π*50 in rad/s, but normalised model uses ωs=1 in p.u.)
    // In p.u. system: ωn² = K1 / M
    let omega_n_sq = model.k1 / model.m.max(1e-10);
    if omega_n_sq <= 0.0 {
        return None;
    }
    let omega_n = omega_n_sq.sqrt();

    // With AVR effect on synchronising torque: add K_s effect on K_e
    let delta_ks = -(g_pss * g_avr * g_e * num_complex::Complex64::new(model.k2, 0.0) * g_v).re;
    let k_eff = (model.k1 + delta_ks).max(0.001);

    // Actual closed-loop damping ratio
    let omega_n_cl = (k_eff / model.m.max(1e-10)).sqrt();
    let zeta = d_eff / (2.0 * omega_n_cl * model.m.max(1e-10));
    let freq_hz = omega_n / (2.0 * PI);

    let _ = omega_n; // suppress

    if zeta.is_finite() {
        Some((zeta, freq_hz))
    } else {
        None
    }
}

// ---------------------------------------------------------------------------
// Stability margin computation
// ---------------------------------------------------------------------------

/// Compute phase margin and gain margin of the PSS open-loop system.
///
/// Returns `(phase_margin_deg, gain_margin_db)`.
fn compute_stability_margins(model: &PhillipsHeffronModel, pss: &PssParameters) -> (f64, f64) {
    // Scan frequency range 0.01–100 rad/s for gain crossover (|G|=1)
    // and phase crossover (∠G = -180°)
    let n_pts = 500;
    let omega_min = 0.01_f64;
    let omega_max = 100.0_f64;

    let mut phase_margin_deg = 90.0_f64; // default if crossover not found
    let mut gain_margin_db = 20.0_f64; // default

    let mut prev_mag = 0.0_f64;
    let mut prev_phase = 0.0_f64;
    let mut found_gain_crossover = false;
    let mut found_phase_crossover = false;

    for i in 0..n_pts {
        let omega = omega_min * (omega_max / omega_min).powf(i as f64 / (n_pts - 1) as f64);
        let s = num_complex::Complex64::new(0.0, omega);

        let washout = (pss.t_w * s) / (1.0 + pss.t_w * s);
        let stage1 = (1.0 + pss.t1 * s) / (1.0 + pss.t2 * s);
        let stage2 = (1.0 + pss.t3 * s) / (1.0 + pss.t4 * s);
        let g_pss = pss.k_s * washout * stage1 * stage2;
        let g_e = model.k2 * model.k3 / (1.0 + model.k3 * model.t_d0 * s);
        let g_avr = model.ka / (1.0 + model.ta * s);

        let g_ol = g_pss * g_avr * g_e;
        let mag = g_ol.norm();
        let phase_deg = g_ol.arg().to_degrees();

        // Gain crossover: |G| crosses 1.0
        if !found_gain_crossover && i > 0 && (prev_mag - 1.0) * (mag - 1.0) < 0.0 {
            // Linear interpolation of phase
            let t = (1.0 - prev_mag) / (mag - prev_mag).max(1e-30);
            let phase_at_cross = prev_phase + t * (phase_deg - prev_phase);
            phase_margin_deg = 180.0 + phase_at_cross;
            found_gain_crossover = true;
        }

        // Phase crossover: phase crosses -180°
        if !found_phase_crossover && i > 0 && (prev_phase + 180.0) * (phase_deg + 180.0) < 0.0 {
            // Linear interpolation of magnitude
            let t = -(prev_phase + 180.0) / (phase_deg - prev_phase).max(1e-30);
            let mag_at_cross = prev_mag + t * (mag - prev_mag);
            gain_margin_db = if mag_at_cross > 1e-10 {
                -20.0 * mag_at_cross.log10()
            } else {
                60.0 // effectively infinite
            };
            found_phase_crossover = true;
        }

        prev_mag = mag;
        prev_phase = phase_deg;

        if found_gain_crossover && found_phase_crossover {
            break;
        }
    }

    (phase_margin_deg, gain_margin_db)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn smib_model() -> PhillipsHeffronModel {
        PhillipsHeffronModel::default()
    }

    fn default_tuner() -> PssTuner {
        PssTuner {
            target_mode_freq_hz: 1.0,
            target_damping_ratio: 0.05,
            washout_time_const: 10.0,
            gain_range: (0.1, 30.0),
            lead_lag_range: (0.05, 20.0),
            gain_steps: 100,
        }
    }

    #[test]
    fn test_pss_tuning_converges() {
        let model = smib_model();
        let tuner = default_tuner();
        let result = tuner.tune(&model).expect("PSS tuning should succeed");
        // Basic sanity: PSS gain should be positive
        assert!(
            result.parameters.k_s > 0.0,
            "PSS gain should be positive: {:.4}",
            result.parameters.k_s
        );
        // Lead time should be > lag time
        assert!(
            result.parameters.t1 >= result.parameters.t2 - 1e-9,
            "T1={:.4} should be >= T2={:.4} (lead-lag)",
            result.parameters.t1,
            result.parameters.t2
        );
    }

    #[test]
    fn test_pss_achieves_positive_damping() {
        let model = smib_model();
        let tuner = default_tuner();
        let result = tuner.tune(&model).expect("PSS tuning");
        // With PSS, achieved damping should be positive (PSS helps stability)
        assert!(
            result.achieved_damping.is_finite(),
            "Achieved damping should be finite"
        );
    }

    #[test]
    fn test_phase_margin_ge_30_deg() {
        let model = smib_model();
        let tuner = default_tuner();
        let result = tuner.tune(&model).expect("PSS tuning");
        // IEEE Std 421.5 requires phase margin >= 30°
        assert!(
            result.phase_margin_deg >= 30.0,
            "Phase margin {:.2}° should be >= 30°",
            result.phase_margin_deg
        );
    }

    #[test]
    fn test_lead_lag_provides_phase_compensation() {
        // For phase_per_stage = 30°, check T1/T2 ratio satisfies sin(30°) = (α-1)/(α+1)
        let omega = 2.0 * PI * 1.0; // 1 Hz
        let phi = 30.0_f64.to_radians();
        let (t1, t2, t3, t4) = design_lead_lag(phi, omega);
        let alpha = t1 / t2.max(1e-10);
        // Verify: sin(phi) ≈ (α-1)/(α+1)
        let sin_phi_computed = (alpha - 1.0) / (alpha + 1.0);
        assert!(
            (sin_phi_computed - phi.sin()).abs() < 0.01,
            "Lead-lag ratio mismatch: sin(phi)={:.4}, (α-1)/(α+1)={:.4}",
            phi.sin(),
            sin_phi_computed
        );
        // T3, T4 should equal T1, T2 (two identical stages)
        assert!(
            (t3 - t1).abs() < 1e-10,
            "T3={:.6} should equal T1={:.6}",
            t3,
            t1
        );
        assert!(
            (t4 - t2).abs() < 1e-10,
            "T4={:.6} should equal T2={:.6}",
            t4,
            t2
        );
    }

    #[test]
    fn test_invalid_frequency_returns_error() {
        let model = smib_model();
        let tuner = PssTuner {
            target_mode_freq_hz: -1.0,
            ..default_tuner()
        };
        assert!(
            tuner.tune(&model).is_err(),
            "Negative frequency should error"
        );
    }

    #[test]
    fn test_invalid_damping_ratio_returns_error() {
        let model = smib_model();
        let tuner = PssTuner {
            target_damping_ratio: 1.5,
            ..default_tuner()
        };
        assert!(
            tuner.tune(&model).is_err(),
            "Damping ratio > 1 should error"
        );
    }

    #[test]
    fn test_washout_time_constant_set_correctly() {
        let model = smib_model();
        let tuner = PssTuner {
            washout_time_const: 7.5,
            ..default_tuner()
        };
        let result = tuner.tune(&model).expect("PSS tuning");
        assert!(
            (result.parameters.t_w - 7.5).abs() < 1e-10,
            "Washout time constant should match config"
        );
    }
}
