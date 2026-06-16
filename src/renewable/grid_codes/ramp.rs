/// Ramp rate requirements for renewable energy generators.
///
/// Grid codes limit how quickly renewable generators may increase or decrease
/// their active power output to prevent destabilising fast power swings. Ramp
/// rate limits are expressed as a percentage of rated capacity per minute.
///
/// # References
/// - ENTSO-E, "Requirements for Generators" (RfG), Network Code 2016
/// - IEC 61400-21: "Wind turbines — Measurement and assessment of power quality"
/// - BDEW, "Technical Guideline: Generating Plants Connected to the Medium-Voltage
///   Network", 2008
use serde::{Deserialize, Serialize};

/// Ramp rate requirement for a renewable generator.
///
/// Both upward and downward ramps are limited, and an emergency ramp rate
/// (e.g. for fault recovery) may be specified separately.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RampRateRequirement {
    /// Maximum ramp-up rate [% of rated power per minute].
    pub max_ramp_up_pct_per_min: f64,
    /// Maximum ramp-down rate [% of rated power per minute].
    pub max_ramp_down_pct_per_min: f64,
    /// Emergency ramp rate allowed during grid events [% per minute].
    /// This may be higher than the normal limits.
    pub emergency_ramp_pct_per_min: f64,
    /// Whether automatic curtailment should be applied when the ramp limit
    /// would otherwise be violated.
    pub gradient_protection: bool,
}

impl RampRateRequirement {
    /// Standard ramp rate requirement (10 %/min up and down).
    ///
    /// Typical for medium-sized wind and solar farms.
    pub fn standard() -> Self {
        Self {
            max_ramp_up_pct_per_min: 10.0,
            max_ramp_down_pct_per_min: 10.0,
            emergency_ramp_pct_per_min: 20.0,
            gradient_protection: true,
        }
    }

    /// Strict ramp rate requirement (3 %/min) for large wind farms.
    ///
    /// Applied in some European grid codes for wind farms above 10 MW.
    pub fn strict() -> Self {
        Self {
            max_ramp_up_pct_per_min: 3.0,
            max_ramp_down_pct_per_min: 3.0,
            emergency_ramp_pct_per_min: 10.0,
            gradient_protection: true,
        }
    }

    /// Convert a ramp limit from %/min to pu/s (for per-unit power, per second).
    fn pct_per_min_to_pu_per_s(pct_per_min: f64) -> f64 {
        pct_per_min / 100.0 / 60.0
    }

    /// Check whether each consecutive step in a power trajectory satisfies the ramp limits.
    ///
    /// Returns a `Vec<bool>` of length `power_pu.len()` where:
    /// - Index 0 is always `true` (no prior step to compare).
    /// - Index `i` is `true` if the ramp from step `i-1` to `i` is within limits.
    ///
    /// # Arguments
    /// - `power_pu` — per-unit active power trajectory
    /// - `dt_s`     — time step \[seconds\]
    pub fn check_trajectory(&self, power_pu: &[f64], dt_s: f64) -> Vec<bool> {
        if power_pu.is_empty() {
            return Vec::new();
        }
        let max_up_pu_s = Self::pct_per_min_to_pu_per_s(self.max_ramp_up_pct_per_min);
        let max_down_pu_s = Self::pct_per_min_to_pu_per_s(self.max_ramp_down_pct_per_min);

        let mut result = Vec::with_capacity(power_pu.len());
        result.push(true); // first step has no prior

        for window in power_pu.windows(2) {
            let delta = window[1] - window[0];
            let ramp_pu_s = if dt_s > 1e-12 { delta / dt_s } else { 0.0 };
            let ok = if ramp_pu_s > 0.0 {
                ramp_pu_s <= max_up_pu_s
            } else {
                ramp_pu_s.abs() <= max_down_pu_s
            };
            result.push(ok);
        }
        result
    }

    /// Limit a power trajectory to respect ramp rate constraints.
    ///
    /// Where the natural trajectory would violate the ramp limit, the output is
    /// clipped to the maximum allowed ramp. The returned trajectory is guaranteed
    /// to satisfy the ramp constraints for every step.
    ///
    /// # Arguments
    /// - `power_pu` — desired per-unit active power trajectory
    /// - `dt_s`     — time step \[seconds\]
    pub fn enforce_ramp_limits(&self, power_pu: &[f64], dt_s: f64) -> Vec<f64> {
        if power_pu.is_empty() {
            return Vec::new();
        }
        let max_up_pu_s = Self::pct_per_min_to_pu_per_s(self.max_ramp_up_pct_per_min);
        let max_down_pu_s = Self::pct_per_min_to_pu_per_s(self.max_ramp_down_pct_per_min);
        let max_up_step = max_up_pu_s * dt_s;
        let max_down_step = max_down_pu_s * dt_s;

        let mut enforced = Vec::with_capacity(power_pu.len());
        enforced.push(power_pu[0]);

        for &p_desired in &power_pu[1..] {
            let p_prev = *enforced.last().unwrap_or(&0.0);
            let delta = p_desired - p_prev;
            let clipped_delta = if delta > 0.0 {
                delta.min(max_up_step)
            } else {
                delta.max(-max_down_step)
            };
            enforced.push(p_prev + clipped_delta);
        }
        enforced
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ramp_rate_enforcement() {
        // Standard: 10%/min → 0.1/60 pu/s ≈ 1.667e-3 pu/s
        // With dt=60 s: max step = 0.1 pu
        let req = RampRateRequirement::standard();
        let trajectory = vec![0.0, 0.5, 1.0]; // wants to jump 0.5 in 60 s each step

        let enforced = req.enforce_ramp_limits(&trajectory, 60.0);
        assert_eq!(enforced.len(), 3);
        assert!((enforced[0] - 0.0).abs() < 1e-9);
        // Max step is 0.10 pu, so first enforced step ≤ 0.10 pu from 0.0
        assert!(
            (enforced[1] - enforced[0]).abs() <= 0.10 + 1e-9,
            "Ramp should be clipped: enforced[1]={:.4}",
            enforced[1]
        );
    }

    #[test]
    fn test_ramp_rate_enforcement_gradual_is_unchanged() {
        // Very gradual ramp: 1%/min over 10 min → easily within 10%/min limit
        let req = RampRateRequirement::standard();
        let trajectory: Vec<f64> = (0..=10).map(|i| i as f64 * 0.01).collect(); // 0.01 per 60s step
        let enforced = req.enforce_ramp_limits(&trajectory, 60.0);
        for (orig, enf) in trajectory.iter().zip(enforced.iter()) {
            assert!(
                (orig - enf).abs() < 1e-9,
                "Gradual ramp should be unchanged: orig={}, enf={}",
                orig,
                enf
            );
        }
    }

    #[test]
    fn test_check_trajectory_detects_violation() {
        let req = RampRateRequirement::standard(); // 10%/min
                                                   // Step of 0.5 pu in 1 second = 0.5 pu/s >> 0.1/60 pu/s
        let traj = vec![0.0, 0.5, 0.6];
        let ok = req.check_trajectory(&traj, 1.0);
        assert_eq!(ok.len(), 3);
        assert!(ok[0], "First step should always be true");
        assert!(!ok[1], "0.5 pu in 1 s should fail 10%/min ramp limit");
    }

    #[test]
    fn test_check_trajectory_gradual_all_ok() {
        let req = RampRateRequirement::strict(); // 3%/min → 0.03/60 pu/s
                                                 // 0.001 pu steps every 60 s → 0.001/60 pu/s << limit
        let traj: Vec<f64> = (0..5).map(|i| i as f64 * 0.001).collect();
        let ok = req.check_trajectory(&traj, 60.0);
        assert!(
            ok.iter().all(|&b| b),
            "All steps within strict limit should be OK"
        );
    }

    #[test]
    fn test_ramp_down_enforced() {
        let req = RampRateRequirement::standard(); // 10%/min
                                                   // Step down from 1.0 to 0.0 in 60 s — should be clipped
        let traj = vec![1.0, 0.0];
        let enforced = req.enforce_ramp_limits(&traj, 60.0);
        assert_eq!(enforced.len(), 2);
        let ramp = enforced[0] - enforced[1];
        assert!(
            ramp <= 0.10 + 1e-9,
            "Ramp down should be clipped to 0.10 pu/step: got {:.4}",
            ramp
        );
    }

    // ---- 7 additional unit tests ----

    /// Gap 1: empty trajectory returns empty Vec from both methods.
    #[test]
    fn test_empty_trajectory_returns_empty_vecs() {
        let req = RampRateRequirement::standard();
        let empty: Vec<f64> = Vec::new();
        let checked = req.check_trajectory(&empty, 60.0);
        assert!(
            checked.is_empty(),
            "check_trajectory on empty slice must return empty Vec"
        );
        let enforced = req.enforce_ramp_limits(&empty, 60.0);
        assert!(
            enforced.is_empty(),
            "enforce_ramp_limits on empty slice must return empty Vec"
        );
    }

    /// Gap 2: single-element trajectory is always [true] / [p[0]].
    #[test]
    fn test_single_element_trajectory() {
        let req = RampRateRequirement::standard();
        let single = vec![0.42_f64];

        let checked = req.check_trajectory(&single, 60.0);
        assert_eq!(checked.len(), 1, "check result length must be 1");
        assert!(checked[0], "single-element check must always be true");

        let enforced = req.enforce_ramp_limits(&single, 60.0);
        assert_eq!(enforced.len(), 1, "enforce result length must be 1");
        assert!(
            (enforced[0] - 0.42).abs() < 1e-12,
            "single element must be returned unchanged: got {:.9}",
            enforced[0]
        );
    }

    /// Gap 3: strict() preset fields are correctly initialised.
    #[test]
    fn test_strict_preset_fields() {
        let req = RampRateRequirement::strict();
        assert!(
            (req.max_ramp_up_pct_per_min - 3.0).abs() < 1e-12,
            "strict() up rate must be 3.0 %/min, got {}",
            req.max_ramp_up_pct_per_min
        );
        assert!(
            (req.max_ramp_down_pct_per_min - 3.0).abs() < 1e-12,
            "strict() down rate must be 3.0 %/min, got {}",
            req.max_ramp_down_pct_per_min
        );
        assert!(
            (req.emergency_ramp_pct_per_min - 10.0).abs() < 1e-12,
            "strict() emergency rate must be 10.0 %/min, got {}",
            req.emergency_ramp_pct_per_min
        );
        assert!(
            req.gradient_protection,
            "strict() must have gradient_protection = true"
        );
    }

    /// Gap 4: ramp-down violation detected by check_trajectory (strict, 3 %/min).
    ///
    /// strict() allows at most 0.03 pu per 60 s step downward.
    /// A drop of 0.10 pu in 60 s must be flagged.
    #[test]
    fn test_ramp_down_violation_detected_strict() {
        let req = RampRateRequirement::strict();
        let traj = vec![0.50_f64, 0.40]; // −0.10 pu in 60 s
        let ok = req.check_trajectory(&traj, 60.0);
        assert_eq!(ok.len(), 2, "result length must match trajectory length");
        assert!(ok[0], "first element is always true");
        assert!(
            !ok[1],
            "−0.10 pu drop in 60 s must violate the 3 %/min down limit"
        );
    }

    /// Gap 5: enforce_ramp_limits output is monotone when enforcing a step-up sequence.
    ///
    /// Each enforced value must be >= the previous one (non-decreasing) and
    /// the step between consecutive values must not exceed 0.10 pu (10 %/min, dt=60 s).
    #[test]
    fn test_enforce_ramp_up_monotone() {
        let req = RampRateRequirement::standard(); // 10 %/min, max step = 0.10 pu @ dt=60 s
        let desired = vec![0.0_f64, 0.5, 1.0, 1.5, 2.0];
        let enforced = req.enforce_ramp_limits(&desired, 60.0);
        assert_eq!(enforced.len(), desired.len(), "length must be preserved");
        for i in 1..enforced.len() {
            let step = enforced[i] - enforced[i - 1];
            assert!(
                step >= -1e-12,
                "enforced trajectory must be non-decreasing at index {}: step={:.9}",
                i,
                step
            );
            assert!(
                step <= 0.10 + 1e-9,
                "enforced step must not exceed 0.10 pu at index {}: step={:.9}",
                i,
                step
            );
        }
    }

    /// Gap 6: asymmetric limits — downward step within the up-limit but beyond the down-limit
    /// is correctly flagged as a violation.
    ///
    /// Custom: 5 %/min up / 2 %/min down. Allowed down-step at dt=60 s: 0.02 pu.
    /// A decrease of 0.04 pu passes the up-limit (5 % = 0.05 pu) but fails the down-limit.
    #[test]
    fn test_asymmetric_limits_downward_violation() {
        let req = RampRateRequirement {
            max_ramp_up_pct_per_min: 5.0,
            max_ramp_down_pct_per_min: 2.0,
            emergency_ramp_pct_per_min: 15.0,
            gradient_protection: true,
        };
        let traj = vec![0.50_f64, 0.46]; // −0.04 pu in 60 s
        let ok = req.check_trajectory(&traj, 60.0);
        assert_eq!(ok.len(), 2, "result length must match trajectory length");
        assert!(ok[0], "first element is always true");
        assert!(
            !ok[1],
            "−0.04 pu step in 60 s must violate the 2 %/min down limit (limit is 0.02 pu)"
        );
    }

    /// Gap 7: an enforced trajectory re-verified with check_trajectory passes all steps.
    #[test]
    fn test_enforced_trajectory_passes_check() {
        let req = RampRateRequirement::standard(); // 10 %/min, dt = 60 s
        let desired = vec![0.0_f64, 1.0, 0.0, 1.0, 0.5];
        let enforced = req.enforce_ramp_limits(&desired, 60.0);
        let ok = req.check_trajectory(&enforced, 60.0);
        assert_eq!(
            ok.len(),
            enforced.len(),
            "check result length must match enforced length"
        );
        for (i, &pass) in ok.iter().enumerate() {
            assert!(
                pass,
                "enforced trajectory must pass check at index {}: value={:.9}",
                i, enforced[i]
            );
        }
    }
}
