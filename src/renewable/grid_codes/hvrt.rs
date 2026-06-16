/// High Voltage Ride-Through (HVRT) grid code compliance.
///
/// HVRT requirements define the maximum voltage that a renewable energy source
/// must withstand without disconnecting. During voltage swells (e.g., due to load
/// rejection or Ferranti effect), inverters must absorb reactive power to limit
/// further voltage rise.
///
/// # References
/// - ENTSO-E, "Requirements for Generators" (RfG), Network Code 2016
/// - BDEW, "Technical Guideline: Generating Plants Connected to the
///   Medium-Voltage Network", 2008
use serde::{Deserialize, Serialize};

/// HVRT (High Voltage Ride-Through) requirement profile.
///
/// Defines the maximum voltage that an inverter must withstand for each duration.
/// The `profile_points` form a piecewise-linear boundary in (time_s, max_voltage_pu).
/// If the measured voltage stays at or below the envelope, the inverter must remain
/// connected.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HvrtProfile {
    /// Human-readable profile name.
    pub name: String,
    /// Piecewise-linear (time_s, max_voltage_pu) envelope.
    ///
    /// Must stay connected when `V <= max_voltage_pu` for the given duration.
    pub profile_points: Vec<(f64, f64)>,
    /// Whether reactive power absorption is required during the swell.
    pub absorb_reactive_required: bool,
}

impl HvrtProfile {
    /// ENTSO-E RfG HVRT profile.
    ///
    /// Must withstand 1.20 pu for 0.1 s and 1.15 pu for 0.9 s.
    pub fn entso_e() -> Self {
        Self {
            name: "ENTSO-E RfG HVRT".to_string(),
            profile_points: vec![
                (0.0, 1.20),
                (0.1, 1.20),
                (0.1, 1.15),
                (0.9, 1.15),
                (60.0, 1.10),
            ],
            absorb_reactive_required: true,
        }
    }

    /// German BDEW medium-voltage HVRT profile.
    pub fn bdew_mv() -> Self {
        Self {
            name: "BDEW MV HVRT".to_string(),
            profile_points: vec![
                (0.0, 1.20),
                (0.1, 1.20),
                (0.1, 1.15),
                (1.0, 1.15),
                (60.0, 1.10),
            ],
            absorb_reactive_required: true,
        }
    }

    /// Evaluate the maximum allowed voltage at a given time via linear interpolation.
    fn max_voltage_at(&self, time_s: f64) -> f64 {
        if self.profile_points.is_empty() {
            return 1.15; // safe default
        }
        if time_s <= self.profile_points[0].0 {
            return self.profile_points[0].1;
        }
        let last = self.profile_points[self.profile_points.len() - 1];
        if time_s >= last.0 {
            return last.1;
        }
        for window in self.profile_points.windows(2) {
            let (t0, v0) = window[0];
            let (t1, v1) = window[1];
            if time_s >= t0 && time_s <= t1 {
                let alpha = if (t1 - t0).abs() < 1e-12 {
                    1.0
                } else {
                    (time_s - t0) / (t1 - t0)
                };
                return v0 + alpha * (v1 - v0);
            }
        }
        last.1
    }

    /// Check whether a voltage swell event is HVRT-compliant.
    ///
    /// For each time sample the measured voltage is compared against the profile
    /// ceiling. If voltage exceeds the maximum at any sample the event is
    /// non-compliant and disconnection is permitted.
    pub fn passes_hvrt(&self, event: &HvrtEvent) -> HvrtResult {
        let mut violated_at: Option<f64> = None;
        let mut total_reactive_mw = 0.0_f64;

        let n = event.time_profile.len();
        for (idx, &(t, v)) in event.time_profile.iter().enumerate() {
            let v_max = self.max_voltage_at(t);

            // Estimate reactive absorption proportional to overvoltage
            // ΔQ ≈ (V - 1.0) * 0.1 (simplified linear model in pu → MW/MVAr)
            let dv_over = (v - 1.0).max(0.0);
            let dt = if idx + 1 < n {
                event.time_profile[idx + 1].0 - t
            } else if idx > 0 {
                t - event.time_profile[idx - 1].0
            } else {
                1.0
            };
            total_reactive_mw += dv_over * 0.1 * dt;

            if v > v_max && violated_at.is_none() {
                violated_at = Some(t);
            }
        }

        HvrtResult {
            compliant: violated_at.is_none(),
            violated_at_s: violated_at,
            reactive_absorption_mw: total_reactive_mw,
        }
    }
}

/// A voltage swell event to evaluate against an HVRT profile.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HvrtEvent {
    /// Time-series of (time_s, voltage_pu) measurements during the event.
    pub time_profile: Vec<(f64, f64)>,
}

/// Result of evaluating an HVRT event against a profile.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HvrtResult {
    /// Whether the event stays within the HVRT envelope (no disconnection needed).
    pub compliant: bool,
    /// Time (s) of the first voltage-ceiling violation, if any.
    pub violated_at_s: Option<f64>,
    /// Estimated reactive power absorbed during the swell \[MVAr\].
    pub reactive_absorption_mw: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hvrt_entso_passes() {
        // 1.1 pu for 0.5 s — well below both the 1.20/0.1 s and 1.15/0.9 s limits
        let profile = HvrtProfile::entso_e();
        let event = HvrtEvent {
            time_profile: vec![
                (0.0, 1.0),
                (0.1, 1.1),
                (0.3, 1.1),
                (0.5, 1.1),
                (0.6, 1.05),
                (1.0, 1.0),
            ],
        };
        let result = profile.passes_hvrt(&event);
        assert!(
            result.compliant,
            "1.1 pu for 0.5 s should pass ENTSO-E HVRT"
        );
    }

    #[test]
    fn test_hvrt_entso_fails_sustained_overvoltage() {
        // 1.22 pu even briefly exceeds the 1.20 pu instantaneous limit
        let profile = HvrtProfile::entso_e();
        let event = HvrtEvent {
            time_profile: vec![
                (0.0, 1.0),
                (0.05, 1.22), // exceeds 1.20 ceiling immediately
                (0.15, 1.18),
                (1.0, 1.0),
            ],
        };
        let result = profile.passes_hvrt(&event);
        assert!(
            !result.compliant,
            "1.22 pu should fail ENTSO-E HVRT limit of 1.20 pu"
        );
        assert!(result.violated_at_s.is_some());
    }

    #[test]
    fn test_hvrt_reactive_absorption_positive() {
        let profile = HvrtProfile::entso_e();
        let event = HvrtEvent {
            time_profile: vec![(0.0, 1.0), (0.5, 1.15), (1.0, 1.10), (2.0, 1.0)],
        };
        let result = profile.passes_hvrt(&event);
        assert!(
            result.reactive_absorption_mw >= 0.0,
            "Reactive absorption should be non-negative"
        );
    }

    #[test]
    fn test_hvrt_at_exactly_1_10_pu_passes() {
        // At t=60 s the ENTSO-E ceiling drops to 1.10 pu; a voltage sitting
        // at exactly 1.10 pu must still be considered compliant (v <= v_max).
        let profile = HvrtProfile::entso_e();
        let event = HvrtEvent {
            time_profile: vec![
                (0.0, 1.0),
                (10.0, 1.10),
                (60.0, 1.10),
            ],
        };
        let result = profile.passes_hvrt(&event);
        assert!(
            result.compliant,
            "Voltage at exactly 1.10 pu at t=60 s should be compliant"
        );
        assert!(result.violated_at_s.is_none());
    }

    #[test]
    fn test_hvrt_above_1_20_pu_disconnection_permitted() {
        // 1.21 pu at t=0.05 s exceeds the 1.20 pu ceiling → non-compliant.
        let profile = HvrtProfile::entso_e();
        let event = HvrtEvent {
            time_profile: vec![
                (0.0, 1.0),
                (0.05, 1.21),
                (0.5, 1.05),
                (1.0, 1.0),
            ],
        };
        let result = profile.passes_hvrt(&event);
        assert!(
            !result.compliant,
            "1.21 pu at t=0.05 s must be non-compliant (ceiling is 1.20 pu)"
        );
        assert!(
            result.violated_at_s.is_some(),
            "violated_at_s must be Some for a ceiling breach"
        );
    }

    #[test]
    fn test_hvrt_normal_range_0_9_to_1_1_pu_passes() {
        // Voltages in [0.9, 1.1] pu are below every ceiling on both profiles.
        let entso_e = HvrtProfile::entso_e();
        let bdew = HvrtProfile::bdew_mv();
        let event = HvrtEvent {
            time_profile: vec![
                (0.0, 0.95),
                (0.5, 1.0),
                (1.0, 1.05),
                (10.0, 1.1),
                (60.0, 1.0),
            ],
        };
        let r_entso = entso_e.passes_hvrt(&event);
        let r_bdew = bdew.passes_hvrt(&event);
        assert!(r_entso.compliant, "Normal range must pass ENTSO-E HVRT");
        assert!(r_bdew.compliant, "Normal range must pass BDEW HVRT");
    }

    #[test]
    fn test_hvrt_ride_through_duration_at_1_15_pu() {
        // 1.15 pu from t=0.1 s to t=0.89 s is just within the ENTSO-E window
        // (ceiling is 1.15 pu for t in [0.1, 0.9]).
        let profile = HvrtProfile::entso_e();
        let event = HvrtEvent {
            time_profile: vec![
                (0.0, 1.0),
                (0.1, 1.15),
                (0.5, 1.15),
                (0.89, 1.15),
                (1.0, 1.05),
            ],
        };
        let result = profile.passes_hvrt(&event);
        assert!(
            result.compliant,
            "1.15 pu up to t=0.89 s must be compliant under ENTSO-E (ceiling is 1.15 pu)"
        );
    }

    #[test]
    fn test_hvrt_bdew_differs_from_entso_e() {
        // BDEW uses (1.0, 1.15) while ENTSO-E uses (0.9, 1.15) — profiles differ.
        let entso_e = HvrtProfile::entso_e();
        let bdew = HvrtProfile::bdew_mv();
        assert_ne!(
            entso_e.profile_points, bdew.profile_points,
            "ENTSO-E and BDEW profiles must have different ride-through windows"
        );
    }

    #[test]
    fn test_hvrt_instantaneous_above_1_3_pu_non_compliant() {
        // A spike to 1.35 pu at t=0.02 s is well above the 1.20 pu ceiling.
        let profile = HvrtProfile::entso_e();
        let event = HvrtEvent {
            time_profile: vec![
                (0.0, 1.0),
                (0.02, 1.35),
                (0.5, 1.05),
                (1.0, 1.0),
            ],
        };
        let result = profile.passes_hvrt(&event);
        assert!(
            !result.compliant,
            "1.35 pu spike must be non-compliant"
        );
        let violated = result
            .violated_at_s
            .expect("violated_at_s must be Some for 1.35 pu spike");
        assert!(
            (violated - 0.02).abs() < 1e-9,
            "First violation must be at t=0.02 s, got {violated}"
        );
    }

    #[test]
    fn test_hvrt_continuous_operation_band_no_reactive_event() {
        // Steady operation at 0.95–1.05 pu: compliant, reactive absorption non-negative.
        let profile = HvrtProfile::entso_e();
        let event = HvrtEvent {
            time_profile: vec![
                (0.0, 0.95),
                (10.0, 1.0),
                (30.0, 1.05),
                (60.0, 1.0),
            ],
        };
        let result = profile.passes_hvrt(&event);
        assert!(result.compliant, "Normal operation band must be compliant");
        assert!(
            result.reactive_absorption_mw >= 0.0,
            "Reactive absorption must be non-negative, got {}",
            result.reactive_absorption_mw
        );
    }

    #[test]
    fn test_hvrt_empty_event_is_compliant() {
        let profile = HvrtProfile::entso_e();
        let event = HvrtEvent {
            time_profile: vec![],
        };
        let result = profile.passes_hvrt(&event);
        assert!(result.compliant, "Empty event should be compliant");
        assert!(
            result.violated_at_s.is_none(),
            "Empty event must not report a violation time"
        );
    }

    #[test]
    fn test_hvrt_single_sample_below_ceiling() {
        let profile = HvrtProfile::entso_e();
        let event = HvrtEvent {
            time_profile: vec![(0.0, 1.05)],
        };
        let result = profile.passes_hvrt(&event);
        assert!(
            result.compliant,
            "Single sample at 1.05 pu should be compliant on ENTSO-E profile"
        );
    }

    #[test]
    fn test_hvrt_linear_interpolation_midpoint() {
        // ENTSO-E ceiling between t=0.9 s and t=60.0 s interpolates linearly
        // from 1.15 pu down to 1.10 pu.
        // Midpoint t=30.45 s  =>  ceiling ≈ 1.125 pu.
        // A voltage of 1.12 pu at that instant is strictly below the ceiling.
        let profile = HvrtProfile::entso_e();
        let event = HvrtEvent {
            time_profile: vec![(0.0, 1.0), (30.45, 1.12), (60.0, 1.0)],
        };
        let result = profile.passes_hvrt(&event);
        assert!(
            result.compliant,
            "1.12 pu at t=30.45 s is below the interpolated ENTSO-E ceiling (~1.125 pu)"
        );
    }

    #[test]
    fn test_hvrt_bdew_ceiling_at_t_1s() {
        // BDEW MV: ceiling at t=1.0 s is exactly 1.15 pu.
        // A voltage equal to the ceiling must be considered compliant.
        let profile = HvrtProfile::bdew_mv();
        let event = HvrtEvent {
            time_profile: vec![(0.0, 1.0), (1.0, 1.15), (2.0, 1.0)],
        };
        let result = profile.passes_hvrt(&event);
        assert!(
            result.compliant,
            "1.15 pu at t=1.0 s equals the BDEW MV ceiling and must be compliant"
        );
    }

    #[test]
    fn test_hvrt_reactive_absorption_increases_with_overvoltage() {
        // Higher overvoltage depth must drive more reactive power absorption.
        let profile = HvrtProfile::entso_e();

        let event_low = HvrtEvent {
            time_profile: vec![(0.0, 1.0), (0.5, 1.05), (1.0, 1.0)],
        };
        let event_high = HvrtEvent {
            time_profile: vec![(0.0, 1.0), (0.5, 1.15), (1.0, 1.0)],
        };

        let r_low = profile.passes_hvrt(&event_low);
        let r_high = profile.passes_hvrt(&event_high);

        assert!(
            r_high.reactive_absorption_mw > r_low.reactive_absorption_mw,
            "Higher overvoltage ({} pu peak) must require more reactive absorption than \
             lower overvoltage ({} pu peak): got {:.6} vs {:.6}",
            1.15,
            1.05,
            r_high.reactive_absorption_mw,
            r_low.reactive_absorption_mw
        );
    }
}
