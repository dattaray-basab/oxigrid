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
}
