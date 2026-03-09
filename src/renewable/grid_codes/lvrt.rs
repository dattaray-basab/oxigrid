/// Low Voltage Ride-Through (LVRT) grid code compliance.
///
/// LVRT requirements define the minimum voltage envelope that a renewable
/// energy source must withstand without disconnecting. During faults,
/// inverters must inject reactive current to support grid voltage.
///
/// # References
/// - IEC 61400-21: "Measurement and assessment of power quality characteristics
///   of grid connected wind turbines"
/// - ENTSO-E, "Requirements for Generators" (RfG), 2016
/// - NERC, "Reliability Standard PRC-024-2", 2015
/// - BDEW, "Technical Guideline: Generating Plants Connected to the Medium-Voltage
///   Network", 2008
use serde::{Deserialize, Serialize};

/// LVRT requirement profile (voltage vs. time envelope).
///
/// Defines the minimum voltage that an inverter must withstand for each duration.
/// The `profile_points` form a piecewise-linear boundary in the (time, voltage)
/// plane. If the measured voltage remains at or above the envelope at all times,
/// the event is compliant and the inverter must stay connected.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LvrtProfile {
    /// Human-readable profile name, e.g. "IEC 61400-21" or "ENTSO-E".
    pub name: String,
    /// Piecewise-linear (time_s, min_voltage_pu) envelope.
    ///
    /// Must stay connected when `V >= min_voltage_pu` for `t <= time_s`.
    pub profile_points: Vec<(f64, f64)>,
    /// Whether reactive current injection is required during the fault.
    pub reactive_injection_required: bool,
    /// Reactive current gain: `ΔIq = k * ΔV` (default 2.0 for European codes).
    pub reactive_k_factor: f64,
}

impl LvrtProfile {
    /// IEC 61400-21 LVRT profile for wind turbines.
    ///
    /// Allows zero voltage for 150 ms, then stepwise recovery to nominal.
    pub fn iec61400() -> Self {
        Self {
            name: "IEC 61400-21".to_string(),
            profile_points: vec![
                (0.0, 0.0),
                (0.15, 0.0),
                (0.15, 0.85),
                (0.5, 0.85),
                (1.5, 0.9),
                (3.0, 1.0),
            ],
            reactive_injection_required: true,
            reactive_k_factor: 2.0,
        }
    }

    /// ENTSO-E Requirements for Generators (RfG) LVRT profile.
    ///
    /// Applies to power-park modules (PPM) in Type B–D categories.
    pub fn entso_e() -> Self {
        Self {
            name: "ENTSO-E RfG".to_string(),
            profile_points: vec![
                (0.0, 0.0),
                (0.14, 0.0),
                (0.14, 0.85),
                (0.45, 0.85),
                (1.5, 0.9),
                (3.0, 1.05),
            ],
            reactive_injection_required: true,
            reactive_k_factor: 2.0,
        }
    }

    /// US NERC PRC-024-2 LVRT requirement.
    ///
    /// Allows zero voltage for 625 ms without disconnection.
    pub fn nerc() -> Self {
        Self {
            name: "NERC PRC-024-2".to_string(),
            profile_points: vec![
                (0.0, 0.0),
                (0.625, 0.0),
                (0.625, 0.15),
                (3.0, 0.9),
                (10.0, 0.9),
            ],
            reactive_injection_required: false,
            reactive_k_factor: 0.0,
        }
    }

    /// German BDEW medium-voltage LVRT profile.
    ///
    /// Applies to generating plants connected to MV networks (≥ 100 kW).
    pub fn bdew_mv() -> Self {
        Self {
            name: "BDEW MV".to_string(),
            profile_points: vec![
                (0.0, 0.0),
                (0.15, 0.0),
                (0.15, 0.8),
                (0.7, 0.8),
                (1.5, 0.9),
                (3.0, 1.0),
            ],
            reactive_injection_required: true,
            reactive_k_factor: 2.0,
        }
    }

    /// Evaluate the minimum allowed voltage at a given time using linear interpolation.
    fn min_voltage_at(&self, time_s: f64) -> f64 {
        if self.profile_points.is_empty() {
            return 1.0;
        }
        // Before first point
        if time_s <= self.profile_points[0].0 {
            return self.profile_points[0].1;
        }
        // After last point
        let last = self.profile_points[self.profile_points.len() - 1];
        if time_s >= last.0 {
            return last.1;
        }
        // Linear interpolation between surrounding points
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

    /// Check whether a voltage sag event is LVRT-compliant.
    ///
    /// For each time sample in the event, the measured voltage is compared against
    /// the profile boundary. If the voltage ever falls below the minimum allowed
    /// by the profile, the inverter is permitted (required) to disconnect.
    pub fn passes_lvrt(&self, event: &LvrtEvent) -> LvrtResult {
        let mut violated_at: Option<f64> = None;
        let mut max_duration_ok = 0.0_f64;

        for &(t, v) in &event.time_profile {
            let v_min = self.min_voltage_at(t);
            if v >= v_min {
                max_duration_ok = max_duration_ok.max(t);
            } else if violated_at.is_none() {
                violated_at = Some(t);
            }
        }

        let compliant = violated_at.is_none();

        // Compute required reactive support profile
        let required_reactive: Vec<(f64, f64)> = if self.reactive_injection_required {
            event
                .time_profile
                .iter()
                .map(|&(t, v)| {
                    let q = compute_lvrt_reactive_injection(
                        v,
                        event.pre_fault_power_pu,
                        self.reactive_k_factor,
                        1.0,
                    );
                    (t, q)
                })
                .collect()
        } else {
            Vec::new()
        };

        LvrtResult {
            compliant,
            disconnection_required: !compliant,
            max_duration_compliant_s: max_duration_ok,
            reactive_support_required: self.reactive_injection_required,
            required_reactive_pu: required_reactive,
            violated_at_s: violated_at,
        }
    }
}

/// A voltage sag event to evaluate against an LVRT profile.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LvrtEvent {
    /// Time-series of (time_s, voltage_pu) during the event.
    pub time_profile: Vec<(f64, f64)>,
    /// Pre-fault active power output [pu of rated].
    pub pre_fault_power_pu: f64,
}

/// Result of evaluating an LVRT event against a profile.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LvrtResult {
    /// Whether the event is within the LVRT ride-through envelope (no disconnection needed).
    pub compliant: bool,
    /// Whether disconnection is permitted/required under the profile.
    pub disconnection_required: bool,
    /// Longest continuous duration (s) during which the event was within the envelope.
    pub max_duration_compliant_s: f64,
    /// Whether reactive current injection is required by the profile.
    pub reactive_support_required: bool,
    /// Required reactive injection at each time sample [(time_s, Q_pu)].
    pub required_reactive_pu: Vec<(f64, f64)>,
    /// Time (s) at which the first violation occurred, if any.
    pub violated_at_s: Option<f64>,
}

/// Compute the reactive current injection required during an LVRT event.
///
/// Uses the European k-factor approach:
/// ```text
/// ΔIq = k * (1 - V_pu) * I_rated_pu
/// ```
/// where `k` is typically 2.0 (inject 100% reactive at 50% voltage drop).
/// The result is clamped to `[0, i_rated_pu]`.
///
/// # Arguments
/// - `v_pu`         — current terminal voltage \[pu\]
/// - `v_pre_fault`  — pre-fault voltage \[pu\] (used as reference for ΔV)
/// - `k_factor`     — reactive gain (`ΔIq / ΔV`, typically 2.0)
/// - `i_rated_pu`   — rated current \[pu\] (ceiling for injection)
pub fn compute_lvrt_reactive_injection(
    v_pu: f64,
    v_pre_fault: f64,
    k_factor: f64,
    i_rated_pu: f64,
) -> f64 {
    let delta_v = (v_pre_fault - v_pu).max(0.0);
    (k_factor * delta_v * i_rated_pu).clamp(0.0, i_rated_pu)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lvrt_iec61400_passes() {
        // Zero voltage for 100 ms then recovery — within 150 ms limit
        let profile = LvrtProfile::iec61400();
        let event = LvrtEvent {
            time_profile: vec![
                (0.0, 0.0),
                (0.05, 0.0),
                (0.10, 0.0),
                (0.12, 0.5),
                (0.20, 0.85),
                (0.50, 0.9),
            ],
            pre_fault_power_pu: 1.0,
        };
        let result = profile.passes_lvrt(&event);
        assert!(
            result.compliant,
            "100 ms zero-voltage event should pass IEC 61400-21 LVRT"
        );
    }

    #[test]
    fn test_lvrt_too_long_fails() {
        // Zero voltage persists past 150 ms — violates IEC 61400-21
        let profile = LvrtProfile::iec61400();
        let event = LvrtEvent {
            time_profile: vec![
                (0.0, 0.0),
                (0.10, 0.0),
                (0.20, 0.0), // still 0 after 150 ms boundary → violation
                (0.30, 0.0),
                (0.50, 0.85),
            ],
            pre_fault_power_pu: 1.0,
        };
        let result = profile.passes_lvrt(&event);
        assert!(
            !result.compliant,
            "200 ms zero-voltage should fail IEC 61400-21"
        );
        assert!(result.violated_at_s.is_some());
        assert!(result.disconnection_required);
    }

    #[test]
    fn test_lvrt_reactive_injection_proportional() {
        // At 0.5 pu voltage with k=2, i_rated=1: ΔIq = 2*(1-0.5)*1 = 1.0 pu
        let q = compute_lvrt_reactive_injection(0.5, 1.0, 2.0, 1.0);
        assert!(
            (q - 1.0).abs() < 1e-9,
            "Reactive injection should be 1.0 pu: got {}",
            q
        );

        // At 0.9 pu voltage: ΔIq = 2*(1-0.9)*1 = 0.2 pu
        let q2 = compute_lvrt_reactive_injection(0.9, 1.0, 2.0, 1.0);
        assert!(
            (q2 - 0.2).abs() < 1e-9,
            "Reactive injection at 0.9 pu should be 0.2: got {}",
            q2
        );
    }

    #[test]
    fn test_lvrt_nerc_zero_voltage_within_625ms() {
        let profile = LvrtProfile::nerc();
        let event = LvrtEvent {
            time_profile: vec![
                (0.0, 0.0),
                (0.3, 0.0),
                (0.6, 0.0), // 0.6 s but NERC allows 0.625 s at zero
                (1.0, 0.5),
                (3.0, 0.9),
            ],
            pre_fault_power_pu: 1.0,
        };
        let result = profile.passes_lvrt(&event);
        assert!(result.compliant, "NERC allows zero voltage for 625 ms");
    }

    #[test]
    fn test_lvrt_reactive_required_in_iec() {
        let profile = LvrtProfile::iec61400();
        assert!(profile.reactive_injection_required);
    }

    #[test]
    fn test_lvrt_reactive_not_required_in_nerc() {
        let profile = LvrtProfile::nerc();
        assert!(!profile.reactive_injection_required);
    }
}
