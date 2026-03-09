/// Frequency response requirements for renewable generators.
///
/// Covers droop response, fast frequency response (FFR), synthetic inertia,
/// and operating frequency ranges per ENTSO-E RfG and NERC BAL standards.
///
/// # References
/// - ENTSO-E, "Requirements for Generators" (RfG), Network Code 2016
/// - NERC, "BAL-003-2 Frequency Response and Frequency Bias Setting", 2020
/// - National Grid ESO, "Grid Code: Fast Frequency Response", 2021
use serde::{Deserialize, Serialize};

/// Frequency response requirement for a renewable generator.
///
/// Specifies the droop characteristic, dead-band, operating frequency range,
/// and whether fast frequency response or synthetic inertia are required.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FrequencyResponseRequirement {
    /// Profile name (e.g. "ENTSO-E RfG" or "NERC BAL-003").
    pub name: String,
    /// Frequency dead-band half-width \[Hz\]. No response within ±dead_band_hz
    /// of nominal frequency.
    pub dead_band_hz: f64,
    /// Governor droop [%]. A 4% droop means 4% frequency deviation → 100% power change.
    pub droop_pct: f64,
    /// Minimum operating frequency \[Hz\]. Must stay connected above this.
    pub f_min_hz: f64,
    /// Maximum operating frequency \[Hz\]. Must stay connected below this.
    pub f_max_hz: f64,
    /// Whether limited sensitive to island detection is required.
    pub lsip_required: bool,
    /// Whether sub-second (fast) frequency response is required.
    pub fast_frequency_response: bool,
    /// Whether synthetic inertia is required.
    pub inertia_response_required: bool,
}

impl FrequencyResponseRequirement {
    /// ENTSO-E Requirements for Generators (RfG) — European standard.
    ///
    /// 4% droop, ±200 mHz dead-band, 47–52 Hz operating range (Type B–D).
    pub fn entso_e_rfg() -> Self {
        Self {
            name: "ENTSO-E RfG".to_string(),
            dead_band_hz: 0.0,
            droop_pct: 4.0,
            f_min_hz: 47.0,
            f_max_hz: 52.0,
            lsip_required: false,
            fast_frequency_response: false,
            inertia_response_required: false,
        }
    }

    /// NERC BAL-003-2 frequency response requirement (North American standard).
    ///
    /// 5% droop, no mandatory dead-band, 59–61 Hz operating range.
    pub fn nerc_bal() -> Self {
        Self {
            name: "NERC BAL-003-2".to_string(),
            dead_band_hz: 0.036, // ±0.036 Hz common for North American interconnections
            droop_pct: 5.0,
            f_min_hz: 59.0,
            f_max_hz: 61.0,
            lsip_required: false,
            fast_frequency_response: false,
            inertia_response_required: false,
        }
    }

    /// Compute the per-unit power response required at the given frequency.
    ///
    /// Formula (droop characteristic):
    /// ```text
    /// if |Δf| <= dead_band:
    ///     response = 0
    /// else:
    ///     response = -(1/droop_frac) * Δf/f0
    /// ```
    /// Clamped to `[-1.0, 1.0]`.
    ///
    /// # Arguments
    /// - `freq_hz`    — measured frequency \[Hz\]
    /// - `nominal_hz` — nominal system frequency \[Hz\] (50 or 60)
    pub fn required_response_pu(&self, freq_hz: f64, nominal_hz: f64) -> f64 {
        let delta_f = freq_hz - nominal_hz;
        if delta_f.abs() <= self.dead_band_hz {
            return 0.0;
        }
        let droop_frac = self.droop_pct / 100.0;
        if droop_frac < 1e-9 {
            return 0.0;
        }
        let response = -(1.0 / droop_frac) * (delta_f / nominal_hz);
        response.clamp(-1.0, 1.0)
    }

    /// Verify that a synthetic inertia response is adequate.
    ///
    /// An adequate inertia response must deliver `p_response` ≥ 5% of rated
    /// power within 500 ms of a ROCOF event exceeding 0.5 Hz/s.
    ///
    /// # Arguments
    /// - `rocof`      — Rate of Change of Frequency [Hz/s]
    /// - `p_response` — per-unit power delivered within 500 ms
    pub fn check_inertia_response(&self, rocof: f64, p_response: f64) -> bool {
        if !self.inertia_response_required {
            return true; // no requirement → always compliant
        }
        // Trigger threshold: ROCOF must exceed 0.5 Hz/s to activate
        if rocof.abs() < 0.5 {
            return true;
        }
        // Minimum response: 5% of rated within 500 ms
        p_response >= 0.05
    }

    /// Check whether the generator is within its permissible frequency range.
    pub fn within_operating_range(&self, freq_hz: f64) -> bool {
        freq_hz >= self.f_min_hz && freq_hz <= self.f_max_hz
    }
}

/// Fast Frequency Response (FFR) requirement.
///
/// Specifies timing and magnitude requirements for sub-second frequency support.
/// FFR is particularly important in low-inertia systems with high renewable
/// penetration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FfrRequirement {
    /// Maximum time to reach full power response \[ms\].
    pub response_time_ms: f64,
    /// Minimum duration to sustain full response \[s\].
    pub sustained_duration_s: f64,
    /// Frequency deviation that activates FFR [Hz below nominal].
    pub activation_threshold_hz: f64,
    /// Required power response magnitude [pu of rated].
    pub required_response_pu: f64,
}

impl FfrRequirement {
    /// Typical FFR requirement (e.g. GB Grid Code, National Grid ESO).
    ///
    /// Full response within 500 ms, sustained for 30 s.
    pub fn standard() -> Self {
        Self {
            response_time_ms: 500.0,
            sustained_duration_s: 30.0,
            activation_threshold_hz: 0.5,
            required_response_pu: 0.05,
        }
    }

    /// Strict FFR (e.g. Ireland/GB enhanced FFR tender specification).
    ///
    /// Full response within 150 ms, sustained for 10 s.
    pub fn enhanced() -> Self {
        Self {
            response_time_ms: 150.0,
            sustained_duration_s: 10.0,
            activation_threshold_hz: 0.2,
            required_response_pu: 0.10,
        }
    }

    /// Check whether an observed response meets the FFR requirement.
    ///
    /// # Arguments
    /// - `actual_response_time_ms` — time to full response \[ms\]
    /// - `actual_duration_s`       — duration response is sustained \[s\]
    /// - `actual_response_pu`      — power response magnitude \[pu\]
    pub fn is_compliant(
        &self,
        actual_response_time_ms: f64,
        actual_duration_s: f64,
        actual_response_pu: f64,
    ) -> bool {
        actual_response_time_ms <= self.response_time_ms
            && actual_duration_s >= self.sustained_duration_s
            && actual_response_pu >= self.required_response_pu
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_frequency_response_within_deadband() {
        let req = FrequencyResponseRequirement::nerc_bal();
        // 0.02 Hz deviation is within ±0.036 Hz dead-band
        let resp = req.required_response_pu(60.02, 60.0);
        assert_eq!(resp, 0.0, "Response within dead-band should be zero");
    }

    #[test]
    fn test_frequency_response_droop() {
        let req = FrequencyResponseRequirement::entso_e_rfg();
        // No dead-band, 4% droop, 50 Hz nominal, 49 Hz → -1 Hz → response
        // response = -(1/0.04) * (-1/50) = 0.5 pu
        let resp = req.required_response_pu(49.0, 50.0);
        assert!(
            (resp - 0.5).abs() < 1e-9,
            "Expected 0.5 pu response at 49 Hz with 4% droop, got {:.6}",
            resp
        );
    }

    #[test]
    fn test_frequency_response_clamped() {
        let req = FrequencyResponseRequirement::entso_e_rfg();
        // Very large deviation should clamp to 1.0 pu
        let resp = req.required_response_pu(43.0, 50.0);
        assert!(
            (resp - 1.0).abs() < 1e-9,
            "Response should be clamped to 1.0"
        );
    }

    #[test]
    fn test_frequency_response_over_frequency() {
        let req = FrequencyResponseRequirement::entso_e_rfg();
        // Over-frequency → negative response (curtail generation)
        let resp = req.required_response_pu(51.0, 50.0);
        assert!(
            resp < 0.0,
            "Over-frequency should produce negative (curtailment) response"
        );
    }

    #[test]
    fn test_frequency_within_operating_range() {
        let req = FrequencyResponseRequirement::entso_e_rfg();
        assert!(req.within_operating_range(50.0));
        assert!(req.within_operating_range(49.0));
        assert!(!req.within_operating_range(46.9));
        assert!(!req.within_operating_range(52.1));
    }

    #[test]
    fn test_inertia_response_not_required_always_passes() {
        let mut req = FrequencyResponseRequirement::entso_e_rfg();
        req.inertia_response_required = false;
        assert!(req.check_inertia_response(5.0, 0.0));
    }

    #[test]
    fn test_ffr_standard_compliance() {
        let ffr = FfrRequirement::standard();
        assert!(ffr.is_compliant(400.0, 35.0, 0.08));
        assert!(!ffr.is_compliant(600.0, 35.0, 0.08)); // too slow
        assert!(!ffr.is_compliant(400.0, 20.0, 0.08)); // not sustained long enough
    }
}
