/// Reactive power capability requirements for renewable generators.
///
/// Defines PQ-diagram boundaries and grid code reactive power requirements
/// that specify the reactive capability that must be maintained across the
/// full active power range.
///
/// # References
/// - IEC 61400-21: "Measurement and assessment of power quality characteristics"
/// - ENTSO-E, "Requirements for Generators" (RfG), Network Code 2016
use serde::{Deserialize, Serialize};

/// Reactive power capability (PQ diagram).
///
/// Represents the set of feasible (P, Q) operating points as piecewise-linear
/// Q_min(P) and Q_max(P) curves. All values are in per-unit of rated MVA.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PqDiagram {
    /// Active power sample points [pu of rated].
    pub p_points: Vec<f64>,
    /// Minimum reactive power at each P sample point (leading / absorbing) \[pu\].
    pub q_min: Vec<f64>,
    /// Maximum reactive power at each P sample point (lagging / injecting) \[pu\].
    pub q_max: Vec<f64>,
}

impl PqDiagram {
    /// IEC 61400-21 reactive capability curve for grid-connected wind turbines.
    ///
    /// ±0.33 pu reactive at all active power levels (0 to 1 pu).
    pub fn iec61400_wind() -> Self {
        Self {
            p_points: vec![0.0, 0.2, 0.5, 0.75, 1.0],
            q_min: vec![-0.33, -0.33, -0.33, -0.33, -0.33],
            q_max: vec![0.33, 0.33, 0.33, 0.33, 0.33],
        }
    }

    /// ENTSO-E generator reactive capability diagram.
    ///
    /// Typical large generator with reduced reactive range at low active output.
    pub fn entso_e_generator() -> Self {
        Self {
            p_points: vec![0.0, 0.2, 0.5, 0.8, 1.0],
            q_min: vec![-0.20, -0.25, -0.30, -0.33, -0.33],
            q_max: vec![0.20, 0.25, 0.30, 0.33, 0.40],
        }
    }

    /// Interpolate Q_min or Q_max at a given P value.
    fn interpolate_q(p_points: &[f64], q_vals: &[f64], p_pu: f64) -> f64 {
        let n = p_points.len();
        if n == 0 {
            return 0.0;
        }
        let p = p_pu.clamp(p_points[0], p_points[n - 1]);
        if n == 1 {
            return q_vals[0];
        }
        for i in 0..n - 1 {
            let p0 = p_points[i];
            let p1 = p_points[i + 1];
            if p >= p0 && p <= p1 {
                let alpha = if (p1 - p0).abs() < 1e-12 {
                    1.0
                } else {
                    (p - p0) / (p1 - p0)
                };
                return q_vals[i] + alpha * (q_vals[i + 1] - q_vals[i]);
            }
        }
        q_vals[n - 1]
    }

    /// Maximum reactive injection (lagging) at the given active power output.
    pub fn q_max_at_p(&self, p_pu: f64) -> f64 {
        Self::interpolate_q(&self.p_points, &self.q_max, p_pu)
    }

    /// Minimum reactive absorption (leading) at the given active power output.
    pub fn q_min_at_p(&self, p_pu: f64) -> f64 {
        Self::interpolate_q(&self.p_points, &self.q_min, p_pu)
    }

    /// Check whether the operating point (P, Q) is within the capability diagram.
    ///
    /// Returns `true` when `Q_min(P) ≤ Q ≤ Q_max(P)`.
    pub fn within_capability(&self, p_pu: f64, q_pu: f64) -> bool {
        q_pu >= self.q_min_at_p(p_pu) && q_pu <= self.q_max_at_p(p_pu)
    }
}

/// Grid code reactive power requirement at the point of common coupling (PCC).
///
/// Specifies the voltage range within which reactive requirements apply,
/// the minimum power factor, and whether reactive priority is required
/// during faults.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReactiveRequirement {
    /// Voltage range (V_min, V_max) in pu within which requirements apply.
    pub voltage_range: (f64, f64),
    /// Minimum power factor at PCC that must be maintained.
    pub power_factor_at_pcc: f64,
    /// Whether reactive current has priority over active current during fault.
    pub q_priority: bool,
}

impl ReactiveRequirement {
    /// ENTSO-E RfG reactive power requirement.
    ///
    /// 0.95 lagging – 0.95 leading power factor, voltage range 0.9–1.1 pu,
    /// reactive priority during faults.
    pub fn entso_e() -> Self {
        Self {
            voltage_range: (0.90, 1.10),
            power_factor_at_pcc: 0.95,
            q_priority: true,
        }
    }

    /// Check compliance of a given (P, Q, V) operating point.
    ///
    /// Returns `true` when the voltage is within the required range and the
    /// apparent power power factor meets the minimum requirement.
    pub fn check_compliance(&self, p_pu: f64, q_pu: f64, v_pu: f64) -> bool {
        // Check voltage range
        if v_pu < self.voltage_range.0 || v_pu > self.voltage_range.1 {
            return false;
        }
        // Compute power factor; |S| = sqrt(P² + Q²)
        let s = (p_pu * p_pu + q_pu * q_pu).sqrt();
        if s < 1e-9 {
            return true; // no generation → trivially compliant
        }
        let pf = (p_pu / s).abs();
        pf >= self.power_factor_at_pcc
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pq_diagram_within_capability() {
        let pq = PqDiagram::iec61400_wind();
        // (0.8, 0.2) should be within ±0.33 pu
        assert!(pq.within_capability(0.8, 0.2));
        assert!(pq.within_capability(0.5, -0.33));
        assert!(pq.within_capability(1.0, 0.33));
    }

    #[test]
    fn test_pq_diagram_outside_capability() {
        let pq = PqDiagram::iec61400_wind();
        // Q = 0.5 exceeds max 0.33 pu
        assert!(!pq.within_capability(0.8, 0.5));
        // Q = -0.5 is below min -0.33 pu
        assert!(!pq.within_capability(0.5, -0.5));
    }

    #[test]
    fn test_q_max_at_full_power() {
        let pq = PqDiagram::iec61400_wind();
        let q_max = pq.q_max_at_p(1.0);
        assert!((q_max - 0.33).abs() < 1e-9);
    }

    #[test]
    fn test_q_min_at_zero_power() {
        let pq = PqDiagram::iec61400_wind();
        let q_min = pq.q_min_at_p(0.0);
        assert!((q_min - (-0.33)).abs() < 1e-9);
    }

    #[test]
    fn test_entso_e_generator_q_max_varies_with_p() {
        let pq = PqDiagram::entso_e_generator();
        let q_max_half = pq.q_max_at_p(0.5);
        let q_max_full = pq.q_max_at_p(1.0);
        assert!(
            q_max_full > q_max_half,
            "Q_max should increase with P for ENTSO-E generator"
        );
    }

    #[test]
    fn test_reactive_requirement_entso_e_compliant() {
        let req = ReactiveRequirement::entso_e();
        // P=0.8, Q=0.1 → pf = 0.8/sqrt(0.65) ≈ 0.99 ≥ 0.95, V=1.0 in range
        assert!(req.check_compliance(0.8, 0.1, 1.0));
    }

    #[test]
    fn test_reactive_requirement_low_pf_fails() {
        let req = ReactiveRequirement::entso_e();
        // P=0.5, Q=0.5 → pf ≈ 0.707 < 0.95 → non-compliant
        assert!(!req.check_compliance(0.5, 0.5, 1.0));
    }

    #[test]
    fn test_reactive_requirement_voltage_out_of_range() {
        let req = ReactiveRequirement::entso_e();
        // Good PF but voltage out of range
        assert!(!req.check_compliance(0.9, 0.1, 0.85));
        assert!(!req.check_compliance(0.9, 0.1, 1.15));
    }
}
