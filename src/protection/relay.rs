/// Protective relay characteristics.
///
/// Implements IEC 60255 / IEEE C37.112 inverse-time overcurrent relay curves
/// and distance relay zones.
use serde::{Deserialize, Serialize};

/// IEC 60255 overcurrent relay curve types.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub enum RelayCharacteristic {
    /// Standard Inverse (SI): t = 0.14 / ((I/Ip)^0.02 − 1) × TMS
    StandardInverse,
    /// Very Inverse (VI): t = 13.5 / ((I/Ip) − 1) × TMS
    VeryInverse,
    /// Extremely Inverse (EI): t = 80 / ((I/Ip)^2 − 1) × TMS
    ExtremelyInverse,
    /// Long-Time Inverse (LTI): t = 120 / ((I/Ip) − 1) × TMS
    LongTimeInverse,
    /// IEEE Moderately Inverse: t = 0.0515/((I/Ip)^0.02 − 1) + 0.1140
    IeeeModeInverse,
    /// IEEE Very Inverse: t = 19.61/((I/Ip)^2 − 1) + 0.4910
    IeeeVeryInverse,
    /// IEEE Extremely Inverse: t = 28.2/((I/Ip)^2 − 1) + 0.1217
    IeeeExtrInverse,
}

impl RelayCharacteristic {
    /// IEC 60255 coefficients (A, B, p).
    fn iec_coeffs(self) -> Option<(f64, f64, f64)> {
        match self {
            Self::StandardInverse => Some((0.14, 0.0, 0.02)),
            Self::VeryInverse => Some((13.5, 0.0, 1.0)),
            Self::ExtremelyInverse => Some((80.0, 0.0, 2.0)),
            Self::LongTimeInverse => Some((120.0, 0.0, 1.0)),
            _ => None,
        }
    }

    /// IEEE coefficients (A, B, p) for the formula t = (A/((I/Ip)^p − 1) + B) * TMS.
    fn ieee_coeffs(self) -> Option<(f64, f64, f64)> {
        match self {
            Self::IeeeModeInverse => Some((0.0515, 0.1140, 0.02)),
            Self::IeeeVeryInverse => Some((19.61, 0.4910, 2.0)),
            Self::IeeeExtrInverse => Some((28.2, 0.1217, 2.0)),
            _ => None,
        }
    }
}

/// Inverse-time overcurrent relay.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OcRelay {
    /// Pickup current setting [A or p.u.]
    pub i_pickup: f64,
    /// Time multiplier setting (TMS / TDS)
    pub tms: f64,
    /// Relay characteristic curve
    pub curve: RelayCharacteristic,
    /// Instantaneous trip level `A` (0 = disabled)
    pub inst_level: f64,
    /// Instantaneous trip time `s` (typically 0.02–0.05)
    pub inst_time: f64,
}

impl OcRelay {
    pub fn new(i_pickup: f64, tms: f64, curve: RelayCharacteristic) -> Self {
        Self {
            i_pickup,
            tms,
            curve,
            inst_level: 0.0,
            inst_time: 0.0,
        }
    }

    pub fn with_instantaneous(mut self, level: f64, time: f64) -> Self {
        self.inst_level = level;
        self.inst_time = time;
        self
    }

    /// Compute trip time `s` for fault current `i_fault`.
    ///
    /// Returns `None` if current is below pickup.
    pub fn trip_time(&self, i_fault: f64) -> Option<f64> {
        if i_fault <= self.i_pickup {
            return None;
        }
        // Instantaneous trip check
        if self.inst_level > 0.0 && i_fault >= self.inst_level {
            return Some(self.inst_time);
        }

        let m = i_fault / self.i_pickup;

        if let Some((a, _b, p)) = self.curve.iec_coeffs() {
            let denom = m.powf(p) - 1.0;
            if denom < 1e-12 {
                return Some(f64::INFINITY);
            }
            Some(self.tms * a / denom)
        } else if let Some((a, b, p)) = self.curve.ieee_coeffs() {
            let denom = m.powf(p) - 1.0;
            if denom < 1e-12 {
                return Some(f64::INFINITY);
            }
            Some(self.tms * (a / denom + b))
        } else {
            None
        }
    }

    /// Check coordination: this relay (backup) must trip > `margin_s` after `primary`.
    ///
    /// Returns `Ok(margin)` if coordinated, `Err(margin)` if not.
    pub fn check_coordination(
        &self,
        primary: &OcRelay,
        i_fault: f64,
        margin_s: f64,
    ) -> std::result::Result<f64, f64> {
        let t_primary = primary.trip_time(i_fault).unwrap_or(f64::INFINITY);
        let t_backup = self.trip_time(i_fault).unwrap_or(f64::INFINITY);
        let actual_margin = t_backup - t_primary;
        if actual_margin >= margin_s {
            Ok(actual_margin)
        } else {
            Err(actual_margin)
        }
    }
}

/// Mho distance relay (single zone).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DistanceRelay {
    /// Zone 1 reach `Ω` (typically 80% of line impedance)
    pub z1_reach: f64,
    /// Zone 2 reach `Ω` (typically 120% of line impedance)
    pub z2_reach: f64,
    /// Zone 3 reach `Ω` (typically 220% of line impedance)
    pub z3_reach: f64,
    /// Zone 1 trip time `s` (instantaneous ≈ 0.02)
    pub t1: f64,
    /// Zone 2 trip time `s` (typically 0.3–0.5)
    pub t2: f64,
    /// Zone 3 trip time `s` (typically 1.0)
    pub t3: f64,
}

impl DistanceRelay {
    /// Trip time for apparent impedance `z_app` `Ω`.
    pub fn trip_time(&self, z_app: f64) -> Option<f64> {
        if z_app <= self.z1_reach {
            Some(self.t1)
        } else if z_app <= self.z2_reach {
            Some(self.t2)
        } else if z_app <= self.z3_reach {
            Some(self.t3)
        } else {
            None // Outside all zones
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_no_trip_below_pickup() {
        let relay = OcRelay::new(100.0, 0.5, RelayCharacteristic::StandardInverse);
        assert!(relay.trip_time(50.0).is_none());
        assert!(relay.trip_time(100.0).is_none()); // exactly at pickup
    }

    #[test]
    fn test_si_trip_time_finite() {
        let relay = OcRelay::new(100.0, 0.5, RelayCharacteristic::StandardInverse);
        let t = relay.trip_time(300.0).expect("should trip");
        assert!(t > 0.0 && t.is_finite(), "t={:.4}", t);
    }

    #[test]
    fn test_higher_current_faster_trip() {
        let relay = OcRelay::new(100.0, 0.5, RelayCharacteristic::VeryInverse);
        let t1 = relay.trip_time(200.0).unwrap();
        let t2 = relay.trip_time(500.0).unwrap();
        assert!(t2 < t1, "t1={:.4} t2={:.4}", t1, t2);
    }

    #[test]
    fn test_instantaneous_trip_overrides() {
        let relay = OcRelay::new(100.0, 0.5, RelayCharacteristic::StandardInverse)
            .with_instantaneous(800.0, 0.03);
        let t = relay.trip_time(1000.0).unwrap();
        assert!((t - 0.03).abs() < 1e-9, "should be instantaneous: {}", t);
    }

    #[test]
    fn test_ieee_very_inverse_curve() {
        let relay = OcRelay::new(100.0, 1.0, RelayCharacteristic::IeeeVeryInverse);
        let t = relay.trip_time(500.0).expect("should trip at 5× pickup");
        assert!(t > 0.0 && t < 10.0, "t={:.4}", t);
    }

    #[test]
    fn test_coordination_check_pass() {
        let primary = OcRelay::new(100.0, 0.2, RelayCharacteristic::VeryInverse);
        let backup = OcRelay::new(80.0, 0.5, RelayCharacteristic::VeryInverse);
        let result = backup.check_coordination(&primary, 500.0, 0.3);
        assert!(result.is_ok(), "margin={:.3}", result.unwrap_or_else(|e| e));
    }

    #[test]
    fn test_distance_relay_zones() {
        let relay = DistanceRelay {
            z1_reach: 8.0,
            z2_reach: 12.0,
            z3_reach: 20.0,
            t1: 0.02,
            t2: 0.40,
            t3: 1.0,
        };
        assert!((relay.trip_time(5.0).unwrap() - 0.02).abs() < 1e-9);
        assert!((relay.trip_time(10.0).unwrap() - 0.40).abs() < 1e-9);
        assert!((relay.trip_time(18.0).unwrap() - 1.0).abs() < 1e-9);
        assert!(relay.trip_time(25.0).is_none());
    }
}
