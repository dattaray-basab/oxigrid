use core::fmt;
use core::ops::{Add, Div, Mul, Neg, Sub};

/// Electrical energy `Wh`.  Inner value stores watt-hours.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
#[cfg_attr(feature = "std", derive(serde::Serialize, serde::Deserialize))]
pub struct Energy(pub f64);

impl Energy {
    /// Create from kilowatt-hours (1 kWh = 1000 Wh).
    pub fn from_kwh(kwh: f64) -> Self {
        Self(kwh * 1000.0)
    }

    /// Convert to kilowatt-hours.
    pub fn to_kwh(self) -> f64 {
        self.0 / 1000.0
    }

    /// Average power required to deliver this energy over `dt_hours` hours: Wh ÷ h = W.
    pub fn to_power_w(self, dt_hours: f64) -> crate::units::Power {
        crate::units::Power(self.0 / dt_hours)
    }
}

impl fmt::Display for Energy {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:.4} Wh", self.0)
    }
}

impl Add for Energy {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self(self.0 + rhs.0)
    }
}

impl Sub for Energy {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self(self.0 - rhs.0)
    }
}

impl Mul<f64> for Energy {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self(self.0 * rhs)
    }
}

impl Div<f64> for Energy {
    type Output = Self;
    fn div(self, rhs: f64) -> Self {
        Self(self.0 / rhs)
    }
}

impl Neg for Energy {
    type Output = Self;
    fn neg(self) -> Self {
        Self(-self.0)
    }
}

/// Battery charge capacity `Ah`.  Inner value stores ampere-hours.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
#[cfg_attr(feature = "std", derive(serde::Serialize, serde::Deserialize))]
pub struct Capacity(pub f64);

impl fmt::Display for Capacity {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:.4} Ah", self.0)
    }
}

impl Add for Capacity {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self(self.0 + rhs.0)
    }
}

impl Sub for Capacity {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self(self.0 - rhs.0)
    }
}

impl Mul<f64> for Capacity {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self(self.0 * rhs)
    }
}

impl Div<f64> for Capacity {
    type Output = Self;
    fn div(self, rhs: f64) -> Self {
        Self(self.0 / rhs)
    }
}

impl Neg for Capacity {
    type Output = Self;
    fn neg(self) -> Self {
        Self(-self.0)
    }
}

/// State of charge in [0, 1].  Inner value is a fraction (0 = empty, 1 = full).
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
#[cfg_attr(feature = "std", derive(serde::Serialize, serde::Deserialize))]
pub struct StateOfCharge(pub f64);

impl StateOfCharge {
    /// Construct SoC clamped to [0, 1].
    pub fn new(value: f64) -> Self {
        Self(value.clamp(0.0, 1.0))
    }

    /// SoC as a percentage in [0, 100].
    pub fn as_percentage(self) -> f64 {
        self.0 * 100.0
    }
}

impl fmt::Display for StateOfCharge {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:.1}%", self.as_percentage())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_energy_kwh() {
        let e = Energy::from_kwh(1.0);
        assert!((e.0 - 1000.0).abs() < 1e-10);
        assert!((e.to_kwh() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_soc_clamping() {
        assert_eq!(StateOfCharge::new(1.5).0, 1.0);
        assert_eq!(StateOfCharge::new(-0.1).0, 0.0);
        assert_eq!(StateOfCharge::new(0.5).0, 0.5);
    }

    #[test]
    fn test_wh_to_joules_ratio() {
        // Inner value is Wh directly — 1 Wh stored as 1.0
        assert!((Energy(1.0).0 - 1.0).abs() < 1e-10);
        // from_kwh(1.0) => 1000 Wh
        assert!((Energy::from_kwh(1.0).0 - 1000.0).abs() < 1e-10);
        // Conceptual: 1 Wh * 3600 s/h = 3600 J
        let joules = Energy(1.0).0 * 3600.0;
        assert!((joules - 3600.0).abs() < 1e-10);
        // Inner value of Energy(2.0) is 2.0
        assert!((Energy(2.0).0 - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_mwh_kwh_scaling() {
        // 1 MWh = 1000 kWh = 1_000_000 Wh
        assert!((Energy::from_kwh(1000.0).0 - 1_000_000.0).abs() < 1e-10);
        // Round-trip: from_kwh(1000).to_kwh() == 1000
        assert!((Energy::from_kwh(1000.0).to_kwh() - 1000.0).abs() < 1e-10);
    }

    #[test]
    fn test_energy_addition() {
        let sum = Energy(500.0) + Energy(700.0);
        assert!((sum.0 - 1200.0).abs() < 1e-10);
        let diff = Energy(700.0) - Energy(200.0);
        assert!((diff.0 - 500.0).abs() < 1e-10);
    }

    #[test]
    fn test_energy_comparison() {
        assert!(Energy(100.0) < Energy(200.0));
        assert!(Energy(500.0) > Energy(499.9));
        assert!(Energy(0.0) == Energy(0.0));
    }

    #[test]
    fn test_zero_energy() {
        assert!((Energy::default().0 - 0.0).abs() < 1e-10);
        let added = Energy(0.0) + Energy(100.0);
        assert!((added.0 - 100.0).abs() < 1e-10);
        let zeroed = Energy(50.0) - Energy(50.0);
        assert!(zeroed.0.abs() < 1e-10);
    }

    #[test]
    fn test_negative_energy_deficit() {
        assert!((Energy(-500.0).0 - (-500.0)).abs() < 1e-10);
        let neg = -Energy(300.0);
        assert!((neg.0 - (-300.0)).abs() < 1e-10);
        let deficit = Energy(100.0) - Energy(400.0);
        assert!((deficit.0 - (-300.0)).abs() < 1e-10);
    }
}
