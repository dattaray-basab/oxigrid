use serde::{Deserialize, Serialize};
use std::fmt;
use std::ops::{Add, Div, Mul, Neg, Sub};

/// Electrical energy [Wh].  Inner value stores watt-hours.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize, Default)]
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

/// Battery charge capacity [Ah].  Inner value stores ampere-hours.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize, Default)]
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
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize, Default)]
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
}
