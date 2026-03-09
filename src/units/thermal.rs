use serde::{Deserialize, Serialize};
use std::fmt;
use std::ops::{Add, Div, Mul, Neg, Sub};

/// Thermodynamic temperature [K].  Inner value stores kelvin.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize, Default)]
pub struct Temperature(pub f64);

impl Temperature {
    /// Construct from degrees Celsius (adds 273.15).
    pub fn from_celsius(c: f64) -> Self {
        Self(c + 273.15)
    }

    /// Convert to degrees Celsius (subtracts 273.15).
    pub fn to_celsius(self) -> f64 {
        self.0 - 273.15
    }

    /// Construct directly from kelvin.
    pub fn from_kelvin(k: f64) -> Self {
        Self(k)
    }
}

impl fmt::Display for Temperature {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:.2} K ({:.2} \u{00b0}C)", self.0, self.to_celsius())
    }
}

impl Add for Temperature {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self(self.0 + rhs.0)
    }
}

impl Sub for Temperature {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self(self.0 - rhs.0)
    }
}

/// Thermal conductivity λ [W/(m·K)].
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize, Default)]
pub struct ThermalConductivity(pub f64);

impl fmt::Display for ThermalConductivity {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:.4} W/(m\u{00b7}K)", self.0)
    }
}

impl Mul<f64> for ThermalConductivity {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self(self.0 * rhs)
    }
}

impl Div<f64> for ThermalConductivity {
    type Output = Self;
    fn div(self, rhs: f64) -> Self {
        Self(self.0 / rhs)
    }
}

impl Neg for ThermalConductivity {
    type Output = Self;
    fn neg(self) -> Self {
        Self(-self.0)
    }
}

/// Specific heat capacity Cp [J/(kg·K)].
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize, Default)]
pub struct HeatCapacity(pub f64);

impl fmt::Display for HeatCapacity {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:.4} J/(kg\u{00b7}K)", self.0)
    }
}

impl Mul<f64> for HeatCapacity {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self(self.0 * rhs)
    }
}

impl Div<f64> for HeatCapacity {
    type Output = Self;
    fn div(self, rhs: f64) -> Self {
        Self(self.0 / rhs)
    }
}

impl Neg for HeatCapacity {
    type Output = Self;
    fn neg(self) -> Self {
        Self(-self.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_temperature_conversion() {
        let t = Temperature::from_celsius(25.0);
        assert!((t.0 - 298.15).abs() < 1e-10);
        assert!((t.to_celsius() - 25.0).abs() < 1e-10);
    }

    #[test]
    fn test_temperature_display() {
        let t = Temperature::from_celsius(25.0);
        let s = format!("{t}");
        assert!(s.contains("298.15 K"));
        assert!(s.contains("25.00"));
    }
}
