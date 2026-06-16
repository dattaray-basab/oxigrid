use core::fmt;
use core::ops::{Add, Div, Mul, Neg, Sub};

/// Thermodynamic temperature `K`.  Inner value stores kelvin.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
#[cfg_attr(feature = "std", derive(serde::Serialize, serde::Deserialize))]
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
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
#[cfg_attr(feature = "std", derive(serde::Serialize, serde::Deserialize))]
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
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
#[cfg_attr(feature = "std", derive(serde::Serialize, serde::Deserialize))]
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

    #[test]
    fn test_kelvin_to_celsius_roundtrip() {
        let celsius = Temperature::from_kelvin(300.0).to_celsius();
        assert!((celsius - 26.85).abs() < 1e-10);
    }

    #[test]
    fn test_celsius_at_absolute_zero() {
        let t = Temperature::from_celsius(-273.15);
        assert!(t.0.abs() < 1e-10);
    }

    #[test]
    fn test_kelvin_always_positive_from_celsius() {
        for c in [0.0_f64, 100.0, -50.0] {
            let t = Temperature::from_celsius(c);
            assert!(
                t.0 > 0.0,
                "expected positive Kelvin for {}°C, got {}",
                c,
                t.0
            );
        }
    }

    #[test]
    fn test_thermal_conductivity_scaling() {
        let scaled = ThermalConductivity(50.0) * 2.0;
        assert!((scaled.0 - 100.0).abs() < 1e-10);
        let divided = ThermalConductivity(60.0) / 3.0;
        assert!((divided.0 - 20.0).abs() < 1e-10);
    }

    #[test]
    fn test_heat_capacity_scaling() {
        let scaled = HeatCapacity(1000.0) * 2.0;
        assert!((scaled.0 - 2000.0).abs() < 1e-10);
        let divided = HeatCapacity(900.0) / 3.0;
        assert!((divided.0 - 300.0).abs() < 1e-10);
    }

    #[test]
    fn test_temperature_ordering() {
        assert!(Temperature::from_celsius(0.0) < Temperature::from_celsius(100.0));
        assert!(Temperature::from_kelvin(300.0) > Temperature::from_kelvin(200.0));
    }
}
