use core::fmt;
use core::ops::{Add, Div, Mul, Neg, Sub};

macro_rules! impl_unit_type {
    ($name:ident, $unit:expr) => {
        #[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
        #[cfg_attr(feature = "std", derive(serde::Serialize, serde::Deserialize))]
        pub struct $name(pub f64);

        impl fmt::Display for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                write!(f, "{:.4} {}", self.0, $unit)
            }
        }

        impl Add for $name {
            type Output = Self;
            fn add(self, rhs: Self) -> Self {
                Self(self.0 + rhs.0)
            }
        }

        impl Sub for $name {
            type Output = Self;
            fn sub(self, rhs: Self) -> Self {
                Self(self.0 - rhs.0)
            }
        }

        impl Mul<f64> for $name {
            type Output = Self;
            fn mul(self, rhs: f64) -> Self {
                Self(self.0 * rhs)
            }
        }

        impl Mul<$name> for f64 {
            type Output = $name;
            fn mul(self, rhs: $name) -> $name {
                $name(self * rhs.0)
            }
        }

        impl Div<f64> for $name {
            type Output = Self;
            fn div(self, rhs: f64) -> Self {
                Self(self.0 / rhs)
            }
        }

        impl Neg for $name {
            type Output = Self;
            fn neg(self) -> Self {
                Self(-self.0)
            }
        }
    };
}

impl_unit_type!(Voltage, "V");
impl_unit_type!(Current, "A");
impl_unit_type!(Power, "W");
impl_unit_type!(ReactivePower, "VAr");
impl_unit_type!(Frequency, "Hz");
impl_unit_type!(PerUnit, "p.u.");

// Unit-type descriptions (rustdoc via inherent impl blocks below)

/// Terminal voltage `V`.
///
/// Positive value = voltage magnitude above ground reference.
/// Inner `f64` stores volts.
///
/// # Examples
///
/// ```rust
/// use oxigrid::units::electrical::Voltage;
///
/// // Construct a 230 V bus voltage
/// let v = Voltage(230.0);
/// assert_eq!(v.0, 230.0);
///
/// // Per-unit conversion: 115 V on 230 V base = 0.5 p.u.
/// let base = Voltage(230.0);
/// let pu = Voltage(115.0).to_per_unit(base);
/// assert!((pu.0 - 0.5).abs() < 1e-9);
///
/// // Recover from per-unit
/// let v_back = Voltage::from_per_unit(pu, base);
/// assert!((v_back.0 - 115.0).abs() < 1e-9);
/// ```
impl Voltage {}

/// Electric current `A`.
///
/// Convention: positive = discharge (conventional current out of positive terminal).
/// Inner `f64` stores amperes.
impl Current {}

/// Active (real) power `W`.
///
/// Inner `f64` stores watts.  Use `to_energy_wh(dt_h)` to integrate over time.
impl Power {}

/// Reactive power `VAr`.
///
/// Positive = inductive reactive power consumption.  Inner `f64` stores volt-amperes reactive.
impl ReactivePower {}

/// Electrical frequency `Hz`.  Inner `f64` stores hertz.
impl Frequency {}

/// Dimensionless per-unit value.
///
/// Quantity normalised by its base value.  Inner `f64` is already normalised.
impl PerUnit {}

/// Complex impedance Z = R + jX `Ω`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "std", derive(serde::Serialize, serde::Deserialize))]
pub struct Impedance {
    /// Resistance `Ω`.
    pub r: f64,
    /// Reactance `Ω`.  Positive = inductive, negative = capacitive.
    pub x: f64,
}

impl Impedance {
    /// Construct Z = R + jX.
    pub fn new(r: f64, x: f64) -> Self {
        Self { r, x }
    }

    /// Impedance magnitude |Z| = √(R² + X²) `Ω`.
    pub fn magnitude(&self) -> f64 {
        (self.r * self.r + self.x * self.x).sqrt()
    }

    /// Convert to complex form R + jX.
    pub fn to_complex(&self) -> num_complex::Complex64 {
        num_complex::Complex64::new(self.r, self.x)
    }

    /// Admittance Y = 1/Z.  Returns zero for near-zero impedance.
    pub fn to_admittance(&self) -> num_complex::Complex64 {
        let z = self.to_complex();
        if z.norm() < 1e-15 {
            num_complex::Complex64::new(0.0, 0.0)
        } else {
            1.0 / z
        }
    }
}

impl fmt::Display for Impedance {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:.4} + j{:.4} \u{03a9}", self.r, self.x)
    }
}

impl Add for Impedance {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self {
            r: self.r + rhs.r,
            x: self.x + rhs.x,
        }
    }
}

impl Sub for Impedance {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self {
            r: self.r - rhs.r,
            x: self.x - rhs.x,
        }
    }
}

impl Mul<f64> for Impedance {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self {
            r: self.r * rhs,
            x: self.x * rhs,
        }
    }
}

impl Voltage {
    /// Normalise this voltage to per-unit given a base voltage.
    pub fn to_per_unit(self, base: Voltage) -> PerUnit {
        PerUnit(self.0 / base.0)
    }

    /// Recover actual voltage from a per-unit value and base voltage.
    pub fn from_per_unit(pu: PerUnit, base: Voltage) -> Voltage {
        Voltage(pu.0 * base.0)
    }
}

impl Power {
    /// Normalise this power to per-unit given a base power.
    pub fn to_per_unit(self, base: Power) -> PerUnit {
        PerUnit(self.0 / base.0)
    }

    /// Recover actual power from a per-unit value and base power.
    pub fn from_per_unit(pu: PerUnit, base: Power) -> Power {
        Power(pu.0 * base.0)
    }
}

impl ReactivePower {
    /// Normalise this reactive power to per-unit given a base power (not base reactive power).
    pub fn to_per_unit(self, base: Power) -> PerUnit {
        PerUnit(self.0 / base.0)
    }

    /// Recover reactive power from a per-unit value and base power.
    pub fn from_per_unit(pu: PerUnit, base: Power) -> ReactivePower {
        ReactivePower(pu.0 * base.0)
    }
}

impl Current {
    /// Normalise this current to per-unit given a base current.
    pub fn to_per_unit(self, base: Current) -> PerUnit {
        PerUnit(self.0 / base.0)
    }

    /// Recover actual current from a per-unit value and base current.
    pub fn from_per_unit(pu: PerUnit, base: Current) -> Current {
        Current(pu.0 * base.0)
    }
}

// Cross-type dimensional arithmetic: V × A = W
impl core::ops::Mul<Current> for Voltage {
    type Output = Power;
    fn mul(self, rhs: Current) -> Power {
        Power(self.0 * rhs.0)
    }
}

impl core::ops::Mul<Voltage> for Current {
    type Output = Power;
    fn mul(self, rhs: Voltage) -> Power {
        Power(self.0 * rhs.0)
    }
}

impl Power {
    /// Convert power to energy given a duration in hours: W × h = Wh
    pub fn to_energy_wh(self, dt_hours: f64) -> crate::units::Energy {
        crate::units::Energy(self.0 * dt_hours)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(test)]
    mod proptest_tests {
        use super::*;
        use proptest::prelude::*;

        proptest! {
            #[test]
            fn prop_voltage_per_unit_roundtrip(v in 0.1f64..1000.0, base in 0.1f64..1000.0) {
                let voltage = Voltage(v);
                let base_v = Voltage(base);
                let pu = voltage.to_per_unit(base_v);
                let back = Voltage::from_per_unit(pu, base_v);
                prop_assert!((back.0 - v).abs() < 1e-9);
            }

            #[test]
            fn prop_power_per_unit_roundtrip(p in 0.1f64..1000.0, base in 0.1f64..1000.0) {
                let power = Power(p);
                let base_p = Power(base);
                let pu = power.to_per_unit(base_p);
                let back = Power::from_per_unit(pu, base_p);
                prop_assert!((back.0 - p).abs() < 1e-9);
            }

            #[test]
            fn prop_voltage_add_commutative(a in -1000.0f64..1000.0, b in -1000.0f64..1000.0) {
                let va = Voltage(a);
                let vb = Voltage(b);
                prop_assert_eq!((va + vb).0, (vb + va).0);
            }

            #[test]
            fn prop_voltage_scale_identity(v in -1000.0f64..1000.0) {
                let voltage = Voltage(v);
                prop_assert_eq!((voltage * 1.0).0, v);
            }
        }
    }

    #[test]
    fn test_voltage_arithmetic() {
        let v1 = Voltage(100.0);
        let v2 = Voltage(50.0);
        assert_eq!((v1 + v2).0, 150.0);
        assert_eq!((v1 - v2).0, 50.0);
        assert_eq!((v1 * 2.0).0, 200.0);
        assert_eq!((v1 / 2.0).0, 50.0);
        assert_eq!((-v1).0, -100.0);
    }

    #[test]
    fn test_per_unit_conversion() {
        let v = Voltage(115.0);
        let base = Voltage(230.0);
        let pu = v.to_per_unit(base);
        assert!((pu.0 - 0.5).abs() < 1e-10);
        let v_back = Voltage::from_per_unit(pu, base);
        assert!((v_back.0 - 115.0).abs() < 1e-10);
    }

    #[test]
    fn test_impedance() {
        let z = Impedance::new(3.0, 4.0);
        assert!((z.magnitude() - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_display() {
        let v = Voltage(230.0);
        assert_eq!(format!("{v}"), "230.0000 V");
    }
}
