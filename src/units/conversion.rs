use super::electrical::{PerUnit, Power, Voltage};

pub trait ToPerUnit {
    fn to_per_unit(&self, base: f64) -> PerUnit;
}

pub trait FromPerUnit {
    fn from_per_unit(pu: PerUnit, base: f64) -> Self;
}

pub fn base_impedance(base_kv: Voltage, base_mva: Power) -> f64 {
    (base_kv.0 * base_kv.0) / base_mva.0
}

pub fn base_current(base_mva: Power, base_kv: Voltage) -> f64 {
    base_mva.0 / (base_kv.0 * 3.0_f64.sqrt())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_base_impedance() {
        let z_base = base_impedance(Voltage(230.0), Power(100.0));
        assert!((z_base - 529.0).abs() < 1e-10);
    }
}
