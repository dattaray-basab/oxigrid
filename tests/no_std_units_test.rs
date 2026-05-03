//! Functional tests verifying units module arithmetic correctness.
//! Demonstrates that units do not depend on std-only features.

use oxigrid::units::conversion::{base_current, base_impedance};
use oxigrid::units::{Capacity, Energy, StateOfCharge};
use oxigrid::units::{Current, Frequency, Power, ReactivePower, Voltage};

#[test]
fn test_voltage_add() {
    let v1 = Voltage(1.0);
    let v2 = Voltage(2.0);
    let v3 = v1 + v2;
    assert!((v3.0 - 3.0).abs() < 1e-12);
}

#[test]
fn test_voltage_sub() {
    let v1 = Voltage(5.0);
    let v2 = Voltage(3.0);
    let v3 = v1 - v2;
    assert!((v3.0 - 2.0).abs() < 1e-12);
}

#[test]
fn test_voltage_mul_scalar() {
    let v = Voltage(2.0);
    let v2 = v * 3.0;
    assert!((v2.0 - 6.0).abs() < 1e-12);
}

#[test]
fn test_scalar_mul_voltage() {
    let v = Voltage(4.0);
    let v2 = 2.5 * v;
    assert!((v2.0 - 10.0).abs() < 1e-12);
}

#[test]
fn test_voltage_div_scalar() {
    let v = Voltage(8.0);
    let v2 = v / 2.0;
    assert!((v2.0 - 4.0).abs() < 1e-12);
}

#[test]
fn test_voltage_neg() {
    let v = Voltage(5.0);
    assert!(((-v).0 + 5.0).abs() < 1e-12);
}

#[test]
fn test_voltage_times_current_gives_power() {
    let v = Voltage(230.0);
    let i = Current(10.0);
    let p: Power = v * i;
    assert!((p.0 - 2300.0).abs() < 1e-12);
}

#[test]
fn test_current_times_voltage_gives_power() {
    let i = Current(5.0);
    let v = Voltage(100.0);
    let p: Power = i * v;
    assert!((p.0 - 500.0).abs() < 1e-12);
}

#[test]
fn test_soc_clamping_above() {
    let soc = StateOfCharge::new(1.5);
    assert!((soc.0 - 1.0).abs() < 1e-12);
}

#[test]
fn test_soc_clamping_below() {
    let soc = StateOfCharge::new(-0.1);
    assert!((soc.0 - 0.0).abs() < 1e-12);
}

#[test]
fn test_soc_midrange() {
    let soc = StateOfCharge::new(0.75);
    assert!((soc.0 - 0.75).abs() < 1e-12);
    assert!((soc.as_percentage() - 75.0).abs() < 1e-12);
}

#[test]
fn test_energy_arithmetic() {
    let e = Energy(1000.0);
    assert!((e.0 - 1000.0).abs() < 1e-12);
    assert!((e.to_kwh() - 1.0).abs() < 1e-12);
    let e2 = Energy::from_kwh(2.5);
    assert!((e2.0 - 2500.0).abs() < 1e-12);
}

#[test]
fn test_energy_add() {
    let e1 = Energy(500.0);
    let e2 = Energy(300.0);
    assert!(((e1 + e2).0 - 800.0).abs() < 1e-12);
}

#[test]
fn test_capacity_arithmetic() {
    let c = Capacity(100.0);
    let c2 = c * 0.5;
    assert!((c2.0 - 50.0).abs() < 1e-12);
}

#[test]
fn test_reactive_power() {
    let q1 = ReactivePower(100.0);
    let q2 = ReactivePower(50.0);
    assert!(((q1 + q2).0 - 150.0).abs() < 1e-12);
}

#[test]
fn test_frequency() {
    let f = Frequency(50.0);
    assert!((f.0 - 50.0).abs() < 1e-12);
}

#[test]
fn test_per_unit_conversion_impedance() {
    // Z_base = V_base² / S_base = 110² / 100 = 121 Ω
    let z_base = base_impedance(Voltage(110.0), Power(100.0));
    assert!(z_base > 0.0);
    assert!((z_base - 121.0).abs() < 0.01);
}

#[test]
fn test_per_unit_conversion_current() {
    // I_base = S_base / (sqrt(3) * V_base) = 100 / (sqrt(3) * 110) ≈ 0.5249 kA
    let i_base = base_current(Power(100.0), Voltage(110.0));
    assert!(i_base > 0.0);
    let expected = 100.0 / (3.0_f64.sqrt() * 110.0);
    assert!((i_base - expected).abs() < 1e-10);
}

#[test]
fn test_voltage_per_unit_roundtrip() {
    use oxigrid::units::electrical::PerUnit;
    let v = Voltage(115.0);
    let base = Voltage(230.0);
    let pu: PerUnit = v.to_per_unit(base);
    assert!((pu.0 - 0.5).abs() < 1e-12);
    let v_back = Voltage::from_per_unit(pu, base);
    assert!((v_back.0 - 115.0).abs() < 1e-12);
}

#[test]
fn test_power_per_unit_roundtrip() {
    let p = Power(500.0);
    let base = Power(1000.0);
    let pu = p.to_per_unit(base);
    assert!((pu.0 - 0.5).abs() < 1e-12);
    let p_back = Power::from_per_unit(pu, base);
    assert!((p_back.0 - 500.0).abs() < 1e-12);
}

#[test]
fn test_power_to_energy() {
    let p = Power(1000.0); // 1 kW
    let e = p.to_energy_wh(2.0); // 2 hours
    assert!((e.0 - 2000.0).abs() < 1e-12); // 2 kWh
}

#[test]
fn test_energy_to_power() {
    let e = Energy(3000.0); // 3 kWh
    let p = e.to_power_w(3.0); // over 3 hours
    assert!((p.0 - 1000.0).abs() < 1e-12); // 1 kW
}
