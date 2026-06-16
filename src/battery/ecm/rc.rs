/// 1RC and 2RC Thevenin battery models.
///
/// # 1RC Model
///
/// Circuit: OCV(SoC) — R0 — [R1 ‖ C1] — V_terminal
///
/// State equations:
///   V_1(k+1) = V_1(k)·exp(−Δt/(R1·C1)) + I·R1·(1 − exp(−Δt/(R1·C1)))
///   V_t = OCV(SoC) − I·R0 − V_1
///   SoC(k+1) = SoC(k) − I·Δt / (3600·Q_n)
///
/// # 2RC Model
///
/// Adds a second RC pair (R2, C2):
///   V_t = OCV(SoC) − I·R0 − V_1 − V_2
use crate::battery::{BatteryModel, BatteryState, OcvSocCurve};
use crate::units::{Current, Energy, StateOfCharge, Temperature, Voltage};
use serde::{Deserialize, Serialize};

// ── OneRcModel ──────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OneRcModel {
    pub ocv_curve: OcvSocCurve,
    pub r0: f64,          // [Ω] series resistance
    pub r1: f64,          // [Ω] RC pair resistance
    pub c1: f64,          // [F] RC pair capacitance
    pub capacity_ah: f64, // [Ah] nominal capacity
    pub coulombic_efficiency: f64,

    // State
    pub soc: f64,
    pub v_rc1: f64,       // voltage across RC1 [V]
    pub temperature: f64, // [K]
}

impl OneRcModel {
    pub fn new(ocv_curve: OcvSocCurve, r0: f64, r1: f64, c1: f64, capacity_ah: f64) -> Self {
        Self {
            ocv_curve,
            r0,
            r1,
            c1,
            capacity_ah,
            coulombic_efficiency: 0.98,
            soc: 1.0,
            v_rc1: 0.0,
            temperature: 298.15,
        }
    }

    pub fn with_soc(mut self, soc: f64) -> Self {
        self.soc = soc.clamp(0.0, 1.0);
        self
    }

    pub fn time_constant_1(&self) -> f64 {
        self.r1 * self.c1
    }
}

impl BatteryModel for OneRcModel {
    fn terminal_voltage(
        &self,
        soc: StateOfCharge,
        current: Current,
        _temp: Temperature,
    ) -> Voltage {
        let ocv = self.ocv_curve.ocv(soc.0);
        Voltage(ocv - current.0 * self.r0 - self.v_rc1)
    }

    fn step(&mut self, current: Current, dt: f64, temp: Temperature) -> BatteryState {
        let tau1 = self.time_constant_1();
        let exp1 = (-dt / tau1).exp();

        // RC1 voltage update (exact discrete solution)
        self.v_rc1 = self.v_rc1 * exp1 + current.0 * self.r1 * (1.0 - exp1);

        // SoC update
        let eta = if current.0 >= 0.0 {
            1.0
        } else {
            self.coulombic_efficiency
        };
        self.soc = (self.soc - current.0 * dt / (3600.0 * self.capacity_ah) * eta).clamp(0.0, 1.0);
        self.temperature = temp.0;

        let ocv = self.ocv_curve.ocv(self.soc);
        let v_t = ocv - current.0 * self.r0 - self.v_rc1;

        BatteryState {
            voltage: Voltage(v_t),
            soc: StateOfCharge::new(self.soc),
            temperature: temp,
            internal_resistance: self.r0 + self.r1,
            capacity_remaining: Energy(self.soc * self.capacity_ah * ocv),
            current,
        }
    }

    fn state(&self) -> BatteryState {
        let ocv = self.ocv_curve.ocv(self.soc);
        BatteryState {
            voltage: Voltage(ocv - self.v_rc1),
            soc: StateOfCharge::new(self.soc),
            temperature: Temperature(self.temperature),
            internal_resistance: self.r0 + self.r1,
            capacity_remaining: Energy(self.soc * self.capacity_ah * ocv),
            current: Current(0.0),
        }
    }
}

// ── TwoRcModel ──────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TwoRcModel {
    pub ocv_curve: OcvSocCurve,
    pub r0: f64,
    pub r1: f64,
    pub c1: f64,
    pub r2: f64,
    pub c2: f64,
    pub capacity_ah: f64,
    pub coulombic_efficiency: f64,

    // State
    pub soc: f64,
    pub v_rc1: f64,
    pub v_rc2: f64,
    pub temperature: f64,
}

impl TwoRcModel {
    pub fn new(
        ocv_curve: OcvSocCurve,
        r0: f64,
        r1: f64,
        c1: f64,
        r2: f64,
        c2: f64,
        capacity_ah: f64,
    ) -> Self {
        Self {
            ocv_curve,
            r0,
            r1,
            c1,
            r2,
            c2,
            capacity_ah,
            coulombic_efficiency: 0.98,
            soc: 1.0,
            v_rc1: 0.0,
            v_rc2: 0.0,
            temperature: 298.15,
        }
    }

    pub fn with_soc(mut self, soc: f64) -> Self {
        self.soc = soc.clamp(0.0, 1.0);
        self
    }
}

impl BatteryModel for TwoRcModel {
    fn terminal_voltage(
        &self,
        soc: StateOfCharge,
        current: Current,
        _temp: Temperature,
    ) -> Voltage {
        let ocv = self.ocv_curve.ocv(soc.0);
        Voltage(ocv - current.0 * self.r0 - self.v_rc1 - self.v_rc2)
    }

    fn step(&mut self, current: Current, dt: f64, temp: Temperature) -> BatteryState {
        let tau1 = self.r1 * self.c1;
        let tau2 = self.r2 * self.c2;
        let exp1 = (-dt / tau1).exp();
        let exp2 = (-dt / tau2).exp();

        self.v_rc1 = self.v_rc1 * exp1 + current.0 * self.r1 * (1.0 - exp1);
        self.v_rc2 = self.v_rc2 * exp2 + current.0 * self.r2 * (1.0 - exp2);

        let eta = if current.0 >= 0.0 {
            1.0
        } else {
            self.coulombic_efficiency
        };
        self.soc = (self.soc - current.0 * dt / (3600.0 * self.capacity_ah) * eta).clamp(0.0, 1.0);
        self.temperature = temp.0;

        let ocv = self.ocv_curve.ocv(self.soc);
        let v_t = ocv - current.0 * self.r0 - self.v_rc1 - self.v_rc2;

        BatteryState {
            voltage: Voltage(v_t),
            soc: StateOfCharge::new(self.soc),
            temperature: temp,
            internal_resistance: self.r0 + self.r1 + self.r2,
            capacity_remaining: Energy(self.soc * self.capacity_ah * ocv),
            current,
        }
    }

    fn state(&self) -> BatteryState {
        let ocv = self.ocv_curve.ocv(self.soc);
        BatteryState {
            voltage: Voltage(ocv - self.v_rc1 - self.v_rc2),
            soc: StateOfCharge::new(self.soc),
            temperature: Temperature(self.temperature),
            internal_resistance: self.r0 + self.r1 + self.r2,
            capacity_remaining: Energy(self.soc * self.capacity_ah * ocv),
            current: Current(0.0),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_1rc_voltage_relaxation() {
        // After discharge pulse, RC voltage should decay
        let mut model = OneRcModel::new(OcvSocCurve::nmc_default(), 0.02, 0.05, 3000.0, 75.0);
        // Apply a discharge pulse
        model.step(Current(75.0), 10.0, Temperature(298.15));
        let v_rc1_after = model.v_rc1;
        // Rest: current = 0
        model.step(Current(0.0), 100.0, Temperature(298.15));
        // RC voltage should have decayed
        assert!(model.v_rc1.abs() < v_rc1_after.abs());
    }

    #[test]
    fn test_2rc_discharge_curve() {
        let mut model = TwoRcModel::new(
            OcvSocCurve::nmc_default(),
            0.02,
            0.05,
            3000.0,
            0.03,
            500.0,
            75.0,
        );
        let initial_soc = model.soc;
        // 0.1C discharge for 1h: SoC decreases by ~0.1, voltage well above cutoff
        let state = model.step(Current(7.5), 3600.0, Temperature(298.15));
        assert!(state.soc.0 < initial_soc);
        assert!(state.voltage.0 > 2.5);
    }

    #[test]
    fn test_2rc_energy_balance() {
        let capacity = 10.0; // Ah
        let mut model = TwoRcModel::new(
            OcvSocCurve::nmc_default(),
            0.01,
            0.02,
            1000.0,
            0.01,
            200.0,
            capacity,
        );
        model.soc = 1.0;

        // Full 1C discharge (10A for 3600s)
        let current = capacity; // 1C
        let dt = 1.0;
        let mut state = model.state();
        for _ in 0..3600 {
            state = model.step(Current(current), dt, Temperature(298.15));
            if state.soc.0 < 0.01 {
                break;
            }
        }
        // SoC should be near 0 after full discharge
        assert!(state.soc.0 < 0.05);
    }

    #[test]
    fn test_1rc_time_constant() {
        let r1 = 0.05_f64;
        let c1 = 3000.0_f64;
        let model = OneRcModel::new(OcvSocCurve::nmc_default(), 0.02, r1, c1, 75.0);
        let expected = r1 * c1;
        assert!(
            (model.time_constant_1() - expected).abs() < f64::EPSILON * expected.abs().max(1.0)
        );
    }

    #[test]
    fn test_1rc_soc_clamping() {
        let model_hi =
            OneRcModel::new(OcvSocCurve::nmc_default(), 0.02, 0.05, 3000.0, 75.0).with_soc(1.5);
        assert_eq!(model_hi.state().soc.0, 1.0);

        let model_lo =
            OneRcModel::new(OcvSocCurve::nmc_default(), 0.02, 0.05, 3000.0, 75.0).with_soc(-0.3);
        assert_eq!(model_lo.state().soc.0, 0.0);
    }

    #[test]
    fn test_1rc_zero_current_steady_state() {
        let mut model = OneRcModel::new(OcvSocCurve::nmc_default(), 0.02, 0.05, 3000.0, 75.0);
        // First apply a small discharge pulse to charge up the RC voltage
        model.step(Current(10.0), 60.0, Temperature(298.15));
        // Then rest for 100 s with zero current — RC voltage should fully decay
        for _ in 0..10 {
            model.step(Current(0.0), 100.0, Temperature(298.15));
        }
        let soc = model.soc;
        let ocv = model.ocv_curve.ocv(soc);
        // With zero current, terminal voltage = OCV − 0·R0 − V_RC1 ≈ OCV when V_RC1 → 0
        let state = model.state();
        let v_terminal = state.voltage.0;
        // RC voltage decays with τ = r1·c1 = 0.05 × 3000 = 150 s.
        // After 10 × 100 s rest the remaining RC voltage is ~0.13 mV; use 0.5 mV tolerance.
        assert!(
            (v_terminal - ocv).abs() < 5e-4,
            "terminal voltage {v_terminal} should equal OCV {ocv} after long rest"
        );
    }

    #[test]
    fn test_1rc_step_soc_decreases_on_discharge() {
        let mut model = OneRcModel::new(OcvSocCurve::nmc_default(), 0.02, 0.05, 3000.0, 75.0);
        let initial_soc = model.state().soc.0;
        // Positive current = discharge
        model.step(Current(10.0), 60.0, Temperature(298.15));
        let new_soc = model.state().soc.0;
        assert!(
            new_soc < initial_soc,
            "SoC {new_soc} should be less than initial {initial_soc} after discharge"
        );
    }

    #[test]
    fn test_1rc_step_soc_increases_on_charge() {
        let mut model =
            OneRcModel::new(OcvSocCurve::nmc_default(), 0.02, 0.05, 3000.0, 75.0).with_soc(0.5);
        let initial_soc = model.state().soc.0;
        // Negative current = charging
        model.step(Current(-10.0), 60.0, Temperature(298.15));
        let new_soc = model.state().soc.0;
        assert!(
            new_soc > initial_soc,
            "SoC {new_soc} should exceed initial {initial_soc} after charging"
        );
    }

    #[test]
    fn test_2rc_internal_resistance() {
        let r0 = 0.010_f64;
        let r1 = 0.025_f64;
        let r2 = 0.015_f64;
        let model = TwoRcModel::new(OcvSocCurve::nmc_default(), r0, r1, 5000.0, r2, 1000.0, 50.0);
        let expected = r0 + r1 + r2;
        let actual = model.state().internal_resistance;
        assert!(
            (actual - expected).abs() < 1e-12,
            "internal_resistance {actual} != r0+r1+r2 = {expected}"
        );
    }

    #[test]
    fn test_2rc_soc_clamping() {
        let model_hi = TwoRcModel::new(
            OcvSocCurve::nmc_default(),
            0.02,
            0.05,
            3000.0,
            0.03,
            500.0,
            75.0,
        )
        .with_soc(1.5);
        assert_eq!(model_hi.state().soc.0, 1.0);

        let model_lo = TwoRcModel::new(
            OcvSocCurve::nmc_default(),
            0.02,
            0.05,
            3000.0,
            0.03,
            500.0,
            75.0,
        )
        .with_soc(-0.3);
        assert_eq!(model_lo.state().soc.0, 0.0);
    }
}
