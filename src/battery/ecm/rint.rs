/// Internal-resistance (Rint) battery model.
///
/// The simplest ECM: a voltage source OCV(SoC) in series with a
/// constant internal resistance R0.
///
/// Terminal voltage:
///   V_t = OCV(SoC) − I · R0(T)
///
/// SoC update (Coulomb counting):
///   SoC(k+1) = SoC(k) − I · Δt / (3600 · Q_n · η)
///
/// where I > 0 for discharge, η is Coulombic efficiency.
use crate::battery::{BatteryModel, BatteryState, OcvSocCurve};
use crate::units::{Current, Energy, StateOfCharge, Temperature, Voltage};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RintModel {
    pub ocv_curve: OcvSocCurve,
    /// Internal resistance at reference temperature `Ω`
    pub r0: f64,
    /// Nominal capacity `Ah`
    pub capacity_ah: f64,
    /// Coulombic efficiency (discharge / charge ratio)
    pub coulombic_efficiency: f64,
    /// Temperature coefficient for R0 [1/K]
    pub r0_temp_coeff: f64,
    /// Reference temperature `K`
    pub t_ref: f64,

    // State
    pub soc: f64,
    pub temperature: f64, // K
}

impl RintModel {
    pub fn new(ocv_curve: OcvSocCurve, r0: f64, capacity_ah: f64) -> Self {
        Self {
            ocv_curve,
            r0,
            capacity_ah,
            coulombic_efficiency: 0.98,
            r0_temp_coeff: 0.003,
            t_ref: 298.15,
            soc: 1.0,
            temperature: 298.15,
        }
    }

    pub fn with_soc(mut self, soc: f64) -> Self {
        self.soc = soc.clamp(0.0, 1.0);
        self
    }

    /// Temperature-adjusted R0.
    fn r0_at(&self, temp_k: f64) -> f64 {
        self.r0 * (1.0 + self.r0_temp_coeff * (temp_k - self.t_ref))
    }
}

impl BatteryModel for RintModel {
    fn terminal_voltage(&self, soc: StateOfCharge, current: Current, temp: Temperature) -> Voltage {
        let ocv = self.ocv_curve.ocv(soc.0);
        let r0 = self.r0_at(temp.0);
        Voltage(ocv - current.0 * r0)
    }

    fn step(&mut self, current: Current, dt: f64, temp: Temperature) -> BatteryState {
        let r0 = self.r0_at(temp.0);
        let ocv = self.ocv_curve.ocv(self.soc);
        let v_t = ocv - current.0 * r0;

        // Coulomb counting: positive current = discharge
        let eta = if current.0 >= 0.0 {
            1.0
        } else {
            self.coulombic_efficiency
        };
        let dsoc = -current.0 * dt / (3600.0 * self.capacity_ah) * eta;
        self.soc = (self.soc + dsoc).clamp(0.0, 1.0);
        self.temperature = temp.0;

        BatteryState {
            voltage: Voltage(v_t),
            soc: StateOfCharge::new(self.soc),
            temperature: temp,
            internal_resistance: r0,
            capacity_remaining: Energy(self.soc * self.capacity_ah * v_t),
            current,
        }
    }

    fn state(&self) -> BatteryState {
        let r0 = self.r0_at(self.temperature);
        let ocv = self.ocv_curve.ocv(self.soc);
        BatteryState {
            voltage: Voltage(ocv),
            soc: StateOfCharge::new(self.soc),
            temperature: Temperature(self.temperature),
            internal_resistance: r0,
            capacity_remaining: Energy(self.soc * self.capacity_ah * ocv),
            current: Current(0.0),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_rint() -> RintModel {
        RintModel::new(OcvSocCurve::nmc_default(), 0.05, 75.0)
    }

    #[test]
    fn test_rint_terminal_voltage() {
        let model = make_rint();
        let v = model.terminal_voltage(StateOfCharge::new(1.0), Current(75.0), Temperature(298.15));
        // V = OCV(1.0) - 75*0.05 = 4.2 - 3.75 = 0.45 V
        assert!((v.0 - (4.2 - 75.0 * 0.05)).abs() < 1e-6);
    }

    #[test]
    fn test_rint_discharge_soc_decreases() {
        let mut model = make_rint().with_soc(0.8);
        // 0.1C discharge for 1h: SoC decreases by 0.1
        let state = model.step(Current(7.5), 3600.0, Temperature(298.15));
        assert!(state.soc.0 < 0.8);
        assert!(state.soc.0 > 0.0);
    }

    #[test]
    fn test_rint_charge_soc_increases() {
        let mut model = make_rint().with_soc(0.5);
        let state = model.step(Current(-75.0), 3600.0, Temperature(298.15));
        assert!(state.soc.0 > 0.5);
    }

    #[test]
    fn test_rint_soc_clamped() {
        let mut model = make_rint().with_soc(0.0);
        let state = model.step(Current(1000.0), 3600.0, Temperature(298.15));
        assert_eq!(state.soc.0, 0.0);
    }
}
