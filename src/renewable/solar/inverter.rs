/// Grid-connected PV inverter efficiency model.
///
/// Implements a simplified version of the CEC (California Energy Commission)
/// single-point efficiency model.  The inverter converts DC power from the
/// PV array to AC power with a voltage-dependent efficiency curve.
///
/// # Reference
/// Sandia National Laboratories, "Performance Model for Grid-Connected
/// Photovoltaic Inverters" (2007).
use serde::{Deserialize, Serialize};

/// Inverter parameters for the CEC/Sandia model.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InverterParams {
    /// Rated AC output power `W` at reference conditions
    pub p_ac_rated: f64,
    /// DC input power at which efficiency is rated `W`
    pub p_dc_rated: f64,
    /// Reference DC voltage `V`
    pub v_dc_ref: f64,
    /// Self-consumption / minimum operating power `W`
    pub p_self: f64,
    /// Efficiency curve coefficient: quadratic term for power deviation
    pub c0: f64,
    /// Efficiency curve coefficient: linear DC voltage dependence
    pub c1: f64,
    /// Efficiency curve coefficient: power loss vs. DC voltage
    pub c2: f64,
    /// Efficiency curve coefficient: self-consumption vs. DC voltage
    pub c3: f64,
}

impl InverterParams {
    /// Typical residential/commercial string inverter (CEC parameters).
    ///
    /// Based on a representative 5 kW grid-tied inverter (~97 % peak efficiency).
    pub fn residential_5kw() -> Self {
        Self {
            p_ac_rated: 5000.0,
            p_dc_rated: 5200.0,
            v_dc_ref: 400.0,
            p_self: 15.0,
            c0: -2.5e-6,
            c1: -1.5e-5,
            c2: 1.2e-4,
            c3: -8.0e-5,
        }
    }

    /// Utility-scale central inverter (~98.5 % peak efficiency).
    pub fn utility_500kw() -> Self {
        Self {
            p_ac_rated: 500_000.0,
            p_dc_rated: 510_000.0,
            v_dc_ref: 600.0,
            p_self: 200.0,
            c0: -1.5e-7,
            c1: -6.0e-6,
            c2: 8.0e-5,
            c3: -3.0e-5,
        }
    }

    /// Compute AC output power `W` for given DC input.
    ///
    /// Returns 0.0 if `p_dc` is below the self-consumption threshold.
    pub fn ac_power(&self, p_dc: f64, v_dc: f64) -> f64 {
        if p_dc <= self.p_self {
            return 0.0;
        }

        let dv = (v_dc - self.v_dc_ref) / self.v_dc_ref;
        // Adjust rated parameters for DC voltage deviation
        let a = self.p_dc_rated * (1.0 + self.c1 * dv);
        let b = self.p_self * (1.0 + self.c2 * dv);
        let c = self.c0 * (1.0 + self.c3 * dv);

        let denom = a - b;
        if denom.abs() < 1e-6 {
            return 0.0;
        }

        let p_ac = (self.p_ac_rated / denom) * (p_dc - b) + c * (p_dc - b).powi(2);
        p_ac.clamp(0.0, self.p_ac_rated)
    }

    /// Conversion efficiency η = P_ac / P_dc (dimensionless, 0–1).
    pub fn efficiency(&self, p_dc: f64, v_dc: f64) -> f64 {
        if p_dc < 1e-6 {
            return 0.0;
        }
        self.ac_power(p_dc, v_dc) / p_dc
    }

    /// European efficiency: weighted average over typical irradiance distribution.
    ///
    /// η_EU = 0.03·η(5%) + 0.06·η(10%) + 0.13·η(20%) + 0.10·η(30%)
    ///        + 0.48·η(50%) + 0.20·η(100%)
    pub fn european_efficiency(&self) -> f64 {
        let weights = [0.03, 0.06, 0.13, 0.10, 0.48, 0.20];
        let fractions = [0.05, 0.10, 0.20, 0.30, 0.50, 1.00];
        let v = self.v_dc_ref;
        weights
            .iter()
            .zip(fractions.iter())
            .map(|(&w, &f)| w * self.efficiency(f * self.p_dc_rated, v))
            .sum()
    }

    /// CEC efficiency: weighted average over California irradiance profile.
    ///
    /// η_CEC = 0.04·η(10%) + 0.05·η(20%) + 0.12·η(30%) + 0.21·η(50%)
    ///         + 0.53·η(75%) + 0.05·η(100%)
    pub fn cec_efficiency(&self) -> f64 {
        let weights = [0.04, 0.05, 0.12, 0.21, 0.53, 0.05];
        let fractions = [0.10, 0.20, 0.30, 0.50, 0.75, 1.00];
        let v = self.v_dc_ref;
        weights
            .iter()
            .zip(fractions.iter())
            .map(|(&w, &f)| w * self.efficiency(f * self.p_dc_rated, v))
            .sum()
    }
}

/// Simulate an inverter over a time series of DC power values.
///
/// Returns AC power output at each timestep `W`.
pub fn simulate_inverter(params: &InverterParams, p_dc_series: &[f64], v_dc: f64) -> Vec<f64> {
    p_dc_series
        .iter()
        .map(|&p| params.ac_power(p, v_dc))
        .collect()
}

/// Compute daily energy yield `Wh` from DC power series.
///
/// `dt_h` is the time step in hours.
pub fn daily_yield_wh(params: &InverterParams, p_dc_series: &[f64], v_dc: f64, dt_h: f64) -> f64 {
    p_dc_series
        .iter()
        .map(|&p| params.ac_power(p, v_dc) * dt_h)
        .sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_peak_efficiency_near_rated() {
        let inv = InverterParams::residential_5kw();
        let eta = inv.efficiency(inv.p_dc_rated, inv.v_dc_ref);
        assert!(
            eta > 0.90 && eta <= 1.0,
            "Peak efficiency should be > 90%: η={:.4}",
            eta
        );
    }

    #[test]
    fn test_zero_input_zero_output() {
        let inv = InverterParams::residential_5kw();
        assert_eq!(inv.ac_power(0.0, inv.v_dc_ref), 0.0);
    }

    #[test]
    fn test_below_self_consumption_no_output() {
        let inv = InverterParams::residential_5kw();
        assert_eq!(inv.ac_power(inv.p_self * 0.5, inv.v_dc_ref), 0.0);
    }

    #[test]
    fn test_ac_output_does_not_exceed_rated() {
        let inv = InverterParams::residential_5kw();
        let p_ac = inv.ac_power(inv.p_dc_rated * 1.5, inv.v_dc_ref);
        assert!(
            p_ac <= inv.p_ac_rated + 1.0,
            "AC output exceeds rated: {:.1} W",
            p_ac
        );
    }

    #[test]
    fn test_european_efficiency_reasonable() {
        let inv = InverterParams::residential_5kw();
        let eta_eu = inv.european_efficiency();
        assert!(
            eta_eu > 0.80 && eta_eu < 1.0,
            "European efficiency out of range: η_EU={:.4}",
            eta_eu
        );
    }

    #[test]
    fn test_utility_inverter_efficiency_reasonable() {
        let large = InverterParams::utility_500kw();
        let eta = large.efficiency(large.p_dc_rated, large.v_dc_ref);
        // Both residential and utility inverters should be > 85% efficient at rated power
        assert!(
            eta > 0.85 && eta <= 1.0,
            "Utility η={:.4} should be > 85%",
            eta
        );
    }

    #[test]
    fn test_simulate_inverter_series() {
        let inv = InverterParams::residential_5kw();
        let p_dc = vec![0.0, 1000.0, 3000.0, 5000.0, 6000.0];
        let p_ac = simulate_inverter(&inv, &p_dc, inv.v_dc_ref);
        assert_eq!(p_ac.len(), 5);
        assert_eq!(p_ac[0], 0.0);
        assert!(p_ac[3] > p_ac[1]); // higher DC → higher AC
        assert!(p_ac[4] <= inv.p_ac_rated + 1.0); // clipped at rated
    }

    #[test]
    fn test_voltage_deviation_effect() {
        let inv = InverterParams::residential_5kw();
        let p = inv.p_dc_rated * 0.5;
        let p_ref = inv.ac_power(p, inv.v_dc_ref);
        let p_high_v = inv.ac_power(p, inv.v_dc_ref * 1.1);
        // Output should differ with voltage deviation
        assert!(
            (p_ref - p_high_v).abs() < p_ref * 0.05,
            "Voltage effect should be small: {:.1} vs {:.1}",
            p_ref,
            p_high_v
        );
    }
}
