/// Single-diode photovoltaic cell/module model.
///
/// Uses the 5-parameter single-diode equation:
///
///   I = I_ph − I_0 · (exp((V + I·R_s) / (n·V_T)) − 1) − (V + I·R_s) / R_sh
///
/// Parameters are derived from STC (Standard Test Conditions:
/// G = 1000 W/m², T = 25 °C, AM 1.5).
use serde::{Deserialize, Serialize};

/// Physical constants
const K_B: f64 = 1.380649e-23; // Boltzmann constant [J/K]
const Q_E: f64 = 1.602176634e-19; // Electron charge [C]

/// Five-parameter single-diode model parameters at STC.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SingleDiodeParams {
    /// Photocurrent `A` at STC
    pub i_ph_stc: f64,
    /// Dark saturation current `A` at STC
    pub i_0_stc: f64,
    /// Series resistance `Ω`
    pub r_s: f64,
    /// Shunt resistance `Ω`
    pub r_sh: f64,
    /// Ideality factor (typically 1.0–1.5)
    pub n_diode: f64,
    /// Number of cells in series
    pub n_cells: u32,
    /// Temperature coefficient of I_sc [A/K]
    pub alpha_isc: f64,
    /// Temperature coefficient of V_oc [V/K]
    pub beta_voc: f64,
}

impl SingleDiodeParams {
    /// Typical crystalline silicon 60-cell module (≈ 250 Wp at STC).
    pub fn crystalline_si_250w() -> Self {
        Self {
            i_ph_stc: 8.76,
            i_0_stc: 2.5e-10,
            r_s: 0.38,
            r_sh: 300.0,
            n_diode: 1.1,
            n_cells: 60,
            alpha_isc: 0.0053, // A/K
            beta_voc: -0.1090, // V/K
        }
    }

    /// Typical thin-film (CdTe) module (≈ 85 Wp, 116 cells).
    pub fn thin_film_cdte_85w() -> Self {
        Self {
            i_ph_stc: 1.17,
            i_0_stc: 1.0e-12,
            r_s: 3.0,
            r_sh: 1500.0,
            n_diode: 1.5,
            n_cells: 116,
            alpha_isc: 0.00043,
            beta_voc: -0.160,
        }
    }

    /// Thermal voltage at temperature T `K`.
    pub fn v_t(&self, temp_k: f64) -> f64 {
        self.n_diode * K_B * temp_k / Q_E * self.n_cells as f64
    }
}

/// Point on an I-V curve.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct IVPoint {
    pub voltage: f64,
    pub current: f64,
    pub power: f64,
}

/// Solve the implicit single-diode equation for current at voltage V
/// using Newton-Raphson iteration.
///
/// Returns I such that the diode equation is satisfied.
pub fn diode_current(params: &SingleDiodeParams, v: f64, g: f64, temp_k: f64) -> f64 {
    let g_ratio = (g / 1000.0).max(0.0);
    let dt = temp_k - 298.15; // ΔT from STC

    let i_ph = (params.i_ph_stc + params.alpha_isc * dt) * g_ratio;
    let i_0 = params.i_0_stc
        * (temp_k / 298.15).powi(3)
        * ((Q_E / (params.n_diode * K_B)) * (1.0 / 298.15 - 1.0 / temp_k)).exp();
    let v_t = params.v_t(temp_k);

    if g <= 0.0 {
        return 0.0;
    }

    // Initial guess: no series resistance
    let mut i = i_ph - i_0 * ((v / v_t).exp() - 1.0);
    i = i.clamp(0.0, i_ph);

    // Newton-Raphson for implicit solution
    for _ in 0..50 {
        let exp_arg = (v + i * params.r_s) / v_t;
        let exp_val = exp_arg.min(700.0).exp();
        let f = i - i_ph + i_0 * (exp_val - 1.0) + (v + i * params.r_s) / params.r_sh;
        let df = 1.0 + i_0 * params.r_s / v_t * exp_val + params.r_s / params.r_sh;
        let di = f / df;
        i -= di;
        i = i.clamp(0.0, i_ph * 1.1);
        if di.abs() < 1e-9 {
            break;
        }
    }
    i.max(0.0)
}

/// Compute the maximum power point (MPP) by scanning the I-V curve.
///
/// Returns (V_mpp, I_mpp, P_mpp).
pub fn find_mpp(params: &SingleDiodeParams, g: f64, temp_k: f64) -> IVPoint {
    if g <= 0.0 {
        return IVPoint {
            voltage: 0.0,
            current: 0.0,
            power: 0.0,
        };
    }

    // Estimate V_oc
    let v_oc_stc = params.n_cells as f64 * params.n_diode * K_B * 298.15 / Q_E
        * ((params.i_ph_stc / params.i_0_stc) + 1.0).ln();
    let dt = temp_k - 298.15;
    let v_oc_est = (v_oc_stc + params.beta_voc * dt).max(0.1);

    // Golden-section search for Pmax on [0, V_oc]
    let phi = (5.0_f64.sqrt() - 1.0) / 2.0;
    let mut a = 0.0_f64;
    let mut b = v_oc_est;
    let mut c = b - phi * (b - a);
    let mut d = a + phi * (b - a);

    for _ in 0..50 {
        let pc = diode_current(params, c, g, temp_k) * c;
        let pd = diode_current(params, d, g, temp_k) * d;
        if pc < pd {
            a = c;
            c = d;
            d = a + phi * (b - a);
        } else {
            b = d;
            d = c;
            c = b - phi * (b - a);
        }
        if (b - a).abs() < 1e-6 {
            break;
        }
    }

    let v_mpp = (a + b) / 2.0;
    let i_mpp = diode_current(params, v_mpp, g, temp_k);
    let p_mpp = v_mpp * i_mpp;

    IVPoint {
        voltage: v_mpp,
        current: i_mpp,
        power: p_mpp,
    }
}

/// Sample the I-V curve at `n_points` evenly spaced voltages.
pub fn iv_curve(params: &SingleDiodeParams, g: f64, temp_k: f64, n_points: usize) -> Vec<IVPoint> {
    if g <= 0.0 || n_points == 0 {
        return vec![];
    }

    let v_oc_stc = params.n_cells as f64 * params.n_diode * K_B * 298.15 / Q_E
        * ((params.i_ph_stc / params.i_0_stc) + 1.0).ln();
    let dt = temp_k - 298.15;
    let v_oc = (v_oc_stc + params.beta_voc * dt).max(0.1);

    (0..n_points)
        .map(|k| {
            let v = v_oc * k as f64 / (n_points - 1) as f64;
            let i = diode_current(params, v, g, temp_k);
            IVPoint {
                voltage: v,
                current: i,
                power: v * i,
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mpp_at_stc() {
        let params = SingleDiodeParams::crystalline_si_250w();
        let mpp = find_mpp(&params, 1000.0, 298.15);
        // 250W module: Pmpp should be ~240-285 W at STC
        assert!(
            mpp.power > 230.0 && mpp.power < 285.0,
            "Pmpp={:.1} W",
            mpp.power
        );
        assert!(
            mpp.voltage > 20.0 && mpp.voltage < 35.0,
            "Vmpp={:.2} V",
            mpp.voltage
        );
    }

    #[test]
    fn test_mpp_lower_irradiance() {
        let params = SingleDiodeParams::crystalline_si_250w();
        let mpp_full = find_mpp(&params, 1000.0, 298.15);
        let mpp_half = find_mpp(&params, 500.0, 298.15);
        // Power roughly proportional to irradiance
        assert!(mpp_half.power < mpp_full.power);
        assert!(mpp_half.power > mpp_full.power * 0.4);
    }

    #[test]
    fn test_mpp_higher_temp_lower_power() {
        let params = SingleDiodeParams::crystalline_si_250w();
        let mpp_cool = find_mpp(&params, 1000.0, 298.15);
        let mpp_hot = find_mpp(&params, 1000.0, 328.15); // +30°C
        assert!(
            mpp_hot.power < mpp_cool.power,
            "hot={:.1} cool={:.1}",
            mpp_hot.power,
            mpp_cool.power
        );
    }

    #[test]
    fn test_zero_irradiance_gives_zero_power() {
        let params = SingleDiodeParams::crystalline_si_250w();
        let mpp = find_mpp(&params, 0.0, 298.15);
        assert_eq!(mpp.power, 0.0);
    }

    #[test]
    fn test_iv_curve_monotone_voltage() {
        let params = SingleDiodeParams::crystalline_si_250w();
        let curve = iv_curve(&params, 800.0, 298.15, 20);
        for i in 1..curve.len() {
            assert!(curve[i].voltage >= curve[i - 1].voltage);
        }
    }
}
