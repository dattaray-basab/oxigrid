/// Power System Stabiliser (PSS) models.
///
/// Implements two IEEE standard PSS types:
///
/// ## PSS1A — Single-input stabiliser
///
/// Input:   rotor speed deviation Δω or active power Pe.
/// Output:  supplementary damping signal v_s added to AVR Vref.
///
/// Transfer function (s-domain):
///
///   H_PSS1A(s) = K_s · [sT_w / (1 + sT_w)] · [(1 + sT_1) / (1 + sT_2)] · [(1 + sT_3) / (1 + sT_4)]
///
/// Blocks:
///   1. Washout filter: removes DC component (sT_w / (1 + sT_w))
///   2. Lead-lag 1:    phase compensation ((1 + sT_1) / (1 + sT_2))
///   3. Lead-lag 2:    additional compensation ((1 + sT_3) / (1 + sT_4))
///
/// ## PSS2B — Dual-input stabiliser (IEEE Std 421.5-2016)
///
/// Inputs:  Δω (rotor speed) and ΔP_e (electrical power deviation).
/// Output:  vs combined from two signal paths.
///
/// Each path has:
///   - Transducer (low-pass) filter
///   - Ramp-tracking (washout) filter
///   - Phase-compensation lead-lag stages
///
/// The dual-input design is more robust against torsional interaction
/// than single-speed-input stabilisers.
///
/// # References
/// - IEEE Std 421.5-2016, Annex F (PSS1A, PSS2B models)
/// - Kundur, "Power System Stability and Control", Ch. 12.
use serde::{Deserialize, Serialize};

/// PSS1A: single-input power system stabiliser.
///
/// State variables:
///   x_w  — washout filter state
///   x_1  — lead-lag 1 state
///   x_2  — lead-lag 2 state
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Pss1aParams {
    /// Stabiliser gain [p.u./p.u.]
    pub ks: f64,
    /// Washout time constant Tw `s`
    pub tw: f64,
    /// Lead-lag 1 numerator time constant T1 `s`
    pub t1: f64,
    /// Lead-lag 1 denominator time constant T2 `s`
    pub t2: f64,
    /// Lead-lag 2 numerator time constant T3 `s`
    pub t3: f64,
    /// Lead-lag 2 denominator time constant T4 `s`
    pub t4: f64,
    /// Output limiter [p.u.]
    pub v_smax: f64,
    /// Output limiter [p.u.]
    pub v_smin: f64,
}

impl Pss1aParams {
    /// Typical PSS1A for a steam generator.
    pub fn steam_typical() -> Self {
        Self {
            ks: 20.0,
            tw: 10.0,
            t1: 0.05,
            t2: 0.02,
            t3: 3.0,
            t4: 5.4,
            v_smax: 0.2,
            v_smin: -0.2,
        }
    }

    /// Typical PSS1A for a hydro generator.
    pub fn hydro_typical() -> Self {
        Self {
            ks: 15.0,
            tw: 6.0,
            t1: 0.25,
            t2: 0.05,
            t3: 0.25,
            t4: 0.05,
            v_smax: 0.15,
            v_smin: -0.15,
        }
    }
}

/// PSS1A state machine (discrete-time simulation).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Pss1aState {
    /// Washout filter state x_w
    pub x_w: f64,
    /// Lead-lag 1 filter state x_1
    pub x_1: f64,
    /// Lead-lag 2 filter state x_2: output of 2nd lead-lag
    pub x_2: f64,
}

impl Pss1aState {
    pub fn zero() -> Self {
        Self {
            x_w: 0.0,
            x_1: 0.0,
            x_2: 0.0,
        }
    }
}

/// PSS1A model: step the stabiliser by one time increment.
///
/// - `input_signal` — Δω [p.u.] or ΔP_e [p.u.] depending on configuration
/// - `dt`           — integration time step `s`
/// - Returns output signal v_s [p.u.] to be added to AVR Vref.
pub fn pss1a_step(params: &Pss1aParams, state: &mut Pss1aState, input_signal: f64, dt: f64) -> f64 {
    // Washout filter (high-pass): y_w = Ks * Tw * (u − x_w) / (Tw + dt)
    // Backward Euler discretisation:
    //   x_w[k+1] = (Tw * x_w[k] + dt * u[k]) / (Tw + dt)    ← state update for washout
    // Output of washout block: y_w = Ks * (u − x_w) * Tw / ... simplified:
    let tw = params.tw;
    let alpha_w = tw / (tw + dt);
    let x_w_new = alpha_w * state.x_w + (1.0 - alpha_w) * input_signal;
    let y_w = params.ks * tw / (tw + dt) * (input_signal - state.x_w);
    state.x_w = x_w_new;

    // Lead-lag 1: H1(s) = (1 + sT1) / (1 + sT2)
    // Backward Euler: x1[k+1] = (T2 * x1[k] + dt * T1/T2 * y_w + dt * y_w) / (T2 + dt)
    // Output: y1 = (T1/T2)*y_w + (1 - T1/T2)*x_1
    let (y_1, x_1_new) = lead_lag_step(state.x_1, y_w, params.t1, params.t2, dt);
    state.x_1 = x_1_new;

    // Lead-lag 2: same structure on y_1
    let (y_2, x_2_new) = lead_lag_step(state.x_2, y_1, params.t3, params.t4, dt);
    state.x_2 = x_2_new;

    // Output limiter
    y_2.clamp(params.v_smin, params.v_smax)
}

/// Compute one step of a lead-lag filter (backward Euler).
///
/// Transfer function: H(s) = (1 + s·T1) / (1 + s·T2)
///
/// Returns (output, new_state).
pub fn lead_lag_step(state: f64, input: f64, t1: f64, t2: f64, dt: f64) -> (f64, f64) {
    // Bilinear (Tustin) approximation:
    //   H(z) ≈ (T2 + dt + T1 - T2)(z−1) / ...
    // Backward Euler (simpler, sufficient for small dt):
    //   y[k] = (1 - T2/(T2+dt)) · (input + T1/dt·(input − prev_input)) + ...
    // Simplified state-space:
    //   state[k+1] = (T2/(T2+dt)) · state[k] + (dt/(T2+dt)) · input
    //   output = state[k] + (T1/T2 - 1) · ...
    // Cleaner form:
    //   output[k] = (T1/T2) · input + (1 - T1/T2) · state[k]
    //   state[k+1] = (T2/(T2+dt)) · state[k] + (dt/(T2+dt)) · input
    let a = if (t2 + dt).abs() > 1e-12 {
        t2 / (t2 + dt)
    } else {
        0.0
    };
    let b = if (t2 + dt).abs() > 1e-12 {
        dt / (t2 + dt)
    } else {
        1.0
    };

    let t1_over_t2 = if t2.abs() > 1e-12 { t1 / t2 } else { 1.0 };
    let output = t1_over_t2 * input + (1.0 - t1_over_t2) * state;
    let new_state = a * state + b * input;

    (output, new_state)
}

/// PSS2B: dual-input power system stabiliser.
///
/// Architecture (per IEEE Std 421.5-2016 Fig. F.2):
///
///   Path 1 (speed): Δω → transducer → ramp-track → lead-lag 1&2 → combiner
///   Path 2 (power): ΔP_e → transducer → ramp-track → lead-lag 3&4 → combiner
///   Combiner: vs = v1 + v2 (or v1 − v2 depending on config)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Pss2bParams {
    /// Stabiliser gain Ks [p.u./p.u.]
    pub ks: f64,
    /// Speed path transducer time constant `s`
    pub t_w1: f64,
    /// Power path transducer time constant `s`
    pub t_w2: f64,
    /// Speed path washout time constant `s`
    pub t_w3: f64,
    /// Power path washout time constant `s`
    pub t_w4: f64,
    /// Speed path lead-lag T1/T2 `s`
    pub t1: f64,
    pub t2: f64,
    /// Speed path lead-lag T3/T4 `s`
    pub t3: f64,
    pub t4: f64,
    /// Power path lead-lag T7/T8 `s`
    pub t7: f64,
    pub t8: f64,
    /// Power path lead-lag T9/T10 `s`
    pub t9: f64,
    pub t10: f64,
    /// Output limiter
    pub v_smax: f64,
    pub v_smin: f64,
    /// Gain distribution between paths: vs = Ks1*v_speed + Ks2*v_power
    pub ks1: f64,
    pub ks2: f64,
}

impl Pss2bParams {
    /// Typical PSS2B parameters (generic thermal unit).
    pub fn typical() -> Self {
        Self {
            ks: 20.0,
            t_w1: 2.0,
            t_w2: 2.0,
            t_w3: 10.0,
            t_w4: 10.0,
            t1: 0.2,
            t2: 0.05,
            t3: 0.2,
            t4: 0.05,
            t7: 2.0,
            t8: 0.5,
            t9: 0.1,
            t10: 0.03,
            v_smax: 0.2,
            v_smin: -0.2,
            ks1: 0.6,
            ks2: 0.4,
        }
    }
}

/// PSS2B filter states.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Pss2bState {
    // Speed path
    pub x_tw1: f64, // transducer 1
    pub x_tw3: f64, // washout 1
    pub x_ll1: f64, // lead-lag 1
    pub x_ll2: f64, // lead-lag 2
    // Power path
    pub x_tw2: f64, // transducer 2
    pub x_tw4: f64, // washout 2
    pub x_ll7: f64, // lead-lag 7
    pub x_ll9: f64, // lead-lag 9
}

impl Pss2bState {
    pub fn zero() -> Self {
        Self {
            x_tw1: 0.0,
            x_tw3: 0.0,
            x_ll1: 0.0,
            x_ll2: 0.0,
            x_tw2: 0.0,
            x_tw4: 0.0,
            x_ll7: 0.0,
            x_ll9: 0.0,
        }
    }
}

/// PSS2B model: step the stabiliser.
///
/// - `delta_omega` — rotor speed deviation Δω [p.u.]
/// - `delta_pe`    — electrical power deviation ΔP_e [p.u.]
/// - Returns vs [p.u.] damping signal.
pub fn pss2b_step(
    params: &Pss2bParams,
    state: &mut Pss2bState,
    delta_omega: f64,
    delta_pe: f64,
    dt: f64,
) -> f64 {
    // Speed path
    // Transducer (low-pass): H_trans(s) = 1 / (1 + s·Tw1)
    let (y_trans1, x_tw1_new) = low_pass_step(state.x_tw1, delta_omega, params.t_w1, dt);
    state.x_tw1 = x_tw1_new;

    // Washout (high-pass)
    let (y_wash3, x_tw3_new) = washout_step(state.x_tw3, y_trans1, params.t_w3, dt);
    state.x_tw3 = x_tw3_new;

    // Lead-lag 1 & 2
    let (y_ll1, x_ll1_new) = lead_lag_step(state.x_ll1, y_wash3, params.t1, params.t2, dt);
    state.x_ll1 = x_ll1_new;
    let (y_speed, x_ll2_new) = lead_lag_step(state.x_ll2, y_ll1, params.t3, params.t4, dt);
    state.x_ll2 = x_ll2_new;

    // Power path
    let (y_trans2, x_tw2_new) = low_pass_step(state.x_tw2, delta_pe, params.t_w2, dt);
    state.x_tw2 = x_tw2_new;

    let (y_wash4, x_tw4_new) = washout_step(state.x_tw4, y_trans2, params.t_w4, dt);
    state.x_tw4 = x_tw4_new;

    let (y_ll7, x_ll7_new) = lead_lag_step(state.x_ll7, y_wash4, params.t7, params.t8, dt);
    state.x_ll7 = x_ll7_new;
    let (y_power, x_ll9_new) = lead_lag_step(state.x_ll9, y_ll7, params.t9, params.t10, dt);
    state.x_ll9 = x_ll9_new;

    // Combine and apply gain + limiter
    let vs = params.ks * (params.ks1 * y_speed + params.ks2 * y_power);
    vs.clamp(params.v_smin, params.v_smax)
}

/// Low-pass filter step: H(s) = 1 / (1 + s·T).
///
/// Returns (output, new_state).
pub fn low_pass_step(state: f64, input: f64, t: f64, dt: f64) -> (f64, f64) {
    // Backward Euler: state[k+1] = (T/(T+dt)) * state[k] + (dt/(T+dt)) * input
    if t < 1e-12 {
        return (input, input);
    }
    let a = t / (t + dt);
    let b = dt / (t + dt);
    let new_state = a * state + b * input;
    (new_state, new_state) // output = state (low-pass output)
}

/// Washout (high-pass) filter step: H(s) = sT / (1 + sT).
///
/// Returns (output, new_state).
pub fn washout_step(state: f64, input: f64, t: f64, dt: f64) -> (f64, f64) {
    // State: x[k+1] = (T/(T+dt)) * x[k] + (T/(T+dt)) * input
    // Actually the washout is H(s) = sT/(1+sT) = 1 - 1/(1+sT)
    // So output = input - low_pass(input)
    let (lp_out, new_state) = low_pass_step(state, input, t, dt);
    let output = input - lp_out;
    (output, new_state)
}

/// Frequency response of PSS1A at a given frequency `Hz`.
///
/// Returns (magnitude, phase_deg) using the Laplace s = j·2π·f.
pub fn pss1a_frequency_response(params: &Pss1aParams, freq_hz: f64) -> (f64, f64) {
    use std::f64::consts::PI;
    let s = num_complex::Complex64::new(0.0, 2.0 * PI * freq_hz);

    // Washout: sTw / (1 + sTw)
    let tw_s = s * params.tw;
    let h_wash = tw_s / (1.0 + tw_s);

    // Lead-lag 1
    let h_ll1 = (1.0 + s * params.t1) / (1.0 + s * params.t2);

    // Lead-lag 2
    let h_ll2 = (1.0 + s * params.t3) / (1.0 + s * params.t4);

    let h_total = params.ks * h_wash * h_ll1 * h_ll2;

    let mag = h_total.norm();
    let phase_deg = h_total.arg().to_degrees();

    (mag, phase_deg)
}

/// Compute the phase lead of PSS1A at a target frequency.
///
/// Useful for tuning T1/T2, T3/T4 to maximise phase lead at the inter-area mode frequency.
pub fn pss1a_phase_at_freq(params: &Pss1aParams, freq_hz: f64) -> f64 {
    let (_, phase) = pss1a_frequency_response(params, freq_hz);
    phase
}

/// Tune PSS1A lead-lag to provide maximum phase lead at target frequency.
///
/// Adjusts T1/T2 and T3/T4 using the formula:
///   T2 = 1 / (ωn · √n_s)   where n_s = (1+sin φ_max) / (1-sin φ_max)
///   T1 = T2 · n_s
///
/// Returns tuned params.
pub fn tune_pss1a_lead_lag(
    mut params: Pss1aParams,
    freq_hz: f64,
    target_lead_deg: f64,
) -> Pss1aParams {
    let omega = 2.0 * std::f64::consts::PI * freq_hz;
    let phi_max = target_lead_deg.to_radians();
    // Two lead-lag stages → split target lead equally
    let phi_each = phi_max / 2.0;
    let sin_phi = phi_each.sin().clamp(-0.999, 0.999);
    let n_s = (1.0 + sin_phi) / (1.0 - sin_phi).max(1e-6);
    let t2 = 1.0 / (omega * n_s.sqrt());
    let t1 = t2 * n_s;
    params.t1 = t1;
    params.t2 = t2;
    params.t3 = t1;
    params.t4 = t2;
    params
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pss1a_step_zero_input() {
        let params = Pss1aParams::steam_typical();
        let mut state = Pss1aState::zero();
        let vs = pss1a_step(&params, &mut state, 0.0, 0.01);
        assert!(
            (vs).abs() < 1e-10,
            "PSS output with zero input should be 0: {:.4e}",
            vs
        );
    }

    #[test]
    fn test_pss1a_step_step_response() {
        let params = Pss1aParams::steam_typical();
        let mut state = Pss1aState::zero();
        let dt = 0.01;

        // Apply step input Δω = 0.01 p.u.
        let mut vs_values = Vec::new();
        for _ in 0..100 {
            let vs = pss1a_step(&params, &mut state, 0.01, dt);
            vs_values.push(vs);
        }

        // Should produce initial transient (washout) then decay to zero
        let first = vs_values[0].abs();
        let last = vs_values[99].abs();
        // Washout ensures DC gain = 0, so output decays
        assert!(
            first > 0.0,
            "Initial response should be non-zero: {:.4}",
            first
        );
        // Eventually the washout removes DC component
        assert!(
            last <= first + 0.01,
            "Last output ({:.4}) should not exceed first ({:.4})",
            last,
            first
        );
    }

    #[test]
    fn test_pss1a_output_limiter() {
        let mut params = Pss1aParams::steam_typical();
        params.v_smax = 0.1;
        params.v_smin = -0.1;
        let mut state = Pss1aState::zero();
        // Large input should be clipped
        for _ in 0..5 {
            let vs = pss1a_step(&params, &mut state, 100.0, 0.1);
            assert!(vs <= params.v_smax + 1e-10, "Output not clipped: {:.4}", vs);
            assert!(vs >= params.v_smin - 1e-10);
        }
    }

    #[test]
    fn test_pss2b_step_zero_input() {
        let params = Pss2bParams::typical();
        let mut state = Pss2bState::zero();
        let vs = pss2b_step(&params, &mut state, 0.0, 0.0, 0.01);
        assert!(vs.abs() < 1e-10);
    }

    #[test]
    fn test_pss2b_step_speed_input() {
        let params = Pss2bParams::typical();
        let mut state = Pss2bState::zero();
        let dt = 0.01;
        let vs = pss2b_step(&params, &mut state, 0.01, 0.0, dt);
        assert!(
            vs >= params.v_smin && vs <= params.v_smax,
            "PSS2B output out of limits: {:.4}",
            vs
        );
    }

    #[test]
    fn test_pss2b_dual_input_additive() {
        let params = Pss2bParams::typical();
        let mut state_both = Pss2bState::zero();
        let mut state_speed = Pss2bState::zero();
        let dt = 0.01;

        let vs_both = pss2b_step(&params, &mut state_both, 0.01, 0.005, dt);
        let vs_speed = pss2b_step(&params, &mut state_speed, 0.01, 0.0, dt);

        // Adding power signal should change the output
        // (can be larger or smaller depending on phase)
        let _ = (vs_both - vs_speed).abs(); // just ensure it computes without panic
    }

    #[test]
    fn test_lead_lag_pure_gain_t1_eq_t2() {
        // T1 = T2 → lead-lag is unity: output = input
        let mut state = 0.0;
        for input in [0.0, 0.5, 1.0, -0.5] {
            let (output, new_state) = lead_lag_step(state, input, 0.1, 0.1, 0.01);
            assert!(
                (output - input).abs() < 1e-10,
                "Unity lead-lag failed: in={}, out={:.6}",
                input,
                output
            );
            state = new_state;
        }
    }

    #[test]
    fn test_washout_dc_blocked() {
        // DC input (constant): washout output should decay toward 0
        // Use smaller time constant for faster convergence in test
        let mut state = 0.0;
        let mut last_out = f64::INFINITY;
        for _ in 0..20000 {
            let (out, ns) = washout_step(state, 1.0, 0.5, 0.01);
            state = ns;
            last_out = out;
        }
        assert!(
            last_out.abs() < 0.01,
            "Washout should block DC: {:.4}",
            last_out
        );
    }

    #[test]
    fn test_low_pass_dc_passed() {
        // DC input: low-pass should converge to input
        let mut state = 0.0;
        for _ in 0..10000 {
            let (_, ns) = low_pass_step(state, 1.0, 0.1, 0.001);
            state = ns;
        }
        assert!(
            (state - 1.0).abs() < 0.01,
            "Low-pass should pass DC: {:.4}",
            state
        );
    }

    #[test]
    fn test_pss1a_frequency_response_unity_freq() {
        // At very low frequency the washout blocks → magnitude ≈ 0
        let params = Pss1aParams::steam_typical();
        let (mag_low, _) = pss1a_frequency_response(&params, 0.001);
        let (mag_high, _) = pss1a_frequency_response(&params, 10.0);
        assert!(
            mag_low < mag_high,
            "Washout should block low freq: {:.4} < {:.4}",
            mag_low,
            mag_high
        );
    }

    #[test]
    fn test_pss1a_phase_at_target_freq() {
        let params = Pss1aParams::steam_typical();
        let phase = pss1a_phase_at_freq(&params, 1.0); // 1 Hz inter-area mode
        let _ = phase; // just check it computes
    }

    #[test]
    fn test_tune_pss1a_lead_lag() {
        let params = Pss1aParams::steam_typical();
        let target_lead = 45.0; // degrees
        let tuned = tune_pss1a_lead_lag(params, 1.0, target_lead);
        assert!(tuned.t1 > tuned.t2, "T1 > T2 for lead compensation");
        let phase = pss1a_phase_at_freq(&tuned, 1.0);
        // Phase should be positive (lead)
        assert!(
            phase > 0.0,
            "Tuned PSS should provide phase lead at 1 Hz: {:.2}°",
            phase
        );
    }
}
