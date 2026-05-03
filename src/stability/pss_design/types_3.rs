//! Auto-generated module
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

use std::f64::consts::PI;

use super::types::{
    FreqResponsePoint, GeneratorModal, PssBandParams, PssDesignResult, PssInput, PssModel,
    PssState, PssType, TransferFunction,
};

/// PSS designer using the residue method for automated parameter tuning.
pub struct PssDesigner {
    /// Modal data for each generator.
    pub modal_data: Vec<GeneratorModal>,
    /// Tuning configuration.
    pub config: PssTuningConfig,
}
impl PssDesigner {
    /// Create a new `PssDesigner` with the given configuration.
    pub fn new(config: PssTuningConfig) -> Self {
        Self {
            modal_data: Vec::new(),
            config,
        }
    }
    /// Add modal data for a generator.
    pub fn add_generator_modal(&mut self, modal: GeneratorModal) {
        self.modal_data.push(modal);
    }
    /// Design a PSS for the generator with index `gen_id`.
    ///
    /// Returns a [`PssDesignResult`] or an error string if the generator is
    /// not found or tuning fails.
    pub fn design_pss(&self, gen_id: usize) -> std::result::Result<PssDesignResult, String> {
        let modal = self
            .modal_data
            .iter()
            .find(|m| m.gen_id == gen_id)
            .ok_or_else(|| format!("No modal data found for generator {gen_id}"))?;
        let pss_model = match self.config.pss_type {
            PssType::Pss1A => self.design_pss1a(modal)?,
            PssType::Pss2B => self.design_pss2b(modal)?,
            PssType::Pss4B => self.design_pss4b(modal)?,
        };
        let tf = Self::compute_transfer_function(&pss_model);
        let freq_range = Self::log_freq_range(
            self.config.freq_min_hz,
            self.config.freq_max_hz,
            self.config.n_freq_points,
        );
        let freq_response = Self::evaluate_frequency_response(&tf, &freq_range);
        let (_, phase_at_mode) = tf.evaluate_at_freq(modal.mode_freq_hz);
        let phase_compensation_deg = phase_at_mode;
        let (gain_margin_db, phase_margin_deg) = Self::compute_gain_phase_margins(&freq_response);
        let omega_m = 2.0 * PI * modal.mode_freq_hz.max(1e-9);
        let (mag_at_mode_db, _) = tf.evaluate_at_freq(modal.mode_freq_hz);
        let mag_at_mode = 10.0_f64.powf(mag_at_mode_db / 20.0);
        let two_h = 2.0 * modal.inertia_h.max(1e-9);
        let expected_damping_improvement =
            mag_at_mode * modal.residue_magnitude / (two_h * omega_m);
        let design_converged = gain_margin_db > 0.0 && phase_margin_deg > 0.0;
        Ok(PssDesignResult {
            generator_id: gen_id,
            pss_model,
            transfer_function: tf,
            freq_response,
            phase_compensation_deg,
            gain_margin_db,
            phase_margin_deg,
            expected_damping_improvement,
            design_converged,
        })
    }
    /// Design a PSS1A via the residue phase-compensation method.
    fn design_pss1a(&self, modal: &GeneratorModal) -> std::result::Result<PssModel, String> {
        let phi_r = modal.residue_angle_deg;
        let phi_c = 180.0 - phi_r;
        let phi_each_deg = phi_c / 2.0;
        let (t1, t2) = Self::lead_lag_constants(phi_each_deg, modal.mode_freq_hz);
        let (t3, t4) = (t1, t2);
        let tw = 10.0 / (2.0 * PI * 0.1_f64);
        let k_target = 10.0_f64.powf(self.config.target_gain_db / 20.0);
        let ll_tf = TransferFunction::lead_lag(t1, t2).series(&TransferFunction::lead_lag(t3, t4));
        let (mag_ll_db, _) = ll_tf.evaluate_at_freq(modal.mode_freq_hz);
        let mag_ll = 10.0_f64.powf(mag_ll_db / 20.0).max(1e-9);
        let k_s = k_target / (modal.residue_magnitude.max(1e-9) * mag_ll);
        let k_s = k_s.clamp(0.1, 100.0);
        Ok(PssModel::Pss1A {
            input: PssInput::RotorSpeed,
            tw,
            lead_lag_1: (t1, t2),
            lead_lag_2: (t3, t4),
            k_s,
            v_st_min: -0.1,
            v_st_max: 0.1,
        })
    }
    /// Design a PSS2B dual-input stabiliser.
    fn design_pss2b(&self, modal: &GeneratorModal) -> std::result::Result<PssModel, String> {
        let phi_r = modal.residue_angle_deg;
        let phi_c = 180.0 - phi_r;
        let phi_each_deg = phi_c / 2.0;
        let (t1, t2) = Self::lead_lag_constants(phi_each_deg, modal.mode_freq_hz);
        let (t3, t4) = (t1, t2);
        let tw_base = 10.0 / (2.0 * PI * 0.1_f64);
        let k_target = 10.0_f64.powf(self.config.target_gain_db / 20.0);
        let k_s1 = (k_target / modal.residue_magnitude.max(1e-9)).clamp(0.1, 100.0);
        let k_s2 = k_s1 * 0.5;
        Ok(PssModel::Pss2B {
            k_s1,
            k_s2,
            t_w1: tw_base,
            t_w2: tw_base,
            t_w3: tw_base,
            t_w4: tw_base,
            t1,
            t2,
            t3,
            t4,
            t10: 0.2,
            t11: 0.05,
            v_st_min: -0.1,
            v_st_max: 0.1,
        })
    }
    /// Design a PSS4B multi-band stabiliser.
    fn design_pss4b(&self, modal: &GeneratorModal) -> std::result::Result<PssModel, String> {
        let f_l = 0.05_f64;
        let f_i = modal.mode_freq_hz.clamp(0.1, 1.0);
        let f_h = 2.0_f64;
        let phi_r = modal.residue_angle_deg;
        let phi_c = 180.0 - phi_r;
        let phi_each = phi_c / 2.0;
        let (tl1, tl2) = Self::lead_lag_constants(phi_each, f_l);
        let (ti1, ti2) = Self::lead_lag_constants(phi_each, f_i);
        let (th1, th2) = Self::lead_lag_constants(phi_each, f_h);
        let k_target = 10.0_f64.powf(self.config.target_gain_db / 20.0);
        let k_l = (k_target / 3.0 / modal.residue_magnitude.max(1e-9)).clamp(0.1, 50.0);
        let k_i = k_l;
        let k_h = k_l * 0.5;
        let low_band = PssBandParams {
            k_l,
            t_l1: tl1,
            t_l2: tl2,
            t_l3: tl1,
            t_l4: tl2,
            v_lmax: 0.05,
            v_lmin: -0.05,
        };
        let inter_band = PssBandParams {
            k_l: k_i,
            t_l1: ti1,
            t_l2: ti2,
            t_l3: ti1,
            t_l4: ti2,
            v_lmax: 0.05,
            v_lmin: -0.05,
        };
        let high_band = PssBandParams {
            k_l: k_h,
            t_l1: th1,
            t_l2: th2,
            t_l3: th1,
            t_l4: th2,
            v_lmax: 0.03,
            v_lmin: -0.03,
        };
        Ok(PssModel::Pss4B {
            low_band,
            inter_band,
            high_band,
            v_st_min: -0.1,
            v_st_max: 0.1,
        })
    }
    /// Compute lead-lag time constants T1 > T2 that provide phase lead `phi_deg`
    /// at frequency `f_hz` using the standard residue-method formulae.
    ///
    /// ```text
    /// α = (1 − sin φ) / (1 + sin φ)
    /// T1 = 1 / (ω_m × √α)
    /// T2 = α × T1
    /// ```
    pub(crate) fn lead_lag_constants(phi_deg: f64, f_hz: f64) -> (f64, f64) {
        let omega = 2.0 * PI * f_hz.max(1e-6);
        let phi = phi_deg.clamp(1.0, 89.0).to_radians();
        let sin_phi = phi.sin().clamp(1e-9, 0.9999);
        let alpha = (1.0 - sin_phi) / (1.0 + sin_phi);
        let alpha = alpha.max(1e-6);
        let t1 = 1.0 / (omega * alpha.sqrt());
        let t2 = alpha * t1;
        (t1, t2)
    }
    /// Build the equivalent `TransferFunction` for a `PssModel`.
    pub fn compute_transfer_function(model: &PssModel) -> TransferFunction {
        match model {
            PssModel::Pss1A {
                tw,
                lead_lag_1,
                lead_lag_2,
                k_s,
                ..
            } => TransferFunction::gain(*k_s)
                .series(&TransferFunction::washout(*tw))
                .series(&TransferFunction::lead_lag(lead_lag_1.0, lead_lag_1.1))
                .series(&TransferFunction::lead_lag(lead_lag_2.0, lead_lag_2.1)),
            PssModel::Pss2B {
                k_s1,
                t_w1,
                t1,
                t2,
                t3,
                t4,
                ..
            } => TransferFunction::gain(*k_s1)
                .series(&TransferFunction::washout(*t_w1))
                .series(&TransferFunction::lead_lag(*t1, *t2))
                .series(&TransferFunction::lead_lag(*t3, *t4)),
            PssModel::Pss4B {
                low_band,
                inter_band,
                high_band,
                ..
            } => {
                let tf_l = TransferFunction::gain(low_band.k_l)
                    .series(&TransferFunction::lead_lag(low_band.t_l1, low_band.t_l2));
                let tf_i = TransferFunction::gain(inter_band.k_l).series(
                    &TransferFunction::lead_lag(inter_band.t_l1, inter_band.t_l2),
                );
                let tf_h = TransferFunction::gain(high_band.k_l)
                    .series(&TransferFunction::lead_lag(high_band.t_l1, high_band.t_l2));
                tf_l.series(&tf_i).series(&tf_h)
            }
        }
    }
    /// Evaluate a transfer function over an array of frequencies \[Hz\].
    pub fn evaluate_frequency_response(
        tf: &TransferFunction,
        freq_hz_range: &[f64],
    ) -> Vec<FreqResponsePoint> {
        freq_hz_range
            .iter()
            .map(|&f| {
                let (mag_db, phase_deg) = tf.evaluate_at_freq(f);
                FreqResponsePoint {
                    freq_hz: f,
                    magnitude_db: mag_db,
                    phase_deg,
                }
            })
            .collect()
    }
    /// Compute gain margin \[dB\] and phase margin \[deg\] from frequency response data.
    ///
    /// - **Gain margin**: `-magnitude_db` at the phase-crossover frequency (phase = −180°).
    /// - **Phase margin**: `phase + 180°` at the gain-crossover frequency (magnitude = 0 dB).
    pub fn compute_gain_phase_margins(freq_resp: &[FreqResponsePoint]) -> (f64, f64) {
        let mut gain_margin_db = 20.0_f64;
        let mut phase_margin_deg = 45.0_f64;
        for w in freq_resp.windows(2) {
            let p1 = w[0].phase_deg;
            let g1 = w[0].magnitude_db;
            let p2 = w[1].phase_deg;
            let g2 = w[1].magnitude_db;
            if (p1 + 180.0) * (p2 + 180.0) <= 0.0 {
                let frac = if (p2 - p1).abs() < 1e-12 {
                    0.5
                } else {
                    (-180.0 - p1) / (p2 - p1)
                };
                let g_cross = g1 + frac * (g2 - g1);
                gain_margin_db = -g_cross;
                break;
            }
        }
        for w in freq_resp.windows(2) {
            let g1 = w[0].magnitude_db;
            let g2 = w[1].magnitude_db;
            let p1 = w[0].phase_deg;
            let p2 = w[1].phase_deg;
            if g1 * g2 <= 0.0 {
                let frac = if (g2 - g1).abs() < 1e-12 {
                    0.5
                } else {
                    (0.0 - g1) / (g2 - g1)
                };
                let p_cross = p1 + frac * (p2 - p1);
                phase_margin_deg = p_cross + 180.0;
                break;
            }
        }
        (gain_margin_db, phase_margin_deg)
    }
    /// Advance PSS state by one time step `dt_s` with scalar `input`.
    ///
    /// Uses forward-Euler integration of the washout + lead-lag filter chain.
    /// Returns the clipped output voltage \[pu\].
    pub fn simulate_pss_step(model: &PssModel, state: &mut PssState, input: f64, dt_s: f64) -> f64 {
        let dt = dt_s.max(1e-9);
        state.time_s += dt;
        let out = match model {
            PssModel::Pss1A {
                tw,
                lead_lag_1,
                lead_lag_2,
                k_s,
                v_st_min,
                v_st_max,
                ..
            } => {
                while state.states.len() < 3 {
                    state.states.push(0.0);
                }
                let x_w = state.states[0];
                let x_l1 = state.states[1];
                let x_l2 = state.states[2];
                let tw_s = tw.max(1e-9);
                let wash = input - x_w;
                let (t1, t2) = *lead_lag_1;
                let (t3, t4) = *lead_lag_2;
                let y_l1 = Self::ll_output(t1, t2, wash, x_l1);
                let y_l2 = Self::ll_output(t3, t4, y_l1, x_l2);
                state.states[0] += dt / tw_s * (input - x_w);
                state.states[1] += dt / t2.max(1e-9) * (wash - x_l1);
                state.states[2] += dt / t4.max(1e-9) * (y_l1 - x_l2);
                (k_s * y_l2).clamp(*v_st_min, *v_st_max)
            }
            PssModel::Pss2B {
                k_s1,
                t_w1,
                t1,
                t2,
                t3,
                t4,
                v_st_min,
                v_st_max,
                ..
            } => {
                while state.states.len() < 3 {
                    state.states.push(0.0);
                }
                let x_w = state.states[0];
                let x_l1 = state.states[1];
                let x_l2 = state.states[2];
                let tw_s = t_w1.max(1e-9);
                let wash = input - x_w;
                let y_l1 = Self::ll_output(*t1, *t2, wash, x_l1);
                let y_l2 = Self::ll_output(*t3, *t4, y_l1, x_l2);
                state.states[0] += dt / tw_s * (input - x_w);
                state.states[1] += dt / t2.max(1e-9) * (wash - x_l1);
                state.states[2] += dt / t4.max(1e-9) * (y_l1 - x_l2);
                (k_s1 * y_l2).clamp(*v_st_min, *v_st_max)
            }
            PssModel::Pss4B {
                low_band,
                inter_band,
                high_band,
                v_st_min,
                v_st_max,
            } => {
                while state.states.len() < 3 {
                    state.states.push(0.0);
                }
                let x_l = state.states[0];
                let x_i = state.states[1];
                let x_h = state.states[2];
                let y_l = Self::ll_output(low_band.t_l1, low_band.t_l2, input, x_l);
                let y_i = Self::ll_output(inter_band.t_l1, inter_band.t_l2, input, x_i);
                let y_h = Self::ll_output(high_band.t_l1, high_band.t_l2, input, x_h);
                state.states[0] += dt / low_band.t_l2.max(1e-9) * (input - x_l);
                state.states[1] += dt / inter_band.t_l2.max(1e-9) * (input - x_i);
                state.states[2] += dt / high_band.t_l2.max(1e-9) * (input - x_h);
                let vst = low_band.k_l * y_l.clamp(low_band.v_lmin, low_band.v_lmax)
                    + inter_band.k_l * y_i.clamp(inter_band.v_lmin, inter_band.v_lmax)
                    + high_band.k_l * y_h.clamp(high_band.v_lmin, high_band.v_lmax);
                vst.clamp(*v_st_min, *v_st_max)
            }
        };
        state.output = out;
        out
    }
    /// Lead-lag output: `y = x + (T1/T2) * (u − x)`.
    fn ll_output(t1: f64, t2: f64, u: f64, x: f64) -> f64 {
        let ratio = if t2.abs() > 1e-12 { t1 / t2 } else { 1.0 };
        x + ratio * (u - x)
    }
    /// Generate `n` logarithmically spaced frequencies between `f_min` and `f_max` \[Hz\].
    fn log_freq_range(f_min: f64, f_max: f64, n: usize) -> Vec<f64> {
        let n = n.max(2);
        let lo = f_min.max(1e-6).log10();
        let hi = f_max.max(f_min * 10.0).log10();
        (0..n)
            .map(|i| 10.0_f64.powf(lo + (hi - lo) * i as f64 / (n - 1) as f64))
            .collect()
    }
}
/// PSS tuning configuration.
#[derive(Debug, Clone)]
pub struct PssTuningConfig {
    /// Target closed-loop damping ratio (default 0.05).
    pub target_damping: f64,
    /// Target modal gain at mode frequency \[dB\] (default 20 dB).
    pub target_gain_db: f64,
    /// Minimum frequency for frequency response \[Hz\].
    pub freq_min_hz: f64,
    /// Maximum frequency for frequency response \[Hz\].
    pub freq_max_hz: f64,
    /// Number of logarithmically spaced frequency points.
    pub n_freq_points: usize,
    /// PSS type to design.
    pub pss_type: PssType,
}
/// Heffron-Phillips PSS design result (legacy).
#[derive(Debug, Clone)]
pub struct HpDesignResult {
    /// The designed PSS model.
    pub pss_model: PssModel,
    /// Achieved closed-loop damping ratio.
    pub achieved_damping: f64,
    /// Achieved closed-loop mode frequency \[Hz\].
    pub achieved_frequency_hz: f64,
    /// Phase margin \[deg\].
    pub phase_margin_deg: f64,
    /// Gain margin \[dB\].
    pub gain_margin_db: f64,
    /// True when the open-loop system (without PSS) is stable.
    pub is_stable_without_pss: bool,
    /// Open-loop critical eigenvalue.
    pub eigenvalue_without_pss: Eigenvalue,
    /// Closed-loop critical eigenvalue with PSS.
    pub eigenvalue_with_pss: Eigenvalue,
    /// Human-readable design notes.
    pub design_notes: Vec<String>,
}
/// Small-signal model eigenvalue with frequency and damping.
#[derive(Debug, Clone)]
pub struct Eigenvalue {
    /// Real part σ.
    pub real: f64,
    /// Imaginary part ω \[rad/s\].
    pub imag: f64,
    /// Oscillation frequency \[Hz\].
    pub frequency_hz: f64,
    /// Damping ratio ζ.
    pub damping_ratio: f64,
}
impl Eigenvalue {
    /// Construct from real + imaginary parts.
    pub fn new(real: f64, imag: f64) -> Self {
        let magnitude = (real * real + imag * imag).sqrt();
        let frequency_hz = imag.abs() / (2.0 * PI);
        let damping_ratio = if magnitude > 1e-10 {
            -real / magnitude
        } else {
            0.0
        };
        Self {
            real,
            imag,
            frequency_hz,
            damping_ratio,
        }
    }
    /// True when eigenvalue lies in the left half-plane (Re < 0).
    pub fn is_stable(&self) -> bool {
        self.real < 0.0
    }
    /// True when stable but poorly damped (ζ < 5 %).
    pub fn is_poorly_damped(&self) -> bool {
        self.damping_ratio < 0.05 && self.is_stable()
    }
}
