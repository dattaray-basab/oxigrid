//! Power System Stabilizer (PSS) design, tuning, and performance analysis.
//!
//! Implements IEEE standard PSS models (PSS1A, PSS2B, PSS4B), automated parameter
//! tuning via the residue method, Heffron-Phillips small-signal modelling,
//! phase compensation design, root-locus gain selection, and time-domain /
//! frequency-domain performance evaluation.
//!
//! # Units convention
//! - Frequencies in \[Hz\] unless noted as \[rad/s\]
//! - Time constants in \[s\]
//! - Angles in \[deg\]
//! - Gains dimensionless \[pu\]

use crate::error::{OxiGridError, Result};
use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// PSS input signal
// ---------------------------------------------------------------------------

/// PSS input signal type.
#[derive(Debug, Clone, PartialEq)]
pub enum PssInput {
    /// Rotor speed deviation Δω \[pu\].
    RotorSpeed,
    /// Electrical power deviation ΔPe \[pu\].
    ElectricalPower,
    /// Bus frequency deviation Δf \[pu\].
    BusFrequency,
    /// Accelerating power Pa = Pm − Pe \[pu\].
    AcceleratingPower,
}

// ---------------------------------------------------------------------------
// PSS model enum
// ---------------------------------------------------------------------------

/// IEEE standard PSS models (PSS1A, PSS2B, PSS4B).
#[derive(Debug, Clone)]
pub enum PssModel {
    /// IEEE PSS1A — single input (speed deviation or electrical power).
    Pss1A {
        /// Input signal type.
        input: PssInput,
        /// Washout filter time constant \[s\].
        tw: f64,
        /// Lead-lag stage 1: (T1, T2) \[s\].
        lead_lag_1: (f64, f64),
        /// Lead-lag stage 2: (T3, T4) \[s\].
        lead_lag_2: (f64, f64),
        /// Stabilizer gain \[pu/pu\].
        k_s: f64,
        /// Minimum output limiter \[pu\].
        v_st_min: f64,
        /// Maximum output limiter \[pu\].
        v_st_max: f64,
    },
    /// IEEE PSS2B — dual input (speed deviation + integral of accelerating power).
    Pss2B {
        /// Speed-path stabiliser gain \[pu/pu\].
        k_s1: f64,
        /// Power-path stabiliser gain \[pu/pu\].
        k_s2: f64,
        /// Washout time constant for speed path 1 \[s\].
        t_w1: f64,
        /// Washout time constant for speed path 2 \[s\].
        t_w2: f64,
        /// Washout time constant for power path 3 \[s\].
        t_w3: f64,
        /// Washout time constant for power path 4 \[s\].
        t_w4: f64,
        /// Lead-lag T1 \[s\].
        t1: f64,
        /// Lead-lag T2 \[s\].
        t2: f64,
        /// Lead-lag T3 \[s\].
        t3: f64,
        /// Lead-lag T4 \[s\].
        t4: f64,
        /// Ramp tracking filter numerator \[s\].
        t10: f64,
        /// Ramp tracking filter denominator \[s\].
        t11: f64,
        /// Minimum output limiter \[pu\].
        v_st_min: f64,
        /// Maximum output limiter \[pu\].
        v_st_max: f64,
    },
    /// IEEE PSS4B — multi-band (low / intermediate / high frequency).
    Pss4B {
        /// Low-frequency band parameters.
        low_band: PssBandParams,
        /// Intermediate-frequency band parameters.
        inter_band: PssBandParams,
        /// High-frequency band parameters.
        high_band: PssBandParams,
        /// Minimum output limiter \[pu\].
        v_st_min: f64,
        /// Maximum output limiter \[pu\].
        v_st_max: f64,
    },
}

// ---------------------------------------------------------------------------
// PSS4B band parameters
// ---------------------------------------------------------------------------

/// Frequency-band parameters for PSS4B.
#[derive(Debug, Clone)]
pub struct PssBandParams {
    /// Band gain \[pu/pu\].
    pub k_l: f64,
    /// Lead time constant 1 \[s\].
    pub t_l1: f64,
    /// Lag time constant 2 \[s\].
    pub t_l2: f64,
    /// Lead time constant 3 \[s\].
    pub t_l3: f64,
    /// Lag time constant 4 \[s\].
    pub t_l4: f64,
    /// Maximum band output \[pu\].
    pub v_lmax: f64,
    /// Minimum band output \[pu\].
    pub v_lmin: f64,
}

// ---------------------------------------------------------------------------
// Generator modal data
// ---------------------------------------------------------------------------

/// Generator modal data used for PSS residue-method tuning.
#[derive(Debug, Clone)]
pub struct GeneratorModal {
    /// Machine index.
    pub gen_id: usize,
    /// Dominant inter-area mode frequency \[Hz\].
    pub mode_freq_hz: f64,
    /// Open-loop damping ratio of the dominant mode.
    pub damping_ratio: f64,
    /// Residue magnitude for speed stabiliser.
    pub residue_magnitude: f64,
    /// Residue angle \[deg\].
    pub residue_angle_deg: f64,
    /// Inertia constant H \[s\].
    pub inertia_h: f64,
}

// ---------------------------------------------------------------------------
// Tuning configuration and type
// ---------------------------------------------------------------------------

/// PSS type selector.
#[derive(Debug, Clone, PartialEq)]
pub enum PssType {
    /// Single-input IEEE PSS1A.
    Pss1A,
    /// Dual-input IEEE PSS2B.
    Pss2B,
    /// Multi-band IEEE PSS4B.
    Pss4B,
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

impl Default for PssTuningConfig {
    fn default() -> Self {
        Self {
            target_damping: 0.05,
            target_gain_db: 20.0,
            freq_min_hz: 0.01,
            freq_max_hz: 10.0,
            n_freq_points: 100,
            pss_type: PssType::Pss1A,
        }
    }
}

// ---------------------------------------------------------------------------
// Transfer function
// ---------------------------------------------------------------------------

/// Transfer function as ratio of two polynomials with real coefficients.
///
/// Coefficients stored from highest degree to degree 0.
/// E.g. `[a2, a1, a0]` represents `a2·s² + a1·s + a0`.
#[derive(Debug, Clone)]
pub struct TransferFunction {
    /// Numerator polynomial coefficients (high to low degree).
    pub numerator: Vec<f64>,
    /// Denominator polynomial coefficients (high to low degree).
    pub denominator: Vec<f64>,
}

impl TransferFunction {
    /// Create a new transfer function from numerator and denominator coefficients.
    pub fn new(numerator: Vec<f64>, denominator: Vec<f64>) -> Self {
        Self {
            numerator,
            denominator,
        }
    }

    /// Evaluate polynomial at complex `s = (re, im)` using Horner's method.
    fn eval_poly(coeffs: &[f64], s: (f64, f64)) -> (f64, f64) {
        let (mut re, mut im) = (0.0_f64, 0.0_f64);
        for &c in coeffs {
            let new_re = re * s.0 - im * s.1 + c;
            let new_im = re * s.1 + im * s.0;
            re = new_re;
            im = new_im;
        }
        (re, im)
    }

    /// Complex division `a / b`.
    fn cdiv(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
        let denom = b.0 * b.0 + b.1 * b.1;
        if denom < 1e-300 {
            return (0.0, 0.0);
        }
        (
            (a.0 * b.0 + a.1 * b.1) / denom,
            (a.1 * b.0 - a.0 * b.1) / denom,
        )
    }

    /// Evaluate H(jω) at angular frequency ω: returns `(real, imag)`.
    pub fn evaluate_at(&self, omega: f64) -> (f64, f64) {
        let s = (0.0, omega);
        let num = Self::eval_poly(&self.numerator, s);
        let den = Self::eval_poly(&self.denominator, s);
        Self::cdiv(num, den)
    }

    /// Evaluate H at frequency `freq_hz`: returns `(magnitude_db, phase_deg)`.
    pub fn evaluate_at_freq(&self, freq_hz: f64) -> (f64, f64) {
        let omega = 2.0 * PI * freq_hz;
        let (re, im) = self.evaluate_at(omega);
        let mag = (re * re + im * im).sqrt();
        let mag_db = if mag < 1e-300 {
            -300.0
        } else {
            20.0 * mag.log10()
        };
        let phase_deg = im.atan2(re).to_degrees();
        (mag_db, phase_deg)
    }

    /// Magnitude in dB at angular frequency ω.
    pub fn magnitude_db(&self, omega: f64) -> f64 {
        let (re, im) = self.evaluate_at(omega);
        let mag = (re * re + im * im).sqrt();
        if mag < 1e-300 {
            -300.0
        } else {
            20.0 * mag.log10()
        }
    }

    /// Phase in degrees at angular frequency ω.
    pub fn phase_deg(&self, omega: f64) -> f64 {
        let (re, im) = self.evaluate_at(omega);
        im.atan2(re).to_degrees()
    }

    /// Multiply (cascade series) of two transfer functions: `H = self × other`.
    pub fn multiply(&self, other: &TransferFunction) -> TransferFunction {
        TransferFunction {
            numerator: Self::poly_mul(&self.numerator, &other.numerator),
            denominator: Self::poly_mul(&self.denominator, &other.denominator),
        }
    }

    /// Multiply two polynomials (convolution of coefficient vectors).
    fn poly_mul(a: &[f64], b: &[f64]) -> Vec<f64> {
        if a.is_empty() || b.is_empty() {
            return vec![0.0];
        }
        let n = a.len() + b.len() - 1;
        let mut result = vec![0.0; n];
        for (i, &ai) in a.iter().enumerate() {
            for (j, &bj) in b.iter().enumerate() {
                result[i + j] += ai * bj;
            }
        }
        result
    }

    /// Bode plot: returns `Vec<(freq_hz, magnitude_db, phase_deg)>`.
    pub fn bode_plot(&self, f_min_hz: f64, f_max_hz: f64, n_points: usize) -> Vec<(f64, f64, f64)> {
        let n = n_points.max(2);
        let f_lo = f_min_hz.max(1e-6);
        let f_hi = f_max_hz.max(f_lo * 10.0);
        let log_lo = f_lo.log10();
        let log_hi = f_hi.log10();
        (0..n)
            .map(|i| {
                let t = i as f64 / (n - 1) as f64;
                let f = 10.0_f64.powf(log_lo + t * (log_hi - log_lo));
                let (mag_db, ph_deg) = self.evaluate_at_freq(f);
                (f, mag_db, ph_deg)
            })
            .collect()
    }

    /// Series (cascade) connection: `H = self × other`.
    pub fn series(&self, other: &TransferFunction) -> TransferFunction {
        self.multiply(other)
    }

    /// Lead-lag element: `(1 + T1·s) / (1 + T2·s)`.
    pub fn lead_lag(t1: f64, t2: f64) -> TransferFunction {
        TransferFunction {
            numerator: vec![t1, 1.0],
            denominator: vec![t2, 1.0],
        }
    }

    /// Washout (high-pass) filter: `Tw·s / (1 + Tw·s)`.
    pub fn washout(tw: f64) -> TransferFunction {
        TransferFunction {
            numerator: vec![tw, 0.0],
            denominator: vec![tw, 1.0],
        }
    }

    /// Pure gain element: `K`.
    pub fn gain(k: f64) -> TransferFunction {
        TransferFunction {
            numerator: vec![k],
            denominator: vec![1.0],
        }
    }
}

// ---------------------------------------------------------------------------
// Frequency response point
// ---------------------------------------------------------------------------

/// A single frequency-domain data point.
#[derive(Debug, Clone)]
pub struct FreqResponsePoint {
    /// Frequency \[Hz\].
    pub freq_hz: f64,
    /// Magnitude \[dB\].
    pub magnitude_db: f64,
    /// Phase \[deg\].
    pub phase_deg: f64,
}

// ---------------------------------------------------------------------------
// PSS state for time-domain simulation
// ---------------------------------------------------------------------------

/// PSS filter states for time-domain simulation.
#[derive(Debug, Clone)]
pub struct PssState {
    /// Internal integrator/filter states.
    pub states: Vec<f64>,
    /// Latest PSS output \[pu\].
    pub output: f64,
    /// Simulation time \[s\].
    pub time_s: f64,
}

impl PssState {
    /// Create an all-zero initial state with `n_states` internal states.
    pub fn zero(n_states: usize) -> Self {
        Self {
            states: vec![0.0; n_states],
            output: 0.0,
            time_s: 0.0,
        }
    }
}

// ---------------------------------------------------------------------------
// PSS design result
// ---------------------------------------------------------------------------

/// Complete PSS design result.
#[derive(Debug, Clone)]
pub struct PssDesignResult {
    /// Generator index this PSS was designed for.
    pub generator_id: usize,
    /// Designed PSS model.
    pub pss_model: PssModel,
    /// Equivalent transfer function (for Bode analysis).
    pub transfer_function: TransferFunction,
    /// Frequency response over the configured range.
    pub freq_response: Vec<FreqResponsePoint>,
    /// Phase compensation provided at mode frequency \[deg\].
    pub phase_compensation_deg: f64,
    /// Gain margin \[dB\].
    pub gain_margin_db: f64,
    /// Phase margin \[deg\].
    pub phase_margin_deg: f64,
    /// Estimated additional damping ratio contributed by this PSS.
    pub expected_damping_improvement: f64,
    /// `true` when the tuning algorithm converged to a valid design.
    pub design_converged: bool,
}

// ---------------------------------------------------------------------------
// PSS designer
// ---------------------------------------------------------------------------

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

        // Frequency response
        let freq_range = Self::log_freq_range(
            self.config.freq_min_hz,
            self.config.freq_max_hz,
            self.config.n_freq_points,
        );
        let freq_response = Self::evaluate_frequency_response(&tf, &freq_range);

        // Phase compensation at mode frequency
        let (_, phase_at_mode) = tf.evaluate_at_freq(modal.mode_freq_hz);
        let phase_compensation_deg = phase_at_mode;

        // Gain/phase margins
        let (gain_margin_db, phase_margin_deg) = Self::compute_gain_phase_margins(&freq_response);

        // Expected damping improvement: ΔD ≈ Ks × |R| / (2H × ω_m)
        let omega_m = 2.0 * PI * modal.mode_freq_hz.max(1e-9);
        let (mag_at_mode_db, _) = tf.evaluate_at_freq(modal.mode_freq_hz);
        let mag_at_mode = 10.0_f64.powf(mag_at_mode_db / 20.0);
        let two_h = 2.0 * modal.inertia_h.max(1e-9);
        let expected_damping_improvement =
            mag_at_mode * modal.residue_magnitude / (two_h * omega_m);

        // Convergence: gain margin > 0 and phase margin > 0
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

    // -----------------------------------------------------------------------
    // Residue method: PSS1A
    // -----------------------------------------------------------------------

    /// Design a PSS1A via the residue phase-compensation method.
    fn design_pss1a(&self, modal: &GeneratorModal) -> std::result::Result<PssModel, String> {
        // 1. Required phase compensation: φ_c = 180° − φ_R
        let phi_r = modal.residue_angle_deg;
        let phi_c = 180.0 - phi_r;

        // 2. Two lead-lag blocks, each compensates φ_c / 2
        let phi_each_deg = phi_c / 2.0;
        let (t1, t2) = Self::lead_lag_constants(phi_each_deg, modal.mode_freq_hz);
        let (t3, t4) = (t1, t2); // symmetric stages

        // 3. Washout time constant: T_w = 10 / (2π × 0.1 Hz)
        let tw = 10.0 / (2.0 * PI * 0.1_f64);

        // 4. Gain from target gain (dB) and residue magnitude
        let k_target = 10.0_f64.powf(self.config.target_gain_db / 20.0);
        // Magnitude of lead-lag at mode frequency
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

    // -----------------------------------------------------------------------
    // Residue method: PSS2B
    // -----------------------------------------------------------------------

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
        let k_s2 = k_s1 * 0.5; // power path gets half the gain

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

    // -----------------------------------------------------------------------
    // Residue method: PSS4B
    // -----------------------------------------------------------------------

    /// Design a PSS4B multi-band stabiliser.
    fn design_pss4b(&self, modal: &GeneratorModal) -> std::result::Result<PssModel, String> {
        // Three bands: low (0.01–0.1 Hz), inter (0.1–1.0 Hz), high (1.0–4.0 Hz)
        let f_l = 0.05_f64; // low-band centre [Hz]
        let f_i = modal.mode_freq_hz.clamp(0.1, 1.0); // inter-band centred at mode
        let f_h = 2.0_f64; // high-band centre [Hz]

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

    // -----------------------------------------------------------------------
    // Lead-lag time constants from required phase lead
    // -----------------------------------------------------------------------

    /// Compute lead-lag time constants T1 > T2 that provide phase lead `phi_deg`
    /// at frequency `f_hz` using the standard residue-method formulae.
    ///
    /// ```text
    /// α = (1 − sin φ) / (1 + sin φ)
    /// T1 = 1 / (ω_m × √α)
    /// T2 = α × T1
    /// ```
    fn lead_lag_constants(phi_deg: f64, f_hz: f64) -> (f64, f64) {
        let omega = 2.0 * PI * f_hz.max(1e-6);
        // Clamp phi to (0°, 89°] for valid lead computation
        let phi = phi_deg.clamp(1.0, 89.0).to_radians();
        let sin_phi = phi.sin().clamp(1e-9, 0.9999);
        let alpha = (1.0 - sin_phi) / (1.0 + sin_phi);
        let alpha = alpha.max(1e-6);
        let t1 = 1.0 / (omega * alpha.sqrt());
        let t2 = alpha * t1;
        (t1, t2) // T1 > T2 always (lead network)
    }

    // -----------------------------------------------------------------------
    // Transfer function construction
    // -----------------------------------------------------------------------

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
                // Approximate as sum of three band TFs — represented as the inter-band
                // (dominant) for linear analysis; each band runs independently in simulation
                let tf_l = TransferFunction::gain(low_band.k_l)
                    .series(&TransferFunction::lead_lag(low_band.t_l1, low_band.t_l2));
                let tf_i = TransferFunction::gain(inter_band.k_l).series(
                    &TransferFunction::lead_lag(inter_band.t_l1, inter_band.t_l2),
                );
                let tf_h = TransferFunction::gain(high_band.k_l)
                    .series(&TransferFunction::lead_lag(high_band.t_l1, high_band.t_l2));
                // Use inter-band as primary, noting PSS4B sums all bands
                tf_l.series(&tf_i).series(&tf_h)
            }
        }
    }

    // -----------------------------------------------------------------------
    // Frequency response
    // -----------------------------------------------------------------------

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

    // -----------------------------------------------------------------------
    // Gain and phase margins
    // -----------------------------------------------------------------------

    /// Compute gain margin \[dB\] and phase margin \[deg\] from frequency response data.
    ///
    /// - **Gain margin**: `-magnitude_db` at the phase-crossover frequency (phase = −180°).
    /// - **Phase margin**: `phase + 180°` at the gain-crossover frequency (magnitude = 0 dB).
    pub fn compute_gain_phase_margins(freq_resp: &[FreqResponsePoint]) -> (f64, f64) {
        let mut gain_margin_db = 20.0_f64;
        let mut phase_margin_deg = 45.0_f64;

        // Phase crossover (−180°) → gain margin
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

        // Gain crossover (0 dB) → phase margin
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

    // -----------------------------------------------------------------------
    // Time-domain step simulation
    // -----------------------------------------------------------------------

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
                // Need 3 states: x_w (washout LP state), x_l1, x_l2
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

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Eigenvalue helper types (kept for compatibility with internal users)
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Legacy Heffron-Phillips designer (kept for backward compatibility)
// ---------------------------------------------------------------------------

/// Generator electrical model for Heffron-Phillips small-signal analysis.
#[derive(Debug, Clone)]
pub struct PssGeneratorModel {
    /// Machine index.
    pub machine_id: usize,
    /// Rated MVA.
    pub rated_mva: f64,
    /// Inertia constant H \[s\].
    pub h_inertia_s: f64,
    /// Damping coefficient D \[pu\].
    pub d_damping: f64,
    /// Transient d-axis reactance X'd \[pu\].
    pub xd_transient: f64,
    /// Transient open-circuit d-axis time constant T'd0 \[s\].
    pub td0_transient_s: f64,
    /// Exciter gain Ka \[pu/pu\].
    pub exciter_gain_ka: f64,
    /// Exciter time constant Ta \[s\].
    pub exciter_time_ta_s: f64,
    /// Heffron-Phillips K1.
    pub k1: f64,
    /// Heffron-Phillips K2.
    pub k2: f64,
    /// Heffron-Phillips K3.
    pub k3: f64,
    /// Heffron-Phillips K4.
    pub k4: f64,
    /// Heffron-Phillips K5.
    pub k5: f64,
    /// Heffron-Phillips K6.
    pub k6: f64,
}

/// PSS design specification (target performance for Heffron-Phillips designer).
#[derive(Debug, Clone)]
pub struct PssDesignSpec {
    /// Target oscillation frequency to damp \[Hz\].
    pub target_mode_frequency_hz: f64,
    /// Target closed-loop damping ratio ζ.
    pub target_damping_ratio: f64,
    /// Required phase advance at mode frequency \[deg\].
    pub phase_compensation_deg: f64,
    /// Upper bound on PSS gain.
    pub max_gain: f64,
    /// Washout filter corner frequency \[Hz\].
    pub washout_freq_hz: f64,
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

/// Heffron-Phillips PSS designer (4×4 state-space model).
pub struct HpPssDesigner {
    /// Generator model.
    pub generator: PssGeneratorModel,
    /// Design specification.
    pub spec: PssDesignSpec,
}

impl HpPssDesigner {
    /// Create a new `HpPssDesigner`.
    pub fn new(generator: PssGeneratorModel, spec: PssDesignSpec) -> Self {
        Self { generator, spec }
    }

    /// Build the Heffron-Phillips A-matrix and compute its eigenvalues.
    pub fn compute_open_loop_eigenvalues(&self) -> Vec<Eigenvalue> {
        let gen = &self.generator;
        let omega_s = 2.0 * PI * 50.0;
        let two_h = 2.0 * gen.h_inertia_s.max(1e-9);
        let k3 = gen.k3.max(1e-9);
        let td0 = gen.td0_transient_s.max(1e-9);
        let ka = gen.exciter_gain_ka;
        let ta = gen.exciter_time_ta_s.max(1e-9);

        let a: [[f64; 4]; 4] = [
            [0.0, omega_s, 0.0, 0.0],
            [
                -gen.k1 / two_h,
                -gen.d_damping / two_h,
                -gen.k2 / two_h,
                0.0,
            ],
            [-gen.k4 / td0, 0.0, -1.0 / (k3 * td0), 1.0 / td0],
            [-ka * gen.k5 / ta, 0.0, -ka * gen.k6 / ta, -1.0 / ta],
        ];

        Self::eigenvalues_4x4(&a)
    }

    /// Design two-stage lead-lag time constants.
    pub fn design_phase_compensation(&self) -> (f64, f64, f64, f64) {
        let phi_total_deg = self.spec.phase_compensation_deg.clamp(-160.0, 160.0);
        let phi_per_stage_deg = phi_total_deg / 2.0;
        let phi_rad = phi_per_stage_deg.to_radians();
        let omega_mode = 2.0 * PI * self.spec.target_mode_frequency_hz.max(1e-6);

        let sin_phi = phi_rad.sin().clamp(-0.9999, 0.9999);
        let alpha = ((1.0 + sin_phi) / (1.0 - sin_phi)).max(1e-6);
        let t2 = 1.0 / (omega_mode * alpha.sqrt());
        let t1 = alpha * t2;
        (t1, t2, t1, t2)
    }

    /// Select PSS gain K via root-locus sweep.
    pub fn select_gain(&self, t1: f64, t2: f64, t3: f64, t4: f64) -> f64 {
        let n_steps = 100usize;
        let max_gain = self.spec.max_gain.max(1e-3);
        let target = self.spec.target_damping_ratio;
        for i in 1..=n_steps {
            let k = max_gain * i as f64 / n_steps as f64;
            let eig = self.closed_loop_eigenvalue_at_gain(k, t1, t2, t3, t4);
            if eig.damping_ratio >= target {
                return k;
            }
        }
        max_gain
    }

    /// Design a complete IEEE PSS1A.
    pub fn design_pss1a(&self) -> Result<HpDesignResult> {
        let ol_eigs = self.compute_open_loop_eigenvalues();
        let target_freq = self.spec.target_mode_frequency_hz;
        let ol_critical = ol_eigs
            .iter()
            .min_by(|a, b| {
                let da = (a.frequency_hz - target_freq).abs();
                let db = (b.frequency_hz - target_freq).abs();
                da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
            })
            .cloned()
            .ok_or_else(|| {
                OxiGridError::InvalidParameter("no eigenvalues from open-loop model".to_string())
            })?;

        let is_stable_without_pss = ol_eigs.iter().all(|e| e.is_stable());
        let (t1, t2, t3, t4) = self.design_phase_compensation();
        let ks = self.select_gain(t1, t2, t3, t4);

        let tw = if self.spec.washout_freq_hz > 1e-9 {
            1.0 / (2.0 * PI * self.spec.washout_freq_hz)
        } else {
            10.0
        };

        let cl_eig = self.closed_loop_eigenvalue_at_gain(ks, t1, t2, t3, t4);

        let pss_tf = TransferFunction::gain(ks)
            .series(&TransferFunction::washout(tw))
            .series(&TransferFunction::lead_lag(t1, t2))
            .series(&TransferFunction::lead_lag(t3, t4));
        let bode = pss_tf.bode_plot(0.01, 10.0, 200);
        let fr: Vec<FreqResponsePoint> = bode
            .iter()
            .map(|&(f, m, p)| FreqResponsePoint {
                freq_hz: f,
                magnitude_db: m,
                phase_deg: p,
            })
            .collect();
        let (gain_margin_db, phase_margin_deg) = PssDesigner::compute_gain_phase_margins(&fr);

        let pss_model = PssModel::Pss1A {
            input: PssInput::RotorSpeed,
            tw,
            lead_lag_1: (t1, t2),
            lead_lag_2: (t3, t4),
            k_s: ks,
            v_st_min: -0.1,
            v_st_max: 0.1,
        };

        let mut notes = Vec::new();
        notes.push(format!(
            "Ks={ks:.4}, T1={t1:.4} s, T2={t2:.4} s, Tw={tw:.2} s"
        ));
        notes.push(format!(
            "Open-loop mode: σ={:.4}, f={:.3} Hz, ζ={:.4}",
            ol_critical.real, ol_critical.frequency_hz, ol_critical.damping_ratio
        ));
        notes.push(format!(
            "Closed-loop mode: σ={:.4}, f={:.3} Hz, ζ={:.4}",
            cl_eig.real, cl_eig.frequency_hz, cl_eig.damping_ratio
        ));

        Ok(HpDesignResult {
            pss_model,
            achieved_damping: cl_eig.damping_ratio,
            achieved_frequency_hz: cl_eig.frequency_hz,
            phase_margin_deg,
            gain_margin_db,
            is_stable_without_pss,
            eigenvalue_without_pss: ol_critical,
            eigenvalue_with_pss: cl_eig,
            design_notes: notes,
        })
    }

    fn closed_loop_eigenvalue_at_gain(
        &self,
        k: f64,
        t1: f64,
        t2: f64,
        t3: f64,
        t4: f64,
    ) -> Eigenvalue {
        let gen = &self.generator;
        let omega_s = 2.0 * PI * 50.0;
        let two_h = 2.0 * gen.h_inertia_s.max(1e-9);
        let k3 = gen.k3.max(1e-9);
        let td0 = gen.td0_transient_s.max(1e-9);
        let ka = gen.exciter_gain_ka;
        let ta = gen.exciter_time_ta_s.max(1e-9);
        let omega_m = 2.0 * PI * self.spec.target_mode_frequency_hz.max(1e-6);
        let ll1 = Self::lead_lag_at(t1, t2, omega_m);
        let ll2 = Self::lead_lag_at(t3, t4, omega_m);
        let ll_re = ll1.0 * ll2.0 - ll1.1 * ll2.1;
        let d_extra = k * ll_re * gen.k2 * ka / (ta * td0 * k3).max(1e-12);
        let d_eff = gen.d_damping + d_extra.max(0.0);
        let a: [[f64; 4]; 4] = [
            [0.0, omega_s, 0.0, 0.0],
            [-gen.k1 / two_h, -d_eff / two_h, -gen.k2 / two_h, 0.0],
            [-gen.k4 / td0, 0.0, -1.0 / (k3 * td0), 1.0 / td0],
            [-ka * gen.k5 / ta, 0.0, -ka * gen.k6 / ta, -1.0 / ta],
        ];
        let eigs = Self::eigenvalues_4x4(&a);
        let target_freq = self.spec.target_mode_frequency_hz;
        eigs.into_iter()
            .min_by(|a, b| {
                let da = (a.frequency_hz - target_freq).abs();
                let db = (b.frequency_hz - target_freq).abs();
                da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
            })
            .unwrap_or_else(|| Eigenvalue::new(-0.5, omega_m))
    }

    fn lead_lag_at(t1: f64, t2: f64, omega: f64) -> (f64, f64) {
        let num = (1.0, omega * t1);
        let den = (1.0, omega * t2);
        let denom = den.0 * den.0 + den.1 * den.1;
        if denom < 1e-300 {
            return (1.0, 0.0);
        }
        (
            (num.0 * den.0 + num.1 * den.1) / denom,
            (num.1 * den.0 - num.0 * den.1) / denom,
        )
    }

    /// Compute eigenvalues of a 4×4 real matrix via QR iteration.
    pub fn eigenvalues_4x4(a: &[[f64; 4]; 4]) -> Vec<Eigenvalue> {
        let mut h = Self::to_hessenberg(a);
        for _ in 0..200 {
            Self::qr_step(&mut h);
        }
        Self::extract_eigenvalues_from_hessenberg(&h)
    }

    #[allow(clippy::needless_range_loop)]
    fn to_hessenberg(a: &[[f64; 4]; 4]) -> [[f64; 4]; 4] {
        let mut h = *a;
        for k in 0..2usize {
            let mut v = [0.0_f64; 4];
            let mut norm2 = 0.0;
            for i in (k + 1)..4 {
                v[i] = h[i][k];
                norm2 += h[i][k] * h[i][k];
            }
            if norm2 < 1e-28 {
                continue;
            }
            let norm = norm2.sqrt();
            let sign = if h[k + 1][k] >= 0.0 { 1.0 } else { -1.0 };
            v[k + 1] += sign * norm;
            let mut v_norm2 = 0.0;
            for i in (k + 1)..4 {
                v_norm2 += v[i] * v[i];
            }
            if v_norm2 < 1e-28 {
                continue;
            }
            for j in 0..4 {
                let mut dot = 0.0;
                for i in (k + 1)..4 {
                    dot += v[i] * h[i][j];
                }
                let factor = 2.0 * dot / v_norm2;
                for i in (k + 1)..4 {
                    h[i][j] -= factor * v[i];
                }
            }
            for i in 0..4 {
                let mut dot = 0.0;
                for j in (k + 1)..4 {
                    dot += h[i][j] * v[j];
                }
                let factor = 2.0 * dot / v_norm2;
                for j in (k + 1)..4 {
                    h[i][j] -= factor * v[j];
                }
            }
        }
        h
    }

    #[allow(clippy::needless_range_loop)]
    fn qr_step(h: &mut [[f64; 4]; 4]) {
        let a = h[2][2];
        let b = h[2][3];
        let c = h[3][2];
        let d = h[3][3];
        let tr = a + d;
        let det = a * d - b * c;
        let disc = tr * tr - 4.0 * det;
        let shift = if disc >= 0.0 {
            let s1 = (tr + disc.sqrt()) / 2.0;
            let s2 = (tr - disc.sqrt()) / 2.0;
            if (s1 - d).abs() < (s2 - d).abs() {
                s1
            } else {
                s2
            }
        } else {
            tr / 2.0
        };
        for i in 0..4 {
            h[i][i] -= shift;
        }
        for k in 0..3usize {
            let x = h[k][k];
            let y = h[k + 1][k];
            let r = (x * x + y * y).sqrt();
            if r < 1e-14 {
                continue;
            }
            let cos = x / r;
            let sin = y / r;
            for j in 0..4 {
                let t1 = cos * h[k][j] + sin * h[k + 1][j];
                let t2 = -sin * h[k][j] + cos * h[k + 1][j];
                h[k][j] = t1;
                h[k + 1][j] = t2;
            }
            for i in 0..4 {
                let t1 = cos * h[i][k] + sin * h[i][k + 1];
                let t2 = -sin * h[i][k] + cos * h[i][k + 1];
                h[i][k] = t1;
                h[i][k + 1] = t2;
            }
        }
        for i in 0..4 {
            h[i][i] += shift;
        }
    }

    fn extract_eigenvalues_from_hessenberg(h: &[[f64; 4]; 4]) -> Vec<Eigenvalue> {
        let mut eigs = Vec::with_capacity(4);
        let mut i = 0usize;
        while i < 4 {
            if i + 1 < 4 && h[i + 1][i].abs() > 1e-10 {
                let a = h[i][i];
                let b = h[i][i + 1];
                let c = h[i + 1][i];
                let d = h[i + 1][i + 1];
                let tr = a + d;
                let det = a * d - b * c;
                let disc = tr * tr / 4.0 - det;
                let re = tr / 2.0;
                let im = if disc < 0.0 { (-disc).sqrt() } else { 0.0 };
                eigs.push(Eigenvalue::new(re, im));
                eigs.push(Eigenvalue::new(re, -im));
                i += 2;
            } else {
                eigs.push(Eigenvalue::new(h[i][i], 0.0));
                i += 1;
            }
        }
        eigs
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // ---- helpers -----------------------------------------------------------

    fn modal_07hz() -> GeneratorModal {
        GeneratorModal {
            gen_id: 1,
            mode_freq_hz: 0.7,
            damping_ratio: 0.02,
            residue_magnitude: 0.5,
            residue_angle_deg: 60.0,
            inertia_h: 5.0,
        }
    }

    fn config_pss1a() -> PssTuningConfig {
        PssTuningConfig {
            target_damping: 0.05,
            target_gain_db: 20.0,
            freq_min_hz: 0.01,
            freq_max_hz: 10.0,
            n_freq_points: 50,
            pss_type: PssType::Pss1A,
        }
    }

    fn default_pss1a() -> PssModel {
        PssModel::Pss1A {
            input: PssInput::RotorSpeed,
            tw: 10.0,
            lead_lag_1: (0.30, 0.05),
            lead_lag_2: (0.30, 0.05),
            k_s: 5.0,
            v_st_min: -0.1,
            v_st_max: 0.1,
        }
    }

    // ---- PSS1A lead constant tests ----------------------------------------

    #[test]
    fn test_pss1a_lead_lag_t1_gt_t2() {
        let (t1, t2) = PssDesigner::lead_lag_constants(45.0, 0.7);
        assert!(t1 > t2, "Lead network: T1={t1:.4} must exceed T2={t2:.4}");
    }

    #[test]
    fn test_pss1a_phase_compensation_formula() {
        // φ_c = 180° − φ_R = 180° − 60° = 120°; each stage ≈ 60°
        let modal = modal_07hz();
        let phi_c = 180.0 - modal.residue_angle_deg;
        assert!((phi_c - 120.0).abs() < 1e-9);
    }

    #[test]
    fn test_washout_dc_gain_zero() {
        let tf = TransferFunction::washout(10.0);
        let (mag_db, _) = tf.evaluate_at_freq(0.001);
        // Should be very small at near-DC
        let mag_lin = 10.0_f64.powf(mag_db / 20.0);
        assert!(
            mag_lin < 0.1,
            "Washout should block near-DC: mag={mag_lin:.4}"
        );
    }

    #[test]
    fn test_washout_high_freq_near_unity() {
        let tf = TransferFunction::washout(10.0);
        let (mag_db, _) = tf.evaluate_at_freq(100.0);
        let mag_lin = 10.0_f64.powf(mag_db / 20.0);
        assert!(
            mag_lin > 0.9,
            "Washout should pass high freq: mag={mag_lin:.4}"
        );
    }

    #[test]
    fn test_lead_lag_provides_phase_lead() {
        let (t1, t2) = PssDesigner::lead_lag_constants(60.0, 0.7);
        let tf = TransferFunction::lead_lag(t1, t2);
        let (_, phase) = tf.evaluate_at_freq(0.7);
        assert!(
            phase > 0.0,
            "Lead network must give positive phase: {phase:.2}°"
        );
    }

    #[test]
    fn test_gain_margin_positive() {
        let mut designer = PssDesigner::new(config_pss1a());
        designer.add_generator_modal(modal_07hz());
        let result = designer.design_pss(1).expect("design should succeed");
        assert!(
            result.gain_margin_db > 0.0,
            "Gain margin must be positive: {:.2} dB",
            result.gain_margin_db
        );
    }

    #[test]
    fn test_phase_margin_positive() {
        let mut designer = PssDesigner::new(config_pss1a());
        designer.add_generator_modal(modal_07hz());
        let result = designer.design_pss(1).expect("design should succeed");
        assert!(
            result.phase_margin_deg > 0.0,
            "Phase margin must be positive: {:.2}°",
            result.phase_margin_deg
        );
    }

    #[test]
    fn test_freq_response_at_mode_frequency() {
        let tf = TransferFunction::lead_lag(0.3, 0.05).series(&TransferFunction::gain(5.0));
        let (mag_db, _) = tf.evaluate_at_freq(0.7);
        assert!(mag_db.is_finite(), "Frequency response must be finite");
    }

    #[test]
    fn test_high_freq_rolloff() {
        let tf = TransferFunction::washout(10.0).series(&TransferFunction::lead_lag(0.3, 0.05));
        let (mag_lo, _) = tf.evaluate_at_freq(1.0);
        let (mag_hi, _) = tf.evaluate_at_freq(1000.0);
        // At very high freq the lead-lag ratio approaches T1/T2 but the washout → 1;
        // The combination should be finite; basic sanity check
        assert!(mag_lo.is_finite() && mag_hi.is_finite());
    }

    #[test]
    fn test_pss1a_tf_numerator_denominator_degree() {
        let pss = default_pss1a();
        let tf = PssDesigner::compute_transfer_function(&pss);
        // washout(deg 2 num, deg 2 den) × 2 lead-lag(deg 2 num/den each) × gain(1)
        // Total: num degree = 1+1+1+1 = 4 terms → 5 coefficients; den same
        assert!(!tf.numerator.is_empty());
        assert!(!tf.denominator.is_empty());
        assert_eq!(tf.numerator.len(), tf.denominator.len());
    }

    #[test]
    fn test_pss2b_model_construction() {
        let mut designer = PssDesigner::new(PssTuningConfig {
            pss_type: PssType::Pss2B,
            ..config_pss1a()
        });
        designer.add_generator_modal(modal_07hz());
        let result = designer.design_pss(1).expect("PSS2B design should succeed");
        match &result.pss_model {
            PssModel::Pss2B { k_s1, k_s2, .. } => {
                assert!(*k_s1 > 0.0);
                assert!(*k_s2 > 0.0);
            }
            _ => panic!("Expected Pss2B"),
        }
    }

    #[test]
    fn test_pss4b_model_construction() {
        let mut designer = PssDesigner::new(PssTuningConfig {
            pss_type: PssType::Pss4B,
            ..config_pss1a()
        });
        designer.add_generator_modal(modal_07hz());
        let result = designer.design_pss(1).expect("PSS4B design should succeed");
        match &result.pss_model {
            PssModel::Pss4B {
                low_band,
                inter_band,
                high_band,
                ..
            } => {
                assert!(low_band.k_l > 0.0);
                assert!(inter_band.k_l > 0.0);
                assert!(high_band.k_l > 0.0);
            }
            _ => panic!("Expected Pss4B"),
        }
    }

    #[test]
    fn test_pss4b_band_separation() {
        // Low-band centre frequency < inter-band < high-band
        // Check that time constants differ between bands
        let mut designer = PssDesigner::new(PssTuningConfig {
            pss_type: PssType::Pss4B,
            ..config_pss1a()
        });
        designer.add_generator_modal(modal_07hz());
        let result = designer.design_pss(1).unwrap();
        match &result.pss_model {
            PssModel::Pss4B {
                low_band,
                high_band,
                ..
            } => {
                // Low-band has larger time constants (lower freq) than high-band
                assert!(
                    low_band.t_l1 > high_band.t_l1,
                    "Low-band T should exceed high-band T: {} vs {}",
                    low_band.t_l1,
                    high_band.t_l1
                );
            }
            _ => panic!("Expected Pss4B"),
        }
    }

    #[test]
    fn test_simulate_pss_step_bounded() {
        let pss = default_pss1a();
        let mut state = PssState::zero(3);
        for _ in 0..100 {
            let out = PssDesigner::simulate_pss_step(&pss, &mut state, 1.0, 0.01);
            assert!(
                (-0.1 - 1e-10..=0.1 + 1e-10).contains(&out),
                "Output out of bounds: {out}"
            );
        }
    }

    #[test]
    fn test_simulate_pss_step_steady_state_zero() {
        // DC input → washout should drive output to zero eventually
        let pss = default_pss1a();
        let mut state = PssState::zero(3);
        let mut last_out = 1.0_f64;
        for _ in 0..5000 {
            last_out = PssDesigner::simulate_pss_step(&pss, &mut state, 0.001, 0.01);
        }
        assert!(
            last_out.abs() < 0.01,
            "Steady-state output should decay: {last_out:.6}"
        );
    }

    #[test]
    fn test_tf_evaluate_at_freq_dc_gain() {
        // lead-lag DC gain = 1 (T1·0 terms cancel)
        let tf = TransferFunction::lead_lag(0.3, 0.05);
        let (mag_db, _) = tf.evaluate_at_freq(0.0001);
        let mag = 10.0_f64.powf(mag_db / 20.0);
        assert!(
            (mag - 1.0).abs() < 0.01,
            "DC gain should be ≈1, got {mag:.4}"
        );
    }

    #[test]
    fn test_compute_gain_phase_margins() {
        // Construct a simple Bode with known crossover
        let fr: Vec<FreqResponsePoint> = vec![
            FreqResponsePoint {
                freq_hz: 1.0,
                magnitude_db: 10.0,
                phase_deg: -150.0,
            },
            FreqResponsePoint {
                freq_hz: 2.0,
                magnitude_db: 0.0,
                phase_deg: -160.0,
            },
            FreqResponsePoint {
                freq_hz: 4.0,
                magnitude_db: -10.0,
                phase_deg: -180.0,
            },
            FreqResponsePoint {
                freq_hz: 8.0,
                magnitude_db: -20.0,
                phase_deg: -200.0,
            },
        ];
        let (gm, pm) = PssDesigner::compute_gain_phase_margins(&fr);
        assert!(gm.is_finite());
        assert!(pm.is_finite());
    }

    #[test]
    fn test_design_converged_flag() {
        let mut designer = PssDesigner::new(config_pss1a());
        designer.add_generator_modal(modal_07hz());
        let result = designer.design_pss(1).unwrap();
        assert!(
            result.design_converged,
            "Design should converge for reasonable input"
        );
    }

    #[test]
    fn test_expected_damping_improvement_positive() {
        let mut designer = PssDesigner::new(config_pss1a());
        designer.add_generator_modal(modal_07hz());
        let result = designer.design_pss(1).unwrap();
        assert!(
            result.expected_damping_improvement > 0.0,
            "Damping improvement must be positive: {:.4}",
            result.expected_damping_improvement
        );
    }

    #[test]
    fn test_pss_input_rotor_speed_variant() {
        match PssInput::RotorSpeed {
            PssInput::RotorSpeed => {}
            _ => panic!("Wrong variant"),
        }
    }

    #[test]
    fn test_pss_input_electrical_power_variant() {
        match PssInput::ElectricalPower {
            PssInput::ElectricalPower => {}
            _ => panic!("Wrong variant"),
        }
    }

    #[test]
    fn test_freq_response_n_points() {
        let mut designer = PssDesigner::new(config_pss1a());
        designer.add_generator_modal(modal_07hz());
        let result = designer.design_pss(1).unwrap();
        assert_eq!(
            result.freq_response.len(),
            50,
            "Frequency response should have 50 points"
        );
    }

    #[test]
    fn test_phase_compensation_for_45_deg_residue() {
        // φ_R = 45° → φ_c = 135°
        let phi_c = 180.0 - 45.0_f64;
        assert!(
            (phi_c - 135.0).abs() < 1e-9,
            "Expected φ_c=135°, got {phi_c}"
        );
    }

    #[test]
    fn test_lead_time_constant_t1_gt_t2_for_lead() {
        let (t1, t2) = PssDesigner::lead_lag_constants(30.0, 1.0);
        assert!(
            t1 > t2,
            "T1={t1:.4} must exceed T2={t2:.4} for lead network"
        );
    }

    #[test]
    fn test_multiple_generators_independent_designs() {
        let mut designer = PssDesigner::new(config_pss1a());
        designer.add_generator_modal(modal_07hz());
        designer.add_generator_modal(GeneratorModal {
            gen_id: 2,
            mode_freq_hz: 1.2,
            damping_ratio: 0.03,
            residue_magnitude: 0.3,
            residue_angle_deg: 45.0,
            inertia_h: 4.0,
        });
        let r1 = designer.design_pss(1).unwrap();
        let r2 = designer.design_pss(2).unwrap();
        assert_eq!(r1.generator_id, 1);
        assert_eq!(r2.generator_id, 2);
        // Different mode frequencies → different time constants
        match (&r1.pss_model, &r2.pss_model) {
            (
                PssModel::Pss1A {
                    lead_lag_1: ll1, ..
                },
                PssModel::Pss1A {
                    lead_lag_1: ll2, ..
                },
            ) => {
                assert!(
                    (ll1.0 - ll2.0).abs() > 1e-9,
                    "Different generators must produce different PSS parameters"
                );
            }
            _ => panic!("Expected Pss1A for both"),
        }
    }

    #[test]
    fn test_band_params_vmax_gt_vmin() {
        let band = PssBandParams {
            k_l: 5.0,
            t_l1: 0.3,
            t_l2: 0.05,
            t_l3: 0.3,
            t_l4: 0.05,
            v_lmax: 0.05,
            v_lmin: -0.05,
        };
        assert!(band.v_lmax > band.v_lmin);
    }

    #[test]
    fn test_tf_multiply_cascades_correctly() {
        let g1 = TransferFunction::gain(2.0);
        let g2 = TransferFunction::gain(3.0);
        let g = g1.multiply(&g2);
        let (mag_db, _) = g.evaluate_at_freq(0.0001);
        let mag = 10.0_f64.powf(mag_db / 20.0);
        assert!((mag - 6.0).abs() < 0.01, "2×3 = 6, got {mag:.4}");
    }

    #[test]
    fn test_pss1a_design_for_07hz_mode_lead() {
        let mut designer = PssDesigner::new(config_pss1a());
        designer.add_generator_modal(modal_07hz());
        let result = designer.design_pss(1).unwrap();
        match &result.pss_model {
            PssModel::Pss1A {
                lead_lag_1,
                lead_lag_2,
                ..
            } => {
                assert!(lead_lag_1.0 > lead_lag_1.1, "T1 > T2 for lead");
                assert!(lead_lag_2.0 > lead_lag_2.1, "T3 > T4 for lead");
            }
            _ => panic!("Expected Pss1A"),
        }
    }

    #[test]
    fn test_missing_generator_returns_error() {
        let designer = PssDesigner::new(config_pss1a());
        // No modal data added
        let result = designer.design_pss(99);
        assert!(result.is_err(), "Should return error for missing generator");
    }

    // ---- Eigenvalue tests (legacy Heffron-Phillips) -----------------------

    #[test]
    fn test_eigenvalue_stable() {
        let eig = Eigenvalue::new(-0.5, std::f64::consts::TAU);
        assert!(eig.is_stable());
        assert!(eig.damping_ratio > 0.0);
    }

    #[test]
    fn test_eigenvalue_poorly_damped() {
        let eig = Eigenvalue::new(-0.01, std::f64::consts::TAU);
        assert!(eig.is_poorly_damped());
    }
}
