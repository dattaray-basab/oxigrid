//! Auto-generated module
//!
//! 🤖 Generated with [SplitRS](https://github.com/cool-japan/splitrs)

use crate::error::{OxiGridError, Result};
use std::f64::consts::PI;

use super::types_3::{Eigenvalue, HpDesignResult, PssDesigner};

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

#[cfg(test)]
mod tests {
    use super::*;

    // ---------------------------------------------------------------------------
    // PssInput / PssType variant identity
    // ---------------------------------------------------------------------------

    #[test]
    fn pss_input_variants_are_distinguishable() {
        assert_ne!(PssInput::RotorSpeed, PssInput::ElectricalPower);
        assert_ne!(PssInput::BusFrequency, PssInput::AcceleratingPower);
        assert_eq!(PssInput::RotorSpeed, PssInput::RotorSpeed);
    }

    #[test]
    fn pss_type_variants_are_distinguishable() {
        assert_ne!(PssType::Pss1A, PssType::Pss2B);
        assert_ne!(PssType::Pss2B, PssType::Pss4B);
        assert_eq!(PssType::Pss4B, PssType::Pss4B);
    }

    // ---------------------------------------------------------------------------
    // TransferFunction primitives
    // ---------------------------------------------------------------------------

    #[test]
    fn transfer_function_gain_element() {
        let tf = TransferFunction::gain(3.0);
        let (re, im) = tf.evaluate_at(1.0);
        assert!((re - 3.0).abs() < 1e-9, "gain re should be 3.0, got {re}");
        assert!(im.abs() < 1e-9, "gain im should be 0.0, got {im}");
    }

    #[test]
    fn lead_lag_dc_gain_is_one() {
        // At DC (ω → 0) the lead-lag (1+T1·s)/(1+T2·s) → 1.
        let tf = TransferFunction::lead_lag(0.2, 0.1);
        let (re, im) = tf.evaluate_at(1e-6);
        assert!(
            (re - 1.0).abs() < 1e-3,
            "lead-lag DC re should ≈ 1.0, got {re}"
        );
        assert!(im.abs() < 1e-3, "lead-lag DC im should ≈ 0.0, got {im}");
    }

    #[test]
    fn washout_dc_rejection() {
        // Washout Tw·s/(1+Tw·s) → 0 as ω → 0.
        let tf = TransferFunction::washout(10.0);
        let (re, im) = tf.evaluate_at(1e-6);
        assert!(re.abs() < 0.01, "washout DC re should ≈ 0, got {re}");
        assert!(im.abs() < 0.01, "washout DC im should ≈ 0, got {im}");
    }

    #[test]
    fn washout_high_freq_passthrough() {
        // Washout → 1 as ω → ∞.
        let tf = TransferFunction::washout(10.0);
        let (re, im) = tf.evaluate_at(1e6);
        let mag = (re * re + im * im).sqrt();
        assert!(
            (mag - 1.0).abs() < 0.01,
            "washout HF magnitude should ≈ 1.0, got {mag}"
        );
    }

    // ---------------------------------------------------------------------------
    // PssState initialisation
    // ---------------------------------------------------------------------------

    #[test]
    fn pss_state_zero_initialises_correctly() {
        let st = PssState::zero(3);
        assert_eq!(st.states.len(), 3, "states length should be 3");
        assert!(
            st.states.iter().all(|&v| v == 0.0),
            "all states should be 0.0"
        );
        assert_eq!(st.output, 0.0, "output should be 0.0");
        assert_eq!(st.time_s, 0.0, "time_s should be 0.0");
    }

    // ---------------------------------------------------------------------------
    // HpPssDesigner: phase-compensation lead-lag ordering
    // ---------------------------------------------------------------------------

    #[test]
    fn design_phase_compensation_returns_t1_gt_t2_for_positive_phase() {
        let gen = PssGeneratorModel {
            machine_id: 0,
            rated_mva: 100.0,
            h_inertia_s: 5.0,
            d_damping: 1.0,
            xd_transient: 0.2,
            td0_transient_s: 5.0,
            exciter_gain_ka: 200.0,
            exciter_time_ta_s: 0.05,
            k1: 0.9,
            k2: 0.8,
            k3: 0.3,
            k4: 1.0,
            k5: 0.1,
            k6: 0.4,
        };
        let spec = PssDesignSpec {
            target_mode_frequency_hz: 1.0,
            target_damping_ratio: 0.1,
            phase_compensation_deg: 60.0,
            max_gain: 50.0,
            washout_freq_hz: 0.1,
        };
        let designer = HpPssDesigner::new(gen, spec);
        let (t1, t2, t3, t4) = designer.design_phase_compensation();
        assert!(
            t1 > t2,
            "T1 should be > T2 for positive phase lead (T1={t1}, T2={t2})"
        );
        assert!(
            t3 > t4,
            "T3 should be > T4 for positive phase lead (T3={t3}, T4={t4})"
        );
    }
}
