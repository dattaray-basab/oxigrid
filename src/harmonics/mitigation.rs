//! Harmonic mitigation strategies: passive filters, active power filters,
//! hybrid filters, phase multiplication, PWM control, and installation measures.
//!
//! # Overview
//! This module provides design tools and analysis utilities for selecting and
//! sizing harmonic mitigation equipment in power systems.  It covers:
//!
//! - [`PassiveFilterDesigner`] — LC-circuit filter parameter calculation
//! - [`ActivePowerFilter`] — APF control logic and rating estimation
//! - [`MitigationAnalyzer`] — cost-effectiveness comparison of technologies
//! - [`HarmonicsMitigator`] — full mitigation study (legacy-compatible)
//!
//! # References
//! - IEEE Std 519-2022, "IEEE Recommended Practice for Harmonic Control in
//!   Electric Power Systems"
//! - IEC 61000-3-4, "Limitation of emission of harmonic currents"
//! - IEEE Std 1531-2003, "Application of Shunt Power Capacitors"

use std::f64::consts::PI;

use num_complex::Complex;
use serde::{Deserialize, Serialize};

use crate::error::OxiGridError;

// ─── Base frequency ────────────────────────────────────────────────────────

/// Default power-system fundamental frequency [Hz].
const F1: f64 = 60.0;

/// Default quality factor for single-tuned filters (X_L / R).
const DEFAULT_Q_FACTOR: f64 = 50.0;

// ═══════════════════════════════════════════════════════════════════════════
// Section A: Mitigation technology taxonomy (new API)
// ═══════════════════════════════════════════════════════════════════════════

/// Top-level harmonic mitigation technology selector.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum MitigationTechnology {
    PassiveFilter(PassiveFilterType),
    ActivePowerFilter(ApfConfig),
    HybridFilter(HybridFilterConfig),
    /// Phase-multiplication converter (e.g. 12-pulse, 18-pulse).
    PhaseMultiplication {
        n_pulses: usize,
    },
    PwmControl(PwmConfig),
    InstallationMeasures(InstallationConfig),
}

// ─── Passive filter types ──────────────────────────────────────────────────

/// Passive filter topology enumeration (new API).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum PassiveFilterType {
    /// Series-resonant (single-tuned) filter.
    SingleTuned {
        tuned_order: f64,
        quality_factor: f64,
        rated_mvar: f64,
    },
    /// Double-tuned filter (two resonant frequencies in one branch).
    DoubleTuned {
        order1: f64,
        order2: f64,
        quality_factor: f64,
        rated_mvar: f64,
    },
    /// High-pass damped filter (second-order damped).
    Damped {
        order: f64,
        m_factor: f64,
        rated_mvar: f64,
    },
    /// C-type or simple R-C high-pass filter.
    HighPass { cutoff_order: f64, rated_mvar: f64 },
    /// Third-order damped high-pass filter.
    ThirdOrderDamped { rated_mvar: f64 },
}

// ─── Active power filter ───────────────────────────────────────────────────

/// Active power filter configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ApfConfig {
    pub rated_kva: f64,
    pub compensation_strategy: ApfStrategy,
    /// Usable bandwidth in Hz (typically up to 1 000 Hz).
    pub bandwidth_hz: f64,
    /// Control-loop response time in ms.
    pub response_time_ms: f64,
    pub dc_bus_voltage_v: f64,
}

/// APF compensation strategy.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ApfStrategy {
    /// Inject anti-harmonic currents to cancel load harmonics.
    CurrentHarmonicCancellation,
    /// Shunt-connected voltage harmonic compensation.
    VoltageHarmonicCancellation,
    /// Target specific harmonic orders only.
    SelectiveHarmonic { orders: Vec<usize> },
    /// Combined harmonic + reactive power compensation.
    SelectivePlusReactive,
}

// ─── Hybrid filter ─────────────────────────────────────────────────────────

/// Hybrid (passive + active) filter configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HybridFilterConfig {
    pub passive: PassiveFilterType,
    pub active: ApfConfig,
    pub coupling: HybridCoupling,
}

/// Electrical coupling between the passive and active parts.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum HybridCoupling {
    Series,
    Parallel,
    SplitWinding,
}

// ─── PWM control ───────────────────────────────────────────────────────────

/// PWM converter control configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PwmConfig {
    pub switching_frequency_hz: f64,
    pub pwm_strategy: PwmStrategy,
    pub modulation_index: f64,
}

/// PWM modulation strategy.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum PwmStrategy {
    SinusoidalPwm,
    SpacePwm,
    /// Selective harmonic elimination (SHE-PWM).
    SelectiveHarmonicElimination {
        orders: Vec<usize>,
    },
}

// ─── Installation measures ─────────────────────────────────────────────────

/// Installation and system-arrangement mitigation measures.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InstallationConfig {
    /// Use K-rated transformers for harmonic-heavy loads.
    pub use_k_rated_transformer: bool,
    /// Isolation transformer (delta-delta or zig-zag) to block 3rd harmonics.
    pub isolation_transformer: bool,
    /// Separate circuits for linear and non-linear loads.
    pub separate_circuits: bool,
    /// Derating factor applied to cable ampacity (0.0–1.0).
    pub cable_derating: f64,
}

// ═══════════════════════════════════════════════════════════════════════════
// Section B: Passive filter designer (new API)
// ═══════════════════════════════════════════════════════════════════════════

/// Design tool for shunt passive harmonic filters.
///
/// All voltages and impedances are in physical units (V, Ω, H, F).
pub struct PassiveFilterDesigner {
    pub system_voltage_kv: f64,
    pub base_mva: f64,
    pub fundamental_hz: f64,
    /// System short-circuit impedance [pu on base_mva].
    pub source_impedance_pu: f64,
    /// Harmonic spectrum as `(order, magnitude_pu)` pairs.
    pub harmonic_spectrum: Vec<(usize, f64)>,
}

/// Complete set of electrical parameters for a designed passive filter.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PassiveFilterParameters {
    pub filter_type: PassiveFilterType,
    /// Inductance per phase [H].
    pub l_henry: f64,
    /// Capacitance per phase [F].
    pub c_farad: f64,
    /// Series resistance per phase [Ω] (damping).
    pub r_ohm: f64,
    /// Rated current [A].
    pub rated_current_a: f64,
    /// Reactive power at fundamental [MVAr] (capacitive, positive).
    pub reactive_power_mvar: f64,
    /// Tuned frequency [Hz].
    pub tuned_frequency_hz: f64,
    /// Quality factor Q = ω_n·L / R.
    pub quality_factor: f64,
    /// Insertion loss at tuned frequency [dB].
    pub insertion_loss_db: f64,
    /// 3 dB bandwidth [Hz].
    pub bandwidth_hz: f64,
}

impl PassiveFilterDesigner {
    /// Create a new designer.
    pub fn new(
        system_voltage_kv: f64,
        base_mva: f64,
        fundamental_hz: f64,
        source_impedance_pu: f64,
        harmonic_spectrum: Vec<(usize, f64)>,
    ) -> Self {
        Self {
            system_voltage_kv,
            base_mva,
            fundamental_hz,
            source_impedance_pu,
            harmonic_spectrum,
        }
    }

    /// System voltage (line-to-line) in volts.
    fn v_ll_v(&self) -> f64 {
        self.system_voltage_kv * 1_000.0
    }

    /// Base impedance [Ω] = V_LL² / (base_mva × 1e6).
    fn z_base_ohm(&self) -> f64 {
        let v = self.v_ll_v();
        v * v / (self.base_mva * 1e6)
    }

    /// Source impedance [Ω] from per-unit value.
    fn z_source_ohm(&self) -> f64 {
        self.source_impedance_pu * self.z_base_ohm()
    }

    /// Design a single-tuned shunt filter.
    ///
    /// The capacitor is sized to provide `reactive_power_mvar` at fundamental.
    /// The inductor is chosen so that resonance occurs at `h_n × ω₁`, where
    /// `h_n` is tuned slightly below the integer harmonic order to avoid
    /// magnifying that harmonic (common practice: tune at *h* − 0.15).
    ///
    /// # Derivation
    /// ```text
    /// C = Q / (V_LL² · ω₁)           [F]
    /// L = 1 / (C · (h_n · ω₁)²)     [H]
    /// R = (h_n · ω₁ · L) / Q_factor  [Ω]
    /// ```
    pub fn design_single_tuned(
        &self,
        harmonic_order: usize,
        reactive_power_mvar: f64,
        quality_factor: f64,
    ) -> Result<PassiveFilterParameters, OxiGridError> {
        if harmonic_order < 2 {
            return Err(OxiGridError::InvalidParameter(
                "harmonic_order must be ≥ 2".into(),
            ));
        }
        if reactive_power_mvar <= 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "reactive_power_mvar must be positive".into(),
            ));
        }
        if quality_factor <= 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "quality_factor must be positive".into(),
            ));
        }

        let omega_1 = 2.0 * PI * self.fundamental_hz;
        let v_ll = self.v_ll_v();
        let q_var = reactive_power_mvar * 1e6;

        // Detune slightly below the integer order to avoid parallel resonance
        // with the source at the harmonic of interest.
        let h_n = harmonic_order as f64 - 0.15;
        let omega_n = h_n * omega_1;

        // Capacitance: Q = V_LL² · ω₁ · C
        let c_farad = q_var / (v_ll * v_ll * omega_1);

        // Inductance from resonance condition: ω_n² = 1/(LC)
        let l_henry = 1.0 / (c_farad * omega_n * omega_n);

        // Series resistance from quality factor: Q = ω_n·L / R
        let r_ohm = omega_n * l_henry / quality_factor;

        // Rated current: I = Q / (√3 · V_LL)
        let rated_current_a = q_var / (3.0_f64.sqrt() * v_ll);

        // Insertion loss at tuned frequency (voltage-divider model)
        let z_source = self.z_source_ohm();
        let insertion_loss_db = 20.0 * (z_source / (z_source + r_ohm).abs()).log10();

        // 3 dB bandwidth: Δω = R/L  →  Δf = R/(2π·L)
        let bandwidth_hz = r_ohm / (2.0 * PI * l_henry);

        let tuned_frequency_hz = omega_n / (2.0 * PI);

        Ok(PassiveFilterParameters {
            filter_type: PassiveFilterType::SingleTuned {
                tuned_order: h_n,
                quality_factor,
                rated_mvar: reactive_power_mvar,
            },
            l_henry,
            c_farad,
            r_ohm,
            rated_current_a,
            reactive_power_mvar,
            tuned_frequency_hz,
            quality_factor,
            insertion_loss_db,
            bandwidth_hz,
        })
    }

    /// Design a C-type high-pass filter.
    ///
    /// The C-type topology uses two capacitors (C1 and C2) in series with an
    /// inductor L and resistor R.  At fundamental frequency, C1 and L form a
    /// series resonance so that no current flows through R, eliminating
    /// fundamental-frequency losses.  Above the cutoff, R provides damping.
    ///
    /// # Derivation
    /// ```text
    /// C_total = Q / (V_LL² · ω₁)
    /// n = h_c² / (h_c² - 1)
    /// C1 = n · C_total
    /// L resonates with C1 at ω₁: L = 1 / (ω₁² · C1)
    /// R = damping_factor / (ω_c · C_total)
    /// ```
    pub fn design_c_type_high_pass(
        &self,
        cutoff_order: f64,
        reactive_power_mvar: f64,
        damping_factor: f64,
    ) -> Result<PassiveFilterParameters, OxiGridError> {
        if cutoff_order <= 1.0 {
            return Err(OxiGridError::InvalidParameter(
                "cutoff_order must be > 1.0".into(),
            ));
        }
        if reactive_power_mvar <= 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "reactive_power_mvar must be positive".into(),
            ));
        }
        if damping_factor <= 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "damping_factor must be positive".into(),
            ));
        }

        let omega_1 = 2.0 * PI * self.fundamental_hz;
        let v_ll = self.v_ll_v();
        let q_var = reactive_power_mvar * 1e6;

        let c_total = q_var / (v_ll * v_ll * omega_1);
        let omega_c = cutoff_order * omega_1;

        let n = (cutoff_order * cutoff_order) / (cutoff_order * cutoff_order - 1.0);
        let c1 = n * c_total;
        let l_henry = 1.0 / (omega_1 * omega_1 * c1);

        let inv_c2 = 1.0 / c_total - 1.0 / c1;
        if inv_c2 <= 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "C-type filter geometry is infeasible for the given cutoff_order".into(),
            ));
        }

        let r_ohm = damping_factor / (omega_c * c_total);
        let rated_current_a = q_var / (3.0_f64.sqrt() * v_ll);
        let quality_factor = 1.0 / (omega_c * c_total * r_ohm);
        let z_source = self.z_source_ohm();
        let insertion_loss_db = 20.0 * (z_source / (z_source + r_ohm).abs()).log10();
        let bandwidth_hz = (r_ohm * c_total * omega_c * omega_c) / (2.0 * PI);
        let tuned_frequency_hz = omega_c / (2.0 * PI);

        Ok(PassiveFilterParameters {
            filter_type: PassiveFilterType::HighPass {
                cutoff_order,
                rated_mvar: reactive_power_mvar,
            },
            l_henry,
            c_farad: c_total,
            r_ohm,
            rated_current_a,
            reactive_power_mvar,
            tuned_frequency_hz,
            quality_factor,
            insertion_loss_db,
            bandwidth_hz,
        })
    }

    /// Compute harmonic attenuation [dB] provided by the shunt filter at a given order.
    ///
    /// A shunt filter attenuates the harmonic bus voltage by providing a low-impedance
    /// path to ground.  The bus voltage reduction ratio is:
    /// ```text
    /// V_h_after / V_h_before = Z_filter(h) / (Z_source + Z_filter(h))
    /// Attenuation [dB] = 20·log10(|Z_filter(h)| / |Z_source + Z_filter(h)|)
    /// ```
    /// At resonance Z_filter ≈ R → ratio ≈ 0 → large negative dB (strong attenuation).
    /// Far from resonance Z_filter >> Z_source → ratio ≈ 1 → 0 dB (no attenuation).
    pub fn compute_attenuation(
        &self,
        params: &PassiveFilterParameters,
        harmonic_order: usize,
    ) -> f64 {
        let h = harmonic_order as f64;
        let omega_h = h * 2.0 * PI * self.fundamental_hz;

        let x_l = omega_h * params.l_henry;
        let x_c = 1.0 / (omega_h * params.c_farad);
        let z_filter = (params.r_ohm * params.r_ohm + (x_l - x_c).powi(2)).sqrt();

        let z_source = self.z_source_ohm();
        let z_total = z_source + z_filter;

        if z_total < 1e-12 {
            return -f64::INFINITY;
        }

        // Voltage reduction ratio: V_after/V_before = Z_filter / (Z_source + Z_filter)
        20.0 * (z_filter / z_total).abs().log10()
    }

    /// Check for parallel resonance between the filter capacitor and source inductance.
    ///
    /// Parallel resonance at:
    /// ```text
    /// h_r = (1/ω₁) · √(1 / (L_source · C_filter))
    /// ```
    /// where `L_source = Z_source / ω₁`.
    ///
    /// Returns `Some(h_r)` if the resonance falls in the range 2nd–50th harmonic.
    pub fn check_resonance(&self, params: &PassiveFilterParameters) -> Option<f64> {
        let omega_1 = 2.0 * PI * self.fundamental_hz;
        let z_source = self.z_source_ohm();
        let l_source = z_source / omega_1;

        if l_source < 1e-12 || params.c_farad < 1e-12 {
            return None;
        }

        let omega_r = 1.0 / (l_source * params.c_farad).sqrt();
        let h_r = omega_r / omega_1;

        if (2.0..=50.0).contains(&h_r) {
            Some(h_r)
        } else {
            None
        }
    }

    /// Compute the THD after installing the filter.
    ///
    /// Each harmonic magnitude is reduced by the voltage-divider attenuation.
    pub fn post_filter_thd(&self, params: &PassiveFilterParameters) -> f64 {
        if self.harmonic_spectrum.is_empty() {
            return 0.0;
        }

        let sum_sq: f64 = self
            .harmonic_spectrum
            .iter()
            .map(|&(order, mag)| {
                let atten_db = self.compute_attenuation(params, order);
                let linear_factor = 10.0_f64.powf(atten_db / 20.0);
                let mag_after = mag * linear_factor.abs();
                mag_after * mag_after
            })
            .sum();

        sum_sq.sqrt() * 100.0
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Section C: Active power filter (new API)
// ═══════════════════════════════════════════════════════════════════════════

/// Active power filter controller.
pub struct ActivePowerFilter {
    pub config: ApfConfig,
    pub dc_bus_voltage_v: f64,
    pub system_voltage_v: f64,
    /// Proportional gain for the current controller.
    pub kp_current: f64,
    /// Integral gain for the current controller.
    pub ki_current: f64,
    /// Integral state variable per tracked harmonic order.
    pub integral_state: Vec<f64>,
    /// Detected harmonics: `(order, magnitude_pu, phase_rad)`.
    pub detected_harmonics: Vec<(usize, f64, f64)>,
}

impl ActivePowerFilter {
    /// Create a new APF controller with default PI gains.
    pub fn new(config: ApfConfig, system_voltage_v: f64) -> Self {
        let dc_bus = config.dc_bus_voltage_v;
        Self {
            config,
            dc_bus_voltage_v: dc_bus,
            system_voltage_v,
            kp_current: 10.0,
            ki_current: 100.0,
            integral_state: Vec::new(),
            detected_harmonics: Vec::new(),
        }
    }

    /// Detect harmonic components from a time-domain current waveform using DFT.
    ///
    /// The method first locates the fundamental frequency bin (the bin with
    /// maximum magnitude below `bandwidth_hz / 2`), then reports harmonics as
    /// integer multiples of that fundamental bin index.
    ///
    /// Returns `(order, magnitude_pu, phase_rad)` for each harmonic order from
    /// 2 up to the Nyquist limit (or `bandwidth_hz`, whichever is lower).
    pub fn detect_harmonics(
        &mut self,
        current_waveform: &[f64],
        sampling_rate_hz: f64,
    ) -> Vec<(usize, f64, f64)> {
        let n = current_waveform.len();
        if n == 0 {
            self.detected_harmonics = Vec::new();
            return Vec::new();
        }

        // Frequency resolution: df = fs / N
        let df = sampling_rate_hz / n as f64;

        // Maximum DFT bin index within bandwidth
        let max_bin = ((self.config.bandwidth_hz / df).floor() as usize).min(n / 2);
        if max_bin < 2 {
            self.detected_harmonics = Vec::new();
            return Vec::new();
        }

        // Pre-compute DFT magnitudes for bins 1..=max_bin
        let mut bin_mags = vec![0.0_f64; max_bin + 1];
        let mut bin_phases = vec![0.0_f64; max_bin + 1];
        for k in 1..=max_bin {
            let (re, im) = dft_bin(current_waveform, k);
            let mag = (re * re + im * im).sqrt() * 2.0 / n as f64;
            bin_mags[k] = mag;
            bin_phases[k] = im.atan2(re);
        }

        // Find the fundamental bin: the bin with the largest magnitude in the
        // lower half of the spectrum (bins 1 to max_bin/4 to avoid aliasing).
        let search_end = (max_bin / 4).max(1);
        let fund_bin = (1..=search_end)
            .max_by(|&a, &b| {
                bin_mags[a]
                    .partial_cmp(&bin_mags[b])
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .unwrap_or(1);

        let fund_mag = bin_mags[fund_bin];
        let fundamental_ref = if fund_mag < 1e-12 { 1.0 } else { fund_mag };

        // Collect harmonic orders (integer multiples of the fundamental bin)
        let max_order = max_bin / fund_bin;
        let mut harmonics = Vec::new();
        for order in 2..=max_order.min(50) {
            let k = order * fund_bin;
            if k > max_bin {
                break;
            }
            let mag_pu = bin_mags[k] / fundamental_ref;
            if mag_pu > 1e-6 {
                harmonics.push((order, mag_pu, bin_phases[k]));
            }
        }

        self.integral_state = vec![0.0; harmonics.len()];
        self.detected_harmonics = harmonics.clone();
        harmonics
    }

    /// Generate the compensation current at time `t`.
    ///
    /// ```text
    /// i_comp(t) = -Σ_h  I_h · sin(h·ω₁·t + φ_h)
    /// ```
    pub fn generate_compensation_current(
        &self,
        harmonics: &[(usize, f64, f64)],
        t: f64,
        omega_1: f64,
    ) -> f64 {
        harmonics.iter().fold(0.0, |acc, &(order, mag, phase)| {
            acc - mag * (order as f64 * omega_1 * t + phase).sin()
        })
    }

    /// Estimate the THD after APF compensation.
    ///
    /// The APF cancels a fraction of each harmonic limited by its rating and
    /// bandwidth.
    pub fn compute_post_apf_thd(
        &self,
        original_harmonics: &[(usize, f64, f64)],
        _sampling_rate_hz: f64,
    ) -> f64 {
        if original_harmonics.is_empty() {
            return 0.0;
        }

        // Approximate fundamental frequency from bandwidth
        let fund_freq = self.config.bandwidth_hz / 20.0;

        let total_harmonic_pu_sq: f64 = original_harmonics
            .iter()
            .map(|&(_, mag, _)| mag * mag)
            .sum();
        let total_harmonic_pu = total_harmonic_pu_sq.sqrt();

        let apf_current_pu = if self.system_voltage_v > 1.0 {
            self.config.rated_kva * 1_000.0
                / (3.0_f64.sqrt() * self.system_voltage_v * total_harmonic_pu.max(1e-6))
        } else {
            1.0
        };

        let cancellation_fraction = apf_current_pu.min(1.0);

        let sum_sq: f64 = original_harmonics
            .iter()
            .map(|&(order, mag, _)| {
                let freq_h = order as f64 * fund_freq;
                let bw_factor = if freq_h <= self.config.bandwidth_hz {
                    1.0
                } else {
                    (self.config.bandwidth_hz / freq_h).min(1.0)
                };
                let residual = mag * (1.0 - cancellation_fraction * bw_factor);
                residual * residual
            })
            .sum();

        sum_sq.sqrt() * 100.0
    }

    /// Estimate the required APF kVA rating for a given harmonic spectrum.
    ///
    /// ```text
    /// S_apf = 3 · √(Σ_h (V · I_h)²)   [kVA]
    /// ```
    pub fn estimate_rating(harmonic_spectrum: &[(usize, f64, f64)], system_voltage_kv: f64) -> f64 {
        let v = system_voltage_kv * 1_000.0;
        let sum_sq: f64 = harmonic_spectrum
            .iter()
            .map(|&(_, mag, _)| (v * mag) * (v * mag))
            .sum();
        3.0 * sum_sq.sqrt() / 1_000.0
    }
}

// ─── DFT helper (private) ──────────────────────────────────────────────────

/// Compute one DFT bin (real, imaginary) at the given integer bin index.
fn dft_bin(signal: &[f64], bin: usize) -> (f64, f64) {
    let n = signal.len();
    let (mut re, mut im) = (0.0_f64, 0.0_f64);
    for (k, &x) in signal.iter().enumerate() {
        let angle = -2.0 * PI * bin as f64 * k as f64 / n as f64;
        re += x * angle.cos();
        im += x * angle.sin();
    }
    (re, im)
}

// ═══════════════════════════════════════════════════════════════════════════
// Section D: Mitigation analyzer (new API)
// ═══════════════════════════════════════════════════════════════════════════

/// Compares multiple mitigation strategies and recommends the most
/// cost-effective solution.
pub struct MitigationAnalyzer {
    pub system_voltage_kv: f64,
    pub load_kva: f64,
    /// Harmonic spectrum: `(order, magnitude_pu, phase_rad)`.
    pub harmonic_spectrum: Vec<(usize, f64, f64)>,
    /// Target total harmonic voltage distortion [%] per IEEE 519.
    pub target_thd_v_pct: f64,
    /// Target total harmonic current distortion [%].
    pub target_thd_i_pct: f64,
}

/// A single mitigation option with techno-economic metrics.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MitigationOption {
    pub technology: MitigationTechnology,
    pub capital_cost_eur: f64,
    pub annual_operating_cost_eur: f64,
    pub achieved_thd_v_pct: f64,
    pub achieved_thd_i_pct: f64,
    pub meets_ieee519: bool,
    /// Net capacitive reactive power benefit [MVAr].
    pub reactive_power_benefit_mvar: f64,
    /// Installation complexity on a 1–5 scale.
    pub installation_complexity: f64,
    pub life_expectancy_years: f64,
    pub simple_payback_years: f64,
}

impl MitigationAnalyzer {
    /// Create a new analyzer.
    pub fn new(
        system_voltage_kv: f64,
        load_kva: f64,
        harmonic_spectrum: Vec<(usize, f64, f64)>,
        target_thd_v_pct: f64,
        target_thd_i_pct: f64,
    ) -> Self {
        Self {
            system_voltage_kv,
            load_kva,
            harmonic_spectrum,
            target_thd_v_pct,
            target_thd_i_pct,
        }
    }

    /// Compute current THD before any mitigation [%].
    ///
    /// ```text
    /// THD = √(Σ_{h≥2} I_h²) × 100
    /// ```
    pub fn current_thd(&self) -> f64 {
        let sum_sq: f64 = self
            .harmonic_spectrum
            .iter()
            .map(|&(_, mag, _)| mag * mag)
            .sum();
        sum_sq.sqrt() * 100.0
    }

    /// Analyse the single-tuned passive filter option.
    ///
    /// Targets the dominant harmonic in the spectrum.
    pub fn analyze_passive_filter(
        &self,
        reactive_mvar: f64,
        quality_factor: f64,
    ) -> MitigationOption {
        let dominant_order = self
            .harmonic_spectrum
            .iter()
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
            .map(|&(order, _, _)| order)
            .unwrap_or(5);

        let designer = PassiveFilterDesigner::new(
            self.system_voltage_kv,
            self.load_kva / 1_000.0, // kVA → MVA approximate
            50.0,
            0.05,
            self.harmonic_spectrum
                .iter()
                .map(|&(o, m, _)| (o, m))
                .collect(),
        );

        let params_result =
            designer.design_single_tuned(dominant_order, reactive_mvar, quality_factor);

        let achieved_thd = match &params_result {
            Ok(p) => designer.post_filter_thd(p),
            Err(_) => self.current_thd(),
        };

        let capital_cost = reactive_mvar * 1_000.0 * 75.0; // 75 EUR/kVAr
        let annual_opex = capital_cost * 0.01;
        let reactive_benefit = reactive_mvar;
        let reactive_savings = reactive_benefit * 1_000.0 * 30.0;
        let payback = Self::compute_payback(capital_cost, annual_opex, reactive_savings, 0.0);

        MitigationOption {
            technology: MitigationTechnology::PassiveFilter(PassiveFilterType::SingleTuned {
                tuned_order: dominant_order as f64 - 0.15,
                quality_factor,
                rated_mvar: reactive_mvar,
            }),
            capital_cost_eur: capital_cost,
            annual_operating_cost_eur: annual_opex,
            achieved_thd_v_pct: achieved_thd,
            achieved_thd_i_pct: achieved_thd,
            meets_ieee519: achieved_thd <= self.target_thd_i_pct,
            reactive_power_benefit_mvar: reactive_benefit,
            installation_complexity: 2.0,
            life_expectancy_years: 20.0,
            simple_payback_years: payback,
        }
    }

    /// Analyse the active power filter option.
    pub fn analyze_apf(&self, apf_kva: f64) -> MitigationOption {
        let config = ApfConfig {
            rated_kva: apf_kva,
            compensation_strategy: ApfStrategy::CurrentHarmonicCancellation,
            bandwidth_hz: 1_000.0,
            response_time_ms: 0.1,
            dc_bus_voltage_v: self.system_voltage_kv * 1_000.0 * 1.5,
        };

        let apf = ActivePowerFilter::new(config.clone(), self.system_voltage_kv * 1_000.0);
        let achieved_thd = apf.compute_post_apf_thd(&self.harmonic_spectrum, 10_000.0);

        let capital_cost = apf_kva * 300.0; // 300 EUR/kVA
        let annual_opex = capital_cost * 0.03;
        let payback = Self::compute_payback(capital_cost, annual_opex, 0.0, 5_000.0);

        MitigationOption {
            technology: MitigationTechnology::ActivePowerFilter(config),
            capital_cost_eur: capital_cost,
            annual_operating_cost_eur: annual_opex,
            achieved_thd_v_pct: achieved_thd,
            achieved_thd_i_pct: achieved_thd,
            meets_ieee519: achieved_thd <= self.target_thd_i_pct,
            reactive_power_benefit_mvar: 0.0,
            installation_complexity: 4.0,
            life_expectancy_years: 10.0,
            simple_payback_years: payback,
        }
    }

    /// Analyse a phase-multiplication converter option.
    ///
    /// An *n*-pulse rectifier only generates harmonics at orders `k·n ± 1`.
    /// All other harmonic orders are suppressed by approximately 80 % via
    /// phase cancellation.
    pub fn analyze_phase_multiplication(&self, n_pulses: usize) -> MitigationOption {
        let eliminated_orders: Vec<usize> = (2..=50)
            .filter(|&h| {
                let r = h % n_pulses;
                r != 1 && (n_pulses == 0 || r != n_pulses.saturating_sub(1))
            })
            .collect();

        let thd_after_sq: f64 = self
            .harmonic_spectrum
            .iter()
            .map(|&(order, mag, _)| {
                if eliminated_orders.contains(&order) {
                    let residual = mag * 0.20;
                    residual * residual
                } else {
                    mag * mag
                }
            })
            .sum();
        let achieved_thd = thd_after_sq.sqrt() * 100.0;

        let capital_cost = self.load_kva * 150.0;
        let annual_opex = capital_cost * 0.005;
        let payback = Self::compute_payback(capital_cost, annual_opex, 0.0, 8_000.0);

        MitigationOption {
            technology: MitigationTechnology::PhaseMultiplication { n_pulses },
            capital_cost_eur: capital_cost,
            annual_operating_cost_eur: annual_opex,
            achieved_thd_v_pct: achieved_thd,
            achieved_thd_i_pct: achieved_thd,
            meets_ieee519: achieved_thd <= self.target_thd_i_pct,
            reactive_power_benefit_mvar: 0.0,
            installation_complexity: 3.0,
            life_expectancy_years: 25.0,
            simple_payback_years: payback,
        }
    }

    /// Rank options by cost-effectiveness.
    ///
    /// ```text
    /// cost_effectiveness = (THD_before − THD_after) / capital_cost_EUR
    /// ```
    /// Returns indices into `options` sorted from best (highest ratio) to worst.
    pub fn rank_options(&self, options: &[MitigationOption]) -> Vec<usize> {
        let thd_before = self.current_thd();

        let mut indexed: Vec<(usize, f64)> = options
            .iter()
            .enumerate()
            .map(|(i, opt)| {
                let improvement = (thd_before - opt.achieved_thd_i_pct).max(0.0);
                let ce = if opt.capital_cost_eur > 0.0 {
                    improvement / opt.capital_cost_eur
                } else {
                    f64::INFINITY
                };
                (i, ce)
            })
            .collect();

        indexed.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        indexed.into_iter().map(|(i, _)| i).collect()
    }

    /// Calculate simple payback period in years.
    ///
    /// ```text
    /// payback = capital_cost / (annual_savings − annual_opex)
    /// ```
    pub fn compute_payback(
        capital_cost_eur: f64,
        annual_operating_cost_eur: f64,
        reactive_savings_eur_per_yr: f64,
        penalty_savings_eur_per_yr: f64,
    ) -> f64 {
        let annual_savings =
            reactive_savings_eur_per_yr + penalty_savings_eur_per_yr - annual_operating_cost_eur;
        if annual_savings <= 0.0 {
            return f64::INFINITY;
        }
        capital_cost_eur / annual_savings
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Section E: Legacy HarmonicsMitigator (preserved from original file)
// ═══════════════════════════════════════════════════════════════════════════

/// Top-level harmonic mitigation designer (legacy API).
///
/// Holds the harmonic sources and system configuration, and provides methods
/// for passive/active filter design, resonance scanning, compliance checking,
/// and strategy recommendation.
#[derive(Debug, Clone)]
pub struct HarmonicsMitigator {
    /// All known harmonic current sources in the network.
    pub sources: Vec<HarmonicSource>,
    /// Study configuration and limits.
    pub config: MitigationConfig,
    /// Filters already committed to the design.
    pub installed_filters: Vec<PassiveFilterDesign>,
}

/// A non-linear load or generator injecting harmonic currents into the network.
#[derive(Debug, Clone)]
pub struct HarmonicSource {
    /// Network bus identifier.
    pub bus_id: usize,
    /// Harmonic current spectrum: (harmonic_order, magnitude [pu]).
    pub harmonic_currents: Vec<(u32, f64)>,
    /// Descriptive type string (e.g. "VFD", "rectifier", "arc furnace").
    pub source_type: String,
}

/// Configuration parameters for the harmonic mitigation study.
#[derive(Debug, Clone)]
pub struct MitigationConfig {
    /// Nominal system voltage [kV].
    pub system_voltage_kv: f64,
    /// System base MVA.
    pub base_mva: f64,
    /// Target total harmonic distortion [%] (IEEE 519 default: 5 %).
    pub target_thd_pct: f64,
    /// Per-harmonic voltage limits: (order, max [pu]).
    pub harmonic_limits: Vec<(u32, f64)>,
    /// Harmonic order range over which to scan for parallel resonances.
    pub resonance_scan_range: (u32, u32),
}

impl Default for MitigationConfig {
    fn default() -> Self {
        Self {
            system_voltage_kv: 13.8,
            base_mva: 100.0,
            target_thd_pct: 5.0,
            harmonic_limits: vec![
                (3, 0.03),
                (5, 0.03),
                (7, 0.03),
                (9, 0.015),
                (11, 0.015),
                (13, 0.015),
                (15, 0.003),
                (17, 0.003),
                (19, 0.003),
                (23, 0.003),
                (25, 0.003),
            ],
            resonance_scan_range: (1, 25),
        }
    }
}

/// Topology / tuning specification of a passive or active filter bank (legacy).
#[derive(Debug, Clone)]
pub enum FilterType {
    /// Notch filter tuned to a single harmonic order.
    SingleTuned { tuned_harmonic: u32 },
    /// Double-notch filter with two tuning points.
    DoubleTuned { harmonic1: u32, harmonic2: u32 },
    /// High-pass filter above the cutoff harmonic (1st/2nd/3rd order).
    HighPass { cutoff_harmonic: u32, order: u8 },
    /// Damped high-pass with explicit Q factor.
    DampedHighPass { cutoff_harmonic: u32, q_factor: f64 },
    /// Active power filter targeting specific harmonic orders.
    ActiveFilter { controlled_harmonics: Vec<u32> },
    /// Hybrid: passive backbone + active supplemental compensation.
    HybridFilter {
        passive: Box<FilterType>,
        active_mva: f64,
    },
}

/// Complete design record for a passive filter bank (legacy).
#[derive(Debug, Clone)]
pub struct PassiveFilterDesign {
    pub filter_id: String,
    pub filter_type: FilterType,
    pub rated_kvar: f64,
    pub rated_kv: f64,
    pub capacitance_uf: f64,
    pub inductance_mh: f64,
    pub resistance_ohm: f64,
    pub installation_bus: usize,
}

/// A parallel-resonance peak found during an impedance frequency sweep.
#[derive(Debug, Clone)]
pub struct ResonancePoint {
    pub harmonic_order: f64,
    pub impedance_magnitude_pu: f64,
    pub risk: String,
}

/// Filter strategy recommendation from the automated adviser.
#[derive(Debug, Clone)]
pub struct FilterRecommendation {
    pub recommended_type: String,
    pub harmonics_to_filter: Vec<u32>,
    pub estimated_kvar: f64,
    pub estimated_cost_usd: f64,
    pub expected_thd_reduction_pct: f64,
}

/// IEEE 519 compliance assessment result.
#[derive(Debug, Clone)]
pub struct ComplianceResult {
    pub individual_compliant: Vec<bool>,
    pub thd_compliant: bool,
    pub thd_pct: f64,
    pub worst_violation: Option<(u32, f64)>,
}

impl HarmonicsMitigator {
    pub fn new(sources: Vec<HarmonicSource>, config: MitigationConfig) -> Self {
        Self {
            sources,
            config,
            installed_filters: Vec::new(),
        }
    }

    pub fn design_single_tuned_filter(
        &self,
        harmonic_order: u32,
        reactive_power_kvar: f64,
        v_kv: f64,
        bus: usize,
    ) -> PassiveFilterDesign {
        let f_n = harmonic_order as f64 * F1;
        let omega_n = 2.0 * PI * f_n;
        let v_v = v_kv * 1_000.0;
        let c_farad = reactive_power_kvar * 1_000.0 / (v_v * v_v * omega_n);
        let l_henry = 1.0 / (c_farad * omega_n * omega_n);
        let x_l = omega_n * l_henry;
        let r_ohm = x_l / DEFAULT_Q_FACTOR;
        PassiveFilterDesign {
            filter_id: format!("ST_H{harmonic_order}"),
            filter_type: FilterType::SingleTuned {
                tuned_harmonic: harmonic_order,
            },
            rated_kvar: reactive_power_kvar,
            rated_kv: v_kv,
            capacitance_uf: c_farad * 1e6,
            inductance_mh: l_henry * 1e3,
            resistance_ohm: r_ohm,
            installation_bus: bus,
        }
    }

    pub fn design_high_pass_filter(
        &self,
        cutoff_harmonic: u32,
        q_factor: f64,
        reactive_power_kvar: f64,
        v_kv: f64,
        bus: usize,
    ) -> PassiveFilterDesign {
        let f_c = cutoff_harmonic as f64 * F1;
        let omega_c = 2.0 * PI * f_c;
        let omega_1 = 2.0 * PI * F1;
        let v_v = v_kv * 1_000.0;
        let c_farad = reactive_power_kvar * 1_000.0 / (v_v * v_v * omega_1);
        let x_c_fund = 1.0 / (omega_1 * c_farad);
        let l_henry = 1.0 / (c_farad * omega_c * omega_c);
        let r_ohm = q_factor * x_c_fund;
        PassiveFilterDesign {
            filter_id: format!("HP_H{cutoff_harmonic}"),
            filter_type: FilterType::HighPass {
                cutoff_harmonic,
                order: 2,
            },
            rated_kvar: reactive_power_kvar,
            rated_kv: v_kv,
            capacitance_uf: c_farad * 1e6,
            inductance_mh: l_henry * 1e3,
            resistance_ohm: r_ohm,
            installation_bus: bus,
        }
    }

    pub fn filter_impedance(
        &self,
        filter: &PassiveFilterDesign,
        frequency_hz: f64,
    ) -> Complex<f64> {
        let omega = 2.0 * PI * frequency_hz;
        let c = filter.capacitance_uf * 1e-6;
        let l = filter.inductance_mh * 1e-3;
        let r = filter.resistance_ohm;

        match &filter.filter_type {
            FilterType::SingleTuned { .. } | FilterType::DoubleTuned { .. } => {
                let z_l = omega * l;
                let z_c = if omega > 0.0 {
                    1.0 / (omega * c)
                } else {
                    f64::MAX
                };
                Complex::new(r, z_l - z_c)
            }
            FilterType::HighPass { .. } | FilterType::DampedHighPass { .. } => {
                let z_cap = Complex::new(0.0, -1.0 / (omega * c));
                let z_ind = Complex::new(0.0, omega * l);
                let z_res = Complex::new(r, 0.0);
                let z_rl_parallel = (z_res * z_ind) / (z_res + z_ind);
                z_cap + z_rl_parallel
            }
            FilterType::ActiveFilter { .. } => Complex::new(1e6, 0.0),
            FilterType::HybridFilter { passive, .. } => {
                let inner = PassiveFilterDesign {
                    filter_id: filter.filter_id.clone(),
                    filter_type: *passive.clone(),
                    rated_kvar: filter.rated_kvar,
                    rated_kv: filter.rated_kv,
                    capacitance_uf: filter.capacitance_uf,
                    inductance_mh: filter.inductance_mh,
                    resistance_ohm: filter.resistance_ohm,
                    installation_bus: filter.installation_bus,
                };
                self.filter_impedance(&inner, frequency_hz)
            }
        }
    }

    pub fn harmonic_voltage_after_filter(
        &self,
        sources: &[HarmonicSource],
        filter: &PassiveFilterDesign,
        bus_impedance_pu: f64,
    ) -> Vec<(u32, f64)> {
        let mut current_map: std::collections::BTreeMap<u32, f64> =
            std::collections::BTreeMap::new();
        for src in sources {
            for &(order, mag) in &src.harmonic_currents {
                *current_map.entry(order).or_insert(0.0) += mag;
            }
        }
        let z_bus = Complex::new(bus_impedance_pu, 0.0);
        current_map
            .into_iter()
            .map(|(h, i_h)| {
                let f_h = h as f64 * F1;
                let z_filter = self.filter_impedance(filter, f_h);
                let z_parallel = if (z_filter + z_bus).norm() > 1e-12 {
                    z_filter * z_bus / (z_filter + z_bus)
                } else {
                    Complex::new(0.0, 0.0)
                };
                (h, i_h * z_parallel.norm())
            })
            .collect()
    }

    pub fn calculate_thd_after_mitigation(
        &self,
        before: &[(u32, f64)],
        after: &[(u32, f64)],
    ) -> (f64, f64) {
        let thd = |voltages: &[(u32, f64)]| -> f64 {
            let v1 = voltages
                .iter()
                .find(|&&(h, _)| h == 1)
                .map(|&(_, v)| v)
                .unwrap_or(1.0);
            let v1 = if v1 < 1e-12 { 1.0 } else { v1 };
            let sum_sq: f64 = voltages
                .iter()
                .filter(|&&(h, _)| h >= 2)
                .map(|&(_, v)| v * v)
                .sum();
            if v1 > 1e-12 {
                sum_sq.sqrt() / v1 * 100.0
            } else {
                0.0
            }
        };
        (thd(before), thd(after))
    }

    pub fn scan_for_resonance(
        &self,
        filter: &PassiveFilterDesign,
        system_impedance_freq_scan: &[(f64, f64)],
    ) -> Vec<ResonancePoint> {
        if system_impedance_freq_scan.len() < 3 {
            return Vec::new();
        }
        let magnitudes: Vec<(f64, f64)> = system_impedance_freq_scan
            .iter()
            .map(|&(freq_hz, z_sys_pu)| {
                let z_filter = self.filter_impedance(filter, freq_hz);
                let z_sys = Complex::new(z_sys_pu, 0.0);
                let denom = z_sys + z_filter;
                let z_total = if denom.norm() > 1e-12 {
                    z_sys * z_filter / denom
                } else {
                    Complex::new(0.0, 0.0)
                };
                (freq_hz, z_total.norm())
            })
            .collect();
        let n = magnitudes.len();
        let mut resonances = Vec::new();
        for i in 1..(n - 1) {
            let (freq, mag) = magnitudes[i];
            if mag > magnitudes[i - 1].1 && mag > magnitudes[i + 1].1 {
                let harmonic_order = freq / F1;
                let risk = if mag > 5.0 {
                    "High".to_string()
                } else if mag > 2.0 {
                    "Medium".to_string()
                } else {
                    "Low".to_string()
                };
                resonances.push(ResonancePoint {
                    harmonic_order,
                    impedance_magnitude_pu: mag,
                    risk,
                });
            }
        }
        resonances
    }

    pub fn recommend_filter_strategy(&self, sources: &[HarmonicSource]) -> FilterRecommendation {
        let mut current_map: std::collections::BTreeMap<u32, f64> =
            std::collections::BTreeMap::new();
        for src in sources {
            for &(order, mag) in &src.harmonic_currents {
                *current_map.entry(order).or_insert(0.0) += mag;
            }
        }
        let i1 = current_map.get(&1).copied().unwrap_or(1.0);
        let i1 = if i1 < 1e-12 { 1.0 } else { i1 };
        let thd_current: f64 = current_map
            .iter()
            .filter(|(&h, _)| h >= 2)
            .map(|(_, &mag)| mag * mag)
            .sum::<f64>()
            .sqrt()
            / i1
            * 100.0;
        let mut dominant: Vec<(u32, f64)> = current_map
            .iter()
            .filter(|(&h, &mag)| h >= 2 && mag > 0.02 * i1)
            .map(|(&h, &mag)| (h, mag))
            .collect();
        dominant.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        let harmonics_to_filter: Vec<u32> = dominant.iter().map(|&(h, _)| h).collect();
        let n_dom = dominant.len();
        let (recommended_type, estimated_kvar, estimated_cost_usd, expected_thd_reduction_pct) =
            if n_dom == 0 {
                ("None".to_string(), 0.0, 0.0, 0.0)
            } else if n_dom == 1 {
                let dom_mag = dominant[0].1;
                let kvar = 200.0 * dom_mag * 100.0;
                ("SingleTuned".to_string(), kvar, 15_000.0, 55.0)
            } else if n_dom <= 3 {
                let kvar = dominant
                    .iter()
                    .map(|&(_, m)| 200.0 * m * 100.0)
                    .sum::<f64>();
                ("DoubleTuned".to_string(), kvar, 25_000.0, 70.0)
            } else {
                let kvar = 300.0 * thd_current;
                ("HighPass".to_string(), kvar, 20_000.0, 80.0)
            };
        FilterRecommendation {
            recommended_type,
            harmonics_to_filter,
            estimated_kvar,
            estimated_cost_usd,
            expected_thd_reduction_pct,
        }
    }

    pub fn active_filter_reference_current(
        &self,
        harmonic_sources: &[HarmonicSource],
        controlled_harmonics: &[u32],
    ) -> Vec<f64> {
        let n_samples = 1_000usize;
        let dt = 1.0 / (n_samples as f64 * F1);
        let mut reference = vec![0.0f64; n_samples];
        for (k, slot) in reference.iter_mut().enumerate() {
            let t = k as f64 * dt;
            let mut sample = 0.0f64;
            for src in harmonic_sources {
                for &(order, mag) in &src.harmonic_currents {
                    if controlled_harmonics.contains(&order) {
                        sample += mag * (2.0 * PI * order as f64 * F1 * t).sin();
                    }
                }
            }
            *slot = sample;
        }
        reference
    }

    pub fn evaluate_compliance(&self, harmonic_voltages_pu: &[(u32, f64)]) -> ComplianceResult {
        let v1 = harmonic_voltages_pu
            .iter()
            .find(|&&(h, _)| h == 1)
            .map(|&(_, v)| v)
            .unwrap_or(1.0);
        let v1 = if v1 < 1e-12 { 1.0 } else { v1 };
        let sum_sq: f64 = harmonic_voltages_pu
            .iter()
            .filter(|&&(h, _)| h >= 2)
            .map(|&(_, v)| v * v)
            .sum();
        let thd_pct = sum_sq.sqrt() / v1 * 100.0;
        let thd_compliant = thd_pct <= self.config.target_thd_pct;
        let mut individual_compliant = Vec::with_capacity(harmonic_voltages_pu.len());
        let mut worst_violation: Option<(u32, f64)> = None;
        for &(order, voltage) in harmonic_voltages_pu {
            let limit_opt = self
                .config
                .harmonic_limits
                .iter()
                .find(|&&(lim_order, _)| lim_order == order)
                .map(|&(_, lim)| lim);
            let compliant = match limit_opt {
                Some(lim) => voltage <= lim,
                None => true,
            };
            individual_compliant.push(compliant);
            if let Some(lim) = limit_opt {
                let excess = voltage - lim;
                if excess > 0.0 {
                    let update = match worst_violation {
                        None => true,
                        Some((_, prev_excess)) => excess > prev_excess,
                    };
                    if update {
                        worst_violation = Some((order, excess));
                    }
                }
            }
        }
        ComplianceResult {
            individual_compliant,
            thd_compliant,
            thd_pct,
            worst_violation,
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    // ── helpers for new API ───────────────────────────────────────────────

    fn sample_designer() -> PassiveFilterDesigner {
        PassiveFilterDesigner::new(
            11.0,
            10.0,
            50.0,
            0.05,
            vec![(5, 0.20), (7, 0.14), (11, 0.09), (13, 0.07)],
        )
    }

    fn sample_5th_filter() -> PassiveFilterParameters {
        sample_designer()
            .design_single_tuned(5, 3.0, 50.0)
            .expect("5th filter design must succeed")
    }

    fn sample_spectrum() -> Vec<(usize, f64, f64)> {
        vec![
            (5, 0.20, 0.0),
            (7, 0.14, 0.5),
            (11, 0.09, 1.0),
            (13, 0.07, 1.5),
        ]
    }

    fn sample_apf() -> ActivePowerFilter {
        let cfg = ApfConfig {
            rated_kva: 500.0,
            compensation_strategy: ApfStrategy::CurrentHarmonicCancellation,
            bandwidth_hz: 1_000.0,
            response_time_ms: 0.1,
            dc_bus_voltage_v: 800.0,
        };
        ActivePowerFilter::new(cfg, 11_000.0)
    }

    fn sample_analyzer() -> MitigationAnalyzer {
        MitigationAnalyzer::new(11.0, 2_000.0, sample_spectrum(), 5.0, 8.0)
    }

    // ── helpers for legacy API ────────────────────────────────────────────

    fn default_mitigator() -> HarmonicsMitigator {
        HarmonicsMitigator::new(Vec::new(), MitigationConfig::default())
    }

    // ════════════════════════════════════════════════════════════════════
    // New-API tests (20 required)
    // ════════════════════════════════════════════════════════════════════

    #[test]
    fn test_passive_filter_designer_creation() {
        let d = sample_designer();
        assert!((d.system_voltage_kv - 11.0).abs() < 1e-10);
        assert!((d.fundamental_hz - 50.0).abs() < 1e-10);
        assert_eq!(d.harmonic_spectrum.len(), 4);
    }

    #[test]
    fn test_design_single_tuned_5th() {
        let p = sample_5th_filter();
        assert!(p.l_henry > 0.0, "L must be positive");
        assert!(p.c_farad > 0.0, "C must be positive");
        assert!(p.r_ohm > 0.0, "R must be positive");
        let expected_hz = (5.0 - 0.15) * 50.0;
        assert!(
            (p.tuned_frequency_hz - expected_hz).abs() < 1.0,
            "tuned_frequency_hz={:.2} expected≈{:.2}",
            p.tuned_frequency_hz,
            expected_hz
        );
    }

    #[test]
    fn test_design_single_tuned_7th() {
        let d = sample_designer();
        let p = d
            .design_single_tuned(7, 2.0, 40.0)
            .expect("7th harmonic filter design must succeed");
        let expected_hz = (7.0 - 0.15) * 50.0;
        assert!(
            (p.tuned_frequency_hz - expected_hz).abs() < 1.0,
            "7th filter: tuned_frequency_hz={:.2}, expected≈{:.2}",
            p.tuned_frequency_hz,
            expected_hz
        );
        assert!(p.quality_factor > 0.0);
    }

    #[test]
    fn test_single_tuned_l_c_product() {
        // LC = 1 / (h_n · ω₁)²  by construction
        let p = sample_5th_filter();
        let omega_1 = 2.0 * PI * 50.0;
        let h_n = 5.0 - 0.15;
        let expected_lc = 1.0 / (h_n * omega_1).powi(2);
        let actual_lc = p.l_henry * p.c_farad;
        let rel_err = ((actual_lc - expected_lc) / expected_lc).abs();
        assert!(
            rel_err < 1e-6,
            "LC product relative error too large: {:.2e}",
            rel_err
        );
    }

    #[test]
    fn test_single_tuned_quality_factor() {
        // Q = ω_n · L / R  must match stored quality_factor
        let p = sample_5th_filter();
        let omega_n = p.tuned_frequency_hz * 2.0 * PI;
        let q_computed = omega_n * p.l_henry / p.r_ohm;
        assert!(
            (q_computed - p.quality_factor).abs() < 0.01,
            "quality_factor mismatch: stored={:.4}, computed={:.4}",
            p.quality_factor,
            q_computed
        );
    }

    #[test]
    fn test_design_c_type_high_pass() {
        let d = sample_designer();
        let p = d
            .design_c_type_high_pass(3.0, 2.0, 1.0)
            .expect("C-type HP design must succeed");
        assert!(p.l_henry > 0.0);
        assert!(p.c_farad > 0.0);
        assert!(p.r_ohm > 0.0);
        let expected_hz = 3.0 * 50.0;
        assert!(
            (p.tuned_frequency_hz - expected_hz).abs() < 1.0,
            "C-type cutoff: {:.2} Hz vs expected {:.2} Hz",
            p.tuned_frequency_hz,
            expected_hz
        );
    }

    #[test]
    fn test_compute_attenuation_at_tuned_freq() {
        let d = sample_designer();
        let p = d
            .design_single_tuned(5, 3.0, 50.0)
            .expect("design must succeed");
        // At tuned order (≈5), attenuation should be greatest (most negative dB)
        let atten_5 = d.compute_attenuation(&p, 5);
        let atten_11 = d.compute_attenuation(&p, 11);
        assert!(
            atten_5 < atten_11,
            "Attenuation at 5th ({:.2} dB) should be greater than at 11th ({:.2} dB)",
            atten_5,
            atten_11
        );
    }

    #[test]
    fn test_compute_attenuation_off_tune() {
        let d = sample_designer();
        let p = sample_5th_filter();
        let atten_5 = d.compute_attenuation(&p, 5);
        let atten_7 = d.compute_attenuation(&p, 7);
        assert!(
            atten_5.abs() > atten_7.abs(),
            "Off-tune attenuation: |atten_5|={:.4} should exceed |atten_7|={:.4}",
            atten_5.abs(),
            atten_7.abs()
        );
    }

    #[test]
    fn test_check_resonance_present() {
        let d = PassiveFilterDesigner::new(11.0, 10.0, 50.0, 0.05, vec![(5, 0.20)]);
        let p = d
            .design_single_tuned(5, 3.0, 50.0)
            .expect("design must succeed");
        if let Some(h_r) = d.check_resonance(&p) {
            assert!((2.0..=50.0).contains(&h_r), "h_r={:.2} out of range", h_r);
        }
        // Pass even if None — resonance may be outside [2,50] for this config
    }

    #[test]
    fn test_check_resonance_absent() {
        // Very small source impedance → L_source ≈ 0 → ω_r → ∞ → h_r >> 50
        let d = PassiveFilterDesigner::new(11.0, 10.0, 50.0, 1e-6, vec![(5, 0.20)]);
        let p = d
            .design_single_tuned(5, 3.0, 50.0)
            .expect("design must succeed");
        let resonance = d.check_resonance(&p);
        assert!(
            resonance.is_none(),
            "Expected no resonance in [2,50], got h_r={:?}",
            resonance
        );
    }

    #[test]
    fn test_post_filter_thd_reduced() {
        let d = sample_designer();
        let p = sample_5th_filter();
        let thd_before: f64 = {
            let sum_sq: f64 = d.harmonic_spectrum.iter().map(|&(_, m)| m * m).sum();
            sum_sq.sqrt() * 100.0
        };
        let thd_after = d.post_filter_thd(&p);
        assert!(
            thd_after < thd_before,
            "THD after filter ({:.2}%) should be less than before ({:.2}%)",
            thd_after,
            thd_before
        );
    }

    #[test]
    fn test_apf_creation() {
        let apf = sample_apf();
        assert!((apf.config.rated_kva - 500.0).abs() < 1e-10);
        assert!((apf.system_voltage_v - 11_000.0).abs() < 1e-10);
        assert!((apf.dc_bus_voltage_v - 800.0).abs() < 1e-10);
    }

    #[test]
    fn test_detect_harmonics_pure_fundamental() {
        let mut apf = sample_apf();
        let fs = 10_000.0_f64;
        let n = 800_usize; // 4 cycles @ 50 Hz
        let waveform: Vec<f64> = (0..n)
            .map(|k| (2.0 * PI * 50.0 * k as f64 / fs).sin())
            .collect();
        let harmonics = apf.detect_harmonics(&waveform, fs);
        for &(order, mag, _) in &harmonics {
            assert!(
                mag < 0.05,
                "order={order}: mag={mag:.4} too large for pure fundamental"
            );
        }
    }

    #[test]
    fn test_detect_harmonics_with_5th() {
        let mut apf = sample_apf();
        let fs = 10_000.0_f64;
        let n = 800_usize;
        let waveform: Vec<f64> = (0..n)
            .map(|k| {
                let t = k as f64 / fs;
                (2.0 * PI * 50.0 * t).sin() + 0.20 * (2.0 * PI * 250.0 * t).sin()
            })
            .collect();
        let harmonics = apf.detect_harmonics(&waveform, fs);
        let fifth = harmonics.iter().find(|&&(o, _, _)| o == 5);
        assert!(fifth.is_some(), "5th harmonic should be detected");
        if let Some(&(_, mag, _)) = fifth {
            assert!(
                (mag - 0.20).abs() < 0.05,
                "5th harmonic magnitude {mag:.4} should be ≈ 0.20"
            );
        }
    }

    #[test]
    fn test_generate_compensation_current() {
        let apf = sample_apf();
        let harmonics = vec![(5_usize, 0.20_f64, 0.0_f64)];
        let omega_1 = 2.0 * PI * 50.0;
        let t = 0.001;
        let i_comp = apf.generate_compensation_current(&harmonics, t, omega_1);
        let expected = -0.20 * (5.0 * omega_1 * t).sin();
        assert!(
            (i_comp - expected).abs() < 1e-10,
            "compensation current mismatch: {i_comp:.6} vs {expected:.6}"
        );
    }

    #[test]
    fn test_estimate_apf_rating() {
        let spectrum = vec![(5, 0.20, 0.0), (7, 0.14, 0.0)];
        let rating = ActivePowerFilter::estimate_rating(&spectrum, 11.0);
        assert!(rating > 0.0, "APF rating must be positive");
        let v = 11_000.0_f64;
        let sum_sq: f64 = spectrum.iter().map(|&(_, m, _)| (v * m) * (v * m)).sum();
        let expected = 3.0 * sum_sq.sqrt() / 1_000.0;
        assert!(
            (rating - expected).abs() < 1e-6,
            "rating={rating:.4} expected={expected:.4}"
        );
    }

    #[test]
    fn test_mitigation_analyzer_current_thd() {
        let a = sample_analyzer();
        let thd = a.current_thd();
        let expected = (0.20_f64.powi(2) + 0.14_f64.powi(2) + 0.09_f64.powi(2) + 0.07_f64.powi(2))
            .sqrt()
            * 100.0;
        assert!(
            (thd - expected).abs() < 0.01,
            "THD={thd:.4}% expected={expected:.4}%"
        );
    }

    #[test]
    fn test_analyze_passive_filter_option() {
        let a = sample_analyzer();
        let opt = a.analyze_passive_filter(3.0, 50.0);
        assert!(opt.capital_cost_eur > 0.0);
        assert!(opt.life_expectancy_years > 0.0);
        assert!(opt.reactive_power_benefit_mvar > 0.0);
        assert!(opt.achieved_thd_i_pct >= 0.0);
    }

    #[test]
    fn test_analyze_apf_option() {
        let a = sample_analyzer();
        let opt = a.analyze_apf(200.0);
        assert!(opt.capital_cost_eur > 0.0, "capital cost must be positive");
        assert!(opt.achieved_thd_i_pct >= 0.0, "THD must be non-negative");
        assert!(
            opt.achieved_thd_i_pct <= a.current_thd(),
            "APF THD ({:.2}%) must be ≤ uncompensated ({:.2}%)",
            opt.achieved_thd_i_pct,
            a.current_thd()
        );
    }

    #[test]
    fn test_rank_options_by_cost_effectiveness() {
        let a = sample_analyzer();
        let opt1 = a.analyze_passive_filter(3.0, 50.0);
        let opt2 = a.analyze_apf(200.0);
        let opt3 = a.analyze_phase_multiplication(12);
        let options = vec![opt1, opt2, opt3];
        let ranking = a.rank_options(&options);
        assert_eq!(ranking.len(), 3, "ranking must contain all options");
        let mut sorted_ranking = ranking.clone();
        sorted_ranking.sort_unstable();
        assert_eq!(sorted_ranking, vec![0, 1, 2]);
    }

    // ════════════════════════════════════════════════════════════════════
    // Legacy-API tests (preserved)
    // ════════════════════════════════════════════════════════════════════

    #[test]
    fn test_single_tuned_5th_harmonic_impedance_minimum() {
        let m = default_mitigator();
        let filter = m.design_single_tuned_filter(5, 300.0, 13.8, 1);
        let z_at_tuning = m.filter_impedance(&filter, 5.0 * F1);
        let z_off = m.filter_impedance(&filter, 3.0 * F1);
        let ratio = z_at_tuning.im.abs() / z_at_tuning.re.abs();
        assert!(
            ratio < 1e-6,
            "Expected near-zero imaginary part at tuning frequency, got Im/Re = {ratio}"
        );
        assert!(
            z_off.norm() > z_at_tuning.norm(),
            "Off-tune |Z| should exceed tuning-frequency |Z|"
        );
    }

    #[test]
    fn test_single_tuned_lc_resonance() {
        let m = default_mitigator();
        for &h in &[5u32, 7, 11, 13] {
            let filter = m.design_single_tuned_filter(h, 200.0, 13.8, 1);
            let omega_n = 2.0 * PI * h as f64 * F1;
            let l = filter.inductance_mh * 1e-3;
            let c = filter.capacitance_uf * 1e-6;
            let product = omega_n * omega_n * l * c;
            assert!(
                (product - 1.0).abs() < 1e-9,
                "Harmonic {h}: ω²LC = {product}, expected 1.0"
            );
        }
    }

    #[test]
    fn test_thd_reduced_after_mitigation() {
        let m = default_mitigator();
        let before: Vec<(u32, f64)> = vec![(1, 1.0), (5, 0.20), (7, 0.14), (11, 0.09), (13, 0.07)];
        let after: Vec<(u32, f64)> = vec![(1, 1.0), (5, 0.02), (7, 0.14), (11, 0.09), (13, 0.07)];
        let (thd_b, thd_a) = m.calculate_thd_after_mitigation(&before, &after);
        assert!(
            thd_a < thd_b,
            "THD after ({thd_a:.2}%) should be less than THD before ({thd_b:.2}%)"
        );
    }

    #[test]
    fn test_resonance_scan_finds_peak() {
        let m = default_mitigator();
        let filter = m.design_single_tuned_filter(5, 300.0, 13.8, 1);
        let scan: Vec<(f64, f64)> = (10..=300)
            .map(|i| {
                let freq = i as f64 * F1 / 10.0;
                let z = if (freq - 240.0).abs() < 30.0 {
                    0.1 + 5.0 * (1.0 - (freq - 240.0).abs() / 30.0)
                } else {
                    0.1
                };
                (freq, z)
            })
            .collect();
        let resonances = m.scan_for_resonance(&filter, &scan);
        assert!(
            !resonances.is_empty(),
            "Expected at least one resonance point, found none"
        );
    }

    #[test]
    fn test_high_pass_filter_design() {
        let m = default_mitigator();
        let filter = m.design_high_pass_filter(7, 1.0, 300.0, 13.8, 1);
        let z_fund = m.filter_impedance(&filter, F1).norm();
        let z_high = m.filter_impedance(&filter, 11.0 * F1).norm();
        assert!(
            z_high < z_fund,
            "High-pass: |Z| at high freq ({z_high:.4}) < fundamental ({z_fund:.4})"
        );
    }

    #[test]
    fn test_active_filter_reference_current_length() {
        let m = default_mitigator();
        let sources = vec![HarmonicSource {
            bus_id: 1,
            harmonic_currents: vec![(5, 0.2), (7, 0.14)],
            source_type: "VFD".to_string(),
        }];
        let ref_curr = m.active_filter_reference_current(&sources, &[5, 7]);
        assert_eq!(ref_curr.len(), 1_000);
    }

    #[test]
    fn test_compliance_check_within_limits() {
        let m = default_mitigator();
        let voltages: Vec<(u32, f64)> = vec![(1, 1.0), (5, 0.02), (7, 0.02), (11, 0.01)];
        let result = m.evaluate_compliance(&voltages);
        assert!(result.individual_compliant.iter().all(|&c| c));
        assert!(result.thd_compliant);
        assert!(result.worst_violation.is_none());
    }

    #[test]
    fn test_compliance_check_violation() {
        let m = default_mitigator();
        let voltages: Vec<(u32, f64)> = vec![(1, 1.0), (5, 0.08), (7, 0.01)];
        let result = m.evaluate_compliance(&voltages);
        let h5_idx = voltages
            .iter()
            .position(|&(h, _)| h == 5)
            .expect("5th present");
        assert!(!result.individual_compliant[h5_idx]);
        assert!(result.worst_violation.is_some());
        let (viol_order, excess) = result.worst_violation.expect("violation expected");
        assert_eq!(viol_order, 5);
        assert!(excess > 0.0);
    }
}
