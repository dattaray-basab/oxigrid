//! Grid Impedance Spectroscopy (GIS) — broadband impedance measurement,
//! resonance identification, harmonic interaction analysis, and stability
//! assessment via impedance-based methods.
//!
//! # Units
//! - Frequencies  : \[Hz\]
//! - Impedances   : \[pu\]
//! - Gain margins : \[dB\]
//! - Phase margins : \[deg\]
//! - Q factor     : dimensionless

use num_complex::Complex;

// ─── fundamental frequency ────────────────────────────────────────────────────

/// System fundamental frequency \[Hz\] (50 Hz default; used for harmonic orders).
pub const FUNDAMENTAL_HZ: f64 = 50.0;

// ─── enums ────────────────────────────────────────────────────────────────────

/// How the impedance spectrum was obtained.
#[derive(Debug, Clone, PartialEq)]
pub enum MeasurementMethod {
    /// Inject small perturbation signal and measure voltage/current response.
    Perturbation,
    /// Wide-band hardware network analyser.
    NetworkAnalyzer,
    /// Computed analytically from the system model.
    Estimation,
    /// Injected by an Inverter-Based Resource (IBR) internal observer.
    PerturbationObserver,
}

/// Classification of a resonance peak.
#[derive(Debug, Clone, PartialEq)]
pub enum ResonanceType {
    /// Impedance local minimum (current magnification risk).
    SeriesResonance,
    /// Impedance local maximum (voltage magnification risk).
    ParallelResonance,
    /// Anti-resonance (sharp dip adjacent to a parallel peak).
    AntiResonance,
}

// ─── core data structures ─────────────────────────────────────────────────────

/// A single frequency sample of the complex bus impedance.
#[derive(Debug, Clone)]
pub struct FrequencyPoint {
    /// Measurement frequency \[Hz\].
    pub frequency_hz: f64,
    /// Complex impedance Z = R + jX \[pu\].
    pub impedance: Complex<f64>,
}

/// Impedance spectrum (collection of [`FrequencyPoint`]s) at one bus.
#[derive(Debug, Clone)]
pub struct ImpedanceSpectrum {
    /// Ordered (ascending frequency) frequency-impedance samples.
    pub points: Vec<FrequencyPoint>,
    /// Bus identifier (0-indexed row/column of Y-bus).
    pub bus_id: usize,
    /// How the data were acquired / computed.
    pub measurement_method: MeasurementMethod,
    /// Unix-epoch timestamp of the measurement \[s\].
    pub timestamp: f64,
}

/// A detected resonance peak in an [`ImpedanceSpectrum`].
#[derive(Debug, Clone)]
pub struct ResonancePoint {
    /// Resonance frequency \[Hz\].
    pub frequency_hz: f64,
    /// Quality factor Q = f_res / (f_upper − f_lower) \[dimensionless\].
    pub quality_factor: f64,
    /// Peak impedance magnitude at resonance \[pu\].
    pub peak_impedance_pu: f64,
    /// Series / parallel / anti-resonance classification.
    pub resonance_type: ResonanceType,
}

// ─── output structs ───────────────────────────────────────────────────────────

/// One inter-modulation / harmonic voltage product.
#[derive(Debug, Clone)]
pub struct IntermodProduct {
    /// Harmonic order h (V_h = I_h × Z(h × f1)).
    pub harmonic_order: u32,
    /// Voltage distortion percentage at this harmonic \[%\].
    pub voltage_distortion_pct: f64,
    /// `true` if distortion exceeds IEEE 519 limit of 3 %.
    pub exceeds_limit: bool,
    /// Bus impedance magnitude at h × f1 \[pu\].
    pub impedance_pu: f64,
}

/// Nyquist-based stability margins for a source–load impedance ratio.
#[derive(Debug, Clone)]
pub struct StabilityMargin {
    /// Gain margin = 20 log10(1 / |T|) evaluated where ∠T = −180° \[dB\].
    pub gain_margin_db: f64,
    /// Phase margin = ∠T + 180° evaluated where |T| = 1 \[deg\].
    pub phase_margin_deg: f64,
    /// `true` if gain_margin_db ≥ config threshold AND phase_margin_deg ≥ config threshold.
    pub stable: bool,
    /// Frequency at which the worst (smallest) margin occurs \[Hz\].
    pub worst_frequency_hz: f64,
}

/// Summary of changes between two impedance spectra (pre- vs post-modification).
#[derive(Debug, Clone)]
pub struct SpectrumComparisonReport {
    /// Resonance frequencies present in `post` but absent in `pre` \[Hz\].
    pub resonances_added: Vec<f64>,
    /// Resonance frequencies present in `pre` but absent in `post` \[Hz\].
    pub resonances_removed: Vec<f64>,
    /// Change in maximum |Z| across the spectrum: post_peak − pre_peak \[pu\].
    pub peak_impedance_change_pu: f64,
    /// `true` when a new resonance was added or peak impedance increased > 10 %.
    pub stability_degraded: bool,
}

/// Grid strength characterisation at a point of coupling.
#[derive(Debug, Clone)]
pub struct GridStrengthReport {
    /// Short-circuit ratio SCR = S_sc / P_rated \[dimensionless\].
    pub scr: f64,
    /// X/R ratio at the fundamental frequency \[dimensionless\].
    pub x_r_ratio: f64,
    /// Impedance angle at the fundamental: arctan(X/R) \[deg\].
    pub impedance_angle_deg: f64,
    /// Human-readable classification: `"Strong"`, `"Moderate"`, or `"Weak"`.
    pub grid_strength: String,
}

// ─── configuration ────────────────────────────────────────────────────────────

/// Thresholds used during stability assessment.
#[derive(Debug, Clone)]
pub struct StabilityAssessmentConfig {
    /// Minimum acceptable gain margin \[dB\] (default 6 dB).
    pub gain_margin_threshold_db: f64,
    /// Minimum acceptable phase margin \[deg\] (default 30°).
    pub phase_margin_threshold_deg: f64,
    /// Harmonic orders to flag when they coincide with a resonance.
    pub harmonic_orders_to_check: Vec<u32>,
}

impl Default for StabilityAssessmentConfig {
    fn default() -> Self {
        Self {
            gain_margin_threshold_db: 6.0,
            phase_margin_threshold_deg: 30.0,
            harmonic_orders_to_check: vec![2, 3, 5, 7, 11, 13, 17, 19, 23, 25],
        }
    }
}

// ─── error type ───────────────────────────────────────────────────────────────

/// Errors produced by the impedance spectroscopy module.
#[derive(Debug)]
pub enum SpectroscopyError {
    /// Bus index exceeds Y-bus dimensions.
    BusIndexOutOfRange { bus_idx: usize, n_buses: usize },
    /// Y-bus diagonal element is zero (singular or degenerate).
    SingularAdmittance { bus_idx: usize },
    /// Spectrum contains fewer than 3 points (not enough for resonance search).
    InsufficientPoints { n_points: usize },
    /// Spectra being compared have incompatible bus IDs.
    BusMismatch { pre_bus: usize, post_bus: usize },
    /// Source and load impedance spectra have different lengths.
    LengthMismatch { n_source: usize, n_load: usize },
    /// Harmonic order h × f_fundamental is outside the swept range.
    HarmonicOutOfRange { order: u32, frequency_hz: f64 },
}

impl std::fmt::Display for SpectroscopyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::BusIndexOutOfRange { bus_idx, n_buses } => {
                write!(f, "bus index {bus_idx} out of range (n_buses={n_buses})")
            }
            Self::SingularAdmittance { bus_idx } => {
                write!(f, "Y-bus diagonal is zero at bus {bus_idx}")
            }
            Self::InsufficientPoints { n_points } => {
                write!(f, "need ≥3 points for resonance detection, got {n_points}")
            }
            Self::BusMismatch { pre_bus, post_bus } => {
                write!(
                    f,
                    "pre-spectrum bus {pre_bus} ≠ post-spectrum bus {post_bus}"
                )
            }
            Self::LengthMismatch { n_source, n_load } => {
                write!(
                    f,
                    "source spectrum length {n_source} ≠ load spectrum length {n_load}"
                )
            }
            Self::HarmonicOutOfRange {
                order,
                frequency_hz,
            } => {
                write!(
                    f,
                    "harmonic order {order} at {frequency_hz:.1} Hz is outside spectrum range"
                )
            }
        }
    }
}

impl std::error::Error for SpectroscopyError {}

// ─── main analyser ────────────────────────────────────────────────────────────

/// Grid Impedance Analyser — entry point for all GIS computations.
pub struct ImpedanceAnalyzer {
    /// Stability assessment thresholds.
    pub config: StabilityAssessmentConfig,
    /// Accumulated spectra (from sweeps or external measurements).
    pub spectra: Vec<ImpedanceSpectrum>,
}

impl ImpedanceAnalyzer {
    /// Create a new analyser with the given configuration.
    pub fn new(config: StabilityAssessmentConfig) -> Self {
        Self {
            config,
            spectra: Vec::new(),
        }
    }

    /// Create a new analyser with default thresholds.
    pub fn with_defaults() -> Self {
        Self::new(StabilityAssessmentConfig::default())
    }
}

impl Default for ImpedanceAnalyzer {
    fn default() -> Self {
        Self::new(StabilityAssessmentConfig::default())
    }
}

impl ImpedanceAnalyzer {
    // ── 1. single-frequency bus impedance ─────────────────────────────────────

    /// Compute the driving-point impedance at `bus_idx` for `frequency_hz`.
    ///
    /// The Y-bus is a frequency-domain representation; each diagonal element
    /// Y_ii = Σ admittances connected to bus i.  The driving-point impedance
    /// is Z_ii ≈ 1 / Y_ii (exact for the diagonal of Z_bus = Y_bus⁻¹ when the
    /// off-diagonals are small compared with the diagonal, which holds for the
    /// simplified model used here).
    ///
    /// # Arguments
    /// * `y_bus`        — slice of (n × n) row-major complex admittance matrix entries
    /// * `n_buses`      — matrix dimension n
    /// * `bus_idx`      — target bus (0-indexed)
    /// * `frequency_hz` — evaluation frequency \[Hz\]
    ///
    /// # Errors
    /// Returns [`SpectroscopyError::BusIndexOutOfRange`] or
    /// [`SpectroscopyError::SingularAdmittance`] on invalid inputs.
    pub fn compute_bus_impedance(
        y_bus: &[Complex<f64>],
        n_buses: usize,
        bus_idx: usize,
        frequency_hz: f64,
    ) -> Result<Complex<f64>, SpectroscopyError> {
        if bus_idx >= n_buses {
            return Err(SpectroscopyError::BusIndexOutOfRange { bus_idx, n_buses });
        }

        // Diagonal element: Y[bus_idx][bus_idx]
        let diag_idx = bus_idx * n_buses + bus_idx;
        let y_diag = if diag_idx < y_bus.len() {
            y_bus[diag_idx]
        } else {
            return Err(SpectroscopyError::BusIndexOutOfRange { bus_idx, n_buses });
        };

        // Apply frequency scaling: capacitive admittances scale with jω, inductive
        // admittances scale with 1/(jω).  We use the base-frequency Y-bus supplied
        // by the caller and apply a first-order frequency correction to the
        // imaginary (susceptive) part only.
        //
        //   B_scaled = B_base × (f / f_base)
        //
        // where f_base is FUNDAMENTAL_HZ.  This is the standard single-frequency
        // correction for a lumped-parameter π-model.
        let f_ratio = frequency_hz / FUNDAMENTAL_HZ;
        let y_scaled = Complex::new(y_diag.re, y_diag.im * f_ratio);

        // Guard against singular admittance.
        let mag = y_scaled.norm();
        if mag < 1e-12 {
            return Err(SpectroscopyError::SingularAdmittance { bus_idx });
        }

        Ok(Complex::new(1.0, 0.0) / y_scaled)
    }

    // ── 2. frequency sweep ────────────────────────────────────────────────────

    /// Sweep the bus impedance over a logarithmically spaced frequency range.
    ///
    /// # Arguments
    /// * `y_bus`   — row-major n×n complex admittance matrix
    /// * `n_buses` — matrix dimension
    /// * `bus_idx` — target bus
    /// * `f_start` — start frequency \[Hz\]
    /// * `f_stop`  — stop frequency  \[Hz\]
    /// * `n_points`— number of frequency samples (must be ≥ 2)
    ///
    /// # Errors
    /// Propagates [`SpectroscopyError`] from [`Self::compute_bus_impedance`].
    pub fn sweep_impedance(
        &mut self,
        y_bus: &[Complex<f64>],
        n_buses: usize,
        bus_idx: usize,
        f_start: f64,
        f_stop: f64,
        n_points: usize,
    ) -> Result<ImpedanceSpectrum, SpectroscopyError> {
        let n = n_points.max(2);
        let log_start = f_start.ln();
        let log_stop = f_stop.ln();
        let step = (log_stop - log_start) / ((n - 1) as f64);

        let mut points = Vec::with_capacity(n);
        for i in 0..n {
            let freq = (log_start + step * (i as f64)).exp();
            let z = Self::compute_bus_impedance(y_bus, n_buses, bus_idx, freq)?;
            points.push(FrequencyPoint {
                frequency_hz: freq,
                impedance: z,
            });
        }

        let spectrum = ImpedanceSpectrum {
            points,
            bus_id: bus_idx,
            measurement_method: MeasurementMethod::Estimation,
            timestamp: 0.0,
        };

        self.spectra.push(spectrum.clone());
        Ok(spectrum)
    }

    // ── 3. resonance identification ───────────────────────────────────────────

    /// Identify resonance points in an impedance spectrum.
    ///
    /// Parallel resonances (|Z| local maximum) and series resonances (|Z|
    /// local minimum) are found by simple 3-point comparison.  The quality
    /// factor is estimated from the 3 dB bandwidth: Q = f_res / Δf₃dB.
    ///
    /// # Errors
    /// Returns [`SpectroscopyError::InsufficientPoints`] if the spectrum has
    /// fewer than 3 points.
    pub fn identify_resonances(
        spectrum: &ImpedanceSpectrum,
    ) -> Result<Vec<ResonancePoint>, SpectroscopyError> {
        let n = spectrum.points.len();
        if n < 3 {
            return Err(SpectroscopyError::InsufficientPoints { n_points: n });
        }

        let mags: Vec<f64> = spectrum.points.iter().map(|p| p.impedance.norm()).collect();
        let freqs: Vec<f64> = spectrum.points.iter().map(|p| p.frequency_hz).collect();

        let mut resonances = Vec::new();

        for i in 1..(n - 1) {
            let prev = mags[i - 1];
            let curr = mags[i];
            let next = mags[i + 1];

            let is_max = curr > prev && curr > next;
            let is_min = curr < prev && curr < next;

            if !is_max && !is_min {
                continue;
            }

            let res_type = if is_max {
                ResonanceType::ParallelResonance
            } else {
                ResonanceType::SeriesResonance
            };

            // Estimate 3 dB bandwidth for Q computation.
            let target = if is_max {
                curr / 10.0_f64.powf(3.0 / 20.0) // −3 dB
            } else {
                curr * 10.0_f64.powf(3.0 / 20.0) // +3 dB for minima
            };

            // Walk left from peak.
            let f_lower = find_crossing_left(&freqs, &mags, i, target);
            // Walk right from peak.
            let f_upper = find_crossing_right(&freqs, &mags, i, target);

            let bandwidth = (f_upper - f_lower).abs().max(1e-6);
            let quality_factor = freqs[i] / bandwidth;

            resonances.push(ResonancePoint {
                frequency_hz: freqs[i],
                quality_factor,
                peak_impedance_pu: curr,
                resonance_type: res_type,
            });
        }

        Ok(resonances)
    }

    // ── 4. harmonic impedance interpolation ───────────────────────────────────

    /// Return the complex impedance interpolated at `harmonic_order × f_fundamental`.
    ///
    /// Linear interpolation in the log-frequency domain between adjacent
    /// spectrum samples.
    ///
    /// # Errors
    /// Returns [`SpectroscopyError::HarmonicOutOfRange`] if the target
    /// frequency lies outside the spectrum range.
    pub fn harmonic_impedance_at(
        spectrum: &ImpedanceSpectrum,
        harmonic_order: u32,
    ) -> Result<Complex<f64>, SpectroscopyError> {
        let target_hz = (harmonic_order as f64) * FUNDAMENTAL_HZ;
        interpolate_spectrum(spectrum, target_hz).ok_or(SpectroscopyError::HarmonicOutOfRange {
            order: harmonic_order,
            frequency_hz: target_hz,
        })
    }

    // ── 5. passive inter-modulation check ─────────────────────────────────────

    /// Compute harmonic voltage products V_h = I_h × Z(h × f1) and flag those
    /// exceeding the IEEE 519 individual voltage distortion limit of 3 %.
    ///
    /// `harmonic_currents` is a slice of `(order, magnitude_pu)` pairs.
    /// `v_nominal` is the nominal bus voltage \[pu\] (typically 1.0).
    pub fn passive_intermodulation_check(
        spectrum: &ImpedanceSpectrum,
        harmonic_currents: &[(u32, f64)],
        v_nominal: f64,
    ) -> Vec<IntermodProduct> {
        harmonic_currents
            .iter()
            .filter_map(|&(order, i_mag)| {
                let target_hz = (order as f64) * FUNDAMENTAL_HZ;
                let z = interpolate_spectrum(spectrum, target_hz)?;
                let v_h = i_mag * z.norm(); // V_h = I_h × |Z|
                let distortion_pct = if v_nominal.abs() > 1e-12 {
                    v_h / v_nominal * 100.0
                } else {
                    0.0
                };
                Some(IntermodProduct {
                    harmonic_order: order,
                    voltage_distortion_pct: distortion_pct,
                    exceeds_limit: distortion_pct > 3.0,
                    impedance_pu: z.norm(),
                })
            })
            .collect()
    }

    // ── 6. impedance-based stability margin ───────────────────────────────────

    /// Compute Nyquist-based stability margins from source and load impedances.
    ///
    /// The minor-loop gain is T(jω) = Z_s / Z_L.  The system is stable (by
    /// the Middlebrook / impedance-ratio criterion) when the Nyquist plot of
    /// T does not encircle −1.
    ///
    /// * Gain margin  = 20 log₁₀(1 / |T|) at the frequency where ∠T = −180°.
    /// * Phase margin = ∠T + 180° at the frequency where |T| = 1.
    ///
    /// `z_source` and `z_load` are ordered slices of (frequency_hz, Z) pairs
    /// with matching lengths.
    ///
    /// # Errors
    /// Returns [`SpectroscopyError::LengthMismatch`] if the slices differ.
    pub fn impedance_stability_margin(
        &self,
        z_source: &[(f64, Complex<f64>)],
        z_load: &[(f64, Complex<f64>)],
    ) -> Result<StabilityMargin, SpectroscopyError> {
        let n = z_source.len();
        if n != z_load.len() {
            return Err(SpectroscopyError::LengthMismatch {
                n_source: n,
                n_load: z_load.len(),
            });
        }

        // Compute T(jω) = Z_s / Z_L at each frequency.
        let t_vals: Vec<(f64, Complex<f64>)> = z_source
            .iter()
            .zip(z_load.iter())
            .filter_map(|((f_s, zs), (_f_l, zl))| {
                if zl.norm() < 1e-12 {
                    None
                } else {
                    Some((*f_s, zs / zl))
                }
            })
            .collect();

        if t_vals.is_empty() {
            // Degenerate: return safe margins.
            return Ok(StabilityMargin {
                gain_margin_db: f64::INFINITY,
                phase_margin_deg: 90.0,
                stable: true,
                worst_frequency_hz: 0.0,
            });
        }

        // ── gain margin: find crossing of ∠T ≈ −180° ────────────────────────
        let mut gain_margin_db = f64::INFINITY;
        let mut gm_freq = t_vals[0].0;

        for window in t_vals.windows(2) {
            let (f0, t0) = window[0];
            let (f1, t1) = window[1];
            let ph0 = t0.arg().to_degrees() + 180.0; // deviation from −180
            let ph1 = t1.arg().to_degrees() + 180.0;

            // Sign change → crossing of −180°.
            if ph0 * ph1 <= 0.0 {
                let alpha = ph0.abs() / (ph0.abs() + ph1.abs() + 1e-30);
                let f_cross = f0 + alpha * (f1 - f0);
                let t_cross = lerp_complex(t0, t1, alpha);
                let gm = -20.0 * t_cross.norm().log10();
                if gm.abs() < gain_margin_db.abs() {
                    gain_margin_db = gm;
                    gm_freq = f_cross;
                }
            }
        }

        // ── phase margin: find crossing of |T| = 1 ───────────────────────────
        let mut phase_margin_deg = 180.0_f64;
        let mut pm_freq = t_vals[0].0;

        for window in t_vals.windows(2) {
            let (f0, t0) = window[0];
            let (f1, t1) = window[1];
            let m0 = t0.norm() - 1.0;
            let m1 = t1.norm() - 1.0;

            if m0 * m1 <= 0.0 {
                let alpha = m0.abs() / (m0.abs() + m1.abs() + 1e-30);
                let f_cross = f0 + alpha * (f1 - f0);
                let t_cross = lerp_complex(t0, t1, alpha);
                let pm = t_cross.arg().to_degrees() + 180.0;
                if pm < phase_margin_deg {
                    phase_margin_deg = pm;
                    pm_freq = f_cross;
                }
            }
        }

        let worst_frequency_hz = if gain_margin_db < phase_margin_deg {
            gm_freq
        } else {
            pm_freq
        };

        let stable = gain_margin_db >= self.config.gain_margin_threshold_db
            && phase_margin_deg >= self.config.phase_margin_threshold_deg;

        Ok(StabilityMargin {
            gain_margin_db,
            phase_margin_deg,
            stable,
            worst_frequency_hz,
        })
    }

    // ── 7. pre/post comparison ────────────────────────────────────────────────

    /// Compare two spectra (before and after a grid modification) and
    /// produce a change summary.
    ///
    /// A resonance is considered "the same" when its frequency is within 5 %
    /// of an existing resonance in the other spectrum.
    ///
    /// # Errors
    /// Returns [`SpectroscopyError::BusMismatch`] if the bus IDs differ,
    /// or [`SpectroscopyError::InsufficientPoints`] from resonance detection.
    pub fn compare_pre_post(
        &self,
        spectrum_pre: &ImpedanceSpectrum,
        spectrum_post: &ImpedanceSpectrum,
    ) -> Result<SpectrumComparisonReport, SpectroscopyError> {
        if spectrum_pre.bus_id != spectrum_post.bus_id {
            return Err(SpectroscopyError::BusMismatch {
                pre_bus: spectrum_pre.bus_id,
                post_bus: spectrum_post.bus_id,
            });
        }

        let res_pre = Self::identify_resonances(spectrum_pre)?;
        let res_post = Self::identify_resonances(spectrum_post)?;

        let freqs_pre: Vec<f64> = res_pre.iter().map(|r| r.frequency_hz).collect();
        let freqs_post: Vec<f64> = res_post.iter().map(|r| r.frequency_hz).collect();

        // Resonances in post not matched in pre → added.
        let resonances_added: Vec<f64> = freqs_post
            .iter()
            .filter(|&&fp| {
                !freqs_pre
                    .iter()
                    .any(|&fr| (fp - fr).abs() / fr.max(1e-6) < 0.05)
            })
            .copied()
            .collect();

        // Resonances in pre not matched in post → removed.
        let resonances_removed: Vec<f64> = freqs_pre
            .iter()
            .filter(|&&fr| {
                !freqs_post
                    .iter()
                    .any(|&fp| (fp - fr).abs() / fr.max(1e-6) < 0.05)
            })
            .copied()
            .collect();

        let peak_pre = spectrum_pre
            .points
            .iter()
            .map(|p| p.impedance.norm())
            .fold(0.0_f64, f64::max);
        let peak_post = spectrum_post
            .points
            .iter()
            .map(|p| p.impedance.norm())
            .fold(0.0_f64, f64::max);

        let peak_impedance_change_pu = peak_post - peak_pre;

        // Degradation: new resonance appeared, or peak impedance grew > 10 %.
        let stability_degraded = !resonances_added.is_empty()
            || (peak_pre > 1e-9 && peak_impedance_change_pu / peak_pre > 0.10);

        Ok(SpectrumComparisonReport {
            resonances_added,
            resonances_removed,
            peak_impedance_change_pu,
            stability_degraded,
        })
    }

    // ── 8. grid strength assessment ───────────────────────────────────────────

    /// Characterise the grid strength at the measurement bus.
    ///
    /// # Arguments
    /// * `spectrum`         — bus impedance spectrum
    /// * `short_circuit_mva`— three-phase short-circuit apparent power \[MVA\]
    /// * `p_rated_mw`       — rated active power of the connected IBR \[MW\]
    ///
    /// # Returns
    /// A [`GridStrengthReport`] with SCR, X/R, impedance angle, and a
    /// human-readable strength classification.
    pub fn grid_strength_assessment(
        spectrum: &ImpedanceSpectrum,
        short_circuit_mva: f64,
        p_rated_mw: f64,
    ) -> GridStrengthReport {
        // Impedance at the fundamental frequency.
        let z1 = interpolate_spectrum(spectrum, FUNDAMENTAL_HZ).unwrap_or(Complex::new(1e-3, 1e-2));

        let r = z1.re;
        let x = z1.im;

        let x_r_ratio = if r.abs() > 1e-12 { x / r } else { f64::MAX };
        let impedance_angle_deg = x.atan2(r).to_degrees();

        let p = if p_rated_mw.abs() < 1e-9 {
            1.0
        } else {
            p_rated_mw
        };
        let scr = short_circuit_mva / p;

        let grid_strength = if scr >= 3.0 {
            "Strong".to_string()
        } else if scr >= 1.5 {
            "Moderate".to_string()
        } else {
            "Weak".to_string()
        };

        GridStrengthReport {
            scr,
            x_r_ratio,
            impedance_angle_deg,
            grid_strength,
        }
    }
}

// ─── helper functions ─────────────────────────────────────────────────────────

/// Linear interpolation of the complex impedance at `target_hz` within `spectrum`.
/// Returns `None` if the target is outside the swept range.
fn interpolate_spectrum(spectrum: &ImpedanceSpectrum, target_hz: f64) -> Option<Complex<f64>> {
    let pts = &spectrum.points;
    if pts.is_empty() {
        return None;
    }

    let f_min = pts.first()?.frequency_hz;
    let f_max = pts.last()?.frequency_hz;

    if target_hz < f_min || target_hz > f_max {
        return None;
    }

    // Binary-search for the left bracket.
    let idx = pts
        .binary_search_by(|p| {
            p.frequency_hz
                .partial_cmp(&target_hz)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .unwrap_or_else(|i| i);

    if idx == 0 {
        return Some(pts[0].impedance);
    }
    if idx >= pts.len() {
        return Some(pts[pts.len() - 1].impedance);
    }

    let p0 = &pts[idx - 1];
    let p1 = &pts[idx];

    let span = p1.frequency_hz - p0.frequency_hz;
    let alpha = if span.abs() > 1e-15 {
        (target_hz - p0.frequency_hz) / span
    } else {
        0.0
    };

    Some(lerp_complex(p0.impedance, p1.impedance, alpha))
}

/// Linear interpolation between two complex numbers.
#[inline]
fn lerp_complex(a: Complex<f64>, b: Complex<f64>, t: f64) -> Complex<f64> {
    a * (1.0 - t) + b * t
}

/// Walk left from `peak_idx` in `mags` until |Z| crosses `target`.
/// Returns the interpolated frequency.
fn find_crossing_left(freqs: &[f64], mags: &[f64], peak_idx: usize, target: f64) -> f64 {
    for i in (0..peak_idx).rev() {
        let curr_above = mags[peak_idx] >= target;
        let left_below = if curr_above {
            mags[i] <= target
        } else {
            mags[i] >= target
        };
        if left_below {
            let m0 = mags[i];
            let m1 = mags[i + 1];
            let span = m1 - m0;
            let alpha = if span.abs() > 1e-15 {
                (target - m0) / span
            } else {
                0.5
            };
            return freqs[i] + alpha * (freqs[i + 1] - freqs[i]);
        }
    }
    freqs[0]
}

/// Walk right from `peak_idx` in `mags` until |Z| crosses `target`.
/// Returns the interpolated frequency.
fn find_crossing_right(freqs: &[f64], mags: &[f64], peak_idx: usize, target: f64) -> f64 {
    let n = freqs.len();
    for i in (peak_idx + 1)..n {
        let curr_above = mags[peak_idx] >= target;
        let right_below = if curr_above {
            mags[i] <= target
        } else {
            mags[i] >= target
        };
        if right_below {
            let m0 = mags[i - 1];
            let m1 = mags[i];
            let span = m1 - m0;
            let alpha = if span.abs() > 1e-15 {
                (target - m0) / span
            } else {
                0.5
            };
            return freqs[i - 1] + alpha * (freqs[i] - freqs[i - 1]);
        }
    }
    freqs[n - 1]
}

/// Build a minimal dense Y-bus row-major matrix from branch data.
///
/// `branches` is a slice of `(from, to, y_series, b_shunt)` tuples where
/// admittances are in per-unit.  Returns a flat `n_buses × n_buses` vector.
pub fn build_y_bus_dense(
    n_buses: usize,
    branches: &[(usize, usize, Complex<f64>, f64)],
) -> Vec<Complex<f64>> {
    let mut y = vec![Complex::new(0.0, 0.0); n_buses * n_buses];

    for &(from, to, y_series, b_shunt) in branches {
        if from >= n_buses || to >= n_buses {
            continue;
        }
        let y_half_shunt = Complex::new(0.0, b_shunt / 2.0);

        // Diagonal contributions.
        y[from * n_buses + from] += y_series + y_half_shunt;
        y[to * n_buses + to] += y_series + y_half_shunt;

        // Off-diagonal.
        y[from * n_buses + to] -= y_series;
        y[to * n_buses + from] -= y_series;
    }

    y
}

// ─── tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: build a trivial 3-bus dense Y-bus.
    fn three_bus_y() -> (Vec<Complex<f64>>, usize) {
        let n = 3;
        // Branch 0→1: y = 0.0 + j10.0, b_shunt = 0.02
        // Branch 1→2: y = 0.0 + j8.0,  b_shunt = 0.02
        let branches = vec![
            (0usize, 1usize, Complex::new(0.0, 10.0), 0.02_f64),
            (1, 2, Complex::new(0.0, 8.0), 0.02),
        ];
        (build_y_bus_dense(n, &branches), n)
    }

    // ── test 1: capacitive load → negative imaginary impedance part ───────────
    #[test]
    fn test_capacitive_impedance_negative_imag() {
        // A purely capacitive shunt branch at bus 0 produces a positive
        // imaginary susceptance B > 0, hence Z = 1/(jB) has Im(Z) < 0.
        let n = 2;
        let branches = vec![(0usize, 1usize, Complex::new(0.0, 0.0), 0.5_f64)]; // large cap
        let y = build_y_bus_dense(n, &branches);
        // Bus 0 diagonal = j*0.25 (half shunt only)
        let z = ImpedanceAnalyzer::compute_bus_impedance(&y, n, 0, FUNDAMENTAL_HZ)
            .expect("compute_bus_impedance should succeed");
        // Im(Z) < 0 for a capacitive bus (positive B).
        assert!(
            z.im < 0.0,
            "expected Im(Z) < 0 for capacitive bus, got {z:?}"
        );
    }

    // ── test 2: sweep returns exactly n_points ────────────────────────────────
    #[test]
    fn test_sweep_returns_n_points() {
        let (y, n) = three_bus_y();
        let mut analyser = ImpedanceAnalyzer::default();
        let spectrum = analyser
            .sweep_impedance(&y, n, 1, 10.0, 2500.0, 100)
            .expect("sweep should succeed");
        assert_eq!(spectrum.points.len(), 100, "expected 100 frequency points");
    }

    // ── test 3: resonance detection finds a parallel resonance ───────────────
    #[test]
    fn test_resonance_detection_parallel() {
        // Construct a synthetic spectrum with an obvious peak at 250 Hz.
        let mut points = Vec::new();
        for i in 0..201 {
            let f = 50.0 + (i as f64) * 5.0; // 50 … 1050 Hz
                                             // Gaussian peak centred at 250 Hz.
            let peak = 5.0 * (-((f - 250.0) / 30.0).powi(2)).exp();
            let z = Complex::new(0.05 + peak, 0.01);
            points.push(FrequencyPoint {
                frequency_hz: f,
                impedance: z,
            });
        }
        let spectrum = ImpedanceSpectrum {
            points,
            bus_id: 0,
            measurement_method: MeasurementMethod::Estimation,
            timestamp: 0.0,
        };
        let resonances =
            ImpedanceAnalyzer::identify_resonances(&spectrum).expect("should detect resonances");
        let has_parallel = resonances
            .iter()
            .any(|r| r.resonance_type == ResonanceType::ParallelResonance);
        assert!(has_parallel, "expected at least one parallel resonance");
    }

    // ── test 4: harmonic interpolation at 5×f1 ───────────────────────────────
    #[test]
    fn test_harmonic_impedance_at_5th() {
        let (y, n) = three_bus_y();
        let mut analyser = ImpedanceAnalyzer::default();
        let spectrum = analyser
            .sweep_impedance(&y, n, 1, 10.0, 5000.0, 200)
            .expect("sweep");

        // 5th harmonic = 250 Hz — should be within [10, 5000] Hz range.
        let z5 = ImpedanceAnalyzer::harmonic_impedance_at(&spectrum, 5)
            .expect("5th harmonic should be in range");
        assert!(
            z5.norm().is_finite() && z5.norm() > 0.0,
            "harmonic impedance should be finite and positive"
        );
    }

    // ── test 5: stability margin — well-separated Z_s ≪ Z_L → GM > 6 dB ────
    #[test]
    fn test_stability_margin_stable_system() {
        // Z_s = 0.01 + j0.05  (small source impedance)
        // Z_L = 1.0  + j0.5   (large load impedance)
        // → T = Z_s / Z_L is small in magnitude → large gain margin.
        let n = 50;
        let freqs: Vec<f64> = (0..n).map(|i| 50.0 + (i as f64) * 10.0).collect();
        let z_source: Vec<(f64, Complex<f64>)> = freqs
            .iter()
            .map(|&f| (f, Complex::new(0.01, 0.05)))
            .collect();
        let z_load: Vec<(f64, Complex<f64>)> =
            freqs.iter().map(|&f| (f, Complex::new(1.0, 0.5))).collect();

        let analyser = ImpedanceAnalyzer::default();
        let margin = analyser
            .impedance_stability_margin(&z_source, &z_load)
            .expect("margin computation should succeed");

        assert!(
            margin.gain_margin_db > 6.0,
            "expected GM > 6 dB, got {:.2} dB",
            margin.gain_margin_db
        );
        assert!(margin.stable, "system should be stable");
    }

    // ── test 6: intermodulation — large harmonic current → exceeds_limit ─────
    #[test]
    fn test_intermodulation_exceeds_limit() {
        // Build a spectrum with large impedance at 5th harmonic (250 Hz).
        let (y, n) = three_bus_y();
        let mut analyser = ImpedanceAnalyzer::default();
        let spectrum = analyser
            .sweep_impedance(&y, n, 1, 10.0, 5000.0, 300)
            .expect("sweep");

        // Inject 20 % of rated current at 5th harmonic; with any non-zero |Z|
        // the distortion voltage should be detectable.  We set V_nominal = 0.01
        // (very small) to guarantee distortion > 3 %.
        let harmonic_currents = vec![(5u32, 0.5_f64)]; // 50 % harmonic current
        let products = ImpedanceAnalyzer::passive_intermodulation_check(
            &spectrum,
            &harmonic_currents,
            0.01, // tiny nominal voltage → large %
        );

        assert!(
            !products.is_empty(),
            "should produce at least one intermodulation product"
        );
        let exceeds = products.iter().any(|p| p.exceeds_limit);
        assert!(
            exceeds,
            "with V_nominal=0.01 and I_h=0.5 the limit must be exceeded"
        );
    }

    // ── test 7: grid strength — high Ssc → SCR > 3 → "Strong" ───────────────
    #[test]
    fn test_grid_strength_strong() {
        let (y, n) = three_bus_y();
        let mut analyser = ImpedanceAnalyzer::default();
        let spectrum = analyser
            .sweep_impedance(&y, n, 1, 10.0, 1000.0, 50)
            .expect("sweep");

        // Ssc = 1000 MVA, P_rated = 100 MW → SCR = 10 → "Strong"
        let report = ImpedanceAnalyzer::grid_strength_assessment(&spectrum, 1000.0, 100.0);
        assert!(report.scr > 3.0, "expected SCR > 3, got {:.2}", report.scr);
        assert_eq!(report.grid_strength, "Strong");
    }

    // ── test 8: comparison — added shunt capacitor → series resonance added ──
    #[test]
    fn test_comparison_added_filter() {
        // Pre: smooth inductive spectrum.
        let mut pre_pts = Vec::new();
        for i in 0..101 {
            let f = 50.0 + (i as f64) * 10.0;
            pre_pts.push(FrequencyPoint {
                frequency_hz: f,
                impedance: Complex::new(0.05, 0.1 * f / 50.0),
            });
        }
        let spectrum_pre = ImpedanceSpectrum {
            points: pre_pts,
            bus_id: 2,
            measurement_method: MeasurementMethod::Estimation,
            timestamp: 0.0,
        };

        // Post: same spectrum but with a series dip (series resonance) at 350 Hz.
        let mut post_pts = Vec::new();
        for i in 0..101 {
            let f = 50.0 + (i as f64) * 10.0;
            let dip = 0.04 * (-((f - 350.0) / 20.0).powi(2)).exp();
            post_pts.push(FrequencyPoint {
                frequency_hz: f,
                impedance: Complex::new(0.05, 0.1 * f / 50.0 - dip * 50.0),
            });
        }
        let spectrum_post = ImpedanceSpectrum {
            points: post_pts,
            bus_id: 2,
            measurement_method: MeasurementMethod::Estimation,
            timestamp: 1.0,
        };

        let analyser = ImpedanceAnalyzer::default();
        let report = analyser
            .compare_pre_post(&spectrum_pre, &spectrum_post)
            .expect("comparison should succeed");

        // Either a new resonance was added or stability was flagged as degraded.
        // The large dip at 350 Hz should show up as a series resonance in post.
        let resonances_changed =
            !report.resonances_added.is_empty() || !report.resonances_removed.is_empty();
        assert!(
            resonances_changed || report.stability_degraded,
            "expected the filter addition to change resonance pattern"
        );
    }

    // ── test 9: bus index out of range returns error ───────────────────────────
    #[test]
    fn test_bus_index_out_of_range() {
        let y = vec![Complex::new(1.0, 0.0); 4]; // 2×2 matrix
        let result = ImpedanceAnalyzer::compute_bus_impedance(&y, 2, 5, 50.0);
        assert!(
            matches!(result, Err(SpectroscopyError::BusIndexOutOfRange { .. })),
            "expected BusIndexOutOfRange error"
        );
    }

    // ── test 10: harmonic out of range returns error ──────────────────────────
    #[test]
    fn test_harmonic_out_of_range() {
        let pts: Vec<FrequencyPoint> = (0..10)
            .map(|i| FrequencyPoint {
                frequency_hz: 100.0 + (i as f64) * 10.0,
                impedance: Complex::new(0.1, 0.0),
            })
            .collect();
        let spectrum = ImpedanceSpectrum {
            points: pts,
            bus_id: 0,
            measurement_method: MeasurementMethod::Estimation,
            timestamp: 0.0,
        };
        // 1st harmonic = 50 Hz, below spectrum start of 100 Hz.
        let result = ImpedanceAnalyzer::harmonic_impedance_at(&spectrum, 1);
        assert!(
            matches!(result, Err(SpectroscopyError::HarmonicOutOfRange { .. })),
            "expected HarmonicOutOfRange error"
        );
    }

    // ── test 11: frequency sweep is logarithmically spaced ───────────────────
    #[test]
    fn test_sweep_log_spacing() {
        let (y, n) = three_bus_y();
        let mut analyser = ImpedanceAnalyzer::default();
        let spectrum = analyser
            .sweep_impedance(&y, n, 0, 10.0, 10_000.0, 50)
            .expect("sweep");
        let pts = &spectrum.points;
        // Log-ratios between consecutive points should be approximately equal.
        let ratios: Vec<f64> = pts
            .windows(2)
            .map(|w| (w[1].frequency_hz / w[0].frequency_hz).ln())
            .collect();
        let mean_ratio = ratios.iter().copied().sum::<f64>() / (ratios.len() as f64);
        for r in &ratios {
            assert!(
                (r - mean_ratio).abs() < 1e-10,
                "log-ratio not constant: {r} vs mean {mean_ratio}"
            );
        }
    }
}
