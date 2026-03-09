/// PMU (Phasor Measurement Unit) synchrophasor data processing.
///
/// Implements IEEE C37.118.2 data structures, quality processing,
/// symmetrical components, Park (dq0) transform, and Prony modal analysis
/// for inter-area oscillation detection.
///
/// # References
/// - IEEE Std C37.118.2-2011, Synchrophasor Data Transfer
/// - Anderson & Fouad, "Power System Control and Stability", 2nd ed.
/// - Hauer et al., "Initial Results in Prony Analysis", IEEE TPWRS 1990
use crate::error::{OxiGridError, Result};
use num_complex::Complex64;
use std::f64::consts::PI;

// ─────────────────────────────────────────────────────────────────────────────
// Core phasor types
// ─────────────────────────────────────────────────────────────────────────────

/// A single phasor measurement (polar form): magnitude and angle in radians.
///
/// Magnitudes are typically in per-unit \[pu\] for voltages or \[kA\] for currents.
/// Angles are relative to the UTC-synchronised reference.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Phasor {
    /// Magnitude [pu or kA]
    pub magnitude: f64,
    /// Angle relative to reference \[radians\]
    pub angle_rad: f64,
}

impl Phasor {
    /// Create a new phasor from polar coordinates.
    pub fn new(magnitude: f64, angle_rad: f64) -> Self {
        Self {
            magnitude,
            angle_rad,
        }
    }

    /// Convert to rectangular complex representation.
    pub fn to_complex(&self) -> Complex64 {
        Complex64::from_polar(self.magnitude, self.angle_rad)
    }

    /// Create a phasor from a rectangular complex number.
    pub fn from_complex(c: Complex64) -> Self {
        Self {
            magnitude: c.norm(),
            angle_rad: c.arg(),
        }
    }

    /// Magnitude in decibels: 20·log₁₀(|magnitude|).
    pub fn magnitude_db(&self) -> f64 {
        if self.magnitude <= 0.0 {
            f64::NEG_INFINITY
        } else {
            20.0 * self.magnitude.log10()
        }
    }

    /// True if the magnitude is within [lo, hi].
    pub fn in_range(&self, lo: f64, hi: f64) -> bool {
        self.magnitude >= lo && self.magnitude <= hi
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// PMU channel and configuration
// ─────────────────────────────────────────────────────────────────────────────

/// Metadata for one PMU measurement channel (voltage or current).
#[derive(Debug, Clone)]
pub struct PmuChannel {
    /// Human-readable name (e.g., "BUS1_VA")
    pub name: String,
    /// Associated bus index in the network model (voltage channels)
    pub bus_idx: Option<usize>,
    /// Associated branch index in the network model (current channels)
    pub branch_idx: Option<usize>,
    /// Engineering-unit scale factor applied to raw measurement
    pub scale_factor: f64,
    /// Phase label: 'A', 'B', 'C', or 'P' (positive-sequence)
    pub phase: char,
}

impl PmuChannel {
    /// Create a positive-sequence voltage channel mapped to a bus.
    pub fn voltage_positive_seq(name: &str, bus_idx: usize, scale_factor: f64) -> Self {
        Self {
            name: name.to_string(),
            bus_idx: Some(bus_idx),
            branch_idx: None,
            scale_factor,
            phase: 'P',
        }
    }

    /// Create a three-phase voltage channel for a given phase letter.
    pub fn voltage_phase(name: &str, bus_idx: usize, scale_factor: f64, phase: char) -> Self {
        Self {
            name: name.to_string(),
            bus_idx: Some(bus_idx),
            branch_idx: None,
            scale_factor,
            phase,
        }
    }

    /// Create a current channel mapped to a branch.
    pub fn current(name: &str, branch_idx: usize, scale_factor: f64, phase: char) -> Self {
        Self {
            name: name.to_string(),
            bus_idx: None,
            branch_idx: Some(branch_idx),
            scale_factor,
            phase,
        }
    }
}

/// PMU device configuration (channel layout and time base).
#[derive(Debug, Clone)]
pub struct PmuConfig {
    /// Station / substation name
    pub station_name: String,
    /// Unique PMU device identifier
    pub pmu_id: u16,
    /// Frames-per-second measurement rate (e.g., 50.0 or 60.0)
    pub data_rate: f64,
    /// Voltage measurement channels
    pub voltage_channels: Vec<PmuChannel>,
    /// Current measurement channels
    pub current_channels: Vec<PmuChannel>,
    /// Nominal power-system frequency \[Hz\] (50 or 60)
    pub nominal_freq: f64,
    /// Subdivisions of one second used by the TIME_BASE field
    pub time_base: u32,
}

impl PmuConfig {
    /// Create a default 50 fps / 50 Hz PMU configuration.
    pub fn new_50hz(station_name: &str, pmu_id: u16) -> Self {
        Self {
            station_name: station_name.to_string(),
            pmu_id,
            data_rate: 50.0,
            voltage_channels: Vec::new(),
            current_channels: Vec::new(),
            nominal_freq: 50.0,
            time_base: 1_000_000,
        }
    }

    /// Nominal period \[s\] = 1 / data_rate.
    pub fn nominal_period_s(&self) -> f64 {
        if self.data_rate > 0.0 {
            1.0 / self.data_rate
        } else {
            f64::INFINITY
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// PMU frame (one measurement instant)
// ─────────────────────────────────────────────────────────────────────────────

/// One PMU data frame corresponding to a single measurement instant.
#[derive(Debug, Clone)]
pub struct PmuFrame {
    /// UTC timestamp in microseconds since Unix epoch
    pub timestamp_us: i64,
    /// PMU device identifier (matches PmuConfig::pmu_id)
    pub pmu_id: u16,
    /// IEEE C37.118.2 STAT word (bit-field, 0 = OK)
    pub stat: u16,
    /// Phasor measurements (voltage channels first, then current channels)
    pub phasors: Vec<Phasor>,
    /// Estimated frequency \[Hz\]
    pub freq_hz: f64,
    /// Rate of change of frequency [Hz/s]
    pub rocof: f64,
    /// Analog channel values (engineering units)
    pub analogs: Vec<f64>,
    /// Digital (status) channel values
    pub digitals: Vec<bool>,
}

impl PmuFrame {
    /// True if the STAT field indicates no error conditions.
    pub fn is_ok(&self) -> bool {
        // STAT bit 15 = data error, bit 14 = PMU error, bit 13 = data modified
        self.stat & 0xE000 == 0
    }

    /// Timestamp in seconds since Unix epoch.
    pub fn timestamp_s(&self) -> f64 {
        self.timestamp_us as f64 * 1e-6
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Quality flags
// ─────────────────────────────────────────────────────────────────────────────

/// Per-frame data quality assessment.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PmuQuality {
    /// Frame is valid and consistent
    Good,
    /// Frame was reconstructed by linear interpolation
    Interpolated,
    /// Frame is absent (timestamp gap)
    Missing,
    /// STAT word indicates device or data error
    BadStatus,
    /// |Δf| > 2 Hz relative to previous frame
    FrequencyJump,
    /// |Δθ| > 30° (π/6 rad) on any phasor relative to previous frame
    PhaseJump,
    /// Timestamp gap > 2 × nominal period
    Latency,
}

// ─────────────────────────────────────────────────────────────────────────────
// PMU dataset
// ─────────────────────────────────────────────────────────────────────────────

/// Multi-PMU time-aligned dataset.
///
/// `frames[pmu_idx][time_idx]` gives the frame for PMU `pmu_idx` at time step
/// `time_idx`.  All PMUs are assumed to share the same (or aligned) time grid
/// after calling [`PmuDataset::synchronize`].
#[derive(Debug, Clone)]
pub struct PmuDataset {
    /// One configuration per PMU
    pub configs: Vec<PmuConfig>,
    /// `frames[pmu_idx][time_idx]`
    pub frames: Vec<Vec<PmuFrame>>,
    /// Dataset start timestamp \[μs\]
    pub t_start_us: i64,
    /// Dataset end timestamp \[μs\]
    pub t_end_us: i64,
}

impl PmuDataset {
    /// Create an empty dataset.
    pub fn new() -> Self {
        Self {
            configs: Vec::new(),
            frames: Vec::new(),
            t_start_us: 0,
            t_end_us: 0,
        }
    }

    /// Add a PMU stream.  Returns the PMU index.
    pub fn add_pmu(&mut self, config: PmuConfig, frames: Vec<PmuFrame>) -> usize {
        let idx = self.configs.len();
        // Update dataset time bounds.
        if let (Some(first), Some(last)) = (frames.first(), frames.last()) {
            if idx == 0 {
                self.t_start_us = first.timestamp_us;
                self.t_end_us = last.timestamp_us;
            } else {
                self.t_start_us = self.t_start_us.min(first.timestamp_us);
                self.t_end_us = self.t_end_us.max(last.timestamp_us);
            }
        }
        self.configs.push(config);
        self.frames.push(frames);
        idx
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Quality assessment
    // ─────────────────────────────────────────────────────────────────────────

    /// Compute per-frame quality flags for every PMU stream.
    ///
    /// Returns `quality[pmu_idx][time_idx]`.
    pub fn quality_flags(&self) -> Vec<Vec<PmuQuality>> {
        self.configs
            .iter()
            .enumerate()
            .map(|(pmu_idx, cfg)| {
                let frames = &self.frames[pmu_idx];
                let nominal_period_us = (cfg.nominal_period_s() * 1_000_000.0).round() as i64;

                frames
                    .iter()
                    .enumerate()
                    .map(|(t_idx, frame)| {
                        // 1. Bad STAT word
                        if !frame.is_ok() {
                            return PmuQuality::BadStatus;
                        }

                        if t_idx == 0 {
                            return PmuQuality::Good;
                        }

                        let prev = &frames[t_idx - 1];
                        let dt_us = frame.timestamp_us - prev.timestamp_us;

                        // 2. Latency: gap > 2× nominal period
                        if dt_us > 2 * nominal_period_us {
                            return PmuQuality::Latency;
                        }

                        // 3. Frequency jump |Δf| > 2 Hz
                        if (frame.freq_hz - prev.freq_hz).abs() > 2.0 {
                            return PmuQuality::FrequencyJump;
                        }

                        // 4. Phase jump |Δθ| > 30° on any phasor
                        let phase_jump_threshold = PI / 6.0; // 30°
                        let has_phase_jump =
                            frame
                                .phasors
                                .iter()
                                .zip(prev.phasors.iter())
                                .any(|(cur, prv)| {
                                    let mut delta = (cur.angle_rad - prv.angle_rad).abs();
                                    // wrap to [0, π]
                                    if delta > PI {
                                        delta = 2.0 * PI - delta;
                                    }
                                    delta > phase_jump_threshold
                                });
                        if has_phase_jump {
                            return PmuQuality::PhaseJump;
                        }

                        PmuQuality::Good
                    })
                    .collect()
            })
            .collect()
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Missing frame interpolation
    // ─────────────────────────────────────────────────────────────────────────

    /// Fill gaps in each PMU stream by linear interpolation.
    ///
    /// A gap is detected when the timestamp difference between consecutive
    /// frames exceeds 1.5× the nominal period.  The function inserts synthetic
    /// frames and returns the total number of frames interpolated.
    pub fn interpolate_missing(&mut self) -> usize {
        let mut total_interpolated = 0usize;

        for pmu_idx in 0..self.configs.len() {
            let nominal_period_us = {
                let cfg = &self.configs[pmu_idx];
                (cfg.nominal_period_s() * 1_000_000.0).round() as i64
            };
            let gap_threshold_us = (nominal_period_us as f64 * 1.5) as i64;

            let mut new_frames: Vec<PmuFrame> = Vec::new();
            let src = std::mem::take(&mut self.frames[pmu_idx]);

            let mut iter = src.into_iter().peekable();
            while let Some(frame) = iter.next() {
                new_frames.push(frame.clone());
                if let Some(next_frame) = iter.peek() {
                    let dt_us = next_frame.timestamp_us - frame.timestamp_us;
                    if dt_us > gap_threshold_us {
                        // Insert interpolated frames at nominal spacing.
                        let mut t = frame.timestamp_us + nominal_period_us;
                        while t < next_frame.timestamp_us - nominal_period_us / 2 {
                            let alpha = (t - frame.timestamp_us) as f64 / dt_us as f64;
                            let interp = interpolate_frames(&frame, next_frame, alpha, t);
                            new_frames.push(interp);
                            total_interpolated += 1;
                            t += nominal_period_us;
                        }
                    }
                }
            }

            self.frames[pmu_idx] = new_frames;
        }

        total_interpolated
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Multi-PMU synchronisation
    // ─────────────────────────────────────────────────────────────────────────

    /// Resample all PMU streams to a common time grid at `target_rate_fps`.
    ///
    /// Uses linear interpolation to produce frames at regular timestamps
    /// aligned to the dataset start.  After this call all `frames[pmu_idx]`
    /// have the same length and identical timestamps.
    pub fn synchronize(&mut self, target_rate_fps: f64) -> Result<()> {
        if target_rate_fps <= 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "target_rate_fps must be positive".to_string(),
            ));
        }
        if self.frames.is_empty() {
            return Ok(());
        }

        let dt_us = (1_000_000.0 / target_rate_fps).round() as i64;
        let mut t = self.t_start_us;
        let mut common_times: Vec<i64> = Vec::new();
        while t <= self.t_end_us {
            common_times.push(t);
            t += dt_us;
        }

        for pmu_idx in 0..self.configs.len() {
            let src = &self.frames[pmu_idx];
            if src.is_empty() {
                continue;
            }
            let pmu_id = self.configs[pmu_idx].pmu_id;
            let mut resampled: Vec<PmuFrame> = Vec::with_capacity(common_times.len());

            for &ts in &common_times {
                // Find surrounding frames using binary search.
                let pos = src.partition_point(|f| f.timestamp_us <= ts);
                let frame = if pos == 0 {
                    src[0].clone()
                } else if pos >= src.len() {
                    src[src.len() - 1].clone()
                } else {
                    let f0 = &src[pos - 1];
                    let f1 = &src[pos];
                    let total = f1.timestamp_us - f0.timestamp_us;
                    let alpha = if total == 0 {
                        0.0
                    } else {
                        (ts - f0.timestamp_us) as f64 / total as f64
                    };
                    interpolate_frames(f0, f1, alpha, ts)
                };
                let mut f = frame;
                f.timestamp_us = ts;
                f.pmu_id = pmu_id;
                resampled.push(f);
            }

            self.frames[pmu_idx] = resampled;
        }

        Ok(())
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Frequency / ROCOF
    // ─────────────────────────────────────────────────────────────────────────

    /// Extract frequency profile: `[(timestamp_us, [freq_hz per PMU])]`.
    pub fn frequency_profile(&self) -> Vec<(i64, Vec<f64>)> {
        if self.frames.is_empty() {
            return Vec::new();
        }
        // Use the first PMU's time axis as reference.
        let n_times = self.frames[0].len();
        let n_pmus = self.frames.len();

        (0..n_times)
            .map(|t_idx| {
                let ts = self.frames[0][t_idx].timestamp_us;
                let freqs: Vec<f64> = (0..n_pmus)
                    .map(|p| {
                        self.frames[p]
                            .get(t_idx)
                            .map(|f| f.freq_hz)
                            .unwrap_or(f64::NAN)
                    })
                    .collect();
                (ts, freqs)
            })
            .collect()
    }

    /// Compute ROCOF from frequency measurements using a sliding window.
    ///
    /// `window_s` is the differentiation window in seconds.
    /// Returns `rocof[pmu_idx][time_idx]`.
    pub fn compute_rocof(&self, window_s: f64) -> Vec<Vec<f64>> {
        self.configs
            .iter()
            .enumerate()
            .map(|(pmu_idx, cfg)| {
                let frames = &self.frames[pmu_idx];
                let window_frames = ((window_s * cfg.data_rate).round() as usize).max(2);

                frames
                    .iter()
                    .enumerate()
                    .map(|(t_idx, frame)| {
                        if t_idx < window_frames - 1 {
                            return frame.rocof; // use embedded ROCOF for early frames
                        }
                        let prev = &frames[t_idx + 1 - window_frames];
                        let dt_us = frame.timestamp_us - prev.timestamp_us;
                        if dt_us == 0 {
                            return 0.0;
                        }
                        (frame.freq_hz - prev.freq_hz) / (dt_us as f64 * 1e-6)
                    })
                    .collect()
            })
            .collect()
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Total Vector Error
    // ─────────────────────────────────────────────────────────────────────────

    /// Compute Total Vector Error (TVE) between this dataset and a reference.
    ///
    /// TVE\[t\] = |X_m(t) − X_r(t)| / |X_r(t)|  (expressed as a ratio, not %)
    ///
    /// Returns `tve[pmu_idx][time_idx]` where each entry is the mean TVE
    /// across all phasor channels at that instant.
    pub fn compute_tve(&self, reference: &PmuDataset) -> Vec<Vec<f64>> {
        self.frames
            .iter()
            .enumerate()
            .map(|(pmu_idx, frames)| {
                let ref_frames = match reference.frames.get(pmu_idx) {
                    Some(rf) => rf,
                    None => return vec![f64::NAN; frames.len()],
                };

                frames
                    .iter()
                    .zip(ref_frames.iter())
                    .map(|(meas, refer)| {
                        let n = meas.phasors.len().min(refer.phasors.len());
                        if n == 0 {
                            return f64::NAN;
                        }
                        let tve_sum: f64 = meas
                            .phasors
                            .iter()
                            .zip(refer.phasors.iter())
                            .take(n)
                            .map(|(m, r)| {
                                let mc = m.to_complex();
                                let rc = r.to_complex();
                                let ref_norm = rc.norm();
                                if ref_norm < 1e-15 {
                                    0.0
                                } else {
                                    (mc - rc).norm() / ref_norm
                                }
                            })
                            .sum();
                        tve_sum / n as f64
                    })
                    .collect()
            })
            .collect()
    }
}

impl Default for PmuDataset {
    fn default() -> Self {
        Self::new()
    }
}

/// Linear interpolation between two frames at fractional position `alpha ∈ [0,1]`.
fn interpolate_frames(f0: &PmuFrame, f1: &PmuFrame, alpha: f64, ts: i64) -> PmuFrame {
    let lerp = |a: f64, b: f64| a + alpha * (b - a);
    let interp_phasor = |pa: &Phasor, pb: &Phasor| {
        // Interpolate in rectangular form to handle phase wrapping correctly.
        let ca = pa.to_complex();
        let cb = pb.to_complex();
        let ci = ca + alpha * (cb - ca);
        Phasor::from_complex(ci)
    };

    let n_phasors = f0.phasors.len().min(f1.phasors.len());
    let phasors = (0..n_phasors)
        .map(|i| interp_phasor(&f0.phasors[i], &f1.phasors[i]))
        .collect();

    let n_analogs = f0.analogs.len().min(f1.analogs.len());
    let analogs = (0..n_analogs)
        .map(|i| lerp(f0.analogs[i], f1.analogs[i]))
        .collect();

    // For digitals, take the value of the earlier frame.
    let digitals = f0.digitals.clone();

    PmuFrame {
        timestamp_us: ts,
        pmu_id: f0.pmu_id,
        stat: f0.stat | f1.stat, // propagate any error bits
        phasors,
        freq_hz: lerp(f0.freq_hz, f1.freq_hz),
        rocof: lerp(f0.rocof, f1.rocof),
        analogs,
        digitals,
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Symmetrical components  (Fortescue transform)
// ─────────────────────────────────────────────────────────────────────────────

/// Compute symmetrical (Fortescue) sequence components from 3-phase phasors.
///
/// The rotation operator is `a = exp(j·2π/3)`.
///
/// Returns `(V0, V1, V2)` — zero, positive, and negative sequence.
///
/// ```text
/// V0 = (Va + Vb + Vc) / 3
/// V1 = (Va + a·Vb + a²·Vc) / 3
/// V2 = (Va + a²·Vb + a·Vc) / 3
/// ```
pub fn symmetrical_components(
    va: Complex64,
    vb: Complex64,
    vc: Complex64,
) -> (Complex64, Complex64, Complex64) {
    // a = exp(j·2π/3)
    let a = Complex64::from_polar(1.0, 2.0 * PI / 3.0);
    let a2 = a * a;

    let v0 = (va + vb + vc) / 3.0;
    let v1 = (va + a * vb + a2 * vc) / 3.0;
    let v2 = (va + a2 * vb + a * vc) / 3.0;

    (v0, v1, v2)
}

/// Compute Voltage Unbalance Factor (VUF) as a percentage.
///
/// VUF = |V2| / |V1| × 100 %
///
/// Returns 0 for a balanced system and NaN if V1 ≈ 0.
pub fn voltage_unbalance_pct(v1: Complex64, v2: Complex64) -> f64 {
    let v1_mag = v1.norm();
    if v1_mag < 1e-15 {
        return f64::NAN;
    }
    v2.norm() / v1_mag * 100.0
}

// ─────────────────────────────────────────────────────────────────────────────
// Park (dq0) transform
// ─────────────────────────────────────────────────────────────────────────────

/// Three-phase (abc) to rotating reference frame (dq0) — Park transform.
///
/// Uses the amplitude-invariant convention:
///
/// ```text
/// [vd]   2/3 · [cos(θ)         cos(θ−2π/3)     cos(θ+2π/3)  ] [va]
/// [vq] =       [−sin(θ)        −sin(θ−2π/3)    −sin(θ+2π/3) ] [vb]
/// [v0]         [1/2            1/2              1/2           ] [vc]
/// ```
///
/// `theta` is the reference angle (rotor or synchronous reference frame angle).
///
/// Returns `(vd, vq, v0)`.
pub fn abc_to_dq0(va: f64, vb: f64, vc: f64, theta: f64) -> (f64, f64, f64) {
    let cos0 = theta.cos();
    let cos1 = (theta - 2.0 * PI / 3.0).cos();
    let cos2 = (theta + 2.0 * PI / 3.0).cos();

    let sin0 = theta.sin();
    let sin1 = (theta - 2.0 * PI / 3.0).sin();
    let sin2 = (theta + 2.0 * PI / 3.0).sin();

    let vd = (2.0 / 3.0) * (cos0 * va + cos1 * vb + cos2 * vc);
    let vq = (2.0 / 3.0) * (-sin0 * va - sin1 * vb - sin2 * vc);
    let v0 = (1.0 / 3.0) * (va + vb + vc); // zero sequence (1/2 per row × 2/3 overall → 1/3)

    (vd, vq, v0)
}

// ─────────────────────────────────────────────────────────────────────────────
// Prony analysis — SVD-based modal identification
// ─────────────────────────────────────────────────────────────────────────────

/// Identified oscillatory mode from Prony analysis.
#[derive(Debug, Clone)]
pub struct PronyMode {
    /// Mode frequency \[Hz\]
    pub frequency_hz: f64,
    /// Damping ratio ζ = −σ / √(σ² + ω²).  Positive → stable.
    pub damping_ratio: f64,
    /// Mode amplitude (half the peak-to-peak for a single mode)
    pub amplitude: f64,
    /// Mode phase \[radians\]
    pub phase_rad: f64,
}

/// SVD-based Prony analysis for modal identification from time-series signals.
///
/// The Prony method fits a sum of complex exponentials to the signal:
///
///   x(k·dt) ≈ Σ_i  A_i · exp(z_i · k·dt) · cos(ω_i · k·dt + φ_i)
///
/// where z_i = σ_i + j·ω_i are the modal poles.
///
/// # Algorithm (SVD-Prony / Matrix Pencil variant)
///
/// 1. Construct the Hankel data matrix of the signal.
/// 2. Truncate via SVD to `n_modes` singular values (noise filtering).
/// 3. Solve for the autoregressive (AR) coefficients via least squares.
/// 4. Find eigenvalues of the companion matrix → modal poles z_i.
/// 5. Compute complex residues via least-squares fit → amplitudes & phases.
pub struct PronyAnalysis {
    /// Number of modes to identify
    pub n_modes: usize,
    /// Analysis window length \[s\]
    pub window_s: f64,
}

impl PronyAnalysis {
    /// Create a Prony analyser.
    pub fn new(n_modes: usize, window_s: f64) -> Self {
        Self { n_modes, window_s }
    }

    /// Identify oscillatory modes from a uniformly sampled signal.
    ///
    /// Uses the **Matrix Pencil Method** (Hua & Sarkar 1990), which builds a Hankel
    /// data matrix, reduces its rank via SVD (noise filtering), then solves a
    /// generalised eigenvalue problem for the complex poles.
    ///
    /// `signal` must be sampled at `dt` \[s\] intervals.
    /// Returns up to `n_modes` modes sorted by descending amplitude.
    pub fn identify_modes(&self, signal: &[f64], dt: f64) -> Result<Vec<PronyMode>> {
        if signal.len() < 4 {
            return Err(OxiGridError::InvalidParameter(
                "Prony analysis requires at least 4 samples".to_string(),
            ));
        }
        if dt <= 0.0 {
            return Err(OxiGridError::InvalidParameter(
                "Sample interval dt must be positive".to_string(),
            ));
        }

        // Clip signal to the analysis window.
        let n_window = ((self.window_s / dt).round() as usize).min(signal.len());
        let n_window = n_window.max(4);
        let x: Vec<f64> = signal[..n_window].to_vec();
        let n = x.len();

        // Pencil parameter L: chosen as n/2 or n/3 per Sarkar's guideline.
        // Number of modes to identify (poles = 2*n_modes for conjugate pairs).
        let l = n / 2; // Hankel matrix column count
        let n_poles = (2 * self.n_modes).min(l.saturating_sub(1));
        if n_poles == 0 || l < 2 {
            return Err(OxiGridError::InvalidParameter(
                "Signal too short for requested n_modes".to_string(),
            ));
        }

        // ── Step 1: Build Hankel data matrix Y (n-L × L+1, real) ─────────────
        // Y[i, j] = x[i + j]  for i in 0..n-L, j in 0..L+1
        let n_rows = n - l; // rows of Y
        let n_cols = l + 1; // cols of Y

        let mut y_mat = vec![0.0f64; n_rows * n_cols];
        for i in 0..n_rows {
            for j in 0..n_cols {
                y_mat[i * n_cols + j] = x[i + j];
            }
        }

        // ── Step 2: Compute SVD of Y via Golub-Reinsch bidiagonalisation ──────
        // We only need the first n_poles singular values/vectors.
        // Use economy SVD: Y ≈ U · diag(σ) · Vᵀ
        let (u_mat, sigma, v_mat) = svd_economy(&y_mat, n_rows, n_cols)?;

        // Determine effective rank r = n_poles (or fewer if singular values drop off).
        let sigma_max = sigma.first().copied().unwrap_or(0.0);
        let threshold = sigma_max * 1e-6;
        let rank = sigma
            .iter()
            .take(n_poles)
            .filter(|&&s| s > threshold)
            .count()
            .max(1);

        // ── Step 3: Form pencil matrices Y1 and Y2 from V ────────────────────
        // V1 = V[0..L, 0..rank]  (first L rows of V, first rank columns)
        // V2 = V[1..L+1, 0..rank]  (last L rows of V, first rank columns)
        // Poles = eigenvalues of Y1† · Y2 (pseudoinverse of V1 times V2).
        let v_rows = n_cols; // L+1
        let v_rank_cols = rank;

        // V1: rows 0..l (= 0..v_rows-1), cols 0..rank
        let mut v1 = vec![0.0f64; l * v_rank_cols];
        for row in 0..l {
            for col in 0..v_rank_cols {
                v1[row * v_rank_cols + col] = v_mat[row * v_rows + col];
            }
        }
        // V2: rows 1..l+1, cols 0..rank
        let mut v2 = vec![0.0f64; l * v_rank_cols];
        for row in 1..=l {
            for col in 0..v_rank_cols {
                v2[(row - 1) * v_rank_cols + col] = v_mat[row * v_rows + col];
            }
        }

        // ── Step 4: Compute Z = (V1ᵀV1)⁻¹ V1ᵀ V2 = V1† V2 ─────────────────
        // V1 is (l × rank), V1ᵀ is (rank × l).
        // V1ᵀV1 is (rank × rank), V1ᵀV2 is (rank × rank).
        let v1t_v1 = mat_mul_at_a(&v1, l, v_rank_cols);
        let v1t_v2 = mat_mul_at_b_rect(&v1, l, v_rank_cols, &v2, v_rank_cols);
        let z_mat = solve_spd_mat(&v1t_v1, v_rank_cols, &v1t_v2, v_rank_cols)?;

        // ── Step 5: Eigenvalues of Z (rank×rank) → discrete poles z_i ────────
        let mut z_complex_mat: Vec<Complex64> =
            vec![Complex64::new(0.0, 0.0); v_rank_cols * v_rank_cols];
        for i in 0..v_rank_cols {
            for j in 0..v_rank_cols {
                z_complex_mat[i * v_rank_cols + j] =
                    Complex64::new(z_mat[i * v_rank_cols + j], 0.0);
            }
        }
        let poles_complex = qr_eigenvalues_complex(&mut z_complex_mat, v_rank_cols)?;

        // ── Step 6: Compute residues via Vandermonde LS fit ───────────────────
        let n_poles_found = poles_complex.len();
        if n_poles_found == 0 {
            return Ok(Vec::new());
        }

        let vand = vandermonde_complex(&poles_complex, n);
        let residues = solve_complex_ls(&vand, &x, n, n_poles_found)?;

        // ── Step 7: Extract physical modes ────────────────────────────────────
        let nyquist = 0.5 / dt;
        let mut modes: Vec<PronyMode> = Vec::new();
        let mut used = vec![false; n_poles_found];

        for i in 0..n_poles_found {
            if used[i] {
                continue;
            }

            let z = poles_complex[i];
            // Only process non-real poles (conjugate pairs carry the oscillation info).
            if z.im.abs() < 1e-10 {
                used[i] = true;
                continue;
            }

            // ln(z) / dt gives continuous-time pole s = σ + jω
            let ln_z = z.ln();
            let sigma = ln_z.re / dt;
            let omega = ln_z.im / dt; // rad/s

            // Only keep positive-frequency half (discard negative-frequency conjugate).
            if omega <= 0.0 {
                continue;
            }

            let freq_hz = omega / (2.0 * PI);

            // Reject near-DC and near-Nyquist artefacts.
            if freq_hz < 0.01 || freq_hz > nyquist * 0.99 {
                used[i] = true;
                continue;
            }

            let omega2 = sigma * sigma + omega * omega;
            let damping_ratio = if omega2 < 1e-30 {
                0.0
            } else {
                (-sigma / omega2.sqrt()).clamp(-1.0, 1.0)
            };

            let b = residues[i];
            let amplitude = b.norm() * 2.0; // conjugate pair factor
            let phase_rad = b.arg();

            // Mark this pole and its conjugate as used.
            used[i] = true;
            for j in (i + 1)..n_poles_found {
                if (poles_complex[j] - z.conj()).norm() < 1e-6 {
                    used[j] = true;
                }
            }

            modes.push(PronyMode {
                frequency_hz: freq_hz,
                damping_ratio,
                amplitude,
                phase_rad,
            });
        }

        // Sort by descending amplitude.
        modes.sort_by(|a, b| {
            b.amplitude
                .partial_cmp(&a.amplitude)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        modes.truncate(self.n_modes);

        drop(u_mat); // used only for U=AV/sigma; not needed for mode extraction
        Ok(modes)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Linear algebra helpers (no external LA dependency)
// ─────────────────────────────────────────────────────────────────────────────

/// Compute XᵀX where X is stored row-major with `n_rows` rows and `n_cols` cols.
fn mat_mul_at_a(x: &[f64], n_rows: usize, n_cols: usize) -> Vec<f64> {
    let mut out = vec![0.0f64; n_cols * n_cols];
    for i in 0..n_cols {
        for j in 0..=i {
            let mut s = 0.0f64;
            for r in 0..n_rows {
                s += x[r * n_cols + i] * x[r * n_cols + j];
            }
            out[i * n_cols + j] = s;
            out[j * n_cols + i] = s;
        }
    }
    out
}

/// Compute Xᵀy.
#[allow(dead_code)]
fn mat_vec_at_b(x: &[f64], n_rows: usize, n_cols: usize, y: &[f64]) -> Vec<f64> {
    let mut out = vec![0.0f64; n_cols];
    for j in 0..n_cols {
        let mut s = 0.0f64;
        for r in 0..n_rows {
            s += x[r * n_cols + j] * y[r];
        }
        out[j] = s;
    }
    out
}

/// Solve a symmetric positive definite linear system A·x = b via Cholesky
/// decomposition (in-place).  Falls back to LDLᵀ with diagonal regularisation
/// if the matrix is near-singular.
#[allow(dead_code)]
fn solve_spd(a: &[f64], n: usize, b: &[f64]) -> Result<Vec<f64>> {
    // Cholesky: A = L·Lᵀ
    let mut l = vec![0.0f64; n * n];
    let mut a_work = a.to_vec();

    // Add a small regulariser for numerical stability.
    let diag_max = (0..n).map(|i| a_work[i * n + i]).fold(0.0f64, f64::max);
    let eps = diag_max * 1e-10 + 1e-30;
    for i in 0..n {
        a_work[i * n + i] += eps;
    }

    for j in 0..n {
        let mut s = a_work[j * n + j];
        for k in 0..j {
            s -= l[j * n + k] * l[j * n + k];
        }
        if s <= 0.0 {
            return Err(OxiGridError::LinearAlgebra(
                "Prony normal equations: matrix not positive definite".to_string(),
            ));
        }
        l[j * n + j] = s.sqrt();
        for i in (j + 1)..n {
            let mut t = a_work[i * n + j];
            for k in 0..j {
                t -= l[i * n + k] * l[j * n + k];
            }
            l[i * n + j] = t / l[j * n + j];
        }
    }

    // Forward substitution: L·y = b
    let mut y = vec![0.0f64; n];
    for i in 0..n {
        let mut s = b[i];
        for k in 0..i {
            s -= l[i * n + k] * y[k];
        }
        y[i] = s / l[i * n + i];
    }

    // Back substitution: Lᵀ·x = y
    let mut x = vec![0.0f64; n];
    for i in (0..n).rev() {
        let mut s = y[i];
        for k in (i + 1)..n {
            s -= l[k * n + i] * x[k];
        }
        x[i] = s / l[i * n + i];
    }

    Ok(x)
}

/// Compute AᵀB where A is (m×k) and B is (m×n), both row-major.
/// Returns (k × n) result.
fn mat_mul_at_b_rect(a: &[f64], m: usize, k: usize, b: &[f64], n: usize) -> Vec<f64> {
    let mut out = vec![0.0f64; k * n];
    for i in 0..k {
        for j in 0..n {
            let mut s = 0.0f64;
            for r in 0..m {
                s += a[r * k + i] * b[r * n + j];
            }
            out[i * n + j] = s;
        }
    }
    out
}

/// Solve A·X = B where A is n×n SPD and B is n×m (multiple right-hand sides).
/// Returns X (n×m) as a flat row-major vec.
fn solve_spd_mat(a: &[f64], n: usize, b: &[f64], m: usize) -> Result<Vec<f64>> {
    // Cholesky factorisation of A.
    let mut l = vec![0.0f64; n * n];
    let mut a_work = a.to_vec();
    let diag_max = (0..n).map(|i| a_work[i * n + i]).fold(0.0f64, f64::max);
    let eps = diag_max * 1e-10 + 1e-30;
    for i in 0..n {
        a_work[i * n + i] += eps;
    }
    for j in 0..n {
        let mut s = a_work[j * n + j];
        for k in 0..j {
            s -= l[j * n + k] * l[j * n + k];
        }
        if s <= 0.0 {
            return Err(OxiGridError::LinearAlgebra(
                "Pencil matrix not positive definite".to_string(),
            ));
        }
        l[j * n + j] = s.sqrt();
        for i in (j + 1)..n {
            let mut t = a_work[i * n + j];
            for k in 0..j {
                t -= l[i * n + k] * l[j * n + k];
            }
            l[i * n + j] = t / l[j * n + j];
        }
    }

    // Solve for each RHS column separately.
    let mut x = vec![0.0f64; n * m];
    for col in 0..m {
        // Forward substitution: L·y = b[:,col]
        let mut y = vec![0.0f64; n];
        for i in 0..n {
            let mut s = b[i * m + col];
            for k in 0..i {
                s -= l[i * n + k] * y[k];
            }
            y[i] = s / l[i * n + i];
        }
        // Back substitution: Lᵀ·x = y
        for i in (0..n).rev() {
            let mut s = y[i];
            for k in (i + 1)..n {
                s -= l[k * n + i] * x[k * m + col];
            }
            x[i * m + col] = s / l[i * n + i];
        }
    }
    Ok(x)
}

/// Economy SVD via Golub-Reinsch bidiagonalisation (one-sided Jacobi for simplicity).
///
/// Computes an approximate thin SVD A ≈ U·Σ·Vᵀ where A is (m×n), m ≥ n.
/// Returns (U: m×k, sigma: k, V: n×k) where k = min(m,n).
///
/// Algorithm: one-sided Jacobi on AᵀA to obtain V and singular values,
/// then U = A·V·Σ⁻¹.
fn svd_economy(a: &[f64], m: usize, n: usize) -> Result<(Vec<f64>, Vec<f64>, Vec<f64>)> {
    // We compute the SVD of A (m×n) via the eigendecomposition of AᵀA (n×n).
    // AᵀA = Vᵀ·Σ²·V  → right singular vectors V and singular values σ_i = √λ_i.
    //
    // One-sided Jacobi sweeps on the symmetric n×n matrix AᵀA.

    let k = m.min(n);

    // Form C = AᵀA  (n×n).
    let mut c = vec![0.0f64; n * n];
    for i in 0..n {
        for j in 0..=i {
            let mut s = 0.0f64;
            for r in 0..m {
                s += a[r * n + i] * a[r * n + j];
            }
            c[i * n + j] = s;
            c[j * n + i] = s;
        }
    }

    // Jacobi eigendecomposition of C to obtain eigenvalues and eigenvectors (= V columns).
    let (eigenvalues, v_mat) = jacobi_eigen_sym(&c, n)?;

    // Singular values = sqrt of eigenvalues (clamp negatives to 0 due to round-off).
    let sigma: Vec<f64> = eigenvalues
        .iter()
        .take(k)
        .map(|&e| e.max(0.0).sqrt())
        .collect();

    // Compute U columns: u_i = A·v_i / σ_i.
    let mut u_mat = vec![0.0f64; m * k];
    for i in 0..k {
        if sigma[i] < 1e-30 {
            // Null singular value: U column can be arbitrary; leave as zero.
            continue;
        }
        for row in 0..m {
            let mut s = 0.0f64;
            for col in 0..n {
                s += a[row * n + col] * v_mat[col * n + i];
            }
            u_mat[row * k + i] = s / sigma[i];
        }
    }

    // Return full V (n×n row-major, eigenvectors as columns) for pencil slicing.
    Ok((u_mat, sigma, v_mat))
}

/// Symmetric Jacobi eigendecomposition.
///
/// Returns (eigenvalues, V) where V (n×n row-major) has eigenvectors as columns,
/// sorted by descending eigenvalue.
fn jacobi_eigen_sym(a: &[f64], n: usize) -> Result<(Vec<f64>, Vec<f64>)> {
    const MAX_SWEEPS: usize = 100;

    let mut d = a.to_vec(); // working copy (will converge to diagonal)
    let mut v = vec![0.0f64; n * n];
    // V = I initially.
    for i in 0..n {
        v[i * n + i] = 1.0;
    }

    for _sweep in 0..MAX_SWEEPS {
        // Check off-diagonal norm.
        let off: f64 = (0..n)
            .flat_map(|i| (0..i).map(move |j| (i, j)))
            .map(|(i, j)| d[i * n + j] * d[i * n + j])
            .sum::<f64>()
            .sqrt();
        if off < 1e-14 {
            break;
        }

        // Sweep over all off-diagonal pairs.
        for p in 0..n {
            for q in (p + 1)..n {
                let dpq = d[p * n + q];
                if dpq.abs() < 1e-15 {
                    continue;
                }
                let dpp = d[p * n + p];
                let dqq = d[q * n + q];
                let tau = (dqq - dpp) / (2.0 * dpq);
                let t = if tau >= 0.0 {
                    1.0 / (tau + (1.0 + tau * tau).sqrt())
                } else {
                    1.0 / (tau - (1.0 + tau * tau).sqrt())
                };
                let cos = 1.0 / (1.0 + t * t).sqrt();
                let sin = t * cos;

                // Update d = GᵀDG.
                d[p * n + p] = dpp - t * dpq;
                d[q * n + q] = dqq + t * dpq;
                d[p * n + q] = 0.0;
                d[q * n + p] = 0.0;
                for r in 0..n {
                    if r == p || r == q {
                        continue;
                    }
                    let drp = d[r * n + p];
                    let drq = d[r * n + q];
                    d[r * n + p] = cos * drp - sin * drq;
                    d[p * n + r] = d[r * n + p];
                    d[r * n + q] = sin * drp + cos * drq;
                    d[q * n + r] = d[r * n + q];
                }

                // Update V = V·G.
                for r in 0..n {
                    let vrp = v[r * n + p];
                    let vrq = v[r * n + q];
                    v[r * n + p] = cos * vrp - sin * vrq;
                    v[r * n + q] = sin * vrp + cos * vrq;
                }
            }
        }
    }

    // Extract eigenvalues from diagonal.
    let mut pairs: Vec<(f64, usize)> = (0..n).map(|i| (d[i * n + i], i)).collect();
    // Sort descending.
    pairs.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));

    let eigenvalues: Vec<f64> = pairs.iter().map(|&(e, _)| e).collect();
    // Permute columns of V to match sorted order.
    let perm: Vec<usize> = pairs.iter().map(|&(_, i)| i).collect();
    let mut v_sorted = vec![0.0f64; n * n];
    for row in 0..n {
        for col in 0..n {
            v_sorted[row * n + col] = v[row * n + perm[col]];
        }
    }

    Ok((eigenvalues, v_sorted))
}

/// Compute eigenvalues of the companion matrix of the AR polynomial (reference implementation).
#[allow(dead_code)]
///
/// AR model: x[k] = a[0]·x[k-1] + a[1]·x[k-2] + … + a[p-1]·x[k-p]
///
/// Characteristic polynomial: z^p − a[0]·z^{p-1} − … − a[p-1] = 0
///
/// Companion matrix (first column form):
///   Col 0: [a[0], 1, 0, …, 0]ᵀ
///   Col 1: [a[1], 0, 1, …, 0]ᵀ
///   …
///   Col p-1: [a[p-1], 0, 0, …, 0]ᵀ
///
/// Equivalently (row-major, the transpose form):
///   Row 0: [a[0], a[1], …, a[p-1]]
///   Row i: e_{i-1}  (1 at column i-1, zeros elsewhere)
fn companion_eigenvalues(ar: &[f64], p: usize) -> Result<Vec<Complex64>> {
    if p == 0 {
        return Ok(Vec::new());
    }
    if p == 1 {
        return Ok(vec![Complex64::new(ar[0], 0.0)]);
    }

    // Build companion matrix (row-major, p×p) as complex.
    // Row 0 = AR coefficients.
    // Row i (i >= 1): 1 at column i-1.
    let mut c: Vec<Complex64> = vec![Complex64::new(0.0, 0.0); p * p];
    for j in 0..p {
        c[j] = Complex64::new(ar[j], 0.0);
    }
    for i in 1..p {
        c[i * p + (i - 1)] = Complex64::new(1.0, 0.0);
    }

    // QR iteration to extract all eigenvalues.
    qr_eigenvalues_complex(&mut c, p)
}

/// Unshifted complex QR algorithm to find eigenvalues of a p×p complex matrix.
///
/// Applies up to `max_iter` Householder-based QR decompositions.
/// Returns eigenvalues from the upper triangular Schur form diagonal.
fn qr_eigenvalues_complex(h: &mut [Complex64], p: usize) -> Result<Vec<Complex64>> {
    const MAX_ITER: usize = 500;

    // Reduce to upper Hessenberg form first (for efficiency).
    hessenberg_reduction(h, p);

    let mut eigenvalues: Vec<Complex64> = Vec::with_capacity(p);
    let mut size = p;

    let mut iter_count = 0usize;

    while size > 1 {
        if iter_count > MAX_ITER * p {
            // Collect whatever eigenvalues remain on the diagonal.
            for i in 0..size {
                eigenvalues.push(h[i * p + i]);
            }
            return Ok(eigenvalues);
        }

        // Check for deflation: if |h[size-1, size-2]| is tiny.
        let off_diag = h[(size - 1) * p + (size - 2)].norm();
        let scale = h[(size - 2) * p + (size - 2)].norm() + h[(size - 1) * p + (size - 1)].norm();
        if off_diag <= 1e-14 * scale {
            eigenvalues.push(h[(size - 1) * p + (size - 1)]);
            size -= 1;
            iter_count = 0;
            continue;
        }

        // Wilkinson shift: use eigenvalue of the 2×2 trailing submatrix
        // closest to h[size-1,size-1].
        let shift = wilkinson_shift(h, p, size);

        // Apply shifted QR step.
        qr_step_hessenberg(h, p, size, shift);

        iter_count += 1;
    }

    if size == 1 {
        eigenvalues.push(h[0]);
    }

    Ok(eigenvalues)
}

/// Reduce a square matrix to upper Hessenberg form in-place via Householder.
fn hessenberg_reduction(h: &mut [Complex64], p: usize) {
    for k in 0..(p.saturating_sub(2)) {
        // Build Householder vector for column k, rows k+1..p.
        let col_len = p - k - 1;
        if col_len == 0 {
            break;
        }
        let mut v: Vec<Complex64> = (0..col_len).map(|i| h[(k + 1 + i) * p + k]).collect();

        let norm_v: f64 = v.iter().map(|x| x.norm_sqr()).sum::<f64>().sqrt();
        if norm_v < 1e-30 {
            continue;
        }
        // Choose sign to avoid cancellation.
        let alpha = -v[0].signum_complex() * norm_v;
        v[0] -= alpha;
        let norm_v2: f64 = v.iter().map(|x| x.norm_sqr()).sum::<f64>().sqrt();
        if norm_v2 < 1e-30 {
            continue;
        }
        for vi in v.iter_mut() {
            *vi /= norm_v2;
        }

        // H ← (I − 2vvᴴ) H (from the left, cols k..p)
        for j in k..p {
            let dot: Complex64 = (0..col_len)
                .map(|i| v[i].conj() * h[(k + 1 + i) * p + j])
                .sum();
            for i in 0..col_len {
                h[(k + 1 + i) * p + j] -= 2.0 * v[i] * dot;
            }
        }
        // H ← H (I − 2vvᴴ) (from the right, rows 0..p)
        for i in 0..p {
            let dot: Complex64 = (0..col_len).map(|j| h[i * p + (k + 1 + j)] * v[j]).sum();
            for j in 0..col_len {
                h[i * p + (k + 1 + j)] -= 2.0 * dot * v[j].conj();
            }
        }
    }
}

/// Wilkinson shift: choose the eigenvalue of the 2×2 trailing submatrix
/// closest to h[size-1,size-1].
fn wilkinson_shift(h: &[Complex64], p: usize, size: usize) -> Complex64 {
    let a = h[(size - 2) * p + (size - 2)];
    let b = h[(size - 2) * p + (size - 1)];
    let c = h[(size - 1) * p + (size - 2)];
    let d = h[(size - 1) * p + (size - 1)];

    let trace = a + d;
    let det = a * d - b * c;
    let disc = (trace * trace - 4.0 * det).sqrt();

    let lam1 = (trace + disc) / 2.0;
    let lam2 = (trace - disc) / 2.0;

    if (lam1 - d).norm() < (lam2 - d).norm() {
        lam1
    } else {
        lam2
    }
}

/// Single QR step with shift on the upper Hessenberg submatrix of size `size`.
fn qr_step_hessenberg(h: &mut [Complex64], p: usize, size: usize, shift: Complex64) {
    // Givens-rotation based QR step on h[0..size, 0..size].
    for k in 0..(size - 1) {
        let a = h[k * p + k] - shift;
        let b = h[(k + 1) * p + k];
        let r = (a.norm_sqr() + b.norm_sqr()).sqrt();
        if r < 1e-30 {
            continue;
        }
        let cos = a / r;
        let sin = b / r;

        // Apply from the left: rows k and k+1, cols k..size.
        for j in k..size {
            let t1 = cos.conj() * h[k * p + j] + sin.conj() * h[(k + 1) * p + j];
            let t2 = -sin * h[k * p + j] + cos * h[(k + 1) * p + j];
            h[k * p + j] = t1;
            h[(k + 1) * p + j] = t2;
        }
        // Apply from the right: rows 0..min(size,k+2), cols k and k+1.
        for i in 0..size.min(k + 3) {
            let t1 = h[i * p + k] * cos + h[i * p + (k + 1)] * sin;
            let t2 = h[i * p + k] * (-sin.conj()) + h[i * p + (k + 1)] * cos.conj();
            h[i * p + k] = t1;
            h[i * p + (k + 1)] = t2;
        }
    }
}

/// Extension trait for complex "signum" (unit vector in the direction of z).
trait ComplexSignum {
    fn signum_complex(self) -> Self;
}

impl ComplexSignum for Complex64 {
    fn signum_complex(self) -> Self {
        let n = self.norm();
        if n < 1e-30 {
            Complex64::new(1.0, 0.0)
        } else {
            self / n
        }
    }
}

/// Build complex Vandermonde matrix V of shape (n × n_poles):
///   V[k, i] = z_i^k
fn vandermonde_complex(poles: &[Complex64], n: usize) -> Vec<Complex64> {
    let n_poles = poles.len();
    let mut v = vec![Complex64::new(0.0, 0.0); n * n_poles];
    for i in 0..n_poles {
        let mut pow = Complex64::new(1.0, 0.0);
        for k in 0..n {
            v[k * n_poles + i] = pow;
            pow *= poles[i];
        }
    }
    v
}

/// Solve a complex least-squares problem V·b ≈ x using normal equations VᴴV·b = Vᴴx.
fn solve_complex_ls(
    v: &[Complex64],
    x: &[f64],
    n: usize,
    n_poles: usize,
) -> Result<Vec<Complex64>> {
    // Build VᴴV (n_poles × n_poles, Hermitian).
    let mut vhv = vec![Complex64::new(0.0, 0.0); n_poles * n_poles];
    for i in 0..n_poles {
        for j in 0..=i {
            let mut s = Complex64::new(0.0, 0.0);
            for k in 0..n {
                s += v[k * n_poles + i].conj() * v[k * n_poles + j];
            }
            vhv[i * n_poles + j] = s;
            vhv[j * n_poles + i] = s.conj();
        }
    }

    // Build Vᴴx (n_poles).
    let mut vhx = vec![Complex64::new(0.0, 0.0); n_poles];
    for i in 0..n_poles {
        let mut s = Complex64::new(0.0, 0.0);
        for k in 0..n {
            s += v[k * n_poles + i].conj() * x[k];
        }
        vhx[i] = s;
    }

    // Solve VᴴV·b = Vᴴx via complex Cholesky (Hermitian positive definite).
    solve_hermitian_pd(&vhv, n_poles, &vhx)
}

/// Solve a Hermitian positive-definite linear system via complex Cholesky.
fn solve_hermitian_pd(a: &[Complex64], n: usize, b: &[Complex64]) -> Result<Vec<Complex64>> {
    // Regularise diagonal.
    let mut a_work = a.to_vec();
    let diag_max = (0..n).map(|i| a_work[i * n + i].re).fold(0.0f64, f64::max);
    let eps = diag_max * 1e-10 + 1e-30;
    for i in 0..n {
        a_work[i * n + i] += eps;
    }

    // Complex Cholesky: A = L·Lᴴ
    let mut l: Vec<Complex64> = vec![Complex64::new(0.0, 0.0); n * n];
    for j in 0..n {
        let mut s = a_work[j * n + j].re;
        for k in 0..j {
            s -= l[j * n + k].norm_sqr();
        }
        if s <= 0.0 {
            return Err(OxiGridError::LinearAlgebra(
                "Prony Vandermonde system not positive definite".to_string(),
            ));
        }
        l[j * n + j] = Complex64::new(s.sqrt(), 0.0);
        for i in (j + 1)..n {
            let mut t = a_work[i * n + j];
            for k in 0..j {
                t -= l[i * n + k] * l[j * n + k].conj();
            }
            l[i * n + j] = t / l[j * n + j];
        }
    }

    // Forward: L·y = b
    let mut y = vec![Complex64::new(0.0, 0.0); n];
    for i in 0..n {
        let mut s = b[i];
        for k in 0..i {
            s -= l[i * n + k] * y[k];
        }
        y[i] = s / l[i * n + i];
    }

    // Backward: Lᴴ·x = y
    let mut x = vec![Complex64::new(0.0, 0.0); n];
    for i in (0..n).rev() {
        let mut s = y[i];
        for k in (i + 1)..n {
            s -= l[k * n + i].conj() * x[k];
        }
        x[i] = s / l[i * n + i].conj();
    }

    Ok(x)
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    // ── Phasor ──────────────────────────────────────────────────────────────

    #[test]
    fn test_phasor_complex_conversion() {
        let p = Phasor::new(1.0, PI / 4.0);
        let c = p.to_complex();
        assert!(
            (c.re - (1.0f64 / 2.0f64.sqrt())).abs() < 1e-10,
            "re = {:.12}",
            c.re
        );
        assert!(
            (c.im - (1.0f64 / 2.0f64.sqrt())).abs() < 1e-10,
            "im = {:.12}",
            c.im
        );
    }

    #[test]
    fn test_phasor_from_complex_roundtrip() {
        let original = Phasor::new(1.5, 0.8);
        let c = original.to_complex();
        let recovered = Phasor::from_complex(c);
        assert!((recovered.magnitude - original.magnitude).abs() < 1e-12);
        assert!((recovered.angle_rad - original.angle_rad).abs() < 1e-12);
    }

    #[test]
    fn test_phasor_magnitude_db() {
        let p = Phasor::new(10.0, 0.0);
        assert!((p.magnitude_db() - 20.0).abs() < 1e-10, "10→20 dB");
    }

    #[test]
    fn test_phasor_magnitude_db_zero() {
        let p = Phasor::new(0.0, 0.0);
        assert!(p.magnitude_db().is_infinite() && p.magnitude_db() < 0.0);
    }

    // ── Symmetrical components ───────────────────────────────────────────────

    #[test]
    fn test_symmetrical_components_balanced() {
        // Balanced positive-sequence set: Va=1∠0°, Vb=1∠-120°, Vc=1∠+120°
        let va = Complex64::from_polar(1.0, 0.0);
        let vb = Complex64::from_polar(1.0, -2.0 * PI / 3.0);
        let vc = Complex64::from_polar(1.0, 2.0 * PI / 3.0);
        let (v0, v1, v2) = symmetrical_components(va, vb, vc);

        assert!(
            v0.norm() < 1e-10,
            "V0 should be zero for balanced: {:.3e}",
            v0.norm()
        );
        assert!(
            (v1.norm() - 1.0).abs() < 1e-10,
            "V1 should equal Va: {:.6}",
            v1.norm()
        );
        assert!(
            v2.norm() < 1e-10,
            "V2 should be zero for balanced: {:.3e}",
            v2.norm()
        );
    }

    #[test]
    fn test_symmetrical_components_zero_sequence() {
        // Pure zero sequence: all three phasors identical.
        let v = Complex64::new(1.0, 0.0);
        let (v0, v1, v2) = symmetrical_components(v, v, v);
        assert!(
            (v0 - v).norm() < 1e-10,
            "V0 = Va for zero-seq: {:.3e}",
            (v0 - v).norm()
        );
        assert!(v1.norm() < 1e-10, "V1 = 0 for zero-seq");
        assert!(v2.norm() < 1e-10, "V2 = 0 for zero-seq");
    }

    // ── Voltage unbalance ────────────────────────────────────────────────────

    #[test]
    fn test_voltage_unbalance_pct_balanced() {
        let va = Complex64::from_polar(1.0, 0.0);
        let vb = Complex64::from_polar(1.0, -2.0 * PI / 3.0);
        let vc = Complex64::from_polar(1.0, 2.0 * PI / 3.0);
        let (_v0, v1, v2) = symmetrical_components(va, vb, vc);
        let vuf = voltage_unbalance_pct(v1, v2);
        assert!(vuf < 1e-8, "Balanced → VUF ≈ 0%, got {:.3e}", vuf);
    }

    #[test]
    fn test_voltage_unbalance_pct_known() {
        // V1 = 1.0, V2 = 0.05 → VUF = 5 %
        let v1 = Complex64::new(1.0, 0.0);
        let v2 = Complex64::new(0.05, 0.0);
        let vuf = voltage_unbalance_pct(v1, v2);
        assert!((vuf - 5.0).abs() < 1e-10, "VUF = {:.6}", vuf);
    }

    // ── Park (dq0) transform ─────────────────────────────────────────────────

    #[test]
    fn test_abc_to_dq0_balanced_theta0() {
        // Balanced positive-sequence at peak: Va=V, Vb=V∠-120°, Vc=V∠+120° (instantaneous)
        // At θ=0, amplitude-invariant Park transform gives vd=V, vq=0, v0=0.
        let v_peak = 1.0;
        let va = v_peak;
        let vb = v_peak * (-(2.0 * PI / 3.0)).cos();
        let vc = v_peak * (2.0 * PI / 3.0).cos();
        let (vd, vq, v0) = abc_to_dq0(va, vb, vc, 0.0);
        assert!((vd - v_peak).abs() < 1e-10, "vd = {:.6}", vd);
        assert!(vq.abs() < 1e-10, "vq = {:.6}", vq);
        assert!(v0.abs() < 1e-10, "v0 = {:.6}", v0);
    }

    #[test]
    fn test_abc_to_dq0_zero_sequence_only() {
        // va = vb = vc = 1 → v0 = 1, vd = 0, vq = 0
        let (vd, vq, v0) = abc_to_dq0(1.0, 1.0, 1.0, 0.0);
        assert!(vd.abs() < 1e-10, "vd should be 0: {:.6}", vd);
        assert!(vq.abs() < 1e-10, "vq should be 0: {:.6}", vq);
        assert!((v0 - 1.0).abs() < 1e-10, "v0 should be 1: {:.6}", v0);
    }

    // ── Prony analysis ───────────────────────────────────────────────────────

    #[test]
    fn test_prony_single_undamped_mode() {
        // x(t) = cos(2π·1·t), f=1 Hz, dt=0.02 s, T=10 s
        let dt = 0.02_f64;
        let f = 1.0_f64;
        let n_samples = 500usize;
        let signal: Vec<f64> = (0..n_samples)
            .map(|k| (2.0 * PI * f * k as f64 * dt).cos())
            .collect();

        let prony = PronyAnalysis::new(3, 10.0);
        let modes = prony.identify_modes(&signal, dt).expect("Prony failed");

        assert!(!modes.is_empty(), "Should identify at least one mode");
        let dominant = &modes[0];
        assert!(
            (dominant.frequency_hz - f).abs() < 0.05,
            "Frequency should be ≈1 Hz, got {:.4}",
            dominant.frequency_hz
        );
        // Damping ratio should be near zero (undamped cosine).
        assert!(
            dominant.damping_ratio.abs() < 0.1,
            "Damping should be ≈0, got {:.4}",
            dominant.damping_ratio
        );
    }

    #[test]
    fn test_prony_damped_mode() {
        // x(t) = e^{−σt}·cos(ω·t) with σ=0.2, f=0.5 Hz
        let dt = 0.02_f64;
        let sigma = 0.2_f64;
        let f = 0.5_f64;
        let omega = 2.0 * PI * f;
        let n_samples = 500usize;
        let signal: Vec<f64> = (0..n_samples)
            .map(|k| {
                let t = k as f64 * dt;
                (-sigma * t).exp() * (omega * t).cos()
            })
            .collect();

        let prony = PronyAnalysis::new(3, 10.0);
        let modes = prony.identify_modes(&signal, dt).expect("Prony failed");

        assert!(!modes.is_empty(), "Should identify at least one mode");
        let dominant = &modes[0];
        assert!(
            (dominant.frequency_hz - f).abs() < 0.1,
            "Frequency ≈0.5 Hz, got {:.4}",
            dominant.frequency_hz
        );
        // Damping ratio must be positive (stable / decaying signal).
        assert!(
            dominant.damping_ratio > 0.0,
            "Damping ratio should be positive (damped), got {:.4}",
            dominant.damping_ratio
        );
    }

    #[test]
    fn test_prony_too_short_signal() {
        let prony = PronyAnalysis::new(3, 10.0);
        let result = prony.identify_modes(&[1.0, 2.0], 0.02);
        assert!(result.is_err(), "Should fail for very short signal");
    }

    // ── Quality flags ────────────────────────────────────────────────────────

    #[test]
    fn test_pmu_quality_flag_phase_jump() {
        let cfg = PmuConfig::new_50hz("TEST", 1);
        let make_frame = |ts: i64, angle: f64| PmuFrame {
            timestamp_us: ts,
            pmu_id: 1,
            stat: 0,
            phasors: vec![Phasor::new(1.0, angle)],
            freq_hz: 50.0,
            rocof: 0.0,
            analogs: vec![],
            digitals: vec![],
        };

        // Frame 1 at angle 0°, Frame 2 at angle 45° (OK), Frame 3 at angle 90° jump (> 30°)
        let frames = vec![
            make_frame(0, 0.0),
            make_frame(20_000, PI / 8.0), // 22.5° change — OK
            make_frame(40_000, PI / 2.0), // 45° total, 22.5° incremental — OK actually
        ];

        // Create a frame with big jump: previous 0°, current 60° = π/3 > π/6 threshold
        let frames_with_jump = vec![
            make_frame(0, 0.0),
            make_frame(20_000, PI / 3.0 + 0.1), // > 30° jump
        ];

        let mut dataset = PmuDataset::new();
        dataset.add_pmu(cfg.clone(), frames);
        let q1 = dataset.quality_flags();
        assert_eq!(q1[0][0], PmuQuality::Good);

        let mut dataset2 = PmuDataset::new();
        dataset2.add_pmu(cfg, frames_with_jump);
        let q2 = dataset2.quality_flags();
        assert_eq!(q2[0][1], PmuQuality::PhaseJump);
    }

    #[test]
    fn test_pmu_quality_flag_bad_status() {
        let cfg = PmuConfig::new_50hz("TEST", 1);
        let bad_frame = PmuFrame {
            timestamp_us: 0,
            pmu_id: 1,
            stat: 0x8000, // data error bit set
            phasors: vec![Phasor::new(1.0, 0.0)],
            freq_hz: 50.0,
            rocof: 0.0,
            analogs: vec![],
            digitals: vec![],
        };
        let mut dataset = PmuDataset::new();
        dataset.add_pmu(cfg, vec![bad_frame]);
        let q = dataset.quality_flags();
        assert_eq!(q[0][0], PmuQuality::BadStatus);
    }

    #[test]
    fn test_pmu_interpolate_missing() {
        let cfg = PmuConfig::new_50hz("TEST", 1);
        // Two frames with a 3-period gap (should insert 2 interpolated frames).
        let f0 = PmuFrame {
            timestamp_us: 0,
            pmu_id: 1,
            stat: 0,
            phasors: vec![Phasor::new(1.0, 0.0)],
            freq_hz: 50.0,
            rocof: 0.0,
            analogs: vec![],
            digitals: vec![],
        };
        let f1 = PmuFrame {
            timestamp_us: 60_000, // 3 × 20_000 μs gap
            pmu_id: 1,
            stat: 0,
            phasors: vec![Phasor::new(1.0, 0.0)],
            freq_hz: 50.0,
            rocof: 0.0,
            analogs: vec![],
            digitals: vec![],
        };
        let mut dataset = PmuDataset::new();
        dataset.add_pmu(cfg, vec![f0, f1]);
        let n_interp = dataset.interpolate_missing();
        assert!(n_interp > 0, "Should have interpolated at least one frame");
    }

    #[test]
    fn test_pmu_synchronize() {
        // Two PMUs at 50 fps; synchronize to 25 fps.
        let cfg1 = PmuConfig::new_50hz("PMU1", 1);
        let mut cfg2 = PmuConfig::new_50hz("PMU2", 2);
        cfg2.pmu_id = 2;

        let make_frames = |pmu_id: u16, n: usize| {
            (0..n)
                .map(|k| PmuFrame {
                    timestamp_us: k as i64 * 20_000,
                    pmu_id,
                    stat: 0,
                    phasors: vec![Phasor::new(1.0, k as f64 * 0.01)],
                    freq_hz: 50.0,
                    rocof: 0.0,
                    analogs: vec![],
                    digitals: vec![],
                })
                .collect::<Vec<_>>()
        };

        let mut dataset = PmuDataset::new();
        dataset.add_pmu(cfg1, make_frames(1, 100));
        dataset.add_pmu(cfg2, make_frames(2, 100));

        dataset.synchronize(25.0).expect("Synchronize failed");

        assert_eq!(
            dataset.frames[0].len(),
            dataset.frames[1].len(),
            "Both PMUs should have same number of frames after sync"
        );
    }

    #[test]
    fn test_frequency_profile() {
        let cfg = PmuConfig::new_50hz("PMU1", 1);
        let frames: Vec<PmuFrame> = (0..5)
            .map(|k| PmuFrame {
                timestamp_us: k as i64 * 20_000,
                pmu_id: 1,
                stat: 0,
                phasors: vec![],
                freq_hz: 50.0 + k as f64 * 0.01,
                rocof: 0.0,
                analogs: vec![],
                digitals: vec![],
            })
            .collect();
        let mut dataset = PmuDataset::new();
        dataset.add_pmu(cfg, frames);
        let profile = dataset.frequency_profile();
        assert_eq!(profile.len(), 5);
        assert!((profile[2].1[0] - 50.02).abs() < 1e-10);
    }

    #[test]
    fn test_compute_tve_identical() {
        // TVE against identical reference should be zero.
        let cfg = PmuConfig::new_50hz("PMU1", 1);
        let frames: Vec<PmuFrame> = (0..3)
            .map(|k| PmuFrame {
                timestamp_us: k as i64 * 20_000,
                pmu_id: 1,
                stat: 0,
                phasors: vec![Phasor::new(1.0, 0.0)],
                freq_hz: 50.0,
                rocof: 0.0,
                analogs: vec![],
                digitals: vec![],
            })
            .collect();
        let mut ds = PmuDataset::new();
        ds.add_pmu(cfg.clone(), frames.clone());
        let mut ref_ds = PmuDataset::new();
        ref_ds.add_pmu(cfg, frames);

        let tve = ds.compute_tve(&ref_ds);
        for &v in &tve[0] {
            assert!(v < 1e-10, "TVE against self should be 0, got {:.3e}", v);
        }
    }
}
